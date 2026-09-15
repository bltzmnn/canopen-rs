//! SDO server: sans-I/O state machine servicing requests against an
//! [`ObjectDictionary`].
//!
//! The contract is `handle(od, request_frame) -> Vec<response_frame>`: feed it
//! each frame received on the request channel (`0x600 + node`) and transmit
//! every returned payload on the response channel (`0x580 + node`) in order.
//! The vector form exists because block transfer *streams*: one received
//! "start" or acknowledgement triggers a whole sub-block of outgoing segment
//! frames. No I/O, no clocks — timeouts are enforced by the host layer.
//!
//! Behavior notes:
//!
//! - One in-flight transfer per channel (CiA 301 behavior). A new initiate
//!   first aborts the active transfer with `GeneralError`, then starts the
//!   new one (docs/IMPLEMENTATION.md §5.5). Exception: while a **block
//!   download** is active every non-abort frame is routed as a segment
//!   (sub-segment sequence numbers are wire-ambiguous with initiates —
//!   CANopenNode routes the same way), so the stale-transfer abort applies
//!   at the four initiate entry points only.
//! - An incoming **abort** clears any active transfer and produces no reply.
//! - Access checks happen at *initiate* time: read of write-only aborts with
//!   [`SdoAbortCode::ReadOfWriteOnlyObject`], write of read-only with
//!   [`SdoAbortCode::WriteOfReadOnlyObject`].
//! - A missing address reports [`SdoAbortCode::SubindexDoesNotExist`] when the
//!   index exists but the subindex does not, otherwise
//!   [`SdoAbortCode::ObjectDoesNotExist`].
//! - While a block download is active, incoming frames are routed by phase:
//!   before the last segment is acknowledged, every non-abort frame is a
//!   sub-block segment (their byte 0 carries a raw sequence number whose top
//!   bits overlap other command specifiers — classification is state-driven);
//!   once `awaiting_end` is set, nothing but the end frame is accepted —
//!   anything else aborts with `InvalidCommandSpecifier`.
//! - A failed end-of-transfer CRC aborts with [`SdoAbortCode::CrcError`] and
//!   the value is NOT written to the OD.

use super::block::{
    clamp_blksize, crc16, decode_end, decode_sub_response, decode_upload_initiate,
    decode_upload_start, encode_download_initiate_response, encode_end, encode_end_response,
    encode_sub_response, encode_sub_segment, encode_upload_initiate_response, is_end_frame,
    is_sub_response, is_valid_blksize, BlockInitiate, BlockReceiver, CS_5, CS_6, MAX_BLKSIZE,
    SEGMENT_DATA_MAX as BLOCK_SEGMENT_MAX, SUBCMD_MASK,
};
use super::{
    decode_data_segment, decode_download_initiate_segmented, encode_abort, encode_data_segment,
    encode_download_response, encode_download_segment_response, encode_upload_expedited_response,
    encode_upload_initiate_segmented_response, SdoPayload, CCS_DOWNLOAD_INITIATE,
    CCS_DOWNLOAD_SEGMENT, CCS_UPLOAD_INITIATE, CCS_UPLOAD_SEGMENT, CS_ABORT, EXPEDITED,
    SEGMENT_DATA_MAX, SIZE_INDICATED, TOGGLE,
};
use crate::datatypes::{DataType, Value};
use crate::object_dictionary::ObjectDictionary;
use crate::sdo::SdoAbortCode;
use crate::types::{Address, Error};

/// An active server-side transfer between initiate and completion.
#[derive(Debug)]
enum Transfer {
    /// Segmented download (client writes a large value).
    Download {
        addr: Address,
        dt: DataType,
        declared: Option<usize>,
        buf: Vec<u8>,
        expected_toggle: bool,
    },
    /// Segmented upload (client reads a large value).
    Upload {
        addr: Address,
        buf: Vec<u8>,
        pos: usize,
        expected_toggle: bool,
    },
    /// Block download (client writes a bulk value).
    BlockDownload {
        addr: Address,
        dt: DataType,
        declared: Option<usize>,
        receiver: BlockReceiver,
        blksize: u8,
        /// Sequence number expected for the next segment (`1..=blksize`).
        expected_seqno: u8,
        /// Segments already received in the current sub-block.
        received: u8,
        peer_crc: bool,
        /// True once a last-flagged segment has been acknowledged; only then
        /// is an end frame recognized. A last segment's wire form
        /// `0x80 | seqno` collides with the end frame `0xC1` for seqno in
        /// {65,69,…,93}, so earlier recognition would swallow valid data
        /// segments (BUGS.md P1-1). Mirrors CANopenNode's dedicated
        /// `DOWNLOAD_BLK_END_REQ` phase (`CO_SDOserver.c:906`).
        awaiting_end: bool,
    },
    /// Block upload (server streams a bulk value).
    BlockUpload {
        addr: Address,
        buf: Vec<u8>,
        pos: usize,
        /// Position where the current sub-block started (for ACK rewinds).
        sub_base: usize,
        /// Sequence number for the next segment (1..=blksize while
        /// streaming; may briefly reach blksize+1 before the ACK arrives,
        /// when every slot of the window was just sent).
        seqno: u8,
        blksize: u8,
        peer_crc: bool,
    },
}

/// The SDO server state machine.
///
/// Construct one instance per SDO channel; call [`SdoServer::handle`] for
/// every received request frame.
#[derive(Debug, Default)]
pub struct SdoServer {
    transfer: Option<Transfer>,
    /// Monotonic-ms stamp of the last observed activity; refreshed by
    /// [`Self::touch`] from the driver loop so the §5.5 timeout is a true
    /// *inactivity* window rather than a total-session cap
    /// (`BUGS.md` R10-P1-4).
    last_activity_ms: Option<u64>,
    /// Addresses and values of successful OD writes since the last drain,
    /// consumed via [`SdoServer::take_last_written`] (used by `Node` for live
    /// PDO reconfiguration and storage magic words). Kept in arrival order;
    /// bounded by one entry per completed write transaction.
    last_written: Vec<(Address, Value)>,
    /// Most recent abort received from a client (CiA 309-style visibility;
    /// pyport test_local:118).
    last_received_abort: Option<SdoAbortCode>,
}

impl SdoServer {
    /// Creates an idle server.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            transfer: None,
            last_activity_ms: None,
            last_written: Vec::new(),
            last_received_abort: None,
        }
    }

    /// Whether a multi-frame transfer is currently in flight.
    #[must_use]
    pub const fn is_busy(&self) -> bool {
        self.transfer.is_some()
    }

    /// Clears any active transfer (e.g. on NMT reset).
    pub fn reset(&mut self) {
        self.transfer = None;
        self.last_written.clear();
        self.last_activity_ms = None;
    }

    /// Session-timeout maintenance (§5.5, `BUGS.md` P2-8): call from the
    /// driver loop with the current monotonic ms. When a transfer has been
    /// in flight for longer than `timeout_ms` without any frame arriving on
    /// the channel, it is torn down exactly like a received abort —
    /// releasing the server for the next transaction instead of leaving it
    /// wedged after a client vanished mid-transfer. The returned abort frame
    /// (`0x05040000`, naming the in-flight object) must be transmitted,
    /// matching CANopenNode `CO_SDOserver.c:1076-1079,1496-1509`
    /// (`BUGS.md` C1). `None` means no timeout fired.
    pub fn on_timeout(&mut self, now_ms: u64, timeout_ms: u64) -> Option<SdoPayload> {
        if !self.is_busy() {
            self.last_activity_ms = None;
            return None;
        }
        match self.last_activity_ms {
            Some(start) => {
                if now_ms.saturating_sub(start) >= timeout_ms {
                    let addr = self.transfer.as_ref().map(transfer_addr);
                    self.transfer = None;
                    self.last_activity_ms = None;
                    addr.map(|a| encode_abort(a, SdoAbortCode::ProtocolTimeout))
                } else {
                    None
                }
            }
            // Arm the window at first observation.
            None => {
                self.last_activity_ms = Some(now_ms);
                None
            }
        }
    }

    /// Records activity at `now_ms`, sliding the §5.5 inactivity window
    /// forward. Call from the driver loop whenever any frame was handled —
    /// previously nothing refreshed the stamp, so long-running transfers
    /// were torn down after a fixed total duration instead of actual silence
    /// (`BUGS.md` R10-P1-4).
    pub fn touch(&mut self, now_ms: u64) {
        if self.is_busy() {
            self.last_activity_ms = Some(now_ms);
        }
    }

    /// Returns and clears all writes recorded since the previous drain.
    pub fn take_last_written(&mut self) -> Vec<(Address, Value)> {
        std::mem::take(&mut self.last_written)
    }

    /// Returns and clears the abort code of the most recent client abort.
    pub fn take_last_received_abort(&mut self) -> Option<SdoAbortCode> {
        self.last_received_abort.take()
    }

    /// Aborts an active transfer with `GeneralError` (§5.5 / `BUGS.md` P1-3)
    /// and returns the ABORT frame to transmit before the new transfer's own
    /// responses. Empty when the channel was idle.
    fn abort_stale_transfer(&mut self) -> Vec<SdoPayload> {
        match self.transfer.take() {
            Some(t) => vec![encode_abort(transfer_addr(&t), SdoAbortCode::GeneralError)],
            None => Vec::new(),
        }
    }

    /// Handles one request frame, returning the response payloads to transmit
    /// in order. Often empty or a single frame; a block-upload start or
    /// acknowledgement yields a whole sub-block at once.
    pub fn handle(&mut self, od: &mut ObjectDictionary, req: &SdoPayload) -> Vec<SdoPayload> {
        // Active block download captures almost every frame shape (segments
        // carry raw sequence numbers) — route by state first. Only the exact
        // abort command byte (0x80, lower bits clear) is an abort: a
        // last-segment frame is always >= 0x81 since its sequence number >= 1.
        if matches!(self.transfer, Some(Transfer::BlockDownload { .. })) {
            if req[0] == CS_ABORT {
                self.transfer = None;
                return Vec::new();
            }
            // Phase separation (BUGS.md P1-1): the end frame is only
            // recognized after a last-flagged segment has been acked. A last
            // segment's wire form `0x80 | seqno` collides with the end frame
            // `0xC1` for seqno in {65,69,…,93}, so an unconditional end check
            // here would swallow valid data segments (and, without CRC
            // negotiation, silently truncate the value).
            let awaiting_end = matches!(
                self.transfer,
                Some(Transfer::BlockDownload {
                    awaiting_end: true,
                    ..
                })
            );
            if awaiting_end {
                if is_end_frame(req) {
                    return self.block_download_end(od, req);
                }
                // CANopenNode's END phase likewise accepts nothing but the
                // end frame; anything else tears the transfer down with an
                // explicit abort instead of being digested as a segment.
                // The abort names the in-flight object, not the frame's data
                // bytes (BUGS.md B1).
                let abort_addr = self
                    .transfer
                    .as_ref()
                    .map_or_else(|| echoed_addr(req), transfer_addr);
                self.transfer = None;
                return vec![encode_abort(
                    abort_addr,
                    SdoAbortCode::InvalidCommandSpecifier,
                )];
            }
            return self.block_download_segment(req);
        }

        let addr = Address::new(u16::from_le_bytes([req[1], req[2]]), req[3]);
        // §5.5 (BUGS.md P1-3): a fresh initiate must first abort the active
        // transfer with GeneralError instead of silently replacing it.
        let initiating = match req[0] & super::CS_MASK {
            CCS_DOWNLOAD_INITIATE | CCS_UPLOAD_INITIATE => true,
            // CS_5 family: sub-command bits 1..0, initiate is exactly 00.
            super::block::CS_5 => {
                req[0] & super::block::SUBCMD_MASK == super::block::SUBCMD_INITIATE
            }
            // CS_6 family (BUGS.md R10-P1-2): bit 1 carries the size flag on
            // initiates, so the initiate/end discrimination is bit 0 alone —
            // matching `decode_initiate_with_size`. The old full-mask check
            // classified a size-carrying 0xC6 as "not an initiate" and let it
            // silently replace an active transfer without the §5.5 abort.
            super::block::CS_6 => req[0] & super::block::SUBCMD_END == 0,
            _ => false,
        };
        let mut out = if initiating {
            self.abort_stale_transfer()
        } else {
            Vec::new()
        };
        out.extend(match req[0] & super::CS_MASK {
            CS_ABORT => {
                if let Ok((_, code)) = super::decode_abort(req) {
                    self.last_received_abort = Some(code);
                }
                self.transfer = None;
                Vec::new()
            }
            CCS_DOWNLOAD_INITIATE => self.download_initiate(od, req, addr),
            CCS_DOWNLOAD_SEGMENT => self.download_segment(od, req),
            CCS_UPLOAD_INITIATE => self.upload_initiate(od, addr),
            CCS_UPLOAD_SEGMENT => self.upload_segment(req),
            // Block download initiate (CS_6; initiate/end discriminated by
            // bit 0 alone — bit 1 is the size flag, BUGS.md R10-P1-2).
            CS_6 => match super::block::decode_download_initiate(req) {
                Ok(init) => self.block_download_initiate(od, init),
                Err(_) => {
                    // Undecodable CS_6: clear any in-flight transfer like
                    // every other abort path (BUGS.md #25).
                    self.transfer = None;
                    vec![encode_abort(addr, SdoAbortCode::InvalidCommandSpecifier)]
                }
            },
            // CS_5 family: block upload initiate / start / sub-response /
            // late end-response.
            CS_5 => self.on_cs5(od, req, addr),
            _ => {
                // Unknown command specifier: clear any in-flight transfer
                // (CANopenNode `CO_SDOserver.c` clears state on any illegal
                // frame, BUGS.md #25).
                self.transfer = None;
                vec![encode_abort(addr, SdoAbortCode::InvalidCommandSpecifier)]
            }
        });
        out
    }

    /// Frames arriving with command specifier `101b`: either a block upload
    /// initiation (idle) or continuation frames for an active block upload.
    fn on_cs5(
        &mut self,
        od: &mut ObjectDictionary,
        req: &SdoPayload,
        addr: Address,
    ) -> Vec<SdoPayload> {
        let subcmd = req[0] & SUBCMD_MASK;
        if let Some(Transfer::BlockUpload { .. }) = self.transfer {
            if decode_upload_start(req).is_ok() {
                return self.block_upload_stream();
            }
            if is_sub_response(req) {
                return self.block_upload_ack(req);
            }
            // A brand-new initiate while a block upload streams: §5.5 —
            // abort the active transfer first, then service the new one.
            if let Ok(neg) = decode_upload_initiate(req) {
                let mut out = self.abort_stale_transfer();
                out.extend(self.block_upload_initiate(od, neg));
                return out;
            }
            // Neither a start, sub-response nor initiate: abort, echoing the
            // address from the offending frame (the transfer address is
            // identical in every valid flow).
            self.transfer = None;
            return vec![encode_abort(addr, SdoAbortCode::InvalidCommandSpecifier)];
        }
        // Late end-response after our own end frame is a no-op — but only
        // while the channel is idle. During an in-flight segmented transfer
        // a CS=5 END frame is illegal and must tear the session down
        // (BUGS.md C2; 八M7). The END frame carries unused/CRC bytes in
        // positions 1-3, so the abort names the in-flight object, not the
        // request header.
        if subcmd == super::block::SUBCMD_END {
            if let Some(t) = self.transfer.take() {
                return vec![encode_abort(
                    transfer_addr(&t),
                    SdoAbortCode::InvalidCommandSpecifier,
                )];
            }
            return Vec::new();
        }
        match decode_upload_initiate(req) {
            Ok(neg) => self.block_upload_initiate(od, neg),
            Err(_) => {
                // BUGS.md 八M7: a CS=5 frame that is neither start,
                // sub-response nor a block-upload initiate is as illegal
                // for an in-flight *segmented* transfer as for the block
                // case above — tear the transfer down together with the
                // abort, otherwise the session would keep serving segment
                // polls after having declared the abort (zombie session).
                self.transfer = None;
                vec![encode_abort(addr, SdoAbortCode::InvalidCommandSpecifier)]
            }
        }
    }

    // --- Expedited / segmented -------------------------------------------------

    fn download_initiate(
        &mut self,
        od: &mut ObjectDictionary,
        req: &SdoPayload,
        addr: Address,
    ) -> Vec<SdoPayload> {
        let expedited = req[0] & EXPEDITED != 0;

        let Some(entry) = od.entry(addr) else {
            return vec![encode_abort(addr, missing_code(od, addr))];
        };
        let dt = entry.data_type;
        if !entry.access.is_writable() {
            return vec![encode_abort(addr, SdoAbortCode::WriteOfReadOnlyObject)];
        }

        if expedited {
            let len = if req[0] & SIZE_INDICATED != 0 {
                4 - usize::from((req[0] >> 2) & 0x03)
            } else {
                4
            };
            // Decode against the declared type so range/type validation runs.
            let value = match Value::decode_le(dt, &req[4..4 + len]) {
                Ok(v) => v,
                // Width mismatch on an expedited download reports the generic
                // type-mismatch code, matching python-canopen / CANopenNode
                // practice (docs/EXTENSIONS.md 批次 D 差异 D4). Declared-size
                // mismatches in segmented transfers keep the 12/13 codes.
                Err(Error::BadLength { .. }) => {
                    return vec![encode_abort(addr, SdoAbortCode::TypeMismatch)];
                }
                Err(e) => return vec![encode_abort(addr, abort_for(&e))],
            };
            if let Err(e) = od_write_guard(od, addr, &value) {
                return vec![encode_abort(addr, abort_for(&e))];
            }
            match od.write(addr, value.clone()) {
                Ok(()) => {
                    self.last_written.push((addr, value));
                    vec![encode_download_response(addr)]
                }
                Err(e) => vec![encode_abort(addr, abort_for(&e))],
            }
        } else {
            // CiA 301 §7.2.4.3.2: `e=0` starts a segmented transfer. When
            // `s=0` the total size is *not indicated* — a legal streaming
            // download (CANopenNode `CO_SDOserver.c:789-793`,
            // python-canopen `sdo/server.py:149-155`); `declared` stays
            // `None` and the transfer is bounded only by the absolute
            // `MAX_VALUE_LEN` limit. `decode_download_initiate_segmented`
            // only decodes the s=1 form, so the `.ok()` below yields
            // `declared=None` for the streaming initiate.
            let declared = decode_download_initiate_segmented(req)
                .map(|(_, size)| (size as usize).min(crate::datatypes::MAX_VALUE_LEN))
                .ok(); // peer-declared sizes capped by the absolute limit (R10-P2-1)
            self.transfer = Some(Transfer::Download {
                addr,
                dt,
                declared,
                buf: Vec::new(),
                expected_toggle: false,
            });
            vec![encode_download_response(addr)]
        }
    }

    #[allow(clippy::too_many_lines)]
    fn download_segment(&mut self, od: &mut ObjectDictionary, req: &SdoPayload) -> Vec<SdoPayload> {
        // Aborts on illegal frames must name the in-flight object, not the
        // frame's data bytes (BUGS.md B1); echo the header only while idle.
        let session_addr = self
            .transfer
            .as_ref()
            .map_or_else(|| echoed_addr(req), transfer_addr);
        enum Step {
            Ack(bool),
            Clear(SdoAbortCode),
            Finish,
        }
        // Decode *before* borrowing `self.transfer` so an undecodable
        // segment can clear the in-flight transfer (BUGS.md #25) without
        // fighting the `as_mut()` borrow below.
        let seg = match decode_data_segment(req) {
            Ok(s) => s,
            Err(_) => {
                self.transfer = None;
                return vec![encode_abort(
                    session_addr,
                    SdoAbortCode::InvalidCommandSpecifier,
                )];
            }
        };
        let (step, seg_toggle, abort_addr) = {
            let Some(Transfer::Download {
                addr,
                declared,
                buf,
                expected_toggle,
                ..
            }) = self.transfer.as_mut()
            else {
                // BUGS.md 十M7: a direction mismatch (download segment while
                // an upload streams) must tear the session down like every
                // other illegal frame (八M7), not leave a zombie that keeps
                // serving later segment polls.
                self.transfer = None;
                return vec![encode_abort(
                    session_addr,
                    SdoAbortCode::InvalidCommandSpecifier,
                )];
            };
            if seg.toggle != *expected_toggle {
                (
                    Step::Clear(SdoAbortCode::ToggleNotAlternated),
                    seg.toggle,
                    Some(*addr),
                )
            } else if !seg.last {
                // 越界校验：声明长度时按 `declared` 上限（且已填满即中止）；
                // 未声明时按全局 `MAX_VALUE_LEN` 防无界增长（BUGS.md P2-15）。
                // 两分支合并（BUGS.md P3-2）。
                let limit = declared.unwrap_or(crate::datatypes::MAX_VALUE_LEN);
                let exceeds = if let Some(max) = *declared {
                    buf.len() + seg.data.len() > max || buf.len() >= max
                } else {
                    buf.len() + seg.data.len() > limit
                };
                if exceeds {
                    (
                        Step::Clear(SdoAbortCode::TypeMismatchLengthHigh),
                        seg.toggle,
                        Some(*addr),
                    )
                } else {
                    buf.extend_from_slice(seg.data);
                    *expected_toggle = !*expected_toggle;
                    (Step::Ack(seg.toggle), seg.toggle, None)
                }
            } else {
                buf.extend_from_slice(seg.data);
                *expected_toggle = !*expected_toggle;
                (Step::Finish, seg.toggle, None)
            }
        };

        match step {
            Step::Ack(toggle) => vec![encode_download_segment_response(toggle)],
            Step::Clear(code) => {
                self.transfer = None;
                vec![encode_abort(abort_addr.unwrap_or(session_addr), code)]
            }
            Step::Finish => {
                let Transfer::Download {
                    addr,
                    dt,
                    declared,
                    buf,
                    ..
                } = self.transfer.take().expect("state verified above")
                else {
                    unreachable!("state verified above");
                };
                if let Some(expected) = declared {
                    if buf.len() < expected {
                        return vec![encode_abort(addr, SdoAbortCode::TypeMismatchLengthLow)];
                    }
                    if buf.len() > expected {
                        return vec![encode_abort(addr, SdoAbortCode::TypeMismatchLengthHigh)];
                    }
                }
                match Value::decode_le(dt, &buf) {
                    Ok(value) => {
                        if let Err(e) = od_write_guard(od, addr, &value) {
                            return vec![encode_abort(addr, abort_for(&e))];
                        }
                        match od.write(addr, value.clone()) {
                            Ok(()) => {
                                self.last_written.push((addr, value));
                                vec![encode_download_segment_response(seg_toggle)]
                            }
                            Err(e) => vec![encode_abort(addr, abort_for(&e))],
                        }
                    }
                    Err(e) => vec![encode_abort(addr, abort_for(&e))],
                }
            }
        }
    }

    fn upload_initiate(&mut self, od: &ObjectDictionary, addr: Address) -> Vec<SdoPayload> {
        let Some(entry) = od.entry(addr) else {
            return vec![encode_abort(addr, missing_code(od, addr))];
        };
        let dt = entry.data_type;
        if !entry.access.is_readable() {
            return vec![encode_abort(addr, SdoAbortCode::ReadOfWriteOnlyObject)];
        }
        let value = match od.read(addr) {
            Ok(v) => v,
            Err(e) => return vec![encode_abort(addr, abort_for(&e))],
        };
        let bytes = match value_to_bytes(&value, dt) {
            Ok(b) => b,
            Err(e) => return vec![encode_abort(addr, abort_for(&e))],
        };

        if (1..=4).contains(&bytes.len()) {
            match encode_upload_expedited_response(addr, &bytes) {
                Ok(p) => vec![p],
                Err(e) => vec![encode_abort(addr, abort_for(&e))],
            }
        } else {
            // Empty or >4-byte values go segmented (expedited cannot carry a
            // zero-length payload). The client may later switch to block
            // transfer via its own initiate.
            let size = u32::try_from(bytes.len()).unwrap_or(u32::MAX);
            self.transfer = Some(Transfer::Upload {
                addr,
                buf: bytes,
                pos: 0,
                expected_toggle: false,
            });
            vec![encode_upload_initiate_segmented_response(addr, size)]
        }
    }

    fn upload_segment(&mut self, req: &SdoPayload) -> Vec<SdoPayload> {
        let poll_toggle = req[0] & TOGGLE != 0;
        // A direction mismatch must still name the in-flight object
        // (BUGS.md B1); echo the header only while idle.
        let session_addr = self
            .transfer
            .as_ref()
            .map_or_else(|| echoed_addr(req), transfer_addr);
        let chunk_info = {
            let Some(Transfer::Upload {
                addr,
                buf,
                pos,
                expected_toggle,
                ..
            }) = self.transfer.as_mut()
            else {
                // BUGS.md 十M7: symmetric with `download_segment` — an
                // upload segment while a download streams aborts *and*
                // clears the session.
                self.transfer = None;
                return vec![encode_abort(
                    session_addr,
                    SdoAbortCode::InvalidCommandSpecifier,
                )];
            };
            if poll_toggle != *expected_toggle {
                let abort_addr = *addr;
                self.transfer = None;
                return vec![encode_abort(abort_addr, SdoAbortCode::ToggleNotAlternated)];
            }
            let remaining = buf.len() - *pos;
            let len = remaining.min(SEGMENT_DATA_MAX);
            let last = remaining <= SEGMENT_DATA_MAX;
            let start = *pos;
            *pos += len;
            *expected_toggle = !*expected_toggle;
            (start, len, last)
        };
        let (start, len, last) = chunk_info;
        let out = {
            let Some(Transfer::Upload { buf, addr, .. }) = self.transfer.as_ref() else {
                unreachable!("state verified above");
            };
            let addr = *addr;
            match encode_data_segment(&buf[start..start + len], poll_toggle, last) {
                Ok(p) => vec![p],
                // 回显原地址而非 (0,0)（BUGS.md P3-3；该路径当前不可达，仅代码卫生）。
                Err(e) => vec![encode_abort(addr, abort_for(&e))],
            }
        };
        if last {
            // CiA 301: transmitting the final segment (c=1) returns the
            // server to idle. Leaving the transfer armed left a zombie
            // session that poisoned the *next* transaction on this channel
            // with a stale-transfer GeneralError ABORT (BUGS.md P1-2).
            self.transfer = None;
        }
        out
    }

    // --- Block download (client writes a bulk value) ---------------------------

    fn block_download_initiate(
        &mut self,
        od: &mut ObjectDictionary,
        init: BlockInitiate,
    ) -> Vec<SdoPayload> {
        let addr = init.address;
        let Some(entry) = od.entry(addr) else {
            return vec![encode_abort(addr, missing_code(od, addr))];
        };
        let dt = entry.data_type;
        if !entry.access.is_writable() {
            return vec![encode_abort(addr, SdoAbortCode::WriteOfReadOnlyObject)];
        }

        // Adaptive blksize ceiling: the buffer is dynamically sized, so the
        // protocol maximum applies (docs/IMPLEMENTATION.md §5.3).
        // Cap a peer-declared size by the absolute value limit too: u32::MAX
        // as `declared` would otherwise defeat the MAX_VALUE_LEN guard that
        // undeclared transfers have (BUGS.md R10-P2-1).
        let declared = init
            .size
            .map(|s| (s as usize).min(crate::datatypes::MAX_VALUE_LEN));
        self.transfer = Some(Transfer::BlockDownload {
            addr,
            dt,
            declared,
            receiver: BlockReceiver::new(),
            blksize: MAX_BLKSIZE,
            expected_seqno: 1,
            received: 0,
            peer_crc: init.crc_support,
            awaiting_end: false,
        });
        vec![encode_download_initiate_response(
            addr,
            MAX_BLKSIZE,
            init.crc_support,
        )]
    }

    fn block_download_segment(&mut self, req: &SdoPayload) -> Vec<SdoPayload> {
        enum Step {
            Continue,
            Ack { ackseq: u8, blksize: u8 },
            Finish { ackseq: u8, blksize: u8 },
            Clear(SdoAbortCode),
        }
        // Block segments carry sequence numbers in byte 0 and payload in the
        // rest: an abort must name the in-flight object, not those bytes
        // (BUGS.md B1).
        let session_addr = self
            .transfer
            .as_ref()
            .map_or_else(|| echoed_addr(req), transfer_addr);
        let step = {
            let Some(Transfer::BlockDownload {
                declared,
                receiver,
                blksize,
                expected_seqno,
                received,
                awaiting_end,
                ..
            }) = self.transfer.as_mut()
            else {
                unreachable!("dispatch guarantees BlockDownload");
            };
            let seg = super::block::decode_sub_segment(req);
            if seg.seqno == 0 {
                Step::Clear(SdoAbortCode::InvalidSequenceNumber)
            } else if seg.seqno < *expected_seqno {
                // Duplicate segment (e.g. our ack was lost): ignore it —
                // CANopenNode `CO_SDOserver.c:88-101` skips repeats instead
                // of tearing the transfer down (BUGS.md P2-7).
                Step::Continue
            } else if seg.seqno > *expected_seqno {
                // BUGS.md #24: while awaiting the *renumbered*
                // retransmission (expected=1, received=0) any higher
                // sequence number is a stale straggler from the old window —
                // ignore it without acknowledging, otherwise the sender
                // rewinds again and re-sends data already buffered.
                if *expected_seqno == 1 && *received == 0 {
                    Step::Continue
                } else {
                    // Gap: end the sub-block early and ack what we have so
                    // the sender retransmits from there ("data after last
                    // good seqno will be re-transmitted"), matching our own
                    // upload client.
                    *expected_seqno = 1;
                    let ackseq = core::mem::replace(received, 0);
                    Step::Ack {
                        ackseq,
                        blksize: *blksize,
                    }
                }
            } else if receiver
                .push(&seg, (*declared).or(Some(crate::datatypes::MAX_VALUE_LEN)))
                .is_err()
            {
                Step::Clear(SdoAbortCode::TypeMismatchLengthHigh)
            } else {
                *received += 1;
                *expected_seqno += 1;
                let full = *received >= *blksize;
                if seg.last || full {
                    let ackseq = *received;
                    *expected_seqno = 1;
                    *received = 0;
                    if seg.last {
                        // Arm end-frame recognition only after this ack
                        // (BUGS.md P1-1).
                        *awaiting_end = true;
                        Step::Finish {
                            ackseq,
                            blksize: *blksize,
                        }
                    } else {
                        Step::Ack {
                            ackseq,
                            blksize: *blksize,
                        }
                    }
                } else {
                    Step::Continue
                }
            }
        };

        match step {
            Step::Continue => Vec::new(),
            Step::Ack { ackseq, blksize } => vec![encode_sub_response(ackseq, blksize)],
            Step::Finish { ackseq, blksize } => {
                // Data complete; ack and wait for the end frame.
                vec![encode_sub_response(ackseq, blksize)]
            }
            Step::Clear(code) => {
                self.transfer = None;
                vec![encode_abort(session_addr, code)]
            }
        }
    }

    fn block_download_end(
        &mut self,
        od: &mut ObjectDictionary,
        req: &SdoPayload,
    ) -> Vec<SdoPayload> {
        let Some(Transfer::BlockDownload {
            addr,
            dt,
            declared,
            mut receiver,
            peer_crc,
            ..
        }) = self.transfer.take()
        else {
            unreachable!("dispatch guarantees BlockDownload");
        };
        let (unused, crc) = match decode_end(req) {
            Ok(v) => v,
            Err(_) => {
                return vec![encode_abort(addr, SdoAbortCode::InvalidCommandSpecifier)];
            }
        };
        // Trim to the declared length (or the unused-byte count), then check CRC.
        if let Err(e) = receiver.finish(declared, unused, crc, peer_crc) {
            // BUGS.md 十L1: `Error::abort_code` distinguishes an over-long
            // payload (0x0607_0012) from a short one (0x0607_0013);
            // CANopenNode `CO_SDOserver.c:403-404` does the same.
            return vec![encode_abort(addr, abort_for(&e))];
        }
        let bytes = receiver.into_inner();
        match Value::decode_le(dt, &bytes) {
            Ok(value) => {
                if let Err(e) = od_write_guard(od, addr, &value) {
                    return vec![encode_abort(addr, abort_for(&e))];
                }
                match od.write(addr, value.clone()) {
                    Ok(()) => {
                        self.last_written.push((addr, value));
                        vec![encode_end_response()]
                    }
                    Err(e) => vec![encode_abort(addr, abort_for(&e))],
                }
            }
            Err(e) => vec![encode_abort(addr, abort_for(&e))],
        }
    }

    // --- Block upload (server streams a bulk value) ----------------------------

    fn block_upload_initiate(
        &mut self,
        od: &ObjectDictionary,
        neg: super::block::BlockNegotiation,
    ) -> Vec<SdoPayload> {
        let addr = neg.address;
        // CiA 301: an out-of-range blksize must be rejected explicitly, not
        // silently clamped (BUGS.md P3-26).
        if !is_valid_blksize(neg.blksize) {
            return vec![encode_abort(addr, SdoAbortCode::InvalidBlockSize)];
        }
        let Some(entry) = od.entry(addr) else {
            return vec![encode_abort(addr, missing_code(od, addr))];
        };
        let dt = entry.data_type;
        if !entry.access.is_readable() {
            return vec![encode_abort(addr, SdoAbortCode::ReadOfWriteOnlyObject)];
        }
        let value = match od.read(addr) {
            Ok(v) => v,
            Err(e) => return vec![encode_abort(addr, abort_for(&e))],
        };
        let bytes = match value_to_bytes(&value, dt) {
            Ok(b) => b,
            Err(e) => {
                return vec![encode_abort(addr, abort_for(&e))];
            }
        };
        let size = u32::try_from(bytes.len()).unwrap_or(u32::MAX);
        // Zero-length object: a block stream would have to consist of the
        // end frame alone, which the phase-separated receiver cannot
        // distinguish from a colliding segment. Fall back to segmented
        // semantics instead (the server-side mirror of the pst switch,
        // BUGS.md P3-29).
        if bytes.is_empty() {
            self.transfer = Some(Transfer::Upload {
                addr,
                buf: bytes,
                pos: 0,
                expected_toggle: false,
            });
            return vec![super::encode_upload_initiate_segmented_response(addr, 0)];
        }
        // Protocol switch threshold (BUGS.md P3-28): a client whose pst is
        // not smaller than the object size prefers segmented transfer —
        // answer with a plain upload-initiate response so it falls back.
        if let Some(pst) = neg.pst {
            if u32::from(pst) >= size && size > 4 {
                self.transfer = Some(Transfer::Upload {
                    addr,
                    buf: bytes,
                    pos: 0,
                    expected_toggle: false,
                });
                return vec![super::encode_upload_initiate_segmented_response(addr, size)];
            }
        }

        self.transfer = Some(Transfer::BlockUpload {
            addr,
            buf: bytes,
            pos: 0,
            sub_base: 0,
            seqno: 1,
            blksize: clamp_blksize(neg.blksize),
            peer_crc: neg.crc_support,
        });
        vec![encode_upload_initiate_response(
            addr,
            Some(size),
            neg.crc_support,
        )]
    }

    /// Emits up to `blksize` segments of the active block upload.
    fn block_upload_stream(&mut self) -> Vec<SdoPayload> {
        let mut out = Vec::new();
        // 零长度对象已在 `block_upload_initiate` 走分段路径（BUGS.md P3-29），
        // 因此此处 `BlockUpload` 的 `buf` 必非空，无需处理空长度分支。
        let Some(Transfer::BlockUpload {
            buf,
            pos,
            seqno,
            blksize,
            ..
        }) = self.transfer.as_mut()
        else {
            return out;
        };
        while *pos < buf.len() && *seqno <= *blksize {
            let take = (buf.len() - *pos).min(BLOCK_SEGMENT_MAX);
            let last = *pos + take >= buf.len();
            out.push(
                encode_sub_segment(*seqno, &buf[*pos..*pos + take], last)
                    .expect("seqno and length bounded"),
            );
            *pos += take;
            *seqno += 1;
            if last {
                break;
            }
        }
        out
    }

    fn block_upload_ack(&mut self, req: &SdoPayload) -> Vec<SdoPayload> {
        let (ackseq, next_blksize) = match decode_sub_response(req) {
            Ok(v) => v,
            Err(_) => {
                // Name the in-flight object, not the malformed ACK bytes
                // (BUGS.md B1).
                let abort_addr = self
                    .transfer
                    .as_ref()
                    .map_or_else(|| echoed_addr(req), transfer_addr);
                self.transfer = None;
                return vec![encode_abort(
                    abort_addr,
                    SdoAbortCode::InvalidCommandSpecifier,
                )];
            }
        };

        enum Step {
            Rewind { from: usize },
            Next,
            DoneEnd,
            Clear(SdoAbortCode),
        }
        let step = {
            let Some(Transfer::BlockUpload {
                buf,
                pos,
                sub_base,
                seqno,
                blksize,
                ..
            }) = self.transfer.as_mut()
            else {
                unreachable!("dispatch guarantees BlockUpload");
            };
            let sent_this_sub = seqno.wrapping_sub(1);
            if ackseq > sent_this_sub {
                // Client acknowledging more than we sent: protocol violation
                // — previously we silently advanced and could emit the end
                // frame early (BUGS.md R10-P2-6).
                Step::Clear(SdoAbortCode::InvalidCommandSpecifier)
            } else if ackseq < sent_this_sub {
                // Partial acknowledgement: retransmit the unacknowledged
                // tail with sequence numbering restarted at 1 (CiA 301 /
                // python-canopen semantics). The peer's new blksize proposal
                // is adopted right away, not one round late (BUGS.md P3-27),
                // and validated exactly like the Next branch (BUGS.md
                // R10-P2-5).
                if !is_valid_blksize(next_blksize) {
                    Step::Clear(SdoAbortCode::InvalidBlockSize)
                } else {
                    *blksize = next_blksize;
                    Step::Rewind {
                        from: (*sub_base + usize::from(ackseq) * BLOCK_SEGMENT_MAX).min(buf.len()),
                    }
                }
            } else if *pos >= buf.len() {
                Step::DoneEnd
            } else {
                // A changed blksize must still be in range (BUGS.md P3-26).
                if !is_valid_blksize(next_blksize) {
                    Step::Clear(SdoAbortCode::InvalidBlockSize)
                } else {
                    *sub_base = *pos;
                    *seqno = 1;
                    *blksize = next_blksize;
                    Step::Next
                }
            }
        };

        match step {
            Step::Rewind { from } => {
                if let Some(Transfer::BlockUpload {
                    buf,
                    pos,
                    sub_base,
                    seqno: st_seqno,
                    ..
                }) = self.transfer.as_mut()
                {
                    let from = from.min(buf.len());
                    *pos = from;
                    // The rewound window *is* the new sub-block base: without
                    // this, a second partial ack inside the same logical
                    // sub-block computes its rewind offset from the ORIGINAL
                    // base and re-sends already-acked data (BUGS.md
                    // R10-P1-1). Mirrors the client's writer.position()
                    // bookkeeping.
                    *sub_base = from;
                    *st_seqno = 1;
                }
                self.block_upload_stream()
            }
            Step::Next => self.block_upload_stream(),
            Step::Clear(code) => {
                let abort_addr = self
                    .transfer
                    .as_ref()
                    .map_or_else(|| echoed_addr(req), transfer_addr);
                self.transfer = None;
                vec![encode_abort(abort_addr, code)]
            }
            Step::DoneEnd => {
                // BUGS.md L5: this state is only reachable while a block
                // upload is in flight (the dispatch above enforces it), so
                // the transfer must be present — but keep the message for
                // diagnostics instead of a bare panic.
                let Some(Transfer::BlockUpload { buf, peer_crc, .. }) = self.transfer.take() else {
                    unreachable!("DoneEnd requires an active BlockUpload");
                };
                let last_len = match buf.len() % BLOCK_SEGMENT_MAX {
                    0 if !buf.is_empty() => BLOCK_SEGMENT_MAX,
                    r => r,
                };
                let unused = (BLOCK_SEGMENT_MAX - last_len) as u8;
                let crc = crc16(&buf);
                let _ = peer_crc; // CRC always computed; client verifies when negotiated
                vec![encode_end(unused, crc)]
            }
        }
    }
}

/// Distinguishes "whole index missing" from "subindex missing".
fn missing_code(od: &ObjectDictionary, addr: Address) -> SdoAbortCode {
    if addr.subindex != 0 && od.entry(Address::new(addr.index, 0)).is_some() {
        SdoAbortCode::SubindexDoesNotExist
    } else {
        SdoAbortCode::ObjectDoesNotExist
    }
}

/// Pre-write protocol guards applied to every OD write reaching the server
/// (BUGS.md P2-5 / P3-39):
///
/// - PDO communication-record COB-ID rules (bit layout and the CAN-ID-0 NMT
///   channel) — abort `INVALID_VALUE`-class codes. Restricted CAN-IDs and
///   the enable-with-empty-mapping gate are deliberately *not* applied here
///   (BUGS.md 八M2: the live re-assembly architecture matches
///   python-canopen; the empty-mapping case surfaces through the
///   config-error ring);
/// - `0x1014` EMCY COB-ID rules, mirroring CANopenNode `OD_write_1014`
///   (`CO_Emergency.c:72-102`): reserved bits 30..11, restricted CAN-IDs
///   and a COB-ID change while production is enabled are rejected with
///   `InvalidValue` (0x0609_0030). A low-11-bit value of 0 disables
///   production (handled by `Node::emcy_cob_id`);
/// - `0x1003:0` must be written as zero (non-zero is rejected with
///   `VALUE_TOO_HIGH` like CANopenNode's `OD_write_1003`) instead of being
///   silently rolled back after the fact;
/// - `0x1010`/`0x1011` only accept their magic words ('save'/'load');
///   anything else is `DataCannotBeTransferred` like CANopenNode
///   `CO_storage.c:42-45,96-99` (write of 0x1010) — previously every value
///   was confirmed and silently ignored.
fn od_write_guard(od: &ObjectDictionary, addr: Address, value: &Value) -> crate::types::Result<()> {
    crate::pdo::guard_comm_record_write(addr, value)?;
    if addr == Address::new(0x1003, 0) {
        if let Value::Uint(n) = value {
            if *n != 0 {
                return Err(Error::ValueOutOfRange {
                    value: *n as i64,
                    min: 0,
                    max: 0,
                });
            }
        }
    }
    if addr == Address::new(0x1019, 0) {
        if let Value::Uint(v) = value {
            // CANopenNode `OD_write_1019` (`CO_SYNC.c:151-153`): value 1 is
            // reserved and >240 exceeds the counter range — reject with
            // `ODR_INVALID_VALUE` (0x0609_0030), not a range code
            // (BUGS.md 八L14 / 十L2).
            if *v == 1 || *v > 240 {
                return Err(Error::InvalidValue);
            }
        }
    }
    // 0x1005 SYNC COB-ID write guard (BUGS.md 十M4): CANopenNode
    // `OD_write_1005` (`CO_SYNC.c:87-95`) rejects bit31, the reserved bits
    // 29..11 and restricted CAN-IDs; bit30 is the gen/producer flag.
    // Without this, an illegal value was stored and silently blinded the
    // node to every SYNC (no abort, no config-error entry).
    if addr == Address::new(0x1005, 0) {
        if let Value::Uint(v) = value {
            let raw = *v as u32;
            if raw & 0xBFFF_F800 != 0 || crate::pdo::is_restricted_can_id((raw & 0x7FF) as u16) {
                return Err(Error::InvalidValue);
            }
        }
    }
    // 0x1014 EMCY COB-ID write guard (CANopenNode `OD_write_1014`,
    // `CO_Emergency.c:72-102`): bit31 (disabled) is allowed, but reserved
    // bits 30..11, restricted CAN-IDs and a COB-ID change while production
    // is already enabled are rejected. Without this, an illegal value was
    // stored and silently disabled/redirected EMCY production.
    if addr == Address::new(0x1014, 0) {
        if let Value::Uint(v) = value {
            let raw = *v as u32;
            let new_can = (raw & 0x7FF) as u16;
            let new_enabled = raw & (1 << 31) == 0 && new_can != 0;
            let (cur_enabled, cur_can) = match od.read(Address::new(0x1014, 0)) {
                Ok(Value::Uint(cur)) => {
                    let cur = cur as u32;
                    let can = (cur & 0x7FF) as u16;
                    (cur & (1 << 31) == 0 && can != 0, can)
                }
                _ => (false, 0),
            };
            if raw & 0x7FFF_F800 != 0
                || crate::pdo::is_restricted_can_id(new_can)
                || (cur_enabled && new_enabled && new_can != cur_can)
            {
                return Err(Error::InvalidValue);
            }
        }
    }
    // 0x1010 Save / 0x1011 Restore only accept their magic words; a
    // non-magic write is `ODR_DATA_TRANSF` in CANopenNode
    // (`CO_storage.c:42-45,96-99`), mapped to
    // `SdoAbortCode::DataCannotBeTransferred` (0x0800_0020).
    if addr.subindex >= 1 && matches!(addr.index, 0x1010 | 0x1011) {
        if let Value::Uint(v) = value {
            let expected = if addr.index == 0x1010 {
                crate::storage::SAVE_MAGIC_U32
            } else {
                crate::storage::LOAD_MAGIC_U32
            };
            if *v as u32 != expected {
                return Err(Error::DataCannotBeTransferred);
            }
        }
    }
    Ok(())
}

/// The object address of an in-flight transfer (for stale-transfer aborts).
fn transfer_addr(t: &Transfer) -> Address {
    match t {
        Transfer::Download { addr, .. }
        | Transfer::Upload { addr, .. }
        | Transfer::BlockDownload { addr, .. }
        | Transfer::BlockUpload { addr, .. } => *addr,
    }
}

/// Recovers the address echoed in a frame's header bytes (index LE + sub).
fn echoed_addr(req: &SdoPayload) -> Address {
    Address::new(u16::from_le_bytes([req[1], req[2]]), req[3])
}

/// Maps an [`Error`] onto its abort code, defaulting to general error.
fn abort_for(err: &Error) -> SdoAbortCode {
    err.abort_code().unwrap_or(SdoAbortCode::GeneralError)
}

/// Converts a [`Value`] into wire bytes under the entry's declared type.
pub(crate) fn value_to_bytes(value: &Value, dt: DataType) -> Result<Vec<u8>, Error> {
    match dt.fixed_size() {
        Some(n) => {
            let mut b = vec![0u8; n];
            value.encode_as(dt, &mut b)?;
            Ok(b)
        }
        None => {
            // `encoded_len_as` accounts for target types whose wire form is
            // wider than the value's minimal one (UNICODE_STRING UTF-16LE,
            // BUGS.md L7).
            let mut b = vec![0u8; value.encoded_len_as(dt)];
            let n = value.encode_as(dt, &mut b)?;
            b.truncate(n);
            Ok(b)
        }
    }
}
