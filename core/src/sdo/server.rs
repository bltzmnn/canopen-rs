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
//!   implicitly replaces an active transfer.
//! - An incoming **abort** clears any active transfer and produces no reply.
//! - Access checks happen at *initiate* time: read of write-only aborts with
//!   [`SdoAbortCode::ReadOfWriteOnlyObject`], write of read-only with
//!   [`SdoAbortCode::WriteOfReadOnlyObject`].
//! - A missing address reports [`SdoAbortCode::SubindexDoesNotExist`] when the
//!   index exists but the subindex does not, otherwise
//!   [`SdoAbortCode::ObjectDoesNotExist`].
//! - While a block download is active, every non-end frame is treated as a
//!   sub-block segment (their byte 0 carries a raw sequence number whose top
//!   bits overlap other command specifiers — classification is state-driven).
//! - A failed end-of-transfer CRC aborts with [`SdoAbortCode::CrcError`] and
//!   the value is NOT written to the OD.

use super::block::{
    clamp_blksize, crc16, decode_end, decode_sub_response, decode_upload_initiate,
    decode_upload_start, encode_download_initiate_response, encode_end, encode_end_response,
    encode_sub_response, encode_sub_segment, encode_upload_initiate_response, is_end_frame,
    is_sub_response, BlockInitiate, BlockReceiver, CS_5, CS_6, MAX_BLKSIZE,
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
    },
    /// Block upload (server streams a bulk value).
    BlockUpload {
        addr: Address,
        buf: Vec<u8>,
        pos: usize,
        /// Position where the current sub-block started (for ACK rewinds).
        sub_base: usize,
        /// Sequence number for the next segment (`1..=blksize`).
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
    /// Address and value of the most recent successful OD write, consumed
    /// via [`SdoServer::take_last_written`] (used by `Node` for live PDO
    /// reconfiguration and storage magic words).
    last_written: Option<(Address, Value)>,
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
            last_written: None,
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
        self.last_written = None;
    }

    /// Returns and clears the address/value pair of the most recent
    /// successful write.
    pub fn take_last_written(&mut self) -> Option<(Address, Value)> {
        self.last_written.take()
    }

    /// Returns and clears the abort code of the most recent client abort.
    pub fn take_last_received_abort(&mut self) -> Option<SdoAbortCode> {
        self.last_received_abort.take()
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
            if is_end_frame(req) {
                return self.block_download_end(od, req);
            }
            return self.block_download_segment(req);
        }

        let addr = Address::new(u16::from_le_bytes([req[1], req[2]]), req[3]);
        match req[0] & super::CS_MASK {
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
            // Block download initiate (CS_6 | sub-command 00).
            CS_6 => match super::block::decode_download_initiate(req) {
                Ok(init) => self.block_download_initiate(od, init),
                Err(_) => vec![encode_abort(addr, SdoAbortCode::InvalidCommandSpecifier)],
            },
            // CS_5 family: block upload initiate / start / sub-response /
            // late end-response.
            CS_5 => self.on_cs5(od, req, addr),
            _ => vec![encode_abort(addr, SdoAbortCode::InvalidCommandSpecifier)],
        }
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
            let upload_addr = if let Some(Transfer::BlockUpload { addr, .. }) = &self.transfer {
                *addr
            } else {
                addr
            };
            self.transfer = None;
            return vec![encode_abort(
                upload_addr,
                SdoAbortCode::InvalidCommandSpecifier,
            )];
        }
        // Late end-response after our end frame (transfer already cleared).
        if subcmd == super::block::SUBCMD_END {
            return Vec::new();
        }
        match decode_upload_initiate(req) {
            Ok(neg) => self.block_upload_initiate(od, neg),
            Err(_) => vec![encode_abort(addr, SdoAbortCode::InvalidCommandSpecifier)],
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
            match od.write(addr, value.clone()) {
                Ok(()) => {
                    self.last_written = Some((addr, value));
                    vec![encode_download_response(addr)]
                }
                Err(e) => vec![encode_abort(addr, abort_for(&e))],
            }
        } else {
            let declared = decode_download_initiate_segmented(req)
                .map(|(_, size)| size as usize)
                .ok();
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
        let echoed = Address::new(u16::from_le_bytes([req[1], req[2]]), req[3]);
        enum Step {
            Ack(bool),
            Clear(SdoAbortCode),
            Finish,
        }
        let (step, seg_toggle, abort_addr) = {
            let Some(Transfer::Download {
                addr,
                declared,
                buf,
                expected_toggle,
                ..
            }) = self.transfer.as_mut()
            else {
                return vec![encode_abort(echoed, SdoAbortCode::InvalidCommandSpecifier)];
            };
            let seg = match decode_data_segment(req) {
                Ok(s) => s,
                Err(_) => {
                    return vec![encode_abort(*addr, SdoAbortCode::InvalidCommandSpecifier)];
                }
            };
            if seg.toggle != *expected_toggle {
                (
                    Step::Clear(SdoAbortCode::ToggleNotAlternated),
                    seg.toggle,
                    Some(*addr),
                )
            } else if let Some(max) = *declared {
                if !seg.last && (buf.len() + seg.data.len() > max || buf.len() >= max) {
                    (
                        Step::Clear(SdoAbortCode::TypeMismatchLengthHigh),
                        seg.toggle,
                        Some(*addr),
                    )
                } else {
                    buf.extend_from_slice(seg.data);
                    *expected_toggle = !*expected_toggle;
                    if seg.last {
                        (Step::Finish, seg.toggle, None)
                    } else {
                        (Step::Ack(seg.toggle), seg.toggle, None)
                    }
                }
            } else {
                buf.extend_from_slice(seg.data);
                *expected_toggle = !*expected_toggle;
                if seg.last {
                    (Step::Finish, seg.toggle, None)
                } else {
                    (Step::Ack(seg.toggle), seg.toggle, None)
                }
            }
        };

        match step {
            Step::Ack(toggle) => vec![encode_download_segment_response(toggle)],
            Step::Clear(code) => {
                self.transfer = None;
                vec![encode_abort(abort_addr.unwrap_or(echoed), code)]
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
                    Ok(value) => match od.write(addr, value.clone()) {
                        Ok(()) => {
                            self.last_written = Some((addr, value));
                            vec![encode_download_segment_response(seg_toggle)]
                        }
                        Err(e) => vec![encode_abort(addr, abort_for(&e))],
                    },
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
                buf: bytes,
                pos: 0,
                expected_toggle: false,
            });
            vec![encode_upload_initiate_segmented_response(addr, size)]
        }
    }

    fn upload_segment(&mut self, req: &SdoPayload) -> Vec<SdoPayload> {
        let poll_toggle = req[0] & TOGGLE != 0;
        let chunk_info = {
            let Some(Transfer::Upload {
                buf,
                pos,
                expected_toggle,
            }) = self.transfer.as_mut()
            else {
                return vec![encode_abort(
                    echoed_addr(req),
                    SdoAbortCode::InvalidCommandSpecifier,
                )];
            };
            if poll_toggle != *expected_toggle {
                self.transfer = None;
                return vec![encode_abort(
                    echoed_addr(req),
                    SdoAbortCode::ToggleNotAlternated,
                )];
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
        let Transfer::Upload { buf, .. } = self.transfer.as_ref().unwrap() else {
            unreachable!("state verified above");
        };
        match encode_data_segment(&buf[start..start + len], poll_toggle, last) {
            Ok(p) => vec![p],
            Err(e) => vec![encode_abort(Address::new(0, 0), abort_for(&e))],
        }
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
        self.transfer = Some(Transfer::BlockDownload {
            addr,
            dt,
            declared: init.size.map(|s| s as usize),
            receiver: BlockReceiver::new(),
            blksize: MAX_BLKSIZE,
            expected_seqno: 1,
            received: 0,
            peer_crc: init.crc_support,
        });
        vec![encode_download_initiate_response(addr, MAX_BLKSIZE, true)]
    }

    fn block_download_segment(&mut self, req: &SdoPayload) -> Vec<SdoPayload> {
        enum Step {
            Continue,
            Ack { ackseq: u8, blksize: u8 },
            Finish { ackseq: u8, blksize: u8 },
            Clear(SdoAbortCode),
        }
        let echoed = echoed_addr(req);
        let step = {
            let Some(Transfer::BlockDownload {
                declared,
                receiver,
                blksize,
                expected_seqno,
                received,
                ..
            }) = self.transfer.as_mut()
            else {
                unreachable!("dispatch guarantees BlockDownload");
            };
            let seg = super::block::decode_sub_segment(req);
            if seg.seqno == 0 || seg.seqno != *expected_seqno {
                Step::Clear(SdoAbortCode::InvalidSequenceNumber)
            } else if receiver.push(&seg, *declared).is_err() {
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
                vec![encode_abort(echoed, code)]
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
            let code = match e {
                Error::CrcMismatch => SdoAbortCode::CrcError,
                _ => SdoAbortCode::TypeMismatchLengthLow,
            };
            return vec![encode_abort(addr, code)];
        }
        let bytes = receiver.into_inner();
        match Value::decode_le(dt, &bytes) {
            Ok(value) => match od.write(addr, value.clone()) {
                Ok(()) => {
                    self.last_written = Some((addr, value));
                    vec![encode_end_response()]
                }
                Err(e) => vec![encode_abort(addr, abort_for(&e))],
            },
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
        let size = u32::try_from(bytes.len()).unwrap_or(u32::MAX);

        self.transfer = Some(Transfer::BlockUpload {
            addr,
            buf: bytes,
            pos: 0,
            sub_base: 0,
            seqno: 1,
            blksize: clamp_blksize(neg.blksize),
            peer_crc: neg.crc_support,
        });
        vec![encode_upload_initiate_response(addr, Some(size), true)]
    }

    /// Emits up to `blksize` segments of the active block upload.
    fn block_upload_stream(&mut self) -> Vec<SdoPayload> {
        let mut out = Vec::new();
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
                let echoed = echoed_addr(req);
                self.transfer = None;
                return vec![encode_abort(echoed, SdoAbortCode::InvalidCommandSpecifier)];
            }
        };

        enum Step {
            Rewind { from: usize },
            Next,
            DoneEnd,
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
            if ackseq < sent_this_sub {
                // Partial acknowledgement: retransmit the unacknowledged
                // tail with sequence numbering restarted at 1 (CiA 301 /
                // python-canopen semantics).
                Step::Rewind {
                    from: (*sub_base + usize::from(ackseq) * BLOCK_SEGMENT_MAX).min(buf.len()),
                }
            } else if *pos >= buf.len() {
                Step::DoneEnd
            } else {
                *sub_base = *pos;
                *seqno = 1;
                *blksize = clamp_blksize(next_blksize);
                Step::Next
            }
        };

        match step {
            Step::Rewind { from } => {
                if let Some(Transfer::BlockUpload {
                    buf,
                    pos,
                    seqno: st_seqno,
                    ..
                }) = self.transfer.as_mut()
                {
                    *pos = from.min(buf.len());
                    *st_seqno = 1;
                }
                self.block_upload_stream()
            }
            Step::Next => self.block_upload_stream(),
            Step::DoneEnd => {
                let Some(Transfer::BlockUpload { buf, peer_crc, .. }) = self.transfer.take() else {
                    unreachable!();
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

/// Recovers the address echoed in a segment frame's header.
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
            let mut b = vec![0u8; value.encoded_len()];
            let n = value.encode_as(dt, &mut b)?;
            b.truncate(n);
            Ok(b)
        }
    }
}
