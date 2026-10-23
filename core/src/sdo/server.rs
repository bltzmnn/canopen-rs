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
//! - One in-flight transfer per channel (CiA 301 behavior). A new initiate on
//!   a **busy** channel is answered with a single `InvalidCommandSpecifier`
//!   abort (0x0504_0001) that names the stale transfer; the new request is
//!   *not* serviced (docs/DESIGN-DECISIONS.md SDO-01, matching CANopenNode
//!   `CO_SDOserver.c:803-841,1025-1058`, which routes a non-idle initiate into
//!   `default → CO_SDO_AB_CMD`). The peer must abort first; an idle channel
//!   still dispatches normally.
//!   Exception: while a **block download** is active every non-abort frame is
//!   routed as a segment (sub-segment sequence numbers are wire-ambiguous with
//!   initiates — CANopenNode routes the same way), so the abort-only policy
//!   applies at the initiate entry points that are reachable in each phase.
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
    SdoPayload, CCS_DOWNLOAD_INITIATE, CCS_DOWNLOAD_SEGMENT, CCS_UPLOAD_INITIATE,
    CCS_UPLOAD_SEGMENT, CS_ABORT, EXPEDITED, SEGMENT_DATA_MAX, SIZE_INDICATED, TOGGLE,
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
        /// segments (docs/DESIGN-DECISIONS.md P1-1). Mirrors CANopenNode's dedicated
        /// `DOWNLOAD_BLK_END_REQ` phase (`CO_SDOserver.c:906`).
        awaiting_end: bool,
        /// A partial ACK was already sent for the current silent sub-block
        /// window (docs/DESIGN-DECISIONS.md 审-S3). Re-armed by every received segment; a
        /// second consecutive timeout aborts, mirroring CANopenNode where
        /// the partial ACK resets only `block_timeoutTimer` and the general
        /// `timeoutTimer` still expires (`CO_SDOserver.c:1092-1101,1360`).
        partial_ack_sent: bool,
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
        /// The END frame has been sent and the client's `0xA1` end response
        /// is still outstanding. CANopenNode keeps the dedicated
        /// `UPLOAD_BLK_END_CRSP` phase here: `0xA1` completes the transfer,
        /// while every other frame falls into the receive switch's
        /// `default → CO_SDO_AB_CMD` (0x0504_0001, addressed at the in-flight
        /// object — `CO_SDOserver.c:1025-1058,1496-1503`); a truly idle
        /// `0xA1` is likewise an illegal command (docs/DESIGN-DECISIONS.md 审-S4/SDO-31).
        end_sent: bool,
    },
}

/// Optional SDO server strictness controls.
///
/// The defaults keep the historical/python-canopen-compatible lenient
/// behaviour. `strict` switches to CANopenNode's exact command-byte and
/// abort-byte validation (docs/DESIGN-DECISIONS.md SDO-33 / SDO-35), which rejects reserved
/// low bits with `0x0504_0001` instead of servicing them.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SdoServerConfig {
    /// Require exact command bytes: abort is only `0x80`, upload initiate is
    /// only `0x40`, upload-segment polls are `0x60`/`0x70`, and abort-like
    /// bytes `0x81..=0x9F` are invalid commands rather than aborts.
    pub strict: bool,
}

/// The SDO server state machine.
///
/// Construct one instance per SDO channel; call [`SdoServer::handle`] for
/// every received request frame.
#[derive(Debug)]
pub struct SdoServer {
    config: SdoServerConfig,
    pdo_policy: crate::pdo::PdoConfigPolicy,
    transfer: Option<Transfer>,
    /// Monotonic-ms stamp of the last observed activity; refreshed by
    /// [`Self::touch`] from the driver loop so the §5.5 timeout is a true
    /// *inactivity* window rather than a total-session cap
    /// (`docs/DESIGN-DECISIONS.md` R10-P1-4).
    last_activity_ms: Option<u64>,
    /// Addresses and values of successful OD writes since the last drain,
    /// consumed via [`SdoServer::take_last_written`] (used by `Node` for live
    /// PDO reconfiguration and storage magic words). Kept in arrival order;
    /// bounded by one entry per completed write transaction.
    last_written: Vec<(Address, Value)>,
    /// Most recent abort received from a client (CiA 309-style visibility;
    /// pyport test_local:118).
    last_received_abort: Option<SdoAbortCode>,
    /// Whether a parameter-storage backend is available to the caller, as
    /// last passed to [`Self::handle_with_storage`]. Gates the 0x1010/0x1011
    /// magic-word writes so a backend-less node answers 0x0601_0002 like
    /// CANopenLinux's `ODR_READONLY` (`storage/CO_storage.c:38-40`;
    /// `docs/DESIGN-DECISIONS.md` HOST-04).
    storage_enabled: bool,
}

/// `Default` is [`SdoServer::new`], not a bare derive: a `#[derive(Default)]`
/// would leave `storage_enabled: false` while `new()` sets `true`
/// (`docs/DESIGN-DECISIONS.md` SDO-42), so a `Default`-constructed server would behave
/// differently from a `new()` one as soon as any path reads the field.
impl Default for SdoServer {
    fn default() -> Self {
        Self::new()
    }
}

impl SdoServer {
    /// Creates an idle server.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            config: SdoServerConfig { strict: false },
            pdo_policy: crate::pdo::PdoConfigPolicy::AssemblyTime,
            transfer: None,
            last_activity_ms: None,
            last_written: Vec::new(),
            last_received_abort: None,
            storage_enabled: true,
        }
    }

    /// Replaces the optional strictness configuration.
    pub fn set_config(&mut self, config: SdoServerConfig) {
        self.config = config;
    }

    /// Current strictness configuration.
    #[must_use]
    pub const fn config(&self) -> SdoServerConfig {
        self.config
    }

    /// Selects when PDO mapping writes are validated.
    pub fn set_pdo_policy(&mut self, policy: crate::pdo::PdoConfigPolicy) {
        self.pdo_policy = policy;
    }

    /// Current PDO write-validation policy.
    #[must_use]
    pub const fn pdo_policy(&self) -> crate::pdo::PdoConfigPolicy {
        self.pdo_policy
    }

    /// Whether a multi-frame transfer is currently in flight.
    #[must_use]
    pub const fn is_busy(&self) -> bool {
        self.transfer.is_some()
    }

    /// Whether a block upload has sent its END frame and is waiting for the
    /// client's `0xA1` end response (CANopenNode `UPLOAD_BLK_END_CRSP`,
    /// `CO_SDOserver.c:65-67`).
    fn block_upload_end_sent(&self) -> bool {
        matches!(
            self.transfer,
            Some(Transfer::BlockUpload { end_sent: true, .. })
        )
    }

    /// Clears any active transfer (e.g. on NMT reset).
    pub fn reset(&mut self) {
        self.transfer = None;
        self.last_written.clear();
        self.last_activity_ms = None;
    }

    /// Session-timeout maintenance (§5.5, `docs/DESIGN-DECISIONS.md` P2-8): call from the
    /// driver loop with the current monotonic ms. When a transfer has been
    /// in flight for longer than `timeout_ms` without any frame arriving on
    /// the channel, it is torn down exactly like a received abort —
    /// releasing the server for the next transaction instead of leaving it
    /// wedged after a client vanished mid-transfer. The returned abort frame
    /// (`0x05040000`, naming the in-flight object) must be transmitted,
    /// matching CANopenNode `CO_SDOserver.c:1076-1079,1496-1509`
    /// (`docs/DESIGN-DECISIONS.md` C1). `None` means no timeout fired.
    ///
    /// One exception: a **block download** whose *sub-block* window lapses
    /// before the client's ACK is answered with a partial ACK instead of an
    /// abort — CANopenNode's `block_timeoutTimer` (0.7 × the general window,
    /// `block_SDOtimeoutTime_us = SDOtimeoutTime_ms * 700`, `:269`) flushes
    /// the received segments and prompts the client to retransmit the
    /// unacknowledged tail, keeping the transfer alive (`:1092-1101,
    /// 1320-1361`; `docs/DESIGN-DECISIONS.md` 审-S3 / SDO-26). The *general* `timeoutTimer`
    /// is not reset by that ACK (`:1359-1361`) and still aborts at 1×T, so
    /// the peer gets the reference's 0.3×T reaction window instead of the
    /// one-poll-tick window the shared threshold used to leave.
    ///
    /// # Driver contract
    /// The core is sans-I/O and clock-free: this method only runs when the
    /// host calls it. A host that never drives it leaves a peer-abandoned
    /// transfer wedged — `is_busy()` stays true and every new initiate is
    /// aborted with `InvalidCommandSpecifier` 0x0504_0001 (docs/DESIGN-DECISIONS.md SDO-27;
    /// the stale-transfer abort code, not the timeout's `ProtocolTimeout`).
    /// The bundled
    /// `serve_node` drivers call it with a 1500 ms default
    /// (`host/src/sdo_exchange.rs`); bare-core integrators must do the same.
    pub fn on_timeout(&mut self, now_ms: u64, timeout_ms: u64) -> Option<SdoPayload> {
        if !self.is_busy() {
            self.last_activity_ms = None;
            return None;
        }
        match self.last_activity_ms {
            Some(start) => {
                let elapsed = now_ms.saturating_sub(start);
                // The general inactivity window aborts at 1×T, and the
                // reference runs that check *before* the sub-block one
                // (`CO_SDOserver.c:1073-1079`) — a host that polls less often
                // than the window therefore sees the abort, not a late
                // partial ACK.
                if elapsed >= timeout_ms {
                    let addr = self.transfer.as_ref().map(transfer_addr);
                    self.transfer = None;
                    self.last_activity_ms = None;
                    return addr.map(|a| encode_abort(a, SdoAbortCode::ProtocolTimeout));
                }
                // Sub-block window at 0.7×T (`CO_SDOserver.c:269,1092-1101`,
                // docs/DESIGN-DECISIONS.md SDO-26): one partial ACK per silent episode; a
                // received segment re-arms both the window and the retry
                // (`record_activity`).
                // docs/DESIGN-DECISIONS.md SDO-39: `timeout_ms` comes from
                // `Node::set_sdo_timeout(timeout_ms: u64)` and has no upper
                // bound, so `timeout_ms * 7` could overflow (debug panic /
                // release wrap to a tiny window that aborts immediately).
                // Divide first; the sub-millisecond rounding is moot at
                // millisecond resolution.
                let sub_block_window = timeout_ms / 10 * 7 + timeout_ms % 10 * 7 / 10;
                if elapsed >= sub_block_window {
                    if let Some(partial_ack) = self.block_download_timeout_ack() {
                        return Some(partial_ack);
                    }
                }
                None
            }
            // Arm the window at first observation.
            None => {
                self.last_activity_ms = Some(now_ms);
                None
            }
        }
    }

    /// Partial ACK for a lapsed block-download sub-block window, or `None`
    /// when the transfer is not a block download in its segment phase or the
    /// current silent episode already used its retry (`docs/DESIGN-DECISIONS.md` 审-S3).
    ///
    /// The ACK carries the number of segments received in the current
    /// sub-block and restarts its sequence numbering, so the sender
    /// retransmits from `ackseq + 1` — exactly the bookkeeping of the
    /// gap-ACK path and of CANopenNode's `DOWNLOAD_BLK_SUBBLOCK_RSP`
    /// (`CO_SDOserver.c:1320-1361`).
    fn block_download_timeout_ack(&mut self) -> Option<SdoPayload> {
        let Some(Transfer::BlockDownload {
            awaiting_end,
            partial_ack_sent,
            blksize,
            received,
            expected_seqno,
            ..
        }) = self.transfer.as_mut()
        else {
            return None;
        };
        if *awaiting_end || *partial_ack_sent {
            return None;
        }
        *partial_ack_sent = true;
        *expected_seqno = 1;
        let ackseq = core::mem::replace(received, 0);
        Some(encode_sub_response(ackseq, *blksize))
    }

    /// Records activity at `now_ms`, sliding the §5.5 inactivity window
    /// forward. Call from the driver loop whenever any frame was handled —
    /// previously nothing refreshed the stamp, so long-running transfers
    /// were torn down after a fixed total duration instead of actual silence
    /// (`docs/DESIGN-DECISIONS.md` R10-P1-4).
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

    /// Aborts an active transfer and returns the ABORT frame to transmit.
    /// Empty when the channel was idle.
    ///
    /// The code is `InvalidCommandSpecifier` (0x0504_0001): CANopenNode
    /// answers a fresh initiate on a non-idle channel from its
    /// `default → CO_SDO_AB_CMD` branch (`CO_SDOserver.c:1025-1058`), and the
    /// previous `GeneralError` made the failure reason incomparable with the
    /// reference (`docs/DESIGN-DECISIONS.md` SDO-27).
    fn abort_stale_transfer(&mut self) -> Vec<SdoPayload> {
        match self.transfer.take() {
            Some(t) => vec![encode_abort(
                transfer_addr(&t),
                SdoAbortCode::InvalidCommandSpecifier,
            )],
            None => Vec::new(),
        }
    }

    /// Handles one request frame, returning the response payloads to transmit
    /// in order. Often empty or a single frame; a block-upload start or
    /// acknowledgement yields a whole sub-block at once.
    pub fn handle(&mut self, od: &mut ObjectDictionary, req: &SdoPayload) -> Vec<SdoPayload> {
        self.handle_with_storage(od, req, true)
    }

    /// [`Self::handle`], with the caller's view of whether a parameter-storage
    /// backend is attached.
    ///
    /// `storage_enabled == false` makes writes of the 0x1010/0x1011 magic
    /// words abort with `WriteOfReadOnlyObject` (0x0601_0002): CANopenLinux
    /// only installs the 0x1010 write handler when a store callback is
    /// registered and answers `ODR_READONLY` otherwise
    /// (`storage/CO_storage.c:38-40`), so accepting `'save'` on a node
    /// without a backend would report success while nothing is stored
    /// (`docs/DESIGN-DECISIONS.md` HOST-04). [`Node`](crate::node::Node) passes
    /// `self.storage.is_some()`; bare-core integrators that never installed
    /// 0x1010/0x1011 are unaffected either way.
    pub fn handle_with_storage(
        &mut self,
        od: &mut ObjectDictionary,
        req: &SdoPayload,
        storage_enabled: bool,
    ) -> Vec<SdoPayload> {
        self.storage_enabled = storage_enabled;
        // Active block download captures almost every frame shape (segments
        // carry raw sequence numbers) — route by state first. Only the exact
        // abort command byte (0x80, lower bits clear) is an abort: a
        // last-segment frame is always >= 0x81 since its sequence number >= 1.
        // Everything else is a data segment, *including* frames whose byte 0
        // looks like an initiate or abort command — the byte space collides
        // with the 1..=127 sequence numbers, so a new initiate cannot be
        // distinguished and CANopenNode digests it the same way
        // (`CO_SDOserver.c:68-126`, `docs/DESIGN-DECISIONS.md` R1 = 误报).
        if matches!(self.transfer, Some(Transfer::BlockDownload { .. })) {
            if req[0] == CS_ABORT {
                // docs/DESIGN-DECISIONS.md R2: record the abort code like the general abort
                // path does, so `take_last_received_abort` does not depend on
                // which internal state the channel happened to be in.
                if let Ok((_, code)) = super::decode_abort(req) {
                    self.last_received_abort = Some(code);
                }
                self.transfer = None;
                return Vec::new();
            }
            // Phase separation (docs/DESIGN-DECISIONS.md P1-1): the end frame is only
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
                // explicit abort instead of being digested as a segment
                // (`CO_SDOserver.c:907-934`). The abort names the in-flight
                // object, not the frame's data bytes (docs/DESIGN-DECISIONS.md B1).
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

        if self.config.strict && !strict_command_valid(req) {
            // CANopenNode dispatches exact command bytes (`CO_SDOserver.c:620-632`)
            // and treats reserved low bits as `CO_SDO_AB_CMD`; python-canopen
            // masks them (docs/DESIGN-DECISIONS.md SDO-33/SDO-35). Opt-in only.
            let abort_addr = self.transfer.as_ref().map_or_else(
                || Address::new(u16::from_le_bytes([req[1], req[2]]), req[3]),
                transfer_addr,
            );
            self.transfer = None;
            return vec![encode_abort(
                abort_addr,
                SdoAbortCode::InvalidCommandSpecifier,
            )];
        }
        let addr = Address::new(u16::from_le_bytes([req[1], req[2]]), req[3]);
        // A block upload whose END frame is awaiting the client's end
        // response owns the channel: CANopenNode keeps the dedicated
        // `UPLOAD_BLK_END_CRSP` phase where only `0xA1` completes the
        // transfer (`CO_SDOserver.c:65-67`) and every other frame falls into
        // the receive switch's `default → CO_SDO_AB_CMD`
        // (`:1036-1058`). docs/DESIGN-DECISIONS.md SDO-04: the previous silent `Vec::new()`
        // delayed the error until the 1500 ms server timeout, and it made a
        // legitimate end response indistinguishable from an unsolicited one
        // (`docs/DESIGN-DECISIONS.md` 审-S4).
        //
        // docs/DESIGN-DECISIONS.md SDO-33: a **repeated** sub-response (`0xA2`) after the END
        // frame used to fall through to `block_upload_ack → DoneEnd`, which
        // re-sent the END frame and recovered a lossy link; it now aborts
        // `0x0504_0001`, which is what the reference does in
        // `UPLOAD_BLK_END_CRSP` (`CO_SDOserver.c:65-67,1053-1058`). The
        // recovery path is therefore intentionally gone — a peer that lost the
        // END frame must re-initiate the upload.
        if self.block_upload_end_sent() {
            if req[0] == CS_ABORT {
                if let Ok((_, code)) = super::decode_abort(req) {
                    self.last_received_abort = Some(code);
                }
                self.transfer = None;
            } else if req[0] == super::block::CS_5 | super::block::SUBCMD_END {
                self.transfer = None;
            } else {
                // Illegal in the END phase: abort naming the in-flight
                // transfer (the END frame's data bytes are not an address).
                let abort_addr = self.transfer.as_ref().map_or(addr, transfer_addr);
                self.transfer = None;
                return vec![encode_abort(
                    abort_addr,
                    SdoAbortCode::InvalidCommandSpecifier,
                )];
            }
            return Vec::new();
        }
        // docs/DESIGN-DECISIONS.md SDO-01 / §5.5: a fresh initiate on a *busy* channel gets a
        // single `InvalidCommandSpecifier` abort (0x0504_0001) naming the stale
        // transfer and nothing else — CANopenNode never services the new
        // request in the same step (`CO_SDOserver.c:803-841,1025-1058`).
        // Servicing both made the peer
        // observe a failure for a write whose side effect had already run, and
        // livelocked segmented/block retries (the retry kept aborting the
        // transfer the server had just armed). An idle channel (no in-flight
        // transfer) still dispatches normally.
        if is_initiate(req) {
            let aborted = self.abort_stale_transfer();
            if !aborted.is_empty() {
                return aborted;
            }
        }
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
            // Block download initiate (CS_6; initiate/end discriminated by
            // bit 0 alone — bit 1 is the size flag, docs/DESIGN-DECISIONS.md R10-P1-2).
            CS_6 => match super::block::decode_download_initiate(req) {
                Ok(init) => self.block_download_initiate(od, init),
                Err(_) => {
                    // Undecodable CS_6: clear any in-flight transfer like
                    // every other abort path (docs/DESIGN-DECISIONS.md #25).
                    // docs/DESIGN-DECISIONS.md SDO-05: the abort names the *in-flight* object
                    // — CANopenNode always answers with `SDO->index` and the
                    // final segment's bytes 1..3 are CRC, not an address
                    // (`CO_SDOserver.c:1496-1503`).
                    let abort_addr = self.transfer.as_ref().map_or(addr, transfer_addr);
                    self.transfer = None;
                    vec![encode_abort(
                        abort_addr,
                        SdoAbortCode::InvalidCommandSpecifier,
                    )]
                }
            },
            // CS_5 family: block upload initiate / start / sub-response /
            // late end-response.
            CS_5 => self.on_cs5(od, req, addr),
            _ => {
                // Unknown command specifier: clear any in-flight transfer
                // (CANopenNode `CO_SDOserver.c` clears state on any illegal
                // frame, docs/DESIGN-DECISIONS.md #25) and name the *in-flight* object in the
                // abort — the frame's own bytes are exactly what could not be
                // decoded (docs/DESIGN-DECISIONS.md SDO-28; reference always answers with
                // `SDO->index`, `:1496-1503`).
                let abort_addr = self.transfer.as_ref().map_or(addr, transfer_addr);
                self.transfer = None;
                vec![encode_abort(
                    abort_addr,
                    SdoAbortCode::InvalidCommandSpecifier,
                )]
            }
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
        if let Some(Transfer::BlockUpload { end_sent, pos, .. }) = self.transfer {
            if end_sent {
                // Unreachable: `handle`'s END-phase guard consumes every
                // frame while `end_sent` is set (docs/DESIGN-DECISIONS.md SDO-31).
                return Vec::new();
            }
            if decode_upload_start(req).is_ok() {
                // The first `0xA3` starts the stream (CANopenNode
                // `SUBBLOCK_SREQ`). Once data has been sent, docs/DESIGN-DECISIONS.md SDO-40
                // applies: a repeated start is answered with `CO_SDO_AB_CMD`
                // (`CO_SDOserver.c:982-1024,1017-1019`), not treated as a
                // retransmission request. A peer that lost a frame must
                // re-initiate the upload.
                if pos == 0 {
                    return self.block_upload_stream();
                }
                let abort_addr = self.transfer.as_ref().map_or(addr, transfer_addr);
                self.transfer = None;
                return vec![encode_abort(
                    abort_addr,
                    SdoAbortCode::InvalidCommandSpecifier,
                )];
            }
            if is_sub_response(req) {
                return self.block_upload_ack(req);
            }
            // A brand-new initiate while a block upload streams: same
            // abort-only policy as the generic initiate path (docs/DESIGN-DECISIONS.md SDO-01).
            if decode_upload_initiate(req).is_ok() {
                return self.abort_stale_transfer();
            }
            // Neither a start, sub-response nor initiate: abort, naming the
            // in-flight object (this branch is only reachable for *illegal*
            // frames, so the frame header is not trustworthy — docs/DESIGN-DECISIONS.md
            // SDO-28; `CO_SDOserver.c:1496-1503`).
            let abort_addr = self.transfer.as_ref().map_or(addr, transfer_addr);
            self.transfer = None;
            return vec![encode_abort(
                abort_addr,
                SdoAbortCode::InvalidCommandSpecifier,
            )];
        }
        if subcmd == super::block::SUBCMD_END {
            // A CS=5 END frame with no END phase in flight is illegal: during
            // a segmented transfer it tears the session down (docs/DESIGN-DECISIONS.md C2;
            // 八M7), while an unsolicited one on an idle channel aborts
            // `InvalidCommandSpecifier` — CANopenNode's idle dispatch aborts
            // `CO_SDO_AB_CMD` for a bare `0xA1` (`CO_SDOserver.c:627-637`;
            // `docs/DESIGN-DECISIONS.md` 审-S4 / 五G13). python-canopen does *not* share this
            // code on that path: a bare `0xA1` goes through its ccs=5 block
            // upload branch into `_find_object` with address 0 and raises
            // `0x0602_0000` (`sdo/server.py:44-45`, `node/local.py:142-145`,
            // docs/DESIGN-DECISIONS.md SDO-09) — only its ccs=111 fallback answers 0x0504_0001
            // (`:51`). The END frame carries unused/CRC
            // bytes in positions 1-3, so the abort names the in-flight
            // object, not the request header (docs/DESIGN-DECISIONS.md B1).
            //
            // docs/DESIGN-DECISIONS.md SDO-43: on an **idle** channel there is no in-flight
            // object, and `addr` here is those very bytes — for an end frame
            // that is the CRC, not an index. CANopenNode has no idle case at
            // all (it aborts with `SDO->index`). Documented rather than
            // invented: there is nothing to name, and a python slave ignores
            // aborts for unknown addresses.
            let abort_addr = self.transfer.take().map_or(addr, |t| transfer_addr(&t));
            return vec![encode_abort(
                abort_addr,
                SdoAbortCode::InvalidCommandSpecifier,
            )];
        }
        match decode_upload_initiate(req) {
            Ok(neg) => self.block_upload_initiate(od, neg),
            Err(_) => {
                // docs/DESIGN-DECISIONS.md 八M7: a CS=5 frame that is neither start,
                // sub-response nor a block-upload initiate is as illegal
                // for an in-flight *segmented* transfer as for the block
                // case above — tear the transfer down together with the
                // abort, otherwise the session would keep serving segment
                // polls after having declared the abort (zombie session).
                // The abort names the torn-down transfer (docs/DESIGN-DECISIONS.md SDO-28).
                let abort_addr = self.transfer.as_ref().map_or(addr, transfer_addr);
                self.transfer = None;
                vec![encode_abort(
                    abort_addr,
                    SdoAbortCode::InvalidCommandSpecifier,
                )]
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
                // type-mismatch code, matching python-canopen's **numeric
                // types** (`node/local.py:130-132` raises 0x0607_0010 only
                // for `NUMBER_TYPES`, docs/DESIGN-DECISIONS.md SDO-10) — docs/EXTENSIONS.md
                // 批次 D 差异 D4.
                // CANopenNode is more specific here: it answers
                // `DATA_LONG`/`DATA_SHORT` (0x0607_0012/0x0013) for the same
                // input (`CO_SDOserver.c:706-740`), i.e. the codes this stack
                // already uses for the segmented/declared-size paths below.
                Err(Error::BadLength { .. }) => {
                    return vec![encode_abort(addr, SdoAbortCode::TypeMismatch)];
                }
                Err(e) => return vec![encode_abort(addr, abort_for(&e))],
            };
            if let Err(e) = od_write_guard(od, addr, &value, self.storage_enabled, self.pdo_policy)
            {
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
                       // docs/DESIGN-DECISIONS.md SDO-08: when the target has a *fixed* width the declared
                       // size can be validated right here — CANopenNode rejects the
                       // mismatch at initiate with DATA_LONG / DATA_SHORT
                       // (`CO_SDOserver.c:766-792`) instead of after the whole transfer.
                       // python-canopen defers the check to the end
                       // (`sdo/server.py:150-155`), which is what this stack used to do;
                       // variable-length targets (strings, blobs, Domain) still defer.
            if let (Some(expected), Some(fixed)) = (declared, dt.fixed_size()) {
                if expected > fixed {
                    return vec![encode_abort(addr, SdoAbortCode::TypeMismatchLengthHigh)];
                }
                if expected < fixed {
                    return vec![encode_abort(addr, SdoAbortCode::TypeMismatchLengthLow)];
                }
            }
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
        // frame's data bytes (docs/DESIGN-DECISIONS.md B1); echo the header only while idle.
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
        // segment can clear the in-flight transfer (docs/DESIGN-DECISIONS.md #25) without
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
                // docs/DESIGN-DECISIONS.md 十M7: a direction mismatch (download segment while
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
                // 越界校验：只按「本次追加后是否超过上限」判定——声明长度时用
                // `declared`，未声明时用全局 `MAX_VALUE_LEN` 防无界增长
                // （docs/DESIGN-DECISIONS.md P2-15；两分支合并自 P3-2）。
                //
                // docs/DESIGN-DECISIONS.md SDO-37: 此前还带一个 `buf.len() >= max` 的析取项，
                // 于是对端「先发满、再补一个**空**非末段」（合法的 0 字节段）
                // 也会被 abort 0x0607_0012。两参考都只按总长判定
                // （`CO_SDOserver.c:815-826`：`n = 0 → count = 0` 后仅比
                // `sizeTran > dataLength`；python `server.py:160-181` 按 n
                // 切片后直接 extend），故该写法在两参考上都可用。现在收满后
                // 不再追加字节的段放行并 ack（toggle 照常翻转），末尾的空**末**
                // 段仍走 `Finish` 分支。
                let limit = declared.unwrap_or(crate::datatypes::MAX_VALUE_LEN);
                let exceeds = buf.len() + seg.data.len() > limit;
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
                        if let Err(e) =
                            od_write_guard(od, addr, &value, self.storage_enabled, self.pdo_policy)
                        {
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
        if bytes.is_empty() {
            // CiA 301: a zero-length object cannot be uploaded expedited
            // nor segmented (there is no data to transfer). CANopenNode
            // aborts `DEVICE_INCOMPAT` = 0x0604_0047 from `readFromOd`
            // (`CO_SDOserver.c:557-563`); python-canopen aborts its
            // `ABORT_NO_DATA_AVAILABLE` = 0x0800_0024 (`sdo/server.py:68-72`).
            // Taking the CANopenNode code keeps the server wire behavior
            // aligned with the reference used elsewhere (`docs/DESIGN-DECISIONS.md` 审-S2).
            return vec![encode_abort(
                addr,
                SdoAbortCode::GeneralInternalIncompatibility,
            )];
        }
        self.upload_initiate_response(addr, bytes)
    }

    /// Emits the ordinary (non-block) upload-initiate response for an
    /// already-read value: expedited for `1..=4` bytes, segmented (size
    /// indicated) otherwise. Shared by [`Self::upload_initiate`] and the
    /// block-upload protocol switch.
    fn upload_initiate_response(&mut self, addr: Address, bytes: Vec<u8>) -> Vec<SdoPayload> {
        if (1..=4).contains(&bytes.len()) {
            match encode_upload_expedited_response(addr, &bytes) {
                Ok(p) => vec![p],
                Err(e) => vec![encode_abort(addr, abort_for(&e))],
            }
        } else {
            let size = u32::try_from(bytes.len()).unwrap_or(u32::MAX);
            self.transfer = Some(Transfer::Upload {
                addr,
                buf: bytes,
                pos: 0,
                expected_toggle: false,
            });
            vec![super::encode_upload_initiate_segmented_response(addr, size)]
        }
    }

    fn upload_segment(&mut self, req: &SdoPayload) -> Vec<SdoPayload> {
        let poll_toggle = req[0] & TOGGLE != 0;
        // A direction mismatch must still name the in-flight object
        // (docs/DESIGN-DECISIONS.md B1); echo the header only while idle.
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
                // docs/DESIGN-DECISIONS.md 十M7: symmetric with `download_segment` — an
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
                // 回显原地址而非 (0,0)（docs/DESIGN-DECISIONS.md P3-3；该路径当前不可达，仅代码卫生）。
                Err(e) => vec![encode_abort(addr, abort_for(&e))],
            }
        };
        if last {
            // CiA 301: transmitting the final segment (c=1) returns the
            // server to idle. Leaving the transfer armed left a zombie
            // session that poisoned the *next* transaction on this channel
            // with a stale-transfer abort (`InvalidCommandSpecifier`,
            // docs/DESIGN-DECISIONS.md SDO-01 / SDO-27).
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
        // docs/DESIGN-DECISIONS.md SDO-25: CANopenNode validates the indicated size against a
        // fixed-width OD variable right at the block-download initiate
        // (`CO_SDOserver.c:880-892`: `sizeInd > sizeInOd → DATA_LONG`,
        // `sizeInd < sizeInOd && !ODA_STR → DATA_SHORT`), exactly like the
        // segmented path at `:773-788`. Without this the mismatch only
        // surfaced after the whole block had been transferred.
        if let (Some(expected), Some(fixed)) = (init.size, dt.fixed_size()) {
            if expected as usize > fixed {
                return vec![encode_abort(addr, SdoAbortCode::TypeMismatchLengthHigh)];
            }
            if (expected as usize) < fixed {
                return vec![encode_abort(addr, SdoAbortCode::TypeMismatchLengthLow)];
            }
        }

        // Adaptive blksize ceiling: the buffer is dynamically sized, so the
        // protocol maximum applies (docs/IMPLEMENTATION.md §5.3).
        // Cap a peer-declared size by the absolute value limit too: u32::MAX
        // as `declared` would otherwise defeat the MAX_VALUE_LEN guard that
        // undeclared transfers have (docs/DESIGN-DECISIONS.md R10-P2-1).
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
            partial_ack_sent: false,
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
        // (docs/DESIGN-DECISIONS.md B1).
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
                partial_ack_sent,
                ..
            }) = self.transfer.as_mut()
            else {
                unreachable!("dispatch guarantees BlockDownload");
            };
            let seg = super::block::decode_sub_segment(req);
            if seg.seqno == 0 {
                // docs/DESIGN-DECISIONS.md SDO-30: the reference predicate is
                // `seqno != SDO->block_seqno && SDO->block_seqno != 0U`
                // (`CO_SDOserver.c:98`) — it tests the **expected** counter,
                // so a received `seqno == 0` while a sub-block is open is a
                // wrong sequence number like any other and must break the
                // sub-block (which acks `ackseq`, letting the peer resume)
                // rather than being dropped in silence. Only the *first*
                // segment of a sub-block (our `received == 0`, the
                // reference's "sequence didn't start yet" case) is ignored.
                if *received == 0 {
                    Step::Continue
                } else {
                    *expected_seqno = 1;
                    let ackseq = core::mem::replace(received, 0);
                    Step::Ack {
                        ackseq,
                        blksize: *blksize,
                    }
                }
            } else if seg.seqno < *expected_seqno {
                // Adjacent duplicate (e.g. our ack was lost): ignore it —
                // CANopenNode `CO_SDOserver.c:88-101` skips repeats instead
                // of tearing the transfer down (docs/DESIGN-DECISIONS.md P2-7).
                //
                // docs/DESIGN-DECISIONS.md SDO-07: a *non-adjacent* old sequence number is not
                // a repeat. The reference's predicate is
                // `seqno != block_seqno && block_seqno != 0`, which ends the
                // sub-block with a response (`:93-101`) so the sender rewinds
                // to the right offset instead of the server silently waiting.
                if seg.seqno + 1 == *expected_seqno {
                    Step::Continue
                } else {
                    *expected_seqno = 1;
                    let ackseq = core::mem::replace(received, 0);
                    Step::Ack {
                        ackseq,
                        blksize: *blksize,
                    }
                }
            } else if seg.seqno > *expected_seqno {
                // docs/DESIGN-DECISIONS.md #24: while awaiting the *renumbered*
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
                // Fresh traffic re-arms the sub-block timeout retry
                // (docs/DESIGN-DECISIONS.md 审-S3).
                *partial_ack_sent = false;
                let full = *received >= *blksize;
                if seg.last || full {
                    let ackseq = *received;
                    *expected_seqno = 1;
                    *received = 0;
                    if seg.last {
                        // Arm end-frame recognition only after this ack
                        // (docs/DESIGN-DECISIONS.md P1-1).
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
            // docs/DESIGN-DECISIONS.md 十L1: `Error::abort_code` distinguishes an over-long
            // payload (0x0607_0012) from a short one (0x0607_0013);
            // CANopenNode `CO_SDOserver.c:403-404` does the same.
            return vec![encode_abort(addr, abort_for(&e))];
        }
        let bytes = receiver.into_inner();
        match Value::decode_le(dt, &bytes) {
            Ok(value) => {
                if let Err(e) =
                    od_write_guard(od, addr, &value, self.storage_enabled, self.pdo_policy)
                {
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
        // Order mirrors CANopenNode's `UPLOAD_BLK_INITIATE_REQ` handling:
        // object/access checks (performed in the idle dispatch), then the
        // protocol switch threshold, then blksize validation — the reference
        // only validates blksize when it actually enters block mode
        // (`CO_SDOserver.c:938-958`).
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
        // Zero-length object: no data can be streamed. CANopenNode aborts
        // `DEVICE_INCOMPAT` = 0x0604_0047 already while reading the value
        // (`CO_SDOserver.c:557-563`), python-canopen aborts
        // `ABORT_NO_DATA_AVAILABLE` = 0x0800_0024 (`sdo/server.py:68-72`);
        // the CANopenNode code is used (docs/DESIGN-DECISIONS.md 审-S2).
        if bytes.is_empty() {
            return vec![encode_abort(
                addr,
                SdoAbortCode::GeneralInternalIncompatibility,
            )];
        }
        // Protocol switch threshold (CiA 301 §7.2.6.3): a client whose pst
        // (byte 5) is non-zero and not smaller than the object size prefers
        // a regular transfer — answer with the ordinary upload-initiate
        // response: expedited for `1..=4` bytes, segmented otherwise
        // (CANopenNode `CO_SDOserver.c:938-942` then `:1173-1193`). The
        // previous `size > 4` lower bound wrongly kept small objects in
        // block mode (`docs/DESIGN-DECISIONS.md` 审-S1).
        if let Some(pst) = neg.pst {
            if pst > 0 && u32::from(pst) >= size {
                return self.upload_initiate_response(addr, bytes);
            }
        }
        // CiA 301: an out-of-range blksize must be rejected explicitly, not
        // silently clamped (docs/DESIGN-DECISIONS.md P3-26).
        if !is_valid_blksize(neg.blksize) {
            return vec![encode_abort(addr, SdoAbortCode::InvalidBlockSize)];
        }

        self.transfer = Some(Transfer::BlockUpload {
            addr,
            buf: bytes,
            pos: 0,
            sub_base: 0,
            seqno: 1,
            blksize: clamp_blksize(neg.blksize),
            peer_crc: neg.crc_support,
            end_sent: false,
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
        // Zero-length objects abort at initiate (docs/DESIGN-DECISIONS.md 审-S2), so a
        // `BlockUpload` transfer always carries a non-empty buffer and the
        // empty-length branch is unreachable.
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
                // (docs/DESIGN-DECISIONS.md B1).
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
                // frame early (docs/DESIGN-DECISIONS.md R10-P2-6).
                Step::Clear(SdoAbortCode::InvalidCommandSpecifier)
            } else if ackseq < sent_this_sub {
                // Partial acknowledgement: retransmit the unacknowledged
                // tail with sequence numbering restarted at 1 (CiA 301 /
                // python-canopen semantics). The peer's new blksize proposal
                // is adopted right away, not one round late (docs/DESIGN-DECISIONS.md P3-27),
                // and validated exactly like the Next branch (docs/DESIGN-DECISIONS.md
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
                // A changed blksize must still be in range (docs/DESIGN-DECISIONS.md P3-26).
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
                    // base and re-sends already-acked data (docs/DESIGN-DECISIONS.md
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
                // docs/DESIGN-DECISIONS.md L5: this state is only reachable while a block
                // upload is in flight (the dispatch above enforces it), so
                // the transfer must be present — but keep the message for
                // diagnostics instead of a bare panic.
                let Some(Transfer::BlockUpload {
                    buf,
                    peer_crc,
                    end_sent,
                    ..
                }) = self.transfer.as_mut()
                else {
                    unreachable!("DoneEnd requires an active BlockUpload");
                };
                // A duplicate ACK while the client's end response is still
                // outstanding must not re-send the END frame; the `handle`
                // guard already routed such frames (docs/DESIGN-DECISIONS.md SDO-31) — this
                // branch is unreachable and kept only as a defensive
                // assertion of that invariant.
                if *end_sent {
                    return Vec::new();
                }
                let last_len = match buf.len() % BLOCK_SEGMENT_MAX {
                    0 if !buf.is_empty() => BLOCK_SEGMENT_MAX,
                    r => r,
                };
                let unused = (BLOCK_SEGMENT_MAX - last_len) as u8;
                let crc = crc16(buf);
                let _ = peer_crc; // CRC always computed; client verifies when negotiated
                                  // Stay armed in the END phase until the client's `0xA1`
                                  // arrives (docs/DESIGN-DECISIONS.md 审-S4); the transfer is cleared there or
                                  // by the inactivity timeout.
                *end_sent = true;
                vec![encode_end(unused, crc)]
            }
        }
    }
}

/// Distinguishes "whole index missing" from "subindex missing".
fn missing_code(od: &ObjectDictionary, addr: Address) -> SdoAbortCode {
    // docs/DESIGN-DECISIONS.md SDO-29: the OD is a flat (index, sub) table, so probing
    // `index:0` reported "object does not exist" for an index that only
    // carries sub 1..N (application-built records, EDS RECORD sections
    // without a sub-0 template). Index existence is the right criterion —
    // both references distinguish it from sub existence
    // (`CO_ODinterface.c:177-232`, python `node/local.py:142-153`).
    let index_exists = od.iter().any(|(a, _)| a.index == addr.index);
    // docs/DESIGN-DECISIONS.md SDO-34: the subindex is irrelevant to the choice — once the
    // index exists, *any* missing subentry (sub 0 included) is
    // `SUB_NOT_EXIST`. python-canopen checks `subindex not in obj` after
    // resolving the index (`node/local.py:142-152`) and CANopenNode's ODT
    // lookup returns `ODR_SUB_NOT_EXIST` on a miss without special-casing 0
    // (`CO_ODinterface.c:218-232`). The `addr.subindex != 0` guard left a
    // sub-only index (`0x2000:1..N`, or an EDS RECORD without a sub-0
    // section) reporting `0x0602_0000` for `0x2000:0` while `0x2000:5`
    // correctly reported `0x0609_0011`.
    if index_exists {
        SdoAbortCode::SubindexDoesNotExist
    } else {
        SdoAbortCode::ObjectDoesNotExist
    }
}

/// Exact command-byte gate used by [`SdoServerConfig::strict`].
///
/// Block families (`CS_5`/`CS_6`) keep their own reserved-bit validators;
/// download segments legitimately carry the `n` bits (unused-byte count), so
/// only the upload/abort bytes have a single legal form in the reference.
fn strict_command_valid(req: &SdoPayload) -> bool {
    match req[0] & super::CS_MASK {
        CS_ABORT => req[0] == CS_ABORT,
        CCS_UPLOAD_INITIATE => req[0] == CCS_UPLOAD_INITIATE,
        CCS_UPLOAD_SEGMENT => matches!(req[0], 0x60 | 0x70),
        _ => true,
    }
}

/// Pre-write protocol guards applied to every OD write reaching the server
/// (docs/DESIGN-DECISIONS.md P2-5 / P3-39):
///
/// - PDO communication-record COB-ID rules (bit layout and the CAN-ID-0 NMT
///   channel) — abort `INVALID_VALUE`-class codes. Restricted CAN-IDs and
///   the enable-with-empty-mapping gate are deliberately *not* applied here
///   (docs/DESIGN-DECISIONS.md 八M2: the live re-assembly architecture matches
///   python-canopen; the empty-mapping case surfaces through the
///   config-error ring);
/// - `0x1014` EMCY COB-ID rules, mirroring CANopenNode `OD_write_1014`
///   (`CO_Emergency.c:72-102`): reserved bits 30..11, restricted CAN-IDs
///   and a COB-ID change while production is enabled are rejected with
///   `InvalidValue` (0x0609_0030). A low-11-bit value of 0 disables
///   production (handled by `Node::emcy_cob_id`);
/// - `0x1003:0` must be written as zero; a non-zero value is rejected with
///   `InvalidValue` (0x0609_0030) exactly like CANopenNode's
///   `OD_write_1003` (`CO_Emergency.c:200-202` returns `ODR_INVALID_VALUE`)
///   instead of being silently rolled back after the fact;
/// - `0x1010`/`0x1011` only accept their magic words ('save'/'load');
///   anything else is `DataCannotBeTransferred` like CANopenNode
///   `CO_storage.c:42-45,96-99` (write of 0x1010) — previously every value
///   was confirmed and silently ignored. A *valid* magic word additionally
///   needs a storage backend (`storage_enabled`, from
///   [`SdoServer::handle_with_storage`]): without one the write is
///   `ODR_READONLY` (0x0601_0002), mirroring `CO_storage.c:38-40`
///   (`docs/DESIGN-DECISIONS.md` HOST-04).
fn od_write_guard(
    od: &ObjectDictionary,
    addr: Address,
    value: &Value,
    storage_enabled: bool,
    pdo_policy: crate::pdo::PdoConfigPolicy,
) -> crate::types::Result<()> {
    crate::pdo::guard_comm_record_write_with_policy(od, addr, value, pdo_policy)?;
    if addr == Address::new(0x1003, 0) {
        if let Value::Uint(n) = value {
            if *n != 0 {
                // CANopenNode `OD_write_1003` answers ODR_INVALID_VALUE
                // (0x0609_0030): a non-zero value is not a legal member of
                // this command at all, not a range overshoot — same rule as
                // the 0x1019 branch below (docs/DESIGN-DECISIONS.md 十L2).
                return Err(Error::InvalidValue);
            }
        }
    }
    if addr == Address::new(0x1019, 0) {
        if let Value::Uint(v) = value {
            // CANopenNode `OD_write_1019` (`CO_SYNC.c:151-153`): value 1 is
            // reserved and >240 exceeds the counter range — reject with
            // `ODR_INVALID_VALUE` (0x0609_0030), not a range code
            // (docs/DESIGN-DECISIONS.md 八L14 / 十L2).
            if *v == 1 || *v > 240 {
                return Err(Error::InvalidValue);
            }
        }
    }
    // 0x1005 SYNC COB-ID write guard (docs/DESIGN-DECISIONS.md 十M4): CANopenNode
    // `OD_write_1005` (`CO_SYNC.c:87-95`) rejects bit31, the reserved bits
    // 29..11 and restricted CAN-IDs; bit30 is the gen/producer flag.
    // Without this, an illegal value was stored and silently blinded the
    // node to every SYNC (no abort, no config-error entry).
    if addr == Address::new(0x1005, 0) {
        if let Value::Uint(v) = value {
            let raw = *v as u32;
            let new_can = (raw & 0x7FF) as u16;
            // docs/DESIGN-DECISIONS.md SYN-11: CANopenNode also refuses a COB-ID change while
            // the SYNC **producer** flag stays set — `(SYNC->isProducer &&
            // isProducer && (CAN_ID != SYNC->CAN_ID))` (`CO_SYNC.c:86-91`).
            // Note this is keyed on bit30 (gen), not on a "disabled" bit: 0x1005
            // has none (the write guard rejects bit31 outright), so dropping
            // the producer flag is the way to move the identifier — the RX
            // filter is then re-registered (`CO_CANrxBufferInit`,
            // `CO_SYNC.c:97-104`). The 0x1014 branch below implements the same
            // rule for its own disabled bit.
            let new_producer = raw & (1 << 30) != 0;
            let (cur_producer, cur_can) = match od.read(Address::new(0x1005, 0)) {
                Ok(Value::Uint(cur)) => {
                    let cur = cur as u32;
                    (cur & (1 << 30) != 0, (cur & 0x7FF) as u16)
                }
                _ => (false, 0),
            };
            if raw & 0xBFFF_F800 != 0
                || crate::pdo::is_restricted_can_id(new_can)
                || (cur_producer && new_producer && new_can != cur_can)
            {
                return Err(Error::InvalidValue);
            }
        }
    }
    // 0x1012 TIME COB-ID write guard (CiA 301 / `CO_TIME.c:58-80`):
    // bit31 = consumer, bit30 = producer, bits 29..11 reserved, bits 10..0
    // the CAN-ID. Restricted CAN-IDs are rejected; changing the identifier
    // is allowed (the reference re-registers the RX filter on init).
    if addr == Address::new(0x1012, 0) {
        if let Value::Uint(v) = value {
            let raw = *v as u32;
            let new_can = (raw & 0x7FF) as u16;
            if raw & 0x3FFF_F800 != 0 || crate::pdo::is_restricted_can_id(new_can) {
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
    // `SdoAbortCode::DataCannotBeTransferred` (0x0800_0020). A *valid* magic
    // word on a node without a storage backend is `ODR_READONLY` instead
    // (0x0601_0002, `CO_storage.c:38-40`) — confirming it would report a
    // successful store that never happens and leave the magic word in the
    // object dictionary (`docs/DESIGN-DECISIONS.md` HOST-04).
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
            if !storage_enabled {
                return Err(Error::AccessDenied { addr, read: false });
            }
        }
    }
    Ok(())
}

/// Whether `req` begins a fresh transfer: expedited/segmented/block download
/// or any upload initiate. Used for the §5.5 stale-transfer abort: on a busy
/// channel the server answers the new initiate with that single abort and does
/// **not** service it (`docs/DESIGN-DECISIONS.md` SDO-01).
fn is_initiate(req: &SdoPayload) -> bool {
    let cs = req[0] & super::CS_MASK;
    // Reserved bits 4..3 are part of the block-family command byte and must
    // be zero for a valid initiate (CANopenNode's `0xF9`/`0xFB` masks,
    // `CO_SDOserver.c:627-632`; docs/DESIGN-DECISIONS.md SDO-30).
    if matches!(cs, super::block::CS_5 | super::block::CS_6)
        && req[0] & super::block::RESERVED_MASK != 0
    {
        return false;
    }
    match cs {
        CCS_DOWNLOAD_INITIATE | CCS_UPLOAD_INITIATE => true,
        // CS_5 family: sub-command bits 1..0, initiate is exactly 00.
        super::block::CS_5 => req[0] & super::block::SUBCMD_MASK == super::block::SUBCMD_INITIATE,
        // CS_6 family (docs/DESIGN-DECISIONS.md R10-P1-2): bit 1 carries the size flag on
        // initiates, so the initiate/end discrimination is bit 0 alone —
        // matching `decode_initiate_with_size`. The old full-mask check
        // classified a size-carrying 0xC6 as "not an initiate" and let it
        // silently replace an active transfer without the §5.5 abort.
        super::block::CS_6 => req[0] & super::block::SUBCMD_END == 0,
        _ => false,
    }
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
            // docs/DESIGN-DECISIONS.md L7).
            let mut b = vec![0u8; value.encoded_len_as(dt)];
            let n = value.encode_as(dt, &mut b)?;
            b.truncate(n);
            Ok(b)
        }
    }
}
