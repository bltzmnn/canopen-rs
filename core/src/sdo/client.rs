//! SDO client: sans-I/O state machine driving read/write transactions.
//!
//! Usage pattern (the transport supplies the bus):
//!
//! 1. [`SdoClient::read`] / [`SdoClient::write`] / [`SdoClient::read_block`]
//!    start a transaction and return the first frame(s) to transmit on
//!    `0x600 + node`.
//! 2. Each frame received on `0x580 + node` is passed to
//!    [`SdoClient::on_response`], which yields the next batch of frames to
//!    send ([`SdoEvent::Send`]), the final value ([`SdoEvent::Complete`])
//!    plus any trailing frames, or write completion ([`SdoEvent::Done`]).
//!
//! Transfer mode selection:
//!
//! - **Writes** automatically use expedited (≤4 bytes), segmented (≤
//!   [`BLOCK_THRESHOLD`]) or block transfer (> threshold).
//! - **Reads** cannot know the size up front, so block upload is opt-in via
//!   [`SdoClient::read_block`] (mirroring python-canopen's explicit
//!   `block_transfer=True`).
//!
//! Robustness behaviors adopted from python-canopen (docs/REFERENCES.md
//! P8/P9): responses are validated against the requested address,
//! partial block acknowledgements trigger an ACK-based retransmission, and
//! [`SdoClient::abort_now`] produces an explicit ABORT frame for timeout
//! paths so the peer does not keep stale transfer state.

use super::block::{
    decode_download_initiate_response, decode_end, decode_end_response, decode_sub_response,
    decode_upload_initiate_response as decode_block_upload_init_resp, encode_download_initiate,
    encode_end_response, encode_sub_response, encode_upload_initiate, encode_upload_start,
    is_end_frame, is_sub_response, BlockReceiver, BlockWriter, MAX_BLKSIZE,
};
use super::{
    decode_abort, decode_data_segment, decode_download_response, decode_upload_expedited_response,
    decode_upload_initiate_response, encode_abort, encode_data_segment, encode_download_expedited,
    encode_download_initiate_segmented, encode_upload_request, encode_upload_segment_request,
    SdoPayload, CS_ABORT,
};
use crate::datatypes::{DataType, Value};
use crate::sdo::SdoAbortCode;
use crate::types::{Address, Error, Result};

/// Maximum data bytes carried by one segmented-transfer frame — single
/// source of truth in the module root (`BUGS.md` L6).
use super::SEGMENT_DATA_MAX;

/// Writes larger than this many bytes automatically use block transfer
/// (`MAX_BLKSIZE * SEGMENT_DATA_MAX`): beyond one full sub-block, per-segment
/// acknowledgements start dominating throughput.
pub const BLOCK_THRESHOLD: usize = MAX_BLKSIZE as usize * SEGMENT_DATA_MAX;

/// Outcome of feeding a response frame to [`SdoClient::on_response`].
#[derive(Debug, Clone, PartialEq)]
#[allow(variant_size_differences)]
pub enum SdoEvent {
    /// Transmit these frames next, in order (may be empty).
    Send(Vec<SdoPayload>),
    /// Read transaction finished with this value; transmit any trailing
    /// frames afterwards (e.g. the block-upload end response).
    Complete(Value, Vec<SdoPayload>),
    /// Write transaction finished; nothing more to send.
    Done,
}

#[derive(Debug, Default)]
enum State {
    #[default]
    Idle,
    /// Expedited/segmented download in progress.
    Downloading {
        addr: Address,
        data: Vec<u8>,
        pos: usize,
        /// Toggle bit carried by the last sent (or next) segment;
        /// acknowledgements must echo it.
        expected_toggle: bool,
        /// False until the download-initiate response (0x60) arrives.
        initiated: bool,
        /// Zero-length segmented transfer (`BUGS.md` R10-P2-4): after the
        /// initiate response the only thing left to send is the empty final
        /// segment (`c=1`, no data) — `pos == data.len()` cannot be used as
        /// the expedited-completion signal here because both are 0.
        empty_final: bool,
    },
    /// Block download in progress.
    BlockDownloading {
        addr: Address,
        writer: BlockWriter,
        initiated: bool,
        /// End frame transmitted, awaiting the end response.
        sent_end: bool,
        /// Segments sent since the current sub-block started.
        sent_this_sub: u8,
        /// Writer position where the current sub-block began.
        sub_base: usize,
    },
    /// Upload initiate sent, waiting for the response (expedited/segmented).
    AwaitingUpload { addr: Address, dt: DataType },
    /// Segmented upload streaming in.
    Uploading {
        addr: Address,
        dt: DataType,
        buf: Vec<u8>,
        expected_toggle: bool,
        /// Server-declared total size (initiate response); segments beyond
        /// it fail the transfer instead of growing the buffer unbounded
        /// (`BUGS.md` P3-31). `None` falls back to [`MAX_VALUE_LEN`](crate::datatypes::MAX_VALUE_LEN).
        size_cap: Option<usize>,
    },
    /// Block upload streaming in.
    BlockUploading {
        addr: Address,
        dt: DataType,
        receiver: BlockReceiver,
        started: bool,
        size_hint: Option<u32>,
        expected_seqno: u8,
        received: u8,
        blksize: u8,
        peer_crc: bool,
        /// True once the last segment has been acknowledged; only then may an
        /// end frame be recognized. The wire formats of a last segment
        /// (`0x80 | seqno`) with seqno in {65,69,…,93} and of the end frame
        /// (`0xC1`) overlap, so earlier recognition would swallow valid data
        /// (BUGS.md P1-1).
        awaiting_end: bool,
    },
}

/// The SDO client state machine.
#[derive(Debug, Default)]
pub struct SdoClient {
    state: State,
    /// Abort frame queued by a failed `on_response` (protocol violation);
    /// drained and transmitted by the transport so the server is not left
    /// hanging until its own timeout (python-canopen aborts before raising).
    pending_abort: Option<SdoPayload>,
}

impl SdoClient {
    /// Creates an idle client.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: State::Idle,
            pending_abort: None,
        }
    }

    /// Whether no transaction is in progress.
    #[must_use]
    pub const fn is_idle(&self) -> bool {
        matches!(self.state, State::Idle)
    }

    /// Takes the abort frame queued by a protocol error in [`Self::on_response`],
    /// if any. The caller must transmit it on the request channel.
    pub fn take_pending_abort(&mut self) -> Option<SdoPayload> {
        self.pending_abort.take()
    }

    /// Starts a read transaction (expedited or segmented); returns the upload
    /// request frame.
    pub fn read(&mut self, addr: Address, dt: DataType) -> Vec<SdoPayload> {
        self.state = State::AwaitingUpload { addr, dt };
        vec![encode_upload_request(addr)]
    }

    /// Starts a block-upload read transaction. Use when the object is known
    /// to be large; the server then streams whole sub-blocks between
    /// acknowledgements.
    pub fn read_block(&mut self, addr: Address, dt: DataType) -> Vec<SdoPayload> {
        self.state = State::BlockUploading {
            addr,
            dt,
            receiver: BlockReceiver::new(),
            started: false,
            size_hint: None,
            expected_seqno: 1,
            received: 0,
            blksize: MAX_BLKSIZE,
            peer_crc: false,
            awaiting_end: false,
        };
        vec![encode_upload_initiate(addr, MAX_BLKSIZE, true)]
    }

    /// Starts an explicit block-download write transaction regardless of
    /// payload size — mirrors python-canopen's `block_transfer=True` opt-in.
    ///
    /// # Errors
    /// [`Error::BadLength`] when `data` is empty — block transfer has no
    /// zero-length representation; route empty values through
    /// [`Self::write`] (segmented) instead (`BUGS.md` R10-P2-4).
    pub fn write_block(&mut self, addr: Address, data: &[u8]) -> Result<Vec<SdoPayload>> {
        if data.is_empty() {
            return Err(Error::BadLength {
                declared: 1,
                actual: 0,
            });
        }
        let size = u32::try_from(data.len()).unwrap_or(u32::MAX);
        self.state = State::BlockDownloading {
            addr,
            writer: BlockWriter::new(data.to_vec(), MAX_BLKSIZE),
            initiated: false,
            sent_end: false,
            sent_this_sub: 0,
            sub_base: 0,
        };
        Ok(vec![encode_download_initiate(addr, Some(size), true)])
    }

    /// Starts a write transaction of raw little-endian bytes; returns the
    /// first frame batch. Chooses expedited (≤4 bytes), segmented (≤
    /// [`BLOCK_THRESHOLD`]) or block transfer automatically.
    ///
    /// # Errors
    /// [`Error::BadLength`] when `data` is empty.
    pub fn write(&mut self, addr: Address, data: &[u8]) -> Result<Vec<SdoPayload>> {
        if data.is_empty() {
            // Zero-length segmented transfer, mirroring python-canopen's
            // `WritableStream` for empty values (`size=0` initiate followed
            // by an empty final segment with `c=1`) — previously rejected
            // outright while the server happily accepts it (BUGS.md
            // R10-P2-4).
            self.state = State::Downloading {
                addr,
                data: Vec::new(),
                pos: 0,
                expected_toggle: false,
                initiated: false,
                empty_final: true,
            };
            return Ok(vec![encode_download_initiate_segmented(addr, 0)]);
        }
        if data.len() <= 4 {
            self.state = State::Downloading {
                addr,
                data: data.to_vec(),
                pos: data.len(),
                expected_toggle: false,
                initiated: false,
                empty_final: false,
            };
            Ok(vec![encode_download_expedited(addr, data)?])
        } else if data.len() <= BLOCK_THRESHOLD {
            let size = u32::try_from(data.len()).unwrap_or(u32::MAX);
            self.state = State::Downloading {
                addr,
                data: data.to_vec(),
                pos: 0,
                expected_toggle: false,
                initiated: false,
                empty_final: false,
            };
            Ok(vec![encode_download_initiate_segmented(addr, size)])
        } else {
            // Auto-select block transfer above the threshold (§5.3).
            let size = u32::try_from(data.len()).unwrap_or(u32::MAX);
            self.state = State::BlockDownloading {
                addr,
                writer: BlockWriter::new(data.to_vec(), MAX_BLKSIZE),
                initiated: false,
                sent_end: false,
                sent_this_sub: 0,
                sub_base: 0,
            };
            Ok(vec![encode_download_initiate(addr, Some(size), true)])
        }
    }

    /// Feeds one response frame received on the response channel.
    ///
    /// # Errors
    /// - [`Error::SdoAborted`] when the server aborted;
    /// - [`Error::UnexpectedFrame`] for frames not matching the current
    ///   transaction;
    /// - [`Error::Other`] on an echoed address mismatch (diagnostic for
    ///   "another client occupies this channel"; deliberately not queued as
    ///   an ABORT, `BUGS.md` P3-30);
    /// - [`Error::ToggleBit`] on segment toggle violations;
    /// - [`Error::CrcMismatch`] when a block transfer's CRC fails;
    /// - [`Error::BadLength`] when the server-declared size or the absolute
    ///   value cap is exceeded (`size_cap`), or a fixed-size decode fails.
    ///
    /// On protocol violations the address of the abandoned transaction is
    /// captured first and an ABORT frame is queued (retrieve it with
    /// [`Self::take_pending_abort`]) so the server can be released instead
    /// of waiting for its own timeout — python-canopen aborts before
    /// raising, CANopenNode enters `CO_SDO_ST_ABORT`.
    pub fn on_response(&mut self, resp: &SdoPayload) -> Result<SdoEvent> {
        // Capture where we are *before* the inner handler takes the state,
        // so a protocol error can still address an ABORT to the right
        // channel.
        let active_addr = self.active_addr();
        let result = self.on_response_inner(resp);
        // Protocol violations abandon the transaction: queue an ABORT so the
        // server is released instead of waiting for its own timeout. Covers
        // unexpected frames, toggle violations, CRC mismatches and length
        // violations (final-segment decode / block-receiver size checks).
        if let Err(
            e @ (Error::UnexpectedFrame
            | Error::ToggleBit
            | Error::CrcMismatch
            | Error::InvalidBlockSize
            | Error::BadLength { .. }),
        ) = &result
        {
            // Each violation is reported with its dedicated CiA 301 abort
            // code (python-canopen `sdo/client.py:306-311` and CANopenNode
            // `CO_SDOclient.c:624-631,1291-1303,1311-1314` all do): toggle
            // 0x0503_0000, length 0x0607_0012/13, CRC 0x0504_0004, block
            // size 0x0504_0002 and an illegal command specifier
            // 0x0504_0001 (`Error::abort_code` already maps all but the
            // last).
            let code = if matches!(e, Error::UnexpectedFrame) {
                SdoAbortCode::InvalidCommandSpecifier
            } else {
                e.abort_code().unwrap_or(SdoAbortCode::GeneralError)
            };
            if let Some(addr) = active_addr {
                self.pending_abort = Some(crate::sdo::encode_abort(addr, code));
            }
        }
        result
    }

    /// The address of the transaction in flight, if any.
    fn active_addr(&self) -> Option<Address> {
        match &self.state {
            State::Downloading { addr, .. }
            | State::BlockDownloading { addr, .. }
            | State::AwaitingUpload { addr, .. }
            | State::Uploading { addr, .. }
            | State::BlockUploading { addr, .. } => Some(*addr),
            State::Idle => None,
        }
    }

    #[allow(clippy::too_many_lines)]
    fn on_response_inner(&mut self, resp: &SdoPayload) -> Result<SdoEvent> {
        // Exact-byte abort check: a block final segment has byte 0 =
        // 0x80 | seqno (>= 0x81), so only the exact abort command byte counts.
        if resp[0] == CS_ABORT {
            let (_, code) = decode_abort(resp)?;
            self.state = State::Idle;
            return Err(Error::SdoAborted(code));
        }
        match std::mem::take(&mut self.state) {
            State::Idle => Err(Error::UnexpectedFrame),

            State::Downloading {
                addr,
                data,
                mut pos,
                mut expected_toggle,
                mut initiated,
                empty_final,
            } => {
                if !initiated {
                    let confirmed = decode_download_response(resp)?;
                    check_addr(&confirmed, &addr)?;
                    initiated = true;
                    if empty_final {
                        // Zero-length segmented transfer: emit the empty
                        // final segment and wait for its acknowledgement.
                        // `expected_toggle` keeps the just-sent value — the
                        // ack must echo it, and the regular flip happens in
                        // the acknowledged branch below (same convention as
                        // every other segment).
                        let frame = encode_data_segment(&[], expected_toggle, true)
                            .expect("empty segment is always valid");
                        self.state = State::Downloading {
                            addr,
                            data,
                            pos,
                            expected_toggle,
                            initiated,
                            empty_final,
                        };
                        return Ok(SdoEvent::Send(vec![frame]));
                    }
                    if pos == data.len() {
                        // Expedited: confirmation completes the transfer.
                        return Ok(SdoEvent::Done);
                    }
                } else {
                    let acked = super::decode_download_segment_response(resp)?;
                    if acked != expected_toggle {
                        return Err(Error::ToggleBit);
                    }
                    if pos == data.len() {
                        // Final segment acknowledged.
                        return Ok(SdoEvent::Done);
                    }
                    expected_toggle = !expected_toggle;
                }
                let remaining = data.len() - pos;
                let len = remaining.min(SEGMENT_DATA_MAX);
                let last = remaining <= SEGMENT_DATA_MAX;
                let chunk = data[pos..pos + len].to_vec();
                pos += len;
                self.state = State::Downloading {
                    addr,
                    data,
                    pos,
                    expected_toggle,
                    initiated,
                    empty_final,
                };
                Ok(SdoEvent::Send(vec![encode_data_segment(
                    &chunk,
                    expected_toggle,
                    last,
                )
                .expect("chunk bounded")]))
            }

            State::BlockDownloading {
                addr,
                mut writer,
                mut initiated,
                mut sent_end,
                mut sent_this_sub,
                mut sub_base,
            } => {
                if sent_end {
                    // End response confirms completion (CRC failures arrive
                    // as aborts and were handled above).
                    decode_end_response(resp)?;
                    return Ok(SdoEvent::Done);
                }
                if !initiated {
                    let neg = decode_download_initiate_response(resp)?;
                    check_addr(&neg.address, &addr)?;
                    if !super::block::is_valid_blksize(neg.blksize) {
                        // BUGS.md 八L7: dedicated abort 0x0504_0002.
                        return Err(Error::InvalidBlockSize);
                    }
                    writer.start_sub_block(neg.blksize);
                    initiated = true;
                    sub_base = 0;
                } else {
                    if !is_sub_response(resp) {
                        return Err(Error::UnexpectedFrame);
                    }
                    let (ackseq, next_blksize) = decode_sub_response(resp)?;
                    if ackseq > sent_this_sub {
                        return Err(Error::UnexpectedFrame);
                    }
                    if ackseq < sent_this_sub {
                        // Partial acknowledgement: retransmit from ackseq+1.
                        // Rewind path validates inside rewind's caller too.
                        if !super::block::is_valid_blksize(next_blksize) {
                            return Err(Error::InvalidBlockSize);
                        }
                        writer.rewind(sub_base, ackseq, next_blksize);
                    } else {
                        if !super::block::is_valid_blksize(next_blksize) {
                            return Err(Error::InvalidBlockSize);
                        }
                        writer.start_sub_block(next_blksize);
                    }
                    sub_base = writer.position();
                    sent_this_sub = 0;
                    if writer.is_done() {
                        // All data acknowledged; close with the end frame.
                        let end = writer.end_frame(true);
                        sent_end = true;
                        self.state = State::BlockDownloading {
                            addr,
                            writer,
                            initiated,
                            sent_end,
                            sent_this_sub,
                            sub_base,
                        };
                        return Ok(SdoEvent::Send(vec![end]));
                    }
                }
                let mut frames = Vec::new();
                while let Some(frame) = writer.next_segment() {
                    frames.push(frame);
                }
                sent_this_sub = u8::try_from(frames.len()).unwrap_or(u8::MAX);
                self.state = State::BlockDownloading {
                    addr,
                    writer,
                    initiated,
                    sent_end,
                    sent_this_sub,
                    sub_base,
                };
                Ok(SdoEvent::Send(frames))
            }

            State::AwaitingUpload { addr, dt } => {
                if let Ok(up) = decode_upload_expedited_response(resp) {
                    check_addr(&up.addr, &addr)?;
                    let value = decode_value(dt, up.data)?;
                    return Ok(SdoEvent::Complete(value, Vec::new()));
                }
                let (confirmed, size) = decode_upload_initiate_response(resp)?;
                check_addr(&confirmed, &addr)?;
                self.state = State::Uploading {
                    addr,
                    dt,
                    buf: Vec::new(),
                    expected_toggle: false,
                    // BUGS.md 十M6: a peer-declared size must be capped by the
                    // absolute maximum, exactly like the block path
                    // (`BlockUploading` size_hint) and the server side.
                    size_cap: size.map(|s| (s as usize).min(crate::datatypes::MAX_VALUE_LEN)),
                };
                Ok(SdoEvent::Send(vec![encode_upload_segment_request(false)]))
            }

            State::Uploading {
                addr,
                dt,
                mut buf,
                expected_toggle,
                size_cap,
            } => {
                let seg = decode_data_segment(resp)?;
                if seg.toggle != expected_toggle {
                    return Err(Error::ToggleBit);
                }
                // Enforce the server-declared size (and the absolute cap when
                // undeclared) so a faulty peer cannot grow the buffer
                // without bound (BUGS.md P3-31).
                let cap = size_cap.unwrap_or(crate::datatypes::MAX_VALUE_LEN);
                if buf.len() + seg.data.len() > cap {
                    return Err(Error::BadLength {
                        declared: cap,
                        actual: buf.len() + seg.data.len(),
                    });
                }
                buf.extend_from_slice(seg.data);
                if seg.last {
                    // BUGS.md 十M6: a final segment that is shorter than the
                    // server-declared size must abort with BadLength
                    // (0x0607_0013) — CANopenNode `CO_SDOclient.c:1299-1307`
                    // does the same, and the server-side guard is symmetric
                    // (`short_final_segment_against_declared_size_aborts`).
                    // Variable-length types previously accepted the
                    // truncated payload silently.
                    if let Some(declared) = size_cap {
                        if buf.len() != declared {
                            return Err(Error::BadLength {
                                declared,
                                actual: buf.len(),
                            });
                        }
                    }
                    let value = decode_value(dt, &buf)?;
                    return Ok(SdoEvent::Complete(value, Vec::new()));
                }
                self.state = State::Uploading {
                    addr,
                    dt,
                    buf,
                    expected_toggle: !expected_toggle,
                    size_cap,
                };
                Ok(SdoEvent::Send(vec![encode_upload_segment_request(
                    !expected_toggle,
                )]))
            }

            State::BlockUploading {
                addr,
                dt,
                mut receiver,
                started,
                size_hint,
                mut expected_seqno,
                mut received,
                blksize,
                peer_crc,
                awaiting_end,
            } => {
                if !started {
                    // Server-side protocol switch (CiA): a segmented
                    // initiate response here means "no block transfer" —
                    // either the object is tiny/empty or our pst asked for
                    // segmented. Continue as a plain segmented read
                    // (BUGS.md P3-28/P3-29).
                    if let Ok((confirmed, size)) = super::decode_upload_initiate_response(resp) {
                        check_addr(&confirmed, &addr)?;
                        self.state = State::Uploading {
                            addr,
                            dt,
                            buf: Vec::new(),
                            expected_toggle: false,
                            // 十M6: cap the declared size like the direct
                            // segmented path.
                            size_cap: size
                                .map(|s| (s as usize).min(crate::datatypes::MAX_VALUE_LEN)),
                        };
                        return Ok(SdoEvent::Send(vec![encode_upload_segment_request(false)]));
                    }
                    // A server may also legally answer a block-upload
                    // initiate with an *expedited* upload response when the
                    // object fits into 4 bytes (python-canopen's server does
                    // exactly that, `sdo/server.py:113-118`; CANopenNode's
                    // client handles this branch, `CO_SDOclient.c:1348-1383`).
                    // Complete directly instead of aborting.
                    if let Ok(up) = decode_upload_expedited_response(resp) {
                        check_addr(&up.addr, &addr)?;
                        let value = decode_value(dt, up.data)?;
                        return Ok(SdoEvent::Complete(value, Vec::new()));
                    }
                    let init = decode_block_upload_init_resp(resp)?;
                    check_addr(&init.address, &addr)?;
                    // Deliberately keep our own window size: both reference
                    // implementations do the same on block *upload*
                    // (python-canopen keeps the class attribute 127;
                    // CANopenNode only honours byte 4 in the download
                    // direction) — R10-P2-3 resolved as "keep, documented".
                    // (`BUGS.md` L6: the byte-4 counter-proposal field was
                    // removed from `BlockInitiate` as dead — nothing read it.)
                    self.state = State::BlockUploading {
                        addr,
                        dt,
                        receiver,
                        started: true,
                        size_hint: init.size,
                        expected_seqno,
                        received,
                        blksize,
                        peer_crc: init.crc_support,
                        awaiting_end: false,
                    };
                    return Ok(SdoEvent::Send(vec![encode_upload_start()]));
                }
                // Phase separation (BUGS.md P1-1): the end frame is only
                // recognized after the last segment has been acknowledged.
                // A last segment's wire form `0x80 | seqno` collides with the
                // end frame `0xC1` for seqno in {65,69,…,93}, so recognizing
                // it earlier would swallow valid data segments.
                if awaiting_end && is_end_frame(resp) {
                    let (unused, crc) = decode_end(resp)?;
                    receiver.finish(size_hint.map(|s| s as usize), unused, crc, peer_crc)?;
                    let value = decode_value(dt, &receiver.into_inner())?;
                    return Ok(SdoEvent::Complete(value, vec![encode_end_response()]));
                }
                if awaiting_end {
                    // CANopenNode's END phase likewise rejects anything but
                    // the end frame (CO_SDOclient.c UPLOAD_BLK_END_SREQ).
                    return Err(Error::UnexpectedFrame);
                }
                let seg = super::block::decode_sub_segment(resp);
                if seg.seqno == 0 || seg.seqno != expected_seqno {
                    // BUGS.md #24: while awaiting the *renumbered*
                    // retransmission (expected=1, received=0) any other
                    // sequence number is a stale straggler from the old
                    // window — ignore it without acknowledging, otherwise
                    // the server rewinds again and re-sends data we already
                    // buffered (duplicate data, CANopenNode likewise ignores
                    // out-of-order segments after a rewind). The initial
                    // window also starts at expected=1, so a legitimately
                    // re-sent seqno==1 flows through the normal path below.
                    if expected_seqno == 1 && received == 0 {
                        return Ok(SdoEvent::Send(Vec::new()));
                    }
                    // Gap detected: acknowledge what we have so far so the
                    // server retransmits from there. The server answers the
                    // partial ack with a *renumbered* rewind (seqno restarts
                    // at 1 — python-canopen `_ack_block` resets `_ackseq`
                    // to 0 for exactly this reason), so our expectation
                    // must reset too; keeping the old counter would reject
                    // every retransmitted segment and deadlock the transfer
                    // until the host deadline (BUGS.md P1-1, see git history).
                    self.state = State::BlockUploading {
                        addr,
                        dt,
                        receiver,
                        started: true,
                        size_hint,
                        expected_seqno: 1,
                        received: 0,
                        blksize,
                        peer_crc,
                        awaiting_end,
                    };
                    return Ok(SdoEvent::Send(vec![encode_sub_response(received, blksize)]));
                }
                // Cap an undeclared/huge size hint by the absolute value
                // limit so a malicious server cannot grow the buffer without
                // bound (BUGS.md #2; segmented path has the same guard).
                let cap = size_hint.map_or(crate::datatypes::MAX_VALUE_LEN, |s| {
                    (s as usize).min(crate::datatypes::MAX_VALUE_LEN)
                });
                receiver.push(&seg, Some(cap))?;
                received += 1;
                expected_seqno += 1;
                let window_full = seg.last || received >= blksize;
                let ackseq = received;
                if window_full {
                    expected_seqno = 1;
                    received = 0;
                }
                self.state = State::BlockUploading {
                    addr,
                    dt,
                    receiver,
                    started: true,
                    size_hint,
                    expected_seqno,
                    received,
                    blksize,
                    peer_crc,
                    awaiting_end: awaiting_end || seg.last,
                };
                if window_full {
                    Ok(SdoEvent::Send(vec![encode_sub_response(ackseq, blksize)]))
                } else {
                    Ok(SdoEvent::Send(Vec::new()))
                }
            }
        }
    }

    /// Aborts the current transaction (e.g. after a timeout): returns the
    /// ABORT frame to transmit and resets the client to idle. This mirrors
    /// python-canopen's "abort the peer before retrying" behavior.
    pub fn abort_now(&mut self, code: SdoAbortCode) -> Option<SdoPayload> {
        match std::mem::take(&mut self.state) {
            State::Idle => None,
            State::Downloading { addr, .. }
            | State::BlockDownloading { addr, .. }
            | State::AwaitingUpload { addr, .. }
            | State::Uploading { addr, .. }
            | State::BlockUploading { addr, .. } => Some(encode_abort(addr, code)),
        }
    }
}

/// Validates that the response echoes the requested address.
///
/// A mismatch means another client is occupying this SDO channel. We
/// deliberately do **not** queue an ABORT for it (unlike other protocol
/// violations): the offender is an unknown third party and interrupting it
/// could corrupt its transfer — we only abandon our own transaction
/// (`BUGS.md` P3-30, resolved by documentation).
fn check_addr(confirmed: &Address, requested: &Address) -> Result<()> {
    if confirmed != requested {
        return Err(Error::Other("SDO response address mismatch"));
    }
    Ok(())
}

/// Decodes wire bytes into a typed value, enforcing fixed sizes.
fn decode_value(dt: DataType, bytes: &[u8]) -> Result<Value> {
    if let Some(size) = dt.fixed_size() {
        if bytes.len() != size {
            return Err(Error::BadLength {
                declared: size,
                actual: bytes.len(),
            });
        }
    }
    Value::decode_le(dt, bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn threshold_is_one_full_block() {
        assert_eq!(BLOCK_THRESHOLD, 127 * 7);
    }

    #[test]
    fn protocol_error_queues_abort_frame() {
        // BUGS.md M10: a protocol violation must queue an ABORT for the
        // server (python-canopen aborts before raising) instead of leaving
        // it hanging until its own timeout.
        let mut c = SdoClient::new();
        let addr = Address::new(0x2000, 0);
        let _ = c.read(addr, DataType::Unsigned32);

        // Wrong command specifier for an upload response → UnexpectedFrame.
        let mut resp = [0u8; 8];
        resp[0] = 0x60; // download-response specifier, not upload
        assert!(c.on_response(&resp).is_err());

        let abort = c.take_pending_abort().expect("abort queued");
        assert_eq!(abort[0], CS_ABORT);
        assert_eq!(&abort[1..4], &[0x00, 0x20, 0x00]);
        // 十M26: an illegal command specifier maps to 0x0504_0001, like
        // python-canopen and CANopenNode.
        let (_, code) = crate::sdo::decode_abort(&abort).unwrap();
        assert_eq!(code, SdoAbortCode::InvalidCommandSpecifier);
        // Drained: nothing left afterwards.
        assert!(c.take_pending_abort().is_none());
    }

    #[test]
    fn block_upload_server_expedited_fallback_completes() {
        // BUGS.md 十M26: a server may answer a block-upload initiate with an
        // expedited upload response (python-canopen's server does for ≤4-byte
        // objects); the client must complete instead of aborting.
        let mut c = SdoClient::new();
        let addr = Address::new(0x2000, 0);
        let init = c.read_block(addr, DataType::Unsigned32);
        assert_eq!(init[0][0] & 0xE0, 0xA0, "block-upload initiate request");
        let resp = crate::sdo::encode_upload_expedited_response(addr, &0x1234u32.to_le_bytes())
            .expect("4 bytes");
        match c.on_response(&resp).unwrap() {
            SdoEvent::Complete(Value::Uint(0x1234), _) => {}
            other => panic!("expected expedited completion, got {other:?}"),
        }
    }

    #[test]
    fn segmented_upload_decode_failure_queues_abort() {
        // REVIEW #3 (M10 coverage gap): a length mismatch on the final
        // segment abandons the transaction — an ABORT must be queued so the
        // server is released instead of waiting for its own timeout.
        let mut c = SdoClient::new();
        let addr = Address::new(0x2000, 0);
        let _ = c.read(addr, DataType::Unsigned32);

        // Server switches to a segmented transfer (size 4): client polls.
        let init = crate::sdo::encode_upload_initiate_segmented_response(addr, 4);
        assert!(c.on_response(&init).is_ok());

        // Final segment carries only 3 bytes — too short for UNSIGNED32.
        let seg = encode_data_segment(&[1, 2, 3], false, true).expect("≤7 bytes");
        assert!(matches!(c.on_response(&seg), Err(Error::BadLength { .. })));

        let abort = c.take_pending_abort().expect("abort queued");
        assert_eq!(abort[0], CS_ABORT);
        assert_eq!(&abort[1..4], &[0x00, 0x20, 0x00]);
        // 十M26: a short final segment maps to 0x0607_0013.
        let (_, code) = crate::sdo::decode_abort(&abort).unwrap();
        assert_eq!(code, SdoAbortCode::TypeMismatchLengthLow);
    }

    #[test]
    fn write_selects_modes_by_size() {
        let mut c = SdoClient::new();
        let addr = Address::new(0x2000, 0);

        // Expedited: single initiate request.
        let f = c.write(addr, &[1, 2, 3, 4]).unwrap();
        assert_eq!(f.len(), 1);
        assert_eq!(f[0][0], 0x23);

        // Segmented: size-indicated initiate (0x21).
        let f = c.write(addr, &[0xAB; 100]).unwrap();
        assert_eq!(f.len(), 1);
        assert_eq!(f[0][0], 0x21);

        // Block: block-download initiate (0xC6 = CS_6 | cc | s).
        let f = c.write(addr, &[0xAB; BLOCK_THRESHOLD + 1]).unwrap();
        assert_eq!(f.len(), 1);
        assert_eq!(f[0][0], 0xC6);
    }

    #[test]
    fn l7_invalid_server_blksize_gets_dedicated_abort() {
        // BUGS.md 八L7: a server counter-proposing an illegal blksize must
        // be aborted with 0x0504_0002, not a generic error.
        let mut c = SdoClient::new();
        let addr = Address::new(0x2000, 0);
        let _ = c.write(addr, &[0xAB; BLOCK_THRESHOLD + 1]).unwrap();

        let resp = crate::sdo::block::encode_download_initiate_response(addr, 0, false);
        let err = c.on_response(&resp).unwrap_err();
        assert!(matches!(err, Error::InvalidBlockSize), "got {err:?}");

        let abort = c.take_pending_abort().expect("abort queued");
        let (_, code) = crate::sdo::decode_abort(&abort).unwrap();
        assert_eq!(code, SdoAbortCode::InvalidBlockSize);
    }
}
