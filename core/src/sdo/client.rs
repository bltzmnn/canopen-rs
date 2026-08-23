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
    clamp_blksize, decode_download_initiate_response, decode_end, decode_end_response,
    decode_sub_response, decode_upload_initiate_response as decode_block_upload_init_resp,
    encode_download_initiate, encode_end_response, encode_sub_response, encode_upload_initiate,
    encode_upload_start, is_end_frame, is_sub_response, BlockReceiver, BlockWriter, MAX_BLKSIZE,
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

/// Maximum data bytes carried by one segmented-transfer frame.
const SEGMENT_DATA_MAX: usize = 7;

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
        };
        vec![encode_upload_initiate(addr, MAX_BLKSIZE, true)]
    }

    /// Starts an explicit block-download write transaction regardless of
    /// payload size — mirrors python-canopen's `block_transfer=True` opt-in.
    ///
    /// # Errors
    /// [`Error::BadLength`] when `data` is empty.
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
            return Err(Error::BadLength {
                declared: 1,
                actual: 0,
            });
        }
        if data.len() <= 4 {
            self.state = State::Downloading {
                addr,
                data: data.to_vec(),
                pos: data.len(),
                expected_toggle: false,
                initiated: false,
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
    ///   transaction (including an echoed address mismatch — diagnostic
    ///   for "another client occupies this channel");
    /// - [`Error::ToggleBit`] on segment toggle violations;
    /// - [`Error::CrcMismatch`] when a block transfer's CRC fails.
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
        if let Err(Error::UnexpectedFrame | Error::ToggleBit | Error::CrcMismatch) = &result {
            if let Some(addr) = active_addr {
                self.pending_abort =
                    Some(crate::sdo::encode_abort(addr, SdoAbortCode::GeneralError));
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
            } => {
                if !initiated {
                    let confirmed = decode_download_response(resp)?;
                    check_addr(&confirmed, &addr)?;
                    initiated = true;
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
                    writer.start_sub_block(clamp_blksize(neg.blksize));
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
                        writer.rewind(sub_base, ackseq, next_blksize);
                    } else {
                        writer.start_sub_block(clamp_blksize(next_blksize));
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
                let (confirmed, _size) = decode_upload_initiate_response(resp)?;
                check_addr(&confirmed, &addr)?;
                self.state = State::Uploading {
                    addr,
                    dt,
                    buf: Vec::new(),
                    expected_toggle: false,
                };
                Ok(SdoEvent::Send(vec![encode_upload_segment_request(false)]))
            }

            State::Uploading {
                addr,
                dt,
                mut buf,
                expected_toggle,
            } => {
                let seg = decode_data_segment(resp)?;
                if seg.toggle != expected_toggle {
                    return Err(Error::ToggleBit);
                }
                buf.extend_from_slice(seg.data);
                if seg.last {
                    let value = decode_value(dt, &buf)?;
                    return Ok(SdoEvent::Complete(value, Vec::new()));
                }
                self.state = State::Uploading {
                    addr,
                    dt,
                    buf,
                    expected_toggle: !expected_toggle,
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
            } => {
                if !started {
                    let init = decode_block_upload_init_resp(resp)?;
                    check_addr(&init.address, &addr)?;
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
                    };
                    return Ok(SdoEvent::Send(vec![encode_upload_start()]));
                }
                if is_end_frame(resp) {
                    let (unused, crc) = decode_end(resp)?;
                    receiver.finish(size_hint.map(|s| s as usize), unused, crc, peer_crc)?;
                    let value = decode_value(dt, &receiver.into_inner())?;
                    return Ok(SdoEvent::Complete(value, vec![encode_end_response()]));
                }
                let seg = super::block::decode_sub_segment(resp);
                if seg.seqno == 0 || seg.seqno != expected_seqno {
                    // Out-of-sequence: acknowledge what we have so far so the
                    // server retransmits from there.
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
                    };
                    return Ok(SdoEvent::Send(vec![encode_sub_response(received, blksize)]));
                }
                receiver.push(&seg, size_hint.map(|s| s as usize))?;
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
        // Drained: nothing left afterwards.
        assert!(c.take_pending_abort().is_none());
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
}
