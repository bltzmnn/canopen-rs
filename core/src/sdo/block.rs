//! SDO **block** transfer (CiA 301 §7.2.4.3) — high-throughput bulk transfer.
//!
//! Where segmented transfer acknowledges every 7-byte segment, block transfer
//! streams a whole *sub-block* of up to [`MAX_BLKSIZE`] segments before a
//! single acknowledgement, then verifies the whole transfer with a CRC-16.
//!
//! Command-specifier layout in byte 0's top three bits (swapped between the
//! two directions):
//! - `CS_6` (`110b`): download client / upload server — carries sizes and the
//!   end frame.
//! - `CS_5` (`101b`): download server / upload client — carries blksize
//!   negotiation, acknowledgements and confirmations.
//!
//! ```
//! use canopen_core::sdo::block::crc16;
//!
//! // CRC-16/XMODEM — the algorithm CANopen block transfer uses.
//! assert_eq!(crc16(b"123456789"), 0x31C3);
//! ```

use super::SdoPayload;
use crate::types::{Address, Error, Result};

// Command specifiers (top three bits).
pub(super) const CS_6: u8 = 0xC0;
pub(super) const CS_5: u8 = 0xA0;

const CS_MASK: u8 = 0xE0;

/// 'cc'/'sc' bit in an initiate frame; also the CRC-error flag in the end
/// response.
pub(super) const CRC_SUPPORTED: u8 = 0x04;
const SIZE_INDICATED: u8 = 0x02;

pub(super) const SUBCMD_MASK: u8 = 0x03;
const SUBCMD_INITIATE: u8 = 0x00;
pub(super) const SUBCMD_END: u8 = 0x01;
pub(super) const SUBCMD_RESPONSE: u8 = 0x02;
pub(super) const SUBCMD_START: u8 = 0x03;

const LAST_SEGMENT: u8 = 0x80; // 'c' bit in a sub-block segment
const SEQNO_MASK: u8 = 0x7F;

/// Maximum data bytes in one sub-block segment.
pub const SEGMENT_DATA_MAX: usize = 7;
/// Maximum segments in one sub-block.
pub const MAX_BLKSIZE: u8 = 127;

/// CRC-16/XMODEM (CCITT, polynomial `0x1021`, initial value `0`) over the
/// complete transferred data — the checksum CANopen block transfer uses.
#[must_use]
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Whether a frame is a block **end** frame (`CS_6`, sub-command end).
///
/// During an active block download every incoming frame is a sub-block
/// segment *except* this one, so receivers check it first — segment frames'
/// byte 0 carries a raw sequence number whose top bits vary.
#[must_use]
pub fn is_end_frame(p: &SdoPayload) -> bool {
    p[0] & CS_MASK == CS_6 && p[0] & SUBCMD_MASK == SUBCMD_END
}

/// Whether a frame is a sub-block **response** (`CS_5`, sub-command response).
#[must_use]
pub fn is_sub_response(p: &SdoPayload) -> bool {
    p[0] & CS_MASK == CS_5 && p[0] & SUBCMD_MASK == SUBCMD_RESPONSE
}

/// The size-carrying initiate frame: a download-initiate *request* or an
/// upload-initiate *response*.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockInitiate {
    /// The object being transferred.
    pub address: Address,
    /// The declared total size in bytes, if indicated.
    pub size: Option<u32>,
    /// Whether the sender supports the end-of-transfer CRC.
    pub crc_support: bool,
}

/// The blksize-carrying initiate frame: a download-initiate *response* or an
/// upload-initiate *request*.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockNegotiation {
    /// The object being transferred.
    pub address: Address,
    /// Segments allowed per sub-block before an acknowledgement (`1..=127`).
    pub blksize: u8,
    /// Whether the sender supports the end-of-transfer CRC.
    pub crc_support: bool,
}

/// A decoded sub-block segment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SubSegment<'a> {
    /// The sequence number within the sub-block (`1..=blksize`).
    pub seqno: u8,
    /// Whether this is the last segment of the whole transfer.
    pub last: bool,
    /// The seven raw payload bytes (the final segment's unused tail is
    /// trimmed later using the end frame's byte count).
    pub data: &'a [u8],
}

fn address_of(p: &SdoPayload) -> Address {
    Address::new(u16::from_le_bytes([p[1], p[2]]), p[3])
}

fn put_address(p: &mut SdoPayload, addr: Address) {
    p[1..3].copy_from_slice(&addr.index.to_le_bytes());
    p[3] = addr.subindex;
}

// --- Initiate handshake ------------------------------------------------------

/// Encodes a client **block-download initiate** request (`CS_6`).
#[must_use]
pub fn encode_download_initiate(addr: Address, size: Option<u32>, crc_support: bool) -> SdoPayload {
    encode_initiate_with_size(CS_6, addr, size, crc_support)
}

/// Decodes a block-download initiate request.
///
/// # Errors
/// [`Error::UnexpectedFrame`] on a foreign command byte.
pub fn decode_download_initiate(p: &SdoPayload) -> Result<BlockInitiate> {
    decode_initiate_with_size(CS_6, p)
}

/// Encodes the server's **block-download initiate response** (`CS_5`),
/// proposing `blksize` segments per sub-block.
#[must_use]
pub fn encode_download_initiate_response(
    addr: Address,
    blksize: u8,
    crc_support: bool,
) -> SdoPayload {
    encode_initiate_with_blksize(CS_5, addr, blksize, crc_support)
}

/// Decodes a block-download initiate response.
///
/// # Errors
/// [`Error::UnexpectedFrame`] on a foreign command byte.
pub fn decode_download_initiate_response(p: &SdoPayload) -> Result<BlockNegotiation> {
    decode_initiate_with_blksize(CS_5, p)
}

/// Encodes a client **block-upload initiate** request (`CS_5`) proposing
/// `blksize`.
#[must_use]
pub fn encode_upload_initiate(addr: Address, blksize: u8, crc_support: bool) -> SdoPayload {
    encode_initiate_with_blksize(CS_5, addr, blksize, crc_support)
}

/// Decodes a block-upload initiate request.
///
/// # Errors
/// [`Error::UnexpectedFrame`] on a foreign command byte.
pub fn decode_upload_initiate(p: &SdoPayload) -> Result<BlockNegotiation> {
    decode_initiate_with_blksize(CS_5, p)
}

/// Encodes the server's **block-upload initiate response** (`CS_6`), carrying
/// the transfer size.
#[must_use]
pub fn encode_upload_initiate_response(
    addr: Address,
    size: Option<u32>,
    crc_support: bool,
) -> SdoPayload {
    encode_initiate_with_size(CS_6, addr, size, crc_support)
}

/// Decodes a block-upload initiate response.
///
/// # Errors
/// [`Error::UnexpectedFrame`] on a foreign command byte.
pub fn decode_upload_initiate_response(p: &SdoPayload) -> Result<BlockInitiate> {
    decode_initiate_with_size(CS_6, p)
}

/// Encodes the client's **start block-upload** request (`CS_5`, sub-command
/// start), telling the server to begin streaming segments.
#[must_use]
pub fn encode_upload_start() -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CS_5 | SUBCMD_START;
    p
}

/// Decodes a start block-upload request.
///
/// # Errors
/// [`Error::UnexpectedFrame`] on a foreign command byte.
pub fn decode_upload_start(p: &SdoPayload) -> Result<()> {
    if p[0] & CS_MASK != CS_5 || p[0] & SUBCMD_MASK != SUBCMD_START {
        return Err(Error::UnexpectedFrame);
    }
    Ok(())
}

fn encode_initiate_with_size(cs: u8, addr: Address, size: Option<u32>, crc: bool) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = cs | SUBCMD_INITIATE;
    if crc {
        p[0] |= CRC_SUPPORTED;
    }
    if let Some(size) = size {
        p[0] |= SIZE_INDICATED;
        p[4..8].copy_from_slice(&size.to_le_bytes());
    }
    put_address(&mut p, addr);
    p
}

fn decode_initiate_with_size(cs: u8, p: &SdoPayload) -> Result<BlockInitiate> {
    // On a size-carrying initiate, bit 1 is the 's' flag, so the initiate/end
    // sub-command is bit 0 alone (0 = initiate); don't match the full 2 bits.
    if p[0] & CS_MASK != cs || p[0] & SUBCMD_END != 0 {
        return Err(Error::UnexpectedFrame);
    }
    let size = (p[0] & SIZE_INDICATED != 0).then(|| u32::from_le_bytes([p[4], p[5], p[6], p[7]]));
    Ok(BlockInitiate {
        address: address_of(p),
        size,
        crc_support: p[0] & CRC_SUPPORTED != 0,
    })
}

fn encode_initiate_with_blksize(cs: u8, addr: Address, blksize: u8, crc: bool) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = cs | SUBCMD_INITIATE;
    if crc {
        p[0] |= CRC_SUPPORTED;
    }
    put_address(&mut p, addr);
    p[4] = blksize;
    p
}

fn decode_initiate_with_blksize(cs: u8, p: &SdoPayload) -> Result<BlockNegotiation> {
    if p[0] & CS_MASK != cs || p[0] & SUBCMD_MASK != SUBCMD_INITIATE {
        return Err(Error::UnexpectedFrame);
    }
    Ok(BlockNegotiation {
        address: address_of(p),
        blksize: p[4],
        crc_support: p[0] & CRC_SUPPORTED != 0,
    })
}

// --- Sub-block segments and acknowledgements (shared both directions) --------

/// Encodes a **sub-block segment** carrying 1..=7 bytes with sequence number
/// `seqno` (`1..=127`); `last` marks the final segment of the transfer.
///
/// # Errors
/// [`Error::BadLength`] when `seqno` or `data.len()` is out of range.
pub fn encode_sub_segment(seqno: u8, data: &[u8], last: bool) -> Result<SdoPayload> {
    if seqno == 0 || seqno > MAX_BLKSIZE || data.is_empty() || data.len() > SEGMENT_DATA_MAX {
        return Err(Error::BadLength {
            declared: SEGMENT_DATA_MAX,
            actual: data.len(),
        });
    }
    let mut p = [0u8; 8];
    p[0] = seqno;
    if last {
        p[0] |= LAST_SEGMENT;
    }
    p[1..1 + data.len()].copy_from_slice(data);
    Ok(p)
}

/// Decodes a sub-block segment frame.
#[must_use]
pub fn decode_sub_segment(p: &SdoPayload) -> SubSegment<'_> {
    SubSegment {
        seqno: p[0] & SEQNO_MASK,
        last: p[0] & LAST_SEGMENT != 0,
        data: &p[1..8],
    }
}

/// Encodes a **sub-block response**: `ackseq` is the highest sequence number
/// correctly received, `blksize` the size of the next sub-block. (Sent by the
/// download server or the upload client — identical bytes.)
#[must_use]
pub fn encode_sub_response(ackseq: u8, blksize: u8) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CS_5 | SUBCMD_RESPONSE;
    p[1] = ackseq;
    p[2] = blksize;
    p
}

/// Decodes a sub-block response into `(ackseq, blksize)`.
///
/// # Errors
/// [`Error::UnexpectedFrame`] on a foreign command byte.
pub fn decode_sub_response(p: &SdoPayload) -> Result<(u8, u8)> {
    if p[0] & CS_MASK != CS_5 || p[0] & SUBCMD_MASK != SUBCMD_RESPONSE {
        return Err(Error::UnexpectedFrame);
    }
    Ok((p[1], p[2]))
}

/// Encodes the **end** frame: `unused` is how many of the last segment's seven
/// bytes carried no data; `crc` is the CRC over the whole transfer (`0` when
/// unused). (Sent by the download client or the upload server.)
#[must_use]
pub fn encode_end(unused: u8, crc: u16) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CS_6 | ((unused & 0x07) << 2) | SUBCMD_END;
    p[1..3].copy_from_slice(&crc.to_le_bytes());
    p
}

/// Decodes an end frame into `(unused, crc)`.
///
/// # Errors
/// [`Error::UnexpectedFrame`] on a foreign command byte.
pub fn decode_end(p: &SdoPayload) -> Result<(u8, u16)> {
    if p[0] & CS_MASK != CS_6 || p[0] & SUBCMD_MASK != SUBCMD_END {
        return Err(Error::UnexpectedFrame);
    }
    Ok(((p[0] >> 2) & 0x07, u16::from_le_bytes([p[1], p[2]])))
}

/// Encodes the **end response** confirming completion (download server or
/// upload client).
#[must_use]
pub fn encode_end_response() -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CS_5 | SUBCMD_END;
    p
}

/// Decodes an end response.
///
/// # Errors
/// [`Error::UnexpectedFrame`] on a foreign command byte.
pub fn decode_end_response(p: &SdoPayload) -> Result<()> {
    if p[0] & CS_MASK != CS_5 || p[0] & SUBCMD_MASK != SUBCMD_END {
        return Err(Error::UnexpectedFrame);
    }
    Ok(())
}

// --- Stateful helpers ----------------------------------------------------------

/// Clamps a peer-proposed blksize into the legal `1..=127` range.
#[must_use]
pub fn clamp_blksize(blksize: u8) -> u8 {
    blksize.clamp(1, MAX_BLKSIZE)
}

/// Splits owned data into sub-block segments — the engine behind the download
/// client and the upload server.
///
/// Drive pattern: emit [`BlockWriter::next_segment`] until `None`; if
/// [`BlockWriter::is_done`] send [`BlockWriter::end_frame`], otherwise await
/// the acknowledgement and call [`BlockWriter::start_sub_block`] with the
/// negotiated next blksize.
#[derive(Debug)]
pub struct BlockWriter {
    data: Vec<u8>,
    pos: usize,
    seqno: u8,
    blksize: u8,
}

impl BlockWriter {
    /// Starts splitting `data` using the negotiated first `blksize`.
    #[must_use]
    pub fn new(data: Vec<u8>, blksize: u8) -> Self {
        Self {
            data,
            pos: 0,
            seqno: 1,
            blksize: clamp_blksize(blksize),
        }
    }

    /// Whether every byte has been emitted (then send [`Self::end_frame`]).
    #[must_use]
    pub fn is_done(&self) -> bool {
        self.pos >= self.data.len()
    }

    /// Absolute position of the next byte to emit (for ACK-rewind retransmits).
    #[must_use]
    pub const fn position(&self) -> usize {
        self.pos
    }

    /// Rewinds to retransmit after a partial acknowledgement: the peer
    /// received only `ackseq` segments of the current sub-block. Per the
    /// CiA 301 block protocol (and python-canopen's mock script), the
    /// **sequence numbering restarts at 1** while the data position moves
    /// back to `sub_base + ackseq * 7`.
    ///
    /// `sub_base` must be the [`Self::position`] snapshot taken when the
    /// current sub-block started.
    pub fn rewind(&mut self, sub_base: usize, ackseq: u8, blksize: u8) {
        self.pos = sub_base + usize::from(ackseq) * SEGMENT_DATA_MAX;
        self.seqno = 1;
        self.blksize = clamp_blksize(blksize);
    }

    /// The next segment of the current sub-block, or `None` when the
    /// sub-block is full or the data is exhausted (disambiguate with
    /// [`Self::is_done`]).
    pub fn next_segment(&mut self) -> Option<SdoPayload> {
        if self.is_done() || self.seqno > self.blksize {
            return None;
        }
        let remaining = self.data.len() - self.pos;
        let take = remaining.min(SEGMENT_DATA_MAX);
        let last = remaining <= SEGMENT_DATA_MAX;
        let segment = encode_sub_segment(self.seqno, &self.data[self.pos..self.pos + take], last)
            .expect("seqno and length in range");
        self.pos += take;
        self.seqno += 1;
        Some(segment)
    }

    /// Begins the next sub-block after an acknowledgement, using `blksize`.
    pub fn start_sub_block(&mut self, blksize: u8) {
        self.seqno = 1;
        self.blksize = clamp_blksize(blksize);
    }

    /// The end frame: the CRC over all data (when `crc_support`) and the
    /// count of unused bytes in the last segment.
    #[must_use]
    pub fn end_frame(&self, crc_support: bool) -> SdoPayload {
        let last_len = match self.data.len() % SEGMENT_DATA_MAX {
            0 if !self.data.is_empty() => SEGMENT_DATA_MAX,
            r => r,
        };
        let unused = (SEGMENT_DATA_MAX - last_len) as u8;
        let crc = if crc_support { crc16(&self.data) } else { 0 };
        encode_end(unused, crc)
    }
}

/// Reassembles sub-block segments into a dynamically sized buffer — the
/// engine behind the download server and the upload client.
#[derive(Debug, Default)]
pub struct BlockReceiver {
    buf: Vec<u8>,
}

impl BlockReceiver {
    /// A new, empty receiver.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The reassembled bytes so far.
    #[must_use]
    pub fn as_slice(&self) -> &[u8] {
        &self.buf
    }

    /// Consumes the receiver, returning the reassembled bytes.
    #[must_use]
    pub fn into_inner(self) -> Vec<u8> {
        self.buf
    }

    /// Appends a decoded sub-block segment's bytes. Growth beyond `max_len`
    /// aborts, except on the segment flagged `last`, which may overshoot by
    /// up to one full segment (`SEGMENT_DATA_MAX`) — its unused tail is then
    /// trimmed by [`Self::finish`] down to the declared size (CANopenNode
    /// accumulates first and slices at the end).
    ///
    /// # Errors
    /// [`Error::BadLength`] when the bound is exceeded.
    pub fn push(&mut self, segment: &SubSegment<'_>, max_len: Option<usize>) -> Result<()> {
        if let Some(max) = max_len {
            // The final sub-block segment legitimately streams a full seven
            // bytes past the declared size before `finish()` trims the
            // unused tail (CANopenNode's client accumulates and slices at
            // the end the same way). Overshoot is therefore only tolerated
            // on the segment carrying `last`; any other growth beyond the
            // declared size aborts immediately.
            let projected = self.buf.len() + segment.data.len();
            let allowed = if segment.last {
                max + SEGMENT_DATA_MAX
            } else {
                max
            };
            if projected > allowed {
                return Err(Error::BadLength {
                    declared: max,
                    actual: projected,
                });
            }
        }
        self.buf.extend_from_slice(segment.data);
        Ok(())
    }

    /// Finalises: trims to `final_len` bytes (when known) or removes the
    /// `unused` tail, then optionally verifies the transfer CRC.
    ///
    /// # Errors
    /// - [`Error::BadLength`] when trimming below zero / beyond the buffer;
    /// - [`Error::CrcMismatch`] on a CRC failure.
    pub fn finish(
        &mut self,
        final_len: Option<usize>,
        unused: u8,
        crc: u16,
        verify_crc: bool,
    ) -> Result<()> {
        if let Some(n) = final_len {
            if n > self.buf.len() {
                return Err(Error::BadLength {
                    declared: n,
                    actual: self.buf.len(),
                });
            }
            self.buf.truncate(n);
        } else {
            let unused = usize::from(unused);
            if unused > self.buf.len() {
                return Err(Error::BadLength {
                    declared: self.buf.len().saturating_sub(unused),
                    actual: self.buf.len(),
                });
            }
            self.buf.truncate(self.buf.len() - unused);
        }
        if verify_crc && crc16(&self.buf) != crc {
            return Err(Error::CrcMismatch);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn receiver_bound_allows_only_last_segment_overshoot() {
        // BUGS.md m9: a declared 10-byte transfer streams 7 + (7 with pad).
        // Mid-transfer growth past the bound must abort; the final
        // segment's overshoot is trimmed by `finish`.
        let mut r = BlockReceiver::new();
        let seg1 = SubSegment {
            seqno: 1,
            last: false,
            data: &[0xAA; 7],
        };
        r.push(&seg1, Some(10)).unwrap();

        // Non-final push that would exceed the bound → rejected.
        let seg_bad = SubSegment {
            seqno: 2,
            last: false,
            data: &[0xBB; 7],
        };
        assert!(r.push(&seg_bad, Some(10)).is_err());

        // Fresh receiver: final segment may overshoot, then trim.
        let mut r = BlockReceiver::new();
        r.push(&seg1, Some(10)).unwrap();
        let seg_last = SubSegment {
            seqno: 2,
            last: true,
            data: &[0xCC; 7],
        };
        r.push(&seg_last, Some(10)).unwrap();
        r.finish(Some(10), 0, 0, false).unwrap();
        assert_eq!(r.into_inner().len(), 10);
    }

    #[test]
    fn crc16_known_vector() {
        assert_eq!(crc16(b"123456789"), 0x31C3); // canonical CRC-16/XMODEM check
        assert_eq!(crc16(&[]), 0x0000);
    }

    // Known-good frames: initiate a 100-byte download to 0x2000 with CRC and
    // size (command 0xC6), response with blksize 10 and server CRC (0xA4).
    #[test]
    fn download_initiate_frames() {
        let req = encode_download_initiate(Address::new(0x2000, 0), Some(100), true);
        assert_eq!(req, [0xC6, 0x00, 0x20, 0x00, 100, 0, 0, 0]);
        let resp = encode_download_initiate_response(Address::new(0x2000, 0), 10, true);
        assert_eq!(resp, [0xA4, 0x00, 0x20, 0x00, 10, 0, 0, 0]);
        assert_eq!(
            decode_download_initiate_response(&resp).unwrap().blksize,
            10
        );
    }

    // Upload mirrors download: request carries blksize (CS_5), response
    // carries size (CS_6), plus the start frame (0xA3).
    #[test]
    fn upload_initiate_and_start_frames() {
        let req = encode_upload_initiate(Address::new(0x2000, 0), 5, false);
        assert_eq!(req, [0xA0, 0x00, 0x20, 0x00, 5, 0, 0, 0]);

        let resp = encode_upload_initiate_response(Address::new(0x2000, 0), Some(42), true);
        assert_eq!(resp, [0xC6, 0x00, 0x20, 0x00, 42, 0, 0, 0]);
        assert_eq!(
            decode_upload_initiate_response(&resp).unwrap().size,
            Some(42)
        );

        assert_eq!(encode_upload_start(), [0xA3, 0, 0, 0, 0, 0, 0, 0]);
        assert!(decode_upload_start(&encode_upload_start()).is_ok());
    }

    #[test]
    fn shared_segment_and_ack_frames() {
        let f = encode_sub_segment(1, &[1, 2, 3, 4, 5, 6, 7], false).unwrap();
        assert_eq!(f, [0x01, 1, 2, 3, 4, 5, 6, 7]);
        let last = encode_sub_segment(3, &[0xAA, 0xBB], true).unwrap();
        assert_eq!(last, [0x83, 0xAA, 0xBB, 0, 0, 0, 0, 0]);
        assert!(decode_sub_segment(&last).last);

        assert_eq!(encode_sub_response(10, 20), [0xA2, 10, 20, 0, 0, 0, 0, 0]);
        assert_eq!(
            decode_sub_response(&encode_sub_response(10, 20)).unwrap(),
            (10, 20)
        );

        assert_eq!(encode_end(5, 0xBEEF), [0xD5, 0xEF, 0xBE, 0, 0, 0, 0, 0]);
        assert_eq!(decode_end(&encode_end(5, 0xBEEF)).unwrap(), (5, 0xBEEF));
        assert_eq!(encode_end_response(), [0xA1, 0, 0, 0, 0, 0, 0, 0]);
        assert!(decode_end_response(&encode_end_response()).is_ok());
    }

    #[test]
    fn frame_classification_helpers() {
        assert!(is_end_frame(&encode_end(0, 0)));
        assert!(is_sub_response(&encode_sub_response(1, 2)));
        assert!(is_end_frame(&encode_end(0, 0)));
        assert!(!is_sub_response(&[0x83, 0, 0, 0, 0, 0, 0, 0])); // last-segment byte0
        assert!(!is_sub_response(&encode_upload_start()));
    }

    /// Split `data` with a [`BlockWriter`] and reassemble with a
    /// [`BlockReceiver`], verifying the CRC — the shared engine of both
    /// directions.
    fn roundtrip(data: &[u8], blksize: u8) {
        let mut writer = BlockWriter::new(data.to_vec(), blksize);
        let mut receiver = BlockReceiver::new();
        loop {
            while let Some(seg) = writer.next_segment() {
                receiver.push(&decode_sub_segment(&seg), None).unwrap();
            }
            if writer.is_done() {
                break;
            }
            writer.start_sub_block(blksize);
        }
        let (unused, crc) = decode_end(&writer.end_frame(true)).unwrap();
        receiver.finish(None, unused, crc, true).unwrap();
        assert_eq!(receiver.as_slice(), data);
    }

    #[test]
    fn block_transfer_roundtrips() {
        let data: Vec<u8> = (0..30u8).collect();
        roundtrip(&data, 2); // spans multiple sub-blocks
        roundtrip(&data, 127); // one sub-block
        roundtrip(&[0xAB; 7], 4); // exactly one full segment
    }

    #[test]
    fn crc_mismatch_is_detected() {
        let data = [1u8, 2, 3, 4, 5];
        let mut writer = BlockWriter::new(data.to_vec(), 1);
        let mut receiver = BlockReceiver::new();
        while let Some(seg) = writer.next_segment() {
            receiver.push(&decode_sub_segment(&seg), None).unwrap();
        }
        let (unused, _) = decode_end(&writer.end_frame(true)).unwrap();
        assert_eq!(
            receiver.finish(None, unused, 0x0000, true),
            Err(Error::CrcMismatch)
        );
    }

    #[test]
    fn rewind_resumes_from_ack_point() {
        // Simulate: segments 1-2 arrived, segment 3 was lost. The peer ACKs 2,
        // the sender rewinds and resends from sequence number 3 onwards; the
        // reassembled stream must equal the original data.
        let data: Vec<u8> = (0..40u8).collect();
        let mut writer = BlockWriter::new(data.clone(), 3);
        let mut receiver = BlockReceiver::new();

        // Segments 1-2 delivered...
        for _ in 0..2 {
            let seg = writer.next_segment().expect("segments pending");
            receiver.push(&decode_sub_segment(&seg), None).unwrap();
        }
        // ...segment 3 lost; peer ACKs 2 of 3.
        writer.rewind(0, 2, 3);

        // Retransmit from sequence number 3 until the transfer completes.
        loop {
            while let Some(seg) = writer.next_segment() {
                receiver.push(&decode_sub_segment(&seg), None).unwrap();
            }
            if writer.is_done() {
                break;
            }
            writer.start_sub_block(3);
        }
        let (unused, crc) = decode_end(&writer.end_frame(true)).unwrap();
        receiver.finish(None, unused, crc, true).unwrap();
        assert_eq!(receiver.as_slice(), &data[..]);
    }
}
