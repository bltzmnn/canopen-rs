//! SDO (Service Data Object) protocol — wire codecs and helpers (CiA 301 §7.2.4).
//!
//! SDO provides confirmed, addressed read/write access to object dictionary
//! entries over a request channel (`0x600 + node`) and a response channel
//! (`0x580 + node`). The free functions here encode and decode the raw
//! 8-byte CAN *data field*; [`SdoServer`] services
//! requests against an `ObjectDictionary` and
//! [`SdoClient`] drives transactions — both sans-I/O.
//!
//! This module covers **expedited** (≤4 bytes inline) and **segmented**
//! transfer. Block transfer is integrated in phase P4.
//!
//! # Example: expedited download frame
//!
//! Writing `0x1234_5678` to object `0x2000` produces the known-good frame:
//!
//! ```
//! use canopen_core::sdo::encode_download_expedited;
//! use canopen_core::types::Address;
//!
//! let frame = encode_download_expedited(Address::new(0x2000, 0), &[0x78, 0x56, 0x34, 0x12])
//!     .unwrap();
//! // cmd | index (LE) | subindex | value (LE)
//! assert_eq!(frame, [0x23, 0x00, 0x20, 0x00, 0x78, 0x56, 0x34, 0x12]);
//! ```

pub mod block;
pub mod client;
pub mod server;

pub use client::{SdoClient, SdoEvent, BLOCK_THRESHOLD};
pub use server::SdoServer;

/// CiA 301 SDO abort codes.
///
/// `Display` prints the official description text; unknown codes are
/// preserved verbatim via [`SdoAbortCode::Other`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum SdoAbortCode {
    /// Toggle bit not alternated (0x0503_0000).
    ToggleNotAlternated,
    /// SDO protocol timed out (0x0504_0000).
    ProtocolTimeout,
    /// Client/server command specifier not valid or unknown (0x0504_0001).
    InvalidCommandSpecifier,
    /// Invalid block size (0x0504_0002).
    InvalidBlockSize,
    /// Invalid sequence number (0x0504_0003).
    InvalidSequenceNumber,
    /// CRC error (0x0504_0004).
    CrcError,
    /// Out of memory (0x0504_0005).
    OutOfMemory,
    /// Unsupported access to an object (0x0601_0000).
    UnsupportedAccess,
    /// Attempt to read a write-only object (0x0601_0001).
    ReadOfWriteOnlyObject,
    /// Attempt to write a read-only object (0x0601_0002).
    WriteOfReadOnlyObject,
    /// Object does not exist in the object dictionary (0x0602_0000).
    ObjectDoesNotExist,
    /// Object cannot be mapped to the PDO (0x0604_0041).
    ObjectCannotBeMapped,
    /// Number/length of objects to be mapped exceeds PDO length (0x0604_0042).
    MappingLengthExceeded,
    /// General parameter incompatibility (0x0604_0043).
    GeneralParameterIncompatibility,
    /// General internal incompatibility in the device (0x0604_0047).
    GeneralInternalIncompatibility,
    /// Access failed due to a hardware error (0x0606_0000).
    HardwareError,
    /// Data type does not match, service parameter length mismatch (0x0607_0010).
    TypeMismatch,
    /// Data type does not match, length too high (0x0607_0012).
    TypeMismatchLengthHigh,
    /// Data type does not match, length too low (0x0607_0013).
    TypeMismatchLengthLow,
    /// Sub-index does not exist (0x0609_0011).
    SubindexDoesNotExist,
    /// Invalid value for parameter (0x0609_0030).
    InvalidValue,
    /// Value of parameter written too high (0x0609_0031).
    ValueTooHigh,
    /// Value of parameter written too low (0x0609_0032).
    ValueTooLow,
    /// Maximum value is less than minimum value (0x0609_0036).
    MaxLessThanMin,
    /// Resource not available (0x060A_0023).
    ResourceNotAvailable,
    /// General error (0x0800_0000).
    GeneralError,
    /// Data cannot be transferred or stored (0x0800_0020).
    DataCannotBeTransferred,
    /// Data cannot be transferred due to local control (0x0800_0021).
    AccessDeniedDueToLocalControl,
    /// Data cannot be transferred due to present device state (0x0800_0022).
    CannotTransferDeviceState,
    /// No object dictionary present / dynamically generated (0x0800_0023).
    NoObjectDictionary,
    /// No data available (0x0800_0024).
    NoDataAvailable,
    /// Any code not covered by the variants above, preserved verbatim.
    Other(u32),
}

impl SdoAbortCode {
    /// Decodes from the raw 32-bit wire value.
    #[must_use]
    pub fn from_u32(raw: u32) -> Self {
        match raw {
            0x0503_0000 => Self::ToggleNotAlternated,
            0x0504_0000 => Self::ProtocolTimeout,
            0x0504_0001 => Self::InvalidCommandSpecifier,
            0x0504_0002 => Self::InvalidBlockSize,
            0x0504_0003 => Self::InvalidSequenceNumber,
            0x0504_0004 => Self::CrcError,
            0x0504_0005 => Self::OutOfMemory,
            0x0601_0000 => Self::UnsupportedAccess,
            0x0601_0001 => Self::ReadOfWriteOnlyObject,
            0x0601_0002 => Self::WriteOfReadOnlyObject,
            0x0602_0000 => Self::ObjectDoesNotExist,
            0x0604_0041 => Self::ObjectCannotBeMapped,
            0x0604_0042 => Self::MappingLengthExceeded,
            0x0604_0043 => Self::GeneralParameterIncompatibility,
            0x0604_0047 => Self::GeneralInternalIncompatibility,
            0x0606_0000 => Self::HardwareError,
            0x0607_0010 => Self::TypeMismatch,
            0x0607_0012 => Self::TypeMismatchLengthHigh,
            0x0607_0013 => Self::TypeMismatchLengthLow,
            0x0609_0011 => Self::SubindexDoesNotExist,
            0x0609_0030 => Self::InvalidValue,
            0x0609_0031 => Self::ValueTooHigh,
            0x0609_0032 => Self::ValueTooLow,
            0x0609_0036 => Self::MaxLessThanMin,
            0x060A_0023 => Self::ResourceNotAvailable,
            0x0800_0000 => Self::GeneralError,
            0x0800_0020 => Self::DataCannotBeTransferred,
            0x0800_0021 => Self::AccessDeniedDueToLocalControl,
            0x0800_0022 => Self::CannotTransferDeviceState,
            0x0800_0023 => Self::NoObjectDictionary,
            0x0800_0024 => Self::NoDataAvailable,
            other => Self::Other(other),
        }
    }

    /// Encodes into the raw 32-bit wire value.
    #[must_use]
    pub const fn to_u32(self) -> u32 {
        match self {
            Self::ToggleNotAlternated => 0x0503_0000,
            Self::ProtocolTimeout => 0x0504_0000,
            Self::InvalidCommandSpecifier => 0x0504_0001,
            Self::InvalidBlockSize => 0x0504_0002,
            Self::InvalidSequenceNumber => 0x0504_0003,
            Self::CrcError => 0x0504_0004,
            Self::OutOfMemory => 0x0504_0005,
            Self::UnsupportedAccess => 0x0601_0000,
            Self::ReadOfWriteOnlyObject => 0x0601_0001,
            Self::WriteOfReadOnlyObject => 0x0601_0002,
            Self::ObjectDoesNotExist => 0x0602_0000,
            Self::ObjectCannotBeMapped => 0x0604_0041,
            Self::MappingLengthExceeded => 0x0604_0042,
            Self::GeneralParameterIncompatibility => 0x0604_0043,
            Self::GeneralInternalIncompatibility => 0x0604_0047,
            Self::HardwareError => 0x0606_0000,
            Self::TypeMismatch => 0x0607_0010,
            Self::TypeMismatchLengthHigh => 0x0607_0012,
            Self::TypeMismatchLengthLow => 0x0607_0013,
            Self::SubindexDoesNotExist => 0x0609_0011,
            Self::InvalidValue => 0x0609_0030,
            Self::ValueTooHigh => 0x0609_0031,
            Self::ValueTooLow => 0x0609_0032,
            Self::MaxLessThanMin => 0x0609_0036,
            Self::ResourceNotAvailable => 0x060A_0023,
            Self::GeneralError => 0x0800_0000,
            Self::DataCannotBeTransferred => 0x0800_0020,
            Self::AccessDeniedDueToLocalControl => 0x0800_0021,
            Self::CannotTransferDeviceState => 0x0800_0022,
            Self::NoObjectDictionary => 0x0800_0023,
            Self::NoDataAvailable => 0x0800_0024,
            Self::Other(raw) => raw,
        }
    }
}

impl core::fmt::Display for SdoAbortCode {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let text = match self {
            Self::ToggleNotAlternated => "toggle bit not alternated",
            Self::ProtocolTimeout => "SDO protocol timed out",
            Self::InvalidCommandSpecifier => "invalid client/server command specifier",
            Self::InvalidBlockSize => "invalid block size",
            Self::InvalidSequenceNumber => "invalid sequence number",
            Self::CrcError => "CRC error",
            Self::OutOfMemory => "out of memory",
            Self::UnsupportedAccess => "unsupported access to an object",
            Self::ReadOfWriteOnlyObject => "attempt to read a write only object",
            Self::WriteOfReadOnlyObject => "attempt to write a read only object",
            Self::ObjectDoesNotExist => "object does not exist in the object dictionary",
            Self::ObjectCannotBeMapped => "object cannot be mapped to the PDO",
            Self::MappingLengthExceeded => {
                "number and length of the objects to be mapped would exceed PDO length"
            }
            Self::GeneralParameterIncompatibility => "general parameter incompatibility",
            Self::GeneralInternalIncompatibility => {
                "general internal incompatibility in the device"
            }
            Self::HardwareError => "access failed due to a hardware error",
            Self::TypeMismatch => {
                "data type does not match, length of service parameter does not match"
            }
            Self::TypeMismatchLengthHigh => {
                "data type does not match, length of service parameter too high"
            }
            Self::TypeMismatchLengthLow => {
                "data type does not match, length of service parameter too low"
            }
            Self::SubindexDoesNotExist => "sub-index does not exist",
            Self::InvalidValue => "invalid value for parameter",
            Self::ValueTooHigh => "value of parameter written too high",
            Self::ValueTooLow => "value of parameter written too low",
            Self::MaxLessThanMin => "maximum value is less than minimum value",
            Self::ResourceNotAvailable => "resource not available: SDO connection",
            Self::GeneralError => "general error",
            Self::DataCannotBeTransferred => "data cannot be transferred or stored",
            Self::AccessDeniedDueToLocalControl => {
                "data cannot be transferred or stored because of local control"
            }
            Self::CannotTransferDeviceState => {
                "data cannot be transferred or stored because of the present device state"
            }
            Self::NoObjectDictionary => {
                "object dictionary dynamic generation fails or no object dictionary present"
            }
            Self::NoDataAvailable => "no data available",
            Self::Other(raw) => return write!(f, "unknown abort code {raw:#010x}"),
        };
        write!(f, "{text} ({:#010x})", self.to_u32())
    }
}

use crate::types::{Address, Error, Result};

/// COB-ID base for SDO client→server (request) frames: `0x600 + node id`.
pub const SDO_REQUEST_COB_BASE: u16 = 0x600;
/// COB-ID base for SDO server→client (response) frames: `0x580 + node id`.
pub const SDO_RESPONSE_COB_BASE: u16 = 0x580;

/// The 8-byte SDO payload carried in a CAN frame's data field.
pub type SdoPayload = [u8; 8];

// --- Command specifiers (top three bits of byte 0) --------------------------
const CCS_DOWNLOAD_SEGMENT: u8 = 0x00; // client: 000xxxxx
const CCS_DOWNLOAD_INITIATE: u8 = 0x20; // client: 001xxxxx
const CCS_UPLOAD_INITIATE: u8 = 0x40; // client: 010xxxxx
const CCS_UPLOAD_SEGMENT: u8 = 0x60; // client: 011xxxxx
                                     // The upload-segment *response* specifier (scs 000) equals CCS_DOWNLOAD_SEGMENT,
                                     // so one data-segment codec serves both directions.
const SCS_DOWNLOAD_SEGMENT: u8 = 0x20; // server: 001xxxxx
const SCS_UPLOAD_INITIATE: u8 = 0x40; // server: 010xxxxx
const SCS_DOWNLOAD_INITIATE: u8 = 0x60; // server: 011xxxxx
const CS_ABORT: u8 = 0x80; // either:  100xxxxx

/// Top-three-bit command-specifier mask.
const CS_MASK: u8 = 0xE0;

// Low-byte flag bits.
const EXPEDITED: u8 = 0x02; // 'e' in an initiate frame
const SIZE_INDICATED: u8 = 0x01; // 's' in an initiate frame
const EXPEDITED_SIZED: u8 = EXPEDITED | SIZE_INDICATED;
const TOGGLE: u8 = 0x10; // 't' in a segment frame
const NO_MORE_SEGMENTS: u8 = 0x01; // 'c' in a data-segment frame

/// Maximum data bytes carried by a single SDO segment frame.
pub const SEGMENT_DATA_MAX: usize = 7;

/// The COB-ID of the SDO request channel (client → server) for `node`.
#[must_use]
pub const fn request_cob_id(node_id: u8) -> u16 {
    SDO_REQUEST_COB_BASE + node_id as u16
}

/// The COB-ID of the SDO response channel (server → client) for `node`.
#[must_use]
pub const fn response_cob_id(node_id: u8) -> u16 {
    SDO_RESPONSE_COB_BASE + node_id as u16
}

// --- Abort codes -------------------------------------------------------------
// `SdoAbortCode` lives at the top of this file (kept from phase P1); see the
// enum definition above the codec section.

fn addr_of(p: &SdoPayload) -> Address {
    Address::new(u16::from_le_bytes([p[1], p[2]]), p[3])
}

fn put_addr(p: &mut SdoPayload, addr: Address) {
    p[1..3].copy_from_slice(&addr.index.to_le_bytes());
    p[3] = addr.subindex;
}

// === Expedited transfer ======================================================

/// Encodes an expedited SDO **download** (write) request carrying `data`
/// (1..=4 bytes, little-endian already) to `addr`.
///
/// Command byte layout: `0x2F/0x2B/0x27/0x23` for 1/2/3/4 data bytes
/// (`n = 4 - len` unused-byte field at bits 3..2).
///
/// # Errors
/// [`Error::BadLength`] unless `1 <= data.len() <= 4`.
pub fn encode_download_expedited(addr: Address, data: &[u8]) -> Result<SdoPayload> {
    if data.is_empty() || data.len() > 4 {
        return Err(Error::BadLength {
            declared: 4,
            actual: data.len(),
        });
    }
    let mut p = [0u8; 8];
    p[0] = CCS_DOWNLOAD_INITIATE | (((4 - data.len()) as u8) << 2) | EXPEDITED_SIZED;
    put_addr(&mut p, addr);
    p[4..4 + data.len()].copy_from_slice(data);
    Ok(p)
}

/// Encodes the server's **download response** (write confirmation) for `addr`
/// (command `0x60`). Also used to acknowledge a segmented download initiate.
#[must_use]
pub fn encode_download_response(addr: Address) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = SCS_DOWNLOAD_INITIATE;
    put_addr(&mut p, addr);
    p
}

/// Decodes a server download response, returning the confirmed address.
///
/// # Errors
/// [`Error::UnexpectedFrame`] if not a download-initiate response.
pub fn decode_download_response(p: &SdoPayload) -> Result<Address> {
    if p[0] & CS_MASK != SCS_DOWNLOAD_INITIATE {
        return Err(Error::UnexpectedFrame);
    }
    Ok(addr_of(p))
}

/// Encodes an SDO **upload** (read) request for `addr` (command `0x40`).
#[must_use]
pub fn encode_upload_request(addr: Address) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CCS_UPLOAD_INITIATE;
    put_addr(&mut p, addr);
    p
}

/// Encodes the server's expedited **upload response** carrying `data`
/// (1..=4 bytes) for `addr` (command `0x43/0x47/0x4B/0x4F`).
///
/// # Errors
/// [`Error::BadLength`] unless `1 <= data.len() <= 4`.
pub fn encode_upload_expedited_response(addr: Address, data: &[u8]) -> Result<SdoPayload> {
    if data.is_empty() || data.len() > 4 {
        return Err(Error::BadLength {
            declared: 4,
            actual: data.len(),
        });
    }
    let mut p = [0u8; 8];
    p[0] = SCS_UPLOAD_INITIATE | (((4 - data.len()) as u8) << 2) | EXPEDITED_SIZED;
    put_addr(&mut p, addr);
    p[4..4 + data.len()].copy_from_slice(data);
    Ok(p)
}

/// A decoded expedited upload response: the confirmed address and the inline
/// data bytes (already sliced by the `n` field).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpeditedUpload<'a> {
    /// Confirmed object address.
    pub addr: Address,
    /// Inline payload (1..=4 bytes).
    pub data: &'a [u8],
}

/// Decodes an expedited upload response into its address and raw bytes.
///
/// Interpretation against the expected [`crate::datatypes::DataType`] is the
/// caller's job (via [`crate::datatypes::Value::decode_le`]).
///
/// # Errors
/// [`Error::UnexpectedFrame`] if not expedited/sized.
pub fn decode_upload_expedited_response(p: &SdoPayload) -> Result<ExpeditedUpload<'_>> {
    let cmd = p[0];
    // e=1 is required; s may be clear, in which case all four bytes are
    // data (python-canopen takes `res_data`, CANopenNode only honours the
    // size nibble when s is set).
    if cmd & CS_MASK != SCS_UPLOAD_INITIATE || cmd & EXPEDITED == 0 {
        return Err(Error::UnexpectedFrame);
    }
    let len = if cmd & SIZE_INDICATED != 0 {
        4 - usize::from((cmd >> 2) & 0x03)
    } else {
        4
    };
    Ok(ExpeditedUpload {
        addr: addr_of(p),
        data: &p[4..4 + len],
    })
}

/// Encodes an SDO **abort** frame for `addr` with `code`.
#[must_use]
pub fn encode_abort(addr: Address, code: crate::sdo::SdoAbortCode) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CS_ABORT;
    put_addr(&mut p, addr);
    p[4..8].copy_from_slice(&code.to_u32().to_le_bytes());
    p
}

/// Decodes an SDO abort frame into `(address, code)`.
///
/// # Errors
/// [`Error::UnexpectedFrame`] if not an abort frame.
pub fn decode_abort(p: &SdoPayload) -> Result<(Address, crate::sdo::SdoAbortCode)> {
    if p[0] & CS_MASK != CS_ABORT {
        return Err(Error::UnexpectedFrame);
    }
    let code = crate::sdo::SdoAbortCode::from_u32(u32::from_le_bytes([p[4], p[5], p[6], p[7]]));
    Ok((addr_of(p), code))
}

// === Segmented transfer ======================================================
//
// An *initiate* exchange declares the total byte count, then each side streams
// ≤7-byte segments with an alternating toggle bit; the final segment sets the
// 'no more segments' bit.

/// A decoded SDO data segment: toggle bit, last flag, and payload (≤7 bytes).
///
/// The download-segment request (client → server) and the upload-segment
/// response (server → client) share this exact layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Segment<'a> {
    /// Alternates per segment, starting at `false`.
    pub toggle: bool,
    /// Whether this is the final segment of the transfer.
    pub last: bool,
    /// Payload bytes (0..=7; empty allowed only with `last`).
    pub data: &'a [u8],
}

/// Encodes a client **segmented download initiate** request declaring a
/// transfer of `size` bytes to `addr` (command `0x21`).
#[must_use]
pub fn encode_download_initiate_segmented(addr: Address, size: u32) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CCS_DOWNLOAD_INITIATE | SIZE_INDICATED;
    put_addr(&mut p, addr);
    p[4..8].copy_from_slice(&size.to_le_bytes());
    p
}

/// A decoded segmented download initiate: `(address, declared size)`.
///
/// # Errors
/// [`Error::UnexpectedFrame`] if expedited or not size-indicated.
pub fn decode_download_initiate_segmented(p: &SdoPayload) -> Result<(Address, u32)> {
    if p[0] & CS_MASK != CCS_DOWNLOAD_INITIATE
        || p[0] & EXPEDITED != 0
        || p[0] & SIZE_INDICATED == 0
    {
        return Err(Error::UnexpectedFrame);
    }
    Ok((addr_of(p), u32::from_le_bytes([p[4], p[5], p[6], p[7]])))
}

/// Encodes the server's **segmented upload initiate response** declaring a
/// transfer of `size` bytes for `addr` (command `0x41`).
#[must_use]
pub fn encode_upload_initiate_segmented_response(addr: Address, size: u32) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = SCS_UPLOAD_INITIATE | SIZE_INDICATED;
    put_addr(&mut p, addr);
    p[4..8].copy_from_slice(&size.to_le_bytes());
    p
}

/// A decoded segmented upload initiate response: `(address, size)` or
/// [`None`] when the server did not indicate a size.
///
/// # Errors
/// [`Error::UnexpectedFrame`] if expedited.
pub fn decode_upload_initiate_response(p: &SdoPayload) -> Result<(Address, Option<u32>)> {
    if p[0] & CS_MASK != SCS_UPLOAD_INITIATE || p[0] & EXPEDITED != 0 {
        return Err(Error::UnexpectedFrame);
    }
    let sized = p[0] & SIZE_INDICATED != 0;
    let size = sized.then(|| u32::from_le_bytes([p[4], p[5], p[6], p[7]]));
    Ok((addr_of(p), size))
}

/// Encodes a **data segment** carrying `data` (0..=7 bytes).
///
/// Used for both the download-segment request and the upload-segment response.
///
/// # Errors
/// [`Error::BadLength`] if `data.len() > 7`.
pub fn encode_data_segment(data: &[u8], toggle: bool, last: bool) -> Result<SdoPayload> {
    if data.len() > SEGMENT_DATA_MAX {
        return Err(Error::BadLength {
            declared: SEGMENT_DATA_MAX,
            actual: data.len(),
        });
    }
    let mut p = [0u8; 8];
    // ccs/scs for a data segment are both 000, so top bits stay clear.
    let n = (SEGMENT_DATA_MAX - data.len()) as u8;
    p[0] = (n << 1) & 0x0E;
    if toggle {
        p[0] |= TOGGLE;
    }
    if last {
        p[0] |= NO_MORE_SEGMENTS;
    }
    p[1..1 + data.len()].copy_from_slice(data);
    Ok(p)
}

/// Decodes a **data segment** frame.
///
/// # Errors
/// [`Error::UnexpectedFrame`] if the command specifier is wrong.
pub fn decode_data_segment(p: &SdoPayload) -> Result<Segment<'_>> {
    if p[0] & CS_MASK != CCS_DOWNLOAD_SEGMENT {
        return Err(Error::UnexpectedFrame);
    }
    let n = usize::from((p[0] >> 1) & 0x07);
    Ok(Segment {
        toggle: p[0] & TOGGLE != 0,
        last: p[0] & NO_MORE_SEGMENTS != 0,
        data: &p[1..1 + (SEGMENT_DATA_MAX - n)],
    })
}

/// Encodes the server's **download segment response** (acknowledgement) with
/// the received segment's toggle bit (command `0x20 | toggle`).
#[must_use]
pub fn encode_download_segment_response(toggle: bool) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = SCS_DOWNLOAD_SEGMENT | if toggle { TOGGLE } else { 0 };
    p
}

/// Decodes a download segment response, returning its toggle bit.
///
/// # Errors
/// [`Error::UnexpectedFrame`] on wrong command specifier.
pub fn decode_download_segment_response(p: &SdoPayload) -> Result<bool> {
    if p[0] & CS_MASK != SCS_DOWNLOAD_SEGMENT {
        return Err(Error::UnexpectedFrame);
    }
    Ok(p[0] & TOGGLE != 0)
}

/// Encodes the client's **upload segment request** poll with `toggle`
/// (command `0x60 | toggle`).
#[must_use]
pub fn encode_upload_segment_request(toggle: bool) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CCS_UPLOAD_SEGMENT | if toggle { TOGGLE } else { 0 };
    p
}

/// Decodes an upload segment request, returning its toggle bit.
///
/// # Errors
/// [`Error::UnexpectedFrame`] on wrong command specifier.
pub fn decode_upload_segment_request(p: &SdoPayload) -> Result<bool> {
    if p[0] & CS_MASK != CCS_UPLOAD_SEGMENT {
        return Err(Error::UnexpectedFrame);
    }
    Ok(p[0] & TOGGLE != 0)
}

/// Splits owned data into SDO data segments, tracking the toggle bit.
///
/// Drive it after a successful initiate handshake: emit each
/// [`SegmentWriter::next_segment`], await the acknowledgement, repeat.
#[derive(Debug)]
pub struct SegmentWriter {
    data: Vec<u8>,
    pos: usize,
    toggle: bool,
}

impl SegmentWriter {
    /// Starts splitting `data` (the full >4-byte payload).
    #[must_use]
    pub const fn new(data: Vec<u8>) -> Self {
        Self {
            data,
            pos: 0,
            toggle: false,
        }
    }

    /// Whether every byte has been emitted.
    #[must_use]
    pub fn is_done(&self) -> bool {
        self.pos >= self.data.len()
    }

    /// Produces the next segment frame, or `None` when finished.
    pub fn next_segment(&mut self) -> Option<SdoPayload> {
        if self.is_done() {
            return None;
        }
        let remaining = self.data.len() - self.pos;
        let len = remaining.min(SEGMENT_DATA_MAX);
        let last = remaining <= SEGMENT_DATA_MAX;
        let frame = encode_data_segment(&self.data[self.pos..self.pos + len], self.toggle, last)
            .expect("segment length bounded");
        self.pos += len;
        self.toggle = !self.toggle;
        Some(frame)
    }

    /// The toggle bit the *next* emitted segment will carry (for ack checks).
    #[must_use]
    pub const fn current_toggle(&self) -> bool {
        self.toggle
    }
}

/// Reassembles streamed data segments into an unbounded buffer, validating
/// the toggle bit.
#[derive(Debug, Default)]
pub struct SegmentBuffer {
    buf: Vec<u8>,
    toggle: bool,
    done: bool,
}

impl SegmentBuffer {
    /// A new, empty reassembler.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether the final segment has been received.
    #[must_use]
    pub const fn is_done(&self) -> bool {
        self.done
    }

    /// The reassembled bytes.
    #[must_use]
    pub fn as_slice(&self) -> &[u8] {
        &self.buf
    }

    /// Consumes the buffer, returning the reassembled bytes.
    #[must_use]
    pub fn into_inner(self) -> Vec<u8> {
        self.buf
    }

    /// Appends a decoded segment.
    ///
    /// # Errors
    /// - [`Error::ToggleBit`] when out of sequence;
    /// - [`Error::UnexpectedFrame`] when already complete.
    pub fn push(&mut self, segment: &Segment<'_>, max_len: Option<usize>) -> Result<()> {
        if self.done {
            return Err(Error::UnexpectedFrame);
        }
        if segment.toggle != self.toggle {
            return Err(Error::ToggleBit);
        }
        if let Some(max) = max_len {
            // Our segment decoder strips the sender-declared padding (`n`),
            // so accumulated payload must never exceed the declared size;
            // tolerate no phantom "padding" slack.
            if self.buf.len() + segment.data.len() > max {
                return Err(Error::BadLength {
                    declared: max,
                    actual: self.buf.len() + segment.data.len(),
                });
            }
        }
        self.buf.extend_from_slice(segment.data);
        self.toggle = !self.toggle;
        self.done = segment.last;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdo::SdoAbortCode;

    #[test]
    fn cob_ids_follow_convention() {
        assert_eq!(request_cob_id(5), 0x605);
        assert_eq!(response_cob_id(5), 0x585);
    }

    #[test]
    fn upload_response_without_size_bit_is_accepted() {
        // BUGS.md M11: e=1, s=0 (command 0x43-style with s clear → 0x42 for
        // a u32) means "all four bytes are data" — python-canopen and
        // CANopenNode both accept it.
        let mut p = [0u8; 8];
        p[0] = SCS_UPLOAD_INITIATE | EXPEDITED; // e=1, s=0, n ignored
        p[1] = 0x00;
        p[2] = 0x20;
        let up = decode_upload_expedited_response(&p).unwrap();
        assert_eq!(up.data.len(), 4);
    }

    // --- Expedited download (golden bytes, oracle-checked) ------------------
    #[test]
    fn download_u32_matches_known_frame() {
        let f =
            encode_download_expedited(Address::new(0x2000, 0), &[0x78, 0x56, 0x34, 0x12]).unwrap();
        assert_eq!(f, [0x23, 0x00, 0x20, 0x00, 0x78, 0x56, 0x34, 0x12]);
    }

    #[test]
    fn download_u8_matches_known_frame() {
        let f = encode_download_expedited(Address::new(0x2001, 0), &[0x7F]).unwrap();
        assert_eq!(f, [0x2F, 0x01, 0x20, 0x00, 0x7F, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn download_i16_matches_known_frame() {
        // -2 as INTEGER16 = FE FF little-endian; command 0x2B = 2 data bytes.
        let f = encode_download_expedited(Address::new(0x6000, 1), &[0xFE, 0xFF]).unwrap();
        assert_eq!(f, [0x2B, 0x00, 0x60, 0x01, 0xFE, 0xFF, 0x00, 0x00]);
    }

    #[test]
    fn expedited_length_bounds_enforced() {
        let addr = Address::new(0x2000, 0);
        assert!(encode_download_expedited(addr, &[]).is_err());
        assert!(encode_download_expedited(addr, &[0; 5]).is_err());
        assert!(encode_upload_expedited_response(addr, &[0; 5]).is_err());
    }

    // --- Download/upload responses ------------------------------------------
    #[test]
    fn download_response_roundtrips() {
        let f = encode_download_response(Address::new(0x2000, 0));
        assert_eq!(f, [0x60, 0x00, 0x20, 0x00, 0, 0, 0, 0]);
        assert_eq!(
            decode_download_response(&f).unwrap(),
            Address::new(0x2000, 0)
        );
    }

    #[test]
    fn upload_request_matches_known_frame() {
        assert_eq!(
            encode_upload_request(Address::new(0x1000, 0)),
            [0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]
        );
    }

    #[test]
    fn upload_expedited_response_roundtrip() {
        // Known-good: device type 0x192 read back as UNSIGNED32.
        let f =
            encode_upload_expedited_response(Address::new(0x1000, 0), &[0x92, 0x01, 0x00, 0x00])
                .unwrap();
        assert_eq!(f, [0x43, 0x00, 0x10, 0x00, 0x92, 0x01, 0x00, 0x00]);
        let up = decode_upload_expedited_response(&f).unwrap();
        assert_eq!(up.addr, Address::new(0x1000, 0));
        assert_eq!(up.data, &[0x92, 0x01, 0x00, 0x00]);
    }

    // --- Abort ----------------------------------------------------------------
    #[test]
    fn abort_object_missing_matches_known_frame() {
        let f = encode_abort(Address::new(0x1000, 0), SdoAbortCode::ObjectDoesNotExist);
        assert_eq!(f, [0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x02, 0x06]);
        let (addr, code) = decode_abort(&f).unwrap();
        assert_eq!(addr, Address::new(0x1000, 0));
        assert_eq!(code, SdoAbortCode::ObjectDoesNotExist);
    }

    // --- Segmented --------------------------------------------------------------
    #[test]
    fn download_initiate_segmented_matches_known_frame() {
        let f = encode_download_initiate_segmented(Address::new(0x2000, 0), 20);
        assert_eq!(f, [0x21, 0x00, 0x20, 0x00, 20, 0, 0, 0]);
        assert_eq!(
            decode_download_initiate_segmented(&f).unwrap(),
            (Address::new(0x2000, 0), 20)
        );
    }

    #[test]
    fn upload_initiate_segmented_response_matches_known_frame() {
        let f = encode_upload_initiate_segmented_response(Address::new(0x2000, 0), 20);
        assert_eq!(f, [0x41, 0x00, 0x20, 0x00, 20, 0, 0, 0]);
        assert_eq!(
            decode_upload_initiate_response(&f).unwrap(),
            (Address::new(0x2000, 0), Some(20))
        );
        // Expedited frames must not decode here.
        let expedited = [0x43, 0x00, 0x10, 0x00, 0x92, 0x01, 0x00, 0x00];
        assert!(decode_upload_initiate_response(&expedited).is_err());
    }

    #[test]
    fn data_segment_frames_match_known_bytes() {
        let f = encode_data_segment(&[1, 2, 3, 4, 5, 6, 7], false, false).unwrap();
        assert_eq!(f, [0x00, 1, 2, 3, 4, 5, 6, 7]);

        // Final 3-byte segment, toggle 1: 0x19 = toggle | n=4<<1 | last.
        let f = encode_data_segment(&[0xAA, 0xBB, 0xCC], true, true).unwrap();
        assert_eq!(f, [0x19, 0xAA, 0xBB, 0xCC, 0, 0, 0, 0]);
        let seg = decode_data_segment(&f).unwrap();
        assert!(seg.toggle && seg.last && seg.data == [0xAA, 0xBB, 0xCC]);
    }

    #[test]
    fn segment_ack_and_poll_toggle_roundtrip() {
        assert_eq!(encode_download_segment_response(false)[0], 0x20);
        assert_eq!(encode_download_segment_response(true)[0], 0x30);
        assert!(decode_download_segment_response(&encode_download_segment_response(true)).unwrap());

        assert_eq!(encode_upload_segment_request(false)[0], 0x60);
        assert_eq!(encode_upload_segment_request(true)[0], 0x70);
        assert!(decode_upload_segment_request(&encode_upload_segment_request(true)).unwrap());
    }

    #[test]
    fn writer_reader_roundtrip() {
        let payload: Vec<u8> = (0..12u8).map(|i| 10 * (i + 1)).collect();
        let mut writer = SegmentWriter::new(payload.clone());
        let mut reader = SegmentBuffer::new();

        let f1 = writer.next_segment().unwrap();
        assert_eq!(f1[0] & TOGGLE, 0);
        reader
            .push(&decode_data_segment(&f1).unwrap(), None)
            .unwrap();

        let f2 = writer.next_segment().unwrap();
        assert_ne!(f2[0] & TOGGLE, 0);
        reader
            .push(&decode_data_segment(&f2).unwrap(), None)
            .unwrap();

        assert!(reader.is_done() && writer.next_segment().is_none());
        assert_eq!(reader.as_slice(), &payload[..]);
    }

    #[test]
    fn buffer_rejects_toggle_out_of_sequence() {
        let mut buf = SegmentBuffer::new();
        buf.push(
            &Segment {
                toggle: false,
                last: false,
                data: &[1, 2, 3],
            },
            None,
        )
        .unwrap();
        assert_eq!(
            buf.push(
                &Segment {
                    toggle: false,
                    last: true,
                    data: &[4, 5]
                },
                None,
            ),
            Err(Error::ToggleBit)
        );
    }
}
