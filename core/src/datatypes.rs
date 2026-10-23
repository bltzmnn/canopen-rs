//! CiA 301 data types and their little-endian wire codec.
//!
//! [`DataType`] discriminants are the CiA 301 data-type indices used inside
//! EDS files (`DataType=0x0007` etc.). [`Value`] is the typed payload stored
//! in object dictionary entries; all multi-byte encodings are little-endian
//! per the CANopen specification.

use crate::types::{Address, Error, Result};
use core::fmt;

/// CiA 301 basic data types. The discriminant equals the standard's
/// data-type index (e.g. `Unsigned32 = 0x0007`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
#[repr(u16)]
pub enum DataType {
    /// BOOLEAN (0x01), one byte on the wire (0/1).
    Boolean = 0x0001,
    /// INTEGER8 (0x02).
    Integer8 = 0x0002,
    /// INTEGER16 (0x03).
    Integer16 = 0x0003,
    /// INTEGER32 (0x04).
    Integer32 = 0x0004,
    /// UNSIGNED8 (0x05).
    Unsigned8 = 0x0005,
    /// UNSIGNED16 (0x06).
    Unsigned16 = 0x0006,
    /// UNSIGNED32 (0x07).
    Unsigned32 = 0x0007,
    /// REAL32 (0x08).
    Real32 = 0x0008,
    /// VISIBLE_STRING (0x09), variable length.
    VisibleString = 0x0009,
    /// OCTET_STRING (0x0A), variable length.
    OctetString = 0x000A,
    /// UNICODE_STRING (0x0B), UTF-16LE bytes, variable length.
    UnicodeString = 0x000B,
    /// TIME_OF_DAY (0x0C), fixed 6 bytes.
    TimeOfDay = 0x000C,
    /// TIME_DIFFERENCE (0x0D), fixed 6 bytes.
    TimeDifference = 0x000D,
    /// DOMAIN (0x0F), opaque bytes, variable length.
    Domain = 0x000F,
    /// INTEGER24 (0x10), 3 bytes sign-extended.
    Integer24 = 0x0010,
    /// REAL64 (0x11).
    Real64 = 0x0011,
    /// INTEGER40 (0x12), 5 bytes sign-extended.
    Integer40 = 0x0012,
    /// INTEGER48 (0x13), 6 bytes sign-extended.
    Integer48 = 0x0013,
    /// INTEGER56 (0x14), 7 bytes sign-extended.
    Integer56 = 0x0014,
    /// INTEGER64 (0x15).
    Integer64 = 0x0015,
    /// UNSIGNED24 (0x16), 3 bytes zero-extended.
    Unsigned24 = 0x0016,
    /// UNSIGNED40 (0x18), 5 bytes zero-extended.
    Unsigned40 = 0x0018,
    /// UNSIGNED48 (0x19), 6 bytes zero-extended.
    Unsigned48 = 0x0019,
    /// UNSIGNED56 (0x1A), 7 bytes zero-extended.
    Unsigned56 = 0x001A,
    /// UNSIGNED64 (0x1B).
    Unsigned64 = 0x001B,
}

impl DataType {
    /// Byte length for fixed-size types; `None` for variable-length ones.
    #[must_use]
    pub const fn fixed_size(self) -> Option<usize> {
        match self {
            Self::Boolean | Self::Integer8 | Self::Unsigned8 => Some(1),
            Self::Integer16 | Self::Unsigned16 => Some(2),
            Self::Integer24 | Self::Unsigned24 => Some(3),
            Self::Integer32 | Self::Unsigned32 | Self::Real32 => Some(4),
            Self::Integer40 | Self::Unsigned40 => Some(5),
            Self::TimeOfDay | Self::TimeDifference | Self::Integer48 | Self::Unsigned48 => Some(6),
            Self::Integer56 | Self::Unsigned56 => Some(7),
            Self::Real64 | Self::Integer64 | Self::Unsigned64 => Some(8),
            Self::VisibleString | Self::OctetString | Self::UnicodeString | Self::Domain => None,
        }
    }

    /// Whether values of this type have variable length.
    #[must_use]
    pub const fn is_variable(self) -> bool {
        self.fixed_size().is_none()
    }

    /// Whether this is a signed integer type (`INTEGERn`).
    #[must_use]
    pub const fn is_signed(self) -> bool {
        matches!(
            self,
            Self::Integer8
                | Self::Integer16
                | Self::Integer24
                | Self::Integer32
                | Self::Integer40
                | Self::Integer48
                | Self::Integer56
                | Self::Integer64
        )
    }

    /// Looks up a type by its CiA 301 index.
    #[must_use]
    pub fn from_index(index: u16) -> Option<Self> {
        Some(match index {
            0x0001 => Self::Boolean,
            0x0002 => Self::Integer8,
            0x0003 => Self::Integer16,
            0x0004 => Self::Integer32,
            0x0005 => Self::Unsigned8,
            0x0006 => Self::Unsigned16,
            0x0007 => Self::Unsigned32,
            0x0008 => Self::Real32,
            0x0009 => Self::VisibleString,
            0x000A => Self::OctetString,
            0x000B => Self::UnicodeString,
            0x000C => Self::TimeOfDay,
            0x000D => Self::TimeDifference,
            0x000F => Self::Domain,
            0x0010 => Self::Integer24,
            0x0011 => Self::Real64,
            0x0012 => Self::Integer40,
            0x0013 => Self::Integer48,
            0x0014 => Self::Integer56,
            0x0015 => Self::Integer64,
            0x0016 => Self::Unsigned24,
            0x0018 => Self::Unsigned40,
            0x0019 => Self::Unsigned48,
            0x001A => Self::Unsigned56,
            0x001B => Self::Unsigned64,
            _ => return None,
        })
    }

    /// Inclusive representable range as `(min, max)` for integer types.
    fn int_range(self) -> Option<(i128, i128)> {
        let (bits, signed) = match self {
            Self::Integer8 => (8, true),
            Self::Integer16 => (16, true),
            Self::Integer24 => (24, true),
            Self::Integer32 => (32, true),
            Self::Integer40 => (40, true),
            Self::Integer48 => (48, true),
            Self::Integer56 => (56, true),
            Self::Integer64 => (64, true),
            Self::Unsigned8 => (8, false),
            Self::Unsigned16 => (16, false),
            Self::Unsigned24 => (24, false),
            Self::Unsigned32 => (32, false),
            Self::Unsigned40 => (40, false),
            Self::Unsigned48 => (48, false),
            Self::Unsigned56 => (56, false),
            Self::Unsigned64 => (64, false),
            _ => return None,
        };
        if signed {
            let max = (1i128 << (bits - 1)) - 1;
            Some((-max - 1, max))
        } else {
            Some((0, (1i128 << bits) - 1))
        }
    }
}

impl fmt::Display for DataType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

/// Upper bound for variable-length values (VisibleString / OctetString /
/// UnicodeString / Domain), matching docs/IMPLEMENTATION.md §3.2. Decoding
/// and SDO segmented download enforce this so peers cannot grow buffers
/// without bound (`docs/DESIGN-DECISIONS.md` P2-15).
pub const MAX_VALUE_LEN: usize = 65535;

/// A typed value, as stored in an object dictionary entry or transferred
/// over SDO/PDO.
///
/// Wide integer types (INTEGER24/40/48/56 and unsigned counterparts) are
/// carried in the smallest fitting Rust primitive and encoded without
/// padding; decoding zero/sign-extends.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum Value {
    /// BOOLEAN.
    Bool(bool),
    /// INTEGER8 .. INTEGER64.
    Int(i64),
    /// UNSIGNED8 .. UNSIGNED64.
    Uint(u64),
    /// REAL32 / REAL64.
    Real(f64),
    /// VISIBLE_STRING.
    Str(String),
    /// OCTET_STRING / UNICODE_STRING / DOMAIN / TIME_* (raw bytes).
    Bytes(Vec<u8>),
}

impl Value {
    /// Number of bytes [`Value::encode_le`] writes: the *minimal* fitting
    /// representation for integers, fixed size otherwise.
    ///
    /// Use [`Value::encode_as`] when a specific entry type governs the wire
    /// width instead.
    #[must_use]
    pub fn encoded_len(&self) -> usize {
        match self {
            Value::Bool(_) => 1,
            Value::Int(v) => signed_width(*v),
            Value::Uint(v) => unsigned_width(*v),
            Value::Real(_) => 8,
            Value::Str(s) => s.len(),
            Value::Bytes(b) => b.len(),
        }
    }

    /// Number of bytes [`Value::encode_as`] writes for the given target
    /// type. Mostly identical to [`Value::encoded_len`], except when the
    /// target's wire representation is wider than the value's minimal one —
    /// currently only UNICODE_STRING, whose UTF-16LE encoding doubles (or
    /// more) a Rust `Str` (`docs/DESIGN-DECISIONS.md` L7).
    #[must_use]
    pub fn encoded_len_as(&self, dt: DataType) -> usize {
        if dt == DataType::UnicodeString {
            if let Value::Str(s) = self {
                return s.encode_utf16().count() * 2;
            }
        }
        self.encoded_len()
    }

    /// The default data type this value would be stored as.
    ///
    /// Note that narrow integer values can also be stored under narrower
    /// types; the OD entry's declared type always wins — this only fixes
    /// the default when no explicit type is known.
    #[must_use]
    pub fn data_type(&self) -> DataType {
        match self {
            Value::Bool(_) => DataType::Boolean,
            Value::Int(v) => match signed_width(*v) {
                1 => DataType::Integer8,
                2 => DataType::Integer16,
                3 => DataType::Integer24,
                4 => DataType::Integer32,
                5 => DataType::Integer40,
                6 => DataType::Integer48,
                7 => DataType::Integer56,
                _ => DataType::Integer64,
            },
            Value::Uint(v) => match unsigned_width(*v) {
                1 => DataType::Unsigned8,
                2 => DataType::Unsigned16,
                3 => DataType::Unsigned24,
                4 => DataType::Unsigned32,
                5 => DataType::Unsigned40,
                6 => DataType::Unsigned48,
                7 => DataType::Unsigned56,
                _ => DataType::Unsigned64,
            },
            Value::Real(_) => DataType::Real64,
            Value::Str(_) => DataType::VisibleString,
            Value::Bytes(_) => DataType::Domain,
        }
    }

    /// Encodes this value little-endian into `buf` using the minimal
    /// representation ([`Value::encoded_len`]).
    ///
    /// Returns the number of bytes written. Errors with
    /// [`Error::BufferTooSmall`] if the buffer cannot hold the encoding.
    pub fn encode_le(&self, buf: &mut [u8]) -> Result<usize> {
        let n = self.encoded_len();
        if buf.len() < n {
            return Err(Error::BufferTooSmall {
                needed: n,
                available: buf.len(),
            });
        }
        match self {
            Value::Bool(b) => buf[0] = u8::from(*b),
            Value::Int(v) => buf[..n].copy_from_slice(&v.to_le_bytes()[..n]),
            Value::Uint(v) => buf[..n].copy_from_slice(&v.to_le_bytes()[..n]),
            Value::Real(r) => buf[..8].copy_from_slice(&r.to_le_bytes()),
            Value::Str(s) => buf[..n].copy_from_slice(s.as_bytes()),
            Value::Bytes(b) => buf[..n].copy_from_slice(b),
        }
        Ok(n)
    }

    /// Encodes this value as if stored under `dt`, checking range/type.
    ///
    /// This is the entry-point used by the object dictionary where the
    /// entry's declared [`DataType`] governs the wire width: e.g.
    /// `Value::Uint(300)` fails under [`DataType::Unsigned8`] instead of
    /// silently truncating, and `Value::Int(-1)` fails under any unsigned
    /// type.
    pub fn encode_as(&self, dt: DataType, buf: &mut [u8]) -> Result<usize> {
        match self {
            Value::Bool(b) => match dt {
                DataType::Boolean => write_fixed(buf, &[u8::from(*b)]),
                _ => Err(type_mismatch(dt, self)),
            },
            Value::Uint(b @ (0 | 1)) if dt == DataType::Boolean => {
                // python-canopen accepts integer 0/1 for BOOLEAN (docs/DESIGN-DECISIONS.md
                // P3-36); keep the pyport surface equally lenient.
                write_fixed(buf, &[*b as u8])
            }
            Value::Int(v) => {
                if !matches!(
                    dt,
                    DataType::Integer8
                        | DataType::Integer16
                        | DataType::Integer24
                        | DataType::Integer32
                        | DataType::Integer40
                        | DataType::Integer48
                        | DataType::Integer56
                        | DataType::Integer64
                ) {
                    return Err(type_mismatch(dt, self));
                }
                let (min, max) = dt.int_range().expect("checked above");
                if i128::from(*v) < min || i128::from(*v) > max {
                    return Err(Error::ValueOutOfRange {
                        value: *v,
                        min: min.clamp(i64::MIN as i128, i64::MAX as i128) as i64,
                        max: max.clamp(i64::MIN as i128, i64::MAX as i128) as i64,
                    });
                }
                let n = dt.fixed_size().expect("integer types are fixed");
                write_le_n(buf, &v.to_le_bytes(), n)
            }
            Value::Uint(v) => {
                if !is_unsigned(dt) {
                    return Err(type_mismatch(dt, self));
                }
                let (min, max) = dt.int_range().expect("checked above");
                if i128::from(*v) > max || i128::from(*v) < min {
                    return Err(Error::ValueOutOfRange {
                        value: (*v).min(i64::MAX as u64) as i64,
                        min: 0,
                        max: max.clamp(i64::MIN as i128, i64::MAX as i128) as i64,
                    });
                }
                let n = dt.fixed_size().expect("unsigned types are fixed");
                write_le_n(buf, &v.to_le_bytes(), n)
            }
            Value::Real(r) => match dt {
                DataType::Real32 => {
                    let f = f32::from_bits(trunc_real32(*r));
                    write_fixed(buf, &f.to_le_bytes())
                }
                DataType::Real64 => write_fixed(buf, &r.to_le_bytes()),
                _ => Err(type_mismatch(dt, self)),
            },
            Value::Str(s) => match dt {
                DataType::VisibleString => write_fixed(buf, s.as_bytes()),
                // CiA 306 / python-canopen `encode_raw`: UNICODE_STRING is
                // UTF-16LE on the wire (docs/DESIGN-DECISIONS.md L7). Previously the Rust
                // string was written as UTF-8 bytes, which python-canopen
                // decodes as UTF-16LE — corrupting every non-ASCII
                // character exchanged with it. Callers must size `buf` with
                // [`Value::encoded_len_as`] (the UTF-16LE form is wider).
                DataType::UnicodeString => {
                    let mut tmp = [0u8; 2];
                    let mut written = 0usize;
                    for unit in s.encode_utf16() {
                        tmp.copy_from_slice(&unit.to_le_bytes());
                        written += write_fixed(&mut buf[written..], &tmp)?;
                    }
                    Ok(written)
                }
                _ => Err(type_mismatch(dt, self)),
            },
            Value::Bytes(b) => match dt {
                DataType::OctetString | DataType::UnicodeString | DataType::Domain => {
                    write_fixed(buf, b)
                }
                // TIME types are fixed 6-byte values (CANopenNode
                // CO_SDOserver.c validates the declared size for TIME);
                // short or long byte strings would not round-trip through
                // `decode_le`.
                DataType::TimeOfDay | DataType::TimeDifference => {
                    if b.len() != dt.fixed_size().unwrap_or(0) {
                        return Err(Error::BadLength {
                            declared: dt.fixed_size().unwrap_or(0),
                            actual: b.len(),
                        });
                    }
                    write_fixed(buf, b)
                }
                _ => Err(type_mismatch(dt, self)),
            },
        }
    }

    /// Decodes a value of type `dt` from little-endian `bytes`.
    ///
    /// Fixed-size types require `bytes.len()` to equal
    /// [`DataType::fixed_size`]; variable-length types accept any length up
    /// to [`MAX_VALUE_LEN`].
    pub fn decode_le(dt: DataType, bytes: &[u8]) -> Result<Self> {
        if let Some(size) = dt.fixed_size() {
            if bytes.len() != size {
                return Err(Error::BadLength {
                    declared: size,
                    actual: bytes.len(),
                });
            }
        } else if bytes.len() > MAX_VALUE_LEN {
            // Variable-length types are capped so a misbehaving peer cannot
            // grow transfer buffers without bound (docs/DESIGN-DECISIONS.md P2-15).
            return Err(Error::BadLength {
                declared: MAX_VALUE_LEN,
                actual: bytes.len(),
            });
        }
        Ok(match dt {
            DataType::Boolean => Value::Bool(bytes[0] != 0),
            DataType::Integer8 => Value::Int(i8::from_le_bytes([bytes[0]]) as i64),
            DataType::Integer16
            | DataType::Integer24
            | DataType::Integer32
            | DataType::Integer40
            | DataType::Integer48
            | DataType::Integer56
            | DataType::Integer64 => Value::Int(sign_extend_le(bytes)),
            DataType::Unsigned8 => Value::Uint(u64::from(bytes[0])),
            DataType::Unsigned16
            | DataType::Unsigned24
            | DataType::Unsigned32
            | DataType::Unsigned40
            | DataType::Unsigned48
            | DataType::Unsigned56
            | DataType::Unsigned64 => Value::Uint(zero_extend_le(bytes)),
            DataType::Real32 => Value::Real(f64::from(f32::from_le_bytes(
                bytes.try_into().expect("length checked above"),
            ))),
            DataType::Real64 => Value::Real(f64::from_le_bytes(
                bytes.try_into().expect("length checked above"),
            )),
            // CiA 306 declares VisibleString ASCII; bytes above 0x7F are
            // replaced per Rust's from_utf8_lossy and do not survive a
            // round trip — documented behaviour (`docs/DESIGN-DECISIONS.md` R10-P2-13).
            // C-based peers NUL-pad fixed-length strings; python strips the
            // trailing NULs (`ascii` + `errors="ignore"` + `rstrip("\x00")`,
            // `objectdictionary/__init__.py:448-450`), so do we (七E3).
            DataType::VisibleString => Value::Str(
                String::from_utf8_lossy(bytes)
                    .trim_end_matches('\0')
                    .to_string(),
            ),
            DataType::OctetString
            | DataType::Domain
            | DataType::TimeOfDay
            | DataType::TimeDifference => Value::Bytes(bytes.to_vec()),
            // UnicodeString stays a raw UTF-16LE byte blob in this stack
            // (八M5); strip trailing NUL *code units* like python's
            // `utf_16_le` decode + rstrip (`:451-454`), never a lone byte
            // (which would leave an odd-length string). docs/DESIGN-DECISIONS.md 七E3.
            DataType::UnicodeString => {
                let mut end = bytes.len();
                while end >= 2 && bytes[end - 2] == 0 && bytes[end - 1] == 0 {
                    end -= 2;
                }
                Value::Bytes(bytes[..end].to_vec())
            }
        })
    }
}

impl fmt::Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Value::Bool(b) => write!(f, "{b}"),
            Value::Int(v) => write!(f, "{v}"),
            Value::Uint(v) => write!(f, "{v}"),
            Value::Real(r) => write!(f, "{r}"),
            Value::Str(s) => write!(f, "{s:?}"),
            Value::Bytes(b) => {
                for byte in b {
                    write!(f, "{byte:02X}")?;
                }
                Ok(())
            }
        }
    }
}

const fn is_unsigned(dt: DataType) -> bool {
    matches!(
        dt,
        DataType::Unsigned8
            | DataType::Unsigned16
            | DataType::Unsigned24
            | DataType::Unsigned32
            | DataType::Unsigned40
            | DataType::Unsigned48
            | DataType::Unsigned56
            | DataType::Unsigned64
    )
}

/// Minimal byte width to represent `v` (two's complement, no padding).
const fn signed_width(v: i64) -> usize {
    if v >= -128 && v <= 127 {
        1
    } else if v >= -32_768 && v <= 32_767 {
        2
    } else if v >= -(1 << 23) && v < (1 << 23) {
        3
    } else if v >= -(1i64 << 31) && v < (1i64 << 31) {
        4
    } else if v >= -(1i64 << 39) && v < (1i64 << 39) {
        5
    } else if v >= -(1i64 << 47) && v < (1i64 << 47) {
        6
    } else if v >= -(1i64 << 55) && v < (1i64 << 55) {
        7
    } else {
        8
    }
}

/// Minimal byte width to represent `v` (no padding).
const fn unsigned_width(v: u64) -> usize {
    // Byte-exact boundaries: width n holds up to 2^(8n)-1 (docs/DESIGN-DECISIONS.md C1).
    match v.leading_zeros() / 8 {
        8 | 7 => 1,
        6 => 2,
        5 => 3,
        4 => 4,
        3 => 5,
        2 => 6,
        1 => 7,
        _ => 8,
    }
}

fn type_mismatch(expected: DataType, actual: &Value) -> Error {
    Error::TypeMismatch {
        addr: Address::new(0, 0),
        expected,
        actual: actual.data_type(),
    }
}

/// Writes exactly `buf.len()` bytes, or errors if the source is longer.
fn write_fixed(buf: &mut [u8], src: &[u8]) -> Result<usize> {
    if buf.len() < src.len() {
        return Err(Error::BufferTooSmall {
            needed: src.len(),
            available: buf.len(),
        });
    }
    buf[..src.len()].copy_from_slice(src);
    Ok(src.len())
}

/// Writes the low `n` bytes of a little-endian primitive buffer.
fn write_le_n(buf: &mut [u8], le_bytes: &[u8; 8], n: usize) -> Result<usize> {
    if buf.len() < n {
        return Err(Error::BufferTooSmall {
            needed: n,
            available: buf.len(),
        });
    }
    buf[..n].copy_from_slice(&le_bytes[..n]);
    Ok(n)
}

/// Truncates an f64 into f32 bit pattern.
///
/// Values beyond the f32 range silently saturate to ±infinity and NaN is
/// preserved — Rust's `as` semantics, matching CANopenNode's implicit
/// conversion (`docs/DESIGN-DECISIONS.md` P3-35).
fn trunc_real32(r: f64) -> u32 {
    (r as f32).to_bits()
}

/// Sign-extends a truncated little-endian slice into i64.
fn sign_extend_le(bytes: &[u8]) -> i64 {
    let mut v: u64 = 0;
    for (i, b) in bytes.iter().enumerate() {
        v |= u64::from(*b) << (8 * i);
    }
    let bits = bytes.len() * 8;
    if bits < 64 && bytes.last().is_some_and(|b| b & 0x80 != 0) {
        v |= u64::MAX << bits;
    }
    v as i64
}

/// Zero-extends a truncated little-endian slice into u64.
fn zero_extend_le(bytes: &[u8]) -> u64 {
    let mut v: u64 = 0;
    for (i, b) in bytes.iter().enumerate() {
        v |= u64::from(*b) << (8 * i);
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unsigned_width_boundaries_are_exact() {
        // docs/DESIGN-DECISIONS.md C1: each boundary must be the full-width max, not one
        // nibble short.
        assert_eq!(Value::Uint(0xFFFF_FFFF).encoded_len(), 4);
        assert_eq!(Value::Uint(0xFF_FFFF_FFFF).encoded_len(), 5);
        assert_eq!(Value::Uint(0x10_0000_0000).encoded_len(), 5);
        assert_eq!(Value::Uint(0xFFFF_FFFF_FFFF).encoded_len(), 6);
        assert_eq!(Value::Uint(0x1000_0000_0000).encoded_len(), 6); // 2^47
        assert_eq!(Value::Uint(0x0080_0000_0000_0000).encoded_len(), 7); // 2^55
        assert_eq!(Value::Uint(0x0100_0000_0000_0000).encoded_len(), 8); // 2^56
    }

    #[test]
    fn time_types_reject_non_six_byte_writes() {
        // docs/DESIGN-DECISIONS.md M8: TIME is a fixed 6-byte value; short/long byte strings
        // must not be accepted (they could not round-trip through decode_le).
        let mut buf = [0u8; 8];
        assert!(Value::Bytes(vec![1, 2, 3])
            .encode_as(DataType::TimeOfDay, &mut buf)
            .is_err());
        assert!(Value::Bytes(vec![1, 2, 3, 4, 5, 6, 7, 8])
            .encode_as(DataType::TimeDifference, &mut buf)
            .is_err());
        let six = Value::Bytes(vec![1, 2, 3, 4, 5, 6]);
        assert_eq!(six.encode_as(DataType::TimeOfDay, &mut buf).unwrap(), 6);
        assert_eq!(
            Value::decode_le(DataType::TimeOfDay, &[1, 2, 3, 4, 5, 6]).unwrap(),
            six
        );
    }

    #[test]
    fn fixed_sizes_match_cia301() {
        assert_eq!(DataType::Boolean.fixed_size(), Some(1));
        assert_eq!(DataType::Unsigned32.fixed_size(), Some(4));
        assert_eq!(DataType::Integer24.fixed_size(), Some(3));
        assert_eq!(DataType::Real64.fixed_size(), Some(8));
        assert_eq!(DataType::TimeOfDay.fixed_size(), Some(6));
        assert_eq!(DataType::VisibleString.fixed_size(), None);
        assert_eq!(DataType::Domain.fixed_size(), None);
    }

    #[test]
    fn type_index_round_trip() {
        for idx in [
            0x0001u16, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008, 0x0009, 0x000A,
            0x000B, 0x000C, 0x000D, 0x000F, 0x0010, 0x0011, 0x0012, 0x0013, 0x0014, 0x0015, 0x0016,
            0x0018, 0x0019, 0x001A, 0x001B,
        ] {
            let dt = DataType::from_index(idx).unwrap_or_else(|| panic!("missing {idx:#06x}"));
            assert_eq!(dt as u16, idx);
            assert_eq!(DataType::from_index(idx), Some(dt));
        }
        assert_eq!(DataType::from_index(0x000E), None);
        assert_eq!(DataType::from_index(0x0020), None);
    }

    #[test]
    fn encode_u32_golden_bytes() {
        // 0xDEADBEEF little-endian
        let mut buf = [0u8; 8];
        let n = Value::Uint(0xDEAD_BEEF)
            .encode_as(DataType::Unsigned32, &mut buf)
            .unwrap();
        assert_eq!(n, 4);
        assert_eq!(&buf[..4], &[0xEF, 0xBE, 0xAD, 0xDE]);
    }

    #[test]
    fn encode_i16_negative_golden_bytes() {
        // -2 => 0xFFFE little-endian
        let mut buf = [0u8; 4];
        let n = Value::Int(-2)
            .encode_as(DataType::Integer16, &mut buf)
            .unwrap();
        assert_eq!(n, 2);
        assert_eq!(&buf[..2], &[0xFE, 0xFF]);
    }

    #[test]
    fn wide_integers_encode_without_padding() {
        let mut buf = [0u8; 8];
        // INTEGER24: 0x123456 -> 56 34 12
        let n = Value::Uint(0x123_456)
            .encode_as(DataType::Unsigned24, &mut buf)
            .unwrap();
        assert_eq!(n, 3);
        assert_eq!(&buf[..3], &[0x56, 0x34, 0x12]);

        // INTEGER40 negative sign-extends on decode only
        let v = Value::Int(-1_000_000);
        let n = v.encode_as(DataType::Integer40, &mut buf).unwrap();
        assert_eq!(n, 5);
        assert_eq!(Value::decode_le(DataType::Integer40, &buf[..n]).unwrap(), v);

        // UNSIGNED48
        let v = Value::Uint(0xAB_CD_EF_01_23_45);
        let n = v.encode_as(DataType::Unsigned48, &mut buf).unwrap();
        assert_eq!(n, 6);
        assert_eq!(
            Value::decode_le(DataType::Unsigned48, &buf[..n]).unwrap(),
            v
        );
    }

    #[test]
    fn decode_sign_extends() {
        // INTEGER24 -1 => FF FF FF
        assert_eq!(
            Value::decode_le(DataType::Integer24, &[0xFF, 0xFF, 0xFF]).unwrap(),
            Value::Int(-1)
        );
        // INTEGER16 -32768 => 00 80
        assert_eq!(
            Value::decode_le(DataType::Integer16, &[0x00, 0x80]).unwrap(),
            Value::Int(i16::MIN as i64)
        );
    }

    #[test]
    fn range_checks_reject_overflow() {
        let mut buf = [0u8; 8];
        assert!(Value::Uint(300)
            .encode_as(DataType::Unsigned8, &mut buf)
            .is_err());
        assert!(Value::Uint(70_000)
            .encode_as(DataType::Unsigned16, &mut buf)
            .is_err());
        assert!(Value::Int(-1)
            .encode_as(DataType::Unsigned8, &mut buf)
            .is_err());
        assert!(Value::Int(130)
            .encode_as(DataType::Integer8, &mut buf)
            .is_err());
        // Boundary values pass.
        assert_eq!(
            Value::Uint(255)
                .encode_as(DataType::Unsigned8, &mut buf)
                .unwrap(),
            1
        );
        assert_eq!(
            Value::Int(-128)
                .encode_as(DataType::Integer8, &mut buf)
                .unwrap(),
            1
        );
    }

    #[test]
    fn real_types_round_trip() {
        let mut buf = [0u8; 8];
        let n = Value::Real(3.5)
            .encode_as(DataType::Real32, &mut buf)
            .unwrap();
        assert_eq!(n, 4);
        assert_eq!(
            Value::decode_le(DataType::Real32, &buf[..4]).unwrap(),
            Value::Real(3.5)
        );

        let n = Value::Real(-273.15f64)
            .encode_as(DataType::Real64, &mut buf)
            .unwrap();
        assert_eq!(n, 8);
        assert_eq!(
            Value::decode_le(DataType::Real64, &buf[..8]).unwrap(),
            Value::Real(-273.15)
        );
    }

    #[test]
    fn strings_and_bytes_round_trip() {
        let mut buf = [0u8; 64];
        let s = Value::Str("hello".into());
        assert_eq!(s.encode_le(&mut buf).unwrap(), 5);
        assert_eq!(
            Value::decode_le(DataType::VisibleString, &buf[..5]).unwrap(),
            s
        );

        let b = Value::Bytes(vec![0x01, 0x02, 0xFE]);
        assert_eq!(b.encode_as(DataType::Domain, &mut buf).unwrap(), 3);
        assert_eq!(Value::decode_le(DataType::Domain, &buf[..3]).unwrap(), b);
    }

    #[test]
    fn unicode_string_encodes_utf16le_like_python_canopen() {
        // docs/DESIGN-DECISIONS.md L7: UNICODE_STRING is UTF-16LE on the wire (CiA 306,
        // python-canopen `encode_raw` → `utf_16_le`); a Rust `Str` must be
        // transcoded, not written as UTF-8 bytes (which python-canopen
        // would decode as UTF-16LE, corrupting non-ASCII characters).
        let mut buf = [0u8; 64];
        let s = Value::Str("hé".into());
        // é is U+00E9 → UTF-16LE 0xE9 0x00.
        let n = s.encode_as(DataType::UnicodeString, &mut buf).unwrap();
        assert_eq!(n, 4);
        assert_eq!(&buf[..n], &[b'h', 0x00, 0xE9, 0x00]);
        // The minimal-length form differs from the UTF-8 byte count.
        assert_eq!(s.encoded_len(), 3);
        assert_eq!(s.encoded_len_as(DataType::UnicodeString), 4);

        // Multi-byte (astral) code point: U+1F600 → surrogate pair.
        let emoji = Value::Str("😀".into());
        let n = emoji.encode_as(DataType::UnicodeString, &mut buf).unwrap();
        assert_eq!(n, 4);
        assert_eq!(&buf[..n], &[0x3D, 0xD8, 0x00, 0xDE]); // UTF-16LE of D83D DE00

        // Raw bytes pass through verbatim (EDS hex defaults etc.).
        let raw = Value::Bytes(vec![0x41, 0x00, 0x42, 0x00]);
        let n = raw.encode_as(DataType::UnicodeString, &mut buf).unwrap();
        assert_eq!(n, 4);
        assert_eq!(&buf[..n], &[0x41, 0x00, 0x42, 0x00]);
    }

    #[test]
    fn bad_lengths_are_rejected_on_decode() {
        assert_eq!(
            Value::decode_le(DataType::Unsigned32, &[0x01, 0x02]),
            Err(Error::BadLength {
                declared: 4,
                actual: 2
            })
        );
    }

    /// docs/DESIGN-DECISIONS.md 七E3: python strips trailing NUL padding from C-style
    /// fixed-length strings (`rstrip("\x00")`,
    /// `objectdictionary/__init__.py:448-454`); the decoder must match.
    #[test]
    fn decoded_strings_strip_trailing_nul_padding() {
        assert_eq!(
            Value::decode_le(DataType::VisibleString, b"abc\0\0").unwrap(),
            Value::Str("abc".to_string())
        );
        assert_eq!(
            Value::decode_le(DataType::VisibleString, b"abc").unwrap(),
            Value::Str("abc".to_string())
        );
        // UTF-16LE "ab" padded with two NUL *code units*.
        assert_eq!(
            Value::decode_le(
                DataType::UnicodeString,
                &[0x61, 0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00]
            )
            .unwrap(),
            Value::Bytes(vec![0x61, 0x00, 0x62, 0x00])
        );
        // Trailing OCTET bytes are data, never stripped.
        assert_eq!(
            Value::decode_le(DataType::OctetString, &[0x01, 0x00, 0x00]).unwrap(),
            Value::Bytes(vec![0x01, 0x00, 0x00])
        );
    }

    #[test]
    fn buffer_too_small_is_reported() {
        let mut small = [0u8; 3];
        assert_eq!(
            Value::Uint(u64::MAX).encode_le(&mut small),
            Err(Error::BufferTooSmall {
                needed: 8,
                available: 3
            })
        );
    }

    #[test]
    fn minimal_width_matches_type_boundaries() {
        assert_eq!(signed_width(127), 1);
        assert_eq!(signed_width(128), 2);
        assert_eq!(signed_width(-128), 1);
        assert_eq!(signed_width(-129), 2);
        assert_eq!(signed_width(8_388_607), 3);
        assert_eq!(unsigned_width(0xFFFF_FFFF), 4);
        assert_eq!(unsigned_width(0x1_0000_0000), 5);
    }
}
