//! SYNC (CiA 301 §7.2.5) and TIME (§7.2.6) frame codecs.
//!
//! SYNC frames travel on a configurable COB-ID (default `0x080`, register
//! 0x1005); the payload is empty or carries one counter byte when object
//! 0x1019 (counter overflow) is non-zero. TIME frames travel on `0x100` and
//! carry six bytes: milliseconds since midnight + days since 1984-01-01.

use crate::types::{Error, Result};

/// Default SYNC COB-ID from the pre-defined connection set.
pub const SYNC_DEFAULT_COB_ID: u16 = 0x080;
/// Default TIME COB-ID.
pub const TIME_COB_ID: u16 = 0x100;

/// Seconds between 1970-01-01 and 1984-01-01 (python-canopen's offset).
const UNIX_TO_1984_S: i64 = 441_763_200;
const MS_PER_DAY: i64 = 86_400_000;

/// Encodes a SYNC payload: empty, or one counter byte when the producer
/// enables counters (object 0x1019).
#[must_use]
pub const fn encode_sync(counter: Option<u8>) -> ([u8; 8], usize) {
    match counter {
        Some(c) => ([c, 0, 0, 0, 0, 0, 0, 0], 1),
        None => ([0; 8], 0),
    }
}

/// Extracts the SYNC counter, if the frame carries one.
#[must_use]
pub fn decode_sync(data: &[u8]) -> Option<Option<u8>> {
    match data.first() {
        None => Some(None), // zero-length payload
        Some(&c) if data.len() == 1 => Some(Some(c)),
        _ => None, // SYNC never carries more than one byte
    }
}

/// A decoded TIME stamp: absolute UTC time as `(unix_ms)` plus the raw parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimeStamp {
    /// Milliseconds within the day, restricted to the 28-bit TIME_OF_DAY
    /// domain (`0..=0x0FFF_FFFF`; `decode` masks the reserved high nibble).
    pub ms_of_day: u32,
    /// Days since 1984-01-01.
    pub days_since_1984: u16,
}

impl TimeStamp {
    /// Encodes the six-byte TIME payload.
    ///
    /// docs/DESIGN-DECISIONS.md SYN-09 ③: only the low 28 bits of TIME_OF_DAY are defined
    /// (CiA 301; CANopenNode masks on receive, `CO_TIME.c:163`), so the
    /// reserved high nibble is forced to zero on the wire instead of being
    /// echoed back as received.
    #[must_use]
    pub fn encode(&self) -> [u8; 8] {
        let mut out = [0u8; 8];
        out[..4].copy_from_slice(&(self.ms_of_day & 0x0FFF_FFFF).to_le_bytes());
        out[4..6].copy_from_slice(&self.days_since_1984.to_le_bytes());
        out
    }

    /// Decodes the six-byte TIME payload.
    ///
    /// docs/DESIGN-DECISIONS.md SYN-09 ③: TIME_OF_DAY's reserved high 4 bits are masked off
    /// exactly like CANopenNode (`CO_TIME.c:163`,
    /// `CO_SWAP_32(ms_swapped) & 0x0FFFFFFFU`), so a producer that leaves them
    /// set cannot push `ms_of_day` outside the 28-bit domain. The remaining
    /// fields are **not** range-checked (a day of 86 400 000 ms and a
    /// `days` value are both accepted and passed to the application), matching
    /// the reference, which only checks the payload length
    /// (`CO_TIME.c:39-49`).
    ///
    /// # Errors
    /// [`Error::BadLength`] unless exactly six bytes are supplied.
    pub fn decode(data: &[u8]) -> Result<Self> {
        if data.len() != 6 {
            return Err(Error::BadLength {
                declared: 6,
                actual: data.len(),
            });
        }
        Ok(Self {
            ms_of_day: u32::from_le_bytes([data[0], data[1], data[2], data[3]]) & 0x0FFF_FFFF,
            days_since_1984: u16::from_le_bytes([data[4], data[5]]),
        })
    }

    /// Converts from Unix epoch milliseconds.
    ///
    /// # Errors
    /// [`Error::ValueOutOfRange`] for timestamps before 1984-01-01 or after
    /// 1984-01-01 + 65536 days (≈ 2163), the representable TIME_OF_DAY range.
    pub fn from_unix_ms(unix_ms: i64) -> Result<Self> {
        let since_1984 = unix_ms - UNIX_TO_1984_S * 1000;
        if !(0..MS_PER_DAY * 0x1_0000).contains(&since_1984) {
            return Err(Error::ValueOutOfRange {
                value: unix_ms,
                min: UNIX_TO_1984_S * 1000,
                max: UNIX_TO_1984_S * 1000 + MS_PER_DAY * 0x1_0000,
            });
        }
        let days = (since_1984 / MS_PER_DAY) as u16;
        let ms = (since_1984 % MS_PER_DAY) as u32;
        Ok(Self {
            ms_of_day: ms,
            days_since_1984: days,
        })
    }

    /// Converts to Unix epoch milliseconds.
    #[must_use]
    pub fn to_unix_ms(&self) -> i64 {
        UNIX_TO_1984_S * 1000
            + i64::from(self.days_since_1984) * MS_PER_DAY
            + i64::from(self.ms_of_day)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sync_counter_roundtrip() {
        assert_eq!(decode_sync(&[]), Some(None));
        assert_eq!(decode_sync(&[42]), Some(Some(42)));
        assert!(decode_sync(&[1, 2]).is_none());
        let (data, len) = encode_sync(Some(7));
        assert_eq!(len, 1);
        assert_eq!(decode_sync(&data[..len]), Some(Some(7)));
    }

    #[test]
    fn time_known_epoch_conversion() {
        // 2000-01-01T00:00:00Z unix = 946684800 s → 5844 days since 1984.
        let ts = TimeStamp::from_unix_ms(946_684_800_000).unwrap();
        assert_eq!(ts.days_since_1984, 5844);
        assert_eq!(ts.ms_of_day, 0);
        assert_eq!(ts.to_unix_ms(), 946_684_800_000);
    }

    #[test]
    fn time_payload_roundtrip() {
        let ts = TimeStamp {
            ms_of_day: 43_210_000,
            days_since_1984: 12_345,
        };
        let payload = ts.encode();
        assert_eq!(payload.len(), 8);
        assert_eq!(TimeStamp::decode(&payload[..6]).unwrap(), ts);
        assert!(TimeStamp::decode(&payload[..5]).is_err());
    }

    #[test]
    fn time_rejects_out_of_range() {
        // Before 1984.
        assert!(TimeStamp::from_unix_ms(0).is_err());
    }
}
