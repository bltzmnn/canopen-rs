//! Foundation types: errors, node IDs and object dictionary addresses.

use core::fmt;

/// Crate-wide result type.
pub type Result<T> = std::result::Result<T, Error>;

/// Highest CANopen node ID (used internally by LSS for "unconfigured").
pub const UNCONFIGURED_NODE_ID: u8 = 0xFF;
/// Highest valid node ID on the bus.
pub const MAX_NODE_ID: u8 = 127;

/// Errors produced by the protocol core.
///
/// Variants carry enough context to be mapped one-to-one onto SDO abort
/// codes where applicable (see [`Error::abort_code`]).
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    /// The addressed object (index/subindex) does not exist in the OD.
    #[error("object {index:#06x}:{subindex} does not exist")]
    ObjectDoesNotExist {
        /// Object index.
        index: u16,
        /// Object subindex.
        subindex: u8,
    },
    /// Access violates the entry's access type (read of write-only, write
    /// of read-only).
    #[error("access denied at {addr}")]
    AccessDenied {
        /// Address that was violated.
        addr: Address,
    },
    /// The value's data type does not match the entry's declared type.
    #[error("type mismatch at {addr}: expected {expected:?}, got {actual:?}")]
    TypeMismatch {
        /// Address written to.
        addr: Address,
        /// Data type expected by the OD entry.
        expected: crate::datatypes::DataType,
        /// Data type of the offending value.
        actual: crate::datatypes::DataType,
    },
    /// Byte length does not match the declared/expected length.
    #[error("length mismatch: declared {declared}, actual {actual}")]
    BadLength {
        /// Declared length in bytes.
        declared: usize,
        /// Actual length in bytes.
        actual: usize,
    },
    /// Destination buffer was too small for an encode operation.
    #[error("encode buffer too small: need {needed} bytes, have {available}")]
    BufferTooSmall {
        /// Required byte count.
        needed: usize,
        /// Provided capacity.
        available: usize,
    },
    /// Segmented transfer toggle bit out of sequence.
    #[error("toggle bit out of sequence")]
    ToggleBit,
    /// Block transfer CRC-16 check failed.
    #[error("block transfer CRC mismatch")]
    CrcMismatch,
    /// A frame was received that is invalid for the current state.
    #[error("unexpected frame in current state")]
    UnexpectedFrame,
    /// A numeric value is out of the allowed range.
    #[error("value {value} out of range [{min}, {max}]")]
    ValueOutOfRange {
        /// Offending value.
        value: i64,
        /// Inclusive minimum.
        min: i64,
        /// Inclusive maximum.
        max: i64,
    },
    /// A generic error with a message (last resort; prefer typed variants).
    #[error("{0}")]
    Other(&'static str),
    /// The remote SDO server aborted the transaction.
    #[error("SDO aborted: {0}")]
    SdoAborted(crate::sdo::SdoAbortCode),
    /// Host-side IO failure reported through a backend (e.g. storage file
    /// access); kept as a string so the core stays decoupled from `std::io`.
    #[error("io: {0}")]
    Io(String),
}

impl Error {
    /// Map this error onto the matching CiA 301 SDO abort code, if any.
    ///
    /// The mapping mirrors CANopenNode's `ODR_*` → abort-code table
    /// (see docs/REFERENCES.md S7): application-level validation failures
    /// surface to the SDO client as precise abort codes instead of a
    /// generic error.
    #[must_use]
    pub fn abort_code(&self) -> Option<crate::sdo::SdoAbortCode> {
        use crate::sdo::SdoAbortCode;
        match self {
            Error::ObjectDoesNotExist { .. } => Some(SdoAbortCode::ObjectDoesNotExist),
            Error::AccessDenied { .. } => Some(SdoAbortCode::AccessDeniedDueToLocalControl),
            Error::TypeMismatch { .. } | Error::BadLength { .. } => {
                Some(SdoAbortCode::TypeMismatchLengthLow)
            }
            Error::ToggleBit => Some(SdoAbortCode::ToggleNotAlternated),
            Error::CrcMismatch => Some(SdoAbortCode::GeneralError),
            Error::UnexpectedFrame | Error::BufferTooSmall { .. } => None,
            Error::ValueOutOfRange { value, min, .. } => {
                // Below the inclusive range → ValueTooLow; at/above the
                // maximum (or any other violation) → ValueTooHigh.
                let below = *value < *min;
                Some(if below {
                    SdoAbortCode::ValueTooLow
                } else {
                    SdoAbortCode::ValueTooHigh
                })
            }
            Error::SdoAborted(code) => Some(*code),
            Error::Io(_) | Error::Other(_) => Some(SdoAbortCode::GeneralError),
        }
    }
}

/// A CANopen node identifier (1..=127).
///
/// Stored raw so LSS can represent the unconfigured state (`0xFF`); use
/// [`NodeId::new`] / [`NodeId::get`] at application boundaries.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct NodeId(u8);

impl NodeId {
    /// Creates a node ID if `id` is in `1..=127`.
    #[must_use]
    pub fn new(id: u8) -> Option<Self> {
        (1..=MAX_NODE_ID).contains(&id).then_some(Self(id))
    }

    /// Raw ID value.
    #[must_use]
    pub const fn get(self) -> u8 {
        self.0
    }

    /// Raw bytes of an SDO request/response channel COB-ID base (`0x600 + id`).
    #[must_use]
    pub const fn sdo_rx_cob_id(self) -> u16 {
        0x600 + self.0 as u16
    }

    /// COB-ID of the server→client SDO channel (`0x580 + id`).
    #[must_use]
    pub const fn sdo_tx_cob_id(self) -> u16 {
        0x580 + self.0 as u16
    }

    /// COB-ID of heartbeat/boot-up frames (`0x700 + id`).
    #[must_use]
    pub const fn heartbeat_cob_id(self) -> u16 {
        0x700 + self.0 as u16
    }

    /// COB-ID of emergency frames (`0x80 + id`).
    #[must_use]
    pub const fn emcy_cob_id(self) -> u16 {
        0x80 + self.0 as u16
    }
}

impl fmt::Display for NodeId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl TryFrom<u8> for NodeId {
    type Error = Error;

    fn try_from(id: u8) -> Result<Self> {
        Self::new(id).ok_or(Error::Other("node id out of range 1..=127"))
    }
}

/// An address inside the object dictionary: 16-bit index + 8-bit subindex.
///
/// Ordering is `(index, subindex)` lexicographic, which matches the wire
/// order and makes `BTreeMap<Address, _>` iterate in dictionary order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Address {
    /// Object index (0x0000..=0xFFFF).
    pub index: u16,
    /// Subindex within the object (0..=255).
    pub subindex: u8,
}

impl Address {
    /// Creates a new address.
    #[must_use]
    pub const fn new(index: u16, subindex: u8) -> Self {
        Self { index, subindex }
    }
}

impl fmt::Display for Address {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:#06x}:{:02x}", self.index, self.subindex)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn value_out_of_range_picks_low_or_high_abort() {
        // BUGS.md m11: below the range → ValueTooLow, above → ValueTooHigh.
        let err = Error::ValueOutOfRange {
            value: -5,
            min: 0,
            max: 255,
        };
        assert_eq!(
            err.abort_code(),
            Some(crate::sdo::SdoAbortCode::ValueTooLow)
        );
        let err = Error::ValueOutOfRange {
            value: 300,
            min: 0,
            max: 255,
        };
        assert_eq!(
            err.abort_code(),
            Some(crate::sdo::SdoAbortCode::ValueTooHigh)
        );
    }

    #[test]
    fn node_id_range() {
        assert!(NodeId::new(0).is_none());
        assert!(NodeId::new(1).is_some());
        assert!(NodeId::new(127).is_some());
        assert!(NodeId::new(128).is_none());
    }

    #[test]
    fn cob_ids_match_predefined_connection_set() {
        let n = NodeId::new(5).unwrap();
        assert_eq!(n.sdo_rx_cob_id(), 0x605);
        assert_eq!(n.sdo_tx_cob_id(), 0x585);
        assert_eq!(n.heartbeat_cob_id(), 0x705);
        assert_eq!(n.emcy_cob_id(), 0x85);
    }

    #[test]
    fn address_orders_by_index_then_sub() {
        let mut v = vec![
            Address::new(0x1017, 0),
            Address::new(0x1000, 0),
            Address::new(0x1003, 2),
            Address::new(0x1003, 1),
        ];
        v.sort();
        assert_eq!(
            v,
            vec![
                Address::new(0x1000, 0),
                Address::new(0x1003, 1),
                Address::new(0x1003, 2),
                Address::new(0x1017, 0),
            ]
        );
    }
}
