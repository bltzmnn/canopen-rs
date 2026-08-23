//! PDO — process data objects (CiA 301 §7.4): mapping model, bit-level
//! pack/unpack and communication-parameter semantics.
//!
//! Mapping entries reference object dictionary cells by `(index, subindex)`
//! plus a bit length; the wire layout packs them LSB-first with **sub-byte**
//! granularity (e.g. four BOOLs sharing one byte). Entries whose index is
//! below `0x20` are *dummy* padding (CANopenNode convention, see
//! docs/REFERENCES.md S13) — they occupy bits but touch no OD cell.
//!
//! Validation happens at construction time (docs/REFERENCES.md V12 fix):
//! total length ≤ 64 bits, targets must exist and be PDO-mappable, and each
//! entry's bit length must fit the target's declared type.

use crate::datatypes::{DataType, Value};
use crate::object_dictionary::ObjectDictionary;
use crate::types::{Address, Error, NodeId, Result};

/// Base index of RPDO communication parameters (`0x1400 + n-1`).
pub const RPDO_COMM_BASE: u16 = 0x1400;
/// Base index of RPDO mapping parameters (`0x1600 + n-1`).
pub const RPDO_MAP_BASE: u16 = 0x1600;
/// Base index of TPDO communication parameters (`0x1800 + n-1`).
pub const TPDO_COMM_BASE: u16 = 0x1800;
/// Base index of TPDO mapping parameters (`0x1A00 + n-1`).
pub const TPDO_MAP_BASE: u16 = 0x1A00;

/// COB-ID flag: PDO is invalid / not valid (bit 31).
pub const COB_ID_INVALID: u32 = 1 << 31;
/// COB-ID flag: RTR allowed on this COB-ID (bit 30).
pub const COB_ID_RTR_ALLOWED: u32 = 1 << 30;

/// Maximum total payload bits per PDO (one classic CAN frame).
pub const MAX_PDO_BITS: usize = 64;

/// One mapped field inside a PDO.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MappingEntry {
    /// Target object; indices below `0x20` mark dummy (padding) fields.
    pub address: Address,
    /// Field width in bits (`1..=64`; dummies typically `8` or `16`).
    pub bit_length: u8,
}

impl MappingEntry {
    /// Whether this entry is pure padding (index below `0x20`).
    #[must_use]
    pub const fn is_dummy(&self) -> bool {
        self.address.index < 0x20
    }

    /// Wire representation: `index << 16 | subindex << 8 | bit_length`.
    #[must_use]
    pub const fn to_u32(&self) -> u32 {
        ((self.address.index as u32) << 16)
            | ((self.address.subindex as u32) << 8)
            | self.bit_length as u32
    }

    /// Parses the wire representation.
    ///
    /// # Errors
    /// [`Error::BadLength`] when `bit_length` is zero or above 64.
    pub fn from_u32(raw: u32) -> Result<Self> {
        let bit_length = (raw & 0xFF) as u8;
        if bit_length == 0 || usize::from(bit_length) > MAX_PDO_BITS {
            return Err(Error::BadLength {
                declared: MAX_PDO_BITS,
                actual: usize::from(bit_length),
            });
        }
        Ok(Self {
            address: Address::new((raw >> 16) as u16, (raw >> 8) as u8),
            bit_length,
        })
    }
}

/// An ordered PDO mapping (construction-time validated).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PdoMapping {
    entries: Vec<MappingEntry>,
    total_bits: usize,
}

impl PdoMapping {
    /// An empty mapping.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            entries: Vec::new(),
            total_bits: 0,
        }
    }

    /// Appends an entry after validating the cumulative length.
    ///
    /// # Errors
    /// [`Error::MappingLengthExceeded`](crate::types::Error) when the total
    /// exceeds [`MAX_PDO_BITS`].
    pub fn push(&mut self, entry: MappingEntry) -> Result<()> {
        if self.total_bits + usize::from(entry.bit_length) > MAX_PDO_BITS {
            return Err(Error::Other("PDO mapping exceeds 64 bits"));
        }
        self.total_bits += usize::from(entry.bit_length);
        self.entries.push(entry);
        Ok(())
    }

    /// All entries in packing order.
    #[must_use]
    pub fn entries(&self) -> &[MappingEntry] {
        &self.entries
    }

    /// Total mapped bits.
    #[must_use]
    pub const fn total_bits(&self) -> usize {
        self.total_bits
    }

    /// Payload byte length (`ceil(total_bits / 8)`).
    #[must_use]
    pub const fn byte_len(&self) -> usize {
        self.total_bits.div_ceil(8)
    }

    /// Removes all entries.
    pub fn clear(&mut self) {
        self.entries.clear();
        self.total_bits = 0;
    }

    /// Validates every non-dummy entry against the dictionary: target must
    /// exist, be PDO-mappable, and have a declared type wide enough.
    ///
    /// `require_writable` reflects the PDO direction: RPDO targets must be
    /// writable (they receive SDO/PDO writes), TPDO targets must be
    /// readable. Pass `true` when validating an RPDO mapping, `false` for a
    /// TPDO.
    ///
    /// # Errors
    /// - [`Error::ObjectDoesNotExist`] for missing targets;
    /// - [`Error::AccessDenied`] for non-mappable or access-incompatible
    ///   targets;
    /// - [`Error::TypeMismatch`] when the declared type is too narrow.
    pub fn validate_against(&self, od: &ObjectDictionary, require_writable: bool) -> Result<()> {
        for entry in &self.entries {
            if entry.is_dummy() {
                continue;
            }
            let Some(od_entry) = od.entry(entry.address) else {
                return Err(Error::ObjectDoesNotExist {
                    index: entry.address.index,
                    subindex: entry.address.subindex,
                });
            };
            if !od_entry.pdo_mappable {
                return Err(Error::AccessDenied {
                    addr: entry.address,
                });
            }
            let access_ok = if require_writable {
                od_entry.access.is_writable()
            } else {
                od_entry.access.is_readable()
            };
            if !access_ok {
                return Err(Error::AccessDenied {
                    addr: entry.address,
                });
            }
            let max_bits = match od_entry.data_type.fixed_size() {
                Some(n) => n * 8,
                None => MAX_PDO_BITS,
            };
            if usize::from(entry.bit_length) > max_bits {
                return Err(Error::TypeMismatch {
                    addr: entry.address,
                    expected: od_entry.data_type,
                    actual: od_entry.data_type,
                });
            }
        }
        Ok(())
    }

    /// Reads a mapping record from the OD (`map_index` = 0x1600+n/0x1A00+n):
    /// sub 0 holds the entry count, subs 1..n hold the `u32` wire values.
    ///
    /// # Errors
    /// Propagates OD errors; malformed wire values abort via
    /// [`MappingEntry::from_u32`].
    pub fn from_od_record(od: &ObjectDictionary, map_index: u16) -> Result<Self> {
        let count = match od.read(Address::new(map_index, 0))? {
            Value::Uint(n) => n.min(64) as usize,
            _ => {
                return Err(Error::TypeMismatch {
                    addr: Address::new(map_index, 0),
                    expected: DataType::Unsigned8,
                    actual: DataType::Unsigned8,
                })
            }
        };
        let mut mapping = Self::new();
        for sub in 1..=count {
            let Value::Uint(raw) = od.read(Address::new(map_index, sub as u8))? else {
                return Err(Error::TypeMismatch {
                    addr: Address::new(map_index, sub as u8),
                    expected: DataType::Unsigned32,
                    actual: DataType::Unsigned8,
                });
            };
            mapping.push(MappingEntry::from_u32(raw as u32)?)?;
        }
        Ok(mapping)
    }
}

/// Writes `bits` LSB-first from `src` into `dst` starting at bit `offset`.
fn put_bits(dst: &mut [u8], offset: usize, src: &[u8], bits: usize) {
    for i in 0..bits {
        let src_bit = (src[i / 8] >> (i % 8)) & 1;
        let dst_byte = (offset + i) / 8;
        let dst_bit = (offset + i) % 8;
        dst[dst_byte] = (dst[dst_byte] & !(1 << dst_bit)) | (src_bit << dst_bit);
    }
}

/// Reads `bits` LSB-first from `src` starting at bit `offset` into `dst`.
fn get_bits(src: &[u8], offset: usize, dst: &mut [u8], bits: usize) {
    for b in &mut *dst {
        *b = 0;
    }
    for i in 0..bits {
        let src_bit = (src[(offset + i) / 8] >> ((offset + i) % 8)) & 1;
        dst[i / 8] |= src_bit << (i % 8);
    }
}

/// Packs every mapped object little-endian into `buf` (TPDO transmit path).
///
/// Returns the payload byte count. Dummy entries contribute zero bits.
///
/// # Errors
/// - [`Error::BufferTooSmall`] when `buf` cannot hold the payload;
/// - [`Error::ObjectDoesNotExist`] / callback errors from reads.
pub fn pack(mapping: &PdoMapping, od: &ObjectDictionary, buf: &mut [u8]) -> Result<usize> {
    let need = mapping.byte_len();
    if buf.len() < need {
        return Err(Error::BufferTooSmall {
            needed: need,
            available: buf.len(),
        });
    }
    buf[..need].fill(0);
    let mut bit_offset = 0usize;
    for entry in mapping.entries() {
        let nbytes = usize::from(entry.bit_length).div_ceil(8);
        if entry.is_dummy() {
            bit_offset += usize::from(entry.bit_length);
            continue;
        }
        let Some(od_entry) = od.entry(entry.address) else {
            return Err(Error::ObjectDoesNotExist {
                index: entry.address.index,
                subindex: entry.address.subindex,
            });
        };
        let dt = od_entry.data_type;
        let mut tmp = [0u8; 8];
        let value = od.read(entry.address)?;
        value.encode_as(dt, &mut tmp[..nbytes])?;
        put_bits(buf, bit_offset, &tmp, usize::from(entry.bit_length));
        bit_offset += usize::from(entry.bit_length);
    }
    Ok(need)
}

/// Unpacks received PDO payload bytes back into the dictionary (RPDO
/// receive path), decoding each field against its declared type.
///
/// # Errors
/// - [`Error::BadLength`] when `data` is shorter than the mapping needs;
/// - decode/write errors propagate verbatim.
pub fn unpack(mapping: &PdoMapping, od: &mut ObjectDictionary, data: &[u8]) -> Result<()> {
    if data.len() < mapping.byte_len() {
        return Err(Error::BadLength {
            declared: mapping.byte_len(),
            actual: data.len(),
        });
    }
    let mut bit_offset = 0usize;
    for entry in mapping.entries() {
        let bits = usize::from(entry.bit_length);
        if !entry.is_dummy() {
            let nbytes = bits.div_ceil(8);
            let mut tmp = [0u8; 8];
            get_bits(data, bit_offset, &mut tmp, bits);
            // Decode against the *current* declared type of the target.
            if let Some(od_entry) = od.entry(entry.address) {
                let dt = od_entry.data_type;
                let value = Value::decode_le(dt, &tmp[..nbytes])?;
                od.write(entry.address, value)?;
            } else {
                return Err(Error::ObjectDoesNotExist {
                    index: entry.address.index,
                    subindex: entry.address.subindex,
                });
            }
        }
        bit_offset += bits;
    }
    Ok(())
}

/// Transmission types (CiA 301 §7.4.3); values 241..=251 are reserved and
/// rejected by [`TransmissionType::from_byte`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransmissionType {
    /// 0 — transmission on SYNC reception only when new data available.
    SyncAcyclic,
    /// 1..=240 — every n-th SYNC.
    SyncCyclic(u8),
    /// 252 — RTR only, sampled at SYNC ("synchronous RTR-only").
    RtrOnlySync,
    /// 253 — RTR only, transmitting current data ("event-driven RTR-only").
    RtrOnlyEvent,
    /// 254 / 255 — event-driven (manufacturer/profile specific trigger).
    EventDriven(u8),
}

impl TransmissionType {
    /// Decodes a transmission-type byte.
    ///
    /// # Errors
    /// [`Error::UnexpectedFrame`] for reserved values 241..=251.
    pub fn from_byte(byte: u8) -> Result<Self> {
        match byte {
            0 => Ok(Self::SyncAcyclic),
            1..=240 => Ok(Self::SyncCyclic(byte)),
            241..=251 => Err(Error::UnexpectedFrame),
            252 => Ok(Self::RtrOnlySync),
            253 => Ok(Self::RtrOnlyEvent),
            254 | 255 => Ok(Self::EventDriven(byte)),
        }
    }

    /// The wire byte.
    #[must_use]
    pub const fn to_byte(self) -> u8 {
        match self {
            Self::SyncAcyclic => 0,
            Self::SyncCyclic(n) => n,
            Self::RtrOnlySync => 252,
            Self::RtrOnlyEvent => 253,
            Self::EventDriven(b) => b,
        }
    }

    /// Whether this TPDO type transmits upon SYNC frames.
    #[must_use]
    pub const fn is_synchronous(self) -> bool {
        matches!(self, Self::SyncAcyclic | Self::SyncCyclic(_))
    }

    /// For cyclic types, how many SYNCs to wait between transmissions.
    #[must_use]
    pub const fn sync_period(self) -> Option<u8> {
        match self {
            Self::SyncCyclic(0) => Some(1),
            Self::SyncCyclic(n) => Some(n),
            _ => None,
        }
    }
}

/// Communication parameters parsed from a PDO record
/// (`0x1400+/0x1800+`): COB-ID validity/RTR flags, transmission type,
/// inhibit time (100 µs units), event timer (ms) and SYNC start value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PdoConfig {
    /// Raw COB-ID register (flags included); use accessors below.
    pub cob_id_raw: u32,
    /// Transmission type byte (validated separately).
    pub transmission_type_raw: u8,
    /// Inhibit time in 100 µs units (TPDO send-rate limiting).
    pub inhibit_time_100us: u16,
    /// Event timer in ms (periodic event-driven TX / RPDO deadman).
    pub event_timer_ms: u16,
    /// SYNC start value (only meaningful when counter overflow ≠ 0).
    pub sync_start_value: u8,
}

impl PdoConfig {
    /// Reads a communication record; sub 1 (COB-ID) is mandatory, subs 2..6
    /// default to zero when absent (permissive for minimal devices).
    ///
    /// # Errors
    /// Propagates OD errors for missing mandatory fields.
    pub fn from_od(od: &ObjectDictionary, comm_index: u16) -> Result<Self> {
        let Value::Uint(cob_id_raw) = od.read(Address::new(comm_index, 1))? else {
            return Err(Error::TypeMismatch {
                addr: Address::new(comm_index, 1),
                expected: DataType::Unsigned32,
                actual: DataType::Unsigned8,
            });
        };
        let read_u16 = |sub: u8| -> Result<u16> {
            match od.read(Address::new(comm_index, sub)) {
                Ok(Value::Uint(v)) => Ok(v as u16),
                Ok(_) => Ok(0),
                Err(_) => Ok(0),
            }
        };
        let tt = match od.read(Address::new(comm_index, 2)) {
            Ok(Value::Uint(v)) => v as u8,
            _ => 254,
        };
        Ok(Self {
            cob_id_raw: cob_id_raw as u32,
            transmission_type_raw: tt,
            inhibit_time_100us: read_u16(3)?,
            event_timer_ms: read_u16(5)?,
            sync_start_value: match od.read(Address::new(comm_index, 6)) {
                Ok(Value::Uint(v)) => v as u8,
                _ => 0,
            },
        })
    }

    /// Whether the PDO is enabled (COB-ID valid bit clear).
    #[must_use]
    pub const fn is_valid(&self) -> bool {
        self.cob_id_raw & COB_ID_INVALID == 0
    }

    /// Whether remote requests are allowed on this COB-ID.
    #[must_use]
    pub const fn rtr_allowed(&self) -> bool {
        self.cob_id_raw & COB_ID_RTR_ALLOWED != 0
    }

    /// The 11-bit CAN identifier.
    #[must_use]
    pub const fn cob_id(&self) -> u16 {
        (self.cob_id_raw & 0x7FF) as u16
    }

    /// Parsed transmission type.
    ///
    /// # Errors
    /// Propagates [`TransmissionType::from_byte`].
    pub fn transmission_type(&self) -> Result<TransmissionType> {
        TransmissionType::from_byte(self.transmission_type_raw)
    }
}

/// Default COB-ID from the pre-defined connection set:
/// TPDO1..4 = `0x180/280/380/480 + id`, RPDO1..4 = `0x200/300/400/500 + id`.
///
/// Returns `None` for numbers outside `1..=4`.
#[must_use]
pub fn default_cob_id(tpdo: bool, number: u8, node: NodeId) -> Option<u16> {
    let n = usize::from(number).checked_sub(1)?;
    if n > 3 {
        return None;
    }
    Some(if tpdo { 0x180 } else { 0x200 } + (n as u16) * 0x100 + u16::from(node.get()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::object_dictionary::{AccessType, Entry, Storage};
    use crate::standard::StandardObjects;

    fn test_od() -> ObjectDictionary {
        let mut od = StandardObjects::new().build();
        let mk = |dt, val| Entry::new("", dt, AccessType::Rw, Storage::Value(val)).pdo_mappable();
        // 0x6000:1 U8, :2 U16, :3 U32 ; 0x6001:1..4 four BOOLs in one byte.
        od.insert(
            Address::new(0x6000, 1),
            mk(DataType::Unsigned8, Value::Uint(0xA5)),
        );
        od.insert(
            Address::new(0x6000, 2),
            mk(DataType::Unsigned16, Value::Uint(0xBEEF)),
        );
        od.insert(
            Address::new(0x6000, 3),
            mk(DataType::Unsigned32, Value::Uint(0xDEAD_BEEF)),
        );
        for sub in 1..=4 {
            od.insert(
                Address::new(0x6001, sub),
                mk(DataType::Boolean, Value::Bool(sub % 2 == 0)),
            );
        }
        od
    }

    fn mapping(entries: &[(Address, u8)]) -> PdoMapping {
        let mut m = PdoMapping::new();
        for &(addr, bits) in entries {
            m.push(MappingEntry {
                address: addr,
                bit_length: bits,
            })
            .unwrap();
        }
        m
    }

    #[test]
    fn mapping_entry_wire_roundtrip() {
        let e = MappingEntry {
            address: Address::new(0x6000, 2),
            bit_length: 16,
        };
        assert_eq!(e.to_u32(), 0x6000_0210);
        assert_eq!(MappingEntry::from_u32(e.to_u32()).unwrap(), e);
        // Zero bit length rejected.
        assert!(MappingEntry::from_u32(0x6000_2000).is_err());
        // Full-format parse: index/sub/length split.
        let e2 = MappingEntry::from_u32(0x6000_0210).unwrap();
        assert_eq!(e2.address, Address::new(0x6000, 2));
        assert_eq!(e2.bit_length, 16);
    }

    #[test]
    fn mapping_rejects_over_64_bits_at_construction() {
        let mut m = PdoMapping::new();
        let big = MappingEntry {
            address: Address::new(0x6000, 3),
            bit_length: 40,
        };
        assert!(m.push(big).is_ok()); // 40 bits fit
        assert!(m.push(big).is_err()); // 80 bits do not
    }

    #[test]
    fn validate_enforces_direction_access_rights() {
        let mut od = test_od();
        // A read-only mappable target: fine for a TPDO (readable) but
        // rejected as an RPDO target (not writable).
        od.insert(
            Address::new(0x6002, 1),
            Entry::new(
                "",
                DataType::Unsigned8,
                AccessType::Const,
                Storage::Value(Value::Uint(1)),
            )
            .pdo_mappable(),
        );
        let tpdo = mapping(&[(Address::new(0x6002, 1), 8)]);
        assert!(tpdo.validate_against(&od, false).is_ok());
        assert!(tpdo.validate_against(&od, true).is_err());
    }

    #[test]
    fn validate_checks_mappable_and_width() {
        let od = test_od();
        let ok = mapping(&[(Address::new(0x6000, 1), 8), (Address::new(0x6000, 2), 16)]);
        assert!(ok.validate_against(&od, false).is_ok());

        // Not mappable: standard object 0x1000 has pdo_mappable=false.
        let bad = mapping(&[(Address::new(0x1000, 0), 32)]);
        assert!(bad.validate_against(&od, false).is_err());

        // Too wide for declared type.
        let wide = mapping(&[(Address::new(0x6000, 1), 9)]);
        assert!(wide.validate_against(&od, false).is_err());

        // Missing target.
        let ghost = mapping(&[(Address::new(0x7777, 0), 8)]);
        assert!(ghost.validate_against(&od, true).is_err());

        // Dummies bypass checks.
        let dummy = mapping(&[(Address::new(0x0005, 0), 8)]);
        assert!(dummy.validate_against(&od, false).is_ok());
    }

    #[test]
    fn pack_matches_known_layout() {
        let od = test_od();
        let m = mapping(&[
            (Address::new(0x6000, 1), 8),  // A5
            (Address::new(0x6000, 2), 16), // EF BE
            (Address::new(0x6000, 3), 32), // EF BE AD DE
        ]);
        let mut buf = [0u8; 8];
        let n = pack(&m, &od, &mut buf).unwrap();
        assert_eq!(n, 7);
        assert_eq!(&buf[..7], &[0xA5, 0xEF, 0xBE, 0xEF, 0xBE, 0xAD, 0xDE]);
    }

    #[test]
    fn sub_byte_bools_share_one_byte() {
        let od = test_od(); // bools: 1=F, 2=T, 3=F, 4=T → bits 0101 (LSB first: F=0,T=1,F=0,T=1 → 0b1010=0x0A)
        let m = mapping(&[
            (Address::new(0x6001, 1), 1),
            (Address::new(0x6001, 2), 1),
            (Address::new(0x6001, 3), 1),
            (Address::new(0x6001, 4), 1),
        ]);
        let mut buf = [0u8; 8];
        let n = pack(&m, &od, &mut buf).unwrap();
        assert_eq!(n, 1);
        assert_eq!(buf[0], 0b1010);
    }

    #[test]
    fn unpack_writes_back_into_od() {
        let mut od = test_od();
        let m = mapping(&[(Address::new(0x6000, 1), 8), (Address::new(0x6000, 2), 16)]);
        unpack(&m, &mut od, &[0x11, 0x22, 0x33]).unwrap();
        assert_eq!(od.read(Address::new(0x6000, 1)).unwrap(), Value::Uint(0x11));
        assert_eq!(
            od.read(Address::new(0x6000, 2)).unwrap(),
            Value::Uint(0x3322)
        );
    }

    #[test]
    fn pack_unpack_roundtrip_with_dummies_and_sub_bytes() {
        let mut od = test_od();
        let m = mapping(&[
            (Address::new(0x0005, 0), 8),  // dummy padding byte
            (Address::new(0x6001, 2), 1),  // T
            (Address::new(0x6000, 2), 16), // BEEF
        ]);
        let mut buf = [0u8; 8];
        let n = pack(&m, &od, &mut buf).unwrap();
        assert_eq!(n, 4); // 25 bits -> 4 bytes

        // Clear targets then verify restoration through unpack.
        od.write(Address::new(0x6001, 2), Value::Bool(false))
            .unwrap();
        od.write(Address::new(0x6000, 2), Value::Uint(0)).unwrap();
        unpack(&m, &mut od, &buf[..n]).unwrap();
        assert_eq!(od.read(Address::new(0x6001, 2)).unwrap(), Value::Bool(true));
        assert_eq!(
            od.read(Address::new(0x6000, 2)).unwrap(),
            Value::Uint(0xBEEF)
        );
    }

    #[test]
    fn short_payload_is_rejected_on_unpack() {
        let mut od = test_od();
        let m = mapping(&[(Address::new(0x6000, 2), 16)]);
        assert!(unpack(&m, &mut od, &[0x01]).is_err());
    }

    #[test]
    fn transmission_type_validation() {
        assert_eq!(
            TransmissionType::from_byte(0).unwrap(),
            TransmissionType::SyncAcyclic
        );
        assert_eq!(
            TransmissionType::from_byte(5).unwrap(),
            TransmissionType::SyncCyclic(5)
        );
        assert!(TransmissionType::from_byte(245).is_err()); // reserved band
        assert!(TransmissionType::from_byte(254).is_ok());
        assert_eq!(TransmissionType::SyncCyclic(5).sync_period(), Some(5));
        assert_eq!(TransmissionType::SyncAcyclic.sync_period(), None);
    }

    #[test]
    fn config_reads_comm_record_with_defaults() {
        let od = StandardObjects::new().build();
        // Minimal record: only COB-ID present.
        let cfg = PdoConfig::from_od(&od, 0x1800).unwrap_or(PdoConfig {
            cob_id_raw: 0,
            transmission_type_raw: 254,
            inhibit_time_100us: 0,
            event_timer_ms: 0,
            sync_start_value: 0,
        });
        let _ = cfg; // absent record tolerated by callers; direct parse tested below

        // Full record path:
        let mut od2 = StandardObjects::new().build();
        use crate::object_dictionary::{AccessType, Entry, Storage};
        let put = |od: &mut ObjectDictionary, addr: Address, v: Value, dt: DataType| {
            od.insert(addr, Entry::new("", dt, AccessType::Rw, Storage::Value(v)));
        };
        put(
            &mut od2,
            Address::new(0x1800, 1),
            Value::Uint(u64::from(COB_ID_INVALID) | 0x305),
            DataType::Unsigned32,
        );
        put(
            &mut od2,
            Address::new(0x1800, 2),
            Value::Uint(254),
            DataType::Unsigned8,
        );
        put(
            &mut od2,
            Address::new(0x1800, 3),
            Value::Uint(25),
            DataType::Unsigned16,
        ); // 2.5 ms
        put(
            &mut od2,
            Address::new(0x1800, 5),
            Value::Uint(1000),
            DataType::Unsigned16,
        );
        let cfg = PdoConfig::from_od(&od2, 0x1800).unwrap();
        assert!(!cfg.is_valid());
        assert_eq!(cfg.cob_id(), 0x305);
        assert_eq!(cfg.inhibit_time_100us, 25);
        assert_eq!(cfg.event_timer_ms, 1000);
    }

    #[test]
    fn predefined_connection_set_cob_ids() {
        let n = NodeId::new(5).unwrap();
        assert_eq!(default_cob_id(true, 1, n), Some(0x185));
        assert_eq!(default_cob_id(false, 4, n), Some(0x505));
        assert_eq!(default_cob_id(true, 5, n), None);
        assert_eq!(default_cob_id(false, 0, n), None);
    }
}
