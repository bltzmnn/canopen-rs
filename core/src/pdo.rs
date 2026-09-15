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

/// Validates an SDO write into a PDO communication record (`0x1400..=0x15FF`
/// RPDO / `0x1800..=0x19FF` TPDO, subindex 1 = COB-ID) before it reaches the
/// dictionary. The CiA 301 bit-layout rule is enforced here
/// (`CO_PDO.c:503-509`, `BUGS.md` P2-5): bits 29..11 must be clear (11-bit
/// CAN-ID space), and identifier 0 (the NMT command COB) is rejected.
///
/// The stricter CANopenNode write-time rules — PDO must be disabled before
/// its mapping changes, and a COB-ID cannot be enabled while the mapping is
/// empty (`ODR_UNSUPP_ACCESS` / `ODR_INVALID_VALUE`) — are deliberately NOT
/// applied here: canopen-rs rebuilds PDO slots live on every write in the
/// `0x1400..=0x1BFF` range (IMPLEMENTATION §7.3), so masters may configure
/// in either order. The empty-mapping case is instead caught at runtime
/// assembly and reported via the bounded config-error ring (`BUGS.md` 八M2).
///
/// Writes anywhere else pass through untouched.
pub fn guard_comm_record_write(addr: Address, value: &Value) -> Result<()> {
    let in_comm_range = (RPDO_COMM_BASE..=0x15FF).contains(&addr.index)
        || (TPDO_COMM_BASE..=0x19FF).contains(&addr.index);
    if !in_comm_range || addr.subindex != 1 {
        return Ok(());
    }
    let Value::Uint(raw) = value else {
        return Ok(()); // type checking happens in the OD layer
    };
    let raw = *raw as u32;
    if raw & 0x3FFF_F800 != 0 {
        return Err(Error::ValueOutOfRange {
            value: i64::from(raw),
            min: 0,
            max: 0x3FFF_F7FF,
        });
    }
    if raw & COB_ID_INVALID == 0 && raw & 0x7FF == 0 {
        // CAN-ID 0 is the NMT command channel — never a legal PDO
        // identifier (BUGS.md 八M1; CANopenNode rejects it at write time,
        // `CO_PDO.c:507`, with `ODR_INVALID_VALUE` — 十L2).
        return Err(Error::InvalidValue);
    }
    Ok(())
}

/// CiA 301 restricted CAN-IDs, mirroring CANopenNode's
/// `CO_IS_RESTRICTED_CAN_ID` (`CO_driver.h:420-422`): identifiers that are
/// reserved for NMT/SDO/heartbeat and must not be used for SYNC, TIME,
/// EMCY or PDO.
#[must_use]
pub const fn is_restricted_can_id(can_id: u16) -> bool {
    can_id <= 0x7F
        || (can_id >= 0x101 && can_id <= 0x180)
        || (can_id >= 0x581 && can_id <= 0x5FF)
        || (can_id >= 0x601 && can_id <= 0x67F)
        || (can_id >= 0x6E0 && can_id <= 0x6FF)
        || can_id >= 0x701
}

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
/// COB-ID flag: RTR *not* allowed on this COB-ID (bit 30).
///
/// CiA 301 §7.4 / python-canopen `pdo/base.py:22-23`: bit 30 *set* forbids
/// RTR on the PDO; a clear bit means RTR is allowed (BUGS.md 八H1).
pub const COB_ID_RTR_NOT_ALLOWED: u32 = 1 << 30;

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
    /// [`Error::Other`](crate::types::Error) when the total exceeds
    /// [`MAX_PDO_BITS`].
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
    /// - [`Error::TypeMismatch`] when the declared type is too narrow;
    /// - [`Error::Other`] when two non-dummy entries target the same object
    ///   (duplicate mapping).
    pub fn validate_against(&self, od: &ObjectDictionary, require_writable: bool) -> Result<()> {
        // No two non-dummy entries may target the same object: pack/unpack
        // would write the same bytes twice with undefined ordering
        // (`BUGS.md` P2-6, docs/IMPLEMENTATION.md §7.3).
        let mut seen = std::collections::BTreeSet::new();
        for entry in &self.entries {
            if entry.is_dummy() {
                continue;
            }
            if !seen.insert(entry.address) {
                return Err(Error::Other(
                    "PDO mapping: duplicate mapping target address",
                ));
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
                    read: !require_writable,
                });
            }
            let access_ok = if require_writable {
                od_entry.access.is_writable()
            } else {
                od_entry.access.is_readable()
            };
            if !access_ok {
                // TPDO reads the object (require_writable=false is a TPDO);
                // RPDO writes it.
                return Err(Error::AccessDenied {
                    addr: entry.address,
                    read: !require_writable,
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
            // A signed type narrower than its declared width cannot be
            // sign-extended on decode (the high bits would read as zero and
            // negatives turn positive) — reject like most stacks do. Unsigned
            // truncation stays legal (BUGS.md P2-1).
            if let Some(full_bits) = od_entry.data_type.fixed_size().map(|n| n * 8) {
                if od_entry.data_type.is_signed() && usize::from(entry.bit_length) != full_bits {
                    return Err(Error::TypeMismatch {
                        addr: entry.address,
                        expected: od_entry.data_type,
                        actual: od_entry.data_type,
                    });
                }
                // BUGS.md M3: 浮点/时间类型不支持部分宽度映射——解包时
                // `get_bits` 只填低位、高位补零，`decode_le` 会解出高位清零的
                // 垃圾值并**无报错**写入 OD（签名整数已在此前拒绝）。仅允许完整
                // 宽度（Bool 的 1 位子字节映射属正常用法，不在此列；Domain 是
                // 变长类型，不进入此分支）。
                let is_real_or_time = matches!(
                    od_entry.data_type,
                    DataType::Real32
                        | DataType::Real64
                        | DataType::TimeOfDay
                        | DataType::TimeDifference
                );
                if is_real_or_time && usize::from(entry.bit_length) != full_bits {
                    return Err(Error::TypeMismatch {
                        addr: entry.address,
                        expected: od_entry.data_type,
                        actual: od_entry.data_type,
                    });
                }
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
        // Encode at the FULL declared width first (`encode_as` always writes
        // exactly `fixed_size()` bytes), then let `put_bits` copy only the
        // mapped bits. Encoding into a truncated `nbytes` slice failed for
        // every legal unsigned sub-width mapping such as U16@8
        // (BUGS.md R10-P1-6).
        if dt.fixed_size().is_some() {
            value.encode_as(dt, &mut tmp)?;
        } else {
            value.encode_as(dt, &mut tmp[..nbytes])?;
        }
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
/// - the **first** decode/write error is returned after the remaining
///   entries have still been applied — one broken entry must not discard
///   the rest of the frame (`BUGS.md` H2).
pub fn unpack(mapping: &PdoMapping, od: &mut ObjectDictionary, data: &[u8]) -> Result<()> {
    if data.len() < mapping.byte_len() {
        return Err(Error::BadLength {
            declared: mapping.byte_len(),
            actual: data.len(),
        });
    }
    let mut bit_offset = 0usize;
    let mut first_err: Option<Error> = None;
    for entry in mapping.entries() {
        let bits = usize::from(entry.bit_length);
        if !entry.is_dummy() {
            let nbytes = bits.div_ceil(8);
            let mut tmp = [0u8; 8];
            get_bits(data, bit_offset, &mut tmp, bits);
            // Decode against the *current* declared type of the target. For
            // fixed-width types decode from the FULL width: `get_bits` only
            // fills the low `bits` bits and the rest of `tmp` is zero, which
            // is exactly the zero-extension a narrow unsigned mapping means
            // (U16@8 previously demanded an impossible 1-byte buffer,
            // BUGS.md R10-P1-6).
            let result = (|| -> Result<()> {
                let Some(od_entry) = od.entry(entry.address) else {
                    return Err(Error::ObjectDoesNotExist {
                        index: entry.address.index,
                        subindex: entry.address.subindex,
                    });
                };
                let dt = od_entry.data_type;
                let use_len = dt.fixed_size().unwrap_or(nbytes).min(8);
                let value = Value::decode_le(dt, &tmp[..use_len])?;
                // BUGS.md M3: a partial-width mapping (fewer bits than the
                // object's full width, e.g. U16@8) must merge into the
                // existing value instead of overwriting it — decoding from
                // the full width zero-extends the unmapped high bits, so a
                // wholesale write would clobber them on every reception.
                let full_bits = dt.fixed_size().map_or(bits, |n| n * 8);
                let merged = if bits < full_bits {
                    match (&value, od_entry.raw_read(entry.address)?) {
                        (Value::Uint(inc), Value::Uint(cur)) => {
                            // BUGS.md L4: `bits` 受 `bits < full_bits` 门控必
                            // 小于 64，此处仍显式避免 64 位移位 UB。
                            let mask = if bits >= 64 {
                                u64::MAX
                            } else {
                                (1u64 << bits) - 1
                            };
                            Value::Uint((cur & !mask) | (*inc & mask))
                        }
                        (Value::Int(inc), Value::Int(cur)) => {
                            // Two's-complement bit merge; signed partial
                            // mappings are rejected by `validate_against`,
                            // this arm is defensive only.
                            let mask = if bits >= 64 {
                                u64::MAX
                            } else {
                                (1u64 << bits) - 1
                            };
                            Value::Int(((cur as u64 & !mask) | (*inc as u64 & mask)) as i64)
                        }
                        _ => value, // Bool/Real/Str/Bytes have no mergeable bits
                    }
                } else {
                    value
                };
                od.write(entry.address, merged)?;
                Ok(())
            })();
            if let Err(e) = result {
                // Remember the first failure but keep applying the other
                // entries of this frame (`BUGS.md` H2).
                if first_err.is_none() {
                    first_err = Some(e);
                }
            }
        }
        bit_offset += bits;
    }
    match first_err {
        Some(e) => Err(e),
        None => Ok(()),
    }
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

    /// Whether this TPDO type transmits **upon SYNC frames** — the trigger
    /// used by the synchronous scheduling loop.
    ///
    /// Note (`BUGS.md` H2): CiA 301 §7.4.3 groups type 252 (RtrOnlySync)
    /// under "synchronous" because its *sample* is the last SYNC value, but
    /// its *transmission trigger* is a remote request, not the SYNC frame —
    /// so it deliberately returns `false` here and is serviced separately
    /// via `on_remote_request` (which requires the first SYNC of the
    /// session before answering).
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
    /// default to zero when absent (permissive for minimal devices) — with
    /// one deliberate exception: sub 2 (transmission type) falls back to
    /// `tt_default`, which the caller supplies per direction (RPDO → 0 =
    /// synchronous, TPDO → 254 = event-driven; CiA 301 `BUGS.md` R10-P2-12).
    ///
    /// # Errors
    /// Propagates OD errors for missing mandatory fields.
    pub fn from_od(od: &ObjectDictionary, comm_index: u16, tt_default: u8) -> Result<Self> {
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
                // 错误类型显式报错（与 sub 2 的 `TypeMismatch` 行为一致，
                // BUGS.md P2-2），而非静默降级为 0；缺失（Err）仍按 0 默认。
                Ok(other) => Err(Error::TypeMismatch {
                    addr: Address::new(comm_index, sub),
                    expected: DataType::Unsigned16,
                    actual: other.data_type(),
                }),
                Err(_) => Ok(0),
            }
        };
        // Missing sub 2 adopts the caller's direction default; an *invalid*
        // value (reserved band 241..=251) still flows through
        // `transmission_type()` as an error at the caller; a present-but-
        // wrongly-typed sub 2 is a configuration error, not a silent default
        // (BUGS.md #29).
        let tt = match od.read(Address::new(comm_index, 2)) {
            Ok(Value::Uint(v)) => v as u8,
            Err(_) => tt_default,
            Ok(_) => {
                return Err(Error::TypeMismatch {
                    addr: Address::new(comm_index, 2),
                    expected: DataType::Unsigned8,
                    actual: DataType::Unsigned8,
                })
            }
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

    /// Whether remote requests are allowed on this COB-ID (CiA 301 §7.4:
    /// bit 30 *clear* allows RTR, *set* forbids it — BUGS.md 八H1).
    #[must_use]
    pub const fn rtr_allowed(&self) -> bool {
        self.cob_id_raw & COB_ID_RTR_NOT_ALLOWED == 0
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
    fn validate_rejects_real_partial_width_mapping() {
        // BUGS.md M3：浮点/时间类型不支持部分宽度映射——解包时 `get_bits`
        // 只填低位、高位补零，`decode_le` 会解出高位清零的垃圾值并**无报错**
        // 写入 OD。完整宽度（32/64 位）仍合法，部分宽度（如 REAL32@16）被拒绝。
        let mut od = test_od();
        let mk = |dt, val| Entry::new("", dt, AccessType::Rw, Storage::Value(val)).pdo_mappable();
        od.insert(
            Address::new(0x6003, 1),
            mk(DataType::Real32, Value::Real(1.0)),
        );
        od.insert(
            Address::new(0x6004, 1),
            mk(DataType::TimeOfDay, Value::Bytes(vec![1, 2, 3, 4, 5, 6])),
        );

        // REAL32@16（部分宽度）→ 拒绝。
        let partial = mapping(&[(Address::new(0x6003, 1), 16)]);
        assert!(partial.validate_against(&od, false).is_err());

        // REAL32@32（完整宽度）→ 合法。
        let full = mapping(&[(Address::new(0x6003, 1), 32)]);
        assert!(full.validate_against(&od, false).is_ok());

        // TIME_OF_DAY@24（部分宽度）→ 拒绝。
        let partial_time = mapping(&[(Address::new(0x6004, 1), 24)]);
        assert!(partial_time.validate_against(&od, false).is_err());
    }

    #[test]
    fn validate_rejects_signed_partial_width_mapping() {
        // BUGS.md P2-1: a signed type mapped at less than its declared width
        // cannot be sign-extended on decode (narrow field would read as
        // positive), so `validate_against` rejects it — unsigned truncation
        // stays legal.
        let mut od = test_od();
        let mk = |dt, val| Entry::new("", dt, AccessType::Rw, Storage::Value(val)).pdo_mappable();
        od.insert(
            Address::new(0x6002, 1),
            mk(DataType::Integer16, Value::Int(-1)),
        );

        // Full-width INTEGER16: fine.
        let full = mapping(&[(Address::new(0x6002, 1), 16)]);
        assert!(full.validate_against(&od, false).is_ok());

        // 9-bit slice of a signed type: rejected.
        let narrow = mapping(&[(Address::new(0x6002, 1), 9)]);
        assert!(narrow.validate_against(&od, false).is_err());

        // Unsigned truncation to 9 bits remains legal.
        let trunc = mapping(&[(Address::new(0x6000, 2), 9)]);
        assert!(trunc.validate_against(&od, false).is_ok());
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
    fn unpack_continues_after_a_broken_entry() {
        // BUGS.md H2: one failing entry must not discard the rest of the
        // frame — the first error is reported, but the remaining entries are
        // still applied. Previously the `?` early-returned and 0x6000:2 was
        // left stale.
        let mut od = test_od();
        let m = mapping(&[
            (Address::new(0x2000, 0), 8), // no such object in `test_od`
            (Address::new(0x6000, 1), 8),
        ]);
        let err = unpack(&m, &mut od, &[0x11, 0x22]).unwrap_err();
        assert!(matches!(err, Error::ObjectDoesNotExist { .. }));
        // The healthy entry after the broken one was still written.
        assert_eq!(od.read(Address::new(0x6000, 1)).unwrap(), Value::Uint(0x22));
    }

    #[test]
    fn partial_width_unpack_preserves_unmapped_high_bits() {
        // BUGS.md M3: a U16@8 mapping must merge into the existing value —
        // the previous decode-and-overwrite zeroed the unmapped high byte on
        // every reception.
        let mut od = test_od();
        od.write(Address::new(0x6000, 2), Value::Uint(0xBEEF))
            .unwrap();
        let m = mapping(&[(Address::new(0x6000, 2), 8)]);
        unpack(&m, &mut od, &[0x12]).unwrap();
        // High byte 0xBE survives; only the mapped low byte is replaced.
        assert_eq!(
            od.read(Address::new(0x6000, 2)).unwrap(),
            Value::Uint(0xBE12)
        );

        // A wider partial mapping (U32@16) keeps the top 16 bits too.
        od.write(Address::new(0x6000, 3), Value::Uint(0xDEAD_BEEF))
            .unwrap();
        let m = mapping(&[(Address::new(0x6000, 3), 16)]);
        unpack(&m, &mut od, &[0x12, 0x34]).unwrap();
        assert_eq!(
            od.read(Address::new(0x6000, 3)).unwrap(),
            Value::Uint(0xDEAD_3412)
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
        // Minimal record: only COB-ID present. TPDO direction → default 254.
        let cfg = PdoConfig::from_od(&od, 0x1800, 254).unwrap_or(PdoConfig {
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
        let cfg = PdoConfig::from_od(&od2, 0x1800, 254).unwrap();
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

    /// BUGS.md P2-2：`from_od` 的 `read_u16` 对错误类型（非 Uint）的 sub 3/5
    /// 应显式 `TypeMismatch`，而非静默降级为 0（与 sub 2 行为一致）。
    #[test]
    fn p2_2_from_od_rejects_wrong_typed_sub() {
        let mut od = test_od();
        od.insert(
            Address::new(0x1800, 1),
            Entry::new(
                "",
                DataType::Unsigned32,
                AccessType::Rw,
                Storage::Value(Value::Uint(0x185)),
            ),
        );
        od.insert(
            Address::new(0x1800, 2),
            Entry::new(
                "",
                DataType::Unsigned8,
                AccessType::Rw,
                Storage::Value(Value::Uint(254)),
            ),
        );
        // sub 5（event timer）误设为 Boolean —— 应报错而非静默 0。
        od.insert(
            Address::new(0x1800, 5),
            Entry::new(
                "",
                DataType::Boolean,
                AccessType::Rw,
                Storage::Value(Value::Bool(true)),
            ),
        );
        let err = PdoConfig::from_od(&od, 0x1800, 254).unwrap_err();
        assert!(
            matches!(err, Error::TypeMismatch { .. }),
            "非 Uint 的 sub5 应报 TypeMismatch，实际 {err:?}"
        );
    }
}
