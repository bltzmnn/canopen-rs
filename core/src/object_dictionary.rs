//! Object dictionary: the device's data model.
//!
//! Objects are addressed by a 16-bit index and 8-bit subindex
//! ([`Address`]). Every access is access-controlled ([`AccessType`]) and
//! type-checked against the entry's declared [`DataType`] — violations map
//! onto precise SDO abort codes via [`crate::types::Error::abort_code`]
//! (see docs/REFERENCES.md S7).
//!
//! Entries hold either a static [`Value`] or a callback-backed
//! implementation of [`ObjectAccess`], letting applications virtualize any
//! entry (hardware registers, computed values) without copying data in and
//! out of the dictionary.

use std::collections::BTreeMap;
use std::sync::Arc;

use crate::datatypes::{DataType, Value};
use crate::types::{Address, Error, Result};

/// Access permissions of an object dictionary entry (CiA 301 / EDS
/// `AccessType`). `rwr`/`rww` from EDS files are mapped to [`AccessType::Rw`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AccessType {
    /// Read-only.
    Ro,
    /// Write-only.
    Wo,
    /// Readable and writable.
    Rw,
    /// Readable, never writable, value may change at runtime (const).
    Const,
}

impl AccessType {
    /// Whether the entry may be read.
    #[must_use]
    pub const fn is_readable(self) -> bool {
        !matches!(self, Self::Wo)
    }

    /// Whether the entry may be written.
    #[must_use]
    pub const fn is_writable(self) -> bool {
        matches!(self, Self::Rw | Self::Wo)
    }
}

/// Callback interface for dynamically backed entries.
///
/// Implementations must be thread-safe; they are invoked while the object
/// dictionary is accessed from SDO/PDO contexts. Returning an [`Error`]
/// propagates verbatim — the SDO server converts it into an abort code
/// (`From<Error> for AbortCode` mapping), so validation failures inside the
/// application surface precisely to the remote client.
///
/// # Contract
///
/// Implementations must **not panic**: a panic inside `read`/`write`
/// propagates through [`ObjectDictionary::read`] / [`ObjectDictionary::write`]
/// into the caller's `Node::poll` / `Node::on_frame` stack (unlike EMCY
/// consumer callbacks, which are isolated) (`docs/DESIGN-DECISIONS.md` P3-43).
pub trait ObjectAccess: Send + Sync {
    /// Reads the current value for the addressed subentry.
    ///
    /// # Errors
    /// Implementation-defined; propagated to the accessor.
    fn read(&self, addr: Address) -> Result<Value>;

    /// Writes `value` to the addressed subentry.
    ///
    /// # Errors
    /// Implementation-defined; propagated to the accessor.
    fn write(&self, addr: Address, value: Value) -> Result<()>;
}

/// Backing storage of an entry.
#[derive(Clone)]
pub enum Storage {
    /// A static value held in the dictionary itself.
    Value(Value),
    /// Delegated to an application callback.
    Callback(Arc<dyn ObjectAccess>),
}

impl core::fmt::Debug for Storage {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Storage::Value(v) => write!(f, "Value({v})"),
            Storage::Callback(_) => f.write_str("Callback(..)"),
        }
    }
}

/// A single object dictionary entry (one index/subindex cell).
#[derive(Debug, Clone)]
pub struct Entry {
    /// Human-readable name (kept from EDS imports for diagnostics).
    pub name: String,
    /// Declared CiA 301 data type; governs wire width and type checks.
    pub data_type: DataType,
    /// Access permissions enforced on every read/write.
    pub access: AccessType,
    /// Whether this entry may appear in a PDO mapping.
    pub pdo_mappable: bool,
    /// Whether parameter-storage backends should save/restore this entry.
    pub persistent: bool,
    /// Where the value lives.
    pub storage: Storage,
}

impl Entry {
    /// Creates a new entry with explicit parameters.
    #[must_use]
    pub fn new(
        name: impl Into<String>,
        data_type: DataType,
        access: AccessType,
        storage: Storage,
    ) -> Self {
        Self {
            name: name.into(),
            data_type,
            access,
            pdo_mappable: false,
            persistent: false,
            storage,
        }
    }

    /// Marks the entry as PDO-mappable.
    #[must_use]
    pub const fn pdo_mappable(mut self) -> Self {
        self.pdo_mappable = true;
        self
    }

    /// Marks the entry as persisted by parameter-storage backends.
    #[must_use]
    pub const fn persistent(mut self) -> Self {
        self.persistent = true;
        self
    }

    /// Reads through the storage without access control.
    ///
    /// Reads the current stored value, bypassing access checks — used by
    /// internal consumers (e.g. the SDO server to learn an entry's type, or
    /// PDO packing) that already know the entry is readable; external
    /// readers go through [`ObjectDictionary::read`].
    ///
    /// # Errors
    /// Propagates callback errors for [`Storage::Callback`].
    pub fn raw_read(&self, addr: Address) -> Result<Value> {
        match &self.storage {
            Storage::Value(v) => Ok(v.clone()),
            Storage::Callback(cb) => cb.read(addr),
        }
    }
}

/// The object dictionary: all objects addressable over the bus.
///
/// Backed by a `BTreeMap`, so capacity is dynamic (std target) and
/// iteration is in `(index, subindex)` order.
#[derive(Debug, Default, Clone)]
pub struct ObjectDictionary {
    entries: BTreeMap<Address, Entry>,
    /// Addresses successfully written since the last [`Self::take_dirty`]
    /// drain (`docs/DESIGN-DECISIONS.md` P3-11): the flagsPDO trigger source for
    /// event-driven TPDOs.
    dirty: std::collections::BTreeSet<Address>,
}

impl ObjectDictionary {
    /// Creates an empty dictionary.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Inserts an entry, replacing any existing one at that address.
    pub fn insert(&mut self, addr: Address, entry: Entry) {
        self.entries.insert(addr, entry);
    }

    /// Removes an entry, returning it if present.
    pub fn remove(&mut self, addr: Address) -> Option<Entry> {
        self.entries.remove(&addr)
    }

    /// Borrows the raw entry at `addr`, bypassing access control.
    #[must_use]
    pub fn entry(&self, addr: Address) -> Option<&Entry> {
        self.entries.get(&addr)
    }

    /// Number of stored entries.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// True when no entries exist.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Iterates all addresses in ascending `(index, subindex)` order.
    pub fn iter(&self) -> impl Iterator<Item = (&Address, &Entry)> {
        self.entries.iter()
    }

    /// Stack-internal value update: bypasses [`AccessType`] (the stack owns
    /// maintenance objects such as 0x1001/0x1003) but still enforces the
    /// declared type and range, mirroring CANopenNode's direct OD access for
    /// internal state (docs/REFERENCES.md S7).
    ///
    /// This is the historical name and is kept for compatibility; new code
    /// should use [`Self::set_value_unchecked`] to make the bypass explicit.
    ///
    /// # Errors
    /// Same as [`ObjectDictionary::write`] minus the access check.
    pub fn set_value(&mut self, addr: Address, value: Value) -> Result<()> {
        self.set_value_unchecked(addr, value)
    }

    /// Explicitly named form of [`Self::set_value`]: writes a value while
    /// bypassing the entry's [`AccessType`] check, but still validating its
    /// declared type and range.
    ///
    /// # Errors
    /// Same as [`ObjectDictionary::write`] minus the access check.
    pub fn set_value_unchecked(&mut self, addr: Address, value: Value) -> Result<()> {
        let entry = self
            .entries
            .get_mut(&addr)
            .ok_or(Error::ObjectDoesNotExist {
                index: addr.index,
                subindex: addr.subindex,
            })?;
        Self::check_encode(entry.data_type, addr, &value)?;
        let outcome = match &mut entry.storage {
            Storage::Value(slot) => {
                *slot = value;
                Ok(())
            }
            Storage::Callback(cb) => cb.write(addr, value.clone()),
        };
        if outcome.is_ok() {
            self.dirty.insert(addr);
        }
        outcome
    }

    /// Fills in the addressed entry on context-free errors (`encode_as` has
    /// no `Address` parameter, so type mismatches would otherwise report a
    /// `(0,0)` address).
    fn with_addr(e: Error, addr: Address) -> Error {
        match e {
            Error::TypeMismatch {
                expected, actual, ..
            } => Error::TypeMismatch {
                addr,
                expected,
                actual,
            },
            other => other,
        }
    }

    /// Reads `addr`, enforcing readability.
    ///
    /// # Errors
    /// - [`Error::AccessDenied`] on a write-only entry;
    /// - [`Error::ObjectDoesNotExist`] when absent;
    /// - callback errors for callback-backed entries.
    pub fn read(&self, addr: Address) -> Result<Value> {
        let entry = self.entries.get(&addr).ok_or(Error::ObjectDoesNotExist {
            index: addr.index,
            subindex: addr.subindex,
        })?;
        if !entry.access.is_readable() {
            return Err(Error::AccessDenied { addr, read: true });
        }
        entry.raw_read(addr)
    }

    /// Writes `value` to `addr`, enforcing writability and type.
    ///
    /// For fixed-size types the declared [`DataType`] governs range/width
    /// (via [`Value::encode_as`] semantics); variable-length types accept
    /// any byte length. Callback entries receive the value after these
    /// checks and may apply further validation.
    ///
    /// # Errors
    /// - [`Error::AccessDenied`] on a non-writable entry;
    /// - [`Error::TypeMismatch`] when `value` cannot represent as the
    ///   entry's declared type;
    /// - [`Error::ValueOutOfRange`] for integer overflow of the declared
    ///   width;
    /// - callback errors for callback-backed entries.
    pub fn write(&mut self, addr: Address, value: Value) -> Result<()> {
        let entry = self
            .entries
            .get_mut(&addr)
            .ok_or(Error::ObjectDoesNotExist {
                index: addr.index,
                subindex: addr.subindex,
            })?;
        if !entry.access.is_writable() {
            return Err(Error::AccessDenied { addr, read: false });
        }
        // Type/range check against the declared type — shared with
        // `set_value` via `check_encode` (single source of truth,
        // `docs/DESIGN-DECISIONS.md` R10-P3).
        Self::check_encode(entry.data_type, addr, &value)?;
        let outcome = match &mut entry.storage {
            Storage::Value(slot) => {
                *slot = value;
                Ok(())
            }
            Storage::Callback(cb) => cb.write(addr, value.clone()),
        };
        if outcome.is_ok() {
            self.dirty.insert(addr);
        }
        outcome
    }

    /// Type/width validation shared by [`Self::set_value`] and
    /// [`Self::write`]: fixed-size types must fit a CAN-frame-width scratch
    /// buffer and re-encode exactly; variable-length kinds are checked by
    /// encoding into a throwaway buffer (`docs/DESIGN-DECISIONS.md` P3-48).
    fn check_encode(dt: DataType, addr: Address, value: &Value) -> Result<()> {
        if let Some(size) = dt.fixed_size() {
            // Fixed-size types are at most 8 bytes (the CAN-frame width), so
            // `size > 8` can never happen — the branch was dead code
            // (`docs/DESIGN-DECISIONS.md` L2); keep the invariant as a debug assertion.
            let mut buf = [0u8; 8];
            debug_assert!(size <= buf.len(), "fixed_size() exceeds 8 bytes");
            value
                .encode_as(dt, &mut buf[..size])
                .map_err(|e| Self::with_addr(e, addr))?;
        } else {
            // Variable-length: only check kind compatibility by encoding
            // into a throwaway buffer sized to the value. The absolute
            // value cap mirrors the SDO side so an over-long value can
            // never be stored yet unreadable on the wire (docs/DESIGN-DECISIONS.md #31).
            // `encoded_len_as` covers target types whose wire form is wider
            // than the value's minimal one (UNICODE_STRING UTF-16LE,
            // docs/DESIGN-DECISIONS.md L7).
            let n = value.encoded_len_as(dt);
            if n > crate::datatypes::MAX_VALUE_LEN {
                return Err(Error::BadLength {
                    declared: crate::datatypes::MAX_VALUE_LEN,
                    actual: n,
                });
            }
            let mut buf = vec![0u8; n];
            value.encode_as(dt, &mut buf)?;
        }
        Ok(())
    }

    /// Drains the set of addresses written since the last call (docs/DESIGN-DECISIONS.md
    /// P3-11): consumed by `Node::poll` to fire event-driven TPDOs whose
    /// mappings were touched (flagsPDO semantics).
    pub fn take_dirty(&mut self) -> std::collections::BTreeSet<Address> {
        std::mem::take(&mut self.dirty)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_object_reports_address() {
        let od = ObjectDictionary::new();
        assert_eq!(
            od.read(Address::new(0x1000, 0)),
            Err(Error::ObjectDoesNotExist {
                index: 0x1000,
                subindex: 0
            })
        );
    }

    #[test]
    fn set_value_unchecked_bypasses_access_but_not_type() {
        let mut od = ObjectDictionary::new();
        od.insert(
            Address::new(0x2000, 0),
            Entry::new(
                "ro",
                DataType::Unsigned8,
                AccessType::Ro,
                Storage::Value(Value::Uint(1)),
            ),
        );
        od.set_value_unchecked(Address::new(0x2000, 0), Value::Uint(9))
            .unwrap();
        assert_eq!(od.read(Address::new(0x2000, 0)).unwrap(), Value::Uint(9));
        assert!(matches!(
            od.set_value_unchecked(Address::new(0x2000, 0), Value::Uint(300)),
            Err(Error::ValueOutOfRange { .. })
        ));
    }

    #[test]
    fn access_control_matrix() {
        let mut od = ObjectDictionary::new();
        od.insert(
            Address::new(0x2000, 0),
            Entry::new(
                "ro",
                DataType::Unsigned8,
                AccessType::Ro,
                Storage::Value(Value::Uint(1)),
            ),
        );
        od.insert(
            Address::new(0x2001, 0),
            Entry::new(
                "wo",
                DataType::Unsigned8,
                AccessType::Wo,
                Storage::Value(Value::Uint(0)),
            ),
        );
        od.insert(
            Address::new(0x2002, 0),
            Entry::new(
                "rw",
                DataType::Unsigned8,
                AccessType::Rw,
                Storage::Value(Value::Uint(2)),
            ),
        );
        od.insert(
            Address::new(0x2003, 0),
            Entry::new(
                "const",
                DataType::Unsigned32,
                AccessType::Const,
                Storage::Value(Value::Uint(42)),
            ),
        );

        // ro: readable, not writable
        assert_eq!(od.read(Address::new(0x2000, 0)).unwrap(), Value::Uint(1));
        assert_eq!(
            od.write(Address::new(0x2000, 0), Value::Uint(9)),
            Err(Error::AccessDenied {
                addr: Address::new(0x2000, 0),
                read: false
            })
        );
        // wo: writable, not readable
        assert!(od.write(Address::new(0x2001, 0), Value::Uint(9)).is_ok());
        assert_eq!(
            od.read(Address::new(0x2001, 0)),
            Err(Error::AccessDenied {
                addr: Address::new(0x2001, 0),
                read: true
            })
        );
        // rw: both work, value persists
        assert!(od.write(Address::new(0x2002, 0), Value::Uint(7)).is_ok());
        assert_eq!(od.read(Address::new(0x2002, 0)).unwrap(), Value::Uint(7));
        // const: readable, not writable
        assert_eq!(od.read(Address::new(0x2003, 0)).unwrap(), Value::Uint(42));
        assert!(od.write(Address::new(0x2003, 0), Value::Uint(0)).is_err());
    }

    #[test]
    fn write_checks_declared_type_and_range() {
        let mut od = ObjectDictionary::new();
        let addr = Address::new(0x2010, 0);
        od.insert(
            addr,
            Entry::new(
                "u8",
                DataType::Unsigned8,
                AccessType::Rw,
                Storage::Value(Value::Uint(0)),
            ),
        );
        // Overflow rejected, original preserved.
        assert_eq!(
            od.write(addr, Value::Uint(300)),
            Err(Error::ValueOutOfRange {
                value: 300,
                min: 0,
                max: 255
            })
        );
        assert_eq!(od.read(addr).unwrap(), Value::Uint(0));
        // Type mismatch rejected.
        assert!(matches!(
            od.write(addr, Value::Str("x".into())),
            Err(Error::TypeMismatch { .. })
        ));
        // In-range write succeeds.
        od.write(addr, Value::Uint(42)).unwrap();
        assert_eq!(od.read(addr).unwrap(), Value::Uint(42));
    }

    #[test]
    fn signed_entry_accepts_negative() {
        let mut od = ObjectDictionary::new();
        let addr = Address::new(0x2011, 0);
        od.insert(
            addr,
            Entry::new(
                "i16",
                DataType::Integer16,
                AccessType::Rw,
                Storage::Value(Value::Int(-2)),
            ),
        );
        od.write(addr, Value::Int(-32_769)).unwrap_err(); // out of i16 range
        od.write(addr, Value::Int(-32_768)).unwrap();
        assert_eq!(od.read(addr).unwrap(), Value::Int(-32_768));
    }

    #[test]
    fn callback_entry_delegates_and_errors_propagate() {
        use std::sync::Mutex;

        #[derive(Default)]
        struct Reg(Mutex<u8>);
        impl ObjectAccess for Reg {
            fn read(&self, _addr: Address) -> Result<Value> {
                Ok(Value::Uint(u64::from(*self.0.lock().unwrap())))
            }
            fn write(&self, _addr: Address, value: Value) -> Result<()> {
                let Value::Uint(v) = value else {
                    return Err(Error::Other("expected uint"));
                };
                if v > 10 {
                    return Err(Error::Other("hw limit"));
                }
                *self.0.lock().unwrap() = v as u8;
                Ok(())
            }
        }

        let mut od = ObjectDictionary::new();
        let addr = Address::new(0x2020, 0);
        od.insert(
            addr,
            Entry::new(
                "reg",
                DataType::Unsigned8,
                AccessType::Rw,
                Storage::Callback(Arc::new(Reg::default())),
            ),
        );
        assert_eq!(od.read(addr).unwrap(), Value::Uint(0));
        od.write(addr, Value::Uint(10)).unwrap();
        assert_eq!(od.read(addr).unwrap(), Value::Uint(10));
        // Application error surfaces verbatim.
        assert_eq!(
            od.write(addr, Value::Uint(11)),
            Err(Error::Other("hw limit"))
        );
    }

    #[test]
    fn iteration_is_in_dictionary_order() {
        let mut od = ObjectDictionary::new();
        for addr in [
            Address::new(0x1018, 4),
            Address::new(0x1000, 0),
            Address::new(0x1003, 1),
        ] {
            od.insert(
                addr,
                Entry::new(
                    "",
                    DataType::Unsigned32,
                    AccessType::Ro,
                    Storage::Value(Value::Uint(0)),
                ),
            );
        }
        let addrs: Vec<Address> = od.iter().map(|(a, _)| *a).collect();
        assert_eq!(
            addrs,
            vec![
                Address::new(0x1000, 0),
                Address::new(0x1003, 1),
                Address::new(0x1018, 4),
            ]
        );
    }
}
