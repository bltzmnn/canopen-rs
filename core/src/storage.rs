//! Parameter persistence (CiA 301 §7.2.5.3): objects 0x1010 *Save* and
//! 0x1011 *Restore*, the standard `'save'`/`'load'` magic words and a
//! backend trait — CANopenNode's `CO_storage` model (docs/EXTENSIONS.md
//! §A1).
//!
//! Flow: an SDO client writes the magic word to `0x1010:group` /
//! `0x1011:group`. The SDO server records the write via its
//! last-written hook; [`crate::node::Node::attach_storage`] routes it here and dispatches
//! to the [`ParameterStorage`] backend. Which entries are persisted is
//! decided by the application through
//! [`Entry::persistent`](crate::object_dictionary::Entry::persistent).

use std::sync::Arc;

use crate::datatypes::{DataType, Value};
use crate::object_dictionary::{AccessType, Entry, ObjectDictionary, Storage};
use crate::types::{Address, Result};

/// ASCII `'save'` little-endian (CANopenNode `CO_STORAGE_MAGIC_STRING_SAVE`).
pub const SAVE_MAGIC_U32: u32 = u32::from_le_bytes(*b"save");
/// ASCII `'load'` little-endian.
pub const LOAD_MAGIC_U32: u32 = u32::from_le_bytes(*b"load");

/// Backend for parameter storage groups (subindex of 0x1010/0x1011).
///
/// All methods receive the live dictionary so backends decide their own
/// serialization scope (typically entries flagged
/// [`Entry::persistent`]).
pub trait ParameterStorage: Send + Sync {
    /// Persists storable values (write of `'save'`).
    ///
    /// # Errors
    /// Implementation-defined; surfaced as debug log + stored error state,
    /// never aborts the confirming SDO write (the dictionary update itself
    /// already succeeded per CiA 301).
    ///
    /// Implementations manage interior mutability themselves (`Mutex`) since
    /// the node shares the backend behind an [`Arc`].
    fn save(&self, od: &ObjectDictionary) -> Result<()>;

    /// Re-applies previously saved values (write of `'load'`).
    ///
    /// # Errors
    /// Implementation-defined; same policy as [`ParameterStorage::save`].
    fn load(&self, od: &mut ObjectDictionary) -> Result<()>;

    /// Clears stored parameters and reverts the dictionary to defaults
    /// (write of `'load'` on 0x1011). `defaults` is the snapshot captured at
    /// node construction.
    ///
    /// # Errors
    /// Implementation-defined; same policy as [`ParameterStorage::save`].
    fn reset(&self, od: &mut ObjectDictionary, defaults: &ObjectDictionary) -> Result<()>;
}

/// Installs the 0x1010/0x1011 record skeletons with `groups` storage groups
/// (sub 1..=groups), each defaulting to `1` ("save/restore on command
/// supported").
pub fn install_standard_objects(od: &mut ObjectDictionary, groups: u8) {
    let put = |od: &mut ObjectDictionary, addr: Address, v: u64, dt: DataType| {
        od.insert(
            addr,
            Entry::new("", dt, AccessType::Rw, Storage::Value(Value::Uint(v))),
        );
    };
    put(
        od,
        Address::new(0x1010, 0),
        u64::from(groups),
        DataType::Unsigned8,
    );
    put(
        od,
        Address::new(0x1011, 0),
        u64::from(groups),
        DataType::Unsigned8,
    );
    for sub in 1..=groups {
        put(od, Address::new(0x1010, sub), 1, DataType::Unsigned32);
        put(od, Address::new(0x1011, sub), 1, DataType::Unsigned32);
    }
}

/// Dispatches one recorded parameter write to the storage backend.
/// Returns whether a storage action ran.
pub(crate) fn handle_write(
    backend: &Arc<dyn ParameterStorage>,
    od: &mut ObjectDictionary,
    defaults: &ObjectDictionary,
    addr: Address,
    value: &Value,
) -> bool {
    let Value::Uint(raw) = value else {
        return false;
    };
    match (addr.index, *raw as u32, addr.subindex) {
        (0x1010, m, sub) if m == SAVE_MAGIC_U32 && sub >= 1 => {
            // Backend errors are its own concern; the SDO write itself was
            // already confirmed by the dictionary update.
            let _ = backend.save(od);
            // Reset the command entry to CiA's "idle" indicator so the
            // one-shot magic is never persisted as dictionary state.
            od.set_value(Address::new(0x1010, sub), Value::Uint(1)).ok();
            true
        }
        (0x1011, m, 1) if m == LOAD_MAGIC_U32 => {
            // Standard semantics for group 1: restore defaults AND clear.
            let _ = backend.reset(od, defaults);
            true
        }
        (0x1011, m, _) if m == LOAD_MAGIC_U32 => {
            let _ = backend.load(od);
            true
        }
        _ => false,
    }
}
