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

    /// Restores default parameters for storage group `group` (write of
    /// `'load'` on 0x1011:*sub* — CiA 301 §7.2.5.3.2 semantics; previously
    /// this was mis-mapped to a plain replay of saved values,
    /// `BUGS.md` P2-13). Group 1 covers *all* parameters. `defaults` is the
    /// snapshot captured at node construction.
    ///
    /// # Errors
    /// Implementation-defined; same policy as [`ParameterStorage::save`].
    fn reset(
        &self,
        od: &mut ObjectDictionary,
        defaults: &ObjectDictionary,
        group: u8,
    ) -> Result<()>;
}

/// Installs the 0x1010/0x1011 record skeletons with `groups` storage groups
/// (sub 1..=groups), each defaulting to `1` ("save/restore on command
/// supported"). Sub 0 — the highest-sub-index counter — is **read-only**
/// (CiA 301, BUGS.md M1): a writable counter lets a client SDO-rewrite it
/// and poison the `groups` read in `handle_write` (the private dispatcher),
/// silently disabling all save/restore (storage DoS).
pub fn install_standard_objects(od: &mut ObjectDictionary, groups: u8) {
    let put =
        |od: &mut ObjectDictionary, addr: Address, v: u64, dt: DataType, access: AccessType| {
            od.insert(
                addr,
                Entry::new("", dt, access, Storage::Value(Value::Uint(v))),
            );
        };
    put(
        od,
        Address::new(0x1010, 0),
        u64::from(groups),
        DataType::Unsigned8,
        AccessType::Ro,
    );
    put(
        od,
        Address::new(0x1011, 0),
        u64::from(groups),
        DataType::Unsigned8,
        AccessType::Ro,
    );
    for sub in 1..=groups {
        put(
            od,
            Address::new(0x1010, sub),
            1,
            DataType::Unsigned32,
            AccessType::Rw,
        );
        put(
            od,
            Address::new(0x1011, sub),
            1,
            DataType::Unsigned32,
            AccessType::Rw,
        );
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
    // Reject sub-indices beyond the record's highest-sub counter so a stray
    // write to e.g. 0x1010:9 with groups=2 is not dispatched at all
    // (BUGS.md R10-P3).
    let groups = match od.read(Address::new(addr.index, 0)) {
        Ok(Value::Uint(n)) => n as u8,
        _ => 0,
    };
    if addr.subindex == 0 || addr.subindex > groups {
        return false;
    }
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
        // Every 0x1011:*sub* means *restore defaults for group sub*
        // (CiA 301 §7.2.5.3.2); replaying saved values was the wrong
        // semantic and has been removed (BUGS.md P2-13).
        (0x1011, m, sub @ 1..=127) if m == LOAD_MAGIC_U32 => {
            let _ = backend.reset(od, defaults, sub);
            // Same one-shot hygiene as 0x1010 (BUGS.md P2-12): restore the
            // idle indicator so the magic never persists as dictionary state.
            od.set_value(Address::new(0x1011, sub), Value::Uint(1)).ok();
            true
        }
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    /// Counting backend — records save/reset dispatches so the test can
    /// assert the magic-word routing of [`handle_write`] (BUGS.md L12).
    #[derive(Default)]
    struct MockBackend {
        saves: Mutex<u32>,
        resets: Mutex<Vec<u8>>,
    }

    impl ParameterStorage for MockBackend {
        fn save(&self, _od: &ObjectDictionary) -> Result<()> {
            *self.saves.lock().unwrap() += 1;
            Ok(())
        }
        fn load(&self, _od: &mut ObjectDictionary) -> Result<()> {
            Ok(())
        }
        fn reset(
            &self,
            _od: &mut ObjectDictionary,
            _defaults: &ObjectDictionary,
            group: u8,
        ) -> Result<()> {
            self.resets.lock().unwrap().push(group);
            Ok(())
        }
    }

    fn od_with_groups() -> ObjectDictionary {
        let mut od = ObjectDictionary::new();
        install_standard_objects(&mut od, 2);
        od
    }

    #[test]
    fn save_magic_dispatches_and_resets_entry() {
        let od = od_with_groups();
        let mut od = od;
        let backend = Arc::new(MockBackend::default());
        let dyn_backend: Arc<dyn ParameterStorage> = backend.clone();
        let defaults = ObjectDictionary::new();

        // The magic word on 0x1010:1 triggers a save…
        assert!(handle_write(
            &dyn_backend,
            &mut od,
            &defaults,
            Address::new(0x1010, 1),
            &Value::Uint(u64::from(SAVE_MAGIC_U32)),
        ));
        assert_eq!(*backend.saves.lock().unwrap(), 1);
        // …and the command entry snaps back to the CiA idle indicator so
        // the magic is never persisted as dictionary state.
        assert_eq!(od.read(Address::new(0x1010, 1)).unwrap(), Value::Uint(1));
    }

    #[test]
    fn load_magic_dispatches_to_reset() {
        let od = od_with_groups();
        let mut od = od;
        let backend = Arc::new(MockBackend::default());
        let dyn_backend: Arc<dyn ParameterStorage> = backend.clone();
        let defaults = ObjectDictionary::new();

        assert!(handle_write(
            &dyn_backend,
            &mut od,
            &defaults,
            Address::new(0x1011, 2),
            &Value::Uint(u64::from(LOAD_MAGIC_U32)),
        ));
        assert_eq!(*backend.resets.lock().unwrap(), vec![2]);
        assert_eq!(od.read(Address::new(0x1011, 2)).unwrap(), Value::Uint(1));
    }

    #[test]
    fn sub0_and_out_of_range_subindices_are_not_dispatched() {
        let od = od_with_groups();
        let mut od = od;
        let backend = Arc::new(MockBackend::default());
        let dyn_backend: Arc<dyn ParameterStorage> = backend.clone();
        let defaults = ObjectDictionary::new();
        let magic = Value::Uint(u64::from(SAVE_MAGIC_U32));

        // Sub 0 is the read-only highest-sub counter — never a command.
        assert!(!handle_write(
            &dyn_backend,
            &mut od,
            &defaults,
            Address::new(0x1010, 0),
            &magic,
        ));
        // Groups = 2, so sub 3 is outside the record (BUGS.md R10-P3).
        assert!(!handle_write(
            &dyn_backend,
            &mut od,
            &defaults,
            Address::new(0x1010, 3),
            &magic,
        ));
        assert_eq!(*backend.saves.lock().unwrap(), 0);
        assert!(backend.resets.lock().unwrap().is_empty());
    }

    #[test]
    fn non_magic_write_is_ignored() {
        let od = od_with_groups();
        let mut od = od;
        let backend = Arc::new(MockBackend::default());
        let dyn_backend: Arc<dyn ParameterStorage> = backend.clone();
        let defaults = ObjectDictionary::new();

        assert!(!handle_write(
            &dyn_backend,
            &mut od,
            &defaults,
            Address::new(0x1010, 1),
            &Value::Uint(0xDEADBEEF),
        ));
        // A non-Uint value can never be a magic word.
        assert!(!handle_write(
            &dyn_backend,
            &mut od,
            &defaults,
            Address::new(0x1010, 1),
            &Value::Int(1),
        ));
        assert_eq!(*backend.saves.lock().unwrap(), 0);
    }
}
