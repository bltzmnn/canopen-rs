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
    /// Implementation-defined; never aborts the confirming SDO write (the
    /// dictionary update itself already succeeded per CiA 301). Failures are
    /// recorded by [`crate::node::Node`] and surfaced through
    /// [`crate::node::Node::take_config_errors`] (`docs/DESIGN-DECISIONS.md` G7).
    ///
    /// Implementations manage interior mutability themselves (`Mutex`) since
    /// the node shares the backend behind an [`Arc`].
    fn save(&self, od: &ObjectDictionary) -> Result<()>;

    /// Re-applies previously saved values (write of `'load'`).
    ///
    /// # Errors
    /// Implementation-defined; recorded like [`ParameterStorage::save`].
    fn load(&self, od: &mut ObjectDictionary) -> Result<()>;

    /// Restores default parameters for storage group `group` (write of
    /// `'load'` on 0x1011:*sub* — CiA 301 §7.2.5.3.2 semantics; previously
    /// this was mis-mapped to a plain replay of saved values,
    /// `docs/DESIGN-DECISIONS.md` P2-13). Group 1 covers *all* parameters. `defaults` is the
    /// snapshot captured at node construction.
    ///
    /// # Errors
    /// Implementation-defined; recorded like [`ParameterStorage::save`].
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
/// (CiA 301, docs/DESIGN-DECISIONS.md M1): a writable counter lets a client SDO-rewrite it
/// and poison the `groups` read in `handle_write` (the private dispatcher),
/// silently disabling all save/restore (storage DoS).
///
/// The records only *act* once a backend is attached with
/// [`Node::attach_storage`](crate::node::Node::attach_storage): until then an
/// SDO write of the `'save'`/`'load'` magic word is refused with
/// `0x0601_0002` (CANopenLinux's `ODR_READONLY`,
/// `storage/CO_storage.c:38-40`; `docs/DESIGN-DECISIONS.md` HOST-04) instead of reporting a
/// successful store that never happens.
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
///
/// Returns the backend failure, if one occurred: the SDO write itself has
/// already been confirmed (CiA 301), so the node records the error through
/// [`Node::take_config_errors`](crate::node::Node::take_config_errors)
/// instead of aborting the transaction. `None` means "nothing dispatched or
/// the action succeeded".
pub(crate) fn handle_write(
    backend: &Arc<dyn ParameterStorage>,
    od: &mut ObjectDictionary,
    defaults: &ObjectDictionary,
    addr: Address,
    value: &Value,
) -> Option<crate::types::Error> {
    let Value::Uint(raw) = value else {
        return None;
    };
    // Reject sub-indices beyond the record's highest-sub counter so a stray
    // write to e.g. 0x1010:9 with groups=2 is not dispatched at all
    // (docs/DESIGN-DECISIONS.md R10-P3).
    let groups = match od.read(Address::new(addr.index, 0)) {
        Ok(Value::Uint(n)) => n as u8,
        _ => 0,
    };
    if addr.subindex == 0 || addr.subindex > groups {
        return None;
    }
    match (addr.index, *raw as u32, addr.subindex) {
        (0x1010, m, sub) if m == SAVE_MAGIC_U32 && sub >= 1 => {
            let result = backend.save(od);
            // Reset the command entry to CiA's "idle" indicator so the
            // one-shot magic is never persisted as dictionary state.
            od.set_value(Address::new(0x1010, sub), Value::Uint(1)).ok();
            result.err()
        }
        // Every 0x1011:*sub* means *restore defaults for group sub*
        // (CiA 301 §7.2.5.3.2); replaying saved values was the wrong
        // semantic and has been removed (docs/DESIGN-DECISIONS.md P2-13).
        (0x1011, m, sub @ 1..=127) if m == LOAD_MAGIC_U32 => {
            let result = backend.reset(od, defaults, sub);
            // Same one-shot hygiene as 0x1010 (docs/DESIGN-DECISIONS.md P2-12): restore the
            // idle indicator so the magic never persists as dictionary state.
            od.set_value(Address::new(0x1011, sub), Value::Uint(1)).ok();
            result.err()
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    /// Counting backend — records save/reset dispatches so the test can
    /// assert the magic-word routing of [`handle_write`] (docs/DESIGN-DECISIONS.md L12).
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
        )
        .is_none());
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
        )
        .is_none());
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
        assert!(handle_write(
            &dyn_backend,
            &mut od,
            &defaults,
            Address::new(0x1010, 0),
            &magic,
        )
        .is_none());
        // Groups = 2, so sub 3 is outside the record (docs/DESIGN-DECISIONS.md R10-P3).
        assert!(handle_write(
            &dyn_backend,
            &mut od,
            &defaults,
            Address::new(0x1010, 3),
            &magic,
        )
        .is_none());
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

        assert!(handle_write(
            &dyn_backend,
            &mut od,
            &defaults,
            Address::new(0x1010, 1),
            &Value::Uint(0xDEADBEEF),
        )
        .is_none());
        // A non-Uint value can never be a magic word.
        assert!(handle_write(
            &dyn_backend,
            &mut od,
            &defaults,
            Address::new(0x1010, 1),
            &Value::Int(1),
        )
        .is_none());
        assert_eq!(*backend.saves.lock().unwrap(), 0);
    }

    /// A failing backend must hand its error back to the caller (the node
    /// records it via `take_config_errors`, `docs/DESIGN-DECISIONS.md` G7) — the confirming
    /// SDO write is still one-shot-reset.
    #[test]
    fn backend_failure_is_returned_to_the_caller() {
        struct FailingBackend;

        impl ParameterStorage for FailingBackend {
            fn save(&self, _od: &ObjectDictionary) -> Result<()> {
                Err(crate::types::Error::Other("save failed"))
            }
            fn load(&self, _od: &mut ObjectDictionary) -> Result<()> {
                Ok(())
            }
            fn reset(
                &self,
                _od: &mut ObjectDictionary,
                _defaults: &ObjectDictionary,
                _group: u8,
            ) -> Result<()> {
                Err(crate::types::Error::Other("reset failed"))
            }
        }

        let mut od = od_with_groups();
        let backend: Arc<dyn ParameterStorage> = Arc::new(FailingBackend);
        let defaults = ObjectDictionary::new();

        let save_err = handle_write(
            &backend,
            &mut od,
            &defaults,
            Address::new(0x1010, 1),
            &Value::Uint(u64::from(SAVE_MAGIC_U32)),
        )
        .expect("save failure must surface");
        assert!(matches!(save_err, crate::types::Error::Other(_)));
        assert_eq!(od.read(Address::new(0x1010, 1)).unwrap(), Value::Uint(1));

        let reset_err = handle_write(
            &backend,
            &mut od,
            &defaults,
            Address::new(0x1011, 1),
            &Value::Uint(u64::from(LOAD_MAGIC_U32)),
        )
        .expect("reset failure must surface");
        assert!(matches!(reset_err, crate::types::Error::Other(_)));
        assert_eq!(od.read(Address::new(0x1011, 1)).unwrap(), Value::Uint(1));
    }
}
