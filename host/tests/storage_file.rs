//! `FileStorage` round-trip: save → mutate → load restores; reset reverts
//! to defaults and empties the file.

use std::path::PathBuf;

use canopen_core::datatypes::{DataType, Value};
use canopen_core::object_dictionary::{AccessType, Entry, ObjectDictionary, Storage};
use canopen_core::standard::StandardObjects;
use canopen_core::storage::{install_standard_objects, ParameterStorage};
use canopen_core::types::Address;
use canopen_host::storage::FileStorage;

fn test_od() -> ObjectDictionary {
    let mut od = StandardObjects::new().build();
    install_standard_objects(&mut od, 1);
    for (sub, default) in [(1u8, 111u64), (2, 222)] {
        let mut e = Entry::new(
            "persisted",
            DataType::Unsigned16,
            AccessType::Rw,
            Storage::Value(Value::Uint(default)),
        );
        if sub == 1 {
            e = e.persistent();
        }
        od.insert(Address::new(0x6000, sub), e);
    }
    od
}

#[test]
fn file_roundtrip_and_reset() {
    let path = std::env::temp_dir().join("canopen-rs-test-params.txt");
    let _ = std::fs::remove_file(&path);
    let backend = FileStorage::new(path.clone());

    let mut od = test_od();
    // Only 0x6000:1 is persistent — 0x6000:2 must never appear in the file.
    od.set_value(Address::new(0x6000, 1), Value::Uint(0xBEEF))
        .unwrap();

    backend.save(&od).unwrap();
    let raw = std::fs::read_to_string(&path).unwrap();
    assert!(raw.contains("# canopen-rs parameter storage v1"));
    assert!(
        raw.contains("6000 001 02 ef be"),
        "unexpected content:\n{raw}"
    );
    assert!(!raw.contains("6000 002"));

    // Mutate, then load restores the saved value.
    od.set_value(Address::new(0x6000, 1), Value::Uint(7))
        .unwrap();
    backend.load(&mut od).unwrap();
    assert_eq!(
        od.read(Address::new(0x6000, 1)).unwrap(),
        Value::Uint(0xBEEF)
    );

    // Reset reverts to the construction-time defaults and clears the store.
    let defaults = test_od();
    backend.reset(&mut od, &defaults).unwrap();
    assert_eq!(od.read(Address::new(0x6000, 1)).unwrap(), Value::Uint(111));
    // …so a subsequent load is a no-op on values.
    backend.load(&mut od).unwrap();
    assert_eq!(od.read(Address::new(0x6000, 1)).unwrap(), Value::Uint(111));

    let _ = std::fs::remove_file(&path);
}

#[test]
fn missing_file_loads_as_noop() {
    let backend = FileStorage::new(PathBuf::from("/tmp/canopen-rs-nonexistent-dir/params.txt"));
    let mut od = test_od();
    backend.load(&mut od).unwrap(); // no error
    assert_eq!(od.read(Address::new(0x6000, 1)).unwrap(), Value::Uint(111));
}

#[test]
fn save_is_atomic_via_temp_file() {
    // BUGS.md m8: a crash mid-save must never leave a truncated parameter
    // file — the payload lands in a sibling .tmp first, then renames over
    // the target (CANopenNode CO_storageLinux.c).
    let dir = std::env::temp_dir().join("canopen-rs-atomic-test");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("params.txt");
    let _ = std::fs::remove_file(&path);

    let storage = FileStorage::new(path.clone());
    let od = StandardObjects::new().build();
    storage.save(&od).unwrap();

    assert!(path.exists());
    assert!(
        !dir.join("params.tmp").exists(),
        "tmp file must be renamed away"
    );
    // Content intact and re-loadable.
    let reloaded = FileStorage::new(path.clone());
    let mut od2 = StandardObjects::new().build();
    reloaded.load(&mut od2).unwrap();
}
