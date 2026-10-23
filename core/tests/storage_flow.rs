//! Storage integration tests: magic-word flow through a real Node over the
//! SDO path, plus the host `FileStorage` round-trip.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use canopen_core::datatypes::{DataType, Value};
use canopen_core::node::Node;
use canopen_core::object_dictionary::{AccessType, Entry, ObjectDictionary, Storage};
use canopen_core::sdo::{SdoAbortCode, SdoClient, SdoEvent};
use canopen_core::standard::StandardObjects;
use canopen_core::storage::{
    install_standard_objects, ParameterStorage, LOAD_MAGIC_U32, SAVE_MAGIC_U32,
};
use canopen_core::types::{Address, NodeId, Result};

const NODE: u8 = 4;

fn test_od() -> ObjectDictionary {
    let mut od = StandardObjects::new().build();
    // Two groups so 0x1011:1 (defaults+clear) and 0x1011:2 (plain load)
    // are both addressable.
    install_standard_objects(&mut od, 2);
    od.insert(
        Address::new(0x6000, 1),
        Entry::new(
            "persisted setting",
            DataType::Unsigned16,
            AccessType::Rw,
            Storage::Value(Value::Uint(111)),
        )
        .persistent(),
    );
    od.insert(
        Address::new(0x6000, 2),
        Entry::new(
            "volatile value",
            DataType::Unsigned16,
            AccessType::Rw,
            Storage::Value(Value::Uint(222)),
        ),
    );
    od
}

fn sdo_write(node: &mut Node, addr: Address, raw: &[u8]) {
    let mut client = SdoClient::new();
    let first = client.write(addr, raw).unwrap();
    let mut queue: std::collections::VecDeque<[u8; 8]> = first.into();
    while let Some(frame) = queue.pop_front() {
        for resp in node.on_frame(0x600 + u16::from(NODE), &frame) {
            if let SdoEvent::Send(frames) = client.on_response(&resp.data).unwrap() {
                queue.extend(frames);
            }
        }
    }
}

/// Recording mock: counts calls per kind.
#[derive(Default)]
struct MockBackend {
    saves: AtomicUsize,
    loads: AtomicUsize,
    resets: AtomicUsize,
    saved_values: Mutex<Vec<(u16, u8, u64)>>,
}

impl ParameterStorage for MockBackend {
    fn save(&self, od: &ObjectDictionary) -> Result<()> {
        self.saves.fetch_add(1, Ordering::Relaxed);
        let mut snapshot = Vec::new();
        for (addr, entry) in od.iter() {
            if !entry.persistent {
                continue;
            }
            if let Value::Uint(v) = entry.raw_read(*addr).unwrap() {
                snapshot.push((addr.index, addr.subindex, v));
            }
        }
        *self.saved_values.lock().unwrap() = snapshot;
        Ok(())
    }

    fn load(&self, od: &mut ObjectDictionary) -> Result<()> {
        self.loads.fetch_add(1, Ordering::Relaxed);
        for (index, sub, v) in self.saved_values.lock().unwrap().iter() {
            od.set_value(Address::new(*index, *sub), Value::Uint(*v))?;
        }
        Ok(())
    }

    fn reset(
        &self,
        od: &mut ObjectDictionary,
        defaults: &ObjectDictionary,
        _group: u8,
    ) -> Result<()> {
        self.resets.fetch_add(1, Ordering::Relaxed);
        for (addr, entry) in defaults.iter() {
            if entry.persistent {
                let v = entry.raw_read(*addr)?;
                od.set_value(*addr, v)?;
            }
        }
        Ok(())
    }
}

#[test]
fn save_magic_triggers_backend_via_sdo_path() {
    let mut node = Node::new(NodeId::new(NODE).unwrap(), test_od()).unwrap();
    node.boot().unwrap(); // SDO is served from PreOperational upwards
    let backend = Arc::new(MockBackend::default());
    node.attach_storage(backend.clone());

    // Change the persisted setting through SDO…
    sdo_write(&mut node, Address::new(0x6000, 1), &0xBEEFu16.to_le_bytes());
    assert_eq!(backend.saves.load(Ordering::Relaxed), 0);

    // …then issue 'save' on group 1.
    sdo_write(
        &mut node,
        Address::new(0x1010, 1),
        &SAVE_MAGIC_U32.to_le_bytes(),
    );
    assert_eq!(backend.saves.load(Ordering::Relaxed), 1);
    assert_eq!(
        backend.saved_values.lock().unwrap().as_slice(),
        &[(0x6000, 1, 0xBEEF)]
    );

    // Wrong magic is rejected with DataCannotBeTransferred (CANopenNode
    // `OD_write_1010` -> `ODR_DATA_TRANSF`) and the backend is not called.
    assert_eq!(
        sdo_write_expect_abort(&mut node, Address::new(0x1010, 1), &0xDEADu32.to_le_bytes()),
        canopen_core::sdo::SdoAbortCode::DataCannotBeTransferred
    );
    assert_eq!(backend.saves.load(Ordering::Relaxed), 1);
}

/// Drives a write expected to end in an SDO abort; returns the code.
fn sdo_write_expect_abort(
    node: &mut Node,
    addr: Address,
    raw: &[u8],
) -> canopen_core::sdo::SdoAbortCode {
    let mut client = SdoClient::new();
    let first = client.write(addr, raw).unwrap();
    let mut queue: std::collections::VecDeque<[u8; 8]> = first.into();
    while let Some(frame) = queue.pop_front() {
        for resp in node.on_frame(0x600 + u16::from(NODE), &frame) {
            match client.on_response(&resp.data) {
                Ok(SdoEvent::Send(frames)) => queue.extend(frames),
                Ok(_) => {}
                Err(canopen_core::types::Error::SdoAborted(code)) => return code,
                Err(e) => panic!("unexpected error {e:?}"),
            }
        }
    }
    panic!("transaction completed without abort");
}

#[test]
fn load_magic_reapplies_and_reset_restores_defaults() {
    let mut node = Node::new(NodeId::new(NODE).unwrap(), test_od()).unwrap();
    node.boot().unwrap();
    let backend = Arc::new(MockBackend {
        saved_values: Mutex::new(vec![(0x6000, 1, 777)]),
        ..Default::default()
    });
    node.attach_storage(backend.clone());

    // Group 1 'load': restore defaults AND clear stored parameters
    // (standard semantics).
    sdo_write(
        &mut node,
        Address::new(0x1011, 1),
        &LOAD_MAGIC_U32.to_le_bytes(),
    );
    assert_eq!(backend.resets.load(Ordering::Relaxed), 1);
    assert_eq!(
        node.od().read(Address::new(0x6000, 1)).unwrap(),
        Value::Uint(111) // construction-time default
    );

    // Group 2 'load': restore defaults for group 2 — the old "replay saved
    // values" behaviour was a CiA-semantics mistake (docs/DESIGN-DECISIONS.md P2-13); every
    // 0x1011:*sub* write is a reset for that group.
    sdo_write(
        &mut node,
        Address::new(0x1011, 2),
        &LOAD_MAGIC_U32.to_le_bytes(),
    );
    assert_eq!(backend.resets.load(Ordering::Relaxed), 2);
    assert_eq!(backend.loads.load(Ordering::Relaxed), 0);
    assert_eq!(
        node.od().read(Address::new(0x6000, 1)).unwrap(),
        Value::Uint(111) // back to construction-time default, not 777
    );
    // Non-persistent entry untouched by the whole flow.
    assert_eq!(
        node.od().read(Address::new(0x6000, 2)).unwrap(),
        Value::Uint(222)
    );
}

#[test]
fn nmt_reset_reapplies_stored_parameters() {
    let mut node = Node::new(NodeId::new(NODE).unwrap(), test_od()).unwrap();
    let backend = Arc::new(MockBackend {
        saved_values: Mutex::new(vec![(0x6000, 1, 42)]),
        ..Default::default()
    });
    node.attach_storage(backend.clone());

    node.boot().unwrap();
    let loads_before = backend.loads.load(Ordering::Relaxed);

    let _ = node.on_frame(0x000, &[0x82, NODE]); // reset communication
    assert_eq!(backend.loads.load(Ordering::Relaxed), loads_before + 1);
    assert_eq!(
        node.od().read(Address::new(0x6000, 1)).unwrap(),
        Value::Uint(42)
    );
}

/// docs/DESIGN-DECISIONS.md G7：后端 save/load/reset 失败不再被丢弃，而是经
/// `Node::take_config_errors()` 上报给应用。
#[test]
fn backend_failures_are_recorded_in_config_errors() {
    struct FailingBackend;

    impl ParameterStorage for FailingBackend {
        fn save(&self, _od: &ObjectDictionary) -> Result<()> {
            Err(canopen_core::types::Error::Other("save boom"))
        }
        fn load(&self, _od: &mut ObjectDictionary) -> Result<()> {
            Err(canopen_core::types::Error::Other("load boom"))
        }
        fn reset(
            &self,
            _od: &mut ObjectDictionary,
            _defaults: &ObjectDictionary,
            _group: u8,
        ) -> Result<()> {
            Err(canopen_core::types::Error::Other("reset boom"))
        }
    }

    let mut node = Node::new(NodeId::new(NODE).unwrap(), test_od()).unwrap();
    node.boot().unwrap();
    node.attach_storage(Arc::new(FailingBackend));

    // 0x1010 'save' failure (the confirming SDO write still succeeds).
    sdo_write(
        &mut node,
        Address::new(0x1010, 1),
        &SAVE_MAGIC_U32.to_le_bytes(),
    );
    assert!(
        node.take_config_errors()
            .any(|e| e.to_string().contains("save boom")),
        "save failure must be reported"
    );

    // 0x1011 'load' magic maps to reset() per CiA 301.
    sdo_write(
        &mut node,
        Address::new(0x1011, 1),
        &LOAD_MAGIC_U32.to_le_bytes(),
    );
    assert!(
        node.take_config_errors()
            .any(|e| e.to_string().contains("reset boom")),
        "reset failure must be reported"
    );

    // NMT reset communication re-applies persisted parameters via load().
    let _ = node.on_frame(0x000, &[0x82, NODE]);
    assert!(
        node.take_config_errors()
            .any(|e| e.to_string().contains("load boom")),
        "load failure must be reported"
    );
}

/// Single-shot expedited write; returns the abort code when the server
/// refused the request (`None` on confirmation).
fn sdo_write_abort_code(node: &mut Node, addr: Address, raw: &[u8]) -> Option<SdoAbortCode> {
    let mut client = SdoClient::new();
    let first = client.write(addr, raw).unwrap();
    let out = node.on_frame(0x600 + u16::from(NODE), &first[0]);
    match client.on_response(&out[0].data) {
        Ok(_) => None,
        Err(canopen_core::types::Error::SdoAborted(code)) => Some(code),
        Err(e) => panic!("unexpected client error: {e}"),
    }
}

/// docs/DESIGN-DECISIONS.md HOST-04: without a storage backend a `'save'`/`'load'` magic word
/// must be refused with 0x0601_0002 (CANopenLinux's `ODR_READONLY`,
/// `storage/CO_storage.c:38-40`) instead of being confirmed while nothing is
/// stored — and the object dictionary must not keep the magic word.
#[test]
fn storage_magic_write_without_backend_is_refused() {
    let mut node = Node::new(NodeId::new(NODE).unwrap(), test_od()).unwrap();
    node.boot().unwrap(); // SDO is served from PreOperational upwards
                          // No attach_storage() here on purpose.

    let mut client = SdoClient::new();
    let first = client
        .write(Address::new(0x1010, 1), &SAVE_MAGIC_U32.to_le_bytes())
        .unwrap();
    let out = node.on_frame(0x600 + u16::from(NODE), &first[0]);
    assert_eq!(out.len(), 1, "the server must answer the request");
    assert_eq!(out[0].data[0] & 0xE0, 0x80, "expected an abort frame");
    assert_eq!(
        u32::from_le_bytes([
            out[0].data[4],
            out[0].data[5],
            out[0].data[6],
            out[0].data[7]
        ]),
        0x0601_0002,
        "ODR_READONLY: write of a read-only object"
    );
    assert_eq!(
        node.od().read(Address::new(0x1010, 1)),
        Ok(Value::Uint(1)),
        "the magic word must not land in the object dictionary"
    );
    assert!(
        node.take_config_errors().next().is_none(),
        "a refused write is not a configuration error"
    );

    // A wrong magic word still reports the transfer error, not read-only.
    assert_eq!(
        sdo_write_abort_code(
            &mut node,
            Address::new(0x1010, 1),
            &0xDEAD_BEEFu32.to_le_bytes()
        ),
        Some(SdoAbortCode::DataCannotBeTransferred)
    );
    // 0x1011 behaves the same way.
    assert_eq!(
        sdo_write_abort_code(
            &mut node,
            Address::new(0x1011, 1),
            &LOAD_MAGIC_U32.to_le_bytes()
        ),
        Some(SdoAbortCode::WriteOfReadOnlyObject)
    );
    assert_eq!(
        node.od().read(Address::new(0x1011, 1)),
        Ok(Value::Uint(1)),
        "the 'load' magic word must not land either"
    );

    // With a backend attached the same write is accepted and dispatched.
    let backend = Arc::new(MockBackend::default());
    node.attach_storage(backend.clone());
    sdo_write(
        &mut node,
        Address::new(0x1010, 1),
        &SAVE_MAGIC_U32.to_le_bytes(),
    );
    assert_eq!(backend.saves.load(Ordering::Relaxed), 1);
    assert_eq!(
        node.od().read(Address::new(0x1010, 1)),
        Ok(Value::Uint(1)),
        "the command entry snaps back to the CiA idle indicator"
    );
}
