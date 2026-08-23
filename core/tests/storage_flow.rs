//! Storage integration tests: magic-word flow through a real Node over the
//! SDO path, plus the host `FileStorage` round-trip.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use canopen_core::datatypes::{DataType, Value};
use canopen_core::node::Node;
use canopen_core::object_dictionary::{AccessType, Entry, ObjectDictionary, Storage};
use canopen_core::sdo::SdoClient;
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

use canopen_core::sdo::SdoEvent;

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

    fn reset(&self, od: &mut ObjectDictionary, defaults: &ObjectDictionary) -> Result<()> {
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

    // Wrong magic does nothing.
    sdo_write(&mut node, Address::new(0x1010, 1), &0xDEADu32.to_le_bytes());
    assert_eq!(backend.saves.load(Ordering::Relaxed), 1);
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

    // Group 2 'load': plain re-application of the stored set.
    sdo_write(
        &mut node,
        Address::new(0x1011, 2),
        &LOAD_MAGIC_U32.to_le_bytes(),
    );
    assert_eq!(backend.loads.load(Ordering::Relaxed), 1);
    assert_eq!(
        node.od().read(Address::new(0x6000, 1)).unwrap(),
        Value::Uint(777)
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
