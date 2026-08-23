//! On-bus integration tests over a real `vcan0` interface (skip-if-missing).
//!
//! Setup: `sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0`
//!
//! Topology: a core `Node` is served on one socket (server thread); the test
//! thread acts as the master on a second socket.

#![cfg(target_os = "linux")]

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use canopen_core::datatypes::{DataType, Value};
use canopen_core::emcy::install_standard_objects;
use canopen_core::node::Node;
use canopen_core::object_dictionary::{AccessType, Entry, ObjectDictionary, Storage};
use canopen_core::pdo::MappingEntry;
use canopen_core::standard::StandardObjects;
use canopen_core::types::{Address, NodeId};
use canopen_host::transport::{Incoming, SocketCan};

const IFACE: &str = "vcan0";
const NODE_ID: u8 = 5;

fn test_od() -> ObjectDictionary {
    let mut od = StandardObjects::new().device_type(0x0000_0192).build();
    install_standard_objects(&mut od, NodeId::new(NODE_ID).unwrap());
    od.insert(
        Address::new(0x6000, 1),
        Entry::new(
            "app u8",
            DataType::Unsigned8,
            AccessType::Rw,
            Storage::Value(Value::Uint(0)),
        )
        .pdo_mappable(),
    );
    od.insert(
        Address::new(0x6000, 2),
        Entry::new(
            "app u16",
            DataType::Unsigned16,
            AccessType::Rw,
            Storage::Value(Value::Uint(0)),
        )
        .pdo_mappable(),
    );
    // PDO parameter objects: TPDO1/RPDO1 with invalid default COB-IDs.
    for (comm, map) in [(0x1800u16, 0x1A00u16), (0x1400, 0x1600)] {
        let cob = if comm == 0x1800 { 0x185 } else { 0x205 };
        let put = |od: &mut ObjectDictionary, addr: Address, v: u64, dt: DataType| {
            od.insert(
                addr,
                Entry::new("", dt, AccessType::Rw, Storage::Value(Value::Uint(v))),
            );
        };
        put(
            &mut od,
            Address::new(comm, 1),
            cob | 0x8000_0000,
            DataType::Unsigned32,
        );
        put(&mut od, Address::new(comm, 2), 254, DataType::Unsigned8);
        put(&mut od, Address::new(map, 0), 0, DataType::Unsigned8);
        for sub in 1..=4u16 {
            put(
                &mut od,
                Address::new(map, sub as u8),
                0,
                DataType::Unsigned32,
            );
        }
    }
    od
}

fn open_or_skip() -> Option<SocketCan> {
    match SocketCan::open(IFACE) {
        Ok(sock) => Some(sock),
        Err(canopen_host::HostError::InterfaceNotFound(name)) => {
            eprintln!("skipping vcan test: interface {name} not available");
            None
        }
        Err(e) => panic!("failed to open {IFACE}: {e}"),
    }
}

#[test]
fn full_flow_on_vcan() {
    let Some(mut master) = open_or_skip() else {
        return;
    };

    // Server thread.
    let shutdown = Arc::new(AtomicBool::new(false));
    let server_shutdown = shutdown.clone();
    let server = std::thread::spawn(move || -> Result<(), canopen_host::HostError> {
        let mut bus = SocketCan::open(IFACE)?;
        let mut node = Node::new(NodeId::new(NODE_ID).unwrap(), test_od()).unwrap();
        bus.serve_node(&mut node, &server_shutdown)
    });

    // Give the server time to boot; wait for boot-up/heartbeat from the node.
    let deadline = std::time::Instant::now() + Duration::from_secs(2);
    loop {
        assert!(
            std::time::Instant::now() < deadline,
            "no sign of life from node"
        );
        match master.recv(Duration::from_millis(50)).unwrap() {
            Some(Incoming::Data(rx)) if rx.cob_id == 0x700 + u16::from(NODE_ID) => break,
            _ => continue,
        }
    }

    // Start the node.
    master
        .send_nmt(
            canopen_core::nmt::NmtCommand::StartRemoteNode,
            Some(NODE_ID),
        )
        .unwrap();

    // SDO read: device type.
    let dt = master
        .sdo_read(NODE_ID, Address::new(0x1000, 0), DataType::Unsigned32)
        .unwrap();
    assert_eq!(dt, Value::Uint(0x192));

    // SDO write + read-back of application data.
    master
        .sdo_write(NODE_ID, Address::new(0x6000, 1), &[0xAB])
        .unwrap();
    assert_eq!(
        master
            .sdo_read(NODE_ID, Address::new(0x6000, 1), DataType::Unsigned8)
            .unwrap(),
        Value::Uint(0xAB)
    );

    // Configure TPDO1 via SDO: valid COB-ID 0x185, SYNC every 2nd, mapping
    // 0x6000:1 (8 bit) — then trigger it with two SYNCs after updating data.
    master
        .sdo_write(NODE_ID, Address::new(0x1800, 1), &0x185u32.to_le_bytes())
        .unwrap();
    master
        .sdo_write(NODE_ID, Address::new(0x1800, 2), &[2])
        .unwrap();
    master
        .sdo_write(NODE_ID, Address::new(0x1A00, 0), &[1])
        .unwrap();
    master
        .sdo_write(
            NODE_ID,
            Address::new(0x1A00, 1),
            &MappingEntry {
                address: Address::new(0x6000, 1),
                bit_length: 8,
            }
            .to_u32()
            .to_le_bytes(),
        )
        .unwrap();

    master
        .sdo_write(NODE_ID, Address::new(0x6000, 1), &[0x5A])
        .unwrap();
    master.send(0x080, &[]).unwrap();
    master.send(0x080, &[]).unwrap();

    // Expect exactly one TPDO1 frame carrying [0x5A].
    let deadline = std::time::Instant::now() + Duration::from_secs(2);
    let mut tpdo = None;
    while std::time::Instant::now() < deadline {
        match master.recv(Duration::from_millis(50)).unwrap() {
            Some(Incoming::Data(rx)) if rx.cob_id == 0x185 => {
                tpdo = Some(rx);
                break;
            }
            _ => continue,
        }
    }
    let tpdo = tpdo.expect("TPDO not observed");
    assert_eq!(tpco_payload(&tpdo), &[0x5A]);

    shutdown.store(true, Ordering::Relaxed);
    server.join().unwrap().expect("server io");
}

/// Helper so the assertion above stays honest about payload extraction.
fn tpco_payload(rx: &canopen_host::transport::Received) -> &[u8] {
    rx.payload()
}
