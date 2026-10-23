//! Port of `ref/canopen/test/test_pdo.py`, `test_sync.py`, `test_time.py`
//! subsets — PDO config via SDO, SYNC-triggered TPDO, RPDO write-back and
//! TIME codec round-trip (docs/EXTENSIONS.md 批次 D P-c).
//!
//! NOTE (docs/DESIGN-DECISIONS.md M9): the golden frames below are hand-transcribed from
//! python-canopen's suite and are **not** re-checked by `cargo test` against
//! `ref/`. After touching any codec, run
//! `python3 tools/interop/python_canopen_oracle.py` (8/8) to re-validate the
//! transcription against the reference implementation.

use canopen_core::datatypes::{DataType, Value};
use canopen_core::node::Node;
use canopen_core::object_dictionary::{AccessType, Entry, ObjectDictionary, Storage};
use canopen_core::pdo::MappingEntry;
use canopen_core::sdo::SdoClient;
use canopen_core::standard::StandardObjects;
use canopen_core::sync_time::{decode_sync, encode_sync, TimeStamp};
use canopen_core::types::{Address, NodeId};

const NODE: u8 = 5;

fn test_node() -> Node {
    let mut od = StandardObjects::new().device_type(0x1234).build();
    canopen_core::emcy::install_standard_objects(&mut od, NodeId::new(NODE).unwrap());

    for (sub, dt) in [(1u8, DataType::Unsigned8), (2, DataType::Unsigned16)] {
        od.insert(
            Address::new(0x6000, sub),
            Entry::new("app", dt, AccessType::Rw, Storage::Value(Value::Uint(0))).pdo_mappable(),
        );
    }
    // PDO parameter objects.
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
        for s in 1..=4u16 {
            put(&mut od, Address::new(map, s as u8), 0, DataType::Unsigned32);
        }
    }
    Node::new(NodeId::new(NODE).unwrap(), od).unwrap()
}

fn sdo_write(node: &mut Node, addr: Address, raw: &[u8]) {
    let mut client = SdoClient::new();
    let first = client.write(addr, raw).unwrap();
    let mut q: std::collections::VecDeque<[u8; 8]> = first.into();
    while let Some(f) = q.pop_front() {
        for r in node.on_frame(0x600 + u16::from(NODE), &f) {
            if let SdoEvent::Send(frames) = client.on_response(&r.data).expect("valid response") {
                q.extend(frames);
            }
        }
    }
}

use canopen_core::sdo::SdoEvent;

#[test]
fn pdo_config_via_sdo_then_sync_trigger() {
    let mut node = test_node();
    node.boot().unwrap();

    // Configure TPDO1: COB-ID valid, sync every 1, mapping 0x6000:1 (8 bit).
    sdo_write(&mut node, Address::new(0x1800, 1), &0x185u32.to_le_bytes());
    sdo_write(&mut node, Address::new(0x1800, 2), &[1]);
    sdo_write(&mut node, Address::new(0x1A00, 0), &[1]);
    sdo_write(
        &mut node,
        Address::new(0x1A00, 1),
        &MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32()
        .to_le_bytes(),
    );

    // Set application data.
    sdo_write(&mut node, Address::new(0x6000, 1), &[0x42]);

    // Start + SYNC → TPDO emitted.
    let _ = node.on_frame(0x000, &[0x01, NODE]); // start
    let (sync_payload, sync_len) = canopen_core::sync_time::encode_sync(None);
    let _ = sync_len;
    let tpdos: Vec<_> = node
        .on_frame(0x080, &sync_payload[..0])
        .into_iter()
        .filter(|f| f.cob_id == 0x185)
        .collect();
    assert_eq!(tpdos.len(), 1);
    assert_eq!(tpdos[0].payload(), &[0x42]);
}

#[test]
fn rpdo_writes_back_into_od() {
    let mut node = test_node();
    boot_and_start(&mut node);

    // Configure RPDO1: COB-ID valid 0x205, mapping 0x6000:2 (16 bits).
    sdo_write(&mut node, Address::new(0x1400, 1), &0x205u32.to_le_bytes());
    sdo_write(&mut node, Address::new(0x1600, 0), &[1]);
    sdo_write(
        &mut node,
        Address::new(0x1600, 1),
        &MappingEntry {
            address: Address::new(0x6000, 2),
            bit_length: 16,
        }
        .to_u32()
        .to_le_bytes(),
    );

    // Send an RPDO frame → OD value updated.
    let _ = node.on_frame(0x205, &[0x34, 0x12]);
    assert_eq!(
        node.od().read(Address::new(0x6000, 2)).unwrap(),
        Value::Uint(0x1234)
    );
}

fn boot_and_start(node: &mut Node) {
    node.boot().unwrap();
    let _ = node.on_frame(0x000, &[0x01, NODE]);
}

// --- SYNC / TIME codec ports ---------------------------------------------------

/// test_sync.py — SYNC producer transmit with/without counter.
#[test]
fn sync_counter_roundtrip() {
    assert_eq!(decode_sync(&[]), Some(None));
    assert_eq!(decode_sync(&[42]), Some(Some(42)));
    let (data, len) = encode_sync(Some(7));
    assert_eq!(len, 1);
    assert_eq!(decode_sync(&data[..len]), Some(Some(7)));
}

/// test_time.py — TIME_OF_DAY round-trip through the 6-byte payload.
#[test]
fn time_payload_roundtrip() {
    // 2000-01-01T00:00:00Z = 946684800 unix seconds.
    let ts = TimeStamp::from_unix_ms(946_684_800_000).unwrap();
    assert_eq!(ts.days_since_1984, 5844);
    assert_eq!(ts.ms_of_day, 0);

    let payload = ts.encode();
    assert_eq!(TimeStamp::decode(&payload[..6]).unwrap(), ts);
}
