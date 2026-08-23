//! Full-flow node integration tests: boot → SDO-driven PDO configuration →
//! NMT start → RPDO in / SYNC / TPDO out → heartbeat timing → EMCY — all
//! without a bus (client ↔ `Node` loopback).

use canopen_core::datatypes::{DataType, Value};
use canopen_core::emcy::{err_reg, install_standard_objects};
use canopen_core::node::{Node, TxFrame};
use canopen_core::object_dictionary::{AccessType, Entry, ObjectDictionary, Storage};
use canopen_core::pdo::MappingEntry;
use canopen_core::sdo::{SdoClient, SdoEvent};
use canopen_core::standard::StandardObjects;
use canopen_core::types::{Address, NodeId};

const NODE: u8 = 5;

/// Installs communication/mapping records for `count` RPDOs and TPDOs,
/// COB-IDs marked invalid with their pre-defined-connection-set defaults.
fn install_pdo_objects(od: &mut ObjectDictionary, tpdo_count: u16, rpdo_count: u16) {
    let put = |od: &mut ObjectDictionary, addr: Address, v: u64, dt: DataType| {
        od.insert(
            addr,
            Entry::new("", dt, AccessType::Rw, Storage::Value(Value::Uint(v))),
        );
    };
    for n in 0..rpdo_count {
        let comm = 0x1400 + n;
        let map = 0x1600 + n;
        let cob =
            canopen_core::pdo::default_cob_id(false, (n + 1) as u8, NodeId::new(NODE).unwrap())
                .unwrap_or(0) as u64
                | 0x8000_0000;
        put(od, Address::new(comm, 1), cob, DataType::Unsigned32);
        put(od, Address::new(comm, 2), 254, DataType::Unsigned8);
        put(od, Address::new(comm, 3), 0, DataType::Unsigned16);
        put(od, Address::new(comm, 5), 0, DataType::Unsigned16);
        put(od, Address::new(map, 0), 0, DataType::Unsigned8);
        for sub in 1..=8u16 {
            put(od, Address::new(map, sub as u8), 0, DataType::Unsigned32);
        }
    }
    for n in 0..tpdo_count {
        let comm = 0x1800 + n;
        let map = 0x1A00 + n;
        let cob = canopen_core::pdo::default_cob_id(true, (n + 1) as u8, NodeId::new(NODE).unwrap())
            .unwrap_or(0) as u64
            | 0x8000_0000;
        put(od, Address::new(comm, 1), cob, DataType::Unsigned32);
        put(od, Address::new(comm, 2), 254, DataType::Unsigned8);
        put(od, Address::new(comm, 3), 0, DataType::Unsigned16);
        put(od, Address::new(comm, 5), 0, DataType::Unsigned16);
        put(od, Address::new(map, 0), 0, DataType::Unsigned8);
        for sub in 1..=8u16 {
            put(od, Address::new(map, sub as u8), 0, DataType::Unsigned32);
        }
    }
}

fn test_od_dict() -> ObjectDictionary {
    let mut od = StandardObjects::new().device_type(0x1234).build();
    install_standard_objects(&mut od, NodeId::new(NODE).unwrap());
    install_pdo_objects(&mut od, 2, 1);
    for (sub, dt) in [(1u8, DataType::Unsigned8), (2, DataType::Unsigned16)] {
        od.insert(
            Address::new(0x6000, sub),
            Entry::new("app", dt, AccessType::Rw, Storage::Value(Value::Uint(0))).pdo_mappable(),
        );
    }
    od
}

fn test_node() -> Node {
    let mut od = StandardObjects::new().device_type(0x1234).build();
    install_standard_objects(&mut od, NodeId::new(NODE).unwrap());
    install_pdo_objects(&mut od, 2, 1);
    // Application data (mappable).
    for (sub, dt) in [(1u8, DataType::Unsigned8), (2, DataType::Unsigned16)] {
        od.insert(
            Address::new(0x6000, sub),
            Entry::new("app", dt, AccessType::Rw, Storage::Value(Value::Uint(0))).pdo_mappable(),
        );
    }
    Node::new(NodeId::new(NODE).unwrap(), od).unwrap()
}

/// Reads a Uint value straight from the node's dictionary (test helper).
fn od_value(node: &Node, addr: Address) -> u64 {
    match node.od().read(addr) {
        Ok(Value::Uint(v)) => v,
        other => panic!("expected Uint at {addr:?}, got {other:?}"),
    }
}

/// Relays a client transaction against the node until the client finishes.
fn transact(node: &mut Node, client: &mut SdoClient, first: Vec<[u8; 8]>) -> Option<Value> {
    let mut queue: std::collections::VecDeque<[u8; 8]> = first.into();
    let mut completed = None;
    while let Some(frame) = queue.pop_front() {
        for resp in node.on_frame(0x600 + u16::from(NODE), &frame) {
            match client.on_response(&resp.data).expect("valid response") {
                SdoEvent::Send(frames) => queue.extend(frames),
                SdoEvent::Complete(value, trailing) => {
                    completed = Some(value);
                    queue.extend(trailing);
                }
                SdoEvent::Done => {}
            }
        }
    }
    completed
}

fn sdo_write_u32(node: &mut Node, addr: Address, value: u32) {
    sdo_write_raw(node, addr, &value.to_le_bytes());
}

fn sdo_write_u16(node: &mut Node, addr: Address, value: u16) {
    sdo_write_raw(node, addr, &value.to_le_bytes());
}

fn sdo_write_u8(node: &mut Node, addr: Address, value: u8) {
    sdo_write_raw(node, addr, &[value]);
}

fn sdo_write_raw(node: &mut Node, addr: Address, raw: &[u8]) {
    let mut client = SdoClient::new();
    let first = client.write(addr, raw).unwrap();
    assert!(transact(node, &mut client, first).is_none());
}

#[test]
fn bootup_frame_and_nmt_lifecycle() {
    let mut node = test_node();
    // Boot: PreOperational + one boot-up frame on 0x700+id.
    let boot = node.boot().unwrap();
    assert_eq!(boot.cob_id, 0x705);
    assert_eq!(boot.payload(), &[0x00]);
    assert_eq!(
        node.nmt_state(),
        canopen_core::nmt::NmtState::PreOperational
    );

    // Broadcast start.
    let _ = node.on_frame(0x000, &[0x01, 0x00]);
    assert_eq!(node.nmt_state(), canopen_core::nmt::NmtState::Operational);

    // Directed stop.
    let _ = node.on_frame(0x000, &[0x02, NODE]);
    assert_eq!(node.nmt_state(), canopen_core::nmt::NmtState::Stopped);

    // Command for another node is ignored.
    let _ = node.on_frame(0x000, &[0x01, 9]);
    assert_eq!(node.nmt_state(), canopen_core::nmt::NmtState::Stopped);

    // Reset communication returns to Initialising and drops the SDO server.
    let _ = node.on_frame(0x000, &[0x82, 0x00]);
    assert_eq!(node.nmt_state(), canopen_core::nmt::NmtState::Initialising);

    // SDO is not served while initialising.
    let mut client = SdoClient::new();
    let req = client.read(Address::new(0x1000, 0), DataType::Unsigned32);
    assert!(node.on_frame(0x600 + u16::from(NODE), &req[0]).is_empty());
}

#[test]
fn sdo_configures_pdos_and_data_flows_end_to_end() {
    let mut node = test_node();
    node.boot().unwrap();

    // --- Configure TPDO1 via SDO (live reconfiguration path) ---------------
    // COB-ID valid = default TPDO1 for node 5; transmission type = SYNC every
    // 2nd sync; mapping = 0x6000:01 (8 bits) + 0x6000:02 (16 bits).
    sdo_write_u32(&mut node, Address::new(0x1800, 1), 0x185);
    sdo_write_u8(&mut node, Address::new(0x1800, 2), 2);
    sdo_write_u8(&mut node, Address::new(0x1A00, 0), 2);
    sdo_write_u32(
        &mut node,
        Address::new(0x1A00, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    sdo_write_u32(
        &mut node,
        Address::new(0x1A00, 2),
        MappingEntry {
            address: Address::new(0x6000, 2),
            bit_length: 16,
        }
        .to_u32(),
    );

    // --- Configure RPDO1 ----------------------------------------------------
    sdo_write_u32(&mut node, Address::new(0x1400, 1), 0x205);
    sdo_write_u8(&mut node, Address::new(0x1400, 2), 254);
    sdo_write_u8(&mut node, Address::new(0x1600, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1600, 1),
        MappingEntry {
            address: Address::new(0x6000, 2),
            bit_length: 16,
        }
        .to_u32(),
    );

    // Not started yet: RPDO frames are ignored in PreOperational.
    let _ = node.on_frame(0x205, &[0x99, 0x98]);
    assert_eq!(
        node.od().read(Address::new(0x6000, 2)).unwrap(),
        Value::Uint(0)
    );

    // Start the node.
    let _ = node.on_frame(0x000, &[0x01, 0x00]);
    assert_eq!(node.nmt_state(), canopen_core::nmt::NmtState::Operational);

    // --- RPDO in -------------------------------------------------------------
    let _ = node.on_frame(0x205, &[0x34, 0x12]);
    assert_eq!(
        node.od().read(Address::new(0x6000, 2)).unwrap(),
        Value::Uint(0x1234)
    );

    // --- SYNC-driven TPDO out (every 2nd sync, per configuration) -----------
    node.od_mut()
        .write(Address::new(0x6000, 1), Value::Uint(7))
        .unwrap();

    let (sync_payload, sync_len) = canopen_core::sync_time::encode_sync(None);
    let _ = sync_len;
    // First SYNC: no transmission yet (divider = 2).
    assert!(!node
        .on_frame(0x080, &sync_payload[..0])
        .iter()
        .any(|f| f.cob_id == 0x185));
    // Second SYNC: TPDO1 with packed payload [07, 34, 12].
    let tpdos: Vec<TxFrame> = node
        .on_frame(0x080, &sync_payload[..0])
        .into_iter()
        .filter(|f| f.cob_id == 0x185)
        .collect();
    assert_eq!(tpdos.len(), 1);
    assert_eq!(tpdos[0].payload(), &[0x07, 0x34, 0x12]);

    // --- Event-driven TPDO via request + poll --------------------------------
    // Reconfigure TPDO2 as event-driven with inhibit = 10 x 100 µs = 1 ms...
    sdo_write_u32(&mut node, Address::new(0x1801, 1), 0x285);
    sdo_write_u8(&mut node, Address::new(0x1801, 2), 254);
    sdo_write_u16(&mut node, Address::new(0x1801, 3), 100); // 100 * 100 µs = 10 ms
    sdo_write_u8(&mut node, Address::new(0x1A01, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1A01, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );

    assert!(node.request_tpdo(2));
    let now = 1000;
    let frames: Vec<TxFrame> = node
        .poll(now)
        .into_iter()
        .filter(|f| f.cob_id == 0x285)
        .collect();
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].payload(), &[0x07]);

    // Inhibit window blocks immediate resend even if requested again.
    node.request_tpdo(2);
    let frames: Vec<TxFrame> = node
        .poll(now + 5)
        .into_iter()
        .filter(|f| f.cob_id == 0x285)
        .collect();
    assert!(frames.is_empty());
    // After the window passes the pending event goes out.
    let frames: Vec<TxFrame> = node
        .poll(now + 150)
        .into_iter()
        .filter(|f| f.cob_id == 0x285)
        .collect();
    assert_eq!(frames.len(), 1);
}

#[test]
fn heartbeat_is_produced_on_schedule() {
    let mut node = test_node();
    node.boot().unwrap();
    // Heartbeat period 50 ms (UNSIGNED16).
    sdo_write_u16(&mut node, Address::new(0x1017, 0), 50);

    assert!(node.poll(0).iter().any(|f| f.cob_id == 0x705));
    assert!(!node.poll(30).iter().any(|f| f.cob_id == 0x705));
    let beats: Vec<TxFrame> = node
        .poll(60)
        .into_iter()
        .filter(|f| f.cob_id == 0x705)
        .collect();
    assert_eq!(beats.len(), 1);
    assert_eq!(beats[0].payload()[0], 0x7F); // pre-operational at that point

    // After start the state byte reflects Operational.
    let _ = node.on_frame(0x000, &[0x01, 0x00]);
    let beats: Vec<TxFrame> = node
        .poll(120)
        .into_iter()
        .filter(|f| f.cob_id == 0x705)
        .collect();
    assert_eq!(beats[0].payload()[0], 0x05);

    // Period 0 disables production.
    sdo_write_u16(&mut node, Address::new(0x1017, 0), 0);
    assert!(!node.poll(10_000).iter().any(|f| f.cob_id == 0x705));
}

#[test]
fn guarding_rtr_poll_alternates_toggle() {
    let mut node = test_node();
    node.boot().unwrap();
    let r1 = node.on_remote_request(0x705).unwrap();
    assert_eq!(r1.payload(), &[0x7F]); // toggle false
    let r2 = node.on_remote_request(0x705).unwrap();
    assert_eq!(r2.payload(), &[0xFF]); // toggle true (bit 7)
    let r3 = node.on_remote_request(0x705).unwrap();
    assert_eq!(r3.payload(), &[0x7F]);
}

#[test]
fn emergency_flow_updates_od_and_frames() {
    let mut node = test_node();
    node.boot().unwrap();

    let emcy = node
        .raise_emergency(0x2310, err_reg::CURRENT, [1, 2, 3, 4])
        .unwrap();
    assert_eq!(emcy.cob_id, 0x85);
    assert_eq!(&emcy.payload()[..3], &[0x10, 0x23, err_reg::CURRENT]);

    // Idempotent re-raise produces nothing.
    assert!(node
        .raise_emergency(0x2310, err_reg::CURRENT, [0; 4])
        .is_none());

    // Error register mirrored into OD.
    assert_eq!(
        node.od().read(Address::new(0x1001, 0)).unwrap(),
        Value::Uint(u64::from(err_reg::CURRENT))
    );

    let reset = node.clear_emergency(0x2310).unwrap();
    assert_eq!(reset.payload()[0], 0x00); // code 0x0000
}

// --- LSS integration -----------------------------------------------------------

use canopen_core::lss::{
    decode_configure_node_id_response, encode_configure_node_id, encode_store,
    encode_switch_selective, FastscanMaster, LssAddress, LSS_MASTER_COB_ID, LSS_SLAVE_COB_ID,
};

fn lss_address() -> LssAddress {
    LssAddress {
        vendor_id: 0x00C0_FFEE,
        product_code: 42,
        revision_number: 1,
        serial_number: 0xDEAD_BEEF,
    }
}

/// Full unconfigured-device lifecycle: discovery → assignment → adoption.
#[test]
fn lss_unconfigured_lifecycle() {
    let mut node = Node::new_unconfigured(lss_address(), test_od_dict());

    // While unconfigured, only LSS is served — SDO/NMT frames are ignored.
    assert!(node.on_frame(0x000, &[0x01, 0x00]).is_empty());
    let mut client = SdoClient::new();
    let req = client.read(Address::new(0x1000, 0), DataType::Unsigned32);
    assert!(node.on_frame(0x600 + u16::from(NODE), &req[0]).is_empty());

    // Master discovers the device by fastscan through the node itself.
    let mut master = FastscanMaster::new();
    while let Some(req) = master.next_request() {
        let answered = node
            .on_frame(LSS_MASTER_COB_ID, &req)
            .first()
            .map(|f| f.cob_id == LSS_SLAVE_COB_ID)
            .unwrap_or(false);
        master.on_response(answered);
    }
    assert!(master.found() && master.is_complete());
    assert_eq!(master.address(), lss_address());

    // Assign a node-id and store it.
    let resp = node
        .on_frame(LSS_MASTER_COB_ID, &encode_configure_node_id(0x21))
        .remove(0);
    assert_eq!(decode_configure_node_id_response(&resp.data), Some(Ok(())));
    let _ = node.on_frame(LSS_MASTER_COB_ID, &encode_store());

    // Reset path adopts the pending id; the node becomes fully functional.
    let new_id = node.apply_lss_node_id().unwrap();
    assert_eq!(new_id.get(), 0x21);
    node.boot().unwrap();

    // SDO now answers on the new COB-IDs.
    let mut client2 = SdoClient::new();
    let req = client2.read(Address::new(0x1000, 0), DataType::Unsigned32);
    let responses = node.on_frame(0x600 + u16::from(new_id.get()), &req[0]);
    assert_eq!(responses.len(), 1);
    assert_eq!(responses[0].cob_id, 0x580 + u16::from(new_id.get()));
}

/// A configured node with an attached LSS slave still accepts selective
/// reconfiguration (e.g. changing its id in the field).
#[test]
fn lss_selective_reconfiguration_on_configured_node() {
    let mut node = test_node();
    node.attach_lss(lss_address());

    for f in encode_switch_selective(&lss_address()) {
        let _ = node.on_frame(LSS_MASTER_COB_ID, &f);
    }
    let resp = node
        .on_frame(LSS_MASTER_COB_ID, &encode_configure_node_id(9))
        .remove(0);
    assert_eq!(decode_configure_node_id_response(&resp.data), Some(Ok(())));
    let _ = node.on_frame(LSS_MASTER_COB_ID, &encode_store());
    let new_id = node.apply_lss_node_id().unwrap();
    assert_eq!(new_id.get(), 9);
}

#[test]
fn heartbeat_silent_during_initialising() {
    // BUGS.md C2: the periodic producer must not emit boot-up frames while
    // the node sits in Initialising (only `boot()` sends the single 0x00).
    let mut node = test_node();
    node.boot().unwrap(); // SDO (and thus 0x1017) is served from PreOp on.
    sdo_write_u16(&mut node, Address::new(0x1017, 0), 10);
    // Re-enter Initialising via NMT reset; the producer must stay silent.
    let _ = node.on_frame(0x000, &[0x81, NODE]); // Reset Communication
    for ms in [0u64, 20, 40, 100] {
        let beats: Vec<TxFrame> = node
            .poll(ms)
            .into_iter()
            .filter(|f| f.cob_id == 0x705)
            .collect();
        assert!(beats.is_empty(), "Initialising produced {beats:?}");
    }
    // Boot emits exactly one boot-up frame and starts the producer.
    assert_eq!(node.boot().unwrap().payload(), &[0x00]);
    assert!(node.poll(200).iter().any(|f| f.cob_id == 0x705));
}

#[test]
fn writing_zero_to_1003_0_clears_emcy_history() {
    // BUGS.md M4 (CANopenNode OD_write_1003): sub-0 write of 0 wipes the
    // history; non-zero writes leave the live count authoritative.
    let mut node = test_node();
    node.boot().unwrap();
    node.raise_emergency(0x2310, err_reg::GENERIC, [1, 2, 3, 4]);
    node.raise_emergency(0x8110, err_reg::GENERIC, [0; 4]);
    assert_eq!(od_value(&node, Address::new(0x1003, 0)), 2);

    sdo_write_u8(&mut node, Address::new(0x1003, 0), 0);
    assert_eq!(od_value(&node, Address::new(0x1003, 0)), 0);
    for sub in 1..=8u8 {
        assert_eq!(od_value(&node, Address::new(0x1003, sub)), 0);
    }

    // A bogus non-zero write does not corrupt the count.
    sdo_write_u8(&mut node, Address::new(0x1003, 0), 42);
    assert_eq!(od_value(&node, Address::new(0x1003, 0)), 0);
}

#[test]
fn emcy_cob_id_follows_od_1014() {
    // BUGS.md M5: producer honours the writable COB-ID entry.
    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_u32(&mut node, Address::new(0x1014, 0), 0x0000_00C1); // 0xC1 valid
    let f = node.raise_emergency(0x5000, 0x01, [0; 4]).unwrap();
    assert_eq!(f.cob_id, 0xC1);

    // bit31 set → EMCY production disabled.
    sdo_write_u32(&mut node, Address::new(0x1014, 0), 0x8000_00C1);
    assert!(node.clear_emergency(0x5000).is_none());
    assert!(node.raise_emergency(0x6000, 0x01, [0; 4]).is_none());
}

#[test]
fn pdo_reconfiguration_errors_are_captured_not_swallowed() {
    // BUGS.md m17: a broken PDO configuration must leave a trace in
    // `take_config_errors()` instead of disappearing silently.
    let mut node = test_node();
    node.boot().unwrap();

    // Corrupt the RPDO1 mapping record: target object does not exist, so
    // slot assembly fails and the error must be recorded.
    sdo_write_u32(&mut node, Address::new(0x1400, 1), 0x205);
    sdo_write_u8(&mut node, Address::new(0x1600, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1600, 1),
        MappingEntry {
            address: Address::new(0x7777, 0),
            bit_length: 8,
        }
        .to_u32(),
    );

    assert!(
        !node.take_config_errors().collect::<Vec<_>>().is_empty(),
        "broken mapping must be reported"
    );
    // Drained: a second call yields nothing new.
    assert!(node.take_config_errors().next().is_none());
}
