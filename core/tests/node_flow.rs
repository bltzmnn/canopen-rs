//! Full-flow node integration tests: boot → SDO-driven PDO configuration →
//! NMT start → RPDO in / SYNC / TPDO out → heartbeat timing → EMCY — all
//! without a bus (client ↔ `Node` loopback).

use canopen_core::datatypes::{DataType, Value};
use canopen_core::emcy::{err_reg, install_standard_objects};
use canopen_core::node::{Node, TxFrame};
use canopen_core::object_dictionary::{AccessType, Entry, ObjectDictionary, Storage};
use canopen_core::pdo::MappingEntry;
use canopen_core::sdo::{SdoAbortCode, SdoClient, SdoEvent};
use canopen_core::standard::StandardObjects;
use canopen_core::types::{Address, Error, NodeId};

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

    // Reset communication: the node re-enters Initialising and immediately
    // re-boots (BUGS.md H2) — the reset hand-back IS the boot-up frame and
    // the node returns to PreOperational, so a master can start it again.
    let reset_out = node.on_frame(0x000, &[0x82, 0x00]);
    assert_eq!(reset_out.len(), 1, "reset re-boots the node");
    assert_eq!(reset_out[0].cob_id, 0x705);
    assert_eq!(reset_out[0].payload(), &[0x00]);
    assert_eq!(
        node.nmt_state(),
        canopen_core::nmt::NmtState::PreOperational
    );

    // SDO is served again from PreOperational.
    let mut client = SdoClient::new();
    let req = client.read(Address::new(0x1000, 0), DataType::Unsigned32);
    assert!(!node.on_frame(0x600 + u16::from(NODE), &req[0]).is_empty());
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
    // Reconfigure TPDO2 as event-driven with inhibit = 100 x 100 µs = 10 ms...
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
    // Preload 0x1017 = 50 ms (UNSIGNED16) like an EDS default, so this test
    // covers the boot-up anchor (BUGS.md L4) without the write-triggered
    // immediate frame of BUGS.md B4.
    node.od_mut()
        .set_value(Address::new(0x1017, 0), Value::Uint(50))
        .unwrap();
    node.boot().unwrap();

    // BUGS.md L4: the boot-up frame itself is the first heartbeat (0x00),
    // so the periodic producer does not fire an immediate second frame —
    // the first periodic heartbeat lands only after one full period.
    assert!(!node.poll(0).iter().any(|f| f.cob_id == 0x705));
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
    node.od_mut()
        .set_value(Address::new(0x1017, 0), Value::Uint(0))
        .unwrap();
    assert!(!node.poll(10_000).iter().any(|f| f.cob_id == 0x705));
}

/// BUGS.md B4: writing 0x1017 restarts the producer and emits one heartbeat
/// at the next poll (CANopenNode `CO_NMT_Heartbeat.c:64-66,187-189`).
#[test]
fn writing_producer_heartbeat_time_sends_immediately() {
    let mut node = test_node();
    node.boot().unwrap();
    assert!(!node.poll(0).iter().any(|f| f.cob_id == 0x705));

    sdo_write_u16(&mut node, Address::new(0x1017, 0), 50);
    let beats: Vec<TxFrame> = node
        .poll(10)
        .into_iter()
        .filter(|f| f.cob_id == 0x705)
        .collect();
    assert_eq!(beats.len(), 1, "write must trigger an immediate heartbeat");
    assert_eq!(beats[0].payload()[0], 0x7F);

    // The following period is anchored at that send.
    assert!(!node.poll(30).iter().any(|f| f.cob_id == 0x705));
    assert!(node.poll(60).iter().any(|f| f.cob_id == 0x705));
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

/// BUGS.md L1: the node-guarding toggle resets to 0 on *every* NMT state
/// change (CiA 301 §7.3.1), not only on resets — a master polling across a
/// Stop/Start cycle must see the toggle restart, not a stale value that
/// would look like a missed toggle.
#[test]
fn guarding_toggle_keeps_alternating_across_nmt_transition() {
    let mut node = test_node();
    node.boot().unwrap();
    // First response carries toggle 0 and arms the toggle at 1.
    assert_eq!(node.on_remote_request(0x705).unwrap().payload(), &[0x7F]);
    // BUGS.md B5: an NMT Stop must *not* reset the toggle — the reference
    // master enforces strict alternation (`CO_Node_Guarding.c:255-259`), so
    // the next response still carries the toggled bit (old behavior reset it
    // to 0 here, which a real master reads as a lost toggle).
    let _ = node.on_frame(0x000, &[0x02, 0x00]); // Stop
    let stopped = node.on_remote_request(0x705).unwrap();
    assert_eq!(stopped.payload(), &[0x84]); // Stopped + toggled bit set
    assert_eq!(node.on_remote_request(0x705).unwrap().payload(), &[0x04]);
}

/// BUGS.md B3: node guarding and heartbeat are mutually exclusive; once the
/// heartbeat producer is enabled (0x1017 > 0) an RTR is not answered.
#[test]
fn guarding_disabled_while_heartbeat_enabled() {
    let mut node = test_node();
    node.boot().unwrap();
    assert!(node.on_remote_request(0x705).is_some());
    sdo_write_u16(&mut node, Address::new(0x1017, 0), 50);
    assert!(node.on_remote_request(0x705).is_none());
}

#[test]
fn emergency_flow_updates_od_and_frames() {
    let mut node = test_node();
    node.boot().unwrap();

    let emcy = node
        .raise_emergency(0x2310, err_reg::CURRENT, [1, 2, 3, 4, 5])
        .unwrap();
    assert_eq!(emcy.cob_id, 0x85);
    assert_eq!(&emcy.payload()[..3], &[0x10, 0x23, err_reg::CURRENT]);

    // Idempotent re-raise produces nothing.
    assert!(node
        .raise_emergency(0x2310, err_reg::CURRENT, [0; 5])
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
    encode_switch_global, encode_switch_selective, FastscanMaster, LssAddress, LSS_MASTER_COB_ID,
    LSS_SLAVE_COB_ID,
};

fn lss_address() -> LssAddress {
    LssAddress {
        vendor_id: 0x00C0_FFEE,
        product_code: 42,
        revision_number: 1,
        serial_number: 0xDEAD_BEEF,
    }
}

/// BUGS.md 八L17: an unconfigured node (CiA 305, LSS only) must not emit a
/// boot-up frame on the placeholder node-1 heartbeat COB.
#[test]
fn unconfigured_node_does_not_boot_up() {
    let mut node = Node::new_unconfigured(lss_address(), test_od_dict());
    assert!(
        node.boot().is_none(),
        "unconfigured node must not send a boot-up frame"
    );
}

/// BUGS.md 八L15: the TIME receive queue is bounded — a chatty 0x100
/// producer while the application never drains must not grow memory
/// without bound; the oldest frames get dropped.
#[test]
fn time_rx_queue_is_bounded() {
    let mut node = test_node();
    node.boot().unwrap();
    for i in 0..400u32 {
        let mut f = [0u8; 6];
        f[..4].copy_from_slice(&i.to_le_bytes());
        let _ = node.on_frame(0x100, &f);
    }
    let events = node.take_time_events();
    assert_eq!(events.len(), 256, "oldest TIME frames dropped at the cap");
    assert!(!events.is_empty());
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
    // BUGS.md P2-2 (see git history): 0x1280:1 must follow the new node segment on
    // the LSS adoption path too (previously only set_node_id did).
    assert_eq!(
        od_value(&node, Address::new(0x1280, 1)),
        u64::from(0x600u16 + u16::from(new_id.get()))
    );
    node.boot().unwrap();

    // SDO now answers on the new COB-IDs.
    let mut client2 = SdoClient::new();
    let req = client2.read(Address::new(0x1000, 0), DataType::Unsigned32);
    let responses = node.on_frame(0x600 + u16::from(new_id.get()), &req[0]);
    assert_eq!(responses.len(), 1);
    assert_eq!(responses[0].cob_id, 0x580 + u16::from(new_id.get()));
}

/// BUGS.md 十M22: switching a configured-but-not-adopted node back to
/// Waiting must autonomously run an NMT communication reset and adopt the
/// pending id (CANopenNode `CO_LSSslave.c:56-66,275-280`).
#[test]
fn lss_switch_global_waiting_adopts_pending_and_boots() {
    let mut node = Node::new_unconfigured(lss_address(), test_od_dict());
    for f in encode_switch_selective(&lss_address()) {
        let _ = node.on_frame(LSS_MASTER_COB_ID, &f);
    }
    let resp = node
        .on_frame(LSS_MASTER_COB_ID, &encode_configure_node_id(0x21))
        .remove(0);
    assert_eq!(decode_configure_node_id_response(&resp.data), Some(Ok(())));

    // Switch back to Waiting: the node adopts 0x21 and emits boot-up on
    // 0x700+0x21.
    let frames = node.on_frame(LSS_MASTER_COB_ID, &encode_switch_global(false));
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].cob_id, 0x700 + 0x21);
    assert_eq!(frames[0].payload(), &[0x00]);
    assert_eq!(node.node_id().get(), 0x21);

    // SDO now answers on the new segment.
    let mut client = SdoClient::new();
    let req = client.read(Address::new(0x1000, 0), DataType::Unsigned32);
    let responses = node.on_frame(0x600 + 0x21, &req[0]);
    assert_eq!(responses.len(), 1);
}

/// BUGS.md A2: configuring `0xFF` invalidates the node-id (CiA 305 §5.4.1) —
/// the node returns to the LSS-init state, serves only LSS again and
/// suppresses its boot-up/data services.
#[test]
fn lss_configure_ff_unconfigures_node() {
    let mut node = test_node();
    node.attach_lss(lss_address());
    for f in encode_switch_selective(&lss_address()) {
        let _ = node.on_frame(LSS_MASTER_COB_ID, &f);
    }
    let resp = node
        .on_frame(LSS_MASTER_COB_ID, &encode_configure_node_id(0xFF))
        .remove(0);
    assert_eq!(decode_configure_node_id_response(&resp.data), Some(Ok(())));

    // Adoption invalidates rather than assigning a `NodeId`.
    assert!(node.apply_lss_node_id().is_none());
    assert!(node.boot().is_none(), "unconfigured nodes do not boot up");
    assert!(node.on_frame(0x000, &[0x01, 0x00]).is_empty());
    let mut client = SdoClient::new();
    let req = client.read(Address::new(0x1000, 0), DataType::Unsigned32);
    assert!(node.on_frame(0x600 + u16::from(NODE), &req[0]).is_empty());

    // LSS still works: a fresh fastscan finds the node again.
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

/// BUGS.md M4: COB 0x7E5 belongs exclusively to LSS. A frame on it that the
/// LSS handler does not consume (here: no LSS attached at all) must be
/// swallowed by the LSS block — never dispatched as an ordinary service. An
/// RPDO configured on that COB-ID must not receive it.
#[test]
fn lss_cob_is_never_delivered_as_regular_service() {
    let mut node = test_node();
    node.boot().unwrap();
    // Point RPDO1 at 0x7E5 and map 0x6000:1 (8 bits) into it.
    sdo_write_u32(&mut node, Address::new(0x1400, 1), 0x7E5);
    sdo_write_u8(&mut node, Address::new(0x1400, 2), 254);
    sdo_write_u8(&mut node, Address::new(0x1600, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1600, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    start_node(&mut node);

    // A frame on 0x7E5 (this node has no LSS slave attached, so the handler
    // consumes nothing): the LSS block must still claim the COB-ID — the
    // frame must not reach the RPDO matcher.
    assert!(node
        .on_frame(0x7E5, &[0xAB, 0, 0, 0, 0, 0, 0, 0])
        .is_empty());
    assert_eq!(od_value(&node, Address::new(0x6000, 1)), 0);
}

#[test]
fn heartbeat_silent_during_initialising() {
    // BUGS.md C2: the periodic producer must not emit boot-up frames while
    // the node sits in Initialising (only `boot()` sends the single 0x00).
    // A fresh node starts in Initialising (SDO is not served yet), so arm
    // 0x1017 directly in the OD instead of via SDO.
    let mut node = test_node();
    node.od_mut()
        .write(
            Address::new(0x1017, 0),
            canopen_core::datatypes::Value::Uint(10),
        )
        .unwrap();
    for ms in [0u64, 20, 40, 100] {
        let beats: Vec<TxFrame> = node
            .poll(ms)
            .into_iter()
            .filter(|f| f.cob_id == 0x705)
            .collect();
        assert!(beats.is_empty(), "Initialising produced {beats:?}");
    }
    // Boot emits exactly one boot-up frame and starts the producer.
    // BUGS.md 八M4: the first poll after boot anchors the heartbeat epoch;
    // the first periodic beat comes one full 0x1017 period later — never
    // back-to-back with boot-up.
    assert_eq!(node.boot().unwrap().payload(), &[0x00]);
    assert!(
        node.poll(200).iter().all(|f| f.cob_id != 0x705),
        "boot-up must not be followed immediately by a periodic heartbeat"
    );
    assert!(node.poll(210).iter().any(|f| f.cob_id == 0x705));
}

/// BUGS.md 八M4: on a real monotonic clock `now_ms` is far from zero when
/// the node boots (host serve loop). The old `last_hb_ms = Some(0)` made the
/// first periodic heartbeat due immediately at the first poll, emitting a
/// second frame back-to-back with boot-up. The epoch must instead anchor at
/// the first observed `now_ms`.
#[test]
fn boot_up_and_first_heartbeat_are_not_back_to_back() {
    let mut node = test_node();
    node.od_mut()
        .write(
            Address::new(0x1017, 0),
            canopen_core::datatypes::Value::Uint(100),
        )
        .unwrap();
    assert_eq!(node.boot().unwrap().payload(), &[0x00]);
    // Real-clock-like timestamp: 1_000_000 ms after some process start.
    let beats: Vec<TxFrame> = node
        .poll(1_000_000)
        .into_iter()
        .filter(|f| f.cob_id == 0x705)
        .collect();
    assert!(
        beats.is_empty(),
        "first poll after boot must only arm the producer: {beats:?}"
    );
    assert!(
        node.poll(1_000_100).iter().any(|f| f.cob_id == 0x705),
        "one full 0x1017 period after arming the beat must fire"
    );
}

#[test]
fn writing_zero_to_1003_0_clears_emcy_history() {
    // BUGS.md M4 (CANopenNode OD_write_1003): sub-0 write of 0 wipes the
    // history; non-zero writes leave the live count authoritative.
    let mut node = test_node();
    node.boot().unwrap();
    node.raise_emergency(0x2310, err_reg::GENERIC, [1, 2, 3, 4, 5]);
    node.raise_emergency(0x8110, err_reg::GENERIC, [0; 5]);
    assert_eq!(od_value(&node, Address::new(0x1003, 0)), 2);

    sdo_write_u8(&mut node, Address::new(0x1003, 0), 0);
    assert_eq!(od_value(&node, Address::new(0x1003, 0)), 0);
    for sub in 1..=8u8 {
        assert_eq!(od_value(&node, Address::new(0x1003, sub)), 0);
    }

    // A bogus non-zero write is rejected with VALUE_TOO_HIGH (BUGS.md
    // P3-39, matching CANopenNode `OD_write_1003`) and does not corrupt the
    // count.
    let mut client = SdoClient::new();
    let first = client.write(Address::new(0x1003, 0), &[42]).unwrap();
    let aborted = transact_expect_abort(&mut node, &mut client, first);
    assert_eq!(aborted, SdoAbortCode::ValueTooHigh);
    assert_eq!(od_value(&node, Address::new(0x1003, 0)), 0);
}

/// Drives a transaction expected to end in an SDO abort; returns the code.
fn transact_expect_abort(
    node: &mut Node,
    client: &mut SdoClient,
    first: Vec<[u8; 8]>,
) -> SdoAbortCode {
    let mut queue: std::collections::VecDeque<[u8; 8]> = first.into();
    while let Some(frame) = queue.pop_front() {
        for resp in node.on_frame(0x600 + u16::from(NODE), &frame) {
            match client.on_response(&resp.data) {
                Ok(SdoEvent::Send(frames)) => queue.extend(frames),
                Ok(_) => {}
                Err(Error::SdoAborted(code)) => return code,
                Err(e) => panic!("unexpected error {e:?}"),
            }
        }
    }
    panic!("transaction completed without abort");
}

#[test]
fn emcy_cob_id_follows_od_1014() {
    // BUGS.md M5 / 十M19: producer honours the writable COB-ID entry and
    // CANopenNode `OD_write_1014`'s guards, and a low-11-bit value of 0
    // disables production.
    let mut node = test_node();
    node.boot().unwrap();

    // While production is enabled (default 0x80+NODE) a COB-ID change is
    // rejected with InvalidValue (CANopenNode `CO_Emergency.c:72-102`).
    let mut client = SdoClient::new();
    let first = client
        .write(Address::new(0x1014, 0), &0x0000_00C1u32.to_le_bytes())
        .unwrap();
    assert_eq!(
        transact_expect_abort(&mut node, &mut client, first),
        SdoAbortCode::InvalidValue
    );

    // Disable (bit31) while storing the new COB-ID, then re-enable it.
    sdo_write_u32(&mut node, Address::new(0x1014, 0), 0x8000_00C1);
    sdo_write_u32(&mut node, Address::new(0x1014, 0), 0x0000_00C1); // 0xC1 valid
    let f = node.raise_emergency(0x5000, 0x01, [0; 5]).unwrap();
    assert_eq!(f.cob_id, 0xC1);

    // bit31 set → EMCY production disabled.
    sdo_write_u32(&mut node, Address::new(0x1014, 0), 0x8000_00C1);
    assert!(node.clear_emergency(0x5000).is_none());
    assert!(node.raise_emergency(0x6000, 0x01, [0; 5]).is_none());

    // Low 11 bits = 0 also disables production (read directly from the OD,
    // as the EDS preload path can leave such a value).
    node.od_mut()
        .set_value(Address::new(0x1014, 0), Value::Uint(0))
        .unwrap();
    assert!(node.raise_emergency(0x7000, 0x01, [0; 5]).is_none());
}

/// BUGS.md 八M3: with *only* the `StandardObjects` baseline (no
/// `emcy::install_standard_objects`), 0x1014 defaults to node 1's EMCY COB
/// 0x81. A node built at another id must rewrite it to `0x80+id` so its
/// emergency frames do not collide with node 1 on the bus.
#[test]
fn standard_objects_only_emcy_cob_follows_node_id() {
    use canopen_core::datatypes::DataType;
    use canopen_core::object_dictionary::{AccessType, Entry, Storage};

    // Baseline only — no EMCY install (which would already rewrite 0x1014).
    let mut od = StandardObjects::new().device_type(0x1234).build();
    od.insert(
        Address::new(0x1001, 0),
        Entry::new(
            "Error register",
            DataType::Unsigned8,
            AccessType::Rw,
            Storage::Value(Value::Uint(0)),
        ),
    );
    let mut node = Node::new(NodeId::new(NODE).unwrap(), od).unwrap();
    node.boot().unwrap();

    assert_eq!(
        od_value(&node, Address::new(0x1014, 0)),
        u64::from(0x80u16 + u16::from(NODE)),
        "0x1014 must follow the node id, not stay at node 1's 0x81"
    );
    let f = node.raise_emergency(0x5000, 0x01, [0; 5]).unwrap();
    assert_eq!(
        f.cob_id,
        0x80 + u16::from(NODE),
        "EMCY must be produced on 0x80+node"
    );
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

fn start_node(node: &mut Node) {
    // NMT StartRemoteNode, broadcast.
    let _ = node.on_frame(0x0000, &[0x01, 0x00]);
}

/// §7.4 / BUGS.md 八H1: RTR on a TPDO COB-ID answers with the current mapping
/// payload only when bit30 is *clear* (CiA 301: bit30 set = RTR not allowed,
/// same as python-canopen `RTR_NOT_ALLOWED = 1 << 30`); with bit30 set the
/// request stays unanswered.
#[test]
fn rtr_triggers_tpdo_when_bit30_clear() {
    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_raw(&mut node, Address::new(0x6000, 1), &[0xAB]);
    // TPDO1 @0x185, RTR-only-event (253), bit30 clear => RTR allowed.
    sdo_write_u32(&mut node, Address::new(0x1800, 1), 0x185);
    sdo_write_u8(&mut node, Address::new(0x1800, 2), 253);
    sdo_write_u8(&mut node, Address::new(0x1A00, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1A00, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    start_node(&mut node);

    let frame = node.on_remote_request(0x185).expect("RTR answered");
    assert_eq!(frame.cob_id, 0x185);
    assert_eq!(frame.payload(), &[0xAB]);

    // Same setup but WITH bit30 set (RTR not allowed): no answer.
    let mut node2 = test_node();
    node2.boot().unwrap();
    sdo_write_raw(&mut node2, Address::new(0x6000, 1), &[0xCD]);
    sdo_write_u32(&mut node2, Address::new(0x1800, 1), 0x185 | 0x4000_0000);
    sdo_write_u8(&mut node2, Address::new(0x1800, 2), 253);
    sdo_write_u8(&mut node2, Address::new(0x1A00, 0), 1);
    sdo_write_u32(
        &mut node2,
        Address::new(0x1A00, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    start_node(&mut node2);
    assert!(node2.on_remote_request(0x185).is_none());
}

/// BUGS.md 八M1: a COB-ID whose 11-bit identifier is 0 (bit31 clear, "valid")
/// must never assemble a PDO slot — TPDO frames would go to the NMT command
/// COB 0x000. SDO writes of such a COB-ID are rejected at write time
/// (CiA/CANopenNode); a dictionary that *already* carries the value (EDS
/// import path) is caught at slot assembly and reported via the
/// config-error ring.
#[test]
fn cob_id_zero_is_rejected_at_slot_build() {
    use canopen_core::datatypes::DataType;
    use canopen_core::object_dictionary::{AccessType, Entry, Storage};

    let mut node = test_node();
    node.boot().unwrap();
    // Direct OD write bypassing the SDO guard — as an EDS import would.
    node.od_mut().insert(
        Address::new(0x1800, 1),
        Entry::new(
            "COB-ID TPDO1",
            DataType::Unsigned32,
            AccessType::Rw,
            Storage::Value(Value::Uint(0x0000_0000)),
        ),
    );
    // Mapping record with one real entry so the slot would otherwise build.
    node.od_mut().insert(
        Address::new(0x1A00, 0),
        Entry::new(
            "",
            DataType::Unsigned8,
            AccessType::Rw,
            Storage::Value(Value::Uint(1)),
        ),
    );
    node.od_mut().insert(
        Address::new(0x1A00, 1),
        Entry::new(
            "",
            DataType::Unsigned32,
            AccessType::Rw,
            Storage::Value(Value::Uint(u64::from(
                MappingEntry {
                    address: Address::new(0x6000, 1),
                    bit_length: 8,
                }
                .to_u32(),
            ))),
        ),
    );
    // Force slot assembly from the (already poisoned) OD contents.
    node.configure_pdos_from_od().unwrap();
    start_node(&mut node);

    // The event-driven slot must not fire a frame on COB 0x000.
    for now in [1u64, 2, 3, 4, 5] {
        assert!(
            node.poll(now).is_empty(),
            "TPDO must not transmit on the NMT COB"
        );
    }
    let errs: Vec<_> = node.take_config_errors().collect();
    assert!(
        errs.iter().any(|e| format!("{e:?}").contains("0x000")),
        "COB-ID-0 misconfiguration must be reported: {errs:?}"
    );
}

/// SDO writes of COB-ID 0 are rejected at write time (BUGS.md 八M1).
#[test]
fn cob_id_zero_sdo_write_is_rejected() {
    let mut node = test_node();
    node.boot().unwrap();
    // TPDO1 COB-ID = 0x00000000 via SDO → must be aborted, never stored.
    let mut client = SdoClient::new();
    let first = client
        .write(Address::new(0x1800, 1), &0x0000_0000u32.to_le_bytes())
        .expect("initiate frame");
    // Walk the transaction until the server answers with an abort.
    let mut queue: std::collections::VecDeque<[u8; 8]> = first.into();
    let mut abort_code = None;
    while let Some(frame) = queue.pop_front() {
        for resp in node.on_frame(0x600 + u16::from(NODE), &frame) {
            if resp.data[0] & 0x80 != 0 {
                abort_code = Some(u32::from_le_bytes([
                    resp.data[4],
                    resp.data[5],
                    resp.data[6],
                    resp.data[7],
                ]));
                continue; // abort ends the transaction; client would Err
            }
            match client.on_response(&resp.data).expect("valid response") {
                SdoEvent::Send(frames) => queue.extend(frames),
                SdoEvent::Complete(_, trailing) => queue.extend(trailing),
                SdoEvent::Done => {}
            }
        }
    }
    assert_eq!(
        abort_code,
        Some(0x0609_0030),
        "COB-ID-0 SDO write must abort with InvalidValue (十L2)"
    );
    // The dictionary must still hold the previous (predefined, disabled)
    // COB-ID, not the rejected 0.
    assert_eq!(od_value(&node, Address::new(0x1800, 1)), 0x8000_0185);
}

/// BUGS.md M2: a type-252 (RtrOnlySync) TPDO answers a remote request
/// only after the first SYNC of the session — its payload is the last
/// SYNC sample (CiA 301 §7.4.3); before any SYNC there is no valid
/// sample to return.
#[test]
fn rtr_only_sync_waits_for_first_sync() {
    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_raw(&mut node, Address::new(0x6000, 1), &[0xAB]);
    // bit30 clear => RTR allowed (CiA 301，BUGS.md 八H1).
    sdo_write_u32(&mut node, Address::new(0x1800, 1), 0x185);
    sdo_write_u8(&mut node, Address::new(0x1800, 2), 252); // RtrOnlySync
    sdo_write_u8(&mut node, Address::new(0x1A00, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1A00, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    start_node(&mut node);

    // No SYNC yet: the request must stay unanswered.
    assert!(node.on_remote_request(0x185).is_none());
    // After the first SYNC the payload is valid again.
    assert!(node.on_frame(0x080, &[]).is_empty());
    let frame = node.on_remote_request(0x185).expect("answered after SYNC");
    assert_eq!(frame.payload(), &[0xAB]);
}

/// BUGS.md M1: a TPDO whose mapping cannot be packed at transmit time
/// (callback-backed mapped object failing its read) surfaces as a config
/// error instead of silently dropping the frame. The mapping itself is
/// *valid* at configuration time — this exercises the pack-time error
/// path, not `validate_against`.
#[test]
fn broken_tpdo_mapping_records_config_error() {
    use canopen_core::object_dictionary::ObjectAccess;
    use std::sync::Arc;

    struct Failing;
    impl ObjectAccess for Failing {
        fn read(&self, _addr: Address) -> canopen_core::types::Result<Value> {
            Err(Error::Other("backend read failed"))
        }
        fn write(&self, _addr: Address, _v: Value) -> canopen_core::types::Result<()> {
            Err(Error::Other("backend write failed"))
        }
    }

    let mut node = test_node();
    // A mappable object backed by a callback whose read fails: passes
    // `validate_against`, but `pdo::pack` propagates the read error.
    node.od_mut().insert(
        Address::new(0x6000, 3),
        Entry::new(
            "failing",
            DataType::Unsigned8,
            AccessType::Rw,
            Storage::Callback(Arc::new(Failing)),
        )
        .pdo_mappable(),
    );
    node.boot().unwrap();
    sdo_write_u32(&mut node, Address::new(0x1800, 1), 0x185 | 0x4000_0000);
    sdo_write_u8(&mut node, Address::new(0x1800, 2), 254); // EventDriven
    sdo_write_u8(&mut node, Address::new(0x1A00, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1A00, 1),
        MappingEntry {
            address: Address::new(0x6000, 3), // callback-backed, read fails
            bit_length: 8,
        }
        .to_u32(),
    );
    start_node(&mut node);

    assert!(node.request_tpdo(1));
    // The broken pack must not emit a frame…
    assert!(node.poll(1000).is_empty());
    // …and must be reported, not swallowed (M1).
    let errs: Vec<Error> = node.take_config_errors().collect();
    assert!(
        errs.iter()
            .any(|e| matches!(e, Error::Other("backend read failed"))),
        "expected the propagated read error, got {errs:?}"
    );
}

/// BUGS.md M3: an installed `EmcyConsumer` receives foreign EMCY frames
/// (0x80+n, n≠self) via `on_frame`; without one the frame falls through
/// (no panic, nothing consumed).
#[test]
fn emcy_consumer_receives_foreign_emergency_frames() {
    use canopen_core::emcy::EmcyConsumer;

    let mut node = test_node();
    node.boot().unwrap();
    start_node(&mut node);
    let consumer = EmcyConsumer::new(16);
    node.set_emcy_consumer(consumer);

    let frame = canopen_core::emcy::EmcyFrame {
        code: 0x1000,
        register: 0x10,
        data: [1, 2, 3, 4, 5],
    };
    // Node 9's EMCY COB-ID is 0x89 — foreign to self (0x85).
    let _ = node.on_frame(0x89, &frame.encode());
    let recent: Vec<_> = node.emcy_consumer().unwrap().recent().collect();
    assert_eq!(recent.len(), 1, "peer EMCY must reach the consumer");
    assert_eq!(recent[0].0, NodeId::new(9).unwrap());
    assert_eq!(recent[0].1.code, 0x1000);
}

/// §8.2 / BUGS.md P2-6: with 0x1015 configured, EMCY frames are deferred to
/// `poll` and spaced at least the inhibit window apart.
#[test]
fn emcy_inhibit_defers_frames_to_poll() {
    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_u16(&mut node, Address::new(0x1015, 0), 100); // 100 * 100 µs = 10 ms

    // The FIRST EMCY has no predecessor: it releases on the next poll.
    let raised = node.raise_emergency(0x1111, 0, [0; 5]);
    assert!(raised.is_none(), "raise itself stays silent under inhibit");
    let frames = node.poll(5);
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].cob_id, 0x85);
    assert_eq!(&frames[0].payload()[..2], &0x1111u16.to_le_bytes());

    // A second event waits a full window since the previous emission.
    assert!(node.raise_emergency(0x2222, 0, [0; 5]).is_none());
    assert!(node.poll(14).is_empty()); // only 9 ms since the first EMCY
    let frames = node.poll(15);
    assert_eq!(frames.len(), 1);
    assert_eq!(&frames[0].payload()[..2], &0x2222u16.to_le_bytes());
}

/// §7.4/§7.5 / BUGS.md P2-7: an armed RPDO slot whose event timer expires
/// without traffic reports exactly once per silent window via
/// `take_rpdo_timeouts`, re-arming after traffic resumes.
#[test]
fn rpdo_event_timer_reports_timeout_once_per_window() {
    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_u32(&mut node, Address::new(0x1400, 1), 0x205);
    sdo_write_u8(&mut node, Address::new(0x1400, 2), 254);
    sdo_write_u16(&mut node, Address::new(0x1400, 5), 10); // dead line 10 ms
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
    start_node(&mut node);

    // First reception arms the deadman.
    let _ = node.on_frame(0x205, &[0x34, 0x12]);
    node.poll(1);
    assert!(
        node.take_rpdo_timeouts().is_empty(),
        "armed, window running"
    );

    node.poll(50);
    assert_eq!(node.take_rpdo_timeouts(), vec![Address::new(0x1400, 0)]);
    node.poll(60);
    assert!(
        node.take_rpdo_timeouts().is_empty(),
        "reported once per window"
    );

    // Traffic resumes: clears the flag, then a fresh silence reports again.
    let _ = node.on_frame(0x205, &[0x00, 0x00]);
    node.poll(70);
    assert!(node.take_rpdo_timeouts().is_empty());
    node.poll(200);
    assert_eq!(node.take_rpdo_timeouts(), vec![Address::new(0x1400, 0)]);
}

/// BUGS.md P2 (third-round review): the deadman must compare elapsed time
/// against `event_timer_ms`. Polling *inside* an unexpired window — as the
/// host's ~5 ms serve loop does constantly — must never report a timeout,
/// no matter how many polls pass since arming.
#[test]
fn rpdo_event_timer_does_not_report_inside_window() {
    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_u32(&mut node, Address::new(0x1400, 1), 0x205);
    sdo_write_u8(&mut node, Address::new(0x1400, 2), 254);
    sdo_write_u16(&mut node, Address::new(0x1400, 5), 100); // dead line 100 ms
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
    start_node(&mut node);

    // First reception arms the deadman around t=0; then poll several times
    // well inside the 100 ms window.
    let _ = node.on_frame(0x205, &[0x34, 0x12]);
    node.poll(1);
    node.poll(5);
    node.poll(10);
    node.poll(20);
    assert!(
        node.take_rpdo_timeouts().is_empty(),
        "window has not expired yet"
    );

    // A timeout is due only after a full window of silence.
    node.poll(150);
    assert_eq!(node.take_rpdo_timeouts(), vec![Address::new(0x1400, 0)]);
}

/// BUGS.md B6: an under-long RPDO is rejected and must not count as traffic —
/// it may not push the event-timer deadline forward (CANopenNode only sets
/// `CANrxNew` on an accepted frame, `CO_PDO.c:442-475`).
#[test]
fn short_rpdo_does_not_rearm_event_deadline() {
    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_u32(&mut node, Address::new(0x1400, 1), 0x205);
    sdo_write_u8(&mut node, Address::new(0x1400, 2), 254);
    sdo_write_u16(&mut node, Address::new(0x1400, 5), 100); // dead line 100 ms
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
    start_node(&mut node);

    // A valid frame at t≈0 arms the deadman and is applied.
    let _ = node.on_frame(0x205, &[0x34, 0x12]);
    node.poll(1);

    // At t=90 a short frame (1 byte < declared 2) raises 0x8210 but must not
    // re-arm the window.
    let out = node.on_frame(0x205, &[0xAA]);
    assert!(
        out.iter().any(|f| f.cob_id == 0x80 + u16::from(NODE)),
        "short frame must report 0x8210"
    );
    node.poll(90);
    assert!(node.take_rpdo_timeouts().is_empty(), "window still open");

    // A full window after the *valid* frame: the timeout fires on schedule.
    node.poll(101);
    assert_eq!(node.take_rpdo_timeouts(), vec![Address::new(0x1400, 0)]);
    assert_eq!(od_value(&node, Address::new(0x6000, 2)), 0x1234);
}

/// BUGS.md P3 (third-round review): heartbeat interception must only claim
/// COB-IDs of *supervised* nodes. A custom RPDO configured anywhere else in
/// `0x700..=0x77F` (legal via bit31/30 semantics or `$NODEID+offset`) must
/// still reach the RPDO dispatch and update the dictionary.
#[test]
fn rpdo_cob_id_inside_heartbeat_range_is_not_swallowed() {
    let mut node = test_node();
    node.boot().unwrap();
    // 0x70A = node 10's HB slot; node 5 is *us* (own-HB branch) so pick a
    // foreign, unsupervised id.
    sdo_write_u32(&mut node, Address::new(0x1400, 1), 0x70A);
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
    start_node(&mut node);

    // Node 10 is NOT under heartbeat supervision (no 0x1016 entry): the
    // frame must flow through to the RPDO slot, not vanish into the HB
    // consumer.
    let _ = node.on_frame(0x70A, &[0x34, 0x12]);
    assert_eq!(
        od_value(&node, Address::new(0x6000, 2)),
        0x1234,
        "custom RPDO inside 0x700..=0x77F must still unpack"
    );
    assert!(node.take_rpdo_timeouts().is_empty());
}

/// §7.2 / BUGS.md P2-5: a configured SYNC start value only phase-locks when
/// 0x1019 (counter overflow) is non-zero; otherwise cyclic TPDOs run from
/// the first SYNC even without counters.
#[test]
fn sync_start_value_requires_counter_overflow() {
    let mut node = test_node();
    node.boot().unwrap();
    // TPDO1: cob 0x185, SYNC every 1st, start value 3.
    sdo_write_u32(&mut node, Address::new(0x1800, 1), 0x185);
    sdo_write_u8(&mut node, Address::new(0x1800, 2), 1);
    node.od_mut().insert(
        Address::new(0x1800, 6),
        Entry::new(
            "sync start",
            DataType::Unsigned8,
            AccessType::Rw,
            Storage::Value(Value::Uint(3)),
        ),
    );
    sdo_write_u8(&mut node, Address::new(0x1A00, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1A00, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    sdo_write_raw(&mut node, Address::new(0x6000, 1), &[0x42]);
    start_node(&mut node);
    // 0x1019 stays 0: no phase-locking — the very first SYNC produces data.
    let tpdos: Vec<TxFrame> = node.on_frame(0x080, &[]).into_iter().collect();
    assert_eq!(tpdos.len(), 1, "start value ignored while counters are off");

    // With 0x1019 set, phase-locking applies again. A counter byte is now
    // mandatory (BUGS.md 八M11): send a non-matching counter while waiting.
    sdo_write_u8(&mut node, Address::new(0x1019, 0), 240);
    let none: Vec<TxFrame> = node.on_frame(0x080, &[0]).into_iter().collect();
    assert!(none.is_empty(), "waiting for counter == start value");
    let hit: Vec<TxFrame> = node.on_frame(0x080, &[3]).into_iter().collect();
    assert_eq!(hit.len(), 1);
}

/// BUGS.md #27: the first SyncCyclic frame goes out at the middle of the
/// first period (`n/2 + 1` SYNCs observed), exactly like CANopenNode
/// `CO_PDO.c:1315-1321` — not after a full period.
#[test]
fn sync_cyclic_first_frame_at_period_midpoint() {
    let mut node = test_node();
    node.boot().unwrap();
    // TPDO1: cob 0x185, SYNC every 4th (n=4 → first frame on the 3rd SYNC),
    // mapping 0x6000:01 (8 bits).
    sdo_write_u32(&mut node, Address::new(0x1800, 1), 0x185);
    sdo_write_u8(&mut node, Address::new(0x1800, 2), 4);
    sdo_write_u8(&mut node, Address::new(0x1A00, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1A00, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    sdo_write_raw(&mut node, Address::new(0x6000, 1), &[0x42]);
    start_node(&mut node);

    // SYNC #1 and #2: silent (before the midpoint of the first period).
    assert!(node.on_frame(0x080, &[]).is_empty());
    assert!(node.on_frame(0x080, &[]).is_empty());
    // SYNC #3 = n/2 + 1: the first frame appears.
    let first: Vec<TxFrame> = node.on_frame(0x080, &[]).into_iter().collect();
    assert_eq!(first.len(), 1, "first frame must land on SYNC n/2+1");
    // The steady-state period is still n SYNCs (next frame on #7).
    assert!(node.on_frame(0x080, &[]).is_empty());
    assert!(node.on_frame(0x080, &[]).is_empty());
    assert!(node.on_frame(0x080, &[]).is_empty());
    let next: Vec<TxFrame> = node.on_frame(0x080, &[]).into_iter().collect();
    assert_eq!(next.len(), 1, "steady state period is n SYNCs");
}

/// §7.2 / BUGS.md P1-12: `set_node_id` rewrites identity-dependent default
/// COB-IDs (EMCY + predefined PDO entries) while custom values survive.
#[test]
fn set_node_id_rewrites_predefined_cobs_only() {
    let mut node = test_node(); // node id 5, PDO records hold predefined|invalid
    node.boot().unwrap();

    // A custom COB-ID must not be touched by renumbering.
    sdo_write_u32(&mut node, Address::new(0x1400, 1), 0x321 | 0x8000_0000);

    node.set_node_id(NodeId::new(9).unwrap());
    assert_eq!(node.node_id().get(), 9);

    // EMCY default followed the new id.
    assert_eq!(
        od_value(&node, Address::new(0x1014, 0)),
        u64::from(NodeId::new(9).unwrap().emcy_cob_id())
    );
    // Predefined PDO defaults followed…
    let tpdo1_new =
        canopen_core::pdo::default_cob_id(true, 1, NodeId::new(9).unwrap()).unwrap() as u64;
    assert_eq!(od_value(&node, Address::new(0x1800, 1)) & 0x7FF, tpdo1_new);
    // …while the custom entry kept its COB-ID (invalid bit still set).
    assert_eq!(
        od_value(&node, Address::new(0x1400, 1)),
        0x321 | 0x8000_0000
    );
}

/// §6.5 / §10.3 / BUGS.md P1-11: writing OD 0x1016 configures the heartbeat
/// consumer immediately; supervised heartbeats produce Started/Lost events
/// through `poll` + `take_hb_events`.
#[test]
fn od_heartbeat_consumer_configures_and_times_out() {
    use canopen_core::nmt::HbEvent;

    let mut node = test_node();
    node.boot().unwrap();
    // Supervise node 7 with a 30 ms timeout: (7 << 16) | 30.
    sdo_write_u32(&mut node, Address::new(0x1016, 1), (7 << 16) | 30);

    // First heartbeat from node 7 → Started.
    let _ = node.on_frame(0x707, &[0x05]);
    node.poll(10);
    let events = node.take_hb_events();
    assert!(events
        .iter()
        .any(|e| matches!(e, HbEvent::Started { node } if node.get() == 7)));

    // Silence past the timeout → Lost.
    node.poll(500);
    let events = node.take_hb_events();
    assert!(events
        .iter()
        .any(|e| matches!(e, HbEvent::Lost { node } if node.get() == 7)));

    // Clearing the entry removes the consumer (no further events).
    sdo_write_u32(&mut node, Address::new(0x1016, 1), 0);
    let _ = node.on_frame(0x707, &[0x05]);
    node.poll(510);
    let events = node.take_hb_events();
    assert!(!events.iter().any(|e| matches!(e, HbEvent::Started { .. })));
}

// --- BUGS.md P2-1 / P2-3 regressions -----------------------------------------

/// BUGS.md P2-1: the SYNC start-value phase lock is one-shot. After the
/// counter first matches the start value, cyclic transmission must continue
/// on every Nth SYNC *without* requiring counter == start again (previously
/// the lock was re-armed after every send, silencing the TPDO until the
/// global counter wrapped back around).
#[test]
fn sync_start_value_locks_once_and_stays_locked() {
    let mut node = test_node();
    node.boot().unwrap();
    // TPDO1: cob 0x185, transmit on every 2nd SYNC, start value 3.
    sdo_write_u32(&mut node, Address::new(0x1800, 1), 0x185);
    sdo_write_u8(&mut node, Address::new(0x1800, 2), 2);
    node.od_mut().insert(
        Address::new(0x1800, 6),
        Entry::new(
            "sync start",
            DataType::Unsigned8,
            AccessType::Rw,
            Storage::Value(Value::Uint(3)),
        ),
    );
    sdo_write_u8(&mut node, Address::new(0x1A00, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1A00, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    sdo_write_raw(&mut node, Address::new(0x6000, 1), &[0x42]);
    sdo_write_u8(&mut node, Address::new(0x1019, 0), 240);
    start_node(&mut node);

    // Before the start value hits: silence.
    assert!(node.on_frame(0x080, &[1]).is_empty());
    assert!(node.on_frame(0x080, &[2]).is_empty());
    // counter == start → phase locks AND the TPDO goes out immediately
    // (CANopenNode `CO_PDO.c:1325-1329`, BUGS.md 八L3).
    let hit = node.on_frame(0x080, &[3]);
    assert_eq!(hit.len(), 1, "matching SYNC sends immediately");

    // Divider counts from the immediate send: tt=2 fires on the 2nd SYNC
    // after the hit — no re-locking needed (counter != start is fine).
    assert!(node.on_frame(0x080, &[99]).is_empty()); // cycle 1
    let second = node.on_frame(0x080, &[100]); // cycle 2 → fires
    assert_eq!(second.len(), 1);
    assert!(node.on_frame(0x080, &[7]).is_empty());
}

/// BUGS.md P2-3: transmission type 0 (synchronous acyclic) TPDOs accept an
/// application request via `request_tpdo` and transmit it on the next SYNC.
#[test]
fn sync_acyclic_tpdo_fires_on_request_plus_sync() {
    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_u32(&mut node, Address::new(0x1800, 1), 0x185);
    sdo_write_u8(&mut node, Address::new(0x1800, 2), 0); // SyncAcyclic
    sdo_write_u8(&mut node, Address::new(0x1A00, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1A00, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    sdo_write_raw(&mut node, Address::new(0x6000, 1), &[0x42]);
    start_node(&mut node);

    // SYNC without a pending request: silent.
    assert!(node.on_frame(0x080, &[]).is_empty());
    // Request latches…
    assert!(node.request_tpdo(1), "tt=0 accepts application requests");
    // …and the next SYNC transmits exactly once.
    let tx = node.on_frame(0x080, &[]);
    assert_eq!(tx.len(), 1);
    // The request is consumed: further SYNCs stay silent until re-armed.
    assert!(node.on_frame(0x080, &[]).is_empty());

    // tt=240 (cyclic every SYNC) still rejects application requests.
    sdo_write_u8(&mut node, Address::new(0x1800, 2), 240);
    rebuild_pdos_via_sdo_touch(&mut node);
    assert!(!node.request_tpdo(1));
}

/// Re-runs PDO assembly by touching a comm parameter through SDO semantics.
fn rebuild_pdos_via_sdo_touch(node: &mut Node) {
    // 0x1800:5 (event timer) is UNSIGNED16.
    sdo_write_raw(node, Address::new(0x1800, 5), &[0x00, 0x00]);
}

// --- BUGS.md 批次 B 回归（P2-4 / P3-11 / P3-13 / P3-17 / P3-22）---------------

/// BUGS.md P2-4: a synchronous RPDO buffers its payload until the next SYNC;
/// OD reads before the boundary still see the previous cycle's values.
#[test]
fn sync_rpdo_applies_data_only_after_sync() {
    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_u32(&mut node, Address::new(0x1400, 1), 0x205);
    sdo_write_u8(&mut node, Address::new(0x1400, 2), 1); // synchronous cyclic
    sdo_write_u8(&mut node, Address::new(0x1600, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1600, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    sdo_write_raw(&mut node, Address::new(0x6000, 1), &[0]);
    start_node(&mut node);

    // Frame arrives: NOT yet visible in the OD.
    let _ = node.on_frame(0x205, &[0xAB]);
    assert_eq!(od_value(&node, Address::new(0x6000, 1)), 0, "buffered");

    // The next SYNC applies it.
    let _ = node.on_frame(0x080, &[]);
    assert_eq!(od_value(&node, Address::new(0x6000, 1)), 0xAB);
}

/// BUGS.md C3: leaving Operational discards a buffered synchronous RPDO —
/// the stale sample must not be applied at the first SYNC after a restart
/// (CANopenNode `CO_PDO.c:846-862` clears both directions).
#[test]
fn leaving_operational_discards_buffered_sync_rpdo() {
    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_u32(&mut node, Address::new(0x1400, 1), 0x205);
    sdo_write_u8(&mut node, Address::new(0x1400, 2), 1); // synchronous
    sdo_write_u8(&mut node, Address::new(0x1600, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1600, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    sdo_write_raw(&mut node, Address::new(0x6000, 1), &[0]);
    start_node(&mut node);

    // Buffer a frame, then stop before the SYNC that would apply it.
    let _ = node.on_frame(0x205, &[0xAB]);
    let _ = node.on_frame(0x000, &[0x02, 0x00]); // Stop
    let _ = node.on_frame(0x000, &[0x01, 0x00]); // Start again
    let _ = node.on_frame(0x080, &[]); // SYNC must not apply the stale sample
    assert_eq!(od_value(&node, Address::new(0x6000, 1)), 0);
}

/// BUGS.md R10-P2-12: a minimal RPDO record (only sub 1 COB-ID, no sub 2
/// transmission type) defaults to *synchronous* per CiA 301 — the payload is
/// buffered until the next SYNC, not applied immediately.
#[test]
fn minimal_rpdo_record_defaults_to_synchronous() {
    // Build a node whose RPDO1 comm record has NO sub 2 at all (unlike
    // `test_node`, whose `install_pdo_objects` pre-seeds sub 2 = 254).
    let mut od = StandardObjects::new().device_type(0x1234).build();
    install_standard_objects(&mut od, NodeId::new(NODE).unwrap());
    let put = |od: &mut ObjectDictionary, addr: Address, v: u64, dt: DataType| {
        od.insert(
            addr,
            Entry::new("", dt, AccessType::Rw, Storage::Value(Value::Uint(v))),
        );
    };
    put(
        &mut od,
        Address::new(0x1400, 1),
        0x205,
        DataType::Unsigned32,
    );
    put(&mut od, Address::new(0x1400, 3), 0, DataType::Unsigned16);
    put(&mut od, Address::new(0x1400, 5), 0, DataType::Unsigned16);
    // NOTE: no Address::new(0x1400, 2) — the transmission type is absent.
    put(&mut od, Address::new(0x1600, 0), 1, DataType::Unsigned8);
    put(
        &mut od,
        Address::new(0x1600, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32() as u64,
        DataType::Unsigned32,
    );
    od.insert(
        Address::new(0x6000, 1),
        Entry::new(
            "app",
            DataType::Unsigned8,
            AccessType::Rw,
            Storage::Value(Value::Uint(0)),
        )
        .pdo_mappable(),
    );
    let mut node = Node::new(NodeId::new(NODE).unwrap(), od).unwrap();
    node.boot().unwrap();
    start_node(&mut node);

    // Frame arrives: NOT yet visible in the OD (buffered for SYNC).
    let _ = node.on_frame(0x205, &[0xCD]);
    assert_eq!(od_value(&node, Address::new(0x6000, 1)), 0, "buffered");

    // The next SYNC applies it — the default is synchronous, not 254.
    let _ = node.on_frame(0x080, &[]);
    assert_eq!(od_value(&node, Address::new(0x6000, 1)), 0xCD);
}

/// BUGS.md P3-17: malformed SYNC payloads (>1 byte) must not drive the
/// synchronous machinery.
#[test]
fn malformed_sync_payload_is_rejected() {
    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_u32(&mut node, Address::new(0x1800, 1), 0x185);
    sdo_write_u8(&mut node, Address::new(0x1800, 2), 1);
    sdo_write_u8(&mut node, Address::new(0x1A00, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1A00, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    sdo_write_raw(&mut node, Address::new(0x6000, 1), &[0x42]);
    start_node(&mut node);

    // Valid empty SYNC fires; malformed 2-byte payload does not drive any
    // TPDO (it does raise the SYNC-length emergency 0x8240, BUGS.md 八M11).
    let fired: Vec<TxFrame> = node.on_frame(0x080, &[]);
    assert!(fired.iter().any(|f| f.cob_id == 0x185));
    let malformed: Vec<TxFrame> = node.on_frame(0x080, &[7, 7]);
    assert!(
        malformed.iter().all(|f| f.cob_id != 0x185),
        "malformed SYNC must not drive TPDOs: {malformed:?}"
    );
    // BUGS.md 八M11: the width mismatch is reported as
    // CO_EMC_SYNC_DATA_LENGTH (0x8240) via the emergency channel.
    assert!(
        malformed.iter().any(|f| f.cob_id == 0x80 + u16::from(NODE)),
        "0x8240 emergency expected: {malformed:?}"
    );
    // With 0x1019 == 0, even a one-byte SYNC is malformed (the 0x8240
    // condition is already active, so no second frame — idempotent raise).
    let one_byte: Vec<TxFrame> = node.on_frame(0x080, &[0x01]);
    assert!(
        one_byte.iter().all(|f| f.cob_id != 0x185),
        "1-byte SYNC must not drive TPDOs while 0x1019 == 0: {one_byte:?}"
    );
}

/// BUGS.md 八M12: 0x1005 bit30 is the *gen* (SYNC producer) flag — a node
/// declaring itself producer must still consume SYNC frames; the reserved
/// bit29 must not match as a CAN-ID.
#[test]
fn sync_cob_id_gen_bit_does_not_blind_consumer() {
    use canopen_core::datatypes::DataType;
    use canopen_core::object_dictionary::{AccessType, Entry, Storage};

    let mut node = test_node();
    node.boot().unwrap();
    // SYNC TPDO1 (transmission type 1) with one mappable entry.
    sdo_write_u32(&mut node, Address::new(0x1800, 1), 0x185);
    sdo_write_u8(&mut node, Address::new(0x1800, 2), 1);
    sdo_write_u8(&mut node, Address::new(0x1A00, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1A00, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    sdo_write_raw(&mut node, Address::new(0x6000, 1), &[0x42]);
    start_node(&mut node);

    // We are the SYNC producer (bit30 set) at COB-ID 0x80: still a consumer.
    node.od_mut().insert(
        Address::new(0x1005, 0),
        Entry::new(
            "COB-ID SYNC",
            DataType::Unsigned32,
            AccessType::Rw,
            Storage::Value(Value::Uint(0x4000_0080)),
        ),
    );
    let frames: Vec<TxFrame> = node.on_frame(0x080, &[]);
    assert!(
        frames.iter().any(|f| f.cob_id == 0x185),
        "gen bit must not blind the SYNC consumer: {frames:?}"
    );

    // Reserved bit29 set: not a valid SYNC COB-ID, must not match.
    node.od_mut()
        .set_value(Address::new(0x1005, 0), Value::Uint(0x2000_0080))
        .unwrap();
    let frames: Vec<TxFrame> = node.on_frame(0x080, &[]);
    assert!(
        frames.iter().all(|f| f.cob_id != 0x185),
        "bit29-reserved value must not match: {frames:?}"
    );
}

/// BUGS.md P3-11: writing a mapped OD entry (via SDO) latches the
/// event-driven TPDO like an application request would (flagsPDO).
#[test]
fn od_write_triggers_event_driven_tpdo() {
    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_u32(&mut node, Address::new(0x1800, 1), 0x185);
    sdo_write_u8(&mut node, Address::new(0x1800, 2), 254); // event-driven
    sdo_write_raw(&mut node, Address::new(0x1800, 5), &[0x00, 0x00]); // no event timer (U16)
    sdo_write_u8(&mut node, Address::new(0x1A00, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1A00, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    start_node(&mut node);

    // No poll output without a write.
    assert!(node.poll(1000).is_empty());
    // SDO write to the mapped object latches the event…
    sdo_write_raw(&mut node, Address::new(0x6000, 1), &[0x77]);
    let tx = node.poll(1100);
    assert_eq!(tx.len(), 1, "OD write fired the event-driven TPDO");
    assert_eq!(tx[0].payload()[0], 0x77);
}

/// BUGS.md P3-13: a boot-up from a node previously flagged Lost reports
/// RemoteReset, not just Started.
#[test]
fn remote_reset_reported_after_timeout() {
    use canopen_core::nmt::{HbEvent, HeartbeatConsumer};
    let mut hb = HeartbeatConsumer::new();
    let n9 = NodeId::new(9).unwrap();
    hb.set_timeout(n9, 100);
    assert!(!hb.on_heartbeat(n9, &[0x05], 0).is_empty()); // Started
    let lost = hb.poll(300);
    assert!(matches!(&lost[..], [HbEvent::Lost { .. }]));
    // Node restarts: boot-up after TimedOut → RemoteReset (previously the
    // event was only produced from the Active phase).
    let events = hb.on_heartbeat(n9, &[0x00], 400);
    assert!(
        matches!(&events[..], [HbEvent::RemoteReset { .. }]),
        "expected RemoteReset, got {events:?}"
    );
}

/// BUGS.md B2: a configured node that has never been seen (phase Unknown)
/// still reports `RemoteReset` on boot-up — CANopenNode fires
/// `pFunctSignalRemoteReset` for every tracked entry, UNKNOWN included
/// (`CO_HBconsumer.c:320-334`).
#[test]
fn remote_reset_reported_from_unknown_phase() {
    use canopen_core::nmt::{HbEvent, HeartbeatConsumer};
    let mut hb = HeartbeatConsumer::new();
    let n9 = NodeId::new(9).unwrap();
    hb.set_timeout(n9, 100);
    let events = hb.on_heartbeat(n9, &[0x00], 0);
    assert!(
        matches!(&events[..], [HbEvent::RemoteReset { .. }]),
        "expected RemoteReset for a never-seen node, got {events:?}"
    );
    // A regular heartbeat then starts the supervision normally.
    let events = hb.on_heartbeat(n9, &[0x7F], 10);
    assert!(
        events.iter().any(|e| matches!(e, HbEvent::Started { .. })),
        "expected Started, got {events:?}"
    );
}

/// BUGS.md P3-14: tolerance widens the supervision deadline.
#[test]
fn hb_tolerance_widens_deadline() {
    use canopen_core::nmt::{HbEvent, HeartbeatConsumer};
    let mut strict = HeartbeatConsumer::new();
    let mut loose = HeartbeatConsumer::new();
    loose.set_tolerance_pct(100); // deadline ×2
    for hb in [&mut strict, &mut loose] {
        hb.set_timeout(NodeId::new(5).unwrap(), 100);
        assert!(!hb
            .on_heartbeat(NodeId::new(5).unwrap(), &[0x05], 0)
            .is_empty());
    }
    assert!(matches!(&strict.poll(150)[..], [HbEvent::Lost { .. }]));
    assert!(loose.poll(150).is_empty(), "tolerance keeps it alive");
    assert!(matches!(&loose.poll(250)[..], [HbEvent::Lost { .. }]));
}

/// BUGS.md P3-22: a frame on a COB-ID this node transmits as TPDO is treated
/// as self-echo and never applied to an RPDO slot.
#[test]
fn self_echo_tpdo_cobid_not_applied_as_rpdo() {
    let mut node = test_node();
    node.boot().unwrap();
    // TPDO1 valid on 0x185; an RPDO slot configured on the SAME COB-ID —
    // frames there are our own echoes and must never reach the RPDO.
    sdo_write_u8(&mut node, Address::new(0x1800, 2), 254);
    sdo_write_u8(&mut node, Address::new(0x1A00, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1A00, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    sdo_write_raw(&mut node, Address::new(0x6000, 1), &[0]);
    sdo_write_u32(&mut node, Address::new(0x1800, 1), 0x185);
    sdo_write_u8(&mut node, Address::new(0x1A00, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1A00, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    sdo_write_u32(&mut node, Address::new(0x1400, 1), 0x185);
    sdo_write_u8(&mut node, Address::new(0x1600, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1600, 1),
        MappingEntry {
            address: Address::new(0x6000, 2),
            bit_length: 8,
        }
        .to_u32(),
    );
    start_node(&mut node);

    // A frame on the shared COB-ID is our own echo: RPDO target untouched.
    let _ = node.on_frame(0x185, &[0xDE]);
    assert_eq!(
        od_value(&node, Address::new(0x6000, 2)),
        0,
        "echo suppressed"
    );
}

/// BUGS.md P3-1 (see git history): an NMT reset must clear the SDO session state
/// (`sdo.reset()` drops the in-flight transfer and the inactivity window).
/// Otherwise the first post-reset transfer inherits the pre-reset timestamp
/// and can be torn down by `sdo_timeout_due` while perfectly alive.
#[test]
fn nmt_reset_clears_sdo_session_baseline() {
    let mut node = test_node();
    node.boot().unwrap();
    let _ = node.on_frame(0x000, &[0x01, 0x00]); // start
    node.set_sdo_timeout(100);

    // Open a segmented download and leave it in flight.
    let mut client = SdoClient::new();
    let init = client
        .write(Address::new(0x6000, 1), b"xxxxxxxxxxxxxxxxxxxx")
        .unwrap();
    for f in &init {
        let _ = node.on_frame(0x600 + u16::from(NODE), f);
    }
    node.touch_sdo_activity(1000);
    assert!(node.sdo_is_busy());

    // Reset communication mid-transfer: the reset re-boots automatically
    // (BUGS.md H2), handing back the boot-up frame.
    let reset_out = node.on_frame(0x000, &[0x82, 0x00]);
    assert_eq!(reset_out.len(), 1);
    assert_eq!(reset_out[0].payload(), &[0x00]);

    // A fresh transfer after the reset: its baseline must be its own.
    let _ = node.on_frame(0x000, &[0x01, 0x00]);
    let mut client2 = SdoClient::new();
    let init2 = client2
        .write(Address::new(0x6000, 1), b"yyyyyyyyyyyyyyyyyyyy")
        .unwrap();
    for f in &init2 {
        let _ = node.on_frame(0x600 + u16::from(NODE), f);
    }
    node.touch_sdo_activity(2000);

    // 50 ms into the new session: no timeout despite the stale T=1000 stamp.
    assert!(
        node.sdo_timeout_due(2050).is_none(),
        "fresh session must not inherit"
    );
    assert!(node.sdo_is_busy());
}

/// BUGS.md C1: a session timeout tears the transfer down and returns the
/// `0x05040000` abort naming the in-flight object for transmission
/// (CANopenNode `CO_SDOserver.c:1076-1079,1496-1509`).
#[test]
fn sdo_session_timeout_emits_timeout_abort() {
    let mut node = test_node();
    node.boot().unwrap();
    node.set_sdo_timeout(100);

    let mut client = SdoClient::new();
    let init = client
        .write(Address::new(0x6000, 1), b"xxxxxxxxxxxxxxxxxxxx")
        .unwrap();
    for f in &init {
        let _ = node.on_frame(0x600 + u16::from(NODE), f);
    }
    node.touch_sdo_activity(1000);
    assert!(node.sdo_timeout_due(1050).is_none());

    let abort = node.sdo_timeout_due(1200).expect("timeout must fire");
    assert_eq!(abort.cob_id, 0x580 + u16::from(NODE));
    let p = abort.payload();
    assert_eq!(p[0], 0x80);
    assert_eq!(u16::from_le_bytes([p[1], p[2]]), 0x6000);
    assert_eq!(p[3], 1);
    assert_eq!(u32::from_le_bytes([p[4], p[5], p[6], p[7]]), 0x0504_0000);
    assert!(!node.sdo_is_busy());
}

/// BUGS.md 八L14: writing 0x1019 must reject 1 (reserved) and >240
/// (CANopenNode `OD_write_1019`), leaving the previous value intact.
#[test]
fn sync_counter_overflow_write_is_validated() {
    use canopen_core::sdo::SdoAbortCode;

    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_u8(&mut node, Address::new(0x1019, 0), 10);

    let mut client = SdoClient::new();
    let first = client.write(Address::new(0x1019, 0), &[1]).unwrap();
    assert_eq!(
        transact_expect_abort(&mut node, &mut client, first),
        SdoAbortCode::InvalidValue
    );
    let mut client = SdoClient::new();
    let first = client.write(Address::new(0x1019, 0), &[241]).unwrap();
    assert_eq!(
        transact_expect_abort(&mut node, &mut client, first),
        SdoAbortCode::InvalidValue
    );
    assert_eq!(od_value(&node, Address::new(0x1019, 0)), 10);
}

// --- 第十轮审计回归（2026-09-14） -------------------------------------------

/// 十H5/十L5: over-long RPDO raises EMCY 0x8220 (and still applies the
/// frame), an under-long frame raises 0x8210 and is not applied, and a
/// correctly-sized frame clears the latched condition — matching
/// CANopenNode `CO_PDO.c:420-465,717-727`.
#[test]
fn rpdo_length_emergencies_raise_and_clear() {
    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_u32(&mut node, Address::new(0x1400, 1), 0x205);
    sdo_write_u8(&mut node, Address::new(0x1400, 2), 254); // event-driven
    sdo_write_u8(&mut node, Address::new(0x1600, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1600, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    sdo_write_raw(&mut node, Address::new(0x6000, 1), &[0]);
    start_node(&mut node);
    let emcy = 0x80 + u16::from(NODE);

    // Over-long (2 bytes vs declared 1): 0x8220 is raised and the frame is
    // applied like CANopenNode (`CO_EMC_PDO_LENGTH_EXC`).
    let long = node.on_frame(0x205, &[0x11, 0x22]);
    let frame = long
        .iter()
        .find(|f| f.cob_id == emcy)
        .expect("0x8220 EMCY expected");
    assert_eq!(&frame.payload()[..2], &0x8220u16.to_le_bytes());
    assert_eq!(od_value(&node, Address::new(0x6000, 1)), 0x11);

    // Back to the declared length: the condition is cleared (reset EMCY).
    let clear = node.on_frame(0x205, &[0x33]);
    let frame = clear
        .iter()
        .find(|f| f.cob_id == emcy)
        .expect("0x8220 clear frame expected");
    assert_eq!(&frame.payload()[..2], &[0, 0]);
    assert_eq!(od_value(&node, Address::new(0x6000, 1)), 0x33);

    // Under-long: 0x8210, not applied.
    let short = node.on_frame(0x205, &[]);
    let frame = short
        .iter()
        .find(|f| f.cob_id == emcy)
        .expect("0x8210 EMCY expected");
    assert_eq!(&frame.payload()[..2], &0x8210u16.to_le_bytes());
    assert_eq!(od_value(&node, Address::new(0x6000, 1)), 0x33);

    // …and the next correct frame clears it too.
    let clear = node.on_frame(0x205, &[0x44]);
    assert!(
        clear.iter().any(|f| f.cob_id == emcy),
        "0x8210 clear frame expected: {clear:?}"
    );
    assert_eq!(od_value(&node, Address::new(0x6000, 1)), 0x44);
}

/// 十M1: an NMT state change that happens before the first `poll` must
/// still emit the immediate state heartbeat — the boot-up epoch anchor
/// (`hb_arm_pending`) must not swallow it (CANopenNode
/// `CO_NMT_Heartbeat.c:184-204`).
#[test]
fn state_change_before_first_poll_sends_immediate_heartbeat() {
    let mut node = test_node();
    node.od_mut()
        .write(Address::new(0x1017, 0), Value::Uint(1000))
        .unwrap();
    node.boot().unwrap();
    // Start arrives before the first poll: the heartbeat epoch has not been
    // anchored yet.
    let _ = node.on_frame(0x000, &[0x01, NODE]);
    let beats: Vec<TxFrame> = node
        .poll(1_000_000)
        .into_iter()
        .filter(|f| f.cob_id == 0x705)
        .collect();
    assert_eq!(
        beats.len(),
        1,
        "state change must beat immediately: {beats:?}"
    );
    assert_eq!(beats[0].payload()[0], 0x05);

    // Enabling the producer after entering Operational (still before any
    // beat) must also fire on the next poll, not one period later.
    let mut node = test_node();
    node.boot().unwrap();
    let _ = node.on_frame(0x000, &[0x01, NODE]);
    assert!(!node.poll(1_000).iter().any(|f| f.cob_id == 0x705));
    sdo_write_u16(&mut node, Address::new(0x1017, 0), 100);
    let beats: Vec<TxFrame> = node
        .poll(1_000_001)
        .into_iter()
        .filter(|f| f.cob_id == 0x705)
        .collect();
    assert_eq!(
        beats.len(),
        1,
        "0x1017 write must beat immediately: {beats:?}"
    );
}

/// 十M2: a communication reset re-initialises the heartbeat consumer — the
/// pre-reset `last_seen` stamp must not produce an immediate false `Lost`.
#[test]
fn nmt_reset_reinitialises_heartbeat_consumer() {
    use canopen_core::nmt::HbEvent;

    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_u32(&mut node, Address::new(0x1016, 1), (7 << 16) | 30);
    let _ = node.on_frame(0x707, &[0x05]);
    node.poll(10);
    let _ = node.take_hb_events(); // Started

    // ResetCommunication: consumer back to Unknown, no stale timestamps.
    let _ = node.on_frame(0x000, &[0x82, NODE]);
    node.poll(20);
    let events = node.take_hb_events();
    assert!(
        !events.iter().any(|e| matches!(e, HbEvent::Lost { .. })),
        "reset must not fake Lost: {events:?}"
    );

    // Supervision restarts from the new epoch.
    let _ = node.on_frame(0x707, &[0x05]);
    node.poll(30);
    assert!(node
        .take_hb_events()
        .iter()
        .any(|e| matches!(e, HbEvent::Started { node } if node.get() == 7)));
    node.poll(100);
    assert!(node
        .take_hb_events()
        .iter()
        .any(|e| matches!(e, HbEvent::Lost { node } if node.get() == 7)));
}

/// 十M3: SyncAcyclic (tt=0) must ignore `syncStartValue` — the phase lock
/// belongs to cyclic types only (CANopenNode `CO_PDO.c:1305-1329`); a
/// matching counter must not send unsolicited, and an explicit request is
/// consumed exactly once.
#[test]
fn sync_acyclic_ignores_sync_start_value() {
    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_u32(&mut node, Address::new(0x1800, 1), 0x185);
    sdo_write_u8(&mut node, Address::new(0x1800, 2), 0); // SyncAcyclic
    node.od_mut().insert(
        Address::new(0x1800, 6),
        Entry::new(
            "sync start",
            DataType::Unsigned8,
            AccessType::Rw,
            Storage::Value(Value::Uint(3)),
        ),
    );
    sdo_write_u8(&mut node, Address::new(0x1019, 0), 240);
    sdo_write_u8(&mut node, Address::new(0x1A00, 0), 1);
    sdo_write_u32(
        &mut node,
        Address::new(0x1A00, 1),
        MappingEntry {
            address: Address::new(0x6000, 1),
            bit_length: 8,
        }
        .to_u32(),
    );
    sdo_write_raw(&mut node, Address::new(0x6000, 1), &[0x42]);
    start_node(&mut node);

    // Counter matches the start value but nothing was requested → silent.
    let unsolicited = node.on_frame(0x080, &[3]);
    assert!(
        unsolicited.iter().all(|f| f.cob_id != 0x185),
        "tt=0 must not fire on a matching counter: {unsolicited:?}"
    );

    // An explicit request fires on the next SYNC, once.
    assert!(node.request_tpdo(1));
    let requested: Vec<TxFrame> = node
        .on_frame(0x080, &[3])
        .into_iter()
        .filter(|f| f.cob_id == 0x185)
        .collect();
    assert_eq!(requested.len(), 1);
    let next: Vec<TxFrame> = node
        .on_frame(0x080, &[4])
        .into_iter()
        .filter(|f| f.cob_id == 0x185)
        .collect();
    assert!(next.is_empty(), "request consumed once: {next:?}");
}

/// 十M4: writing OD 0x1005 must reject reserved-bit and restricted CAN-ID
/// values (`ODR_INVALID_VALUE`), otherwise the node silently goes blind to
/// every SYNC.
#[test]
fn sync_cob_id_write_is_validated() {
    let mut node = test_node();
    node.boot().unwrap();

    for raw in [0x2000_0080u32, 0x8000_0080] {
        let mut client = SdoClient::new();
        let first = client
            .write(Address::new(0x1005, 0), &raw.to_le_bytes())
            .unwrap();
        assert_eq!(
            transact_expect_abort(&mut node, &mut client, first),
            SdoAbortCode::InvalidValue,
            "0x{raw:08X} must be rejected"
        );
    }
    // The legal default remains writable.
    sdo_write_u32(&mut node, Address::new(0x1005, 0), 0x80);
    assert_eq!(od_value(&node, Address::new(0x1005, 0)), 0x80);
}

/// 十L4: a SYNC width mismatch is reported (0x8240) in PreOperational too —
/// CANopenNode `CO_SYNC.c:386-392` reports in PreOp and Op.
#[test]
fn sync_width_error_reported_in_preoperational() {
    let mut node = test_node();
    node.boot().unwrap(); // PreOperational
    let out = node.on_frame(0x080, &[7, 7]);
    let frame = out
        .iter()
        .find(|f| f.cob_id == 0x80 + u16::from(NODE))
        .expect("0x8240 EMCY expected in PreOp");
    assert_eq!(&frame.payload()[..2], &0x8240u16.to_le_bytes());
}

/// 十M8: the heartbeat-event output queue is bounded (drop-oldest at 256,
/// the same policy as the input queues) so an application that polls but
/// rarely drains cannot grow it without limit.
#[test]
fn heartbeat_event_queue_is_bounded() {
    let mut node = test_node();
    node.boot().unwrap();
    sdo_write_u32(&mut node, Address::new(0x1016, 1), (7 << 16) | 60_000);
    for i in 0..1000u16 {
        let state = if i % 2 == 0 { 0x05 } else { 0x7F };
        let _ = node.on_frame(0x707, &[state]);
    }
    node.poll(1_000);
    let events = node.take_hb_events();
    assert!(!events.is_empty());
    assert!(
        events.len() <= 256,
        "output queue must be bounded: {}",
        events.len()
    );
    node.poll(2_000_000);
    assert!(node.take_hb_events().len() <= 256);
}
