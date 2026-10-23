//! Port of `ref/canopen/test/test_local.py` — SDO server semantics with a
//! client↔server pair over an **in-process virtual bus** (slave = `Node`,
//! master = `SdoClient`), mirroring python-canopen's two-Network virtual
//! loopback. Deviations from python behaviour are documented inline and in
//! docs/EXTENSIONS.md 批次 D 差异表 D1–D4.
//!
//! NOTE (docs/DESIGN-DECISIONS.md M9): the golden frames below are hand-transcribed from
//! python-canopen's suite and are **not** re-checked by `cargo test` against
//! `ref/`. After touching any codec, run
//! `python3 tools/interop/python_canopen_oracle.py` (8/8) to re-validate the
//! transcription against the reference implementation.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use canopen_core::datatypes::{DataType, Value};
use canopen_core::node::Node;
use canopen_core::object_dictionary::{AccessType, Entry, ObjectDictionary, Storage};
use canopen_core::sdo::{SdoAbortCode, SdoClient, SdoEvent};
use canopen_core::standard::StandardObjects;
use canopen_core::types::{Address, Error, NodeId};

const SLAVE_A: u8 = 2;
const SLAVE_B: u8 = 3;

// --- sample.eds-equivalent object dictionary ---------------------------------

/// Recording accessor used for the callback test (python's
/// add_read_callback/add_write_callback).
#[derive(Default)]
struct RecordingAccess {
    last_write: Mutex<Option<(u16, u8)>>,
}

impl canopen_core::object_dictionary::ObjectAccess for RecordingAccess {
    fn read(&self, _addr: Address) -> Result<Value, Error> {
        // python `_some_read_callback` returns 0x0201 → bytes 01 02 00 00.
        Ok(Value::Uint(0x0201))
    }
    fn write(&self, addr: Address, _value: Value) -> Result<(), Error> {
        *self.last_write.lock().unwrap() = Some((addr.index, addr.subindex));
        Ok(())
    }
}

fn sample_od(node_id: u8) -> ObjectDictionary {
    let mut od = StandardObjects::new()
        .vendor_id(1)
        .product_code(2)
        .serial_number(4)
        .build();
    let put = |od: &mut ObjectDictionary,
               addr: Address,
               name: &str,
               dt: DataType,
               v: Value,
               access: AccessType| {
        od.insert(addr, Entry::new(name, dt, access, Storage::Value(v)));
    };

    put(
        &mut od,
        Address::new(0x1001, 0),
        "Error register",
        DataType::Unsigned8,
        Value::Uint(0),
        AccessType::Ro,
    );
    // 0x1003 as a recording callback entry (python test_callbacks).
    od.insert(
        Address::new(0x1003, 5),
        Entry::new(
            "Pre-defined error field",
            DataType::Unsigned32,
            AccessType::Ro,
            Storage::Callback(std::sync::Arc::new(RecordingAccess::default())),
        ),
    );
    put(
        &mut od,
        Address::new(0x1008, 0),
        "Manufacturer device name",
        DataType::VisibleString,
        Value::Str("TEST DEVICE".into()),
        AccessType::Const,
    );
    put(
        &mut od,
        Address::new(0x1017, 0),
        "Producer heartbeat time",
        DataType::Unsigned16,
        Value::Uint(0),
        AccessType::Rw,
    );
    // Identity record with sub3 deliberately absent (sample.eds quirk).
    put(
        &mut od,
        Address::new(0x1018, 0),
        "Highest sub-index supported",
        DataType::Unsigned8,
        Value::Uint(4),
        AccessType::Ro,
    );
    put(
        &mut od,
        Address::new(0x1018, 1),
        "Vendor-ID",
        DataType::Unsigned32,
        Value::Uint(1),
        AccessType::Ro,
    );
    put(
        &mut od,
        Address::new(0x1018, 2),
        "Product code",
        DataType::Unsigned32,
        Value::Uint(2),
        AccessType::Ro,
    );
    // sub3 left out for testing
    put(
        &mut od,
        Address::new(0x1018, 4),
        "Serial number",
        DataType::Unsigned32,
        Value::Uint(4),
        AccessType::Ro,
    );

    // RPDO1 communication parameter ($NODEID+512).
    put(
        &mut od,
        Address::new(0x1400, 1),
        "COB-ID used by RPDO 1",
        DataType::Unsigned32,
        Value::Uint(512 + u64::from(node_id)),
        AccessType::Rw,
    );

    // Application objects.
    put(
        &mut od,
        Address::new(0x2000, 0),
        "Writable string",
        DataType::VisibleString,
        Value::Str(String::new()),
        AccessType::Rw,
    );
    put(
        &mut od,
        Address::new(0x2001, 0),
        "Signed 16-bit value",
        DataType::Integer16,
        Value::Int(0),
        AccessType::Rw,
    );
    put(
        &mut od,
        Address::new(0x2004, 0),
        "Unsigned 16-bit value",
        DataType::Unsigned16,
        Value::Uint(0),
        AccessType::Rw,
    );
    put(
        &mut od,
        Address::new(0x2010, 0),
        "Sensor Sampling Rate (Hz)",
        DataType::Real32,
        Value::Real(5.2),
        AccessType::Rw,
    );
    put(
        &mut od,
        Address::new(0x2101, 0),
        "Block transfer target",
        DataType::Domain,
        Value::Bytes(Vec::new()),
        AccessType::Rw,
    );
    od
}

// --- virtual bus harness -------------------------------------------------------

struct Bus {
    nodes: Vec<Node>,
}

impl Bus {
    /// Both slaves from the python blueprint (nodes 2 and 3).
    fn new() -> Self {
        Self {
            nodes: [SLAVE_A, SLAVE_B]
                .map(|id| Node::new(NodeId::new(id).unwrap(), sample_od(id)).unwrap())
                .into(),
        }
    }

    fn node(&mut self, id: u8) -> &mut Node {
        self.nodes
            .iter_mut()
            .find(|n| n.node_id().get() == id)
            .expect("node present")
    }

    /// Broadcasts an NMT command to every node.
    fn nmt(&mut self, command_byte: u8, target: u8) {
        let frame = [command_byte, target];
        for node in &mut self.nodes {
            if target == 0 || target == node.node_id().get() {
                let _ = node.on_frame(0x000, &frame);
            }
        }
    }

    /// Sends a raw abort frame to one node.
    fn send_abort(&mut self, node_id: u8, addr: Address, code: u32) {
        let mut f = [0u8; 8];
        f[0] = 0x80;
        f[1..3].copy_from_slice(&addr.index.to_le_bytes());
        f[3] = addr.subindex;
        f[4..8].copy_from_slice(&code.to_le_bytes());
        let node = self.node(node_id);
        let _ = node.on_frame(0x600 + u16::from(node_id), &f);
    }

    /// Drives an SDO transaction to completion; returns the read value when
    /// the transaction produced one.
    fn sdo_transact(
        &mut self,
        node_id: u8,
        client: &mut SdoClient,
        first: Vec<[u8; 8]>,
    ) -> std::result::Result<Option<Value>, SdoAbortCode> {
        let rx_cob = 0x600 + u16::from(node_id);
        let tx_cob = 0x580 + u16::from(node_id);
        let mut queue: VecDeque<[u8; 8]> = first.into();
        let mut completed = None;
        while let Some(frame) = queue.pop_front() {
            let node = self
                .nodes
                .iter_mut()
                .find(|n| n.node_id().get() == node_id)
                .expect("node present");
            for resp in node.on_frame(rx_cob, &frame) {
                assert_eq!(resp.cob_id, tx_cob, "response COB-ID mismatch");
                let event = match client.on_response(&resp.data) {
                    Ok(ev) => ev,
                    Err(Error::SdoAborted(code)) => return Err(code),
                    Err(other) => panic!("unexpected protocol failure: {other}"),
                };
                match event {
                    SdoEvent::Send(frames) => queue.extend(frames),
                    SdoEvent::Complete(value, trailing) => {
                        completed = Some(value);
                        queue.extend(trailing);
                    }
                    SdoEvent::Done => {}
                }
            }
        }
        Ok(completed)
    }

    fn sdo_read(
        &mut self,
        node_id: u8,
        addr: Address,
        dt: DataType,
    ) -> std::result::Result<Value, SdoAbortCode> {
        let mut client = SdoClient::new();
        let first = client.read(addr, dt);
        Ok(self
            .sdo_transact(node_id, &mut client, first)?
            .expect("read transactions complete with a value"))
    }

    fn sdo_write(
        &mut self,
        node_id: u8,
        addr: Address,
        raw: &[u8],
    ) -> std::result::Result<(), SdoAbortCode> {
        let mut client = SdoClient::new();
        let first = client.write(addr, raw).expect("non-empty payload");
        self.sdo_transact(node_id, &mut client, first)?;
        Ok(())
    }
}

fn boot_all(bus: &mut Bus) {
    for i in 0..bus.nodes.len() {
        bus.nodes[i].boot();
    }
}

// --- ported cases ---------------------------------------------------------------

/// test_local.py:35 — write on the slave side directly, read over SDO.
#[test]
fn expedited_upload_roundtrip() {
    let mut bus = Bus::new();
    boot_all(&mut bus);
    bus.node(SLAVE_A)
        .od_mut()
        .set_value(Address::new(0x1400, 1), Value::Uint(0x99))
        .unwrap();
    assert_eq!(
        bus.sdo_read(SLAVE_A, Address::new(0x1400, 1), DataType::Unsigned32)
            .unwrap(),
        Value::Uint(0x99)
    );
}

/// test_local.py:40 — python expects an error here ("Unexpected response
/// 0x41"); our client transparently degrades block→segmented, so the case
/// becomes POSITIVE (D1): block upload of a short object succeeds.
#[test]
fn block_upload_degrades_gracefully() {
    let mut bus = Bus::new();
    boot_all(&mut bus);
    let payload = b"TEST DEVICE".to_vec();
    bus.node(SLAVE_A)
        .od_mut()
        .set_value(Address::new(0x2101, 0), Value::Bytes(payload.clone()))
        .unwrap();

    let mut client = SdoClient::new();
    let first = client.read_block(Address::new(0x2101, 0), DataType::Domain);
    let completed = bus.sdo_transact(SLAVE_A, &mut client, first).unwrap();
    assert_eq!(completed, Some(Value::Bytes(payload)));
}

/// test_local.py:48 — python server rejects block download with
/// `0x05040001`; OUR server supports it, so the case is positive (adapted):
/// a >887-byte download exercises the full block path with CRC.
#[test]
fn block_download_supported_and_integrity_checked() {
    let mut bus = Bus::new();
    boot_all(&mut bus);
    let data: Vec<u8> = (0..900_u16).map(|i| (i % 251) as u8).collect();
    bus.sdo_write(SLAVE_A, Address::new(0x2101, 0), &data)
        .unwrap();
    assert_eq!(
        bus.node(SLAVE_A)
            .od()
            .read(Address::new(0x2101, 0))
            .unwrap(),
        Value::Bytes(data)
    );
}

/// test_local.py:57 — default visible string via SDO.
#[test]
fn visible_string_default_value() {
    let mut bus = Bus::new();
    boot_all(&mut bus);
    assert_eq!(
        bus.sdo_read(SLAVE_A, Address::new(0x1008, 0), DataType::VisibleString)
            .unwrap(),
        Value::Str("TEST DEVICE".into())
    );
}

/// test_local.py:61 — REAL32 default 5.2.
#[test]
fn real_default_value() {
    let mut bus = Bus::new();
    boot_all(&mut bus);
    match bus
        .sdo_read(SLAVE_A, Address::new(0x2010, 0), DataType::Real32)
        .unwrap()
    {
        Value::Real(v) => assert!((v - 5.2).abs() < 1e-6),
        other => panic!("expected real, got {other:?}"),
    }
}

/// test_local.py:65 — zero-length string read. D2 adaptation was to return an
/// empty string instead of aborting; docs/DESIGN-DECISIONS.md 审-S2 aligned the server with
/// both references (CANopenNode `CO_SDOserver.c:557-563` aborts
/// `0x0604_0047`, python `sdo/server.py:68-72` aborts `0x0800_0024`) — the
/// CANopenNode code is used, so the read surfaces an abort.
#[test]
fn zero_length_read_aborts() {
    let mut bus = Bus::new();
    boot_all(&mut bus);
    bus.node(SLAVE_A)
        .od_mut()
        .set_value(Address::new(0x1008, 0), Value::Str(String::new()))
        .unwrap();
    match bus.sdo_read(SLAVE_A, Address::new(0x1008, 0), DataType::VisibleString) {
        Err(SdoAbortCode::GeneralInternalIncompatibility) => {}
        other => panic!("expected 0x0604_0047 abort, got {other:?}"),
    }
}

/// test_local.py:72 — segmented upload after renaming.
#[test]
fn segmented_upload_read() {
    let mut bus = Bus::new();
    boot_all(&mut bus);
    bus.node(SLAVE_A)
        .od_mut()
        .set_value(
            Address::new(0x2000, 0),
            Value::Str("Some cool device".into()),
        )
        .unwrap();
    assert_eq!(
        bus.sdo_read(SLAVE_A, Address::new(0x2000, 0), DataType::VisibleString)
            .unwrap(),
        Value::Str("Some cool device".into())
    );
}

/// test_local.py:77 — expedited download verified on the slave OD.
#[test]
fn expedited_download_roundtrip() {
    let mut bus = Bus::new();
    boot_all(&mut bus);
    bus.sdo_write(SLAVE_A, Address::new(0x2004, 0), &0xfeffu16.to_le_bytes())
        .unwrap();
    assert_eq!(
        bus.node(SLAVE_A)
            .od()
            .read(Address::new(0x2004, 0))
            .unwrap(),
        Value::Uint(0xfeff)
    );
}

/// test_local.py:82 — wrong width aborts with the generic type-mismatch code
/// (D4-aligned); the correct width then succeeds.
#[test]
fn expedited_download_wrong_datatype() {
    let mut bus = Bus::new();
    boot_all(&mut bus);
    assert_eq!(
        bus.sdo_write(SLAVE_A, Address::new(0x2001, 0), &[10, 10, 10, 10]),
        Err(SdoAbortCode::TypeMismatch)
    );
    bus.sdo_write(SLAVE_A, Address::new(0x2001, 0), &[10, 10])
        .unwrap();
    assert_eq!(
        bus.node(SLAVE_A)
            .od()
            .read(Address::new(0x2001, 0))
            .unwrap(),
        Value::Int(0x0A0A)
    );
}

/// test_local.py:92 — segmented download into the writable string.
#[test]
fn segmented_download_string() {
    let mut bus = Bus::new();
    boot_all(&mut bus);
    bus.sdo_write(SLAVE_A, Address::new(0x2000, 0), b"Another cool device")
        .unwrap();
    assert_eq!(
        bus.node(SLAVE_A)
            .od()
            .read(Address::new(0x2000, 0))
            .unwrap(),
        Value::Str("Another cool device".into())
    );
}

/// test_local.py:97 — setting the heartbeat time starts heartbeating; state
/// byte shows PRE-OPERATIONAL (device booted but not started).
#[test]
fn slave_sends_heartbeat_after_period_configured() {
    let mut bus = Bus::new();
    boot_all(&mut bus);
    bus.sdo_write(SLAVE_A, Address::new(0x1017, 0), &100u16.to_le_bytes())
        .unwrap();

    let mut seen_preop = false;
    let mut now = 0u64;
    while now < 500 {
        now += 20;
        for frame in bus.node(SLAVE_A).poll(now) {
            if frame.cob_id == 0x700 + u16::from(SLAVE_A) && frame.payload() == [0x7F] {
                seen_preop = true;
            }
        }
    }
    assert!(seen_preop, "no PRE-OPERATIONAL heartbeat observed");
}

/// test_local.py:107 — INITIALISING → PRE-OPERATIONAL transition surfaces as
/// boot-up frame first, then pre-operational heartbeats.
#[test]
fn bootup_then_preoper_heartbeat_sequence() {
    let mut node = Node::new(NodeId::new(SLAVE_A).unwrap(), sample_od(SLAVE_A)).unwrap();
    let bootup = node.boot().unwrap();
    assert_eq!(bootup.cob_id, 0x702);
    assert_eq!(bootup.payload(), &[0x00]);

    node.od_mut()
        .set_value(Address::new(0x1017, 0), Value::Uint(100))
        .unwrap();
    // Heartbeats after boot show PRE-OPERATIONAL; the boot-up byte itself was
    // already returned by boot().
    let mut saw_preop = false;
    let mut now = 50u64;
    while now < 400 {
        now += 25;
        for frame in node.poll(now) {
            if frame.cob_id == 0x702 && frame.payload() == [0x7F] {
                saw_preop = true;
            }
        }
    }
    assert!(saw_preop);
}

/// test_local.py:118 — an explicit client abort is recorded by the server.
#[test]
fn receive_abort_request_is_recorded() {
    let mut bus = Bus::new();
    boot_all(&mut bus);
    bus.send_abort(
        SLAVE_A,
        Address::new(0, 0),
        0x0504_0003, // invalid sequence number
    );
    assert_eq!(
        bus.node(SLAVE_A).take_last_received_abort(),
        Some(SdoAbortCode::InvalidSequenceNumber)
    );
}

/// test_local.py:125 — NMT start puts the slave into OPERATIONAL.
#[test]
fn start_remote_node() {
    let mut bus = Bus::new();
    boot_all(&mut bus);
    bus.nmt(0x01, SLAVE_A); // start remote node, directed
    assert_eq!(
        bus.node(SLAVE_A).nmt_state(),
        canopen_core::nmt::NmtState::Operational
    );
}

/// test_local.py:133 — two nodes on the bus keep independent strings.
#[test]
fn two_nodes_on_the_bus_are_independent() {
    let mut bus = Bus::new();
    boot_all(&mut bus);
    bus.sdo_write(SLAVE_A, Address::new(0x2000, 0), b"Some cool device")
        .unwrap();
    bus.sdo_write(SLAVE_B, Address::new(0x2000, 0), b"Some cool device2")
        .unwrap();
    assert_eq!(
        bus.sdo_read(SLAVE_A, Address::new(0x2000, 0), DataType::VisibleString)
            .unwrap(),
        Value::Str("Some cool device".into())
    );
    assert_eq!(
        bus.sdo_read(SLAVE_B, Address::new(0x2000, 0), DataType::VisibleString)
            .unwrap(),
        Value::Str("Some cool device2".into())
    );
}

/// test_local.py:142 — abort matrix. D3 adaptation: reading 0x1001 returns
/// the materialised zero value instead of 0x060A0023 (our OD always holds a
/// concrete value).
#[test]
fn abort_matrix() {
    let mut bus = Bus::new();
    boot_all(&mut bus);

    // Object does not exist.
    assert_eq!(
        bus.sdo_read(SLAVE_A, Address::new(0x1234, 0), DataType::Unsigned32),
        Err(SdoAbortCode::ObjectDoesNotExist)
    );
    // Sub-index does not exist (sub3 deliberately missing in sample layout).
    assert_eq!(
        bus.sdo_read(SLAVE_A, Address::new(0x1018, 100), DataType::Unsigned32),
        Err(SdoAbortCode::SubindexDoesNotExist)
    );
    // D3: readable entry returns its value rather than aborting.
    assert_eq!(
        bus.sdo_read(SLAVE_A, Address::new(0x1001, 0), DataType::Unsigned8)
            .unwrap(),
        Value::Uint(0)
    );
}

/// test_local.py:166 — read/write callbacks. Our equivalent is
/// callback-backed entries: reads serve a crafted value; writes are captured
/// with their address.
#[test]
fn callbacks_capture_writes_and_serve_reads() {
    let mut bus = Bus::new();
    boot_all(&mut bus);

    let recorder = Arc::new(RecordingAccess::default());
    // Read-side: ro entry serving the crafted value (mirrors the python read
    // callback returning 0x0201).
    bus.node(SLAVE_A).od_mut().insert(
        Address::new(0x1003, 5),
        Entry::new(
            "Pre-defined error field",
            DataType::Unsigned32,
            AccessType::Ro,
            Storage::Callback(recorder.clone()),
        ),
    );
    // Write-side: rw entry capturing the last written address.
    bus.node(SLAVE_A).od_mut().insert(
        Address::new(0x2102, 0),
        Entry::new(
            "Callback write target",
            DataType::Unsigned32,
            AccessType::Rw,
            Storage::Callback(recorder.clone()),
        ),
    );

    assert_eq!(
        bus.sdo_read(SLAVE_A, Address::new(0x1003, 5), DataType::Unsigned32)
            .unwrap(),
        Value::Uint(0x0201)
    );
    bus.sdo_write(SLAVE_A, Address::new(0x2102, 0), &[0x03, 0x04, 0x00, 0x00])
        .unwrap();
    assert_eq!(*recorder.last_write.lock().unwrap(), Some((0x2102, 0)));
}
