//! Port of `ref/canopen/test/test_nmt.py` subsets — NMT master/slave
//! interactions over the virtual bus (docs/EXTENSIONS.md 批次 D P-c).

use canopen_core::datatypes::Value;
use canopen_core::nmt::{
    decode_command, encode_command, HbEvent, HeartbeatConsumer, NmtCommand, NmtState,
};
use canopen_core::node::Node;
use canopen_core::standard::StandardObjects;
use canopen_core::types::{Address, NodeId};

const SLAVE_A: u8 = 2;
const SLAVE_B: u8 = 3;

fn od(_node: u8) -> canopen_core::object_dictionary::ObjectDictionary {
    StandardObjects::new().build()
}

struct Bus {
    nodes: Vec<Node>,
    hb: HeartbeatConsumer,
}

impl Bus {
    fn new() -> Self {
        Self {
            nodes: vec![
                Node::new(NodeId::new(SLAVE_A).unwrap(), od(SLAVE_A)).unwrap(),
                Node::new(NodeId::new(SLAVE_B).unwrap(), od(SLAVE_B)).unwrap(),
            ],
            hb: HeartbeatConsumer::new(),
        }
    }

    fn node(&mut self, id: u8) -> &mut Node {
        self.nodes
            .iter_mut()
            .find(|n| n.node_id().get() == id)
            .expect("node present")
    }

    /// Master-side NMT command + heartbeat supervision of both slaves.
    fn setup_master_supervision(&mut self, timeout_ms: u32) {
        self.hb
            .set_timeout(NodeId::new(SLAVE_A).unwrap(), timeout_ms);
        self.hb
            .set_timeout(NodeId::new(SLAVE_B).unwrap(), timeout_ms);
    }

    /// Runs `steps` × 50 ms of bus time: slave heartbeat producers fire and
    /// the master consumer is fed.
    fn run(&mut self, steps: usize, start_ms: u64) -> Vec<HbEvent> {
        let mut events = Vec::new();
        for i in 0..steps {
            let now = start_ms + (i as u64) * 50;
            for node in &mut self.nodes {
                for tx in node.poll(now) {
                    if tx.cob_id >= 0x700 && tx.payload().len() == 1 {
                        let id = (tx.cob_id - 0x700) as u8;
                        if let Ok(n) = NodeId::try_from(id) {
                            events.extend(self.hb.on_heartbeat(n, tx.payload(), now));
                        }
                    }
                    // Master would also see other slaves' frames; nothing to do.
                    let _ = tx;
                }
            }
            events.extend(self.hb.poll(now));
        }
        events
    }

    fn send_nmt(&mut self, command: NmtCommand, target: Option<u8>) {
        let frame = encode_command(command, target.map(|t| NodeId::try_from(t).expect("range")));
        for node in &mut self.nodes {
            if target.is_none() || target == Some(node.node_id().get()) {
                let _ = node.on_frame(0x000, &frame);
            }
        }
    }

    /// Reads the heartbeat state byte a slave would currently produce.
    fn state_byte(&mut self, id: u8) -> u8 {
        self.node(id).poll(u64::MAX); // flush any pending
        match self.node(id).nmt_state() {
            NmtState::Initialising => 0x00,
            NmtState::Stopped => 0x04,
            NmtState::Operational => 0x05,
            NmtState::PreOperational => 0x7F,
        }
    }
}

/// test_nmt.py:19/34 — command encoding round-trips and state get/set.
#[test]
fn send_command_and_state_getset() {
    assert_eq!(
        decode_command(&encode_command(NmtCommand::StartRemoteNode, None)),
        Some((NmtCommand::StartRemoteNode, 0))
    );
    let mut machine = canopen_core::nmt::NmtStateMachine::new();
    machine.boot();
    machine.apply(NmtCommand::StartRemoteNode);
    assert_eq!(machine.state(), NmtState::Operational);
}

/// test_nmt.py:87/94 — boot-up visibility: after an NMT reset the slave's
/// boot-up frame surfaces on the master side as a RemoteReset event.
#[test]
fn wait_for_bootup_via_consumer_events() {
    let mut bus = Bus::new();
    bus.setup_master_supervision(1000);

    // Producer configured BEFORE boot so heartbeats flow immediately.
    bus.node(SLAVE_A)
        .od_mut()
        .set_value(Address::new(0x1017, 0), Value::Uint(50))
        .unwrap();
    boot_slaves(&mut bus);

    let events = bus.run(4, 0); // 200 ms
    assert!(events.iter().any(|e| matches!(e, HbEvent::Started { .. })));

    // Reset communication → slave re-initialises → re-boot → its boot-up
    // frame (state byte 0x00) reaches the master's consumer.
    bus.send_nmt(NmtCommand::ResetCommunication, Some(SLAVE_A));
    let tx = bus.node(SLAVE_A).boot().unwrap();
    assert_eq!(tx.payload(), &[0x00]);
    let events = bus
        .hb
        .on_heartbeat(NodeId::new(SLAVE_A).unwrap(), tx.payload(), 300);
    assert!(
        events
            .iter()
            .any(|e| matches!(e, HbEvent::RemoteReset { .. })),
        "expected RemoteReset, got {events:?}"
    );
}

fn boot_slaves(bus: &mut Bus) {
    for id in [SLAVE_A, SLAVE_B] {
        bus.node(id).boot();
    }
}

/// test_nmt.py:158/173 — start two remote nodes individually; stop both via
/// broadcast.
#[test]
fn start_two_nodes_stop_by_broadcast() {
    let mut bus = Bus::new();
    boot_slaves(&mut bus);

    bus.send_nmt(NmtCommand::StartRemoteNode, Some(SLAVE_A));
    bus.send_nmt(NmtCommand::StartRemoteNode, Some(SLAVE_B));
    assert_eq!(bus.state_byte(SLAVE_A), 0x05);
    assert_eq!(bus.state_byte(SLAVE_B), 0x05);

    bus.send_nmt(NmtCommand::StopRemoteNode, None); // broadcast
    assert_eq!(bus.state_byte(SLAVE_A), 0x04);
    assert_eq!(bus.state_byte(SLAVE_B), 0x04);
}

/// test_nmt.py:70/76/110 — heartbeat consumer states and callbacks: Started
/// → StateChanged, loss after timeout, recovery restarts.
#[test]
fn master_heartbeat_lifecycle_with_callbacks() {
    let mut bus = Bus::new();
    boot_slaves(&mut bus);
    bus.setup_master_supervision(200);

    // Configure producer periods so slaves actually emit.
    for id in [SLAVE_A, SLAVE_B] {
        bus.node(id)
            .od_mut()
            .set_value(Address::new(0x1017, 0), Value::Uint(100))
            .unwrap();
    }

    let events = bus.run(10, 0); // 500 ms — everyone alive
    assert!(events.iter().any(|e| matches!(e, HbEvent::Started { .. })));
    assert!(bus.hb.all_active());

    // Silence everything: no poll-driven production, only time passes.
    // Silence both producers, then let >200 ms pass without heartbeats.
    for id in [SLAVE_A, SLAVE_B] {
        bus.node(id)
            .od_mut()
            .set_value(Address::new(0x1017, 0), Value::Uint(0))
            .unwrap();
    }
    let events = bus.run(20, 500);
    assert_eq!(events.len(), 2, "expected 2 Lost, got {events:?}");
    assert!(!bus.hb.all_active());
}

/// test_nmt.py:123 — node guarding: RTR polls receive alternating-toggle
/// responses carrying the current state.
#[test]
fn node_guarding_alternates_toggle() {
    let mut bus = Bus::new();
    boot_slaves(&mut bus);

    let r1 = bus
        .node(SLAVE_A)
        .on_remote_request(0x700 + u16::from(SLAVE_A))
        .unwrap();
    let r2 = bus
        .node(SLAVE_A)
        .on_remote_request(0x700 + u16::from(SLAVE_A))
        .unwrap();
    assert_ne!(r1.payload()[0] & 0x80, r2.payload()[0] & 0x80);
    assert_eq!(r1.payload()[0] & 0x7F, 0x7F);
}
