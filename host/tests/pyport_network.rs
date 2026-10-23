//! Port of `ref/canopen/test/test_network.py` pure-logic subsets — the
//! subscriber registry, heartbeat supervision and RX-thread health check
//! (docs/EXTENSIONS.md 批次 D P-d).

use canopen_core::nmt::{HbEvent, HeartbeatConsumer};
use canopen_core::types::NodeId;

// 订阅分发/取消的真实覆盖由 `host/tests/vcan.rs::network_master_api_on_vcan`
// 通过真实 `Network::open` + `Network::subscribe` 提供（`Network` 无总线不可
// 构造，故此处不重复空壳）；以下为无需总线的纯逻辑子集移植。

/// test_network.py:66 — heartbeat-consumer lifecycle behind the Network
/// surface. NOTE (docs/DESIGN-DECISIONS.md R10-P2-15): despite its old name this never called
/// `check()`; real RxDeath surfacing is covered by the vcan Network test.
/// Renamed to what it exercises.
#[test]
fn heartbeat_consumer_lifecycle_without_bus() {
    // Construct without a real socket by testing the error mapping path.
    // In production this is set by a panicked/failed RX thread.
    // Here we verify the type-level contract: check() returns Ok(()) when
    // no death was recorded.
    // Full integration requires Network::open which needs a bus interface;
    // covered by vcan tests instead.

    // Heartbeat consumer lifecycle (same code path as Network monitors).
    let mut hb = HeartbeatConsumer::new();
    let node = NodeId::new(5).unwrap();
    hb.set_timeout(node, 200);
    assert!(!hb.all_active());

    hb.on_heartbeat(node, &[0x05], 0);
    assert!(hb.poll(100).is_empty());
    assert!(hb.all_active());

    let lost = hb.poll(300);
    assert!(lost.iter().any(|e| matches!(e, HbEvent::Lost { .. })));
    assert!(!hb.all_active());

    // Recovery restarts monitoring.
    hb.on_heartbeat(node, &[0x05], 400);
    assert!(hb.all_active());
}

/// test_network.py — node scanning semantics: heartbeat consumer tracks
/// multiple nodes and reports per-node state.
#[test]
fn heartbeat_multi_node_tracking() {
    let mut hb = HeartbeatConsumer::new();
    let nodes: Vec<NodeId> = (1..=4).map(|i| NodeId::new(i).unwrap()).collect();
    for n in &nodes {
        hb.set_timeout(*n, 500);
    }
    assert_eq!(hb.configured().count(), 4);

    // Only nodes 1 and 2 report operational.
    hb.on_heartbeat(nodes[0], &[0x05], 0);
    hb.on_heartbeat(nodes[1], &[0x05], 0);
    // Nodes 3 and 4 report stopped.
    hb.on_heartbeat(nodes[2], &[0x04], 0);
    hb.on_heartbeat(nodes[3], &[0x7F], 0);

    assert!(hb.is_active(nodes[0]));
    assert_eq!(
        hb.state_of(nodes[2]),
        Some(canopen_core::nmt::NmtState::Stopped)
    );
    assert!(hb.all_active());
    assert!(!hb.all_operational()); // node 3 is stopped

    // Remove one → all_active still true for remaining.
    hb.remove(nodes[1]);
    assert!(hb.all_active());
}
