//! The CANopen node integrator: wires the object dictionary, NMT state
//! machine, SDO server, PDO slots, EMCY producer and heartbeat scheduling
//! into one sans-I/O device.
//!
//! Contract: `on_frame(cob_id, data) -> Vec<TxFrame>` consumes every frame
//! addressed to the node; producer methods ([`Node::poll`], boot-up,
//! emergency, event TPDOs) return frames for the surrounding loop to
//! transmit. No I/O and no clock reads — all timing uses monotonic
//! milliseconds supplied by the caller (CANopenNode's `timerNext`
//! philosophy, docs/REFERENCES.md S6).
//!
//! State gating per CiA 301:
//! - SDO is served in PreOperational and Operational;
//! - RPDO reception and SYNC-triggered/event TPDO transmission only in
//!   Operational;
//! - heartbeat/boot-up run once booted regardless of state.

use crate::datatypes::Value;
use crate::emcy::EmcyProducer;
use crate::nmt::{
    decode_command, encode_heartbeat, HeartbeatConsumer, NmtCommand, NmtState, NmtStateMachine,
};
use crate::object_dictionary::ObjectDictionary;
use crate::pdo::{self, PdoConfig, PdoMapping, TransmissionType};
use crate::sdo::SdoServer;
use std::sync::Arc;

use crate::types::{Address, Error, NodeId, Result};

/// An outgoing CANopen frame produced by [`Node`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxFrame {
    /// COB-ID (11-bit).
    pub cob_id: u16,
    /// Payload bytes; `data[..len]` is valid.
    pub data: [u8; 8],
    /// Valid payload length (`0..=8`).
    pub len: usize,
}

impl TxFrame {
    /// Builds a frame from a payload slice.
    ///
    /// # Errors
    /// [`Error::BadLength`] when `payload.len() > 8`.
    pub fn new(cob_id: u16, payload: &[u8]) -> Result<Self> {
        if payload.len() > 8 {
            return Err(Error::BadLength {
                declared: 8,
                actual: payload.len(),
            });
        }
        let mut data = [0u8; 8];
        data[..payload.len()].copy_from_slice(payload);
        Ok(Self {
            cob_id,
            data,
            len: payload.len(),
        })
    }

    /// Payload slice.
    #[must_use]
    pub fn payload(&self) -> &[u8] {
        &self.data[..self.len]
    }
}

/// One configured transmit-PDO slot.
#[derive(Debug)]
struct TpdoSlot {
    /// Communication-record index (0x1800+n), used to address the slot by
    /// PDO number in [`Node::request_tpdo`] (`BUGS.md` P3-19).
    comm_index: u16,
    config: PdoConfig,
    mapping: PdoMapping,
    /// Resolved transmission type (241..251 rejected at build time).
    tt: TransmissionType,
    /// SYNCs observed since the last transmission (cyclic divider).
    /// Seeded with `n - (n/2 + 1)` so the first SyncCyclic frame goes out
    /// at the middle of the first period, matching CANopenNode
    /// `CO_PDO.c:1315-1321` (BUGS.md #27).
    syncs_seen: u32,
    /// Last transmission in monotonic ms (inhibit + event timer).
    last_tx_ms: Option<u64>,
    /// Application-requested transmission (event-driven trigger).
    pending_event: bool,
    /// SYNC start-value phase lock (S14 / CANopenNode `CO_PDO.c:1315-1336`):
    /// set once when the global counter first matches the configured start
    /// value and kept for the whole Operational session — cyclic counting
    /// then proceeds independently of further counter values. Cleared when
    /// NMT leaves Operational (`BUGS.md` P2-1: re-locking after every send
    /// silenced cyclic TPDOs until counter wraparound).
    phase_locked: bool,
    /// A SYNC was received this Operational session (`BUGS.md` M2): a
    /// type-252 (RTR-only-sync) TPDO answers a remote request only after
    /// the first SYNC — CiA 301 §7.4.3, the response is the last SYNC
    /// sample. Cleared when NMT leaves Operational, like `syncs_seen`.
    sync_received: bool,
    /// 类型 252（RtrOnlySync）PDO 在最近一次 SYNC 处采样的载荷快照
    /// （BUGS.md M4）：规范规定 RTR 应答的数据应在"最后 SYNC"时刻采样，
    /// 而非当前实时 OD 值。每个有效 SYNC 刷新一次；`None` 表示本会话
    /// 尚无有效样本（与 `sync_received` 状态一致）。
    sync_sample: Option<([u8; 8], usize)>,
}

impl TpdoSlot {
    fn cob_id(&self) -> u16 {
        self.config.cob_id()
    }

    fn inhibit_ms(&self) -> u64 {
        // Inhibit time unit is 100 µs; round up to whole ms at our API
        // granularity.
        u64::from(self.config.inhibit_time_100us).div_ceil(10)
    }
}

/// One configured receive-PDO slot.
#[derive(Debug)]
struct RpdoSlot {
    /// Communication-record index (0x1400+n), reported in timeout events.
    comm_index: u16,
    config: PdoConfig,
    mapping: PdoMapping,
    /// Transmission type snapshot (`BUGS.md` P2-4): synchronous slots buffer
    /// received data until the next SYNC instead of applying it immediately.
    tt: TransmissionType,
    /// Buffered payload of a synchronous RPDO awaiting the next SYNC
    /// (double-buffer semantics: "takes effect after the next SYNC",
    /// CANopenNode `CO_PDO.c:717-760`). `None` when nothing is pending.
    sync_pending: Option<([u8; 8], usize)>,
    /// A frame arrived since the last [`Node::poll`] window (P2-7).
    rx_seen: bool,
    /// Poll timestamp when traffic was last observed on this slot; `None`
    /// until the first reception arms the deadman (`BUGS.md` P2-7).
    last_rx_ms: Option<u64>,
    /// Timeout already reported for the current silent window.
    reported: bool,
}

/// A complete CANopen slave node.
pub struct Node {
    od: ObjectDictionary,
    nmt: NmtStateMachine,
    sdo: SdoServer,
    emcy: EmcyProducer,
    node_id: NodeId,
    tpdos: Vec<TpdoSlot>,
    rpdos: Vec<RpdoSlot>,
    guarding_toggle: bool,
    last_hb_ms: Option<u64>,
    /// BUGS.md 八M4: set by `boot()`/reset until the first `poll` after
    /// boot-up — the core has no clock, so the *start of the heartbeat
    /// period* is anchored at the first observed `now_ms` instead of an
    /// assumed zero. Without this a node running on a real monotonic clock
    /// (now ≫ 0x1017) would fire its first periodic heartbeat back-to-back
    /// with the boot-up frame.
    hb_arm_pending: bool,
    /// LSS slave state machine when the device participates in node-id
    /// assignment; `unconfigured` gates all non-LSS services.
    lss: Option<crate::lss::LssSlave>,
    unconfigured: bool,
    /// Parameter-storage backend.
    storage: Option<Arc<dyn crate::storage::ParameterStorage>>,
    /// Construction-time dictionary snapshot — the restore-to-default
    /// reference for [`crate::storage::ParameterStorage::reset`].
    defaults: ObjectDictionary,
    /// Set once `attach_storage` has taken the defaults snapshot; later
    /// attaches must keep the earliest snapshot.
    defaults_snapshot_taken: bool,
    /// Errors swallowed while (re-)assembling PDO slots or unpacking RPDOs;
    /// drained by [`Node::take_config_errors`] so misconfiguration never
    /// disappears without a trace (BUGS.md m17). Bounded ring: once
    /// [`MAX_CONFIG_ERRORS`] entries are held, further errors increment
    /// `config_errors_dropped` instead of growing unboundedly (P2-3).
    config_errors: Vec<crate::types::Error>,
    config_errors_dropped: u64,
    /// EMCY payloads waiting out their 0x1015 inhibit window (`BUGS.md`
    /// P2-6); released by [`Node::poll`].
    pending_emcy: std::collections::VecDeque<[u8; 8]>,
    /// Monotonic-ms timestamp of the last EMCY transmission (inhibit base).
    last_emcy_ms: Option<u64>,
    /// RPDO event-timer timeouts (communication addresses), drained via
    /// [`Node::take_rpdo_timeouts`] (`BUGS.md` P2-7).
    rpdo_timeouts: std::collections::VecDeque<Address>,
    /// Consumer-heartbeat nodes currently installed *from OD 0x1016*
    /// (`BUGS.md` P1-11); programmatic `HeartbeatConsumer` users are
    /// never touched by the OD resync.
    hb_od_consumers: Vec<NodeId>,
    /// OD-configured heartbeat consumer (§10.3 supervision).
    hb_consumer: crate::nmt::HeartbeatConsumer,
    /// Optional consumer for foreign emergency frames (0x80+n, n≠self);
    /// `None` by default — a slave does not consume peer EMCY unless an
    /// application installs a consumer (`BUGS.md` M3).
    emcy_consumer: Option<crate::emcy::EmcyConsumer>,
    /// Foreign heartbeat frames received since the last poll.
    hb_rx_queue: std::collections::VecDeque<(NodeId, [u8; 8])>,
    /// TIME frames received on 0x100, drained via [`Node::take_time_events`]
    /// (`BUGS.md` P3-21).
    time_rx_queue: std::collections::VecDeque<[u8; 6]>,
    /// SDO session timeout in ms (`None` = disabled); enforced via
    /// [`Node::touch_sdo_activity`] / [`Node::sdo_timeout_due`] with a
    /// caller-supplied clock (§5.5, `BUGS.md` P2-8). The underlying
    /// `SdoServer` keeps a sliding inactivity window, so sustained traffic
    /// never trips the timer (`BUGS.md` R10-P1-4).
    sdo_timeout_ms: Option<u64>,
    /// Heartbeat events pending [`Node::take_hb_events`].
    pending_hb_events: Vec<crate::nmt::HbEvent>,
}

/// Upper bound on retained configuration errors before counting drops (P2-3).
const MAX_CONFIG_ERRORS: usize = 64;
/// Upper bound for the receive queues (`hb_rx_queue` / `time_rx_queue`).
///
/// Sans-I/O nodes process frames faster than they are drained only if the
/// application stops polling; without a cap a busy 0x100/0x700 COB would
/// grow the queues without bound (BUGS.md 八L15). Oldest frames are
/// dropped, mirroring the EMCY active-table policy (M5).
const MAX_RX_QUEUE: usize = 256;

impl Node {
    /// Records a configuration error, dropping (and counting) it once the
    /// bounded buffer is full so sustained bus noise cannot exhaust memory.
    fn record_config_error(&mut self, e: crate::types::Error) {
        if self.config_errors.len() < MAX_CONFIG_ERRORS {
            self.config_errors.push(e);
        } else {
            self.config_errors_dropped += 1;
        }
    }

    /// Reconfigures the heartbeat consumer from OD 0x1016 contents (§6.5,
    /// `BUGS.md` P1-11). Every non-zero entry `nodeId<<16 | timeout_ms`
    /// installs/updates a consumer immediately; configured consumers absent
    /// from the dictionary are removed. A nodeId appearing twice rejects the
    /// later entry (zeroed back) with a recorded error — CiA 301 allows a
    /// node under exactly one consumer.
    ///
    /// Only consumers installed through this path are ever removed here;
    /// programmatic `HeartbeatConsumer::set_timeout` users are untouched.
    fn resync_hb_consumers_from_od(&mut self) {
        let mut wanted: Vec<(u8, u32)> = Vec::new();
        for sub in 1..=0xFEu8 {
            let Ok(Value::Uint(v)) = self.od.read(Address::new(0x1016, sub)) else {
                continue;
            };
            let node = ((v >> 16) & 0xFF) as u8;
            let ms = (v & 0xFFFF) as u32;
            if node == 0 || ms == 0 {
                continue;
            }
            if wanted.iter().any(|(n, _)| *n == node) {
                let addr = Address::new(0x1016, sub);
                let _ = self.od.set_value(addr, Value::Uint(0));
                self.record_config_error(crate::types::Error::Other(
                    "0x1016: duplicate consumer nodeId rejected",
                ));
                continue;
            }
            wanted.push((node, ms));
        }
        let mut mapped: Vec<NodeId> = Vec::new();
        for (node, ms) in wanted {
            match NodeId::try_from(node) {
                Ok(n) => {
                    self.hb_consumer.set_timeout(n, ms);
                    mapped.push(n);
                }
                Err(_) => {
                    // Out-of-range nodeId (node > 127): zero the entry and
                    // record it like duplicates — silent residue would be
                    // re-parsed on every resync (BUGS.md P3-15).
                    let sub = (1..=0xFEu8)
                        .find(|s| {
                            matches!(self.od.read(Address::new(0x1016, *s)),
                                Ok(Value::Uint(v)) if ((v >> 16) & 0xFF) as u8 == node)
                        })
                        .map(|s| Address::new(0x1016, s));
                    if let Some(addr) = sub {
                        let _ = self.od.set_value(addr, Value::Uint(0));
                    }
                    self.record_config_error(crate::types::Error::Other(
                        "0x1016: consumer nodeId out of 1..=127 rejected",
                    ));
                }
            }
        }
        for n in std::mem::take(&mut self.hb_od_consumers) {
            if !mapped.contains(&n) {
                self.hb_consumer.remove(n);
            }
        }
        self.hb_od_consumers = mapped;
    }
    /// Creates a node over `od` (which must already contain the mandatory
    /// baseline — see [`crate::standard::StandardObjects`] and
    /// [`crate::emcy::install_standard_objects`]).
    ///
    /// PDO slots are configured from whatever communication/mapping records
    /// the dictionary contains; call [`Node::configure_pdos_from_od`] after
    /// changing them.
    ///
    /// # Errors
    /// Propagates OD errors from the initial configuration pass.
    pub fn new(node_id: NodeId, od: ObjectDictionary) -> Result<Self> {
        let defaults = od.clone();
        let mut node = Self {
            od,
            nmt: NmtStateMachine::new(),
            sdo: SdoServer::new(),
            emcy: EmcyProducer::new(),
            node_id,
            tpdos: Vec::new(),
            rpdos: Vec::new(),
            guarding_toggle: false,
            last_hb_ms: None,
            hb_arm_pending: false,
            lss: None,
            unconfigured: false,
            storage: None,
            defaults,
            defaults_snapshot_taken: false,
            config_errors: Vec::new(),
            config_errors_dropped: 0,
            pending_emcy: std::collections::VecDeque::new(),
            last_emcy_ms: None,
            rpdo_timeouts: std::collections::VecDeque::new(),
            hb_od_consumers: Vec::new(),
            hb_consumer: crate::nmt::HeartbeatConsumer::default(),
            emcy_consumer: None,
            hb_rx_queue: std::collections::VecDeque::new(),
            time_rx_queue: std::collections::VecDeque::new(),
            sdo_timeout_ms: None,
            pending_hb_events: Vec::new(),
        };
        // BUGS.md 八M3: the `StandardObjects` baseline stores node-1
        // placeholders for node-dependent COB-IDs (0x1014 EMCY = 0x81,
        // 0x1280:1 SDO client = 0x601). `set_node_id` later rewrites them
        // when the id *changes*, but a node built directly at `node_id`
        // would otherwise keep node-1's EMCY COB forever — colliding with
        // the real node 1 on the bus. Rewrite the placeholder now.
        node.renumber_entry(
            Address::new(0x1014, 0),
            NodeId::new(1).unwrap_or(node_id),
            node_id,
        );
        node.renumber_entry(
            Address::new(0x1280, 1),
            NodeId::new(1).unwrap_or(node_id),
            node_id,
        );
        node.configure_pdos_from_od()?;
        Ok(node)
    }

    /// Creates a node that boots **unconfigured**: it serves only LSS on
    /// `0x7E5` until an id is assigned and adopted via
    /// [`Self::apply_lss_node_id`] (docs/IMPLEMENTATION.md §9).
    #[must_use]
    pub fn new_unconfigured(address: crate::lss::LssAddress, od: ObjectDictionary) -> Self {
        let defaults = od.clone();
        let mut node = Self {
            od,
            nmt: NmtStateMachine::new(),
            sdo: SdoServer::new(),
            emcy: EmcyProducer::new(),
            // Placeholder; meaningless while unconfigured (COB-IDs unused).
            node_id: NodeId::new(1).expect("static"),
            tpdos: Vec::new(),
            rpdos: Vec::new(),
            guarding_toggle: false,
            last_hb_ms: None,
            hb_arm_pending: false,
            lss: Some(crate::lss::LssSlave::new(
                address,
                crate::lss::UNCONFIGURED_NODE_ID,
            )),
            unconfigured: true,
            storage: None,
            defaults,
            defaults_snapshot_taken: false,
            config_errors: Vec::new(),
            config_errors_dropped: 0,
            pending_emcy: std::collections::VecDeque::new(),
            last_emcy_ms: None,
            rpdo_timeouts: std::collections::VecDeque::new(),
            hb_od_consumers: Vec::new(),
            hb_consumer: crate::nmt::HeartbeatConsumer::default(),
            emcy_consumer: None,
            hb_rx_queue: std::collections::VecDeque::new(),
            time_rx_queue: std::collections::VecDeque::new(),
            sdo_timeout_ms: None,
            pending_hb_events: Vec::new(),
        };
        let _ = node.configure_pdos_from_od();
        node
    }

    /// Adopts the pending LSS node-id after a master-assigned configuration:
    /// returns the new id once the node becomes fully operational-ready.
    /// Call this from the reset path that follows a successful
    /// configure-node-id/store session.
    pub fn apply_lss_node_id(&mut self) -> Option<NodeId> {
        let raw = self.lss.as_ref()?.pending_node_id();
        // CiA 305 §5.4.1: `0xFF` invalidates the node-id; the slave re-enters
        // the LSS init state, serving only LSS until a new id is assigned and
        // adopted (BUGS.md A2). `NodeId` cannot represent 0xFF, so signal the
        // reset with `None` after adopting.
        if raw == crate::lss::UNCONFIGURED_NODE_ID {
            self.lss.as_mut()?.adopt_pending();
            self.unconfigured = true;
            return None;
        }
        if !(1..=127).contains(&raw) {
            return None;
        }
        let previous = self.node_id;
        self.lss.as_mut()?.adopt_pending();
        if let Ok(id) = NodeId::try_from(raw) {
            self.node_id = id;
            self.unconfigured = false;
            // Re-point identity-shaped entries at the new node segment —
            // through the guarded rewrite so custom COB-IDs survive, exactly
            // like `set_node_id` (BUGS.md P3-6). 0x1280:1 follows too so an
            // LSS-configured node does not keep the stale 0x600 placeholder
            // (BUGS.md P2-2, see git history).
            self.renumber_entry(Address::new(0x1014, 0), previous, id);
            self.renumber_entry(Address::new(0x1280, 1), previous, id);
            // Keep PDO COB-IDs alive across re-numbering: entries still
            // carrying the previous predefined node segment follow along
            // (docs/IMPLEMENTATION.md §7.2; CANopenNode CO_PDO.c:384).
            self.rewrite_pdo_node_bits(previous, id);
            if let Err(e) = self.configure_pdos_from_od() {
                self.record_config_error(e);
            }
            Some(id)
        } else {
            None
        }
    }

    /// Re-assigns the node id outside LSS: identity-dependent default
    /// COB-IDs (EMCY 0x1014, PDO comm records 0x1400+/0x1800+ sub1 that
    /// still hold the predefined value for the *old* id) are rewritten and
    /// PDO slots rebuilt (docs/IMPLEMENTATION.md §7.2, `BUGS.md` P1-12).
    /// Custom COB-IDs are left untouched (CANopenNode semantics).
    pub fn set_node_id(&mut self, id: NodeId) {
        let previous = self.node_id;
        if previous == id {
            return;
        }
        self.node_id = id;
        self.renumber_entry(Address::new(0x1014, 0), previous, id);
        self.renumber_entry(Address::new(0x1280, 1), previous, id);
        self.rewrite_pdo_node_bits(previous, id);
        if let Err(e) = self.configure_pdos_from_od() {
            self.record_config_error(e);
        }
    }

    /// Predefined (connection-set) COB-ID pair `(old, new)` for an
    /// identity-dependent entry; `None` for entries without one.
    fn predefined_pair(index: u16, old: NodeId, new: NodeId) -> Option<(u32, u32)> {
        match index {
            0x1014 => Some((u32::from(old.emcy_cob_id()), u32::from(new.emcy_cob_id()))),
            // Default SDO client request COB-ID follows the node too so a
            // master inspecting 0x1280:1 does not read a stale 0x600
            // (`BUGS.md` P3-38).
            0x1280 => Some((0x600 + u32::from(old.get()), 0x600 + u32::from(new.get()))),
            idx if (0x1400..=0x15FF).contains(&idx) => {
                let num = (idx - 0x1400 + 1) as u8;
                Some((
                    u32::from(pdo::default_cob_id(false, num, old)?),
                    u32::from(pdo::default_cob_id(false, num, new)?),
                ))
            }
            idx if (0x1800..=0x19FF).contains(&idx) => {
                let num = (idx - 0x1800 + 1) as u8;
                Some((
                    u32::from(pdo::default_cob_id(true, num, old)?),
                    u32::from(pdo::default_cob_id(true, num, new)?),
                ))
            }
            _ => None,
        }
    }

    /// Rewrites one COB-ID entry when it matches the old node's predefined
    /// connection-set value (low 11 bits only; flags preserved).
    fn renumber_entry(&mut self, addr: Address, old: NodeId, new: NodeId) {
        let Some((expected_old, expected_new)) = Self::predefined_pair(addr.index, old, new) else {
            return;
        };
        let Ok(Value::Uint(v)) = self.od.read(addr) else {
            return;
        };
        let v = v as u32;
        if v & 0x7FF == expected_old {
            let nv = (v & !0x7FF) | expected_new;
            let _ = self.od.set_value(addr, Value::Uint(u64::from(nv)));
        }
    }

    /// Rewrites every PDO communication record's sub1 whose low bits still
    /// carry `old`'s predefined COB-ID to `new`.
    fn rewrite_pdo_node_bits(&mut self, old: NodeId, new: NodeId) {
        for index in 0x1400..=0x15FF {
            self.renumber_entry(Address::new(index, 1), old, new);
        }
        for index in 0x1800..=0x19FF {
            self.renumber_entry(Address::new(index, 1), old, new);
        }
    }

    /// Attaches an LSS slave to an already-configured node so masters can
    /// re-assign its id in the field (selective switch by identity).
    pub fn attach_lss(&mut self, address: crate::lss::LssAddress) {
        self.lss = Some(crate::lss::LssSlave::new(address, self.node_id.get()));
    }

    /// Attaches a parameter-storage backend. Requires 0x1010/0x1011 in the
    /// dictionary ([`crate::storage::install_standard_objects`]); the OD
    /// state at attach time becomes the restore-to-default snapshot.
    pub fn attach_storage(&mut self, backend: Arc<dyn crate::storage::ParameterStorage>) {
        if !self.defaults_snapshot_taken {
            // Take the snapshot once: attaching twice must not move the
            // factory-default reference forward.
            self.defaults = self.od.clone();
            self.defaults_snapshot_taken = true;
        }
        self.storage = Some(backend);
    }

    /// Returns and clears the most recent client-received SDO abort
    /// (visibility for tests and application diagnostics).
    pub fn take_last_received_abort(&mut self) -> Option<crate::sdo::SdoAbortCode> {
        self.sdo.take_last_received_abort()
    }

    /// Drains errors recorded while re-assembling PDO slots or unpacking
    /// received RPDO payloads. The node keeps operating with its working
    /// PDOs (per docs/IMPLEMENTATION.md §7.3 a partially misconfigured
    /// dictionary must not disable the rest), but nothing is swallowed
    /// silently — check this after configuration writes.
    pub fn take_config_errors(&mut self) -> std::vec::Drain<'_, crate::types::Error> {
        self.config_errors.drain(..)
    }

    /// Installs an EMCY consumer, enabling consumption of foreign
    /// emergency frames (0x80+n, n≠self) via [`Node::on_frame`]
    /// (`BUGS.md` M3). Without one, the slave does not consume peer EMCY
    /// and those frames fall through to the PDO matcher.
    pub fn set_emcy_consumer(&mut self, consumer: crate::emcy::EmcyConsumer) {
        self.emcy_consumer = Some(consumer);
    }

    /// Read access to the installed EMCY consumer, if any.
    #[must_use]
    pub fn emcy_consumer(&self) -> Option<&crate::emcy::EmcyConsumer> {
        self.emcy_consumer.as_ref()
    }

    /// Returns and clears how many configuration errors were dropped after
    /// the bounded buffer filled (`BUGS.md` P2-3).
    pub fn take_dropped_config_errors(&mut self) -> u64 {
        std::mem::take(&mut self.config_errors_dropped)
    }

    /// Whether an SDO transfer is in flight on the default channel.
    #[must_use]
    pub fn sdo_is_busy(&self) -> bool {
        self.sdo.is_busy()
    }

    /// Enables the SDO session timeout (§5.5, `BUGS.md` P2-8): a transfer
    /// with no frames for `timeout_ms` is torn down by
    /// [`Node::sdo_timeout_due`]. Disabled until called; the host
    /// `serve_node` loop turns it on at 1500 ms by default.
    pub fn set_sdo_timeout(&mut self, timeout_ms: u64) {
        self.sdo_timeout_ms = Some(timeout_ms);
    }

    /// Enables the SDO session timeout only when still disabled (used by the
    /// `serve_node` driver to install its default without overriding an
    /// application's explicit configuration).
    pub fn set_sdo_timeout_if_unset(&mut self, timeout_ms: u64) {
        if self.sdo_timeout_ms.is_none() {
            self.sdo_timeout_ms = Some(timeout_ms);
        }
    }

    /// Slides the SDO §5.5 inactivity window forward (any handled frame
    /// counts as activity; `BUGS.md` R10-P1-4).
    pub fn touch_sdo_activity(&mut self, now_ms: u64) {
        self.sdo.touch(now_ms);
    }

    /// Session-timeout maintenance: tears down a wedged SDO transfer once no
    /// frame arrived for the configured timeout and returns the timeout abort
    /// frame (`0x05040000`) for transmission — the host serve loop sends it,
    /// matching CANopenNode `CO_SDOserver.c:1076-1079,1496-1509`
    /// (`BUGS.md` C1). The inactivity window slides forward with
    /// [`Node::touch_sdo_activity`], so a long-running transfer with steady
    /// traffic is never torn down by a fixed total-session cap
    /// (`BUGS.md` R10-P1-4).
    pub fn sdo_timeout_due(&mut self, now_ms: u64) -> Option<TxFrame> {
        let timeout = self.sdo_timeout_ms?;
        let abort = self.sdo.on_timeout(now_ms, timeout)?;
        TxFrame::new(self.node_id.sdo_tx_cob_id(), &abort).ok()
    }

    /// Drains TIME frames received on COB-ID 0x100 as raw 6-byte stamps
    /// (ms-of-day little-endian u32 + days since 1984 u16) — `BUGS.md` P3-21.
    pub fn take_time_events(&mut self) -> Vec<[u8; 6]> {
        self.time_rx_queue.drain(..).collect()
    }

    /// Drains buffered heartbeat events (Started/Lost/RemoteReset/
    /// StateChanged) produced by the consumer since the last call.
    pub fn take_hb_events(&mut self) -> Vec<crate::nmt::HbEvent> {
        std::mem::take(&mut self.pending_hb_events)
    }

    /// Appends a heartbeat event to the bounded output queue; the oldest
    /// event is dropped once the cap is reached (BUGS.md 十M8, the same
    /// drop-oldest policy as the input queues in 八L15). Without the bound
    /// a long-running application that polls but rarely drains grows the
    /// queue without limit.
    fn push_hb_event(&mut self, event: crate::nmt::HbEvent) {
        if self.pending_hb_events.len() >= MAX_RX_QUEUE {
            self.pending_hb_events.remove(0);
        }
        self.pending_hb_events.push(event);
    }

    /// Dispatches storage magic words after successful SDO writes; other
    /// parameter writes are ignored.
    fn on_parameter_write(&mut self, addr: Address, value: Value) {
        // CiA 301 §7.2.2.3: writing zero to 0x1003:0 clears the error
        // history (CANopenNode `CO_Emergency.c OD_write_1003`). Any other
        // value is not accepted as a history edit; the live count is
        // restored so a bogus write cannot stick.
        if addr.index == 0x1003 && addr.subindex == 0 {
            if matches!(value, Value::Uint(0)) {
                self.emcy.clear_history(&mut self.od);
            } else {
                self.emcy.resync_od(&mut self.od);
            }
            return;
        }
        // §6.5 (BUGS.md P1-11): consumer heartbeat timeouts come from
        // OD 0x1016 (u32: nodeId<<16 | timeout_ms); any write re-syncs the
        // consumer configuration immediately.
        if addr.index == 0x1016 {
            self.resync_hb_consumers_from_od();
            return;
        }
        // CiA 301 / CANopenNode `OD_write_1017`: writing the producer
        // heartbeat time restarts the producer and sends the next heartbeat
        // immediately (`CO_NMT_Heartbeat.c:64-66,187-189`; BUGS.md B4). The
        // core is clockless, so mark the period expired for the next `poll`
        // and clear the boot anchor so a write before the first poll is not
        // swallowed.
        if addr.index == 0x1017 && addr.subindex == 0 {
            self.last_hb_ms = None;
            self.hb_arm_pending = false;
            return;
        }
        if !matches!(addr.index, 0x1010 | 0x1011) || addr.subindex == 0 {
            return;
        }
        let magic = match &value {
            Value::Uint(v) => Some(*v as u32),
            _ => return,
        };
        match magic {
            Some(m) if m == crate::storage::SAVE_MAGIC_U32 => {}
            Some(m) if m == crate::storage::LOAD_MAGIC_U32 => {}
            _ => return,
        }
        let Some(backend) = self.storage.clone() else {
            return;
        };
        crate::storage::handle_write(&backend, &mut self.od, &self.defaults, addr, &value);
    }

    /// The attached LSS slave, if any.
    #[must_use]
    pub const fn lss(&self) -> Option<&crate::lss::LssSlave> {
        self.lss.as_ref()
    }

    /// Node id.
    #[must_use]
    pub const fn node_id(&self) -> NodeId {
        self.node_id
    }

    /// Object dictionary access.
    #[must_use]
    pub const fn od(&self) -> &ObjectDictionary {
        &self.od
    }

    /// Mutable object dictionary access (application-owned cells).
    pub fn od_mut(&mut self) -> &mut ObjectDictionary {
        &mut self.od
    }

    /// Current NMT state.
    #[must_use]
    pub const fn nmt_state(&self) -> NmtState {
        self.nmt.state()
    }

    /// Leaves Initialising and returns the boot-up frame to transmit.
    pub fn boot(&mut self) -> Option<TxFrame> {
        // BUGS.md 八L17: a node without an assigned node-id serves only LSS
        // (CiA 305) — it must not emit a boot-up frame on the placeholder
        // node-1 heartbeat COB 0x701 (CANopenLinux refuses to enter normal
        // service while unconfigured).
        if self.unconfigured {
            return None;
        }
        if !self.nmt.boot() {
            return None;
        }
        // BUGS.md B5: the guarding toggle starts at 0 on boot/reset
        // (CANopenNode `CO_nodeGuardingSlave_init`); NMT state transitions do
        // not reset it.
        self.guarding_toggle = false;
        if let Err(e) = self.configure_pdos_from_od() {
            self.record_config_error(e);
        }
        // Pick up any pre-loaded OD 0x1016 consumer configuration (§6.5).
        self.resync_hb_consumers_from_od();
        // BUGS.md 八M4: the boot-up frame itself is the first heartbeat
        // (state 0x00), so the periodic producer must not fire an
        // immediate second frame — CANopenNode starts its heartbeat timer
        // only after boot (`CO_NMT_Heartbeat.c` `firstHBTime`). The core
        // has no clock, so instead of assuming `t=0` (wrong on a real
        // monotonic clock where `now ≫ period`) the first `poll` anchors
        // the start of the period at the observed `now_ms`; the first
        // periodic heartbeat then goes out one full period later.
        self.last_hb_ms = None;
        self.hb_arm_pending = true;
        // BUGS.md L6: the boot-up frame is a constant 1-byte payload, so
        // `new` cannot fail — return `.ok()` to keep the no-panic contract
        // instead of `.expect()`.
        TxFrame::new(self.node_id.heartbeat_cob_id(), &[0x00]).ok()
    }

    // --- Frame intake ---------------------------------------------------------

    /// Consumes one received frame addressed to this node, returning frames
    /// to transmit in order.
    #[must_use]
    pub fn on_frame(&mut self, cob_id: u16, data: &[u8]) -> Vec<TxFrame> {
        // LSS is served in every state; while unconfigured it is the ONLY
        // service (data COB-IDs are meaningless yet).
        if cob_id == crate::lss::LSS_MASTER_COB_ID {
            let mut response = None;
            let mut reset_requested = false;
            if let Some(lss) = &mut self.lss {
                // BUGS.md L3: LSS frames are always exactly 8 bytes on the
                // wire (CiA 305); zero-padding a short frame could turn an
                // abnormal/stray frame into a valid-looking command (e.g. a
                // fastscan probe) — reject anything shorter outright.
                if data.len() == 8 {
                    let mut req = [0u8; 8];
                    req.copy_from_slice(data);
                    response = lss.handle(&req);
                }
                reset_requested = lss.take_reset_requested();
            }
            let mut out = Vec::new();
            if reset_requested {
                // CiA 305 §5.4.1 / CANopenNode `CO_LSSslave.c:56-66,275-280`:
                // a freshly configured node switched back to Waiting runs an
                // NMT communication reset autonomously so the pending node-id
                // becomes active. The boot-up frame is returned to the
                // caller for transmission (BUGS.md 十M22).
                self.apply_lss_node_id();
                self.nmt.apply(NmtCommand::ResetCommunication);
                if let Some(boot) = self.communication_reset() {
                    out.push(boot);
                }
            }
            if let Some(resp) = response {
                if let Ok(frame) = TxFrame::new(crate::lss::LSS_SLAVE_COB_ID, &resp) {
                    out.push(frame);
                }
            }
            // BUGS.md M4: 0x7E5 is exclusively LSS. A frame on it that the
            // handler did not consume (no LSS attached, or a command it left
            // unanswered) must never fall through to the ordinary service
            // dispatch — the COB-ID is not a data service.
            return out;
        } else if self.unconfigured {
            return Vec::new();
        }
        let sdo_rx = self.node_id.sdo_rx_cob_id();
        let hb = self.node_id.heartbeat_cob_id();

        if cob_id == crate::nmt::NMT_COMMAND_COB_ID {
            // A reset may re-enter Initialising and re-boot — the boot-up
            // frame comes back through the same call (BUGS.md H2).
            return self.on_nmt(data).into_iter().collect();
        }
        if cob_id == sdo_rx
            && matches!(
                self.nmt_state(),
                NmtState::PreOperational | NmtState::Operational
            )
        {
            let mut out = Vec::new();
            // BUGS.md H2: SDO requests are always exactly 8 bytes on the
            // wire (CiA 301). Zero-padding shorter frames silently fed a
            // malformed expedited request to the server (no abort, no
            // error); drop them instead — same strictness the LSS path
            // already applies (it requires `len == 8`).
            let Some(req) = sdo_payload(data) else {
                return Vec::new();
            };
            for payload in self.sdo.handle(&mut self.od, &req) {
                if let Ok(frame) = TxFrame::new(self.node_id.sdo_tx_cob_id(), &payload) {
                    out.push(frame);
                }
                // Live-reconfiguration: a write into the PDO parameter range
                // re-runs slot assembly (docs/IMPLEMENTATION.md §7.3).
                for (addr, value) in self.sdo.take_last_written() {
                    if (0x1400..=0x1BFF).contains(&addr.index) {
                        if let Err(e) = self.configure_pdos_from_od() {
                            self.record_config_error(e);
                        }
                    }
                    self.on_parameter_write(addr, value);
                }
            }
            return out;
        }
        if cob_id == hb {
            // RTR-based guarding is handled via [`Self::on_remote_request`];
            // regular frames here are ignored.
            return Vec::new();
        }
        // Foreign heartbeats feed the heartbeat consumer (§10.3 / P1-11);
        // timestamps are applied when the host next polls (sans-I/O).
        // Only *supervised* nodes' frames are claimed here: a blanket
        // 0x700..=0x77F interception would silently swallow custom
        // RPDO/SYNC COB-IDs configured inside that range (`BUGS.md` P3,
        // cf. CANopenNode's per-entry rx filter and python-canopen's
        // per-node subscription).
        // BUGS.md 八L13: a heartbeat frame is exactly 1 byte on the wire
        // (CANopenNode `CO_HBconsumer.c:43` only latches DLC == 1); longer
        // frames are malformed and must not be consumed as heartbeats.
        if (0x700..=0x77F).contains(&cob_id) && data.len() == 1 {
            if let Ok(node) = NodeId::try_from((cob_id - 0x700) as u8) {
                if self.hb_consumer.supervises(node) {
                    let mut buf = [0u8; 8];
                    let n = data.len().min(8);
                    buf[..n].copy_from_slice(&data[..n]);
                    if self.hb_rx_queue.len() >= MAX_RX_QUEUE {
                        self.hb_rx_queue.pop_front();
                    }
                    self.hb_rx_queue.push_back((node, buf));
                    return Vec::new();
                }
            }
        }
        // Foreign EMCY frames (0x80+n, n≠self) feed the optional consumer
        // (`BUGS.md` M3): only when an application installed one via
        // [`Self::set_emcy_consumer`]. The interception is deliberately
        // **narrow** (`BUGS.md` H2): a COB-ID claimed by a configured RPDO
        // slot belongs to that RPDO, not to a loosely decoded EMCY — the
        // frame must fall through to the PDO matcher below. Only *unclaimed*
        // standard EMCY COBs (0x80+n) are consumed.
        // SYNC COB-ID（含 0x1005 自定义 0x81..=0xFF）必须排除，否则 EMCY 的
        // 宽松解码会吞掉 SYNC 帧，使其永远到不了 `on_sync`（BUGS.md P2-1）。
        // The standard EMCY COB-ID range is exactly 0x80..=0xFF: higher
        // identifiers (e.g. 0x181/0x281 TPDO1..4) must not be wrapped into a
        // bogus node number by `(cob - 0x80) as u8` and swallowed before the
        // PDO/SYNC/TIME dispatch below. CANopenNode `CO_Emergency.c:459` and
        // python-canopen (`node/remote.py:65`, per-node `0x80+id`
        // subscription) only accept the standard range.
        let sync = self.is_sync(cob_id);
        if let Some(consumer) = &mut self.emcy_consumer {
            if !sync && (0x80..=0xFF).contains(&cob_id) {
                let claimed_by_rpdo = self
                    .rpdos
                    .iter()
                    .any(|s| s.config.is_valid() && s.config.cob_id() == cob_id);
                if !claimed_by_rpdo {
                    if let Ok(node) = NodeId::try_from((cob_id - 0x80) as u8) {
                        if node != self.node_id {
                            if let Some(frame) = crate::emcy::EmcyFrame::decode(data) {
                                consumer.on_frame(node, frame);
                                return Vec::new();
                            }
                        }
                    }
                }
            }
        }
        if self.is_sync(cob_id) {
            return self.on_sync(data);
        }
        // TIME (0x100): queue the raw 6-byte stamp for the application
        // instead of letting it fall through to the RPDO matcher and vanish
        // (BUGS.md P3-21).
        if cob_id == crate::sync_time::TIME_COB_ID && data.len() == 6 {
            let mut buf = [0u8; 6];
            buf.copy_from_slice(data);
            if self.time_rx_queue.len() >= MAX_RX_QUEUE {
                self.time_rx_queue.pop_front();
            }
            self.time_rx_queue.push_back(buf);
            return Vec::new();
        }
        if let NmtState::Operational = self.nmt_state() {
            // Self-echo suppression (BUGS.md P3-22, python-canopen
            // `pdo/base.py:331-342` idea): a frame on a COB-ID this node
            // itself transmits as a TPDO is our own traffic looped back —
            // never apply it as RPDO data.
            if self
                .tpdos
                .iter()
                .any(|t| t.config.is_valid() && t.config.cob_id() == cob_id)
            {
                return Vec::new();
            }
            // Index-based matching so `raise_emergency` (which borrows all
            // of `self`) can run after the slot borrow ends.
            if let Some(idx) = self
                .rpdos
                .iter()
                .position(|s| s.config.is_valid() && s.config.cob_id() == cob_id)
            {
                let declared = self.rpdos[idx].mapping.byte_len();
                let mut out = Vec::new();
                if data.len() < declared {
                    // BUGS.md 十L5: CANopenNode `CO_PDO.c:420-465,717-726`
                    // rejects an under-long RPDO (DLC < dataLength) with
                    // CO_EMC_PDO_LENGTH (0x8210) and does not apply it; the
                    // old code fed it to `unpack` and only recorded a
                    // config error.
                    // BUGS.md B6: a short frame is not traffic either —
                    // CANopenNode only sets `CANrxNew` on an accepted frame
                    // (`CO_PDO.c:442-475`), so the event-timer deadline must
                    // not be re-armed here.
                    if let Some(frame) =
                        self.raise_emergency(0x8210, crate::emcy::err_reg::COMMUNICATION, [0; 5])
                    {
                        out.push(frame);
                    }
                    return out;
                }
                self.rpdos[idx].rx_seen = true;
                // BUGS.md 八L2: CANopenNode `CO_PDO.c:448-451,722-728`
                // accepts over-long RPDO frames but reports
                // CO_EMC_PDO_LENGTH_EXC (0x8220); canopen-rs applied the
                // frame silently with no trace at all.
                if self.rpdos[idx].tt.is_synchronous() {
                    let mut buf = [0u8; 8];
                    let n = data.len().min(8);
                    buf[..n].copy_from_slice(&data[..n]);
                    self.rpdos[idx].sync_pending = Some((buf, n));
                } else if let Err(e) = pdo::unpack(&self.rpdos[idx].mapping, &mut self.od, data) {
                    self.record_config_error(e);
                }
                if data.len() > declared {
                    if let Some(frame) =
                        self.raise_emergency(0x8220, crate::emcy::err_reg::COMMUNICATION, [0; 5])
                    {
                        out.push(frame);
                    }
                } else {
                    // BUGS.md 十H5: a correctly-sized frame clears the
                    // latched length condition — CANopenNode
                    // `CO_PDO.c:723-727` calls `CO_error(setError=false)`
                    // once `receiveError` returns to OK (and re-announces
                    // any remaining active EMCY).
                    if let Some(frame) = self.clear_emergency(0x8220) {
                        out.push(frame);
                    }
                    if let Some(frame) = self.clear_emergency(0x8210) {
                        out.push(frame);
                    }
                }
                return out;
            }
        }
        Vec::new()
    }

    /// Producer heartbeat time from OD 0x1017 (ms); 0 disables the producer.
    fn producer_heartbeat_ms(&self) -> u64 {
        match self.od.read(Address::new(0x1017, 0)) {
            Ok(Value::Uint(v)) => v,
            _ => 0,
        }
    }

    /// Handles a remote-request (RTR) frame: answers node-guarding polls on
    /// the heartbeat COB-ID with an alternating toggle bit, and — in
    /// Operational — services RTR-triggered TPDOs whose COB-ID bit30 is
    /// *clear* (RTR allowed, CiA 301; BUGS.md 八H1) — see §7.4.
    pub fn on_remote_request(&mut self, cob_id: u16) -> Option<TxFrame> {
        let hb = self.node_id.heartbeat_cob_id();
        // Node guarding and heartbeat are mutually exclusive error-control
        // protocols: CANopenNode passes `slaveDisable = 0x1017 > 0` into the
        // guarding slave, which then refuses to answer RTRs (`CANopen.c:1384`,
        // `CO_Node_Guarding.c:178-183`; BUGS.md B3).
        if cob_id == hb
            && self.nmt.state() != NmtState::Initialising
            && self.producer_heartbeat_ms() == 0
        {
            // First response carries toggle 0 (CiA 301 §7.3.1).
            let payload = crate::nmt::encode_node_guard(self.nmt.state(), self.guarding_toggle);
            self.guarding_toggle = !self.guarding_toggle;
            // BUGS.md L6: a constant 1-byte payload cannot fail `new`.
            return TxFrame::new(hb, &payload).ok();
        }
        if self.nmt.state() == NmtState::Operational {
            let mut pack_error: Option<crate::types::Error> = None;
            for slot in &mut self.tpdos {
                if !slot.config.is_valid() || !slot.config.rtr_allowed() {
                    continue;
                }
                if slot.config.cob_id() != cob_id {
                    continue;
                }
                if matches!(
                    slot.tt,
                    TransmissionType::RtrOnlySync | TransmissionType::RtrOnlyEvent
                ) {
                    if slot.tt == TransmissionType::RtrOnlyEvent {
                        // 类型 253（RtrOnlyEvent）用当前 OD 值，符合规范。
                        match pack_tpdo(slot, &self.od) {
                            Ok(frame) => return frame,
                            Err(e) => {
                                pack_error = Some(e);
                                break;
                            }
                        }
                    } else {
                        // BUGS.md M4: 类型 252（RtrOnlySync）的 RTR 应答必须
                        // 回放"最后 SYNC"采样快照，不能用当前实时 OD 值。
                        if !slot.sync_received {
                            continue;
                        }
                        match &slot.sync_sample {
                            Some((buf, len)) => {
                                return TxFrame::new(slot.cob_id(), &buf[..*len]).ok();
                            }
                            None => continue,
                        }
                    }
                }
            }
            if let Some(e) = pack_error {
                self.record_config_error(e);
            }
        }
        None
    }

    /// Re-initialises the communication profile and re-boots — the shared
    /// body of an NMT reset (the caller has already set the NMT state to
    /// Initialising). Returns the boot-up frame like a power-on sequence.
    fn communication_reset(&mut self) -> Option<TxFrame> {
        self.sdo.reset();
        self.tpdos.clear();
        self.rpdos.clear();
        self.last_hb_ms = None;
        self.pending_emcy.clear();
        self.last_emcy_ms = None;
        self.rpdo_timeouts.clear();
        // Stale heartbeats/events from the previous epoch must not
        // surface in the new one (BUGS.md P3-16).
        self.hb_rx_queue.clear();
        self.pending_hb_events.clear();
        self.time_rx_queue.clear();
        // BUGS.md 十M2: a communication reset re-initialises the
        // heartbeat consumer (CANopenNode re-runs
        // `CO_HBconsumer_init`); keeping `phase`/`last_seen` from
        // the previous epoch made the first post-reset `poll`
        // report an immediate false `Lost`.
        self.hb_consumer = HeartbeatConsumer::new();
        // Communication profile re-initialisation: drop latched EMCY
        // conditions and the node-guarding toggle (the EM module is
        // re-initialised in CANopenNode's reset sequence).
        self.emcy = EmcyProducer::new();
        self.emcy.resync_od(&mut self.od);
        // Remote EMCY conditions tracked by the consumer belong to
        // the previous epoch as well (CANopenNode re-initialises the
        // EM consumer on a communication reset).
        if let Some(consumer) = &mut self.emcy_consumer {
            consumer.reset();
        }
        self.guarding_toggle = false;
        // Power-on semantics: re-apply persisted parameters.
        if let Some(backend) = &self.storage {
            let _ = backend.load(&mut self.od);
        }
        // BUGS.md M3: a reset abandons any in-progress LSS session
        // (fastscan/switch/bit-timing) and returns the slave to
        // Waiting; pending values survive for the application to
        // consume (adopt_pending / take_pending_bit_timing /
        // take_store_requested).
        if let Some(lss) = &mut self.lss {
            lss.reset();
        }
        // BUGS.md H2: the reset leaves Initialising and re-boots —
        // PDOs are rebuilt, heartbeat consumers re-synced, and the
        // boot-up frame is handed back, exactly like a power-on
        // sequence. Without this the node would stay in
        // Initialising forever (Start is rejected there) and the
        // master could never bring it back to Operational.
        self.boot()
    }

    /// Handles an NMT command addressed to this node. Returns the boot-up
    /// frame when the command reset the node (re-entering Initialising) —
    /// the reset restarts the communication profile and re-boots, so the
    /// caller must transmit it like a power-on boot-up (BUGS.md H2).
    fn on_nmt(&mut self, data: &[u8]) -> Option<TxFrame> {
        let (command, target) = decode_command(data)?;
        let me = self.node_id.get();
        if target != 0 && target != me {
            return None;
        }
        match command {
            // Both reset flavours share this branch by design: the core has
            // no notion of application vs communication parameters beyond
            // what storage re-applies; distinguishing them is the host's job
            // (`BUGS.md` R10-P3, documented).
            NmtCommand::ResetNode | NmtCommand::ResetCommunication => {
                self.nmt.apply(command);
                self.communication_reset()
            }
            _ => {
                let changed = self.nmt.apply(command);
                // BUGS.md B3-2: 进入 Operational 时，对在 Op 前已锁存（因
                // `raise` 幂等而从未出帧）的 EMCY 条件补发一次，使上位机
                // 能感知既有故障。
                if changed && self.nmt.state() == NmtState::Operational {
                    for payload in self.emcy.reannounce_active() {
                        self.pending_emcy.push_back(payload);
                    }
                }
                // BUGS.md #5: an NMT state transition emits a heartbeat
                // immediately (CANopenNode CO_NMT_Heartbeat.c:184-204), so
                // the next `poll` fires one instead of waiting a full 0x1017
                // period.
                if changed {
                    self.last_hb_ms = None;
                    // BUGS.md 十M1: a state transition must fire a heartbeat
                    // immediately (CANopenNode `CO_NMT_Heartbeat.c:184-204`);
                    // the boot-up epoch anchor must not swallow it when the
                    // transition happens before the first `poll`.
                    self.hb_arm_pending = false;
                    // BUGS.md B5: the node-guarding toggle keeps alternating
                    // across NMT state changes — CANopenNode resets it only
                    // when guarding is disabled (`CO_Node_Guarding.c:178-183`,
                    // `slaveDisable`), and the reference master enforces
                    // strict alternation without resyncing on a transition
                    // (`:255-259`). Resetting here made a poll spanning a
                    // Stop/Start cycle look like a missed toggle. The toggle
                    // is still reset by `boot()` and on communication reset
                    // above (power-on / reset semantics).
                }
                // Leaving Operational resets the synchronous PDO phase state
                // (CANopenNode `CO_PDO.c:1351` resets syncCounter on the NMT
                // transition): cyclic dividers and SYNC start-value locks
                // start over at the next entry.
                if self.nmt.state() != NmtState::Operational {
                    for slot in &mut self.tpdos {
                        // Re-seed on re-entry (BUGS.md #27): the next
                        // Operational session restarts the cyclic phase at
                        // the period midpoint, like CANopenNode resetting
                        // `syncCounter` on the NMT transition.
                        slot.syncs_seen = match slot.tt {
                            TransmissionType::SyncCyclic(n) => {
                                let n = u32::from(n.max(1));
                                n - (n / 2 + 1)
                            }
                            _ => 0,
                        };
                        slot.phase_locked = false;
                        slot.pending_event = false;
                        // BUGS.md M2: the new session starts with no SYNC
                        // sample — RtrOnlySync answers resume only after the
                        // next SYNC re-arms the flag.
                        slot.sync_received = false;
                        slot.sync_sample = None;
                    }
                    // BUGS.md C3: RPDO receive state belongs to the
                    // Operational session too. CANopenNode `CO_PDO.c:846-862`
                    // clears `CANrxNew` and the timeout timer for both
                    // directions when the node leaves Operational; without
                    // this a frame buffered before Stop is applied at the
                    // first SYNC after Start, and a pre-Stop timestamp can
                    // report an immediate timeout.
                    for slot in &mut self.rpdos {
                        slot.sync_pending = None;
                        slot.rx_seen = false;
                        slot.last_rx_ms = None;
                        slot.reported = false;
                    }
                }
                None
            }
        }
    }

    fn is_sync(&self, cob_id: u16) -> bool {
        // CiA 301 默认 SYNC COB-ID 为 0x80；OD 缺 0x1005 时使用该默认
        // 回退（BUGS.md B3-1）。COB-ID SYNC 位布局（CANopenNode
        // `CO_SYNC.c:87-104`）：bit31 必须清零（有效）；**bit30 是 gen
        // 标志（本设备为 SYNC 生产者），不影响消费/过滤**——此前误当作
        // "29 位扩展帧" 拒绝，使声明生产者的节点对 SYNC 完全失明
        // （BUGS.md 八M12）；bits 29..11 为保留位必须为零；低 11 位为
        // CAN-ID。
        let raw = match self.od.read(Address::new(0x1005, 0)) {
            Ok(Value::Uint(v)) => v,
            _ => 0x80,
        };
        raw & 0x8000_0000 == 0 && raw & 0x3FFF_F800 == 0 && (raw & 0x7FF) as u16 == cob_id
    }

    /// SYNC consumer: advances synchronous TPDOs.
    fn on_sync(&mut self, data: &[u8]) -> Vec<TxFrame> {
        // BUGS.md 十L4: the width check and its 0x8240 report also apply in
        // PreOperational (CANopenNode `CO_SYNC.c:386-392` reports in
        // PreOp and Op); only *driving* PDOs is Operational-only.
        if !matches!(
            self.nmt.state(),
            NmtState::PreOperational | NmtState::Operational
        ) {
            return Vec::new();
        }
        // BUGS.md 八M11: the node's own 0x1019 counter-overflow value dictates
        // the legal SYNC payload width (CANopenNode `CO_SYNC.c:37-51`):
        // 0x1019 == 0 accepts only a bare SYNC, otherwise exactly one counter
        // byte. A width mismatch is not a SYNC for this node — it must not
        // release the buffered RPDO cycle nor drive TPDOs, and the condition
        // is reported like CANopenNode's `CO_EMC_SYNC_DATA_LENGTH` (0x8240).
        let counter_overflow = match self.od.read(Address::new(0x1019, 0)) {
            Ok(Value::Uint(v)) => v as u8,
            _ => 0,
        };
        let length_ok = if counter_overflow == 0 {
            data.is_empty()
        } else {
            data.len() == 1
        };
        if !length_ok {
            if let Some(frame) =
                self.raise_emergency(0x8240, crate::emcy::err_reg::COMMUNICATION, [0; 5])
            {
                return vec![frame];
            }
            return Vec::new();
        }
        if self.nmt.state() != NmtState::Operational {
            return Vec::new();
        }
        // Payload validation FIRST (BUGS.md P3-2, see git history): a frame whose
        // COB-ID matches SYNC but whose payload is malformed (>1 byte) is
        // not a SYNC and must not release the buffered RPDO cycle either.
        let counter = match crate::sync_time::decode_sync(data) {
            Some(c) => c,
            None => return Vec::new(),
        };
        // Synchronous RPDO double buffering (BUGS.md P2-4): data received
        // during the previous cycle becomes visible exactly at this SYNC
        // boundary — CANopenNode `CO_PDO.c:717-760` applies the buffered
        // frame when `syncWas` fires.
        let mut config_errors = Vec::new();
        for slot in &mut self.rpdos {
            if let Some((buf, len)) = slot.sync_pending.take() {
                // Field-disjoint borrows: `od` is mutated while `rpdos` is
                // iterated; errors are recorded after the loop ends.
                if let Err(e) = pdo::unpack(&slot.mapping, &mut self.od, &buf[..len]) {
                    config_errors.push(e);
                }
            }
        }
        for e in config_errors {
            self.record_config_error(e);
        }
        // CiA 301 / CANopenNode `CO_PDO.c`: sync-start phase-locking only
        // applies when BOTH a start value and a global counter overflow
        // (0x1019) are configured (`BUGS.md` P2-5); `counter_overflow` was
        // read and validated at the top of this function.
        // BUGS.md M2: any received SYNC arms the type-252 (RtrOnlySync)
        // TPDOs of this session — their RTR payload is the last SYNC
        // sample, so before the first SYNC there is nothing valid to
        // answer with. Applied to every valid slot (synchronous ones get
        // it too; the flag only gates RtrOnlySync responses).
        for slot in &mut self.tpdos {
            if !slot.config.is_valid() {
                continue;
            }
            slot.sync_received = true;
            // BUGS.md M4: 类型 252（RtrOnlySync）PDO 的 RTR 应答须在"最后
            // SYNC"采样——在此处（每个有效 SYNC）对该槽打包一次并缓存快照，
            // RTR 时回放该快照，而非打包当前实时 OD 值。
            if slot.tt == TransmissionType::RtrOnlySync {
                slot.sync_sample = match pack_tpdo(slot, &self.od) {
                    Ok(Some(frame)) => {
                        let mut buf = [0u8; 8];
                        let n = frame.payload().len().min(8);
                        buf[..n].copy_from_slice(&frame.payload()[..n]);
                        Some((buf, n))
                    }
                    _ => None,
                };
            }
        }
        let mut out = Vec::new();
        let mut tpdo_errors = Vec::new();
        for slot in &mut self.tpdos {
            if !slot.config.is_valid() || !slot.tt.is_synchronous() {
                continue;
            }
            // Sync start value (S14): wait for the matching global counter
            // once, then keep the phase lock for the whole Operational
            // session (CANopenNode treats the wait as a one-shot state).
            // Adjudicated limitation (`BUGS.md` H2): when 0x1019 ≠ 0 but the
            // incoming SYNC carries no counter byte, the phase can never be
            // locked and the sync-start TPDO stays silent — the counterless
            // SYNC is a producer misconfiguration (0x1019 is the producer's
            // own counter-overflow declaration), so silence is deliberate.
            let mut lock_now = false;
            // BUGS.md 十M3: the sync-start phase lock belongs to cyclic
            // transmission types only (CANopenNode `CO_PDO.c:1305-1329`
            // keeps `syncStartValue` inside the cyclic branch); applying it
            // to SyncAcyclic (tt=0) produced unsolicited/duplicated frames.
            if counter_overflow != 0 && matches!(slot.tt, TransmissionType::SyncCyclic(_)) {
                if let Some(start) = nonzero(slot.config.sync_start_value) {
                    match counter {
                        Some(global) if global == start && !slot.phase_locked => {
                            slot.phase_locked = true;
                            // BUGS.md 八L3: CANopenNode `CO_PDO.c:1325-1329`
                            // sends the sync-start TPDO *immediately* on the
                            // matching SYNC (then divides normally) — the
                            // old code waited `n - seed` further SYNCs.
                            lock_now = true;
                        }
                        Some(_) => {}
                        // cannot phase-lock without counters
                        None => {}
                    }
                    if !slot.phase_locked {
                        continue;
                    }
                }
            }
            let due = if lock_now {
                // Reset the divider phase so the next periodic frame lands
                // one full period after the immediate one (CANopenNode sets
                // `syncCounter = transmissionType` right after the send).
                slot.syncs_seen = 0;
                true
            } else {
                match slot.tt {
                    TransmissionType::SyncAcyclic => {
                        let p = slot.pending_event;
                        slot.pending_event = false;
                        p
                    }
                    TransmissionType::SyncCyclic(n) => {
                        slot.syncs_seen = slot.syncs_seen.wrapping_add(1);
                        if u32::from(n.max(1)) <= slot.syncs_seen {
                            slot.syncs_seen = 0;
                            true
                        } else {
                            false
                        }
                    }
                    _ => false,
                }
            };
            if due {
                match pack_tpdo(slot, &self.od) {
                    Ok(Some(frame)) => out.push(frame),
                    Ok(None) => {}
                    // BUGS.md M1: a due TPDO whose mapping cannot be packed
                    // must surface as a config error instead of silently
                    // dropping the frame (symmetric with the RPDO unpack
                    // path above).
                    Err(e) => tpdo_errors.push(e),
                }
            }
        }
        for e in tpdo_errors {
            self.record_config_error(e);
        }
        out
    }

    // --- Producers --------------------------------------------------------------

    /// Periodic maintenance: heartbeat production and event-driven TPDO
    /// timers (event timer + inhibit time). Call regularly with monotonic ms.
    pub fn poll(&mut self, now_ms: u64) -> Vec<TxFrame> {
        let mut out = Vec::new();

        // Heartbeat consumption (§10.3, BUGS.md P1-11): frames received
        // since the last poll get their timestamps here; timeout events
        // surface via [`Node::take_hb_events`].
        while let Some((node, data)) = self.hb_rx_queue.pop_front() {
            for event in self.hb_consumer.on_heartbeat(node, &data, now_ms) {
                self.push_hb_event(event);
            }
        }
        for event in self.hb_consumer.poll(now_ms) {
            self.push_hb_event(event);
        }

        // EMCY inhibit release (§8.2 / BUGS.md P2-6): payloads queued by
        // `raise_emergency`/`clear_emergency` go out once the 0x1015 window
        // since the previous EMCY frame has elapsed. Frames only leave in
        // PreOperational/Operational (BUGS.md #8); the register byte is
        // composed at send time so it reflects conditions raised/cleared
        // while the frame was queued (BUGS.md #9).
        if matches!(
            self.nmt.state(),
            NmtState::PreOperational | NmtState::Operational
        ) && !self.pending_emcy.is_empty()
        {
            let window_ms = self.emcy_inhibit_window_ms();
            while let Some(front) = self.pending_emcy.front() {
                let due = self
                    .last_emcy_ms
                    .map_or(true, |t| now_ms.saturating_sub(t) >= window_ms);
                if !due {
                    break;
                }
                let mut payload = *front;
                payload[2] = self.emcy.error_register();
                self.pending_emcy.pop_front();
                self.last_emcy_ms = Some(now_ms);
                if let Some(cob) = self.emcy_cob_id() {
                    if let Ok(f) = TxFrame::new(cob, &payload) {
                        out.push(f);
                    }
                }
            }
        }

        // Heartbeat producer (any post-boot state). During Initialising the
        // periodic producer stays silent: boot-up is sent exactly once by
        // `boot()` (CANopenNode CO_NMT_Heartbeat.c sends 0x00 once on the
        // NMT initialise→pre-operational edge, never on the timer).
        let period_ms = self.producer_heartbeat_ms();
        if period_ms > 0 && self.nmt.state() != NmtState::Initialising {
            // BUGS.md 八M4: anchor the heartbeat epoch at the first poll
            // after boot-up/reset (no clock in core). The first periodic
            // frame therefore goes out one full period after this poll —
            // never back-to-back with the boot-up frame.
            if self.hb_arm_pending {
                self.hb_arm_pending = false;
                self.last_hb_ms = Some(now_ms);
            }
            let due = self
                .last_hb_ms
                .map_or(true, |t| now_ms.saturating_sub(t) >= period_ms);
            if due {
                self.last_hb_ms = Some(now_ms);
                let payload = encode_heartbeat(self.nmt.state());
                if let Ok(f) = TxFrame::new(self.node_id.heartbeat_cob_id(), &payload) {
                    out.push(f);
                }
            }
        }

        // Event-driven TPDOs and RPDO deadman monitoring (Operational only).
        if self.nmt.state() == NmtState::Operational {
            // RPDO event timer as dead line (§7.4/§7.5, BUGS.md P2-7): a
            // slot with event_timer_ms > 0 must receive traffic within each
            // window once armed by its first frame. The core is time-blind
            // in `on_frame`, so the reception time is stamped at the first
            // poll observing traffic — resolution is one poll interval.
            for slot in &mut self.rpdos {
                if !slot.config.is_valid() || slot.config.event_timer_ms == 0 {
                    continue;
                }
                if slot.rx_seen {
                    // Traffic in this window: (re)arm, stamp the time and
                    // re-enable reporting for the next silence.
                    slot.last_rx_ms = Some(now_ms);
                    slot.rx_seen = false;
                    slot.reported = false;
                    continue;
                }
                let Some(last_rx_ms) = slot.last_rx_ms else {
                    continue; // never received anything: not armed yet
                };
                if now_ms.saturating_sub(last_rx_ms) < u64::from(slot.config.event_timer_ms) {
                    continue; // window still open
                }
                if !slot.reported {
                    slot.reported = true;
                    // Field-disjoint borrow: `rpdo_timeouts` is bounded with
                    // the same drop-oldest policy as the input queues
                    // (BUGS.md 十M8).
                    if self.rpdo_timeouts.len() >= MAX_RX_QUEUE {
                        self.rpdo_timeouts.pop_front();
                    }
                    self.rpdo_timeouts
                        .push_back(Address::new(slot.comm_index, 0));
                }
            }

            // flagsPDO trigger source (BUGS.md P3-11): an OD write to any
            // mapped address latches the event-driven slot, exactly like an
            // application `request_tpdo` call would.
            let dirty = self.od.take_dirty();
            if !dirty.is_empty() {
                for slot in &mut self.tpdos {
                    if !slot.config.is_valid() {
                        continue;
                    }
                    // BUGS.md #6: tt=0 (SyncAcyclic) also latches on a mapped
                    // OD write — the payload goes out at the next SYNC
                    // (CANopenNode's flagPDO covers tt==0 || tt>=254).
                    if !matches!(
                        slot.tt,
                        TransmissionType::EventDriven(_) | TransmissionType::SyncAcyclic
                    ) {
                        continue;
                    }
                    if slot
                        .mapping
                        .entries()
                        .iter()
                        .any(|m| dirty.contains(&m.address))
                    {
                        slot.pending_event = true;
                    }
                }
            }

            let mut tpdo_errors = Vec::new();
            for slot in &mut self.tpdos {
                if !slot.config.is_valid() {
                    continue;
                }
                let TransmissionType::EventDriven(_) = slot.tt else {
                    continue;
                };
                let timer_due = slot.config.event_timer_ms > 0
                    && slot.last_tx_ms.map_or(true, |t| {
                        now_ms.saturating_sub(t) >= u64::from(slot.config.event_timer_ms)
                    });
                if !slot.pending_event && !timer_due {
                    continue;
                }
                // Inhibit window since the last transmission.
                if let Some(t) = slot.last_tx_ms {
                    if now_ms.saturating_sub(t) < slot.inhibit_ms() {
                        continue; // window open later; event stays pending
                    }
                }
                // BUGS.md L5: 仅在打包**成功**后才清除触发标志与更新发送时间，
                // 否则一次失败的 pack 会丢失本次触发（pending_event 被提前清零、
                // 事件静默丢失，与 python-canopen 行为不符）。
                match pack_tpdo(slot, &self.od) {
                    Ok(Some(frame)) => {
                        slot.pending_event = false;
                        slot.last_tx_ms = Some(now_ms);
                        out.push(frame);
                    }
                    Ok(None) => {}
                    // BUGS.md M1: an event-driven TPDO whose mapping cannot
                    // be packed must surface as a config error, not silently
                    // drop the frame (symmetric with the RPDO unpack path).
                    Err(e) => tpdo_errors.push(e),
                }
            }
            for e in tpdo_errors {
                self.record_config_error(e);
            }
        }
        out
    }

    /// Requests transmission of an event-driven TPDO (numbered from 1),
    /// e.g. after application data changed. The frame goes out on the next
    /// [`Node::poll`] outside its inhibit window. A synchronous slot with
    /// transmission type 0 (SyncAcyclic) also honours the request — the
    /// payload is latched and emitted at the next SYNC.
    pub fn request_tpdo(&mut self, number: u8) -> bool {
        // PDO number n maps to comm record 0x1800 + n-1 (BUGS.md P3-19:
        // positional nth-valid-slot indexing mis-addressed requests when an
        // earlier slot was invalid).
        let Some(comm_index) = u16::from(number)
            .checked_sub(1)
            .and_then(|n| pdo::TPDO_COMM_BASE.checked_add(n))
        else {
            return false;
        };
        match self
            .tpdos
            .iter_mut()
            .find(|s| s.comm_index == comm_index && s.config.is_valid())
        {
            // Synchronous-acyclic (tt=0) requests are latched and consumed by
            // the next SYNC (`on_sync`); CANopenNode `CO_PDO.c:1305-1309`
            // drives tt=0 the same way (BUGS.md P2-3).
            Some(slot)
                if matches!(
                    slot.tt,
                    TransmissionType::EventDriven(_) | TransmissionType::SyncAcyclic
                ) =>
            {
                slot.pending_event = true;
                true
            }
            _ => false,
        }
    }

    /// EMCY producer COB-ID, read from OD 0x1014 honouring the CiA 301
    /// bit31 "bit invalid" flag and the "CAN-ID 0 disables production" rule
    /// (CANopenNode `CO_Emergency.c:377-378`:
    /// `producerEnabled = (bit31 == 0) && (producerCanId != 0)`).
    /// `None` when EMCY production is disabled; falls back to
    /// `0x80 + node-id` when the entry is missing.
    fn emcy_cob_id(&self) -> Option<u16> {
        const INVALID: u32 = 1 << 31;
        match self.od.read(Address::new(0x1014, 0)) {
            Ok(Value::Uint(v)) => {
                let v = v as u32;
                let cob = (v & 0x7FF) as u16;
                if v & INVALID != 0 || cob == 0 {
                    return None;
                }
                Some(cob)
            }
            _ => Some(self.node_id.emcy_cob_id()),
        }
    }

    /// Inhibit window for EMCY production from OD 0x1015 (unit 100 µs),
    /// rounded up to whole ms at the API's granularity; `0` disables
    /// inhibition (docs/IMPLEMENTATION.md §8.2, `BUGS.md` P2-6).
    fn emcy_inhibit_window_ms(&self) -> u64 {
        match self.od.read(Address::new(0x1015, 0)) {
            Ok(Value::Uint(v)) => u64::from(v as u16).div_ceil(10),
            _ => 0,
        }
    }

    /// Raises an emergency; returns the EMCY frame when the code was newly
    /// activated (idempotent otherwise). EMCY is not produced while the node
    /// is outside PreOperational/Operational (CiA 301 §7.2.7, CANopenNode
    /// `CO_Emergency.c:557`); the condition is still recorded in
    /// 0x1001/0x1003 so the error state stays authoritative.
    ///
    /// With OD 0x1015 configured the frame is deferred: it is queued and
    /// released by [`Node::poll`] once the inhibit window since the previous
    /// EMCY transmission has elapsed (returns `None` in that case).
    pub fn raise_emergency(
        &mut self,
        code: u16,
        register_bits: u8,
        info: [u8; 5],
    ) -> Option<TxFrame> {
        let payload = self.emcy.raise(&mut self.od, code, register_bits, info)?;
        let cob = self.emcy_cob_id()?;
        // EMCY frames only leave in PreOperational/Operational (BUGS.md #8,
        // CANopenNode `CO_Emergency.c:557`); Stopped/Initialising still
        // record the condition via `emcy.raise` above.
        if !matches!(
            self.nmt.state(),
            NmtState::PreOperational | NmtState::Operational
        ) {
            return None;
        }
        if self.emcy_inhibit_window_ms() == 0 {
            // BUGS.md L6: EMCY payloads are constant 8 bytes — `new` cannot
            // fail; keep the no-panic contract with `.ok()`.
            return TxFrame::new(cob, &payload).ok();
        }
        // Inhibited: queue for release in `poll` (P2-6).
        self.pending_emcy.push_back(payload);
        None
    }

    /// Clears an emergency code; returns the reset/no-error frame when it
    /// was active. Suppressed during Initialising like
    /// [`Self::raise_emergency`]; subject to the 0x1015 inhibit window the
    /// same way (queued for [`Node::poll`]).
    pub fn clear_emergency(&mut self, code: u16) -> Option<TxFrame> {
        let payload = self.emcy.clear(&mut self.od, code)?;
        let cob = self.emcy_cob_id()?;
        // Same frame gate as `raise_emergency` (BUGS.md #8): the clear is
        // recorded in 0x1001/0x1003, the reset frame only goes out in
        // PreOperational/Operational.
        if !matches!(
            self.nmt.state(),
            NmtState::PreOperational | NmtState::Operational
        ) {
            return None;
        }
        if self.emcy_inhibit_window_ms() == 0 {
            // BUGS.md L6: constant 8-byte payload, `new` cannot fail.
            return TxFrame::new(cob, &payload).ok();
        }
        self.pending_emcy.push_back(payload);
        None
    }

    /// Returns and clears communication addresses (0x1400+n) of RPDO slots
    /// whose event timer expired without reception (`BUGS.md` P2-7). Each
    /// silent window reports a slot once until traffic resumes.
    pub fn take_rpdo_timeouts(&mut self) -> Vec<Address> {
        self.rpdo_timeouts.drain(..).collect()
    }

    /// Rebuilds all PDO slots from the communication (0x1400+/0x1800+) and
    /// mapping (0x1600+/0x1A00+) records present in the dictionary. Slots
    /// whose COB-ID is marked invalid are skipped.
    ///
    /// Per-slot problems (missing targets, unmappable objects, malformed
    /// mapping values) do not abort the rebuild: the offending slot is
    /// dropped, its error is recorded for [`Self::take_config_errors`], and
    /// every other slot keeps working.
    ///
    /// # Errors
    /// Currently infallible; the signature is kept for forward compatibility.
    pub fn configure_pdos_from_od(&mut self) -> Result<()> {
        self.tpdos.clear();
        self.rpdos.clear();
        // Per-slot problems are recorded in `config_errors` (see
        // [`Self::take_config_errors`]) and the remaining slots keep
        // working — a partially misconfigured dictionary must not disable
        // the whole PDO layer.
        for comm_index in 0x1400..=0x15FF {
            match self.build_rpdo(comm_index) {
                Ok(Some(slot)) => self.rpdos.push(slot),
                Ok(None) => {}
                // Route through the bounded ring (BUGS.md P3-1): a raw push
                // here bypassed the MAX_CONFIG_ERRORS cap.
                Err(e) => self.record_config_error(e),
            }
        }
        for comm_index in 0x1800..=0x19FF {
            match self.build_tpdo(comm_index) {
                Ok(Some(slot)) => self.tpdos.push(slot),
                Ok(None) => {}
                Err(e) => self.record_config_error(e),
            }
        }
        Ok(())
    }

    fn build_rpdo(&self, comm_index: u16) -> Result<Option<RpdoSlot>> {
        // RPDO direction default: missing sub 2 means synchronous (CiA 301
        // default 0), so a minimal RPDO record applies at SYNC boundaries,
        // not immediately (BUGS.md R10-P2-12).
        let cfg = match PdoConfig::from_od(&self.od, comm_index, 0) {
            Ok(cfg) => cfg,
            // No communication record at all: the slot simply isn't
            // configured — silent (BUGS.md #28).
            Err(Error::ObjectDoesNotExist { .. }) => return Ok(None),
            // Present but malformed: a configuration error, surfaced via
            // `take_config_errors` instead of silently skipped (BUGS.md #28).
            Err(e) => return Err(e),
        };
        if !cfg.is_valid() {
            return Ok(None); // COB-ID marked invalid: slot intentionally off
        }
        if cfg.cob_id() == 0 {
            // COB 0x000 is the NMT command channel — never a legal PDO
            // identifier. CANopenNode treats `valid && CAN_ID==0` as
            // invalid at runtime (`CO_PDO.c:625`); an OD carrying such a
            // COB-ID is a configuration error, surfaced via
            // `take_config_errors` (BUGS.md 八M1).
            return Err(Error::Other("PDO COB-ID 0x000 is not a valid identifier"));
        }
        let map_index = comm_index - pdo::RPDO_COMM_BASE + pdo::RPDO_MAP_BASE;
        // Malformed records and validation failures are *errors* — recorded
        // by the caller so a broken slot never vanishes without a trace
        // (BUGS.md m17).
        let mapping = PdoMapping::from_od_record(&self.od, map_index)?;
        if mapping.total_bits() == 0 {
            // CiA 301 §7.4 / CANopenNode runtime (`CO_PDO.c:623-627`): a
            // PDO whose COB-ID is *valid* but whose mapping is empty can
            // never carry data — CANopenNode forces `valid=false` and
            // reports an error. Surface it here instead of silently
            // dropping the slot (BUGS.md 八M2).
            return Err(Error::Other(
                "RPDO is enabled but its mapping record has zero entries",
            ));
        }
        mapping.validate_against(&self.od, true)?;
        // `?` (not `unwrap_or`): an invalid transmission type in the record
        // is a configuration error, surfaced via `take_config_errors` rather
        // than silently downgraded to synchronous (BUGS.md R10-P2-12).
        let tt = cfg.transmission_type()?;
        if matches!(
            tt,
            pdo::TransmissionType::RtrOnlySync | pdo::TransmissionType::RtrOnlyEvent
        ) {
            // BUGS.md 八L1: transmission types 252/253 are TPDO-only
            // ("RTR only"); CANopenNode rejects 241..253 for RPDO
            // (`CO_PDO.c:542-547`, `DS301_profile.md:543`). Turn the
            // receive slot's misuse into a visible configuration error.
            return Err(Error::Other(
                "RPDO transmission type 252/253 is not allowed",
            ));
        }
        Ok(Some(RpdoSlot {
            comm_index,
            tt,
            sync_pending: None,
            config: cfg,
            mapping,
            rx_seen: false,
            last_rx_ms: None,
            reported: false,
        }))
    }

    fn build_tpdo(&self, comm_index: u16) -> Result<Option<TpdoSlot>> {
        // TPDO direction default: missing sub 2 means event-driven (254).
        let cfg = match PdoConfig::from_od(&self.od, comm_index, 254) {
            Ok(cfg) => cfg,
            // No communication record at all: the slot simply isn't
            // configured — silent (BUGS.md #28).
            Err(Error::ObjectDoesNotExist { .. }) => return Ok(None),
            // Present but malformed: a configuration error, surfaced via
            // `take_config_errors` instead of silently skipped (BUGS.md #28).
            Err(e) => return Err(e),
        };
        if !cfg.is_valid() {
            return Ok(None);
        }
        if cfg.cob_id() == 0 {
            // COB 0x000 is the NMT command channel — never a legal PDO
            // identifier (BUGS.md 八M1, CANopenNode `CO_PDO.c:625`).
            return Err(Error::Other("PDO COB-ID 0x000 is not a valid identifier"));
        }
        let tt = cfg.transmission_type()?;
        let map_index = comm_index - pdo::TPDO_COMM_BASE + pdo::TPDO_MAP_BASE;
        let mapping = PdoMapping::from_od_record(&self.od, map_index)?;
        if mapping.total_bits() == 0 {
            // Symmetric with `build_rpdo` (BUGS.md 八M2): an enabled TPDO
            // with an empty mapping is a configuration error, not a silent
            // no-op.
            return Err(Error::Other(
                "TPDO is enabled but its mapping record has zero entries",
            ));
        }
        mapping.validate_against(&self.od, false)?;
        // BUGS.md #27: seed the cyclic divider so the first SyncCyclic
        // frame goes out at the middle of the first period (`n/2 + 1`
        // SYNCs), exactly like CANopenNode `CO_PDO.c:1315-1321` — an
        // unseeded counter starting at 0 would send the first frame only
        // after a full period (n SYNCs).
        let syncs_seen = match tt {
            TransmissionType::SyncCyclic(n) => {
                let n = u32::from(n.max(1));
                n - (n / 2 + 1)
            }
            _ => 0,
        };
        Ok(Some(TpdoSlot {
            comm_index,
            config: cfg,
            mapping,
            tt,
            syncs_seen,
            last_tx_ms: None,
            pending_event: false,
            phase_locked: false,
            sync_received: false,
            sync_sample: None,
        }))
    }
}

/// SDO request payload: `None` when the frame is not exactly 8 bytes —
/// malformed/short frames are dropped instead of silently zero-padded
/// (`BUGS.md` H2, matching the strict `len == 8` the LSS path requires).
fn sdo_payload(data: &[u8]) -> Option<[u8; 8]> {
    let buf: [u8; 8] = data.try_into().ok()?;
    Some(buf)
}

const fn nonzero(v: u8) -> Option<u8> {
    if v == 0 {
        None
    } else {
        Some(v)
    }
}

/// Packs one TPDO payload from its mapping.
///
/// # Errors
/// Propagates `pdo::pack` failures (mapping type mismatches, missing
/// objects) and frame-construction errors — the caller decides whether to
/// drop the frame silently or record a configuration error (`BUGS.md` M1,
/// symmetric with the RPDO unpack path which records into
/// `config_errors`).
fn pack_tpdo(slot: &TpdoSlot, od: &ObjectDictionary) -> Result<Option<TxFrame>> {
    let mut buf = [0u8; 8];
    let n = pdo::pack(&slot.mapping, od, &mut buf)?;
    Ok(TxFrame::new(slot.cob_id(), &buf[..n]).ok())
}

#[cfg(test)]
mod bugs_tier3_regression_tests {
    use super::*;
    use crate::datatypes::Value;
    use crate::emcy::{install_standard_objects, EmcyConsumer};
    use crate::standard::StandardObjects;
    use crate::types::{Address, NodeId};

    fn base_node() -> Node {
        let mut od = StandardObjects::new().device_type(0x1234).build();
        install_standard_objects(&mut od, NodeId::new(5).unwrap());
        Node::new(NodeId::new(5).unwrap(), od).unwrap()
    }

    /// BUGS.md B3-1：OD 缺 0x1005 时 SYNC 应回退到默认 COB-ID 0x80。
    #[test]
    fn b3_1_is_sync_defaults_to_0x80_when_0x1005_absent() {
        let mut node = base_node();
        // 移除 0x1005，验证回退默认 0x80。
        node.od_mut().remove(Address::new(0x1005, 0));
        assert!(node.is_sync(0x80), "缺 0x1005 应回退默认 SYNC 0x80");
        assert!(!node.is_sync(0x81));
    }

    /// BUGS.md B3-1：显式配置 0x1005 时按配置值识别而非默认 0x80。
    #[test]
    fn b3_1_is_sync_honours_configured_0x1005() {
        let mut node = base_node();
        node.od_mut()
            .set_value(Address::new(0x1005, 0), Value::Uint(0x100))
            .unwrap();
        assert!(node.is_sync(0x100), "应按配置的 0x100 识别 SYNC");
        assert!(!node.is_sync(0x80));
    }

    /// BUGS.md B3-2：Initialising 阶段锁存的 EMCY 条件，进入 Operational 时应
    /// 补发一次（此前因 `raise` 幂等而从未出帧，上位机误判健康）。
    #[test]
    fn b3_2_latched_emcy_reannounced_on_operational() {
        let mut node = base_node();
        // Initialising 阶段 raise：条件被记录但不出帧（状态门拦截）。
        assert!(node.raise_emergency(0x1000, 0x01, [0; 5]).is_none());
        // boot → PreOperational（仍不应补发）。
        assert!(node.boot().is_some());
        // StartRemoteNode → Operational：应补发锁存的 EMCY。
        let _ = node.on_frame(0, &[0x01, 0x05]);
        let out = node.poll(0);
        assert!(
            out.iter().any(|f| f.cob_id == 0x80 + 5),
            "进入 Operational 应补发锁存的 EMCY 帧，实际帧: {out:?}"
        );
    }

    /// BUGS.md P2-1：SYNC COB-ID（含 0x1005 自定义 0x81..=0xFF）不得被可选
    /// EMCY 消费者吞掉——否则 SYNC 帧永远到不了 `on_sync`。
    #[test]
    fn p2_1_sync_cob_not_swallowed_by_emcy_consumer() {
        let mut node = base_node();
        node.set_emcy_consumer(EmcyConsumer::new(4));
        // 将 SYNC 配置到非默认 0x85。
        node.od_mut()
            .set_value(Address::new(0x1005, 0), Value::Uint(0x85))
            .unwrap();
        // 8 字节 EMCY 样帧落在 0x85（SYNC）上：应被当作 SYNC，而非 EMCY 消费。
        let _ = node.on_frame(0x85, &[0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0]);
        assert!(
            node.emcy_consumer().unwrap().active().is_empty(),
            "SYNC COB 上的帧不应被 EMCY 消费者消费"
        );
        // 真正的 EMCY 帧落在非 SYNC 的 0x82（node 2）上：应正常消费。
        let _ = node.on_frame(0x82, &[0x10, 0x00, 0x00, 0x00, 0, 0, 0, 0]);
        assert_eq!(node.emcy_consumer().unwrap().active().len(), 1);
    }

    /// BUGS.md H2: a short (<8 byte) frame on the SDO request channel must be
    /// dropped, not silently zero-padded into a malformed expedited request.
    #[test]
    fn h2_short_sdo_frame_is_dropped_not_zero_padded() {
        let mut node = base_node();
        // SDO is served only from PreOperational/Operational — boot first so
        // the short-frame path is actually exercised.
        node.boot().unwrap();
        // A 4-byte "expedited" frame on 0x605 (node 5's SDO RX) would, if
        // zero-padded, decode as a bogus expedited request; it must produce
        // no response and no dictionary change.
        let out = node.on_frame(0x605, &[0x40, 0x00, 0x10, 0x00]);
        assert!(out.is_empty(), "short SDO frame must be dropped: {out:?}");
        // And a genuine 8-byte request still works.
        use crate::sdo::SdoClient;
        let mut client = SdoClient::new();
        let req = client.read(
            Address::new(0x1000, 0),
            crate::datatypes::DataType::Unsigned32,
        );
        assert!(!node.on_frame(0x605, &req[0]).is_empty());
    }

    /// BUGS.md H2：落在 EMCY 区间（0x81..=0xFF）内的**自定义 RPDO COB-ID**
    /// 不得被 EMCY 消费者宽松解码吞掉——该帧属于 RPDO，应落到 PDO 匹配器
    /// 并写入映射对象。
    #[test]
    fn h2_custom_rpdo_in_emcy_range_not_swallowed() {
        use crate::datatypes::DataType;
        use crate::object_dictionary::{AccessType, Entry, Storage};
        use crate::pdo::MappingEntry;

        let mut node = base_node();
        // 映射目标：0x6000:1（U8，初始 0）。PDO 映射要求条目带
        // `pdo_mappable` 标志，否则 `validate_against` 拒绝该 slot。
        node.od_mut().insert(
            Address::new(0x6000, 1),
            Entry::new(
                "Target",
                DataType::Unsigned8,
                AccessType::Rw,
                Storage::Value(Value::Uint(0)),
            )
            .pdo_mappable(),
        );
        // RPDO1 通信记录（0x1400）与映射记录（0x1600）骨架。
        let put = |od: &mut crate::object_dictionary::ObjectDictionary,
                   addr: Address,
                   dt: DataType,
                   v: u64| {
            od.insert(
                addr,
                Entry::new("", dt, AccessType::Rw, Storage::Value(Value::Uint(v))),
            );
        };
        put(
            node.od_mut(),
            Address::new(0x1400, 1),
            DataType::Unsigned32,
            0,
        );
        put(
            node.od_mut(),
            Address::new(0x1400, 2),
            DataType::Unsigned8,
            0,
        );
        put(
            node.od_mut(),
            Address::new(0x1600, 0),
            DataType::Unsigned8,
            0,
        );
        put(
            node.od_mut(),
            Address::new(0x1600, 1),
            DataType::Unsigned32,
            0,
        );
        // RPDO1 → 自定义 COB 0x89（= node 9 的标准 EMCY COB，落在区间内）。
        node.od_mut()
            .set_value(Address::new(0x1400, 1), Value::Uint(0x89))
            .unwrap();
        // 传输类型 254（事件驱动）：收到即应用，无需 SYNC。
        node.od_mut()
            .set_value(Address::new(0x1400, 2), Value::Uint(254))
            .unwrap();
        node.od_mut()
            .set_value(Address::new(0x1600, 0), Value::Uint(1))
            .unwrap();
        node.od_mut()
            .set_value(
                Address::new(0x1600, 1),
                Value::Uint(
                    MappingEntry {
                        address: Address::new(0x6000, 1),
                        bit_length: 8,
                    }
                    .to_u32()
                    .into(),
                ),
            )
            .unwrap();
        node.configure_pdos_from_od().unwrap();

        node.set_emcy_consumer(EmcyConsumer::new(4));
        node.boot().unwrap();
        // StartRemoteNode → Operational（RPDO 只在 Operational 生效）。
        let _ = node.on_frame(0, &[0x01, 0x00]);
        assert_eq!(node.nmt_state(), NmtState::Operational);

        // 8 字节帧落在 0x89（自定义 RPDO COB）：必须被 RPDO 应用，
        // 而不是被 EMCY 消费者当作宽松解码的 EMCY 吞掉。
        let _ = node.on_frame(0x89, &[0xAB, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(
            node.od().read(Address::new(0x6000, 1)).unwrap(),
            Value::Uint(0xAB),
            "自定义 RPDO COB 上的帧应写入映射对象"
        );
        assert!(
            node.emcy_consumer().unwrap().active().is_empty(),
            "该帧是 RPDO 数据而非 EMCY，不应被消费者记录"
        );

        // 对照组：同一个消费者仍消费未认领的标准 EMCY COB（0x82）。
        let _ = node.on_frame(0x82, &[0x10, 0x00, 0x00, 0x00, 0, 0, 0, 0]);
        assert_eq!(node.emcy_consumer().unwrap().active().len(), 1);
    }

    /// BUGS.md 十M20：EMCY 消费者只接受标准范围 0x80..=0xFF。旧实现把
    /// 0x181/0x281（TPDO1..）等按 `(cob-0x80) as u8` 回绕成 "节点号" 消费，
    /// 使这些 COB 永远到不了 PDO/SYNC 分发。
    #[test]
    fn emcy_consumer_ignores_cob_ids_above_0xff() {
        let mut node = base_node();
        node.set_emcy_consumer(EmcyConsumer::new(4));
        node.boot().unwrap();
        // TPDO1 COB 0x181 携带 EMCY 形状的载荷：不得被消费者记录。
        let _ = node.on_frame(0x181, &[0x10, 0x00, 0x00, 0x00, 0, 0, 0, 0]);
        assert!(node.emcy_consumer().unwrap().active().is_empty());
        assert!(node.emcy_consumer().unwrap().recent().next().is_none());
    }

    /// BUGS.md M4：类型 252（RtrOnlySync）TPDO 的 RTR 应答必须在"最后 SYNC"
    /// 采样，而非当前实时 OD 值。先在 SYNC 前/后改变映射对象，确认 RTR 回放
    /// 的是 SYNC 时刻的快照。
    #[test]
    fn m4_rtronlysync_rtr_returns_sync_snapshot() {
        use crate::datatypes::DataType;
        use crate::object_dictionary::{AccessType, Entry, Storage};
        use crate::pdo::MappingEntry;

        let mut node = base_node();
        node.od_mut().insert(
            Address::new(0x6000, 1),
            Entry::new(
                "Target",
                DataType::Unsigned8,
                AccessType::Rw,
                Storage::Value(Value::Uint(0)),
            )
            .pdo_mappable(),
        );
        let put = |od: &mut crate::object_dictionary::ObjectDictionary,
                   addr: Address,
                   dt: DataType,
                   v: u64| {
            od.insert(
                addr,
                Entry::new("", dt, AccessType::Rw, Storage::Value(Value::Uint(v))),
            );
        };
        // TPDO1 通信记录（0x1800）：COB-ID = 0x185（node 5 默认）+ RTR 允许，
        // 传输类型 252（RtrOnlySync）。
        put(
            node.od_mut(),
            Address::new(0x1800, 1),
            DataType::Unsigned32,
            0x185u64, // bit30=0：RTR 允许（CiA 语义，BUGS.md 八H1）
        );
        put(
            node.od_mut(),
            Address::new(0x1800, 2),
            DataType::Unsigned8,
            252,
        );
        // TPDO1 映射记录（0x1A00）→ 0x6000:1（U8，8 位）。
        put(
            node.od_mut(),
            Address::new(0x1A00, 0),
            DataType::Unsigned8,
            1,
        );
        put(
            node.od_mut(),
            Address::new(0x1A00, 1),
            DataType::Unsigned32,
            u64::from(
                MappingEntry {
                    address: Address::new(0x6000, 1),
                    bit_length: 8,
                }
                .to_u32(),
            ),
        );
        node.configure_pdos_from_od().unwrap();
        node.boot().unwrap();
        let _ = node.on_frame(0, &[0x01, 0x00]); // StartRemoteNode → Operational
        assert_eq!(node.nmt_state(), NmtState::Operational);

        // SYNC 前设置值 0x11。
        node.od_mut()
            .set_value(Address::new(0x6000, 1), Value::Uint(0x11))
            .unwrap();
        // 首个 SYNC：快照锁定为 0x11。
        let _ = node.on_frame(0x80, &[]); // 0x1019=0：无 counter 的 SYNC
                                          // SYNC 之后改值为 0x22（实时值已变）。
        node.od_mut()
            .set_value(Address::new(0x6000, 1), Value::Uint(0x22))
            .unwrap();

        // RTR 应答必须回放 SYNC 快照（0x11），而非当前值 0x22。
        let frame = node
            .on_remote_request(0x185)
            .expect("RtrOnlySync 应应答 RTR");
        assert_eq!(
            frame.payload()[0],
            0x11,
            "RTR 应答应为 SYNC 采样值 0x11，而非实时值 0x22（BUGS.md M4）"
        );

        // 第二个 SYNC 后再改值：快照刷新为 0x33。
        node.od_mut()
            .set_value(Address::new(0x6000, 1), Value::Uint(0x33))
            .unwrap();
        let _ = node.on_frame(0x80, &[]);
        node.od_mut()
            .set_value(Address::new(0x6000, 1), Value::Uint(0x44))
            .unwrap();
        let frame2 = node
            .on_remote_request(0x185)
            .expect("RtrOnlySync 应应答 RTR");
        assert_eq!(
            frame2.payload()[0],
            0x33,
            "RTR 应答应为最新 SYNC 采样值 0x33"
        );
    }
}
