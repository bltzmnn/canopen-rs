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
    /// PDO number in [`Node::request_tpdo`] (`docs/DESIGN-DECISIONS.md` P3-19).
    comm_index: u16,
    config: PdoConfig,
    mapping: PdoMapping,
    /// Resolved transmission type (241..251 rejected at build time).
    tt: TransmissionType,
    /// SYNCs observed since the last transmission (cyclic divider).
    /// Seeded with `n - (n/2 + 1)` so the first SyncCyclic frame goes out
    /// at the middle of the first period, matching CANopenNode
    /// `CO_PDO.c:1315-1321` (docs/DESIGN-DECISIONS.md #27).
    syncs_seen: u32,
    /// Last transmission in monotonic ms (inhibit + event timer).
    last_tx_ms: Option<u64>,
    /// A synchronous slot transmitted at the last SYNC but not yet stamped:
    /// `on_frame` carries no time (sans-I/O), so the next `poll` promotes
    /// this into `last_tx_ms`. Re-anchors the tt=0 event timer on every
    /// actual transmission like CANopenNode's `CO_TPDOsend`
    /// (`docs/DESIGN-DECISIONS.md` N1); resolution is one poll interval.
    tx_seen: bool,
    /// Application-requested transmission (event-driven trigger).
    pending_event: bool,
    /// SYNC start-value phase lock (S14 / CANopenNode `CO_PDO.c:1315-1336`):
    /// set once when the global counter first matches the configured start
    /// value and kept for the whole Operational session — cyclic counting
    /// then proceeds independently of further counter values. Cleared when
    /// NMT leaves Operational (`docs/DESIGN-DECISIONS.md` P2-1: re-locking after every send
    /// silenced cyclic TPDOs until counter wraparound).
    phase_locked: bool,
    /// The slot has produced **at least one** TPDO frame since the last
    /// NMT reset.
    ///
    /// docs/DESIGN-DECISIONS.md PDO-26: the reference consumes `syncStartValue` only while
    /// *seeding* a fresh slot (`CO_PDO.c:1315-1336`, the one-shot
    /// `syncCounter == 255` branch) and `OD_write_1019` never touches
    /// `syncCounter` (`:139-169`), so enabling 0x1019 at runtime must not
    /// re-gate a TPDO that is already transmitting. This flag makes "not yet
    /// seeded" the wait condition instead of re-evaluating it every SYNC.
    ever_sent: bool,
    /// A SYNC was received this Operational session (`docs/DESIGN-DECISIONS.md` M2): a
    /// type-252 (RTR-only-sync) TPDO answers a remote request only after
    /// the first SYNC — CiA 301 §7.4.3, the response is the last SYNC
    /// sample. Cleared when NMT leaves Operational, like `syncs_seen`.
    sync_received: bool,
    /// 类型 252（RtrOnlySync）PDO 在最近一次 SYNC 处采样的载荷快照
    /// （docs/DESIGN-DECISIONS.md M4）：规范规定 RTR 应答的数据应在"最后 SYNC"时刻采样，
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
    /// Transmission type snapshot (`docs/DESIGN-DECISIONS.md` P2-4): synchronous slots buffer
    /// received data until the next SYNC instead of applying it immediately.
    tt: TransmissionType,
    /// Buffered payload of a synchronous RPDO awaiting the next SYNC
    /// (double-buffer semantics: "takes effect after the next SYNC",
    /// CANopenNode `CO_PDO.c:717-760`). `None` when nothing is pending.
    sync_pending: Option<([u8; 8], usize)>,
    /// A frame arrived since the last [`Node::poll`] window (P2-7).
    rx_seen: bool,
    /// Poll timestamp when traffic was last observed on this slot; `None`
    /// until the first reception arms the deadman (`docs/DESIGN-DECISIONS.md` P2-7).
    last_rx_ms: Option<u64>,
    /// Length-condition latch of the RPDO receive path (see [`RxLenState`]).
    rx_len: RxLenState,
    /// Timeout already reported for the current silent window.
    reported: bool,
}

/// CANopenNode's RPDO `receiveError` state machine, kept verbatim
/// (`CO_PDO.c:439-478` receive path, `:717-728` reporting gate).
///
/// docs/DESIGN-DECISIONS.md PDO-25: the reference keeps **one** length condition per slot, only
/// sets a code on the "no error → error" edge, and acks it exclusively with a
/// frame of *exactly* the declared length; `CO_error` itself returns without
/// producing a frame when the bit is already in the requested state
/// (`CO_Emergency.c:658-666`). The stack models that single condition as this
/// five-state latch (the `CO_RPDO_RX_*` constants of `CO_PDO.h`), so the
/// "short frame followed by an over-long frame" sequence produces **zero**
/// new frames instead of clearing 0x8210 and raising 0x8220.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
enum RxLenState {
    /// `CO_RPDO_RX_ACK_NO_ERROR`: nothing latched.
    #[default]
    NoError,
    /// `CO_RPDO_RX_SHORT`: an under-long frame was seen, not yet reported.
    PendingShort,
    /// `CO_RPDO_RX_LONG`: an over-long frame was seen, not yet reported.
    PendingLong,
    /// `CO_RPDO_RX_OK`: an exactly-sized frame followed a latched error; the
    /// reset still has to be announced by the next gate.
    PendingOk,
    /// `CO_RPDO_RX_ACK_ERROR`: the error is on the bus and latched until an
    /// exactly-sized frame arrives.
    Latched,
}

/// CAN controller health inputs that the sans-I/O core cannot observe.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CanStatus {
    /// Controller is bus-off.
    pub bus_off: bool,
    /// Controller reached the error-warning limit.
    pub bus_warn: bool,
}

/// CANopenNode-compatible NMT control flags (not a CiA object; an
/// implementation configuration surface). All flags default to the stack's
/// historical behaviour: boot into PreOperational and never leave it on
/// errors automatically.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct NmtControl {
    /// Enter Operational immediately after boot/reset.
    pub startup_to_operational: bool,
    /// Leave Operational when the masked error register is non-zero.
    pub err_on_err_reg: bool,
    /// Error-register bits considered by `err_on_err_reg`.
    pub err_reg_mask: u8,
    /// On an error condition enter Stopped instead of PreOperational.
    pub err_to_stopped: bool,
    /// Automatically return from PreOperational to Operational when no error
    /// condition is present.
    pub err_free_to_operational: bool,
    /// Treat bus-off and heartbeat-consumer errors as the error condition.
    pub err_on_busoff_hb: bool,
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
    /// Node-guarding slave life-time supervision (`docs/DESIGN-DECISIONS.md` NMT-03): a
    /// guarding RTR seen since the last poll re-anchors `guard_last_ms`; a
    /// silent master raises 0x8130 and the next RTR clears it. Guarding is
    /// disabled while 0x1017 > 0, exactly like CANopenNode's
    /// `slaveDisable` (`CANopen.c:1384`).
    guard_last_ms: Option<u64>,
    guard_timeout_active: bool,
    guard_rtr_since_poll: bool,
    guard_rearm_pending: bool,
    /// CiA 303-3 indicator state machine and the last poll timestamp used to
    /// advance it (docs/DESIGN-DECISIONS.md LED-01).
    leds: crate::leds::Leds,
    last_led_ms: Option<u64>,
    /// Host-injected CAN controller health for the LED/NMTcontrol inputs.
    can_status: CanStatus,
    /// Application-injected firmware-download flag for the green LED.
    firmware_download: bool,
    /// CANopenNode-compatible NMT control flags (docs/DESIGN-DECISIONS.md NMT-06).
    nmt_control: NmtControl,
    last_hb_ms: Option<u64>,
    /// docs/DESIGN-DECISIONS.md 八M4: set by `boot()`/reset until the first `poll` after
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
    /// disappears without a trace (docs/DESIGN-DECISIONS.md m17). Bounded ring: once
    /// [`MAX_CONFIG_ERRORS`] entries are held, further errors increment
    /// `config_errors_dropped` instead of growing unboundedly (P2-3).
    config_errors: Vec<crate::types::Error>,
    config_errors_dropped: u64,
    /// EMCY payloads waiting out their 0x1015 inhibit window (`docs/DESIGN-DECISIONS.md`
    /// P2-6); released by [`Node::poll`].
    pending_emcy: std::collections::VecDeque<[u8; 8]>,
    /// Monotonic-ms timestamp of the last EMCY transmission (inhibit base).
    last_emcy_ms: Option<u64>,
    /// Set by an OD 0x1015 write: the next `poll` anchors a fresh inhibit
    /// window, mirroring CANopenNode's `inhibitEmTimer = 0`
    /// (`CO_Emergency.c:134-146`; docs/DESIGN-DECISIONS.md EMC-05).
    emcy_inhibit_rearm: bool,
    /// Whether a valid SYNC frame arrived since the last [`Node::poll`]
    /// (SYNC producer supervision, `docs/DESIGN-DECISIONS.md` SYN-01).
    sync_seen_since_poll: bool,
    /// Whether *any* valid SYNC has been observed since construction or the
    /// last communication reset (`docs/DESIGN-DECISIONS.md` SYN-08/SYN-12): the reference only
    /// arms the silence supervision after the first SYNC
    /// (`CO_SYNC.c:402-406`), so a master that starts producing later must not
    /// trigger a phantom timeout. Leaving (Pre)Operational **keeps** this
    /// armed, matching `timeoutError` surviving the not-(Pre)Op branch
    /// (`CO_SYNC.c:395-400`).
    sync_ever_seen: bool,
    /// Monotonic-ms anchor the 1.5 × period window is counted from: the poll
    /// that observed the last SYNC — or, when re-entering (Pre)Operational
    /// after a Stop, the first poll back in that state (the reference's
    /// `timer = 0` then re-accumulating, `CO_SYNC.c:316-329,395-400`).
    /// `None` while not in (Pre)Operational, and before the first SYNC ever.
    last_sync_ms: Option<u64>,
    /// Whether the `SYNC_TIMEOUT_EMCY` condition is currently raised
    /// (`docs/DESIGN-DECISIONS.md` SYN-01).
    sync_timeout_active: bool,
    /// RPDO event-timer timeouts (communication addresses), drained via
    /// [`Node::take_rpdo_timeouts`] (`docs/DESIGN-DECISIONS.md` P2-7).
    rpdo_timeouts: std::collections::VecDeque<Address>,
    /// Whether the communication-profile 0x8250 EMCY condition is currently
    /// raised because at least one RPDO dead line lapsed (`docs/DESIGN-DECISIONS.md` 七F2).
    /// Kept as a single latch: CANopenNode reports one shared
    /// `CO_EM_RPDO_TIME_OUT` error bit for every RPDO
    /// (`CO_PDO.c:822-831`), and `EmcyProducer::raise` is idempotent.
    ///
    /// Measured silence (µs) behind that 0x8250 report, replayed into the
    /// recovery frame's infoCode like CANopenNode's
    /// `CO_errorReset(..., RPDO->timeoutTimer)` (`docs/DESIGN-DECISIONS.md` EMC-17).
    rpdo_elapsed_us: u32,
    rpdo_emcy_active: bool,
    /// Whether the 0x8130 heartbeat-consumer EMCY condition is currently
    /// raised because a monitored node timed out (`docs/DESIGN-DECISIONS.md` 审-N1). Single
    /// shared latch, like CANopenNode's `CO_EM_HEARTBEAT_CONSUMER` error
    /// bit (`CO_HBconsumer.c:353-367,423-428`).
    hb_emcy_active: bool,
    /// Consumer-heartbeat nodes currently installed *from OD 0x1016*
    /// (`docs/DESIGN-DECISIONS.md` P1-11); programmatic `HeartbeatConsumer` users are
    /// never touched by the OD resync.
    hb_od_consumers: Vec<NodeId>,
    /// OD-configured heartbeat consumer (§10.3 supervision).
    hb_consumer: crate::nmt::HeartbeatConsumer,
    /// Optional consumer for foreign emergency frames (0x80+n, n≠self);
    /// `None` by default — a slave does not consume peer EMCY unless an
    /// application installs a consumer (`docs/DESIGN-DECISIONS.md` M3).
    emcy_consumer: Option<crate::emcy::EmcyConsumer>,
    /// Foreign heartbeat frames received since the last poll.
    /// Pending foreign heartbeat payloads (always exactly one byte, `docs/DESIGN-DECISIONS.md`
    /// NMT-04) with their node id, drained by [`Node::poll`].
    hb_rx_queue: std::collections::VecDeque<(NodeId, Vec<u8>)>,
    /// TIME frames received on the 0x1012-configured COB-ID (default 0x100),
    /// drained via [`Node::take_time_events`] (`docs/DESIGN-DECISIONS.md` P3-21/SYN-10).
    time_rx_queue: std::collections::VecDeque<[u8; 6]>,
    /// SDO session timeout in ms (`None` = disabled); enforced via
    /// [`Node::touch_sdo_activity`] / [`Node::sdo_timeout_due`] with a
    /// caller-supplied clock (§5.5, `docs/DESIGN-DECISIONS.md` P2-8). The underlying
    /// `SdoServer` keeps a sliding inactivity window, so sustained traffic
    /// never trips the timer (`docs/DESIGN-DECISIONS.md` R10-P1-4).
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
/// grow the queues without bound (docs/DESIGN-DECISIONS.md 八L15). Oldest frames are
/// dropped, mirroring the EMCY active-table policy (M5).
const MAX_RX_QUEUE: usize = 256;
/// Upper bound for EMCY payloads deferred behind the 0x1015 inhibit window
/// (`docs/DESIGN-DECISIONS.md` 七F5). CANopenNode's emergency fifo is fixed-size too and a
/// full fifo drops the *new* message (setting `fifoOverflow`,
/// `CO_Emergency.c:695-706`); the condition itself is still recorded in
/// 0x1001/0x1003, only its frame is suppressed.
const MAX_PENDING_EMCY: usize = 256;
/// EMCY code reported when the SYNC producer goes silent (`docs/DESIGN-DECISIONS.md` SYN-01).
///
/// CANopenNode reports `CO_EM_SYNC_TIME_OUT` with the generic communication
/// code (`CO_SYNC.c:362`: `CO_errorReport(em, CO_EM_SYNC_TIME_OUT,
/// CO_EMC_COMMUNICATION, timer)`), i.e. wire code `0x8100` with the error bit
/// (`0x18`) in the first manufacturer byte — the CiA 301 code table has no
/// dedicated "SYNC timeout" number (`0x8240` is the *data length* code this
/// stack already uses).
const SYNC_TIMEOUT_EMCY_CODE: u16 = 0x8100;
/// CANopenNode's `CO_EM_SYNC_TIME_OUT` error-bit index, carried in
/// `info[0]` (`CO_Emergency.c:673`).
const SYNC_TIMEOUT_EMCY_BIT: u8 = 0x18;

/// Communication error-bit indices (`CO_Emergency.h:185,205-207`) for the
/// conditions this stack raises; each travels in frame byte 3.
const EM_BIT_RPDO_WRONG_LENGTH: u8 = 0x04;
const EM_BIT_RPDO_TIME_OUT: u8 = 0x17;
const EM_BIT_SYNC_LENGTH: u8 = 0x19;
/// CANopenNode's `CO_EM_HEARTBEAT_CONSUMER` error-bit index, carried in the
/// 0x8130 frame's first manufacturer byte (`CO_Emergency.h:209`; raised from
/// `CO_HBconsumer.c:364`).
const EM_BIT_HEARTBEAT_CONSUMER: u8 = 0x1B;

/// CANopenNode's emergency manufacturer field: `[errorBit, infoCode]` — the
/// error-bit index in frame byte 3 followed by the little-endian 32-bit info
/// value in frame bytes 4..7 (`CO_Emergency.c:673-676` builds
/// `errMsg = errorBit<<24 | code`, `:578-581` copies `msg` + `info` into the
/// 8-byte payload; the consumer side reads the info code from `data[4]`,
/// `:291-293`). docs/DESIGN-DECISIONS.md EMC-14: the previous `length_info` put the value in
/// frame bytes 3..6 and left byte 7 zero, one byte off the reference.
const fn emcy_vendor_info(err_bit: u8, info: u32) -> [u8; 5] {
    let b = info.to_le_bytes();
    [err_bit, b[0], b[1], b[2], b[3]]
}

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
    /// `docs/DESIGN-DECISIONS.md` P1-11). Every non-zero entry `nodeId<<16 | timeout_ms`
    /// installs/updates a consumer immediately; configured consumers absent
    /// from the dictionary are removed. A nodeId appearing twice rejects the
    /// later entry (zeroed back) with a recorded error — CiA 301 allows a
    /// node under exactly one consumer.
    ///
    /// Only consumers installed through this path are ever removed here;
    /// programmatic `HeartbeatConsumer::set_timeout` users are untouched.
    fn resync_hb_consumers_from_od(&mut self) {
        let mut wanted: Vec<(u8, u32, u16)> = Vec::new();
        for sub in 1..=0xFEu8 {
            let Ok(Value::Uint(v)) = self.od.read(Address::new(0x1016, sub)) else {
                continue;
            };
            let node = ((v >> 16) & 0xFF) as u8;
            let ms = (v & 0xFFFF) as u32;
            if node == 0 || ms == 0 {
                continue;
            }
            if wanted.iter().any(|(n, _, _)| *n == node) {
                let addr = Address::new(0x1016, sub);
                let _ = self.od.set_value(addr, Value::Uint(0));
                self.record_config_error(crate::types::Error::Other(
                    "0x1016: duplicate consumer nodeId rejected",
                ));
                continue;
            }
            wanted.push((node, ms, u16::from(sub - 1)));
        }
        let mut mapped: Vec<NodeId> = Vec::new();
        for (node, ms, index) in wanted {
            match NodeId::try_from(node) {
                Ok(n) => {
                    self.hb_consumer.set_timeout_with_index(n, ms, index);
                    mapped.push(n);
                }
                Err(_) => {
                    // Out-of-range nodeId (node > 127): zero the entry and
                    // record it like duplicates — silent residue would be
                    // re-parsed on every resync (docs/DESIGN-DECISIONS.md P3-15).
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
            guard_last_ms: None,
            guard_timeout_active: false,
            guard_rtr_since_poll: false,
            guard_rearm_pending: false,
            leds: crate::leds::Leds::new(),
            last_led_ms: None,
            can_status: CanStatus::default(),
            firmware_download: false,
            nmt_control: NmtControl::default(),
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
            emcy_inhibit_rearm: false,
            sync_seen_since_poll: false,
            sync_ever_seen: false,
            last_sync_ms: None,
            sync_timeout_active: false,
            rpdo_timeouts: std::collections::VecDeque::new(),
            rpdo_emcy_active: false,
            rpdo_elapsed_us: 0,
            hb_emcy_active: false,
            hb_od_consumers: Vec::new(),
            hb_consumer: crate::nmt::HeartbeatConsumer::default(),
            emcy_consumer: None,
            hb_rx_queue: std::collections::VecDeque::new(),
            time_rx_queue: std::collections::VecDeque::new(),
            sdo_timeout_ms: None,
            pending_hb_events: Vec::new(),
        };
        // docs/DESIGN-DECISIONS.md 八M3: the `StandardObjects` baseline stores node-1
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
        node.renumber_entry(
            Address::new(0x1280, 2),
            NodeId::new(1).unwrap_or(node_id),
            node_id,
        );
        node.configure_pdos_from_od()?;
        Ok(node)
    }

    /// Creates a node that boots **unconfigured**: it serves only LSS on
    /// `0x7E5` until an id is assigned and adopted via
    /// [`Self::apply_lss_node_id`] (docs/IMPLEMENTATION.md §9).
    ///
    /// `address` and the dictionary's 0x1018:1..4 are **two independent
    /// sources** of the device identity — LSS matching uses the former, SDO
    /// reads of 0x1018 report the latter, and nothing here reconciles them
    /// (the reference has the same split, `CO_LSSslave.c:193`). Derive one from
    /// the other with [`crate::lss::LssAddress::from_od`] to keep them equal
    /// (`docs/DESIGN-DECISIONS.md` LSS-02).
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
            guard_last_ms: None,
            guard_timeout_active: false,
            guard_rtr_since_poll: false,
            guard_rearm_pending: false,
            leds: crate::leds::Leds::new(),
            last_led_ms: None,
            can_status: CanStatus::default(),
            firmware_download: false,
            nmt_control: NmtControl::default(),
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
            emcy_inhibit_rearm: false,
            sync_seen_since_poll: false,
            sync_ever_seen: false,
            last_sync_ms: None,
            sync_timeout_active: false,
            rpdo_timeouts: std::collections::VecDeque::new(),
            rpdo_emcy_active: false,
            rpdo_elapsed_us: 0,
            hb_emcy_active: false,
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
        // adopted (docs/DESIGN-DECISIONS.md A2). `NodeId` cannot represent 0xFF, so signal the
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
            // like `set_node_id` (docs/DESIGN-DECISIONS.md P3-6). 0x1280:1 follows too so an
            // LSS-configured node does not keep the stale 0x600 placeholder
            // (docs/DESIGN-DECISIONS.md P2-2, see git history).
            self.renumber_entry(Address::new(0x1014, 0), previous, id);
            self.renumber_entry(Address::new(0x1280, 1), previous, id);
            // docs/DESIGN-DECISIONS.md OD-01: sub2 (server→client) is a second predefined pair.
            self.renumber_entry(Address::new(0x1280, 2), previous, id);
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
    /// PDO slots rebuilt (docs/IMPLEMENTATION.md §7.2, `docs/DESIGN-DECISIONS.md` P1-12).
    /// Custom COB-IDs are left untouched (CANopenNode semantics).
    pub fn set_node_id(&mut self, id: NodeId) {
        let previous = self.node_id;
        if previous == id {
            return;
        }
        self.node_id = id;
        self.renumber_entry(Address::new(0x1014, 0), previous, id);
        self.renumber_entry(Address::new(0x1280, 1), previous, id);
        // docs/DESIGN-DECISIONS.md OD-01: sub2 (server→client) is a second predefined pair.
        self.renumber_entry(Address::new(0x1280, 2), previous, id);
        self.rewrite_pdo_node_bits(previous, id);
        if let Err(e) = self.configure_pdos_from_od() {
            self.record_config_error(e);
        }
    }

    /// Predefined (connection-set) COB-ID pair `(old, new)` for an
    /// identity-dependent entry; `None` for entries without one.
    fn predefined_pair(addr: Address, old: NodeId, new: NodeId) -> Option<(u32, u32)> {
        match (addr.index, addr.subindex) {
            (0x1014, _) => Some((u32::from(old.emcy_cob_id()), u32::from(new.emcy_cob_id()))),
            // The SDO client record's two COB-IDs follow the node too, so a
            // master inspecting 0x1280:1/2 does not read stale node-1
            // placeholders (`docs/DESIGN-DECISIONS.md` P3-38; docs/DESIGN-DECISIONS.md OD-01 added sub2).
            (0x1280, 1) => Some((0x600 + u32::from(old.get()), 0x600 + u32::from(new.get()))),
            (0x1280, 2) => Some((0x580 + u32::from(old.get()), 0x580 + u32::from(new.get()))),
            (idx, 1) if (0x1400..=0x15FF).contains(&idx) => {
                let num = (idx - 0x1400 + 1) as u8;
                Some((
                    u32::from(pdo::default_cob_id(false, num, old)?),
                    u32::from(pdo::default_cob_id(false, num, new)?),
                ))
            }
            (idx, 1) if (0x1800..=0x19FF).contains(&idx) => {
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
        let Some((expected_old, expected_new)) = Self::predefined_pair(addr, old, new) else {
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

    /// Drains errors recorded while re-assembling PDO slots, unpacking
    /// received RPDO payloads, or dispatching parameter storage (backend
    /// save/load/reset failures, e.g. a corrupt storage file — `docs/DESIGN-DECISIONS.md`
    /// G7). The node keeps operating with its working PDOs (per
    /// docs/IMPLEMENTATION.md §7.4 a partially misconfigured dictionary must
    /// not disable the rest), but nothing is swallowed silently — check this
    /// after configuration writes.
    pub fn take_config_errors(&mut self) -> std::vec::Drain<'_, crate::types::Error> {
        self.config_errors.drain(..)
    }

    /// Installs an EMCY consumer, enabling consumption of foreign
    /// emergency frames (0x80+n, n≠self) via [`Node::on_frame`]
    /// (`docs/DESIGN-DECISIONS.md` M3). Without one, the slave does not consume peer EMCY
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
    /// the bounded buffer filled (`docs/DESIGN-DECISIONS.md` P2-3).
    pub fn take_dropped_config_errors(&mut self) -> u64 {
        std::mem::take(&mut self.config_errors_dropped)
    }

    /// Whether an SDO transfer is in flight on the default channel.
    #[must_use]
    pub fn sdo_is_busy(&self) -> bool {
        self.sdo.is_busy()
    }

    /// Enables the SDO session timeout (§5.5, `docs/DESIGN-DECISIONS.md` P2-8): a transfer
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

    /// Installs the optional SDO-server strictness configuration
    /// ([`crate::sdo::SdoServerConfig`]). Defaults are lenient and preserve
    /// the historical behaviour; `strict` enables CANopenNode's exact
    /// command/abort-byte validation (docs/DESIGN-DECISIONS.md SDO-33/SDO-35).
    pub fn set_sdo_config(&mut self, config: crate::sdo::SdoServerConfig) {
        self.sdo.set_config(config);
    }

    /// Current SDO-server strictness configuration.
    #[must_use]
    pub const fn sdo_config(&self) -> crate::sdo::SdoServerConfig {
        self.sdo.config()
    }

    /// Selects the PDO mapping write-validation policy
    /// ([`crate::pdo::PdoConfigPolicy`]). Defaults to assembly-time checks.
    pub fn set_pdo_config_policy(&mut self, policy: crate::pdo::PdoConfigPolicy) {
        self.sdo.set_pdo_policy(policy);
    }

    /// Current PDO mapping write-validation policy.
    #[must_use]
    pub const fn pdo_config_policy(&self) -> crate::pdo::PdoConfigPolicy {
        self.sdo.pdo_policy()
    }

    /// Updates the CAN controller health used by CiA 303-3 LED inputs and by
    /// NMT `NMTcontrol` bus-off evaluation. The core cannot observe the
    /// controller, so the host integration calls this on error frames/state
    /// changes.
    pub fn set_can_status(&mut self, status: CanStatus) {
        self.can_status = status;
    }

    /// Current host-injected CAN status.
    #[must_use]
    pub const fn can_status(&self) -> CanStatus {
        self.can_status
    }

    /// Installs CANopenNode-compatible NMT control flags (docs/DESIGN-DECISIONS.md NMT-06).
    /// Defaults are all-off, preserving the historical PreOperational boot.
    pub fn set_nmt_control(&mut self, control: NmtControl) {
        self.nmt_control = control;
    }

    /// Current NMT control flags.
    #[must_use]
    pub const fn nmt_control(&self) -> NmtControl {
        self.nmt_control
    }

    /// Marks an application firmware download for the green LED.
    pub fn set_firmware_download(&mut self, active: bool) {
        self.firmware_download = active;
    }

    /// Latest CiA 303-3 LED state computed by [`Node::poll`].
    #[must_use]
    pub fn leds(&self) -> crate::leds::LedOutput {
        self.leds.output()
    }

    /// Slides the SDO §5.5 inactivity window forward (`docs/DESIGN-DECISIONS.md` R10-P1-4).
    ///
    /// Call it when a frame arrives on **this node's SDO RX channel**
    /// (`0x600 + node`), not for every frame the node handles: the bundled
    /// `serve_node` driver does exactly that (`host/src/sdo_exchange.rs:430-443`)
    /// and blanket "any handled frame" callers would keep a wedged SDO session
    /// alive forever on a busy bus (`docs/DESIGN-DECISIONS.md` API-02).
    pub fn touch_sdo_activity(&mut self, now_ms: u64) {
        self.sdo.touch(now_ms);
    }

    /// Session-timeout maintenance: tears down a wedged SDO transfer once no
    /// frame arrived for the configured timeout and returns the timeout abort
    /// frame (`0x05040000`) for transmission — the host serve loop sends it,
    /// matching CANopenNode `CO_SDOserver.c:1076-1079,1496-1509`
    /// (`docs/DESIGN-DECISIONS.md` C1). The inactivity window slides forward with
    /// [`Node::touch_sdo_activity`], so a long-running transfer with steady
    /// traffic is never torn down by a fixed total-session cap
    /// (`docs/DESIGN-DECISIONS.md` R10-P1-4).
    pub fn sdo_timeout_due(&mut self, now_ms: u64) -> Option<TxFrame> {
        let timeout = self.sdo_timeout_ms?;
        let abort = self.sdo.on_timeout(now_ms, timeout)?;
        TxFrame::new(self.node_id.sdo_tx_cob_id(), &abort).ok()
    }

    /// Drains TIME frames received on the configured 0x1012 COB-ID (default
    /// 0x100) as raw 6-byte stamps (ms-of-day little-endian u32 + days since
    /// 1984 u16) — `docs/DESIGN-DECISIONS.md` P3-21/SYN-10.
    pub fn take_time_events(&mut self) -> Vec<[u8; 6]> {
        self.time_rx_queue.drain(..).collect()
    }

    /// Drains buffered heartbeat events (Started/Lost/RemoteReset/
    /// StateChanged) produced by the consumer since the last call.
    pub fn take_hb_events(&mut self) -> Vec<crate::nmt::HbEvent> {
        std::mem::take(&mut self.pending_hb_events)
    }

    /// Appends a heartbeat event to the bounded output queue; the oldest
    /// event is dropped once the cap is reached (docs/DESIGN-DECISIONS.md 十M8, the same
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
        // §6.5 (docs/DESIGN-DECISIONS.md P1-11): consumer heartbeat timeouts come from
        // OD 0x1016 (u32: nodeId<<16 | timeout_ms); any write re-syncs the
        // consumer configuration immediately.
        if addr.index == 0x1016 {
            self.resync_hb_consumers_from_od();
            return;
        }
        // NMT-03: guard-time / life-factor writes restart a running life
        // timer (CANopenNode `OD_write_100C/100D`, `CO_Node_Guarding.c:53-91`).
        // The core has no clock in the write path, so the fresh anchor is
        // applied by the next `poll` (only when a timer was already running).
        if matches!(addr.index, 0x100C | 0x100D) {
            self.guard_rearm_pending = true;
            return;
        }
        // CiA 301 / CANopenNode `OD_write_1017`: writing the producer
        // heartbeat time restarts the producer and sends the next heartbeat
        // immediately (`CO_NMT_Heartbeat.c:64-66,187-189`; docs/DESIGN-DECISIONS.md B4). The
        // core is clockless, so mark the period expired for the next `poll`
        // and clear the boot anchor so a write before the first poll is not
        // swallowed.
        if addr.index == 0x1017 && addr.subindex == 0 {
            self.last_hb_ms = None;
            self.hb_arm_pending = false;
            // docs/DESIGN-DECISIONS.md 审-N2: guarding is disabled while the heartbeat
            // producer is active; CANopenNode resets the guarding toggle to
            // its base state for the whole disabled period
            // (`CO_Node_Guarding.c:178-183`). Mirror it on the enabling
            // write so no poll is needed before the reset takes effect.
            if matches!(value, Value::Uint(v) if v > 0) {
                self.guarding_toggle = false;
            }
            // NMT-03: heartbeat and node guarding are mutually exclusive;
            // CANopenNode zeroes the life timer while guarding is disabled
            // and starts from zero when it is re-enabled.
            self.guard_last_ms = None;
            // Keep `guard_timeout_active` until the next poll so `poll` can
            // emit the 0x8130 reset when guarding is disabled mid-condition.
            self.guard_rtr_since_poll = false;
            self.guard_rearm_pending = false;
            return;
        }
        // CANopenNode `OD_write_1015` restarts the inhibit window
        // (`inhibitEmTime_us = val*100; inhibitEmTimer = 0`,
        // `CO_Emergency.c:134-146`): the next frame waits the *new* window
        // from this write, not from the previous transmission (docs/DESIGN-DECISIONS.md
        // EMC-05). The write path has no clock, so arm a flag that anchors
        // the fresh window at the first `poll` after the write.
        if addr == Address::new(0x1015, 0) {
            self.emcy_inhibit_rearm = true;
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
            // Unreachable through the SDO path: `handle_with_storage` refuses
            // magic words outright while no backend is attached (0x0601_0002,
            // `docs/DESIGN-DECISIONS.md` HOST-04). Kept as a guard so a future caller of this
            // hook cannot leave `'save'`/`'load'` in the dictionary.
            self.record_config_error(crate::types::Error::Other(
                "0x1010/0x1011 magic word handled without a storage backend",
            ));
            let _ = self
                .od
                .set_value(Address::new(addr.index, addr.subindex), Value::Uint(1));
            return;
        };
        // A backend failure (e.g. a storage file that failed its write-back
        // CRC verification) is recorded for the application instead of
        // being dropped (`docs/DESIGN-DECISIONS.md` G7).
        if let Some(e) =
            crate::storage::handle_write(&backend, &mut self.od, &self.defaults, addr, &value)
        {
            self.record_config_error(e);
        }
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

    /// Runs post-boot initialisation and returns the boot-up frame when the
    /// node was still Initialising. If a Start already advanced the state,
    /// the frame cannot be emitted but PDO/consumer initialisation still runs.
    pub fn boot(&mut self) -> Option<TxFrame> {
        // docs/DESIGN-DECISIONS.md 八L17: a node without an assigned node-id serves only LSS
        // (CiA 305) — it must not emit a boot-up frame on the placeholder
        // node-1 heartbeat COB 0x701 (CANopenLinux refuses to enter normal
        // service while unconfigured).
        if self.unconfigured {
            return None;
        }
        if !self.nmt.boot() {
            // Start may already have moved the node out of Initialising. The
            // boot-up frame can no longer be emitted, but PDO (re)assembly and
            // heartbeat-consumer arming must still run.
            self.initialize_after_boot();
            return None;
        }
        self.initialize_after_boot();
        TxFrame::new(self.node_id.heartbeat_cob_id(), &[0x00]).ok()
    }

    /// Common post-boot initialisation, shared by the normal Initialising
    /// path and the Start-before-boot race.
    fn initialize_after_boot(&mut self) {
        self.guarding_toggle = false;
        self.guard_last_ms = None;
        self.guard_timeout_active = false;
        self.guard_rtr_since_poll = false;
        self.guard_rearm_pending = false;
        if let Err(e) = self.configure_pdos_from_od() {
            self.record_config_error(e);
        }
        self.resync_hb_consumers_from_od();
        self.last_hb_ms = None;
        self.hb_arm_pending = true;
        if self.nmt_control.startup_to_operational {
            self.nmt.apply(NmtCommand::StartRemoteNode);
            self.last_hb_ms = None;
            self.hb_arm_pending = false;
        }
    }

    /// Applies the session-edge side effects of an NMT transition. Shared by
    /// incoming NMT commands and the optional NMTcontrol state machine.
    fn after_nmt_transition(&mut self, prev_state: NmtState, changed: bool) {
        if changed && self.nmt.state() == NmtState::Operational {
            for slot in &mut self.tpdos {
                if slot.config.is_valid()
                    && matches!(
                        slot.tt,
                        TransmissionType::EventDriven(_) | TransmissionType::SyncAcyclic
                    )
                {
                    slot.pending_event = true;
                }
            }
        }
        if changed {
            self.last_hb_ms = None;
            self.hb_arm_pending = false;
        }
        if self.nmt.state() != NmtState::Operational {
            for slot in &mut self.tpdos {
                slot.syncs_seen = match slot.tt {
                    TransmissionType::SyncCyclic(n) => {
                        let n = u32::from(n.max(1));
                        n - (n / 2 + 1)
                    }
                    _ => 0,
                };
                slot.phase_locked = false;
                slot.ever_sent = false;
                slot.pending_event = false;
                slot.last_tx_ms = None;
                slot.tx_seen = false;
                slot.sync_received = false;
                slot.sync_sample = None;
            }
            for slot in &mut self.rpdos {
                slot.sync_pending = None;
                slot.rx_seen = false;
                slot.last_rx_ms = None;
                slot.rx_len = RxLenState::NoError;
                slot.reported = false;
            }
            // EMC-20: clear both the node latch and the producer's active
            // condition; queue the reset frame for the next poll.
            if self.rpdo_emcy_active {
                self.rpdo_emcy_active = false;
                let info = emcy_vendor_info(EM_BIT_RPDO_TIME_OUT, self.rpdo_elapsed_us);
                if let Some(payload) = self.emcy.clear_with_info(&mut self.od, 0x8250, info) {
                    if self.pending_emcy.len() >= MAX_PENDING_EMCY {
                        self.emcy.set_overflow(&mut self.od, true);
                    } else {
                        self.pending_emcy.push_back(payload);
                    }
                }
            }
        }
        let prev_supervised =
            matches!(prev_state, NmtState::PreOperational | NmtState::Operational);
        let next_supervised = matches!(
            self.nmt.state(),
            NmtState::PreOperational | NmtState::Operational
        );
        if changed && prev_supervised != next_supervised {
            self.hb_consumer.reset_states();
            self.hb_rx_queue.clear();
        }
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
                // LSS frames are always exactly 8 bytes on the wire (CiA 305);
                // zero-padding a short frame could turn an abnormal/stray
                // frame into a valid-looking command (e.g. a fastscan probe) —
                // reject anything but DLC == 8, like CANopenNode
                // (`if ((DLC == 8U) && …)`, `CO_LSSslave.c:45`).
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
                // caller for transmission (docs/DESIGN-DECISIONS.md 十M22).
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
            // docs/DESIGN-DECISIONS.md M4: 0x7E5 is exclusively LSS. A frame on it that the
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
            // frame comes back through the same call (docs/DESIGN-DECISIONS.md H2).
            return self.on_nmt(data).into_iter().collect();
        }
        if cob_id == sdo_rx
            && matches!(
                self.nmt_state(),
                NmtState::PreOperational | NmtState::Operational
            )
        {
            let mut out = Vec::new();
            // docs/DESIGN-DECISIONS.md H2: SDO requests are always exactly 8 bytes on the
            // wire (CiA 301). Zero-padding shorter frames silently fed a
            // malformed expedited request to the server (no abort, no
            // error); drop them instead — same strictness the LSS path
            // already applies (it requires `len == 8`).
            let Some(req) = sdo_payload(data) else {
                return Vec::new();
            };
            // docs/DESIGN-DECISIONS.md HOST-04: the SDO server must know whether a storage
            // backend exists, otherwise it confirms a `'save'`/`'load'`
            // magic word that nothing will act on (and leaves the magic word
            // in the dictionary). CANopenLinux answers `ODR_READONLY` there
            // (`storage/CO_storage.c:38-40`).
            let storage_enabled = self.storage.is_some();
            for payload in self
                .sdo
                .handle_with_storage(&mut self.od, &req, storage_enabled)
            {
                if let Ok(frame) = TxFrame::new(self.node_id.sdo_tx_cob_id(), &payload) {
                    out.push(frame);
                }
                // Live-reconfiguration: a write into the PDO parameter range
                // rebuilds just that slot, leaving every other slot's runtime
                // state untouched (`docs/DESIGN-DECISIONS.md` 审-P1; docs/IMPLEMENTATION.md
                // §7.3).
                for (addr, value) in self.sdo.take_last_written() {
                    self.reconfigure_pdo(addr);
                    self.on_parameter_write(addr, value);
                }
            }
            return out;
        }
        if cob_id == hb {
            // Frames on our own heartbeat COB-ID are normally our own
            // (echoed) traffic — RTR-based guarding is handled via
            // [`Self::on_remote_request`]. An RPDO slot claiming that
            // COB-ID takes precedence though: CANopenNode registers RX
            // filters per COB (`CO_HBconsumer.c:203-213`), nothing globally
            // reserves 0x700+id (docs/DESIGN-DECISIONS.md N7).
            let claimed_by_rpdo = self
                .rpdos
                .iter()
                .any(|s| s.config.is_valid() && s.config.cob_id() == cob_id);
            if !claimed_by_rpdo {
                return Vec::new();
            }
        }
        // Foreign heartbeats feed the heartbeat consumer (§10.3 / P1-11);
        // timestamps are applied when the host next polls (sans-I/O).
        // Only *supervised* nodes' frames are claimed here: a blanket
        // 0x700..=0x77F interception would silently swallow custom
        // RPDO/SYNC COB-IDs configured inside that range (`docs/DESIGN-DECISIONS.md` P3,
        // cf. CANopenNode's per-entry rx filter and python-canopen's
        // per-node subscription).
        // docs/DESIGN-DECISIONS.md 八L13: a heartbeat frame is exactly 1 byte on the wire
        // (CANopenNode `CO_HBconsumer.c:43` only latches DLC == 1); longer
        // frames are malformed and must not be consumed as heartbeats.
        if (0x700..=0x77F).contains(&cob_id) && data.len() == 1 {
            if let Ok(node) = NodeId::try_from((cob_id - 0x700) as u8) {
                // docs/DESIGN-DECISIONS.md NMT-05: a configured RPDO claiming the same COB-ID
                // wins, exactly like the own-heartbeat and foreign-EMCY
                // interceptions above — CANopenNode registers one rx filter
                // per COB-ID (`CO_HBconsumer.c:202-213`), so supervision must
                // not swallow a frame the application mapped to an RPDO. The
                // frame then falls through to the PDO matcher below.
                let claimed_by_rpdo = self
                    .rpdos
                    .iter()
                    .any(|s| s.config.is_valid() && s.config.cob_id() == cob_id);
                if !claimed_by_rpdo && self.hb_consumer.supervises(node) {
                    if self.hb_rx_queue.len() >= MAX_RX_QUEUE {
                        self.hb_rx_queue.pop_front();
                    }
                    self.hb_rx_queue.push_back((node, data.to_vec()));
                    return Vec::new();
                }
            }
        }
        // Foreign EMCY frames (0x80+n, n≠self) feed the optional consumer
        // (`docs/DESIGN-DECISIONS.md` M3): only when an application installed one via
        // [`Self::set_emcy_consumer`]. The interception is deliberately
        // **narrow** (`docs/DESIGN-DECISIONS.md` H2): a COB-ID claimed by a configured RPDO
        // slot belongs to that RPDO, not to a loosely decoded EMCY — the
        // frame must fall through to the PDO matcher below. Only *unclaimed*
        // standard EMCY COBs (0x80+n) are consumed.
        // SYNC COB-ID（含 0x1005 自定义 0x81..=0xFF）必须排除，否则 EMCY 的
        // 宽松解码会吞掉 SYNC 帧，使其永远到不了 `on_sync`（docs/DESIGN-DECISIONS.md P2-1）。
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
        // (docs/DESIGN-DECISIONS.md P3-21).
        //
        // docs/DESIGN-DECISIONS.md SYN-09 ①: CANopenNode only consumes TIME in
        // PreOperational/Operational and clears the latch otherwise
        // (`CO_TIME.c:155-172`, `NMTisPreOrOperational && isConsumer`);
        // ② the interception mirrors the heartbeat/EMCY ones and yields to a
        // configured RPDO that claims 0x100 — a 6-byte RPDO payload on that
        // COB-ID is data, not a time stamp (same-file policy consistency,
        // docs/DESIGN-DECISIONS.md NMT-05 / P2-1).
        let (time_cob_id, time_consumer, _) = self.time_config();
        if cob_id == time_cob_id
            && time_consumer
            && data.len() == 6
            && matches!(
                self.nmt.state(),
                NmtState::PreOperational | NmtState::Operational
            )
            && !self
                .rpdos
                .iter()
                .any(|s| s.config.is_valid() && s.config.cob_id() == cob_id)
        {
            let mut buf = [0u8; 6];
            buf.copy_from_slice(data);
            if self.time_rx_queue.len() >= MAX_RX_QUEUE {
                self.time_rx_queue.pop_front();
            }
            self.time_rx_queue.push_back(buf);
            return Vec::new();
        }
        if let NmtState::Operational = self.nmt_state() {
            // Self-echo suppression (docs/DESIGN-DECISIONS.md P3-22, python-canopen
            // `pdo/base.py:331-342` idea): a frame on a COB-ID this node
            // itself transmits as a TPDO is our own traffic looped back —
            // never apply it as RPDO data. Strictly, python-canopen only
            // drops such a frame while this node is the one currently sending
            // it (`self._task is not None`, `pdo/base.py:331-342`), so a peer
            // transmitting on a COB-ID we also use would be swallowed here;
            // the stack is deliberately the stricter of the two because the
            // configuration itself is invalid (docs/DESIGN-DECISIONS.md PDO-08 / P3-22: the
            // decision is kept and the divergence recorded in
            // docs/EXTENSIONS.md §2.6).
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
                    // docs/DESIGN-DECISIONS.md 十L5: CANopenNode `CO_PDO.c:442-475` neither
                    // stores nor applies an under-long RPDO (DLC <
                    // dataLength) — it only moves `receiveError` to
                    // `CO_RPDO_RX_SHORT` when no error is latched yet.
                    // docs/DESIGN-DECISIONS.md B6: a short frame is not traffic either, so the
                    // deadman must not be re-armed here.
                    if self.rpdos[idx].rx_len == RxLenState::NoError {
                        self.rpdos[idx].rx_len = RxLenState::PendingShort;
                    }
                    // docs/DESIGN-DECISIONS.md PDO-25: the code (0x8210) is chosen and put on
                    // the bus by the *process gate*
                    // (`PDO->valid && NMTisOperational && (syncWas ||
                    // !synchronous)`, `CO_PDO.c:717-728`), not by the receive
                    // callback. A synchronous RPDO therefore announces the
                    // error at the next SYNC; a non-synchronous one enters the
                    // gate on the next main-loop pass, which `on_frame` models.
                    if !self.rpdos[idx].tt.is_synchronous() {
                        out.extend(self.report_rx_len(idx));
                    }
                    return out;
                }
                // docs/DESIGN-DECISIONS.md PDO-25: an exactly-sized frame acks a latched error
                // (`err == CO_RPDO_RX_ACK_ERROR → CO_RPDO_RX_OK`,
                // `CO_PDO.c:443-447`); anything else leaves the latch alone,
                // which is why "short frame, then over-long frame" produces no
                // further frames at all.
                if data.len() == declared && self.rpdos[idx].rx_len == RxLenState::Latched {
                    self.rpdos[idx].rx_len = RxLenState::PendingOk;
                } else if data.len() > declared && self.rpdos[idx].rx_len == RxLenState::NoError {
                    // docs/DESIGN-DECISIONS.md 八L2: an over-long frame *is* accepted and
                    // applied (`DLC >= dataLength` copies the data), only
                    // CO_EMC_PDO_LENGTH_EXC (0x8220) is reported.
                    self.rpdos[idx].rx_len = RxLenState::PendingLong;
                }
                if self.rpdos[idx].tt.is_synchronous() {
                    let mut buf = [0u8; 8];
                    let n = data.len().min(8);
                    buf[..n].copy_from_slice(&data[..n]);
                    self.rpdos[idx].sync_pending = Some((buf, n));
                } else {
                    // Applied right here, so the deadman starts counting now
                    // (docs/DESIGN-DECISIONS.md PDO-07: CANopenNode re-arms `timeoutTimer` only
                    // inside the apply loop, `CO_PDO.c:824-828`).
                    self.rpdos[idx].rx_seen = true;
                    if let Err(e) = pdo::unpack(&self.rpdos[idx].mapping, &mut self.od, data) {
                        self.record_config_error(e);
                    }
                    // Non-synchronous slots enter the gate on the spot.
                    out.extend(self.report_rx_len(idx));
                }
                return out;
            }
        }
        Vec::new()
    }

    /// Runs one RPDO length-condition report through the EMCY layer, the
    /// `receiveError > CO_RPDO_RX_ACK` block of CANopenNode's
    /// `CO_RPDO_process` (`CO_PDO.c:721-728`). See [`RxLenState`] for the
    /// state mapping and docs/DESIGN-DECISIONS.md PDO-25.
    ///
    /// Only the *rising* edge leaves the bus: `PendingShort`/`PendingLong`
    /// raise 0x8210/0x8220 and latch as [`RxLenState::Latched`], so a
    /// differently-sized follow-up frame changes nothing (the reference's
    /// `CO_error` returns early for an unchanged bit,
    /// `CO_Emergency.c:658-666`), and `PendingOk` — set only by an
    /// exactly-sized frame after a latched error — resets the condition.
    fn report_rx_len(&mut self, idx: usize) -> Vec<TxFrame> {
        let declared = self.rpdos[idx].mapping.byte_len() as u32;
        // docs/DESIGN-DECISIONS.md EMC-07: the reference reports the *declared* length as the
        // info value (`CO_error(..., code, PDO->dataLength)`, `CO_PDO.c:725`).
        let info = emcy_vendor_info(EM_BIT_RPDO_WRONG_LENGTH, declared);
        let mut out = Vec::new();
        match self.rpdos[idx].rx_len {
            RxLenState::NoError | RxLenState::Latched => {}
            RxLenState::PendingShort => {
                if let Some(frame) =
                    self.raise_emergency(0x8210, crate::emcy::err_reg::COMMUNICATION, info)
                {
                    out.push(frame);
                }
                self.rpdos[idx].rx_len = RxLenState::Latched;
            }
            RxLenState::PendingLong => {
                if let Some(frame) =
                    self.raise_emergency(0x8220, crate::emcy::err_reg::COMMUNICATION, info)
                {
                    out.push(frame);
                }
                self.rpdos[idx].rx_len = RxLenState::Latched;
            }
            RxLenState::PendingOk => {
                // docs/DESIGN-DECISIONS.md 十H5: only one condition can be latched at a time, so
                // at most one reset frame leaves the bus (the reference resets
                // the single `CO_EM_RPDO_WRONG_LENGTH` bit).
                if let Some(frame) = self.clear_emergency(0x8210) {
                    out.push(frame);
                }
                if let Some(frame) = self.clear_emergency(0x8220) {
                    out.push(frame);
                }
                self.rpdos[idx].rx_len = RxLenState::NoError;
            }
        }
        out
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
    /// *clear* (RTR allowed, CiA 301; docs/DESIGN-DECISIONS.md 八H1) — see §7.4.
    pub fn on_remote_request(&mut self, cob_id: u16) -> Option<TxFrame> {
        let hb = self.node_id.heartbeat_cob_id();
        // Node guarding and heartbeat are mutually exclusive error-control
        // protocols: CANopenNode passes `slaveDisable = 0x1017 > 0` into the
        // guarding slave, which then refuses to answer RTRs (`CANopen.c:1384`,
        // `CO_Node_Guarding.c:178-183`; docs/DESIGN-DECISIONS.md B3).
        if cob_id == hb
            && self.nmt.state() != NmtState::Initialising
            && self.producer_heartbeat_ms() == 0
        {
            // First response carries toggle 0 (CiA 301 §7.3.1).
            let payload = crate::nmt::encode_node_guard(self.nmt.state(), self.guarding_toggle);
            self.guarding_toggle = !self.guarding_toggle;
            // NMT-03: an RTR re-anchors the life timer when `poll` next runs.
            self.guard_rtr_since_poll = true;
            // docs/DESIGN-DECISIONS.md L6: a constant 1-byte payload cannot fail `new`.
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
                        // docs/DESIGN-DECISIONS.md M4: 类型 252（RtrOnlySync）的 RTR 应答必须
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
        self.sync_seen_since_poll = false;
        self.last_sync_ms = None;
        self.sync_timeout_active = false;
        // docs/DESIGN-DECISIONS.md SYN-12: a communication reset re-initialises the SYNC
        // consumer wholesale — CANopenNode re-runs `CO_SYNC_init`, which
        // memsets the whole `CO_SYNC_t` (`CO_SYNC.c:193`) and therefore also
        // zeroes `timeoutError`. Keeping `sync_ever_seen` would let the node
        // report a 0x8100 timeout for SYNCs that were never observed in this
        // epoch.
        self.sync_ever_seen = false;
        self.rpdo_timeouts.clear();
        self.rpdo_emcy_active = false;
        // The heartbeat-consumer EMCY latch belongs to the previous epoch as
        // well (the EMCY producer below is re-created empty) — leaving it set
        // would suppress every later 0x8130 report (`docs/DESIGN-DECISIONS.md` 审-N1).
        self.hb_emcy_active = false;
        // Stale heartbeats/events from the previous epoch must not
        // surface in the new one (docs/DESIGN-DECISIONS.md P3-16).
        self.hb_rx_queue.clear();
        self.pending_hb_events.clear();
        self.time_rx_queue.clear();
        // docs/DESIGN-DECISIONS.md 十M2: a communication reset re-initialises the
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
        self.guard_last_ms = None;
        self.guard_timeout_active = false;
        self.guard_rtr_since_poll = false;
        self.guard_rearm_pending = false;
        // Power-on semantics: re-apply persisted parameters. A corrupt store
        // (bad CRC / damaged line detected by the backend) is recorded for
        // the application rather than silently ignored (`docs/DESIGN-DECISIONS.md` G7); good
        // entries are still applied by the backend.
        if let Some(backend) = self.storage.clone() {
            if let Err(e) = backend.load(&mut self.od) {
                self.record_config_error(e);
            }
        }
        // docs/DESIGN-DECISIONS.md M3: a reset abandons any in-progress LSS session
        // (fastscan/switch/bit-timing) and returns the slave to
        // Waiting; pending values survive for the application to
        // consume (adopt_pending / take_pending_bit_timing /
        // take_store_requested).
        if let Some(lss) = &mut self.lss {
            lss.reset();
        }
        // docs/DESIGN-DECISIONS.md H2: the reset leaves Initialising and re-boots —
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
    /// caller must transmit it like a power-on boot-up (docs/DESIGN-DECISIONS.md H2).
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
            // (`docs/DESIGN-DECISIONS.md` R10-P3, documented).
            NmtCommand::ResetNode | NmtCommand::ResetCommunication => {
                self.nmt.apply(command);
                self.communication_reset()
            }
            _ => {
                // Honour an Initialising Start before applying it, so the
                // boot-up frame and consumer/PDO initialization are not lost.
                let boot_frame = if self.nmt.state() == NmtState::Initialising {
                    self.boot()
                } else {
                    None
                };
                let prev_state = self.nmt.state();
                let changed = self.nmt.apply(command);
                self.after_nmt_transition(prev_state, changed);
                boot_frame
            }
        }
    }

    /// Reads the CiA 301 TIME configuration from 0x1012:
    /// `(cob_id, consumer_enabled, producer_enabled)`.
    fn time_config(&self) -> (u16, bool, bool) {
        let raw = match self.od.read(Address::new(0x1012, 0)) {
            Ok(Value::Uint(v)) => v as u32,
            _ => crate::sync_time::TIME_COB_ID.into(),
        };
        (
            (raw & 0x7FF) as u16,
            raw & (1 << 31) != 0,
            raw & (1 << 30) != 0,
        )
    }

    /// Builds a TIME producer frame for `unix_ms` if 0x1012 has the producer
    /// bit set. The application owns the transmission cadence (CiA 301 does
    /// not define one); this method only enforces the configured COB-ID and
    /// the six-byte payload.
    ///
    /// # Errors
    /// [`crate::types::Error::ValueOutOfRange`] when the timestamp is outside
    /// the TIME epoch, or a frame-construction error.
    pub fn time_frame(&self, unix_ms: i64) -> crate::types::Result<Option<TxFrame>> {
        let (cob_id, _, producer) = self.time_config();
        if !producer {
            return Ok(None);
        }
        let stamp = crate::sync_time::TimeStamp::from_unix_ms(unix_ms)?;
        let payload = stamp.encode();
        TxFrame::new(cob_id, &payload[..6]).map(Some)
    }

    fn is_sync(&self, cob_id: u16) -> bool {
        // CiA 301 默认 SYNC COB-ID 为 0x80；OD 缺 0x1005 时使用该默认
        // 回退（docs/DESIGN-DECISIONS.md B3-1）。COB-ID SYNC 位布局（CANopenNode
        // `CO_SYNC.c:87-104`）：bit30 是 gen 标志（本设备为 SYNC 生产者），
        // 不影响消费/过滤——此前误当作「29 位扩展帧」拒绝，使声明生产者的
        // 节点对 SYNC 完全失明（docs/DESIGN-DECISIONS.md 八M12）；低 11 位为 CAN-ID。
        //
        // docs/DESIGN-DECISIONS.md SYN-02: bit31 与保留位 29..11 **在消费侧一律忽略**，只取
        // 低 11 位比对——与参考的 `CO_SYNC_init` 一致（脏值下仍按低 11 位
        // 监听 RX，只在写时拒，`CO_SYNC.c:87-95`）。此前的严格判据使
        // EDS 预载的脏 0x1005（不经写守卫、无 abort 无 config-error）让节点
        // 对 SYNC 完全失聪。写时守卫仍在 `sdo::server::od_write_guard`
        // （十M4 + SYN-11）负责拒绝脏值。
        let raw = match self.od.read(Address::new(0x1005, 0)) {
            Ok(Value::Uint(v)) => v,
            _ => 0x80,
        };
        (raw & 0x7FF) as u16 == cob_id
    }

    /// SYNC consumer: advances synchronous TPDOs.
    fn on_sync(&mut self, data: &[u8]) -> Vec<TxFrame> {
        // docs/DESIGN-DECISIONS.md 十L4: the width check and its 0x8240 report also apply in
        // PreOperational (CANopenNode `CO_SYNC.c:386-392` reports in
        // PreOp and Op); only *driving* PDOs is Operational-only.
        if !matches!(
            self.nmt.state(),
            NmtState::PreOperational | NmtState::Operational
        ) {
            return Vec::new();
        }
        // docs/DESIGN-DECISIONS.md 八M11: the node's own 0x1019 counter-overflow value dictates
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
            // docs/DESIGN-DECISIONS.md EMC-07: CANopenNode encodes the offending DLC in the
            // info value, `DLC | 0x40` for the counterless case and
            // `DLC | 0x80` for the counter case (`CO_SYNC.c:38-51`).
            let dlc = data.len().min(0xFF) as u8;
            let marker = if counter_overflow == 0 { 0x40 } else { 0x80 };
            if let Some(frame) = self.raise_emergency(
                0x8240,
                crate::emcy::err_reg::COMMUNICATION,
                emcy_vendor_info(EM_BIT_SYNC_LENGTH, u32::from(dlc | marker)),
            ) {
                return vec![frame];
            }
            return Vec::new();
        }
        // A well-formed SYNC: `poll` anchors the SYNC-producer supervision
        // epoch to it (docs/DESIGN-DECISIONS.md SYN-01 — `on_frame` carries no clock).
        self.sync_seen_since_poll = true;
        if self.nmt.state() != NmtState::Operational {
            return Vec::new();
        }
        // Payload validation FIRST (docs/DESIGN-DECISIONS.md P3-2, see git history): a frame whose
        // COB-ID matches SYNC but whose payload is malformed (>1 byte) is
        // not a SYNC and must not release the buffered RPDO cycle either.
        let counter = match crate::sync_time::decode_sync(data) {
            Some(c) => c,
            None => return Vec::new(),
        };
        // Synchronous RPDO double buffering (docs/DESIGN-DECISIONS.md P2-4): data received
        // during the previous cycle becomes visible exactly at this SYNC
        // boundary — CANopenNode `CO_PDO.c:717-760` applies the buffered
        // frame when `syncWas` fires.
        let mut config_errors = Vec::new();
        let mut applied: Vec<usize> = Vec::new();
        for (idx, slot) in self.rpdos.iter_mut().enumerate() {
            if let Some((buf, len)) = slot.sync_pending.take() {
                // Field-disjoint borrows: `od` is mutated while `rpdos` is
                // iterated; errors are recorded after the loop ends.
                if let Err(e) = pdo::unpack(&slot.mapping, &mut self.od, &buf[..len]) {
                    config_errors.push(e);
                }
                // docs/DESIGN-DECISIONS.md PDO-07: CANopenNode re-arms `timeoutTimer` inside the
                // apply loop (`CO_PDO.c:824-828`, guarded by `rpdoReceived`) —
                // not when the frame was received. A synchronous RPDO's
                // deadman therefore starts at the SYNC that makes the data
                // visible, which can be almost one period later than before.
                slot.rx_seen = true;
                applied.push(idx);
            }
        }
        for e in config_errors {
            self.record_config_error(e);
        }
        // docs/DESIGN-DECISIONS.md PDO-25: the length condition of a *synchronous* RPDO is
        // announced here, inside the reference's process gate
        // (`PDO->valid && NMTisOperational && syncWas`, `CO_PDO.c:717-728`),
        // i.e. at the next SYNC — not at reception. Reported per slot so the
        // frames precede the next TPDO batch, and after the apply pass because
        // the gate clears the flags *before* the reference reports the code.
        let mut emcy_frames = Vec::new();
        for idx in 0..self.rpdos.len() {
            let synchronous = self.rpdos[idx].config.is_valid()
                && self.rpdos[idx].tt.is_synchronous()
                && (applied.contains(&idx) || self.rpdos[idx].rx_len != RxLenState::NoError);
            if synchronous {
                emcy_frames.extend(self.report_rx_len(idx));
            }
        }
        // CiA 301 / CANopenNode `CO_PDO.c`: sync-start phase-locking only
        // applies when BOTH a start value and a global counter overflow
        // (0x1019) are configured (`docs/DESIGN-DECISIONS.md` P2-5); `counter_overflow` was
        // read and validated at the top of this function.
        // docs/DESIGN-DECISIONS.md M2: any received SYNC arms the type-252 (RtrOnlySync)
        // TPDOs of this session — their RTR payload is the last SYNC
        // sample, so before the first SYNC there is nothing valid to
        // answer with. Applied to every valid slot (synchronous ones get
        // it too; the flag only gates RtrOnlySync responses).
        let mut sync_pack_errors: Vec<crate::types::Error> = Vec::new();
        for slot in &mut self.tpdos {
            if !slot.config.is_valid() {
                continue;
            }
            slot.sync_received = true;
            // docs/DESIGN-DECISIONS.md M4: 类型 252（RtrOnlySync）PDO 的 RTR 应答须在"最后
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
                    Ok(None) => None,
                    // docs/DESIGN-DECISIONS.md PDO-05: a failed pack is a configuration
                    // problem and must not vanish into the catch-all — the
                    // tt=253 branch below already reports it (M1).
                    Err(e) => {
                        sync_pack_errors.push(e);
                        None
                    }
                };
            }
        }
        for e in sync_pack_errors {
            // docs/DESIGN-DECISIONS.md PDO-05: report the failed RtrOnlySync snapshot instead
            // of swallowing it (the bounded config-error ring is the
            // project-wide channel for assembly/pack failures, M1).
            self.record_config_error(e);
        }
        // docs/DESIGN-DECISIONS.md PDO-25: the length-condition frames leave the bus ahead of
        // the PDOs driven by this SYNC.
        let mut out = emcy_frames;
        let mut tpdo_errors = Vec::new();
        for slot in &mut self.tpdos {
            if !slot.config.is_valid() || !slot.tt.is_synchronous() {
                continue;
            }
            // Sync start value (S14): wait for the matching global counter
            // once, then keep the phase lock for the whole Operational
            // session (CANopenNode treats the wait as a one-shot state).
            // Adjudicated limitation (`docs/DESIGN-DECISIONS.md` H2): when 0x1019 ≠ 0 but the
            // incoming SYNC carries no counter byte, the phase can never be
            // locked and the sync-start TPDO stays silent — the counterless
            // SYNC is a producer misconfiguration (0x1019 is the producer's
            // own counter-overflow declaration), so silence is deliberate.
            let mut lock_now = false;
            // docs/DESIGN-DECISIONS.md 十M3: the sync-start phase lock belongs to cyclic
            // transmission types only (CANopenNode `CO_PDO.c:1305-1329`
            // keeps `syncStartValue` inside the cyclic branch); applying it
            // to SyncAcyclic (tt=0) produced unsolicited/duplicated frames.
            if counter_overflow != 0 && matches!(slot.tt, TransmissionType::SyncCyclic(_)) {
                // docs/DESIGN-DECISIONS.md PDO-26: a slot that has already transmitted is
                // past the reference's one-shot seeding, so a later 0x1019
                // write must not park it in the sync-start wait (which used
                // to silence a running cyclic TPDO for up to 240 SYNCs).
                if let Some(start) =
                    nonzero(slot.config.sync_start_value).filter(|_| !slot.ever_sent)
                {
                    match counter {
                        Some(global) if global == start && !slot.phase_locked => {
                            slot.phase_locked = true;
                            // docs/DESIGN-DECISIONS.md 八L3: CANopenNode `CO_PDO.c:1325-1329`
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
            // docs/DESIGN-DECISIONS.md PDO-04: a latched tt=0 request is consumed **after** a
            // successful pack — CANopenNode clears `sendRequest` only once
            // its length check passed (`CO_TPDOsend`, `CO_PDO.c:1213-1221`);
            // clearing it up front lost the trigger whenever the pack failed.
            let mut consume_pending_event = false;
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
                        consume_pending_event = p;
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
                    Ok(Some(frame)) => {
                        if consume_pending_event {
                            slot.pending_event = false;
                        }
                        // Stamp at the next `poll`; re-arms the tt=0 event
                        // timer (docs/DESIGN-DECISIONS.md N1).
                        slot.tx_seen = true;
                        // docs/DESIGN-DECISIONS.md PDO-26: a slot that has produced a frame is
                        // past the reference's one-shot SYNC-start seeding
                        // (`CO_PDO.c:1315-1336`), so a later 0x1019 write must
                        // not park it in the start-value wait.
                        slot.ever_sent = true;
                        out.push(frame);
                    }
                    Ok(None) => {
                        if consume_pending_event {
                            slot.pending_event = false;
                        }
                    }
                    // docs/DESIGN-DECISIONS.md M1: a due TPDO whose mapping cannot be packed
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

    /// Applies the optional CANopenNode-compatible NMT control flags
    /// (docs/DESIGN-DECISIONS.md NMT-06). Called from [`Node::poll`] after heartbeat-consumer
    /// supervision so the error inputs are current.
    fn apply_nmt_control(&mut self, out: &mut Vec<TxFrame>) {
        if !self.nmt_control.err_on_err_reg
            && !self.nmt_control.err_on_busoff_hb
            && !self.nmt_control.startup_to_operational
            && !self.nmt_control.err_free_to_operational
        {
            return;
        }
        let error_register = match self.od.read(Address::new(0x1001, 0)) {
            Ok(Value::Uint(v)) => v as u8,
            _ => 0,
        };
        let mut error = false;
        if self.nmt_control.err_on_err_reg && (error_register & self.nmt_control.err_reg_mask) != 0
        {
            error = true;
        }
        if self.nmt_control.err_on_busoff_hb && (self.can_status.bus_off || self.hb_emcy_active) {
            error = true;
        }
        let state = self.nmt.state();
        if state == NmtState::Operational && error {
            let command = if self.nmt_control.err_to_stopped {
                NmtCommand::StopRemoteNode
            } else {
                NmtCommand::EnterPreOperational
            };
            let prev_state = self.nmt.state();
            if self.nmt.apply(command) {
                self.after_nmt_transition(prev_state, true);
            }
        } else if state == NmtState::PreOperational
            && self.nmt_control.err_free_to_operational
            && !error
        {
            let prev_state = self.nmt.state();
            if self.nmt.apply(NmtCommand::StartRemoteNode) {
                self.after_nmt_transition(prev_state, true);
            }
        }
        // A state transition writes its heartbeat through the normal producer
        // path on the next poll; `out` is reserved for future immediate-frame
        // semantics and keeps the helper's signature stable.
        let _ = out;
    }

    /// Periodic maintenance: heartbeat production and event-driven TPDO
    /// timers (event timer + inhibit time). Call regularly with monotonic ms.
    pub fn poll(&mut self, now_ms: u64) -> Vec<TxFrame> {
        let mut out = Vec::new();

        // CiA 303-3 LED state machine (docs/DESIGN-DECISIONS.md LED-01): advance it with the
        // poll clock and a snapshot of the node state. The host injects the
        // controller-only inputs via `set_can_status`; everything else is
        // derived here.
        let led_elapsed = self
            .last_led_ms
            .map_or(0, |last| now_ms.saturating_sub(last));
        self.last_led_ms = Some(now_ms);
        let error_register = match self.od.read(Address::new(0x1001, 0)) {
            Ok(Value::Uint(v)) => v != 0,
            _ => false,
        };
        let led_inputs = crate::leds::LedInputs {
            nmt: self.nmt.state(),
            lss_configuring: self
                .lss
                .as_ref()
                .is_some_and(|lss| lss.state() == crate::lss::LssState::Configuration),
            bus_off: self.can_status.bus_off,
            bus_warn: self.can_status.bus_warn,
            rpdo_timeout: self.rpdo_emcy_active,
            sync_timeout: self.sync_timeout_active,
            hb_consumer_error: self.hb_emcy_active,
            other_error: error_register,
            firmware_download: self.firmware_download,
        };
        let _ = self.leds.process(led_elapsed, &led_inputs);

        // Heartbeat consumption (§10.3, docs/DESIGN-DECISIONS.md P1-11): frames received
        // since the last poll get their timestamps here; timeout events
        // surface via [`Node::take_hb_events`].
        //
        // docs/DESIGN-DECISIONS.md 七F3: supervision runs only while this node is itself in
        // PreOperational/Operational — CANopenNode gates
        // `CO_HBconsumer_process` on `NMTisPreOrOperational && prev`
        // (`CO_HBconsumer.c:304-311`). While Stopped/Initialising the cached
        // frames are dropped (the edge handler cleared the tracking state,
        // so a stale frame from the old epoch must not restart supervision).
        if matches!(
            self.nmt.state(),
            NmtState::PreOperational | NmtState::Operational
        ) {
            let mut hb_lost: Option<NodeId> = None;
            while let Some((node, data)) = self.hb_rx_queue.pop_front() {
                for event in self.hb_consumer.on_heartbeat(node, &data, now_ms) {
                    self.push_hb_event(event);
                }
            }
            for event in self.hb_consumer.poll(now_ms) {
                if let crate::nmt::HbEvent::Lost { node } = event {
                    if hb_lost.is_none() {
                        hb_lost = Some(node);
                    }
                }
                self.push_hb_event(event);
            }
            // docs/DESIGN-DECISIONS.md 审-N1: CANopenNode's consumer supervision reports
            // CO_EMC_HEARTBEAT (0x8130) once when a monitored node times out,
            // and clears the condition when every monitored node is active
            // again (`CO_HBconsumer.c:353-367,423-428`). The error bit is
            // shared, so further timeouts while it is already raised produce
            // no duplicate frame (`CO_Emergency.c:658-663`); the single
            // `hb_emcy_active` latch mirrors that.
            //
            // docs/DESIGN-DECISIONS.md EMC-16: the manufacturer field follows the reference
            // layout `[errorBit, infoLE(4)]` — `CO_HBconsumer.c:364` passes
            // `CO_EM_HEARTBEAT_CONSUMER` (0x1B) as the error bit and the
            // monitoring-table index `i` as the infoCode, so the master can
            // tell *which* 0x1016 entry died.
            if let Some(node) = hb_lost.filter(|_| !self.hb_emcy_active) {
                self.hb_emcy_active = true;
                let index = self.hb_consumer.entry_index(node).unwrap_or(0);
                let info = emcy_vendor_info(EM_BIT_HEARTBEAT_CONSUMER, u32::from(index));
                if let Some(frame) =
                    self.raise_emergency(0x8130, crate::emcy::err_reg::COMMUNICATION, info)
                {
                    out.push(frame);
                }
            } else if self.hb_emcy_active
                && (self.hb_consumer.all_active() || self.hb_consumer.is_empty())
            {
                self.hb_emcy_active = false;
                if let Some(frame) = self.clear_emergency(0x8130) {
                    out.push(frame);
                }
            }
        } else {
            self.hb_rx_queue.clear();
        }

        // NMT-06: error-register/bus-off driven state transitions.
        self.apply_nmt_control(&mut out);

        // SYNC producer supervision (docs/DESIGN-DECISIONS.md SYN-01): CANopenNode's SYNC
        // consumer reports `CO_EM_SYNC_TIME_OUT` once no SYNC arrived within
        // 1.5× the 0x1006 period, and clears it again on the next SYNC
        // (`CO_SYNC.c:334-363,400-406`). `on_frame` receives no clock, so a
        // seen SYNC is anchored to the first poll that observes it — the same
        // pattern the heartbeat producer uses (八M4).
        let sync_period = match self.od.read(Address::new(0x1006, 0)) {
            Ok(Value::Uint(v)) => v,
            _ => 0,
        };
        // docs/DESIGN-DECISIONS.md SYN-08: the reference only runs the silence supervision on
        // a *consumer* node (`CO_SYNC.c:335-351`) and only after the first
        // SYNC was seen (`timeoutError` is set by `syncStatus == RX_TX`,
        // `:401-406`) — a master that starts producing late must not raise a
        // phantom 0x8100.
        let sync_producer = matches!(
            self.od.read(Address::new(0x1005, 0)),
            Ok(Value::Uint(v)) if v & (1 << 30) != 0
        );
        if matches!(
            self.nmt.state(),
            NmtState::PreOperational | NmtState::Operational
        ) {
            if self.sync_seen_since_poll {
                self.sync_seen_since_poll = false;
                self.last_sync_ms = Some(now_ms);
                self.sync_ever_seen = true;
                if self.sync_timeout_active {
                    self.sync_timeout_active = false;
                    if let Some(frame) = self.clear_emergency(SYNC_TIMEOUT_EMCY_CODE) {
                        out.push(frame);
                    }
                }
            } else if self.sync_ever_seen && self.last_sync_ms.is_none() {
                // docs/DESIGN-DECISIONS.md SYN-12: re-entering (Pre)Operational after a Stop
                // must **not** disarm the supervision. CANopenNode keeps
                // `timeoutError` across the not-(Pre)Op branch — it zeroes
                // `timer`/`counter`/`receiveError` but not `timeoutError`
                // (`CO_SYNC.c:395-400`), and `timeoutError` only goes back to
                // 1 on `syncStatus == RX_TX` (`:402-406`) — so it reports
                // `CO_EM_SYNC_TIME_OUT` again 1.5 × period after the node
                // comes back, even if no SYNC arrived in between. The
                // reference's `timer = 0` maps to "no anchor yet": the first
                // `poll` back in (Pre)Op re-anchors, and the 1.5 × window
                // runs from there.
                self.last_sync_ms = Some(now_ms);
            }
            if sync_period > 0 && !sync_producer && self.sync_ever_seen && !self.sync_timeout_active
            {
                if let Some(last) = self.last_sync_ms {
                    // 1.5 × period without a SYNC (`CO_SYNC.c:355-357`).
                    let deadline = sync_period + sync_period / 2;
                    if now_ms.saturating_sub(last) > deadline {
                        self.sync_timeout_active = true;
                        // Vendor info = measured silence, mirroring the
                        // reference's `SYNC->timer` in µs (`CO_SYNC.c:361-363`).
                        let silence_us =
                            u32::try_from(now_ms.saturating_sub(last).saturating_mul(1_000))
                                .unwrap_or(u32::MAX);
                        if let Some(frame) = self.raise_emergency(
                            SYNC_TIMEOUT_EMCY_CODE,
                            crate::emcy::err_reg::COMMUNICATION,
                            emcy_vendor_info(SYNC_TIMEOUT_EMCY_BIT, silence_us),
                        ) {
                            out.push(frame);
                        }
                    }
                }
            }
        } else {
            // Leaving (Pre)Operational restarts the epoch, mirroring the
            // reference's gate (`CO_SYNC.c:395-400`).
            self.last_sync_ms = None;
            self.sync_seen_since_poll = false;
        }

        // Node-guarding slave life-time supervision (docs/DESIGN-DECISIONS.md NMT-03).
        // CANopenNode's `CO_nodeGuardingSlave_process` is called with
        // `slaveDisable = (0x1017 > 0)` and only starts the life timer once an
        // RTR has actually been received (`CO_Node_Guarding.c:178-230`).
        // An RTR is timestamped here (sans-I/O `on_remote_request` has no
        // clock); silence beyond guard-time × life-factor raises 0x8130 and
        // the next RTR clears it.
        let guard_time_ms = match self.od.read(Address::new(0x100C, 0)) {
            Ok(Value::Uint(v)) => v,
            _ => 0,
        };
        let guard_factor = match self.od.read(Address::new(0x100D, 0)) {
            Ok(Value::Uint(v)) => v,
            _ => 0,
        };
        let guard_enabled =
            guard_time_ms > 0 && guard_factor > 0 && self.producer_heartbeat_ms() == 0;
        if !guard_enabled {
            if self.guard_timeout_active {
                self.guard_timeout_active = false;
                if let Some(frame) = self.clear_emergency(0x8130) {
                    out.push(frame);
                }
            }
            self.guard_last_ms = None;
            self.guard_rtr_since_poll = false;
            self.guard_rearm_pending = false;
        } else {
            if self.guard_rearm_pending {
                self.guard_rearm_pending = false;
                if self.guard_last_ms.is_some() {
                    self.guard_last_ms = Some(now_ms);
                }
            }
            if self.guard_rtr_since_poll {
                self.guard_rtr_since_poll = false;
                self.guard_last_ms = Some(now_ms);
                if self.guard_timeout_active {
                    self.guard_timeout_active = false;
                    if let Some(frame) = self.clear_emergency(0x8130) {
                        out.push(frame);
                    }
                }
            } else if let Some(last) = self.guard_last_ms {
                let life_ms = guard_time_ms.saturating_mul(guard_factor);
                if now_ms.saturating_sub(last) >= life_ms && !self.guard_timeout_active {
                    self.guard_timeout_active = true;
                    let info = emcy_vendor_info(EM_BIT_HEARTBEAT_CONSUMER, 0);
                    if let Some(frame) =
                        self.raise_emergency(0x8130, crate::emcy::err_reg::COMMUNICATION, info)
                    {
                        out.push(frame);
                    }
                }
            }
        }

        // EMCY inhibit release (§8.2 / docs/DESIGN-DECISIONS.md P2-6): payloads queued by
        // `raise_emergency`/`clear_emergency` go out once the 0x1015 window
        // since the previous EMCY frame has elapsed. Frames only leave in
        // PreOperational/Operational (docs/DESIGN-DECISIONS.md #8); the register byte is
        // composed at send time so it reflects conditions raised/cleared
        // while the frame was queued (docs/DESIGN-DECISIONS.md #9).
        if matches!(
            self.nmt.state(),
            NmtState::PreOperational | NmtState::Operational
        ) {
            // docs/DESIGN-DECISIONS.md EMC-05: a 0x1015 write resets the reference's
            // `inhibitEmTimer` to 0; anchor that fresh window at this poll
            // (before the release check, even with an empty queue).
            if self.emcy_inhibit_rearm {
                self.emcy_inhibit_rearm = false;
                self.last_emcy_ms = Some(now_ms);
            }
        }
        if matches!(
            self.nmt.state(),
            NmtState::PreOperational | NmtState::Operational
        ) && !self.pending_emcy.is_empty()
        {
            // docs/DESIGN-DECISIONS.md EMC-15（并订正 §3.5 EMC-08 的出队门）：0x1014 禁用生产时
            // **不消费**队列。参考的 `CO_EM_process` 出队条件里没有
            // `producerEnabled`（`CO_Emergency.c:571-582`），`CO_error` 同样
            // 无条件写 fifo（`:688-706`）——帧不会因为「此刻不能发」而消失。
            // 旧实现先 `pop_front()` 再查 COB-ID，于是禁用期间出队即**丢弃**且
            // 照常推进 `last_emcy_ms`。现在 COB-ID 不可用就整队不动（冻结计时，
            // §3.5 EMC-08 备选 B），重新使能后原帧照发。
            // COB-ID unavailable: freeze the queue and timing, but never
            // short-circuit the rest of `poll`.
            if let Some(cob) = self.emcy_cob_id() {
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
                    if let Ok(f) = TxFrame::new(cob, &payload) {
                        out.push(f);
                    }
                }
                if self.pending_emcy.is_empty() {
                    self.emcy.set_overflow(&mut self.od, false);
                }
            }
        }

        // Heartbeat producer (any post-boot state). During Initialising the
        // periodic producer stays silent: boot-up is sent exactly once by
        // `boot()` (CANopenNode CO_NMT_Heartbeat.c sends 0x00 once on the
        // NMT initialise→pre-operational edge, never on the timer).
        let period_ms = self.producer_heartbeat_ms();
        // docs/DESIGN-DECISIONS.md 审-N2: while the heartbeat producer is active, node
        // guarding is disabled and CANopenNode forces the guarding toggle
        // back to the base state every cycle (`CO_Node_Guarding.c:178-183`,
        // entered via `slaveDisable = HBproducerTime > 0`,
        // `CANopen.c:1384`). Without this, disabling 0x1017 later resumed
        // guarding from a stale toggle and the first reply was ignored by a
        // strictly-alternating guard master.
        if period_ms > 0 {
            self.guarding_toggle = false;
        }
        if period_ms > 0 && self.nmt.state() != NmtState::Initialising {
            // docs/DESIGN-DECISIONS.md 八M4: anchor the heartbeat epoch at the first poll
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
            // RPDO event timer as dead line (§7.4/§7.5, docs/DESIGN-DECISIONS.md P2-7): a
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
                    // (docs/DESIGN-DECISIONS.md 十M8).
                    if self.rpdo_timeouts.len() >= MAX_RX_QUEUE {
                        self.rpdo_timeouts.pop_front();
                    }
                    self.rpdo_timeouts
                        .push_back(Address::new(slot.comm_index, 0));
                }
            }

            // docs/DESIGN-DECISIONS.md 七F2: mirror CANopenNode's EMCY reporting for a lapsed
            // RPDO dead line (`CO_PDO.c:822-831`): `CO_errorReport` once when
            // the first line times out (a single shared
            // `CO_EM_RPDO_TIME_OUT` condition, **elapsed** timer as vendor
            // info), `CO_errorReset` when traffic has resumed on every line.
            // Frames leave through the ordinary EMCY path (inhibit window
            // applies), so this runs after the `rpdos` iteration to avoid
            // overlapping borrows.
            //
            // docs/DESIGN-DECISIONS.md EMC-17: the infoCode is the *measured* silence in
            // microseconds (`RPDO->timeoutTimer`, `CO_PDO.c:831`), not the
            // configured event-timer window in milliseconds.
            let any_timed_out = self.rpdos.iter().any(|s| s.reported);
            if any_timed_out && !self.rpdo_emcy_active {
                self.rpdo_emcy_active = true;
                let elapsed_ms = self
                    .rpdos
                    .iter()
                    .find(|s| s.reported)
                    .and_then(|s| s.last_rx_ms.map(|last| now_ms.saturating_sub(last)))
                    .unwrap_or(0);
                let elapsed_us =
                    u32::try_from(elapsed_ms.saturating_mul(1_000)).unwrap_or(u32::MAX);
                self.rpdo_elapsed_us = elapsed_us;
                let info = emcy_vendor_info(EM_BIT_RPDO_TIME_OUT, elapsed_us);
                if let Some(frame) =
                    self.raise_emergency(0x8250, crate::emcy::err_reg::COMMUNICATION, info)
                {
                    out.push(frame);
                }
            } else if !any_timed_out && self.rpdo_emcy_active {
                self.rpdo_emcy_active = false;
                // CANopenNode's reset also carries the timer
                // (`CO_errorReset(..., RPDO->timeoutTimer)`, `CO_PDO.c:823`);
                // when other conditions are still active the producer
                // re-announces the lowest remaining code instead, and the
                // info field of *that* frame belongs to the other condition
                // (`EmcyProducer::clear_with_info`).
                let info = emcy_vendor_info(EM_BIT_RPDO_TIME_OUT, self.rpdo_elapsed_us);
                if let Some(frame) = self.clear_emergency_with_info(0x8250, info) {
                    out.push(frame);
                }
            }

            // flagsPDO trigger source (docs/DESIGN-DECISIONS.md P3-11): an OD write to any
            // mapped address latches the event-driven slot, exactly like an
            // application `request_tpdo` call would.
            let dirty = self.od.take_dirty();
            if !dirty.is_empty() {
                for slot in &mut self.tpdos {
                    if !slot.config.is_valid() {
                        continue;
                    }
                    // docs/DESIGN-DECISIONS.md #6: tt=0 (SyncAcyclic) also latches on a mapped
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
                // A SYNC-driven transmission performed since the last poll
                // re-anchors the timers here (the sans-I/O `on_frame` has no
                // time; one poll of resolution, like the RPDO deadman).
                if std::mem::take(&mut slot.tx_seen) {
                    slot.last_tx_ms = Some(now_ms);
                }
                match slot.tt {
                    TransmissionType::EventDriven(_) => {}
                    // docs/DESIGN-DECISIONS.md N1: the event timer also runs for tt=0
                    // (CANopenNode `CO_PDO.c:1245-1252`). Expiry only latches
                    // the request; `on_sync` emits it at the next SYNC
                    // (`:1305-1310`). `last_tx_ms == None` means "never sent
                    // this session", which the reference treats as an
                    // immediately expired `eventTimer` (initialised to 0).
                    TransmissionType::SyncAcyclic => {
                        if slot.config.event_timer_ms > 0
                            && slot.last_tx_ms.map_or(true, |t| {
                                now_ms.saturating_sub(t) >= u64::from(slot.config.event_timer_ms)
                            })
                        {
                            slot.pending_event = true;
                        }
                        continue;
                    }
                    _ => continue,
                }
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
                // docs/DESIGN-DECISIONS.md L5: 仅在打包**成功**后才清除触发标志与更新发送时间，
                // 否则一次失败的 pack 会丢失本次触发（pending_event 被提前清零、
                // 事件静默丢失，与 python-canopen 行为不符）。
                match pack_tpdo(slot, &self.od) {
                    Ok(Some(frame)) => {
                        slot.pending_event = false;
                        slot.last_tx_ms = Some(now_ms);
                        // docs/DESIGN-DECISIONS.md PDO-26: see the SYNC-driven path above.
                        slot.ever_sent = true;
                        out.push(frame);
                    }
                    Ok(None) => {}
                    // docs/DESIGN-DECISIONS.md M1: an event-driven TPDO whose mapping cannot
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
        // PDO number n maps to comm record 0x1800 + n-1 (docs/DESIGN-DECISIONS.md P3-19:
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
            // drives tt=0 the same way (docs/DESIGN-DECISIONS.md P2-3).
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
    /// inhibition (docs/IMPLEMENTATION.md §8.2, `docs/DESIGN-DECISIONS.md` P2-6).
    fn emcy_inhibit_window_ms(&self) -> u64 {
        match self.od.read(Address::new(0x1015, 0)) {
            Ok(Value::Uint(v)) => u64::from(v as u16).div_ceil(10),
            _ => 0,
        }
    }

    /// Raises an emergency; returns the EMCY frame when the code was newly
    /// activated (idempotent otherwise). EMCY is not *transmitted* while the
    /// node is outside PreOperational/Operational (CiA 301 §7.2.7,
    /// CANopenNode `CO_Emergency.c:557`); the condition is still recorded in
    /// 0x1001/0x1003 and the frame is queued, so it leaves on the first
    /// [`Node::poll`] after entering (Pre)Operational — the reference keeps it
    /// in the fifo and its `CO_EM_process` sends it as soon as
    /// `NMTisPreOrOperational` is true (`:557-582`, docs/DESIGN-DECISIONS.md EMC-01).
    ///
    /// With OD 0x1015 configured the frame is deferred the same way and
    /// released by [`Node::poll`] once the inhibit window since the previous
    /// EMCY transmission has elapsed (returns `None` in that case).
    pub fn raise_emergency(
        &mut self,
        code: u16,
        register_bits: u8,
        info: [u8; 5],
    ) -> Option<TxFrame> {
        let payload = self.emcy.raise(&mut self.od, code, register_bits, info)?;
        let may_transmit = matches!(
            self.nmt.state(),
            NmtState::PreOperational | NmtState::Operational
        );
        // docs/DESIGN-DECISIONS.md EMC-15: a **disabled producer** (0x1014 bit31 or COB-ID 0)
        // only gates the *send*, never the queueing. CANopenNode's
        // `CO_error` writes the fifo unconditionally (`CO_Emergency.c:688-706`)
        // and its dequeue has no enable gate either (`:571-582`); this stack
        // used to return early at `emcy_cob_id()?`, so frames raised while
        // disabled never even reached `pending_emcy` and were silently lost
        // (contradicting this method's own doc and EMC-01's "the frame is
        // queued"). `poll` re-checks the COB-ID before every send, so a
        // disabled producer simply parks the queue.
        if may_transmit && self.emcy_inhibit_window_ms() == 0 {
            if let Some(cob) = self.emcy_cob_id() {
                // docs/DESIGN-DECISIONS.md L6: EMCY payloads are constant 8 bytes — `new`
                // cannot fail; keep the no-panic contract with `.ok()`.
                return TxFrame::new(cob, &payload).ok();
            }
        }
        // Inhibited, or not (yet) allowed to transmit: queue for release in
        // `poll` (P2-6 / EMC-01). A full queue drops the new payload
        // (CANopenNode drops the new fifo entry, `CO_Emergency.c:695-706`)
        // and that loss sets the sticky overflow bit (docs/DESIGN-DECISIONS.md EMC-02); the
        // condition itself is already recorded in 0x1001/0x1003 by
        // `emcy.raise` above (`docs/DESIGN-DECISIONS.md` 七F5).
        if self.pending_emcy.len() >= MAX_PENDING_EMCY {
            self.emcy.set_overflow(&mut self.od, true);
            return None;
        }
        self.pending_emcy.push_back(payload);
        None
    }

    /// Clears an emergency code; returns the reset/no-error frame when it
    /// was active. Gated on (Pre)Operational like [`Self::raise_emergency`] —
    /// outside that the reset frame is queued for [`Node::poll`] instead of
    /// being dropped (docs/DESIGN-DECISIONS.md EMC-01); subject to the 0x1015 inhibit window
    /// the same way.
    pub fn clear_emergency(&mut self, code: u16) -> Option<TxFrame> {
        let payload = self.emcy.clear(&mut self.od, code)?;
        let may_transmit = matches!(
            self.nmt.state(),
            NmtState::PreOperational | NmtState::Operational
        );
        // docs/DESIGN-DECISIONS.md EMC-15: same as `raise_emergency` — a disabled producer
        // parks the frame instead of dropping it.
        if may_transmit && self.emcy_inhibit_window_ms() == 0 {
            if let Some(cob) = self.emcy_cob_id() {
                // docs/DESIGN-DECISIONS.md L6: constant 8-byte payload, `new` cannot fail.
                return TxFrame::new(cob, &payload).ok();
            }
        }
        // Same queue/drop-new policy as `raise_emergency` (`docs/DESIGN-DECISIONS.md` 七F5 /
        // EMC-01 / EMC-02).
        if self.pending_emcy.len() >= MAX_PENDING_EMCY {
            self.emcy.set_overflow(&mut self.od, true);
            return None;
        }
        self.pending_emcy.push_back(payload);
        None
    }

    /// Clears an emergency code with an explicit manufacturer field for the
    /// resulting reset/no-error frame.
    ///
    /// Same gating and queueing as [`Self::clear_emergency`], but the info
    /// bytes travel into the frame instead of being dropped. CANopenLinux's
    /// `CO_errorReset` stores the caller's infoCode next to
    /// `CO_EMC_NO_ERROR` (`CO_Emergency.c:668,673-675`), so a master can read
    /// how long the recovered condition had been pending (used for the 0x8250
    /// dead-line timer, `CO_PDO.c:823`; `docs/DESIGN-DECISIONS.md` EMC-17).
    pub fn clear_emergency_with_info(&mut self, code: u16, info: [u8; 5]) -> Option<TxFrame> {
        let payload = self.emcy.clear_with_info(&mut self.od, code, info)?;
        let may_transmit = matches!(
            self.nmt.state(),
            NmtState::PreOperational | NmtState::Operational
        );
        // docs/DESIGN-DECISIONS.md EMC-15: same as `raise_emergency` — a disabled producer
        // parks the frame instead of dropping it.
        if may_transmit && self.emcy_inhibit_window_ms() == 0 {
            if let Some(cob) = self.emcy_cob_id() {
                // docs/DESIGN-DECISIONS.md L6: constant 8-byte payload, `new` cannot fail.
                return TxFrame::new(cob, &payload).ok();
            }
        }
        // Same queue/drop-new policy as `clear_emergency` (`docs/DESIGN-DECISIONS.md` 七F5 /
        // EMC-01 / EMC-02).
        if self.pending_emcy.len() >= MAX_PENDING_EMCY {
            self.emcy.set_overflow(&mut self.od, true);
            return None;
        }
        self.pending_emcy.push_back(payload);
        None
    }

    /// Returns and clears communication addresses (0x1400+n) of RPDO slots
    /// whose event timer expired without reception (`docs/DESIGN-DECISIONS.md` P2-7). Each
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
                // Route through the bounded ring (docs/DESIGN-DECISIONS.md P3-1): a raw push
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

    /// Rebuilds only the PDO slot touched by a write into the communication
    /// (`0x1400..=0x15FF` RPDO / `0x1800..=0x19FF` TPDO) or mapping
    /// (`0x1600..=0x17FF` / `0x1A00..=0x1BFF`) parameter range, returning
    /// `false` when the address is not a PDO parameter address.
    ///
    /// CANopenNode's write handlers touch exactly the addressed PDO instance
    /// (`OD_write_14xx/16xx/18xx/1Axx`, `CO_PDO.c:487-676,876-1020`); the
    /// previous full rebuild on every write reset the runtime state of *all*
    /// other slots, so e.g. writing RPDO2's event timer silently cancelled a
    /// latched TPDO1 event (`docs/DESIGN-DECISIONS.md` 审-P1).
    ///
    /// When the addressed slot still assembles to the same configuration and
    /// mapping, its runtime state (phase lock, pending event, timers, SYNC
    /// sample, dead-line bookkeeping) is carried over so an idempotent write
    /// does not disturb an in-flight slot either. A changed or newly invalid
    /// configuration rebuilds/drops the slot exactly like the full rebuild
    /// would, with assembly errors recorded via [`Self::take_config_errors`].
    fn reconfigure_pdo(&mut self, addr: Address) -> bool {
        let comm_index = match addr.index {
            0x1400..=0x15FF => addr.index,
            0x1600..=0x17FF => pdo::RPDO_COMM_BASE + (addr.index - pdo::RPDO_MAP_BASE),
            0x1800..=0x19FF => addr.index,
            0x1A00..=0x1BFF => pdo::TPDO_COMM_BASE + (addr.index - pdo::TPDO_MAP_BASE),
            _ => return false,
        };
        if comm_index >= pdo::TPDO_COMM_BASE {
            let old = self.tpdos.iter().position(|s| s.comm_index == comm_index);
            let rebuilt = match self.build_tpdo(comm_index) {
                Ok(slot) => slot,
                Err(e) => {
                    self.record_config_error(e);
                    None
                }
            };
            match (old, rebuilt) {
                (Some(i), Some(slot)) => {
                    let prev = &self.tpdos[i];
                    // docs/DESIGN-DECISIONS.md PDO-09: a change limited to the **timer**
                    // fields (sub 3 inhibit, sub 5 event timer) must not throw
                    // the slot's running state away — CANopenNode's
                    // `OD_write_18xx` cases 3/5 only clear the corresponding
                    // timer and leave `syncCounter`/`sendRequest` alone
                    // (`CO_PDO.c:949-963`). So the merge condition compares
                    // the *structural* fields only: same direction/mapping
                    // identity (COB-ID, transmission type, SYNC start value).
                    let structural_unchanged = prev.mapping == slot.mapping
                        && prev.config.cob_id_raw == slot.config.cob_id_raw
                        && prev.config.transmission_type_raw == slot.config.transmission_type_raw
                        && prev.config.sync_start_value == slot.config.sync_start_value;
                    let merged = if structural_unchanged {
                        TpdoSlot {
                            syncs_seen: prev.syncs_seen,
                            last_tx_ms: prev.last_tx_ms,
                            tx_seen: prev.tx_seen,
                            pending_event: prev.pending_event,
                            phase_locked: prev.phase_locked,
                            // docs/DESIGN-DECISIONS.md PDO-28: a timer-only rewrite (sub 3 /
                            // sub 5, allowed while running) must not re-arm
                            // the SYNC start-value wait of a TPDO that is
                            // already transmitting — that is the silence
                            // PDO-26 was fixed for.
                            ever_sent: prev.ever_sent,
                            sync_received: prev.sync_received,
                            sync_sample: prev.sync_sample,
                            ..slot
                        }
                    } else {
                        slot
                    };
                    self.tpdos[i] = merged;
                }
                (Some(i), None) => {
                    self.tpdos.remove(i);
                }
                (None, Some(slot)) => {
                    let pos = self
                        .tpdos
                        .partition_point(|s| s.comm_index < slot.comm_index);
                    self.tpdos.insert(pos, slot);
                }
                (None, None) => {}
            }
        } else {
            let old = self.rpdos.iter().position(|s| s.comm_index == comm_index);
            let rebuilt = match self.build_rpdo(comm_index) {
                Ok(slot) => slot,
                Err(e) => {
                    self.record_config_error(e);
                    None
                }
            };
            match (old, rebuilt) {
                (Some(i), Some(slot)) => {
                    let prev = &self.rpdos[i];
                    // docs/DESIGN-DECISIONS.md PDO-09: same rule as the TPDO side — a
                    // dead-line timer change (sub 5) keeps the RPDO's
                    // receive/sync state; only a COB-ID / transmission-type /
                    // mapping change rebuilds it (`CO_PDO.c:894-899`).
                    let structural_unchanged = prev.mapping == slot.mapping
                        && prev.config.cob_id_raw == slot.config.cob_id_raw
                        && prev.config.transmission_type_raw == slot.config.transmission_type_raw
                        && prev.config.sync_start_value == slot.config.sync_start_value;
                    let merged = if structural_unchanged {
                        RpdoSlot {
                            sync_pending: prev.sync_pending,
                            rx_seen: prev.rx_seen,
                            last_rx_ms: prev.last_rx_ms,
                            // docs/DESIGN-DECISIONS.md PDO-25: a latched length condition
                            // survives a timer-only reconfiguration; the
                            // reference's `receiveError` is a property of the
                            // RPDO object, not of the record's timers.
                            rx_len: prev.rx_len,
                            reported: prev.reported,
                            ..slot
                        }
                    } else {
                        slot
                    };
                    self.rpdos[i] = merged;
                }
                (Some(i), None) => {
                    self.rpdos.remove(i);
                }
                (None, Some(slot)) => {
                    let pos = self
                        .rpdos
                        .partition_point(|s| s.comm_index < slot.comm_index);
                    self.rpdos.insert(pos, slot);
                }
                (None, None) => {}
            }
        }
        true
    }

    fn build_rpdo(&self, comm_index: u16) -> Result<Option<RpdoSlot>> {
        // docs/DESIGN-DECISIONS.md PDO-24: a **missing** sub 2 now defaults to 254
        // (`CO_PDO_TRANSM_TYPE_SYNC_EVENT_LO`, "apply immediately"), the same
        // value CANopenNode pre-loads for both directions
        // (`CO_PDO.c:654-655` for RPDO, `:1019-1020` for TPDO) — the
        // transmission type is a mandatory CiA 301 record field, so "0 =
        // synchronous" was an invented default that made a minimal RPDO apply
        // only at SYNC boundaries and, on a network without a SYNC producer,
        // never reach the OD at all. python-canopen keeps `trans_type = None`
        // and applies such a value immediately (`pdo/base.py:209,377`).
        let cfg = match PdoConfig::from_od(&self.od, comm_index, 254) {
            Ok(cfg) => cfg,
            // No communication record at all: the slot simply isn't
            // configured — silent (docs/DESIGN-DECISIONS.md #28).
            Err(Error::ObjectDoesNotExist { .. }) => return Ok(None),
            // Present but malformed: a configuration error, surfaced via
            // `take_config_errors` instead of silently skipped (docs/DESIGN-DECISIONS.md #28).
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
            // `take_config_errors` (docs/DESIGN-DECISIONS.md 八M1).
            return Err(Error::Other("PDO COB-ID 0x000 is not a valid identifier"));
        }
        let map_index = comm_index - pdo::RPDO_COMM_BASE + pdo::RPDO_MAP_BASE;
        // Malformed records and validation failures are *errors* — recorded
        // by the caller so a broken slot never vanishes without a trace
        // (docs/DESIGN-DECISIONS.md m17).
        let mapping = PdoMapping::from_od_record(&self.od, map_index)?;
        if mapping.total_bits() == 0 {
            // CiA 301 §7.4 / CANopenNode runtime (`CO_PDO.c:623-627`): a
            // PDO whose COB-ID is *valid* but whose mapping is empty can
            // never carry data — CANopenNode forces `valid=false` and
            // reports an error. Surface it here instead of silently
            // dropping the slot (docs/DESIGN-DECISIONS.md 八M2).
            return Err(Error::Other(
                "RPDO is enabled but its mapping record has zero entries",
            ));
        }
        mapping.validate_against(&self.od, true)?;
        // `?` (not `unwrap_or`): an invalid transmission type in the record
        // is a configuration error, surfaced via `take_config_errors` rather
        // than silently downgraded to synchronous (docs/DESIGN-DECISIONS.md R10-P2-12).
        let tt = cfg.transmission_type()?;
        if matches!(
            tt,
            pdo::TransmissionType::RtrOnlySync | pdo::TransmissionType::RtrOnlyEvent
        ) {
            // docs/DESIGN-DECISIONS.md 八L1: transmission types 252/253 are TPDO-only
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
            rx_len: RxLenState::NoError,
            reported: false,
        }))
    }

    fn build_tpdo(&self, comm_index: u16) -> Result<Option<TpdoSlot>> {
        // TPDO direction default: missing sub 2 means event-driven (254).
        let cfg = match PdoConfig::from_od(&self.od, comm_index, 254) {
            Ok(cfg) => cfg,
            // No communication record at all: the slot simply isn't
            // configured — silent (docs/DESIGN-DECISIONS.md #28).
            Err(Error::ObjectDoesNotExist { .. }) => return Ok(None),
            // Present but malformed: a configuration error, surfaced via
            // `take_config_errors` instead of silently skipped (docs/DESIGN-DECISIONS.md #28).
            Err(e) => return Err(e),
        };
        if !cfg.is_valid() {
            return Ok(None);
        }
        if cfg.cob_id() == 0 {
            // COB 0x000 is the NMT command channel — never a legal PDO
            // identifier (docs/DESIGN-DECISIONS.md 八M1, CANopenNode `CO_PDO.c:625`).
            return Err(Error::Other("PDO COB-ID 0x000 is not a valid identifier"));
        }
        let tt = cfg.transmission_type()?;
        let map_index = comm_index - pdo::TPDO_COMM_BASE + pdo::TPDO_MAP_BASE;
        let mapping = PdoMapping::from_od_record(&self.od, map_index)?;
        if mapping.total_bits() == 0 {
            // Symmetric with `build_rpdo` (docs/DESIGN-DECISIONS.md 八M2): an enabled TPDO
            // with an empty mapping is a configuration error, not a silent
            // no-op.
            return Err(Error::Other(
                "TPDO is enabled but its mapping record has zero entries",
            ));
        }
        mapping.validate_against(&self.od, false)?;
        // docs/DESIGN-DECISIONS.md #27: seed the cyclic divider so the first SyncCyclic
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
            tx_seen: false,
            pending_event: false,
            phase_locked: false,
            ever_sent: false,
            sync_received: false,
            sync_sample: None,
        }))
    }
}

/// SDO request payload: `None` when the frame is not exactly 8 bytes —
/// malformed/short frames are dropped instead of silently zero-padded
/// (`docs/DESIGN-DECISIONS.md` H2, matching the strict `len == 8` the LSS path requires).
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
/// drop the frame silently or record a configuration error (`docs/DESIGN-DECISIONS.md` M1,
/// symmetric with the RPDO unpack path which records into
/// `config_errors`).
fn pack_tpdo(slot: &TpdoSlot, od: &ObjectDictionary) -> Result<Option<TxFrame>> {
    let mut buf = [0u8; 8];
    let n = pdo::pack(&slot.mapping, od, &mut buf)?;
    Ok(TxFrame::new(slot.cob_id(), &buf[..n]).ok())
}

#[cfg(test)]
mod regression_tests_tier3 {
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

    /// docs/DESIGN-DECISIONS.md B3-1：OD 缺 0x1005 时 SYNC 应回退到默认 COB-ID 0x80。
    #[test]
    fn b3_1_is_sync_defaults_to_0x80_when_0x1005_absent() {
        let mut node = base_node();
        // 移除 0x1005，验证回退默认 0x80。
        node.od_mut().remove(Address::new(0x1005, 0));
        assert!(node.is_sync(0x80), "缺 0x1005 应回退默认 SYNC 0x80");
        assert!(!node.is_sync(0x81));
    }

    /// docs/DESIGN-DECISIONS.md 七F5：0x1015 inhibit 窗口内的延期 EMCY 队列必须有界，
    /// 病态高频 raise/clear 不得无限积压（CANopenNode 的 fifo 同样有界，
    /// 满时丢弃新条目，`CO_Emergency.c:695-706`）。
    #[test]
    fn pending_emcy_queue_is_bounded() {
        let mut node = base_node();
        node.boot().unwrap();
        // 0x1015 = 10000（1 s 窗口），所有 raise 都进延期队列。
        node.od_mut()
            .set_value(Address::new(0x1015, 0), Value::Uint(10_000))
            .unwrap();
        for i in 0..(MAX_PENDING_EMCY as u16 + 50) {
            assert!(node.raise_emergency(0x3000 + i, 0x01, [0; 5]).is_none());
        }
        assert_eq!(
            node.pending_emcy.len(),
            MAX_PENDING_EMCY,
            "queue must stop growing at the cap"
        );
        // docs/DESIGN-DECISIONS.md EMC-02: the dropped frames really were lost, so the sticky
        // overflow bit is set and mirrored into 0x1001…
        assert!(node.emcy.overflowed());
        assert_eq!(
            node.od.read(Address::new(0x1001, 0)).unwrap(),
            Value::Uint(u64::from(crate::emcy::err_reg::GENERIC))
        );
        // …and it clears once the transmit queue has drained.
        node.od_mut()
            .set_value(Address::new(0x1015, 0), Value::Uint(0))
            .unwrap();
        let _ = node.poll(1_000);
        assert!(node.pending_emcy.is_empty());
        assert!(!node.emcy.overflowed());
    }

    /// docs/DESIGN-DECISIONS.md B3-1：显式配置 0x1005 时按配置值识别而非默认 0x80。
    #[test]
    fn b3_1_is_sync_honours_configured_0x1005() {
        let mut node = base_node();
        node.od_mut()
            .set_value(Address::new(0x1005, 0), Value::Uint(0x100))
            .unwrap();
        assert!(node.is_sync(0x100), "应按配置的 0x100 识别 SYNC");
        assert!(!node.is_sync(0x80));
    }

    /// docs/DESIGN-DECISIONS.md B3-2：Initialising 阶段锁存的 EMCY 条件，进入 Operational 时应
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

    /// docs/DESIGN-DECISIONS.md P2-1：SYNC COB-ID（含 0x1005 自定义 0x81..=0xFF）不得被可选
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
        // 码 0x8110（通信类），不是 0x00xx reset 族。
        let _ = node.on_frame(0x82, &[0x10, 0x81, 0x00, 0x00, 0, 0, 0, 0]);
        assert_eq!(node.emcy_consumer().unwrap().active().len(), 1);
    }

    /// docs/DESIGN-DECISIONS.md H2: a short (<8 byte) frame on the SDO request channel must be
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

    /// docs/DESIGN-DECISIONS.md H2：落在 EMCY 区间（0x81..=0xFF）内的**自定义 RPDO COB-ID**
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
        let _ = node.on_frame(0x82, &[0x10, 0x81, 0x00, 0x00, 0, 0, 0, 0]);
        assert_eq!(node.emcy_consumer().unwrap().active().len(), 1);
    }

    /// docs/DESIGN-DECISIONS.md 十M20：EMCY 消费者只接受标准范围 0x80..=0xFF。旧实现把
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

    /// docs/DESIGN-DECISIONS.md M4：类型 252（RtrOnlySync）TPDO 的 RTR 应答必须在"最后 SYNC"
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
            0x185u64, // bit30=0：RTR 允许（CiA 语义，docs/DESIGN-DECISIONS.md 八H1）
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
            "RTR 应答应为 SYNC 采样值 0x11，而非实时值 0x22（docs/DESIGN-DECISIONS.md M4）"
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
