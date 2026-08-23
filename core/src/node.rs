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
use crate::nmt::{decode_command, encode_heartbeat, NmtCommand, NmtState, NmtStateMachine};
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
    config: PdoConfig,
    mapping: PdoMapping,
    tt: TransmissionType,
    /// SYNCs observed since the last transmission (cyclic divider).
    syncs_seen: u32,
    /// Last transmission in monotonic ms (inhibit + event timer).
    last_tx_ms: Option<u64>,
    /// Application-requested transmission (event-driven trigger).
    pending_event: bool,
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
    config: PdoConfig,
    mapping: PdoMapping,
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
    /// disappears without a trace (BUGS.md m17).
    config_errors: Vec<crate::types::Error>,
}

impl Node {
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
            lss: None,
            unconfigured: false,
            storage: None,
            defaults,
            defaults_snapshot_taken: false,
            config_errors: Vec::new(),
        };
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
            lss: Some(crate::lss::LssSlave::new(
                address,
                crate::lss::UNCONFIGURED_NODE_ID,
            )),
            unconfigured: true,
            storage: None,
            defaults,
            defaults_snapshot_taken: false,
            config_errors: Vec::new(),
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
        if !(1..=127).contains(&raw) {
            return None;
        }
        self.lss.as_mut()?.adopt_pending();
        if let Ok(id) = NodeId::try_from(raw) {
            self.node_id = id;
            self.unconfigured = false;
            // Re-point the EMCY producer entry at the new default COB-ID.
            let _ = self.od.set_value(
                Address::new(0x1014, 0),
                Value::Uint(u64::from(id.emcy_cob_id())),
            );
            if let Err(e) = self.configure_pdos_from_od() {
                self.config_errors.push(e);
            }
            Some(id)
        } else {
            None
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
        if !self.nmt.boot() {
            return None;
        }
        if let Err(e) = self.configure_pdos_from_od() {
            self.config_errors.push(e);
        }
        Some(TxFrame::new(self.node_id.heartbeat_cob_id(), &[0x00]).expect("one byte"))
    }

    // --- Frame intake ---------------------------------------------------------

    /// Consumes one received frame addressed to this node, returning frames
    /// to transmit in order.
    #[must_use]
    pub fn on_frame(&mut self, cob_id: u16, data: &[u8]) -> Vec<TxFrame> {
        // LSS is served in every state; while unconfigured it is the ONLY
        // service (data COB-IDs are meaningless yet).
        if cob_id == crate::lss::LSS_MASTER_COB_ID {
            if let Some(lss) = &mut self.lss {
                let mut req = [0u8; 8];
                let n = data.len().min(8);
                req[..n].copy_from_slice(&data[..n]);
                if let Some(resp) = lss.handle(&req) {
                    if let Ok(frame) = TxFrame::new(crate::lss::LSS_SLAVE_COB_ID, &resp) {
                        return vec![frame];
                    }
                }
            }
            if self.unconfigured {
                return Vec::new();
            }
        } else if self.unconfigured {
            return Vec::new();
        }
        let sdo_rx = self.node_id.sdo_rx_cob_id();
        let hb = self.node_id.heartbeat_cob_id();

        if cob_id == crate::nmt::NMT_COMMAND_COB_ID {
            self.on_nmt(data);
            return Vec::new();
        }
        if cob_id == sdo_rx
            && matches!(
                self.nmt_state(),
                NmtState::PreOperational | NmtState::Operational
            )
        {
            let mut out = Vec::new();
            let req = sdo_payload(data);
            for payload in self.sdo.handle(&mut self.od, &req) {
                if let Ok(frame) = TxFrame::new(self.node_id.sdo_tx_cob_id(), &payload) {
                    out.push(frame);
                }
                // Live-reconfiguration: a write into the PDO parameter range
                // re-runs slot assembly (docs/IMPLEMENTATION.md §7.3).
                if let Some((addr, value)) = self.sdo.take_last_written() {
                    if (0x1400..=0x1BFF).contains(&addr.index) {
                        if let Err(e) = self.configure_pdos_from_od() {
                            self.config_errors.push(e);
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
        if self.is_sync(cob_id) {
            return self.on_sync(data);
        }
        if let NmtState::Operational = self.nmt_state() {
            for slot in &mut self.rpdos {
                if slot.config.is_valid() && slot.config.cob_id() == cob_id {
                    if let Err(e) = pdo::unpack(&slot.mapping, &mut self.od, data) {
                        self.config_errors.push(e);
                    }
                    break;
                }
            }
        }
        Vec::new()
    }

    /// Handles a remote-request (RTR) on `cob_id`; answers node-guarding
    /// polls on the heartbeat COB-ID with an alternating toggle bit.
    pub fn on_remote_request(&mut self, cob_id: u16) -> Option<TxFrame> {
        let hb = self.node_id.heartbeat_cob_id();
        if cob_id != hb || self.nmt.state() == NmtState::Initialising {
            return None;
        }
        // First response carries toggle 0 (CiA 301 §7.3.1).
        let payload = crate::nmt::encode_node_guard(self.nmt.state(), self.guarding_toggle);
        self.guarding_toggle = !self.guarding_toggle;
        Some(TxFrame::new(hb, &payload).expect("one byte"))
    }

    fn on_nmt(&mut self, data: &[u8]) {
        let Some((command, target)) = decode_command(data) else {
            return;
        };
        let me = self.node_id.get();
        if target != 0 && target != me {
            return;
        }
        match command {
            NmtCommand::ResetNode | NmtCommand::ResetCommunication => {
                self.nmt.apply(command);
                self.sdo.reset();
                self.tpdos.clear();
                self.rpdos.clear();
                self.last_hb_ms = None;
                // Communication profile re-initialisation: drop latched EMCY
                // conditions and the node-guarding toggle (the EM module is
                // re-initialised in CANopenNode's reset sequence).
                self.emcy = EmcyProducer::new();
                self.emcy.resync_od(&mut self.od);
                self.guarding_toggle = false;
                // Power-on semantics: re-apply persisted parameters.
                if let Some(backend) = &self.storage {
                    let _ = backend.load(&mut self.od);
                }
            }
            _ => {
                self.nmt.apply(command);
            }
        }
    }

    fn is_sync(&self, cob_id: u16) -> bool {
        matches!(self.od.read(Address::new(0x1005, 0)), Ok(Value::Uint(raw)) if raw as u16 == cob_id && raw & 0x8000_0000 == 0)
    }

    /// SYNC consumer: advances synchronous TPDOs.
    fn on_sync(&mut self, data: &[u8]) -> Vec<TxFrame> {
        if self.nmt.state() != NmtState::Operational {
            return Vec::new();
        }
        let counter = crate::sync_time::decode_sync(data).flatten();
        let mut out = Vec::new();
        for slot in &mut self.tpdos {
            if !slot.config.is_valid() || !slot.tt.is_synchronous() {
                continue;
            }
            // Sync start value (S14): wait for the matching global counter
            // before starting cyclic counting when one is configured.
            if let Some(start) = nonzero(slot.config.sync_start_value) {
                if let Some(global) = counter {
                    if global != start && slot.syncs_seen == 0 {
                        continue;
                    }
                } else {
                    continue; // cannot phase-lock without counters
                }
            }
            let due = match slot.tt {
                TransmissionType::SyncAcyclic => {
                    let p = slot.pending_event;
                    slot.pending_event = false;
                    p
                }
                TransmissionType::SyncCyclic(n) => {
                    slot.syncs_seen += 1;
                    if u32::from(n.max(1)) <= slot.syncs_seen {
                        slot.syncs_seen = 0;
                        true
                    } else {
                        false
                    }
                }
                _ => false,
            };
            if due {
                if let Some(frame) = pack_tpdo(slot, &self.od) {
                    out.push(frame);
                }
            }
        }
        out
    }

    // --- Producers --------------------------------------------------------------

    /// Periodic maintenance: heartbeat production and event-driven TPDO
    /// timers (event timer + inhibit time). Call regularly with monotonic ms.
    pub fn poll(&mut self, now_ms: u64) -> Vec<TxFrame> {
        let mut out = Vec::new();

        // Heartbeat producer (any post-boot state). During Initialising the
        // periodic producer stays silent: boot-up is sent exactly once by
        // `boot()` (CANopenNode CO_NMT_Heartbeat.c sends 0x00 once on the
        // NMT initialise→pre-operational edge, never on the timer).
        let period_ms = match self.od.read(Address::new(0x1017, 0)) {
            Ok(Value::Uint(v)) => v,
            _ => 0,
        };
        if period_ms > 0 && self.nmt.state() != NmtState::Initialising {
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

        // Event-driven TPDOs (Operational only).
        if self.nmt.state() == NmtState::Operational {
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
                slot.pending_event = false;
                slot.last_tx_ms = Some(now_ms);
                if let Some(frame) = pack_tpdo(slot, &self.od) {
                    out.push(frame);
                }
            }
        }
        out
    }

    /// Requests transmission of an event-driven TPDO (numbered from 1),
    /// e.g. after application data changed. The frame goes out on the next
    /// [`Node::poll`] outside its inhibit window.
    pub fn request_tpdo(&mut self, number: u8) -> bool {
        let Some(index) = number.checked_sub(1) else {
            return false;
        };
        match self
            .tpdos
            .iter_mut()
            .filter(|s| s.config.is_valid())
            .nth(usize::from(index))
        {
            Some(slot) if matches!(slot.tt, TransmissionType::EventDriven(_)) => {
                slot.pending_event = true;
                true
            }
            _ => false,
        }
    }

    /// EMCY producer COB-ID, read from OD 0x1014 honouring the CiA 301
    /// bit31 "bit invalid" flag (CANopenNode `CO_Emergency.c` reads the
    /// producer COB-ID from the dictionary on every send). `None` when
    /// EMCY production is disabled; falls back to `0x80 + node-id` when
    /// the entry is missing.
    fn emcy_cob_id(&self) -> Option<u16> {
        const INVALID: u32 = 1 << 31;
        match self.od.read(Address::new(0x1014, 0)) {
            Ok(Value::Uint(v)) => {
                let v = v as u32;
                if v & INVALID != 0 {
                    return None;
                }
                let cob = (v & 0x7FF) as u16;
                Some(if cob == 0 {
                    self.node_id.emcy_cob_id()
                } else {
                    cob
                })
            }
            _ => Some(self.node_id.emcy_cob_id()),
        }
    }

    /// Raises an emergency; returns the EMCY frame when the code was newly
    /// activated (idempotent otherwise). EMCY is not produced while the node
    /// is still in Initialising (CiA 301: PreOp/Op/Stopped only).
    pub fn raise_emergency(
        &mut self,
        code: u16,
        register_bits: u8,
        info: [u8; 4],
    ) -> Option<TxFrame> {
        if self.nmt.state() == NmtState::Initialising {
            return None;
        }
        let payload = self.emcy.raise(&mut self.od, code, register_bits, info)?;
        let cob = self.emcy_cob_id()?;
        Some(TxFrame::new(cob, &payload).expect("eight bytes"))
    }

    /// Clears an emergency code; returns the reset/no-error frame when it
    /// was active. Suppressed during Initialising like [`Self::raise_emergency`].
    pub fn clear_emergency(&mut self, code: u16) -> Option<TxFrame> {
        if self.nmt.state() == NmtState::Initialising {
            return None;
        }
        let payload = self.emcy.clear(&mut self.od, code)?;
        let cob = self.emcy_cob_id()?;
        Some(TxFrame::new(cob, &payload).expect("eight bytes"))
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
                Err(e) => self.config_errors.push(e),
            }
        }
        for comm_index in 0x1800..=0x19FF {
            match self.build_tpdo(comm_index) {
                Ok(Some(slot)) => self.tpdos.push(slot),
                Ok(None) => {}
                Err(e) => self.config_errors.push(e),
            }
        }
        Ok(())
    }

    fn build_rpdo(&self, comm_index: u16) -> Result<Option<RpdoSlot>> {
        let Ok(cfg) = PdoConfig::from_od(&self.od, comm_index) else {
            return Ok(None);
        };
        if !cfg.is_valid() {
            return Ok(None); // COB-ID marked invalid: slot intentionally off
        }
        let map_index = comm_index - pdo::RPDO_COMM_BASE + pdo::RPDO_MAP_BASE;
        // Malformed records and validation failures are *errors* — recorded
        // by the caller so a broken slot never vanishes without a trace
        // (BUGS.md m17).
        let mapping = PdoMapping::from_od_record(&self.od, map_index)?;
        if mapping.total_bits() == 0 {
            return Ok(None); // empty mapping: nothing to receive
        }
        mapping.validate_against(&self.od, true)?;
        Ok(Some(RpdoSlot {
            config: cfg,
            mapping,
        }))
    }

    fn build_tpdo(&self, comm_index: u16) -> Result<Option<TpdoSlot>> {
        let Ok(cfg) = PdoConfig::from_od(&self.od, comm_index) else {
            return Ok(None);
        };
        if !cfg.is_valid() {
            return Ok(None);
        }
        let Ok(tt) = cfg.transmission_type() else {
            return Ok(None);
        };
        let map_index = comm_index - pdo::TPDO_COMM_BASE + pdo::TPDO_MAP_BASE;
        let mapping = PdoMapping::from_od_record(&self.od, map_index)?;
        if mapping.total_bits() == 0 {
            return Ok(None); // empty mapping: nothing to transmit
        }
        mapping.validate_against(&self.od, false)?;
        Ok(Some(TpdoSlot {
            config: cfg,
            mapping,
            tt,
            syncs_seen: 0,
            last_tx_ms: None,
            pending_event: false,
        }))
    }
}

fn sdo_payload(data: &[u8]) -> [u8; 8] {
    let mut buf = [0u8; 8];
    let n = data.len().min(8);
    buf[..n].copy_from_slice(&data[..n]);
    buf
}

const fn nonzero(v: u8) -> Option<u8> {
    if v == 0 {
        None
    } else {
        Some(v)
    }
}

/// Packs one TPDO payload from its mapping.
fn pack_tpdo(slot: &TpdoSlot, od: &ObjectDictionary) -> Option<TxFrame> {
    let mut buf = [0u8; 8];
    let n = pdo::pack(&slot.mapping, od, &mut buf).ok()?;
    TxFrame::new(slot.cob_id(), &buf[..n]).ok()
}
