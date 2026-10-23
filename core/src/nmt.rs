//! NMT — network management (CiA 301 §7.2.8): node state machine, master
//! commands, heartbeat/boot-up producers and the heartbeat consumer.
//!
//! Wire facts:
//!
//! - Master commands travel on COB-ID `0x000` as `[command, target]`, where
//!   target `0` means broadcast.
//! - Heartbeat/boot-up frames travel on `0x700 + node-id`; byte 0 is the
//!   state (boot-up reports `0x00`). Node-guard responses share the frame
//!   shape with an alternating toggle in bit 7.
//!
//! The [`NmtStateMachine`] implements the slave side: it starts in
//! [`NmtState::Initialising`], leaves only via [`NmtStateMachine::boot`]
//! (which also yields the boot-up frame), and ignores operational commands
//! while still initialising. Resets return to Initialising from any state.

use std::collections::BTreeMap;

use crate::types::NodeId;

/// COB-ID of NMT master command frames.
pub const NMT_COMMAND_COB_ID: u16 = 0x000;

/// COB-ID base for heartbeat / boot-up / node-guarding frames (`0x700 + id`).
pub const HEARTBEAT_COB_BASE: u16 = 0x700;

/// NMT device states; the discriminants are the wire values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum NmtState {
    /// Boot-up state; reported once via the boot-up frame (0x00).
    Initialising = 0x00,
    /// Stopped: NMT/guarding/heartbeat only, no PDO/SDO (0x04).
    Stopped = 0x04,
    /// Fully operational (0x05).
    Operational = 0x05,
    /// Pre-operational: SDO active, PDOs off (0x7F).
    PreOperational = 0x7F,
}

impl NmtState {
    /// Decodes a heartbeat/guarding payload byte (toggle bit masked).
    #[must_use]
    pub fn from_byte(byte: u8) -> Option<Self> {
        Some(match byte & !GUARD_TOGGLE {
            0x00 => Self::Initialising,
            0x04 => Self::Stopped,
            0x05 => Self::Operational,
            0x7F => Self::PreOperational,
            _ => return None,
        })
    }

    /// The wire value for heartbeat frames (without guarding toggle).
    #[must_use]
    pub const fn to_byte(self) -> u8 {
        self as u8
    }
}

/// Toggle bit of node-guarding responses (bit 7).
const GUARD_TOGGLE: u8 = 0x80;

/// NMT master commands; the discriminants are the wire values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum NmtCommand {
    /// Start remote node (0x01).
    StartRemoteNode = 0x01,
    /// Stop remote node (0x02).
    StopRemoteNode = 0x02,
    /// Enter pre-operational (0x80).
    EnterPreOperational = 0x80,
    /// Reset node — application reset (0x81).
    ResetNode = 0x81,
    /// Reset communication — communication profile reset (0x82).
    ResetCommunication = 0x82,
}

impl NmtCommand {
    /// Decodes from the first payload byte of a command frame.
    #[must_use]
    pub fn from_byte(byte: u8) -> Option<Self> {
        Some(match byte {
            0x01 => Self::StartRemoteNode,
            0x02 => Self::StopRemoteNode,
            0x80 => Self::EnterPreOperational,
            0x81 => Self::ResetNode,
            0x82 => Self::ResetCommunication,
            _ => return None,
        })
    }

    /// The wire value.
    #[must_use]
    pub const fn to_byte(self) -> u8 {
        self as u8
    }
}

/// Encodes a master command frame payload: `[command, target]`, target
/// `0` = broadcast.
#[must_use]
pub fn encode_command(command: NmtCommand, target: Option<NodeId>) -> [u8; 2] {
    [command.to_byte(), target.map_or(0, |n| n.get())]
}

/// Decodes a master command frame payload into `(command, target)` where
/// target `0` means broadcast.
///
/// Returns `None` for frames shorter than two bytes or unknown commands.
#[must_use]
pub fn decode_command(data: &[u8]) -> Option<(NmtCommand, u8)> {
    // CiA 301: the NMT command frame is exactly 2 bytes; CANopenNode
    // (`CO_NMT_Heartbeat.c:38`) ignores any other DLC (docs/DESIGN-DECISIONS.md 八L12).
    if data.len() != 2 {
        return None;
    }
    // Target byte: 0 = all nodes, 1..=127 a specific node; anything else
    // is outside the CiA 301 range and must not reach the state machine.
    let target = data[1];
    if target > 0x7F {
        return None;
    }
    Some((NmtCommand::from_byte(data[0])?, target))
}

/// Encodes a heartbeat payload byte (also used for boot-up with
/// [`NmtState::Initialising`]).
#[must_use]
pub const fn encode_heartbeat(state: NmtState) -> [u8; 1] {
    [state.to_byte()]
}

/// Encodes a legacy node-guarding response: state byte plus alternating
/// toggle in bit 7.
#[must_use]
pub const fn encode_node_guard(state: NmtState, toggle: bool) -> [u8; 1] {
    [state.to_byte() | if toggle { GUARD_TOGGLE } else { 0 }]
}

/// Decodes a heartbeat or guarding payload, masking the toggle bit.
///
/// Returns `(state, toggle)`.
#[must_use]
pub fn decode_heartbeat(data: &[u8]) -> Option<(NmtState, bool)> {
    let byte = *data.first()?;
    Some((NmtState::from_byte(byte)?, byte & GUARD_TOGGLE != 0))
}

/// The slave-side NMT state machine.
///
/// Starts in [`NmtState::Initialising`]; [`NmtStateMachine::boot`] performs
/// the initial transition and reports whether the boot-up frame must be
/// sent. Commands targeting this node (or broadcast) are applied via
/// [`NmtStateMachine::apply`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NmtStateMachine {
    state: NmtState,
}

impl Default for NmtStateMachine {
    fn default() -> Self {
        Self::new()
    }
}

impl NmtStateMachine {
    /// A machine in the initialising state.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: NmtState::Initialising,
        }
    }

    /// Current state.
    #[must_use]
    pub const fn state(&self) -> NmtState {
        self.state
    }

    /// Leaves the initialising state, returning whether the caller should
    /// transmit a boot-up frame. No-op when already booted.
    pub fn boot(&mut self) -> bool {
        if self.state == NmtState::Initialising {
            self.state = NmtState::PreOperational;
            true
        } else {
            false
        }
    }

    /// Applies a master command addressed to this node (directly or by
    /// broadcast). Returns whether the state changed.
    ///
    /// Transition rules (CiA 301):
    /// - resets return to Initialising from any state;
    /// - `StartRemoteNode` is also honoured while Initialising;
    /// - re-entering the current state is a no-op.
    ///
    /// # Reset trap (adjudicated, `docs/DESIGN-DECISIONS.md` H2)
    ///
    /// A reset leaves the machine in `Initialising`; the caller must follow
    /// up with [`Self::boot`] so the node re-enters PreOperational and
    /// emits boot-up, exactly like a power-on sequence. This API documents
    /// the requirement instead of hiding it — `Node::on_nmt` already chains
    /// the two for the high-level slave.
    pub fn apply(&mut self, command: NmtCommand) -> bool {
        match command {
            // Resets always win.
            NmtCommand::ResetNode | NmtCommand::ResetCommunication => {
                let changed = self.state != NmtState::Initialising;
                self.state = NmtState::Initialising;
                changed
            }
            NmtCommand::StartRemoteNode => {
                // docs/DESIGN-DECISIONS.md NMT-14: CiA 301 requires a node to accept NMT
                // commands while Initialising, and both references apply
                // `CO_NMT_ENTER_OPERATIONAL` unconditionally
                // (`CO_NMT_Heartbeat.c:207-220` has no state gate,
                // python `nmt.py:57-66` assigns `self._state` outright). The
                // old gate silently dropped a master command that raced
                // application start-up (the reachable window is
                // `Node::new` → the application's first `boot()`).
                if !matches!(self.state, NmtState::Operational) {
                    self.state = NmtState::Operational;
                    true
                } else {
                    false
                }
            }
            NmtCommand::StopRemoteNode => {
                if matches!(self.state, NmtState::Operational | NmtState::PreOperational) {
                    self.state = NmtState::Stopped;
                    true
                } else {
                    false
                }
            }
            NmtCommand::EnterPreOperational => {
                if matches!(self.state, NmtState::Operational | NmtState::Stopped) {
                    self.state = NmtState::PreOperational;
                    true
                } else {
                    false
                }
            }
        }
    }
}

/// Phase of a monitored node inside the [`HeartbeatConsumer`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Configured but no heartbeat seen yet.
    Unknown,
    /// Heartbeats arriving within the timeout window.
    Active,
    /// Timeout elapsed since the last heartbeat.
    TimedOut,
}

/// Events produced by the [`HeartbeatConsumer`]. All carry the specific
/// `NodeId` (unlike CANopenNode's shared error index — see
/// docs/REFERENCES.md W11).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HbEvent {
    /// First heartbeat after configuration or recovery from timeout.
    Started {
        /// The node that came online.
        node: NodeId,
    },
    /// Heartbeat timeout elapsed; the node appears dead.
    Lost {
        /// The node that timed out.
        node: NodeId,
    },
    /// A previously active node sent its boot-up frame (remote reset).
    RemoteReset {
        /// The node that reset.
        node: NodeId,
    },
    /// The reported state changed between heartbeats.
    StateChanged {
        /// The node whose state changed.
        node: NodeId,
        /// Newly reported state.
        new_state: NmtState,
    },
}

#[derive(Debug, Clone, Copy)]
struct MonitoredEntry {
    timeout_ms: u32,
    phase: Phase,
    state: Option<NmtState>,
    last_seen_ms: u64,
    /// 0-based 0x1016 array position when this entry came from the object
    /// dictionary, used for the 0x8130 infoCode.
    sub_index: Option<u16>,
}

/// Consumer-side heartbeat monitoring: per-node timeout supervision with a
/// four-phase state machine (`UNCONFIGURED → UNKNOWN → ACTIVE ⇄ TIMEOUT`),
/// configured like object 0x1016 (node id + consumer time in ms).
///
/// Feed every received heartbeat frame through [`HeartbeatConsumer::on_heartbeat`]
/// and drive timeouts with periodic [`HeartbeatConsumer::poll`] calls; both
/// take monotonic milliseconds supplied by the caller (no clock reads).
///
/// Aggregated queries mirror CANopenNode's convenience flags:
/// [`HeartbeatConsumer::all_active`] / [`HeartbeatConsumer::all_operational`].
#[derive(Debug, Default)]
pub struct HeartbeatConsumer {
    monitored: BTreeMap<u8, MonitoredEntry>,
    /// Supervision tolerance in percent (§6.5): the effective deadline is
    /// `timeout_ms × (1 + tolerance_pct / 100)`. Zero (default) means
    /// strict deadlines (`docs/DESIGN-DECISIONS.md` P3-14).
    tolerance_pct: u32,
}

impl HeartbeatConsumer {
    /// Creates an empty monitor.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the supervision tolerance in percent applied to every monitored
    /// node's deadline (0 = strict). Useful on buses with scheduling jitter.
    pub fn set_tolerance_pct(&mut self, pct: u32) {
        self.tolerance_pct = pct.min(100);
    }

    /// Current supervision tolerance in percent.
    #[must_use]
    pub const fn tolerance_pct(&self) -> u32 {
        self.tolerance_pct
    }

    /// Effective supervision deadline for a configured timeout, including
    /// the tolerance margin (`docs/DESIGN-DECISIONS.md` P3-14).
    #[must_use]
    pub fn deadline_ms(timeout_ms: u32, tolerance_pct: u32) -> u64 {
        u64::from(timeout_ms) + (u64::from(timeout_ms) * u64::from(tolerance_pct)) / 100
    }

    /// Clears the supervision state of every monitored node back to
    /// `Unknown` without dropping the configuration (timeout values survive).
    ///
    /// Mirrors CANopenNode's `(pre)operational state changed` branch
    /// (`CO_HBconsumer.c:404-417`): on any edge into or out of
    /// PreOperational/Operational every entry's NMT state becomes unknown
    /// and its heartbeat state returns to `UNKNOWN`, so the next heartbeat
    /// re-announces `Started` and stale `last_seen` stamps cannot fire an
    /// immediate false `Lost` after a long Stopped period (`docs/DESIGN-DECISIONS.md` 七F3).
    pub fn reset_states(&mut self) {
        for entry in self.monitored.values_mut() {
            entry.phase = Phase::Unknown;
            entry.state = None;
            entry.last_seen_ms = 0;
        }
    }

    /// Starts monitoring `node` with a timeout in ms (reconfiguration is
    /// allowed at runtime, mirroring writes to 0x1016). A `timeout_ms` of
    /// zero disables supervision of that node (CANopenNode semantics for
    /// `time_us == 0`) and drops the entry entirely, so `configured()` and
    /// the aggregate predicates stop reporting it (`docs/DESIGN-DECISIONS.md` 十M5).
    ///
    /// Reconfiguration with a **different** non-zero timeout **resets the
    /// supervision state** of the entry (`phase`/`state`/`last_seen` back to
    /// unknown), not just the deadline: CANopenNode's `OD_write_1016` calls
    /// `CO_HBconsumer_initEntry` on every write, which sets
    /// `NMTstate = CO_NMT_UNKNOWN`, clears `CANrxNew` and puts
    /// `HBstate = CO_HBconsumer_UNKNOWN` (`CO_HBconsumer.c:76-101,168-216`).
    /// Keeping the pre-write silence made a reconfiguration able to fire a
    /// false `Lost` (plus 0x8130) on the very next poll — docs/DESIGN-DECISIONS.md NMT-13.
    pub fn set_timeout(&mut self, node: NodeId, timeout_ms: u32) {
        if timeout_ms == 0 {
            self.monitored.remove(&node.get());
            return;
        }
        let entry = self.monitored.entry(node.get()).or_insert(MonitoredEntry {
            timeout_ms,
            phase: Phase::Unknown,
            state: None,
            last_seen_ms: 0,
            sub_index: None,
        });
        // docs/DESIGN-DECISIONS.md NMT-15: the reference re-inits **the written entry only**
        // (`CO_HBconsumer_initEntry(stream->subIndex - 1U)`,
        // `CO_HBconsumer.c:93-94`). `Node::resync_hb_consumers_from_od`
        // re-applies every 0x1016 sub on any write, so resetting
        // unconditionally would throw away the supervision state of unrelated
        // entries (they go back to `Unknown` and cannot report `Lost` until
        // their next heartbeat, and a latched 0x8130 can no longer be cleared
        // by a recovering node alone). Reset only on an actual change.
        if entry.timeout_ms != timeout_ms {
            entry.timeout_ms = timeout_ms;
            entry.phase = Phase::Unknown;
            entry.state = None;
            entry.last_seen_ms = 0;
        }
    }

    /// Starts monitoring `node` and records its 0-based 0x1016 array
    /// position for the 0x8130 infoCode.
    pub(crate) fn set_timeout_with_index(&mut self, node: NodeId, timeout_ms: u32, sub_index: u16) {
        self.set_timeout(node, timeout_ms);
        if let Some(entry) = self.monitored.get_mut(&node.get()) {
            entry.sub_index = Some(sub_index);
        }
    }

    /// Stops monitoring a node.
    pub fn remove(&mut self, node: NodeId) {
        self.monitored.remove(&node.get());
    }

    /// Iterates `(node, timeout_ms)` pairs currently configured.
    pub fn configured(&self) -> impl Iterator<Item = (NodeId, u32)> + '_ {
        self.monitored
            .iter()
            .map(|(&id, e)| (NodeId::new(id).expect("stored ids are valid"), e.timeout_ms))
    }

    /// Feeds one heartbeat/boot-up frame payload (`data` = state byte,
    /// possibly with guarding toggle which is masked here).
    ///
    /// Frames from unconfigured nodes are ignored.
    ///
    /// An undecodable state byte is silently dropped as a *state* — no
    /// `Started`/`StateChanged` event — but it still refreshes the
    /// supervision deadline: CANopenNode resets `timeoutTimer` for any
    /// DLC==1 heartbeat because it casts the raw byte without validation
    /// (`CO_HBconsumer.c:43-46,335,346`). Without the refresh a node
    /// emitting malformed heartbeats was judged `Lost` (docs/DESIGN-DECISIONS.md NMT-N1).
    pub fn on_heartbeat(&mut self, node: NodeId, data: &[u8], now_ms: u64) -> Vec<HbEvent> {
        // docs/DESIGN-DECISIONS.md NMT-04: only DLC==1 frames are heartbeats. The reference's
        // receive filter drops everything else (`CO_HBconsumer.c:43`: `if
        // (DLC == 1U)`), and `Node::on_frame` applies that gate — but this
        // public API was reachable directly with a longer slice, which was
        // then treated as a perfectly good heartbeat.
        if data.len() != 1 {
            return Vec::new();
        }
        let Some(entry) = self.monitored.get_mut(&node.get()) else {
            return Vec::new();
        };
        entry.last_seen_ms = now_ms;
        let Some((state, _toggle)) = decode_heartbeat(data) else {
            return Vec::new();
        };

        let mut events = Vec::new();
        if state == NmtState::Initialising {
            // Boot-up frame from a configured (tracked) node: remote reset.
            // CANopenNode `CO_HBconsumer.c:320-334` fires the
            // `pFunctSignalRemoteReset` callback for *every* non-UNCONFIGURED
            // entry — UNKNOWN (never seen since configuration) included; only
            // the accompanying EMCY report is gated on `HBstate == ACTIVE`.
            // Entries only exist for monitored nodes, so an event always
            // fires here (docs/DESIGN-DECISIONS.md B2).
            events.push(HbEvent::RemoteReset { node });
            entry.phase = Phase::Unknown;
            entry.state = None;
            return events;
        }
        if entry.phase != Phase::Active {
            entry.phase = Phase::Active;
            events.push(HbEvent::Started { node });
        }
        if entry.state != Some(state) {
            events.push(HbEvent::StateChanged {
                node,
                new_state: state,
            });
            entry.state = Some(state);
        }
        events
    }

    /// Drives timeout detection; call periodically with the current time.
    pub fn poll(&mut self, now_ms: u64) -> Vec<HbEvent> {
        let mut events = Vec::new();
        for (&id, entry) in &mut self.monitored {
            // A zero timeout disables supervision of that node entirely
            // (CANopenNode skips entries with `time_us == 0`).
            if entry.timeout_ms > 0
                && entry.phase == Phase::Active
                && now_ms.saturating_sub(entry.last_seen_ms)
                    >= Self::deadline_ms(entry.timeout_ms, self.tolerance_pct)
            {
                entry.phase = Phase::TimedOut;
                // CANopenNode `CO_HBconsumer.c:364-366`: a timed-out entry's
                // NMT state becomes *unknown*; `state_of` must not keep
                // reporting the pre-timeout state, and a recovery to the same
                // state must re-announce `StateChanged` (`getNmtState` only
                // returns the state while ACTIVE, `:480-484`).
                entry.state = None;
                events.push(HbEvent::Lost {
                    node: NodeId::new(id).expect("stored ids are valid"),
                });
            }
        }
        events
    }

    /// Last reported state, if any heartbeat was seen since the latest
    /// (re-)start/reset.
    #[must_use]
    pub fn state_of(&self, node: NodeId) -> Option<NmtState> {
        self.monitored.get(&node.get())?.state
    }

    /// 0-based 0x1016 array index of a monitored entry, when known. This is
    /// the counterpart of CANopenNode's loop counter `i` and the value used
    /// as the 0x8130 infoCode (`CO_HBconsumer.c:364`). Programmatic users
    /// that did not come through 0x1016 fall back to node-id ordering.
    #[must_use]
    pub fn entry_index(&self, node: NodeId) -> Option<u16> {
        let entry = self.monitored.get(&node.get())?;
        entry.sub_index.or_else(|| {
            self.monitored
                .keys()
                .position(|id| *id == node.get())
                .and_then(|i| u16::try_from(i).ok())
        })
    }

    /// Whether `node` is under active supervision (a non-zero timeout is
    /// configured). Only supervised nodes' `0x700+n` frames belong to the
    /// consumer — anything else in that COB-ID range must stay available
    /// for custom RPDO/SYNC configurations (`docs/DESIGN-DECISIONS.md` P3, mirroring
    /// python-canopen's per-node subscription and CANopenNode's
    /// per-entry rx filter in `CO_HBconsumer.c`).
    #[must_use]
    pub fn supervises(&self, node: NodeId) -> bool {
        self.monitored
            .get(&node.get())
            .is_some_and(|entry| entry.timeout_ms > 0)
    }

    /// Whether the node is currently alive (phase Active).
    #[must_use]
    pub fn is_active(&self, node: NodeId) -> bool {
        self.monitored
            .get(&node.get())
            .is_some_and(|e| e.phase == Phase::Active)
    }

    /// Whether at least one node is monitored and every monitored node is
    /// alive (vacuous truth would make safety gates useless when nothing is
    /// configured).
    #[must_use]
    pub fn all_active(&self) -> bool {
        !self.monitored.is_empty() && self.monitored.values().all(|e| e.phase == Phase::Active)
    }

    /// Whether no node is monitored at all (0x1016 has no active entries).
    ///
    /// Distinct from the negation of [`Self::all_active`] (which is also
    /// false while configured nodes have never been seen): clearing a
    /// consumer-bus EMCY condition treats an empty monitoring set as
    /// "nothing outstanding" (`docs/DESIGN-DECISIONS.md` 审-N1).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.monitored.is_empty()
    }

    /// Whether at least one node is monitored and every monitored node is
    /// alive **and** reports Operational.
    #[must_use]
    pub fn all_operational(&self) -> bool {
        !self.monitored.is_empty()
            && self
                .monitored
                .values()
                .all(|e| e.phase == Phase::Active && e.state == Some(NmtState::Operational))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_frames_match_known_bytes() {
        // Start node 5: [0x01, 0x05]; broadcast reset: [0x81, 0x00].
        let n5 = NodeId::new(5).unwrap();
        assert_eq!(
            encode_command(NmtCommand::StartRemoteNode, Some(n5)),
            [0x01, 0x05]
        );
        assert_eq!(encode_command(NmtCommand::ResetNode, None), [0x81, 0x00]);
        assert_eq!(
            decode_command(&[0x82, 0x07]),
            Some((NmtCommand::ResetCommunication, 7))
        );
        assert!(decode_command(&[0x99]).is_none());
        assert!(decode_command(&[0xAB, 0x05]).is_none()); // unknown command
                                                          // docs/DESIGN-DECISIONS.md m13: targets above the CiA node-id range are invalid.
        assert!(decode_command(&[0x01, 0x80]).is_none());
        assert!(decode_command(&[0x01, 0xFF]).is_none());
    }

    #[test]
    fn state_machine_transitions() {
        let mut nmt = NmtStateMachine::new();
        assert_eq!(nmt.state(), NmtState::Initialising);

        // docs/DESIGN-DECISIONS.md NMT-14: unlike Stop/EnterPreOperational (which have no
        // meaning during Initialising), a Start is honoured right away — CiA
        // 301 wants a command that races application start-up to be applied,
        // and both references do so unconditionally
        // (`CO_NMT_Heartbeat.c:207-220`、python `nmt.py:57-66`).
        assert!(nmt.apply(NmtCommand::StartRemoteNode));
        assert_eq!(nmt.state(), NmtState::Operational);

        // boot() from Operational is a no-op (the node is already started).
        assert!(!nmt.boot());
        assert_eq!(nmt.state(), NmtState::Operational);
        nmt.apply(NmtCommand::ResetCommunication);
        assert_eq!(nmt.state(), NmtState::Initialising);
        assert!(nmt.boot());
        assert_eq!(nmt.state(), NmtState::PreOperational);
        assert!(!nmt.boot()); // second boot is a no-op

        assert!(nmt.apply(NmtCommand::StartRemoteNode));
        assert_eq!(nmt.state(), NmtState::Operational);

        // Re-entering the current state is a no-op.
        assert!(!nmt.apply(NmtCommand::StartRemoteNode));
        assert_eq!(nmt.state(), NmtState::Operational);

        assert!(nmt.apply(NmtCommand::StopRemoteNode));
        assert_eq!(nmt.state(), NmtState::Stopped);

        assert!(nmt.apply(NmtCommand::EnterPreOperational));
        assert_eq!(nmt.state(), NmtState::PreOperational);

        // Resets go back to Initialising from anywhere.
        assert!(nmt.apply(NmtCommand::ResetCommunication));
        assert_eq!(nmt.state(), NmtState::Initialising);
        // Reset while already initialising: no change reported.
        assert!(!nmt.apply(NmtCommand::ResetNode));
    }

    #[test]
    fn heartbeat_and_guard_frames() {
        // Boot-up payload is exactly one zero byte.
        assert_eq!(encode_heartbeat(NmtState::Initialising), [0x00]);
        assert_eq!(encode_heartbeat(NmtState::Operational), [0x05]);

        // Guarding response alternates bit 7; decode masks it out.
        assert_eq!(encode_node_guard(NmtState::Stopped, false), [0x04]);
        assert_eq!(encode_node_guard(NmtState::Stopped, true), [0x84]);
        assert_eq!(
            decode_heartbeat(&[0x85]),
            Some((NmtState::Operational, true))
        );
        assert_eq!(decode_heartbeat(&[]), None);
        assert_eq!(decode_heartbeat(&[0x42]), None); // invalid state nibble
    }

    fn consumer_with_node(timeout_ms: u32) -> (HeartbeatConsumer, NodeId) {
        let mut c = HeartbeatConsumer::new();
        let n = NodeId::new(9).unwrap();
        c.set_timeout(n, timeout_ms);
        (c, n)
    }

    #[test]
    fn consumer_reports_started_then_lost_then_recovery() {
        let (mut c, n) = consumer_with_node(100);

        // First heartbeat → Started + StateChanged.
        let ev = c.on_heartbeat(n, &[0x7F], 10);
        assert!(ev.contains(&HbEvent::Started { node: n }));
        assert!(ev.contains(&HbEvent::StateChanged {
            node: n,
            new_state: NmtState::PreOperational
        }));

        // Within window: nothing.
        assert!(c.poll(90).is_empty());
        assert!(c.is_active(n));

        // Past deadline (>100ms since last beat): Lost.
        assert_eq!(c.poll(115), vec![HbEvent::Lost { node: n }]);
        assert!(!c.is_active(n));

        // Recovery: next heartbeat restarts monitoring and reports the new
        // state (was PreOperational before the loss).
        let ev = c.on_heartbeat(n, &[0x05], 200);
        assert!(ev.contains(&HbEvent::Started { node: n }));
        assert!(ev.contains(&HbEvent::StateChanged {
            node: n,
            new_state: NmtState::Operational
        }));
        assert!(c.all_operational());
    }

    #[test]
    fn malformed_heartbeat_refreshes_the_deadline_without_events() {
        // docs/DESIGN-DECISIONS.md NMT-N1: CANopenNode resets the timeout timer for *any*
        // DLC==1 heartbeat because it casts the raw byte without validation
        // (`CO_HBconsumer.c:43-46,335,346`); only the state update is
        // skipped for undecodable bytes.
        let (mut c, n) = consumer_with_node(100);
        let _ = c.on_heartbeat(n, &[0x05], 10);
        assert!(c.is_active(n));

        // 0x42 is not a valid state: no event, but the beat counts as life.
        assert!(c.on_heartbeat(n, &[0x42], 50).is_empty());
        assert!(c.poll(100).is_empty(), "deadline refreshed at t=50");
        assert!(c.is_active(n));

        // Silence from the last (malformed) beat does time out on schedule.
        assert_eq!(c.poll(160), vec![HbEvent::Lost { node: n }]);

        // A node that only ever sent malformed bytes is never ACTIVE and
        // therefore never times out (no Started either) — the remaining
        // intentional narrowing vs CANopenNode, cf. docs/DESIGN-DECISIONS.md adjudications.
        let (mut c, n) = consumer_with_node(100);
        assert!(c.on_heartbeat(n, &[0x42], 10).is_empty());
        assert!(!c.is_active(n));
        assert!(c.poll(200).is_empty());
    }

    #[test]
    fn consumer_detects_remote_reset() {
        let (mut c, n) = consumer_with_node(500);
        c.on_heartbeat(n, &[0x05], 10);
        // Boot-up frame (state 0x00) from a live node → RemoteReset.
        let ev = c.on_heartbeat(n, &[0x00], 20);
        assert_eq!(ev, vec![HbEvent::RemoteReset { node: n }]);
        assert!(!c.is_active(n)); // back to UNKNOWN until next beat
        assert_eq!(c.state_of(n), None);
    }

    #[test]
    fn consumer_ignores_unconfigured_nodes() {
        let mut c = HeartbeatConsumer::new();
        let stranger = NodeId::new(42).unwrap();
        assert!(c.on_heartbeat(stranger, &[0x05], 0).is_empty());
        assert!(c.poll(u64::MAX).is_empty());
        assert!(!c.all_active());
        c.remove(stranger);
    }

    #[test]
    fn all_flags_aggregate_correctly() {
        let mut c = HeartbeatConsumer::new();
        let a = NodeId::new(1).unwrap();
        let b = NodeId::new(2).unwrap();
        c.set_timeout(a, 100);
        c.set_timeout(b, 100);
        assert!(!c.all_active()); // nothing seen yet
        c.on_heartbeat(a, &[0x05], 0);
        c.on_heartbeat(b, &[0x04], 0);
        assert!(c.all_active());
        assert!(!c.all_operational()); // b is stopped
        c.on_heartbeat(b, &[0x05], 50);
        assert!(c.all_operational());
    }

    #[test]
    fn set_timeout_zero_fully_disables_supervision() {
        // docs/DESIGN-DECISIONS.md 十M5: the documented "0 disables supervision" must remove
        // the entry, not leave a zombie that `configured()`/`all_active()`
        // keep reporting.
        let mut c = HeartbeatConsumer::new();
        let n = NodeId::new(3).unwrap();
        c.set_timeout(n, 100);
        c.on_heartbeat(n, &[0x05], 10);
        assert!(c.all_active());
        assert_eq!(c.state_of(n), Some(NmtState::Operational));

        c.set_timeout(n, 0);
        assert!(!c.supervises(n));
        assert!(c.configured().next().is_none());
        assert!(!c.all_active());
        assert_eq!(c.state_of(n), None);
        assert!(c.poll(10_000).is_empty());
    }

    #[test]
    fn entry_index_follows_0x1016_subindex_not_node_order() {
        let mut c = HeartbeatConsumer::new();
        let nine = NodeId::new(9).unwrap();
        let seven = NodeId::new(7).unwrap();
        c.set_timeout_with_index(nine, 100, 0);
        c.set_timeout_with_index(seven, 100, 1);
        assert_eq!(c.entry_index(nine), Some(0));
        assert_eq!(c.entry_index(seven), Some(1));
    }

    #[test]
    fn configured_lists_timeouts() {
        let (c, n) = consumer_with_node(250);
        assert_eq!(c.configured().collect::<Vec<_>>(), vec![(n, 250)]);
    }
}
