//! High-level master API: a [`Network`] with COB-ID subscriber dispatch,
//! heartbeat supervision and SDO/NMT convenience — the Rust counterpart of
//! python-canopen's `Network`/`RemoteNode` shape (docs/IMPLEMENTATION.md
//! §10.3) on the blocking transport.
//!
//! Two sockets are opened per network: one owned by the RX dispatch thread
//! (subscribers + heartbeat consumer), one mutex-guarded for command
//! traffic (`sdo_read`/`sdo_write`/`send_nmt`). Both receive all bus frames;
//! command sends are therefore visible to subscribers as well.
//!
//! Robustness policies carried over from python-canopen (docs/REFERENCES.md
//! P2/P3): subscriber callbacks run in isolation — a panicking handler is
//! caught per-callback (counted via [`Network::handler_panic_count`],
//! `docs/DESIGN-DECISIONS.md` P1-4), and [`Network::check`] re-raises a crashed RX thread to
//! the synchronous caller.
//!
//! Two contracts applications should know about (`docs/DESIGN-DECISIONS.md` P3-61):
//!
//! - A **non-timeout I/O error** on the socket (e.g. the interface going
//!   down) permanently ends the RX thread; it surfaces through
//!   [`Network::check`] rather than recovering silently.
//! - Subscriber callbacks run on the RX thread and should return quickly;
//!   a slow callback delays heartbeat supervision by exactly its runtime.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use canopen_core::datatypes::{DataType, Value};
use canopen_core::gateway::{GatewayAction, GatewayErrorCode, GatewayFailure};
use canopen_core::nmt::{HbEvent, HeartbeatConsumer, NmtCommand};
use canopen_core::types::{Address, NodeId};

use crate::error::HostError;
use crate::transport::{Incoming, SocketCan};

type Subscriber = std::sync::Arc<dyn Fn(&crate::transport::Received) + Send + Sync>;

/// CiA 301 parameter group for 0x1010/0x1011 (`1` = all, `2` =
/// communication, `3` = application, `4..=127` = manufacturer-specific).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StorageGroup(u8);

impl StorageGroup {
    /// Group 1: all parameters.
    pub const ALL: Self = Self(1);
    /// Group 2: communication-profile parameters.
    pub const COMMUNICATION: Self = Self(2);
    /// Group 3: application-profile parameters.
    pub const APPLICATION: Self = Self(3);

    /// Validates and constructs a group number (`1..=127`).
    ///
    /// # Errors
    /// [`StorageGroupError`] for `0` or `>127`.
    pub fn new(group: u8) -> Result<Self, StorageGroupError> {
        Self::try_from(group)
    }

    /// Raw CiA 301 group number.
    #[must_use]
    pub const fn get(self) -> u8 {
        self.0
    }
}

impl From<StorageGroup> for u8 {
    fn from(group: StorageGroup) -> Self {
        group.0
    }
}

/// Error returned when a storage group is outside `1..=127`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StorageGroupError;

impl std::fmt::Display for StorageGroupError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("storage group must be in 1..=127")
    }
}

impl std::error::Error for StorageGroupError {}

impl TryFrom<u8> for StorageGroup {
    type Error = StorageGroupError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        (1..=127)
            .contains(&value)
            .then_some(Self(value))
            .ok_or(StorageGroupError)
    }
}

/// CiA 301 parameter-save signature written to 0x1010:1 (ASCII "save").
const SAVE_SIGNATURE: u32 = u32::from_le_bytes(*b"save");
/// CiA 301 parameter-restore signature written to 0x1011:1 (ASCII "load").
const RESTORE_SIGNATURE: u32 = u32::from_le_bytes(*b"load");

/// Upper bound for the buffered heartbeat-event queue (`docs/DESIGN-DECISIONS.md` P3-62):
/// an application that never drains must not grow the process without
/// limit; the oldest events are dropped once full.
const HB_EVENTS_CAP: usize = 1024;

/// Pushes `events` into the shared queue, dropping the oldest entries past
/// [`HB_EVENTS_CAP`].
fn push_bounded(
    queue: &Mutex<std::collections::VecDeque<canopen_core::nmt::HbEvent>>,
    events: Vec<canopen_core::nmt::HbEvent>,
) {
    let mut q = lock_recovered(queue.lock());
    for e in events {
        if q.len() >= HB_EVENTS_CAP {
            q.pop_front();
        }
        q.push_back(e);
    }
}

/// Dispatches one received frame to all exact-match handlers with per-handler
/// panic isolation (`docs/DESIGN-DECISIONS.md` P1-4): a panicking callback must not take down
/// the RX thread — remaining subscribers still run and the panic is counted.
/// Returns the number of handler panics observed for this frame.
fn dispatch_isolated(handlers: &[Subscriber], rx: &crate::transport::Received) -> usize {
    let mut panics = 0;
    for handler in handlers {
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| handler(rx)));
        if outcome.is_err() {
            panics += 1;
        }
    }
    panics
}

/// Takes a std sync lock, recovering from poisoning (`docs/DESIGN-DECISIONS.md` P2-3): a
/// panic in one task while it held the lock must not cascade into panics at
/// every later accessor — the guarded state stays usable afterwards.
fn lock_recovered<T>(lock: std::sync::LockResult<T>) -> T {
    lock.unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// Wire payload for [`RemoteNode::write_int`]: minimal-width signed-aware
/// encoding, byte-identical to python-canopen's untyped `download()`
/// (`docs/DESIGN-DECISIONS.md` P2-1 — a fixed 4-byte pad here had regressed every U8/U16
/// target; see git history for the full post-mortem).
#[must_use]
pub fn encode_write_int(value: i64) -> Vec<u8> {
    let mut buf = [0u8; 8];
    let n = canopen_core::datatypes::Value::Int(value)
        .encode_le(&mut buf)
        .expect("i64 always fits 8 bytes");
    buf[..n].to_vec()
}

/// Wire payload for [`RemoteNode::write_int_as`]: exactly
/// `dt.fixed_size()` bytes, with negative values bit-cast to unsigned
/// targets at that width (two's complement).
///
/// # Errors
/// [`canopen_core::types::Error::Other`] when `dt` has no fixed width.
pub fn encode_write_int_as(
    value: i64,
    dt: canopen_core::datatypes::DataType,
) -> Result<Vec<u8>, canopen_core::types::Error> {
    use canopen_core::datatypes::{DataType as Dt, Value};
    // docs/DESIGN-DECISIONS.md H2: a floating-point target is not an integer encoding —
    // routing it through the unsigned fallback below produced a confusing
    // `TypeMismatch` deep inside `encode_as`. Reject at the entry with a
    // targeted message instead.
    if matches!(dt, Dt::Real32 | Dt::Real64) {
        return Err(canopen_core::types::Error::Other(
            "write_int_as requires an integer/boolean type, got a floating-point target",
        ));
    }
    let size = dt
        .fixed_size()
        .filter(|&n| n <= 8)
        .ok_or(canopen_core::types::Error::Other(
            "write_int_as requires a fixed-width type",
        ))?;
    let typed = match dt {
        Dt::Boolean => Value::Bool(value != 0),
        Dt::Integer8
        | Dt::Integer16
        | Dt::Integer24
        | Dt::Integer32
        | Dt::Integer40
        | Dt::Integer48
        | Dt::Integer56
        | Dt::Integer64 => Value::Int(value),
        // Unsigned targets: truncate to the declared width in two's
        // complement so e.g. `-1 as Unsigned16` becomes 0xFFFF instead of
        // tripping the range check with a full-width u64. A uniform width
        // mask covers every fixed size (docs/DESIGN-DECISIONS.md M5 — previously only
        // 1/2/4-byte types were handled and Unsigned24/40/48/56 rejected
        // negative values outright).
        _ => {
            let mask = if size >= 8 {
                u64::MAX
            } else {
                (1u64 << (size * 8)) - 1
            };
            Value::Uint((value as u64) & mask)
        }
    };
    let mut buf = vec![0u8; size];
    typed.encode_as(dt, &mut buf)?;
    Ok(buf)
}

/// How the RX thread died, if it did.
#[derive(Debug)]
enum RxDeath {
    Panic(String),
    Io(String),
}

/// One CAN bus with node registry semantics.
pub struct Network {
    /// Command socket, guarded so concurrent masters serialize.
    cmd: Arc<Mutex<SocketCan>>,
    /// COB-ID → callbacks (exact match, like python-canopen).
    subscribers: Arc<Mutex<BTreeMap<u16, Vec<Subscriber>>>>,
    /// Heartbeat supervision shared between the RX thread and queries.
    hb: Arc<Mutex<HeartbeatConsumer>>,
    /// Heartbeat events produced by the RX thread (`Started`/`Lost`/…),
    /// drained by [`Network::heartbeat_events`] (`docs/DESIGN-DECISIONS.md` P1-9). The
    /// consumer is edge-triggered, so events must be buffered where they
    /// are generated or the application never sees them.
    hb_events: Arc<Mutex<std::collections::VecDeque<HbEvent>>>,
    /// Persistent CiA 309-3 gateway parser (`set node` survives calls).
    gateway: Mutex<canopen_core::gateway::Gateway>,
    /// RX thread liveness: set when the thread died unexpectedly.
    rx_state: Arc<Mutex<Option<RxDeath>>>,
    rx_shutdown: Arc<AtomicBool>,
    /// Join slot behind a mutex so [`Network::shutdown_shared`] can retire
    /// the RX thread through an `Arc` handle (`docs/DESIGN-DECISIONS.md` P3-57).
    rx_handle: Mutex<Option<std::thread::JoinHandle<()>>>,
    /// Cumulative count of subscriber callbacks that panicked, incremented
    /// by the RX thread (`docs/DESIGN-DECISIONS.md` P1-4); observable via
    /// [`Network::handler_panic_count`].
    handler_panics: Arc<AtomicUsize>,
}

impl Network {
    /// Opens both sockets on `iface` and starts the RX dispatch thread.
    ///
    /// # Errors
    /// Propagates transport open errors ([`HostError::InterfaceNotFound`]
    /// etc.).
    pub fn open(iface: &str) -> Result<Self, HostError> {
        let rx_sock = SocketCan::open(iface)?;
        let cmd_sock = SocketCan::open(iface)?;
        let mut network = Self {
            cmd: Arc::new(Mutex::new(cmd_sock)),
            subscribers: Arc::new(Mutex::new(BTreeMap::new())),
            hb: Arc::new(Mutex::new(HeartbeatConsumer::new())),
            hb_events: Arc::new(Mutex::new(std::collections::VecDeque::new())),
            gateway: Mutex::new(canopen_core::gateway::Gateway::new()),
            rx_state: Arc::new(Mutex::new(None)),
            rx_shutdown: Arc::new(AtomicBool::new(false)),
            rx_handle: Mutex::new(None),
            handler_panics: Arc::new(AtomicUsize::new(0)),
        };
        network.spawn_rx(rx_sock);
        Ok(network)
    }

    fn spawn_rx(&mut self, mut sock: SocketCan) {
        let subscribers = self.subscribers.clone();
        let hb = self.hb.clone();
        let hb_events = self.hb_events.clone();
        let state = self.rx_state.clone();
        let shutdown = self.rx_shutdown.clone();
        let handler_panics = self.handler_panics.clone();
        *lock_recovered(self.rx_handle.lock()) = Some(std::thread::spawn(move || {
            let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(move || {
                while !shutdown.load(Ordering::Relaxed) {
                    // Buffer edge-triggered events instead of dropping them
                    // (`docs/DESIGN-DECISIONS.md` P1-9): the application drains them via
                    // [`Network::heartbeat_events`].
                    let timed_out = lock_recovered(hb.lock()).poll(crate::transport::now_ms());
                    push_bounded(&hb_events, timed_out);
                    let Some(incoming) = sock.recv(Duration::from_millis(10))? else {
                        continue;
                    };
                    // Remote frames are deliberately not surfaced to
                    // subscribers: node-guarding observation belongs to a
                    // dedicated master feature, not the generic dispatch
                    // (`docs/DESIGN-DECISIONS.md` P3-64, resolved by documentation).
                    let Incoming::Data(rx) = incoming else {
                        continue;
                    };
                    // Heartbeat supervision first (internal bookkeeping).
                    // A heartbeat frame is exactly 1 byte on the wire
                    // (CANopenNode `CO_HBconsumer.c:43`), same rule as the
                    // core path (`core/src/node.rs`, docs/DESIGN-DECISIONS.md C5).
                    if is_heartbeat_frame(rx.cob_id, rx.payload()) {
                        if let Ok(node) = NodeId::try_from((rx.cob_id - 0x700) as u8) {
                            let produced = lock_recovered(hb.lock()).on_heartbeat(
                                node,
                                rx.payload(),
                                crate::transport::now_ms(),
                            );
                            push_bounded(&hb_events, produced);
                        }
                    }
                    // Exact-match subscriber dispatch; one failing callback
                    // must not kill the loop (`docs/DESIGN-DECISIONS.md` P1-4): handlers run
                    // with per-callback panic isolation and are cloned out
                    // so a callback that re-enters subscribe/unsubscribe (or
                    // any other lock-taking Network call) cannot deadlock.
                    let handlers = {
                        let subs = lock_recovered(subscribers.lock());
                        subs.get(&rx.cob_id).cloned().unwrap_or_default()
                    };
                    let panics = dispatch_isolated(&handlers, &rx);
                    if panics > 0 {
                        handler_panics.fetch_add(panics, Ordering::Relaxed);
                    }
                }
                Ok::<(), HostError>(())
            }));
            let death = match outcome {
                Ok(Ok(())) => None,
                Ok(Err(e)) => Some(RxDeath::Io(e.to_string())),
                Err(payload) => {
                    let msg = payload
                        .downcast_ref::<String>()
                        .cloned()
                        .or_else(|| payload.downcast_ref::<&str>().map(|s| (*s).to_string()))
                        .unwrap_or_else(|| "unknown panic".to_string());
                    Some(RxDeath::Panic(msg))
                }
            };
            if let Some(death) = death {
                *lock_recovered(state.lock()) = Some(death);
            }
        }));
    }

    /// Registers a callback for exact-COB-ID traffic. Duplicate registration
    /// of the same id appends another handler (python-canopen dedupes; we
    /// allow multiple observers and document it).
    ///
    /// # Loopback note (`docs/DESIGN-DECISIONS.md` H2)
    ///
    /// SocketCAN sockets loop frames back by default, so frames **this
    /// process transmits** (e.g. an SDO request on `0x600+node` sent via the
    /// command socket) are also delivered to the RX thread. Subscribing to a
    /// COB-ID the master itself sends therefore fires callbacks on the
    /// self-echo as well; applications that must ignore it should filter by
    /// payload or compare the frame against what they last transmitted.
    pub fn subscribe<F>(&self, cob_id: u16, callback: F)
    where
        F: Fn(&crate::transport::Received) + Send + Sync + 'static,
    {
        lock_recovered(self.subscribers.lock())
            .entry(cob_id)
            .or_default()
            .push(std::sync::Arc::new(callback));
    }

    /// Removes all callbacks for `cob_id`; returns how many were removed.
    pub fn unsubscribe(&self, cob_id: u16) -> usize {
        lock_recovered(self.subscribers.lock())
            .remove(&cob_id)
            .map_or(0, |handlers| handlers.len())
    }

    /// Starts heartbeat supervision of `node` with `timeout_ms`.
    pub fn monitor_heartbeat(&self, node: NodeId, timeout_ms: u32) {
        lock_recovered(self.hb.lock()).set_timeout(node, timeout_ms);
    }

    /// Drains pending heartbeat events (Started/Lost/RemoteReset/StateChanged).
    ///
    /// Events are captured by the RX thread into an internal queue as they
    /// occur — the consumer is edge-triggered (a node transitions to
    /// `TimedOut` exactly once per loss), so this is the only reliable way
    /// for the application to observe them (`docs/DESIGN-DECISIONS.md` P1-9). A final
    /// application-thread poll catches deadlines that elapsed since the
    /// last RX pass; its output is appended after the queued events.
    pub fn heartbeat_events(&self) -> Vec<HbEvent> {
        let mut out: Vec<HbEvent> = {
            let mut queue = lock_recovered(self.hb_events.lock());
            std::mem::take(&mut *queue).into_iter().collect()
        };
        out.extend(lock_recovered(self.hb.lock()).poll(crate::transport::now_ms()));
        out
    }

    /// Last reported NMT state of `node`, when known.
    #[must_use]
    pub fn node_state(&self, node: NodeId) -> Option<canopen_core::nmt::NmtState> {
        lock_recovered(self.hb.lock()).state_of(node)
    }

    /// Whether every monitored node is alive.
    #[must_use]
    pub fn all_nodes_active(&self) -> bool {
        lock_recovered(self.hb.lock()).all_active()
    }

    /// Re-raises an RX-thread crash to the caller (P3). Call periodically
    /// from long-lived masters; every command checks implicitly.
    ///
    /// # Errors
    /// Propagates the stored panic message / IO error as [`HostError::Rx`].
    pub fn check(&self) -> Result<(), HostError> {
        match &*lock_recovered(self.rx_state.lock()) {
            None => Ok(()),
            Some(RxDeath::Panic(msg)) => Err(HostError::Rx(format!("panic: {msg}"))),
            Some(RxDeath::Io(msg)) => Err(HostError::Rx(format!("io: {msg}"))),
        }
    }

    /// Cumulative number of subscriber callbacks that panicked since the
    /// network was opened (`docs/DESIGN-DECISIONS.md` P1-4): panicking handlers are isolated
    /// so the RX thread survives, and this counter makes the failures
    /// observable to the application.
    #[must_use]
    pub fn handler_panic_count(&self) -> usize {
        self.handler_panics.load(Ordering::Relaxed)
    }

    /// Runs `f` with exclusive access to the command socket.
    pub fn with_cmd<T>(
        &self,
        f: impl FnOnce(&mut SocketCan) -> Result<T, HostError>,
    ) -> Result<T, HostError> {
        self.check()?;
        let mut guard = lock_recovered(self.cmd.lock());
        f(&mut guard)
    }

    /// SDO read through the command socket.
    ///
    /// # Errors
    /// Transport errors; see [`SocketCan::sdo_read`].
    pub fn sdo_read(&self, node: u8, addr: Address, dt: DataType) -> Result<Value, HostError> {
        self.with_cmd(|bus| bus.sdo_read(node, addr, dt))
    }

    /// SDO write through the command socket.
    ///
    /// # Errors
    /// Transport errors; see [`SocketCan::sdo_write`].
    pub fn sdo_write(&self, node: u8, addr: Address, data: &[u8]) -> Result<(), HostError> {
        self.with_cmd(|bus| bus.sdo_write(node, addr, data))
    }

    /// Sends an NMT command.
    ///
    /// # Errors
    /// Transport errors; see [`SocketCan::send_nmt`].
    pub fn send_nmt(&self, command: NmtCommand, target: Option<u8>) -> Result<(), HostError> {
        self.with_cmd(|bus| bus.send_nmt(command, target))
    }

    /// Sends a raw frame (e.g. SYNC producer).
    ///
    /// # Errors
    /// Transport errors.
    pub fn send_raw(&self, cob_id: u16, data: &[u8]) -> Result<(), HostError> {
        self.with_cmd(|bus| bus.send(cob_id, data))
    }

    /// A handle bound to one node id for ergonomic remote operations.
    #[must_use]
    pub fn node(&self, id: u8) -> RemoteNode<'_> {
        RemoteNode { network: self, id }
    }

    /// Pulls the EDS file a device stores at object 0x1021 (*Store EDS*)
    /// over SDO and parses it. The transfer uses expedited/segmented/block
    /// automatically (CiA 306 files are typically several KiB).
    ///
    /// Parsing is lenient: recoverable findings (unknown options, invalid
    /// defaults, oversized `CompactSubObj`, …) land in
    /// [`Eds::warnings`](crate::eds::Eds::warnings) — callers should surface
    /// them. Turning the document into a dictionary can still fail on
    /// content errors; use [`Eds::build`](crate::eds::Eds::build) to receive
    /// them as [`HostError::Eds`] (docs/DESIGN-DECISIONS.md EDS-01/API-08).
    ///
    /// # Errors
    /// Transport errors; an SDO abort (e.g. the device does not implement
    /// 0x1021) surfaces as [`HostError::SdoAborted`] (`docs/DESIGN-DECISIONS.md` 十M24:
    /// previously the docs claimed `ProtocolIncomplete`, but the abort is
    /// mapped by `read_domain_bytes` before it could be folded).
    pub fn read_eds_from_device(&self, node: u8) -> Result<crate::eds::Eds, HostError> {
        self.with_cmd(|bus| {
            let bytes = bus.read_domain_bytes(node, Address::new(0x1021, 0))?;
            // EDS is ASCII, but vendor tools emit Latin-1 comments/names
            // (python-canopen decodes with a tolerant codec). Try UTF-8,
            // fall back to Latin-1 byte-for-byte mapping instead of failing.
            let text = match std::str::from_utf8(&bytes) {
                Ok(t) => t,
                Err(_) => &*bytes.iter().map(|&b| b as char).collect::<String>(),
            };
            crate::eds::parse(text).map_err(HostError::Eds)
        })
    }

    /// Stops the RX thread. Idempotent; called by [`Drop`](Self::drop).
    pub fn shutdown(&mut self) {
        self.rx_shutdown.store(true, Ordering::Relaxed);
        if let Some(handle) = lock_recovered(self.rx_handle.lock()).take() {
            let _ = handle.join();
        }
    }

    /// Stops the RX thread through a shared handle (`docs/DESIGN-DECISIONS.md` P3-57): the
    /// previous `shutdown(&mut self)` was unreachable for applications that
    /// hand `Arc<Network>` clones around (the owning drop never runs until
    /// process exit, leaving the RX thread polling forever).
    ///
    /// Idempotent: a second call finds no thread left to join.
    pub fn shutdown_shared(self: &Arc<Self>) {
        self.shutdown_arc_inner();
    }

    fn shutdown_arc_inner(self: &Arc<Self>) {
        self.rx_shutdown.store(true, Ordering::Relaxed);
        if let Some(handle) = lock_recovered(self.rx_handle.lock()).take() {
            let _ = handle.join();
        }
    }
}

impl Drop for Network {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Convenience wrapper binding a [`Network`] to one node id.
pub struct RemoteNode<'a> {
    network: &'a Network,
    id: u8,
}

impl RemoteNode<'_> {
    /// Typed SDO read.
    ///
    /// # Errors
    /// See [`Network::sdo_read`].
    pub fn sdo_read(&self, addr: Address, dt: DataType) -> Result<Value, HostError> {
        self.network.sdo_read(self.id, addr, dt)
    }

    /// Raw little-endian SDO write.
    ///
    /// # Errors
    /// See [`Network::sdo_write`].
    pub fn sdo_write(&self, addr: Address, data: &[u8]) -> Result<(), HostError> {
        self.network.sdo_write(self.id, addr, data)
    }

    /// Writes an integer value encoded at its *minimal* width (signed-aware),
    /// mirroring python-canopen's untyped `download()`.
    ///
    /// # Width semantics (`docs/DESIGN-DECISIONS.md` P2-1, see git history)
    ///
    /// SDO servers decode expedited downloads at the entry's **declared**
    /// width, so this succeeds only when the target's declared width equals
    /// the value's minimal width (e.g. `500` → 2 bytes fits an `Unsigned16`
    /// like 0x1017, but aborts on an `Unsigned32` object). For any other
    /// width use [`Self::write_int_as`] with the explicit type, or
    /// [`Self::sdo_write`] with hand-built bytes.
    ///
    /// # Errors
    /// Non-integer input, a width mismatch on the server, or transport
    /// errors.
    pub fn write_int(&self, addr: Address, value: i64) -> Result<(), HostError> {
        let payload = encode_write_int(value);
        self.network.sdo_write(self.id, addr, &payload)
    }

    /// Writes an integer encoded at an **explicitly given** type's width —
    /// the reliable variant of [`Self::write_int`] when the declared entry
    /// type is known. Negative values into unsigned targets are written as
    /// two's-complement at that width (`-1` → `[FF FF]` for `Unsigned16`).
    ///
    /// # Errors
    /// `dt` is not a fixed-width type, or the value cannot be represented /
    /// transport errors.
    pub fn write_int_as(&self, addr: Address, value: i64, dt: DataType) -> Result<(), HostError> {
        let payload = encode_write_int_as(value, dt).map_err(HostError::Core)?;
        self.network.sdo_write(self.id, addr, &payload)
    }

    /// NMT start this node.
    ///
    /// # Errors
    /// See [`Network::send_nmt`].
    pub fn start(&self) -> Result<(), HostError> {
        self.network
            .send_nmt(NmtCommand::StartRemoteNode, Some(self.id))
    }

    /// NMT stop this node.
    ///
    /// # Errors
    /// See [`Network::send_nmt`].
    pub fn stop(&self) -> Result<(), HostError> {
        self.network
            .send_nmt(NmtCommand::StopRemoteNode, Some(self.id))
    }

    /// Parameter storage: writes the confirm signature to 0x1010:1 ("save"
    /// = 0x65766173 as ASCII little-endian), CiA 301 standard save-all.
    ///
    /// # Errors
    /// Transport errors or an SDO abort when the device refuses.
    pub fn store(&self) -> Result<(), HostError> {
        self.store_group(StorageGroup::ALL)
    }

    /// Parameter storage for a specific CiA 301 group (`0x1010:<group>`).
    ///
    /// # Errors
    /// Transport errors or an SDO abort when the device refuses.
    pub fn store_group(&self, group: StorageGroup) -> Result<(), HostError> {
        self.sdo_write(
            Address::new(0x1010, group.get()),
            &SAVE_SIGNATURE.to_le_bytes(),
        )
    }

    /// Parameter restore: writes "load" (0x64616F6C) to 0x1011:1; defaults
    /// return after the next reset.
    ///
    /// # Errors
    /// Same as [`RemoteNode::store`].
    pub fn restore(&self) -> Result<(), HostError> {
        self.restore_group(StorageGroup::ALL)
    }

    /// Parameter restore for a specific CiA 301 group (`0x1011:<group>`).
    ///
    /// # Errors
    /// Same as [`RemoteNode::store_group`].
    pub fn restore_group(&self, group: StorageGroup) -> Result<(), HostError> {
        self.sdo_write(
            Address::new(0x1011, group.get()),
            &RESTORE_SIGNATURE.to_le_bytes(),
        )
    }
}

/// CiA 309-3 gateway execution bridge: parses ASCII lines through
/// `canopen_core::gateway::Gateway` and executes the resulting actions on
/// this network. `set node` state persists on the [`Network`] instance.
impl Network {
    /// Feeds one gateway line, executes any parsed action synchronously and
    /// returns the exact response line (without CR/LF).
    ///
    /// Failures are encoded as `ERROR:` frames per CiA 309-3 so a
    /// misbehaving client cannot kill the session; `Err` is never returned.
    /// A blank or comment-only line returns the empty string — the caller
    /// still frames it as `\r\n`, matching CANopenNode's `responseWithEmpty`
    /// (`G4`).
    pub fn gateway_line(&self, line: &str) -> String {
        use canopen_core::gateway::GatewayEvent;

        let mut gw = lock_recovered(self.gateway.lock());
        match gw.feed_line(line) {
            GatewayEvent::Respond(text) => text,
            GatewayEvent::Request(req) => match self.execute_gateway_action(&req.action) {
                Ok(Some(value)) => {
                    canopen_core::gateway::Gateway::format_value(req.sequence, &value)
                }
                Ok(None) => canopen_core::gateway::Gateway::format_ok(req.sequence),
                Err(failure) => {
                    canopen_core::gateway::Gateway::format_error(req.sequence, &failure)
                }
            },
            // `GatewayEvent` is `#[non_exhaustive]`: a future event variant
            // gets no response rather than breaking the build here.
            _ => String::new(),
        }
    }

    fn execute_gateway_action(
        &self,
        action: &GatewayAction,
    ) -> Result<Option<Value>, GatewayFailure> {
        use canopen_core::datatypes::DataType;
        match *action {
            GatewayAction::Read {
                node,
                index,
                sub,
                data_type,
            } => {
                // `GwDataType::Hex` maps to `DataType::Domain` (raw bytes),
                // so a read without an explicit type is an unfiltered hex
                // dump. The fallback is only for a future unmapped variant:
                // raw bytes beat guessing a fixed width (`G1`).
                let dt = data_type.to_core().unwrap_or(DataType::Domain);
                let addr = Address::new(index, sub);
                read_typed_with_hex_fallback(dt, |dt| self.sdo_read(node, addr, dt))
                    .map(Some)
                    .map_err(sdo_failure)
            }
            GatewayAction::Write {
                node,
                index,
                sub,
                ref payload,
                ..
            } => self
                .sdo_write(node, Address::new(index, sub), payload)
                .map(|_| None)
                .map_err(sdo_failure),
            GatewayAction::Nmt { command, node } => {
                let target = (node != 0).then_some(node);
                self.send_nmt(command.into(), target)
                    .map(|_| None)
                    .map_err(|_| GatewayFailure::Code(GatewayErrorCode::InternalState))
            }
            GatewayAction::SetNode(_) => Ok(None),
        }
    }
}

/// Reads an object with the requested data type, falling back to a raw hex
/// dump when the readback cannot be decoded at that width.
///
/// docs/DESIGN-DECISIONS.md GTW-15: a typed `read` whose response length differs from the
/// requested width is **not** an error. CANopenNode's fixed-width readback
/// helpers print the raw bytes unless `getOccupied == sizeof(n)`
/// (`CO_fifo_readU82a`, `CO_fifo.c:461-475`, and the U16/U32/X8 variants), so a
/// device that answers with a different length still yields a value — this
/// stack used to answer `ERROR:102` (internal state) and made debug scripts
/// non-portable.
///
/// Only a *decode* failure is retried: an aborted or timed-out transfer is a
/// different error and propagates unchanged. `DataType::Domain` (the default
/// `hex` read) has no fixed width, so it is never retried.
fn read_typed_with_hex_fallback<F>(dt: DataType, read: F) -> Result<Value, HostError>
where
    F: Fn(DataType) -> Result<Value, HostError>,
{
    match read(dt) {
        Ok(value) => Ok(value),
        Err(HostError::Core(canopen_core::types::Error::BadLength { .. }))
            if dt != DataType::Domain =>
        {
            read(DataType::Domain)
        }
        Err(e) => Err(e),
    }
}

fn sdo_failure(err: HostError) -> GatewayFailure {
    match err {
        HostError::SdoAborted(code) => GatewayFailure::SdoAborted(code),
        // CANopenNode's gateway SDO path reports a timeout as the standard
        // SDO abort `0x05040000` in hex (`CO_gateway_ascii.c:1571-1573`);
        // the numeric 103 is reserved for its LSS command family, which
        // this stack does not expose (docs/DESIGN-DECISIONS.md G8). Transparent passthrough
        // also keeps the response format uniform (`0x%08X`).
        HostError::Timeout => {
            GatewayFailure::SdoAborted(canopen_core::sdo::SdoAbortCode::ProtocolTimeout)
        }
        // A generic local IO/CAN failure is "not processed due to internal
        // state" (CiA 309-3 102); the numeric codes 200/300/301 all carry
        // specific standard meanings (lost guarding / error passive / bus
        // off) that this generic path cannot claim.
        _ => GatewayFailure::Code(GatewayErrorCode::InternalState),
    }
}

/// Whether `cob_id`/`data` form a well-formed heartbeat frame: the
/// `0x700 + node` identifier and exactly one payload byte (CANopenNode
/// `CO_HBconsumer.c:43` latches only DLC == 1; the core path enforces the
/// same rule, `docs/DESIGN-DECISIONS.md` C5).
fn is_heartbeat_frame(cob_id: u16, data: &[u8]) -> bool {
    (0x700..=0x77F).contains(&cob_id) && data.len() == 1
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicUsize;

    #[test]
    fn storage_groups_are_validated_and_named() {
        assert_eq!(StorageGroup::ALL.get(), 1);
        assert_eq!(StorageGroup::COMMUNICATION.get(), 2);
        assert_eq!(StorageGroup::APPLICATION.get(), 3);
        assert_eq!(StorageGroup::try_from(1).unwrap(), StorageGroup::ALL);
        assert_eq!(StorageGroup::try_from(127).unwrap().get(), 127);
        assert!(StorageGroup::try_from(0).is_err());
        assert!(StorageGroup::try_from(128).is_err());
    }

    #[test]
    fn heartbeat_frame_requires_one_byte() {
        assert!(is_heartbeat_frame(0x705, &[0x05]));
        assert!(!is_heartbeat_frame(0x705, &[]));
        assert!(!is_heartbeat_frame(0x705, &[0x05, 0x00]));
        assert!(!is_heartbeat_frame(0x185, &[0x05]));
    }

    /// Registry behaviour: multiple subscribers on one COB-ID all fire and
    /// removal stops delivery. Exercises the same code path the RX thread
    /// uses, without sockets.
    #[test]
    fn subscriber_registry_dispatch_and_removal() {
        let subs: Arc<Mutex<BTreeMap<u16, Vec<Subscriber>>>> =
            Arc::new(Mutex::new(BTreeMap::new()));
        let hits = Arc::new(AtomicUsize::new(0));

        let h = hits.clone();
        subs.lock()
            .unwrap()
            .entry(0x185)
            .or_default()
            .push(std::sync::Arc::new(move |_| {
                h.fetch_add(1, Ordering::Relaxed);
            }));

        let rx = crate::transport::Received {
            cob_id: 0x185,
            data: [1, 0, 0, 0, 0, 0, 0, 0],
            len: 1,
        };
        {
            let guards = subs.lock().unwrap();
            for handler in guards.get(&rx.cob_id).expect("registered") {
                handler(&rx);
            }
        }
        assert_eq!(hits.load(Ordering::Relaxed), 1);

        // Removal returns the handler count and stops delivery.
        let removed = subs.lock().unwrap().remove(&0x185).map_or(0, |v| v.len());
        assert_eq!(removed, 1);
        assert_eq!(hits.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn heartbeat_events_surface_through_the_network_path() {
        // HeartbeatConsumer semantics behind Network's supervision API.
        let mut hb = HeartbeatConsumer::new();
        let node = NodeId::new(7).unwrap();
        hb.set_timeout(node, 100);
        assert!(!hb.all_active()); // nothing seen yet
        assert!(!hb.on_heartbeat(node, &[0x05], 0).is_empty());
        assert!(hb.poll(50).is_empty());
        assert!(hb.all_active());
        match &hb.poll(200)[..] {
            [HbEvent::Lost { node: lost }] => assert_eq!(*lost, node),
            other => panic!("expected Lost, got {other:?}"),
        }
    }

    /// docs/DESIGN-DECISIONS.md GTW-15: a typed gateway read whose readback length does not
    /// match the requested width falls back to the raw hex dump, like
    /// `CO_fifo_readU82a` (`CO_fifo.c:461-475`).
    #[test]
    fn typed_read_falls_back_to_hex_on_length_mismatch() {
        use canopen_core::types::Error as CoreError;

        // Length mismatch → second read as raw bytes, printed like `hex`.
        let calls = std::cell::RefCell::new(Vec::new());
        let value = read_typed_with_hex_fallback(DataType::Unsigned16, |dt| {
            calls.borrow_mut().push(dt);
            if dt == DataType::Unsigned16 {
                Err(HostError::Core(CoreError::BadLength {
                    declared: 2,
                    actual: 4,
                }))
            } else {
                Ok(Value::Bytes(vec![0xde, 0xad, 0xbe, 0xef]))
            }
        })
        .expect("the hex fallback succeeds");
        assert_eq!(value, Value::Bytes(vec![0xde, 0xad, 0xbe, 0xef]));
        assert_eq!(
            *calls.borrow(),
            vec![DataType::Unsigned16, DataType::Domain]
        );
        // …and the response line is the plain hex dump, not `ERROR:102`.
        assert_eq!(
            canopen_core::gateway::Gateway::format_value(1, &value),
            "[1] DE AD BE EF"
        );

        // A matching read is served by the first transaction alone.
        let calls = std::cell::RefCell::new(0);
        let value = read_typed_with_hex_fallback(DataType::Unsigned16, |_| {
            *calls.borrow_mut() += 1;
            Ok(Value::Uint(500))
        })
        .expect("first read succeeds");
        assert_eq!(value, Value::Uint(500));
        assert_eq!(*calls.borrow(), 1);

        // An aborted transfer is *not* retried: the abort is the answer.
        let calls = std::cell::RefCell::new(0);
        let err = read_typed_with_hex_fallback(DataType::Unsigned16, |_| {
            *calls.borrow_mut() += 1;
            Err(HostError::SdoAborted(
                canopen_core::sdo::SdoAbortCode::ObjectDoesNotExist,
            ))
        })
        .expect_err("abort propagates");
        assert!(matches!(err, HostError::SdoAborted(_)));
        assert_eq!(*calls.borrow(), 1);

        // `hex` reads (Domain) are never retried either.
        let calls = std::cell::RefCell::new(0);
        let err = read_typed_with_hex_fallback(DataType::Domain, |_| {
            *calls.borrow_mut() += 1;
            Err(HostError::Core(CoreError::Other("nope")))
        })
        .expect_err("propagates");
        assert!(matches!(err, HostError::Core(_)));
        assert_eq!(*calls.borrow(), 1);
    }

    #[test]
    fn lock_recovered_survives_poisoned_mutex() {
        // docs/DESIGN-DECISIONS.md P2-3: lock acquisition must recover from a poisoned mutex
        // instead of cascading panics through every later accessor.
        let m = Arc::new(Mutex::new(7_u8));
        let joined = {
            let m2 = Arc::clone(&m);
            std::thread::spawn(move || {
                let _guard = m2.lock().unwrap();
                panic!("poison the mutex");
            })
            .join()
        };
        assert!(joined.is_err());
        assert_eq!(*lock_recovered(m.lock()), 7);
    }
}

#[cfg(test)]
mod p1_regressions {
    use super::*;

    /// docs/DESIGN-DECISIONS.md P1-3: the restore signature must be the CiA 301 "load"
    /// (0x64616F6C) on the wire — a transposed literal ("daol") would be
    /// silently ignored or aborted by real devices.
    #[test]
    fn store_restore_signatures_match_cia301_wire_bytes() {
        assert_eq!(SAVE_SIGNATURE.to_le_bytes(), *b"save");
        assert_eq!(SAVE_SIGNATURE, 0x6576_6173);
        assert_eq!(RESTORE_SIGNATURE.to_le_bytes(), *b"load");
        assert_eq!(RESTORE_SIGNATURE, 0x6461_6F6C);
    }

    /// docs/DESIGN-DECISIONS.md P2-1 (see git history): `write_int` is minimal-width (the
    /// fixed 4-byte pad regressed every U8/U16 target); explicit widths go
    /// through `encode_write_int_as`. Both are validated against a *real*
    /// server decoder — the first round only tested the encoder, which is
    /// exactly how the U16 regression slipped through.
    #[test]
    fn write_int_width_matrix_against_real_server() {
        use canopen_core::datatypes::DataType as Dt;
        use canopen_core::sdo::{decode_abort, encode_download_expedited, SdoServer};

        fn server_accepts(addr: Address, dt: Dt, payload: &[u8]) -> bool {
            let mut od = canopen_core::object_dictionary::ObjectDictionary::new();
            od.insert(
                addr,
                canopen_core::object_dictionary::Entry::new(
                    "t",
                    dt,
                    canopen_core::object_dictionary::AccessType::Rw,
                    canopen_core::object_dictionary::Storage::Value(
                        canopen_core::datatypes::Value::Uint(0),
                    ),
                ),
            );
            let frame = encode_download_expedited(addr, payload).unwrap();
            let out = SdoServer::new().handle(&mut od, &frame);
            out.is_empty() || decode_abort(&out[0]).is_err() // success response is not an abort
        }

        // Minimal-width convenience: succeeds when declared width == value
        // width; aborts otherwise (documented semantics).
        assert_eq!(encode_write_int(500), vec![0xF4, 0x01]);
        assert_eq!(encode_write_int(5), vec![5]);
        assert!(server_accepts(
            Address::new(0x1017, 0),
            Dt::Unsigned16,
            &encode_write_int(500)
        ));
        assert!(!server_accepts(
            Address::new(0x1017, 0),
            Dt::Unsigned16,
            &encode_write_int(5)
        ));

        // Explicit type: correct at every width.
        assert_eq!(
            encode_write_int_as(-1, Dt::Unsigned16).unwrap(),
            vec![0xFF, 0xFF]
        );
        assert_eq!(
            encode_write_int_as(500, Dt::Unsigned32).unwrap(),
            vec![0xF4, 0x01, 0x00, 0x00]
        );
        assert_eq!(
            encode_write_int_as(-2, Dt::Integer16).unwrap(),
            vec![0xFE, 0xFF]
        );
        assert_eq!(encode_write_int_as(7, Dt::Unsigned8).unwrap(), vec![7]);
        // docs/DESIGN-DECISIONS.md M5: 3/5/6/7-byte unsigned targets take two's-complement
        // at their own width too (previously only 1/2/4-byte sizes were
        // handled and negative values hit ValueOutOfRange).
        assert_eq!(
            encode_write_int_as(-1, Dt::Unsigned24).unwrap(),
            vec![0xFF, 0xFF, 0xFF]
        );
        assert_eq!(
            encode_write_int_as(-1, Dt::Unsigned40).unwrap(),
            vec![0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
        );
        assert_eq!(
            encode_write_int_as(-1, Dt::Unsigned48).unwrap(),
            vec![0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
        );
        assert_eq!(
            encode_write_int_as(-1, Dt::Unsigned56).unwrap(),
            vec![0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
        );
        // Positive values still encode little-endian at the wider widths.
        assert_eq!(
            encode_write_int_as(0x123456, Dt::Unsigned24).unwrap(),
            vec![0x56, 0x34, 0x12]
        );
        // Server-acceptance matrix: expedited downloads only carry ≤4 bytes
        // (`encode_download_expedited`), so the >4-byte widths are covered
        // by the encoder assertions above — those frames would go out as
        // segmented/block transfers instead.
        for (dt, payload) in [
            (Dt::Unsigned16, vec![0xFF, 0xFF]),
            (Dt::Unsigned32, vec![0xF4, 0x01, 0x00, 0x00]),
            (Dt::Integer16, vec![0xFE, 0xFF]),
            (Dt::Unsigned8, vec![7]),
            (Dt::Unsigned24, vec![0xFF, 0xFF, 0xFF]),
        ] {
            assert!(
                server_accepts(Address::new(0x2000, 0), dt, &payload),
                "{dt:?} must accept {payload:?}"
            );
        }
        // Non-fixed type rejected up front.
        assert!(encode_write_int_as(1, Dt::Domain).is_err());
    }

    /// docs/DESIGN-DECISIONS.md H2: floating-point targets must be rejected at the entry
    /// with a targeted message, not routed through the unsigned fallback
    /// into a confusing `TypeMismatch` from `encode_as`.
    #[test]
    fn write_int_as_rejects_float_targets_explicitly() {
        use canopen_core::datatypes::DataType as Dt;
        let err = encode_write_int_as(3, Dt::Real32).unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("floating-point"),
            "targeted message expected, got: {msg}"
        );
        assert!(encode_write_int_as(3, Dt::Real64).is_err());
        // Integer/boolean targets still work after the entry guard.
        assert!(encode_write_int_as(3, Dt::Unsigned16).is_ok());
        assert!(encode_write_int_as(1, Dt::Boolean).is_ok());
    }

    /// docs/DESIGN-DECISIONS.md H2: EDS content errors surface as `HostError::Eds`, distinct
    /// from network/protocol `HostError::Core` so callers can tell them
    /// apart.
    #[test]
    fn eds_content_error_is_distinct_variant() {
        // An EDS whose default overflows its declared type fails the
        // dictionary build; `Eds::build` is the host-layer entry point that
        // wraps that content error as `HostError::Eds` (API-08).
        let eds = crate::eds::parse("[2000]\nParameterName=T\nDataType=0x0005\nDefaultValue=300\n")
            .unwrap();
        let core_err = eds.to_object_dictionary(1).unwrap_err();
        let wrapped = HostError::Eds(core_err);
        assert!(wrapped.to_string().contains("EDS content error"));
        // …and the same wrap happens through the public builder path.
        let via_builder = eds.build(1).unwrap_err();
        assert!(matches!(via_builder, HostError::Eds(_)));
        assert!(via_builder.to_string().contains("EDS content error"));
        // And it stays distinguishable from a plain core error.
        assert!(!HostError::Core(canopen_core::types::Error::Other("x"))
            .to_string()
            .contains("EDS content error"));
    }

    /// docs/DESIGN-DECISIONS.md P1-4: a panicking subscriber must not prevent later handlers
    /// from running (per-handler isolation), and each panic is counted.
    #[test]
    fn handler_panic_is_isolated_and_counted() {
        let hits = Arc::new(AtomicUsize::new(0));

        let before = Arc::clone(&hits);
        let ok_before: Subscriber = Arc::new(move |_| {
            before.fetch_add(1, Ordering::Relaxed);
        });
        let boom: Subscriber = Arc::new(|_: &crate::transport::Received| {
            panic!("subscriber exploded");
        });
        let after = Arc::clone(&hits);
        let ok_after: Subscriber = Arc::new(move |_| {
            after.fetch_add(10, Ordering::Relaxed);
        });

        let rx = crate::transport::Received {
            cob_id: 0x185,
            data: [0, 0, 0, 0, 0, 0, 0, 0],
            len: 0,
        };
        let panics = dispatch_isolated(&[ok_before, boom, ok_after], &rx);

        // Both well-behaved handlers ran despite the middle one panicking.
        assert_eq!(hits.load(Ordering::Relaxed), 11);
        assert_eq!(panics, 1);

        // No panic at all → zero count.
        let quiet = dispatch_isolated(&[Arc::new(|_: &crate::transport::Received| {})], &rx);
        assert_eq!(quiet, 0);
    }
}
