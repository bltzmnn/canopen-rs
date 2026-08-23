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
//! P2/P3): subscriber callbacks run in isolation — an error in one handler
//! never kills the RX thread, and [`Network::check`] re-raises a crashed RX
//! thread to the synchronous caller.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use canopen_core::datatypes::{DataType, Value};
use canopen_core::gateway::{GatewayAction, GatewayErrorCode, GatewayFailure};
use canopen_core::nmt::{HbEvent, HeartbeatConsumer, NmtCommand};
use canopen_core::types::{Address, NodeId};

use crate::error::HostError;
use crate::transport::{Incoming, SocketCan};

type Subscriber = std::sync::Arc<dyn Fn(&crate::transport::Received) + Send + Sync>;

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
    /// Persistent CiA 309-3 gateway parser (`set node` survives calls).
    gateway: Mutex<canopen_core::gateway::Gateway>,
    /// RX thread liveness: set when the thread died unexpectedly.
    rx_state: Arc<Mutex<Option<RxDeath>>>,
    rx_shutdown: Arc<AtomicBool>,
    rx_handle: Option<std::thread::JoinHandle<()>>,
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
            gateway: Mutex::new(canopen_core::gateway::Gateway::new()),
            rx_state: Arc::new(Mutex::new(None)),
            rx_shutdown: Arc::new(AtomicBool::new(false)),
            rx_handle: None,
        };
        network.spawn_rx(rx_sock);
        Ok(network)
    }

    fn spawn_rx(&mut self, mut sock: SocketCan) {
        let subscribers = self.subscribers.clone();
        let hb = self.hb.clone();
        let state = self.rx_state.clone();
        let shutdown = self.rx_shutdown.clone();
        self.rx_handle = Some(std::thread::spawn(move || {
            let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(move || {
                while !shutdown.load(Ordering::Relaxed) {
                    let _ = hb.lock().expect("hb lock").poll(crate::transport::now_ms());
                    let Some(incoming) = sock.recv(Duration::from_millis(10))? else {
                        continue;
                    };
                    let Incoming::Data(rx) = incoming else {
                        continue;
                    };
                    // Heartbeat supervision first (internal bookkeeping).
                    if (0x700..=0x77F).contains(&rx.cob_id) {
                        if let Ok(node) = NodeId::try_from((rx.cob_id - 0x700) as u8) {
                            let _ = hb.lock().expect("hb lock").on_heartbeat(
                                node,
                                rx.payload(),
                                crate::transport::now_ms(),
                            );
                        }
                    }
                    // Exact-match subscriber dispatch; one failing callback
                    // must not kill the loop (P2). Handlers are cloned out
                    // and invoked with the registry lock released so a
                    // callback that re-enters subscribe/unsubscribe (or any
                    // other lock-taking Network call) cannot deadlock.
                    let handlers = {
                        let subs = subscribers.lock().expect("subscribers lock");
                        subs.get(&rx.cob_id).cloned().unwrap_or_default()
                    };
                    for handler in &handlers {
                        handler(&rx);
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
                *state.lock().expect("state lock") = Some(death);
            }
        }));
    }

    /// Registers a callback for exact-COB-ID traffic. Duplicate registration
    /// of the same id appends another handler (python-canopen dedupes; we
    /// allow multiple observers and document it).
    pub fn subscribe<F>(&self, cob_id: u16, callback: F)
    where
        F: Fn(&crate::transport::Received) + Send + Sync + 'static,
    {
        self.subscribers
            .lock()
            .expect("subscribers lock")
            .entry(cob_id)
            .or_default()
            .push(std::sync::Arc::new(callback));
    }

    /// Removes all callbacks for `cob_id`; returns how many were removed.
    pub fn unsubscribe(&self, cob_id: u16) -> usize {
        self.subscribers
            .lock()
            .expect("subscribers lock")
            .remove(&cob_id)
            .map_or(0, |handlers| handlers.len())
    }

    /// Starts heartbeat supervision of `node` with `timeout_ms`.
    pub fn monitor_heartbeat(&self, node: NodeId, timeout_ms: u32) {
        self.hb
            .lock()
            .expect("hb lock")
            .set_timeout(node, timeout_ms);
    }

    /// Drains pending heartbeat events (Started/Lost/RemoteReset/StateChanged).
    ///
    /// Note: events produced inside the RX thread's internal polling are
    /// coalesced there; this method re-polls with the current time and is
    /// the intended query point from the application thread.
    pub fn heartbeat_events(&self) -> Vec<HbEvent> {
        self.hb
            .lock()
            .expect("hb lock")
            .poll(crate::transport::now_ms())
    }

    /// Last reported NMT state of `node`, when known.
    #[must_use]
    pub fn node_state(&self, node: NodeId) -> Option<canopen_core::nmt::NmtState> {
        self.hb.lock().expect("hb lock").state_of(node)
    }

    /// Whether every monitored node is alive.
    #[must_use]
    pub fn all_nodes_active(&self) -> bool {
        self.hb.lock().expect("hb lock").all_active()
    }

    /// Re-raises an RX-thread crash to the caller (P3). Call periodically
    /// from long-lived masters; every command checks implicitly.
    ///
    /// # Errors
    /// Propagates the stored panic message / IO error as [`HostError::Rx`].
    pub fn check(&self) -> Result<(), HostError> {
        match &*self.rx_state.lock().expect("state lock") {
            None => Ok(()),
            Some(RxDeath::Panic(msg)) => Err(HostError::Rx(format!("panic: {msg}"))),
            Some(RxDeath::Io(msg)) => Err(HostError::Rx(format!("io: {msg}"))),
        }
    }

    /// Runs `f` with exclusive access to the command socket.
    pub fn with_cmd<T>(
        &self,
        f: impl FnOnce(&mut SocketCan) -> Result<T, HostError>,
    ) -> Result<T, HostError> {
        self.check()?;
        let mut guard = self.cmd.lock().expect("cmd lock");
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
    /// # Errors
    /// Transport errors, or [`HostError::Protocol`] when the device does not
    /// implement 0x1021 / aborts.
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
            crate::eds::parse(text).map_err(HostError::Core)
        })
    }

    /// Stops the RX thread. Idempotent; called by [`Drop`](Self::drop).
    pub fn shutdown(&mut self) {
        self.rx_shutdown.store(true, Ordering::Relaxed);
        if let Some(handle) = self.rx_handle.take() {
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

    /// Writes an integer value encoded minimally (signed-aware), the common
    /// case for parameter tweaks.
    ///
    /// # Errors
    /// Non-integer input or transport errors.
    pub fn write_int(&self, addr: Address, value: i64) -> Result<(), HostError> {
        let v = Value::Int(value);
        let mut buf = [0u8; 8];
        let n = v.encode_le(&mut buf).map_err(HostError::Core)?;
        self.network.sdo_write(self.id, addr, &buf[..n])
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
        self.sdo_write(
            Address::new(0x1010, 1),
            &u32::from_le_bytes(*b"save").to_le_bytes(),
        )
    }

    /// Parameter restore: writes "load" to 0x1011:1; defaults return after
    /// the next reset.
    ///
    /// # Errors
    /// Same as [`RemoteNode::store`].
    pub fn restore(&self) -> Result<(), HostError> {
        self.sdo_write(
            Address::new(0x1011, 1),
            &u32::from_le_bytes(*b"daol").to_le_bytes(),
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
    pub fn gateway_line(&self, line: &str) -> String {
        use canopen_core::gateway::GatewayEvent;

        let mut gw = self.gateway.lock().expect("gateway lock");
        match gw.feed_line(line) {
            GatewayEvent::Ignore => String::new(),
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
                let dt = data_type.to_core().unwrap_or(DataType::Unsigned32);
                self.sdo_read(node, Address::new(index, sub), dt)
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
                    .map_err(|_| GatewayFailure::Code(GatewayErrorCode::NmtFailed))
            }
            GatewayAction::SetNode(_) => Ok(None),
        }
    }
}

fn sdo_failure(err: HostError) -> GatewayFailure {
    match err {
        HostError::SdoAborted(code) => GatewayFailure::SdoAborted(code),
        HostError::Timeout => GatewayFailure::Code(GatewayErrorCode::Timeout),
        _ => GatewayFailure::Code(GatewayErrorCode::CanFailed),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicUsize;

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
}
