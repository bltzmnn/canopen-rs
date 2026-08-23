//! Blocking Linux SocketCAN transport plus high-level master helpers.
//!
//! Receive path classification: extended-ID frames are dropped (classic
//! CANopen traffic is 11-bit); remote frames surface as
//! [`Incoming::Remote`] so node guarding works; error frames are dropped.
//!
//! [`SocketCan::sdo_read`] / [`SocketCan::sdo_write`] run a whole SDO
//! transaction per call (expedited/segmented/block chosen automatically),
//! enforcing one deadline per call and mapping aborts onto
//! [`HostError::SdoAborted`] (docs/IMPLEMENTATION.md §5.6).

use socketcan::Socket as _;
use std::io::ErrorKind;
use std::time::{Duration, Instant};

use canopen_core::datatypes::{DataType, Value};
use canopen_core::nmt::{encode_command, NmtCommand, NMT_COMMAND_COB_ID};
use canopen_core::node::Node;
use canopen_core::types::{Address, NodeId};

use crate::error::HostError;
use crate::sdo_exchange::Progress;

/// Default deadline for one complete SDO transaction.
pub const SDO_DEADLINE: Duration = Duration::from_millis(1_000);

const POLL_SLICE: Duration = Duration::from_millis(20);

/// Maps a core SDO error to the host error it represents. Server aborts
/// surface as [`HostError::SdoAborted`] carrying the abort code; everything
/// else falls through the generic `From<core::Error>` conversion.
pub(crate) fn sdo_error(err: canopen_core::types::Error) -> HostError {
    match err {
        canopen_core::types::Error::SdoAborted(code) => HostError::SdoAborted(code),
        other => other.into(),
    }
}

/// One received frame, narrowed to what CANopen consumes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Incoming {
    /// A classic 11-bit data frame.
    Data(Received),
    /// A remote transmission request on an 11-bit ID.
    Remote(u16),
}

/// A received data frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Received {
    /// 11-bit COB-ID.
    pub cob_id: u16,
    /// Payload bytes (`data[..len]` valid).
    pub data: [u8; 8],
    /// Payload length.
    pub len: usize,
}

impl Received {
    /// Payload slice.
    #[must_use]
    pub fn payload(&self) -> &[u8] {
        &self.data[..self.len]
    }
}

fn classify(frame: &socketcan::CanFrame) -> Option<Incoming> {
    use socketcan::{CanFrame, EmbeddedFrame as _};
    match frame {
        CanFrame::Data(data_frame) => {
            let embedded_can::Id::Standard(sid) = data_frame.id() else {
                return None;
            };
            let payload = data_frame.data();
            let mut data = [0u8; 8];
            let len = payload.len().min(8);
            data[..len].copy_from_slice(&payload[..len]);
            Some(Incoming::Data(Received {
                cob_id: sid.as_raw(),
                data,
                len,
            }))
        }
        CanFrame::Remote(remote) => {
            let embedded_can::Id::Standard(sid) = remote.id() else {
                return None;
            };
            Some(Incoming::Remote(sid.as_raw()))
        }
        CanFrame::Error(_) => None,
    }
}

/// Monotonic milliseconds since process start (node timing source).
#[must_use]
pub fn now_ms() -> u64 {
    use std::sync::OnceLock;
    static START: OnceLock<Instant> = OnceLock::new();
    let start = START.get_or_init(Instant::now);
    start.elapsed().as_millis() as u64
}

/// Blocking SocketCAN socket.
pub struct SocketCan {
    sock: socketcan::CanSocket,
}

impl SocketCan {
    /// Opens `interface` (e.g. `"can0"`, `"vcan0"`).
    ///
    /// # Errors
    /// - [`HostError::InterfaceNotFound`] when the interface does not exist
    ///   or is not up;
    /// - [`HostError::Io`] for other socket errors.
    pub fn open(interface: &str) -> Result<Self, HostError> {
        let sock = socketcan::CanSocket::open(interface).map_err(|e| {
            if matches!(e.kind(), ErrorKind::NotFound | ErrorKind::AddrNotAvailable)
                || e.raw_os_error() == Some(19)
            {
                HostError::InterfaceNotFound(interface.to_string())
            } else {
                HostError::Io(e)
            }
        })?;
        Ok(Self { sock })
    }

    /// Sends one classic data frame.
    ///
    /// # Errors
    /// - [`HostError::Protocol`] for oversized IDs/payloads;
    /// - I/O errors propagate.
    pub fn send(&mut self, cob_id: u16, data: &[u8]) -> Result<(), HostError> {
        let id = socketcan::CanId::standard(cob_id)
            .ok_or(HostError::Protocol("COB-ID exceeds 11 bits"))?;
        use socketcan::EmbeddedFrame as _;
        let frame = socketcan::CanFrame::new(id, data)
            .ok_or(HostError::Protocol("payload exceeds 8 bytes"))?;
        use socketcan::Socket as _;
        self.sock.write_frame(&frame)?;
        Ok(())
    }

    /// Receives the next classified frame within `timeout`; `Ok(None)` on
    /// timeout.
    ///
    /// # Errors
    /// Propagates I/O errors.
    pub fn recv(&mut self, timeout: Duration) -> Result<Option<Incoming>, HostError> {
        match self.sock.read_frame_timeout(timeout) {
            Ok(frame) => Ok(classify(&frame)),
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => Ok(None),
            Err(e) => Err(HostError::Io(e)),
        }
    }

    /// Waits for a data frame from `tx_cob`, skipping unrelated traffic,
    /// until `deadline`.
    fn wait_response(&mut self, tx_cob: u16, deadline: Instant) -> Result<[u8; 8], HostError> {
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(HostError::Timeout);
            }
            match self.recv(remaining.min(POLL_SLICE))? {
                // SDO segments are always exactly 8 bytes (CANopenNode
                // CO_SDOclient/server check `DLC == 8U`); short frames are
                // malformed on the wire and must not be zero-padded into a
                // valid-looking response.
                Some(Incoming::Data(rx)) if rx.cob_id == tx_cob && rx.payload().len() == 8 => {
                    return Ok(rx.data)
                }
                Some(_) => continue,
                None => continue,
            }
        }
    }

    /// Drives an SDO client to completion against the bus.
    /// Drives one [`Exchange`] to completion against the bus. All decision
    /// logic (queueing, abort transmission, terminal interpretation) lives
    /// in the shared core; this only supplies send/receive (BUGS.md m12).
    fn drive_exchange(
        &mut self,
        mut ex: crate::sdo_exchange::Exchange,
        deadline: Instant,
    ) -> Result<crate::sdo_exchange::Progress, HostError> {
        use crate::sdo_exchange::Progress;
        while let Some(frame) = ex.pop_request() {
            self.send(ex.request_cob(), &frame)?;
            let response = self.wait_response(ex.response_cob(), deadline)?;
            let (extra, progress) = ex.on_response(&response)?;
            for f in extra {
                self.send(ex.request_cob(), &f)?;
            }
            if !matches!(progress, Progress::Continue) {
                return Ok(progress);
            }
        }
        Err(HostError::Protocol("transaction ended without completion"))
    }

    /// Reads an object from `node` over its default SDO channel, decoding
    /// against `dt`.
    ///
    /// # Errors
    /// - [`HostError::Timeout`] past [`SDO_DEADLINE`];
    /// - [`HostError::SdoAborted`] when the server aborts;
    /// - I/O and protocol errors propagate.
    pub fn sdo_read(&mut self, node: u8, addr: Address, dt: DataType) -> Result<Value, HostError> {
        let ex = crate::sdo_exchange::Exchange::read(node, addr, dt);
        match self.drive_exchange(ex, Instant::now() + SDO_DEADLINE)? {
            Progress::Complete(v) => Ok(v),
            _ => Err(HostError::Protocol("read ended without a value")),
        }
    }

    /// Reads a bulk object into bytes, trying **block transfer** first and
    /// falling back to plain expedited/segmented when the server aborts it
    /// (e.g. no block support). Used for pulling device-stored EDS files
    /// from 0x1021 where size is unknown up front but typically large.
    ///
    /// # Errors
    /// - [`HostError::SdoAborted`] when both paths are aborted;
    /// - timeouts / IO propagate.
    pub fn read_domain_bytes(&mut self, node: u8, addr: Address) -> Result<Vec<u8>, HostError> {
        use crate::sdo_exchange::{value_into_bytes, Exchange};
        let deadline = Instant::now() + SDO_DEADLINE;

        // 1st attempt: block upload.
        if let Progress::Complete(v) =
            self.drive_exchange(Exchange::read_block(node, addr, DataType::Domain), deadline)?
        {
            return Ok(value_into_bytes(v));
        }
        // Complete is the only terminal event for reads; drive_exchange errors
        // on anything else. Aborts surface as Err(SdoAborted) — fall through.

        // Fallback: expedited/segmented.
        match self.drive_exchange(Exchange::read(node, addr, DataType::Domain), deadline)? {
            Progress::Complete(v) => Ok(value_into_bytes(v)),
            _ => Err(HostError::Protocol("domain read ended without a value")),
        }
    }

    /// Writes raw little-endian bytes to an object on `node`.
    ///
    /// # Errors
    /// Same as [`SocketCan::sdo_read`].
    pub fn sdo_write(&mut self, node: u8, addr: Address, data: &[u8]) -> Result<(), HostError> {
        let ex = crate::sdo_exchange::Exchange::write(node, addr, data)?;
        match self.drive_exchange(ex, Instant::now() + SDO_DEADLINE)? {
            Progress::Done => Ok(()),
            _ => Err(HostError::Protocol("write ended without confirmation")),
        }
    }

    /// Broadcasts (`None` or `Some(0)`) or targets an NMT command.
    ///
    /// # Errors
    /// Propagates I/O errors; invalid target ids yield
    /// [`HostError::Protocol`].
    pub fn send_nmt(&mut self, command: NmtCommand, target: Option<u8>) -> Result<(), HostError> {
        let target = match target {
            None | Some(0) => None,
            Some(raw) => Some(
                NodeId::try_from(raw)
                    .map_err(|_| HostError::Protocol("NMT target must be 0..=127"))?,
            ),
        };
        self.send(NMT_COMMAND_COB_ID, &encode_command(command, target))
    }

    /// Serves a [`Node`] until `shutdown` becomes true: forwards received
    /// frames into the node and drains its producers (heartbeat, event
    /// TPDOs). RTR polls matching the heartbeat COB-ID drive node guarding.
    ///
    /// The node boots on entry. Returns when `shutdown` is set; the node
    /// keeps its state so callers may resume serving later.
    ///
    /// # Errors
    /// Propagates transport errors.
    pub fn serve_node(
        &mut self,
        node: &mut Node,
        shutdown: &std::sync::atomic::AtomicBool,
    ) -> Result<(), HostError> {
        use std::sync::atomic::Ordering;

        if let Some(boot) = node.boot() {
            self.send(boot.cob_id, boot.payload())?;
        }
        while !shutdown.load(Ordering::Relaxed) {
            for tx in node.poll(now_ms()) {
                self.send(tx.cob_id, tx.payload())?;
            }
            let Some(incoming) = self.recv(Duration::from_millis(5))? else {
                continue;
            };
            let responses = match incoming {
                Incoming::Data(rx) => node.on_frame(rx.cob_id, rx.payload()),
                Incoming::Remote(cob) => node.on_remote_request(cob).into_iter().collect(),
            };
            for tx in responses {
                self.send(tx.cob_id, tx.payload())?;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use canopen_core::sdo::SdoAbortCode;

    #[test]
    fn server_abort_surfaces_with_its_code() {
        // BUGS.md M1: `HostError::SdoAborted` was never constructed — aborts
        // were swallowed by the generic `From<core::Error>` conversion and
        // the gateway mapping became dead code.
        let err = sdo_error(canopen_core::types::Error::SdoAborted(
            SdoAbortCode::ObjectDoesNotExist,
        ));
        assert!(matches!(
            err,
            HostError::SdoAborted(SdoAbortCode::ObjectDoesNotExist)
        ));
        // Non-abort errors still take the generic path.
        let err = sdo_error(canopen_core::types::Error::ToggleBit);
        assert!(matches!(err, HostError::Core(_)));
    }
}
