//! Blocking Linux SocketCAN transport plus high-level master helpers.
//!
//! Receive path classification: extended-ID frames are dropped (classic
//! CANopen traffic is 11-bit); remote frames surface as
//! [`Incoming::Remote`] so node guarding works; error frames are dropped.
//!
//! [`SocketCan::sdo_read`] / [`SocketCan::sdo_write`] run a whole SDO
//! transaction per call (expedited/segmented/block chosen automatically),
//! with one timeout retry per call and a *fresh* deadline for each attempt,
//! mapping aborts onto [`HostError::SdoAborted`]
//! (docs/IMPLEMENTATION.md §5.6; the deadline reset on retry is intentional,
//! `BUGS.md` #33).

use socketcan::Socket as _;
use std::io::ErrorKind;
use std::time::{Duration, Instant};

use canopen_core::datatypes::{DataType, Value};
use canopen_core::nmt::NmtCommand;
use canopen_core::node::Node;
use canopen_core::types::Address;

use crate::error::HostError;

/// Default deadline for one complete SDO transaction.
pub const SDO_DEADLINE: Duration = Duration::from_millis(1_000);

pub(crate) const POLL_SLICE: Duration = Duration::from_millis(20);

/// Maps a core SDO error to the host error it represents. Server aborts
/// surface as [`HostError::SdoAborted`] carrying the abort code; everything
/// else falls through the generic `From<core::Error>` conversion.
pub(crate) fn sdo_error(err: canopen_core::types::Error) -> HostError {
    match err {
        canopen_core::types::Error::SdoAborted(code) => HostError::SdoAborted(code),
        other => other.into(),
    }
}

/// Shared SDO-response predicate (BUGS.md M5): SDO segments are always
/// exactly 8 bytes (CANopenNode `CO_SDOclient.c` / `CO_SDOserver.c` check
/// `DLC == 8U`), so a short frame is malformed on the wire and must not be
/// zero-padded into a valid-looking response. Both transports call this —
/// a duplicated inline predicate would drift and start accepting zero-filled
/// frames.
#[must_use]
pub(crate) fn is_valid_sdo_response(rx: &Received, tx_cob: u16) -> bool {
    rx.cob_id == tx_cob && rx.payload().len() == 8
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

pub(crate) fn classify(frame: &socketcan::CanFrame) -> Option<Incoming> {
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
    /// - [`HostError::BadCobId`] / [`HostError::BadPayloadLen`] for
    ///   oversized IDs/payloads;
    /// - I/O errors propagate.
    pub fn send(&mut self, cob_id: u16, data: &[u8]) -> Result<(), HostError> {
        let id = socketcan::CanId::standard(cob_id).ok_or(HostError::BadCobId)?;
        use socketcan::EmbeddedFrame as _;
        let frame = socketcan::CanFrame::new(id, data).ok_or(HostError::BadPayloadLen)?;
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

    /// Receives the next response frame from `tx_cob` (8-byte SDO segments
    /// only), skipping unrelated traffic, until `deadline`.
    fn recv_response(&mut self, tx_cob: u16, deadline: Instant) -> Result<[u8; 8], HostError> {
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(HostError::Timeout);
            }
            match self.recv(remaining.min(POLL_SLICE))? {
                Some(Incoming::Data(rx)) if is_valid_sdo_response(&rx, tx_cob) => {
                    return Ok(rx.data)
                }
                Some(_) => continue,
                None => continue,
            }
        }
    }

    /// Receives one frame (bounded 5 ms window) for the serving loop.
    fn recv_frame(&mut self) -> Result<Option<Incoming>, HostError> {
        self.recv(Duration::from_millis(5))
    }

    // 算法体由 `sdo_exchange` 的 `drive_exchange_body!` 提供（BUGS.md m12）。
    fn drive_exchange(
        &mut self,
        mut ex: crate::sdo_exchange::Exchange,
        deadline: Instant,
    ) -> Result<crate::sdo_exchange::Progress, HostError> {
        crate::sdo_exchange::drive_exchange_body!(self, ex, deadline, block)
    }

    /// Reads an object from `node` over its default SDO channel, decoding
    /// against `dt`.
    ///
    /// # Errors
    /// - [`HostError::Timeout`] past [`SDO_DEADLINE`];
    /// - [`HostError::SdoAborted`] when the server aborts;
    /// - I/O and protocol errors propagate.
    pub fn sdo_read(&mut self, node: u8, addr: Address, dt: DataType) -> Result<Value, HostError> {
        crate::sdo_exchange::sdo_read_body!(self, node, addr, dt, block)
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
        crate::sdo_exchange::read_domain_bytes_body!(self, node, addr, block)
    }

    /// Writes raw little-endian bytes to an object on `node`.
    ///
    /// # Errors
    /// Same as [`SocketCan::sdo_read`].
    pub fn sdo_write(&mut self, node: u8, addr: Address, data: &[u8]) -> Result<(), HostError> {
        crate::sdo_exchange::sdo_write_body!(self, node, addr, data, block)
    }

    /// Broadcasts (`None` or `Some(0)`) or targets an NMT command.
    ///
    /// # Errors
    /// Propagates I/O errors; invalid target ids yield
    /// [`HostError::BadNmtTarget`].
    pub fn send_nmt(&mut self, command: NmtCommand, target: Option<u8>) -> Result<(), HostError> {
        crate::sdo_exchange::send_nmt_body!(self, command, target, block)
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
        crate::sdo_exchange::serve_node_body!(self, node, shutdown, block)
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
