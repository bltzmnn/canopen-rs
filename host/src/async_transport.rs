//! Tokio-based SocketCAN transport — the async mirror of
//! [`crate::transport::SocketCan`]. Requires the `tokio` feature.
//!
//! The same SDO transaction driver semantics apply (one deadline per call,
//! aborts mapped onto [`HostError::SdoAborted`]); awaiting replaces blocking
//! reads and `tokio::time::timeout` enforces deadlines.

use std::io::ErrorKind;
use std::time::{Duration, Instant};

use canopen_core::datatypes::{DataType, Value};
use canopen_core::nmt::NmtCommand;
use canopen_core::node::Node;
use canopen_core::types::Address;

use crate::error::HostError;
use crate::transport::{classify, is_valid_sdo_response, Incoming, POLL_SLICE};

/// Async SocketCAN socket.
pub struct AsyncSocketCan {
    sock: socketcan::tokio::CanSocket,
}

impl AsyncSocketCan {
    /// Opens `interface`.
    ///
    /// # Errors
    /// Same mapping as [`crate::transport::SocketCan::open`].
    pub fn open(interface: &str) -> Result<Self, HostError> {
        let sock = socketcan::tokio::CanSocket::open(interface).map_err(|e| {
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
    /// Propagates I/O errors.
    pub async fn send(&mut self, cob_id: u16, data: &[u8]) -> Result<(), HostError> {
        let id = socketcan::CanId::standard(cob_id).ok_or(HostError::BadCobId)?;
        use socketcan::EmbeddedFrame as _;
        let frame = socketcan::CanFrame::new(id, data).ok_or(HostError::BadPayloadLen)?;
        self.sock.write_frame(frame).await?;
        Ok(())
    }

    /// Receives the next classified frame; never returns on its own — wrap
    /// in [`tokio::time::timeout`] for deadlines. Extended and error frames
    /// classify to `Ok(None)`; the caller re-reads in a loop until it gets
    /// data or a remote frame.
    ///
    /// # Errors
    /// Propagates I/O errors.
    pub async fn recv(&mut self) -> Result<Option<Incoming>, HostError> {
        let frame = self.sock.read_frame().await?;
        Ok(classify(&frame))
    }

    async fn recv_response(
        &mut self,
        tx_cob: u16,
        deadline: Instant,
    ) -> Result<[u8; 8], HostError> {
        // BUGS.md L4: the deadline arrives on the std clock (shared driver
        // macro), but the wait below uses `tokio::time::timeout` — mixing
        // the two clocks directly would decouple them under
        // `tokio::time::pause()` (std time keeps advancing while tokio's is
        // frozen), hanging the loop. Re-anchor the deadline onto the tokio
        // clock at entry so remaining-time arithmetic and the timeout share
        // one clock.
        let deadline =
            tokio::time::Instant::now() + deadline.saturating_duration_since(Instant::now());
        loop {
            let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
            if remaining.is_zero() {
                return Err(HostError::Timeout);
            }
            match tokio::time::timeout(remaining.min(POLL_SLICE), self.recv()).await {
                Err(_elapsed) => continue,
                Ok(Err(e)) => return Err(e),
                // SDO frames are always exactly 8 bytes (DLC == 8U in
                // CANopenNode); reject zero-padded short frames (shared
                // predicate with the blocking transport, BUGS.md M5).
                Ok(Ok(Some(Incoming::Data(rx)))) if is_valid_sdo_response(&rx, tx_cob) => {
                    return Ok(rx.data);
                }
                Ok(Ok(_)) => continue,
            }
        }
    }

    /// Drives one shared [`Exchange`] to completion (async twin of the
    /// blocking driver; decision logic lives in the common core, BUGS.md
    /// m12).
    async fn drive_exchange(
        &mut self,
        mut ex: crate::sdo_exchange::Exchange,
        deadline: Instant,
    ) -> Result<crate::sdo_exchange::Progress, HostError> {
        crate::sdo_exchange::drive_exchange_body!(self, ex, deadline, await)
    }

    /// Receives one frame (bounded 5 ms window) for the serving loop.
    async fn recv_frame(&mut self) -> Result<Option<Incoming>, HostError> {
        match tokio::time::timeout(Duration::from_millis(5), self.recv()).await {
            Ok(r) => r,
            Err(_elapsed) => Ok(None),
        }
    }

    /// Reads an object from `node`, decoding against `dt`.
    ///
    /// # Errors
    /// Same as the blocking transport.
    pub async fn sdo_read(
        &mut self,
        node: u8,
        addr: Address,
        dt: DataType,
    ) -> Result<Value, HostError> {
        crate::sdo_exchange::sdo_read_body!(self, node, addr, dt, await)
    }

    /// Reads a bulk object into bytes, trying **block transfer** first and
    /// falling back to plain expedited/segmented when the server aborts it.
    /// Used for pulling device-stored EDS files from 0x1021.
    ///
    /// # Errors
    /// - [`HostError::SdoAborted`] when both paths are aborted;
    /// - timeouts / IO propagate.
    pub async fn read_domain_bytes(
        &mut self,
        node: u8,
        addr: Address,
    ) -> Result<Vec<u8>, HostError> {
        crate::sdo_exchange::read_domain_bytes_body!(self, node, addr, await)
    }

    /// Writes raw little-endian bytes to an object on `node`.
    ///
    /// # Errors
    /// Same as the blocking transport.
    pub async fn sdo_write(
        &mut self,
        node: u8,
        addr: Address,
        data: &[u8],
    ) -> Result<(), HostError> {
        crate::sdo_exchange::sdo_write_body!(self, node, addr, data, await)
    }

    /// Broadcasts or targets an NMT command.
    ///
    /// # Errors
    /// Same as the blocking transport.
    pub async fn send_nmt(
        &mut self,
        command: NmtCommand,
        target: Option<u8>,
    ) -> Result<(), HostError> {
        crate::sdo_exchange::send_nmt_body!(self, command, target, await)
    }

    /// Serves a [`Node`] until `shutdown` flips (async twin of the blocking
    /// serving loop).
    ///
    /// # Errors
    /// Propagates transport errors.
    pub async fn serve_node(
        &mut self,
        node: &mut Node,
        shutdown: &std::sync::atomic::AtomicBool,
    ) -> Result<(), HostError> {
        crate::sdo_exchange::serve_node_body!(self, node, shutdown, await)
    }
}
