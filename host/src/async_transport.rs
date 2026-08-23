//! Tokio-based SocketCAN transport — the async mirror of
//! [`crate::transport::SocketCan`]. Requires the `tokio` feature.
//!
//! The same SDO transaction driver semantics apply (one deadline per call,
//! aborts mapped onto [`HostError::SdoAborted`]); awaiting replaces blocking
//! reads and `tokio::time::timeout` enforces deadlines.

use std::io::ErrorKind;
use std::time::{Duration, Instant};

use canopen_core::datatypes::{DataType, Value};
use canopen_core::nmt::{encode_command, NmtCommand, NMT_COMMAND_COB_ID};
use canopen_core::node::Node;
use canopen_core::types::{Address, NodeId};

use crate::error::HostError;
use crate::transport::{now_ms, Received, SDO_DEADLINE};

const POLL_SLICE: Duration = Duration::from_millis(20);

/// One received frame, narrowed to what CANopen consumes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Incoming {
    /// A classic 11-bit data frame.
    Data(Received),
    /// A remote transmission request on an 11-bit ID.
    Remote(u16),
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
        let id = socketcan::CanId::standard(cob_id)
            .ok_or(HostError::Protocol("COB-ID exceeds 11 bits"))?;
        use socketcan::EmbeddedFrame as _;
        let frame = socketcan::CanFrame::new(id, data)
            .ok_or(HostError::Protocol("payload exceeds 8 bytes"))?;
        self.sock.write_frame(frame).await?;
        Ok(())
    }

    /// Receives the next classified frame; never returns on its own — wrap
    /// in [`tokio::time::timeout`] for deadlines. Skips extended/error
    /// frames internally by re-reading.
    ///
    /// # Errors
    /// Propagates I/O errors.
    pub async fn recv(&mut self) -> Result<Option<Incoming>, HostError> {
        let frame = self.sock.read_frame().await?;
        Ok(classify(&frame))
    }

    async fn wait_response(
        &mut self,
        tx_cob: u16,
        deadline: Instant,
    ) -> Result<[u8; 8], HostError> {
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(HostError::Timeout);
            }
            match tokio::time::timeout(remaining.min(POLL_SLICE), self.recv()).await {
                Err(_elapsed) => continue,
                Ok(Err(e)) => return Err(e),
                // SDO frames are always exactly 8 bytes (DLC == 8U in
                // CANopenNode); reject zero-padded short frames.
                Ok(Ok(Some(Incoming::Data(rx))))
                    if rx.cob_id == tx_cob && rx.payload().len() == 8 =>
                {
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
        use crate::sdo_exchange::Progress;
        while let Some(frame) = ex.pop_request() {
            self.send(ex.request_cob(), &frame).await?;
            let response = self.wait_response(ex.response_cob(), deadline).await?;
            let (extra, progress) = ex.on_response(&response)?;
            for f in extra {
                self.send(ex.request_cob(), &f).await?;
            }
            if !matches!(progress, Progress::Continue) {
                return Ok(progress);
            }
        }
        Err(HostError::Protocol("transaction ended without completion"))
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
        use crate::sdo_exchange::Progress;
        let ex = crate::sdo_exchange::Exchange::read(node, addr, dt);
        match self
            .drive_exchange(ex, Instant::now() + SDO_DEADLINE)
            .await?
        {
            Progress::Complete(v) => Ok(v),
            _ => Err(HostError::Protocol("read ended without a value")),
        }
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
        use crate::sdo_exchange::Progress;
        let ex = crate::sdo_exchange::Exchange::write(node, addr, data)?;
        match self
            .drive_exchange(ex, Instant::now() + SDO_DEADLINE)
            .await?
        {
            Progress::Done => Ok(()),
            _ => Err(HostError::Protocol("write ended without confirmation")),
        }
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
        let target = match target {
            None | Some(0) => None,
            Some(raw) => Some(
                NodeId::try_from(raw)
                    .map_err(|_| HostError::Protocol("NMT target must be 0..=127"))?,
            ),
        };
        self.send(NMT_COMMAND_COB_ID, &encode_command(command, target))
            .await
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
        use std::sync::atomic::Ordering;

        if let Some(boot) = node.boot() {
            self.send(boot.cob_id, boot.payload()).await?;
        }
        while !shutdown.load(Ordering::Relaxed) {
            for tx in node.poll(now_ms()) {
                self.send(tx.cob_id, tx.payload()).await?;
            }
            // Bounded receive window keeps poll() running periodically.
            let incoming = match tokio::time::timeout(Duration::from_millis(5), self.recv()).await {
                Ok(Ok(Some(frame))) => frame,
                // Transport failures must surface, not be retried silently.
                Ok(Err(e)) => return Err(e),
                _ => continue,
            };
            let responses = match incoming {
                Incoming::Data(rx) => node.on_frame(rx.cob_id, rx.payload()),
                Incoming::Remote(cob) => node.on_remote_request(cob).into_iter().collect(),
            };
            for tx in responses {
                self.send(tx.cob_id, tx.payload()).await?;
            }
        }
        Ok(())
    }
}
