//! Transport-agnostic SDO transaction core shared by the blocking and tokio
//! transports (`BUGS.md` m12): one implementation of client lifecycle,
//! outgoing queue management, protocol-error ABORT transmission and
//! terminal-event interpretation. The two transports only supply
//! send/receive primitives.

use std::collections::VecDeque;

use canopen_core::datatypes::{DataType, Value};
use canopen_core::sdo::{request_cob_id, response_cob_id, SdoClient, SdoEvent};
use canopen_core::types::Address;

use crate::error::HostError;
use crate::transport::sdo_error;

/// How a driven transaction ended.
#[derive(Debug)]
pub(crate) enum Progress {
    /// More responses expected; keep popping requests and waiting.
    Continue,
    /// Read finished with a decoded value.
    Complete(Value),
    /// Write finished (server confirmed).
    Done,
}

/// One SDO transaction against a node's default channel.
#[derive(Debug)]
pub(crate) struct Exchange {
    client: SdoClient,
    queue: VecDeque<[u8; 8]>,
    rx_cob: u16,
    tx_cob: u16,
}

impl Exchange {
    fn new(client: SdoClient, first: Vec<[u8; 8]>, node: u8) -> Self {
        Self {
            client,
            queue: first.into(),
            rx_cob: request_cob_id(node),
            tx_cob: response_cob_id(node),
        }
    }

    /// Starts an expedited/segmented read.
    #[must_use]
    pub fn read(node: u8, addr: Address, dt: DataType) -> Self {
        let mut client = SdoClient::new();
        let first = client.read(addr, dt);
        Self::new(client, first, node)
    }

    /// Starts an expedited/segmented write.
    ///
    /// # Errors
    /// [`HostError::Core`] when `data` cannot be framed (oversized etc.).
    pub fn write(node: u8, addr: Address, data: &[u8]) -> Result<Self, HostError> {
        let mut client = SdoClient::new();
        let first = client.write(addr, data).map_err(sdo_error)?;
        Ok(Self::new(client, first, node))
    }

    /// Starts a block upload (used for large domain objects like 0x1021).
    #[must_use]
    pub fn read_block(node: u8, addr: Address, dt: DataType) -> Self {
        let mut client = SdoClient::new();
        let first = client.read_block(addr, dt);
        Self::new(client, first, node)
    }

    /// Next frame for the request channel, if any.
    pub fn pop_request(&mut self) -> Option<[u8; 8]> {
        self.queue.pop_front()
    }

    /// COB-ID carrying requests to the server.
    #[must_use]
    pub const fn request_cob(&self) -> u16 {
        self.rx_cob
    }

    /// COB-ID carrying responses from the server.
    #[must_use]
    pub const fn response_cob(&self) -> u16 {
        self.tx_cob
    }

    /// Feeds one response frame from the server. Returns additional frames
    /// that must go out before the next wait (a protocol-error ABORT queued
    /// by the client so the server is released, and unconfirmed block-end
    /// trailing frames), plus the progress verdict.
    ///
    /// # Errors
    /// - [`HostError::SdoAborted`] when the server aborted (code preserved,
    ///   BUGS.md M1);
    /// - I/O and protocol errors propagate.
    pub fn on_response(&mut self, resp: &[u8; 8]) -> Result<(Vec<[u8; 8]>, Progress), HostError> {
        // Transmit a protocol-error ABORT queued by the client so the
        // server does not hang until its own timeout (BUGS.md M10).
        let mut extra: Vec<[u8; 8]> = self.client.take_pending_abort().into_iter().collect();
        let result = self.client.on_response(resp).map_err(sdo_error);
        match result? {
            SdoEvent::Send(frames) => {
                self.queue.extend(frames);
                Ok((extra, Progress::Continue))
            }
            SdoEvent::Complete(value, trailing) => {
                // Trailing frames (block end confirmation) are unconfirmed.
                extra.extend(trailing);
                Ok((extra, Progress::Complete(value)))
            }
            SdoEvent::Done => Ok((extra, Progress::Done)),
        }
    }
}

/// Reinterprets a decoded upload as raw bytes (domain reads).
#[must_use]
pub fn value_into_bytes(value: Value) -> Vec<u8> {
    match value {
        Value::Bytes(b) => b,
        Value::Str(s) => s.into_bytes(),
        other => {
            let n = other.encoded_len();
            let mut buf = vec![0u8; n];
            let written = other.encode_le(&mut buf).unwrap_or(0);
            buf.truncate(written);
            buf
        }
    }
}
