//! Host-side error types shared by all transports.

/// Errors surfaced by the host layer.
#[derive(Debug, thiserror::Error)]
pub enum HostError {
    /// Underlying socket I/O failure.
    #[error(transparent)]
    Io(#[from] std::io::Error),
    /// The requested operation did not complete within its deadline.
    #[error("operation timed out")]
    Timeout,
    /// The interface does not exist or is not up.
    #[error("CAN interface '{0}' not found (is it up? try `sudo ip link set up {0}`)")]
    InterfaceNotFound(String),
    /// The remote SDO server aborted the transaction.
    #[error("SDO aborted: {0}")]
    SdoAborted(canopen_core::sdo::SdoAbortCode),
    /// COB-ID does not fit the 11-bit classic CAN identifier space.
    #[error("COB-ID exceeds 11 bits")]
    BadCobId,
    /// Payload exceeds the 8-byte classic CAN frame limit.
    #[error("payload exceeds 8 bytes")]
    BadPayloadLen,
    /// NMT target node id is outside the valid 0..=127 range.
    #[error("NMT target must be 0..=127")]
    BadNmtTarget,
    /// SDO node id is outside the valid 1..=127 range (0 is never a valid
    /// SDO peer).
    #[error("SDO node id must be 1..=127")]
    BadSdoNode,
    /// The transaction ended without the expected terminal event (e.g. a
    /// read that finished without a value, or a write without confirmation).
    #[error("transaction ended without the expected terminal event")]
    ProtocolIncomplete,
    /// Core protocol error (decode failures etc.).
    #[error(transparent)]
    Core(#[from] canopen_core::types::Error),
    /// EDS content error surfaced separately from core protocol errors so
    /// callers can tell a device-side EDS problem (out-of-range default,
    /// bad length, …) from a network/protocol failure (`docs/DESIGN-DECISIONS.md` H2).
    ///
    /// Produced by [`crate::eds::Eds::build`], which wraps the underlying
    /// [`canopen_core::types::Error`] from the dictionary build — the
    /// network-facing helpers return the raw [`Eds`](crate::eds::Eds) and
    /// leave the choice of channel to the caller (`docs/DESIGN-DECISIONS.md` API-08).
    #[error("EDS content error: {0}")]
    Eds(canopen_core::types::Error),
    /// The network RX thread died (panic or fatal IO).
    #[error("network RX thread failed: {0}")]
    Rx(String),
}
