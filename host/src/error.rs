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
    /// Protocol violation while driving a transaction.
    #[error("{0}")]
    Protocol(&'static str),
    /// Core protocol error (decode failures etc.).
    #[error(transparent)]
    Core(#[from] canopen_core::types::Error),
    /// The network RX thread died (panic or fatal IO).
    #[error("network RX thread failed: {0}")]
    Rx(String),
}
