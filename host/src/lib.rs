//! # canopen-host
//!
//! Linux host layer around `canopen-core`: SocketCAN transports (blocking
//! and, with the `tokio` feature, async), EDS parsing/code generation and
//! master-side helpers.

pub mod client;
pub mod codegen;
pub mod eds;
pub mod error;
pub mod profile_402;
pub(crate) mod sdo_exchange;
pub mod storage;

#[cfg(target_os = "linux")]
pub mod transport;

#[cfg(all(target_os = "linux", feature = "tokio"))]
pub mod async_transport;

#[cfg(any(feature = "tokio", target_os = "linux"))]
pub use error::HostError;
