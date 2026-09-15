//! # canopen-host
//!
//! Linux host layer around `canopen-core`: SocketCAN transports (blocking
//! and, with the `tokio` feature, async), EDS parsing/code generation and
//! master-side helpers.

#[cfg(target_os = "linux")]
pub mod client;
pub mod codegen;
pub mod eds;
pub mod error;
// BUGS.md 十M15: `sdo_exchange` drives the SocketCAN transports and imports
// `crate::transport`, which only exists on Linux — gating it keeps
// `eds`/`codegen`/`storage` cross-platform buildable as documented.
#[cfg(target_os = "linux")]
pub(crate) mod sdo_exchange;
pub mod storage;

#[cfg(target_os = "linux")]
pub mod transport;

#[cfg(all(target_os = "linux", feature = "tokio"))]
pub mod async_transport;

#[cfg(any(feature = "tokio", target_os = "linux"))]
pub use error::HostError;
