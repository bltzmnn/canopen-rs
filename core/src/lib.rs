//! # canopen-core
//!
//! Sans-I/O CANopen (CiA 301) protocol core.
//!
//! The crate performs **no I/O**: protocol state machines consume decoded
//! frames and produce frames to transmit, but never touch a bus. See
//! `docs/IMPLEMENTATION.md` for the architecture.

pub mod datatypes;
pub mod emcy;
pub mod gateway;
pub mod leds;
pub mod lss;
pub mod nmt;
pub mod node;
pub mod object_dictionary;
pub mod pdo;
pub mod sdo;
pub mod standard;
pub mod storage;
pub mod sync_time;
pub mod types;
