//! # canopen-402
//!
//! CiA 402 drives-and-motion profile: pure sans-I/O state machines over
//! `canopen-core` types. Transport glue lives in the host layer.

pub mod drive;
pub mod homing;
pub mod mode;
pub mod state;

pub use drive::{Drive402, DriveError, DriveIo, DriveStateMachine};
pub use homing::HomingStatus;
pub use mode::{ModesSupported, OperationMode};
pub use state::{CwCommand, State402, Step, TargetState};
