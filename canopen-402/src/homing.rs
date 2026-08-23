//! Homing status decoding from statusword bits 12/13 (CiA 402 §5.3.9).

/// Status of the homing procedure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HomingStatus {
    /// Running.
    InProgress,
    /// Interrupted by user request (halt).
    Interrupted,
    /// Attained but target not reached.
    Attained,
    /// Finished successfully.
    TargetReached,
    /// Error: velocity non-zero during homing window.
    ErrorVelocityNonZero,
    /// Error: velocity zero during homing window.
    ErrorVelocityZero,
}

impl HomingStatus {
    /// Decodes statusword bits 12/13 (`sw & 0x3400`). Other bits ignored.
    #[must_use]
    pub fn from_statusword(sw: u16) -> Option<Self> {
        Some(match sw & 0x3400 {
            0x0000 => Self::InProgress,
            0x0400 => Self::Interrupted,
            0x1000 => Self::Attained,
            0x1400 => Self::TargetReached,
            0x2000 => Self::ErrorVelocityNonZero,
            0x2400 => Self::ErrorVelocityZero,
            _ => return None,
        })
    }
}

/// Controlword bits specific to homing.
pub mod command {
    /// Start/restart homing (bit 4).
    pub const START_BIT: u16 = 1 << 4;
    /// Halt homing (bit 8).
    pub const HALT_BIT: u16 = 1 << 8;
}
