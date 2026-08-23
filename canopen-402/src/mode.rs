//! CiA 402 operation modes (objects 0x6060/0x6061) and the 0x6502 support
//! bitmask.

/// Operation mode codes (object 0x6060).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i8)]
pub enum OperationMode {
    /// Position profile (1).
    ProfiledPosition = 1,
    /// Velocity profile (2).
    Velocity = 2,
    /// Torque profile (3)? — code 4 in CiA; kept explicit below.
    ProfiledTorque = 4,
    /// Homing (6).
    Homing = 6,
    /// Interpolated position (7).
    InterpolatedPosition = 7,
    /// Cyclic synchronous position (8).
    CyclicSyncPosition = 8,
    /// Cyclic synchronous velocity (9).
    CyclicSyncVelocity = 9,
    /// Cyclic synchronous torque (10).
    CyclicSyncTorque = 10,
}

impl OperationMode {
    /// Wire code.
    #[must_use]
    pub const fn code(self) -> i8 {
        self as i8
    }

    /// Decodes a mode byte; `0` (no mode) and vendor ranges yield `None`.
    #[must_use]
    pub fn from_code(code: i8) -> Option<Self> {
        Some(match code {
            1 => Self::ProfiledPosition,
            2 => Self::Velocity,
            4 => Self::ProfiledTorque,
            6 => Self::Homing,
            7 => Self::InterpolatedPosition,
            8 => Self::CyclicSyncPosition,
            9 => Self::CyclicSyncVelocity,
            10 => Self::CyclicSyncTorque,
            _ => return None,
        })
    }
}

/// Bitmask of object 0x6502 *Supported operation modes*.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ModesSupported(pub u16);

impl ModesSupported {
    /// Parses the raw register value.
    #[must_use]
    pub const fn from_raw(raw: u16) -> Self {
        Self(raw)
    }

    /// Whether `mode` is announced as supported.
    #[must_use]
    pub const fn supports(self, mode: OperationMode) -> bool {
        let bit = match mode {
            OperationMode::ProfiledPosition => 1 << 0,
            OperationMode::Velocity => 1 << 1,
            OperationMode::ProfiledTorque => 1 << 2,
            OperationMode::Homing => 1 << 3,
            OperationMode::InterpolatedPosition => 1 << 5,
            OperationMode::CyclicSyncPosition => 1 << 6,
            OperationMode::CyclicSyncVelocity => 1 << 7,
            OperationMode::CyclicSyncTorque => 1 << 8,
        };
        self.0 & bit != 0
    }
}
