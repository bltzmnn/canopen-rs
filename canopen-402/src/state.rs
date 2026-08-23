//! CiA 402 device states, statusword/controlword codecs and the transition
//! graph with compile-time route computation (docs/EXTENSIONS.md §C).
//!
//! Statusword decode is an **ordered** match — several states share bit
//! patterns under looser masks, so precedence is part of the contract
//! (fixing python-canopen's dict-insertion-order dependence, V2/V4 in
//! docs/REFERENCES.md).

/// The eight CiA 402 device states.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum State402 {
    /// Power-up; initialisation running.
    NotReadyToSwitchOn = 0,
    /// Mains off, drive ready to be switched on.
    SwitchOnDisabled = 1,
    /// Mains ready (`0x6041` shows "ready").
    ReadyToSwitchOn = 2,
    /// Main contactor closed, torque disabled.
    SwitchedOn = 3,
    /// Torque enabled — fully operational.
    OperationEnabled = 4,
    /// Quick stop executed, awaiting final ramp.
    QuickStopActive = 5,
    /// Fault triggered, executing the fault-reaction routine.
    FaultReactionActive = 6,
    /// Fault latched.
    Fault = 7,
}

const ALL: [State402; 8] = [
    State402::NotReadyToSwitchOn,
    State402::SwitchOnDisabled,
    State402::ReadyToSwitchOn,
    State402::SwitchedOn,
    State402::OperationEnabled,
    State402::QuickStopActive,
    State402::FaultReactionActive,
    State402::Fault,
];

impl State402 {
    /// All states in declaration order (index = discriminant).
    #[must_use]
    pub const fn all() -> &'static [Self; 8] {
        &ALL
    }

    /// Ordered statusword decode. Returns `None` for combinations outside
    /// the standard figure (callers surface this explicitly instead of a
    /// pseudo-state).
    #[must_use]
    pub fn from_statusword(sw: u16) -> Option<Self> {
        // Fault family first — checked against the tight 0x4F mask so the
        // voltage-enabled bit cannot alias them onto other states.
        if sw & 0x4F == 0x08 {
            return Some(Self::Fault);
        }
        if sw & 0x4F == 0x0F {
            return Some(Self::FaultReactionActive);
        }
        match sw & 0x6F {
            0x21 => Some(Self::ReadyToSwitchOn),
            0x23 => Some(Self::SwitchedOn),
            0x27 => Some(Self::OperationEnabled),
            0x07 => Some(Self::QuickStopActive),
            _ => match sw & 0x4F {
                0x40 | 0x60 => Some(Self::SwitchOnDisabled),
                0x00 => Some(Self::NotReadyToSwitchOn),
                _ => None,
            },
        }
    }

    const fn index(self) -> usize {
        self as usize
    }
}

/// Programmatatically enterable targets (fault states are reached only by
/// the device itself; leaving them is [`Step::FaultReset`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetState {
    /// Mains off.
    SwitchOnDisabled,
    /// Mains ready.
    ReadyToSwitchOn,
    /// Torque disabled, mains on.
    SwitchedOn,
    /// Fully operational.
    OperationEnabled,
}

impl From<TargetState> for State402 {
    fn from(t: TargetState) -> Self {
        match t {
            TargetState::SwitchOnDisabled => Self::SwitchOnDisabled,
            TargetState::ReadyToSwitchOn => Self::ReadyToSwitchOn,
            TargetState::SwitchedOn => Self::SwitchedOn,
            TargetState::OperationEnabled => Self::OperationEnabled,
        }
    }
}

/// Controlword command patterns (low nibble; upper bits reserved 0).
///
/// Note `SwitchOn` and `DisableOperation` share the `0x7` encoding — they
/// are distinguished solely by the current state, exactly as in the
/// standard's figure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CwCommand {
    /// `xxxx x110`
    Shutdown,
    /// `xxxx 0111` (also encodes "disable operation")
    SwitchOn,
    /// `xxxx xx00`
    DisableVoltage,
    /// `xxxx 0010`
    QuickStop,
    /// `xxxx 1111`
    EnableOperation,
}

impl CwCommand {
    /// Controlword bits contributed by this command.
    #[must_use]
    pub const fn bits(self) -> u16 {
        match self {
            Self::Shutdown => 0x06,
            Self::SwitchOn => 0x07,
            Self::DisableVoltage => 0x00,
            Self::QuickStop => 0x02,
            Self::EnableOperation => 0x0F,
        }
    }
}

/// Rising edge of bit 7 resets faults (CiA 402 §5.3.3).
pub const CW_FAULT_RESET_EDGE: u16 = 0x80;

/// A directed transition of the CiA 402 figure.
struct Edge {
    from: State402,
    to: State402,
    /// `None` = automatic (no controlword involvement).
    command: Option<CwCommand>,
}

/// The full directed graph (fault reset handled specially — see
/// [`Step::FaultReset`]).
const EDGES: &[Edge] = &[
    Edge {
        from: State402::NotReadyToSwitchOn,
        to: State402::SwitchOnDisabled,
        command: None,
    },
    Edge {
        from: State402::SwitchOnDisabled,
        to: State402::ReadyToSwitchOn,
        command: Some(CwCommand::Shutdown),
    },
    Edge {
        from: State402::ReadyToSwitchOn,
        to: State402::SwitchedOn,
        command: Some(CwCommand::SwitchOn),
    },
    Edge {
        from: State402::SwitchedOn,
        to: State402::OperationEnabled,
        command: Some(CwCommand::EnableOperation),
    },
    Edge {
        from: State402::OperationEnabled,
        to: State402::QuickStopActive,
        command: Some(CwCommand::QuickStop),
    },
    Edge {
        from: State402::OperationEnabled,
        to: State402::SwitchedOn,
        command: Some(CwCommand::SwitchOn),
    }, // disable operation shares encoding
    Edge {
        from: State402::OperationEnabled,
        to: State402::ReadyToSwitchOn,
        command: Some(CwCommand::Shutdown),
    }, // transition 8
    Edge {
        from: State402::OperationEnabled,
        to: State402::SwitchOnDisabled,
        command: Some(CwCommand::DisableVoltage),
    }, // transition 9
    Edge {
        from: State402::SwitchedOn,
        to: State402::ReadyToSwitchOn,
        command: Some(CwCommand::Shutdown),
    }, // transition 6
    Edge {
        from: State402::SwitchedOn,
        to: State402::SwitchOnDisabled,
        command: Some(CwCommand::DisableVoltage),
    }, // transition 10
    Edge {
        from: State402::ReadyToSwitchOn,
        to: State402::SwitchOnDisabled,
        command: Some(CwCommand::DisableVoltage),
    }, // transition 7
    Edge {
        from: State402::QuickStopActive,
        to: State402::OperationEnabled,
        command: Some(CwCommand::EnableOperation),
    },
    Edge {
        from: State402::QuickStopActive,
        to: State402::SwitchOnDisabled,
        command: Some(CwCommand::DisableVoltage),
    },
    Edge {
        from: State402::FaultReactionActive,
        to: State402::Fault,
        command: None,
    },
];

/// One resolved movement step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Step {
    /// Already there.
    Arrived,
    /// Transmit this controlword command (upper reserved bits stay clear).
    Write(CwCommand),
    /// Pulse the fault-reset bit (rising edge) — required to leave `Fault`.
    FaultReset,
    /// An automatic transition is pending; poll the statusword again shortly.
    Wait,
    /// No path exists (e.g. targeting a fault state, or stuck in `Fault`
    /// without a reset).
    Unreachable,
}

/// Breadth-first search over [`EDGES`]: returns, for every `(from, to)`
/// pair, the **first edge index** to take (or `u16::MAX` when unreachable).
/// Automatic edges cost nothing extra beyond their hop.
const fn build_first_step() -> [[u16; 8]; 8] {
    let mut next = [[u16::MAX; 8]; 8];
    let mut src = 0;
    while src < 8 {
        // BFS from `src`.
        let mut dist = [u16::MAX; 8];
        let mut first_edge = [u16::MAX; 8];
        let mut queue = [0usize; 8];
        let mut head = 0usize;
        let mut tail = 0usize;
        dist[src] = 0;
        queue[tail] = src;
        tail += 1;
        while head < tail {
            let node = queue[head];
            head += 1;
            let mut e = 0;
            while e < EDGES.len() {
                if EDGES[e].from.index() == node && dist[EDGES[e].to.index()] == u16::MAX {
                    let to = EDGES[e].to.index();
                    dist[to] = dist[node] + 1;
                    // First step: ourselves if we are the root, else parent's
                    // recorded first step.
                    first_edge[to] = if node == src {
                        e as u16
                    } else {
                        first_edge[node]
                    };
                    queue[tail] = to;
                    tail += 1;
                }
                e += 1;
            }
        }
        let mut dst = 0;
        while dst < 8 {
            next[src][dst] = first_edge[dst];
            dst += 1;
        }
        src += 1;
    }
    next
}

/// Precomputed first-step table: `[from][to]` → edge index into [`EDGES`].
pub(crate) const FIRST_STEP: [[u16; 8]; 8] = build_first_step();

/// Resolves the single next step moving `from` → `to`.
///
/// Leaving `Fault` requires the special reset pulse; targeting either fault
/// state is [`Step::Unreachable`] (devices enter them themselves).
#[must_use]
pub fn step_toward(from: State402, to: State402) -> Step {
    if from == to {
        return Step::Arrived;
    }
    if matches!(to, State402::Fault | State402::FaultReactionActive) {
        return Step::Unreachable;
    }
    // Leaving `Fault` happens via the reset pulse into SwitchOnDisabled
    // (standard transition); other targets are unreachable until then.
    if from == State402::Fault {
        return if to == State402::SwitchOnDisabled {
            Step::FaultReset
        } else {
            Step::Unreachable
        };
    }
    let edge = FIRST_STEP[from.index()][to.index()];
    if edge == u16::MAX || edge >= EDGES.len() as u16 {
        return Step::Unreachable;
    }
    match EDGES[edge as usize].command {
        Some(cmd) => Step::Write(cmd),
        None => Step::Wait,
    }
}

/// Convenience: validates that every declared state pair resolves to some
/// step (used by tests; public for completeness).
#[must_use]
pub fn resolves(from: State402, to: State402) -> bool {
    !matches!(step_toward(from, to), Step::Unreachable)
}
