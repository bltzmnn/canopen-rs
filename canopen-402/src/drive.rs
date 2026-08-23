//! Sans-I/O drive state machine plus the [`DriveIo`] abstraction and a
//! synchronous [`Drive402`] client with deadline-driven stepping.
//!
//! Waiting is **deadline-driven, not busy-wait**: the caller supplies
//! monotonic milliseconds; between statusword polls it may sleep however it
//! pleases (fixing python-canopen's five spin loops — V3 in
//! docs/REFERENCES.md).

use crate::state::{step_toward, State402, TargetState, CW_FAULT_RESET_EDGE};

/// Transport glue for one drive.
pub trait DriveIo {
    /// IO error type.
    type Error;
    /// Writes object 0x6040.
    ///
    /// # Errors
    /// Implementation-defined.
    fn write_controlword(&mut self, cw: u16) -> std::result::Result<(), Self::Error>;
    /// Reads object 0x6041.
    ///
    /// # Errors
    /// Implementation-defined.
    fn read_statusword(&mut self) -> std::result::Result<u16, Self::Error>;
}

/// Failure modes of drive operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveError<E> {
    /// Underlying IO failure.
    Io(E),
    /// Target not reached within the supplied deadline.
    Timeout,
    /// No transition path to the requested target.
    Unreachable,
    /// Statusword did not decode to any CiA 402 state.
    UnknownStatusword(u16),
    /// A fault was raised while driving toward the target.
    Faulted(State402),
}

/// Pure state machine: change detection + single-step resolution.
#[derive(Debug, Default)]
pub struct DriveStateMachine {
    last: Option<State402>,
}

impl DriveStateMachine {
    /// Feeds a fresh statusword; returns the new state when it differs from
    /// the previously seen decodable state. `None` input = undecodable.
    pub fn on_statusword(&mut self, sw: Option<State402>) -> Option<State402> {
        let changed = sw.is_some_and(|s| self.last != Some(s));
        if let Some(s) = sw {
            self.last = Some(s);
        }
        changed.then_some(self.last).flatten()
    }

    /// Last decodable state, if any seen so far.
    #[must_use]
    pub const fn state(&self) -> Option<State402> {
        self.last
    }

    /// The next movement step toward `target` from the current state
    /// (`None` when no statusword has been observed yet).
    #[must_use]
    pub fn step_toward_target(&self, target: TargetState) -> Option<crate::state::Step> {
        self.last.map(|s| step_toward(s, State402::from(target)))
    }

    /// Whether the current state is `Fault` or `FaultReactionActive`.
    #[must_use]
    pub const fn is_faulted(&self) -> bool {
        matches!(
            self.last,
            Some(State402::Fault) | Some(State402::FaultReactionActive)
        )
    }
}

/// Synchronous CiA 402 driver over an arbitrary [`DriveIo`].
#[derive(Debug)]
pub struct Drive402<I: DriveIo> {
    io: I,
    machine: DriveStateMachine,
    /// `true` once a fault-reset pulse (bit7 high) has been written and no
    /// command clearing bit7 has been issued since. The next reset pulse must
    /// first drive bit7 low so the device sees a fresh rising edge.
    fault_reset_armed: bool,
}

impl<I: DriveIo> Drive402<I> {
    /// Wraps `io`.
    #[must_use]
    pub fn new(io: I) -> Self {
        Self {
            io,
            machine: DriveStateMachine::default(),
            fault_reset_armed: false,
        }
    }

    /// Borrows the underlying transport.
    pub const fn io(&self) -> &I {
        &self.io
    }

    /// Mutable borrow of the underlying transport (target register writes).
    pub fn io_mut(&mut self) -> &mut I {
        &mut self.io
    }

    /// Polls the statusword once; returns the new state on change.
    ///
    /// # Errors
    /// IO or unknown statusword combinations.
    pub fn poll_statusword(
        &mut self,
    ) -> std::result::Result<Option<State402>, DriveError<I::Error>> {
        let raw = self.io.read_statusword().map_err(DriveError::Io)?;
        let decoded = State402::from_statusword(raw);
        if decoded.is_none() {
            return Err(DriveError::UnknownStatusword(raw));
        }
        Ok(self.machine.on_statusword(decoded))
    }

    /// Current cached state (no bus traffic).
    #[must_use]
    pub const fn state(&self) -> Option<State402> {
        self.machine.state()
    }

    /// Drives toward `target`, writing controlwords as resolved by the
    /// transition graph, until arrived or `timeout_ms` elapse.
    ///
    /// `on_wait` is invoked between poll iterations with `(suggested
    /// back-off ms, current now_ms)` and must return the advanced monotonic
    /// time. Real callers should sleep there (e.g.
    /// `|d, _| { thread::sleep(d…); now_ms() }`); tests may advance a purely
    /// simulated clock (`|_, t| t + 20`), which keeps the loop deadline-
    /// driven instead of busy-spinning the CPU (BUGS.md M6).
    ///
    /// Automatic transitions are awaited via repeated polling. Faults
    /// abort with [`DriveError::Faulted`]; leaving a latched `Fault` toward
    /// `TargetState::SwitchOnDisabled` issues the standard reset pulse
    /// (bit7 low/high alternation, see [`CW_FAULT_RESET_EDGE`]).
    ///
    /// # Errors
    /// See [`DriveError`].
    pub fn set_target_state(
        &mut self,
        target: TargetState,
        timeout_ms: u64,
        mut now_ms: u64,
        mut on_wait: impl FnMut(u64, u64) -> u64,
    ) -> std::result::Result<State402, DriveError<I::Error>> {
        let deadline = now_ms.saturating_add(timeout_ms);
        loop {
            if now_ms > deadline {
                return Err(DriveError::Timeout);
            }
            // Refresh knowledge of the device state.
            let _ = self.poll_statusword()?;
            let Some(current) = self.machine.state() else {
                return Err(DriveError::Unreachable);
            };
            if current == State402::from(target) {
                return Ok(current);
            }
            // A fresh fault while driving somewhere else is fatal for this
            // operation.
            if self.machine.is_faulted()
                && !matches!(current, State402::Fault | State402::FaultReactionActive)
            {
                return Err(DriveError::Faulted(current));
            }
            match crate::state::step_toward(current, target.into()) {
                crate::state::Step::Arrived => return Ok(current),
                crate::state::Step::Write(cmd) => {
                    // Normal commands carry bit7 low, clearing a previously
                    // sent fault-reset level.
                    self.fault_reset_armed = false;
                    self.io
                        .write_controlword(cmd.bits())
                        .map_err(DriveError::Io)?;
                }
                crate::state::Step::FaultReset => {
                    // CiA 402 clears the fault on the *rising* edge of bit7.
                    // If bit7 is still high from an earlier pulse, drive it
                    // low first (python-canopen p402.py reset_from_fault
                    // writes CW_DISABLE_VOLTAGE for the same reason); the
                    // high pulse follows on the next poll iteration.
                    if self.fault_reset_armed {
                        self.io.write_controlword(0).map_err(DriveError::Io)?;
                        self.fault_reset_armed = false;
                    } else {
                        self.io
                            .write_controlword(CW_FAULT_RESET_EDGE)
                            .map_err(DriveError::Io)?;
                        self.fault_reset_armed = true;
                    }
                }
                crate::state::Step::Wait => {}
                crate::state::Step::Unreachable => {
                    return Err(DriveError::Faulted(current));
                }
            }
            now_ms = on_wait(POLL_INTERVAL_MS, now_ms);
        }
    }
}

const POLL_INTERVAL_MS: u64 = 20;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::{step_toward, CwCommand};

    #[test]
    fn decodes_all_eight_states() {
        let cases: [(u16, State402); 8] = [
            (0x0000, State402::NotReadyToSwitchOn),
            (0x0040, State402::SwitchOnDisabled),
            (0x0050, State402::SwitchOnDisabled), // voltage-enabled bit ignored
            (0x0021, State402::ReadyToSwitchOn),
            (0x0031, State402::ReadyToSwitchOn),
            (0x0023, State402::SwitchedOn),
            (0x0027, State402::OperationEnabled),
            (0x000F, State402::FaultReactionActive),
        ];
        for (sw, want) in cases {
            assert_eq!(State402::from_statusword(sw), Some(want), "{sw:#06x}");
        }
        assert_eq!(State402::from_statusword(0x0008), Some(State402::Fault));
        assert_eq!(
            State402::from_statusword(0x0007),
            Some(State402::QuickStopActive)
        );
        assert_eq!(State402::from_statusword(0x00FF), None); // unknown combo
    }

    #[test]
    fn machine_reports_only_changes() {
        let mut m = DriveStateMachine::default();
        assert_eq!(
            m.on_statusword(Some(State402::NotReadyToSwitchOn)),
            Some(State402::NotReadyToSwitchOn)
        );
        assert_eq!(m.on_statusword(Some(State402::NotReadyToSwitchOn)), None);
        assert_eq!(
            m.on_statusword(Some(State402::Fault)),
            Some(State402::Fault)
        );
        assert_eq!(m.on_statusword(None), None); // undecodable keeps last
        assert!(m.is_faulted());
    }

    /// Minimal fake transport: scripted statuswords, recorded controlwords.
    struct FakeDrive {
        statuswords: Vec<u16>,
        idx: usize,
        written: Vec<u16>,
    }
    impl DriveIo for FakeDrive {
        type Error = ();
        fn write_controlword(&mut self, cw: u16) -> Result<(), ()> {
            self.written.push(cw);
            Ok(())
        }
        fn read_statusword(&mut self) -> Result<u16, ()> {
            let sw = *self
                .statuswords
                .get(self.idx)
                .unwrap_or(self.statuswords.last().unwrap_or(&0));
            if self.idx < self.statuswords.len() {
                self.idx += 1;
            }
            Ok(sw)
        }
    }

    #[test]
    fn startup_walk_reaches_operation_enabled() {
        let io = FakeDrive {
            statuswords: vec![
                0x0040, // switch on disabled (boot state)
                0x0021, // ready to switch on  (after shutdown)
                0x0023, // switched on         (after switch on)
                0x0027, // operation enabled   (after enable op)
            ],
            idx: 0,
            written: Vec::new(),
        };
        let mut drive = Drive402::new(io);
        let final_state = drive
            .set_target_state(TargetState::OperationEnabled, 1000, 0, |_, t| {
                t + POLL_INTERVAL_MS
            })
            .unwrap();
        assert_eq!(final_state, State402::OperationEnabled);
        assert_eq!(
            drive.io().written,
            vec![
                CwCommand::Shutdown.bits(),
                CwCommand::SwitchOn.bits(),
                CwCommand::EnableOperation.bits(),
            ]
        );
    }

    #[test]
    fn fault_blocks_until_reset_then_walks_out() {
        let io = FakeDrive {
            statuswords: vec![0x0008, 0x0040, 0x0040],
            idx: 0,
            written: Vec::new(),
        };
        let mut drive = Drive402::new(io);

        // Driving toward OperationEnabled from FAULT is refused outright…
        let io2 = FakeDrive {
            statuswords: vec![0x0008],
            idx: 0,
            written: Vec::new(),
        };
        let mut drive2 = Drive402::new(io2);
        assert_eq!(
            drive2.set_target_state(TargetState::OperationEnabled, 200, 0, |_, t| t
                + POLL_INTERVAL_MS),
            Err(DriveError::Faulted(State402::Fault))
        );

        // …while targeting SwitchOnDisabled performs the reset pulse itself.
        let state = drive
            .set_target_state(TargetState::SwitchOnDisabled, 200, 0, |_, t| {
                t + POLL_INTERVAL_MS
            })
            .unwrap();
        assert_eq!(state, State402::SwitchOnDisabled);
        assert_eq!(drive.io().written.first(), Some(&CW_FAULT_RESET_EDGE));
    }

    #[test]
    fn timeout_surfaces_as_error() {
        // Statusword stuck in switch-on-disabled while target is OE.
        let io = FakeDrive {
            statuswords: vec![0x0040],
            idx: 0,
            written: Vec::new(),
        };
        let mut drive = Drive402::new(io);
        assert_eq!(
            drive.set_target_state(TargetState::OperationEnabled, 60, 0, |_, t| t
                + POLL_INTERVAL_MS),
            Err(DriveError::Timeout)
        );
    }

    #[test]
    fn route_resolution_spot_checks() {
        use crate::state::Step;
        assert_eq!(
            step_toward(State402::SwitchOnDisabled, State402::ReadyToSwitchOn),
            Step::Write(CwCommand::Shutdown)
        );
        assert_eq!(
            step_toward(State402::OperationEnabled, State402::QuickStopActive),
            Step::Write(CwCommand::QuickStop)
        );
        assert_eq!(
            step_toward(State402::NotReadyToSwitchOn, State402::OperationEnabled),
            Step::Wait
        );
        assert_eq!(
            step_toward(State402::NotReadyToSwitchOn, State402::Fault),
            Step::Unreachable
        );
    }

    #[test]
    fn standard_transitions_resolve_in_one_step() {
        // BUGS.md M7: the direct CiA 402 edges must exist so a single
        // command suffices — no multi-hop detours.
        use crate::state::Step;
        // Transition 8: OperationEnabled → ReadyToSwitchOn via Shutdown.
        assert_eq!(
            step_toward(State402::OperationEnabled, State402::ReadyToSwitchOn),
            Step::Write(CwCommand::Shutdown)
        );
        // Transition 10: SwitchedOn → SwitchOnDisabled via DisableVoltage.
        assert_eq!(
            step_toward(State402::SwitchedOn, State402::SwitchOnDisabled),
            Step::Write(CwCommand::DisableVoltage)
        );
    }
}
