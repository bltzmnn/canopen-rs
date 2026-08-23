//! CiA 303-3 indicator LEDs — faithful port of CANopenNode's
//! `303/CO_LEDs.c` state machine (docs/EXTENSIONS.md §B2).
//!
//! Patterns (all timing derives from 50 ms process slices):
//!
//! - **flicker** 10 Hz: alternates between the red and green LED
//! - **blink** 2.5 Hz: 200 ms on/off
//! - **single/double/triple/quadruple flash**: burst of 1–4 pulses every
//!   1.2/1.6/2.0/2.4 s respectively, alternating red/green within each pulse
//!
//! Priority chains (first match wins):
//!
//! ```text
//! red:   bus-off → initialising(flicker) → RPDO timeout(×4) → SYNC timeout(×3)
//!        → HB consumer error(×2) → bus warning(×1) → other error(blink) → off
//! green: LSS configuring(flicker) → firmware download(×3) → stopped(×1)
//!        → pre-operational(blink) → operational(on) → off
//! ```
//!
//! Sans-I/O: [`Leds::process`] takes the elapsed milliseconds since the
//! previous call plus the input snapshot, and returns the current on/off
//! state of both LEDs.

use crate::nmt::NmtState;

// Bit masks mirroring CO_LED_bitmasks.
const FLICKER: u8 = 0x01;
const BLINK: u8 = 0x02;
const FLASH_1: u8 = 0x04;
const FLASH_2: u8 = 0x08;
const FLASH_3: u8 = 0x10;
const FLASH_4: u8 = 0x20;
const CANOPEN: u8 = 0x80;

/// Input snapshot for one [`Leds::process`] call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LedInputs {
    /// Current NMT state.
    pub nmt: NmtState,
    /// LSS configuration session active.
    pub lss_configuring: bool,
    /// CAN controller is bus-off.
    pub bus_off: bool,
    /// CAN error-warning limit reached.
    pub bus_warn: bool,
    /// RPDO not received before its event timer elapsed.
    pub rpdo_timeout: bool,
    /// SYNC reception timeout.
    pub sync_timeout: bool,
    /// Heartbeat-consumer error on a monitored remote node.
    pub hb_consumer_error: bool,
    /// Any other device error.
    pub other_error: bool,
    /// Firmware download running.
    pub firmware_download: bool,
}

/// Current physical on/off states of both indicators.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LedOutput {
    /// Green RUN LED.
    pub green_on: bool,
    /// Red ERROR LED.
    pub red_on: bool,
}

/// The CiA 303-3 LED state machine.
#[derive(Debug)]
pub struct Leds {
    acc_ms: u32,
    tmr_200ms: u8,
    tmr_flash_1: u8,
    tmr_flash_2: u8,
    tmr_flash_3: u8,
    tmr_flash_4: u8,
    /// 10 Hz flicker phase; `true` = red owns the flicker this tick.
    flicker_red: bool,
    red: u8,
    green: u8,
}

impl Default for Leds {
    fn default() -> Self {
        Self::new()
    }
}

impl Leds {
    /// All patterns dark.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            acc_ms: 0,
            tmr_200ms: 0,
            tmr_flash_1: 0,
            tmr_flash_2: 0,
            tmr_flash_3: 0,
            tmr_flash_4: 0,
            flicker_red: false,
            red: 0,
            green: 0,
        }
    }

    /// Advances the machine by `elapsed_ms` and evaluates `inputs`,
    /// returning the LED states. Call periodically (any cadence; internal
    /// slicing is 50 ms).
    #[allow(clippy::too_many_lines)]
    pub fn process(&mut self, elapsed_ms: u64, inputs: &LedInputs) -> LedOutput {
        let mut rd = 0u8;
        let mut gr = 0u8;
        let mut ticked = false;

        self.acc_ms = self
            .acc_ms
            .saturating_add(elapsed_ms.min(u64::from(u32::MAX)) as u32);
        while self.acc_ms >= 50 {
            ticked = true;
            self.acc_ms -= 50;
            // The flicker phase flips every 50 ms tick, so multi-tick
            // process() calls keep the 10 Hz alternation (CiA 303-3).
            self.flicker_red = !self.flicker_red;

            self.tmr_200ms += 1;
            if self.tmr_200ms > 3 {
                // Recompute 2.5 Hz blinking and the flash bursts.
                self.tmr_200ms = 0;
                rd = 0;
                gr = 0;

                // Blink alternates between red and green windows.
                if self.red & BLINK == 0 {
                    rd |= BLINK;
                } else {
                    gr |= BLINK;
                }

                // Flash bursts: red pulse(s), then mirrored on green.
                self.tmr_flash_1 += 1;
                match self.tmr_flash_1 {
                    1 => rd |= FLASH_1,
                    2 => gr |= FLASH_1,
                    6 => self.tmr_flash_1 = 0,
                    _ => {}
                }
                self.tmr_flash_2 += 1;
                match self.tmr_flash_2 {
                    1 | 3 => rd |= FLASH_2,
                    2 | 4 => gr |= FLASH_2,
                    8 => self.tmr_flash_2 = 0,
                    _ => {}
                }
                self.tmr_flash_3 += 1;
                match self.tmr_flash_3 {
                    1 | 3 | 5 => rd |= FLASH_3,
                    2 | 4 | 6 => gr |= FLASH_3,
                    10 => self.tmr_flash_3 = 0,
                    _ => {}
                }
                self.tmr_flash_4 += 1;
                match self.tmr_flash_4 {
                    1 | 3 | 5 | 7 => rd |= FLASH_4,
                    2 | 4 | 6 | 8 => gr |= FLASH_4,
                    12 => self.tmr_flash_4 = 0,
                    _ => {}
                }
            } else {
                // Keep blink/flash bits, clear flicker and CANopen bits.
                rd = self.red & !(FLICKER | CANOPEN);
                gr = self.green & !(FLICKER | CANOPEN);
            }

            // 10 Hz flickering alternates ownership between the two LEDs.
            if self.flicker_red {
                rd |= FLICKER;
            } else {
                gr |= FLICKER;
            }
        }

        if ticked {
            // --- CANopen red ERROR LED (priority chain) -------------------
            let rd_co = if inputs.bus_off {
                1
            } else if inputs.nmt == NmtState::Initialising {
                rd & FLICKER
            } else if inputs.rpdo_timeout {
                rd & FLASH_4
            } else if inputs.sync_timeout {
                rd & FLASH_3
            } else if inputs.hb_consumer_error {
                rd & FLASH_2
            } else if inputs.bus_warn {
                rd & FLASH_1
            } else if inputs.other_error {
                rd & BLINK
            } else {
                0
            };

            // --- CANopen green RUN LED ------------------------------------
            let gr_co = if inputs.lss_configuring {
                gr & FLICKER
            } else if inputs.firmware_download {
                gr & FLASH_3
            } else if inputs.nmt == NmtState::Stopped {
                gr & FLASH_1
            } else if inputs.nmt == NmtState::PreOperational {
                gr & BLINK
            } else if inputs.nmt == NmtState::Operational {
                1
            } else {
                0
            };

            if rd_co != 0 {
                rd |= CANOPEN;
            }
            if gr_co != 0 {
                gr |= CANOPEN;
            }
            self.red = rd;
            self.green = gr;
        }

        LedOutput {
            green_on: self.green & CANOPEN != 0,
            red_on: self.red & CANOPEN != 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn op_inputs() -> LedInputs {
        LedInputs {
            nmt: NmtState::Operational,
            lss_configuring: false,
            bus_off: false,
            bus_warn: false,
            rpdo_timeout: false,
            sync_timeout: false,
            hb_consumer_error: false,
            other_error: false,
            firmware_download: false,
        }
    }

    /// Drives `leds` for `steps` × 50 ms from t=0, collecting outputs.
    fn drive(leds: &mut Leds, inputs: &LedInputs, steps: usize) -> Vec<LedOutput> {
        (1..=steps).map(|_| leds.process(50, inputs)).collect()
    }

    #[test]
    fn operational_is_steady_green() {
        let mut leds = Leds::new();
        let out = drive(&mut leds, &op_inputs(), 20);
        assert!(out.iter().all(|o| o.green_on && !o.red_on));
    }

    #[test]
    fn preoperational_blinks_green_at_2_5hz() {
        let mut leds = Leds::new();
        let inputs = LedInputs {
            nmt: NmtState::PreOperational,
            ..op_inputs()
        };
        drive(&mut leds, &inputs, 4); // warm up: align to the 200 ms grid
        let out = drive(&mut leds, &inputs, 16); // 800 ms = two blink cycles
        let greens: Vec<bool> = out.iter().map(|o| o.green_on).collect();
        // Both phases present, repeating with a 400 ms period.
        assert!(greens.contains(&true) && greens.contains(&false));
        assert_eq!(greens[0], greens[8]);
        assert!(!out.iter().any(|o| o.red_on));
    }

    #[test]
    fn stopped_single_flashes_green_every_1s2() {
        let mut leds = Leds::new();
        let inputs = LedInputs {
            nmt: NmtState::Stopped,
            ..op_inputs()
        };
        let out = drive(&mut leds, &inputs, 24); // 1200 ms = one flash_1 cycle
        let flashes: Vec<bool> = out.iter().map(|o| o.green_on).collect();
        assert!(flashes.iter().filter(|b| **b).count() >= 1);
        // Exactly one burst window (two consecutive 50ms slots lit).
        let bursts = flashes.windows(2).filter(|w| w[0] && !w[1]).count()
            + usize::from(*flashes.last().unwrap());
        assert_eq!(bursts, 1);
    }

    #[test]
    fn heartbeat_error_double_flashes_red() {
        let mut leds = Leds::new();
        let inputs = LedInputs {
            hb_consumer_error: true,
            ..op_inputs()
        };
        let out = drive(&mut leds, &inputs, 32); // 1600 ms = one flash_2 cycle
        let reds: Vec<bool> = out.iter().map(|o| o.red_on).collect();
        // Double flash: two separate lit bursts inside the cycle…
        let transitions_up =
            reds.windows(2).filter(|w| !w[0] && w[1]).count() + usize::from(reds[0]);
        assert_eq!(transitions_up, 2);
        // …while green keeps following the (operational) NMT chain.
        assert!(out.iter().all(|o| o.green_on));
    }

    #[test]
    fn bus_off_is_constant_red_overriding_everything() {
        let mut leds = Leds::new();
        let inputs = LedInputs {
            bus_off: true,
            hb_consumer_error: true,
            ..op_inputs()
        };
        let out = drive(&mut leds, &inputs, 40);
        // Red burns constantly…
        assert!(out.iter().all(|o| o.red_on));
        // …while green simply reflects the (operational) NMT chain, exactly
        // like the reference implementation.
        assert!(out.iter().all(|o| o.green_on));
    }

    #[test]
    fn lss_configuring_flickers_green_alternating_with_red_phase() {
        let mut leds = Leds::new();
        let inputs = LedInputs {
            lss_configuring: true,
            ..op_inputs()
        };
        let out = drive(&mut leds, &inputs, 20);
        // Flicker alternates ownership: green lit on alternating 50ms slices.
        let greens: Vec<bool> = out.iter().map(|o| o.green_on).collect();
        assert!(greens.contains(&true));
        assert!(greens.contains(&false));
        assert_eq!(greens[0], greens[2]); // 100 ms period
    }

    #[test]
    fn initialising_flickers_red() {
        let mut leds = Leds::new();
        let inputs = LedInputs {
            nmt: NmtState::Initialising,
            ..op_inputs()
        };
        let out = drive(&mut leds, &inputs, 20);
        let reds: Vec<bool> = out.iter().map(|o| o.red_on).collect();
        assert!(reds.contains(&true) && reds.contains(&false));
    }

    #[test]
    fn flicker_phase_advances_within_multi_tick_process() {
        // BUGS.md M9: one process() call covering several 50 ms ticks must
        // still flip the flicker phase per tick (10 Hz alternation).
        let mut leds = Leds::new();
        let inputs = LedInputs {
            nmt: NmtState::Initialising,
            ..op_inputs()
        };
        // Three ticks per call: ticks 1..3 own flicker R,G,R → red on;
        // the next call covers ticks 4..6 → G,R,G → red off.
        let mut samples = Vec::new();
        for _ in 0..2 {
            let o = leds.process(150, &inputs);
            samples.push(o.red_on);
        }
        assert_eq!(samples, vec![true, false], "phase stuck: {samples:?}");
    }

    #[test]
    fn error_priority_rpdo_beats_sync_beats_hb() {
        // RPDO timeout (flash_4) wins over SYNC/HB errors: quadruple burst.
        let mut leds = Leds::new();
        let inputs = LedInputs {
            rpdo_timeout: true,
            sync_timeout: true,
            hb_consumer_error: true,
            ..op_inputs()
        };
        let out = drive(&mut leds, &inputs, 32);
        let ups = out
            .iter()
            .map(|o| o.red_on)
            .collect::<Vec<_>>()
            .windows(2)
            .filter(|w| !w[0] && w[1])
            .count()
            + usize::from(out[0].red_on);
        assert_eq!(ups, 4); // quadruple flash proves flash_4 won
    }
}
