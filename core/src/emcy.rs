//! EMCY — emergency objects (CiA 301 §7.2.7): error register layout, an
//! idempotent producer with a shared history ring (object 0x1003) and the
//! consumer-side frame codec.
//!
//! Producer design follows CANopenNode's CO_Emergency (docs/REFERENCES.md
//! S9/S10):
//!
//! - raising/clearing is **idempotent** — an already-active code produces no
//!   new frame;
//! - the error register (0x1001) is recomputed from the active-condition
//!   bitmap rather than poked bit-by-bit;
//! - the history ring doubles as the backing store for object 0x1003
//!   (sub 0 = count of *newest-first* entries);
//! - the register byte is composed at **send time**, so it always reflects
//!   the current state.

use std::collections::BTreeMap;
use std::collections::VecDeque;

use crate::datatypes::{DataType, Value};
use crate::object_dictionary::ObjectDictionary;
use crate::types::{Address, NodeId};

/// Error register bits (object 0x1001).
pub mod err_reg {
    /// Generic error.
    pub const GENERIC: u8 = 1 << 0;
    /// Current.
    pub const CURRENT: u8 = 1 << 1;
    /// Voltage.
    pub const VOLTAGE: u8 = 1 << 2;
    /// Temperature.
    pub const TEMPERATURE: u8 = 1 << 3;
    /// Communication.
    pub const COMMUNICATION: u8 = 1 << 4;
    /// Device-profile specific.
    pub const DEVICE_PROFILE: u8 = 1 << 5;
    /// Manufacturer specific.
    pub const MANUFACTURER: u8 = 1 << 6;

    /// All defined bits.
    pub const ALL: u8 = 0x7F;
}

/// Default number of history slots (and 0x1003 sub-entries).
pub const HISTORY_CAPACITY: usize = 8;

/// EMCY code families with their standard descriptions (python-canopen's
/// classification, docs/REFERENCES.md P18/P12).
#[must_use]
pub fn describe_code(code: u16) -> &'static str {
    match code {
        0x0000 => "error reset / no error",
        0x1000 => "generic error",
        0x2000..=0x2FFF => "current",
        0x3000..=0x3FFF => "voltage",
        0x4000..=0x4FFF => "temperature",
        0x5000..=0x5FFF => "device hardware",
        0x6000..=0x6FFF => "software inefficiency",
        0x7000..=0x7FFF => "additional modules",
        0x8000 => "monitoring",
        0x8100 => "communication",
        0x8110 => "overload",
        0x8120 => "controller mode conflict",
        0x8130 => "protocol error",
        0x8140 => "process data out-dated",
        0x8150 => "transmission mode not available",
        0x8210 => "always high at transmission",
        0x9000..=0x9FFF => "external error",
        0xF000..=0xFFFF => "additional functions",
        _ => "reserved / manufacturer specific",
    }
}

/// A decoded emergency frame payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EmcyFrame {
    /// Emergency code (`0x0000` = reset/no error).
    pub code: u16,
    /// Error register snapshot at send time.
    pub register: u8,
    /// Five manufacturer-specific bytes.
    pub data: [u8; 5],
}

impl EmcyFrame {
    /// Decodes the 8-byte payload of a frame on `0x80 + node`.
    #[must_use]
    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 8 {
            return None;
        }
        Some(Self {
            code: u16::from_le_bytes([data[0], data[1]]),
            register: data[2],
            data: [data[3], data[4], data[5], data[6], data[7]],
        })
    }

    /// Encodes the 8-byte payload.
    #[must_use]
    pub fn encode(&self) -> [u8; 8] {
        let lo = self.code.to_le_bytes();
        [
            lo[0],
            lo[1],
            self.register,
            self.data[0],
            self.data[1],
            self.data[2],
            self.data[3],
            self.data[4],
        ]
    }
}

/// One active or historical error condition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Condition {
    code: u16,
    register_bits: u8,
    info: [u8; 4],
}

/// EMCY producer: maintains active conditions, the error register and the
/// shared history ring; produces frame payloads on state changes only.
#[derive(Debug)]
pub struct EmcyProducer {
    active: BTreeMap<u16, Condition>,
    history: VecDeque<Condition>,
    overflow: bool,
}

impl Default for EmcyProducer {
    fn default() -> Self {
        Self::new()
    }
}

impl EmcyProducer {
    /// A producer without errors.
    #[must_use]
    pub fn new() -> Self {
        Self {
            active: BTreeMap::new(),
            history: VecDeque::with_capacity(HISTORY_CAPACITY),
            overflow: false,
        }
    }

    /// Raises an emergency. Returns the frame payload when this is a *new*
    /// code (idempotent re-raise yields `None`), mirroring 0x1001/0x1003 in
    /// the dictionary as a side effect.
    pub fn raise(
        &mut self,
        od: &mut ObjectDictionary,
        code: u16,
        register_bits: u8,
        info: [u8; 4],
    ) -> Option<[u8; 8]> {
        if self.active.contains_key(&code) {
            return None; // idempotent
        }
        let cond = Condition {
            code,
            register_bits,
            info,
        };
        // History: newest first; overflow sets the sticky flag which itself
        // becomes reportable via [`Self::overflow_active`].
        if self.history.len() == HISTORY_CAPACITY {
            self.history.pop_back();
            self.overflow = true;
        }
        self.history.push_front(cond);
        self.active.insert(code, cond);
        self.sync_od(od);

        let register = self.error_register();
        Some(
            EmcyFrame {
                code,
                register,
                data: [info[0], info[1], info[2], info[3], 0],
            }
            .encode(),
        )
    }

    /// Clears an emergency code. When any errors remain active, returns the
    /// "error reset" payload (`code 0x0000` + current register); when all are
    /// gone, also returns it (CiA 301 permits both; receivers distinguish by
    /// the register byte). Returns `None` when the code was not active.
    pub fn clear(&mut self, od: &mut ObjectDictionary, code: u16) -> Option<[u8; 8]> {
        self.active.remove(&code)?;
        // Full recovery also drops the sticky history-overflow indication;
        // otherwise the error register would stay non-zero with no active
        // error behind it (BUGS.md m5).
        if self.active.is_empty() {
            self.overflow = false;
        }
        self.sync_od(od);
        Some(
            EmcyFrame {
                code: 0x0000,
                register: self.error_register(),
                data: [0; 5],
            }
            .encode(),
        )
    }

    /// Clears the whole error history and the overflow flag, mirroring the
    /// reset into 0x1003 (sub 0 back to the live count of active... i.e. 0).
    /// Invoked when the application writes zero to 0x1003:0 — the semantics
    /// of CANopenNode's `OD_write_1003`.
    pub fn clear_history(&mut self, od: &mut ObjectDictionary) {
        self.history.clear();
        self.overflow = false;
        self.sync_od(od);
    }

    /// Rewrites 0x1001/0x1003 from current producer state. Used after an
    /// external agent touches 0x1003 so the dictionary stays authoritative.
    pub fn resync_od(&mut self, od: &mut ObjectDictionary) {
        self.sync_od(od);
    }

    /// Whether the history overflowed since the last successful raise.
    #[must_use]
    pub const fn overflowed(&self) -> bool {
        self.overflow
    }

    /// Current error-register value (recomputed from active conditions).
    #[must_use]
    pub fn error_register(&self) -> u8 {
        let overflow_bit = if self.overflow { err_reg::GENERIC } else { 0 };
        self.active
            .values()
            .fold(overflow_bit, |acc, c| acc | c.register_bits)
    }

    /// Active codes sorted ascending.
    #[must_use]
    pub fn active_codes(&self) -> Vec<u16> {
        self.active.keys().copied().collect()
    }

    /// History entries, newest first.
    pub fn history(&self) -> impl Iterator<Item = (u16, u8)> + '_ {
        self.history.iter().map(|c| (c.code, c.register_bits))
    }

    /// Mirrors the error register into OD 0x1001 and the newest-first history
    /// into 0x1003 (sub 0 = count, subs 1..n = codes).
    fn sync_od(&mut self, od: &mut ObjectDictionary) {
        od.set_value(
            Address::new(0x1001, 0),
            Value::Uint(u64::from(self.error_register())),
        )
        .ok();
        // Rewrite sub 0 and the used range; subs beyond the live count are
        // zeroed so no ghost codes linger from earlier, longer histories
        // (readers honour sub 0, but a clean table is friendlier and keeps
        // SDO uploads of stale subs well-defined).
        let count = self.history.len() as u64;
        od.set_value(Address::new(0x1003, 0), Value::Uint(count))
            .ok();
        for i in 0..HISTORY_CAPACITY {
            let code = self.history.get(i).map_or(0, |c| u64::from(c.code));
            od.set_value(Address::new(0x1003, i as u8 + 1), Value::Uint(code))
                .ok();
        }
    }
}

/// Callback registered on an [`EmcyConsumer`].
pub type EmcyCallback = Box<dyn Fn(&NodeId, &EmcyFrame) + Send>;

/// Consumer-side listener state (python-canopen `emcy.py` semantics):
///
/// - bounded recent-error log,
/// - **active** table per node cleared by error-reset frames (`code 0x0000`),
/// - registered callbacks invoked in registration order with panic
///   isolation — one failing callback never breaks the others or the
///   caller (docs/REFERENCES.md V6/V11 fixes).
#[derive(Default)]
pub struct EmcyConsumer {
    /// Most recent frames seen, capped.
    log: VecDeque<(NodeId, EmcyFrame)>,
    capacity: usize,
    /// Currently active `(raw node id, code)` pairs; cleared by reset frames.
    active: Vec<(u8, u16)>,
    callbacks: Vec<EmcyCallback>,
}

impl std::fmt::Debug for EmcyConsumer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("EmcyConsumer")
            .field("capacity", &self.capacity)
            .field("log_len", &self.log.len())
            .field("active", &self.active)
            .field("callbacks", &self.callbacks.len())
            .finish()
    }
}

impl EmcyConsumer {
    /// A consumer keeping the last `capacity` frames.
    #[must_use]
    pub fn new(capacity: usize) -> Self {
        Self {
            log: VecDeque::with_capacity(capacity.max(1)),
            capacity: capacity.max(1),
            active: Vec::new(),
            callbacks: Vec::new(),
        }
    }

    /// Registers a callback; invoked in registration order on every frame.
    pub fn add_callback(&mut self, cb: impl Fn(&NodeId, &EmcyFrame) + Send + 'static) {
        self.callbacks.push(Box::new(cb));
    }

    /// Records one decoded frame: updates the log, the active table
    /// (reset code `0x0000` clears that node's entries) and invokes all
    /// callbacks with panic isolation. Returns the frame for event dispatch.
    pub fn on_frame(&mut self, node: NodeId, frame: EmcyFrame) -> EmcyFrame {
        if self.log.len() == self.capacity {
            self.log.pop_back();
        }
        self.log.push_front((node, frame));

        if frame.code == 0x0000 {
            self.active.retain(|(n, _)| *n != node.get());
        } else if !self
            .active
            .iter()
            .any(|(n, c)| *n == node.get() && *c == frame.code)
        {
            self.active.push((node.get(), frame.code));
        }

        for cb in &self.callbacks {
            // Panic isolation: a failing callback must not break the others
            // or the caller (python test_emcy.py:133 behaviour).
            let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                cb(&node, &frame);
            }));
        }
        frame
    }

    /// Currently active `(raw node id, code)` pairs in arrival order.
    #[must_use]
    pub fn active(&self) -> &[(u8, u16)] {
        &self.active
    }

    /// Recent frames, newest first.
    pub fn recent(&self) -> impl Iterator<Item = (NodeId, EmcyFrame)> + '_ {
        self.log.iter().copied()
    }
}

/// Inserts the standard EMCY-related objects into an OD: 0x1014 COB-ID EMCY
/// and the 0x1003 history record skeleton.
pub fn install_standard_objects(od: &mut ObjectDictionary, node: NodeId) {
    use crate::object_dictionary::{AccessType, Entry, Storage};
    let cob = u64::from(0x80 + u16::from(node.get()));
    od.insert(
        Address::new(0x1014, 0),
        Entry::new(
            "COB-ID EMCY",
            DataType::Unsigned32,
            AccessType::Rw,
            Storage::Value(Value::Uint(cob)),
        ),
    );
    od.insert(
        Address::new(0x1003, 0),
        Entry::new(
            "Number of errors",
            DataType::Unsigned8,
            AccessType::Rw,
            Storage::Value(Value::Uint(0)),
        ),
    );
    for sub in 1..=HISTORY_CAPACITY as u8 {
        od.insert(
            Address::new(0x1003, sub),
            Entry::new(
                "Standard error field",
                DataType::Unsigned32,
                AccessType::Ro,
                Storage::Value(Value::Uint(0)),
            ),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::standard::StandardObjects;

    fn od() -> ObjectDictionary {
        let mut od = StandardObjects::new().build();
        install_standard_objects(&mut od, NodeId::new(3).unwrap());
        od
    }

    fn read_reg(od: &ObjectDictionary) -> u64 {
        match od.read(Address::new(0x1001, 0)).unwrap() {
            Value::Uint(v) => v,
            _ => unreachable!(),
        }
    }

    #[test]
    fn frame_roundtrip_and_known_layout() {
        let f = EmcyFrame {
            code: 0x8110,
            register: 0x10,
            data: [1, 2, 3, 4, 5],
        };
        let bytes = f.encode();
        assert_eq!(bytes[0], 0x10); // code little-endian low byte
        assert_eq!(bytes[1], 0x81);
        assert_eq!(bytes[2], 0x10);
        assert_eq!(EmcyFrame::decode(&bytes).unwrap(), f);
        assert!(EmcyFrame::decode(&[0; 4]).is_none());
    }

    #[test]
    fn full_recovery_drops_overflow_flag() {
        // BUGS.md m5: after every active condition cleared, the sticky
        // history-overflow bit must leave the error register too.
        let mut em = EmcyProducer::new();
        let mut od = od();
        for i in 0..=HISTORY_CAPACITY as u16 {
            em.raise(&mut od, 0x1000 + i, err_reg::GENERIC, [0; 4]);
        }
        assert!(em.overflowed());
        assert_eq!(read_reg(&od), u64::from(err_reg::GENERIC));

        // Clear all codes; the last clear must reset the flag.
        for i in 0..=HISTORY_CAPACITY as u16 {
            em.clear(&mut od, 0x1000 + i);
        }
        assert!(!em.overflowed());
        assert_eq!(read_reg(&od), 0);
    }

    #[test]
    fn raise_is_idempotent_and_updates_register() {
        let mut em = EmcyProducer::new();
        let mut od = od();

        let f1 = em.raise(&mut od, 0x2310, err_reg::CURRENT, [0; 4]).unwrap();
        assert_eq!(f1[2], err_reg::CURRENT);
        assert_eq!(read_reg(&od), u64::from(err_reg::CURRENT));

        // Same code again → no new frame.
        assert!(em
            .raise(&mut od, 0x2310, err_reg::CURRENT, [0; 4])
            .is_none());

        // Second, independent condition ORs its bit in.
        em.raise(&mut od, 0x4210, err_reg::TEMPERATURE, [0; 4])
            .unwrap();
        assert_eq!(
            read_reg(&od),
            u64::from(err_reg::CURRENT | err_reg::TEMPERATURE)
        );
        assert_eq!(em.active_codes(), vec![0x2310, 0x4210]);
    }

    #[test]
    fn clear_reports_reset_with_remaining_register() {
        let mut em = EmcyProducer::new();
        let mut od = od();
        em.raise(&mut od, 0x2310, err_reg::CURRENT, [0; 4]).unwrap();
        em.raise(&mut od, 0x4210, err_reg::TEMPERATURE, [0; 4])
            .unwrap();

        let reset = em.clear(&mut od, 0x2310).unwrap();
        assert_eq!(reset[0], 0x00); // code 0x0000 little-endian
        assert_eq!(reset[2], err_reg::TEMPERATURE);

        // Clearing unknown code → None.
        assert!(em.clear(&mut od, 0xBEEF).is_none());

        let reset = em.clear(&mut od, 0x4210).unwrap();
        assert_eq!(reset[2], 0x00);
        assert_eq!(read_reg(&od), 0);
    }

    #[test]
    fn history_ring_is_newest_first_and_shared_with_1003() {
        let mut em = EmcyProducer::new();
        let mut od = od();
        for i in 0..10u16 {
            em.raise(&mut od, 0x2000 + i, err_reg::GENERIC, [0; 4])
                .unwrap();
        }
        // Capacity 8: newest ten kept? No — oldest two dropped, overflow set.
        assert!(em.overflowed());
        let hist: Vec<u16> = em.history().map(|(c, _)| c).collect();
        assert_eq!(
            hist,
            vec![0x2009, 0x2008, 0x2007, 0x2006, 0x2005, 0x2004, 0x2003, 0x2002]
        );

        // 0x1003 mirrors the same ring: sub0=count, sub1=newest.
        assert_eq!(od.read(Address::new(0x1003, 0)).unwrap(), Value::Uint(8));
        assert_eq!(
            od.read(Address::new(0x1003, 1)).unwrap(),
            Value::Uint(u64::from(hist[0]))
        );
    }

    #[test]
    fn consumer_log_is_bounded() {
        let mut cons = EmcyConsumer::new(3);
        let n = NodeId::new(1).unwrap();
        for i in 0..5u16 {
            cons.on_frame(
                n,
                EmcyFrame {
                    code: i,
                    register: 0,
                    data: [0; 5],
                },
            );
        }
        let codes: Vec<u16> = cons.recent().map(|(_, f)| f.code).collect();
        assert_eq!(codes, vec![4, 3, 2]); // newest first, capped at 3
    }

    #[test]
    fn describe_known_families() {
        assert_eq!(describe_code(0x0000), "error reset / no error");
        assert_eq!(describe_code(0x3000), "voltage");
        assert_eq!(describe_code(0x8110), "overload");
    }
}
