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
    /// Reserved (bit 6, always 0 per CiA 301).
    pub const RESERVED: u8 = 1 << 6;
    /// Manufacturer specific.
    pub const MANUFACTURER: u8 = 1 << 7;

    /// All defined bits (reserved bit 6 excluded).
    pub const ALL: u8 =
        GENERIC | CURRENT | VOLTAGE | TEMPERATURE | COMMUNICATION | DEVICE_PROFILE | MANUFACTURER;
}

/// Default number of history slots (and 0x1003 sub-entries).
pub const HISTORY_CAPACITY: usize = 8;

/// `EmcyConsumer::active` 的容量上限（BUGS.md M5）：防止恶意/故障节点通过
/// 持续发送不同 EMCY 码使消费者活动表无界增长、耗尽内存。与 `log` 同款环形上限。
pub const ACTIVE_CAPACITY: usize = 256;

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
        // Aligned with CANopenNode `CO_Emergency.h:138-148` (十L3); the
        // 0x82xx codes below are emitted by this stack itself.
        0x8110 => "CAN overrun (objects lost)",
        0x8120 => "CAN in error passive mode",
        0x8130 => "life guard error or heartbeat error",
        0x8140 => "recovered from bus off",
        0x8150 => "CAN-ID collision",
        0x8200 => "protocol error",
        0x8210 => "PDO not processed due to length error",
        0x8220 => "PDO length exceeded",
        0x8230 => "DAM MPDO not processed, destination object not available",
        0x8240 => "unexpected SYNC data length",
        0x8250 => "RPDO timeout",
        0x9000..=0x9FFF => "external error",
        0xF000..=0xFEFF => "additional functions",
        // CiA 301 / python-canopen `emcy.py:121-122`: the 0xFFxx block is
        // device specific, not part of the additional-functions range
        // (BUGS.md 八L16).
        0xFF00..=0xFFFF => "device specific",
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
    info: [u8; 5],
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
        info: [u8; 5],
    ) -> Option<[u8; 8]> {
        if let Some(cond) = self.active.get_mut(&code) {
            // BUGS.md L1: a repeated raise of an already-active code still
            // merges any *new* error-register bits into the condition (and
            // refreshes 0x1001) — previously the idempotent early-return
            // left the register stale when the same code surfaced through a
            // different error class.
            let register_changed = register_bits & !cond.register_bits != 0;
            if register_changed {
                cond.register_bits |= register_bits;
            }
            // BUGS.md H2: the idempotent path must also keep the vendor info
            // field fresh — a re-raise with newer data used to discard it,
            // so a later re-announcement (or 0x1003 mirror) carried stale
            // bytes.
            cond.info = info;
            // `sync_od` re-borrows `self`, so it must run after the
            // `active` borrow above ends.
            if register_changed {
                self.sync_od(od);
            }
            return None; // idempotent: no new frame
        }
        let cond = Condition {
            code,
            register_bits,
            info,
        };
        // History: newest first. A full ring drops the *new* entry and only
        // sets the sticky overflow flag — CANopenNode `CO_Emergency.c:695-706`
        // (`if (fifoWrPtrNext == fifoPpPtr) { fifoOverflow = 1; } else
        // { store }`) never evicts the oldest message. The flag is
        // reportable via [`Self::overflowed`] and cleared when the history
        // is emptied (`CO_EM_EMERGENCY_BUFFER_FULL` resets once the fifo has
        // drained, `:597-603`).
        if self.history.len() < HISTORY_CAPACITY {
            self.history.push_front(cond);
        } else {
            self.overflow = true;
        }
        self.active.insert(code, cond);
        self.sync_od(od);

        let register = self.error_register();
        Some(
            EmcyFrame {
                code,
                register,
                data: info,
            }
            .encode(),
        )
    }

    /// 返回当前所有 active 条件的 EMCY 帧载荷，供节点在进入 Operational 时
    /// 补发。节点在 Op 之前锁存的故障仅被记录、因 [`Self::raise`] 幂等而从不
    /// 出帧（BUGS.md B3-2）；借此通道重发，避免上位机误判健康。
    pub fn reannounce_active(&self) -> Vec<[u8; 8]> {
        self.active
            .values()
            .map(|cond| {
                EmcyFrame {
                    code: cond.code,
                    register: self.error_register(),
                    data: cond.info,
                }
                .encode()
            })
            .collect()
    }

    /// Clears an emergency code. When **other** conditions remain active,
    /// the returned frame re-announces the remaining highest-priority code
    /// (the lowest active code, ascending); only the clear of the *last*
    /// active code emits the error-reset payload (`code 0x0000`). Returns
    /// `None` when the code was not active.
    ///
    /// A `0x0000` frame tells consumers "this node is fully recovered" — the
    /// [`EmcyConsumer`] wipes the node's entire active table on it — so
    /// emitting it while other errors are still active would desynchronise
    /// producer and consumer (BUGS.md M2).
    pub fn clear(&mut self, od: &mut ObjectDictionary, code: u16) -> Option<[u8; 8]> {
        self.active.remove(&code)?;
        self.sync_od(od);
        let frame = if self.active.is_empty() {
            EmcyFrame {
                code: 0x0000,
                register: self.error_register(),
                data: [0; 5],
            }
        } else {
            // Re-announce the remaining highest-priority condition: the
            // lowest active code (ascending iteration order).
            let cond = self.active.values().next().expect("active is non-empty");
            EmcyFrame {
                code: cond.code,
                register: self.error_register(),
                data: cond.info,
            }
        };
        Some(frame.encode())
    }

    /// Clears the whole error history and the overflow flag, mirroring the
    /// reset into 0x1003 (sub 0 back to the history entry count, i.e. 0).
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
            // Standard error field layout (CANopenNode `CO_Emergency.c:578`):
            // `errorBit<<24 | register<<16 | code`. This stack has no
            // errorBit dimension, but the error register belongs in bits
            // 16..23 (previously only the 16-bit code was written).
            let value = self.history.get(i).map_or(0, |c| {
                (u64::from(c.register_bits) << 16) | u64::from(c.code)
            });
            od.set_value(Address::new(0x1003, i as u8 + 1), Value::Uint(value))
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
    ///
    /// A `capacity` of 0 is clamped to 1 so the ring always holds at least
    /// the newest event (`BUGS.md` R10-P3, documented).
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
            // BUGS.md M5: `active` 此前完全无界——任一远端在订阅 COB 上发 N
            // 个不同 EMCY 码即可无界追加，令消费者内存耗尽。与 `log` 同款
            // 上限，超出时丢弃最旧条目（环形语义）。
            if self.active.len() >= ACTIVE_CAPACITY {
                self.active.remove(0);
            }
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

    /// Clears the active table and recent-frame log. The node calls this on
    /// an NMT communication reset: conditions (and their ordering) from the
    /// previous epoch are stale, mirroring CANopenNode re-initialising the
    /// EM consumer on reset.
    pub fn reset(&mut self) {
        self.active.clear();
        self.log.clear();
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
            // BUGS.md L5: the standard OD installs 0x1001 as Uint — any
            // other type means the test harness mis-configured it.
            _ => unreachable!("0x1001 must be an Unsigned value"),
        }
    }

    #[test]
    fn error_register_bits_match_cia301() {
        // BUGS.md A1: bit 6 is reserved (always 0), bit 7 is manufacturer
        // specific (CANopenNode `CO_Emergency.h:107-108`); the two were
        // previously swapped.
        assert_eq!(err_reg::RESERVED, 0x40);
        assert_eq!(err_reg::MANUFACTURER, 0x80);
        assert_eq!(err_reg::ALL, 0xBF);
        assert_eq!(
            err_reg::ALL & err_reg::RESERVED,
            0,
            "reserved bit must not be part of ALL"
        );
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
    fn full_recovery_keeps_overflow_until_history_cleared() {
        // BUGS.md 十M20: a full history drops the *new* entry and sets the
        // sticky overflow flag; clearing active conditions does not consume
        // the history, so the flag leaves only when the history is emptied
        // (CANopenNode `CO_EM_EMERGENCY_BUFFER_FULL` resets once the fifo
        // has drained, `CO_Emergency.c:597-603`).
        let mut em = EmcyProducer::new();
        let mut od = od();
        for i in 0..=HISTORY_CAPACITY as u16 {
            em.raise(&mut od, 0x1000 + i, err_reg::GENERIC, [0; 5]);
        }
        assert!(em.overflowed());
        assert_eq!(read_reg(&od), u64::from(err_reg::GENERIC));

        // Clear all codes; the overflow indication survives.
        for i in 0..=HISTORY_CAPACITY as u16 {
            em.clear(&mut od, 0x1000 + i);
        }
        assert!(em.overflowed());
        assert_eq!(read_reg(&od), u64::from(err_reg::GENERIC));

        // Only wiping the history (0x1003:0 write) resets the flag.
        em.clear_history(&mut od);
        assert!(!em.overflowed());
        assert_eq!(read_reg(&od), 0);
    }

    #[test]
    fn raise_is_idempotent_and_updates_register() {
        let mut em = EmcyProducer::new();
        let mut od = od();

        let f1 = em.raise(&mut od, 0x2310, err_reg::CURRENT, [0; 5]).unwrap();
        assert_eq!(f1[2], err_reg::CURRENT);
        assert_eq!(read_reg(&od), u64::from(err_reg::CURRENT));

        // Same code again → no new frame.
        assert!(em
            .raise(&mut od, 0x2310, err_reg::CURRENT, [0; 5])
            .is_none());

        // Second, independent condition ORs its bit in.
        em.raise(&mut od, 0x4210, err_reg::TEMPERATURE, [0; 5])
            .unwrap();
        assert_eq!(
            read_reg(&od),
            u64::from(err_reg::CURRENT | err_reg::TEMPERATURE)
        );
        assert_eq!(em.active_codes(), vec![0x2310, 0x4210]);
    }

    /// BUGS.md H2: an idempotent re-raise of an already-active code must
    /// keep the vendor info field fresh — previously the idempotent path
    /// discarded newer `info`, so a later re-announcement carried stale
    /// bytes.
    #[test]
    fn idempotent_reralse_refreshes_vendor_info() {
        let mut em = EmcyProducer::new();
        let mut od = od();

        em.raise(&mut od, 0x2310, err_reg::CURRENT, [1, 2, 3, 4, 5])
            .unwrap();
        // No new frame, but the condition's info must be refreshed.
        assert!(em
            .raise(&mut od, 0x2310, err_reg::CURRENT, [9, 8, 7, 6, 5])
            .is_none());

        let frames = em.reannounce_active();
        assert_eq!(frames.len(), 1);
        assert_eq!(&frames[0][3..8], &[9, 8, 7, 6, 5], "info must be refreshed");
    }

    #[test]
    fn producer_carries_full_manufacturer_field() {
        // BUGS.md P2-2: CiA 301 gives EMCY frames a five-byte manufacturer
        // field (frame byte3..byte7); the producer must transmit all five.
        let mut em = EmcyProducer::new();
        let mut od = od();
        let f = em
            .raise(&mut od, 0x2310, err_reg::CURRENT, [1, 2, 3, 4, 5])
            .unwrap();
        assert_eq!(&f[3..8], &[1, 2, 3, 4, 5]);
    }

    #[test]
    fn clear_reannounces_remaining_and_resets_on_last() {
        let mut em = EmcyProducer::new();
        let mut od = od();
        em.raise(&mut od, 0x2310, err_reg::CURRENT, [0; 5]).unwrap();
        em.raise(&mut od, 0x4210, err_reg::TEMPERATURE, [0; 5])
            .unwrap();

        // BUGS.md M2: clearing one of several active codes must NOT emit a
        // 0x0000 "all clear" frame — the remaining highest-priority
        // (lowest) code is re-announced instead, so consumers keep it
        // active.
        let reset = em.clear(&mut od, 0x2310).unwrap();
        assert_eq!(&reset[..2], &[0x10, 0x42]); // remaining code 0x4210
        assert_eq!(reset[2], err_reg::TEMPERATURE);

        // Clearing unknown code → None.
        assert!(em.clear(&mut od, 0xBEEF).is_none());

        // Clearing the last active code emits the 0x0000 error-reset frame.
        let reset = em.clear(&mut od, 0x4210).unwrap();
        assert_eq!(&reset[..2], &[0x00, 0x00]);
        assert_eq!(reset[2], 0x00);
        assert_eq!(read_reg(&od), 0);
    }

    #[test]
    fn producer_never_emits_false_all_clear() {
        // BUGS.md M2 regression: a producer clearing one of several errors
        // must not emit 0x0000 — consumers wipe the node's whole active
        // table on that code, so a false "all clear" desynchronises them
        // (and a CANopenNode master would misjudge the node recovered).
        let mut em = EmcyProducer::new();
        let mut cons = EmcyConsumer::new(8);
        let mut od = od();
        let node = NodeId::new(3).unwrap();

        let f1 = em.raise(&mut od, 0x2310, err_reg::CURRENT, [0; 5]).unwrap();
        cons.on_frame(node, EmcyFrame::decode(&f1).unwrap());
        let f2 = em
            .raise(&mut od, 0x4210, err_reg::TEMPERATURE, [0; 5])
            .unwrap();
        cons.on_frame(node, EmcyFrame::decode(&f2).unwrap());
        assert_eq!(cons.active().len(), 2);

        // Partial clear: the frame must carry a non-zero code — never the
        // consumer-wide reset marker — and the producer keeps 0x4210.
        let reset = em.clear(&mut od, 0x2310).unwrap();
        assert_ne!(u16::from_le_bytes([reset[0], reset[1]]), 0x0000);
        assert_eq!(em.active_codes(), vec![0x4210]);

        // Final clear: only now does the 0x0000 error-reset frame appear.
        let reset = em.clear(&mut od, 0x4210).unwrap();
        assert_eq!(u16::from_le_bytes([reset[0], reset[1]]), 0x0000);
        assert!(em.active_codes().is_empty());
    }

    #[test]
    fn history_ring_is_newest_first_and_shared_with_1003() {
        let mut em = EmcyProducer::new();
        let mut od = od();
        for i in 0..10u16 {
            em.raise(&mut od, 0x2000 + i, err_reg::GENERIC, [0; 5])
                .unwrap();
        }
        // Capacity 8: the *newest* entries are dropped (CANopenNode never
        // evicts the oldest message, `CO_Emergency.c:695-706`); overflow set.
        assert!(em.overflowed());
        let hist: Vec<u16> = em.history().map(|(c, _)| c).collect();
        assert_eq!(
            hist,
            vec![0x2007, 0x2006, 0x2005, 0x2004, 0x2003, 0x2002, 0x2001, 0x2000]
        );

        // 0x1003 mirrors the same ring: sub0=count, sub1=newest — with the
        // error register in bits 16..23 (CiA 301 standard error field).
        assert_eq!(od.read(Address::new(0x1003, 0)).unwrap(), Value::Uint(8));
        assert_eq!(
            od.read(Address::new(0x1003, 1)).unwrap(),
            Value::Uint((u64::from(err_reg::GENERIC) << 16) | u64::from(hist[0]))
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
        // BUGS.md 十L3: aligned with CANopenNode `CO_Emergency.h`.
        assert_eq!(describe_code(0x8110), "CAN overrun (objects lost)");
        assert_eq!(describe_code(0x8120), "CAN in error passive mode");
        assert_eq!(describe_code(0x8150), "CAN-ID collision");
        assert_eq!(
            describe_code(0x8210),
            "PDO not processed due to length error"
        );
        assert_eq!(describe_code(0x8220), "PDO length exceeded");
        assert_eq!(describe_code(0x8240), "unexpected SYNC data length");
    }

    #[test]
    fn active_table_is_bounded() {
        // BUGS.md M5：`active` 此前完全无界——任一远端持续发送不同 EMCY 码
        // 即可无界追加。修复后超过 `ACTIVE_CAPACITY` 时丢弃最旧条目。
        let mut cons = EmcyConsumer::new(8);
        let node = NodeId::new(1).unwrap();
        for i in 0..(ACTIVE_CAPACITY as u16 + 100) {
            cons.on_frame(
                node,
                EmcyFrame {
                    code: 0x1000 + i,
                    register: 0,
                    data: [0; 5],
                },
            );
        }
        assert!(
            cons.active().len() <= ACTIVE_CAPACITY,
            "active 表应受 ACTIVE_CAPACITY 限制，实际 {}",
            cons.active().len()
        );
        // 最旧条目（code 0x1000）已被挤掉，最新条目（最大 code）仍在。
        let codes: Vec<u16> = cons.active().iter().map(|(_, c)| *c).collect();
        assert!(!codes.contains(&0x1000));
        assert!(codes.contains(&(0x1000 + ACTIVE_CAPACITY as u16 + 99)));
    }

    #[test]
    fn consumer_reset_clears_active_and_log() {
        // An NMT communication reset must drop remote conditions from the
        // previous epoch (node.rs reset branch calls this).
        let mut cons = EmcyConsumer::new(8);
        let node = NodeId::new(1).unwrap();
        cons.on_frame(
            node,
            EmcyFrame {
                code: 0x8110,
                register: 1,
                data: [0; 5],
            },
        );
        assert!(!cons.active().is_empty());
        assert_eq!(cons.recent().count(), 1);
        cons.reset();
        assert!(cons.active().is_empty());
        assert_eq!(cons.recent().count(), 0);
    }
}
