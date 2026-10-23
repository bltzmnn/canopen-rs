//! LSS — Layer Setting Services (CiA 305): node-id assignment over the bus
//! for unconfigured devices, plus Fastscan discovery of unknown identities.
//!
//! Fixed COB-IDs: master→slave [`LSS_MASTER_COB_ID`] (`0x7E5`), slave→master
//! [`LSS_SLAVE_COB_ID`] (`0x7E4`). A slave starts in the *waiting* state and
//! enters the *configuration* state via switch-global, a four-frame selective
//! switch (full match on its [`LssAddress`]) or a completed Fastscan. In the
//! configuration state it accepts a pending node-id which becomes active on
//! the next reset ([`LssSlave::adopt_pending`]).
//!
//! While unconfigured a device serves **only** LSS — its data COB-IDs are not
//! meaningful yet.

/// COB-ID of LSS master→slave frames.
pub const LSS_MASTER_COB_ID: u16 = 0x7E5;
/// COB-ID of LSS slave→master frames.
pub const LSS_SLAVE_COB_ID: u16 = 0x7E4;
/// Raw node-id meaning "unconfigured" (LSS internal).
pub const UNCONFIGURED_NODE_ID: u8 = 0xFF;

/// `bit_check` value that starts a Fastscan (probe all unconfigured nodes).
/// `0x80` matches both reference implementations — CANopenNode's
/// `CO_LSS_FASTSCAN_CONFIRM` and python-canopen's initial probe of 128
/// (docs/DESIGN-DECISIONS.md #3). A legacy `32` is *not* accepted: CANopenNode's
/// `CO_LSS_FASTSCAN_BITCHECK_VALID` rejects it like every other value above
/// 31 — CANopenNode's `CO_LSS_FASTSCAN_BITCHECK_VALID` (`CO_LSSslave.c:30`;
/// the `BIT31`/`CONFIRM` constants live in `CO_LSS.h:133-134`).
pub const FASTSCAN_INIT: u8 = 0x80;

const CS_SWITCH_GLOBAL: u8 = 0x04;
const CS_CONFIGURE_NODE_ID: u8 = 0x11;
pub(super) const CS_CONFIGURE_BIT_TIMING: u8 = 0x13;
/// CiA 305 "activate bit timing" command specifier (`0x15`).
///
/// 注意：不要与 `0x23` 混用——那是 CANopenNode 309 ASCII 网关的内部状态码
/// `CO_GTWA_ST_LSS_CONF_BITRATE`，不是线上的 CS。
pub(super) const CS_ACTIVATE_BIT_TIMING: u8 = 0x15;
const CS_STORE_CONFIGURATION: u8 = 0x17;
const CS_SWITCH_SELECTIVE_VENDOR: u8 = 0x40;
const CS_SWITCH_SELECTIVE_PRODUCT: u8 = 0x41;
const CS_SWITCH_SELECTIVE_REVISION: u8 = 0x42;
const CS_SWITCH_SELECTIVE_SERIAL: u8 = 0x43;
const CS_SWITCH_SELECTIVE_RESPONSE: u8 = 0x44;
const CS_IDENTIFY_REMOTE_VENDOR: u8 = 0x46;
const CS_IDENTIFY_REMOTE_PRODUCT: u8 = 0x47;
const CS_IDENTIFY_REMOTE_REVISION_LOW: u8 = 0x48;
const CS_IDENTIFY_REMOTE_REVISION_HIGH: u8 = 0x49;
const CS_IDENTIFY_REMOTE_SERIAL_LOW: u8 = 0x4A;
const CS_IDENTIFY_REMOTE_SERIAL_HIGH: u8 = 0x4B;
const CS_IDENTIFY_NON_CONFIGURED: u8 = 0x4C;
const CS_IDENTIFY_SLAVE: u8 = 0x4F;
const CS_IDENTIFY_NON_CONFIGURED_SLAVE: u8 = 0x50;
const CS_INQUIRE_VENDOR: u8 = 0x5A;
const CS_INQUIRE_PRODUCT: u8 = 0x5B;
const CS_INQUIRE_REVISION: u8 = 0x5C;
const CS_INQUIRE_SERIAL: u8 = 0x5D;
const CS_INQUIRE_NODE_ID: u8 = 0x5E;
const CS_FASTSCAN: u8 = 0x51;

const MODE_WAITING: u8 = 0;
const MODE_CONFIGURATION: u8 = 1;

/// The 128-bit device identity LSS matches against (objects 0x1018:1..4).
///
/// The identity has two sources — this value and the object dictionary's
/// 0x1018 — and nothing keeps them consistent automatically (the reference
/// leaves the same duty to the application, `CO_LSSslave.c:193`). Prefer
/// [`LssAddress::from_od`] so both come from one place; a mismatch shows up as
/// "the selective switch selects a different node than the SDO read suggests"
/// (`docs/DESIGN-DECISIONS.md` LSS-02).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LssAddress {
    /// Vendor-ID (0x1018:1).
    pub vendor_id: u32,
    /// Product code (0x1018:2).
    pub product_code: u32,
    /// Revision number (0x1018:3).
    pub revision_number: u32,
    /// Serial number (0x1018:4).
    pub serial_number: u32,
}

impl LssAddress {
    /// Reads the identity from an object dictionary's 0x1018:1..4.
    ///
    /// Returns `None` when any of the four entries is missing, unreadable or
    /// not an unsigned integer — the caller then has to supply the identity
    /// explicitly rather than silently getting zeros (`docs/DESIGN-DECISIONS.md` LSS-02).
    #[must_use]
    pub fn from_od(od: &crate::object_dictionary::ObjectDictionary) -> Option<Self> {
        use crate::datatypes::Value;
        use crate::types::Address;
        let sub = |n: u8| -> Option<u32> {
            match od.read(Address::new(0x1018, n)) {
                Ok(Value::Uint(v)) => u32::try_from(v).ok(),
                _ => None,
            }
        };
        Some(Self {
            vendor_id: sub(1)?,
            product_code: sub(2)?,
            revision_number: sub(3)?,
            serial_number: sub(4)?,
        })
    }
}

/// An LSS frame payload (8 bytes on either fixed COB-ID).
pub type LssFrame = [u8; 8];

fn frame(cs: u8) -> LssFrame {
    let mut f = [0u8; 8];
    f[0] = cs;
    f
}

fn frame_u32(cs: u8, value: u32) -> LssFrame {
    let mut f = frame(cs);
    f[1..5].copy_from_slice(&value.to_le_bytes());
    f
}

fn u32_at(f: &LssFrame) -> u32 {
    u32::from_le_bytes([f[1], f[2], f[3], f[4]])
}

// --- Master-side encoders -----------------------------------------------------

/// Switch every node globally to the configuration (`true`) or waiting
/// (`false`) state.
#[must_use]
pub fn encode_switch_global(configuration: bool) -> LssFrame {
    let mut f = frame(CS_SWITCH_GLOBAL);
    f[1] = if configuration {
        MODE_CONFIGURATION
    } else {
        MODE_WAITING
    };
    f
}

/// The four-message sequence that selectively switches the node matching
/// `address` into the configuration state.
#[must_use]
pub fn encode_switch_selective(address: &LssAddress) -> [LssFrame; 4] {
    [
        frame_u32(CS_SWITCH_SELECTIVE_VENDOR, address.vendor_id),
        frame_u32(CS_SWITCH_SELECTIVE_PRODUCT, address.product_code),
        frame_u32(CS_SWITCH_SELECTIVE_REVISION, address.revision_number),
        frame_u32(CS_SWITCH_SELECTIVE_SERIAL, address.serial_number),
    ]
}

/// The six-message CiA 305 *identify remote slave* sequence.
///
/// Unlike selective switching this does not change the slave's LSS state: a
/// matching unconfigured or configured node answers `0x4F`
/// ([`decode_identify_slave_response`]). Vendor/product are equality checks;
/// revision and serial form inclusive `[low, high]` ranges.
#[must_use]
pub fn encode_identify_remote_slave(
    vendor_id: u32,
    product_code: u32,
    revision_low: u32,
    revision_high: u32,
    serial_low: u32,
    serial_high: u32,
) -> [LssFrame; 6] {
    [
        frame_u32(CS_IDENTIFY_REMOTE_VENDOR, vendor_id),
        frame_u32(CS_IDENTIFY_REMOTE_PRODUCT, product_code),
        frame_u32(CS_IDENTIFY_REMOTE_REVISION_LOW, revision_low),
        frame_u32(CS_IDENTIFY_REMOTE_REVISION_HIGH, revision_high),
        frame_u32(CS_IDENTIFY_REMOTE_SERIAL_LOW, serial_low),
        frame_u32(CS_IDENTIFY_REMOTE_SERIAL_HIGH, serial_high),
    ]
}

/// Ask every unconfigured node to identify itself (`0x4C`); such nodes
/// answer `0x50` ([`decode_identify_non_configured_response`]). Configured
/// nodes stay silent.
#[must_use]
pub fn encode_identify_non_configured() -> LssFrame {
    frame(CS_IDENTIFY_NON_CONFIGURED)
}

/// Whether `response` is an *identify slave* response (`0x4F`).
#[must_use]
pub fn decode_identify_slave_response(response: &LssFrame) -> bool {
    response[0] == CS_IDENTIFY_SLAVE
}

/// Whether `response` is an *identify non-configured slave* response
/// (`0x50`).
#[must_use]
pub fn decode_identify_non_configured_response(response: &LssFrame) -> bool {
    response[0] == CS_IDENTIFY_NON_CONFIGURED_SLAVE
}

/// Assign `node_id` to the node currently in the configuration state.
#[must_use]
pub fn encode_configure_node_id(node_id: u8) -> LssFrame {
    let mut f = frame(CS_CONFIGURE_NODE_ID);
    f[1] = node_id;
    f
}

/// Ask the configured node to store its configuration to non-volatile memory.
#[must_use]
pub fn encode_store() -> LssFrame {
    frame(CS_STORE_CONFIGURATION)
}

/// Configure the bit timing of the selected node: `timing_selector` picks
/// the CiA 305 timing table (currently only `0`), `table_entry` is the index
/// into that table's rate list (0=1 Mbit/s … 8=10 kbit/s per CiA 305 §3.4;
/// **index 9 is the standard automatic-bit-rate-detection entry**,
/// `CO_LSS_BIT_TIMING_AUTO` — the slave's validator admits it like any other
/// index ≤ 9 (`CO_LSSslave.c:34`, `CO_LSS.h:180-195`; docs/DESIGN-DECISIONS.md LSS-09).
///
/// API note: [`LssSlave::take_pending_bit_timing`] hands the application the
/// raw `(selector, index)` pair; CANopenNode
/// instead looks the index up in `CO_LSS_bitTimingTableLookup[]` and hands
/// over a numeric bit rate (index 9 → 0). Convert with the CiA 305 table or
/// via `set_bit_timing_supported` when the rate is not available.
///
/// 线上 `table_entry` 仅占单字节 `f[2]`（f[3..6] 为保留位，固定为零），与
/// 从站端 [`LssSlave::handle`] 只读 `req[2]` 的解码一致（docs/DESIGN-DECISIONS.md M6）：旧实现
/// 曾把整个 `u32` 写入 `f[2..6]`，导致 `table_entry` 高位被静默折叠。调用方
/// 传入 >255 的值会被截断为该字节（属越界误用，从站端 `valid_index` 仍会
/// 拒绝 >9 的索引）。
#[must_use]
pub fn encode_configure_bit_timing(timing_selector: u8, table_entry: u32) -> LssFrame {
    let mut f = frame(CS_CONFIGURE_BIT_TIMING);
    f[1] = timing_selector;
    f[2] = table_entry as u8;
    f
}

/// Decodes a configure-bit-timing response: `Ok(())` on success,
/// `Err(error_code)` otherwise. `None` when not such a response.
#[must_use]
pub fn decode_configure_bit_timing_response(response: &LssFrame) -> Option<Result<(), u8>> {
    if response[0] != CS_CONFIGURE_BIT_TIMING {
        return None;
    }
    Some(if response[1] == 0 {
        Ok(())
    } else {
        Err(response[1])
    })
}

/// Broadcast activation of a previously configured bit timing after
/// `delay_ms` — every node switches at (roughly) the same moment.
///
/// CiA 305 把 switch delay 定义为 u16 小端（f[1..3]），支持 >255 ms。
#[must_use]
pub fn encode_activate_bit_timing(delay_ms: u16) -> LssFrame {
    let mut f = frame(CS_ACTIVATE_BIT_TIMING);
    f[1..3].copy_from_slice(&delay_ms.to_le_bytes());
    f
}

/// Inquire the configured node's active node-id.
#[must_use]
pub fn encode_inquire_node_id() -> LssFrame {
    frame(CS_INQUIRE_NODE_ID)
}

/// Inquire the selected node's vendor-id (response CS `0x5A`, u32 LE).
#[must_use]
pub fn encode_inquire_vendor_id() -> LssFrame {
    frame(CS_INQUIRE_VENDOR)
}

/// Inquire the selected node's product code (response CS `0x5B`, u32 LE).
#[must_use]
pub fn encode_inquire_product_code() -> LssFrame {
    frame(CS_INQUIRE_PRODUCT)
}

/// Inquire the selected node's revision number (response CS `0x5C`, u32 LE).
#[must_use]
pub fn encode_inquire_revision_number() -> LssFrame {
    frame(CS_INQUIRE_REVISION)
}

/// Inquire the selected node's serial number (response CS `0x5D`, u32 LE).
#[must_use]
pub fn encode_inquire_serial_number() -> LssFrame {
    frame(CS_INQUIRE_SERIAL)
}

/// Decodes an inquire-vendor-id response into the reported value.
#[must_use]
pub fn decode_inquire_vendor_id_response(response: &LssFrame) -> Option<u32> {
    (response[0] == CS_INQUIRE_VENDOR).then(|| u32_at(response))
}

/// Decodes an inquire-product-code response into the reported value.
#[must_use]
pub fn decode_inquire_product_code_response(response: &LssFrame) -> Option<u32> {
    (response[0] == CS_INQUIRE_PRODUCT).then(|| u32_at(response))
}

/// Decodes an inquire-revision-number response into the reported value.
#[must_use]
pub fn decode_inquire_revision_number_response(response: &LssFrame) -> Option<u32> {
    (response[0] == CS_INQUIRE_REVISION).then(|| u32_at(response))
}

/// Decodes an inquire-serial-number response into the reported value.
#[must_use]
pub fn decode_inquire_serial_number_response(response: &LssFrame) -> Option<u32> {
    (response[0] == CS_INQUIRE_SERIAL).then(|| u32_at(response))
}

/// Whether `response` is a selective-switch confirmation.
#[must_use]
pub const fn is_switch_selective_response(response: &LssFrame) -> bool {
    response[0] == CS_SWITCH_SELECTIVE_RESPONSE
}

/// Decodes a configure-node-id response: `Ok(())` on success,
/// `Err(error_code)` otherwise (`1` = node-id out of range). `None` when the
/// frame is no such response.
#[must_use]
pub fn decode_configure_node_id_response(response: &LssFrame) -> Option<Result<(), u8>> {
    if response[0] != CS_CONFIGURE_NODE_ID {
        return None;
    }
    Some(if response[1] == 0 {
        Ok(())
    } else {
        Err(response[1])
    })
}

/// Decodes an inquire-node-id response into the reported node-id.
#[must_use]
pub fn decode_inquire_node_id_response(response: &LssFrame) -> Option<u8> {
    (response[0] == CS_INQUIRE_NODE_ID).then_some(response[1])
}

// --- Fastscan -----------------------------------------------------------------

/// Encodes an LSS Fastscan request. `id_number` is the candidate value under
/// test, `bit_check` the probed bit ([`FASTSCAN_INIT`] to start; then
/// `31..=0`), `lss_sub` which identity value is scanned (`0`=vendor .. `3`=
/// serial), `lss_next` the sub to move to after confirmation.
#[must_use]
pub fn encode_fastscan(id_number: u32, bit_check: u8, lss_sub: u8, lss_next: u8) -> LssFrame {
    let mut f = frame_u32(CS_FASTSCAN, id_number);
    f[5] = bit_check;
    f[6] = lss_sub;
    f[7] = lss_next;
    f
}

/// Whether `response` is the "identify slave" answer an unconfigured node
/// sends while the Fastscan pattern still matches its identity.
#[must_use]
pub const fn is_identify_slave_response(response: &LssFrame) -> bool {
    response[0] == CS_IDENTIFY_SLAVE
}

/// Sans-I/O LSS master discovering an unconfigured node's [`LssAddress`] by
/// bisecting each of the four identity values MSB-first (CiA 305 §3.7).
///
/// Drive loop:
///
/// ```
/// use canopen_core::lss::{FastscanMaster, LssAddress, LssSlave, UNCONFIGURED_NODE_ID};
///
/// let address = LssAddress {
///     vendor_id: 0x1F, product_code: 0x2A, revision_number: 1, serial_number: 0x1234,
/// };
/// let mut slave = LssSlave::new(address, UNCONFIGURED_NODE_ID);
/// let mut master = FastscanMaster::new();
///
/// while let Some(req) = master.next_request() {
///     // Bus round-trip stand-in: does the slave answer?
///     let answered = slave.handle(&req).is_some();
///     master.on_response(answered);
/// }
/// assert!(master.found() && master.is_complete());
/// assert_eq!(master.address(), address);
/// ```
#[derive(Debug)]
pub struct FastscanMaster {
    id: [u32; 4],
    /// `0..=3` while scanning; `4` once complete.
    sub: u8,
    /// `-2` initial probe, `31..=0` bit under test, `-1` confirm sub.
    bit: i8,
    found: bool,
    /// Re-transmissions allowed per probe before silence becomes decisive.
    /// Default 0 mirrors CANopenNode, where one timed-out probe already
    /// decides ("no response ⇒ assumption wrong"); raise it for noisy buses.
    retries_per_probe: u8,
    retries_left: u8,
    /// Initial-probe retry budget (docs/DESIGN-DECISIONS.md M2): like `confirm_retries_left`,
    /// independent of `retries_left`, which `on_response` re-arms on every
    /// call and therefore could never reach 0 in the `-2` branch. Shared by
    /// the `probe_timed_out` driver hook and the direct `on_response(false)`
    /// path, so both driver styles see the same total.
    probe_retries_left: u8,
    /// Confirmation-phase retry budget (R10-P1-5): armed when the scan
    /// enters the confirm step (`bit == -1`), decremented only there —
    /// unlike `retries_left`, which `on_response` re-arms on every call and
    /// therefore could never reach 0 inside the confirm branch.
    confirm_retries_left: u8,
}

impl Default for FastscanMaster {
    fn default() -> Self {
        Self::new()
    }
}

impl FastscanMaster {
    /// Starts a fresh scan.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            id: [0; 4],
            sub: 0,
            bit: -2,
            found: false,
            retries_per_probe: 0,
            retries_left: 0,
            probe_retries_left: 0,
            confirm_retries_left: 0,
        }
    }

    /// Configures how many times each probe is re-sent after a lapsed
    /// response window ([`Self::probe_timed_out`]) before the silence is
    /// treated as an answer. Default 0 (single probe, CANopenNode style).
    pub fn set_retries_per_probe(&mut self, retries: u8) {
        self.retries_per_probe = retries;
        self.retries_left = retries;
        self.probe_retries_left = retries;
    }

    /// Driver hook for the lapsed response window of the *current* request
    /// (the CiA 305 LSS timeout). Returns `Some(frame)` to re-transmit while
    /// retries remain; `None` marks the silence final — feed
    /// [`Self::on_response`](false) afterwards.
    ///
    /// The driver owns timing entirely: this type never observes clocks
    /// (sans-I/O).
    #[must_use]
    pub fn probe_timed_out(&mut self) -> Option<LssFrame> {
        // docs/DESIGN-DECISIONS.md M2: the initial probe draws from its own budget (shared
        // with the direct `on_response(false)` path) so both driver styles
        // behave identically on a lost first frame.
        if self.bit == -2 {
            if self.probe_retries_left > 0 {
                self.probe_retries_left -= 1;
                self.next_request()
            } else {
                None
            }
        } else if self.retries_left > 0 {
            self.retries_left -= 1;
            self.next_request()
        } else {
            None
        }
    }

    /// The next frame to transmit on [`LSS_MASTER_COB_ID`], or `None` when
    /// finished.
    #[must_use]
    pub fn next_request(&self) -> Option<LssFrame> {
        if self.sub > 3 {
            return None;
        }
        let idx = usize::from(self.sub);
        Some(match self.bit {
            -2 => encode_fastscan(0, FASTSCAN_INIT, 0, 0),
            -1 => encode_fastscan(self.id[idx], 0, self.sub, (self.sub + 1) & 3),
            b => encode_fastscan(self.id[idx], b as u8, self.sub, self.sub),
        })
    }

    /// Reports whether the last request drew an identify-slave response and
    /// advances the scan.
    ///
    /// Calling this after [`Self::is_complete`] is a harmless no-op — late
    /// or duplicated driver feedback must not panic on the finished state
    /// (`docs/DESIGN-DECISIONS.md` P2-11).
    pub fn on_response(&mut self, responded: bool) {
        if self.sub > 3 {
            return;
        }
        self.retries_left = self.retries_per_probe;
        match self.bit {
            -2 => {
                if responded {
                    self.found = true;
                    self.bit = 31;
                } else if self.probe_retries_left > 0 {
                    // docs/DESIGN-DECISIONS.md M2: a lost initial probe must not end the
                    // whole scan while retries remain — stay on `-2` so
                    // `next_request` re-emits the probe. `probe_retries_left`
                    // is independent of `retries_left` (re-armed above).
                    self.probe_retries_left -= 1;
                } else {
                    self.sub = 4; // nobody answered the probe
                }
            }
            -1 => {
                // Confirmation phase (docs/DESIGN-DECISIONS.md P3-45): the confirmation frame
                // may be lost on a noisy bus; re-probe this sub until the
                // *confirmation* budget runs out. Uses its own counter
                // because `on_response` re-arms `retries_left` on every call,
                // which made the old check unreachable (R10-P1-5).
                if responded {
                    self.sub += 1; // sub confirmed; next value
                    self.bit = 31;
                } else if self.confirm_retries_left == 0 {
                    // Budget exhausted with no confirmation: the scan failed.
                    // Trusting an unverified address produced phantom
                    // discoveries; both references abort here — CANopenNode
                    // `CO_LSSmaster.c:792-821` (`FsVerifyWait` → `SCAN_NOACK`)
                    // and python-canopen `lss.py:274-275`
                    // (`return False, None`). (docs/DESIGN-DECISIONS.md B7.)
                    self.found = false;
                    self.sub = 4;
                } else {
                    self.confirm_retries_left -= 1;
                }
            }
            b => {
                // No answer ⇒ this bit of the identity is a 1.
                if !responded {
                    self.id[usize::from(self.sub)] |= 1u32 << b;
                }
                self.bit -= 1;
                if self.bit == -1 {
                    // Entering the confirmation phase: arm its own budget so
                    // a silent slave gives up after N tries instead of
                    // oscillating forever (docs/DESIGN-DECISIONS.md R10-P1-5).
                    self.confirm_retries_left = self.retries_per_probe;
                }
            }
        }
    }

    /// Whether an unconfigured node answered the initial probe.
    #[must_use]
    pub const fn found(&self) -> bool {
        self.found
    }

    /// Feeds a raw response frame received on [`LSS_SLAVE_COB_ID`], applying
    /// a command-specifier check to its CS: only an "identify slave" frame
    /// ([`is_identify_slave_response`], CS `0x4F`) counts as an answer; any
    /// other CS is treated as *silence* for the current probe — it marks the
    /// probed bit as **set** (`on_response(false)` does `bits |= 1 << bit`,
    /// exactly like python-canopen `lss.py`'s "no answer" handling), i.e. the
    /// opposite of the "bit stays 0/inverted" wording this comment used
    /// before (`docs/DESIGN-DECISIONS.md` DOC-03).
    ///
    /// Two-reference divergence (`docs/DESIGN-DECISIONS.md` 审-L1): python-canopen compares
    /// the CS the same way (`lss.py:266-267,293-297`, silent skip), while
    /// CANopenNode fails the whole scan on any wrong-CS frame
    /// (`CO_LSSmaster.c:738-740,814-816` return `CO_LSSmaster_SCAN_FAILED`).
    /// This stack follows python-canopen; the reference line range is cited
    /// only for the *existence* of the CS check, not for its outcome.
    ///
    /// Prefer this over [`Self::on_response`] in drivers unless the
    /// transport already filters the CS (`docs/DESIGN-DECISIONS.md` 七F6).
    pub fn on_response_frame(&mut self, response: &LssFrame) {
        self.on_response(is_identify_slave_response(response));
    }

    /// Whether the scan ran to completion.
    #[must_use]
    pub const fn is_complete(&self) -> bool {
        self.sub > 3
    }

    /// The discovered address — meaningful once complete and found.
    #[must_use]
    pub const fn address(&self) -> LssAddress {
        LssAddress {
            vendor_id: self.id[0],
            product_code: self.id[1],
            revision_number: self.id[2],
            serial_number: self.id[3],
        }
    }
}

/// Discovers **all** unconfigured nodes on the bus by repeated Fastscan:
/// each found node is assigned the next temporary node-id and stored, so it
/// leaves the unconfigured pool and the next scan round finds another.
///
/// The driver is transport-agnostic: emit each [`ScanAction`] frame, wait
/// for responses per the action's expectation, feed results back via
/// [`AllNodesScanner::on_response`].
///
/// ```
/// use canopen_core::lss::{
///     encode_configure_node_id, encode_store, encode_switch_global, AllNodesScanner,
///     LssAddress, LssSlave, ScanAction, UNCONFIGURED_NODE_ID,
/// };
///
/// # let addresses = vec![LssAddress { vendor_id: 1, product_code: 1, revision_number: 1, serial_number: 1 }];
/// # // Stand-in bus: a single unconfigured slave with `addresses[0]`.
/// # let mut slave = LssSlave::new(addresses[0], UNCONFIGURED_NODE_ID);
/// let mut scanner = AllNodesScanner::new(10);
/// let mut discovered = Vec::new();
/// loop {
///     match scanner.next_action() {
///         Some(ScanAction::Send(frame, expect_answer)) => {
///             // Transmit on 0x7E5; wait up to the timeout for an answer on
///             // 0x7E4 when expect_answer is true.
///             let answered = expect_answer && slave.handle(&frame).is_some();
///             scanner.on_response(answered);
///         }
///         Some(ScanAction::AssignTempId(id)) => {
///             // The discovered node (in Configuration) receives its id…
///             let _ = slave.handle(&encode_configure_node_id(id));
///             let _ = slave.handle(&encode_store());
///             discovered.push(*scanner.discovered().last().unwrap());
///             scanner.on_assigned();
///         }
///         Some(ScanAction::Deselect) => {
///             // The master switches the node back to Waiting; the node then
///             // requests the communication reset that makes the id active
///             // (`CO_LSSslave.c:56-66`, docs/DESIGN-DECISIONS.md LSS-11). No answer expected.
///             let _ = slave.handle(&encode_switch_global(false));
///             // …which `Node::on_frame` performs via `take_reset_requested()`;
///             // in this bus-less example we do it by hand.
///             assert!(slave.take_reset_requested());
///             slave.adopt_pending();
///             if !scanner.on_deselected() {
///                 break;
///             }
///         }
///         None => break,
///     }
/// }
/// assert_eq!(scanner.discovered(), &addresses);
/// ```
#[derive(Debug)]
pub struct AllNodesScanner {
    fastscan: FastscanMaster,
    next_id: u8,
    discovered: Vec<LssAddress>,
    /// Set while the current find is being assigned an id.
    assigning: bool,
    /// Set between the store and the follow-up switch-global(waiting) that
    /// ends the node's configuration session (`docs/DESIGN-DECISIONS.md` LSS-11).
    deselecting: bool,
    max_nodes: u8,
}

/// One step of the all-nodes discovery loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanAction {
    /// Transmit this frame on `0x7E5`. When `expect_answer` is set, wait for
    /// a response frame on `0x7E4` within the configured timeout and feed it
    /// to [`AllNodesScanner::on_response_frame`] (which applies the
    /// reference master's CS `0x4F` check); use
    /// [`AllNodesScanner::on_response`] instead only when the driver has
    /// already validated the command specifier (`docs/DESIGN-DECISIONS.md` 七F6).
    Send(LssFrame, bool),
    /// A node was just discovered and selected; send this configure-node-id
    /// request followed by [`encode_store`], then call
    /// [`AllNodesScanner::on_assigned`].
    AssignTempId(u8),
    /// The temporary id is stored: transmit this `switch mode global:
    /// waiting` frame, then call [`AllNodesScanner::on_deselected`].
    ///
    /// This is the step that makes the assignment *effective*. CiA 305 has no
    /// other trigger: a node leaves the unconfigured pool only after the
    /// master switches it back to Waiting, which makes it request an NMT
    /// communication reset and adopt its pending node-id
    /// (`CO_LSSslave.c:56-66,275-280`). The reference master sends exactly
    /// this frame after the store — `CO_LSSmaster_swStateDeselect` in
    /// `CO_gateway_ascii.c:1879-1886`, unconditional and not acknowledged —
    /// so a scanner that skips it leaves every discovered node at
    /// node-id `0xFF` (`docs/DESIGN-DECISIONS.md` LSS-11).
    ///
    /// The frame is **not** confirmed: the reference calls it a
    /// "non-confirmed message" and continues regardless, so
    /// [`AllNodesScanner::on_deselected`] takes no `bool` about a response.
    Deselect,
}

impl AllNodesScanner {
    /// Scans until `max_nodes` are found or the bus runs dry.
    #[must_use]
    pub const fn new(max_nodes: u8) -> Self {
        Self {
            fastscan: FastscanMaster::new(),
            next_id: 1,
            discovered: Vec::new(),
            assigning: false,
            deselecting: false,
            max_nodes,
        }
    }

    /// Next driver step or `None` when complete.
    #[must_use]
    pub fn next_action(&self) -> Option<ScanAction> {
        if self.assigning {
            // The id pool is 1..=127; a saturated counter (255) can only be
            // reached by a driver that keeps calling `on_deselected` past its
            // `false` return, and must not be handed out (`docs/DESIGN-DECISIONS.md` LSS-12).
            return (self.next_id <= 127).then_some(ScanAction::AssignTempId(self.next_id));
        }
        if self.deselecting {
            return Some(ScanAction::Deselect);
        }
        if self.fastscan.is_complete() || usize::from(self.max_nodes) <= self.discovered.len() {
            return None;
        }
        self.fastscan
            .next_request()
            .map(|f| ScanAction::Send(f, true))
    }

    /// Feeds whether the last sent frame drew a slave response.
    pub fn on_response(&mut self, responded: bool) {
        if self.assigning {
            // Assignment phase: fastscan feedback belongs to the previous
            // round; ignore it instead of re-entering a finished scan.
            return;
        }
        self.fastscan.on_response(responded);
        if self.fastscan.found() && self.fastscan.is_complete() {
            self.discovered.push(self.fastscan.address());
            self.assigning = true;
        }
    }

    /// Feeds a raw response frame received on [`LSS_SLAVE_COB_ID`]: only an
    /// "identify slave" frame counts as an answer (`docs/DESIGN-DECISIONS.md` 七F6). Prefer
    /// this over [`Self::on_response`] when the transport does not already
    /// filter the command specifier.
    pub fn on_response_frame(&mut self, response: &LssFrame) {
        self.on_response(is_identify_slave_response(response));
    }

    /// Confirms the temp-id assignment completed (configure + store acked);
    /// scanning continues for further nodes.
    ///
    /// Known limitation (`docs/DESIGN-DECISIONS.md` R10-P2-9): the scanner assumes every
    /// completed fastscan round found a real node. On buses with multiple
    /// heterogeneous unconfigured nodes the identity AND can converge to a
    /// phantom address; applications should verify the configure/store
    /// acknowledgement of each assigned node. Returns false when ids ran out.
    ///
    /// Returns `true`; the assignment is only *complete* once the follow-up
    /// [`ScanAction::Deselect`] frame has been sent — use
    /// [`Self::on_deselected`] for the "are there more nodes?" answer, which
    /// is where the id-pool check moved (`docs/DESIGN-DECISIONS.md` LSS-11).
    pub fn on_assigned(&mut self) -> bool {
        self.assigning = false;
        self.deselecting = true;
        true
    }

    /// Confirms the [`ScanAction::Deselect`] frame was transmitted; scanning
    /// continues with the next temporary id. Returns `false` when the id pool
    /// is exhausted (also the `max_nodes` stop condition is handled by
    /// [`Self::next_action`]).
    pub fn on_deselected(&mut self) -> bool {
        self.deselecting = false;
        // docs/DESIGN-DECISIONS.md LSS-12: saturating, so a driver that keeps calling this past
        // `false` walks up to 255 instead of panicking in debug builds or
        // wrapping to 0 in release (where node-id 0 would be handed out).
        self.next_id = self.next_id.saturating_add(1);
        let more = self.next_id <= 127;
        // docs/DESIGN-DECISIONS.md #32: when the temp-id pool is exhausted keep the previous
        // (completed) fastscan in place so `next_action` reports `None`
        // instead of starting yet another scan round — a driver that ignores
        // our `false` return cannot loop forever.
        if more {
            self.fastscan = FastscanMaster::new();
        }
        more
    }

    /// Addresses discovered so far, in assignment order.
    #[must_use]
    pub fn discovered(&self) -> &[LssAddress] {
        &self.discovered
    }

    /// Whether no further actions remain.
    #[must_use]
    pub const fn is_complete(&self) -> bool {
        !self.assigning && !self.deselecting && self.fastscan.is_complete()
    }
}

/// LSS operational states of a slave.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LssState {
    /// Normal operation; fastscan participation possible.
    Waiting,
    /// Selected for configuration services.
    Configuration,
}

/// Node-side LSS state machine. Feed each frame received on
/// [`LSS_MASTER_COB_ID`] to [`LssSlave::handle`] and transmit any returned
/// frame on [`LSS_SLAVE_COB_ID`].
#[derive(Debug)]
pub struct LssSlave {
    address: LssAddress,
    node_id: u8,
    pending_node_id: u8,
    /// Bit-rate change configured but not yet applied:
    /// `(timing_selector, table_entry)`; consumed by the application on the
    /// reset/restart that follows [`CS_ACTIVATE_BIT_TIMING`].
    pending_bit_timing: Option<(u8, u32)>,
    /// Set when the master sends *store configuration*; drained via
    /// [`LssSlave::take_store_requested`] so the application knows when to
    /// persist node-id / bit-timing (`docs/DESIGN-DECISIONS.md` P2-16).
    store_requested: bool,
    /// Whether the application provides a store callback. `false` answers
    /// *store configuration* with error 1 ("not supported"), like
    /// CANopenNode when `pFunctLSScfgStore == NULL`
    /// (`CO_LSSslave.c:351-372`).
    store_supported: bool,
    /// Switch delay of the last *activate bit timing* broadcast (u16 ms,
    /// CiA 305 §3.4), drained via
    /// [`LssSlave::take_activate_bit_timing_delay`]. CANopenNode hands the
    /// parsed value to the application instead of discarding it
    /// (`CO_LSSslave.c:337-349`).
    activate_bit_timing_delay: Option<u16>,
    /// Whether the application can change the bit rate at all. Mirrors
    /// CANopenNode's `pFunctLSScheckBitRate != NULL` gate
    /// (`CO_LSSslave.c:303-341`): with `false`, *configure bit timing* and
    /// *activate bit timing* are silently dropped (no response, no
    /// forwarded delay), exactly like the reference without the callback.
    bit_timing_supported: bool,
    /// Set when the slave must autonomously run an NMT communication reset
    /// (configured node switched back to Waiting with a still-invalid active
    /// id); drained via [`LssSlave::take_reset_requested`]. Mirrors
    /// CANopenNode's `CO_LSS_SWITCH_STATE_GLOBAL` service
    /// (`CO_LSSslave.c:56-66,275-280`).
    reset_requested: bool,
    state: LssState,
    selective: u8,
    fs_sub: u8,
    /// Strict CiA 305 *identify remote slave* collection state: `None` when
    /// idle, otherwise the six already-received values and the next expected
    /// step (`0..=5`, then matching). Repeated/out-of-order intermediate
    /// frames reset it silently (strict sequence, docs/DESIGN-DECISIONS.md LSS-07).
    identify: Option<IdentifySession>,
}

/// Values collected by one identify-remote-slave sequence.
#[derive(Debug, Clone, Copy, Default)]
struct IdentifySession {
    vendor_id: u32,
    product_code: u32,
    revision_low: u32,
    revision_high: u32,
    serial_low: u32,
    serial_high: u32,
    /// Next expected command-specifier step (0 = vendor).
    step: u8,
}

impl LssSlave {
    /// A slave with `address` and current raw node-id
    /// ([`UNCONFIGURED_NODE_ID`] when none assigned yet).
    ///
    /// # Panics (debug builds)
    /// `node_id` outside `1..=127 ∪ {0xFF}` would put the slave into an
    /// unrepresentable state; debug builds assert, release builds proceed
    /// with the caller's value (`docs/DESIGN-DECISIONS.md` P3-44).
    #[must_use]
    #[allow(clippy::manual_range_contains)]
    pub const fn new(address: LssAddress, node_id: u8) -> Self {
        debug_assert!(
            (node_id >= 1 && node_id <= 127) || node_id == UNCONFIGURED_NODE_ID,
            "LSS node-id must be 1..=127 or 0xFF"
        );
        Self {
            address,
            node_id,
            pending_node_id: node_id,
            pending_bit_timing: None,
            store_requested: false,
            store_supported: true,
            activate_bit_timing_delay: None,
            bit_timing_supported: true,
            reset_requested: false,
            state: LssState::Waiting,
            selective: 0,
            fs_sub: 0,
            identify: None,
        }
    }

    /// Declares whether the application can persist LSS configuration.
    /// With `false`, *store configuration* answers error 1 (not supported)
    /// and `take_store_requested` never fires (CANopenNode
    /// `CO_LSSslave.c:351-372`).
    pub fn set_store_supported(&mut self, supported: bool) {
        self.store_supported = supported;
    }

    /// Declares whether the application can change the CAN bit rate. With
    /// `false`, *configure bit timing* and *activate bit timing* are
    /// silently dropped — no response and no
    /// [`LssSlave::take_activate_bit_timing_delay`] event — matching
    /// CANopenNode when no `pFunctLSScheckBitRate` was registered
    /// (`CO_LSSslave.c:303-341`, `docs/DESIGN-DECISIONS.md` N12/L1). Defaults to `true`
    /// (the previous always-answer behaviour).
    pub fn set_bit_timing_supported(&mut self, supported: bool) {
        self.bit_timing_supported = supported;
    }

    /// Returns and clears whether the master requested *store configuration*
    /// since the last call. The application should persist the pending node-id /
    /// bit-timing now (`docs/DESIGN-DECISIONS.md` P2-16 — previously the event had no exit at all).
    pub fn take_store_requested(&mut self) -> bool {
        std::mem::take(&mut self.store_requested)
    }

    /// Takes the switch delay (ms) of the last *activate bit timing*
    /// broadcast, if any. CANopenNode forwards this value to the
    /// application (`CO_LSSslave.c:337-349`); dropping it made applications
    /// unable to time the switch.
    pub fn take_activate_bit_timing_delay(&mut self) -> Option<u16> {
        self.activate_bit_timing_delay.take()
    }

    /// Returns and clears whether the slave must run an NMT communication
    /// reset (a freshly configured node was switched back to Waiting with a
    /// pending node-id; CANopenNode requests the reset autonomously,
    /// `CO_LSSslave.c:56-66,275-280`). Applications that drive the reset
    /// themselves can observe the request here instead.
    pub fn take_reset_requested(&mut self) -> bool {
        std::mem::take(&mut self.reset_requested)
    }

    /// Takes the bit-rate change staged by configure/activate, if any.
    pub fn take_pending_bit_timing(&mut self) -> Option<(u8, u32)> {
        self.pending_bit_timing.take()
    }

    /// Active raw node-id (`0xFF` while unconfigured).
    #[must_use]
    pub const fn node_id(&self) -> u8 {
        self.node_id
    }

    /// Node-id assigned by the master but not yet adopted.
    #[must_use]
    pub const fn pending_node_id(&self) -> u8 {
        self.pending_node_id
    }

    /// Current LSS state.
    #[must_use]
    pub const fn state(&self) -> LssState {
        self.state
    }

    /// This node's identity.
    #[must_use]
    pub const fn address(&self) -> LssAddress {
        self.address
    }

    /// Applies the pending node-id as active — call from the reset handler
    /// that follows an LSS configuration session. The reset also ends the
    /// configuration session: the node returns to the waiting state (a real
    /// device restarts its communication stack here).
    pub fn adopt_pending(&mut self) {
        self.node_id = self.pending_node_id;
        self.state = LssState::Waiting;
        self.fs_sub = 0;
        self.identify = None;
        // The pending id is active now — a pending reset request has been
        // fulfilled.
        self.reset_requested = false;
    }

    /// NMT reset handler (Reset Node / Reset Communication): abandons any
    /// in-progress LSS session and returns to Waiting. The pending node-id /
    /// bit-timing / store flag are kept — they are consumed by the
    /// application on the reset via [`Self::adopt_pending`] /
    /// [`Self::take_pending_bit_timing`] / [`Self::take_store_requested`]
    /// (docs/DESIGN-DECISIONS.md M3).
    pub fn reset(&mut self) {
        self.state = LssState::Waiting;
        self.selective = 0;
        self.fs_sub = 0;
        self.identify = None;
        // A pending reset request has just been executed.
        self.reset_requested = false;
    }

    fn identity(&self, sub: u8) -> u32 {
        match sub {
            0 => self.address.vendor_id,
            1 => self.address.product_code,
            2 => self.address.revision_number,
            _ => self.address.serial_number,
        }
    }

    fn handle_fastscan(&mut self, req: &LssFrame) -> Option<LssFrame> {
        // Only unconfigured waiting nodes participate in discovery. Both the
        // *active* and *pending* ids must be unconfigured: a node whose
        // node-id was configured but not yet adopted is treated as
        // configured by CANopenNode (`CO_LSSslave.c:104-107`) and must not
        // answer fastscan (`docs/DESIGN-DECISIONS.md` C4).
        if self.node_id != UNCONFIGURED_NODE_ID
            || self.pending_node_id != UNCONFIGURED_NODE_ID
            || self.state != LssState::Waiting
        {
            return None;
        }
        let id_number = u32_at(req);
        let bit_check = req[5];
        let lss_sub = req[6];
        let lss_next = req[7];

        // Validate every field like CANopenNode's slave
        // (`CO_LSSslave.c:113-121`): a malformed probe (e.g. an out-of-range
        // lss_sub/lss_next together with the confirm bit) must be silently
        // ignored, not reset the scan (docs/DESIGN-DECISIONS.md 十L6). `bit_check` accepts
        // 0..=31 plus the confirm value; the legacy 32 is *not* a reset —
        // CANopenNode's `CO_LSS_FASTSCAN_BITCHECK_VALID` rejects it and the
        // request is dropped (`CO_LSS_FASTSCAN_BITCHECK_VALID`,
        // `CO_LSSslave.c:30,115-117`).
        if (bit_check > 31 && bit_check != FASTSCAN_INIT) || lss_sub > 3 || lss_next > 3 {
            return None;
        }
        if bit_check == FASTSCAN_INIT {
            self.fs_sub = 0;
            return Some(frame(CS_IDENTIFY_SLAVE));
        }
        if lss_sub != self.fs_sub {
            return None;
        }
        // Match only the bits the master has already determined
        // ([bit_check..=31]: check 31 → one bit, check 0 → whole value).
        let mask = u32::MAX << bit_check;
        if (id_number ^ self.identity(lss_sub)) & mask != 0 {
            return None;
        }
        // A full match with changing lss_next confirms this sub. The scan
        // completes when the confirmed sub is past the next expected one
        // (`lss_next < lss_sub`, CANopenNode `CO_LSSslave.c:138-141`), which
        // in the normal flow happens only after the serial sub (3).
        if bit_check == 0 && lss_next != lss_sub {
            // CANopenNode acks by advancing `fastscanPos = lssNext` before
            // the completion check (`CO_LSSslave.c:136`), so a completed scan
            // leaves position 0 (VENDOR_ID) rather than the serial index —
            // a non-conformant master that skips the 0x80 restart and
            // continues directly is still answered (`docs/DESIGN-DECISIONS.md` L4).
            self.fs_sub = lss_next;
            if lss_next < lss_sub {
                self.state = LssState::Configuration;
            }
        }
        Some(frame(CS_IDENTIFY_SLAVE))
    }

    /// Collects one frame of the strict six-step identify sequence and
    /// answers `0x4F` once the full identity matches. Vendor/product are
    /// exact; revision/serial are inclusive ranges (CiA 305).
    fn handle_identify(&mut self, req: &LssFrame) -> Option<LssFrame> {
        let mut session = if req[0] == CS_IDENTIFY_REMOTE_VENDOR {
            IdentifySession::default()
        } else {
            self.identify.unwrap_or_default()
        };
        if session.step >= 6 || req[0] != CS_IDENTIFY_REMOTE_VENDOR + session.step {
            self.identify = None;
            return None;
        }
        let value = u32_at(req);
        match session.step {
            0 => session.vendor_id = value,
            1 => session.product_code = value,
            2 => session.revision_low = value,
            3 => session.revision_high = value,
            4 => session.serial_low = value,
            _ => session.serial_high = value,
        }
        session.step += 1;
        if session.step == 6 {
            self.identify = None;
            let revision_ok = session.revision_low <= self.address.revision_number
                && self.address.revision_number <= session.revision_high;
            let serial_ok = session.serial_low <= self.address.serial_number
                && self.address.serial_number <= session.serial_high;
            let matches = session.vendor_id == self.address.vendor_id
                && session.product_code == self.address.product_code
                && revision_ok
                && serial_ok;
            return matches.then(|| frame(CS_IDENTIFY_SLAVE));
        }
        self.identify = Some(session);
        None
    }

    /// Processes one master frame, returning the response to transmit.
    pub fn handle(&mut self, req: &LssFrame) -> Option<LssFrame> {
        // Strict identify-remote-slave sequence (CiA 305; docs/DESIGN-DECISIONS.md LSS-07):
        // any non-identify request abandons the partial collection, and a
        // repeated/out-of-order intermediate frame resets it silently.
        if !(CS_IDENTIFY_REMOTE_VENDOR..=CS_IDENTIFY_NON_CONFIGURED).contains(&req[0]) {
            self.identify = None;
        }
        if req[0] == CS_IDENTIFY_NON_CONFIGURED {
            self.identify = None;
            return (self.node_id == UNCONFIGURED_NODE_ID
                && self.pending_node_id == UNCONFIGURED_NODE_ID)
                .then(|| frame(CS_IDENTIFY_NON_CONFIGURED_SLAVE));
        }
        if (CS_IDENTIFY_REMOTE_VENDOR..=CS_IDENTIFY_REMOTE_SERIAL_HIGH).contains(&req[0]) {
            return self.handle_identify(req);
        }
        let configuring = self.state == LssState::Configuration;
        match req[0] {
            CS_SWITCH_GLOBAL => {
                match req[1] {
                    MODE_WAITING => {
                        // CANopenNode `CO_LSSslave.c:56-66`: a node that was
                        // configured in this session (active id still
                        // unconfigured, pending id valid) and is switched
                        // back to Waiting must request an NMT communication
                        // reset so the pending id becomes active; the
                        // `service` flag is handled by `CO_LSSslave_process`
                        // (`:275-280`).
                        if self.state == LssState::Configuration
                            && self.node_id == UNCONFIGURED_NODE_ID
                            && self.pending_node_id != UNCONFIGURED_NODE_ID
                        {
                            self.reset_requested = true;
                        }
                        self.state = LssState::Waiting;
                        self.selective = 0;
                    }
                    MODE_CONFIGURATION => self.state = LssState::Configuration,
                    // docs/DESIGN-DECISIONS.md 十M21: an invalid mode is ignored entirely —
                    // CANopenNode `CO_LSSslave.c:68-70` neither changes the
                    // state nor clears the selective-switch progress.
                    _ => {}
                }
                None
            }
            CS_SWITCH_SELECTIVE_VENDOR if self.state == LssState::Waiting => {
                // Selective switch is a Waiting-state service (CiA 305 §3.3):
                // a node already in Configuration must not be re-captured by
                // a matching sequence (docs/DESIGN-DECISIONS.md H2; the serial step below
                // already had this guard).
                self.selective = u8::from(u32_at(req) == self.address.vendor_id);
                None
            }
            CS_SWITCH_SELECTIVE_PRODUCT if self.state == LssState::Waiting => {
                self.selective = if self.selective == 1 && u32_at(req) == self.address.product_code
                {
                    2
                } else {
                    0
                };
                None
            }
            CS_SWITCH_SELECTIVE_REVISION if self.state == LssState::Waiting => {
                self.selective =
                    if self.selective == 2 && u32_at(req) == self.address.revision_number {
                        3
                    } else {
                        0
                    };
                None
            }
            CS_SWITCH_SELECTIVE_SERIAL
                if self.state == LssState::Waiting || self.state == LssState::Configuration =>
            {
                let serial_matches = u32_at(req) == self.address.serial_number;
                if self.state == LssState::Configuration {
                    // docs/DESIGN-DECISIONS.md LSS-01: a node already in Configuration is not
                    // *re-captured* (state and progress stay untouched,
                    // R10-P2-8 / H2) but still answers the serial step when
                    // its identity matches. CANopenNode forwards the frame in
                    // Configuration state and answers `0x44` unconditionally
                    // (`CO_LSSslave.c:161-169,279-286` — it does not verify the
                    // identity there), while python's master sends the whole
                    // four-frame sequence and raises on timeout
                    // (`lss.py:134-141,392`): staying silent made a repeated
                    // `select` of an already-selected node hang forever.
                    serial_matches.then(|| frame(CS_SWITCH_SELECTIVE_RESPONSE))
                } else if self.selective == 3 && serial_matches {
                    self.selective = 0;
                    self.state = LssState::Configuration;
                    Some(frame(CS_SWITCH_SELECTIVE_RESPONSE))
                } else {
                    self.selective = 0;
                    None
                }
            }
            CS_CONFIGURE_NODE_ID if configuring => {
                let id = req[1];
                // CiA 305 §5.4.1: an assigned value of `0xFF` makes the
                // node-id *invalid* (the slave re-enters the LSS init state),
                // so it is accepted alongside 1..=127 — CANopenNode
                // `CO_LSS_NODE_ID_VALID` (`CO_LSS.h:205`) and the historical
                // canopen-rs baseline agree (docs/DESIGN-DECISIONS.md A2).
                let error = if (1..=127).contains(&id) || id == UNCONFIGURED_NODE_ID {
                    self.pending_node_id = id;
                    0
                } else {
                    1 // node-id out of range
                };
                let mut f = frame(CS_CONFIGURE_NODE_ID);
                f[1] = error;
                Some(f)
            }
            CS_STORE_CONFIGURATION if configuring => {
                // CANopenNode `CO_LSSslave.c:351-372`: error 1 when no store
                // callback is registered; with one, the callback returns
                // synchronously and its failure is reported as error 2.
                // This stack's store runs asynchronously (the application
                // drains `take_store_requested` and persists later), so only
                // the synchronous outcome (0/1) is answerable here — the
                // deferred failure has no reply channel (`docs/DESIGN-DECISIONS.md` L2).
                let error = if self.store_supported {
                    self.store_requested = true;
                    0
                } else {
                    1
                };
                let mut f = frame(CS_STORE_CONFIGURATION);
                f[1] = error;
                Some(f)
            }
            CS_CONFIGURE_BIT_TIMING if configuring => {
                // No bit-rate callback registered: the reference drops the
                // request silently (`CO_LSSslave.c:303-307`); answering
                // would let a master believe the rate was accepted
                // (docs/DESIGN-DECISIONS.md N12/L1).
                if !self.bit_timing_supported {
                    return None;
                }
                // Only table selector 0 (the CiA 305 default rate list) is
                // recognised; the index must be a valid CiA 301 rate entry
                // (0..=9, excluding the reserved 5) — CANopenNode
                // `CO_LSS_BIT_TIMING_VALID`. Bytes 3..6 are reserved and
                // deliberately ignored, never folded into pending state.
                let valid_index = |i: u8| i != 5 && i <= 9;
                let error = if req[1] == 0 && valid_index(req[2]) {
                    self.pending_bit_timing = Some((req[1], u32::from(req[2])));
                    0
                } else {
                    // 1 = out of range / not supported; CANopenNode
                    // `CO_LSS.h:110-112` defines 0/1/0xFF, `2` is undefined
                    // (docs/DESIGN-DECISIONS.md 十L7).
                    1
                };
                let mut f = frame(CS_CONFIGURE_BIT_TIMING);
                f[1] = error;
                Some(f)
            }
            CS_ACTIVATE_BIT_TIMING if configuring => {
                // The reference only forwards the switch delay when the
                // bit-rate callback exists (`CO_LSSslave.c:326-341`) and
                // this service is only reachable in the Configuration state
                // (`:163-167`); otherwise it is dropped silently
                // (docs/DESIGN-DECISIONS.md N12/L1).
                if !self.bit_timing_supported {
                    return None;
                }
                // CiA 305 §3.4: the 16-bit switch delay (ms, little-endian)
                // must reach the application — CANopenNode forwards it via
                // `pFunctLSSactivateBitRate` (`CO_LSSslave.c:337-349`).
                // Previously it was discarded.
                self.activate_bit_timing_delay = Some(u16::from_le_bytes([req[1], req[2]]));
                None // broadcast; applied by the app on restart
            }
            CS_INQUIRE_NODE_ID if configuring => {
                let mut f = frame(CS_INQUIRE_NODE_ID);
                f[1] = self.node_id;
                Some(f)
            }
            CS_INQUIRE_VENDOR if configuring => {
                Some(frame_u32(CS_INQUIRE_VENDOR, self.address.vendor_id))
            }
            CS_INQUIRE_PRODUCT if configuring => {
                Some(frame_u32(CS_INQUIRE_PRODUCT, self.address.product_code))
            }
            CS_INQUIRE_REVISION if configuring => {
                Some(frame_u32(CS_INQUIRE_REVISION, self.address.revision_number))
            }
            CS_INQUIRE_SERIAL if configuring => {
                Some(frame_u32(CS_INQUIRE_SERIAL, self.address.serial_number))
            }
            CS_FASTSCAN => self.handle_fastscan(req),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn address() -> LssAddress {
        LssAddress {
            vendor_id: 0x0000_001F,
            product_code: 0x0000_002A,
            revision_number: 0x0000_0001,
            serial_number: 0x0000_1234,
        }
    }

    fn slave() -> LssSlave {
        LssSlave::new(address(), UNCONFIGURED_NODE_ID)
    }

    #[test]
    fn golden_frames() {
        assert_eq!(encode_switch_global(true), [0x04, 0x01, 0, 0, 0, 0, 0, 0]);
        assert_eq!(
            encode_configure_node_id(0x33),
            [0x11, 0x33, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(encode_store(), [0x17, 0, 0, 0, 0, 0, 0, 0]);
        // CiA 305: activate bit timing is CS 0x15 with u16 LE switch delay
        // (python-canopen lss.py packs '<H'; CANopenNode uses CO_setUint16).
        assert_eq!(
            encode_activate_bit_timing(1000),
            [0x15, 0xE8, 0x03, 0, 0, 0, 0, 0]
        );
        let sel = encode_switch_selective(&address());
        assert_eq!(sel[0][0], 0x40);
        assert_eq!(&sel[0][1..5], &0x1Fu32.to_le_bytes());
        assert_eq!(sel[3][0], 0x43);
        assert_eq!(encode_fastscan(0xAB, 7, 2, 3)[5..8], [7, 2, 3]);
        let identify = encode_identify_remote_slave(0x1F, 0x20, 0x30, 0x31, 0x40, 0x41);
        assert_eq!(identify.map(|f| f[0]), [0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B]);
        assert_eq!(encode_identify_non_configured()[0], 0x4C);
    }

    #[test]
    fn lss14_retry_after_lost_sixth_frame_succeeds_immediately() {
        let address = address();
        let mut s = slave();
        let frames = encode_identify_remote_slave(
            address.vendor_id,
            address.product_code,
            address.revision_number,
            address.revision_number,
            address.serial_number,
            address.serial_number,
        );
        for f in &frames[..5] {
            assert!(s.handle(f).is_none());
        }
        let response = frames
            .iter()
            .find_map(|f| s.handle(f))
            .expect("0x46 restarts");
        assert!(decode_identify_slave_response(&response));
    }

    #[test]
    fn lss13_nonconfigured_requires_pending_unset() {
        let mut s = slave();
        assert!(s.handle(&encode_identify_non_configured()).is_some());
        let _ = s.handle(&encode_switch_global(true));
        let _ = s.handle(&encode_configure_node_id(9));
        assert!(s.handle(&encode_identify_non_configured()).is_none());
    }

    #[test]
    fn identify_remote_slave_is_strict_and_range_checked() {
        let address = address();
        let frames = encode_identify_remote_slave(
            address.vendor_id,
            address.product_code,
            address.revision_number,
            address.revision_number,
            address.serial_number,
            address.serial_number,
        );
        assert_eq!(frames[0][0], 0x46);
        assert_eq!(frames[5][0], 0x4B);

        let mut s = slave();
        for f in &frames[..5] {
            assert!(s.handle(f).is_none(), "partial identify must stay silent");
        }
        let response = s.handle(&frames[5]).expect("identity matches");
        assert_eq!(response[0], 0x4F);
        assert!(decode_identify_slave_response(&response));

        // A repeated intermediate frame resets the strict chain silently.
        let mut s = slave();
        assert!(s.handle(&frames[0]).is_none());
        assert!(s.handle(&frames[0]).is_none(), "repeat resets, no response");
        assert!(
            s.handle(&frames[5]).is_none(),
            "the chain no longer starts at step 0"
        );

        // Out-of-range revision and serial do not match.
        for bad in [
            encode_identify_remote_slave(
                address.vendor_id,
                address.product_code,
                0,
                0,
                address.serial_number,
                address.serial_number,
            ),
            encode_identify_remote_slave(
                address.vendor_id,
                address.product_code,
                address.revision_number,
                address.revision_number,
                0,
                0,
            ),
        ] {
            let mut s = slave();
            for f in &bad[..5] {
                let _ = s.handle(f);
            }
            assert!(s.handle(&bad[5]).is_none());
        }

        // 0x4C: only an unconfigured node answers 0x50.
        let mut s = slave();
        let r = s.handle(&encode_identify_non_configured()).unwrap();
        assert!(decode_identify_non_configured_response(&r));
        let mut configured = LssSlave::new(address, 5);
        assert!(configured
            .handle(&encode_identify_non_configured())
            .is_none());
    }

    #[test]
    fn selective_switch_and_configuration_services() {
        let mut s = slave();
        // Partial mismatch keeps the slave waiting and silent.
        let wrong = encode_switch_selective(&LssAddress {
            vendor_id: 9,
            ..address()
        });
        for f in wrong.iter().take(3) {
            assert!(s.handle(f).is_none());
        }
        assert!(s.handle(&wrong[3]).is_none());
        assert_eq!(s.state(), LssState::Waiting);

        // Wrong vendor leaves no trace; the full correct sequence selects
        // the node and answers exactly once (on the serial frame).
        let mut answered = false;
        for f in encode_switch_selective(&address()) {
            if let Some(resp) = s.handle(&f) {
                assert!(is_switch_selective_response(&resp));
                assert_eq!(f[0], CS_SWITCH_SELECTIVE_SERIAL);
                answered = true;
            }
        }
        assert!(answered);
        let mut s = slave();
        for f in encode_switch_selective(&address()) {
            let resp = s.handle(&f);
            let _ = resp;
        }
        assert_eq!(s.state(), LssState::Configuration);

        // Invalid id rejected with error code 1; valid ids become pending.
        let bad = s.handle(&encode_configure_node_id(200)).unwrap();
        assert_eq!(decode_configure_node_id_response(&bad), Some(Err(1)));
        assert!(s.node_id() == UNCONFIGURED_NODE_ID);

        assert_eq!(
            decode_configure_node_id_response(&s.handle(&encode_configure_node_id(0x21)).unwrap()),
            Some(Ok(()))
        );
        assert_eq!(s.pending_node_id(), 0x21);
        assert_eq!(s.node_id(), UNCONFIGURED_NODE_ID); // not active yet

        // Store + inquire reflect the still-active old id.
        assert!(s.handle(&encode_store()).is_some());
        let inquired = s.handle(&encode_inquire_node_id()).unwrap();
        assert_eq!(
            decode_inquire_node_id_response(&inquired),
            Some(UNCONFIGURED_NODE_ID)
        );

        // Adopting happens on reset: the node leaves the configuration
        // session, so services stop until it is selected again.
        s.adopt_pending();
        assert_eq!(s.node_id(), 0x21);
        assert!(s.handle(&encode_inquire_node_id()).is_none());
        for f in encode_switch_selective(&address()) {
            let _ = s.handle(&f);
        }
        let inquired = s.handle(&encode_inquire_node_id()).unwrap();
        assert_eq!(decode_inquire_node_id_response(&inquired), Some(0x21));
    }

    /// docs/DESIGN-DECISIONS.md H2: selective switch is a Waiting-state service (CiA 305
    /// §3.3) — a node already in Configuration must not be re-captured by a
    /// matching vendor/product/revision/serial sequence. Previously only the
    /// serial step guarded on `state == Waiting`.
    #[test]
    fn selective_sequence_ignored_when_not_waiting() {
        // Drive a slave fully into Configuration via a correct sequence.
        let mut s = slave();
        for f in encode_switch_selective(&address()) {
            let _ = s.handle(&f);
        }
        assert_eq!(s.state(), LssState::Configuration);

        // A matching sequence must NOT re-capture it: the vendor/product/
        // revision steps stay silent, and the serial step only *confirms* the
        // already-configured node (0x44) without touching state or progress —
        // docs/DESIGN-DECISIONS.md LSS-01: staying silent there made a repeated `select` from a
        // stateless master time out forever.
        let seq = encode_switch_selective(&address());
        for f in &seq[..3] {
            assert!(
                s.handle(f).is_none(),
                "a Configuration-state slave must ignore selective switch"
            );
        }
        let confirm = s
            .handle(&seq[3])
            .expect("the serial step still confirms the configured node");
        assert!(is_switch_selective_response(&confirm));
        assert_eq!(s.state(), LssState::Configuration);

        // A foreign serial number is still ignored entirely.
        let mut other = address();
        other.serial_number ^= 0xFFFF;
        assert!(s.handle(&encode_switch_selective(&other)[3]).is_none());
        // Configure-node-id still works afterwards (state intact).
        assert!(s.handle(&encode_configure_node_id(0x33)).is_some());
    }

    #[test]
    fn fastscan_discovers_the_address_end_to_end() {
        let address = address();
        let mut slave = slave();
        let mut master = FastscanMaster::new();

        while let Some(req) = master.next_request() {
            let answered = slave.handle(&req).is_some();
            master.on_response(answered);
        }
        assert!(master.found() && master.is_complete());
        assert_eq!(master.address(), address);
        // Discovery switched the slave into configuration.
        assert_eq!(slave.state(), LssState::Configuration);
    }

    #[test]
    fn fastscan_probe_timeout_retries_then_decides() {
        // docs/DESIGN-DECISIONS.md m14: the driver owns timing; `probe_timed_out` re-emits
        // while retries remain, then hands the silence to `on_response(false)`.
        let mut master = FastscanMaster::new();
        master.set_retries_per_probe(2);
        let first = master.next_request().unwrap();

        // Two lapsed windows → two identical re-transmissions.
        assert_eq!(master.probe_timed_out(), Some(first));
        assert_eq!(master.probe_timed_out(), Some(first));
        // Third lapse: silence is final.
        assert_eq!(master.probe_timed_out(), None);
        master.on_response(false); // nobody present → scan ends
        assert!(master.is_complete() && !master.found());
    }

    #[test]
    fn fastscan_initial_probe_retries_on_direct_timeout() {
        // docs/DESIGN-DECISIONS.md M2: a driver that feeds `on_response(false)` directly
        // (no `probe_timed_out` hook) must still honour `retries_per_probe`
        // on the initial probe — a lost first frame must not end the whole
        // scan while retries remain.
        let mut master = FastscanMaster::new();
        master.set_retries_per_probe(2);
        let first = master.next_request().unwrap();
        assert_eq!(first[5], FASTSCAN_INIT);

        // Two silent windows: the probe is re-emitted both times.
        master.on_response(false);
        assert!(!master.is_complete());
        assert_eq!(master.next_request().unwrap(), first);
        master.on_response(false);
        assert!(!master.is_complete());
        assert_eq!(master.next_request().unwrap(), first);

        // Budget exhausted: the third silence ends the scan without a find.
        master.on_response(false);
        assert!(master.is_complete() && !master.found());
    }

    #[test]
    fn fastscan_default_single_probe_matches_canopennode() {
        let mut master = FastscanMaster::new();
        let _ = master.next_request().unwrap();
        // Default retries = 0: one timed-out probe decides immediately.
        assert_eq!(master.probe_timed_out(), None);
    }

    #[test]
    fn fastscan_recovers_extreme_identities() {
        for addr in [
            LssAddress {
                vendor_id: u32::MAX,
                product_code: 0,
                revision_number: 0xFFFF_FFFF,
                serial_number: 0xA5A5_A5A5,
            },
            LssAddress {
                vendor_id: 0,
                product_code: 1,
                revision_number: 1 << 31,
                serial_number: 0x8000_0001,
            },
        ] {
            let mut slave = LssSlave::new(addr, UNCONFIGURED_NODE_ID);
            let mut master = FastscanMaster::new();
            while let Some(req) = master.next_request() {
                master.on_response(slave.handle(&req).is_some());
            }
            assert_eq!(master.address(), addr);
        }
    }

    #[test]
    fn configured_nodes_do_not_answer_fastscan() {
        let mut s = LssSlave::new(address(), 12); // already configured
        let probe = encode_fastscan(0, FASTSCAN_INIT, 0, 0);
        assert!(s.handle(&probe).is_none());
    }

    #[test]
    fn fastscan_initial_probe_matches_reference_0x80() {
        // docs/DESIGN-DECISIONS.md #3: the probe byte must be 0x80, matching CANopenNode's
        // `CO_LSS_FASTSCAN_CONFIRM` and python-canopen's initial probe of 128.
        assert_eq!(FASTSCAN_INIT, 0x80);
        let master = FastscanMaster::new();
        let first = master.next_request().unwrap();
        assert_eq!(first[5], 0x80);
        // The slave answers the reference probe; the legacy 32 is rejected
        // like any other out-of-range bit_check (`CO_LSSslave.c:30`).
        let mut s = slave();
        assert!(s.handle(&encode_fastscan(0, 0x80, 0, 0)).is_some());
        assert!(s.handle(&encode_fastscan(0, 32, 0, 0)).is_none());
        // Anything else >= 32 is silently ignored (CO_LSSslave.c:30,115).
        assert!(s.handle(&encode_fastscan(0, 33, 0, 0)).is_none());
        assert!(s.handle(&encode_fastscan(0, 0x81, 0, 0)).is_none());
    }

    #[test]
    fn fastscan_slave_ignores_out_of_range_confirmation() {
        // docs/DESIGN-DECISIONS.md #4: a malformed confirmation must not corrupt `fs_sub`.
        let mut s = slave();
        let mut f = encode_fastscan(0x1F, 0, 0, 200); // lss_next out of range
        assert!(s.handle(&f).is_none());
        assert_eq!(s.state(), LssState::Waiting);
        f = encode_fastscan(0x1F, 0, 200, 1); // lss_sub out of range
        assert!(s.handle(&f).is_none());
        assert_eq!(s.state(), LssState::Waiting);
        // The node still answers a well-formed confirmation afterwards.
        assert!(s.handle(&encode_fastscan(0x1F, 0, 0, 1)).is_some());
    }

    #[test]
    fn fastscan_completion_resets_position_like_reference() {
        // docs/DESIGN-DECISIONS.md L4: CANopenNode acks with `fastscanPos = lssNext` before
        // the completion check (`CO_LSSslave.c:136`), so a completed scan
        // leaves position 0 (VENDOR_ID), not the serial index 3.
        let mut s = slave();
        let mut master = FastscanMaster::new();
        while let Some(req) = master.next_request() {
            master.on_response(s.handle(&req).is_some());
        }
        assert_eq!(master.address(), address());
        assert_eq!(s.state(), LssState::Configuration);
        assert_eq!(s.fs_sub, 0, "completed scan must reset the scan position");

        // A non-conformant master that skips the 0x80 restart after leaving
        // configuration continues at sub 0 and must still be answered.
        s.handle(&encode_switch_global(false));
        assert_eq!(s.state(), LssState::Waiting);
        let probe = encode_fastscan(address().vendor_id, 31, 0, 0);
        assert!(
            s.handle(&probe).is_some(),
            "continuation probe at sub 0 must be answered"
        );
    }

    /// docs/DESIGN-DECISIONS.md 七F6: only an "identify slave" response counts as an answer.
    /// The reference master checks the CS (CANopenNode
    /// `CO_LSSmaster.c:734-741`); `on_response_frame` applies that check.
    #[test]
    fn fastscan_response_frame_checks_the_command_specifier() {
        let mut master = FastscanMaster::new();
        let _ = master.next_request().unwrap();
        // A foreign response frame on 0x7E4 must not confirm a node.
        master.on_response_frame(&frame(CS_SWITCH_SELECTIVE_RESPONSE));
        assert!(!master.found(), "wrong CS must not confirm a node");
        assert!(master.is_complete());

        let mut master = FastscanMaster::new();
        let _ = master.next_request().unwrap();
        master.on_response_frame(&frame(CS_IDENTIFY_SLAVE));
        assert!(master.found(), "identify-slave frame confirms the probe");

        // The all-nodes scanner filters the same way.
        let mut scanner = AllNodesScanner::new(1);
        let _ = scanner.next_action().unwrap();
        scanner.on_response_frame(&frame(CS_INQUIRE_NODE_ID));
        assert!(scanner.discovered().is_empty());
        assert!(scanner.is_complete());
    }

    /// docs/DESIGN-DECISIONS.md 七L5: master-side inquire vendor/product/revision/serial
    /// services (slave side 0x5A-0x5D existed already).
    #[test]
    fn inquire_address_services_roundtrip() {
        let mut s = slave();
        s.handle(&encode_switch_global(true));
        let vendor = s.handle(&encode_inquire_vendor_id()).unwrap();
        assert_eq!(
            decode_inquire_vendor_id_response(&vendor),
            Some(address().vendor_id)
        );
        let product = s.handle(&encode_inquire_product_code()).unwrap();
        assert_eq!(
            decode_inquire_product_code_response(&product),
            Some(address().product_code)
        );
        let revision = s.handle(&encode_inquire_revision_number()).unwrap();
        assert_eq!(
            decode_inquire_revision_number_response(&revision),
            Some(address().revision_number)
        );
        let serial = s.handle(&encode_inquire_serial_number()).unwrap();
        assert_eq!(
            decode_inquire_serial_number_response(&serial),
            Some(address().serial_number)
        );
        // Wrong CS decodes to None.
        assert_eq!(decode_inquire_vendor_id_response(&product), None);
    }

    #[test]
    fn allnodes_scanner_enumerates_multiple_nodes() {
        // Simulation bus: every frame on 0x7E5 is delivered to both slaves;
        // the bus "answers" if any of them produced a response frame.
        struct Bus(Vec<LssSlave>);
        impl Bus {
            fn transmit(&mut self, frame: &LssFrame) -> bool {
                self.0.iter_mut().any(|s| s.handle(frame).is_some())
            }
        }

        let mut bus = Bus(vec![
            LssSlave::new(
                LssAddress {
                    vendor_id: 1,
                    product_code: 1,
                    revision_number: 1,
                    serial_number: 0xAA,
                },
                UNCONFIGURED_NODE_ID,
            ),
            LssSlave::new(
                LssAddress {
                    vendor_id: 1,
                    product_code: 1,
                    revision_number: 1,
                    serial_number: 0xBB,
                },
                UNCONFIGURED_NODE_ID,
            ),
        ]);

        let mut scanner = AllNodesScanner::new(127);
        let mut discovered = Vec::new();
        let mut assigned_ids = Vec::new();

        loop {
            match scanner.next_action() {
                Some(ScanAction::Send(frame, _expect_answer)) => {
                    let answered = bus.transmit(&frame);
                    scanner.on_response(answered);
                }
                Some(ScanAction::AssignTempId(id)) => {
                    // Only the node currently in the configuration state
                    // reacts to configure/store — exactly like real LSS.
                    assert!(bus.transmit(&encode_configure_node_id(id)));
                    assert!(bus.transmit(&encode_store()));
                    // docs/DESIGN-DECISIONS.md LSS-11: storing is *not* enough — the node is
                    // still unconfigured here. It leaves the pool only when
                    // the master switches it back to Waiting, which makes it
                    // request the communication reset carrying the new id.
                    for s in &mut bus.0 {
                        if s.state() == LssState::Configuration && s.pending_node_id() == id {
                            assert!(
                                !s.take_reset_requested(),
                                "the id must not be active before the deselect"
                            );
                        }
                    }
                    discovered.push(*scanner.discovered().last().expect("just found"));
                    assert!(scanner.on_assigned());
                }
                Some(ScanAction::Deselect) => {
                    // The reference master's `CO_LSSmaster_swStateDeselect`
                    // (`CO_gateway_ascii.c:1879-1886`): a non-confirmed
                    // switch-global(waiting) after every store.
                    let answered = bus.transmit(&encode_switch_global(false));
                    assert!(!answered, "deselect is not confirmed");
                    for s in &mut bus.0 {
                        // Only a node that is *still* unconfigured has work to
                        // do here; an already-configured slave ignores the
                        // deselect (`CO_LSSslave.c:56-66` guards on the
                        // unconfigured id) and must not be re-adopted.
                        if s.node_id() != UNCONFIGURED_NODE_ID
                            || s.pending_node_id() == UNCONFIGURED_NODE_ID
                        {
                            continue;
                        }
                        assert!(
                            s.take_reset_requested(),
                            "a configured node switched to Waiting must request a reset"
                        );
                        let id = s.pending_node_id();
                        s.adopt_pending();
                        assigned_ids.push((s.address(), id));
                    }
                    if !scanner.on_deselected() {
                        break;
                    }
                }
                None => break,
            }
        }

        // Both nodes discovered with distinct addresses and ids.
        assert_eq!(assigned_ids.len(), 2);
        assert_ne!(assigned_ids[0], assigned_ids[1]);
        let mut serials: Vec<u32> = discovered.iter().map(|a| a.serial_number).collect();
        serials.sort_unstable();
        assert_eq!(serials, vec![0xAA, 0xBB]);
        // Both slaves now hold their assigned ids.
        let mut node_ids: Vec<u8> = bus.0.iter().map(|s| s.node_id()).collect();
        node_ids.sort_unstable();
        assert_eq!(node_ids, vec![1, 2]);
    }

    /// docs/DESIGN-DECISIONS.md LSS-02: the identity has two sources (the `LssAddress` handed to
    /// the node and the OD's 0x1018); `from_od` lets the application derive
    /// one from the other so selective switch and SDO reads cannot disagree.
    #[test]
    fn lss_address_can_be_derived_from_the_identity_object() {
        use crate::object_dictionary::ObjectDictionary;
        use crate::standard::StandardObjects;

        let od = StandardObjects::new()
            .vendor_id(0x1234)
            .product_code(0x5678)
            .revision_number(0x0001_0002)
            .serial_number(0xCAFE_0001)
            .build();
        assert_eq!(
            LssAddress::from_od(&od),
            Some(LssAddress {
                vendor_id: 0x1234,
                product_code: 0x5678,
                revision_number: 0x0001_0002,
                serial_number: 0xCAFE_0001,
            })
        );
        // A dictionary without a full 0x1018 yields `None` rather than zeros
        // that would silently match a different node.
        assert_eq!(LssAddress::from_od(&ObjectDictionary::new()), None);
    }

    /// docs/DESIGN-DECISIONS.md LSS-12: the temp-id counter saturates instead of panicking
    /// (debug) or wrapping to 0 (release, which would hand out node-id 0) when
    /// a driver keeps driving the scan past its `false` return.
    #[test]
    fn allnodes_scanner_id_counter_saturates_and_stops() {
        let mut scanner = AllNodesScanner::new(255);
        let address = LssAddress {
            vendor_id: 1,
            product_code: 1,
            revision_number: 1,
            serial_number: 1,
        };
        let mut handed_out = Vec::new();
        // Drive far more rounds than the 127-id pool can serve, entering the
        // assignment phase the way a completed fastscan round would.
        for _ in 0..300 {
            scanner.discovered.push(address);
            scanner.assigning = true;
            let action = scanner.next_action();
            let Some(ScanAction::AssignTempId(id)) = action else {
                // The pool is exhausted: no id is ever handed out again, no
                // matter how often the driver reports another find.
                assert_eq!(action, None);
                assert!(id_counter_stays_exhausted(&scanner));
                break;
            };
            handed_out.push(id);
            assert!(scanner.on_assigned());
            assert_eq!(scanner.next_action(), Some(ScanAction::Deselect));
            let _ = scanner.on_deselected();
        }
        // Exactly the 127 valid ids, ascending, no 0 and no repeat.
        assert_eq!(handed_out.len(), 127);
        assert_eq!(handed_out[0], 1);
        assert_eq!(*handed_out.last().expect("non-empty"), 127);
        assert!(handed_out.windows(2).all(|w| w[1] == w[0] + 1));
        // The scan stopped right after the last valid id.
        assert_eq!(scanner.next_id, 128);
    }

    /// A driver that ignores the `false` return of [`AllNodesScanner::
    /// on_deselected`] must never be handed an out-of-range id again.
    fn id_counter_stays_exhausted(scanner: &AllNodesScanner) -> bool {
        let mut probe = AllNodesScanner::new(255);
        probe.next_id = scanner.next_id;
        for _ in 0..200 {
            probe.assigning = true;
            if probe.next_action().is_some() {
                return false;
            }
            probe.on_assigned();
            assert!(!probe.on_deselected());
        }
        // 200 further rounds pushed the counter to the saturation point — it
        // did not wrap to 0, so node-id 0 was never handed out.
        assert_eq!(probe.next_id, 255);
        true
    }

    #[test]
    fn bit_timing_configure_and_activate() {
        let mut s = slave();
        // Only works in configuration state.
        s.handle(&encode_switch_global(true));

        let resp = s.handle(&encode_configure_bit_timing(0, 6)).unwrap();
        assert_eq!(decode_configure_bit_timing_response(&resp), Some(Ok(())));
        assert_eq!(s.take_pending_bit_timing(), Some((0, 6)));

        // Unsupported table selector -> error code 1 (out of range;
        // docs/DESIGN-DECISIONS.md 十L7 — 2 was undefined).
        let resp = s.handle(&encode_configure_bit_timing(3, 1)).unwrap();
        assert_eq!(decode_configure_bit_timing_response(&resp), Some(Err(1)));
        assert_eq!(s.take_pending_bit_timing(), None);

        // Activation frame is broadcast-silent but forwards the delay.
        assert!(s.handle(&encode_activate_bit_timing(100)).is_none());
        assert_eq!(s.take_activate_bit_timing_delay(), Some(100));
    }

    #[test]
    fn bit_timing_services_are_dropped_without_application_support() {
        // docs/DESIGN-DECISIONS.md N12/L1: CANopenNode drops configure/activate bit timing
        // entirely when no `pFunctLSScheckBitRate` is registered
        // (`CO_LSSslave.c:303-341`) — no confirmation and no forwarded
        // delay.
        let mut s = slave();
        s.handle(&encode_switch_global(true));
        s.set_bit_timing_supported(false);
        assert!(s.handle(&encode_configure_bit_timing(0, 6)).is_none());
        assert_eq!(s.take_pending_bit_timing(), None);
        assert!(s.handle(&encode_activate_bit_timing(100)).is_none());
        assert_eq!(s.take_activate_bit_timing_delay(), None);
    }

    #[test]
    fn activate_bit_timing_is_configuration_state_only() {
        // docs/DESIGN-DECISIONS.md N12/L1: the reference reaches `CO_LSS_CFG_ACTIVATE_BIT_TIMING`
        // only in the Configuration state (`CO_LSSslave.c:163-167`), so a
        // Waiting slave must not latch the switch delay.
        let mut s = slave();
        assert!(s.handle(&encode_activate_bit_timing(250)).is_none());
        assert_eq!(s.take_activate_bit_timing_delay(), None);
        // Configure too: a Waiting slave must not answer.
        assert!(s.handle(&encode_configure_bit_timing(0, 6)).is_none());
        s.handle(&encode_switch_global(true));
        assert!(s.handle(&encode_activate_bit_timing(250)).is_none());
        assert_eq!(s.take_activate_bit_timing_delay(), Some(250));
    }

    #[test]
    fn bit_timing_index_validation_matches_canopennode() {
        // docs/DESIGN-DECISIONS.md C5 (CO_LSS_BIT_TIMING_VALID): index must be 0..=9 minus
        // the reserved 5; reserved bytes 3..6 are ignored entirely.
        let mut s = slave();
        s.handle(&encode_switch_global(true));

        // Reserved index 5 rejected (error code 1 = out of range, 十L7).
        let resp = s.handle(&encode_configure_bit_timing(0, 5)).unwrap();
        assert_eq!(decode_configure_bit_timing_response(&resp), Some(Err(1)));
        // Index 9 accepted.
        let resp = s.handle(&encode_configure_bit_timing(0, 9)).unwrap();
        assert_eq!(decode_configure_bit_timing_response(&resp), Some(Ok(())));
        assert_eq!(s.take_pending_bit_timing(), Some((0, 9)));
        // Out-of-range index rejected.
        let resp = s.handle(&encode_configure_bit_timing(0, 10)).unwrap();
        assert_eq!(decode_configure_bit_timing_response(&resp), Some(Err(1)));
    }

    #[test]
    fn fastscan_initial_probe_validates_sub_and_next() {
        // docs/DESIGN-DECISIONS.md 十L6: the confirm-bit probe with an out-of-range
        // lss_sub/lss_next must be ignored — CANopenNode validates all three
        // fields before the confirm branch (`CO_LSSslave.c:113-121`); the
        // old code reset the scan for any 0x80/32 probe.
        let mut s = slave();
        assert!(s
            .handle(&encode_fastscan(0, FASTSCAN_INIT, 200, 99))
            .is_none());
        // bit_check 32 is out of range (`CO_LSSslave.c:30`) and rejected before the
        // field checks; a *valid* bit_check with a bad lss_next too.
        assert!(s.handle(&encode_fastscan(0, 32, 0, 99)).is_none());
        assert!(s.handle(&encode_fastscan(0, 0, 0, 99)).is_none());
        // A well-formed probe still restarts the scan.
        assert!(s.handle(&encode_fastscan(0, FASTSCAN_INIT, 0, 0)).is_some());
    }

    #[test]
    fn bit_timing_reserved_bytes_are_ignored() {
        // docs/DESIGN-DECISIONS.md C5: `req=[0,0,9,1,0,0]` used to stash 257 (index folded
        // with reserved byte); now only req[2] matters.
        let mut s = slave();
        s.handle(&encode_switch_global(true));
        let mut req = encode_configure_bit_timing(0, 9);
        req[3] = 1; // garbage in the reserved area
        req[4] = 0xFF;
        let resp = s.handle(&req).unwrap();
        assert_eq!(decode_configure_bit_timing_response(&resp), Some(Ok(())));
        assert_eq!(s.take_pending_bit_timing(), Some((0, 9)));
    }

    #[test]
    fn m6_configure_bit_timing_encodes_single_byte() {
        // docs/DESIGN-DECISIONS.md M6：旧实现把整个 `u32` 写入 `f[2..6]`，而从站只读 `req[2]`，
        // 编码/解码长度不一致、调用方越界传参被静默折叠。修复后 `table_entry`
        // 仅占单字节 `f[2]`，`f[3..6]` 保留为零。
        let f = encode_configure_bit_timing(0, 9);
        assert_eq!(f[2], 9);
        assert_eq!(&f[3..6], &[0, 0, 0]);

        // 调用方传入 >255 的值被截断到单字节（属越界误用，但不再折叠到高位）。
        let g = encode_configure_bit_timing(0, 0x1234u32);
        assert_eq!(g[2], 0x34);
        assert_eq!(&g[3..6], &[0, 0, 0]);
    }

    #[test]
    fn switch_global_moves_states() {
        let mut s = slave();
        s.handle(&encode_switch_global(true));
        assert_eq!(s.state(), LssState::Configuration);
        s.handle(&encode_switch_global(false));
        assert_eq!(s.state(), LssState::Waiting);
    }

    /// docs/DESIGN-DECISIONS.md 十M21: an invalid mode must not disturb the state machine
    /// (CANopenNode `CO_LSSslave.c:68-70` default: no change at all).
    #[test]
    fn switch_global_ignores_invalid_mode() {
        let mut s = slave();
        // Partial selective-switch progress before the invalid frame.
        let _ = s.handle(&frame_u32(CS_SWITCH_SELECTIVE_VENDOR, address().vendor_id));
        let mut bad = encode_switch_global(true);
        bad[1] = 7;
        assert!(s.handle(&bad).is_none());
        assert_eq!(s.state(), LssState::Waiting);
        // The selective progress survived: completing the sequence works.
        let seq = encode_switch_selective(&address());
        for f in &seq[1..] {
            let _ = s.handle(f);
        }
        assert_eq!(s.state(), LssState::Configuration);
    }

    /// docs/DESIGN-DECISIONS.md 十M22: a freshly configured node switched back to Waiting
    /// requests an NMT communication reset (CANopenNode
    /// `CO_LSSslave.c:56-66,275-280`).
    #[test]
    fn switch_global_to_waiting_requests_reset_after_configuration() {
        let mut s = LssSlave::new(address(), UNCONFIGURED_NODE_ID);
        s.handle(&encode_switch_global(true));
        let _ = s.handle(&encode_configure_node_id(0x21));
        assert!(!s.take_reset_requested(), "no request before the switch");
        s.handle(&encode_switch_global(false));
        assert!(s.take_reset_requested());
        assert!(!s.take_reset_requested(), "request is one-shot");
    }

    /// docs/DESIGN-DECISIONS.md 十M23: *store configuration* answers error 1 when the
    /// application provides no store callback (CANopenNode
    /// `CO_LSSslave.c:351-372`) instead of always claiming success.
    #[test]
    fn store_reports_not_supported() {
        let mut s = slave();
        s.handle(&encode_switch_global(true));
        s.set_store_supported(false);
        let resp = s.handle(&encode_store()).unwrap();
        assert_eq!(resp[0], CS_STORE_CONFIGURATION);
        assert_eq!(resp[1], 1);
        assert!(!s.take_store_requested());
        // Supported again → error 0 and the request is queued.
        s.set_store_supported(true);
        let resp = s.handle(&encode_store()).unwrap();
        assert_eq!(resp[1], 0);
        assert!(s.take_store_requested());
    }

    /// docs/DESIGN-DECISIONS.md 十M23: the *activate bit timing* switch delay must reach the
    /// application (`CO_LSSslave.c:337-349`) instead of being discarded.
    #[test]
    fn activate_bit_timing_delay_is_reported() {
        let mut s = slave();
        s.handle(&encode_switch_global(true));
        assert!(s.handle(&encode_activate_bit_timing(1500)).is_none());
        assert_eq!(s.take_activate_bit_timing_delay(), Some(1500));
        assert_eq!(s.take_activate_bit_timing_delay(), None);
    }

    /// docs/DESIGN-DECISIONS.md A2: Configure Node-ID accepts `0xFF`, which invalidates the
    /// node-id (CiA 305 §5.4.1, CANopenNode `CO_LSS_NODE_ID_VALID`).
    #[test]
    fn configure_node_id_accepts_unconfigured_sentinel() {
        let mut s = slave();
        for f in encode_switch_selective(&address()) {
            let _ = s.handle(&f);
        }
        assert_eq!(s.state(), LssState::Configuration);

        let resp = s
            .handle(&encode_configure_node_id(UNCONFIGURED_NODE_ID))
            .unwrap();
        assert_eq!(decode_configure_node_id_response(&resp), Some(Ok(())));
        assert_eq!(s.pending_node_id(), UNCONFIGURED_NODE_ID);
        // Values outside 1..=127 ∪ {0xFF} are still rejected.
        let resp = s.handle(&encode_configure_node_id(200)).unwrap();
        assert_eq!(decode_configure_node_id_response(&resp), Some(Err(1)));
    }

    /// docs/DESIGN-DECISIONS.md C4: a node whose id was configured but not yet adopted is
    /// treated as configured and stays silent during fastscan
    /// (`CO_LSSslave.c:104-107` checks both pending and active ids).
    #[test]
    fn pending_but_unadopted_node_does_not_answer_fastscan() {
        let mut s = slave();
        for f in encode_switch_selective(&address()) {
            let _ = s.handle(&f);
        }
        let _ = s.handle(&encode_configure_node_id(0x21));
        assert_eq!(s.pending_node_id(), 0x21);
        // Back to Waiting by the reset that precedes adoption (state reset,
        // pending kept) — the situation CANopenNode guards against.
        s.reset();
        assert_eq!(s.state(), LssState::Waiting);
        assert!(s.handle(&encode_fastscan(0, FASTSCAN_INIT, 0, 0)).is_none());
        // After adoption the node is configured and still silent.
        s.adopt_pending();
        assert!(s.handle(&encode_fastscan(0, FASTSCAN_INIT, 0, 0)).is_none());
    }

    /// docs/DESIGN-DECISIONS.md B7: losing the confirmation frame makes the whole scan fail
    /// (CANopenNode `FsVerifyWait` → `SCAN_NOACK`; python returns
    /// `(False, None)`) instead of trusting an unverified address.
    #[test]
    fn fastscan_lost_confirmation_fails_scan() {
        let mut slave = slave();
        let mut master = FastscanMaster::new();
        while let Some(req) = master.next_request() {
            // A confirmation frame has bit_check 0 and lss_next != lss_sub.
            let is_confirm = req[5] == 0 && req[7] != req[6];
            let answered = !is_confirm && slave.handle(&req).is_some();
            master.on_response(answered);
        }
        assert!(master.is_complete());
        assert!(!master.found(), "unverified address must not be reported");
    }
}
