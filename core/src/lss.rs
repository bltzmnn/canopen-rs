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
pub const FASTSCAN_INIT: u8 = 32;

const CS_SWITCH_GLOBAL: u8 = 0x04;
const CS_CONFIGURE_NODE_ID: u8 = 0x11;
pub(super) const CS_CONFIGURE_BIT_TIMING: u8 = 0x13;
pub(super) const CS_ACTIVATE_BIT_TIMING: u8 = 0x23;
const CS_STORE_CONFIGURATION: u8 = 0x17;
const CS_SWITCH_SELECTIVE_VENDOR: u8 = 0x40;
const CS_SWITCH_SELECTIVE_PRODUCT: u8 = 0x41;
const CS_SWITCH_SELECTIVE_REVISION: u8 = 0x42;
const CS_SWITCH_SELECTIVE_SERIAL: u8 = 0x43;
const CS_SWITCH_SELECTIVE_RESPONSE: u8 = 0x44;
const CS_INQUIRE_VENDOR: u8 = 0x5A;
const CS_INQUIRE_PRODUCT: u8 = 0x5B;
const CS_INQUIRE_REVISION: u8 = 0x5C;
const CS_INQUIRE_SERIAL: u8 = 0x5D;
const CS_INQUIRE_NODE_ID: u8 = 0x5E;
const CS_IDENTIFY_SLAVE: u8 = 0x4F;
const CS_FASTSCAN: u8 = 0x51;

const MODE_WAITING: u8 = 0;
const MODE_CONFIGURATION: u8 = 1;

/// The 128-bit device identity LSS matches against (objects 0x1018:1..4).
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
/// into that table's rate list (0=1 Mbit/s … 8=10 kbit/s per CiA 305 §3.4).
#[must_use]
pub fn encode_configure_bit_timing(timing_selector: u8, table_entry: u32) -> LssFrame {
    let mut f = frame(CS_CONFIGURE_BIT_TIMING);
    f[1] = timing_selector;
    f[2..6].copy_from_slice(&table_entry.to_le_bytes());
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
#[must_use]
pub fn encode_activate_bit_timing(delay_ms: u8) -> LssFrame {
    let mut f = frame(CS_ACTIVATE_BIT_TIMING);
    f[1] = delay_ms;
    f
}

/// Inquire the configured node's active node-id.
#[must_use]
pub fn encode_inquire_node_id() -> LssFrame {
    frame(CS_INQUIRE_NODE_ID)
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
        }
    }

    /// Configures how many times each probe is re-sent after a lapsed
    /// response window ([`Self::probe_timed_out`]) before the silence is
    /// treated as an answer. Default 0 (single probe, CANopenNode style).
    pub fn set_retries_per_probe(&mut self, retries: u8) {
        self.retries_per_probe = retries;
        self.retries_left = retries;
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
        if self.retries_left > 0 {
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
    pub fn on_response(&mut self, responded: bool) {
        self.retries_left = self.retries_per_probe;
        match self.bit {
            -2 => {
                if responded {
                    self.found = true;
                    self.bit = 31;
                } else {
                    self.sub = 4; // nobody answered the probe
                }
            }
            -1 => {
                self.sub += 1; // sub confirmed; continue with the next value
                self.bit = 31;
            }
            b => {
                // No answer ⇒ this bit of the identity is a 1.
                if !responded {
                    self.id[usize::from(self.sub)] |= 1u32 << b;
                }
                self.bit -= 1;
            }
        }
    }

    /// Whether an unconfigured node answered the initial probe.
    #[must_use]
    pub const fn found(&self) -> bool {
        self.found
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
///     encode_configure_node_id, encode_store, AllNodesScanner, LssAddress,
///     LssSlave, ScanAction, UNCONFIGURED_NODE_ID,
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
///             // …and adopts it on the reset that ends its session.
///             slave.adopt_pending();
///             discovered.push(*scanner.discovered().last().unwrap());
///             if !scanner.on_assigned() {
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
    max_nodes: u8,
}

/// One step of the all-nodes discovery loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanAction {
    /// Transmit this frame on `0x7E5`. When `expect_answer` is set, wait for
    /// any response frame on `0x7E4` within the configured timeout and feed
    /// the boolean outcome into [`AllNodesScanner::on_response`].
    Send(LssFrame, bool),
    /// A node was just discovered and selected; send this configure-node-id
    /// request followed by [`encode_store`], then call
    /// [`AllNodesScanner::on_assigned`].
    AssignTempId(u8),
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
            max_nodes,
        }
    }

    /// Next driver step or `None` when complete.
    #[must_use]
    pub fn next_action(&self) -> Option<ScanAction> {
        if self.assigning {
            return Some(ScanAction::AssignTempId(self.next_id));
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
        self.fastscan.on_response(responded);
        if self.fastscan.found() && self.fastscan.is_complete() {
            self.discovered.push(self.fastscan.address());
            self.assigning = true;
        }
    }

    /// Confirms the temp-id assignment completed (configure + store acked);
    /// scanning continues for further nodes. Returns false when ids ran out.
    pub fn on_assigned(&mut self) -> bool {
        self.assigning = false;
        self.next_id += 1;
        self.fastscan = FastscanMaster::new();
        self.next_id <= 127
    }

    /// Addresses discovered so far, in assignment order.
    #[must_use]
    pub fn discovered(&self) -> &[LssAddress] {
        &self.discovered
    }

    /// Whether no further actions remain.
    #[must_use]
    pub const fn is_complete(&self) -> bool {
        !self.assigning && self.fastscan.is_complete()
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
    state: LssState,
    selective: u8,
    fs_sub: u8,
}

impl LssSlave {
    /// A slave with `address` and current raw node-id
    /// ([`UNCONFIGURED_NODE_ID`] when none assigned yet).
    #[must_use]
    pub const fn new(address: LssAddress, node_id: u8) -> Self {
        Self {
            address,
            node_id,
            pending_node_id: node_id,
            pending_bit_timing: None,
            state: LssState::Waiting,
            selective: 0,
            fs_sub: 0,
        }
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
        // Only unconfigured waiting nodes participate in discovery.
        if self.node_id != UNCONFIGURED_NODE_ID || self.state != LssState::Waiting {
            return None;
        }
        let id_number = u32_at(req);
        let bit_check = req[5];
        let lss_sub = req[6];
        let lss_next = req[7];

        if bit_check >= FASTSCAN_INIT {
            // Initial probe: restart and announce presence.
            self.fs_sub = 0;
            return Some(frame(CS_IDENTIFY_SLAVE));
        }
        if lss_sub != self.fs_sub {
            return None;
        }
        // Match only the bits the master has already determined
        // ([bit_check..=31]: check 31 → one bit, check 0 → whole value).
        let mask = u32::MAX << (bit_check & 31);
        if (id_number ^ self.identity(lss_sub)) & mask != 0 {
            return None;
        }
        // A full match with changing lss_next confirms this sub.
        if bit_check == 0 && lss_next != lss_sub {
            if lss_sub >= 3 {
                self.state = LssState::Configuration;
            } else {
                self.fs_sub = lss_next;
            }
        }
        Some(frame(CS_IDENTIFY_SLAVE))
    }

    /// Processes one master frame, returning the response to transmit.
    pub fn handle(&mut self, req: &LssFrame) -> Option<LssFrame> {
        let configuring = self.state == LssState::Configuration;
        match req[0] {
            CS_SWITCH_GLOBAL => {
                self.selective = 0;
                self.state = if req[1] == MODE_CONFIGURATION {
                    LssState::Configuration
                } else {
                    LssState::Waiting
                };
                None
            }
            CS_SWITCH_SELECTIVE_VENDOR => {
                self.selective = u8::from(u32_at(req) == self.address.vendor_id);
                None
            }
            CS_SWITCH_SELECTIVE_PRODUCT => {
                self.selective = if self.selective == 1 && u32_at(req) == self.address.product_code
                {
                    2
                } else {
                    0
                };
                None
            }
            CS_SWITCH_SELECTIVE_REVISION => {
                self.selective =
                    if self.selective == 2 && u32_at(req) == self.address.revision_number {
                        3
                    } else {
                        0
                    };
                None
            }
            CS_SWITCH_SELECTIVE_SERIAL => {
                if self.selective == 3 && u32_at(req) == self.address.serial_number {
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
            CS_STORE_CONFIGURATION if configuring => Some(frame(CS_STORE_CONFIGURATION)),
            CS_CONFIGURE_BIT_TIMING if configuring => {
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
                    2 // unsupported bit-timing table / entry
                };
                let mut f = frame(CS_CONFIGURE_BIT_TIMING);
                f[1] = error;
                Some(f)
            }
            CS_ACTIVATE_BIT_TIMING => None, // broadcast; applied by the app on restart
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
        let sel = encode_switch_selective(&address());
        assert_eq!(sel[0][0], 0x40);
        assert_eq!(&sel[0][1..5], &0x1Fu32.to_le_bytes());
        assert_eq!(sel[3][0], 0x43);
        assert_eq!(encode_fastscan(0xAB, 7, 2, 3)[5..8], [7, 2, 3]);
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
        // BUGS.md m14: the driver owns timing; `probe_timed_out` re-emits
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
                    // The application applies the pending id on reset; the
                    // node then leaves the unconfigured pool.
                    for s in &mut bus.0 {
                        if s.state() == LssState::Configuration && s.pending_node_id() == id {
                            s.adopt_pending();
                            assigned_ids.push((s.address(), id));
                        }
                    }
                    discovered.push(*scanner.discovered().last().expect("just found"));
                    assert!(scanner.on_assigned(), "temp ids exhausted");
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

    #[test]
    fn bit_timing_configure_and_activate() {
        let mut s = slave();
        // Only works in configuration state.
        s.handle(&encode_switch_global(true));

        let resp = s.handle(&encode_configure_bit_timing(0, 6)).unwrap();
        assert_eq!(decode_configure_bit_timing_response(&resp), Some(Ok(())));
        assert_eq!(s.take_pending_bit_timing(), Some((0, 6)));

        // Unsupported table selector -> error code 2.
        let resp = s.handle(&encode_configure_bit_timing(3, 1)).unwrap();
        assert_eq!(decode_configure_bit_timing_response(&resp), Some(Err(2)));
        assert_eq!(s.take_pending_bit_timing(), None);

        // Activation frame is broadcast-silent.
        assert!(s.handle(&encode_activate_bit_timing(100)).is_none());
    }

    #[test]
    fn bit_timing_index_validation_matches_canopennode() {
        // BUGS.md C5 (CO_LSS_BIT_TIMING_VALID): index must be 0..=9 minus
        // the reserved 5; reserved bytes 3..6 are ignored entirely.
        let mut s = slave();
        s.handle(&encode_switch_global(true));

        // Reserved index 5 rejected.
        let resp = s.handle(&encode_configure_bit_timing(0, 5)).unwrap();
        assert_eq!(decode_configure_bit_timing_response(&resp), Some(Err(2)));
        // Index 9 accepted.
        let resp = s.handle(&encode_configure_bit_timing(0, 9)).unwrap();
        assert_eq!(decode_configure_bit_timing_response(&resp), Some(Ok(())));
        assert_eq!(s.take_pending_bit_timing(), Some((0, 9)));
        // Out-of-range index rejected.
        let resp = s.handle(&encode_configure_bit_timing(0, 10)).unwrap();
        assert_eq!(decode_configure_bit_timing_response(&resp), Some(Err(2)));
    }

    #[test]
    fn bit_timing_reserved_bytes_are_ignored() {
        // BUGS.md C5: `req=[0,0,9,1,0,0]` used to stash 257 (index folded
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
    fn switch_global_moves_states() {
        let mut s = slave();
        s.handle(&encode_switch_global(true));
        assert_eq!(s.state(), LssState::Configuration);
        s.handle(&encode_switch_global(false));
        assert_eq!(s.state(), LssState::Waiting);
    }
}
