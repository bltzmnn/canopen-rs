//! `canopen` — command line tool for the canopen-rs stack.
//!
//! Cross-platform subcommands (`eds dump` / `eds gen`) work anywhere; bus
//! subcommands (`read`/`write`/`nmt`/`monitor`) require Linux SocketCAN.

use std::path::PathBuf;

use anyhow::{Context, Result};
use canopen_core::datatypes::{DataType, Value};
use canopen_core::types::Address;
use clap::{Parser, Subcommand, ValueEnum};

mod eds_io {
    //! Shared EDS load/save helpers.
    use std::fs;
    use std::path::Path;

    use anyhow::Context;

    /// Loads and parses an EDS file.
    pub fn load(path: &Path) -> anyhow::Result<canopen_host::eds::Eds> {
        let text =
            fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        canopen_host::eds::parse(&text).context("parsing EDS")
    }
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum SdoType {
    U8,
    U16,
    U32,
    U64,
    I8,
    I16,
    I32,
    I64,
    Str,
}

impl SdoType {
    fn data_type(self) -> DataType {
        match self {
            Self::U8 => DataType::Unsigned8,
            Self::U16 => DataType::Unsigned16,
            Self::U32 => DataType::Unsigned32,
            Self::U64 => DataType::Unsigned64,
            Self::I8 => DataType::Integer8,
            Self::I16 => DataType::Integer16,
            Self::I32 => DataType::Integer32,
            Self::I64 => DataType::Integer64,
            Self::Str => DataType::VisibleString,
        }
    }
}

#[derive(Debug, Parser)]
#[command(name = "canopen", about = "CANopen master/node toolkit")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// EDS/DCF file operations.
    Eds {
        #[command(subcommand)]
        cmd: EdsCommand,
    },
    /// SDO read from a node.
    Read {
        /// CAN interface, e.g. vcan0
        iface: String,
        /// Node id 1..=127
        node: NodeArg,
        /// Object index (dec or 0x-prefixed hex)
        index: IndexArg,
        /// Subindex (default 0; dec or 0x-prefixed hex)
        sub: Option<SubArg>,
        /// Expected data type for decoding
        #[arg(long)]
        r#type: Option<SdoType>,
    },
    /// SDO write to a node.
    Write {
        iface: String,
        node: NodeArg,
        index: IndexArg,
        /// Raw value: integer literal or string. 不带 `--type` 时整数按有符号
        /// 最小宽度（符号感知）小端编码，其余按 ASCII 字节；`--type str` 时按
        /// 可见字符串原样写入（非小端字节）；精确宽度请用 `--type`。
        ///
        /// `allow_hyphen_values`: `-2`/`-0x80` are data, not options
        /// (docs/DESIGN-DECISIONS.md 十M9).
        #[arg(allow_hyphen_values = true)]
        value: String,
        /// Subindex (dec or 0x-prefixed hex)
        sub: Option<SubArg>,
        #[arg(long)]
        r#type: Option<SdoType>,
    },
    /// Send an NMT command.
    Nmt {
        iface: String,
        /// start | stop | preop | reset | reset-comm
        cmd: NmtCmd,
        /// "all" for broadcast or a node id
        target: TargetArg,
    },
    /// Live bus monitor with COB-ID classification and heartbeat tracking.
    Monitor {
        iface: String,
        /// Seconds to run (0 = until interrupted)
        #[arg(short = 'd', long, default_value_t = 10)]
        duration: u64,
    },
}

#[derive(Debug, Subcommand)]
enum EdsCommand {
    /// Print a summary of the object dictionary.
    Dump { path: PathBuf },
    /// Generate a Rust module building the dictionary.
    Gen {
        path: PathBuf,
        /// Output file (stdout when omitted)
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Generated function name
        #[arg(long, default_value = "build_object_dictionary")]
        fn_name: String,
    },
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum NmtCmd {
    Start,
    Stop,
    Preop,
    Reset,
    ResetComm,
}

impl From<NmtCmd> for canopen_core::nmt::NmtCommand {
    fn from(c: NmtCmd) -> Self {
        match c {
            NmtCmd::Start => Self::StartRemoteNode,
            NmtCmd::Stop => Self::StopRemoteNode,
            NmtCmd::Preop => Self::EnterPreOperational,
            NmtCmd::Reset => Self::ResetNode,
            NmtCmd::ResetComm => Self::ResetCommunication,
        }
    }
}

/// Hex-or-decimal object index.
#[derive(Debug, Clone, Copy)]
struct IndexArg(u16);

impl std::str::FromStr for IndexArg {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self> {
        Ok(Self(parse_index(s)?))
    }
}

/// SDO node id argument: validated `1..=127` at the CLI boundary so node=0
/// can never masquerade as a unicast SDO target (`docs/DESIGN-DECISIONS.md` P2-2; NMT's
/// `TargetArg` already enforced this).
#[derive(Debug, Clone, Copy)]
struct NodeArg(u8);

impl std::str::FromStr for NodeArg {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self> {
        let n: u8 = s.trim().parse()?;
        anyhow::ensure!((1..=127).contains(&n), "node id out of range 1..=127");
        Ok(Self(n))
    }
}

/// Hex-or-decimal SDO subindex (`docs/IMPLEMENTATION.md` §784: numeric
/// arguments accept a `0x` prefix — the subindex previously did not).
#[derive(Debug, Clone, Copy)]
struct SubArg(u8);

impl std::str::FromStr for SubArg {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self> {
        let t = s.trim();
        let v = if let Some(hex) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
            u8::from_str_radix(hex, 16)?
        } else {
            t.parse()?
        };
        Ok(Self(v))
    }
}

#[derive(Debug, Clone)]
enum TargetArg {
    All,
    Node(u8),
}

impl std::str::FromStr for TargetArg {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self> {
        if s.eq_ignore_ascii_case("all") {
            Ok(Self::All)
        } else {
            let n: u8 = s.parse()?;
            anyhow::ensure!((1..=127).contains(&n), "node id out of range 1..=127");
            Ok(Self::Node(n))
        }
    }
}

fn parse_index(s: &str) -> Result<u16> {
    let t = s.trim();
    if let Some(hex) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        Ok(u16::from_str_radix(hex, 16)?)
    } else {
        Ok(t.parse()?)
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::Eds { cmd } => eds_command(cmd),
        Command::Read {
            iface,
            node,
            index,
            sub,
            r#type,
        } => {
            let index = index.0;
            #[cfg(target_os = "linux")]
            {
                bus::cmd_read(&iface, node.0, index, sub.map_or(0, |s| s.0), r#type)
            }
            #[cfg(not(target_os = "linux"))]
            {
                let _ = (iface, node, index, sub, r#type);
                bail!("bus commands require Linux SocketCAN")
            }
        }
        Command::Write {
            iface,
            node,
            index,
            value,
            sub,
            r#type,
        } => {
            let index = index.0;
            #[cfg(target_os = "linux")]
            {
                bus::cmd_write(
                    &iface,
                    node.0,
                    index,
                    &value,
                    sub.map_or(0, |s| s.0),
                    r#type,
                )
            }
            #[cfg(not(target_os = "linux"))]
            {
                let _ = (iface, node, index, value, sub, r#type);
                bail!("bus commands require Linux SocketCAN")
            }
        }
        Command::Nmt { iface, cmd, target } => {
            #[cfg(target_os = "linux")]
            {
                bus::cmd_nmt(&iface, cmd.into(), target)
            }
            #[cfg(not(target_os = "linux"))]
            {
                let _ = (iface, cmd, target);
                bail!("bus commands require Linux SocketCAN")
            }
        }
        Command::Monitor { iface, duration } => {
            #[cfg(target_os = "linux")]
            {
                bus::cmd_monitor(&iface, duration)
            }
            #[cfg(not(target_os = "linux"))]
            {
                let _ = (iface, duration);
                bail!("bus commands require Linux SocketCAN")
            }
        }
    }
}

fn eds_command(cmd: EdsCommand) -> Result<()> {
    match cmd {
        EdsCommand::Dump { path } => {
            let eds = eds_io::load(&path)?;
            // docs/DESIGN-DECISIONS.md EDS-01: every tolerant-parsing fallback is recorded in
            // `eds.warnings` — python-canopen logs the same events via
            // `logger.warning`, so the CLI must not swallow them.
            for warning in &eds.warnings {
                eprintln!("warning: {warning}");
            }
            println!(
                "{:<8} {:<28} {:<14} {:<6} PDO",
                "INDEX", "NAME", "TYPE", "ACCESS"
            );
            for object in eds.objects.values() {
                // CompactSubObj objects are shown expanded, exactly like
                // `eds gen` builds them (`docs/DESIGN-DECISIONS.md` 十M24: the raw sub-0
                // template used to be printed instead of subs 1..=N).
                let expanded = canopen_host::eds::expand_compact_sub_obj(object);
                let subs: Vec<(&u8, &canopen_host::eds::EdsSubEntry)> = match &expanded {
                    Some(map) => map.iter().collect(),
                    None => object.subs.iter().collect(),
                };
                if subs.is_empty() {
                    continue;
                }
                // Object header line for multi-sub objects.
                if subs.len() > 1 || !subs.iter().any(|(s, _)| **s == 0) && !object.name.is_empty()
                {
                    println!("{:#06x}   {}", object.index, object.name);
                }
                for (sub, se) in subs {
                    println!(
                        "{index:>4x}:{sub:<3}  {:<28} {:<14} {:<6} {}",
                        se.name,
                        format!("{:?}", se.data_type),
                        format!("{:?}", se.access).to_lowercase(),
                        if se.pdo_mappable { "yes" } else { "-" },
                        index = object.index,
                    );
                }
            }
            if let Some(node) = eds.configured_node_id {
                println!("\nconfigured NodeID expression: {node:?}");
            }
            Ok(())
        }
        EdsCommand::Gen {
            path,
            output,
            fn_name,
        } => {
            let eds = eds_io::load(&path)?;
            for warning in &eds.warnings {
                eprintln!("warning: {warning}");
            }
            let source = canopen_host::codegen::generate_od_module(&eds, &fn_name)
                .map_err(anyhow::Error::msg)?;
            match output {
                Some(p) => {
                    std::fs::write(&p, &source)
                        .with_context(|| format!("writing {}", p.display()))?;
                    println!("wrote {}", p.display());
                }
                None => print!("{source}"),
            }
            Ok(())
        }
    }
}

#[cfg(target_os = "linux")]
mod bus {
    use super::*;
    use canopen_host::transport::{Incoming, SocketCan};

    pub fn cmd_read(iface: &str, node: u8, index: u16, sub: u8, ty: Option<SdoType>) -> Result<()> {
        let mut bus = SocketCan::open(iface)?;
        if let Some(ty) = ty {
            let value = bus.sdo_read(node, Address::new(index, sub), ty.data_type())?;
            println!("{index:#06x}:{sub:02x} = {value}");
            return Ok(());
        }
        // No --type: read raw bytes and decode by the response width
        // (docs/DESIGN-DECISIONS.md P3-20 — a fixed Unsigned32 guess aborted on narrower
        // objects).
        let bytes = bus.read_domain_bytes(node, Address::new(index, sub))?;
        let shown: Vec<String> = bytes.iter().map(|b| format!("{b:#04x}")).collect();
        match bytes.len() {
            0 => println!("{index:#06x}:{sub:02x} = <empty>"),
            1 => println!("{index:#06x}:{sub:02x} = {:#x} (u8)", bytes[0]),
            2 => {
                let v = u16::from_le_bytes(bytes.as_slice().try_into().expect("len checked"));
                println!("{index:#06x}:{sub:02x} = {v:#06x} (u16)");
            }
            // 4/8-byte values are shown unsigned, consistent with the 1/2-byte
            // cases — a U32 with the high bit set must not print as a negative
            // i32 (docs/DESIGN-DECISIONS.md #20); the raw bytes stay visible for signed reads.
            4 => {
                let v = u32::from_le_bytes(bytes.as_slice().try_into().expect("len checked"));
                println!(
                    "{index:#06x}:{sub:02x} = {v:#010x} (u32) [raw: {}]",
                    shown.join(" ")
                );
            }
            8 => {
                let v = u64::from_le_bytes(bytes.as_slice().try_into().expect("len checked"));
                println!(
                    "{index:#06x}:{sub:02x} = {v:#018x} (u64) [raw: {}]",
                    shown.join(" ")
                );
            }
            n => println!("{index:#06x}:{sub:02x} = [{n} bytes] {}", shown.join(" ")),
        }
        Ok(())
    }

    pub fn cmd_write(
        iface: &str,
        node: u8,
        index: u16,
        raw: &str,
        sub: u8,
        ty: Option<SdoType>,
    ) -> Result<()> {
        let payload = encode_value(raw, ty.map(SdoType::data_type))?;
        let mut bus = SocketCan::open(iface)?;
        bus.sdo_write(node, Address::new(index, sub), &payload)?;
        println!("{index:#06x}:{sub:02x} <- {}", hex(&payload));
        Ok(())
    }

    pub fn cmd_nmt(
        iface: &str,
        cmd: canopen_core::nmt::NmtCommand,
        target: TargetArg,
    ) -> Result<()> {
        let mut bus = SocketCan::open(iface)?;
        bus.send_nmt(
            cmd,
            match target {
                TargetArg::All => None,
                TargetArg::Node(n) => Some(n),
            },
        )?;
        println!("NMT {:?} sent", cmd);
        Ok(())
    }

    pub fn cmd_monitor(iface: &str, duration: u64) -> Result<()> {
        let mut bus = SocketCan::open(iface)?;
        let mut hb = canopen_core::nmt::HeartbeatConsumer::new();
        let deadline = if duration == 0 {
            None
        } else {
            // docs/DESIGN-DECISIONS.md 十L12: `u64::MAX` seconds would overflow the platform
            // `Instant`; fail loudly instead of panicking.
            Some(
                std::time::Instant::now()
                    .checked_add(std::time::Duration::from_secs(duration))
                    .ok_or_else(|| anyhow::anyhow!("--duration is too large"))?,
            )
        };
        println!("monitoring {iface}…");
        let mut seen_nodes: std::collections::BTreeSet<u16> = Default::default();
        let mut last_table = 0.0f64;
        loop {
            if deadline.is_some_and(|d| std::time::Instant::now() >= d) {
                break;
            }
            for event in hb.poll(canopen_host::transport::now_ms()) {
                println!("  !! {event:?}");
            }
            let Some(incoming) = bus.recv(std::time::Duration::from_millis(100))? else {
                continue;
            };
            let Incoming::Data(rx) = incoming else {
                continue;
            };
            if let Some((state, _)) =
                canopen_core::nmt::decode_heartbeat(rx.payload()).filter(|_| {
                    // A heartbeat is exactly one byte (CANopenNode
                    // `CO_HBconsumer.c:43`); a longer `0x700+n` frame must
                    // not refresh the deadline and suppress Lost warnings
                    // (`docs/DESIGN-DECISIONS.md` 十M18).
                    (0x700..=0x77F).contains(&rx.cob_id) && rx.payload().len() == 1
                })
            {
                let node = rx.cob_id - 0x700;
                // COB 0x700 decodes to node 0, which is outside the valid
                // 1..=127 range — guard like `host/src/client.rs` instead of
                // panicking on a stray frame (docs/DESIGN-DECISIONS.md #13).
                let Ok(node_id) = canopen_core::types::NodeId::try_from(node as u8) else {
                    continue;
                };
                // Supervise every producer we see: default deadline 1.5 s so
                // `!! Lost` warnings can actually fire (docs/DESIGN-DECISIONS.md P3-50).
                hb.set_timeout(node_id, 1500);
                for ev in hb.on_heartbeat(node_id, rx.payload(), canopen_host::transport::now_ms())
                {
                    println!("  !! {ev:?}");
                }
                println!("{:>9.3}  HB  node {node:<3} {state:?}", secs());
                seen_nodes.insert(node);
                if secs() - last_table >= 5.0 {
                    last_table = secs();
                    println!("  -- heartbeat table --");
                    for &n in &seen_nodes {
                        // docs/DESIGN-DECISIONS.md L8: `seen_nodes` only ever contains ids
                        // that passed `hb.on_heartbeat` earlier, but rely on
                        // the invariant explicitly instead of `.expect` so a
                        // future regression cannot turn into a panic.
                        let Some(nid) = canopen_core::types::NodeId::try_from(n as u8).ok() else {
                            continue;
                        };
                        println!(
                            "     node {n:<3} {:?} {}",
                            hb.state_of(nid)
                                .unwrap_or(canopen_core::nmt::NmtState::Initialising),
                            if hb.supervises(nid) { "supervised" } else { "" }
                        );
                    }
                }
            } else {
                let (kind, detail) = classify(rx.cob_id);
                println!(
                    "{:>9.3}  {:<5}{} cob {:#06x} {}",
                    secs(),
                    kind,
                    if detail.is_empty() {
                        String::new()
                    } else {
                        format!(" [{detail}]")
                    },
                    rx.cob_id,
                    hex(rx.payload())
                );
            }
        }
        Ok(())
    }

    fn secs() -> f64 {
        canopen_host::transport::now_ms() as f64 / 1000.0
    }

    fn pdo_detail(num: Option<u8>) -> &'static str {
        match num {
            Some(1) => "PDO1",
            Some(2) => "PDO2",
            Some(3) => "PDO3",
            _ => "PDO4",
        }
    }

    fn classify(cob_id: u16) -> (&'static str, &'static str) {
        // Predefined connection set: TPDO{n} lives in (0x180+n·0x100, …+0x80],
        // RPDO{n} in (0x200+n·0x100, …+0x80]. The previous single-range match
        // swallowed the RPDO1..3 segments into "TPDO" (docs/DESIGN-DECISIONS.md P3-3).
        fn in_pdo_band(cob_id: u16, first: u16) -> Option<u8> {
            (0..4u16)
                .find(|&i| {
                    let lo = first + i * 0x100;
                    // Upper bound excludes the next band's base COB-ID
                    // (node-id starts at 1, so `lo + 0x80` itself has no
                    // owner — docs/DESIGN-DECISIONS.md P3-3, see git history).
                    lo < cob_id && cob_id <= lo + 0x7F
                })
                .map(|i| i as u8 + 1)
        }
        match cob_id {
            0x000 => ("NMT", ""),
            0x080 => ("SYNC", ""),
            0x100 => ("TIME", ""),
            0x7E4 | 0x7E5 => ("LSS", ""),
            _ if (0x081..=0x0FF).contains(&cob_id) => ("EMCY", ""),
            _ if in_pdo_band(cob_id, 0x180).is_some() => {
                ("TPDO", pdo_detail(in_pdo_band(cob_id, 0x180)))
            }
            _ if in_pdo_band(cob_id, 0x200).is_some() => {
                ("RPDO", pdo_detail(in_pdo_band(cob_id, 0x200)))
            }
            _ if (0x581..=0x5FF).contains(&cob_id) => ("SDOtx", ""),
            _ if (0x601..=0x67F).contains(&cob_id) => ("SDOrx", ""),
            _ if (0x700..=0x77F).contains(&cob_id) => ("HB", ""),
            _ => ("?", ""),
        }
    }

    fn hex(data: &[u8]) -> String {
        data.iter()
            .map(|b| format!("{b:02x}"))
            .collect::<Vec<_>>()
            .join(" ")
    }

    /// Encodes a CLI value into wire bytes.
    ///
    /// Explicit `--type` wins; otherwise integers encode minimally
    /// (signed-aware) and anything else is treated as ASCII bytes.
    fn encode_value(raw: &str, ty: Option<DataType>) -> Result<Vec<u8>> {
        if let Some(dt) = ty {
            let value = match dt {
                DataType::VisibleString => Value::Str(raw.to_string()),
                DataType::OctetString | DataType::Domain | DataType::UnicodeString => {
                    // Variable-length payloads may exceed one CAN frame's 8
                    // bytes; the SDO layer picks the right transfer mode
                    // (docs/DESIGN-DECISIONS.md P3, --type str >8B).
                    let bytes = canopen_host::eds::hex_bytes(raw);
                    if bytes.is_empty() && !raw.is_empty() {
                        Value::Str(raw.to_string())
                    } else {
                        Value::Bytes(bytes)
                    }
                }
                other => {
                    // Detect the sign BEFORE stripping the radix prefix:
                    // "-0x8000" must parse as -32768, not fail (or worse,
                    // lose the sign) — `docs/DESIGN-DECISIONS.md` P2-2. Parsing goes through
                    // i128 so Unsigned64 values up to 2^64-1 survive
                    // (docs/DESIGN-DECISIONS.md R10-P1-7); anything wider is a hard error.
                    let trimmed = raw.trim();
                    let (negative, digits) = match trimmed.strip_prefix('-') {
                        Some(rest) => (true, rest.trim()),
                        None => (false, trimmed),
                    };
                    let (digits, radix) = match digits
                        .strip_prefix("0x")
                        .or_else(|| digits.strip_prefix("0X"))
                    {
                        Some(hex) => (hex, 16),
                        None => (digits, 10),
                    };
                    let magnitude = i128::from_str_radix(digits, radix)
                        .map_err(|_| anyhow::anyhow!("value is not an integer"))?;
                    let n = if negative { -magnitude } else { magnitude };
                    match other {
                        t if is_signed(t) => {
                            // docs/DESIGN-DECISIONS.md #34: a hex literal may be the
                            // two's-complement spelling of a negative value
                            // (`--type i32 0x80000000` → -2147483648),
                            // matching the EDS `wrap_signed_hex` path.
                            // Decimal literals keep their plain sign.
                            let n = if radix == 16 {
                                let bits = t.fixed_size().unwrap_or(8) * 8;
                                // docs/DESIGN-DECISIONS.md 十M10: reject a hex magnitude that
                                // does not fit the target width instead of
                                // wrapping it silently (`--type i8 0x100` used
                                // to write 0x00). python-canopen's
                                // `_signed_int_from_hex` raises for the same
                                // input.
                                let max_unsigned = if bits >= 64 {
                                    i128::from(u64::MAX)
                                } else {
                                    (1i128 << bits) - 1
                                };
                                anyhow::ensure!(
                                    magnitude <= max_unsigned,
                                    "value {raw} out of range for {t:?}"
                                );
                                canopen_host::eds::wrap_signed_hex(n as i64, bits).ok_or_else(
                                    || anyhow::anyhow!("value {raw} out of range for {t:?}"),
                                )? as i128
                            } else {
                                n
                            };
                            anyhow::ensure!(
                                (i64::MIN as i128..=i64::MAX as i128).contains(&n),
                                "value {n} out of range for {t:?}"
                            );
                            Value::Int(n as i64)
                        }
                        _ => {
                            // Negative values for unsigned types are a hard
                            // error, never a silent clamp to 0 (`docs/DESIGN-DECISIONS.md` P2-2).
                            anyhow::ensure!(n >= 0, "negative value for {other:?}");
                            anyhow::ensure!(
                                n <= u64::MAX as i128,
                                "value {n} out of range for {other:?}"
                            );
                            Value::Uint(n as u64)
                        }
                    }
                }
            };
            if dt.fixed_size().is_some() {
                let mut buf = [0u8; 8];
                let n = value.encode_as(dt, &mut buf)?;
                return Ok(buf[..n].to_vec());
            }
            // Variable-length: size the scratch buffer to the value.
            let mut buf = vec![0u8; value.encoded_len()];
            let n = value.encode_as(dt, &mut buf)?;
            return Ok(buf[..n].to_vec());
        }
        // Auto mode: honour 0x-prefixed / signed integers first (docs/DESIGN-DECISIONS.md
        // P2-18 — previously "0xFF" was silently sent as 4 ASCII bytes),
        // falling back to ASCII text only when the token is not numeric.
        if let Some(n) = canopen_host::eds::parse_int(raw.trim()) {
            let v = Value::Int(n);
            let mut buf = [0u8; 8];
            let n = v.encode_le(&mut buf)?;
            return Ok(buf[..n].to_vec());
        }
        // A token that *looks* numeric but exceeds i64 (e.g. a full
        // Unsigned64) must fail loudly instead of silently degrading to
        // ASCII bytes (docs/DESIGN-DECISIONS.md R10-P1-7).
        let t = raw.trim();
        // docs/DESIGN-DECISIONS.md 八L11: strip the sign first — a negative literal beyond
        // i64 (e.g. `-99999999999999999999999`) previously failed the
        // u128-only probe and silently degraded to ASCII bytes.
        let magnitude = t
            .strip_prefix('-')
            .or_else(|| t.strip_prefix('+'))
            .map_or(t, str::trim_start);
        // docs/DESIGN-DECISIONS.md 十M11: "looks numeric" must not depend on `u128` being
        // able to hold the literal — a 44-digit decimal used to fall
        // through as ASCII. Digits-only after the optional radix prefix is
        // the robust criterion (and `i64::MIN` now parses via i128 below).
        let (digits, radix) = match magnitude
            .strip_prefix("0x")
            .or_else(|| magnitude.strip_prefix("0X"))
        {
            Some(hex) => (hex, 16),
            None => (magnitude, 10),
        };
        let looks_numeric = !digits.is_empty() && digits.chars().all(|c| c.is_digit(radix));
        anyhow::ensure!(
            !looks_numeric,
            "numeric value {t} exceeds the supported 64-bit signed range — \
             pass an explicit --type and a value within it"
        );
        Ok(raw.as_bytes().to_vec())
    }

    const fn is_signed(dt: DataType) -> bool {
        matches!(
            dt,
            DataType::Integer8
                | DataType::Integer16
                | DataType::Integer24
                | DataType::Integer32
                | DataType::Integer40
                | DataType::Integer48
                | DataType::Integer56
                | DataType::Integer64
        )
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        /// docs/DESIGN-DECISIONS.md 十M24: positional subindex arguments accept `0x` hex, as
        /// `docs/IMPLEMENTATION.md` promises for numeric arguments.
        #[test]
        fn sub_arg_accepts_hex_and_decimal() {
            use std::str::FromStr;
            assert_eq!(SubArg::from_str("5").unwrap().0, 5);
            assert_eq!(SubArg::from_str("0x0A").unwrap().0, 10);
            assert_eq!(SubArg::from_str("0Xff").unwrap().0, 0xFF);
            assert!(SubArg::from_str("0x100").is_err());
            assert!(SubArg::from_str("nope").is_err());
        }

        /// docs/DESIGN-DECISIONS.md R10-P3: `classify` had zero coverage while its PDO band
        /// boundaries were being reworked — pin the predefined connection set
        /// segmentation, including the exclusive upper bounds.
        #[test]
        fn classify_predefined_connection_set() {
            let cases: &[(u16, &str)] = &[
                (0x000, "NMT"),
                (0x080, "SYNC"),
                (0x100, "TIME"),
                (0x081, "EMCY"),
                (0x0C1, "EMCY"), // node-id 0x41
                (0x7E4, "LSS"),
                // TPDO bands are (base, base+0x80]: 0x180 itself is NOT TPDO1.
                (0x180, "?"),
                (0x181, "TPDO"),
                (0x200, "?"),
                (0x201, "RPDO"),
                (0x27F, "RPDO"),
                (0x280, "?"),
                (0x281, "TPDO"),
                (0x480, "?"),
                (0x481, "TPDO"),
                (0x4FF, "TPDO"),
                (0x500, "?"),
                (0x501, "RPDO"),
                (0x57F, "RPDO"),
                (0x581, "SDOtx"),
                (0x601, "SDOrx"),
                (0x700, "HB"),
                (0x77F, "HB"),
            ];
            for &(cob, kind) in cases {
                assert_eq!(classify(cob).0, kind, "cob {cob:#06x}");
            }
        }

        /// docs/DESIGN-DECISIONS.md R10-P3 / R10-P1-7: `encode_value` must cover U64 full
        /// range under an explicit type, keep the auto mode's minimal-width
        /// integer encoding, and refuse out-of-i64 numerics instead of
        /// silently degrading them to ASCII.
        #[test]
        fn encode_value_width_matrix() {
            use canopen_core::datatypes::DataType as Dt;

            // Explicit types.
            let u64_max = encode_value("0xFFFFFFFFFFFFFFFF", Some(Dt::Unsigned64)).unwrap();
            assert_eq!(u64_max, [0xFF; 8]);
            let i16_neg = encode_value("-2", Some(Dt::Integer16)).unwrap();
            assert_eq!(i16_neg, [0xFE, 0xFF]);
            let str_long = encode_value("abcdefgh", Some(Dt::VisibleString)).unwrap();
            assert_eq!(str_long, b"abcdefgh"); // >8 bytes allowed now

            // Auto mode: minimal-width integers, hex accepted.
            assert_eq!(encode_value("5", None).unwrap(), vec![5]);
            // Signed-aware minimal width: 500 needs 2 bytes, and so does
            // 255 (> i8::MAX).
            assert_eq!(encode_value("500", None).unwrap(), vec![0xF4, 0x01]);
            assert_eq!(encode_value("0xFF", None).unwrap(), vec![0xFF, 0x00]);

            // Numeric but beyond i64 → loud error, never ASCII bytes.
            assert!(
                encode_value("0xFFFFFFFFFFFFFFFF", None).is_err(),
                "out-of-i64 numeric must not degrade to ASCII"
            );
            // docs/DESIGN-DECISIONS.md 八L11: negative overflow must also be loud — the old
            // u128-only probe let `-999…` slip through as ASCII bytes.
            assert!(
                encode_value("-99999999999999999999999", None).is_err(),
                "negative out-of-i64 numeric must not degrade to ASCII"
            );
            // Non-numeric still falls back to ASCII.
            assert_eq!(encode_value("hello", None).unwrap(), b"hello".to_vec());
        }

        /// 十M9/十M4: `-2` is data (not an option) and `-d` is the monitor
        /// duration short flag documented in the README.
        #[test]
        fn clap_accepts_negative_value_and_duration_short() {
            let cli = Cli::try_parse_from([
                "canopen", "write", "vcan0", "5", "0x6000", "--type", "i16", "-2",
            ])
            .expect("clap must accept -2 as the value");
            match cli.command {
                Command::Write { value, .. } => assert_eq!(value, "-2"),
                other => panic!("unexpected command {other:?}"),
            }
            let cli = Cli::try_parse_from(["canopen", "monitor", "vcan0", "-d", "30"]).unwrap();
            match cli.command {
                Command::Monitor { duration, .. } => assert_eq!(duration, 30),
                other => panic!("unexpected command {other:?}"),
            }
        }

        /// 十M10: a signed hex magnitude beyond the target width must be
        /// rejected, not silently wrapped (python-canopen
        /// `_signed_int_from_hex` raises for the same input).
        #[test]
        fn signed_hex_magnitude_must_fit_the_target_width() {
            use canopen_core::datatypes::DataType as Dt;
            assert!(encode_value("0x100", Some(Dt::Integer8)).is_err());
            assert!(encode_value("0x17F", Some(Dt::Integer8)).is_err());
            assert!(encode_value("0x180", Some(Dt::Integer8)).is_err());
            // In-range two's-complement spellings still wrap as before.
            assert_eq!(
                encode_value("0xFF", Some(Dt::Integer8)).unwrap(),
                vec![0xFF]
            );
            assert_eq!(
                encode_value("0x80", Some(Dt::Integer8)).unwrap(),
                vec![0x80]
            );
        }

        /// 负号在 radix 选择之前应用，显式有符号类型下的负十六进制字面量
        /// 必须保留符号（四轮审计 N9 核验为误报，此前仅有正 hex 覆盖）。
        #[test]
        fn negative_hex_literal_keeps_its_sign() {
            use canopen_core::datatypes::DataType as Dt;
            assert_eq!(
                encode_value("-0x1", Some(Dt::Integer16)).unwrap(),
                vec![0xFF, 0xFF]
            );
            assert_eq!(
                encode_value("-0x80", Some(Dt::Integer8)).unwrap(),
                vec![0x80]
            );
            assert_eq!(
                encode_value("-0x1", Some(Dt::Integer32)).unwrap(),
                vec![0xFF; 4]
            );
            // Auto mode agrees with the explicit path.
            assert_eq!(
                encode_value("-0x1", None).unwrap(),
                encode_value("-1", None).unwrap()
            );
        }

        /// 十M11/十L11: the auto mode's numeric probe no longer depends on
        /// `u128` parseability, and `i64::MIN` is accepted.
        #[test]
        fn auto_mode_rejects_huge_numerics_and_accepts_i64_min() {
            let huge = "99999999999999999999999999999999999999999999";
            assert!(encode_value(huge, None).is_err());
            assert!(encode_value("0xFFFFFFFFFFFFFFFFFFFFFFFF", None).is_err());
            assert_eq!(
                encode_value("-9223372036854775808", None).unwrap(),
                i64::MIN.to_le_bytes().to_vec()
            );
        }
    }
}
