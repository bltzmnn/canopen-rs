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
        node: u8,
        /// Object index (dec or 0x-prefixed hex)
        index: IndexArg,
        /// Subindex (default 0)
        sub: Option<u8>,
        /// Expected data type for decoding
        #[arg(long)]
        r#type: Option<SdoType>,
    },
    /// SDO write to a node.
    Write {
        iface: String,
        node: u8,
        index: IndexArg,
        /// Raw value: integer literal or string; bytes are little-endian.
        value: String,
        sub: Option<u8>,
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
        #[arg(long, default_value_t = 10)]
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
                bus::cmd_read(&iface, node, index, sub.unwrap_or(0), r#type)
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
                bus::cmd_write(&iface, node, index, &value, sub.unwrap_or(0), r#type)
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
            println!(
                "{:<8} {:<28} {:<14} {:<6} PDO",
                "INDEX", "NAME", "TYPE", "ACCESS"
            );
            for object in eds.objects.values() {
                if object.subs.is_empty() {
                    continue;
                }
                // Object header line for multi-sub objects.
                if object.subs.len() > 1 || !object.subs.contains_key(&0) && !object.name.is_empty()
                {
                    println!("{:#06x}   {}", object.index, object.name);
                }
                for (sub, se) in &object.subs {
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
            let source = canopen_host::codegen::generate_od_module(&eds, &fn_name);
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
        let dt = ty.map_or(DataType::Unsigned32, SdoType::data_type);
        let mut bus = SocketCan::open(iface)?;
        let value = bus.sdo_read(node, Address::new(index, sub), dt)?;
        println!("{index:#06x}:{sub:02x} = {value}");
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
            Some(std::time::Instant::now() + std::time::Duration::from_secs(duration))
        };
        println!("monitoring {iface}…");
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
            if let Some((state, _)) = canopen_core::nmt::decode_heartbeat(rx.payload())
                .filter(|_| (0x700..=0x77F).contains(&rx.cob_id))
            {
                let node = rx.cob_id - 0x700;
                for ev in hb.on_heartbeat(
                    canopen_core::types::NodeId::try_from(node as u8).expect("in range"),
                    rx.payload(),
                    canopen_host::transport::now_ms(),
                ) {
                    println!("  !! {ev:?}");
                }
                println!("{:>9.3}  HB  node {node:<3} {state:?}", secs());
            } else {
                let (kind, detail) = classify(rx.cob_id);
                println!(
                    "{:>9.3}  {:<5} cob {:#06x} {}",
                    secs(),
                    kind,
                    rx.cob_id,
                    hex(rx.payload())
                );
                let _ = detail;
            }
        }
        Ok(())
    }

    fn secs() -> f64 {
        canopen_host::transport::now_ms() as f64 / 1000.0
    }

    fn classify(cob_id: u16) -> (&'static str, &'static str) {
        match cob_id {
            0x000 => ("NMT", ""),
            0x080 => ("SYNC", ""),
            0x100 => ("TIME", ""),
            0x7E4 | 0x7E5 => ("LSS", ""),
            _ if (0x081..=0x0FF).contains(&cob_id) => ("EMCY", ""),
            _ if (0x180..=0x17F + 0x300).contains(&cob_id) => ("TPDO", ""),
            _ if (0x200..=0x27F).contains(&cob_id) || (0x300..=0x57F).contains(&cob_id) => {
                ("RPDO", "")
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
            let mut buf = [0u8; 8];
            let value = match dt {
                DataType::VisibleString => Value::Str(raw.to_string()),
                other => {
                    let n = i64::from_str_radix(
                        raw.trim().trim_start_matches("0x"),
                        if raw.trim().to_ascii_lowercase().starts_with("0x") {
                            16
                        } else {
                            10
                        },
                    )
                    .context("value is not an integer")?;
                    match other {
                        t if is_signed(t) => Value::Int(n),
                        _ => Value::Uint(u64::try_from(n.max(0)).context("negative unsigned")?),
                    }
                }
            };
            let n = value.encode_as(dt, &mut buf)?;
            return Ok(buf[..n].to_vec());
        }
        // Auto mode.
        if let Ok(n) = raw.trim().parse::<i64>() {
            let v = Value::Int(n);
            let mut buf = [0u8; 8];
            let n = v.encode_le(&mut buf)?;
            return Ok(buf[..n].to_vec());
        }
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
}
