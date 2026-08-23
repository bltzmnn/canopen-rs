//! CiA 309-3 ASCII gateway — minimal command slice, sans-I/O (docs/EXTENSIONS.md §B1).
//!
//! Wire format (CANopenNode `309/CO_gateway_ascii.h:67-133`):
//!
//! ```text
//! [1] 4 read 0x1017 0 u16        →  [1] OK | 500
//! [1] 4 write 0x1017 0 u16 500   →  [1] OK
//! [1] start node 4               →  [1] OK
//! [1] set sdo_timeout 500        →  [1] OK
//! ```
//!
//! Every request/response line is framed by `[<sequence>] … <CR><LF>`.
//! Responses are `OK`, a bare value, or `ERROR:<code>` where the code is
//! either an SDO abort code (hex) or one of the CiA numeric error codes:
//! 100 syntax error, 101 unsupported object, 200 NMT/HB failure,
//! 300 timeout / CAN failure.
//!
//! The module is transport-free: [`Gateway::feed_line`] consumes one ASCII
//! line and returns the exact response bytes to write back. Executing SDO/
//! NMT operations is the host's job — [`GatewayAction`] tells it what to do.

use crate::datatypes::{DataType, Value};
use crate::nmt::NmtCommand;
use crate::sdo::SdoAbortCode;

/// Numeric gateway error codes (CiA 309-3 subset).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GatewayErrorCode {
    /// Syntax or value out of range.
    Syntax = 100,
    /// Unsupported feature requested.
    UnsupportedObject = 101,
    /// NMT command failed.
    NmtFailed = 200,
    /// SDO timeout.
    Timeout = 300,
    /// Generic CAN failure.
    CanFailed = 301,
}

impl GatewayErrorCode {
    /// Numeric wire code.
    #[must_use]
    pub const fn code(self) -> u16 {
        self as u16
    }
}

/// Data type tokens accepted in read/write commands (CiA 309-3 subset).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GwDataType {
    /// boolean.
    Bool,
    /// i8..i64.
    Int(u8),
    /// u8..u64.
    Uint(u8),
    /// Visible string.
    Str,
}

impl GwDataType {
    /// Maps to the core [`DataType`].
    #[must_use]
    pub const fn to_core(self) -> Option<DataType> {
        Some(match self {
            Self::Bool => DataType::Boolean,
            Self::Str => DataType::VisibleString,
            Self::Int(w) => match w {
                8 => DataType::Integer8,
                16 => DataType::Integer16,
                32 => DataType::Integer32,
                64 => DataType::Integer64,
                _ => return None,
            },
            Self::Uint(w) => match w {
                8 => DataType::Unsigned8,
                16 => DataType::Unsigned16,
                32 => DataType::Unsigned32,
                64 => DataType::Unsigned64,
                _ => return None,
            },
        })
    }

    /// Parses `i16`, `u32`, `b`, `vs` style tokens.
    #[must_use]
    pub fn parse(token: &str) -> Option<Self> {
        let lower = token.to_ascii_lowercase();
        Some(match lower.as_str() {
            "b" => Self::Bool,
            "vs" | "str" => Self::Str,
            _ => {
                let (kind, width) = lower.split_at(1);
                let width: u8 = width.parse().ok()?;
                if !(width == 8 || width == 16 || width == 32 || width == 64) {
                    return None;
                }
                match kind {
                    "i" => Self::Int(width),
                    "u" => Self::Uint(width),
                    _ => return None,
                }
            }
        })
    }
}

/// What the host must execute for one parsed request.
#[derive(Debug, Clone, PartialEq)]
pub enum GatewayAction {
    /// SDO upload from `node`.
    Read {
        /// Target node id.
        node: u8,
        /// Object index.
        index: u16,
        /// Subindex.
        sub: u8,
        /// Declared type for decoding.
        data_type: GwDataType,
    },
    /// SDO download to `node`.
    Write {
        /// Target node id.
        node: u8,
        /// Object index.
        index: u16,
        /// Subindex.
        sub: u8,
        /// Declared type.
        data_type: GwDataType,
        /// Raw little-endian payload.
        payload: Vec<u8>,
    },
    /// NMT command to `node` (`None` = broadcast).
    Nmt {
        /// Command.
        command: GatewayNmt,
        /// Target node id.
        node: u8,
    },
    /// Gateway-local configuration (`set node <v>`); the parser has already
    /// mutated its default-node state — just respond OK.
    SetNode(u8),
}

/// NMT verbs accepted by the gateway.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GatewayNmt {
    /// Start remote node.
    Start,
    /// Stop remote node.
    Stop,
    /// Enter pre-operational.
    PreOperational,
    /// Reset node.
    ResetNode,
    /// Reset communication.
    ResetCommunication,
}

impl From<GatewayNmt> for NmtCommand {
    fn from(g: GatewayNmt) -> Self {
        match g {
            GatewayNmt::Start => Self::StartRemoteNode,
            GatewayNmt::Stop => Self::StopRemoteNode,
            GatewayNmt::PreOperational => Self::EnterPreOperational,
            GatewayNmt::ResetNode => Self::ResetNode,
            GatewayNmt::ResetCommunication => Self::ResetCommunication,
        }
    }
}

/// A parsed request awaiting execution.
#[derive(Debug, Clone, PartialEq)]
pub struct GatewayRequest {
    /// Sequence number echoed in the response frame.
    pub sequence: u32,
    /// What to execute.
    pub action: GatewayAction,
}

/// Result of feeding one input line.
#[derive(Debug, Clone, PartialEq)]
pub enum GatewayEvent {
    /// Execute this and emit [`Gateway::format_ok`] /
    /// [`Gateway::format_value`] / [`Gateway::format_error`] afterwards.
    Request(GatewayRequest),
    /// Respond immediately with these bytes (help text etc.).
    Respond(String),
    /// Line was blank or a comment — no response at all.
    Ignore,
}

/// The gateway parser/formatter state machine.
#[derive(Debug, Default)]
pub struct Gateway {
    /// Configured default node id (`set node <v>`); requests may omit it.
    default_node: Option<u8>,
}

impl Gateway {
    /// A gateway with no default node configured.
    #[must_use]
    pub const fn new() -> Self {
        Self { default_node: None }
    }

    /// Current default node id.
    #[must_use]
    pub const fn default_node(&self) -> Option<u8> {
        self.default_node
    }

    /// Feeds one complete input line (without line terminator).
    ///
    /// # Errors
    /// Never fails: malformed lines become [`GatewayEvent::Respond`] with an
    /// `ERROR:100` frame.
    #[must_use]
    pub fn feed_line(&mut self, line: &str) -> GatewayEvent {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            return GatewayEvent::Ignore;
        }
        // Frame: [<seq>] <body>
        let Some(rest) = trimmed.strip_prefix('[') else {
            return self.frameless_error();
        };
        let Some(close) = rest.find(']') else {
            return self.frameless_error();
        };
        let seq = &rest[..close];
        let Ok(seq_num) = seq.parse::<u32>() else {
            return GatewayEvent::Respond(format!("[{seq}] ERROR:100"));
        };
        let body = rest[close + 1..].trim();
        match self.parse_request(body) {
            Ok(action) => GatewayEvent::Request(GatewayRequest {
                sequence: seq_num,
                action,
            }),
            Err(code) => GatewayEvent::Respond(format!("[{seq}] ERROR:{}", code.code())),
        }
    }

    fn frameless_error(&self) -> GatewayEvent {
        GatewayEvent::Respond("ERROR:100".to_string())
    }

    /// Parses the request body after the `[seq]` prefix.
    fn parse_request(&mut self, body: &str) -> Result<GatewayAction, GatewayErrorCode> {
        let mut tokens = body.split_whitespace();
        let first = tokens.next().ok_or(GatewayErrorCode::Syntax)?;

        // Optional leading node id before the verb: "4 read …".
        let (node_token, verb): (Option<&str>, &str) = match first {
            "read" | "write" | "start" | "stop" | "preop" | "reset" | "set" | "help" => {
                (None, first)
            }
            other => (Some(other), tokens.next().ok_or(GatewayErrorCode::Syntax)?),
        };

        match verb {
            "read" => {
                let node = self.resolve_node(node_token)?;
                let index = parse_index(tokens.next().ok_or(GatewayErrorCode::Syntax)?)?;
                let sub = tokens.next().map_or(Ok(0), parse_sub)?;
                let ty = tokens.next().map_or(Ok(GwDataType::Uint(32)), |t| {
                    GwDataType::parse(t).ok_or(GatewayErrorCode::Syntax)
                })?;
                Ok(GatewayAction::Read {
                    node,
                    index,
                    sub,
                    data_type: ty,
                })
            }
            "write" => {
                let node = self.resolve_node(node_token)?;
                let index = parse_index(tokens.next().ok_or(GatewayErrorCode::Syntax)?)?;
                let sub = tokens.next().map_or(Ok(0), parse_sub)?;
                let ty_tok = tokens.next().ok_or(GatewayErrorCode::Syntax)?;
                let ty = GwDataType::parse(ty_tok).ok_or(GatewayErrorCode::Syntax)?;
                let raw = tokens.collect::<Vec<_>>().join(" ");
                let payload = encode_payload(ty, &raw).ok_or(GatewayErrorCode::Syntax)?;
                Ok(GatewayAction::Write {
                    node,
                    index,
                    sub,
                    data_type: ty,
                    payload,
                })
            }
            "start" | "stop" | "preop" => {
                let node = self.nmt_target(node_token, &mut tokens)?;
                let command = match verb {
                    "start" => GatewayNmt::Start,
                    "stop" => GatewayNmt::Stop,
                    _ => GatewayNmt::PreOperational,
                };
                Ok(GatewayAction::Nmt { command, node })
            }
            "reset" => {
                let mode = tokens.next().ok_or(GatewayErrorCode::Syntax)?;
                let node = self.nmt_target(node_token, &mut tokens)?;
                let command = match mode {
                    "node" => GatewayNmt::ResetNode,
                    "comm" => GatewayNmt::ResetCommunication,
                    _ => return Err(GatewayErrorCode::Syntax),
                };
                Ok(GatewayAction::Nmt { command, node })
            }
            "set" => {
                let key = tokens.next().ok_or(GatewayErrorCode::Syntax)?;
                let val = tokens.next().ok_or(GatewayErrorCode::Syntax)?;
                match key {
                    "node" => {
                        let n: u8 = val.parse().map_err(|_| GatewayErrorCode::Syntax)?;
                        if !(1..=127).contains(&n) {
                            return Err(GatewayErrorCode::Syntax);
                        }
                        self.default_node = Some(n);
                        Ok(GatewayAction::SetNode(n))
                    }
                    _ => Err(GatewayErrorCode::UnsupportedObject),
                }
            }
            "help" | "led" | "log" => Err(GatewayErrorCode::UnsupportedObject),
            _ => Err(GatewayErrorCode::Syntax),
        }
    }

    /// Resolves the target of an NMT verb, accepting all three spellings:
    /// `<id> start`, `start <id>`, `start node <id>`.
    fn nmt_target(
        &self,
        node_token: Option<&str>,
        tokens: &mut dyn Iterator<Item = &str>,
    ) -> Result<u8, GatewayErrorCode> {
        if let Some(t) = node_token {
            return self.resolve_node(Some(t));
        }
        match tokens.next() {
            None => self.resolve_node(None),
            Some("node") => self.resolve_node(Some(tokens.next().ok_or(GatewayErrorCode::Syntax)?)),
            Some(t) => self.resolve_node(Some(t)),
        }
    }

    fn resolve_node(&self, token: Option<&str>) -> Result<u8, GatewayErrorCode> {
        match token {
            Some(t) => t
                .parse::<u8>()
                .map_err(|_| GatewayErrorCode::Syntax)
                .and_then(|n| {
                    (1..=127)
                        .contains(&n)
                        .then_some(n)
                        .ok_or(GatewayErrorCode::Syntax)
                }),
            None => self.default_node.ok_or(GatewayErrorCode::Syntax),
        }
    }

    /// `[<seq>] OK`
    #[must_use]
    pub fn format_ok(sequence: u32) -> String {
        format!("[{sequence}] OK")
    }

    /// `[<seq>] <value>`
    #[must_use]
    pub fn format_value(sequence: u32, value: &Value) -> String {
        format!("[{sequence}] {value}")
    }

    /// `[<seq>] ERROR:<code>` — numeric CiA code or hex SDO abort.
    #[must_use]
    pub fn format_error(sequence: u32, error: &GatewayFailure) -> String {
        match error {
            GatewayFailure::Code(c) => format!("[{sequence}] ERROR:{}", c.code()),
            GatewayFailure::SdoAborted(a) => {
                format!("[{sequence}] ERROR:{:#x}", a.to_u32())
            }
        }
    }
}

/// Why an executed action failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GatewayFailure {
    /// One of the numeric CiA codes.
    Code(GatewayErrorCode),
    /// The device aborted with a standard SDO abort code.
    SdoAborted(SdoAbortCode),
}

fn parse_index(tok: &str) -> Result<u16, GatewayErrorCode> {
    let t = tok.trim();
    if let Some(hex) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        u16::from_str_radix(hex, 16).map_err(|_| GatewayErrorCode::Syntax)
    } else {
        t.parse::<u16>().map_err(|_| GatewayErrorCode::Syntax)
    }
}

fn parse_sub(tok: &str) -> Result<u8, GatewayErrorCode> {
    tok.parse::<u8>().map_err(|_| GatewayErrorCode::Syntax)
}

/// Encodes a typed literal into wire bytes.
#[must_use]
pub fn encode_payload(ty: GwDataType, raw: &str) -> Option<Vec<u8>> {
    match ty {
        GwDataType::Bool => {
            let n = raw.parse::<u8>().ok()?;
            Some(vec![u8::from(n != 0)])
        }
        GwDataType::Str => Some(raw.as_bytes().to_vec()),
        GwDataType::Int(w) | GwDataType::Uint(w) => {
            let signed_int = raw.parse::<i64>().ok()?;
            let unsigned = raw.parse::<u64>().ok();
            let mut buf = [0u8; 8];
            let n = match ty {
                GwDataType::Int(_) => Value::Int(signed_int).encode_le(&mut buf).ok()?,
                _ => Value::Uint(unsigned?).encode_le(&mut buf).ok()?,
            };
            let width = usize::from(w / 8);
            Some(buf[..width.max(n)].to_vec())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_request_with_explicit_node() {
        let mut gw = Gateway::new();
        match gw.feed_line("[1] 4 read 0x1017 0 u16") {
            GatewayEvent::Request(req) => {
                assert_eq!(req.sequence, 1);
                assert_eq!(
                    req.action,
                    GatewayAction::Read {
                        node: 4,
                        index: 0x1017,
                        sub: 0,
                        data_type: GwDataType::Uint(16)
                    }
                );
            }
            other => panic!("expected request, got {other:?}"),
        }
        assert_eq!(Gateway::format_value(1, &Value::Uint(500)), "[1] 500");
    }

    #[test]
    fn default_node_from_set_is_used_when_omitted() {
        let mut gw = Gateway::new();
        // Missing default → syntax error.
        assert_eq!(
            gw.feed_line("[2] read 0x1000"),
            GatewayEvent::Respond("[2] ERROR:100".to_string())
        );
        // Configure default.
        assert_eq!(
            gw.feed_line("[3] set node 5"),
            GatewayEvent::Request(GatewayRequest {
                sequence: 3,
                action: GatewayAction::SetNode(5),
            })
        );
        assert_eq!(gw.default_node(), Some(5));
        // Now the bare read resolves to node 5.
        match gw.feed_line("[4] read 0x1000") {
            GatewayEvent::Request(GatewayRequest { action, .. }) => {
                assert!(matches!(
                    action,
                    GatewayAction::Read {
                        node: 5,
                        index: 0x1000,
                        sub: 0,
                        ..
                    }
                ));
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn write_request_encodes_typed_payload() {
        let mut gw = Gateway::new();
        match gw.feed_line("[7] 9 write 0x1017 0 u16 500") {
            GatewayEvent::Request(req) => {
                assert_eq!(
                    req.action,
                    GatewayAction::Write {
                        node: 9,
                        index: 0x1017,
                        sub: 0,
                        data_type: GwDataType::Uint(16),
                        payload: vec![0xF4, 0x01],
                    }
                );
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(Gateway::format_ok(7), "[7] OK");
    }

    #[test]
    fn nmt_verbs_map_correctly() {
        let mut gw = Gateway::new();
        for (line, node, command) in [
            ("[1] start node 4", 4u8, GatewayNmt::Start),
            ("[2] 4 stop", 4, GatewayNmt::Stop),
            ("[3] 8 preop", 8, GatewayNmt::PreOperational),
            ("[4] start 6", 6, GatewayNmt::Start), // bare id without keyword
        ] {
            match gw.feed_line(line) {
                GatewayEvent::Request(GatewayRequest { action, .. }) => {
                    assert_eq!(action, GatewayAction::Nmt { command, node });
                }
                other => panic!("{line}: {other:?}"),
            }
        }
        // reset has a mandatory mode token.
        assert!(matches!(
            gw.feed_line("[5] reset comm 4"),
            GatewayEvent::Request(GatewayRequest {
                action: GatewayAction::Nmt {
                    command: GatewayNmt::ResetCommunication,
                    node: 4
                },
                ..
            })
        ));
        // Wrong mode token → syntax.
        assert_eq!(
            gw.feed_line("[6] reset banana 4"),
            GatewayEvent::Respond("[6] ERROR:100".to_string())
        );
    }

    #[test]
    fn malformed_frames_and_comments() {
        let mut gw = Gateway::new();
        assert_eq!(gw.feed_line(""), GatewayEvent::Ignore);
        assert_eq!(gw.feed_line("# comment"), GatewayEvent::Ignore);
        // Frameless garbage still yields a response so clients never hang.
        assert_eq!(
            gw.feed_line("garbage"),
            GatewayEvent::Respond("ERROR:100".to_string())
        );
        assert_eq!(
            gw.feed_line("[abc] read 0x1000"),
            GatewayEvent::Respond("[abc] ERROR:100".to_string())
        );
        // Unknown verb → syntax error (CiA reserves 101 for SDO-level
        // unsupported-object aborts).
        assert_eq!(
            gw.feed_line("[9] teleport 4"),
            GatewayEvent::Respond("[9] ERROR:100".to_string())
        );
    }

    #[test]
    fn sdo_abort_formatting_uses_hex_code() {
        assert_eq!(
            Gateway::format_error(
                3,
                &GatewayFailure::SdoAborted(SdoAbortCode::ObjectDoesNotExist)
            ),
            "[3] ERROR:0x6020000"
        );
        assert_eq!(
            Gateway::format_error(3, &GatewayFailure::Code(GatewayErrorCode::Timeout)),
            "[3] ERROR:300"
        );
    }
}
