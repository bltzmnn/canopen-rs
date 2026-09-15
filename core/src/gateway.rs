//! CiA 309-3 ASCII gateway — minimal command slice, sans-I/O (docs/EXTENSIONS.md §B1).
//!
//! Wire format (CANopenNode `309/CO_gateway_ascii.h:67-133`):
//!
//! ```text
//! [1] 4 read 0x1017 0 u16        →  [1] OK | 500
//! [1] 4 write 0x1017 0 u16 500   →  [1] OK
//! [1] start node 4               →  [1] OK
//! [1] set node 5                  →  [1] OK
//! ```
//!
//! Every request/response line is framed by `[<sequence>] … <CR><LF>`.
//! Responses are `OK`, a bare value, or `ERROR:<code>` where the code is
//! either an SDO abort code (hex) or one of the CiA 309-3 numeric codes:
//! 100 request not supported, 101 syntax error, 102 request not processed
//! due to internal state, 103 time-out (`CO_gateway_ascii.h:154-183`).
//!
//! The module is transport-free: [`Gateway::feed_line`] consumes one ASCII
//! line and returns the exact response bytes to write back. Executing SDO/
//! NMT operations is the host's job — [`GatewayAction`] tells it what to do.

use crate::datatypes::{DataType, Value};
use crate::nmt::NmtCommand;
use crate::sdo::SdoAbortCode;

/// Numeric gateway error codes (CiA 309-3 subset, `CO_gateway_ascii.h:154-183`).
///
/// The numeric values follow CiA 309-3 exactly:
/// `100` request not supported, `101` syntax error, `102` not processed due
/// to internal state, `103` time-out. SDO failures are transmitted as their
/// abort code in hex (`ERROR:0x…`), never folded into a numeric code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GatewayErrorCode {
    /// Unknown command / unsupported feature (CiA 309-3 `100`).
    RequestNotSupported = 100,
    /// Malformed request or out-of-range literal (CiA 309-3 `101`).
    Syntax = 101,
    /// Valid request that could not be executed (CiA 309-3 `102`).
    InternalState = 102,
    /// Operation timed out locally (CiA 309-3 `103`).
    Timeout = 103,
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
    /// Never fails: malformed lines become [`GatewayEvent::Respond`] with an
    /// `ERROR:101` (CiA 309-3 syntax error) frame.
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
            // BUGS.md 八L5: an unparseable sequence must not be echoed back
            // as a garbage token. CANopenNode (`CO_gateway_ascii.c:633-635`)
            // answers with its last valid sequence number (0 right after
            // reset); the frameless branch below already answers `[0]` — do
            // the same here.
            return self.frameless_error();
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
        // CiA 309-3 frames always answer with a sequence number; requests
        // whose `[seq]` header is missing/unparseable answer with seq 0
        // so the client's pending request can time out against something
        // (`BUGS.md` P3-9). Missing framing is a syntax error (101).
        GatewayEvent::Respond(format!("[0] ERROR:{}", GatewayErrorCode::Syntax.code()))
    }

    /// Parses the request body after the `[seq]` prefix.
    fn parse_request(&mut self, body: &str) -> Result<GatewayAction, GatewayErrorCode> {
        // CiA 309-3 (§5.4): a `#` comment may follow the command string
        // (`CO_gateway_ascii.c:170-171`). Strip an inline comment *token*
        // (`# ...`) before tokenising — a `#` glued to a value like
        // `vs a#b` stays part of the value (BUGS.md #30).
        let body = body
            .split_whitespace()
            .take_while(|t| !t.starts_with('#'))
            .collect::<Vec<_>>()
            .join(" ");
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
                let node = self.reject_broadcast(self.resolve_node(node_token)?)?;
                let index = parse_index(tokens.next().ok_or(GatewayErrorCode::Syntax)?)?;
                let sub = tokens.next().map_or(Ok(0), parse_sub)?;
                let ty = tokens.next().map_or(Ok(GwDataType::Uint(32)), |t| {
                    GwDataType::parse(t).ok_or(GatewayErrorCode::Syntax)
                })?;
                if tokens.next().is_some() {
                    return Err(GatewayErrorCode::Syntax); // trailing junk
                }
                Ok(GatewayAction::Read {
                    node,
                    index,
                    sub,
                    data_type: ty,
                })
            }
            "write" => {
                let node = self.reject_broadcast(self.resolve_node(node_token)?)?;
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
                // NMT verbs take no further arguments — reject trailing
                // tokens instead of silently swallowing them (BUGS.md #30).
                if tokens.next().is_some() {
                    return Err(GatewayErrorCode::Syntax);
                }
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
                if tokens.next().is_some() {
                    return Err(GatewayErrorCode::Syntax);
                }
                let command = match mode {
                    "node" => GatewayNmt::ResetNode,
                    "comm" => GatewayNmt::ResetCommunication,
                    _ => return Err(GatewayErrorCode::Syntax),
                };
                Ok(GatewayAction::Nmt { command, node })
            }
            "set" => {
                // `set` is a configuration verb, not an NMT target — a
                // leading node id ("4 set node 5") is a syntax error, not a
                // silently dropped token (BUGS.md #30).
                if node_token.is_some() {
                    return Err(GatewayErrorCode::Syntax);
                }
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
                    _ => Err(GatewayErrorCode::RequestNotSupported),
                }
            }
            "help" | "led" | "log" => Err(GatewayErrorCode::RequestNotSupported),
            // Unknown verb: CANopenNode answers `100` (request not
            // supported) — `CO_gateway_ascii.c:1531`.
            _ => Err(GatewayErrorCode::RequestNotSupported),
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

    /// SDO verbs have no broadcast semantics: node 0 is only valid for NMT.
    fn reject_broadcast(&self, node: u8) -> Result<u8, GatewayErrorCode> {
        if node == 0 {
            return Err(GatewayErrorCode::Syntax);
        }
        Ok(node)
    }

    fn resolve_node(&self, token: Option<&str>) -> Result<u8, GatewayErrorCode> {
        match token {
            Some(t) => t
                .parse::<u8>()
                .map_err(|_| GatewayErrorCode::Syntax)
                .and_then(|n| {
                    // 0 = broadcast: valid for NMT verbs only. SDO verbs
                    // reject it later when building their action.
                    (0..=127)
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
    ///
    /// Strings are printed verbatim (CiA 309-3 readback has no surrounding
    /// quotes; the `Value` Display uses Debug-quoting for them).
    #[must_use]
    pub fn format_value(sequence: u32, value: &Value) -> String {
        match value {
            // CR/LF inside a string would break the line-framed protocol
            // and let a stored value forge response lines — escape them so
            // the readback stays single-line (BUGS.md #30).
            Value::Str(s) => {
                let escaped = s.replace('\r', "\\r").replace('\n', "\\n");
                format!("[{sequence}] {escaped}")
            }
            other => format!("[{sequence}] {other}"),
        }
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
            // Strict boolean literals only (BUGS.md P3-46): "2" is a syntax
            // error, not `true`.
            match raw.trim() {
                "0" | "false" => Some(vec![0]),
                "1" | "true" => Some(vec![1]),
                _ => None,
            }
        }
        GwDataType::Str => Some(raw.as_bytes().to_vec()),
        GwDataType::Int(w) | GwDataType::Uint(w) => {
            // Encode at the *declared* width via `encode_as` so negatives
            // sign-extend (encode_le + zero-padding corrupted e.g. `i16 -1`
            // into 0xFF00 = +255, BUGS.md #1). Out-of-range literals are a
            // gateway-level syntax error, not something to ship and let the
            // remote abort on length (BUGS.md R10-P2-10).
            let Some(dt) = ty.to_core() else {
                return None; // parse() only yields 8/16/32/64 widths
            };
            let width = usize::from(w / 8);
            let mut buf = [0u8; 8];
            let value = match ty {
                GwDataType::Int(_) => Value::Int(raw.parse::<i64>().ok()?),
                // u64 directly: `u64::MAX` must be writable, an i64 parse
                // would reject it (BUGS.md #30).
                _ => Value::Uint(raw.parse::<u64>().ok()?),
            };
            value.encode_as(dt, &mut buf[..width]).ok()?;
            Some(buf[..width].to_vec())
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
            GatewayEvent::Respond("[2] ERROR:101".to_string())
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
        // BUGS.md L3: reset accepts a leading node id like the other NMT
        // verbs ("4 reset node" / "4 reset comm").
        assert!(matches!(
            gw.feed_line("[7] 4 reset node"),
            GatewayEvent::Request(GatewayRequest {
                action: GatewayAction::Nmt {
                    command: GatewayNmt::ResetNode,
                    node: 4
                },
                ..
            })
        ));
        assert!(matches!(
            gw.feed_line("[8] 4 reset comm"),
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
            GatewayEvent::Respond("[6] ERROR:101".to_string())
        );
    }

    #[test]
    fn malformed_frames_and_comments() {
        let mut gw = Gateway::new();
        assert_eq!(gw.feed_line(""), GatewayEvent::Ignore);
        assert_eq!(gw.feed_line("# comment"), GatewayEvent::Ignore);
        // Frameless garbage still yields a response (seq 0 per CiA 309-3)
        // so clients never hang.
        assert_eq!(
            gw.feed_line("garbage"),
            GatewayEvent::Respond("[0] ERROR:101".to_string())
        );
        assert_eq!(
            gw.feed_line("[abc] read 0x1000"),
            // BUGS.md 八L5: an unparseable sequence is not echoed back —
            // answer with sequence 0 like the frameless branch.
            GatewayEvent::Respond("[0] ERROR:101".to_string())
        );
        // Unknown verb → request not supported, CiA 309-3 code 100.
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
            "[3] ERROR:103"
        );
    }

    #[test]
    fn encode_payload_sign_extends_negative_ints() {
        // BUGS.md #1: negatives must be encoded at the declared width with
        // sign extension (encode_le + zero-padding produced +255 for `i16 -1`).
        assert_eq!(
            encode_payload(GwDataType::Int(16), "-1"),
            Some(vec![0xFF, 0xFF])
        );
        assert_eq!(encode_payload(GwDataType::Int(8), "-5"), Some(vec![0xFB]));
        assert_eq!(
            encode_payload(GwDataType::Int(32), "-300"),
            Some(vec![0xD4, 0xFE, 0xFF, 0xFF])
        );
        // u64::MAX is writable (BUGS.md #30: u64 parse, not i64).
        assert_eq!(
            encode_payload(GwDataType::Uint(64), "18446744073709551615"),
            Some(vec![0xFF; 8])
        );
        // Out-of-range literals remain a syntax error (R10-P2-10).
        assert_eq!(encode_payload(GwDataType::Int(8), "300"), None);
        assert_eq!(encode_payload(GwDataType::Uint(8), "-1"), None);
    }
}
