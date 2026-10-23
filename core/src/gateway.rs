//! CiA 309-3 ASCII gateway — minimal command slice, sans-I/O (docs/EXTENSIONS.md §B1).
//!
//! Wire format (CANopenNode `309/CO_gateway_ascii.h:67-133`):
//!
//! ```text
//! [1] 4 read 0x1017 0 u16        →  [1] OK | 500
//! [1] 4 read 0x1017 0            →  [1] F4 01   (hex is the default)
//! [1] 4 write 0x1017 0 u16 500   →  [1] OK
//! [1] 4 write 0x1008 0 vs "my device"  →  [1] OK
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
/// to internal state, `103` time-out, `105` no default node set, `106`
/// unsupported network, `107` unsupported node. SDO failures are transmitted
/// as their abort code in hex (`ERROR:0x…`), never folded into a numeric
/// code.
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
    /// A request omitted the node id and no default node was configured
    /// (CiA 309-3 `105`, CANopenNode `checkNetNode`).
    NoDefaultNodeSet = 105,
    /// Network id outside the supported range (CiA 309-3 `106`); this
    /// single-network gateway only enforces the 16-bit bound of the wire
    /// grammar (`CO_gateway_ascii.c:669-673`).
    UnsupportedNet = 106,
    /// Node id outside the range supported by the verb (CiA 309-3 `107`,
    /// CANopenNode `checkNetNode`; broadcast = 0 is unsupported for SDO).
    UnsupportedNode = 107,
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
#[non_exhaustive]
pub enum GwDataType {
    /// Raw byte transfer, rendered as space-separated hex (non-standard,
    /// CANopenNode's `hex`; the default when `read` omits the type).
    Hex,
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
    ///
    /// `Hex` maps to [`DataType::Domain`]: the raw-bytes type, so the SDO
    /// client performs no width validation and the gateway can dump any
    /// object (`G1`).
    #[must_use]
    pub const fn to_core(self) -> Option<DataType> {
        Some(match self {
            Self::Hex => DataType::Domain,
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

    /// Parses `hex`, `i16`, `u32`, `b`, `vs` style tokens.
    #[must_use]
    pub fn parse(token: &str) -> Option<Self> {
        let lower = token.to_ascii_lowercase();
        Some(match lower.as_str() {
            "hex" => Self::Hex,
            "b" => Self::Bool,
            "vs" | "str" => Self::Str,
            _ => {
                // Split at the *character* boundary after the first char:
                // `split_at(1)` panicked for multi-byte UTF-8 datatype tokens
                // ("é"), which are reachable from any network input through
                // `feed_line`'s "Never fails" contract (docs/DESIGN-DECISIONS.md GTW-01).
                let (kind, width) = match lower.char_indices().nth(1) {
                    Some((idx, _)) => lower.split_at(idx),
                    None => return None,
                };
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
#[non_exhaustive]
pub enum GatewayEvent {
    /// Execute this and emit [`Gateway::format_ok`] /
    /// [`Gateway::format_value`] / [`Gateway::format_error`] afterwards.
    Request(GatewayRequest),
    /// Respond immediately with these bytes (help text etc.). An empty
    /// string is the empty response frame for a blank or comment-only line
    /// (CANopenNode `responseWithEmpty`, `CO_gateway_ascii.c:522-526`, G4).
    Respond(String),
}

/// Longest command token the reference accepts: every token of the command
/// parser is read into a single `char tok[20]` (`CO_gateway_ascii.c:602`,
/// all `CO_fifo_readToken` call sites), which leaves room for 19 characters
/// plus the terminator. A longer token fills the buffer, which
/// `CO_fifo_readToken` reports as `err` — and it returns the token *empty* —
/// so the line is answered with 101 and nothing is dispatched
/// (`CO_fifo.c:419-436`; docs/DESIGN-DECISIONS.md GTW-25).
const MAX_TOKEN_LEN: usize = 19;

/// Longest literal a typed write accepts, or `None` for the streaming value
/// types that have no fixed token buffer.
///
/// CANopenNode's `CO_fifo_cpyTok2*` helpers read the value into their own
/// buffer — `char buf[15]` for the 8/16/32-bit types and `char buf[25]` for
/// the 64-bit ones (`CO_fifo.c:830,859,888,917,946,975,1004,1033`) — so a
/// longer literal is an `errTok` (101) there. `hex` and `vs` copy
/// character by character and impose no limit (`CO_fifo.c:1119,1231`).
const fn value_token_limit(ty: GwDataType) -> Option<usize> {
    match ty {
        GwDataType::Bool | GwDataType::Int(8..=32) | GwDataType::Uint(8..=32) => Some(14),
        GwDataType::Int(_) | GwDataType::Uint(_) => Some(24),
        GwDataType::Hex | GwDataType::Str => None,
    }
}

/// Cursor over a CiA 309-3 request body.
///
/// Tokens are whitespace-delimited and follow the reference tokenizer's
/// comment rule: a `#` *inside* a token terminates it and turns the rest of
/// the line into a comment (`CO_fifo.c:352-360`). The raw tail is left
/// untouched for the value parsers, because `vs` copies `#` literally
/// (`CO_fifo_cpyTok2Vs`, `CO_fifo.c:1266-1333`) while the hex/numeric
/// scanners treat it as a comment (`:1187-1188`) — docs/DESIGN-DECISIONS.md GTW-14.
struct RequestCursor<'a> {
    rest: &'a str,
    comment: bool,
}

impl<'a> RequestCursor<'a> {
    const fn new(body: &'a str) -> Self {
        Self {
            rest: body,
            comment: false,
        }
    }

    /// Next command/number token; `Ok(None)` once the line (or its comment)
    /// ends.
    ///
    /// `Err(Syntax)` for a token longer than [`MAX_TOKEN_LEN`], mirroring
    /// `CO_fifo_readToken`'s full-buffer error (docs/DESIGN-DECISIONS.md GTW-25).
    fn command_token(&mut self) -> Result<Option<&'a str>, GatewayErrorCode> {
        if self.comment {
            return Ok(None);
        }
        self.rest = self.rest.trim_start();
        if self.rest.is_empty() {
            return Ok(None);
        }
        if self.rest.starts_with('#') {
            self.comment = true;
            self.rest = "";
            return Ok(None);
        }
        let end = self
            .rest
            .find(char::is_whitespace)
            .unwrap_or(self.rest.len());
        let token = &self.rest[..end];
        if let Some(hash) = token.find('#') {
            self.comment = true;
            self.rest = "";
            // The comment delimiter ends the token *while* it is being
            // filled, so the part before `#` is what gets length-checked
            // (`CO_fifo.c:352-360`).
            return check_token_len(&token[..hash]);
        }
        self.rest = &self.rest[end..];
        check_token_len(token)
    }

    /// Text after the last consumed token, for the value parser.
    const fn tail(&self) -> &'a str {
        self.rest
    }

    /// Whether nothing but an optional comment remains.
    fn finished(&self) -> bool {
        let rest = self.rest.trim_start();
        rest.is_empty() || rest.starts_with('#')
    }
}

/// Truncates a value payload at the first `#` (comment) — the rule the
/// reference's hex/numeric scanners apply (`CO_fifo.c:1187-1188`); `vs`
/// values use [`parse_vs`] instead, which copies `#` verbatim.
fn cut_comment(raw: &str) -> &str {
    match raw.find('#') {
        Some(i) => &raw[..i],
        None => raw,
    }
}

/// The gateway parser/formatter state machine.
#[derive(Debug, Default)]
pub struct Gateway {
    /// Configured default node id (`set node <v>`); requests may omit it.
    default_node: Option<u8>,
    /// Last sequence number that was successfully extracted from a `[<seq>]`
    /// header. CANopenNode keeps `gtwa->sequence` as parser state and echoes
    /// it for frames whose header is missing or malformed
    /// (`CO_gateway_ascii.c:633-636,1541-1545`; docs/DESIGN-DECISIONS.md 审-G2). Starts at 0.
    last_sequence: u32,
}

impl Gateway {
    /// A gateway with no default node configured.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            default_node: None,
            last_sequence: 0,
        }
    }

    /// Current default node id.
    #[must_use]
    pub const fn default_node(&self) -> Option<u8> {
        self.default_node
    }

    /// Feeds one complete input line (without line terminator).
    ///
    /// Never fails: malformed lines become [`GatewayEvent::Respond`] with an
    /// `ERROR:101` (CiA 309-3 syntax error) frame. A blank or comment-only
    /// line answers with an empty response frame (`Respond(String::new())`,
    /// like CANopenNode's `responseWithEmpty`), so line-oriented clients
    /// waiting for an answer never hang (`G4`).
    #[must_use]
    pub fn feed_line(&mut self, line: &str) -> GatewayEvent {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            return GatewayEvent::Respond(String::new());
        }
        // Frame: [<seq>] <body>
        //
        // The reference reads **one** whitespace-delimited token, requires it
        // to start with `[` and to *end* with `]`, then strips that last `]`
        // and parses the rest with `getU32(..., &err)` — assigning
        // `gtwa->sequence` **before** it tests `err`
        // (`CO_gateway_ascii.c:611-636`, `getU32` at `:223-232`). That token
        // is read into a `char tok[20]`, so an over-long one is a token error
        // (docs/DESIGN-DECISIONS.md GTW-25).
        if !trimmed.starts_with('[') {
            return self.frameless_error();
        }
        let tok_end = trimmed.find(char::is_whitespace).unwrap_or(trimmed.len());
        let tok = &trimmed[..tok_end];
        if tok.len() > MAX_TOKEN_LEN {
            // `CO_fifo_readToken` returns an empty token and flags `err`
            // before the `[`/`]` checks run, so nothing is assigned and the
            // previous sequence is echoed (`CO_fifo.c:419-436`).
            return GatewayEvent::Respond(format!(
                "[{}] ERROR:{}",
                self.last_sequence,
                GatewayErrorCode::Syntax.code()
            ));
        }
        // docs/DESIGN-DECISIONS.md GTW-17/GTW-25: a token that does not end with `]` breaks
        // *before* the assignment (`CO_gateway_ascii.c:627-631`), so `[1]x …`
        // — and a bare `[1` — are answered with the previous sequence and are
        // not dispatched. A trailing `]` (as in `[1]]`) is part of the token,
        // so the value is assigned first and the trailing-garbage error is
        // reported after it.
        let Some(inner) = tok.strip_prefix('[').and_then(|t| t.strip_suffix(']')) else {
            return GatewayEvent::Respond(format!(
                "[{}] ERROR:{}",
                self.last_sequence,
                GatewayErrorCode::Syntax.code()
            ));
        };
        // docs/DESIGN-DECISIONS.md GTW-06 — three edges that follow from the reference's
        // `readToken` + `strtoul` pair and are mirrored here:
        //   `[]`    → `getU32("")` yields 0 *without* setting err (no digits
        //             were rejected), so the command still dispatches;
        //   `[1 2]` → the token stops at the space, so it never ends with `]`
        //             and the previous sequence stays current;
        //   `[-1]`  → `strtoul` wraps to 0xFFFF_FFFF and the command runs.
        let (seq_num, seq_err) = if inner.is_empty() {
            (0, false)
        } else {
            // `strtoul(token, &end, 0)` semantics (docs/DESIGN-DECISIONS.md GTW-06/GTW-17):
            // a numeric prefix is taken and negative values wrap; the value
            // is still assigned when trailing garbage sets `err`.
            parse_strtoul_prefix(inner).unwrap_or((0, true))
        };
        // docs/DESIGN-DECISIONS.md GTW-28: the reference breaks **before** assigning when the
        // token was closed by the command delimiter (a bare `[1]` line, no
        // frame terminator — `CO_gateway_ascii.c:611-615`, checked before the
        // assignment at `:633`), so the echoed sequence stays the previous one.
        // Assigning here poisoned every later frameless/malformed frame's echo.
        if !trimmed[tok_end..].trim().is_empty() {
            self.last_sequence = seq_num;
        }
        if seq_err {
            return GatewayEvent::Respond(format!(
                "[{seq_num}] ERROR:{}",
                GatewayErrorCode::Syntax.code()
            ));
        }
        let body = trimmed[tok_end..].trim();
        match self.parse_request(body) {
            Ok(action) => GatewayEvent::Request(GatewayRequest {
                sequence: seq_num,
                action,
            }),
            Err(code) => GatewayEvent::Respond(format!("[{seq_num}] ERROR:{}", code.code())),
        }
    }

    fn frameless_error(&self) -> GatewayEvent {
        // CiA 309-3 frames always answer with a sequence number; requests
        // whose `[seq]` header is missing or malformed answer with the last
        // successfully parsed sequence (0 right after construction) — the
        // reference keeps `gtwa->sequence` as parser state
        // (`CO_gateway_ascii.c:633-636,1541-1545`; docs/DESIGN-DECISIONS.md 审-G2). Missing
        // framing is a syntax error (101).
        GatewayEvent::Respond(format!(
            "[{}] ERROR:{}",
            self.last_sequence,
            GatewayErrorCode::Syntax.code()
        ))
    }

    /// Parses the request body after the `[seq]` prefix.
    fn parse_request(&mut self, body: &str) -> Result<GatewayAction, GatewayErrorCode> {
        // Comment handling lives in the token cursor for command/number
        // tokens and in the value parsers for the payload — `vs` keeps `#`
        // verbatim, everything else treats it as a comment (docs/DESIGN-DECISIONS.md GTW-14).
        let mut cursor = RequestCursor::new(body);
        let first = cursor.command_token()?.ok_or(GatewayErrorCode::Syntax)?;
        // Command tokens are case-insensitive: CANopenNode lowercases the
        // command token before dispatching (`CO_gateway_ascii.c:698`), so
        // `[1] READ 0x1000 0` is valid. The optional leading node id is
        // numerical and stays verbatim.
        let first_cmd = first.to_ascii_lowercase();

        // Optional `[[<net>] <node>]` prefix before the verb:
        // "4 read …", "2 4 read …". CANopenNode scans up to two leading
        // numeric tokens — one = node, two = net,node — and treats the first
        // non-numeric token as the command; a third numeric token is a
        // syntax error (`CO_gateway_ascii.c:638-661`). This gateway serves a
        // single network, so a syntactically valid net value is accepted and
        // ignored, exactly like a single-net CANopenNode build
        // (`checkNetNode`, `CO_gateway_ascii.c:236-257`; docs/DESIGN-DECISIONS.md 审-G3).
        let (node_token, verb): (Option<&str>, String) = if is_numeric_token(&first_cmd) {
            // `case 1` / `case 2` / `case 3` of the reference's `switch (i)`
            // (`CO_gateway_ascii.c:638-661`). N2: CANopenNode accepts both
            // `preop` and its full spelling (`:708-710`); 七G9: `r`/`w` are
            // aliases for `read`/`write` (`:702-705`).
            let second = cursor.command_token()?.ok_or(GatewayErrorCode::Syntax)?;
            if is_numeric_token(second) {
                // docs/DESIGN-DECISIONS.md GTW-05: the *prefix* numbers are validated before the
                // verb is dispatched, so the error code no longer depends on
                // whether the verb happens to be known
                // (`CO_gateway_ascii.c:666-694`: net > 0xFFFF → 106 first,
                // then node > 127 → 107). Doing the verb match first made
                // `[1] 300 banana` answer 100 and `[1] 300 set node 5` answer
                // 101. A syntactically valid net is accepted and ignored
                // (single-net build, 审-G3).
                let net = parse_prefix_number(first)?;
                let node = parse_prefix_number(second)?;
                if net > 0xFFFF {
                    return Err(GatewayErrorCode::UnsupportedNet);
                }
                if node > 127 {
                    return Err(GatewayErrorCode::UnsupportedNode);
                }
                let verb_tok = cursor.command_token()?.ok_or(GatewayErrorCode::Syntax)?;
                if is_numeric_token(verb_tok) {
                    // `case 3`: a third numeric token can never be a command.
                    return Err(GatewayErrorCode::Syntax);
                }
                (Some(second), verb_tok.to_ascii_lowercase())
            } else {
                // docs/DESIGN-DECISIONS.md GTW-21: a *single* number is the node id and is
                // range-checked before the verb dispatch, exactly like the
                // two-number case (`case 1`, `CO_gateway_ascii.c:666-675`).
                // It used to be handed to the verb instead, and `set`/`help`/
                // `log`/`led` dropped it entirely — so `[1] 300 set node 5`
                // really rewrote the default node to 5 and answered OK where
                // the reference answers 107.
                let node = parse_prefix_number(first)?;
                if node > 127 {
                    return Err(GatewayErrorCode::UnsupportedNode);
                }
                (Some(first), second.to_ascii_lowercase())
            }
        } else {
            // `case 0`: the first token *is* the command.
            //
            // docs/DESIGN-DECISIONS.md GTW-22: the old code always demanded a second token in
            // this branch, so an unknown verb with no argument (`[1] banana`),
            // or `[1] banana read 0x1000 0` (where `banana` was misread as a
            // node id), answered 101. The reference's prefix loop breaks at
            // `i == 0` and dispatches the token, and an unknown command
            // answers 100 (`CO_gateway_ascii.c:640-643,1531`).
            (None, first_cmd)
        };

        match verb.as_str() {
            "read" | "r" => {
                let node = self.reject_broadcast(self.resolve_node(node_token)?)?;
                let index = parse_index(cursor.command_token()?.ok_or(GatewayErrorCode::Syntax)?)?;
                // N4: CiA 309-3 / CANopenNode (`CO_gateway_ascii.c:863-870`)
                // require the subindex — omitting it is a syntax error, not a
                // silent sub 0.
                let sub = parse_sub(cursor.command_token()?.ok_or(GatewayErrorCode::Syntax)?)?;
                let ty = cursor.command_token()?.map_or(Ok(GwDataType::Hex), |t| {
                    GwDataType::parse(t).ok_or(GatewayErrorCode::Syntax)
                })?;
                if !cursor.finished() {
                    return Err(GatewayErrorCode::Syntax); // trailing junk
                }
                Ok(GatewayAction::Read {
                    node,
                    index,
                    sub,
                    data_type: ty,
                })
            }
            "write" | "w" => {
                let node = self.reject_broadcast(self.resolve_node(node_token)?)?;
                let index = parse_index(cursor.command_token()?.ok_or(GatewayErrorCode::Syntax)?)?;
                // docs/DESIGN-DECISIONS.md GTW-18: the subindex is *mandatory* like in `read`
                // — the reference reads it with `closed = 0`
                // (`CO_gateway_ascii.c:932-938`), so omitting it is a syntax
                // error rather than a silent sub 0.
                let sub = parse_sub(cursor.command_token()?.ok_or(GatewayErrorCode::Syntax)?)?;
                let ty_tok = cursor.command_token()?.ok_or(GatewayErrorCode::Syntax)?;
                let ty = GwDataType::parse(ty_tok).ok_or(GatewayErrorCode::Syntax)?;
                // Values are parsed from the untouched tail: `vs` keeps `#`,
                // the other types cut it as a comment (GTW-14).
                let raw = cursor.tail();
                let payload = if ty == GwDataType::Str {
                    encode_payload(ty, raw).ok_or(GatewayErrorCode::Syntax)?
                } else {
                    encode_payload(ty, cut_comment(raw)).ok_or(GatewayErrorCode::Syntax)?
                };
                // CiA 309-3 / CANopenNode reject a zero-length payload as a
                // syntax error (`CO_gateway_ascii.c:976-980`: `size == 0` →
                // 101), independently of the data type — this covers an empty
                // `hex` token list and a quoted empty `vs ""` alike
                // (docs/DESIGN-DECISIONS.md 审-G1).
                if payload.is_empty() {
                    return Err(GatewayErrorCode::Syntax);
                }
                Ok(GatewayAction::Write {
                    node,
                    index,
                    sub,
                    data_type: ty,
                    payload,
                })
            }
            "start" | "stop" | "preop" | "preoperational" => {
                let node = self.nmt_target(node_token, &mut cursor)?;
                // NMT verbs take no further arguments — reject trailing
                // tokens instead of silently swallowing them (docs/DESIGN-DECISIONS.md #30).
                if !cursor.finished() {
                    return Err(GatewayErrorCode::Syntax);
                }
                let command = match verb.as_str() {
                    "start" => GatewayNmt::Start,
                    "stop" => GatewayNmt::Stop,
                    _ => GatewayNmt::PreOperational,
                };
                Ok(GatewayAction::Nmt { command, node })
            }
            "reset" => {
                // The mode token is case-insensitive too
                // (`CO_gateway_ascii.c:1076`).
                let mode = cursor
                    .command_token()?
                    .ok_or(GatewayErrorCode::Syntax)?
                    .to_ascii_lowercase();
                let node = self.nmt_target(node_token, &mut cursor)?;
                if !cursor.finished() {
                    return Err(GatewayErrorCode::Syntax);
                }
                let command = match mode.as_str() {
                    "node" => GatewayNmt::ResetNode,
                    // CANopenNode accepts the full spelling too
                    // (`CO_gateway_ascii.c:1077-1086`, G3).
                    "comm" | "communication" => GatewayNmt::ResetCommunication,
                    _ => return Err(GatewayErrorCode::Syntax),
                };
                Ok(GatewayAction::Nmt { command, node })
            }
            "set" => {
                // docs/DESIGN-DECISIONS.md GTW-07: a leading node id is *accepted and ignored*
                // here — CANopenNode's prefix `case 1` only stores the node
                // for NMT verbs and the `set` branch never reads it
                // (`CO_gateway_ascii.c:666-675,769-792`), so `4 set node 5`
                // answers OK there.
                // Sub-command tokens are case-insensitive
                // (`CO_gateway_ascii.c:749`).
                let key = cursor
                    .command_token()?
                    .ok_or(GatewayErrorCode::Syntax)?
                    .to_ascii_lowercase();
                // docs/DESIGN-DECISIONS.md GTW-16: an unknown key is answered with 100 *before*
                // the value is read — CANopenNode matches the key first and
                // jumps straight to `RequestNotSupported`
                // (`CO_gateway_ascii.c:835-838`), so a missing or malformed
                // value must not change the code to 101.
                if key != "node" {
                    return Err(GatewayErrorCode::RequestNotSupported);
                }
                let val = cursor.command_token()?.ok_or(GatewayErrorCode::Syntax)?;
                // docs/DESIGN-DECISIONS.md GTW-03: the value token must end the line. The
                // reference reads it with `closed = 1` (`CO_gateway_ascii.c:
                // 769-792`), and a trailing token makes `CO_fifo_readToken`
                // set err (`CO_fifo.c:416-430`) → 101; the old code answered
                // OK for `[1] set node 5 garbage`.
                if !cursor.finished() {
                    return Err(GatewayErrorCode::Syntax);
                }
                let n = parse_u32_auto(val).ok_or(GatewayErrorCode::Syntax)?;
                if !(1..=127).contains(&n) {
                    return Err(GatewayErrorCode::Syntax);
                }
                self.default_node = Some(n as u8);
                Ok(GatewayAction::SetNode(n as u8))
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
        cursor: &mut RequestCursor<'_>,
    ) -> Result<u8, GatewayErrorCode> {
        if let Some(t) = node_token {
            return self.resolve_node(Some(t));
        }
        match cursor.command_token()? {
            None => self.resolve_node(None),
            Some("node") => self.resolve_node(Some(
                cursor.command_token()?.ok_or(GatewayErrorCode::Syntax)?,
            )),
            Some(t) => self.resolve_node(Some(t)),
        }
    }

    /// SDO verbs have no broadcast semantics: node 0 is only valid for NMT
    /// (CANopenNode passes `NodeMin=1` for SDO, `NodeMin=0` for NMT, so 0
    /// maps to `107` on this path — `CO_gateway_ascii.c:848,917,998`).
    fn reject_broadcast(&self, node: u8) -> Result<u8, GatewayErrorCode> {
        if node == 0 {
            return Err(GatewayErrorCode::UnsupportedNode);
        }
        Ok(node)
    }

    fn resolve_node(&self, token: Option<&str>) -> Result<u8, GatewayErrorCode> {
        match token {
            Some(t) => {
                // docs/DESIGN-DECISIONS.md G6: CANopenNode `checkNetNode`
                // (`CO_gateway_ascii.c:240-245`) distinguishes a missing
                // default node (105) from an out-of-range id (107); the
                // range check happens after numeric parsing (`:669-671`),
                // so non-numeric tokens stay syntax errors. The literal may
                // carry the `0x` prefix (docs/DESIGN-DECISIONS.md 五G10).
                let n = parse_u32_auto(t).ok_or(GatewayErrorCode::Syntax)?;
                if n > 127 {
                    return Err(GatewayErrorCode::UnsupportedNode);
                }
                // 0 = broadcast: valid for NMT verbs only. SDO verbs reject
                // it later when building their action.
                Ok(n as u8)
            }
            None => self.default_node.ok_or(GatewayErrorCode::NoDefaultNodeSet),
        }
    }

    /// `[<seq>] OK`
    #[must_use]
    pub fn format_ok(sequence: u32) -> String {
        format!("[{sequence}] OK")
    }

    /// `[<seq>] <value>`
    ///
    /// Strings follow CANopenNode's `CO_fifo_readVs2a` (`CO_fifo.c:714-740`,
    /// type table `:318`): always wrapped in `"`, with embedded `"` doubled.
    /// A client that tokenises on whitespace would otherwise shred multi-word
    /// values (docs/DESIGN-DECISIONS.md GTW-02). CR/LF stay escaped as `\r`/`\n` instead of
    /// being skipped like the reference does — a raw CR/LF would break this
    /// line-framed transport and let a stored value forge response lines
    /// (`docs/DESIGN-DECISIONS.md` #30). Booleans render as `1`/`0`, matching
    /// `CO_fifo_readU82a` (`%" PRIu8`, `CO_fifo.c:462-477`; type table
    /// `:303`) — `true`/`false` is not parseable by a CiA 309 client. Byte
    /// blobs (`hex` reads) are space-separated uppercase hex with no leading
    /// space — `CO_fifo_readHex2a` (`CO_fifo.c:686-696`, G1).
    #[must_use]
    pub fn format_value(sequence: u32, value: &Value) -> String {
        match value {
            Value::Str(s) => {
                let escaped = s
                    .replace('"', "\"\"")
                    .replace('\r', "\\r")
                    .replace('\n', "\\n");
                format!("[{sequence}] \"{escaped}\"")
            }
            Value::Bool(b) => format!("[{sequence}] {}", u8::from(*b)),
            Value::Bytes(b) => {
                let hex = b
                    .iter()
                    .map(|byte| format!("{byte:02X}"))
                    .collect::<Vec<_>>()
                    .join(" ");
                format!("[{sequence}] {hex}")
            }
            other => format!("[{sequence}] {other}"),
        }
    }

    /// `[<seq>] ERROR:<code>` — numeric CiA code or hex SDO abort.
    ///
    /// The SDO abort spelling mirrors CANopenNode `responseWithErrorSDO`
    /// (`CO_gateway_ascii.c:502-507`): `0x%08X` — uppercase, zero-padded to
    /// eight digits (docs/DESIGN-DECISIONS.md G8).
    #[must_use]
    pub fn format_error(sequence: u32, error: &GatewayFailure) -> String {
        match error {
            GatewayFailure::Code(c) => format!("[{sequence}] ERROR:{}", c.code()),
            GatewayFailure::SdoAborted(a) => {
                format!("[{sequence}] ERROR:0x{:08X}", a.to_u32())
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

/// Enforces the reference's fixed-size command-token buffer
/// ([`MAX_TOKEN_LEN`]).
fn check_token_len(token: &str) -> Result<Option<&str>, GatewayErrorCode> {
    if token.len() > MAX_TOKEN_LEN {
        Err(GatewayErrorCode::Syntax)
    } else {
        Ok(Some(token))
    }
}

/// Whether the reference would treat a leading token as one of the optional
/// `[[<net>] <node>]` numbers: it tests the *first character* with
/// `isdigit` (`CO_gateway_ascii.c:647`), so `0x10` and `12` count while
/// `-1` does not (that one is dispatched as a command).
fn is_numeric_token(tok: &str) -> bool {
    tok.starts_with(|c: char| c.is_ascii_digit())
}

/// Parses one `[[<net>] <node>]` prefix number.
///
/// The reference uses `getU32(tok, 0, 0xFFFFFFFF, &err)`
/// (`CO_gateway_ascii.c:648,656`), i.e. a malformed literal is a token error
/// (101) while the range checks live in the caller's `switch (i)`. This
/// parser keeps decimal for unprefixed literals on purpose (docs/DESIGN-DECISIONS.md 五G10),
/// so only the `0x`/`0X` forms carry a radix.
fn parse_prefix_number(tok: &str) -> Result<u32, GatewayErrorCode> {
    parse_u32_auto(tok).ok_or(GatewayErrorCode::Syntax)
}

fn parse_index(tok: &str) -> Result<u16, GatewayErrorCode> {
    let t = tok.trim();
    if let Some(hex) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        u16::from_str_radix(hex, 16).map_err(|_| GatewayErrorCode::Syntax)
    } else {
        t.parse::<u16>().map_err(|_| GatewayErrorCode::Syntax)
    }
}

/// Parses an unsigned token accepting the `0x`/`0X` hex prefix like
/// CANopenNode's `getU32` (`strtoul(token, &sRet, 0)` with trailing-garbage
/// rejection, `CO_gateway_ascii.c:223-231`). The C reference also accepts
/// octal through base 0; this parser deliberately keeps decimal for
/// unprefixed tokens (consistent with `parse_index` and less surprising for
/// node/subindex literals — docs/DESIGN-DECISIONS.md 五G10).
fn parse_u32_auto(tok: &str) -> Option<u32> {
    let t = tok.trim();
    if let Some(hex) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        u32::from_str_radix(hex, 16).ok()
    } else {
        t.parse::<u32>().ok()
    }
}

/// C `strtoul(token, &end, 0)` prefix with the stack's explicit overflow
/// policy: optional sign, base-0 prefixes (`0x` hex / leading `0` octal /
/// decimal), stop at the first invalid character, negatives wrapped like
/// `strtoul` (`-1` → 0xFFFF_FFFF), and **positive** values above `u32::MAX`
/// saturated to `u32::MAX` (not truncated to zero).
///
/// Returns `(value, had_trailing_garbage)`; `None` when no digit was
/// consumed at all (`strtoul` yields 0 with `end == start`, which the
/// reference reports as `err`). Used for the `[<seq>]` header
/// (`docs/DESIGN-DECISIONS.md` GTW-06/GTW-17).
fn parse_strtoul_prefix(tok: &str) -> Option<(u32, bool)> {
    let s = tok.trim();
    let (negative, rest) = match s.as_bytes().first() {
        Some(b'+') => (false, &s[1..]),
        Some(b'-') => (true, &s[1..]),
        _ => (false, s),
    };
    let (radix, digits) =
        if let Some(hex) = rest.strip_prefix("0x").or_else(|| rest.strip_prefix("0X")) {
            (16u32, hex)
        } else if rest.len() > 1 && rest.starts_with('0') {
            (8u32, &rest[1..])
        } else {
            (10u32, rest)
        };
    let end = digits
        .find(|c: char| !c.is_digit(radix))
        .unwrap_or(digits.len());
    if end == 0 {
        return None;
    }
    let magnitude = u64::from_str_radix(&digits[..end], radix).ok()?;
    let value = if negative {
        magnitude.wrapping_neg() as u32
    } else {
        magnitude.min(u64::from(u32::MAX)) as u32
    };
    Some((value, end != digits.len()))
}

/// Parses a typed-write integer literal with the C base-0 grammar that
/// CANopenNode's `CO_fifo_cpyTok2*` get from `strtoul`/`strtoll`/`strtoull`
/// (`CO_fifo.c:840,869,896,927,956,985,1014,1043`): an optional `+`/`-` sign, `0x`/
/// `0X` hexadecimal, a leading `0` selecting octal, otherwise decimal. The
/// whole token must be consumed — trailing garbage is a value error, exactly
/// like the reference's `sRet != strchr(buf, '\0')` check. The magnitude is
/// accumulated in `i128` so `u64::MAX` and `i64::MIN` both parse.
///
/// Returns `(negative, magnitude)`. Previously the payload tokens were parsed
/// with Rust's decimal-only `str::parse`, so `write 0x1017 0 u16 0x1F4` — a
/// literal CANopenNode accepts — answered `ERROR:101` (`docs/DESIGN-DECISIONS.md` 五G10 was
/// recorded for node/sub/index/sequence only).
fn parse_c_int_token(tok: &str) -> Option<(bool, i128)> {
    let t = tok.trim();
    let (negative, body) = match t.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, t.strip_prefix('+').unwrap_or(t)),
    };
    // A bare sign, or a second sign after it (`+-5`), is not a number.
    if body.is_empty() || body.starts_with('+') || body.starts_with('-') {
        return None;
    }
    let hex = body.strip_prefix("0x").or_else(|| body.strip_prefix("0X"));
    let (radix, digits) = if let Some(hex) = hex {
        (16u32, hex)
    } else if body.len() > 1 && body.starts_with('0') {
        // C: a leading 0 (not the number 0 itself) selects base 8.
        (8u32, &body[1..])
    } else {
        (10u32, body)
    };
    if digits.is_empty() {
        return None;
    }
    let magnitude = i128::from_str_radix(digits, radix).ok()?;
    Some((negative, magnitude))
}

fn parse_sub(tok: &str) -> Result<u8, GatewayErrorCode> {
    let n = parse_u32_auto(tok).ok_or(GatewayErrorCode::Syntax)?;
    u8::try_from(n).map_err(|_| GatewayErrorCode::Syntax)
}

/// Encodes a typed literal into wire bytes.
#[must_use]
pub fn encode_payload(ty: GwDataType, raw: &str) -> Option<Vec<u8>> {
    match ty {
        GwDataType::Hex => parse_hex_bytes(raw),
        GwDataType::Bool => {
            // CANopenNode `CO_fifo_cpyTok2U8` (`CO_fifo.c:828-856`) reads the
            // token into `char buf[15]` and parses it with `strtoul(_, _, 0)`,
            // accepting any value that fits the byte — so `b 2` is a valid
            // write (docs/DESIGN-DECISIONS.md 五G12). `false`/`true` stay accepted as a
            // convenience spelling the reference does not know.
            //
            // docs/DESIGN-DECISIONS.md GTW-24: the base-0 grammar was documented but not used
            // here (`parse_u32_auto` is decimal + `0x` only), so `b 010`
            // wrote 10 where the reference writes 8. GTW-25 adds the token
            // length cap of that `buf[15]`.
            match raw.trim() {
                "false" => Some(vec![0]),
                "true" => Some(vec![1]),
                other => {
                    if other.len() > value_token_limit(GwDataType::Bool).unwrap_or(0) {
                        return None;
                    }
                    let (negative, magnitude) = parse_c_int_token(other)?;
                    let n = if negative {
                        // `strtoul` wraps a negative literal modulo 2**32 and
                        // the result is range-checked against `UINT8_MAX`
                        // (`CO_fifo.c:838-841`), so `-1` is a value error.
                        u32::try_from(magnitude.unsigned_abs()).ok()?.wrapping_neg()
                    } else {
                        u32::try_from(magnitude).ok()?
                    };
                    u8::try_from(n).ok().map(|b| vec![b])
                }
            }
        }
        GwDataType::Str => parse_vs(raw).map(String::into_bytes),
        GwDataType::Int(w) | GwDataType::Uint(w) => {
            // Encode at the *declared* width via `encode_as` so negatives
            // sign-extend (encode_le + zero-padding corrupted e.g. `i16 -1`
            // into 0xFF00 = +255, docs/DESIGN-DECISIONS.md #1). Out-of-range literals are a
            // gateway-level syntax error, not something to ship and let the
            // remote abort on length (docs/DESIGN-DECISIONS.md R10-P2-10).
            let Some(dt) = ty.to_core() else {
                return None; // parse() only yields 8/16/32/64 widths
            };
            let width = usize::from(w / 8);
            let mut buf = [0u8; 8];
            // Base-0 literal grammar, like the reference tokenizers
            // (`CO_fifo_cpyTok2U8..I64`, `CO_fifo.c:840-1043`).
            //
            // docs/DESIGN-DECISIONS.md GTW-25: the reference reads the literal into a fixed
            // `buf[15]`/`buf[25]` per width, so an over-long token is an
            // `errTok` there (101) instead of a value that gets shipped.
            if raw.trim().len() > value_token_limit(ty).unwrap_or(usize::MAX) {
                return None;
            }
            let (negative, magnitude) = parse_c_int_token(raw)?;
            let value = match ty {
                GwDataType::Int(_) => {
                    let signed = if negative { -magnitude } else { magnitude };
                    Value::Int(i64::try_from(signed).ok()?)
                }
                // Unsigned widths keep `u64` directly: `u64::MAX` must be
                // writable, an i64 parse would reject it (docs/DESIGN-DECISIONS.md #30). A
                // leading `-` wraps modulo 2**64 like `strtoul`/`strtoull`;
                // `encode_as` below then re-applies the declared width's
                // range check.
                //
                // docs/DESIGN-DECISIONS.md GTW-20: the reference truncates the wrapped 64-bit
                // `strtoul` result to the declared width *before* the range
                // check (`CO_fifo_cpyTok2U32` has no check at all,
                // `CO_fifo.c:887-905`), so `u32 -1` is the legal value
                // 0xFFFF_FFFF while `u8/u16 -1` stay value errors.
                _ => {
                    let mag = u64::try_from(magnitude).ok()?;
                    Value::Uint(if negative && w <= 32 {
                        u64::from((mag as u32).wrapping_neg())
                    } else if negative {
                        mag.wrapping_neg()
                    } else {
                        mag
                    })
                }
            };
            value.encode_as(dt, &mut buf[..width]).ok()?;
            Some(buf[..width].to_vec())
        }
    }
}

/// Parses a `vs` literal: a double-quoted string with `""` as an escaped
/// quote (CANopenNode `CO_fifo_cpyTok2Vs`, `CO_fifo.c:1266-1333`), or a bare
/// single word without quotes.
///
/// `#` is an ordinary visible character *inside* a `vs` value and is copied
/// verbatim (the reference's `cpyTok2Vs` only routes it to `CO_fifo_putc`);
/// comment semantics apply only *after* the value, and only a comment may
/// follow it (`docs/DESIGN-DECISIONS.md` GTW-14). An unquoted multi-word value, an
/// unterminated quote or trailing garbage after the closing quote is a
/// syntax error (`G2`).
///
/// Two `#` edges are **not** free-form, though (docs/DESIGN-DECISIONS.md GTW-23):
///
/// * a value that *starts* with `#` is a syntax error, because the reference
///   inspects the first character with `CO_fifo_trimSpaces` before entering
///   the value state, and a comment delimiter there sets `errTok`
///   (`CO_fifo.c:1246-1252`);
/// * a closing quote must be followed by whitespace (or end of line) — a
///   comment may only follow that whitespace. `"a" #c` is accepted, `"a"#c`
///   is not, because `#` is a graph character and the reference rejects any
///   graph character there (`CO_fifo.c:1314-1332`).
fn parse_vs(raw: &str) -> Option<String> {
    let raw = raw.trim_start();
    let Some(body) = raw.strip_prefix('"') else {
        // Bare single word without quotes: `#` stays part of the value.
        let end = raw.find(char::is_whitespace).unwrap_or(raw.len());
        let word = &raw[..end];
        if word.is_empty() || word.contains('"') || word.starts_with('#') {
            return None;
        }
        let rest = raw[end..].trim_start();
        return (rest.is_empty() || rest.starts_with('#')).then(|| word.to_string());
    };
    let mut out = String::new();
    let mut chars = body.char_indices();
    while let Some((i, c)) = chars.next() {
        if c == '"' {
            if body[i + 1..].starts_with('"') {
                // `""` inside the string is an escaped double quote.
                out.push('"');
                let _ = chars.next();
                continue;
            }
            // Closing quote: end of line, or whitespace followed by at most a
            // comment (`CO_fifo.c:1314-1332`; docs/DESIGN-DECISIONS.md GTW-23).
            let rest = &body[i + 1..];
            if rest.is_empty() {
                return Some(out);
            }
            if !rest.starts_with(char::is_whitespace) {
                return None; // `"a"#c` — a graph character right after the quote
            }
            let rest = rest.trim_start();
            return (rest.is_empty() || rest.starts_with('#')).then_some(out);
        }
        out.push(c);
    }
    None // unterminated string
}

/// Parses `hex` literals: hex digits may be space separated or contiguous,
/// each pair is one byte, and a `#` starts a comment (CANopenNode
/// `CO_fifo_cpyTok2Hex`, `CO_fifo.c:1119-1180`). A non-hex character
/// invalidates the whole literal (`G1`).
///
/// docs/DESIGN-DECISIONS.md GTW-26: **whitespace terminates a digit pair**. The reference writes
/// the pending nibble as its own byte when it meets a space or a delimiter
/// (`step == 1U` in the final `else`, `CO_fifo.c:1188-1203`), so `hex 0 1 2`
/// yields three bytes `00 01 02`. Skipping the separator while keeping the
/// nibble paired the first digit of the *next* group, which silently wrote the
/// wrong data for any payload printed with single-digit byte groups
/// (`format(b,'X')` instead of `'02X'`) — the master got `OK` and the object
/// held a different value. A single trailing digit at end of input is carried
/// over by the reference in `dest->aux` for a continued token
/// (`CO_fifo.c:1223-1227`); we emit it as its own byte, which is the
/// recoverable reading of the same shape.
fn parse_hex_bytes(raw: &str) -> Option<Vec<u8>> {
    let mut out = Vec::new();
    let mut high: Option<u8> = None;
    for c in raw.chars() {
        if c.is_whitespace() {
            // `CO_fifo.c:1188-1203`: a pending nibble becomes its own byte.
            if let Some(h) = high.take() {
                out.push(h);
            }
            continue;
        }
        if c == '#' {
            break;
        }
        let digit = u8::try_from(c.to_digit(16)?).ok()?;
        match high.take() {
            None => high = Some(digit),
            Some(h) => out.push((h << 4) | digit),
        }
    }
    if let Some(h) = high {
        out.push(h);
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// docs/DESIGN-DECISIONS.md GTW-01: a multi-byte UTF-8 datatype token must not panic
    /// `GwDataType::parse` — the parser contract ("Never fails") only allows
    /// an error response.
    #[test]
    fn non_ascii_datatype_token_is_rejected_not_panicked() {
        assert_eq!(GwDataType::parse("é"), None);
        assert_eq!(GwDataType::parse("ü16"), None);
        assert_eq!(GwDataType::parse("u16"), Some(GwDataType::Uint(16)));
        assert_eq!(GwDataType::parse(""), None);

        let mut gw = Gateway::new();
        assert_eq!(
            gw.feed_line("[1] 4 read 0x1000 0 é"),
            GatewayEvent::Respond("[1] ERROR:101".to_string())
        );
    }

    /// docs/DESIGN-DECISIONS.md GTW-02: `vs` readback is always quoted and doubles embedded
    /// quotes (`CO_fifo_readVs2a`, `CO_fifo.c:714-740`); `b` renders `1`/`0`
    /// (`CO_fifo_readU82a`, `CO_fifo.c:462-477`).
    #[test]
    fn readback_quotes_strings_and_prints_booleans_as_digits() {
        assert_eq!(
            Gateway::format_value(3, &Value::Str("a b".into())),
            "[3] \"a b\""
        );
        assert_eq!(
            Gateway::format_value(3, &Value::Str("say \"hi\"".into())),
            "[3] \"say \"\"hi\"\"\""
        );
        assert_eq!(
            Gateway::format_value(4, &Value::Str("x\ry\nz".into())),
            "[4] \"x\\ry\\nz\"",
            "CR/LF stay escaped so a stored value cannot forge response lines"
        );
        assert_eq!(Gateway::format_value(5, &Value::Bool(true)), "[5] 1");
        assert_eq!(Gateway::format_value(5, &Value::Bool(false)), "[5] 0");
        assert_eq!(Gateway::format_value(6, &Value::Uint(500)), "[6] 500");
    }

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
        // Missing default → 105 (no default node set), CANopenNode
        // `checkNetNode` / `CO_gateway_ascii.c:240-242` (docs/DESIGN-DECISIONS.md G6).
        assert_eq!(
            gw.feed_line("[2] read 0x1000"),
            GatewayEvent::Respond("[2] ERROR:105".to_string())
        );
        assert_eq!(
            gw.feed_line("[2] start"),
            GatewayEvent::Respond("[2] ERROR:105".to_string())
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
        // Now the bare read (with its mandatory subindex) resolves to node 5.
        match gw.feed_line("[4] read 0x1000 0") {
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

    /// docs/DESIGN-DECISIONS.md G6: an out-of-range / unsupported node is `107`, not folded
    /// into the syntax code (CANopenNode `checkNetNode`; SDO rejects the NMT
    /// broadcast id 0 with the same code, `NodeMin=1`).
    #[test]
    fn unsupported_node_is_reported_as_107() {
        let mut gw = Gateway::new();
        for (seq, line) in [
            (1, "[1] 128 read 0x1000 0"),
            (2, "[2] 9999 start"),
            (3, "[3] 0 read 0x1000 0"),
            (4, "[4] 255 write 0x1000 0 hex 00"),
        ] {
            assert_eq!(
                gw.feed_line(line),
                GatewayEvent::Respond(format!("[{seq}] ERROR:107")),
                "line {line:?}"
            );
        }
        // NMT broadcast (node 0) stays valid.
        assert!(matches!(
            gw.feed_line("[5] start 0"),
            GatewayEvent::Request(_)
        ));
        // docs/DESIGN-DECISIONS.md GTW-22: a non-numeric *first* token is the command, not a
        // node id — the reference's prefix loop breaks at `i == 0`
        // (`isdigit('b') == 0`, `CO_gateway_ascii.c:647`) and dispatches
        // `banana`, and an unknown command answers 100 (`:1531`). The extra
        // `read 0x1000 0` is never looked at.
        assert_eq!(
            gw.feed_line("[6] banana read 0x1000 0"),
            GatewayEvent::Respond("[6] ERROR:100".to_string())
        );
        // A numeric-but-malformed prefix number is still a token error: the
        // reference's `getU32` rejects trailing garbage (`CO_gateway_ascii.c:
        // 223-232`).
        assert_eq!(
            gw.feed_line("[7] 12ab read 0x1000 0"),
            GatewayEvent::Respond("[7] ERROR:101".to_string())
        );
    }

    /// docs/DESIGN-DECISIONS.md GTW-21: the optional `[[<net>] <node>]` prefix is range-checked
    /// **before** the verb is dispatched, for *every* verb
    /// (`CO_gateway_ascii.c:666-694`, `case 1`). `set` used to drop the token
    /// entirely, so a malformed one really rewrote the default node.
    #[test]
    fn single_numeric_prefix_is_validated_before_the_verb() {
        let mut gw = Gateway::new();
        assert!(matches!(
            gw.feed_line("[1] set node 4"),
            GatewayEvent::Request(_)
        ));
        for (seq, line) in [
            (2, "[2] 300 set node 5"),
            (3, "[3] 300 help"),
            (4, "[4] 300 led"),
            (5, "[5] 300 log"),
            (6, "[6] 0x1FF start"),
        ] {
            assert_eq!(
                gw.feed_line(line),
                GatewayEvent::Respond(format!("[{seq}] ERROR:107")),
                "line {line:?}"
            );
        }
        // The rejected frames had **no** side effect: the default node is
        // still 4, not 5.
        assert_eq!(gw.default_node(), Some(4));
        // A valid id is still accepted and ignored by `set`
        // (`CO_gateway_ascii.c:666-675`; docs/DESIGN-DECISIONS.md GTW-07).
        assert!(matches!(
            gw.feed_line("[7] 6 set node 4"),
            GatewayEvent::Request(_)
        ));
        assert_eq!(gw.default_node(), Some(4));
    }

    /// docs/DESIGN-DECISIONS.md GTW-22: an unknown verb reaches the dispatcher's
    /// `RequestNotSupported` (100) instead of dying on a missing second
    /// token (101) — CANopenNode's prefix loop stops at the first non-numeric
    /// token (`CO_gateway_ascii.c:638-661`) and an unknown command answers
    /// 100 (`:1531`).
    #[test]
    fn unknown_verb_without_argument_answers_100() {
        let mut gw = Gateway::new();
        for (seq, line) in [
            (1, "[1] banana"),
            (2, "[2] 4 banana"),
            (3, "[3] banana read 0x1000 0"),
        ] {
            assert_eq!(
                gw.feed_line(line),
                GatewayEvent::Respond(format!("[{seq}] ERROR:100")),
                "line {line:?}"
            );
        }
        // `led`/`log`/`help` remain "not supported" (100) in every position.
        for (seq, line) in [(4, "[4] led 0"), (5, "[5] log node"), (6, "[6] 4 help")] {
            assert_eq!(
                gw.feed_line(line),
                GatewayEvent::Respond(format!("[{seq}] ERROR:100")),
                "line {line:?}"
            );
        }
    }

    /// docs/DESIGN-DECISIONS.md GTW-23: `#` is free-form only *inside* a `vs` value. A value
    /// starting with `#` is a comment (syntax error, `CO_fifo.c:1246-1252`),
    /// and a closing quote must be followed by whitespace before a comment
    /// (`"a" #c` ok, `"a"#c` error, `CO_fifo.c:1314-1332`).
    #[test]
    fn vs_rejects_leading_hash_and_hash_right_after_the_closing_quote() {
        assert_eq!(parse_vs("#abc"), None);
        assert_eq!(parse_vs("# abc"), None);
        assert_eq!(parse_vs("\"a\"#c"), None);
        assert_eq!(parse_vs("\"a\" #c"), Some("a".to_string()));
        assert_eq!(parse_vs("a#b"), Some("a#b".to_string()));
        assert_eq!(parse_vs("a #c"), Some("a".to_string()));
        assert_eq!(parse_vs("\"a#b\" #c"), Some("a#b".to_string()));
        // And through the request path: 101, no action.
        let mut gw = Gateway::new();
        let _ = gw.feed_line("[1] set node 4");
        for (seq, line) in [
            (2, "[2] 4 write 0x2000 0 vs #abc"),
            (3, "[3] 4 write 0x2000 0 vs \"a\"#c"),
        ] {
            assert_eq!(
                gw.feed_line(line),
                GatewayEvent::Respond(format!("[{seq}] ERROR:101")),
                "line {line:?}"
            );
        }
    }

    /// docs/DESIGN-DECISIONS.md GTW-24: the `b` literal follows the C base-0 grammar like every
    /// other typed write (`strtoul(_, _, 0)`, `CO_fifo.c:840`), so a leading
    /// zero is octal.
    #[test]
    fn bool_literals_use_the_c_base_zero_grammar() {
        assert_eq!(encode_payload(GwDataType::Bool, "010"), Some(vec![0x08]));
        assert_eq!(encode_payload(GwDataType::Bool, "0x10"), Some(vec![0x10]));
        assert_eq!(encode_payload(GwDataType::Bool, "8"), Some(vec![0x08]));
        assert_eq!(encode_payload(GwDataType::Bool, "2"), Some(vec![0x02]));
        assert_eq!(encode_payload(GwDataType::Bool, "255"), Some(vec![0xFF]));
        // Range and grammar errors stay errors (`CO_fifo.c:838-841`).
        assert_eq!(encode_payload(GwDataType::Bool, "256"), None);
        assert_eq!(encode_payload(GwDataType::Bool, "-1"), None);
        assert_eq!(encode_payload(GwDataType::Bool, "1x"), None);
        // The convenience spellings still work.
        assert_eq!(encode_payload(GwDataType::Bool, "true"), Some(vec![1]));
        assert_eq!(encode_payload(GwDataType::Bool, "false"), Some(vec![0]));
    }

    /// docs/DESIGN-DECISIONS.md GTW-25: the reference reads every command token into a
    /// `char tok[20]` and flags a full buffer as a token error
    /// (`CO_gateway_ascii.c:602`, `CO_fifo.c:419-436`); typed value literals
    /// use `buf[15]`/`buf[25]` per width (`CO_fifo.c:830,917`).
    #[test]
    fn over_long_tokens_are_syntax_errors() {
        // The token *includes* the brackets: 19 bytes fit, 20 do not.
        let seq19 = "12345678901234567";
        let seq20 = "123456789012345678";
        assert_eq!(seq19.len() + 2, MAX_TOKEN_LEN);
        assert_eq!(seq20.len() + 2, MAX_TOKEN_LEN + 1);
        let mut gw = Gateway::new();
        let _ = gw.feed_line("[1] set node 4");
        let long_header = format!("[{seq20}] 4 read 0x1000 0");
        assert_eq!(
            gw.feed_line(&long_header),
            GatewayEvent::Respond("[1] ERROR:101".to_string()),
            "an over-long sequence token is rejected with the *previous* sequence"
        );
        // A 19-byte sequence header is fine.
        assert!(matches!(
            gw.feed_line(&format!("[{seq19}] 4 read 0x1000 0")),
            GatewayEvent::Request(_)
        ));
        // An over-long command token is a token error, too.
        let long_verb = "r".repeat(MAX_TOKEN_LEN + 1);
        assert_eq!(
            gw.feed_line(&format!("[2] {long_verb}")),
            GatewayEvent::Respond("[2] ERROR:101".to_string())
        );
        // …and so is an over-long index/subindex/type token.
        let long_index = format!("0x{}", "1".repeat(MAX_TOKEN_LEN));
        assert_eq!(
            gw.feed_line(&format!("[3] 4 read {long_index} 0")),
            GatewayEvent::Respond("[3] ERROR:101".to_string())
        );
        // Value literals: `buf[15]` for ≤32-bit, `buf[25]` for 64-bit.
        assert_eq!(
            encode_payload(GwDataType::Uint(32), "4294967295"),
            Some(4294967295u32.to_le_bytes().to_vec())
        );
        // 15 bytes no longer fit the reference's `buf[15]`.
        assert_eq!(
            encode_payload(GwDataType::Uint(32), &"1".repeat(15)),
            None,
            "an over-long value literal is an errTok in the reference"
        );
        // …while 64-bit literals get the larger `buf[25]`.
        assert_eq!(
            encode_payload(GwDataType::Uint(64), "18446744073709551615"),
            Some(u64::MAX.to_le_bytes().to_vec())
        );
        assert_eq!(encode_payload(GwDataType::Uint(64), &"1".repeat(25)), None);
    }

    /// docs/DESIGN-DECISIONS.md N2/N3: command and sub-command tokens are case-insensitive and
    /// the `preoperational` full spelling is accepted (CANopenNode
    /// `CO_gateway_ascii.c:698,708-710,749,1076`).
    #[test]
    fn command_tokens_are_case_insensitive_and_preoperational_is_accepted() {
        let mut gw = Gateway::new();
        // Full `preoperational` spelling, default node set later.
        let _ = gw.feed_line("[1] set node 5");
        match gw.feed_line("[1] preoperational 5") {
            GatewayEvent::Request(GatewayRequest { action, .. }) => assert_eq!(
                action,
                GatewayAction::Nmt {
                    command: GatewayNmt::PreOperational,
                    node: 5,
                }
            ),
            other => panic!("{other:?}"),
        }
        // Upper case verb, upper case reset mode and set key.
        match gw.feed_line("[2] READ 0x1000 0") {
            GatewayEvent::Request(GatewayRequest { action, .. }) => assert_eq!(
                action,
                GatewayAction::Read {
                    node: 5,
                    index: 0x1000,
                    sub: 0,
                    data_type: GwDataType::Hex,
                }
            ),
            other => panic!("{other:?}"),
        }
        assert!(matches!(
            gw.feed_line("[3] 4 RESET COMMUNICATION"),
            GatewayEvent::Request(GatewayRequest {
                action: GatewayAction::Nmt {
                    command: GatewayNmt::ResetCommunication,
                    node: 4
                },
                ..
            })
        ));
        assert!(matches!(
            gw.feed_line("[4] SET NODE 6"),
            GatewayEvent::Request(GatewayRequest {
                action: GatewayAction::SetNode(6),
                ..
            })
        ));
        assert_eq!(gw.default_node(), Some(6));
    }

    /// docs/DESIGN-DECISIONS.md 七G9: CANopenNode accepts the `r`/`w` shorthand aliases
    /// (`CO_gateway_ascii.c:702-705`).
    #[test]
    fn read_and_write_accept_short_aliases() {
        let mut gw = Gateway::new();
        let _ = gw.feed_line("[1] set node 5");
        match gw.feed_line("[1] r 0x1000 0") {
            GatewayEvent::Request(GatewayRequest { action, .. }) => assert_eq!(
                action,
                GatewayAction::Read {
                    node: 5,
                    index: 0x1000,
                    sub: 0,
                    data_type: GwDataType::Hex,
                }
            ),
            other => panic!("{other:?}"),
        }
        match gw.feed_line("[2] w 0x1017 0 u16 500") {
            GatewayEvent::Request(GatewayRequest { action, .. }) => assert_eq!(
                action,
                GatewayAction::Write {
                    node: 5,
                    index: 0x1017,
                    sub: 0,
                    data_type: GwDataType::Uint(16),
                    payload: 500u16.to_le_bytes().to_vec(),
                }
            ),
            other => panic!("{other:?}"),
        }
    }

    /// docs/DESIGN-DECISIONS.md N4: the `read` subindex is mandatory (CiA 309-3 / CANopenNode
    /// `CO_gateway_ascii.c:863-870`); omitting it is a syntax error.
    #[test]
    fn read_requires_a_subindex() {
        let mut gw = Gateway::new();
        // Configure a default node so the error under test is the missing
        // subindex, not a missing default node (G6 → 105).
        let _ = gw.feed_line("[1] set node 4");
        assert_eq!(
            gw.feed_line("[1] read 0x1000"),
            GatewayEvent::Respond("[1] ERROR:101".to_string())
        );
        assert_eq!(
            gw.feed_line("[2] 4 read 0x1000"),
            GatewayEvent::Respond("[2] ERROR:101".to_string())
        );
        assert!(matches!(
            gw.feed_line("[3] 4 read 0x1000 2"),
            GatewayEvent::Request(_)
        ));
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
        // docs/DESIGN-DECISIONS.md GTW-04: reset accepts a leading node id like the other NMT
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
        // G3: CANopenNode also accepts the full spelling
        // (`CO_gateway_ascii.c:1077-1086`).
        assert!(matches!(
            gw.feed_line("[9] reset communication 4"),
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
        // G4: blank and comment-only lines answer with an empty response
        // frame instead of no response at all.
        assert_eq!(gw.feed_line(""), GatewayEvent::Respond(String::new()));
        assert_eq!(
            gw.feed_line("# comment"),
            GatewayEvent::Respond(String::new())
        );
        // Frameless garbage still yields a response (seq 0 per CiA 309-3)
        // so clients never hang.
        assert_eq!(
            gw.feed_line("garbage"),
            GatewayEvent::Respond("[0] ERROR:101".to_string())
        );
        assert_eq!(
            gw.feed_line("[abc] read 0x1000"),
            // docs/DESIGN-DECISIONS.md 八L5: an unparseable sequence is not echoed back —
            // answer with sequence 0 like the frameless branch.
            GatewayEvent::Respond("[0] ERROR:101".to_string())
        );
        // Unknown verb → request not supported, CiA 309-3 code 100.
        assert_eq!(
            gw.feed_line("[9] teleport 4"),
            GatewayEvent::Respond("[9] ERROR:100".to_string())
        );
    }

    /// docs/DESIGN-DECISIONS.md 审-G2: the last successfully parsed sequence is parser state
    /// (CANopenNode's `gtwa->sequence`): a later frameless or malformed frame
    /// echoes it, and a `strtoul`-style numeric prefix is taken.
    #[test]
    fn sequence_state_survives_frameless_frames() {
        let mut gw = Gateway::new();
        assert!(matches!(
            gw.feed_line("[7] set node 4"),
            GatewayEvent::Request(GatewayRequest { sequence: 7, .. })
        ));
        // Frameless garbage now echoes the last valid sequence.
        assert_eq!(
            gw.feed_line("garbage"),
            GatewayEvent::Respond("[7] ERROR:101".to_string())
        );
        // A missing closing bracket likewise.
        assert_eq!(
            gw.feed_line("[8 read 0x1000 0"),
            GatewayEvent::Respond("[7] ERROR:101".to_string())
        );
        // `[12ab]` takes the numeric prefix like `strtoul`, still errors, and
        // must not dispatch the command.
        assert_eq!(
            gw.feed_line("[12ab] read 0x1000 0"),
            GatewayEvent::Respond("[12] ERROR:101".to_string())
        );
        // 12 is the current sequence from then on.
        assert_eq!(
            gw.feed_line("still frameless"),
            GatewayEvent::Respond("[12] ERROR:101".to_string())
        );
        // A token with no leading digits yields 0, like the C reference.
        assert_eq!(
            gw.feed_line("[abc] read 0x1000 0"),
            GatewayEvent::Respond("[0] ERROR:101".to_string())
        );
        // docs/DESIGN-DECISIONS.md GTW-17: a token that does not *end* with `]` breaks before
        // the assignment — the previous sequence stays current …
        assert_eq!(
            gw.feed_line("[1]x read 0x1000 0"),
            GatewayEvent::Respond("[0] ERROR:101".to_string())
        );
        assert_eq!(
            gw.feed_line("frameless again"),
            GatewayEvent::Respond("[0] ERROR:101".to_string())
        );
        // … while `strtoul`-style prefixes still assign before erroring:
        // `[-1x]` wraps to 0xFFFF_FFFF, `[010x]` is octal 8.
        assert_eq!(
            gw.feed_line("[-1x] read 0x1000 0"),
            GatewayEvent::Respond("[4294967295] ERROR:101".to_string())
        );
        assert_eq!(
            gw.feed_line("[010x] read 0x1000 0"),
            GatewayEvent::Respond("[8] ERROR:101".to_string())
        );
        // Octal without trailing garbage still dispatches (GTW-19).
        assert!(matches!(
            gw.feed_line("[010] read 0x1000 0"),
            GatewayEvent::Request(GatewayRequest { sequence: 8, .. })
        ));
    }

    #[test]
    fn sequence_overflow_saturates_and_negative_wraps() {
        let mut gw = Gateway::new();
        assert!(matches!(
            gw.feed_line("[4294967296] 1 read 0x1000 0"),
            GatewayEvent::Request(GatewayRequest {
                sequence: 0xFFFF_FFFF,
                ..
            })
        ));
        assert!(matches!(
            gw.feed_line("[-1] 1 read 0x1000 0"),
            GatewayEvent::Request(GatewayRequest {
                sequence: 0xFFFF_FFFF,
                ..
            })
        ));
        assert_eq!(
            gw.feed_line("[18446744073709551615] 1 read 0x1000 0"),
            GatewayEvent::Respond("[4294967295] ERROR:101".to_string())
        );
    }

    /// docs/DESIGN-DECISIONS.md 审-G3: the optional `[[<net>] <node>]` prefix — two leading
    /// numeric tokens are parsed as net+node (CANopenNode
    /// `CO_gateway_ascii.c:638-661`); this single-network gateway accepts and
    /// ignores a valid net.
    #[test]
    fn net_node_prefix_is_parsed() {
        let mut gw = Gateway::new();
        // net 2, node 3 → the command runs on node 3.
        match gw.feed_line("[1] 2 3 read 0x1000 0") {
            GatewayEvent::Request(req) => assert_eq!(
                req.action,
                GatewayAction::Read {
                    node: 3,
                    index: 0x1000,
                    sub: 0,
                    data_type: GwDataType::Hex
                }
            ),
            other => panic!("expected request, got {other:?}"),
        }
        // One numeric token remains the node.
        match gw.feed_line("[2] 3 read 0x1000 0") {
            GatewayEvent::Request(req) => {
                assert!(matches!(req.action, GatewayAction::Read { node: 3, .. }))
            }
            other => panic!("expected request, got {other:?}"),
        }
        // A third numeric token can never be a command → syntax error.
        assert_eq!(
            gw.feed_line("[3] 2 3 4 read 0x1000 0"),
            GatewayEvent::Respond("[3] ERROR:101".to_string())
        );
        // Net above the 16-bit grammar → CiA 309-3 code 106.
        assert_eq!(
            gw.feed_line("[4] 70000 3 read 0x1000 0"),
            GatewayEvent::Respond("[4] ERROR:106".to_string())
        );
        // Out-of-range node keeps its own code 107.
        assert_eq!(
            gw.feed_line("[5] 2 300 read 0x1000 0"),
            GatewayEvent::Respond("[5] ERROR:107".to_string())
        );
    }

    /// docs/DESIGN-DECISIONS.md 审-G1: a zero-length write payload is a syntax error
    /// (CANopenNode `CO_gateway_ascii.c:976-980`, type-independent) — both an
    /// empty `hex` token list and a quoted empty `vs ""`.
    #[test]
    fn empty_write_payload_is_a_syntax_error() {
        let mut gw = Gateway::new();
        assert_eq!(
            gw.feed_line("[1] 4 write 0x1000 0 hex"),
            GatewayEvent::Respond("[1] ERROR:101".to_string())
        );
        assert_eq!(
            gw.feed_line("[2] 4 write 0x1000 0 hex    "),
            GatewayEvent::Respond("[2] ERROR:101".to_string())
        );
        assert_eq!(
            gw.feed_line("[3] 4 write 0x1000 0 vs \"\""),
            GatewayEvent::Respond("[3] ERROR:101".to_string())
        );
        // A non-empty payload is unaffected.
        match gw.feed_line("[4] 4 write 0x1000 0 hex ab") {
            GatewayEvent::Request(req) => match req.action {
                GatewayAction::Write { payload, .. } => assert_eq!(payload, vec![0xAB]),
                other => panic!("expected write, got {other:?}"),
            },
            other => panic!("expected request, got {other:?}"),
        }
    }

    #[test]
    fn sdo_abort_formatting_uses_hex_code() {
        // docs/DESIGN-DECISIONS.md G8: CANopenNode renders SDO aborts as `0x%08X`
        // (`CO_gateway_ascii.c:502-507`) — uppercase and zero-padded.
        assert_eq!(
            Gateway::format_error(
                3,
                &GatewayFailure::SdoAborted(SdoAbortCode::ObjectDoesNotExist)
            ),
            "[3] ERROR:0x06020000"
        );
        assert_eq!(
            Gateway::format_error(3, &GatewayFailure::Code(GatewayErrorCode::Timeout)),
            "[3] ERROR:103"
        );
    }

    #[test]
    fn node_and_subindex_literals_accept_hex_prefix() {
        // docs/DESIGN-DECISIONS.md 五G10: CANopenNode parses node/subindex with
        // `strtoul(_, _, 0)` (`CO_gateway_ascii.c:223-231`), so the index's
        // `0x` prefix must work on every numeric gateway token.
        let mut gw = Gateway::new();
        match gw.feed_line("[1] 0x4 read 0x1017 0x0 u16") {
            GatewayEvent::Request(req) => {
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
        // docs/DESIGN-DECISIONS.md GTW-18: the `write` subindex is mandatory (the reference
        // reads it with `closed = 0`); `0x1` is the sub here, `vs` the type.
        match gw.feed_line("[2] 4 write 0x1008 0x1 vs x") {
            GatewayEvent::Request(req) => {
                assert!(matches!(req.action, GatewayAction::Write { sub: 1, .. }))
            }
            other => panic!("expected request, got {other:?}"),
        }
        // Omitting the subindex swallows the type token → syntax error.
        assert!(matches!(
            gw.feed_line("[2] 4 write 0x1008 vs x"),
            GatewayEvent::Respond(msg) if msg.ends_with("ERROR:101")
        ));
        // docs/DESIGN-DECISIONS.md GTW-16: an unknown `set` key answers 100 even without a
        // value, and with trailing garbage.
        assert!(matches!(
            gw.feed_line("[4] set banana"),
            GatewayEvent::Respond(msg) if msg.ends_with("ERROR:100")
        ));
        assert!(matches!(
            gw.feed_line("[5] set banana x y"),
            GatewayEvent::Respond(msg) if msg.ends_with("ERROR:100")
        ));
        // docs/DESIGN-DECISIONS.md GTW-20: unsigned negative literals wrap at the declared
        // width like the reference's `strtoul` truncation — `u32 -1` is
        // 0xFFFF_FFFF, while `u16/u8 -1` stay value errors.
        match gw.feed_line("[6] 4 write 0x1008 0 u32 -1") {
            GatewayEvent::Request(req) => assert!(matches!(
                req.action,
                GatewayAction::Write { payload, .. } if payload == [0xFF; 4]
            )),
            other => panic!("expected request, got {other:?}"),
        }
        for line in ["[7] 4 write 0x1008 0 u16 -1", "[8] 4 write 0x1008 0 u8 -1"] {
            assert!(matches!(
                gw.feed_line(line),
                GatewayEvent::Respond(msg) if msg.ends_with("ERROR:101")
            ));
        }
        // `set node` follows the same rule.
        assert!(matches!(
            gw.feed_line("[3] set node 0x10"),
            GatewayEvent::Request(GatewayRequest {
                action: GatewayAction::SetNode(16),
                ..
            })
        ));
        // A sequence number may also be hex.
        assert!(matches!(
            gw.feed_line("[0x2a] 4 start"),
            GatewayEvent::Request(GatewayRequest { sequence: 42, .. })
        ));
    }

    #[test]
    fn hash_comment_terminates_a_glued_token() {
        // docs/DESIGN-DECISIONS.md GTW-14: the reference's `vs` value parser copies `#`
        // verbatim (`CO_fifo_cpyTok2Vs`, `CO_fifo.c:1266-1333`) — only the
        // *command/number* token reader and the hex/numeric scanners treat it
        // as a comment (`:352-360`, `:1187-1188`).
        let mut gw = Gateway::new();
        match gw.feed_line("[1] 4 write 0x1008 0 vs ab#cd") {
            GatewayEvent::Request(req) => assert_eq!(
                req.action,
                GatewayAction::Write {
                    node: 4,
                    index: 0x1008,
                    sub: 0,
                    data_type: GwDataType::Str,
                    payload: b"ab#cd".to_vec(),
                }
            ),
            other => panic!("expected request, got {other:?}"),
        }
        // Quoted strings may carry `#` (and spaces) too.
        match gw.feed_line("[2] 4 write 0x1008 0 vs \"device #1\"") {
            GatewayEvent::Request(req) => assert_eq!(
                req.action,
                GatewayAction::Write {
                    node: 4,
                    index: 0x1008,
                    sub: 0,
                    data_type: GwDataType::Str,
                    payload: b"device #1".to_vec(),
                }
            ),
            other => panic!("expected request, got {other:?}"),
        }
        // Trailing garbage after a closed string is still a syntax error.
        assert!(matches!(
            gw.feed_line("[3] 4 write 0x1008 0 vs \"a\" b"),
            GatewayEvent::Respond(msg) if msg.ends_with("ERROR:101")
        ));
        // Numeric/hex payloads keep the comment rule.
        assert!(matches!(
            gw.feed_line("[4] 4 write 0x1008 0 hex 01 # one byte"),
            GatewayEvent::Request(GatewayRequest {
                action: GatewayAction::Write {
                    data_type: GwDataType::Hex,
                    ..
                },
                ..
            })
        ));
        // …and a `#` inside a command token still ends the token and the
        // line (the reference's `readToken`).
        assert!(matches!(
            gw.feed_line("[5] 4 read#comment 0x1000 0"),
            GatewayEvent::Respond(msg) if msg.ends_with("ERROR:101")
        ));
        // A token-only comment after the value is the classic case.
        assert!(matches!(
            gw.feed_line("[2] 4 start # go"),
            GatewayEvent::Request(GatewayRequest {
                action: GatewayAction::Nmt {
                    command: GatewayNmt::Start,
                    node: 4
                },
                ..
            })
        ));
    }

    #[test]
    fn boolean_payloads_accept_any_byte_value() {
        // docs/DESIGN-DECISIONS.md 五G12: `CO_fifo_cpyTok2U8` accepts any 0..=255 literal;
        // true/false remain as a convenience.
        assert_eq!(encode_payload(GwDataType::Bool, "0"), Some(vec![0]));
        assert_eq!(encode_payload(GwDataType::Bool, "1"), Some(vec![1]));
        assert_eq!(encode_payload(GwDataType::Bool, "2"), Some(vec![2]));
        assert_eq!(encode_payload(GwDataType::Bool, "255"), Some(vec![0xFF]));
        assert_eq!(encode_payload(GwDataType::Bool, "0x10"), Some(vec![0x10]));
        assert_eq!(encode_payload(GwDataType::Bool, "false"), Some(vec![0]));
        assert_eq!(encode_payload(GwDataType::Bool, "true"), Some(vec![1]));
        assert_eq!(encode_payload(GwDataType::Bool, "256"), None);
        assert_eq!(encode_payload(GwDataType::Bool, "-1"), None);
    }

    #[test]
    fn encode_payload_sign_extends_negative_ints() {
        // docs/DESIGN-DECISIONS.md #1: negatives must be encoded at the declared width with
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
        // u64::MAX is writable (docs/DESIGN-DECISIONS.md #30: u64 parse, not i64).
        assert_eq!(
            encode_payload(GwDataType::Uint(64), "18446744073709551615"),
            Some(vec![0xFF; 8])
        );
        // Out-of-range literals remain a syntax error (R10-P2-10).
        assert_eq!(encode_payload(GwDataType::Int(8), "300"), None);
        assert_eq!(encode_payload(GwDataType::Uint(8), "-1"), None);
    }

    /// docs/DESIGN-DECISIONS.md 五G10: the typed-write literals must accept the C base-0
    /// grammar CANopenNode's `CO_fifo_cpyTok2*` inherit from
    /// `strtoul`/`strtoll` (`CO_fifo.c:840,869,896,927,956,985,1014,1043`) — `0x`
    /// hex, a leading `0` selecting octal, optional `+`/`-` — and reject
    /// trailing garbage. Previously only decimal parsed, so
    /// `write 0x1017 0 u16 0x1F4` answered `ERROR:101`.
    #[test]
    fn typed_write_literals_follow_c_base_zero_grammar() {
        // Hex, in both directions and both prefix spellings.
        assert_eq!(
            encode_payload(GwDataType::Uint(16), "0x1F4"),
            Some(vec![0xF4, 0x01])
        );
        assert_eq!(
            encode_payload(GwDataType::Uint(32), "0X1234ABCD"),
            Some(vec![0xCD, 0xAB, 0x34, 0x12])
        );
        assert_eq!(
            encode_payload(GwDataType::Int(16), "-0x80"),
            Some(vec![0x80, 0xFF])
        );
        assert_eq!(encode_payload(GwDataType::Int(64), "0x7FFFFFFFFFFFFFFF"), {
            let mut v = vec![0xFFu8; 8];
            v[7] = 0x7F;
            Some(v)
        });
        // A leading 0 selects octal (C `strtoul(_, _, 0)`), so `010` is 8.
        assert_eq!(encode_payload(GwDataType::Uint(8), "010"), Some(vec![8]));
        // Decimal and the plain number 0 are unchanged.
        assert_eq!(encode_payload(GwDataType::Uint(8), "10"), Some(vec![10]));
        assert_eq!(encode_payload(GwDataType::Uint(8), "0"), Some(vec![0]));
        assert_eq!(encode_payload(GwDataType::Int(8), "+5"), Some(vec![5]));
        // Unsigned `-1` wraps modulo 2**64 like `strtoull`; the declared
        // width's range check then decides (u8 rejects, u64 accepts).
        assert_eq!(
            encode_payload(GwDataType::Uint(64), "-1"),
            Some(vec![0xFF; 8])
        );
        assert_eq!(encode_payload(GwDataType::Uint(8), "-1"), None);
        // Trailing garbage, bare signs and malformed prefixes stay errors.
        for bad in ["0x", "1F4", "12ab", "-", "+", "-0x", "8 9"] {
            assert_eq!(
                encode_payload(GwDataType::Uint(32), bad),
                None,
                "{bad:?} must be rejected"
            );
        }
        // Out-of-range values are still gateway-level syntax errors
        // (docs/DESIGN-DECISIONS.md R10-P2-10).
        assert_eq!(encode_payload(GwDataType::Uint(8), "0x100"), None);
        assert_eq!(encode_payload(GwDataType::Uint(16), "0x10000"), None);
        assert_eq!(
            encode_payload(GwDataType::Uint(64), "0x10000000000000000"),
            None
        );
    }

    #[test]
    fn read_defaults_to_hex_and_formats_like_reference() {
        // G1: an omitted read type must be `hex` raw passthrough, not
        // decimal `Uint(32)`.
        let mut gw = Gateway::new();
        match gw.feed_line("[1] 4 read 0x1017 0") {
            GatewayEvent::Request(req) => {
                assert_eq!(
                    req.action,
                    GatewayAction::Read {
                        node: 4,
                        index: 0x1017,
                        sub: 0,
                        data_type: GwDataType::Hex,
                    }
                );
            }
            other => panic!("expected request, got {other:?}"),
        }
        // `CO_fifo_readHex2a` format: uppercase, space separated, first
        // byte without a leading space.
        assert_eq!(
            Gateway::format_value(1, &Value::Bytes(vec![0xF4, 0x01])),
            "[1] F4 01"
        );
        assert_eq!(
            Gateway::format_value(2, &Value::Bytes(vec![0x00])),
            "[2] 00"
        );
        assert_eq!(Gateway::format_value(3, &Value::Bytes(Vec::new())), "[3] ");
    }

    #[test]
    fn hex_payloads_accept_separated_and_contiguous_digits() {
        // G1 / GTW-26: `hex` write literals mirror `CO_fifo_cpyTok2Hex` —
        // optional space separation **between** digit pairs, and a lone digit
        // group is its own byte (whitespace closes the pair,
        // `CO_fifo.c:1188-1203`).
        assert_eq!(
            encode_payload(GwDataType::Hex, "F4 01"),
            Some(vec![0xF4, 0x01])
        );
        assert_eq!(
            encode_payload(GwDataType::Hex, "f401"),
            Some(vec![0xF4, 0x01])
        );
        assert_eq!(encode_payload(GwDataType::Hex, "F"), Some(vec![0x0F]));
        // GTW-26: a space closes the pair, so a single-digit group is its own
        // byte — the old code paired it with the next group's first digit and
        // wrote wrong data while answering `OK`.
        assert_eq!(
            encode_payload(GwDataType::Hex, "0 1 2"),
            Some(vec![0x00, 0x01, 0x02])
        );
        assert_eq!(
            encode_payload(GwDataType::Hex, "f f"),
            Some(vec![0x0F, 0x0F])
        );
        assert_eq!(
            encode_payload(GwDataType::Hex, "ab c d"),
            Some(vec![0xAB, 0x0C, 0x0D])
        );
        assert_eq!(
            encode_payload(GwDataType::Hex, "01 a2 ff"),
            Some(vec![0x01, 0xA2, 0xFF])
        );
        assert_eq!(
            encode_payload(GwDataType::Hex, "F4 0"),
            Some(vec![0xF4, 0x00])
        );
        assert_eq!(encode_payload(GwDataType::Hex, "zz"), None);
        assert_eq!(encode_payload(GwDataType::Hex, "F4x01"), None);
    }

    #[test]
    fn vs_payloads_honour_quotes_and_escapes() {
        // G2: `CO_fifo_cpyTok2Vs` semantics — quotes delimit a multi-word
        // string, `""` is an escaped quote, a bare word needs no quotes.
        let mut gw = Gateway::new();
        match gw.feed_line("[1] 4 write 0x1008 0 vs \"my device\"") {
            GatewayEvent::Request(req) => {
                assert_eq!(
                    req.action,
                    GatewayAction::Write {
                        node: 4,
                        index: 0x1008,
                        sub: 0,
                        data_type: GwDataType::Str,
                        payload: b"my device".to_vec(),
                    }
                );
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(
            encode_payload(GwDataType::Str, "\"a\"\"b\""),
            Some(b"a\"b".to_vec())
        );
        assert_eq!(encode_payload(GwDataType::Str, "\"\""), Some(Vec::new()));
        assert_eq!(
            encode_payload(GwDataType::Str, "device"),
            Some(b"device".to_vec())
        );
        // Unquoted multi-word and trailing garbage after the closing quote
        // are syntax errors (reference `errTok`).
        assert_eq!(encode_payload(GwDataType::Str, "my device"), None);
        assert_eq!(encode_payload(GwDataType::Str, "\"a\"b\""), None);
        assert_eq!(encode_payload(GwDataType::Str, "a\"b"), None);
        // A stray opening quote without a close is malformed.
        assert_eq!(encode_payload(GwDataType::Str, "\"abc"), None);
    }
}
