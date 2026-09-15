//! EDS/DCF (CiA 306) file parsing into [`ObjectDictionary`]s.
//!
//! Grammar: INI-style sections `[<index>]` / `[<index>sub<subindex>]` with
//! keys `ParameterName`, `DataType`, `AccessType`, `DefaultValue`,
//! `PDOMapping`, `SubNumber`, `CompactSubObj`, plus the
//! `[DeviceComissioning]` section (the CiA 306 misspelling is canonical and
//! accepted, as is the corrected spelling).
//!
//! `$NODEID` expressions (`$NODEID+0x200`, case-insensitive) are stored as
//! [`OdExpr::NodeRelative`] and resolved lazily against the actual node id —
//! fixing python-canopen's parse-time binding (docs/REFERENCES.md V8).

use std::collections::BTreeMap;

use canopen_core::datatypes::{DataType, Value};
use canopen_core::object_dictionary::{AccessType, Entry, ObjectDictionary, Storage};
use canopen_core::types::{Address, Error, Result};

/// A default-value expression that may depend on the node id.
#[derive(Debug, Clone, PartialEq)]
pub enum OdExpr {
    /// A decimal or hexadecimal integer literal.
    Literal(i64),
    /// `$NODEID ± offset` — resolved at dictionary build time. Signed
    /// offsets are supported (`$NODEID-0x100`), matching python-canopen's
    /// `\\+?\\$NODEID\\+?` grammar (`BUGS.md` P1-5).
    NodeRelative {
        /// Constant added to the runtime node id.
        offset: i64,
    },
    /// A double-quoted string default (VisibleString/OCTET/DOMAIN).
    Str(String),
    /// A floating-point default (`REAL32`/`REAL64`).
    Float(f64),
}

impl OdExpr {
    /// Parses an EDS default-value expression.
    ///
    /// Understands quoted strings, `$NODEID±offset` (signed offsets, both
    /// hex and decimal), hexadecimal/decimal integers and decimal
    /// floating-point values (`BUGS.md` P1-5/P1-6).
    #[must_use]
    pub fn parse(raw: &str) -> Option<Self> {
        let raw = raw.trim();
        if raw.len() >= 2 && raw.starts_with('"') && raw.ends_with('"') {
            return Some(Self::Str(raw[1..raw.len() - 1].to_string()));
        }
        // `$NODEID` may appear before OR after its offset and with optional
        // surrounding `+` and whitespace: python-canopen removes
        // `\+?\$NODEID\+?` from the whole (space-free) value and adds the
        // node id afterwards (`eds.py:254-259`), so `1280+$NODEID` and
        // `$NODEID+0x200` are equivalent (十H3). The byte-preserving ASCII
        // uppercase copy keeps `pos` valid for the original `compact` and
        // avoids slicing inside a multi-byte character.
        let compact: String = raw.chars().filter(|c| !c.is_whitespace()).collect();
        let upper = compact.to_ascii_uppercase();
        if let Some(pos) = upper.find("$NODEID") {
            let head = &compact[..pos];
            let tail = &compact[pos + 7..];
            let before = head.strip_suffix('+').unwrap_or(head);
            let after = tail.strip_prefix('+').unwrap_or(tail);
            // A bare trailing '+' (`$NODEID+`, `1280+$NODEID+`) is malformed,
            // matching python's `int()` failure; a leading '+' (`+$NODEID`)
            // is the documented optional sign (`BUGS.md` P1-5).
            if tail == "+" {
                return None;
            }
            let offset = match (before.is_empty(), after.is_empty()) {
                (true, true) => 0,
                (true, false) => parse_int(after)?,
                (false, true) => parse_int(before)?,
                // An offset on both sides cannot be expressed as a single
                // node-relative value — reject it like python's `int()` does.
                (false, false) => return None,
            };
            return Some(Self::NodeRelative { offset });
        }
        let lower = raw.to_ascii_lowercase();
        let looks_hex = ["+0x", "-0x", "0x"].iter().any(|p| lower.starts_with(p));
        // `NaN`/`inf` carry neither '.' nor 'e' but are valid `f64`
        // spellings; python parses FLOAT defaults with plain `float(value)`
        // (`eds.py:251-252`), so they must not degrade to 0 (十M25).
        let looks_float = lower.contains('.')
            || lower.contains('e')
            || lower.contains("nan")
            || lower.contains("inf");
        if !looks_hex && looks_float {
            if let Ok(f) = raw.parse::<f64>() {
                return Some(Self::Float(f));
            }
        }
        Some(Self::Literal(parse_int(raw)?))
    }

    /// Resolves against a concrete node id (numeric expressions only;
    /// string/float defaults resolve to 0 — see [`EdsSubEntry::to_value`]).
    #[must_use]
    pub fn resolve(&self, node_id: u8) -> i64 {
        match self {
            Self::Literal(v) => *v,
            Self::NodeRelative { offset } => i64::from(node_id) + offset,
            Self::Str(_) | Self::Float(_) => 0,
        }
    }
}

/// Decodes a whitespace-separated hex byte string ("01 A2 ff"); empty on
/// malformed input. Strictly mirrors python-canopen `bytes.fromhex`: any
/// non-hex character invalidates the whole string (previously stray letters
/// were silently filtered and the remainder paired up — `"Bed"` produced a
/// byte instead of nothing, `BUGS.md` P3-65).
#[must_use]
pub fn hex_bytes(raw: &str) -> Vec<u8> {
    let clean: String = raw.split_whitespace().collect::<Vec<_>>().join("");
    if clean.len() % 2 != 0 || !clean.chars().all(|c| c.is_ascii_hexdigit()) {
        return Vec::new();
    }
    (0..clean.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&clean[i..i + 2], 16).unwrap_or(0))
        .collect()
}

/// Normalises an unsigned hex literal into its signed two's-complement
/// value at `bits` width (`INTEGER16 DefaultValue=0xFFFF` → -1), matching
/// python-canopen's `_signed_int_from_hex` (`eds.py:211-241`). Shared by the
/// runtime EDS path and the code generator so both build exits agree
/// (`BUGS.md` P1-7/P2-14).
///
/// Returns `None` when the magnitude exceeds the type's *unsigned* range
/// (`0x100` for INTEGER8, `0x10000` for INTEGER16): python raises
/// `ValueError` there (`eds.py:236-239`) and silently wrapping would forge a
/// different legal value. Negative literals below `min_signed` are rejected
/// the same way.
#[must_use]
pub fn wrap_signed_hex(v: i64, bits: usize) -> Option<i64> {
    if bits == 0 || bits >= 64 {
        return Some(v);
    }
    let min_signed = -(1i64 << (bits - 1));
    let max_signed = (1i64 << (bits - 1)) - 1;
    let max_unsigned = (1i64 << bits) - 1;
    if v < min_signed || v > max_unsigned {
        return None;
    }
    Some(if v > max_signed {
        v - (1i64 << bits)
    } else {
        v
    })
}

/// Applies [`wrap_signed_hex`] to an `i128` integer default, mapping the
/// out-of-range / over-i64 cases onto [`Error::ValueOutOfRange`] for the
/// runtime dictionary build.
fn signed_default(raw: i128, dt: DataType) -> Result<i64> {
    let bits = dt.fixed_size().unwrap_or(0) * 8;
    let (min, max) = int_bounds(dt).unwrap_or((i128::MIN, i128::MAX));
    let out_of_range = || Error::ValueOutOfRange {
        value: raw.clamp(i64::MIN as i128, i64::MAX as i128) as i64,
        min: to_i64_bound(min),
        max: to_i64_bound(max),
    };
    let v = i64::try_from(raw).map_err(|_| out_of_range())?;
    wrap_signed_hex(v, bits).ok_or_else(out_of_range)
}

/// Inclusive bounds of a fixed-size integer DataType, for range-checking
/// defaults (`BUGS.md` H2 / M1). 返回 `i128` 以容纳 `Unsigned64` 的完整上界
/// `u64::MAX`（旧实现把 U64 上界截成 `i64::MAX`，导致合法的 U64 默认值如
/// `0xFFFFFFFFFFFFFFFF` 被误判越界拒绝）。
pub fn int_bounds(dt: DataType) -> Option<(i128, i128)> {
    let bits = dt.fixed_size()? * 8;
    if dt.is_signed() {
        if bits >= 64 {
            return Some((i128::from(i64::MIN), i128::from(i64::MAX)));
        }
        let half = 1i128 << (bits - 1);
        Some((-half, half - 1))
    } else if bits >= 64 {
        // 注意：U64 上界是 `u64::MAX`，不能用 `i64::MAX`（BUGS.md M1）。
        Some((0, i128::from(u64::MAX)))
    } else {
        Some((0, (1i128 << bits) - 1))
    }
}

/// 把 `int_bounds` 的 `i128` 边界夹到 [`Error::ValueOutOfRange`] 所需的 `i64`
/// 字段（诊断信息只需大致范围：超 `i64` 上界的 U64 截断为 `i64::MAX`）。
fn to_i64_bound(v: i128) -> i64 {
    v.clamp(i64::MIN as i128, i64::MAX as i128) as i64
}

/// Truncates a float default (`DefaultValue=1e9`) to the declared integer
/// type's range. `None` when the value is non-finite or out of range —
/// the EDS is then rejected loudly instead of silently zeroing the
/// default (`BUGS.md` H2). Shared by the runtime EDS path and the code
/// generator so both build exits agree.
#[must_use]
pub fn float_default_to_int(f: f64, dt: DataType) -> Option<i128> {
    if !f.is_finite() {
        return None;
    }
    let (min, max) = int_bounds(dt)?;
    // Compare through the saturating float→int conversion: comparing with
    // `max as f64` is inexact at 2^63/2^64 (`as f64` rounds up to 2^64),
    // which let `Unsigned64 DefaultValue=1.8446744073709552e19` through and
    // silently produced `Uint(0)` at runtime / an out-of-range literal in
    // codegen (BUGS.md 十M13). `f64 as i128` saturates, so values beyond
    // i128 still fail the bounds check.
    let v = f.trunc() as i128;
    if v < min || v > max {
        return None;
    }
    Some(v)
}

/// 整数/无符号/布尔类型谓词（供默认值解析区分字符串类型）。
#[must_use]
pub fn is_integer_type(dt: DataType) -> bool {
    use DataType as Dt;
    matches!(
        dt,
        Dt::Boolean
            | Dt::Integer8
            | Dt::Integer16
            | Dt::Integer24
            | Dt::Integer32
            | Dt::Integer40
            | Dt::Integer48
            | Dt::Integer56
            | Dt::Integer64
            | Dt::Unsigned8
            | Dt::Unsigned16
            | Dt::Unsigned24
            | Dt::Unsigned32
            | Dt::Unsigned40
            | Dt::Unsigned48
            | Dt::Unsigned56
            | Dt::Unsigned64
    )
}

/// 将字符串形式的整数默认值解析为 `i64` 并按类型范围校验（BUGS.md B3-3）。
/// 越界或不可解析（如非数字字符串）显式报错，不再静默退化为 0。
///
/// # Errors
/// 数值超出 `int_bounds(dt)` 范围或无法解析为整数时返回 [`Error::ValueOutOfRange`]。
pub fn parse_integer_default(s: &str, dt: DataType) -> Result<i128> {
    let s = s.trim();
    let (min, max) = int_bounds(dt).unwrap_or((i128::MIN, i128::MAX));
    let v: i128 = if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        i128::from_str_radix(hex, 16).map_err(|_| Error::ValueOutOfRange {
            value: 0,
            min: to_i64_bound(min),
            max: to_i64_bound(max),
        })?
    } else {
        s.parse::<i128>().map_err(|_| Error::ValueOutOfRange {
            value: 0,
            min: to_i64_bound(min),
            max: to_i64_bound(max),
        })?
    };
    if v < min || v > max {
        return Err(Error::ValueOutOfRange {
            value: v as i64,
            min: to_i64_bound(min),
            max: to_i64_bound(max),
        });
    }
    Ok(v)
}

/// Parses a decimal or `0x`-prefixed hexadecimal (optionally signed)
/// integer literal.
///
/// The magnitude is accumulated in `i128` before the sign is applied, so
/// `i64::MIN` (whose magnitude does not fit `i64`) parses instead of being
/// rejected as "out of range" (BUGS.md 十L11).
#[must_use]
pub fn parse_int(raw: &str) -> Option<i64> {
    let raw = raw.trim();
    let (negative, digits) = match raw.strip_prefix('-') {
        Some(rest) => (true, rest.trim()),
        // BUGS.md 八L10: python's `int(value, 0)` accepts an explicit '+'
        // (`+0x8000` == 32768); EDS files generated by such tooling must
        // parse instead of failing the whole import.
        None => (false, raw.strip_prefix('+').map_or(raw, str::trim_start)),
    };
    let magnitude: i128 = if let Some(hex) = digits
        .strip_prefix("0x")
        .or_else(|| digits.strip_prefix("0X"))
    {
        i128::from_str_radix(hex, 16).ok()?
    } else {
        digits.parse::<i128>().ok()?
    };
    let value = if negative { -magnitude } else { magnitude };
    i64::try_from(value).ok()
}

/// One subentry definition from the file.
#[derive(Debug, Clone)]
pub struct EdsSubEntry {
    /// ParameterName.
    pub name: String,
    /// Declared CiA 301 data type index; unknown/absent → `Unsigned32`.
    pub data_type: DataType,
    /// AccessType (`rwr`/`rww` map to `Rw`).
    pub access: AccessType,
    /// Raw DefaultValue expression, if any.
    pub default: Option<OdExpr>,
    /// Original spelling of `DefaultValue` (after trimming). VisibleString
    /// defaults keep their textual form — python returns the raw string
    /// (`eds.py:249-250`), so `0x41` must stay `"0x41"` instead of being
    /// normalised to `"65"` (十M25). `None` when the key was absent.
    pub default_raw: Option<String>,
    /// PDOMapping flag = 1.
    pub pdo_mappable: bool,
    /// Vendor-specific subentry keys this parser does not understand,
    /// preserved verbatim like python-canopen's per-variable
    /// `custom_options` (eds.py:398) so round-tripping an EDS never loses
    /// them (`BUGS.md` #19).
    pub custom_options: std::collections::BTreeMap<String, String>,
}

/// One object (index) from the file.
#[derive(Debug, Clone)]
pub struct EdsObject {
    /// 16-bit object index.
    pub index: u16,
    /// ParameterName of the object itself.
    pub name: String,
    /// Subentries keyed by subindex (VAR objects keep them at sub 0).
    pub subs: BTreeMap<u8, EdsSubEntry>,
    /// `CompactSubObj=N`: the object's inline definition is a template to be
    /// expanded into N identical subentries (plus an Unsigned8 count at
    /// sub 0). Captured at parse time, expanded by [`Eds::to_object_dictionary`].
    pub compact_sub_obj: Option<u16>,
    /// Per-subentry names from an `[index]Name` section (`1=Channel 1`),
    /// applied to the expanded subentries (python-canopen
    /// `eds.py:172-182`).
    pub sub_names: BTreeMap<u8, String>,
    /// Vendor-specific keys that this parser does not understand, preserved
    /// verbatim like python-canopen's `custom_options` instead of being
    /// misapplied to a fabricated sub-0 entry.
    pub custom_options: std::collections::BTreeMap<String, String>,
}

/// A parsed EDS/DCF file.
#[derive(Debug, Clone, Default)]
pub struct Eds {
    /// Objects by index, ascending.
    pub objects: BTreeMap<u16, EdsObject>,
    /// `[DeviceComissioning] NodeID` raw expression, when present.
    pub configured_node_id: Option<OdExpr>,
    /// Non-fatal parse findings (unknown AccessType spellings, oversized
    /// CompactSubObj, …) surfaced instead of silently swallowed
    /// (`BUGS.md` P3-52/P3-54).
    pub warnings: Vec<String>,
}

/// Strips an inline comment from one EDS line, honouring double-quoted
/// strings and python-configparser's whitespace rule (verified against
/// `RawConfigParser(inline_comment_prefixes=(';',))`): a `;` only starts a
/// comment when it is **outside** a `"…"` string **and** preceded by
/// whitespace (or at the very start of the line). A quoted `DefaultValue`
/// like `"a;b"` must survive intact, while `X ; trailing` must lose its
/// tail (`BUGS.md` H2).
fn strip_comment(line: &str) -> &str {
    let mut in_quote = false;
    let mut prev_ws = true; // line start counts as "preceded by whitespace"
    for (i, c) in line.char_indices() {
        match c {
            '"' => in_quote = !in_quote,
            ';' if !in_quote && prev_ws => return &line[..i],
            _ => {}
        }
        prev_ws = c.is_whitespace();
    }
    line
}

/// Parses an EDS/DCF file from its full text.
///
/// Unknown (non-index) section headers are skipped, mirroring
/// python-canopen; sub-index headers accept hexadecimal digits.
///
/// # Errors
/// [`Error::Other`] on lines without `=` inside a recognised section.
pub fn parse(text: &str) -> Result<Eds> {
    let mut eds = Eds::default();
    let mut current = Section::Skipped;

    for raw_line in text.lines() {
        // python RawConfigParser's `comment_prefixes` includes `#`: a
        // whole-line comment must be dropped before key parsing (十L8).
        // Inline `#` is NOT a comment there (`inline_comment_prefixes`
        // only contains `;`), so only the line-start form is skipped.
        if raw_line.trim_start().starts_with('#') {
            continue;
        }
        let line = strip_comment(raw_line).trim();
        if line.is_empty() {
            continue;
        }
        if line.starts_with('[') && line.contains(']') {
            // Non-index sections ([FileInfo], [DeviceInfo], [Comments],
            // [MandatoryObjects], [DummyUsage], …) are skipped wholesale —
            // python-canopen parses/skips them by name and must not fail
            // the file (`BUGS.md` P1-4). `[index]Name` suffixed headers
            // are also treated as section headers (the trailing name is
            // ignored), matching configparser's lenient section regex
            // (`BUGS.md` #14).
            let close = line.find(']').expect("checked above");
            current = parse_header(&line[1..close]);
            continue;
        }
        if matches!(current, Section::Commissioning) {
            // BUGS.md 八L8: `NodeIDSet` is a boolean commissioning flag
            // (CiA 306), not a node id — only `NodeID` carries the number.
            // python-canopen (`eds.py:97-100`) likewise reads only `NodeID`.
            if let Some((key, value)) = line.split_once('=') {
                if key.trim().eq_ignore_ascii_case("NodeId") {
                    eds.configured_node_id = OdExpr::parse(value.trim());
                }
            }
            continue;
        }
        let Section::Object(index, sub, name_section) = current else {
            continue;
        };
        let Some((key, value)) = line.split_once('=') else {
            // BUGS.md L1: a stray line without `=` (e.g. a hand-edited
            // vendor section) must not abort the whole file — record a
            // warning and skip it, like python-canopen's lenient
            // configparser does.
            eds.warnings
                .push(format!("EDS line without '=' ignored: {raw_line:?}"));
            continue;
        };
        let key = key.trim();
        let value = value.trim();

        let mut warnings = std::mem::take(&mut eds.warnings);
        let object = eds.objects.entry(index).or_insert_with(|| EdsObject {
            index,
            name: String::new(),
            subs: BTreeMap::new(),
            compact_sub_obj: None,
            sub_names: BTreeMap::new(),
            custom_options: BTreeMap::new(),
        });

        match sub {
            // `[index]Name` (python-canopen `eds.py:172-182`): `NrOfEntries`
            // is the CompactSubObj expansion count and numeric keys are the
            // per-subentry names. Both used to be dropped (NrOfEntries) or
            // parked in `custom_options` (names) where nothing consumed
            // them. Any *other* key falls through to the ordinary inline
            // object handling — vendor files also use Name-suffixed sections
            // for plain object definitions (`BUGS.md` L5).
            None => {
                if name_section && key.eq_ignore_ascii_case("NrOfEntries") {
                    if let Some(n) = parse_int(value) {
                        let n = n.max(0);
                        object.compact_sub_obj = Some(u16::try_from(n).unwrap_or(u16::MAX));
                        if n > 255 {
                            warnings.push(format!(
                                "NrOfEntries={value} exceeds the 255-entry sub limit at [{index:#06x}] - expansion skipped (BUGS.md P3-54)"
                            ));
                        }
                    }
                } else if name_section {
                    if let Ok(sub_idx) = key.parse::<u8>() {
                        object.sub_names.insert(sub_idx, unquote(value));
                    } else {
                        apply_object_key(object, key, value, index, &mut warnings);
                    }
                } else {
                    apply_object_key(object, key, value, index, &mut warnings);
                }
            }
            Some(sub) => {
                apply_key(
                    object.subs.entry(sub).or_insert_with(|| EdsSubEntry {
                        name: String::new(),
                        data_type: DataType::Unsigned32,
                        access: AccessType::Rw,
                        default: None,
                        default_raw: None,
                        pdo_mappable: false,
                        custom_options: std::collections::BTreeMap::new(),
                    }),
                    key,
                    value,
                    index,
                    &mut warnings,
                );
            }
        }
        eds.warnings.extend(warnings);
    }
    Ok(eds)
}

/// Parsed section header kind. A sentinel pseudo-index would collide with a
/// real `[FFFE]` object and make it unparseable (`BUGS.md` 十L9), so the
/// commissioning section is modelled as its own variant.
#[derive(Debug, Clone, Copy)]
enum Section {
    /// `[DeviceComissioning]` / `[DeviceCommissioning]`.
    Commissioning,
    /// `[index]` / `[indexsubN]`. The `bool` marks the `[index]Name`
    /// variant, whose numeric keys are per-subentry names and whose
    /// `NrOfEntries` drives CompactSubObj expansion (python-canopen
    /// `eds.py:172-182`) — previously both were silently dropped.
    Object(u16, Option<u8>, bool),
    /// Any other section (`[FileInfo]`, `[DummyUsage]`, unknown names, …).
    Skipped,
}

fn parse_header(header: &str) -> Section {
    let lower = header.to_ascii_lowercase();
    if lower.starts_with("devicecomi") {
        // DeviceComissioning / DeviceCommissioning.
        return Section::Commissioning;
    }
    // BUGS.md L5: `[index]Name` / `[indexSubNName]` headers (python-canopen
    // eds.py:172 `^([0-9A-Fa-f]{4})Name`) must parse leniently — the
    // trailing name is not a hex digit, so stripping it before the radix
    // parse keeps the object's fields from being silently dropped. A
    // pure-hex header is unaffected: no valid index/subindex string ends
    // in "name" ('n' and 'm' are not hex digits), so the strip is a no-op
    // for them.
    let name_section = lower.ends_with("name");
    let base = lower.strip_suffix("name").unwrap_or(&lower);
    if let Some((idx_part, sub_part)) = base.split_once("sub") {
        // CiA 306 sub-indices are hexadecimal (python-canopen eds.py:158
        // uses [0-9A-Fa-f]+); "02" and "0A" both decode correctly.
        match (
            u16::from_str_radix(idx_part.trim(), 16),
            u8::from_str_radix(sub_part.trim(), 16),
        ) {
            (Ok(index), Ok(sub)) => Section::Object(index, Some(sub), false),
            _ => Section::Skipped,
        }
    } else {
        match u16::from_str_radix(base.trim(), 16) {
            Ok(index) => Section::Object(index, None, name_section),
            Err(_) => Section::Skipped,
        }
    }
}

/// Applies one key of an inline `[index]` / `[index]Name` section:
/// `ParameterName` names the object (and backfills its sub-0 entry), the
/// CiA 306 VAR keys configure the sub-0 template, the count keys are
/// consumed (`CompactSubObj` above 255 warns), and unknown vendor keys are
/// parked in `custom_options` instead of fabricating a bogus sub-0 entry.
fn apply_object_key(
    object: &mut EdsObject,
    key: &str,
    value: &str,
    index: u16,
    warnings: &mut Vec<String>,
) {
    match key {
        "ParameterName" => {
            object.name = unquote(value);
            // BUGS.md 十M16: `ParameterName` may appear *after*
            // `DataType`/`AccessType`/… in the same section; the
            // inline sub-0 entry is then created with the
            // still-empty object name, so backfill it here too.
            // (python-canopen resolves the name from the finished
            // section, `eds.py:313`.)
            if let Some(entry) = object.subs.get_mut(&0) {
                entry.name.clone_from(&object.name);
            }
        }
        "CompactSubObj" => {
            object.compact_sub_obj = parse_int(value).and_then(|n| u16::try_from(n.max(0)).ok());
            if object.compact_sub_obj.is_some_and(|n| n > 255) {
                warnings.push(format!(
                    "CompactSubObj={value} exceeds the 255-entry sub limit at [{index:#06x}] - expansion skipped (BUGS.md P3-54)"
                ));
            }
        }
        "SubNumber" | "NrOfEntries" | "ObjectType" | "Denotation" | "Comment" => {}
        // Known CiA 306 keys of an inline-defined VAR object apply
        // to its single entry at sub 0.
        "DataType" | "AccessType" | "DefaultValue" | "PDOMapping" => {
            apply_key(
                object.subs.entry(0).or_insert_with(|| EdsSubEntry {
                    name: object.name.clone(),
                    data_type: DataType::Unsigned32,
                    access: AccessType::Rw,
                    default: None,
                    default_raw: None,
                    pdo_mappable: false,
                    custom_options: std::collections::BTreeMap::new(),
                }),
                key,
                value,
                index,
                warnings,
            );
        }
        _ => {
            // Unknown vendor key: preserved verbatim
            // (python-canopen keeps these in `custom_options`)
            // instead of fabricating a bogus sub-0 entry.
            object
                .custom_options
                .insert(key.to_string(), unquote(value));
        }
    }
}

fn apply_key(
    entry: &mut EdsSubEntry,
    key: &str,
    value: &str,
    idx: u16,
    warnings: &mut Vec<String>,
) {
    match key {
        "ParameterName" => entry.name = unquote(value),
        "DataType" => {
            if let Some(dt_idx) = parse_int(value) {
                // BUGS.md L1: an out-of-u16 DataType index would silently
                // truncate through `as u16`; range-check and fall back with
                // a warning instead. python-canopen warns and falls back to
                // DOMAIN for an unknown index (`eds.py:322-335`) — silently
                // treating it as Unsigned32 changed the SDO width and PDO
                // semantics (BUGS.md 十M12).
                match u16::try_from(dt_idx).ok().and_then(DataType::from_index) {
                    Some(dt) => entry.data_type = dt,
                    None => {
                        warnings.push(format!(
                            "unknown DataType={value} at [{idx:#06x}] - using Domain"
                        ));
                        entry.data_type = DataType::Domain;
                    }
                }
            }
        }
        "AccessType" => {
            match value.to_ascii_lowercase().as_str() {
                "ro" | "const" => entry.access = AccessType::Const,
                "wo" => entry.access = AccessType::Wo,
                // rwr/rww behave as Rw for the SDO-visible surface.
                "rw" | "rwr" | "rww" => entry.access = AccessType::Rw,
                // Unknown spellings default to Rw like before but are now
                // reported instead of silently swallowed (BUGS.md P3-52).
                other => warnings.push(format!(
                    "unknown AccessType {other:?} at [{idx:#06x}] - using Rw"
                )),
            }
        }
        // Unparsable-as-number defaults (bare strings like `01 A2 ff`,
        // `Hello`) are preserved verbatim as Str expressions so typed
        // consumers (OCTET/DOMAIN/VisibleString) can decode them.
        "DefaultValue" => {
            // An empty `DefaultValue=` means "no default", not a malformed
            // string: python-canopen warns `Invalid DefaultValue ''` and
            // keeps `None`; rejecting it aborted whole official files (the
            // 0x1003 subentries of DS301_profile.eds, BUGS.md 十H4).
            if value.is_empty() {
                warnings.push(format!(
                    "empty DefaultValue at [{idx:#06x}] ignored (no default)"
                ));
            } else {
                entry.default =
                    Some(OdExpr::parse(value).unwrap_or_else(|| OdExpr::Str(unquote(value))));
                // Keep the original spelling for textual consumers
                // (VisibleString) — python returns the raw string
                // (`eds.py:249-250`), so `0x41` must not become `"65"`.
                entry.default_raw = Some(value.to_string());
            }
        }
        // BUGS.md 八L9: any non-zero value marks the object mappable, like
        // python-canopen's `bool(int(...))` (`eds.py:337`). CANFestival's
        // `PDOMapping=2` ("RPDO only") is therefore accepted as mappable
        // for both directions rather than silently rejected.
        "PDOMapping" => entry.pdo_mappable = parse_int(value).unwrap_or(0) != 0,
        // Unknown subentry keys are preserved verbatim for round-tripping
        // (BUGS.md #19), mirroring python-canopen's per-variable
        // `custom_options` instead of silently dropping vendor data.
        _ => {
            entry.custom_options.insert(key.to_string(), unquote(value));
        }
    }
}

fn unquote(v: &str) -> String {
    v.trim().trim_matches('"').to_string()
}

impl EdsSubEntry {
    /// Materialises the default value for the dictionary. Missing defaults
    /// become zero values of the declared type.
    ///
    /// # Errors
    /// Propagates encoding errors from type/range validation.
    pub fn to_value(&self, node_id: u8) -> Result<Value> {
        use DataType as Dt;
        // Non-numeric types read their defaults from dedicated expression
        // variants (`BUGS.md` P1-6).
        if self.data_type == Dt::VisibleString {
            return Ok(match &self.default {
                Some(OdExpr::Str(s)) => Value::Str(s.clone()),
                // Bare numeric / node-relative defaults keep their original
                // textual form, like python-canopen's `_decode_from_eds`
                // (`eds.py:249-250` returns the raw string): `0x41` stays
                // `"0x41"` and `1e3` stays `"1e3"` (十M25). Quoted strings
                // take the normal `Str` path above.
                Some(_) => Value::Str(self.default_raw.clone().unwrap_or_default()),
                None => Value::Str(String::new()),
            });
        }
        if matches!(
            self.data_type,
            Dt::OctetString | Dt::UnicodeString | Dt::Domain
        ) {
            let bytes = match &self.default {
                Some(OdExpr::Str(s)) => crate::eds::hex_bytes(s),
                _ => Vec::new(),
            };
            return Ok(Value::Bytes(bytes));
        }
        if matches!(self.data_type, Dt::Real32 | Dt::Real64) {
            let f = match &self.default {
                Some(OdExpr::Float(f)) => *f,
                Some(expr) => expr.resolve(node_id) as f64,
                None => 0.0,
            };
            return Ok(Value::Real(f));
        }
        // TIME types are fixed 6-byte little-endian values: `encode_as`/
        // `decode_le` treat them as `Value::Bytes`, so the default must be
        // materialised as bytes or every SDO access to the entry fails with
        // `TypeMismatch` (`BUGS.md` H1). Hex-string defaults are decoded
        // verbatim; numeric defaults encode little-endian (ms since
        // midnight in the low 32 bits, days in the high 16).
        if matches!(self.data_type, Dt::TimeOfDay | Dt::TimeDifference) {
            let bytes = match &self.default {
                Some(OdExpr::Str(s)) => crate::eds::hex_bytes(s),
                _ => (self
                    .default
                    .as_ref()
                    .map_or(0, |e| e.resolve(node_id))
                    .max(0) as u64)
                    .to_le_bytes()[..6]
                    .to_vec(),
            };
            // BUGS.md L4: TIME types are fixed 6-byte values. A hex-string
            // default of any other width must fail here at build time —
            // previously the early `return` skipped the shared
            // `encode_as` validation below, so a 3-byte default was
            // accepted and then aborted every later SDO access with
            // `BadLength` (build-time failure is fail-fast, like the
            // numeric path's range check).
            if bytes.len() != 6 {
                return Err(Error::BadLength {
                    declared: 6,
                    actual: bytes.len(),
                });
            }
            return Ok(Value::Bytes(bytes));
        }

        // Integer defaults. A float expression (`DefaultValue=1e9`) used to
        // resolve to 0 via `OdExpr::resolve`, silently corrupting valid
        // defaults (`BUGS.md` H2). Convert with a range check instead.
        let raw = match (&self.default, self.data_type) {
            (Some(OdExpr::Float(f)), dt) if !matches!(dt, Dt::Boolean) => {
                match float_default_to_int(*f, dt) {
                    Some(v) => v,
                    None => {
                        let (min, max) = int_bounds(dt).unwrap_or((0, i128::MAX));
                        return Err(Error::ValueOutOfRange {
                            value: f.trunc().clamp(min as f64, max as f64) as i64,
                            min: to_i64_bound(min),
                            max: to_i64_bound(max),
                        });
                    }
                }
            }
            // BUGS.md B3-3: 整数类型条目的字符串默认（如超出 i64 的数值字面量
            // `0xFFFFFFFFFFFFFFFF`）此前退化为 Str 并在下方 `resolve` 静默得 0。
            // 改为按完整宽度解析并做范围校验：越界或不可解析则显式报错，而非
            // 静默零值。字符串/可见字符串类型的默认在此分支之外由各自路径处理。
            (Some(OdExpr::Str(s)), dt) if is_integer_type(dt) => parse_integer_default(s, dt)?,
            _ => self
                .default
                .as_ref()
                .map_or(0i128, |e| i128::from(e.resolve(node_id))),
        };
        let value = match self.data_type {
            Dt::Boolean => Value::Bool(raw != 0),
            Dt::Integer8 => Value::Int(signed_default(raw, Dt::Integer8)?),
            Dt::Integer16 => Value::Int(signed_default(raw, Dt::Integer16)?),
            Dt::Integer24 => Value::Int(signed_default(raw, Dt::Integer24)?),
            Dt::Integer32 => Value::Int(signed_default(raw, Dt::Integer32)?),
            Dt::Integer40 => Value::Int(signed_default(raw, Dt::Integer40)?),
            Dt::Integer48 => Value::Int(signed_default(raw, Dt::Integer48)?),
            Dt::Integer56 => Value::Int(signed_default(raw, Dt::Integer56)?),
            Dt::Integer64 => Value::Int(raw as i64),
            dt if dt.fixed_size().is_some() => {
                // BUGS.md M2: 无符号类型收到负默认（如 `DefaultValue=-1`）
                // 应 fail-fast 报错，与 codegen 的 `int_default_or_panic`
                // 行为一致；旧实现静默 `raw.max(0)` 归零，两端构建出口行为
                // 不一致。
                if raw < 0 {
                    let max = int_bounds(dt).map_or(i128::MAX, |(_, m)| m);
                    return Err(Error::ValueOutOfRange {
                        value: raw as i64,
                        min: 0,
                        max: to_i64_bound(max),
                    });
                }
                Value::Uint(raw as u64)
            }
            _ => Value::Uint(0),
        };
        // BUGS.md L1: 构建时校验默认值范围——此前 Literal 默认值不校验，
        // 越界默认值（如 Unsigned8 DefaultValue=300）能构建成功但任何
        // SDO 读写都失败（读写不一致）；与 H2 的浮点路径一样 fail-fast。
        let mut scratch = [0u8; 8];
        value
            .encode_as(self.data_type, &mut scratch)
            .map_err(|e| match e {
                Error::ValueOutOfRange { .. } => e,
                other => other,
            })?;
        Ok(value)
    }
}

impl Eds {
    /// Builds a runtime [`ObjectDictionary`] bound to `node_id`.
    ///
    /// Objects without any parsable subentries are skipped. String, blob
    /// and float defaults are materialised from their quoted/hex/decimal
    /// expressions (`BUGS.md` P1-6/P1-7).
    ///
    /// # Errors
    /// Propagates per-entry validation errors.
    pub fn to_object_dictionary(&self, node_id: u8) -> Result<ObjectDictionary> {
        let mut od = ObjectDictionary::new();
        for object in self.objects.values() {
            let expanded = expand_compact_sub_obj(object);
            let subs_iter: Box<dyn std::iter::Iterator<Item = (&u8, &EdsSubEntry)>> =
                match &expanded {
                    Some(subs) => Box::new(subs.iter()),
                    None => Box::new(object.subs.iter()),
                };
            for (sub, se) in subs_iter {
                od.insert(
                    Address::new(object.index, *sub),
                    Entry::new(se.name.clone(), se.data_type, se.access, {
                        Storage::Value(se.to_value(node_id)?)
                    })
                    .pdo_mappable_if(se.pdo_mappable),
                );
            }
        }
        Ok(od)
    }
}

/// Expands a `CompactSubObj=N` object: the parser lands the `idx`
/// section's inline definition in the sub-0 slot — that IS the template.
/// Expansion replaces it: sub0 becomes the element count and subs 1..=N
/// are deep copies of the template. Shared by the runtime dictionary
/// builder and the code generator (`BUGS.md` P1-8).
///
/// Returns `Some(empty)` — not `None` — for a `CompactSubObj` beyond the
/// u8 sub-index limit (n > 255): the parse already warned ("expansion
/// skipped", `BUGS.md` P3-54), and an empty expansion skips the object
/// cleanly instead of degrading it to its lone sub-0 template (`BUGS.md`
/// L1).
#[must_use]
pub fn expand_compact_sub_obj(object: &EdsObject) -> Option<BTreeMap<u8, EdsSubEntry>> {
    let n = object.compact_sub_obj?;
    if n > 255 {
        return Some(BTreeMap::new());
    }
    // Template selection: an inline definition in the `[index]` section
    // (canopen-rs / CANopenNode CompactSubObj convention) lands at sub 0;
    // python-canopen instead copies `[indexsub1]` (`eds.py:177`). Prefer the
    // inline sub-0 template when present so explicit `[indexsubN]` entries
    // stay genuine overrides, and fall back to sub 1 for the python layout.
    let template = match object.subs.get(&0) {
        Some(inline) => inline.clone(),
        None => object.subs.get(&1)?.clone(),
    };
    let mut subs = BTreeMap::new();
    subs.insert(
        0u8,
        EdsSubEntry {
            name: "Number of entries".to_string(),
            data_type: DataType::Unsigned8,
            access: AccessType::Ro,
            default: Some(OdExpr::Literal(i64::from(n))),
            default_raw: None,
            pdo_mappable: false,
            custom_options: std::collections::BTreeMap::new(),
        },
    );
    for i in 1..=n {
        let sub = u8::try_from(i).ok()?;
        // An explicit `[indexsubN]` definition wins over the template
        // (python merges both into one object via `add_member`,
        // `eds.py:402-408`); the `[indexName]` name, when given, labels the
        // slot either way.
        let mut entry = object
            .subs
            .get(&sub)
            .cloned()
            .unwrap_or_else(|| template.clone());
        if let Some(name) = object.sub_names.get(&sub) {
            entry.name = name.clone();
        }
        subs.insert(sub, entry);
    }
    Some(subs)
}

trait PdoMappableIf {
    fn pdo_mappable_if(self, yes: bool) -> Self;
}

impl PdoMappableIf for Entry {
    fn pdo_mappable_if(mut self, yes: bool) -> Self {
        if yes {
            self.pdo_mappable = true;
        }
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"
[DeviceComissioning]
NodeID=0x30

[1000]
ParameterName=Device type
DataType=0x0007
AccessType=ro
DefaultValue=0x00010192
PDOMapping=0

[1017]
ParameterName=Producer heartbeat time
DataType=0x0006
AccessType=rw
DefaultValue=500
PDOMapping=0

[6000]
ParameterName=AI values
ObjectType=0x8
SubNumber=2

[6000sub0]
ParameterName=Number of channels
DataType=0x0005
AccessType=ro
DefaultValue=1
PDOMapping=0

[6000sub1]
ParameterName=Channel 1
DataType=0x0003
AccessType=rw
DefaultValue=$NODEID+0x200
PDOMapping=1
"#;

    #[test]
    fn parses_sections_and_types() {
        let eds = parse(SAMPLE).unwrap();
        assert_eq!(eds.objects.len(), 3);
        let dev = &eds.objects[&0x1000].subs[&0];
        assert_eq!(dev.data_type, DataType::Unsigned32);
        assert_eq!(dev.access, AccessType::Const);
        assert_eq!(dev.default, Some(OdExpr::Literal(0x0001_0192)));
        // rwr/rww → Rw handled; plain rw stays rw:
        assert_eq!(eds.objects[&0x1017].subs[&0].access, AccessType::Rw);
    }

    #[test]
    fn nodeid_expression_resolves_lazily() {
        let eds = parse(SAMPLE).unwrap();
        let ch1 = &eds.objects[&0x6000].subs[&1];
        assert_eq!(ch1.default, Some(OdExpr::NodeRelative { offset: 0x200 }));
        assert!(ch1.pdo_mappable);
        let od = eds.to_object_dictionary(0x10).unwrap();
        assert_eq!(od.read(Address::new(0x6000, 1)).unwrap(), Value::Int(0x210));
        // Re-binding to another node id needs only a rebuild, not a re-parse.
        let od2 = eds.to_object_dictionary(0x20).unwrap();
        assert_eq!(
            od2.read(Address::new(0x6000, 1)).unwrap(),
            Value::Int(0x220)
        );
        assert_eq!(eds.configured_node_id, Some(OdExpr::Literal(0x30)));
    }

    #[test]
    fn hex_decimal_and_negative_literals() {
        assert_eq!(parse_int("0xAB"), Some(171));
        assert_eq!(parse_int("-42"), Some(-42));
        assert_eq!(parse_int("42"), Some(42));
        assert_eq!(parse_int("nope"), None);
        assert_eq!(
            OdExpr::parse("$NODEID+100"),
            Some(OdExpr::NodeRelative { offset: 100 })
        );
        assert_eq!(
            OdExpr::parse("$nodeid"),
            Some(OdExpr::NodeRelative { offset: 0 })
        );
    }

    #[test]
    fn compact_sub_obj_expands_into_count_and_copies() {
        let text = r#"
[6400]
ParameterName=AI inputs
ObjectType=0x8
CompactSubObj=4
DataType=0x0003
AccessType=rw
DefaultValue=$NODEID+0x100
PDOMapping=1
"#;
        let eds = parse(text).unwrap();
        let obj = &eds.objects[&0x6400];
        assert_eq!(obj.compact_sub_obj, Some(4));

        let od = eds.to_object_dictionary(0x10).unwrap();
        // sub0: element count
        assert_eq!(od.read(Address::new(0x6400, 0)).unwrap(), Value::Uint(4));
        // subs 1..=4: template copies, $NODEID resolved per build
        for sub in 1..=4u8 {
            assert_eq!(
                od.read(Address::new(0x6400, sub)).unwrap(),
                Value::Int(0x110)
            );
        }
        assert!(od.entry(Address::new(0x6400, 5)).is_none());
    }

    /// BUGS.md 十M24: `[indexName]` supplies `NrOfEntries` and per-subentry
    /// names, and explicit `[indexsubN]` definitions survive the expansion
    /// (python merges all three into one object, `eds.py:172-182,402-408`).
    #[test]
    fn name_section_merges_count_names_and_explicit_subs() {
        let text = r#"
[6400]
ParameterName=AI inputs
ObjectType=0x8
CompactSubObj=3
DataType=0x0003
AccessType=rw
DefaultValue=0

[6400Name]
NrOfEntries=3
1=First
2=Second
3=Third

[6400sub2]
ParameterName=Second
DataType=0x0005
AccessType=ro
DefaultValue=7
"#;
        let eds = parse(text).unwrap();
        let od = eds.to_object_dictionary(1).unwrap();
        assert_eq!(od.read(Address::new(0x6400, 0)).unwrap(), Value::Uint(3));
        // Template applied to 1 and 3, names from [6400Name].
        assert_eq!(od.read(Address::new(0x6400, 1)).unwrap(), Value::Int(0));
        assert_eq!(od.entry(Address::new(0x6400, 1)).unwrap().name, "First");
        assert_eq!(od.entry(Address::new(0x6400, 3)).unwrap().name, "Third");
        // The explicit sub2 definition wins over the template.
        assert_eq!(od.read(Address::new(0x6400, 2)).unwrap(), Value::Uint(7));
        assert_eq!(od.entry(Address::new(0x6400, 2)).unwrap().name, "Second");
    }

    /// python-canopen layout: no `CompactSubObj`, `[indexsub1]` is the
    /// template and `[indexName].NrOfEntries` the count.
    #[test]
    fn name_section_without_compact_expands_from_sub1() {
        let text = r#"
[6401]
ParameterName=Channels
ObjectType=0x8

[6401sub1]
ParameterName=Channel
DataType=0x0005
AccessType=rw
DefaultValue=9

[6401Name]
NrOfEntries=2
1=A
2=B
"#;
        let eds = parse(text).unwrap();
        let od = eds.to_object_dictionary(0).unwrap();
        assert_eq!(od.read(Address::new(0x6401, 0)).unwrap(), Value::Uint(2));
        assert_eq!(od.read(Address::new(0x6401, 1)).unwrap(), Value::Uint(9));
        assert_eq!(od.read(Address::new(0x6401, 2)).unwrap(), Value::Uint(9));
        assert_eq!(od.entry(Address::new(0x6401, 2)).unwrap().name, "B");
    }

    /// BUGS.md 十M25: VisibleString defaults keep the raw EDS spelling
    /// (`0x41` → `"0x41"`), like python-canopen's `_decode_from_eds`.
    #[test]
    fn visible_string_hex_default_keeps_raw_spelling() {
        let text = "[2001]\nParameterName=S\nDataType=0x0009\nAccessType=rw\nDefaultValue=0x41\n";
        let od = parse(text).unwrap().to_object_dictionary(1).unwrap();
        assert_eq!(
            od.read(Address::new(0x2001, 0)).unwrap(),
            Value::Str("0x41".to_string())
        );
    }

    /// BUGS.md 十M25: `NaN`/`inf` are valid FLOAT defaults (python uses
    /// plain `float(value)`) and must not silently degrade to 0.
    #[test]
    fn nan_and_inf_float_defaults_are_preserved() {
        let text = r#"
[2002]
ParameterName=NaNv
DataType=0x0011
AccessType=rw
DefaultValue=NaN

[2003]
ParameterName=Infv
DataType=0x0011
AccessType=rw
DefaultValue=inf
"#;
        let od = parse(text).unwrap().to_object_dictionary(1).unwrap();
        assert!(matches!(
            od.read(Address::new(0x2002, 0)).unwrap(),
            Value::Real(f) if f.is_nan()
        ));
        assert_eq!(
            od.read(Address::new(0x2003, 0)).unwrap(),
            Value::Real(f64::INFINITY)
        );
    }

    /// BUGS.md 十M15: a signed hex default beyond the type's unsigned range
    /// (`INTEGER8 0x100`) is rejected, not wrapped to a different value.
    #[test]
    fn signed_hex_magnitude_beyond_unsigned_range_is_rejected() {
        assert_eq!(wrap_signed_hex(0xFF, 8), Some(-1));
        assert_eq!(wrap_signed_hex(0x100, 8), None);
        assert_eq!(wrap_signed_hex(0xFFFF, 16), Some(-1));
        assert_eq!(wrap_signed_hex(0x10000, 16), None);
        assert_eq!(wrap_signed_hex(-129, 8), None);
        let text = "[2001]\nParameterName=X\nDataType=0x0002\nAccessType=rw\nDefaultValue=0x100\n";
        let eds = parse(text).unwrap();
        assert!(eds.to_object_dictionary(1).is_err());
    }

    #[test]
    fn comments_and_blank_lines_ignored() {
        let eds =
            parse("; header comment\n\n[1000]\n; c\nParameterName=X ; trailing\nDataType=0x0007\n")
                .unwrap();
        assert_eq!(eds.objects[&0x1000].name, "X");
    }

    #[test]
    fn quoted_semicolon_survives_comment_stripping() {
        // BUGS.md H2: the old `split(';')` truncated a quoted
        // `DefaultValue="a;b"` to `"a` — a `;` inside a `"…"` string is
        // data, not a comment (verified against python-configparser's
        // `inline_comment_prefixes=(';',)`). A `;` outside quotes preceded
        // by whitespace is still a comment.
        let eds = parse(
            "[1000]\nParameterName=X\nDataType=0x0009\nDefaultValue=\"a;b\"\n\
             [1001]\nParameterName=Y ; trailing\nDataType=0x0007\nDefaultValue=7\n",
        )
        .unwrap();
        let od = eds.to_object_dictionary(1).unwrap();
        assert_eq!(
            od.read(Address::new(0x1000, 0)).unwrap(),
            Value::Str("a;b".to_string())
        );
        assert_eq!(od.read(Address::new(0x1001, 0)).unwrap(), Value::Uint(7));
    }

    #[test]
    fn h3_node_relative_suffix_form_is_supported() {
        // BUGS.md 十H3: `1280+$NODEID` (the official sample.eds spelling)
        // must resolve like `$NODEID+1280`; whitespace and '+' are
        // transparent, matching python-canopen's `\+?\$NODEID\+?`.
        assert_eq!(
            OdExpr::parse("1280+$NODEID"),
            Some(OdExpr::NodeRelative { offset: 1280 })
        );
        assert_eq!(
            OdExpr::parse("$NODEID+1280"),
            Some(OdExpr::NodeRelative { offset: 1280 })
        );
        assert_eq!(
            OdExpr::parse("0x500+$nodeid"),
            Some(OdExpr::NodeRelative { offset: 0x500 })
        );
        assert_eq!(
            OdExpr::parse(" $NODEID + 0x20 "),
            Some(OdExpr::NodeRelative { offset: 0x20 })
        );
        assert_eq!(
            OdExpr::parse("+$NODEID"),
            Some(OdExpr::NodeRelative { offset: 0 })
        );
        assert_eq!(
            OdExpr::parse("$NODEID-0x100"),
            Some(OdExpr::NodeRelative { offset: -0x100 })
        );
        assert_eq!(
            OdExpr::parse("-0x100+$NODEID"),
            Some(OdExpr::NodeRelative { offset: -0x100 })
        );
        // Malformed expressions still fall back to a string expression.
        assert_eq!(OdExpr::parse("$NODEID+"), None);
        assert_eq!(OdExpr::parse("1+$NODEID+2"), None);
    }

    #[test]
    fn h3_official_sample_style_default_builds() {
        let eds = parse(
            "[1403sub1]\nParameterName=COB-ID\nDataType=0x0007\n\
             AccessType=rw\nDefaultValue=1280+$NODEID\n",
        )
        .unwrap();
        let od = eds.to_object_dictionary(16).unwrap();
        assert_eq!(od.read(Address::new(0x1403, 1)).unwrap(), Value::Uint(1296));
    }

    #[test]
    fn h4_empty_default_value_is_no_default() {
        let eds = parse(
            "[1003sub1]\nParameterName=E\nDataType=0x0007\n\
             AccessType=ro\nDefaultValue=\n",
        )
        .unwrap();
        assert!(eds
            .warnings
            .iter()
            .any(|w| w.contains("empty DefaultValue")));
        let od = eds.to_object_dictionary(1).unwrap();
        assert_eq!(od.read(Address::new(0x1003, 1)).unwrap(), Value::Uint(0));
    }

    #[test]
    fn l8_hash_full_line_comments_are_ignored() {
        let eds = parse(
            "#ParameterValue=0x1\n[2000]\nParameterName=X\nDataType=0x0007\n\
             #ParameterValue=0x2\n",
        )
        .unwrap();
        assert!(eds.objects[&0x2000].subs[&0].custom_options.is_empty());
    }

    #[test]
    fn l9_ffe_object_is_not_swallowed_by_commissioning() {
        let eds = parse(
            "[FFFE]\nParameterName=Vendor\nDataType=0x0007\n\
             AccessType=ro\nDefaultValue=5\n",
        )
        .unwrap();
        assert!(eds.objects.contains_key(&0xFFFE));
        let od = eds.to_object_dictionary(1).unwrap();
        assert_eq!(od.read(Address::new(0xFFFE, 0)).unwrap(), Value::Uint(5));
    }

    #[test]
    fn l10_unquoted_numeric_visible_string_default_keeps_text() {
        let eds = parse(
            "[2000]\nParameterName=S\nDataType=0x0009\n\
             AccessType=ro\nDefaultValue=123\n",
        )
        .unwrap();
        assert_eq!(
            eds.to_object_dictionary(1)
                .unwrap()
                .read(Address::new(0x2000, 0))
                .unwrap(),
            Value::Str("123".to_string())
        );
        let eds = parse(
            "[2001]\nParameterName=S\nDataType=0x0009\n\
             AccessType=ro\nDefaultValue=1.5\n",
        )
        .unwrap();
        assert_eq!(
            eds.to_object_dictionary(1)
                .unwrap()
                .read(Address::new(0x2001, 0))
                .unwrap(),
            Value::Str("1.5".to_string())
        );
    }

    #[test]
    fn m12_unknown_datatype_falls_back_to_domain_with_warning() {
        let eds = parse("[2000]\nParameterName=X\nDataType=0x0040\nAccessType=ro\n").unwrap();
        assert_eq!(eds.objects[&0x2000].subs[&0].data_type, DataType::Domain);
        assert!(eds.warnings.iter().any(|w| w.contains("unknown DataType")));
    }

    #[test]
    fn m13_float_boundary_defaults_are_rejected() {
        // 2^64 as a float literal is out of range for Unsigned64…
        let eds = parse(
            "[2000]\nParameterName=X\nDataType=0x001B\n\
             AccessType=ro\nDefaultValue=1.8446744073709552e19\n",
        )
        .unwrap();
        assert!(eds.to_object_dictionary(1).is_err());
        // …and 2^63 is out of range for Integer64.
        let eds = parse(
            "[2001]\nParameterName=X\nDataType=0x0015\n\
             AccessType=ro\nDefaultValue=9.223372036854776e18\n",
        )
        .unwrap();
        assert!(eds.to_object_dictionary(1).is_err());
    }
}

#[cfg(test)]
mod audit_tests {
    use super::*;
    use canopen_core::datatypes::Value;

    /// P1-4: python-canopen 官方 sample 风格的非索引节不再炸整文件；
    /// 十六进制 subindex（[1018sub02]/sub0A）按 hex 解析。
    #[test]
    fn skips_non_index_sections_and_parses_hex_subs() {
        const EDS: &str = r#"
[FileInfo]
FileName=demo.eds
Description=demo

[DeviceInfo]
VendorName=ACME

[Comments]
Lines=1
Line1=hello

[DummyUsage]
Dummy0007=1

[MandatoryObjects]
SupportedObjects=1

[1000]
ParameterName=Device type
DataType=0x0007
AccessType=ro

[1018sub02]
ParameterName=Product code
DataType=0x0007
AccessType=ro
DefaultValue=0xDEAD

[1018sub0A]
ParameterName=Hex sub ten
DataType=0x0005
AccessType=rw
DefaultValue=3
"#;
        let eds = parse(EDS).unwrap();
        assert!(eds.objects.contains_key(&0x1000));
        let obj = &eds.objects[&0x1018];
        assert!(obj.subs.contains_key(&2), "hex sub02 must decode");
        assert!(obj.subs.contains_key(&10), "hex sub0A must decode as 10");
        assert_eq!(
            Eds {
                objects: eds.objects.clone(),
                configured_node_id: None,
                warnings: Vec::new(),
            }
            .to_object_dictionary(1)
            .unwrap()
            .read(Address::new(0x1018, 10))
            .unwrap(),
            Value::Uint(3)
        );
    }

    /// P1-5: `$NODEID` 支持带符号偏移（含前置 `+`）。
    #[test]
    fn nodeid_offsets_may_be_negative() {
        assert_eq!(
            OdExpr::parse("$NODEID-0x100"),
            Some(OdExpr::NodeRelative { offset: -256 })
        );
        assert_eq!(
            OdExpr::parse("+$NODEID+8"),
            Some(OdExpr::NodeRelative { offset: 8 })
        );
        assert_eq!(OdExpr::parse("$NODEID").unwrap().resolve(5), 5);
        assert_eq!(OdExpr::parse("$nodeid - 4").unwrap().resolve(5), 1);
        // 负偏移落进默认值：COB-ID 场景（如 0x800 区段）不再静默归零。
        const EDS: &str = r#"
[1003sub1]
ParameterName=CobId
DataType=0x0007
AccessType=rw
DefaultValue=$NODEID+0x480
"#;
        let eds = parse(EDS).unwrap();
        let od = eds.to_object_dictionary(5).unwrap();
        assert_eq!(
            od.read(Address::new(0x1003, 1)).unwrap(),
            Value::Uint(0x485)
        );
    }

    /// P1-6/P1-7: 字符串/OCTET/浮点/有符号十六进制默认值全部落地。
    #[test]
    fn typed_defaults_materialise() {
        const EDS: &str = r#"
[2001]
ParameterName=Name
DataType=0x0009
AccessType=ro
DefaultValue="My Device"

[2002]
ParameterName=Blob
DataType=0x000F
AccessType=ro
DefaultValue=01 A2 ff

[2003]
ParameterName=Gain
DataType=0x0008
AccessType=rw
DefaultValue=1.5

[2004]
ParameterName=Signed16Hex
DataType=0x0003
AccessType=rw
DefaultValue=0xFFFF

[2005]
ParameterName=Signed32Min
DataType=0x0004
AccessType=rw
DefaultValue=0x80000000
"#;
        let od = parse(EDS).unwrap().to_object_dictionary(1).unwrap();
        assert_eq!(
            od.read(Address::new(0x2001, 0)).unwrap(),
            Value::Str("My Device".into())
        );
        assert_eq!(
            od.read(Address::new(0x2002, 0)).unwrap(),
            Value::Bytes(vec![1, 0xA2, 0xFF])
        );
        assert_eq!(od.read(Address::new(0x2003, 0)).unwrap(), Value::Real(1.5));
        assert_eq!(od.read(Address::new(0x2004, 0)).unwrap(), Value::Int(-1));
        assert_eq!(
            od.read(Address::new(0x2005, 0)).unwrap(),
            Value::Int(i32::MIN as i64)
        );
    }

    /// BUGS.md B3-3：整数类型条目的字符串默认值若超出 i64 范围（如
    /// `0xFFFFFFFFFFFFFFFF`），此前退化为 `Str` 并在 `to_value` 静默得 0；
    /// 现应显式报错（`ValueOutOfRange`）而非静默零值。
    #[test]
    fn b3_3_overflow_integer_default_reports_error() {
        const EDS: &str = r#"
[2006]
ParameterName=Huge
DataType=0x0005
AccessType=ro
DefaultValue=0xFFFFFFFFFFFFFFFF
"#;
        let od = parse(EDS).expect("EDS 解析成功");
        let err = od.to_object_dictionary(1).expect_err("超界整数默认应报错");
        assert!(
            matches!(err, Error::ValueOutOfRange { .. }),
            "超界整数默认应显式报错，实际: {err:?}"
        );
    }

    /// BUGS.md B3-3 反向：合法范围内的整数默认仍正常物化（无回归）。
    #[test]
    fn b3_3_valid_integer_default_still_materialises() {
        const EDS: &str = r#"
[2007]
ParameterName=Ok
DataType=0x0005
AccessType=ro
DefaultValue=200
"#;
        let od = parse(EDS).unwrap().to_object_dictionary(1).unwrap();
        assert_eq!(od.read(Address::new(0x2007, 0)).unwrap(), Value::Uint(200));
    }

    /// H1: TIME_OF_DAY/TIME_DIFFERENCE 默认值必须是 6 字节 `Value::Bytes`
    /// （与 `encode_as`/`decode_le` 一致），否则任何 SDO 访问都
    /// `TypeMismatch`；数值默认值按小端编码、缺失时补零。
    #[test]
    fn time_defaults_materialise_as_six_byte_bytes() {
        const EDS: &str = r#"
[2001]
ParameterName=Clock
DataType=0x000C
AccessType=rw
DefaultValue=01 02 03 04 05 06

[2002]
ParameterName=Delta
DataType=0x000D
AccessType=rw
DefaultValue=0x0102

[2003]
ParameterName=NoDefault
DataType=0x000C
AccessType=rw
"#;
        let od = parse(EDS).unwrap().to_object_dictionary(1).unwrap();
        // hex 字符串默认值原样解码
        assert_eq!(
            od.read(Address::new(0x2001, 0)).unwrap(),
            Value::Bytes(vec![1, 2, 3, 4, 5, 6])
        );
        // 数值默认值小端编码为 6 字节
        assert_eq!(
            od.read(Address::new(0x2002, 0)).unwrap(),
            Value::Bytes(vec![0x02, 0x01, 0, 0, 0, 0])
        );
        // 无默认值 → 6 个零字节
        assert_eq!(
            od.read(Address::new(0x2003, 0)).unwrap(),
            Value::Bytes(vec![0; 6])
        );
        // 编码往返必须成功（修复前是 TypeMismatch）
        let mut buf = [0u8; 8];
        assert_eq!(
            Value::Bytes(vec![1, 2, 3, 4, 5, 6])
                .encode_as(DataType::TimeOfDay, &mut buf)
                .unwrap(),
            6
        );
    }

    /// L4: TIME 类型的 hex 默认值宽度必须在构建期校验——3 字节默认值
    /// 此前漏过构建期检查、直到每次 SDO 访问才 `BadLength`。
    #[test]
    fn time_defaults_reject_wrong_hex_width_at_build() {
        const EDS: &str = r#"
[2001]
ParameterName=Clock
DataType=0x000C
AccessType=rw
DefaultValue=01 02 03
"#;
        let err = parse(EDS).unwrap().to_object_dictionary(1).unwrap_err();
        assert!(matches!(
            err,
            canopen_core::types::Error::BadLength {
                declared: 6,
                actual: 3
            }
        ));
        // 正确宽度仍然通过。
        const OK: &str = r#"
[2001]
ParameterName=Clock
DataType=0x000C
AccessType=rw
DefaultValue=01 02 03 04 05 06
"#;
        assert!(parse(OK).unwrap().to_object_dictionary(1).is_ok());
    }

    /// L5: `[index]Name` 后缀头（python-canopen `eds.py:172`）必须被宽松
    /// 解析为普通索引——修复前整个 section 被当作未知头静默跳过、字段丢失。
    #[test]
    fn name_suffixed_headers_parse_leniently() {
        const EDS: &str = r#"
[2000Name]
ParameterName=Object
DataType=0x0007
AccessType=rw
DefaultValue=42

[2001Sub1Name]
ParameterName=SubOne
DataType=0x0005
AccessType=rw
DefaultValue=7
"#;
        let eds = parse(EDS).unwrap();
        // 顶层 [2000Name] → 对象 0x2000 的字段全部保留。
        assert_eq!(eds.objects[&0x2000].subs[&0].name, "Object");
        // sub 头带 Name 后缀同样宽松（sub 部分先剥离 name 再按 hex 解析）。
        let od = eds.to_object_dictionary(1).unwrap();
        assert_eq!(od.read(Address::new(0x2001, 1)).unwrap(), Value::Uint(7));
    }

    /// H2: 整型条目的浮点/科学计数默认值（`DefaultValue=1e9`）被转换并
    /// 范围校验，而不是静默归零；越界值报 `ValueOutOfRange`。
    #[test]
    fn float_defaults_on_integers_convert_with_range_check() {
        const EDS: &str = r#"
[2001]
ParameterName=Big
DataType=0x0007
AccessType=rw
DefaultValue=1e9

[2002]
ParameterName=Tiny
DataType=0x0005
AccessType=rw
DefaultValue=3.0
"#;
        let od = parse(EDS).unwrap().to_object_dictionary(1).unwrap();
        assert_eq!(
            od.read(Address::new(0x2001, 0)).unwrap(),
            Value::Uint(1_000_000_000)
        );
        assert_eq!(od.read(Address::new(0x2002, 0)).unwrap(), Value::Uint(3));
    }

    /// H2 越界：Unsigned8 容不下 1e9 —— 构建时报错而非静默 0。
    #[test]
    fn out_of_range_float_default_rejected_at_build() {
        const EDS: &str = r#"
[2003]
ParameterName=Overflow
DataType=0x0005
AccessType=rw
DefaultValue=1e9
"#;
        assert!(parse(EDS).unwrap().to_object_dictionary(1).is_err());
    }

    /// BUGS.md M1：`int_bounds` 对 `Unsigned64` 上界曾截成 `i64::MAX`，
    /// 导致合法的 U64 默认（如 `0xFFFFFFFFFFFFFFFF` = `u64::MAX`）被误判越界
    /// 拒绝。修复后应收下并物化为 `Value::Uint(u64::MAX)`。
    #[test]
    fn m1_u64_max_default_materialises() {
        const EDS: &str = r#"
[2008]
ParameterName=Huge64
DataType=0x001B
AccessType=ro
DefaultValue=0xFFFFFFFFFFFFFFFF
"#;
        let od = parse(EDS).unwrap().to_object_dictionary(1).unwrap();
        assert_eq!(
            od.read(Address::new(0x2008, 0)).unwrap(),
            Value::Uint(u64::MAX)
        );
    }

    /// BUGS.md M1：U64 上界现在确实是 `u64::MAX`，`0x1_0000_0000_0000_0000`
    /// （= u64::MAX + 1）应当越界报错，而非被截断接受。
    #[test]
    fn m1_u64_overflow_rejected() {
        const EDS: &str = r#"
[2009]
ParameterName=TooBig
DataType=0x001B
AccessType=ro
DefaultValue=0x10000000000000000
"#;
        assert!(parse(EDS).unwrap().to_object_dictionary(1).is_err());
    }

    /// BUGS.md M2：无符号类型收到负默认（如 `DefaultValue=-1`）运行时必须
    /// fail-fast 报错，而非静默归零——与 codegen `int_default_or_panic` 行为
    /// 一致（修复前两端构建出口行为不一致：运行时归零、codegen 崩溃）。
    #[test]
    fn m2_negative_unsigned_default_rejected_at_runtime() {
        const EDS: &str = r#"
[2010]
ParameterName=Neg
DataType=0x0005
AccessType=ro
DefaultValue=-1
"#;
        let err = parse(EDS).unwrap().to_object_dictionary(1).unwrap_err();
        assert!(
            matches!(err, Error::ValueOutOfRange { .. }),
            "负无符号默认应显式报错，实际：{err:?}"
        );
    }
}
