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

use std::collections::{BTreeMap, BTreeSet};

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
    /// `\\+?\\$NODEID\\+?` grammar (`docs/DESIGN-DECISIONS.md` P1-5).
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
    /// hex and decimal, offsets on **both** sides concatenated like python's
    /// `re.sub`, `docs/DESIGN-DECISIONS.md` EDS-15), hexadecimal/decimal integers and decimal
    /// floating-point values (`docs/DESIGN-DECISIONS.md` P1-5/P1-6).
    ///
    /// A **paired** pair of quotes is consumed as the string delimiter
    /// (`"a;b"` → `Str("a;b")`). python-canopen keeps the quotes because
    /// `RawConfigParser` never unquotes (`eds.py:249-250,365`) — a deliberate
    /// leniency recorded in `docs/EXTENSIONS.md` §2.6 (`docs/DESIGN-DECISIONS.md` EDS-16), and
    /// note that it runs *opposite* to `ParameterName`, whose quotes are kept
    /// (docs/DESIGN-DECISIONS.md N13). Real-world defaults carry the quotes far more often than
    /// not, so stripping them is the readable choice; the divergence is
    /// documented rather than accidental.
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
            // is the documented optional sign (`docs/DESIGN-DECISIONS.md` P1-5).
            if tail == "+" {
                return None;
            }
            let offset = match (before.is_empty(), after.is_empty()) {
                (true, true) => 0,
                (true, false) => parse_int(after)?,
                (false, true) => parse_int(before)?,
                // docs/DESIGN-DECISIONS.md EDS-15: offsets on **both** sides are *not* an
                // error. python-canopen deletes `\+?\$NODEID\+?` outright, so
                // the two literals end up **concatenated** and the sum with
                // the node id is taken afterwards (`eds.py:254-259`; measured:
                // `1+$NODEID+2` with node 5 reads back 17, `0x10+$NODEID+0x20`
                // raises `ValueError` because `int("0x100x20", 0)` is
                // invalid). Concatenate and let `parse_int` decide — the
                // previous outright rejection made one malformed-ish default
                // veto the whole file (and the comment blamed `int()`, which
                // never sees the string).
                (false, false) => {
                    let joined = format!("{before}{after}");
                    parse_int(&joined)?
                }
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
    ///
    /// The arithmetic runs in `i128` and saturates into `i64` on overflow:
    /// `DefaultValue=$NODEID+0x7FFFFFFFFFFFFFFF` used to panic in debug
    /// builds (or wrap silently in release) — it must do neither
    /// (`docs/DESIGN-DECISIONS.md` EDS-11). Callers that need the exact value (integer
    /// defaults) use [`Self::resolve_wide`].
    #[must_use]
    pub fn resolve(&self, node_id: u8) -> i64 {
        let wide = self.resolve_wide(node_id);
        i64::try_from(wide).unwrap_or(if wide.is_negative() {
            i64::MIN
        } else {
            i64::MAX
        })
    }

    /// Exact `i128` resolution against a concrete node id.
    ///
    /// Integer/float defaults resolve through this entry point so that an
    /// extreme `$NODEID±offset` reaches the per-type range check
    /// (`Error::ValueOutOfRange`) instead of overflowing (`docs/DESIGN-DECISIONS.md` EDS-11).
    #[must_use]
    pub fn resolve_wide(&self, node_id: u8) -> i128 {
        match self {
            Self::Literal(v) => i128::from(*v),
            Self::NodeRelative { offset } => i128::from(node_id) + i128::from(*offset),
            Self::Str(_) | Self::Float(_) => 0,
        }
    }
}

/// Decodes a whitespace-separated hex byte string ("01 A2 ff"); empty on
/// malformed input. Strictly mirrors python-canopen `bytes.fromhex`: any
/// non-hex character invalidates the whole string (previously stray letters
/// were silently filtered and the remainder paired up — `"Bed"` produced a
/// byte instead of nothing, `docs/DESIGN-DECISIONS.md` P3-65). Use [`hex_bytes_checked`]
/// when the caller must distinguish "empty blob" from "malformed"
/// (`docs/DESIGN-DECISIONS.md` N10).
#[must_use]
pub fn hex_bytes(raw: &str) -> Vec<u8> {
    hex_bytes_checked(raw).unwrap_or_default()
}

/// Strict [`hex_bytes`]: `None` when `raw` is not an even-length hex byte
/// string (optional whitespace between pairs). python-canopen raises
/// `ValueError` there and the caller warns + drops the default
/// (`eds.py:365-372`); the EDS build records that warning at parse time
/// (`docs/DESIGN-DECISIONS.md` N10).
#[must_use]
pub fn hex_bytes_checked(raw: &str) -> Option<Vec<u8>> {
    let clean: String = raw.split_whitespace().collect::<Vec<_>>().join("");
    if clean.len() % 2 != 0 || !clean.chars().all(|c| c.is_ascii_hexdigit()) {
        return None;
    }
    Some(
        (0..clean.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&clean[i..i + 2], 16).unwrap_or(0))
            .collect(),
    )
}

/// Normalises an unsigned hex literal into its signed two's-complement
/// value at `bits` width (`INTEGER16 DefaultValue=0xFFFF` → -1). python-canopen
/// uses the same arithmetic only for LowLimit/HighLimit
/// (`_signed_int_from_hex`, `eds.py:222-243,343,355`); its **DefaultValue**
/// path goes through `_decode_from_eds → int(value, 0)` (`:246-259`) and later
/// fails in `encode_raw` (`objectdictionary/__init__.py:475-491`) — i.e. the
/// wire bytes agree while python fails late, so this is a documented
/// intentional difference (`docs/DESIGN-DECISIONS.md` EDS-07, P1-7/P2-14). Shared by the
/// runtime EDS path and the code generator so both build exits agree.
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
/// defaults (`docs/DESIGN-DECISIONS.md` H2 / M1). 返回 `i128` 以容纳 `Unsigned64` 的完整上界
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
        // 注意：U64 上界是 `u64::MAX`，不能用 `i64::MAX`（docs/DESIGN-DECISIONS.md M1）。
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
/// default (`docs/DESIGN-DECISIONS.md` H2). Shared by the runtime EDS path and the code
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
    // codegen (docs/DESIGN-DECISIONS.md 十M13). `f64 as i128` saturates, so values beyond
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

/// 将字符串形式的整数默认值解析为 `i64` 并按类型范围校验（docs/DESIGN-DECISIONS.md B3-3）。
/// 越界或不可解析（如非数字字符串）显式报错，不再静默退化为 0。
///
/// # Errors
/// 数值超出 `int_bounds(dt)` 范围或无法解析为整数时返回 [`Error::ValueOutOfRange`]。
pub fn parse_integer_default(s: &str, dt: DataType) -> Result<i128> {
    let s = s.trim();
    let (min, max) = int_bounds(dt).unwrap_or((i128::MIN, i128::MAX));
    let (negative, digits) = match s.strip_prefix('-') {
        Some(rest) => (true, rest.trim()),
        None => (false, s.strip_prefix('+').map_or(s, str::trim_start)),
    };
    // docs/DESIGN-DECISIONS.md 七E4: python's `int(value, 0)` also accepts `0b`/`0o` prefixes
    // and `_` digit separators; share the magnitude parser with `parse_int`.
    let magnitude = parse_int_magnitude(digits).ok_or(Error::ValueOutOfRange {
        value: 0,
        min: to_i64_bound(min),
        max: to_i64_bound(max),
    })?;
    let v = if negative { -magnitude } else { magnitude };
    if v < min || v > max {
        return Err(Error::ValueOutOfRange {
            value: v as i64,
            min: to_i64_bound(min),
            max: to_i64_bound(max),
        });
    }
    Ok(v)
}

/// Parses a Python `int(x, 0)`-style magnitude: decimal, or `0x`/`0b`/`0o`
/// prefixed, with optional `_` digit separators (`docs/DESIGN-DECISIONS.md` 七E4). The
/// magnitude is returned unsigned (sign handling stays with the caller).
fn parse_int_magnitude(digits: &str) -> Option<i128> {
    let clean: String = digits.chars().filter(|c| *c != '_').collect();
    let (radix, body) = if let Some(hex) = clean
        .strip_prefix("0x")
        .or_else(|| clean.strip_prefix("0X"))
    {
        (16, hex)
    } else if let Some(bin) = clean
        .strip_prefix("0b")
        .or_else(|| clean.strip_prefix("0B"))
    {
        (2, bin)
    } else if let Some(oct) = clean
        .strip_prefix("0o")
        .or_else(|| clean.strip_prefix("0O"))
    {
        (8, oct)
    } else {
        (10, clean.as_str())
    };
    i128::from_str_radix(body, radix).ok()
}

/// Parses a decimal or prefixed (`0x`/`0b`/`0o`, `_` separators allowed)
/// integer literal, optionally signed.
///
/// The magnitude is accumulated in `i128` before the sign is applied, so
/// `i64::MIN` (whose magnitude does not fit `i64`) parses instead of being
/// rejected as "out of range" (docs/DESIGN-DECISIONS.md 十L11).
#[must_use]
pub fn parse_int(raw: &str) -> Option<i64> {
    let raw = raw.trim();
    let (negative, digits) = match raw.strip_prefix('-') {
        Some(rest) => (true, rest.trim()),
        // docs/DESIGN-DECISIONS.md 八L10: python's `int(value, 0)` accepts an explicit '+'
        // (`+0x8000` == 32768); EDS files generated by such tooling must
        // parse instead of failing the whole import.
        None => (false, raw.strip_prefix('+').map_or(raw, str::trim_start)),
    };
    let magnitude = parse_int_magnitude(digits)?;
    let value = if negative { -magnitude } else { magnitude };
    i64::try_from(value).ok()
}

/// One subentry definition from the file.
#[derive(Debug, Clone)]
#[non_exhaustive]
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
    /// `ParameterValue` expression, when present. python-canopen parses it
    /// like `DefaultValue` and the local node serves it **in preference to**
    /// the default (`eds.py:375-382`, `node/local.py:99-107`); the
    /// dictionary build here materialises it the same way (`docs/DESIGN-DECISIONS.md` 七E2).
    pub parameter_value: Option<OdExpr>,
    /// Original spelling of `ParameterValue`, mirroring [`Self::default_raw`].
    pub parameter_value_raw: Option<String>,
    /// PDOMapping flag = 1.
    pub pdo_mappable: bool,
    /// Vendor-specific subentry keys this parser does not understand,
    /// preserved verbatim like python-canopen's per-variable
    /// `custom_options` (eds.py:398) so round-tripping an EDS never loses
    /// them (`docs/DESIGN-DECISIONS.md` #19).
    pub custom_options: std::collections::BTreeMap<String, String>,
    /// Deferred composite `DataType` index (`> 0x1B`, e.g. CANFestival's
    /// `0x20`): resolved after the whole file is parsed from the
    /// `[<index:X>sub1]` `DefaultValue` (python-canopen
    /// `objectdictionary/eds.py:322-335`, `G5`). `None` for every ordinary
    /// entry.
    pub deferred_data_type: Option<i64>,
    /// `LowLimit` of the entry, parsed like python-canopen's
    /// `var.min`: base-0 integer, and for a **signed** data type a hex literal
    /// in the upper half of the range is read as two's complement
    /// (`_signed_int_from_hex`, `eds.py:222-245,339-350`). A malformed literal
    /// leaves the field `None` and records a warning, like the reference's
    /// "Invalid LowLimit … ignoring".
    ///
    /// Purely informational here — python only *warns* about a value outside
    /// it when encoding (`objectdictionary/__init__.py:480-487`); it never
    /// rejects a write at the protocol level, and neither does this stack
    /// (`docs/DESIGN-DECISIONS.md` EDS-12).
    pub low_limit: Option<i64>,
    /// `HighLimit`, mirroring [`Self::low_limit`] (`eds.py:351-362`).
    pub high_limit: Option<i64>,
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
    /// CiA 306 `ObjectType` of the section (`0x0008` ARRAY, `0x0009` RECORD,
    /// `0x0007` VAR, `0x0002` DOMAIN, plus the type-definition codes
    /// `0x0005` DEFTYPE / `0x0006` DEFSTRUCT), when the key was present.
    ///
    /// It decides whether the section's inline CiA 306 keys describe a
    /// *variable* (python-canopen only calls `build_variable` for
    /// `ObjectType ∈ {VAR, DOMAIN}`, `eds.py:132-153`) or only the container —
    /// a RECORD/ARRAY header's `DataType`/`DefaultValue`/… are `_STANDARD_OPTIONS`
    /// there and are dropped, not turned into a sub-0 entry
    /// (`docs/DESIGN-DECISIONS.md` EDS-02). `None` means "no `ObjectType` key", which CiA 306
    /// §4.6.3.2 defines as VAR — the python default (`eds.py:122-126`).
    pub object_type: Option<i64>,
}

/// Metadata from the CiA 306 `[FileInfo]` section.
#[derive(Debug, Clone, Default)]
#[non_exhaustive]
pub struct EdsFileInfo {
    /// `FileName`
    pub file_name: String,
    /// `FileVersion`
    pub file_version: u32,
    /// `FileRevision`
    pub file_revision: u32,
    /// `EDSVersion`
    pub eds_version: String,
    /// `Description`
    pub description: String,
    /// `CreationTime`
    pub creation_time: String,
    /// `CreationDate`
    pub creation_date: String,
    /// `ModificationTime`
    pub modification_time: String,
    /// `ModificationDate`
    pub modification_date: String,
    /// `CreatedBy`
    pub created_by: String,
    /// `ModifiedBy`
    pub modified_by: String,
}

/// Metadata from the CiA 306 `[DeviceInfo]` section.
#[derive(Debug, Clone, Default)]
#[non_exhaustive]
pub struct EdsDeviceInfo {
    /// `VendorName`
    pub vendor_name: String,
    /// `VendorNumber`
    pub vendor_number: Option<u32>,
    /// `ProductName`
    pub product_name: String,
    /// `ProductNumber`
    pub product_number: Option<u32>,
    /// `RevisionNumber`
    pub revision_number: Option<u32>,
    /// `OrderCode`
    pub order_code: String,
    /// `BaudRate_10`
    pub baudrate_10: bool,
    /// `BaudRate_20`
    pub baudrate_20: bool,
    /// `BaudRate_50`
    pub baudrate_50: bool,
    /// `BaudRate_125`
    pub baudrate_125: bool,
    /// `BaudRate_250`
    pub baudrate_250: bool,
    /// `BaudRate_500`
    pub baudrate_500: bool,
    /// `BaudRate_800`
    pub baudrate_800: bool,
    /// `BaudRate_1000`
    pub baudrate_1000: bool,
    /// `SimpleBootUpMaster`
    pub simple_bootup_master: bool,
    /// `SimpleBootUpSlave`
    pub simple_bootup_slave: bool,
    /// `Granularity`
    pub granularity: u32,
    /// `DynamicChannelsSupported`
    pub dynamic_channels_supported: u32,
    /// `GroupMessaging`
    pub group_messaging: bool,
    /// `NrOfRXPDO`
    pub nr_of_rx_pdo: u32,
    /// `NrOfTXPDO`
    pub nr_of_tx_pdo: u32,
    /// `LSS_Supported`
    pub lss_supported: bool,
}

/// The `[Comments]` section: its `Line1..LineN` values in order.
#[derive(Debug, Clone, Default)]
#[non_exhaustive]
pub struct EdsComments {
    /// Text lines (`Line1`, `Line2`, …), sorted by their numeric suffix.
    pub lines: Vec<String>,
}

/// A parsed EDS/DCF file.
#[derive(Debug, Clone, Default)]
#[non_exhaustive]
pub struct Eds {
    /// Objects by index, ascending.
    pub objects: BTreeMap<u16, EdsObject>,
    /// `[DeviceComissioning] NodeID` raw expression, when present.
    pub configured_node_id: Option<OdExpr>,
    /// Non-fatal parse findings (unknown AccessType spellings, oversized
    /// CompactSubObj, …) surfaced instead of silently swallowed
    /// (`docs/DESIGN-DECISIONS.md` P3-52/P3-54).
    pub warnings: Vec<String>,
    /// `DefaultValue` of every `[<index:X>sub1]` section keyed by its index —
    /// the CANFestival composite-`DataType` dereference table (python-canopen
    /// `eds.py:322-335`, `G5`).
    pub type_defs: BTreeMap<u16, i64>,
    /// `[DummyUsage]` entries: index → data type index, both `1..=7`
    /// (`Dummy000n=1`). python-canopen creates a `const` VAR at index `n`
    /// with data type index `n` for each (`eds.py:105-113`); CANFestival
    /// EDS files map these dummy objects for PDO padding.
    pub dummy_usage: BTreeMap<u8, u8>,
    /// `[FileInfo]` metadata (CiA 306).
    pub file_info: EdsFileInfo,
    /// `[DeviceInfo]` metadata (CiA 306).
    pub device_info: EdsDeviceInfo,
    /// `[Comments]` free text (CiA 306).
    pub comments: EdsComments,
    /// `[DeviceComissioning] Baudrate`, when present.
    pub baudrate: Option<u32>,
}

/// Strips an inline comment from one EDS line: a `;` only starts a comment
/// when it is **outside** a `"…"` string **and** preceded by whitespace (or
/// at the very start of the line). `X ; trailing` loses its tail
/// (`docs/DESIGN-DECISIONS.md` H2) and `a;b` (no preceding whitespace) stays intact, matching
/// python's `RawConfigParser(inline_comment_prefixes=(';',))` on both.
///
/// The quote awareness is **this stack's own leniency**, deliberately *not*
/// configparser parity: configparser does not know about quotes, so python
/// reads `DefaultValue="a ;b"` back as `'"a'` (measured on 2.4.1) while this
/// function keeps the whole `a ;b` value. Keeping the tail is the recoverable
/// direction — a vendor file with a `;` inside a string still yields the value
/// it meant — and the divergence is recorded in `docs/EXTENSIONS.md` §11.1
/// (`docs/DESIGN-DECISIONS.md` EDS-18).
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
///
/// Currently **infallible**: a stray line without `=` (or an unknown option)
/// is recorded in [`Eds::warnings`] and parsing continues, mirroring
/// python-canopen's lenient `configparser` (docs/DESIGN-DECISIONS.md L1/API-08). Content
/// errors that *cannot* be represented in a dictionary (invalid or
/// out-of-range defaults) surface later from [`Eds::to_object_dictionary`] /
/// [`Eds::build`]; the `Result` return is kept for future strict modes.
pub fn parse(text: &str) -> Result<Eds> {
    let mut eds = Eds::default();
    let mut current = Section::Skipped;
    // docs/DESIGN-DECISIONS.md EDS-19: configparser joins **indented** lines into the previous
    // option's value with `\n` (measured on the local python: a
    // `ParameterName=Long name` plus an indented line reads back as
    // `'Long name\ncontinued here'`, and even an indented `key=value` line is
    // a continuation, never a new option). Remember where the last option was
    // applied so a continuation can re-apply the joined value.
    let mut last_option: Option<(u16, Option<u8>, String, String)> = None;
    // docs/DESIGN-DECISIONS.md EDS-23: `(index, 0)` pairs whose sub-0 entry this parser created
    // **optimistically** from an object section's CiA 306 keys (the
    // `ObjectType` line may come later). `finalize_object_types` folds exactly
    // those back out for a container; a sub 0 declared by its own
    // `[<index>sub0]` section is a real member and stays.
    let mut inline_sub0: BTreeSet<(u16, u8)> = BTreeSet::new();
    // Sub-0 entries declared by their own `[<index>sub0]` section.
    let mut explicit_sub0: BTreeSet<(u16, u8)> = BTreeSet::new();
    // Indentation of the line that opened the current option; configparser
    // treats a line as a continuation only when it is indented **more** than
    // that (an EDS file indented as a whole therefore keeps its options
    // separate, which is what the local python does).
    let mut key_indent = 0usize;
    // docs/DESIGN-DECISIONS.md EDS-20: configparser runs in `strict=True` mode and aborts the
    // import on a duplicate option/section. This parser keeps the lenient
    // last-wins behaviour — but the loss must be visible, so both are recorded
    // here.
    let mut seen_sections: BTreeSet<String> = BTreeSet::new();
    let mut seen_options: BTreeSet<(u16, Option<u8>, String)> = BTreeSet::new();

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
            // the file (`docs/DESIGN-DECISIONS.md` P1-4). `[index]Name` suffixed headers
            // are also treated as section headers (the trailing name is
            // ignored), matching configparser's lenient section regex
            // (`docs/DESIGN-DECISIONS.md` #14).
            let close = line.find(']').expect("checked above");
            let header = &line[1..close];
            // docs/DESIGN-DECISIONS.md EDS-20: a repeated section header is last-wins here and
            // an error in python (`DuplicateSectionError`).
            if !seen_sections.insert(header.to_ascii_lowercase()) {
                eds.warnings.push(format!(
                    "duplicate EDS section [{header}] - the later one wins (python-canopen aborts the import, docs/DESIGN-DECISIONS.md EDS-20)"
                ));
            }
            // EDS-22: python only accepts a four-digit hexadecimal
            // index section (`eds.py:99-101`, `^[0-9A-Fa-f]{4}$`) and ignores
            // the rest; a 1–3 digit header is this stack's lenient superset.
            // Accepting it is harmless (a real index needs 4 hex digits), so
            // only the divergence is surfaced.
            if !header.contains(']')
                && !header.trim_end().ends_with("name")
                && header.trim().len() != 4
                && header.trim().chars().all(|c| c.is_ascii_hexdigit())
            {
                eds.warnings.push(format!(
                    "EDS section [{header}] is not a 4-digit index - accepted anyway (python-canopen ignores it, docs/DESIGN-DECISIONS.md EDS-22)"
                ));
            }
            current = parse_header(header);
            if let Section::Object(index, Some(0), _) = current {
                explicit_sub0.insert((index, 0));
                inline_sub0.remove(&(index, 0));
                if let Some(object) = eds.objects.get_mut(&index) {
                    object.subs.remove(&0);
                }
            }
            seen_options.clear();
            last_option = None;
            key_indent = 0;
            continue;
        }
        if matches!(current, Section::Commissioning) {
            // docs/DESIGN-DECISIONS.md 八L8: `NodeIDSet` is a boolean commissioning flag
            // (CiA 306), not a node id — only `NodeID` carries the number.
            // python-canopen (`eds.py:97-100`) likewise reads only `NodeID`.
            if let Some((key, value)) = line.split_once('=') {
                let key = key.trim();
                let value = value.trim();
                if key.eq_ignore_ascii_case("NodeId") {
                    eds.configured_node_id = OdExpr::parse(value);
                } else if key.eq_ignore_ascii_case("Baudrate") {
                    eds.baudrate = parse_int(value).and_then(|v| u32::try_from(v).ok());
                }
            }
            continue;
        }
        if matches!(current, Section::FileInfo) {
            if let Some((key, value)) = line.split_once('=') {
                apply_file_info(&mut eds.file_info, key.trim(), value.trim());
            }
            continue;
        }
        if matches!(current, Section::DeviceInfo) {
            if let Some((key, value)) = line.split_once('=') {
                apply_device_info(&mut eds.device_info, key.trim(), value.trim());
            }
            continue;
        }
        if matches!(current, Section::Comments) {
            if let Some((key, value)) = line.split_once('=') {
                apply_comment(&mut eds.comments, key.trim(), value.trim());
            }
            continue;
        }
        if matches!(current, Section::DummyUsage) {
            // `Dummy000n=1` → dummy VAR at index n (python-canopen
            // `eds.py:105-113`). A missing/`0`/malformed key is skipped —
            // unlike python's `getint`, which aborts the whole import with
            // `NoOptionError` when a key is absent; skipping is the lenient
            // superset (docs/DESIGN-DECISIONS.md N11).
            if let Some((key, value)) = line.split_once('=') {
                let key = key.trim().to_ascii_lowercase();
                if let Some(digits) = key.strip_prefix("dummy") {
                    if let Ok(n) = digits.parse::<u8>() {
                        if (1..=7).contains(&n) && parse_int(value.trim()).unwrap_or(0) == 1 {
                            eds.dummy_usage.insert(n, n);
                        }
                    }
                }
            }
            continue;
        }
        let Section::Object(index, sub, name_section) = current else {
            continue;
        };
        // docs/DESIGN-DECISIONS.md EDS-19: an indented line continues the previous option's
        // value (`\n`-joined, continuation text stripped) — including an
        // indented `key=value`, which configparser does *not* treat as a new
        // option. Full-line comments were already dropped above, exactly like
        // configparser's `comment_prefixes`.
        let indent = raw_line.len() - raw_line.trim_start().len();
        if indent > key_indent && !line.is_empty() {
            if let Some((prev_index, prev_sub, prev_key, prev_value)) = last_option.take() {
                let joined = format!("{prev_value}\n{line}");
                reapply_option(
                    &mut eds,
                    prev_index,
                    prev_sub,
                    name_section,
                    &prev_key,
                    &joined,
                    explicit_sub0.contains(&(prev_index, 0)),
                );
                last_option = Some((prev_index, prev_sub, prev_key, joined));
                continue;
            }
        }
        let Some((key, value)) = line.split_once('=') else {
            // docs/DESIGN-DECISIONS.md L1: a stray line without `=` (e.g. a hand-edited
            // vendor section) must not abort the whole file — record a
            // warning and skip it, like python-canopen's lenient
            // configparser does. (python-canopen itself raises
            // `configparser.ParsingError` here; the leniency is documented in
            // `docs/EXTENSIONS.md` §11.1.)
            eds.warnings
                .push(format!("EDS line without '=' ignored: {raw_line:?}"));
            last_option = None;
            continue;
        };
        let key = key.trim();
        key_indent = indent;
        // docs/DESIGN-DECISIONS.md EDS-20: last-wins here, `DuplicateOptionError` in python.
        if !seen_options.insert((index, sub, key.to_ascii_lowercase())) {
            eds.warnings.push(format!(
                "duplicate EDS option {key} in section [{index:#06x}{} - the later value wins (python-canopen aborts the import, docs/DESIGN-DECISIONS.md EDS-20)",
                sub.map(|n| format!("Sub{n:02}")).unwrap_or_default()
            ));
        }
        // Surrounding whitespace is stripped (configparser does the same),
        // but quotes are **kept**: python-canopen reads through
        // `RawConfigParser` and never unquotes
        // (`objectdictionary/eds.py:313,365,376`), so `ParameterName="X"`
        // yields `"X"` verbatim (docs/DESIGN-DECISIONS.md N13).
        let value = value.trim();

        // G5: remember the `DefaultValue` of every `[<index:X>sub1]` section
        // — the CANFestival composite-DataType dereference table
        // (python-canopen `eds.py:322-335`). Captured before the object
        // borrow below; resolution happens after the whole file is read.
        let type_def_default = if sub == Some(1) && key == "DefaultValue" {
            parse_int(value).map(|v| (index, v))
        } else {
            None
        };

        let mut warnings = std::mem::take(&mut eds.warnings);
        let object = eds.objects.entry(index).or_insert_with(|| EdsObject {
            index,
            name: String::new(),
            subs: BTreeMap::new(),
            compact_sub_obj: None,
            sub_names: BTreeMap::new(),
            custom_options: BTreeMap::new(),
            object_type: None,
        });

        match sub {
            // `[index]Name` (python-canopen `eds.py:172-182`): `NrOfEntries`
            // is the CompactSubObj expansion count and numeric keys are the
            // per-subentry names. Both used to be dropped (NrOfEntries) or
            // parked in `custom_options` (names) where nothing consumed
            // them. Any *other* key falls through to the ordinary inline
            // object handling — vendor files also use Name-suffixed sections
            // for plain object definitions (`docs/DESIGN-DECISIONS.md` L5).
            None => {
                if name_section && key.eq_ignore_ascii_case("NrOfEntries") {
                    if let Some(n) = parse_int(value) {
                        let n = n.max(0);
                        object.compact_sub_obj = Some(u16::try_from(n).unwrap_or(u16::MAX));
                        if n > 255 {
                            warnings.push(format!(
                                "NrOfEntries={value} exceeds the 255-entry sub limit at [{index:#06x}] - expansion skipped (docs/DESIGN-DECISIONS.md P3-54)"
                            ));
                        }
                    }
                } else if name_section {
                    if let Ok(sub_idx) = key.parse::<u8>() {
                        object.sub_names.insert(sub_idx, value.to_string());
                    } else {
                        apply_object_key(
                            object,
                            key,
                            value,
                            index,
                            &mut warnings,
                            explicit_sub0.contains(&(index, 0)),
                        );
                    }
                } else {
                    let created = !object.subs.contains_key(&0);
                    apply_object_key(
                        object,
                        key,
                        value,
                        index,
                        &mut warnings,
                        explicit_sub0.contains(&(index, 0)),
                    );
                    if created && object.subs.contains_key(&0) {
                        inline_sub0.insert((index, 0));
                    }
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
                        parameter_value: None,
                        parameter_value_raw: None,
                        pdo_mappable: false,
                        custom_options: std::collections::BTreeMap::new(),
                        deferred_data_type: None,
                        low_limit: None,
                        high_limit: None,
                    }),
                    key,
                    value,
                    index,
                    sub,
                    &mut warnings,
                );
            }
        }
        eds.warnings.extend(warnings);
        if let Some((idx, v)) = type_def_default {
            eds.type_defs.insert(idx, v);
        }
        last_option = Some((index, sub, key.to_string(), value.to_string()));
    }
    finalize_object_types(&mut eds, &inline_sub0);
    resolve_composite_types(&mut eds);
    warn_invalid_defaults(&mut eds);
    // docs/DESIGN-DECISIONS.md EDS-03: an object section that never yields a subentry (e.g. it
    // only carries `ParameterName`, or its keys all went to
    // `custom_options`) produces **no** dictionary entry at all — python
    // aborts the import with `NoOptionError` for a missing `DataType`
    // (`eds.py:317-320`), this stack drops it silently. The drop is now
    // visible.
    for object in eds.objects.values() {
        if object.subs.is_empty() && object.compact_sub_obj.is_none() {
            eds.warnings.push(format!(
                "EDS object [{:#06x}] ({:?}) has no parsable subentry and is not created in the dictionary (python-canopen aborts the import, docs/DESIGN-DECISIONS.md EDS-03)",
                object.index, object.name
            ));
        }
    }
    Ok(eds)
}

/// Records warnings for defaults that python-canopen would log and ignore:
/// an OCTET_STRING/DOMAIN default that is not a valid hex byte string
/// (`bytes.fromhex` raises `ValueError`; the caller logs "Invalid
/// DefaultValue … ignoring" and leaves the default unset,
/// `eds.py:246-250,365-372`), and a float expression on a Boolean entry
/// (`int(value, 0)` raises there; `:246-256,367-373`, `docs/DESIGN-DECISIONS.md` 七E1).
/// The defaults are materialised as empty blob / `false` — but the loss is
/// now visible in [`Eds::warnings`] instead of silent (`docs/DESIGN-DECISIONS.md` N10).
/// `ParameterValue` is checked the same way as `DefaultValue`.
fn warn_invalid_defaults(eds: &mut Eds) {
    let mut warnings = Vec::new();
    for object in eds.objects.values() {
        for (sub, entry) in &object.subs {
            // The parsed `Str` variant has its paired quotes stripped; every
            // other expression is checked through the raw EDS spelling
            // (python always runs the conversion on the raw value).
            // docs/DESIGN-DECISIONS.md EDS-26/EDS-27: only a `ParameterValue` that can be
            // materialised is the effective source; an invalid one is reported
            // and the `DefaultValue` takes over.
            let prefer = entry.parameter_value.is_some() && entry.parameter_value_materialises();
            if entry.parameter_value.is_some() && !prefer {
                warnings.push(format!(
                    "invalid ParameterValue {:?} at [{:#06x}sub{sub}] - falling back to DefaultValue",
                    entry.parameter_value_raw.as_deref().unwrap_or_default(),
                    object.index
                ));
            }
            let source = if prefer {
                entry.parameter_value.as_ref()
            } else {
                entry.default.as_ref()
            };
            let source_raw = if prefer {
                entry.parameter_value_raw.as_deref()
            } else {
                entry.default_raw.as_deref()
            };
            let key = if prefer {
                "ParameterValue"
            } else {
                "DefaultValue"
            };
            if matches!(entry.data_type, DataType::OctetString | DataType::Domain) {
                let candidate = match source {
                    Some(OdExpr::Str(s)) => Some(s.as_str()),
                    Some(_) => source_raw,
                    None => None,
                };
                if let Some(raw) = candidate {
                    if hex_bytes_checked(raw).is_none() {
                        warnings.push(format!(
                            "invalid {key} {raw:?} at [{:#06x}sub{sub}] ignored",
                            object.index
                        ));
                    }
                }
            }
            // python's BOOLEAN conversion is `int(value, 0)`, so `1.5` is a
            // `ValueError` → warning + ignored default; both build exits
            // keep the dictionary value at `false` for that (七E1).
            if entry.data_type == DataType::Boolean && matches!(source, Some(OdExpr::Float(_))) {
                warnings.push(format!(
                    "invalid {key} {source_raw:?} at [{:#06x}sub{sub}] ignored",
                    object.index
                ));
            }
            if matches!(source, Some(OdExpr::NodeRelative { .. })) {
                continue;
            }
            if is_integer_type(entry.data_type) {
                if let Some(raw) = source_raw {
                    if parse_integer_default(raw, entry.data_type).is_err() {
                        warnings.push(format!(
                            "invalid {key} {raw:?} at [{:#06x}sub{sub}]",
                            object.index
                        ));
                    }
                }
            }
            // docs/DESIGN-DECISIONS.md EDS-13: python's REAL conversion is `float(value)`
            // (`eds.py:251-252`), so `abc` and a hex literal such as `0x10`
            // are `ValueError`s → "Invalid DefaultValue … ignoring", the
            // default stays unset. The two exits used to disagree with each
            // other *and* with python here (0.0 / 16.0, silently).
            if matches!(entry.data_type, DataType::Real32 | DataType::Real64) {
                if let Some(raw) = source_raw {
                    if parse_float_expr(raw).is_none() {
                        warnings.push(format!(
                            "invalid {key} {raw:?} at [{:#06x}sub{sub}] ignored",
                            object.index
                        ));
                    }
                }
            }
        }
    }
    eds.warnings.extend(warnings);
}

/// Parses a `LowLimit`/`HighLimit` the way python-canopen does
/// (`eds.py:339-362`): a base-0 integer, and for a **signed** data type an
/// unsigned hex literal in the upper half of the range read as two's
/// complement — `_signed_int_from_hex`, `eds.py:222-245`. A literal outside
/// the type's bit width is an error (the reference logs "Invalid LowLimit …"
/// and keeps no limit), which is what `None` means here.
fn parse_limit(raw: &str, dt: DataType) -> Option<i64> {
    let value = parse_int(raw)?;
    let Some(bits) = dt_bit_length(dt) else {
        // Non-integer types: python only reaches `_signed_int_from_hex` for
        // `SIGNED_TYPES`, everything else is a plain `int(value, 0)`.
        return Some(value);
    };
    if dt.is_signed() {
        let min_signed = -(1i128 << (bits - 1));
        let max_signed = (1i128 << (bits - 1)) - 1;
        let max_unsigned = (1i128 << bits) - 1;
        let v = i128::from(value);
        if v < min_signed || v > max_unsigned {
            return None;
        }
        let signed = if v > max_signed {
            v - (1i128 << bits)
        } else {
            v
        };
        i64::try_from(signed).ok()
    } else {
        // Unsigned types: python takes the plain integer (a negative literal
        // is simply kept as-is by `int(value, 0)`).
        Some(value)
    }
}

/// Bit width of an integer data type (`_calc_bit_length`, `eds.py:210-219`).
fn dt_bit_length(dt: DataType) -> Option<u32> {
    match dt {
        DataType::Integer8 => Some(8),
        DataType::Integer16 => Some(16),
        DataType::Integer24 => Some(24),
        DataType::Integer32 => Some(32),
        DataType::Integer40 => Some(40),
        DataType::Integer48 => Some(48),
        DataType::Integer56 => Some(56),
        DataType::Integer64 => Some(64),
        DataType::Unsigned8 => Some(8),
        DataType::Unsigned16 => Some(16),
        DataType::Unsigned24 => Some(24),
        DataType::Unsigned32 => Some(32),
        DataType::Unsigned40 => Some(40),
        DataType::Unsigned48 => Some(48),
        DataType::Unsigned56 => Some(56),
        DataType::Unsigned64 => Some(64),
        _ => None,
    }
}

/// Parses a REAL default with python's `float()` grammar (`eds.py:251-252`):
/// decimal/exponent forms, `inf`/`nan` — and nothing else. In particular a
/// hex literal is *not* a float in python (`float("0x10")` raises
/// `ValueError`), so an EDS author who wrote one lands on the same
/// "invalid default" path as any other malformed literal (`docs/DESIGN-DECISIONS.md` EDS-13).
/// Rust's `f64::from_str` accepts the same spellings (plus `infinity`), so no
/// grammar table of our own is needed.
///
/// Public so the code generator can take the same decision on the same
/// spelling — the two build exits must not disagree about a default
/// (`docs/DESIGN-DECISIONS.md` 十H1 family).
#[must_use]
pub fn parse_float_expr(raw: &str) -> Option<f64> {
    raw.trim().parse::<f64>().ok()
}

/// Resolves deferred composite `DataType` indexes (`> 0x1B`, e.g.
/// CANFestival's `0x20`) from the `[<index:X>sub1]` `DefaultValue` table.
/// An index without a matching section (or with an unresolvable value)
/// falls back to `Domain` with a warning, like an unknown type
/// (python-canopen `objectdictionary/eds.py:322-335`, `G5`).
fn resolve_composite_types(eds: &mut Eds) {
    // Temporarily move the table out so `objects` can be borrowed mutably;
    // it is restored afterwards (the table stays inspectable on the result).
    let type_defs = std::mem::take(&mut eds.type_defs);
    let mut warnings = Vec::new();
    for object in eds.objects.values_mut() {
        let index = object.index;
        for entry in object.subs.values_mut() {
            let Some(raw) = entry.deferred_data_type.take() else {
                continue;
            };
            let resolved = u16::try_from(raw)
                .ok()
                .and_then(|idx| type_defs.get(&idx).copied())
                .and_then(|v| u16::try_from(v).ok())
                .and_then(DataType::from_index);
            match resolved {
                Some(dt) => entry.data_type = dt,
                None => {
                    warnings.push(format!(
                        "unknown DataType={raw:#x} at [{index:#06x}] - using Domain"
                    ));
                    entry.data_type = DataType::Domain;
                }
            }
        }
    }
    eds.type_defs = type_defs;
    eds.warnings.extend(warnings);
}

/// Parsed section header kind. A sentinel pseudo-index would collide with a
/// real `[FFFE]` object and make it unparseable (`docs/DESIGN-DECISIONS.md` 十L9), so the
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
    /// `[DummyUsage]`: `Dummy000n=1` defines a dummy object at index `n`
    /// (python-canopen `eds.py:105-113`).
    DummyUsage,
    /// `[FileInfo]` metadata.
    FileInfo,
    /// `[DeviceInfo]` metadata.
    DeviceInfo,
    /// `[Comments]` free text.
    Comments,
    /// Any other section (`[MandatoryObjects]`, unknown names, …).
    Skipped,
}

/// Parses a CiA 306 boolean (`0`/`1`), ignoring malformed values.
fn parse_bool(value: &str) -> bool {
    parse_int(value).is_some_and(|v| v != 0)
}

/// Applies one `[FileInfo]` key.
fn apply_file_info(info: &mut EdsFileInfo, key: &str, value: &str) {
    match key.to_ascii_lowercase().as_str() {
        "filename" => info.file_name = value.to_string(),
        "fileversion" => info.file_version = parse_int(value).unwrap_or(0) as u32,
        "filerevision" => info.file_revision = parse_int(value).unwrap_or(0) as u32,
        "edsversion" => info.eds_version = value.to_string(),
        "description" => info.description = value.to_string(),
        "creationtime" => info.creation_time = value.to_string(),
        "creationdate" => info.creation_date = value.to_string(),
        "modificationtime" => info.modification_time = value.to_string(),
        "modificationdate" => info.modification_date = value.to_string(),
        "createdby" => info.created_by = value.to_string(),
        "modifiedby" => info.modified_by = value.to_string(),
        _ => {}
    }
}

/// Applies one `[DeviceInfo]` key.
fn apply_device_info(info: &mut EdsDeviceInfo, key: &str, value: &str) {
    match key.to_ascii_lowercase().as_str() {
        "vendorname" => info.vendor_name = value.to_string(),
        "vendornumber" => info.vendor_number = parse_int(value).and_then(|v| u32::try_from(v).ok()),
        "productname" => info.product_name = value.to_string(),
        "productnumber" => {
            info.product_number = parse_int(value).and_then(|v| u32::try_from(v).ok());
        }
        "revisionnumber" => {
            info.revision_number = parse_int(value).and_then(|v| u32::try_from(v).ok());
        }
        "ordercode" => info.order_code = value.to_string(),
        "baudrate_10" => info.baudrate_10 = parse_bool(value),
        "baudrate_20" => info.baudrate_20 = parse_bool(value),
        "baudrate_50" => info.baudrate_50 = parse_bool(value),
        "baudrate_125" => info.baudrate_125 = parse_bool(value),
        "baudrate_250" => info.baudrate_250 = parse_bool(value),
        "baudrate_500" => info.baudrate_500 = parse_bool(value),
        "baudrate_800" => info.baudrate_800 = parse_bool(value),
        "baudrate_1000" => info.baudrate_1000 = parse_bool(value),
        "simplebootupmaster" => info.simple_bootup_master = parse_bool(value),
        "simplebootupslave" => info.simple_bootup_slave = parse_bool(value),
        "granularity" => info.granularity = parse_int(value).unwrap_or(0) as u32,
        "dynamicchannelssupported" => {
            info.dynamic_channels_supported = parse_int(value).unwrap_or(0) as u32;
        }
        "groupmessaging" => info.group_messaging = parse_bool(value),
        "nrofrxpdo" => info.nr_of_rx_pdo = parse_int(value).unwrap_or(0) as u32,
        "nroftxpdo" => info.nr_of_tx_pdo = parse_int(value).unwrap_or(0) as u32,
        "lss_supported" => info.lss_supported = parse_bool(value),
        _ => {}
    }
}

/// Applies one `[Comments]` `LineN` value. `Lines` is the count and is
/// intentionally ignored; entries are placed by their numeric suffix (gaps
/// become empty strings), so out-of-order sections still read back sorted.
fn apply_comment(comments: &mut EdsComments, key: &str, value: &str) {
    let lower = key.to_ascii_lowercase();
    let Some(number) = lower.strip_prefix("line") else {
        return;
    };
    let Ok(number) = number.parse::<usize>() else {
        return;
    };
    if number == 0 {
        return;
    }
    while comments.lines.len() < number {
        comments.lines.push(String::new());
    }
    comments.lines[number - 1] = value.to_string();
}

fn parse_header(header: &str) -> Section {
    let lower = header.to_ascii_lowercase();
    if lower.starts_with("devicecomi") {
        // DeviceComissioning / DeviceCommissioning.
        return Section::Commissioning;
    }
    if lower == "dummyusage" {
        return Section::DummyUsage;
    }
    if lower == "fileinfo" {
        return Section::FileInfo;
    }
    if lower == "deviceinfo" {
        return Section::DeviceInfo;
    }
    if lower == "comments" {
        return Section::Comments;
    }
    // docs/DESIGN-DECISIONS.md L5: `[index]Name` / `[indexSubNName]` headers (python-canopen
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
    explicit_sub0: bool,
) {
    match key {
        "ParameterName" => {
            object.name = value.to_string();
            // docs/DESIGN-DECISIONS.md 十M16: `ParameterName` may appear *after*
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
                    "CompactSubObj={value} exceeds the 255-entry sub limit at [{index:#06x}] - expansion skipped (docs/DESIGN-DECISIONS.md P3-54)"
                ));
            }
        }
        "SubNumber" | "NrOfEntries" | "Denotation" | "Comment" => {}
        // `ObjectType` decides how the section is read: VAR/DOMAIN describe a
        // variable. ARRAY keys are applied optimistically so a later
        // `CompactSubObj` can still use the sub-0 template; finalization
        // removes the optimistic entry when no compact count follows.
        "ObjectType" => object.object_type = parse_int(value),
        "DataType" | "AccessType" | "DefaultValue" | "ParameterValue" | "PDOMapping"
        | "LowLimit" | "HighLimit" => {
            let optimistic_array = object.object_type == Some(0x0008);
            if (!optimistic_array
                && is_container_object_type(object.object_type, object.compact_sub_obj))
                || (explicit_sub0 && object.subs.contains_key(&0))
            {
                object
                    .custom_options
                    .insert(key.to_string(), value.to_string());
            } else {
                apply_key(
                    object.subs.entry(0).or_insert_with(|| EdsSubEntry {
                        name: object.name.clone(),
                        data_type: DataType::Unsigned32,
                        access: AccessType::Rw,
                        default: None,
                        default_raw: None,
                        parameter_value: None,
                        parameter_value_raw: None,
                        pdo_mappable: false,
                        custom_options: std::collections::BTreeMap::new(),
                        deferred_data_type: None,
                        low_limit: None,
                        high_limit: None,
                    }),
                    key,
                    value,
                    index,
                    0,
                    warnings,
                );
            }
        }
        _ => {
            // Unknown vendor key — or a CiA 306 key of a container section
            // (docs/DESIGN-DECISIONS.md EDS-02): preserved verbatim (python-canopen keeps
            // these in `custom_options`) instead of fabricating a bogus sub-0
            // entry.
            object
                .custom_options
                .insert(key.to_string(), value.to_string());
        }
    }
}

/// Whether an `ObjectType` makes a section a *container* (ARRAY `0x0008` /
/// RECORD `0x0009`, plus the type-definition codes `0x0005`/`0x0006` which
/// python never turns into an object) rather than a variable.
///
/// python-canopen builds a variable only for `VAR` (0x0007) and `DOMAIN`
/// (0x0002) and treats everything else as a container whose standard options
/// are dropped (`eds.py:132-153`). `None` (no `ObjectType` key) is VAR per
/// CiA 306 §4.6.3.2 — the same default python applies
/// (`eds.py:122-126`).
fn is_container_object_type(object_type: Option<i64>, compact_sub_obj: Option<u16>) -> bool {
    // docs/DESIGN-DECISIONS.md EDS-23: the container codes are **ARRAY 0x0008 / RECORD 0x0009**
    // (CiA 306 §4.6.3.2, `objectdictionary/objectcodes.py:4-9`), not
    // `DEFTYPE 0x0005` / `DEFSTRUCT 0x0006` — the previous pair gated the two
    // types python never builds an object from and let RECORD/ARRAY fabricate a
    // sub 0, which is the exact failure EDS-02 set out to remove. `DEFTYPE` /
    // `DEFSTRUCT` have no branch in python's dispatch either
    // (`eds.py:132-153`), so they keep producing no object.
    match object_type {
        // VAR / DOMAIN — the only types python builds a variable for
        // (`eds.py:132-133`); an absent `ObjectType` defaults to VAR
        // (CiA 306 §4.6.3.2, python `eds.py:122-126`).
        None | Some(0x0002 | 0x0007) => false,
        // An ARRAY **with** `CompactSubObj` is exempt: its inline definition is
        // this stack's expansion template and lives at sub 0 (CANopenNode EDS
        // convention, see `expand_compact_sub_obj`). python instead synthesises
        // sub 0 and builds the member at sub 1 (`eds.py:133-141`) — a
        // pre-existing divergence, recorded in `docs/EXTENSIONS.md` §2.6.
        Some(0x0008) => compact_sub_obj.is_none(),
        Some(_) => true,
    }
}

/// Re-applies one option to the position it was first written to, with a
/// possibly *updated* value.
///
/// Used by the continuation-line handling (docs/DESIGN-DECISIONS.md EDS-19): a configparser
/// continuation extends the previous value, so the same key has to be applied
/// again with the `\n`-joined text. `name_section` decides whether an
/// object-level key is a name/`NrOfEntries`/sub-name selector or an ordinary
/// inline key — the same classification the first pass performed, and the
/// current section cannot have changed in between (headers are never
/// indented).
fn reapply_option(
    eds: &mut Eds,
    index: u16,
    sub: Option<u8>,
    name_section: bool,
    key: &str,
    value: &str,
    explicit_sub0: bool,
) {
    let mut warnings = std::mem::take(&mut eds.warnings);
    if let Some(object) = eds.objects.get_mut(&index) {
        match sub {
            Some(sub) => {
                if let Some(entry) = object.subs.get_mut(&sub) {
                    apply_key(entry, key, value, index, sub, &mut warnings);
                }
            }
            None if name_section => {
                if key.eq_ignore_ascii_case("NrOfEntries") {
                    if let Some(n) = parse_int(value) {
                        let n = n.max(0);
                        object.compact_sub_obj = Some(u16::try_from(n).unwrap_or(u16::MAX));
                    }
                } else if let Ok(sub_idx) = key.parse::<u8>() {
                    object.sub_names.insert(sub_idx, value.to_string());
                } else {
                    apply_object_key(object, key, value, index, &mut warnings, explicit_sub0);
                }
            }
            None => apply_object_key(object, key, value, index, &mut warnings, explicit_sub0),
        }
    }
    eds.warnings.extend(warnings);
}

/// Re-applies the `ObjectType` gate once the **whole** section has been read
/// (`docs/DESIGN-DECISIONS.md` EDS-23). python-canopen decides how to read a section after
/// `eds.get(section, "ObjectType")` has been resolved (`eds.py:118-153`), so a
/// `DataType` line *before* `ObjectType` must not slip past the gate the way it
/// did while the gate was evaluated line by line. Doing it as a post-pass makes
/// the outcome independent of the key order.
///
/// * A container section's optimistically-created sub-0 entry is folded into
///   `custom_options` (python keeps those CiA 306 keys there, `eds.py:275-285`)
///   and removed — a python slave answers `0x0609_0011` for `idx:00`.
/// * A `CompactSubObj` ARRAY is exempt from the gate: its inline definition is
///   the expansion template and belongs at sub 0 in this stack (see
///   [`expand_compact_sub_obj`]).
/// * A sub 0 declared by its own `[<index>sub0]` section is a genuine member
///   and is never folded away (`inline_sub0`).
fn finalize_object_types(eds: &mut Eds, inline_sub0: &BTreeSet<(u16, u8)>) {
    for object in eds.objects.values_mut() {
        if !is_container_object_type(object.object_type, object.compact_sub_obj) {
            continue;
        }
        // A sub 0 that came from its own `[<index>sub0]` section is a real
        // member (python builds a variable for every sub-section regardless of
        // the parent's `ObjectType`, `eds.py:156-171`) and must survive.
        if !inline_sub0.contains(&(object.index, 0)) {
            continue;
        }
        if let Some(entry) = object.subs.remove(&0) {
            for key in EDS_VAR_KEYS {
                if let Some(raw) = eds_entry_key(&entry, key) {
                    object.custom_options.insert(key.to_string(), raw);
                }
            }
        }
    }
}

/// Exactly the key set [`apply_object_key`] gates behind
/// [`is_container_object_type`] — the CiA 306 keys a container section's
/// `custom_options` has to give back.
const EDS_VAR_KEYS: [&str; 7] = [
    "DataType",
    "AccessType",
    "DefaultValue",
    "ParameterValue",
    "PDOMapping",
    "LowLimit",
    "HighLimit",
];

/// Best-effort inverse of the gated arms of [`apply_object_key`], used only to
/// hand a rejected container section's CiA 306 keys to `custom_options` so
/// they survive for round-tripping.
///
/// `DefaultValue`/`ParameterValue` go back as the **raw EDS spelling** (the
/// struct keeps it); the rest are re-rendered from the parsed value, because
/// no reverse index/parser exists for those enums — so a section that only hits
/// this path *after* an optimistic sub 0 was built (i.e. `ObjectType` came
/// last) yields the resolved form (`Unsigned32`, `Const`) rather than the file's
/// literal. That is the lesser evil: these keys are inert either way, and
/// python drops them altogether (`_STANDARD_OPTIONS`, `eds.py:275-285`).
fn eds_entry_key(entry: &EdsSubEntry, key: &str) -> Option<String> {
    match key {
        "DataType" => Some(format!("{:?}", entry.data_type)),
        "AccessType" => Some(format!("{:?}", entry.access)),
        "DefaultValue" => entry.default_raw.clone(),
        "ParameterValue" => entry.parameter_value_raw.clone(),
        "PDOMapping" => Some(u8::from(entry.pdo_mappable).to_string()),
        "LowLimit" => entry.low_limit.map(|v| v.to_string()),
        "HighLimit" => entry.high_limit.map(|v| v.to_string()),
        _ => None,
    }
}

fn apply_key(
    entry: &mut EdsSubEntry,
    key: &str,
    value: &str,
    idx: u16,
    sub: u8,
    warnings: &mut Vec<String>,
) {
    match key {
        "ParameterName" => entry.name = value.to_string(),
        "DataType" => {
            if let Some(dt_idx) = parse_int(value) {
                // docs/DESIGN-DECISIONS.md L1: an out-of-u16 DataType index would silently
                // truncate through `as u16`; range-check and fall back with
                // a warning instead. python-canopen warns and falls back to
                // DOMAIN for an unknown index (`eds.py:322-335`) — silently
                // treating it as Unsigned32 changed the SDO width and PDO
                // semantics (docs/DESIGN-DECISIONS.md 十M12).
                match u16::try_from(dt_idx).ok().and_then(DataType::from_index) {
                    // docs/DESIGN-DECISIONS.md EDS-25: a real type cancels any pending
                    // composite index (last-wins, EDS-20).
                    Some(dt) => {
                        entry.data_type = dt;
                        entry.deferred_data_type = None;
                    }
                    // G5: a CANFestival composite DataType (`> 0x1B`, e.g.
                    // `0x20`) names its real type in the `[<idx:X>sub1]`
                    // `DefaultValue`; defer until the whole file is parsed
                    // (`resolve_composite_types`).
                    // docs/DESIGN-DECISIONS.md EDS-25: a *later* explicit `DataType` in the
                    // same section replaces the deferred one — duplicate keys
                    // are last-wins (EDS-20, python's dict assignment
                    // `eds.py:325-335`), so the composite resolution must not
                    // resurrect the stale index.
                    None if dt_idx > 0x1B => {
                        entry.data_type = DataType::Unsigned32;
                        entry.deferred_data_type = Some(dt_idx);
                    }
                    None => {
                        warnings.push(format!(
                            "unknown DataType={value} at [{idx:#06x}sub{sub}] - using Domain"
                        ));
                        entry.data_type = DataType::Domain;
                        entry.deferred_data_type = None;
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
                // reported instead of silently swallowed (docs/DESIGN-DECISIONS.md P3-52).
                other => warnings.push(format!(
                    "unknown AccessType {other:?} at [{idx:#06x}sub{sub}] - using Rw"
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
            // 0x1003 subentries of DS301_profile.eds, docs/DESIGN-DECISIONS.md 十H4).
            if value.is_empty() {
                warnings.push(format!(
                    "empty DefaultValue at [{idx:#06x}sub{sub}] ignored (no default)"
                ));
            } else {
                entry.default =
                    Some(OdExpr::parse(value).unwrap_or_else(|| OdExpr::Str(value.to_string())));
                // Keep the original spelling for textual consumers
                // (VisibleString) — python returns the raw string
                // (`eds.py:249-250`), so `0x41` must not become `"65"`.
                entry.default_raw = Some(value.to_string());
            }
        }
        // docs/DESIGN-DECISIONS.md 八L9: any non-zero value marks the object mappable, like
        // python-canopen's `bool(int(...))` (`eds.py:337`). CANFestival's
        // `PDOMapping=2` ("RPDO only") is therefore accepted as mappable
        // for both directions rather than silently rejected.
        "PDOMapping" => entry.pdo_mappable = parse_int(value).unwrap_or(0) != 0,
        // docs/DESIGN-DECISIONS.md EDS-12: `LowLimit`/`HighLimit` used to land in
        // `custom_options` (round-trip only), so the min/max semantics were
        // lost. python-canopen stores them on the variable (`var.min`/`max`)
        // with `_signed_int_from_hex` for signed types
        // (`eds.py:339-362`, `:222-245`) and merely *warns* on a malformed
        // literal — the value is informational here too, since neither stack
        // rejects a protocol-level write for it.
        "LowLimit" | "HighLimit" => {
            let parsed = parse_limit(value, entry.data_type);
            if parsed.is_none() {
                warnings.push(format!(
                    "invalid {key} {value:?} at [{idx:#06x}sub{sub}] ignored (python-canopen warns and keeps no limit, docs/DESIGN-DECISIONS.md EDS-12)"
                ));
            }
            if key == "LowLimit" {
                entry.low_limit = parsed;
            } else {
                entry.high_limit = parsed;
            }
        }
        // `ParameterValue` is a *standard* CiA 306 option (python excludes it
        // from `custom_options`, `_STANDARD_OPTIONS`, `eds.py:275-285`) and
        // overrides the default for the local node's initial value
        // (`docs/DESIGN-DECISIONS.md` 七E2). Parsed exactly like `DefaultValue`.
        "ParameterValue" => {
            if value.is_empty() {
                warnings.push(format!(
                    "empty ParameterValue at [{idx:#06x}sub{sub}] ignored (no value)"
                ));
            } else {
                entry.parameter_value =
                    Some(OdExpr::parse(value).unwrap_or_else(|| OdExpr::Str(value.to_string())));
                entry.parameter_value_raw = Some(value.to_string());
            }
        }
        // Unknown subentry keys are preserved verbatim for round-tripping
        // (docs/DESIGN-DECISIONS.md #19), mirroring python-canopen's per-variable
        // `custom_options` instead of silently dropping vendor data.
        _ => {
            entry
                .custom_options
                .insert(key.to_string(), value.to_string());
        }
    }
}

impl EdsSubEntry {
    /// Materialises the default value for the dictionary. Missing defaults
    /// become zero values of the declared type.
    ///
    /// Whether the stored `ParameterValue` can actually be materialised for
    /// this entry's data type (docs/DESIGN-DECISIONS.md EDS-26/EDS-27).
    ///
    /// python-canopen decodes the value inside a `try` and simply leaves
    /// `var.value` unset on failure (`eds.py:375-382`); the integer path
    /// re-uses `parse_integer_default`, which is what turns a non-numeric
    /// `ParameterValue` into a hard build failure in this stack.
    pub(crate) fn parameter_value_materialises(&self) -> bool {
        let Some(raw) = self.parameter_value_raw.as_deref() else {
            return false;
        };
        if matches!(self.parameter_value, Some(OdExpr::NodeRelative { .. })) {
            return true;
        }
        match self.data_type {
            dt if is_integer_type(dt) => parse_integer_default(raw, self.data_type).is_ok(),
            // Blob types: python runs `bytes.fromhex` on the raw text
            // (`eds.py:246-250`), so a non-hex `ParameterValue` is the same
            // kind of `ValueError`.
            DataType::OctetString | DataType::Domain => {
                let candidate = match &self.parameter_value {
                    Some(OdExpr::Str(s)) => Some(s.as_str()),
                    Some(_) => Some(raw),
                    None => None,
                };
                candidate.and_then(hex_bytes_checked).is_some()
            }
            _ => true,
        }
    }

    /// # Errors
    /// Propagates encoding errors from type/range validation.
    pub fn to_value(&self, node_id: u8) -> Result<Value> {
        use DataType as Dt;
        // ParameterValue overrides DefaultValue for the initial dictionary
        // value, mirroring python-canopen's local node, which serves
        // `obj.value` (ParameterValue) in preference to `obj.default`
        // (`node/local.py:99-107`). `None` falls back to the default
        // (docs/DESIGN-DECISIONS.md 七E2).
        // docs/DESIGN-DECISIONS.md EDS-26: python only assigns `var.value` when
        // `_decode_from_eds` succeeds (`eds.py:375-382`), so an **invalid**
        // `ParameterValue` leaves `var.value = None` and the local node falls
        // back to `obj.default` (`node/local.py:104-107`). Preferring it
        // unconditionally threw away a perfectly good `DefaultValue`.
        // `prefer_parameter_value` is therefore only set when the
        // `ParameterValue` is known to be materialisable (see
        // `warn_invalid_defaults`).
        let prefer_parameter_value =
            self.parameter_value.is_some() && self.parameter_value_materialises();
        let default = if prefer_parameter_value {
            self.parameter_value.as_ref()
        } else {
            self.default.as_ref()
        };
        let default_raw = if prefer_parameter_value {
            self.parameter_value_raw.as_deref()
        } else {
            self.default_raw.as_deref()
        };
        // Non-numeric types read their defaults from dedicated expression
        // variants (`docs/DESIGN-DECISIONS.md` P1-6).
        if self.data_type == Dt::VisibleString {
            return Ok(match default {
                Some(OdExpr::Str(s)) => Value::Str(s.clone()),
                // Bare numeric / node-relative defaults keep their original
                // textual form, like python-canopen's `_decode_from_eds`
                // (`eds.py:249-250` returns the raw string): `0x41` stays
                // `"0x41"` and `1e3` stays `"1e3"` (十M25). Quoted strings
                // take the normal `Str` path above.
                Some(_) => Value::Str(default_raw.unwrap_or_default().to_string()),
                None => Value::Str(String::new()),
            });
        }
        if self.data_type == Dt::UnicodeString {
            // The EDS spelling is text (python-canopen `_decode_from_eds`
            // returns the raw string, `eds.py:246-250`), the dictionary holds
            // the **wire** form: UTF-16LE bytes, exactly like every SDO read
            // of such an entry (八M5) and like the code generator's exit
            // (docs/DESIGN-DECISIONS.md EDS-14).
            //
            // docs/DESIGN-DECISIONS.md EDS-05: this used to materialise `Value::Str` while the
            // wire decode produced `Value::Bytes`, so the *same* address
            // changed `Value` kind after one SDO write — `Display`,
            // `PartialEq` and the persisted payload drifted with it. Both
            // exits now emit the UTF-16LE bytes.
            let text = match default {
                Some(OdExpr::Str(s)) => s.clone(),
                // Bare numeric / node-relative defaults keep their textual
                // form, like the VisibleString branch above and python's
                // raw-string return.
                Some(_) => default_raw.unwrap_or_default().to_string(),
                None => String::new(),
            };
            let units: Vec<u16> = text.encode_utf16().collect();
            let mut buf = vec![0u8; units.len() * 2];
            for (chunk, unit) in buf.chunks_exact_mut(2).zip(units) {
                chunk.copy_from_slice(&unit.to_le_bytes());
            }
            return Ok(Value::Bytes(buf));
        }
        if matches!(self.data_type, Dt::OctetString | Dt::Domain) {
            // Invalid hex leaves the default unset — the parse pass already
            // recorded a warning (`warn_invalid_defaults`), mirroring
            // python's `bytes.fromhex` + ValueError handling (docs/DESIGN-DECISIONS.md N10).
            let bytes = match default {
                Some(OdExpr::Str(s)) => crate::eds::hex_bytes(s),
                _ => Vec::new(),
            };
            return Ok(Value::Bytes(bytes));
        }
        if matches!(self.data_type, Dt::Real32 | Dt::Real64) {
            // docs/DESIGN-DECISIONS.md EDS-13: python-canopen parses REAL defaults with plain
            // `float(value)` (`eds.py:251-252`) and a `ValueError` — a hex
            // literal such as `0x10`, or plain garbage — only logs
            // "Invalid DefaultValue …" and leaves the default unset
            // (`:369-373`). Routing the *raw* text through
            // [`parse_float_expr`] reproduces both halves: `0x10` is no
            // longer read as the integer 16, and a garbage literal no longer
            // becomes a silent `0.0` (the parse pass recorded the warning).
            let f = match default {
                Some(OdExpr::Float(f)) => *f,
                _ => default_raw.and_then(parse_float_expr).unwrap_or(0.0),
            };
            return Ok(Value::Real(f));
        }
        // TIME types are fixed 6-byte little-endian values: `encode_as`/
        // `decode_le` treat them as `Value::Bytes`, so the default must be
        // materialised as bytes or every SDO access to the entry fails with
        // `TypeMismatch` (`docs/DESIGN-DECISIONS.md` H1). Hex-string defaults are decoded
        // verbatim; numeric defaults encode little-endian (ms since
        // midnight in the low 32 bits, days in the high 16).
        if matches!(self.data_type, Dt::TimeOfDay | Dt::TimeDifference) {
            let bytes = match default {
                Some(OdExpr::Str(s)) => crate::eds::hex_bytes(s),
                _ => (default
                    .map_or(0i128, |e| e.resolve_wide(node_id))
                    .clamp(0, i128::from(u64::MAX)) as u64)
                    .to_le_bytes()[..6]
                    .to_vec(),
            };
            // docs/DESIGN-DECISIONS.md L4: TIME types are fixed 6-byte values. A hex-string
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
        // defaults (`docs/DESIGN-DECISIONS.md` H2). Convert with a range check instead.
        let raw = match (default, self.data_type) {
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
            // docs/DESIGN-DECISIONS.md B3-3: 整数类型条目的字符串默认（如超出 i64 的数值字面量
            // `0xFFFFFFFFFFFFFFFF`）此前退化为 Str 并在下方 `resolve` 静默得 0。
            // 改为按完整宽度解析并做范围校验：越界或不可解析则显式报错，而非
            // 静默零值。字符串/可见字符串类型的默认在此分支之外由各自路径处理。
            (Some(OdExpr::Str(s)), dt) if is_integer_type(dt) => parse_integer_default(s, dt)?,
            _ => default.map_or(0i128, |e| e.resolve_wide(node_id)),
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
            Dt::Integer64 => Value::Int(signed_default(raw, Dt::Integer64)?),
            dt if dt.fixed_size().is_some() => {
                // docs/DESIGN-DECISIONS.md M2: 无符号类型收到负默认（如 `DefaultValue=-1`）
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
        // docs/DESIGN-DECISIONS.md L1: 构建时校验默认值范围——此前 Literal 默认值不校验，
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

/// How [`Eds::build_with`] treats content-level default errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdsBuildPolicy {
    /// A malformed or out-of-range default aborts the whole dictionary build
    /// (the historical `Eds::build` behaviour).
    Strict,
    /// A malformed/out-of-range default is dropped with a warning; the entry
    /// falls back to a typed zero value (python-canopen's lenient runtime
    /// behaviour).
    Lenient,
}

/// A dictionary-build content error carrying the EDS-specific type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EdsBuildError(pub canopen_core::types::Error);

impl std::fmt::Display for EdsBuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "EDS content error: {}", self.0)
    }
}

impl std::error::Error for EdsBuildError {}

/// Typed zero value used by the lenient build when one entry cannot be
/// materialised.
fn zero_value(dt: DataType) -> Value {
    match dt {
        DataType::Boolean => Value::Bool(false),
        DataType::Integer8
        | DataType::Integer16
        | DataType::Integer24
        | DataType::Integer32
        | DataType::Integer40
        | DataType::Integer48
        | DataType::Integer56
        | DataType::Integer64 => Value::Int(0),
        DataType::Real32 | DataType::Real64 => Value::Real(0.0),
        DataType::VisibleString => Value::Str(String::new()),
        DataType::UnicodeString | DataType::OctetString | DataType::Domain => {
            Value::Bytes(Vec::new())
        }
        DataType::TimeOfDay | DataType::TimeDifference => Value::Bytes(vec![0; 6]),
        _ => Value::Uint(0),
    }
}

impl Eds {
    /// Materialises the `[DummyUsage]` entries as `(index, subentry)`
    /// pairs: `Dummy000n=1` → a `const` VAR at index `n` whose data type
    /// index is also `n` (`1..=7`), with a typed zero value. Mirrors
    /// python-canopen `eds.py:105-113` (`ODVariable(key, i, 0)` with
    /// `data_type = i` and `access_type = "const"`). Unknown data type
    /// indices are dropped.
    #[must_use]
    pub fn dummy_usage_entries(&self) -> Vec<(u16, EdsSubEntry)> {
        self.dummy_usage
            .iter()
            .filter_map(|(&index, &dt_index)| {
                let data_type = DataType::from_index(u16::from(dt_index))?;
                Some((
                    u16::from(index),
                    EdsSubEntry {
                        name: format!("Dummy{index:04}"),
                        data_type,
                        access: AccessType::Const,
                        default: None,
                        default_raw: None,
                        parameter_value: None,
                        parameter_value_raw: None,
                        pdo_mappable: false,
                        custom_options: std::collections::BTreeMap::new(),
                        deferred_data_type: None,
                        low_limit: None,
                        high_limit: None,
                    },
                ))
            })
            .collect()
    }

    /// Builds a runtime [`ObjectDictionary`] bound to `node_id`.
    ///
    /// Objects without any parsable subentries are skipped. String, blob
    /// and float defaults are materialised from their quoted/hex/decimal
    /// expressions (`docs/DESIGN-DECISIONS.md` P1-6/P1-7).
    ///
    /// # Errors
    /// Propagates per-entry validation errors.
    pub fn to_object_dictionary(&self, node_id: u8) -> Result<ObjectDictionary> {
        let mut od = ObjectDictionary::new();
        // Dummy entries first: a real object at the same index wins.
        for (index, se) in self.dummy_usage_entries() {
            od.insert(
                Address::new(index, 0),
                Entry::new(
                    se.name.clone(),
                    se.data_type,
                    se.access,
                    Storage::Value(se.to_value(node_id)?),
                ),
            );
        }
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

    /// Builds a runtime [`ObjectDictionary`] with EDS **content** errors
    /// surfaced as [`crate::error::HostError::Eds`].
    ///
    /// This is the host-layer counterpart of [`Self::to_object_dictionary`]:
    /// out-of-range/invalid defaults and other content-level problems come
    /// back as `HostError::Eds`, so callers can tell a device-side EDS
    /// problem apart from a network/transport failure — the promise
    /// `HostError::Eds` makes (docs/DESIGN-DECISIONS.md API-08). The raw `types::Error` stays
    /// available through [`Self::to_object_dictionary`].
    ///
    /// # Errors
    /// [`crate::error::HostError::Eds`] wrapping per-entry validation errors
    /// (e.g. a default that overflows its declared type).
    pub fn build(
        &self,
        node_id: u8,
    ) -> std::result::Result<ObjectDictionary, crate::error::HostError> {
        self.to_object_dictionary(node_id)
            .map_err(crate::error::HostError::Eds)
    }

    /// Builds with an explicit policy and returns the content warnings.
    ///
    /// `Strict` is equivalent to [`Self::build`] but also returns the
    /// parse-time warnings; `Lenient` replaces a failing entry with a typed
    /// zero and records why. The dictionary shape (objects/subentries) is
    /// otherwise identical, so both modes can be compared entry by entry.
    ///
    /// # Errors
    /// [`crate::error::HostError::Eds`] only in [`EdsBuildPolicy::Strict`].
    pub fn build_with(
        &self,
        node_id: u8,
        policy: EdsBuildPolicy,
    ) -> std::result::Result<(ObjectDictionary, Vec<String>), crate::error::HostError> {
        match policy {
            EdsBuildPolicy::Strict => self
                .to_object_dictionary(node_id)
                .map(|od| (od, self.warnings.clone()))
                .map_err(crate::error::HostError::Eds),
            EdsBuildPolicy::Lenient => {
                let mut od = ObjectDictionary::new();
                let mut warnings = self.warnings.clone();
                for (index, se) in self.dummy_usage_entries() {
                    let value = match se.to_value(node_id) {
                        Ok(v) => v,
                        Err(e) => {
                            warnings.push(format!(
                                "EDS entry [{index:#06x}:0] dropped default ({e}); using zero value"
                            ));
                            zero_value(se.data_type)
                        }
                    };
                    od.insert(
                        Address::new(index, 0),
                        Entry::new(
                            se.name.clone(),
                            se.data_type,
                            se.access,
                            Storage::Value(value),
                        ),
                    );
                }
                for object in self.objects.values() {
                    let expanded = expand_compact_sub_obj(object);
                    let subs_iter: Box<dyn std::iter::Iterator<Item = (&u8, &EdsSubEntry)>> =
                        match &expanded {
                            Some(subs) => Box::new(subs.iter()),
                            None => Box::new(object.subs.iter()),
                        };
                    for (sub, se) in subs_iter {
                        let value = match se.to_value(node_id) {
                            Ok(v) => v,
                            Err(e) => {
                                warnings.push(format!(
                                    "EDS entry [{:#06x}:{sub}] dropped default ({e}); using zero value",
                                    object.index
                                ));
                                zero_value(se.data_type)
                            }
                        };
                        od.insert(
                            Address::new(object.index, *sub),
                            Entry::new(
                                se.name.clone(),
                                se.data_type,
                                se.access,
                                Storage::Value(value),
                            )
                            .pdo_mappable_if(se.pdo_mappable),
                        );
                    }
                }
                Ok((od, warnings))
            }
        }
    }

    /// Lenient convenience wrapper around [`Self::build_with`].
    ///
    /// # Errors
    /// Currently infallible, kept symmetric with [`Self::build`] for future
    /// dictionary-shape errors.
    pub fn build_lenient(
        &self,
        node_id: u8,
    ) -> std::result::Result<(ObjectDictionary, Vec<String>), crate::error::HostError> {
        self.build_with(node_id, EdsBuildPolicy::Lenient)
    }

    /// Typed variant for callers that need an EDS-specific error without
    /// touching [`crate::error::HostError`] (docs/DESIGN-DECISIONS.md API-03).
    ///
    /// # Errors
    /// [`EdsBuildError`] wrapping a per-entry dictionary-build error.
    pub fn build_typed(&self, node_id: u8) -> std::result::Result<ObjectDictionary, EdsBuildError> {
        self.to_object_dictionary(node_id).map_err(EdsBuildError)
    }
}

/// Expands a `CompactSubObj=N` object: the parser lands the `idx`
/// section's inline definition in the sub-0 slot — that IS the template.
/// Expansion replaces it: sub0 becomes the element count and subs 1..=N
/// are deep copies of the template. Shared by the runtime dictionary
/// builder and the code generator (`docs/DESIGN-DECISIONS.md` P1-8).
///
/// Returns `Some(empty)` — not `None` — for a `CompactSubObj` beyond the
/// u8 sub-index limit (n > 255): the parse already warned ("expansion
/// skipped", `docs/DESIGN-DECISIONS.md` P3-54), and an empty expansion skips the object
/// cleanly instead of degrading it to its lone sub-0 template (`docs/DESIGN-DECISIONS.md`
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
            parameter_value: None,
            parameter_value_raw: None,
            pdo_mappable: false,
            custom_options: std::collections::BTreeMap::new(),
            deferred_data_type: None,
            low_limit: None,
            high_limit: None,
        },
    );
    for i in 1..=n {
        let sub = u8::try_from(i).ok()?;
        // An explicit `[indexsubN]` definition wins over the template. This
        // is an **intentional difference**: python copies the sub-1 template
        // over every existing subentry (`copy_variable`, `eds.py:172-182,402-408`
        // + `objectdictionary/__init__.py:248-252`), so its result depends on
        // the order of the sections, while this stack is order-independent and
        // never discards an explicit definition (`docs/DESIGN-DECISIONS.md` EDS-06); the
        // `[indexName]` name, when given, labels the slot either way.
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

    /// docs/DESIGN-DECISIONS.md EDS-23: the ObjectType gate must use the **real** container
    /// codes (`ARRAY 0x0008` / `RECORD 0x0009`, `objectcodes.py:4-9`). The
    /// previous `0x0005`/`0x0006` pair gated the two type-definition codes
    /// (which python never turns into an object) and let a RECORD fabricate a
    /// readable sub 0 — the very failure EDS-02 set out to remove.
    #[test]
    fn record_and_array_object_types_do_not_fabricate_sub_zero() {
        for (label, object_type) in [("RECORD", "0x9"), ("ARRAY", "0x8")] {
            let text = format!(
                "[1018]\nParameterName=Identity\nObjectType={object_type}\n\
                 DataType=0x0007\nAccessType=ro\nDefaultValue=3\n"
            );
            let eds = parse(&text).unwrap();
            let od = eds.to_object_dictionary(0x05).unwrap();
            assert!(
                od.read(Address::new(0x1018, 0)).is_err(),
                "{label} {object_type} must not create a readable sub 0: {:?}",
                od.read(Address::new(0x1018, 0))
            );
            // The CiA 306 keys survive for round-tripping, like python's
            // `_STANDARD_OPTIONS` (`eds.py:275-285`).
            assert!(
                eds.objects[&0x1018].custom_options.contains_key("DataType"),
                "{label}: keys must be preserved in custom_options"
            );
        }
    }

    /// docs/DESIGN-DECISIONS.md EDS-23: the gate is applied **after** the whole section is
    /// read, so `ObjectType` may appear after the variable keys (python resolves
    /// it up front, `eds.py:118-126`). A VAR section keeps its sub 0; a RECORD
    /// does not gain one.
    #[test]
    fn object_type_gate_is_independent_of_key_order() {
        let var = parse("[2000]\nDataType=0x0007\nAccessType=rw\nDefaultValue=7\nObjectType=0x7\n")
            .unwrap();
        assert_eq!(
            var.to_object_dictionary(0x05)
                .unwrap()
                .read(Address::new(0x2000, 0))
                .unwrap(),
            Value::Uint(7)
        );
        let record =
            parse("[2001]\nDataType=0x0007\nAccessType=rw\nDefaultValue=7\nObjectType=0x9\n")
                .unwrap();
        assert!(record
            .to_object_dictionary(0x05)
            .unwrap()
            .read(Address::new(0x2001, 0))
            .is_err());
    }

    /// docs/DESIGN-DECISIONS.md EDS-23: the post-pass folds away only the sub 0 it created
    /// *optimistically* from the object section's keys. A sub 0 declared by its
    /// own `[<index>sub0]` section is a real member and survives the container
    /// gate — python builds a variable for every sub-section regardless of the
    /// parent's `ObjectType` (`eds.py:156-171`).
    #[test]
    fn explicit_sub_zero_section_survives_the_container_gate() {
        let eds = parse(
            "[1018]\nParameterName=Identity\nObjectType=0x9\nSubNumber=0x2\n\
             [1018sub0]\nParameterName=MaxSub\nDataType=0x0005\n\
             AccessType=ro\nDefaultValue=0x01\n\
             [1018sub1]\nParameterName=VendorID\nDataType=0x0007\n\
             AccessType=ro\nDefaultValue=0x1234\n",
        )
        .unwrap();
        let od = eds.to_object_dictionary(0x05).unwrap();
        assert_eq!(
            od.read(Address::new(0x1018, 0)).unwrap(),
            Value::Uint(1),
            "a declared sub 0 is a member, not a fabricated header"
        );
        assert_eq!(
            od.read(Address::new(0x1018, 1)).unwrap(),
            Value::Uint(0x1234)
        );
    }

    /// docs/DESIGN-DECISIONS.md EDS-23: an ARRAY **with** `CompactSubObj` keeps the CANopenNode
    /// convention (inline definition = expansion template at sub 0, sub 0
    /// becomes the element count) — it must not be swept up by the container
    /// gate.
    #[test]
    fn compact_sub_obj_array_keeps_its_template() {
        let eds = parse(
            "[2100]\nParameterName=AI\nObjectType=0x8\nCompactSubObj=3\n\
             DataType=0x0003\nAccessType=rw\nDefaultValue=9\n",
        )
        .unwrap();
        let od = eds.to_object_dictionary(0x10).unwrap();
        assert_eq!(od.read(Address::new(0x2100, 0)).unwrap(), Value::Uint(3));
        for sub in 1..=3u8 {
            assert_eq!(
                od.read(Address::new(0x2100, sub)).unwrap(),
                Value::Int(9),
                "sub {sub} must be a template copy"
            );
        }
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

    /// docs/DESIGN-DECISIONS.md 十M24: `[indexName]` supplies `NrOfEntries` and per-subentry
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

    /// docs/DESIGN-DECISIONS.md 十M25: VisibleString defaults keep the raw EDS spelling
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

    /// docs/DESIGN-DECISIONS.md 十M25: `NaN`/`inf` are valid FLOAT defaults (python uses
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

    /// docs/DESIGN-DECISIONS.md 十M15: a signed hex default beyond the type's unsigned range
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

    /// docs/DESIGN-DECISIONS.md EDS-11: an extreme `$NODEID±offset` must neither panic in
    /// debug builds nor wrap silently — the wide resolution feeds the
    /// per-type range check, so the build fails cleanly.
    #[test]
    fn extreme_node_relative_offset_errors_without_panicking() {
        // Resolution is exact in i128 and saturates (with the sign kept).
        let huge = OdExpr::parse("$NODEID+0x7FFFFFFFFFFFFFFF").unwrap();
        assert_eq!(huge.resolve_wide(1), i128::from(i64::MAX) + 1);
        assert_eq!(huge.resolve(1), i64::MAX);
        let tiny = OdExpr::parse("$NODEID-0x7FFFFFFFFFFFFFFF").unwrap();
        assert_eq!(tiny.resolve_wide(1), 1 - i128::from(i64::MAX));
        assert_eq!(tiny.resolve(1), 1 - i64::MAX);

        // The runtime build exit reports ValueOutOfRange instead of
        // overflowing (this used to panic with "attempt to add with
        // overflow" inside `resolve`).
        let text = "[2000]\nParameterName=P\nDataType=0x0007\nAccessType=rw\n\
                    DefaultValue=$NODEID+0x7FFFFFFFFFFFFFFF\n";
        let eds = parse(text).unwrap();
        let err = eds.to_object_dictionary(1).unwrap_err();
        assert!(
            matches!(err, Error::ValueOutOfRange { .. }),
            "expected ValueOutOfRange, got {err:?}"
        );
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
        // docs/DESIGN-DECISIONS.md H2: the old `split(';')` truncated a quoted
        // `DefaultValue="a;b"` to `"a` — a `;` inside a `"…"` string is
        // data, not a comment. A `;` outside quotes preceded by whitespace is
        // still a comment — that half *is* configparser parity (python's
        // `inline_comment_prefixes=(';',)`), the quote awareness is this
        // stack's own leniency (docs/DESIGN-DECISIONS.md EDS-18, EXTENSIONS §2.6).
        // docs/DESIGN-DECISIONS.md N13: `unquote` no longer over-strips single-sided or
        // repeated quotes; names/options keep their quotes. Paired quotes in
        // a *typed* default are still consumed by `OdExpr::parse`, so
        // `"a;b"` materialises as `a;b` — an intentional, now-documented
        // divergence from python (docs/DESIGN-DECISIONS.md EDS-16, EXTENSIONS §2.6).
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
    fn n13_quotes_are_kept_in_names_and_custom_options() {
        // docs/DESIGN-DECISIONS.md N13: python-canopen reads through `RawConfigParser` and
        // never unquotes (`objectdictionary/eds.py:313,365,376`; empirically
        // `od[0x1000].name == '"Quoted name"'`). The old
        // `trim_matches('"')` additionally over-stripped single-sided and
        // repeated quotes; only the surrounding whitespace is trimmed now.
        let eds = parse(
            "[1000]\nParameterName=\"Quoted name\"\nDataType=0x0007\nAccessType=ro\n\
             VendorKey=\"v\"\n\
             [1000sub1]\nParameterName=\"Sub name\"\nDataType=0x0007\nAccessType=ro\n",
        )
        .unwrap();
        let object = &eds.objects[&0x1000];
        assert_eq!(object.name, "\"Quoted name\"");
        assert_eq!(
            object.custom_options.get("VendorKey").map(String::as_str),
            Some("\"v\"")
        );
        assert_eq!(
            object.subs[&1].name.as_str(),
            "\"Sub name\"",
            "sub-entry names keep their quotes too"
        );
    }

    #[test]
    fn h3_node_relative_suffix_form_is_supported() {
        // docs/DESIGN-DECISIONS.md 十H3: `1280+$NODEID` (the official sample.eds spelling)
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
        // docs/DESIGN-DECISIONS.md EDS-15: offsets on **both** sides are not malformed —
        // python-canopen deletes `\+?\$NODEID\+?` outright, so the literals are
        // concatenated before the node id is added (`eds.py:254-259`;
        // measured: `1+$NODEID+2` reads back 17 with node 5). The old code
        // rejected the whole expression, which for an integer entry aborted
        // the entire dictionary build.
        assert_eq!(
            OdExpr::parse("1+$NODEID+2"),
            Some(OdExpr::NodeRelative { offset: 12 })
        );
        assert_eq!(OdExpr::parse("1+$NODEID+2").map(|e| e.resolve(5)), Some(17));
        // …while a concatenation that is not a valid integer stays invalid
        // (python: `int("0x100x20", 0)` raises `ValueError`).
        assert_eq!(OdExpr::parse("0x10+$NODEID+0x20"), None);
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
    fn g5_composite_datatype_dereferences_sub1_default() {
        // CANFestival marks PDO communication/mapping parameters with a
        // composite DataType (e.g. 0x20) whose real type is the
        // `DefaultValue` of the `[<idx:X>sub1]` section (python-canopen
        // `objectdictionary/eds.py:322-335`).
        let eds = parse(
            "[20sub1]\nParameterName=TYPE\nDataType=0x0007\nAccessType=ro\nDefaultValue=7\n\
             [1400]\nParameterName=COB-ID\nDataType=0x20\nAccessType=rw\n\
             [1400sub1]\nParameterName=COB-ID\nDataType=0x20\nAccessType=rw\n",
        )
        .unwrap();
        assert_eq!(
            eds.objects[&0x1400].subs[&0].data_type,
            DataType::Unsigned32
        );
        assert_eq!(
            eds.objects[&0x1400].subs[&1].data_type,
            DataType::Unsigned32
        );
        assert!(!eds.warnings.iter().any(|w| w.contains("unknown DataType")));
        // The dereferenced type drives default materialisation: `DataType`
        // 0x20 with no default becomes a U32 zero, not a Domain blob.
        let od = eds.to_object_dictionary(1).unwrap();
        assert_eq!(od.read(Address::new(0x1400, 0)).unwrap(), Value::Uint(0));
        // Without the `[<idx:X>sub1]` section the fallback stays Domain +
        // warning, matching python-canopen's `NoSectionError` branch.
        let eds = parse("[2000]\nParameterName=X\nDataType=0x21\nAccessType=ro\n").unwrap();
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

    /// docs/DESIGN-DECISIONS.md EDS-02: an inline `[index]` section only describes a variable
    /// for `ObjectType ∈ {VAR, DOMAIN}`; a RECORD/ARRAY header's CiA 306 keys
    /// are standard options that python-canopen drops instead of turning into
    /// a readable sub 0 (`eds.py:132-153,275-285`).
    #[test]
    fn e3_record_and_array_headers_do_not_fabricate_sub_zero() {
        // docs/DESIGN-DECISIONS.md EDS-23: the codes here were `0x0006`/`0x0005` — the
        // *type-definition* codes, not containers; `RECORD` is `0x0009` and
        // `ARRAY` is `0x0008` (`objectcodes.py:4-9`). They happened to be gated
        // too, so the test passed while the real containers were not gated.
        let eds = parse(
            "[1000]\nObjectType=0x0009\nParameterName=Rec\nDataType=0x0007\n\
             AccessType=rwr\nDefaultValue=3\n\
             [1001]\nObjectType=0x0008\nParameterName=Arr\nDataType=0x0007\n\
             [1002]\nParameterName=Var\nDataType=0x0007\nDefaultValue=7\n",
        )
        .unwrap();
        assert_eq!(eds.objects[&0x1000].object_type, Some(0x0009));
        assert!(
            eds.objects[&0x1000].subs.is_empty(),
            "a RECORD header must not create sub 0: {:?}",
            eds.objects[&0x1000].subs
        );
        // The keys are preserved for round-trips, like python's dropped
        // standard options end up nowhere but ours stay inspectable.
        assert_eq!(
            eds.objects[&0x1000].custom_options.get("DefaultValue"),
            Some(&"3".to_string())
        );
        assert!(eds.objects[&0x1001].subs.is_empty(), "same for ARRAY");
        // A VAR (and a section without `ObjectType`, which CiA 306 §4.6.3.2
        // defines as VAR) still gets its variable.
        assert_eq!(
            eds.objects[&0x1002].subs[&0].default,
            Some(OdExpr::Literal(7))
        );
        let od = eds.to_object_dictionary(1).unwrap();
        assert!(od.read(Address::new(0x1000, 0)).is_err());
        assert!(od.read(Address::new(0x1001, 0)).is_err());
        assert_eq!(od.read(Address::new(0x1002, 0)).unwrap(), Value::Uint(7));
    }

    /// docs/DESIGN-DECISIONS.md EDS-12: `LowLimit`/`HighLimit` are parsed into
    /// `low_limit`/`high_limit` with python's `_signed_int_from_hex`
    /// semantics (two's complement for signed types) and a malformed literal
    /// warns instead of vanishing into `custom_options`.
    #[test]
    fn e3_low_and_high_limits_are_parsed() {
        let eds = parse(
            // 0x0003 = INTEGER16, 0x0006 = UNSIGNED16 (CiA 306 data types).
            "[2000]\nParameterName=Signed\nDataType=0x0003\n\
             LowLimit=0xFFFF\nHighLimit=0x7FFF\n\
             [2001]\nParameterName=Unsigned\nDataType=0x0006\n\
             LowLimit=0\nHighLimit=0xFFFF\n\
             [2002]\nParameterName=Broken\nDataType=0x0003\nHighLimit=zzz\n",
        )
        .unwrap();
        assert_eq!(eds.objects[&0x2000].subs[&0].data_type, DataType::Integer16);
        // 0xFFFF on a signed 16-bit entry is -1, exactly like python
        // (`_signed_int_from_hex`, eds.py:222-245).
        assert_eq!(eds.objects[&0x2000].subs[&0].low_limit, Some(-1));
        assert_eq!(eds.objects[&0x2000].subs[&0].high_limit, Some(0x7FFF));
        // Unsigned entries take the plain integer.
        assert_eq!(eds.objects[&0x2001].subs[&0].low_limit, Some(0));
        assert_eq!(eds.objects[&0x2001].subs[&0].high_limit, Some(0xFFFF));
        // A malformed literal warns and stays unset.
        assert_eq!(eds.objects[&0x2002].subs[&0].high_limit, None);
        assert!(
            eds.warnings
                .iter()
                .any(|w| w.contains("invalid HighLimit") && w.contains("zzz")),
            "{:?}",
            eds.warnings
        );
        // The keys are no longer parked as vendor options.
        assert!(!eds.objects[&0x2000].custom_options.contains_key("LowLimit"));
    }

    /// docs/DESIGN-DECISIONS.md EDS-13: REAL defaults follow python's `float()` grammar, so a
    /// hex literal or garbage warns and materialises as no default (0.0)
    /// instead of silently becoming 16.0 / 0.0 depending on the build exit.
    #[test]
    fn e3_real_defaults_follow_the_python_float_grammar() {
        let eds = parse(
            "[2000]\nParameterName=Good\nDataType=0x0008\nDefaultValue=1.5\n\
             [2001]\nParameterName=Hex\nDataType=0x0008\nDefaultValue=0x10\n\
             [2002]\nParameterName=Garbage\nDataType=0x0008\nDefaultValue=abc\n\
             [2003]\nParameterName=Int\nDataType=0x0008\nDefaultValue=16\n",
        )
        .unwrap();
        let od = eds.to_object_dictionary(1).unwrap();
        assert_eq!(od.read(Address::new(0x2000, 0)).unwrap(), Value::Real(1.5));
        assert_eq!(od.read(Address::new(0x2001, 0)).unwrap(), Value::Real(0.0));
        assert_eq!(od.read(Address::new(0x2002, 0)).unwrap(), Value::Real(0.0));
        // A plain integer is a valid float in python.
        assert_eq!(od.read(Address::new(0x2003, 0)).unwrap(), Value::Real(16.0));
        for (index, spelling) in [(0x2001, "0x10"), (0x2002, "abc")] {
            assert!(
                eds.warnings
                    .iter()
                    .any(|w| w.contains(&format!("{index:#06x}")) && w.contains(spelling)),
                "expected an invalid-default warning for {spelling:?}: {:?}",
                eds.warnings
            );
        }
        // Both build exits agree.
        let source = crate::codegen::generate_od_module(&eds, "od_real").expect("codegen");
        assert!(
            source.contains("Value::Real(1.5)"),
            "float default lost:
{source}"
        );
        assert!(
            !source.contains("Value::Real(16.0) || 16.0"),
            "hex float default must not be read as an integer:
{source}"
        );
        // …the hex one is emitted as 0.0 (python's ignored default).
        assert_eq!(
            source.matches("Value::Real(0.0)").count(),
            2,
            "hex + garbage float defaults are ignored in codegen too:
{source}"
        );
    }

    /// docs/DESIGN-DECISIONS.md EDS-05: a UNICODE_STRING default and an SDO read of the same
    /// entry are the same `Value` kind, so the value no longer changes shape
    /// (and `Display`/persistence) after one write.
    #[test]
    fn e3_unicode_default_and_readback_agree() {
        let eds =
            parse("[2000]\nParameterName=U\nDataType=0x000B\nAccessType=rw\nDefaultValue=abc\n")
                .unwrap();
        let od = eds.to_object_dictionary(1).unwrap();
        let default = od.read(Address::new(0x2000, 0)).unwrap().clone();
        assert_eq!(
            default,
            Value::Bytes("abc".encode_utf16().flat_map(u16::to_le_bytes).collect())
        );
        // Round-trip through the wire form: what a read returns equals the
        // default (八M5 decode keeps bytes).
        let mut wire = [0u8; 8];
        let n = default
            .encode_as(DataType::UnicodeString, &mut wire)
            .unwrap();
        assert_eq!(
            Value::decode_le(DataType::UnicodeString, &wire[..n]).unwrap(),
            default
        );
    }

    /// docs/DESIGN-DECISIONS.md EDS-19: an indented continuation line is joined into the
    /// previous value with `\n` (configparser semantics, measured on the
    /// local python), instead of being dropped as a stray line. An indented
    /// `key=value` is a continuation too, and a file indented as a whole keeps
    /// its options separate (the indent is compared *relatively*).
    #[test]
    fn e3_continuation_lines_are_joined_into_the_previous_value() {
        let eds = parse("[2000]\nParameterName=Long name\n    continued here\nDataType=0x0007\n")
            .unwrap();
        let sub = &eds.objects[&0x2000].subs[&0];
        assert_eq!(sub.name, "Long name\ncontinued here");
        // The following option is untouched by the continuation.
        assert_eq!(sub.data_type, DataType::Unsigned32);
        assert!(
            !eds.warnings.iter().any(|w| w.contains("without '='")),
            "a continuation must not be reported as a stray line: {:?}",
            eds.warnings
        );

        // An indented `key=value` continues the value (configparser), so no
        // separate option is created for it.
        let eds = parse("[2000]\nParameterName=A\n    B=1\n").unwrap();
        assert_eq!(eds.objects[&0x2000].name, "A\nB=1");
        assert!(
            !eds.objects[&0x2000].custom_options.contains_key("B"),
            "an indented key=value is a continuation, not an option"
        );

        // A uniformly indented file keeps its options apart.
        let eds =
            parse("    [2000]\n    ParameterName=Gain\n    DataType=0x0007\n    DefaultValue=3\n")
                .unwrap();
        let sub = &eds.objects[&0x2000].subs[&0];
        assert_eq!(sub.name, "Gain");
        assert_eq!(sub.default, Some(OdExpr::Literal(3)));
    }

    /// docs/DESIGN-DECISIONS.md EDS-20: a repeated option or section is last-wins (as before)
    /// but now says so; python-canopen aborts the import instead.
    #[test]
    fn e3_duplicate_options_and_sections_warn() {
        let eds = parse("[2000]\nParameterName=a\nParameterName=b\n").unwrap();
        assert_eq!(eds.objects[&0x2000].name, "b", "last wins");
        assert!(
            eds.warnings
                .iter()
                .any(|w| w.contains("duplicate EDS option") && w.contains("ParameterName")),
            "{:?}",
            eds.warnings
        );

        let eds = parse("[2000]\nParameterName=a\n[2000]\nParameterName=c\n").unwrap();
        assert_eq!(eds.objects[&0x2000].name, "c", "last wins");
        assert!(
            eds.warnings
                .iter()
                .any(|w| w.contains("duplicate EDS section")),
            "{:?}",
            eds.warnings
        );
    }

    /// docs/DESIGN-DECISIONS.md EDS-22: a 1–3 digit index section stays accepted (lenient
    /// superset) but is now reported; python-canopen ignores such a section.
    #[test]
    fn e3_short_index_section_is_accepted_with_a_warning() {
        let eds = parse("[100]\nParameterName=Short\nDataType=0x0007\n").unwrap();
        // Still built (nothing is lost).
        assert_eq!(eds.objects[&0x0100].name, "Short");
        assert!(
            eds.warnings
                .iter()
                .any(|w| w.contains("not a 4-digit index")),
            "{:?}",
            eds.warnings
        );
        // A proper 4-digit header stays silent.
        let eds = parse("[0100]\nParameterName=Ok\n").unwrap();
        assert_eq!(eds.objects[&0x0100].name, "Ok");
        assert!(
            !eds.warnings.iter().any(|w| w.contains("4-digit")),
            "{:?}",
            eds.warnings
        );
    }

    /// docs/DESIGN-DECISIONS.md EDS-03: an object section that only carries `ParameterName`
    /// evaporates silently when the dictionary is built. The loss is now
    /// visible in `warnings`.
    #[test]
    fn e3_name_only_object_is_reported() {
        let eds = parse("[2000]\nParameterName=Lonely\n[2001]\nParameterName=V\nDataType=0x0007\n")
            .unwrap();
        assert!(
            eds.warnings
                .iter()
                .any(|w| w.contains("0x2000") && w.contains("no parsable subentry")),
            "{:?}",
            eds.warnings
        );
        // …and the parsable object is untouched.
        assert!(
            !eds.warnings
                .iter()
                .any(|w| w.contains("0x2001") && w.contains("no parsable subentry")),
            "{:?}",
            eds.warnings
        );
    }

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
                type_defs: BTreeMap::new(),
                dummy_usage: BTreeMap::new(),
                ..Eds::default()
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

    /// docs/DESIGN-DECISIONS.md B3-3：整数类型条目的字符串默认值若超出 i64 范围（如
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

    /// docs/DESIGN-DECISIONS.md B3-3 反向：合法范围内的整数默认仍正常物化（无回归）。
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

    /// docs/DESIGN-DECISIONS.md M1：`int_bounds` 对 `Unsigned64` 上界曾截成 `i64::MAX`，
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

    /// docs/DESIGN-DECISIONS.md M1：U64 上界现在确实是 `u64::MAX`，`0x1_0000_0000_0000_0000`
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

    /// docs/DESIGN-DECISIONS.md M2：无符号类型收到负默认（如 `DefaultValue=-1`）运行时必须
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

    /// docs/DESIGN-DECISIONS.md N10：UNICODE_STRING 默认值必须保持为字符串（线上按 UTF-16LE
    /// 编码，python `_decode_from_eds` 对 VISIBLE/UNICODE 返回原始 str），
    /// 而不是按 hex 解码成 `Value::Bytes`。
    #[test]
    fn n10_unicode_string_default_stays_a_string() {
        const EDS: &str = r#"
[2001]
ParameterName=Unicode
DataType=0x000B
AccessType=rw
DefaultValue=abc

[2002]
ParameterName=Quoted
DataType=0x000B
AccessType=rw
DefaultValue="hello"
"#;
        let eds = parse(EDS).unwrap();
        let od = eds.to_object_dictionary(1).unwrap();
        // docs/DESIGN-DECISIONS.md EDS-05 (supersedes the N10 `Value::Str` form): the default is
        // materialised as the **wire** bytes — UTF-16LE — exactly like an SDO
        // read of the same entry (八M5). The previous `Value::Str` meant one
        // SDO write flipped the value's kind, and `Display`/`PartialEq`/the
        // persisted payload drifted with it.
        assert_eq!(
            od.read(Address::new(0x2001, 0)).unwrap(),
            Value::Bytes("abc".encode_utf16().flat_map(u16::to_le_bytes).collect())
        );
        assert_eq!(
            od.read(Address::new(0x2002, 0)).unwrap(),
            Value::Bytes("hello".encode_utf16().flat_map(u16::to_le_bytes).collect())
        );
        // …and that is byte-for-byte what python-canopen's `encode_raw` would
        // put on the wire (`objectdictionary/__init__.py:468-474`).
        let mut buf = [0u8; 8];
        let n = Value::Str("abc".to_string())
            .encode_as(DataType::UnicodeString, &mut buf)
            .unwrap();
        assert_eq!(&buf[..n], &[0x61, 0, 0x62, 0, 0x63, 0]);
        // A read of the same bytes decodes back to the identical value, so the
        // two paths no longer disagree.
        let default = od.read(Address::new(0x2001, 0)).unwrap().clone();
        let mut wire = [0u8; 8];
        let n = default
            .encode_as(DataType::UnicodeString, &mut wire)
            .unwrap();
        assert_eq!(
            Value::decode_le(DataType::UnicodeString, &wire[..n]).unwrap(),
            default
        );
    }

    /// docs/DESIGN-DECISIONS.md N10：OCTET_STRING/DOMAIN 的非法 hex 默认值只触发警告并保持
    /// 空 blob（python `bytes.fromhex` 抛 `ValueError` → warning + 默认被
    /// 忽略，`eds.py:246-250,365-372`），不再静默无痕。
    #[test]
    fn n10_invalid_hex_default_warns_and_stays_empty() {
        const EDS: &str = r#"
[2003]
ParameterName=Octet
DataType=0x000A
AccessType=rw
DefaultValue=@ABCD

[2004]
ParameterName=GoodHex
DataType=0x000A
AccessType=rw
DefaultValue=01 A2 ff
"#;
        let eds = parse(EDS).unwrap();
        assert!(
            eds.warnings
                .iter()
                .any(|w| w.contains("invalid DefaultValue") && w.contains("@ABCD")),
            "expected an invalid-hex warning, got {:?}",
            eds.warnings
        );
        let od = eds.to_object_dictionary(1).unwrap();
        assert_eq!(
            od.read(Address::new(0x2003, 0)).unwrap(),
            Value::Bytes(Vec::new())
        );
        assert_eq!(
            od.read(Address::new(0x2004, 0)).unwrap(),
            Value::Bytes(vec![0x01, 0xA2, 0xFF])
        );
    }

    /// docs/DESIGN-DECISIONS.md N11：`[DummyUsage] Dummy000n=1` 生成 index=n、DataType=n 的
    /// `const` 虚拟条目（python-canopen `eds.py:105-113`）。
    #[test]
    fn n11_dummy_usage_creates_const_entries() {
        const EDS: &str = r#"
[DummyUsage]
Dummy0001=1
Dummy0002=0
Dummy0005=1
Dummy0008=1

[2000]
ParameterName=Real
DataType=0x0005
AccessType=rw
DefaultValue=7
"#;
        let eds = parse(EDS).unwrap();
        assert_eq!(eds.dummy_usage.get(&1), Some(&1));
        assert_eq!(eds.dummy_usage.get(&5), Some(&5));
        assert!(!eds.dummy_usage.contains_key(&2), "0 does not activate");
        assert!(!eds.dummy_usage.contains_key(&8), "out of 1..=7 ignored");
        let od = eds.to_object_dictionary(1).unwrap();
        let e1 = od.entry(Address::new(0x0001, 0)).expect("dummy 1");
        assert_eq!(e1.data_type, DataType::Boolean);
        assert_eq!(e1.access, AccessType::Const);
        assert_eq!(
            od.read(Address::new(0x0001, 0)).unwrap(),
            Value::Bool(false)
        );
        let e5 = od.entry(Address::new(0x0005, 0)).expect("dummy 5");
        assert_eq!(e5.data_type, DataType::Unsigned8);
        assert_eq!(od.read(Address::new(0x0005, 0)).unwrap(), Value::Uint(0));
    }

    /// docs/DESIGN-DECISIONS.md 七E2: `ParameterValue` is a standard option parsed like
    /// `DefaultValue`, overrides it for the initial dictionary value
    /// (python `node/local.py:99-107`) and must not leak into
    /// `custom_options`.
    #[test]
    fn e2_parameter_value_overrides_default() {
        const EDS: &str = r#"
        [2000]
        ParameterName=Gain
        DataType=0x0007
        AccessType=rw
        DefaultValue=1
        ParameterValue=42
        "#;
        let eds = parse(EDS).unwrap();
        let sub = &eds.objects[&0x2000].subs[&0];
        assert_eq!(sub.parameter_value, Some(OdExpr::Literal(42)));
        assert!(
            !sub.custom_options.contains_key("ParameterValue"),
            "ParameterValue is a standard option, not a custom one"
        );
        let od = eds.to_object_dictionary(1).unwrap();
        assert_eq!(
            od.read(Address::new(0x2000, 0)).unwrap(),
            Value::Uint(42),
            "ParameterValue must win over DefaultValue"
        );
    }

    /// docs/DESIGN-DECISIONS.md 七E4: python's `int(x, 0)` also accepts `0b`/`0o` prefixes and
    /// `_` digit separators.
    #[test]
    fn e4_int_literals_accept_binary_octal_and_separators() {
        assert_eq!(parse_int("0b1011"), Some(11));
        assert_eq!(parse_int("-0B10"), Some(-2));
        assert_eq!(parse_int("0o17"), Some(15));
        assert_eq!(parse_int("0O10"), Some(8));
        assert_eq!(parse_int("1_000_000"), Some(1_000_000));
        assert_eq!(parse_int("0xFF_FF"), Some(0xFFFF));
        assert_eq!(
            parse_integer_default("0b1111", DataType::Unsigned8).unwrap(),
            15
        );
        assert_eq!(
            parse_integer_default("0o777", DataType::Unsigned16).unwrap(),
            0o777
        );
        assert_eq!(
            parse_integer_default("1_2_3", DataType::Unsigned8).unwrap(),
            123
        );
    }

    /// docs/DESIGN-DECISIONS.md EDS-21: `[FileInfo]` / `[DeviceInfo]` / `[Comments]` and
    /// `[DeviceComissioning] Baudrate` are collected instead of dropped.
    #[test]
    fn eds21_collects_file_device_comment_and_baudrate_metadata() {
        const EDS: &str = r#"
[FileInfo]
FileName=device.eds
FileVersion=2
FileRevision=7
EDSVersion=4.2
Description=Demo device
CreationTime=12:34:56
CreationDate=10-23-2026
ModificationTime=01:02:03
ModificationDate=10-24-2026
CreatedBy=Test
ModifiedBy=Test2

[DeviceInfo]
VendorName=Acme
VendorNumber=0x1234
ProductName=Widget
ProductNumber=0x5678
RevisionNumber=0x9ABC
OrderCode=W-1
BaudRate_10=0
BaudRate_20=1
BaudRate_50=0
BaudRate_125=1
BaudRate_250=0
BaudRate_500=1
BaudRate_800=0
BaudRate_1000=1
SimpleBootUpMaster=0
SimpleBootUpSlave=1
Granularity=8
DynamicChannelsSupported=2
GroupMessaging=1
NrOfRXPDO=4
NrOfTXPDO=5
LSS_Supported=1

[Comments]
Lines=2
Line1=first line
Line2=second line

[DeviceComissioning]
NodeID=0x20
Baudrate=500
"#;
        let eds = parse(EDS).unwrap();
        assert_eq!(eds.file_info.file_name, "device.eds");
        assert_eq!(eds.file_info.file_version, 2);
        assert_eq!(eds.file_info.file_revision, 7);
        assert_eq!(eds.file_info.eds_version, "4.2");
        assert_eq!(eds.file_info.description, "Demo device");
        assert_eq!(eds.file_info.created_by, "Test");
        assert_eq!(eds.file_info.modified_by, "Test2");
        assert_eq!(eds.device_info.vendor_name, "Acme");
        assert_eq!(eds.device_info.vendor_number, Some(0x1234));
        assert_eq!(eds.device_info.product_name, "Widget");
        assert_eq!(eds.device_info.product_number, Some(0x5678));
        assert_eq!(eds.device_info.revision_number, Some(0x9ABC));
        assert_eq!(eds.device_info.order_code, "W-1");
        assert!(!eds.device_info.baudrate_10);
        assert!(eds.device_info.baudrate_20);
        assert!(eds.device_info.baudrate_125);
        assert!(eds.device_info.baudrate_500);
        assert!(eds.device_info.baudrate_1000);
        assert!(!eds.device_info.simple_bootup_master);
        assert!(eds.device_info.simple_bootup_slave);
        assert_eq!(eds.device_info.granularity, 8);
        assert_eq!(eds.device_info.dynamic_channels_supported, 2);
        assert!(eds.device_info.group_messaging);
        assert_eq!(eds.device_info.nr_of_rx_pdo, 4);
        assert_eq!(eds.device_info.nr_of_tx_pdo, 5);
        assert!(eds.device_info.lss_supported);
        assert_eq!(eds.comments.lines, vec!["first line", "second line"]);
        assert_eq!(eds.baudrate, Some(500));
        assert_eq!(eds.configured_node_id, Some(OdExpr::Literal(0x20)));
    }

    /// docs/DESIGN-DECISIONS.md EDS-17: strict build aborts on an out-of-range default;
    /// lenient build keeps the entry with a typed zero and a warning.
    #[test]
    fn eds17_build_lenient_keeps_entry_and_warns() {
        const EDS: &str = r#"
[2000]
ParameterName=Too large
DataType=0x0005
AccessType=rw
DefaultValue=0x1FFFF

[2001]
ParameterName=Fine
DataType=0x0007
AccessType=ro
DefaultValue=1
"#;
        let eds = parse(EDS).unwrap();
        assert!(eds.build(1).is_err(), "strict build must abort");
        assert!(eds.build_typed(1).is_err(), "typed strict build must abort");
        let (od, warnings) = eds.build_lenient(1).unwrap();
        assert_eq!(
            od.read(Address::new(0x2000, 0)).unwrap(),
            Value::Uint(0),
            "lenient fallback is a typed zero"
        );
        assert_eq!(od.read(Address::new(0x2001, 0)).unwrap(), Value::Uint(1));
        assert!(
            warnings.iter().any(|w| w.contains("dropped default")),
            "expected a content warning, got {warnings:?}"
        );
    }

    /// docs/DESIGN-DECISIONS.md EDS-21: a malformed numeric metadata value does not poison the
    /// rest of the file.
    #[test]
    fn eds21_malformed_metadata_is_ignored() {
        const EDS: &str = r#"
[DeviceInfo]
VendorName=Acme
VendorNumber=not-a-number
BaudRate_500=0
"#;
        let eds = parse(EDS).unwrap();
        assert_eq!(eds.device_info.vendor_name, "Acme");
        assert_eq!(eds.device_info.vendor_number, None);
        assert!(!eds.device_info.baudrate_500);
    }

    /// docs/DESIGN-DECISIONS.md 七E1: a float default on a Boolean entry is ignored with a
    /// warning (python's `int("1.5", 0)` raises), not silently zeroed
    /// through different paths in runtime and codegen.
    #[test]
    fn e1_float_default_on_boolean_warns_and_stays_false() {
        const EDS: &str = r#"
        [2000]
        ParameterName=Flag
        DataType=0x0001
        AccessType=rw
        DefaultValue=1.5
        "#;
        let eds = parse(EDS).unwrap();
        assert!(
            eds.warnings
                .iter()
                .any(|w| w.contains("invalid DefaultValue") && w.contains("1.5")),
            "expected an invalid-default warning, got {:?}",
            eds.warnings
        );
        let od = eds.to_object_dictionary(1).unwrap();
        assert_eq!(
            od.read(Address::new(0x2000, 0)).unwrap(),
            Value::Bool(false)
        );
    }
    #[test]
    fn eds30_explicit_sub_zero_survives_both_key_orders() {
        const A: &str = "[1018]\nDataType=0x0005\nAccessType=ro\nDefaultValue=0x1234\nObjectType=0x9\n\n[1018sub0]\nObjectType=0x7\nDataType=0x0005\nAccessType=ro\nDefaultValue=1\n\n[1018sub1]\nObjectType=0x7\nDataType=0x0007\nAccessType=ro\nDefaultValue=2\n";
        const B: &str = "[1018sub0]\nObjectType=0x7\nDataType=0x0005\nAccessType=ro\nDefaultValue=1\n\n[1018]\nDataType=0x0007\nAccessType=rw\nDefaultValue=0x1234\nObjectType=0x9\n";
        for text in [A, B] {
            let eds = parse(text).unwrap();
            let od = eds.to_object_dictionary(5).unwrap();
            assert_eq!(od.read(Address::new(0x1018, 0)).unwrap(), Value::Uint(1));
            let e = od.entry(Address::new(0x1018, 0)).unwrap();
            assert_eq!(e.data_type, DataType::Unsigned8);
            assert_eq!(e.access, AccessType::Const);
        }
    }

    #[test]
    fn eds31_compact_sub_obj_after_inline_keys_expands() {
        let eds = parse("[2100]\nParameterName=AI\nDataType=0x0003\nAccessType=rw\nDefaultValue=9\nObjectType=0x8\nCompactSubObj=3\n").unwrap();
        let od = eds.to_object_dictionary(5).unwrap();
        assert_eq!(od.read(Address::new(0x2100, 0)).unwrap(), Value::Uint(3));
        assert_eq!(od.read(Address::new(0x2100, 1)).unwrap(), Value::Int(9));
    }

    #[test]
    fn eds32_33_node_relative_values_are_not_rejected() {
        let e = parse("[2000]\nDataType=0x0007\nAccessType=ro\nDefaultValue=1\nParameterValue=$NODEID+0x200\n").unwrap();
        assert_eq!(
            e.to_object_dictionary(5)
                .unwrap()
                .read(Address::new(0x2000, 0))
                .unwrap(),
            Value::Uint(517)
        );
        assert!(!e
            .warnings
            .iter()
            .any(|w| w.contains("invalid ParameterValue")));
        let e =
            parse("[2001]\nDataType=0x0007\nAccessType=ro\nDefaultValue=$NODEID+0x200\n").unwrap();
        assert_eq!(
            e.to_object_dictionary(5)
                .unwrap()
                .read(Address::new(0x2001, 0))
                .unwrap(),
            Value::Uint(517)
        );
        assert!(!e
            .warnings
            .iter()
            .any(|w| w.contains("invalid DefaultValue")));
    }

    #[test]
    fn eds34_lenient_time_zero_is_six_bytes() {
        let e = parse("[2000]\nDataType=0x000C\nAccessType=ro\nDefaultValue=zz\n").unwrap();
        let (od, _) = e.build_lenient(1).unwrap();
        assert_eq!(
            od.read(Address::new(0x2000, 0)).unwrap(),
            Value::Bytes(vec![0; 6])
        );
    }

    #[test]
    fn eds35_unicode_default_has_no_trailing_nul() {
        let e = parse("[2000]\nDataType=0x000B\nAccessType=rw\nDefaultValue=héllo\n").unwrap();
        assert_eq!(
            e.to_object_dictionary(1)
                .unwrap()
                .read(Address::new(0x2000, 0))
                .unwrap(),
            Value::Bytes(vec![104, 0, 233, 0, 108, 0, 108, 0, 111, 0])
        );
    }

    #[test]
    fn eds36_integer64_node_relative_overflow_is_rejected() {
        let e = parse(
            "[2000]\nDataType=0x0015\nAccessType=ro\nDefaultValue=$NODEID+0x7FFFFFFFFFFFFFFF\n",
        )
        .unwrap();
        assert!(e.to_object_dictionary(1).is_err());
    }

    #[test]
    fn eds37_wide_integer_garbage_warns() {
        let e = parse("[2000]\nDataType=0x0010\nAccessType=ro\nDefaultValue=abc\n").unwrap();
        assert!(e
            .warnings
            .iter()
            .any(|w| w.contains("invalid DefaultValue")));
    }
}
