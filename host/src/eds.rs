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

/// An integer that may depend on the node id.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OdExpr {
    /// A literal value.
    Literal(i64),
    /// `node_id + offset` — resolved at dictionary build time.
    NodeRelative {
        /// Constant added to the runtime node id.
        offset: i64,
    },
}

impl OdExpr {
    /// Parses an EDS default-value expression.
    #[must_use]
    pub fn parse(raw: &str) -> Option<Self> {
        let raw = raw.trim();
        let lower = raw.to_ascii_lowercase();
        if let Some(rest) = lower.strip_prefix("$nodeid") {
            let rest = rest.trim_start();
            if rest.is_empty() {
                return Some(Self::NodeRelative { offset: 0 });
            }
            let offset = rest.strip_prefix('+')?.trim();
            return Some(Self::NodeRelative {
                offset: parse_int(offset)?,
            });
        }
        Some(Self::Literal(parse_int(raw)?))
    }

    /// Resolves against a concrete node id.
    #[must_use]
    pub fn resolve(&self, node_id: u8) -> i64 {
        match self {
            Self::Literal(v) => *v,
            Self::NodeRelative { offset } => i64::from(node_id) + offset,
        }
    }
}

/// Parses decimal or `0x`-prefixed hexadecimal integers with optional sign.
#[must_use]
pub fn parse_int(raw: &str) -> Option<i64> {
    let raw = raw.trim();
    let (negative, digits) = match raw.strip_prefix('-') {
        Some(rest) => (true, rest.trim()),
        None => (false, raw),
    };
    let magnitude = if let Some(hex) = digits
        .strip_prefix("0x")
        .or_else(|| digits.strip_prefix("0X"))
    {
        i64::from_str_radix(hex, 16).ok()?
    } else {
        digits.parse::<i64>().ok()?
    };
    Some(if negative { -magnitude } else { magnitude })
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
    /// PDOMapping flag = 1.
    pub pdo_mappable: bool,
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
}

/// Parses an EDS/DCF file from its full text.
///
/// # Errors
/// [`Error::Other`] on malformed section headers or unparsable indices.
pub fn parse(text: &str) -> Result<Eds> {
    let mut eds = Eds::default();
    let mut current: Option<(u16, Option<u8>)> = None;

    for raw_line in text.lines() {
        // Comments start with ';'.
        let line = raw_line.split(';').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        if line.starts_with('[') && line.ends_with(']') {
            let header = parse_header(&line[1..line.len() - 1])
                .ok_or(Error::Other("malformed EDS section header"))?;
            current = Some(header);
            continue;
        }
        let Some((index, sub)) = current else {
            continue;
        };
        let Some((key, value)) = line.split_once('=') else {
            return Err(Error::Other("EDS line without '='"));
        };
        let key = key.trim();
        let value = value.trim();

        match (index, sub) {
            (_, _) if index == 0xFFFF => {} // reserved noise sections ignored
            (_, None) if index == u16::MAX - 1 => {}
            _ => {}
        }

        // Device commissioning section carries no numeric index in our model;
        // it is detected by header parsing below via sentinel.
        if index == COMMISSIONING_SENTINEL {
            if key.eq_ignore_ascii_case("NodeId") || key.eq_ignore_ascii_case("NodeIDSet") {
                eds.configured_node_id = OdExpr::parse(value);
            }
            continue;
        }

        let object = eds.objects.entry(index).or_insert_with(|| EdsObject {
            index,
            name: String::new(),
            subs: BTreeMap::new(),
            compact_sub_obj: None,
            custom_options: BTreeMap::new(),
        });

        match sub {
            None => match key {
                "ParameterName" => object.name = unquote(value),
                "CompactSubObj" => {
                    object.compact_sub_obj =
                        parse_int(value).and_then(|n| u16::try_from(n.max(0)).ok());
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
                            pdo_mappable: false,
                        }),
                        key,
                        value,
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
            },
            Some(sub) => {
                apply_key(
                    object.subs.entry(sub).or_insert_with(|| EdsSubEntry {
                        name: String::new(),
                        data_type: DataType::Unsigned32,
                        access: AccessType::Rw,
                        default: None,
                        pdo_mappable: false,
                    }),
                    key,
                    value,
                );
            }
        }
    }
    Ok(eds)
}

/// Sentinel pseudo-index carrying the `[DeviceComissioning]` section.
const COMMISSIONING_SENTINEL: u16 = 0xFFFE;

fn parse_header(header: &str) -> Option<(u16, Option<u8>)> {
    let lower = header.to_ascii_lowercase();
    if lower.starts_with("devicecomi") {
        // DeviceComissioning / DeviceCommissioning.
        return Some((COMMISSIONING_SENTINEL, None));
    }
    if let Some((idx_part, sub_part)) = lower.split_once("sub") {
        let index = u16::from_str_radix(idx_part.trim(), 16).ok()?;
        let sub = sub_part.trim().parse::<u8>().ok()?;
        Some((index, Some(sub)))
    } else {
        let index = u16::from_str_radix(header.trim(), 16).ok()?;
        Some((index, None))
    }
}

fn apply_key(entry: &mut EdsSubEntry, key: &str, value: &str) {
    match key {
        "ParameterName" => entry.name = unquote(value),
        "DataType" => {
            if let Some(idx) = parse_int(value) {
                entry.data_type = DataType::from_index(idx as u16).unwrap_or(DataType::Unsigned32);
            }
        }
        "AccessType" => {
            entry.access = match value.to_ascii_lowercase().as_str() {
                "ro" | "const" => AccessType::Const,
                "wo" => AccessType::Wo,
                // rwr/rww behave as Rw for the SDO-visible surface.
                _ => AccessType::Rw,
            };
            if value.eq_ignore_ascii_case("const") {
                entry.access = AccessType::Const;
            }
        }
        "DefaultValue" => entry.default = OdExpr::parse(value),
        "PDOMapping" => entry.pdo_mappable = parse_int(value).unwrap_or(0) & 1 == 1,
        _ => {}
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
        let raw = self.default.map(|e| e.resolve(node_id));
        Ok(match self.data_type {
            DataType::Boolean => Value::Bool(raw.unwrap_or(0) != 0),
            DataType::VisibleString
            | DataType::OctetString
            | DataType::UnicodeString
            | DataType::Domain => match (&self.default, self.data_type) {
                (Some(OdExpr::Literal(_)), DataType::VisibleString) => {
                    // Strings cannot round-trip through i64; re-read raw is
                    // unavailable here, so fall back to empty.
                    Value::Str(String::new())
                }
                _ => Value::Bytes(Vec::new()),
            },
            DataType::Integer8
            | DataType::Integer16
            | DataType::Integer24
            | DataType::Integer32
            | DataType::Integer40
            | DataType::Integer48
            | DataType::Integer56
            | DataType::Integer64 => Value::Int(raw.unwrap_or(0)),
            dt if dt.fixed_size().is_some() => Value::Uint(raw.unwrap_or(0).max(0) as u64),
            _ => Value::Uint(0),
        })
    }
}

impl Eds {
    /// Builds a runtime [`ObjectDictionary`] bound to `node_id`.
    ///
    /// Objects without any parsable subentries are skipped; string defaults
    /// are currently materialised as empty values (numeric semantics are the
    /// production focus).
    ///
    /// # Errors
    /// Propagates per-entry validation errors.
    pub fn to_object_dictionary(&self, node_id: u8) -> Result<ObjectDictionary> {
        let mut od = ObjectDictionary::new();
        for object in self.objects.values() {
            // CompactSubObj expansion: the parser lands the [idx] section's
            // inline definition in the sub-0 slot — that IS the template.
            // Expansion replaces it: sub0 becomes the element count and
            // subs 1..=N are deep copies of the template.
            let expanded = object.compact_sub_obj.and_then(|n| {
                let template = object.subs.get(&0)?;
                let mut subs = BTreeMap::new();
                subs.insert(
                    0u8,
                    EdsSubEntry {
                        name: "Number of entries".to_string(),
                        data_type: DataType::Unsigned8,
                        access: AccessType::Ro,
                        default: Some(OdExpr::Literal(i64::from(n))),
                        pdo_mappable: false,
                    },
                );
                for i in 1..=n {
                    subs.insert(u8::try_from(i).ok()?, template.clone());
                }
                Some(subs)
            });
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

    #[test]
    fn comments_and_blank_lines_ignored() {
        let eds =
            parse("; header comment\n\n[1000]\n; c\nParameterName=X ; trailing\nDataType=0x0007\n")
                .unwrap();
        assert_eq!(eds.objects[&0x1000].name, "X");
    }
}
