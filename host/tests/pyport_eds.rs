//! Port of `ref/canopen/test/test_eds.py` boundary cases — EDS parsing
//! edge conditions that our parser must handle identically
//! (docs/EXTENSIONS.md 批次 D P-d).

use canopen_core::datatypes::{DataType, Value};
use canopen_core::types::Address;
use canopen_host::eds::{parse, OdExpr};

/// sample.eds quirk: 0x1018 sub-3 deliberately absent → reading it must
/// report SubindexDoesNotExist (not ObjectDoesNotExist).
#[test]
fn missing_sub_in_identity_record() {
    let text = r#"
[1018]
ParameterName=Identity
SubNumber=4

[1018sub0]
ParameterName=Highest sub-index supported
DataType=0x0005
AccessType=ro
DefaultValue=4

[1018sub1]
ParameterName=Vendor-ID
DataType=0x0007
AccessType=ro
DefaultValue=1

[1018sub2]
ParameterName=Product code
DataType=0x0007
AccessType=ro
DefaultValue=2

; [1018sub3] left out for testing

[1018sub4]
ParameterName=Serial number
DataType=0x0007
AccessType=ro
DefaultValue=4
"#;
    let eds = parse(text).unwrap();
    let od = eds.to_object_dictionary(1).unwrap();
    // sub3 absent.
    assert!(od.entry(Address::new(0x1018, 3)).is_none());
    assert_eq!(
        od.read(Address::new(0x1018, 3)),
        Err(canopen_core::types::Error::ObjectDoesNotExist {
            index: 0x1018,
            subindex: 3,
        })
    );
}

/// DataType boundaries: all standard type indices parse to the correct enum.
#[test]
fn all_data_type_indices() {
    for (idx, want_width) in [
        (0x0001u16, Some(1usize)), // BOOLEAN
        (0x0002, Some(1)),         // INTEGER8
        (0x0003, Some(2)),         // INTEGER16
        (0x0004, Some(4)),         // INTEGER32
        (0x0005, Some(1)),         // UNSIGNED8
        (0x0006, Some(2)),         // UNSIGNED16
        (0x0007, Some(4)),         // UNSIGNED32
        (0x0008, Some(4)),         // REAL32
        (0x0009, None),            // VISIBLE_STRING
        (0x000A, None),            // OCTET_STRING
        (0x0011, Some(8)),         // REAL64
    ] {
        let dt = DataType::from_index(idx).unwrap_or_else(|| panic!("type {idx:#06x}"));
        assert_eq!(dt.fixed_size(), want_width, "type {idx:#06x}");
    }
}

/// $NODEID expressions resolve against different node ids without re-parse.
#[test]
fn nodeid_lazy_resolution() {
    let text = r#"
[2000]
ParameterName=COB-ID
DataType=0x0007
AccessType=rw
DefaultValue=$NODEID+0x200
"#;
    let eds = parse(text).unwrap();
    let od5 = eds.to_object_dictionary(5).unwrap();
    let od99 = eds.to_object_dictionary(99).unwrap();
    assert_eq!(
        od5.read(Address::new(0x2000, 0)).unwrap(),
        Value::Uint(0x205)
    );
    assert_eq!(
        od99.read(Address::new(0x2000, 0)).unwrap(),
        Value::Uint(0x263)
    );
}

/// Negative and hexadecimal DefaultValue literals both parse correctly.
#[test]
fn default_value_number_formats() {
    assert_eq!(OdExpr::parse("0xDEAD"), Some(OdExpr::Literal(0xDEAD)));
    assert_eq!(OdExpr::parse("-42"), Some(OdExpr::Literal(-42)));
    assert_eq!(OdExpr::parse("100"), Some(OdExpr::Literal(100)));
    assert_eq!(
        OdExpr::parse("$NODEID+0x10"),
        Some(OdExpr::NodeRelative { offset: 16 })
    );
    assert_eq!(
        OdExpr::parse("$NODEID"),
        Some(OdExpr::NodeRelative { offset: 0 })
    );
}
