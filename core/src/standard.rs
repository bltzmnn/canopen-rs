//! CiA 301 mandatory communication-profile object baseline.
//!
//! [`StandardObjects`] inserts the mandatory objects every CANopen node
//! must expose. All values here are node-id-independent, so the baseline
//! can be populated before LSS assigns a node ID; node-id-dependent
//! COB-IDs (EMCY 0x1014) are rewritten by `Node::set_node_id` later.

use crate::datatypes::{DataType, Value};
use crate::object_dictionary::{AccessType, Entry, ObjectDictionary, Storage};
use crate::types::{Address, NodeId};

/// Builder for the mandatory object baseline.
///
/// # Examples
///
/// ```
/// use canopen_core::standard::StandardObjects;
/// use canopen_core::types::{Address, NodeId};
///
/// let mut od = StandardObjects::new().build();
/// let hb_ms = od.read(Address::new(0x1017, 0)).unwrap();
/// assert_eq!(format!("{hb_ms}"), "0");
/// assert!(od.entry(Address::new(0x1000, 0)).is_some());
/// ```
#[derive(Debug, Default)]
pub struct StandardObjects {
    device_type: u32,
    vendor_id: u32,
    product_code: u32,
    revision: u32,
    serial: u32,
}

impl StandardObjects {
    /// Creates a builder with all-zero defaults.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets object 0x1000 *Device type*.
    #[must_use]
    pub const fn device_type(mut self, value: u32) -> Self {
        self.device_type = value;
        self
    }

    /// Sets identity object 0x1018:1 *Vendor-ID* (also used by LSS).
    #[must_use]
    pub const fn vendor_id(mut self, value: u32) -> Self {
        self.vendor_id = value;
        self
    }

    /// Sets identity object 0x1018:2 *Product code* (also used by LSS).
    #[must_use]
    pub const fn product_code(mut self, value: u32) -> Self {
        self.product_code = value;
        self
    }

    /// Sets identity object 0x1018:3 *Revision number* (also used by LSS).
    #[must_use]
    pub const fn revision_number(mut self, value: u32) -> Self {
        self.revision = value;
        self
    }

    /// Sets identity object 0x1018:4 *Serial number* (also used by LSS).
    #[must_use]
    pub const fn serial_number(mut self, value: u32) -> Self {
        self.serial = value;
        self
    }

    /// Inserts the baseline objects into `od`.
    pub fn apply(&self, od: &mut ObjectDictionary) {
        // 0x1000 Device type — mandatory, ro.
        od.insert(
            Address::new(0x1000, 0),
            Entry::new(
                "Device type",
                DataType::Unsigned32,
                AccessType::Ro,
                Storage::Value(Value::Uint(u64::from(self.device_type))),
            ),
        );

        // 0x1001 Error register — mandatory, ro (maintained by EMCY).
        // docs/DESIGN-DECISIONS.md OD-03: DS-301 declares `PDOMapping=1` for this object
        // (`DS301_profile.eds:66-73`; contrast 0x1000 with `PDOMapping=0`),
        // and mapping the error register into a TPDO is everyday practice.
        // Without the flag such a slot failed `PdoMapping::validate_against`
        // and vanished into the config-error ring.
        od.insert(
            Address::new(0x1001, 0),
            Entry::new(
                "Error register",
                DataType::Unsigned8,
                AccessType::Ro,
                Storage::Value(Value::Uint(0)),
            )
            .pdo_mappable(),
        );

        // 0x1003 Pre-defined error field — sub0 is the number of logged
        // errors (managed by EMCY); error entries are added/removed at
        // runtime by the EMCY producer.
        od.insert(
            Address::new(0x1003, 0),
            Entry::new(
                "Number of errors",
                DataType::Unsigned8,
                AccessType::Rw,
                Storage::Value(Value::Uint(0)),
            ),
        );

        // 0x1005 COB-ID SYNC — rw per CiA 301 (SYNC consumer/producer).
        od.insert(
            Address::new(0x1005, 0),
            Entry::new(
                "COB-ID SYNC message",
                DataType::Unsigned32,
                AccessType::Rw,
                Storage::Value(Value::Uint(0x80)),
            ),
        );

        // 0x1012 COB-ID time stamp object — CiA 301 TIME configuration
        // (`DS301_profile.eds:447-454`): bit31 = consumer, bit30 = producer,
        // bits 10..0 = COB-ID; default 0x100 with both roles disabled.
        // docs/DESIGN-DECISIONS.md SYN-10 used to hardcode 0x100 and consume it unconditionally.
        od.insert(
            Address::new(0x1012, 0),
            Entry::new(
                "COB-ID time stamp object",
                DataType::Unsigned32,
                AccessType::Rw,
                Storage::Value(Value::Uint(0x100)),
            ),
        );
        // 0x1013 High resolution time stamp — optional DS-301 object; stored
        // for masters that read it, with no producer semantics here.
        od.insert(
            Address::new(0x1013, 0),
            Entry::new(
                "High resolution time stamp",
                DataType::Unsigned32,
                AccessType::Rw,
                Storage::Value(Value::Uint(0)),
            ),
        );

        // 0x1014 COB-ID EMCY — defaults to the predefined value for node id 1
        // (`0x81`); `Node` rewrites it on set_node_id.
        od.insert(
            Address::new(0x1014, 0),
            Entry::new(
                "COB-ID EMCY",
                DataType::Unsigned32,
                AccessType::Rw,
                Storage::Value(Value::Uint(0x81)), // 0x80 + node-id(1)
            ),
        );

        // 0x1006 Communication cycle period (µs) — readWrite per DS-301
        // (`DS301_profile.xpd:741`: `UID_OBJ_1006 access="readWrite"`) and
        // consumed by this stack's SYNC silence supervision (`Node::poll`:
        // 1.5× period without a SYNC raises 0x8100). It used to be a
        // read-only skeleton, which made the supervision unreachable through
        // the standard SDO configuration path (docs/DESIGN-DECISIONS.md SYN-07).
        od.insert(
            Address::new(0x1006, 0),
            Entry::new(
                "Communication cycle period",
                DataType::Unsigned32,
                AccessType::Rw,
                Storage::Value(Value::Uint(0)),
            ),
        );
        // 0x100C / 0x100D guard time (ms) / life-time factor. DS-301 declares
        // both `readWrite` (`DS301_profile.xpd:772,779`) and CANopenNode's
        // slave consumes them for node guarding (`CO_Node_Guarding.c:94-152`),
        // so they are writable and consumed by the guarding RTR responder and
        // the slave-side life-time supervision in `Node` (docs/DESIGN-DECISIONS.md NMT-02 /
        // NMT-03).
        od.insert(
            Address::new(0x100C, 0),
            Entry::new(
                "Guard time",
                DataType::Unsigned16,
                AccessType::Rw,
                Storage::Value(Value::Uint(0)),
            ),
        );
        od.insert(
            Address::new(0x100D, 0),
            Entry::new(
                "Life time factor",
                DataType::Unsigned8,
                AccessType::Rw,
                Storage::Value(Value::Uint(0)),
            ),
        );

        // 0x1017 Producer heartbeat time (ms) — rw, 0 disables heartbeat.
        od.insert(
            Address::new(0x1017, 0),
            Entry::new(
                "Producer heartbeat time",
                DataType::Unsigned16,
                AccessType::Rw,
                Storage::Value(Value::Uint(0)),
            ),
        );

        // 0x1019 Synchronous counter overflow — rw. Valid per CiA 301 /
        // CANopenNode `OD_write_1019`: 0 (counters off) or 2..=240; the
        // producer only appends a counter byte when this is non-zero
        // (`sync_time::encode_sync`).
        od.insert(
            Address::new(0x1019, 0),
            Entry::new(
                "Synchronous counter overflow",
                DataType::Unsigned8,
                AccessType::Rw,
                Storage::Value(Value::Uint(0)),
            ),
        );

        // 0x1008/0x1009/0x100A manufacturer device name / hardware version /
        // software version — the three standard communication-area strings of
        // DS-301 (`DS301_profile.xpd:754-766`, `UID_OBJ_1008/1009/100A`).
        // docs/DESIGN-DECISIONS.md OD-02: the previous comment claimed CANopenNode
        // `CO_ODinterface.h:55-80` lists them as *required*, which that header
        // does not do (it is the plain standard-index enumeration, 0x100A at
        // `:67`); the only mandatory communication objects are 0x1000, 0x1001,
        // 0x1017 and 0x1018. Application may overwrite the values via the
        // dictionary.
        od.insert(
            Address::new(0x1008, 0),
            Entry::new(
                "Manufacturer device name",
                DataType::VisibleString,
                AccessType::Ro,
                Storage::Value(Value::Str(String::new())),
            ),
        );
        od.insert(
            Address::new(0x1009, 0),
            Entry::new(
                "Manufacturer hardware version",
                DataType::VisibleString,
                AccessType::Ro,
                Storage::Value(Value::Str(String::new())),
            ),
        );
        // docs/DESIGN-DECISIONS.md OD-02: 0x100A was missing from the baseline although DS-301
        // lists the three strings side by side.
        od.insert(
            Address::new(0x100A, 0),
            Entry::new(
                "Manufacturer software version",
                DataType::VisibleString,
                AccessType::Ro,
                Storage::Value(Value::Str(String::new())),
            ),
        );

        // 0x1015 Emergency inhibit time (100 µs units) — rw; consumed by
        // `Node::raise_emergency`/`poll` (docs/IMPLEMENTATION.md §8.2).
        od.insert(
            Address::new(0x1015, 0),
            Entry::new(
                "Emergency inhibit time",
                DataType::Unsigned16,
                AccessType::Rw,
                Storage::Value(Value::Uint(0)),
            ),
        );

        // 0x1016 Consumer heartbeat time — ARRAY of u32 (nodeId<<16 | ms);
        // writes reconfigure the node's heartbeat consumer immediately
        // (`Node::on_parameter_write`, docs/IMPLEMENTATION.md §6.5). Four
        // slots cover the typical small-network supervision set.
        od.insert(
            Address::new(0x1016, 0),
            Entry::new(
                "Highest sub-index supported",
                DataType::Unsigned8,
                AccessType::Ro,
                Storage::Value(Value::Uint(4)),
            ),
        );
        for sub in 1..=4u8 {
            od.insert(
                Address::new(0x1016, sub),
                Entry::new(
                    "Consumer heartbeat time",
                    DataType::Unsigned32,
                    AccessType::Rw,
                    Storage::Value(Value::Uint(0)),
                ),
            );
        }

        // 0x1280 SDO client parameter (CiA 301 / `DS301_profile.eds:612-652`):
        // a **three**-entry record — sub1 COB-ID client→server, sub2 COB-ID
        // server→client, sub3 the SDO server's node-id — with `SubNumber=4`
        // (sub0 counts sub1..3) and all three values `rw`.
        // docs/DESIGN-DECISIONS.md OD-01: the baseline used to expose a single read-only sub1,
        // so a master walking the DCF record got subindex-missing for
        // 0x1280:2/3 and `0x0601_0002` for a write to 0x1280:1.
        od.insert(
            Address::new(0x1280, 0),
            Entry::new(
                "Highest sub-index supported",
                DataType::Unsigned8,
                AccessType::Const,
                Storage::Value(Value::Uint(3)),
            ),
        );
        // Defaults follow the builder's node-id-1 convention (like 0x1005 =
        // 0x81): 0x600 + 1 and 0x580 + 1. A bare 0x600/0x580 placeholder would
        // never match the shape-guarded renumbering on node-id changes
        // (`docs/DESIGN-DECISIONS.md` P2-2 / P3-38).
        for (sub, name, data_type, value) in [
            (
                1u8,
                "COB-ID client to server (SDO)",
                DataType::Unsigned32,
                0x601u64,
            ),
            (
                2,
                "COB-ID server to client (SDO)",
                DataType::Unsigned32,
                0x581,
            ),
            // docs/DESIGN-DECISIONS.md OD-04: sub3 is **UNSIGNED8** with default 1 in DS-301
            // (`[1280sub3] DataType=0x0005, DefaultValue=0x01`,
            // `DS301_profile.eds:648-652`). Declaring it U32 made a DCF
            // write of the profile's one byte abort `0x0607_0007`
            // (`TypeMismatchLengthLow`) where the reference accepts.
            (3, "Node-ID of the SDO server", DataType::Unsigned8, 1),
        ] {
            od.insert(
                Address::new(0x1280, sub),
                Entry::new(
                    name,
                    data_type,
                    AccessType::Rw,
                    Storage::Value(Value::Uint(value)),
                ),
            );
        }

        // 0x1018 Identity object — mandatory record, ro subentries.
        od.insert(
            Address::new(0x1018, 0),
            Entry::new(
                "Highest sub-index supported",
                DataType::Unsigned8,
                AccessType::Const,
                Storage::Value(Value::Uint(4)),
            ),
        );
        for (sub, name, value) in [
            (1u8, "Vendor-ID", self.vendor_id),
            (2, "Product code", self.product_code),
            (3, "Revision number", self.revision),
            (4, "Serial number", self.serial),
        ] {
            od.insert(
                Address::new(0x1018, sub),
                Entry::new(
                    name,
                    DataType::Unsigned32,
                    AccessType::Ro,
                    Storage::Value(Value::Uint(u64::from(value))),
                ),
            );
        }
    }

    /// Applies the baseline to a fresh dictionary and returns it.
    #[must_use]
    pub fn build(&self) -> ObjectDictionary {
        let mut od = ObjectDictionary::new();
        self.apply(&mut od);
        od
    }

    /// EMCY COB-ID for `node` (`0x80 + node-id`).
    #[must_use]
    pub const fn emcy_cob_id_raw(node: NodeId) -> u16 {
        0x80 + node.get() as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn baseline_contains_mandatory_objects() {
        let od = StandardObjects::new().build();
        for addr in [
            Address::new(0x1000, 0),
            Address::new(0x1001, 0),
            Address::new(0x1003, 0),
            Address::new(0x1005, 0),
            Address::new(0x1014, 0),
            Address::new(0x1017, 0),
            Address::new(0x1019, 0),
            Address::new(0x1018, 0),
            Address::new(0x1018, 1),
            Address::new(0x1018, 2),
            Address::new(0x1018, 3),
            Address::new(0x1018, 4),
        ] {
            assert!(od.read(addr).is_ok(), "mandatory object {addr} missing");
        }
    }

    #[test]
    fn baseline_contains_strings_supervision_and_sdo_client_skeleton() {
        let od = StandardObjects::new().build();
        for addr in [
            Address::new(0x1008, 0),
            Address::new(0x1009, 0),
            // docs/DESIGN-DECISIONS.md OD-02
            Address::new(0x100A, 0),
            Address::new(0x1015, 0),
            Address::new(0x1016, 0),
            // docs/DESIGN-DECISIONS.md OD-01: the full three-entry 0x1280 record.
            Address::new(0x1280, 1),
            Address::new(0x1280, 2),
            Address::new(0x1280, 3),
        ] {
            assert!(od.entry(addr).is_some(), "{addr:?} missing");
        }
        assert_eq!(od.read(Address::new(0x1016, 0)).unwrap(), Value::Uint(4));
        for sub in 1..=4u8 {
            assert_eq!(od.read(Address::new(0x1016, sub)).unwrap(), Value::Uint(0));
        }
        // docs/DESIGN-DECISIONS.md OD-01: sub0 = 3 and all three subentries are writable, as
        // DS-301 declares (`[1280sub0] DefaultValue=0x03`, `AccessType=rw`).
        assert_eq!(od.read(Address::new(0x1280, 0)).unwrap(), Value::Uint(3));
        for sub in 1..=3u8 {
            assert_eq!(
                od.entry(Address::new(0x1280, sub)).unwrap().access,
                AccessType::Rw,
                "0x1280:{sub} must be rw"
            );
        }
        assert_eq!(
            od.read(Address::new(0x1280, 1)).unwrap(),
            Value::Uint(0x601)
        );
        assert_eq!(
            od.read(Address::new(0x1280, 2)).unwrap(),
            Value::Uint(0x581)
        );
        assert_eq!(od.read(Address::new(0x1280, 3)).unwrap(), Value::Uint(1));
        assert_eq!(
            od.entry(Address::new(0x1280, 3)).unwrap().data_type,
            DataType::Unsigned8,
            "DS-301 declares 0x1280:3 as UNSIGNED8"
        );
    }

    #[test]
    fn identity_values_are_applied() {
        let od = StandardObjects::new()
            .vendor_id(0x123)
            .product_code(0x456)
            .revision_number(7)
            .serial_number(0xDEAD_BEEF)
            .build();
        assert_eq!(
            od.read(Address::new(0x1018, 1)).unwrap(),
            Value::Uint(0x123)
        );
        assert_eq!(
            od.read(Address::new(0x1018, 4)).unwrap(),
            Value::Uint(0xDEAD_BEEF)
        );
    }

    #[test]
    fn sync_cob_id_default_and_error_register_zero() {
        let od = StandardObjects::new().build();
        assert_eq!(od.read(Address::new(0x1005, 0)).unwrap(), Value::Uint(0x80));
        assert_eq!(od.read(Address::new(0x1001, 0)).unwrap(), Value::Uint(0));
        assert_eq!(od.read(Address::new(0x1017, 0)).unwrap(), Value::Uint(0));
    }

    #[test]
    fn error_register_is_read_only() {
        let mut od = StandardObjects::new().build();
        assert!(od.write(Address::new(0x1001, 0), Value::Uint(1)).is_err());
    }
}
