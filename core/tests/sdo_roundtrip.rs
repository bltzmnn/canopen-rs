//! SDO client ↔ server integration tests: full transactions against a real
//! object dictionary, without any bus.

use canopen_core::datatypes::{DataType, Value};
use canopen_core::object_dictionary::{AccessType, Entry, ObjectDictionary, Storage};
use canopen_core::sdo::{
    decode_abort, encode_abort, encode_download_segment_response, SdoAbortCode, SdoClient,
    SdoEvent, SdoPayload, SdoServer, SdoServerConfig,
};
use canopen_core::standard::StandardObjects;
use canopen_core::types::{Address, Error};

fn test_od() -> ObjectDictionary {
    let mut od = StandardObjects::new()
        .device_type(0x0000_0192)
        .vendor_id(0x123)
        .build();
    od.insert(
        Address::new(0x2200, 0),
        Entry::new(
            "Device name",
            DataType::VisibleString,
            AccessType::Rw,
            Storage::Value(Value::Str(String::new())),
        ),
    );
    od.insert(
        Address::new(0x2201, 0),
        Entry::new(
            "Secret",
            DataType::Unsigned32,
            AccessType::Wo,
            Storage::Value(Value::Uint(0)),
        ),
    );
    od.insert(
        Address::new(0x2202, 0),
        Entry::new(
            "Highest sub-index",
            DataType::Unsigned8,
            AccessType::Rw,
            Storage::Value(Value::Uint(5)),
        ),
    );
    od.insert(
        Address::new(0x2202, 5),
        Entry::new(
            "Array cell",
            DataType::Unsigned16,
            AccessType::Rw,
            Storage::Value(Value::Uint(7)),
        ),
    );
    od
}

/// Drives whole transactions: relays every frame batch between client and
/// server until the client reports a read completion or write completion.
///
/// Returns the `Complete` value if one occurred; writes report `Done`.
fn drive(
    client: &mut SdoClient,
    server: &mut SdoServer,
    od: &mut ObjectDictionary,
    first: Vec<SdoPayload>,
) -> Result<Option<Value>, Error> {
    let mut queue: std::collections::VecDeque<SdoPayload> = first.into();
    let mut completed = None;
    while let Some(frame) = queue.pop_front() {
        for response in server.handle(od, &frame) {
            match client.on_response(&response)? {
                SdoEvent::Send(frames) => queue.extend(frames),
                SdoEvent::Complete(value, trailing) => {
                    completed = Some(value);
                    queue.extend(trailing);
                }
                SdoEvent::Done => {}
            }
        }
    }
    Ok(completed)
}

#[test]
fn expedited_read_roundtrip() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od();
    let req = client.read(Address::new(0x1000, 0), DataType::Unsigned32);
    // Golden request bytes: upload initiate for 0x1000.
    assert_eq!(req[0], [0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]);
    assert_eq!(
        drive(&mut client, &mut server, &mut od, req).unwrap(),
        Some(Value::Uint(0x192))
    );
    assert!(client.is_idle());
}

#[test]
fn expedited_write_roundtrip() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od();
    // Golden frame for writing UNSIGNED16 500 to 0x1017.
    let req = client
        .write(Address::new(0x1017, 0), &[0xF4, 0x01])
        .unwrap();
    assert_eq!(req[0], [0x2B, 0x17, 0x10, 0x00, 0xF4, 0x01, 0x00, 0x00]);
    assert_eq!(drive(&mut client, &mut server, &mut od, req).unwrap(), None);
    assert_eq!(od.read(Address::new(0x1017, 0)).unwrap(), Value::Uint(500));
}

#[test]
fn segmented_string_write_then_read() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od();
    let name = "Hello, CANopen!"; // 15 bytes -> segmented both ways

    // Write.
    let data = name.as_bytes().to_vec();
    let req = client.write(Address::new(0x2200, 0), &data).unwrap();
    assert_eq!(drive(&mut client, &mut server, &mut od, req).unwrap(), None);
    assert_eq!(
        od.read(Address::new(0x2200, 0)).unwrap(),
        Value::Str(name.into())
    );

    // Read back.
    let req = client.read(Address::new(0x2200, 0), DataType::VisibleString);
    assert_eq!(
        drive(&mut client, &mut server, &mut od, req).unwrap(),
        Some(Value::Str(name.into()))
    );
}

#[test]
fn segmented_read_of_u64() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od();
    let addr = Address::new(0x2300, 0);
    od.insert(
        addr,
        Entry::new(
            "big",
            DataType::Unsigned64,
            AccessType::Ro,
            Storage::Value(Value::Uint(0xDEAD_BEEF_CAFE_F001)),
        ),
    );
    let req = client.read(addr, DataType::Unsigned64);
    assert_eq!(
        drive(&mut client, &mut server, &mut od, req).unwrap(),
        Some(Value::Uint(0xDEAD_BEEF_CAFE_F001))
    );
}

#[test]
fn read_missing_object_aborts() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od();
    let req = client.read(Address::new(0x5555, 0), DataType::Unsigned8);
    assert_eq!(
        drive(&mut client, &mut server, &mut od, req),
        Err(Error::SdoAborted(SdoAbortCode::ObjectDoesNotExist))
    );
    assert!(client.is_idle());
    assert!(!server.is_busy());
}

#[test]
fn missing_subindex_reports_subindex_code() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od(); // 0x2202 has only sub 5
    let req = client.read(Address::new(0x2202, 9), DataType::Unsigned16);
    assert_eq!(
        drive(&mut client, &mut server, &mut od, req),
        Err(Error::SdoAborted(SdoAbortCode::SubindexDoesNotExist))
    );
}

#[test]
fn write_to_read_only_aborts() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od();
    let req = client
        .write(Address::new(0x1000, 0), &[1, 2, 3, 4])
        .unwrap();
    assert_eq!(
        drive(&mut client, &mut server, &mut od, req),
        Err(Error::SdoAborted(SdoAbortCode::WriteOfReadOnlyObject))
    );
}

#[test]
fn read_of_write_only_aborts() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od();
    let req = client.read(Address::new(0x2201, 0), DataType::Unsigned32);
    assert_eq!(
        drive(&mut client, &mut server, &mut od, req),
        Err(Error::SdoAborted(SdoAbortCode::ReadOfWriteOnlyObject))
    );
}

#[test]
fn wrong_width_write_maps_to_length_abort() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od();
    // Writing 4 bytes into an Unsigned16 entry trips the width check,
    // reported as the generic type-mismatch code (D4 alignment with
    // python-canopen / CANopenNode practice).
    let req = client
        .write(Address::new(0x2202, 5), &[0x70, 0x11, 0x01, 0x00])
        .unwrap();
    assert_eq!(
        drive(&mut client, &mut server, &mut od, req),
        Err(Error::SdoAborted(SdoAbortCode::TypeMismatch))
    );
}

#[test]
fn width_mismatch_on_expedited_maps_to_type_mismatch() {
    use canopen_core::sdo::SdoAbortCode;

    // NOTE (docs/DESIGN-DECISIONS.md R10-P2-16 resolved): a *width-correct* value-range
    // violation is unreachable over SDO — `decode_le` always yields an
    // in-range value, and `encode_as`'s range branch is exercised directly
    // by datatypes unit tests. What the wire DOES surface for wrong-width
    // writes is the TypeMismatch abort asserted below.
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od();
    let addr = Address::new(0x2300, 0);
    od.insert(
        addr,
        Entry::new(
            "u16 cell",
            DataType::Unsigned16,
            AccessType::Rw,
            Storage::Value(Value::Uint(0)),
        ),
    );
    // 1-byte payload into Unsigned16: BadLength → TYPE_MISMATCH (0x06070010).
    let req = client.write(addr, &[0xAA]).unwrap();
    let mut aborted: Option<SdoAbortCode> = None;
    let mut queue: std::collections::VecDeque<_> = req.into();
    while let Some(frame) = queue.pop_front() {
        for resp in server.handle(&mut od, &frame) {
            match client.on_response(&resp) {
                Ok(SdoEvent::Send(f)) => queue.extend(f),
                Ok(_) => {}
                Err(Error::SdoAborted(code)) => aborted = Some(code),
                Err(e) => panic!("unexpected {e:?}"),
            }
        }
    }
    assert_eq!(aborted, Some(SdoAbortCode::TypeMismatch));
    assert_eq!(od.read(addr).unwrap(), Value::Uint(0));
}

#[test]
fn short_final_segment_against_declared_size_aborts() {
    let (mut _client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od();
    let addr = Address::new(0x2200, 0);
    // Hand-drive the server: declare 10 bytes, send one 3-byte final segment.
    use canopen_core::sdo::encode_data_segment;
    use canopen_core::sdo::encode_download_initiate_segmented;
    let init = encode_download_initiate_segmented(addr, 10);
    let ack = server.handle(&mut od, &init);
    assert_eq!(ack.first().map(|f| f[0] & 0xE0), Some(0x60)); // download initiate response
    let seg = encode_data_segment(b"abc", false, true).unwrap();
    let resp = server.handle(&mut od, &seg);
    assert!(!resp.is_empty(), "expected abort response");
    let (_, code) = canopen_core::sdo::decode_abort(&resp[0]).unwrap();
    assert_eq!(code, SdoAbortCode::TypeMismatchLengthLow);
}

#[test]
fn client_detects_toggle_violation() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od();
    let payload: Vec<u8> = (0..20u8).collect();
    let first = client.write(Address::new(0x2200, 0), &payload).unwrap();

    // Initiate ack -> client emits first segment with toggle=false.
    let acks = server.handle(&mut od, &first[0]);
    assert_eq!(acks.len(), 1);
    match client.on_response(&acks[0]).unwrap() {
        SdoEvent::Send(segs) => {
            assert_eq!(segs.len(), 1);
            assert_eq!(segs[0][0] & 0x10, 0);
        }
        other => panic!("expected Send, got {other:?}"),
    }
    // Feed an acknowledgement with the WRONG toggle bit.
    let bad_ack = encode_download_segment_response(true);
    assert_eq!(client.on_response(&bad_ack), Err(Error::ToggleBit));
}

#[test]
fn client_abort_now_sends_abort_frame_and_resets() {
    let mut client = SdoClient::new();
    let addr = Address::new(0x2000, 0);
    let _req = client.read(addr, DataType::Unsigned32);
    let abort = client.abort_now(SdoAbortCode::ProtocolTimeout).unwrap();
    assert_eq!(abort, encode_abort(addr, SdoAbortCode::ProtocolTimeout));
    assert!(client.is_idle());
    assert!(client.abort_now(SdoAbortCode::ProtocolTimeout).is_none());
}

#[test]
fn incoming_server_abort_resets_client() {
    let mut client = SdoClient::new();
    let addr = Address::new(0x1000, 0);
    let _req = client.read(addr, DataType::Unsigned32);
    let abort = encode_abort(addr, SdoAbortCode::HardwareError);
    assert_eq!(
        client.on_response(&abort),
        Err(Error::SdoAborted(SdoAbortCode::HardwareError))
    );
    assert!(client.is_idle());
}

#[test]
fn zero_length_upload_aborts() {
    // docs/DESIGN-DECISIONS.md 审-S2: a zero-length object cannot be uploaded at all —
    // CANopenNode aborts DEVICE_INCOMPAT 0x0604_0047 while loading the value
    // (`CO_SDOserver.c:557-563`), python-canopen aborts 0x0800_0024
    // (`sdo/server.py:68-72`). The server takes the CANopenNode code; the
    // client surfaces it as an abort instead of a bogus empty segmented
    // transfer.
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od(); // 0x2200 starts as an empty string
    let req = client.read(Address::new(0x2200, 0), DataType::VisibleString);
    assert_eq!(req.len(), 1);
    let responses = server.handle(&mut od, &req[0]);
    assert_eq!(responses.len(), 1);
    assert_eq!(
        responses[0][0], 0x80,
        "zero-length upload must abort, not start a transfer"
    );
    assert_eq!(
        u32::from_le_bytes([
            responses[0][4],
            responses[0][5],
            responses[0][6],
            responses[0][7]
        ]),
        0x0604_0047
    );
    assert!(!server.is_busy(), "no transfer may be armed");
    assert!(matches!(
        client.on_response(&responses[0]),
        Err(Error::SdoAborted(
            SdoAbortCode::GeneralInternalIncompatibility
        ))
    ));
}

// --- Block transfer ----------------------------------------------------------

use canopen_core::sdo::block::{
    decode_sub_segment, encode_download_initiate, BlockReceiver, BlockWriter,
};

fn bulk_od() -> ObjectDictionary {
    let mut od = test_od();
    od.insert(
        Address::new(0x2400, 0),
        Entry::new(
            "Bulk domain",
            DataType::Domain,
            AccessType::Rw,
            Storage::Value(Value::Bytes(Vec::new())),
        ),
    );
    od.insert(
        Address::new(0x2401, 0),
        Entry::new(
            "Firmware image",
            DataType::OctetString,
            AccessType::Ro,
            Storage::Value(Value::Bytes((0..900u16).map(|i| (i % 251) as u8).collect())),
        ),
    );
    od
}

#[test]
fn large_write_automatically_uses_block_transfer() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = bulk_od();
    let data: Vec<u8> = (0..1000u16).map(|i| (i % 251) as u8).collect();

    let req = client.write(Address::new(0x2400, 0), &data).unwrap();
    assert_eq!(req.len(), 1);
    assert_eq!(req[0][0], 0xC6); // block-download initiate with size + CRC

    assert_eq!(drive(&mut client, &mut server, &mut od, req).unwrap(), None);
    assert_eq!(
        od.read(Address::new(0x2400, 0)).unwrap(),
        Value::Bytes(data)
    );
    assert!(client.is_idle());
}

/// docs/DESIGN-DECISIONS.md SDO-29: an index that exists only as sub 1..N (no sub-0 template)
/// must still answer `SubindexDoesNotExist` (0x0609_0011) for a missing sub,
/// not `ObjectDoesNotExist`.
#[test]
fn missing_subindex_on_sub_only_index_reports_subindex_code() {
    use canopen_core::sdo::decode_abort;
    let mut od = test_od();
    od.insert(
        Address::new(0x2000, 1),
        Entry::new(
            "sub-only",
            DataType::Unsigned32,
            AccessType::Rw,
            Storage::Value(Value::Uint(1)),
        ),
    );
    let mut server = SdoServer::new();
    let out = server.handle(
        &mut od,
        &canopen_core::sdo::encode_upload_request(Address::new(0x2000, 5)),
    );
    let (addr, code) = decode_abort(&out[0]).expect("upload of a missing sub must abort");
    assert_eq!(addr, Address::new(0x2000, 5));
    assert_eq!(code, SdoAbortCode::SubindexDoesNotExist);
}

/// docs/DESIGN-DECISIONS.md SDO-34: once the *index* exists, a missing subentry is
/// `SUB_NOT_EXIST` — for sub 0 just as much as for any other subindex
/// (python `node/local.py:142-152`, `CO_ODinterface.c:218-232`). A sub-only
/// index (application-built record, EDS RECORD without a sub-0 section) used
/// to report `0x0602_0000` for `0x2000:0` while `0x2000:5` correctly
/// reported `0x0609_0011`.
#[test]
fn missing_sub_zero_on_sub_only_index_reports_subindex_code() {
    use canopen_core::sdo::decode_abort;
    let mut od = test_od();
    od.insert(
        Address::new(0x2000, 1),
        Entry::new(
            "sub-only",
            DataType::Unsigned32,
            AccessType::Rw,
            Storage::Value(Value::Uint(1)),
        ),
    );
    let mut server = SdoServer::new();
    for (sub, expected) in [
        (0u8, SdoAbortCode::SubindexDoesNotExist),
        (5, SdoAbortCode::SubindexDoesNotExist),
    ] {
        let out = server.handle(
            &mut od,
            &canopen_core::sdo::encode_upload_request(Address::new(0x2000, sub)),
        );
        let (addr, code) = decode_abort(&out[0]).expect("upload of a missing sub must abort");
        assert_eq!(addr, Address::new(0x2000, sub));
        assert_eq!(code, expected, "sub {sub}");
    }
    // A completely absent index still reports 0x0602_0000.
    let out = server.handle(
        &mut od,
        &canopen_core::sdo::encode_upload_request(Address::new(0x2001, 0)),
    );
    let (_, code) = decode_abort(&out[0]).expect("missing index must abort");
    assert_eq!(code, SdoAbortCode::ObjectDoesNotExist);
}

/// docs/DESIGN-DECISIONS.md SDO-36: an end frame whose `noData` count covers the **whole**
/// received block is malformed — CANopenNode compares `bufOffsetWr <= noData`
/// first and aborts `CO_SDO_AB_DEVICE_INCOMPAT` (0x0604_0047,
/// `CO_SDOserver.c:908-916`). The old `saturating_sub` folded `declared = 0`
/// plus a 7-byte padding segment into `0 == 0` and stored an empty value with
/// a `0xA1` end response.
#[test]
fn block_download_end_segment_that_is_all_padding_aborts() {
    use canopen_core::sdo::block::{encode_download_initiate, encode_end, encode_sub_segment};
    use canopen_core::sdo::decode_abort;
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    let init = encode_download_initiate(Address::new(0x2400, 0), Some(0), false);
    let resp = server.handle(&mut od, &init);
    assert!(resp.iter().all(|f| f[0] != 0x80), "initiate accepted");
    // One 7-byte segment that is entirely padding (noData = 7).
    let seg = encode_sub_segment(1, &[0xAA; 7], true).unwrap();
    let _ = server.handle(&mut od, &seg);
    let out = server.handle(&mut od, &encode_end(7, 0));
    let (addr, code) = decode_abort(&out[0]).expect("all-padding end must abort");
    assert_eq!(addr, Address::new(0x2400, 0));
    assert_eq!(
        code,
        SdoAbortCode::GeneralInternalIncompatibility,
        "0x0604_0047, like the reference"
    );
    assert!(!server.is_busy());

    // docs/DESIGN-DECISIONS.md SDO-36: the same guard must fire when the client **omits the
    // size indication** (`s=0`) — which is what both reference clients send
    // (`encode_end` here never sets the bit, python `sdo/client.py:806-809`
    // likewise). That arm used to `truncate(0)` and answer `0xA1`, silently
    // wiping the object.
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    let _ = server.handle(
        &mut od,
        &encode_download_initiate(Address::new(0x2400, 0), None, false),
    );
    assert!(server.is_busy());
    let _ = server.handle(&mut od, &seg);
    let out = server.handle(&mut od, &encode_end(7, 0));
    let (addr, code) = decode_abort(&out[0]).expect("all-padding end must abort");
    assert_eq!(addr, Address::new(0x2400, 0));
    assert_eq!(
        code,
        SdoAbortCode::GeneralInternalIncompatibility,
        "0x0604_0047 without the size bit too"
    );
    assert!(!server.is_busy());
    // A block whose last segment carries data plus a shorter padding tail
    // still succeeds (the guard only rejects an *all*-padding segment).
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    let init = encode_download_initiate(Address::new(0x2400, 0), Some(4), false);
    let resp = server.handle(&mut od, &init);
    assert!(resp.iter().all(|f| f[0] != 0x80), "initiate accepted");
    let seg = encode_sub_segment(1, &[0xAA; 7], true).unwrap();
    let _ = server.handle(&mut od, &seg);
    let out = server.handle(&mut od, &encode_end(3, 0));
    assert_eq!(out[0][0] & 0xE0, 0xA0, "0xA1 end response expected");
    assert_eq!(
        od.read(Address::new(0x2400, 0)).unwrap(),
        Value::Bytes(vec![0xAA; 4])
    );
}

/// docs/DESIGN-DECISIONS.md SDO-37: a **zero-length non-final** segment after the declared
/// length has been filled must be accepted (both references only compare the
/// total: `CO_SDOserver.c:815-826` computes `count = 7 - n` and aborts only on
/// `sizeTran > dataLength`; python `server.py:160-181` extends by `n`). The
/// old `buf.len() >= max` disjunct aborted such a segment with 0x0607_0012.
#[test]
fn segmented_download_accepts_empty_non_final_segment_after_fill() {
    use canopen_core::sdo::{decode_abort, encode_download_initiate_segmented};
    // Client download segment: `c` (last) in bit 0, `t` (toggle) in bit 4 and
    // `n` (unused bytes) in bits 1..3 — `NO_MORE_SEGMENTS = 0x01`,
    // `TOGGLE = 0x10` (`sdo/mod.rs:269-273`, `decode_data_segment:555-564`).
    let segment = |last: bool, toggle: bool, data: &[u8]| -> SdoPayload {
        let unused = (7 - data.len()) as u8;
        let mut p = [0u8; 8];
        p[0] = (unused << 1) | (u8::from(toggle) << 4) | u8::from(last);
        p[1..1 + data.len()].copy_from_slice(data);
        p
    };

    let mut server = SdoServer::new();
    let mut od = bulk_od();
    let init = encode_download_initiate_segmented(Address::new(0x2400, 0), 4);
    let _ = server.handle(&mut od, &init);
    assert!(server.is_busy(), "download armed");

    // Four bytes, non-final (toggle 0) → acknowledged.
    let out = server.handle(&mut od, &segment(false, false, &[1, 2, 3, 4]));
    assert_eq!(out[0][0] & 0xE0, 0x20, "segment ack expected");
    // An empty non-final segment (toggle 1) must still be acknowledged.
    let out = server.handle(&mut od, &segment(false, true, &[]));
    assert!(
        decode_abort(&out[0]).is_err(),
        "an empty non-final segment after the declared length must be accepted: {out:?}"
    );
    assert_eq!(out[0][0] & 0xE0, 0x20, "still a segment ack");
    assert!(server.is_busy(), "the session waits for the final segment");
    // The empty final segment ends the transfer with the full value.
    // The server confirms the last segment with a download-segment response
    // carrying the echoed toggle (`encode_download_segment_response`), like
    // CANopenNode's `DOWNLOAD_SEGMENT_RSP`.
    let out = server.handle(&mut od, &segment(true, false, &[]));
    assert_eq!(out[0][0] & 0xE0, 0x20, "download confirmed");
    assert!(!server.is_busy(), "the transfer is complete");
    assert_eq!(
        od.read(Address::new(0x2400, 0)).unwrap(),
        Value::Bytes(vec![1, 2, 3, 4])
    );

    // Over-long data is still rejected with the *high* length code.
    let mut server = SdoServer::new();
    let init = encode_download_initiate_segmented(Address::new(0x2400, 0), 2);
    let _ = server.handle(&mut od, &init);
    let out = server.handle(&mut od, &segment(false, false, &[1, 2, 3]));
    let (_, code) = decode_abort(&out[0]).expect("over-long payload must abort");
    assert_eq!(code, SdoAbortCode::TypeMismatchLengthHigh);
}

/// docs/DESIGN-DECISIONS.md SDO-39: `Node::set_sdo_timeout` takes an unbounded `u64`, so the
/// 0.7 × sub-block window must not be computed as `timeout_ms * 7 / 10`
/// (debug overflow panic / release wrap to a tiny window).
#[test]
fn sub_block_timeout_window_does_not_overflow() {
    use canopen_core::sdo::block::{encode_download_initiate, encode_sub_segment};
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    let init = encode_download_initiate(Address::new(0x2400, 0), Some(900), false);
    assert!(
        server
            .handle(&mut od, &init)
            .iter()
            .all(|f| f[0] & 0xE0 != 0x80),
        "the block-download initiate must be accepted"
    );
    let seg = encode_sub_segment(1, &[0xAA; 7], false).unwrap();
    let _ = server.handle(&mut od, &seg);
    assert!(server.is_busy());

    // A huge timeout must not panic and must not fire the sub-block window
    // at an absurdly small elapsed time.
    let huge = u64::MAX;
    assert!(server.on_timeout(0, huge).is_none(), "window not armed yet");
    assert!(
        server.on_timeout(1_000, huge).is_none(),
        "0.7x of u64::MAX is far beyond 1 s"
    );
    // The general window still aborts, and it does so at `timeout_ms`.
    let out = server.on_timeout(u64::MAX, huge);
    assert!(out.is_some(), "the 1xT window must still abort");
    let (_, code) = canopen_core::sdo::decode_abort(&out.expect("abort")).unwrap();
    assert_eq!(code, SdoAbortCode::ProtocolTimeout);
}

/// docs/DESIGN-DECISIONS.md SDO-30: reserved command bits 4..3 make a block-family frame
/// illegal (CANopenNode's `0xF9`/`0xFB` masks) — a `0xDE`/`0xD8` initiate
/// must not be accepted as a block download.
#[test]
fn block_initiate_with_reserved_bits_is_rejected() {
    use canopen_core::sdo::decode_abort;
    let mut od = bulk_od();
    let mut server = SdoServer::new();
    let mut frame = [0u8; 8];
    frame[0] = 0xC0 | 0x02 | 0x04 | 0x18; // 0xDE: CS_6 + size + CRC + bits 4..3
    frame[1] = 0x00;
    frame[2] = 0x24; // 0x2400
    frame[4..8].copy_from_slice(&4u32.to_le_bytes());
    let out = server.handle(&mut od, &frame);
    let (_, code) = decode_abort(&out[0]).expect("reserved bits must abort");
    assert_eq!(code, SdoAbortCode::InvalidCommandSpecifier);
    assert!(!server.is_busy());
}

/// docs/DESIGN-DECISIONS.md SDO-28: an undecodable command byte during an in-flight transfer
/// must name the *in-flight* object in its abort, not the garbage frame
/// header (the reference always answers with `SDO->index`).
#[test]
fn illegal_frame_abort_names_the_in_flight_transfer() {
    use canopen_core::sdo::{decode_abort, encode_download_initiate_segmented};
    let mut od = test_od();
    let mut server = SdoServer::new();
    let init = encode_download_initiate_segmented(Address::new(0x2200, 0), 20);
    assert_eq!(server.handle(&mut od, &init).len(), 1);

    // Unknown command specifier (CS = 0x60) carrying a bogus address.
    let mut stray = [0u8; 8];
    stray[0] = 0x60;
    stray[1] = 0xFF;
    stray[2] = 0xFF;
    stray[3] = 0x7F;
    let out = server.handle(&mut od, &stray);
    let (addr, code) = decode_abort(&out[0]).expect("illegal frame must abort");
    assert_eq!(code, SdoAbortCode::InvalidCommandSpecifier);
    assert_eq!(
        addr,
        Address::new(0x2200, 0),
        "abort names the torn-down transfer"
    );
    assert!(!server.is_busy());
}

/// docs/DESIGN-DECISIONS.md SDO-25: a block-download initiate that declares a size which
/// cannot match a fixed-width OD variable must be rejected *at the
/// initiate*, like the segmented path and CANopenNode
/// (`CO_SDOserver.c:880-892`) — not after the whole block has arrived.
#[test]
fn block_download_initiate_checks_declared_size_against_fixed_width() {
    use canopen_core::sdo::block::encode_download_initiate;
    use canopen_core::sdo::decode_abort;
    let mut server = SdoServer::new();
    let mut od = test_od();
    od.insert(
        Address::new(0x2000, 0),
        Entry::new(
            "u32",
            DataType::Unsigned32,
            AccessType::Rw,
            Storage::Value(Value::Uint(0)),
        ),
    );

    // Too long: 5 bytes declared for a 4-byte object.
    let long = server.handle(
        &mut od,
        &encode_download_initiate(Address::new(0x2000, 0), Some(5), false),
    );
    let (addr, code) = decode_abort(&long[0]).expect("initiate must abort");
    assert_eq!(addr, Address::new(0x2000, 0));
    assert_eq!(code, SdoAbortCode::TypeMismatchLengthHigh);
    assert!(!server.is_busy(), "no transfer is armed");

    // Too short: 2 bytes declared for a 4-byte object.
    let short = server.handle(
        &mut od,
        &encode_download_initiate(Address::new(0x2000, 0), Some(2), false),
    );
    let (_, code) = decode_abort(&short[0]).expect("initiate must abort");
    assert_eq!(code, SdoAbortCode::TypeMismatchLengthLow);

    // Exact size still enters the block transfer.
    let exact = server.handle(
        &mut od,
        &encode_download_initiate(Address::new(0x2000, 0), Some(4), false),
    );
    assert_ne!(exact[0][0], 0x80, "exact size must start the transfer");
    assert!(server.is_busy());
}

#[test]
fn block_download_without_size_declared_is_capped() {
    use canopen_core::sdo::block::{encode_download_initiate, encode_sub_segment};
    use canopen_core::sdo::decode_abort;
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    // No size declared (s bit clear): the receiver must still be capped at
    // MAX_VALUE_LEN instead of growing without bound (docs/DESIGN-DECISIONS.md #2).
    let init = encode_download_initiate(Address::new(0x2400, 0), None, false);
    let resp = server.handle(&mut od, &init);
    assert!(resp.iter().all(|f| f[0] != 0x80)); // initiate response, not abort

    // Stream 7-byte segments; seqno wraps 1..=127 (blksize 127). Once the
    // buffer would exceed MAX_VALUE_LEN the server must abort.
    let max_segs = canopen_core::datatypes::MAX_VALUE_LEN / 7 + 2;
    let mut aborted = None;
    for i in 1..=max_segs {
        let seqno = ((i - 1) % 127 + 1) as u8;
        let seg = encode_sub_segment(seqno, &[0xAA; 7], false).unwrap();
        let out = server.handle(&mut od, &seg);
        if let Some(f) = out.iter().find(|f| f[0] == 0x80) {
            aborted = Some(decode_abort(f).unwrap().1);
            break;
        }
    }
    assert_eq!(aborted, Some(SdoAbortCode::TypeMismatchLengthHigh));
    // And the wedged transfer is released for the next transaction.
    assert!(!server.is_busy());
}

#[test]
fn download_initiate_without_e_or_s_opens_undeclared_transfer() {
    // CiA 301 §7.2.4.3.2 / CANopenNode `CO_SDOserver.c:789-793`: e=0,s=0 is a
    // legal segmented start whose total size is simply not indicated. The
    // transfer must open and be bounded by the absolute value cap, not be
    // rejected with InvalidCommandSpecifier.
    use canopen_core::sdo::encode_data_segment;
    let mut server = SdoServer::new();
    let mut od = test_od();
    // byte0 = CCS_DOWNLOAD_INITIATE (0x20): neither 'e' (0x02) nor 's' (0x01).
    let req: [u8; 8] = [0x20, 0x00, 0x22, 0x00, 0, 0, 0, 0];
    let out = server.handle(&mut od, &req);
    assert_eq!(out.len(), 1);
    assert_eq!(out[0][0], 0x60, "segmented initiate must be confirmed");
    assert!(server.is_busy(), "an undeclared transfer must be in flight");
    // A single final segment completes the write.
    let seg = encode_data_segment(b"AB", false, true).unwrap();
    let out = server.handle(&mut od, &seg);
    assert_eq!(out[0][0], 0x20, "final segment acknowledged with toggle 0");
    assert!(!server.is_busy());
    assert_eq!(
        od.read(Address::new(0x2200, 0)).unwrap(),
        Value::Str("AB".to_string())
    );
}

#[test]
fn block_read_roundtrips_bulk_value() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = bulk_od();
    let expected: Vec<u8> = (0..900u16).map(|i| (i % 251) as u8).collect();

    let req = client.read_block(Address::new(0x2401, 0), DataType::OctetString);
    // Block-upload initiate with blksize proposal + CRC support (0xA4).
    assert_eq!(req[0][0], 0xA4);

    match drive(&mut client, &mut server, &mut od, req).unwrap() {
        Some(Value::Bytes(bytes)) => {
            assert_eq!(bytes.len(), 900);
            assert_eq!(bytes, expected);
        }
        other => panic!("expected bulk read, got {other:?}"),
    }
}

#[test]
fn segmented_write_below_threshold_stays_segmented() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = bulk_od();
    // Exactly at threshold: still segmented.
    let data = vec![0xEE; canopen_core::sdo::BLOCK_THRESHOLD];
    let req = client.write(Address::new(0x2400, 0), &data).unwrap();
    assert_eq!(req[0][0], 0x21); // segmented initiate
    assert_eq!(drive(&mut client, &mut server, &mut od, req).unwrap(), None);
}

/// Hand-drives a block download and corrupts one segment payload, expecting
/// the server to reject the transfer with `CrcError`.
#[test]
fn corrupted_segment_fails_crc_check() {
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    let addr = Address::new(0x2400, 0);
    let data: Vec<u8> = (0..50u8).collect();

    // Initiate.
    let init = encode_download_initiate(addr, Some(data.len() as u32), true);
    let responses = server.handle(&mut od, &init);
    assert_eq!(responses.len(), 1);
    assert_eq!(responses[0][0] & 0xE0, 0xA0);

    // Stream the single sub-block through a writer, corrupting segment 2.
    let mut writer = BlockWriter::new(data.clone(), canopen_core::sdo::block::MAX_BLKSIZE);
    while let Some(mut frame) = writer.next_segment() {
        if decode_sub_segment(&frame).seqno == 2 {
            frame[3] ^= 0xFF; // corrupt payload
        }
        let out = server.handle(&mut od, &frame);
        if !out.is_empty() && writer.is_done() {
            // Sub-response after final segment; now send the end frame.
            let end = writer.end_frame(true);
            let out = server.handle(&mut od, &end);
            assert_eq!(out.len(), 1);
            assert_eq!(
                canopen_core::sdo::decode_abort(&out[0]).unwrap().1,
                SdoAbortCode::CrcError
            );
            return;
        }
        if writer.is_done() {
            let end = writer.end_frame(true);
            let out = server.handle(&mut od, &end);
            assert_eq!(
                canopen_core::sdo::decode_abort(&out[0]).unwrap().1,
                SdoAbortCode::CrcError
            );
            return;
        }
    }
    panic!("writer never finished");
}

/// Verifies the block receiver enforces the declared size bound: growth
/// beyond it aborts with `BadLength`, while a fit (and an undeclared size)
/// is accepted.
#[test]
fn block_receiver_rejects_length_overflow() {
    use canopen_core::types::Error;
    let mut receiver = BlockReceiver::new();
    let mut raw = [0u8; 8];
    raw[0] = 1;
    let seg = decode_sub_segment(&raw); // 7 data bytes, non-final
                                        // A declared bound below the segment size must be rejected.
    let err = receiver.push(&seg, Some(3)).unwrap_err();
    assert!(matches!(err, Error::BadLength { declared: 3, .. }));
    // A bound that fits is accepted.
    let mut receiver2 = BlockReceiver::new();
    receiver2.push(&seg, Some(7)).unwrap();
    // declared None → unbounded push is fine (the caller caps it).
    receiver2.push(&seg, None).unwrap();
}

/// §5.5 / docs/DESIGN-DECISIONS.md SDO-01: a fresh initiate while another transfer is active
/// is answered with **only** the stale-transfer abort
/// (`InvalidCommandSpecifier` 0x0504_0001, old address, docs/DESIGN-DECISIONS.md SDO-27) — the
/// new request is not serviced in the same step (CANopenNode
/// `CO_SDOserver.c:803-841,1025-1058`). The next initiate on the now-idle
/// channel succeeds.
#[test]
fn new_initiate_aborts_stale_transfer_first() {
    use canopen_core::sdo::{
        decode_abort, decode_download_response, encode_download_expedited,
        encode_download_initiate_segmented, SdoAbortCode,
    };

    let mut od = test_od();
    let mut server = SdoServer::new();

    // Start a segmented download at 0x2200 (20 bytes) -> transfer active.
    let init = encode_download_initiate_segmented(Address::new(0x2200, 0), 20);
    let out = server.handle(&mut od, &init);
    assert_eq!(out.len(), 1);
    assert_eq!(decode_download_response(&out[0]).unwrap().index, 0x2200);
    assert!(server.is_busy());

    // A new expedited write arrives mid-transfer.
    let write = encode_download_expedited(Address::new(0x2201, 0), &[1, 2, 3, 4]).unwrap();
    let out = server.handle(&mut od, &write);

    // Only the InvalidCommandSpecifier abort addressed at the STALE
    // transfer (docs/DESIGN-DECISIONS.md SDO-27: the reference answers CO_SDO_AB_CMD here).
    assert_eq!(
        out.len(),
        1,
        "stale abort only — no service of the new request"
    );
    let (addr, code) = decode_abort(&out[0]).unwrap();
    assert_eq!(addr.index, 0x2200);
    assert_eq!(code, SdoAbortCode::InvalidCommandSpecifier);
    assert!(!server.is_busy());

    // The rejected request left no side effect behind.
    assert!(server.take_last_written().is_empty());

    // A clean retry on the idle channel is serviced normally.
    let out = server.handle(&mut od, &write);
    assert_eq!(out.len(), 1);
    assert_eq!(decode_download_response(&out[0]).unwrap().index, 0x2201);
    let drained = server.take_last_written();
    assert_eq!(drained.len(), 1);
    assert_eq!(drained[0].0.index, 0x2201);
}

/// docs/DESIGN-DECISIONS.md SDO-02: a callback entry whose `read` returns a value of a
/// different kind than declared aborts the upload with the **generic**
/// type-mismatch code 0x0607_0010, not the "length too short" variant 0x0607_0013.
#[test]
fn callback_type_mismatch_upload_aborts_with_generic_code() {
    use canopen_core::object_dictionary::ObjectAccess;
    use canopen_core::sdo::{decode_abort, encode_upload_request};

    struct Wrong;
    impl ObjectAccess for Wrong {
        fn read(&self, _addr: Address) -> canopen_core::types::Result<Value> {
            Ok(Value::Str("not a u32".into()))
        }
        fn write(&self, _addr: Address, _v: Value) -> canopen_core::types::Result<()> {
            Ok(())
        }
    }

    let mut od = test_od();
    od.insert(
        Address::new(0x2203, 0),
        Entry::new(
            "Wrong-typed callback",
            DataType::Unsigned32,
            AccessType::Ro,
            Storage::Callback(std::sync::Arc::new(Wrong)),
        ),
    );
    let mut server = SdoServer::new();
    let out = server.handle(&mut od, &encode_upload_request(Address::new(0x2203, 0)));
    assert_eq!(out.len(), 1);
    let (addr, code) = decode_abort(&out[0]).unwrap();
    assert_eq!(addr.index, 0x2203);
    assert_eq!(code, SdoAbortCode::TypeMismatch);
}

/// take_last_written drains every recorded write in order (docs/DESIGN-DECISIONS.md P2-4).
#[test]
fn last_written_drains_all_entries_in_order() {
    use canopen_core::sdo::encode_download_expedited;

    let mut od = test_od();
    let mut server = SdoServer::new();
    for v in [[1u8, 0], [3, 0], [7, 0]] {
        // 0x2202:5 is UNSIGNED16 — payloads must be 2 bytes.
        let f = encode_download_expedited(Address::new(0x2202, 5), &v).unwrap();
        server.handle(&mut od, &f);
    }
    let drained = server.take_last_written();
    assert_eq!(drained.len(), 3);
    assert_eq!(drained[0].1, Value::Uint(1));
    assert_eq!(drained[2].1, Value::Uint(7));
    assert!(server.take_last_written().is_empty());
}

// --- docs/DESIGN-DECISIONS.md P1-1 / P1-2 regressions -----------------------------------------

/// docs/DESIGN-DECISIONS.md P1-1: sizes whose final sub-block segment number collides with the
/// end-frame wire form (`0x80 | seqno` == `0xC1` for seqno ≡ 1 mod 4,
/// seqno >= 65) must round-trip intact in both directions.
#[test]
fn block_transfer_survives_end_frame_collision_sizes() {
    // ceil(len/7) ∈ {65,69,73} → final segment bytes 0xC1/0xC5/0xC9.
    for len in [455_usize, 483, 651] {
        let data: Vec<u8> = (0..len as u16).map(|i| (i % 251 + 1) as u8).collect();

        // Server direction: an explicit block download must land the full
        // value (no silent truncation, no spurious abort).
        let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
        let mut od = bulk_od();
        let req = client.write_block(Address::new(0x2400, 0), &data).unwrap();
        drive(&mut client, &mut server, &mut od, req).unwrap();
        assert_eq!(
            od.read(Address::new(0x2400, 0)).unwrap(),
            Value::Bytes(data.clone()),
            "server-side write of len={len}"
        );

        // Client direction: a block upload must return the full value
        // (the final segment must not be mistaken for the end frame).
        let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
        let mut od = bulk_od();
        od.insert(
            Address::new(0x2402, 0),
            Entry::new(
                "Variable blob",
                DataType::OctetString,
                AccessType::Ro,
                Storage::Value(Value::Bytes(data.clone())),
            ),
        );
        let req = client.read_block(Address::new(0x2402, 0), DataType::OctetString);
        match drive(&mut client, &mut server, &mut od, req).unwrap() {
            Some(Value::Bytes(bytes)) => assert_eq!(bytes, data, "client-side read len={len}"),
            other => panic!("expected bulk read of len={len}, got {other:?}"),
        }
    }
}

/// docs/DESIGN-DECISIONS.md SDO-01: after a segmented upload completes, the server must return
/// to idle — the *next* transaction on the same channel must succeed without
/// receiving the stale-transfer abort (`InvalidCommandSpecifier`) first.
#[test]
fn consecutive_transactions_after_segmented_read() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od();

    // First transaction: segmented read (>4 bytes).
    let long = "hello, segmented world!".to_string();
    od.insert(
        Address::new(0x2200, 0),
        Entry::new(
            "Device name",
            DataType::VisibleString,
            AccessType::Rw,
            Storage::Value(Value::Str(long.clone())),
        ),
    );
    let req = client.read(Address::new(0x2200, 0), DataType::VisibleString);
    match drive(&mut client, &mut server, &mut od, req).unwrap() {
        Some(Value::Str(s)) => assert_eq!(s, long),
        other => panic!("expected string, got {other:?}"),
    }

    // The zombie-transfer symptom: the server stayed busy forever.
    assert!(!server.is_busy());

    // Second transaction on the same channel: previously this received the
    // stale ABORT first and failed once.
    let req = client.read(Address::new(0x1000, 0), DataType::Unsigned32);
    assert_eq!(
        drive(&mut client, &mut server, &mut od, req).unwrap(),
        Some(Value::Uint(0x0000_0192))
    );
}

/// docs/DESIGN-DECISIONS.md 审-S2: a zero-length object read via block upload aborts like the
/// ordinary upload (CANopenNode rejects it while loading the value,
/// `CO_SDOserver.c:557-563`) instead of arming an empty block stream.
#[test]
fn block_read_of_empty_object_aborts() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = bulk_od(); // 0x2400 starts as empty Domain
    let req = client.read_block(Address::new(0x2400, 0), DataType::Domain);
    assert!(matches!(
        drive(&mut client, &mut server, &mut od, req),
        Err(Error::SdoAborted(
            SdoAbortCode::GeneralInternalIncompatibility
        ))
    ));
    assert!(!server.is_busy(), "no transfer may be armed");
}

/// docs/DESIGN-DECISIONS.md P3-26: a block-upload initiate with blksize=0 must be rejected
/// with the dedicated InvalidBlockSize abort (not silently clamped to 1).
#[test]
fn block_upload_initiate_rejects_zero_blksize() {
    use canopen_core::sdo::block::encode_upload_initiate;
    use canopen_core::sdo::decode_abort;
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    // blksize byte = 0, CRC requested.
    let frame = encode_upload_initiate(Address::new(0x2401, 0), 0, true);
    let out = server.handle(&mut od, &frame);
    assert_eq!(out.len(), 1);
    let (addr, code) = decode_abort(&out[0]).unwrap();
    assert_eq!(addr.index, 0x2401);
    assert_eq!(code, SdoAbortCode::InvalidBlockSize);
}

/// docs/DESIGN-DECISIONS.md 审-S1: a block-upload initiate whose pst (byte 5) is not smaller
/// than the object size switches to the ordinary upload response — expedited
/// for `1..=4` bytes (CANopenNode `CO_SDOserver.c:938-942` then `:1173-1181`).
/// The previous `size > 4` lower bound kept small objects in block mode.
#[test]
fn block_upload_pst_switches_small_object_to_expedited() {
    use canopen_core::sdo::block::encode_upload_initiate;
    use canopen_core::sdo::decode_upload_expedited_response;
    let mut server = SdoServer::new();
    let mut od = test_od(); // 0x1000:0 = Unsigned32 0x192 (4 bytes)
    let mut frame = encode_upload_initiate(Address::new(0x1000, 0), 1, false);
    frame[5] = 4; // pst == size → switch
    let out = server.handle(&mut od, &frame);
    assert_eq!(out.len(), 1);
    assert_eq!(out[0][0] & 0xE0, 0x40, "ordinary upload initiate response");
    let up = decode_upload_expedited_response(&out[0]).unwrap();
    assert_eq!(up.addr, Address::new(0x1000, 0));
    assert_eq!(up.data, &0x192u32.to_le_bytes());
    assert!(!server.is_busy(), "no transfer may be armed");
}

/// docs/DESIGN-DECISIONS.md 审-S1: for an object larger than 4 bytes the pst switch answers
/// with a segmented initiate response (`0x41` + size, CANopenNode
/// `CO_SDOserver.c:1184-1189`); the client then polls segments.
#[test]
fn block_upload_pst_switches_large_object_to_segmented() {
    use canopen_core::sdo::block::encode_upload_initiate;
    use canopen_core::sdo::decode_upload_initiate_response;
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    // A 10-byte object small enough for a u8 pst to reach it.
    od.insert(
        Address::new(0x2403, 0),
        Entry::new(
            "Medium blob",
            DataType::OctetString,
            AccessType::Ro,
            Storage::Value(Value::Bytes(vec![0x5A; 10])),
        ),
    );
    let mut frame = encode_upload_initiate(Address::new(0x2403, 0), 127, false);
    frame[5] = 127; // pst >= size → switch
    let out = server.handle(&mut od, &frame);
    assert_eq!(out.len(), 1);
    assert_eq!(out[0][0], 0x41, "segmented initiate response with size");
    let (addr, size) = decode_upload_initiate_response(&out[0]).unwrap();
    assert_eq!(addr, Address::new(0x2403, 0));
    assert_eq!(size, Some(10));
    assert!(server.is_busy(), "segmented upload armed");
}

/// docs/DESIGN-DECISIONS.md 审-S1: the reference validates blksize only when it actually
/// enters block mode (`CO_SDOserver.c:938-958`); a pst switch must win over
/// an out-of-range blksize byte.
#[test]
fn block_upload_pst_switch_ignores_invalid_blksize() {
    use canopen_core::sdo::block::encode_upload_initiate;
    let mut server = SdoServer::new();
    let mut od = test_od();
    let mut frame = encode_upload_initiate(Address::new(0x1000, 0), 0, false);
    frame[5] = 4; // pst triggers the switch
    let out = server.handle(&mut od, &frame);
    assert_eq!(out[0][0] & 0xE0, 0x40, "switch, not InvalidBlockSize");
}

/// docs/DESIGN-DECISIONS.md 审-S3: a lapsed block-download sub-block window is answered with
/// a partial ACK (CANopenNode `CO_SDOserver.c:1092-1101,1320-1361`) instead
/// of tearing the transfer down; a second silent window aborts with
/// `ProtocolTimeout`, mirroring the reference where the partial ACK does not
/// reset the general timeout (`:1360`).
#[test]
fn block_download_timeout_sends_partial_ack_once() {
    use canopen_core::sdo::block::{
        decode_sub_response, encode_download_initiate, encode_sub_segment,
    };
    use canopen_core::sdo::decode_abort;
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    let init = encode_download_initiate(Address::new(0x2400, 0), Some(100), false);
    let _ = server.handle(&mut od, &init);
    // Two segments arrive, then the client goes silent.
    let _ = server.handle(&mut od, &encode_sub_segment(1, &[0x11; 7], false).unwrap());
    let _ = server.handle(&mut od, &encode_sub_segment(2, &[0x22; 7], false).unwrap());

    // First observation arms the inactivity window (T = 1500 ms).
    assert!(server.on_timeout(1_000, 1_500).is_none());
    // docs/DESIGN-DECISIONS.md SDO-26: the sub-block window is 0.7×T, not 1×T — nothing is
    // sent just before it lapses.
    assert!(
        server.on_timeout(2_049, 1_500).is_none(),
        "0.7×T not reached"
    );
    // 0.7×T: partial ACK of the two received segments, transfer stays alive.
    let ack = server.on_timeout(2_050, 1_500).expect("partial ACK");
    let (ackseq, _blksize) = decode_sub_response(&ack).unwrap();
    assert_eq!(ackseq, 2, "ack reports the segments received so far");
    assert!(server.is_busy(), "transfer survives the partial ACK");

    // The peer retransmits inside the reference's 0.3×T reaction window
    // (a driver feeds the frame and stamps activity together).
    let _ = server.handle(&mut od, &encode_sub_segment(1, &[0x11; 7], false).unwrap());
    server.touch(2_600);
    // Re-armed window: again nothing at 0.7×T−1, a fresh ACK at 0.7×T.
    assert!(server.on_timeout(3_649, 1_500).is_none());
    assert!(
        server.on_timeout(3_650, 1_500).is_some(),
        "second silent episode ACKs again"
    );
    // No further data: 1×T after the re-arm aborts with ProtocolTimeout.
    let abort = server.on_timeout(4_100, 1_500).expect("abort at 1×T");
    let (addr, code) = decode_abort(&abort).unwrap();
    assert_eq!(addr, Address::new(0x2400, 0));
    assert_eq!(code, SdoAbortCode::ProtocolTimeout);
    assert!(!server.is_busy());
}

/// docs/DESIGN-DECISIONS.md 审-S4 / SDO-04: after the block-upload END frame is sent the server
/// keeps the END phase open (CANopenNode `UPLOAD_BLK_END_CRSP`): the client's
/// `0xA1` completes it, and any *other* frame is illegal — the reference's
/// `default → CO_SDO_AB_CMD` (`CO_SDOserver.c:1036-1058`) aborts instead of
/// stalling until the 1500 ms timeout. A truly idle `0xA1` is likewise an
/// illegal command.
#[test]
fn block_upload_end_phase_waits_for_client_ack() {
    let mut client = SdoClient::new();
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    let req = client.read_block(Address::new(0x2401, 0), DataType::OctetString);
    let mut queue: std::collections::VecDeque<SdoPayload> = req.into();
    let mut end_ack: Option<SdoPayload> = None;
    while let Some(frame) = queue.pop_front() {
        for response in server.handle(&mut od, &frame) {
            match client.on_response(&response).unwrap() {
                SdoEvent::Send(frames) => {
                    queue.extend(frames);
                }
                SdoEvent::Complete(_, trailing) => {
                    for f in trailing {
                        if f[0] == 0xA1 {
                            end_ack = Some(f);
                        } else {
                            queue.push_back(f);
                        }
                    }
                }
                SdoEvent::Done => {}
            }
        }
    }
    assert!(
        server.is_busy(),
        "the server stays in its END phase until the client acknowledges"
    );

    // A stray frame (here a fresh upload initiate) is illegal while the END
    // phase is open: the reference's END_CRSP state falls into the receive
    // switch's `default → CO_SDO_AB_CMD` (`CO_SDOserver.c:1036-1058`), so it
    // aborts with the in-flight object's address instead of stalling until
    // the 1500 ms timeout (docs/DESIGN-DECISIONS.md SDO-04).
    let stray: SdoPayload = [0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0];
    let out = server.handle(&mut od, &stray);
    assert_eq!(out.len(), 1, "a stray END-phase frame must abort");
    assert_eq!(out[0][0], 0x80);
    assert_eq!(
        u16::from_le_bytes([out[0][1], out[0][2]]),
        0x2401,
        "the abort names the in-flight object, not the stray frame's bits"
    );
    assert_eq!(out[0][4..8], 0x0504_0001u32.to_le_bytes());
    assert!(!server.is_busy(), "the abort tears the phase down");

    // The client had emitted its own end response before the stray frame.
    let _ = end_ack.expect("client emitted its end response");

    // Unsolicited (or duplicate) 0xA1 while idle: illegal command.
    let out = server.handle(&mut od, &[0xA1, 0, 0, 0, 0, 0, 0, 0]);
    assert_eq!(out.len(), 1);
    assert_eq!(out[0][0], 0x80, "abort frame");
    assert_eq!(out[0][4..8], 0x0504_0001u32.to_le_bytes());
}

/// docs/DESIGN-DECISIONS.md P2-7: a gap in sub-block sequence numbers ends the sub-block with
/// an ack of what was received so the sender retransmits from there — the
/// transfer completes instead of aborting outright.
#[test]
fn block_download_survives_sequence_gap() {
    use canopen_core::sdo::block::{
        decode_sub_response, encode_download_initiate, encode_sub_segment,
    };

    let data: Vec<u8> = (0..100u16).map(|i| (i % 251) as u8).collect();
    let mut server = SdoServer::new();
    let mut od = bulk_od();

    // Initiate a block download by hand so we control segment injection.
    let init = encode_download_initiate(Address::new(0x2400, 0), Some(data.len() as u32), true);
    let resp = server.handle(&mut od, &init);
    assert_eq!(resp.len(), 1); // ack carrying the negotiated blksize

    // First segment in order: silently accepted.
    let s1 = encode_sub_segment(1, &data[0..7], false).unwrap();
    assert!(server.handle(&mut od, &s1).is_empty());

    // Duplicate of seqno 1 (e.g. our ack was lost): ignored, not an abort.
    assert!(server.handle(&mut od, &s1).is_empty());

    // Gap (skip 2..=4, jump to 5): ends the sub-block with an early ack of
    // the one good segment so the sender retransmits from seqno 2.
    let s5 = encode_sub_segment(5, &data[28..35], false).unwrap();
    let ack = server.handle(&mut od, &s5);
    assert_eq!(ack.len(), 1, "gap triggers early sub-block ack");
    let (acked, _) = decode_sub_response(&ack[0]).unwrap();
    assert_eq!(acked, 1);

    // Retransmission resumes at 1-based numbering for the rewound window.
    let s_re = encode_sub_segment(1, &data[7..14], false).unwrap();
    assert!(
        server.handle(&mut od, &s_re).is_empty(),
        "retransmit accepted"
    );
}

/// docs/DESIGN-DECISIONS.md P1-1 (see git history): after a gap the client acks what it has and the
/// server rewinds with sequence numbers restarting at 1 — the client must
/// reset its expectation accordingly or both sides never resync and the
/// transaction dies at the host deadline.
#[test]
fn block_upload_gap_recovery_renumbers_from_one() {
    use canopen_core::sdo::block::{decode_sub_response, decode_sub_segment};

    let data: Vec<u8> = (0..30u16).map(|i| (i % 251 + 1) as u8).collect();
    let mut client = SdoClient::new();
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    od.insert(
        Address::new(0x2402, 0),
        Entry::new(
            "Variable blob",
            DataType::OctetString,
            AccessType::Ro,
            Storage::Value(Value::Bytes(data.clone())),
        ),
    );

    // Initiate: client request → server response → start.
    let req = client.read_block(Address::new(0x2402, 0), DataType::OctetString);
    for resp in server.handle(&mut od, &req[0]) {
        match client.on_response(&resp).unwrap() {
            SdoEvent::Send(frames) => {
                // The start frame triggers the first sub-block stream.
                let mut stream = Vec::new();
                for f in frames {
                    stream.extend(server.handle(&mut od, &f));
                }
                assert!(stream.len() >= 5, "30 bytes need 5 segments");
            }
            other => panic!("unexpected {other:?}"),
        }
    }

    // Server streams segments; we deliver only 1 and 2 to the client, drop 3,
    // then deliver 4 — the client must ack exactly 2 received segments.
    // Rebuild the stream deterministically from the codec instead of keeping
    // server output order-dependent.
    let segs: Vec<_> = (0..4)
        .map(|i| {
            let start = i * 7;
            let end = ((i + 1) * 7).min(data.len());
            canopen_core::sdo::block::encode_sub_segment((i + 1) as u8, &data[start..end], false)
                .unwrap()
        })
        .collect();

    for frame in segs.iter().take(2) {
        match client.on_response(frame).unwrap() {
            SdoEvent::Send(f) => assert!(f.is_empty(), "no ack mid-window"),
            other => panic!("unexpected {other:?}"),
        }
    }
    // Gap: segment 3 lost, segment 4 arrives out of order.
    let acks = match client.on_response(&segs[3]).unwrap() {
        SdoEvent::Send(f) => f,
        other => panic!("expected partial ack, got {other:?}"),
    };
    assert_eq!(acks.len(), 1);
    let (acked, _) = decode_sub_response(&acks[0]).unwrap();
    assert_eq!(acked, 2);

    // Feed the partial ack to the server: it must rewind and re-emit with
    // seqno restarting at 1, carrying data from offset 14 onward.
    let rewind = server.handle(&mut od, &acks[0]);
    assert!(!rewind.is_empty(), "server re-emits after partial ack");
    let first_re = decode_sub_segment(&rewind[0]);
    assert_eq!(first_re.seqno, 1, "rewind renumbers from 1");
    assert_eq!(
        &rewind[0][1..8],
        &data[14..21],
        "resumes after acked offset"
    );

    // The regression: the client ACCEPTS the renumbered segment (previously
    // it rejected every retransmitted frame, deadlocking until timeout).
    match client.on_response(&rewind[0]).unwrap() {
        SdoEvent::Send(f) => assert!(f.is_empty()),
        other => panic!("renumbered retransmission rejected: {other:?}"),
    }

    // Stream the rest to completion and verify integrity end-to-end.
    let mut collected = data[..14].to_vec();
    collected.extend_from_slice(&data[14..21]);
    let mut queue: std::collections::VecDeque<_> = rewind.iter().skip(1).cloned().collect();
    while let Some(frame) = queue.pop_front() {
        let seg = decode_sub_segment(&frame);
        collected.extend_from_slice(seg.data);
        match client.on_response(&frame).unwrap() {
            SdoEvent::Send(f) => {
                for ack_frame in f {
                    for more in server.handle(&mut od, &ack_frame) {
                        // The end frame arrives here on completion.
                        if let SdoEvent::Complete(value, _) = client.on_response(&more).unwrap() {
                            assert_eq!(value, Value::Bytes(data.clone()));
                            return;
                        }
                    }
                }
            }
            SdoEvent::Complete(value, _) => {
                assert_eq!(value, Value::Bytes(data.clone()));
                return;
            }
            _ => {}
        }
    }
    panic!("block upload did not complete after gap recovery");
}

/// docs/DESIGN-DECISIONS.md R10-P2-4 (见 git 历史): a zero-length value must be writable via the
/// segmented path — initiate(size=0) followed by an empty final segment
/// (`c=1`, no data), mirroring python-canopen's `WritableStream` for empty
/// values. Previously rejected outright while the server accepts it.
#[test]
fn zero_length_write_roundtrips_via_segmented() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od(); // 0x2200 starts as an empty string

    let req = client.write(Address::new(0x2200, 0), &[]).unwrap();
    // Initiate carries size=0 with the segmented command specifier.
    assert_eq!(req[0][0] & 0xE0, 0x20);
    assert_eq!(&req[0][4..8], &[0, 0, 0, 0]);

    // Initiate response → client emits the empty final segment.
    let resp = server.handle(&mut od, &req[0]);
    assert_eq!(resp.len(), 1);
    let seg = match client.on_response(&resp[0]).unwrap() {
        SdoEvent::Send(f) => f[0],
        other => panic!("expected empty final segment, got {other:?}"),
    };
    // c=1, n=(7-0)<<1 → 0x0F for toggle 0; no data bytes.
    assert_eq!(seg[0], 0x0F);
    assert_eq!(&seg[1..], &[0; 7]);

    // Server acknowledges; the transaction completes and stores emptiness.
    let ack = server.handle(&mut od, &seg);
    assert_eq!(ack.len(), 1);
    match client.on_response(&ack[0]).unwrap() {
        SdoEvent::Done => {}
        other => panic!("expected Done, got {other:?}"),
    }
    assert_eq!(
        od.read(Address::new(0x2200, 0)).unwrap(),
        Value::Str(String::new())
    );
    assert!(client.is_idle());
}

/// docs/DESIGN-DECISIONS.md 八M7：在途分段传输收到非法 CS=5 帧（既非 start、非 sub-response、
/// 也非 block-upload initiate）时，服务器必须在发 abort 的同时清除会话；
/// 此前 transfer 保持武装，后续合法分段轮询仍被服务（僵尸会话）。
#[test]
fn invalid_cs5_during_segmented_transfer_clears_session() {
    use canopen_core::sdo::{decode_abort, encode_download_initiate_segmented};
    let mut server = SdoServer::new();
    let mut od = test_od();

    // 启动一次 10 字节分段下载（0x2200:0 VisibleString）。
    let init = encode_download_initiate_segmented(Address::new(0x2200, 0), 10);
    let out = server.handle(&mut od, &init);
    assert!(
        !out.iter().any(|f| f[0] == 0x80),
        "segmented initiate must be accepted"
    );
    assert!(server.is_busy(), "transfer armed");

    // 0xA2（CS=5 + sub-response）在分段阶段非法。
    let bad: SdoPayload = [0xA2, 0, 0, 0, 0, 0, 0, 0];
    let out = server.handle(&mut od, &bad);
    let (_, code) = decode_abort(&out[0]).unwrap();
    assert_eq!(code, SdoAbortCode::InvalidCommandSpecifier);
    assert!(
        !server.is_busy(),
        "非法帧的 abort 必须连同会话一起拆除（八M7）"
    );
}

/// 十M6：客户端分段上传的末段短于服务端声明长度时必须中止
/// （BadLength → 0x0607_0013），不能静默返回截断数据。
#[test]
fn short_final_upload_segment_is_rejected() {
    use canopen_core::sdo::{encode_data_segment, encode_upload_initiate_segmented_response};
    let mut client = SdoClient::new();
    let _ = client.read(Address::new(0x2200, 0), DataType::Domain);
    // Server declares 10 bytes…
    let init = encode_upload_initiate_segmented_response(Address::new(0x2200, 0), 10);
    match client.on_response(&init).unwrap() {
        SdoEvent::Send(frames) => assert_eq!(frames[0][0], 0x60, "segment request"),
        other => panic!("expected segment request, got {other:?}"),
    }
    // …but the final segment carries only 3.
    let short = encode_data_segment(&[0xAA, 0xBB, 0xCC], false, true).unwrap();
    let err = client.on_response(&short).unwrap_err();
    assert!(
        matches!(
            err,
            Error::BadLength {
                declared: 10,
                actual: 3
            }
        ),
        "got {err:?}"
    );
}

/// 十M7：方向错误的段帧 abort 时必须同时清除会话（僵尸会话回归）。
#[test]
fn direction_mismatch_segment_clears_session() {
    use canopen_core::sdo::{
        decode_abort, encode_download_initiate_segmented, encode_upload_request,
    };
    // Download in flight + upload-segment request (CS=4).
    let mut server = SdoServer::new();
    let mut od = test_od();
    let init = encode_download_initiate_segmented(Address::new(0x2200, 0), 10);
    let _ = server.handle(&mut od, &init);
    assert!(server.is_busy(), "download armed");
    let bad: SdoPayload = [0x60, 0x00, 0x22, 0x00, 0, 0, 0, 0];
    let out = server.handle(&mut od, &bad);
    let (_, code) = decode_abort(&out[0]).unwrap();
    assert_eq!(code, SdoAbortCode::InvalidCommandSpecifier);
    assert!(
        !server.is_busy(),
        "download direction mismatch must clear the session（十M7）"
    );

    // Upload in flight + download-segment request (CS=0). 0x2401 is a
    // multi-byte object, so the request really arms a segmented upload
    // (zero-length objects abort at initiate, docs/DESIGN-DECISIONS.md 审-S2).
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    let req = encode_upload_request(Address::new(0x2401, 0));
    let resp = server.handle(&mut od, &req);
    assert!(resp[0][0] != 0x80, "segmented initiate response");
    assert!(server.is_busy(), "upload armed");
    let bad: SdoPayload = [0x00, 0x00, 0x22, 0x00, 0, 0, 0, 0];
    let out = server.handle(&mut od, &bad);
    let (_, code) = decode_abort(&out[0]).unwrap();
    assert_eq!(code, SdoAbortCode::InvalidCommandSpecifier);
    assert!(
        !server.is_busy(),
        "upload direction mismatch must clear the session（十M7）"
    );
}

/// 十L1：block download 的 end 帧数据超过声明长度时报
/// `TypeMismatchLengthHigh`（0x0607_0012），不再固定报 "low"。
#[test]
fn block_download_overlong_end_reports_length_high() {
    use canopen_core::sdo::block::{encode_download_initiate, encode_end, encode_sub_segment};
    use canopen_core::sdo::decode_abort;
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    let init = encode_download_initiate(Address::new(0x2400, 0), Some(10), false);
    let resp = server.handle(&mut od, &init);
    assert!(resp.iter().all(|f| f[0] != 0x80), "initiate accepted");
    // 7 + 4 = 11 bytes for a declared 10.
    let seg = encode_sub_segment(1, &[0xAA; 7], false).unwrap();
    let _ = server.handle(&mut od, &seg);
    let seg = encode_sub_segment(2, &[0xBB; 4], true).unwrap();
    let _ = server.handle(&mut od, &seg);
    let out = server.handle(&mut od, &encode_end(0, 0));
    let (_, code) = decode_abort(&out[0]).unwrap();
    assert_eq!(code, SdoAbortCode::TypeMismatchLengthHigh);
}

/// docs/DESIGN-DECISIONS.md B1: an upload-segment toggle violation aborts *and* names the
/// in-flight object (CANopenNode `CO_SDOserver.c:1500-1505`); the old code
/// echoed the request's reserved bytes, yielding index 0.
#[test]
fn upload_segment_toggle_abort_names_object() {
    use canopen_core::sdo::{decode_abort, encode_upload_request, encode_upload_segment_request};
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    // 0x2401 is multi-byte (900 B) → a segmented upload is armed on it.
    let _ = server.handle(&mut od, &encode_upload_request(Address::new(0x2401, 0)));
    assert!(server.is_busy(), "upload armed");

    // First poll expects toggle 0; send toggle 1.
    let bad = encode_upload_segment_request(true);
    let out = server.handle(&mut od, &bad);
    let (addr, code) = decode_abort(&out[0]).unwrap();
    assert_eq!(code, SdoAbortCode::ToggleNotAlternated);
    assert_eq!(
        addr,
        Address::new(0x2401, 0),
        "abort must name the in-flight object"
    );
    assert!(!server.is_busy());
}

/// docs/DESIGN-DECISIONS.md C2 (with the B1 address rule): a CS=5 END frame (0xA1) during a
/// *segmented* transfer is illegal — it must abort and clear the zombie
/// session instead of being swallowed as a late block-end response. The
/// abort names the object, not the END frame's unused/CRC bytes.
#[test]
fn cs5_end_during_segmented_transfer_clears_session() {
    use canopen_core::sdo::{decode_abort, encode_download_initiate_segmented};
    let mut server = SdoServer::new();
    let mut od = test_od();
    let init = encode_download_initiate_segmented(Address::new(0x2200, 0), 10);
    let _ = server.handle(&mut od, &init);
    assert!(server.is_busy(), "download armed");

    let bad: SdoPayload = [0xA1, 0, 0, 0, 0, 0, 0, 0];
    let out = server.handle(&mut od, &bad);
    let (addr, code) = decode_abort(&out[0]).unwrap();
    assert_eq!(code, SdoAbortCode::InvalidCommandSpecifier);
    assert_eq!(
        addr,
        Address::new(0x2200, 0),
        "abort must name the in-flight object"
    );
    assert!(
        !server.is_busy(),
        "illegal CS=5 END must tear the segmented session down"
    );
}

/// docs/DESIGN-DECISIONS.md F1 + SDO-30: a `seqno == 0` frame is treated like the reference's
/// `CO_SDOserver.c:98` predicate `seqno != SDO->block_seqno &&
/// SDO->block_seqno != 0U` — i.e. **the expected counter decides**, not the
/// received one:
///
/// * before the first segment of a sub-block (`block_seqno == 0`, "sequence
///   didn't start yet") the frame is ignored in silence;
/// * mid-sub-block a `seqno == 0` is a wrong sequence number, which breaks the
///   sub-block and **acks** the good prefix (never an abort), so the peer
///   learns where to resume instead of waiting for the 0.7×T partial ACK.
#[test]
fn block_download_seqno_zero_follows_the_expected_counter() {
    use canopen_core::sdo::block::{encode_download_initiate, encode_end, encode_sub_segment};
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    let init = encode_download_initiate(Address::new(0x2400, 0), Some(10), false);
    let _ = server.handle(&mut od, &init);
    assert!(server.is_busy(), "block download armed");

    // seqno 0 is not encodable as a valid segment, so craft the raw frame
    // (byte 0 = seqno, bytes 1.. = payload without the last flag).
    let bad: SdoPayload = [0x00, 0xAA, 0, 0, 0, 0, 0, 0];
    assert!(
        server.handle(&mut od, &bad).is_empty(),
        "seqno 0 must be ignored, not answered with an abort"
    );
    assert!(
        server.is_busy(),
        "the transfer must survive the stray frame"
    );

    // One good segment, then a mid-sub-block `seqno 0`: the sub-block breaks
    // with an ack of the good prefix (ackseq = 1) and the transfer stays
    // armed — the sender retransmits from there.
    let _ = server.handle(&mut od, &encode_sub_segment(1, &[0x11; 7], false).unwrap());
    let out = server.handle(&mut od, &bad);
    assert!(
        !out.is_empty() && out[0][0] & 0xF8 == 0xA0,
        "a wrong seqno mid-sub-block ends the sub-block with a 0xA2 ack: {out:?}"
    );
    assert_eq!(out[0][1], 1, "ack names the last good sequence number");
    assert!(server.is_busy(), "the transfer survives the sequence error");

    // A fresh window then completes normally: the last segment carries 3 data
    // bytes + 4 pad, reported as `unused` in the end frame (segments decode to
    // a fixed 7-byte window). The interrupted window is dropped with a client
    // abort first — re-sending an already-buffered segment would append it
    // twice, a pre-existing property of the rewind path shared with the gap
    // case, which this test deliberately does not pin.
    let abort = encode_abort(Address::new(0x2400, 0), SdoAbortCode::OutOfMemory);
    let _ = server.handle(&mut od, &abort);
    assert!(!server.is_busy(), "the client abort clears the transfer");
    let _ = server.handle(&mut od, &init);
    let _ = server.handle(&mut od, &encode_sub_segment(1, &[0x11; 7], false).unwrap());
    let _ = server.handle(&mut od, &encode_sub_segment(2, &[0x22; 3], true).unwrap());
    let out = server.handle(&mut od, &encode_end(4, 0));
    assert!(
        out.iter().all(|f| f[0] != 0x80),
        "no abort on the completing end frame: {out:?}"
    );
    assert!(!server.is_busy(), "end frame completes the transfer");
    let mut expected = vec![0x11; 7];
    expected.extend_from_slice(&[0x22; 3]);
    assert_eq!(
        od.read(Address::new(0x2400, 0)).unwrap(),
        Value::Bytes(expected)
    );
}

/// docs/DESIGN-DECISIONS.md R2: an ABORT received while a block download is active must clear
/// the transfer *and* record its code for `take_last_received_abort`, like
/// every other abort path.
#[test]
fn block_download_abort_records_reason() {
    use canopen_core::sdo::block::encode_download_initiate;
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    let init = encode_download_initiate(Address::new(0x2400, 0), Some(10), false);
    let _ = server.handle(&mut od, &init);
    assert!(server.is_busy(), "block download armed");

    let abort = encode_abort(Address::new(0x2400, 0), SdoAbortCode::OutOfMemory);
    assert!(
        server.handle(&mut od, &abort).is_empty(),
        "an abort is not answered"
    );
    assert!(!server.is_busy(), "abort clears the transfer");
    assert_eq!(
        server.take_last_received_abort(),
        Some(SdoAbortCode::OutOfMemory),
        "the abort reason must be recorded"
    );
}

/// docs/DESIGN-DECISIONS.md SDO-33 / SDO-35: the strict opt-in uses exact command bytes.
/// Lenient mode keeps python-canopen's masked dispatch.
#[test]
fn strict_sdo_config_rejects_masked_aborts_and_upload_initiate_bits() {
    let mut od = test_od();
    let malformed_abort = [0x81, 0x00, 0x10, 0x00, 0x01, 0x00, 0x04, 0x05];

    // Lenient: 0x81 is treated as an abort, clears the channel and records
    // the code from bytes 4..8 without a reply.
    let mut lenient = SdoServer::new();
    assert!(lenient.handle(&mut od, &malformed_abort).is_empty());
    assert_eq!(
        lenient.take_last_received_abort(),
        Some(SdoAbortCode::InvalidCommandSpecifier)
    );

    // Strict: 0x81 is an invalid command specifier, so the peer is notified.
    let mut strict = SdoServer::new();
    strict.set_config(SdoServerConfig { strict: true });
    let out = strict.handle(&mut od, &malformed_abort);
    assert_eq!(out.len(), 1, "strict mode answers with an abort");
    let (_, code) = decode_abort(&out[0]).unwrap();
    assert_eq!(code, SdoAbortCode::InvalidCommandSpecifier);
    assert!(strict.take_last_received_abort().is_none());

    // 0x41 is a masked upload initiate; only strict mode rejects it.
    let bad_upload = [0x41, 0x00, 0x10, 0x00, 0, 0, 0, 0];
    let out = lenient.handle(&mut od, &bad_upload);
    assert_eq!(out.len(), 1);
    assert_ne!(out[0][0], 0x80, "lenient mode services 0x41");
    let out = strict.handle(&mut od, &bad_upload);
    assert_eq!(out.len(), 1);
    let (_, code) = decode_abort(&out[0]).unwrap();
    assert_eq!(code, SdoAbortCode::InvalidCommandSpecifier);

    // Upload-segment polls accept only 0x60/0x70; 0x61 is rejected by the
    // strict gate before the toggle comparison can turn it into ToggleBit.
    od.write(Address::new(0x2200, 0), Value::Str("abcdefgh".to_string()))
        .unwrap();
    let upload = [0x40, 0x00, 0x22, 0x00, 0, 0, 0, 0];
    let out = strict.handle(&mut od, &upload);
    assert_eq!(out.len(), 1);
    assert_ne!(out[0][0], 0x80, "segmented upload started");
    let out = strict.handle(&mut od, &[0x61, 0, 0, 0, 0, 0, 0, 0]);
    assert_eq!(out.len(), 1);
    let (_, code) = decode_abort(&out[0]).unwrap();
    assert_eq!(code, SdoAbortCode::InvalidCommandSpecifier);
}
