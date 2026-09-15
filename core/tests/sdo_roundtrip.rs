//! SDO client ↔ server integration tests: full transactions against a real
//! object dictionary, without any bus.

use canopen_core::datatypes::{DataType, Value};
use canopen_core::object_dictionary::{AccessType, Entry, ObjectDictionary, Storage};
use canopen_core::sdo::{
    encode_abort, encode_download_segment_response, SdoAbortCode, SdoClient, SdoEvent, SdoPayload,
    SdoServer,
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

    // NOTE (BUGS.md R10-P2-16 resolved): a *width-correct* value-range
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
fn empty_string_goes_segmented_with_zero_length() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = test_od(); // 0x2200 starts as empty string
    let req = client.read(Address::new(0x2200, 0), DataType::VisibleString);
    assert_eq!(
        drive(&mut client, &mut server, &mut od, req).unwrap(),
        Some(Value::Str(String::new()))
    );
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

#[test]
fn block_download_without_size_declared_is_capped() {
    use canopen_core::sdo::block::{encode_download_initiate, encode_sub_segment};
    use canopen_core::sdo::decode_abort;
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    // No size declared (s bit clear): the receiver must still be capped at
    // MAX_VALUE_LEN instead of growing without bound (BUGS.md #2).
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

/// §5.5 / BUGS.md P1-3: a fresh initiate while another transfer is active
/// must first ABORT the stale transfer (GeneralError, old address), then be
/// serviced normally.
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

    // Frame 1: GeneralError abort addressed at the STALE transfer.
    assert_eq!(out.len(), 2, "stale abort + new response");
    let (addr, code) = decode_abort(&out[0]).unwrap();
    assert_eq!(addr.index, 0x2200);
    assert_eq!(code, SdoAbortCode::GeneralError);

    // Frame 2: the normal expedited-write response for the new request.
    assert_eq!(decode_download_response(&out[1]).unwrap().index, 0x2201);
    assert!(!server.is_busy());

    // The new write was recorded (the object itself is write-only).
    let drained = server.take_last_written();
    assert_eq!(drained.len(), 1);
    assert_eq!(drained[0].0.index, 0x2201);
}

/// take_last_written drains every recorded write in order (BUGS.md P2-4).
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

// --- BUGS.md P1-1 / P1-2 regressions -----------------------------------------

/// BUGS.md P1-1: sizes whose final sub-block segment number collides with the
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

/// BUGS.md P1-2: after a segmented upload completes, the server must return
/// to idle — the *next* transaction on the same channel must succeed without
/// receiving the stale-transfer GeneralError ABORT first.
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

/// BUGS.md P3-29: a zero-length object read via block upload must complete
/// immediately with the end frame instead of stalling after start.
#[test]
fn block_read_of_empty_object_completes_with_end() {
    let (mut client, mut server) = (SdoClient::new(), SdoServer::new());
    let mut od = bulk_od(); // 0x2400 starts as empty Domain
    let req = client.read_block(Address::new(0x2400, 0), DataType::Domain);
    match drive(&mut client, &mut server, &mut od, req).unwrap() {
        Some(Value::Bytes(b)) => assert!(b.is_empty()),
        other => panic!("expected empty bytes, got {other:?}"),
    }
}

/// BUGS.md P3-26: a block-upload initiate with blksize=0 must be rejected
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

/// BUGS.md P2-7: a gap in sub-block sequence numbers ends the sub-block with
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

/// BUGS.md P1-1 (see git history): after a gap the client acks what it has and the
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

/// BUGS.md R10-P2-4 (见 git 历史): a zero-length value must be writable via the
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

/// BUGS.md 八M7：在途分段传输收到非法 CS=5 帧（既非 start、非 sub-response、
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

    // Upload in flight + download-segment request (CS=0).
    let mut server = SdoServer::new();
    let mut od = test_od();
    let req = encode_upload_request(Address::new(0x2200, 0));
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

/// BUGS.md B1: an upload-segment toggle violation aborts *and* names the
/// in-flight object (CANopenNode `CO_SDOserver.c:1500-1505`); the old code
/// echoed the request's reserved bytes, yielding index 0.
#[test]
fn upload_segment_toggle_abort_names_object() {
    use canopen_core::sdo::{decode_abort, encode_upload_request, encode_upload_segment_request};
    let mut server = SdoServer::new();
    let mut od = test_od();
    // Empty string → segmented upload armed on 0x2200:0.
    let _ = server.handle(&mut od, &encode_upload_request(Address::new(0x2200, 0)));
    assert!(server.is_busy(), "upload armed");

    // First poll expects toggle 0; send toggle 1.
    let bad = encode_upload_segment_request(true);
    let out = server.handle(&mut od, &bad);
    let (addr, code) = decode_abort(&out[0]).unwrap();
    assert_eq!(code, SdoAbortCode::ToggleNotAlternated);
    assert_eq!(
        addr,
        Address::new(0x2200, 0),
        "abort must name the in-flight object"
    );
    assert!(!server.is_busy());
}

/// BUGS.md C2 (with the B1 address rule): a CS=5 END frame (0xA1) during a
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

/// BUGS.md B1: a block-download segment error (seqno 0) aborts naming the
/// in-flight object, not the segment's data bytes.
#[test]
fn block_download_invalid_seqno_abort_names_object() {
    use canopen_core::sdo::block::encode_download_initiate;
    use canopen_core::sdo::decode_abort;
    let mut server = SdoServer::new();
    let mut od = bulk_od();
    let init = encode_download_initiate(Address::new(0x2400, 0), Some(10), false);
    let _ = server.handle(&mut od, &init);
    assert!(server.is_busy(), "block download armed");

    // seqno 0 is not encodable as a valid segment, so craft the raw frame
    // (byte 0 = seqno, bytes 1.. = payload without the last flag).
    let bad: SdoPayload = [0x00, 0xAA, 0, 0, 0, 0, 0, 0];
    let out = server.handle(&mut od, &bad);
    let (addr, code) = decode_abort(&out[0]).unwrap();
    assert_eq!(code, SdoAbortCode::InvalidSequenceNumber);
    assert_eq!(
        addr,
        Address::new(0x2400, 0),
        "abort must name the in-flight object"
    );
    assert!(!server.is_busy());
}
