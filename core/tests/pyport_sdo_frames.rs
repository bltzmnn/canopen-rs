//! Port of `ref/canopen/test/test_sdo.py` block-transfer golden frame
//! sequences (docs/EXTENSIONS.md 批次 D P-b).
//!
//! The python suite drives its SDO client against a scripted mock bus
//! (`(TX, bytes) / (RX, bytes)` lists). This port runs **our** client against
//! the same scripts: every TX entry asserts a byte-exact match with what our
//! client emits, every RX entry is fed into it. Any divergence from the
//! python-canopen wire behaviour fails here.
//!
//! NOTE (BUGS.md M9): the golden frames below are hand-transcribed from
//! python-canopen's suite and are **not** re-checked by `cargo test` against
//! `ref/`. After touching any codec, run
//! `python3 tools/interop/python_canopen_oracle.py` (8/8) to re-validate the
//! transcription against the reference implementation.

use canopen_core::datatypes::{DataType, Value};
use std::collections::VecDeque;

use canopen_core::sdo::{SdoClient, SdoEvent, SdoPayload};
use canopen_core::types::Address;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dir {
    Tx,
    Rx,
}

use Dir::*;

fn f(bytes: &[u8]) -> SdoPayload {
    let mut p = [0u8; 8];
    p[..bytes.len()].copy_from_slice(bytes);
    p
}

/// Walks a python-style script: TX entries assert byte-exact output from the
/// client's pending queue, RX entries feed frames in. Returns the frames the
/// client actually transmitted plus the value of the last `Complete` event
/// (if any) — so golden-frame tests can also assert the *decoded* result,
/// not just the wire bytes (`BUGS.md` TH3).
fn run_script(
    client: &mut SdoClient,
    first_tx: Vec<SdoPayload>,
    script: &[(Dir, SdoPayload)],
) -> (Vec<SdoPayload>, Option<Value>) {
    let mut pending: VecDeque<SdoPayload> = first_tx.into();
    let mut sent = Vec::new();
    let mut completed = None;
    for (dir, frame) in script {
        match dir {
            Dir::Tx => {
                let actual = pending.pop_front().expect("script wants TX, client idle");
                assert_eq!(&actual, frame, "TX mismatch");
                sent.push(actual);
            }
            Dir::Rx => {
                let event = match client.on_response(frame) {
                    Ok(ev) => ev,
                    Err(e) => panic!("protocol failure at RX {frame:02x?}: {e}"),
                };
                match event {
                    SdoEvent::Send(frames) => pending.extend(frames),
                    // Trailing frames (e.g. the block end response) still
                    // need to go on the wire.
                    SdoEvent::Complete(value, trailing) => {
                        completed = Some(value);
                        pending.extend(trailing);
                    }
                    SdoEvent::Done => {}
                }
            }
        }
    }
    assert!(
        pending.is_empty(),
        "client still holds {} unsent frames",
        pending.len()
    );
    (sent, completed)
}

/// test_sdo.py:173 — block download of 30 bytes, full frame sequence.
#[test]
fn block_download_golden_frames() {
    let data = b"A really really long string...";
    let mut client = SdoClient::new();
    let first = client.write_block(Address::new(0x2000, 0), data).unwrap();

    let script = [
        (Tx, f(&[0xC6, 0x00, 0x20, 0x00, 0x1E, 0x00, 0x00, 0x00])),
        (Rx, f(&[0xA4, 0x00, 0x20, 0x00, 0x7F, 0x00, 0x00, 0x00])),
        (Tx, f(&[0x01, 0x41, 0x20, 0x72, 0x65, 0x61, 0x6C, 0x6C])), // "A reall"
        (Tx, f(&[0x02, 0x79, 0x20, 0x72, 0x65, 0x61, 0x6C, 0x6C])), // "y reall"
        (Tx, f(&[0x03, 0x79, 0x20, 0x6C, 0x6F, 0x6E, 0x67, 0x20])), // "y long "
        (Tx, f(&[0x04, 0x73, 0x74, 0x72, 0x69, 0x6E, 0x67, 0x2E])), // "string."
        (Tx, f(&[0x85, 0x2E, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00])), // ".."+pad, last
        (Rx, f(&[0xA2, 0x05, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00])),
        // End frame carries unused=5 and CRC-16/XMODEM over the payload.
        (Tx, f(&[0xD5, 0x45, 0x69, 0x00, 0x00, 0x00, 0x00, 0x00])),
        (Rx, f(&[0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),
    ];
    let (_sent, _value) = run_script(&mut client, first, &script);

    // Cross-check the end-frame CRC against an independent computation.
    let crc = canopen_core::sdo::block::crc16(data);
    assert_eq!(crc, 0x6945);
}

/// test_sdo.py:201 — block upload of "Tiny Node - Mega Domains !".
#[test]
fn block_upload_golden_frames() {
    let mut client = SdoClient::new();
    let first = client.read_block(Address::new(0x1008, 0), DataType::VisibleString);

    let script = [
        (Tx, f(&[0xA4, 0x08, 0x10, 0x00, 0x7F, 0x00, 0x00, 0x00])),
        (Rx, f(&[0xC6, 0x08, 0x10, 0x00, 0x1A, 0x00, 0x00, 0x00])),
        (Tx, f(&[0xA3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),
        (Rx, f(&[0x01, 0x54, 0x69, 0x6E, 0x79, 0x20, 0x4E, 0x6F])), // "Tiny No"
        (Rx, f(&[0x02, 0x64, 0x65, 0x20, 0x2D, 0x20, 0x4D, 0x65])), // "de - Me"
        (Rx, f(&[0x03, 0x67, 0x61, 0x20, 0x44, 0x6F, 0x6D, 0x61])), // "ga Doma"
        (Rx, f(&[0x84, 0x69, 0x6E, 0x73, 0x20, 0x21, 0x00, 0x00])), // "ins !"+pad, last
        (Tx, f(&[0xA2, 0x04, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00])),
        (Rx, f(&[0xC9, 0x40, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x00])),
        (Tx, f(&[0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),
    ];
    let (_sent, value) = run_script(&mut client, first, &script);
    // BUGS.md TH3: the golden frames were only ever compared byte-by-byte —
    // a value mis-decoded into the wrong string still passed. Assert the
    // *decoded* result surfaces through `Complete`.
    assert_eq!(
        value,
        Some(Value::Str("Tiny Node - Mega Domains !".to_string()))
    );
}

/// test_sdo.py:218 (compressed) — a partial acknowledgement rewinds the
/// transfer: sequence numbering restarts at 1 and the unacknowledged tail is
/// resent. The python original scripts the same behaviour over a
/// 127-segment block with a corrupted seqno at 0x34.
#[test]
fn partial_ack_triggers_rewind_and_resend() {
    let data: Vec<u8> = (0..28u8).collect();
    let mut client = SdoClient::new();
    let first = client.write_block(Address::new(0x2000, 0), &data).unwrap();

    use canopen_core::sdo::block::encode_sub_segment as enc_seg;
    let seg = |last: bool, seqno: u8, chunk: &[u8]| -> SdoPayload {
        enc_seg(seqno, chunk, last).expect("bounded")
    };

    let script: Vec<(Dir, SdoPayload)> = vec![
        (Tx, f(&[0xC6, 0x00, 0x20, 0x00, 28, 0x00, 0x00, 0x00])),
        // Server negotiates blksize=4.
        (Rx, f(&[0xA4, 0x00, 0x20, 0x00, 0x04, 0x00, 0x00, 0x00])),
        // First attempt: four segments; #4 ends the payload → last flag.
        (Tx, seg(false, 1, &data[0..7])),
        (Tx, seg(false, 2, &data[7..14])),
        (Tx, seg(false, 3, &data[14..21])),
        (Tx, seg(true, 4, &data[21..28])),
        // Partial ack: only segments 1–2 arrived…
        (Rx, f(&[0xA2, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00])),
        // …retransmission restarts at sequence number 1 with the ORIGINAL
        // bytes from the acked offset; segment 2 again ends the payload.
        (Tx, seg(false, 1, &data[14..21])),
        (Tx, seg(true, 2, &data[21..28])),
        // Full retransmitted window acknowledged → end frame. 28 bytes =
        // 4 × 7 → unused = 0 → command byte 0xC1. CRC covers the payload.
        (Rx, f(&[0xA2, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00])),
        (Tx, {
            let crc = canopen_core::sdo::block::crc16(&data).to_le_bytes();
            f(&[0xC1, crc[0], crc[1], 0x00, 0x00, 0x00, 0x00, 0x00])
        }),
        // …and the end response closes the transaction.
        (Rx, f(&[0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),
    ];

    let (sent, _value) = run_script(&mut client, first, &script);

    // Rewind verification: frames 8–9 (after initiate/resp + 4 segments +
    // partial-ack RX) are the retransmission, restarting at seqno 1.
    let resent: Vec<&SdoPayload> = sent.iter().skip(5).take(2).collect();
    assert_eq!(
        *resent[0],
        seg(false, 1, &data[14..21]),
        "rewound to seqno 1"
    );
    assert_eq!(
        *resent[1],
        seg(true, 2, &data[21..28]),
        "final segment flagged"
    );
}
/// test_sdo.py:191 — zero-length segmented download. Formerly documented
/// deviation D5 (API rejected empty payloads); since BUGS.md R10-P2-4 the
/// client matches python-canopen exactly: `size=0` initiate followed by an
/// empty final segment (`c=1`, no data). Full wire coverage lives in
/// `zero_length_write_roundtrips_via_segmented`; this keeps the frame-shape
/// assertion at the API boundary.
#[test]
fn zero_length_download_emits_empty_final_segment() {
    let mut client = SdoClient::new();
    let frames = client.write(Address::new(0x2000, 0), b"").unwrap();
    assert_eq!(frames.len(), 1);
    // Segmented initiate with size=0 (command 0x21, s flag set).
    assert_eq!(frames[0][0], 0x21);
    assert_eq!(&frames[0][4..8], &[0, 0, 0, 0]);
}
