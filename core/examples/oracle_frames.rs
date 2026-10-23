//! Live frame dump for the wire-format oracle (`tools/interop/python_canopen_oracle.py`).
//!
//! Prints the frames canopen-rs's real SDO client produces for the same
//! operations the oracle drives through python-canopen, plus CRC-16/XMODEM
//! values over several payloads. The python oracle runs this example and
//! diffs its output against python-canopen's live frames — replacing the
//! hand-transcribed `EXPECTED_FRAMES` constants (`docs/DESIGN-DECISIONS.md` TH1/TH2).
//!
//! Output: one line per operation, `name hex hex …` (space-separated frame
//! bytes); CRC lines are `crc <label> <hex16>`.
//!
//! Run: `cargo run -q -p canopen-core --example oracle_frames`

use std::collections::VecDeque;

use canopen_core::datatypes::DataType;
use canopen_core::sdo::block::crc16;
use canopen_core::sdo::{encode_abort, SdoAbortCode, SdoClient, SdoEvent, SdoPayload};
use canopen_core::types::Address;

fn hex_frame(f: &SdoPayload) -> String {
    f.iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn print_frames(name: &str, frames: &[SdoPayload]) {
    let joined = frames.iter().map(hex_frame).collect::<Vec<_>>().join(" ");
    println!("{name} {joined}");
}

/// Download-initiate acknowledgement (SCCS=3) echoing the address.
fn dl_response(index: u16, sub: u8) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = 0x60;
    p[1] = index as u8;
    p[2] = (index >> 8) as u8;
    p[3] = sub;
    p
}

/// Download-segment acknowledgement (SCCS=1) carrying the toggle bit.
fn seg_response(toggle: bool) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = 0x20 | (0x10 * u8::from(toggle));
    p
}

/// Drives a segmented download to completion with canned acknowledgements,
/// collecting every transmitted frame — the Rust equivalent of the oracle's
/// `CapturedSdoClient` + `WritableStream` loop.
fn drive_segmented_download(addr: Address, payload: &[u8]) -> Vec<SdoPayload> {
    let mut client = SdoClient::new();
    let mut pending: VecDeque<SdoPayload> = client.write(addr, payload).unwrap().into();
    let mut sent = Vec::new();
    while let Some(frame) = pending.pop_front() {
        sent.push(frame);
        // The first TX is the initiate; every later one is a data segment
        // whose toggle bit we echo in the acknowledgement.
        let resp = if sent.len() == 1 {
            dl_response(addr.index, addr.subindex)
        } else {
            seg_response(frame[0] & 0x10 != 0)
        };
        match client.on_response(&resp).unwrap() {
            SdoEvent::Send(frames) => pending.extend(frames),
            SdoEvent::Done => break,
            SdoEvent::Complete(_, trailing) => {
                pending.extend(trailing);
                break;
            }
        }
    }
    sent
}

fn main() {
    // --- Expedited downloads (mirror the oracle's u32/u8/i16 cases) --------
    let mut client = SdoClient::new();
    let f = client
        .write(Address::new(0x2000, 0), &0x1234_5678u32.to_le_bytes())
        .unwrap();
    print_frames("u32_write", &f);

    let mut client = SdoClient::new();
    let f = client.write(Address::new(0x2001, 0), &[0x7F]).unwrap();
    print_frames("u8_write", &f);

    let mut client = SdoClient::new();
    let f = client
        .write(Address::new(0x6000, 1), &(-2i16).to_le_bytes())
        .unwrap();
    print_frames("i16_write", &f);

    // --- Upload request ------------------------------------------------------
    let mut client = SdoClient::new();
    let f = client.read(Address::new(0x1000, 0), DataType::Unsigned32);
    print_frames("upload_request", &f);

    // --- Segmented download of 20 bytes (initiate + full segment stream) ----
    let frames = drive_segmented_download(Address::new(0x2000, 0), &[b'A'; 20]);
    print_frames("segmented_download", &frames);

    // --- Block download initiate ---------------------------------------------
    let mut client = SdoClient::new();
    let f = client
        .write_block(Address::new(0x2000, 0), &[0u8; 100])
        .unwrap();
    print_frames("block_initiate", &f);

    // --- Abort (address is implementation-defined, code is CiA-legal) -------
    let abort = encode_abort(Address::new(0x1000, 0), SdoAbortCode::ObjectDoesNotExist);
    print_frames("abort", &[abort]);

    // --- CRC-16/XMODEM vectors for the end-to-end cross-check ----------------
    // Each line also carries the payload hex so the oracle can recompute
    // `binascii.crc_hqx(payload, 0)` over the identical bytes and diff
    // against our value (`docs/DESIGN-DECISIONS.md` TH2).
    let vectors: [(&str, &[u8]); 4] = [
        ("empty", b""),
        ("check", b"123456789"),
        (
            "lorem",
            b"Lorem ipsum dolor sit amet, consectetur adipiscing elit.",
        ),
        ("binary", &[0x00, 0x01, 0xFE, 0xFF, 0x80, 0x7F, 0x42]),
    ];
    for (label, data) in vectors {
        let payload_hex = data.iter().map(|b| format!("{b:02x}")).collect::<String>();
        println!("crc {label} {:04x} {payload_hex}", crc16(data));
    }
}
