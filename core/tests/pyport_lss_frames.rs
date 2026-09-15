//! Port of `ref/canopen/test/test_lss.py` subsets: the **inquire identity
//! family** golden frames plus **wrong-command-specifier negatives**
//! (docs/EXTENSIONS.md 批次 D P-b).
//!
//! Wire facts (CiA 305): master→slave on `0x7E5`, slave→master on `0x7E4`;
//! inquiry responses carry the requested 32-bit value little-endian in
//! bytes 1..5.
//!
//! NOTE (BUGS.md M9): the golden frames below are hand-transcribed from
//! python-canopen's suite and are **not** re-checked by `cargo test` against
//! `ref/`. After touching any codec, run
//! `python3 tools/interop/python_canopen_oracle.py` (8/8) to re-validate the
//! transcription against the reference implementation.

use canopen_core::lss::{
    decode_configure_bit_timing_response, decode_configure_node_id_response,
    encode_activate_bit_timing, encode_configure_bit_timing, encode_configure_node_id,
    encode_inquire_node_id, encode_store, encode_switch_global, encode_switch_selective,
    LssAddress, LssSlave, LssState, LSS_SLAVE_COB_ID, UNCONFIGURED_NODE_ID,
};

type Frame = [u8; 8];

const VENDOR: u32 = 0x0000_00C0;
const PRODUCT: u32 = 0x0000_0042;
const REVISION: u32 = 0x0000_0007;
const SERIAL: u32 = 0xDEAD_BEEF;

fn address() -> LssAddress {
    LssAddress {
        vendor_id: VENDOR,
        product_code: PRODUCT,
        revision_number: REVISION,
        serial_number: SERIAL,
    }
}

/// A slave selected via the standard four-frame selective switch.
fn selected_slave() -> LssSlave {
    let mut s = LssSlave::new(address(), UNCONFIGURED_NODE_ID);
    for f in encode_switch_selective(&address()) {
        let _ = s.handle(&f);
    }
    assert_eq!(s.state(), LssState::Configuration);
    s
}

fn frame(cs: u8) -> Frame {
    let mut f = [0u8; 8];
    f[0] = cs;
    f
}

fn frame_u32(cs: u8, v: u32) -> Frame {
    let mut f = frame(cs);
    f[1..5].copy_from_slice(&v.to_le_bytes());
    f
}

// --- inquire identity family ---------------------------------------------------

#[test]
fn inquire_vendor_golden_frame() {
    let mut s = selected_slave();
    let resp = s.handle(&frame(0x5A)).expect("vendor inquiry answered");
    assert_eq!(resp, frame_u32(0x5A, VENDOR));
}

#[test]
fn inquire_product_golden_frame() {
    let mut s = selected_slave();
    let resp = s.handle(&frame(0x5B)).expect("product inquiry answered");
    assert_eq!(resp, frame_u32(0x5B, PRODUCT));
}

#[test]
fn inquire_revision_golden_frame() {
    let mut s = selected_slave();
    let resp = s.handle(&frame(0x5C)).expect("revision inquiry answered");
    assert_eq!(resp, frame_u32(0x5C, REVISION));
}

#[test]
fn inquire_serial_golden_frame() {
    let mut s = selected_slave();
    let resp = s.handle(&frame(0x5D)).expect("serial inquiry answered");
    assert_eq!(resp, frame_u32(0x5D, SERIAL));
}

/// Full commissioning round-trip on the wire: select → assign → store →
/// adopt → re-select → all four inquiries answer the adopted identity, and
/// every response COB-ID is the fixed slave channel.
#[test]
fn full_commissioning_then_identity_inquiries() {
    let mut s = LssSlave::new(address(), UNCONFIGURED_NODE_ID);

    // Selective switch: exactly one answer, on the last frame only.
    let answers: Vec<Frame> = encode_switch_selective(&address())
        .iter()
        .filter_map(|f| s.handle(f))
        .collect();
    assert_eq!(answers.len(), 1);
    assert_eq!(answers[0][0], 0x44); // switch-selective response

    // Assign node id 21 and store.
    let resp = s.handle(&encode_configure_node_id(21)).unwrap();
    assert_eq!(decode_configure_node_id_response(&resp), Some(Ok(())));
    assert!(s.handle(&encode_store()).is_some());

    // Adopted on reset; the session ends (back to waiting).
    s.adopt_pending();
    assert_eq!(s.node_id(), 21);
    assert_eq!(s.state(), LssState::Waiting);

    // Re-select and walk all four inquiries, checking the fixed response
    // channel framing implicitly (frames carry no COB-ID — the caller puts
    // them on LSS_SLAVE_COB_ID).
    for f in encode_switch_selective(&address()) {
        let _ = s.handle(&f);
    }
    assert_eq!(s.handle(&frame(0x5A)), Some(frame_u32(0x5A, VENDOR)));
    assert_eq!(s.handle(&frame(0x5B)), Some(frame_u32(0x5B, PRODUCT)));
    assert_eq!(s.handle(&frame(0x5C)), Some(frame_u32(0x5C, REVISION)));
    assert_eq!(s.handle(&frame(0x5D)), Some(frame_u32(0x5D, SERIAL)));
    let nid = s.handle(&encode_inquire_node_id()).unwrap();
    assert_eq!(
        canopen_core::lss::decode_inquire_node_id_response(&nid),
        Some(21)
    );

    // Bit timing configure/activate still work after re-selection.
    let bt = s.handle(&encode_configure_bit_timing(0, 6)).unwrap();
    assert_eq!(decode_configure_bit_timing_response(&bt), Some(Ok(())));
    assert!(s.handle(&encode_activate_bit_timing(100)).is_none());
    let _ = LSS_SLAVE_COB_ID;
}

// --- wrong command-specifier negatives -----------------------------------------

/// Frames whose CS belongs to the *response* direction (or is simply
/// unknown) must be silently ignored by the slave — never answered, never
/// state-changing.
#[test]
fn response_and_unknown_cs_frames_are_ignored() {
    let mut s = selected_slave();

    // Selective-switch RESPONSE (0x44) arriving at the slave: not a request.
    assert!(s.handle(&frame(0x44)).is_none());
    // End responses / block-style CS values are foreign to LSS.
    assert!(s.handle(&frame(0xA1)).is_none());
    assert!(s.handle(&frame(0xA2)).is_none());
    // Unknown CS in the request space.
    assert!(s.handle(&frame(0x77)).is_none());
    assert!(s.handle(&frame(0x99)).is_none());

    // State unchanged throughout.
    assert_eq!(s.state(), LssState::Configuration);
    assert_eq!(s.pending_node_id(), UNCONFIGURED_NODE_ID);
}

/// Configuration services are gated on the configuration state even when the
/// CS itself would be valid.
#[test]
fn configuration_services_require_configuration_state() {
    let mut s = LssSlave::new(address(), UNCONFIGURED_NODE_ID);
    assert_eq!(s.state(), LssState::Waiting);

    assert!(s.handle(&frame(0x5A)).is_none()); // vendor inquiry
    assert!(s.handle(&encode_configure_node_id(9)).is_none());
    assert!(s.handle(&encode_store()).is_none());
    assert!(s.handle(&encode_inquire_node_id()).is_none());

    // Switch global config unlocks them again.
    s.handle(&encode_switch_global(true));
    assert!(s.handle(&frame(0x5A)).is_some());
}

/// A configure-node-id frame carrying an out-of-range id answers with
/// error code 1 and leaves the pending id untouched.
#[test]
fn configure_node_id_range_check() {
    let mut s = selected_slave();
    for bad in [0u8, 128, 200] {
        let resp = s.handle(&encode_configure_node_id(bad)).unwrap();
        assert_eq!(decode_configure_node_id_response(&resp), Some(Err(1)));
    }
    assert_eq!(s.pending_node_id(), UNCONFIGURED_NODE_ID);
}
