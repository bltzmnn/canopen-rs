//! Port of `ref/canopen/test/test_emcy.py` — consumer/producer pairs over
//! the enhanced [`EmcyConsumer`] (callbacks, isolation, active table) plus
//! producer idempotence (docs/EXTENSIONS.md 批次 D P-c).
//!
//! NOTE (docs/DESIGN-DECISIONS.md M9): the golden frames below are hand-transcribed from
//! python-canopen's suite and are **not** re-checked by `cargo test` against
//! `ref/`. After touching any codec, run
//! `python3 tools/interop/python_canopen_oracle.py` (8/8) to re-validate the
//! transcription against the reference implementation.

use std::sync::{Arc, Mutex};

use canopen_core::emcy::{err_reg, EmcyConsumer, EmcyFrame, EmcyProducer};
use canopen_core::object_dictionary::ObjectDictionary;
use canopen_core::types::NodeId;

const NODE: u8 = 9;

fn frame(code: u16, reg: u8, data: [u8; 5]) -> EmcyFrame {
    EmcyFrame {
        code,
        register: reg,
        data,
    }
}

fn od() -> ObjectDictionary {
    let mut od = ObjectDictionary::new();
    canopen_core::emcy::install_standard_objects(&mut od, NodeId::new(NODE).unwrap());
    od
}

/// python test_emcy_consumer_on_emcy — multiple callbacks receive the same
/// information; reset code clears the active table but keeps the log.
#[test]
fn consumer_on_emcy_multiple_callbacks_and_reset() {
    let mut cons = EmcyConsumer::new(16);
    let acc1: Arc<Mutex<Vec<EmcyFrame>>> = Arc::new(Mutex::new(Vec::new()));
    let acc2: Arc<Mutex<Vec<EmcyFrame>>> = Arc::new(Mutex::new(Vec::new()));
    {
        let a = acc1.clone();
        cons.add_callback(move |_, f| a.lock().unwrap().push(*f));
    }
    {
        let a = acc2.clone();
        cons.add_callback(move |_, f| a.lock().unwrap().push(*f));
    }

    let node = NodeId::new(NODE).unwrap();
    cons.on_frame(node, frame(0x2001, 0x02, [0x00, 1, 2, 3, 4]));
    cons.on_frame(node, frame(0x9010, 0x01, [0x04, 3, 2, 1, 0]));

    assert_eq!(cons.recent().count(), 2);
    assert_eq!(cons.active().len(), 2);
    assert_eq!(cons.active()[0], (NODE, 0x2001));
    assert_eq!(cons.active()[1], (NODE, 0x9010));
    assert_eq!(acc1.lock().unwrap().len(), 2);
    assert_eq!(acc2.lock().unwrap().len(), 2);
    assert_eq!(*acc1.lock().unwrap(), *acc2.lock().unwrap());

    // Error-reset frame (code 0x0000): log grows, active clears.
    cons.on_frame(node, frame(0x0000, 0x00, [0; 5]));
    assert_eq!(cons.recent().count(), 3);
    assert!(cons.active().is_empty());
}

/// python test_emcy_consumer_callback_exception_handling — a panicking
/// callback must not break the others or the caller.
#[test]
fn callback_panic_isolation() {
    let mut cons = EmcyConsumer::new(8);
    let ok: Arc<Mutex<Vec<&'static str>>> = Arc::new(Mutex::new(Vec::new()));

    let o1 = ok.clone();
    cons.add_callback(move |_, _| o1.lock().unwrap().push("success1"));
    cons.add_callback(|_, _| panic!("Test exception in callback"));
    let o2 = ok.clone();
    cons.add_callback(move |_, _| o2.lock().unwrap().push("success2"));

    let node = NodeId::new(NODE).unwrap();
    cons.on_frame(node, frame(0x2001, 0, [0; 5]));
    assert_eq!(*ok.lock().unwrap(), vec!["success1", "success2"]);
}

/// python test_emcy_consumer_error_reset_variants — any code with value
/// `0x0000` acts as a reset regardless of register/data content.
#[test]
fn error_reset_variants_clear_active() {
    let mut cons = EmcyConsumer::new(8);
    let node = NodeId::new(NODE).unwrap();
    cons.on_frame(node, frame(0x2310, err_reg::CURRENT, [0; 5]));
    cons.on_frame(node, frame(0x5000, err_reg::GENERIC, [0; 5]));
    assert_eq!(cons.active().len(), 2);

    // Reset variant with non-zero data bytes still resets.
    cons.on_frame(node, frame(0x0000, 0x7F, [1, 2, 3, 4, 5]));
    assert!(cons.active().is_empty());
}

/// python test_emcy_producer_send/reset — idempotent raise, reset frame on
/// clear, and re-raise after clear produces a fresh frame.
#[test]
fn producer_send_reset_lifecycle() {
    let mut em = EmcyProducer::new();
    let mut od = od();

    let f1 = em
        .raise(&mut od, 0x2001, err_reg::CURRENT, [1, 2, 3, 4, 5])
        .unwrap();
    assert_eq!(&f1[..3], &[0x01, 0x20, err_reg::CURRENT]);
    // Idempotent re-raise.
    assert!(em
        .raise(&mut od, 0x2001, err_reg::CURRENT, [0; 5])
        .is_none());

    let reset = em.clear(&mut od, 0x2001).unwrap();
    assert_eq!(&reset[..2], &[0x00, 0x00]);

    // Re-raise after clear works again.
    assert!(em
        .raise(&mut od, 0x2001, err_reg::CURRENT, [0; 5])
        .is_some());
}

/// python test_producer_reset_consumer_integration — a produced reset frame
/// clears the consumer's active table when fed through it.
#[test]
fn producer_reset_frame_clears_consumer_active() {
    let mut em = EmcyProducer::new();
    let mut cons = EmcyConsumer::new(8);
    let mut prod_od = od();
    let node = NodeId::new(NODE).unwrap();

    let f1 = em
        .raise(&mut prod_od, 0x3100, err_reg::VOLTAGE, [0; 5])
        .unwrap();
    cons.on_frame(node, EmcyFrame::decode(&f1).unwrap());
    assert_eq!(cons.active().len(), 1);

    let reset = em.clear(&mut prod_od, 0x3100).unwrap();
    cons.on_frame(node, EmcyFrame::decode(&reset).unwrap());
    assert!(cons.active().is_empty());
}
