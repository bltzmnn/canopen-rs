//! `FileStorage` round-trip: save → mutate → load restores; reset reverts
//! to defaults and empties the file.

use canopen_core::datatypes::{DataType, Value};
use canopen_core::object_dictionary::{AccessType, Entry, ObjectDictionary, Storage};
use canopen_core::standard::StandardObjects;
use canopen_core::storage::{install_standard_objects, ParameterStorage};
use canopen_core::types::Address;
use canopen_host::storage::FileStorage;

fn test_od() -> ObjectDictionary {
    let mut od = StandardObjects::new().build();
    install_standard_objects(&mut od, 1);
    for (sub, default) in [(1u8, 111u64), (2, 222)] {
        let mut e = Entry::new(
            "persisted",
            DataType::Unsigned16,
            AccessType::Rw,
            Storage::Value(Value::Uint(default)),
        );
        if sub == 1 {
            e = e.persistent();
        }
        od.insert(Address::new(0x6000, sub), e);
    }
    od
}

#[test]
fn file_roundtrip_and_reset() {
    let path = std::env::temp_dir().join(format!(
        "canopen-rs-test-params-{}-{}.txt",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.subsec_nanos())
            .unwrap_or(0)
    ));
    let _ = std::fs::remove_file(&path);
    let backend = FileStorage::new(path.clone());

    let mut od = test_od();
    // Only 0x6000:1 is persistent — 0x6000:2 must never appear in the file.
    od.set_value(Address::new(0x6000, 1), Value::Uint(0xBEEF))
        .unwrap();

    backend.save(&od).unwrap();
    let raw = std::fs::read_to_string(&path).unwrap();
    assert!(raw.contains("# canopen-rs parameter storage v1"));
    assert!(
        raw.contains("6000 001 02 ef be"),
        "unexpected content:\n{raw}"
    );
    assert!(!raw.contains("6000 002"));

    // Mutate, then load restores the saved value.
    od.set_value(Address::new(0x6000, 1), Value::Uint(7))
        .unwrap();
    backend.load(&mut od).unwrap();
    assert_eq!(
        od.read(Address::new(0x6000, 1)).unwrap(),
        Value::Uint(0xBEEF)
    );

    // Reset reverts to the construction-time defaults and clears the store.
    let defaults = test_od();
    backend.reset(&mut od, &defaults, 1).unwrap();
    assert_eq!(od.read(Address::new(0x6000, 1)).unwrap(), Value::Uint(111));
    // …so a subsequent load is a no-op on values.
    backend.load(&mut od).unwrap();
    assert_eq!(od.read(Address::new(0x6000, 1)).unwrap(), Value::Uint(111));

    let _ = std::fs::remove_file(&path);
}

#[test]
fn missing_file_loads_as_noop() {
    let backend = FileStorage::new(
        std::env::temp_dir()
            .join(format!(
                "canopen-rs-nonexistent-dir-{}/params.txt",
                std::process::id()
            ))
            .join("nested")
            .join("deeper"),
    );
    // Distinct per-process path keeps parallel CI runs from racing on a fixed
    // /tmp name (BUGS.md R10-P3).
    let mut od = test_od();
    backend.load(&mut od).unwrap(); // no error
    assert_eq!(od.read(Address::new(0x6000, 1)).unwrap(), Value::Uint(111));
}

#[test]
fn save_is_atomic_via_temp_file() {
    // BUGS.md m8: a crash mid-save must never leave a truncated parameter
    // file — the payload lands in a sibling .tmp first, then renames over
    // the target (CANopenNode CO_storageLinux.c).
    let dir = std::env::temp_dir().join("canopen-rs-atomic-test");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("params.txt");
    let _ = std::fs::remove_file(&path);

    let storage = FileStorage::new(path.clone());
    let od = StandardObjects::new().build();
    storage.save(&od).unwrap();

    assert!(path.exists());
    assert!(
        !dir.join("params.tmp").exists(),
        "tmp file must be renamed away"
    );
    // Content intact and re-loadable.
    let reloaded = FileStorage::new(path.clone());
    let mut od2 = StandardObjects::new().build();
    reloaded.load(&mut od2).unwrap();
}

/// `BUGS.md` P2-1: declared-width encoding — zero and negative values of
/// fixed-width types must survive save → load (previously saved at minimal
/// width and silently skipped on load).
#[test]
fn small_and_negative_fixed_width_values_round_trip() {
    use canopen_core::datatypes::{DataType, Value};
    use canopen_core::object_dictionary::{Entry, Storage};
    use canopen_core::types::Address;

    let path = std::env::temp_dir().join(format!(
        "canopen_rs_p21_{}_{}.bin",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    let backend = FileStorage::new(path.clone());

    let mut od = ObjectDictionary::new();
    for (addr, dt, value) in [
        (
            Address::new(0x4001, 0),
            DataType::Unsigned16,
            Value::Uint(0),
        ),
        (
            Address::new(0x4002, 0),
            DataType::Integer16,
            Value::Int(-1234),
        ),
        (Address::new(0x4003, 0), DataType::Unsigned8, Value::Uint(7)),
    ] {
        let mut entry = Entry::new(
            "p",
            dt,
            canopen_core::object_dictionary::AccessType::Rw,
            Storage::Value(value.clone()),
        );
        entry.persistent = true;
        od.insert(addr, entry);
        od.set_value(addr, value).unwrap();
    }

    backend.save(&od).expect("save");
    // Clobber in-memory state, then restore.
    od.set_value(Address::new(0x4003, 0), Value::Uint(0))
        .unwrap();
    backend.load(&mut od).expect("load");

    assert_eq!(od.read(Address::new(0x4001, 0)).unwrap(), Value::Uint(0));
    assert_eq!(od.read(Address::new(0x4002, 0)).unwrap(), Value::Int(-1234));
    assert_eq!(od.read(Address::new(0x4003, 0)).unwrap(), Value::Uint(7));
}

/// BUGS.md H2: a line carrying MORE hex tokens than its declared `len` is
/// corrupt — the old `parts.take(len)` silently dropped the excess, so a
/// damaged row whose first `len` bytes happened to be valid loaded as a
/// clean entry. It must now be skipped like any other bad line.
#[test]
fn line_with_extra_tokens_is_rejected() {
    use canopen_core::object_dictionary::ObjectDictionary;

    let path = std::env::temp_dir().join(format!(
        "canopen_rs_extra_tokens_{}.bin",
        std::process::id()
    ));
    let _ = std::fs::remove_file(&path);
    // Declared len=1 with two data bytes: the excess token must mark the
    // line corrupt, so the value never loads.
    std::fs::write(
        &path,
        "# canopen-rs parameter storage v1\n6000 001 01 ef be\n",
    )
    .unwrap();

    let mut od = ObjectDictionary::new();
    let mut entry = test_od()
        .entry(Address::new(0x6000, 1))
        .expect("test_od has 0x6000:1")
        .clone();
    entry.persistent = true;
    od.insert(Address::new(0x6000, 1), entry);
    let backend = FileStorage::new(path.clone());
    backend.load(&mut od).unwrap();

    // Default (111) untouched — the corrupt line was skipped.
    assert_eq!(od.read(Address::new(0x6000, 1)).unwrap(), Value::Uint(111));
    let _ = std::fs::remove_file(&path);
}

/// BUGS.md M7：损坏/恶意的存储文件传入超大 `len`（如 `4294967295`）曾触发
/// 无上限 `with_capacity(len)` 分配（DoS/OOM）。修复后超限行按坏行跳过，
/// 既不预分配也不解析其数据字节，加载不恐慌、值保持默认。
#[test]
fn m7_oversized_len_line_is_skipped() {
    let path = std::env::temp_dir().join(format!("canopen_rs_m7_{}.bin", std::process::id()));
    let _ = std::fs::remove_file(&path);
    // `len = 4294967295` 远超上限——该行必须被跳过，而非尝试分配约 4GB。
    std::fs::write(
        &path,
        "# canopen-rs parameter storage v1\n6000 001 4294967295 ef be\n",
    )
    .unwrap();

    let mut od = ObjectDictionary::new();
    let mut entry = test_od()
        .entry(Address::new(0x6000, 1))
        .expect("test_od has 0x6000:1")
        .clone();
    entry.persistent = true;
    od.insert(Address::new(0x6000, 1), entry);
    let backend = FileStorage::new(path.clone());
    // 不恐慌、不 OOM；坏行被跳过，值保持默认 111。
    backend.load(&mut od).unwrap();
    assert_eq!(od.read(Address::new(0x6000, 1)).unwrap(), Value::Uint(111));
    let _ = std::fs::remove_file(&path);
}

/// BUGS.md M8：`save` 遇到条目编码失败（如取值越界 `encode_as` 报错）必须
/// 上报首个错误并返回 `Err`，而非静默跳过该条目却仍返回 `Ok(())`，造成
/// 重载时该对象回到默认值的隐蔽数据丢失。
#[test]
fn m8_save_reports_encode_error() {
    let path = std::env::temp_dir().join(format!("canopen_rs_m8_{}.bin", std::process::id()));
    let _ = std::fs::remove_file(&path);
    let backend = FileStorage::new(path.clone());

    let mut od = ObjectDictionary::new();
    // Unsigned8 条目却持有越界值 300 —— `encode_as` 必然失败。
    let mut entry = Entry::new(
        "bad",
        DataType::Unsigned8,
        AccessType::Rw,
        Storage::Value(Value::Uint(300)),
    );
    entry.persistent = true;
    od.insert(Address::new(0x6000, 1), entry);

    let err = backend.save(&od).expect_err("编码失败的保存必须返回 Err");
    assert!(
        matches!(err, canopen_core::types::Error::ValueOutOfRange { .. }),
        "save 应上报 ValueOutOfRange，实际：{err:?}"
    );
    let _ = std::fs::remove_file(&path);
}

/// BUGS.md 八M5：`UnicodeString` 条目在 OD 中以 `Value::Str` 存放时，持久化必须
/// 写目标类型的线格式 UTF-16LE（CiA 306 / python-canopen），而不是值的最小
/// UTF-8 编码——否则重载后 `Str` 漂移为 `Bytes`，后续 SDO 上传把 UTF-8 字节
/// 当 UTF-16LE 发出，非 ASCII 文本全部乱码。
#[test]
fn m5_unicode_string_persists_utf16le() {
    let path = std::env::temp_dir().join(format!("canopen_rs_m5_{}.bin", std::process::id()));
    let _ = std::fs::remove_file(&path);
    let backend = FileStorage::new(path.clone());

    let mut od = test_od();
    let mut entry = Entry::new(
        "unicode",
        DataType::UnicodeString,
        AccessType::Rw,
        Storage::Value(Value::Str("é中".into())),
    );
    entry.persistent = true;
    od.insert(Address::new(0x6001, 0), entry);
    let addr = Address::new(0x6001, 0);
    od.set_value(addr, Value::Str("é中".into())).unwrap();

    backend.save(&od).unwrap();

    // 线上形式：'é' = U+00E9 → e9 00，'中' = U+4E2D → 2d 4e（小端）。
    let raw = std::fs::read(&path).unwrap();
    let text = String::from_utf8_lossy(&raw).into_owned();
    assert!(
        text.contains("6001 000 04 e9 00 2d 4e"),
        "file must store the UTF-16LE wire form, got:\n{text}"
    );
    assert!(
        !text.contains("c3 a9"),
        "file must not contain the UTF-8 form of 'é':\n{text}"
    );

    // 重载后再次按 UnicodeString 编码必须与线上形式逐字节一致。
    od.set_value(addr, Value::Str("".into())).unwrap();
    backend.load(&mut od).unwrap();
    let value = od.read(addr).unwrap();
    let mut buf = vec![0u8; value.encoded_len_as(DataType::UnicodeString)];
    value.encode_as(DataType::UnicodeString, &mut buf).unwrap();
    assert_eq!(buf, vec![0xE9, 0x00, 0x2D, 0x4E]);

    let _ = std::fs::remove_file(&path);
}
