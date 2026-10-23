//! File-backed `canopen_core::storage::ParameterStorage`
//! implementation — the Linux counterpart of CANopenNode's
//! `CO_storageLinux.c`.
//!
//! Format (v2): line-based, one persisted entry per line, preceded by a
//! `v2` marker:
//!
//! ```text
//! # canopen-rs parameter storage v2
//! <index-hex> <sub-dec> <len> <hex bytes…> <crc16>
//! ```
//!
//! The CRC16 (XMODEM/CCITT init 0, the same polynomial as CANopenNode's
//! `crc16_ccitt(…, 0)`) covers the canonical text of all fields before it.
//! Writes are verified by re-reading the temp file before the atomic rename,
//! and corruption found while loading is reported instead of silently
//! skipped (`docs/DESIGN-DECISIONS.md` G7). Legacy v1 files (no CRC) are still readable, and
//! they are **trusted verbatim**: a v1 line has no checksum, so a flipped hex
//! nibble is structurally valid and is applied as written, with nothing to
//! count as `corrupt`/`stale` (docs/DESIGN-DECISIONS.md HOST-06). Re-saving through this backend
//! upgrades the file to v2.
//! Dependency-free by design (no serde).

use std::fs;
use std::path::{Path, PathBuf};

use canopen_core::datatypes::Value;
use canopen_core::object_dictionary::ObjectDictionary;
use canopen_core::sdo::block::crc16;
use canopen_core::storage::ParameterStorage;
use canopen_core::types::{Address, Error, Result};

/// Current on-disk format version.
const FORMAT_VERSION: u8 = 2;

/// Comment/version marker prefix shared by every format version.
const VERSION_MARKER: &str = "# canopen-rs parameter storage v";

/// 单条参数的最大字节数上限（docs/DESIGN-DECISIONS.md M7）：`read_entries` 曾用文件里的
/// `len` 直接 `with_capacity(len)`，损坏/恶意的存储文件传入超大 `len`
/// （如 `4294967295`）会尝试分配约 4GB。超过此上限的行按坏行跳过。
const MAX_PARAM_LEN: usize = 8192;

/// Parses one data line's canonical fields (without the CRC token):
/// `<index-hex> <sub-dec> <len> <hex bytes…>`. `None` on any structural
/// damage — malformed header, oversized/negative `len`, wrong number of
/// data tokens (docs/DESIGN-DECISIONS.md H2/M7).
fn parse_line(fields: &[&str]) -> Option<(Address, Vec<u8>)> {
    let (Some(index), Some(sub), Some(len)) = (
        fields.first().and_then(|p| u16::from_str_radix(p, 16).ok()),
        fields.get(1).and_then(|p| p.parse::<u8>().ok()),
        fields.get(2).and_then(|p| p.parse::<usize>().ok()),
    ) else {
        return None;
    };
    if len > MAX_PARAM_LEN || fields.len() != 3 + len {
        return None;
    }
    let mut data = Vec::with_capacity(len);
    for chunk in &fields[3..] {
        data.push(u8::from_str_radix(chunk, 16).ok()?);
    }
    Some((Address::new(index, sub), data))
}

/// File-backed parameter storage. Clones cheaply; all state is the file.
#[derive(Debug, Clone)]
pub struct FileStorage {
    path: PathBuf,
}

impl FileStorage {
    /// Storage file at `path` (created on first save).
    #[must_use]
    pub const fn new(path: PathBuf) -> Self {
        Self { path }
    }

    /// Whole file, or `None` when it does not exist yet.
    fn file_text(&self) -> Result<Option<String>> {
        match fs::read_to_string(&self.path) {
            Ok(t) => Ok(Some(t)),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
            Err(e) => Err(Error::Io(format!("reading {}: {e}", self.path.display()))),
        }
    }

    /// Detected format version (1 for legacy/unmarked files), rejecting
    /// files written by an incompatible future format instead of misparsing
    /// them line by line (`docs/DESIGN-DECISIONS.md` P3-40). The marker is searched across the
    /// whole file so a hand-edited comment header (any `#` line) cannot make
    /// a valid file look like a future format (`docs/DESIGN-DECISIONS.md` #17).
    fn file_version(text: &str) -> Result<u8> {
        for line in text.lines() {
            let line = line.trim();
            if let Some(version) = line.strip_prefix(VERSION_MARKER) {
                return match version {
                    "1" => Ok(1),
                    "2" => Ok(2),
                    _ => Err(Error::Other(
                        "unsupported storage file version (expected v1 or v2)",
                    )),
                };
            }
        }
        // No version marker at all: either an old file or one whose header
        // was edited; parse it as v1.
        Ok(1)
    }

    /// Parses entries, returning the good ones plus the number of corrupt
    /// lines. Corrupt lines are still skipped individually (one bad row must
    /// not hide the good ones, `docs/DESIGN-DECISIONS.md` P3-40) but the count is propagated
    /// so corruption is never silent (`docs/DESIGN-DECISIONS.md` G7). For v2 files the
    /// trailing CRC16 must match the canonical text of the preceding fields
    /// (CO_storageLinux.c:78-100 uses the same `crc16_ccitt(…, 0)`).
    fn parse_entries(text: &str, version: u8) -> (Vec<(Address, Vec<u8>)>, usize) {
        let mut out = Vec::new();
        let mut corrupt = 0usize;
        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let tokens: Vec<&str> = line.split_whitespace().collect();
            let (fields, expected_crc) = if version >= 2 {
                let Some((last, head)) = tokens.split_last() else {
                    corrupt += 1;
                    continue;
                };
                if last.len() != 4 || !last.chars().all(|c| c.is_ascii_hexdigit()) {
                    corrupt += 1;
                    continue;
                }
                let crc = u16::from_str_radix(last, 16).expect("validated hex digits above");
                (head, Some(crc))
            } else {
                (tokens.as_slice(), None)
            };
            let Some(entry) = parse_line(fields) else {
                corrupt += 1;
                continue;
            };
            if let Some(expected) = expected_crc {
                let canonical = fields.join(" ");
                if crc16(canonical.as_bytes()) != expected {
                    corrupt += 1;
                    continue;
                }
            }
            out.push(entry);
        }
        (out, corrupt)
    }

    fn write_entries(&self, entries: &[(Address, &[u8])]) -> Result<()> {
        // docs/DESIGN-DECISIONS.md 七F7: `parse_line` refuses `len > MAX_PARAM_LEN`, so a
        // larger persistent entry written out here could never be read
        // back — the post-write verification below would fail the whole
        // save with an opaque "corrupt line" error. Reject it up front with
        // the real reason.
        for (addr, bytes) in entries {
            if bytes.len() > MAX_PARAM_LEN {
                return Err(Error::Io(format!(
                    "persistent entry {:#06x}:{} is {} bytes, exceeding the \
                     {MAX_PARAM_LEN} byte storage limit",
                    addr.index,
                    addr.subindex,
                    bytes.len()
                )));
            }
        }
        let mut out = format!("{VERSION_MARKER}{FORMAT_VERSION}\n");
        for (addr, bytes) in entries {
            let mut tokens: Vec<String> = vec![
                format!("{:04x}", addr.index),
                format!("{:03}", addr.subindex),
                format!("{:02}", bytes.len()),
            ];
            tokens.extend(bytes.iter().map(|b| format!("{b:02x}")));
            let canonical = tokens.join(" ");
            let crc = crc16(canonical.as_bytes());
            out.push_str(&format!("{canonical} {crc:04x}\n"));
        }
        let parent = self.path.parent();
        if parent.is_some_and(|p| !p.as_os_str().is_empty()) {
            fs::create_dir_all(parent.unwrap())
                .map_err(|e| Error::Io(format!("creating {}: {e}", parent.unwrap().display())))?;
        }
        // Write to a sibling temp file and atomically rename over the
        // target (CANopenNode CO_storageLinux.c) so a crash mid-write can
        // never leave a truncated parameter file behind. The temp name is
        // process-unique AND per-call unique: `FileStorage` is Clone-able
        // and savers may run on multiple threads, so a bare pid would let
        // two same-process instances clobber each other's staging file
        // (docs/DESIGN-DECISIONS.md #16).
        static TMP_SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let seq = TMP_SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let tmp = self
            .path
            .with_extension(format!("tmp.{}.{}", std::process::id(), seq));
        fs::write(&tmp, out).map_err(|e| Error::Io(format!("writing {}: {e}", tmp.display())))?;
        // Flush to stable storage before the rename closes the power-loss
        // window where the rename is durable but the data is not
        // (CANopenNode relies on ext4 heuristics here; we do better,
        // docs/DESIGN-DECISIONS.md P3-41).
        fs::File::open(&tmp)
            .and_then(|f| f.sync_all())
            .map_err(|e| Error::Io(format!("syncing {}: {e}", tmp.display())))?;
        // Verify what landed on disk before replacing the live file
        // (CO_storageLinux.c:78-100): a truncated or garbled temp must
        // never be renamed over a good parameter file (`docs/DESIGN-DECISIONS.md` G7).
        let written = match fs::read_to_string(&tmp) {
            Ok(t) => t,
            Err(e) => {
                let _ = fs::remove_file(&tmp);
                return Err(Error::Io(format!("reading back {}: {e}", tmp.display())));
            }
        };
        let (verified, corrupt) = Self::parse_entries(&written, FORMAT_VERSION);
        if corrupt != 0 || verified.len() != entries.len() {
            let _ = fs::remove_file(&tmp);
            return Err(Error::Io(format!(
                "storage write verification failed for {}: {corrupt} corrupt line(s), {} of {} entries readable",
                self.path.display(),
                verified.len(),
                entries.len()
            )));
        }
        fs::rename(&tmp, &self.path)
            .map_err(|e| Error::Io(format!("renaming {}: {e}", self.path.display())))?;
        let parent = self.path.parent().filter(|p| !p.as_os_str().is_empty());
        let dir = parent.unwrap_or_else(|| std::path::Path::new("."));
        fs::File::open(dir)
            .and_then(|f| f.sync_all())
            .map_err(|e| Error::Io(format!("syncing directory {}: {e}", dir.display())))?;
        Ok(())
    }
}

impl ParameterStorage for FileStorage {
    fn save(&self, od: &ObjectDictionary) -> Result<()> {
        // docs/DESIGN-DECISIONS.md M8: 条目编码失败（如取值越界 `encode_as` 报错）曾通过
        // `.ok()?` 静默从保存集合跳过，但 `save()` 仍返回 `Ok(())`，重载时
        // 该对象回到默认值形成隐蔽数据丢失。改为与 `load`/`reset` 一致：
        // 记录首个编码错误并在最后返回，其余条目照常编码。
        let mut entries: Vec<(Address, Vec<u8>)> = Vec::new();
        let mut first_err: Option<Error> = None;
        for (addr, entry) in od.iter().filter(|(_, e)| e.persistent) {
            let value = match entry.raw_read(*addr) {
                Ok(v) => v,
                Err(e) => {
                    if first_err.is_none() {
                        first_err = Some(e);
                    }
                    continue;
                }
            };
            let buf = match entry.data_type.fixed_size() {
                Some(n) => {
                    // Encode at the DECLARED width so small/negative
                    // values survive the round trip: `decode_le`
                    // requires exactly `fixed_size()` bytes on
                    // load, and minimal-width encoding of e.g.
                    // `Unsigned16 = 0` produced 1 byte — silently
                    // never restored (`docs/DESIGN-DECISIONS.md` P2-1).
                    let mut buf = vec![0u8; n];
                    match value.encode_as(entry.data_type, &mut buf) {
                        Ok(_) => buf,
                        Err(e) => {
                            if first_err.is_none() {
                                first_err = Some(e);
                            }
                            continue;
                        }
                    }
                }
                None => {
                    // Variable-length types persist in the *entry's* wire
                    // form, mirroring the fixed-size branch: a
                    // `Value::Str` stored under `UnicodeString` must be
                    // written as UTF-16LE (CiA 306 / python-canopen), not
                    // as the value's minimal UTF-8 encoding — otherwise a
                    // reload turns `Str` into `Bytes` and every SDO upload
                    // corrupts non-ASCII text (docs/DESIGN-DECISIONS.md 八M5, cf. L7).
                    let n = value.encoded_len_as(entry.data_type);
                    let mut buf = vec![0u8; n];
                    match value.encode_as(entry.data_type, &mut buf) {
                        Ok(written) => {
                            buf.truncate(written);
                            buf
                        }
                        Err(e) => {
                            if first_err.is_none() {
                                first_err = Some(e);
                            }
                            continue;
                        }
                    }
                }
            };
            entries.push((*addr, buf));
        }
        if let Some(e) = first_err {
            return Err(e);
        }
        self.write_entries(
            &entries
                .iter()
                .map(|(a, b)| (*a, b.as_slice()))
                .collect::<Vec<_>>(),
        )
    }

    fn load(&self, od: &mut ObjectDictionary) -> Result<()> {
        let Some(text) = self.file_text()? else {
            return Ok(()); // no store yet: defaults stay
        };
        let version = Self::file_version(&text)?;
        let (entries, corrupt) = Self::parse_entries(&text, version);
        // A single out-of-range/stale value must not abort the whole load —
        // remember the first failure and keep restoring the rest, matching
        // `reset()`'s strategy (`docs/DESIGN-DECISIONS.md` #18).
        let mut first_err: Option<Error> = None;
        let mut stale = 0usize;
        for (addr, bytes) in entries {
            let Some(entry) = od.entry(addr).cloned() else {
                continue; // object vanished from the dictionary since save
            };
            match Value::decode_le(entry.data_type, &bytes) {
                Ok(value) => match od.set_value(addr, value) {
                    Ok(()) => {}
                    Err(e) if first_err.is_none() => first_err = Some(e),
                    Err(_) => {}
                },
                Err(_) => {
                    // docs/DESIGN-DECISIONS.md HOST-02: a mismatch means the saved line is
                    // *stale* (the entry's type/width changed since the
                    // store). Skipping it silently contradicted the module's
                    // "corruption is reported, never silent" promise, so it is
                    // counted and reported below.
                    stale += 1;
                }
            }
        }
        // Corruption is reported, never silent (`docs/DESIGN-DECISIONS.md` G7) — the good
        // entries above are still applied.
        if corrupt != 0 || stale != 0 {
            // docs/DESIGN-DECISIONS.md HOST-07: keep the per-entry `set_value` failure (the one
            // class that says the *value* was bad, not the line's syntax) in
            // the message instead of discarding it behind a line count.
            let detail = first_err
                .as_ref()
                .map_or_else(String::new, |e| format!("; first rejected entry: {e}"));
            return Err(Error::Io(format!(
                "parameter storage {}: {corrupt} corrupt line(s) and {stale} stale entr{} skipped{detail}",
                self.path.display(),
                if stale == 1 { "y" } else { "ies" },
            )));
        }
        match first_err {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    /// Restores all groups from the single backing file: per-group files are
    /// a future refinement; group granularity is passed through for
    /// interface compatibility (`docs/DESIGN-DECISIONS.md` P2-13).
    fn reset(
        &self,
        od: &mut ObjectDictionary,
        defaults: &ObjectDictionary,
        _group: u8,
    ) -> Result<()> {
        // Revert persistent entries to their construction-time values. A
        // failing entry no longer aborts the loop mid-way leaving a
        // half-reset OD plus a stale store: we keep going, remember the
        // first failure and *still* clear the stored set afterwards so a
        // subsequent load cannot re-apply old parameters on top of the
        // partial reset (docs/DESIGN-DECISIONS.md P3-55).
        let mut first_err: Option<Error> = None;
        for (addr, default_entry) in defaults.iter() {
            let Some(current) = od.entry(*addr) else {
                continue;
            };
            if !current.persistent {
                continue;
            }
            match default_entry
                .raw_read(*addr)
                .and_then(|v| od.set_value(*addr, v))
            {
                Ok(()) => {}
                Err(e) if first_err.is_none() => first_err = Some(e),
                Err(_) => {}
            }
        }
        // …clear the stored set so a later load is a no-op until save…
        let clear_result = self.write_entries(&[]);
        // …and surface the first restore failure, if any.
        first_err.map_or(clear_result, Err)
    }
}

/// Convenience constructor mirroring CANopenLinux's `/etc` convention.
#[must_use]
pub fn in_etc(node_id: u8) -> FileStorage {
    FileStorage::new(Path::new("/etc/canopen").join(format!("node-{node_id}-params.txt")))
}
