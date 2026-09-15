//! File-backed `canopen_core::storage::ParameterStorage`
//! implementation — the Linux counterpart of CANopenNode's
//! `CO_storageLinux.c`.
//!
//! Format: line-based, one persisted entry per line,
//! `<index-hex> <sub-dec> <len> <hex bytes…>`, preceded by a `v1` marker.
//! Dependency-free by design (no serde).

use std::fs;
use std::path::{Path, PathBuf};

use canopen_core::datatypes::Value;
use canopen_core::object_dictionary::ObjectDictionary;
use canopen_core::storage::ParameterStorage;
use canopen_core::types::{Address, Error, Result};

/// 单条参数的最大字节数上限（BUGS.md M7）：`read_entries` 曾用文件里的
/// `len` 直接 `with_capacity(len)`，损坏/恶意的存储文件传入超大 `len`
/// （如 `4294967295`）会尝试分配约 4GB。超过此上限的行按坏行跳过。
const MAX_PARAM_LEN: usize = 8192;

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

    fn read_entries(&self) -> Result<Vec<(Address, Vec<u8>)>> {
        let text = match fs::read_to_string(&self.path) {
            Ok(t) => t,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Vec::new()),
            Err(e) => return Err(Error::Io(format!("reading {}: {e}", self.path.display()))),
        };
        let mut out = Vec::new();
        // Uniform per-line tolerance (BUGS.md P3-40): a corrupt field skips
        // exactly that line — whether the damage is in the address fields or
        // the data bytes — so one bad row can never hide all good ones.
        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let mut parts = line.split_whitespace();
            let (Some(index), Some(sub), Some(len)) = (
                parts.next().and_then(|p| u16::from_str_radix(p, 16).ok()),
                parts.next().and_then(|p| p.parse::<u8>().ok()),
                parts.next().and_then(|p| p.parse::<usize>().ok()),
            ) else {
                continue; // malformed header fields: skip the line
            };
            // BUGS.md M7: 损坏/恶意的 `len` 会导致无上限分配（DoS/OOM）。
            // 超过上限的行按坏行跳过，既不预分配也不解析其数据字节。
            if len > MAX_PARAM_LEN {
                continue;
            }
            let mut data = Vec::with_capacity(len.min(MAX_PARAM_LEN));
            let mut bad = false;
            for chunk in parts.by_ref().take(len) {
                match u8::from_str_radix(chunk, 16) {
                    Ok(b) => data.push(b),
                    Err(_) => {
                        bad = true;
                        break;
                    }
                }
            }
            // BUGS.md H2: a line carrying MORE hex tokens than `len` is
            // corrupt — `parts.take(len)` would silently drop the excess, so
            // a damaged row whose first `len` bytes happened to be valid was
            // loaded as a clean entry. Reject it like any other bad line.
            if bad || data.len() != len || parts.next().is_some() {
                continue; // corrupt/short data body: skip the line
            }
            out.push((Address::new(index, sub), data));
        }
        Ok(out)
    }

    /// Rejects files written by an incompatible future format instead of
    /// misparsing them line by line (`BUGS.md` P3-40). The version marker is
    /// searched across the whole file so a hand-edited comment header (any
    /// `#` line) cannot make a valid v1 file look like a future format
    /// (`BUGS.md` #17); `read_entries` tolerates arbitrary comment lines.
    fn check_version(&self) -> Result<()> {
        let text = match fs::read_to_string(&self.path) {
            Ok(t) => t,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(()),
            Err(e) => return Err(Error::Io(format!("reading {}: {e}", self.path.display()))),
        };
        for line in text.lines() {
            let line = line.trim();
            if let Some(version) = line.strip_prefix("# canopen-rs parameter storage v") {
                if version != "1" {
                    // `Error::Other` carries a static message (core types).
                    return Err(Error::Other(
                        "unsupported storage file version (expected v1)",
                    ));
                }
                return Ok(());
            }
        }
        // No version marker at all: either an old file or one whose header
        // was edited; let the tolerant line parser deal with it.
        Ok(())
    }

    fn write_entries(&self, entries: &[(Address, &[u8])]) -> Result<()> {
        let mut out = String::from("# canopen-rs parameter storage v1\n");
        for (addr, bytes) in entries {
            let hex_bytes = bytes
                .iter()
                .map(|b| format!("{b:02x}"))
                .collect::<Vec<_>>()
                .join(" ");
            out.push_str(&format!(
                "{:04x} {:03} {:02} {hex_bytes}\n",
                addr.index,
                addr.subindex,
                bytes.len()
            ));
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
        // (BUGS.md #16).
        static TMP_SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let seq = TMP_SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let tmp = self
            .path
            .with_extension(format!("tmp.{}.{}", std::process::id(), seq));
        fs::write(&tmp, out).map_err(|e| Error::Io(format!("writing {}: {e}", tmp.display())))?;
        // Flush to stable storage before the rename closes the power-loss
        // window where the rename is durable but the data is not
        // (CANopenNode relies on ext4 heuristics here; we do better,
        // BUGS.md P3-41).
        if let Ok(f) = fs::File::open(&tmp) {
            let _ = f.sync_all();
        }
        fs::rename(&tmp, &self.path)
            .map_err(|e| Error::Io(format!("renaming {}: {e}", self.path.display())))
    }
}

impl ParameterStorage for FileStorage {
    fn save(&self, od: &ObjectDictionary) -> Result<()> {
        // BUGS.md M8: 条目编码失败（如取值越界 `encode_as` 报错）曾通过
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
                    // never restored (`BUGS.md` P2-1).
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
                    // corrupts non-ASCII text (BUGS.md 八M5, cf. L7).
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
        self.check_version()?;
        // A single out-of-range/stale value must not abort the whole load —
        // remember the first failure and keep restoring the rest, matching
        // `reset()`'s strategy (`BUGS.md` #18).
        let mut first_err: Option<Error> = None;
        for (addr, bytes) in self.read_entries()? {
            let Some(entry) = od.entry(addr).cloned() else {
                continue; // object vanished from the dictionary since save
            };
            match Value::decode_le(entry.data_type, &bytes) {
                Ok(value) => match od.set_value(addr, value) {
                    Ok(()) => {}
                    Err(e) if first_err.is_none() => first_err = Some(e),
                    Err(_) => {}
                },
                Err(_) => continue, // stale entry with mismatched type/length
            }
        }
        match first_err {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    /// Restores all groups from the single backing file: per-group files are
    /// a future refinement; group granularity is passed through for
    /// interface compatibility (`BUGS.md` P2-13).
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
        // partial reset (BUGS.md P3-55).
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
