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
        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let mut parts = line.split_whitespace();
            let Some(index) = parts.next().and_then(|p| u16::from_str_radix(p, 16).ok()) else {
                continue;
            };
            let Some(sub) = parts.next().and_then(|p| p.parse::<u8>().ok()) else {
                continue;
            };
            let Some(len) = parts.next().and_then(|p| p.parse::<usize>().ok()) else {
                continue;
            };
            let mut data = Vec::with_capacity(len);
            for chunk in parts.take(len) {
                match u8::from_str_radix(chunk, 16) {
                    Ok(b) => data.push(b),
                    Err(_) => return Err(Error::Other("corrupt storage line")),
                }
            }
            if data.len() != len {
                return Err(Error::Other("storage line shorter than declared"));
            }
            out.push((Address::new(index, sub), data));
        }
        Ok(out)
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
        // never leave a truncated parameter file behind.
        let tmp = self.path.with_extension("tmp");
        fs::write(&tmp, out).map_err(|e| Error::Io(format!("writing {}: {e}", tmp.display())))?;
        fs::rename(&tmp, &self.path)
            .map_err(|e| Error::Io(format!("renaming {}: {e}", self.path.display())))
    }
}

impl ParameterStorage for FileStorage {
    fn save(&self, od: &ObjectDictionary) -> Result<()> {
        let entries: Vec<(Address, Vec<u8>)> = od
            .iter()
            .filter(|(_, e)| e.persistent)
            .filter_map(|(addr, entry)| {
                entry.raw_read(*addr).ok().map(|value| {
                    let n = value.encoded_len();
                    let mut buf = vec![0u8; n];
                    let written = value.encode_le(&mut buf).unwrap_or(0);
                    buf.truncate(written);
                    (*addr, buf)
                })
            })
            .collect();
        self.write_entries(
            &entries
                .iter()
                .map(|(a, b)| (*a, b.as_slice()))
                .collect::<Vec<_>>(),
        )
    }

    fn load(&self, od: &mut ObjectDictionary) -> Result<()> {
        for (addr, bytes) in self.read_entries()? {
            let Some(entry) = od.entry(addr).cloned() else {
                continue; // object vanished from the dictionary since save
            };
            match Value::decode_le(entry.data_type, &bytes) {
                Ok(value) => od.set_value(addr, value)?,
                Err(_) => continue, // stale entry with mismatched type/length
            }
        }
        Ok(())
    }

    fn reset(&self, od: &mut ObjectDictionary, defaults: &ObjectDictionary) -> Result<()> {
        // Revert persistent entries to their construction-time values…
        for (addr, default_entry) in defaults.iter() {
            let Some(current) = od.entry(*addr) else {
                continue;
            };
            if !current.persistent {
                continue;
            }
            let value = default_entry.raw_read(*addr)?;
            od.set_value(*addr, value)?;
        }
        // …and clear the stored set so a later load is a no-op until save.
        self.write_entries(&[])
    }
}

/// Convenience constructor mirroring CANopenLinux's `/etc` convention.
#[must_use]
pub fn in_etc(node_id: u8) -> FileStorage {
    FileStorage::new(Path::new("/etc/canopen").join(format!("node-{node_id}-params.txt")))
}
