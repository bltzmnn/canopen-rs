# AGENTS.md

Rust CANopen stack (CiA 301/305/306), Linux/SocketCAN. Workspace of 4 crates:
`core` (sans-I/O protocol core), `canopen-402` (CiA 402 drive, sans-I/O),
`host` (blocking + tokio transports, EDS, CLI glue), `cli` (`canopen` binary).
MSRV 1.75, edition 2021.

## Commands (exact)

- Build: `cargo build --workspace`
- Full test (no bus needed, 203 tests): `cargo test --workspace`
- Per crate/test: `cargo test -p canopen-core --lib sdo` ; `cargo test -p canopen-host --test vcan`
- Wire-format oracle vs python-canopen (CI runs it): `python3 tools/interop/python_canopen_oracle.py`
  — requires `pip install canopen` first. Exits non-zero on any mismatch.
- On-bus integration: `sudo tools/vcan_setup.sh` (brings up vcan0) then
  `cargo test -p canopen-host --test vcan -- --nocapture`
- Async/host tokio feature gate: build/test with `--features tokio`
- **Quality gate (CI enforces, do not break):** `cargo clippy --workspace --all-targets -- -D warnings`
  plus `cargo fmt --all -- --check` and `cargo doc --no-deps` must be warning-free.

## Architecture facts an agent would miss

- **`core` is strictly sans-I/O**: no network, no threads, no clock reads. All time
  is passed in by the caller as milliseconds (`node.poll(now_ms)`, `node.on_frame(cob_id, data) -> Option<TxFrame>`). Do not add `std::time`/`Instant` or I/O in `core`.
- **`TxFrame` = `cob_id: u16` + ≤8 data bytes.** SDO channels are `0x600+node` (rx) / `0x580+node` (tx).
- **CAN FD is permanently unsupported** (classic CAN only, 8-byte max). Do not add FD paths.
- **`docs/IMPLEMENTATION.md` is the authoritative spec** ("single source of truth"). Keep it in sync when you change behavior; CiA spec wins over it if they conflict.
- Reference implementations live in `ref/`: `ref/canopen` (python-canopen) and
  `ref/CANopenLinux/CANopenNode`. When unsure about CiA semantics, diff against these rather than guessing.
- `canopen-402` state machine uses a **compile-time BFS** (`const fn`, `state.rs`) over the transition graph — extend the edge table there, don't add runtime branching.
- EDS parser keeps `$NODEID+offset` **lazy** (resolved per node id, not at parse time).

## Known bugs to consult before editing

`BUGS.md` tracks verified bugs (each cross-checked against python-canopen/CANopenNode).
Highlights that bite during routine edits:
- `datatypes.rs` `unsigned_width` (C1) and `sdo/client.rs` abort handling (M1/M10) — touched by any SDO/value work.
- `node.rs` heartbeat is emitted during `Initialising` (C2) and `emcy.rs` 0x1003 clear-on-zero is missing (M4) — touched by NMT/EMCY edits.
- `host/src/transport.rs`/`async_transport.rs` read 8-byte padded buffers but must honor received **frame length** (C6); short frames must be rejected.
- `canopen-402` fault-reset needs a `0x00`→`0x80` edge (C4) and lacks transitions 5/9 (M7).

## Conventions / gotchas

- `#![deny(unsafe_code)]` is set for `core` and `host`; `missing_docs` warns. Public API must stay documented or clippy/doc CI fails.
- `NodeId` valid range is `1..=127`; `0xFF` is reserved (LSS unconfigured only).
- Golden-byte tests + the python-canopen oracle are the regression net — if you change
  any codec, re-run both `cargo test --workspace` and the oracle script.
- `host` transport is duplicated between `transport.rs` (blocking) and `async_transport.rs`
  (tokio): fix both paths, they have already drifted once (see BUGS.md M2).
