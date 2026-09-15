# AGENTS.md

## 每次任务前必读
- **开始任何编辑/修复任务前，先重新阅读 `BUGS.md`**：它是当前权威的、随修复持续更新的已核实 bug 清单。不要依赖记忆或上一轮会话中的 bug 状态——已修复项会随时从清单删除，未修复项也可能新增。先读再动手，避免对已修复项重复劳动或误判未修复项。
- 修复 bug 后**同步更新 `BUGS.md`**（删除/标注已修复条目），并保持 `cargo test`/`clippy -D warnings`/`cargo fmt --check`/python oracle 四类门禁全绿。

Rust CANopen stack (CiA 301/305/306), Linux/SocketCAN. Workspace of 3 crates:
`core` (sans-I/O protocol core), `host` (blocking + tokio transports, EDS,
CLI glue), `cli` (`canopen` binary). CiA 402 is **out of scope** — the
former `canopen-402` crate has been removed by decision; do not re-add it.
MSRV 1.75, edition 2021.

## Commands (exact)

- Build: `cargo build --workspace`
- Full test (no bus needed): `cargo test --workspace`
- Per crate/test: `cargo test -p canopen-core --lib sdo` ; `cargo test -p canopen-host --test vcan -- --ignored`
- Wire-format oracle vs python-canopen (CI runs it): `python3 tools/interop/python_canopen_oracle.py`
  — requires `pip install canopen` first. Exits non-zero on any mismatch.
- On-bus integration: `sudo tools/vcan_setup.sh` (brings up vcan0) then
  `cargo test -p canopen-host --test vcan -- --ignored --nocapture`
- Async/host tokio feature gate: build/test with `--features tokio`
- **Quality gate (CI enforces, do not break):** `cargo clippy --workspace --all-targets -- -D warnings`
  plus `cargo fmt --all -- --check` and `cargo doc --no-deps` must be warning-free.

## Architecture facts an agent would miss

- **`core` is strictly sans-I/O**: no network, no threads, no clock reads. All time
  is passed in by the caller as milliseconds (`node.poll(now_ms)`, `node.on_frame(cob_id, data) -> Vec<TxFrame>`). Do not add `std::time`/`Instant` or I/O in `core`.
- **`TxFrame` = `cob_id: u16` + ≤8 data bytes.** SDO channels are `0x600+node` (rx) / `0x580+node` (tx).
- **CAN FD is permanently unsupported** (classic CAN only, 8-byte max). Do not add FD paths.
- **`docs/IMPLEMENTATION.md` is the authoritative spec** ("single source of truth"). Keep it in sync when you change behavior; CiA spec wins over it if they conflict.
- Reference implementations live in `ref/`: `ref/canopen` (python-canopen) and
  `ref/CANopenLinux/CANopenNode`. When unsure about CiA semantics, diff against these rather than guessing.
- EDS parser keeps `$NODEID+offset` **lazy** (resolved per node id, not at parse time).

## Known bugs to consult before editing

`BUGS.md` 是权威的已核实 bug 清单（仅列未修复项，随修复持续更新）。编辑相关模块前先读它，避免与已知未修复项冲突或重复劳动。

## Conventions / gotchas

- `#![deny(unsafe_code)]` is set for `core` and `host`; `missing_docs` warns. Public API must stay documented or clippy/doc CI fails.
- `NodeId` valid range is `1..=127`; `0xFF` is reserved (LSS unconfigured only).
- Golden-byte tests + the python-canopen oracle are the regression net — if you change
  any codec, re-run both `cargo test --workspace` and the oracle script.
- `host` transport 的 SDO 事务驱动/`serve_node`/NMT 发送逻辑经 `host/src/sdo_exchange.rs`
  的宏统一为单源；改动这些逻辑只需编辑该宏，`transport.rs` 与 `async_transport.rs`
  仅保留 `send`/`recv` 原语（按 `await`/`block` 参数化）。
