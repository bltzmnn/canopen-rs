# AGENTS.md

## 每次任务前必读
- **开始任何编辑/修复任务前，先阅读 `docs/DESIGN-DECISIONS.md` 与 `docs/VERIFICATION.md`**：前者记录设计取舍、参考无锚点裁决与已排除误报，后者记录当前门禁和实板验证；当前未修复 bug 为 0。不要重新创建旧 bug 台账，历史变更见 git。
- 修复/调整行为后同步相关设计文档与验证记录，并保持 `cargo test`/`clippy -D warnings`/`cargo fmt --check`/python oracle 四类门禁全绿。

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

## Known decisions to consult before editing

`docs/DESIGN-DECISIONS.md` 记录设计取舍、参考无锚点裁决与已排除误报；`docs/VERIFICATION.md` 记录门禁与实板验证。编辑相关模块前先读它们，避免重新引入已裁决差异或重复调查已知误报。

## Conventions / gotchas

- `#![deny(unsafe_code)]` is set for `core` and `host`; `missing_docs` warns. Public API must stay documented or clippy/doc CI fails.
- `NodeId` valid range is `1..=127`; `0xFF` is reserved (LSS unconfigured only).
- Golden-byte tests + the python-canopen oracle are the regression net — if you change
  any codec, re-run both `cargo test --workspace` and the oracle script.
- `host` transport 的 SDO 事务驱动/`serve_node`/NMT 发送逻辑经 `host/src/sdo_exchange.rs`
  的宏统一为单源；改动这些逻辑只需编辑该宏，`transport.rs` 与 `async_transport.rs`
  仅保留 `send`/`recv` 原语（按 `await`/`block` 参数化）。
