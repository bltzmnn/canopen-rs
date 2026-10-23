# 验证记录

> 当前状态：未修复行为缺陷 **0**，文档/注释缺陷 **0**。
> 设计取舍与已排除误报见 [`DESIGN-DECISIONS.md`](DESIGN-DECISIONS.md)。

## 门禁（2026-10-23）

- `cargo fmt --all -- --check`
- `cargo clippy --workspace --all-targets -- -D warnings`
- `cargo clippy -p canopen-host --all-targets --features tokio -- -D warnings`
- `cargo test --workspace`（vcan 2 例按约定 ignored）
- `cargo test -p canopen-host --features tokio`
- `cargo doc --no-deps`
- `python3 tools/interop/python_canopen_oracle.py`：11/11

全部通过；最近一次全量测试为 0 failed。

## 实板验证

两块 T536：`.120 can0` ↔ `.63 can1`，经典 CAN，500 kbit/s。

- 双板 `canopen-audit` 探针：EDS/EMC/NMT/PDO/LSS/GTW 全项 PASS。
- 总线级：EMC-20（Stop→Start 后复位帧）、LSS-13（pending 后 0x4C 静默）、
  LSS-14（丢第 6 帧后第一次重试恢复 0x4F）均 PASS。
- NMT-16：Start 先到时 `on_frame` 返回 boot-up `0x705 [00]`，0x1016 消费者可武装。

详细结果与日志：

- [`../tests/interop/RESULTS-2026-10-23-fixes.md`](../tests/interop/RESULTS-2026-10-23-fixes.md)
- [`../tests/interop/logs-2026-10-23-fixes/`](../tests/interop/logs-2026-10-23-fixes/)
