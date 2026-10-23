# 行为修复实板验证（2026-10-23）

## 环境

- 两块 T536 开发板：`.120 can0` ↔ `.63 can1`，经典 CAN，500 kbit/s。
- Rust 探针：`canopen-audit` aarch64-musl，SHA-256
 `ddf2af70679476972b8ee0627562bc2268149feebe174bf36a06197ce291adb5`。
- 从站：`rust_node` aarch64-musl，SHA-256
 `5f01119234e307ed5ef8baf4a9b9f0e5aedee47e0d25fca3c5f50a8c8bc1490c`。
- 基线提交：`7dd373a`（工作区含本轮修复，未提交）。

## 门禁（主机）

- `cargo test --workspace`：**0 failed**；`vcan` 2 例按既有约定 ignored。
- `cargo test -p canopen-host --features tokio`：0 failed。
- `cargo clippy --workspace --all-targets -- -D warnings`：通过。
- `cargo clippy -p canopen-host --all-targets --features tokio -- -D warnings`：通过。
- `cargo fmt --all -- --check`：通过。
- `cargo doc --no-deps`：通过。
- `python3 tools/interop/python_canopen_oracle.py`：**11/11**。

## 双板公共 API 探针

两板（`.120`、`.63`）运行同一 aarch64-musl 探针，全部 PASS：

- EDS-30：A1/A1b/A2/B1/B3/B3p/B3a 全部保留显式 sub0；值、类型、权限不再被头部键改写。
- EDS-31：`CompactSubObj` 放在内联键之后仍正确展开为 `Uint(3)` + 3 个模板成员。
- EDS-32/33：`$NODEID±offset` 的 `ParameterValue`/`DefaultValue` 均物化为 `Uint(517)`，无虚假 warning。
- EDS-34：lenient TIME 零值为 6 字节且可编码。
- EDS-35：`héllo` 的 UNICODE_STRING 为 10 字节，无尾随 NUL 码元。
- EDS-36：INTEGER64 `$NODEID` 溢出在运行时返回 `ValueOutOfRange`，不再静默截断。
- EDS-37：INTEGER24 非法默认值产生 warning。
- EMC-19：队列非空且 0x1014 禁用时仍产生心跳。
- EMC-20：Stop→Start 后恢复流量产生 0x8250 复位帧（不再永久卡住）。
- NMT-16：Initialising 期收到 Start 时 `on_frame` 直接返回 boot-up `0x705 [00]`，0x1016 消费者随后能被武装并报 `Lost`；之后的 `boot` 为 `None` 属预期。
- NMT-17：错误驱动的 Operational 入口与 NMT Start 入口都会触发事件型 TPDO。
- NMT-18：0x1016 sub-index 不再按 node-id 排序秩解释。
- PDO-30：python 写序“条目先写、计数最后写”在计数写处以 `0x0604_0042` 拒绝。
- PDO-31：先配 TPDO2 再配 TPDO1，总线顺序仍为 `0x181, 0x281`。
- PDO-32：缺失目标保持 `0x0602`；不可映射/重复映射为 `0x0604_0041`；位宽错误保持 `0x0607_0010`。
- LSS-13：pending node-id 已设置时 0x4C 不应答 0x50。
- LSS-14：丢第 6 帧后，第一次完整重试即恢复 0x4F。
- GTW-30：`4294967296` 饱和到 `0xFFFFFFFF`；`-1` 回绕到 `0xFFFFFFFF`；20 位 `u64::MAX` token 超长不派发、回 `ERROR:101`。

探针日志：

- `/tmp/canopen-interop/audit_120_final.log`
- `/tmp/canopen-interop/audit_63_final.log`

仓内归档：`tests/interop/logs-2026-10-23-fixes/audit_120_final.log`、`audit_63_final.log`。

## 总线级验证

在两板总线上以 python-canopen 主站驱动 `rust_node`：

| 用例 | 结果 | 证据 |
| --- | --- | --- |
| EMC-20 Stop→Start 后复位帧 | **PASS** | `/tmp/canopen-interop/logs/fix_emc20_bus.log`：`first=5082101710270000 reset=0000001710270000` |
| LSS-13 pending 后 0x4C | **PASS** | `/tmp/canopen-interop/logs/fix_lss13_bus.log`：`response=None` |
| LSS-14 丢第 6 帧后第一次重试 | **PASS** | `/tmp/canopen-interop/logs/fix_lss14_bus.log`：`retry_ok=True` |

仓内归档：`tests/interop/logs-2026-10-23-fixes/fix_*_bus.log`。

EMC-20 日志末尾的 `error_register=0x10` 是复位后测试窗口内下一次 RPDO 死线再次超时所致；复位帧本身已按 `00000017...` 发出，说明旧闩锁已清除。
