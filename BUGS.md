# CANopen 协议栈 Bug 清单（仅列未修复项）

> 范围：`core`、`host`、`canopen-402`。
> 核验基线：与 python-canopen（`ref/canopen`）、CANopenNode（`ref/CANopenLinux/CANopenNode`）交叉比对，并对照当前工作区代码。
>
> **状态**：原清单全部严重项（C1–C6）、主要项（M1–M11）及次要项
> m2/m5/m6/m7/m8/m9/m11/m13/m14/m15/m16/m17/m18 均已修复并附回归测试，
> 条目随之删除，不再跟踪；修复详情见 git 历史与本文件旧版。

## 未修复项

### m12 — transport 阻塞/异步剩余重复
- **位置**：`host/src/transport.rs` ↔ `host/src/async_transport.rs`
- **现状**：SDO 事务核心已抽取到 `host/src/sdo_exchange.rs`（客户端生命周期、队列管理、协议错误 ABORT 发送、终态解释单一实现，m12 主体完成）；两传输层各自仅保留 send/recv 原语 + `drive_exchange` 循环 + `serve_node`。这两处仍为平行实现（async 需 `.await`/timeout 包装，无法无代价合并）。
- **后续**：如再发生漂移，考虑宏或 `Future` 参数化生成 serve 循环。

## 撤销项（参考比对非 bug，不修复）
C3 homing 掩码 · m1 is_sync bit31 · m3 Stopped 态发 EMCY（CiA 301 允许 PreOp/Op/Stopped 三态发送）。

## 质量门禁（最近一次全绿）
`cargo test --workspace` 223 通过 · clippy `-D warnings`（含 tokio feature）0 告警 ·
fmt/doc 干净 · oracle 脚本 11/11。
