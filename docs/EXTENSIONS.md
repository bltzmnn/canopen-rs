# 后续拓展规划（EXTENSIONS）

本文档是一期协议栈（P1–P9，见 `IMPLEMENTATION.md`）之后的拓展路线图：范围
决策、参考项目调研结论、分批次实施计划与验收出口。

> **架构约束**：本栈**仅支持经典 CAN**（11-bit ID、8 字节帧上限）。CAN FD
> 永久不在支持范围内——`TxFrame.data: [u8; 8]` 是最终形态，不是待办。

---

## 目录

1. [范围声明](#1-范围声明)
2. [调研结论摘要](#2-调研结论摘要)
3. [批次规划](#3-批次规划)
4. [验收出口](#4-验收出口)
5. [批次 D：python 测试套件移植（pyport）](#5-批次-dpython-测试套件移植pyport)

---

## 1. 范围声明

基于对 `ref/` 六个开源项目的深入调研（调研方法与原始发现见 §2），后续拓展
的功能取舍如下：

| 功能 | 决定 | 理由 |
|---|---|---|
| EDS 文件导入（解析 → OD / codegen） | ✅ 已完成 | P9 交付，`host/src/eds.rs` |
| **SDO 0x1021 导入**（从运行中设备拉取 EDS → `eds::parse()`） | ✅ 批次 A | 成本极低（复用现有分段/block client + 解析器）；python-canopen `import_from_node`（eds.py:187-207）为行为参考 |
| EDS 导出（写出 `.eds` 文件） | ❌ 不做 | 无消费方；需先补 parser 的 ObjectType/SubNumber 捕获，投入产出比差。未来若需要，从解析级 `Eds` IR 导出以保留 `$NODEID`/字符串/未知键 |
| CompactSubObj 展开解析 | ✅ 批次 A（小项） | 纯导入质量问题：紧凑数组目前不会展开成子条目 |
| **CAN FD（CiA 1301）** | ❌ 永久不支持 | 调研确认三个 Rust 参考项目均无可用实现（zencan README 虚标 `[u8;8]` 硬编码；本地 CANopenNode 快照早于上游 FD 合入）；语义需查规范文本且无 oracle 可对拍，风险收益比不支持 |
| 参数持久化 Storage（0x1010/0x1011 从站侧） | ✅ 批次 A | 实际部署高频需求；CANopenNode `storage/` 提供清晰参考 |
| LSS 补缺（allnodes scan + bit-rate configure/activate） | ✅ 批次 A | 对照 `CO_LSSmaster.h:33-74` 审计的已知差距 |
| CiA 309-3 ASCII 网关（最小切片） | ✅ 批次 B | 调试/互操作利器；协议核心传输无关可直接移植 |
| CiA 303-3 LED 指示 | ✅ 批次 B（先行） | 极廉价纯状态机；309 `led` 命令的前置依赖 |
| **CiA 402 驱动轮廓** | ✅ 批次 C（新 crate） | 唯一参考 python-canopen `p402.py`；8 个已记录缺陷全部有对策（见 §2.1） |
| SRDO / GFC（CiA 304 安全数据对象） | ⏸ 观望 | 双帧反相+三定时器+配置 CRC 约 M–L 工作量；依赖毫秒级安全定时器抽象评审；仅互操作用途无认证价值 |
| CiA 315 bootloader / 固件升级 | ⏸ 观望 | 所有参考项目均无实现，greenfield，需独立预研 |
| TOML 设备配置层 | ❌ 不做 | zencan 的双后端架构（Rust codegen + 缺失的 EDS writer）是反面教材；若未来固件侧需要配置表达力（存储提示/app 回调条目/PDO 槽位数），正确形态是 **TOML→EDS→Rust 单一 IR**，而非平行后端 |

---

## 2. 调研结论摘要

### 2.1 CiA 402（python-canopen `profiles/p402.py`，581 行）

可移植资产：

- **状态字解码表** `SW_MASK`(:41-50)：8 态 mask/value 对——移植为有序
  `match`（dict 插入序的 first-match-wins 语义必须显式化并文档化优先级）
- **转移表** `TRANSITIONTABLE`(:63-87)：`(from, to) → controlword`，含 3 条
  自动迁移行（命令 0x00）——移植为 const 数组
- **间接路径解析** `next_state_indirect`(:89-104)：到 OPERATION ENABLED 的
  多跳路由（QUICK STOP ACTIVE 特殊经 SWITCH ON DISABLED）——编译期 BFS
  预计算成查找表
- **模式切换确认走 0x6061**（mode display）而非 0x6060 写值本身，
  支持位掩码来自 0x6502(:147-158)
- Homing 状态解码用 statusword bits12/13 mask `0x3400`(:186-193)

必须修正的缺陷（对应 REFERENCES.md V2/V3 编号体系）：

1. 忙等轮询 ×5 处（state setter :547、_change_state :570、op-mode :423-427、
   homing :356-364、fault-reset :298-302）→ watch channel + deadline await
2. 字符串状态名/`'UNKNOWN'` 伪态 → 穷举 enum，未知组合返回显式错误
3. op_mode setter 吞错只打日志(:429-432) → Result 全链路
4. fault-reset 内联 FIXME 承认失效(:296-297) → 真实上升沿脉冲（置 bit7、
   厂商最短延时、清除）
5. setup 校验只 warn(:269-289) 导致误配设备静默降速 SDO → `setup()` 返回
   校验报告，SDO 回退逐对象 opt-in
6. 与心跳监督完全脱钩 → HeartbeatConsumer Lost 事件立即中止进行中的
   状态迁移/homing
7. homing `_homing_status` last-match 且可能 None(:309-318) → 有序 match
8. 无类型化目标访问器 → 0x607A/0x60FF/0x6071 i32/i16 访问器 +
   `clear_targets()`

注：CANopenNode v4 本地快照**无任何 402 模块**（全树 grep 无
Statusword/0x6040），驱动侧从站实现属应用层职责——与我们分层一致。

### 2.2 CiA 309-3 ASCII 网关（CANopenNode `309/CO_gateway_ascii.{h,c}`，2419 行）

- 协议核心**传输无关**：命令进字节 FIFO（`CO_GTWA_write`, h:369）、响应出
  应用回调（h:240）；TCP/串口/unix socket 全在宿主层
  （`CO_epoll_interface.h:179-185`）
- 帧：`[<seq>] <cmd...><CR><LF>`；响应 `[<seq>] OK|value|ERROR:<code>`；
  `#` 注释；单命令在途，超时 1.2s(h:146-148)
- 命令语法（h:67-133）：`[n] <node> read 0x1017 0 i16` /
  `write ... u16 500` / `start|stop|preop|reset node|reset comm` /
  `set node|network|sdo_timeout|sdo_block`
- 错误码分段：100 语法 / 200 NMT·HB / 300 CAN / 400 PDO / 500 LSS /
  600 OOM(h:154-183)

最小切片 = read/write/NMT/set 系列（跳过 LSS 命令族——占 C 实现 ~40%——
及 log/led）。估算 600–1000 行 Rust + 测试。

### 2.3 Storage（CANopenNode `storage/CO_storage.{h,c}` + `CO_storageLinux.c`）

- 0x1010 sub1 写 `'save'`(0x65766173) / 0x1011 sub1 写 `'load'`(0x64616F6C)
  触发；存储分组按 subindex 展开(h:53-77)
- 后端函数指针注入(store/restore, h:97-98)；Linux 后端落文件
- 我们已有 master 侧 `RemoteNode::store()/restore()`（写标准魔数），批次 A
  补从站侧 trait + host 文件后端

### 2.4 LSS 差距审计（对照 `305/CO_LSSmaster.h:33-74`）

现有 `core/src/lss.rs` 已有：selective switch、configure/store/inquire、
fastscan 双端。缺口两项：

- **allnodes scan**：广播式枚举总线上所有 LSS 地址（非 fastscan 单目标）
- **bit-rate configure/activate**：`configureBitTiming` 服务的主站驱动 +
  从站应答（从站侧已有骨架未接）

### 2.5 其余观望项要点

- **SRDO**（304/CO_SRDO.c, 1010 行）：每周期双帧发送（plain + bitwise
  NOT），接收校验反相一致性与 SCT/SRVT 两定时器；0x13FF 存整配置的
  CRC16-CCITT（我们 sdo/block.rs 的 crc16 可复用）；映射条目须偶数个交替
  plain/inverted(c:541,766)。无认证声明，仅互操作。
- **303 LED**（303/CO_LEDs.c, 177 行）：红绿 LED 位型由 NMT 态/LSS 配置态/
  bus-off/RPDO 超时/SYNC 超时/HB consumer 错误/固件下载标志驱动(h:38-66)。
- **TOML 层**（zencan device_config.rs）：其价值点（persist 提示、app 回调
  条目、PDO 槽位数、内联定长字符串）均为固件侧需求；zencan 自身 EDS writer
  缺失（README 虚标）印证双后端路线的维护成本。

---

## 3. 批次规划

### 批次 A：低成本高价值

#### A1 参数持久化 Storage

- core `src/storage.rs`：
  - `StorageBackend` trait：`fn save(&mut self, group: u8, od: &ObjectDictionary) -> Result<()>` /
    `fn load(&mut self, group: u8, od: &mut ObjectDictionary) -> Result<()>`
    （或按 Address 列表的细粒度接口，实现期定）
  - OD 扩展挂载：0x1010/0x1011 各 sub 为回调条目，写入标准魔数触发
    backend；恢复默认语义（0x1011）= 重置为编译期默认再 save
  - 与 `Node` 集成：`Node::with_storage(backend)`；NMT ResetNode 时自动
    load（可选开关）
- host：文件后端 `FileStorage`（JSON 或 bincode 格式存 `(Address, Value)`
  列表，带格式版本号）

#### A2 EDS 导入增强

- `read_eds_from_device(node)`：client.rs 新增——SDO 读 0x1021 全量字节流
  （长度先读 sub0，分段/block 自动选择）→ `eds::parse()`
- CompactSubObj 展开：parser 捕获该字段后按模板展开 sub1..=N（sub0 =
  Unsigned8 count），导入保真度提升

#### A3 LSS 补缺

- allnodes scan：master 顺序探测（switch global config + inquire serial，
  逐节点 assign 临时 id 或 fastscan 连续收敛），提供 `scan_all_nodes()`
  高层 API
- bit-rate configure/activate：主站 encode/decode + 从站应答接入现有
  `LssSlave::handle`

### 批次 B：调试互操作

#### B2 CiA 303-3 LED 模块（先行）

- core `src/leds.rs`：纯 sans-I/O 位型机——输入 NMT 态/LSS 态/错误标志位/
  now_ms，输出 (绿, 红) 两 bool；CANopenNode 表驱动逻辑移植
- `Node` 集成：`poll()` 附带更新 LED 状态查询接口

#### B1 CiA 309-3 ASCII 网关（最小版）

- core `src/gateway.rs`：sans-I/O 解析状态机——字节流入 → 命令事件出；
  响应字节流出。首期命令集：
  - `[n] <node> read <idx> [sub] [type]`
  - `[n] <node> write <idx> [sub] <type> <val>`
  - `start|stop|preop|reset node|reset comm`
  - `set node <v>` / `set sdo_timeout <ms>`
  - `help`
- host：TCP 监听封装（每连接一个网关实例桥接 Network/SocketCan）+
  stdio 模式；`cocomm` 语法的集成测试客户端脚本
- 错误码：实现 100（语法）/200（NMT）/300（CAN/timeout）段；500 LSS 段随
  LSS 命令族后置

### 批次 C：CiA 402 驱动轮廓（新 crate `canopen-402`）

```
canopen-402/src/
  state.rs     # State402/TargetState/CwCommand 枚举 + SW 解码 + const TRANSITIONS 表
               # + 编译期 BFS 间接路径表 NEXT_HOP_TO_OP_ENABLED
  mode.rs      # OperationMode 枚举 + ModesSupported(0x6502) 位掩码
  homing.rs    # HomingMethod/HomingStatus(statusword bits12/13)
  machine.rs   # sans-I/O DriveStateMachine{on_statusword, step_toward, poll(now_ms)}
  client.rs    # 同步 Drive402 over Network::subscribe + RemoteNode(SDO fallback)
  async_client.rs  # tokio feature: watch channel 版 wait_state/set_target_state
```

设计要点（逐条针对 §2.1 缺陷清单）：

- 状态字解码：单函数有序 match，文档注明优先级；未知组合
  `State402::Invalid(u16)` 显式表示
- 等待原语：同步版 `poll()`/`step()` 零睡眠；async 版
  `sw_rx.wait_for(|sw| decode(sw)==target)` + `tokio::time::timeout`
- `setup() -> ConfigReport`：逐对象报告 PDO 映射存在性/mappable/访问类型，
  SDO 回退需显式 opt-in
- fault reset：置 0x80 → 延时参数化 → 清除 → 重新 enable 的完整序列
- 心跳联动：构造时接受 `&Network` 的监控句柄，Lost 即中止迁移/homing 并报
  `DriveError::HeartbeatLost`

---


### 批次 D：python 测试套件移植（pyport）

**目标**：将 `ref/canopen/test/test*.py`（15 文件，~200 条用例）系统性转换为
本工程 Rust 测试，作为与真机交叉验证（`tests/interop/TESTPLAN.md`）并行的
**进程内回归测试线**。

#### 关键基建：进程内虚拟总线

python 用 `interface="virtual"` 实现同进程双 Network 回环；我方对应方案为
测试专用 **`VirtualBus`**：master 侧客户端的 `send()` 直通
`Node::on_frame()`/`poll()`，节点输出帧回注为接收流。零 socketcan 依赖，
CI 全环境可跑。

#### 层次定义

- **A 层**：进程内虚拟总线直驱（slave = `Node`，master = `SdoClient`/API）
- **B 层**：golden 帧直接喂 sans-I/O 状态机，断言响应字节序列
- **L2 层**：host `Network` API 级（需 socketcan 接口，skip-if-missing）

#### 逐文件映射表

| Python 文件 | Rust 落点 | 层 | 可转 | 备注 |
|---|---|---|---|---|
| test_local.py (19) | `core/tests/pyport_local.rs` | A | 19 | SDO server 全语义；4 处行为差异按差异表适配 |
| test_sdo.py (45) | sdo 模块 golden 补充 + `core/tests/pyport_sdo_frames.rs` | B | ~30 | 含 block download/upload/**127 段重传逐帧序列**作断言 |
| test_nmt.py (14) | `core/tests/pyport_nmt.rs` | A | ~12 | 从站状态机+心跳部分；guarding 主站侧留 L2 |
| test_emcy.py (17) | `core/tests/pyport_emcy.rs` | A | ~14 | producer/consumer 对测；wait()/回调异常隔离需先增强 EmcyConsumer |
| test_pdo.py (8) | `core/tests/pyport_pdo.rs` | A | ~6 | 经 SDO 的 PDO 配置组；map export 属 python 内部跳过 |
| test_sync.py + test_time.py (13) | `core/tests/pyport_sync_time.rs` | A/B | ~8 | codec 部分直转；周期发送需 PeriodicSendTask（暂缓项） |
| test_lss.py (26) | `core/tests/pyport_lss_frames.rs` | B | 26 | 全命令 golden 帧（含 bit-timing/activate——覆盖 A3 特性） |
| test_eds.py 边界子集 | host `eds.rs` 测试补充 | C | ~10 | 缺 sub 触发 `0x06090011`、CompactSubObj、类型边界；夹具直接复制 sample.eds/datatypes.eds |
| test_network.py (13) | `host/tests/pyport_network.rs` | L2 | ~8 | subscribe 多订阅/check 重抛为纯逻辑可测；send_periodic 属暂缓项 |
| test_variable.py (84 行) | ⚠️ 暂缓 | C | 0 | 依赖未实现的 phys/desc/bits 分层值访问（EDS factor/unit/value_descriptions/bit_definitions 解析 + 访问器），单独立项 |
| test_od.py / test_utils.py | 跳过 | — | 0 | python OD 内部结构 |
| test_p402.py (32) | canopen-402 行为一致性对拍素材 | C | 后续 | statusword 序列 diff 素材（C 批次验收出口引用） |

**合计约 133 条新增 Rust 测试。**

#### 已知行为差异对照表（转换时按此适配断言）

| # | 蓝本行为 | 本栈行为 | 适配 |
|---|---|---|---|
| D1 | block upload 收到分段应答即报错（客户端无降级） | 自动降级分段成功取数 | 改为正向用例 |
| D2 | 空字符串读取 → abort `0x08000024` | 返回空串 | 断言改 `Value::Str("")` |
| D3 | 无默认值对象读取 → abort `0x060A0023` | OD 物化零值可读 | 断言改为读到值 |
| D4 | 加速写宽度不符 → abort `0x06070010` | 当前发 `0x06070013` | **P-c 中对齐 server 为 `0x06070010`**（主流实践；分段声明长度不符仍用 12/13） |

#### 功能缺口处置决定

| 缺口 | 处置 |
|---|---|
| EmcyConsumer wait()/多回调/回调异常隔离 | **随 P-c 实现**（~3 条用例依赖） |
| abort 码 D4 对齐 | **随 P-c 实现** |
| phys/desc/bits 分层值访问（test_variable 12 条） | 单独立项暂缓；对应范围不转换 |
| PeriodicSendTask（test_sync 周期 / send_periodic） | 单独立项暂缓；相关用例标 `#[ignore]` 注明原因 |
| 多 SDO 通道 add_sdo_channel | 同上暂缓 |

#### 执行步骤

| 步骤 | 内容 | 新增 |
|---|---|---|
| P-a | VirtualBus 基建 + test_local 全量移植 | 19 |
| P-b | B 类 golden 帧：sdo block/重传序列 + lss 全命令 | ~56 |
| P-c | nmt/emcy/pdo/sync_time 补充（含 EmcyConsumer 增强 + D4 对齐） | ~40 |
| P-d | eds/network 子集 + 收尾 | ~18 |

---

## 4. 验收出口

| 批次项 | 出口标准 |
|---|---|
| A1 Storage | store → drop 重建 Node → load → OD 与存储文件逐条比对一致；魔数错误拒绝；0x1011 恢复后值等于编译期默认 |
| A2 EDS 增强 | 含 CompactSubObj 的样例文件导入后子条目完整展开；对支持 0x1021 的设备（CANopenDemo）成功拉取并解析其 EDS |
| A3 LSS 补缺 | 双未配置节点 allnodes 扫描各自收敛；bit-rate configure 帧与 python-canopen LSS master 对拍 |
| B2 LED | 状态机穷举测试覆盖全部输入组合矩阵 |
| B1 网关 | cocomm 语法脚本对 TCP 会话完成 read/write/nmt 全流程；语法错回 ERROR:100；SDO abort 透传为规范错误码；RX 线程崩溃经 check 上抛不静默 |
| C 402 | 转移表穷举（8 态 × 17 转移）+ 间接路径全对测试；模式切换确认超时路径；fault reset 时序断言；心跳丢失中止迁移；oracle：与 python p402 对同一 statusword 序列的状态判定 diff 为零 |
| P-a local 移植 | test_local.py 19 条全部转换且通过；D1–D3 差异断言与本表一致 |
| P-b golden 帧 | block 重传全序列、lss 全命令帧与 test_sdo/test_lss 脚本化帧逐字节一致 |
| P-c 补充 | nmt/emcy/pdo/sync_time 移植全过；D4 对齐后 oracle 追加对拍项通过 |
| P-d 收尾 | eds 边界子集通过；全部暂缓项均有 `#[ignore]` 及原因注释 |

每批次完成后跑全量门禁：`cargo test --workspace` +
`cargo clippy --workspace --all-targets -- -D warnings` + `cargo doc` 无警告
+ `python_canopen_oracle.py` 全过。
