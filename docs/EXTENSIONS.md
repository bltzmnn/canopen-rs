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
| CiA 303-3 LED 指示 | ✅ 状态机 + `Node::poll` 集成 + `Node::leds()` | 极廉价纯状态机；控制器状态由宿主 `set_can_status` 注入，见 §B2 |
| **CiA 402 驱动轮廓** | ❌ 已裁决移除（2026-08-25） | 复审后裁决不实现 CiA 402；原 canopen-402 crate 与 host profile_402 胶水已删除（§2.1 调研存档保留） |
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
- 命令语法（h:67-133）：`[[<net>] <node>] read 0x1017 0 i16` /
  `write ... u16 500` / `start|stop|preop|reset node|reset comm` /
  `set node|network|sdo_timeout|sdo_block`
- 错误码（CiA 309-3，h:154-183）：**100 请求不支持 / 101 语法错误 /
  102 内部状态 / 103 超时 / 105 未设默认节点 / 106 不支持的网络 /
  107 不支持的节点**；
  SDO abort 以 `ERROR:0x…` 原码透传
  （`CO_gateway_ascii.c:414,502-507`），不折算为数值码。
- **`[[<net>] <node>]` 前缀（DESIGN-DECISIONS.md 审-G3 / GTW-21 / GTW-22）**：参考的解析
  循环先读最多两个「首字符是数字」的 token——一个为 node、两个为 net+node——
  遇到首个非数字 token 即视为命令；第三个数字 token 判语法错
  （`CO_gateway_ascii.c:638-661`）。**范围校验发生在动词派发之前**且对所有动词
  一致（`switch (i)` 的 `case 1`/`case 2`，`:666-694`）：`[1] 300 set node 5`
  与 `[1] 300 help` 都回 107（畸形前缀此前能改写默认节点，见 GTW-21）；反之
  首个 token 非数字时它**就是**命令，未知动词回 100（GTW-22，此前回 101）。
  单网络网关对合法 net 解析后忽略（限 16 位，越界回 106），与关闭
  `CO_CONFIG_GTW_MULTI_NET` 的参考构建一致（`checkNetNode`，`:236-257`）。
- **序列号解析与保持（DESIGN-DECISIONS.md 审-G2）**：参考用 `strtoul` 解析 `[<seq>]`
  （数字前缀有效、无数字得 0）并把结果保存为解析器状态 `gtwa->sequence`，
  后续缺失/畸形 `[seq]` 帧回显该值（`CO_gateway_ascii.c:633-636,1541-1545`）。
  本栈同样保持 last-sequence：`[12ab]` 回 `[12] ERROR:101`（不派发命令），
  无 frameless 回显最近一次合法序列。
- **空写载荷（DESIGN-DECISIONS.md 审-G1）**：参考对 `size == 0` 的写载荷（空 `hex`、
  空 `vs ""`）判语法错 101（`CO_gateway_ascii.c:976-980`，与数据类型无关）；
  本栈已对齐（此前产生零长 SDO 下载）。
- **datatype 裁剪（DESIGN-DECISIONS.md 五G9）**：参考表 `dataTypes`
  （`CO_gateway_ascii.c:303-325`）含 `x8..x64`（hex 显示的无符号整数）、
  `r32/r64`（实数）与 `os/us/d`（Base64 编解码的 OCTET/UNICODE/DOMAIN）；
  本栈只实现 `hex / b / i8..64 / u8..64 / vs`，其余 token 按语法错 101
  拒绝（反向接受参考没有的 `str` 别名，宽松方向无害）。实现 Base64
  路径需补 `CO_fifo_readB642a`/`cpyTok2B64` 等价物，暂无消费方。
- **字面量进制（DESIGN-DECISIONS.md 五G10 / 审3-G1 / GTW-19）**：参考用
  `strtoul(_, _, 0)` 解析 node/sub/index/sequence，typed `write` 的数值
  token 则由 `CO_fifo_cpyTok2U16/U32/U64/I64` 经
  `strtoul/strtoll(buf,&sRet,0)` 解析（`CO_fifo.c:840,927,956,985,1014,1043`）
  ——两处均为基 0。本栈**分两种口径**：
  - **sequence**（`[010]`）走完整 C 基 0 文法（`parse_c_int_token`），前导 `0`
    为八进制、负值经 `strtoul` 回绕——与参考逐字一致；
  - **node/sub/index** 只接受十进制与 `0x`/`0X`（`parse_u32_auto`），不接受
    前导 `0` 八进制——**有意的收窄**（避免把 `08` 之类的十进制拼写误读）；
  - **typed `write` 的数值字面量**自审3-G1 起按完整基 0 文法解析
    （`0x`/`0X`、前导 `0` 八进制、可选正负号，尾随垃圾判语法错），与参考
    一致；越界值仍按 R10-P2-10 判语法错（参考对 u32/u64/i32/i64 为静默
    截断/饱和，本栈更严格；`u32 -1` 的回绕差异见 GTW-20 条目）。
- **`#` 注释语义（DESIGN-DECISIONS.md GTW-14 / GTW-23，原五G11 归因已订正）**：注释只作用于
  **命令/数值 token 读取器**与 **hex/数值扫描器**——`#` 在该 token 内即终止
  token 并清空行剩余内容（`CO_fifo.c:352-360`、`:1187-1188`）；
  `vs` 的**值解析路径**（`CO_fifo_cpyTok2Vs`，`:1266-1333`）把 `#` 当普通
  可见字符逐字复制，故 `vs ab#cd` 的值是 `ab#cd`、`vs "a #b"` 的值是
  `a #b`；字符串结束后的 `#` 才起注释作用，其后出现非注释文本仍是语法错。
  两处例外已按参考收紧（GTW-23）：**以 `#` 开头的 `vs` 值是注释**，判语法错
  101（`CO_fifo_trimSpaces` 先看到注释分隔符，`:1246-1252`）；**闭合引号后必须
  先有空白**才能接注释——`vs "a" #c` 合法、`vs "a"#c` 判 101（引号后出现 graph
  字符即错，`:1314-1332`）。
- **token 长度上限（DESIGN-DECISIONS.md GTW-25）**：参考把每个命令 token 读进同一个
  `char tok[20]`，填满即 `err`（101）并返回空 token（`CO_gateway_ascii.c:602`、
  `CO_fifo.c:419-436`）；typed `write` 的数值字面量另有限制——8/16/32 位用
  `buf[15]`、64 位用 `buf[25]`（`CO_fifo.c:830,859,888,917,946,975,1004,1033`），
  而 `hex`/`vs` 逐字复制无限长。本栈三类上限均已对齐。
- **布尔字面量（DESIGN-DECISIONS.md 五G12 / GTW-24）**：`b` 接受任意 0..=255 数值，**按完整
  基 0 文法**（`0x`/前导 `0` 八进制，`CO_fifo_cpyTok2U8` 的 `strtoul(_,_,0)`，
  `CO_fifo.c:828-856`），另接受参考没有的 `false`/`true` 拼写。
- **`r`/`w` 动词缩写（DESIGN-DECISIONS.md 七G9）**：与参考一致，`r`/`w` 分别等价于
  `read`/`write`（`CO_gateway_ascii.c:702-705`）。
- **`ERROR:<code> #<desc>` 描述后缀（DESIGN-DECISIONS.md 七G10）**：参考的描述文本由
  可选编译项 `CO_CONFIG_GTW_ASCII_ERROR_DESC` 控制
  （`CO_gateway_ascii.c:460-461,481-484`）；本栈恒输出裸码
  （`ERROR:101` / `ERROR:0x…`），不实现描述后缀。
- **NMT 动词的 node 后缀写法（DESIGN-DECISIONS.md GTW-04，有意宽松超集）**：参考只接受
  `[[<net>] <node>] start|stop|preop`（node 必须是前缀，动词 token 之后必须
  立即结束行，`CO_gateway_ascii.c:996-1046`；`reset` 的模式 token 同，
  `:1058-1092`），`start 5` / `start node 5` 在参考上都是 101。本栈额外接受
  后缀写法（并保留为预期行为，测试 `gateway.rs:1013`）。选择保留而非收紧是
  有意的：收紧会破坏既有脚本，而宽松方向不产生新的互操作风险（主站仍可只用
  参考拼法）。
- **`set` 前缀 node（DESIGN-DECISIONS.md GTW-07）**：`4 set node 5` 在参考上**接受并忽略**
  前缀（前缀 `case 1` 只存 node，`set` 分支不读它，`CO_gateway_ascii.c:666-675,
  769-792`）；本栈自本轮起对齐（此前回 101）。
- **序列号 token 的三个边缘（DESIGN-DECISIONS.md GTW-06）**：`[]` 得 0 且正常派发、
  `[1 2]` 在空格处截断且**不更新** last-sequence（回显旧值 + 101）、`[-1]`
  按 `strtoul` 回绕为 0xFFFFFFFF 并派发——三者均已对齐参考
  （`CO_gateway_ascii.c:221-232,624-640`）。**赋值时机**亦已对齐：参考在检查
  末字符为 `]` 之后**先**把 `getU32` 的前缀值写入 `gtwa->sequence`、再判
  `err`（`:632-636`），故 `[1]]` 先记 1 再回 `[1] ERROR:101`（GTW-25）。

最小切片 = read/write/NMT/set 系列（跳过 LSS 命令族——占 C 实现 ~40%——
及 log/led）。估算 600–1000 行 Rust + 测试。

### 2.3 Storage（CANopenNode `storage/CO_storage.{h,c}` + `CO_storageLinux.c`）

- 0x1010 sub1 写 `'save'`(0x65766173) / 0x1011 sub1 写 `'load'`(0x64616F6C)
  触发；存储分组按 subindex 展开(h:53-77)
- 后端函数指针注入(store/restore, h:97-98)；Linux 后端落文件
- master 侧 `RemoteNode::store()/restore()` 写标准魔数；2026-10-23 新增
  `StorageGroup` 与 `store_group`/`restore_group`（API-09），旧方法保留；
  批次 A 补从站侧 trait + host 文件后端

### 2.4 LSS 差距审计（对照 `305/CO_LSSmaster.h:33-74` 与 CiA 305 服务族）

**已交付**（此前列的「两个缺口」均已落地并有测试/golden 帧）：

- allnodes scan：`AllNodesScanner`（`core/src/lss.rs`；`on_assigned` 注明的
  幻影地址局限见 `DESIGN-DECISIONS.md` LSS-06）
- bit-rate configure/activate：主站编码器 `encode_configure_bit_timing` /
  `encode_activate_bit_timing` + 从站应答与 pending 通道双端接线
  （`pyport_lss` golden 帧覆盖）

**identify 服务（2026-10-23 已交付，DESIGN-DECISIONS.md LSS-07）**：

- 从站 `LssSlave::handle` 支持 0x46..0x4B 的严格六连帧（vendor/product
  等值、revision/serial 闭区间），全匹配回 0x4F；新的 0x46 会重启一轮，
  丢帧后下一次完整重试即可恢复。0x4C 仅在 active 与 pending 均为 0xFF 时回
  0x50。
- 主站编码器：`encode_identify_remote_slave` / `encode_identify_non_configured`；
  响应判定：`decode_identify_slave_response` /
  `decode_identify_non_configured_response`。
- 网关 LSS 命令族仍不暴露 identify 类命令（独立的网关缺口，保持）。

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

### 2.6 有意差异记录（无参考锚点，或有锚点但裁决为有意为之）

- **LSS Configuration 态的 selective-serial 条件应答**（DESIGN-DECISIONS.md LSS-01 的裁决边界）：
  本栈在 Configuration 态收到 0x43 时**仅在身份匹配**才回 0x44（不改状态、不改 selective 进度）；
  CANopenNode 该态把任何 CS 转交 process 并**无条件**回 `0x44`（`CO_LSSslave.c:161-169,281-285`），
  不校验身份（属参考瑕疵）。本栈更严格，避免误应答他人的 select；代价是会话被中途打断时主站会超时。
- **LSS Configuration 态的 fastscan 静默**（DESIGN-DECISIONS.md LSS-03）：本栈在 Configuration 态不处理 0x51；
  参考直接转交 process 并回 0x4F（`CO_LSSslave.c:163-167,408-412`），易让主站误判存在未配置节点。
  本栈行为更贴合 CiA 305 意图、参考反是瑕疵，裁决「不跟随」。
- **LSS selective 为严格链 `0→1→2→3`**（DESIGN-DECISIONS.md LSS-10）：中间帧重复或交错即把进度归零，
  需四帧连续全对才回 0x44；参考是四值各自暂存、serial 步对 128 位整体比较
  （`CO_LSSslave.c:74-102`、`CO_LSS.h:210-213`），故 `[v][p][p][r][s]` 这类带单帧重试的序列参考照常选中、
  本栈超时。CiA 305 的逐段比较语义上本栈更严格（贴合规范 FSA），属参考更宽松的单侧差异，裁决维持现状。
- **EDS `[<index>Name]` 节的合并语义**（DESIGN-DECISIONS.md EDS-06）：本栈**显式定义优先、顺序无关**；
  python-canopen 用 `copy_variable` 把 sub1 模板**完整副本**覆盖 1..N 全部已存在 sub
  （`eds.py:172-182`、`objectdictionary/__init__.py:248-252`），结果依赖文件内段落出现顺序。
  本栈语义更确定，且与「`[<index>subN]` 显式段覆盖模板」的实现自洽，裁决维持现状。
- **EMCY error-reset 不入 0x1003 历史**（DESIGN-DECISIONS.md EMC-03）：CANopenNode 的
  `CO_error` 对 set 与 reset **都**写 fifo——复位行 `errorCode = CO_EMC_NO_ERROR`
  会占用历史槽位、使 0x1003 中交错出现 `0x0000` 行（`CO_Emergency.c:664-707`）；
  本栈 `EmcyProducer::clear` 只更新 active 表、向 0x1003 仅写实际故障行。
  取舍：0x1003 的可读性更好（容量 8 不被复位行挤占），且复位帧本身照常发出；
  python-canopen 无本地 EMCY 生产者可对照，属单侧参考差异，裁决留档。
- **EMCY 0x1014 禁用后的已入队帧**（DESIGN-DECISIONS.md N6）：本栈在禁用期间
  **冻结队列与计时**，重新使能后原帧照发；CANopenNode v4 的出队条件无
  `producerEnabled` 门禁（`CO_Emergency.c:571-582`），禁用后已入队帧仍按旧
  COB 发出；python-canopen 无本地 EMCY 生产者。本栈采用冻结语义。
- **LSS fastscan 错误 CS 应答帧**（DESIGN-DECISIONS.md 审-L1）：本栈与 python-canopen
  (`lss.py:266-267,293-297`) 把非 `0x4F` 应答视为该 probe 静默、继续扫描；
  CANopenNode `CO_LSSmaster.c:738-740,814-816` 判整个 `SCAN_FAILED`。
  两参考自身分歧，本栈取 python 侧收敛性更好的行为，裁决留档。
- **自回环抑制严于 python**（DESIGN-DECISIONS.md PDO-08）：只要本节点有 valid TPDO 占用
  该 COB-ID，`on_frame` 即把帧当自身回声丢弃（先于 RPDO 匹配，
  `core/src/node.rs`）；python-canopen 仅在自身正在发送时忽略
  （`self._task is not None`，`pdo/base.py:331-342`），故在「他站也用同一
  COB-ID 发送」的**冲突配置**下两参考行为不同（CANopenNode 的 rx filter 由
  RPDO 注册，倾向投递）。该冲突配置本身非法，两参考均无一致锚点，
  裁决维持 DESIGN-DECISIONS.md P3-22 的严格侧（宁可丢帧也不把回声当 RPDO 数据，
  并避免误触发 PDO 映射的 OD 写入）。
- **TIME 消费的 NMT 门、RPDO 让位与 0x1012**（DESIGN-DECISIONS.md SYN-09/SYN-10）：
  (Pre)Op 才消费、`claimed_by_rpdo` 让位；0x1012/0x1013 已入标准 OD，
  默认 `0x1012 = 0x100` 且消费者/生产者位关闭，COB-ID 与角色均可配置
  （见 IMPLEMENTATION §8.3）。
- **EDS `CompactSubObj` ARRAY 的模板位置**（DESIGN-DECISIONS.md EDS-23）：本栈沿用 CANopenNode 的
  约定——`[index]` 段的**内联定义落在 sub 0**，它就是展开模板（sub 0 随后成为元素个数，
  见 `expand_compact_sub_obj`）；python-canopen 则合成 sub 0 并把成员建在 **sub 1**
  （`eds.py:133-141`）。仅影响显式写 `ObjectType=0x8` 且带 `CompactSubObj` 的节，
  且两者展开后的字典形状一致（sub0 计数 + sub1..N 成员），故保留本栈约定。
- **EDS 前导零整数字面量**（DESIGN-DECISIONS.md 审-E1）：本栈按十进制物化（`"010"`→10，
  宽松超集，见 IMPLEMENTATION §11.1）；python `int("010", 0)` 抛
  `ValueError` 并 warning+忽略默认值 (`eds.py:369-373`)。属可解析输入上的
  有意宽松，裁决留档。
- **EDS `;` 内联注释感知引号**（DESIGN-DECISIONS.md EDS-18）：本栈的 `strip_comment`
  **引号感知**——`DefaultValue="a;b"` 保留全值 `a;b`；python 的
  `RawConfigParser(inline_comment_prefixes=(';',))` 不感知引号，同一文件读回
  `"a`（自空白后的 `;` 起注释，本机 2.4.1 实测）。保留尾部是**可恢复方向**：
  供应商在字符串里写 `;` 时仍能拿到他想要的默认值，而截断是不可恢复的。差异
  仅出现在「引号内含 `;`」这一种写法上，裁决留档（有 warning 之外无副作用）。
- **EDS index 段位数与 `[DummyUsage]` 取值**（DESIGN-DECISIONS.md EDS-22）：本栈接受 1–3 位
  十六进制 index 段（python 只认 `^[0-9A-Fa-f]{4}$`，其余忽略）并对该情况记一条
  `warnings`；`Dummy000n` 的值按基 0 解析（python `getint` 为十进制，`0x1` 会
  抛 `ValueError` 中止导入）。两处都是**只放宽不收窄**的方向：多认出的段至少不会
  丢掉真实对象（真实 index 必然 4 位十六进制），故留档。
- **EDS 成对引号被当作定界符消费**（DESIGN-DECISIONS.md EDS-16）：`DefaultValue="a;b"` 的
  引号在**字符串类型**上被剥掉（值 `a;b`），python 逐字保留（`"a;b"`，且在
  OCTET/DOMAIN 上 `bytes.fromhex('"01 A2"')` 直接失败）。注意这与
  `ParameterName` 保留引号（DESIGN-DECISIONS.md N13）方向相反：名字按 CiA 306 视为
  自由文本，默认值按类型文法视为定界字符串。取舍：引号内的 `;` 不会被误当注释
  （见上一条），而 python 侧的失败是硬错误；裁决留档。
- **EMCY 0x1003 历史项的 register 冻结点**（DESIGN-DECISIONS.md EMC-18，修正 EMC-13 的
  「等价」措辞）：本栈在条件 `raise` 那一刻把 error register 冻结进历史条目；参考在
  `CO_EM_process` **出队那一刻**才 `msg |= errorRegister << 16`
  （`CO_Emergency.c:571-582`）。帧在 0x1015 抑制窗、非 (Pre)Op 态或生产禁用
  （EMC-15）期间排队时两者不等价：排队期间新增的 register 位不会补进较早那条历史项
  （抑制窗下 0x2001→0x2002，参考首条为 0x11、本栈为 0x01）。核心断言（历史不被回溯改写）
  一致；改「出队时回写」需 `Node` 在发帧时把当拍 register 交给 producer 并重写 0x1003，
  而 `pending_emcy` 不保存「载荷 → 条件」映射，收益与改动面不成比例，裁决留档。
- **EMCY 生产禁用期间已排队帧的处理**（DESIGN-DECISIONS.md EMC-08 + EMC-15）：0x1014 禁用生产时
  本栈**冻结**整个 `pending_emcy`（COB-ID 不可用即不出队、计时不动），重新使能后原帧照发；
  参考的 fifo 两侧都没有 `producerEnabled` 门（`CO_Emergency.c:571-582,688-706`），
  即「照发」——本栈没有可用的 COB-ID 可发，冻结是唯一不丢帧的选择（§3 EMC-08 备选 B）。
- **node-guarding 主站生产者 / 主站 commanded-state**（DESIGN-DECISIONS.md 审-M1/M2）：
  本栈 master 侧为 observe-only——`Network::node_state` 只读心跳消费端状态，
  `RemoteNode` 无 `reset`/`reset_communication`/`preop` 便捷方法，也没有
  周期 RTR 生产者（`serve_node` 仅回答入站 RTR，不做轮询）。python-canopen
  在 `send_command` 后乐观更新本机状态并可通过 `start_node_guarding` 周期发
  RTR（`nmt.py:74-78,141-150,185-193`）。属主站能力缺口（非协议缺陷），
  需要时按 `PeriodicTask` 路线图一并补。

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
- host：文件后端 `FileStorage`——**已交付**为行式文本格式 v2（每行
  `index sub len bytes… crc16`，CRC 覆盖 CRC 前字段的规范文本；v1 旧文件
  仍可读）：写入经 temp + fsync + **回读 CRC 校验**后原子 rename
  （对齐 `CO_storageLinux.c:78-100`）；加载逐行校验，损坏行跳过但计数经
  `load` 的 `Err` 上报，`Node` 将其记入 `take_config_errors()`
  （DESIGN-DECISIONS.md G7）。单条参数上限 8 KiB（`MAX_PARAM_LEN`，防超长 len 的
  恶意/损坏文件触发大分配 M7）；**保存侧同样预检**：超限条目直接返回
  说明性 `Err`，不再先写出再被回读判为 corrupt 而整体失败
  （DESIGN-DECISIONS.md 七F7）——更大对象请改用应用自有存储，或调大两侧上限。

#### A2 EDS 导入增强（✅ 已交付）

- `read_eds_from_device(node)`：client.rs 新增——SDO 读 0x1021 全量字节流
  （长度先读 sub0，分段/block 自动选择）→ `eds::parse()`
- CompactSubObj 展开：parser 捕获该字段后按模板展开 sub1..=N（sub0 =
  Unsigned8 count），导入保真度提升

#### A3 LSS 补缺（✅ 已交付）

- allnodes scan：master 顺序探测（switch global config + inquire serial，
  逐节点 assign 临时 id 或 fastscan 连续收敛），提供 `scan_all_nodes()`
  高层 API
- bit-rate configure/activate：主站 encode/decode + 从站应答接入现有
  `LssSlave::handle`

### 批次 B：调试互操作

#### B2 CiA 303-3 LED 模块（✅ 状态机 + Node 集成已交付）

- core `src/leds.rs`：纯 sans-I/O 位型机——输入 NMT 态/LSS 态/错误标志位/
  now_ms，输出 (绿, 红) 两 bool；CANopenNode 表驱动逻辑移植
- **有意差异（DESIGN-DECISIONS.md LED-02）**：flicker 相位本栈按 **每个 50 ms tick** 翻转（保持 CiA 303-3 的 10 Hz 交替）；
  参考 `CO_LEDs.c:52` 在 while 内重算但 `LEDred` 只在循环外提交（`:163-164`），
  故**每次 `CO_LEDs_process` 调用**只翻转一次——50 ms 节拍的单 tick 调用二者逐字节一致，
  大 Δt 调用下参考退化为按调用节拍交替。模块头与 §2.6 已互链记录。
- **Node 集成（2026-10-23）**：`Node::poll(now_ms)` 按经过时间推进
  `Leds`，聚合 NMT 态、LSS Configuration 态、RPDO/SYNC/HB 错误、0x1001
  错误寄存器与固件下载标志；`Node::leds()` 返回最近一次 `LedOutput`。
- 控制器相关输入无法在 sans-I/O core 读取：宿主在 SocketCAN 错误帧/状态
  变化时调用 `Node::set_can_status(CanStatus { bus_off, bus_warn })`；
  阻塞/异步传输均提供 `can_status()`（netlink 尽力查询，失败返回 `None`）
  方便宿主轮询/注入。应用用 `Node::set_firmware_download(bool)` 标记固件下载。
- 未配置节点（`new_unconfigured`）在 `Node` 中仍按 NMT Initialising 参与
  红灯逻辑；LSS Configuration 态使绿灯 flicker。

#### B1 CiA 309-3 ASCII 网关（✅ 部分交付）

> 交付范围：core `gateway.rs` 解析器 + `Network::gateway_line` 内存桥接。
> **未做**：TCP 监听封装、stdio 模式、`cocomm` 语法集成测试客户端脚本。

- core `src/gateway.rs`：sans-I/O 解析状态机——字节流入 → 命令事件出；
  响应字节流出。首期命令集：
  - `[n] <node> read <idx> <sub> [type]`（sub 必填，CiA 309-3 /
    CANopenNode `CO_gateway_ascii.c:863-870`；省略 `type` 时按 CANopenNode
    缺省 `hex` 原始字节透传，空格分隔大写十六进制回显）
  - `[n] <node> write <idx> <sub> <type> <val>`（sub **必填**，对齐参考 `closed = 0U`
    读法 `CO_gateway_ascii.c:932-938`，省略即 101；`vs` 支持双引号定界与
    `""` 转义；`hex` 按可选空格分隔的十六进制字节写入）
  - `start|stop|preop|preoperational|reset node|reset comm[unication]`
    （命令与子命令 token 不区分大小写，CANopenNode `CO_gateway_ascii.c:698,749,1076`）
  - `set node <v>`（**仅此**；参考的 `set network|sdo_timeout|sdo_block`
    未实现，按请求不支持 100 应答）
  - 空行/纯注释行回空响应帧，与 CANopenNode `responseWithEmpty` 一致
- **未实现**：`help`（本栈与参考均按请求不支持 100 应答；参考受编译开关
  `CO_CONFIG_GTW_ASCII_PRINT_HELP` 控制且默认关闭，`CO_gateway_ascii.h:34`、
  `CO_gateway_ascii.c:1488-1513`；DESIGN-DECISIONS.md DOC-07）；`log`/`led` 命令族
- host：TCP 监听封装（每连接一个网关实例桥接 Network/SocketCan）+
  stdio 模式；`cocomm` 语法的集成测试客户端脚本
- 错误码：实现 100（请求不支持）/101（语法）/102（内部状态）/103（超时，
  仅保留给未实现的 LSS 命令族）/
  105（未设默认节点）/107（不支持的节点，含 SDO 的广播 id 0）；
  SDO abort **与超时**均按原码透传（`ERROR:0x%08X`，SDO 超时为
  `0x05040000`，对齐参考的 SDO 路径 `CO_gateway_ascii.c:1571-1573`，
  DESIGN-DECISIONS.md G8）；500 LSS 段随 LSS 命令族后置

### 批次 C：CiA 402 驱动轮廓 —— 已裁决移除

原计划新增 `canopen-402` crate；2026-08-24 复审后决定 **CiA 402 不在本栈
范围内**，已实现的 crate 与 host `profile_402` 胶水一并删除。相关缺陷清单
（P0-2/P1-13/P2-9）随之作废，不再跟踪。

### 批次 D：python 测试套件移植（pyport）—— ✅ 已完成（见 IMPLEMENTATION §16）

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
| test_pdo.py (8) | `core/tests/pyport_pdo_sync.rs` | A | ~6 | 经 SDO 的 PDO 配置组；map export 属 python 内部跳过 |
| test_sync.py + test_time.py (13) | `core/tests/pyport_pdo_sync.rs` | A/B | ~8 | codec 部分直转；周期发送需 PeriodicSendTask（暂缓项） |
| test_lss.py (26) | `core/tests/pyport_lss_frames.rs` | B | 26 | 全命令 golden 帧（含 bit-timing/activate——覆盖 A3 特性） |
| test_eds.py 边界子集 | host `eds.rs` 测试补充 | C | ~10 | 缺 sub 触发 `0x06090011`、CompactSubObj、类型边界；夹具直接复制 sample.eds/datatypes.eds |
| test_network.py (13) | `host/tests/pyport_network.rs` | L2 | ~8 | subscribe 多订阅/check 重抛为纯逻辑可测；send_periodic 属暂缓项 |
| test_variable.py (84 行) | ⚠️ 暂缓 | C | 0 | 依赖未实现的 phys/desc/bits 分层值访问（EDS factor/unit/value_descriptions/bit_definitions 解析 + 访问器），单独立项 |
| test_od.py / test_utils.py | 跳过 | — | 0 | python OD 内部结构 |

**合计 53 条 pyport 回归测试（`cargo test` 实测计数）。**

#### 已知行为差异对照表（转换时按此适配断言）

| # | 蓝本行为 | 本栈行为 | 适配 |
|---|---|---|---|
| D1 | block upload 收到分段应答即报错（客户端无降级） | 自动降级分段成功取数 | 改为正向用例 |
| D2 | 空字符串读取 → abort `0x08000024`（python 服务端） | 服务端同样 abort（`0x0604_0047`，CANopenNode 码；DESIGN-DECISIONS.md 审-S2 已对齐行为，仅码不同） | 断言改 abort 码 |
| D3 | 无默认值对象读取 → abort `0x060A0023` | OD 物化零值可读 | 断言改为读到值 |
| D4 | 加速写宽度不符 → abort `0x06070010` | 已对齐 `0x06070010`（TypeMismatch） | **已对齐**（主流实践；分段声明长度不符仍用 12/13） |

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
| B2 LED | 状态机穷举测试覆盖全部输入组合矩阵；flicker 相位 per-tick 为**有意差异**（LED-02），单 tick 调用与参考逐字节一致 |
| B1 网关 | cocomm 语法脚本对 TCP 会话完成 read/write/nmt 全流程；语法错回 ERROR:101（未知命令回 100）；SDO abort 透传为规范错误码；RX 线程崩溃经 check 上抛不静默 |
| C 402 | 转移表穷举（8 态 × 17 转移）+ 间接路径全对测试；模式切换确认超时路径；fault reset 时序断言；心跳丢失中止迁移；oracle：与 python p402 对同一 statusword 序列的状态判定 diff 为零 |
| P-a local 移植 | test_local.py 19 条全部转换且通过；D1–D3 差异断言与本表一致 |
| P-b golden 帧 | block 重传全序列、lss 全命令帧与 test_sdo/test_lss 脚本化帧逐字节一致 |
| P-c 补充 | nmt/emcy/pdo/sync_time 移植全过；D4 对齐后 oracle 追加对拍项通过 |
| P-d 收尾 | eds 边界子集通过；全部暂缓项均有 `#[ignore]` 及原因注释 |

每批次完成后跑全量门禁：`cargo test --workspace` +
`cargo clippy --workspace --all-targets -- -D warnings` + `cargo doc` 无警告
+ `python_canopen_oracle.py` 全过。
