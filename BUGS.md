# CANopen 协议栈 Bug 清单

> 权威台账：**只列未修复的已核实 bug**，修复后即时从清单删除（变更历史见 git）。
> 核验方式：与 python-canopen（`ref/canopen`）、CANopenLinux
> （`ref/CANopenLinux/CANopenNode`）交叉比对，并对照当前工作区代码。

## 未修复 bug

（无）

> 本轮（2026-09）对照 `ref/canopen`（python-canopen）与
> `ref/CANopenLinux/CANopenNode` 逐行重新审计后核实的新条目已全部修复并有
> 回归测试覆盖：SDO 服务端接受 `e=0,s=0` 分段下载、EMCY 消费者 COB 范围与
> 历史溢出策略、0x1003 标准错误字段、0x1014 禁用/写守卫、心跳超时清状态、
> block upload 加速降级、Gateway 错误码对齐 CiA 309-3、EDS 名称顺序
> 无关/`wrap_signed_hex` 上界/CompactSubObj 合并/NaN 与 VisibleString
> 默认保真、codegen 拒绝 `_`、CLI monitor DLC 与 `sub` 十六进制/`eds dump`
> 展开、SDO 客户端专用 ABORT 码、storage 非魔数拒绝、LSS 非法 mode/store
> 错误/切换延时/复位出口。详见 `git log` 与 `docs/IMPLEMENTATION.md`
> 对应小节；此前轮次的 A1-A3 / B1-B7 / C1-C5 亦已全部修复。

## 裁决保留项（非 bug，注明理由；已核对参考实现）

- **node-guarding RTR 应答不查 0x100C/0x100D**（`core/src/node.rs`）
  0x100C（guard time）/0x100D（life time factor）是**主站侧**超时计算参数；
  从站对 guard RTR 的应答不查这两个对象，CANopenNode `CO_Node_Guarding.c`
  的从站侧同样如此。裁决：维持现状。是否应答取决于 0x1017 心跳是否使能
  （见 IMPLEMENTATION §6.3）。

- **`AllNodesScanner` 临时 node-id 不校验总线冲突**（`core/src/lss.rs`）
  已在 `on_assigned` 文档标注已知局限：临时 id 复用已配置节点是调用方职责，
  主站场景下由应用层拓扑保证。裁决：维持现状。

- **SDO 分段下载大小守卫延迟一个段报错**（`core/src/sdo/server.rs`）
  非末段恰好填满 declared 上限（`buf.len() + seg.data.len() == max`）时不立即报错，
  但下一段必然触发 `buf.len() >= max` 中止——结果等价，仅时序晚一段。裁决：维持现状。

- **block-download 服务器忽略客户端 blksize**（`core/src/sdo/server.rs`）
  与已裁决的 upload 方向对称：内部自洽可用，第三方互操作性弱。裁决：维持现状。

- **`SegmentWriter`/`BlockWriter` 与 `SegmentBuffer`/`BlockReceiver` 重组逻辑重复**
  纯重构项，无功能影响，重构风险大于收益。裁决：维持现状。
  （同源子项 `SEGMENT_DATA_MAX` 三处重复、`block.rs` 死字段、`mod.rs` 冗余掩码已修复。）

- **TH1 分段切块/重组三处重复**（`core/src/sdo/mod.rs:599` / `block.rs:409` /
  `server.rs` 内联）
  发送侧 `SegmentWriter`/`BlockWriter`/server 内联，接收侧
  `SegmentBuffer`/`BlockReceiver`/server 内联 `download_segment` 三份重复。
  重构风险 > 收益（与上方 `SegmentWriter` 条目同源），仅在有实际改动需求时
  才抽公共 `SegmentEmitter`/`SegmentCollector`。裁决：维持现状。

- **L3 expedited upload `s=0` 固定取 4 字节**（`core/src/sdo/mod.rs:398-414`）
  对定长 <4 对象会 `BadLength`。与 python-canopen **逐字节一致**（`client.py:269-276`
  在 `SIZE_SPECIFIED` 未置位时同样取满 `res_data` 4 字节，其后按对象类型解码亦失败；
  CANopenNode 同样只在 s 置位时读 size nibble）——仅野路子设备触发，宽松截断反而
  偏离参考实现。裁决：维持现状。

- **L9 block 下载子块 ACK 丢失无子块级重传**
  （`core/src/sdo/client.rs` `BlockDownloading` + host `drive_exchange_body`）
  **部分 ACK**（`ackseq < sent_this_sub`）已有 `writer.rewind` 重传，与 python-canopen
  `_retransmit` 对齐；**ACK 完全丢失**（超时）时 python-canopen 同样不做子块级重传
  （`_block_ack` 超时即 `abort(ABORT_TIMED_OUT)` + raise，`client.py:748-754`），
  CANopenNode 亦无此路径。canopen-rs 的 host 层超时后 ABORT 对端并整事务重试一次，
  比 python-canopen 直接上抛更健壮。裁决：维持现状（与参考实现一致）。

- **`serve_node` 的 SDO 会话活动判定固定用 `0x600+node`**
  （`host/src/sdo_exchange.rs:337`）
  已改为调用 `$node.node_id().sdo_rx_cob_id()` 单一来源；但 core 的 `Node::on_frame`
  目前只监听默认 `0x600+node`（0x1200 多通道未实现，IMPLEMENTATION §5.5「第一期不做」），
  因此行为等价，仅消除重复推导。待 core 支持自定义 SDO RX COB 时自动跟随。裁决：维持现状。

- **`install_standard_objects` 与 `standard.rs` 的 0x1003/0x1014 条目重复**
  （`core/src/emcy.rs` / `core/src/standard.rs`）
  同一地址被两处 `od.insert`，后插者（EMCY 侧，值随 node-id）覆盖先插者（基线侧占位）——
  行为确定、当前无害；重复仅属定义冗余，拆分会破坏两者各自独立可用的约定。裁决：维持现状。

- **sync-start 相位依赖全局计数器**（`core/src/node.rs`）
  `0x1019≠0` 但 SYNC 帧无 counter 字节时相位永不锁定、sync-start TPDO 保持沉默——
  counterless SYNC 是生产端配置不一致（0x1019 本就是生产端声明），沉默是有意行为，
  已加文档注释。裁决：维持现状。

- **心跳消费端对非法状态字节静默丢弃**（`core/src/nmt.rs`）
  畸形心跳不报警也不计超时——消费端视角合理（畸形帧不是生产者死掉的证据），
  帧被静默消费但无副作用，已加文档注释。裁决：维持现状。

- **八M2 写时 mapping 序列门禁不实现**（`core/src/sdo/server.rs` / `core/src/pdo.rs`）
  CANopenNode `OD_write_PDO_mapping` 的写时门禁（PDO 使能时拒绝改映射、
  子项要求计数为 0）与本栈「SDO 写即重装配 + config-error 环」架构冲突，
  且 python-canopen 从站侧同样没有写时门禁——写时仅拒绝 bit 布局与
  COB-ID 0。valid COB-ID + 空 mapping 已在装配期作为配置错误上报
  （`CO_PDO.c:623-627` 运行时同判据）。IMPLEMENTATION §7.3 已按此订正。
  restricted CAN-ID 同样不在写时拒绝（与 python-canopen 一致），
  `od_write_guard` 的文档注释已按实际行为订正。裁决：维持现状
  （架构一致性 + 参考实现一致）。

- **八Q1-Q9/Q11/Q12 存疑项裁决**：`from_od` sub6 静默 0、
  TPDO tt=0 不启用 event timer（CiA 措辞偏 254/255）、同 COB 多 RPDO 槽
  first-match、离开 Operational 清 `pending_event`（CiA 未硬性规定）、
  `poll(now_ms)` 对时钟倒退无守卫（调用方契约）、EMCY reset 判据严于
  python（更符合 CiA）、0x1003 越界 sub 读返回 0（注释已自述）、
  UnicodeString 默认按 hex、无 CompactSubObj 的 ARRAY 不展开、
  size=0 block download 与参考逐字节等价、block upload 等 ACK 无自发重发
  （CANopenNode 亦无）——均经与参考实现逐项核对，行为自洽或参考实现
  自身不一致，无内部矛盾。裁决：维持现状，已在本段留档。

- **RTR 仅触发 tt=252/253 的 TPDO 应答**（`core/src/node.rs` `on_remote_request`）
  类型 254/255 等其它类型的 RTR 语义在两套参考中均无从站实现可对照
  （CANopenNode v4 无 RTR-TPDO 从站路径；python-canopen 的
  `PdoMap.remote_request` 不区分传输类型，但其从站并不参与 RTR）。
  本栈只对 `RtrOnlySync`（回放 SYNC 快照）/`RtrOnlyEvent`（实时打包）
  应答，其余类型沉默属有意收窄。裁决：维持现状。

- **0x1019 读回 `v as u8` 截断**（`core/src/node.rs` `on_sync`）
  SDO 写已被 0x1019 门禁（拒 1、>240）拦截；只有 EDS 预载或直接 OD
  写入 >255 才会截断。CANopenNode 的 `counterOverflowValue` 本身就是
  `uint8_t`，参考同样截断。裁决：维持现状。

- **被监督节点 TimedOut 时 boot-up 仍报 `RemoteReset`**（`core/src/nmt.rs` `on_heartbeat`）
  逐行核对 CANopenNode `CO_HBconsumer.c:320-338`：`pFunctSignalRemoteReset`
  回调对**所有已跟踪条目无条件**触发，只有 EMCY `CO_errorReport` 额外
  限定 `HBstate == ACTIVE`。本栈的 `HbEvent::RemoteReset` 对应前者
  （且本栈不为 remote reset 单独发 EMCY），故 Active/TimedOut/Unknown
  都报与回调语义一致。裁决：维持现状。

## 已知互操作局限（裁决保留，非 bug）

- **block upload 客户端 blksize 不协商**：提案恒 127 且不读取 initiate 响应
  byte4 的服务端反提案。这是与两套参考实现**逐字节一致**的行为（python-canopen
  保持类属性 `blksize=127`；CANopenNode v4 仅在 download 方向读 byte4）——自洽
  可用，下载方向具备完整协商。相关健壮性已覆盖：非法 blksize 显式 abort
  （0x05040002）、Rewind 即时采纳新值。裁决：维持，不实施。

## 已排除（核实为误报，非 bug）

- `core/src/datatypes.rs` 的 `UNSIGNED40/48/56/64` 类型索引（`0x18/0x19/0x1A/0x1B`，
  跳过 `0x17`）——曾误报为“偏移 +1”。经 CANopenNode 官方
  `objectDictionary.md:433-436` 与 python-canopen `data_type` 字典核实，此非连续映射为
  **事实标准**，代码正确，无需修改。

- **SDO 客户端 abort 检测「精确 `0x80` 非掩码」**（`core/src/sdo/client.rs:340`）
  曾建议改为 `resp[0] & CS_MASK == CS_ABORT`，但 block 末段字节 0 为
  `0x80 | seqno`（≥ `0x81`），掩码判断（`CS_MASK=0xE0`）会把 `0x81..=0xFF` 的
  block 末段全部误判为 ABORT——`client.rs:338-339` 注释明确说明精确比较是**有意为之**，
  与 `decode_abort` 使用掩码并不矛盾（后者只处理 abort 帧）。误报，维持现状。

- **异步重试泄流窗口「只排空首帧」**
  （`host/src/sdo_exchange.rs:277-286` + `host/src/async_transport.rs:78-79`）
  该锚定逻辑（把 std 时钟 deadline 重新锚定到 tokio 时钟）正是 L4 的修复
  （`async_transport.rs:71-77` 注释自述）；泄流循环在时钟同步时与阻塞路径等价，
  `saturating_duration_since` 得 0 即窗口到期、继续排空剩余帧。误报，维持现状。

- **写 0x1003:0 非 0 值「SDO 成功但静默回滚」**
  （`core/src/sdo/server.rs` `od_write_guard`）
  曾报告写入成功后被 `resync_od` 静默回滚；实测 `od_write_guard`
  早已对非 0 值返回 `ValueOutOfRange`（abort 0x06090031，回归测试
  `writing_zero_to_1003_0_clears_emcy_history` 覆盖）；`node.rs` 的
  `resync_od` 分支只是绕过 SDO 服务端的防御路径。误报，维持现状。

- **block download 重复段「应重发 ACK」**（`core/src/sdo/server.rs:748-754`）
  曾援引 `CO_SDOserver.c:88-101` 称 CANopenNode 会重发 0xA2；
  逐行核对该参考代码为「duplicate … ignore it」——重复段（seqno == 已收）
  本就忽略，只有非重复的错误序号才转 ACK 态。本栈 `Step::Continue` 与
  参考一致。误报，维持现状。

- **`Node::new` 的 `defaults` 快照早于重编号**（`core/src/node.rs:307`）
  曾担心持久化的 0x1014 会在 storage reset 时回退 node-1 占位。
  实测 `defaults_snapshot_taken` 在构造时为 `false`，`attach_storage`
  （`node.rs:516-524`）会用**当时**（重编号之后）的 OD 重新快照——
  构造期那次 clone 在挂载 storage 前不会被使用。误报，维持现状。

## 质量门禁（现状）

`cargo test --workspace` 全绿（394 passed / 0 failed；`--features tokio` 亦 394/0）·
clippy `-D warnings` 0 告警 · fmt 干净 · `cargo doc --no-deps` 0 告警 ·
oracle 实时对拍 **11/11**（驱动真实 python-canopen `SdoClient` 状态机捕获请求帧，
并与 canopen-rs 的 `oracle_frames` example 实时输出 diff——不再使用手抄常量）。
