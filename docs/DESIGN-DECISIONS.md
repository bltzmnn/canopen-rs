# 设计取舍与已排除项

> 本文件收纳原 bug 台账中的设计取舍、参考无锚点裁决与已排除误报。
> **当前未修复行为/文档缺陷为 0**；修复后的条目不再保留历史修改记录（见 git）。
> 源码注释中出现的旧 bug ID 仅作历史标签/背景，不再指向待办清单。
> 运行状态、门禁与实板验证见 [`VERIFICATION.md`](VERIFICATION.md)。

## 1. 设计取舍（34 条）

> 每条已核对参考，裁决理由随条目给出；本清单不改变行为。

### 1.1 SDO

- **SDO-11（旧：五G13）空闲时收到上传段轮询的 abort 码三方不一致**：Rust `InvalidCommandSpecifier` 0x0504_0001 与 CANopenNode 一致（`CO_SDOserver.c:617-637`）；
  python 实为**两条路径**（二轮已订正错引）：toggle=1 命中 `server.py:87-90` → 0x0503_0000；空闲 toggle=0 在 `server.py:91` 对 `None` 抛 TypeError，经 `on_request` 兜底（`server.py:56-58`，默认码 `:186`）发 0x0800_0000。
  **裁决**：取 Rust 侧，维持现状（另见 SDO-09）。
- **SDO-12（旧：L3）expedited upload `s=0` 固定取 4 字节**：对定长 <4 对象会 `BadLength`；与 python **逐字节一致**（`client.py:269-276`），CANopenNode 亦只在 s 置位时读 size nibble。
  分叉：python 静默交回 4 字节，Rust 客户端 `decode_value` 失败后**向服务端排队 ABORT** 0x0607_0012（`core/src/sdo/client.rs:527-531`、`:309-332`）。
  **裁决**：仅野路子设备触发，维持现状。
- **SDO-13（旧：L9）block 下载子块 ACK 丢失无子块级重传**：部分 ACK（`ackseq < sent_this_sub`）已有 `writer.rewind` 重传，与 python `_retransmit` 对齐；
  ACK **完全丢失**时 python 同样 `abort(ABORT_TIMED_OUT)`（`sdo/client.py:748-754`），CANopenNode 亦无此路径；host 层超时后 ABORT 对端并整事务重试一次，比 python 更健壮。**裁决**：维持现状。
- **SDO-14（旧：裁决保留项/SDO 分段下载大小守卫延迟一个段报错）**：非末段恰好填满 declared 上限时不立即报错，下一段必然触发中止——结果等价、仅时序晚一段。**裁决**：维持现状。
- **SDO-15（旧：block-download 服务器忽略客户端 blksize）**：与已裁决的 upload 方向对称，内部自洽可用、第三方互操作性弱。**裁决**：维持现状。
- **SDO-16（旧：`SegmentWriter`/`BlockWriter` 与 `SegmentBuffer`/`BlockReceiver` 重组逻辑重复）**：纯重构，无功能影响，重构风险 > 收益。**裁决**：维持现状。
- **SDO-17（旧：TH1）分段切块/重组三处重复**（`core/src/sdo/mod.rs:608` `SegmentWriter` / `:656` `SegmentBuffer` / `block.rs:422` / `server.rs` 内联）：
  与 SDO-16 同源，抽公共 `SegmentEmitter`/`SegmentCollector` 需在有实际改动需求时再做。**裁决**：维持现状。
- **SDO-18（旧：已知互操作局限）block upload 客户端 blksize 不协商**：提案恒 127 且不读服务端反提案 —— 与两参考**逐字节一致**（python 保持类属性 `blksize=127`；CANopenNode v4 仅在 download 方向读 byte4）。
  相关健壮性已覆盖（非法 blksize 显式 abort 0x0504_0002、Rewind 即时采纳新值）。**裁决**：维持，不实施。

- **SDO-33（旧：——，复审轮新增）字节 0 ∈ 0x81..=0x9F 的帧被当作「客户端 abort」：静默拆掉在途传输并伪造 `last_received_abort`**：
  非块态派发对 abort 用**掩码**判定（`req[0] & CS_MASK == 0x80`），把 0x81..=0x9F 的畸形帧整体解释为 abort：无任何回复、清空在途传输、
  并把帧尾 4 字节当作 abort 码记入 `last_received_abort`。CANopenNode 对 abort 用**精确** `data[0] == 0x80`，其余回 0x0504_0001 并告知对端
  （`CO_SDOserver.c:58-60`、`:634-637`；客户端同 `CO_SDOclient.c:576`、`:1206`）；python 与本栈一致（掩码判定，`server.py:32-49` + `constants.py:12`）。
  后果：单帧总线毛刺/串扰可在不通知任何一方的情况下杀死一个多段/块传输，且 `take_last_received_abort` 上报凭空的错误码；
  本栈客户端侧的精确判定（`client.rs:350-352`，SDO-19 已裁决）与服务端不对称。证据：`core/src/sdo/server.rs:404-411`、
  `core/src/sdo/mod.rs:431-437`（`decode_abort` 亦掩码）；块路径倒是精确（`server.rs:315,371`）。
  **裁决（2026-10-23 更新）**：默认保持与 python 一致的掩码判定；新增 `SdoServerConfig::strict` 可选启用 CANopenNode 精确判定（0x81..=0x9F 落 `InvalidCommandSpecifier`，且不写入 `last_received_abort`）。回归 `core/tests/sdo_roundtrip.rs::strict_sdo_config_rejects_masked_aborts_and_upload_initiate_bits`。

- **SDO-35 空闲/分段态对畸形命令字节的严格度低于 CANopenNode（python 同宽松）**：
  `handle` 用 `req[0] & CS_MASK` 派发（`core/src/sdo/server.rs:420-431`），`upload_segment` 只看 toggle 位（`:814-815`）——上传发起 `0x41`
  （参考要求精确 `0x40`）、段轮询 `0x61`（参考 `(x&0xEF)==0x60`）都被正常服务并应答，而非 0x0504_0001。参考：
  `CO_SDOserver.c:620-632`（`(x&0xF0)==0x20` 精确 `0x40`）、`:851`（`&0xEF` 掩码）；python `server.py:33-43` 与本栈同样宽松 →
  **两参考分歧**。与 SDO-30（block initiate 保留位，已按 C 收紧）不同面。
  **裁决（2026-10-23 更新）**：默认维持与 python 一致的宽松派发；`SdoServerConfig::strict` 启用 CANopenNode 的精确命令字节（`0x40`、`0x60/0x70`、`0x80`），与 SDO-33 共用同一严格开关。

### 1.2 PDO

- **PDO-11（旧：P1）子字节（bit 级非整字节）mapping**：python 允许（`pdo/base.py:618-643` bit_offset），CANopenNode 要求字节对齐否则 `ODR_NO_MAP`（`CO_PDO.c:115`）；Rust 跟随 python（测试 `sub_byte_bools_share_one_byte`）。
  **裁决**：两参考矛盾，维持现状。
- **PDO-12（旧：P4）同步 RPDO 的 event-timer 死线按墙钟在 `poll` 推进**：CANopenNode 的 timeout 计时仅在 SYNC 处理门内推进（`CO_PDO.c:717-720,817-844`）——SYNC 停发时 Rust 仍报超时、参考冻结。
  **裁决**：sans-I/O 死线设计取舍，维持现状（与 PDO-07 相邻不同面）。
- **PDO-13（旧：P6）SDO/RPDO 写已映射 OD 地址自动触发事件 TPDO**：CANopenNode 需应用显式 `OD_requestTPDO`，SDO 写不触发；本栈为有意扩展（IMPLEMENTATION §7.4「flagsPDO 思想」）。
  **裁决**：与两参考均不同，属有意扩展，维持现状。
- **PDO-14（旧：八M2）写时 mapping 序列门禁不实现**：CANopenNode `OD_write_PDO_mapping` 的写时门禁与本栈「SDO 写即重装配 + config-error 环」架构冲突，python 从站侧同样没有写时门禁；
  valid COB-ID + 空 mapping 已在装配期按 `CO_PDO.c:623-627` 同判据上报。restricted CAN-ID 同样不在写时拒绝（与 python 一致）。
  **裁决**：维持现状（架构一致性 + 参考实现一致）。
  ⚠ **2026-09-23 复审**：裁决边界需收口——PDO-01/PDO-22 已对 sub2/sub0 引入写时门禁，与「写时门禁整体不实现」的表述冲突；
  mapping 条目（sub1..64）写时放行导致的「SDO 成功 + 槽位静默摘除」另立 **PDO-23**，三者需一次性对齐（建议在本条补记边界后再裁）。
- **PDO-15（旧：八Q1-Q9/Q11/Q12 存疑项）**：`from_od` sub6 静默 0、TPDO tt=0 事件定时器**启用**（对齐 `CO_PDO.c:1245-1252,1305-1310`）、同 COB 多 RPDO 槽 first-match、离开 Operational 清 `pending_event`、
  `poll(now_ms)` 对时钟倒退无守卫（调用方契约）、0x1003 越界 sub 读返回 0、无 CompactSubObj 的 ARRAY 不展开、size=0 block download 与参考逐字节等价、block upload 等 ACK 无自发重发（CANopenNode 亦无）。
  **裁决**：均经参考核对，行为自洽，维持现状。
- **PDO-16（旧：RTR 仅触发 tt=252/253 的 TPDO 应答）**：类型 254/255 等在两套参考中均无从站实现可对照（CANopenNode v4 无 RTR-TPDO 从站路径；python 从站不参与 RTR）。
  **裁决**：维持现状（有意收窄）。

### 1.3 SYN

- **SYN-04（旧：sync-start 相位依赖全局计数器）**：0x1019≠0 时只接受恰 1 字节的 SYNC——counterless SYNC 在 `on_sync` 的宽度门
  （`core/src/node.rs:1457-1476`：`length_ok` 不成立即上报 0x8240 并返回）就被整帧拒收，相位锁分支（`:1564` 起）因此永远看不到无 counter 的帧；
  参考同（`CO_SYNC.c:38-51`）。「0x1019≠0 却发 counterless SYNC」只能由生产者误配置触发。
  ⚠ **全量复核轮订正**：原引「`counter == None` 分支（`:1471-1472`）为死代码」已失效——该分支随 SYN-03 的签名改造
  （`on_sync(&mut self, data: &[u8])`）**整体消失**，不再是死代码；现行证据改指宽度门。行为与裁决不变。
  **裁决**：沉默/不锁相属有意行为，维持现状。
- **SYN-05（旧：0x1019 读回 `v as u8` 截断）**：SDO 写已被 0x1019 门禁（拒 1、>240）拦截；只有 EDS 预载或直接 OD 写入 >255 才会截断，CANopenNode 的 `counterOverflowValue` 本身即 `uint8_t`（同样截断）。
  **裁决**：维持现状。
- **SYN-06（旧：审2-SYNC1）写 0x1019 未检查 0x1006 是否非 0**：本栈 `od_write_guard` 只拒 `== 1 || > 240`（与 `CO_SYNC.c:149-151` 一致）；CANopenNode 通过范围校验后若 `*OD_1006_period != 0` 回 `ODR_DATA_DEV_STATE`（0x0800_0022，`:152-154`）。
  本栈无 SYNC 生产者，触发条件基本不可达；`SdoAbortCode::CannotTransferDeviceState`（`core/src/sdo/mod.rs:97`）目前无产生点。
  **裁决**：留档不修；将来补 SYNC 生产者需一并加此门禁。

### 1.4 NMT

- **NMT-07（旧：NMT-N2）0x1016 非法条目写：Rust 成功 + 零化 + config-error**，CANopenNode abort 拒写（`OD_write_1016`，`CO_HBconsumer.c:76-101`）；python 无 0x1016。
  参考并非一律 0x0604_0043——subindex/count 越界返回 `ODR_DEV_INCOMPAT` = **0x0604_0047**（`CO_HBconsumer.c:85-88`、`CO_ODinterface.c:272-296`），仅重复 nodeId 等 `CO_HBconsumer_initEntry` 失败才映射 0x0604_0043（`:94-96,178-185`）；参考也不校验 nodeId>127（Rust 反而拒绝并零化）。
  **裁决**：单侧参考，留档。
- **NMT-08（旧：心跳消费端对非法状态字节的静默收窄）**：畸形状态字节不产生事件但**刷新监督计时**（CANopenNode 对任意 `DLC==1` 字节重置 `timeoutTimer`）；从未收到合法状态的条目保持 UNKNOWN、不判 Lost。
  **裁决**：维持现状（IMPLEMENTATION §6.5 已记录）。
- **NMT-09（旧：被监督节点 TimedOut 时 boot-up 仍报 `RemoteReset`）**：逐行核对 `CO_HBconsumer.c:320-338`——`pFunctSignalRemoteReset` 对**所有已跟踪条目无条件**触发，只有 EMCY 额外限定 `HBstate == ACTIVE`；本栈对应前者且不为 remote reset 单独发 EMCY。
  **裁决**：维持现状。
- **NMT-10（旧：node-guarding RTR 应答不查 0x100C/0x100D）**：【二轮订正】原「CANopenNode 从站侧同样如此」**不成立**：参考从站**消费**两对象（`CO_Node_Guarding.c:94-152`，`OD_write_100C/100D` `:54-59,81-86`）、每次 RTR 应答重装 lifeTimer、倒计时归零报 0x8130（`:219-223`）、收 RTR 后 `CO_errorReset`（`:199-202`）；
  归属表 `objectDictionary.md:470-471` 标注 "Node guarding slave"，DS-301 profile 两对象 readWrite（`DS301_profile.xpd:772,779`，连带 NMT-02）。
  仍成立的部分：**应答本身**不查两对象（参考应答亦无条件），是否应答取决于 0x1017 心跳是否使能。缺失的从站 life-time→0x8130 监视另立 NMT-03。
  **裁决**：维持现状。

### 1.5 EMC

- **EMC-08（旧：N6）0x1014 禁用后已排队 EMCY 帧被静默丢弃且照常推进 `last_emcy_ms`**：参考 `CO_EM_process`（`CO_Emergency.c:571-582`）出队条件仅 `!bufferFull && fifoPpPtr != fifoWrPtr && inhibit 到期`，**无 `producerEnabled` 门禁**——
  禁用后已入队帧照发；`producerEnabled` 只影响 COB 配置（`:63,378`）与 `pFunctSignalPre`（`:715`）。python 无本地从站 EMCY 生产者。
  **裁决**：双方行为均非理想语义、无一致参考锚点，留档不修；EXTENSIONS §2.6 已记录。备选：B 冻结计时、C 排空不发送。
  ⚠ **当前实现采用备选 B（随 EMC-15）**：`Node::poll` 的出队循环在 0x1014 禁用生产时
  整队冻结（COB-ID 先解析、再 `pop_front()`），帧不再被丢弃、`last_emcy_ms` 也不再推进，重新使能后原帧照发。
  依据是 EMC-15 查明的参考事实：`CO_error` 与 `CO_EM_process` 两侧**都没有** `producerEnabled` 门
  （`CO_Emergency.c:571-582,688-706`），本栈的「丢弃」既无锚点也不可恢复。条目保留在 §3（出队时机/计时语义仍属取舍），
  但「已排队帧被静默丢弃」这半边不再是本栈行为。
- **EMC-09（旧：`install_standard_objects` 与 `standard.rs` 的 0x1003/0x1014 条目重复）**：同一地址两处 `od.insert`，后插者（EMCY 侧，值随 node-id）覆盖先插者（基线侧占位）——行为确定、当前无害。
  **裁决**：维持现状（拆分会破坏两者各自独立可用的约定）。

### 1.6 LSS

- **LSS-03（旧：审4-L2）Configuration 态对 fastscan（0x51）静默**：参考 config 态把 0x51 转交 process 且 `CO_LSS_IDENT_FASTSCAN` 分支不查状态直接回 0x4F（`CO_LSSslave.c:163-167,408-412`）——易让主站误判存在未配置节点。
  **裁决**：本栈行为更贴合 CiA 305 意图、参考反是瑕疵（单侧参考），留档「不跟随」。
- **LSS-04（旧：L2）store-configuration 失败错误码 2 不可达**：参考在同步回调失败时回 error 2（`CO_LSSslave.c:351-372`、`CO_LSS.h:122`）；本栈持久化异步（`take_store_requested` 由应用稍后落盘），无回执通道，只应答 0/1；
  `store_supported` 默认 `true`（未注册持久化的应用仍得应答 0）为**有意的向后兼容取舍**，与参考「无回调 → error 1」不同。
  **裁决**：留档不修；需要时增加回调或错误通道。
- **LSS-05（旧：审2-LSS1）configure_bit_timing 无「逐速率可用性」回调**：本栈只要 tableSelector=0 且 index 合法（`i != 5 && i <= 9`）即应答 error 0 并置 pending；
  CANopenNode 先经 `pFunctLSScheckBitRate` 询问，不支持时回 error=0xFF(manufacturer)+manuf=0x01（`CO_LSSslave.c:303-323`、`CO_LSS.h:110-112`）。
  **已核实一致的部分**：selector/index 非法时本栈回 error 1，与参考 else 分支逐字节相同；`pFunctLSScheckBitRate == NULL` 时参考整帧丢弃（`:303-307`），即本栈 `set_bit_timing_supported(false)`（IMPLEMENTATION §8.3）。
  差异仅剩「合法但设备不支持的速率」：本栈把判定推迟给 app（`take_pending_bit_timing`），但**无回执通道**。**裁决**：留档不修。
- **LSS-06（旧：`AllNodesScanner` 临时 node-id 不校验总线冲突）**：【二轮订正】`on_assigned` 文档（`lss.rs:619-626`）实际标注的局限是**幻影地址/需校验 configure+store 应答**（R10-P2-9），「临时 id 复用已配置节点」这条注解**并不存在**（原「已在文档标注」表述失实）。
  **裁决**：维持——临时 id 复用已配置节点是调用方职责；注解随文件触碰补写。

- **LSS-10（旧：——，复审轮新增）selective 进度链在中间帧重复/交错时拒绝选中，参考仍选中**：本栈 selective 是严格链 `0→1→2→3`：
  任一帧不匹配或**重复中间帧**即把 `selective` 归零，需四帧连续全对才回 0x44；参考是四值各自存储、serial 步对 **128 位整体**比较——
  `[v][p][p][r][s]`（product 重发一次，例如主站重试）在本栈静默（第二个 p 因 `selective==1` 不成立归零，serial 步 `selective==3` 失败），
  参考照常选中应答。后果：带单帧重试/交错的选择序列在本栈超时、在参考从站成功。CiA 305 的逐段比较语义上本栈更严格（贴合规范 FSA），
  属参考更宽松的单侧差异，但互操作可见。
  **参考**：`CO_LSSslave.c:74-102`（四个 case 各自 `memcpy` 存值不做链式门禁；`:97` serial 步 `CO_LSS_ADDRESS_EQUAL(lssAddress, lssSelect)` **全 128 位**比较）、
  `CO_LSS.h:210-213`（ADDRESS_EQUAL 宏定义）。**证据**：`core/src/lss.rs:938-963,980-987`。
  **裁决**：**维持现状**（更贴合规范）；应在 `LssSlave` 文档与 EXTENSIONS 记为「比参考严格的序列状态机，重复中间帧不兼容」；
  如需参考兼容，改为四值暂存 + 末态比较。

### 1.7 GTW / EDS / API

- **GTW-08（旧：G10）未实现 `ERROR:<code> #<desc>` 描述后缀**：参考为可选编译项 `CO_CONFIG_GTW_ASCII_ERROR_DESC`（`CO_gateway_ascii.c:460-461,481-484`）。**裁决**：有意裁剪，EXTENSIONS §2.2 已记录。
- **EDS-08（旧：E5）ObjectType 语义不消费**（任何对象都接受 sub 段、缺 DataType 默认 Unsigned32）：python 对 ObjectType=0x7 静默忽略 sub、缺 DataType 中止导入（`eds.py:163,319`）。**裁决**：宽松超集，IMPLEMENTATION §11.1 已记录（反向缺口见 EDS-02）。
- **EDS-09（旧：E6）未知 AccessType 拼写归一为 Rw 并记 warning**：python 保留原串按子串判定（`objectdictionary/__init__.py:418-424`）；仅畸形拼写受影响。**裁决**：宽松超集，文档同 §11.1。
- **API-05（旧：`serve_node` 的 SDO 会话活动判定固定用 `0x600+node`）**：已改为 `$node.node_id().sdo_rx_cob_id()` 单一来源（`host/src/sdo_exchange.rs:437-441`）；core 的 `Node::on_frame` 目前只监听默认 `0x600+node`（0x1200 多通道未实现，IMPLEMENTATION §5.5「第一期不做」），行为等价。
  **裁决**：维持现状；待 core 支持自定义 SDO RX COB 时自动跟随。

## 2. 已排除（误报，含反证）

#### SDO-19【已排除】（旧：SDO 客户端 abort 检测「精确 `0x80` 非掩码」）

- **原断言**：应改为 `resp[0] & CS_MASK == CS_ABORT`。
- **反证**：block 末段字节 0 为 `0x80 | seqno`（≥ `0x81`），掩码判断会把 `0x81..=0xFF` 的末段全部误判为 ABORT；`core/src/sdo/client.rs:350-351` 注释明确精确比较是有意为之，与 `decode_abort` 用掩码并不矛盾（后者只处理 abort 帧）。代码：`client.rs:352`。
- **结论**：**误报**，维持现状。

#### SDO-20【已排除】（旧：block download 重复段「应重发 ACK」）

- **原断言**：曾援引 `CO_SDOserver.c:88-101` 称 CANopenNode 会重发 0xA2。
- **反证**：该参考段落为「duplicate … ignore it」——**重复段**（seqno == 已收）本就忽略，只有非重复的错误序号才转 ACK 态（见 SDO-07）。本栈重复段忽略与参考一致。
- **备注**：原条目的行号 `core/src/sdo/server.rs:752-758` 指的是空闲上传段轮询/会话中止路径，**引用失效**，正确位置为 `:871-875`（本轮订正，见附录 B）。
- **结论**：**误报**（重复段部分），维持现状。

#### PDO-17【已排除】（旧：审-P2）「同步 RPDO 单缓冲，参考双缓冲排空两帧」

- **反证**：CANopenNode 双缓冲按 SYNC toggle 选择（`CANrxToggle` 仅在收到 SYNC 时翻转，`CO_SYNC.c:53-55`），同一 SYNC 周期内两帧落同一 `bufNo`、后者覆盖前者；
  接收函数注释明确 "previous message will be lost and overwritten by the new message"（`CO_PDO.c:429-431`），SYNC 处理里的 while（`:740-747`）只是拷贝期竞态保护。原条目对参考的解读有误。
- **结论**：**误报**，无需修改。

#### EMC-10【已排除】（旧：写 0x1003:0 非 0 值「SDO 成功但静默回滚」）

- **反证**：`od_write_guard`（`core/src/sdo/server.rs:1340` 起，⚠ 复审订正行号）早已拒写非 0 值（回归测试 `writing_zero_to_1003_0_clears_emcy_history` 覆盖）；`node.rs` 的 `resync_od` 分支只是绕过 SDO 服务端的防御路径，不是「静默回滚」。
  另：该 abort 码已由 **EMC-11** 修正为参考的 0x0609_0030。
- **结论**：**误报**，维持现状。

#### GTW-09【已排除】（旧：审-G4）「`read` 尾随垃圾回 101，参考应回落 unknown command 100」

- **反证**：`read` 的可选 datatype token 以 `closed=1` 读取（要求结束于行尾，`CO_gateway_ascii.c:872-880`），尾随垃圾使 `CO_fifo_readToken` 置 err（`CO_fifo.c:420-427`）→ 仍回 101；
  `:1529-1533` 的 100 路径只服务未知**动词**（本栈同样回 100）。原条目把两条路径混为一谈；按原建议改成 100 反而引入偏离。
- **结论**：**误报**，维持现状。

#### EDS-10【已排除】（旧：`core/src/datatypes.rs` 的 UNSIGNED40/48/56/64 类型索引「偏移 +1」）

- **反证**：类型索引 `0x18/0x19/0x1A/0x1B`（跳过 `0x17`）经 CANopenNode 官方 `objectDictionary.md:433-437`（⚠ 复审订正范围，0x1B 在原圈定范围外相邻行）与 python-canopen `data_type` 字典核实，为**事实标准**。
- **结论**：**误报**，代码正确。

#### API-06【已排除】（旧：异步重试泄流窗口「只排空首帧」）

- **反证**：`host/src/async_transport.rs:71-77` 把 std 时钟 deadline 重新锚定到 tokio 时钟（L4 修复自带注释），泄流循环在时钟同步时与阻塞路径等价；`saturating_duration_since` 得 0 即窗口到期、继续排空剩余帧（`sdo_exchange.rs:287-299`：仅超时结束窗口，真 IO 错误上抛）。
- **备注**：原引 `:271-279/:329-341` 已订正为 `:287-299/:347-361`。
- **结论**：**误报**，维持现状。

#### API-07【已排除】（旧：`Node::new` 的 `defaults` 快照早于重编号）

- **反证**：`defaults_snapshot_taken` 构造时为 `false`（`core/src/node.rs:378,442`），`attach_storage`（`core/src/node.rs:590-597`）用**当时**（重编号之后）的 OD 重新快照——构造期那次 clone 在挂载 storage 前不会被使用。
- **备注**：原引 `node.rs:531-537` → `:552-558`（修复轮）→ **`:590-597`（全量复核轮刷新）**。
- **结论**：**误报**，维持现状。

#### HOST-01【已排除】（复审轮新立目 → 全量复核轮驳回）

- **原断言**：对 0x1010:sub≥1 / 0x1011:sub≥1 写入非 `'save'`/`'load'` 魔数（如 0、0xDEADBEEF）时，SDO 写**成功**且值落 OD，
  读回被污染、主站据此误判存储能力；次级观察：`handle_write` 对 0x1011 匹配 `sub @ 1..=127` 而 0x1010 是 `sub >= 1`。
- **反证**：`od_write_guard`（`core/src/sdo/server.rs:1403-1418`）对 `addr.subindex >= 1 && matches!(addr.index, 0x1010 | 0x1011)`
  比对魔数（`SAVE_MAGIC_U32`/`LOAD_MAGIC_U32`），不符即 `Error::DataCannotBeTransferred`；
  三条下载路径（加速 `:555`、分段 `:709`、块 `:1043`）全部经该守卫，`Node::on_frame` 也走 `self.sdo.handle`（`core/src/node.rs:913`）——
  该守卫在 `HEAD`（本轮改动之前）即存在，不是修复轮新增。探针实测：
  写 `0x1010:1 = 0xDEADBEEF` → **`cmd=0x80` abort `0x0800_0020`**、读回仍 `Uint(1)`；写 `0` → 同样 abort，读回不变。
  CANopenNode `storage/CO_storage.c:42-45,96-99` 的拒绝行为与本栈一致。
- **残余观察**（不改变驳回结论，仅记录）：`core/src/storage.rs:138`（0x1010 用 `sub >= 1`）与 `:148`（0x1011 用 `sub @ 1..=127`）区间判定不对称；
  该路径只在魔数写入后才被触发，`groups > 127` 时 0x1011:128.. 的魔数写会落 OD 但不派发动作——属写后钩子细节，可随文件触碰统一。
- **备注**：探针见上条反证；原条目 `core/src/node.rs:750-758`、`core/src/storage.rs:137-156` 描述的是写**后**钩子，不构成「SDO 写成功」。
- **结论**：**误报**（主断言不成立），维持现状。

#### DOC-11【已排除】`block.rs` 的 `RESERVED_MASK` 注释对 CS_5 不成立（误报）

- **原断言**：`core/src/sdo/block.rs` 的 `RESERVED_MASK = 0x18` 要求 bits 4 **与** 3 同时为 0，
  而注释称「`0xF9`/`0xFB` require them [bits 4..3] to be zero」——该说法对 `0xFB`（CS_5）**不成立**，
  「`0xFB` 屏蔽掉 bit3，故 CS_5 只要求 bit4 为 0」；并称本栈拒绝 0xA8/0xAC 而**参考接受**。
- **反证（位算）**：`0xFB = 1111'1011` 清掉的是 **bit2**（0x04），**不是 bit3**（0x08）——
  bit3 在掩码里被**保留**；因此 `(& 0xFB) == 0xA0` 与 `0xA0`（bit3 = 0、bit4 = 0）比较时
  **同样要求 bit3 = 0**，连同 bit4 = 0 恰是「bits 4..3 均为 0」；`0xF9 = 1111'1001`
  （清 bit1/bit2）对 CS_6 同理。故 `core/src/sdo/block.rs` 的注释**对 CS_5 与 CS_6 都成立**。
- **参考核对**：`ref/CANopenLinux/CANopenNode/301/CO_SDOserver.c:627`（CS_6 判据 `(& 0xF9) == 0xC0`）、
  `:629`（CS_5 判据 `(& 0xFB) == 0xA0`；⚠ 原条目引 `:631`，实为状态赋值行）——
  参考对 `0xA8`/`0xAC` 同样**拒绝**（`0xA8 & 0xFB = 0xA8 ≠ 0xA0`），「参考接受」不成立。
- **证据**：`core/src/sdo/block.rs:29-32`（掩码与注释）、`:259`、`:282`（两处使用点）。
- **结论**：**误报**——位算错误（`0xFB` 清 bit2 而非 bit3）；注释、实现与参考行为三者一致，**无需改动**。
