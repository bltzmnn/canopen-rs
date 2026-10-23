# 变更日志

本项目遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/) 格式。

## [Unreleased]

### Added（2026-10-23 能力补齐）

- **EDS 元数据（EDS-21）**：新增 `EdsFileInfo`、`EdsDeviceInfo`、`EdsComments`
  与 `Eds.baudrate`；解析 `[FileInfo]`、`[DeviceInfo]`、`[Comments]` 和
  `[DeviceComissioning] Baudrate`，数值/布尔容错，`CommentN` 按编号排序。
- **EDS 双构建模式（EDS-17/API-03）**：`EdsBuildPolicy`、`Eds::build_with`、
  `build_lenient`；Lenient 对单个坏默认值记 warning 并以类型零值继续。
  新增独立 `EdsBuildError` + `Eds::build_typed`，不修改 `HostError` 变体。
- **存储组（API-09）**：`StorageGroup`（`ALL`/`COMMUNICATION`/`APPLICATION`，
  `TryFrom<u8>` 校验 1..=127）与 `RemoteNode::store_group`/`restore_group`；
  旧 `store`/`restore` 保留并委托 `ALL`。
- **SDO 严格模式（SDO-33/35）**：`SdoServerConfig { strict }` 与
  `Node::set_sdo_config`，可选启用 CANopenNode 精确命令/abort 字节判定。
- **PDO 写时严格模式（PDO-23）**：`PdoConfigPolicy::{AssemblyTime, StrictWrite}`
  与 `Node::set_pdo_config_policy`；默认装配期，StrictWrite 在 SDO 写时校验
  目标存在性/方向/重复映射。
- **guarding 生命期（NMT-03）**：`Node::poll` 按 0x100C × 0x100D 监视主站
  RTR 静默，超时 `raise_emergency(0x8130)`，新 RTR 清除；0x1017 > 0 时禁用。
- **NMTcontrol（NMT-06）**：`NmtControl` + `Node::set_nmt_control`，支持
  startup-to-Operational、错误寄存器/bus-off 驱动的状态迁移与自动恢复；默认全关。
- **TIME 配置（SYN-10）**：标准 OD 新增 0x1012/0x1013；默认
  `0x1012 = 0x100` 且消费者关闭；按 bit31/bit30/低 11 位配置消费与生产，
  新增 `Node::time_frame(unix_ms)`；0x1012 写守卫检查保留位与受限 CAN-ID。
- **LED 集成（LED-01）**：`Node::poll` 驱动 `Leds`，新增 `CanStatus`、
  `Node::set_can_status`、`Node::set_firmware_download`、`Node::leds()`；
  宿主注入 bus-off/bus-warn，应用注入固件下载态。阻塞/异步
  `SocketCan` 同时新增 `can_status()` netlink 查询。
- **LSS identify（LSS-07）**：从站严格六连帧 identify（0x46..0x4B）与
  0x4C/0x50；主站 `encode_identify_remote_slave` /
  `encode_identify_non_configured` 及响应判定函数。
- **API-04**：新增显式 `ObjectDictionary::set_value_unchecked`；旧 `set_value`
  保留为委托（仍绕过 AccessType、仍做类型/范围校验）。

### Fixed（2026-10-23）

- **SDO-38**：块下载客户端对服务端非法/零 blksize 反提案按 CANopenNode 夹到
  127，不再 abort 0x0504_0002。
- **SDO-40**：块上传开始发送数据后重复的 `0xA3` 改回参考语义，立即
  `0x0504_0001` abort；首个 `0xA3` 仍正常启动子块。

### 破坏性变更（Rust API）

- PDO 通信记录**写时门禁收紧**（2026-09-28 批次 E6a）：使能态写 sub1 COB-ID（改
  标识）/ sub3 inhibit / sub6 SYNC start value 现在 abort `0x0609_0030`。依赖
  「先使能再填记录」顺序的既有主站流程需改为**先填后使**。

- 移除 `canopen_core::gateway::GatewayEvent::Ignore`：空行/纯注释行现在返回
  `GatewayEvent::Respond(String::new())`（空响应帧，对齐 CANopenNode
  `responseWithEmpty`）。
- `canopen_core::gateway::GwDataType` 新增 `Hex` 变体；`read` 省略类型 token
  时缺省为 `hex` 原始字节透传（此前缺省 `Uint(32)` 十进制）。
- `canopen_core::gateway::GatewayErrorCode` 新增 `UnsupportedNet = 106` 变体
  （枚举非 `#[non_exhaustive]`，外部穷举 match 需补臂）。
- `canopen_host::eds::Eds` 新增 `type_defs` 字段，`EdsSubEntry` 新增
  `deferred_data_type` 字段（复合 DataType `>0x1B` 的延迟解引用）。
- 以下公共类型标记 `#[non_exhaustive]`，后续新增变体/字段不再算破坏性变更：
  `GatewayEvent`、`GwDataType`、`Eds`、`EdsSubEntry`（外部 match 需通配臂、
  外部结构体字面量构造不再允许）。

### 破坏性变更（存储文件格式）

- 参数存储文件升级为 **v2**：每行末尾追加 CRC16（XMODEM/CCITT init 0，
  与 CANopenNode `CO_storageLinux.c` 相同多项式），写入后回读校验成功才
  原子替换；加载时校验失败的行不再静默跳过，而是通过
  `Node::take_config_errors()` 上报（`Error::Io`，消息含 `corrupt`）。
  v1 旧文件仍可读取（不校验 CRC）。
- `ParameterStorage` 后端 save/load/reset 的失败现在会记录到
  `Node::take_config_errors()`（此前被丢弃）。
- 加载时**类型/宽度错位的陈旧行**（`Value::decode_le` 失败）也计入 `stale`
  并随 `corrupt` 一起上报，不再静默跳过（docs/DESIGN-DECISIONS.md HOST-02）。
- 0x1006（通信周期）由只读占位改为 **readWrite**（DS-301 profile），
  SYNC 生产者静默监测因此可经标准 SDO 配置流程启用（docs/DESIGN-DECISIONS.md SYN-07）。
- 0x100C/0x100D（guarding guard time / life time）改为 **readWrite**
  （DS-301 profile）；2026-10-23 起由从站侧 life-time 监视消费（NMT-03）。

### Added

- Gateway `hex` 数据类型（读写）、`reset communication` 全拼。
- Gateway typed `write` 的数值字面量改按参考的 C 基 0 文法解析
  （`0x`/`0X` 十六进制、前导 `0` 八进制、`+`/`-` 号，尾随垃圾仍判错）——
  `write 0x1017 0 u16 0x1F4` 现在返回 OK，此前回 `ERROR:101`
  （审3-G1，`CO_fifo.c:840,927,956,985,1014,1043`）。
- EDS 复合 `DataType`（CANFestival `0x20` 等）按 `[<idx:X>sub1]`
  `DefaultValue` 解引用；缺段回落 `Domain` + warning。
- TPDO `0x18xx:6`（SYNC start value）>240 的写门禁（abort 0x0609_0030）。
- LSS fastscan 完成后扫描位置复位为 0（对齐 `CO_LSSslave.c:136`）。

### Fixed

- 0x1003:0 写非 0 值的 abort 码由 `0x0609_0031` 改为参考的 `0x0609_0030`
  （`InvalidValue`；CANopenNode `OD_write_1003` → `ODR_INVALID_VALUE`，
  审3-S1）；PDO 通信记录 sub1 保留位 29..11 的 abort 码同样由
  `0x0609_0031` 改为 `0x0609_0030`（`CO_PDO.c:507-509`，审3-P1）。
- Gateway `vs` 写入支持双引号定界与 `""` 转义；未加引号多词报语法错误。
- Gateway 空行/注释行回空响应帧，逐行等应答的客户端不再死等。
- 存储损坏可上报（G7）；写后回读 CRC16 校验。
- 非法 SYNC start value 不再静默导致 TPDO 永不锁相（P5）。
- fastscan 完成后续扫不再因残留位置被忽略（L4）。
- block download 激活时收到的 ABORT 现在记录 abort code，`take_last_received_abort`
  不再随服务器内部状态漂移（R2）。
- block download end 帧的 CRC 按服务端 initiate 响应的协商位决定，不再硬编码
  携带（R4）。
- PDO 映射的 dummy 判据补上 `sub == 0`：`0x0005:01` 这类条目按真实目标查找，
  对象不存在时报映射错误而非静默当填充（R7）。

### Added（七轮）

- `LssSlave::set_bit_timing_supported`、`FastscanMaster::on_response_frame` /
  `AllNodesScanner::on_response_frame`（校验响应 CS==0x4F）、LSS 主站
  `encode_inquire_vendor_id/product_code/revision_number/serial_number` 与
  `decode_inquire_*_response`。
- `EdsSubEntry::parameter_value` / `parameter_value_raw`：EDS `ParameterValue`
  解析，初始 OD 值优先于 `DefaultValue`（`#[non_exhaustive]`，外部构造已禁用）。

### Fixed（2026-09-20 全面审核）

- **SDO 服务端**：零长度对象上传（普通/block initiate）改为 abort
  `0x0604_0047`（CANopenNode `DEVICE_INCOMPAT`），不再返回 `0x40` 零长分段
  （审-S2）；block upload initiate 的 pst 协议切换去掉 `size > 4` 下限——
  ≤4 字节对象答 expedited（审-S1）；block download 段接收窗口失活时回部分
  ACK 保活而非直接拆除（审-S3）；上传 END 帧发出后保持等待客户端 `0xA1`
  的相位，空闲 `0xA1` 按非法命令 abort `0x05040001`（审-S4）。
- **PDO**：PDO 参数写入只重建被写槽位，不再清掉无关 TPDO 的锁存事件/相位
  （审-P1）；离开 Operational 清 TPDO 发送计时（审-P3）；部分宽度 RPDO
  映射整宽覆写、未映射高位清零（审-P4，替换此前的合并保留）；mapping sub0
  计数 >64 写时 abort `0x0604_0042`、装配期不再静默截断（审-P5）。
- **NMT/EMCY**：心跳消费端超时上报 EMCY `0x8130`、全部恢复后清除（审-N1）；
  心跳生产者使能期间每个 poll 把 node-guarding toggle 复位置 0（审-N2）。
- **Gateway**：空 `hex`/空 `vs ""` 写载荷判语法错 101（审-G1）；序列号按
  `strtoul` 语义取数字前缀并保持 last-sequence 回显（审-G2）；支持
  `[[<net>] <node>]` 前缀（审-G3，新增错误码 106）。
- **LSS**：`on_response_frame` 文档注释订正——错误 CS 按 python-canopen 视为
  静默，参考 CANopenNode 实为 `SCAN_FAILED`，两参考分歧留档（审-L1）。
- **EDS**：前导零整数字面量（`"010"`）为有意宽松超集，补文档（审-E1）。
- 复核误报并留档：同步 RPDO 单缓冲与 CANopenNode 双缓冲行为等价（审-P2）、
  `read` 尾随垃圾参考同样回 101（审-G4）。

### Fixed（七轮）

- tt=0（SyncAcyclic）TPDO 进入 Operational 也发一次初始帧（不再要求配置
  event timer）。
- RPDO event-timer 超时/恢复上报 EMCY 0x8250 / error-reset；离开 Operational
  清闩锁。
- 心跳监督随本节点 NMT 态门控，Pre/Op 边沿重置被监督条目（Stopped 不再误报
  Lost，重入不再用陈旧时间戳）。
- EMCY 消费者按整个 `0x00xx` reset 族清 active 表（对齐 python-canopen）。
- `pending_emcy` 延期队列有界（256），满时丢弃新条目。
- 持久化单条 >8 KiB 时保存侧预检并报说明性错误，不再写出后被回读判 corrupt。
- Boolean 条目的浮点默认值不再在两个构建出口分歧：warning + `Bool(false)`。
- VisibleString/UnicodeString 解码剥离尾部 NUL 填充；EDS 整数字面量支持
  `0b`/`0o` 与 `_` 分隔符；Gateway 接受 `r`/`w` 动词缩写。

### Fixed（2026-09-24 修复批次 A/B/C1/C2；git 提交历史）

- **EDS**：`$NODEID±offset` 改 i128 精确解析，极端偏移返回 `ValueOutOfRange`
  而非 debug panic / release 回绕（EDS-11）；新增 `Eds::build` 把字典构建的内容错误
  包成 `HostError::Eds`，`Eds::parse` 文档改为「当前不可失败，问题记入 `warnings`」
  （API-08）。
- **EMCY**：0x1003 历史改**滚动窗口**（队满淘汰最旧，不再永久冻结在最早 8 条，
  EMC-12）；各历史项 bits16..23 改为**入史时冻结**的 register 快照，不再被当前
  全局值回溯改写（EMC-13）；厂商信息域统一为参考布局 **`[errorBit, infoLE(4)]`**
  （帧字节 3..7），0x8210/0x8220 补 errorBit `0x04`、0x8240 `0x19`、
  0x8250 `0x17`、SYNC 超时 `0x18`（EMC-14）；overflow 位只在传输队列**真丢帧**时
  置位、队列排空即清，写 0x1003:0 不再清该位（EMC-02）；写 0x1015 重启 inhibit
  窗口（EMC-05）；error-reset 行不入 0x1003 历史（**有意差异**，见 EXTENSIONS §2.6）。
- **SYN**：新增 SYNC 生产者静默监测（1.5× 0x1006 周期无 SYNC → EMCY 0x8100，
  下个 SYNC 清除，SYN-01）；首次 SYNC 前不武装、0x1005 bit30 生产者时跳过监测、
  vendor info 记实测静默 µs（SYN-08）。
- **SDO**：stale-transfer abort 码 `GeneralError` → `InvalidCommandSpecifier`
  0x0504_0001（SDO-27）；三处畸形帧 abort 改取在途对象地址（SDO-28）；
  缺失子项的 abort 码改按**索引存在性**判定（无 sub-0 的索引不再误报
  0x0602_0000，SDO-29）；block initiate 解码器拒绝保留位 bits 4..3（SDO-30）；
  block download initiate 补「声明长度 vs 定宽」即时校验（SDO-25）；子块超时拆成
  **0.7×T 发一次 partial ACK / 1×T abort** 双阈值，恢复窗由 ≈0 恢复到参考的 0.3×T
  （SDO-26）。
- **PDO**：通信记录 sub2 保留传输类型 241..=251 写时拒（abort 0x0609_0030，
  PDO-01）；0 位 mapping 条目按 dummy 容忍而非摘掉整个槽位（PDO-02）；映射条目
  `bit_length > 64` 或累计 >64 bit 写时拒（0x0604_0042，PDO-23）。
- **LSS**：bit-timing index 9 订正为标准 `CO_LSS_BIT_TIMING_AUTO`（LSS-09）；
  EXTENSIONS §2.4 的 LSS 缺口清单重写（LSS-08）。
- **Gateway**：`set` 未知 key 改为先匹配 key、回 100（GTW-16）；序列 token 按
  `strtoul(_,_,0)` 前缀语义解析并要求以 `]` 结尾（GTW-17）；`write` 的 subindex
  改为**必填**（GTW-18）；`u32`/`x32` 负值字面量按声明宽度回绕（GTW-20）；
  `#` 注释收窄到命令/数值 token 与 hex 载荷，`vs` 值逐字携带 `#`（GTW-14）。
- **EDS/HOST**：`Eds::warnings` 由 `cli eds dump/gen` 打印到 stderr、
  `read_eds_from_device` 文档提示检查（EDS-01）。
- **LED**：CiA 303-3 指示灯状态机 blink 400 ms（200/200）与 flash 1.2/1.6/2.0/2.4 s
  注释订正（LED-03）；flicker per-tick 相位记为**有意差异**（LED-02）。
- **文档**：`transport` 模块头改为「逐响应窗口 + 写不重试」（HOST-03）；
  IMPLEMENTATION §8.2 EMCY 段落重写（DOC-05）。

### Fixed（2026-09-28 修复批次 C3/D；git 提交历史）

- **SYN**：SYNC 静默监测不再被 NMT Stop→Start 永久解除——离开 (Pre)Op 只清
  时间锚点、保留武装位，重入后从首次 `poll` 重新起算 1.5× 周期（对齐
  `CO_SYNC.c:395-400,402-406`）；NMT reset communication 同步复位「已见 SYNC」
  状态（对齐 `CO_SYNC_init` 的 `memset`，SYN-12）。
- **存储**：未挂后端时 0x1010/0x1011 的**合法魔数**写改为 abort
  `0x0601_0002`（`ODR_READONLY`），不再确认一次不会发生的保存并把 `'save'`/
  `'load'` 留在字典里（对齐 `storage/CO_storage.c:38-40`）；挂上后端后同一写照旧
  生效（HOST-04）。新增 `SdoServer::handle_with_storage`（`handle` 行为不变）。
- **EMCY**：0x8130 心跳消费超时帧的厂商信息域补齐参考布局——字节 3
  `errorBit = 0x1B`、字节 4..7 为超时的 0x1016 表位索引（EMC-16，新
  `HeartbeatConsumer::entry_index`）；0x8250 的 info 由「配置窗口 ms」改为
  **实测静默 µs**，恢复帧回放同一实测值（EMC-17，新
  `EmcyProducer::clear_with_info` / `Node::clear_emergency_with_info`）。
- **文档**：CHANGELOG 补记 2026-09-24 批次 A~C2 十组（DOC-06）；EXTENSIONS
  §2.6 补 LSS-01/03/10 与 EDS-06 四条有意差异、`help` 移出已交付（DOC-07）；
  SDO/GTW/LED/EDS 的注释与 IMPLEMENTATION §5.3/§5.5 同步订正（批次 C3）。

### Fixed（2026-09-28 修复批次 E1；git 提交历史）

- **网关 · `[[<net>] <node>]` 前缀**：范围校验前移到**所有**动词之前，
  `[1] 300 set node 5`、`[1] 300 help` 等一律回 107，畸形前缀不再改写默认节点
  （GTW-21，同时补完 GTW-05 的残余）。
- **网关 · 未知动词**：首 token 非数字时它就是命令，`[1] banana` 由 `ERROR:101`
  改为参考的 `ERROR:100`（GTW-22）。
- **网关 · token 长度**：命令/序列 token 超过 19 字节、typed `write` 的数值
  字面量超过 14 字节（8/16/32 位）或 24 字节（64 位）时回 101（GTW-25）。
- **网关 · 序列头**：按参考「先赋值再判错」，`[1]]` 由回显旧序列改为
  `[1] ERROR:101`（GTW-25）。
- **网关 · `vs`**：以 `#` 开头的值、闭合引号后紧贴 `#`（`"a"#c`）均回 101，
  `"a" #c` 仍合法（GTW-23）。
- **网关 · `b` 字面量**：改走 C 基 0 文法，`b 010` 由写 10 改为写 8（GTW-24）。
- **网关 · typed `read`**：响应长度与请求宽度不符时回落 hex 输出（与缺省
  `hex` read 同格式），不再报 `ERROR:102`；abort/超时仍原样透传（GTW-15）。

### Fixed（2026-09-28 修复批次 E2；git 提交历史）

- **LSS 主站 · all-nodes 扫描**：`AllNodesScanner` 新增 `ScanAction::Deselect`
  与 `AllNodesScanner::on_deselected()`，在 configure+store 之后发送参考的
  `switch mode global: waiting`（非确认帧）。此前临时 node-id 存下后**永不生效**，
  被发现的节点仍停在 `0xFF`；现在从站请求通信复位并采纳新 id，节点真正离开
  未配置池（`CO_gateway_ascii.c:1879-1886`、`CO_LSSslave.c:56-66`，LSS-11）。
  ⚠ 驱动须在 `on_assigned()` 之后调用 `on_deselected()` 才会推进到下一 id。
- **LSS 主站 · 临时 id 计数**：自增改为饱和，池耗尽后不再回绕到 0（release）
  或在 debug 下溢出 panic，也不会再把越界 id 交给驱动（LSS-12）。
- **LSS · 身份双源**：新增 `LssAddress::from_od`，并写明 `LssAddress` 与
  0x1018 需由应用保持一致的约束（LSS-02）。

### Fixed（2026-09-28 修复批次 E3；git 提交历史）

- **EDS · 续行**：缩进更深的行按 configparser 语义以 `\n` 拼进上一条值，
  名字/默认值不再被静默截断（EDS-19）。
- **EDS · 重复键**：重复 option/section 改为记 `warnings`（last-wins 行为不变，
  python 会中止导入）（EDS-20）。
- **EDS · `ObjectType` 门禁**：`RECORD`/`ARRAY` 头的 `DataType`/`DefaultValue`
  等不再凭空造出可读 sub 0（python 只为 `VAR`/`DOMAIN` 建变量），标准键转入
  `custom_options` 以保 round-trip（EDS-02）。
- **EDS · UNICODE_STRING**：EDS 默认值与 SDO 读回统一为 UTF-16LE `Value::Bytes`
  （原为 `Value::Str` 与 `Value::Bytes` 两种形态，写一次 SDO 就漂移），
  运行时与 codegen 两出口一致（EDS-05、EDS-14）。
- **EDS · REAL 默认值**：按 python `float()` 文法解析，十六进制字面量与垃圾
  字面量记 warning 并按「无默认」处理（此前一个出口得 16.0、另一个得 0.0）
  （EDS-13）。
- **EDS · `$NODEID` 两侧偏移**：`1+$NODEID+2` 按 python 的拼接语义求值
  （node 5 时为 17），不再因此否决整份 EDS（EDS-15）。
- **EDS · `LowLimit`/`HighLimit`**：解析进 `EdsSubEntry.low_limit`/`high_limit`，
  有符号类型按二补码读上半区十六进制，非法字面量记 warning（EDS-12）。
- **EDS · 可见性**：无子项对象节、短 index 段、成对引号消费、引号内 `;` 注释
  四处宽松/丢弃面补 `warnings` 或 EXTENSIONS §2.6 留档（EDS-03、EDS-22、
  EDS-16、EDS-18）。

### Fixed（2026-09-28 修复批次 E4；git 提交历史）

- **SDO 服务端 · 缺失子项的 abort 码**：索引存在而子项缺失时一律 `0x0609_0011`
  （含 sub 0），只有索引本身不存在才是 `0x0602_0000`——仅含 sub 1..N 的对象
  读 `idx:0` 不再与读 `idx:5` 报不同码（SDO-34）。
- **SDO 服务端 · block 末段全 padding**：`noData` 覆盖全部已收字节时按畸形块
  abort `0x0604_0047`（`CO_SDO_AB_DEVICE_INCOMPAT`），不再被 `saturating_sub`
  折成「0 == 0」而接受空值（SDO-36）。
- **SDO 服务端 · 分段下载**：声明长度收满后再到的**空非末段**不再被 abort
  `0x0607_0012`，与两参考一致正常 ack（SDO-37）。
- **SDO 服务端 · 子块超时窗口**：0.7×T 改为先除后乘，超大 `set_sdo_timeout`
  值不再触发 debug 溢出 panic / release 回绕（SDO-39）。

### Fixed（2026-09-28 修复批次 E5；git 提交历史）

- **EMCY · 0x1014 禁用生产期间的帧不再丢失**：禁用只挡「此刻发不发」，帧照常入
  `pending_emcy`，出队循环改为先解析 COB-ID 再弹出——禁用期间整队冻结、计时不动，
  重新使能后原帧照发（EMC-15；同时落实 §3 EMC-08 的「备选 B」）。
- **EMCY · 进入 Operational 不再全量重发** active 条件：上一周期已发过的条件不会
  再发一遍；「Op 前锁存的故障要能被看到」这一目标改由 `pending_emcy` 承担，与参考
  「只出队未发送条目」一致（EMC-06）。
- **EMCY · 0x1003 历史项 register 冻结点**：保留在 `raise`（与参考出队时冻结的差异
  已留档于 EXTENSIONS §2.6），EMC-13 的「等价」措辞同步订正（EMC-18）。

### Fixed（2026-09-28 修复批次 E6a；git 提交历史）

- **PDO · 通信记录三处写时门禁**（`pdo::guard_comm_record_write`，均 abort
  `0x0609_0030`）：使能态**改 COB-ID**、使能态写 **inhibit time（sub3）**、使能态写
  **SYNC start value（sub6）** 一律拒写，对齐 CANopenNode `OD_write_14xx/18xx`
  （`CO_PDO.c:505-510,891-897,949-952,971-973`）。sub5 event timer 按参考保持
  运行期可写。主站须「先填记录、最后写 sub1 使能」（PDO-03）。
- **PDO · 缺 sub2 的 RPDO 记录**默认传输类型由 0（同步）改为 **254（立即应用）**，
  与 TPDO 方向及 CANopenNode 双向预置一致（`CO_PDO.c:654-655,1019-1020`）——消除
  「无 SYNC 生产者时最小 RPDO 永不落 OD」；显式 tt=0 仍等 SYNC（PDO-24）。
- **PDO · 只改计时器（sub3/sub5）不再清空槽位运行态**：重装配的合并判据改为按结构
  字段（COB-ID/传输类型/start value/mapping）比较，相位锁、挂起事件、SYNC 快照、
  死线记账与长度闩锁均保留（`CO_PDO.c:949-963`；PDO-09）。
- **PDO · sync start value 只在播种期消费**：新增 `TpdoSlot::ever_sent`，运行期把
  0x1019 由 0 改为非 0 不再把**已在发帧**的 cyclic TPDO 静默至多 240 个 SYNC
  （`CO_PDO.c:1315-1336`；PDO-26）。

### Fixed（2026-09-28 修复批次 E6b；git 提交历史）

- **RPDO · 长度条件改用参考的 `receiveError` 状态机**（`RxLenState`）：接收路径只
  改状态，编码由处理门发出——同步 RPDO 的 0x8210/0x8220 改到**下一次 SYNC** 才上
  总线，非同步 RPDO 仍在收帧当拍（`CO_PDO.c:717-728`；PDO-25、PDO-06①）。
- **RPDO · 长度条件为单一闩锁**：置位后不再出帧（`CO_error` 对同状态直接返回），
  「短帧 → 超长帧」产生**零**新帧，仅恰好等长的帧 ack 复位——不再多发 2 帧
  （`CO_Emergency.c:658-666`；PDO-06②/PDO-25）。
- **RPDO · 死线起算点改到应用时刻**：同步 RPDO 的死线从让数据可见的那次 SYNC 起算
  （参考只在 apply 循环内、`rpdoReceived` 时重装 `timeoutTimer`，
  `CO_PDO.c:740-828`；PDO-07）。
- **留档**：自回环抑制维持「TPDO 占用的 COB-ID 一律丢弃」的严格侧（PDO-08 +
  P3-22），与 python-canopen 的差异记入 EXTENSIONS §2.6。

### Fixed（2026-09-28 修复批次 E6c；git 提交历史）

- **NMT · Initialising 期间接受 Start**：不再静默丢弃与「应用启动竞态」相撞的主站
  命令（CiA 301 要求 Initialising 也接受 NMT 命令；两参考均无条件应用，
  `CO_NMT_Heartbeat.c:207-220` / python `nmt.py:57-66`）。Stop/EnterPreOp 与 Reset 语义不变
  （NMT-14）。
- **NMT · 重配消费超时即复位监督状态**：`HeartbeatConsumer::set_timeout` 连同
  `phase`/`state`/`last_seen` 复位（参考 `OD_write_1016 → CO_HBconsumer_initEntry`），
  「运行中把超时改小」不再立即打出伪 `Lost`（NMT-13）。
- **SYNC · 0x1005 写守卫补生产者标志判据**：bit30 不变时改 COB-ID 一律 abort
  `0x0609_0030`（`CO_SYNC.c:86-91`）。0x1005 无 disabled 位，清 bit30 才能改标识
  （SYN-11）。
- **TIME 消费**：仅 (Pre)Op 消费（`CO_TIME.c:155-172`）；RPDO 声明 0x100 时让位；
  TIME_OF_DAY 保留高 4 位在 decode 掩掉、encode 清零（`CO_TIME.c:163`）（SYN-09）。
- **SYNC 消费侧只取低 11 位**：EDS 预载的脏 0x1005（bit31/保留位脏）不再让节点对 SYNC
  失聪（参考 `CO_SYNC.c:274`；写时守卫仍拒脏值）（SYN-02）。
- **0x1280 补齐 DS-301 形状**：sub0=3、sub1/sub2（COB-ID client→server / server→client）
  与 sub3（node-id）均 rw，sub2 随节点号重编号（OD-01）。
- **补 0x100A** Manufacturer software version，并订正「`CO_ODinterface.h` 列为 required」
  的不实引证（OD-02）。
- **0x1001 打开 `pdo_mappable`**（DS-301 `PDOMapping=1`）：错误寄存器可映射进 TPDO，
  不再导致整槽静默消失（OD-03）。
- **测试**：`block_fallback_worthwhile` 单测（4 个「不支持 block」码 + 5 个确定性答复码），
  杜绝 EDS-04 的收窄被改回（HOST-05）。

### Added（2026-09-28 修复批次 E6c）

- `docs/IMPLEMENTATION.md` **§17「已知能力缺口（文档化不实现清单）」**：
  NMT-03（guarding life-time）、NMT-06（`NMTcontrol` 等价物）、SYN-10（0x1012/0x1013）、
  API-09（`store`/`restore` 存储组参数）四项按「文档化不实现」处置，实现时须先改该节。

### Fixed（2026-09-28 修复批次 F；git 提交历史）

- 6 处源码/文档中的 `docs/DESIGN-DECISIONS.md L3` 活指针改指权威来源（DOC-02）：`core/src/lss.rs` 4 处
  fastscan `bit_check` 合法域 → `CO_LSS_FASTSCAN_BITCHECK_VALID`（`CO_LSSslave.c:30,115-117`），
  `core/src/node.rs` 的「LSS 帧恰好 8 字节」→ `CO_LSSslave.c:45`，`core/src/gateway.rs`
  的 reset 前置 node id → `docs/DESIGN-DECISIONS.md GTW-04`；顺带订正把该宏归因到 `CO_LSS.h:133-134` 的错误
  （该头文件只有 `BIT31`/`CONFIRM` 常量）。
- 台账侧：DOC-01 复核销号、DOC-04 按「映射表已建成」留档销号、附录 B 销号 2 条并刷新
  引用完整性统计（仓内 `path:line` 引用 359 处全部可达，断链 0）。

### Added（2026-09-28 修复批次 F）

- 无（零行为变更批次）。

### Fixed（2026-09-28 修复批次 G —— 独立代码评审轮；git 提交历史）

本批共 **19 条新立目**（15 条行为修复 + 4 条留档），另 **2 处既有条目回炉**
（SDO-36 全 padding 守卫的 `s=0` 分支、PDO-03② 的门禁范围）与 **1 处注释面订正**
（EDS-29：EDS 告警统一带 subindex）。

高危（静默数据损坏 / 静默对象擦除）：

- **`hex` 网关写载荷跨空白配对**：`parse_hex_bytes` 跳过空白时不结算挂起的半字节，
  于是把下一组的第一位当作配对对象——`hex 0 1 2` 写进 `01 02`（参考 `00 01 02`）、
  `hex f f` 写进 `FF`（参考 `0F 0F`）。主站收 `OK`、设备里是**另一份数据**，
  无 abort 无告警。参考在空白处结算（`CO_fifo.c:1188-1203`）。回归 +3 例（GTW-26）。
- **EDS `ObjectType` 门禁用错容器码**：`is_container_object_type` 匹配
  `DEFTYPE 0x0005`/`DEFSTRUCT 0x0006`（python 从不建对象的两类型），真实容器是
  `ARRAY 0x0008`/`RECORD 0x0009`——门禁对不上，RECORD 段照旧造出可读 sub 0
  （即 EDS-02 要消灭的假 sub 0）。同时门禁改为**整节读完后再判定**（`finalize_object_types`），
  不再与 `ObjectType` 的行序耦合；被拒容器的 CiA 306 键折回 `custom_options`；
  带 `CompactSubObj` 的 ARRAY 豁免（位置差异留档 `EXTENSIONS.md` §2.6）。回归 3 个（EDS-23）。

中危：

- **SDO block download 全 padding 末段守卫回炉**：守卫原先只在「声明了长度」的分支内，
  而**不指示长度**（`s=0`）才是两参考客户端的常态（本栈 `encode_end` 从不置该位、
  python `sdo/client.py:806-809` 同样不置）——该分支仍 `truncate(0)` 并回 `0xA1`，
  把变长对象静默擦成空值。守卫上提到长度判定之前，对齐 `CO_SDOserver.c:908-916`
  的无条件先判（SDO-36）。
- **PDO inhibit 写时门禁收窄到 TPDO**：`0x1400:3` 在使能态曾被 `0x0609_0030` 拒绝，
  而 `OD_write_14xx` 根本没有 case 3（只处理 1/2/5）、DS-301 的 `[1400]` 也没有 sub 3
  （GTW-27 / PDO-03② 回炉）。
- **0x1016 重配不再波及其他消费者条目**：`set_timeout` 复位监督状态本身对齐参考，但
  重放会对**全部**子项生效——写一个无关子项就把其余打回 `Unknown`，真掉线的节点再也不报
  `Lost`。改为「超时值真的变了才复位」（幂等），对齐参考只重初始化被写条目
  （`CO_HBconsumer.c:93-94`）（NMT-15）。
- **EDS 复合 `DataType` 不再压过后出现的显式类型**：真实类型/未知类型两条路径都清除挂起索引，
  兑现本批的重复键 last-wins 规则（EDS-25）。
- **EDS 无效 `ParameterValue` 回落默认值**：python 只在解码成功时赋值 `var.value`
  （`eds.py:375-382`），本栈无条件优先它，`DefaultValue=01 A2` + `ParameterValue=zz`
  会把好的默认值丢掉。新增 `parameter_value_materialises()` 判定 + warning（EDS-26）。
- **EDS 整数条目上的非数字 `ParameterValue` 不再否决整份字典**：改为 warning + 回落
  `DefaultValue`（python 同样只是 warning），并给 `warn_invalid_defaults` 补整数档
  告警（EDS-27）。

低危：

- SDO-41：block download 的 `seqno == 0` 改按**期望**计数器分流（`CO_SDOserver.c:98`）——
  子块开始前静默忽略（参考的「sequence didn't start yet」），子块中途则结束子块并回
  ACK（此前静默丢弃，对端无从续传）。
- PDO-27：`ever_sent` 是**每 Operational 会话**标志，随 NMT 转换复位（参考每次非
  operational 重播种 `syncCounter = 255`，`CO_PDO.c:1351`）——此前 Stop→Start 后
  SYNC start-value 等待永久失效。
- PDO-28：PDO-09 的运行态合并补 `ever_sent`，一次 event-timer 改写不再重新武装等待。
- PDO-29：NMT 转换的接收态复位列表补 `rx_len` 闩锁，陈旧 `Latched` 不再压掉下一次
  0x8210/0x8220 长度错上报。
- OD-04：`0x1280:3` 按 DS-301 profile 改为 **UNSIGNED8 / 缺省 1**（`DS301_profile.eds:648-652`），
  DCF 的 1 字节写不再被 `0x0607_0007` 拒。
- SDO-42：`SdoServer` 的 `Default` 改为委托 `new()`，消除 `storage_enabled` 相反的
  公开类型陷阱。
- GTW-28：网关序列号改在「token 之后确有内容」时才赋值，对齐参考先 break 后赋值
  （`CO_gateway_ascii.c:611-615` vs `:633`）——一行裸 `[1]` 不再污染其后所有回显。
- HOST-07：`ParameterStorage::load` 的损坏消息并入 `first_err`，值级错误不再被行数盖掉。
- EDS-29：5 处子项告警统一带 subindex（`[{index}sub{sub}]`）。

同源追平（本轮自查发现的两处「第二现场」）：

- `host/src/codegen.rs` 有一份**独立**的 `ParameterValue`/`DefaultValue` 选择副本，
  不同步 EDS-26 会让**生成的模块与运行时字典在同一份 EDS 上给出不同初始值**
  （生成侧会对 `ParameterValue=zz` 生成 `panic!`/别的字面量）。已同步判据
  （`parameter_value_materialises` 提为 `pub(crate)`），回归
  `codegen.rs::invalid_parameter_value_falls_back_to_default_value`。
- EDS-23 的门禁后处理改为**只折叠「乐观创建」的 sub 0**：`[<index>sub0]` 段声明的
  sub 0 是真成员（python 对每个子段都建变量，与父 `ObjectType` 无关，
  `eds.py:156-171`），不能被容器门禁扫掉。解析循环用局部 `BTreeSet<(u16, u8)>`
  记录来源，不新增公开字段。回归
  `eds.rs::explicit_sub_zero_section_survives_the_container_gate`。
- 订正 EDS-02 的既有回归 `e3_record_and_array_headers_do_not_fabricate_sub_zero`：
  它钉的是 `0x0006`/`0x0005`（类型定义码）——**此前通过但钉错了东西**，
  已改用真容器码 `0x0009`/`0x0008`；同批订正 `EdsObject::object_type` 字段文档里的
  同一处错误常量。

低危·留档（行为不动，文档写明）：

- SDO-43：空闲通道上裸 `0xA1` 的 abort 地址取自帧头（end 帧即 CRC 字段），注释改为如实描述
  （参考无空闲分支可对照）。
- SDO-33：END 相位收到重复 `0xA2` 改为 abort `0x0504_0001`（与参考
  `UPLOAD_BLK_END_CRSP` 一致），显式记录由此失去的丢包恢复路径。
- HOST-06：v1 存储文件无 CRC，按字面信任；模块头写明该口径与「重存即升级 v2」。
- EDS-28：`LowLimit`/`HighLimit` 按该行出现时的数据类型解析（定型后重解需先保留 raw
  拼写 = 增量 API 决策）；目前无消费方，按预期债务留档。
