# canopen-rs 实现文档

一个面向 **Linux/std** 生产环境的 Rust CANopen（CiA 301）协议栈，含 SDO block
transfer、EDS 解析、OD 代码生成与 CLI 工具。异步支持通过 `tokio` feature 门控。

> 本文档是实现的唯一权威参考：模块边界、线格式、状态机、错误码、测试策略均在
> 此定义。实现过程中如与 CiA 301/305 规范冲突，以规范为准并回改本文档。

---

## 目录

1. [设计原则与总体架构](#1-设计原则与总体架构)
2. [Workspace 与 crate 划分](#2-workspace-与-crate-划分)
3. [核心数据模型](#3-核心数据模型)
4. [对象字典（OD）](#4-对象字典od)
5. [SDO — 服务数据对象](#5-sdo--服务数据对象)
6. [NMT — 网络管理与心跳](#6-nmt--网络管理与心跳)
7. [PDO — 过程数据对象](#7-pdo--过程数据对象)
8. [SYNC / EMCY / TIME](#8-sync--emcy--time)
9. [LSS 与 Fastscan（CiA 305）](#9-lss-与-fastscancia-305)
10. [Host 层：传输与高层 API](#10-host-层传输与高层-api)
11. [EDS 解析与代码生成](#11-eds-解析与代码生成)
12. [CLI 工具](#12-cli-工具)
13. [分阶段实施计划](#13-分阶段实施计划)
14. [测试与质量策略](#14-测试与质量策略)
15. [附录：COB-ID 分配表与 abort 码表](#15-appendix)

---

## 1. 设计原则与总体架构

### 1.1 Sans-I/O 核心协议栈

借鉴 `ref/canopen-rs` 已验证的架构：**core 不做任何 I/O**。契约：

```rust
// 输入一帧收到的报文 → 返回可选的待发送帧
node.on_frame(cob_id: u16, data: &[u8]) -> Option<TxFrame>
```

好处：

- 协议状态机可脱离总线独立单测（golden-byte 测试）；
- 阻塞与 tokio 异步两套 transport 共享同一份协议逻辑；
- 线格式可用 python-canopen 做 oracle 对拍。

`TxFrame` 为值类型：`cob_id: u16` + 最多 8 字节 data。

### 1.2 相对参考项目的生产化改进

| # | 改进点 | 参考项目现状 |
|---|--------|--------------|
| 1 | SDO block transfer **集成进** `SdoServer`/`SdoClient`/`Node::on_frame` | ref/canopen-rs 仅有裸 codec，未接入状态机 |
| 2 | OD 条目支持 **回调型读写**（`AccessFn`），std 环境用 `BTreeMap` 动态容量 | ref 用 heapless 定容 `LinearMap` |
| 3 | PDO 数量按 OD 配置不限 4 路；mapping 支持 **子字节打包** | ref 固定 4 路、仅字节对齐 |
| 4 | 心跳消费者带 **超时事件回调**（生产监控必需） | ref 仅查询式 `is_alive` |
| 5 | 统一 `thiserror` 错误体系，100% 公共 API 文档 | — |

> 每项设计决策的来源项目、参考位置与本栈改进形式的完整对照见
> **docs/REFERENCES.md §4 设计决策溯源总表**。核心策略：
> **CANopenNode 的协议行为语义 + python-canopen 的 API 形态，替换两者全部的
> 并发与内存架构**。

### 1.3 分层图

```
┌─────────────────────────────────────────────────────┐
│  cli (canopen 二进制)                                │
│    read/write/nmt/monitor/eds/codegen               │
├─────────────────────────────────────────────────────┤
│  host (std)                                          │
│    SocketCan(阻塞) │ AsyncSocketCan(tokio feature)   │
│    eds 解析 │ codegen │ HeartbeatMonitor             │
│    高层 client API (RemoteNode / Network)            │
├─────────────────────────────────────────────────────┤
│  core (sans-I/O, 无 unsafe, 不依赖 tokio/socketcan)  │
│    node │ sdo{server,client,block} │ pdo │ nmt       │
│    emcy │ sync │ time │ lss │ object_dictionary      │
│    datatypes │ types                                 │
└─────────────────────────────────────────────────────┘
```

core 虽运行于 std 环境，仍保持无 I/O、无线程、无时钟读取——所有时间由调用方
以 `Instant`/毫秒数传入。这保证可测性并为未来 no_std 移植留门。

---

## 2. Workspace 与 crate 划分

```
canopen-rs/
├── Cargo.toml              # [workspace], MSRV 1.75, edition 2021
├── core/Cargo.toml         # canopen-core
├── host/Cargo.toml         # canopen-host   (features: tokio)
├── cli/Cargo.toml          # canopen-cli    (bin: canopen)
└── docs/IMPLEMENTATION.md
```

依赖选型：

| crate | 用途 | 版本约束 |
|---|---|---|
| socketcan | SocketCAN（阻塞 + `tokio` feature） | 3.x |
| tokio | async runtime（仅 host 的 `tokio` feature） | 1, features=["net","time","rt","sync"] |
| thiserror | 错误派生 | 1 或 2 |
| clap | CLI | 4 |
| embedded-can | 帧类型桥接（可选保留，便于未来移植） | 0.4 |

feature 门控：

```toml
[features]
default = []
tokio = ["socketcan/tokio", "dep:tokio"]
```

非 Linux 平台：host 的 transport 模块整体 `#[cfg(target_os = "linux")]`，
eds/codegen/monitor 保持跨平台可编译。

---

## 3. 核心数据模型

### 3.1 错误体系（`core/src/types.rs`）

```rust
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("object {0:#06x}:{1} does not exist")]
    ObjectDoesNotExist(u16, u8),
    #[error("object is read-only / write-only")] AccessDenied,
    #[error("type mismatch: expected {expected:?}, got {actual:?}")]
    TypeMismatch { expected: DataType, actual: DataType },
    #[error("length mismatch: declared {declared}, got {actual}")]
    BadLength { declared: usize, actual: usize },
    #[error("segmented transfer toggle out of sequence")]
    ToggleBit,
    #[error("block transfer CRC mismatch")]
    CrcMismatch,
    #[error("invalid frame for current state")]
    UnexpectedFrame,
    // ...
}
pub type Result<T> = std::result::Result<T, Error>;
```

`NodeId(u8)`：newtype，合法域 `1..=127`，另含 `NodeId::UNCONFIGURED`(0xFF,
仅 LSS 内部使用) 与广播语义（NMT 目标为 0 时广播）。

### 3.2 数据类型与值编解码（`core/src/datatypes.rs`）

CiA 301 基础类型，discriminant 即规范中的数据类型索引：

```rust
#[non_exhaustive]
pub enum DataType {
    Boolean = 0x01, Integer8 = 0x02, Integer16 = 0x03, Integer32 = 0x04,
    Unsigned8 = 0x05, Unsigned16 = 0x06, Unsigned32 = 0x07, Real32 = 0x08,
    VisibleString = 0x09, OctetString = 0x0A, UnicodeString = 0x0B,
    TimeOfDay = 0x0C, TimeDifference = 0x0D, Domain = 0x0F,
    Integer24 = 0x10, Real64 = 0x11, Integer40 = 0x12, Integer48 = 0x13,
    Integer56 = 0x14, Integer64 = 0x15, Unsigned24 = 0x16,
    Unsigned40 = 0x18, Unsigned48 = 0x19, Unsigned56 = 0x1A, Unsigned64 = 0x1B,
}
impl DataType { pub fn fixed_size(self) -> Option<usize>; }
```

`Value` 枚举承载具体值（i8..i64/u8..u64/f32/f64/`ByteString`）。约定：

- CANopen 线上小端；
- `Value::encode_le(&mut buf) -> Result<usize>`；
- `Value::decode_le(DataType, &[u8]) -> Result<Value>`；
- 变长类型（字符串/DOMAIN）长度即字节数，上限 `MAX_VALUE_LEN = 65535`
  （std 环境，用 `Vec<u8>`/`String`，不再有 heapless 上限）。
- `Value::data_type() -> DataType` 用于 OD 写入时的类型校验。

### 3.3 地址

```rust
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Address { pub index: u16, pub subindex: u8 }  // Ord 派生 → BTreeMap 直接可用
```

---

## 4. 对象字典（OD）

文件：`core/src/object_dictionary.rs`

```rust
pub struct ObjectDictionary {
    entries: BTreeMap<Address, Entry>,
}

pub enum Storage {
    Value(Value),                          // 静态值
    Callback(Arc<dyn ObjectAccess + Send + Sync>),  // 回调型（CiA 401 DI 等）
}

pub trait ObjectAccess: Send + Sync {
    fn read(&self, addr: Address) -> Result<Value>;
    fn write(&self, addr: Address, value: Value) -> Result<()>;
}

pub struct Entry {
    pub name: heapless/String<64>,   // EDS 导入时保留名字（诊断用）
    pub data_type: DataType,
    pub access: AccessType,
    pub pdo_mappable: bool,
    pub storage: Storage,
}
```

要点：

- **访问控制每次访问都强制**：read 对 Wo 返回 `AccessDenied`；write 对 Ro 返回
  `AccessDenied`；类型不符返回 `TypeMismatch`。`Const` 可读不可写。
- 回调型条目使应用数据零拷贝进出 OD（如计数器直接映射硬件寄存器）。
- 写入回调条目失败时错误透传给 SDO 层转成对应 abort 码。
- `standard.rs` 提供 CiA 301 强制对象基线 builder：

| 索引 | 对象 | 说明 |
|---|---|---|
| 0x1000 | Device type | U32, ro |
| 0x1001 | Error register | U8, ro（EMCY 维护） |
| 0x1003 | Pre-defined error field | 数组, ro（EMCY 历史，sub0=个数） |
| 0x1005 | COB-ID SYNC | U32, rw |
| 0x1014 | COB-ID EMCY | U32, rw |
| 0x1017 | Producer heartbeat time | U16, rw |
| 0x1018 | Identity object | 结构体 sub1..4 ro |

节点号相关 COB-ID（0x1400+ PDO 等）在 `Node::set_node_id()` 时统一重算。

---

## 5. SDO — 服务数据对象

目录：`core/src/sdo/{mod,server,client,block}.rs`

通道：请求 `0x600+node`，响应 `0x580+node`。三种传输模式：

### 5.1 加速传输（≤4 字节）

单帧往返：initiate request/response 中内联值。命令字节：

- upload（读）：req CCS=2(`0x40`)，resp SCCS=2，bit4 n/a、bits0-3 size；
- download（写）：req CCS=1 + bit4 e=1 + bit5 n（有效字节数-1），resp SCCS=3(`0x60`)。

### 5.2 分段传输（>4 字节）

- initiate 阶段声明总长（可选，size-indicated）；
- 数据段每帧 ≤7 字节，**toggle 位交替**（起始 false），最后一段置 c 位；
- 任一方 abort 即终止，服务端清空 in-flight transfer 状态且不回包。

### 5.3 Block transfer（生产要求：完整集成）

文件 `sdo/block.rs`：codec + CRC-16/XMODEM（poly 0x1021, init 0）+
`BlockWriter`/`BlockReceiver` 组装器。**本栈将其接入状态机**：

- `SdoServer::Transfer` 增加 `BlockUpload`/`BlockDownload` 变体，
  `Node::on_frame` 正常路由 block 帧；
- 子块最大 `blksize ≤ 127` 段，每段 seqno 1..=127（7bit 循环）+ last 位；
- **blksize 自适应**（源自 CANopenNode `CO_SDOserver.c:1294-1347`）：
  blksize = min(127, 剩余容量/7)，接收缓冲将满时先 flush 已收数据到 OD 再
  收缩子块继续——流控不依赖固定缓冲尺寸；
- 每 sub-block 结束发一次 ack（含下一个 blksize 协商）；
- end 帧带 unused 字节数与 CRC，接收方校验失败回 end response 且置 CRC 错误位，
  发送方可整块重传（重传按 ACK 截断重放，参照 python-canopen
  `sdo/client.py:776-795`）；
- 客户端策略：`> MAX_SEGMENTED_LEN(887 字节)` 自动选 block（可通过配置强制
  关闭 block 退回分段）。

### 5.4 Abort 码

完整实现 CiA 301 附录标准 abort 码表（见 §15.2），`SdoAbortCode` 枚举 +
`u32` 互转，未知码保留原值透传。

### 5.5 Server 并发与会话管理

单个 SDO 通道同一时刻只允许一个 in-flight transfer（规范行为）；收到新
initiate 时若已有 transfer 则 abort 旧 transfer (`SdoAbortCode::GeneralError`)
并开始新的。**会话超时**（CANopenNode 双定时器方案，`CO_SDOserver.c:266-270`）：
段级 `timeout`（默认 1s）与子块级 `block_timeout = timeout × 0.7`，超时即
终止传输并清状态。权限检查在 initiate 时刻执行，不推迟到数据阶段。
多通道（多 SDO server，如 0x1200/0x1201）第一期不做，结构上预留。

### 5.6 Client 健壮性策略与流式 API

照抄 python-canopen 的客户端行为细节（`sdo/client.py:86-101,262-266`）：

- 发起新事务前清空通道上的陈旧响应；
- 超时后**主动向对端发送 ABORT** 再重试（默认重试 1 次），最终失败上抛；
- 校验响应中的 index/subindex 与请求一致；不一致时错误信息携带实际收到的
  地址（诊断"另一客户端占用通道"场景）；
- abort 码转成类型化 `SdoError::Aborted(SdoAbortCode)`，Display 附官方文案。

流式 API（对标 python-canopen `SdoBase::open`）：

```rust
// 阻塞版：返回 impl Read / impl Write，内部自动选择加速/分段/block
let mut r = client.open_read(0x2100, 1)?;      // 大对象读
r.read_exact(&mut buf)?;
let mut w = client.open_write(0x2100, 3, 4096)?; // 预知长度走 block
w.write_all(&data)?;

// tokio 版：impl AsyncRead / AsyncWrite
```

状态机实现要求：转移函数为纯函数 `(state, frame) -> (state, Option<TxFrame>)`
——吸取 CANopenNode 900 行巨型 switch（`CO_SDOserver.c:593-1520`）不可测试的
教训。

---

## 6. NMT — 网络管理与心跳

文件：`core/src/nmt.rs`、`core/src/node.rs`

### 6.1 从站状态机

```
Initialising --boot()--> PreOperational --StartRemoteNode(0x01)--> Operational
Operational --EnterPreOp(0x80)--> PreOperational
任意态 --StopRemoteNode(0x02)--> Stopped
Stopped --Start/EnterPreOp--> Operational/PreOperational
任意态 --ResetNode(0x81)/ResetComm(0x82)--> Initialising（应用层触发复位流程）
Initialising 中忽略 operational 命令；重复同态命令为 no-op
```

心跳判别值：Operational=0x05, Stopped=0x04, PreOperational=0x7F,
Initialising/boot-up=0x00。

### 6.2 心跳生产者

- boot() 后发一帧 boot-up（`0x700+id`, data `[0x00]`）；
- 周期由 OD 0x1017（ms）决定，`Node::heartbeat_due(now)` 返回到期判断 +
  `Node::heartbeat()` 产出帧；0x1017=0 表示关闭。

### 6.3 Node guarding（保留）

RTR 轮询响应：state 字节 + bit7 交替 toggle（`encode_node_guard`）。

### 6.4 主站侧

NMT master：`send_nmt(cmd, target)` 单帧发送；复位后自动等待 boot-up 报文
（可选，host 层实现 `wait_bootup(node, timeout)`）。

### 6.5 心跳消费者状态机（HeartbeatConsumer）

采用 CANopenNode `CO_HBconsumer.c:76-333` 的四态机，修正其"所有节点共用一个
emergency index"缺陷（W11）：

```
每被监视节点一个实例：
UNCONFIGURED --配置超时(OD 0x1016)--> UNKNOWN
UNKNOWN --收到心跳--> ACTIVE
ACTIVE  --超时未收到--> TIMEOUT（产生 NodeLost { node_id } 事件）
TIMEOUT --收到心跳--> ACTIVE
任意态  --收到 boot-up(state=0x00)--> UNKNOWN + RemoteReset { node_id } 事件
聚合查询：all_monitored_active() / all_monitored_operational()
```

- 超时配置来源：OD **0x1016**（u32：高字节 nodeId、低 u16 消费时间 ms），
  运行时写入即刻生效，含重复 nodeId 校验；
- 事件携带具体 `NodeId` 的结构体（区别于 CANopenNode 只给 infoCode）；
- host 层包装为 tokio 任务：`sleep_until(next_deadline)` 协作调度
  （源自 CANopenNode timerNext 思想），事件经 mpsc 上报；
- 超时容差：deadline = last_seen + consumer_time × (1 + tolerance)，默认
  tolerance 0（严格），可配。

---

## 7. PDO — 过程数据对象

文件：`core/src/pdo.rs`、`core/src/node.rs`

### 7.1 映射模型

```rust
pub struct MappingEntry { pub address: Address, pub bit_length: u8 }
// wire 格式 u32: index<<16 | subindex<<8 | bit_len

pub struct PdoMapping { entries: Vec<MappingEntry> }  // 总长 ≤64 bit
```

相对 ref 的改进：**支持子字节打包**（如 4×BOOL 共享 1 字节），pack/unpack 按
bit 偏移读写；同时保持对旧字节对齐数据的兼容。

### 7.2 COB-ID 与有效性

默认预定义连接集：TPDO1..4 = 0x180/280/380/480 + id；RPDO1..4 =
0x200/300/400/500 + id。通信参数（0x1400+/0x1800+）COB-ID 的 bit31 为 valid
位，bit30 为 RTR allowed，bit29..11 必须为 0（11-bit ID 校验）。
**OD 中存储去 node-id 的裸 COB-ID，装配时再加回**（CANopenNode
`CO_PDO.c:384-401`）——换节点号不破坏 PDO 配置。

通信参数对象完整语义（对照 python-canopen `pdo/base.py:353-399`、
CANopenNode `CO_PDO.c:953-1336`；单位全部强类型化，杜绝魔法数字）：

| sub | 含义 | 单位 | 适用 |
|---|---|---|---|
| 1 | COB-ID（含 valid/RTR 位） | — | 双方 |
| 2 | 传输类型 | 枚举 | 双方 |
| 3 | **Inhibit time**：发送后禁止再次发送的窗口 | **100 µs** | TPDO |
| 4 | 兼容保留 | — | — |
| 5 | **Event timer**：事件驱动的周期发送定时 | ms | TPDO 发送 / RPDO 死线监测 |
| 6 | **SYNC start value**：仅当 0x1019 counter overflow≠0 时生效，首个同步 TPDO 等 `SYNC.counter == start_value` 再按 N 分频发送 | 计数值 | 同步 TPDO |

### 7.3 数量与配置

- PDO 槽位数不硬编码 4：`configure_pdos_from_od()` 扫描 OD 中 0x1400..0x15FF
  （RPDO comm）、0x1600..0x17FF（RPDO map）、0x1800..0x19FF、0x1A00..0x1BFF，
  按 valid 位装配；
- SDO 写入 0x1400..0x1BFF 区间后自动重跑配置（通过 `take_written_object()`
  通知机制）；
- mapping 修改的安全序列遵循 CiA 301：sub0 置 0 → 改映射 → sub0 恢复个数；
  服务端在 mapping 计数为 0 时才允许修改映射项；COB-ID valid 置位时若
  映射为空则拒绝（`INVALID_VALUE` abort，CANopenNode `CO_PDO.c:507-509`）；
- **映射构造期校验**（`PdoMappingBuilder`，修正 python-canopen
  >64bit 事后才 warn 的缺陷 V12）：总长 ≤64 bit、无地址重叠、目标条目必须
  `pdo_mappable`、长度不超过目标类型——违规在构造时返回 Err 而非上线后；
- **dummy entry**：index < 0x20 的映射项为填充字节，不读写 OD
  （CANopenNode `CO_PDO.c:320-328`）；
- 映射非法时上报 EMCY（`PdoWrongMapping`，携带出错 map 值）。

### 7.4 传输类型与发送语义

`TransmissionType`: 0 同步acyclic、1..=240 同步cyclic（每 n 次 SYNC）、
252/253 RTR-only/sync 后 RTR、254/255 事件驱动。241..=251 保留拒收。

- **同步 TPDO**：观测到 SYNC 帧时经 `Node::on_sync(now)` 收集产出；cyclic
  N 分频计数 + sync start value 相位逻辑（S14）；SYNC 窗口外的挂起发送被
  拦截丢弃（对应 CANopenNode syncFlag 机制 S15）；
- **事件驱动 TPDO** 三种触发源：应用显式 `tpdo(index)`、OD 条目触发标志
  （flagsPDO 思想 S8）、event timer 周期到期；inhibit time 窗口内的请求
  挂起，窗口过后补发一次（不排队多次）；
- **RTR 触发**：收到 RTR 且 COB-ID bit30 允许时打包发送当前数据；
- **同步 RPDO 双缓冲**（S5）：以 SYNC toggle 区分本周期/上一周期数据，
  严格实现"下一 SYNC 后才生效"。

### 7.5 RPDO 接收

仅在 Operational 处理；unpack 将各字段按当前 OD 类型解码写回；非法帧静默丢弃
（记 debug 日志计数，不产生总线流量）。忽略自发送防环：本节点 TX 路径登记的
COB-ID 上的 RPDO 帧跳过（python-canopen `pdo/base.py:331-342` 思想）。
event timer 作为 RPDO 死线：到期未收到数据可产生事件（可选启用）。

### 7.6 分层值访问（SDO/PDO 共用）

OD 条目值解码分四层（源自 python-canopen `objectdictionary/__init__.py:448-567`
与 `variable.py:14-136`），SDO 变量与 PDO 映射变量共用同一套：

```rust
value.raw()   -> Value            // 线上原始值（小端字节）
value.phys()  -> f64 / typed      // factor/unit 缩放后
value.desc()  -> Option<String>   // value_descriptions 枚举文本
value.bits()  -> Option<BitField> // bit_definitions 位域访问
```

---

## 8. SYNC / EMCY / TIME

文件：`core/src/{sync,emcy,time}.rs`

### 8.1 SYNC

- codec：COB-ID 默认 0x080（可被 0x1005 改配，含 bit30/31 标志位解析），
  可带 1 字节 counter；
- 从站消费：收到即触发同步 TPDO；
- 主站生产：host 层周期发送任务（周期来自配置）。

### 8.2 EMCY

行为基准：CANopenNode `CO_Emergency.c`（S9/S10）+ python-canopen `emcy.py`。

**Error register 标准位定义**（CiA 301，0x1001）：

| bit | 含义 |
|---|---|
| 0 | generic |
| 1 | current |
| 2 | voltage |
| 3 | temperature |
| 4 | communication |
| 5 | device profile specific（预留） |
| 6..7 | 保留 |

**生产者设计**（吸收 S9/S10，替换 volatile 为原子原语）：

- 内部维护**错误条件位图**（`AtomicBool` 数组/位集，通信信息类/通信致命类/
  通用类/厂商自定义分区，参照 `CO_Emergency.h:181-240`），error register
  每次从条件位图**重算**而非逐位置位；
- `raise_emergency(code, condition)` / `clear_errors(condition)` **幂等**：
  条件已存在时不再入队发送；变化才产生 EMCY 帧；
- **历史环形队列**（默认 8 条，容量可配）：元素预格式化为
  `{code:u32, info:u32}`——恰为 EMCY 报文前 8 字节，同时充当 OD 0x1003 的
  后备存储（一份存储两用）；队列满只置 overflow 标志不入队，
  **溢出自身作为错误条件报告并在恢复时自动清除**；
- error register 字节延迟到**发送时刻**合成当前值（保证读到最新状态）；
- 可选生产 inhibit（OD 0x1015，单位 100 µs）。

**消费者**（host 层 `EmcyListener`，修正 python 缺陷 V6）：

- 有界 ring（最近 8 条）+ 按 `(node, code)` 去重的 active 错误表；
- 事件流携带 `EmcyFrame { node_id, code, register }`，`EmcyCode` Display
  附官方文案查表；
- 节点丢失事件（来自 HB consumer）与 EMCY 流汇聚到同一监督通道。

### 8.3 TIME

`0x100`，6 字节（ms since epoch + days since 1984），codec 双向转换 +
`DateTime` 辅助（host 层用 chrono 可选 feature，core 只处理原始 6 字节）。

---

## 9. LSS 与 Fastscan（CiA 305）

文件：`core/src/lss.rs`

固定 COB-ID：master→slave `0x7E5`，slave→master `0x7E4`。

### 9.1 Slave 状态机

```
Waiting --switch_mode(config)/selective 全匹配/fastscan 完成--> Configuration
Configuration --switch_mode(waiting)--> Waiting
Configuration 内服务：configure_node_id(校验 1..=127)、
  configure_bit_timing、store(回调持久化)、inquire_identity/node_id
```

- selective switch 四连帧（vendor/product/revision/serial）逐段匹配
  0x1018 身份，全匹配才应答并进入 Configuration；
- configure_node_id 只置 pending，`adopt_pending()` 在节点复位时生效并重建
  SDO/PDO COB-ID；
- 未配置节点（`enable_lss_unconfigured(addr)` 启动）只服务 LSS，忽略其余帧。

### 9.2 Fastscan master

按 §ref/canopen-rs 已验证算法：对 4 个 32 位身份字段逐个 MSB-first 二分探测，
slave 仅在候选高段仍匹配自身身份时回 identify-slave 帧。接口：

```rust
let mut fs = FastscanMaster::new();
while let Some(req) = fs.next_request() {
    let answered = send_and_wait_identify(req);   // host 层
    fs.on_response(answered);
}
assert!(fs.found());
```

---

## 10. Host 层：传输与高层 API

文件：`host/src/{transport,async_transport,nmt_monitor,client}.rs`

### 10.1 Transport trait

```rust
pub trait CanBus: Send {
    fn send(&self, cob_id: u16, data: &[u8]) -> std::io::Result<()>;
    fn recv(&self, timeout: Duration) -> std::io::Result<(u16, [u8; 8], usize)>;
}
```

- `SocketCan`：基于 `socketcan::CanSocket`，`set_read_timeout`；
- `AsyncSocketCan`（feature=tokio）：基于 `socketcan::tokio::CanSocket`，
  超时用 `tokio::time::timeout`；
- 打开失败时给出友好错误（ENODEV → 提示 `ip link set up <iface>`）；
- recv 过滤 remote/error 帧与 29-bit ID。

### 10.2 节点运行循环（从站模式）

阻塞版骨架（async 版等价，tokio select! 驱动）：

```rust
let tx = node.boot(); bus.send(tx.cob_id, tx.data());
loop {
    // 协作式调度：poll 返回下次事件到期时间（CANopenNode timerNext 思想 S6）
    let next = node.poll_heartbeat(now());
    match bus.recv_until(next) {
        Ok((cob_id, buf, len)) => {
            if cob_id == sync_cob { for f in node.on_sync(now()) { bus.send(..) } }
            else if let Some(tx) = node.on_frame(cob_id, &buf[..len]) { bus.send(..) }
        }
        Err(Timeout) => continue,
    }
}
```

### 10.3 Network 与高层 client API（主站模式）

API 形态对标 python-canopen（P1-P6），运行时模型替换为单 async RX 任务 +
mpsc 路由：

```rust
let net = Network::open("vcan0")?;
// COB-ID → 回调注册表，精确匹配分发；handler 返回 Result，
// 错误走监督通道而非静默吞掉（修正 V11）
net.subscribe(0x085, |frame| { ... })?;

let node = net.add_node(5)?;                    // RemoteNode
let v: u16 = node.sdo().read(0x1017, 0).await?; // 子服务句柄
node.sdo().write(0x6040, 0, 0x06u16).await?;
node.nmt().start().await?;
node.store()?;                                   // 写 0x1010 保存参数
node.restore(1)?;                                // 写 0x1011 恢复默认

let mut scan = NodeScanner::new(&net);           // 被动+主动发现
scan.passive(Duration::from_secs(5));            // 监听 HB/SDO-resp/TPDO/EMCY 段
let found = scan.active_probe(1..=127).await;    // 广播 SDO 探测

net.wait_bootup(5, Duration::from_secs(2)).await?;
```

要点：

- **RX 任务隔离**（P2/P3）：单个订阅 handler 的错误不杀死接收任务；
  RX 任务 panic 被捕获，`Network::check()` 可显式重抛；
- **RemoteNode 组合式子服务**：`sdo()/pdo()/nmt()/emcy()` 句柄；支持
  `add_sdo(rx_cob, tx_cob)` 多通道（P5），每通道独立事务状态与互斥；
- SDO 并发事务按 (node, channel) 排队互斥，响应以 index/subindex 校验关联
  （修正 oze-canopen 粗粒度 per-node mutex）；
- `PeriodicTask` 句柄化周期发送（SYNC/TPDO/TIME）：`start(period)` 返回可
  stop/update 的句柄，更新数据**不重启循环**（修正 python V10）；
- 全栈时间一律 `std::time::Instant` 单调时钟（修正 V10 墙钟混用）。

---

## 11. EDS 解析与代码生成

文件：`host/src/{eds,codegen}.rs`

### 11.1 EDS/DCF 解析

- INI 语法：`[<index>]`、`[<index>sub<subindex>]` 两级 section；
- 字段：ParameterName、DataType、AccessType(rw/ro/wo/const/rwr/rww)、
  DefaultValue/ParameterValue、PDOMapping、LowLimit/HighLimit、SubNumber；
- `$NODEID` 表达式（`$NODEID+0x200`）**惰性求值**（修正 python-canopen
  解析时一次性绑定的缺陷 V8）：解析产物保留
  `OdValue::NodeRelative { base: i64, offset: i64 }` 表达式，绑定实际
  node-id 时才求解——换节点号无需重解析 EDS；
- rwr/rww 映射为 Rw（区分 TPDO/RPDO 方向留给后续 DCF 增强）；
- CompactSubObj 展开时深拷贝（避免 python `eds.py:402-408` 的浅拷贝别名）；
- 解析产物 `Eds { objects: BTreeMap<u16, ObjectDescription> }`；
- 出口：①`Eds::to_object_dictionary(node_id)` 运行时构建 OD；②codegen
  编译期生成；③`export_eds()` 导出 EDS/DCF（路线图，参照 python
  `eds.py:415`）；④经 SDO 从设备 0x1021 取 EDS 导入（路线图）。

### 11.2 代码生成

`codegen::generate_od_module(&eds, crate_name) -> String`：输出 Rust 源文件，

```rust
pub fn build_object_dictionary() -> canopen_core::ObjectDictionary { ... }
```

用法：项目 `build.rs` 读设备 EDS → `OUT_DIR` 生成模块 → include。生成代码
全限定路径，不依赖用户 import。TOML 设备配置（zencan 风格）作为后续可选
增强，第一期以 EDS 为唯一输入源。

---

## 12. CLI 工具

二进制 `canopen`（clap 4）：

| 子命令 | 平台 | 功能 |
|---|---|---|
| `eds dump <file>` | 全平台 | 解析 EDS 打印摘要 |
| `eds gen <file> -o od.rs` | 全平台 | EDS → Rust OD 模块 |
| `read <iface> <node> <index> [sub]` | Linux | SDO 读（自动模式选择） |
| `write <iface> <node> <idx> <val>` | Linux | SDO 写 |
| `nmt <iface> start\|stop\|reset <node\|all>` | Linux | NMT 控制 |
| `monitor <iface>` | Linux | 总线监听：COB-ID 分类（NMT/SYNC/EMCY/TIME/PDO/SDO/HB/LSS）+ 心跳表 + 超时告警 |
| `lss discover <iface>` | Linux | Fastscan 发现未配置节点并可分配 node-id |

数值参数支持 `0x` 十六进制与十进制。

---

## 13. 分阶段实施计划

每阶段结束必须全绿（单测 + clippy -D warnings + doc 无警告）再进入下一阶段。

| 阶段 | 内容 | 验证出口 |
|---|---|---|
| P1 | workspace 脚手架；types/datatypes（Error/NodeId/DataType/Value 小端 codec） | golden-byte 单测 |
| P2 | 对象字典（BTreeMap + 回调条目 + standard 基线） | 单测（访问控制矩阵） |
| P3 | SDO 加速+分段：server/client/abort 全表 | CS 互测 + python-canopen oracle 对拍 |
| P4 | SDO block transfer 集成进状态机 | 大数据传输测试 + oracle |
| P5 | NMT 状态机 + 心跳/bootup/guarding + HeartbeatMonitor | 单测 |
| P6 | PDO（子字节打包、OD 配置、SYNC 触发）+ EMCY + TIME | vcan0 回环全流程 |
| P7 | LSS slave + Fastscan master | 双节点发现/分配测试 |
| P8 | host transports（阻塞+tokio）+ 高层 client API | 与 CANopenDemo 互通冒烟 |
| P9 | EDS 解析 + codegen | 真实厂商 EDS 文件测试 |
| P10 | CLI | 手工验收清单 |

后续路线图（第一期之后）：见 **`docs/EXTENSIONS.md`**——批次 A（Storage
持久化、EDS 导入增强含 0x1021 拉取与 CompactSubObj 展开、LSS 补缺）、批次 B
（303 LED + 309-3 ASCII 网关最小版）、批次 C（`canopen-402` 驱动轮廓 crate）。
已裁决不做：EDS 导出、CAN FD（本栈仅支持经典 CAN）、TOML 配置层；SRDO 与
CiA 315 观望。

里程碑演示脚本：P6 后跑通「vcan0 上 LSS 分配节点号 → SDO 配置 PDO → NMT 启动
→ SYNC 驱动 TPDO」端到端。

---

## 14. 测试与质量策略

三层验证（沿用 ref/canopen-rs 成熟方案）：

1. **Golden-byte 单测**：每个 codec 模块断言手工核实的字节序列；server/client
   互为对手做状态机穷举测试。
2. **python-canopen oracle**（`tools/interop/python_canopen_oracle.py`）：
   驱动 python-canopen 真实编码器离线生成期望帧，与本栈输出 diff——覆盖
   SDO 三种传输、NMT、EMCY、SYNC、PDO mapping、LSS/Fastscan、CRC 校验值。
   CI 必须全过。
3. **vcan0 集成测试**（`tests/vcan/`，需 root 建 vcan0，CI 中 skip-if-missing）：
   - 本栈自回环（server 线程 + client 线程）；
   - 与 CANopenLinux `canopenDemo` 设备互通；
   - 与 oze-canopen 示例互通。

质量门禁：

- `cargo clippy --workspace -- -D warnings`
- `#![deny(unsafe_code)]` 于 core 与 host
- `cargo doc --no-deps` 无警告；公共 API 文档率 100%
- oracle 脚本退出码非零即 CI 失败

---

## 15. Appendix

### 15.1 COB-ID 预定义分配（预连接集）

| COB-ID | 用途 |
|---|---|
| 0x000 | NMT |
| 0x080 | SYNC（可配 0x1005） |
| 0x100 | TIME |
| 0x080+id | EMCY |
| 0x180,0x280,0x380,0x480 +id | TPDO1..4 默认 |
| 0x200,0x300,0x400,0x500 +id | RPDO1..4 默认 |
| 0x580+id | SDO server→client (tx) |
| 0x600+id | SDO client→server (rx) |
| 0x700+id | 心跳/boot-up/guarding |
| 0x7E4 / 0x7E5 | LSS slave→master / master→slave |

### 15.2 SDO abort 码（必实现全集）

| Abort 码 | 含义 |
|---|---|
| 0x05030000 | toggle 未交替 |
| 0x05040000 | SDO 超时 |
| 0x05040001 | 无效命令说明符 |
| 0x06010000 | 不支持该对象访问 |
| 0x06010001 | 读 WO 对象 |
| 0x06010002 | 写 RO 对象 |
| 0x06020000 | 对象不存在 |
| 0x06040041 | 对象不能映射到 PDO |
| 0x06040042 | 映射数量/长度超出 |
| 0x06040043 | 参数通用不兼容 |
| 0x06040047 | 内部不兼容 |
| 0x06060000 | 访问失败（硬件错误） |
| 0x06070010 | 类型不匹配 |
| 0x06070012/13 | 类型不匹配（长度过高/过低） |
| 0x06090011 | subindex 不存在 |
| 0x06090030 | 值超出范围 |
| 0x06090031 | 值过高 |
| 0x06090032 | 值过低 |
| 0x060A0023 | 资源不可用 |
| 0x08000000 | 通用错误 |
| 0x08000020 | 数据无法传输/存储 |
| 0x08000021/22/23 | 因本地控制/设备状态/无 OD 而失败 |
| 0x08000024 | 无可用 OD |

### 15.3 参考实现索引

| 主题 | 参考 |
|---|---|
| 整体架构与 Node::on_frame 契约 | ref/canopen-rs/docs/ARCHITECTURE.md |
| SDO/NMT/PDO/LSS 线格式 | ref/canopen-rs/core/src/ |
| 工业级行为基准 | ref/CANopenLinux（CANopenNode CO_* 模块） |
| async 收发模式 | ref/oze-canopen/src/ |
| OD 代码生成 | ref/zencan/zencan-build/ |
| oracle 工具库 | ref/canopen（python-canopen） |

---

## 16. 实施状态（2026-08）

| 阶段 | 状态 | 交付物 |
|---|---|---|
| P1 数据模型 | ✅ | core/{types,datatypes,sdo(SdoAbortCode)} |
| P2 对象字典 | ✅ | object_dictionary.rs（回调条目+set_value）、standard.rs |
| P3 SDO 加速/分段 | ✅ | sdo/{mod,server,client}.rs + oracle 对拍 |
| P4 SDO block | ✅ | sdo/block.rs 完整集成（CRC/重传/自适应 blksize） |
| P5 NMT | ✅ | nmt.rs（状态机+心跳消费者四态机） |
| P6 PDO/EMCY/SYNC/TIME/Node | ✅ | pdo.rs（子字节位级打包）、emcy.rs、sync_time.rs、node.rs |
| P7 LSS/Fastscan | ✅ | lss.rs + Node unconfigured 模式集成 |
| P8 Host 层 | ✅ | transport.rs（阻塞）、async_transport.rs(tokio)、vcan0 测试(skip-if-missing) |
| P9 EDS/codegen/CLI | ✅ | eds.rs（$NODEID 惰性求值）、codegen.rs、CLI(eds/read/write/nmt/monitor) |
| + Network/RemoteNode | ✅ | host/src/client.rs（订阅分发/回调隔离/check 重抛/心跳监控） |
| + 批次 A Storage/EDS/LSS | ✅ | storage.rs、FileStorage、CompactSubObj、0x1021 拉取、AllNodesScanner、bit-rate |
| + 批次 B LED/网关 | ✅ | leds.rs（CiA 303-3 移植）、gateway.rs + Network::gateway_line |
| + 批次 C CiA 402 | ✅ | canopen-402 crate（状态机/转移图 BFS/mode/homing/Drive402）+ host SDO glue |
| + 批次 D pyport | ✅ | pyport_{local,sdo_frames,lss_frames,emcy,nmt,pdo_sync}.rs、pyport_{eds,network}（host） |

验证：**203 tests 全绿**；clippy `-D warnings`；doc 无警告；python-canopen
oracle 11/11。vcan0 全流程测试需要 `sudo tools/vcan_setup.sh`（CI 环境）。

**架构约束：仅支持经典 CAN（11-bit ID、8 字节帧上限），CAN FD 永久不支持。**

- 真机交叉验证计划（四套件：我方主/从站 × CANopenLinux/python-canopen）：
  **`tests/interop/TESTPLAN.md`**
- python 测试套件移植（~56 条 Rust 回归测试，VirtualBus 进程内直驱）：
  **`docs/EXTENSIONS.md` §批次 D（pyport）**

后续路线图见 **`docs/EXTENSIONS.md`**。已裁决不做：EDS 导出、CAN FD、TOML
配置层；SRDO 与 CiA 315 观望。

LSS 补缺）→ 批次 B（LED + 309 网关）→ 批次 C（CiA 402 crate）。已裁决不做：
EDS 导出、CAN FD、TOML 配置层；SRDO 与 CiA 315 观望。
