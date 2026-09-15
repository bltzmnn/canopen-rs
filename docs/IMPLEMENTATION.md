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
// 输入一帧收到的报文 → 返回本帧触发的待发送帧
node.on_frame(cob_id: u16, data: &[u8]) -> Vec<TxFrame>
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
以毫秒数（`now_ms: u64`）传入，从不使用 `Instant`。这保证可测性并为未来
no_std 移植留门。

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
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    #[error("object {index:#06x}:{subindex} does not exist")]
    ObjectDoesNotExist { index: u16, subindex: u8 },
    #[error("object is read-only / write-only")] AccessDenied { addr: Address },
    #[error("type mismatch at {addr}: expected {expected:?}, got {actual:?}")]
    TypeMismatch { addr: Address, expected: DataType, actual: DataType },
    #[error("length mismatch: declared {declared}, got {actual}")]
    BadLength { declared: usize, actual: usize },
    #[error("segmented transfer toggle out of sequence")]
    ToggleBit,
    #[error("block transfer CRC mismatch")]
    CrcMismatch,
    #[error("invalid frame for current state")]
    UnexpectedFrame,
    #[error("encode buffer too small: need {needed} bytes, have {available}")]
    BufferTooSmall { needed: usize, available: usize },
    #[error("value {value} out of range [{min}, {max}]")]
    ValueOutOfRange { value: i64, min: i64, max: i64 },
    #[error("{0}")] Other(&'static str),
    #[error("SDO aborted: {0}")] SdoAborted(crate::sdo::SdoAbortCode),
    #[error("io: {0}")] Io(String),
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

`Value` 枚举承载具体值（`Bool(bool)`/`Int(i64)`/`Uint(u64)`/`Real(f64)`/
`Str(String)`/`Bytes(Vec<u8>)`）。约定：

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
    pub name: String,   // EDS 导入时保留名字（诊断用）
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
| 0x1003 | Pre-defined error field | sub0=个数（Rw，EMCY 运行时维护）、sub1..8 标准错误字段 `register<<16 \| code`（Ro） |
| 0x1005 | COB-ID SYNC | U32, rw |
| 0x1006 | Communication cycle period | U32, ro（SYNC 周期占位，无生产者） |
| 0x1008 | Manufacturer device name | VS, ro |
| 0x1009 | Manufacturer hardware version | VS, ro |
| 0x100C | Guard time | U16, ro（guard 未实现，占位） |
| 0x100D | Life time factor | U8, ro（同上） |
| 0x1010 | Save parameters | sub0=组数（ro）、sub1..N 命令（rw）：写 `'save'` 触发保存；非魔数 abort `0x0800_0020`（CANopenNode `OD_write_1010`） |
| 0x1011 | Restore default parameters | 同上，写 `'load'` 触发组恢复 |
| 0x1014 | COB-ID EMCY | U32, rw（bit31=禁用；低 11 位=0 也禁用；写守卫拒保留位/受限 CAN-ID/使能期间改 COB-ID，CANopenNode `OD_write_1014`） |
| 0x1015 | Emergency inhibit time | U16, rw（100 µs 单位） |
| 0x1016 | Consumer heartbeat time | U32, rw（见 §6.5） |
| 0x1017 | Producer heartbeat time | U16, rw |
| 0x1018 | Identity object | 结构体 sub1..4 ro |
| 0x1019 | Synchronous counter overflow | U8, rw |
| 0x1280 | SDO client parameter | U32, ro（占位） |

节点号相关 COB-ID 由 `Node::set_node_id()` / `apply_lss_node_id()` 重算：
仅当条目仍携带**旧节点号的预定义连接集值**（低 11 位匹配，含 EMCY 0x1014
与 0x1400+/0x1800+ sub1）时改写为 `预定义基址 | 新 node-id`，标志位保留；
自定义 COB-ID 一律不动（CANopenNode `CO_PDO.c:384-401` 同判据）。

---

## 5. SDO — 服务数据对象

目录：`core/src/sdo/{mod,server,client,block}.rs`

通道：请求 `0x600+node`，响应 `0x580+node`。三种传输模式：

### 5.1 加速传输（≤4 字节）

单帧往返：initiate request/response 中内联值。命令字节：

- upload（读）：req CCS=2(`0x40`)，resp SCCS=2，bit4 n/a、bits0-3 size；
- download（写）：req CCS=1，e=bit1、s=bit0、n 位于 bits 2..3（**不含**数据的字节数 = 4-len），resp SCCS=3(`0x60`)。

### 5.2 分段传输（>4 字节）

- initiate 阶段声明总长（可选，size-indicated）；**s=0（长度未指示）是合法的
  分段起始**（CiA 301 §7.2.4.3.2；CANopenNode `CO_SDOserver.c:789-793`、
  python-canopen `sdo/server.py:149-155` 均接受），未声明传输只受绝对上限
  `MAX_VALUE_LEN` 约束，末段即完成；
- 数据段每帧 ≤7 字节，**toggle 位交替**（起始 false），最后一段置 c 位；
- 任一方 abort 即终止，服务端清空 in-flight transfer 状态且不回包。

### 5.3 Block transfer（生产要求：完整集成）

文件 `sdo/block.rs`：codec + CRC-16/XMODEM（poly 0x1021, init 0）+
`BlockWriter`/`BlockReceiver` 组装器。**本栈将其接入状态机**：

- `SdoServer::Transfer` 增加 `BlockUpload`/`BlockDownload` 变体，
  `Node::on_frame` 正常路由 block 帧；
- 子块最大 `blksize ≤ 127` 段，每段 seqno 1..=127（7bit 循环）+ last 位；
- **blksize 协商**：客户端 block upload 提案恒为协议上限 127（与 python-canopen
  `sdo/client.py:461` 一致；CANopenNode v4 同样只在 download 方向读取服务端
  byte4 反提案——upload 方向不协商是两参考一致的设计决策，见 BUGS.md 已知局限）；
  服务端 `clamp_blksize` 将收到的提案夹到 1..=127，
  接收缓冲为动态 `Vec`（不做 CANopenNode 那种"缓冲将满收缩子块"的流控，
  见 BUGS.md 已知局限）；
- 每 sub-block 结束发一次 ack（含下一个 blksize 协商）；
- end 帧带 unused 字节数与 CRC；**end 帧仅在已对 last 位段回 ack 后才被识别**
  （末段 `0x80|seqno` 与 end `0xC1` 在 seqno≡1 mod 4 且 ≥65 时线格式重叠，
  相位分离对齐 CANopenNode `CO_SDOserver.c:906` 专用态，BUGS.md P1-1）。
  接收方 CRC 校验失败以 abort `0x05040004` 终止传输（CANopenNode 与
  python-canopen 同样不实现 CiA 的 bit2 整块重传路径）；
- 客户端策略：`> BLOCK_THRESHOLD(889 字节 = 127×7)` 自动选 block；block 被服务端
  拒绝时按错误上报（写方向不自动回退分段，需应用显式处理）。

### 5.4 Abort 码

完整实现 CiA 301 附录标准 abort 码表（见 §15.2），`SdoAbortCode` 枚举 +
`u32` 互转，未知码保留原值透传。

### 5.5 Server 并发与会话管理

单个 SDO 通道同一时刻只允许一个 in-flight transfer（规范行为）；收到新
initiate 时若已有 transfer 则 abort 旧 transfer (`SdoAbortCode::GeneralError`)
并开始新的。**会话超时**为单一失活窗口方案：server 记录最近活动戳
（`SdoServer::touch` 随每帧滑动），serve 循环经 `Node::sdo_timeout_due(now)`
驱动 `SdoServer::on_timeout`——在途传输超过 `timeout_ms`（默认 1500ms，由
serve 经 `set_sdo_timeout_if_unset` 安装、应用可覆盖）无任何帧即拆除，并向
对端发送 `0x05040000` abort（对象地址取在途 transfer，CANopenNode
`CO_SDOserver.c:1076-1079,1496-1509` 同）；长传输因窗口随帧滑动而不被总时长
误杀。权限检查在 initiate 时刻执行，不推迟到数据阶段。
多通道（多 SDO server，如 0x1200/0x1201）第一期不做，结构上预留。

### 5.6 Client 健壮性策略与流式 API

照抄 python-canopen 的客户端行为细节（`sdo/client.py:86-101,262-266`）：

- 超时重启前排空通道残留响应（2 ms 有界窗口），避免旧事务的迟到回复被
  当作新事务的应答；
- 超时后**主动向对端发送 ABORT** 再重试（默认重试 1 次），最终失败上抛；
- 校验响应中的 index/subindex 与请求一致；不一致时错误信息携带实际收到的
  地址（诊断"另一客户端占用通道"场景）；
- abort 码转成类型化 `SdoError::Aborted(SdoAbortCode)`，Display 附官方文案；
- 协议违规的本地 ABORT 使用专用码：toggle 0x05030000、长度 0x06070012/13、
  CRC 0x05040004、非法 blksize 0x05040002、非法命令 0x05040001（python-canopen
  `sdo/client.py:306-311`、CANopenNode `CO_SDOclient.c:624-631,1291-1303`）；
- block upload 客户端识别服务端降级：分段 initiate（0x40 族）切到普通分段读，
  **加速上传（0x43/0x47…，≤4 字节对象）直接完成**（python-canopen 服务端即如此，
  `sdo/server.py:113-118`；CANopenNode `CO_SDOclient.c:1348-1383`）。

流式 API（对标 python-canopen `SdoBase::open` 的 `impl Read/Write`
封装）为**路线图项**——当前公开面是整值 `sdo_read/sdo_write` 与
`read_domain_bytes`；§5.6 其余健壮性策略已实现（发起新事务前排空陈旧
响应、超时后向对端 ABORT 再重启重试一次、响应地址校验）：

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
Operational/PreOperational --StopRemoteNode(0x02)--> Stopped
（Initialising 中忽略 Stop；Stopped 重复 Stop 为 no-op）
Stopped --Start/EnterPreOp--> Operational/PreOperational
任意态 --ResetNode(0x81)/ResetComm(0x82)--> Initialising（应用层触发复位流程）
Initialising 中忽略 operational 命令；重复同态命令为 no-op
```

心跳判别值：Operational=0x05, Stopped=0x04, PreOperational=0x7F,
Initialising/boot-up=0x00。

### 6.2 心跳生产者

- boot() 后发一帧 boot-up（`0x700+id`, data `[0x00]`）；
- 周期由 OD 0x1017（ms）决定，`Node::poll(now_ms)` 内联心跳到期判断并产出帧；
- 首帧周期心跳在 boot 后首次 `poll` 的时刻起算一个完整周期（core 无时钟，
  不以调用方时间原点为基准），不会与 boot-up 背靠背连发；
- **写入 0x1017 立即发一帧心跳**（CANopenNode `OD_write_1017` 置
  `HBproducerTimer=0`，`CO_NMT_Heartbeat.c:64-66,187-189`）；之后从该帧起算
  下一个周期；

### 6.3 Node guarding（保留）

RTR 轮询响应：state 字节 + bit7 交替 toggle（`encode_node_guard`）。
toggle 在 boot/通信复位时归零，**NMT 状态切换不复位**（CANopenNode
`CO_Node_Guarding.c:178-197` 仅在 `slaveDisable` 分支复位；主站按严格交替校验）。
心跳生产者使能时（0x1017 ≠ 0）guarding **不应答**——二者是互斥的错误控制协议
（`CANopen.c:1384` 传 `slaveDisable = 0x1017 > 0`）。

### 6.4 主站侧

NMT master：`send_nmt(cmd, target)` 单帧发送；复位后自动等待 boot-up 报文
（可选，host 层实现 `wait_bootup(node, timeout)`（路线图））。

### 6.5 心跳消费者状态机（HeartbeatConsumer）

采用 CANopenNode `CO_HBconsumer.c:76-333` 的四态机，修正其"所有节点共用一个
emergency index"缺陷（W11）：

```
每被监视节点一个实例：
UNCONFIGURED --配置超时(OD 0x1016)--> UNKNOWN
UNKNOWN --收到心跳--> ACTIVE
ACTIVE  --超时未收到--> TIMEOUT（产生 NodeLost { node_id } 事件，
                        并清空已报告状态：state_of() 返回 None，恢复时
                        同状态重新产生 StateChanged，CANopenNode
                        `CO_HBconsumer.c:364-366,480-484`）
TIMEOUT --收到心跳--> ACTIVE
任意态  --收到 boot-up(state=0x00)--> UNKNOWN + RemoteReset { node_id } 事件
聚合查询：all_active() / all_operational()
```

- 超时配置来源：OD **0x1016**（u32：高字节 nodeId、低 u16 消费时间 ms），
  运行时写入即刻生效，含重复 nodeId 校验；
- **接收过滤按被监督节点逐个匹配**（`0x700+被监督 nodeId`，同
  CANopenNode 每条目 rx filter / python-canopen 逐节点订阅）：未被监督
  节点的 `0x700..=0x77F` 帧不进消费者，继续走 SYNC/RPDO 分发，自定义
  COB-ID 不被吞掉（原 BUGS.md P3 第三轮条目）；
- 事件携带具体 `NodeId` 的结构体（区别于 CANopenNode 只给 infoCode）；
- host 层由 RX 线程驱动并把事件推入**有界** `VecDeque`（容量 1024，满时丢最旧），
  应用经 `Network::heartbeat_events()` 轮询排空（无 mpsc；async 程序在 tokio
  runtime 内调用同一接口）；
- 超时容差：`HeartbeatConsumer::set_tolerance_pct(pct)`（clamp 100）——
  deadline = last_seen + consumer_time × (1 + pct/100)，默认 tolerance 0
  （严格），可配。

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
bit 偏移读写；同时保持对旧字节对齐数据的兼容。映射校验规则：条目位宽不得
超过目标类型宽度；**有符号类型必须按完整类型宽度映射**（窄域有符号字段无法
做符号扩展，负值会被误读为正值），无符号截断仍然合法（原 BUGS.md P2-1）。

### 7.2 COB-ID 与有效性

默认预定义连接集：TPDO1..4 = 0x180/280/380/480 + id；RPDO1..4 =
0x200/300/400/500 + id。通信参数（0x1400+/0x1800+）COB-ID 的 bit31 为 valid
位，**bit30 置位表示禁止 RTR**（CiA 301；与 python-canopen `RTR_NOT_ALLOWED = 1 << 30`、
CANopenNode `DS301_profile.md` 一致），bit29..11 必须为 0（11-bit ID 校验），
低 11 位为 0 的值在写入与装配两处均被拒绝。
本栈与 python-canopen 一致：**OD 中存储完整 COB-ID**（EDS `$NODEID+offset`
在构建时解析为完整值），不采用 CANopenNode 的"存裸 COB-ID、运行时加回"
方案。换节点号不破坏配置的等价能力由 `Node::set_node_id()` /
`apply_lss_node_id()` 提供：预定义形状的 COB-ID 条目按旧值匹配后改写
node-id 段（见 §4 表格下方说明），自定义值保持原样。

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
- SDO 写入 0x1400..0x1BFF 区间后自动重跑配置（通过 `SdoServer::take_last_written()`
  通知机制）；
- mapping 写入**不做** CANopenNode 式写时序列门禁（`ODR_UNSUPP_ACCESS`）：
  本栈在每次写入 `0x1400..=0x1BFF` 后即时重装配槽位，主站可用任意顺序配置
  （python-canopen 从站侧同样无此写时门禁）。COB-ID 低 11 位为 0 在写入与
  装配两处均被拒绝；valid COB-ID + 空 mapping 在装配期作为配置错误进入
  `config_errors` 环（CANopenNode 运行时对 `valid && mapped==0` 同样强制
  `valid=false` 并报错，`CO_PDO.c:623-627`）；
- **映射构造期校验**（`PdoMapping::push`/`validate_against`，修正 python-canopen
  >64bit 事后才 warn 的缺陷 V12）：总长 ≤64 bit、目标条目必须
   `pdo_mappable`、长度不超过目标类型——违规在构造时返回 Err 而非上线后
   （地址重叠检测已实现，BUGS.md P2-6）；
- **dummy entry**：index < 0x20 的映射项为填充字节，不读写 OD
  （CANopenNode `CO_PDO.c:320-328`）；
- 装配错误（含映射非法）进 `config_errors` 有界环，应用经
  `Node::take_config_errors` 获取（BUGS.md #7：不自动上报 EMCY——本栈
  SDO 写记录即动态重装配，配置期中间态会误报；CANopenNode 的
  PdoWrongMapping 是运行期收到错误 PDO 时才产生）。

### 7.4 传输类型与发送语义

`TransmissionType`: 0 同步acyclic、1..=240 同步cyclic（每 n 次 SYNC）、
252/253 RTR-only/sync 后 RTR、254/255 事件驱动。241..=251 保留拒收。
通信记录 sub 2 **缺失**时按方向取 CiA 缺省：RPDO → 0（同步）、TPDO →
254（事件驱动）（`PdoConfig::from_od(od, index, tt_default)`，BUGS.md
R10-P2-12）。

- **同步 TPDO**：观测到 SYNC 帧时经 `Node::on_sync(now)` 收集产出；cyclic
  N 分频计数 + sync start value 相位逻辑（S14）；SYNC 窗口外的挂起发送被
  拦截丢弃（对应 CANopenNode syncFlag 机制 S15）；
- **事件驱动 TPDO** 三种触发源：应用显式 `tpdo(index)`、OD 条目触发标志
  （flagsPDO 思想 S8）、event timer 周期到期；inhibit time 窗口内的请求
  挂起，窗口过后补发一次（不排队多次）；
- **RTR 触发**：收到 RTR 且 COB-ID bit30 **清零**（RTR allowed）时打包发送
  当前数据；bit30 置位（默认禁止 RTR）不应答；
- **同步 RPDO 双缓冲**（S5）：以 SYNC toggle 区分本周期/上一周期数据，
  严格实现"下一 SYNC 后才生效"。

### 7.5 RPDO 接收

仅在 Operational 处理；unpack 将各字段按当前 OD 类型解码写回；非法帧静默丢弃
（记 debug 日志计数，不产生总线流量）。自发送防环（本节点 TX 路径登记的
COB-ID 上的 RPDO 帧跳过）**已实现**（BUGS.md P3-22，见 `core/src/node.rs`）；
SocketCAN 默认回环开启时该机制已自动规避同 COB-ID 双配。
event timer 作为 RPDO 死线：armed（首次接收）后按时间戳比较——静默时长
`now - last_rx ≥ event_timer_ms` 才上报超时，每窗口至多一次；接收时刻在
poll 观察到流量时盖戳（core sans-I/O），分辨率为一个 poll 间隔。轮询落在
未到期窗口内绝不误报（原 BUGS.md P2 第三轮条目）。**只有长度合法的帧算流量**：
短帧（DLC < declared）报 0x8210 且不重置死线（CANopenNode 对短帧不置
`CANrxNew`，`CO_PDO.c:442-475`）。离开 Operational 时双向接收状态归零
（`sync_pending`/`rx_seen`/`last_rx_ms`/`reported`，CANopenNode
`CO_PDO.c:846-862`），Stop/Start 之间不残留旧样本与旧时间戳。

### 7.6 分层值访问（SDO/PDO 共用）——暂缓

python-canopen 的 `raw/phys/desc/bits` 分层值 API（源自
`objectdictionary/__init__.py:448-567` 与 `variable.py:14-136`）**本栈暂未
实现**，`EXTENSIONS.md` 已列为暂缓项：`Value` 直接以类型化枚举承载线上
值，factor/unit 缩放、value_descriptions、bit_definitions 由应用层自行处理。

---

## 8. SYNC / EMCY / TIME

文件：`core/src/{sync,emcy,time}.rs`

### 8.1 SYNC

- codec：COB-ID 默认 0x080（可被 0x1005 改配；bit30 为 gen 生产者标志，
  不影响消费，bit29..11 保留必须为零），可带 1 字节 counter；
- 从站消费：状态为 PreOperational/Operational 时校验 payload 宽度——0x1019=0
  仅接受无 counter 帧，≠0 仅接受恰 1 字节 counter 帧；宽度不符不驱动同步机制并
  上报 EMCY 0x8240（CANopenNode `CO_SYNC.c:37-51,386-392`）；仅 Operational
  才驱动缓冲 RPDO/TPDO；
- 主站生产：无内置周期任务，由应用按配置节拍直接发送 SYNC（`PeriodicTask`
  属路线图 §10.3）。

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
| 5 | device profile specific |
| 6 | 保留（恒 0） |
| 7 | manufacturer specific |

**生产者设计**（吸收 S9/S10，替换 volatile 为原子原语）：

- 内部维护**错误条件表**（`BTreeMap<code, Condition>` 条件折叠，core 单线程
   sans-I/O 无原子；通信信息类/通信致命类/通用类/厂商自定义分区，参照
   `CO_Emergency.h:181-240`），error register 每次从条件表**重算**而非逐位置位；
- `raise_emergency(code, condition)` / `clear_errors(condition)` **幂等**：
  条件已存在时不再入队发送；变化才产生 EMCY 帧；
- **历史环形队列**（**固定 8 条**，`HISTORY_CAPACITY`，不可配；消费者容量可配）：元素预格式化为
  `{code, register, info:[u8;5]}`——覆盖 EMCY 报文全部 8 字节（含完整
  5 字节厂商数据域），同时充当 OD 0x1003 的后备存储（一份存储两用）；
  **队满时丢弃新条目、保留已有历史**并置 overflow 标志
  （CANopenNode `CO_Emergency.c:695-706` 不淘汰最旧消息）；
  0x1003 每个子项为 CiA 301 标准错误字段 `register<<16 | code`
  （bits 16..23 为 error register；本栈无 errorBit 维度）；
  **溢出经 error register 的 generic 位报告，写 0x1003:0 清空历史时清除**
  （对应参考 fifo 排空后 `CO_EM_EMERGENCY_BUFFER_FULL` 复位，
  `CO_Emergency.c:597-603`）；
- error register 字节延迟到**发送时刻**合成当前值（保证读到最新状态）；
- 可选生产 inhibit（OD 0x1015，单位 100 µs）。

**消费者**（core 层 `EmcyConsumer`，修正 python 缺陷 V6）：

- 有界 ring（最近 8 条）+ 按 `(node, code)` 去重的 active 错误表；
- 事件流携带 `EmcyFrame { node_id, code, register }`，`EmcyCode` Display
  附官方文案查表；
- 节点丢失事件（来自 HB consumer）与 EMCY 流汇聚到同一监督通道。

### 8.3 TIME

`0x100`，6 字节（ms-of-day 小端 u32 + days since 1984 u16），codec 双向转换 +
`from_unix_ms`/`to_unix_ms` 主机时钟换算（core 只处理原始 6 字节，不引入
chrono/DateTime 依赖）。

---

## 9. LSS 与 Fastscan（CiA 305）

文件：`core/src/lss.rs`

固定 COB-ID：master→slave `0x7E5`，slave→master `0x7E4`。

### 9.1 Slave 状态机

```
Waiting --switch_mode(config)/selective 全匹配/fastscan 完成--> Configuration
Configuration --switch_mode(waiting)--> Waiting（若 active id 无效而 pending
  有效，则同时请求 NMT communication reset：节点自动 adopt pending id 并
  重新 boot，boot-up 帧随本次 on_frame 返回；CANopenNode
  `CO_LSSslave.c:56-66,275-280`）
Configuration 内服务：configure_node_id(校验 1..=127 或 0xFF)、
  configure_bit_timing、store(支持持久化时应用经 take_store_requested
  消费；不支持时应答 error 1)、inquire_identity/node_id
```

- 非法 switch_mode mode（非 0/1）整帧忽略：状态与 selective 进度都不变
  （CANopenNode `CO_LSSslave.c:68-70` 的 `default:`）；
- `activate_bit_timing` 的 16 位切换延时（ms）经
  `take_activate_bit_timing_delay()` 交给应用（`CO_LSSslave.c:337-349`）；
- selective switch 四连帧（vendor/product/revision/serial）逐段匹配
  0x1018 身份，全匹配才应答并进入 Configuration；
- configure_node_id 只置 pending，`adopt_pending()` 在节点复位时生效并重建
  SDO/PDO COB-ID；赋 `0xFF` 表示**使 node-id 无效**（CiA 305 §5.4.1，
  CANopenNode `CO_LSS_NODE_ID_VALID`）——adopt 后节点回到 LSS-init 态，
  只服务 LSS，直到重新分配；
- fastscan 仅由 **active 与 pending 均为 0xFF** 的 Waiting 节点应答
  （`CO_LSSslave.c:104-107`）；确认帧丢失且重试耗尽时整次扫描失败
  （`CO_LSSmaster.c:792-821` 的 `SCAN_NOACK`），不回报未验证地址；
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

文件：`host/src/{transport,async_transport,client}.rs`

### 10.1 传输类型（具体类型 `SocketCan`，非 trait）

传输层是**具体类型** `host::transport::SocketCan`（`&mut self` 方法，无
trait 抽象；async 对应 `host::async_transport::AsyncSocketCan`）：

```rust
pub struct SocketCan { /* socketcan::CanSocket */ }
impl SocketCan {
    pub fn open(interface: &str) -> Result<Self, HostError>;
    pub fn send(&mut self, cob_id: u16, data: &[u8]) -> Result<(), HostError>;
    pub fn recv(&mut self, timeout: Duration) -> Result<Option<Incoming>, HostError>;
}
```

- `Incoming = Data(Received) | Remote(u16)`——remote 帧**保留**（node
  guarding 需要）；仅扩展 29-bit ID 帧与 error 帧丢弃（`classify`）；
- `recv` 超时返回 `Ok(None)`（`read_frame_timeout` 传参，无
  `set_read_timeout` 状态配置）；
- `AsyncSocketCan`（feature=tokio）：基于 `socketcan::tokio::CanSocket`，
  超时用 `tokio::time::timeout`；
- 打开失败时给出友好错误（ENODEV → 提示 `ip link set up <iface>`）；
- 主站 `Network` 的订阅分发**不转发 remote 帧**（`Incoming::Remote` 在 RX 线程
  被跳过，node-guarding 观察属主站专用特性，BUGS.md P3-64）。

### 10.2 节点运行循环（从站模式）

阻塞版骨架（async 版等价，tokio select! 驱动）：

```rust
if let Some(boot) = node.boot() { bus.send(boot.cob_id, boot.payload()); }
loop {
    // poll 返回本周期应发送的帧；调用方按固定节拍提供单调 `now_ms()`
    for f in node.poll(now_ms()) { bus.send(f.cob_id, f.payload()); }
    match bus.recv(10ms) {
        Ok(Some(Incoming::Data(rx))) => {
            for f in node.on_frame(rx.cob_id, rx.payload()) { bus.send(f.cob_id, f.payload()) }
        }
        Ok(Some(Incoming::Remote(cob))) => { /* node guarding RTR */ }
        Ok(None) => continue, // timeout
        Err(e) => return Err(e),
    }
}
```
（伪码示意，以实际签名为准——`poll(now_ms)`/`recv(timeout)` 为真实接口名；
实际 serve 循环见 `host/src/sdo_exchange.rs` 宏单源。）

### 10.3 Network 与高层 client API（主站模式）

**已实现**（`host/src/client.rs`，阻塞版；tokio 版为路线图）：

```rust
let net = Network::open("vcan0")?;
// COB-ID → 回调注册表，精确匹配分发；单 handler panic 被隔离并计数
net.subscribe(0x085, |frame| { ... });   // 返回 ()，无 ?

let node = net.node(5);                          // RemoteNode 句柄
let v = node.sdo_read(Address::new(0x1017,0), DataType::Unsigned16)?;
node.sdo_write(Address::new(0x6040,0), &[0x06])?;
node.start()?;                                   // NMT 启动
node.store()?;                                   // 写 0x1010 保存参数
node.restore()?;                                 // 写 0x1011 恢复默认

network.check()?;                                // RX 线程死亡重抛
network.handler_panic_count();                   // 隔离 panic 计数
network.heartbeat_events();                      // Started/Lost/RemoteReset 事件
Network::shutdown_shared(&arc_net);              // Arc 场景停 RX 线程
```

要点：

- **RX 线程 + 回调直调分发**（无 mpsc）：订阅回调在 RX 线程上同步执行，
  per-handler `catch_unwind` 隔离 panic 并计入
  [`Network::handler_panic_count`]；handlers 先 clone 出注册表再解锁调用，
  回调内重入 subscribe/unsubscribe 不死锁；
- **命令面**（均经互斥命令 socket）：`sdo_read/sdo_write/send_nmt/
  gateway_line/read_eds_from_device` 等为 `Network` 方法；`write_int`/
  `store`/`restore` 属于 `RemoteNode` 句柄（`net.node(id).write_int(...)`），
  `read_domain_bytes` 属于 `SocketCan` 直连方法——勿在 `Network` 上查找；
- **心跳监督**：RX 线程喂 `HeartbeatConsumer`，事件入有界队列
  （1024 上限，溢出丢最老），应用经 `heartbeat_events()` 排空；
- **RX 线程契约**：非超时 IO 错误永久终止线程（经 `check()` 可见），
  慢回调会延迟心跳监督——两者均已在 API 文档注明；
- **路线图（未实现，勿按图索骥）**：async Network、mpsc 路由与背压、
  `PeriodicTask` 周期发送句柄、`NodeScanner` 主动/被动扫描（注意：与已实现的
  LSS `AllNodesScanner`（`core/src/lss.rs`）不同——后者是 LSS fastscan 协议
  实现，前者是路线图上的通用节点发现服务，二者无代码共享）、
  `wait_bootup`、`add_sdo(rx_cob, tx_cob)` 多 SDO 通道、
  `sdo()/pdo()/nmt()/emcy()` 组合子服务。

---

## 11. EDS 解析与代码生成

文件：`host/src/{eds,codegen}.rs`

### 11.1 EDS/DCF 解析

- INI 语法：`[<index>]`、`[<index>sub<subindex>]` 两级 section；
- 字段：ParameterName、DataType、AccessType(rw/ro/wo/const/rwr/rww)、
  DefaultValue（**ParameterValue 暂未解析**，仅 DefaultValue 进入 OD 构建）、
  PDOMapping、SubNumber（LowLimit/HighLimit 解析为路线图——Entry 尚无 limit 字段）；
- `$NODEID` 表达式（`$NODEID+0x200`）**惰性求值**（修正 python-canopen
  解析时一次性绑定的缺陷 V8）：解析产物保留
  `OdExpr::NodeRelative { offset: i64 }` 表达式，绑定实际
  node-id 时才求解——换节点号无需重解析 EDS；
- rwr/rww 映射为 Rw（区分 TPDO/RPDO 方向留给后续 DCF 增强）；
- CompactSubObj 展开时深拷贝（避免 python `eds.py:402-408` 的浅拷贝别名）；
  `[index]Name` 段的 `NrOfEntries` 提供展开条数、数字键提供子项名，
  显式 `[indexsubN]` 定义覆盖模板内容（python `eds.py:172-182` 的合并语义；
  无 `CompactSubObj` 时以 `[indexsub1]` 为模板）；
- 解析产物
  `Eds { objects: BTreeMap<u16, EdsObject>, configured_node_id: Option<OdExpr>, warnings: Vec<String> }`；
- 出口：①`Eds::to_object_dictionary(node_id)` 运行时构建 OD；②codegen
  编译期生成；③`export_eds()` 导出 EDS/DCF（**已裁决不做**，参照 python
  `eds.py:415`，见 §13）；④经 SDO 从设备 0x1021 取 EDS 导入——**已实现**
  （`Network::read_eds_from_device`）。

### 11.2 代码生成

`codegen::generate_od_module(eds: &Eds, fn_name: &str) -> Result<String, Error>`：输出 Rust 源文件，

```rust
pub fn {fn_name}(node_id: u8) -> canopen_core::ObjectDictionary { ... }
```

`fn_name` 须为合法小写 Rust 标识符（非法名直接返回 `Error::Other`，避免源码注入，BUGS.md P1-8；`_` 也按非法名拒绝）；生成的默认值与运行时 `EdsSubEntry::to_value` 走同一套范围校验，越界默认在生成的 `fn` 被调用时 panic、运行时解析时返回 `Error::ValueOutOfRange`（两端一致显式拒绝，与 `Value::encode_as` 语义相同，BUGS.md T1-2 / M2）。非有限浮点默认值（如 `1e999` → `f64::INFINITY`、`NaN`）生成 `f64::INFINITY`/`f64::NEG_INFINITY`/`f64::NAN` 字面量，不再输出非法源码（BUGS.md A3）；有符号类型的十六进制默认拒绝超出无符号范围的字面量（`INTEGER8 0x100` 报错，python `_signed_int_from_hex` 同）；`NaN`/`inf` 作为 FLOAT 默认被保留；VisibleString 的裸数字/node-relative 默认保留 EDS 原始拼写（`0x41` → `"0x41"`，python `eds.py:249-250`）。

用法：项目 `build.rs` 读设备 EDS → `OUT_DIR` 生成模块 → include。生成代码
全限定路径，不依赖用户 import。TOML 设备配置（zencan 风格）作为后续可选
增强，第一期以 EDS 为唯一输入源。

---

## 12. CLI 工具

二进制 `canopen`（clap 4）：

| 子命令 | 平台 | 功能 |
|---|---|---|
| `eds dump <file>` | 全平台 | 解析 EDS 打印摘要（CompactSubObj 按展开后的子项显示，与 `eds gen` 一致） |
| `eds gen <file> -o od.rs` | 全平台 | EDS → Rust OD 模块 |
| `read <iface> <node> <index> [sub] [--type T]` | Linux | SDO 读；缺省 `--type` 时按响应宽度解码显示 |
| `write <iface> <node> <idx> <val> [sub] [--type T]` | Linux | SDO 写；缺省类型时整数按最小宽度编码（`0x` 十六进制字面量可识别），超 i64 的数字报错而非静默降级为 ASCII |
| `nmt <iface> start\|stop\|preop\|reset\|reset-comm <node\|all>` | Linux | NMT 控制（含 `reset-comm` 复位通信） |
| `monitor <iface> [-d/--duration secs]` | Linux | 总线监听（默认 10s，0=直到中断）：COB-ID 分类（NMT/SYNC/EMCY/TIME/PDO/SDO/HB/LSS）+ 心跳表（每 5s）+ Lost 超时告警 |

数值参数支持 `0x` 十六进制与十进制。路线图：`lss discover`（core 的
FastscanMaster 已就绪，CLI 接线待做）。

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
（303 LED + 309-3 ASCII 网关最小版）；批次 C（CiA 402 驱动轮廓）已裁决移除，不在本栈范围。
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
   驱动 python-canopen **真实 `SdoClient` 状态机**（捕获其网络发送路径）产生
   请求帧，与本栈 golden 帧 diff——覆盖 SDO 加速下载、分段下载全流
   （initiate+各段）、block download initiate、upload request、abort 布局
   （地址域 python 置零/本栈回显，均为 CiA 合法，仅比对命令字节 + 32 位码）
   与 CRC-16/XMODEM（经 binascii.crc_hqx 双源验证）。NMT、EMCY、SYNC、PDO
   mapping、LSS/Fastscan 由金样字节测试与 pyport 套件覆盖（python-canopen
   对这些无离线编码器）。CI 必须全过。
3. **vcan0 集成测试**（`host/tests/vcan.rs`，需 root 建 vcan0；测试为
   `#[ignore]`，用 `-- --ignored` 运行）：
   - 本栈自回环（server 线程 + client 线程）——已实现；
   - 与 CANopenLinux `canopenDemo` 设备互通——计划（见 `tests/interop/TESTPLAN.md`）；
   - 与 oze-canopen 示例互通——计划。

质量门禁：

- `cargo clippy --workspace --all-targets -- -D warnings`
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
| 0x05040002 | 无效 blksize（block） |
| 0x05040003 | 无效 seqno（block） |
| 0x05040004 | CRC 校验失败（block） |
| 0x05040005 | 内存不足（block，预留） |
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
| 0x06090036 | 最大值小于最小值 |
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
| P4 SDO block | ✅ | sdo/block.rs 完整集成（CRC/重传/blksize 协商） |
| P5 NMT | ✅ | nmt.rs（状态机+心跳消费者四态机） |
| P6 PDO/EMCY/SYNC/TIME/Node | ✅ | pdo.rs（子字节位级打包）、emcy.rs、sync_time.rs、node.rs |
| P7 LSS/Fastscan | ✅ | lss.rs + Node unconfigured 模式集成 |
| P8 Host 层 | ✅ | transport.rs（阻塞）、async_transport.rs(tokio)、vcan0 测试(host/tests/vcan.rs，无 vcan0 时 `#[ignore]`) |
| P9 EDS/codegen/CLI | ✅ | eds.rs（$NODEID 惰性求值）、codegen.rs、CLI(eds/read/write/nmt/monitor) |
| + Network/RemoteNode | ✅ | host/src/client.rs（订阅分发/回调隔离/check 重抛/心跳监控） |
| + 批次 A Storage/EDS/LSS | ✅ | storage.rs、FileStorage、CompactSubObj、0x1021 拉取、AllNodesScanner、bit-rate |
| + 批次 B LED/网关 | ✅ | leds.rs（CiA 303-3 移植）、gateway.rs + Network::gateway_line |
| + 批次 C CiA 402 | ❌ 移除 | 裁决不实现 CiA 402；原 canopen-402 crate 已删除 |
| + 批次 D pyport | ✅ | pyport_{local,sdo_frames,lss_frames,emcy,nmt,pdo_sync}.rs、pyport_{eds,network}（host） |

验证：**394 tests 全绿**；clippy `-D warnings`；doc 无警告；python-canopen
oracle 11/11。vcan0 全流程测试需要 `sudo tools/vcan_setup.sh`（CI 环境）。

**架构约束：仅支持经典 CAN（11-bit ID、8 字节帧上限），CAN FD 永久不支持。**

- 真机交叉验证计划（四套件：我方主/从站 × CANopenLinux/python-canopen）：
  **`tests/interop/TESTPLAN.md`**
- python 测试套件移植（~56 条 Rust 回归测试，VirtualBus 进程内直驱）：
  **`docs/EXTENSIONS.md` §批次 D（pyport）**

后续路线图见 **`docs/EXTENSIONS.md`**。已裁决不做：EDS 导出、CAN FD、TOML
配置层；SRDO 与 CiA 315 观望。


