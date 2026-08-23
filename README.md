# canopen-rs

生产级 Rust CANopen（CiA 301/305/306）协议栈，面向 Linux（SocketCAN）。

```
┌──────────────────────────────────────────────────────┐
│  cli (canopen 二进制)                                 │
│    eds dump/gen · read/write · nmt · monitor         │
├──────────────────────────────────────────────────────┤
│  canopen-host (std)                                   │
│    SocketCan(阻塞) │ AsyncSocketCan(tokio feature)    │
│    Network/RemoteNode │ EDS 解析 │ OD 代码生成        │
│    Storage(FileStorage) │ Gateway(CiA309) │ Drive402  │
├──────────────────────────────────────────────────────┤
│  canopen-402 (CiA 402 驱动轮廓，sans-I/O)              │
│    State402 │ CwCommand │ 转移图 BFS │ Drive402      │
├──────────────────────────────────────────────────────┤
│  canopen-core (sans-I/O 协议核心)                     │
│    node │ sdo{server,client,block} │ pdo │ nmt        │
│    lss │ emcy │ leds │ gateway │ storage             │
│    sync_time │ object_dictionary │ standard           │
└──────────────────────────────────────────────────────┘
```

## 特性

- **SDO**：加速 / 分段 / block transfer（CRC-16/XMODEM、ACK 截断重传、自适应 blksize），server 端 initiate 时权限检查，完整 abort 码表
- **PDO**：子字节位级映射打包、从 OD 装配槽位（0x1400–0x1BFF）、SYNC acyclic/cyclic + sync start value 相位锁定、inhibit time / event timer、RTR 触发
- **NMT**：从站状态机、boot-up/心跳生产者、node guarding；主站心跳消费者四态机（Started/Lost/RemoteReset 事件携带具体 NodeId）
- **EMCY**：幂等 raise/clear、错误寄存器重算、历史环形队列与 0x1003 共享存储；消费者多回调 + panic 隔离 + active 表跟踪
- **LSS**：selective switch、configure/store/inquire、Fastscan 双端发现、AllNodesScanner 多节点枚举、bit-rate configure/activate；未配置节点只服务 LSS
- **EDS**：CiA 306 解析（`$NODEID` 惰性求值）→ 运行时 OD 或 build.rs 代码生成；CompactSubObj 展开；经 SDO 从设备 0x1021 拉取 EDS
- **Storage**：0x1010/0x1011 参数持久化（`ParameterStorage` trait + FileStorage 后端）
- **LED**：CiA 303-3 指示灯位型机（flicker/blink/flash 序列）
- **Gateway**：CiA 309-3 ASCII 网关最小切片（read/write/NMT/set node）
- **CiA 402**：独立 crate `canopen-402`——8 态状态机、编译期 BFS 转移路由、操作模式、homing 状态解码
- **Host**：阻塞 + tokio 异步双 transport、`Network` 订阅分发（回调隔离 + RX 健康检查）、`RemoteNode`（store/restore/write_int/start/stop）、EDS 解析与代码生成、CLI 工具

## 快速开始

```bash
cargo build --workspace
sudo tools/vcan_setup.sh          # 虚拟 CAN 接口（集成测试用）

# 主站：读设备类型、写参数、启动节点
canopen read vcan0 5 0x1000
canopen write vcan0 5 0x1017 --type u16 500
canopen nmt vcan0 start all
canopen monitor vcan0 -d 30

# EDS → 运行时字典 或 编译期代码生成
canopen eds dump device.eds
canopen eds gen device.eds -o od.rs --fn-name build_od
```

库用法（master）：

```rust
use canopen_core::types::Address;
use canopen_host::client::Network;

let net = Network::open("vcan0")?;
let node = net.node(5);
let dt = node.sdo_read(Address::new(0x1000, 0), DataType::Unsigned32)?;
node.write_int(Address::new(0x1017, 0), 500)?;
node.start()?;
net.monitor_heartbeat(NodeId::new(5)?, 1500);
for ev in net.heartbeat_events() { /* Started/Lost/… */ }
let eds = net.read_eds_from_device(5)?; // 经 0x1021 拉取设备自带 EDS
```

库用法（slave，见 `core/src/node.rs` 文档）：

```rust
let mut node = Node::new(node_id, standard_od)?;
if let Some(boot) = node.boot() { bus.send(boot.cob_id, boot.payload())?; }
loop {
    for tx in node.poll(now_ms()) { bus.send(tx.cob_id, tx.payload())?; }
    if let Some(rx) = bus.recv(10ms)? {
        for tx in node.on_frame(rx.cob_id, rx.payload()) { bus.send(..)?; }
    }
}
```

## 测试与质量

```bash
cargo test --workspace                              # 单测 + 集成 + pyport 回归（203 条，无总线依赖）
python3 tools/interop/python_canopen_oracle.py      # python-canopen 线格式对拍（11/11）
cargo test -p canopen-host --test vcan              # vcan0 实总线流程
cargo clippy --workspace --all-targets -- -D warnings
```

CI（GitHub Actions）运行以上全部；线格式以 [python-canopen](https://github.com/christiansandberg/canopen) 为独立 oracle 对拍。

真机交叉验证计划（四套件：我方主/从站 × CANopenLinux/python-canopen）见 `tests/interop/TESTPLAN.md`。

## 参考实现

- 架构调研与借鉴来源：`docs/REFERENCES.md`
- 实施规格与状态：`docs/IMPLEMENTATION.md`
- 后续拓展路线图：`docs/EXTENSIONS.md`

License: Apache-2.0 OR MIT
