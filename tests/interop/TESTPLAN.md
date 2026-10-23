# 交叉验证测试计划（Cross-Validation Test Plan）

> **实现状态**（2026-10-23）：本计划已在两块 T536 开发板上完成四向实测
> （Rust ↔ CANopenLinux C、Rust ↔ python-canopen），结果与抓包证据见
> **`tests/interop/RESULTS-2026-10-23.md`**。测试 harness、C 副本和日志位于
> 仓外 `/tmp/canopen-interop`；下方引用的 in-repo 路径仍未落地，不能仅依赖
> 本计划当作可重复脚本。`host/tests/vcan.rs` 继续作为无硬件的 vcan 预检。

本测试计划将 canopen-rs 与两个独立参考实现进行**四向交叉验证**：

| 套件 | 我方角色 | 对端 | 对端实现 |
|---|---|---|---|
| S1 | **主站** | 从站 | CANopenLinux `canopend`（C） |
| S2 | **主站** | 从站 | python-canopen `LocalNode` |
| S3 | **从站** | 主站 | CANopenLinux `canopend`（TCP 网关） |
| S4 | **从站** | 主站 | python-canopen `Network` |

通过标准：全部用例通过，且任何帧级差异需以 candump 抓包取证后归档。

> **运行前提**：真实 SocketCAN 接口（本文统一 `can0`）。开发期可用 vcan0
> 预检逻辑正确性，但**电气层与时序问题只有真机才能暴露**。

---

## 1. 环境准备

### 1.1 接口配置

```bash
# 双机直连拓扑：两块 CAN 卡经收发器对接，两端各接 120Ω 终端电阻。
sudo ip link set down can0 2>/dev/null
sudo ip link set can0 type can bitrate 250000
sudo ip link set up can0

candump -tz can0 &          # 全程抓包留证
```

单机自测可用 vcan0 替代（无电气层覆盖）：

```bash
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
```

### 1.2 构建产物

```bash
cargo build --workspace                          # canopen / served_node 示例
cd ref/CANopenLinux && make                      # 生成 ./canopend
pip install canopen                              # ≥2.4.1
```

### 1.3 进程启动速查

| 角色 | 命令 |
|---|---|
| canopend 从站（节点 12） | `./canopend -i 12 can0` |
| canopend 主站网关（节点 10） | `./canopend -i 10 -c tcp-60000 can0` |
| python LocalNode 从站（节点 13） | `python3 tools/interop/python_slave_node.py can0` |
| 我方从站（节点 5） | `cargo run -p canopen-host --example served_node -- can0 5` |
| cocomm 客户端（驱动 canopend 主站） | `echo '[1] 12 read 0x1000 0 u32' | nc localhost 60000` |

### 1.4 canopend OD 扩展（S1 分段/block 用例前置）

canopend 示例 OD **只含通信区对象**（0x1000–0x1BFF），无 >4 字节可写对象。
分段/block 用例前需手工扩展 `CANopenNode/example/OD.{h,c}` 增加两个对象：

- `0x2100` VisibleString[64]，rw —— 分段读写目标
- `0x2101` Unsigned64，rw —— block download 目标

修改四处（照抄相邻条目模式）：

1. `OD.h`：`#define OD_CNT_APP_RELAY ...` 区域加计数与
   `#define OD_ENTRY_H2100/2101`
2. `OD.c` 的 `OD_PERSIST` 默认值结构体：加 `.x2100_... = ""`、`.x2101_... = 0`
3. `OD.c` 的对象描述初始化区（`o_1000_deviceType` 模式）：加 VAR 描述，
   subEntriesCount=1、`dataOrig` 指向默认值成员、attribute 含 `ODA_SDO_R | ODA_SDO_W`
4. `OD.c` 的 `OD->list[]`：追加 `{0x2100, 0x01, ODT_VAR, &ODObjs.o_2100, NULL}` 两行

重新 `make` 后用 `cocomm "12 read 0x2100 0 vs"` 验证存在。

---

## 2. 节点号与资源分配表

| 资源 | 分配 | 说明 |
|---|---|---|
| 我方从站节点号 | 5 | S4/S3 被测设备 |
| canopend 从站节点号 | 12 | S1 被测设备 |
| python 从站节点号 | 13 | S2 被测设备 |
| canopend 主站节点号 | 10 | S3 |
| LSS 临时 id 段 | 20–29 | 二期 LSS 用例 |
| SYNC COB-ID | 0x080 | 各套件复用（串行执行） |
| 测试 PDO | TPDO1/RPDO1 默认段 | 见各用例具体 COB-ID |
| 网关 TCP 端口 | 60000 | 仅 S3 |

**约束**：四个套件串行执行；每套件开始前确认总线上无非预期节点
（`candump` 应为空）。

---

## 3. 套件 S1：我方主站 ↔ canopend 从站

实现：`host/tests/xcan_s1_canopenlinux_slave.rs`（Rust 集成测试，子进程拉起
canopend）。

| # | 用例 | 步骤摘要 | 预期 |
|---|---|---|---|
| S1-01 | 上线检测 | 启动 canopend → 订阅 0x70C | 收到 boot-up 帧 `[00]`；随后心跳按 0x1017 周期到达 |
| S1-02 | 设备类型读取 | SDO read 0x1000:0 (u32) | 返回 `0x00000000`（示例 OD 默认值） |
| S1-03 | 加速读写往返 ×3 类型 | write/read 0x1017:0 为 u16 {500, 0xFFFF, 0} | 每次回读值一致；写响应命令字节 `0x60` |
| S1-04 | 分段读写字符串 | write/read 0x2100:0 vs `"Hello CANopen"`（≥8 字节触发分段） | 回读字符串一致；toggle 位交替正确 |
| S1-05 | block download + upload | 向 0x2101 写入 900 字节伪随机序列再回读 | 数据逐字节一致（CRC 校验通过）；期间无 abort |
| S1-06 | Abort 映射 | 读 0x5555:0 | 错误为 `SdoAborted(0x06020000)`；后续事务仍正常 |
| S1-07 | NMT 控制 | start→stop→preop→reset comm | 心跳状态字节依次 0x05→0x04→0x7F；reset 后收到新 boot-up |
| S1-08 | EMCY 触发 | SDO 写非法 TPDO 映射（映射不存在对象）后置 valid 位 | 收到 EMCY 帧 code=0x8210 类，register bit4 置位 |
| S1-09 | 心跳丢失告警 | monitor_heartbeat(node=12) 后 SIGKILL canopend | `HbEvent::Lost` 在超时窗内到达 |

失败取证：保留 candump 日志 + canopend stdout。

---

## 4. 套件 S2：我方主站 ↔ python-canopen 从站

实现：Rust 测试子进程拉起 `python3 tools/interop/python_slave_node.py can0`；
python 脚本以 **`ref/canopen/test/sample.eds`** 为从站 OD（含 0x1008 设备名
字符串、0x2000 可写字符串、RPDO 段），并追加：`0x1021`(domain ro，内容为
同一 EDS 的文本)、`datatypes.eds` 全类型条目挂载于 0x3000 区、写入记录回调。

| # | 用例 | 步骤摘要 | 预期 |
|---|---|---|---|
| S2-01 | 加速读写全类型 | 基于 `datatypes.eds` 类型表驱动：bool/i8..i64/u8..u64/str 各一轮 round-trip（蓝本 `test_sdo.py`） | 全部一致 |
| S2-02 | block download 负向 | 对 0x1008 发起 block download（蓝本 `test_local.py:48-53`）| abort code 精确等于 `0x05040001`；随后普通写仍成功（会话未被破坏） |
| S2-03 | 分段上传/下载可写字符串 | 对 0x2000（sample.eds 内置可写字符串）写 200 字节文本后回读 | 内容逐字节一致 |
| S2-04 | 0x1021 EDS 拉取 | `read_eds_from_device(13)` | 解析成功且对象集合与脚本预置 EDS 一致（打通 A2 特性端到端） |
| S2-05 | 心跳消费 | monitor node=13 → kill python | `Lost` 事件 |

---

## 5. 套件 S3：canopend 主站 ↔ 我方从站

实现：shell 编排——启动我方 `served_node can0 5` 与
`canopend -i 10 -c tcp-60000 can0`；用 python/nc 向 TCP 口发 CiA 309 命令行。
我方从站 OD 由 `served_node` 内置（含 0x2100 字符串、0x2101 u64、0x6000 应用区，
均为 persistent 标记可选）。

| # | 用例 | CiA 309 命令 | 预期 |
|---|---|---|---|
| S3-01 | SDO 读 | `[1] 5 read 0x1000 0 u32` | 返回 served_node 内置设备类型常量 |
| S3-02 | SDO 写+回读 | `[2] 5 write 0x1017 0 u16 500` 后经我方 CLI 复核 | `canopen read can0 5 0x1017` 显示 500 |
| S3-03 | 分段写 vs | `[3] 5 write 0x2100 0 vs <200字符>` | 回读一致 |
| S3-04 | block download | `set sdo_block 1` 后向 0x2101 写 900B | 我方 server CRC 校验接收；`read` 回读一致 |
| S3-05 | NMT 序列 | `start node 5` / `stop node 5` / `reset node 5` | 我方心跳字节 05→04→boot-up 重发 |
| S3-06 | Abort 透传 | `read 0x5555 0` | `ERROR:0x06020000` 格式响应（大写、8 位补零，docs/DESIGN-DECISIONS.md G8） |
| S3-07 | 心跳消费（主站侧） | 先 `[n] 10 write 0x1016 1 u32 0x000C03E8`（监视 12 号…改为监视 5 号：`0x000503E8`=node5/1000ms）→ kill served_node | canopend 报 HB consumer error（EMCY 或日志） |
| S3-08 | LSS 交叉验证 | `lss_switch_glob`/`lss_set_node`/fastscan 族 | **二期项**：依赖 cocomm LSS 行为调研（见 §9） |

---

## 6. 套件 S4：python-canopen 主站 ↔ 我方从站

实现：`tools/interop/s4_python_master.py`（pytest 风格脚本，内部拉起
served_node 子进程，其 OD 按 **`sample.eds`** 对象布局内置）。

| # | 用例 | 步骤摘要 | 预期 |
|---|---|---|---|
| S4-01 | RemoteNode 读写全类型 | 按 sample.eds 布局：0x1017 u16 / 0x1008 str / 0x2000 字符串 round-trip（蓝本同 S2-01） | 一致 |
| S4-02 | reset + wait_bootup | `nmt.reset_communication()` → `wait_bootup(5, 2s)` | 成功返回；此后 pre-op 心跳可见 |
| S4-03 | 心跳状态序列 | 订阅 0x705 记录状态字节序列 | start/stop 命令后出现 0x05/0x04 |
| S4-04 | python 配置 TPDO 并收帧 | 经 SDO 配置我方 TPDO2 映射 → 发 SYNC → 监听 0x285 | 帧载荷与我方 OD 值一致 |
| S4-05 | python 发 RPDO | 向我方 RPDO1 发帧 | SDO 回读映射对象已更新 |
| S4-06 | EMCY 消费 | 我方 `raise_emergency(0x2310,…)` | python EmcyConsumer 回调收到 code/register |
| S4-07 | block download 对拍 | python `sdo[0x2101].open('wb')` 写 900B | 我方 server 接收成功且回读一致（独立实现验证我方 CRC/ACK 语义） |

---

## 7. 验收标准与取证

- 每套件全部用例通过即该套件 PASS；任一 FAIL 需附：
  - 完整 `candump -tz` 抓包片段（覆盖失败事务前后各 ≥2 秒）
  - 对端进程日志（canopend stdout / python stderr）
  - 我方 `RUST_LOG=debug` 输出
- 已知限制（不判 FAIL）：
  - python SDO server 不支持 block download（S2-02 即为此设计的负向验证）
  - canopend 示例 OD 无应用对象（S1-04/05 前置 §1.4 扩展）
  - LSS 交叉验证为二期项（§9）

## 8. 执行顺序建议

S1 → S2 → S3 → S4 串行；每套件结束后 `pkill -f canopend; pkill -f served_node;
pkill -f python_slave_node` 并核对 candump 安静。

## 9. 二期项（本期不做）

- **LSS 交叉验证**（S3-08）：需先调研 canopend/cocomm 的 LSS 命令集行为
  （`lss_switch_glob`、`lss_set_node`、fastscan 参数格式）并确认其对我方
  未配置实例（`served_node --unconfigured`）的发现流程。
- **SRDO 交叉**：双方支持度均有限，暂缓。
- **CiA 402 交叉**：canopend 无 402 模块；如需对拍须引入 CANopenDemo 或
  商用从站模拟器。

## 10. 故障排查速查

| 症状 | 排查 |
|---|---|
| `ENODEV` / 接口打不开 | `ip link` 确认接口 UP；bitrate 是否匹配 |
| 总线全静默 | 终端电阻（120Ω×2）；TX/RX 是否接反 |
| SDO 无响应 | 节点号是否冲突；对端 NMT 是否 Initialising（未 boot）|
| 偶发 CRC/seq 错误 | 线缆过长/干扰；降低波特率复测 |
| canopend 网关无响应 | `-T` 超时参数；nc 是否发送了 `\r\n` 结尾 |


## 11. 可复用参考资产（来自 ref/ 自带测试）

调研结论：**python-canopen 的 pytest 套件可直接复用**——不是原样运行
（virtual bus 同进程自回环只能验证 python vs python），而是复用其**用例蓝本、
断言期望值与 EDS 夹具**，把总线接口从 `interface="virtual"` 改为
`"socketcan"`（python-can 官方参数），一侧替换为本栈实现。

| 资产 | 路径 | 用途 |
|---|---|---|
| sample.eds | `ref/canopen/test/sample.eds` | S2/S4 从站 OD 与我方 served_node 内置布局（0x1008 字符串 / 0x2000 可写字符串 / RPDO 段） |
| datatypes.eds | `ref/canopen/test/datatypes.eds` | S2-01 全数据类型表驱动 round-trip 的 OD 来源 |
| test_local.py | `ref/canopen/test/test_local.py`（19 条） | S2/S4 断言结构蓝本；block 负向期望值出处（:48-53 → abort `0x05040001`） |
| test_sdo.py | `ref/canopen/test/test_sdo.py`（45 条） | SDO 加速/分段用例蓝本（跨栈适用子集） |
| zencan `integration_tests/src/lss_test.rs` | `ref/zencan/integration_tests/` | LSS 从站侧交叉测试（python 无 LSS slave）的双真实节点骨架范本——二期 S3-08 引用 |

不可直接运行的原因备忘：python-can 基础 virtual bus 仅进程内按 channel 广播；
原套件两侧都是 python-canopen，跑通≠本栈正确。CANopenLinux 无测试套件；
oze-canopen tests 缺对端设备实际不可运行；canopen-rust vendored 副本无 tests/。

> **pyport 并行线**：上述蓝本用例正被系统性转换为进程内 Rust 回归测试
> （VirtualBus 直驱，无需总线），映射表与差异对照见
> **`docs/EXTENSIONS.md` §批次 D**。真机交叉验证（本文档）与 pyport 回归
> 互补：前者验互操作与时序，后者验语义回归。
