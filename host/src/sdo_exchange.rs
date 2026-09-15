//! Transport-agnostic SDO transaction core shared by the blocking and tokio
//! transports (`BUGS.md` m12): one implementation of client lifecycle,
//! outgoing queue management, protocol-error ABORT transmission and
//! terminal-event interpretation. The two transports only supply
//! send/receive primitives.

use std::collections::VecDeque;

use canopen_core::datatypes::{DataType, Value};
use canopen_core::sdo::{request_cob_id, response_cob_id, SdoClient, SdoEvent};
use canopen_core::types::{Address, NodeId};

use crate::error::HostError;
use crate::transport::sdo_error;

/// How a driven transaction ended.
#[derive(Debug)]
pub enum Progress {
    /// More responses expected; keep popping requests and waiting.
    Continue,
    /// Read finished with a decoded value.
    Complete(Value),
    /// Write finished (server confirmed).
    Done,
}

/// One SDO transaction against a node's default channel.
#[derive(Debug)]
pub struct Exchange {
    client: SdoClient,
    queue: VecDeque<[u8; 8]>,
    /// Frames that must go out before the exchange may end: protocol-error
    /// ABORTs queued by the client and unconfirmed block-end trailing
    /// frames. Buffered so they survive even an error return (`BUGS.md`
    /// P1-1).
    pending_extra: Vec<[u8; 8]>,
    rx_cob: u16,
    tx_cob: u16,
    /// How to rebuild this transaction after a timeout retry (`BUGS.md`
    /// P1-2).
    op: Op,
}

/// The transaction kind, kept so [`Exchange::restart`] can replay it.
#[derive(Debug)]
enum Op {
    Read { addr: Address, dt: DataType },
    Write { addr: Address, data: Vec<u8> },
    ReadBlock { addr: Address, dt: DataType },
}

impl Exchange {
    fn new(client: SdoClient, first: Vec<[u8; 8]>, node: u8, op: Op) -> Self {
        Self {
            client,
            queue: first.into(),
            pending_extra: Vec::new(),
            rx_cob: request_cob_id(node),
            tx_cob: response_cob_id(node),
            op,
        }
    }

    fn begin(op: &Op) -> Result<(SdoClient, Vec<[u8; 8]>), HostError> {
        let mut client = SdoClient::new();
        let first = match op {
            Op::Read { addr, dt } => client.read(*addr, *dt),
            Op::ReadBlock { addr, dt } => client.read_block(*addr, *dt),
            // BUGS.md L5: re-framing the same payload cannot fail in
            // practice (it was already framed in `Exchange::write`), but a
            // silent `unwrap_or_default()` would mask a genuine framing bug
            // with an empty queue that just re-times-out. Fail fast instead.
            Op::Write { addr, data } => client.write(*addr, data).map_err(sdo_error)?,
        };
        Ok((client, first))
    }

    /// Starts an expedited/segmented read.
    ///
    /// # Errors
    /// [`HostError::BadSdoNode`] when `node` is outside the valid 1..=127
    /// range (a raw `node` was previously folded into the request COB-ID,
    /// sending frames to a reserved/illegal channel; `send_nmt_body` already
    /// validated via `NodeId::try_from`, `BUGS.md` H2).
    pub fn read(node: u8, addr: Address, dt: DataType) -> Result<Self, HostError> {
        let node = NodeId::try_from(node)
            .map_err(|_| HostError::BadSdoNode)?
            .get();
        let mut client = SdoClient::new();
        let first = client.read(addr, dt);
        Ok(Self::new(client, first, node, Op::Read { addr, dt }))
    }

    /// Starts an expedited/segmented write.
    ///
    /// # Errors
    /// [`HostError::Core`] when `data` cannot be framed (oversized etc.);
    /// [`HostError::BadSdoNode`] when `node` is outside 1..=127.
    pub fn write(node: u8, addr: Address, data: &[u8]) -> Result<Self, HostError> {
        let node = NodeId::try_from(node)
            .map_err(|_| HostError::BadSdoNode)?
            .get();
        let mut client = SdoClient::new();
        let first = client.write(addr, data).map_err(sdo_error)?;
        Ok(Self::new(
            client,
            first,
            node,
            Op::Write {
                addr,
                data: data.to_vec(),
            },
        ))
    }

    /// Starts a block upload (used for large domain objects like 0x1021).
    ///
    /// # Errors
    /// [`HostError::BadSdoNode`] when `node` is outside the valid 1..=127
    /// range.
    pub fn read_block(node: u8, addr: Address, dt: DataType) -> Result<Self, HostError> {
        let node = NodeId::try_from(node)
            .map_err(|_| HostError::BadSdoNode)?
            .get();
        let mut client = SdoClient::new();
        let first = client.read_block(addr, dt);
        Ok(Self::new(client, first, node, Op::ReadBlock { addr, dt }))
    }

    /// Resets the transaction to its initial state so the driver can retry
    /// after a timeout (`BUGS.md` P1-2).
    ///
    /// # Errors
    /// [`HostError::Core`] when re-framing fails (`BUGS.md` L5: previously
    /// the failure was masked by an empty queue that just re-timed-out).
    pub fn restart(&mut self) -> Result<(), HostError> {
        let (client, first) = Self::begin(&self.op)?;
        self.client = client;
        self.queue = first.into();
        self.pending_extra.clear();
        Ok(())
    }

    /// Queues an explicit ABORT to release the peer and returns the frame to
    /// transmit, if any.
    pub fn abort_with(&mut self, code: canopen_core::sdo::SdoAbortCode) -> Option<[u8; 8]> {
        self.client.abort_now(code)
    }

    /// Next frame for the request channel, if any.
    pub fn pop_request(&mut self) -> Option<[u8; 8]> {
        self.queue.pop_front()
    }

    /// COB-ID carrying requests to the server.
    #[must_use]
    pub const fn request_cob(&self) -> u16 {
        self.rx_cob
    }

    /// COB-ID carrying responses from the server.
    #[must_use]
    pub const fn response_cob(&self) -> u16 {
        self.tx_cob
    }

    /// Feeds one response frame from the server. Frames that must go out
    /// (a protocol-error ABORT queued by the client so the server is
    /// released, and unconfirmed block-end trailing frames) are buffered in
    /// [`Exchange::pending_extra`] — including those triggered by *this*
    /// response on the error path — and retrieved via [`Exchange::drain_extra`].
    ///
    /// # Errors
    /// - [`HostError::SdoAborted`] when the server aborted (code preserved,
    ///   BUGS.md M1);
    /// - I/O and protocol errors propagate.
    pub fn on_response(&mut self, resp: &[u8; 8]) -> Result<Progress, HostError> {
        // Run the state machine first, then collect aborts: an ABORT caused
        // by *this* very response must reach the wire before we give up
        // (BUGS.md P1-1/M10).
        let result = self.client.on_response(resp).map_err(sdo_error);
        self.pending_extra.extend(self.client.take_pending_abort());
        Ok(match result? {
            SdoEvent::Send(frames) => {
                self.queue.extend(frames);
                Progress::Continue
            }
            SdoEvent::Complete(value, trailing) => {
                // Trailing frames (block end confirmation) are unconfirmed.
                self.pending_extra.extend(trailing);
                Progress::Complete(value)
            }
            SdoEvent::Done => Progress::Done,
        })
    }

    /// Takes all buffered outbound frames (ABORT / block-end trailers).
    pub fn drain_extra(&mut self) -> Vec<[u8; 8]> {
        std::mem::take(&mut self.pending_extra)
    }
}

/// Reinterprets a decoded upload as raw bytes (domain reads).
#[must_use]
pub fn value_into_bytes(value: Value) -> Vec<u8> {
    match value {
        Value::Bytes(b) => b,
        Value::Str(s) => s.into_bytes(),
        other => {
            let n = other.encoded_len();
            let mut buf = vec![0u8; n];
            let written = other.encode_le(&mut buf).unwrap_or(0);
            buf.truncate(written);
            buf
        }
    }
}

// --- m12 修复：阻塞/异步传输层共享的算法体（单一来源） ---
//
// 两个 transport 仅 `send`/`recv` 的调用方式不同（阻塞 vs `.await`）。
// 下面的宏把 SDO 事务驱动、节点服务循环、NMT 发送等算法写成一份，
// 通过 `suspend!` 把 `await`/`block` 两种调度统一，消除平行实现漂移。
// 调用处各自只写方法签名（是否 `async`）并展开对应宏体。

/// 把 `await`/`block` 两种挂起方式统一为一个 token。
macro_rules! suspend {
    (await $e:expr) => {
        $e.await
    };
    (block $e:expr) => {
        $e
    };
}

/// 驱动一个 SDO 事务（`Exchange`）到完成（阻塞/异步共用算法体）。
///
/// 调度规则（`BUGS.md` P0-3/P1-1/P1-2）：
/// 1. 每轮**排空整个发送队列**（block download 一次排队 ≤127 个 segment，
///    服务端收满子块才应答，不能"发一帧等一帧"）；
/// 2. 然后等**一帧**响应——block upload 的中间段不产生新请求帧，
///    因此队列已空也要继续等待，循环只以终态或错误为出口；
/// 3. 协议违规上抛前先冲出客户端排队的 ABORT（P1-1）；
/// 4. 超时先 ABORT 对端、原样重试一次再放弃（§5.6，P1-2）。
macro_rules! drive_exchange_body {
    ($self:ident, $ex:ident, $deadline:expr, $mode:ident) => {{
        use $crate::error::HostError;
        use $crate::sdo_exchange::Progress;
        // BUGS.md 八M8: the deadline is a *per-response* window, not a budget
        // for the whole transaction. python-canopen (`sdo/client.py:23,77`)
        // and CANopenNode (`CO_SDOclient.c`, timer cleared on every frame)
        // both enforce a per-response timeout with no total-transaction
        // bound; a single absolute deadline made a 64 KiB block download
        // (8192 frames at 1 Mbit/s ≈ 0.9 s) time out mid-transfer even
        // though every frame arrived in time. Capture the caller's window
        // (tests pass short windows to simulate timeouts) and re-arm it for
        // every response wait below.
        let window = $deadline.saturating_duration_since(std::time::Instant::now());
        let mut retried = false;
        // BUGS.md H1: 新事务起点（发出首帧前）先排空本节点 cmd 套接字上的
        // 陈旧响应，与 python-canopen `request_response` 在每次调用开头清空
        // 队列一致（§5.6 旧注释只覆盖了超时重试路径）。否则跨对象复用同一个
        // node 的 cmd 套接字、或 `read_domain_bytes` 的 block→分段两次独立
        // 事务回退时，上一事务残留/迟到的响应会被当作新事务首帧喂入
        // `SdoClient::on_response`，造成状态错乱（UnexpectedFrame 等）。
        // 2ms 有界窗口——窗口到期后迟到响应与 python-canopen 一样按竞争处理。
        let drain_until = std::time::Instant::now() + std::time::Duration::from_millis(2);
        // BUGS.md 八Q10: only a *timeout* ends the drain window — a real IO
        // error (interface down etc.) must surface instead of being silently
        // discarded with `.is_ok()`.
        loop {
            match $crate::sdo_exchange::suspend!($mode
                $self.recv_response($ex.response_cob(), drain_until)
            ) {
                Ok(_stale) => continue,
                Err(HostError::Timeout) => break,
                Err(e) => return Err(e),
            }
        }
        loop {
            while let Some(frame) = $ex.pop_request() {
                $crate::sdo_exchange::suspend!($mode $self.send($ex.request_cob(), &frame))?;
            }
            match $crate::sdo_exchange::suspend!($mode $self.recv_response(
                $ex.response_cob(),
                std::time::Instant::now() + window
            )) {
                Ok(response) => {
                    let progress = match $ex.on_response(&response) {
                        Ok(progress) => progress,
                        Err(e) => {
                            for f in $ex.drain_extra() {
                                $crate::sdo_exchange::suspend!($mode $self.send($ex.request_cob(), &f))?;
                            }
                            return Err(e);
                        }
                    };
                    for f in $ex.drain_extra() {
                        $crate::sdo_exchange::suspend!($mode $self.send($ex.request_cob(), &f))?;
                    }
                    if !matches!(progress, Progress::Continue) {
                        return Ok(progress);
                    }
                }
                Err(HostError::Timeout) if !retried => {
                    retried = true;
                    if let Some(abort) =
                        $ex.abort_with(canopen_core::sdo::SdoAbortCode::ProtocolTimeout)
                    {
                        $crate::sdo_exchange::suspend!($mode $self.send($ex.request_cob(), &abort))?;
                    }
                    // BUGS.md L5: re-framing cannot fail here (the payload
                    // was framed successfully in `Exchange::write`), but a
                    // failure must surface rather than silently retry with
                    // an empty queue.
                    $ex.restart()?;
                    // §5.6 stale-response hygiene (BUGS.md P2-10): the peer
                    // may still be streaming replies to the abandoned
                    // transaction, so drain everything queued in a short
                    // 2 ms bounded window before restarting — a late response
                    // must not be fed into the new attempt as
                    // `UnexpectedFrame`. The window is bounded by design: a
                    // response arriving after it races the retry exactly as
                    // in python-canopen (it clears the queue only at the
                    // send point), so this is a documented known limitation
                    // (BUGS.md #12), not a silent corruption path.
                    let drain_until =
                        std::time::Instant::now() + std::time::Duration::from_millis(2);
                    loop {
                        match $crate::sdo_exchange::suspend!($mode
                            $self.recv_response($ex.response_cob(), drain_until)
                        ) {
                            Ok(_stale) => continue,
                            Err(_) => break,
                        }
                    }
                    // No deadline reset needed: the per-response window is
                    // re-armed at every `recv_response` call (BUGS.md 八M8),
                    // so the retry automatically gets a full window (the
                    // pre-八M8 code reset the single absolute deadline here).
                }
                // BUGS.md #11: the retry also timed out. Release the peer's
                // pending transaction with an explicit abort before giving
                // up, exactly like python-canopen (`request_response`) and
                // CANopenNode do when they abandon a transfer — otherwise
                // the peer is left wedged until its own timeout.
                Err(HostError::Timeout) if retried => {
                    if let Some(abort) =
                        $ex.abort_with(canopen_core::sdo::SdoAbortCode::ProtocolTimeout)
                    {
                        $crate::sdo_exchange::suspend!($mode $self.send($ex.request_cob(), &abort))?;
                    }
                    return Err(HostError::Timeout);
                }
                Err(e) => return Err(e),
            }
        }
    }};
}

/// 服务一个 canopen-core `Node` 直到 `shutdown` 置位（阻塞/异步共用算法体）。
macro_rules! serve_node_body {
    ($self:ident, $node:ident, $shutdown:ident, $mode:ident) => {{
        use std::sync::atomic::Ordering;
        if let Some(boot) = $node.boot() {
            $crate::sdo_exchange::suspend!($mode $self.send(boot.cob_id, boot.payload()))?;
        }
        // Session timeout (§5.5 / BUGS.md P2-8): default 1500 ms unless the
        // application configured something explicitly.
        $node.set_sdo_timeout_if_unset(1500);
        while !$shutdown.load(Ordering::Relaxed) {
            for tx in $node.poll($crate::transport::now_ms()) {
                $crate::sdo_exchange::suspend!($mode $self.send(tx.cob_id, tx.payload()))?;
            }
            let incoming = match $crate::sdo_exchange::suspend!($mode $self.recv_frame()) {
                Ok(Some(frame)) => frame,
                Ok(None) => {
                    if let Some(tx) = $node.sdo_timeout_due($crate::transport::now_ms()) {
                        $crate::sdo_exchange::suspend!($mode $self.send(tx.cob_id, tx.payload()))?;
                    }
                    continue;
                }
                Err(e) => return Err(e),
            };
            // Only frames on the SDO request channel (0x600+node) count as
            // activity for the SDO session timer — unrelated heartbeats/PDOs/
            // SYNC must not keep a wedged transfer alive on a busy bus
            // (BUGS.md #10, CANopenNode resets the timeout only on SDO
            // frames). `touch_sdo_activity` records activity whenever a
            // transfer is in flight.
            if let $crate::transport::Incoming::Data(rx) = &incoming {
                // BUGS.md H2: use the node's *own* SDO RX COB-ID (single
                // source of truth) instead of re-deriving `0x600+node` here —
                // core currently hardcodes the default channel (0x1200
                // multi-channel is not implemented, docs/IMPLEMENTATION.md
                // §5.5), so this is behaviour-identical today but stays
                // correct if a custom SDO RX COB ever lands in core.
                let sdo_cob = $node.node_id().sdo_rx_cob_id();
                if rx.cob_id == sdo_cob {
                    let now = $crate::transport::now_ms();
                    $node.touch_sdo_activity(now);
                }
            }
            let responses = match incoming {
                $crate::transport::Incoming::Data(rx) => $node.on_frame(rx.cob_id, rx.payload()),
                $crate::transport::Incoming::Remote(cob) => {
                    $node.on_remote_request(cob).into_iter().collect()
                }
            };
            if let Some(tx) = $node.sdo_timeout_due($crate::transport::now_ms()) {
                $crate::sdo_exchange::suspend!($mode $self.send(tx.cob_id, tx.payload()))?;
            }
            for tx in responses {
                $crate::sdo_exchange::suspend!($mode $self.send(tx.cob_id, tx.payload()))?;
            }
            // NMT reset re-initialisation (BUGS.md P2-2): ResetNode /
            // ResetCommunication put the node back into Initialising; CiA 301
            // requires it to re-enter PreOperational and emit boot-up on its
            // own. `boot()` is a no-op in every other state.
            if $node.nmt_state() == canopen_core::nmt::NmtState::Initialising {
                if let Some(boot) = $node.boot() {
                    $crate::sdo_exchange::suspend!($mode $self.send(boot.cob_id, boot.payload()))?;
                }
            }
        }
        Ok(())
    }};
}

/// 广播/定向 NMT 命令（阻塞/异步共用算法体）。
macro_rules! send_nmt_body {
    ($self:ident, $command:ident, $target:ident, $mode:ident) => {{
        use canopen_core::types::NodeId;
        let target = match $target {
            None | Some(0) => None,
            Some(raw) => Some(NodeId::try_from(raw).map_err(|_| {
                $crate::error::HostError::BadNmtTarget
            })?),
        };
        $crate::sdo_exchange::suspend!($mode $self.send(
            canopen_core::nmt::NMT_COMMAND_COB_ID,
            &canopen_core::nmt::encode_command($command, target),
        ))?;
        Ok(())
    }};
}

/// 读取对象并解码（阻塞/异步共用算法体）。
macro_rules! sdo_read_body {
    ($self:ident, $node:ident, $addr:ident, $dt:ident, $mode:ident) => {{
        let ex = $crate::sdo_exchange::Exchange::read($node, $addr, $dt)?;
        match $crate::sdo_exchange::suspend!($mode $self.drive_exchange(
            ex,
            std::time::Instant::now() + $crate::transport::SDO_DEADLINE,
        ))? {
            $crate::sdo_exchange::Progress::Complete(v) => Ok(v),
            _ => Err($crate::error::HostError::ProtocolIncomplete),
        }
    }};
}

/// 写入对象（阻塞/异步共用算法体）。
macro_rules! sdo_write_body {
    ($self:ident, $node:ident, $addr:ident, $data:ident, $mode:ident) => {{
        let ex = $crate::sdo_exchange::Exchange::write($node, $addr, $data)?;
        match $crate::sdo_exchange::suspend!($mode $self.drive_exchange(
            ex,
            std::time::Instant::now() + $crate::transport::SDO_DEADLINE,
        ))? {
            $crate::sdo_exchange::Progress::Done => Ok(()),
            _ => Err($crate::error::HostError::ProtocolIncomplete),
        }
    }};
}

/// 读取大对象为字节（先 block 再回退分段；阻塞/异步共用算法体）。
///
/// block 尝试失败的**回退策略**（`BUGS.md` H2）：
/// - 服务端明确 ABORT（如不支持 block）→ 正常回退分段读取；
/// - **超时** → 不再静默吞掉走分段（此前最坏延迟 2× `SDO_DEADLINE`，
///   且掩盖了「block 被拒」与「block 超时」的区分）——block 超时是
///   事务级失败，直接透传，与 python-canopen 一致；
/// - 其余错误（IO、协议错误等）→ 透传，不再掩盖诊断。
macro_rules! read_domain_bytes_body {
    ($self:ident, $node:ident, $addr:ident, $mode:ident) => {{
        use canopen_core::datatypes::DataType;
        let block_ex = $crate::sdo_exchange::Exchange::read_block($node, $addr, DataType::Domain)?;
        match $crate::sdo_exchange::suspend!($mode $self.drive_exchange(
            block_ex,
            std::time::Instant::now() + $crate::transport::SDO_DEADLINE,
        )) {
            Ok($crate::sdo_exchange::Progress::Complete(v)) => {
                return Ok($crate::sdo_exchange::value_into_bytes(v));
            }
            // 服务端明确拒绝 block：回退分段（正常路径，python-canopen
            // 服务端对不支持的 block 也是 abort）。
            Err($crate::error::HostError::SdoAborted(_)) => {}
            // block 超时/IO/协议错误：透传，不再掩盖（BUGS.md H2）。
            Err(e) => return Err(e),
            // 非 Complete 终态（理论上只可能是 Done）不属于读取语义。
            Ok(_) => return Err($crate::error::HostError::ProtocolIncomplete),
        }
        let ex = $crate::sdo_exchange::Exchange::read($node, $addr, DataType::Domain)?;
        match $crate::sdo_exchange::suspend!($mode $self.drive_exchange(
            ex,
            std::time::Instant::now() + $crate::transport::SDO_DEADLINE,
        ))? {
            $crate::sdo_exchange::Progress::Complete(v) => {
                Ok($crate::sdo_exchange::value_into_bytes(v))
            }
            _ => Err($crate::error::HostError::ProtocolIncomplete),
        }
    }};
}

// Crate-scoped re-exports (BUGS.md P3-60): `#[macro_export]` leaked these
// into the public API where their `pub(crate)` dependencies made them
// unusable from outside anyway.
pub(crate) use drive_exchange_body;
pub(crate) use read_domain_bytes_body;
pub(crate) use sdo_read_body;
pub(crate) use sdo_write_body;
pub(crate) use send_nmt_body;
pub(crate) use serve_node_body;
pub(crate) use suspend;

#[cfg(test)]
mod tests {
    //! 内存总线回归（`BUGS.md` P0-3/P1-1/P1-2）：host 宏 ↔ core `SdoServer`
    //! 全直连，无 SocketCAN。覆盖 block 双向 >127 字节、block 被拒后的分段
    //! 回退、超时 ABORT+单次重试、协议违规补发 ABORT。

    use std::collections::VecDeque;
    use std::time::{Duration, Instant};

    use canopen_core::datatypes::{DataType, Value};
    use canopen_core::object_dictionary::{AccessType, Entry, ObjectDictionary, Storage};
    use canopen_core::sdo::{request_cob_id, response_cob_id, SdoAbortCode, SdoServer};
    use canopen_core::standard::StandardObjects;
    use canopen_core::types::Address;

    use crate::error::HostError;
    use crate::sdo_exchange::{Exchange, Progress};

    const NODE: u8 = 2;
    const BLOB_ADDR: Address = Address::new(0x2500, 0);

    /// 900 字节：> `BLOCK_THRESHOLD`(889) 触发 block download，
    /// 且 block upload 需要两个子块（889+11）。
    fn blob() -> Vec<u8> {
        (0..900u32).map(|i| (i * 7 % 251) as u8).collect()
    }

    fn od_with_blob(value: Option<Vec<u8>>) -> ObjectDictionary {
        let mut od = StandardObjects::new()
            .device_type(0)
            .vendor_id(0x123)
            .build();
        let v = value.unwrap_or_default();
        od.insert(
            BLOB_ADDR,
            Entry::new(
                "Blob",
                DataType::Domain,
                AccessType::Rw,
                Storage::Value(Value::Bytes(v)),
            ),
        );
        // 12 字节的 UNSIGNED 区对象，供分段上传的 toggle 注入测试使用。
        let mut seg = Value::Bytes(vec![0xAA; 12]);
        if let Value::Bytes(b) = &mut seg {
            for (i, byte) in b.iter_mut().enumerate() {
                *byte = byte.wrapping_add(i as u8);
            }
        }
        od.insert(
            Address::new(0x2501, 0),
            Entry::new("Seg", DataType::Domain, AccessType::Rw, Storage::Value(seg)),
        );
        od
    }

    /// 故障注入开关。
    #[derive(Default)]
    struct Faults {
        /// 拒绝一切 block 命令（模拟不支持 block 的从站）。
        reject_block: bool,
        /// 吞掉接下来 N 个请求帧（模拟丢帧/沉默从站）。
        swallow_requests: u32,
        /// 翻转服务端上传 data segment 的 toggle 位（制造 ToggleBit 违规）。
        flip_segment_toggle: bool,
    }

    /// host 宏的对端：真 core SdoServer + 可编程故障注入。
    struct ScriptedBus {
        od: ObjectDictionary,
        server: SdoServer,
        faults: Faults,
        inbox: VecDeque<[u8; 8]>,
        /// host 发出的 ABORT 计数。
        aborts_sent: usize,
        /// 每次 `recv_response` 观察到的 deadline（用于验证逐响应重锚，
        /// BUGS.md 八M8）。
        observed_deadlines: Vec<Instant>,
    }

    impl ScriptedBus {
        fn new(faults: Faults) -> Self {
            Self {
                od: od_with_blob(None),
                server: SdoServer::new(),
                faults,
                inbox: VecDeque::new(),
                aborts_sent: 0,
                observed_deadlines: Vec::new(),
            }
        }

        fn with_blob(mut self, bytes: Vec<u8>) -> Self {
            self.od = od_with_blob(Some(bytes));
            self
        }

        // --- 与 transport 相同的方法面，供宏展开调用 -----------------------

        fn send(&mut self, cob_id: u16, data: &[u8]) -> Result<(), HostError> {
            assert_eq!(data.len(), 8, "SDO frames are always 8 bytes");
            if data[0] == 0x80 {
                self.aborts_sent += 1;
            }
            if cob_id != request_cob_id(NODE) {
                return Ok(()); // 与本测试无关的总线流量
            }
            if self.faults.swallow_requests > 0 {
                self.faults.swallow_requests -= 1;
                return Ok(());
            }
            let req: [u8; 8] = data.try_into().expect("checked length");
            if self.faults.reject_block && (req[0] & 0xE0) >= 0xA0 && (req[0] & 0xE0) != 0x80 {
                // 以 UnsupportedAccess 拒绝 block initiate/start；
                // 分段命令（cs<5）不受影响，保证回退路径可用。
                let addr = Address::new(u16::from_le_bytes([req[1], req[2]]), req[3]);
                self.inbox.push_back(canopen_core::sdo::encode_abort(
                    addr,
                    SdoAbortCode::UnsupportedAccess,
                ));
                return Ok(());
            }
            for mut resp in self.server.handle(&mut self.od, &req) {
                if self.faults.flip_segment_toggle && resp[0] & 0xE0 == 0x00 {
                    resp[0] ^= 0x10; // TOGGLE
                }
                self.inbox.push_back(resp);
            }
            Ok(())
        }

        fn recv_response(&mut self, _tx_cob: u16, deadline: Instant) -> Result<[u8; 8], HostError> {
            self.observed_deadlines.push(deadline);
            self.inbox.pop_front().ok_or(HostError::Timeout)
        }

        // --- 宏展开的宿主方法 -------------------------------------------

        fn drive_exchange(
            &mut self,
            mut ex: Exchange,
            deadline: Instant,
        ) -> Result<Progress, HostError> {
            crate::sdo_exchange::drive_exchange_body!(self, ex, deadline, block)
        }

        fn sdo_write(&mut self, addr: Address, data: &[u8]) -> Result<(), HostError> {
            crate::sdo_exchange::sdo_write_body!(self, NODE, addr, data, block)
        }

        fn domain_read(&mut self) -> Result<Vec<u8>, HostError> {
            crate::sdo_exchange::read_domain_bytes_body!(self, NODE, BLOB_ADDR, block)
        }

        fn sdo_read(&mut self, addr: Address, dt: DataType) -> Result<Value, HostError> {
            crate::sdo_exchange::sdo_read_body!(self, NODE, addr, dt, block)
        }
    }

    fn short_deadline(ms: u64) -> Instant {
        Instant::now() + Duration::from_millis(ms)
    }

    #[test]
    fn block_download_900_bytes_roundtrip() {
        let payload = blob();
        let mut bus = ScriptedBus::new(Faults::default());
        bus.sdo_write(BLOB_ADDR, &payload).expect("block write");
        assert_eq!(
            bus.od.read(BLOB_ADDR).unwrap(),
            Value::Bytes(payload),
            "server must have received the entire burst"
        );
    }

    #[test]
    fn deadline_rearms_per_response_not_per_transaction() {
        // BUGS.md 八M8: 多帧 block 事务的 deadline 必须逐响应重锚——旧实现
        // 全程共用同一绝对时刻，长传输（如 64 KiB EDS）必然中途超时。
        let payload = blob();
        let mut bus = ScriptedBus::new(Faults::default());
        let ex = Exchange::write(NODE, BLOB_ADDR, &payload).expect("exchange");
        let progress = bus
            .drive_exchange(ex, short_deadline(50))
            .expect("block write");
        assert!(matches!(progress, Progress::Done));
        let seen = &bus.observed_deadlines;
        assert!(seen.len() > 2, "expected a multi-frame transaction");
        let min = seen.iter().min().unwrap();
        let max = seen.iter().max().unwrap();
        assert!(
            max > min,
            "deadline must advance per response (八M8): {min:?}..{max:?}"
        );
    }

    #[test]
    fn block_upload_large_domain_read() {
        let payload = blob();
        let mut bus = ScriptedBus::new(Faults::default()).with_blob(payload.clone());
        assert_eq!(bus.domain_read().unwrap(), payload);
    }

    #[test]
    fn domain_read_falls_back_when_server_rejects_block() {
        let payload = blob();
        let mut bus = ScriptedBus::new(Faults {
            reject_block: true,
            ..Faults::default()
        })
        .with_blob(payload.clone());
        assert_eq!(bus.domain_read().unwrap(), payload);
    }

    #[test]
    fn domain_read_propagates_block_timeout_instead_of_segmented_fallback() {
        // BUGS.md H2: a block attempt that *times out* must surface the
        // timeout — the previous `if let Ok(Complete) … else segmented`
        // swallowed it and silently ran a second full transaction (up to 2×
        // SDO_DEADLINE latency, and no way to tell "block rejected" from
        // "block timed out"). Swallow every request frame so both the block
        // attempt and its retry time out.
        let payload = blob();
        let mut bus = ScriptedBus::new(Faults {
            swallow_requests: u32::MAX,
            ..Faults::default()
        })
        .with_blob(payload);
        let err = bus
            .domain_read()
            .expect_err("silent peer must surface a timeout, not fall back");
        assert!(matches!(err, HostError::Timeout));
    }

    #[test]
    fn exchange_constructors_reject_out_of_range_node() {
        // BUGS.md H2: raw `node` used to be folded straight into
        // `request_cob_id` — node > 127 sent SDO frames to a reserved COB.
        // All three constructors must reject 0 and 128.. like send_nmt_body.
        let addr = Address::new(0x1000, 0);
        for bad in [0u8, 128, 255] {
            assert!(matches!(
                Exchange::read(bad, addr, DataType::Unsigned32),
                Err(HostError::BadSdoNode)
            ));
            assert!(matches!(
                Exchange::read_block(bad, addr, DataType::Domain),
                Err(HostError::BadSdoNode)
            ));
            assert!(matches!(
                Exchange::write(bad, addr, &[0]),
                Err(HostError::BadSdoNode)
            ));
        }
        // Valid ids still construct.
        assert!(Exchange::read(1, addr, DataType::Unsigned32).is_ok());
        assert!(Exchange::read(127, addr, DataType::Unsigned32).is_ok());
    }

    #[test]
    fn timeout_aborts_peer_then_retry_succeeds() {
        let mut bus = ScriptedBus::new(Faults {
            swallow_requests: 1,
            ..Faults::default()
        });
        let ex = Exchange::read(NODE, Address::new(0x1000, 0), DataType::Unsigned32).expect("node");
        let progress = bus.drive_exchange(ex, short_deadline(50)).expect("retry");
        assert!(matches!(progress, Progress::Complete(_)));
        assert_eq!(bus.aborts_sent, 1, "exactly one ProtocolTimeout ABORT");
    }

    #[test]
    fn second_timeout_surfaces_after_single_retry() {
        let mut bus = ScriptedBus::new(Faults {
            swallow_requests: u32::MAX,
            ..Faults::default()
        });
        let ex = Exchange::read(NODE, Address::new(0x1000, 0), DataType::Unsigned32).expect("node");
        let err = bus
            .drive_exchange(ex, short_deadline(20))
            .expect_err("peer stays silent");
        assert!(matches!(err, HostError::Timeout));
        // BUGS.md #11: the peer must not be left with a dangling transaction
        // when we give up — the final timeout also sends a ProtocolTimeout
        // ABORT (first timeout + final give-up = 2).
        assert_eq!(bus.aborts_sent, 2);
    }

    #[test]
    fn stale_response_is_drained_before_new_transaction() {
        // BUGS.md H1: 新事务起点（发出首帧前）必须排空本节点 cmd 套接字上的
        // 陈旧响应，否则上一事务残留/迟到的响应会被当作新事务首帧喂入
        // `SdoClient::on_response`，造成状态错乱（UnexpectedFrame 等）。
        // 这里预置一条陈旧响应，验证它被事务起点排空、事务照常完成。
        let mut bus = ScriptedBus::new(Faults::default()).with_blob(blob());
        bus.inbox.push_back([0x60, 0x00, 0x10, 0, 0, 0, 0, 0]);
        let v = bus
            .sdo_read(Address::new(0x1000, 0), DataType::Unsigned32)
            .expect("陈旧帧不应污染新事务");
        assert_eq!(bus.inbox.len(), 0, "陈旧帧应已被事务起点排空");
        assert_eq!(v, Value::Uint(0), "陈旧帧被排空后事务应正常读到真实值");
    }

    #[test]
    fn toggle_violation_sends_abort_before_erroring() {
        let mut bus = ScriptedBus::new(Faults {
            flip_segment_toggle: true,
            ..Faults::default()
        });
        let err = bus
            .sdo_read(Address::new(0x2501, 0), DataType::Domain)
            .expect_err("corrupted segment must fail");
        assert!(
            matches!(err, HostError::Core(_) | HostError::ProtocolIncomplete),
            "corrupted segment must fail, got {err:?}"
        );
        assert_eq!(bus.aborts_sent, 1, "ABORT must precede the error return");
    }

    // 引用检查：response_cob 仅由宏内部使用；保留导入以免未用告警。
    const _: u16 = response_cob_id(NODE);
}
