# isotp_local — 本地 CAN ISO-TP 编解码与重组工具

一个纯本地、纯内存的 CAN ISO-TP（ISO 15765-2 子集）实现。**不连接 SocketCAN、
真实车辆、ECU 或任何外部服务**；所有 CAN frame、会话状态和测试数据只存在于
内存或本仓库文件中。时间语义通过 `VirtualClock` 模拟，不使用真实 `sleep`。

## 帧格式（经典 CAN，8 字节）

每帧首字节为 PCI（Protocol Control Information），高 4 位为帧类型：

| 类型 | PCI 高半字节 | 布局 |
|------|--------------|------|
| Single Frame (SF)      | `0x0` | `0x0L` + 最多 7 字节数据，`L` = SF_DL（1–7） |
| First Frame (FF)       | `0x1` | `0x1H LL` + 6 字节数据，12 位 FF_DL 声明总长度 |
| Consecutive Frame (CF) | `0x2` | `0x2N` + 最多 7 字节数据，`N` = 4 位 sequence number |
| Flow Control (FC)      | `0x3` | `0x3F BS ST`，`F` = FS（0=CTS，1=Wait，2=Overflow） |

- **Sequence number**：CF 的 SN 从 1 开始，每帧加 1，4 位回绕（…→15→0→1…）。
- **Flow Control**：`BS`（block size）为接收方在一个块内允许的最大 CF 数量，
  `0` 表示不限；`STmin` 为相邻 CF 之间的最小间隔（毫秒，虚拟时钟）。

## 发送侧（`Sender` / `transmit`）

- `segment(id, payload)`：payload ≤ 7 字节生成单个 SF；否则生成 FF + 若干 CF，
  自动编号并回绕。payload 超过 `MAX_PAYLOAD`（4095，12 位 FF_DL 上限）直接报错。
- `Sender` 状态机：`WaitFlowControl → Sending → (WaitBlock → Sending)* → Done`，
  另有 `Aborted`（收到 FS=Overflow）。FS=Wait 时回到 `WaitFlowControl`。
- 收到 CTS 后按 `block_size` 发送一个块，块耗尽进入 `WaitBlock` 等待下一个 FC；
  相邻帧间隔 `stmin_ms`，通过 `VirtualClock::advance` 推进，无真实等待。
- `transmit(id, payload, bs, stmin, clock)`：便捷驱动函数，返回
  `(虚拟时间戳, 帧)` 列表。

## 接收侧（`Reassembler`）

- SF：校验 SF_DL（1–7 且不超过帧内数据长度），立即完成。
- FF：解析 12 位声明长度；**超过 `Reassembler::new(max_payload)` 上限时在分配
  任何缓冲区之前拒绝**（恶意 FF 不会导致无界内存分配）；FF_DL ≤ 7 视为非法。
- CF：必须处于接收中且 SN 严格等于期望值，否则报错：
  - SN 等于上一帧 → `DuplicateFrame`（重复帧）；
  - 其他乱序 → `SequenceMismatch`（丢帧/错误序号）；
  - 无 FF 直接收到 CF → `UnexpectedFrame`。
- 长度校验：累计达到声明长度才返回 `Completed`；传输未完成时收到新的
  SF/FF → `Incomplete`（数据不足不静默成功）；完成后多余的 CF →
  `UnexpectedFrame`（数据超长不静默成功）。

## 长度与缓存限制

- `MAX_PAYLOAD = 4095`：协议 12 位长度上限，发送侧硬限制。
- `Reassembler::new(max_payload)`：接收侧缓存上限，缓冲区按声明长度一次性
  预留且不超过该上限。

## 构建与测试

```sh
cargo build
cargo run --bin isotp_verify
```

不使用 `cargo test`。`isotp_verify` 逐项输出 `PASS`/`FAIL`，覆盖：单帧、
多帧、sequence number 回绕、丢帧、重复帧、SF_DL 长度错误、传输截断、
完成后多余数据、Flow Control 编解码、block size、STmin（虚拟时钟）、
Wait/Overflow、超大 First Frame 拒绝、超大 payload 拒绝、端到端收发。
任一失败时进程以非零码退出。
