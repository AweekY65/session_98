# isotp_local — 本地 CAN ISO-TP 编解码与重组工具

纯本地实现的 ISO-TP（ISO 15765-2）协议编解码、拆包与重组库。
所有 CAN frame、会话状态和测试数据只存在于内存或本地文件中：
**不连接 SocketCAN、真实车辆、ECU 或任何外部服务**，时间全部通过
`VirtualClock` 虚拟时钟模拟，不使用真实 `sleep`。

## 帧格式（经典 CAN，8 字节）

每个 ISO-TP 帧首字节高 4 位为 PCI 类型：

| 类型 | PCI | 布局 |
|------|-----|------|
| Single Frame (SF) | `0x0L` | byte0 = `0x00 \| len`(1..=7)，后随 `len` 字节数据 |
| First Frame (FF) | `0x1L` | byte0 = `0x10 \| (len>>8)&0x0F`，byte1 = `len&0xFF`，后随 6 字节数据 |
| Consecutive Frame (CF) | `0x2S` | byte0 = `0x20 \| (sn&0x0F)`，后随 7 字节数据 |
| Flow Control (FC) | `0x3F` | byte0 = `0x30 \| FS`，byte1 = BlockSize，byte2 = STmin |

- **Sequence Number**：CF 序号从 1 开始，每帧加 1，`15 -> 0` 回绕。
- **Flow Status**：`0`=ClearToSend，`1`=Wait，`2`=Overflow（中止发送）。
- **STmin**：`0x00..=0x7F` 表示 0..127 ms；`0xF1..=0xF9` 表示 100..900 µs；
  其余取值非法（`InvalidStMin`）。
- 单帧上限 7 字节；FF 长度字段 12 bit，协议上限 **4095 字节**。

## 状态机

### 发送端 `Sender`（`src/sender.rs`）

```
Idle --start()--> (len<=7) 发 SF --> Done
                 (len>7)  发 FF --> WaitFc --FC.CTS--> SendCf --发完--> Done
                                    |  ^                |
                                    |  \--FC.Wait-------/  (BS 用完后回到 WaitFc)
                                    \--N_Bs 超时 / FC.Overflow--> Failed
```

- `start()` 产出 SF 或 FF；`poll(&clock)` 在 STmin 到期且块内额度未用完时
  产出下一个 CF，并自动生成回绕的 sequence number。
- **Block Size**：收到 FC(BS=n) 后最多连续发送 n 个 CF，随后回到 `WaitFc`
  等待下一个 FC；`BS=0` 表示不限。
- **STmin**：相邻 CF 之间至少间隔 STmin（虚拟时钟判定，`poll` 未到期返回 `None`）。
- **N_Bs**：`on_tick` 检测等待 FC 超时（默认 1 s）。

### 接收端 `Receiver`（`src/receiver.rs`）

```
Idle --SF--> Complete(payload)
     --FF--> Receiving (回发 FC.CTS) --CF 收满 declared--> Complete(payload)
            --CF 序号错/重复/长度越界/N_Cr 超时--> 报错并复位到 Idle
```

- 按顺序校验 SN；重复帧报 `DuplicateSequence`，缺帧/乱序报 `SequenceMismatch`。
- **长度校验**：只有累计字节数恰好等于 FF 声明长度才返回 `Complete`；
  数据超出声明长度报 `LengthMismatch`/`UnexpectedFrame`；数据不足由
  N_Cr 超时（`on_tick`）检出，绝不静默成功。

## 长度与资源限制

- `ReceiverConfig::max_payload`：单个 payload 上限。FF 声明长度超限时在
  **分配任何缓冲区之前**拒绝（`PayloadTooLarge`），恶意 FF 不会造成无界内存分配。
- `ReceiverConfig::max_buffer`：重组缓冲区总上限（`BufferLimitExceeded`）。
- 发送侧 `Sender::new` 拒绝空 payload 和超过 4095 字节的输入。

## 目录结构

- `src/frame.rs` — `CanFrame`、PCI 编解码、SF/FF/CF/FC 构造函数、STmin 编码
- `src/sender.rs` — 拆包与 Flow Control 发送状态机
- `src/receiver.rs` — 重组、序号/长度/超时校验
- `src/clock.rs` — `VirtualClock` 虚拟时钟
- `src/bin/isotp_verify.rs` — 独立测试 runner

## 测试方法

不使用 `cargo test`，运行独立 runner，逐项输出 PASS/FAIL：

```sh
cargo run --bin isotp_verify
```

覆盖：单帧、多帧、sequence number 回绕（300 字节）、丢帧、重复帧、
长度声明与实际不符（超时 + 超长）、Flow Control（block size / STmin /
Wait / Overflow）、N_Bs 超时、超大 First Frame 拒绝、4095 字节边界。
任一失败时进程以非零码退出。
