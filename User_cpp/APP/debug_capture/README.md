# Debug Capture — 无线充电调试数据采集模块

配合 [duo-duo-box](https://gitee.com/duo-duo-box) 通用调试器，通过 **J-Link RTT 实时图表** 或 **内存直接读取** 两种方式观测 STM32 内部变量。

## 架构

```
┌──────────────────────────────────────────────────────────┐
│                    STM32G474 (170 MHz)                   │
│                                                          │
│  HRTIM ──ADC触发──▶ ADC3 DMA ──回调──▶ processAdc3()    │
│                    (469 Hz)               │              │
│                                           │ feed()       │
│                                           ▼              │
│                              ┌─────────────────────┐     │
│                              │  g_liveData          │     │
│                              │  (始终更新, 28 B)    │◀───┼── J-Link 内存直读
│                              │  volatile, 可直读   │     │     duo-duo-box 通用调试器
│                              └─────────────────────┘     │
│                                           │              │
│                              ┌─ g_enable = 1 时 ──┐     │
│                              │                    │     │
│                              ▼                    ▼     │
│                      ┌──────────────┐    ┌──────────┐   │
│                      │  环形缓冲     │    │ TIM6 1kHz│   │
│                      │  512 × 28 B  │───▶│ ÷2=500Hz │   │
│                      │  (14.3 KB)   │    │ transmit │   │
│                      └──────────────┘    └────┬─────┘   │
│                                               │         │
│                                    SEGGER_RTT_printf    │
│                                    4 KB 上行缓冲       │
│                                               │         │
└───────────────────────────────────────────────┼─────────┘
                                                ▼
                                        J-Link RTT Viewer
                                        duo-duo-box 图表
```

## 数据通道

RTT 输出 `ch:` 前缀 CSV 格式，一行一个采样点：

```
ch:tick,vRaw,iRaw,vFilt,iFilt,power,askPtr,askValid\r\n
```

| 通道 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | `tick` | uint32 | DWT CYCCNT 时间戳 (CPU 周期) |
| 1 | `vRaw` | UINT16 | 发射端电压 ADC raw 码值 |
| 2 | `iRaw` | UINT16 | 发射端电流 ADC raw 码值 |
| 3 | `vFilt` | float | 一阶低通滤波后电压 (工程值) |
| 4 | `iFilt` | float | 一阶低通滤波后电流 (工程值) |
| 5 | `power` | float | 发射功率 = vFilt × iFilt |
| 6 | `askPtr` | uint16 | ASK 解码器前导码匹配进度 (0–40) |
| 7 | `askValid` | uint8 | ASK 最近一帧校验有效 (0/1) |

## 缓冲区计算

| 项目 | 大小 | 说明 |
|------|------|------|
| 环形缓冲 `g_buffer[512]` | 14.3 KB | 512 样本 × 28 B，填满 ~1 秒 (469 Hz) |
| RTT 上行 `s_rttUpBuffer` | 4 KB | SEGGER_RTT 专用，NO_BLOCK_SKIP 模式 |
| `g_liveData` | 28 B | 始终更新，不依赖 g_enable |
| **合计** | **~18.3 KB** | MCU 128 KB RAM，当前总用量 ~46 KB |

数据产生速率 ≈ 469 Hz × 70 字节/行 ≈ **32.8 KB/s**，J-Link RTT 实测可到 MB/s 级别，链路不构成瓶颈。

## API 参考

### `void init()`

上电时调用一次，完成：
- DWT CYCCNT 使能（若未由 `debug_trace_init()` 开启）
- SEGGER_RTT 通道 0 配置：4 KB 专用缓冲 + NO_BLOCK_SKIP 模式

### `void feed(...)`

由 **ADC DMA 完成中断** (`processAdc3`) 调用，参数来源于最新采样数据：

```cpp
DebugCapture::feed(
    VVV,                           // float vRaw    — 电压 ADC raw
    III,                           // float iRaw    — 电流 ADC raw
    transmitterVoltage_,           // float vFilt   — 滤波后电压
    transmitterCurrent_,           // float iFilt   — 滤波后电流
    transmitterPower,              // float power   — 发射功率
    askDecoder_.getBitBufferPointer(), // uint16_t askPtr
    askDecoder_.isValid() ? 1 : 0);    // uint8_t  askValid
```

行为：
- **始终**更新 `g_liveData`（供调试器轮询）
- **仅当 `g_enable ≠ 0`** 时写入环形缓冲
- ISR 安全：临界区保护与 `transmit()` 的并发访问

### `void transmit()`

由 500 Hz 定时器任务调用，每批最多发送 **16 条**，防止过度占用 ISR：

```cpp
// SEGGER_RTT_printf 输出格式
"ch:%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u\r\n"
```

### `void onTimPeriodElapsed(TIM_HandleTypeDef *htim)`

TIM6 (1 kHz) 回调分发入口，内部 **÷2 分频** 得到 500 Hz 传输节拍。在 `HAL_TIM_PeriodElapsedCallback` 中调用。

## 使用方式

### 方式一：J-Link RTT 实时图表 (duo-duo-box)

1. 编译烧录固件
2. 打开 duo-duo-box → **J-Link RTT** 页面 → 连接设备
3. J-Link Commander 中执行 `w4 0x20003FF4 1`（使能采集）
4. RTT Terminal 0 出现 `ch:...` 格式数据流
5. 启用**图表解析**，8 个通道自动绑定曲线
6. 调试结束执行 `w4 0x20003FF4 0`（停止采集）

> **注意**：`g_enable` 地址每次编译可能变化，可在 map 文件或 ELF 中搜索 `g_enable` 获取当前地址。
> 也可以通过在 duo-duo-box 中直接填入符号名由调试器自动解析。

### 方式二：通用调试器内存直读 (duo-duo-box)

无需 RTT，通过 J-Link/ST-Link 直接读取 `g_liveData` 结构体：

1. 编译后在 map 文件中搜索 `g_liveData` 获取地址
2. duo-duo-box → **通用调试器** → 连接调试器
3. 填入结构体成员地址（`g_liveData` 基地址 + 偏移）：

| 通道 | 变量 | 偏移 | 类型 |
|------|------|------|------|
| 0 | tick | `base + 0` | uint32 |
| 1 | vRaw | `base + 4` | float |
| 2 | iRaw | `base + 8` | float |
| 3 | vFilt | `base + 12` | float |
| 4 | iFilt | `base + 16` | float |
| 5 | power | `base + 20` | float |
| 6 | askPtr | `base + 24` | uint16 |
| 7 | askValid | `base + 26` | uint8 |

> `g_liveData` 始终更新，不依赖 `g_enable`，适合长期低速趋势监控。

### 方式三：Ozone / J-Link Commander 手动读取

```bash
# J-Link Commander
mem32 0x20005004 7    # 读取 g_liveData 全部 7 个 32-bit 字
mem32 0x20003FF8 1    # 读取 g_count（待发送样本数）
w4 0x20003FF4 1       # 使能采集
```

### 方式四：Serial Oscilloscope / 自定义脚本

RTT 输出为标准 CSV，可直接导入 Python/Excel：

```python
import csv, io
data = io.StringIO(rtt_output)
reader = csv.reader(data)
for row in reader:
    tick, vRaw, iRaw, vFilt, iFilt, power, askPtr, askValid = row
```

## 文件结构

```
User_cpp/debug_capture/
├── debug_capture.hpp    # 头文件：Sample / LiveDebugData 结构体，API 声明
├── debug_capture.cpp    # 实现：环形缓冲、RTT 传输、500 Hz 任务
└── README.md            # 本文档
```

依赖：
```
User_cpp/Driver/jlink_rtt/   — SEGGER_RTT 库
User_cpp/APP/sampling/       — feed() 调用方 (processAdc3)
User_cpp/APP/wireless_charge/ — init() 与 onTimPeriodElapsed 分发
```

## 配置参数

所有可调常量均在 `debug_capture.cpp` 顶部：

| 常量 | 默认值 | 说明 |
|------|--------|------|
| `kBufferSize` | 512 | 环形缓冲样本容量 |
| `kRttUpBufferSize` | 4096 | RTT 上行缓冲字节数 |
| `kTransmitBatchMax` | 16 | 单次 transmit 最大发送样本数 |
| `kTim6FrequencyHz` | 1000 | TIM6 驱动频率 |
| `kDebugTaskDivider` | 2 | 分频系数 (1000/500) |

## 注意事项

1. **`g_enable` 默认关闭**：上电不产生 RTT 输出，避免干扰正常通信。由调试器按需开启。
2. **临界区保护**：`feed()` 和 `transmit()` 通过 `__disable_irq()` 互斥，保证环形缓冲一致性。
3. **NO_BLOCK_SKIP 模式**：RTT 缓冲满时静默丢弃数据，不会阻塞 MCU 运行。J-Link 未连接时也安全。
4. **RAM 占用**：调试模块总计 ~18 KB，MCU 剩余 ~64 KB 供堆栈和应用使用。
5. **DWT 时间戳**：使用 CPU 周期计数 (170 MHz)，溢出周期约 25 秒。上位机应处理 32-bit 回绕。
