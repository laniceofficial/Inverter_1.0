#include "debug_capture.hpp"
#include <cstdint>
#include "SEGGER_RTT.h"
#include "bsp_dwt.hpp"

namespace DebugCapture
{

// ===================================================================
// 缓冲区计算
// ===================================================================
// ADC 触发频率 = HRTIM 主定时器频率 ≈ 170 MHz / 16 / 22666 ≈ 469 Hz
// feed() 调用频率 ≈ 469 Hz（每次 ADC 转换完成触发一次）
//
// 环形缓冲：
//   kBufferSize = 512 样本 × sizeof(Sample) 28 B ≈ 14.3 KB
//   填满时间 = 512 / 469 ≈ 1.09 秒
//
// RTT 上行缓冲：
//   kRttUpBufferSize = 4096 B（4 KB）
//   每条 CSV 行 ≈ 60–80 字符
//   数据产生速率 ≈ 469 × 70 ≈ 32.8 KB/s
//   transmit() 消费速率 = 500 Hz × 16 样本 = 8000 样本/s（远大于产生速率）
//   MCU 总 RAM: 128 KB, 当前用量 ~46 KB, 调试开销 ~18 KB, 余量 ~64 KB
// ===================================================================

constexpr uint32_t kBufferSize = 512U;
constexpr uint32_t kRttChannel = 0U;
constexpr uint32_t kTransmitBatchMax = 16U; // 单次 transmit 最多发送样本数
constexpr uint32_t kTim6FrequencyHz = 1000U;
constexpr uint32_t kDebugTaskDivider = kTim6FrequencyHz / 500U; // = 2
constexpr uint32_t kRttUpBufferSize = 4096U;

// ---- 环形缓冲 ----

Sample g_buffer[kBufferSize] = {};

volatile uint32_t g_enable = 0U;
volatile uint32_t g_count = 0U;
const uint32_t g_capacity = kBufferSize;

static uint32_t s_writeIndex = 0U;
static uint32_t s_readIndex = 0U;
static Driver::DWT_c *DWT_ins = nullptr;
// ---- RTT 上行缓冲区 ----

static char s_rttUpBuffer[kRttUpBufferSize] = {};

// ---- 实时数据（始终更新，duo-duo-box 通用调试器按地址读取）----

volatile LiveDebugData g_liveData = {};

// ---- 内部辅助 ----

static inline uint32_t enterCritical()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void exitCritical(const uint32_t primask)
{
    __set_PRIMASK(primask);
}

// ---- 生命周期 ----

void init()
{
    // DWT CYCCNT 已由 debug_trace_init() 使能，此处仅验证。
    // 若未使能则手动开启，保证时间戳可用。
    DWT_ins = Driver::DWT_c::Get_DwtInstance();

    // 配置 RTT 上行通道：分配专用 4 KB 缓冲区，无阻塞模式，调试器未连接时自动丢弃。
    SEGGER_RTT_ConfigUpBuffer(
            kRttChannel, "DebugCapture", s_rttUpBuffer, kRttUpBufferSize, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    s_writeIndex = 0U;
    s_readIndex = 0U;
    g_count = 0U;
    g_enable = 0U;
}

// ---- 数据采集（ADC 中断上下文）----

void feed(const uint16_t vRaw,
          const uint16_t iRaw,
          const float vFilt,
          const float iFilt,
          const float power,
          const uint16_t askPtr,
          const uint8_t askValid)
{
    // 实时数据始终更新，供 duo-duo-box 通用调试器按地址轮询读取。
    g_liveData.tick = (uint32_t)DWT_ins->GetTimeline_ms();
    g_liveData.vRaw = vRaw;
    g_liveData.iRaw = iRaw;
    g_liveData.vFilt = vFilt;
    g_liveData.iFilt = iFilt;
    g_liveData.power = power;
    g_liveData.askPtr = askPtr;
    g_liveData.askValid = askValid;

    if (g_enable == 0U)
    {
        return;
    }

    // 临界区：防止被更高优先级 TIM6 ISR 中的 transmit() 打断。
    const uint32_t primask = enterCritical();

    const uint32_t count = g_count;
    if (count >= kBufferSize)
    {
        // 缓冲区满，丢弃最旧样本为新样本腾空间。
        s_readIndex = (s_readIndex + 1U) % kBufferSize;
    }
    else
    {
        g_count = count + 1U;
    }

    const uint32_t idx = s_writeIndex;
    s_writeIndex = (s_writeIndex + 1U) % kBufferSize;

    exitCritical(primask);

    Sample &s = g_buffer[idx];
    s.tick = g_liveData.tick;
    s.vRaw = vRaw;
    s.iRaw = iRaw;
    s.vFilt = vFilt;
    s.iFilt = iFilt;
    s.power = power;
    s.askPtr = askPtr;
    s.askValid = askValid;
}

// ===================================================================
// 轻量 float→字符串 格式化器
// SEGGER_RTT_printf 不支持 %f/%e/%g，因此自实现固定小数位转换，
// 在栈上拼接完整 CSV 行后通过 SEGGER_RTT_Write 发送。
// ===================================================================

// buf 末尾写入字符，返回推进后的指针
static inline char* wrc(char* buf, const char c)
{
    *buf = c;
    return buf + 1;
}

// 写入十进制无符号整数，返回推进后的指针
static char* wru32(char* buf, uint32_t val)
{
    if (val == 0U)
    {
        return wrc(buf, '0');
    }
    char tmp[10];
    uint8_t len = 0U;
    while (val > 0U)
    {
        tmp[len++] = static_cast<char>('0' + (val % 10U));
        val /= 10U;
    }
    while (len > 0U)
    {
        --len;
        buf = wrc(buf, tmp[len]);
    }
    return buf;
}

// 写入 float，格式 "整数.小数"，decimalPlaces 位小数，四舍五入
static char* wrf(char* buf, const float val, const uint8_t decimalPlaces)
{
    float v = val;
    if (v < 0.0f)
    {
        buf = wrc(buf, '-');
        v = -v;
    }
    const uint32_t intPart = static_cast<uint32_t>(v);
    buf = wru32(buf, intPart);
    if (decimalPlaces == 0U) { return buf; }
    buf = wrc(buf, '.');
    float frac = v - static_cast<float>(intPart);
    for (uint8_t i = 0U; i < decimalPlaces; ++i) { frac *= 10.0f; }
    uint32_t dec = static_cast<uint32_t>(frac + 0.5f);
    uint32_t div = 1U;
    for (uint8_t i = 1U; i < decimalPlaces; ++i) { div *= 10U; }
    for (uint8_t i = 0U; i < decimalPlaces; ++i)
    {
        buf = wrc(buf, static_cast<char>('0' + ((dec / div) % 10U)));
        div /= 10U;
    }
    return buf;
}

// 将 Sample 格式化为 "ch:tick,vRaw,iRaw,vFilt,iFilt,power,askPtr,askValid\r\n" 并发送
static void sendSample(const Sample& s)
{
    char line[128];
    char* p = line;

    p = wrc(p, 'c');  p = wrc(p, 'h');  p = wrc(p, ':');           // ch:
    p = wru32(p, s.tick);                p = wrc(p, ',');            // tick
    p = wru32(p, static_cast<uint32_t>(s.vRaw)); p = wrc(p, ',');   // vRaw
    p = wru32(p, static_cast<uint32_t>(s.iRaw)); p = wrc(p, ',');   // iRaw
    p = wrf(p, s.vFilt, 3U);             p = wrc(p, ',');            // vFilt
    p = wrf(p, s.iFilt, 3U);             p = wrc(p, ',');            // iFilt
    p = wrf(p, s.power, 3U);             p = wrc(p, ',');            // power
    p = wru32(p, static_cast<uint32_t>(s.askPtr)); p = wrc(p, ','); // askPtr
    p = wru32(p, static_cast<uint32_t>(s.askValid));                 // askValid
    p = wrc(p, '\r'); p = wrc(p, '\n');

    SEGGER_RTT_Write(kRttChannel, line, static_cast<unsigned>(p - line));
}

// ---- 数据传输（500 Hz 定时器任务上下文）----

void transmit()
{
    if (g_enable == 0U)
    {
        return;
    }

    uint32_t batchCount = 0U;

    while (batchCount < kTransmitBatchMax)
    {
        const uint32_t primask = enterCritical();
        const uint32_t count = g_count;
        if (count == 0U)
        {
            exitCritical(primask);
            return;
        }

        g_count = count - 1U;
        const uint32_t idx = s_readIndex;
        s_readIndex = (s_readIndex + 1U) % kBufferSize;
        exitCritical(primask);

        sendSample(g_buffer[idx]);
        ++batchCount;
    }
}

// ---- 定时器分发 ----

void onTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
    if (htim != &htim6)
    {
        return;
    }

    // 软件分频器：TIM6 1 kHz → 500 Hz。
    static uint32_t divider = 0U;
    ++divider;
    if (divider < kDebugTaskDivider)
    {
        return;
    }
    divider = 0U;

    transmit();
}

} // namespace DebugCapture
