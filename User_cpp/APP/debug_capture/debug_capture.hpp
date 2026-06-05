#pragma once

#include <cstdint>

extern "C" {
#include "tim.h"
}

// 环形缓冲 debug 采样，配合 duo-duo-box 通用调试器使用。
// - feed() 由 ADC 采样中断上下文调用（快速返回，不做阻塞操作）。
// - transmit() 由 500 Hz 定时器任务调用，通过 J-Link RTT 输出 ch: 前缀 CSV 数据流。
// - g_liveData 可由 duo-duo-box 通用调试器直接按地址读取，无须使能 RTT。
namespace DebugCapture
{

// ---- 单次采样点（环形缓冲条目）----

struct Sample
{
    uint32_t tick;        // DWT->CYCCNT 时间戳
    uint16_t vRaw;           // 电压 ADC raw 码值
    uint16_t iRaw;           // 电流 ADC raw 码值
    float vFilt;          // 滤波后电压
    float iFilt;          // 滤波后电流
    float power;          // 发射功率
    uint16_t askPtr;      // ASK bitBufferPointer_
    uint8_t askValid;     // ASK valid_ flag
};
// sizeof(Sample) = 4+4+4+4+4+4+2+1+(1 pad) = 28 字节

// ---- 实时调试数据（duo-duo-box 通用调试器直接读取）----

struct LiveDebugData
{
    uint32_t tick;        // DWT->CYCCNT 时间戳（最新一次 feed 的时刻）
    uint16_t vRaw; // 电压 ADC raw 码值
    uint16_t iRaw; // 电流 ADC raw 码值
    float vFilt;          // 滤波后电压
    float iFilt;          // 滤波后电流
    float power;          // 发射功率
    uint16_t askPtr;      // ASK bitBufferPointer_
    uint8_t askValid;     // ASK valid_ flag
};
// sizeof(LiveDebugData) = 4+4+4+4+4+4+2+1+(1 pad) = 28 字节

// ---- 生命周期 ----

// 上电时调用一次：初始化 DWT 时间戳源与 RTT 上行缓冲区。
void init();

// ---- 数据采集（ADC 中断上下文，需快速返回）----

void feed(uint16_t vRaw, uint16_t iRaw, float vFilt, float iFilt, float power, uint16_t askPtr, uint8_t askValid);

// ---- 数据传输（500 Hz 定时器任务上下文）----

// 将缓冲区中未发送的样本通过 RTT 输出（ch: 前缀 CSV 格式）。
void transmit();

// 由 HAL_TIM_PeriodElapsedCallback 统一分发。
// 内部使用软件分频器将 TIM6 的 1 kHz 降为 500 Hz。
void onTimPeriodElapsed(TIM_HandleTypeDef *htim);

// ---- 调试器可读写的外部变量 ----

extern volatile uint32_t g_enable;   // 非零 = 正在采集 / RTT 传输
extern volatile uint32_t g_count;    // 缓冲区中待发送样本数
extern const uint32_t g_capacity;    // 缓冲区容量
extern Sample g_buffer[];            // 环形缓冲数组
extern volatile LiveDebugData g_liveData; // 最新采样值（始终更新，无须 g_enable）

} // namespace DebugCapture
