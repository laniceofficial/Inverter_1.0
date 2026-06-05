#pragma once

extern "C" {
#include "hrtim.h"
#include "main.h"
}

#include <cstdint>

#if defined(HAL_HRTIM_MODULE_ENABLED)

namespace Driver
{

// 只封装当前无线充电用到的 HRTIM 子定时器。
enum class HrtimTimer : uint8_t
{
    TimerB, // 半桥 Buck/Boost PWM
    TimerE, // 无线充电 E 路全桥 PWM
    TimerF, // 无线充电 F 路全桥 PWM
};

struct HrtimConfig
{
    uint32_t timerMask = 0U;                // 参与工作的 HRTIM_TIMERID_* 掩码
    uint16_t period = 22666U; // E/F 默认无线充电周期
    float phase = 0.0f;                     // E/F 相对主定时器复位相位
    bool masterRepetitionInterrupt = false; // 是否打开 Master 重复中断
};

bool configure(const HrtimConfig& config);
void startTimers(uint32_t timerMask);
void stopTimers(uint32_t timerMask);
void enableOutputs(uint32_t outputMask);
void disableOutputs(uint32_t outputMask);

bool setPeriod(HrtimTimer timer, uint16_t period);
bool setDuty(HrtimTimer timer, float duty);
bool setCompare(HrtimTimer timer, uint32_t compareUnit, uint32_t value);
bool setPhase(float phase);
bool setComplementaryDuty(HrtimTimer timer, float duty, uint32_t adcTriggerCompare);

uint32_t getTimerId(HrtimTimer timer);
uint32_t getTimerIndex(HrtimTimer timer);
uint32_t getOutputMask(HrtimTimer timer);
uint16_t getPeriod(HrtimTimer timer);
float clampDuty(float duty, float minDuty = 0.0f, float maxDuty = 0.95f);

} // namespace Driver

#endif // HAL_HRTIM_MODULE_ENABLED
