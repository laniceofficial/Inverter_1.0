#include "bsp_hrtim.hpp"

#if defined(HAL_HRTIM_MODULE_ENABLED)

namespace Driver
{
namespace
{

constexpr uint32_t kSupportedTimerMask =
    HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_E | HRTIM_TIMERID_TIMER_F;
constexpr uint16_t kMasterDefaultPeriod = 22666U;

struct TimerDescriptor
{
    HrtimTimer timer;    // C++ 层枚举
    uint32_t timerId;    // HAL 启停使用的 HRTIM_TIMERID_*
    uint32_t timerIndex; // 寄存器数组下标 HRTIM_TIMERINDEX_*
    uint32_t outputMask; // 互补输出通道掩码
    uint16_t period;     // 当前缓存的周期，设置比较值时用于限幅
};

TimerDescriptor g_timers[] = {
    {HrtimTimer::TimerB, HRTIM_TIMERID_TIMER_B, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2, 27200U},
    {HrtimTimer::TimerE, HRTIM_TIMERID_TIMER_E, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2, 22666U},
    {HrtimTimer::TimerF, HRTIM_TIMERID_TIMER_F, HRTIM_TIMERINDEX_TIMER_F, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2, 22666U},
};

uint32_t g_configuredTimerMask = 0U;
float g_phase = 0.0f;

TimerDescriptor* findTimer(const HrtimTimer timer)
{
    for (auto& descriptor : g_timers)
    {
        if (descriptor.timer == timer)
        {
            return &descriptor;
        }
    }

    return nullptr;
}

uint32_t clampCompare(const TimerDescriptor& descriptor, const uint32_t value)
{
    if (descriptor.period == 0U)
    {
        return 0U;
    }

    return (value >= descriptor.period) ? static_cast<uint32_t>(descriptor.period - 1U) : value;
}

float clampPhaseValue(const float phase)
{
    if (phase < 0.1f)
    {
        return 0.1f;
    }

    if (phase > 0.9f)
    {
        return 0.9f;
    }

    return phase;
}

void setMasterRepetitionInterrupt(const bool enable)
{
    if (enable)
    {
        __HAL_HRTIM_MASTER_ENABLE_IT(&hhrtim1, HRTIM_MASTER_IT_MREP);
    }
    else
    {
        __HAL_HRTIM_MASTER_DISABLE_IT(&hhrtim1, HRTIM_MASTER_IT_MREP);
    }
}

} // namespace

bool configure(const HrtimConfig& config)
{
    if ((config.timerMask & ~kSupportedTimerMask) != 0U)
    {
        return false;
    }

    g_configuredTimerMask = config.timerMask;
    if (config.period != 0U)
    {
        // Master 周期作为 E/F 相移参考，同时同步写入所有参与工作的子定时器。
        hhrtim1.Instance->sMasterRegs.MPER = config.period;
        hhrtim1.Instance->sMasterRegs.MCMP2R = config.period / 2U;

        for (auto& descriptor : g_timers)
        {
            if ((config.timerMask & descriptor.timerId) != 0U)
            {
                setPeriod(descriptor.timer, config.period);
            }
        }
    }

    setMasterRepetitionInterrupt(config.masterRepetitionInterrupt);
    return setPhase(config.phase);
}

void startTimers(const uint32_t timerMask)
{
    if ((timerMask & HRTIM_TIMERID_MASTER) != 0U)
    {
        HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
    }

    for (const auto& descriptor : g_timers)
    {
        if ((timerMask & descriptor.timerId) != 0U)
        {
            HAL_HRTIM_WaveformCounterStart(&hhrtim1, descriptor.timerId);
        }
    }
}

void stopTimers(const uint32_t timerMask)
{
    for (const auto& descriptor : g_timers)
    {
        if ((timerMask & descriptor.timerId) != 0U)
        {
            HAL_HRTIM_WaveformCountStop(&hhrtim1, descriptor.timerId);
        }
    }

    if ((timerMask & HRTIM_TIMERID_MASTER) != 0U)
    {
        HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_MASTER);
    }
}

void enableOutputs(const uint32_t outputMask)
{
    if (outputMask != 0U)
    {
        HAL_HRTIM_WaveformOutputStart(&hhrtim1, outputMask);
    }
}

void disableOutputs(const uint32_t outputMask)
{
    if (outputMask != 0U)
    {
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, outputMask);
    }
}

bool setPeriod(const HrtimTimer timer, const uint16_t period)
{
    TimerDescriptor* descriptor = findTimer(timer);
    if ((descriptor == nullptr) || (period == 0U))
    {
        return false;
    }

    descriptor->period = period;
    hhrtim1.Instance->sTimerxRegs[descriptor->timerIndex].PERxR = period;
    return true;
}

bool setDuty(const HrtimTimer timer, const float duty)
{
    TimerDescriptor* descriptor = findTimer(timer);
    if (descriptor == nullptr)
    {
        return false;
    }

    const float limitedDuty = clampDuty(duty);
    const uint32_t period = descriptor->period;
    const uint32_t highTicks = static_cast<uint32_t>(static_cast<float>(period) * limitedDuty);
    // 普通 PWM 居中输出：CMP1 为上升沿，CMP3 为下降沿。
    const uint32_t cmp1 = (period > highTicks) ? ((period - highTicks) / 2U) : 0U;
    const uint32_t cmp3 = cmp1 + highTicks;

    setCompare(timer, HRTIM_COMPAREUNIT_1, cmp1);
    return setCompare(timer, HRTIM_COMPAREUNIT_3, cmp3);
}

bool setCompare(const HrtimTimer timer, const uint32_t compareUnit, const uint32_t value)
{
    const TimerDescriptor* descriptor = findTimer(timer);
    if (descriptor == nullptr)
    {
        return false;
    }

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, descriptor->timerIndex, compareUnit, clampCompare(*descriptor, value));
    return true;
}
// phase=0 → E/F 180°反相（最大功率），phase=1 → E/F 同相（最小功率），单调递减。
bool setPhase(float phase)
{
    g_phase = clampPhaseValue(phase);

    uint32_t masterPeriod = hhrtim1.Instance->sMasterRegs.MPER;
    if (masterPeriod == 0U)
    {
        masterPeriod = kMasterDefaultPeriod;
    }

    // 沿用已验证的对称公式：MCMP1/3 绕 period/4 对称分布。
    // 将用户 phase 映射到 old_phase：user=0→old=0.5(最大)，user=1→old=0(最小)
    const float oldPhase = 0.5f * (g_phase);
    const int32_t center = static_cast<int32_t>(masterPeriod / 4U);
    const int32_t offset = static_cast<int32_t>(
        static_cast<float>(masterPeriod) * oldPhase * 0.5f);
    int32_t cmp1 = center - offset;
    int32_t cmp3 = center + offset;
    const int32_t maxCompare = static_cast<int32_t>(masterPeriod - 1U);

    if (cmp1 < 1)
    {
        cmp1 = 1;  // 避免 MCMP=0 触发边沿问题
    }
    if (cmp3 > maxCompare)
    {
        cmp3 = maxCompare;
    }

    hhrtim1.Instance->sMasterRegs.MCMP1R = static_cast<uint32_t>(cmp1);
    hhrtim1.Instance->sMasterRegs.MCMP3R = static_cast<uint32_t>(cmp3);
    return true;
}


bool setComplementaryDuty(const HrtimTimer timer, const float duty, const uint32_t adcTriggerCompare)
{
    TimerDescriptor* descriptor = findTimer(timer);
    if (descriptor == nullptr)
    {
        return false;
    }

    const uint32_t cmp3 =
        static_cast<uint32_t>(static_cast<float>(descriptor->period) * clampDuty(duty));
    // 半桥互补输出从周期起点开始导通，CMP4 专门留给 ADC 触发点。
    setCompare(timer, HRTIM_COMPAREUNIT_1, 0U);
    setCompare(timer, HRTIM_COMPAREUNIT_3, cmp3);
    return setCompare(timer, HRTIM_COMPAREUNIT_4, adcTriggerCompare);
}

uint32_t getTimerId(const HrtimTimer timer)
{
    const TimerDescriptor* descriptor = findTimer(timer);
    return (descriptor == nullptr) ? 0U : descriptor->timerId;
}

uint32_t getTimerIndex(const HrtimTimer timer)
{
    const TimerDescriptor* descriptor = findTimer(timer);
    return (descriptor == nullptr) ? 0U : descriptor->timerIndex;
}

uint32_t getOutputMask(const HrtimTimer timer)
{
    const TimerDescriptor* descriptor = findTimer(timer);
    return (descriptor == nullptr) ? 0U : descriptor->outputMask;
}

uint16_t getPeriod(const HrtimTimer timer)
{
    const TimerDescriptor* descriptor = findTimer(timer);
    return (descriptor == nullptr) ? 0U : descriptor->period;
}

float clampDuty(const float duty, const float minDuty, const float maxDuty)
{
    if (duty < minDuty)
    {
        return minDuty;
    }
    if (duty > maxDuty)
    {
        return maxDuty;
    }
    return duty;
}

} // namespace Driver

#endif // HAL_HRTIM_MODULE_ENABLED
