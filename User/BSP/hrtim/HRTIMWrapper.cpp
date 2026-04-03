/**
 * @file HRTIMWrapper.cpp
 * @author
 * @brief HRTIM 包装层实现
 * @details 当前包装层采用“先配置，再启停”的使用方式。
 *          调用方需要先通过 configure() 明确参与工作的从定时器，
 *          然后再按顺序启动计数器、打开高边/低边输出。
 * @example
 * 典型用法如下，示例中启用 TIMER_B 和 TIMER_E：
 *
 * @code{.cpp}
 * HRTIM::Config config;
 * config.slaveTimerMask =
 *     HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_E;
 * config.enableMasterRepetitionInterrupt = true;
 * config.period = 40000U;
 * config.phase = 0.25f;
 *
 * if (HRTIM::configure(config))
 * {
 *   // 先启动主定时器和已配置的从定时器计数器
 *   HRTIM::startTimer();
 *
 *   // 根据需要打开高边或低边输出
 *   HRTIM::enableHighSideOutputs();
 *   // HRTIM::enableLowSideOutputs();
 *
 *   // 运行过程中可单独调整周期和移相
 *   HRTIM::setPeriod(38000U);
 *   HRTIM::setPhase(0.35f);
 * }
 *
 * // 需要停机时，先关闭输出并停止计数器
 * HRTIM::disableOutputs();
 * HRTIM::stopTimer();
 * @endcode
 *
 * @copyright Copyright (c) 2025
 */

#include "HRTIMWrapper.hpp"

#include "hrtim.h"

#if defined(HAL_HRTIM_MODULE_ENABLED)
namespace HRTIM
{
namespace
{
// 这里只暴露已经在 MX_HRTIM1_Init() 中完成初始化的从定时器。
// 这样可以保证包装层行为和底层真实配置一致，避免接口表面上“能开 A”，
// 实际硬件却根本没有把 TIMER_A 配好。
constexpr uint32_t kSupportedSlaveTimers =
    HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_E | HRTIM_TIMERID_TIMER_F;
constexpr uint16_t kDefaultPeriod = 54400U;

struct TimerDescriptor
{
    uint32_t timerId;
    uint32_t timerIndex;
    uint32_t highSideOutput;
    uint32_t lowSideOutput;
};

constexpr TimerDescriptor kTimerDescriptors[] = {
    {HRTIM_TIMERID_TIMER_B, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, HRTIM_OUTPUT_TB2},
    {HRTIM_TIMERID_TIMER_E, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE1, HRTIM_OUTPUT_TE2},
    {HRTIM_TIMERID_TIMER_F, HRTIM_TIMERINDEX_TIMER_F, HRTIM_OUTPUT_TF1, HRTIM_OUTPUT_TF2},
};

float clampPhase(const float phase)
{
    if (phase < 0.0f)
    {
        return 0.0f;
    }
    if (phase > 0.9f)
    {
        return 0.9f;
    }
    return phase;
}

bool isSupportedTimerMask(const uint32_t timerMask)
{
    return (timerMask & ~kSupportedSlaveTimers) == 0U;
}

uint32_t buildOutputMask(const uint32_t timerMask, const bool highSide)
{
    uint32_t outputMask = 0U;

    for (const auto& descriptor : kTimerDescriptors)
    {
        if ((timerMask & descriptor.timerId) != 0U)
        {
            outputMask |= highSide ? descriptor.highSideOutput : descriptor.lowSideOutput;
        }
    }

    return outputMask;
}

void stopConfiguredTimers(const uint32_t timerMask)
{
    for (const auto& descriptor : kTimerDescriptors)
    {
        if ((timerMask & descriptor.timerId) != 0U)
        {
            HAL_HRTIM_WaveformCountStop(&hhrtim1, descriptor.timerId);
        }
    }
}

void startConfiguredTimers(const uint32_t timerMask)
{
    for (const auto& descriptor : kTimerDescriptors)
    {
        if ((timerMask & descriptor.timerId) != 0U)
        {
            HAL_HRTIM_WaveformCountStart(&hhrtim1, descriptor.timerId);
        }
    }
}

void updateTimerPeriodAndDuty(const TimerDescriptor& descriptor, const uint16_t period)
{
    hhrtim1.Instance->sTimerxRegs[descriptor.timerIndex].PERxR = period;
    hhrtim1.Instance->sTimerxRegs[descriptor.timerIndex].CMP1xR = 0U;
    hhrtim1.Instance->sTimerxRegs[descriptor.timerIndex].CMP3xR = (period / 2U) + 1U;
}

void applyPeriodToConfiguredTimers(const uint16_t period, const uint32_t timerMask)
{
    // 主定时器与所有已启用从定时器共用同一周期基准，
    // 这样 ADC 触发、移相和 PWM 更新才能保持一致时序。
    hhrtim1.Instance->sMasterRegs.MPER = period;
    hhrtim1.Instance->sMasterRegs.MCMP2R = period / 2U;

    for (const auto& descriptor : kTimerDescriptors)
    {
        if ((timerMask & descriptor.timerId) != 0U)
        {
            updateTimerPeriodAndDuty(descriptor, period);
        }
    }
}

void setMasterRepetitionInterruptEnabled(const bool enable)
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

HRTIMStatus hrtimStatus = {
    false,
    false,
    false,
    kDefaultPeriod,
    0.0f,
    0U,
    0U,
    0U,
};

bool configure(const Config& config)
{
    if (!isSupportedTimerMask(config.slaveTimerMask) || (config.period == 0U))
    {
        return false;
    }

    const bool wasTimerEnabled = hrtimStatus.timerEnabled;
    const uint32_t previousTimerMask = hrtimStatus.slaveTimerMask;
    const uint32_t nextHighSideOutputMask = buildOutputMask(config.slaveTimerMask, true);
    const uint32_t nextLowSideOutputMask = buildOutputMask(config.slaveTimerMask, false);

    if (wasTimerEnabled)
    {
        // 先回到一个干净的运行态再切配置。下面可以按新配置重启计数器，
        // 但输出保持关闭，由调用方自行决定何时真正上电。
        disableOutputs();
        HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_MASTER);
        stopConfiguredTimers(previousTimerMask);
        setMasterRepetitionInterruptEnabled(false);
        hrtimStatus.timerEnabled = false;
    }
    else if (hrtimStatus.outputEnabled)
    {
        disableOutputs();
    }

    hrtimStatus.slaveTimerMask = config.slaveTimerMask;
    hrtimStatus.highSideOutputMask = nextHighSideOutputMask;
    hrtimStatus.lowSideOutputMask = nextLowSideOutputMask;
    hrtimStatus.masterRepetitionInterruptEnabled = config.enableMasterRepetitionInterrupt;

    setPeriod(config.period);
    setPhase(config.phase);

    if (wasTimerEnabled)
    {
        startTimer();
    }

    return true;
}

void setPeriod(const uint16_t period)
{
    if (period == 0U)
    {
        return;
    }

    hrtimStatus.period = period;
    applyPeriodToConfiguredTimers(period, hrtimStatus.slaveTimerMask);
    setPhase(hrtimStatus.phase);
}

void setPhase(const float phase)
{
    hrtimStatus.phase = clampPhase(phase);

    const uint32_t period = hrtimStatus.period;
    if (period == 0U)
    {
        return;
    }

    // 当前底层 HRTIM 配置是通过主定时器比较事件去复位从定时器，
    // 所以调整 MCMP1/MCMP3 就能改变相对相位，而不需要逐个改写从定时器复位源。
    const int32_t center = static_cast<int32_t>(period / 4U);
    const int32_t offset =
        static_cast<int32_t>(static_cast<float>(period) * hrtimStatus.phase * 0.5f);

    int32_t cmp1 = center - offset;
    int32_t cmp3 = center + offset;
    const int32_t maxCompare = static_cast<int32_t>(period - 1U);

    if (cmp1 < 0)
    {
        cmp1 = 0;
    }
    if (cmp3 > maxCompare)
    {
        cmp3 = maxCompare;
    }

    hhrtim1.Instance->sMasterRegs.MCMP1R = static_cast<uint32_t>(cmp1);
    hhrtim1.Instance->sMasterRegs.MCMP3R = static_cast<uint32_t>(cmp3);
}

void startTimer()
{
    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER);
    startConfiguredTimers(hrtimStatus.slaveTimerMask);
    setMasterRepetitionInterruptEnabled(hrtimStatus.masterRepetitionInterruptEnabled);
    hrtimStatus.timerEnabled = true;
}

void stopTimer()
{
    disableOutputs();
    HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_MASTER);
    stopConfiguredTimers(hrtimStatus.slaveTimerMask);
    setMasterRepetitionInterruptEnabled(false);
    hrtimStatus.timerEnabled = false;
}

void enableLowSideOutputs()
{
    if (hrtimStatus.lowSideOutputMask == 0U)
    {
        return;
    }

    HAL_HRTIM_WaveformOutputStart(&hhrtim1, hrtimStatus.lowSideOutputMask);
    hrtimStatus.outputEnabled = true;
}

void enableHighSideOutputs()
{
    if (hrtimStatus.highSideOutputMask == 0U)
    {
        return;
    }

    HAL_HRTIM_WaveformOutputStart(&hhrtim1, hrtimStatus.highSideOutputMask);
    hrtimStatus.outputEnabled = true;
}

void disableOutputs()
{
    // 按当前配置一次性关闭所有输出，调用方不需要额外记住当前启用了
    // 哪些高边/低边通道。
    const uint32_t outputMask =
        hrtimStatus.highSideOutputMask | hrtimStatus.lowSideOutputMask;

    if (outputMask != 0U)
    {
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, outputMask);
    }

    hrtimStatus.outputEnabled = false;
}

} // namespace HRTIM
#endif
