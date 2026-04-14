/**
 * @file HRTIMWrapper.hpp
 * @author Xian Ziming (zxianaa@connect.ust.hk); Baoqi (zzhongas@connect.ust.hk);
 *
 * @copyright Copyright (c) 2025
 */

#pragma once

#include "main.h"

// #include <cstdint>

#if defined(HAL_HRTIM_MODULE_ENABLED)
namespace BSP
{
struct Config
{
    // 包装层只接受已经在 hrtim.c 中完成底层初始化的从定时器。
    uint32_t slaveTimerMask = 0U;
    // 将主定时器重复中断的开关统一收口到包装层，确保 start/stop
    // 始终按同一套运行时策略生效。
    bool enableMasterRepetitionInterrupt = false;
    // 配置的周期作为统一基准，同时驱动主定时器比较时序和所有已启用从定时器。
    uint16_t period = 54400U;
    // 归一化移相输入。实现中会在写入主定时器比较寄存器前先做钳位，
    // 避免调用方传入越界值后生成非法比较值。
    float phase = 0.0f;
};

struct HRTIMStatus
{
    bool timerEnabled = false;
    bool outputEnabled = false;
    bool masterRepetitionInterruptEnabled = false;
    uint16_t period = 54400U;
    float phase = 0.0f;
    uint32_t slaveTimerMask = 0U;
    uint32_t highSideOutputMask = 0U;
    uint32_t lowSideOutputMask = 0U;
};

extern HRTIMStatus hrtimStatus;

// 更新运行时配置，但不会顺带打开输出。调用方应先明确哪些从定时器参与工作，
// 再按需要显式启动计数器和输出，保证上电顺序可控。
bool configure(const Config& config);
void setPeriod(uint16_t period);
void setPhase(float phase);
void startTimer();
void stopTimer();
void enableLowSideOutputs();
void enableHighSideOutputs();
void disableOutputs();

} // namespace HRTIM

#endif
