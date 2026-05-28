#pragma once

#include <cstdint>

namespace Driver
{

// 软件延时触发器：基于 DWT 周期计数器消抖，不依赖 SysTick 中断。
// timeoutMs: 输入连续有效多少 ms 后才置位
// releaseMs: 输入无效多少 ms 后才清零（0 表示与 timeoutMs 相同）
class DelayedTrigger
{
public:
    void init(uint32_t timeoutMs, uint32_t releaseMs = 0U);
    bool update(bool currentStatus);
    void reset();

private:
    uint32_t timeoutCycles_ = 0U;
    uint32_t releaseCycles_ = 0U;
    uint32_t lastTick_ = 0U;
    uint32_t accumulator_ = 0U;
    bool triggered_ = false;
};

} // namespace Driver
