#pragma once

#include <cstdint>

namespace Driver
{

// 软件延时触发器：输入连续有效达到阈值后置位，输入无效时按下降速度释放。
class DelayedTrigger
{
public:
    void init(uint32_t timeout, uint32_t increasingSpeed = 1U, uint32_t decreasingSpeed = 1U);
    bool update(bool currentStatus);
    void reset();

private:
    uint32_t timeout_ = 0U;         // 触发所需的连续计数阈值
    uint32_t counter_ = 0U;         // 当前累计计数
    uint32_t increasingSpeed_ = 1U; // 输入有效时每次增加的计数
    uint32_t decreasingSpeed_ = 1U; // 输入无效时每次减少的计数
    bool triggered_ = false;        // 当前消抖后的触发状态
};

} // namespace Driver
