#include "delaytrigger.hpp"

namespace Driver
{

void DelayedTrigger::init(const uint32_t timeout, const uint32_t increasingSpeed, const uint32_t decreasingSpeed)
{
    timeout_ = timeout;
    increasingSpeed_ = (increasingSpeed == 0U) ? 1U : increasingSpeed;
    decreasingSpeed_ = (decreasingSpeed == 0U) ? 1U : decreasingSpeed;
    reset();
}

bool DelayedTrigger::update(const bool currentStatus)
{
    if (currentStatus)
    {
        // 有效输入逐步累加，计数到达 timeout 后才真正置位。
        if (counter_ < timeout_)
        {
            const uint32_t nextCounter = counter_ + increasingSpeed_;
            counter_ = (nextCounter > timeout_) ? timeout_ : nextCounter;
        }
        else
        {
            triggered_ = true;
        }
    }
    else
    {
        // 无效输入逐步释放，避免采样抖动让故障位立即清零。
        if (counter_ > decreasingSpeed_)
        {
            counter_ -= decreasingSpeed_;
        }
        else
        {
            counter_ = 0U;
            triggered_ = false;
        }
    }

    return triggered_;
}

void DelayedTrigger::reset()
{
    counter_ = 0U;
    triggered_ = false;
}

} // namespace Driver
