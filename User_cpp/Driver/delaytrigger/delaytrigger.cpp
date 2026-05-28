#include "delaytrigger.hpp"
#include "stm32g4xx.h"

namespace Driver
{

namespace
{

void ensureDwtEnabled()
{
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U)
    {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t msToCycles(const uint32_t ms)
{
    return static_cast<uint32_t>((static_cast<uint64_t>(ms) * SystemCoreClock) / 1000U);
}

} // namespace

void DelayedTrigger::init(const uint32_t timeoutMs, const uint32_t releaseMs)
{
    ensureDwtEnabled();
    timeoutCycles_ = msToCycles(timeoutMs);
    releaseCycles_ = (releaseMs == 0U) ? timeoutCycles_ : msToCycles(releaseMs);
    reset();
}

bool DelayedTrigger::update(const bool currentStatus)
{
    const uint32_t now = DWT->CYCCNT;

    if (currentStatus)
    {
        // 输入有效：按 DWT 周期累加，到达 timeout 后置位。
        if (now != lastTick_)
        {
            accumulator_ += (now - lastTick_);
            lastTick_ = now;
        }

        if (accumulator_ >= timeoutCycles_)
        {
            accumulator_ = timeoutCycles_;
            triggered_ = true;
        }
    }
    else
    {
        // 输入无效：按 DWT 周期递减，归零后释放。
        if (now != lastTick_)
        {
            const uint32_t elapsed = now - lastTick_;
            lastTick_ = now;

            if (accumulator_ > elapsed)
            {
                accumulator_ -= elapsed;
            }
            else
            {
                accumulator_ = 0U;
            }
        }

        if (accumulator_ < releaseCycles_)
        {
            triggered_ = false;
        }
    }

    return triggered_;
}

void DelayedTrigger::reset()
{
    accumulator_ = 0U;
    triggered_ = false;
    lastTick_ = DWT->CYCCNT;
}

} // namespace Driver
