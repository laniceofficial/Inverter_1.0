/**
 * @file DelayedTrigger.c
 * @brief C语言实现的延迟触发器
 * @author Baoqi (zzhongas@connect.ust.hk)
 * @copyright Copyright (c) 2025
 */

#include "delaytrigger.h"

void DelayedTrigger_Init(DelayedTrigger_t *dt, uint32_t timeout, uint32_t increasingSpeed, uint32_t decreasingSpeed)
{
    dt->timeout = timeout;
    dt->increasingSpeed = increasingSpeed;
    dt->decreasingSpeed = decreasingSpeed;
    dt->triggerStatus = 0;
    dt->triggerCounter = 0;
}

void DelayedTrigger_InitWithTimeout(DelayedTrigger_t *dt, uint32_t timeout)
{
    DelayedTrigger_Init(dt, timeout, 1, 1);
}

uint8_t DelayedTrigger_Update(DelayedTrigger_t *dt, uint8_t currentStatus)
{
    if (currentStatus)
    {
        if (dt->triggerCounter < dt->timeout)
        {
            dt->triggerCounter += dt->increasingSpeed;
        }
        else
        {
            dt->triggerStatus = 1; // 触发
        }
    }
    else
    {
        if (dt->triggerCounter)
        {
            dt->triggerCounter -= dt->decreasingSpeed;
        }
        else
        {
            dt->triggerStatus = 0; // 未触发
        }
    }
    return dt->triggerStatus;
}

void DelayedTrigger_Reset(DelayedTrigger_t *dt)
{
    dt->triggerStatus = 0;
    dt->triggerCounter = 0;
}
