/**
 * @file DelayedTrigger.h
 * @brief C语言实现的延迟触发器
 * @author Baoqi (zzhongas@connect.ust.hk)
 * @copyright Copyright (c) 2025
 */

#ifndef DELAYEDTRIGGER_H
#define DELAYEDTRIGGER_H

#include <stdint.h>

typedef struct
{
    uint32_t timeout; // 延迟阈值（毫秒）
    uint8_t triggerStatus; // 当前触发状态
    uint32_t triggerCounter; // 累积计数器
    uint32_t increasingSpeed; // 状态为真时的增加步长
    uint32_t decreasingSpeed; // 状态为假时的减少步长
} DelayedTrigger_t;

/**
 * @brief 初始化延迟触发器（完整参数）
 * @param dt 触发器对象指针
 * @param timeout 延迟阈值
 * @param increasingSpeed 状态真时的计数器增量
 * @param decreasingSpeed 状态假时的计数器减量
 */
void DelayedTrigger_Init(DelayedTrigger_t *dt, uint32_t timeout, uint32_t increasingSpeed, uint32_t decreasingSpeed);

/**
 * @brief 初始化延迟触发器（仅指定超时，步长默认为1）
 * @param dt 触发器对象指针
 * @param timeout 延迟阈值
 */
void DelayedTrigger_InitWithTimeout(DelayedTrigger_t *dt, uint32_t timeout);

/**
 * @brief 更新触发器状态
 * @param dt 触发器对象指针
 * @param currentStatus 当前输入状态（非0表示有效）
 * @return 当前触发状态（0/1）
 */
uint8_t DelayedTrigger_Update(DelayedTrigger_t *dt, uint8_t currentStatus);

/**
 * @brief 重置触发器
 * @param dt 触发器对象指针
 */
void DelayedTrigger_Reset(DelayedTrigger_t *dt);

#endif /* DELAYEDTRIGGER_H */
