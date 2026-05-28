#ifndef USER_CPP_TASK_H
#define USER_CPP_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_TRACE_EVENT_COUNT 256U
#define DEBUG_TRACE_TYPE_ARR 1U
#define DEBUG_TRACE_TYPE_ASK_EDGE 2U
#define DEBUG_TRACE_TYPE_ASK_BIT 3U
#define DEBUG_TRACE_TYPE_ASK_VALID 4U

// 简单环形调试缓冲，用 DWT 周期计数记录关键事件发生时间。
typedef struct
{
    uint32_t seq;      // 递增序号
    uint32_t cyccnt;   // DWT 周期计数
    uint16_t tim6_cnt; // TIM6 当前计数
    uint8_t type;      // DEBUG_TRACE_TYPE_* 事件类型
    uint8_t value;     // 事件附带值
} DebugTraceEvent_t;

void task_init(void);
void debug_trace_init(void);
void debug_trace_log(uint8_t type, uint8_t value);

extern volatile DebugTraceEvent_t debug_trace_events[DEBUG_TRACE_EVENT_COUNT];
extern volatile uint16_t debug_trace_write_index;
extern volatile uint32_t debug_trace_seq;

#ifdef __cplusplus
}
#endif

#endif // USER_CPP_TASK_H
