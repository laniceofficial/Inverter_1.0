#ifndef TASK_H
#define TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
#define PI 3.14159f
#define user_abs(x) ((x) > (0) ? (x) : (-(x)))
#define DEBUG_TRACE_EVENT_COUNT 256U
#define DEBUG_TRACE_TYPE_ARR 1U
#define DEBUG_TRACE_TYPE_ASK_EDGE 2U
#define DEBUG_TRACE_TYPE_ASK_BIT 3U
#define DEBUG_TRACE_TYPE_ASK_VALID 4U
    typedef struct
    {
        uint32_t seq;
        uint32_t cyccnt;
        uint16_t tim6_cnt;
        uint8_t type;
        uint8_t value;
    } DebugTraceEvent_t;
    typedef enum ChangeState
    {
        LOW_POWER = 0,
        PreDetect = 1,
        ASKDetect,
        PreChange,
        Changing,
    } ChangeState_e;
    void task_init();
    void task_loop();
    void duty_update();
    void task_try_enter_low_power(void);
    ChangeState_e GetNowState(void);
    void SetNowState(ChangeState_e state);
    void debug_trace_init(void);
    void debug_trace_log(uint8_t type, uint8_t value);
    extern volatile DebugTraceEvent_t debug_trace_events[DEBUG_TRACE_EVENT_COUNT];
    extern volatile uint16_t debug_trace_write_index;
    extern volatile uint32_t debug_trace_seq;
#ifdef __cplusplus
}
#endif

#endif // !task
