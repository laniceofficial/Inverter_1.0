#ifndef TASK_H
#define TASK_H

#include "task.h"
#ifdef __cplusplus
extern "C"
{
#endif

    // void task_init();
    // void task_loop();
#define  PI 3.14159f
#define user_abs(x) ((x) > (0) ? (x) : (-(x)))
#define V_KP 0.01f
#define V_KI 0.0003f
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
    void sinTab_genarate();
    void PID_Seyduty();
    void duty_update();
    ChangeState_e GetNowState(void);
    void SetNowState(ChangeState_e state);
#ifdef __cplusplus
}
#endif

#endif // !task
