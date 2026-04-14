#ifndef TASK_H
#define TASK_H

#ifdef __cplusplus
extern "C"
{
#endif
#define PI 3.14159f
#define user_abs(x) ((x) > (0) ? (x) : (-(x)))
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
#ifdef __cplusplus
}
#endif

#endif // !task
