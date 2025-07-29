#ifndef PID_H_
#define PID_H_

#include <stdint.h>
typedef enum pid_mode
{
    PID_POSITION,
    PID_DELTA
} PID_MODE;
typedef struct pid_t
{
    PID_MODE PID_Mode;
    float KP;
    float KI;
    float KD;
    float error[3];
    float error_sum;
    float error_max;
    float fdb;
    float ref;
    float last_ref; // 上次反馈值
    float output;
    float outputmax;
    float outputmin;
    float feedforward;
    float stepin; // 步进输入
} PID;

void pid_calculate(PID *pid, float fdb);
void pid_init(PID *pid, PID_MODE PID_Mode, float KP, float KI, float KD, float error_max, float outputmax, float outputmin);
void pid_reset(PID *pid);
void step_in(PID *pid);
void pid_setfeedforward(PID *pid, float feedforward);
void pid_setStepIn(PID *pid, float stepin);
void Loop_Competition_Buck(uint32_t Loop1, uint32_t Loop2, uint32_t *OutputCompare);
void PID_Clear_Integral(PID *pid);
void Loop_Competition_Boost(uint32_t Loop1, uint32_t Loop2, uint32_t *OutputCompare);
float mppt_calculate(float vol, float cur);
#endif
