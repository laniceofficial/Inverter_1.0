#include "pid.h"
// 绝对值
#define user_abs(x) ((x) > (0) ? (x) : (-(x)))

// 数字限幅
#define user_value_limit(val, min, max)        \
    {                                          \
        (val) = (val) < (min) ? (min) : (val); \
        (val) = (val) > (max) ? (max) : (val); \
    }
inline void pid_calculate(PID *pid, float fdb)
{
    if(pid->stepin)
    {
        step_in(pid);
    }
    pid->fdb = fdb;
    pid->error[2] = pid->error[1];       // 上上次误差
    pid->error[1] = pid->error[0];       // 上次误差
    pid->error[0] = pid->ref - pid->fdb; // 本次误差
    if (pid->PID_Mode == PID_POSITION)
    {
        float error_delta = pid->error[0] - pid->error[1];
        pid->error_sum += pid->KI * pid->error[0];
        // 积分上限控制
        if (pid->error_sum > pid->error_max)
            pid->error_sum = pid->error_max;
        else if (pid->error_sum < -pid->error_max)
            pid->error_sum = -pid->error_max;
        pid->output = pid->KP * pid->error[0] + pid->error_sum + pid->KD * error_delta + pid->feedforward;
    }
    else if (pid->PID_Mode == PID_DELTA)
    {
        pid->output += pid->KP * (pid->error[0] - pid->error[1]) +
                       pid->KD * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]) + pid->KI * pid->error[0] + pid->feedforward;
    }
    // 输出上限控制
    pid->last_ref =  pid->ref; // 保存上次反馈值
    if (pid->output > pid->outputmax)
        pid->output = pid->outputmax;
    else if (pid->output < pid->outputmin)
        pid->output = pid->outputmin;
    return;
}

void pid_init(PID *pid, PID_MODE PID_Mode, float KP, float KI, float KD, float error_max, float outputmax, float outputmin)
{
    pid->PID_Mode = PID_Mode;
    pid->KP = KP;
    pid->KI = KI;
    pid->KD = KD;
    pid->error_max = error_max;
    pid->outputmax = outputmax;
    pid->outputmin = outputmin;
    pid->error_sum = 0;
    pid->output = 0;
    pid->fdb = 0;
    pid->ref = 0;
    pid->last_ref = 0; // 初始化上次反馈值
    pid->feedforward = 0;
    pid->stepin = 0; // 初始化步进输入
    for (int i = 0; i < 3; i++)
    {
        pid->error[i] = 0;
    }
    return;
}

inline void step_in(PID *pid)
{
    //需保证last_ref和ref的不同
    float kFactor = 0.0f;
    if (abs(pid->last_ref - pid->ref) <= pid->stepin)
    {
        return;
    }
    else
    {
        if ((pid->last_ref - pid->ref) > 0.0f)
        {
            kFactor = -1.0f;
        }
        else if ((pid->last_ref - pid->ref) < 0.0f)
        {
            kFactor = 1.0f;
        }
        else
        {
            kFactor = 0.0f;
        }
        pid->ref = pid->last_ref + kFactor * pid->stepin;
    }
}

inline void pid_reset(PID *pid)
{
    pid->error_sum = 0;
    pid->output = 0;
    for (int i = 0; i < 3; i++)
    {
        pid->error[i] = 0;
    }
}
inline void PID_Clear_Integral(PID *pid)
{
    pid->error_sum = 0;
    pid->output = 0;
}
 inline void pid_setfeedforward(PID *pid, float feedforward)
{
    pid->feedforward = feedforward;
}
inline void pid_setStepIn(PID *pid, float stepin)
{
    pid->stepin = stepin;
}
// 环路竞争，用于保护CC，CV，CP保护，Buck模式下使用，取小的值作为PWM占空比以达到保护目的
inline void Loop_Competition_Buck(uint32_t Loop1, uint32_t Loop2, uint32_t *OutputCompare)
{
    if (Loop1 < Loop2)
        *OutputCompare = Loop1;
    else
        *OutputCompare = Loop2;
}

// 环路竞争，用于保护CC，CV，CP保护，Boost模式下使用，取大的值作为PWM占空比以达到保护目的
inline void Loop_Competition_Boost(uint32_t Loop1, uint32_t Loop2, uint32_t *OutputCompare)
{
    if (Loop1 > Loop2)
        *OutputCompare = Loop1;
    else
        *OutputCompare = Loop2;
}

// 返回需要是否增加扰动占空比
float mppt_calculate(float vol, float cur)
{
    // static float step = 0.01f; // 初始步长
    // static float last_power = 0.0f;
    static float direction = 1; // 初始方向:增加
    static float power_ref = 0;
    static float power_pre = 0; // 先前的功率
    power_ref = vol * cur;      // 当前功率
    if (power_pre == 0)
    {
        power_pre = power_ref;

        return 0;
    }
    float delta_power = power_ref - power_pre; // 功率变化量
    power_pre = power_ref;                     // 更新先前功率
    direction = (delta_power > 0) ? 1 : -1; // 根据功率变化确定方向
    if (user_abs(delta_power) < 0.5)
    {
        return 0.02f*direction; 
    }
    else
    {
        return 0.1f * direction;
    }
}