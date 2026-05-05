#include "pid.hpp"

namespace
{

float userAbs(const float value)
{
    return (value > 0.0f) ? value : -value;
}

} // namespace

void pid_calculate(PID* pid, const float fdb)
{
    // stepin 用于限制 ref 的突变，避免目标值阶跃导致输出瞬间打满。
    if (pid->stepin != 0.0f)
    {
        step_in(pid);
    }

    pid->fdb = fdb;
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = pid->ref - pid->fdb;

    if (pid->PID_Mode == PID_POSITION)
    {
        const float errorDelta = pid->error[0] - pid->error[1];
        pid->error_sum += pid->KI * pid->error[0];

        // 位置式 PID 需要对积分项限幅，防止长时间误差造成积分饱和。
        if (pid->error_sum > pid->error_max)
        {
            pid->error_sum = pid->error_max;
        }
        else if (pid->error_sum < -pid->error_max)
        {
            pid->error_sum = -pid->error_max;
        }

        pid->output = pid->KP * pid->error[0] + pid->error_sum + pid->KD * errorDelta + pid->feedforward;
    }
    else if (pid->PID_Mode == PID_DELTA)
    {
        // 增量式 PID 在上次 output 基础上累加，适合占空比这类连续调节量。
        pid->output += pid->KP * (pid->error[0] - pid->error[1]) +
                       pid->KD * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]) +
                       pid->KI * pid->error[0] + pid->feedforward;
    }

    pid->last_ref = pid->ref;

    // 输出统一限幅，保护后级 PWM/功率级不会拿到非法占空比。
    if (pid->output > pid->outputmax)
    {
        pid->output = pid->outputmax;
    }
    else if (pid->output < pid->outputmin)
    {
        pid->output = pid->outputmin;
    }
}

void pid_init(PID* pid,
              const PID_MODE pidMode,
              const float kp,
              const float ki,
              const float kd,
              const float errorMax,
              const float outputMax,
              const float outputMin)
{
    pid->PID_Mode = pidMode;
    pid->KP = kp;
    pid->KI = ki;
    pid->KD = kd;
    pid->error_max = errorMax;
    pid->outputmax = outputMax;
    pid->outputmin = outputMin;
    pid->error_sum = 0.0f;
    pid->output = 0.0f;
    pid->fdb = 0.0f;
    pid->ref = 0.0f;
    pid->last_ref = 0.0f;
    pid->feedforward = 0.0f;
    pid->stepin = 0.0f;

    for (float& error : pid->error)
    {
        error = 0.0f;
    }
}

void step_in(PID* pid)
{
    float factor = 0.0f;

    // 目标变化量在 stepin 内时直接接受，超过时按 stepin 缓慢逼近。
    if (userAbs(pid->last_ref - pid->ref) <= pid->stepin)
    {
        return;
    }

    if ((pid->last_ref - pid->ref) > 0.0f)
    {
        factor = -1.0f;
    }
    else if ((pid->last_ref - pid->ref) < 0.0f)
    {
        factor = 1.0f;
    }

    pid->ref = pid->last_ref + factor * pid->stepin;
}

void pid_reset(PID* pid)
{
    pid->error_sum = 0.0f;
    pid->output = 0.0f;

    for (float& error : pid->error)
    {
        error = 0.0f;
    }
}

void PID_Clear_Integral(PID* pid)
{
    pid->error_sum = 0.0f;
    pid->output = 0.0f;
}

void pid_setfeedforward(PID* pid, const float feedforward)
{
    pid->feedforward = feedforward;
}

void pid_setStepIn(PID* pid, const float stepin)
{
    pid->stepin = stepin;
}

void Loop_Competition_Buck(const uint32_t loop1, const uint32_t loop2, uint32_t* outputCompare)
{
    *outputCompare = (loop1 < loop2) ? loop1 : loop2;
}

void Loop_Competition_Boost(const uint32_t loop1, const uint32_t loop2, uint32_t* outputCompare)
{
    *outputCompare = (loop1 > loop2) ? loop1 : loop2;
}

float mppt_calculate(const float vol, const float cur)
{
    static float direction = 1.0f;
    static float powerPre = 0.0f;

    // 扰动观察法：功率上升则沿原方向扰动，功率下降则反向。
    const float powerRef = vol * cur;
    if (powerPre == 0.0f)
    {
        powerPre = powerRef;
        return 0.0f;
    }

    const float deltaPower = powerRef - powerPre;
    powerPre = powerRef;
    direction = (deltaPower > 0.0f) ? 1.0f : -1.0f;

    if (userAbs(deltaPower) < 0.5f)
    {
        return 0.02f * direction;
    }

    return 0.1f * direction;
}
