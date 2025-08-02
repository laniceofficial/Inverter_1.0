#include "svpwm.h"
#define PI2 6.2831853f
#define Sqrt3 1.7320508075688772935
#define MAX_Duty 0.98f
#define MIN_Duty 0.02f
// 当调制比接近或超过极限值（约 0.577）时，零矢量时间Dz可能会变成负数，此时就需要进行过调制处理。
// Udc > Uref/0.577
void svpwm_init(svpwm_t *spwm, float Uref, float target_freq_, float carrier_freq_)
{
    spwm->target_freq = target_freq_;
    spwm->carrier_freq = carrier_freq_;
    spwm->deltaTheta = PI2 * target_freq_ / carrier_freq_;
}
void change_freq(svpwm_t *spwm, float freq)
{
    freq = (freq > 100) ? 100 : (freq < 20 ? 20 : freq);
    spwm->target_freq = freq;
    spwm->deltaTheta = PI2 * freq / spwm->carrier_freq;
}
inline void calcu_UaUb(svpwm_t *spwm) // 1
{
    spwm->theta += spwm->deltaTheta;
    if (spwm->theta >= PI2)
    {
        spwm->theta -= PI2;
    }
    spwm->Ua = spwm->Uref * arm_cos_f32(spwm->theta);
    spwm->Ub = spwm->Uref * arm_sin_f32(spwm->theta);
}
inline void judge_area(svpwm_t *spwm) // 2
{
    char A, B, C, T;
    A = (spwm->Ub) > 0 ? 1 : 0;
    B = (Sqrt3 * spwm->Ua - spwm->Ub) > 0 ? 1 : 0;
    C = (-Sqrt3 * spwm->Ua - spwm->Ub) > 0 ? 1 : 0;
    T = A + 2 * B + 4 * C;
    switch (T)
    {
    case 1:
        spwm->area = 2;
        break;
    case 2:
        spwm->area = 6;
        break;
    case 3:
        spwm->area = 1;
        break;
    case 4:
        spwm->area = 4;
        break;
    case 5:
        spwm->area = 3;
        break;
    case 6:
        spwm->area = 5;
        break;
    default:
        spwm->area = 1;
        break;
    }
}

inline void Cacu_Time(svpwm_t *svpwm_v) // 3
{
    float x, y, z;
    // 你看我x y z的表达式里边已经把开关周期除了，实际上这是持续时间占开关周期的比例
    x = Sqrt3 * svpwm_v->Ub;
    y = (3 * svpwm_v->Ua + Sqrt3 * svpwm_v->Ub) / 2;
    z = (-3 * svpwm_v->Ua + Sqrt3 * svpwm_v->Ub) / 2;
    static float Dx = 0, Dy = 0, Dz = 0; // Tz是零矢量持续时间比
    switch (svpwm_v->area)
    {
    case 1:
        Dx = -z;
        Dy = x;
        break;
    case 2:
        Dx = y;
        Dy = z;
        break;
    case 3:
        Dx = x;
        Dy = -y;
        break;
    case 4:
        Dx = z;
        Dy = -x;
        break;
    case 5:
        Dx = -y;
        Dy = -z;
        break;
    case 6:
        Dx = -x;
        Dy = y;
        break;
    default:
        Dx = -z;
        Dy = x;
        break;
    }
    Dz = 1 - (Dx + Dy);    // z是零矢量持续时间比T0
    switch (svpwm_v->area) // 4
    {
    case 1:
        svpwm_v->duty_a = Dx + Dy + Dz / 2;
        svpwm_v->duty_b = Dy + Dz / 2;
        svpwm_v->duty_c = Dz / 2;
        break;
    case 2:
        svpwm_v->duty_a = Dx + Dz / 2;
        svpwm_v->duty_b = Dx + Dy + Dz / 2;
        svpwm_v->duty_c = Dz / 2;
        break;
    case 3:
        svpwm_v->duty_a = Dz / 2;
        svpwm_v->duty_b = Dx + Dy + Dz / 2;
        svpwm_v->duty_c = Dy + Dz / 2;
        break;
    case 4:
        svpwm_v->duty_a = Dz / 2;
        svpwm_v->duty_b = Dx + Dz / 2;
        svpwm_v->duty_c = Dx + Dy + Dz / 2;
        break;
    case 5:
        svpwm_v->duty_a = Dy + Dz / 2;
        svpwm_v->duty_b = Dz / 2;
        svpwm_v->duty_c = Dx + Dy + Dz / 2;
        break;
    case 6:
        svpwm_v->duty_a = Dx + Dy + Dz / 2;
        svpwm_v->duty_b = Dz / 2;
        svpwm_v->duty_c = Dx + Dz / 2;
        break;
    default:
        svpwm_v->duty_a = Dx + Dy + Dz / 2;
        svpwm_v->duty_b = Dy + Dz / 2;
        svpwm_v->duty_c = Dz / 2;
        break;
    }
    // 限制占空比在0~1
    svpwm_v->duty_a = (svpwm_v->duty_a > MAX_Duty) ? MAX_Duty : (svpwm_v->duty_a < MIN_Duty ? MIN_Duty : svpwm_v->duty_a);
    svpwm_v->duty_b = (svpwm_v->duty_b > MAX_Duty) ? MAX_Duty : (svpwm_v->duty_b < MIN_Duty ? MIN_Duty : svpwm_v->duty_b);
    svpwm_v->duty_c = (svpwm_v->duty_c > MAX_Duty) ? MAX_Duty : (svpwm_v->duty_c < MIN_Duty ? MIN_Duty : svpwm_v->duty_c);
}

inline void svpwm_calculate(svpwm_t *spwm)
{
    calcu_UaUb(spwm);
    judge_area(spwm);
    Cacu_Time(spwm);
}