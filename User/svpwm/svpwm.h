#ifndef SVPWM_H
#define SVPWM_H
#include "stdint.h"
#include "arm_math.h"

typedef struct spwm
{
    float target_freq;
    float carrier_freq;
    float deltaTheta;

    float Ua;
    float Ub; // Vref in (a,b)

    float theta;      // angle between Uref and alpha axis
    float Uref;       // max value =must be less than 0.577 , because Udc is set as 1V.
    //视Udc为1，保证最大Uref为0.577
    // float deltaTheta; // step angle value. every TIM update IT event plus it to theta
    uint8_t area; 
    float duty_a;
    float duty_b;
    float duty_c;
} svpwm_t;
void svpwm_init(svpwm_t *spwm, float Uref, float target_freq_, float carrier_freq_);
void judge_area(svpwm_t *spwm);
void change_freq(svpwm_t *spwm, float freq);
void calcu_UaUb(svpwm_t *spwm);
void Cacu_Time(svpwm_t *svpwm_v) ;
void CacuPWMDuty(svpwm_t *svpwm_v);
void svpwm_calculate(svpwm_t *spwm);
#endif // !SVPWM_H

