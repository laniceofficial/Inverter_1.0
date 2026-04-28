#include "halfbridge_ctrl.h"
#include <stdint.h>
#include <string.h>
#include "pid.h"
#include "stm32g4xx_hal_def.h"
#include "delaytrigger.h"
// static HalfBridge_ctrl_t *instance = NULL;

#define HALFBRIDGE_24KTO50MS_TICKS 1200U // 24kto 50ms
#define HALFBRIDGE_24KTO25MS_TICKS 600U // 24kto 25ms
#define HALFBRIDGE_ADC4_SCAN_TICKS 7107U //adc采样时间
#define HALFBRIDGE_ADC_GUARD_TICKS 1000U

static uint8_t HalfBridge_UpdateFaultBit(DelayedTrigger_t *trigger, uint8_t currentFault);
static uint32_t HalfBridge_CalcAdcTriggerCmp4(uint32_t cmp3);

void HalfBridge_init(HalfBridge_ctrl_t *HalfBridge_ctrl)
{
    HalfBridge_ctrl->Max_duty = 0.85f;
    HalfBridge_ctrl->Min_duty = 0.05f;
    HalfBridge_ctrl->duty = 0.2f;
    // HalfBridge_ctrl->voltage_ref = 0.0f;
    // HalfBridge_ctrl->current_ref = 0.0f;
    HalfBridge_ctrl->POWER_ON = 0;
    HalfBridge_ctrl->Max_step = 0.02f;
    pid_init(&HalfBridge_ctrl->PID_buck_V, PID_DELTA, HalfBridge_V_KP, HalfBridge_V_KI, 0, 0.5, 3, 0);
    pid_init(&HalfBridge_ctrl->PID_buck_I,
             PID_DELTA,
             HalfBridge_I_KP,
             HalfBridge_I_KI,
             0,
             0.1,
             HalfBridge_ctrl->Max_duty,
             HalfBridge_ctrl->Min_duty);
    DelayedTrigger_InitWithTimeout(&HalfBridge_ctrl->uvp_bat_trigger, HALFBRIDGE_24KTO25MS_TICKS);
    DelayedTrigger_InitWithTimeout(&HalfBridge_ctrl->uvp_cap_trigger, HALFBRIDGE_24KTO25MS_TICKS);
    DelayedTrigger_InitWithTimeout(&HalfBridge_ctrl->ocp_trigger, HALFBRIDGE_24KTO50MS_TICKS);
    DelayedTrigger_InitWithTimeout(&HalfBridge_ctrl->ovp_bat_trigger, HALFBRIDGE_24KTO25MS_TICKS);
    DelayedTrigger_InitWithTimeout(&HalfBridge_ctrl->ovp_cap_trigger, HALFBRIDGE_24KTO25MS_TICKS);
    HAL_HRTIM_WaveformCounterStart(HalfBridge_HRTIM, HalfBridge_TIMER_ID);
    HalfBridge_ctrl->state.Enable_bit = 1;
    // instance = HalfBridge_ctrl;
}
uint8_t HalfBridge_start(HalfBridge_ctrl_t *instance)
{
    // 做缓启动
    if (instance->POWER_ON-- <= -2000)
    {
        instance->POWER_ON = 1;
        HalfBridge_set_duty(instance, DutyAdjust(instance));
        HAL_HRTIM_WaveformOutputStart(HalfBridge_HRTIM, HalfBridge_OUTPUT_CHANNEL);
        return 1;
    }
    return 0;
}
void HalfBridge_stop(HalfBridge_ctrl_t *instance)
{
    // HAL_HRTIM_WaveformCounterStop(HalfBridge_HRTIM, HalfBridge_TIMER_ID);
    HAL_HRTIM_WaveformOutputStop(HalfBridge_HRTIM, HalfBridge_OUTPUT_CHANNEL);
    pid_reset(&instance->PID_buck_I);
    pid_reset(&instance->PID_buck_V);
    instance->POWER_ON = 0;
}
void HalfBridge_buck_cal(HalfBridge_ctrl_t *HalfBridge_ctrl,
                         float feedback_voltage,
                         float feedback_current,
                         float target_voltage)
{
    HalfBridge_ctrl->PID_buck_V.ref = target_voltage;
    pid_calculate(&HalfBridge_ctrl->PID_buck_V, feedback_voltage);
    HalfBridge_ctrl->PID_buck_I.ref = HalfBridge_ctrl->PID_buck_V.output;
    pid_calculate(&HalfBridge_ctrl->PID_buck_I, feedback_current);
    HalfBridge_set_duty(HalfBridge_ctrl, HalfBridge_ctrl->PID_buck_V.output);
}
void HalfBridge_boost_cal(HalfBridge_ctrl_t *HalfBridge_ctrl,
                          float feedback_voltage,
                          float feedback_current,
                          float target_voltage)
{
    HalfBridge_ctrl->PID_boost_V.ref = target_voltage;
    pid_calculate(&HalfBridge_ctrl->PID_boost_V, feedback_voltage);
    HalfBridge_ctrl->PID_boost_I.ref = HalfBridge_ctrl->PID_boost_V.output;
    pid_calculate(&HalfBridge_ctrl->PID_boost_I, feedback_current);
    HalfBridge_set_duty(HalfBridge_ctrl, HalfBridge_ctrl->PID_boost_I.output);
}
void HalfBridge_set_duty(HalfBridge_ctrl_t *HalfBridge_ctrl, float duty)
{
    uint32_t cmp3;

    // float duty_adjust = DutyAdjust(HalfBridge_ctrl);
    // if (duty_adjust > duty)
    // {

    //     }
    if (duty > HalfBridge_ctrl->Max_duty)
    {
        duty = HalfBridge_ctrl->Max_duty;
    }
    else if (duty < HalfBridge_ctrl->Min_duty)
    {
        duty = HalfBridge_ctrl->Min_duty;
    }
    HalfBridge_ctrl->duty = duty;
    cmp3 = (uint32_t)(HalfBridge_PREIOD * duty);
    __HAL_HRTIM_SETCOMPARE(HalfBridge_HRTIM, HalfBridge_TIMER_IDDEX, HRTIM_COMPAREUNIT_1, 0);
    __HAL_HRTIM_SETCOMPARE(HalfBridge_HRTIM, HalfBridge_TIMER_IDDEX, HRTIM_COMPAREUNIT_3, cmp3);
    __HAL_HRTIM_SETCOMPARE(
        HalfBridge_HRTIM, HalfBridge_TIMER_IDDEX, HRTIM_COMPAREUNIT_4, HalfBridge_CalcAdcTriggerCmp4(cmp3));
    // __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, 0);
    // __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, 27200 * duty);
}
#define BAT_UVP_THRESHOLD 20.3f // 电池欠压阈值
#define CAP_OVP_THRESHOLD 23.0f // 电容组过压阈值
#define BAT_OVP_THRESHOLD 30.0f // 电池过压阈值
#define CURRENT_OCP_THRESHOLD 15.0f // 电流过流阈值
#define CURRENT_REVERSE_THRESHOLD -0.3f
Power_state_bit_t *state_judge(HalfBridge_ctrl_t *instance)
{
    uint8_t uvp_bat_raw;
    uint8_t uvp_cap_raw;
    uint8_t ocp_raw;
    uint8_t ovp_bat_raw;
    uint8_t ovp_cap_raw;


    /* 先生成原始故障位，再通过延时触发器抑制采样抖动。 */
    uvp_bat_raw = (instance->voltage_bat_feed < BAT_UVP_THRESHOLD) ? 1U : 0U;

    /* 电容欠压保护当前仍保持关闭，只预留延时触发接口。 */
    uvp_cap_raw = 0U;

    /* 正向过流或反向过流任一成立都视为过流。 */
    ocp_raw = ((instance->current_feed > CURRENT_OCP_THRESHOLD) || (instance->current_feed < CURRENT_REVERSE_THRESHOLD))
              ? 1U
              : 0U;

    ovp_bat_raw = (instance->voltage_bat_feed > BAT_OVP_THRESHOLD) ? 1U : 0U;

    ovp_cap_raw =
        ((instance->voltage_cap_feed > CAP_OVP_THRESHOLD) || (instance->voltage_bat_feed < instance->voltage_cap_feed))
        ? 1U
        : 0U;

    instance->state.UVP_Bat_bit = HalfBridge_UpdateFaultBit(&instance->uvp_bat_trigger, uvp_bat_raw);
    instance->state.UVP_Cap_bit = HalfBridge_UpdateFaultBit(&instance->uvp_cap_trigger, uvp_cap_raw);
    instance->state.OCP_bit = HalfBridge_UpdateFaultBit(&instance->ocp_trigger, ocp_raw);
    instance->state.OVP_Bat_bit = HalfBridge_UpdateFaultBit(&instance->ovp_bat_trigger, ovp_bat_raw);
    instance->state.OVP_Cap_bit = HalfBridge_UpdateFaultBit(&instance->ovp_cap_trigger, ovp_cap_raw);

    // 根据故障状态更新使能位
    instance->state.Enable_bit =
        !(instance->state.UVP_Bat_bit || instance->state.UVP_Cap_bit || instance->state.OCP_bit ||
          instance->state.OVP_Bat_bit || instance->state.OVP_Cap_bit);


    return &instance->state;
}
float set_ = 2;
void HB_PowerLoop(HalfBridge_ctrl_t *instance) // 此处默认电流电压已经做完采样了
{
    if (instance->POWER_ON != 1)
    {
        HalfBridge_start(instance);
    }
    HalfBridge_set_duty(instance, 0.2);
return;
    if (instance == NULL)
    {
        return;
    }
    Power_state_bit_t *state = state_judge(instance); //&instance->state; //

    if (state->Charge_bit)
    {
        HalfBridge_buck_cal(instance, instance->voltage_bat_feed, instance->current_feed, instance->voltage_bat_feed);
    }
    else if (state->Decharge_bit)
    {
        HalfBridge_boost_cal(instance, instance->voltage_bat_feed, instance->current_feed, instance->voltage_bat_feed);
    }

    else if (state->OCP_bit)
    {
        HalfBridge_stop(instance);
    }
    else if (state->OTP_CAP_bit)
    {
        HalfBridge_stop(instance);
    }
    else if (state->OVP_Bat_bit)
    {
        HalfBridge_stop(instance);
    }
    else if (state->OVP_Cap_bit)
    {
        HalfBridge_stop(instance);
    }
    else if (state->SoftStart_bit)
    {}
    else if (state->UVP_Bat_bit)
    {
        HalfBridge_stop(instance);
    }
    else if (state->Enable_bit)
    {
        if (instance->POWER_ON != 1)
        {
            HalfBridge_start(instance);
        }
        else
        {
            static float delta = 0;
            float duty_ff;
            float target_duty;

            // instance->PID_buck_V.ref = set_;
            // pid_calculate(&instance->PID_buck_V, instance->voltage_cap_feed);
            instance->PID_buck_I.ref = set_;
            pid_calculate(&instance->PID_buck_I, instance->current_feed); 
            duty_ff = DutyAdjust(instance);
            target_duty = instance->PID_buck_I.output;
            if (target_duty < duty_ff)
            {
                target_duty = duty_ff;
                instance->PID_buck_I.output = duty_ff; // 防止增量式 PID 下次又从过低 output 开始
            }
                                                                          // delta = D_target - D_current;
            delta = target_duty - instance->duty;
            if (delta > instance->Max_step)
            {
                instance->duty += instance->Max_step;
            }
            else if (delta < -instance->Max_step)
            {
                instance->duty -= instance->Max_step;
            }
            else
            {
                instance->duty = target_duty;
            }
            //adjust调整占空比，但是会对于越来越大的电压需求，是不是占空比会越来越大？
            HalfBridge_set_duty(instance, instance->duty);
        }
    }
    else
    {
        HalfBridge_stop(instance);
    }
}
float DutyAdjust(HalfBridge_ctrl_t *instance) // 在开启buck时需要根据电容电压调整占空比防止过流
{

    float duty_adjust;

    if (instance == NULL)
    {
        return 0.0f;
    }

    // if (instance->voltage_bat_feed <= 0.0f)
    // {
    //     return instance->Max_duty;
    // }

    duty_adjust = (instance->voltage_cap_feed / instance->voltage_bat_feed)+0.01;
    if (duty_adjust > instance->Max_duty)
    {
        duty_adjust = instance->Max_duty;
    }
    else if (duty_adjust < instance->Min_duty)
    {
        duty_adjust = instance->Min_duty;
    }

    return duty_adjust;
}

static uint8_t HalfBridge_UpdateFaultBit(DelayedTrigger_t *trigger, uint8_t currentFault)
{
    if (trigger == NULL)
    {
        return currentFault;
    }

    return DelayedTrigger_Update(trigger, currentFault);
}

static uint32_t HalfBridge_CalcAdcTriggerCmp4(uint32_t cmp3)
{
    const uint32_t period = HalfBridge_PREIOD;
    const uint32_t on_ticks = cmp3;
    const uint32_t off_ticks = period - cmp3;
    const uint32_t required_window = HALFBRIDGE_ADC4_SCAN_TICKS + (2U * HALFBRIDGE_ADC_GUARD_TICKS);
    uint32_t cmp4;

    /*
     * ADC4 一次触发会顺扫 3 个通道，因此不能只让“触发点”避开开关边沿，
     * 而是要让整段扫描时间都落在导通区或关断区的安静窗口内部。
     */
    if ((off_ticks >= required_window) && ((off_ticks >= on_ticks) || (on_ticks < required_window)))
    {
        cmp4 = cmp3 + HALFBRIDGE_ADC_GUARD_TICKS +
               ((off_ticks - required_window) / 2U);
    }
    else if (on_ticks >= required_window)
    {
        cmp4 = HALFBRIDGE_ADC_GUARD_TICKS +
               ((on_ticks - required_window) / 2U);
    }
    else if (off_ticks >= HALFBRIDGE_ADC4_SCAN_TICKS)
    {
        cmp4 = cmp3 + ((off_ticks - HALFBRIDGE_ADC4_SCAN_TICKS) / 2U);
    }
    else if (on_ticks >= HALFBRIDGE_ADC4_SCAN_TICKS)
    {
        cmp4 = (on_ticks - HALFBRIDGE_ADC4_SCAN_TICKS) / 2U;
    }
    else
    {
        cmp4 = cmp3;
    }

    if (cmp4 >= period)
    {
        cmp4 = period - 1U;
    }

    return cmp4;
}
