#include <stdint.h>
#include "hrtim.h"
#include "main.h"
#include "pid.h"
#include "delaytrigger.h"
#define HalfBridge_HRTIM &hhrtim1
#define HalfBridge_TIMER_ID HRTIM_TIMERID_TIMER_B
#define HalfBridge_TIMER_IDDEX HRTIM_TIMERINDEX_TIMER_B
#define HalfBridge_OUTPUT_CHANNEL (HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2)
#define HalfBridge_PREIOD (27200)
#define HalfBridge_V_KP 0.01f
#define HalfBridge_V_KI 0.0003f
#define HalfBridge_I_KP 0.02f
#define HalfBridge_I_KI 0.0003f

#define IN_V_COEF(x) (30.814 * x - 0.0664) // y =30.814x - 0.0664

#define OUT_V_COEF(x) (10.919f * x - 0.0731) // y = 10.919x - 0.0731

#define HalfBridge_I_COEF(x) (9.0909f * x - 1.8094) // y = 9.2015x - 1.8094
typedef enum
{
    SoftStart_bit = 0x0000001, // 软起动 000000001
    Charge_bit = 0x0000002, // 充电   000000010
    Enable_bit = 0x0000004, // 使能   000000100
    Decharge_bit = 0x0000008, // 放电   000001000
    CAN_Offline_bit = 0x0000010, //  CAN离线检测 000010000
    UVP_Bat_bit = 0x0000020, // 电池欠压保护 000100000
    UVP_Cap_bit = 0x0000040, // 电容组过放保护 001000000
    OTP_MOS_bit = 0x0000080, // 控制板温度保护 010000000
    OTP_CAP_bit = 0x0000100, // 电容组过温保护 100000000
    OCP_bit = 0x0000200, // 板过流保护  000200000
    OVP_Cap_bit = 0x0000400, // 电容组过压保护 0100000000
    OVP_Bat_bit = 0x0000800, // 板过压保护 1000000000
} Power_state_e;
typedef struct Power_state_bit // 状态位,抄桂工的
{
    uint8_t SoftStart_bit; // 超级电容软起动
    uint8_t Charge_bit; // 超级电容充电。1充电，0放电
    uint8_t Enable_bit; // 超级电容给使能。1使能，0失能
    uint8_t Decharge_bit;
    // uint8_t CAN_Offline_bit; // 控制板CAN离线检测。1离线，0在线
    uint8_t UVP_Bat_bit; // 电池欠压保护。1欠压，0正常
    uint8_t UVP_Cap_bit; // 电容组过放保护。1过放，0正常
    // uint8_t OTP_MOS_bit; // 控制板温度保护。1过温，0正常
    uint8_t OTP_CAP_bit; // 电容组过温保护。1过温，0正常
    uint8_t OCP_bit; // 控制板过流保护。1过流，0正常
    uint8_t OVP_Cap_bit; // 电容组过压保护。1过压，0正常
    uint8_t OVP_Bat_bit; // 控制板过压保护。1过压，0正常

} Power_state_bit_t;
typedef struct HalfBridge_ctrl
{
    // float voltage_ref;
    // float current_ref;
    float current_feed;
    float voltage_cap_feed;
    float voltage_bat_feed;
    Power_state_bit_t state;
    // uint32_t state_now;
    float duty;
    float Max_duty;
    float Min_duty;
    float Max_step;
    int16_t POWER_ON;
    /* 保护位延时触发器：避免采样瞬态抖动导致半桥反复启停。 */
    DelayedTrigger_t uvp_bat_trigger;
    DelayedTrigger_t uvp_cap_trigger;
    DelayedTrigger_t ocp_trigger;
    DelayedTrigger_t ovp_bat_trigger;
    DelayedTrigger_t ovp_cap_trigger;
    PID PID_buck_V;
    PID PID_boost_V;
    PID PID_buck_I;
    PID PID_boost_I;
} HalfBridge_ctrl_t;
Power_state_bit_t *state_judge(HalfBridge_ctrl_t *instance);
uint8_t HalfBridge_start(HalfBridge_ctrl_t *instance);
void HalfBridge_stop(HalfBridge_ctrl_t *instance);
void HalfBridge_init(HalfBridge_ctrl_t *instance);
void HalfBridge_buck_cal(HalfBridge_ctrl_t *HalfBridge_ctrl,
                         float feedback_voltage,
                         float feedback_current,
                         float target_voltage);
void HalfBridge_boost_cal(HalfBridge_ctrl_t *HalfBridge_ctrl,
                          float feedback_voltage,
                          float feedback_current,
                          float target_voltage);
void HalfBridge_set_duty(HalfBridge_ctrl_t *instance, float duty);
void HB_PowerLoop(HalfBridge_ctrl_t *instance);
float DutyAdjust(HalfBridge_ctrl_t *instance);
