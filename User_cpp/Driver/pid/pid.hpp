#pragma once

#include <cstdint>

enum PID_MODE
{
    PID_POSITION,  // 位置式 PID：输出由当前误差、积分、微分直接合成
    PID_DELTA      // 增量式 PID：每次只在上次输出基础上叠加增量
};

// 兼容旧 C 版接口的 PID 数据结构，半桥电压环/电流环仍直接读写 ref/output。
struct PID
{
    PID_MODE PID_Mode = PID_POSITION;
    float KP = 0.0f;          // 比例系数
    float KI = 0.0f;          // 积分系数
    float KD = 0.0f;          // 微分系数
    float error[3] = {};      // error[0] 当前误差，error[1]/[2] 为历史误差
    float error_sum = 0.0f;   // 位置式 PID 的积分累加量
    float error_max = 0.0f;   // 积分限幅，防止积分饱和
    float fdb = 0.0f;         // 本次反馈值
    float ref = 0.0f;         // 目标值
    float last_ref = 0.0f;    // 上一次参与计算的目标值，用于步进输入
    float output = 0.0f;      // PID 输出
    float outputmax = 0.0f;   // 输出上限
    float outputmin = 0.0f;   // 输出下限
    float feedforward = 0.0f; // 前馈量，直接叠加到 PID 输出
    float stepin = 0.0f;      // 目标值单次最大变化量，0 表示不启用
};

void pid_calculate(PID* pid, float fdb);
void pid_init(PID* pid, PID_MODE pidMode, float kp, float ki, float kd, float errorMax, float outputMax, float outputMin);
void pid_reset(PID* pid);
void step_in(PID* pid);
void pid_setfeedforward(PID* pid, float feedforward);
void pid_setStepIn(PID* pid, float stepin);
void PID_Clear_Integral(PID* pid);

// Buck 保护环路取更小输出，Boost 保护环路取更大输出。
void Loop_Competition_Buck(uint32_t loop1, uint32_t loop2, uint32_t* outputCompare);
void Loop_Competition_Boost(uint32_t loop1, uint32_t loop2, uint32_t* outputCompare);

// 扰动观察法 MPPT，返回本次占空比扰动方向和步长。
float mppt_calculate(float vol, float cur);
