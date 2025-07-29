#include "pr.h"
void QPR_Init(QPRController *ctrl,
              float Kp,
              float Kr,
              float w0,
              float wc,
              float Ts)
{

    ctrl->Kp = Kp;
    ctrl->Kr = Kr;
    ctrl->w0 = w0;
    ctrl->wc = wc;
    ctrl->Ts = Ts;

    // 计算中间变量
    float a = 2.0f / Ts;
    float a_sq = a * a;
    float w0_sq = w0 * w0;
    float b0 = 2.0f * Kr * wc * a;

    // 计算分母系数
    float a0 = a_sq + 2.0f * wc * a + w0_sq;
    float a1 = -2.0f * a_sq + 2.0f * w0_sq;
    float a2 = a_sq - 2.0f * wc * a + w0_sq;

    // 归一化系数
    ctrl->b0_prime = b0 / a0;
    ctrl->b2_prime = -b0 / a0; // b2 = -b0
    ctrl->a1_prime = a1 / a0;
    ctrl->a2_prime = a2 / a0;

    // 初始化状态
    ctrl->e_prev1 = 0.0f;
    ctrl->e_prev2 = 0.0f;
    ctrl->yr_prev1 = 0.0f;
    ctrl->yr_prev2 = 0.0f;
}

// 执行控制计算
float QPR_Update(QPRController *ctrl, float ref, float fdb)
{

    // 计算当前误差
    float e = ref - fdb;
    // 计算谐振部分输出
    float yr = ctrl->b0_prime * e + ctrl->b2_prime * ctrl->e_prev2 - ctrl->a1_prime * ctrl->yr_prev1 - ctrl->a2_prime * ctrl->yr_prev2;

    // 比例部分 + 谐振部
    float u = ctrl->Kp * e + yr;

    // 更新状态变量
    ctrl->e_prev2 = ctrl->e_prev1;
    ctrl->e_prev1 = e;
    ctrl->yr_prev2 = ctrl->yr_prev1;
    ctrl->yr_prev1 = yr;

    return u;
}