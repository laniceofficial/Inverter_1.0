

typedef struct cofe
{
    /* data */
    float A1;
    float A2;
    float B0;
    float B1;
    float B2;
    float gain;

} DIS_2ORDER_TF_COEF_DEF;
typedef struct data
{
    float w0;
    float w1;
    float w2;

    float output;
} DIS_2ORDER_TF_DATA_DEF;

// #include "main.h"
// #include "pid.h"
// #include "svpwm.h"
// #include <math.h>

// 系统参数定义
#define SAMPLE_FREQ 100000.0f // 采样频率(Hz)
#define PI 3.1415926535f
#define U_DC_REF 500.0f   // 直流侧目标电压(V)
#define MAX_CURRENT 10.0f // 最大电流限制(A)

// 坐标变换中间变量
typedef struct
{
    float alpha;
    float beta;
} AlphaBeta_t;

typedef struct
{
    float d;
    float q;
} DQ_t;

// SOGI锁相环结构体
typedef struct
{
    float k;     // 增益系数
    float w0;    // 中心角频率
    float x1;    // 状态变量1
    float x2;    // 状态变量2
    float sin;   // 输出正弦波
    float cos;   // 输出余弦波
    float theta; // 估计相位
    float freq;  // 估计频率
} SOGI_PLL_t;

// 全局变量
SOGI_PLL_t sogi_pll;
PID pi_udc, pi_id, pi_iq; // 直流电压环、d轴电流环、q轴电流环
AlphaBeta_t grid_vol_ab, grid_curr_ab;
DQ_t grid_curr_dq, volt_ref_dq;
svpwm_t rect_svpwm;           // SVPWM结构体
float u_dc_fdb = 0.0f;        // 直流侧电压反馈
float grid_vol_abc[3] = {0};  // 电网电压采样
float grid_curr_abc[3] = {0}; // 电网电流采样

// SOGI-PLL初始化
void sogi_pll_init(SOGI_PLL_t *pll, float k, float w0)
{
    pll->k = k;
    pll->w0 = w0;
    pll->x1 = 0.0f;
    pll->x2 = 0.0f;
    pll->sin = 0.0f;
    pll->cos = 1.0f;
    pll->theta = 0.0f;
    pll->freq = 50.0f;
}

// SOGI-PLL更新(输入电网α轴电压)
void sogi_pll_update(SOGI_PLL_t *pll, float u_alpha, float Ts)
{
    // SOGI核心计算
    float x1_new = pll->x1 + Ts * pll->w0 * (u_alpha - pll->x1 - pll->k * pll->x2);
    float x2_new = pll->x2 + Ts * pll->w0 * pll->x1;

    // 相位计算
    pll->sin = x2_new * (pll->w0 / pll->k);
    pll->cos = x1_new;
    pll->theta += Ts * pll->w0;
    if (pll->theta >= 2 * PI)
        pll->theta -= 2 * PI;

    // 频率微调(简单比例控制)
    float theta_err = atan2(-pll->x2, pll->x1 - u_alpha);
    pll->w0 += 10.0f * theta_err; // 频率调整系数
    pll->freq = pll->w0 / (2 * PI);

    pll->x1 = x1_new;
    pll->x2 = x2_new;
}

// Clark变换(三相到两相静止)
void clark_transform(float a, float b, float c, AlphaBeta_t *ab)
{
    ab->alpha = (2.0f * a - b - c) / 3.0f;
    ab->beta = (b - c) * sqrt(3.0f) / 3.0f;
}

// Park变换(两相静止到旋转dq)
void park_transform(AlphaBeta_t *ab, float theta, DQ_t *dq)
{
    dq->d = ab->alpha * cos(theta) + ab->beta * sin(theta);
    dq->q = -ab->alpha * sin(theta) + ab->beta * cos(theta);
}

// 逆Park变换(旋转dq到两相静止)
void inv_park_transform(DQ_t *dq, float theta, AlphaBeta_t *ab)
{
    ab->alpha = dq->d * cos(theta) - dq->q * sin(theta);
    ab->beta = dq->d * sin(theta) + dq->q * cos(theta);
}

// 整流控制初始化
void rect_control_init()
{
    // 初始化SOGI-PLL(50Hz电网)
    sogi_pll_init(&sogi_pll, 1.414f, 2 * PI * 50);

    // 初始化PI控制器
    pid_init(&pi_udc, PID_POSITION, 0.05f, 0.5f, 0, 0, MAX_CURRENT, 0); // 电压环
    pid_init(&pi_id, PID_POSITION, 0.2f, 1.0f, 0, -300, 300, 0);        // d轴电流环
    pid_init(&pi_iq, PID_POSITION, 0.2f, 1.0f, 0, -300, 300, 0);        // q轴电流环(目标0)

    // 初始化SVPWM
    svpwm_init(&rect_svpwm, 0, 50, 18000); // 与硬件匹配的参数
}

// 整流控制主函数(定时调用，建议100kHz)
void rect_control_loop(float Ts)
{
    // 1. 采样数据获取(实际应用中需替换为ADC采样值)
    // grid_vol_abc[0] = ...;  // A相电压
    // grid_vol_abc[1] = ...;  // B相电压
    // grid_vol_abc[2] = ...;  // C相电压
    // grid_curr_abc[0] = ...; // A相电流
    // grid_curr_abc[1] = ...; // B相电流
    // grid_curr_abc[2] = ...; // C相电流
    // u_dc_fdb = ...;         // 直流侧电压反馈

    // 2. SOGI-PLL锁相(基于A相电压)
    clark_transform(grid_vol_abc[0], grid_vol_abc[1], grid_vol_abc[2], &grid_vol_ab);
    sogi_pll_update(&sogi_pll, grid_vol_ab.alpha, Ts);

    // 3. 电流坐标变换
    clark_transform(grid_curr_abc[0], grid_curr_abc[1], grid_curr_abc[2], &grid_curr_ab);
    park_transform(&grid_curr_ab, sogi_pll.theta, &grid_curr_dq);

    // 4. 双闭环控制
    // 4.1 直流电压环(输出d轴电流参考)
    pi_udc.ref = U_DC_REF;
    pi_udc.fdb = u_dc_fdb;
    pid_calculate(&pi_udc);
    float id_ref = pi_udc.output;

    // 4.2 电流环(q轴电流目标为0，实现单位功率因数)
    pi_id.ref = id_ref;
    pi_id.fdb = grid_curr_dq.d;
    pid_calculate(&pi_id);

    pi_iq.ref = 0.0f;
    pi_iq.fdb = grid_curr_dq.q;
    pid_calculate(&pi_iq);

    // 5. 逆坐标变换得到电压参考
    volt_ref_dq.d = pi_id.output;
    volt_ref_dq.q = pi_iq.output;
    AlphaBeta_t volt_ref_ab;
    inv_park_transform(&volt_ref_dq, sogi_pll.theta, &volt_ref_ab);

    // 6. SVPWM计算与输出
    rect_svpwm.Ualpha = volt_ref_ab.alpha;
    rect_svpwm.Ubeta = volt_ref_ab.beta;
    svpwm_calculate(&rect_svpwm);
}

// 定时器中断服务函数(示例，需与实际定时器匹配)
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
        // 100kHz采样周期(10us)
        rect_control_loop(1e-5f);
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}