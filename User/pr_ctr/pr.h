#ifndef QPR_H
#define QPR_H


// 准PR控制器结构体

typedef struct
{
    float Kp; // 比例系数
    float Kr; // 谐振系数
    float w0; // 谐振角频率 (rad/s)
    float wc; // 截止带宽 (rad/s)
    float Ts; // 采样周期 (s)

    // 离散化系数
    float b0_prime;
    float b2_prime;
    float a1_prime;
    float a2_prime;

    // 状态变量
    float e_prev1; // e[k-1]
    float e_prev2; // e[k-2]
    float yr_prev1; // yr[k-1]
    float yr_prev2; // yr[k-2]

} QPRController;

// 初始化控制器
void QPR_Init(QPRController *qpr, float Kp, float Kr, float w0, float wc, float Ts);
float QPR_Update(QPRController *qpr, float ref, float fdb);

#endif // !QPR_H