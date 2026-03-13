#include "FreeRTOS.h"
#include "Matrix.hpp"
#include "task.h"

#pragma once

namespace Core
{
namespace Control
{
namespace Math
{

template <uint32_t dim>
class RLS
{
   public:
    /**
     * @brief 删除默认构造函数
     */
    RLS() = delete;

    /**
     * @brief 构造函数
     * @param delta_ 初始化的非奇异转移矩阵值
     * @param lambda_ 忘记因子
     */
    constexpr RLS(float delta_, float lambda_)
        : dimension(dim), lambda(lambda_), delta(delta_), lastUpdate(0), updateCnt(0), defaultParamsVector(Matrixf<dim, 1>::zeros())
    {
        this->reset();
        this->validate();
    }

    /**
     * @brief 构造函数（带默认参数初始化）
     * @param delta_  初始化的转移矩阵对角线值（需保证矩阵可逆）
     * @param lambda_ 遗忘因子（0 < lambda ≤ 1，1表示不遗忘历史数据）
     * @param initParam 参数向量初始值（dim x 1维向量）
     */
    constexpr RLS(float delta_, float lambda_, Matrixf<dim, 1> initParam) : RLS(delta_, lambda_) 
    { 
        defaultParamsVector = initParam;  // 设置默认参数
        paramsVector = initParam;         // 初始化当前参数
    }

    /**
     * @brief 重置RLS模块到初始状态
     * @note 会重置转移矩阵、增益向量和参数向量
     */
    void reset()
    {
        transMatrix  = Matrixf<dim, dim>::eye() * delta;  // 单位矩阵乘以delta初始化
        gainVector   = Matrixf<dim, 1>::zeros();          // 增益向量清零
        paramsVector = defaultParamsVector;               // 恢复默认参数
    }

    /**
     * @brief 执行一次RLS参数更新
     * @param sampleVector 输入特征向量（dim x 1维）
     * @param actualOutput 实际观测值（标量）
     * @return 更新后的参数向量（引用）
     * 
     * RLS算法步骤：
     * 1. 计算增益向量 K = (P * x) / (lambda + x^T * P * x)
     * 2. 更新参数 theta = theta + K * (y - x^T * theta)
     * 3. 更新协方差矩阵 P = (P - K * x^T * P) / lambda
     */
    const Matrixf<dim, 1>& update(Matrixf<dim, 1>& sampleVector, float actualOutput)
    {
        // 计算中间项：x^T * P * x + lambda
        float denominator = (sampleVector.trans() * transMatrix * sampleVector)[0][0] + lambda;
        
        // 计算增益向量 K = P * x / denominator
        gainVector = (transMatrix * sampleVector) / denominator;
        
        // 计算先验误差：e = y - x^T * theta
        float priorError = actualOutput - (sampleVector.trans() * paramsVector)[0][0];
        
        // 更新参数向量：theta = theta + K * e
        paramsVector += gainVector * priorError;
        
        // 更新协方差矩阵：P = (P - K * x^T * P) / lambda
        transMatrix = (transMatrix - gainVector * sampleVector.trans() * transMatrix) / lambda;

        // 更新统计信息
        updateCnt++;
        lastUpdate = xTaskGetTickCount();
        return paramsVector;
    }

    /**
     * @brief 设置默认回归参数
     * @param updatedParams 更新后的参数向量
     * @retval None
     */
    void setParamVector(const Matrixf<dim, 1> &updatedParams)
    {
        paramsVector        = updatedParams;  // 更新参数向量
        defaultParamsVector = updatedParams;  // 更新默认参数向量
    }

    /**
     * @brief 获取参数向量
     * @param None
     * @retval paramsVector 参数向量
     */
    constexpr Matrixf<dim, 1> &getParamsVector() const { return paramsVector; }

    /**
     * @brief 获取输出向量
     * @param None
     * @retval RLS模块的估计/滤波输出
     */
    const float &getOutput() const { return output; }

   private:
    /**
     * @brief 验证lambda和delta的合法性
     * @param None
     * @retval None
     */
    void validate() const
    {
        configASSERT(lambda >= 0.0f || lambda <= 1.0f);  // 验证lambda是否在0到1之间
        configASSERT(delta > 0);                         // 验证delta是否大于0
    }

    uint32_t dimension;  // RLS空间的维度
    float lambda;        // 忘记因子
    float delta;         // 协方差矩阵初始对角线值

    TickType_t lastUpdate;  // 上次更新的时间戳
    uint32_t updateCnt;     // 总更新次数

    /*RLS相关的矩阵*/
    Matrixf<dim, dim> transMatrix;  // 协方差逆矩阵（通常记作P）
    Matrixf<dim, 1> gainVector;     // 卡尔曼增益向量
    Matrixf<dim, 1> paramsVector;   // 待估计的参数向量
    Matrixf<dim, 1> defaultParamsVector;  // 默认参数向量
    float output;  // 估计/滤波输出
};

}  // namespace Math
}  // namespace Control
}  // namespace Core


/*
// 假设Matrixf是兼容Eigen风格的矩阵类
using namespace Core::Control::Math;

// 示例：二维参数估计（theta = [a; b]）
constexpr uint32_t DIM = 2;

// 初始化RLS：delta=100（大初始不确定性），lambda=0.95（适度遗忘）
RLS<DIM> rlsEstimator(100.0f, 0.95f);

// 模拟数据生成
Matrixf<DIM, 1> sample;
float true_a = 1.5f, true_b = -0.8f;  // 真实参数
float noise_std = 0.1f;               // 观测噪声标准差

for(int i = 0; i < 100; ++i) {
    // 生成输入特征（假设系统输入为u，输出方程为 y = a*u + b*u^2）
    float u = rand()/(float)RAND_MAX * 2.0f - 1.0f; // 输入在[-1,1]之间
    sample[0][0] = u;
    sample[1][0] = u*u;

    // 生成带噪声的观测值
    float true_output = true_a * u + true_b * u*u;
    float noisy_output = true_output + noise_std * (rand()/(float)RAND_MAX - 0.5f);

    // RLS参数更新
    auto& theta = rlsEstimator.update(sample, noisy_output);

    // 打印估计结果
    if(i % 10 == 0) {
        printf("Iter %3d: a=%.3f (true=1.5), b=%.3f (true=-0.8)\n", 
               i, theta[0][0], theta[1][0]);
    }
}


*/