// 使用前先初始化DSP库
#ifndef _KALMAN_FILTER_HPP
#define _KALMAN_FILTER_HPP

#ifdef __cplusplus
extern "C"
{
#ifdef STM32F405xx
#include "stm32f405xx.h"
#elif STM32H723xx
#include "stm32h723xx.h"

#endif

#include "arm_math.h"
// #include "dsp/matrix_functions.h"
#include "math.h"
#include "stdint.h"
#include "stdlib.h"

#ifdef __cplusplus
}
#endif

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif
namespace kalman_filter_N
{
// 若运算速度不够,可以使用q31代替f32,但是精度会降低
#define mat arm_matrix_instance_f32        // 浮点矩阵结构的矩阵结构体   ( 包含 行数、列数、以及数组(数据) )
#define Matrix_Init arm_mat_init_f32       // 浮点矩阵初始化
#define Matrix_Add arm_mat_add_f32         // 浮点矩阵加法
#define Matrix_Subtract arm_mat_sub_f32    // 浮点矩阵减法
#define Matrix_Multiply arm_mat_mult_f32   // 浮点矩阵乘法
#define Matrix_Transpose arm_mat_trans_f32 // 浮点矩阵转置
#define Matrix_Inverse arm_mat_inverse_f32 // 浮点矩阵逆
    // kalman_filter_c(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
    //  ~kalman_filter_c();
    class first_order_kalman_filter_c
    {
        float X_last;      // 上一时刻的最优结果  X(k|k-1)
        float X_predict;   // 当前时刻的预测结果  X(k|k-1)
        float X_now;       // 当前时刻的最优结果  X(k|k)
        float P_predict;   // 当前时刻预测结果的协方差  P(k|k-1)
        float P_now;       // 当前时刻最优结果的协方差  P(k|k)
        float P_last;      // 上一时刻最优结果的协方差  P(k-1|k-1)
        float kalman_gain; // kalman增益
        float Q;           // 过程噪声方差 (需要传入)
        float R;           // 测量噪声
        // 以下为直接省略的内容
        //  float A;     // 系统参数 1
        //  float B;     // 控制矩阵 0
        //  float H;     // 测量矩阵 1
        first_order_kalman_filter_c(float T_Q, float T_R);
        float first_order_kalman_filter_calc(float data);
        void first_order_kalman_filter_init(float T_Q, float T_R); // 简易一阶卡尔曼滤波,可供用户返回
    };

    /***********************************************************************************************************/
    typedef struct kf_t
    {
        float *FilteredValue;  // 最终过滤值
        float *MeasuredVector; // 测量向量
        float *ControlVector;  // 控制向量

        uint8_t xhatSize; // 状态变量维度
        uint8_t uSize;    // controlVector 控制变量维度
        uint8_t zSize;    // measurementVectorSize观测量维度

        uint8_t UseAutoAdjustment;   // 自动调整标志位
        uint8_t MeasurementValidNum; // 有效测量值

        uint8_t *MeasurementMap;      // 量测与状态的关系 how measurement relates to the state
        float *MeasurementDegree;     // 测量值对应H矩阵元素值 elements of each measurement in H
        float *MatR_DiagonalElements; // 量测方差 variance for each measurement
        float *StateMinVariance;      // 最小方差 避免方差过度收敛 suppress filter excessive convergence
        uint8_t *temp;

        // 配合用户定义函数使用,作为标志位用于判断是否要跳过标准KF中五个环节中的任意一个
        uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

        // definiion of struct mat: rows & cols & pointer to vars
        mat xhat;      // x(k|k)   当前预测值
        mat xhatminus; // x(k|k-1) 上一刻的预测值
        mat u;         // control vector u 控制向量
        mat z;         // measurement vector z 测量向量
        mat P;         // covariance matrix P(k|k)   当前时刻协方差矩阵
        mat Pminus;    // covariance matrix P(k|k-1) 上一时刻协方差矩阵
        mat F, FT;     // state transition matrix F FT 状态转移矩阵
        mat B;         // control matrix B  控制矩阵
        mat H, HT;     // measurement matrix H 测量矩阵
        mat Q;         // process noise covariance matrix Q  过程噪声协方差矩阵
        mat R;         // measurement noise covariance matrix R 测量噪声协方差矩阵
        mat K;         // kalman gain  K 卡尔曼增益
        mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

        int8_t MatStatus; // 用于检测矩阵运算状态

        // 用户定义函数,可以替换或扩展基准KF的功能
        void (*User_Func0_f)(struct kf_t *kf);
        void (*User_Func1_f)(struct kf_t *kf);
        void (*User_Func2_f)(struct kf_t *kf);
        void (*User_Func3_f)(struct kf_t *kf);
        void (*User_Func4_f)(struct kf_t *kf);
        void (*User_Func5_f)(struct kf_t *kf);
        void (*User_Func6_f)(struct kf_t *kf);

        // 矩阵存储空间指针
        float *xhat_data, *xhatminus_data; // 预测值
        float *u_data;                     // 控制向量
        float *z_data;                     // 测量向量
        float *P_data, *Pminus_data;       // 协方差
        float *F_data, *FT_data;           // 状态转移矩阵
        float *B_data;                     // 控制矩阵
        float *H_data, *HT_data;           // 测量矩阵
        float *Q_data;                     // 过程噪声方差
        float *R_data;                     // 测量噪声
        float *K_data;                     // 卡尔曼增益
        float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;
    } KalmanFilter_t;

    void kalman_filter_init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize); // 初始化
    float *kalman_filter_update(KalmanFilter_t *kf);                                             // 获取过滤值
    void kalman_filter_measure(KalmanFilter_t *kf);                                              // 获取量测信息
    void kalman_filter_xhatMinus_update(KalmanFilter_t *kf);                                     // 更新上个预测值
    void kalman_filter_Pminus_update(KalmanFilter_t *kf);                                        // 更新上一个协方差
    void kalman_filter_setK(KalmanFilter_t *kf);                                                 // 设置卡尔曼增益
    void kalman_filter_xhat_update(KalmanFilter_t *kf);                                          // 更新预测值
    void kalman_filter_P_update(KalmanFilter_t *kf);                                             // 更新当前协方差

    /*********************************************************************/
    extern uint16_t sizeof_float, sizeof_double;
}
#endif
#endif /*_KALMAN_FILTER_H*/