#ifndef __MATHS_H
#define __MATHS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "arm_math.h"

#ifdef __cplusplus
}
#endif
namespace alg_n
{

#define msin(x) (arm_sin_f32(x))
#define mcos(x) (arm_cos_f32(x))

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

    typedef arm_matrix_instance_f32 mat;
// 若运算速度不够,可以使用q31代替f32,但是精度会降低
#define MatAdd arm_mat_add_f32
#define MatSubtract arm_mat_sub_f32
#define MatMultiply arm_mat_mult_f32
#define MatTranspose arm_mat_trans_f32
#define MatInverse arm_mat_inverse_f32

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif
    void MatInit(mat *m, uint8_t row, uint8_t col);
    // 运动加速度限制斜坡函数
    typedef struct
    {
        float Input;      // 当前取样值
        float Last_Input; // 上次取样值
        float Output;     // 输出值
        float acc_now;    // 当前加速度
        float acc_limit;  // 需要限制的加速度
    } acceleration_control_t;
    class acceleration_control_c
    {
        acceleration_control_t acc_control;
        int16_t motion_acceleration_control(int16_t Input, int16_t Limit);
        float motion_acceleration_control(float Input, float Limit);
    };
// 绝对值
#define user_abs(x) ((x) > (0) ? (x) : (-(x)))

// 数字限幅
#define user_value_limit(val, min, max)        \
    {                                          \
        (val) = (val) < (min) ? (min) : (val); \
        (val) = (val) > (max) ? (max) : (val); \
    }
// 最大值
#define user_max(x, y) ((x) > (y) ? (x) : (y))
#define user_min(x, y) ((x) < (y) ? (x) : (y))
// 弧度格式化为-PI~PI
#define RAD_FORMAT(Ang) loop_fp32_constrain((Ang), -PI, PI)

/**输出的 y 的正负性, 由 x 的正负性质来决定, 可以用于减少 if else*/
#define anti_abs_value(x, y) {(x > 0) ? ((y > 0) ? y : -y) : ((y < 0) ? y : -y)}
    // 取最大值的绝对值

#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

    /**
     * @brief 返回一块干净的内�?,不过仍然需要强制转�?为你需要的类型
     *
     * @param size 分配大小
     * @return void*
     */
    void *zmalloc(size_t size);

    // 快速开方
    float Sqrt(float x);

    // 角度格式化为-180~180
    float theta_format(float Ang);

    int float_rounding(float raw);

    float *Norm3d(float *v);

    float NormOf3d(float *v);

    void Cross3d(float *v1, float *v2, float *res);

    float Dot3d(float *v1, float *v2);

    float AverageFilter(float new_data, float *buf, uint8_t len);

    float invSqrt(float x);                                                               // 平方根倒数
    float float_min_distance(float target, float actual, float minValue, float maxValue); // 寻最小值
    float loop_fp32_constrain(float Input, float minValue, float maxValue);               // 循环限制（云台角度处理）
    float CircleIncreaseLimit(float current_val, float set_val, float limit);             /* 循环限制 */
    float cos_calculate(float angle);
    float sin_calculate(float angle);
    /* 限幅滤波 */
    float limiting_filter(float new_value, float last_value, float delat_max);
    /* 斜坡函数 */
    void data_accelerated_control(float *input, float acc); // 加速度限制斜坡函数
    bool IsInvalid_loat(float x);
    template <typename Type>
    inline Type user_val_limit(Type val, Type min, Type max)
    {
        (val) = (val) < (min) ? (min) : (val);
        (val) = (val) > (max) ? (max) : (val);
        return val;
    }
    /*取两个数最大值的绝对值*/
    template <typename Type>
    inline Type max_abs(Type x, Type y)
    {
        if (user_abs(x) >= user_abs(y))
            return user_abs(x);
        else
            return user_abs(y);
    }
    // 绝对值限制
    inline float abs_limit(float num, float Limit)
    {
        if (num > Limit)
        {
            num = Limit;
        }
        else if (num < -Limit)
        {
            num = -Limit;
        }
        return num;
    }

    // 判断符号位
    inline float sign(float value)
    {
        if (value >= 0.0f)
        {
            return 1.0f;
        }
        else
        {
            return -1.0f;
        }
    }
    // 浮点死区
    template <typename Type>
    inline Type deadband(Type Value, Type minValue, Type maxValue)
    {
        if (Value < maxValue && Value > minValue)
        {
            Value = 0.0f;
        }
        return Value;
    }

    /*
     *功能：正负循环限制
     *传入：1.输入值  2.限制幅度(正数)
     *传出：限幅输出值
     *描述：将输入值限制在 +-限制幅度 的范围内
     */

    template <typename Type>
    inline Type loop_restriction(Type num, Type limit_num)
    {
        if (user_abs(num) > limit_num)
        {
            if (num >= 0)
                num -= limit_num;
            else
                num += limit_num;
        }
        return num;
    }

    /**
     * @brief 判断浮点数是否为无效浮点数
     *
     * @param x 浮点数
     * @return 是否为NaN
     */
    inline bool IsInvalid_loat(float x)
    {
        uint32_t exp = (*(uint32_t *)(&x) >> 23) & 0xff;
        return (exp == 0xff || exp == 0x00);
    }

}

#endif
