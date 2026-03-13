#include "user_maths.hpp"
#include <stddef.h>
#include "list_of_trigfunction.h"
namespace alg_n
{

/*
 *功能：运动控制斜坡函数（加速度限制）
 *传入：1.加速度限制对应结构体  2.控制量 3.限制加速度
 *传出：处理量
 *类型：16位整形
 */
int16_t acceleration_control_c::motion_acceleration_control(int16_t Input, int16_t Limit)
{
    this->acc_control.Input = Input;
    this->acc_control.acc_limit = Limit;

    this->acc_control.acc_now = this->acc_control.Input - this->acc_control.Last_Input;

    if (user_abs(this->acc_control.acc_now) > this->acc_control.acc_limit)
    {
        this->acc_control.Output = this->acc_control.Last_Input + this->acc_control.acc_now / user_abs(this->acc_control.acc_now) * this->acc_control.acc_limit;
    }

    this->acc_control.Last_Input = this->acc_control.Output;

    return this->acc_control.Output;
}
float acceleration_control_c::motion_acceleration_control(float Input, float Limit)
{
    this->acc_control.Input = Input;
    this->acc_control.acc_limit = Limit;

    this->acc_control.acc_now = this->acc_control.Input - this->acc_control.Last_Input;

    if (user_abs(this->acc_control.acc_now) > this->acc_control.acc_limit)
    {
        this->acc_control.Output = this->acc_control.Last_Input + this->acc_control.acc_now / user_abs(this->acc_control.acc_now) * this->acc_control.acc_limit;
    }

    this->acc_control.Last_Input = this->acc_control.Output;

    return this->acc_control.Output;
}

float invSqrt(float x) // 平方根倒数速算法
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/*循环限幅32*/
float loop_fp32_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

// 斜坡函数(加速度限制)
void data_accelerated_control(float *input, float acc)
{
    static int16_t last_num = 0;
    int16_t temp;
    temp = *input - last_num;

    if (user_abs(temp) > acc)
        *input = last_num + temp / user_abs(temp) * acc;

    last_num = *input;
}

// 限幅滤波函数
float limiting_filter(float new_value, float last_value, float delat_max)
{
    if ((new_value - last_value > delat_max) || (last_value - new_value > delat_max))
        return last_value;
    return new_value;
}

// sin函数查表法
float sin_calculate(float angle)
{
    float sin_angle = 0.0f;

    if (angle >= 0.0f && angle < 90.0f)
        sin_angle = (Trigonometric_Functions[(int)(user_abs(angle) * 10.0f)] / 100.0f);
    else if (angle >= 90.0f && angle < 180.f)
        sin_angle = (Trigonometric_Functions[(int)(user_abs(180.0f - angle) * 10.0f)] / 100.0f);
    else if (angle >= -180.0f && angle < -90.f)
        sin_angle = -(Trigonometric_Functions[(int)(user_abs(180.0f + angle) * 10.0f)] / 100.0f);
    else if (angle >= -90.0f && angle < 0.f)
        sin_angle = -(Trigonometric_Functions[(int)(user_abs(180.0f - (180.0f + angle)) * 10.0f)] / 100.0f);
    else if (angle == 180.f)
        sin_angle = 0.0f;

    return sin_angle;
}

// cos函数查表法
float cos_calculate(float angle)
{
    float cos_angle = 0.0f;

    angle = user_abs(angle);

    if (angle >= 0.0f && angle < 90.0f)
        cos_angle = (Trigonometric_Functions[(int)(user_abs(90.0f - angle) * 10.0f)] / 100.0f);
    else if (angle >= 90.0f && angle < 180.f)
        cos_angle = -(Trigonometric_Functions[(int)(user_abs(angle - 90.0f) * 10.0f)] / 100.0f);
    else if (angle == 180.f)
        cos_angle = -1.0f;

    return cos_angle;
}

float float_min_distance(float target, float actual, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return 0;
    }

    target = loop_fp32_constrain(target, minValue, maxValue);

    if (user_abs(target - actual) > (maxValue - minValue) / 2.0f)
    {
        if (maxValue - actual < (maxValue - minValue) / 2.0f)
        {
            return maxValue - actual + target - minValue;
        }
        else
        {
            return minValue - actual + target - maxValue;
        }
    }
    else
    {
        return target - actual;
    }
}
/***
 * @brief 周期递增限制
 * @param current_val 当前实际值
 * @param set_val 设定值
 * @param limit 每次调用该函数最大递增的值, 为正数
 * @return 返回一个处理好的值
 */
float CircleIncreaseLimit(float current_val, float set_val, float limit)
{
    limit = user_abs(limit);
    if (set_val - current_val > limit)
    {
        return current_val + limit;
    }
    else if (set_val - current_val < -limit)
    {
        return current_val - limit;
    }
    else
    {
        return set_val;
    }
}

void *zmalloc(size_t size)
{
    void *ptr = malloc(size);
    memset(ptr, 0, size);
    return ptr;
}

// 快速开方
float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

// 弧度格式化为-PI~PI

// 角度格式化为-180~180
float theta_format(float Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

// 三维向量归一化
float *Norm3d(float *v)
{
    float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
    return v;
}

// 计算模长
float NormOf3d(float *v)
{
    return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 三维向量叉乘v1 x v2
void Cross3d(float *v1, float *v2, float *res)
{
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 三维向量点乘
float Dot3d(float *v1, float *v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// 均值滤波,删除buffer中的最后一个元素,填入新的元素并求平均值
float AverageFilter(float new_data, float *buf, uint8_t len)
{
    float sum = 0;
    for (uint8_t i = 0; i < len - 1; i++)
    {
        buf[i] = buf[i + 1];
        sum += buf[i];
    }
    buf[len - 1] = new_data;
    sum += new_data;
    return sum / len;
}
// todo：建立单独矩阵库
void MatInit(mat *m, uint8_t row, uint8_t col)
{
    m->numCols = col;
    m->numRows = row;
    m->pData = (float *)zmalloc(row * col * sizeof(float));
}
}