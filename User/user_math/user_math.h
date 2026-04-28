#ifndef USER_MATH_H
#define USER_MATH_H

#include <stdint.h>

#ifndef COLLECTION_FILTER_WINDOW_MAX
#define COLLECTION_FILTER_WINDOW_MAX 50U
#endif

typedef struct ave_process
{
    float ave_data;
    float sum;
    uint16_t cnt;
} ave_process_t;

typedef struct
{
    /* 一阶低通滤波器状态：y[n] = a*x[n] + (1-a)*y[n-1] */
    float alpha;
    float last_output;
    uint8_t initialized;
} FirstOrderLPF;

typedef struct
{
    /* 一阶高通滤波器状态：y[n] = a*(y[n-1] + x[n] - x[n-1]) */
    float alpha;
    float last_input;
    float last_output;
    uint8_t initialized;
} FirstOrderHPF;

typedef struct
{
    /* 环形缓冲递推平均，避免旧实现中每次更新都搬移整个 FIFO。 */
    float fifo[COLLECTION_FILTER_WINDOW_MAX];
    float sum;
    uint16_t index;
    uint16_t count;
    uint16_t window;
} Recursive_ave_filter_type_t;

/* 一阶低通滤波器接口。 */
void LPF_Init(FirstOrderLPF *filter, float alpha, float init_value);
float LPF_Update(FirstOrderLPF *filter, float input);
void LPF_Reset(FirstOrderLPF *filter, float new_value);

/* 一阶高通滤波器接口。 */
void HPF_Init(FirstOrderHPF *filter, float alpha, float init_input);
float HPF_Update(FirstOrderHPF *filter, float input);
void HPF_Reset(FirstOrderHPF *filter, float new_input);

/* 递推平均滤波器接口。 */
void Recursive_ave_filter_init(Recursive_ave_filter_type_t *filter, uint16_t window, float init_value);
float Recursive_ave_filter(Recursive_ave_filter_type_t *filter, float input);
void Recursive_ave_filter_reset(Recursive_ave_filter_type_t *filter, float init_value);

/* 保留原有 RMS 计算接口，供后续交流量采样扩展复用。 */
float get_rms(ave_process_t *ave_process, float value);

#endif // USER_MATH_H
