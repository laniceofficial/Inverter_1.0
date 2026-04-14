#include "user_math.h"

#include <math.h>
#include <stddef.h>

void LPF_Init(FirstOrderLPF *filter, float alpha, float init_value)
{
    if (filter == NULL)
    {
        return;
    }

    if (alpha <= 0.0f)
    {
        alpha = 0.1f;
    }
    else if (alpha >= 1.0f)
    {
        alpha = 0.9f;
    }

    filter->alpha = alpha;
    filter->last_output = init_value;
    filter->initialized = 1U;
}

float LPF_Update(FirstOrderLPF *filter, float input)
{
    /* 一阶低通适合快速环路：计算量固定，状态只有一个上次输出值。 */
    float output;

    if ((filter == NULL) || (filter->initialized == 0U))
    {
        return input;
    }

    output = filter->alpha * input + (1.0f - filter->alpha) * filter->last_output;
    filter->last_output = output;
    return output;
}

void LPF_Reset(FirstOrderLPF *filter, float new_value)
{
    if (filter == NULL)
    {
        return;
    }

    filter->last_output = new_value;
}

float get_rms(ave_process_t *ave_process, float value)
{
    /* 保留旧接口，用固定窗口累计平方和后输出均方根。 */
    if (ave_process->cnt < 1000U)
    {
        ave_process->sum += value * value;
        ave_process->cnt++;
    }

    if (ave_process->cnt == 1000U)
    {
        ave_process->ave_data = sqrtf(ave_process->sum / 128.0f);
        ave_process->cnt = 0U;
        ave_process->sum = 0.0f;
    }

    return ave_process->ave_data;
}

void Recursive_ave_filter_init(Recursive_ave_filter_type_t *filter, uint16_t window, float init_value)
{
    uint16_t i;

    if (filter == NULL)
    {
        return;
    }

    if (window == 0U)
    {
        window = 1U;
    }
    else if (window > COLLECTION_FILTER_WINDOW_MAX)
    {
        window = COLLECTION_FILTER_WINDOW_MAX;
    }

    filter->sum = init_value * window;
    filter->index = 0U;
    filter->count = window;
    filter->window = window;

    /* 初始化整个环形缓冲，使滤波器在刚启动时输出稳定。 */
    for (i = 0U; i < COLLECTION_FILTER_WINDOW_MAX; ++i)
    {
        filter->fifo[i] = (i < window) ? init_value : 0.0f;
    }
}

float Recursive_ave_filter(Recursive_ave_filter_type_t *filter, float input)
{
    /* O(1) 递推平均：只减去被覆盖旧值，再加上新值。 */
    float removed_value;

    if ((filter == NULL) || (filter->window == 0U))
    {
        return input;
    }

    removed_value = filter->fifo[filter->index];
    filter->fifo[filter->index] = input;
    filter->sum += input - removed_value;

    filter->index++;
    if (filter->index >= filter->window)
    {
        filter->index = 0U;
    }

    if (filter->count < filter->window)
    {
        filter->count++;
    }

    return filter->sum / (float)filter->count;
}

void Recursive_ave_filter_reset(Recursive_ave_filter_type_t *filter, float init_value)
{
    uint16_t i;

    if ((filter == NULL) || (filter->window == 0U))
    {
        return;
    }

    filter->sum = init_value * filter->window;
    filter->index = 0U;
    filter->count = filter->window;

    for (i = 0U; i < filter->window; ++i)
    {
        filter->fifo[i] = init_value;
    }
}
