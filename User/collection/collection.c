// ADC2--IN4(AC3_v)        IN11(AC2_V)
//     --IN5(AC3_I)        IN12(AC1_I)
// ADC3--IN1(AC1_V)        /***********IN5(DC1_V)
//     --IN12(AC2_I)
#include "collection.h"
#include "arm_math.h"
#include "stdlib.h"
// uint16_t adc1_data[9] = {0};
uint16_t adc2_data[16] = {0};
uint16_t adc3_data[12] = {0};
// uint16_t adc4_data[10] = {0};
// uint16_t adc5_data[10] = {0};
collect_data_t *data = NULL;
Recursive_ave_filter_type_t Vfilter;
Recursive_ave_filter_type_t Cfilter;

collect_data_t *collection_init(void)
{
    // while (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED)!=HAL_OK)
    //     ;
    while (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED)!=HAL_OK)
        ;
    while (HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED) != HAL_OK)
        ;
    // HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);
    // HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);

    // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_data, 9);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_data, 16);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc3_data, 12);
    // HAL_ADC_Start_DMA(&hadc4, (uint32_t *)adc4_data, 6);
    // HAL_ADC_Start_DMA(&hadc5, (uint32_t *)adc5_data, 6);
    Recursive_ave_filter_init(&Vfilter);
    Recursive_ave_filter_init(&Cfilter);
    data = (collect_data_t *)malloc(sizeof(collect_data_t));
    return data;
}

inline void collection_update(void)
{
    static float current_ac = 0;
    static float voltage_ac = 0;
    static ave_process_t currunt_ave;
    static ave_process_t voltage_ave;
    float raw_data = ((float)(adc2_data[3] + adc2_data[7] + adc2_data[11] + adc2_data[15]) / 4.f - 1985.45f) * 12.121f * VOL_REF / ADC_MAX_VALUE;
    current_ac = Recursive_ave_filter(&Cfilter, raw_data, 10);
    data->current = get_rms(&currunt_ave, current_ac);
    raw_data = ((float)(adc3_data[0] + adc3_data[3] + adc3_data[6] + adc3_data[9]) / 4.f - 1985.45f) * VOL_REF * 50 / ADC_MAX_VALUE;
    voltage_ac = Recursive_ave_filter(&Vfilter, raw_data, 20);
    data->volatage = get_rms(&voltage_ave, voltage_ac);
}

void collection_stop(void)
{
    // HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);
    HAL_ADC_Stop_DMA(&hadc4);
    HAL_ADC_Stop_DMA(&hadc5);
}

// 取均方根值
float get_rms(ave_process_t *ave_process, float value)
{
    if (ave_process->cnt < 1000)
    {
        ave_process->sum += value * value; // 计算平方和
        ave_process->cnt++;
    }
    if ((ave_process->cnt == 1000))
    {
        ave_process->ave_data = (uint16_t)sqrt(ave_process->sum / 128); // 平方和取平均，再开方
        ave_process->cnt = 0;
        ave_process->sum = 0;
    }
    return ave_process->ave_data;
}

/*
 *功能：递推平均滤波（浮点型）------抑制大幅度高频噪声
 *传入：1.滤波对象结构体  2.更新值 3.均值数量
 *传出：滑动滤波输出值（max = 100次）
 */
float Recursive_ave_filter(Recursive_ave_filter_type_t *filter, float input, int num)
{
    int i;
    for (i = num - 1; i > 0; i--)
    {
        filter->fifo[i] = filter->fifo[i - 1];
    }
    filter->fifo[0] = input;

    if (filter->count_num == num)
    {
        filter->count_num = 0;
    }

    filter->sum = 0;
    for (i = 0; i < num; i++)
    {
        filter->sum += filter->fifo[i];
    }

    return (filter->sum / num);
}
// 递推平均滤波初始化
float Recursive_ave_filter_init(Recursive_ave_filter_type_t *filter)
{
    filter->count_num = 0;
    filter->sum = 0;
    return 0;
}