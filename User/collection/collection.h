#ifndef  COLLECTION_H
#define  COLLECTION_H
#include "adc.h"
#define VOL_REF 3.309f        // 参考电压
#define ADC_MAX_VALUE 4095.0f // ADC最大值
typedef struct collect_data
{
    float volatage[3];
    float current[3];
} collect_data_t;
typedef struct ave_process
{
    float ave_data;
    float sum;
    uint16_t cnt;
}ave_process_t;
// 递推平均滤波参数
typedef  struct
{
    int32_t count_num;
    float fifo[50];
    float sum;
} Recursive_ave_filter_type_t;
collect_data_t *collection_init(void);
void collection_stop(void);
void collection_update(void);
float get_rms(ave_process_t *ave_process, float value);
float Recursive_ave_filter_init(Recursive_ave_filter_type_t *filter);
float Recursive_ave_filter(Recursive_ave_filter_type_t *filter, float input, int num);
#endif // ! COLLECTION_H
