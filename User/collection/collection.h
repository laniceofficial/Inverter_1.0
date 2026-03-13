#ifndef  COLLECTION_H
#define  COLLECTION_H
#include "adc.h"
#define VOL_REF 3.306f        // 参考电压
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
} ave_process_t;
typedef struct {
  float alpha;         // 滤波系数 (0~1)
  float last_output;   // 上次输出值
  uint8_t initialized; // 初始化标志
} FirstOrderLPF;
/**
 * @brief 初始化一阶低通滤波器
 * @param filter 滤波器结构体指针
 * @param alpha 滤波系数 (0~1)，建议值 0.1~0.3
 * @param init_value 初始值
 */
void LPF_Init(FirstOrderLPF *filter, float alpha, float init_value);

/**
 * @brief 执行滤波计算
 * @param filter 滤波器结构体指针
 * @param input 当前输入值
 * @return 滤波后的输出值
 */
float LPF_Update(FirstOrderLPF *filter, float input);

/**
 * @brief 重置滤波器状态
 * @param filter 滤波器结构体指针
 * @param new_value 新的初始值
 */
void LPF_Reset(FirstOrderLPF *filter, float new_value);

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
