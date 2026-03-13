// ADC2--IN4(AC3_v)        IN11(AC2_V)
//     --IN5(AC3_I)        IN12(AC1_I)
// ADC3--IN1(AC1_V)        /***********IN5(DC1_V)
//     --IN12(AC2_I)
//+-13.4v
#include "collection.h"
#include "arm_math.h"
#include "stdlib.h"
#include <stdint.h>
#define ROW 4.0f
#define SAMPLING_2_LENS 750
#define SAMPLING_3_LENS 1000
// uint16_t adc1_data[9] = {0};
uint16_t adc2_data[16] = {0};
uint16_t adc3_data[12] = {0};
// uint16_t adc4_data[10] = {0};
// uint16_t adc5_data[10] = {0};
collect_data_t *data = NULL;
Recursive_ave_filter_type_t Vfilter1;
Recursive_ave_filter_type_t Vfilter2;
Recursive_ave_filter_type_t Vfilter3;
Recursive_ave_filter_type_t Cfilter;
FirstOrderLPF FOFilter;
FirstOrderLPF FOFilter1;
static float current_ac[3][1000] = {0};
static float voltage_ac[3][SAMPLING_3_LENS] = {0};
collect_data_t *collection_init(void) {
  // while (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED)!=HAL_OK)
  //     ;
  while (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
    ;
  // while (HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED) != HAL_OK)
  //   ;
  // HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);
  // HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);

  // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_data, 9);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_data, 16);
  // HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc3_data, 12);
  // HAL_ADC_Start_DMA(&hadc4, (uint32_t *)adc4_data, 6);
  // HAL_ADC_Start_DMA(&hadc5, (uint32_t *)adc5_data, 6);
  Recursive_ave_filter_init(&Vfilter1);
  Recursive_ave_filter_init(&Vfilter2);
  Recursive_ave_filter_init(&Vfilter3);

  Recursive_ave_filter_init(&Cfilter);
  LPF_Init(&FOFilter, 0.25f, 0.0f);
  LPF_Init(&FOFilter1, 0.2f, 0.0f);
  data = (collect_data_t *)malloc(sizeof(collect_data_t));
  return data;
}

inline void collection_update(void) {

  // static float current_ac= {0};
  // static float voltage_ac= {0};
  // static ave_process_t currunt_ave;
  // static ave_process_t voltage_ave;
  // current_ac = Recursive_ave_filter(&Cfilter, raw_data, 10);
  // data->current = get_rms(&currunt_ave, current_ac);
  data->volatage[0] = Recursive_ave_filter(&Vfilter1, data->volatage[0], 20);
  data->volatage[1] = Recursive_ave_filter(&Vfilter2, data->volatage[1], 20);
  data->volatage[2] = Recursive_ave_filter(&Vfilter2, data->volatage[2], 20);

  // data->volatage = get_rms(&voltage_ave, voltage_ac);
}

void collection_stop(void) {
  // HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Stop_DMA(&hadc2);
  HAL_ADC_Stop_DMA(&hadc3);
  HAL_ADC_Stop_DMA(&hadc4);
  HAL_ADC_Stop_DMA(&hadc5);
}

void LPF_Init(FirstOrderLPF *filter, float alpha, float init_value) {
  if (filter == NULL) {
    return;
  }

  // 限制 alpha 在有效范围内
  if (alpha <= 0.0f) {
    alpha = 0.1f;
  } else if (alpha >= 1.0f) {
    alpha = 0.9f;
  }

  filter->alpha = alpha;
  filter->last_output = init_value;
  filter->initialized = 1;
}

float LPF_Update(FirstOrderLPF *filter, float input) {
  if (filter == NULL || !filter->initialized) {
    return input;
  }

  // 一阶低通滤波公式：y[n] = α * x[n] + (1 - α) * y[n-1]
  float output =
      filter->alpha * input + (1.0f - filter->alpha) * filter->last_output;

  // 更新上次输出值
  filter->last_output = output;

  return output;
}

void LPF_Reset(FirstOrderLPF *filter, float new_value) {
  if (filter == NULL) {
    return;
  }

  filter->last_output = new_value;
}
// 取均方根值
float get_rms(ave_process_t *ave_process, float value) {
  if (ave_process->cnt < 1000) {
    ave_process->sum += value * value; // 计算平方和
    ave_process->cnt++;
  }
  if ((ave_process->cnt == 1000)) {
    ave_process->ave_data =
        sqrt(ave_process->sum / 128); // 平方和取平均，再开方
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
float Recursive_ave_filter(Recursive_ave_filter_type_t *filter, float input,
                           int num) {
  int i;
  for (i = num - 1; i > 0; i--) {
    filter->fifo[i] = filter->fifo[i - 1];
  }
  filter->fifo[0] = input;

  if (filter->count_num == num) {
    filter->count_num = 0;
  }

  filter->sum = 0;
  for (i = 0; i < num; i++) {
    filter->sum += filter->fifo[i];
  }

  return (filter->sum / num);
}
// 递推平均滤波初始化
float Recursive_ave_filter_init(Recursive_ave_filter_type_t *filter) {
  filter->count_num = 0;
  filter->sum = 0;
  return 0;
}

// ADC2--IN4(AC3_v)        IN11(AC2_V)
//     --IN5(AC3_I)        IN12(AC1_I)
// ADC3--IN1(AC1_V)        /***********IN5(DC1_V)
//     --IN12(AC2_I)
float sour_v = 0;
float cal_v = 0;
float base_v = 0;
float cal_without_base_v = 0;
float factor = 17.241f;
// uint16_t SUM_T=0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  //   collection_stop();
  // 预定义常量以提高计算效率
  static const float ADC2_SCALE = VOL_REF / (ROW * ADC_MAX_VALUE);
  static const float ADC3_SCALE = VOL_REF / (ROW * ADC_MAX_VALUE);
  static const float CURRENT_FACTOR = 8.0f;
  static const float VOLTAGE_FACTOR_1 = 250; // 18.9f;
  static const float VOLTAGE_FACTOR_2 = 18.85f;
  static const float CURRENT_OFFSET = 2.49f;
  static const float VOLTAGE_OFFSET_1 = 1.60f;
  static const float VOLTAGE_OFFSET_2 = 1.268f + 0.0f; ///////1.268v
  static const float VOLTAGE_OFFSET_3 = 1.60f + 0.09f;
  static const float VOLTAGE_BASE_OFFSET = 1.133f;
  if (hadc == &hadc2) {
    static uint16_t cnt2 = 0;
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    uint16_t sum3 = 0;
    uint16_t sum4 = 0;
    for (uint8_t i = 0; i < ROW; i++) {
      sum1 += adc2_data[i * 4];
      sum2 += adc2_data[i * 4 + 1];
      sum3 += adc2_data[i * 4 + 2];
      sum4 += adc2_data[i * 4 + 3];
    }

//1.41 -- 1.461  0.058
    // current_ac[0][cnt2] = (sum4 / ROW / ADC_MAX_VALUE * VOL_REF - 2.49) * 8;
    // //ac1_i current_ac[2][cnt2] = (sum2 / ROW / ADC_MAX_VALUE * VOL_REF
    // - 2.49)*8; //ac3_i voltage_ac[2][cnt2] = (sum1 / ROW / ADC_MAX_VALUE *
    // VOL_REF -0.1- 1.60) * 18.9f; voltage_ac[1][cnt2] = (sum3 / ROW /
    // ADC_MAX_VALUE * VOL_REF -0.09- 1.60) * 18.85f; 使用预计算常量减少计算量
    current_ac[0][cnt2] =
        (sum4 * ADC2_SCALE - CURRENT_OFFSET) * CURRENT_FACTOR; // ac1_i
    current_ac[2][cnt2] =
        (sum2 * ADC2_SCALE - CURRENT_OFFSET) * CURRENT_FACTOR; // ac3_i
    // voltage_ac[2][cnt2] =
    // (sum1 * ADC2_SCALE - VOLTAGE_OFFSET_2) * VOLTAGE_FACTOR_1;
    sum3 = LPF_Update(&FOFilter, sum3);
    sum4 = LPF_Update(&FOFilter1, sum4);
    // SUM_T = sum3;

    base_v = (sum4*ADC2_SCALE);
    // float now_base = base_v/ (ROW * ADC_MAX_VALUE);
    sour_v = (sum3 * ADC2_SCALE);
    cal_v = (sum3 * ADC2_SCALE - VOLTAGE_OFFSET_2 + (1.112 - base_v)) * factor;
    cal_without_base_v = (sum3 * ADC2_SCALE - VOLTAGE_OFFSET_2) * factor;
    voltage_ac[1][cnt2] = cal_v;
    // (sum3 * ADC2_SCALE - VOLTAGE_OFFSET_3) * VOLTAGE_FACTOR_2;
    if (++cnt2 >= SAMPLING_2_LENS) {
      arm_rms_f32((const float *)voltage_ac[1], SAMPLING_2_LENS,
                  &data->volatage[1]);
      arm_rms_f32((const float *)voltage_ac[2],
      SAMPLING_2_LENS, &data->volatage[2]);
        arm_rms_f32((const float *)current_ac[0], SAMPLING_2_LENS,
                    &data->current[0]);
        arm_rms_f32((const float *)current_ac[2], SAMPLING_2_LENS,
                    &data->current[2]);
      cnt2 = 0;
    }
  } else if (hadc == &hadc3) {

  static uint16_t cnt3 = 0;
    uint32_t sum1 = 0;
    float sum2 = 0;
    uint32_t sum3 = 0;
    for (uint8_t i = 0; i < ROW; i++) {
      sum1 += adc3_data[i * 3];
      sum2 += adc3_data[i * 3+1];
      sum3 += adc3_data[i * 3 + 2];
    }
    // voltage_ac[0][cnt3] = ((float)sum1 / ROW / ADC_MAX_VALUE * VOL_REF
    // - 1.60) * 18.9f; current_ac[1][cnt3] = ((float)sum3 / ROW / ADC_MAX_VALUE
    // * VOL_REF - 2.49) * 8; 使用预计算常量
    voltage_ac[0][cnt3] =
        (sum1 * ADC3_SCALE - VOLTAGE_OFFSET_1) * VOLTAGE_FACTOR_1;
    current_ac[1][cnt3] = (sum3 * ADC3_SCALE - CURRENT_OFFSET) * CURRENT_FACTOR;
    if (++cnt3 >= SAMPLING_3_LENS) {
    //   arm_rms_f32((const float *)voltage_ac[0], SAMPLING_3_LENS,
    //               &data->volatage[0]);
    //   data->volatage[0] += 0.075;
    //   arm_rms_f32((const float *)current_ac[1], SAMPLING_3_LENS,
    //               &data->current[1]);
      cnt3 = 0;
    }
  }
}
