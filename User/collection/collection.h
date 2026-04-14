#ifndef COLLECTION_H
#define COLLECTION_H

#include "adc.h"

#define VOL_REF 3.306f                  // ADC 参考电压
#define ADC_MAX_VALUE 4095.0f           // 12bit ADC 满量程
#define COLLECTION_SAMPLE_REPEAT 4U     // 同一通道在一次 DMA 缓冲中的重复采样次数
#define COLLECTION_FILTER_WINDOW_MAX 50U // 递推平均滤波允许的最大窗口长度

#include "user_math.h"

#define CONTROL_SEND_HZ(HZ)    \
    {                          \
        static int16_t hz = 0; \
        hz++;                  \
        if (hz < (HZ))         \
            return;            \
        hz = 0;                \
    }

typedef enum
{
    COLLECTION_FILTER_NONE = 0,         // 原样输出，不做滤波
    COLLECTION_FILTER_LPF,              // 一阶低通
    COLLECTION_FILTER_RECURSIVE_AVG     // 递推平均 / 滑动平均
} collection_filter_mode_t;

typedef enum
{
    COLLECTION_CH_SOURCE_VOLTAGE = 0,   // 输入侧源电压，低通后结果
    COLLECTION_CH_SOURCE_VOLTAGE_AVG,   // 输入侧源电压，递推平均后结果
    COLLECTION_CH_HALFBRIDGE_IN_V,      // 半桥输入电压
    COLLECTION_CH_HALFBRIDGE_I,         // 半桥电流反馈
    COLLECTION_CH_HALFBRIDGE_OUT_V,     // 半桥输出电压
    COLLECTION_CH_COUNT
} collection_channel_id_t;

/* 初始化采样模块，完成 ADC 校准、DMA 启动和滤波器初始化。 */
void collection_init(void);
void collection_start(void);
/* 停止 ADC DMA 采样。 */
void collection_stop(void);
void collection_reset_ask_valid(void);
/* 将内部通道结果同步到对外公开的 collect_data_t。 */

/* 获取指定逻辑通道的工程量结果。 */
float collection_get_channel_value(collection_channel_id_t channel);
/* 获取指定逻辑通道的原始 ADC 平均值。 */
uint16_t collection_get_channel_raw_average(collection_channel_id_t channel);
uint8_t get_ask_valid(void);
#endif // COLLECTION_H
