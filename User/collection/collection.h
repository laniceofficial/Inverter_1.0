#ifndef COLLECTION_H
#define COLLECTION_H

#include "adc.h"
#include "user_math.h"

#define VOL_REF 3.306f
#define ADC_MAX_VALUE 4095.0f
#define COLLECTION_SAMPLE_REPEAT 4U
#define COLLECTION_FILTER_WINDOW_MAX 50U

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
    COLLECTION_FILTER_NONE = 0,
    COLLECTION_FILTER_LPF,
    COLLECTION_FILTER_RECURSIVE_AVG
} collection_filter_mode_t;

typedef enum
{
    // COLLECTION_CH_SOURCE_VOLTAGE = 0,
    // COLLECTION_CH_SOURCE_CURRENT,
    COLLECTION_ASK,
    // COLLECTION_CH_HALFBRIDGE_IN_V,      // 半桥输入电压
    // COLLECTION_CH_HALFBRIDGE_I,         // 半桥电流反馈
    // COLLECTION_CH_HALFBRIDGE_OUT_V,     // 半桥输出电压
    COLLECTION_CH_COUNT
} collection_channel_id_t;

void collection_init(void);
void collection_start(void);
void collection_stop(void);
void collection_reset_ask_valid(void);

float collection_get_channel_value(collection_channel_id_t channel);
uint16_t collection_get_channel_raw_average(collection_channel_id_t channel);
uint16_t collection_get_ask_raw_last(void);
float collection_get_ask_raw_voltage(void);
uint16_t collection_get_ask_raw_min(void);
uint16_t collection_get_ask_raw_max(void);
uint16_t collection_get_ask_raw_peak_to_peak(void);
uint8_t get_ask_valid(void);

#endif // COLLECTION_H
