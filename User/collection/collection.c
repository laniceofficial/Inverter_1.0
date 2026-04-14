#include "collection.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "ask.h"
#include "halfbridge_ctrl.h"
#include "task.h"
#include "stm32g4xx_hal_adc.h"

/*
 * ADC 通道与业务量的大致对应关系：
 * ADC2: 当前主要使用 1 路输入侧源电压采样，同时为 ASK 提供原始采样值
 * ADC3: 预留/兼容原始流程，当前回调里仅保留电源环调用
 * ADC4: 3 路序列采样，分别对应半桥输入电压、电流反馈、输出电压
 */
#define ADC2_GROUP_DMA_LENGTH 12U
#define ADC2_GROUP_CHANNEL_COUNT 3U
#define ADC2_GROUP_SAMPLE_REPEAT 4U

#define ADC3_GROUP_DMA_LENGTH 8U
#define ADC3_GROUP_CHANNEL_COUNT 2U
#define ADC3_GROUP_SAMPLE_REPEAT 4U

#define ADC4_GROUP_DMA_LENGTH 12U
#define ADC4_GROUP_CHANNEL_COUNT 3U
#define ADC4_GROUP_SAMPLE_REPEAT 4U

#define ADC5_GROUP_DMA_LENGTH 4U
#define ADC5_GROUP_CHANNEL_COUNT 1U
#define ADC5_GROUP_SAMPLE_REPEAT 4U


#define ASK_ADC_BUFFER_LENGTH ADC2_GROUP_SAMPLE_REPEAT

/* 源电压换算延续旧逻辑：先转电压，再减基准偏置，最后乘分压系数。 */
#define SOURCE_VOLTAGE_OFFSET 1.22f
#define SOURCE_VOLTAGE_FACTOR 17.241f

_Static_assert(ADC2_GROUP_DMA_LENGTH == (ADC2_GROUP_CHANNEL_COUNT * ADC2_GROUP_SAMPLE_REPEAT),
               "ADC2 group config mismatch");
_Static_assert(ADC3_GROUP_DMA_LENGTH == (ADC3_GROUP_CHANNEL_COUNT * ADC3_GROUP_SAMPLE_REPEAT),
               "ADC3 group config mismatch");
_Static_assert(ADC4_GROUP_DMA_LENGTH == (ADC4_GROUP_CHANNEL_COUNT * ADC4_GROUP_SAMPLE_REPEAT),
               "ADC4 group config mismatch");
_Static_assert(ADC5_GROUP_DMA_LENGTH == (ADC5_GROUP_CHANNEL_COUNT * ADC5_GROUP_SAMPLE_REPEAT),
               "ADC5 group config mismatch");
typedef struct collection_adc_group collection_adc_group_t;

typedef void (*collection_group_post_process_fn)(const collection_adc_group_t *group);

typedef struct
{
    /* 逻辑通道描述：一个逻辑量如何从 DMA 序列中取样、换算和滤波。 */
    collection_channel_id_t id;
    uint8_t sample_offset; // 在 ADC 序列中的通道偏移
    float scale; // ADC 电压平均值到工程量的增益
    float offset; // ADC 电压平均值到工程量的偏移
    collection_filter_mode_t filter_mode; // 当前通道选择的滤波模式
    void *filter_state; // 指向对应滤波器状态结构体
} collection_channel_cfg_t;

struct collection_adc_group
{
    /*
     * 一个 ADC 组对应一个 DMA buffer、一组逻辑通道配置，
     * 以及该组采样完成后的业务后处理函数。
     */
    ADC_HandleTypeDef *hadc;
    uint16_t *dma_buffer;
    uint16_t dma_length;
    uint8_t dma_channel_count;
    uint16_t sample_repeat;
    const collection_channel_cfg_t *channel_cfg;
    uint8_t channel_cfg_count;
    collection_group_post_process_fn post_process;
};

static uint16_t adc2_data[ADC2_GROUP_DMA_LENGTH];
static uint16_t adc3_data[ADC3_GROUP_DMA_LENGTH];
static uint16_t adc4_data[ADC4_GROUP_DMA_LENGTH];
static uint16_t adc5_data[ADC5_GROUP_DMA_LENGTH];
static uint16_t ask_adc_data[ASK_ADC_BUFFER_LENGTH];

static ask_com_t g_ask_com;
static HalfBridge_ctrl_t g_halfbridge_ctrl;

static float g_channel_values[COLLECTION_CH_COUNT];
static uint16_t g_channel_raw_avg[COLLECTION_CH_COUNT];

static Recursive_ave_filter_type_t g_source_avg_filter;
static FirstOrderLPF g_source_lpf;
static Recursive_ave_filter_type_t g_halfbridge_in_avg_filter;
static FirstOrderLPF g_halfbridge_i_lpf;
static FirstOrderLPF g_halfbridge_out_v_lpf;

static uint8_t g_allow_ask = 0U;
static uint8_t g_ask_write_index = 0U;

static float g_cal_without_base_v = 0.0f;

static float collection_apply_filter(const collection_channel_cfg_t *cfg, float input);
static void collection_process_group(const collection_adc_group_t *group);
static const collection_adc_group_t *collection_find_group(const ADC_HandleTypeDef *hadc);
static uint8_t collection_validate_group(const collection_adc_group_t *group);
static void collection_sum_samples(const collection_adc_group_t *group, uint32_t *sums);
static void collection_feed_ask_buffer(uint16_t raw_sample, uint16_t sample_repeat);
static void collection_post_process_adc2(const collection_adc_group_t *group);
static void collection_post_process_adc3(const collection_adc_group_t *group);
static void collection_post_process_adc4(const collection_adc_group_t *group);

/*
 * 按通道描述 ADC2 的业务量：
 * 1. 同一份原始采样可以同时生成“低通结果”和“递推平均结果”
 * 2. 每个通道都以“原始平均 ADC 电压”为输入，再做线性换算和滤波
 */
static const collection_channel_cfg_t g_adc2_channels[] = { //-ch4=V ，ch5=I，ch11=V
    // {
    //     COLLECTION_CH_SOURCE_VOLTAGE,
    //     0U,
    //     1.0f,
    //     0.0f,
    //     COLLECTION_FILTER_LPF,
    //     &g_source_lpf,
    // },
    // {
    //     COLLECTION_CH_SOURCE_VOLTAGE_AVG,
    //     0U,
    //     1.0f,
    //     0.0f,
    //     COLLECTION_FILTER_RECURSIVE_AVG,
    //     &g_source_avg_filter,
    // },
    {
        COLLECTION_CH_HALFBRIDGE_IN_V, 
        0U,
        30.814f,
        -0.0664f,
        COLLECTION_FILTER_RECURSIVE_AVG,
        &g_halfbridge_in_avg_filter,
    },
    {
        COLLECTION_CH_HALFBRIDGE_I,
        1U,
        9.0909f,
        -1.8094f,
        COLLECTION_FILTER_LPF,
        &g_halfbridge_i_lpf,
    },

    {
        COLLECTION_CH_HALFBRIDGE_OUT_V,
        2U,
        10.919f,
        -0.0731f,
        COLLECTION_FILTER_LPF,
        &g_halfbridge_out_v_lpf,
    },
};

/*
 * ADC4 是 3 通道交错序列：
 * 0 -> 半桥输入电压
 * 1 -> 半桥电流反馈
 * 2 -> 半桥输出电压
 * scale/offset 直接编码旧工程里的线性标定关系。
 */
static const collection_channel_cfg_t g_adc4_channels[] = { //ch4=V -ch5=I，ch3=V
    {
        COLLECTION_CH_HALFBRIDGE_IN_V,
        0U,
        30.814f,
        -0.0664f,
        COLLECTION_FILTER_RECURSIVE_AVG,
        &g_halfbridge_in_avg_filter,
    },
    {
        COLLECTION_CH_HALFBRIDGE_I,
        1U,
        9.0909f,
        -1.8094f,
        COLLECTION_FILTER_LPF,
        &g_halfbridge_i_lpf,
    },

    {
        COLLECTION_CH_HALFBRIDGE_OUT_V,
        2U,
        10.919f,
        -0.0731f,
        COLLECTION_FILTER_LPF,
        &g_halfbridge_out_v_lpf,
    },
};
// static const collection_channel_cfg_t g_adc5_channels[] = {
//     {
//         COLLECTION_CH_HALFBRIDGE_I,
//         0U,
//         9.0909f,
//         -1.8094f,
//         COLLECTION_FILTER_LPF,
//         &g_halfbridge_i_lpf,
//     },
// };
static const collection_adc_group_t g_adc_groups[] = {
    {
        &hadc2,
        adc2_data,
        ADC2_GROUP_DMA_LENGTH,
        ADC2_GROUP_CHANNEL_COUNT,
        ADC2_GROUP_SAMPLE_REPEAT,
        g_adc2_channels,
        (uint8_t)(sizeof(g_adc2_channels) / sizeof(g_adc2_channels[0])),
        collection_post_process_adc4,
    },
    // {
    //     &hadc3,
    //     adc3_data,
    //     ADC3_GROUP_DMA_LENGTH,
    //     ADC3_GROUP_CHANNEL_COUNT,
    //     ADC3_GROUP_SAMPLE_REPEAT,
    //     NULL,
    //     0U,
    //     collection_post_process_adc3,
    // },
    // {
    //     &hadc4,
    //     adc4_data,
    //     ADC4_GROUP_DMA_LENGTH,
    //     ADC4_GROUP_CHANNEL_COUNT,
    //     ADC4_GROUP_SAMPLE_REPEAT,
    //     g_adc4_channels,
    //     (uint8_t)(sizeof(g_adc4_channels) / sizeof(g_adc4_channels[0])),
    //     collection_post_process_adc4,
    // },
    // {
    //     &hadc5,
    //     adc5_data,
    //     ADC5_GROUP_DMA_LENGTH,
    //     ADC5_GROUP_CHANNEL_COUNT,
    //     ADC5_GROUP_SAMPLE_REPEAT,
    //     g_adc5_channels,
    //     (uint8_t)(sizeof(g_adc5_channels) / sizeof(g_adc5_channels[0])),
    // }
};

void collection_init(void)
{
    uint32_t i;

    /* 启动前统一清零，避免静态状态继承上次运行残留。 */

    memset(&g_ask_com, 0, sizeof(g_ask_com));
    memset(&g_halfbridge_ctrl, 0, sizeof(g_halfbridge_ctrl));
    memset(g_channel_values, 0, sizeof(g_channel_values));
    memset(g_channel_raw_avg, 0, sizeof(g_channel_raw_avg));
    memset(adc2_data, 0, sizeof(adc2_data));
    memset(adc3_data, 0, sizeof(adc3_data));
    memset(adc4_data, 0, sizeof(adc4_data));
    memset(ask_adc_data, 0, sizeof(ask_adc_data));

    g_allow_ask = 0U;
    g_ask_write_index = 0U;
    g_cal_without_base_v = 0.0f;

    /* ADC 组配置统一校验，避免 DMA 长度、通道数和重复次数不一致。 */
    for (i = 0U; i < (uint32_t)(sizeof(g_adc_groups) / sizeof(g_adc_groups[0])); ++i)
    {
        if (collection_validate_group(&g_adc_groups[i]) == 0U)
        {
            Error_Handler();
        }
    }

    /* ADC 上电后先校准，保持与旧版本一致。 */
    for (i = 0U; i < (uint32_t)(sizeof(g_adc_groups) / sizeof(g_adc_groups[0])); ++i)
    {
        while (HAL_ADCEx_Calibration_Start(g_adc_groups[i].hadc, ADC_SINGLE_ENDED) != HAL_OK)
            ;
    }

    /* 为不同业务量绑定各自滤波器：快环路偏向 LPF，慢变量可选递推平均。 */
    LPF_Init(&g_source_lpf, 0.25f, 0.0f);
    Recursive_ave_filter_init(&g_source_avg_filter, 20U, 0.0f);
    Recursive_ave_filter_init(&g_halfbridge_in_avg_filter, 20U, 0.0f);
    LPF_Init(&g_halfbridge_i_lpf, 0.25f, 0.0f);
    LPF_Init(&g_halfbridge_out_v_lpf, 0.25f, 0.0f);

    /* 初始化依赖模块：ASK 读取 ADC2 原始均值，HalfBridge 读取工程量反馈。 */
    ask_init(&g_ask_com, ask_adc_data, (uint8_t)ADC2_GROUP_SAMPLE_REPEAT);
    HalfBridge_init(&g_halfbridge_ctrl);

    /* 按组配置启动 DMA，并统一关闭半传输中断。 */
    for (i = 0U; i < (uint32_t)(sizeof(g_adc_groups) / sizeof(g_adc_groups[0])); ++i)
    {
        HAL_ADC_Start_DMA(g_adc_groups[i].hadc, (uint32_t *)g_adc_groups[i].dma_buffer, g_adc_groups[i].dma_length);
        __HAL_DMA_DISABLE_IT(g_adc_groups[i].hadc->DMA_Handle, DMA_IT_HT);
    }
}

void collection_start(void)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)(sizeof(g_adc_groups) / sizeof(g_adc_groups[0])); ++i)
    {
        HAL_ADC_Start_DMA(g_adc_groups[i].hadc, (uint32_t *)g_adc_groups[i].dma_buffer, g_adc_groups[i].dma_length);
        __HAL_DMA_DISABLE_IT(g_adc_groups[i].hadc->DMA_Handle, DMA_IT_HT);
    }
}

void collection_stop(void)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)(sizeof(g_adc_groups) / sizeof(g_adc_groups[0])); ++i)
    {
        HAL_ADC_Stop_DMA(g_adc_groups[i].hadc);
    }

    // HAL_ADC_Stop_DMA(&hadc5);
}

void collection_reset_ask_valid(void)
{
    memset(ask_adc_data, 0, sizeof(ask_adc_data));
    g_allow_ask = 0U;
    g_ask_write_index = 0U;
    ask_init(&g_ask_com, ask_adc_data, (uint8_t)ADC2_GROUP_SAMPLE_REPEAT);
}

float collection_get_channel_value(collection_channel_id_t channel)
{
    if ((uint32_t)channel >= COLLECTION_CH_COUNT)
    {
        return 0.0f;
    }

    return g_channel_values[channel];
}

uint16_t collection_get_channel_raw_average(collection_channel_id_t channel)
{
    if ((uint32_t)channel >= COLLECTION_CH_COUNT)
    {
        return 0U;
    }

    return g_channel_raw_avg[channel];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* 先按 ADC 组统一求和/换算/滤波，再执行该组的业务后处理。 */
    const collection_adc_group_t *group = collection_find_group(hadc);

    if (group == NULL)
    {
        return;
    }

    collection_process_group(group);

    if (group->post_process != NULL)
    {
        group->post_process(group);
    }
}

static float collection_apply_filter(const collection_channel_cfg_t *cfg, float input)
{
    if (cfg == NULL)
    {
        return input;
    }

    switch (cfg->filter_mode)
    {
        case COLLECTION_FILTER_NONE:
            return input;

        case COLLECTION_FILTER_LPF:
            return LPF_Update((FirstOrderLPF *)cfg->filter_state, input);

        case COLLECTION_FILTER_RECURSIVE_AVG:
            return Recursive_ave_filter((Recursive_ave_filter_type_t *)cfg->filter_state, input);

        default:
            return input;
    }
}

static void collection_process_group(const collection_adc_group_t *group)
{
    /*
     * 回调热路径的核心步骤：
     * 1. 先把同一逻辑通道的重复采样做求和
     * 2. 再转换成原始平均 ADC 电压
     * 3. 最后按配置表做工程量换算与滤波
     */
    uint8_t i;

    if ((group == NULL) || (group->dma_channel_count == 0U) || (group->sample_repeat == 0U))
    {
        return;
    }

    if ((group->channel_cfg == NULL) || (group->channel_cfg_count == 0U))
    {
        return;
    }

    {
        uint32_t sums[group->dma_channel_count];
        memset(sums, 0, sizeof(sums));

        collection_sum_samples(group, sums);

        for (i = 0U; i < group->channel_cfg_count; ++i)
        {
            const collection_channel_cfg_t *cfg = &group->channel_cfg[i];
            uint32_t raw_sum;
            float raw_avg;
            float adc_voltage;
            float converted;

            if (cfg->sample_offset >= group->dma_channel_count)
            {
                continue;
            }

            /* 同一个硬件通道可以映射成多个逻辑结果，例如低通版和平均版。 */
            raw_sum = sums[cfg->sample_offset];
            raw_avg = (float)raw_sum / (float)group->sample_repeat;
            adc_voltage = raw_avg * VOL_REF / ADC_MAX_VALUE;
            converted = adc_voltage * cfg->scale + cfg->offset;

            g_channel_raw_avg[cfg->id] = (uint16_t)raw_avg;
            g_channel_values[cfg->id] = collection_apply_filter(cfg, converted);
        }
    }
}

static const collection_adc_group_t *collection_find_group(const ADC_HandleTypeDef *hadc)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)(sizeof(g_adc_groups) / sizeof(g_adc_groups[0])); ++i)
    {
        if (g_adc_groups[i].hadc == hadc)
        {
            return &g_adc_groups[i];
        }
    }

    return NULL;
}

static uint8_t collection_validate_group(const collection_adc_group_t *group)
{
    uint8_t i;

    if ((group == NULL) || (group->hadc == NULL) || (group->dma_buffer == NULL))
    {
        return 0U;
    }

    if ((group->dma_length == 0U) || (group->dma_channel_count == 0U) || (group->sample_repeat == 0U))
    {
        return 0U;
    }

    if (group->dma_length != ((uint16_t)group->dma_channel_count * group->sample_repeat))
    {
        return 0U;
    }

    if ((group->channel_cfg_count > 0U) && (group->channel_cfg == NULL))
    {
        return 0U;
    }

    for (i = 0U; i < group->channel_cfg_count; ++i)
    {
        if (group->channel_cfg[i].sample_offset >= group->dma_channel_count)
        {
            return 0U;
        }
    }

    return 1U;
}

static void collection_sum_samples(const collection_adc_group_t *group, uint32_t *sums)
{
    /*
     * 按“重复采样次数 x 通道数”做通用求和，不依赖具体 ADC 实例、
     * 也不依赖通道数量常量。后续只要 DMA buffer 仍按
     * [ch0, ch1, ... chN-1, ch0, ch1, ...] 排列，就可以自动适配。
     */
    const uint16_t *buffer;
    uint8_t channel_count;
    uint16_t sample_index;
    uint8_t channel_index;

    if ((group == NULL) || (group->dma_buffer == NULL) || (group->dma_channel_count == 0U) ||
        (group->sample_repeat == 0U) || (sums == NULL))
    {
        return;
    }

    buffer = group->dma_buffer;
    channel_count = group->dma_channel_count;

    for (sample_index = 0U; sample_index < group->sample_repeat; ++sample_index)
    {
        const uint16_t base = (uint16_t)(sample_index * channel_count);

        for (channel_index = 0U; channel_index < channel_count; ++channel_index)
        {
            sums[channel_index] += buffer[base + channel_index];
        }
    }
}

static void collection_feed_ask_buffer(uint16_t raw_sample, uint16_t sample_repeat)
{
    if (sample_repeat == 0U)
    {
        return;
    }

    ask_adc_data[g_ask_write_index] = raw_sample;
    g_ask_write_index++;

    if (g_ask_write_index >= sample_repeat)
    {
        g_ask_write_index = 0U;
        g_allow_ask = 1U;
        ASK_Decode(&g_ask_com);
    }
}

static void collection_post_process_adc2(const collection_adc_group_t *group)
{
    const float source_v = g_channel_values[COLLECTION_CH_SOURCE_VOLTAGE];

    if ((group == NULL) || (group->sample_repeat == 0U))
    {
        return;
    }

    /* ADC2: 更新源电压派生量和 ASK 解码缓冲。 */
    g_cal_without_base_v = (source_v - SOURCE_VOLTAGE_OFFSET) * SOURCE_VOLTAGE_FACTOR;
    collection_feed_ask_buffer(g_channel_raw_avg[COLLECTION_CH_SOURCE_VOLTAGE], group->sample_repeat);
}

static void collection_post_process_adc3(const collection_adc_group_t *group)
{
    (void)group;

    /* ADC3 目前保留原有调用节奏，后续接入通道配置时可直接扩展。 */
}
// uint32_t test_f=0;
static void collection_post_process_adc4(const collection_adc_group_t *group)
{
    ChangeState_e state;

    if (group == NULL)
    {
        return;
    }

    collection_feed_ask_buffer(g_channel_raw_avg[COLLECTION_CH_HALFBRIDGE_IN_V], group->sample_repeat);
    state = GetNowState();

    /* ADC4: 更新半桥控制环所需的 3 路反馈量，并立即驱动功率环。 */
    g_halfbridge_ctrl.current_feed = g_channel_values[COLLECTION_CH_HALFBRIDGE_I];
    g_halfbridge_ctrl.voltage_bat_feed = g_channel_values[COLLECTION_CH_HALFBRIDGE_IN_V];
    g_halfbridge_ctrl.voltage_cap_feed = g_channel_values[COLLECTION_CH_HALFBRIDGE_OUT_V];
    if ((state == PreChange) || (state == Changing))
    {
        HB_PowerLoop(&g_halfbridge_ctrl);
    }


    if (g_allow_ask)
    {
        g_allow_ask = 0U;
    }
}

uint8_t get_ask_valid(void)
{
    return g_ask_com.isValid;
}
// static void collection_post_process_adc5(const collection_adc_group_t *group)
// {
//     (void)group;


//     }
