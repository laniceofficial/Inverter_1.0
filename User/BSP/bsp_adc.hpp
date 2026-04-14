#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "adc.h"
#include "main.h"

#ifdef __cplusplus
}
#endif

#if defined(HAL_ADC_MODULE_ENABLED)

// #include <cstdint>

namespace BSP_n
{

constexpr uint16_t ADC_MAX_NUM = 32U;
constexpr uint8_t ADC_MAX_CHANNEL_NUM = 16U;
constexpr uint8_t ADC_MAX_INSTANCE_NUM = 8U;

class ADC_c;

using ADC_Callback_t = void (*)(ADC_c *instance);

struct ADC_ChannelConfig_s
{
    uint8_t result_index = 0U;
    uint8_t sample_offset = 0U;
    float scale = 1.0f;
    float offset = 0.0f;
};

struct ADC_InitConfig_s
{
    ADC_HandleTypeDef *hadc = nullptr;
    uint16_t *dma_buffer = nullptr;
    uint16_t dma_length = 0U;
    uint8_t dma_channel_count = 0U;
    uint16_t sample_repeat = 1U;
    const ADC_ChannelConfig_s *channels = nullptr;
    uint8_t channel_count = 0U;
    float vref = 3.306f;
    float adc_max_value = 4095.0f;
    bool calibrate_on_start = false;
    bool disable_half_transfer_it = true;
    ADC_Callback_t callback = nullptr;
};

class ADC_c
{
public:
    ADC_c();
    ADC_c(ADC_HandleTypeDef *hadc, uint16_t adc_length);
    explicit ADC_c(const ADC_InitConfig_s &config);

    void Init(ADC_HandleTypeDef *hadc, uint16_t adc_length);
    void Init(const ADC_InitConfig_s &config);

    bool Calibrate();
    bool Start();
    void Stop();

    bool ProcessDmaBuffer();
    bool ProcessDmaBuffer(const uint16_t *dma_buffer, uint16_t dma_length);

    inline float GetChannelValue(uint8_t result_index) const;
    inline uint16_t GetRawAverage(uint8_t result_index) const;
    inline uint16_t *GetDmaBuffer();
    inline const uint16_t *GetDmaBuffer() const;
    inline uint16_t GetDmaLength() const;
    inline uint32_t adc_value(void);

    inline void SetCallback(ADC_Callback_t callback);
    static bool HandleConvCpltCallback(ADC_HandleTypeDef *hadc);

private:
    void ClearResults();
    bool IsStartConfigValid() const;
    bool IsProcessConfigValid(const uint16_t *dma_buffer, uint16_t dma_length) const;
    bool RegisterInstance();
    void UnregisterInstance();
    static ADC_c *FindInstance(const ADC_HandleTypeDef *hadc);

    ADC_HandleTypeDef *hadc_ = nullptr;
    uint16_t internal_buffer_[ADC_MAX_NUM] = {};
    uint16_t *dma_buffer_ = internal_buffer_;
    uint16_t dma_length_ = 0U;
    uint8_t dma_channel_count_ = 0U;
    uint16_t sample_repeat_ = 1U;
    const ADC_ChannelConfig_s *channels_ = nullptr;
    uint8_t channel_count_ = 0U;
    float vref_ = 3.306f;
    float adc_max_value_ = 4095.0f;
    bool calibrate_on_start_ = false;
    bool disable_half_transfer_it_ = true;
    ADC_Callback_t callback_ = nullptr;
    float channel_values_[ADC_MAX_CHANNEL_NUM] = {};
    uint16_t raw_average_[ADC_MAX_CHANNEL_NUM] = {};

    static ADC_c *instances_[ADC_MAX_INSTANCE_NUM];
};

inline float ADC_c::GetChannelValue(uint8_t result_index) const
{
    if (result_index >= ADC_MAX_CHANNEL_NUM)
    {
        return 0.0f;
    }

    return channel_values_[result_index];
}

inline uint16_t ADC_c::GetRawAverage(uint8_t result_index) const
{
    if (result_index >= ADC_MAX_CHANNEL_NUM)
    {
        return 0U;
    }

    return raw_average_[result_index];
}

inline uint16_t *ADC_c::GetDmaBuffer()
{
    return dma_buffer_;
}

inline const uint16_t *ADC_c::GetDmaBuffer() const
{
    return dma_buffer_;
}

inline uint16_t ADC_c::GetDmaLength() const
{
    return dma_length_;
}

inline uint32_t ADC_c::adc_value(void)
{
    return static_cast<uint32_t>(reinterpret_cast<std::uintptr_t>(dma_buffer_));
}

inline void ADC_c::SetCallback(ADC_Callback_t callback)
{
    callback_ = callback;
}

} // namespace BSP_n

#endif // HAL_ADC_MODULE_ENABLED
