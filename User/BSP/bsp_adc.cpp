#include "bsp_adc.hpp"

#if defined(HAL_ADC_MODULE_ENABLED)

namespace BSP_n
{
namespace
{
uint16_t LimitInternalLength(uint16_t length)
{
    if (length > ADC_MAX_NUM)
    {
        return ADC_MAX_NUM;
    }

    return length;
}

bool IsResultIndexValid(uint8_t result_index)
{
    return result_index < ADC_MAX_CHANNEL_NUM;
}
} // namespace

ADC_c *ADC_c::instances_[ADC_MAX_INSTANCE_NUM] = {nullptr};

ADC_c::ADC_c()
{
    ClearResults();
}

ADC_c::ADC_c(ADC_HandleTypeDef *hadc, uint16_t adc_length)
{
    Init(hadc, adc_length);
}

ADC_c::ADC_c(const ADC_InitConfig_s &config)
{
    Init(config);
}

void ADC_c::Init(ADC_HandleTypeDef *hadc, uint16_t adc_length)
{
    ADC_InitConfig_s config;

    config.hadc = hadc;
    config.dma_buffer = internal_buffer_;
    config.dma_length = LimitInternalLength(adc_length);
    config.dma_channel_count = 1U;
    config.sample_repeat = config.dma_length;

    Init(config);
}

void ADC_c::Init(const ADC_InitConfig_s &config)
{
    UnregisterInstance();

    hadc_ = config.hadc;
    dma_buffer_ = (config.dma_buffer != nullptr) ? config.dma_buffer : internal_buffer_;
    dma_length_ = (config.dma_buffer != nullptr) ? config.dma_length : LimitInternalLength(config.dma_length);
    dma_channel_count_ = config.dma_channel_count;
    sample_repeat_ = config.sample_repeat;
    channels_ = config.channels;
    channel_count_ = (config.channel_count > ADC_MAX_CHANNEL_NUM) ? ADC_MAX_CHANNEL_NUM : config.channel_count;
    vref_ = config.vref;
    adc_max_value_ = config.adc_max_value;
    calibrate_on_start_ = config.calibrate_on_start;
    disable_half_transfer_it_ = config.disable_half_transfer_it;
    callback_ = config.callback;

    ClearResults();
}

bool ADC_c::Calibrate()
{
    if (hadc_ == nullptr)
    {
        return false;
    }

    return HAL_ADCEx_Calibration_Start(hadc_, ADC_SINGLE_ENDED) == HAL_OK;
}

bool ADC_c::Start()
{
    if (!IsStartConfigValid())
    {
        return false;
    }

    if (calibrate_on_start_ && !Calibrate())
    {
        return false;
    }

    if (!RegisterInstance())
    {
        return false;
    }

    if (HAL_ADC_Start_DMA(hadc_, reinterpret_cast<uint32_t *>(dma_buffer_), dma_length_) != HAL_OK)
    {
        UnregisterInstance();
        return false;
    }

    if (disable_half_transfer_it_ && (hadc_->DMA_Handle != nullptr))
    {
        __HAL_DMA_DISABLE_IT(hadc_->DMA_Handle, DMA_IT_HT);
    }

    return true;
}

void ADC_c::Stop()
{
    if (hadc_ == nullptr)
    {
        return;
    }

    HAL_ADC_Stop_DMA(hadc_);
    UnregisterInstance();
}

bool ADC_c::ProcessDmaBuffer()
{
    return ProcessDmaBuffer(dma_buffer_, dma_length_);
}

bool ADC_c::ProcessDmaBuffer(const uint16_t *dma_buffer, uint16_t dma_length)
{
    bool processed = false;

    if (!IsProcessConfigValid(dma_buffer, dma_length))
    {
        return false;
    }

    for (uint8_t channel_index = 0U; channel_index < channel_count_; ++channel_index)
    {
        const ADC_ChannelConfig_s &channel = channels_[channel_index];
        uint32_t raw_sum = 0U;

        if (!IsResultIndexValid(channel.result_index) || (channel.sample_offset >= dma_channel_count_))
        {
            continue;
        }

        for (uint16_t sample_index = 0U; sample_index < sample_repeat_; ++sample_index)
        {
            const uint16_t sample_offset =
                static_cast<uint16_t>((sample_index * dma_channel_count_) + channel.sample_offset);
            raw_sum += dma_buffer[sample_offset];
        }

        const float raw_average = static_cast<float>(raw_sum) / static_cast<float>(sample_repeat_);
        const float adc_voltage = raw_average * vref_ / adc_max_value_;

        raw_average_[channel.result_index] = static_cast<uint16_t>(raw_average);
        channel_values_[channel.result_index] = (adc_voltage * channel.scale) + channel.offset;
        processed = true;
    }

    return processed;
}

void ADC_c::ClearResults()
{
    for (uint8_t index = 0U; index < ADC_MAX_CHANNEL_NUM; ++index)
    {
        channel_values_[index] = 0.0f;
        raw_average_[index] = 0U;
    }
}

bool ADC_c::IsStartConfigValid() const
{
    if ((hadc_ == nullptr) || (dma_buffer_ == nullptr) || (dma_length_ == 0U))
    {
        return false;
    }

    if ((channels_ == nullptr) || (channel_count_ == 0U))
    {
        return true;
    }

    return IsProcessConfigValid(dma_buffer_, dma_length_);
}

bool ADC_c::IsProcessConfigValid(const uint16_t *dma_buffer, uint16_t dma_length) const
{
    if ((dma_buffer == nullptr) || (channels_ == nullptr) || (channel_count_ == 0U) ||
        (dma_channel_count_ == 0U) || (sample_repeat_ == 0U) || (adc_max_value_ <= 0.0f))
    {
        return false;
    }

    const uint32_t required_length = static_cast<uint32_t>(dma_channel_count_) * sample_repeat_;
    return dma_length >= required_length;
}

bool ADC_c::RegisterInstance()
{
    if (hadc_ == nullptr)
    {
        return false;
    }

    for (uint8_t index = 0U; index < ADC_MAX_INSTANCE_NUM; ++index)
    {
        if ((instances_[index] == this) || ((instances_[index] != nullptr) && (instances_[index]->hadc_ == hadc_)))
        {
            instances_[index] = this;
            return true;
        }
    }

    for (uint8_t index = 0U; index < ADC_MAX_INSTANCE_NUM; ++index)
    {
        if (instances_[index] == nullptr)
        {
            instances_[index] = this;
            return true;
        }
    }

    return false;
}

void ADC_c::UnregisterInstance()
{
    for (uint8_t index = 0U; index < ADC_MAX_INSTANCE_NUM; ++index)
    {
        if (instances_[index] == this)
        {
            instances_[index] = nullptr;
        }
    }
}

ADC_c *ADC_c::FindInstance(const ADC_HandleTypeDef *hadc)
{
    if (hadc == nullptr)
    {
        return nullptr;
    }

    for (uint8_t index = 0U; index < ADC_MAX_INSTANCE_NUM; ++index)
    {
        if ((instances_[index] != nullptr) && (instances_[index]->hadc_ == hadc))
        {
            return instances_[index];
        }
    }

    return nullptr;
}

bool ADC_c::HandleConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    ADC_c *instance = FindInstance(hadc);

    if (instance == nullptr)
    {
        return false;
    }

    const bool processed = instance->ProcessDmaBuffer();
    if (processed && (instance->callback_ != nullptr))
    {
        instance->callback_(instance);
    }

    return processed;
}

} // namespace BSP_n

#endif // HAL_ADC_MODULE_ENABLED
