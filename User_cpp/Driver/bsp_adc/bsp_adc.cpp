#include "bsp_adc.hpp"

#if defined(HAL_ADC_MODULE_ENABLED)

namespace Driver
{

AdcSampler* AdcSampler::instances_[kAdcMaxInstanceCount] = {};

void AdcSampler::init(const AdcSamplerConfig& config)
{
    unregisterInstance();

    hadc_ = config.hadc;
    dmaBuffer_ = config.dmaBuffer;
    dmaLength_ = config.dmaLength;
    dmaChannelCount_ = config.dmaChannelCount;
    sampleRepeat_ = config.sampleRepeat;
    channels_ = config.channels;
    channelCount_ = (config.channelCount > kAdcMaxChannelCount) ? kAdcMaxChannelCount : config.channelCount;
    vref_ = config.vref;
    adcMaxValue_ = config.adcMaxValue;
    calibrateOnStart_ = config.calibrateOnStart;
    disableHalfTransferInterrupt_ = config.disableHalfTransferInterrupt;
    callback_ = config.callback;

    for (uint8_t i = 0U; i < kAdcMaxChannelCount; ++i)
    {
        values_[i] = 0.0f;
        voltages_[i] = 0.0f;
        rawAverages_[i] = 0U;
    }
}

bool AdcSampler::calibrate()
{
    if (hadc_ == nullptr)
    {
        return false;
    }

    return HAL_ADCEx_Calibration_Start(hadc_, ADC_SINGLE_ENDED) == HAL_OK;
}

bool AdcSampler::start()
{
    if (!isStartConfigValid())
    {
        return false;
    }

    if (calibrateOnStart_ && !calibrate())
    {
        return false;
    }

    if (!registerInstance())
    {
        return false;
    }

    if (HAL_ADC_Start_DMA(hadc_, reinterpret_cast<uint32_t*>(dmaBuffer_), dmaLength_) != HAL_OK)
    {
        unregisterInstance();
        return false;
    }

    if (disableHalfTransferInterrupt_ && (hadc_->DMA_Handle != nullptr))
    {
        // 当前处理逻辑以完整 DMA 窗口为单位，关闭半传输中断可减少无用回调。
        __HAL_DMA_DISABLE_IT(hadc_->DMA_Handle, DMA_IT_HT);
    }

    return true;
}

void AdcSampler::stop()
{
    if (hadc_ != nullptr)
    {
        HAL_ADC_Stop_DMA(hadc_);
    }
    unregisterInstance();
}

bool AdcSampler::processDmaBuffer()
{
    if (!isProcessConfigValid())
    {
        return false;
    }

    bool processed = false;
    for (uint8_t channelIndex = 0U; channelIndex < channelCount_; ++channelIndex)
    {
        const AdcChannelConfig& channel = channels_[channelIndex];
        if ((channel.resultIndex >= kAdcMaxChannelCount) || (channel.sampleOffset >= dmaChannelCount_))
        {
            continue;
        }

        uint32_t rawSum = 0U;
        // DMA 排列为 [sample0_ch0, sample0_ch1, ..., sample1_ch0, ...]，按通道偏移取平均。
        for (uint16_t sampleIndex = 0U; sampleIndex < sampleRepeat_; ++sampleIndex)
        {
            rawSum += getRawSample(sampleIndex, channel.sampleOffset);
        }

        const float rawAverage = static_cast<float>(rawSum) / static_cast<float>(sampleRepeat_);
        const float voltage = rawAverage * vref_ / adcMaxValue_;

        rawAverages_[channel.resultIndex] = static_cast<uint16_t>(rawAverage);
        voltages_[channel.resultIndex] = voltage;
        values_[channel.resultIndex] = (voltage * channel.scale) + channel.offset;
        processed = true;
    }

    return processed;
}

float AdcSampler::getValue(const uint8_t index) const
{
    return (index < kAdcMaxChannelCount) ? values_[index] : 0.0f;
}

float AdcSampler::getVoltage(const uint8_t index) const
{
    return (index < kAdcMaxChannelCount) ? voltages_[index] : 0.0f;
}

uint16_t AdcSampler::getRawAverage(const uint8_t index) const
{
    return (index < kAdcMaxChannelCount) ? rawAverages_[index] : 0U;
}

uint16_t AdcSampler::getRawSample(const uint16_t sampleIndex, const uint8_t channelOffset) const
{
    if ((dmaBuffer_ == nullptr) || (channelOffset >= dmaChannelCount_) || (sampleIndex >= sampleRepeat_))
    {
        return 0U;
    }

    const uint16_t index = static_cast<uint16_t>((sampleIndex * dmaChannelCount_) + channelOffset);
    return (index < dmaLength_) ? dmaBuffer_[index] : 0U;
}

uint16_t AdcSampler::getSampleRepeat() const
{
    return sampleRepeat_;
}

uint8_t AdcSampler::getDmaChannelCount() const
{
    return dmaChannelCount_;
}

ADC_HandleTypeDef* AdcSampler::getHandle() const
{
    return hadc_;
}

bool AdcSampler::handleConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // HAL 只有 ADC 句柄，先反查注册过的 C++ 采样器实例。
    AdcSampler* sampler = findInstance(hadc);
    if (sampler == nullptr)
    {
        return false;
    }

    const bool processed = sampler->processDmaBuffer();
    if (processed && (sampler->callback_ != nullptr))
    {
        sampler->callback_(*sampler);
    }

    return processed;
}

bool AdcSampler::isStartConfigValid() const
{
    return (hadc_ != nullptr) && (dmaBuffer_ != nullptr) && (dmaLength_ != 0U) &&
           (dmaChannelCount_ != 0U) && (sampleRepeat_ != 0U) &&
           (dmaLength_ >= static_cast<uint16_t>(dmaChannelCount_ * sampleRepeat_));
}

bool AdcSampler::isProcessConfigValid() const
{
    if (!isStartConfigValid() || (channels_ == nullptr) || (channelCount_ == 0U) || (adcMaxValue_ <= 0.0f))
    {
        return false;
    }

    return true;
}

bool AdcSampler::registerInstance()
{
    if (hadc_ == nullptr)
    {
        return false;
    }

    for (auto& instance : instances_)
    {
        if ((instance == this) || ((instance != nullptr) && (instance->hadc_ == hadc_)))
        {
            instance = this;
            return true;
        }
    }

    for (auto& instance : instances_)
    {
        if (instance == nullptr)
        {
            instance = this;
            return true;
        }
    }

    return false;
}

void AdcSampler::unregisterInstance()
{
    for (auto& instance : instances_)
    {
        if (instance == this)
        {
            instance = nullptr;
        }
    }
}

AdcSampler* AdcSampler::findInstance(const ADC_HandleTypeDef* hadc)
{
    if (hadc == nullptr)
    {
        return nullptr;
    }

    for (auto* instance : instances_)
    {
        if ((instance != nullptr) && (instance->hadc_ == hadc))
        {
            return instance;
        }
    }

    return nullptr;
}

} // namespace Driver

#endif // HAL_ADC_MODULE_ENABLED
