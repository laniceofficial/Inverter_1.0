#include "sampling.hpp"
#include <cstdint>

extern "C" {
#include "adc.h"
#include "task_cpp.h"
}
float VVV = 0.0f;
float III = 0.0f;
// uint32_t tes = 0;
#include <cstring>

namespace App
{
namespace
{

constexpr float kVoltageRef = 3.306f;
constexpr float kAdcMaxValue = 4095.0f;

constexpr uint8_t kAdc2DmaChannelCount = 1U;
constexpr uint16_t kAdc2SampleRepeat = 4U;
constexpr uint16_t kAdc2DmaLength = kAdc2DmaChannelCount * kAdc2SampleRepeat;

constexpr uint8_t kAdc3DmaChannelCount = 2U;
constexpr uint16_t kAdc3SampleRepeat = 2U;
constexpr uint16_t kAdc3DmaLength = kAdc3DmaChannelCount * kAdc3SampleRepeat;

constexpr uint8_t kAdc4DmaChannelCount = 3U;
constexpr uint16_t kAdc4SampleRepeat = 4U;
constexpr uint16_t kAdc4DmaLength = kAdc4DmaChannelCount * kAdc4SampleRepeat;

constexpr uint8_t kAskChannelIndex = static_cast<uint8_t>(SampleChannel::Ask);
constexpr uint8_t kTransmitterVoltageIndex = static_cast<uint8_t>(SampleChannel::TransmitterVoltage);
constexpr uint8_t kTransmitterCurrentIndex = static_cast<uint8_t>(SampleChannel::TransmitterCurrent);
constexpr uint8_t kHalfBridgeInputVoltageIndex = static_cast<uint8_t>(SampleChannel::HalfBridgeInputVoltage);
constexpr uint8_t kHalfBridgeCurrentIndex = static_cast<uint8_t>(SampleChannel::HalfBridgeCurrent);
constexpr uint8_t kHalfBridgeOutputVoltageIndex = static_cast<uint8_t>(SampleChannel::HalfBridgeOutputVoltage);

constexpr float kPowerSampleFilterAlpha = 0.25f;
constexpr uint16_t kHalfBridgeInputVoltageFilterWindow = 25U;

// 发射端 ADC3 标定公式：工程量 = (raw平均值 - bias) * gain。
// 当前先等效沿用旧的电压换算系数，后续可用万用表/电流表实测点重新修正 bias 和 gain。
constexpr float kTransmitterVoltageRawGain = 15.0f / 1667.0f;
constexpr float kTransmitterVoltageRawBias = 0.0f;
constexpr float kTransmitterCurrentRawGain = 0.55f / (419.0f - 270.0f);
constexpr float kTransmitterCurrentRawBias = 240.0f;

} // namespace
//1I,2V
SamplingService& samplingService()
{
    // 全局唯一采样服务，避免裸全局对象初始化顺序问题。
    static SamplingService instance;
    return instance;
}

void SamplingService::init()
{
    std::memset(adc2Data_, 0, sizeof(adc2Data_));
    std::memset(adc3Data_, 0, sizeof(adc3Data_));
    std::memset(adc4Data_, 0, sizeof(adc4Data_));
    std::memset(askAdcData_, 0, sizeof(askAdcData_));
    resetAdc3RawWindow();
    transmitterVoltage_ = 0.0f;
    transmitterCurrent_ = 0.0f;
    halfBridgeInputVoltage_ = 0.0f;
    halfBridgeCurrent_ = 0.0f;
    halfBridgeOutputVoltage_ = 0.0f;
    transmitterVoltageFilter_.init(kPowerSampleFilterAlpha, 0.0f);
    transmitterCurrentFilter_.init(kPowerSampleFilterAlpha, 0.0f);
    halfBridgeInputVoltageFilter_.init(kHalfBridgeInputVoltageFilterWindow, 0.0f);
    halfBridgeCurrentFilter_.init(kPowerSampleFilterAlpha, 0.0f);
    halfBridgeOutputVoltageFilter_.init(kPowerSampleFilterAlpha, 0.0f);
    resetAskValid();

    static const Driver::AdcChannelConfig adc2Channels[] = {
        // ADC2 只采 ASK 原始波形，scale/offset 保持 1:1。
        {kAskChannelIndex, 0U, 1.0f, 0.0f},
    };

    static const Driver::AdcChannelConfig adc3Channels[] = {
        // ADC3 顺序由 Core/Src/adc.c 决定：Rank1 CH5 为发射端电压，Rank2 CH12 为发射端电流。
        {kTransmitterCurrentIndex, 0U, 0.73F, 0.2f},
        {kTransmitterVoltageIndex, 1U, 1.0f, 0.0f},
    };

    static const Driver::AdcChannelConfig adc4Channels[] = {
        // ADC4 顺序由 Core/Src/adc.c 决定：Rank1 CH3、Rank2 CH4、Rank3 CH5。
        {kHalfBridgeInputVoltageIndex, 0U, 30.814f, -0.0664f},
        {kHalfBridgeCurrentIndex, 1U, 9.0909f, -1.8094f},
        {kHalfBridgeOutputVoltageIndex, 2U, 10.919f, -0.0731f},
    };

    Driver::AdcSamplerConfig adc2Config;
    adc2Config.hadc = &hadc2;
    adc2Config.dmaBuffer = adc2Data_;
    adc2Config.dmaLength = kAdc2DmaLength;
    adc2Config.dmaChannelCount = kAdc2DmaChannelCount;
    adc2Config.sampleRepeat = kAdc2SampleRepeat;
    adc2Config.channels = adc2Channels;
    adc2Config.channelCount = static_cast<uint8_t>(sizeof(adc2Channels) / sizeof(adc2Channels[0]));
    adc2Config.vref = kVoltageRef;
    adc2Config.adcMaxValue = kAdcMaxValue;
    adc2Sampler_.init(adc2Config);

    Driver::AdcSamplerConfig adc3Config;
    adc3Config.hadc = &hadc3;
    adc3Config.dmaBuffer = adc3Data_;
    adc3Config.dmaLength = kAdc3DmaLength;
    adc3Config.dmaChannelCount = kAdc3DmaChannelCount;
    adc3Config.sampleRepeat = kAdc3SampleRepeat;
    adc3Config.channels = adc3Channels;
    adc3Config.channelCount = static_cast<uint8_t>(sizeof(adc3Channels) / sizeof(adc3Channels[0]));
    adc3Config.vref = kVoltageRef;
    adc3Config.adcMaxValue = kAdcMaxValue;
    adc3Sampler_.init(adc3Config);

    Driver::AdcSamplerConfig adc4Config;
    adc4Config.hadc = &hadc4;
    adc4Config.dmaBuffer = adc4Data_;
    adc4Config.dmaLength = kAdc4DmaLength;
    adc4Config.dmaChannelCount = kAdc4DmaChannelCount;
    adc4Config.sampleRepeat = kAdc4SampleRepeat;
    adc4Config.channels = adc4Channels;
    adc4Config.channelCount = static_cast<uint8_t>(sizeof(adc4Channels) / sizeof(adc4Channels[0]));
    adc4Config.vref = kVoltageRef;
    adc4Config.adcMaxValue = kAdcMaxValue;
    adc4Sampler_.init(adc4Config);

    while (!adc2Sampler_.calibrate())
    {
    }
    while (!adc3Sampler_.calibrate())
    {
    }
    while (!adc4Sampler_.calibrate())
    {
    }

    askDecoder_.init(askAdcData_, kAdc2SampleRepeat);
    halfBridgeController_.init();
    start();
}

void SamplingService::start()
{
    adc2Sampler_.start();
    adc3Sampler_.start();
    adc4Sampler_.start();
}

void SamplingService::stop()
{
    adc2Sampler_.stop();
    adc3Sampler_.stop();
    adc4Sampler_.stop();
}

void SamplingService::resetAskValid()
{
    std::memset(askAdcData_, 0, sizeof(askAdcData_));
    askWriteIndex_ = 0U;
    askRawVoltage_ = 0.0f;
    askDecoder_.init(askAdcData_, kAdc2SampleRepeat);
}

bool SamplingService::handleAdcConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc == &hadc3)
    {
        if (!adc3Sampler_.processDmaBuffer())
        {
            return false;
        }

        processAdc3(adc3Sampler_);
        return true;
    }

    if (hadc == &hadc4)
    {
        if (!adc4Sampler_.processDmaBuffer())
        {
            return false;
        }

        processAdc4(adc4Sampler_);
        return true;
    }

    return false;
}

bool SamplingService::isAskValid() const
{
    return askDecoder_.isValid();
}



float SamplingService::getAskRawVoltage() const
{
    return askRawVoltage_;
}

float SamplingService::getTransmitterVoltage() const
{
    return transmitterVoltage_;
}

float SamplingService::getTransmitterCurrent() const
{
    return transmitterCurrent_;
}
float SamplingService::getTransmitterPower() const
{
    return transmitterPower;
}

float SamplingService::getTransmitterVoltageRawAverage() const
{
    return transmitterVoltageRawAverage_;
}

float SamplingService::getTransmitterCurrentRawAverage() const
{
    return transmitterCurrentRawAverage_;
}

float SamplingService::getHalfBridgeInputVoltage() const
{
    return halfBridgeInputVoltage_;
}

float SamplingService::getHalfBridgeCurrent() const
{
    return halfBridgeCurrent_;
}

float SamplingService::getHalfBridgeOutputVoltage() const
{
    return halfBridgeOutputVoltage_;
}

HalfBridgeController& SamplingService::getHalfBridgeController()
{
    return halfBridgeController_;
}

void SamplingService::processAdc2(Driver::AdcSampler& sampler)
{
    for (uint16_t sampleIndex = 0U; sampleIndex < kAdc2SampleRepeat; ++sampleIndex)
    {
        // 当前 ADC2 只有一个通道，取最后一个通道偏移可兼容后续扩展。
        const uint16_t rawSample = sampler.getRawSample(sampleIndex, sampler.getDmaChannelCount() - 1U);
        feedAskBuffer(rawSample, sampler.getSampleRepeat());
    }
}

void SamplingService::processAdc3(Driver::AdcSampler& sampler)
{
    // tes++;
    processAdc2(adc2Sampler_);
    const uint16_t voltageRaw = sampler.getRawAverage(kTransmitterVoltageIndex);
    const uint16_t currentRaw = sampler.getRawAverage(kTransmitterCurrentIndex);
    updateAdc3RawWindow(voltageRaw, currentRaw);

    // 调试变量直接暴露 ADC raw 平均码值，方便用 J-Link/Ozone 和外部仪表做 bias/gain 标定。
    // VVV = transmitterVoltageRawAverage_;
    // III = transmitterCurrentRawAverage_;

    const float voltage = applyRawCalibration(
        transmitterVoltageRawAverage_, kTransmitterVoltageRawBias, kTransmitterVoltageRawGain);
    // const float current = applyRawCalibration(
    //     transmitterCurrentRawAverage_, kTransmitterCurrentRawBias, kTransmitterCurrentRawGain);
    const float current = (float)currentRaw /4095*3.3*0.25+0.2;
    transmitterVoltage_ = transmitterVoltageFilter_.update(voltage);
    transmitterCurrent_ = transmitterCurrentFilter_.update(current);
    transmitterPower = transmitterVoltage_ * transmitterCurrent_;
}

void SamplingService::resetAdc3RawWindow()
{
    std::memset(adc3RawWindow_, 0, sizeof(adc3RawWindow_));
    std::memset(adc3RawWindowSum_, 0, sizeof(adc3RawWindowSum_));
    adc3RawWindowWriteIndex_ = 0U;
    adc3RawWindowCount_ = 0U;
    transmitterVoltageRawAverage_ = 0.0f;
    transmitterCurrentRawAverage_ = 0.0f;
}

void SamplingService::updateAdc3RawWindow(const uint16_t voltageRaw, const uint16_t currentRaw)
{
    adc3RawWindowSum_[0] -= adc3RawWindow_[0][adc3RawWindowWriteIndex_];
    adc3RawWindowSum_[1] -= adc3RawWindow_[1][adc3RawWindowWriteIndex_];

    adc3RawWindow_[0][adc3RawWindowWriteIndex_] = voltageRaw;
    adc3RawWindow_[1][adc3RawWindowWriteIndex_] = currentRaw;

    adc3RawWindowSum_[0] += voltageRaw;
    adc3RawWindowSum_[1] += currentRaw;

    if (adc3RawWindowCount_ < kAdc3RawWindowSize)
    {
        ++adc3RawWindowCount_;
    }

    ++adc3RawWindowWriteIndex_;
    if (adc3RawWindowWriteIndex_ >= kAdc3RawWindowSize)
    {
        adc3RawWindowWriteIndex_ = 0U;
    }

    const float divisor = static_cast<float>(adc3RawWindowCount_);
    transmitterVoltageRawAverage_ = static_cast<float>(adc3RawWindowSum_[0]) / divisor;
    transmitterCurrentRawAverage_ = static_cast<float>(adc3RawWindowSum_[1]) / divisor;
}

float SamplingService::applyRawCalibration(
    const float rawAverage, const float bias, const float gain)
{
    return (rawAverage - bias) * gain;
}

void SamplingService::processAdc4(Driver::AdcSampler& sampler)
{
    halfBridgeInputVoltage_ =
        halfBridgeInputVoltageFilter_.update(sampler.getValue(kHalfBridgeInputVoltageIndex));
    halfBridgeCurrent_ = halfBridgeCurrentFilter_.update(sampler.getValue(kHalfBridgeCurrentIndex));
    halfBridgeOutputVoltage_ =
        halfBridgeOutputVoltageFilter_.update(sampler.getValue(kHalfBridgeOutputVoltageIndex));

    halfBridgeController_.setFeedback(
        halfBridgeCurrent_, halfBridgeOutputVoltage_, halfBridgeInputVoltage_);

    const ChangeState_e state = GetNowState();
    if ((state == PreChange) || (state == Changing))
    {
        halfBridgeController_.powerLoop();
    }
}

void SamplingService::feedAskBuffer(const uint16_t rawSample, const uint16_t sampleRepeat)
{
    if (sampleRepeat == 0U)
    {
        return;
    }

    askAdcData_[askWriteIndex_] = rawSample;
    ++askWriteIndex_;

    if (askWriteIndex_ >= sampleRepeat)
    {
        // 凑满一个短窗口后立即解码，保证 ASK 边沿计时尽量贴近实时采样。
        askWriteIndex_ = 0U;
        askDecoder_.decode();
    }
}

} // namespace App
