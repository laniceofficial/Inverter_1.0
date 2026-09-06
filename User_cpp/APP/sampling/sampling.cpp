#include "sampling.hpp"
#include <cstdint>

extern "C" {
#include "adc.h"
#include "hrtim.h"
#include "task_cpp.h"
}
#include "debug_capture.hpp"
// float VVV = 0.0f;
// float III = 0.0f;
// uint32_t tes = 0;
#include <cstring>

namespace App
{
namespace
{

constexpr uint8_t kAdc2DmaChannelCount = 1U;
constexpr uint16_t kAdc2SampleRepeat = 4U;
constexpr uint16_t kAdc2DmaLength = kAdc2DmaChannelCount * kAdc2SampleRepeat;

constexpr uint8_t kAdc3DmaChannelCount = 2U;
constexpr uint16_t kAdc3SampleRepeat = 4U;
constexpr uint16_t kAdc3DmaLength = kAdc3DmaChannelCount * kAdc3SampleRepeat;


constexpr uint8_t kAskChannelIndex = static_cast<uint8_t>(SampleChannel::Ask);
constexpr uint8_t kTransmitterVoltageIndex = static_cast<uint8_t>(SampleChannel::TransmitterVoltage);
constexpr uint8_t kTransmitterCurrentIndex = static_cast<uint8_t>(SampleChannel::TransmitterCurrent);
// constexpr uint8_t kTransmitterC = static_cast<uint8_t>(SampleChannel::TransmitterCurrentback);
constexpr float kVoltageSampleFilterAlpha = 0.7f;
constexpr float kCurrentSampleFilterAlpha = 0.1f; // 30kHz LPF fc≈1.1kHz, 配合 MA 零点消除 1kHz ASK


// 供 ASK 解码器回调获取发射功率 — 函数指针注入，避免 ask.cpp 依赖 sampling.hpp
static float getTransmitterPowerForAsk()
{
    return samplingService().getTransmitterPower();
}

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
    voltageRawWindow_.init(kVoltageRawWindowSize);
    currentRawWindow_.init(kCurrentRawWindowSize);
    currentNotchFilter_.init(30000.0f, kNotchFreq, kNotchQ);
    transmitterVoltage_ = 0.0f;
    transmitterCurrent_ = 0.0f;

    transmitterVoltageFilter_.init(kVoltageSampleFilterAlpha , 0.0f);
    transmitterCurrentFilter_.init(kCurrentSampleFilterAlpha, 0.0f);
    resetAskValid();

    static const Driver::AdcChannelConfig adc2Channels[] = {
        // ADC2 只采 ASK 原始波形，scale/offset 保持 1:1。
        {kAskChannelIndex, 0U, 1.0f, 0.0f},
    };

    static const Driver::AdcChannelConfig adc3Channels[] = {
        // ADC3 顺序由 Core/Src/adc.c 决定：Rank1 CH5 为发射端电LIU，Rank2 CH12 为发射端电压。
        {kTransmitterCurrentIndex, 0U, 1.0F, 0.0f, Driver::AdcCalMode::RawToValue},

        {kTransmitterVoltageIndex, 1U, 1.0f, 0.0f, Driver::AdcCalMode::RawToValue},


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

    while (!adc2Sampler_.calibrate())
    {
    }
    while (!adc3Sampler_.calibrate())
    {
    }

    askDecoder_.init(adc2Data_, kAdc2SampleRepeat);
    askDecoder_.setTransmitterPowerGetter(getTransmitterPowerForAsk);
    start();
}

void SamplingService::start()
{
    adc2Sampler_.start();
    adc3Sampler_.start();
}

void SamplingService::stop()
{
    adc2Sampler_.stop();
    adc3Sampler_.stop();
}

void SamplingService::resetAskValid()
{
    std::memset(adc2Data_, 0, sizeof(adc2Data_));
    askWriteIndex_ = 0U;
    askDecoder_.init(adc2Data_, kAdc2SampleRepeat);
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

    return false;
}

bool SamplingService::isAskValid() const
{
    return askDecoder_.isValid();
}

bool SamplingService::isRequirePower() const
{
    return askDecoder_.getBackwardData().requiredPowerSelection;
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



void SamplingService::processAdc2(Driver::AdcSampler& sampler)
{
    askDecoder_.decode();
    return;

}

void SamplingService::processAdc3(Driver::AdcSampler& sampler)
{
    // tes++;
    processAdc2(adc2Sampler_);
    const uint16_t voltageRaw = sampler.getRawAverage(kTransmitterVoltageIndex);
    const uint16_t currentRaw = sampler.getRawAverage(kTransmitterCurrentIndex);
    // VVV = static_cast<float>(voltageRaw);
    // III = static_cast<float>(currentRaw);
    transmitterCurrentRawAverage_ = currentRawWindow_.update(currentRaw);
    transmitterVoltageRawAverage_ = voltageRawWindow_.update(voltageRaw);

    const float voltage = applyRawCalibration(
        transmitterVoltageRawAverage_, TransmitterVoltageBias, TransmitterVoltageGain);
    const float current =
        applyRawCalibration(transmitterCurrentRawAverage_, TransmitterCurrentBias, TransmitterCurrentGain);
    const float currentNotched = currentNotchFilter_.update(current);

    transmitterVoltage_ = transmitterVoltageFilter_.update(voltage);
    transmitterCurrent_ = transmitterCurrentFilter_.update(currentNotched);
    transmitterPower = transmitterVoltage_ * transmitterCurrent_;

    // DebugCapture::feed(
    //     VVV, III, transmitterVoltage_, transmitterCurrent_, transmitterPower,
    //     askDecoder_.getBitBufferPointer(),
    //     askDecoder_.isValid() ? 1U : 0U);
}

void SamplingService::resetAdc3RawWindow()
{
    voltageRawWindow_.reset();
    currentRawWindow_.reset();
    transmitterVoltageRawAverage_ = 0.0f;
    transmitterCurrentRawAverage_ = 0.0f;
}


float SamplingService::applyRawCalibration(
    const float rawAverage, const float bias, const float gain)
{
    return (rawAverage * gain + bias);
}

} // namespace App
