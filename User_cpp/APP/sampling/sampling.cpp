#include "sampling.hpp"
#include "config.hpp"
#include <cstdint>
#include "stm32g4xx.h"
#include "stm32g4xx_hal_gpio.h"

extern "C" {
#include "adc.h"
#include "task_cpp.h"
}
// float VVV = 0.0f;
// float III = 0.0f;
// float V_O=0.0f;
// uint32_t tes = 0;
#include <cstring>

namespace App
{
namespace
{

constexpr float kVoltageRef = 2.500f;
constexpr float kAdcMaxValue = 4095.0f;


constexpr uint8_t kAdc4DmaChannelCount = 3U;
constexpr uint16_t kAdc4SampleRepeat = 4U;
constexpr uint16_t kAdc4DmaLength = kAdc4DmaChannelCount * kAdc4SampleRepeat;


constexpr uint8_t kHalfBridgeInputVoltageIndex = static_cast<uint8_t>(SampleChannel::HalfBridgeInputVoltage);
constexpr uint8_t kHalfBridgeCurrentIndex = static_cast<uint8_t>(SampleChannel::HalfBridgeCurrent);
constexpr uint8_t kHalfBridgeOutputVoltageIndex = static_cast<uint8_t>(SampleChannel::HalfBridgeOutputVoltage);

constexpr float kPowerSampleFilterAlpha = 0.3f;
constexpr float kVoltageFilAlpha = 0.7f;
constexpr uint16_t kHalfBridgeInputVoltageFilterWindow = 25U;



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
    std::memset(adc4Data_, 0, sizeof(adc4Data_));
    std::memset(askAdcData_, 0, sizeof(askAdcData_));
    halfBridgeInputVoltage_ = 0.0f;
    halfBridgeCurrent_ = 0.0f;
    halfBridgeOutputVoltage_ = 0.0f;
    // transmitterVoltageFilter_.init(kPowerSampleFilterAlpha, 0.0f);
    // transmitterCurrentFilter_.init(kPowerSampleFilterAlpha, 0.0f);
    halfBridgeInputVoltageFilter_.init(kHalfBridgeInputVoltageFilterWindow, 0.0f);
    halfBridgeCurrentFilter_.init(kPowerSampleFilterAlpha, 0.0f);
    halfBridgeOutputVoltageFilter_.init(kVoltageFilAlpha, 0.0f);

    static const Driver::AdcChannelConfig adc4Channels[] = {
        // ADC4 顺序由 Core/Src/adc.c 决定：Rank1 CH3、Rank2 CH4、Rank3 CH5。
        // RawToValue 模式：scale/offset 直接 map raw→工程量，跳过电压中间计算。

        {kHalfBridgeInputVoltageIndex, 0U,
         HB_InputVoltageGain, HB_InputVoltageBias, Driver::AdcCalMode::RawToValue},
        {kHalfBridgeOutputVoltageIndex, 1U,
         HB_OutputVoltageGain, HB_OutputVoltageBias, Driver::AdcCalMode::RawToValue},
        {kHalfBridgeCurrentIndex, 2U,
         HB_CurrentGain, HB_Currentbias, Driver::AdcCalMode::RawToValue},

    };

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


    while (!adc4Sampler_.calibrate())
    {
    }

    halfBridgeController_.init();

    // 控制环固定 4 kHz，与 ADC 采样率解耦
    lastControlLoopTick_ = DWT->CYCCNT;
    controlLoopPeriodCycles_ = SystemCoreClock / 4000U;

    start();
}

void SamplingService::start()
{
    // adc2Sampler_.start();
    // adc3Sampler_.start();
    adc4Sampler_.start();
}

void SamplingService::stop()
{
    adc4Sampler_.stop();
}



bool SamplingService::handleAdcConvCpltCallback(ADC_HandleTypeDef* hadc)
{

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




void SamplingService::processAdc4(Driver::AdcSampler& sampler)
{
    // VVV = sampler.getRawAverage(kHalfBridgeInputVoltageIndex); //*halfbridge_inV_gain + halfbridge_inV_bias;
    // III = sampler.getRawAverage(kHalfBridgeCurrentIndex) ;
    // V_O = sampler.getRawAverage(kHalfBridgeOutputVoltageIndex);// * OutputVoltageGain + OutputVoltageBias;
    // RawToValue 模式
    halfBridgeInputVoltage_ = halfBridgeInputVoltageFilter_.update(
        sampler.getValue(kHalfBridgeInputVoltageIndex));
    halfBridgeCurrent_ = halfBridgeCurrentFilter_.update(
        sampler.getValue(kHalfBridgeCurrentIndex));
    halfBridgeOutputVoltage_ = halfBridgeOutputVoltageFilter_.update(
        sampler.getValue(kHalfBridgeOutputVoltageIndex));

    halfBridgeController_.setFeedback(
        halfBridgeCurrent_, halfBridgeOutputVoltage_, halfBridgeInputVoltage_);

    // 控制环以固定 4kHz 运行，与 ADC 采样率（~96kHz）解耦，
    // 避免增量 PID 的有效增益随采样率放大导致占空比突变。
    const uint32_t now = DWT->CYCCNT;
    if ((now - lastControlLoopTick_) >= controlLoopPeriodCycles_)
    {
        lastControlLoopTick_ = now;
        // tes++;
        halfBridgeController_.powerLoop();
    }

}

} // namespace App

// ============================================================
// ADC4 模拟看门狗回调 — CH5 电容过压硬件保护
// 由 ADC4_IRQHandler → HAL_ADC_IRQHandler 调用
// ============================================================
// extern "C" void HAL_ADCEx_LevelOutOfWindow2Callback(ADC_HandleTypeDef *hadc)
// {

//     if (hadc->Instance == ADC4)
//     {
//         App::samplingService().getHalfBridgeController().hardwareOvpStop();
//     }
// }
