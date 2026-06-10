#pragma once

#include "ask.hpp"
#include "bsp_adc.hpp"
// #include "halfbridge_controller.hpp"
#include "user_math.hpp"

#include <cstdint>

namespace App
{

// 采样服务内部结果通道：ADC2 负责 ASK，ADC3/ADC4 负责功率采样。
enum class SampleChannel : uint8_t
{
    Ask = 0U,
    TransmitterCurrent,
    TransmitterVoltage,
    Count,
};

// 汇总 ADC DMA、ASK 解码和半桥反馈，给主任务提供统一采样入口。
class SamplingService
{
public:
    void init();
    void start();
    void stop();
    void resetAskValid();
    bool handleAdcConvCpltCallback(ADC_HandleTypeDef* hadc);
    bool isAskValid() const;
    bool isRequirePower() const;
    float getAskRawVoltage() const;
    float getTransmitterVoltage() const;
    float getTransmitterCurrent() const;
    float getTransmitterPower() const;
    float getTransmitterVoltageRawAverage() const;
    float getTransmitterCurrentRawAverage() const;

private:
    static constexpr uint8_t kAdc3PowerChannelCount = 2U;
    // 电压/电流独立滑动窗口大小，可根据响应速度与 ASK 抑制需求分别调节
    // N=30 → 30kHz/30=1kHz null（消除 1kbps ASK）, N=60→500Hz null
    static constexpr uint8_t kVoltageRawWindowSize = 36U;
    static constexpr uint8_t kCurrentRawWindowSize = 120U;

    void processAdc2(Driver::AdcSampler& sampler);
    void processAdc3(Driver::AdcSampler& sampler);

    void feedAskBuffer(uint16_t rawSample, uint16_t sampleRepeat);
    void resetAdc3RawWindow();
    static float applyRawCalibration(float rawAverage, float bias, float gain);

    uint16_t adc2Data_[4] = {};
    uint16_t adc3Data_[8] = {};
    // uint16_t askAdcData_[4] = {};
    Driver::AdcSampler adc2Sampler_;
    Driver::AdcSampler adc3Sampler_;
    Driver::AskDecoder askDecoder_;
    Driver::FirstOrderLpf transmitterVoltageFilter_;
    static constexpr float kNotchFreq = 1000.0f;  // ASK 1kbps 载波频率
    static constexpr float kNotchQ = 5.0f;        // Q=5, ±100Hz 带宽

    Driver::FirstOrderLpf transmitterCurrentFilter_;
    Driver::SlidingWindowU16 voltageRawWindow_;
    Driver::SlidingWindowU16 currentRawWindow_;
    Driver::NotchFilter currentNotchFilter_;

    float transmitterVoltageRawAverage_ = 0.0f;
    float transmitterCurrentRawAverage_ = 0.0f;
    
    uint8_t askWriteIndex_ = 0U;
    float transmitterVoltage_ = 0.0f;
    float transmitterCurrent_ = 0.0f;
    float transmitterPower = 0.0f;
};

SamplingService& samplingService();

} // namespace App
