#pragma once

#include "ask.hpp"
#include "bsp_adc.hpp"
#include "halfbridge_controller.hpp"
#include "user_math.hpp"

#include <cstdint>

namespace App
{

// 采样服务内部结果通道：ADC2 负责 ASK，ADC3/ADC4 负责功率采样。
enum class SampleChannel : uint8_t
{
    Ask = 0U,
    TransmitterVoltage,
    TransmitterCurrent,
    HalfBridgeInputVoltage,
    HalfBridgeCurrent,
    HalfBridgeOutputVoltage,
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
    uint16_t getAskRawLast() const;
    float getAskRawVoltage() const;
    uint16_t getAskRawMin() const;
    uint16_t getAskRawMax() const;
    uint16_t getAskRawPeakToPeak() const;
    float getTransmitterVoltage() const;
    float getTransmitterCurrent() const;
    float getTransmitterPower() const;
    float getTransmitterVoltageRawAverage() const;
    float getTransmitterCurrentRawAverage() const;
    float getHalfBridgeInputVoltage() const;
    float getHalfBridgeCurrent() const;
    float getHalfBridgeOutputVoltage() const;
    HalfBridgeController& getHalfBridgeController();

private:
    static constexpr uint8_t kAdc3PowerChannelCount = 2U;
    static constexpr uint8_t kAdc3RawWindowSize = 36U;

    void processAdc2(Driver::AdcSampler& sampler);
    void processAdc3(Driver::AdcSampler& sampler);
    void processAdc4(Driver::AdcSampler& sampler);
    void feedAskBuffer(uint16_t rawSample, uint16_t sampleRepeat);
    void resetAdc3RawWindow();
    void updateAdc3RawWindow(uint16_t voltageRaw, uint16_t currentRaw);
    static float applyRawCalibration(float rawAverage, float bias, float gain);

    uint16_t adc2Data_[4] = {};
    uint16_t adc3Data_[8] = {};
    uint16_t adc4Data_[12] = {};
    uint16_t askAdcData_[4] = {};
    Driver::AdcSampler adc2Sampler_;
    Driver::AdcSampler adc3Sampler_;
    Driver::AdcSampler adc4Sampler_;
    Driver::AskDecoder askDecoder_;
    Driver::FirstOrderLpf transmitterVoltageFilter_;
    Driver::FirstOrderLpf transmitterCurrentFilter_;
    Driver::RecursiveAverageFilter halfBridgeInputVoltageFilter_;
    Driver::FirstOrderLpf halfBridgeCurrentFilter_;
    Driver::FirstOrderLpf halfBridgeOutputVoltageFilter_;
    HalfBridgeController halfBridgeController_;

    uint16_t adc3RawWindow_[kAdc3PowerChannelCount][kAdc3RawWindowSize] = {};
    uint32_t adc3RawWindowSum_[kAdc3PowerChannelCount] = {};
    uint8_t adc3RawWindowWriteIndex_ = 0U;
    uint8_t adc3RawWindowCount_ = 0U;
    float transmitterVoltageRawAverage_ = 0.0f;
    float transmitterCurrentRawAverage_ = 0.0f;

    float askRawVoltage_ = 0.0f;
    uint8_t askWriteIndex_ = 0U;
    float transmitterVoltage_ = 0.0f;
    float transmitterCurrent_ = 0.0f;
    float transmitterPower = 0.0f;
    float halfBridgeInputVoltage_ = 0.0f;
    float halfBridgeCurrent_ = 0.0f;
    float halfBridgeOutputVoltage_ = 0.0f;
};

SamplingService& samplingService();

} // namespace App
