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
    // Ask = 0U,
    // TransmitterVoltage,
    // TransmitterCurrent,
    HalfBridgeInputVoltage,
    HalfBridgeOutputVoltage,
    HalfBridgeCurrent,
    Count,
};

// 汇总 ADC DMA、ASK 解码和半桥反馈，给主任务提供统一采样入口。
class SamplingService
{
public:
    void init();
    void start();
    void stop();
    bool handleAdcConvCpltCallback(ADC_HandleTypeDef* hadc);

    float getHalfBridgeInputVoltage() const;
    float getHalfBridgeCurrent() const;
    float getHalfBridgeOutputVoltage() const;
    HalfBridgeController& getHalfBridgeController();

private:
    static constexpr uint8_t kAdc3PowerChannelCount = 2U;
    static constexpr uint8_t kAdc3RawWindowSize = 36U;

    void processAdc4(Driver::AdcSampler& sampler);
    uint16_t adc4Data_[12] = {};
    uint16_t askAdcData_[4] = {};
    Driver::AdcSampler adc4Sampler_;
    Driver::RecursiveAverageFilter halfBridgeInputVoltageFilter_;
    Driver::FirstOrderLpf halfBridgeCurrentFilter_;
    Driver::FirstOrderLpf halfBridgeOutputVoltageFilter_;
    HalfBridgeController halfBridgeController_;

    float halfBridgeInputVoltage_ = 0.0f;
    float halfBridgeCurrent_ = 0.0f;
    float halfBridgeOutputVoltage_ = 0.0f;

    uint32_t lastControlLoopTick_ = 0U;
    uint32_t controlLoopPeriodCycles_ = 0U;
};

SamplingService& samplingService();

} // namespace App
