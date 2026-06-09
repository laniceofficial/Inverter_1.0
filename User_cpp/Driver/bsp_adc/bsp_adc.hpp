#pragma once

extern "C" {
#include "adc.h"
#include "main.h"
}

#include <cstdint>

#if defined(HAL_ADC_MODULE_ENABLED)

namespace Driver
{

constexpr uint8_t kAdcMaxChannelCount = 8U;
constexpr uint8_t kAdcMaxInstanceCount = 8U;

class AdcSampler;
using AdcCallback = void (*)(AdcSampler& sampler);

// ADC 标定模式
enum class AdcCalMode : uint8_t
{
    VoltageToValue = 0U, // 两步法：raw → voltage → value（默认，scale/offset 是电压→工程量的系数）
    RawToValue = 1U,     // 一步法：raw → value（scale/offset 是 raw→工程量的系数，跳过电压计算）
};

// 单个工程量通道的映射配置：从 DMA 序列中取样，换算到上层结果数组。
struct AdcChannelConfig
{
    uint8_t resultIndex = 0U;   // 上层结果数组下标
    uint8_t sampleOffset = 0U;  // DMA 序列中的通道偏移
    float scale = 1.0f;         // 标定系数（含义取决于 calMode）
    float offset = 0.0f;        // 标定偏置（含义取决于 calMode）
    AdcCalMode calMode = AdcCalMode::VoltageToValue; // 标定模式
};

// ADC + DMA 采样器配置，支持一个 ADC 实例对应多个工程量通道。
struct AdcSamplerConfig
{
    ADC_HandleTypeDef* hadc = nullptr;          // HAL ADC 句柄
    uint16_t* dmaBuffer = nullptr;              // DMA 原始采样缓存
    uint16_t dmaLength = 0U;                    // DMA 缓存总长度
    uint8_t dmaChannelCount = 0U;               // 每轮扫描包含的 ADC 通道数
    uint16_t sampleRepeat = 1U;                 // 每个通道在一次处理窗口内重复采样次数
    const AdcChannelConfig* channels = nullptr; // 工程量通道映射表
    uint8_t channelCount = 0U;                  // 工程量通道数量
    float vref = 3.306f;                        // ADC 参考电压
    float adcMaxValue = 4095.0f;                // ADC 满量程计数
    bool calibrateOnStart = false;              // start 时是否先校准
    bool disableHalfTransferInterrupt = true;   // 通常只处理满传输中断，减少中断频率
    AdcCallback callback = nullptr;             // DMA 完成并处理后的回调
};

// 对 HAL ADC DMA 的轻量封装，负责平均、换算和回调分发。
class AdcSampler
{
public:
    void init(const AdcSamplerConfig& config);
    bool calibrate();
    bool start();
    void stop();
    bool processDmaBuffer();

    float getValue(uint8_t index) const;
    float getVoltage(uint8_t index) const;
    uint16_t getRawAverage(uint8_t index) const;
    uint16_t getRawSample(uint16_t sampleIndex, uint8_t channelOffset) const;
    uint16_t getSampleRepeat() const;
    uint8_t getDmaChannelCount() const;
    ADC_HandleTypeDef* getHandle() const;

    static bool handleConvCpltCallback(ADC_HandleTypeDef* hadc);

private:
    bool isStartConfigValid() const;
    bool isProcessConfigValid() const;
    bool registerInstance();
    void unregisterInstance();
    static AdcSampler* findInstance(const ADC_HandleTypeDef* hadc);

    ADC_HandleTypeDef* hadc_ = nullptr;
    uint16_t* dmaBuffer_ = nullptr;
    uint16_t dmaLength_ = 0U;
    uint8_t dmaChannelCount_ = 0U;
    uint16_t sampleRepeat_ = 1U;
    const AdcChannelConfig* channels_ = nullptr;
    uint8_t channelCount_ = 0U;
    float vref_ = 3.306f;
    float adcMaxValue_ = 4095.0f;
    bool calibrateOnStart_ = false;
    bool disableHalfTransferInterrupt_ = true;
    AdcCallback callback_ = nullptr;
    float values_[kAdcMaxChannelCount] = {};
    float voltages_[kAdcMaxChannelCount] = {};
    uint16_t rawAverages_[kAdcMaxChannelCount] = {};

    static AdcSampler* instances_[kAdcMaxInstanceCount];
};

} // namespace Driver

#endif // HAL_ADC_MODULE_ENABLED
