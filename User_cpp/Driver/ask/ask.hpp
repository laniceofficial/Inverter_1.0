#pragma once

#include "delaytrigger.hpp"

extern "C" {
#include "main.h"
#include "tim.h"
}

#include <cstdint>

namespace Driver
{

// 接收端通过 ASK 回传的简化数据，供上层判断功率需求和通信状态。
struct BackwardCommunicationData
{
    uint8_t requiredPowerSelection = 0U; // 功率档位选择位
    uint8_t rawPowerFeedback = 0U;       // 8 bit 原始功率反馈
    float powerFeedback = 0.0f;          // 换算后的功率反馈值
    float transmitEfficiency = 0.0f;     // 预留的传输效率估计值
};

// 发射功率获取回调：由上层（采样服务）注入，ASK 层不依赖具体模块。
// 返回当前发射功率 (W)，若未注册则无法做效率校验，仅跳过该步骤。
using TransmitterPowerGetter = float (*)();

// ASK 解码器：从 ADC 采样波形中提取边沿，再按 20 bit 前导码 + 20 bit 数据解码。
class AskDecoder
{
public:
    void init(uint16_t* buffer, uint8_t bufferLength);
    void decode();
    void resetValid();

    bool isValid() const;
    bool isConnected() const;
    uint8_t getBitBufferPointer() const;
    const BackwardCommunicationData& getBackwardData() const;

    // 注册发射功率获取回调（非必须，未注册时跳过效率校验）
    void setTransmitterPowerGetter(TransmitterPowerGetter getter);

private:
    void handleEdge(uint8_t lastLevel);
    void pushBit(uint8_t lastLevel);
    void decode20BitsBuffer();
    void setDebugPin(uint8_t level);

    static constexpr float kMinEfficiency = 0.1f;  // 效率下限
    static constexpr float kMaxEfficiency = 1.5f;  // 效率上限(放宽以容忍测量误差)

    uint16_t* communicationBuffer_ = nullptr; // 本次待解码的 ADC 原始采样窗口
    uint8_t communicationBufferLength_ = 0U;  // 采样窗口长度
    uint8_t currentBitLevel_ = 0U;            // 当前判定出的 ASK 高/低电平
    uint8_t bitBufferPointer_ = 0U;           // 前导码和数据位接收进度
    uint8_t data20Bits_[20] = {};             // Manchester 编码后的 20 bit 数据区
    uint8_t raw10Bit_[10] = {};               // Manchester 解码后的 10 bit 原始数据
    BackwardCommunicationData backwardData_ = {};
    TransmitterPowerGetter powerGetter_ = nullptr; // 发射功率获取回调
    float dynamicMax_ = 2015.0f;              // 动态高电平估计
    float dynamicMin_ = 2015.0f;              // 动态低电平估计
    uint16_t upperThreshold_ = 2315U;         // 高电平触发阈值
    uint16_t lowerThreshold_ = 1715U;         // 低电平触发阈值
    DelayedTrigger upperDelayedTrigger_;
    DelayedTrigger lowerDelayedTrigger_;
    uint32_t disconnectCounter_ = 0U;         // 超时未收到有效帧则判定断连
    bool connected_ = false;                  // 是否近期收到过完整帧
    bool valid_ = false;                      // 最近一帧是否校验有效
};

} // namespace Driver
