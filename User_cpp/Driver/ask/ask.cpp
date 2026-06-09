#include "ask.hpp"
#include <stdint.h>
// uint32_t cout = 0;
namespace Driver
{
namespace
{
    constexpr float kMaxPackedPowerW = 150.0f; // 功率打包满量程，超过后按满量程发送
    constexpr uint16_t kMiddleThreshold = 2015U;
    constexpr uint16_t kTriggerTimeout = 2U;
    constexpr uint32_t kDisconnectTimeoutMs = 50U;
    constexpr uint32_t kPowerControllerHighFreq = 30000U;
    constexpr uint32_t kPowerControllerLowFreq = 1000U;
    constexpr uint16_t kUpperThresholdOffset = 300U;
    constexpr uint16_t kLowerThresholdOffset = 300U;

    // WPC ASK 前导码表现为连续 20 个交替电平，用来锁定后续数据位置。须注意这里和接收端的是反相的
    // constexpr uint8_t kStartSequence[] = {0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    //  0, 1, 0, 1, 0, 1, 0, 1, 1};//原版
    constexpr uint8_t kStartSequence[] = {1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0};
    constexpr uint8_t kStartSequenceLength = static_cast<uint8_t>(sizeof(kStartSequence) / sizeof(kStartSequence[0]));

} // namespace

void AskDecoder::init(uint16_t* buffer, const uint8_t bufferLength)
{
    communicationBuffer_ = buffer;
    communicationBufferLength_ = bufferLength;
    resetValid();

    dynamicMax_ = static_cast<float>(kMiddleThreshold);
    dynamicMin_ = static_cast<float>(kMiddleThreshold);
    upperThreshold_ = kMiddleThreshold + kUpperThresholdOffset;
    lowerThreshold_ = kMiddleThreshold - kLowerThresholdOffset;
    lowerDelayedTrigger_.init(kTriggerTimeout);
    upperDelayedTrigger_.init(kTriggerTimeout);

    setDebugPin(0U);
    HAL_TIM_Base_Init(&htim8);
    HAL_TIM_Base_Start(&htim8);
    __HAL_TIM_SET_COUNTER(&htim8, 0U);
}
// uint8_t aa=0;
void AskDecoder::decode()
{
    if (communicationBuffer_ == nullptr)
    {
        return;
    }

    for (uint8_t i = 0U; i < communicationBufferLength_; ++i)
    {
        const uint16_t sample = communicationBuffer_[i];

        // 动态跟踪 ASK 波形上下包络，避免固定阈值受线圈距离和增益漂移影响。
        if (sample > dynamicMax_)
        {
            dynamicMax_ = dynamicMax_ * 0.2f + static_cast<float>(sample) * 0.8f;
        }
        else
        {
            dynamicMax_ = static_cast<float>(kMiddleThreshold + 200U) * 0.005f + dynamicMax_ * 0.995f;
        }

        if (sample < dynamicMin_)
        {
            dynamicMin_ = dynamicMin_ * 0.2f + static_cast<float>(sample) * 0.8f;
        }
        else
        {
            dynamicMin_ = static_cast<float>(kMiddleThreshold - 200U) * 0.01f + dynamicMin_ * 0.99f;
        }

        upperThreshold_ = static_cast<uint16_t>(static_cast<float>(kMiddleThreshold) * 0.3f + dynamicMax_ * 0.7f);
        lowerThreshold_ = static_cast<uint16_t>(static_cast<float>(kMiddleThreshold) * 0.3f + dynamicMin_ * 0.7f);

        if (upperDelayedTrigger_.update(sample > upperThreshold_))
        {
            lowerDelayedTrigger_.reset();
            if (currentBitLevel_ == 0U)
            {
                currentBitLevel_ = 1U;
                setDebugPin(1U);
                handleEdge(1U);
            }
        }

        if (lowerDelayedTrigger_.update(sample < lowerThreshold_))
        {
            upperDelayedTrigger_.reset();
            if (currentBitLevel_ == 1U)
            {
                currentBitLevel_ = 0U;
                setDebugPin(0U);
                handleEdge(0U);
            }
        }
        
    }

    const uint32_t disconnectLimit = kDisconnectTimeoutMs * kPowerControllerHighFreq / kPowerControllerLowFreq;
    if (disconnectCounter_ < disconnectLimit)
    {
        ++disconnectCounter_;
        if (disconnectCounter_ == disconnectLimit)
        {
            connected_ = false;
            valid_ = false;
        }
    }
}

void AskDecoder::resetValid()
{
    currentBitLevel_ = 0U;
    bitBufferPointer_ = 0U;
    disconnectCounter_ = 0U;
    connected_ = false;
    valid_ = false;

    for (uint8_t& bit : data20Bits_)
    {
        bit = 0U;
    }
    for (uint8_t& bit : raw10Bit_)
    {
        bit = 0U;
    }
}

bool AskDecoder::isValid() const
{
    return valid_;
}

bool AskDecoder::isConnected() const
{
    return connected_;
}

uint8_t AskDecoder::getBitBufferPointer() const
{
    return bitBufferPointer_;
}

const BackwardCommunicationData& AskDecoder::getBackwardData() const
{
    return backwardData_;
}

void AskDecoder::handleEdge(const uint8_t lastLevel)
{
    const uint32_t dt = __HAL_TIM_GetCounter(&htim8);
    __HAL_TIM_SET_COUNTER(&htim8, 0U);
    //这里与ask频率挂钩
    // 边沿间隔过短/过长都认为不是合法 ASK 符号，直接重新同步前导码。
    if ((dt < 200U) || (dt > 1300U)) //2k周期：125，625，375
    {
        bitBufferPointer_ = 0U;
        GPIOC->BSRR = static_cast<uint32_t>(GPIO_PIN_2) << 16U;
        return;
    }

    pushBit(lastLevel);
    if (dt >= 750U)
    {
        // 长电平代表两个相同符号周期，需要补入第二个 bit。
        pushBit(lastLevel);
    }
}

void AskDecoder::pushBit(const uint8_t lastLevel)
{
    if (bitBufferPointer_ < kStartSequenceLength)
    {
        if (kStartSequence[bitBufferPointer_] == lastLevel)
        {
            ++bitBufferPointer_;

        }
        else
        {
            // if (lastLevel == 0 && bitBufferPointer_ == 19)
            // {
            //     cout++;
            // }
            GPIOC->BSRR = static_cast<uint32_t>(GPIO_PIN_2) << 16U;
            bitBufferPointer_ = 0U;
        }
    }
    else
    {
        data20Bits_[bitBufferPointer_ - kStartSequenceLength] = lastLevel;
        ++bitBufferPointer_;
    }

    if (bitBufferPointer_ == 40U)
    {
        bitBufferPointer_ = 0U;
        decode20BitsBuffer();
    }
}

// 偶校验：将 9-bit 值 (bit0=requiredPowerSelection, bit1-8=rawPowerFeedback) 折叠 XOR，
// 与 raw10Bit_[9] 比较，不匹配则校验失败。
static bool checkEvenParity(const uint8_t requiredPowerSelection, const uint8_t rawPowerFeedback,
                            const uint8_t parityBit)
{
    uint16_t parity = static_cast<uint16_t>(requiredPowerSelection) |
                      static_cast<uint16_t>(rawPowerFeedback << 1);
    parity ^= (parity >> 8);
    parity ^= (parity >> 4);
    parity ^= (parity >> 2);
    parity ^= (parity >> 1);
    return (parity & 0x01U) == parityBit;
}

void AskDecoder::setTransmitterPowerGetter(const TransmitterPowerGetter getter)
{
    powerGetter_ = getter;
}

void AskDecoder::decode20BitsBuffer()
{
    disconnectCounter_ = 0U;
    connected_ = true;

    // ---- Manchester 校验：相邻两位必须翻转 ----
    for (uint8_t i = 0U; i < 9U; ++i)
    {
        if (data20Bits_[i * 2U + 1U] == data20Bits_[i * 2U + 2U])
        {
            valid_ = false;
            return;
        }
    }

    // ---- Manchester → 10 bit 原始数据 ----
    for (uint8_t i = 0U; i < 10U; ++i)
    {
        raw10Bit_[i] = (data20Bits_[i * 2U] == data20Bits_[i * 2U + 1U]) ? 0U : 1U;
    }

    const uint8_t requiredPowerSelection = raw10Bit_[0];

    uint8_t rawPowerFeedback = 0U;
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        rawPowerFeedback |= static_cast<uint8_t>(raw10Bit_[i + 1U] << i);
    }

    // ---- 奇偶校验 ----
    if (!checkEvenParity(requiredPowerSelection, rawPowerFeedback, raw10Bit_[9]))
    {
        valid_ = false;
        return;
    }

    // ---- 功率换算 ----
    const float powerFeedback = static_cast<float>(rawPowerFeedback) / 255.0f * kMaxPackedPowerW;

    // ---- 效率校验（仅当上层注册了功率获取回调时执行）----
    float efficiency = 0.0f;
    if (powerGetter_ != nullptr)
    {
        const float pTX = powerGetter_();
        if (pTX > 0.001f)
        {
            efficiency = powerFeedback / pTX;
            if (efficiency < kMinEfficiency || efficiency > kMaxEfficiency)
            {
                // 效率超出合理范围，判定本帧无效
                valid_ = false;
                return;
            }
        }
    }

    // ---- 更新回传数据 ----
    backwardData_.requiredPowerSelection = requiredPowerSelection;
    backwardData_.rawPowerFeedback = rawPowerFeedback;
    backwardData_.powerFeedback = powerFeedback;
    backwardData_.transmitEfficiency =
        efficiency * 0.5f + backwardData_.transmitEfficiency * 0.5f;
    valid_ = true;
    GPIOC->BSRR = GPIO_PIN_2;
}

void AskDecoder::setDebugPin(const uint8_t level)
{
    // PB3 用作示波器调试脚，直接观察软件判定出的 ASK 电平。需改
    if (level != 0U)
    {
        GPIOB->BSRR = GPIO_PIN_9;
    }
    else
    {
        GPIOB->BSRR = static_cast<uint32_t>(GPIO_PIN_9) << 16U;
    }
}

} // namespace Driver