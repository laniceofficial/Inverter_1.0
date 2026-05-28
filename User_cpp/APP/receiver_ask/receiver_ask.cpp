#include "receiver_ask.hpp"

#include "sampling.hpp"

extern "C" {
#include "gpio.h"
#include "main.h"
}

namespace App::ReceiverAsk
{
namespace
{

constexpr float kMaxPackedPowerW = 150.0f; // 功率打包满量程，超过后按满量程发送
// constexpr uint8_t kAskTxBitCount = 10U;    // 1 bit 需求 + 8 bit 功率 + 1 bit 校验
GPIO_TypeDef* const kAskTxPort = GPIOB;    // 当前 PCB 上接收端 ASK 调制 GPIO 端口
constexpr uint16_t kAskTxPins = GPIO_PIN_4 | GPIO_PIN_3; // 两个调制脚保持同相输出

uint16_t dividerTick = 0U; // 发送分频计数，降低 ASK 状态机推进频率

float clampPower(const float power)
{
    if (power < 0.0f)
    {
        return 0.0f;
    }
    if (power > kMaxPackedPowerW)
    {
        return kMaxPackedPowerW;
    }
    return power;
}

void writeAskLevel(const uint8_t level)
{
    // 直接写 BSRR，避免读改写 GPIO 输出寄存器带来的中断竞态。
    if (level != 0U)
    {
        kAskTxPort->BSRR = static_cast<uint32_t>(kAskTxPins);
        arr_value = 1U;
    }
    else
    {
        kAskTxPort->BSRR = static_cast<uint32_t>(kAskTxPins) << 16U;
        arr_value = 0U;
    }
}

void toggleAskLevel()
{
    // ASK 编码通过边沿表示半 bit，调试镜像 arr_value 同步翻转。
    HAL_GPIO_TogglePin(kAskTxPort, kAskTxPins);
    arr_value = static_cast<uint8_t>(arr_value == 0U ? 1U : 0U);
}

void packData()
{
    // 当前接收端没有独立通信 MCU，先从本机采样服务读取半桥输出功率并打包。
    const auto& sampling = samplingService();
    const float power = sampling.getHalfBridgeOutputVoltage() * sampling.getHalfBridgeCurrent();
    data.halfBridgePower = clampPower(power);
    data.packedPower = static_cast<uint8_t>(data.halfBridgePower * (255.0f / kMaxPackedPowerW));

    // 奇偶校验覆盖 powerRequirement 和 packedPower，保持与参考工程 ASKcomm::packData 一致。
    uint16_t parity = static_cast<uint16_t>((data.powerRequirement & 0x01U) |
                                           (static_cast<uint16_t>(data.packedPower) << 1U));
    parity ^= static_cast<uint16_t>(parity >> 8U);
    parity ^= static_cast<uint16_t>(parity >> 4U);
    parity ^= static_cast<uint16_t>(parity >> 2U);
    parity ^= static_cast<uint16_t>(parity >> 1U);

    data.txMessage = static_cast<uint16_t>(0x0|(data.powerRequirement & 0x01U) |
                                           (static_cast<uint16_t>(data.packedPower) << 1U) |
                              ((parity & 0x01U) << 9U));
    /*data.txMessage =
    bit0        powerRequirement
    bit1~bit8   packedPower
    bit9        parity
*/
}

} // namespace

Data data;

void init()
{
    // 上电默认先保持调制脚为低，避免在发射端启动前产生误触发边沿。
    writeAskLevel(0U);
    reset();
}

void reset()
{
    // 只重置当前发送节拍，不清除已经配置好的数据源和开关。
    data.loopIndex = 0U;
    dividerTick = 0U;
}

void setEnabled(const bool enabled)
{
    data.enabled = enabled;
    if (!enabled)
    {
        // 关闭 ASK 时立刻拉低输出，发射端看到的是空闲低电平。
        writeAskLevel(0U);
        reset();
    }
}

void setDivider(const uint16_t divider)
{
    // 防止外部调试把分频写成 0 后状态机每次都异常推进。
    gSignalFreq = (divider == 0U) ? 1U : divider;
    reset();
}

void setPowerRequirement(const uint8_t requirement)
{
    data.powerRequirement = static_cast<uint8_t>(requirement & 0x01U);
}
//协议： 起始位（双高）→ 数据（20 个半 bit）→ 停止位（双低）→ 短帧间隔（约 16 个半 bit 的低电平或简单翻转）
void loop()
{
    freq = static_cast<uint16_t>(freq + 1U);
    if (freq >= 4000)
    {

        freq = 0U;
    }
    if (!data.enabled)
    {
        // 关闭状态下仍可周期调用 loop，但不会产生 ASK 边沿。
        writeAskLevel(0U);
        reset();
        return;
    }

    if (gSignalFreq == 0U)
    {
        gSignalFreq = 1U;
    }

    ++dividerTick;
    if (dividerTick < gSignalFreq)
    {
        // 当前 tick 只维持原电平，不推进 ASK 半 bit。
        return;
    }
    dividerTick = 0U;
    // if ((data.loopIndex & 0x01U))
    // {
    //         toggleAskLevel();
    // }
    // else
    // {
    //     toggleAskLevel();
    // }
    // ++data.loopIndex;
    // if (data.loopIndex >= 40U)
    //     {data.loopIndex = 0U;}
    // return;

    switch (data.loopIndex)
    {
        case 0U:
            // 起始位：固定拉高，并在帧开始时锁存本帧要发送的功率信息。
            writeAskLevel(1U); // 起始位一定为双高
            packData();
            ++data.loopIndex;
            break;

        case 1U:
            // 起始位保持一个半 bit，保证发射端能稳定识别帧头。
            ++data.loopIndex;
            break;

        case 22U:
            // 10 bit 负载发送完成后进入固定停止位：低电平。
            writeAskLevel(0U);
            ++data.loopIndex;
            break;

        case 23U:
            // 停止位保持一个半 bit。
            ++data.loopIndex;
            break;

        case 39U:
            // 帧间隔结束，下一次从新帧起始位重新开始。
            writeAskLevel(0U);
            data.loopIndex = 0U;
            break;

        default:
        {
            // 数据区采用参考工程的边沿编码：
            // 偶数半 bit 必翻转，奇数半 bit 仅在当前数据 bit 为 1 时翻转。
            if ((data.loopIndex & 0x01U))
            {
                /*因为 loopIndex 0、1 是起始位，数据位从 loopIndex 2 开始（bit0 的两个半 bit：偶数半bit 2、奇数半 bit 3）。所以 bitIndex 对应：
                loopIndex 2/3 → bitIndex 0; loopIndex 4/5 → bitIndex 1*/
                const uint8_t bitIndex = static_cast<uint8_t>((data.loopIndex >> 1U) - 1U);
                if ((data.loopIndex > 23U) || ((data.txMessage >> bitIndex) & 0x01U))
                {
                    toggleAskLevel();
                }
            }
            else
            {
                toggleAskLevel();
            }
            ++data.loopIndex;
            break;
        }

    }


}

} // namespace App::ReceiverAsk
extern "C" {
volatile uint16_t gSignalFreq = 2U; //2--1k,1--2k,
volatile uint16_t freq = 0U;
volatile uint8_t arr_value = 0U;
//1k周期，0.5ms变化一次
}
