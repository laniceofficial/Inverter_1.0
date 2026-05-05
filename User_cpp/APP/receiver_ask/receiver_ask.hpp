#pragma once

#include <cstdint>

namespace App::ReceiverAsk
{

// 接收端通过拉动通信引脚产生 ASK 回传。
// 帧格式参考功率板工程：bit0 为功率需求，bit1~8 为 0~255 功率值，bit9 为奇偶校验。
struct Data
{
    bool enabled = true;             // 是否允许接收端 ASK 回传
    uint16_t txMessage = 0U;         // 当前待发送的 10 bit 负载
    uint8_t loopIndex = 0U;          // ASK 半 bit 状态机位置，0~39 为一帧
    uint8_t packedPower = 0U;        // 半桥功率压缩值，0~255
    uint8_t powerRequirement = 1U;   // 接收端功率需求，当前只使用最低 1 bit
    float halfBridgePower = 0.0f;    // 打包前的半桥功率，单位 W，便于 Ozone 观察
};

extern Data data;

// 初始化 ASK 输出引脚状态和发送状态机。
void init();

// 由 TIM6 周期调用，每次按分频推进 ASK 半 bit 状态机。
void loop();

// 复位当前帧位置，不修改 enabled 和功率需求。
void reset();

// 运行时打开/关闭 ASK 回传；关闭时会把输出拉低。
void setEnabled(bool enabled);

// 设置发送分频：1 表示每个 TIM6 tick 推进一次状态机。
void setDivider(uint16_t divider);

// 设置功率需求位，只保留最低 bit。
void setPowerRequirement(uint8_t requirement);

} // namespace App::ReceiverAsk

extern "C" {
extern volatile uint16_t gSignalFreq; // 调试用发送分频，兼容旧工程变量名
extern volatile uint16_t freq;        // 调试计数，每次 loop 调用自增
extern volatile uint8_t arr_value;    // 当前 ASK 输出电平镜像
extern volatile uint8_t allow_;       // 当前 tick 是否推进了 ASK 状态机
}
