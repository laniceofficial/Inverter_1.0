#ifndef ASK_H
#define ASK_H
#include <stdint.h>
#include "delaytrigger.h"
#include "main.h"
#include "tim.h"


#define COMMUNICATION_MIDDLE_THRESHOLD 1000
#define COMMUNICATION_TRIGGER_TIMEOUT 10 /* 连续触发时间阈值, 单位高频 tick */
#define COMMUNICATION_TRIGGER_THRESHOLD 1000
#define COMMUNICATION_DISCONNECT_TIMEOUT 1 /* 反向通信合法包丢失 timeout, 单位 ms */
#define POWER_CONTROLLER_HIGH_FREQ 10000 // ask解码频率
#define POWER_CONTROLLER_LOW_FREQ 1000 // 状态机频率

#define COMUNICATION_TIM htim8

typedef struct
{
    uint8_t requiredPowerSelection; // 用来切换 10W 和 100W 闭环
    uint8_t rawPowerFeedback;
    float powerFeedback; // 换算后的功率, 单位 W
    float transmitEfficiency;
} BackwardCommunicationData_t;

typedef struct ask_com_t
{
    uint16_t *communicationBuffer; // 指向 ADC DMA buffer
    uint8_t communicationBufferLength; // ADC DMA buffer 长度

    uint8_t currentBitLevel;

    uint8_t _40BitsBufferPointer; // 指示下一个 write 的位置, 0 - 39
    uint8_t data20Bits[20]; // Packet body 的 20 bit
    uint8_t raw10Bit[10]; // 2 bit 合一解码后的发送端原始数据

    BackwardCommunicationData_t backwardCommunicationData;

    float dynamicMax;
    float dynamicMin;
    uint16_t upperThreshold; // 上限阈值
    uint16_t lowerThreshold; // 下限阈值
    DelayedTrigger_t upperDelayedTrigger;
    DelayedTrigger_t lowerDelayedTrigger;

    // 解码正确才算 connected
    uint32_t disconnectCounter;
    uint8_t isConnected;

    uint8_t isValid;

} ask_com_t;
void ASK_Decode(ask_com_t *ins);
void commuResultFlipCallback(uint8_t lastLevel);
void newBitCome(uint8_t lastLevel);
void ask_init(ask_com_t *ins, uint16_t *Buffer, uint8_t BufferLength);
void decode20BitsBuffer();
#endif // !ASK_H
