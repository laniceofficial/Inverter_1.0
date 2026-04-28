#ifndef ASK_H
#define ASK_H

#include <stdint.h>

#include "delaytrigger.h"
#include "main.h"
#include "tim.h"
//高频24khz
#define COMMUNICATION_MIDDLE_THRESHOLD 2015
#define COMMUNICATION_TRIGGER_TIMEOUT 2 // /* 连续触发时间阈值, 单位高频 tick */ 
#define COMMUNICATION_TRIGGER_THRESHOLD 1000
#define COMMUNICATION_DISCONNECT_TIMEOUT 20 /* 反向通信合法包丢失 timeout, 单位 ms */
#define POWER_CONTROLLER_HIGH_FREQ 720000
#define POWER_CONTROLLER_LOW_FREQ 1000
#define COMMUNICATION_UPPER_THRESHOLD 300 /* 反向通信上限阈值, 以 middle threshold 为基准, 单位 ADC LSB */
#define COMMUNICATION_LOWER_THRESHOLD 300 /* 反向通信下限阈值, 以 middle threshold 为基准, , 单位 ADC LSB */

#define COMUNICATION_TIM htim8

typedef struct
{
    uint8_t requiredPowerSelection;
    uint8_t rawPowerFeedback;
    float powerFeedback;
    float transmitEfficiency;
} BackwardCommunicationData_t;

typedef struct ask_com_t
{
    uint16_t *communicationBuffer;
    uint8_t communicationBufferLength;

    uint8_t currentBitLevel;

    uint8_t _40BitsBufferPointer;
    uint8_t data20Bits[20];
    uint8_t raw10Bit[10];

    BackwardCommunicationData_t backwardCommunicationData;

    float dynamicMax;
    float dynamicMin;
    uint16_t upperThreshold;
    uint16_t lowerThreshold;
    DelayedTrigger_t upperDelayedTrigger;
    DelayedTrigger_t lowerDelayedTrigger;

    uint32_t disconnectCounter;
    uint8_t isConnected;

    uint8_t isValid;
} ask_com_t;

void ASK_Decode(ask_com_t *ins);
void commuResultFlipCallback(uint8_t lastLevel);
void newBitCome(uint8_t lastLevel);
void ask_init(ask_com_t *ins, uint16_t *Buffer, uint8_t BufferLength);
void decode20BitsBuffer(void);

#endif // ASK_H
