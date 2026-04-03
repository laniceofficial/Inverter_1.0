#include "ask.h"
#include <stdint.h>
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_tim.h"

// const uint8_t stratSequense[] = {0, 0, 1, 0, 1, 0, 1, 0, 1, 0,
//                                  1, 0, 1, 0, 1, 0, 1, 0, 1, 1};
const uint8_t stratSequense[] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
const uint8_t stratSequenseLength = sizeof(stratSequense) / sizeof(stratSequense[0]);
uint32_t now_bits = 0;
static ask_com_t *CommunicationStatus = NULL;
void ASK_Decode(ask_com_t *ins)
{
    for (uint8_t i = 0; i < ins->communicationBufferLength; i++)
    {
        now_bits = ins->communicationBuffer[i];
        // 新 ADC 值突破当前 Dynamic Min/Max 就带滤波地更新, 否则衰减
        if (ins->communicationBuffer[i] > ins->dynamicMax)
        {
            ins->dynamicMax = ins->dynamicMax * 0.2f + ins->communicationBuffer[i] * 0.8f;
        }
        else
        {
            ins->dynamicMax = (COMMUNICATION_MIDDLE_THRESHOLD + 150) * 0.003f + ins->dynamicMax * 0.997f;
        }

        if (ins->communicationBuffer[i] < ins->dynamicMin)
        {
            ins->dynamicMin = ins->dynamicMin * 0.2f + ins->communicationBuffer[i] * 0.8f;
        }
        else
        {
            ins->dynamicMin = (COMMUNICATION_MIDDLE_THRESHOLD - 150) * 0.005f + ins->dynamicMin * 0.995f;
        }

        // 把 threshold 设置为 dynamicMin/Max 的 70%
        ins->upperThreshold = COMMUNICATION_MIDDLE_THRESHOLD * 0.3f + ins->dynamicMax * 0.7f;
        ins->lowerThreshold = COMMUNICATION_MIDDLE_THRESHOLD * 0.3f + ins->dynamicMin * 0.7f;

        if (DelayedTrigger_Update(&ins->upperDelayedTrigger, ins->communicationBuffer[i] > ins->upperThreshold))
        {
            DelayedTrigger_Reset(&ins->upperDelayedTrigger);

            if (ins->currentBitLevel == 0)
            {
                ins->currentBitLevel = 1; // 0变1
                commuResultFlipCallback(1); // 实测发送和接收电平是反向的
            }
        }
        if (DelayedTrigger_Update(&ins->lowerDelayedTrigger, ins->communicationBuffer[i] < ins->lowerThreshold))
        {
            DelayedTrigger_Reset(&ins->upperDelayedTrigger);
            if (ins->currentBitLevel == 1)
            {
                ins->currentBitLevel = 0;
                commuResultFlipCallback(0); // 实测发送和接收电平是反向的
            }
        }
    }

    if (ins->disconnectCounter <
        COMMUNICATION_DISCONNECT_TIMEOUT * POWER_CONTROLLER_HIGH_FREQ / POWER_CONTROLLER_LOW_FREQ)
    {
        ins->disconnectCounter++;
        if (ins->disconnectCounter ==
            COMMUNICATION_DISCONNECT_TIMEOUT * POWER_CONTROLLER_HIGH_FREQ / POWER_CONTROLLER_LOW_FREQ)
        {
            // 反向通信丢失, 断开连接
            ins->isConnected = 0;
            ins->isValid = 0;
        }
    }
}
void ask_init(ask_com_t *ins, uint16_t *Buffer, uint8_t BufferLength)
{
    ins->communicationBufferLength = BufferLength;
    ins->communicationBuffer = Buffer;
    ins->currentBitLevel = 0;

    ins->_40BitsBufferPointer = 0; // 指示下一个 write 的位置, 0 - 39
    //   ins->data20Bits[20];           // Packet body 的 20 bit
    //   ins->raw10Bit[10];             // 2 bit 合一解码后的发送端原始数据

    // ins->backwardCommunicationData;

    ins->dynamicMax = 1500;
    ins->dynamicMin = 500;
    ins->upperThreshold = 1000 + 250; // 上限阈值
    ins->lowerThreshold = 1000 - 250; // 下限阈值

    // 解码正确才算 connected
    ins->disconnectCounter = 0;
    ins->isConnected = 0;

    ins->isValid = 0;
    DelayedTrigger_Init(&ins->lowerDelayedTrigger, COMMUNICATION_TRIGGER_TIMEOUT, 1, 1);
    DelayedTrigger_Init(&ins->upperDelayedTrigger, COMMUNICATION_TRIGGER_TIMEOUT, 1, 1);
    CommunicationStatus = ins;
    HAL_TIM_Base_Init(&COMUNICATION_TIM);
}
void commuResultFlipCallback(uint8_t lastLevel)
{
    uint32_t dt = __HAL_TIM_GetCounter(&COMUNICATION_TIM); // 获取计数器值
    __HAL_TIM_SET_COUNTER(&COMUNICATION_TIM, 0); // 无论后续解码如何都重置计数器

    // if (dt < 125 || dt > 625) {
    //   // 采样到错误的时间间隔
    //   CommunicationStatus->_40BitsBufferPointer = 0;
    //   return;
    // }

    // 250us 为 1 的翻转半周期
    // 500us 为 0 的翻转半周期
    if (dt < 375)
    {
        // 250us 小周期翻转, 不管是前半部分还是后半部分都记录
        newBitCome(lastLevel);
    }
    else
    {
        newBitCome(lastLevel);
        newBitCome(lastLevel);
    }
}
void newBitCome(uint8_t lastLevel)
{
    //<20
    if (CommunicationStatus->_40BitsBufferPointer < stratSequenseLength)
    {
        // 如果当前 bit 是 header, 则需要匹配 header.
        if (stratSequense[CommunicationStatus->_40BitsBufferPointer] == lastLevel)
        {
            CommunicationStatus->_40BitsBufferPointer += 1;
        }
        else
        {
            CommunicationStatus->_40BitsBufferPointer = 0;
        }
    }
    else
    {
        // 不是 header, 直接记录
        CommunicationStatus->data20Bits[CommunicationStatus->_40BitsBufferPointer - stratSequenseLength] = lastLevel;
        CommunicationStatus->_40BitsBufferPointer += 1;
    }
    if (CommunicationStatus->_40BitsBufferPointer == 40)
    {
        CommunicationStatus->_40BitsBufferPointer = 0;
        decode20BitsBuffer();
    }
}
void decode20BitsBuffer()
{
    CommunicationStatus->disconnectCounter = 0;
    CommunicationStatus->isConnected = 1;

    // 两个原始 bit 对应 4bit, 这四个 bit 的 bit[1] bit[2] 必然不同, 否则就是寄了
    for (uint8_t i = 0; i < 9; i++)
    {
        if (CommunicationStatus->data20Bits[i * 2 + 1] == CommunicationStatus->data20Bits[i * 2 + 2])
        {
            // bit 间无跳变
            CommunicationStatus->isValid = 0;
            return;
        }
    }

    // 两个 bit 合二为一
    for (uint8_t i = 0; i < 10; i++)
    {
        if (CommunicationStatus->data20Bits[i * 2] == CommunicationStatus->data20Bits[i * 2 + 1])
        {
            CommunicationStatus->raw10Bit[i] = 0;
        }
        else
        {
            CommunicationStatus->raw10Bit[i] = 1;
        }
    }

    uint8_t requiredPowerSelection = CommunicationStatus->raw10Bit[0];
    uint8_t rawPowerFeedback = 0;
    float powerFeedback = 0.0f;
    float efficiency = 0.5f;

    for (uint8_t i = 0; i < 8; i++)
    {
        rawPowerFeedback |= (CommunicationStatus->raw10Bit[i + 1] << i);
    }

    uint16_t parity = requiredPowerSelection | (rawPowerFeedback << 1);
    parity ^= (parity >> 8);
    parity ^= (parity >> 4);
    parity ^= (parity >> 2);
    parity ^= (parity >> 1);
    if ((parity & 0x01) != CommunicationStatus->raw10Bit[9])
    {
        // 奇偶校验错误
        CommunicationStatus->isValid = 0;
        return;
    }

    powerFeedback = (float)rawPowerFeedback / 255.0f * 150.0f;
    // efficiency = powerFeedback / Analog::adcData.pTX;
    if (efficiency < 0.2f || efficiency > 0.9f)
    {
        // 效率异常
        CommunicationStatus->isValid = 0;
        return;
    }

    CommunicationStatus->backwardCommunicationData.requiredPowerSelection = requiredPowerSelection;
    CommunicationStatus->backwardCommunicationData.rawPowerFeedback = rawPowerFeedback;
    CommunicationStatus->backwardCommunicationData.powerFeedback = powerFeedback;
    CommunicationStatus->backwardCommunicationData.transmitEfficiency =
        efficiency * 0.5f + CommunicationStatus->backwardCommunicationData.transmitEfficiency * 0.5f;

    CommunicationStatus->isValid = 1;
}

// void askLoop() // 在4k循环中调用
// {
//   if (!askData.enableASK) {
//     HAL_GPIO_WritePin(COMM1_GPIO_Port, COMM1_Pin, GPIO_PIN_RESET);
//     askData.askLoopIndex = 0;
//     return;
//   }

//   switch (askData.askLoopIndex) {
//   case 0:
//     COMM1_GPIO_Port->BSRR = (uint32_t)COMM1_Pin; // 起始位一定为双高
//     packData(adcData.pWPTlf, askData.powerRequirement);
//     askData.askLoopIndex++;
//     break;
//   case 1:
//     askData.askLoopIndex++;
//     break;
//   case 22:
//     COMM1_GPIO_Port->BSRR = (uint32_t)COMM1_Pin << 16U; // 停止位一定位双低
//     askData.askLoopIndex++;
//     break;
//   case 23:
//     askData.askLoopIndex++;
//     break;
//   case 39:
//     COMM1_GPIO_Port->BSRR = (uint32_t)COMM1_Pin << 16U; // 重新开始计数
//     askData.askLoopIndex = 0;
//     break;
//   default:
//     if (askData.askLoopIndex & 0b1) {
//       if (askData.askLoopIndex > 23 ||
//           ((askData.txMessage >> ((askData.askLoopIndex >> 1) - 1)) & 0b1)) {
//         HAL_GPIO_TogglePin(COMM1_GPIO_Port, COMM1_Pin);
//       }
//     } else {
//       HAL_GPIO_TogglePin(COMM1_GPIO_Port, COMM1_Pin);
//     }

//     askData.askLoopIndex++;
//     break;
//   }
// }
