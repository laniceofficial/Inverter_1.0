#include "ask.h"

#include <stdint.h>

#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_tim.h"

// const uint8_t stratSequense[] = {
//     0, 0, 1, 0, 1, 0, 1, 0, 1, 0,
//     1, 0, 1, 0, 1, 0, 1, 0, 1, 1,
// };

const uint8_t stratSequense[] = {
    0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
};
const uint8_t stratSequenseLength = sizeof(stratSequense) / sizeof(stratSequense[0]);

uint32_t now_bits = 0;
static ask_com_t *CommunicationStatus = NULL;

static void ask_debug_set_pb3(uint8_t level)
{
    if (level != 0U)
    {
        GPIOB->BSRR = GPIO_PIN_3;
    }
    else
    {
        GPIOB->BSRR = (uint32_t)GPIO_PIN_3 << 16U;
    }
}

void ASK_Decode(ask_com_t *ins)
{
    for (uint8_t i = 0; i < ins->communicationBufferLength; i++)
    {
        now_bits = ins->communicationBuffer[i];

        if (ins->communicationBuffer[i] > ins->dynamicMax)
        {
            ins->dynamicMax = ins->dynamicMax * 0.2f + ins->communicationBuffer[i] * 0.8f;
        }
        else
        {
            ins->dynamicMax = (COMMUNICATION_MIDDLE_THRESHOLD + 200) * 0.003f + ins->dynamicMax * 0.997f;
        }

        if (ins->communicationBuffer[i] < ins->dynamicMin)
        {
            ins->dynamicMin = ins->dynamicMin * 0.2f + ins->communicationBuffer[i] * 0.8f;
        }
        else
        {
            ins->dynamicMin = (COMMUNICATION_MIDDLE_THRESHOLD - 200) * 0.005f + ins->dynamicMin * 0.995f;
        }

        ins->upperThreshold = COMMUNICATION_MIDDLE_THRESHOLD * 0.3f + ins->dynamicMax * 0.7f;
        ins->lowerThreshold = COMMUNICATION_MIDDLE_THRESHOLD * 0.3f + ins->dynamicMin * 0.7f;

        if (DelayedTrigger_Update(&ins->upperDelayedTrigger, ins->communicationBuffer[i] > ins->upperThreshold))
        {
            DelayedTrigger_Reset(&ins->lowerDelayedTrigger);

            if (ins->currentBitLevel == 0U)
            {
                ins->currentBitLevel = 1U;
                ask_debug_set_pb3(1U);
                commuResultFlipCallback(1U);
            }
        }

        if (DelayedTrigger_Update(&ins->lowerDelayedTrigger, ins->communicationBuffer[i] < ins->lowerThreshold))
        {
            DelayedTrigger_Reset(&ins->upperDelayedTrigger);

            if (ins->currentBitLevel == 1U)
            {
                ins->currentBitLevel = 0U;
                ask_debug_set_pb3(0U);
                commuResultFlipCallback(0U);
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
            ins->isConnected = 0U;
            ins->isValid = 0U;
        }
    }
}

void ask_init(ask_com_t *ins, uint16_t *Buffer, uint8_t BufferLength)
{
    ins->communicationBufferLength = BufferLength;
    ins->communicationBuffer = Buffer;
    ins->currentBitLevel = 0U;

    ins->_40BitsBufferPointer = 0U;

    ins->dynamicMax = COMMUNICATION_MIDDLE_THRESHOLD;
    ins->dynamicMin = COMMUNICATION_MIDDLE_THRESHOLD;
    ins->upperThreshold = COMMUNICATION_MIDDLE_THRESHOLD + COMMUNICATION_UPPER_THRESHOLD;
    ins->lowerThreshold = COMMUNICATION_MIDDLE_THRESHOLD - COMMUNICATION_LOWER_THRESHOLD;

    ins->disconnectCounter = 0U;
    ins->isConnected = 0U;
    ins->isValid = 0U;

    DelayedTrigger_Init(&ins->lowerDelayedTrigger, COMMUNICATION_TRIGGER_TIMEOUT, 1U, 1U);
    DelayedTrigger_Init(&ins->upperDelayedTrigger, COMMUNICATION_TRIGGER_TIMEOUT, 1U, 1U);

    CommunicationStatus = ins;
    ask_debug_set_pb3(0U);
    HAL_TIM_Base_Init(&COMUNICATION_TIM);
    HAL_TIM_Base_Start(&COMUNICATION_TIM);
    __HAL_TIM_SET_COUNTER(&COMUNICATION_TIM, 0U);
}

void commuResultFlipCallback(uint8_t lastLevel)
{
    uint32_t dt = __HAL_TIM_GetCounter(&COMUNICATION_TIM);
    __HAL_TIM_SET_COUNTER(&COMUNICATION_TIM, 0U);

    if (dt < 125U || dt > 625U) // 合法窗口
    {
        CommunicationStatus->_40BitsBufferPointer = 0U;
        return;
    }

    if (dt < 375U)
    {
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
    if (CommunicationStatus->_40BitsBufferPointer < stratSequenseLength)
    {
        if (stratSequense[CommunicationStatus->_40BitsBufferPointer] == lastLevel)
        {
            CommunicationStatus->_40BitsBufferPointer += 1U;
        }
        else
        {
            CommunicationStatus->_40BitsBufferPointer = 0U;
        }
    }
    else
    {
        CommunicationStatus->data20Bits[CommunicationStatus->_40BitsBufferPointer - stratSequenseLength] =
            lastLevel;
        CommunicationStatus->_40BitsBufferPointer += 1U;
    }

    if (CommunicationStatus->_40BitsBufferPointer == 40U)
    {
        CommunicationStatus->_40BitsBufferPointer = 0U;
        decode20BitsBuffer();
    }
}

void decode20BitsBuffer(void)
{
    CommunicationStatus->disconnectCounter = 0U;
    CommunicationStatus->isConnected = 1U;

    for (uint8_t i = 0U; i < 9U; i++)
    {
        if (CommunicationStatus->data20Bits[i * 2U + 1U] == CommunicationStatus->data20Bits[i * 2U + 2U])
        {
            CommunicationStatus->isValid = 0U;
            return;
        }
    }

    for (uint8_t i = 0U; i < 10U; i++)
    {
        if (CommunicationStatus->data20Bits[i * 2U] == CommunicationStatus->data20Bits[i * 2U + 1U])
        {
            CommunicationStatus->raw10Bit[i] = 0U;
        }
        else
        {
            CommunicationStatus->raw10Bit[i] = 1U;
        }
    }

    uint8_t requiredPowerSelection = CommunicationStatus->raw10Bit[0];
    uint8_t rawPowerFeedback = 0U;
    float powerFeedback = 0.0f;

    for (uint8_t i = 0U; i < 8U; i++)
    {
        rawPowerFeedback |= (uint8_t)(CommunicationStatus->raw10Bit[i + 1U] << i);
    }

    // uint16_t parity = requiredPowerSelection | (rawPowerFeedback << 1U);
    // parity ^= (parity >> 8U);
    // parity ^= (parity >> 4U);
    // parity ^= (parity >> 2U);
    // parity ^= (parity >> 1U);
    // if ((parity & 0x01U) != CommunicationStatus->raw10Bit[9])
    // {
    //     CommunicationStatus->isValid = 0U;
    //     return;
    // }

    powerFeedback = (float)rawPowerFeedback / 255.0f * 150.0f;

    CommunicationStatus->backwardCommunicationData.requiredPowerSelection = requiredPowerSelection;
    CommunicationStatus->backwardCommunicationData.rawPowerFeedback = rawPowerFeedback;
    CommunicationStatus->backwardCommunicationData.powerFeedback = powerFeedback;
    CommunicationStatus->backwardCommunicationData.transmitEfficiency =
        0.5f * CommunicationStatus->backwardCommunicationData.transmitEfficiency;

    CommunicationStatus->isValid = 1U;
}
