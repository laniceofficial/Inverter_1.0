#include "fdcan_comm.h"

#include "sampling.hpp"

extern "C"
{
#include "fdcan.h"
#include "main.h"
}

namespace
{

// ---- FDCAN2 配置 ----
constexpr uint32_t kTxId = 0x211U; // 本机 → 对端: 电压电流状态

// ---- 滤波器: 接受 0x000–0x7FF 全部标准帧到 FIFO0 ----
void configureFilter(void)
{
    FDCAN_FilterTypeDef filter = {};
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterType   = FDCAN_FILTER_RANGE;
    filter.FilterIndex  = 0;
    filter.FilterID1    = 0x000;
    filter.FilterID2    = 0x7FF;
#ifdef STM32H723xx
    filter.RxBufferIndex    = 0;
    filter.IsCalibrationMsg = 0;
#endif
    filter.FilterConfig     = FDCAN_FILTER_TO_RXFIFO0;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK)
    {
        Error_Handler();
    }
}

// ---- 填充 CAN 帧 ----
void packStatus(uint8_t data[8])
{
    const auto& s = App::samplingService();
    int16_t vbat = static_cast<int16_t>(s.getHalfBridgeInputVoltage()  * 100.0f);
    int16_t vcap = static_cast<int16_t>(s.getHalfBridgeOutputVoltage() * 100.0f);
    int16_t ibat = static_cast<int16_t>(s.getHalfBridgeCurrent()       * 100.0f);

    data[0] = static_cast<uint8_t>((vbat >> 8) & 0xFF);
    data[1] = static_cast<uint8_t>( vbat       & 0xFF);
    data[2] = static_cast<uint8_t>((vcap >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>( vcap       & 0xFF);
    data[4] = static_cast<uint8_t>((ibat >> 8) & 0xFF);
    data[5] = static_cast<uint8_t>( ibat       & 0xFF);
    data[6] = 0;
    data[7] = 0;
}

} // namespace


// ---- 对外接口 ----
extern "C" void fdcan_comm_init(void)
{
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    configureFilter();
}

extern "C" void fdcan_comm_send_status(void)
{
    uint8_t data[8];
    packStatus(data);

    FDCAN_TxHeaderTypeDef txHeader = {};
    txHeader.Identifier         = kTxId;
    txHeader.IdType             = FDCAN_STANDARD_ID;
    txHeader.TxFrameType        = FDCAN_DATA_FRAME;
    txHeader.DataLength         = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch      = FDCAN_BRS_OFF;
    txHeader.FDFormat           = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker      = 0;

    // 非阻塞发送: 邮箱满时直接丢弃本帧, 下一帧会覆盖
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader, data);
    }
}
