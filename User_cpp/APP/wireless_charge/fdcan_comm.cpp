#include "fdcan_comm.h"

extern "C"
{
#include "fdcan.h"
#include "main.h"
}

namespace
{

    // ---- FDCAN2 协议 ----
    // G4 → F3: 0x210 功率指令 (power_w × 100)
    // F3 → G4: 0x211 状态数据 (Vbat,Vcap,Ibat,Icap × 100)
    constexpr uint32_t kTxId = 0x211U; // G4(接收端)→F3(超容) 功率指令
    constexpr float kPowerMinW = 20.0f;
    constexpr float kPowerMaxW = 200.0f;

    // ---- 接收到的 F3 状态 ----
    volatile float g_vbat = 0.0f;
    volatile float g_vcap = 0.0f;
    volatile float g_ibat = 0.0f;
    volatile float g_icap = 0.0f;
    volatile bool g_updated = false;

    // ---- 滤波器: 接受全部标准帧到 FIFO0 ----
    void configureFilter(void)
    {
        FDCAN_FilterTypeDef filter = {};
        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterType = FDCAN_FILTER_RANGE;
        filter.FilterIndex = 0;
        filter.FilterID1 = 0x000;
        filter.FilterID2 = 0x7FF;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

        if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK)
        {
            Error_Handler();
        }
    }

    // ---- 打包功率指令 ----
    void packPowerCmd(uint8_t data[8], float powerW)
    {
        if (powerW < kPowerMinW)
            powerW = kPowerMinW;
        if (powerW > kPowerMaxW)
            powerW = kPowerMaxW;

        int16_t raw = static_cast<int16_t>(powerW * 100.0f);
        data[0] = static_cast<uint8_t>((raw >> 8) & 0xFF);
        data[1] = static_cast<uint8_t>(raw & 0xFF);
        data[2] = 0; // 充电使能标志
        data[3] = 0;
        data[4] = 0;
        data[5] = 0;
        data[6] = 0;
        data[7] = 0;
    }

} // namespace


// ---- 外部可调功率指令 ----
static float g_power_cmd_w = 20.0f; // 默认 20W
static uint8_t g_charge_enable = 0; // 充电使能: 0=禁止, 1=允许

// ---- 对外接口: 设置充电使能 ----
extern "C" void fdcan_comm_set_charge_enable(uint8_t enable)
{
    g_charge_enable = enable;
}

// ---- 对外接口: 初始化 ----
extern "C" void fdcan_comm_init(void)
{
    // 重新初始化以清空 TX FIFO 残余（若有）
    // 注意: 必须设置 CCE 才能清除 INIT, 否则 CCCR 写操作无效
    // hfdcan2.Instance->CCCR |= FDCAN_CCCR_INIT;
    // while ((hfdcan2.Instance->CCCR & FDCAN_CCCR_INIT) == 0U) {}
    // hfdcan2.Instance->CCCR |= FDCAN_CCCR_CCE;  // 设置 CCE 后才能修改 INIT
    // hfdcan2.Instance->CCCR &= ~FDCAN_CCCR_INIT;
    // while ((hfdcan2.Instance->CCCR & FDCAN_CCCR_INIT) != 0U) {}

    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    configureFilter();
}

// ---- 对外接口: 发送功率指令(由 TIM6 1kHz 调用) ----
extern "C" void fdcan_comm_send_power_cmd(float powerW)
{
    g_power_cmd_w = powerW;

    // ---- 主动检查 Bus-Off 状态并恢复 ----
    // Bus-Off 时硬件自动置 INIT=1, 控制器无法收发.
    // 标准恢复: Stop(进入 INIT) → Start(清除 INIT) → 硬件自动等待 128 个隐性位后恢复
    if (hfdcan2.Instance->PSR & FDCAN_PSR_BO_Msk)
    {
        HAL_FDCAN_Stop(&hfdcan2);
        HAL_FDCAN_Start(&hfdcan2);
        // Stop 时禁用了中断, 恢复后重新使能 RX 通知
        HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        return;
    }

    uint8_t data[8];
    packPowerCmd(data, powerW);

    static FDCAN_TxHeaderTypeDef txHeader = {};
    txHeader.Identifier = kTxId;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader, data);
    }
    // else: TX FIFO 满, 丢弃当前帧 (下次发送自然覆盖旧消息)
}

// ---- 对外接口: 读取接收到的 F3 状态 ----
extern "C" float fdcan_comm_get_vbat(void)
{
    return g_vbat;
}
extern "C" float fdcan_comm_get_vcap(void)
{
    return g_vcap;
}
extern "C" float fdcan_comm_get_ibat(void)
{
    return g_ibat;
}
extern "C" float fdcan_comm_get_icap(void)
{
    return g_icap;
}
extern "C" int fdcan_comm_is_updated(void)
{
    int v = g_updated ? 1 : 0;
    g_updated = false;
    return v;
}

// ---- 解析 0x211 状态帧 ----
static void parseStatusFrame(const uint8_t data[8])
{
    int16_t vbat = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    int16_t vcap = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);
    int16_t ibat = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]);
    int16_t icap = static_cast<int16_t>((static_cast<uint16_t>(data[6]) << 8) | data[7]);

    g_vbat = static_cast<float>(vbat) / 100.0f;
    g_vcap = static_cast<float>(vcap) / 100.0f;
    g_ibat = static_cast<float>(ibat) / 100.0f;
    g_icap = static_cast<float>(icap) / 100.0f;
    g_updated = true;
}

// ---- HAL FDCAN 接收中断回调 ----
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, data) == HAL_OK)
    {
        // 0x210: F3 → G4 状态帧; 0x211: 自回环测试 (loopback 发收同 ID)
        if (rxHeader.Identifier == 0x211) // 只接收超容→接收端的状态
        {
            parseStatusFrame(data);
        }
    }
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    (void)RxFifo1ITs;
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rxHeader, data) == HAL_OK)
    {
        // 只接收超容→接收端的状态
        if (rxHeader.Identifier == 0x211)
        {
            parseStatusFrame(data);
        }
    }
}
