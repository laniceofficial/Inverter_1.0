// openocd -f dart_ESPCom.cfg
// 使用ESP32S3 作为ESPCom调试，同时兼具USART上位机串口收发功能
#include "ESPCom.hpp"
#include <cstdint>

namespace ESPCom_n {

ESPCom_c *ESPCom_c::instance = nullptr;

/* 上层 DMA 空闲中断回调 ----------------------------------------------------*/
static uint8_t rx_buffer[2 * sizeof(DapRx_t)]; // 双缓冲保险

static void ESPComRxCallback(const uint8_t *data, uint16_t size) {
  if (data[0] == ESPCom_RX_HEAD && data[size - 1] == ESPCom_RX_TAIL &&
      size == sizeof(DapRx_t)) {
    ESPCom_n::ESPCom_c::GetInstance()->Receive(data, size);
  }
}

/* 初始化 USART3 ，DMA 发送 + 空闲中断接收 ---------------------------------*/
void ESPCom_c::Init() {
  DapCom = new BSP_n::USART_c(&huart3,
                              2 * sizeof(DapRx_t), // DMA 接收缓冲区长度
                              sizeof(DapRx_t),     // 一帧长度
                              BSP_n::USART_c::TxType_t::DMA,
                              BSP_n::USART_c::RxType_t::DMA_IDLE, rx_buffer,
                              nullptr,            // 无发送完成回调
                              &ESPComRxCallback); // 接收完成回调

  DapTx.head = ESPCom_TX_HEAD;
  DapTx.tail = ESPCom_TX_TAIL;
  DapRx.head = ESPCom_RX_HEAD;
  DapRx.tail = ESPCom_RX_TAIL;
  DapRx.cmd = 0;
  DapRx.data = 0;
  // 规定10s必须发一帧，不然自动停
  safe_detect = new SafeTask_c("ESPcom", 5000, &ESPCom_c::is_disconnect, &ESPCom_c::is_online);
}

/* 打包并 DMA 发送一帧 ------------------------------------------------------*/
void ESPCom_c::Send(uint8_t status, uint32_t data) {
  DapTx.status = status;
  DapTx.data = data;
  DapCom->send((uint8_t *)&DapTx, sizeof(DapTx));
}

/* 发送波形数据 -------------------------------------------------------------*/
void ESPCom_c::SendWave(float ch1, float ch2, float ch3, float ch4) {
  VofaWave.ch1 = ch1;
  VofaWave.ch2 = ch2;
  VofaWave.ch3 = ch3;
  VofaWave.ch4 = ch4;
  DapCom->send((uint8_t *)&VofaWave, sizeof(VofaWave));
}

void ESPCom_c::SendWave(const float *channel_data, uint8_t channel_count) {
  channel_count = channel_count > 11 ? 11 : channel_count;
  for (uint8_t i = 0; i < channel_count; i++) {
    VofaWaveMax.channels[i] = channel_data[i];
  }
  DapCom->send((uint8_t *)&VofaWaveMax, sizeof(VofaWaveMax));
}
/* 解包保存到成员变量 -------------------------------------------------------*/
void ESPCom_c::Receive(const uint8_t *data, uint16_t size) {
  safe_detect->Online();
  memcpy(&DapRx, data, size);
}
void ESPCom_c::is_disconnect(void) {}
void ESPCom_c::is_online(void) {}
bool ESPCom_c::GetIsDisConnect() // 获取连接状态，true为失联
    {
      return safe_detect->disconnection_falg;  
  }
} // namespace ESPCom_n