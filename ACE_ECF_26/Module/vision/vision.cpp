#include "vision.hpp"
#include <cstdint>
namespace VISION_n
{
    Vision_c *Vision_c::instance;
    // 小端排列
    void Vision_c::Receive(const uint8_t *data, uint16_t size) {
        safe_detect->Online();
        uint8_t data_buff[VISION_RX_LENGTH] = {0};
        memcpy(&data_buff, data, size);
        VisionRx = *(VisionRx_t *)data_buff;
    }

    void Vision_c::Send(bool cmd, float yaw, float pitch)
    {
        VisionTx.run_cmd = cmd;
        VisionTx.yaw = yaw;
        VisionTx.pitch = pitch;
        // VisionTx.roll = ins->Roll;
        static uint8_t cnt_ = 0;
        if (1000 / cnt_ == sendHz) {
          VisionCom->send((uint8_t *)&VisionTx, VISION_TX_LENGTH);
          cnt_ = 0;
        }
        cnt_++;
    }
    void VisionCallBack(const uint8_t *data, uint16_t size)
    {
        if (data[0] == VISION_RX_HEAD && data[size - 1] == VISION_RX_TAIL && size == VISION_RX_LENGTH)
        {
          // Vision = *(VisionRx_t *)data;
            
            VISION_n::Vision_c::GetInstance()->Receive(data, size);
        }
    }
    static uint8_t rx_buffer[2 * VISION_RX_LENGTH];
    void Vision_c::Init()
    {

        VisionCom = new BSP_n::USART_c(&huart1, 2 * VISION_RX_LENGTH, VISION_RX_LENGTH,
                                       BSP_n::USART_c::TxType_t::DMA,
                                       BSP_n::USART_c::RxType_t::DMA_IDLE, rx_buffer,
                                       nullptr, &VisionCallBack);
        VisionTx.head = VISION_TX_HEAD;
        VisionTx.tail = VISION_TX_TAIL;
        VisionRx.head = VISION_RX_HEAD;
        VisionRx.tail = VISION_RX_TAIL;
        safe_detect = new SafeTask_c("vision",1000,&Vision_c::is_disconnect,&Vision_c::is_online);
    }
    void Vision_c::is_online() {
      VISION_n::Vision_c::GetInstance()->is_online_flag = true;
    }
    void Vision_c::is_disconnect() {
      VISION_n::Vision_c::GetInstance()->is_online_flag = false;
      VISION_n::Vision_c::GetInstance()->GetRx()->is_detect = false;
    }
    bool Vision_c::get_is_detete() {
        return VisionRx.is_detect;
      }
}