#ifndef VISION_HPP
#define VISION_HPP
#include "bsp_usart.hpp"
#include "safe_task.hpp"
#include <cstdint>

namespace VISION_n {
/************VISION SETTING************** */
#define VISION_TX_HEAD 0xA1
#define VISION_TX_TAIL 0xA2
#define VISION_TX_LENGTH (sizeof(VisionTx_t))
#define VISION_RX_HEAD 0xB1
#define VISION_RX_TAIL 0xB2
#define VISION_RX_LENGTH (sizeof(VisionRx_t))
#pragma pack(push, 1)
struct VisionTx_t {
  uint8_t head;
  uint8_t run_cmd; // 是否进行识别
  float pitch;     // 当前pitch
  float yaw;
  // float roll;
  uint8_t tail;
};
struct VisionRx_t {
  uint8_t head;
  uint8_t is_detect;
  float pitch_cmd;
  float yaw_cmd;
  uint8_t tail;
};
#pragma pack(pop) // 恢复之前保存的对齐方式

class Vision_c {
public:
  void Init();
  inline VisionTx_t *GetTx() { return &VisionTx; }
  inline VisionRx_t *GetRx() { return &VisionRx; }
  void Send(bool cmd, float yaw, float pitch);
  void Receive(const uint8_t *data, uint16_t size);
  bool get_is_detete();
  static void is_online();
  static void is_disconnect();
  static Vision_c *GetInstance() {
    if (instance == nullptr) {
      instance = new Vision_c();
    }
    return instance;
  }
  Vision_c() {}

private:
  bool is_online_flag = false;
  SafeTask_c *safe_detect;
  static Vision_c *instance;
  VisionRx_t VisionRx = {};
  VisionTx_t VisionTx = {};
  BSP_n::USART_c *VisionCom = nullptr;
  uint16_t sendHz = 100;
};
}; // namespace VISION_n

#endif // !VISION_HPP
