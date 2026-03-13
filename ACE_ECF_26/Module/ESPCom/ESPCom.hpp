#ifndef ESP_COM_HPP
#define ESP_COM_HPP
#include "bsp_usart.hpp"
#include "safe_task.hpp"
namespace ESPCom_n {
/* 协议常量 ------------------------------------------------------------ */
#define ESPCom_TX_HEAD 0xD5
#define ESPCom_TX_TAIL 0x5D
#define ESPCom_RX_HEAD 0xB5
#define ESPCom_RX_TAIL 0x5B

/* VOFA+ RawData 尾部魔数（小端 inf）---------------------------------- */
#define VOFA_TAIL 0x7f800000UL // 4 字节 00 00 80 7f (AI写的)

#pragma pack(push, 1)
/*
cmd：0x00 停止采集；0x01 yaw采集；0x02 roll; 0x03 pitch; 0x04 加速度
dataL: 0x01 闭环采集；0x02 开环采集；0x03 阶跃采集；0x04 sin采集
* */
struct DapTx_t // 7 字节，
{
  uint8_t head;   // 0xD5
  uint8_t status; // 数据源
  uint8_t data;   // 查看形式
  uint8_t tail;   // 0x5D
};

struct DapRx_t // 7 字节，上位机→下位机命令帧（原结构保留）
{
  uint8_t head; // 0xB5
  uint8_t cmd;
  uint8_t data;
  uint8_t tail; // 0x5B
};

/* VOFA+ 波形帧：4 通道 float + 4 字节尾 -------------------------------- */
// #pragma pack(push, 1)
struct VofaWave_t {
  float ch1;
  float ch2;
  float ch3;
  float ch4;
  uint32_t tail = VOFA_TAIL; // 00 00 80 7f
};
/* VOFA+ 波形帧：最大10通道 float + 4 字节尾 --------------------------- */
struct VofaWaveMax_t {
  float channels[11];        // 支持最多11个通道
  uint32_t tail = VOFA_TAIL; // 00 00 80 7f
};
#pragma pack(pop)

/* 单例 ----------------------------------------------------------------- */
class ESPCom_c {
public:
  void Init();
  inline DapTx_t *GetTx() { return &DapTx; }
  inline DapRx_t *GetRx() { return &DapRx; }
  void Send(uint8_t cmd, uint32_t data);
  void SendWave(const float *channel_data, uint8_t channel_count);
  void SendWave(float ch1, float ch2, float ch3, float ch4);
  void Receive(const uint8_t *data, uint16_t size);
  static ESPCom_c *GetInstance() {
    if (instance == nullptr)
      instance = new ESPCom_c();
    return instance;
  }
  static void is_disconnect();
  static void is_online();
  bool GetIsDisConnect();

private:
  static ESPCom_c *instance;
  DapTx_t DapTx = {};
  DapRx_t DapRx = {};
  BSP_n::USART_c *DapCom = nullptr;
  SafeTask_c *safe_detect;
  /* 新增 VOFA 波形缓冲区 */
  VofaWave_t VofaWave = {};
  VofaWaveMax_t VofaWaveMax = {};
};
} // namespace ESPCom_n
#endif