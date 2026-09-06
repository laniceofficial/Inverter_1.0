#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
// FDCAN 板间通信: G4(接收端上位机) ←→ F3(四开关 buck-boost 控制器)
//   G4 → F3: 0x210 功率指令 (power_w × 100, int16)
//   F3 → G4: 0x211 状态数据 (Vbat,Vcap,Ibat,Icap × 100, int16)

void fdcan_comm_init(void);
void fdcan_comm_send_power_cmd(float power_w);   // 1kHz 发送功率指令
void fdcan_comm_set_charge_enable(uint8_t enable); // 设置充电使能标志 (0=禁止, 1=允许)
float fdcan_comm_get_vbat(void);                  // 底盘电压 (V)
float fdcan_comm_get_vcap(void);                  // 超级电容电压 (V)
float fdcan_comm_get_ibat(void);                  // 底盘输入电流 (A)
float fdcan_comm_get_icap(void);                  // 电容电流 (A)
int   fdcan_comm_is_updated(void);                // 有新数据时为 1

#ifdef __cplusplus
}
#endif
