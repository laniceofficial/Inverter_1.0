#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// 由 fdcan_comm.cpp 实现, 供 wireless_charge / TIM6 回调使用
void fdcan_comm_init(void);
void fdcan_comm_send_status(void);
float fdcan_get_power_target(void);
int  fdcan_is_data_updated(void);

#ifdef __cplusplus
}
#endif
