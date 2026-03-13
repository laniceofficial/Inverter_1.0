// #include "task.h"

// #include "wpc_charge_ctrl.hpp"

// extern "C" void task_init() {
//   // main.c 到 C++ 控制器的统一入口。
//   wpc::WirelessChargeController::Instance().Init();
// }

// extern "C" void task_loop() {
// }

// extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//   // HAL 的 C 回调桥接到 C++ 控制器。
//   wpc::WirelessChargeController::Instance().OnAdcConvCplt(hadc);
// }

// extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
//   // HAL 的 C 回调桥接到 C++ 控制器。
//   wpc::WirelessChargeController::Instance().OnTimPeriodElapsed(htim);
// }