#ifndef WPC_CHARGE_CTRL_HPP
#define WPC_CHARGE_CTRL_HPP

#include <array>
#include <cstdint>

extern "C" {
#include "adc.h"
#include "hrtim.h"
#include "tim.h"
}

#include "incremental_pid.hpp"
#include "wpc_ask_demod.hpp"
#include "wpc_ask_proto_decoder.hpp"
#include "wpc_fod_gate.hpp"

namespace wpc {

struct ChargeCtrlConfig {
  // 探测态和充电态的初始移相（归一化到半周期 0~1）。
  float probe_phase_shift = 0.05f;
  float charge_phase_shift = 0.30f;

  // 功率目标（W）。
  float probe_power_w = 5.0f;
  float charge_power_w = 120.0f;

  // 功率低通滤波系数。
  float power_lpf_alpha = 0.08f;

  // 增量式 PID 参数。
  float pid_kp = 0.00005f;
  float pid_ki = 0.000002f;
  float pid_kd = 0.0f;

  // 移相控制范围（归一化到半周期 0~1）。
  float phase_shift_min = 0.0f;
  float phase_shift_max = 0.95f;

  // 连续有效协议包数量达到该值后，才触发充电门控。
  uint8_t valid_frames_required = 2U;

  uint32_t hrtim_period = 27200U;
};

class WirelessChargeController {
 public:
  static WirelessChargeController& Instance();
  void Init();
  void OnAdcConvCplt(ADC_HandleTypeDef* hadc);
  void OnTimPeriodElapsed(TIM_HandleTypeDef* htim);

 private:
  // pre* 仅用于“进入态一次性动作”，避免重复设置。
  enum class ControlState : uint8_t {
    kPreProbeLowPower = 0,
    kProbeLowPower,
    kPreCharging,
    kCharging,
    kPreRetryGap,
    kRetryGap,
  };

  WirelessChargeController();
  void ApplyState(FodGateState state);
  void SetOutputsEnabled(bool enable);
  void SetPhaseShift(float phase_shift);
  void ProcessAdc3AskSamples(uint32_t now_ms);
  void UpdatePowerEstimate();
  void RunPowerClosedLoop(FodGateState state);

  static constexpr size_t kAdc2DmaLen = 16;
  static constexpr size_t kAdc3DmaLen = 12;
  static constexpr uint8_t kTim6Divider = 10;  // TIM6(10kHz) -> 1kHz 慢环

  std::array<uint16_t, kAdc2DmaLen> adc2_dma_{};
  std::array<uint16_t, kAdc3DmaLen> adc3_dma_{};

  AskDemodulator ask_demod_;
  AskProtoDecoder ask_proto_decoder_;
  FodGate fod_gate_;
  ChargeCtrlConfig cfg_{};

  FodGateState requested_state_ = FodGateState::kRetryGap;
  ControlState control_state_ = ControlState::kPreRetryGap;

  bool skip_loop_once_ = false;
  bool outputs_enabled_ = false;
  bool initialized_ = false;
  uint8_t tim6_div_count_ = 0;
  uint8_t valid_frame_streak_ = 0U;

  float filtered_power_w_ = 0.0f;
  float latest_voltage_v_ = 0.0f;
  float latest_current_a_ = 0.0f;

  control::IncrementalPid power_pid_{};
};

}  // namespace wpc

#endif  // WPC_CHARGE_CTRL_HPP