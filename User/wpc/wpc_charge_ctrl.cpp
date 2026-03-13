#include "wpc_charge_ctrl.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace wpc {

namespace {
constexpr float kAdcRefVoltage = 3.309f;  // ADC 参考电压
constexpr float kAdcMaxValue = 4095.0f;
constexpr float kCurrentOffset = 2.49f;
constexpr float kCurrentFactor = 8.0f;
constexpr float kVoltageOffset = 1.60f;
constexpr float kVoltageFactor = 18.9f;
}  // namespace

WirelessChargeController& WirelessChargeController::Instance() {
  static WirelessChargeController instance;
  return instance;
}

WirelessChargeController::WirelessChargeController()
    : ask_demod_(AskDemodConfig{}),
      ask_proto_decoder_(AskProtoDecoderConfig{}),
      fod_gate_(FodGateConfig{}) {
}

void WirelessChargeController::Init() {
  if (initialized_) {
    return;
  }

  // 保持 HRTIM 计数器运行，保证 ADC 触发和 PWM 更新时序稳定。
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_E);
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_F);

  // ADC 校准并启动 DMA 循环采样。
  while (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
  }
  while (HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED) != HAL_OK) {
  }
  HAL_ADC_Start_DMA(&hadc2, reinterpret_cast<uint32_t*>(adc2_dma_.data()), kAdc2DmaLen);
  HAL_ADC_Start_DMA(&hadc3, reinterpret_cast<uint32_t*>(adc3_dma_.data()), kAdc3DmaLen);

  // TIM6 作为慢速管理节拍（约 1kHz）。
  HAL_TIM_Base_Start_IT(&htim6);

  const uint32_t now_ms = HAL_GetTick();
  ask_demod_.Reset();
  ask_proto_decoder_.Reset();
  valid_frame_streak_ = 0U;
  fod_gate_.Reset(now_ms);

  requested_state_ = FodGateState::kRetryGap;
  control_state_ = ControlState::kPreRetryGap;
  skip_loop_once_ = false;

  power_pid_.Configure(cfg_.pid_kp, cfg_.pid_ki, cfg_.pid_kd, cfg_.phase_shift_min,
                       cfg_.phase_shift_max);
  power_pid_.Reset(cfg_.probe_phase_shift);

  ApplyState(fod_gate_.state());
  initialized_ = true;
}

void WirelessChargeController::OnAdcConvCplt(ADC_HandleTypeDef* hadc) {
  if (!initialized_) {
    return;
  }
  if (hadc != &hadc3 && hadc != &hadc2) {
    return;
  }

  const uint32_t now_ms = HAL_GetTick();
  if (hadc == &hadc3) {
    ProcessAdc3AskSamples(now_ms);
  }

  UpdatePowerEstimate();
  fod_gate_.Update(now_ms);
  ApplyState(fod_gate_.state());
}

void WirelessChargeController::OnTimPeriodElapsed(TIM_HandleTypeDef* htim) {
  if (!initialized_ || htim != &htim6) {
    return;
  }

  tim6_div_count_++;
  if (tim6_div_count_ < kTim6Divider) {
    return;
  }
  tim6_div_count_ = 0;

  const uint32_t now_ms = HAL_GetTick();
  fod_gate_.Update(now_ms);
  ApplyState(fod_gate_.state());

  if (skip_loop_once_) {
    // 进入态刚完成一次性设置时，跳过本周期闭环，避免同周期覆盖入口设定。
    skip_loop_once_ = false;
    return;
  }

  RunPowerClosedLoop(fod_gate_.state());
}

void WirelessChargeController::ProcessAdc3AskSamples(uint32_t now_ms) {
  // ADC3 顺序: [IN1, IN5, IN12] x4，ASK 输入是 IN5(PB13/DC1_V)。
  for (size_t idx = 1U; idx < kAdc3DmaLen; idx += 3U) {
    AskSampleResult result = ask_demod_.PushSample(adc3_dma_[idx], now_ms);

    if (result.transition_bit == 0U) {
      continue;
    }

    const AskProtoDecodeResult decode =
        ask_proto_decoder_.OnTransition(result.level, result.delta_samples);
    result.frame_valid_pulse = decode.frame_valid_pulse;

    if (decode.frame_valid_pulse) {
      if (valid_frame_streak_ < 0xFFU) {
        valid_frame_streak_++;
      }
      const uint8_t required = (cfg_.valid_frames_required == 0U) ? 1U : cfg_.valid_frames_required;
      if (valid_frame_streak_ >= required) {
        fod_gate_.OnAskValid(now_ms);
      }
    } else if (decode.frame_invalid_pulse) {
      valid_frame_streak_ = 0U;
    }
  }
}

void WirelessChargeController::ApplyState(FodGateState state) {
  // 外部状态变化时，先切到对应 pre 状态，执行一次性入口动作。
  if (state != requested_state_) {
    requested_state_ = state;
    switch (state) {
      case FodGateState::kProbeLowPower:
        control_state_ = ControlState::kPreProbeLowPower;
        break;
      case FodGateState::kCharging:
        control_state_ = ControlState::kPreCharging;
        break;
      case FodGateState::kRetryGap:
        control_state_ = ControlState::kPreRetryGap;
        break;
    }
  }

  switch (control_state_) {
    case ControlState::kPreProbeLowPower:
      power_pid_.Reset(cfg_.probe_phase_shift);
      SetPhaseShift(cfg_.probe_phase_shift);
      ask_proto_decoder_.Reset();
      valid_frame_streak_ = 0U;
      SetOutputsEnabled(true);
      control_state_ = ControlState::kProbeLowPower;
      skip_loop_once_ = true;
      break;

    case ControlState::kPreCharging:
      power_pid_.Reset(cfg_.charge_phase_shift);
      SetPhaseShift(cfg_.charge_phase_shift);
      SetOutputsEnabled(true);
      control_state_ = ControlState::kCharging;
      skip_loop_once_ = true;
      break;

    case ControlState::kPreRetryGap:
      SetOutputsEnabled(false);
      ask_proto_decoder_.Reset();
      valid_frame_streak_ = 0U;
      control_state_ = ControlState::kRetryGap;
      skip_loop_once_ = true;
      break;

    case ControlState::kProbeLowPower:
    case ControlState::kCharging:
    case ControlState::kRetryGap:
      break;
  }
}

void WirelessChargeController::SetOutputsEnabled(bool enable) {
  if (outputs_enabled_ == enable) {
    return;
  }

  if (enable) {
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
  } else {
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
  }

  outputs_enabled_ = enable;
}

void WirelessChargeController::SetPhaseShift(float phase_shift) {
  // 移相归一化到半周期：0=同相最小功率，接近 1 时接近最大功率。
  phase_shift = std::clamp(phase_shift, cfg_.phase_shift_min, cfg_.phase_shift_max);

  const uint32_t period = cfg_.hrtim_period;
  const uint32_t half = period / 2U;
  if (period < 8U || half < 4U) {
    return;
  }

  // 固定 50% 占空，通过移动 F 桥臂相位实现功率调节，E 桥臂作为基准。
  const uint32_t base_on = 1U;
  const uint32_t base_off = base_on + half;

  const uint32_t shift_max_counts = half - 2U;
  const uint32_t shift_counts =
      static_cast<uint32_t>(phase_shift * static_cast<float>(shift_max_counts));

  const uint32_t e_on = base_on;
  const uint32_t e_off = base_off;
  const uint32_t f_on = base_on + shift_counts;
  const uint32_t f_off = base_off + shift_counts;

  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, e_on);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, e_off);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_1, f_on);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_3, f_off);
}
float v_s=0;
void WirelessChargeController::UpdatePowerEstimate() {
  // ADC2: [IN4, IN5, IN11, IN12] x4，AC1_I 在 IN12 -> 3/7/11/15。
  uint32_t sum_i = 0U;
  uint32_t sum_v_test = 0U;
  for (size_t i = 3U; i < kAdc2DmaLen; i += 4U) {
    sum_i += adc2_dma_[i];
  }
  for (size_t i = 0U; i < kAdc2DmaLen; i += 4U) {
    sum_v_test += adc2_dma_[i];
  }
  const float adc_i_avg = static_cast<float>(sum_i) / 4.0f;

  // ADC3: [IN1, IN5, IN12] x4，AC1_V 在 IN1 -> 0/3/6/9。
  uint32_t sum_v = 0U;
  for (size_t i = 0U; i < kAdc3DmaLen; i += 3U) {
    sum_v += adc3_dma_[i];
  }
  const float adc_v_avg = static_cast<float>(sum_v) / 4.0f;

  const float i_sense = adc_i_avg * kAdcRefVoltage / kAdcMaxValue;
  const float v_sense = adc_v_avg * kAdcRefVoltage / kAdcMaxValue;
  v_s = sum_v_test / 4.0f * kAdcRefVoltage / kAdcMaxValue;
  latest_current_a_ = (i_sense - kCurrentOffset) * kCurrentFactor;
  latest_voltage_v_ = (v_sense - kVoltageOffset) * kVoltageFactor;

  // 用 |V*I| 估算功率并低通滤波，减少交流符号翻转带来的抖动。
  const float power_inst = std::fabs(latest_voltage_v_ * latest_current_a_);
  filtered_power_w_ += cfg_.power_lpf_alpha * (power_inst - filtered_power_w_);
}

void WirelessChargeController::RunPowerClosedLoop(FodGateState state) {
  if (!outputs_enabled_) {
    return;
  }

  float target_power = 0.0f;
  switch (state) {
    case FodGateState::kProbeLowPower:
      target_power = cfg_.probe_power_w;
      break;
    case FodGateState::kCharging:
      target_power = cfg_.charge_power_w;
      break;
    case FodGateState::kRetryGap:
      return;
  }

  const float phase_cmd = power_pid_.Update(target_power, filtered_power_w_);
  SetPhaseShift(phase_cmd);
}

}  // namespace wpc