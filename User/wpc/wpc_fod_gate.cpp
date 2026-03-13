#include "wpc_fod_gate.hpp"

namespace wpc {

FodGate::FodGate(const FodGateConfig& cfg) : cfg_(cfg) {
  if (cfg_.no_ask_timeout_ms == 0U) {
    cfg_.no_ask_timeout_ms = 50U;
  }
  if (cfg_.probe_retry_ms == 0U) {
    cfg_.probe_retry_ms = 10U;
  }
  if (cfg_.probe_window_ms == 0U) {
    cfg_.probe_window_ms = 20U;
  }
}

void FodGate::Reset(uint32_t now_ms) {
  state_ = FodGateState::kProbeLowPower;
  last_state_change_ms_ = now_ms;
  last_ask_valid_ms_ = now_ms;
}

void FodGate::OnAskValid(uint32_t now_ms) {
  // 只要检测到有效ASK，就刷新通信看门狗时间戳。
  last_ask_valid_ms_ = now_ms;

  // 探测态一旦看到ASK，立刻切入充电态。
  if (state_ == FodGateState::kProbeLowPower) {
    SwitchState(FodGateState::kCharging, now_ms);
  }
}

void FodGate::Update(uint32_t now_ms) {
  switch (state_) {
    case FodGateState::kProbeLowPower:
      // 探测窗口到期仍无ASK：先退避，再进入下一轮探测。
      if ((now_ms - last_state_change_ms_) >= cfg_.probe_window_ms) {
        SwitchState(FodGateState::kRetryGap, now_ms);
      }
      break;

    case FodGateState::kCharging:
      // 充电态通信超时：关断并回到探测循环。
      if ((now_ms - last_ask_valid_ms_) >= cfg_.no_ask_timeout_ms) {
        SwitchState(FodGateState::kRetryGap, now_ms);
      }
      break;

    case FodGateState::kRetryGap:
      // 周期重试策略：不锁存，持续回到探测态。
      if ((now_ms - last_state_change_ms_) >= cfg_.probe_retry_ms) {
        SwitchState(FodGateState::kProbeLowPower, now_ms);
      }
      break;
  }
}

FodGateState FodGate::state() const {
  return state_;
}

void FodGate::SwitchState(FodGateState next, uint32_t now_ms) {
  if (state_ == next) {
    return;
  }
  state_ = next;
  last_state_change_ms_ = now_ms;
}

}  // namespace wpc
