#ifndef WPC_FOD_GATE_HPP
#define WPC_FOD_GATE_HPP

#include <cstdint>

namespace wpc {
//状态机
enum class FodGateState : uint8_t {
  kProbeLowPower = 0,
  kCharging,
  kRetryGap,
};

struct FodGateConfig {
  // 最近一次有效ASK之后，允许维持充电的超时时间。
  uint32_t no_ask_timeout_ms = 50;
  // 退避等待时长，到期后重新进入低功率探测。
  uint32_t probe_retry_ms = 10;
  // 探测态最大持续时长，超时且无ASK则进入退避。
  uint32_t probe_window_ms = 20;
};

class FodGate {
 public:
  explicit FodGate(const FodGateConfig& cfg = FodGateConfig{});
  void Reset(uint32_t now_ms);
  void OnAskValid(uint32_t now_ms);
  void Update(uint32_t now_ms);
  FodGateState state() const;

 private:
  void SwitchState(FodGateState next, uint32_t now_ms);

  FodGateConfig cfg_;
  FodGateState state_ = FodGateState::kProbeLowPower;
  uint32_t last_state_change_ms_ = 0;
  uint32_t last_ask_valid_ms_ = 0;
};

}  // namespace wpc

#endif  // WPC_FOD_GATE_HPP
