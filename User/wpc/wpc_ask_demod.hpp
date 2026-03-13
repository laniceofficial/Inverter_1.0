#ifndef WPC_ASK_DEMOD_HPP
#define WPC_ASK_DEMOD_HPP

#include <cstdint>

namespace wpc {

enum class AskEdge : uint8_t {
  kNone = 0,
  kUp,
  kDown,
};

struct AskDemodConfig {
  // ASK 输入采样率（Hz），用于后级按样本间隔解码。
  uint32_t sample_rate_hz = 25000U;
  // 中值阈值（通常为 ADC 中点）。
  float middle_threshold = 2048.0f;
  // 动态包络回落目标相对中值的偏置。
  float envelope_bias = 200.0f;
  // 包络快速跟随系数（新极值时快速贴合）。
  float envelope_attack_alpha = 0.8f;
  // dynamicMax 回落时的一阶系数。
  float max_release_alpha = 0.003f;
  // dynamicMin 回落时的一阶系数。
  float min_release_alpha = 0.005f;
  // 动态阈值 = middle * threshold_middle_weight + dynamic * threshold_dynamic_weight。
  float threshold_middle_weight = 0.3f;
  float threshold_dynamic_weight = 0.7f;
  // 延时触发长度：连续命中 N 个样本才算翻转。
  uint8_t upper_trigger_samples = 2U;
  uint8_t lower_trigger_samples = 2U;
};

struct AskSampleResult {
  AskEdge edge = AskEdge::kNone;
  // 兼容旧逻辑的诊断位：已降级，不用于门控。
  bool window_valid = false;
  // 每样本二值输出：翻转=1，不变=0。
  uint8_t transition_bit = 0U;
  // 当前判决电平（0/1）。
  uint8_t level = 0U;
  // 本次样本处理是否刚解出一个有效协议包（由上层填充）。
  bool frame_valid_pulse = false;
  // 与上一次翻转的样本间隔，非翻转样本为 0。
  uint32_t delta_samples = 0U;
};

class AskDemodulator {
 public:
  explicit AskDemodulator(const AskDemodConfig& cfg = AskDemodConfig{});
  void Reset();
  // 输入一个 ADC 样本，输出翻转位和当前电平。
  AskSampleResult PushSample(uint16_t sample, uint32_t now_ms);
  uint32_t LastValidMs() const;

 private:
  void UpdateDynamicThresholds(float sample_f);
  bool HitUpper(float sample_f);
  bool HitLower(float sample_f);

  AskDemodConfig cfg_{};
  uint32_t sample_index_ = 0U;
  uint32_t last_transition_sample_ = 0U;
  bool has_transition_sample_ = false;
  uint8_t current_level_ = 0U;
  uint8_t upper_count_ = 0U;
  uint8_t lower_count_ = 0U;
  float dynamic_max_ = 2048.0f;
  float dynamic_min_ = 2048.0f;
  float upper_threshold_ = 2048.0f;
  float lower_threshold_ = 2048.0f;
  uint32_t last_valid_ms_ = 0U;
};

}  // namespace wpc

#endif  // WPC_ASK_DEMOD_HPP