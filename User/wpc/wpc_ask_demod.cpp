#include "wpc_ask_demod.hpp"

namespace wpc {

AskDemodulator::AskDemodulator(const AskDemodConfig& cfg) : cfg_(cfg) {
  if (cfg_.sample_rate_hz == 0U) {
    cfg_.sample_rate_hz = 25000U;
  }
  if (cfg_.envelope_attack_alpha <= 0.0f || cfg_.envelope_attack_alpha >= 1.0f) {
    cfg_.envelope_attack_alpha = 0.8f;
  }
  if (cfg_.max_release_alpha <= 0.0f || cfg_.max_release_alpha >= 1.0f) {
    cfg_.max_release_alpha = 0.003f;
  }
  if (cfg_.min_release_alpha <= 0.0f || cfg_.min_release_alpha >= 1.0f) {
    cfg_.min_release_alpha = 0.005f;
  }
  if (cfg_.threshold_middle_weight < 0.0f || cfg_.threshold_dynamic_weight < 0.0f ||
      (cfg_.threshold_middle_weight + cfg_.threshold_dynamic_weight) <= 0.0f) {
    cfg_.threshold_middle_weight = 0.3f;
    cfg_.threshold_dynamic_weight = 0.7f;
  }
  if (cfg_.upper_trigger_samples == 0U) {
    cfg_.upper_trigger_samples = 1U;
  }
  if (cfg_.lower_trigger_samples == 0U) {
    cfg_.lower_trigger_samples = 1U;
  }
  Reset();
}

void AskDemodulator::Reset() {
  sample_index_ = 0U;
  last_transition_sample_ = 0U;
  has_transition_sample_ = false;
  current_level_ = 0U;
  upper_count_ = 0U;
  lower_count_ = 0U;
  dynamic_max_ = cfg_.middle_threshold;
  dynamic_min_ = cfg_.middle_threshold;
  upper_threshold_ = cfg_.middle_threshold;
  lower_threshold_ = cfg_.middle_threshold;
  last_valid_ms_ = 0U;
}

uint32_t AskDemodulator::LastValidMs() const {
  return last_valid_ms_;
}

AskSampleResult AskDemodulator::PushSample(uint16_t sample, uint32_t now_ms) {
  AskSampleResult result{};
  result.level = current_level_;
  sample_index_++;

  const float sample_f = static_cast<float>(sample);
  UpdateDynamicThresholds(sample_f);

  if (HitUpper(sample_f)) { //上升沿
    lower_count_ = 0U;
    if (current_level_ == 0U) {
      result.edge = AskEdge::kUp;
      result.transition_bit = 1U;
      current_level_ = 1U;
    }
  }

  if (HitLower(sample_f)) { //下降沿
    upper_count_ = 0U;
    if (current_level_ == 1U) {
      result.edge = AskEdge::kDown;
      result.transition_bit = 1U;
      current_level_ = 0U;
    }
  }

  if (result.transition_bit != 0U) {
    if (has_transition_sample_) {
      result.delta_samples = sample_index_ - last_transition_sample_;
    }
    last_transition_sample_ = sample_index_;
    has_transition_sample_ = true;
    last_valid_ms_ = now_ms;
  }

  result.level = current_level_;
  // 旧窗口有效判定保留字段但不再参与门控。
  result.window_valid = false;
  return result;
}

void AskDemodulator::UpdateDynamicThresholds(float sample_f) {
  const float attack = cfg_.envelope_attack_alpha;

  if (sample_f > dynamic_max_) {
    dynamic_max_ = dynamic_max_ * (1.0f - attack) + sample_f * attack;
  } else {
    const float target = cfg_.middle_threshold + cfg_.envelope_bias;
    dynamic_max_ = target * cfg_.max_release_alpha + dynamic_max_ * (1.0f - cfg_.max_release_alpha);
  }

  if (sample_f < dynamic_min_) {
    dynamic_min_ = dynamic_min_ * (1.0f - attack) + sample_f * attack;
  } else {
    const float target = cfg_.middle_threshold - cfg_.envelope_bias;
    dynamic_min_ = target * cfg_.min_release_alpha + dynamic_min_ * (1.0f - cfg_.min_release_alpha);
  }

  upper_threshold_ = cfg_.middle_threshold * cfg_.threshold_middle_weight +
                     dynamic_max_ * cfg_.threshold_dynamic_weight;
  lower_threshold_ = cfg_.middle_threshold * cfg_.threshold_middle_weight +
                     dynamic_min_ * cfg_.threshold_dynamic_weight;

  if (upper_threshold_ <= lower_threshold_) {
    upper_threshold_ = cfg_.middle_threshold + 1.0f;
    lower_threshold_ = cfg_.middle_threshold - 1.0f;
  }
}

bool AskDemodulator::HitUpper(float sample_f) {
  if (sample_f > upper_threshold_) {
    if (upper_count_ < cfg_.upper_trigger_samples) {
      upper_count_++;
    }
  } else {
    upper_count_ = 0U;
  }
  return upper_count_ >= cfg_.upper_trigger_samples;
}

bool AskDemodulator::HitLower(float sample_f) {
  if (sample_f < lower_threshold_) {
    if (lower_count_ < cfg_.lower_trigger_samples) {
      lower_count_++;
    }
  } else {
    lower_count_ = 0U;
  }
  return lower_count_ >= cfg_.lower_trigger_samples;
}

}  // namespace wpc