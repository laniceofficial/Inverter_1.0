#include "wpc_ask_proto_decoder.hpp"

namespace wpc {

uint32_t AskProtoDecoder::UsToSamples(uint32_t sample_rate_hz, uint16_t us) {
  const uint64_t samples = (static_cast<uint64_t>(sample_rate_hz) * static_cast<uint64_t>(us) + 500000ULL) / 1000000ULL;
  return (samples == 0ULL) ? 1U : static_cast<uint32_t>(samples);
}

AskProtoDecoder::AskProtoDecoder(const AskProtoDecoderConfig& cfg) : cfg_(cfg) {
  if (cfg_.sample_rate_hz == 0U) {
    cfg_.sample_rate_hz = 25000U;
  }
  if (cfg_.interval_min_us == 0U) {
    cfg_.interval_min_us = 125U;
  }
  if (cfg_.interval_split_us <= cfg_.interval_min_us) {
    cfg_.interval_split_us = static_cast<uint16_t>(cfg_.interval_min_us + 250U);
  }
  if (cfg_.interval_max_us <= cfg_.interval_split_us) {
    cfg_.interval_max_us = static_cast<uint16_t>(cfg_.interval_split_us + 250U);
  }

  min_interval_samples_ = UsToSamples(cfg_.sample_rate_hz, cfg_.interval_min_us);
  split_interval_samples_ = UsToSamples(cfg_.sample_rate_hz, cfg_.interval_split_us);
  max_interval_samples_ = UsToSamples(cfg_.sample_rate_hz, cfg_.interval_max_us);

  if (split_interval_samples_ <= min_interval_samples_) {
    split_interval_samples_ = min_interval_samples_ + 1U;
  }
  if (max_interval_samples_ <= split_interval_samples_) {
    max_interval_samples_ = split_interval_samples_ + 1U;
  }

  Reset();
}

void AskProtoDecoder::Reset() {
  pointer_ = 0U;
  payload_bits_.fill(0U);
  raw10_bits_.fill(0U);
}

AskProtoDecodeResult AskProtoDecoder::OnTransition(uint8_t level, uint32_t delta_samples) {
  AskProtoDecodeResult result{};
  // 1. 有效性检查：采样间隔不能为 0
  if (delta_samples == 0U) {
    return result;
  }
  // 2. 脉冲宽度验证：必须在 [min_interval, max_interval] 范围内
  // 如果超出范围，重置指针并标记帧无效
  if (delta_samples < min_interval_samples_ || delta_samples > max_interval_samples_) {
    pointer_ = 0U;
    result.frame_invalid_pulse = true;
    return result;
  }

  if (delta_samples < split_interval_samples_) {
    PushBit(level, result);
  } else {
    PushBit(level, result);
    PushBit(level, result);
  }

  return result;
}

void AskProtoDecoder::PushBit(uint8_t bit, AskProtoDecodeResult& result) {
  const uint8_t level = (bit == 0U) ? 0U : 1U;

  if (pointer_ < kStartLen) {
    if (kStartSequence[pointer_] == level) {
      pointer_++;
    } else {
      pointer_ = 0U;
    }
    return;
  }

  payload_bits_[pointer_ - kStartLen] = level;
  pointer_++;

  if (pointer_ == kTotalLen) {
    pointer_ = 0U;
    if (DecodePayload()) {
      result.frame_valid_pulse = true;
    } else {
      result.frame_invalid_pulse = true;
    }
  }
}

bool AskProtoDecoder::DecodePayload() {
  // 曼彻斯特结构检查：按参考实现，bit[1] 与 bit[2] 必须不同。
  for (uint8_t i = 0U; i < 9U; i++) {
    if (payload_bits_[i * 2U + 1U] == payload_bits_[i * 2U + 2U]) {
      return false;
    }
  }

  // 两位合一位：相同=0，不同=1。
  for (uint8_t i = 0U; i < 10U; i++) {
    raw10_bits_[i] = (payload_bits_[i * 2U] == payload_bits_[i * 2U + 1U]) ? 0U : 1U;
  }

  const uint8_t required_power_selection = raw10_bits_[0U];

  uint8_t raw_power_feedback = 0U;
  for (uint8_t i = 0U; i < 8U; i++) {
    raw_power_feedback |= static_cast<uint8_t>(raw10_bits_[i + 1U] << i);
  }

  uint16_t parity = static_cast<uint16_t>(required_power_selection) |
                    static_cast<uint16_t>(raw_power_feedback << 1U);
  parity ^= static_cast<uint16_t>(parity >> 8U);
  parity ^= static_cast<uint16_t>(parity >> 4U);
  parity ^= static_cast<uint16_t>(parity >> 2U);
  parity ^= static_cast<uint16_t>(parity >> 1U);

  return static_cast<uint8_t>(parity & 0x01U) == raw10_bits_[9U];
}

}  // namespace wpc