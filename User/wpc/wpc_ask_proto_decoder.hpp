#ifndef WPC_ASK_PROTO_DECODER_HPP
#define WPC_ASK_PROTO_DECODER_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace wpc {

struct AskProtoDecoderConfig {
  uint32_t sample_rate_hz = 25000U;
  uint16_t interval_min_us = 125U;
  uint16_t interval_split_us = 375U;
  uint16_t interval_max_us = 625U;
};

struct AskProtoDecodeResult {
  bool frame_valid_pulse = false;
  bool frame_invalid_pulse = false;
};

class AskProtoDecoder {
 public:
  explicit AskProtoDecoder(const AskProtoDecoderConfig& cfg = AskProtoDecoderConfig{});
  void Reset();
  // 在每次电平翻转时调用，level 为翻转后的电平。
  AskProtoDecodeResult OnTransition(uint8_t level, uint32_t delta_samples);

 private:
  static constexpr size_t kStartLen = 20U;
  static constexpr size_t kPayloadLen = 20U;
  static constexpr size_t kTotalLen = kStartLen + kPayloadLen;

  static constexpr std::array<uint8_t, kStartLen> kStartSequence = {
      0U, 0U, 1U, 0U, 1U, 0U, 1U, 0U, 1U, 0U,
      1U, 0U, 1U, 0U, 1U, 0U, 1U, 0U, 1U, 1U};

  static uint32_t UsToSamples(uint32_t sample_rate_hz, uint16_t us);

  void PushBit(uint8_t bit, AskProtoDecodeResult& result);
  bool DecodePayload();

  AskProtoDecoderConfig cfg_{};
  uint32_t min_interval_samples_ = 1U;
  uint32_t split_interval_samples_ = 1U;
  uint32_t max_interval_samples_ = 1U;

  uint8_t pointer_ = 0U;  // 0..kTotalLen
  std::array<uint8_t, kPayloadLen> payload_bits_{};
  std::array<uint8_t, 10U> raw10_bits_{};
};

}  // namespace wpc

#endif  // WPC_ASK_PROTO_DECODER_HPP