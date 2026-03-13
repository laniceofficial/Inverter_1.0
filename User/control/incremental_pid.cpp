#include "incremental_pid.hpp"

#include <algorithm>

namespace control {

void IncrementalPid::Configure(float kp, float ki, float kd, float out_min, float out_max) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  out_min_ = out_min;
  out_max_ = out_max;
  if (out_min_ > out_max_) {
    std::swap(out_min_, out_max_);
  }
}

void IncrementalPid::Reset(float initial_output) {
  e0_ = 0.0f;
  e1_ = 0.0f;
  e2_ = 0.0f;
  output_ = std::clamp(initial_output, out_min_, out_max_);
}

float IncrementalPid::Update(float target, float feedback) {
  e2_ = e1_;
  e1_ = e0_;
  e0_ = target - feedback;

  const float delta_u =
      kp_ * (e0_ - e1_) + ki_ * e0_ + kd_ * (e0_ - 2.0f * e1_ + e2_);
  output_ = std::clamp(output_ + delta_u, out_min_, out_max_);
  return output_;
}

float IncrementalPid::output() const {
  return output_;
}

}  // namespace control
