#ifndef INCREMENTAL_PID_HPP
#define INCREMENTAL_PID_HPP

namespace control {

// 增量式PID：
// output[k] = output[k-1] + Δu[k]
// Δu[k] = Kp*(e[k]-e[k-1]) + Ki*e[k] + Kd*(e[k]-2e[k-1]+e[k-2])
class IncrementalPid {
 public:
  void Configure(float kp, float ki, float kd, float out_min, float out_max);
  void Reset(float initial_output);
  float Update(float target, float feedback);
  float output() const;

 private:
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;
  float out_min_ = 0.0f;
  float out_max_ = 1.0f;
  float e0_ = 0.0f;
  float e1_ = 0.0f;
  float e2_ = 0.0f;
  float output_ = 0.0f;
};

}  // namespace control

#endif  // INCREMENTAL_PID_HPP
