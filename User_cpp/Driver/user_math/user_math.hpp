#pragma once

#include <cstdint>

namespace Driver
{

constexpr uint16_t kRecursiveAverageWindowMax = 50U;
constexpr uint8_t kSlidingWindowU16Max = 128U;

// 一阶低通滤波器，适合平滑电压/电流等慢变量。
class FirstOrderLpf
{
public:
    void init(float alpha, float initValue);
    float update(float input);
    void reset(float value);

private:
    float alpha_ = 0.1f;      // y[n] = a*x[n] + (1-a)*y[n-1]
    float lastOutput_ = 0.0f; // 上一次滤波输出
    bool initialized_ = false;
};

// 一阶高通滤波器，适合提取交流分量或变化量。
class FirstOrderHpf
{
public:
    void init(float alpha, float initInput);
    float update(float input);
    void reset(float input);

private:
    float alpha_ = 0.1f;      // y[n] = a*(y[n-1] + x[n] - x[n-1])
    float lastInput_ = 0.0f;  // 上一次输入
    float lastOutput_ = 0.0f; // 上一次滤波输出
    bool initialized_ = false;
};

// 递推平均滤波器，使用固定数组避免动态内存。
class RecursiveAverageFilter
{
public:
    void init(uint16_t window, float initValue);
    float update(float input);
    void reset(float initValue);

private:
    float fifo_[kRecursiveAverageWindowMax] = {};
    float sum_ = 0.0f;
    uint16_t index_ = 0U;
    uint16_t count_ = 0U;
    uint16_t window_ = 0U;
};

// uint16_t 滑动窗口平均滤波器，适用于 ADC 原始码值的降噪。
// 窗口大小在 init 时指定（≤128），内部维护环形缓冲 + 递推求和。
class SlidingWindowU16
{
public:
    void init(uint8_t windowSize);
    float update(uint16_t input);
    void reset();
    float getAverage() const;
    uint8_t getCount() const;

private:
    uint16_t buffer_[kSlidingWindowU16Max] = {};
    uint32_t sum_ = 0U;
    uint8_t index_ = 0U;
    uint8_t count_ = 0U;
    uint8_t windowSize_ = 0U;
};

// 固定窗口 RMS 累加器，目前保留旧逻辑的 1000 点节奏。
class RmsAccumulator
{
public:
    float update(float value);

private:
    float average_ = 0.0f;
    float sum_ = 0.0f;
    uint16_t count_ = 0U;
};

} // namespace Driver
