#include "user_math.hpp"

#include <cmath>

namespace Driver
{
namespace
{

float clampAlpha(const float alpha)
{
    // alpha 必须落在 (0, 1) 内，越大越相信新采样值。
    if (alpha <= 0.0f)
    {
        return 0.1f;
    }

    if (alpha >= 1.0f)
    {
        return 0.9f;
    }

    return alpha;
}

} // namespace

void FirstOrderLpf::init(const float alpha, const float initValue)
{
    alpha_ = clampAlpha(alpha);
    lastOutput_ = initValue;
    initialized_ = true;
}

float FirstOrderLpf::update(const float input)
{
    if (!initialized_)
    {
        return input;
    }

    const float output = alpha_ * input + (1.0f - alpha_) * lastOutput_;
    lastOutput_ = output;
    return output;
}

void FirstOrderLpf::reset(const float value)
{
    lastOutput_ = value;
}

void FirstOrderHpf::init(const float alpha, const float initInput)
{
    alpha_ = clampAlpha(alpha);
    lastInput_ = initInput;
    lastOutput_ = 0.0f;
    initialized_ = true;
}

float FirstOrderHpf::update(const float input)
{
    if (!initialized_)
    {
        return input;
    }

    const float output = alpha_ * (lastOutput_ + input - lastInput_);
    lastInput_ = input;
    lastOutput_ = output;
    return output;
}

void FirstOrderHpf::reset(const float input)
{
    lastInput_ = input;
    lastOutput_ = 0.0f;
}

void RecursiveAverageFilter::init(uint16_t window, const float initValue)
{
    if (window == 0U)
    {
        window = 1U;
    }
    else if (window > kRecursiveAverageWindowMax)
    {
        window = kRecursiveAverageWindowMax;
    }

    window_ = window;
    reset(initValue);
}

float RecursiveAverageFilter::update(const float input)
{
    if (window_ == 0U)
    {
        return input;
    }

    const float removedValue = fifo_[index_];
    // 用“新值减旧值”更新总和，避免每次重新遍历整个窗口。
    fifo_[index_] = input;
    sum_ += input - removedValue;

    ++index_;
    if (index_ >= window_)
    {
        index_ = 0U;
    }

    if (count_ < window_)
    {
        ++count_;
    }

    return sum_ / static_cast<float>(count_);
}

void RecursiveAverageFilter::reset(const float initValue)
{
    sum_ = initValue * static_cast<float>(window_);
    index_ = 0U;
    count_ = window_;

    for (uint16_t i = 0U; i < kRecursiveAverageWindowMax; ++i)
    {
        fifo_[i] = (i < window_) ? initValue : 0.0f;
    }
}

void SlidingWindowU16::init(const uint8_t windowSize)
{
    uint8_t size = windowSize;
    if (size == 0U)
    {
        size = 1U;
    }
    else if (size > kSlidingWindowU16Max)
    {
        size = kSlidingWindowU16Max;
    }

    windowSize_ = size;
    reset();
}

float SlidingWindowU16::update(const uint16_t input)
{
    if (windowSize_ == 0U)
    {
        return static_cast<float>(input);
    }

    // 递推求和：减旧值、存新值、加新值
    sum_ -= buffer_[index_];
    buffer_[index_] = input;
    sum_ += input;

    ++index_;
    if (index_ >= windowSize_)
    {
        index_ = 0U;
    }

    if (count_ < windowSize_)
    {
        ++count_;
    }

    return static_cast<float>(sum_) / static_cast<float>(count_);
}

void SlidingWindowU16::reset()
{
    sum_ = 0U;
    index_ = 0U;
    count_ = 0U;
    for (uint8_t i = 0U; i < kSlidingWindowU16Max; ++i)
    {
        buffer_[i] = 0U;
    }
}

float SlidingWindowU16::getAverage() const
{
    if (count_ == 0U)
    {
        return 0.0f;
    }

    return static_cast<float>(sum_) / static_cast<float>(count_);
}

uint8_t SlidingWindowU16::getCount() const
{
    return count_;
}

float RmsAccumulator::update(const float value)
{
    // 保留旧实现的窗口节奏，便于后续交流量采样扩展。
    if (count_ < 1000U)
    {
        sum_ += value * value;
        ++count_;
    }

    if (count_ == 1000U)
    {
        average_ = std::sqrt(sum_ / 128.0f);
        count_ = 0U;
        sum_ = 0.0f;
    }

    return average_;
}

} // namespace Driver
