#include "halfbridge_controller.hpp"

#include "bsp_hrtim.hpp"

namespace App
{
namespace
{

constexpr uint32_t kTicks24kTo50ms = 1200U;
constexpr uint32_t kTicks24kTo25ms = 600U;
// ADC4 一次扫描窗口加保护时间，用来把触发点放在 PWM 导通/关断的稳定区域。
constexpr uint32_t kAdc4ScanTicks = 7107U;
constexpr uint32_t kAdcGuardTicks = 1000U;
constexpr float kHalfBridgeVoltageKp = 0.01f;
constexpr float kHalfBridgeVoltageKi = 0.0003f;
constexpr float kHalfBridgeCurrentKp = 0.02f;
constexpr float kHalfBridgeCurrentKi = 0.0003f;
constexpr float kBatUvpThreshold = 20.3f;
constexpr float kCapOvpThreshold = 23.0f;
constexpr float kBatOvpThreshold = 30.0f;
constexpr float kCurrentOcpThreshold = 15.0f;
constexpr float kCurrentReverseThreshold = -0.3f;
constexpr float kDebugCurrentSetpoint = 2.0f;

} // namespace

void HalfBridgeController::init()
{
    maxDuty_ = 0.85f;
    minDuty_ = 0.05f;
    duty_ = 0.2f;
    powerOn_ = 0;
    maxStep_ = 0.02f;
    pid_init(&pidBuckV_, PID_DELTA, kHalfBridgeVoltageKp, kHalfBridgeVoltageKi, 0.0f, 0.5f, 3.0f, 0.0f);
    pid_init(&pidBuckI_, PID_DELTA, kHalfBridgeCurrentKp, kHalfBridgeCurrentKi, 0.0f, 0.1f, maxDuty_, minDuty_);

    uvpBatTrigger_.init(kTicks24kTo25ms);
    uvpCapTrigger_.init(kTicks24kTo25ms);
    ocpTrigger_.init(kTicks24kTo50ms);
    ovpBatTrigger_.init(kTicks24kTo25ms);
    ovpCapTrigger_.init(kTicks24kTo25ms);

    Driver::startTimers(Driver::getTimerId(Driver::HrtimTimer::TimerB));
    state_.enable = 1U;
}

uint8_t HalfBridgeController::start()
{
    if (powerOn_-- <= -2000)
    {
        powerOn_ = 1;
        setDuty(dutyAdjust());
        Driver::enableOutputs(Driver::getOutputMask(Driver::HrtimTimer::TimerB));
        return 1U;
    }

    return 0U;
}

void HalfBridgeController::stop()
{
    Driver::disableOutputs(Driver::getOutputMask(Driver::HrtimTimer::TimerB));
    pid_reset(&pidBuckI_);
    pid_reset(&pidBuckV_);
    powerOn_ = 0;
}

void HalfBridgeController::powerLoop()
{
    if (powerOn_ != 1)
    {
        start();
    }

    // 当前工程处于固定占空比调试逻辑，保持旧版本行为不改变。
    setDuty(0.2f);
    return;

    PowerStateBits& state = judgeState();
    if (state.charge != 0U)
    {
        buckCal(voltageBatFeed_, currentFeed_, voltageBatFeed_);
    }
    else if (state.discharge != 0U)
    {
        boostCal(voltageBatFeed_, currentFeed_, voltageBatFeed_);
    }
    else if ((state.ocp != 0U) || (state.otpCap != 0U) || (state.ovpBat != 0U) || (state.ovpCap != 0U) ||
             (state.uvpBat != 0U))
    {
        stop();
    }
    else if (state.enable != 0U)
    {
        if (powerOn_ != 1)
        {
            start();
        }
        else
        {
            pidBuckI_.ref = kDebugCurrentSetpoint;
            pid_calculate(&pidBuckI_, currentFeed_);

            const float feedForwardDuty = dutyAdjust();
            float targetDuty = pidBuckI_.output;
            if (targetDuty < feedForwardDuty)
            {
                // 电压比前馈是最低可行占空比，避免增量式 PID 从过低输出慢慢爬升。
                targetDuty = feedForwardDuty;
                pidBuckI_.output = feedForwardDuty;
            }

            const float delta = targetDuty - duty_;
            if (delta > maxStep_)
            {
                duty_ += maxStep_;
            }
            else if (delta < -maxStep_)
            {
                duty_ -= maxStep_;
            }
            else
            {
                duty_ = targetDuty;
            }

            setDuty(duty_);
        }
    }
    else
    {
        stop();
    }
}

void HalfBridgeController::setDuty(float duty)
{
    duty = Driver::clampDuty(duty, minDuty_, maxDuty_);
    duty_ = duty;

    const uint32_t period = Driver::getPeriod(Driver::HrtimTimer::TimerB);
    const uint32_t cmp3 = static_cast<uint32_t>(static_cast<float>(period) * duty_);
    Driver::setComplementaryDuty(Driver::HrtimTimer::TimerB, duty_, calcAdcTriggerCompare(cmp3));
}

float HalfBridgeController::dutyAdjust() const
{
    if (voltageBatFeed_ == 0.0f)
    {
        return minDuty_;
    }

    // Buck 前馈近似 D = Vcap / Vbat，再加一点裕量作为启动占空比。
    float duty = (voltageCapFeed_ / voltageBatFeed_) + 0.01f;
    return Driver::clampDuty(duty, minDuty_, maxDuty_);
}

void HalfBridgeController::setFeedback(const float current, const float capVoltage, const float batVoltage)
{
    currentFeed_ = current;
    voltageCapFeed_ = capVoltage;
    voltageBatFeed_ = batVoltage;
}

PowerStateBits& HalfBridgeController::judgeState()
{
    // 原始故障先按阈值判定，再进入延时触发器消抖。
    const uint8_t uvpBatRaw = (voltageBatFeed_ < kBatUvpThreshold) ? 1U : 0U;
    const uint8_t uvpCapRaw = 0U;
    const uint8_t ocpRaw =
        ((currentFeed_ > kCurrentOcpThreshold) || (currentFeed_ < kCurrentReverseThreshold)) ? 1U : 0U;
    const uint8_t ovpBatRaw = (voltageBatFeed_ > kBatOvpThreshold) ? 1U : 0U;
    const uint8_t ovpCapRaw =
        ((voltageCapFeed_ > kCapOvpThreshold) || (voltageBatFeed_ < voltageCapFeed_)) ? 1U : 0U;

    state_.uvpBat = updateFaultBit(uvpBatTrigger_, uvpBatRaw);
    state_.uvpCap = updateFaultBit(uvpCapTrigger_, uvpCapRaw);
    state_.ocp = updateFaultBit(ocpTrigger_, ocpRaw);
    state_.ovpBat = updateFaultBit(ovpBatTrigger_, ovpBatRaw);
    state_.ovpCap = updateFaultBit(ovpCapTrigger_, ovpCapRaw);

    state_.enable = static_cast<uint8_t>(
        !(state_.uvpBat || state_.uvpCap || state_.ocp || state_.ovpBat || state_.ovpCap));

    return state_;
}

uint8_t HalfBridgeController::updateFaultBit(Driver::DelayedTrigger& trigger, const uint8_t currentFault)
{
    return trigger.update(currentFault != 0U) ? 1U : 0U;
}

uint32_t HalfBridgeController::calcAdcTriggerCompare(const uint32_t cmp3) const
{
    const uint32_t period = Driver::getPeriod(Driver::HrtimTimer::TimerB);
    const uint32_t onTicks = cmp3;
    const uint32_t offTicks = period - cmp3;
    const uint32_t requiredWindow = kAdc4ScanTicks + (2U * kAdcGuardTicks);
    uint32_t cmp4;

    // 优先把 ADC 触发放在较长的稳定窗口中央；窗口不够时退化到可用区间。
    if ((offTicks >= requiredWindow) && ((offTicks >= onTicks) || (onTicks < requiredWindow)))
    {
        cmp4 = cmp3 + kAdcGuardTicks + ((offTicks - requiredWindow) / 2U);
    }
    else if (onTicks >= requiredWindow)
    {
        cmp4 = kAdcGuardTicks + ((onTicks - requiredWindow) / 2U);
    }
    else if (offTicks >= kAdc4ScanTicks)
    {
        cmp4 = cmp3 + ((offTicks - kAdc4ScanTicks) / 2U);
    }
    else if (onTicks >= kAdc4ScanTicks)
    {
        cmp4 = (onTicks - kAdc4ScanTicks) / 2U;
    }
    else
    {
        cmp4 = cmp3;
    }

    return (cmp4 >= period) ? (period - 1U) : cmp4;
}

void HalfBridgeController::buckCal(const float feedbackVoltage, const float feedbackCurrent, const float targetVoltage)
{
    pidBuckV_.ref = targetVoltage;
    pid_calculate(&pidBuckV_, feedbackVoltage);
    pidBuckI_.ref = pidBuckV_.output;
    pid_calculate(&pidBuckI_, feedbackCurrent);
    setDuty(pidBuckV_.output);
}

void HalfBridgeController::boostCal(const float feedbackVoltage, const float feedbackCurrent, const float targetVoltage)
{
    pidBoostV_.ref = targetVoltage;
    pid_calculate(&pidBoostV_, feedbackVoltage);
    pidBoostI_.ref = pidBoostV_.output;
    pid_calculate(&pidBoostI_, feedbackCurrent);
    setDuty(pidBoostI_.output);
}

} // namespace App
