#include "halfbridge_controller.hpp"

#include "bsp_hrtim.hpp"
#include "stm32g474xx.h"
#include "stm32g4xx_hal_gpio.h"
float setpiont = 10;
namespace App
{
    namespace
    {
        constexpr uint32_t kTicks32kTo100ms = 3200U;

        constexpr uint32_t kTicks32kTo50ms = 1600U;
        constexpr uint32_t kTicks32kTo25ms = 800U;
        // ADC4 一次扫描窗口加保护时间，用来把触发点放在 PWM 导通/关断的稳定区域。
        constexpr uint32_t kAdc4ScanTicks = 7107U;
        constexpr uint32_t kAdcGuardTicks = 1000U;
        constexpr float kHalfBridgeVoltageKp = 0.006f;
        constexpr float kHalfBridgeVoltageKi = 0.0003f;
        constexpr float kHalfBridgeCurrentKp = 0.005f;
        constexpr float kHalfBridgeCurrentKi = 0.002f;
        constexpr float kBatUvpThreshold = 15.0f;
        constexpr float kCapUvpThreshold = 0.5f;
        constexpr float kCapOvpThreshold = 23.0f;
        constexpr float kBatOvpThreshold = 80.0f;
        constexpr float kCurrentOcpThreshold = 20.0f;
        constexpr float kCurrentReverseThreshold = -1.0f;

    } // namespace

    void HalfBridgeController::init()
    {
        maxDuty_ = 0.9f;
        minDuty_ = 0.05f;
        duty_ = 0.1f;
        powerOn_ = 0;
        maxStep_ = 0.02f;
        pid_init(&pidBuckV_, PID_DELTA, kHalfBridgeVoltageKp, kHalfBridgeVoltageKi, 0.0f, 0.5f, 3.5f, 0.0f);
        pid_setStepIn(&pidBuckV_, 0.04f); // 每次最多改变 ref 0.05V，平滑阶跃
        pid_init(&pidBuckI_, PID_DELTA, kHalfBridgeCurrentKp, kHalfBridgeCurrentKi, 0.0f, 0.1f, maxDuty_, 0.0f);
        pid_setStepIn(&pidBuckI_, 0.005f); // 电流 ref 每周期最多变 5mA，软启动防止浪涌

        uvpBatTrigger_.init(25U, 2U); // 25ms 确认触发, 2ms 释放
        uvpCapTrigger_.init(25U, 2U);
        ocpTrigger_.init(50U, 10U); // 50ms 确认触发, 10ms 释放
        ovpBatTrigger_.init(25U, 2U);
        ovpCapTrigger_.init(25U, 2U);

        Driver::startTimers(Driver::getTimerId(Driver::HrtimTimer::TimerB));
        state_.enable = 1U;
    }

    uint8_t HalfBridgeController::start()
    {
        if (powerOn_-- <= -2000)
        {
            powerOn_ = 1;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
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
        duty_ = 0.2f;
    }

    void HalfBridgeController::powerLoop() // 会有震荡可能是pid计算或者参数问题
    {
        // 软启动期间跳过故障检测，防止 ADC 初始噪声触发误保护，匹配参考代码 HB_PowerLoop 的 early return。
        if (powerOn_ != 1)
        {
            start();
            // return;
        }

        PowerStateBits &state = judgeState();
        if ((state.ocp != 0U) || (state.otpCap != 0U) || (state.ovpBat != 0U) || (state.ovpCap != 0U) ||
            (state.uvpBat != 0U))
        {
            stop();
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
        }
        else if (state.enable != 0U)
        {
            // 保存电压环输出用于抗饱和，电流环受限时回退电压环比刻度的增量。
            const float buckVOutputPre = pidBuckV_.output;

            // 电压外环：Vcap → Iref（dVcap/dD > 0，直接输出）
            pidBuckV_.ref = setpiont;
            pid_calculate(&pidBuckV_, voltageCapFeed_);

            // 电流内环：Iref → 占空比（电容充电 dI/dD>0，直接映射）
            // pidBuckI_.ref = 3 - pidBuckV_.output;
            pidBuckI_.ref = pidBuckV_.output;
            pid_calculate(&pidBuckI_, currentFeed_);
            float targetDuty = pidBuckI_.output;

            // bool currentLoopClamped = false;

            // 前馈下限钳位：始终以 Vcap/Vbat + margin 作为占空比下界，
            // 防止 PID 输出低于该值时 Buck 输出电压低于电容电压导致反向电流。
            float dutyLowerBound = dutyAdjust();
            if (targetDuty < dutyLowerBound)
            {
                targetDuty = dutyLowerBound;
                pidBuckI_.output = dutyLowerBound;
                // currentLoopClamped = true;
            }

            // 电压跟踪门：限制目标占空比上限为 (Vcap + ΔVmax) / Vbat，
            // 确保电感两端压差不超过 ΔVmax，避免高占空比+低电容电压导致电流过大。
            // constexpr float kMaxInductorDeltaV = 5.0f;
            // if (voltageBatFeed_ > 0.5f)
            // {
            //     float dutySafeMax = (voltageCapFeed_ + kMaxInductorDeltaV) / voltageBatFeed_;
            //     if (dutySafeMax < minDuty_) dutySafeMax = minDuty_;
            //     if (dutySafeMax > maxDuty_) dutySafeMax = maxDuty_;
            //     if (targetDuty > dutySafeMax)
            //     {
            //         targetDuty = dutySafeMax;
            //         pidBuckI_.output = dutySafeMax;
            //         currentLoopClamped = true;
            //     }
            // }

            // 输入电压跌落保护：Vbat 偏低时进一步收紧占空比上限，
            // 切断 Vbat↓ → duty↑ → 电流↑ → Vbat↓↓ 的正反馈。
            // constexpr float kBatBrownoutThreshold = 16.0f;
            // if (voltageBatFeed_ < kBatBrownoutThreshold && voltageBatFeed_ > 0.5f)
            // {
            //     float dutyBrownoutMax = (voltageCapFeed_ + 1.0f) / voltageBatFeed_;
            //     if (dutyBrownoutMax < minDuty_) dutyBrownoutMax = minDuty_;
            //     if (dutyBrownoutMax > maxDuty_) dutyBrownoutMax = maxDuty_;
            //     if (targetDuty > dutyBrownoutMax)
            //     {
            //         targetDuty = dutyBrownoutMax;
            //         pidBuckI_.output = dutyBrownoutMax;
            //         currentLoopClamped = true;
            //     }
            // }
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
        const uint32_t cmp3 = static_cast<uint32_t>(static_cast<float>(period) * duty);
        Driver::setComplementaryDuty(Driver::HrtimTimer::TimerB, duty, calcAdcTriggerCompare(cmp3));
    }

    float HalfBridgeController::dutyAdjust() const
    {
        if (voltageBatFeed_ == 0.0f)
        {
            return minDuty_;
        }

        float duty = (voltageCapFeed_ / voltageBatFeed_) + 0.01f;

        return Driver::clampDuty(duty, minDuty_, maxDuty_);
    }

    void HalfBridgeController::setFeedback(const float current, const float capVoltage, const float batVoltage)
    {
        currentFeed_ = current;
        voltageCapFeed_ = capVoltage;
        voltageBatFeed_ = batVoltage;
    }

    PowerStateBits &HalfBridgeController::judgeState()
    {
        // 原始故障先按阈值判定，再进入延时触发器消抖。
        const uint8_t uvpBatRaw = (voltageBatFeed_ < kBatUvpThreshold) ? 1U : 0U;
        const uint8_t uvpCapRaw = 0; //(voltageCapFeed_ < kCapUvpThreshold) ? 1U : 0U;
        const uint8_t ocpRaw =
            ((currentFeed_ > kCurrentOcpThreshold) || (currentFeed_ < kCurrentReverseThreshold)) ? 1U : 0U;
        const uint8_t ovpBatRaw = (voltageBatFeed_ > kBatOvpThreshold) ? 1U : 0U;
        const uint8_t ovpCapRaw =
            ((voltageCapFeed_ > kCapOvpThreshold) || (voltageBatFeed_ + 1 < voltageCapFeed_)) ? 1U : 0U;

        state_.uvpBat = updateFaultBit(uvpBatTrigger_, uvpBatRaw);
        state_.uvpCap = updateFaultBit(uvpCapTrigger_, uvpCapRaw);
        state_.ocp = updateFaultBit(ocpTrigger_, ocpRaw);
        state_.ovpBat = updateFaultBit(ovpBatTrigger_, ovpBatRaw);
        state_.ovpCap = updateFaultBit(ovpCapTrigger_, ovpCapRaw);
        state_.errorbit = (state_.uvpBat << 0) | (state_.uvpCap << 1) | (state_.otpCap << 2) |
                          (state_.ocp << 3) | (state_.ovpCap << 4) | (state_.ovpBat << 5);
        state_.enable =
            static_cast<uint8_t>(!(state_.uvpBat || state_.uvpCap || state_.ocp || state_.ovpBat || state_.ovpCap));

        return state_;
    }

    uint8_t HalfBridgeController::updateFaultBit(Driver::DelayedTrigger &trigger, const uint8_t currentFault)
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

    // void HalfBridgeController::buckCal(const float feedbackVoltage, const float feedbackCurrent, const float
    // targetVoltage)
    // {
    //     pidBuckV_.ref = targetVoltage;
    //     pid_calculate(&pidBuckV_, feedbackVoltage);
    //     pidBuckI_.ref = pidBuckV_.output;
    //     pid_calculate(&pidBuckI_, feedbackCurrent);
    //     float targetDuty = pidBuckI_.output;
    //     setDuty(targetDuty);
    // }

    // void HalfBridgeController::boostCal(const float feedbackVoltage, const float feedbackCurrent, const float
    // targetVoltage)
    // {
    //     pidBoostV_.ref = targetVoltage;
    //     pid_calculate(&pidBoostV_, feedbackVoltage);
    //     pidBoostI_.ref = pidBoostV_.output;
    //     pid_calculate(&pidBoostI_, feedbackCurrent);
    //     setDuty(pidBoostI_.output);
    // }

} // namespace App
