#include "halfbridge_controller.hpp"
#include <cstdint>

#include "bsp_hrtim.hpp"
#include "receiver_ask.hpp"
#include "stm32g474xx.h"
#include "stm32g4xx_hal_gpio.h"
float setpiont = 24.5;
namespace App
{
#define CONTROL_SEND_HZ(HZ)    \
    {                          \
        static int16_t hz = 0; \
        hz++;                  \
        if (hz < HZ)           \
            return;            \
        hz = 0;                \
    }
    namespace
    {
        // constexpr uint32_t kTicks32kTo100ms = 3200U;

        // constexpr uint32_t kTicks32kTo50ms = 1600U;
        // constexpr uint32_t kTicks32kTo25ms = 800U;
        // ADC4 一次扫描窗口加保护时间，用来把触发点放在 PWM 导通/关断的稳定区域。
        constexpr uint32_t kAdc4ScanTicks = 7107U;
        constexpr uint32_t kAdcGuardTicks = 1000U;
        constexpr float kBatUvpThreshold = 12.0f; // UVP 触发阈值
        constexpr float kBatUvpRecovery = 14.0f; // UVP 恢复阈值（回差防止反复跳变）
        constexpr float kCapOvpThreshold = 23.5f;
        constexpr float kBatOvpThreshold = 80.0f;
        constexpr float kCurrentOcpThreshold = 20.0f;
        constexpr float kCurrentReverseThreshold = -1.0f;

    } // namespace

    void HalfBridgeController::init()
    {
        maxDuty_ = 0.92f;
        minDuty_ = 0.05f;
        duty_ = 0.2f;
        powerOn_ = 0;
        maxStep_ = 0.03f;
        // Vbat 恒压 PID：输出为前馈上的修正量 [-0.3, 0.3]，Vbat↓→输出↓→duty↓→减载→Vbat↑
        // Vbat 恒压 PID：输出为前馈上的修正量 [-0.3, 0.3]
        pid_init(&pidBuckV_, PID_DELTA, 0.01f, 0.0002f, 0.0f, -0.3f, maxDuty_, minDuty_);
        // 恒流 PID：低压阶段 1.5A 充电，输出占空比 [minDuty_, maxDuty_]
        pid_init(&pidBuckI_, PID_DELTA, 0.005f, 0.002f, 0.0f, minDuty_, maxDuty_, minDuty_);
        currentOverload_ = 0.0f;

        uvpBatTrigger_.init(25U, 2U); // 25ms 确认触发, 2ms 释放
        uvpCapTrigger_.init(25U, 2U);
        ocpTrigger_.init(20U, 10U); // 50ms 确认触发, 10ms 释放
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
            setDuty(dutyAdjust());
            Driver::enableOutputs(Driver::getOutputMask(Driver::HrtimTimer::TimerB));
            return 1U;
        }

        return 0U;
    }

    void HalfBridgeController::stop()
    {
        Driver::disableOutputs(Driver::getOutputMask(Driver::HrtimTimer::TimerB));
        pid_reset(&pidBuckV_);
        pid_reset(&pidBuckI_);
        currentOverload_ = 0.0f;
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
        ledStatus();
        if ((state.ocp != 0U) || (state.otpCap != 0U) || (state.ovpBat != 0U) || (state.ovpCap != 0U) ||
            (state.uvpBat != 0U))
        {
            stop();
        }
        else if (state.enable != 0U)
        {
            if (voltageCapFeed_ >= 15 - 2)
            {
                ReceiverAsk::setPowerRequirement(0);
            }
            else
            {
                ReceiverAsk::setPowerRequirement(1);
            }
            buckCal();

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
        state_.errorbit = (state_.uvpBat << 0) | (state_.uvpCap << 1) | (state_.otpCap << 2) | (state_.ocp << 3) |
            (state_.ovpCap << 4) | (state_.ovpBat << 5);
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
    void HalfBridgeController::buckCal()
    {
        // ============================================================
        // 两阶段充电控制
        //
        // 阶段 1 (Vcap < 6V):  恒流 1.5A — 电容低压时电流可控，不走 Vbat 环
        // 阶段 2 (Vcap ≥ 6V):  Vbat 恒压 — 线圈电压稳定，电容自然浮充
        //
        //   Vcap ↓ → 切换至恒流 → 1.5A 平稳升压
        //   Vcap → 6V → 切换至 Vbat 恒压
        //   Vcap → target → 自然停充
        // ============================================================
        constexpr float kCapCurrentThreshold = 8.0f;
        constexpr float kCapCurrentCharge = 2.8f;

        float targetDuty;

        if (voltageCapFeed_ < kCapCurrentThreshold)
        {
            // ==================== 阶段 1: 恒流 2A ====================
            pidBuckI_.ref = kCapCurrentCharge;
            pid_calculate(&pidBuckI_, currentFeed_);
            targetDuty = pidBuckI_.output;
            // 反流下界
            const float dutyAntiRev = dutyAdjust();
            if (targetDuty < dutyAntiRev)
            {
                targetDuty = dutyAntiRev;
                pidBuckI_.output = dutyAntiRev;
            }

            // Vbat PID 跟随当前 duty，切换时无跳变
            // ref=Vbat, fdb=target: error = Vbat - kBatTarget
            pidBuckV_.ref = voltageBatFeed_;
            pid_calculate(&pidBuckV_, setpiont);
            // 不输出，只让内部状态跟踪
            pidBuckV_.output = targetDuty - (voltageCapFeed_ / setpiont) - 0.01f;
        }
        else
        {
            // ==================== 阶段 2: Vbat 恒压 ====================
            // 前馈（目标 Vbat）：不随实际 Vbat 波动
            float duty_ff = (voltageCapFeed_ / setpiont) + 0.01f;
            if (duty_ff < minDuty_) duty_ff = minDuty_;
            if (duty_ff > maxDuty_) duty_ff = maxDuty_;

            // Vbat PID: ref=Vbat, fdb=target → error=Vbat - target
            // Vbat > target → output↑ → duty↑ → Vbat↓  ✓
            // Vbat < target → output↓ → duty↓ → Vbat↑  ✓
            pidBuckV_.ref = voltageBatFeed_;
            pid_calculate(&pidBuckV_, setpiont);
            targetDuty = duty_ff + pidBuckV_.output;
            // 电压浪涌限制
            constexpr float kMaxInductorDeltaV = 8.0f;
            if (voltageBatFeed_ > 0.5f)
            {
                float dutySurgeMax = (voltageCapFeed_ + kMaxInductorDeltaV) / voltageBatFeed_;
                if (dutySurgeMax < minDuty_)
                    dutySurgeMax = minDuty_;
                if (dutySurgeMax > maxDuty_)
                    dutySurgeMax = maxDuty_;
                if (targetDuty > dutySurgeMax)
                    targetDuty = dutySurgeMax;
            }
            // 反流下界
            const float dutyAntiRev = dutyAdjust();
            if (targetDuty < dutyAntiRev) targetDuty = dutyAntiRev;

            // PID 抗饱和
            if (targetDuty != (duty_ff + pidBuckV_.output))
                pidBuckV_.output = targetDuty - duty_ff;

            // 电流 PID 跟随当前 duty，切换时无跳变
            pidBuckI_.ref = kCapCurrentCharge;
            pid_calculate(&pidBuckI_, currentFeed_);
            pidBuckI_.output = targetDuty;
        }

        // 速率限制
        const float delta = targetDuty - duty_;
        if (delta > maxStep_)
            duty_ += maxStep_;
        else if (delta < -maxStep_)
            duty_ -= maxStep_;
        else
            duty_ = targetDuty;
    }
    uint16_t HalfBridgeController::GetErrorbit()
    {

        return state_.errorbit;
    }

    void HalfBridgeController::ledStatus()
    {
        CONTROL_SEND_HZ(8) // ~0.5kHz from 4kHz base

        const uint16_t err = state_.errorbit;
        constexpr uint16_t kUvpMask = (1U << 0) | (1U << 1); // uvpBat | uvpCap

        if (err == 0U && state_.enable != 0U)
        {
            // 正常 Buck：常亮
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
        }
        else if ((err & kUvpMask) != 0U)
        {
            // 欠压（电池欠压或电容欠压）：熄灭
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
        }
        else
        {
            // 其他故障（ocp/ovp/otp）：快速闪烁 ~2Hz
            static uint16_t blinkCnt = 0U;
            ++blinkCnt;
            if (blinkCnt < 125U)
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
            }
            else if (blinkCnt < 250U)
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
            }
            else
            {
                blinkCnt = 0U;
            }
        }
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
