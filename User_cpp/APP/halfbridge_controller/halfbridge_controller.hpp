#pragma once

#include "delaytrigger.hpp"
#include "pid.hpp"

#include <cstdint>

namespace App
{

    // 半桥功率状态位，故障位经过 DelayedTrigger 消抖后再更新。
    struct PowerStateBits
    {
        uint16_t errorbit = 0U; // 各故障位拼接，bit0:uvpBat bit1:uvpCap bit2:otpCap bit3:ocp bit4:ovpCap bit5:ovpBat
        uint8_t enable = 0U; // 功率级允许工作
        uint8_t uvpBat = 0U; // 电池欠压
        uint8_t uvpCap = 0U; // 超级电容欠压
        uint8_t otpCap = 0U; // 超级电容过温
        uint8_t ocp = 0U; // 过流或反向电流
        uint8_t ovpCap = 0U; // 超级电容过压
        uint8_t ovpBat = 0U; // 电池过压
    };

    // 超级电容无线充电侧半桥控制器，负责故障判定、占空比限制和 Buck/Boost PID。
    class HalfBridgeController
    {
    public:
        void init();
        uint8_t start();
        void stop();
        void powerLoop();
        void setDuty(float duty);
        float dutyAdjust() const;
        void setFeedback(float current, float capVoltage, float batVoltage);
        PowerStateBits &judgeState();
        uint16_t GetErrorbit();
        void ledStatus();

    private:
        uint8_t updateFaultBit(Driver::DelayedTrigger &trigger, uint8_t currentFault);
        uint32_t calcAdcTriggerCompare(uint32_t cmp3) const;
        // void buckCal(float feedbackVoltage, float feedbackCurrent, float targetVoltage);
        // void boostCal(float feedbackVoltage, float feedbackCurrent, float targetVoltage);
        void buckCal();
        float currentOverload_ = 0.0f; // 电流过载积分 (软钳位用)
        float currentFeed_ = 0.0f; // 半桥电流反馈
        float voltageCapFeed_ = 0.0f; // 超级电容电压反馈
        float voltageBatFeed_ = 0.0f; // 电池/母线电压反馈
        PowerStateBits state_ = {};
        float duty_ = 0.2f; // 当前输出占空比
        float maxDuty_ = 0.85f; // 占空比上限
        float minDuty_ = 0.1f; // 占空比下限
        float maxStep_ = 0.02f; // 单周期最大占空比变化量
        int16_t powerOn_ = 0; // 上电延时计数，等于 1 表示已经开输出
        Driver::DelayedTrigger uvpBatTrigger_;
        Driver::DelayedTrigger uvpCapTrigger_;
        Driver::DelayedTrigger ocpTrigger_;
        Driver::DelayedTrigger ovpBatTrigger_;
        Driver::DelayedTrigger ovpCapTrigger_;
        PID pidBuckV_ = {};
        PID pidBuckI_ = {}; // 低压阶段恒流充电用
    };

} // namespace App
