#ifndef __DJI_DRIVER_HPP
#define __DJI_DRIVER_HPP
#include "motor_base_def.hpp"

#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"
#include "string.h"

#ifdef __cplusplus
}

#include "motor_base_def.hpp"
#include "safe_task.hpp"

namespace Motor_n {
    namespace DjiMotor_n {
        class DjiDriver_c:public MotorBaseDef_n::MotorBase_c
        {
        public:
            MotorBase_c& get_base() {return *this;} // 获取基类引用
            float set_length_=0; // 设定的长度值
            float set_speed_=0;  // 设定的速度值
            float set_current_=0;// 设定的电流值
            float set_torque_ =0;// 开环设定的力矩值
            int16_t give_current_=0;// 发送的电流值

            // 分组发送设置
            static uint8_t dji_motor_idx; // 全局电机索引,一共注册了多少个电机
            static DjiDriver_c *dji_motor_instance_p[DJI_MOTOR_CNT];
            uint8_t sender_group;
            uint8_t message_num;

            DjiDriver_c(MotorBaseDef_n::Motor_Base_Config_t motor_config);// 构造函数
            void MotorSenderGrouping();// 电机分组发送设置
            void DjiMotorSetRef(float ref);// 设置参考值
            void DjiMotorSetLoop(MotorBaseDef_n::Closeloop_Type_euc loop);// 增加闭环类型
            void DjiMotorResetLoop(MotorBaseDef_n::Closeloop_Type_euc loop);// 删减闭环类型
            void DjiMotorSelfLock();// 电机自锁
            void DjiMotorClearSelfLock();// 清除电机自锁
            // 重写的函数
            void OnlyReceive() override;
            void Disable() override;
            void Enable() override;
            bool Is_MotorStuck(MotorBase_c *motor_ptr , float bulk_I , float bulk_speed , uint16_t bulk_cnt) override;// 堵转检测
            void ParseCANData(const std::array<uint8_t, 8> data) override;// 回调
            void SetMotorOutputFix(float output) override;// 设置电机固定输出
        };
        // 发送/控制函数
        void DjiMotorControl();// 电机控制函数,在control任务中调用
        void set_GiveCurrent();// 设置发送电流值
        void MotorTransmit();// 发送电机控制报文
    }
}

#endif

#endif
