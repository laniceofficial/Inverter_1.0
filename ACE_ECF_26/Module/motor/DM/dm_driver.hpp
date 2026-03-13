#ifndef __DM_DRIVER_HPP__
#define __DM_DRIVER_HPP__

#include "motor_base_def.hpp"

namespace Motor_n
{
    namespace DmMotor_n {
        class DmDriver_c : public MotorBaseDef_n::MotorBase_c
        {
        public:
            // 控制指令
            typedef enum : unsigned char
            {
                DM_CMD_MOTOR_MODE = 0xfc,    // 使能,会响应指令
                DM_CMD_RESET_MODE = 0xfd,    // 停止
                DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
                DM_CMD_CLEAR_ERROR = 0xfb    // 清除电机过热错误
            } DMMotor_Status_euc;

            // 通用参数设置(需要根据调试工具确定)
            typedef struct
            {
                float kp_min;
                float kp_max;
                float kd_min;
                float kd_max;
                float v_min; // 速度范围
                float v_max;
                float p_min; // 位置范围
                float p_max;
                float t_min; // 扭矩范围
                float t_max;
            } DM_ModePrame_s;

            MotorBase_c& get_base() {return *this;} // 获取基类引用
            DmDriver_c(MotorBaseDef_n::Motor_Base_Config_t motor_config, DM_ModePrame_s prame) : MotorBase_c(motor_config) ,param_(prame)
            {
                switch(motor_config.motor_type)
                {
                    case MotorBaseDef_n::Motor_Type_euc::DM4310:
                        motor_data_.motor_fixed_param.ratio = DM4310_RATIO;
                        break;
                    case MotorBaseDef_n::Motor_Type_euc::DM4340:
                        motor_data_.motor_fixed_param.ratio = DM4340_RATIO;
                        break;
                    case MotorBaseDef_n::Motor_Type_euc::DM6220:
                        motor_data_.motor_fixed_param.ratio = DM6220_RATIO;
                        break;
                    case MotorBaseDef_n::Motor_Type_euc::DM8009:
                        motor_data_.motor_fixed_param.ratio = DM8009_RATIO;
                        break;
                    case MotorBaseDef_n::Motor_Type_euc::DM10010:
                        motor_data_.motor_fixed_param.ratio = DM10010_RATIO;
                        break;
                    case MotorBaseDef_n::Motor_Type_euc::DM2325:
                        motor_data_.motor_fixed_param.ratio = DM2325_RATIO;
                        break;
                    case MotorBaseDef_n::Motor_Type_euc::DM3519:
                        motor_data_.motor_fixed_param.ratio = DM3519_RATIO;
                        break;
                }
                // 实现分发机制
#ifdef STM32H723xx
                this->SetCallback([this](BSP_n::Fdcan_c *can_instance)
                { this->CallBack(can_instance); });
#else
                this->SetCallback([this](BSP_n::Can_c *can_instance)
                { this->CallBack(can_instance); });
#endif
            }
            // 重写的函数
            void Transmit(float outtime) override;
            void Transmit(const std::array<uint8_t, 8> &data, float outtime) override;
            void OnlyReceive() override;
            void Disable() override;
            void Enable() override;
            void DmMotorStateSet(DMMotor_Status_euc state);
            void DmTransmit(float outtime ,DMMotor_Status_euc state_set); // 带状态设置和降频处理的发送函数
            void SetMotorOutputFix(float output) override;
            void SetMITData(float pos, float vel, float kp, float kd, float t) override;
            // 数据解析函数
            void ParseCANData(const std::array<uint8_t, 8> data) override;

            // 特征的函数
            void ClearErrorFlag(); // 清除错误标志
            void SetZero();        // 设置零点
        private:
            DM_ModePrame_s param_;
            // 设置控制参数
            std::array<uint8_t, 8> setCtrl(uint8_t id)
            {
                return {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,id};
            }
        };
    }
}

#endif