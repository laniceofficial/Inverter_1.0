/*************************** Dongguan-University of Technology
 *-ACE**************************
 * @file   servo.cpp
 * @author  KazuHa12441
 * @version v1.0 2024/10/24
 *          v1.0.1 2025/11/24 添加333hz舵机，在初始化时须增加参数freq_
 *          v1.0.2 2026/1/20 修复一些潜在bug和warning
 * @brief   舵机控制
 *
 * @todo: 补充
 *
 ********************************************************************************************/
#include "servo.hpp"

namespace Servo_n
{
    /// @brief 舵机构造
    /// @param pwm pwm结构体
    /// @param type 舵机类型
    Servo_c::Servo_c(Servo_Init_t config)
    {
      BSP_n::PWM_Init_t pwm_init = {
          .htim = config.htim,
          .channel = config.channel,
      };
      PWM_ = new BSP_n::PWM_c(pwm_init);
      freq_ = config.freq;
      servo_type_ = config.servo_type;
      switch (servo_type_) {
      case ANGLE180: {
        angle_max_ = 180;
        break;
        }
        case ANGLE270:
        {
            angle_max_ = 270;
            break;
        }
        case ANGLE360:
        {
            angle_max_ = 360;
            break;
        }
        default:
        {
            while (1)
                ;
        }
        }
        uint32_t arr = PWM_->PWMGetArr();
        // 对ccr进行放缩,需要根据实际ARR确定CCR，ARR越大不一定精度越高，需要看具体舵机
        switch (freq_)
        {
        case Hz_50:
        {
            float cofe = arr / (20000.0f - 1);
            min_ccr = 500 * cofe;
            max_ccr = 2500 * cofe;
        }
            break;
        case Hz_333:
        {
            float cofe = arr / (3000.0f - 1);
            min_ccr = 500 * cofe;
            max_ccr = 2500 * cofe;
        }
            break;
        default:
            break;
        }
        this->SetAngle(config.start_angle);
    }

    /// @brief 设置角度
    /// @param angle 舵机角度 : 0-angle_max_
    void Servo_c::SetAngle(float angle)
    {
        if (angle <= angle_max_ && angle >= 0)
        {
            float persent = angle / angle_max_;
            now_angel_ = angle;
            uint32_t set_ccr = (uint32_t)((max_ccr - min_ccr) * persent) + min_ccr;
            PWM_->PWMSetCcr(set_ccr);
            
        }
    }
}