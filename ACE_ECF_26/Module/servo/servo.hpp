#ifndef __SERVO_HPP
#define __SERVO_HPP

#include "bsp_PWM.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

namespace Servo_n
{
   typedef enum
   {
    ANGLE180,
    ANGLE270,
    ANGLE360
   } Servo_Type_e;
   typedef enum
   {
      Hz_50,
      Hz_333,
      
   } Servo_freq_e;
   typedef struct Servo_Init_t
   {
      TIM_HandleTypeDef *htim = nullptr;
      uint32_t channel;
      uint16_t start_angle = 0; // 初始值角度
      Servo_Type_e servo_type = ANGLE180;
      Servo_freq_e freq = Hz_50;
   } Servo_Init_t;

   class Servo_c
   {
   public:
      Servo_c(Servo_Init_t config);
      void SetAngle(float angle);

   private: 
      BSP_n::PWM_c *PWM_;
      Servo_Type_e servo_type_ = ANGLE180;
      Servo_freq_e freq_ = Hz_50;
      float now_angel_;
      float angle_max_;
      uint32_t max_ccr = 0; //脉宽
      uint32_t min_ccr = 0; // 最小脉宽
   };
}

#endif /*__SERVO_HPP*/
