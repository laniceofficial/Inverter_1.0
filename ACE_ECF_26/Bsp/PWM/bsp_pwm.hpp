#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include <cstdint>
#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "tim.h"

#ifdef __cplusplus
}

namespace BSP_n
{

    // 回调
    /*用using将此名称定义为一个函数指针类型*/
    using pwm_overflow_callback = void (*)(); // 计数器溢出中断回调函数
    using pwm_compare_callback = void (*)();  // 达到比较值触发回调函数

    typedef enum
    {
        PWM_NORMAL = 0x00,
        PWM_OVERFLOW_IT = 0x01, // 定时器溢出中断
        PWM_COMPARE_IT = 0x02,  // 比较中断
        PWM_DMA_SEND = 0x04,    // DMA发送
    } PWM_Mode_e;

    typedef struct PWM_Init_t {
      TIM_HandleTypeDef *htim = nullptr;
      uint32_t channel=0;
      uint8_t pwm_mode = PWM_NORMAL;                   // pwm模式(可选多种)
      pwm_overflow_callback pwm_overflow_callback_ptr = nullptr; // 溢出中断（没有给null）
      pwm_compare_callback pwm_compare_callback_ptr= nullptr;   // 比较中断（没有给null）
      uint8_t is_HobbyWing=false;                            // 是否用于好盈电调控制
      uint8_t first_duty = 0;                          // 初始值设置
    } PWM_Init_t;

    class PWM_c
    {
    public:
        PWM_c(PWM_Init_t pwm_init);
        PWM_c();
        void Init(PWM_Init_t pwm_init);
        inline void PWMSetPsc(uint16_t psc);
        inline void PWMSetArr(uint32_t arr);
        inline void PWMSetCcr(uint32_t ccr);
        inline uint16_t PWMGetPsc();
        inline uint32_t PWMGetArr();
        inline uint32_t PWMGetCcr();
        inline void PWMSetDuty(uint8_t duty);
        inline void PWMSetDuty(float duty);
        inline TIM_HandleTypeDef *GetHtim();
        inline uint32_t GetChannel();
        inline uint8_t GetMode();
        void PWMDMASendArray(uint8_t *buf, uint16_t len);
        pwm_overflow_callback pwm_overflow_callback_ptr_ = nullptr; // 溢出中断（没有给null）
        pwm_compare_callback pwm_compare_callback_ptr_ = nullptr;   // 比较中断（没有给null）
        // 好盈电机
        void HobbyWingESCControl(uint16_t pwm);
        PWM_c *instance_next_ = nullptr;

    private:
        TIM_HandleTypeDef *htim_ = nullptr;
        uint32_t channel_;
        uint8_t is_HobbyWing_;
        uint8_t pwm_mode_;
    };

    /**
     * @brief 设置定时器的预分频值
     * @param psc:预分频值
     */
    inline void PWM_c::PWMSetPsc(uint16_t psc) { __HAL_TIM_PRESCALER(this->htim_, psc); }

    /**
     * @brief 设置定时器的最大计数值
     * @param psc:预分频值
     */
    inline void PWM_c::PWMSetArr(uint32_t arr) { __HAL_TIM_SetAutoreload(this->htim_, arr); }

    /**
     * @brief 设置定时器的比较值
     * @param ccr:比较值
     */
    inline void PWM_c::PWMSetCcr(uint32_t ccr) { __HAL_TIM_SetCompare(this->htim_, this->channel_, ccr); }
 /**
     * @brief 设置PWM占空比
     * @param duty:占空比(0~100)
     */
    inline void PWM_c::PWMSetDuty(uint8_t duty)
    {
        duty =  (((duty) >(100)?(100):(duty)));//限制在0~100
        uint32_t compare = (uint32_t)(htim_->Instance->ARR * (duty / 100.f));
        PWMSetCcr(compare);
    }
    /**
     * @brief 设置PWM占空比
     * @param duty:占空比(0~100)
     */
    inline void PWM_c::PWMSetDuty(float duty)
    {
        duty =  ((duty)<(0)?(0):((duty) >(100)?(100):(duty)));//限制在0~100
        uint32_t compare = (uint32_t)(htim_->Instance->ARR * (duty / 100.f));
        PWMSetCcr(compare);
    }
    /**
     * @brief 获取定时器的预分频值
     * @return uint16_t:预分频值
     */
    inline uint16_t PWM_c::PWMGetPsc() { return (this->htim_->Instance->PSC); }

    /**
     * @brief 获取定时器的最大计数值
     * @return uint16_t:最大计数值
     */
    inline uint32_t PWM_c::PWMGetArr() { return __HAL_TIM_GetAutoreload(this->htim_); }

    /**
     * @brief 获取定时器的比较值
     * @return uint16_t:比较值
     */
    inline uint32_t PWM_c::PWMGetCcr() { return __HAL_TIM_GetCompare(this->htim_, this->channel_); }

    inline TIM_HandleTypeDef *PWM_c::GetHtim() { return this->htim_; }

    inline uint32_t PWM_c::GetChannel() { return this->channel_; }

    inline uint8_t PWM_c::GetMode() { return this->pwm_mode_; }


};

#endif

#endif
