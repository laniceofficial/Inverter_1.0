#ifndef _STEPIN_MOTOR_HPP
#define _STEPIN_MOTOR_HPP

#ifdef __cplusplus
extern "C"
{
#endif
#include "bsp_PWM.hpp"

#ifdef __cplusplus
}
#endif
#define CONNECTION_MOED 1u
// 共阴极接线---1；共阳极接线---0
// 共阴极：分别将Pul-，Dir-，EN-连接到控制系统的地端（与电源地隔离）；此时脉冲输入信号通过Pul+加入，Dir+；EN+在低电平有效
// 共阳极：分别将Pul+，Dir+，EN+连接到STM32板子的输出电压上，脉冲输入信号通过Pul-接入；此时，Dir-控制方向；EN-在高电平有效。即当EN-引脚低电平时，电机失能
// 改动 CONNECTION_MOED 这个值，会影响到设置电平的高低，从而改动电机的使失能和方向
namespace stepin_motor_N
{
    enum stepin_motor_dir_e : uint8_t
    {
        STEP_IN_FORWADR = CONNECTION_MOED,  // 自定义电机向前
        STEP_IN_REVERSE = !CONNECTION_MOED, // 电机向后标志
    };
    enum stepin_motor_state_e : uint8_t
    {
        STEP_IN_DISABLE = CONNECTION_MOED, // 自定义电机失能
        STEP_IN_ENABLE = !CONNECTION_MOED, // 自定义电机使能
    };

    enum stepin_motor_mode_e : uint8_t // 模式，但没用到
    {
        STEP_IN_SPEED_MODE = 0,    // 电机速度模式
        STEP_IN_POSITION_MODE = 1, // 电机位置模式
    };

    // class stepin_main_c //步进电机父类，便于拓展
    // {
    //     public:

    // };

    typedef struct stepin_init_t
    {
        /* data */
        uint16_t dir_pin_ = 0x00; // 方向引脚
        GPIO_TypeDef *dir_port_ = nullptr;

        uint16_t enable_pin_ = 0x00; // 失能引脚
        GPIO_TypeDef *enable_port_ = nullptr;
        uint16_t PerRound_Pulse = 0;
    } stepin_init_t;

    class stepin_motor_c : public bsp_pwm_n::bsp_pwm_c //,public stepin_main_c // 继承pwm
    {
    public:
        stepin_motor_c(bsp_pwm_n::pwm_init_t pwm_init, stepin_init_t &init_cofig);
        stepin_motor_c();
        ~stepin_motor_c();

        void StepIn_direction_set(stepin_motor_dir_e dir);
        void StepIn_motor_enable(stepin_motor_state_e en);
        void StepIn_speed_mode(stepin_motor_dir_e dir, stepin_motor_state_e en);
        void StepIn_angle_set(stepin_motor_dir_e dir, long long angle);
        void StepIn_init(bsp_pwm_n::pwm_init_t pwm_init, stepin_init_t &init_cofig); // 初始化
        static void StepIn_Callback(TIM_HandleTypeDef *htim);
        long long pulse_CNT = 0;

    protected:
        uint16_t PerRound_pulse = 0; // 转一圈需要的脉冲数
        float division_angle = 0;    // 一个脉冲转过的度数
        uint16_t direction_pin;      // 方向引脚
        GPIO_TypeDef *direction_port;

        uint16_t enable_pin; // 失能引脚
        GPIO_TypeDef *enable_port;

    private:
        /* data */
    };

};

#endif /*_STEPIN_MOTOR_HPP*/