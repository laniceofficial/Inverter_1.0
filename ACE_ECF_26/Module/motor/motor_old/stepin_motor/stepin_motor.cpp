/*************************** Dongguan-University of Technology -ACE**************************
 * @file    stepin_motor.cpp
 * @author  laniceee
 * @version V1.0
 * @date    2024/10/25
 * @brief
 *   步进电机cpp,包括失能，正反转，速度模式，角度模式
 * @todo
 *   找到更好的方式来替代这种不断进入中断而占用资源的方式；或者直接换电机。。搞个有编码器的
 ********************************************************************************************
 * @attention 请注意接线问题！！！ 确保接线是 共阴极 或 共阳极 然后在hpp的宏定义进行设置
 * @verbatim
 *    1. cube: 需要在cube使能一个定时器TIM，输出一路PWM方波，测试例子为 ARR = 1000-1; Period = 168-1
 *    2. cube: 使能中断(角度模式需要)，为global interrupt 或者 update interrupt
 *    3. 程序： 创建一个pwm类，再创建一个步进电机初始化结构体，通过创建一个步进电机类并向构造函数传入前两个参数，或者使用init函数初始化
 *    4. 速度模式：直接占空比为50%
 *    5. 角度模式：在HAL_TIM_PeriodElapsedCallback 中断回调中通过一个个计数的方式记录步进值，达到转动固定角度的方法
 * @example
 * 该例子需要先创建一个pwm类，再创建一个步进电机初始化结构体，通过创建一个步进电机类并向构造函数传入前两个参数，或者使用init函数初始化
 * bsp_pwm_n::pwm_init_t pwm_init = {
    .htim = &htim1,
    .channel = TIM_CHANNEL_1,
    .first_value = 0,
    // TIM_HandleTypeDef *htim;
    // uint32_t channel;
                                  // pwm模式(可选多种)
    // pwm_overflow_callback pwm_overflow_callback_ptr; // 溢出中断（没有给null）
    // pwm_compare_callback pwm_compare_callback_ptr;   // 比较中断（没有给null）
    // uint8_t is_HobbyWing;                            // 是否用于好盈电调控制
    // uint16_t first_value = 0;                        // 初始值设置
    };
 * stepin_init_t step_ = {
        .dir_pin_ = GPIO_PIN_11, // 方向为PE11
        .dir_port_ = GPIOE,
        .enable_pin_ = GPIO_PIN_13, // 使能为PE13
        .enable_port_ = GPIOE,
        .PerRound_Pulse = (uint8_t)800,
    };
    stepin_motor_c temp(step_);
 *  temp.StepIn_init(pwm_init, step_);
 *
 ************************** Dongguan-University of Technology -ACE***************************/

// 确保接线是 共阴极 或 共阳极 然后在hpp的宏定义进行设置
#include "stepin_motor.hpp"
// 我需要一个能精确的定位到每个脉冲的tim，设置好arr或者脉冲数量，到达时就进入中断直接暂停输出
namespace stepin_motor_N
{
#define STEP_IN_MAX_NUMS 2
    uint8_t stepin_nums = 0;
    stepin_motor_c *stepin_arr[STEP_IN_MAX_NUMS] = {nullptr};
    stepin_motor_c::stepin_motor_c(bsp_pwm_n::pwm_init_t pwm_init, stepin_init_t &init_cofig) : bsp_pwm_c(pwm_init)
    {
        direction_pin = init_cofig.dir_pin_;
        direction_port = init_cofig.dir_port_;
        enable_pin = init_cofig.enable_pin_;
        enable_port = init_cofig.enable_port_;            // 赋值
        PerRound_pulse = init_cofig.PerRound_Pulse;       // 一圈需要的脉冲
        division_angle = (360.f / (float)PerRound_pulse); // 计算步进细分角度，即一个脉冲转过的角度

        // pwm_compare_callback_ptr_ = StepIn_Callback;
        StepIn_motor_enable(STEP_IN_ENABLE);
        stepin_arr[stepin_nums++] = this; // 将指针保存到本地变量
        if (stepin_nums + 1 > STEP_IN_MAX_NUMS)
        {
            {
                while (1)
                    ;
            }
        }
    }
    stepin_motor_c::stepin_motor_c()
    {
    }
    stepin_motor_c::~stepin_motor_c()
    {
    }
    // 直接继承可以进该pwm的构造函数
    void stepin_motor_c::StepIn_init(bsp_pwm_n::pwm_init_t pwm_init, stepin_init_t &init_cofig) // 自定义初始化函数
    {
        htim_ = pwm_init.htim;
        channel_ = pwm_init.channel;
        is_HobbyWing_ = pwm_init.is_HobbyWing;

        direction_pin = init_cofig.dir_pin_;
        direction_port = init_cofig.dir_port_;

        enable_pin = init_cofig.enable_pin_;
        enable_port = init_cofig.enable_port_;
        PerRound_pulse = init_cofig.PerRound_Pulse;     // 一圈需要的脉冲
        division_angle = 360.f / (float)PerRound_pulse; // 计算步进细分角度，即一个脉冲转过的角度
        ECF_PWM_Set_CCR(pwm_init.first_value);
        HAL_TIM_PWM_Start(htim_, channel_); // pwm构造函数没有的

        StepIn_motor_enable(STEP_IN_ENABLE);

        stepin_arr[stepin_nums++] = this; // 将指针保存到本地变量

        if (stepin_nums + 1 > STEP_IN_MAX_NUMS)
        {
            {
                while (1)
                    ;
            }
        }
    }
    /*速度模式设定速度*/
    void stepin_motor_c::StepIn_speed_mode(stepin_motor_dir_e dir, stepin_motor_state_e en)
    {
        StepIn_direction_set(dir);
        if (en == STEP_IN_ENABLE)
        {
            ECF_PWM_Set_CCR(htim_->Instance->ARR / 2); // 设置为一半占空比
            //__HAL_TIM_SET_COMPARE(pwm_instance.htim_, pwm_instance.channel_, pwm_instance.htim_->Instance->ARR / 2); // 开启PWM输出
        }
        else
        {
            ECF_PWM_Set_CCR(0); // 不失能电机，而是不输出pwm
            // __HAL_TIM_SET_COMPARE(pwm_instance.htim_, pwm_instance.channel_, 0); // 关闭PWM输出
        }
    }
    /**
     * @brief  步进电机方向设定
     * @param  stepin_motor_dir_e dir STEP_IN_FORWADR为正向  STEP_IN_REVERSE为逆向
     *
     * @retval 无
     */
    void stepin_motor_c::StepIn_direction_set(stepin_motor_dir_e dir)
    {
        if (dir == STEP_IN_FORWADR)
        {
            HAL_GPIO_WritePin(direction_port, direction_pin, (GPIO_PinState)STEP_IN_FORWADR);
        }
        else
        {
            HAL_GPIO_WritePin(direction_port, direction_pin, (GPIO_PinState)STEP_IN_REVERSE);
        }
    }

    /**
     * @brief 用于失能引脚，但在实际中一般只将pwm失能或设置0脉冲
     * @param stepin_motor_state_e en 是否使能
     */
    void stepin_motor_c::StepIn_motor_enable(stepin_motor_state_e en)
    {
        if (en == STEP_IN_ENABLE)
        {
            HAL_GPIO_WritePin(enable_port, enable_pin, (GPIO_PinState)STEP_IN_ENABLE);
        }
        else
        {
            HAL_GPIO_WritePin(enable_port, enable_pin, (GPIO_PinState)STEP_IN_DISABLE);
        }
    }

    /**
     * @brief 使步进电机转动固定角度
     * @date 2024-10-19
     * @details 本次使用5718HB3401步进电机的步进角为1.8度，通过细分达到输出若干脉冲转动一圈的效果
     *          这里设置为800脉冲一圈，也就是一个脉冲0.45度
     */
    void stepin_motor_c::StepIn_angle_set(stepin_motor_dir_e dir, long long angle)
    {
        if (angle != 0)
        {
            HAL_TIM_Base_Start_IT(htim_); // 开启定时器中断
            pulse_CNT = angle / division_angle;
            StepIn_direction_set(dir);
            ECF_PWM_Set_CCR(htim_->Instance->ARR / 2);
        }
        else
        {
            HAL_TIM_Base_Start_IT(htim_); // 开启定时器中断
            pulse_CNT = 0;
            ECF_PWM_Set_CCR(0);
        }
    }

    void stepin_motor_c::StepIn_Callback(TIM_HandleTypeDef *htim)
    {
        // 需开始定时器
        uint8_t i = 0;
        for (i = 0; i < stepin_nums; i++)
        {
            if (stepin_arr[i]->htim_ == htim)
            {
                if (stepin_arr[i]->pulse_CNT != 0)
                {
                    stepin_arr[i]->pulse_CNT--; // 步进电机计数脉冲
                }
                else
                {
                    // StepIn_MotorENABLE(0);//当没有输入信号产生时，不使能步进电机
                    stepin_arr[i]->ECF_PWM_Set_CCR(0);
                }
            }
        }
    }
}
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//     /* USER CODE BEGIN Callback 0 */

//     /* USER CODE END Callback 0 */
//     if (htim->Instance == TIM3)
//     {
//         HAL_IncTick();
//     }
//     /* USER CODE BEGIN Callback 1 */

//     stepin_motor_N::stepin_motor_c::StepIn_Callback(htim);
//     /* USER CODE END Callback 1 */
// }