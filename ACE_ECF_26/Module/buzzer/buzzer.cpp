#include "buzzer.hpp"
#include "user_maths.hpp"
namespace Buzzer_n
{
    Buzzer_c::Buzzer_c(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t apb_mhz) : use_tim_(htim), channel_(channel)
    {
        apb_hz_ = apb_mhz * 1000000;
        HAL_TIM_Base_Start(htim);
        HAL_TIM_PWM_Start(htim, channel_);
    }

    Buzzer_c::Buzzer_c(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t apb_mhz, float allow_dif) : use_tim_(htim), channel_(channel), allow_dif_(allow_dif)
    {
        apb_hz_ = apb_mhz * 1000000;
        HAL_TIM_Base_Start(htim);
        HAL_TIM_PWM_Start(htim, channel_);
    }

    void Buzzer_c::On()
    {
        __HAL_TIM_SetCompare(use_tim_, channel_, arr_ / 2);
    }

    void Buzzer_c::Off()
    {
        __HAL_TIM_SetCompare(use_tim_, channel_, 0);
    }

    void Buzzer_c::Test(uint16_t psc, uint16_t arr)
    {
        arr_ = arr;
        __HAL_TIM_PRESCALER(use_tim_, psc);
        __HAL_TIM_SetAutoreload(use_tim_, arr);
        On();
    }

    // 在满足arr足够大情况下计算psc与arr值
    // 确保最终实际频率与目标频率控制在allow_dif差值内
    void Buzzer_c::Play(float note)
    {
        if (note < N_A0)
        {
            Off();
            return;
        }
        uint32_t temp = (uint32_t)((float)apb_hz_ / note);
        for (uint16_t arr = 65535; arr >= 3; arr -= 2)
        {
            uint16_t psc = (temp / arr);

            if (abs(apb_hz_ / psc / arr - note) < allow_dif_)
            {
                Test(psc - 1, arr - 1);
                return;
            }
        }
    }
}
