#include "rgb.hpp"
void RGB_c::Init(GPIO_TypeDef *PortRed_, uint16_t PinRed_, GPIO_TypeDef *PortGreen_,
                 uint16_t PinGreen_, GPIO_TypeDef *PortBlue_, uint16_t PinBlue_)
{
    PortBlue = PortBlue_;
    PinBlue = PinBlue_;
    PortGreen = PortGreen_;
    PinGreen = PinGreen_;
    PortRed = PortRed_;
    PinRed = PinRed_;
    HAL_GPIO_WritePin(PortRed, PinRed, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PortBlue, PinBlue, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PortGreen, PinGreen, GPIO_PIN_SET);
    // 高电平失能
}
void RGB_c::LightRed()
{
    HAL_GPIO_WritePin(PortRed, PinRed, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PortBlue, PinBlue, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PortGreen, PinGreen, GPIO_PIN_SET);
}
void RGB_c::LightGreen()
{
    HAL_GPIO_WritePin(PortRed, PinRed, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PortBlue, PinBlue, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PortGreen, PinGreen, GPIO_PIN_RESET);
}
void RGB_c::LightBlue() //蓝色
{
    HAL_GPIO_WritePin(PortRed, PinRed, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PortGreen, PinGreen, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PortBlue, PinBlue, GPIO_PIN_RESET);
}
void RGB_c::LightPurple() // 紫色
{
    HAL_GPIO_WritePin(PortRed, PinRed, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PortBlue, PinBlue, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PortRed, PinRed, GPIO_PIN_RESET);
}
void RGB_c::LightOff()
{
    HAL_GPIO_WritePin(PortRed, PinRed, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PortBlue, PinBlue, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PortGreen, PinGreen, GPIO_PIN_SET);
}
void RGB_c::LightOrange() // 橙色
{
    HAL_GPIO_WritePin(PortRed, PinRed, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PortBlue, PinBlue, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PortRed, PinGreen, GPIO_PIN_SET);
}
void RGB_c::LightYellow() // 黄色
{
    HAL_GPIO_WritePin(PortRed, PinRed, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PortBlue, PinBlue, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PortRed, PinGreen, GPIO_PIN_RESET);
}
void RGB_c::LightCyan() // 青色
{
    HAL_GPIO_WritePin(PortRed, PinRed, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PortBlue, PinBlue, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PortRed, PinGreen, GPIO_PIN_RESET);
}