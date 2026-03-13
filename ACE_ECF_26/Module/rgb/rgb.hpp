#ifndef RGB_HPP
#define RGB_HPP
#ifdef __cplusplus
extern "C"
{
#endif
// #include "stm32g431xx.h"
#include "main.h"
#ifdef __cplusplus
}
#endif

class RGB_c
{
public:
    void Init(GPIO_TypeDef *PortRed_, uint16_t PinRed_, GPIO_TypeDef *PortGreen_,
              uint16_t PinGreen_, GPIO_TypeDef *PortBlue_, uint16_t PinBlue_);
    void Task();
    // void SetSignal(SIGNAL_e signal);
    void LightRed();
    void LightGreen();
    void LightBlue();
    void LightPurple();
    void LightOff();
    void LightOrange();
    void LightYellow();
    void LightCyan();
  
private:
    GPIO_TypeDef *PortRed;
    uint16_t PinRed;
    GPIO_TypeDef *PortGreen;
    uint16_t PinGreen;
    GPIO_TypeDef *PortBlue;
    uint16_t PinBlue;
    // SIGNAL_e signal_ = SIGNAL_e::INIT;
};
#endif // ! RGB_HPP
