extern "C"
{
#include "main.h"
}

#if defined(HAL_ADC_MODULE_ENABLED)
#ifndef __BSP_ADC_HPP
#define __BSP_ADC_HPP

#include "adc.h"
#include <stdint.h>

#define ADC_MAX_NUM 20

namespace BSP_n
{
    class ADC_c
    {
    public:
        ADC_c(ADC_HandleTypeDef *hadc ,uint16_t adc_length);
        uint32_t adc_value(void);
        void Start(void);
        void Stop(void);

    private:
        ADC_HandleTypeDef *hadc_;
        uint16_t adc_value_[ADC_MAX_NUM];
        uint32_t adc_length_;
    };
}

#endif /* !__BSP_ADC_HPP */
#endif /* HAL_UART_MODULE_ENABLED */
