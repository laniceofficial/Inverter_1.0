/************************** Dongguan-University of Technology -ACE**************************
 * @file bsp_adc.cpp
 * @author Lann 梁健蘅 (rendezook@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-09-07
 *******************************************************************************************
 * @verbatim 仅支持ADC多通道DMA采集
 * @verbatim
 * 
 * 
 * @copyright Copyright (c) 2024
 * 
************************** Dongguan-University of Technology -ACE***************************/
#include "bsp_adc.hpp"
#include "stdlib.h"
#if defined(HAL_ADC_MODULE_ENABLED)

namespace BSP_n
{
    /**
     * @brief ADC初始化构造函数，会在每次创建ADC类的新对象时执行。
     * 
     * @param hadc 
     */
    ADC_c::ADC_c(ADC_HandleTypeDef *hadc, uint16_t adc_length)
    ://初始化列表
    hadc_(hadc),
    adc_length_(adc_length)
    {
    }

    /**
     * @brief 获取ADC值
     * 
     * @return uint32_t 
     * @todo 在C++里好像没必要用到，先丢着
     */
    uint32_t ADC_c::adc_value(void)
    {   
        return (uint32_t)&adc_value_;
    }

    /**
     * @brief 开启ADC_DMA
     * 
     */
    void ADC_c::Start(void)
    {
        HAL_ADC_Start_DMA(this->hadc_, (uint32_t *)this->adc_value_, this->adc_length_);
    }

    /**
     * @brief 关闭ADC_DMA
     * 
     */
    void ADC_c::Stop(void)
    {
        HAL_ADC_Stop_DMA(this->hadc_);
    }

}
#endif
