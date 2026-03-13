/*************************** Dongguan-University of Technology -ACE**************************
 * @file    dm_motor.hpp
 * @author  KazuHa12441
 * @version V1.1
 * @date    2024/10/2
 * @brief   达妙电机父类（继承）
 *
 * @todo: 小凳有需要的话来完善吧CAN吧
/ ********************************************************************************************/
#ifndef __DM_MOTOR_HPP
#define __DM_MOTOR_HPP

#define TXTIME 0.001f

#ifdef STM32H723xx
#include "bsp_fdcan.hpp"
#elif STM32F405xx
#include "bsp_can.hpp"
#endif
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#ifdef __cplusplus
}
#endif

namespace DM_Motor_n
{
    typedef enum{
        TORQUE,
        SPEEDC,
        POSITION,
    }MIT_HandOver_e;

    typedef struct
    {   
        float p_des;
        float v_des;
        float Kp;
        float Kd;
        float t_des;
    }OUTParam_s;


    typedef enum {
        DM_UNABLE                = 0, // 失能
        DM_ENABLE                = 1, // 使能
        DM_PORTECT_ZERO_POSITION = 2, // 保存位置零点
        DM_CLEAR_FAULT           = 3  // 清除电机反馈错误内容
    } DM_StartState_e;

    typedef enum {
        UNABLE               = 0,  // 失能
        ENABLE               = 1,  // 使能
        OVER_VOLTAGE         = 8,  // 过压
        UNDER_VOLTAGE        = 9,  // 欠压
        OVER_CURRENT         = 10, // 过流
        MOS_OVERTEMPERTURE   = 11, // MOS过温
        MOTOR_OVERTEMPERTURE = 12, // 电机过热
        LOSE_COMMINUCATE     = 13, // 通讯丢失
        OVER_LOAD            = 14  // 过载
    } DM_NowState_e;

    typedef enum {
        MIT,
        SPEED,
        LOCTION_SPEED
    } DM_MotorMode_e;

    // 通用参数设置(需要根据调试工具确定)
    typedef struct
    {
        float kp_max;
        float kp_min;
        float kd_min;
        float kd_max;
        float v_min; // 速度范围
        float v_max;
        float p_min; // 位置范围
        float p_max;
        float t_min; // 扭矩范围
        float t_max;
        int postion_bits;  // 位置比特
        int velocity_bits; // 速度比特
        int toeque_bits;   // 扭矩比特
        int kp_bits;
        int kd_bits;

        #ifdef STM32H723xx
        BSP_CAN_Part_n::FDCAN_Init_Config_s fdcan_init_config; 

        #elif STM32F407xx
        BSP_CAN_Part_n::CAN_Init_Config_s can_init_config; 
        #elif STM32F405xx
        BSP_CAN_Part_n::CAN_Init_Config_s can_init_config;
        #endif
        
        OUTParam_s output;
    } DM_ModePrame_s;

    // 电机反馈信息
    typedef struct
    {
        DM_NowState_e nowState;
        float postion;
        float velocity;
        float toeque;
        uint8_t mos_temperture;
        uint8_t motor_temperture;
    } DM_GetMotorData_s;

    // 达妙父类（抽象类）
    class DM_Motor_Main_c
    {
    public: 
        DM_GetMotorData_s get_data_; // 电机获取数据
        DM_ModePrame_s *param_;      // 参数结构体指针
        uint8_t txbuffer_[8];      // 发送缓冲区
        static uint8_t dm_idx_;       // 当前CAN上挂载达妙电机的个数
        DM_MotorMode_e mode_ = MIT; // 默认MIT模式，调用对应的构造函数可更改
        DM_Motor_Main_c(DM_ModePrame_s *param_);
        void DMMotorStateSet(DM_StartState_e state);
        DM_GetMotorData_s *ReturnMotorData(void);
        virtual void StateSet(DM_StartState_e State) = 0;
    };
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
    uint16_t float_to_uint(float x_float, float x_min, float x_max, int bits);
    void TxDataToBuffer(uint8_t *pdata1, uint8_t *pdata2);

}

#endif /*__DM_MOTOR_HPP*/