/************************** Dongguan-University of Technology -ACE**************************
 * @file  Motor_General_def.h
 * @brief 电机的通用配置定义文件
 * @author wuage2335俞永祺
 * @version 1.0
 * @date 2022-11-24
 *
 * ==============================================================================
 * @endverbatim
 ************************** Dongguan-University of Technology -ACE***************************/
#ifndef __MOTOR_GENERAL_DEF_HPP
#define __MOTOR_GENERAL_DEF_HPP

#include "Alg_PID.hpp"
#include "main.h"

#ifdef STM32H723xx
#include "bsp_fdcan.hpp"
#else
#include "bsp_can.hpp"
#endif // USE_H7_if_or_not

#include "General_def.hpp"

namespace Motor_General_Def_n
{
    /* 电机正反转标志 */
    typedef enum
    {
        MOTOR_DIRECTION_NORMAL = 0, // 正转
        MOTOR_DIRECTION_REVERSE = 1 // 反转
    } Motor_Reverse_Flag_e;

    /* 反馈量正反标志 */
    typedef enum
    {
        FEEDBACK_DIRECTION_NORMAL = 0, // 反馈量为正
        FEEDBACK_DIRECTION_REVERSE = 1 // 反馈量为负
    } Feedback_Reverse_Flag_e;
    typedef enum
    {
        MOTOR_STOP = 0,
        MOTOR_ENALBED = 1,
        MOTOR_LOCK = 2, // 龙门架工程专用（待开发）
        MOTOR_INIT = 3, // 龙门架工程专用（待开发）
        MOTOR_SELF_LOCK = 4,      // 电机自锁
        MOTOR_OUTPUT_ONLY_ME = 5, // 发送自己设定的值，不适用PID的计算值
        MOTOR_ONLY_RECEIVE = 6,   // 只用于接收电机的数据，不进行发送
    } Motor_Working_Type_e;

    /* 反馈来源设定,若设为OTHER_FEED则需要指定数据来源指针,详见Motor_Controller_s*/
    typedef enum
    {
        MOTOR_FEED    = 0, // 默认用累计编码值和反馈的速度
        OTHER_FEED    = 1, // 使用非电机的反馈，如陀螺仪的反馈，需要自己调用函数传入指针
        // 角度方面
        TOTAL_ANGLE   = 2, // 总角度
        RECORD_LENGTH = 3, // 输出轴累计长度
        // 速度方面
        SPEED_APS     = 4, // 角速度
        ANGULAR_SPEED = 5, // 输出轴角速度
        LINEAR_SPEED  = 6,  //  输出轴线速度
        // 反馈固定为0
        NONE_FEED     = 7, 
    } Feedback_Source_e;
    // 闭环部分
    typedef enum
    {
        OPEN_LOOP = 0b0000,    // 开环
        CURRENT_LOOP = 0b0001, // 电流闭环
        SPEED_LOOP = 0b0010,   // 速度闭环
        ANGLE_LOOP = 0b0100,   // 角度闭环

        // only for checking
        SPEED_AND_CURRENT_LOOP = 0b0011, // 速度和电流闭环
        ANGLE_AND_SPEED_LOOP = 0b0110,   // 角度和速度闭环
        ALL_THREE_LOOP = 0b0111,         // 角度，速度，电流三闭环
    } Closeloop_Type_e;

    typedef enum
    {
        FEEDFORWARD_NONE = 0b00,
        CURRENT_FEEDFORWARD = 0b01,
        SPEED_FEEDFORWARD = 0b10,
        CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
    } Feedfoward_Type_e;
    /* 电机控制设置,包括闭环类型,反转标志和反馈来源 */
    typedef struct
    {
        Closeloop_Type_e outer_loop_type;              // 最外层的闭环,未设置时默认为最高级的闭环
        Closeloop_Type_e close_loop_type;              // 使用几个闭环(串级)
        Motor_Reverse_Flag_e motor_reverse_flag;       // 是否反转
        Feedback_Reverse_Flag_e feedback_reverse_flag; // 反馈是否反向
        Feedback_Source_e angle_feedback_source;       // 角度反馈类型
        Feedback_Source_e speed_feedback_source;       // 速度反馈类型
    } Motor_Control_Setting_s;

    /* 电机类型枚举 */
    typedef enum
    {
        MOTOR_TYPE_NONE = 0,
        GM6020,
        M3508,
        M2006,
        LK9025,
        HT04,
    } Motor_Type_e;

    /**
     * @brief 电机控制器初始化结构体,包括三环PID的配置以及两个反馈数据来源指针
     *        如果不需要某个控制环,可以不设置对应的pid config
     *        需要其他数据来源进行反馈闭环,不仅要设置这里的指针还需要在Motor_Control_Setting_s启用其他数据来源标志
     */
    typedef struct
    {
        pid_alg_n::PID_Init_Config_t current_PID;
        pid_alg_n::PID_Init_Config_t speed_PID;
        pid_alg_n::PID_Init_Config_t angle_PID;
    } Motor_Controller_Init_s;

    /* 用于初始化CAN电机的结构体,各类电机通用 */
    typedef struct
    {
        Motor_Controller_Init_s controller_param_init_config;
        Motor_Control_Setting_s controller_setting_init_config;
        Motor_Type_e motor_type;
#ifdef STM32H723xx
        BSP_CAN_Part_n::FDCAN_Init_Config_s fdcan_init_config;

#else
        BSP_CAN_Part_n::CAN_Init_Config_s can_init_config;
#endif
        float radius;        // 输出轴半径
        float ecd2length;    // 编码值转长度, 单位为: 编码/毫米
        int16_t zero_offset; // 供绝对编码使用,单位是°
    } Motor_Init_Config_s;

    /* 电机控制器,包括其他来源的反馈数据指针,3环控制器和电机的参考输入*/
    // 后续增加前馈数据指针
    class Motor_Controller_c
    {
    private:
        float pid_ref = 0; // 将会作为每个环的输入和输出顺次通过串级闭环
    public:
        pid_alg_n::pid_alg_c current_PID;
        pid_alg_n::pid_alg_c speed_PID;
        pid_alg_n::pid_alg_c angle_PID;
        void RefValChange(float ref_val);
        float GetRefVal(void);
    };

}

#endif // !__MOTOR_GENERAL_DEF_HPP
