#ifndef CYBERGEAR_DRIVER_HPP
#define CYBERGEAR_DRIVER_HPP

#ifdef __cplusplus
extern "C" {

#include "main.h"
#include <stdbool.h>

#endif

#ifdef __cplusplus
}
#endif
#include "motor_base_def.hpp"

namespace Motor_n {
    namespace CyberGearMotor_n {
        /*---ID值宏定义---*/
        #define MASTER_ID 0
        #define MOTOR_ID 127 //pitch电机127号

        //最大搭载数量
        #define MAX_MI_NUM 10

        //重启计数
        #define MAX_COUNT 2

        /*---写入参数地址宏定义---*/
        #define SET_MODE_INDEX 0x7005
        #define POSTION_MODE_SET_POSTION_INDEX 0x7016
        #define POSTION_MODE_SET_SPEED_INDEX 0x7017
        #define SPEED_MODE_SET_CURRENT_INDEX 0x7018

        /*---CAN拓展标识符宏操作---*/
        #define txCanIdEx (((struct ExtId)&(txMsg.tx_extid)))
        #define rxCanIdEx (((struct ExtId)&(rxMsg.rx_efid))) //将扩展帧id解析为定义数据结构
        /*电机参数限制*/
        #define P_MIN -12.5f
        #define P_MAX 12.5f
        #define V_MIN -30.0f
        #define V_MAX 30.0f
        #define KP_MIN 0.0f
        #define KP_MAX 500.0f
        #define KD_MIN 0.0f
        #define KD_MAX 5.0f
        #define T_MIN -12.0f
        #define T_MAX 12.0f

        #define Temp_Gain 0.1
        #define MAX_P 720      //位置范围只有-4PI到4PI
        #define MIN_P -720
        #define MAX_S 30
        #define MIN_S -30
        #define MAX_T 12
        #define MIN_T -12

        typedef enum{
            RESET_MODE = 0,//Reset[模式]
            CALI_MODE  = 1,//Cali 模式[标定]
            RUN_MODE   = 2 //Motor模式[运行]
        } motor_mode_e;//电机模式

        typedef enum{
            control = 0,//运控模式
            postion = 1,//位置模式
            speed   = 2,//速度模式
            current = 3 //电流模式
        }Mode_rum_mode_t;//模式状态

        //电机参数列表
        typedef struct{
            Mode_rum_mode_t Mode_rum_mode; //0: 运控模式  1: 位置模式  2: 速度模式  3: 电流模式
            float iq_ref;    //电流模式Iq指令  A
            float spd_ref;  //转速模式转速指令 rad/sA
            float imit_torque;
            float cur_kp;  //电流Kp
            float cur_ki;  //电流Ki
            float cur_filt_gain; //电流滤波系数
            float loc_ref;  //位置模式角度 rad
            float limit_spd;
            float limit_cur;
        }motor_parameter_list_t;

        //电机故障及警告列表
        typedef struct{
            bool Over_temperature_warning;  //过温警告

            bool Over_temperature_fault;    //过温故障
            bool Over_voltage_fault;        //过压故障
            bool Over_current_fault;        //过流故障
            bool Overload_fault;            //过载故障
            bool Under_voltage_fault;       //欠压故障
            bool Encoder_not_calibrated_fault;//编码器未标定故障
            bool Magnetic_encoding_fault;   //磁编码故障
            bool HALL_encoding_fault;       //HALL编码故障
            bool Drive_core_fault;          //驱动芯片故障
            bool A_phase_current_sampling_overcurrent;//A相电流采用过流
            bool B_phase_current_sampling_overcurrent;//B相电流采用过流
            bool C_phase_current_sampling_overcurrent;//C相电流采用过流
        }motor_error_list_t;//电机故障及警告列表

        //电机参数下标
        typedef enum{
            run_mode        = 0x7005,  //0: 运控模式  1: 位置模式  2: 速度模式  3: 电流模式
            iq_ref          = 0x7006,  //电流模式Iq指令   -23 - 23A
            spd_ref         = 0x700A,  //转速模式转速指令 -30 - 30rad/s
            imit_torque     = 0x700B,  //最大转矩 0-12Nm
            cur_kp          = 0x7010,  //电流Kp 默认0.125
            cur_ki          = 0x7011,  //电流Ki 默认0.0158
            cur_filt_gain   = 0x7014,  //电流滤波系数 0-1 默认0.1
            loc_ref         = 0x7016,  //位置模式角度指令 rad
            limit_spd       = 0x7017,  //位置模式最大速度 0-30rad/s
            limit_cur       = 0x7018,  //位置模式最大电流 0-23A
            mechPos         = 0x7019,  //负载端机械角度 rad           只读
            iqf             = 0x701A,  //iq滤波值   -23 - 23A        只读
            mechVel         = 0x701B,  //负载端转速 -30 - 30rad/s     只读
            VBUS            = 0x701C,  //母线电压 V                  只读
            retation        = 0x701D,  //圈数
            loc_kp          = 0x701E,  //位置Kp  30
            spd_kp          = 0x701F,  //速度Kp  1
            spd_ki          = 0x7020,  //速度Ki  0.002
        }parameter_index_e;

        //CAN拓展标识符结构体  motor_id、data、mode共29位
        typedef struct {
            uint32_t motor_id:8;    //bit7~0        电机canID
            uint32_t data:16;       //bit15~23       信息位
            uint32_t mode:5;        //bit28~24      通信类型
            uint32_t res:3;         //占空位，无信息
        }txExtId_t;

        typedef struct {
            uint32_t motor_id:8;                //bit7~0    电机canID
            uint32_t motor_now_id:8;            //bit15~8   当前电机canID
            uint32_t Under_voltage_fault:1;     //bit16     欠压故障
            uint32_t Over_current:1;            //bit17     过流
            uint32_t Over_temperature:1;        //bit18     过温
            uint32_t Magnetic_encoding_fault:1; //bit19     磁编码故障
            uint32_t HALL_encoding_failt:1;     //bit20     HALL编码故障
            uint32_t Uncalibrated:1;            //bit21     未标定
            uint32_t Mode_status:2;             //bit22~23  模式状态
            uint32_t mode:5;                    //bit28~24  通信类型
            uint32_t res:3;                     //占空位，无信息
        }rxExtId_t;

        //CAN通信结构体 主控板->电机
        typedef struct {
            txExtId_t     ExtId;//CAN 29位拓展标识符
            uint8_t       Data[8];//发送数据
        }txMsg_t;

        //CAN通信结构体 电机->主控板
        typedef struct {
            rxExtId_t     ExtId;//CAN 29位拓展标识符
            uint8_t       Data[8];//接收数据
        }rxMsg_t;

        class CyberGearDriver_c:public MotorBaseDef_n::MotorBase_c
        {
        public:
            static CyberGearDriver_c* MI_motor_instance_head;//实例链表头
            static uint8_t MI_num_;  //小米电机数量
            txMsg_t            txMsg;//发送
            rxMsg_t            rxMsg;//接收
            Mode_rum_mode_t current_mode = control; //0: 运控模式（默认）  1: 位置模式  2: 速度模式  3: 电流模式
            motor_parameter_list_t motor_parameter_list;//参数列表
            motor_error_list_t    motor_error_list;     //状态列表
            CyberGearDriver_c* next_;

            MotorBase_c& get_base() {return *this;} // 获取基类引用
            CyberGearDriver_c(MotorBaseDef_n::Motor_Base_Config_t motor_config);
            void MiMotorSetMechPosition2Zero();
            void MiMotorGetId();
            void MiMotorEnable();
            void MiMotorStop();
            void MiMotorChangeID(uint8_t Target_ID);
            void MiMotorWriteOnePara(parameter_index_e index , float Value);
            void MiMotorReadOnePara(parameter_index_e index);
            void SetMode(Mode_rum_mode_t Mode);

            //电机四模式控制函数
            void MiMotorControlMode(float torque, float MechPosition , float speed , float kp , float kd);
            void MiSpeedModeSetSpeed(float set_speed);
            void MiPostionModeSetPosition(float set_position);
            void MiCurrentModeSetCurrent(float set_current);

    #ifdef STM32H723xx
                friend void MiMotorReceiveCallback(BSP_n::Fdcan_c* register_instance);
    #else
                friend void MiMotorReceiveCallback(BSP_n::Can_c* register_instance);
    #endif
            };
        }
    }

#endif //CYBERGEAR_DRIVER_HPP
