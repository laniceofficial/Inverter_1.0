#ifndef LK9025_H
#define LK9025_H


#include "main.h"
#include "bsp_can.hpp"
#include "Motor_General_def.hpp"
#include "Alg_PID.hpp"
#include "safe_task.hpp"
#include "user_maths.hpp"

#define LK_MOTOR_MX_CNT 32 // 使用单电机指令，最多允许32个电机挂在在总线上

#define CURRENT_SMOOTH_COEF 0.9f
#define SPEED_SMOOTH_COEF 0.85f
#define REDUCTION_RATIO_DRIVEN 1
#define LK_ENCODER 65535 //16384  //一圈编码值,手册上是16384，但是实际测试发现是65535
#define ECD_ANGLE_COEF_LK (360.0f / ((1.0f)*LK_ENCODER))
#define CURRENT_TORQUE_COEF_LK 0.003645f // 电流设定值转换成扭矩的系数,算出来的设定值除以这个系数就是扭矩值

typedef enum
{
    Read_PID = 0x30,  //读取PID参数
    PID_To_RAM = 0x31, //写入PID参数到RAM（掉电丢失）
    PID_To_ROM = 0x32, //写入PID参数到ROM（掉电不丢失）
    Read_Accel = 0x33, //读取加速度命令
    Write_Accel = 0x34, //写入加速度命令到RAM
    Read_Encoder = 0x90, //读取编码器命令
    Write_Encoder_Offset_To_ROM = 0x91, //写入编码器值到ROM作为电机零点命令
    Write_Now_Encoder_As_Zero_To_ROM = 0x19, //写入当前位置到ROM作为电机零点命令
    Read_Muli_Turns = 0x92, //读取多圈角度命令
    Read_Single_Turns = 0x94, //读取单圈角度命令
    Delect_Turns = 0x95, //清除电机角度命令（设置初始值）
    Read_State_And_Error_1 = 0x9A,
    Delect_Error_State = 0x9B,
    Read_State_2 = 0x9C,
    Read_State_3 = 0x9D,
    Motor_Close = 0x80, //电机锁定信号
    Motor_Stop = 0x81, //电机停止信号
    Motor_Stop_To_Run = 0x88, //电机运行命令
    Torque_Open_Loop_Control = 0xA0,  //转矩开环控制
    Torque_Closed_Loop_Control = 0xA1, //转矩闭环控制
    Speed_Closed_Loop_Control = 0xA2,  //速度闭环控制
    Position_Closed_Loop_Control_1 = 0xA3, //位置闭环控制模式
    Position_Closed_Loop_Control_2 = 0xA4,
    Position_Closed_Loop_Control_3 = 0xA5,
    Position_Closed_Loop_Control_4 = 0xA6,
    Position_Closed_Loop_Control_5 = 0xA7,
    Position_Closed_Loop_Control_6 = 0xA8,

    Mul_Motor_Torque_Control = 0x280

} Command_LK9025_ID;

typedef enum{
    Open,    //开环
    Speed,   //速度控制
    Position,//位置控制
    Torque   //力矩控制
}LK9025_Ctrl_type_e;

typedef struct // 9025
{
    uint16_t feedback_ecd;               // 反馈数据, 电机末端单圈编码, 0-16383,刻度总共有16384格
    int16_t feedback_speed;                // 反馈数据, 电机末端实时转速, 单位bps(°/s)
    int16_t feedback_real_current;       // 反馈数据, 实际电流
    int8_t feedback_temperature;        // 反馈数据, 温度 Celsius

    uint16_t last_ecd;        // 上一次读取的编码器值
    float angle_single_round; // 单圈角度

    float total_angle;   // 总角度
    int32_t total_round; // 总圈数

    uint8_t first_flag;
} LKMotor_Measure_t;

class LKMotorInstance{
    public:
    uint8_t motor_id_;  //电机ID号，可设置为0~32，发送标识符为0x140+ID
    LKMotor_Measure_t measure_;
    Motor_General_Def_n::Motor_Control_Setting_s motor_settings_;
    Motor_General_Def_n::Motor_Working_Type_e mode_;  //电机模式设置
    LK9025_Ctrl_type_e ctrl_type_; // 控制模式
    uint8_t overVoltage_ = 0,overTemperature_ = 0; //过温、过压标志位0正常
    uint32_t dt_ = 0; //发送一次加一

    float *other_speed_feedback_ptr_ = NULL;
    float *other_angle_feedback_ptr_ = NULL; // 其他反馈来源的反馈数据指针
    //使用电机内部FOC控制，因此不需要进行PID计算，只需要设定对应PI参数即可
    float lk_set_ref_;
    uint8_t motor_pi_data[6]; //参数说明：位置环PI，速度环PI，电流环PI
    //Motor_General_Def_n::Motor_Controller_c pid_controller_; 

    BSP_CAN_Part_n::CANInstance_c motor_can_ins_; // can实例
    LKMotorInstance* next_ = NULL; //指向下一个节点的指针

    LKMotorInstance(uint8_t motor_id, Motor_General_Def_n::Motor_Init_Config_s motor_config,LK9025_Ctrl_type_e ctrl_type,float* other_feedback_ptr[2]);
    LKMotorInstance(uint8_t motor_id, Motor_General_Def_n::Motor_Init_Config_s motor_config,LK9025_Ctrl_type_e ctrl_type);
    
    //电机模式设置
    void LK_Motor_Enable();
    void LK_Motor_Disable();
    void LK_Motor_Lock();
    //读取编码器数据
    void LK_Read_encoder();
    //设置电机零点位置（将读取编码器的零偏值发送进行设置）
    void LK_Set_zero();
    //清除错误标志位（过压、过温保护）
    void LK_Clear_error();
    //读取错误标志位命令
    void LK_Read_error();
    //读取电机控制参数
    void LK_Read_PI();
    //写入PID控制参数（掉电保存）
    void LK_Write_PI(uint8_t* PI_parameter);
    //设定数据指针
    void set_other_feedback_ptr(float* other_speed_feedback_ptr,float* other_angle_feedback_ptr);
    //设定目标输出
    void set_target_data(float target_data);
    //设置电机只用于接收
    void set_only_receive();
    
    friend void LKMotorDecode(BSP_CAN_Part_n::CANInstance_c* register_instance);//数据接收处理
    friend void LKMotorControl(); //计算所有电机PID并发送
    friend void LKMotorLost(void);

    private:
    static LKMotorInstance* LK_Motor_Instance_Head;  //表头



} ;

void LKMotorControl();

#endif

