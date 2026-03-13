#ifndef UNITREE_MOTOR_H
#define UNITREE_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#ifdef __cplusplus
}

#include "main.h"
#include "bsp_usart_F4.hpp"
#include "Motor_General_def.hpp"
#include "Alg_PID.hpp"

#define Unitree_MOTOR_MX_CNT 3       // 最多允许3个宇数电机使用多电机指令（因为只存在0，1，2三种ID）,挂载在一条总线上
#define Unitree_MOTOR_USART_MX_CNT 2 // 最多挂载在2条总线上
#define W_SMOOTH_COEF 0.8f
#define T_SMOOTH_COEF 0.85f

namespace A1_motor_n{

typedef union
{
    int32_t L;
    uint8_t u8[4];
    uint16_t u16[2];
    uint32_t u32;
    float F;
} COMData32;

//电机模式设置
typedef enum
{
    TORQUE = 0,      //力矩模式
    POISITION = 1,   //位置模式
    SPEED = 2,       //速度模式
    DAMPING = 3,     //阻尼模式
    ZEROTORQUE = 4,  //零力矩模式
    MULTI = 5,       //混合模式
} Control_Type_e;

 // 发送报文
typedef struct
{
    uint8_t head[2];                // 包头 FE EE
    uint8_t motorID;                //电机编号，可以为 0、1、2、0xBB，0xBB代表向所有电机广播
    uint8_t reserved1;              //预留位，可忽略
    uint8_t mode;                   //电机运行模式，可为 0（停转）、5（开环缓慢转动）、10（闭环伺服控制）
    uint8_t ModifyBit;              //电机内部控制参数修改位，可忽略
    uint8_t ReadBit;                //电机内部控制参数发送位，可忽略
    uint8_t reserved2;              //预留位，可忽略
    COMData32 Modify;               //电机参数修改数据，可忽略
    int16_t T;                      //电机前馈力矩 τff，×256 倍描述
    int16_t W;                      //电机速度命令 ωdes，×128 倍描述
    int32_t Pos;                    //电机位置命令 pdes，×16384/2π 倍描述
    int16_t K_P;                    //电机位置刚度 kp，×2048 倍描述
    int16_t K_W;                    //电机速度刚度 kd，×1024 倍描述
    int16_t LowHzMotorCmdIndex;     //电机低频率控制命令，可忽略
    COMData32 reserved3;            //预留位，可忽略
    COMData32 crc;                  // CRC校验码
} __attribute__((packed)) A1_Tx_Data_t;

//控制数据
typedef struct
{
    float T;            //电机前馈力矩
    float W;            //电机速度命令
    float Pos;          //电机位置命令
    float K_P;          //电机位置刚度
    float K_W;          //电机速度刚度
} A1_Motor_control_t; 

 // 接收
typedef struct
{
    // 定义 接收数据
    uint8_t motor_id; // 电机ID
    uint8_t mode;     // 0:空闲, 5:开环转动, 10:闭环FOC控制
    int32_t Temp;     // 温度
    int32_t MError;   // 错误码，0为正常
    float T;          // 当前实际电机输出力矩
    float W;          // 当前实际电机速度（高速）
    float Pos;        // 当前电机位置
    float LW;         // 当前实际电机速度（低速） //有延迟
    int Acc;          // 电机转子加速度
    // float gyro[3]; // 电机驱动板6轴传感器数据
    // float acc[3];
    COMData32 crc;    // CRC校验码
} A1_Motor_Measure_t;

//由于宇树不通过can进行控制，所以这里不使用motor_General_def中的初始化结构体，而是通过这个结构体进行初始化
typedef struct{
    uint8_t motor_id;               //电机ID
    pid_alg_n::PID_Init_Config_t speed_init_PID;
    pid_alg_n::PID_Init_Config_t angle_init_PID;
    USART_N::usart_init_t        motor_usart_instance;
    float radius;       // 输出轴半径
}A1_motor_init_t;

class A1_motor_c
{
public:
    uint8_t motor_id;
    union
    {
        A1_Tx_Data_t send_data;
        uint8_t send_array[34];
    } send;
    Control_Type_e control_type;
    A1_Motor_Measure_t measure;  
    A1_Motor_control_t control;  //控制数据

    Motor_General_Def_n::Motor_Control_Setting_s motor_settings;
    Motor_General_Def_n::Motor_Working_Type_e stop_flag;      // 启停标志
    float *other_angle_feedback_ptr = NULL; // 其他反馈来源的反馈数据指针
    float *other_speed_feedback_ptr = NULL;
    float *speed_feedforward_ptr = NULL; // 速度前馈数据指针,可以通过此指针设置速度前馈值,或LQR等时作为速度状态变量的输入

    pid_alg_n::pid_alg_c speed_PID;
    pid_alg_n::pid_alg_c angle_PID;
    USART_N::usart_c motor_usart_instance; // 宇数电机串口实例
    float pid_ref_;               // 电机PID测量值和设定值
    float radius_;                // 输出轴半径
    float reduction_radio_;       //减速比

    A1_motor_c(A1_motor_init_t a1_init);
    void A1MotorControl(void);

    void A1MotorDataClear(void);
    void A1MotorStop(void);
    void A1MotorEnable(void);
    void A1MotorSetT(float T);
    void A1MotorSetW(float W);
    void A1MotorSetKP(float KP);
    void A1MotorSetKW(float KW);
    void A1MotorSetControlType(Control_Type_e ControlType);
    void A1MotorSetMulti(float T, float Pos, float KP, float KW);

    

    friend void A1_motor_callback(uint8_t* buf,uint16_t len);

    private:
    static uint8_t usart_idx_;         //串口编号，一个串口一个编号，这里设定最多两个
    static uint8_t idx_[Unitree_MOTOR_USART_MX_CNT];    //每个串口搭载的电机数量
    static A1_motor_c *A1motor_instance_[Unitree_MOTOR_USART_MX_CNT][Unitree_MOTOR_MX_CNT];  //宇树电机数组
    static uint32_t crc32_core(uint32_t *ptr, uint32_t len);
    static float loop_restriction_float(float num, float max_num, float limit_num);
};


}


#endif


#endif
