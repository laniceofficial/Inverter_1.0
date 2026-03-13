#ifndef __UNITREE_MOTOR_GO_M8010_6_H
#define __UNITREE_MOTOR_GO_M8010_6_H


#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"


#ifdef __cplusplus
}

#ifndef PI
#define PI 3.141592653589793
#endif

#define PACKET_HEAD  ((0xFD<<8)|0xEE)

#define REDUCTION_RATION   6.33f  //减速比
#define TORQUE_LIMIT 127.99f //电机转矩限制N*m
#define SPEED_LIMIT 804.0f   //电机转速限制rad/s
#define POSITION_LIMIT 411774  //期望输出位置rad(65535圈)
#define UNITREE_GO_ENCODER 32768  //go电机一圈编码值
#define KP_LIMIT 25.599f //比例系数限制
#define KD_LIMIT 25.599f //微分系数限制

//发送的数据需要进行以下处理：
#define FLOAT_TO_SHORT(feedbackTorque) ((int16_t)((feedbackTorque) * 256.0f))  //32位float转16位，用于向电机发送数据
#define SHORT_TO_FLOAT(shortValue)     ((float)(shortValue) / 256.0f)   //再次转回32位浮点数，用于解析，会损失一点精度

#define SPEED_TO_SENDDATA(x)        ((int16_t)(((x)/(2*PI))*256.0f))  //期望电机速度转2字节发送
#define RECIVEDATA_TO_SPEED(x)      (((float)(x)/256.0f)*2*PI)    //2字节接收解析速度

#define ANGLE_TO_RAD(x)             ((x)/180.0f*PI)  //角度转弧度
#define RAD_TO_ANGLE(x)             ((x)*180.0f/PI)  //弧度转角度
#define ANGLE_TO_SENDDATA(x)        (ANGLE_TO_RAD(x)/(2*PI)*UNITREE_GO_ENCODER)  //角度转发送数据
#define RECIVEDDATA_TO_ANGLE(x)     (RAD_TO_ANGLE((x)/UNITREE_GO_ENCODER*2*PI))  //接收数据解析角度

#define GET_K_POS(x) ((x)*1280)       //计算电机刚度系数/ 位置误差比例系数
#define GET_K_SPD(x) ((x)*1280)       //计算电机阻尼系数/ 速度误差比例系数


namespace unitree_motor_n{

    typedef enum {
        A1,             // 4.8M baudrate
        B1,             // 6.0M baudrate
        GO_M8010_6
    }unitree_motor_type_e; //电机型号

    typedef enum{
        CLOCK = 0,                //锁定模式
        FOC_LOOP = 1,             //FOC闭环
        ENCODER_CALIBRATION = 2,  //编码器校准
    }unitree_mode_e;  //电机模式

    typedef enum{
        NORMAL = 0,                //锁定模式
        OVER_HOT = 1,              //过热
        OVER_CURRENT = 2,          //过流
        OVER_PRESSURE = 3,         //过压
        ENCODER_ERROR = 4,         //编码器故障
        BUS_UNDERVOLTAGE = 5,      //母线欠压
        WINDING_OVERHOT = 6,       //绕组过热
    }unitree_err_e;  //电机错误标识

    typedef struct{
        uint8_t  ID             : 4  ; //目标电机ID
        uint8_t  STATUS         : 3  ; //电机工作模式
        uint8_t  SAVE           : 1  ; //保留
    }messag_mode_t;  //电机模式信息

    typedef struct{
        uint16_t set_torque               ;//期望电机转矩
        uint16_t set_speed                ;//期望电机转速
        uint32_t set_location             ;//期望电机输出位置
        uint16_t stiffness_pos_err_coeff  ;//电机刚度系数/ 位置误差比例系数
        uint16_t damping_speed_err_coeff  ;//电机阻尼系数/ 速度误差比例系数
    }send_data_t;  //控制参数

    typedef struct{
        uint16_t real_torque     : 16 ;//实际电机转矩
        uint16_t real_speed      : 16 ;//实际电机转速
        uint32_t real_location   : 32 ;//实际电机输出位置
        uint8_t  temp            : 8  ;//电机温度
        uint8_t  merror          : 3  ;//电机错误标识
        uint16_t force           : 12 ;//足端力
        uint8_t  save            : 1  ;//保留
    }recive_data_t;  //反馈数据

    typedef struct {
        uint16_t        HEAD;           //数据包头0xFD 0xEE
        messag_mode_t   MODE;           //电机模式信息
        send_data_t     SEND_DATA;      //控制参数
        uint16_t        CRC16;           //16位CRC校验
    }unitree_send_t; //宇树电机发送数据结构体

    typedef struct {
        uint16_t         HEAD      ; //数据包头0xFD 0xEE
        messag_mode_t    MODE      ; //电机模式信息
        recive_data_t    SEND_DATA ; //反馈数据
        uint16_t         CRC16     ; //16位CRC校验
    }unitree_recive_t; //宇树电机接收数据结构体



    class unitree_motor_c{
        public:
            unitree_motor_type_e motorType;  //电机型号GO_M8010_6
            int hex_len;            //发送的数据长度       
            uint8_t id;      //电机ID 0~14          
            unitree_mode_e mode;    //电机模式

            unitree_motor_c(UART_HandleTypeDef *husart);
            ~unitree_motor_c(){}

            void send_motor_control_message(uint8_t id,uint8_t mode,float tau,float dq,float q,float kp,float kd);
            
            

            unitree_send_t Unitree_Tx_data;   //发送结构体
            unitree_recive_t Unitree_Rx_data; //接收结构体

        private:
            float tau;              //前馈力矩           
            float dq;               //期望角速度rad/s
            float q;                //期望角度rad
            float kp;               //刚度比例系数
            float kd;               //阻尼系数

            UART_HandleTypeDef *husart;
    };
}



#endif
#endif
