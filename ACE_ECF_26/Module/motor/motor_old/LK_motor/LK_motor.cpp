/*************************** Dongguan-University of Technology -ACE**************************
 * @file    LK_motor.cpp
 * @author  胡炜
 * @version V2.0
 * @date    2024/10/21
 *          2024/10/22 加入了速度闭环和位置闭环控制
 *          2024/11/7  完善所有功能，已测试
 * @brief
 ******************************************************************************
 * @verbatim
 *  瓴控9025电机驱动，加入了失联检测
 *  使用方法：
 *      构建初始化结构体，创建对象，设置好控制模式
 *      设置设定值，调用统一控制发送函数
 *  demo：
 *       Motor_General_Def_n::Motor_Init_Config_s motor_init = {
        .controller_setting_init_config = {
            .outer_loop_type = OPEN_LOOP, //电机内部FOC控制，不需要额外设置PID
            .close_loop_type = OPEN_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .feedforward_flag = FEEDFORWARD_NONE,
        },
        .motor_type = LK9025,
        .can_init_config = {
            .can_handle = &hcan2,
            0,0,0,NULL
        },
    };
    //创建实例
    LKMotorInstance* motor = new LKMotorInstance(1,motor_init,Speed); //设置好电机模式，不希望在程序运行过程中修改
    float set_s = 0.0f;
    //如果需要，设置PI参数（掉电保存）
    //motor->LK_Write_PI(anglePI,speedPI,currentPI);
    while(1)
    {
        //设置设定值
        motor->set_target_data(set_s);
        //发送控制信息
        LKMotorControl();
        text_task_dwt->ECF_DWT_Delay_ms(2);
    }
    
 * @attention
 * 电机速度单位为度每秒，角度单位也是角度值
 * 不希望在程序运行过程中修改电机控制模式，因为没有设置每次模式切换设置电机零点，可能会导致使用位置模式时电机一直转动找到初始零点（而且频繁设置零点会有损电机寿命）
 * @version           time
 * v1.1   基础版本（未连接电机测试）
 * v2.0   速度，位置，力矩模式均已经测试
 * v2.1   新增只收模式 
 ************************** Dongguan-University of Technology -ACE***************************/
#include "LK_motor.hpp"

user_maths_c LK_math_;

using namespace Motor_General_Def_n;

//数据解析函数声明
static void LKMotorDecode(BSP_CAN_Part_n::CANInstance_c* register_instance);
//电机掉线回调函数声明
static void LKMotorLost(void);

//LK电机失联检测
Safe_task_c lost_detection_("LK_Motor_Safe_task",10,LKMotorLost);

LKMotorInstance* LKMotorInstance::LK_Motor_Instance_Head = nullptr;

/**
 * @brief 电机初始化构造函数
 * @param motor_id 电机id号，可设置为0~32
 * @param motor_config 电机初始化结构体，包括can，三环PID，标识设置等（can发送接收ID不需要设置，构造函数会根据ID计算）
 * @param other_feedback_ptr [0]其他速度反馈来源指针 [1]其他角度反馈来源指针
 * @param feedforward_ptr [0]速度前馈 [1]电流前馈指针
 */
LKMotorInstance::LKMotorInstance(uint8_t motor_id, Motor_General_Def_n::Motor_Init_Config_s motor_config,LK9025_Ctrl_type_e ctrl_type,float* other_feedback_ptr[2]):
                LKMotorInstance(motor_id, motor_config,ctrl_type)
            {
                set_other_feedback_ptr(other_feedback_ptr[0],other_feedback_ptr[1]);
            }

/**
 * @brief 电机初始化构造函数
 * @param motor_id 电机id号，可设置为0~32
 * @param motor_config 电机初始化结构体，包括can，三环PID，标识设置等（can发送接收ID不需要设置，构造函数会根据ID计算）
 */
LKMotorInstance::LKMotorInstance(uint8_t motor_id, Motor_General_Def_n::Motor_Init_Config_s motor_config,LK9025_Ctrl_type_e ctrl_type):
            motor_id_(motor_id),
            ctrl_type_(ctrl_type),
            motor_can_ins_(
                motor_config.can_init_config.can_handle,

                motor_id + 0x140,  //txID
                motor_id + 0x140,  //rxID
                motor_config.can_init_config.SAND_IDE = CAN_ID_STD,
                LKMotorDecode),
            motor_settings_(motor_config.controller_setting_init_config)
{
    memset(&this->measure_, 0, sizeof(this->measure_));
    this->next_ = nullptr;
    LK_Motor_Enable();

    //链表尾插法
    if(LK_Motor_Instance_Head == nullptr)
    {
        LK_Motor_Instance_Head = this;
        return;
    } 
    else
    {
        LKMotorInstance* last_ptr = LK_Motor_Instance_Head;
        while(last_ptr->next_ != nullptr)     last_ptr = last_ptr->next_;
        last_ptr->next_ = this;
    }
    //读取电机PI参数
    LK_Read_PI();
    //读取初始编码器参数并设置原点
    LK_Read_encoder();
    LK_Set_zero();
}


//电机使能设置
void LKMotorInstance::LK_Motor_Enable()
{
    //发送值清空
    memset(this->motor_can_ins_.tx_buff, 0, 8 * sizeof(uint8_t));
    this->motor_can_ins_.tx_buff[0] = Motor_Stop_To_Run;
    //电机发送启动命令
    this->motor_can_ins_.ECF_Transmit(0.01);
    this->mode_ = MOTOR_ENALBED;
    this->LK_Clear_error();
}

//电机失能
void LKMotorInstance::LK_Motor_Disable()
{
    //测量值清空
    memset(&this->measure_, 0, sizeof(this->measure_));
    //发送值清空
    memset(this->motor_can_ins_.tx_buff, 0, 8 * sizeof(uint8_t));
    this->motor_can_ins_.tx_buff[0] = Motor_Stop;
    
    //电机发送停止命令
    this->motor_can_ins_.ECF_Transmit(0.01);
    this->measure_.first_flag = 1;
    this->mode_ = MOTOR_STOP;
}

//电机锁定
void LKMotorInstance::LK_Motor_Lock()
{
    //发送值清空
    memset(this->motor_can_ins_.tx_buff, 0, 8 * sizeof(uint8_t));
    this->motor_can_ins_.tx_buff[0] = Motor_Close;
    //电机发送启动命令
    this->motor_can_ins_.ECF_Transmit(0.01);
    this->mode_ = MOTOR_Lock;
}

//读取电机编码器值
uint16_t ecd_pos[3] = {0}; //编码器位置、编码器原始位置、编码器零偏(因为只在初始化使用一次，所以所有LK公用)
void LKMotorInstance::LK_Read_encoder()
{
    //发送值清空
    memset(this->motor_can_ins_.tx_buff, 0, 8 * sizeof(uint8_t));
    this->motor_can_ins_.tx_buff[0] = Read_Encoder;
    //电机发送启动命令
    this->motor_can_ins_.ECF_Transmit(0.01);
}

//设置电机零点位置（将读取编码器的零偏值发送进行设置）
void LKMotorInstance::LK_Set_zero()
{
    //发送值清空
    memset(this->motor_can_ins_.tx_buff, 0, 8 * sizeof(uint8_t));
    this->motor_can_ins_.tx_buff[0] = Write_Encoder_Offset_To_ROM;
    this->motor_can_ins_.tx_buff[6] = ecd_pos[2];
    this->motor_can_ins_.tx_buff[7] = ecd_pos[2]>>8;
    //电机发送启动命令
    this->motor_can_ins_.ECF_Transmit(0.01);
}

//清除错误标志位
void LKMotorInstance::LK_Clear_error()
{
    //发送值清空
    memset(this->motor_can_ins_.tx_buff, 0, 8 * sizeof(uint8_t));
    this->motor_can_ins_.tx_buff[0] = Delect_Error_State;
    //电机发送启动命令
    this->motor_can_ins_.ECF_Transmit(0.01);
}

//读取错误标志位命令
void LKMotorInstance::LK_Read_error()
{
    //发送值清空
    memset(this->motor_can_ins_.tx_buff, 0, 8 * sizeof(uint8_t));
    this->motor_can_ins_.tx_buff[0] = Read_State_And_Error_1;
    //电机发送启动命令
    this->motor_can_ins_.ECF_Transmit(0.01);
}

//读取电机控制参数
void LKMotorInstance::LK_Read_PI()
{
    //发送值清空
    memset(this->motor_can_ins_.tx_buff, 0, 8 * sizeof(uint8_t));
    this->motor_can_ins_.tx_buff[0] = Read_PID;  //读取PID参数
    
    this->motor_can_ins_.ECF_Transmit(0.01);
}

//写入电机控制参数(掉电保存)
void LKMotorInstance::LK_Write_PI(uint8_t* PI_parameter)
{
    //发送值清空
    memset(this->motor_can_ins_.tx_buff, 0, 8 * sizeof(uint8_t));
    this->motor_can_ins_.tx_buff[0] = PID_To_ROM;  //写入PID参数
    this->motor_can_ins_.tx_buff[2] = PI_parameter[0];
    this->motor_can_ins_.tx_buff[3] = PI_parameter[1];
    this->motor_can_ins_.tx_buff[4] = PI_parameter[2];
    this->motor_can_ins_.tx_buff[5] = PI_parameter[3];
    this->motor_can_ins_.tx_buff[6] = PI_parameter[4];
    this->motor_can_ins_.tx_buff[7] = PI_parameter[5];
    
    memcpy(&this->motor_pi_data,PI_parameter,sizeof(uint8_t)*8);

    this->motor_can_ins_.ECF_Transmit(0.01);
}

/**
 * @brief 电机其他反馈数据指针设置
 * @param[in] other_sprrd_feedback_ptr 其他来源速度数据反馈指针
 * @param[in] other_angle_feedback_ptr 其他来源角度数据反馈指针
 */
void LKMotorInstance::set_other_feedback_ptr(float* other_speed_feedback_ptr,float* other_angle_feedback_ptr)
{
    if(other_speed_feedback_ptr == NULL)
    {
        this->motor_settings_.speed_feedback_source = MOTOR_FEED;
        this->other_speed_feedback_ptr_ = NULL;
    }
    else
    {
        this->motor_settings_.speed_feedback_source = OTHER_FEED;
        this->other_speed_feedback_ptr_ = other_speed_feedback_ptr;
    }

    if(other_angle_feedback_ptr == NULL)
    {
        this->motor_settings_.angle_feedback_source = MOTOR_FEED;
        this->other_angle_feedback_ptr_ = NULL;
    }
    else
    {
        this->motor_settings_.angle_feedback_source = OTHER_FEED;
        this->other_angle_feedback_ptr_ = other_angle_feedback_ptr;
    }           
}


/**
 * @brief 电机控制目标值设置
 * @param[in] target_data 目标值
 */
void LKMotorInstance::set_target_data(float target_data)
{
    this->lk_set_ref_ = target_data;
}

//设置电机只用于接收
void LKMotorInstance::set_only_receive()
{
    this->mode_ = MOTOR_ONLY_RECEIVE;
}


/**
 * @brief 电机反馈数据解析函数
 */
static void LKMotorDecode(BSP_CAN_Part_n::CANInstance_c* register_instance)
{
    uint8_t *rxbuff = register_instance->rx_buff;      //反馈数据
    LKMotorInstance *motor;
    LKMotorInstance* lk_ptr = NULL;
    if(LKMotorInstance::LK_Motor_Instance_Head == NULL)
        return;
    else
        lk_ptr = LKMotorInstance::LK_Motor_Instance_Head;
    lost_detection_.Online(); //喂狗
    //链表轮询
    while(lk_ptr != NULL)
    {
        //外层判断接收的是那个电机的反馈数据
        if(lk_ptr->motor_can_ins_.rx_id_ == register_instance->rx_id_)
        {
            //内层判断接收数据的格式
            motor = lk_ptr;
            LKMotor_Measure_t* measure = &motor->measure_;
            switch(rxbuff[0])
            {
            case Read_PID:
            {
                //获取PI参数
                motor->motor_pi_data[0] = rxbuff[2];
                motor->motor_pi_data[1] = rxbuff[3];
                motor->motor_pi_data[2] = rxbuff[4];
                motor->motor_pi_data[3] = rxbuff[5];
                motor->motor_pi_data[4] = rxbuff[6];
                motor->motor_pi_data[5] = rxbuff[7];
                break;
            }
            case 0xA1:
            case 0xA2:
            case 0xA3:
            {//接收数据处理
                measure->last_ecd = measure->feedback_ecd;
                measure->feedback_ecd = (uint16_t)((rxbuff[7] << 8) | rxbuff[6]);
                //判断是否为首次进入
                if (motor->measure_.first_flag == 1)
                {
                    measure->last_ecd = measure->feedback_ecd;
                    motor->measure_.first_flag = 0;
                    motor->LK_Motor_Enable();//能接收到反馈再使能
                }

                measure->feedback_speed = (int16_t)rxbuff[5] << 8 | rxbuff[4];
                measure->feedback_real_current = (int16_t)rxbuff[3]<<8 | rxbuff[2];
                measure->feedback_temperature = rxbuff[1];

                measure->angle_single_round = ECD_ANGLE_COEF_LK * measure->feedback_ecd;
                if (measure->feedback_ecd - measure->last_ecd > 16384/2)
                    measure->total_round--;
                else if (measure->feedback_ecd - measure->last_ecd < -16384/2)
                    measure->total_round++;
                measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
                break;
            }
            case Read_Encoder://编码器参数
            {
                ecd_pos[0] = (uint16_t)rxbuff[3]<<8 | rxbuff[2];
                ecd_pos[1] = (uint16_t)rxbuff[5]<<8 | rxbuff[4];
                ecd_pos[2] = (uint16_t)rxbuff[7]<<8 | rxbuff[6];
                break;
            }
            case Read_State_And_Error_1:
            {
                motor->overVoltage_ = rxbuff[7] & 0x01;
                motor->overTemperature_ = rxbuff[7] & 0x08;
            }
            default: break;
            }
        }
        lk_ptr = lk_ptr->next_;
    }
}

/**
 * @brief 电机掉线回调函数
 */
static void LKMotorLost(void)
{
    LKMotorInstance* lk_ptr = NULL;
    if(LKMotorInstance::LK_Motor_Instance_Head == NULL)
        return;
    else
        lk_ptr = LKMotorInstance::LK_Motor_Instance_Head;
    //链表轮询
    while(lk_ptr != NULL)
    {
        lk_ptr->LK_Motor_Disable();
        lk_ptr = lk_ptr->next_;
    }
}


/**
 * @brief 电机转矩闭环控制命令
 * @note 计算所有电机PID并发送
 */
void LKMotorControl()
{
    int16_t set16 = 0;
    int32_t set32 = 0;
    float pid_ref = 0.0f;
    LKMotorInstance *motor = NULL;
    BSP_CAN_Part_n::CANInstance_c *sender_instance = NULL;
    LKMotorInstance* lk_ptr = NULL;
    if(LKMotorInstance::LK_Motor_Instance_Head == NULL)
        return;
    else
        lk_ptr = LKMotorInstance::LK_Motor_Instance_Head;
    //链表轮询
    while(lk_ptr != NULL)
    {
        motor = lk_ptr;
        sender_instance = &motor->motor_can_ins_;
        pid_ref = motor->lk_set_ref_; //获取设定值用于计算

        // //计算角度环PID
        // if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
        // {
        //     //实际值来源设定
        //     if (setting->angle_feedback_source == OTHER_FEED)
        //         motor->pid_controller_.angle_PID.ECF_PID_ChangeActValSource(motor->other_angle_feedback_ptr_);
        //     else
        //         motor->pid_controller_.angle_PID.ECF_PID_ChangeActValSource(&motor->measure_.angle_single_round);
        //     // 更新pid_ref进入下一个环
        //     pid_ref = motor->pid_controller_.angle_PID.ECF_PID_Calculate(pid_ref); 
        // }

        // //计算速度环PID
        // if ((setting->close_loop_type & SPEED_LOOP) && setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))
        // {
        //     //实际值来源设定
        //     if (setting->speed_feedback_source == OTHER_FEED)
        //         motor->pid_controller_.speed_PID.ECF_PID_ChangeActValSource(motor->other_speed_feedback_ptr_);
        //     else
        //         motor->pid_controller_.speed_PID.ECF_PID_ChangeActValSource(&motor->measure_.feedback_speed);
        //     // 更新pid_ref进入下一个环
        //     pid_ref = motor->pid_controller_.speed_PID.ECF_PID_Calculate(pid_ref);
        // }

        // //计算电流环PID
        // if (setting->close_loop_type & CURRENT_LOOP)
        // {
        //     motor->pid_controller_.current_PID.ECF_PID_ChangeActValSource(&motor->measure_.real_current);
        //     pid_ref = motor->pid_controller_.current_PID.ECF_PID_Calculate(pid_ref);
        // }

        if(motor->mode_ == MOTOR_ONLY_RECEIVE) //只接收模式，不发送信号
            continue;

        //输出是否反向
        if (motor->motor_settings_.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;


        memset(sender_instance->tx_buff, 0, 8 * sizeof(uint8_t)); //先清空数组
        switch(motor->ctrl_type_)
        {
            case Open:
            case Speed: //速度模式，此模式下转速非常慢
                //设置设定值,单位：0.01度每秒
                set32 = (int32_t)(pid_ref);
                sender_instance->tx_buff[0] = Speed_Closed_Loop_Control;
                sender_instance->tx_buff[4] = set32;
                sender_instance->tx_buff[5] = set32 >> 8;
                sender_instance->tx_buff[6] = set32 >> 16;
                sender_instance->tx_buff[7] = set32 >> 24;

                break;

            case Position:
                //设置设定值
                set16 = (int32_t)(pid_ref);
                //对应实际位置为0.01 degree/LSB 36000表示360度 旋转方向为最短路径
                //位置模式第一次切换时会一直转动，这是在找位置零点
                sender_instance->tx_buff[0] = Position_Closed_Loop_Control_1;
                user_value_limit(set16,-35999,35999);
                set16 = (set16 < 0) ? 35999 + set16 : set16;//角度限制
                
                sender_instance->tx_buff[4] = set16;
                sender_instance->tx_buff[5] = set16 >> 8;
                break;

            case Torque://一般使用这个
                //设置设定值
                set16 = (int16_t)(pid_ref);
                sender_instance->tx_buff[0] = Torque_Closed_Loop_Control;  //转矩闭环控制
                user_value_limit(set16,-2000,2000);//转矩电流限制，对应电流±32A
                sender_instance->tx_buff[4] = set16;
                sender_instance->tx_buff[5] = set16 >> 8;
                
            default: break;
        }
        
        if (motor->mode_ == MOTOR_STOP)
        { // 若该电机处于停止状态,直接将发送buff置零
            memcpy(&sender_instance->tx_buff[4], 0, sizeof(int32_t));
        }
        sender_instance->ECF_Transmit(0.001);
        motor->dt_++;
        //过温过压检测
        if(motor->dt_ % 20 == 0)
        {
            motor->LK_Read_error();
        }
        if(motor->dt_ > 4000000000)
            motor->dt_ = 0;
        lk_ptr = lk_ptr->next_;
    }
}




