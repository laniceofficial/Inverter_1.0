/*************************** Dongguan-University of Technology -ACE**************************
 * @file    dji_motor.cpp
 * @author  study-sheep
 * @version V1.0
 * @date    2024/10/18
 * @brief   电机模块文件
 ******************************************************************************
 * @verbatim
 * 支持PID控制 + 温度保护
 * 详细使用看demo.h文件，太长了，看起来太难受了
 * @attention
 * 使用电机自锁函数(DJIMotorSelfLock)时，需要在电机初始化的时候把位置环的PID参数设置了
 * 
 * @version           time
 * v1.0   基础版本（fdcan）     2024-10-18    已测试
 * v2.0   重构+加上了绝对编码    2024-11-20
 * v2.1   加上了外部可以通过在初始化的时候设置反馈的类型（枚举类型：Feedback_Source_e）
 *        电机以外的反馈需要调用以下两个函数 
 *                             Set_ANGLE_PID_other_feedback(float *other_ptr);
 *                             Set_SPEED_PID_other_feedback(float *other_ptr);
 * v3.0(胡炜)    区分了绝对角度和相对角度，修改了GM6020编码值设置错误的问题，修复了解析数据不正确的问题
 *              修复了默认反馈指针设置错误的问题（我建议是自己调用函数设置）
 *              数据Mesure结构体添加了相对角度，零偏角    2024-11-24 已测试
 * v3.1   添加了对象可调用函数获取PID的不同环的输出，修复初始化后电机的累计编码值不是0的bug
 * v3.2   加上了电机失联处理       √ 已测试
 * todo   在有6020电机升级固件时再测试它的电流控制     
 ************************** Dongguan-University of Technology -ACE***************************/

#include "dji_motor_H7.hpp"
#include "bsp_dwt.hpp"
#include "user_maths.hpp"
// 超电
// #include "power_limit.h"
// dwt
static BSP_DWT_n::BSP_DWT_c* dwt_DJIMotor = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance();
//自定义数学库
user_maths_c dji_maths;
/***函数声明***/
static void DecodeDJIMotor(BSP_CAN_Part_n::FDCANInstance_c *can_instance);

namespace DJI_Motor_n
{
    DJI_Motor_Instance* DJI_Motor_Instance::dji_motor_instance_p[DJI_MOTOR_CNT] = {nullptr}; // 会在control任务中遍历该指针数组进行pid计算
    uint8_t DJI_Motor_Instance::dji_motor_idx = 0;                                           // dji_motor_idx,是该文件的全局电机索引,在注册时使用

    /**
     * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_instance专门负责发送
     *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
     *
     * @note  因为只用于发送,所以不需要在bsp_fdcan中注册
     *
     * C610(m2006)/C620(m3508):0x1ff,0x200;
     * GM6020:0x1ff,0x2ff
     * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
     * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
     * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
     * can3: [6]:0x1FF,[7]:0x200,[8]:0x2FF
     */
    // 下面的这几个对象只负责发送，不负责接收回调函数，不会占用CAN实例数组
    BSP_CAN_Part_n::FDCANInstance_c sender_assignment[9]=
    {
        BSP_CAN_Part_n::FDCANInstance_c(0x1ff,0x000),
        BSP_CAN_Part_n::FDCANInstance_c(0x200,0x000),
        BSP_CAN_Part_n::FDCANInstance_c(0x2ff,0x000),
        BSP_CAN_Part_n::FDCANInstance_c(0x1ff,0x000),
        BSP_CAN_Part_n::FDCANInstance_c(0x200,0x000),
        BSP_CAN_Part_n::FDCANInstance_c(0x2ff,0x000),
        BSP_CAN_Part_n::FDCANInstance_c(0x1ff,0x000),
        BSP_CAN_Part_n::FDCANInstance_c(0x200,0x000),
        BSP_CAN_Part_n::FDCANInstance_c(0x2ff,0x000) 
    };
    // 大疆电机失联处理函数，失联处理：电机失能，置标志位
    void DJI_Motor_Instance::DJIMotorLost()
    {
        
        if(this->motor_fdcan_instance.tx_id_ == 2)
        {
            this->is_lost_dji = false;
        }
        else{
        this->is_lost_dji = true;
        this->DJIMotorStop(); // 电机失能
        }
    }
    /**
     * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
     *        flag的初始化在 MotorSenderGrouping()中进行
     */
    uint8_t DJI_Motor_Instance::sender_enable_flag[9] = {0};
    DJI_Motor_Instance::DJI_Motor_Instance(Motor_General_Def_n::Motor_Init_Config_s config):
                        motor_fdcan_instance(config.fdcan_init_config)
    {
        lost_detection_dji = new Safe_task_c("LK_Motor_Safe_task", 20, [this] () {
            DJIMotorLost();
        }, nullptr); // 20ms接不到数据就进错误回调
        if(!dji_motor_idx)
        {
            sender_assignment[0].FDCAN_Motor_Init(&hfdcan1,FDCAN_STANDARD_ID);
            sender_assignment[1].FDCAN_Motor_Init(&hfdcan1,FDCAN_STANDARD_ID);
            sender_assignment[2].FDCAN_Motor_Init(&hfdcan1,FDCAN_STANDARD_ID);
            sender_assignment[3].FDCAN_Motor_Init(&hfdcan2,FDCAN_STANDARD_ID);
            sender_assignment[4].FDCAN_Motor_Init(&hfdcan2,FDCAN_STANDARD_ID);
            sender_assignment[5].FDCAN_Motor_Init(&hfdcan2,FDCAN_STANDARD_ID);
            sender_assignment[6].FDCAN_Motor_Init(&hfdcan3,FDCAN_STANDARD_ID);
            sender_assignment[7].FDCAN_Motor_Init(&hfdcan3,FDCAN_STANDARD_ID);
            sender_assignment[8].FDCAN_Motor_Init(&hfdcan3,FDCAN_STANDARD_ID);
        }
        // motor basic setting 电机基本设置
        this->motor_type = config.motor_type;                         // 6020 or 2006 or 3508
        this->motor_settings = config.controller_setting_init_config; // 正反转,闭环类型等
        this->motor_controller.angle_PID.pid_init(config.controller_param_init_config.angle_PID);
        this->motor_controller.speed_PID.pid_init(config.controller_param_init_config.speed_PID);
        this->motor_controller.current_PID.pid_init(config.controller_param_init_config.current_PID);
        this->motor_fdcan_instance.tx_id_ = config.fdcan_init_config.tx_id;
        this->motor_fdcan_instance.ECF_SetRxCallBack(DecodeDJIMotor);
        this->MotorSenderGrouping();
        dji_motor_instance_p[dji_motor_idx++] = this;
        switch(this->motor_type)
        {
            case Motor_General_Def_n::M3508:
            {
                this->MotorMeasure.lap_encoder =  8192;
                this->MotorMeasure.gear_Ratio = 19;
                break;
            }
            case Motor_General_Def_n::M2006:
            {
                this->MotorMeasure.lap_encoder =  8192;
                this->MotorMeasure.gear_Ratio = 36;
                break;
            }
            case Motor_General_Def_n::GM6020:
            {
                this->MotorMeasure.lap_encoder =  8192;
                this->MotorMeasure.gear_Ratio = 1;
                break;
            }
        
            default:
                break;
        }
        this->MotorMeasure.MeasureClear();     
        this->MotorMeasure.radius = config.radius;
        this->MotorMeasure.ecd2length = config.ecd2length;    
        this->MotorMeasure.measure.zero_offset = config.zero_offset;
        this->DJIMotorStop();
    }

    /*** 
     * @brief 设定闭环的参考值
     * @param ref 用作闭环计算大的参考值
     * @note ref 的值具体设置为什么看你在初始化设置的闭环环路, 以及最外环的数据来源 ActualValueSource
    */
    void DJI_Motor_n::DJI_Motor_Instance::DJIMotorSetRef(float ref)
    {
        if(this->stop_flag == Motor_General_Def_n::MOTOR_STOP)
        {
            this->set_length = 0;
            this->set_speed  = 0;
            return;// stop直接把设定值置0
        }
        // 位置环
        if ((this->motor_settings.close_loop_type & Motor_General_Def_n::ANGLE_LOOP) && 
             this->motor_settings.outer_loop_type == Motor_General_Def_n::ANGLE_LOOP)
        {   
            this->set_length = ref;
        }
        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((this->motor_settings.close_loop_type & Motor_General_Def_n::SPEED_LOOP) && 
            (this->motor_settings.outer_loop_type &  Motor_General_Def_n::SPEED_LOOP))
        {
            this->set_speed = ref;
        }
        this->motor_controller.RefValChange(ref);
    }

    // 电机分组的函数
    void DJI_Motor_n::DJI_Motor_Instance::MotorSenderGrouping()
    {
        uint8_t motor_id = this->motor_fdcan_instance.tx_id_ - 1; // 下标从零开始,先减一方便赋值
        uint8_t motor_send_num;
        uint8_t motor_grouping;

        switch (this->motor_type)
        {
            case Motor_General_Def_n::M3508:
            case Motor_General_Def_n::M2006:
            {
                if (motor_id < 4) // 根据ID分组
                {
                    motor_send_num = motor_id;
                    motor_grouping = this->motor_fdcan_instance.ECF_GetFDCanhandle() == &hfdcan1 ? 1 : 
                                    (this->motor_fdcan_instance.ECF_GetFDCanhandle() == &hfdcan2) ? 4 : 7;
                }
                else
                {
                    motor_send_num = motor_id - 4;
                    motor_grouping = this->motor_fdcan_instance.ECF_GetFDCanhandle() == &hfdcan1 ? 0 : 
                                    (this->motor_fdcan_instance.ECF_GetFDCanhandle() == &hfdcan2) ? 3 : 6;
                }
                // 计算接收id并设置分组发送id
                this->motor_fdcan_instance.rx_id_ = 0x200 + motor_id + 1;   // 把ID+1,进行分组设置
                sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
                this->message_num  = motor_send_num;
                this->sender_group = motor_grouping;
                // 检查是否发生id冲突
                for (size_t i = 0; i < dji_motor_idx; ++i)
                {
                    if ((dji_motor_instance_p[i]->motor_fdcan_instance.ECF_GetFDCanhandle() == this->motor_fdcan_instance.ECF_GetFDCanhandle()) &&
                        (dji_motor_instance_p[i]->motor_fdcan_instance.rx_id_ == this->motor_fdcan_instance.rx_id_))
                    {                                                                                                                    
                        while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突
                        {}
                    }
                }
                break;
            }
            case Motor_General_Def_n::GM6020:
            {
                if (motor_id < 4)
                {
                    motor_send_num = motor_id;
                    motor_grouping =  this->motor_fdcan_instance.ECF_GetFDCanhandle() == &hfdcan1 ? 0 : 
                                    (this->motor_fdcan_instance.ECF_GetFDCanhandle() == &hfdcan2) ? 3 : 6;  
                }
                else
                {
                    motor_send_num = motor_id - 4;
                    motor_grouping =  this->motor_fdcan_instance.ECF_GetFDCanhandle() == &hfdcan1 ? 2 : 
                                    (this->motor_fdcan_instance.ECF_GetFDCanhandle() == &hfdcan2) ? 5 : 8;
                }
                this->motor_fdcan_instance.rx_id_ = 0x204 + motor_id + 1;   // 把ID+1,进行分组设置
                sender_enable_flag[motor_grouping] = 1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
                this->message_num = motor_send_num;
                this->sender_group = motor_grouping;
                for (size_t i = 0; i < dji_motor_idx; ++i)
                {
                    if (dji_motor_instance_p[i]->motor_fdcan_instance.ECF_GetFDCanhandle() == this->motor_fdcan_instance.ECF_GetFDCanhandle() &&
                        dji_motor_instance_p[i]->motor_fdcan_instance.rx_id_ == this->motor_fdcan_instance.rx_id_)
                    {
                        while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突
                        {}
                    }
                }
                break;
            }
        
            default:
                    while (1) // 其他类型电机不适用于该初始化
                    {}
                break;
        }
    }

    /**
     * @brief DJI 电机正反向转换函数
     * @note 正转变成反转 反转变成正转
    */
    void DJI_Motor_n::DJI_Motor_Instance::DJIMotorDirectionTurn(void)
    {
        if(this->motor_settings.motor_reverse_flag == Motor_General_Def_n::MOTOR_DIRECTION_NORMAL)
        {
            this->motor_settings.motor_reverse_flag = Motor_General_Def_n::MOTOR_DIRECTION_REVERSE;
        }
        else
        {
            this->motor_settings.motor_reverse_flag = Motor_General_Def_n::MOTOR_DIRECTION_NORMAL;
        }

    }
    
    /**
     * @brief DJI 电机使能
     * @note 默认为使能
    */
    void DJI_Motor_n::DJI_Motor_Instance::DJIMotorEnable(void)
    {
        this->stop_flag = Motor_General_Def_n::MOTOR_ENALBED;
    }

    /**
     * @brief DJI 电机停止
     * @note 具体实现为在发送时对应数据发0
    */
    void DJI_Motor_n::DJI_Motor_Instance::DJIMotorStop(void)
    {
        this->stop_flag = Motor_General_Def_n::MOTOR_STOP;
    }

    /**
     * @brief Dji 电机自锁
     * @note  添加位置环并把它设置成最外层的闭环
    */
    void DJI_Motor_n::DJI_Motor_Instance::DJIMotorSelfLock()
    {
        this->stop_flag = Motor_General_Def_n::MOTOR_SELF_LOCK;
        this->DJIMotorOuterLoop(Motor_General_Def_n::ANGLE_LOOP);
        this->DJIMotorSetRef(0);
    }
    /**
     * @brief Dji 电机解除自锁
     * @note  恢复速度环为最外层的闭环,并删除位置环
    */
    void DJI_Motor_Instance::DJIMotorClearSelfLock(void)
    {
        this->stop_flag = Motor_General_Def_n::MOTOR_ENALBED;
        this->motor_settings.outer_loop_type = Motor_General_Def_n::SPEED_LOOP;
        this->motor_settings.close_loop_type = (Motor_General_Def_n::Closeloop_Type_e)(this->motor_settings.close_loop_type & 0b1011);
    }

    /**
     * @brief 只接收电机的数据，不发送
     * @note  把电机所总线的电机ID分组标志位置0
     */
    void DJI_Motor_Instance::DJIMotorOnlyRecevie(void)
    {
        this->stop_flag = Motor_General_Def_n::MOTOR_ONLY_RECEIVE;
    }

    /**
     * @brief 更新 dt 值
     * @param new_dt 新的dt值
     * @return none
    */
    void DJI_Motor_n::DJI_Motor_Instance::Update_DJIMotor_dt(float new_dt)
    {
        this->dt = new_dt;
    }

    /**
     * @brief 获取当前 dt 值
     * @return 当前 dt 值
    */
    float DJI_Motor_n::DJI_Motor_Instance::Get_DJIMotor_dt(void)
    {
        return this->dt;
    }
    /**
     * @brief 修改并添加最外层的闭环
    */
    void DJI_Motor_n::DJI_Motor_Instance::DJIMotorOuterLoop(Motor_General_Def_n::Closeloop_Type_e outer_loop)
    {
        this->motor_settings.outer_loop_type = outer_loop;
        this->motor_settings.close_loop_type = (Motor_General_Def_n::Closeloop_Type_e)(this->motor_settings.close_loop_type | outer_loop);
    }

    // 为所有电机实例计算三环PID,发送控制报文
    void DJIMotorControl()
    {
        // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
        uint8_t group, num; // 电机组号和组内编号
        DJI_Motor_n::DJI_Motor_Instance *motor;
        Motor_General_Def_n::Motor_Control_Setting_s *motor_setting; // 电机控制参数
        Motor_General_Def_n::Motor_Controller_c *motor_controller;   // 电机控制器
        float pid_ref;                                               // 电机PID测量值和设定值
        // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
        for (size_t i = 0; i < DJI_Motor_Instance::dji_motor_idx; ++i)
        {
            // 减小访存开销,先保存指针引用
            motor = DJI_Motor_Instance::dji_motor_instance_p[i];
            motor_setting    = &DJI_Motor_Instance::dji_motor_instance_p[i]->motor_settings;
            motor_controller = &DJI_Motor_Instance::dji_motor_instance_p[i]->motor_controller;
            pid_ref = motor_controller->GetRefVal(); // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
            // 分组填入发送数据
            group = motor->sender_group;
            num = motor->message_num;
            // 若该电机处于停止状态,直接将buff置零
            if (motor->stop_flag == Motor_General_Def_n::MOTOR_STOP)
            {
                memset(sender_assignment[group].tx_buff + 2 * num, 0, sizeof(uint16_t));
                continue;
            }
            // 若只接收该电机的数据，则该电机不发送且别把其他电机注册在该总线上面的ID分组上面
            if(motor->stop_flag == Motor_General_Def_n::MOTOR_ONLY_RECEIVE)
            {
                DJI_Motor_Instance::sender_enable_flag[group] = 0;
                continue;
            }
            // 不是设置为固定输出才计算PID
            if(motor->stop_flag != Motor_General_Def_n::MOTOR_OUTPUT_ONLY_ME)
            {
                /* 以下是PID的计算部分 */
                if (motor_setting->motor_reverse_flag == Motor_General_Def_n::MOTOR_DIRECTION_REVERSE)
                    pid_ref = -pid_ref; // 设置反转

                // pid_ref会顺次通过被启用的闭环充当数据的载体
                // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
                if ((motor_setting->close_loop_type & Motor_General_Def_n::ANGLE_LOOP) &&
                    motor_setting->outer_loop_type == Motor_General_Def_n::ANGLE_LOOP)
                {
                    if(motor_setting->angle_feedback_source == Motor_General_Def_n::TOTAL_ANGLE)
                    {
                        if(motor->motor_controller.angle_PID.ECF_PID_GetActValSource()!= &motor->MotorMeasure.measure.total_angle)
                        {
                            motor->motor_controller.angle_PID.ECF_PID_ChangeActValSource(&motor->MotorMeasure.measure.total_angle);
                        }

                    }
                    else if(motor_setting->angle_feedback_source == Motor_General_Def_n::RECORD_LENGTH)
                    {
                        if(motor->motor_controller.angle_PID.ECF_PID_GetActValSource()!= &motor->MotorMeasure.measure.record_length)
                        {
                            motor->motor_controller.angle_PID.ECF_PID_ChangeActValSource(&motor->MotorMeasure.measure.record_length);
                        }
                    }
                    else if(motor_setting->angle_feedback_source == Motor_General_Def_n::MOTOR_FEED)
                    {
                        if(motor->motor_controller.angle_PID.ECF_PID_GetActValSource()!= &motor->MotorMeasure.measure.record_ecd)
                        {
                            // 把累计编码器赋值给float的变量
                            motor->motor_controller.angle_PID.ECF_PID_ChangeActValSource(&motor->MotorMeasure.measure.record_ecd);
                        }
                    }
                    else if(motor_setting->angle_feedback_source == Motor_General_Def_n::NONE_FEED) {
                       if(motor->motor_controller.angle_PID.ECF_PID_GetActValSource()!= NULL) {
                            // 把位置环的数据来源设置为NULL，在PID算法代码里面表示实际值一直是0
                            motor->motor_controller.angle_PID.ECF_PID_ChangeActValSource(NULL);
                        }
                    }
                    // 更新pid_ref进入下一个环
                    pid_ref = motor_controller->angle_PID.ECF_PID_Calculate(pid_ref);
                    motor->pid_angle_out = pid_ref;
                }

                // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
                if ((motor_setting->close_loop_type & Motor_General_Def_n::SPEED_LOOP) &&
                    (motor_setting->outer_loop_type & (Motor_General_Def_n::ANGLE_LOOP | Motor_General_Def_n::SPEED_LOOP)))
                {
                    if(motor_setting->speed_feedback_source == Motor_General_Def_n::SPEED_APS)
                    {
                        if(motor->motor_controller.speed_PID.ECF_PID_GetActValSource()!= &motor->MotorMeasure.measure.speed_aps)
                        {
                            motor->motor_controller.speed_PID.ECF_PID_ChangeActValSource(&motor->MotorMeasure.measure.speed_aps);
                        }
                    }
                    else if(motor_setting->speed_feedback_source == Motor_General_Def_n::ANGULAR_SPEED)
                    {
                        if(motor->motor_controller.speed_PID.ECF_PID_GetActValSource()!= &motor->MotorMeasure.measure.angular_speed)
                        {
                            motor->motor_controller.speed_PID.ECF_PID_ChangeActValSource(&motor->MotorMeasure.measure.angular_speed);
                        }
                    }
                    else if(motor_setting->speed_feedback_source == Motor_General_Def_n::LINEAR_SPEED)
                    {
                        if(motor->motor_controller.speed_PID.ECF_PID_GetActValSource()!= &motor->MotorMeasure.measure.linear_speed)
                        {
                            motor->motor_controller.speed_PID.ECF_PID_ChangeActValSource(&motor->MotorMeasure.measure.linear_speed);
                        }
                    }
                    else if(motor_setting->speed_feedback_source == Motor_General_Def_n::MOTOR_FEED)
                    {
                        if(motor->motor_controller.speed_PID.ECF_PID_GetActValSource()!= &motor->MotorMeasure.measure.feedback_speed)
                        {
                            motor->motor_controller.speed_PID.ECF_PID_ChangeActValSource(&motor->MotorMeasure.measure.feedback_speed);
                        }
                    } 
                    // 更新pid_ref进入下一个环
                    pid_ref = motor_controller->speed_PID.ECF_PID_Calculate(pid_ref);
                    motor->pid_speed_out = pid_ref;
                }
                // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈、一般用不到电流环
                if (motor_setting->close_loop_type & Motor_General_Def_n::CURRENT_LOOP)
                {
                    motor->motor_controller.current_PID.ECF_PID_ChangeActValSource(&motor->MotorMeasure.measure.feedback_real_current);
                    pid_ref = motor_controller->current_PID.ECF_PID_Calculate(pid_ref);
                    motor->pid_speed_out = pid_ref;
                }
                // 限幅
                switch (motor->motor_type)
                {
                    case Motor_General_Def_n::GM6020:
                    {
                        if(pid_ref > motor->Current_limit_H_6020)
                            pid_ref = motor->Current_limit_H_6020;
                        else if(pid_ref < motor->Current_limit_L_6020)
                            pid_ref = motor->Current_limit_L_6020;      
                        break;
                    }
                    case Motor_General_Def_n::M3508:
                    {
                        if(pid_ref > motor->Current_limit_H_3508)
                            pid_ref = motor->Current_limit_H_3508;
                        else if(pid_ref < motor->Current_limit_L_3508)
                            pid_ref = motor->Current_limit_L_3508;      
                        break;
                    }
                    case Motor_General_Def_n::M2006:
                    {
                        if(pid_ref > motor->Current_limit_H_2006)
                            pid_ref = motor->Current_limit_H_2006;
                        else if(pid_ref < motor->Current_limit_L_2006)
                            pid_ref = motor->Current_limit_L_2006;
                        break;
                    }
                    default:
                        break;
                }
                // 获取最终输出
                motor->set = (int16_t)pid_ref;
                // 外部查看PID的输出
                motor->pid_out = pid_ref;
                motor->give_current = motor->set;
            }
            sender_assignment[group].tx_buff[2 * num] = (uint8_t)(motor->give_current >> 8);         // 低八位
            sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(motor->give_current & 0x00ff); // 高八位
        }
        // Chassis_Power_Limit();
        //遍历flag,检查是否要发送这一帧报文
        for (size_t i = 0; i < 9; ++i)
        {
            if (DJI_Motor_Instance::sender_enable_flag[i])
            {
                sender_assignment[i].ECF_Transmit(1);
            }
        }
    }
    // 哨兵测试代码，别用捏
    void DJIMotorRuning()
    {
        // 遍历flag,检查是否要发送这一帧报文
        for (size_t i = 0; i < 9; ++i)
        {
            if (DJI_Motor_Instance::sender_enable_flag[i])
            {
                sender_assignment[i].ECF_Transmit(1);
            }
        }
    }

    // 输出只用自己设置的输出值put
    void DJI_Motor_Instance::DJIMotorOutputFix(int16_t output)
    {
        this->stop_flag = Motor_General_Def_n::MOTOR_OUTPUT_ONLY_ME;
        this->give_current = output;
    }
/*---------------------------------------------电机以外的反馈来源设置--------------------------------------------*/
    // 修改位置环的电机反馈类型
    void DJI_Motor_Instance::ChangeAnglePid_MotorFeedback(Motor_General_Def_n::Feedback_Source_e feedback)
    {
        this->motor_settings.angle_feedback_source = feedback;
    }
    // 修改位置环为其他反馈指针(如陀螺仪的反馈)
    void DJI_Motor_Instance::Set_ANGLE_PID_other_feedback(float *other_ptr)
    {
        this->motor_settings.angle_feedback_source = Motor_General_Def_n::OTHER_FEED;
        this->other_angle_feedback_ptr_ = other_ptr;
    }
    // 修改速度环为其他反馈指针
    void DJI_Motor_Instance::Set_SPEED_PID_other_feedback(float *other_ptr)
    {
        this->motor_settings.speed_feedback_source = Motor_General_Def_n::OTHER_FEED;
        this->other_speed_feedback_ptr_ = other_ptr;
    }
/*-----------------------------------------------------------------------------------------------*/
    // 修改3508、6020的自定义限幅
    void DJI_Motor_Instance::Set_Current_limit_H_3508(int16_t high)
    {
        this->Current_limit_H_3508 = high;
    }
    void DJI_Motor_Instance::Set_Current_limit_L_3508(int16_t low)
    {
        this->Current_limit_L_3508 = low;
    }
    // 修改6020的自定义限幅
    void DJI_Motor_Instance::Set_Current_limit_H_6020(int16_t high)
    {
        this->Current_limit_H_6020 = high;
    }
    void DJI_Motor_Instance::Set_Current_limit_L_6020(int16_t low)
    {
        this->Current_limit_L_6020 = low;
    }
    // 修改2006电机的自定义限幅
    void DJI_Motor_Instance::Set_Current_limit_H_2006(int16_t high)
    {
        this->Current_limit_H_2006 = high;
    }
    void DJI_Motor_Instance::Set_Current_limit_L_2006(int16_t low)
    {
        this->Current_limit_L_2006 = low;
    }

    // 获取电机实际数据的函数
    // 返回DJIMotorMeasure_t类型的结构体，可用于调试
    DJIMotorMeasure_t DJI_Motor_Instance::Get_Motor_measure()
    {
        return MotorMeasure.measure;
    }
    // 获取电机位置环输出
    float DJI_Motor_Instance::Get_Motor_Angle_PidOut()
    {
        return this->pid_angle_out;
    }
    // 获取电机速度环输出
    float DJI_Motor_Instance::Get_Motor_Speed_PidOut()
    {
        return this->pid_speed_out;
    }
    // 获取电机电流环输出
    float DJI_Motor_Instance::Get_Motor_Current_PidOut()
    {
        return this->pid_current_out;
    }
    // 获取电机的最终PID输出结果
    float DJI_Motor_Instance::Get_Motor_Final_PidOut()
    {
        return this->pid_out;
    }
    /***
     * @brief 清空measure 结构体中的内容 
    */
    void DJIMotorMeasure_c::MeasureClear()
    {
        memset(&this->measure, 0, sizeof(DJIMotorMeasure_t));
        this->measure.init_flag = false;
    }

}

// 临角处理16位（对应角度正值）
int16_t angle_limiting_int16(int16_t Angl_Err, int16_t lap_encoder) {
   //|当前值 - 上一次值| > 编码器最大值/2 时说明向上溢出
   if (Angl_Err < -(lap_encoder / 2))
   {
       Angl_Err += (lap_encoder - 1);
   }
   if (Angl_Err > (lap_encoder / 2)) {
       Angl_Err -= (lap_encoder - 1);
   }
   return Angl_Err;
}

/**
 * @todo  是否可以简化多圈角度的计算？
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void DecodeDJIMotor(BSP_CAN_Part_n::FDCANInstance_c *register_instance)
{
    uint8_t current_idx;
    uint8_t *rxbuff;
    if (register_instance->rx_id_ > 0x200 && register_instance->rx_id_ < 0x20C)
    {
    }
    else // 不符合电机返回ID的数据不进行处理
    {
        return;
    }
    // Debug查看数据看dji_motor_instance_p这个数组里面的值
    for (uint8_t i = 0; i < DJI_Motor_n::DJI_Motor_Instance::dji_motor_idx; i++)
    {
        if (DJI_Motor_n::DJI_Motor_Instance::dji_motor_instance_p[i]->motor_fdcan_instance.rx_id_ == register_instance->rx_id_ &&
            DJI_Motor_n::DJI_Motor_Instance::dji_motor_instance_p[i]->motor_fdcan_instance.fdcan_handle_ == register_instance->fdcan_handle_)
        {
            rxbuff = register_instance->rx_buff;
            current_idx = i;
            DJI_Motor_n::DJI_Motor_Instance *motor = DJI_Motor_n::DJI_Motor_Instance::dji_motor_instance_p[current_idx];
            DJI_Motor_n::DJIMotorMeasure_c *measure = &motor->MotorMeasure; // measure要多次使用,保存指针减小访存开销
            motor->lost_detection_dji->Online();
            motor->Update_DJIMotor_dt(dwt_DJIMotor->ECF_DWT_GetDeltaT(&motor->feed_cnt));
            // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
            measure->measure.feedback_ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
            measure->measure.angle_single_round = (float)measure->measure.feedback_ecd / measure->lap_encoder * 360.0;
            measure->measure.feedback_speed = (float)(int16_t)(rxbuff[2] << 8 | rxbuff[3]);
            measure->measure.speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->measure.speed_aps +
                                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * measure->measure.feedback_speed;
            measure->measure.feedback_real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->measure.feedback_real_current +
                                                     CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
            measure->measure.feedback_temperature = rxbuff[6];
            // 标志位更新
            if(measure->measure.init_flag == false)
            {
                measure->measure.last_ecd = measure->measure.feedback_ecd;
                measure->measure.init_flag = true;
            }
            // 单个电机温度保护
            if (measure->measure.feedback_temperature > 60)
            {
                motor->DJIMotorStop();
            }
            // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
            int16_t erro = measure->measure.feedback_ecd - measure->measure.last_ecd;
            if (erro > (measure->lap_encoder / 2.0f))
            {
                measure->measure.total_round--;
            }
            else if (erro < -(measure->lap_encoder / 2.0f))
            {
                measure->measure.total_round++;
            }
            measure->measure.last_ecd = measure->measure.feedback_ecd;
            measure->measure.last_record_ecd = measure->measure.record_ecd;
            erro = angle_limiting_int16(erro, measure->lap_encoder);

            measure->measure.record_ecd +=  erro;
            measure->measure.relative_angle = dji_maths.loop_fp32_constrain(measure->measure.angle_single_round - measure->measure.zero_offset,0,360);
            
            measure->measure.total_angle = measure->measure.total_round * 360 + measure->measure.angle_single_round;
            measure->measure.record_length = measure->measure.record_ecd / measure->ecd2length;
            measure->measure.angular_speed = measure->measure.speed_aps / measure->gear_Ratio;
            measure->measure.linear_speed = measure->measure.angular_speed * measure->radius;
            break;
        }
    }
}

/* ------------------------------------------------------------------------------------------这是一条分割线------------------------------------------------------------------------------------------------------- */
// /**
//  * @todo  后面工程用到再开发吧
//  * @brief 电机堵转初始化
//  * @param init_current 堵转初始化设置电流, 注意方向
//  * @return 如果初始化完成, 则返回 0;
//  * @note 为了方便可能的重复初始化, 因此在初始化判断完成, 返回 1 之后, 再次调用会重新开始初始化进程
//  * @note 因此, 最好在外部额外用一个变量, 判断是否完成初始化, 初始化完成, 后不再继续调用该函数 
// */
// uint8_t DJI_Motor_Instance::DJIMotorBlockInit(int16_t init_current)
// {
//         switch (this->motor_type)
//         {
//             case Motor_General_Def_n::GM6020:
//             case Motor_General_Def_n::M3508:
//             {
//                 if(init_current > 16384)
//                     init_current = (int16_t)16384;
//                 else if(init_current < -16384)
//                     init_current = (int16_t)-16384;
                
//                 break;
//             }
//             case Motor_General_Def_n::M2006:
//             {

//                 if(init_current > 10000)
//                     init_current = (int16_t)10000;
//                 else if(init_current < -10000)
//                     init_current = (int16_t)-10000;
//                 break;
//             }
//             default:
//                 break;
//         }

//     this->block_val.init_current = init_current;
//     if(this->block_val.block_init_if_finish)
//     {  
//         this->block_val.block_init_if_finish = 0;
//         this->block_val.last_position = 0;
//         return 1;
//     }
//     this->stop_flag = Motor_General_Def_n::MOTOR_INIT;
//     return 0;
// }

// @todo  后面工程用到再开发吧
// void DJI_Motor_Instance::DJIMotorBlockInitAchieve()
// {
//     if(this->stop_flag == Motor_General_Def_n::MOTOR_INIT)
//     {
//         if(this->block_val.block_init_if_finish)
//         {
//             this->MotorMeasure.MeasureClear();
//             this->DJIMotorEnable();
//             return;
//         }
//         if(this->MotorMeasure.measure.init_flag)
//         {
//             if(user_abs(this->block_val.last_position - this->MotorMeasure.measure.record_ecd) < 20)
//             {
//                 this->block_val.block_times++;
//             }
//             else
//             {
//                 this->block_val.block_times = 0;
//             }

//             if(this->block_val.block_times > 400)
//             {
//                 this->block_val.block_init_if_finish = 1;
//             }
//         }
//         this->block_val.last_position =  this->MotorMeasure.measure.record_ecd;
//     }
// }

// /**
//  * @todo 工程用（待开发）
//  * @brief Dji 电机锁死
//  * @note 具体实现为不允许更改闭环运算参考值
// */
// void DJI_Motor_n::DJI_Motor_Instance::DJIMotorLock(void)
// {
//     this->stop_flag = Motor_General_Def_n::MOTOR_LOCK;
// }