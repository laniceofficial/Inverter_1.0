/*************************** Dongguan-University of Technology -ACE**************************
* @file    dji_motor.cpp->dji_driver.cpp
 * @author  study-sheep
 * @version V1.0
 * @date    2024/10/18
 * @brief   电机模块文件
 ******************************************************************************
 * @verbatim
 * 支持PID控制 + 温度保护
 * 详细使用看demo.md文件，太长了，看起来太难受了
 * @attention
 *      无
 * @version           time
 * v1.0   基础版本（can）     2024-10-18    已测试
 * v2.0   重构+加上了绝对编码   2024-11-20
 * v2.1   加上了外部可以通过在初始化的时候设置反馈的类型（枚举类型：Feedback_Source_e）
 *        电机以外的反馈需要调用以下两个函数
 *                             Set_ANGLE_PID_other_feedback(float *other_ptr);
 *                             Set_SPEED_PID_other_feedback(float *other_ptr);
 * v3.0(胡炜)    区分了绝对角度和相对角度，修改了GM6020编码值设置错误的问题，修复了解析数据不正确的问题
 *              修复了默认反馈指针设置错误的问题（我建议是自己调用函数设置）
 *              数据Mesure结构体添加了相对角度，零偏角    2024-11-24 已测试
 * v3.1   加上了电机失联处理       √ 已测试
 * v4.0   继承大炜哥的基类，保留基本分组逻辑      2025-10-6
 * todo   在有6020电机升级固件时再测试它的电流控制
 ************************** Dongguan-University of Technology -ACE***************************/
#include <cstdint>
#include "dji_driver.hpp"
#include "user_maths.hpp"
using namespace Motor_n::DjiMotor_n;

//自定义数学库
user_maths_c dji_maths;

    DjiDriver_c *DjiDriver_c::dji_motor_instance_p[DJI_MOTOR_CNT] = {nullptr}; // 会在control任务中遍历该指针数组进行pid计算
    uint8_t DjiDriver_c::dji_motor_idx = 0;                                  // register idx,是该文件的全局电机索引,在注册时使用

    /**
     * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个或者9个(can或者fdcan)can_instance专门负责发送
     *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
     *
     * @note  因为只用于发送,所以不需要在can文件中注册
     *
     * C610(m2006)/C620(m3508):0x1ff,0x200;
     * GM6020:0x1ff,0x2ff
     *
     * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
     * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
     * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
     * can3: [6]:0x1FF,[7]:0x200,[8]:0x2FF
     */
    // 下面的这几个对象只负责发送，不负责接收回调函数，不会占用CAN实例数组
#ifdef STM32H723xx
    BSP_n::Fdcan_c sender_assignment[9]=
    {
        BSP_n::Fdcan_c(0x1ff,0x000),
        BSP_n::Fdcan_c(0x200,0x000),
        BSP_n::Fdcan_c(0x2ff,0x000),
        BSP_n::Fdcan_c(0x1ff,0x000),
        BSP_n::Fdcan_c(0x200,0x000),
        BSP_n::Fdcan_c(0x2ff,0x000),
        BSP_n::Fdcan_c(0x1ff,0x000),
        BSP_n::Fdcan_c(0x200,0x000),
        BSP_n::Fdcan_c(0x2ff,0x000)
    };
    /**
     * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
     *        flag的初始化在 MotorSenderGrouping()中进行
     */
    uint8_t sender_enable_flag[9] = {0};
#else
    BSP_n::Can_c sender_assignment[6] =
    {
        BSP_n::Can_c(0x1ff, 0x000),
        BSP_n::Can_c(0x200, 0x000),
        BSP_n::Can_c(0x2ff, 0x000),
        BSP_n::Can_c(0x1ff, 0x000),
        BSP_n::Can_c(0x200, 0x000),
        BSP_n::Can_c(0x2ff, 0x000)
    };
    uint8_t sender_enable_flag[6] = {0};
#endif

    /***
     * @brief 构造函数
     * @param motor_config 电机基类配置结构体
     */
    DjiDriver_c::DjiDriver_c(MotorBaseDef_n::Motor_Base_Config_t motor_config) : MotorBase_c(motor_config)
    {
#ifdef STM32H723xx
        if(!dji_motor_idx)
        {
            sender_assignment[0].MotorInit(&hfdcan1,FDCAN_STANDARD_ID);
            sender_assignment[1].MotorInit(&hfdcan1,FDCAN_STANDARD_ID);
            sender_assignment[2].MotorInit(&hfdcan1,FDCAN_STANDARD_ID);
            sender_assignment[3].MotorInit(&hfdcan2,FDCAN_STANDARD_ID);
            sender_assignment[4].MotorInit(&hfdcan2,FDCAN_STANDARD_ID);
            sender_assignment[5].MotorInit(&hfdcan2,FDCAN_STANDARD_ID);
            sender_assignment[6].MotorInit(&hfdcan3,FDCAN_STANDARD_ID);
            sender_assignment[7].MotorInit(&hfdcan3,FDCAN_STANDARD_ID);
            sender_assignment[8].MotorInit(&hfdcan3,FDCAN_STANDARD_ID);
        }
#else
        if (!dji_motor_idx)
        {
            sender_assignment[0].MotorInit(&hcan1, CAN_ID_STD);
            sender_assignment[1].MotorInit(&hcan1, CAN_ID_STD);
            sender_assignment[2].MotorInit(&hcan1, CAN_ID_STD);
            sender_assignment[3].MotorInit(&hcan2, CAN_ID_STD);
            sender_assignment[4].MotorInit(&hcan2, CAN_ID_STD);
            sender_assignment[5].MotorInit(&hcan2, CAN_ID_STD);
        }
#endif
        this->MotorSenderGrouping();
        // 实现分发机制
#ifdef STM32H723xx
        this->SetCallback([this](BSP_n::Fdcan_c *can_instance)
        { this->CallBack(can_instance); });
#else
        this->SetCallback([this](BSP_n::Can_c *can_instance)
        { this->CallBack(can_instance); });
#endif // USE_H7_if_or_not
        dji_motor_instance_p[dji_motor_idx++] = this;
        switch (this->motor_type_)
        {
            case MotorBaseDef_n::Motor_Type_euc::M3508:
            {
                this->motor_data_.motor_fixed_param.encoder_resolution = 8192;
                this->motor_data_.motor_fixed_param.torque_constant = 0.3f;
                this->motor_data_.motor_fixed_param.current_resolution = 20 / 16384;
                // this->motor_data_.motor_fixed_param.ratio = 19;
                break;
            }
            case MotorBaseDef_n::Motor_Type_euc::M2006:
            {
                this->motor_data_.motor_fixed_param.encoder_resolution = 8192;
                this->motor_data_.motor_fixed_param.torque_constant = 0.18f;
                this->motor_data_.motor_fixed_param.current_resolution = 10 / 10000;
                // this->motor_data_.motor_fixed_param.ratio = 36;
                break;
            }
            case MotorBaseDef_n::Motor_Type_euc::GM6020:
            {
                this->motor_data_.motor_fixed_param.encoder_resolution = 8192;
                this->motor_data_.motor_fixed_param.torque_constant = 0.741f;
                this->motor_data_.motor_fixed_param.current_resolution = 3 / 16384;
                // this->motor_data_.motor_fixed_param.ratio = 1;
                break;
            }
            default:
                break;
        }

        this->motor_controller_.SetAngleFeedbackPtr(&this->motor_data_.motor_processed_data.total_ecd);// 编码值
        this->motor_controller_.SetSpeedFeedbackPtr(&this->motor_data_.motor_raw_data.feedback_speed);// rpm
        this->motor_controller_.SetCurrentFeedbackPtr(&this->motor_data_.motor_raw_data.force_feedback);
        // 可以填写你自己想要的反馈值
        // motor->motor_controller_.SetOtherAngleFeedbackPtr()...

        this->MotorDataClear();
        this->Disable();
    }

    /***
     * @brief 电机分组的函数
     * @note 先分组再注册回调函数
     */
    void DjiDriver_c::MotorSenderGrouping() {
        uint8_t motor_id = this->motor_can_instance_.tx_id_ - 1; // 下标从零开始,先减一方便赋值
        uint8_t motor_send_num;
        uint8_t motor_grouping;

        switch (this->motor_type_) {
#ifdef STM32H723xx
            case MotorBaseDef_n::Motor_Type_euc::M3508:
            case MotorBaseDef_n::Motor_Type_euc::M2006:
            {
                if (motor_id < 4) // 根据ID分组
                {
                    motor_send_num = motor_id;
                    motor_grouping = this->motor_can_instance_.GetFdcanhandle() == &hfdcan1 ? 1 :
                                    (this->motor_can_instance_.GetFdcanhandle() == &hfdcan2) ? 4 : 7;
                }
                else
                {
                    motor_send_num = motor_id - 4;
                    motor_grouping = this->motor_can_instance_.GetFdcanhandle() == &hfdcan1 ? 0 :
                                    (this->motor_can_instance_.GetFdcanhandle() == &hfdcan2) ? 3 : 6;
                }
                // 计算接收id并设置分组发送id
                this->motor_can_instance_.rx_id_ = 0x200 + motor_id + 1;   // 把ID+1,进行分组设置
                sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
                this->message_num  = motor_send_num;
                this->sender_group = motor_grouping;
                // 检查是否发生id冲突
                for (size_t i = 0; i < dji_motor_idx; ++i)
                {
                    if ((dji_motor_instance_p[i]->motor_can_instance_.GetFdcanhandle() == this->motor_can_instance_.GetFdcanhandle()) &&
                        (dji_motor_instance_p[i]->motor_can_instance_.rx_id_ == this->motor_can_instance_.rx_id_))
                    {
                        while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突
                        {}
                    }
                }
                break;
            }
            case MotorBaseDef_n::Motor_Type_euc::GM6020:
            {
                if (motor_id < 4)
                {
                    motor_send_num = motor_id;
                    motor_grouping =  this->motor_can_instance_.GetFdcanhandle() == &hfdcan1 ? 0 :
                                    (this->motor_can_instance_.GetFdcanhandle() == &hfdcan2) ? 3 : 6;
                }
                else
                {
                    motor_send_num = motor_id - 4;
                    motor_grouping =  this->motor_can_instance_.GetFdcanhandle() == &hfdcan1 ? 2 :
                                    (this->motor_can_instance_.GetFdcanhandle() == &hfdcan2) ? 5 : 8;
                }
                this->motor_can_instance_.rx_id_ = 0x204 + motor_id + 1;   // 把ID+1,进行分组设置
                sender_enable_flag[motor_grouping] = 1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
                this->message_num = motor_send_num;
                this->sender_group = motor_grouping;
                for (size_t i = 0; i < dji_motor_idx; ++i)
                {
                    if (dji_motor_instance_p[i]->motor_can_instance_.GetFdcanhandle() == this->motor_can_instance_.GetFdcanhandle() &&
                        dji_motor_instance_p[i]->motor_can_instance_.rx_id_ == this->motor_can_instance_.rx_id_)
                    {
                        while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突
                        {}
                    }
                }
                break;
            }

            default:
                while (1) // 其他类型电机不适用于该初始化
                {
                }
                break;
#else
            case MotorBaseDef_n::Motor_Type_euc::M3508:
            case MotorBaseDef_n::Motor_Type_euc::M2006:
            {
                if (motor_id < 4) // 根据ID分组
                {
                    motor_send_num = motor_id;
                    motor_grouping = this->motor_can_instance_.GetCanhandle() == &hcan1 ? 1 : 4;
                }
                else
                {
                    motor_send_num = motor_id - 4;
                    motor_grouping = this->motor_can_instance_.GetCanhandle() == &hcan1 ? 0 : 3;
                }
                // 计算接收id并设置分组发送id
                this->motor_can_instance_.rx_id_ = 0x200 + motor_id + 1; // 把ID+1,进行分组设置
                sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
                this->message_num = motor_send_num;
                this->sender_group = motor_grouping;
                // 检查是否发生id冲突
                for (size_t i = 0; i < dji_motor_idx; ++i)
                {
                    if ((dji_motor_instance_p[i]->motor_can_instance_.GetCanhandle() == this->motor_can_instance_.GetCanhandle()) &&
                        (dji_motor_instance_p[i]->motor_can_instance_.rx_id_ == this->motor_can_instance_.rx_id_))
                    {
                        while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突
                        {
                        }
                    }
                }
                break;
            }
            case MotorBaseDef_n::Motor_Type_euc::GM6020:
            {
                if (motor_id < 4)
                {
                    motor_send_num = motor_id;
                    motor_grouping = this->motor_can_instance_.GetCanhandle() == &hcan1 ? 0 : 3;
                }
                else
                {
                    motor_send_num = motor_id - 4;
                    motor_grouping = this->motor_can_instance_.GetCanhandle() == &hcan1 ? 2 : 5;
                }
                this->motor_can_instance_.rx_id_ = 0x204 + motor_id + 1; // 把ID+1,进行分组设置
                sender_enable_flag[motor_grouping] = 1;                 // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
                this->message_num = motor_send_num;
                this->sender_group = motor_grouping;
                for (size_t i = 0; i < dji_motor_idx; ++i)
                {
                    if (dji_motor_instance_p[i]->motor_can_instance_.GetCanhandle() == this->motor_can_instance_.GetCanhandle() &&
                        dji_motor_instance_p[i]->motor_can_instance_.rx_id_ == this->motor_can_instance_.rx_id_)
                    {
                        while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突
                        {
                        }
                    }
                }
                break;
            }
            default:
                while (1) // 其他类型电机不适用于该初始化
                {
                }
                break;
#endif
        }
    }

    /***
     * @brief 设定闭环的参考值
     * @param ref 用作闭环计算大的参考值
     */
    void DjiDriver_c::DjiMotorSetRef(float ref)
    {
        if (this->motor_working_status_ == MotorBaseDef_n::Motor_Working_Status_e::MOTOR_STOP)
        {
            this->set_length_ = 0;
            this->set_speed_ = 0;
            this->set_current_ = 0;
            return; // stop直接把设定值置0
        }
        // 设置位置环时会设置
        if (MotorBaseDef_n::isUseFlag(this->motor_controller_.motor_setting.close_loop_type, MotorBaseDef_n::Closeloop_Type_euc::ANGLE_LOOP))
        {
            this->set_length_ = ref;
        }
        // 设置速度环时会设置
        else if (MotorBaseDef_n::isUseFlag(this->motor_controller_.motor_setting.close_loop_type , MotorBaseDef_n::SPEED_LOOP))
        {
            this->set_speed_ = ref;
        }
        // 设置电流环时会设置
        else if (MotorBaseDef_n::isUseFlag(this->motor_controller_.motor_setting.close_loop_type , MotorBaseDef_n::CURRENT_LOOP))
        {
            this->set_current_ = ref;
        }
        // 开环时会设置
        else if (this->motor_controller_.motor_setting.close_loop_type == MotorBaseDef_n::OPEN_LOOP)
        {
            this->set_torque_ = ref;
        }
    }

    /***
     * @brief 电机控制函数,在control任务中调用
     * @note 该函数会遍历所有注册的电机实例,进行pid计算
     */
    void Motor_n::DjiMotor_n::DjiMotorControl() {
        // 为所有电机实例计算三环PID,发送控制报文
        // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
        uint8_t group, num; // 电机组号和组内编号
        DjiDriver_c *motor;
        float pid_ref;
        // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
        for (size_t i = 0; i < DjiDriver_c::dji_motor_idx; ++i)
        {
            // 减小访存开销,先保存指针引用
            motor = DjiDriver_c::dji_motor_instance_p[i];
            // 分组填入发送数据
            group = motor->sender_group;
            num = motor->message_num;
            // 设置位置环时会设置
            if (MotorBaseDef_n::isUseFlag(motor->motor_controller_.motor_setting.close_loop_type, MotorBaseDef_n::Closeloop_Type_euc::ANGLE_LOOP))
            {
                pid_ref = motor->set_length_;// 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
            }
            // 设置速度环时会设置
            else if (MotorBaseDef_n::isUseFlag(motor->motor_controller_.motor_setting.close_loop_type , MotorBaseDef_n::SPEED_LOOP))
            {
                pid_ref = motor->set_speed_;
            }
            // 设置电流环时会设置
            else if (MotorBaseDef_n::isUseFlag(motor->motor_controller_.motor_setting.close_loop_type , MotorBaseDef_n::CURRENT_LOOP))
            {
                pid_ref = motor->set_current_;
            }
            else if (motor->motor_controller_.motor_setting.close_loop_type == MotorBaseDef_n::OPEN_LOOP)
            {
                pid_ref = motor->set_torque_;
            }

            // 若该电机处于停止状态,直接将buff置零
            if (motor->motor_working_status_ == MotorBaseDef_n::Motor_Working_Status_e::MOTOR_STOP)
            {
                memset(sender_assignment[group].tx_buff_ + 2 * num, 0, sizeof(uint16_t));
                continue;
            }
            // 若只接收该电机的数据，则该电机不发送且别把其他电机注册在该总线上面的ID分组上面
            if(motor->motor_working_status_ == MotorBaseDef_n::Motor_Working_Status_e::MOTOR_ONLY_RECEIVE)
            {
                sender_enable_flag[group] = 0;
                continue;
            }
            // 不是设置为固定输出才计算PID
            if(motor->motor_working_status_ != MotorBaseDef_n::Motor_Working_Status_e::MOTOR_OUTPUT_ONLY_ME) {
                motor->give_current_ = motor->PIDCalculate(&pid_ref);
            }
        }
    }

    /***
     * @brief 设置发送数据的电流值
     * @note 该函数会遍历所有注册的电机实例,把计算好的电流值填入发送数据
     */
    void Motor_n::DjiMotor_n::set_GiveCurrent()
    {
        uint8_t group_, num_; // 电机组号和组内编号
        DjiDriver_c *motor = nullptr;
        for (size_t i = 0; i < DjiDriver_c::dji_motor_idx; ++i)
        {
            motor = DjiDriver_c::dji_motor_instance_p[i];
            if (motor->motor_working_status_ == MotorBaseDef_n::Motor_Working_Status_e::MOTOR_STOP)
            {
                motor->give_current_ = 0; // 确保电机停止时电流值为零
            }
            // 临终限幅
            if(motor->give_current_ > motor->motor_max_output_)
                 motor->give_current_ = motor->motor_max_output_;
            else if( motor->give_current_ < motor->motor_min_output_)
                 motor->give_current_ = motor->motor_min_output_;
            group_ = motor->sender_group;
            num_   = motor->message_num;
            sender_assignment[group_].tx_buff_[2 * num_] = (uint8_t)(motor->give_current_ >> 8);         // 低八位
            sender_assignment[group_].tx_buff_[2 * num_ + 1] = (uint8_t)(motor->give_current_ & 0x00ff); // 高八位
        }
    }

    /***
     * @brief 发送数据的函数
     */
    void Motor_n::DjiMotor_n::MotorTransmit()
    {
#ifdef STM32H723xx
        for (size_t i = 0; i < 9; ++i)
        {
#else
        for (size_t i = 0; i < 6; ++i)
        {
#endif
            // 遍历flag,检查是否要发送这一帧报文
            if (sender_enable_flag[i])
            {
                sender_assignment[i].Transmit(0.01);
            }
        }
    }

    /**
     * @brief 修改并添加闭环
     */
    void DjiDriver_c::DjiMotorSetLoop(MotorBaseDef_n::Closeloop_Type_euc loop)
    {
        MotorBaseDef_n::SetFlag(this->motor_controller_.motor_setting.close_loop_type , loop);
        this->set_length_ = 0.0f;
        this->set_speed_ = 0.0f;
        this->set_current_ = 0.0f;
    }

    /**
     * @brief 修改并删除闭环
     */
    void DjiDriver_c::DjiMotorResetLoop(MotorBaseDef_n::Closeloop_Type_euc loop)
    {
        MotorBaseDef_n::ClearFlag(this->motor_controller_.motor_setting.close_loop_type , loop);
        this->set_length_ = 0.0f;
        this->set_speed_ = 0.0f;
        this->set_current_ = 0.0f;
    }

    /**
     * @brief Dji电机使能
     */
    void DjiDriver_c::Enable()
    {
        motor_working_status_ = MotorBaseDef_n::Motor_Working_Status_euc::MOTOR_ENALBED; motor_init_flag_ = true;
    }

    /**
     * @brief Dji电机失能
     */
    void DjiDriver_c::Disable()
    {
        motor_working_status_ = MotorBaseDef_n::Motor_Working_Status_euc::MOTOR_STOP;
    }

    /**
     * @brief Dji电机只接收数据
     * @note  不发送数据,且把该电机注册在该总线上的其他电机的发送标志位置0,防止发送空帧
     */
    void DjiDriver_c::OnlyReceive()
    {
        motor_working_status_ = MotorBaseDef_n::Motor_Working_Status_euc::MOTOR_ONLY_RECEIVE;
    }

    /**
     * @brief Dji 电机自锁
     * @note  添加位置环并把它设置成最外层的闭环
     */
    void DjiDriver_c::DjiMotorSelfLock(void)
    {
        this->MotorDataClear();
        this->motor_working_status_ = MotorBaseDef_n::Motor_Working_Status_euc::MOTOR_SELF_LOCK;
        this->DjiMotorSetLoop(MotorBaseDef_n::ANGLE_LOOP);
        this->DjiMotorSetRef(0);
    }
    /**
     * @brief Dji 电机解除自锁
     * @note  恢复速度环为最外层的闭环,并删除位置环
     */
    void DjiDriver_c::DjiMotorClearSelfLock(void)
    {
        this->MotorDataClear();
        this->motor_working_status_ = MotorBaseDef_n::Motor_Working_Status_euc::MOTOR_ENALBED;
        this->DjiMotorResetLoop(MotorBaseDef_n::ANGLE_LOOP);
    }

    /**
     * @brief 设置电机输出固定值
     * @note 该模式下不进行pid计算,直接输出给定的电流值
     * @param output 力矩值
     * @param state_set
     */
    void DjiDriver_c::SetMotorOutputFix(float output)
    {
        this->motor_working_status_ = MotorBaseDef_n::Motor_Working_Status_euc::MOTOR_OUTPUT_ONLY_ME;
        int16_t current = output / this->motor_data_.motor_fixed_param.torque_constant / this->motor_data_.motor_fixed_param.current_resolution; // 转化为电流值
        motor_controller_.pid_output = current;
        this->give_current_ = current;
    }
    /**
     * @brief  堵转检测+处理
     * @param  motor: 电机结构体
     * @retval 堵转返回1 不堵转返回0
     */
    bool DjiDriver_c::Is_MotorStuck(MotorBase_c *motor_ptr , float bulk_I , float bulk_speed , uint16_t bulk_cnt)
    {
        if ((abs(motor_ptr->motor_data_.motor_raw_data.force_feedback) > bulk_I) && (abs(motor_ptr->motor_data_.motor_raw_data.feedback_speed) < bulk_speed))
        {
            motor_ptr->stuck_cnt_++;
        }
        else
        {
            if (motor_ptr->stuck_cnt_ > 0)
                motor_ptr->stuck_cnt_ -= 20;
            else
            {
                motor_ptr->stuck_cnt_ = 0;
            }
        }
        if (motor_ptr->stuck_cnt_ > bulk_cnt)
        {
            motor_ptr->stuck_cnt_ = 0;
            motor_ptr->stuck_flag_ = MotorBaseDef_n::Motor_Stuck_Status_e::MOTOR_IS_STUCK;
            return true; // 堵转返回1
        }
        return false;
    }

    // 临角处理16位（对应角度正值）
    int16_t angle_limiting_int16(int16_t Angl_Err, float lap_encoder)
    {
        //|当前值 - 上一次值| > 编码器最大值/2 时说明向上溢出
        if (Angl_Err < -(lap_encoder / 2.0f))
        {
            Angl_Err += (lap_encoder - 1);
        }
        if (Angl_Err > (lap_encoder / 2.0f))
        {
            Angl_Err -= (lap_encoder - 1);
        }
        return Angl_Err;
    }

    /**
     * @brief can回调函数
     */
    /* 纯虚函数，子类必须实现 */
    // 子类只需重新这个解析数据函数，can分发器可以根据其id精确分配数据给子类
    void DjiDriver_c::ParseCANData(const std::array<uint8_t, 8> data)
    {
        // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
        this->motor_data_.motor_raw_data.feedback_ecd = ((uint16_t)data[0]) << 8 | data[1];
        this->motor_data_.motor_raw_data.feedback_speed = (float)(int16_t)(data[2] << 8 | data[3]);
        this->motor_data_.motor_raw_data.force_feedback = ((1.0f - CURRENT_SMOOTH_COEF) * this->motor_data_.motor_raw_data.force_feedback +
                                                                CURRENT_SMOOTH_COEF * (float)((int16_t)(data[4] << 8 | data[5])) );// 经过一阶低通滤波
        this->motor_data_.motor_raw_data.temperature[0] = data[6];


        this->motor_data_.motor_processed_data.torque = ((1.0f - CURRENT_SMOOTH_COEF) * this->motor_data_.motor_raw_data.force_feedback +
        CURRENT_SMOOTH_COEF * (float)((int16_t)(data[4] << 8 | data[5])) ) * this->motor_data_.motor_fixed_param.current_resolution * this->motor_data_.motor_fixed_param.torque_constant ;// 经过一阶低通滤波

        this->motor_data_.motor_processed_data.absolute_angle = (float)this->motor_data_.motor_raw_data.feedback_ecd / this->motor_data_.motor_fixed_param.encoder_resolution * 360.0;
        this->motor_data_.motor_processed_data.relative_angle = dji_maths.loop_fp32_constrain(this->motor_data_.motor_processed_data.absolute_angle - this->motor_data_.motor_fixed_param.zero_offset,0,360);
        this->motor_data_.motor_processed_data.speed = ((1.0f - SPEED_SMOOTH_COEF) * this->motor_data_.motor_processed_data.speed +
                                                        RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * this->motor_data_.motor_raw_data.feedback_speed)/motor_data_.motor_fixed_param.ratio;
        this->motor_data_.motor_processed_data.speed_difference = (this->motor_data_.motor_raw_data.feedback_speed - this->motor_data_.motor_raw_data.last_feedback_speed)*RPM_2_ANGLE_PER_SEC/motor_data_.motor_fixed_param.ratio;

        // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
        int16_t error = this->motor_data_.motor_raw_data.feedback_ecd - this->motor_data_.motor_raw_data.last_ecd;
        if (error > (this->motor_data_.motor_fixed_param.encoder_resolution / 2))
        {
            this->motor_data_.motor_processed_data.total_round--;
        }
        else if (error < -(this->motor_data_.motor_fixed_param.encoder_resolution / 2))
        {
            this->motor_data_.motor_processed_data.total_round++;
        }
        this->motor_data_.motor_raw_data.last_ecd = this->motor_data_.motor_raw_data.feedback_ecd;
        this->motor_data_.motor_raw_data.last_feedback_speed = this->motor_data_.motor_raw_data.feedback_speed;
        error = angle_limiting_int16(error, this->motor_data_.motor_fixed_param.encoder_resolution);
        this->motor_data_.motor_processed_data.total_ecd += error;
        this->motor_data_.motor_processed_data.total_angle = this->motor_data_.motor_processed_data.total_round * 360 + this->motor_data_.motor_processed_data.absolute_angle;
        this->motor_data_.motor_processed_data.linear_speed = this->motor_data_.motor_processed_data.speed / this->motor_data_.motor_fixed_param.ratio * this->motor_data_.motor_fixed_param.radius;
        this->motor_data_.motor_processed_data.linear_displacement =
                this->motor_data_.motor_processed_data.total_ecd / this->motor_data_.motor_fixed_param.ecd2length;
        // this->motor_data_.motor_fixed_param.encoder_resolution * 2.0f * 3.1415926f * this->motor_data_.motor_fixed_param.radius / this->motor_data_.motor_fixed_param.ratio;

        if (this->motor_data_.motor_raw_data.temperature[1] > 60)
        {
            this->Disable();
        }
}





