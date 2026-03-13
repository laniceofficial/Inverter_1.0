/************************** Dongguan-University of Technology -ACE**************************
* @file  dm_driver.cpp
* @brief DM关节电机驱动库
* @author 胡炜
* @version 1.0
* @date 2025-8-20
* @note 避免冗余，用MIT控制就够了
*
* ==============================================================================
* @endverbatim
************************** Dongguan-University of Technology -ACE***************************/
#include "dm_driver.hpp"

// 发送控制频率控制宏
#define CONTROL_SEND_HZ(HZ)\
{\
static int16_t hz = 0;\
hz++;\
if(hz < HZ)   return;\
hz = 0;\
}

using namespace Motor_n::DmMotor_n;

// 设置电机输出
void DmDriver_c::SetMotorOutputFix(float output) {
    DmMotorStateSet(DM_CMD_MOTOR_MODE);

    motor_working_status_ = MotorBaseDef_n::Motor_Working_Status_euc::MOTOR_OUTPUT_ONLY_ME;
    motor_controller_.pid_output = output;

    SetMITData(0,0,0,0,output);
    MotorBaseDef_n::Motor_Send_t send_data;
    send_data.position_des = float_to_uint(motor_mit_contorl_data_.pos, param_.p_min, param_.p_max, 16);
    send_data.velocity_des = float_to_uint(motor_mit_contorl_data_.vel, param_.v_min, param_.v_max, 12);
    send_data.Kp = float_to_uint(motor_mit_contorl_data_.kp, param_.kp_min, param_.kp_max, 12);
    send_data.Kd = float_to_uint(motor_mit_contorl_data_.kd, param_.kd_min, param_.kd_max, 12);
    send_data.torque_des = float_to_uint(motor_mit_contorl_data_.torq, param_.t_min, param_.t_max, 12);

    // std::copy(reinterpret_cast<uint8_t *>(&send_data), reinterpret_cast<uint8_t *>(&send_data) + sizeof(send_data), tx_buffer_.begin());
    // std::copy(tx_buffer_.begin(), tx_buffer_.end(), motor_can_instace_.tx_buff);
    tx_buffer_[0] = send_data.position_des >> 8;
    tx_buffer_[1] = send_data.position_des;
    tx_buffer_[2] = send_data.velocity_des >> 4;
    tx_buffer_[3] = ((send_data.velocity_des & 0xF) << 4) | (send_data.Kp >> 8);
    tx_buffer_[4] = send_data.Kp;
    tx_buffer_[5] = send_data.Kd >> 4;
    tx_buffer_[6] = ((send_data.Kd & 0xf) << 4) | (send_data.torque_des >> 8);
    tx_buffer_[7] = send_data.torque_des;
    std::copy(tx_buffer_.begin(), tx_buffer_.end(), motor_can_instance_.tx_buff_);
    CONTROL_SEND_HZ(1);
    motor_can_instance_.Transmit(1.0f);
}

// 设置MIT控制数据
void DmDriver_c::SetMITData(float pos, float vel, float kp, float kd, float t)
{
    // 可添加一些通用参数的限幅，可参考dm_mit_mode.cpp

    motor_mit_contorl_data_.pos = pos;
    motor_mit_contorl_data_.vel = vel;
    motor_mit_contorl_data_.kp = kp;
    motor_mit_contorl_data_.kd = kd;
    motor_mit_contorl_data_.torq = t;
}

// 发送函数
void DmDriver_c::Transmit(float outtime)
{
    MotorBaseDef_n::Motor_Send_t send_data;
    send_data.position_des = float_to_uint(motor_mit_contorl_data_.pos, param_.p_min, param_.p_max, 16);
    send_data.velocity_des = float_to_uint(motor_mit_contorl_data_.vel, param_.v_min, param_.v_max, 12);
    send_data.Kp = float_to_uint(motor_mit_contorl_data_.kp, param_.kp_min, param_.kp_max, 12);
    send_data.Kd = float_to_uint(motor_mit_contorl_data_.kd, param_.kd_min, param_.kd_max, 12);
    send_data.torque_des = float_to_uint(motor_mit_contorl_data_.torq, param_.t_min, param_.t_max, 12);
    
    // std::copy(reinterpret_cast<uint8_t *>(&send_data), reinterpret_cast<uint8_t *>(&send_data) + sizeof(send_data), tx_buffer_.begin());
    // std::copy(tx_buffer_.begin(), tx_buffer_.end(), motor_can_instace_.tx_buff);
    tx_buffer_[0] = send_data.position_des >> 8;
    tx_buffer_[1] = send_data.position_des;
    tx_buffer_[2] = send_data.velocity_des >> 4;
    tx_buffer_[3] = ((send_data.velocity_des & 0xF) << 4) | (send_data.Kp >> 8);
    tx_buffer_[4] = send_data.Kp;
    tx_buffer_[5] = send_data.Kd >> 4;
    tx_buffer_[6] = ((send_data.Kd & 0xf) << 4) | (send_data.torque_des >> 8);
    tx_buffer_[7] = send_data.torque_des;
    std::copy(tx_buffer_.begin(), tx_buffer_.end(), motor_can_instance_.tx_buff_);
    if (motor_working_status_ != MotorBaseDef_n::Motor_Working_Status_euc::MOTOR_OUTPUT_ONLY_ME)
    motor_can_instance_.Transmit(outtime);
}

void DmDriver_c::Transmit(const std::array<uint8_t, 8> &data, float outtime)
{
    std::copy(data.begin(), data.end(), tx_buffer_.begin());
    std::copy(data.begin(), data.end(), motor_can_instance_.tx_buff_);

    motor_can_instance_.Transmit(outtime);
}

void DmDriver_c::DmTransmit(float outtime ,DMMotor_Status_euc state_set)
{
    CONTROL_SEND_HZ(1);// >1才降频
    DmMotorStateSet(state_set);
    Transmit(outtime);
}

void DmDriver_c::ParseCANData(const std::array<uint8_t, 8> data)
{
    // 处理原始数据
    int p_int, v_int, t_int;
    p_int = (data[1] << 8) | data[2];
    v_int = (data[3] << 4) | (data[4] >> 4);
    t_int = ((data[4] & 0xF) << 8) | data[5];
    motor_data_.motor_raw_data.error = data[0] >> 4; // 错误码在高4位
    motor_data_.motor_raw_data.position = uint_to_float(p_int, param_.p_min, param_.p_max, 16);
    motor_data_.motor_raw_data.feedback_speed = uint_to_float(v_int, param_.v_min, param_.v_max, 12);
    motor_data_.motor_raw_data.force_feedback = uint_to_float(t_int, param_.t_min, param_.t_max, 12);
    motor_data_.motor_raw_data.temperature[0] = static_cast<float>(data[6]);
    motor_data_.motor_raw_data.temperature[1] = static_cast<float>(data[7]);

    // 处理后的数据
    static float last_absolute_angle = 0.0f;
    motor_data_.motor_processed_data.speed = motor_data_.motor_raw_data.feedback_speed*RPM_2_ANGLE_PER_SEC/motor_data_.motor_fixed_param.ratio; // 单位°/s
    this->motor_data_.motor_processed_data.speed_difference = (this->motor_data_.motor_raw_data.feedback_speed - this->motor_data_.motor_raw_data.last_feedback_speed)*RPM_2_ANGLE_PER_SEC/motor_data_.motor_fixed_param.ratio;// 单位°/s
    motor_data_.motor_processed_data.torque = motor_data_.motor_raw_data.force_feedback; // 单位Nm
    float temp_absangle = (motor_data_.motor_raw_data.position*RAD_2_DEGREE/motor_data_.motor_fixed_param.ratio); // 单位°
    temp_absangle = motor_maths.loop_fp32_constrain(temp_absangle,0.0f,360.0f);
    motor_data_.motor_processed_data.absolute_angle = temp_absangle;
    motor_data_.motor_processed_data.relative_angle = motor_data_.motor_processed_data.absolute_angle - motor_data_.motor_fixed_param.zero_offset; // 单位°

    float error = motor_data_.motor_processed_data.absolute_angle - last_absolute_angle;
    if (error > 180.0f)
    {
        error -= 360.0f;
        motor_data_.motor_processed_data.total_round --;
    }
    else if (error < -180.0f)
    {
        error += 360.0f;
        motor_data_.motor_processed_data.total_round ++;
    }
    motor_data_.motor_processed_data.total_angle += error;
//    motor_data_.motor_processed_data.total_round = motor_data_.motor_processed_data.total_angle / 360.0f;
    motor_data_.motor_raw_data.last_feedback_speed = motor_data_.motor_raw_data.feedback_speed;
    last_absolute_angle = motor_data_.motor_processed_data.absolute_angle;
}

void DmDriver_c::DmMotorStateSet(DMMotor_Status_euc state_set)
{
    switch (state_set)
    {
        case DM_CMD_MOTOR_MODE:
            Enable();
            break;
        case DM_CMD_RESET_MODE:
            Disable();
            break;
        case DM_CMD_CLEAR_ERROR:
            ClearErrorFlag();
            break;
        case DM_CMD_ZERO_POSITION:
            SetZero();
            break;
        default:
            break;
    }
}

void DmDriver_c::Enable()
{
    Transmit(setCtrl(0xFC),1.0f);   // 使能
    motor_init_flag_ = true; 
    motor_working_status_ = MotorBaseDef_n::Motor_Working_Status_euc::MOTOR_ENALBED;
}

void DmDriver_c::Disable()
{
    Transmit(setCtrl(0xFD),1.0f);   // 失能
    motor_working_status_ = MotorBaseDef_n::Motor_Working_Status_euc::MOTOR_STOP;
}

void DmDriver_c::OnlyReceive()
{
    motor_working_status_ = MotorBaseDef_n::Motor_Working_Status_euc::MOTOR_ONLY_RECEIVE;
}

void DmDriver_c::ClearErrorFlag()
{
    Transmit(setCtrl(0xFB),1.0f);  // 清除错误标志
}

void DmDriver_c::SetZero()
{
    Transmit(setCtrl(0xFE),1.0f);  // 设置零点
}

