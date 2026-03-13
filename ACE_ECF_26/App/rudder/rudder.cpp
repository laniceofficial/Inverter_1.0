#include "rudder.hpp"

#include "bsp_dwt.hpp"
void Rudder_c::Init(const INS_t *ins)
{
    Servo_n::Servo_Init_t config = {.htim = &htim3, .channel = TIM_CHANNEL_2, .start_angle = 135, .servo_type = Servo_n::ANGLE270, .freq = Servo_n::Hz_333};
    wing[0] = new Servo_n::Servo_c(config);
    config.channel = TIM_CHANNEL_1;
    config.htim = &htim4;
    wing[1] = new Servo_n::Servo_c(config);
    config.htim = &htim3;
    config.channel = TIM_CHANNEL_4;
    wing[2] = new Servo_n::Servo_c(config);
    config.channel = TIM_CHANNEL_1;
    config.htim = &htim2;
    wing[3] = new Servo_n::Servo_c(config);
    YawControllerInit(&ins->Gyro[1], &ins->Yaw);
    PitchControllerInit(&ins->Gyro[0], &ins->Pitch);
    RollControllerInit(&ins->Gyro[2], &ins->Roll);
}
void Rudder_c::Calculate(float yaw_cmd, float pitch_cmd, float roll_cmd)
{
    PitchCal(pitch_cmd);
    RollCal(roll_cmd);
    YawCal(yaw_cmd);
    CalDelta(ycmd, pcmd, rcmd);
}
void Rudder_c::RollStableOnly()
{
    RollCal(0);
    CalDelta(0, 0, rcmd);
}
void Rudder_c::PitchCal(float pitch_cmd)
{
    float rate_set = PID_pos_p.Calc(pitch_cmd);
    pcmd = PID_rate_p.Calc(rate_set);
}
void Rudder_c::YawCal(float yaw_cmd)
{
    float rate_set = PID_pos_y.Calc(yaw_cmd);
    ycmd = PID_rate_y.Calc(rate_set);
}
void Rudder_c::RollCal(float roll_cmd)
{
    float rate_set = PID_pos_r.Calc(roll_cmd);
    rcmd = PID_rate_r.Calc(rate_set);
}

// 从镖头垂直与纸面向外，左上1，右上2，右下3，左下4
// 设俯仰舵偏δp（正值为抬头）、偏航舵偏δy（正值为向右偏航(与陀螺仪相反)）、滚转舵偏δr（正值为右滚转）。
//-90~90舵偏，舵面向上偏转为正
//  delta_pitch 为俯仰指令（正值通常表示抬头指令）；
//  delta_roll 为滚转指令（正值通常表示右滚指令）；
//  delta_yaw 为偏航指令（正值通常表示右偏航指令）。
void Rudder_c::CalDelta(float delta_yaw, float delta_pitch, float delta_roll)
{
    delta1 = -delta_roll + delta_pitch - delta_yaw;
    delta2 = +delta_roll + delta_pitch + delta_yaw;
    delta3 = +delta_roll + delta_pitch - delta_yaw;
    delta4 = -delta_roll + delta_pitch + delta_yaw;
}
// 测试用
// uint32_t i = 0;
// float angle = 135;
//     while (1)
//     {
//         if (i == 1)
//         {
//             wing[0]->SetAngle(angle);
//         }
//         else if (i == 2)
//         {
//             wing[1]->SetAngle(angle);
//         }
//         else if (i == 3)
//         {
//             wing[2]->SetAngle(angle);
//         }
//         else if (i == 4)
//         {
//             wing[3]->SetAngle(angle);
//         }
//     }
void Rudder_c::Calibrate()
{
    BSP_n::DWT_c *dwt = BSP_n::DWT_c::Get_DwtInstance();
    wing[0]->SetAngle(MIDDLE_ANGLE);
    wing[1]->SetAngle(MIDDLE_ANGLE);
    wing[2]->SetAngle(MIDDLE_ANGLE);
    wing[3]->SetAngle(MIDDLE_ANGLE);
    dwt->Delay_ms(700);
    wing[0]->SetAngle(MIM_ANGLE);
    wing[1]->SetAngle(MIM_ANGLE);
    wing[2]->SetAngle(MIM_ANGLE);
    wing[3]->SetAngle(MIM_ANGLE);
    dwt->Delay_ms(700);
    wing[0]->SetAngle(MAX_ANGLE);
    wing[1]->SetAngle(MAX_ANGLE);
    wing[2]->SetAngle(MAX_ANGLE);
    wing[3]->SetAngle(MAX_ANGLE);
    dwt->Delay_ms(700);
    wing[0]->SetAngle(MIDDLE_ANGLE);
    wing[1]->SetAngle(MIDDLE_ANGLE);
    wing[2]->SetAngle(MIDDLE_ANGLE);
    wing[3]->SetAngle(MIDDLE_ANGLE);
    dwt->Delay_ms(700);

}
void Rudder_c::SetDelta()
{

    delta1 = alg_n::user_val_limit(MIDDLE_ANGLE + delta1, MIM_ANGLE, MAX_ANGLE);
    
    wing[0]->SetAngle(delta1);
    delta2 = alg_n::user_val_limit(MIDDLE_ANGLE + delta2, MIM_ANGLE, MAX_ANGLE);
    wing[1]->SetAngle(delta2);
    delta3 = alg_n::user_val_limit(MIDDLE_ANGLE + delta3, MIM_ANGLE, MAX_ANGLE);
    wing[2]->SetAngle(delta3);
    delta4 = alg_n::user_val_limit(MIDDLE_ANGLE + delta4, MIM_ANGLE, MAX_ANGLE);
    wing[3]->SetAngle(delta4);
}

void Rudder_c::YawControllerInit(const float *rate, const float *pos)
{

    alg_n::PidInitConfig_t rate_config = {
        .Kp = -1,
        .Ki = 0,
        .Kd = 0,
        .D_T = 0.001,
        .Kfa = 0,
        .Kfb = 0,
        .ActualValueSource = rate,
        .mode =  alg_n::Integral_Limit,
        .max_out = 0,
        .max_Ierror = 0,
        .gama = 0,
        .deadband = 0,
        .threshold_max = 0,
        .threshold_min = 0,
        .errorabsmax = 0,
        .errorabsmin = -0,
        .d_filter_num = 0,
        .out_filter_num = 0,
        .stepIn = 0};
    PID_rate_y.Init(rate_config);
    alg_n::PidInitConfig_t pos_config =
        {
            .Kp = 1,
            .Ki = 0,
            .Kd = 0.5,
            .D_T = 0.005,
            .Kfa = 0,
            .Kfb = 0,
            .ActualValueSource = pos,
            .mode = alg_n::Output_Limit | alg_n::Integral_Limit,
            .max_out = 30,
            .max_Ierror = 0,
            .gama = 0,
            .deadband = 0,
            .threshold_max = 0,
            .threshold_min = 0,
            .errorabsmax = 0,
            .errorabsmin = -0,
            .d_filter_num = 0,
            .out_filter_num = 0,
            .stepIn = 0};
    PID_pos_y.Init(pos_config);
}
void Rudder_c::PitchControllerInit(const float *rate, const float *pos)
{
    alg_n::PidInitConfig_t rate_config = {
        .Kp = 1,
        .Ki = 0,
        .Kd = 0,
        .D_T = 0.001,
        .Kfa = 0,
        .Kfb = 0,
        .ActualValueSource = rate,
        .mode =  alg_n::Integral_Limit,
        .max_out = 0,
        .max_Ierror = 0,
        .gama = 0,
        .deadband = 0,
        .threshold_max = 0,
        .threshold_min = 0,
        .errorabsmax = 0,
        .errorabsmin = -0,
        .d_filter_num = 0,
        .out_filter_num = 0,
        .stepIn = 0};
    PID_rate_p.Init(rate_config);
    alg_n::PidInitConfig_t pos_config =
        {
            .Kp = 1,
            .Ki = 0,
            .Kd = 0.5,
            .D_T = 0.005,
            .Kfa = 0,
            .Kfb = 0,
            .ActualValueSource = pos,
            .mode = alg_n::Output_Limit | alg_n::Integral_Limit,
            .max_out = 30,
            .max_Ierror = 0,
            .gama = 0,
            .deadband = 0,
            .threshold_max = 0,
            .threshold_min = 0,
            .errorabsmax = 0,
            .errorabsmin = -0,
            .d_filter_num = 0,
            .out_filter_num = 0,
            .stepIn = 0};
    PID_pos_p.Init(pos_config);
}
void Rudder_c::RollControllerInit(const float *rate, const float *pos)
{
    alg_n::PidInitConfig_t rate_config = {
        .Kp = 1,
        .Ki = 0,
        .Kd = 0,
        .D_T = 0.001,
        .Kfa = 0,
        .Kfb = 0,
        .ActualValueSource = rate,
        .mode =  alg_n::Integral_Limit,
        .max_out = 0,
        .max_Ierror = 0,
        .gama = 0,
        .deadband = 0,
        .threshold_max = 0,
        .threshold_min = 0,
        .errorabsmax = 0,
        .errorabsmin = -0,
        .d_filter_num = 0,
        .out_filter_num = 0,
        .stepIn = 0};
    PID_rate_r.Init(rate_config);
    alg_n::PidInitConfig_t pos_config =
        {
            .Kp = 1,
            .Ki = 0,
            .Kd = 0.5,
            .D_T = 0.005,
            .Kfa = 0,
            .Kfb = 0,
            .ActualValueSource = pos,
            .mode = alg_n::Output_Limit | alg_n::Integral_Limit,
            .max_out = 30,
            .max_Ierror = 0,
            .gama = 0,
            .deadband = 0,
            .threshold_max = 0,
            .threshold_min = 0,
            .errorabsmax = 0,
            .errorabsmin = -0,
            .d_filter_num = 0,
            .out_filter_num = 0,
            .stepIn = 0};
    PID_pos_r.Init(pos_config);
}