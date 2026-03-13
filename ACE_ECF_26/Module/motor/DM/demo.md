float ctrl_mit_data[5] = {0,0,0,0,0};
float target_angle = 0; // rad
float out = 0;
Motor_n::MotorBaseDef_n::MotorBase_c * motorbase;
namespace MM = Motor_n::MotorBaseDef_n;
Motor_n::DmMotor_n::DmDriver_c * motor;

// 初始化开始，关中断
__disable_irq();
void init()
{

    Motor_n::MotorBaseDef_n::Motor_Base_Config_t text_config = MM::Motor_Base_Config_t("WheelMotor", MM::Motor_Type_euc::DM8009)
    .SetControlSetting(MM::Motor_Control_Setting_t{MM::Closeloop_Type_euc::ANGLE_AND_SPEED_LOOP})
    .SetPIDConfig(// 这个部分一定要三个PID结构体都设置，且按顺序设置
        alg_n::PidInitConfig_t{  // Angle PID
        .Kp = 1.f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .Kfa = 0.0f,
        .Kfb = 0.0f,
        .ActualValueSource = nullptr,
        .mode = Output_Limit,
        .max_out = 30.0f,
    },
    alg_n::PidInitConfig_t{  // Speed PID
        .Kp = 0.1f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .Kfa = 0.0f,
        .Kfb = 0.0f,
        .ActualValueSource = nullptr,
        .mode = Output_Limit,
        .max_out = 3.0f,
    },
    alg_n::PidInitConfig_t{  // Current PID
        .Kp = 0.2f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .Kfa = 0.0f,
        .Kfb = 0.0f,
        .ActualValueSource = nullptr})
    .SetCANConfig(BSP_n::CanInitConfig_s{
        .can_handle = &hcan2,
        .tx_id = 0x4C,  // Slave ID
        .rx_id = 0x4D,  // Master ID
        .can_module_callback = nullptr,
        .SAND_IDE = CAN_ID_STD})
    .SetMechanicalParams(MM::Motor_Data_t::Motor_Fixed_Param_t{
        .zero_offset = 0.0f,
        .radius = 0.05f,
        .ecd2length = 0.0f,
        .ratio = 1.0f})
    .SetOutputLimit(4.0f, -4.0f);

    Motor_n::DmMotor_n::DmDriver_c::DM_ModePrame_s pitch_congfig_ = {
        .kp_min = 0,    .kp_max = 500,
        .kd_min = 0,    .kd_max = 5,    // 这两项固定不能修改
        .v_min = -45,   .v_max = 45,
        .p_min = -12.5, .p_max = 12.5,
        .t_min = -10,   .t_max = 10,    // 这三项必须与上位机软件参数一致
    };

    motor = new Motor_n::DmMotor_n::DmDriver_c(text_config,pitch_congfig_);
    motor->motor_controller_.SetSpeedFeedbackPtr(&motor->motor_data_.motor_raw_data.feedback_speed);//记得放在构造函数之后或者构造函数里面
    motor->motor_controller_.SetAngleFeedbackPtr(&motor->motor_data_.motor_raw_data.position);
    motor->SetZero();
    float t = 0;

    motorbase = &motor->get_base();

}
// 初始化完成,开启中断
__enable_irq();

// 主循环
while(1)
{

    static float t;
    target_angle = 3.0f*sin(t);
    out = motor->PIDCalculate(&target_angle);
    //motor->SetMotorOutputFix(out);
    motor->SetMITData(0,0,0,0,out);
    motor->DmTransmit(1.0f, Motor_n::DmMotor_n::DmDriver_c::DM_CMD_MOTOR_MODE);   // 函数里面可以配置降频操作，失能使能

    t+=0.01f;
    HAL_Delay(1);

}