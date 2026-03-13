namespace MM = Motor_n::MotorBaseDef_n;
Motor_n::DjiMotor_n::DjiDriver_c * motor;
Motor_n::MotorBaseDef_n::MotorBase_c * motorbase;

// 初始化开始，关中断
__disable_irq();
void init()
{

    MM::Motor_Base_Config_t text_config = MM::Motor_Base_Config_t("WheelMotor", MM::Motor_Type_euc::M3508)
    .SetControlSetting(MM::Motor_Control_Setting_t{MM::Closeloop_Type_euc::ANGLE_AND_SPEED_LOOP})
    .SetPIDConfig( // 这个部分一定要三个PID结构体都设置，且按顺序设置
      alg_n::PidInitConfig_t{  // Angle PID
          .Kp = 0.1f,
          .Ki = 0.0f,
          .Kd = 0.0f,
          .mode = Output_Limit | Integral_Limit | StepIn,
          .max_out = 8192,
          .max_Ierror = 220000,
          .deadband = 0,
          .stepIn = 10,
      },
      alg_n::PidInitConfig_t{  // Speed PID
          .Kp = 1.0f,
          .Ki = 0.0f,
          .Kd = 0.0f,
          .mode = Output_Limit | Integral_Limit,
          .max_out = Current_limit_H_3508,
          .max_Ierror = 200,
          .deadband = 0,
          .errorabsmax = 0,
          .errorabsmin = 0,
      },
      alg_n::PidInitConfig_t{  // Current PID
        .Kp = 0.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .Kfa = 0.0f,
        .Kfb = 0.0f,
        .ActualValueSource = nullptr})
    .SetCANConfig(BSP_n::CanInitConfig_s{
      .can_handle = &hcan1,
      .tx_id = 3,  // Slave ID
        })
    .SetMechanicalParams(MM::Motor_Data_t::Motor_Fixed_Param_t{
      .zero_offset = 0.0f,
      .radius = 0.05f,
      .ecd2length = 0.0f,
      .ratio = 1.0f})
    .SetOutputLimit(Current_limit_H_3508, Current_limit_L_3508);

    motor =  new Motor_n::DjiMotor_n::DjiDriver_c(text_config);
    //motor->motor_controller_.SetSpeedFeedbackPtr(&motor->motor_data_.motor_raw_data.feedback_speed);//记得放在构造函数之后或者构造函数里面
    //motor->motor_controller_.SetAngleFeedbackPtr(&motor->motor_data_.motor_raw_data.total_ecd);
    //反馈值设置在初始化就有
    motorbase = &motor->get_base();

}
// 初始化完成,开启中断
__enable_irq();

// 主循环
while(1)
{

    motor->Enable();
    motor->DjiMotorSetRef(1.0f);
    Motor_n::DjiMotor_n::DjiMotorControl();
    Motor_n::DjiMotor_n::set_GiveCurrent();
    Motor_n::DjiMotor_n::MotorTransmit();

}

// 开环输出可以直接配置open_loop，然后set_ref即可，也可以使用SetMotorOutputFix函数
