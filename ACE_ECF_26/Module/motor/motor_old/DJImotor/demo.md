// 初始化开始，关中断
__disable_irq();
// 只有速度环
Motor_General_Def_n::Motor_Init_Config_s config = {
    .controller_param_init_config = {
      .speed_PID = {
        .Kp = 2.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .mode = Output_Limit,
        .max_out = 9000,
        .deadband = 0,
      },
    },
    .controller_setting_init_config = { 
      .outer_loop_type       =  Motor_General_Def_n::SPEED_LOOP,
      .close_loop_type       =  Motor_General_Def_n::SPEED_LOOP,
      .motor_reverse_flag    =  Motor_General_Def_n::MOTOR_DIRECTION_NORMAL,
      .feedback_reverse_flag =  Motor_General_Def_n::FEEDBACK_DIRECTION_NORMAL,
      // 反馈来源可设置为 MOTOR_FEED OTHER_FEED SPEED_APS ANGULAR_SPEED LINEAR_SPEED
      .speed_feedback_source =  Motor_General_Def_n::MOTOR_FEED,
    },
    .motor_type = Motor_General_Def_n::M3508,
    .can_init_config = { 
      .can_handle = &hcan1,
      .tx_id = 4,// 看电调闪几下就填几
    }
};
// 只有位置环
Motor_General_Def_n::Motor_Init_Config_s config_cn = {
    .controller_param_init_config = {
      .angle_PID = {
        .Kp = 2.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .mode = Output_Limit,
        .max_out = 9000,
        .deadband = 0.3,
      },
    },
    .controller_setting_init_config = { 
      .outer_loop_type       =  Motor_General_Def_n::ANGLE_LOOP,
      .close_loop_type       =  Motor_General_Def_n::ANGLE_LOOP,
      .motor_reverse_flag    =  Motor_General_Def_n::MOTOR_DIRECTION_NORMAL,
      .feedback_reverse_flag =  Motor_General_Def_n::FEEDBACK_DIRECTION_NORMAL,
      // 反馈来源可设置为 MOTOR_FEED OTHER_FEED RECORD_LENGTH 
      .angle_feedback_source =  Motor_General_Def_n::MOTOR_FEED,
    },
    .motor_type = Motor_General_Def_n::M3508,
    .can_init_config = { 
      .can_handle = &hcan1,
      .tx_id = 1,// 看电调闪几下就填几
    }
};

DJI_Motor_n::DJI_Motor_Instance dji_motor1(config);
DJI_Motor_n::DJI_Motor_Instance dji_motor2(config_cn);

dji_motor1.DJIMotorEnable();//先使能再设置数值
dji_motor2.DJIMotorEnable();
dji_motor1.DJIMotorSetRef(1000);
dji_motor2.DJIMotorSetRef(1000);
// 初始化完成,开启中断
__enable_irq();