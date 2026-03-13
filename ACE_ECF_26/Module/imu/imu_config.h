/********加热器参数********** */
/*
 * @warning  此处参数只适合ACE 26.02.01 F4主控，其余主控需要重新调教加热器参数
*/
//PID参数
#define Heater_Kp 10.f
#define Heater_Ki 0.1f
#define Heater_Kd 0.f
#define Heater_maxIerror 40.f

#define Heater_forward 15//加热器前馈值
#define Init_temp_if_high 45.f//初始化前温度
#define Target_temp_if_high 50.f//目标温度
#define Init_temp_if_low 35.f
#define Target_temp_if_low 40.f
#define Max_temp 70.f//最高温度
#define Preheat_Timeout 45*1000//预热超时时间，单位ms
//TIM通道
#define IMU_HEATER_TIM &htim15
#define IMU_HEATER_TIM_CHANNEL TIM_CHANNEL_2
/************************************ */
//spi
#define BMI088_SPI &hspi1
//
#define isHeaterEnabled 1 //是否启用加热器
#define isCalibrate 1 //是否启用在线校准
/***********离线校准数据*************** */
// 需手动修改
#define GxOFFSET -0.0f
#define GyOFFSET -0.0f
#define GzOFFSET 0.0f
#define gNORM 9.6f
