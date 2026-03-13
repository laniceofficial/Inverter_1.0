#include "imu_task.h"
#include "Alg_PID.hpp"
#include "BMI088driver.hpp"
#include "imu_config.h"
#include "main.h"
#include <cstdint>

extern "C"
{
#include "imu_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "spi.h"
#include "tim.h"
#include "cmsis_os.h"
#include "usart.h"
#include <cstdio>
}

const INS_t *INS = nullptr;
BMI088Heat_c *imu = nullptr;
/*
 * @brief 返回陀螺仪类
 */
BMI088Heat_c *getImuPtr()
{
    return imu;
}

void IMU_Task(void const *argument)
{
    imu_Init();

    TickType_t xLastWakeTime;
    const TickType_t xDelay1ms = pdMS_TO_TICKS(1);
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        taskENTER_CRITICAL();
        BMI088_c::BMI_UpData();
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&xLastWakeTime, xDelay1ms);
    }
}

void IMU_Heat_Task(void const *argument)
{

    imu->timInit();
    imu->preHeat();
    while (1)
    {
        imu->heatLoop();
        vTaskDelay(10);
    }
}

void imu_Init()
{
    alg_n::PidInitConfig_t pid_config =
        {
            .Kp = Heater_Kp, // 5
            .Ki = Heater_Ki,
            .Kd = Heater_Kd,
            .D_T = 0.01f,
            .mode = alg_n::Output_Limit | alg_n::Integral_Limit | alg_n::OutputFilter,
            .max_out = 100,
            .max_Ierror = Heater_maxIerror,
            .out_filter_num = 0.1f};
    IMU_Heat_Config_s_t _configP =
        {
            IMU_HEATER_TIM,
            IMU_HEATER_TIM_CHANNEL,
            Max_temp,
            Init_temp_if_low, // 55
            Target_temp_if_low,
            Preheat_Timeout,
            &pid_config,
            Heater_forward // 50
        };
    imu = new BMI088Heat_c(&_configP);
    // 配置imu接口
    imu->physicalInit(BMI088_SPI, isCalibrate, ACC_CS0_GPIO_Port, ACC_CS0_Pin, GYRO_CS1_GPIO_Port, GYRO_CS1_Pin);

// 加热相关，判断是否拥有加热器
#if isHeaterEnabled
    // 创建加热任务
    osThreadId imuHeatTask;
    osThreadDef(imuHeatTask, IMU_Heat_Task, osPriorityNormal, 0, 128);
    imuHeatTask = osThreadCreate(osThread(imuHeatTask), NULL);
    // 等待加热完成
    while (imu->getHeatingStatus() != IMU_Heat_OK)
    {
        if (imu->getHeatingStatus() == IMU_Heat_PreHeatFail)
        {
            break;
        }
        osDelay(10);
    }

#endif
    imu->stopheat();
    imu->CalibrateMpuOffset(); // 标定零飘，初始化卡尔曼
    imu->init_flag = 1;
    INS = imu->Get_INS_Data_Point();
}
