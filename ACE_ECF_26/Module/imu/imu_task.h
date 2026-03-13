#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
#include "BMI088driver.hpp"
void imu_Init();
BMI088Heat_c *getImuPtr();
extern "C" {
#endif

void IMU_Task(void const * argument);
void IMU_Heat_Task(void const * argument);

#ifdef __cplusplus
}
#endif



#endif // IMU_TASK_H
