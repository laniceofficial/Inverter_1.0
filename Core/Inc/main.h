/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ASK_Pin GPIO_PIN_5
#define ASK_GPIO_Port GPIOC
#define CHAR_I_Pin GPIO_PIN_0
#define CHAR_I_GPIO_Port GPIOB
#define CHAR_V_Pin GPIO_PIN_13
#define CHAR_V_GPIO_Port GPIOB
#define OUT_LOWPOWER_Pin GPIO_PIN_15
#define OUT_LOWPOWER_GPIO_Port GPIOA
#define OUT_LOWPOWER_EXTI_IRQn EXTI15_10_IRQn
#define FCHAN_Pin GPIO_PIN_10
#define FCHAN_GPIO_Port GPIOC
#define FCHAN_EXTI_IRQn EXTI15_10_IRQn
#define F100HZ_Pin GPIO_PIN_11
#define F100HZ_GPIO_Port GPIOC
#define F100HZ_EXTI_IRQn EXTI15_10_IRQn
#define F1KHZ_Pin GPIO_PIN_12
#define F1KHZ_GPIO_Port GPIOC
#define F1KHZ_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
