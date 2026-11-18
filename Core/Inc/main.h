/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define enable_pin_Pin GPIO_PIN_0
#define enable_pin_GPIO_Port GPIOA
#define dir1_Pin GPIO_PIN_4
#define dir1_GPIO_Port GPIOA
#define dir2_Pin GPIO_PIN_5
#define dir2_GPIO_Port GPIOA
#define gimbal_servo_pwm_Pin GPIO_PIN_6
#define gimbal_servo_pwm_GPIO_Port GPIOA
#define gripper_motor_pwm_Pin GPIO_PIN_7
#define gripper_motor_pwm_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_12
#define DIR_GPIO_Port GPIOB
#define ENA_Pin GPIO_PIN_15
#define ENA_GPIO_Port GPIOB
#define PUL_Pin GPIO_PIN_10
#define PUL_GPIO_Port GPIOA
#define encoderA_Pin GPIO_PIN_15
#define encoderA_GPIO_Port GPIOA
#define encoderB_Pin GPIO_PIN_3
#define encoderB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
