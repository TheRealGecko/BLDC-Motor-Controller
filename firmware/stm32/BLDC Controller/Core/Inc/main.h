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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GATE_SD_Pin GPIO_PIN_3
#define GATE_SD_GPIO_Port GPIOA
#define SPEED_CMD_Pin GPIO_PIN_4
#define SPEED_CMD_GPIO_Port GPIOA
#define LIN_A_Pin GPIO_PIN_7
#define LIN_A_GPIO_Port GPIOA
#define LIN_B_Pin GPIO_PIN_0
#define LIN_B_GPIO_Port GPIOB
#define LIN_C_Pin GPIO_PIN_1
#define LIN_C_GPIO_Port GPIOB
#define HIN_A_Pin GPIO_PIN_8
#define HIN_A_GPIO_Port GPIOA
#define HIN_B_Pin GPIO_PIN_9
#define HIN_B_GPIO_Port GPIOA
#define HIN_C_Pin GPIO_PIN_10
#define HIN_C_GPIO_Port GPIOA
#define BEMF_A_Pin GPIO_PIN_5
#define BEMF_A_GPIO_Port GPIOB
#define BEMF_A_EXTI_IRQn EXTI9_5_IRQn
#define BEMF_B_Pin GPIO_PIN_6
#define BEMF_B_GPIO_Port GPIOB
#define BEMF_B_EXTI_IRQn EXTI9_5_IRQn
#define BEMF_C_Pin GPIO_PIN_7
#define BEMF_C_GPIO_Port GPIOB
#define BEMF_C_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
