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
#include "stm32l0xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define X_STEP_Pin GPIO_PIN_0
#define X_STEP_GPIO_Port GPIOA
#define Y_STEP_Pin GPIO_PIN_1
#define Y_STEP_GPIO_Port GPIOA
#define Z_STEP_Pin GPIO_PIN_2
#define Z_STEP_GPIO_Port GPIOA
#define X_DIR_Pin GPIO_PIN_3
#define X_DIR_GPIO_Port GPIOA
#define Y_DIR_Pin GPIO_PIN_4
#define Y_DIR_GPIO_Port GPIOA
#define Z_DIR_Pin GPIO_PIN_5
#define Z_DIR_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_0
#define EN_GPIO_Port GPIOB
#define A_DIR_Pin GPIO_PIN_1
#define A_DIR_GPIO_Port GPIOB
#define Y_DIRB10_Pin GPIO_PIN_10
#define Y_DIRB10_GPIO_Port GPIOB
#define A_STEP_Pin GPIO_PIN_11
#define A_STEP_GPIO_Port GPIOB
#define ENA9_Pin GPIO_PIN_9
#define ENA9_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define X_STEP_Pin GPIO_PIN_0
#define X_STEP_Port GPIOA
#define Y_STEP_Pin GPIO_PIN_1
#define Y_STEP_Port GPIOA
#define Z_STEP_Pin GPIO_PIN_2
#define Z_STEP_Port GPIOA
#define A_STEP_Pin GPIO_PIN_11
#define A_STEP_Port GPIOB

#define X_DIR_Pin GPIO_PIN_3
#define X_DIR_Port GPIOA
#define Y_DIR_Pin GPIO_PIN_4
#define Y_DIR_Port GPIOA
#define Z_DIR_Pin GPIO_PIN_5
#define Z_DIR_Port GPIOA
#define EN_Pin GPIO_PIN_0
#define EN_Port GPIOB
#define A_DIR_Pin GPIO_PIN_1
#define A_DIR_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
