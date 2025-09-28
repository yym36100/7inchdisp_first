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
#include "stm32h7xx_hal.h"

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
#define lcd_rst_Pin GPIO_PIN_2
#define lcd_rst_GPIO_Port GPIOE
#define gt_int_Pin GPIO_PIN_3
#define gt_int_GPIO_Port GPIOE
#define led_Pin GPIO_PIN_13
#define led_GPIO_Port GPIOC
#define GT_RST_Pin GPIO_PIN_7
#define GT_RST_GPIO_Port GPIOH
#define cam_pwdn_Pin GPIO_PIN_14
#define cam_pwdn_GPIO_Port GPIOH
#define cam_reset_Pin GPIO_PIN_3
#define cam_reset_GPIO_Port GPIOI
#define cam_xclock_Pin GPIO_PIN_15
#define cam_xclock_GPIO_Port GPIOA
#define dbg1_Pin GPIO_PIN_8
#define dbg1_GPIO_Port GPIOB
#define dbg2_Pin GPIO_PIN_9
#define dbg2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
