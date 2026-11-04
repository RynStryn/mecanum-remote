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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define encoder_lr_Pin GPIO_PIN_0
#define encoder_lr_GPIO_Port GPIOA
#define encoder_lr2_Pin GPIO_PIN_1
#define encoder_lr2_GPIO_Port GPIOA
#define from_rmtctrl_tx_Pin GPIO_PIN_2
#define from_rmtctrl_tx_GPIO_Port GPIOA
#define from_rmtctrl_rx_Pin GPIO_PIN_3
#define from_rmtctrl_rx_GPIO_Port GPIOA
#define encoder_rf_Pin GPIO_PIN_6
#define encoder_rf_GPIO_Port GPIOA
#define encoder_rf2_Pin GPIO_PIN_7
#define encoder_rf2_GPIO_Port GPIOA
#define to_arm_tx_Pin GPIO_PIN_10
#define to_arm_tx_GPIO_Port GPIOB
#define to_arm_rx_Pin GPIO_PIN_11
#define to_arm_rx_GPIO_Port GPIOB
#define motor_lf_Pin GPIO_PIN_6
#define motor_lf_GPIO_Port GPIOC
#define motor_lr_Pin GPIO_PIN_7
#define motor_lr_GPIO_Port GPIOC
#define motor_rf_Pin GPIO_PIN_8
#define motor_rf_GPIO_Port GPIOC
#define motor_rr_Pin GPIO_PIN_9
#define motor_rr_GPIO_Port GPIOC
#define encoder_lf_Pin GPIO_PIN_8
#define encoder_lf_GPIO_Port GPIOA
#define encoder_lf2_Pin GPIO_PIN_9
#define encoder_lf2_GPIO_Port GPIOA
#define encoder_rr_Pin GPIO_PIN_6
#define encoder_rr_GPIO_Port GPIOB
#define encoder_rr2_Pin GPIO_PIN_7
#define encoder_rr2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
