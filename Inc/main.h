/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENCODER_MOSI_Pin GPIO_PIN_0
#define ENCODER_MOSI_GPIO_Port GPIOC
#define SENSE3_Pin GPIO_PIN_0
#define SENSE3_GPIO_Port GPIOA
#define SENSE2_Pin GPIO_PIN_1
#define SENSE2_GPIO_Port GPIOA
#define SENSE1_Pin GPIO_PIN_2
#define SENSE1_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_4
#define LED_GREEN_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_5
#define LED_RED_GPIO_Port GPIOC
#define BR_SO2_Pin GPIO_PIN_0
#define BR_SO2_GPIO_Port GPIOB
#define BR_SO1_Pin GPIO_PIN_1
#define BR_SO1_GPIO_Port GPIOB
#define DC_CAL_Pin GPIO_PIN_12
#define DC_CAL_GPIO_Port GPIOB
#define EN_GATE_Pin GPIO_PIN_10
#define EN_GATE_GPIO_Port GPIOC
#define ENCODER_CSN_Pin GPIO_PIN_11
#define ENCODER_CSN_GPIO_Port GPIOC
#define FAULT_Pin GPIO_PIN_4
#define FAULT_GPIO_Port GPIOB
#define ENCODER_CLK_Pin GPIO_PIN_6
#define ENCODER_CLK_GPIO_Port GPIOB
#define ENCODER_MISO_Pin GPIO_PIN_7
#define ENCODER_MISO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define FOC_FREQ  8000
#define TIM7_FREQ 1000
#define SPEED_FREQ 4000
#define POSITION_FREQ 4000
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
