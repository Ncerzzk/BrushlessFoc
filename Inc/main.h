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

#define LED_RED_Pin GPIO_PIN_5
#define LED_RED_GPIO_Port GPIOC
#define BR_SO2_Pin GPIO_PIN_0
#define BR_SO2_GPIO_Port GPIOB
#define BR_SO1_Pin GPIO_PIN_1
#define BR_SO1_GPIO_Port GPIOB



#define FAULT_Pin GPIO_PIN_4
#define FAULT_GPIO_Port GPIOB
#define ENCODER_CLK_Pin GPIO_PIN_6
#define ENCODER_CLK_GPIO_Port GPIOB
#define ENCODER_MISO_Pin GPIO_PIN_7
#define ENCODER_MISO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define ADCVAL_TO_VOLTAGE(x) (x/4096.0f*3.3f)

//       BOARD CONFIG


#ifdef BENJAMIN

#define DRV8302
#define ENCODER_SOFT_SPI

#define VOLOTAGE_ADC_CHANNEL  ADC_CHANNEL_12
#define C_CURRENT_ADC_CHANNEL ADC_CHANNEL_8
#define A_CURRENT_ADC_CHANNEL ADC_CHANNEL_9

#define LED_GREEN_Pin GPIO_PIN_4
#define LED_GREEN_GPIO_Port GPIOC

#define ENCODER_CSN_Pin GPIO_PIN_11
#define ENCODER_CSN_GPIO_Port GPIOC


#define VOLOTAGE_ADC_GPIO_PIN  GPIO_PIN_2
#define VOLTAGE_ADC_GPIO_PORT GPIOC
#define C_CURRENT_ADC_GPIO_PIN GPIO_PIN_0 
#define C_CURRENT_ADC_GPIO_PORT GPIOB
#define A_CURRENT_ADC_GPIO_PIN GPIO_PIN_1 
#define A_CURRENT_ADC_GPIO_PORT GPIOB 

#define EN_GATE_Pin GPIO_PIN_10
#define EN_GATE_GPIO_Port GPIOC
#define DC_CAL_Pin GPIO_PIN_12
#define DC_CAL_GPIO_Port GPIOB

#define GAIN    40.0f
#define SHUNT_RES     0.003f

#define VOLTAGE_RES1    39000.0f
#define VOLTAGE_RES2    2200.0f

#define GET_SHUNT_VOLTAGE(Vref,Vo)  ADCVAL_TO_VOLTAGE((Vref-Vo))/GAIN

#define CCRA  CCR1
#define CCRB  CCR2
#define CCRC  CCR3

#define CAMERA_SUPPORT

#ifdef CAMERA_SUPPORT

#define CAMERA_EXP_PIN  GPIO_PIN_6
#define CAMERA_EXP_GPIO_Port GPIOA

#endif

#endif


#ifdef MYOWN_FOC

#define FD6288

#define DAC_AS_VREF
//#define USE_GYRO

#define LED_GREEN_Pin GPIO_PIN_9
#define LED_GREEN_GPIO_Port GPIOC

#define VOLOTAGE_ADC_CHANNEL  ADC_CHANNEL_10
#define C_CURRENT_ADC_CHANNEL ADC_CHANNEL_11
#define A_CURRENT_ADC_CHANNEL ADC_CHANNEL_12

#define ENCODER_CSN_Pin GPIO_PIN_6
#define ENCODER_CSN_GPIO_Port GPIOB


#define VOLOTAGE_ADC_GPIO_PIN  GPIO_PIN_0
#define VOLTAGE_ADC_GPIO_PORT GPIOC
#define C_CURRENT_ADC_GPIO_PIN GPIO_PIN_1
#define C_CURRENT_ADC_GPIO_PORT GPIOC
#define A_CURRENT_ADC_GPIO_PIN GPIO_PIN_2 
#define A_CURRENT_ADC_GPIO_PORT GPIOC 

#define GAIN    50.0f
#define SHUNT_RES     0.003f

#define VOLTAGE_RES1    100000.0f
#define VOLTAGE_RES2    10000.0f

#define GET_SHUNT_VOLTAGE(Vref,Vo)  ADCVAL_TO_VOLTAGE((Vo-Vref))/GAIN

#define CCRA  CCR1
#define CCRB  CCR2
#define CCRC  CCR3

#define SOFT_I2C_SCL_PIN  GPIO_PIN_5
#define SOFT_I2C_SCL_PORT GPIOA
#define SOFT_I2C_SDA_PIN  GPIO_PIN_6
#define SOFT_I2C_SDA_PORT GPIOA


#endif




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
