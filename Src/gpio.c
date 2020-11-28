/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_RESET);

  #ifdef DRV8302
  HAL_GPIO_WritePin(EN_GATE_GPIO_Port,EN_GATE_Pin,GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(DC_CAL_GPIO_Port,DC_CAL_Pin,GPIO_PIN_RESET); 

  GPIO_InitStruct.Pin = DC_CAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DC_CAL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = EN_GATE_Pin; 
  HAL_GPIO_Init(EN_GATE_GPIO_Port, &GPIO_InitStruct); 


  
  #endif

  #ifdef ENCODER_SOFT_SPI
  HAL_GPIO_WritePin(ENCODER_CSN_GPIO_Port,ENCODER_CSN_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENCODER_MOSI_GPIO_Port,ENCODER_MOSI_Pin,GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(ENCODER_CLK_GPIO_Port,ENCODER_CLK_Pin,GPIO_PIN_RESET); 

  GPIO_InitStruct.Pin = ENCODER_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENCODER_MOSI_GPIO_Port, &GPIO_InitStruct); 

  GPIO_InitStruct.Pin = ENCODER_CSN_Pin; 
  HAL_GPIO_Init(ENCODER_CSN_GPIO_Port, &GPIO_InitStruct); 
  
  GPIO_InitStruct.Pin = ENCODER_CLK_Pin; 
  HAL_GPIO_Init(ENCODER_CLK_GPIO_Port, &GPIO_InitStruct);  

  GPIO_InitStruct.Pin = ENCODER_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENCODER_MISO_GPIO_Port, &GPIO_InitStruct);

  #endif

  #ifdef CAMERA_SUPPORT
  HAL_GPIO_WritePin(CAMERA_EXP_GPIO_Port,CAMERA_EXP_PIN,GPIO_PIN_RESET); 

  GPIO_InitStruct.Pin = CAMERA_EXP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAMERA_EXP_GPIO_Port, &GPIO_InitStruct); 
  #endif
  //HAL_GPIO_WritePin(GPIOC, ENCODER_MOSI_Pin|LED_GREEN_Pin|LED_RED_Pin|EN_GATE_Pin 
  //                        |ENCODER_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(GPIOB, DC_CAL_Pin|ENCODER_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin 
                           PCPin */
                           /*
  GPIO_InitStruct.Pin = ENCODER_MOSI_Pin|LED_GREEN_Pin|LED_RED_Pin|EN_GATE_Pin 
                          |ENCODER_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);*/

  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct); 

  GPIO_InitStruct.Pin = ENCODER_CSN_Pin; 
  HAL_GPIO_Init(ENCODER_CSN_GPIO_Port, &GPIO_InitStruct); 


  /*Configure GPIO pins : PBPin PBPin */
  /*
  GPIO_InitStruct.Pin = DC_CAL_Pin|ENCODER_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  */

  /*Configure GPIO pins : PBPin PBPin */
  /*
  GPIO_InitStruct.Pin = FAULT_Pin|ENCODER_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
*/
}

/* USER CODE BEGIN 2 */
/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
