/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_ext.h"
#include "encoder.h"
#include "as5047.h"
#include "foc.h"
#include "parameter.h"
#include "dac.h"
#include "utils.h"

#ifdef AS_SPI_SLAVE
#include "spi_slave.h"
#include "foc_spi_com.h"
#endif

#ifdef USE_GYRO
#include "soft_i2c.h"
#include "icm20600.h"
#include "easy_angle.h"

void Angle_Control();
#endif
//#include "sing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern uint32_t cnt;
extern const uint8_t voice[];
uint8_t Init_OK=0;
uint8_t Ms_5_Flag=0;
void Angle_Handler();


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint16_t adc_value[2]={0};
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
#ifdef DAC_AS_VREF
  MX_DAC_Init();
#endif
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_UART5_Init();

  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  debug_uart_init(&huart5,DMA,DMA);
  uprintf_polling("hello,world!\r\n");
  Read_Parameters(1);

  #ifdef ENCODER_SOFT_SPI
  As5047_Init(0,ENCODER_CSN_GPIO_Port,ENCODER_CSN_Pin);
  #else
  MX_SPI3_Init();
  As5047_Init(&hspi3,ENCODER_CSN_GPIO_Port,ENCODER_CSN_Pin); 
  #endif
  Foc_Init();


  #ifdef USE_GYRO
  Soft_I2C_Init(SOFT_I2C_SCL_PORT,SOFT_I2C_SCL_PIN,SOFT_I2C_SDA_PORT,SOFT_I2C_SDA_PIN);
  MPU9250_Init(&MPU9250); 
  #endif

  #ifdef AS_SPI_SLAVE
  MX_SPI1_Init();
  //SPI_Slave_Init(&hspi1,11);
  SPI_Com_Init();
  #endif

  //Song_Init(&htim7,5000,0.3f,voice,297973);

  uprintf_polling("hello,world2!\r\n");
  Init_OK=1;

  //

  //;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    if(buffer_rx_OK){
      UART_Command_Analize_And_Call();
    }
    
#ifdef USE_GYRO
    if(Ms_5_Flag){
      Ms_5_Flag=0;
      Angle_Handler(); 
      Angle_Control();
    }
#endif
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#ifdef USE_GYRO
#include "math.h"
void Ms_Handler(){
  static uint32_t ms_cnt;

  if(!Init_OK){
    return ;
  }

  ms_cnt++;
  if(ms_cnt%5==0){
    Ms_5_Flag=1;
  }

  if(ms_cnt==50000){
    ms_cnt=0;
  }
}

float angle[3]={0};
void Angle_Handler(){
  int16_t gy[3],ac[3];
  float angle_speed[3];
  float ac_angle[3];
  MPU_Read6500(&MPU9250,ac,gy);
  Gyroraw_to_Angle_Speed(&MPU9250,gy,angle_speed);
  get_angle(ac,angle_speed,angle,ac_angle);
  //send_wave(angle[0],angle[1],angle[2],ac_angle[1]);
}

PID_S Angle_PID ={
    .KP=0.02,
    .KD=0,
    .KI=0,
    .I_TIME=0.005f,
    .i_max = __FLT_MAX__,
    .I_ERR_LIMIT = 5.0f
};
// angle[1] 往前低头>90

// phi = 0 时，力矩向前低头
float Target_Angle=90;
void Angle_Control(){
  float out=0;
  float abs_out=0;
  out=PID_Control(&Angle_PID,Target_Angle,angle[1]);
  // 当往前低头，angle[1]=120
  // 此时 target - now 为 负数
  // PID值为正数
  // 则out = 负数
  if(out>0){
    Phi = 0;
  }else{
    Phi = 180;
  }
  abs_out=fabsf(out);
  if(abs_out>0.5f){
    abs_out=0.5f;
  }
  Duty_Amp =Base_Duty* abs_out;
  if(Wave_Flag)
    send_wave(angle[1],0,Duty_Amp*1000,0);
}
#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
