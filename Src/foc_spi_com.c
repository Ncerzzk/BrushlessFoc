#include "foc_spi_com.h"
#include "spi.h"

uint8_t SPI_Slave_Test=55;
uint16_t SPI_Slave_Test2=100;
float SPI_Slave_Test3=3.14f;

extern uint8_t FOC_Flag;
extern float Base_Duty;
extern float Phi;
extern float Duty_Amp;
extern uint8_t buffer_rx_OK;
extern uint8_t buffer_rx[];
extern float Position_Degree;
extern float Position_Offset;

// 以下协议废弃，在发送速度较快的情况下，会以较高频率出现通信异常
SPI_COM_VAL SPI_Com_Val_List[]={
  {0,(uint32_t *)&SPI_Slave_Test,1},
  {1,(uint32_t *)&SPI_Slave_Test2,2},
  {2,(uint32_t *)&SPI_Slave_Test3,sizeof(SPI_Slave_Test3)},
  {3,(uint32_t *)&FOC_Flag,sizeof(FOC_Flag)},
  {4,(uint32_t *)&Base_Duty,sizeof(Base_Duty)},
  {5,(uint32_t *)&Phi,sizeof(Phi)},
  {6,(uint32_t *)&Duty_Amp,sizeof(Duty_Amp)},
  {7,(uint32_t *)&buffer_rx_OK,sizeof(buffer_rx_OK)},
  {8,(uint32_t *)buffer_rx,30},
  {9,(uint32_t *)&Position_Degree,sizeof(Position_Degree)},
  {10,(uint32_t *)&Position_Offset,sizeof(Position_Offset)}
};


void SPI_Com_Init(){
  SPI_Slave_Init(&hspi1,sizeof(SPI_Com_Val_List)/sizeof(SPI_COM_VAL));
}


