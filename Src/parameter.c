#include "parameter.h"
#include "foc.h"
#include "pid.h"

#include "uart_ext.h"

#define PARAMETER_FLASH_ADDRESS 0x08040000   // 扇区6 大小128kb
#define PARAMETER_FLASH_ADDRESS_SECLECTOR FLASH_SECTOR_6
extern uint8_t Board_CAN_ID;

typedef struct 
{
    Motor_Parameter motor_info;
    float position_offset;
    uint8_t ID;
}Flash_Parameters;

#define STR(x)      #x

inline void Print_PID(char *name,PID_S * pid){
    uprintf_polling("%s KP:%f,KI:%f,KD:%f\r\n",name,pid->KP,pid->KI,pid->KD);
}

void Print_Parameters(Flash_Parameters *p){
    uprintf_polling("Motor paris:%d \r\n",p->motor_info.Pairs);
    uprintf_polling("Encoder offset:%f \r\n",p->motor_info.Position_Phase_Offset);
    uprintf_polling("Encoder direction:%d \r\n ",p->motor_info.Encoder_Direction);
    //Print_PID(STR(Id_PID),&(p->id_pid));
    //Print_PID(STR(Iq_PID),&(p->iq_pid));
    uprintf_polling("Position Offset:%f\r\n",p->position_offset);
    uprintf_polling("CAN_ID:%d\r\n",p->ID);
}

void Read_Parameters(uint8_t write_default){


    Flash_Parameters temp = *(Flash_Parameters *)PARAMETER_FLASH_ADDRESS;
    INIT:
    if(temp.motor_info.Pairs==0xFF){
        uprintf_polling("First time used, use default configuration!\r\n");
        if(write_default){
            Write_Parameter();
            temp = *(Flash_Parameters *)PARAMETER_FLASH_ADDRESS; 
            goto INIT;
        }
    }
    Print_Parameters(&temp);

    Motor = temp.motor_info;
    //Id_PID = temp.id_pid;
    //Iq_PID = temp.iq_pid; 
    Board_CAN_ID = temp.ID;
    Position_Offset = temp.position_offset;
}


void Write_Parameter(){
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

   Flash_Parameters parameters;
   parameters.motor_info = Motor;
   parameters.position_offset = Position_Offset;
   parameters.ID = Board_CAN_ID;

   HAL_FLASH_Unlock();
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = PARAMETER_FLASH_ADDRESS_SECLECTOR;
  EraseInitStruct.NbSectors = 1;
  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  { 
    /* 
      Error occurred while sector erase. 
      User can add here some code to deal with this error. 
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
    */
    /*
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
    */
    Error_Handler();
  } 
  uint32_t addr = PARAMETER_FLASH_ADDRESS;
  uint32_t *data_ptr =   (uint32_t *) &parameters;
  while(addr<PARAMETER_FLASH_ADDRESS+sizeof(Flash_Parameters)){
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *data_ptr);
      addr+=4;
      data_ptr+=1;
  }
  HAL_FLASH_Lock();
}