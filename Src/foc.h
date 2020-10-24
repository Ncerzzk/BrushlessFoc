#ifndef __FOC_H
#define __FOC_H

#include "SVPWM.h"
#include "pid.h"

typedef struct
{
    uint8_t Pairs;
    float Position_Phase_Offset;
    uint8_t Encoder_Direction;
    float R;
    float L;
}Motor_Parameter;

typedef enum{
    IDLE=0x00,
    OPENLOOP,
    DUTY,
    CURRENT,
    SPEED,
    POSITION
}Mode;


extern Motor_Parameter Motor;
extern PID_S Id_PID;
extern PID_S Iq_PID;
extern PID_S Speed_PID;
extern PID_S Position_PID;

void Foc_Init();
extern float Id_Set;
extern float Iq_Set;
extern float Speed_Set;
extern float Position_Degree_Set;

extern float Base_Speed;
extern float Speed_Attitude;

extern float Base_Duty;
extern float Duty_Amp;

extern uint8_t Wave_Flag;

#endif
