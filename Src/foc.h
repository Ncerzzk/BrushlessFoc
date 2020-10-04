#ifndef __FOC_H
#define __FOC_H

#include "SVPWM.h"
typedef struct
{
    uint8_t pairs;
    float R;
    float L;
    int8_t direction;
    // 电机的运转方向，约定在开环模式下，电机按u4->u6方向运行（电角度增大），机械角度也增大为正方向
    // 此参数与电机 a b c 三相的焊接有关
    // 此参数仅在有感模式中(SENSOR_FOC)有用
} Motor_Info;




void Foc_Init();

#endif
