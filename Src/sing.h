#ifndef __SING_H
#define __SING_H

#include "tim.h"
#include "SVPWM.h"
#include "math.h"

void Song_Init(TIM_HandleTypeDef * tim,int16_t freq,float duty,const uint8_t * voice,uint32_t length);
void Song_Play_TIM_IRQ_Handler();

#endif