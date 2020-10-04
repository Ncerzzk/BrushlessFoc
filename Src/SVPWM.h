#ifndef __SVPWM_H
#define __SVPWM_H
#include "stdint.h"
#include "tim.h"

#define PI  3.1415926f
#define SVPWM_TIM   TIM1


typedef struct
{
    float ccra;
    float ccrb;
    float ccrc;
} CCR_Duty;

typedef struct
{
    uint8_t a;
    uint8_t b;
    uint8_t c;
    float theta;
    float x;
    float y;
} Uvect_Mos;

extern  Uvect_Mos U0,U1,U2,U3,U4,U5,U6,U7;
void SVPWM(float Target_U, float duty);
void Set_Vector(Uvect_Mos u, float duty);
#endif
