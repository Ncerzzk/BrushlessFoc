
#include "SVPWM.h"
#include "arm_math.h"

Uvect_Mos U0 = {0, 0, 0, 0};
Uvect_Mos U4 = {1, 0, 0, 0};
Uvect_Mos U6 = {1, 1, 0, PI / 3};
Uvect_Mos U2 = {0, 1, 0, PI / 3 * 2};
Uvect_Mos U3 = {0, 1, 1, PI};
Uvect_Mos U1 = {0, 0, 1, PI / 3 * 4};
Uvect_Mos U5 = {1, 0, 1, PI / 3 * 5};
Uvect_Mos U7 = {1, 1, 1, 0};

static void SVPWM_Step(CCR_Duty duty)
{
    SVPWM_TIM->CCR1 = duty.ccra * SVPWM_TIM->ARR;
    SVPWM_TIM->CCR2 = duty.ccrb * SVPWM_TIM->ARR;
    SVPWM_TIM->CCR3 = duty.ccrc * SVPWM_TIM->ARR;
}


void Set_Vector(Uvect_Mos u, float duty)
{
    CCR_Duty temp_duty = {0};
    if(duty>0.95f){
        duty=0.95f;
    }

    temp_duty.ccra = u.a * duty; //011
    temp_duty.ccrb = u.b * duty;
    temp_duty.ccrc = u.c * duty;

    SVPWM_Step(temp_duty);
}

/*
    将电压矢量设置为Un，并持续一段时间
*/
void Set_to_Un(uint8_t t,Uvect_Mos target,float duty)
{
    Set_Vector(target,duty);
    HAL_Delay(t);
    Set_Vector(U0,duty); 
}


inline uint8_t get_area(uint32_t now_theta)
{
    return now_theta / 60 + 1;
}


// 根据扇区号，获取该扇区的两个电压矢量
// area:1-6
static void get_area_u(uint8_t area, Uvect_Mos *u1, Uvect_Mos *u2)
{
    switch (area)
    {
    case 1:
        *u1 = U4;
        *u2 = U6;
        break;
    case 2:
        *u1 = U6;
        *u2 = U2;
        break;
    case 3:
        *u1 = U2;
        *u2 = U3;
        break;
    case 4:
        *u1 = U3;
        *u2 = U1;
        break;
    case 5:
        *u1 = U1;
        *u2 = U5;
        break;
    case 6:
        *u1 = U5;
        *u2 = U4;
        break;
    default:
        break;
    }
}

// 计算某个矢量的CCR1-CCR3的占空比（0-1）
// 该矢量可由U1和U2合成，该矢量在U1轴上的投影为t1，在U2轴上的投影为t2
CCR_Duty Get_CCR_Duty(float t1, float t2, Uvect_Mos u1, Uvect_Mos u2)
{

    float t0, t3;
    CCR_Duty result;
    t0 = (1 - t1 - t2) / 2;  // t0 为U0矢量所占的时间
    t3 = t0;                 // t3 为U7矢量所占的时间

    result.ccra = U7.a * t3 + u1.a * t1 + u2.a * t2;
    result.ccrb = U7.b * t3 + u1.b * t1 + u2.b * t2;
    result.ccrc = U7.c * t3 + u1.c * t1 + u2.c * t2;

    return result;
}

// 计算某个矢量在相邻两个基矢量轴上的投影(t1,t2)
// 所谓基矢量轴，假设当前矢量为45°，则相邻的两个基矢量就是0°(U4),60°(U6)
// alpah 为该矢量在alpha轴的投影，beta同理
// area 为该矢量所在扇区
static void SVPWM_Normal_CalT1T2(float alpha, float beta, uint8_t area, float *t1, float *t2)
{
    float temp = 0;
    switch (area)
    {
    case 1:
        *t1 = alpha - 0.57735f * beta;
        *t2 = 1.1547f * beta;
        break;
    case 2:
        *t1 = alpha + 0.57735f * beta;
        *t2 = 0.57735f * beta - alpha;
        break;
    case 3:
        *t1 = 1.1547f * beta;
        *t2 = -alpha - 0.57735f * beta;
        break;
    case 4:
        *t1 = 0.57735f * beta - alpha;
        *t2 = -1.1547f * beta;
        break;
    case 5:
        temp = -0.57735f * beta;
        *t1 = temp - alpha;
        *t2 = temp + alpha;
        break;
    case 6:
        *t1 = -1.1547f * beta;
        *t2 = alpha + 0.57735f * beta;
        break;
    }
}




// Target_U 期望电压矢量的角度 单位° 0-360
// duty 期望占空比 0-1
void SVPWM(float Target_U, float duty)
{
    Uvect_Mos u1, u2;
    float x, y;
    float t1, t2;
    static CCR_Duty CCR_Dutys = {0};

    if(Target_U>360){
        Target_U -=360*((int)Target_U/360);
    }

    uint8_t area = get_area(Target_U);
    get_area_u(area, &u1, &u2);
    arm_sin_cos_f32(Target_U, &y, &x);
    x *= 0.86f * duty;
    y *= 0.86f * duty; // sqrt(3)/2 没错
    SVPWM_Normal_CalT1T2(x, y, area, &t1, &t2);
    CCR_Dutys = Get_CCR_Duty(t1, t2, u1, u2);
    SVPWM_Step(CCR_Dutys);
}
