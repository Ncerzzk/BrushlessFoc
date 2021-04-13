
#include "foc.h"
#include "utils.h"

extern float Position_Phase_Degree,Position_Degree,Speed;
extern float angle[3];
extern float current[2];
extern float I_Alpha,I_Beta;
extern float Uq,Ud;
extern float Iq_Set,Iq,Id;
/*
Wave_Group wg1={
    0,
    &Position_Degree,
    &Position_Phase_Degree,
    &Speed,
    &Iq
};
*/
extern float Lead_Test_Out,Lead_Test_In;
Wave_Group wg1={
    0,
    &Lead_Test_Out,
    &Position_Phase_Degree,
    &Speed,
    &Lead_Test_In
};

Wave_Group wg2={
    1,
    current,
    current+1,
    &I_Alpha,
    &I_Beta
};


Wave_Group wg3={
    2,
    &Ud,
    &Uq,
    &Id,
    &Iq
};

extern float speed_observed,position_observed;
extern float Speed_Set;
extern float raw_speed;
extern float Speed_Set_Filter;
extern float Lead_Compensator_In;
extern float Duty_Amp_Value;
extern float speed_observed2;
extern float last_phi;
extern float cos_val,cos_val_last_phi;
Wave_Group wg4={
    3,
    &speed_observed,
    &Iq,
    &Position_Phase_Degree,
    &Position_Degree //Uq
};

//如果要辨识速度开环的参数（也就是电流闭环），应以Iq_Set作为输入，Speed_observed作为输出
//如果要辨识电流开环，应以Uq作为输入，Iq作为输出。
