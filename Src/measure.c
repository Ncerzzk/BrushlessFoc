#include "foc.h"

uint8_t Measure_Res_Flag=0;
uint8_t Measure_Ind_Flag=0;

struct masure_sample{
    float i_sum;
    float u_sum;   
    int cnt;

    float measure_ind_duty; // used in measure ind;
    float now_ind_avg_current; // used in measure ind;
    float measure_ind_state; // special used in measure ind;
}Measure_Sample;

void Measure_Ind_Handler(){
    float now_duty = Measure_Sample.measure_ind_duty;
    TIM8->CCR1 = TIM1->ARR*now_duty;
    CCR_Duty temp_ccr={0};

    if(Measure_Sample.measure_ind_state<5){
        temp_ccr.ccra=0;
        temp_ccr.ccrb=0;
        temp_ccr.ccrc=0;
        SVPWM_Step(temp_ccr);
    }else if(Measure_Sample.measure_ind_state==5){
        temp_ccr.ccrb = temp_ccr.ccrc = now_duty;
        temp_ccr.ccra = 0;
        SVPWM_Step(temp_ccr);
    }else if(Measure_Sample.measure_ind_state==6){
        Measure_Sample.i_sum += fabsf(GET_CURRENT(CURR_A_INDEX));   
        Measure_Sample.u_sum += Input_Voltage;
        Measure_Sample.cnt ++;
        temp_ccr.ccra = temp_ccr.ccrb = temp_ccr.ccrc =0;
        SVPWM_Step(temp_ccr);
    }else if(Measure_Sample.measure_ind_state==11){
        temp_ccr.ccrb = temp_ccr.ccra = now_duty;
        temp_ccr.ccrc = 0;
        SVPWM_Step(temp_ccr);
    }else if(Measure_Sample.measure_ind_state==12){
        Measure_Sample.i_sum += fabsf(GET_CURRENT(CURR_C_INDEX)); 
        Measure_Sample.u_sum += Input_Voltage;
        Measure_Sample.cnt ++;

        temp_ccr.ccra = temp_ccr.ccrb = temp_ccr.ccrc =0;
        SVPWM_Step(temp_ccr);
    }else if(Measure_Sample.measure_ind_state==17){
        temp_ccr.ccra = temp_ccr.ccrc = now_duty;
        temp_ccr.ccrb = 0;
        SVPWM_Step(temp_ccr);
    }else if(Measure_Sample.measure_ind_state==18){
        Measure_Sample.i_sum += fabsf(GET_CURRENT(CURR_C_INDEX)) + fabsf(GET_CURRENT(CURR_A_INDEX));  
        Measure_Sample.u_sum += Input_Voltage;
        Measure_Sample.cnt ++;

        temp_ccr.ccra = temp_ccr.ccrb = temp_ccr.ccrc =0;
        SVPWM_Step(temp_ccr);

        Measure_Sample.measure_ind_state =0;
        return ;
    }

    Measure_Sample.measure_ind_state++;
}

float Measure_Res(){
    Mode old_mode = Board_Mode;
    Motor_Parameter old_motor_parameters = Motor;
    PID_S old_Iq_PID,old_Id_PID;

    old_Id_PID = Id_PID;
    old_Iq_PID = Iq_PID;

    Measure_Sample.i_sum=0;
    Measure_Sample.u_sum=0;
    Measure_Sample.cnt=0;

    Motor.Pairs = 0 ;
    Motor.Position_Phase_Offset = 0; // do so to cancel the park_conv, then the Iq = I_Beta, Id = I_Alpha
    
    Id_Set=0;
    Iq_Set=0.5;

    Id_PID.KI = 1.0f;
    Id_PID.KP = 0.5f;

    Iq_PID = Id_PID;

    Board_Mode = CURRENT; 
    FOC_Flag=1;
    
    HAL_Delay(500);

    Measure_Res_Flag=1;

    HAL_Delay(500);

    Measure_Res_Flag=0;
    FOC_Flag=0;

    Board_Mode = old_mode;
    Motor=old_motor_parameters;
    Id_PID = old_Id_PID;
    Iq_PID = old_Iq_PID;

    float res = Measure_Sample.u_sum / Measure_Sample.i_sum *2.0/3.0f;    
    uprintf_polling("res = %f\r\n",res);

    return res;
}

void Measure_Ind(){
    uint32_t old_ARR = TIM1->ARR;
    Mode old_Mode = Board_Mode;
    float res=0;
    float freq = 1500;
    float target_current = 0;

    res = Measure_Res();
    target_current = 0.5f / res;          //本来用1V/res ，现在先用0.5试试吧

    if(target_current > 5){
        uprintf_polling("there may be some problems when measuring res! the target current to measure ind is over 5A!\r\n");
        if(target_current >10){
            uprintf_polling("the target current is over 10A! auto stop now.\r\n");
            return ;
        }
    }

    Measure_Sample.i_sum =0;
    Measure_Sample.u_sum =0;
    Measure_Sample.cnt=0;
    Measure_Sample.now_ind_avg_current =0;
    Measure_Sample.measure_ind_state = 0;
    Measure_Sample.measure_ind_duty = 0;
    //dt =0.0001139
    //u = 11.4
    //di = 6
    // L = u*dt/di = 11.4*0.001
    TIM1->ARR= FOC_TIM_FREQ/freq/2;
    Board_Mode = TEST;
    Measure_Ind_Flag = 1;
    FOC_Flag=1;

    uint32_t old_tick = HAL_GetTick();
    Measure_Sample.measure_ind_duty =0.02;
    while(Measure_Sample.now_ind_avg_current < target_current && HAL_GetTick()<old_tick+2000){
        if(Measure_Sample.cnt==12){ 
            float temp_u=Measure_Sample.u_sum/12.0f;
            Measure_Sample.now_ind_avg_current = Measure_Sample.i_sum / 12.0f;
            HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
            Measure_Sample.cnt=0;
            Measure_Sample.u_sum=Measure_Sample.i_sum=0;
            HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
            Measure_Sample.measure_ind_duty*=1.5;
            uprintf("avg_cur:%f  avg_vol:%f   duty:%f\r\n",Measure_Sample.now_ind_avg_current,temp_u,Measure_Sample.measure_ind_duty);
        }
        
        if(Measure_Sample.measure_ind_duty>0.5f){
            uprintf_polling("there may be some problems that the measure_ind_duty is over 50! it's %f \r\n",Measure_Sample.measure_ind_duty);
            goto FINISH;
        }
    }
            
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    Measure_Sample.cnt=Measure_Sample.u_sum=Measure_Sample.i_sum=0;
    Measure_Sample.measure_ind_state=0;
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    while(Measure_Sample.cnt<100);
    float di = Measure_Sample.i_sum/100.0f;
    float u = Measure_Sample.u_sum/100.0f;
    float dt = TIM1->ARR*Measure_Sample.measure_ind_duty/FOC_TIM_FREQ;
    float L = u*dt/di *2/3;

    uprintf_polling("di:%f,u:%f,dt:%f,ind:%f \r\n",di,u,dt,L);

FINISH:
    FOC_Flag=0;
    Measure_Ind_Flag=0; 
    Board_Mode = old_Mode;
    TIM1->ARR = old_ARR;
    TIM8->CCR1 = TIM1->ARR-ADC_Offset/2;
}
