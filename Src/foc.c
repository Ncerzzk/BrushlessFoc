#include "tim.h"
#include "main.h"
#include "adc.h"
#include "uart_ext.h"
#include "foc.h"
#include "math.h"
#include "as5047.h"
#include "SVPWM.h"
#include "arm_math.h"

#include "gpio.h"
#include "utils.h"


#define CURRENT_ADC ADC1
/**
 * A simple low pass filter.
 *
 * @param value
 * The filtered value.
 *
 * @param sample
 * Next sample.
 *
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 */
#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * (value - (sample)))
#define FOC_STOP() Set_Vector(U0,0.1f) 
inline void Check_Mode();
static void Get_Speed();

float Base_Speed=1500.0f;
float Speed_Attitude=500.0f;

float Base_Duty= 0.1f;
float Duty_Amp = 0.0f;



int16_t Position;
float Position_Degree;
float Position_Degree_Set;
float Position_Phase_Degree; // 电角度

float Speed;
float Speed_Set=3000.0f;  // 单位：机械角度/s
//float Position_Phase_Offset=116.587318f; // 电角度的偏移（编码器与电角度对齐）

int16_t current_adc_value[3]={0};
int16_t current_adc_offset[2]={0};
float current[2]={0};

#define CURR_A_INDEX 1
#define CURR_C_INDEX 0


float input_voltage=0;
float ialpha,ibeta,id,iq;

Motor_Parameter Motor = {7, 123.748848f, 0};
Mode Board_Mode=DUTY;
Mode Last_Mode=DUTY;

uint8_t Measure_Res_Flag=0;
uint8_t Measure_Ind_Flag=0;
//uint8_t Encoder_Direction = 0;

float Id_Set=0;
float Iq_Set=0.6f;

float Phi = 90;
float Position_Offset = 132; 
// This var is used to align the offset of propeller. 旋翼头的零点
// 定义该零点为
/*
左电机：(逆时针转)
        /------------------/
      /------------------/
    /------------------/
  /------------------/

右电机:(顺时针转)
    \----------------\
     \----------------\ 
      \----------------\
       \----------------\

此时的力矩是向前翻滚
*/

struct masure_sample{
    float i_sum;
    float u_sum;   
    int cnt;

    float measure_ind_duty; // used in measure ind;
    float now_ind_avg_current; // used in measure ind;
    float measure_ind_state; // special used in measure ind;
}Measure_Sample;


PID_S Id_PID ={
    .KP=2,
    .KD=0,
    .KI=2,
    .I_TIME=  1.0f/FOC_FREQ,
    .i_max = __FLT_MAX__,
    .I_ERR_LIMIT =__FLT_MAX__,
    };

PID_S Special_Id_PID={
    .KP=0,
    .KD=0,
    .KI=0,
    .I_TIME=  1.0f/FOC_FREQ,
    .i_max = __FLT_MAX__,
    .I_ERR_LIMIT =__FLT_MAX__,
};

PID_S Iq_PID = {
    .KP=2,
    .KD=0,
    .KI=1.5,
    .I_TIME=  1.0f/FOC_FREQ,
    .i_max = __FLT_MAX__,
    .I_ERR_LIMIT = __FLT_MAX__, 
    }; // 启动电流挺大的

PID_S Speed_PID={
    .KP=0.00045f,
    .KD=0,
    .KI=0.00015f,
    .I_TIME= 1.0f/SPEED_FREQ,
    .i_max = __FLT_MAX__,
    .I_ERR_LIMIT =  2000
    };

PID_S Position_PID={
    .KP=20,
    .KD=0,
    .KI=0.1,
    .I_TIME=1.0f/POSITION_FREQ,
    .i_max = __FLT_MAX__,
    .I_ERR_LIMIT = 5 
    };
uint8_t FOC_Flag=0;

static void Speed_Control();
static void Position_Control();
void Measure_Ind_Handler();

void PID_I_Clear(){
    Iq_PID.i=0;
    Id_PID.i=0;
    Speed_PID.i=0;
    Position_PID.i=0;
}

// 该函数用于处理暗室长曝光频闪
// exposure_time:曝光时间,单位:s
// flash_time:闪光时间,单位:ms
#ifdef CAMERA_SUPPORT

uint8_t CAMERA_Open=1;
float Camera_Catch_Angle=90;
#define CAMERA_POS_ERROR 1
void Camera_Exposure(uint32_t exposure_time,float flash_time,float pos){
    static int cnt=0;
    if(!CAMERA_Open){
        return ;
    }
    float angle = Position_Degree - Position_Offset;
    if(angle<0){
        angle+=360;
    }
    if(fabsf(angle-pos)<CAMERA_POS_ERROR && cnt==0){
        HAL_GPIO_WritePin(CAMERA_EXP_GPIO_Port,CAMERA_EXP_PIN,GPIO_PIN_SET);
        cnt++;
        return ;
    }

    if(cnt==(uint32_t)(FOC_FREQ/1000*flash_time)){
        HAL_GPIO_WritePin(CAMERA_EXP_GPIO_Port,CAMERA_EXP_PIN,GPIO_PIN_RESET); 
    }else if(cnt==FOC_FREQ*exposure_time){
        cnt=0;
    }

    if(cnt>0){
        cnt++;
    }
}
#endif


void Current_ADC_Init(int times){
    int32_t curr_sum[2]={0};
    #ifdef DRV8302
    HAL_GPIO_WritePin(DC_CAL_GPIO_Port,DC_CAL_Pin,GPIO_PIN_SET);
    #endif
    HAL_Delay(10);
    for(int i=0;i<times;++i){
        curr_sum[0]+=current_adc_value[0];
        curr_sum[1]+=current_adc_value[1];
        HAL_Delay(1);
    }

    current_adc_offset[0]=curr_sum[0]/times;
    current_adc_offset[1]=curr_sum[1]/times;

   #ifdef DRV8302
   HAL_GPIO_WritePin(DC_CAL_GPIO_Port,DC_CAL_Pin,GPIO_PIN_RESET); 
   #endif
   uprintf_polling("the offset of curr_adc is %d,%d\r\n",current_adc_offset[0],current_adc_offset[1]);

}

void Foc_Init(){
  AS5047_Set_Direction(Motor.Encoder_Direction);
  
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  //HAL_ADC_Start_DMA(&hadc1,current_adc_value,3);
  HAL_ADC_Start(&hadc3);
  HAL_ADC_Start(&hadc2);

  HAL_ADCEx_MultiModeStart_DMA(&hadc1,(uint32_t *)current_adc_value,3);

  HAL_TIM_Base_Start(&htim8);

   //TIM1->RCR=1;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  #ifdef DRV8302
    HAL_GPIO_WritePin(EN_GATE_GPIO_Port,EN_GATE_Pin,GPIO_PIN_SET);
  #endif
  Current_ADC_Init(20);

  //HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
  //HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);  与SLAVE_SPI冲突了
  //HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

  HAL_TIM_Base_Start_IT(&htim7);
}

#define SQRT3    1.73205080756887719000f
#define SQRT3_2  0.866025403784438595f
#define SQRT3_3  0.57735026918f
void Clark_Conv(float ia, float ic, float *ialpha, float *ibeta)
{
    ia= -ia;
    ic= -ic;
    // 为什么这里要加负号呢
    // 因为电流的正负根据相电流的方向为正，如U4（ABC 100），此时A的电流为正，BC电流为负
    // 与硬件上的正负相反

    float ib = -(ia+ic);
    //*ialpha = 1.5f*ia;
    //*ibeta = SQRT3_2 * ib + SQRT3_2* (ib + ia);
    *ialpha = ia;
    *ibeta = SQRT3_3 * ib + SQRT3_3* (ib + ia); 
    // 恒幅值变换
    // 为什么要恒幅值变换？使变换出来的两轴信号平方和开根号，与原来的信号幅值相同
    // 好处是这样计算电阻这些参数可以直接用两轴信号平方和开根号来作为电压
    // 坏处是功率变换前和变换后就不守恒了，不过暂时没什么地方用到功率
}

void Park_Conv(float ialpha,float ibeta,float theta,float *iq,float *id){
    float sin,cos;
    sin=cos=0;
    arm_sin_cos_f32(theta,&sin,&cos);
    *id = ialpha * cos + ibeta *sin;
    *iq = -ialpha *sin + ibeta *cos;
}

void Reverse_Park_Conv(float q,float d,float theta,float *alpha,float *beta){
    float sin,cos;
    sin=cos=0;
    arm_sin_cos_f32(theta,&sin,&cos);
    *beta = d*sin + q*cos;
    *alpha = d*cos -q*sin;
}



void Current_Control(){
    float Ud=PID_Control(&Id_PID,Id_Set,id);
    float Uq=PID_Control(&Iq_PID,Iq_Set,iq);

    float Ualpha,Ubeta;
    Reverse_Park_Conv(Uq,Ud,Position_Phase_Degree,&Ualpha,&Ubeta);

    SVPWM_Alpha_Beta(Ualpha,Ubeta,input_voltage);
    if(Measure_Res_Flag){
        float u = sqrtf(Ualpha*Ualpha+Ubeta*Ubeta);
        float i = sqrtf(iq*iq+id*id);
        Measure_Sample.i_sum+=i;
        Measure_Sample.u_sum+=u;
        Measure_Sample.cnt++;
    }
}


#define GET_CURRENT(x)     (GET_SHUNT_VOLTAGE(current_adc_offset[x],current_adc_value[x])/SHUNT_RES)
#define Limit(value,max)     if(value>max)value=max;else if(value<-max)value=-max
const int Speed_Control_CNT_MAX = FOC_FREQ/SPEED_FREQ;
const int Position_Control_CNT_MAX = FOC_FREQ/POSITION_FREQ;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    float temp_iq,temp_id;
    static int current_control_cnt=0;

    if(hadc->Instance==CURRENT_ADC){
        Check_Mode();
        if(current_control_cnt%Position_Control_CNT_MAX==0){
           if(Board_Mode == POSITION){
                Position_Control();
            }
        }
        if(current_control_cnt%Speed_Control_CNT_MAX==0){
            if(Board_Mode >= SPEED){
                Speed_Control();
            }
        }
        current_control_cnt++;
        if(current_control_cnt==FOC_FREQ/2/2){
            current_control_cnt=0;
        }
        HAL_GPIO_WritePin(GPIOC,LED_GREEN_Pin,GPIO_PIN_SET);   

        UTILS_LP_FAST(current[0],GET_CURRENT(0),0.1);
        UTILS_LP_FAST(current[1],GET_CURRENT(1),0.1);

        UTILS_LP_FAST(input_voltage,ADCVAL_TO_VOLTAGE(current_adc_value[2])*(VOLTAGE_RES1+VOLTAGE_RES2)/VOLTAGE_RES2,0.1f);

        Clark_Conv(current[1],current[0],&ialpha,&ibeta);
    
        Position = Get_Position();   
        Get_Speed();
        Position_Degree = Position_to_Rad_Dgree(Position,1);
        Position_Phase_Degree = Position_Degree * Motor.Pairs-Motor.Position_Phase_Offset;

        Park_Conv(ialpha,ibeta,Position_Phase_Degree,&temp_iq,&temp_id);
        UTILS_LP_FAST(iq,temp_iq,0.1);
        UTILS_LP_FAST(id,temp_id,0.1);
        HAL_GPIO_WritePin(GPIOC,LED_GREEN_Pin,GPIO_PIN_RESET);   

#ifdef CAMERA_SUPPORT
        Camera_Exposure(1,0.25,Camera_Catch_Angle);
#endif

        if(FOC_Flag){
            if(Board_Mode>=CURRENT){
                Current_Control();
            }else if(Board_Mode == DUTY){
                float duty = Base_Duty + Duty_Amp * arm_cos_f32((Position_Degree-Position_Offset-Phi)/180.0f*3.1415926f);
                float out = PID_Control(&Special_Id_PID,0,id);
                Limit(out,20); 
                float leading_angle = 90 - out;  // 超前角度
                SVPWM(Position_Phase_Degree+leading_angle,duty);
            }
        }else{
            if(Board_Mode!=TEST)
                FOC_STOP();
        }
        send_debug_wave(&wg1);
        send_debug_wave(&wg2);

        send_debug_wave(&wg3);

        if(Measure_Ind_Flag)
            Measure_Ind_Handler();
    }
}

#define TEST_DUTY   0.086f
void Test_Direction(){
    uint16_t last_pos=Position;
    Mode old_mode;
    int result=0;

    old_mode = Board_Mode;
    Board_Mode = TEST;
    for(int i=0;i<1000;++i){
        SVPWM(i%360,0.1);
        /*
        for (int j=0;j<1000;++j){
            __NOP();
        }*/
        HAL_Delay(1);
        if(Position>last_pos){
            result++;
        }else if(Position<last_pos){
            result--;
        }
        last_pos=Position;
    }
    FOC_STOP();
    uprintf_polling("the test result is :%d \r\n",result);
    if(result<0){
        Motor.Encoder_Direction=!Motor.Encoder_Direction;
    }

    Board_Mode = old_mode;
}

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
        temp_ccr.ccra = temp_ccr.ccrc = now_duty;
        temp_ccr.ccrc = 0;
        SVPWM_Step(temp_ccr);
    }else if(Measure_Sample.measure_ind_state==6){
        Measure_Sample.i_sum += fabsf(GET_CURRENT(CURR_A_INDEX));
        Measure_Sample.u_sum += input_voltage;
        Measure_Sample.cnt ++;
        temp_ccr.ccra = temp_ccr.ccrb = temp_ccr.ccrc =0;
        SVPWM_Step(temp_ccr);
    }else if(Measure_Sample.measure_ind_state==11){
        temp_ccr.ccrb = temp_ccr.ccrc = now_duty;
        temp_ccr.ccra = 0;
        SVPWM_Step(temp_ccr);
    }else if(Measure_Sample.measure_ind_state==12){
        Measure_Sample.i_sum += fabsf(GET_CURRENT(CURR_C_INDEX));
        Measure_Sample.u_sum += input_voltage;
        Measure_Sample.cnt ++;

        temp_ccr.ccra = temp_ccr.ccrb = temp_ccr.ccrc =0;
        SVPWM_Step(temp_ccr);
    }else if(Measure_Sample.measure_ind_state==17){
        temp_ccr.ccra = temp_ccr.ccrc = now_duty;
        temp_ccr.ccra = 0;
        SVPWM_Step(temp_ccr);
    }else if(Measure_Sample.measure_ind_state==18){
        Measure_Sample.i_sum += fabsf(GET_CURRENT(CURR_C_INDEX)) + fabsf(GET_CURRENT(CURR_A_INDEX));
        Measure_Sample.u_sum += input_voltage;
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
    Motor.Position_Phase_Offset = 0; // do so to cancel the park_conv, then the iq = ibeta, id = ialpha
    
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
    float L = u*dt/di;

    uprintf_polling("di:%f,u:%f,dt:%f,ind:%f \r\n",di,u,dt,L);

FINISH:
    FOC_Flag=0;
    Measure_Ind_Flag=0; 
    Board_Mode = old_Mode;
    TIM1->ARR = old_ARR;
    TIM8->CCR1 = TIM1->ARR-ADC_Offset/2;
}

inline void Rotate_Phase(float start,float stop,float step){
    for(float phase = start;phase<stop;phase+=step){
        SVPWM(phase,TEST_DUTY);
        HAL_Delay(1);
    }
}


#define Test_Phase 120.0f
#define Rotate_Time 500 //ms
void Detect_Encoder(){ 
    // 检测磁编码器静态误差
    // 检测电角度与机械角度的偏移
    float sin_sum,cos_sum;
    float last_position=0;
    float m_sub = 0; // 机械角度转过了多少
    Mode old_mode;

    old_mode = Board_Mode;
    Board_Mode = TEST;

    sin_sum=cos_sum=0;
    Motor.Position_Phase_Offset = 0;
    for(int i =0; i<1000;++i){
        HAL_Delay(1);
        uprintf_polling("pos:%d\r\n",Position);
    }
    Rotate_Phase(0,360,360.0f/Rotate_Time);
    Set_Vector(U0,0.1);
    HAL_Delay(10);
    uprintf_polling("rotate to syn,and the phase pos_degree is %f\r\n",Position_Phase_Degree);
    last_position = Position_Degree;
    for(int i=0;i<21;++i){  // 3个一圈电角度，21个一圈机械角度
        Rotate_Phase(i*Test_Phase,(i+1)*Test_Phase,Test_Phase/Rotate_Time);
        float sin,cos;
        float off_temp = Position_Phase_Degree - (i+1)*Test_Phase;
        arm_sin_cos_f32(off_temp,&sin,&cos);
        sin_sum+=sin;
        cos_sum+=cos;
        uprintf_polling("the sub is %f  sin is %f,cos is %f\r\n",off_temp,sin,cos);

        float tmp=(Position_Degree-last_position);
        if(tmp<0){
            tmp+=360;
        }
        m_sub+=tmp;
        last_position =  Position_Degree;
    }
    Set_Vector(U0,0.1);
    Motor.Position_Phase_Offset=atan2f(sin_sum,cos_sum)*180/3.1415926f;
    Motor.Pairs = (int)(Test_Phase*21/m_sub+0.5f);
    //Motor.Position_Phase_Offset = 5581.0f;
    uprintf_polling("the offset is %f \r\n",Motor.Position_Phase_Offset);
    uprintf_polling("the paris %d\r\n",Motor.Pairs);

    Board_Mode = old_mode;
}


void Openloop_Runing(float duty, float freq, int8_t direction, float rps)
{
    static float theta = 0;
    float delta_p = rps * Motor.Pairs * 360.0f / freq; // 每次调用本函数要增加的值

    theta += direction * delta_p;
    if (theta > 360)
    {
        theta -= 360;
    }
    else if (theta < 0)
    {
        theta += 360;
    }
    //SVPWM(theta, duty);
}

static void Get_Speed(){
    static float last_position_phase;
    float sub=0;
    sub= Position_Degree - last_position_phase;
    if(sub<-180){
        sub+=360;
    }else if(sub>180){
        sub-=360;
    }
    UTILS_LP_FAST(Speed,sub*SPEED_FREQ,0.4);
    last_position_phase = Position_Degree;
}
static void Speed_Control(){

    //Get_Speed();
    Iq_Set=PID_Control(&Speed_PID,Speed_Set,Speed);
    //last_position_phase = Position_Degree;

 
}

static void Position_Control(){
    float err = Position_Degree_Set - Position_Degree;
    float out = 0;
    if(err>180){
        err-=360;         //target = 359,now=0;
    }else if(err<-180){
        err+=360;        // target=0,now=359
    }
    if(fabsf(err)<Position_PID.I_ERR_LIMIT){
        Position_PID.i+=err;
    }
    out = Position_PID.KP * err + Position_PID.KD *(err-Position_PID.last_err) + Position_PID.i * Position_PID.KI;
    Position_PID.last_err = err;

    Speed_Set = out;

}

inline void Mode_Enter_Init(){
    switch(Board_Mode){
        case IDLE:
            FOC_STOP();
        break;
        default:
            PID_I_Clear();
        break;
    }
    Last_Mode = Board_Mode;
}

inline void Check_Mode(){
    if(Last_Mode !=Board_Mode){
        Mode_Enter_Init();
        Last_Mode = Board_Mode;
    }
}

 

/*
1ms中断太慢了，无法满足精度要求。
换成在电流中断中处理。

const int Speed_Control_CNT_MAX = TIM7_FREQ/SPEED_FREQ;
const int Position_COntrol_CNT_MAX = TIM7_FREQ/POSITION_FREQ;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{


  static int ms_cnt;

  if(htim->Instance == TIM7){
      // 1ms Interupt
    ms_cnt++;
    if(ms_cnt%Position_COntrol_CNT_MAX==0){
        Position_Control();
    }
    if(ms_cnt%Speed_Control_CNT_MAX==0){
        Speed_Control();
    }
    if(ms_cnt==1000){
        ms_cnt=0;
    }

  }

}

*/