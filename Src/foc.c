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

float Base_Duty= 0.1f;
float Duty_Amp = 0.0f;

float Base_Speed = 10.0f;
float Speed_Amp=0;

float Base_Iq = 0.0f;
float Iq_Amp=0;


int16_t Position;
float Position_Degree;
float Position_Degree_Set;
float Position_Phase_Degree; // 电角度

float Speed;
float Speed_Set=0;  // 单位：rps 加入超前相位补偿后的实际给定速度
//float Speed_Target=0; // 期望设置的速度
//float Speed_Set_Filter=0;  // 实际给定速度经过极点对消网络后，真正给速度环的速度
//float Position_Phase_Offset=116.587318f; // 电角度的偏移（编码器与电角度对齐）

int16_t current_adc_value[3]={0};
int16_t current_adc_offset[2]={0};
float current[2]={0};

#define CURR_A_INDEX 1
#define CURR_C_INDEX 0


float Input_Voltage=0;
float I_Alpha,I_Beta,Id,Iq;

Motor_Parameter Motor = {7, 123.748848f, 0};
Mode Board_Mode=DUTY;
Mode Last_Mode=DUTY;

uint8_t Measure_Res_Flag=0;
uint8_t Measure_Ind_Flag=0;
//uint8_t Encoder_Direction = 0;

float Uq,Ud; // 电流环的输出值
float Base_Uq,Uq_Amp;
float Id_Set=0;
float Iq_Set=0.6f;

float Phi = 0;
float Position_Offset = 132; 
float Duty_Amp_Value;
typedef struct{
    float last_result;
}Filter;

Filter Base_Speed_Filter,Speed_Amp_Filter;
Filter Speed_Observe_Filter;

float speed_observed;

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

/*
PID_S Id_PID ={
    .KP=0.3f,
    .KD=0,
    .KI=100.0f,
    .I_TIME=  1.0f/FOC_FREQ,
    .i_max = __FLT_MAX__,
    .I_ERR_LIMIT =__FLT_MAX__,
    };
*/
/*
    电流环传递函数：(5K采样率 P=0.3 float精度)
                  4.144e07 (+/- 4.756e05)                                             
  ------------------------------------------------                                
  s^2 + 7236 (+/- 126) s + 1.103e07 (+/- 1.212e05)  
*/

/*
    电流环传递函数：(5K采样率 P=0.3 float精度 将滤波系数改为0.8后) 65.47%
                 2.303e08 (+/- 1.612e06)                                          
  ------------------------------------------------------                          
  s^2 + 1.326e04 (+/- 185.9) s + 6.137e07 (+/- 3.991e05)    
*/

/*
                 6.627e07 (+/- 6.659e06)                                    
  -----------------------------------------------------                     
  s^2 + 1.938e04 (+/- 1985) s + 1.835e07 (+/- 1.844e06)  
95%*/

PID_S Id_PID ={
    .KP=0.27f,//0.27f,//0.211,
    .KD=0,
    .KI=1060.0f,//1060.0f,//275,
    .I_TIME=  1.0f/FOC_FREQ,
    .i_out_max = 8.0f,
    .I_ERR_LIMIT =__FLT_MAX__,
    };

PID_S Special_Id_PID={
    .KP=0,
    .KD=0,
    .KI=0,
    .I_TIME=  1.0f/FOC_FREQ,
    .i_out_max = __FLT_MAX__,
    .I_ERR_LIMIT =__FLT_MAX__,
};

PID_S Iq_PID = {
    .KP=0.27f,//0.27f,//0.211f,
    .KD=0,
    .KI=1060.0f,//1060.0f,//275.0f,
    .I_TIME=  1.0f/FOC_FREQ,
    .i_out_max = 8.0f,
    .I_ERR_LIMIT =__FLT_MAX__, 
    }; // 启动电流挺大的

PID_S Speed_PID={
    .KP=1.0f,//0.94f,//0.1f,//0.12f,
    .KD=0.0003912f,
    .KI=0.0f,//27.0f,//3.6f,
    .I_TIME= 2.0f/FOC_FREQ,
    .i_out_max = 20.0f,
    .I_ERR_LIMIT =  __FLT_MAX__
    };

PID_S Special_Speed_PID={
    .KP=2.9f,//1.158f,//0.38f,//0.12f,
    .KD=0.001085f,
    .KI=0.0f,//38.39f,//1.2f,
    .I_TIME= 2.0f/FOC_FREQ,
    .i_out_max = __FLT_MAX__,
    .I_ERR_LIMIT =  __FLT_MAX__
    };

PID_S Position_PID={
    .KP=20,
    .KD=0,
    .KI=0.1,
    .I_TIME=1.0f/POSITION_FREQ,
    .i_out_max = __FLT_MAX__,
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
    Special_Speed_PID.i=0;
    Position_PID.i=0;
}

#define SAFE_CURRENT    15
static uint8_t Current_Protect_Cnt;
inline void Current_Protect(float id,float iq){
    if(fabsf(id)>SAFE_CURRENT || fabsf(iq)>SAFE_CURRENT){
        Current_Protect_Cnt++;
    }else{
        Current_Protect_Cnt=0;
        return ;
    }

    if(Current_Protect_Cnt>25){
        FOC_Flag=0;
        uprintf("Over Curr!\r\n");
    }
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
  Current_ADC_Init(1000);

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
    //*I_Alpha = 1.5f*ia;
    //*I_Beta = SQRT3_2 * ib + SQRT3_2* (ib + ia);
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
    Ud=PID_Control(&Id_PID,Id_Set,Id);
    Uq=PID_Control(&Iq_PID,Iq_Set,Iq);

    float Ualpha,Ubeta;
    Reverse_Park_Conv(Uq,Ud,Position_Phase_Degree,&Ualpha,&Ubeta);

    SVPWM_Alpha_Beta(Ualpha,Ubeta,Input_Voltage);
    if(Measure_Res_Flag){
        float u = sqrtf(Ualpha*Ualpha+Ubeta*Ubeta);
        float i = sqrtf(Iq*Iq+Id*Id);
        Measure_Sample.i_sum+=i;
        Measure_Sample.u_sum+=u;
        Measure_Sample.cnt++;
    }
}

float last_phi;
float cos_val,cos_val_last_phi;

void Special_Current_Control(){
    float theta;

    float theta_last_phi; // 基于上次的phi计算的theta
    float theta_delay;


    float sub;
    Ud=PID_Control(&Id_PID,Id_Set,Id);

    theta = (Position_Degree-Position_Offset-Phi)/180.0f*3.1415926f;   // 基于新的phi计算的theta
    theta_last_phi = (Position_Degree-Position_Offset-last_phi)/180.0f*3.1415926f; //基于上次的phi计算的theta

    theta_delay = atan2f(-speed_observed*2*PI,1496.0f);  // 根据速度，计算得到的电流滞后电压的角度（此值为负数）
    theta += theta_delay;
    theta_last_phi +=theta_delay;

    cos_val= arm_cos_f32(theta);
    cos_val_last_phi = arm_cos_f32(theta_last_phi);  // 计算新旧两个phi的cos值

    sub = fabsf(cos_val-cos_val_last_phi);
    if(sub<0.05f){// 如果cos值之差小于0.2f，那么直接切换到新值
        Uq = Base_Uq+Uq_Amp*cos_val;
        last_phi = Phi;
    }else{
        Uq = Base_Uq+Uq_Amp*cos_val_last_phi; // 否则还是用旧值，等到新旧两个cos即将相交的时候再切换
    }

    //Uq=Base_Uq+Uq_Amp*arm_cos_f32(theta);

    float Ualpha,Ubeta;
    Reverse_Park_Conv(Uq,Ud,Position_Phase_Degree,&Ualpha,&Ubeta);

    SVPWM_Alpha_Beta(Ualpha,Ubeta,Input_Voltage);
}


float position_observed;

float speed_observed2;
void Speed_Observe(){
    static float last_position;
    float sub =  (position_observed - last_position) ;
    if(sub<-180){
        sub+=360;
    }else if(sub>180){
        sub-=360;
    }
    float now_speed = sub * FOC_FREQ;
    speed_observed=0.96*speed_observed+ 0.04*now_speed;

    last_position = position_observed;
    position_observed =0.9*(position_observed+speed_observed / FOC_FREQ)+ 0.1*Position_Degree;
    position_observed=Position_Degree;
}

#define GET_CURRENT(x)     (GET_SHUNT_VOLTAGE(current_adc_offset[x],current_adc_value[x])/SHUNT_RES)
#define Limit(value,max)     if(value>max)value=max;else if(value<-max)value=-max
const int Speed_Control_CNT_MAX = FOC_FREQ/SPEED_FREQ;
const int Position_Control_CNT_MAX = FOC_FREQ/POSITION_FREQ;

inline float phi_compensate(float speed_amp,float speed,float compensate_angle){
    float phi=0;
    float t=0;

    // 先求平均的速度差 (V最大 - V一般时候)/2
    if(speed_amp<0.1f){  // 防止除0错误
        return 0;
    }
    float delta_v_degree = speed_amp/2.0f*360.0f;  // speed_amp 的单位是RPS
    t = compensate_angle / delta_v_degree;

    phi = t*speed*360;
    return phi;

}



/*
    构造一个前置滤波器来消掉PI控制器引入的零点
    该滤波器传递函数为：G = a/(s+a)
*/
float Lead_Filter(float a,float dt,float input,Filter * f){
    float result;
    //float static last_result;

    result= (dt*a*input + f->last_result)/(a*dt+1);
    f->last_result = result;

    return result;
}
/*
    超前校正网络
            1+a*tau*s
    G = ------------
            a*(1+tau*s)
*/
float Lead_Test_In =0;
float Lead_Test_Out =0;
float Lead_Compensator(float a,float tau,float dt,float input){
    static float last_input;
    static float last_result;

    float result;

    result =(input + a*tau*(input-last_input)/dt + a*tau*last_result/dt)/(a+a*tau/dt);
    last_input = input;
    last_result = result;
    return result;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    float temp_iq,temp_id;
    static int speed_control_cnt=0;
    static int current_control_cnt=0;

    if(hadc->Instance==CURRENT_ADC){
        Check_Mode();
        //Lead_Test_Out = Lead_Compensator(5.667f,0.00175f,1/20000.0f,Lead_Test_In);
        /*
        if(current_control_cnt%Position_Control_CNT_MAX==0){
           if(Board_Mode == POSITION){
                Position_Control();
            }
        }
        */
       
        HAL_GPIO_WritePin(GPIOC,LED_GREEN_Pin,GPIO_PIN_SET);   

        UTILS_LP_FAST(current[0],GET_CURRENT(0),0.1f);
        UTILS_LP_FAST(current[1],GET_CURRENT(1),0.1f);

        UTILS_LP_FAST(Input_Voltage,ADCVAL_TO_VOLTAGE(current_adc_value[2])*(VOLTAGE_RES1+VOLTAGE_RES2)/VOLTAGE_RES2,0.1f);

        Clark_Conv(current[1],current[0],&I_Alpha,&I_Beta);
    
        Position = Get_Position();   
        Position_Degree = Position_to_Rad_Dgree(Position,1);
        Position_Phase_Degree = Position_Degree * Motor.Pairs-Motor.Position_Phase_Offset;
        //Speed_Observe();
        Get_Speed();

        if(speed_control_cnt==1){
            if(Board_Mode >= SPEED){
                if(Board_Mode == SPECIAL_SPEED){
                    static float last_base_speed,last_amp_value;
                    static uint8_t close_amp_flag;
                    float amp_value;
                    float sub;
                    float temp_phi=0;
                    float temp_base_speed,temp_speed_amp;
                    temp_base_speed =Lead_Filter(1000,Special_Speed_PID.I_TIME,Base_Speed,&Base_Speed_Filter);
                    temp_speed_amp = Lead_Filter(1000,Special_Speed_PID.I_TIME,Speed_Amp,&Speed_Amp_Filter);

                    sub=fabsf(Base_Speed-last_base_speed);
                    amp_value=temp_speed_amp* arm_cos_f32((Position_Degree-Position_Offset-Phi+temp_phi)/180.0f*3.1415926f); 

                    if(!close_amp_flag && Base_Speed>35 && sub>2.5f){  // 在高速情况下，要提速（增加base_Speed）
                        close_amp_flag=1;    // 把amp关了，记录此时的amp的值
                        last_amp_value = amp_value;
                    }
                    last_base_speed = Base_Speed;
                    
                    if(close_amp_flag){  // amp关了的情况下，使用在关amp瞬间的amp_value
                        Speed_Set = temp_base_speed + last_amp_value;
                        if(fabsf(Iq)<6.0f && fabsf(amp_value-last_amp_value)<0.2f){
                            close_amp_flag=0;  
                            // 如果IQ<6，说明速度已经提上去了
                            // 此时在和上一次停止amp差不多的位置，再把amp开了
                        }
                    }else{
                        Speed_Set = temp_base_speed + amp_value;
                    }
                    //temp_phi = phi_compensate(temp_speed_amp,temp_base_speed,15);
                    
                }
                Speed_Control();
            }
            speed_control_cnt=0;
        }else{
            speed_control_cnt++;
        }
        Park_Conv(I_Alpha,I_Beta,Position_Phase_Degree,&temp_iq,&temp_id);
        Iq=temp_iq;Id=temp_id;
        Current_Protect(Id,Iq);
        //UTILS_LP_FAST(Iq,temp_iq,0.8f);
        //UTILS_LP_FAST(Id,temp_id,0.8f);
        HAL_GPIO_WritePin(GPIOC,LED_GREEN_Pin,GPIO_PIN_RESET);   

#ifdef CAMERA_SUPPORT
        Camera_Exposure(1,0.25,Camera_Catch_Angle);
#endif

        if(FOC_Flag){
            if(Board_Mode==CURRENT_DUTY){
                Special_Current_Control();
            }
            else if(Board_Mode>=CURRENT){
                //Iq_Set = Base_Iq + Iq_Amp *  arm_cos_f32((Position_Degree-Position_Offset-Phi)/180.0f*3.1415926f);
                Current_Control();
            }else if(Board_Mode == DUTY){
                float theta_delay = atan2f(-speed_observed*2*PI,1496.0f);
                Duty_Amp_Value = Duty_Amp * arm_cos_f32((Position_Degree-Position_Offset-Phi+theta_delay)/180.0f*3.1415926f);
                float duty = Base_Duty + Duty_Amp_Value;
                SVPWM(Position_Phase_Degree+90,duty);
            }
        }else{
            if(Board_Mode!=TEST)
                FOC_STOP();
        }
        send_debug_wave(&wg1);
        send_debug_wave(&wg2);

        send_debug_wave(&wg3);
        send_debug_wave(&wg4);

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

float Ka=130.0f,b;  //加速度系数
float Kb=0.01f; // 编码器得到的速度系数
static void Get_Speed(){
    
    static float last_position_phase;
    float sub=0;
    sub= Position_Degree - last_position_phase;
    if(sub<-180){
        sub+=360;
    }else if(sub>180){
        sub-=360;
    }
    UTILS_LP_FAST(Speed,FOC_FREQ/360.0f*sub,0.04f);
    last_position_phase = Position_Degree;

    float a = Ka*Iq+b;
    float observe_sub = Speed-speed_observed;
    speed_observed+= a*1.0f/FOC_FREQ+observe_sub*Kb;
    //Speed = speed_observed / 360.0f;

    speed_observed2 = Lead_Filter(4.83f,1.0f/FOC_FREQ,27.2f*Iq,&Speed_Observe_Filter);
}

float Lead_Compensator_In;
static void Speed_Control(){
    if(Board_Mode==SPECIAL_SPEED){
        //Speed_Set_Filter = Lead_Filter(Special_Speed_PID.KI/Special_Speed_PID.KP,Special_Speed_PID.I_TIME,Speed_Set);
        
        Iq_Set=PID_Control(&Special_Speed_PID,Speed_Set,speed_observed); 
        //Iq_Set =  Lead_Compensator(5.667f,0.00175f,Speed_PID.I_TIME,Lead_Compensator_In);
    }else{
        //Speed_Set_Filter = Lead_Filter(Speed_PID.KI/Speed_PID.KP,Speed_PID.I_TIME,Speed_Set);
        Iq_Set=PID_Control(&Speed_PID,Speed_Set,speed_observed);
        //Iq_Set = Lead_Compensator(5.667f,0.00175f,Speed_PID.I_TIME,Lead_Compensator_In);
    }

    

    if(Iq_Set>15){
        Iq_Set=15;
    }else if(Iq_Set<-15){
        Iq_Set=-15;
    }

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

#include "can.h"
inline void Mode_Enter_Init(){
    switch(Board_Mode){
        case IDLE:
            FOC_STOP();
        break;
        case SAFE_SPEED:
            FOC_Flag=0;
            uprintf("CURRENT_PROTECT!\r\n");
            CAN_Send_Message(Board_CAN_ID+1,"CUR",3);
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

