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

float Base_Speed=1500.0f;
float Speed_Attitude=500.0f;

float Base_Duty= 0.1f;
float Duty_Amp = 0.0f;

uint8_t Wave_Flag=0;


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
float input_voltage=0;
float ialpha,ibeta,id,iq;

Motor_Parameter Motor = {7, 123.748848f, 0};
Mode Board_Mode=DUTY;
Mode Last_Mode=DUTY;

//uint8_t Encoder_Direction = 0;

float Id_Set=0;
float Iq_Set=0.6f;

PID_S Id_PID ={
    .KP=2,
    .KD=0,
    .KI=2,
    .I_TIME=  1.0f/FOC_FREQ,
    .i_max = __FLT_MAX__,
    .I_ERR_LIMIT = 0.3f
    };
PID_S Iq_PID = {
    .KP=2,
    .KD=0,
    .KI=1.5,
    .I_TIME=  1.0f/FOC_FREQ,
    .i_max = __FLT_MAX__,
    .I_ERR_LIMIT = 0.3f
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

void PID_I_Clear(){
    Iq_PID.i=0;
    Id_PID.i=0;
    Speed_PID.i=0;
    Position_PID.i=0;
}

void Current_ADC_Init(int times){
    int32_t curr_sum[2]={0};
    HAL_GPIO_WritePin(DC_CAL_GPIO_Port,DC_CAL_Pin,GPIO_PIN_SET);
    HAL_Delay(10);
    for(int i=0;i<times;++i){
        curr_sum[0]+=current_adc_value[0];
        curr_sum[1]+=current_adc_value[1];
        HAL_Delay(1);
    }

    current_adc_offset[0]=curr_sum[0]/times;
    current_adc_offset[1]=curr_sum[1]/times;

   HAL_GPIO_WritePin(DC_CAL_GPIO_Port,DC_CAL_Pin,GPIO_PIN_RESET); 
   uprintf_polling("the offset of curr_adc is %d,%d\r\n",current_adc_offset[0],current_adc_offset[1]);

}

void Foc_Init(){
  AS5047_Set_Direction(Motor.Encoder_Direction);
  HAL_ADC_Start_DMA(&hadc1,current_adc_value,3);

  HAL_TIM_Base_Start(&htim8);

   //TIM1->RCR=1;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  HAL_GPIO_WritePin(EN_GATE_GPIO_Port,EN_GATE_Pin,GPIO_PIN_SET);

  Current_ADC_Init(20);

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

  HAL_TIM_Base_Start_IT(&htim7);
}

#define SQRT3    1.73205080756887719000f
#define SQRT3_2  0.866025403784438595f
void Clark_Conv(float ia, float ic, float *ialpha, float *ibeta)
{
    ia= -ia;
    ic= -ic;
    // 为什么这里要加负号呢
    // 因为电流的正负根据相电流的方向为正，如U4（ABC 100），此时A的电流为正，BC电流为负
    // 与硬件上的正负相反

    float ib = -(ia+ic);
    *ialpha = 1.5f*ia;
    *ibeta = SQRT3_2 * ib + SQRT3_2* (ib + ia);
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
    float target_theta= atan2f(Ubeta,Ualpha)*180.0f/3.1415926f;
    if(target_theta<0){
        target_theta+=360;
    }
    float length = sqrtf(Ualpha*Ualpha+Ubeta*Ubeta);
    
    if(length>8){
        length=8;
    }
    

    float phase=(int)Position_Phase_Degree%360;
    //SVPWM(Position_Phase_Degree+90,0.1);

    
    //uprintf("%f %f",target_theta,length);
    SVPWM(target_theta,length/8.0f);  //8V = 12V *2/3;
}

#define ADC_TO_VOLTAGE(x)  ((current_adc_offset[x]-current_adc_value[x])/4096.0f*3.3f)
#define GAIN    40.0f
#define RES     0.003f

const int Speed_Control_CNT_MAX = FOC_FREQ/SPEED_FREQ;
const int Position_Control_CNT_MAX = FOC_FREQ/POSITION_FREQ;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    float temp_iq,temp_id;
    static int current_control_cnt=0;

    if(hadc->Instance==CURRENT_ADC){
        Check_Mode();
        if(current_control_cnt%Position_Control_CNT_MAX==0){
            //p = sin(7200°/s * t) = sin(40 *pi *t)
            //float t = current_control_cnt* 1.0f/FOC_FREQ;
            //Position_Degree_Set = arm_cos_f32(4*PI*t)*180+180;
            //Position_Control();
            if(Board_Mode == POSITION){
                Position_Control();
            }
            //Speed_Set = Base_Speed + Speed_Attitude*arm_sin_f32(Position_Degree/180.0f*3.1415926f);
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
        //current[0]=ADC_TO_VOLTAGE(0)/GAIN/RES;
        //current[1]=ADC_TO_VOLTAGE(1)/GAIN/RES;
        UTILS_LP_FAST(current[0],ADC_TO_VOLTAGE(0)/GAIN/RES,0.1);
        UTILS_LP_FAST(current[1],ADC_TO_VOLTAGE(1)/GAIN/RES,0.1);

        input_voltage = current_adc_value[2]/4096.0f*3.3f*(39+2.2f)/2.2f;

        Clark_Conv(current[1],current[0],&ialpha,&ibeta);
    
        Position = Get_Position();
        Position_Degree = Position_to_Rad_Dgree(Position,1);
        Position_Phase_Degree = Position_Degree * Motor.Pairs-Motor.Position_Phase_Offset;
        //;
        Park_Conv(ialpha,ibeta,Position_Phase_Degree,&temp_iq,&temp_id);
        UTILS_LP_FAST(iq,temp_iq,0.1);
        UTILS_LP_FAST(id,temp_id,0.1);
        HAL_GPIO_WritePin(GPIOC,LED_GREEN_Pin,GPIO_PIN_RESET);   
        
        if(FOC_Flag){
            if(Board_Mode>=CURRENT){
                Current_Control();
            }else if(Board_Mode == DUTY){
                float duty = Base_Duty + Duty_Amp * arm_sin_f32(Position_Degree/180.0f*3.1415926f);
                SVPWM(Position_Phase_Degree+90,duty);
            }
        }else{
            FOC_STOP();
        }
        if(Wave_Flag)
            send_wave(Position_Degree,Position_Phase_Degree,Speed,iq);
/*
        if(FOC_Flag){
            
            //
            //send_wave(current[0],current[1],id,iq);
        }else{
            Set_Vector(U0,0.1f); 
        }
        */
    }
}

void Test_Direction(){
    uint16_t last_pos=Position;
    int result=0;
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
}

inline void Rotate_Phase(float start,float stop,float step){
    for(float phase = start;phase<stop;phase+=step){
        SVPWM(phase,0.1f);
        HAL_Delay(1);
    }
}

void Detect_Encoder(){ 
    // 检测磁编码器静态误差
    // 检测电角度与机械角度的偏移
    float sin_sum,cos_sum;
    float last_position=0;
    float m_sub = 0; // 机械角度转过了多少
    sin_sum=cos_sum=0;
    Motor.Position_Phase_Offset = 0;
    for(int i =0; i<1000;++i){
        HAL_Delay(1);
        uprintf_polling("pos:%d\r\n",Position);
    }
    Rotate_Phase(0,360,360.0f/1000);
    Set_Vector(U0,0.1);
    HAL_Delay(10);
    uprintf_polling("rotate to syn,and the phase pos_degree is %f\r\n",Position_Phase_Degree);
    last_position = Position_Degree;
    for(int i=0;i<21;++i){  // 3个一圈电角度，21个一圈机械角度
        Rotate_Phase(i*120,(i+1)*120,120/500.0f);
        float sin,cos;
        float off_temp = Position_Phase_Degree - (i+1)*120;
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
    uprintf_polling("the offset is %f \r\n",Motor.Position_Phase_Offset);
    uprintf_polling("the paris %f\r\n",120.0f*21/m_sub);
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

static void Speed_Control(){
    static float last_position_phase;
    float sub=0;
    sub= Position_Degree - last_position_phase;
    if(sub<-180){
        sub+=360;
    }else if(sub>180){
        sub-=360;
    }
    UTILS_LP_FAST(Speed,sub*SPEED_FREQ,0.4);

    Iq_Set=PID_Control(&Speed_PID,Speed_Set,Speed);
    last_position_phase = Position_Degree;

 
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