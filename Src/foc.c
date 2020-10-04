#include "tim.h"
#include "main.h"
#include "adc.h"
#include "uart_ext.h"
#include "foc.h"
#include "math.h"
#include "as5047.h"
#include "SVPWM.h"
#include "arm_math.h"
#include "pid.h"
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


Uvect_Mos * Anti_Clock_Order[6]={&U4,&U6,&U2,&U3,&U1,&U5};

int16_t Position;
float Position_Degree;
float Position_Phase_Degree; // 电角度
float Position_Phase_Offset=2.0f; // 电角度的偏移（编码器与电角度对齐）

int16_t current_adc_value[3]={0};
int16_t current_adc_offset[2]={0};
float current[2]={0};
float input_voltage=0;
float ialpha,ibeta,id,iq;

Motor_Info Motor = {7, 0, 0};

float Id_Set=0;
float Iq_Set=1;

PID_S Current_PID={1,0,0};

uint8_t FOC_Flag=0;


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
  AS5047_Set_Direction(0);

  HAL_ADC_Start_DMA(&hadc1,current_adc_value,3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  TIM1->RCR=1;
  HAL_TIM_Base_Start(&htim8);

  HAL_GPIO_WritePin(EN_GATE_GPIO_Port,EN_GATE_Pin,GPIO_PIN_SET);
  
  Current_ADC_Init(20);

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);



}

#define SQRT3    1.73205080756887719000f
#define SQRT3_2  0.866025403784438595f
void Clark_Conv(float ia, float ic, float *ialpha, float *ibeta)
{

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
    float Ud=PID_Control(&Current_PID,Id_Set,id);
    float Uq= PID_Control(&Current_PID,Iq_Set,iq);

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
    SVPWM(Position_Phase_Degree+90,0.1);

    send_wave((int)Position_Phase_Degree%360,target_theta,id,iq);
    //uprintf("%f %f",target_theta,length);
   // SVPWM(target_theta,length/8.0f);  //8V = 12V *2/3;
}

#define ADC_TO_VOLTAGE(x)  ((current_adc_offset[x]-current_adc_value[x])/4096.0f*3.3f)
#define GAIN    40.0f
#define RES     0.003f

uint32_t cnt;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    float temp=0;
    float temp_iq,temp_id;
    if(hadc->Instance==CURRENT_ADC){  
        HAL_GPIO_WritePin(GPIOC,LED_GREEN_Pin,GPIO_PIN_SET);   
        //current[0]=ADC_TO_VOLTAGE(0)/GAIN/RES;
        //current[1]=ADC_TO_VOLTAGE(1)/GAIN/RES;
        UTILS_LP_FAST(current[0],ADC_TO_VOLTAGE(0)/GAIN/RES,0.1);
        UTILS_LP_FAST(current[1],ADC_TO_VOLTAGE(1)/GAIN/RES,0.1);

        input_voltage = current_adc_value[2]/4096.0f*3.3f*(39+2.2f)/2.2f;

        Clark_Conv(current[1],current[0],&ialpha,&ibeta);
        


        Position = Get_Position();
        Position_Degree = Position_to_Rad_Dgree(Position,1);
        Position_Phase_Degree = Position_Degree * Motor.pairs-Position_Phase_Offset;
        //;
        Park_Conv(ialpha,ibeta,Position_Phase_Degree,&temp_iq,&temp_id);
        UTILS_LP_FAST(iq,temp_iq,0.1);
        UTILS_LP_FAST(id,temp_id,0.1);
        HAL_GPIO_WritePin(GPIOC,LED_GREEN_Pin,GPIO_PIN_RESET);   
        
        if(FOC_Flag){
            Current_Control();
            //SVPWM(Position_Phase_Degree+90,0.1);
        }else{
            Set_Vector(U0,0.1f);
        }
        
        //send_wave(current[0],current[1],id,iq);
        
        //send_wave(iq,id,input_voltage,Position);
    }
}

void Test_Direction()
{
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
    Set_Vector(U0,0.1);
    uprintf_polling("the test result is :%d \r\n",result);
}

inline void Rotate_Phase(float start,float stop,float step){
    for(float phase = start;phase<stop;phase+=step){
        SVPWM(phase,0.1f);
        HAL_Delay(1);
    }
}

void Detect_Encoder(){
    float sin_sum,cos_sum;
    sin_sum=cos_sum=0;
    Position_Phase_Offset = 0;
    Rotate_Phase(0,360,360.0f/1000);
    Set_Vector(U0,0.1);
    HAL_Delay(10);
    uprintf_polling("rotate to syn,and the phase pos_degree is %f\r\n",Position_Phase_Degree);
    for(int i=0;i<21;++i){  // 3个一圈电角度，21个一圈机械角度
        Rotate_Phase(i*120,(i+1)*120,120/500.0f);
        float sin,cos;
        float off_temp = Position_Phase_Degree - (i+1)*120;


        arm_sin_cos_f32(off_temp,&sin,&cos);

        sin_sum+=sin;
        cos_sum+=cos;
        uprintf_polling("the sub is %f  sin is %f,cos is %f\r\n",off_temp,sin,cos);        
    }
    Set_Vector(U0,0.1);
    Position_Phase_Offset=atan2f(sin_sum,cos_sum)*180/3.1415926f;
    uprintf_polling("the offset is %f \r\n",Position_Phase_Offset);
}


void Openloop_Runing(float duty, float freq, int8_t direction, float rps)
{
    static float theta = 0;
    float delta_p = rps * Motor.pairs * 360.0f / freq; // 每次调用本函数要增加的值

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





