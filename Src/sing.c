
#include "sing.h"
#include "uart_ext.h"

struct
{
    TIM_HandleTypeDef * USE_TIM;
    uint32_t TIM_MAX_FREQ;
    int16_t FREQ;  // 节拍计数值
    float DUTY;
    const uint8_t * VOICE;
    uint32_t CNT;
    uint32_t LENGTH;
} Song_Info;

void Song_Init(TIM_HandleTypeDef * tim,int16_t freq,float duty,const uint8_t * voice,uint32_t length){
    Song_Info.USE_TIM=tim;
    if(tim->Instance!=TIM1 && tim->Instance!=TIM8){
        Song_Info.TIM_MAX_FREQ=84000000;
    }else{
        Song_Info.TIM_MAX_FREQ=168000000;
    }
    Song_Info.FREQ=freq;
    Song_Info.DUTY=duty;
    Song_Info.VOICE=voice;
    Song_Info.CNT=0;
    Song_Info.LENGTH = length;
    
    tim->Instance->PSC = 0;
    tim->Instance->ARR = Song_Info.TIM_MAX_FREQ/freq;

    Set_Vector(U6,0.3f);
    HAL_Delay(3);
    Set_Vector(U0,0);

}

#define SONG_RANGE 20    // 振幅
uint8_t Song_Start = 0;
void Song_Play_TIM_IRQ_Handler()
{
    float value = (float)(Song_Info.VOICE[Song_Info.CNT]) / 255.0f;
    if(Song_Start){
        float theta = (value)*SONG_RANGE+60;
        SVPWM(theta,0.1f);
        send_wave(theta,0,0,0);
    }else{
        Set_Vector(U0,0.1f);
    }

    Song_Info.CNT++;
    if(Song_Info.CNT>=Song_Info.LENGTH){
        Song_Info.CNT=0;
    }
    
}


