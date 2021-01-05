#include "cmd_fun.h"
#include "command.h"
#include "uart_ext.h"
#include "foc.h"
#include "pid.h"

#define ERROR_PARAGRAM()      uprintf("error arg_num!\r\n"); return 

typedef struct{
  char * var_name;
  void * value_ptr;
}Var_Edit_Struct;

Var_Edit_Struct Var_List[]={
  //{"first",&First_Time_Check}
  {"id",&Id_Set},
  {"iq",&Iq_Set},
  {"sp",&Speed_Set},
  {"pos",&Position_Degree_Set},
  {"base_sp",&Base_Speed},
  {"sp_a",&Speed_Attitude},
  {"duty",&Base_Duty},
  {"duty_amp",&Duty_Amp},
  {"phi",&Phi},
  {"pos_off",&Position_Offset},
  {"syn_off",&Motor.Position_Phase_Offset},
  #ifdef CAMERA_SUPPORT
  {"camera",&Camera_Catch_Angle}
  #endif
};

#define  STR(x) #x

volatile char * test_s = STR(Var_List);
volatile char * test_s2 = STR(test_s);

static void test(int arg_num,char **string_prams,float * arg){
  int i;
  uprintf("this is testkk\r\n");
  uprintf("i get %d args\r\n",arg_num);
  for(i=0;i<(arg_num&0xFF);++i){
    uprintf("one is %f\r\n",arg[i]);
  }
  for(i=0;i<(arg_num>>8);++i){
    uprintf("string_prams is %s\r\n",string_prams[i]);
  }
  uprintf("%s",test_s);
    uprintf("%s",test_s2);
}

void set_val(int arg_num,char ** s,float * args){
  void * edit_value;
  uint8_t write=0;
  if(arg_num!=0x0201 && arg_num!=0x0200){
    ERROR_PARAGRAM();
  }

  if(arg_num==0x0201){
    write=1;
  }

  for(int i=0;i<sizeof(Var_List)/sizeof(Var_Edit_Struct);++i){
    if(compare_string(Var_List[i].var_name,s[0])){
      edit_value=Var_List[i].value_ptr;
      break;
    }
  }
  
  if(compare_string(s[1],"u8")){
    if(write)
      *(uint8_t *)edit_value=(uint8_t)args[0];
    uprintf("ok set %s = %d\r\n",s[0],*(uint8_t *)edit_value);  
  }else if(compare_string(s[1],"int")){
    if(write)
      *(int16_t *)edit_value=(int16_t)args[0];
    uprintf("ok set %s = %d\r\n",s[0],*(int16_t *)edit_value);
  }else if(compare_string(s[1],"f")){
    if(write)
      *(float *)edit_value=args[0];
    uprintf("ok set %s = %f\r\n",s[0],*(float *)edit_value);
  }
}




/*
下面定义用户需要的函数
*/



extern void Set_to_Un(uint8_t t,Uvect_Mos target,float duty);
extern Uvect_Mos U4;
extern float Position_Degree;
#ifdef USE_GYRO
  extern PID_S Angle_PID;
#endif

void set_to_u4(int arg_num,char ** s,float * args){
  uint8_t temp_t;
  if(arg_num!=0x0001){
    return ;
  }
  temp_t=(uint8_t)args[0];
  Set_to_Un(temp_t,U4,0.5);
  HAL_Delay(5);
  uprintf_polling("OK set to us and keep %d ms\r\n",temp_t);
  uprintf_polling("the position is %f\r\n",Position_Degree);
}
extern void Test_Direction();
void test_direction(int arg_num,char **s,float * args){
  uprintf_polling("start to test direction!\r\n");
  Test_Direction();

}

extern void Detect_Encoder();
void detect(int arg_num,char **s,float *args){
  uprintf_polling("start to detect encoder!\r\n");
  Detect_Encoder();
}

extern uint8_t FOC_Flag;
void start(int arg_num,char **s,float *args){
  if(arg_num!=0x0001){
    Iq_PID.i=0;
    Id_PID.i=0;
    Speed_PID.i=0;
    Position_PID.i=0;
    Special_Id_PID.i=0;
    #ifdef USE_GYRO
    Angle_PID.i=0;
    #endif
    FOC_Flag = !FOC_Flag;
    uprintf_polling("OK! flag=%d\r\n",FOC_Flag);
    return ;
  }
  if(args[0]>0){
    FOC_Flag=1;
  }else{
    FOC_Flag=0;
    Set_Vector(U0,0.1f);
  }
  uprintf_polling("OK! flag=%d\r\n",FOC_Flag);
}

/*
extern uint8_t Song_Start;
void song(int arg_num,char **s,float *args){
  Song_Start = !Song_Start;
  uprintf_polling("OK,song = %d\r\n",Song_Start);
}
*/
extern float Measure_Res();
extern void Measure_Ind();
void measure_res(int arg_num,char **s,float *args){
  uprintf_polling("OK,start to measure Res!\r\n");
  Measure_Res();
}

void measure_ind(int arg,char **s,float *args){
  uprintf_polling("OK,start to measure Ind!\r\n");
  Measure_Ind();
}

#include "parameter.h"
void wrtie_param(int arg_num,char **s,float *args){
  Write_Parameter();
  uprintf("OK,Save Params\r\n");
}

#include "debug_utils.h"
void wave(int arg_num,char **s,float * args){
  if(arg_num == 0x0000){
    Wave_Flag = !Wave_Flag;
  }else if(arg_num==0x0001){
    Wave_ID = args[0];
  }
  uprintf("OK ,wave = %d\r\n",Wave_ID);
}


void set_PID(int arg_num,char ** s,float * args){
  PID_S * pid_s=0;
  float * kpkdki=0;
  if(arg_num!=0x0201){
      ERROR_PARAGRAM();
  }
  if(compare_string(s[0],"d")){
    pid_s=&Id_PID;
  }else if(compare_string(s[0],"q")){
    pid_s=&Iq_PID;
  }else if(compare_string(s[0],"sd")){  // speical id
    pid_s=&Special_Id_PID;
  }else if(compare_string(s[0],"sp")){
    pid_s=&Speed_PID;
  }else if(compare_string(s[0],"pos")){
    pid_s=&Position_PID;
  }
  #ifdef USE_GYRO
  else if(compare_string(s[0],"a")){
    pid_s= &Angle_PID;
  }
  #endif


  pid_s->i=0;
  if(compare_string(s[1],"p")){
    kpkdki=&(pid_s->KP);
  }else if(compare_string(s[1],"d")){
    kpkdki=&(pid_s->KD);
  }else if(compare_string(s[1],"i")){
    kpkdki=&(pid_s->KI);
  }
  *kpkdki=args[0];
  uprintf("ok set %s 's %s = %f\r\n",s[0],s[1],args[0]);
}

 #ifdef CAMERA_SUPPORT
void camera(int arg_num,char **s,float * args){
  CAMERA_Open = !CAMERA_Open;
}
#endif

void position(int arg_num,char **s,float *args){
 uprintf("position = %f\r\n",Position_Degree);
}

void set_mode(int arg_num,char **s,float *args){
  if(arg_num!=0x0001){
    ERROR_PARAGRAM();
  }
  uint8_t temp = (uint8_t)args[0];
  if(temp>=LAST_MODE){
    ERROR_PARAGRAM();
  }else{
    Board_Mode= temp;
    uprintf("OK set board mode = %d\r\n",Board_Mode);
  }
}

void set_pos_off_here(int arg_num,char **s,float *args){
  if(arg_num!=0x0000){
    ERROR_PARAGRAM();
  }
  Position_Offset = Position_Degree;
  uprintf("ok pos_off = %f\r\n",Position_Offset);
}

#ifdef AS_SPI_SLAVE
#include "foc_spi_com.h"
void test_spi_com(int arg_num,char **s,float *args){
  uprintf_polling("val 1 2 3 = %d %d %f\r\n",SPI_Slave_Test,SPI_Slave_Test2,SPI_Slave_Test3);
}
#endif
/*
将要增加的命令与函数写在这里
*/
void command_init(void){
  add_cmd("test",test); 
  add_cmd("set",set_val);
  add_cmd("set_u4",set_to_u4);
  add_cmd("test_direction",test_direction);
  add_cmd("detect",detect);
  add_cmd("start",start);
  add_cmd("set_pid",set_PID);
  add_cmd("write",wrtie_param);
  add_cmd("wave",wave);
  add_cmd("pos_off_here",set_pos_off_here);
  #ifdef CAMERA_SUPPORT
  add_cmd("camera",camera);
  #endif
  add_cmd("pos",position);
  add_cmd("set_mode",set_mode);

  add_cmd("measure_res",measure_res);
  add_cmd("measure_ind",measure_ind);

  #ifdef AS_SPI_SLAVE
  add_cmd("test_spi",test_spi_com);
  #endif
  //add_cmd("song",song);
}

