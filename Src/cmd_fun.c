#include "cmd_fun.h"
#include "command.h"
#include "uart_ext.h"
#include "foc.h"
#include "pid.h"

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
  {"phi",&Phi}
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
  if(arg_num!=0x0201){
    uprintf("error arg_num!\r\n");
    return ;
  }

  for(int i=0;i<sizeof(Var_List)/sizeof(Var_Edit_Struct);++i){
    if(compare_string(Var_List[i].var_name,s[0])){
      edit_value=Var_List[i].value_ptr;
      break;
    }
  }
  
  if(compare_string(s[1],"u8")){
    *(uint8_t *)edit_value=(uint8_t)args[0];
    uprintf("ok set %s = %d\r\n",s[0],*(uint8_t *)edit_value);  
  }else if(compare_string(s[1],"int")){
    *(int16_t *)edit_value=(int16_t)args[0];
    uprintf("ok set %s = %d\r\n",s[0],*(int16_t *)edit_value);
  }else if(compare_string(s[1],"f")){
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

#include "parameter.h"
void wrtie_param(int arg_num,char **s,float *args){
  Write_Parameter();
  uprintf("OK,Save Params\r\n");
}

void wave(int arg_num,char **s,float * args){
  Wave_Flag = !Wave_Flag;
}

void set_PID(int arg_num,char ** s,float * args){
  PID_S * pid_s=0;
  float * kpkdki=0;
  if(arg_num!=0x0201){
    uprintf("error arg_num!\r\n");
    return ;
  }
  if(compare_string(s[0],"d")){
    pid_s=&Id_PID;
  }else if(compare_string(s[0],"q")){
    pid_s=&Iq_PID;
  }else if(compare_string(s[0],"sp")){
    pid_s=&Speed_PID;
  }else if(compare_string(s[0],"pos")){
    pid_s=&Position_PID;
  }


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
  //add_cmd("song",song);
}


