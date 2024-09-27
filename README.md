# rbms_rote
rbms.hとrote_robo_ms.hの角度取得の機能を一つにまとめたライブラリです  
速度制御のゲインを外部から変更できるようにしました  

例  
#include "mbed.h"  
#include "rbms_rote.h"  

enum motor{  
　FL,  
　FR,  
　BL,  
　BR,  
　MOTOR_NUM,  
};  
  
BufferedSerial(USBTX, USBRX, 9600);  
CAN can(PA11, PA_12, 1000000);  
rbms_rote(can, 0, MOTOR_NUM); // canの実体, motor_type, motor_num  
  
int torque; // 制御用  
short speed;  
short rote[MOTOR_NUM], spd[MOTOR_NUM]; // 値取得用  

