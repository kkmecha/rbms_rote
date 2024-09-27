# rbms_rote
rbms.hとrote_robo_ms.hの角度取得の機能を一つにまとめたライブラリです  
速度制御のゲインを外部から変更できるようにしました  

例  
#include "mbed.h"  
#include "rbms_rote.h"  

BufferedSerial(USBTX, USBRX, 9600);
CAN can(PA11, PA_12, 1000000);  
rbms_rote(can, 0, 4); // canの実体, motor_type, motor_num  

int torque;  
short speed;
short rote, spd; // 値取得用
