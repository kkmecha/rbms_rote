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
rbms_rote m2006(can, 0, MOTOR_NUM); // canの実体, motor_type, motor_num  
Thread thread_spd;
  
int torque[MOTOR_NUM]; // 制御用  
short speed[MOTOR_NUM];  
short rote[MOTOR_NUM], spd[MOTOR_NUM]; // 値取得用 (0°〜359°), 回転速度(rpm)  
long sumR[MOTOR_NUM]; // (0°~LONG_MAX)  
void spd_con();
  
int main(){  
　　thread_spd.start(spd_con);
　　while(true){  
　　　　// 速度制御の例  
　　　　// 速度の設定  
　　　　speed[FL] = 200; speed[FR] = -200; speed[BL] = 200; speed[BR] = -200;  
　　　　m2006.rbms_send(torque);  
　　　　  
　　　　// 角度の取得 2種類あり、、0°からlong型の最大値までの角度を取得する関数があります  
　　　　m2006.get_rote(rote); // 0°から359°までの角度を取得する関数  
　　　　
　　}  
}  
  
void spd_con(){  
　　m2006.spd_control(speed, torque);  
}
