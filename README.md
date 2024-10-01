# rbms_rote
rbms.hとrote_robo_ms.hの角度取得の機能を一つにまとめたライブラリです  
速度制御のゲインを外部から変更できるようにしました  

例  

```cpp  
#include "mbed.h"  
#include "rbms_rote.h"  
  
#define BUFFER_MAX 4  
  
enum motor{  
    FL,  
    FR,  
    BL,  
    BR,  
    MOTOR_NUM,  
};  
  
BufferedSerial pc(USBTX, USBRX, 9600);  
CAN can(PA_11, PA_12, 1000000);  
CANMessage msg, buffer[BUFFER_MAX];
rbms_rote m2006(can, 0, MOTOR_NUM); // canの実体, motor_type, motor_num  
Thread thread_spd, thread_can;
  
int torque[MOTOR_NUM], speed[MOTOR_NUM]; // 制御用     
short rote[MOTOR_NUM], spd[MOTOR_NUM]; // 値取得用 (0°〜359°), 回転速度(rpm)  
long sumR[MOTOR_NUM]; // (0°~LONG_MAX)  
void spd_con();
void can_recive();  

int main(){  
    // ゲイン設定の例
    m2006.KP = 20;  
    m2006.KI = 5;  
    m2006.KD = 2;  

    // モーターの角度を任意の値で初期化  
    m2006.set_static_reset(0);  
    
    thread_spd.start(spd_con);
    thread_can.start(can_recive);

    while(true){  
        m2006.rote_robo_ms_update(buffer, BUFFER_MAX);
        // 速度制御の例  
        // 速度の設定  
        speed[FL] = 200; speed[FR] = -200; speed[BL] = 200; speed[BR] = -200;  

        // 角度の取得  
        m2006.get_rote(rote); // 0°から359°までの角度を取得する関数  
        m2006.get_rote(sumR); // 0°からlong型の最大値までの角度を取得する関数  
  
        // 速度(rpm)の取得  
        m2006.get_rpm(spd);  
  
        // トルク(制御信号)の送信  
        if(m2006.rbms_send(torque))  
            printf("send\r\n");  
        else  
            printf("fail\r\n");  
    }  
}  
  
void spd_con(){  
    m2006.spd_control(speed, torque);  
}

void can_recive(){
    while(true){
        if(can.read(msg)){
          for (int i = 1; i < BUFFER_MAX; i++){
            buffer[i - 1] = buffer[i];
          }
          buffer[BUFFER_MAX - 1] = msg;
          }
        }
    }
]
