#ifndef __RBMS_ROTE_H__
#define __RBMS_ROTE_H__

#define TORQUE_MAX_M2006 10000
#define TORQUE_MAX_M3508 16384
#define GIAR_RATIO_M2006 36
#define GIAR_RATIO_M3508 19
#define DEG_PER_RAD 0.017444f
#define BRAKE_GAIN 4
#define MOTOR_MAX 8

#include "mbed.h"

class rbms_rote{
    public:
        rbms_rote(CAN &can, bool motor_type, int motor_num);
        int rbms_send(int* motor);
        void rote_robo_ms_update(CANMessage *msg, int BUFFER_MAX);
        void spd_control(int* set_speed,int* motor); 
        void get_rote(long *rote);
        void get_rote(short *rote);
        void get_rpm(short *spd);
        void set_static_reset(int num);
        float KP = 25;
        float KI = 10;
        float KD = 0;

    private:
        void rbms_read(CANMessage &msg, short *rotation, short *speed);
        float speed_pid(float T,short rpm_now, short set_speed,float *delta_rpm_pre,float *ie,float KP,float KI, float KD);
        CANMessage _canMessage,_canMessage2,_msg;
        CAN &_can;
        bool _motor_type;//if 0 m2006,if 1 m3508
        int _motor_num,_motor_max;
        unsigned short _r;
        int _rotation, _speed, _torque, _temperature;

        short deltaR, _rote[MOTOR_MAX], _spd[MOTOR_MAX]; // 角度計算用なので別にしている
        long sumRstatic[MOTOR_MAX];
};

#endif









