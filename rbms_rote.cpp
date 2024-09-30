#include "rbms_rote.h"

rbms_rote::rbms_rote(CAN &can, bool motor_type, int motor_num)
            : _can(can), _motor_type(motor_type), _motor_num(motor_num){
    if(_motor_type){
        _motor_max = TORQUE_MAX_M3508;
    }else{
        _motor_max = TORQUE_MAX_M2006;
    }
    if(_motor_num <= 8){
        _can.frequency(1000000);
        _can.mode(CAN::Normal);
    }
}

int rbms_rote::rbms_send(int* motor) {
    char _byte[_motor_num*2];
    int _a = 0;
    for(int i = 0; i < _motor_num; i++){ 
        if(motor[i] > _motor_max)
            return 0;
        _byte[_a++] = (char)(motor[i] >> 8);
        _byte[_a++] = (char)(motor[i] & 0xFF); 
    }

    _canMessage.id = 0x200;//esc id1~4のcanの送信id
    _canMessage.len = 8;//can data長(8byte固定)
    _canMessage2.id = 0x1ff;//esc id5~8のcanの送信id
    _canMessage2.len = 8;
    _a = 0;
    int _i = 0;
    for(int i = 0; i < _motor_num; i++){
        if(i < 4){
            _canMessage.data[_a] = _byte[_a];
            _a++;
            _canMessage.data[_a] = _byte[_a];
            _a++;
        }else{
            _canMessage2.data[_i++] = _byte[_a++];
            _canMessage2.data[_i++] = _byte[_a++];
        }
    }
    while(_a < 15){
        if(_a < 7){
            _canMessage.data[_a++] = 0;
            _canMessage.data[_a++] = 0;
        }else{
            _canMessage2.data[_a++] = 0;
            _canMessage2.data[_a++] = 0;
        } 
    }
   
    if (_can.write(_canMessage)&&_can.write(_canMessage2)) {
        return 1;
    }else{
        return -1;
    }

}

void rbms_rote::rbms_read(CANMessage &msg, short *rotation, short *speed) {
    _r = (msg.data[0] << 8) | (msg.data[1] & 0xff);
    _rotation = (float)_r / 8192 * 360;//8192=360°
    *rotation = _rotation;

    _speed = (msg.data[2] << 8) | (msg.data[3] & 0xff);
    if (_speed & 0b1000000000000000){//マイナス値の場合(最上位ビットが1のとき)(2の補数)
        _speed--;
        _speed = -~_speed;
    }
    *speed = _speed;

    _torque = (msg.data[4] << 8) | (msg.data[5] & 0xff);
    if (_torque & 0b1000000000000000){
        _torque--;
        _torque = -~_torque;
    }

    _temperature = msg.data[6];
            
}

void rbms_rote::rote_robo_ms_update(CANMessage *msg, int BUFFER_MAX)
{
    short tmpR[_motor_num], rote[_motor_num], spd[_motor_num];
    // if(_debugmsg)printf("motor%d sumS:%d sumD:%d rote:%d spd:%d deltaR:%d tmpR:%d\n",_motornum,(int)sumRstatic,(int)sumRdynamic,rote,spd,deltaR,tmpR);
    int msgnum = 0x201 + _motor_num;
    for (int i = 0; i < BUFFER_MAX; i++)
    {
        if (msg[i].id == msgnum)
            _msg = msg[i];
    }
    rbms_read(_msg, rote, spd);
    
    for(int i = 0; i < _motor_num; i++){
        if (rote[i] < tmpR[i] && spd[i] > 0) // 正転時
        {
            deltaR = (short)(360 - tmpR[i]) + rote[i];
        }
        else if (rote[i] > tmpR[i] && spd[i] > 0)
        {
            deltaR = rote[i] - tmpR[i];
        }
        if (rote[i] > tmpR[i] && spd[i] < 0) // 反転時    
        {
            deltaR = (short)(360 - rote[i]) + tmpR[i];
            deltaR *= -1;
        }
        else if (rote[i] < tmpR[i] && spd[i] < 0)
        {
            deltaR = rote[i] - tmpR[i];
        }
        if (rote[i] == tmpR[i]) //変化なし
        {
            deltaR = 0;
        }

        sumRstatic[i] += deltaR;
        tmpR[i] = rote[i];
        // printf("%x\n",msgnum);
        // printf("motor%d length:%d spd:%d rote:%d deltaR:%d\n",_motornum,(int)sumLength,spd,(int)sumRdynamic,deltaR);
        _spd[i] = spd[i];
        _rote[i] = rote[i];
    }
}

float rbms_rote::speed_pid(float T, short rpm_now, short set_speed,float *delta_rpm_pre,float *ie,float KP, float KI,float KD )//pid制御
{
    float de;
    float delta_rpm;
    delta_rpm  = set_speed - rpm_now;
    de = (delta_rpm - *delta_rpm_pre)/T;
    *ie = *ie + (delta_rpm + *delta_rpm_pre)*T/2;
    float out_torque  = KP*delta_rpm + KI*(*ie) + KD*de;
    *delta_rpm_pre = delta_rpm;
    return out_torque;
}

void rbms_rote::spd_control(int* set_speed,int* motor){
    short rotation[_motor_num], speed[_motor_num];
    float delta_rpm_pre[_motor_num], ie[_motor_num];
    Timer tm[_motor_num];
    for(int i=0;i<_motor_num;i++){//初期化
        delta_rpm_pre[i]=0.0;
        ie[i]=0.0;
        tm[i].start();
    }
    
    while(1){
        for(int id=0;id<_motor_num;id++){
            if(_msg.id==0x201+id){
                CANMessage msg=_msg;
                rbms_read(msg,&rotation[id],&speed[id]);
                if(_motor_type){
                    motor[id] = (int)speed_pid(chrono::duration_cast<chrono::milliseconds>(tm[id].elapsed_time()).count(), speed[id]/GIAR_RATIO_M3508, set_speed[id], &delta_rpm_pre[id], &ie[id], KP, KI, KD);
                }else{
                    motor[id] = (int)speed_pid(chrono::duration_cast<chrono::milliseconds>(tm[id].elapsed_time()).count(), speed[id]/GIAR_RATIO_M2006, set_speed[id], &delta_rpm_pre[id], &ie[id], KP, KI, KD);
                }
                tm[id].reset();
                if(motor[id] > _motor_max){
                    motor[id] = _motor_max;
                }else if(motor[id] < -_motor_max){
                    motor[id] = -_motor_max;
                }
            }
        }
        ThisThread::sleep_for(3ms);
    }
}

void rbms_rote::get_rote(long *rote)
{
    for(int i = 0; i < _motor_num; i++){
        rote[i] = sumRstatic[i];
    }
}

void rbms_rote::get_rote(short *rote)
{
    for(int i = 0; i < _motor_num; i++){
        rote[i] = _rote[i];
    }
}

void rbms_rote::get_rpm(short *spd){
    for(int i = 0; i < _motor_num; i++){
        spd[i] = _spd[i];
    }
}

void rbms_rote::set_static_reset(int num)
{
    for(int i = 0; i < _motor_num; i++){
        sumRstatic[i] = num;
    }       
}


