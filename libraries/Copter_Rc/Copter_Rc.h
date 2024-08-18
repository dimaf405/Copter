#pragma once   //这个文件是负责获取mavlink的遥控器的数值的
#ifndef _Copter_RC_h_
#define _Copter_RC_h_
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL &hal;
// extern __mavlink_rc_channels_t copter_rec_rc;
extern uint16_t *copter_rec_chan;
class Copter_Rc
{
private:
    /* data */
    
public:
    void MAVlink_Data_Receive_Prepare();
};



#endif