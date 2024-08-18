#pragma once 
#ifndef _Copter_RC_h_
#define _Copter_RC_h_
#include <GCS_MAVLink/GCS.h>
// extern __mavlink_rc_channels_t copter_rec_rc;
extern uint16_t *copter_rec_chan;
class Copter_Rc
{
private:
    /* data */
    
public:
    void MAVlink_Data_Receive_Prepare(uint8_t data);
};



#endif