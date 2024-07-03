#pragma once
#ifndef _Fire_Gimbal_h_
#define _Fore_Gimbal_h_
#include <FireFight/FireFightCRC.h>

#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

extern const AP_HAL::HAL &hal;

class Fire_Gimbal
{
private:
    FireFightCRC CRC;
    void up();
    
    /* data */
public:

};



#endif