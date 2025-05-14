#pragma once
#ifndef MAVLINK_BMS_H
#define MAVLINK_BMS_H
#include <AP_HAL/AP_HAL.h>
#define MAVLINK_BMS_BUFFER_SIZE 128
#define MAVLINK_BMS_PAYLOAD_SIZE 128
extern const AP_HAL::HAL &hal;

class mavlink_bms
{
private:


public:
    mavlink_bms(/* args */);
    ~mavlink_bms();
    void rec_bms();
    void init();
};

mavlink_bms::mavlink_bms(/* args */)
{
}

mavlink_bms::~mavlink_bms()
{
}


#endif // MAVLINK_BMS_H