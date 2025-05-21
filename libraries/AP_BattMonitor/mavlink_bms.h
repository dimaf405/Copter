#pragma once
#ifndef MAVLINK_BMS_H
#define MAVLINK_BMS_H
#include <AP_HAL/AP_HAL.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_BattMonitor/AP_BattMonitor_Backend.h>
extern const AP_HAL::HAL &hal;

class mavlink_bms : public AP_BattMonitor_Backend // 继承自AP_BattMonitor_Backend类，主要目的是访问_state数据，然后更新对应的数值
{
private:
    ;
public:
    // inherit constructor
    mavlink_bms(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);
    // read the latest battery voltage
    void read() override;
    void rec_bms();
    // void init() override;
    static const struct AP_Param::GroupInfo var_info[];
};



#endif // MAVLINK_BMS_H