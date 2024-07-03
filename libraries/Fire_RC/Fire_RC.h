#pragma once 
#ifndef _Fire_RC_h_
#define _Fire_RC_h_
#include <FireFight/FireFightCRC.h>
#include <Explosion_gases/Explosion_gases.h> //添加气体检测头文件
#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL &hal;
extern int16_t Rc_In[25];
struct RC_str
{
    uint8_t T0, T2, T4, T7, T8, T9,T10;
    uint8_t P0_LR, P0_UD;
    uint8_t P1_LR, P1_UD;
    uint8_t P2_LR, P2_UD;
    uint8_t P3_LR, P3_UD;
    uint8_t P4_LR, P4_UD;
    uint8_t P5_LR, P5_UD;
    uint8_t F0;  // 急停
    uint8_t F5;  // 报警
    uint8_t F8;  // 雷达
    uint8_t F17; // 画面模式
    uint8_t F18; // 画面切换
    uint8_t F22; // 高低压
    uint8_t F21; // 喷淋

    /* data */
};
class Fire_RC
{
private:
    FireFightCRC CRC;
    Explosion_gases E_g;
    void Data_Receive_Anl_Task(uint8_t *data_buf, uint16_t num);
    RC_str RC;
    // int16_t Rc_In[25];
    /* data */
public:
    void Data_Receive_Prepare();

    int16_t get_RC(uint8_t num);
    // Fire_RC(/* args */);
    // ~Fire_RC();
};

// Fire_RC::Fire_RC(/* args */)
// {
// }

// Fire_RC::~Fire_RC()
// {
// }



#endif