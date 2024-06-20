#pragma once
#ifndef _Explosion_gases_h_
#include <FireFight/FireFightCRC.h>
#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))
extern const AP_HAL::HAL &hal;

struct Ex_Gases
{
    int16_t Co;    //外部一氧化碳含量
    int16_t temp; //设备温度
    int16_t humidity; //外部湿度

    /* data */
};


class Explosion_gases
{
private:
    FireFightCRC CRC;
    
    void Data_Receive_Anl_Task(uint8_t *data_buf, uint16_t num);//数据处理函数
    void Data_Receive_Prepare();                                // 数据接收准备
    void read(uint8_t address_ID, uint16_t reg_adress, uint16_t reg_num); // 只需要填写寄存器ID和寄存器个数
    /* data */
public:
    Ex_Gases gases;
    void read_Explosion_gasese();  //获取气体数据
 
};



#endif