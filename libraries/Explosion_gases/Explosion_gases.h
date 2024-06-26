#pragma once
#ifndef _Explosion_gases_h_
#include <FireFight/FireFightCRC.h>
#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))
#define Thermal_ID 0x01   //热成像ID
#define Gases_ID 0x02     //气体传感器ID
extern const AP_HAL::HAL &hal;

struct Ex_Gases
{
    int16_t Co2;    //外部一氧化碳含量
    int16_t NH3;
    int16_t temp; //设备温度
    int16_t humidity; //外部湿度

    /* data */
};

struct Thermal_imaging
{
   uint16_t x;
   uint16_t y;
   int16_t temp;
    /* data */
};


class Explosion_gases
{
private:
    FireFightCRC CRC;
    
    void Data_Receive_Anl_Task(uint8_t *data_buf, uint16_t num);//数据处理函数
    void Data_Receive_Prepare(uint8_t device_ID);               // 数据接收准备
    void read(uint8_t address_ID, uint16_t reg_adress, uint16_t reg_num); // 只需要填写寄存器ID和寄存器个数
    /* data */
public:
    Ex_Gases gases;
    Thermal_imaging Ther_ima;
    void get_Thermal_imaging();    //读取热成像数据
    void read_Explosion_gasese();  //读取气体传感器数据
};



#endif