#include "FireFight.h"
#include <stdio.h>
#include <GCS_MAVLink/GCS.h> //地面站
#include <array>             // 使用标准库中的array代替C风格数组
// #include "RC_Channel.h"         //加入遥控读取通道
// #include "rover/Rover.h"

#define FRAME_LENGTH 9  // 帧长

// class Action
// {
// public:
//     int16_t record_Left_Right_pulse, record_Up_Down_pulse;
//     uint16_t record_delay;
//     Action() : record_Left_Right_pulse(0), record_Up_Down_pulse(0), record_delay(0) {} // 添加默认构建函数
//     Action(int16_t Action_record_Left_Right_pulse, int16_t Action_record_Up_Down_pulse, uint16_t Action_record_delay) : record_Left_Right_pulse(Action_record_Left_Right_pulse), record_Up_Down_pulse(Action_record_Up_Down_pulse), record_delay(Action_record_delay) {}
// };

// const volatile Action actions[MAX_ACTIONS]; // 动作数组
// std::array<Action, MAX_ACTIONS> actions;
// volatile uint8_t num_actions = 0;  // 记录的动作数量
// volatile uint8_t action_index = 0; // 执行动作

void FireFight::uart_init()
{

    // hal.serial(1)->begin(256000); // 初始化串口程序
    hal.serial(1)->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    hal.serial(1)->set_unbuffered_writes(true);
    hal.scheduler->delay(1); // 等待初始化串口
    read(1,2,2);             // 读取设备堵转电流
    hal.scheduler->delay(2); // 等待串口回复
    check_send_one(1);
    AP_Param::set_object_value(this, var_info, "LR", address_1);
    AP_Param::set_object_value(this, var_info, "UD", address_2);
    read(1, 2, 23);           // 读取设备波特率设置
    hal.scheduler->delay(2); // 等待串口回复
    check_send_one(1);
    AP_Param::set_object_value(this, var_info, "BUARD", address_1);
    // LR = (AP_Int16)address_1;
    // UD = (AP_Int16)address_2;
    // write_one(0x01, 0x0002, 10);
    // hal.scheduler->delay(100); // 上下电机堵转电流
    // write_one(0x01, 0x0003, 200);
    // hal.scheduler->delay(100); // 雾柱电机堵转电流
    // write_one(0x01, 0x0004, 1);
    // hal.scheduler->delay(100); // 堵转时间
    // hal.serial(3)->begin(115200);
    AP_Param::setup_object_defaults(this, var_info);  //调用初始化
    gcs()
        .send_text(MAV_SEVERITY_CRITICAL, // 地面站消息发送
                   "uart set ok");
}

void FireFight::read(uint8_t address_ID, uint16_t reg_adress, uint16_t reg_num) // 只需要填写寄存器ID和寄存器个数
{
    uint8_t data_to_send[10];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    data_to_send[cnt++] = address_ID; // 设备地址为01
    data_to_send[cnt++] = 0x03;       // 读取的功能码为03
    data_to_send[cnt++] = BYTE1(reg_adress);
    data_to_send[cnt++] = BYTE0(reg_adress);
    data_to_send[cnt++] = BYTE1(reg_num);
    data_to_send[cnt++] = BYTE0(reg_num);
    crc = CRC.Funct_CRC16(data_to_send, cnt); // 官方给的CRC校验
    data_to_send[cnt++] = BYTE0(crc);
    data_to_send[cnt++] = BYTE1(crc);
    hal.serial(1)->write(data_to_send, cnt);
}

void FireFight::write_one(uint8_t address_ID, uint16_t reg_adress, uint16_t reg_num) // 只需要填写寄存器ID和寄存器个数
{
    uint8_t data_to_send[10];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    data_to_send[cnt++] = address_ID; // 设备地址为01
    data_to_send[cnt++] = 0x06;       // 写入的功能码为06
    data_to_send[cnt++] = BYTE1(reg_adress);
    data_to_send[cnt++] = BYTE0(reg_adress);
    data_to_send[cnt++] = BYTE1(reg_num);
    data_to_send[cnt++] = BYTE0(reg_num);
    crc = CRC.Funct_CRC16(data_to_send, cnt); // 官方给的CRC校验
    data_to_send[cnt++] = BYTE0(crc);
    data_to_send[cnt++] = BYTE1(crc);
    hal.serial(1)->write(data_to_send, cnt);
}

void FireFight::write_two(uint8_t address_ID, uint16_t start_reg_adress, int16_t val_1, int16_t val_2) // 写两个寄存器，用于归零
{
    uint8_t data_to_send[15];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    uint16_t write_num = 0x0002;      // 怀疑写入地址错误，未验证，若验证清删除
    data_to_send[cnt++] = address_ID; // 设备地址为01
    data_to_send[cnt++] = 0x10;       // 写入的功能码为0x10
    data_to_send[cnt++] = BYTE1(start_reg_adress);
    data_to_send[cnt++] = BYTE0(start_reg_adress);
    data_to_send[cnt++] = BYTE1(write_num); // 写入数量
    data_to_send[cnt++] = BYTE0(write_num);
    data_to_send[cnt++] = 0x04; // 写入字节数
    data_to_send[cnt++] = BYTE1(val_1);
    data_to_send[cnt++] = BYTE0(val_1);
    data_to_send[cnt++] = BYTE1(val_2);
    data_to_send[cnt++] = BYTE0(val_2);
    crc = CRC.Funct_CRC16(data_to_send, cnt); // 官方给的CRC校验
    data_to_send[cnt++] = BYTE0(crc);
    data_to_send[cnt++] = BYTE1(crc);
    hal.serial(1)->write(data_to_send, cnt);
}

uint8_t FireFight::check_send_one(uint8_t addressID)
{
    // uint16_t reg_adress,reg_num;
    uint8_t num = hal.serial(1)->available(); // 读取串口有多少个数据
    uint8_t c;
    static uint8_t stat = 0, len_date = 0;
    if (num > 0)
    {
        // hal.console->printf("当前有%d个数据",num);
        for (; num > 0; num--)
        {
            c = hal.serial(1)->read();
            if (linebuf_len == 0)
            {
                if (c == 1) // ID正确
                {
                    linebuf[linebuf_len++] = c;
                }
            }
            else if (linebuf_len == 1)
            {
                if (c == 3) // 确定是读出的指令
                {
                    linebuf[linebuf_len++] = c;
                }
            }
            else if (linebuf_len == 2)
            {
                if (c == 4) // 读出4个字符
                {
                    linebuf[linebuf_len++] = c;
                    stat = 1; // 开始接受剩下的数据
                    len_date = 6;
                }
            }
            else if (stat == 1 && len_date > 0)
            {
                len_date--;
                linebuf[linebuf_len++] = c;
                if (len_date == 0)
                {
                    stat = 2;
                }
            }
            else if (stat == 2)
            {
                if (linebuf_len == FRAME_LENGTH)
                {
                    uint16_t crc = CRC.Funct_CRC16(linebuf, FRAME_LENGTH - 2);
                    if (crc == ((linebuf[FRAME_LENGTH - 2]) | linebuf[FRAME_LENGTH - 1] << 8))
                    {
                        if (linebuf[0] == 1) // 如果是消防炮消息
                        {
                            address_1 = ((linebuf[3] << 8) | linebuf[4]) / 10;
                            address_2 = ((linebuf[5] << 8) | linebuf[6]) / 10;
                            // gcs().send_text(MAV_SEVERITY_CRITICAL, "Left_Right_pulse:%d", Left_Right_pulse);
                            /* code */
                        }
                        // else if (linebuf[0] == 100) // 读取左边电机转速设定值和当前转速
                        // {
                        //     Set_Left_motor = ((linebuf[3] << 8) | linebuf[4]);
                        //     Read_Left_motor = ((linebuf[5] << 8) | linebuf[6]);
                        //     // gcs().send_text(MAV_SEVERITY_CRITICAL, "Set_Left_motor:%d", (int16_t)Set_Left_motor);
                        //     // gcs().send_text(MAV_SEVERITY_CRITICAL, "Read_Left_motor:%d", (int16_t)Read_Left_motor);
                        // }
                        // else if (linebuf[0] == 101) // 读取右边电机转速设定值和当前转速
                        // {
                        //     Set_Right_motor = ((linebuf[3] << 8) | linebuf[4]);
                        //     Read_Right_motor = ((linebuf[5] << 8) | linebuf[6]);
                        //     // gcs().send_text(MAV_SEVERITY_CRITICAL, "Read_Right_motor:%d", (int16_t)Read_Right_motor);
                        // }

                        // gcs().send_text(MAV_SEVERITY_CRITICAL, "上下的脉冲值为:%d", Up_Down_pulse);
                    }
                    linebuf_len = 0;
                    stat = 0;
                }
            }
            else
            {
                linebuf_len = 0;
                stat = 0;
            }
            // hal.serial(1)->write(c);
        }
    }

    return 0;
}

void FireFight::up_button(uint16_t val)
{
    write_one(0x01, 0x000C, val); // 发送内存地址12,指令1，按键上功能
}

void FireFight::down_button(uint16_t val)
{
    write_one(0x01, 0x000D, val); // 发送内存地址13,指令1，按键上功能
}

void FireFight::left_button(uint16_t val)
{
    write_one(0x01, 0x000E, val); // 发送内存地址14,指令1，按键上功能
}

void FireFight::right_button(uint16_t val)
{
    write_one(0x01, 0x000F, val); // 发送内存地址15,指令1，按键上功能
}

void FireFight::zhu_button(uint16_t val)
{
    write_one(0x01, 0x0010, val); // 发送内存地址16,指令1，按键上功能
}

void FireFight::wu_button(uint16_t val)
{
    write_one(0x01, 0x0011, val); // 发送内存地址17,指令1，按键上功能
}

void FireFight::upanddown_zero() // 上下行程归零
{
    write_two(0x01, 0x000C, 0, 0); // 发送内存地址12,指令1，按键上功能
}

void FireFight::leftandright_zero() // 左右行程归零
{
    write_two(0x01, 0x000E, 0, 0); // 发送内存地址12,指令1，按键上功能
}

void FireFight::wuzhu_zero()
{
    write_two(0x01, 0x0010, 0, 0); // 发送内存地址12,指令1，按键上功能
}

void FireFight::zhu_zero() // 柱归零
{
    write_one(0x01, 0x0010, 0); // 发送内存地址16,指令1，按键上功能
}

void FireFight::wu_zero() // 雾归零
{
    write_one(0x01, 0x0011, 0); // 发送内存地址17,指令1，按键上功能
}

void FireFight::valve_button(uint16_t val) // 具体定义请参考.h文件

{
    write_one(0x01, 0x0013, val);
}

void FireFight::pump_button(uint16_t val)
{
    write_one(0x01, 0x0014, val);
}

void FireFight::Record_button(uint16_t val)
{
    write_one(0x01, 0x0015, val);
}

void FireFight::playback_button(uint16_t val)
{
    write_one(0x01, 0x0016, val);
}

void FireFight::function_fire_fight(uint8_t DT_ms) // 执行周期，传入DT很重要
{

    // static uint8_t replay_flag = 0; // 当为1时候表示正在执行回放
    uint16_t under_offset = 1800;
    uint16_t low_offset = 1200;
    // int8_t exp_offset_Up_Down = 0, exp_offset_Left_Right = 0;
    uint16_t rcin_2 = Rc_In[2];
    uint16_t rcin_3 = Rc_In[3];
    uint16_t rcin_4 = Rc_In[13];
    if (abs(rcin_2 - 1500) > 100)
    {
        ((rcin_2 - 1500) > 0)? write_two(1,12,1,0):write_two(1,12,0,1);
        // {exp_offset_Up_Down = 1} : exp_offset_Up_Down = -1; // 等于1表示上，-1表示向下
    }
    else //if (replay_flag != 1)
    {
        write_two(1, 12, 0, 0);
        // exp_offset_Up_Down = 0; // 期望值给0
                                //  aim_Up_Down_pulse = Up_Down_pulse;
    }

    if (abs(rcin_3 - 1500) > 100)
    {
        ((rcin_3 - 1500) > 0)? write_two(1, 14, 0, 1):write_two(1, 14, 1, 0);
        // exp_offset_Left_Right = 1:exp_offset_Left_Right=-1; //等于1表示向右，-1表示向左边
    }
    else //if (replay_flag != 1)
    {
        write_two(1, 14, 0, 0);
        // exp_offset_Left_Right = 0; // 期望值给0
        // aim_Left_Right_pulse = Left_Right_pulse;
    }

    // write_two(1,0,exp_offset_Up_Down,exp_offset_Left_Right);

    if ((rcin_4) > under_offset)
    {
        // if (/* condition */ time_cnt_zhu == 0)
        // {
        //     wu_button(0); // 将雾清零             /* code */
        // }
        // time_cnt_zhu++;
        // if (time_cnt_zhu >= 2) // 延时一个执行周期
        // {
        //     /* code */
        //     zhu_button(1);
        //     time_cnt_zhu = 0;
        // }
        write_two(0x01,0x0010,1,0);

    }
    else if ((rcin_4) < under_offset)
    {
        // if (/* condition */ time_cnt_zhu == 0)
        // {
        //     zhu_button(0); // 将柱清零            /* code */
        // }
        // time_cnt_zhu++;
        // if (time_cnt_zhu >= 2) // 延时一个执行周期
        // {
        //     /* code */
        //     wu_button(1);
        //     time_cnt_zhu = 0;
        // }
        write_two(0x01,0x0010,0,0);
    }

    else if (((rcin_4) > low_offset) && ((rcin_4) < under_offset))
    {
        // if (/* condition */ time_cnt_zhu == 0)
        // {
        //     zhu_button(0); // 将柱清零            /* code */
        // }
        // time_cnt_zhu++;
        // if (time_cnt_zhu >= 2) // 延时一个执行周期
        // {

        //     /* code */

        //     wu_button(0);
        //     time_cnt_zhu = 0;
        //     // wu_zhu++;
        // }
        write_two(0x01,0x0010,0,1);
    }

    // if ((hal.rcin->read(5)) > under_offset) // 表示正在录制动作
    // {
    //     replay_flag = 2;
    //     write_two(1,21,1,0);  //发送录制按键按下指令

    // }
    // else if ((hal.rcin->read(5)) < low_offset)
    // {
    //     replay_flag = 1;
    //     write_two(1, 21, 0, 1); // 发送回放按键按下指令
    // }

    // else if (((hal.rcin->read(5)) > low_offset) && ((hal.rcin->read(5)) < under_offset))
    // {
    //     write_two(1, 21, 0, 0); // 发送清零指令
    // }

}



void FireFight::parm_change()
{
    static int16_t last_UD = 0,last_LR = 0,last_BUARD = 0;
    if (last_UD == 0)
        last_UD = UD;
    if (last_LR == 0)
        last_LR = LR;
    if (last_BUARD == 0)
        last_BUARD = BUARD;
    if (last_UD != UD)
    {
        last_UD = UD;
        write_one(1, 2, last_UD);  //写入参数
        write_one(1, 24, 1);  // 保存参数
    }
    if (last_LR != LR)
    {
        last_LR = LR;
        write_one(1, 3, last_LR); // 写入参数
        write_one(1, 24, 1);      // 保存参数
    }
    if (last_BUARD != BUARD)
    {
        last_BUARD = BUARD;
        write_one(1, 23, last_BUARD); // 写入参数
        write_one(1, 24, 1);      // 保存参数
    }
}

const AP_Param::GroupInfo FireFight::var_info[] =
{
        // @Param: LR
        // @DisplayName: STA_CURRENT_LR
        // @Description: 这个是左右电机堵转保护数值.
        // @Range:0 5000
        // @User: Standard
        AP_GROUPINFO("LR", 0, FireFight, LR, 3000),

        // @Param:UD
        // @DisplayName: STA_CURRENT_UD
        // @Description: 这个是左右电机堵转保护数值.
        // @Range:0 5000
        // @User:Standard
        // 注意这里是增加的名字
        AP_GROUPINFO("UD", 1, FireFight, UD, 3000),
        // @Param:BUARD
        // @DisplayName: STA_CURRENT_BUARD
        // @Description: 这个是消防炮波特率数值.
        // @Range:0 5000
        // @User:Standard
        // 注意这里是增加的名字
        AP_GROUPINFO("BUARD", 2, FireFight, BUARD, 2560),
        AP_GROUPEND
};
