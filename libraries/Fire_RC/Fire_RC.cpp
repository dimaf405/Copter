#include "Fire_RC.h"
#include <GCS_MAVLink/GCS.h> //地面站
int16_t Rc_In[25];
void Fire_RC::Data_Receive_Anl_Task(uint8_t *data_buf, uint16_t num)
{
    // uint16_t register_add = 0;
    // uint16_t register_num = 0;
    // int16_t write_data = 0;
    // int16_t write_data_buff[27]; // 最多同时写入27个数据
    switch (data_buf[1])
    {
    case 0x03: 
        E_g.read_Explosion_gasese();
        
        // 如果是读寄存器则执行以下函数
        // register_add = data_buf[2] << 8 | data_buf[3];
        // register_num = data_buf[4] << 8 | data_buf[5];
        // read_register_command(register_add, register_num);
        /*需要后期增加读取函数编写*/
        break;
    case 0x10:
        if ( data_buf[3] == 0x10)
        {
            RC.T0 = (data_buf[7] << 8) + data_buf[8];   //云台
            Rc_In[12] = RC.T0 * 1000 + 1000;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RC.T0:%d", RC.T0);
            RC.T2 = (data_buf[11] << 8) + data_buf[12];   //喷雾/聚焦
            Rc_In[13] = RC.T2 * 1000 + 1000;
            RC.T4 = (data_buf[15] << 8) + data_buf[16];   //车灯开关
            Rc_In[14] = RC.T4 * 1000 + 1000;
            RC.T7 = (data_buf[21] << 8) + data_buf[22];   //预留
            Rc_In[15] = RC.T7 * 1000 + 1000;
            RC.T8 = (data_buf[23] << 8) + data_buf[24];   //预留
            Rc_In[16] = RC.T8 * 1000 + 1000;
            RC.T9 = (data_buf[25] << 8) + data_buf[26];   //跟随遥控
            Rc_In[17] = RC.T9 * 1000 + 1000;
            RC.T10 = (data_buf[27] << 8) + data_buf[28];  //模式切换
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RC.T10:%d", RC.T10);
            Rc_In[18] = RC.T10 * 1000 + 1000;
            /* code */
        }
        else if(data_buf[3] == 0x50)
        {
            RC.P0_LR = data_buf[7];
            Rc_In[2] = data_buf[7] * 3.9063f + 1000;
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "Rc_In[0]:%d", Rc_In[0]);
            RC.P0_UD = data_buf[8];
            Rc_In[3] = data_buf[8] * 3.9063f + 1000;
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "Rc_In[1]:%d", Rc_In[1]);
            RC.P1_LR = data_buf[9];
            Rc_In[1] = data_buf[9] * 3.9063f + 1000;
            RC.P1_UD = data_buf[10];
            Rc_In[0] = data_buf[10] * 3.9063f + 1000;
            RC.P2_LR = data_buf[11];
            Rc_In[4] = data_buf[11] * 3.9063f + 1000;
            RC.P2_UD = data_buf[12];
            Rc_In[5] = data_buf[12] * 3.9063f + 1000;
            RC.P3_LR = data_buf[13];
            Rc_In[6] = data_buf[13] * 3.9063f + 1000;
            RC.P3_UD = data_buf[14];
            Rc_In[7] = data_buf[14] * 3.9063f + 1000;
            RC.P4_LR = data_buf[15];
            Rc_In[8] = data_buf[15] * 3.9063f + 1000;
            RC.P4_UD = data_buf[16];
            Rc_In[9] = data_buf[16] * 3.9063f + 1000;
            RC.P5_LR = data_buf[17];
            Rc_In[10] = data_buf[17] * 3.9063f + 1000;
            RC.P5_UD = data_buf[18];
            Rc_In[11] = data_buf[18] * 3.9063f + 1000;
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "RC.P0_LR:%d", RC.P0_LR);
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "RC.P0_UD:%d", RC.P0_UD);
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "RC.P1_LR:%d", RC.P1_LR);
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "RC.P1_UD:%d", RC.P1_UD);
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "RC.P2_LR:%d", RC.P2_LR);
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "RC.P2_UD:%d", RC.P2_UD);
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "RC.P3_LR:%d", RC.P3_LR);
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "RC.P3_UD:%d", RC.P3_UD);
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "RC.P4_LR:%d", RC.P4_LR);
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "RC.P4_UD:%d", RC.P4_UD);
        //     gcs().send_text(MAV_SEVERITY_CRITICAL, "RC.P5_LR:%d", RC.P5_LR);
        //     gcs().send_text(MAV_SEVERITY_CRITICAL, "RC.P5_UD:%d", RC.P5_UD);
        }
        break;
    case 0x0F:
        RC.F0 = (data_buf[7] & 0x01);
        RC.F5 = (uint8_t)((data_buf[7] & 0x20) >> 5);
        RC.F8 = (uint8_t)((data_buf[8] & 0x01) >> 0);
        RC.F18 = (uint8_t)((data_buf[9] & 0x02) >> 1);
        RC.F17 = (uint8_t)((data_buf[9] & 0x04) >> 2);
        RC.F21 = (uint8_t)((data_buf[9] & 0x20) >> 5);
        RC.F22 = (uint8_t)((data_buf[9] & 0x40) >> 6);

        break;
    }
}

int16_t Fire_RC::get_RC(uint8_t num)
{
    if (num <= 18)
    {
        return Rc_In[num]; /* code */
    }
    return -1;
}
void Fire_RC::Data_Receive_Prepare()
{
    uint8_t num = hal.serial(6)->available(); // 读取串口有多少个数据
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "num:%d", num);
    uint8_t c;
    static uint8_t stat = 0, rece_len = 0, data_len = 0;
    static uint8_t data_buff[255];
    while(num > 0)
    {
        c = hal.serial(6)->read();
        num--;
        if (stat == 0)
        {
            if (c == 0x4D)  //如果等于发射器发送的枕头0x4D,则记录
            {
                data_buff[rece_len++] = c;
                stat++;
            }
        }
        else if (stat == 1) // 接收功能码
        {
            if (c == 0x03 || c == 0x10 || c == 0x0F)
            {
                data_buff[rece_len++] = c;
                stat++;
                data_len = 4;
                /* code */
            }
            else
            {
                rece_len = 0;
                stat = 0;
            }
        }
        else if (stat == 2 && data_len > 0) // 记录寄存器地址
        {
            data_len--;
            data_buff[rece_len++] = c;  //直接接收起始地址与寄存器数量
            if (data_len == 0)
            {
                stat++;
            } 
        }
        else if (stat == 3) // 根据上面的功能码进行接下来的数据接收
        {
            data_buff[rece_len++] = c;  //这里已经接收完一个，故下面的需要减掉1
            stat++;
            switch (data_buff[1])
            {
            case 0x03:        // 如果是读取寄存器
                data_len = 1; // 如果功能码是0x03仅需接收2个CRC即可完成接受
                break;
            case 0x0F:        // 写单个寄存器
                data_len = 10; // 如果功能码是0x03仅需接收2个CRC+8个数据位+1个字节数即可完成接受
                break;
            case 0x10:
                data_len = ((data_buff[4] << 8) + data_buff[5])*2 + 2;
                break;
            default:
                rece_len = 0;
                stat = 0; // 当不是上面任何的数据的话，不进行后面的接收
                break;
            }
        }
        else if (stat == 4 && data_len > 0)
        {
            data_len--;
            data_buff[rece_len++] = c;
            if (data_len == 0) // 代表接收完毕
            {
                uint16_t crc = CRC.Funct_CRC16(data_buff, rece_len - 2);
                if (crc == ((data_buff[rece_len - 2]) | data_buff[rece_len - 1] << 8)) // 表示校验通过可以进行解析
                {
                    Data_Receive_Anl_Task(data_buff, rece_len);
                }
                rece_len = 0;
                stat = 0;
            }
        }
        else
        {
            rece_len = 0;
            stat = 0;
        }
    }
}