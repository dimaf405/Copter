#include "Copter_Rc.h"

__mavlink_rc_channels_t copter_rec_rc;
uint16_t *copter_rec_chan;
void Copter_Rc::MAVlink_Data_Receive_Prepare()
{
    uint8_t num = hal.serial(2)->available(); // 读取串口有多少个数据
    // gcs().send_text(MAV_SEVERITY_INFO, "num=%d",num);
    uint8_t c;
    static mavlink_message_t msg;
    static mavlink_status_t status;
    uint16_t temp1,temp3,temp4;
    uint8_t ret;
    while(num > 0)
    {
        num--;
        c = hal.serial(2)->read();
        ret = mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status);
        if (ret == MAVLINK_FRAMING_OK)
        {
            switch (msg.msgid)
            {
            case MAVLINK_MSG_ID_RC_CHANNELS:/* constant-expression */
                mavlink_msg_rc_channels_decode(&msg,&copter_rec_rc);
                copter_rec_chan = &(copter_rec_rc.chan1_raw); // 将copter_rec_chan的初始地址变成chan1_raw
                temp4 = copter_rec_chan[0];   //3
                temp1 = copter_rec_chan[2];   //4
                temp3 = copter_rec_chan[3];   //1
                copter_rec_chan[0] = temp1;
                copter_rec_chan[2] = temp3;
                copter_rec_chan[3] = temp4;
                
                //  遥控器 3通   飞控1通
                //     4通        飞控 3通
                //     1通道     飞控  4通
                // // gcs().send_text(MAV_SEVERITY_CRITICAL, "copter_rec_chan1:%d", copter_rec_chan[0]);
                // gcs().send_text(MAV_SEVERITY_CRITICAL, "copter_rec_rc2:%d", copter_rec_rc.chan2_raw);
                break;

            default:
                break;
            }
        }
    }

}