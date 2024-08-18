#include "Copter_Rc.h"

__mavlink_rc_channels_t copter_rec_rc;
uint16_t *copter_rec_chan;
void Copter_Rc::MAVlink_Data_Receive_Prepare()
{
    uint8_t num = hal.serial(2)->available(); // 读取串口有多少个数据
    uint8_t c;
    static mavlink_message_t msg;
    static mavlink_status_t status;
    uint8_t ret;
    while(num > 0)
    {
        c = hal.serial(2)->read();
        ret = mavlink_parse_char(MAVLINK_COMM_3, c, &msg, &status);
        if (ret == MAVLINK_FRAMING_OK)
        {
            switch (msg.msgid)
            {
            case MAVLINK_MSG_ID_RC_CHANNELS:/* constant-expression */
                mavlink_msg_rc_channels_decode(&msg,&copter_rec_rc);
                copter_rec_chan = &(copter_rec_rc.chan1_raw);
                // gcs().send_text(MAV_SEVERITY_CRITICAL, "copter_rec_chan1:%d", copter_rec_chan[0]);
                // gcs().send_text(MAV_SEVERITY_CRITICAL, "copter_rec_rc2:%d", copter_rec_rc.chan2_raw);
                break;

            default:
                break;
            }
        }
    }

}