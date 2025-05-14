#include "mavlink_bms.h"
#include <GCS_MAVLink/GCS.h>

void mavlink_bms::init()
{
    // Initialize the serial port for MAVLink communication
    hal.serial(5)->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    hal.serial(5)->begin(115200); // Set the baud rate to 57600
    // serial.setTimeout(1000); // Set a timeout for reading data
}


void mavlink_bms::rec_bms()
{
    mavlink_status_t status;
    mavlink_message_t msg;
    int chan = 0;

    while (hal.serial(5)->available() > 0)
    {
        uint8_t byte = hal.serial(5)->read();
        if (mavlink_parse_char(chan, byte, &msg, &status))
        {
            mavlink_battery_status_t pack;
            mavlink_msg_battery_status_decode(&msg, &pack);
        
        }
    }
}
