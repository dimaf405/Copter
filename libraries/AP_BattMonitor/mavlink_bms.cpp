#include "mavlink_bms.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/AP_Common.h>
#include <SRV_Channel/SRV_Channel.h>
extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo mavlink_bms::var_info[] = {

    // @Param: MAX_VOLT
    // @DisplayName: Maximum Battery Voltage
    // @Description: Maximum voltage of battery. Provides scaling of current versus voltage
    // @Range: 7 100
    // @User: Advanced

    AP_GROUPEND};

/// Constructor
mavlink_bms::mavlink_bms(AP_BattMonitor &mon,
                         AP_BattMonitor::BattMonitor_State &mon_state,
                         AP_BattMonitor_Params &params) : AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}
void mavlink_bms::read(void)
{
    rec_bms();
}


// void mavlink_bms::init()
// {
//     // Initialize the serial port for MAVLink communication
//     hal.serial(5)->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
//     hal.serial(5)->begin(115200); // Set the baud rate to 57600
//     // serial.setTimeout(1000); // Set a timeout for reading data
// }


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
            _state.cell_voltages.cells[0] = pack.voltages[0];
            gcs().send_text(MAV_SEVERITY_NOTICE, "Cell 0: %d", pack.voltages[0]);
            _state.cell_voltages.cells[1] = pack.voltages[1];
            _state.cell_voltages.cells[2] = pack.voltages[2];
            _state.cell_voltages.cells[3] = pack.voltages[3];
            _state.cell_voltages.cells[4] = pack.voltages[4];
            _state.cell_voltages.cells[5] = pack.voltages[5];
            _state.cell_voltages.cells[6] = pack.voltages[6];
            _state.cell_voltages.cells[7] = pack.voltages[7]; 
            _state.cell_voltages.cells[8] = pack.voltages[8];
            _state.cell_voltages.cells[9] = pack.voltages[9];
            _state.temperature = pack.temperature;
            _state.current_amps = pack.current_battery;
            _state.consumed_mah = pack.current_consumed;
            _state.consumed_wh = pack.energy_consumed;
            _state.time_remaining = pack.time_remaining;
            _state.instance = pack.id;       
        }
    }
}
