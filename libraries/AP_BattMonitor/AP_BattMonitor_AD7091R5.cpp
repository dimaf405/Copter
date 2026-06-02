#include "AP_BattMonitor_AD7091R5.h"

/**
 * @brief You can use it to Read Current and voltage of 1-3 batteries from a ADC extender IC over I2C.
 * AD7091R5 is a ADC extender and we are using it to read current and voltage of multiple batteries.
 * Examples of Pin combination:
 *  1)Pin 50 = Voltage 51,52,53 =  Current. For 3 battery combination Voltage will be same accross.
 *  2)Pin 50,51 = Voltage and Current Battery 1 -   Pin 52,53 = Voltage and Current Battery 2
 * Only the First instance of Battery Monitor will be reading the values from IC over I2C.
 * Make sure you understand the method of calculation used in this driver before using it.
 * e.g. using pin 1 on IC to read voltage of 2 batteries and pin 2 and 3 to read current from individual battery.
 * Pin number represents 50 = pin 1, 51 = pin 2 and so on 52, 53
 * BATT2_Monitor  = 24 ,  BATT3_Monitor  = 24
 * BATT2_VOLT_PIN = 50 ,  BATT3_VOLT_PIN = 50
 * BATT2_CURR_PIN = 51 ,  BATT3_CURR_PIN = 52
 *
 *
 */

#if AP_BATTERY_AD7091R5_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
//macro defines
#define AD7091R5_I2C_ADDR        0x2F // A0 and A1 tied to GND
#define AD7091R5_I2C_BUS         0
#define AD7091R5_RESET           0x02
#define AD7091R5_RESULT_ADDR     0x00
#define AD7091R5_CHAN_ADDR       0x01
#define AD7091R5_CONF_ADDR       0x02
#define AD7091R5_CH_ID(x)        ((x >> 5) & 0x03)
#define AD7091R5_RES_MASK        0x0F
#define AD7091R5_REF             3.3f
#define AD7091R5_RESOLUTION      (float)4096
#define AD7091R5_PERIOD_USEC     100000
#define AD7091R5_BASE_PIN        50


extern const AP_HAL::HAL& hal;
const AP_Param::GroupInfo AP_BattMonitor_AD7091R5::var_info[] = {

    // @Param: VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin on the AD7091R5 Ic
    // @Description: Sets the analog input pin that should be used for voltage monitoring on AD7091R5.
    // @Values: -1:Disabled
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("VOLT_PIN", 56, AP_BattMonitor_AD7091R5, _volt_pin, 0),

    // @Param: CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Sets the analog input pin that should be used for Current monitoring on AD7091R5.
    // @Values: -1:Disabled
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("CURR_PIN", 57, AP_BattMonitor_AD7091R5, _curr_pin, 0),

    // @Param: VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage of the voltage sensing pin (@PREFIX@VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT).
    // @User: Advanced
    AP_GROUPINFO("VOLT_MULT", 58, AP_BattMonitor_AD7091R5, _volt_multiplier, 0),

    // @Param: AMP_PERVLT
    // @DisplayName: Amps per volt
    // @Description: Number of amps that a 1V reading on the current sensor corresponds to.
    // @Units: A/V
    // @User: Standard
    AP_GROUPINFO("AMP_PERVLT", 59, AP_BattMonitor_AD7091R5, _curr_amp_per_volt, 0),

    // @Param: AMP_OFFSET
    // @DisplayName: AMP offset
    // @Description: Voltage offset at zero current on current sensor
    // @Units: V
    // @User: Standard
    AP_GROUPINFO("AMP_OFFSET", 60, AP_BattMonitor_AD7091R5, _curr_amp_offset, 0),

    // @Param: VLT_OFFSET
    // @DisplayName: Volage offset
    // @Description: Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("VLT_OFFSET", 61, AP_BattMonitor_AD7091R5, _volt_offset, 0),

    // Param indexes must be 56 to 61 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};


//Variable initialised to read from first instance.
AP_BattMonitor_AD7091R5::AnalogData AP_BattMonitor_AD7091R5::_analog_data[AD7091R5_NO_OF_CHANNELS];
bool AP_BattMonitor_AD7091R5::_first = true;
bool AP_BattMonitor_AD7091R5::_health = false;

/**
 * @brief Construct a new ap battmonitor ad7091r5::ap battmonitor ad7091r5 object
 *
 * @param mon
 * @param mon_state
 * @param params
 */
AP_BattMonitor_AD7091R5::AP_BattMonitor_AD7091R5(AP_BattMonitor &mon,
        AP_BattMonitor::BattMonitor_State &mon_state,
        AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

/**
 * @brief probe and initialize the sensor and register call back
 *
 */
void AP_BattMonitor_AD7091R5::init()
{
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_BMS_MAVLink, 0);
    if (_uart == nullptr)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AD7091R5: BMS port missing");
        return;
    }
}

/**
 * @brief read - read the voltage and curren
 *
 */
void AP_BattMonitor_AD7091R5::read()
{

    // WITH_SEMAPHORE(sem);
    // //copy global health status to all instances
    // _state.healthy = _health;

    // //return if system not healthy
    // if (!_state.healthy) {
    //     return;
    // }

    // //voltage conversion
    // _state.voltage = (_data_to_volt(_analog_data[volt_buff_pt].data) - _volt_offset) * _volt_multiplier;

    // //current amps conversion
    // _state.current_amps = (_data_to_volt(_analog_data[curr_buff_pt].data) - _curr_amp_offset) * _curr_amp_per_volt;
    rec_bms();

    uint32_t tnow = AP_HAL::micros();
    uint32_t dt_us = tnow - _state.last_time_micros;

    // 如果没有外部SOC，才考虑用电流积分
    if (!_soc_valid)
    {
        update_consumed(_state, dt_us);
    }

    _state.last_time_micros = tnow;
}

/**
 * @brief read all four channels and store the results
 *
 */
void AP_BattMonitor_AD7091R5::_read_adc()
{
    uint8_t data[AD7091R5_NO_OF_CHANNELS*2];
    //reset and reconfigure IC if health status is not good.
    if (!_state.healthy) {
        _initialize();
    }
    //read value
    bool ret = _dev->transfer(nullptr, 0, data, sizeof(data));
    WITH_SEMAPHORE(sem);
    if (ret) {
        for (int i=0; i<AD7091R5_NO_OF_CHANNELS; i++) {
            uint8_t chan = AD7091R5_CH_ID(data[2*i]);
            _analog_data[chan].data = ((uint16_t)(data[2*i]&AD7091R5_RES_MASK)<<8) | data[2*i+1];
        }
        _health = true;
    } else {
        _health = false;
    }
}

/**
 * @brief config the adc
 *
 * @return true
 * @return false
 */
bool AP_BattMonitor_AD7091R5::_initialize()
{
    //reset the device
    uint8_t data[3] = {AD7091R5_CONF_ADDR, AD7091R5_CONF_CMD | AD7091R5_RESET, AD7091R5_CONF_PDOWN0};

    if(_dev->transfer(data, sizeof(data), nullptr, 0)){
        //command mode, use external 3.3 reference, all channels enabled, set address pointer register to read the adc results
        uint8_t data_2[6] = {AD7091R5_CONF_ADDR, AD7091R5_CONF_CMD, AD7091R5_CONF_PDOWN0, AD7091R5_CHAN_ADDR, AD7091R5_CHAN_ALL, AD7091R5_RESULT_ADDR};
        return _dev->transfer(data_2, sizeof(data_2), nullptr, 0);
    }
    return false;
}

/**
 * @brief convert binary reading to volts
 *
 * @param data
 * @return float
 */
float AP_BattMonitor_AD7091R5::_data_to_volt(uint32_t data)
{
    return (AD7091R5_REF/AD7091R5_RESOLUTION)*data;
}

bool AP_BattMonitor_AD7091R5::capacity_remaining_pct(uint8_t &percentage) const
{
    if (_soc_valid)
    {
        percentage = _soc_pct;
        return true;
    }

    return AP_BattMonitor_Backend::capacity_remaining_pct(percentage);
}

void AP_BattMonitor_AD7091R5::rec_bms()
{
    if (_uart == nullptr)
    {
        return;
    }

    mavlink_status_t status{};
    mavlink_message_t msg{};

    while (_uart->available() > 0)
    {
        const uint8_t byte = _uart->read();

        const uint8_t framing = mavlink_frame_char_buffer(&_bms_rxmsg, &_bms_parser_status, byte, &msg, &status);
        if (framing == MAVLINK_FRAMING_BAD_CRC || framing == MAVLINK_FRAMING_BAD_SIGNATURE)
        {
            _bms_parser_status.msg_received = MAVLINK_FRAMING_INCOMPLETE;
            _bms_parser_status.parse_state = MAVLINK_PARSE_STATE_IDLE;
            if (byte == MAVLINK_STX)
            {
                _bms_parser_status.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
                _bms_rxmsg.len = 0;
                mavlink_start_checksum(&_bms_rxmsg);
            }
            continue;
        }

        if (framing != MAVLINK_FRAMING_OK)
        {
            continue;
        }

        if (msg.msgid != MAVLINK_MSG_ID_BATTERY_STATUS)
        {
            continue;
        }

        mavlink_battery_status_t pack{};
        mavlink_msg_battery_status_decode(&msg, &pack);

        // 先清空
        memset(_state.cell_voltages.cells, 0, sizeof(_state.cell_voltages.cells));

        // 你的协议是 4S，只取前4节
        uint32_t total_mv = 0;
        for (uint8_t i = 0; i < 4; i++)
        {
            const uint16_t mv = pack.voltages[i];
            _state.cell_voltages.cells[i] = mv;

            if (mv > 0 && mv != UINT16_MAX)
            {
                total_mv += mv;
            }
        }

        // 总压：mV -> V
        _state.voltage = total_mv * 0.001f;

        // 电流：cA -> A，-1 表示未知
        if (pack.current_battery != -1)
        {
            _state.current_amps = pack.current_battery * 0.01f;
        }

        // 温度：BATTERY_STATUS.temperature 单位为 1°C，INT16_MAX 表示未知
        if (pack.temperature != INT16_MAX)
        {
            _state.temperature = pack.temperature;
        }

        // SOC：0~100有效，-1表示未知
        if (pack.battery_remaining >= 0 && pack.battery_remaining <= 100)
        {
            _soc_pct = (uint8_t)pack.battery_remaining;
            _soc_valid = true;
        }
        else
        {
            _soc_valid = false;
        }

        // 你协议里 current_consumed 被定义成 systemAlert，不要写到 consumed_mah
        _system_alert = (uint32_t)pack.current_consumed;

        _state.healthy = true;
        _last_bms_ms = AP_HAL::millis();
    }

    // 超时判失联
    if ((AP_HAL::millis() - _last_bms_ms) > 2000U)
    {
        _state.healthy = false;
    }
}
#endif // AP_BATTERY_AD7091R5_ENABLED
