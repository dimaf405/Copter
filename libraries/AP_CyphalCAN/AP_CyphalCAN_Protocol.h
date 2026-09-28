#pragma once

#include <cstring>
#include <AP_HAL/CANIface.h>

namespace AP_CyphalCAN_Protocol {

static constexpr uint16_t THROTTLE_SUBJECT_1 = 6152;
static constexpr uint16_t THROTTLE_SUBJECT_2 = 6153;
static constexpr uint16_t FEEDBACK_SUBJECT_1 = 6160;
static constexpr uint16_t FEEDBACK_SUBJECT_2 = 6161;
static constexpr uint16_t HEARTBEAT_SUBJECT = 7509;

struct Feedback6160 {
    uint8_t source_node_id;
    uint8_t transfer_id;
    uint16_t electrical_frequency_dHz;
    int16_t bus_current_dA;
    uint16_t status;
};

struct Feedback6161 {
    uint8_t source_node_id;
    uint8_t transfer_id;
    uint16_t output_throttle;
    int16_t bus_voltage_dV;
    int16_t mos_temperature_c;
    int16_t capacitor_temperature_c;
    int16_t motor_temperature_c;
};

struct Heartbeat {
    uint8_t source_node_id;
    uint8_t transfer_id;
    uint32_t uptime_s;
    uint8_t health;
    uint8_t mode;
    uint8_t vendor_specific_status;
};

bool make_throttle_frame(uint8_t group, uint8_t source_node_id, uint8_t transfer_id,
                         const uint16_t throttle[4], AP_HAL::CANFrame &frame);

bool decode_feedback_6160(const AP_HAL::CANFrame &frame, Feedback6160 &feedback);
bool decode_feedback_6161(const AP_HAL::CANFrame &frame, Feedback6161 &feedback);
bool decode_heartbeat_7509(const AP_HAL::CANFrame &frame, Heartbeat &heartbeat);

} // namespace AP_CyphalCAN_Protocol
