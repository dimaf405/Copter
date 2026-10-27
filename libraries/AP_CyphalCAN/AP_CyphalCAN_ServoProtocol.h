#pragma once

#include <AP_HAL/CANIface.h>

// Vendor standard-frame servo protocol. This is not a Cyphal message type.
namespace AP_CyphalCAN_ServoProtocol {

enum class FeedbackType : uint8_t { Position, Status, LowVoltage, Abort };

struct Feedback {
    FeedbackType type;
    uint8_t node;
    int16_t position_ddeg;
    uint16_t current_cA;
    int8_t temperature_C;
    uint32_t abort_code;
};

bool pwm_to_position(uint16_t pwm, int16_t zero_us, int16_t us_per_100deg, int16_t &position_ddeg);
bool make_position(uint8_t node, int16_t position_ddeg, AP_HAL::CANFrame &frame);
bool make_read(uint8_t node, bool status, AP_HAL::CANFrame &frame);
bool decode(const AP_HAL::CANFrame &frame, Feedback &feedback);

} // namespace AP_CyphalCAN_ServoProtocol
