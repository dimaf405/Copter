#include "AP_CyphalCAN_ServoProtocol.h"

#include <cmath>

namespace AP_CyphalCAN_ServoProtocol {

static int16_t signed_word(const uint8_t *data)
{
    const uint16_t value = uint16_t(data[0]) | (uint16_t(data[1]) << 8);
    return value <= INT16_MAX ? int16_t(value) : int16_t(int32_t(value) - 65536);
}

bool pwm_to_position(uint16_t pwm, int16_t zero_us, int16_t us_per_100deg, int16_t &position_ddeg)
{
    if (pwm == 0 || zero_us < 500 || zero_us > 2500 || us_per_100deg < 100 || us_per_100deg > 2000) {
        return false;
    }
    const int32_t position = lroundf((int32_t(pwm) - zero_us) * 1000.0f / us_per_100deg);
    if (position < -1000 || position > 1000) {
        return false;
    }
    position_ddeg = int16_t(position);
    return true;
}

bool make_position(uint8_t node, int16_t position_ddeg, AP_HAL::CANFrame &frame)
{
    if (node < 1 || node > 4 || position_ddeg < -1000 || position_ddeg > 1000) {
        return false;
    }
    const uint16_t position = uint16_t(position_ddeg);
    const uint8_t data[8] = {0x22, 0x03, 0x60, 0x00, uint8_t(position), uint8_t(position >> 8), 0, 0};
    frame = AP_HAL::CANFrame(0x600U + node, data, sizeof(data));
    return true;
}

bool make_read(uint8_t node, bool status, AP_HAL::CANFrame &frame)
{
    if (node < 1 || node > 4) {
        return false;
    }
    const uint8_t data[8] = {0x40, uint8_t(status ? 0x05 : 0x02), 0x60, 0, 0, 0, 0, 0};
    frame = AP_HAL::CANFrame(0x600U + node, data, sizeof(data));
    return true;
}

bool decode(const AP_HAL::CANFrame &frame, Feedback &feedback)
{
    if (frame.isExtended() || frame.isRemoteTransmissionRequest() || frame.isErrorFrame() || frame.isCanFDFrame()) {
        return false;
    }
    Feedback result {};
    if (frame.id >= 0x381 && frame.id <= 0x384 && frame.dlc == 4 &&
        frame.data[0] == 0x0A && frame.data[1] == 0x02 && frame.data[2] == 0 && frame.data[3] == 0) {
        result.node = frame.id - 0x380;
        result.type = FeedbackType::LowVoltage;
    } else if (frame.id >= 0x581 && frame.id <= 0x584) {
        result.node = frame.id - 0x580;
        if (frame.dlc == 4 && frame.data[2] == 0 && frame.data[3] == 0) {
            result.type = FeedbackType::Position;
            result.position_ddeg = signed_word(frame.data);
        } else if (frame.dlc == 8 && frame.data[0] == 0x4B && frame.data[1] == 0x02 &&
                   frame.data[2] == 0x60 && frame.data[3] == 0) {
            result.type = FeedbackType::Position;
            result.position_ddeg = signed_word(&frame.data[4]);
        } else if (frame.dlc == 8 && frame.data[0] == 0x43 && frame.data[1] == 0x05 &&
                   frame.data[2] == 0x60 && frame.data[3] == 0 && frame.data[7] == 0) {
            result.type = FeedbackType::Status;
            result.current_cA = uint16_t(frame.data[4]) | (uint16_t(frame.data[5]) << 8);
            result.temperature_C = frame.data[6] < 128 ? int8_t(frame.data[6]) : int8_t(int16_t(frame.data[6]) - 256);
        } else if (frame.dlc == 8 && frame.data[0] == 0x80) {
            result.type = FeedbackType::Abort;
            result.abort_code = uint32_t(frame.data[4]) | (uint32_t(frame.data[5]) << 8) |
                                (uint32_t(frame.data[6]) << 16) | (uint32_t(frame.data[7]) << 24);
        } else {
            return false;
        }
    } else {
        return false;
    }
    if (result.type == FeedbackType::Position && (result.position_ddeg < -1000 || result.position_ddeg > 1000)) {
        return false;
    }
    feedback = result;
    return true;
}

} // namespace AP_CyphalCAN_ServoProtocol
