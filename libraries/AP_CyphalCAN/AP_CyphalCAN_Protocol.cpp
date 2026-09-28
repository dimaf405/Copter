#include "AP_CyphalCAN_Protocol.h"

namespace AP_CyphalCAN_Protocol {

static constexpr uint32_t MESSAGE_RESERVED_BITS = 3U << 21;
static constexpr uint8_t SINGLE_FRAME_TAIL = 0xE0;

static uint16_t read_u16_le(const uint8_t *data)
{
    return uint16_t(data[0]) | (uint16_t(data[1]) << 8);
}

static int16_t read_i16_le(const uint8_t *data)
{
    const uint16_t value = read_u16_le(data);
    return value <= INT16_MAX ? int16_t(value) : int16_t(int32_t(value) - 65536);
}

static bool valid_single_message(const AP_HAL::CANFrame &frame, uint16_t subject,
                                 uint8_t priority, uint8_t length)
{
    if (!frame.isExtended() || frame.isRemoteTransmissionRequest() || frame.isErrorFrame() ||
        frame.isCanFDFrame() || frame.dlc != length) {
        return false;
    }

    const uint32_t expected_id = AP_HAL::CANFrame::FlagEFF |
                                 (uint32_t(priority) << 26) |
                                 MESSAGE_RESERVED_BITS |
                                 (uint32_t(subject) << 8) |
                                 (frame.id & 0x7FU);
    return frame.id == expected_id && (frame.data[length - 1] & 0xE0U) == SINGLE_FRAME_TAIL;
}

bool make_throttle_frame(uint8_t group, uint8_t source_node_id, uint8_t transfer_id,
                         const uint16_t throttle[4], AP_HAL::CANFrame &frame)
{
    if (group > 1 || transfer_id > 31 ||
        !(source_node_id <= 15 || source_node_id == 126 || source_node_id == 127) ||
        throttle == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < 4; i++) {
        if (throttle[i] > 2048) {
            return false;
        }
    }

    AP_HAL::CANFrame result;
    result.id = AP_HAL::CANFrame::FlagEFF |
                (3U << 26) |
                MESSAGE_RESERVED_BITS |
                (uint32_t(THROTTLE_SUBJECT_1 + group) << 8) |
                source_node_id;
    result.dlc = 8;

    for (uint8_t i = 0; i < 3; i++) {
        const uint16_t packed = throttle[i] | ((uint32_t(throttle[3]) << (2U * (i + 1U))) & 0xC000U);
        result.data[2U * i] = uint8_t(packed);
        result.data[2U * i + 1U] = uint8_t(packed >> 8);
    }
    result.data[6] = uint8_t(throttle[3]);
    result.data[7] = SINGLE_FRAME_TAIL | transfer_id;
    frame = result;
    return true;
}

bool decode_feedback_6160(const AP_HAL::CANFrame &frame, Feedback6160 &feedback)
{
    if (!valid_single_message(frame, FEEDBACK_SUBJECT_1, 5, 7)) {
        return false;
    }

    feedback.source_node_id = uint8_t(frame.id & 0x7FU);
    feedback.transfer_id = frame.data[6] & 0x1FU;
    feedback.electrical_frequency_dHz = read_u16_le(&frame.data[0]);
    feedback.bus_current_dA = read_i16_le(&frame.data[2]);
    feedback.status = read_u16_le(&frame.data[4]);
    return true;
}

bool decode_feedback_6161(const AP_HAL::CANFrame &frame, Feedback6161 &feedback)
{
    if (!valid_single_message(frame, FEEDBACK_SUBJECT_2, 5, 8)) {
        return false;
    }

    const uint16_t throttle = read_u16_le(&frame.data[0]);
    if (throttle > 2048) {
        return false;
    }

    feedback.source_node_id = uint8_t(frame.id & 0x7FU);
    feedback.transfer_id = frame.data[7] & 0x1FU;
    feedback.output_throttle = throttle;
    feedback.bus_voltage_dV = read_i16_le(&frame.data[2]);
    feedback.mos_temperature_c = int16_t(frame.data[4]) - 40;
    feedback.capacitor_temperature_c = int16_t(frame.data[5]) - 40;
    feedback.motor_temperature_c = int16_t(frame.data[6]) - 40;
    return true;
}

bool decode_heartbeat_7509(const AP_HAL::CANFrame &frame, Heartbeat &heartbeat)
{
    // The PDF labels this message "6byte", but fields Payload[0..6] total seven bytes.
    if (!valid_single_message(frame, HEARTBEAT_SUBJECT, 4, 8) ||
        frame.data[4] > 3 || frame.data[5] > 3) {
        return false;
    }

    heartbeat.source_node_id = uint8_t(frame.id & 0x7FU);
    heartbeat.transfer_id = frame.data[7] & 0x1FU;
    heartbeat.uptime_s = uint32_t(frame.data[0]) |
                         (uint32_t(frame.data[1]) << 8) |
                         (uint32_t(frame.data[2]) << 16) |
                         (uint32_t(frame.data[3]) << 24);
    heartbeat.health = frame.data[4];
    heartbeat.mode = frame.data[5];
    heartbeat.vendor_specific_status = frame.data[6];
    return true;
}

} // namespace AP_CyphalCAN_Protocol
