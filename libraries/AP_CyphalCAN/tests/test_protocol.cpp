#include <AP_gtest.h>
#include <AP_CyphalCAN/AP_CyphalCAN_Protocol.h>

using namespace AP_CyphalCAN_Protocol;

static AP_HAL::CANFrame message_frame(uint8_t priority, uint16_t subject, uint8_t source_node_id,
                                      const uint8_t *data, uint8_t length)
{
    AP_HAL::CANFrame frame;
    frame.id = AP_HAL::CANFrame::FlagEFF |
               (uint32_t(priority) << 26) |
               (3U << 21) |
               (uint32_t(subject) << 8) |
               source_node_id;
    frame.dlc = length;
    memcpy(frame.data, data, length);
    return frame;
}

TEST(CyphalCANProtocol, ThrottleMatchesDocumentExample)
{
    const uint16_t throttle[4] = {0x123, 0x234, 0x345, 0x456};
    const uint8_t expected[8] = {0x23, 0x01, 0x34, 0x42, 0x45, 0x03, 0x56, 0xE0};
    AP_HAL::CANFrame frame;

    ASSERT_TRUE(make_throttle_frame(0, 10, 0, throttle, frame));
    EXPECT_EQ(AP_HAL::CANFrame::FlagEFF | 0x0C78080AU, frame.id);
    EXPECT_EQ(8U, frame.dlc);
    EXPECT_FALSE(frame.canfd);
    EXPECT_EQ(0, memcmp(expected, frame.data, sizeof(expected)));
}

TEST(CyphalCANProtocol, SecondThrottleGroupAndTransferId)
{
    const uint16_t throttle[4] = {0, 2048, 1, 2048};
    const uint8_t expected[8] = {0x00, 0x00, 0x00, 0x88, 0x01, 0x00, 0x00, 0xFF};
    AP_HAL::CANFrame frame;

    ASSERT_TRUE(make_throttle_frame(1, 127, 31, throttle, frame));
    EXPECT_EQ(AP_HAL::CANFrame::FlagEFF | 0x0C78097FU, frame.id);
    EXPECT_EQ(0, memcmp(expected, frame.data, sizeof(expected)));
}

TEST(CyphalCANProtocol, ThrottleRejectsUnsafeInputs)
{
    const uint16_t valid[4] = {0, 1, 2, 3};
    const uint16_t invalid[4] = {0, 1, 2, 2049};
    AP_HAL::CANFrame frame;
    frame.id = 0x123U;

    EXPECT_FALSE(make_throttle_frame(2, 10, 0, valid, frame));
    EXPECT_FALSE(make_throttle_frame(0, 16, 0, valid, frame));
    EXPECT_FALSE(make_throttle_frame(0, 125, 0, valid, frame));
    EXPECT_FALSE(make_throttle_frame(0, 128, 0, valid, frame));
    EXPECT_FALSE(make_throttle_frame(0, 10, 32, valid, frame));
    EXPECT_FALSE(make_throttle_frame(0, 10, 0, invalid, frame));
    EXPECT_FALSE(make_throttle_frame(0, 10, 0, nullptr, frame));
    EXPECT_EQ(0x123U, frame.id);
    EXPECT_TRUE(make_throttle_frame(0, 15, 0, valid, frame));
    EXPECT_TRUE(make_throttle_frame(0, 126, 0, valid, frame));
}

TEST(CyphalCANProtocol, DecodeFeedback6160)
{
    const uint8_t payload[7] = {0xD2, 0x04, 0xF1, 0xFF, 0x18, 0x90, 0xE7};
    const AP_HAL::CANFrame frame = message_frame(5, FEEDBACK_SUBJECT_1, 0x10, payload, sizeof(payload));
    Feedback6160 feedback {};

    ASSERT_TRUE(decode_feedback_6160(frame, feedback));
    EXPECT_EQ(0x10U, feedback.source_node_id);
    EXPECT_EQ(7U, feedback.transfer_id);
    EXPECT_EQ(1234U, feedback.electrical_frequency_dHz);
    EXPECT_EQ(-15, feedback.bus_current_dA);
    EXPECT_EQ(0x9018U, feedback.status);
}

TEST(CyphalCANProtocol, DecodeFeedback6161)
{
    const uint8_t payload[8] = {0x00, 0x08, 0xF0, 0x00, 63, 40, 20, 0xEF};
    const AP_HAL::CANFrame frame = message_frame(5, FEEDBACK_SUBJECT_2, 0x11, payload, sizeof(payload));
    Feedback6161 feedback {};

    ASSERT_TRUE(decode_feedback_6161(frame, feedback));
    EXPECT_EQ(0x11U, feedback.source_node_id);
    EXPECT_EQ(15U, feedback.transfer_id);
    EXPECT_EQ(2048U, feedback.output_throttle);
    EXPECT_EQ(240, feedback.bus_voltage_dV);
    EXPECT_EQ(23, feedback.mos_temperature_c);
    EXPECT_EQ(0, feedback.capacitor_temperature_c);
    EXPECT_EQ(-20, feedback.motor_temperature_c);
}

TEST(CyphalCANProtocol, DecodeSevenByteHeartbeat)
{
    const uint8_t payload[8] = {0x78, 0x56, 0x34, 0x12, 2, 3, 0xAA, 0xE0};
    const AP_HAL::CANFrame frame = message_frame(4, HEARTBEAT_SUBJECT, 0x12, payload, sizeof(payload));
    Heartbeat heartbeat {};

    ASSERT_TRUE(decode_heartbeat_7509(frame, heartbeat));
    EXPECT_EQ(0x12U, heartbeat.source_node_id);
    EXPECT_EQ(0U, heartbeat.transfer_id);
    EXPECT_EQ(0x12345678U, heartbeat.uptime_s);
    EXPECT_EQ(2U, heartbeat.health);
    EXPECT_EQ(3U, heartbeat.mode);
    EXPECT_EQ(0xAAU, heartbeat.vendor_specific_status);
}

TEST(CyphalCANProtocol, SignedTelemetryAndTemperatureBoundaries)
{
    const uint8_t data0[7] = {0xFF, 0xFF, 0x00, 0x80, 0xFF, 0xFF, 0xE1};
    const uint8_t data1[8] = {0x00, 0x00, 0x00, 0x80, 0, 255, 40, 0xE1};
    Feedback6160 feedback0 {};
    Feedback6161 feedback1 {};

    ASSERT_TRUE(decode_feedback_6160(message_frame(5, FEEDBACK_SUBJECT_1, 0x10, data0, sizeof(data0)), feedback0));
    EXPECT_EQ(65535U, feedback0.electrical_frequency_dHz);
    EXPECT_EQ(-32768, feedback0.bus_current_dA);
    EXPECT_EQ(0xFFFFU, feedback0.status);

    ASSERT_TRUE(decode_feedback_6161(message_frame(5, FEEDBACK_SUBJECT_2, 0x10, data1, sizeof(data1)), feedback1));
    EXPECT_EQ(-32768, feedback1.bus_voltage_dV);
    EXPECT_EQ(-40, feedback1.mos_temperature_c);
    EXPECT_EQ(215, feedback1.capacitor_temperature_c);
    EXPECT_EQ(0, feedback1.motor_temperature_c);
}

TEST(CyphalCANProtocol, RejectMalformedFeedbackFrames)
{
    const uint8_t payload[8] = {0, 0, 0, 0, 0, 0, 0, 0xE0};
    Feedback6160 feedback0 {};
    Feedback6161 feedback1 {};
    Heartbeat heartbeat {};
    AP_HAL::CANFrame frame = message_frame(5, FEEDBACK_SUBJECT_1, 0x10, payload, 7);

    frame.id &= ~AP_HAL::CANFrame::FlagEFF;
    EXPECT_FALSE(decode_feedback_6160(frame, feedback0));
    frame.id |= AP_HAL::CANFrame::FlagEFF;
    frame.id |= AP_HAL::CANFrame::FlagRTR;
    EXPECT_FALSE(decode_feedback_6160(frame, feedback0));
    frame.id &= ~AP_HAL::CANFrame::FlagRTR;
    frame.id |= AP_HAL::CANFrame::FlagERR;
    EXPECT_FALSE(decode_feedback_6160(frame, feedback0));
    frame.id &= ~AP_HAL::CANFrame::FlagERR;
    frame.id ^= 1U << 21;
    EXPECT_FALSE(decode_feedback_6160(frame, feedback0));
    frame.id ^= 1U << 21;
    frame.id |= 1U << 24;
    EXPECT_FALSE(decode_feedback_6160(frame, feedback0));
    frame.id &= ~(1U << 24);
    frame.id |= 1U << 25;
    EXPECT_FALSE(decode_feedback_6160(frame, feedback0));
    frame.id &= ~(1U << 25);
    frame.id |= 1U << 23;
    EXPECT_FALSE(decode_feedback_6160(frame, feedback0));
    frame.id &= ~(1U << 23);
    frame.id |= 1U << 7;
    EXPECT_FALSE(decode_feedback_6160(frame, feedback0));
    frame.id &= ~(1U << 7);
    frame.canfd = true;
    EXPECT_FALSE(decode_feedback_6160(frame, feedback0));
    frame.canfd = false;
    frame.dlc = 6;
    EXPECT_FALSE(decode_feedback_6160(frame, feedback0));
    frame.dlc = 7;
    frame.data[6] = 0xC0;
    EXPECT_FALSE(decode_feedback_6160(frame, feedback0));
    frame.data[6] = 0xE0;
    EXPECT_FALSE(decode_feedback_6161(frame, feedback1));
    EXPECT_FALSE(decode_heartbeat_7509(frame, heartbeat));

    frame = message_frame(4, FEEDBACK_SUBJECT_1, 0x10, payload, 7);
    EXPECT_FALSE(decode_feedback_6160(frame, feedback0));
    frame = message_frame(5, FEEDBACK_SUBJECT_2, 0x10, payload, 8);
    frame.data[1] = 0x08;
    frame.data[0] = 0x01;
    EXPECT_FALSE(decode_feedback_6161(frame, feedback1));
    frame = message_frame(4, HEARTBEAT_SUBJECT, 0x10, payload, 8);
    frame.data[4] = 4;
    EXPECT_FALSE(decode_heartbeat_7509(frame, heartbeat));
    frame.data[4] = 0;
    frame.data[5] = 4;
    EXPECT_FALSE(decode_heartbeat_7509(frame, heartbeat));
}

AP_GTEST_MAIN()
