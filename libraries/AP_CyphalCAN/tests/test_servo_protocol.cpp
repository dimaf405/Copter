#include <AP_gtest.h>
#include <AP_CyphalCAN/AP_CyphalCAN_ServoProtocol.h>

using namespace AP_CyphalCAN_ServoProtocol;

TEST(CyphalCANServo, PositionExamplesAndPhysicalChannelMapping)
{
    const int16_t positions[] = {-1000, -500, -109, -8, 0, 8, 109, 500, 1000};
    const uint8_t low[] = {0x18, 0x0C, 0x93, 0xF8, 0, 8, 0x6D, 0xF4, 0xE8};
    const uint8_t high[] = {0xFC, 0xFE, 0xFF, 0xFF, 0, 0, 0, 1, 3};
    for (uint8_t node = 1; node <= 4; node++) {
        for (uint8_t i = 0; i < ARRAY_SIZE(positions); i++) {
            AP_HAL::CANFrame frame;
            ASSERT_TRUE(make_position(node, positions[i], frame));
            EXPECT_EQ(0x600U + node, frame.id);
            EXPECT_FALSE(frame.isExtended());
            EXPECT_EQ(8U, frame.dlc);
            const uint8_t expected[8] = {0x22, 3, 0x60, 0, low[i], high[i], 0, 0};
            EXPECT_EQ(0, memcmp(frame.data, expected, sizeof(expected)));
        }
    }
}

TEST(CyphalCANServo, VirtualPwmConversionAndRangeRejection)
{
    int16_t angle;
    for (uint16_t pwm = 500; pwm <= 2500; pwm++) {
        ASSERT_TRUE(pwm_to_position(pwm, 1500, 1000, angle));
        EXPECT_EQ(int16_t(pwm - 1500), angle);
    }
    EXPECT_TRUE(pwm_to_position(2050, 1500, 1000, angle));
    EXPECT_EQ(550, angle);
    EXPECT_TRUE(pwm_to_position(980, 1500, 1000, angle));
    EXPECT_EQ(-520, angle);
    EXPECT_TRUE(pwm_to_position(1050, 1500, 1000, angle));
    EXPECT_EQ(-450, angle);
    EXPECT_TRUE(pwm_to_position(1700, 1600, 500, angle));
    EXPECT_EQ(200, angle);
    EXPECT_FALSE(pwm_to_position(0, 1500, 1000, angle));
    EXPECT_FALSE(pwm_to_position(2501, 1500, 1000, angle));
    EXPECT_FALSE(pwm_to_position(499, 1500, 1000, angle));
    EXPECT_FALSE(pwm_to_position(1500, 499, 1000, angle));
    EXPECT_FALSE(pwm_to_position(1500, 2501, 1000, angle));
    EXPECT_FALSE(pwm_to_position(1500, 1500, 0, angle));
    EXPECT_FALSE(pwm_to_position(1500, 1500, 2001, angle));
    AP_HAL::CANFrame frame;
    EXPECT_FALSE(make_position(0, 0, frame));
    EXPECT_FALSE(make_position(5, 0, frame));
    EXPECT_FALSE(make_position(1, -1001, frame));
    EXPECT_FALSE(make_position(1, 1001, frame));
}

TEST(CyphalCANServo, ReadCommands)
{
    AP_HAL::CANFrame frame;
    for (uint8_t node = 1; node <= 4; node++) {
        ASSERT_TRUE(make_read(node, false, frame));
        const uint8_t position[8] = {0x40, 2, 0x60, 0, 0, 0, 0, 0};
        EXPECT_EQ(0x600U + node, frame.id);
        EXPECT_EQ(0, memcmp(frame.data, position, sizeof(position)));
        ASSERT_TRUE(make_read(node, true, frame));
        EXPECT_EQ(5U, frame.data[1]);
    }
    EXPECT_FALSE(make_read(0, false, frame));
    EXPECT_FALSE(make_read(5, false, frame));
}

TEST(CyphalCANServo, PositionStatusAlarmAndAbort)
{
    Feedback feedback;
    const uint8_t automatic[4] = {0x93, 0xFF, 0, 0};
    AP_HAL::CANFrame frame(0x581, automatic, sizeof(automatic));
    ASSERT_TRUE(decode(frame, feedback));
    EXPECT_EQ(FeedbackType::Position, feedback.type);
    EXPECT_EQ(1U, feedback.node);
    EXPECT_EQ(-109, feedback.position_ddeg);
    const uint8_t position[8] = {0x4B, 2, 0x60, 0, 0xE8, 3, 0, 0};
    frame = AP_HAL::CANFrame(0x584, position, sizeof(position));
    ASSERT_TRUE(decode(frame, feedback));
    EXPECT_EQ(4U, feedback.node);
    EXPECT_EQ(1000, feedback.position_ddeg);
    const uint8_t status[8] = {0x43, 5, 0x60, 0, 0xF4, 1, 0xD8, 0};
    frame = AP_HAL::CANFrame(0x582, status, sizeof(status));
    ASSERT_TRUE(decode(frame, feedback));
    EXPECT_EQ(FeedbackType::Status, feedback.type);
    EXPECT_EQ(500U, feedback.current_cA);
    EXPECT_EQ(-40, feedback.temperature_C);
    const uint8_t alarm[4] = {0x0A, 2, 0, 0};
    frame = AP_HAL::CANFrame(0x383, alarm, sizeof(alarm));
    ASSERT_TRUE(decode(frame, feedback));
    EXPECT_EQ(FeedbackType::LowVoltage, feedback.type);
    EXPECT_EQ(3U, feedback.node);
    const uint8_t abort[8] = {0x80, 3, 0x60, 0, 0x30, 0, 9, 6};
    frame = AP_HAL::CANFrame(0x581, abort, sizeof(abort));
    ASSERT_TRUE(decode(frame, feedback));
    EXPECT_EQ(FeedbackType::Abort, feedback.type);
    EXPECT_EQ(0x06090030U, feedback.abort_code);
}

TEST(CyphalCANServo, RejectOtherProtocolsAndMalformedFeedback)
{
    Feedback feedback;
    const uint8_t data[8] = {0x4B, 2, 0x60, 0, 0, 0, 0, 0};
    AP_HAL::CANFrame frame(0x581, data, sizeof(data));
    frame.id |= AP_HAL::CANFrame::FlagEFF;
    EXPECT_FALSE(decode(frame, feedback));
    frame.id = 0x581 | AP_HAL::CANFrame::FlagRTR;
    EXPECT_FALSE(decode(frame, feedback));
    frame.id = 0x581 | AP_HAL::CANFrame::FlagERR;
    EXPECT_FALSE(decode(frame, feedback));
    frame.id = 0x581;
    frame.canfd = true;
    EXPECT_FALSE(decode(frame, feedback));
    frame.canfd = false;
    frame.id = 0x585;
    EXPECT_FALSE(decode(frame, feedback));
    frame.id = 0x581;
    frame.dlc = 7;
    EXPECT_FALSE(decode(frame, feedback));
    frame.dlc = 8;
    frame.data[1] = 3; // A write acknowledgement is not position feedback.
    EXPECT_FALSE(decode(frame, feedback));
    frame.data[1] = 2;
    frame.data[4] = 0xE9;
    frame.data[5] = 3;
    EXPECT_FALSE(decode(frame, feedback));
}

AP_GTEST_MAIN()
