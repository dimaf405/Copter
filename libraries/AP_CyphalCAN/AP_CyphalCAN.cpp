#include "AP_CyphalCAN.h"

#if AP_CYPHALCAN_ENABLED

#include "AP_CyphalCAN_Protocol.h"
#include "AP_CyphalCAN_ServoProtocol.h"

#include <AP_CANManager/AP_CANManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <SRV_Channel/SRV_Channel.h>

#include <cmath>
#include <cstring>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_CyphalCAN::var_info[] = {
    // @Param: NODE
    // @DisplayName: CyphalCAN controller node ID
    // @Description: Source node ID for ESC throttle messages. Valid values are 0 to 15, 126 and 127.
    // @Range: 0 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("NODE", 1, AP_CyphalCAN, _source_node_id, 10),

    // @Param: ESC_ID
    // @DisplayName: First CyphalCAN ESC feedback node ID
    // @Description: First ESC node ID for feedback indexing. Does not change throttle frame IDs, slots, or ESC node IDs.
    // @Range: 16 40
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("ESC_ID", 2, AP_CyphalCAN, _esc_base_id, 16),

    // @Param: ESC_RT
    // @DisplayName: CyphalCAN ESC throttle rate
    // @Description: Frequency of both four-motor throttle message groups.
    // @Units: Hz
    // @Range: 50 400
    // @User: Advanced
    AP_GROUPINFO("ESC_RT", 3, AP_CyphalCAN, _esc_rate_hz, 200),

    // @Param: POLES
    // @DisplayName: Motor pole pairs
    // @Description: Motor pole pairs for conversion of electrical frequency to mechanical RPM. Zero disables RPM reporting.
    // @Range: 0 64
    // @User: Advanced
    AP_GROUPINFO("POLES", 4, AP_CyphalCAN, _pole_pairs, 0),

    // @Param: ESC_BM
    // @DisplayName: CyphalCAN ESC channels
    // @Description: Motor throttle slot mask. Zero disables sending; otherwise both groups broadcast with unselected slots zero.
    // @Bitmask: 0:Motor 1,1:Motor 2,2:Motor 3,3:Motor 4,4:Motor 5,5:Motor 6,6:Motor 7,7:Motor 8
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("ESC_BM", 5, AP_CyphalCAN, _esc_mask, 0),

    // @Param: S_BM
    // @DisplayName: Standard CAN tilt servo mask
    // @Description: Enable vendor standard-frame CAN servos on this driver's bus. Node IDs 1 to 4 follow physical SERVO1 to SERVO4 final PWM values. Zero disables the feature. Set node IDs separately with the vendor tool. FUNCTION=0 or PWM=0 stops sending positions but does not release servo torque.
    // @Bitmask: 0:SERVO1 node 1,1:SERVO2 node 2,2:SERVO3 node 3,3:SERVO4 node 4
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("S_BM", 6, AP_CyphalCAN, _servo_mask, 0),

    // @Param: S_CAL
    // @DisplayName: CAN tilt servo calibration confirmed
    // @Description: Set to one only after checking new servo centers, direction and the entire configured travel without propellers or mechanical load. Enables position transmission for S_BM channels. Zero permits read-only feedback polling. This protocol has no documented torque-off or communication-loss action; stopping commands cannot guarantee torque release.
    // @Values: 0:Read only,1:Calibration confirmed
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("S_CAL", 7, AP_CyphalCAN, _servo_calibrated, 0),

    // @Param: S_RATE
    // @DisplayName: CAN tilt servo command rate
    // @Description: Position command rate for each selected servo. Independent of SERVO_RATE and ESC_RT. Position feedback is polled at 5 Hz and status at 1 Hz per selected servo.
    // @Units: Hz
    // @Range: 10 100
    // @User: Advanced
    AP_GROUPINFO("S_RATE", 8, AP_CyphalCAN, _servo_rate_hz, 50),

    // @Param: S1_Z
    // @DisplayName: CAN servo 1 zero angle PWM
    // @Description: Virtual PWM value corresponding to zero degrees of CAN servo node 1. Does not program the servo's stored mechanical center. Check with linkage disconnected.
    // @Units: us
    // @Range: 500 2500
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("S1_Z", 9, AP_CyphalCAN, _servo_zero[0], 1500),

    // @Param: S2_Z
    // @DisplayName: CAN servo 2 zero angle PWM
    // @Description: Virtual PWM value corresponding to zero degrees of CAN servo node 2. Does not program the servo's stored mechanical center.
    // @Units: us
    // @Range: 500 2500
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("S2_Z", 10, AP_CyphalCAN, _servo_zero[1], 1500),

    // @Param: S3_Z
    // @DisplayName: CAN servo 3 zero angle PWM
    // @Description: Virtual PWM value corresponding to zero degrees of CAN servo node 3. Does not program the servo's stored mechanical center.
    // @Units: us
    // @Range: 500 2500
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("S3_Z", 11, AP_CyphalCAN, _servo_zero[2], 1500),

    // @Param: S4_Z
    // @DisplayName: CAN servo 4 zero angle PWM
    // @Description: Virtual PWM value corresponding to zero degrees of CAN servo node 4. Does not program the servo's stored mechanical center.
    // @Units: us
    // @Range: 500 2500
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("S4_Z", 12, AP_CyphalCAN, _servo_zero[3], 1500),

    // @Param: S_SPAN
    // @DisplayName: CAN servo PWM travel per 100 degrees
    // @Description: PWM change corresponding to 100 degrees. Position in 0.1 degrees is (PWM-Sn_Z)*1000/S_SPAN. Default 1000 matches 500/1500/2500 us for -100/0/100 degrees. Out-of-range targets are rejected, not clipped. Confirm the new servo's mechanical calibration before use.
    // @Units: us
    // @Range: 100 2000
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("S_SPAN", 13, AP_CyphalCAN, _servo_span, 1000),

    AP_GROUPEND
};

AP_CyphalCAN::AP_CyphalCAN(uint8_t driver_index) :
    _driver_index(driver_index)
{
    AP_Param::setup_object_defaults(this, var_info);
}

AP_CyphalCAN *AP_CyphalCAN::get_cyphalcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::CyphalCAN) {
        return nullptr;
    }
    return static_cast<AP_CyphalCAN *>(AP::can().get_driver(driver_index));
}

bool AP_CyphalCAN::add_interface(AP_HAL::CANIface *can_iface)
{
    if (_can_iface != nullptr || can_iface == nullptr || !can_iface->is_initialized()) {
        return false;
    }
    if (!can_iface->set_event_handle(&_can_event)) {
        return false;
    }
    _can_iface = can_iface;
    return true;
}

void AP_CyphalCAN::init(uint8_t driver_index)
{
    if (_initialized || _can_iface == nullptr || driver_index != _driver_index) {
        return;
    }
    _active_source_node_id = _source_node_id;
    _active_esc_base_id = _esc_base_id;
    _active_esc_mask = _esc_mask;
    _active_servo_mask = _servo_mask;
    _active_servo_calibrated = _servo_calibrated == 1;
    _active_servo_span = _servo_span;
    for (uint8_t i = 0; i < 4; i++) {
        _active_servo_zero[i] = _servo_zero[i];
    }
    snprintf(_thread_name, sizeof(_thread_name), "CyphalCAN_%u", driver_index);
    _initialized = hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_CyphalCAN::loop, void),
                                                 _thread_name, 4096,
                                                 AP_HAL::Scheduler::PRIORITY_MAIN, 1);
}

void AP_CyphalCAN::SRV_push()
{
    uint16_t throttle[NUM_ESCS] {};
    const uint16_t esc_mask = _active_esc_mask;
    for (uint8_t i = 0; i < NUM_ESCS; i++) {
        if ((esc_mask & (1U << i)) == 0) {
            continue;
        }
        const SRV_Channel::Function motor_function = SRV_Channels::get_motor_function(i);
        uint16_t pwm;
        if (!SRV_Channels::get_output_pwm(motor_function, pwm)) {
            continue;
        }
        const float scaled = hal.rcout->scale_esc_to_unity(pwm);
        if (std::isfinite(scaled)) {
            throttle[i] = uint16_t(lroundf(constrain_float((scaled + 1.0f) * 1024.0f, 0.0f, 2048.0f)));
        }
    }

    int16_t servo_target[4] {};
    uint8_t servo_valid_mask = 0;
    for (uint8_t i = 0; i < 4; i++) {
        const SRV_Channel *const channel = SRV_Channels::srv_channel(i);
        if ((_active_servo_mask & (1U << i)) == 0 || channel == nullptr ||
            channel->get_function() == SRV_Channel::k_none) {
            continue;
        }
        const uint16_t pwm = channel->get_output_pwm();
        if (channel->get_function() == SRV_Channel::k_motor_tilt &&
            channel->get_output_min() < channel->get_output_max() &&
            pwm >= channel->get_output_min() && pwm <= channel->get_output_max() &&
            AP_CyphalCAN_ServoProtocol::pwm_to_position(pwm, _active_servo_zero[i],
                                                        _active_servo_span, servo_target[i])) {
            servo_valid_mask |= 1U << i;
        }
    }

    WITH_SEMAPHORE(_state_sem);
    memcpy(_throttle, throttle, sizeof(_throttle));
    memcpy(_servo_target, servo_target, sizeof(_servo_target));
    _servo_valid_mask = servo_valid_mask;
    _last_push_us = AP_HAL::micros();
}

bool AP_CyphalCAN::send_frame(const AP_HAL::CANFrame &frame)
{
    bool read_select = false;
    bool write_select = true;
    const uint64_t deadline_us = AP_HAL::micros64() + 1000;
    if (!_can_iface->select(read_select, write_select, &frame, deadline_us) || !write_select) {
        return false;
    }
    return _can_iface->send(frame, deadline_us, AP_HAL::CANIface::AbortOnError) == 1;
}

bool AP_CyphalCAN::read_frame(AP_HAL::CANFrame &frame, uint32_t timeout_us)
{
    bool read_select = true;
    bool write_select = false;
    if (!_can_iface->select(read_select, write_select, nullptr, AP_HAL::micros64() + timeout_us) ||
        !read_select) {
        return false;
    }
    uint64_t timestamp_us;
    AP_HAL::CANIface::CanIOFlags flags {};
    return _can_iface->receive(frame, timestamp_us, flags) == 1;
}

void AP_CyphalCAN::send_throttle()
{
    if (_active_esc_mask == 0) {
        return;
    }
    uint16_t throttle[NUM_ESCS] {};
    {
        WITH_SEMAPHORE(_state_sem);
        if (hal.util->get_soft_armed() &&
            hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED &&
            !SRV_Channels::get_emergency_stop() &&
            _last_push_us != 0 &&
            (AP_HAL::micros() - _last_push_us) < OUTPUT_TIMEOUT_US) {
            memcpy(throttle, _throttle, sizeof(throttle));
        }
    }

    const uint16_t esc_mask = _active_esc_mask;
    for (uint8_t group = 0; group < 2; group++) {
        uint16_t group_throttle[4] {};
        for (uint8_t i = 0; i < 4; i++) {
            const uint8_t idx = group * 4 + i;
            if ((esc_mask & (1U << idx)) != 0) {
                group_throttle[i] = throttle[idx];
            }
        }
        AP_HAL::CANFrame frame;
        if (AP_CyphalCAN_Protocol::make_throttle_frame(group, _active_source_node_id,
                                                        _transfer_id[group], group_throttle, frame) &&
            send_frame(frame)) {
            _transfer_id[group] = (_transfer_id[group] + 1U) & 31U;
        }
    }
}

void AP_CyphalCAN::send_servos()
{
    if (!_active_servo_calibrated || _active_servo_mask <= 0 || _active_servo_mask > 15 ||
        hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED ||
        SRV_Channels::get_emergency_stop()) {
        return;
    }
    int16_t targets[4] {};
    uint8_t valid_mask;
    {
        WITH_SEMAPHORE(_state_sem);
        if (_last_push_us == 0 || AP_HAL::micros() - _last_push_us >= OUTPUT_TIMEOUT_US) {
            return;
        }
        valid_mask = _servo_valid_mask;
        memcpy(targets, _servo_target, sizeof(targets));
    }
    // Servo positioning is allowed while disarmed, like the original PWM outputs.
    // No torque-disable command is defined in the supplied vendor protocol.
    for (uint8_t i = 0; i < 4; i++) {
        if ((valid_mask & (1U << i)) == 0) {
            continue;
        }
        AP_HAL::CANFrame frame;
        if (AP_CyphalCAN_ServoProtocol::make_position(i + 1, targets[i], frame) && !send_frame(frame)) {
            WITH_SEMAPHORE(_state_sem);
            _servo_tx_failures++;
        }
    }
}

void AP_CyphalCAN::poll_servos()
{
    if (_active_servo_mask <= 0 || _active_servo_mask > 15) {
        return;
    }
    // 24 requests/s: five position reads and one status read per node per second.
    // Only one request is issued per slot; position writes have no SDO response.
    const uint8_t index = _servo_poll_index;
    const bool status = _servo_poll_count == 5;
    if ((_active_servo_mask & (1U << index)) != 0) {
        AP_HAL::CANFrame frame;
        if (AP_CyphalCAN_ServoProtocol::make_read(index + 1, status, frame) && !send_frame(frame)) {
            WITH_SEMAPHORE(_state_sem);
            _servo_tx_failures++;
        }
    }
    _servo_poll_index = (_servo_poll_index + 1U) % 4U;
    if (_servo_poll_index == 0) {
        _servo_poll_count = (_servo_poll_count + 1U) % 6U;
    }
}

void AP_CyphalCAN::log_servos()
{
#if HAL_LOGGING_ENABLED
    if (_active_servo_mask <= 0 || _active_servo_mask > 15) {
        return;
    }
    ServoState states[4];
    int16_t targets[4];
    uint32_t tx_failures;
    uint8_t valid_mask;
    {
        WITH_SEMAPHORE(_state_sem);
        memcpy(states, _servos, sizeof(states));
        memcpy(targets, _servo_target, sizeof(targets));
        tx_failures = _servo_tx_failures;
        valid_mask = _servo_valid_mask;
    }
    for (uint8_t i = 0; i < 4; i++) {
        if ((_active_servo_mask & (1U << i)) == 0) {
            continue;
        }
        const auto &state = states[i];
        // Flags: target valid, position fresh, status fresh, low-voltage latched.
        const uint8_t flags = ((valid_mask >> i) & 1U) |
                              ((state.position_ms != 0 && AP_HAL::millis() - state.position_ms < 1000U) ? 2U : 0U) |
                              ((state.status_ms != 0 && AP_HAL::millis() - state.status_ms < 2500U) ? 4U : 0U) |
                              (state.low_voltage ? 8U : 0U);
        AP::logger().Write("CSRV", "TimeUS,Bus,Node,Target,Pos,Cur,Temp,Flags,Err,TxFail", "QBBhhHbBII",
                           AP_HAL::micros64(), uint8_t(_driver_index + 1), uint8_t(i + 1),
                           targets[i], state.position_ddeg, state.current_cA, state.temperature_C,
                           flags, state.abort_code, tx_failures);
    }
#endif
}

void AP_CyphalCAN::handle_frame(const AP_HAL::CANFrame &frame)
{
    AP_CyphalCAN_ServoProtocol::Feedback servo_feedback;
    if (_active_servo_mask > 0 && _active_servo_mask <= 15 &&
        AP_CyphalCAN_ServoProtocol::decode(frame, servo_feedback) &&
        (_active_servo_mask & (1U << (servo_feedback.node - 1U))) != 0) {
        WITH_SEMAPHORE(_state_sem);
        auto &state = _servos[servo_feedback.node - 1U];
        switch (servo_feedback.type) {
        case AP_CyphalCAN_ServoProtocol::FeedbackType::Position:
            state.position_ddeg = servo_feedback.position_ddeg;
            state.position_ms = AP_HAL::millis();
            break;
        case AP_CyphalCAN_ServoProtocol::FeedbackType::Status:
            state.current_cA = servo_feedback.current_cA;
            state.temperature_C = servo_feedback.temperature_C;
            state.status_ms = AP_HAL::millis();
            break;
        case AP_CyphalCAN_ServoProtocol::FeedbackType::LowVoltage:
            state.low_voltage = true;
            break;
        case AP_CyphalCAN_ServoProtocol::FeedbackType::Abort:
            state.abort_code = servo_feedback.abort_code;
            break;
        }
        return;
    }
    AP_CyphalCAN_Protocol::Feedback6160 status;
    AP_CyphalCAN_Protocol::Feedback6161 feedback;
    AP_CyphalCAN_Protocol::Heartbeat heartbeat;

    if (AP_CyphalCAN_Protocol::decode_feedback_6160(frame, status)) {
        const int16_t idx = status.source_node_id - _active_esc_base_id;
        if (idx < 0 || idx >= NUM_ESCS) {
            return;
        }
        {
            WITH_SEMAPHORE(_state_sem);
            _escs[idx].status = status.status;
            _escs[idx].status_ms = AP_HAL::millis();
        }
#if HAL_WITH_ESC_TELEM
        TelemetryData data {};
        data.current = status.bus_current_dA * 0.1f;
        update_telem_data(idx, data, CURRENT);
        if (_pole_pairs > 0) {
            update_rpm(idx, status.electrical_frequency_dHz * 6.0f / _pole_pairs);
        }
#endif
    } else if (AP_CyphalCAN_Protocol::decode_feedback_6161(frame, feedback)) {
        const int16_t idx = feedback.source_node_id - _active_esc_base_id;
        if (idx < 0 || idx >= NUM_ESCS) {
            return;
        }
        {
            WITH_SEMAPHORE(_state_sem);
            _escs[idx].output_throttle = feedback.output_throttle;
            _escs[idx].feedback_ms = AP_HAL::millis();
        }
#if HAL_WITH_ESC_TELEM
        TelemetryData data {};
        data.voltage = feedback.bus_voltage_dV * 0.1f;
        data.temperature_cdeg = feedback.mos_temperature_c * 100;
        data.motor_temp_cdeg = feedback.motor_temperature_c * 100;
        update_telem_data(idx, data, VOLTAGE | TEMPERATURE | MOTOR_TEMPERATURE);
#endif
    } else if (AP_CyphalCAN_Protocol::decode_heartbeat_7509(frame, heartbeat)) {
        const int16_t idx = heartbeat.source_node_id - _active_esc_base_id;
        if (idx < 0 || idx >= NUM_ESCS) {
            return;
        }
        WITH_SEMAPHORE(_state_sem);
        _escs[idx].health = heartbeat.health;
        _escs[idx].mode = heartbeat.mode;
        _escs[idx].heartbeat_ms = AP_HAL::millis();
    }
}

void AP_CyphalCAN::loop()
{
    uint64_t next_tx_us = AP_HAL::micros64();
    uint64_t next_servo_us = next_tx_us;
    uint64_t next_poll_us = next_tx_us;
    uint64_t next_log_us = next_tx_us;
    while (true) {
        const uint64_t now_us = AP_HAL::micros64();
        if (now_us >= next_tx_us) {
            send_throttle();
            const uint16_t rate_hz = constrain_int16(_esc_rate_hz, 50, 400);
            next_tx_us = now_us + 1000000U / rate_hz;
        }
        if (now_us >= next_servo_us) {
            send_servos();
            next_servo_us = now_us + 1000000U / constrain_int16(_servo_rate_hz, 10, 100);
        }
        if (now_us >= next_poll_us) {
            poll_servos();
            next_poll_us = now_us + 41666U;
        }
        if (now_us >= next_log_us) {
            log_servos();
            next_log_us = now_us + 1000000U;
        }

        const uint64_t remaining_us = next_tx_us > now_us ? next_tx_us - now_us : 0;
        const uint32_t wait_us = uint32_t(MIN(remaining_us, uint64_t(1000)));
        AP_HAL::CANFrame frame;
        if (read_frame(frame, wait_us)) {
            handle_frame(frame);
            for (uint8_t i = 0; i < 31 && read_frame(frame, 0); i++) {
                handle_frame(frame);
            }
        }
    }
}

bool AP_CyphalCAN::prearm_check(char *reason, uint8_t reason_len) const
{
    if (!_initialized || _can_iface == nullptr) {
        snprintf(reason, reason_len, "driver not initialized");
        return false;
    }
    if (_source_node_id != _active_source_node_id ||
        _esc_base_id != _active_esc_base_id || _esc_mask != _active_esc_mask ||
        _servo_mask != _active_servo_mask || (_servo_calibrated == 1) != _active_servo_calibrated ||
        _servo_span != _active_servo_span) {
        snprintf(reason, reason_len, "reboot required after parameter change");
        return false;
    }
    if (_active_source_node_id < 0 ||
        (_active_source_node_id > 15 && _active_source_node_id != 126 && _active_source_node_id != 127)) {
        snprintf(reason, reason_len, "invalid controller node ID");
        return false;
    }
    if (_active_esc_base_id < 16 || _active_esc_base_id > 40) {
        snprintf(reason, reason_len, "invalid first ESC node ID");
        return false;
    }
    if ((_active_esc_mask & 0xFF) == 0 || (_active_esc_mask & ~0xFF) != 0) {
        snprintf(reason, reason_len, "invalid ESC channel mask");
        return false;
    }
    if (_esc_rate_hz < 50 || _esc_rate_hz > 400) {
        snprintf(reason, reason_len, "invalid ESC command rate");
        return false;
    }
    if (_pole_pairs < 0 || _pole_pairs > 64) {
        snprintf(reason, reason_len, "invalid motor pole pairs");
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();
    WITH_SEMAPHORE(_state_sem);
    if (_active_servo_mask < 0 || _active_servo_mask > 15 || _servo_calibrated < 0 || _servo_calibrated > 1 ||
        _servo_rate_hz < 10 || _servo_rate_hz > 100) {
        snprintf(reason, reason_len, "invalid CAN servo parameters");
        return false;
    }
    for (uint8_t i = 0; i < 4; i++) {
        if (_servo_zero[i] != _active_servo_zero[i]) {
            snprintf(reason, reason_len, "reboot after servo calibration change");
            return false;
        }
        if ((_active_servo_mask & (1U << i)) == 0) {
            continue;
        }
        const auto *channel = SRV_Channels::srv_channel(i);
        if (channel != nullptr && channel->get_function() == SRV_Channel::k_none) {
            continue;
        }
        int16_t endpoint;
        if (!_active_servo_calibrated) {
            snprintf(reason, reason_len, "CAN servo calibration not confirmed");
            return false;
        }
        if (channel == nullptr || channel->get_function() != SRV_Channel::k_motor_tilt ||
            channel->get_output_min() >= channel->get_output_max() ||
            !AP_CyphalCAN_ServoProtocol::pwm_to_position(channel->get_output_min(), _active_servo_zero[i],
                                                        _active_servo_span, endpoint) ||
            !AP_CyphalCAN_ServoProtocol::pwm_to_position(channel->get_output_max(), _active_servo_zero[i],
                                                        _active_servo_span, endpoint)) {
            snprintf(reason, reason_len, "CAN servo %u function or travel invalid", i + 1);
            return false;
        }
        const auto &servo = _servos[i];
        if (servo.position_ms == 0 || now_ms - servo.position_ms >= 1000U ||
            servo.status_ms == 0 || now_ms - servo.status_ms >= 2500U) {
            snprintf(reason, reason_len, "CAN servo %u feedback missing", i + 1);
            return false;
        }
        if (servo.low_voltage || servo.abort_code != 0 || _servo_tx_failures != 0) {
            snprintf(reason, reason_len, "CAN servo %u fault; inspect CSRV log", i + 1);
            return false;
        }
        if (_last_push_us == 0 || AP_HAL::micros() - _last_push_us >= OUTPUT_TIMEOUT_US ||
            (_servo_valid_mask & (1U << i)) == 0) {
            snprintf(reason, reason_len, "CAN servo %u target invalid", i + 1);
            return false;
        }
    }
    for (uint8_t i = 0; i < NUM_ESCS; i++) {
        if ((_active_esc_mask & (1U << i)) == 0) {
            continue;
        }
        const ESCState &esc = _escs[i];
        if (!SRV_Channels::function_assigned(SRV_Channels::get_motor_function(i))) {
            snprintf(reason, reason_len, "motor %u function missing", i + 1);
            return false;
        }
        if (esc.status_ms == 0 || (now_ms - esc.status_ms) > STATUS_TIMEOUT_MS ||
            esc.feedback_ms == 0 || (now_ms - esc.feedback_ms) > FEEDBACK_TIMEOUT_MS ||
            esc.heartbeat_ms == 0 || (now_ms - esc.heartbeat_ms) > HEARTBEAT_TIMEOUT_MS) {
            snprintf(reason, reason_len, "ESC %u telemetry missing", i + 1);
            return false;
        }
        constexpr uint16_t FAULT_MASK = 0xDFF7;
        if ((esc.status & FAULT_MASK) != 0 || (esc.status & (1U << 3)) == 0 ||
            esc.output_throttle != 0 || esc.health != 0 || esc.mode != 0) {
            snprintf(reason, reason_len, "ESC %u not ready", i + 1);
            return false;
        }
    }
    return true;
}

#endif // AP_CYPHALCAN_ENABLED
