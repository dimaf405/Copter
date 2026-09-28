#include "AP_CyphalCAN.h"

#if AP_CYPHALCAN_ENABLED

#include "AP_CyphalCAN_Protocol.h"

#include <AP_CANManager/AP_CANManager.h>
#include <AP_Math/AP_Math.h>
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
    // @DisplayName: First CyphalCAN ESC node ID
    // @Description: Node ID of motor 1. Motors 2 to 8 must use the next seven consecutive IDs.
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
    // @Description: Bitmask of motor outputs sent over CyphalCAN. Set only after checking ESC node IDs and motor order.
    // @Bitmask: 0:Motor 1,1:Motor 2,2:Motor 3,3:Motor 4,4:Motor 5,5:Motor 6,6:Motor 7,7:Motor 8
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("ESC_BM", 5, AP_CyphalCAN, _esc_mask, 0),

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

    WITH_SEMAPHORE(_state_sem);
    memcpy(_throttle, throttle, sizeof(_throttle));
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

void AP_CyphalCAN::handle_frame(const AP_HAL::CANFrame &frame)
{
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
    while (true) {
        const uint64_t now_us = AP_HAL::micros64();
        if (now_us >= next_tx_us) {
            send_throttle();
            const uint16_t rate_hz = constrain_int16(_esc_rate_hz, 50, 400);
            next_tx_us = now_us + 1000000U / rate_hz;
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
        _esc_base_id != _active_esc_base_id || _esc_mask != _active_esc_mask) {
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
