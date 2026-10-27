#pragma once

#include "AP_CyphalCAN_config.h"

#if AP_CYPHALCAN_ENABLED

#include <AP_CANManager/AP_CANDriver.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <AP_Param/AP_Param.h>

class AP_CyphalCAN : public AP_CANDriver, public AP_ESC_Telem_Backend {
public:
    explicit AP_CyphalCAN(uint8_t driver_index);
    CLASS_NO_COPY(AP_CyphalCAN);

    static const AP_Param::GroupInfo var_info[];
    static AP_CyphalCAN *get_cyphalcan(uint8_t driver_index);

    void init(uint8_t driver_index) override;
    bool add_interface(AP_HAL::CANIface *can_iface) override;

    void SRV_push();
    bool prearm_check(char *reason, uint8_t reason_len) const;

private:
    static constexpr uint8_t NUM_ESCS = 8;
    static constexpr uint32_t OUTPUT_TIMEOUT_US = 200000;
    static constexpr uint32_t STATUS_TIMEOUT_MS = 250;
    static constexpr uint32_t FEEDBACK_TIMEOUT_MS = 500;
    static constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 3000;

    struct ESCState {
        uint32_t status_ms;
        uint32_t feedback_ms;
        uint32_t heartbeat_ms;
        uint16_t status;
        uint16_t output_throttle;
        uint8_t health;
        uint8_t mode;
    };

    struct ServoState {
        uint32_t position_ms;
        uint32_t status_ms;
        uint32_t abort_code;
        int16_t position_ddeg;
        uint16_t current_cA;
        int8_t temperature_C;
        bool low_voltage;
    };

    void loop();
    void send_throttle();
    void send_servos();
    void poll_servos();
    void log_servos();
    void handle_frame(const AP_HAL::CANFrame &frame);
    bool send_frame(const AP_HAL::CANFrame &frame);
    bool read_frame(AP_HAL::CANFrame &frame, uint32_t timeout_us);

    const uint8_t _driver_index;
    AP_HAL::CANIface *_can_iface = nullptr;
    HAL_BinarySemaphore _can_event;
    mutable HAL_Semaphore _state_sem;
    ESCState _escs[NUM_ESCS] {};
    uint16_t _throttle[NUM_ESCS] {};
    uint32_t _last_push_us = 0;
    uint8_t _transfer_id[2] {};
    char _thread_name[16] {};
    bool _initialized = false;
    int16_t _active_source_node_id = 0;
    int16_t _active_esc_base_id = 0;
    int16_t _active_esc_mask = 0;
    int8_t _active_servo_mask = 0;
    bool _active_servo_calibrated = false;
    int16_t _active_servo_zero[4] {};
    int16_t _active_servo_span = 1000;
    ServoState _servos[4] {};
    int16_t _servo_target[4] {};
    uint8_t _servo_valid_mask = 0;
    uint8_t _servo_poll_index = 0;
    uint8_t _servo_poll_count = 0;
    uint32_t _servo_tx_failures = 0;

    AP_Int16 _source_node_id;
    AP_Int16 _esc_base_id;
    AP_Int16 _esc_rate_hz;
    AP_Int16 _pole_pairs;
    AP_Int16 _esc_mask;
    AP_Int8 _servo_mask;
    AP_Int8 _servo_calibrated;
    AP_Int16 _servo_rate_hz;
    AP_Int16 _servo_zero[4];
    AP_Int16 _servo_span;
};

#endif // AP_CYPHALCAN_ENABLED
