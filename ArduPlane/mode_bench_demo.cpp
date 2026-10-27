#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

namespace {
constexpr uint32_t DEMO_PERIOD_MS = 16000;
constexpr uint32_t ALL_MOTORS_MASK = 0xFFU;
constexpr float DEMO_MOTOR_THROTTLE = 0.05f;
constexpr int16_t DEMO_SURFACE_CENTER_US = 1500;
constexpr int16_t DEMO_AILERON_TRAVEL_US = 200;
constexpr int16_t DEMO_VTAIL_TRAVEL_US = 100;
constexpr uint16_t TILT_VERTICAL_US[] = {2050, 2050, 980, 1050};
constexpr uint16_t VERTICAL_TOLERANCE_US = 25;
constexpr uint16_t MIN_TILT_TRAVEL_US = 100;

SRV_Channel::Function demo_servo_function(uint8_t index)
{
    return index < 4 ? SRV_Channel::k_motor_tilt :
           index < 8 ? SRV_Channel::k_aileron :
           index < 10 ? SRV_Channel::k_vtail_left : SRV_Channel::k_vtail_right;
}

bool get_tilt_endpoints(const SRV_Channel &channel, uint8_t index,
                        uint16_t &vertical_pwm, uint16_t &other_pwm)
{
    const uint16_t min_pwm = channel.get_output_min();
    const uint16_t max_pwm = channel.get_output_max();
    if (index >= ARRAY_SIZE(TILT_VERTICAL_US) || max_pwm < min_pwm + MIN_TILT_TRAVEL_US) {
        return false;
    }

    if (index < 2) {
        if (abs(int16_t(max_pwm) - int16_t(TILT_VERTICAL_US[index])) > VERTICAL_TOLERANCE_US) {
            return false;
        }
        // The left servos move away from vertical toward smaller pulse widths.
        vertical_pwm = max_pwm;
        other_pwm = min_pwm;
        return vertical_pwm >= other_pwm + MIN_TILT_TRAVEL_US;
    }

    if (abs(int16_t(min_pwm) - int16_t(TILT_VERTICAL_US[index])) > VERTICAL_TOLERANCE_US) {
        return false;
    }
    // The right servos move away from vertical toward larger pulse widths.
    vertical_pwm = min_pwm;
    other_pwm = max_pwm;
    return other_pwm >= vertical_pwm + MIN_TILT_TRAVEL_US;
}
}

const AP_Param::GroupInfo ModeBenchDemo::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Permit the bench demonstration mode
    // @Description: Permit mode 27 only after removing all propellers, securing the aircraft, checking the full configured tilt travel, and checking the CAN ESCs selected by CAN_D2_CY_ESC_BM. The demonstration repeats until stopped or exited. Motor outputs remain subject to normal arming, safety and emergency stop conditions. Exiting this mode disarms the aircraft. Set this back to zero after the bench test.
    // @Values: 0:Disabled,1:Enabled after bench checks
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, ModeBenchDemo, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: FULL
    // @DisplayName: Legacy bench tilt travel setting
    // @Description: Retained for parameter storage compatibility. This parameter has no effect. Bench Demo always uses the full configured MIN/MAX tilt travel, checked and captured when entering the mode.
    // @Values: 0:Unused,1:Unused
    // @User: Advanced
    AP_GROUPINFO("FULL", 2, ModeBenchDemo, full_sweep, 0),

    AP_GROUPEND
};

ModeBenchDemo::ModeBenchDemo() : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeBenchDemo::ready(char *reason, size_t reason_len, uint16_t *esc_mask_out,
                          uint16_t *vertical_pwm, uint16_t *other_pwm) const
{
    if (!quadplane.available() || quadplane.motor_test.running) {
        hal.util->snprintf(reason, reason_len, "QuadPlane unavailable or Motor Test active");
        return false;
    }
    if (enable != 1) {
        hal.util->snprintf(reason, reason_len, "set BDEMO_ENABLE after bench checks");
        return false;
    }
    if (plane.arming.arming_required() == AP_Arming::Required::NO) {
        hal.util->snprintf(reason, reason_len, "ARMING_REQUIRE must be enabled");
        return false;
    }

    enum ap_var_type param_type;
    AP_Param *const skip_checks_param = AP_Param::find("ARMING_SKIPCHK", &param_type);
    if (skip_checks_param == nullptr || param_type != AP_PARAM_INT32 ||
        static_cast<AP_Int32 *>(skip_checks_param)->get() != 0) {
        hal.util->snprintf(reason, reason_len, "ARMING_SKIPCHK must be zero");
        return false;
    }

    AP_Param *const esc_mask_param = AP_Param::find("CAN_D2_CY_ESC_BM", &param_type);
    if (esc_mask_param == nullptr || param_type != AP_PARAM_INT16) {
        hal.util->snprintf(reason, reason_len, "CAN_D2_CY_ESC_BM unavailable");
        return false;
    }
    const int16_t esc_mask = static_cast<AP_Int16 *>(esc_mask_param)->get();
    if (esc_mask <= 0 || esc_mask > int16_t(ALL_MOTORS_MASK)) {
        hal.util->snprintf(reason, reason_len, "CAN_D2_CY_ESC_BM must be 1..255");
        return false;
    }

    for (uint8_t i = 0; i < 8; i++) {
        if (!SRV_Channels::function_assigned(SRV_Channels::get_motor_function(i))) {
            hal.util->snprintf(reason, reason_len, "Motor %u function missing", i + 1);
            return false;
        }
    }

    for (uint8_t i = 0; i < 12; i++) {
        const SRV_Channel *const channel = SRV_Channels::srv_channel(i);
        if (channel == nullptr) {
            hal.util->snprintf(reason, reason_len, "PWM%u unavailable", i + 1);
            return false;
        }
        if (channel->get_function() == SRV_Channel::k_none) {
            if (i < 4 && vertical_pwm != nullptr && other_pwm != nullptr) {
                vertical_pwm[i] = 0;
                other_pwm[i] = 0;
            }
            continue;
        }
        if (channel->get_function() != demo_servo_function(i)) {
            hal.util->snprintf(reason, reason_len, "PWM%u function incorrect", i + 1);
            return false;
        }
        if (i < 4) {
            uint16_t checked_vertical_pwm;
            uint16_t checked_other_pwm;
            if (!get_tilt_endpoints(*channel, i, checked_vertical_pwm, checked_other_pwm)) {
                hal.util->snprintf(reason, reason_len, "PWM%u tilt endpoints not checked", i + 1);
                return false;
            }
            if (vertical_pwm != nullptr && other_pwm != nullptr) {
                vertical_pwm[i] = checked_vertical_pwm;
                other_pwm[i] = checked_other_pwm;
            }
        } else {
            const int16_t travel_us = i < 8 ? DEMO_AILERON_TRAVEL_US : DEMO_VTAIL_TRAVEL_US;
            if (channel->get_output_min() > DEMO_SURFACE_CENTER_US - travel_us ||
                channel->get_output_max() < DEMO_SURFACE_CENTER_US + travel_us) {
                hal.util->snprintf(reason, reason_len, "PWM%u surface limits too narrow", i + 1);
                return false;
            }
        }
    }
    if (esc_mask_out != nullptr) {
        *esc_mask_out = uint16_t(esc_mask);
    }
    return true;
}

bool ModeBenchDemo::_enter()
{
    char reason[50] {};
    if (plane.arming.is_armed() || plane.is_flying() ||
        !plane.g2.rc_channels.has_valid_input() ||
        !ready(reason, sizeof(reason), &active_esc_mask, tilt_vertical_pwm, tilt_other_pwm)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Bench Demo denied: %s",
                      reason[0] != 0 ? reason : "disarm and check RC input");
        return false;
    }

    active_servo_mask = 0;
    for (uint8_t i = 0; i < 12; i++) {
        if (SRV_Channels::srv_channel(i)->get_function() == demo_servo_function(i)) {
            active_servo_mask |= 1U << i;
        }
    }
    entered_ms = AP_HAL::millis();
    was_armed = false;
    stopped = false;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Bench Demo: propellers must be removed");
    return true;
}

void ModeBenchDemo::_exit()
{
    quadplane.motors->output_motor_mask(0.0f, ALL_MOTORS_MASK, 0.0f);
    if (plane.arming.is_armed()) {
        plane.arming.disarm(AP_Arming::Method::MOTORTEST);
    }
    output_tilts(0.0f);
    output_surfaces(0.0f);
}

bool ModeBenchDemo::_pre_arm_checks(size_t buflen, char *buffer) const
{
    if (!Mode::_pre_arm_checks(buflen, buffer)) {
        return false;
    }
    return ready(buffer, buflen);
}

void ModeBenchDemo::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0f);
    plane.nav_roll_cd = ahrs.roll_sensor;
    plane.nav_pitch_cd = ahrs.pitch_sensor;
}

void ModeBenchDemo::run()
{
    reset_controllers();
}

bool ModeBenchDemo::servo_enabled(uint8_t index)
{
    if (SRV_Channels::srv_channel(index)->get_function() != demo_servo_function(index)) {
        // Disabling a channel latches until the next mode entry.
        active_servo_mask &= ~(1U << index);
    }
    return (active_servo_mask & (1U << index)) != 0;
}

void ModeBenchDemo::output_tilts(float position)
{
    // Bench positions use the measured pulse widths, independent of SERVOx_REVERSED.
    position = constrain_float(position, 0.0f, 1.0f);
    for (uint8_t i = 0; i < 4; i++) {
        SRV_Channel *const channel = SRV_Channels::srv_channel(i);
        if (!servo_enabled(i)) {
            channel->set_output_pwm(0, true);
            continue;
        }
        const float pwm = float(tilt_vertical_pwm[i]) +
                          float(int16_t(tilt_other_pwm[i]) - int16_t(tilt_vertical_pwm[i])) * position;
        channel->set_output_pwm(uint16_t(pwm));
    }
}

void ModeBenchDemo::output_surfaces(float position)
{
    position = constrain_float(position, -1.0f, 1.0f);
    for (uint8_t i = 4; i < 12; i += 2) {
        const bool first_enabled = servo_enabled(i);
        const bool second_enabled = servo_enabled(i + 1);
        if (!first_enabled) {
            SRV_Channels::srv_channel(i)->set_output_pwm(0, true);
        }
        if (!second_enabled) {
            SRV_Channels::srv_channel(i + 1)->set_output_pwm(0, true);
        }
        if (!first_enabled && !second_enabled) {
            continue;
        }
        SRV_Channel *const channel = SRV_Channels::srv_channel(first_enabled ? i : i + 1);
        const int16_t travel_us = i < 8 ? DEMO_AILERON_TRAVEL_US : DEMO_VTAIL_TRAVEL_US;
        const int16_t direction = (i & 2U) == 0 ? 1 : -1;
        // Each pair uses its first enabled channel's reversal.
        const int16_t reverse = channel->get_reversed() ? -1 : 1;
        const int16_t offset = int16_t(travel_us * position) * direction * reverse;
        const int16_t pwm = constrain_int16(DEMO_SURFACE_CENTER_US + offset,
                                            channel->get_output_min(), channel->get_output_max());
        if (first_enabled) {
            SRV_Channels::srv_channel(i)->set_output_pwm(uint16_t(pwm));
        }
        if (second_enabled) {
            SRV_Channels::srv_channel(i + 1)->set_output_pwm(uint16_t(pwm));
        }
    }
}

void ModeBenchDemo::output()
{
    const uint32_t now = AP_HAL::millis();
    const bool armed = plane.arming.is_armed_and_safety_off();
    if (armed && !was_armed) {
        was_armed = true;
    }

    if (enable != 1 || quadplane.motor_test.running ||
        plane.failsafe.rc_failsafe || plane.failsafe.state != FAILSAFE_NONE ||
        !plane.g2.rc_channels.has_valid_input() || SRV_Channels::get_emergency_stop() ||
        plane.is_flying() || (was_armed && !armed)) {
        if (!stopped) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Bench Demo stopped: check RC, safety and enable");
        }
        stopped = true;
    }

    float tilt = 0.0f;
    float surface_position = 0.0f;
    if (!stopped) {
        const float cycle = float((now - entered_ms) % DEMO_PERIOD_MS) / float(DEMO_PERIOD_MS);
        const float phase = 2.0f * cycle;
        tilt = phase <= 1.0f ? phase : 2.0f - phase;
        surface_position = sinf(2.0f * M_PI * cycle);
    }

    output_tilts(tilt);
    output_surfaces(surface_position);
    const uint32_t inactive_mask = ALL_MOTORS_MASK & ~uint32_t(active_esc_mask);
    if (inactive_mask != 0) {
        quadplane.motors->output_motor_mask(0.0f, inactive_mask, 0.0f);
    }
    quadplane.motors->output_motor_mask(armed && !stopped ? DEMO_MOTOR_THROTTLE : 0.0f,
                                        active_esc_mask, 0.0f);
}

#endif // HAL_QUADPLANE_ENABLED
