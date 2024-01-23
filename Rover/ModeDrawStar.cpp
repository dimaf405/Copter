#include "Rover.h"


// void ModeDrawStar::init()
// {
//     path_num = 0;  // 航点号清零，从而切到其他模式再切回来后，可以飞出一个新的五角星航线
//     generate_path();  // 生成五角星航线

//     pos_control_start();  // 开始位置控制

// }

// // 生成五角星航线
// void ModeDrawStar::generate_path()
// {
//     float radius_cm = g2.star_radius_cm;

//     wp_nav->get_wp_stopping_point(path[0]);

//     path[1] = path[0] + Vector3f(1.0f, 0, 0) * radius_cm;
//     path[2] = path[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
//     path[3] = path[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
//     path[4] = path[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
//     path[5] = path[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
//     path[6] = path[1];
// }

// // 开始位置控制
// void ModeDrawStar::pos_control_start()
// {
//     // initialise waypoint and spline controller
//     wp_nav->wp_and_spline_init();

//     // no need to check return status because terrain data is not used
//     wp_nav->set_wp_destination(path[0], false);

//     // initialise yaw
//     auto_yaw.set_mode_to_default(false);
// }

void ModeDrawStar::update()
{
    // process pilot's yaw input

        //以下为自定义代码
    gcs().send_text(MAV_SEVERITY_ALERT,
                    "curccent_test:%0.1fm",
                    3.14f);
    // float target_yaw_rate = 0;
    // if (!copter.failsafe.radio) {
    //     // get pilot's desired yaw rate
    //     target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    //     if (!is_zero(target_yaw_rate)) {
    //         auto_yaw.set_mode(AUTO_YAW_HOLD);
    //     }
    // }

    // // if not armed set throttle to zero and exit immediately
    // if (is_disarmed_or_landed()) {
    //     make_safe_spool_down();
    //     return;
    // }

    // // set motors to full range
    // motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // // run waypoint controller
    // copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // // call z-axis position controller (wpnav should have already updated it's alt target)
    // pos_control->update_z_controller();

    // // call attitude controller
    // if (auto_yaw.mode() == AUTO_YAW_HOLD) {
    //     // roll & pitch from waypoint controller, yaw rate from pilot
    //     attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    // } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
    //     // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
    //     attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
    // } else {
    //     // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
    //     attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    // }
}

// bool ModeAcro::requires_velocity() const
// {
//     return !g2.motors.have_skid_steering();
// }

// // sailboats in acro mode support user manually initiating tacking from transmitter
// void ModeAcro::handle_tack_request()
// {
//     rover.g2.sailboat.handle_tack_request_acro();
// }
