#include "Copter.h"

#if MODE_GUIDED_NOGPS_ENABLED == ENABLED

/*
 * Init and run calls for guided_nogps flight mode
 */
#define GUIDED_NOGPS_ATTITUDE_TIMEOUT_MS  1500    // guided mode's attitude controller times out after 1 second with no new updates

struct {
    uint32_t update_time_ms;
    float roll_cd;  // centi-degree
    float pitch_cd; // centi-degree
    float yaw_cd;   // centi-degree
    float throttle_in;
} static guided_nogps_angle_state;

// initialise guided mode's angle controller
void ModeGuidedNoGPS::angle_control_start() {

    // set guided_mode to attitude controller
    guided_nogps_mode = Guided_NoGPS_Angle;

    // set vertical speed and acceleration
    copter.pos_control->set_max_speed_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up());
    copter.pos_control->set_max_accel_z(wp_nav->get_accel_z());

    // initialise position and desired velocity
    /* if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
     }*/

    // initialise targets
    guided_nogps_angle_state.update_time_ms = millis();
    guided_nogps_angle_state.roll_cd = ahrs.roll_sensor;
    guided_nogps_angle_state.pitch_cd = ahrs.pitch_sensor;
    guided_nogps_angle_state.yaw_cd = ahrs.yaw_sensor;
    guided_nogps_angle_state.throttle_in = 0.0f;

    // pilot always controls yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

 // set guided mode angle target
void ModeGuidedNoGPS::set_target_attitude(const Quaternion & q, float throttle_in) {
    
    // check we are in velocity control mode
/*
    if (guided_nogps_mode != Guided_NoGPS_Angle) {
        angle_control_start();
    }
*/

    // convert quaternion to euler angles
    q.to_euler(guided_nogps_angle_state.roll_cd, guided_nogps_angle_state.pitch_cd, guided_nogps_angle_state.yaw_cd);
    guided_nogps_angle_state.roll_cd = ToDeg(guided_nogps_angle_state.roll_cd) * 100.0f;
    guided_nogps_angle_state.pitch_cd = ToDeg(guided_nogps_angle_state.pitch_cd) * 100.0f;
    guided_nogps_angle_state.yaw_cd = wrap_180_cd(ToDeg(guided_nogps_angle_state.yaw_cd) * 100.0f);

    guided_nogps_angle_state.throttle_in = throttle_in;
    guided_nogps_angle_state.update_time_ms = millis();

    // interpret positive climb rate as triggering take-off
    if (motors->armed() && !copter.ap.auto_armed && (guided_nogps_angle_state.throttle_in > 0.0f)) {
        copter.set_auto_armed(true);
    }

}

// initialise guided_nogps controller
bool ModeGuidedNoGPS::init(bool ignore_checks) {

    angle_control_start();

    if (!motors->armed()) {
        return false;
    }
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void ModeGuidedNoGPS::run() {
    if (guided_nogps_mode == Guided_NoGPS_Angle) {
        ModeGuidedNoGPS::angle_control_run_nogps();
    }
    
}

// guided_angle_control_run - runs the guided angle controller
// called from guided_run
void ModeGuidedNoGPS::angle_control_run_nogps() {
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !copter.ap.auto_armed || !motors->get_interlock() || (copter.ap.land_complete && guided_nogps_angle_state.throttle_in <= 0.0f)) {
        zero_throttle_and_relax_ac();
        //pos_control->relax_alt_hold_controllers(0.0f);
        return;
    }

    // constrain desired lean angles
    float roll_in = guided_nogps_angle_state.roll_cd;
    float pitch_in = guided_nogps_angle_state.pitch_cd;

    float total_in = norm(roll_in, pitch_in);
    float angle_max = MIN(attitude_control->get_althold_lean_angle_max(), copter.aparm.angle_max);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // wrap yaw request
    float yaw_in = wrap_180_cd(guided_nogps_angle_state.yaw_cd);

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    //yaw_in = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // Throttle input 0..1
    float pilot_throttle_scaled = get_pilot_desired_throttle();


    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - guided_nogps_angle_state.update_time_ms > GUIDED_NOGPS_ATTITUDE_TIMEOUT_MS) {
        target_roll = 0.0f;
        target_pitch = 0.0f;
        yaw_in = 0.0f;
        pilot_throttle_scaled = 0.0f;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true);
    //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, yaw_in);

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);

}


#endif
