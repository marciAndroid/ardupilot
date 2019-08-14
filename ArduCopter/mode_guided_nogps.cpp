#include "Copter.h"

/*
 * Init and run calls for guided_nogps flight mode
 */
#define GUIDED_ATTITUDE_TIMEOUT_MS  1000    // guided mode's attitude controller times out after 1 second with no new updates

struct {
    uint32_t update_time_ms;
    float roll_cd;
    float pitch_cd;
    float yaw_cd;
    float yaw_rate_cds;
    float climb_rate_cms;
    bool use_yaw_rate;
} static guided_nogps_angle_state;

// initialise guided mode's angle controller
void Copter::ModeGuidedNoGPS::angle_control_start()
{
    // set guided_mode to velocity controller
    guided_nogps_mode = Guided_NoGPS_Angle;

    // set vertical speed and acceleration
    pos_control->set_speed_z(wp_nav->get_speed_down(), wp_nav->get_speed_up());
    pos_control->set_accel_z(wp_nav->get_accel_z());

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise targets
    guided_nogps_angle_state.update_time_ms = millis();
    guided_nogps_angle_state.roll_cd = ahrs.roll_sensor;
    guided_nogps_angle_state.pitch_cd = ahrs.pitch_sensor;
    guided_nogps_angle_state.yaw_cd = ahrs.yaw_sensor;
    guided_nogps_angle_state.climb_rate_cms = 0.0f;
    guided_nogps_angle_state.yaw_rate_cds = 0.0f;
    guided_nogps_angle_state.use_yaw_rate = false;

    // pilot always controls yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

 // set guided mode angle target
void Copter::ModeGuidedNoGPS::set_target_attitude(const Quaternion &q, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_rads)
{
    // check we are in velocity control mode
    if (guided_nogps_mode != Guided_Angle) {
        angle_control_start();
    }

    // convert quaternion to euler angles
    q.to_euler(guided_nogps_angle_state.roll_cd, guided_nogps_angle_state.pitch_cd, guided_nogps_angle_state.yaw_cd);
    guided_nogps_angle_state.roll_cd = ToDeg(guided_nogps_angle_state.roll_cd) * 100.0f;
    guided_nogps_angle_state.pitch_cd = ToDeg(guided_nogps_angle_state.pitch_cd) * 100.0f;
    guided_nogps_angle_state.yaw_cd = wrap_180_cd(ToDeg(guided_nogps_angle_state.yaw_cd) * 100.0f);
    guided_nogps_angle_state.yaw_rate_cds = ToDeg(yaw_rate_rads) * 100.0f;
    guided_nogps_angle_state.use_yaw_rate = use_yaw_rate;

    guided_nogps_angle_state.climb_rate_cms = climb_rate_cms;
    guided_nogps_angle_state.update_time_ms = millis();

    // interpret positive climb rate as triggering take-off
    if (motors->armed() && !ap.auto_armed && (guided_nogps_angle_state.climb_rate_cms > 0.0f)) {
        copter.set_auto_armed(true);
    }

}

// initialise guided_nogps controller
bool Copter::ModeGuidedNoGPS::init(bool ignore_checks)
{
    if (!motors->armed()) {
        return false;
    }
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void Copter::ModeGuidedNoGPS::run()
{
    if (guided_nogps_mode == Guided_NoGPS_Angle) {
        Copter::ModeGuidedNoGPS::angle_control_run_nogps();
    }
    
}

// guided_angle_control_run - runs the guided angle controller
// called from guided_run
void Copter::ModeGuidedNoGPS::angle_control_run_nogps()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || (ap.land_complete && guided_nogps_angle_state.climb_rate_cms <= 0.0f)) {
        zero_throttle_and_relax_ac();
        pos_control->relax_alt_hold_controllers(0.0f);
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
    float yaw_rate_in = wrap_180_cd(guided_nogps_angle_state.yaw_rate_cds);

    // constrain climb rate
    float climb_rate_cms = constrain_float(guided_nogps_angle_state.climb_rate_cms, -fabsf(wp_nav->get_speed_down()), wp_nav->get_speed_up());

    // get avoidance adjusted climb rate
    climb_rate_cms = get_avoidance_adjusted_climbrate(climb_rate_cms);

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - guided_nogps_angle_state.update_time_ms > GUIDED_ATTITUDE_TIMEOUT_MS) {
        roll_in = 0.0f;
        pitch_in = 0.0f;
        climb_rate_cms = 0.0f;
        yaw_rate_in = 0.0f;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // call attitude controller
    if (guided_nogps_angle_state.use_yaw_rate) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_in, pitch_in, yaw_rate_in);
    }
    else {
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true);
    }

    // call position controller
    pos_control->set_alt_target_from_climb_rate_ff(climb_rate_cms, G_Dt, false);
    pos_control->update_z_controller();
}