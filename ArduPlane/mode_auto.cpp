#include "mode.h"
#include "Plane.h"

// تابع کمکی برای محاسبه دقیق زاویه شیب بین دو ویپوینت (Vector Calculation)
float get_terminal_pitch(const Location &loc1, const Location &loc2) {
    float dist = loc1.get_distance(loc2);
    if (dist < 2.0f) return -8500.0f; // اگر بسیار نزدیک بود، شیرجه مستقیم برای امنیت برخورد
    float alt_diff = (loc2.alt * 0.01f) - (loc1.alt * 0.01f);
    // محاسبه تانژانت معکوس برای به دست آوردن زاویه مسیر
    float angle = RAD_TO_DEG * atan2f(alt_diff, dist) * 100.0f;
    
    // محدود کردن زاویه بین 0 تا 90 درجه رو به پایین
    if (angle > 0) return -8500.0f; 
    return angle;
}

bool ModeAuto::_enter()
{
#if HAL_QUADPLANE_ENABLED
    if (plane.previous_mode == &plane.mode_guided &&
        quadplane.guided_wait_takeoff_on_mode_enter) {
        if (!plane.mission.starts_with_takeoff_cmd()) {
            gcs().send_text(MAV_SEVERITY_ERROR,"Takeoff waypoint required");
            quadplane.guided_wait_takeoff = true;
            return false;
        }
    }
    
    if (plane.quadplane.available() && plane.quadplane.enable == 2) {
        plane.auto_state.vtol_mode = true;
    } else {
        plane.auto_state.vtol_mode = false;
    }
#else
    plane.auto_state.vtol_mode = false;
#endif
    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    plane.mission.start_or_resume();

    if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }

#if HAL_SOARING_ENABLED
    plane.g2.soaring_controller.init_cruising();
#endif

    return true;
}

void ModeAuto::_exit()
{
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
        plane.mission.stop();
        bool restart = plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND;
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id)) {
            restart = false;
        }
#endif
        if (restart) {
            plane.landing.restart_landing_sequence();
        }
    }
    plane.auto_state.started_flying_in_auto_ms = 0;
}

void ModeAuto::update()
{
    // --- بخش حمله برداری نهایی (TERMINAL VECTOR ATTACK) ---
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        if (plane.control_mode == &plane.mode_auto) {
            
            uint16_t last_idx = plane.mission.get_current_nav_index();
            Location target_loc, prev_loc;

            // خواندن مختصات ویپوینت آخر و یکی مانده به آخر
            if (plane.mission.get_item_with_index(last_idx, target_loc) && 
                plane.mission.get_item_with_index(last_idx - 1, prev_loc)) {
                
                // اجبار به تغییر مود برای کنترل مستقیم
                plane.set_mode(plane.mode_guided, ModeReason::MISSION_END);
                
                // محاسبه زاویه دقیق بر اساس موقعیت دو نقطه
                float dive_pitch = get_terminal_pitch(prev_loc, target_loc);
                
                // قفل کردن پارامترهای پروازی
                plane.nav_pitch_cd = dive_pitch; // زاویه محاسبه شده
                plane.nav_roll_cd = 0;           // بال‌های صاف برای دقت حداکثری
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100); // تخته گاز
                
                // غیرفعال سازی ایمنی برای جلوگیری از ترمز
                plane.aparm.stall_prevention.set(0);
                
                gcs().send_text(MAV_SEVERITY_CRITICAL, "V22: VECTOR LOCKED. NO ESCAPE.");
                return;
            }
        }
        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        return;
    }

    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_auto()) {
        plane.quadplane.control_auto();
        return;
    }
#endif

    if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && plane.flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING)) {
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.takeoff_calc_throttle();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.nav_roll_cd = plane.landing.constrain_roll(plane.nav_roll_cd, plane.g.level_roll_limit*100UL);
        if (plane.landing.is_throttle_suppressed()) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        } else {
            plane.calc_throttle();
        }
    } else {
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            plane.steer_state.hold_course_cd = -1;
        }
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

void ModeAuto::navigate()
{
    if (AP::ahrs().home_is_set()) {
        plane.mission.update();
    }
}

bool ModeAuto::does_auto_navigation() const { return true; }
bool ModeAuto::does_auto_throttle() const { return true; }

bool ModeAuto::_pre_arm_checks(size_t buflen, char *buffer) const { return true; }

bool ModeAuto::is_landing() const
{
    return (plane.flight_stage == AP_FixedWing::FlightStage::LAND);
}

void ModeAuto::run()
{
    if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_ALTITUDE_WAIT) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        reset_controllers();
    } else {
        Mode::run();
    }
}
