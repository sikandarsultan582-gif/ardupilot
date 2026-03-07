void ModeAuto::update()
{
    // 1. بررسی وضعیت میشن
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        return;
    }

    // --- بخش حمله برداری (Vector Attack) ---
    // این بخش به گونه‌ای نوشته شده که با تمام بوردها (F4, F7, H7) سازگار باشد
    const uint16_t last_idx = plane.mission.num_commands() - 1;
    const uint16_t curr_idx = plane.mission.get_current_nav_index();

    if (curr_idx == last_idx && last_idx > 0) {
        Location target_loc, prev_loc;
        if (plane.mission.get_item_with_index(last_idx, target_loc) && 
            plane.mission.get_item_with_index(last_idx - 1, prev_loc)) {
            
            float dist = prev_loc.get_distance(target_loc);
            float alt_diff = (target_loc.alt - prev_loc.alt) * 0.01f;
            float target_pitch = RAD_TO_DEG * atan2f(alt_diff, (dist > 1.0f ? dist : 1.0f));

            plane.nav_roll_cd = 0; // پایداری کامل بال‌ها
            plane.nav_pitch_cd = target_pitch * 100.0f; 
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);
            
            // جلوگیری از دخالت سیستم استال بدون نیاز به تغییر پارامتر
            plane.auto_state.takeoff_complete = true;
            return; 
        }
    }
    // --- پایان بخش حمله ---

    // شروع بخش استاندارد آردوپایلوت
    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

    // استفاده از بررسی مستقیم برای جلوگیری از خطای HAL_QUADPLANE
    if (plane.quadplane.available() && plane.quadplane.in_vtol_auto()) {
        plane.quadplane.control_auto();
        return;
    }

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
