void ModeAuto::update()
{
    // 1. چک کردن وضعیت میشن (Mission)
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        return;
    }

    // --- شروع بخش اختصاصی: حمله برداری نهایی (TERMINAL VECTOR ATTACK) ---
    uint16_t last_idx = plane.mission.num_commands() - 1;
    uint16_t curr_idx = plane.mission.get_current_nav_index();

    // اگر در ویپوینت آخر هستیم، محاسبات حمله را جایگزین ناوبری عادی کن
    if (curr_idx == last_idx && last_idx > 0) {
        Location target_loc, prev_loc;
        if (plane.mission.get_item_with_index(last_idx, target_loc) && 
            plane.mission.get_item_with_index(last_idx - 1, prev_loc)) {
            
            // محاسبه زاویه بین دو نقطه (Pitch Calculation)
            float dist = prev_loc.get_distance(target_loc);
            float alt_diff = (target_loc.alt - prev_loc.alt) * 0.01f;
            float target_pitch_deg = RAD_TO_DEG * atan2f(alt_diff, dist);

            // اعمال فرامین مستقیم به کنترلر ناوبری
            plane.nav_roll_cd = 0;                          // بال‌ها کاملاً صاف (Stability)
            plane.nav_pitch_cd = target_pitch_deg * 100.0f; // زاویه محاسبه شده
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100); // قدرت حداکثر
            
            // غیرفعال سازی سیستم جلوگیری از استال برای عبور از محدودیت‌ها
            plane.aparm.stall_prevention.set(0);

            // اطلاع‌رسانی به ایستگاه زمینی
            static uint32_t last_msg_ms = 0;
            if (AP_HAL::millis() - last_msg_ms > 2000) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "V22: VECTOR LOCKED. DIVE ANGLE: %.1f", target_pitch_deg);
                last_msg_ms = AP_HAL::millis();
            }
            return; // خروج از تابع تا کدهای پایین‌تر زاویه ما را تغییر ندهند
        }
    }
    // --- پایان بخش اختصاصی ---

    // روال عادی ArduPilot برای سایر ویپوینت‌ها
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
