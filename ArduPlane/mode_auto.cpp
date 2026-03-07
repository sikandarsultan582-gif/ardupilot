void ModeAuto::update()
{
    // ۱. بررسی وضعیت کلی میشن
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        return;
    }

    // --- بخش حمله برداری (Vector Attack) سازگار با تمام بردها ---
    const uint16_t last_index = plane.mission.num_commands() - 1;
    const uint16_t current_index = plane.mission.get_current_nav_index();

    // فقط وقتی در ویپوینت آخر هستیم عمل کن
    if (current_index == last_index && last_index > 0) {
        Location target_loc, prev_loc;
        if (plane.mission.get_item_with_index(last_index, target_loc) && 
            plane.mission.get_item_with_index(last_index - 1, prev_loc)) {
            
            // محاسبه فاصله و اختلاف ارتفاع با متدهای استاندارد ArduPilot
            const float distance = prev_loc.get_distance(target_loc);
            const float alt_diff_m = (target_loc.alt - prev_loc.alt) * 0.01f;
            
            // محاسبه زاویه شیب
            const float target_pitch_deg = RAD_TO_DEG * atan2f(alt_diff_m, MAX(distance, 1.0f));

            // مقداردهی به کنترلرهای ناوبری (بدون تغییر مود)
            plane.nav_roll_cd = 0;                          // حفظ تعادل بال‌ها (Roll Stability)
            plane.nav_pitch_cd = target_pitch_deg * 100.0f; // اعمال زاویه حمله نهایی
            
            // تنظیم تراتل به صورت مستقیم و ایمن
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);
            
            // غیرفعال کردن موقت سیستم استال برای جلوگیری از ترمز هواپیما
            plane.aparm.stall_prevention.set(0);

            return; // توقف اجرای منطق عادی ناوبری برای ویپوینت آخر
        }
    }
    // --- پایان بخش حمله ---

    // روال پیش‌فرض ArduPilot برای سایر وضعیت‌ها
    const uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

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
