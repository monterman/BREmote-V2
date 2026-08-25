// V2.5-Evo - 2026-08-25 - Follow-Me mode validation extended 0-3 -> 0-4 for F4 In Front. Range only; no confStruct/SW_VERSION change.
// TX-specific config field table and cross-validation.
// Shared engine is in ../Common/ConfigServiceEngine.h (included via BREmote_V2_Tx.h).
// V2.5-Evo - 2026-04-27 - P8: Added rtm_display_mode, fm_warn_distance_m, rtm_steer_exit_on_input; rtm_max_runtime_s min changed 30→0
// V2.5-Evo - 2026-04-28 - P9: Added dist_unit (0=Metres, 1=Feet; range 0-1)
// V2.5-Evo - 2026-07-25 - dist_unit: Feet (1) is NOT yet rendered on the TX display this version — the
//   display always shows metres regardless of this value. Key retained (no schema change) for a future version.
// V2.5-Evo - 2026-04-28 - ChangeA: fm_arm_window_s max raised 60→120s (no struct change, no SPIFFS reset)
// V2.5-Evo - 2026-04-29 - Sleep: added sleep_timeout_s to ConfigService validation table
// V2.5-Evo - 2026-07-20 - MagGesture: added mag_mode (0=off/not fitted, 1=FM, 2=RTM, 3=FM+RTM). bt_enabled unchanged.
// V2.5-Evo - 2026-05-15 - feature/bluetooth: added bt_enabled (0=off, 1=Hall/session, 2=always on)
// V2.5-Evo - 2026-05-01 - thr_expo1 repurposed as fm_display_mode; range 1-4, default 1

const CfgFieldSpec kCfgFields[] = {
  {"radio_preset", CFG_U16, offsetof(confStruct, radio_preset), true, true, true, 1.0f, 2.0f, 0, false},
  {"rf_power", CFG_I16, offsetof(confStruct, rf_power), true, true, true, -9.0f, 22.0f, 0, false},
  {"max_gears", CFG_U16, offsetof(confStruct, max_gears), true, false, true, 1.0f, 10.0f, 0, false},
  {"startgear", CFG_U16, offsetof(confStruct, startgear), true, false, true, 0.0f, 9.0f, 0, false},
  {"no_lock", CFG_U16, offsetof(confStruct, no_lock), true, false, true, 0.0f, 1.0f, 0, false},
  {"throttle_mode", CFG_U16, offsetof(confStruct, throttle_mode), true, false, true, 0.0f, 2.0f, 0, false},
  {"steer_enabled", CFG_U16, offsetof(confStruct, steer_enabled), true, false, true, 0.0f, 1.0f, 0, false},
  {"wifi_password", CFG_STR8, offsetof(confStruct, wifi_password), true, false, false, 0.0f, 0.0f, 8, false},
  {"dynamic_power_start", CFG_U16, offsetof(confStruct, dynamic_power_start), true, false, true, 10.0f, 100.0f, 0, false},
  {"dynamic_power_step", CFG_U16, offsetof(confStruct, dynamic_power_step), true, false, true, 1.0f, 25.0f, 0, false},
  {"thr_expo", CFG_U16, offsetof(confStruct, thr_expo), true, false, true, 0.0f, 100.0f, 0, false},
  {"tog_deadzone", CFG_U16, offsetof(confStruct, tog_deadzone), true, false, true, 100.0f, 3000.0f, 0, false},
  {"tog_diff", CFG_U16, offsetof(confStruct, tog_diff), true, false, true, 1.0f, 200.0f, 0, false},
  {"tog_block_time", CFG_U16, offsetof(confStruct, tog_block_time), true, false, true, 0.0f, 5000.0f, 0, false},
  {"menu_timeout", CFG_U16, offsetof(confStruct, menu_timeout), true, false, true, 0.0f, 1000.0f, 0, false},
  {"version", CFG_U16, offsetof(confStruct, version), true, false, true, (float)SW_VERSION, (float)SW_VERSION, 0, true},
  {"cal_ok", CFG_U16, offsetof(confStruct, cal_ok), true, false, true, 0.0f, 1.0f, 0, false},
  {"cal_offset", CFG_U16, offsetof(confStruct, cal_offset), true, false, true, 0.0f, 65535.0f, 0, false},
  {"thr_idle", CFG_U16, offsetof(confStruct, thr_idle), true, false, true, 0.0f, 65535.0f, 0, false},
  {"thr_pull", CFG_U16, offsetof(confStruct, thr_pull), true, false, true, 0.0f, 65535.0f, 0, false},
  {"tog_left", CFG_U16, offsetof(confStruct, tog_left), true, false, true, 0.0f, 65535.0f, 0, false},
  {"tog_mid", CFG_U16, offsetof(confStruct, tog_mid), true, false, true, 0.0f, 65535.0f, 0, false},
  {"tog_right", CFG_U16, offsetof(confStruct, tog_right), true, false, true, 0.0f, 65535.0f, 0, false},
  {"trig_unlock_timeout", CFG_U16, offsetof(confStruct, trig_unlock_timeout), true, false, true, 0.0f, 65535.0f, 0, false},
  {"lock_waittime", CFG_U16, offsetof(confStruct, lock_waittime), true, false, true, 0.0f, 65535.0f, 0, false},
  {"gear_change_waittime", CFG_U16, offsetof(confStruct, gear_change_waittime), true, false, true, 0.0f, 65535.0f, 0, false},
  {"gear_display_time", CFG_U16, offsetof(confStruct, gear_display_time), true, false, true, 0.0f, 65535.0f, 0, false},
  {"err_delete_time", CFG_U16, offsetof(confStruct, err_delete_time), true, false, true, 0.0f, 65535.0f, 0, false},
  // FM digit zone data selector: 1=TX speed, 2=distance to buggy, 3=buggy speed, 4=throttle %
  {"fm_display_mode", CFG_U16, offsetof(confStruct, fm_display_mode), true, false, true, 1.0f, 4.0f, 1, false},
  {"steer_expo", CFG_U16, offsetof(confStruct, steer_expo), true, false, true, 0.0f, 65535.0f, 0, false},
  // V2.5-Evo - 2026-08-18 - gps_dyn_model: u-blox NAV5 dynamic platform model. Renamed in place
  // from the unused steer_expo1 slot, so sizeof(confStruct) and SW_VERSION are both unchanged.
  // 0 = default (Sea) | 4 = Automotive | 5 = Sea. Range 0-5 with 1/2/3 rejected by
  // gpsBuildNav5(), which resolves anything that is not an explicit 4 to Sea — so an
  // out-of-range or corrupt value fails toward the conservative model, never toward
  // dynModel 0 (Portable). Sea has a 500 m altitude ceiling; above that use 4.
  {"gps_dyn_model", CFG_U16, offsetof(confStruct, gps_dyn_model), true, false, true, 0.0f, 5.0f, 0, false},
  {"ubat_cal", CFG_FLOAT, offsetof(confStruct, ubat_cal), true, false, true, 0.000001f, 1.0f, 9, false},
  {"gps_en", CFG_U16, offsetof(confStruct, gps_en), true, false, true, 0.0f, 1.0f, 0, false},
  {"followme_mode", CFG_U16, offsetof(confStruct, followme_mode), true, false, true, 0.0f, 4.0f, 0, false},
  {"kalman_en", CFG_U16, offsetof(confStruct, kalman_en), true, false, true, 0.0f, 1.0f, 0, false},
  {"speed_src", CFG_U16, offsetof(confStruct, speed_src), true, false, true, 0.0f, 5.0f, 0, false},
  {"tx_gps_stale_timeout_ms", CFG_U16, offsetof(confStruct, tx_gps_stale_timeout_ms), true, false, true, 0.0f, 65535.0f, 0, false},
  // V2.5-Evo - 2026-04-22 - HDOP quality gate for TX GPS (stored as HDOP*100; 200 = HDOP 2.0; range 50-500)
  {"gps_max_hdop", CFG_U16, offsetof(confStruct, gps_max_hdop), true, false, true, 50.0f, 500.0f, 0, false},
  // V2.5-Evo - 2026-04-22 - GPS chip type selector (0=BN-220, 2=M10; types 1/3 rejected by cross-field check since TX has no compass)
  {"gps_chip_type", CFG_U16, offsetof(confStruct, gps_chip_type), true, false, true, 0.0f, 3.0f, 0, false},
  // V2.5-Evo - 2026-04-25 - Priority 7 RTM and FM mode parameters
  {"rtm_enabled",            CFG_U16, offsetof(confStruct, rtm_enabled),            true, false, true,  0.0f,   1.0f,    0, false},
  // V2.5-Evo - 2026-07-20 - min lowered 4.0f → 3.0f so the arm-hold can be set shorter than the
  // old 4s floor. Floor is 3, NOT 2: it must exceed the hardcoded 2000ms simple-hold in
  // handleGearToggle() (Hall.ino) so a combo hold can never fire at the instant the user
  // expects a lock / display-cycle. See that block comment for the full reasoning.
  {"rtm_hold_duration_s",    CFG_U16, offsetof(confStruct, rtm_hold_duration_s),    true, false, true,  3.0f,  10.0f,    0, false},
  {"rtm_arm_window_s",       CFG_U16, offsetof(confStruct, rtm_arm_window_s),       true, false, true,  5.0f,  30.0f,    0, false},
  {"rtm_double_squeeze_en",  CFG_U16, offsetof(confStruct, rtm_double_squeeze_en),  true, false, true,  0.0f,   1.0f,    0, false},
  {"rtm_throttle_start_pct", CFG_U16, offsetof(confStruct, rtm_throttle_start_pct), true, false, true, 10.0f,  50.0f,    0, false},
  {"rtm_throttle_max_pct",   CFG_U16, offsetof(confStruct, rtm_throttle_max_pct),   true, false, true, 30.0f,  90.0f,    0, false},
  {"rtm_ramp_duration_s",    CFG_U16, offsetof(confStruct, rtm_ramp_duration_s),    true, false, true,  2.0f,  15.0f,    0, false},
  {"rtm_disengage_distance_m", CFG_U16, offsetof(confStruct, rtm_disengage_distance_m), true, false, true,  3.0f,  20.0f,    0, false},
  {"rtm_max_runtime_s",      CFG_U16, offsetof(confStruct, rtm_max_runtime_s),      true, false, true,  0.0f, 300.0f,    0, false},  // P8: min 30→0 (0=disabled)
  {"rtm_gps_timeout_ms",     CFG_U16, offsetof(confStruct, rtm_gps_timeout_ms),     true, false, true, 500.0f,3000.0f,   0, false},
  // V2.5-Evo - 2026-07-20 - min lowered 4.0f → 3.0f (same rationale as rtm_hold_duration_s above).
  {"fm_hold_duration_s",     CFG_U16, offsetof(confStruct, fm_hold_duration_s),     true, false, true,  3.0f,  10.0f,    0, false},
  {"fm_override_enabled",    CFG_U16, offsetof(confStruct, fm_override_enabled),    true, false, true,  0.0f,   1.0f,    0, false},
  // V2.5-Evo - 2026-04-27 - Priority 8 UX overhaul parameters
  {"rtm_display_mode",         CFG_U16, offsetof(confStruct, rtm_display_mode),         true, false, true,  0.0f,   2.0f,    0, false},  // 0=distance, 1=speed, 2=alternating 2.5s
  {"fm_warn_distance_m",       CFG_U16, offsetof(confStruct, fm_warn_distance_m),       true, false, true, 50.0f, 1000.0f,   0, false},  // FM proximity warning threshold in meters
  {"rtm_steer_exit_on_input",  CFG_U16, offsetof(confStruct, rtm_steer_exit_on_input),  true, false, true,  0.0f,   1.0f,    0, false},  // 1=steering exits RTM, 0=blend only
  // V2.5-Evo - 2026-04-27 - Priority 8.1 FM UX redesign parameter
  {"fm_arm_window_s",          CFG_U16, offsetof(confStruct, fm_arm_window_s),          true, false, true, 10.0f, 600.0f,    0, false},  // FM auto-disarm after N seconds of no throttle input
  // V2.5-Evo - 2026-04-28 - P9: Distance unit selector. 0=Metres, 1=Feet.
  // NOTE (2026-07-25): Feet (1) is not yet rendered on the TX display this version — metres are shown
  // regardless. Value is still stored/validated so a future version can implement feet without a wipe.
  {"dist_unit",                CFG_U16, offsetof(confStruct, dist_unit),                true, false, true,  0.0f,   1.0f,    0, false},
  // V2.5-Evo - 2026-04-29 - Sleep timeout. 0=disabled, 60-3600 s; default 300 (5 minutes).
  // Controls how long TX waits with no LoRa packet from RX before deep sleeping.
  {"sleep_timeout_s", CFG_U16, offsetof(confStruct, sleep_timeout_s), true, false, true, 0.0f, 3600.0f, 0, false},
  {"bt_enabled",      CFG_U16, offsetof(confStruct, bt_enabled),      true, false, true, 0.0f,    2.0f, 0, false},
  // V2.5-Evo - 2026-07-20 - MagGesture: magnet/Hall gesture role.
  // 0=off/not fitted (default), 1=arm FM (2s), 2=arm RTM (2s), 3=FM (2s) + RTM (5s).
  {"mag_mode",        CFG_U16, offsetof(confStruct, mag_mode),        true, false, true, 0.0f,    3.0f, 0, false},
  {"paired", CFG_U16, offsetof(confStruct, paired), true, false, true, 0.0f, 1.0f, 0, false},
  {"own_address", CFG_ADDR3, offsetof(confStruct, own_address), true, false, false, 0.0f, 0.0f, 0, false},
  {"dest_address", CFG_ADDR3, offsetof(confStruct, dest_address), true, false, false, 0.0f, 0.0f, 0, false}
};

const size_t kCfgFieldCount = sizeof(kCfgFields) / sizeof(kCfgFields[0]);

bool cfgValidateCrossField(confStruct &candidate, String &err)
{
  if (candidate.max_gears < 1 || candidate.max_gears > 10)
  {
    err = "ERR_RANGE:max_gears";
    return false;
  }
  if (candidate.startgear >= candidate.max_gears)
  {
    candidate.startgear = candidate.max_gears - 1;
  }
  if (candidate.throttle_mode == 2 && candidate.dynamic_power_start < 10)
  {
    candidate.dynamic_power_start = 10;
  }
  if (candidate.dynamic_power_step < 1) candidate.dynamic_power_step = 1;
  if (candidate.dynamic_power_step > 25) candidate.dynamic_power_step = 25;

  // V2.5-Evo - 2026-04-22 - TX hardware has no compass. Types 1 (BN-880+compass) and
  // 3 (M10+compass) are RX-only. Reject them here so the user gets a clear error
  // rather than silently falling back to a wrong init path.
  if (candidate.gps_chip_type == 1 || candidate.gps_chip_type == 3)
  {
    err = "ERR_CROSS:gps_chip_type 1/3 (with compass) not valid on TX — use 0 (BN-220) or 2 (M10)";
    return false;
  }

  return true;
}
