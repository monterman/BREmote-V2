// RX-specific config field table and cross-validation.
// Shared engine is in ../Common/ConfigServiceEngine.h (included via BREmote_V2_Rx.h).
// V2.5-Evo - 2026-04-22 - Added gps_chip_type field (GPS module selector: 0=BN-220, 1=BN-880+compass, 2=M10, 3=M10+compass)
// V2.5-Evo - 2026-04-22 - Added Phase A GPS anti-spoofing fields: gps_max_hdop, gps_max_accel_g, gps_max_teleport_kmh, gps_suspect_threshold
// V2.5-Evo - 2026-04-24 - Added Phase B GPS handshake fields: gps_max_pair_dist_m, gps_max_speed_diff_kmh
// V2.5-Evo - 2026-04-25 - P7: Added RTM Phase C + RX safety fields: rtm_vesc_speed_diff_kmh, vesc_erpm_per_kmh, rtm_rx_enabled, rtm_rx_override_steering, rtm_compass_required
// V2.5-Evo - 2026-04-30 - RTM approach decel zone: rtm_approach_zone_m field added (0=disabled, 5-100 m)
// V2.5-Evo - 2026-04-30 - Rename: gps_max_jump_kmh → gps_max_teleport_kmh (clarity)
// V2.5-Evo - 2026-04-30 - Bundle E: gps_update_hz SPIFFS param added; gps_max_teleport_kmh default 200→80
// V2.5-Evo - 2026-04-29 - Bundle A: radio_preset max clamped to 2; dead foil_speed != 99 sentinel removed
// V2.5-Evo - 2026-05-08 - Bundle 1: dummy_delete_me → rtm_steer_response (0-4 preset index)
// V2.5-Evo - 2026-05-06 - D4: Added rtm_use_compass + rtm_cog_min_speed_kmh fields to ConfigService table

#include <stddef.h>

const CfgFieldSpec kCfgFields[] = {
  {"version", CFG_U16, offsetof(confStruct, version), true, false, true, (float)SW_VERSION, (float)SW_VERSION, 0, true},
  {"radio_preset", CFG_U16, offsetof(confStruct, radio_preset), true, true, true, 1.0f, 2.0f, 0, false},
  {"rf_power", CFG_I16, offsetof(confStruct, rf_power), true, true, true, -9.0f, 22.0f, 0, false},
  {"steering_type", CFG_U16, offsetof(confStruct, steering_type), true, false, true, 0.0f, 2.0f, 0, false},
  {"steering_influence", CFG_U16, offsetof(confStruct, steering_influence), true, false, true, 0.0f, 100.0f, 0, false},
  {"steering_inverted", CFG_U16, offsetof(confStruct, steering_inverted), true, false, true, 0.0f, 1.0f, 0, false},
  {"trim", CFG_I16, offsetof(confStruct, trim), true, false, true, -500.0f, 500.0f, 0, false},
  {"pwm0_min", CFG_U16, offsetof(confStruct, PWM0_min), true, false, true, 500.0f, 2500.0f, 0, false},
  {"pwm0_max", CFG_U16, offsetof(confStruct, PWM0_max), true, false, true, 500.0f, 2500.0f, 0, false},
  {"pwm1_min", CFG_U16, offsetof(confStruct, PWM1_min), true, false, true, 500.0f, 2500.0f, 0, false},
  {"pwm1_max", CFG_U16, offsetof(confStruct, PWM1_max), true, false, true, 500.0f, 2500.0f, 0, false},
  {"failsafe_time", CFG_U16, offsetof(confStruct, failsafe_time), true, false, true, 100.0f, 10000.0f, 0, false},
  {"foil_num_cells", CFG_U16, offsetof(confStruct, foil_num_cells), true, false, true, 1.0f, 50.0f, 0, false},
  {"bms_det_active", CFG_U16, offsetof(confStruct, bms_det_active), true, false, true, 0.0f, 1.0f, 0, false},
  {"wet_det_active", CFG_U16, offsetof(confStruct, wet_det_active), true, false, true, 0.0f, 1.0f, 0, false},
  // V2.5-Evo - 2026-05-08 - Bundle 1: rtm_steer_response replaces dummy_delete_me in-place (same offset, same type)
  // 0=Very Soft, 1=Soft, 2=Normal (default), 3=Sharp, 4=Very Sharp. Controls P+D+filter preset in RTMState.ino.
  {"rtm_steer_response", CFG_U16, offsetof(confStruct, rtm_steer_response), true, false, true, 0.0f, 4.0f, 0, false},
  {"data_src", CFG_U16, offsetof(confStruct, data_src), true, false, true, 0.0f, 2.0f, 0, false},
  {"gps_en", CFG_U16, offsetof(confStruct, gps_en), true, false, true, 0.0f, 1.0f, 0, false},
  {"followme_mode", CFG_U16, offsetof(confStruct, followme_mode), true, false, true, 0.0f, 3.0f, 0, false},
  {"kalman_en", CFG_U16, offsetof(confStruct, kalman_en), true, false, true, 0.0f, 1.0f, 0, false},
  {"boogie_vmax_in_followme_kmh", CFG_FLOAT, offsetof(confStruct, boogie_vmax_in_followme_kmh), true, false, true, 0.0f, 100.0f, 1, false},
  {"min_dist_m", CFG_FLOAT, offsetof(confStruct, min_dist_m), true, false, true, 0.0f, 1000.0f, 1, false},
  {"followme_smoothing_band_m", CFG_FLOAT, offsetof(confStruct, followme_smoothing_band_m), true, false, true, 0.0f, 1000.0f, 1, false},
  {"foiler_low_speed_kmh", CFG_FLOAT, offsetof(confStruct, foiler_low_speed_kmh), true, false, true, 0.0f, 100.0f, 1, false},
  {"zone_angle_enter_deg", CFG_FLOAT, offsetof(confStruct, zone_angle_enter_deg), true, false, true, 0.0f, 180.0f, 1, false},
  {"zone_angle_exit_deg", CFG_FLOAT, offsetof(confStruct, zone_angle_exit_deg), true, false, true, 0.0f, 180.0f, 1, false},
  {"near_diag_offset_deg", CFG_FLOAT, offsetof(confStruct, near_diag_offset_deg), true, false, true, 0.0f, 180.0f, 1, false},
  {"ubat_cal", CFG_FLOAT, offsetof(confStruct, ubat_cal), true, false, true, 0.000001f, 1.0f, 9, false},
  {"ubat_offset", CFG_FLOAT, offsetof(confStruct, ubat_offset), true, false, true, -100.0f, 100.0f, 4, false},
  {"tx_gps_stale_timeout_ms", CFG_U16, offsetof(confStruct, tx_gps_stale_timeout_ms), true, false, true, 0.0f, 65535.0f, 0, false},
  // V2.5-Evo - 2026-04-22 - GPS chip type: 0=BN-220, 1=BN-880+compass (RX default), 2=M10, 3=M10+compass
  {"gps_chip_type", CFG_U16, offsetof(confStruct, gps_chip_type), true, false, true, 0.0f, 3.0f, 0, false},
  // V2.5-Evo - 2026-04-22 - Phase A GPS anti-spoofing parameters (see CLAUDE.md Section 11)
  {"gps_max_hdop",           CFG_FLOAT, offsetof(confStruct, gps_max_hdop),           true, false, true,  0.5f,  5.0f, 1, false},
  {"gps_max_accel_g",        CFG_FLOAT, offsetof(confStruct, gps_max_accel_g),        true, false, true,  1.0f, 10.0f, 1, false},
  {"gps_max_teleport_kmh",       CFG_FLOAT, offsetof(confStruct, gps_max_teleport_kmh),       true, false, true, 50.0f,500.0f, 1, false},
  {"gps_suspect_threshold",  CFG_U16,   offsetof(confStruct, gps_suspect_threshold),  true, false, true,  1.0f, 10.0f, 0, false},
  // V2.5-Evo - 2026-04-24 - Phase B GPS handshake anti-spoofing parameters (see CLAUDE.md Section 11)
  {"gps_max_pair_dist_m",    CFG_FLOAT, offsetof(confStruct, gps_max_pair_dist_m),    true, false, true, 50.0f, 2000.0f, 1, false},
  {"gps_max_speed_diff_kmh", CFG_FLOAT, offsetof(confStruct, gps_max_speed_diff_kmh), true, false, true, 10.0f,  200.0f, 1, false},
  // V2.5-Evo - 2026-04-25 - Priority 7 RTM Phase C + RX safety parameters
  {"rtm_vesc_speed_diff_kmh",  CFG_FLOAT, offsetof(confStruct, rtm_vesc_speed_diff_kmh),  true, false, true,  5.0f, 50.0f,   1, false},
  {"vesc_erpm_per_kmh",        CFG_FLOAT, offsetof(confStruct, vesc_erpm_per_kmh),        true, false, true,  0.0f, 9999.0f, 1, false},
  {"rtm_rx_enabled",           CFG_U16,   offsetof(confStruct, rtm_rx_enabled),           true, false, true,  0.0f,  1.0f,   0, false},
  {"rtm_rx_override_steering", CFG_U16,   offsetof(confStruct, rtm_rx_override_steering), true, false, true,  0.0f,  1.0f,   0, false},
  {"rtm_compass_required",     CFG_U16,   offsetof(confStruct, rtm_compass_required),     true, false, true,  0.0f,  1.0f,   0, false},
  {"rtm_stop_distance_m",      CFG_U16,   offsetof(confStruct, rtm_stop_distance_m),      true, false, true,  1.0f, 50.0f,   0, false},
  // V2.5-Evo - 2026-04-29 - Bundle B: configurable VESC UART timeout (replaces hardcoded 20s)
  {"vesc_timeout_s",           CFG_U16,   offsetof(confStruct, vesc_timeout_s),           true, false, true,  5.0f, 60.0f,   0, false},
  // V2.5-Evo - 2026-04-30 - Bundle E: configurable GPS polling rate (replaces hardcoded 1Hz cadence)
  {"gps_update_hz",            CFG_U16,   offsetof(confStruct, gps_update_hz),            true, false, true,  1.0f, 10.0f,   0, false},
  // V2.5-Evo - 2026-04-30 - RTM approach decel zone (0 = disabled; outer edge where throttle ramp begins)
  {"rtm_approach_zone_m",      CFG_U16,   offsetof(confStruct, rtm_approach_zone_m),      true, false, true,  0.0f, 100.0f,  0, false},
  // V2.5-Evo - 2026-05-06 - D4: RTM heading source selection (rtm_use_compass + rtm_cog_min_speed_kmh)
  // rtm_use_compass: 0=GPS COG only, 1=Hybrid (default), 2=Compass only DIAGNOSTIC ONLY DO NOT USE ON WATER
  // rtm_cog_min_speed_kmh: GPS speed threshold below which compass snapshot is used; range 1-15 km/h, default 3
  {"rtm_use_compass",          CFG_U16,   offsetof(confStruct, rtm_use_compass),          true, false, true,  0.0f,   2.0f,  0, false},
  {"rtm_cog_min_speed_kmh",    CFG_U16,   offsetof(confStruct, rtm_cog_min_speed_kmh),    true, false, true,  1.0f,  15.0f,  0, false},
  {"logger_en", CFG_U16, offsetof(confStruct, logger_en), true, false, true, 0.0f, 1.0f, 0, false},
  {"paired", CFG_U16, offsetof(confStruct, paired), true, false, true, 0.0f, 1.0f, 0, false},
  {"own_address", CFG_ADDR3, offsetof(confStruct, own_address), true, false, false, 0.0f, 0.0f, 0, false},
  {"dest_address", CFG_ADDR3, offsetof(confStruct, dest_address), true, false, false, 0.0f, 0.0f, 0, false},
  {"wifi_password", CFG_STR8, offsetof(confStruct, wifi_password), true, false, false, 0.0f, 0.0f, 8, false}
};

const size_t kCfgFieldCount = sizeof(kCfgFields) / sizeof(kCfgFields[0]);

bool cfgValidateCrossField(confStruct &candidate, String &err)
{
  if (candidate.PWM0_max <= candidate.PWM0_min)
  {
    err = "ERR_CROSS:PWM0_max must be > PWM0_min";
    return false;
  }
  if (candidate.PWM1_max <= candidate.PWM1_min)
  {
    err = "ERR_CROSS:PWM1_max must be > PWM1_min";
    return false;
  }
  if (candidate.failsafe_time < 100 || candidate.failsafe_time > 10000)
  {
    err = "ERR_CROSS:failsafe_time out of range (100-10000)";
    return false;
  }
  return true;
}
