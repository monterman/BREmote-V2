// V2.5-Evo - 2026-10-04 - L-2: cfgValidateCrossField() now CLAMPS fm_align_cap too - outside the validated 8-80 range it falls back to the shipped default 13 (never 0, which would stop the buggy, and never above 80). It was the one of SW36's three fields with no cross-field load clamp while fm_return_mode and fm_align_influence had one. Same reasoning as those two: this validator runs on the LOAD path, so a range rejection there fails the load and falls back to defaults - the config/pairing/compass-calibration wipe this function exists to prevent. Not reachable from a valid stored value; consistency and the corrupt-blob case. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-10-02 - P2 (FRONT STATIONS F4/F5), part 2 of 3: one new kCfgFields row, fm_front_angle_deg (u16, 0 = use the 45 deg default, else 35-80 deg off dead ahead), for the field that took the struct's 2 tail padding bytes - sizeof stays 200, SW_VERSION stays 36, no config is wiped. followme_mode's range goes 0-3 -> 0-5 (1 rear-right, 2 behind, 3 rear-left, 4 FRONT-RIGHT, 5 FRONT-LEFT; there is deliberately no 6 and no dead-ahead station). cfgValidateCrossField() gains the front-angle CLAMPS: (0, 35) is raised to 35 with a NOTE naming the 13 m lateral margin, and anything above 80 is lowered to 80 - clamps, never rejections, because this validator runs on the LOAD path and a range rejection there wipes the whole config (the 2026-09-03 lesson), and because the field lives in bytes an older firmware never wrote, so a stored blob may hold anything there. Both corrections move the front station FURTHER from the rider's line.
// V2.5-Evo - 2026-09-19 - DEEP LOG level 5: the log_level row's max is raised 4 -> 5 (5 = Everything, the 109 B test-session record), and cfgValidateCrossField() CLAMPS log_level > 5 down to 5 - a clamp, never a rejection, because this validator runs on the LOAD path and a range rejection there wipes the whole config (the 2026-09-03 lesson, same as fm_return_mode). Same u16 slot, no confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - SEPARATE RAMPS: the kCfgFields row "rsvd_f32_1" (float, RESERVED, -1e6..1e6) becomes "auto_ramp_s" (float, 0-4.0, 2 dp: 0 = the automatic modes ride motor_ramp_s as before, 0.2-4.0 = their own rise-limit), plus CLAMPS in cfgValidateCrossField(): (0, 0.2) -> 0.2 with a NOTE, NaN / negative -> 0, above 4.0 -> 4.0. The confStruct slot is the SAME slot renamed in place, so sizeof stays 200, SW_VERSION stays 36 and no config is wiped; every stored blob reads 0 = inherit there.
// V2.5-Evo - 2026-09-19 - STICK DURING AUTO-STEER: the kCfgFields row "rsvd_u16_1" (u16, RESERVED, 0-65535) becomes "steer_during_auto" (u16, 0-1: 0 = the stick cancels automatic steering as before, 1 = it takes over while deflected and resumes on centring), plus a CLAMP in cfgValidateCrossField() (> 1 -> 1) next to the fm_return_mode clamp. The confStruct slot is the SAME slot renamed in place, so sizeof stays 200, SW_VERSION stays 36 and no config is wiped; every stored blob reads 0 = cancel there.
// V2.5-Evo - 2026-09-19 - SW36: three kCfgFields rows for the appended Follow-Me fields - fm_return_mode (u16, 0-1), fm_align_cap (u16, 8-80), fm_align_influence (u16, 0-100) - and two CLAMPS in cfgValidateCrossField(): fm_return_mode > 1 -> 1, fm_align_influence > 100 -> 100. Clamps, never rejections, because this validator runs on the LOAD path (the 2026-09-03 lesson: a range rejection at boot wipes the whole config). sizeof(confStruct) 192 -> 200, SW_VERSION 35 -> 36; the stored SW35 config is migrated at boot, not reset.
// V2.5-Evo - 2026-09-18 - fm_engage_dist_m: the shared floor kFmEngageDistFloorM is 9.5 m now (was 8.0; the rope is 7.1 m, review finding F3) - the validator reads the constant, so only the comments and the clamp NOTE's advice ("about a third beyond the rope", not "a metre") change here. No confStruct change, sizeof stays 192, SW_VERSION stays 35.
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
// V2.5-Evo - 2026-07-20 - SW34: Added 3 reserved fields to kCfgFields (validation-only; not read by v1): fm_engage_dist_m (0-50, 0=auto), auton_runtime_cap_s (0-3600, 0=disabled), fm_steer_reposition_en (0-1, 0=off)
// V2.5-Evo - 2026-07-25 - A2: fm_engage_dist_m is no longer RESERVED — it is read live by runFmLoop(). Comment/semantics update only; the metadata row (CFG_FLOAT, 0-50, 1 dp) is unchanged. No confStruct change, sizeof stays 184, SW_VERSION stays 34.
// V2.5-Evo - 2026-07-25 - F3: cfgValidateCrossField() gained an fm_engage_dist_m floor — the field is now LIVE (A2) and IS the FM engage distance in metres, so a stored value below the 6.7-7.6 m tow rope defeated the separation interlock outright. Legal values are 0 (auto) or 5.0-50.0 m; anything in between is rejected. Cross-field rule only — no kCfgFields row changed, no confStruct change, sizeof stays 184, SW_VERSION stays 34.
// V2.5-Evo - 2026-07-24 - F1 fix: added the two orphaned SW32 fields (rtm_target_speed_kmh, rtm_align_threshold_deg) to kCfgFields so ?set/?get/web-save can reach them; metadata rows only, no confStruct change, sizeof stays 184, SW_VERSION stays 34
// V2.5-Evo - 2026-07-25 - F3-b: the fm_engage_dist_m floor is raised 5.0 -> 8.0 m AND is no longer a bare literal in this file — cfgValidateCrossField() now reads the single shared kFmEngageDistFloorM from BREmote_V2_Rx.h (the old "Arduino concatenation order stops this file seeing it" note was wrong: that header is included at the top of V2_Integration_Rx.ino, which is compiled first). 5.0 m was below the hazard the error message itself names — the owner's tow rope is 20 ft = 6.10 m, so 5.0-6.1 m was storable and still on-rope. Legal values are now 0 (auto) or 8.0-50.0 m. Threshold + message text only — no kCfgFields row changed, no confStruct change, sizeof stays 184, SW_VERSION stays 34.

// V2.5-Evo - 2026-07-25 - STAGE 0 PART A: kCfgFields row "fm_steer_reposition_en" (u16, 0-1, RESERVED and never read by anything) is replaced by "log_level" (u16, 0-4). The confStruct slot is the SAME slot renamed in place, so sizeof stays 184 and SW_VERSION stays 34 — no config wipe. 0 = unset (behaves as 3), 1 = Basic and 2 = VESC are accepted but currently log as level 3 (reserved for a future storage optimisation), 3 = Developer, 4 = Deep. NOTE FOR CONFIG BACKUPS: a JSON export taken before this change contains the key "fm_steer_reposition_en", which this firmware will reject as unknown on JSON import; the base64 export path is unaffected (it is a raw struct copy of the same size and version). Metadata row only — no struct change, no size change, no SW_VERSION bump.
// V2.5-Evo - 2026-07-25 - F3-c: the fm_engage_dist_m rejection message is rewritten in plain English (owner request) — it now names the setting the way the web UI labels it ("Follow-Me Engage Distance") instead of leading with the raw struct key, states the minimum, gives the REASON (it is the tow-rope safety floor; Follow-Me must never be able to engage while the rider is still on the rope), tells the rider how to choose (measure your rope, add at least a metre), and states that 0/automatic is floored at the same minimum. The threshold is still read from the shared kFmEngageDistFloorM constant, never a bare literal. Message text only — no threshold change, no kCfgFields row changed, no confStruct change, sizeof stays 184, SW_VERSION stays 34.
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
  // V2.5-Evo - 2026-10-02 - P2: range 0-3 -> 0-5. The station numbering is CONTINUOUS and there is
  // no dead-ahead station: 1 = rear-right, 2 = behind, 3 = rear-left, 4 = FRONT-RIGHT, 5 = FRONT-LEFT.
  // There is deliberately no 6. Range only - the field is the same u16 at the same offset, so no
  // struct change, sizeof stays 200, SW_VERSION stays 36. On the RX this field is NOT an auto-arm
  // source (see its web UI text); the TX's live 0xF2 declaration is, and that decoder has accepted
  // 0-5 and failed closed above 5 since 2026-09-19.
  {"followme_mode", CFG_U16, offsetof(confStruct, followme_mode), true, false, true, 0.0f, 5.0f, 0, false},
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
  // V2.5-Evo - 2026-08-16 - gps_dyn_model: u-blox NAV5 dynamic platform model.
  // 0 = default (Sea) | 4 = Automotive | 5 = Sea. Range 0-5 with 1/2/3 rejected by
  // gpsBuildNav5(), which resolves anything that is not an explicit 4 to Sea — so an
  // out-of-range or corrupt value fails toward the conservative model, never toward
  // dynModel 0 (Portable). Sea has a 500 m altitude ceiling; above that use 4.
  {"gps_dyn_model",  CFG_U16, offsetof(confStruct, gps_dyn_model),  true, false, true, 0.0f, 5.0f, 0, false},
  // V2.5-Evo - 2026-04-22 - Phase A GPS anti-spoofing parameters
  {"gps_max_hdop",           CFG_FLOAT, offsetof(confStruct, gps_max_hdop),           true, false, true,  0.5f,  5.0f, 1, false},
  {"gps_max_accel_g",        CFG_FLOAT, offsetof(confStruct, gps_max_accel_g),        true, false, true,  1.0f, 10.0f, 1, false},
  {"gps_max_teleport_kmh",       CFG_FLOAT, offsetof(confStruct, gps_max_teleport_kmh),       true, false, true, 50.0f,500.0f, 1, false},
  {"gps_suspect_threshold",  CFG_U16,   offsetof(confStruct, gps_suspect_threshold),  true, false, true,  1.0f, 10.0f, 0, false},
  // V2.5-Evo - 2026-04-24 - Phase B GPS handshake anti-spoofing parameters
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
  // V2.5-Evo - 2026-07-24 - F1 fix: wire the two SW32 two-phase RTM fields into kCfgFields so ?set/?get and
  // web "Save All" can reach them. Both exist in confStruct (SW32, 2026-05-22) and in WebUiEmbedded fields[]
  // but were never added here — orphaning them exactly like the mag_* fields were (see the SW44 note below):
  // /api/config never returned them and cfgSetValueByKey() rejected them as unknown keys, so the RTM Phase-2
  // speed governor and the Phase 1→2 align threshold were stuck at their defaultConf values. METADATA ROWS
  // ONLY — no struct change, no size change (stays 184), no SW_VERSION bump. Ranges mirror WebUiEmbedded:
  // rtm_target_speed_kmh float 0-20 km/h (0 = governor disabled), rtm_align_threshold_deg u16 10-90 deg.
  {"rtm_target_speed_kmh",     CFG_FLOAT, offsetof(confStruct, rtm_target_speed_kmh),     true, false, true,  0.0f,  20.0f,  1, false},
  {"rtm_align_threshold_deg",  CFG_U16,   offsetof(confStruct, rtm_align_threshold_deg),  true, false, true, 10.0f,  90.0f,  0, false},
  {"logger_en", CFG_U16, offsetof(confStruct, logger_en), true, false, true, 0.0f, 1.0f, 0, false},
  {"paired", CFG_U16, offsetof(confStruct, paired), true, false, true, 0.0f, 1.0f, 0, false},
  {"own_address", CFG_ADDR3, offsetof(confStruct, own_address), true, false, false, 0.0f, 0.0f, 0, false},
  {"dest_address", CFG_ADDR3, offsetof(confStruct, dest_address), true, false, false, 0.0f, 0.0f, 0, false},
  {"wifi_password", CFG_STR8, offsetof(confStruct, wifi_password), true, false, false, 0.0f, 0.0f, 8, false},
  {"motor_ramp_s", CFG_FLOAT, offsetof(confStruct, motor_ramp_s), true, false, true, 0.0f, 4.0f, 2, false},
  // V2.5-Evo - 2026-07-21 - SW44 intent completed: wire the 4 compass-cal fields into kCfgFields so the
  // WebUI/serial config can READ and WRITE them. They were added to confStruct (2026-04-22) and to the
  // WebUI fields[] (SW44, 2026-05-13) but were never added here — orphaning them: /api/config never
  // returned them, so the RX web "Save All" validated them as undefined→"Required" and blocked every edit.
  // These fields already exist in confStruct — this adds METADATA ROWS ONLY: no struct change, no size
  // change (stays 184), no SW_VERSION bump. mag_offset_x/y = int16 (CFG_I16); mag_scale_x/y = float
  // (CFG_FLOAT, 0.1-10.0, 2 dp). Set automatically by ?compasscal; also hand-editable to restore a backup.
  {"mag_offset_x", CFG_I16,   offsetof(confStruct, mag_offset_x), true, false, true, -32768.0f, 32767.0f, 0, false},
  {"mag_offset_y", CFG_I16,   offsetof(confStruct, mag_offset_y), true, false, true, -32768.0f, 32767.0f, 0, false},
  {"mag_scale_x",  CFG_FLOAT, offsetof(confStruct, mag_scale_x),  true, false, true, -10.0f, 10.0f,    2, false},
  {"mag_scale_y",  CFG_FLOAT, offsetof(confStruct, mag_scale_y),  true, false, true, -10.0f, 10.0f,    2, false},
  // V2.5-Evo - 2026-08-16 - mag_scale_x/y range widened to allow NEGATIVE values. A negative
  // mag_scale_y encodes a MIRRORED sensor frame (negating cal_y is exactly the mirror fix),
  // so the sign now carries meaning and a positive-only validator would reject a correct cal.
  // Magnitude is still clamped to [0.1, 10.0] by runCompassCalibration().
  // mag_orientation: compass mounting rotation, 0/90/180/270 deg. Set by ?compasscal (which
  // starts and ends pointing north) or ?magalign. Snapped to cardinals - the 3.2 deg idle
  // noise floor cannot justify finer resolution.
  {"mag_orientation", CFG_U16, offsetof(confStruct, mag_orientation), true, false, true, 0.0f, 270.0f, 0, false},
  // V2.5-Evo - 2026-08-16 - RESERVED slots, validated but unread. They are listed here so a

  // config blob containing them round-trips through ?conf / ?setconf and JSON import without

  // being rejected as unknown. Ranges are wide on purpose - the eventual meaning is unknown,

  // and a slot that rejects its own future value is worse than useless. When one is claimed,

  // RENAME IT IN PLACE here and in confStruct, tighten the range, and do NOT bump SW_VERSION.

  // V2.5-Evo - 2026-09-19 - this row was rsvd_u16_1 (u16, 0-65535, unread). The slot has been RENAMED
  // IN PLACE in confStruct to steer_during_auto - same offset, same uint16_t - so sizeof stays 200,
  // SW_VERSION stays 36 and no config is wiped. Only the key, the range and the meaning change here.
  //   0 = CANCEL (default, what every board already stores): a held stick push ends the automatic
  //       steering - following drops to ARMED, an auto-return stops, the remote exits return-to-me.
  //   1 = TAKE OVER: the push makes the stick steer while it is held; centring it hands the steering
  //       back and cancels nothing. Also clamped (> 1 -> 1) in cfgValidateCrossField() below.
  {"steer_during_auto", CFG_U16, offsetof(confStruct, steer_during_auto), true, false, true, 0.0f, 1.0f, 0, false},

  // V2.5-Evo - 2026-09-19 - this row was rsvd_f32_1 (float, -1e6..1e6, unread). The slot has been RENAMED
  // IN PLACE in confStruct to auto_ramp_s - same offset, same float - so sizeof stays 200, SW_VERSION
  // stays 36 and no config is wiped. 0 = the automatic modes ride motor_ramp_s (what every board already
  // stores); 0.2-4.0 = their own, faster rise-limit. Same 0-4 range and 2 dp as motor_ramp_s; the 0.2
  // floor and the load-path belts are clamps in cfgValidateCrossField() below, never rejections.
  {"auto_ramp_s", CFG_FLOAT, offsetof(confStruct, auto_ramp_s), true, false, true, 0.0f, kAutoRampMaxS, 2, false},

  // V2.5-Evo - 2026-07-20 - SW34 reserved fields (validation only; not read by v1 control law)
  // V2.5-Evo - 2026-07-25 - A2: fm_engage_dist_m is NO LONGER RESERVED — it is now read live by
  // runFmLoop() in RTMState.ino. 0 = auto (engage distance computed from min_dist_m + smoothing band);
  // >0 = the FM engage distance itself, in metres. Range unchanged at 0-50 m; cfgValidateCrossField()
  // below additionally clamps (0, kFmEngageDistFloorM) up. Metadata row is unchanged — comment/semantics only.
  // HOW THE RIDER PICKS THIS VALUE: measure your own tow rope and set this to about a third more
  // than the rope length, so Follow-Me only engages once you have genuinely let go and separated.
  // Example: a 7.1 m rope -> set 9.5 m or more. Setting it at or below your rope length lets FM
  // engage while you are still on the rope. The floor (9.5 m since 2026-09-18; it was 8.0 m, derived
  // for a 6.10 m rope) is the enforced minimum, not a recommendation.
  // V2.5-Evo - 2026-07-25 - F3-c: setting 0 does not bypass that minimum. runFmLoop() applies the
  // same kFmEngageDistFloorM clamp to the AUTO-computed engage distance too, so a small min_dist_m /
  // smoothing-band tuning can no longer produce an on-rope engage distance down the automatic path.
  // V2.5-Evo - 2026-08-16 - auton_runtime_cap_s was RENAMED IN PLACE to gps_dyn_model
  // (registered above, next to gps_chip_type). Same offset, same uint16_t, sizeof stays
  // 184 and SW_VERSION stays 34 - no config wipe. The old key was RESERVED and never read
  // by any logic, so nothing is displaced; a board that had it set simply reads 0 = Sea.
  {"fm_engage_dist_m",       CFG_FLOAT, offsetof(confStruct, fm_engage_dist_m),       true, false, true, 0.0f,  50.0f,   1, false},
  // V2.5-Evo - 2026-07-25 - STAGE 0 PART A: this row was fm_steer_reposition_en. The slot has been
  // RENAMED IN PLACE in confStruct to log_level — same offset, same uint16_t — so sizeof stays 184,
  // SW_VERSION stays 34 and no config is wiped. Only the key, the range and the meaning change here.
  //
  // ACCEPTED RANGE 0-4, and every value in it is stored:
  //   0 = unset. Behaves EXACTLY as level 3. This is what every existing config already holds.
  //   1 = Basic  RESERVED for a future storage optimisation (smaller records) — CURRENTLY LOGS AS 3.
  //   2 = VESC   RESERVED for a future storage optimisation (smaller records) — CURRENTLY LOGS AS 3.
  //   3 = Developer, the full 59-byte record this firmware has always written.
  //   4 = Deep, Developer plus the 6-byte diagnostic block (65 bytes/record; 87 B since 2026-09-19
  //       with the Follow-Me audit block, the raw rider speed and the two mixer outputs).
  //   5 = Everything (V2.5-Evo - 2026-09-19): Deep plus the 22-byte level-5 block = 109 bytes/record,
  //       for test sessions. ACCEPTED RANGE is 0-5 now; anything above 5 is CLAMPED to 5 in
  //       cfgValidateCrossField() below, never rejected (a rejection on the load path wipes config).
  // 1 and 2 are deliberately ACCEPTED rather than rejected: a rider can select them now and a later
  // firmware will honour them without another config migration. They are NOT silently ignored —
  // the fallback to level 3 is stated in the field comment, in both web UIs and in the standalone
  // config tool, so the setting never lies about what it is doing.
  //
  // The FM v2 "steer reposition" feature that owned this slot is NOT cancelled; when it lands it
  // will claim a FRESH confStruct field (a deliberate, announced config-wipe event), not this one.
  {"log_level",              CFG_U16,   offsetof(confStruct, log_level),              true, false, true, 0.0f,  5.0f,    0, false},   // max 4 -> 5 (2026-09-19)
  // V2.5-Evo - 2026-09-19 - SW36: auto-return inside Follow-Me. All three are read live by RTMState.ino.
  //   fm_return_mode     0-1   : power-on default for auto-return (1 = FM_RETURN when the rider stops, 0 = HOLD as before).
  //                              The remote's return gesture overrides it for the session (0xF2 bits 5-6, RAM only).
  //   fm_align_cap       8-80  : throttle cap (0-255 scale) during FM align / FM_RETURN align+engage ramp; 13 = ~5 %.
  //   fm_align_influence 0-100 : mixer steering influence (%) during those align phases only; 100 = one-motor pivot,
  //                              0 = use steering_influence.
  // All three are additionally CLAMPED in cfgValidateCrossField() below, so a stored out-of-range value is
  // corrected on load rather than failing the load. (fm_align_cap joined the other two 2026-10-04, L-2.)
  {"fm_return_mode",         CFG_U16,   offsetof(confStruct, fm_return_mode),         true, false, true, 0.0f,   1.0f,   0, false},
  {"fm_align_cap",           CFG_U16,   offsetof(confStruct, fm_align_cap),           true, false, true, 8.0f,  80.0f,   0, false},
  {"fm_align_influence",     CFG_U16,   offsetof(confStruct, fm_align_influence),     true, false, true, 0.0f, 100.0f,   0, false},
  // V2.5-Evo - 2026-10-02 - P2: fm_front_angle_deg. The field occupies the struct's 2 TAIL PADDING
  // BYTES, so sizeof stays 200, SW_VERSION stays 36 and no config is wiped; every fielded board
  // reads 0 there, which is why 0 MUST mean the shipped default (45 deg).
  //   0     = use kFmFrontAngleDefaultDeg (45 deg off dead ahead).
  //   35-80 = the angle itself. 35 is the owner's settable minimum.
  // The ROW accepts 0-80 so that 0 round-trips through ?set / the web page / a restored blob;
  // the (0, 35) gap is closed by a CLAMP in cfgValidateCrossField() below, never by a rejection -
  // this function runs on the LOAD path and a range rejection there wipes the whole config (the
  // 2026-09-03 lesson). The clamp only ever RAISES the angle, and a larger angle puts the front
  // station further from the rider's line, so there is no version of it that makes the water worse.
  {"fm_front_angle_deg",     CFG_U16,   offsetof(confStruct, fm_front_angle_deg),     true, false, true, 0.0f, kFmFrontAngleMaxDeg, 0, false}
};

const size_t kCfgFieldCount = sizeof(kCfgFields) / sizeof(kCfgFields[0]);

bool cfgValidateCrossField(confStruct &candidate, String &err)
{
  // ---- tx_gps_stale_timeout_ms: never storable below the floor ----
  // A CLAMP, deliberately, not a range rejection. This function runs on the config LOAD path as
  // well as every save path, and a rejection there fails the load and falls back to defaults -
  // wiping config, pairing and compass calibration. Anyone who already stored a 0 (on the strength
  // of a web UI label that called it "disabled") gets corrected on next boot instead of wiped.
  // See kTxGpsStaleFloorMs in BREmote_V2_Rx.h for why zero is fatal rather than permissive.
  if (candidate.tx_gps_stale_timeout_ms < kTxGpsStaleFloorMs)
    candidate.tx_gps_stale_timeout_ms = kTxGpsStaleFloorMs;

  // ---- SW36 auto-return fields: clamp, never reject (V2.5-Evo - 2026-09-19) ----
  // Same reasoning as the floor above: this runs on the LOAD path, and a range rejection there
  // fails the whole load and falls back to defaults - pairing and compass calibration included.
  // fm_return_mode is a 0/1 switch, so anything above 1 means ON; fm_align_influence is a percent,
  // so anything above 100 means full. Both corrections are silent and idempotent.
  if (candidate.fm_return_mode > 1)       candidate.fm_return_mode = 1;
  if (candidate.fm_align_influence > 100) candidate.fm_align_influence = 100;
  // V2.5-Evo - 2026-10-04 - L-2: fm_align_cap gets the same treatment, so that all three SW36
  // fields are now clamped on load rather than two of three. Out of the validated 8-80 range it
  // falls back to the shipped default 13 - never to 0, which would stop the buggy, and never above
  // 80. WHY BOTHER when the kCfgFields row already bounds 8-80 and the read site clamps again: this
  // function runs on the LOAD path, where a range REJECTION fails the whole load and falls back to
  // defaults - the config, pairing and compass-calibration wipe this function exists to prevent. A
  // clamp here means a stored blob holding a stale or corrupt value is corrected in place and the
  // rest of the config survives. Not reachable from a valid stored value; it is for consistency and
  // for the corrupt-blob case.
  if (candidate.fm_align_cap < 8 || candidate.fm_align_cap > 80) candidate.fm_align_cap = 13;
  // ---- V2.5-Evo - 2026-10-02 - P2: fm_front_angle_deg, clamped on the same terms ----
  // Legal shapes: exactly 0 (= use the 45 deg default) or kFmFrontAngleMinDeg..kFmFrontAngleMaxDeg.
  // Anything in the (0, 35) gap is RAISED to 35 and announced; anything above the ceiling is lowered
  // to it silently (that one can only arrive from a stored blob - the row and the web page refuse it
  // first). CLAMPS, NEVER REJECTIONS: this runs on the LOAD path, and the field lives in what used
  // to be the struct's tail padding, so a blob written by an older firmware can legitimately hold
  // anything at all in those two bytes. Rejecting would wipe pairing and the compass calibration
  // over a field the older firmware never even had.
  // THE CORRECTION IS ALWAYS IN THE SAFE DIRECTION, in both branches. Raising a too-small angle
  // moves the front station further off the rider's line. Lowering a too-large one moves it further
  // from dead ahead, i.e. closer to abeam, which is also further from the rider's path. Neither can
  // put the station inside the no-go arc, which is separately and unconditionally clamped at the
  // read site (|station| <= 180 - the effective angle).
  if (candidate.fm_front_angle_deg > (uint16_t)kFmFrontAngleMaxDeg)
    candidate.fm_front_angle_deg = (uint16_t)kFmFrontAngleMaxDeg;
  if (candidate.fm_front_angle_deg != 0 &&
      candidate.fm_front_angle_deg < (uint16_t)kFmFrontAngleMinDeg)
  {
    const unsigned asked = (unsigned)candidate.fm_front_angle_deg;
    candidate.fm_front_angle_deg = (uint16_t)kFmFrontAngleMinDeg;
    Serial.printf("NOTE: Front Station Angle %u deg raised to the %u deg minimum.\n",
                  asked, (unsigned)kFmFrontAngleMinDeg);
    Serial.printf("      The buggy always keeps at least %.0f m beside your line, and it keeps it by\n",
                  (double)kFmFrontLateralMinM);
    Serial.printf("      sitting FURTHER AWAY at a smaller angle (%u deg is about %.0f m out), never closer.\n",
                  (unsigned)kFmFrontAngleMinDeg,
                  (double)(kFmFrontLateralMinM / sinf(kFmFrontAngleMinDeg * 0.01745329f)));
    Serial.println("      Below the minimum that margin would need a radius beyond radio and GPS");
    Serial.println("      confidence, so the angle is held instead. Set 0 for the 45 deg default.");
  }
  // V2.5-Evo - 2026-09-19 - log_level: 5 (Everything) is the top level now; anything above it means
  // "the most detail there is", so it clamps to 5 rather than failing the load. Same reasoning.
  if (candidate.log_level > 5)            candidate.log_level = 5;
  // V2.5-Evo - 2026-09-19 - steer_during_auto is a 0/1 switch on the SAME terms: the slot used to
  // be a RESERVED u16 validated 0-65535, so a value above 1 could in principle be sitting in a
  // stored blob; it is corrected silently on load rather than rejected.
  // V2.5-Evo - 2026-09-25 - R-8: out of range now becomes 0, NOT 1. WHAT WAS WRONG: this field is a
  // PERMISSION - it authorises the rider's stick to take over autonomous steering - and the old
  // clamp sent a corrupt value toward ENABLED, the one direction a clamp must never take a
  // permission. Every other correction in this function moves toward the behaviour-preserving
  // default, and for this field that default is 0 (the stick CANCELS), which is what every fielded
  // board stores. A blob holding 2 now reads as "cancel", the conservative reading; a rider who
  // wants takeover asks for it explicitly with ?set steer_during_auto 1 + ?save.
  if (candidate.steer_during_auto > 1)    candidate.steer_during_auto = 0;
  // V2.5-Evo - 2026-09-25 - R-9: motor_ramp_s NaN guard, the same shape as the auto_ramp_s guard
  // below. WHAT WAS WRONG: validateConfig()'s range test is a pair of comparisons, and BOTH
  // (NaN < min) and (NaN > max) are false, so a NaN in this slot passed validation untouched. It
  // then failed `ramp_s > 0.001f` inside throttleRampStep() (Common/OutputRamp.h) - every
  // comparison against a NaN is false - and the ramp silently switched OFF: full instant throttle
  // where the rider had configured a soft start, with nothing printed to say so. That is precisely
  // the event the ramp exists to prevent, and on a loaded tow rope it lands on the rider's arm.
  // NaN goes to defaultConf.motor_ramp_s, NOT to 0: unlike auto_ramp_s, 0 is a legal value here and
  // it MEANS "ramp off", so sending NaN to 0 would preserve the very behaviour being rejected. The
  // factory default is the behaviour-preserving answer when the stored value is unreadable. A NaN
  // can only arrive from a corrupt SPIFFS blob - ?set and the web page's 0-4 row refuse it first.
  if (!(candidate.motor_ramp_s == candidate.motor_ramp_s))
    candidate.motor_ramp_s = defaultConf.motor_ramp_s;
  // V2.5-Evo - 2026-09-19 - auto_ramp_s: 0 = inherit motor_ramp_s, else kAutoRampMinS..kAutoRampMaxS.
  // The fm_engage_dist_m shape - exactly 0, or at least the floor - as CLAMPS, never rejections: this
  // runs on the LOAD path, and the slot was a RESERVED float validated -1e6..1e6, so a stored blob
  // may hold anything. A value in (0, 0.2) is raised to 0.2 and says so (a ramp that short is
  // instant in all but name, and the rider should know their number moved); NaN or below 0 becomes
  // 0 (inherit) and above the ceiling becomes the ceiling, silently - those two can only come from
  // a blob, never from ?set or the web page, whose 0-4 row refuses them first.
  if (!(candidate.auto_ramp_s == candidate.auto_ramp_s)) candidate.auto_ramp_s = 0.0f;   // NaN
  if (candidate.auto_ramp_s < 0.0f)          candidate.auto_ramp_s = 0.0f;
  if (candidate.auto_ramp_s > kAutoRampMaxS) candidate.auto_ramp_s = kAutoRampMaxS;
  if (candidate.auto_ramp_s > 0.001f && candidate.auto_ramp_s < kAutoRampMinS)
  {
    const float asked = candidate.auto_ramp_s;
    candidate.auto_ramp_s = kAutoRampMinS;
    Serial.printf("NOTE: Automatic modes ramp %.2f s raised to the %.1f s minimum (0 = same as the manual ramp).\n",
                  asked, kAutoRampMinS);
  }

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
  // V2.5-Evo - 2026-07-25 - F3: floor on the manual FM engage-distance override.
  // WHAT THE BUG WAS: the kCfgFields row above range-checks fm_engage_dist_m as 0-50 m and nothing
  // else, so a value like 3 m was accepted and stored. Since A2 that value IS the FM engage distance
  // in METRES — the distance the rider must be beyond before Follow-Me may engage for the first time.
  // An engage distance shorter than the tow rope therefore does not tune the separation interlock,
  // it DEFEATS it: FM would be allowed to engage with the rider still on the rope, which is
  // precisely the situation the latch was added to prevent.
  // V2.5-Evo - 2026-07-25 - F3-b: the floor used to be a bare 5.0f literal here, and 5.0 m was itself
  // BELOW the hazard this message names — the owner's tow rope is 20 ft = 6.10 m, so 5.0-6.1 m was a
  // storable, still-on-rope setting. The floor is now kFmEngageDistFloorM = 8.0 m, defined ONCE in
  // BREmote_V2_Rx.h and shared with the RTMState.ino read-site clamp; the duplicate literal is gone.
  // (The note that used to sit here claimed the Arduino concatenation order stopped this file seeing
  // that constant — wrong: BREmote_V2_Rx.h is included at the top of V2_Integration_Rx.ino, which is
  // concatenated first, so the constant is in scope here.)
  // WHAT THE FIX DOES: only two shapes are legal — exactly 0, meaning auto (the firmware derives the
  // engage distance from Min Distance + Smoothing Band), or at least kFmEngageDistFloorM. Anything in
  // between is rejected with a message that says why. The 0.1f lower compare is the same float "is
  // this really zero" guard RTMState.ino uses at the read site, so the two agree on what is auto.
  // V2.5-Evo - 2026-07-25 - F3-c: the rejection message is rewritten in plain English at the owner's
  // request. WHAT WAS WRONG WITH IT: it opened with the raw struct key (fm_engage_dist_m), which
  // means nothing to a rider looking at a web form labelled "FM Engage Distance", and it explained
  // the limit as "it must clear the tow rope" without ever saying that the 8 m IS the tow-rope
  // safety floor or what the rider should do about it. A safety refusal the rider cannot act on is a
  // refusal they will work around. The message now names the setting the way the UI labels it,
  // states the minimum, gives the reason (Follow-Me must never be able to engage while the rider is
  // still on the rope), and tells them how to pick a value (measure the rope, add a metre). The
  // number is still built from the shared kFmEngageDistFloorM constant — never a bare literal — so
  // the message can never drift away from the threshold it is describing.
  // NOTE for anyone editing this string: it is interpolated raw into a JSON body by
  // webCfgHandleSet()/webCfgHandleSetBatch() in Common/WebConfigEngine.h with no escaping, so it must
  // never contain a double quote or a backslash.
  // V2.5-Evo - 2026-08-16 - CLAMPED rather than rejected, for consistency with the COG-only

  // rule below and because the correction is always in the SAFE direction. Raising a too-small

  // engage distance UP to the tow-rope floor moves Follow-Me FURTHER from the rider, never

  // closer - so a rider who asks for 5 m gets more margin than they requested, not less. There

  // is no version of this clamp that makes the water more dangerous.

  //

  // The old behaviour refused the save outright. That taught the rule, but it also meant a

  // config blob carrying a too-small value - an old backup, a copied config from someone with a

  // shorter rope - was rejected wholesale at boot, which on the SPIFFS load path costs the

  // rider EVERY setting rather than one. Clamping repairs the single field and keeps the rest.

  //

  // Loud on purpose: the rider must learn WHY, or they will set it back next session.

  if (candidate.fm_engage_dist_m > 0.1f && candidate.fm_engage_dist_m < kFmEngageDistFloorM)

  {

    float asked = candidate.fm_engage_dist_m;

    candidate.fm_engage_dist_m = kFmEngageDistFloorM;

    Serial.printf("NOTE: Follow-Me Engage Distance %.1f m raised to the %.1f m minimum.\n",
                  asked, kFmEngageDistFloorM);
    // V2.5-Evo - 2026-09-18 - advice matches the floor's derivation (rope x 1.31): "about a third
    // beyond it", not "a metre beyond it", which a 7.1 m rope would have put inside the floor.
    Serial.println("      That minimum is the tow-rope safety floor: Follow-Me must never be able");
    Serial.println("      to engage while you are still on the rope. Measure your rope and set about");
    Serial.println("      a third beyond it. Setting 0 (automatic) is floored at the same value.");

  }

  // ============================================================

  // V2.5-Evo - 2026-08-16 - COG-ONLY MODE NEEDS ITS ARM GATE RELAXED TOO.

  //

  // rtm_use_compass = 0 disables the compass for STEERING. rtm_compass_required = 1 then still

  // demands a valid heading at arm time - and despite its name that gate does not look for a

  // compass, it calls getRtmHeading() and requires ANY source. In hybrid the compass snapshot

  // satisfies it while the craft sits still. With the compass switched off there is nothing:

  // COG does not exist below rtm_cog_min_speed_kmh, and RTM is armed from a standstill or a

  // drift, which is exactly when there is no course to measure.

  //

  // The result is a silent, misleading failure - RTM refuses to arm with STOP: No valid heading

  // source, and it reads as COG-only mode being broken. It is not; the gate is.

  //

  // Rejected rather than auto-corrected on purpose. Quietly clearing one safety gate because

  // the rider changed a different setting is the kind of helpfulness that surprises someone

  // later. Making them set both means they SEE that turning the compass off also relaxes the

  // arm gate, which is the thing worth understanding when you deliberately disable a sensor.

  //

  // Enforced on every save path - ?set, the RX web portal, the standalone tool, a restored

  // ?setconf blob and SPIFFS load on boot - because cfgValidateCrossField() is called from all

  // of them. There is no way round it, including an old config backup carrying the trap.

  //

  // NOTE: no double quote or backslash in this string - it is interpolated raw into JSON.

  // ============================================================

  // ============================================================

  // V2.5-Evo - 2026-08-16 - COG-ONLY MODE: relax the arm gate automatically.

  //

  // rtm_use_compass = 0 turns the compass off for STEERING. rtm_compass_required = 1 then still

  // demands a valid heading before RTM will arm - and despite its name that gate does not look

  // for a compass, it calls getRtmHeading() and accepts ANY source. In hybrid the compass

  // snapshot satisfies it while the craft sits still. With the compass off there is nothing:

  // COG does not exist below rtm_cog_min_speed_kmh, and RTM is armed from a standstill or a

  // drift - exactly when there is no course to measure. RTM would refuse to arm every time with

  // STOP: No valid heading source, which reads as COG-only mode being broken. It is not.

  //

  // AUTO-CORRECTED rather than rejected, deliberately. This combination is a UI trap, not a

  // hazard: it fails CLOSED - RTM refuses to arm rather than doing anything dangerous - so

  // repairing it cannot create a risk, it only removes a footgun. And COG-only is the mode

  // riders are being pointed at RIGHT NOW to isolate a suspected compass; the first person to

  // reach for it should not have to debug the mode itself. Make it impossible to get wrong on

  // the day it starts being used.

  //

  // Announced, never silent - the rider must see that turning the compass off also relaxed the

  // arm gate, because that IS a real behaviour change. The web portal shows it too: the field

  // reloads as 0.

  //

  // Writing to `candidate` here is intentional despite the function name. It is idempotent, only

  // ever moves settings toward the working combination, and runs on every save path - ?set, web

  // portal, ?applyconf and SPIFFS load - so an old config blob carrying the trap is repaired at

  // boot rather than leaving RTM unarmable until someone works out why.

  // ============================================================

  if (candidate.rtm_use_compass == 0 && candidate.rtm_compass_required != 0)

  {

    candidate.rtm_compass_required = 0;
    Serial.println("NOTE: Heading Source is GPS COG only, so RTM Compass Required was set to 0.");
    Serial.println("      That gate needs a valid heading of ANY kind to arm, and with the compass");
    Serial.println("      off there is none until the buggy is moving - so RTM could never arm.");
    Serial.println("      RTM now steers only above the COG minimum speed, and holds straight below.");

  }

  return true;
}
