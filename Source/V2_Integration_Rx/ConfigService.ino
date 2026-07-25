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
  {"mag_scale_x",  CFG_FLOAT, offsetof(confStruct, mag_scale_x),  true, false, true, 0.1f,      10.0f,    2, false},
  {"mag_scale_y",  CFG_FLOAT, offsetof(confStruct, mag_scale_y),  true, false, true, 0.1f,      10.0f,    2, false},
  // V2.5-Evo - 2026-07-20 - SW34 reserved fields (validation only; not read by v1 control law)
  // V2.5-Evo - 2026-07-25 - A2: fm_engage_dist_m is NO LONGER RESERVED — it is now read live by
  // runFmLoop() in RTMState.ino. 0 = auto (engage distance computed from min_dist_m + smoothing band);
  // >0 = the FM engage distance itself, in metres. Range unchanged at 0-50 m; cfgValidateCrossField()
  // below additionally rejects (0, 8) m. Metadata row is unchanged — comment/semantics only.
  // HOW THE RIDER PICKS THIS VALUE: measure your own tow rope and set this to AT LEAST one metre more
  // than the rope length, so Follow-Me only engages once you have genuinely let go and separated.
  // Example: a 20 ft (6.1 m) rope -> set 8 m or more. Setting it at or below your rope length lets FM
  // engage while you are still on the rope. 8.0 m is the enforced minimum, not a recommendation.
  // V2.5-Evo - 2026-07-25 - F3-c: setting 0 does not bypass that minimum. runFmLoop() applies the
  // same kFmEngageDistFloorM clamp to the AUTO-computed engage distance too, so a small min_dist_m /
  // smoothing-band tuning can no longer produce an on-rope engage distance down the automatic path.
  // auton_runtime_cap_s and fm_steer_reposition_en remain RESERVED and unread.
  {"fm_engage_dist_m",       CFG_FLOAT, offsetof(confStruct, fm_engage_dist_m),       true, false, true, 0.0f,  50.0f,   1, false},
  {"auton_runtime_cap_s",    CFG_U16,   offsetof(confStruct, auton_runtime_cap_s),    true, false, true, 0.0f, 3600.0f,  0, false},
  {"fm_steer_reposition_en", CFG_U16,   offsetof(confStruct, fm_steer_reposition_en), true, false, true, 0.0f,  1.0f,    0, false}
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
  if (candidate.fm_engage_dist_m > 0.1f && candidate.fm_engage_dist_m < kFmEngageDistFloorM)
  {
    err = String("ERR_CROSS:Follow-Me Engage Distance must be 0 (automatic) or at least ") +
          String(kFmEngageDistFloorM, 1) + " m. This " + String(kFmEngageDistFloorM, 1) +
          " m minimum is the tow-rope safety floor: Follow-Me must never be able to engage while you " +
          "are still on the rope. Measure your rope and set at least a metre beyond it (a 20 ft / " +
          "6.1 m rope needs " + String(kFmEngageDistFloorM, 1) + " m or more). Setting 0 does not " +
          "bypass this — automatic is floored at the same " + String(kFmEngageDistFloorM, 1) + " m.";
    return false;
  }
  return true;
}
