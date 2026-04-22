// RX-specific config field table and cross-validation.
// Shared engine is in ../Common/ConfigServiceEngine.h (included via BREmote_V2_Rx.h).

#include <stddef.h>

const CfgFieldSpec kCfgFields[] = {
  {"version", CFG_U16, offsetof(confStruct, version), true, false, true, (float)SW_VERSION, (float)SW_VERSION, 0, true},
  {"radio_preset", CFG_U16, offsetof(confStruct, radio_preset), true, true, true, 1.0f, 3.0f, 0, false},
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
  {"dummy_delete_me", CFG_U16, offsetof(confStruct, dummy_delete_me), true, false, true, 0.0f, 65535.0f, 0, false},
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
