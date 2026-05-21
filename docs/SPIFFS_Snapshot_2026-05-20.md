# SPIFFS Snapshot — 2026-05-20

Pre-water-test config dump. Saved for retrieval. **Do not push to GitHub** (contains wifi_password and MAC addresses).

---

## RX — SW Version 31 · MAC E0:C9:51:7D:78:50

```
version: 31
radio_preset: 2
rf_power: 22
steering_type: 1             ← diff mode CONFIRMED
steering_influence: 60       ← water test value CONFIRMED
steering_inverted: 0
trim: 0
pwm0_min: 1000
pwm0_max: 2000
pwm1_min: 1000
pwm1_max: 2000
failsafe_time: 1000
foil_num_cells: 10
bms_det_active: 0
wet_det_active: 1
rtm_steer_response: 2        ← mid preset (start here)
data_src: 2
gps_en: 1
followme_mode: 2
kalman_en: 1
boogie_vmax_in_followme_kmh: 25.0
min_dist_m: 10.0
followme_smoothing_band_m: 10.0
foiler_low_speed_kmh: 8.0
zone_angle_enter_deg: 35.0
zone_angle_exit_deg: 45.0
near_diag_offset_deg: 45.0
ubat_cal: 0.009555400
ubat_offset: 0.0000
tx_gps_stale_timeout_ms: 3000
gps_chip_type: 1
gps_max_hdop: 2.0
gps_max_accel_g: 3.0
gps_max_teleport_kmh: 80.0
gps_suspect_threshold: 3
gps_max_pair_dist_m: 500.0
gps_max_speed_diff_kmh: 50.0
rtm_vesc_speed_diff_kmh: 20.0
vesc_erpm_per_kmh: 0.0
rtm_rx_enabled: 1
rtm_rx_override_steering: 1
rtm_compass_required: 1
rtm_stop_distance_m: 10
vesc_timeout_s: 6
gps_update_hz: 2
rtm_approach_zone_m: 15
rtm_use_compass: 1
rtm_cog_min_speed_kmh: 3
logger_en: 0                 ← correct — aux button controls logging per session
paired: 1
own_address: 46:C9:E0
dest_address: 46:CB:CC
```

---

## TX — SW Version 26 · MAC CC:CB:51:7D:78:50

```
radio_preset: 2
rf_power: 22
max_gears: 6
startgear: 0
no_lock: 0
throttle_mode: 1             ← no gears — stays here until VESC configured, then → 2
steer_enabled: 1
dynamic_power_start: 85      ← ready for when throttle_mode switches to 2
dynamic_power_step: 5
thr_expo: 70                 ← water test value CONFIRMED
tog_deadzone: 500
tog_diff: 30
tog_block_time: 200
menu_timeout: 5
version: 26
cal_ok: 1
cal_offset: 100
thr_idle: 15195
thr_pull: 11909
tog_left: 12310
tog_mid: 13806
tog_right: 14908
trig_unlock_timeout: 3500
lock_waittime: 2000
gear_change_waittime: 100
gear_display_time: 1000
err_delete_time: 2000
fm_display_mode: 1
steer_expo: 50               (unused)
steer_expo1: 0               (unused)
ubat_cal: 0.000185662
gps_en: 1
followme_mode: 1
kalman_en: 1
speed_src: 5
tx_gps_stale_timeout_ms: 3000
gps_max_hdop: 200            ← very permissive — TX accepts poor GPS (RX is 2.0)
gps_chip_type: 0             ← different chip than RX (RX=1)
rtm_enabled: 1
rtm_hold_duration_s: 5
rtm_arm_window_s: 10
rtm_double_squeeze_en: 0
rtm_throttle_start_pct: 30
rtm_throttle_max_pct: 70
rtm_ramp_duration_s: 5
rtm_disengage_distance_m: 10
rtm_max_runtime_s: 0
rtm_gps_timeout_ms: 2000
fm_hold_duration_s: 5
fm_override_enabled: 1
rtm_display_mode: 0
fm_warn_distance_m: 150
rtm_steer_exit_on_input: 1
fm_arm_window_s: 30
dist_unit: 0
sleep_timeout_s: 300
bt_enabled: 2                ← always-on BLE CONFIRMED
paired: 1
own_address: 46:CB:CC
dest_address: 46:C9:E0
```

---

## Water test checklist — all TX/RX values confirmed

- [x] RX: `steering_type = 1` (diff)
- [x] RX: `steering_influence = 60`
- [x] RX: `rtm_steer_response = 2` (mid preset)
- [x] TX: `thr_expo = 70`
- [x] TX: `dynamic_power_start = 85`, `dynamic_power_step = 5` (ready for mode 2)
- [ ] TX: switch `throttle_mode` 1 → 2 after VESC configured
- [ ] Enable logger via aux button when ready to capture a run (logger_en stays 0 at boot by design)
