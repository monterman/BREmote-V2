# RX Config Snapshot — SW33 (board1)

**Captured:** 2026-06-06 over serial (`?conf`) · COM6
**Board:** RX board1 · MAC `E0C9517D7850` · **SW Version 33** · compiled Jun 6 2026 08:48:30
**Context:** after the SW33 flash (which reset SPIFFS to defaults) + restore of the 3 non-default values.

> To restore this exact config later: `?setconf <blob>` then `?save` (blob is SW33-locked).

---

## Restore blob (SW33 config — exact)
```
IQACABYAAQAyAAEAAADoA9AH6APQB+gDCgAAAAEAAgACAAEAAgABAAAAyEEAACBBAAAgQQAAAEEAAAxCAAA0QgAANEJBjhw8AAAAALgLAAABAEa7nEbLzDEyMzQ1Njc4AAAAAAAAgD8AAIA/AQAAAAAAAEAAAEBAAACgQgMAAAAAAPpDAABIQgAAoEEAAAAAAQABAAEACgAGAAIADwABAAMAAAAAAIBALQAAAAAAQD8=
```

## Battery cal blob (`?setbc <blob>`)
```
ANna2dfV1NLQz83LycjGxMPBv768uri3tbOysK6sq6mnpqSioZ+dm5qYlpWTkZCOjIqJh4WEgoB/fXt5eHZ0c3FvbmxqaGdlY2JgXl1bWVdWVFJRT01LSkhGRUNBQD48Ojk3NTQy
```
`batcal: noload_offset 0.00V, 100% @ 4.17V, 0% @ 2.50V`

---

## Readable config (SW33)
```
version: 33
radio_preset: 2            rf_power: 22                 ← restored (default 20)
--- steering ---
steering_type: 1 (diff)    steering_influence: 50
steering_inverted: 1       ← restored (default 0)       trim: 0
pwm0_min: 1000  pwm0_max: 2000   pwm1_min: 1000  pwm1_max: 2000
failsafe_time: 1000
--- motor / safety ---
motor_ramp_s: 0.75         ← NEW in SW33 (both-motor ramp, seconds; 0=off, 0-4s)
foil_num_cells: 10         bms_det_active: 0            wet_det_active: 1
rtm_steer_response: 2      data_src: 2 (VESC)
--- gps / followme ---
gps_en: 1   followme_mode: 2   kalman_en: 1
boogie_vmax_in_followme_kmh: 25.0   min_dist_m: 10.0
followme_smoothing_band_m: 10.0     foiler_low_speed_kmh: 8.0
zone_angle_enter_deg: 35.0  zone_angle_exit_deg: 45.0   near_diag_offset_deg: 45.0
gps_chip_type: 1            gps_max_hdop: 2.0            gps_max_accel_g: 3.0
gps_max_teleport_kmh: 80.0  gps_suspect_threshold: 3
gps_max_pair_dist_m: 500.0  gps_max_speed_diff_kmh: 50.0
gps_update_hz: 2            tx_gps_stale_timeout_ms: 3000
--- battery ---
ubat_cal: 0.009555400      ubat_offset: 0.0000
--- rtm ---
rtm_vesc_speed_diff_kmh: 20.0   vesc_erpm_per_kmh: 0.0 (VESC speed check OFF)
rtm_rx_enabled: 1          rtm_rx_override_steering: 1
rtm_compass_required: 1    rtm_use_compass: 1
rtm_stop_distance_m: 10    ← RTM SAFE DISTANCE = 10 m (hard stop / Gate 9) — water-test target ✅
rtm_approach_zone_m: 20    ← approach decel-zone outer edge (throttle eases 20m→10m; changed from 15)
rtm_cog_min_speed_kmh: 3   vesc_timeout_s: 6
--- logging / pairing ---
logger_en: 0 (by design)   paired: 1
own_address: 46:BB:9C      ← restored (default 46:C9:E0)  dest_address: 46:CB:CC
wifi_password: 12345678
```

---

## Notes
- **SW33 flash reset SPIFFS** (confStruct 172→176 for `motor_ramp_s`). Defaults are tuned to monterman hardware, so only **3** values needed restoring: `steering_inverted=1`, `rf_power=22`, `own_address=46:BB:9C` — all set + saved.
- **RTM safe distance = `rtm_stop_distance_m` = 10 m** ✅ (the hard stop where RTM hands back to manual). The decel zone `rtm_approach_zone_m` is now **20 m** (throttle eases from 20 m down to the 10 m stop) — changed from 15 m.
- ⚠️ **Restore-blob note:** the base64 blob above was captured *before* the decel change, so it carries `rtm_approach_zone_m=15`. After a blob restore, run `?set rtm_approach_zone_m 20` + `?save`, or re-export `?conf` for a current blob.
- **`motor_ramp_s = 0.75 s`** — new both-motor ramp (smooths throttle + prevents single-motor takeoff; also ramps steering). Tune 0–4 s to taste.
- **Compass detected this boot** (`QMC5883L … Init OK`) — earlier it was "not found"; now present.
- `vesc_erpm_per_kmh = 0` → RTM Phase-C VESC speed cross-check still disabled (uncalibrated).
