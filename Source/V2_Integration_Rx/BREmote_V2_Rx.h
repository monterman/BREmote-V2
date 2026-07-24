// V2.5-Evo - 2026-07-24 - F9: VescLogData +6 bytes (tx_distance_dx10, rssi_dbm, snr_dx10) for owner-requested distance + link-quality CSV columns; sizeof 53->59; old SPIFFS logs misparse after this flash; NO confStruct change, SW_VERSION stays 34
// V2.5-Evo - 2026-07-20 - SW33->34 config bump + defaultConf bake: appended THREE reserved confStruct slots — fm_engage_dist_m (float, 0=auto), auton_runtime_cap_s (uint16_t, 0=disabled), fm_steer_reposition_en (uint16_t, 0=off). All three are default-off storage slots and are NOT read by v1 control law — bundled together so the v2 features that will read them need NO second config wipe. sizeof(confStruct) 176->184 (float+u16+u16, naturally aligned, no tail pad); static_assert updated to 184. defaultConf carries the factory default configuration (compass cal fields made explicit, neutral). Behavior-IDENTICAL control law — config-layer only, no FM/RTM logic change. SPIFFS config IS reset by this flash (struct size changed); this is the one intended config-wipe event.
// V2.5-Evo - 2026-07-20 - FM control brain (Fable v1.4): repurposed the unused reserved_tx_imu telemetry byte (index 16) as fm_flags — the coherent FM engagement sub-state the TX display consumes ([0]armed [1]engaged [2]armed-not-ready [3]fault-stop-sticky). No confStruct change, no telemetry-packet size change (byte was already present) — SW_VERSION stays 33, sizeof(confStruct) stays 176, SPIFFS config is NOT reset by this flash.
// V2.5-Evo - 2026-07-19 - P3 FM: added fm_rx_active + fm_throttle_cap runtime atomics for the Follow-Me state machine. No confStruct change (FM reuses the 8 existing FM params) — SW_VERSION stays 33, sizeof stays 176, SPIFFS config is NOT reset by this flash.
// V2.5-Evo - 2026-07-20 - FM engagement semantics: added fm_mode_last_rx_ms atomic (0xF2 declaration age, drives the 95 s mode-age expiry); R6 comment cleanup on the zone_angle_enter/exit + near_diag_offset block (described a non-existent engagement cone, wrong mode numbers, inverted signs, false "CURRENTLY UNUSED"). No confStruct change — sizeof stays 176, SW_VERSION stays 33, SPIFFS config is NOT reset by this flash.
// V2.5-Evo - 2026-07-19 - FM triage: log the steering byte actually applied by calcPWM() (g_effective_steer global + VescLogData.effective_steer_log); VescLogData sizeof 52→53; old SPIFFS logs misparse after this flash; no confStruct change, SW_VERSION unchanged
// V2.5-Evo - 2026-07-18 - FM mode mapping canonicalized to TX convention (1=Near-Right, 2=Behind, 3=Near-Left). Labels/comment only — no struct/SW_VERSION change. [2026-07-24 F4 correction: the earlier "2→1 preserves Near-Right default" note was stale — that edit never landed; defaultConf.followme_mode is and stays 2 (Behind), the shipped defensive default. All surfaces now agree on 2.]
// V2.5-Evo - 2026-05-22 - SW32: Two-phase RTM throttle: rtm_align_threshold_deg + rtm_target_speed_kmh; sizeof 164→172; SW_VERSION 31→32
// V2.5-Evo - 2026-05-09 - Bundle 9-Final: Added USB CDC On Boot compile-time guard
// V2.5-Evo - 2026-05-11 - E7 Fix: VescLogData +1 byte (error_code_log); sizeof 51→52; old SPIFFS logs misparse after this flash
// V2.5-Evo - 2026-05-08 - Bundle 1: RTM/FM steering preset system (rtm_steer_response 0-4); SW_VERSION 30→31; sizeof unchanged at 164; VescLogData +4 bytes for tuning telemetry
// V2.5-Evo - 2026-05-06 - LOG-EXT-1: VescLogData extended with heading source debug fields (12 fields, +18 bytes)
// V2.5-Evo - 2026-05-06 - D3-Fix: rtm_use_compass + rtm_cog_min_speed_kmh changed uint8_t→uint16_t for ConfigService CFG_U16 compatibility; SW_VERSION 29→30; sizeof 160→164
// V2.5-Evo - 2026-05-06 - D3: Added rtm_use_compass + rtm_cog_min_speed_kmh; sizeof stays 160 (fills tail pad); SW_VERSION 28→29
// V2.5-Evo - 2026-05-01 - Release: DEBUG_RX commented out for production build
// V2.5-Evo - 2026-04-30 - RTM approach decel zone: rtm_approach_zone_m SPIFFS param; rtm_approach_cap atomic global; sizeof 156→160
// V2.5-Evo - 2026-04-30 - Rename: gps_max_jump_kmh → gps_max_teleport_kmh (clarity)
// V2.5-Evo - 2026-04-29 - Bundle B: vesc_timeout_s SPIFFS param replaces hardcoded 20s VESC timeout
// V2.5-Evo - 2026-04-22 - Added gps_chip_type field to confStruct (GPS module selector); sizeof 108→112; updated defaultConf
// V2.5-Evo - 2026-04-22 - Added Phase A GPS anti-spoofing params to confStruct; sizeof 112→128; updated defaultConf
// V2.5-Evo - 2026-04-24 - Added rx_tx_gps_lat/lng/timestamp globals for 0xF3 meta-packet reception
// V2.5-Evo - 2026-04-24 - Added Phase B GPS handshake params to confStruct; sizeof 128→136; updated defaultConf
// V2.5-Evo - 2026-04-25 - P7: Added RTM Phase C + RX safety params; VESC_MORE_VALUES; sizeof 136→152
// V2.5-Evo - 2026-04-27 - P8: TelemetryPacket adds rtm_distance at index 5; link_quality moved to index 6
// V2.5-Evo - 2026-04-25 - P7: Added rtm_rx_active, rtm_rx_emergency_stop, rtm_steer_override, fm_mode_runtime globals
// V2.5-Evo - 2026-04-25 - P7 fix: Changed RTM volatile globals to std::atomic for cross-core safety (core 0 PWM task / core 1 loop task)

// ============================================================
// V2.5-Evo - 2026-05-09 - Bundle 9-Final: USB CDC On Boot guard
//
// ESP32-C3 chip-level hardware default: GPIO 18 = USB D-, GPIO 19 = USB D+.
// RX firmware uses GPIO 18/19 as UART for the BN-880 GPS via Serial1.
// If "USB CDC On Boot" is enabled at compile time, the ESP32-C3 USB
// peripheral claims GPIO 18/19 internally and Serial1.begin() silently
// fails — GPS init never reaches the module, no fix is ever acquired,
// hours of debugging follow.
//
// REQUIRED: Arduino IDE → Tools → USB CDC On Boot → Disabled
//   OR     arduino-cli --fqbn esp32:esp32:esp32c3:CDCOnBoot=default
//
// Debug Serial output goes via UART0 (GPIO 20/21) → CH340 USB-to-UART chip
// → USB connector. Same physical USB cable, same COM port, no debug loss.
// ============================================================
#if defined(ARDUINO_USB_CDC_ON_BOOT) && (ARDUINO_USB_CDC_ON_BOOT != 0)
#error "RX firmware requires USB CDC On Boot = Disabled. ESP32-C3 USB peripheral claims GPIO 18/19 (used by Serial1 for GPS) when CDC On Boot is enabled. Set Tools -> USB CDC On Boot -> Disabled in Arduino IDE, OR pass :CDCOnBoot=default to arduino-cli's --fqbn argument. See file header for full explanation."
#endif

/*
** Includes
*/
#include <Arduino.h>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <RadioLib.h> //V7.1.2
#include <Wire.h>
#include <Adafruit_AW9523.h> //V1.0.5, BusIO 1.17.0
#include "driver/rmt_tx.h"
#define RMT_TX_GPIO_NUM  GPIO_NUM_9
#include <Ticker.h>
#include "esp_task_wdt.h"
#include "FS.h"
#include "SPIFFS.h"
#include "mbedtls/base64.h"

// Uncomment the line below to enable WiFi AP configuration mode
#define WIFI_ENABLED

#ifdef WIFI_ENABLED
#include <WiFi.h>
#include <WebServer.h>
#endif

#include "vesc_datatypes.h"
#include "vesc_buffer.h"
#include "vesc_crc.h"

#include <TinyGPS++.h> //TinyGPSPlus 1.0.3 Mikal Hart

#define SW_VERSION 34  // V2.5-Evo — 34 = added fm_engage_dist_m / auton_runtime_cap_s / fm_steer_reposition_en reserved slots + defaultConf carries factory default config (compass cal, near_diag_offset 45); first flash resets all RX SPIFFS config to defaults
const char* CONF_FILE_PATH = "/data.txt";
const char* BC_FILE_PATH = "/batconf.txt";

/*
** Structs
*/
struct confStruct {
    //Version
    uint16_t version;
    
    uint16_t radio_preset; //1: 868MHz (EU), 2: 915MHz (US/AU)
    int16_t rf_power; //Tx power from -9 to 22

    uint16_t steering_type; //0: single motor, 1: diff motor, 2: servo
    uint16_t steering_influence; //How much (percentually) the steering influences the motor speeds
    uint16_t steering_inverted; //If steering is inverted or not
    int16_t trim; //Trim the steering

    //PWM min and max
    uint16_t PWM0_min;
    uint16_t PWM0_max;
    uint16_t PWM1_min;
    uint16_t PWM1_max;

    uint16_t failsafe_time; //Time after last packet until failsafe

    //Foil battery voltage settings
    uint16_t foil_num_cells; //Amount of cells in series e.g. 14 for a "14SxP" pack

    //Sensors
    uint16_t bms_det_active;
    uint16_t wet_det_active;

    uint16_t rtm_steer_response;  // Steering preset index 0-4 for RTM/FM heading controller.
                                  // 0 = Very Soft (big waves, aggressive surfer)
                                  // 1 = Soft (choppy water)
                                  // 2 = Normal (DEFAULT — mixed conditions)
                                  // 3 = Sharp (calm water, RC use)
                                  // 4 = Very Sharp (glass-flat, no waves)
                                  // Drives kSteerPresets[] table in RTMState.ino which
                                  // sets PID gains (Kp, Kd) + bearing filter time constant.

    //UART config
    uint16_t data_src; //0: off, 1:analog, 2: VESC UART

    // GPS features related flags
    uint16_t gps_en;         // GPS runtime enable flag (0=disabled, 1=enabled)
    uint16_t followme_mode;  // Follow-me mode (0=disabled, 1=near_right, 2=behind, 3=near_left) — canonical mapping, matches TX + README
    uint16_t kalman_en;      // Kalman filter runtime enable flag (0=disabled, 1=enabled)

    //Follow-me
    float boogie_vmax_in_followme_kmh; // Maximum boogie speed in follow-me mode (km/h)
    float min_dist_m; // minimum allowed distance to the foiler
    float followme_smoothing_band_m; // smoothing band above min distance
    float foiler_low_speed_kmh; // low-speed threshold for safety stop (hysteresis)
    // V2.5-Evo - 2026-07-20 - R6: comment block corrected. It previously described an
    // "engagement cone" gate that does not exist, marked all three params "CURRENTLY UNUSED"
    // (the FM geometry consumes all three), and gave the wrong mode numbers with the wrong
    // signs for the diagonal offset. The values and ranges themselves are unchanged.

    // DIAGONAL-BLEND SCHMITT — ENTER half-angle (degrees). This is NOT an engagement gate:
    // it decides whether the buggy is lined up closely enough BEHIND the rider to apply the
    // diagonal side offset, or whether it should just sit directly behind. Measured as the
    // angle between the rider->buggy bearing and "directly behind the rider".
    // Below this angle the diagonal offset is applied (see computeFmTarget in RTMState.ino).
    // Range: 5-90°. Default 35°.
    float zone_angle_enter_deg;

    // DIAGONAL-BLEND SCHMITT — EXIT half-angle (degrees). Once the diagonal is applied it is
    // dropped again only when the off-axis angle exceeds this value. MUST be > zone_angle_enter_deg
    // by 5-15° — the hysteresis stops an unstable rider course from whipping the target point
    // across the rider's wake from one side to the other.
    // Range: 10-95°. Default 45°.
    float zone_angle_exit_deg;

    // NEAR-MODE DIAGONAL OFFSET (degrees from "directly behind the rider").
    // Applied as target_bearing = rider_course + 180 + offset, in this board's one bearing
    // convention (degrees CLOCKWISE from North), where "Near-Right"/"Near-Left" mean the side
    // the buggy ends up on RELATIVE TO THE RIDER as the rider faces along their course:
    //   followme_mode=1 (Near Right): offset = -near_diag_offset_deg  (behind-and-right)
    //   followme_mode=2 (Behind)    : offset = 0
    //   followme_mode=3 (Near Left) : offset = +near_diag_offset_deg  (behind-and-left)
    // 0° = directly behind, 90° = beside the rider. Diagonal placement keeps the buggy out of
    // the rider's wake/spray path. Authoritative derivation: the OFFSET SIGN CONVENTION block
    // above computeFmTarget() in RTMState.ino.
    // Range: 0-90°. Default 45°.
    float near_diag_offset_deg;
    
    //System parameters
    float ubat_cal; //ADC to volt cal for bat meas
    float ubat_offset; //Offset to add to analog/vesc measurement

    uint16_t tx_gps_stale_timeout_ms; // TX GPS data stale timeout (ms)

    //Logger
    uint16_t logger_en; // BREmote Logger runtime enable flag (0=disabled, 1=enabled)

    //Comms
    uint16_t paired;
    uint8_t own_address[3];
    uint8_t dest_address[3];
    char wifi_password[8];  // WPA2 AP password, exactly 8 chars (no null terminator)

    // ---> NEW COMPASS CALIBRATION VARIABLES <---
    int16_t mag_offset_x;
    int16_t mag_offset_y;
    float mag_scale_x;
    float mag_scale_y;

    // ============================================================
    // V2.5-Evo - 2026-04-22 - GPS CHIP TYPE SELECTOR
    //
    // !!! IMPORTANT: Adding this field changed sizeof(confStruct)  !!!
    // !!! from 108 bytes to 112 bytes.                             !!!
    // !!! On first V2.5-Evo boot after this change, SPIFFS config  !!!
    // !!! failed the size check — ALL RX SETTINGS RESET TO        !!!
    // !!! DEFAULTS. One-time migration; complete on existing units. !!!
    // ============================================================
    uint16_t gps_chip_type;  // 0=BN-220, 1=BN-880+compass (default), 2=M10 no compass, 3=M10+compass; range 0-3

    // ============================================================
    // V2.5-Evo - 2026-04-22 - PHASE A GPS ANTI-SPOOFING PARAMETERS
    //
    // These four parameters control the always-on Phase A anti-
    // spoofing filter in GPS.ino. A reading is rejected if ANY
    // check fails. After gps_suspect_threshold consecutive
    // rejections, gps_rejected is set and RTM arming is blocked.
    //
    // !!! Adding these fields changed sizeof(confStruct) 112→128.  !!!
    // !!! On first V2.5-Evo boot after this change, SPIFFS reset  !!!
    // !!! ALL settings to defaults. One-time migration; complete.  !!!
    // ============================================================
    float    gps_max_hdop;            // Max HDOP for a valid fix; range 0.5-5.0; default 2.0; dimensionless
    float    gps_max_accel_g;         // Max implied acceleration between readings; range 1.0-10.0G; default 3.0G
    float    gps_max_teleport_kmh;        // Max position-implied speed for teleport check; range 50-500 km/h; default 80
    uint16_t gps_suspect_threshold;   // Consecutive failures before GPS marked rejected; range 1-10; default 3

    // ============================================================
    // V2.5-Evo - 2026-04-24 - PHASE B GPS HANDSHAKE ANTI-SPOOFING PARAMETERS
    //
    // These two parameters control Phase B, which runs every time a
    // 0xF3 GPS meta-packet is received from TX (at most every 30s).
    //
    // Distance check: TX-RX Haversine distance must be <
    //   gps_max_pair_dist_m or RTM arming is blocked.
    // Speed consistency check: TX implied speed (from consecutive
    //   meta-packet positions) must be within gps_max_speed_diff_kmh
    //   of RX GPS speed or arming is blocked.
    //
    // !!! Adding these fields changes sizeof(confStruct) 128→136. !!!
    // !!! On first flash after this change, SPIFFS resets ALL      !!!
    // !!! settings to defaults. After flashing:                    !!!
    // !!!   1) Re-pair TX and RX                                   !!!
    // !!!   2) Re-configure all settings via web UI                !!!
    // !!!   3) Re-calibrate compass (runcal)                       !!!
    // !!!   4) Verify Phase B defaults (500 m, 50 km/h)            !!!
    // ============================================================
    float gps_max_pair_dist_m;      // Max plausible TX-RX distance at handshake; range 50-2000 m; default 500 m
    float gps_max_speed_diff_kmh;   // Max TX-RX speed difference for handshake; range 10-200 km/h; default 50 km/h

    // ============================================================
    // V2.5-Evo - 2026-04-25 - PRIORITY 7: RTM PHASE C + RX SAFETY PARAMETERS
    //
    // sizeof grows 136->152. Layout:
    //   float rtm_vesc_speed_diff_kmh  (4)
    //   float vesc_erpm_per_kmh        (4)
    //   uint16_t rtm_rx_enabled        (2)
    //   uint16_t rtm_rx_override_steer (2)
    //   uint16_t rtm_compass_required  (2)
    //   uint16_t rtm_stop_distance_m   (2) — fills former 2-byte tail padding; sizeof stays 152
    //
    // First flash of P7 firmware resets all RX settings to defaults.
    // After flashing: re-pair TX/RX, re-enter all settings, re-run runcal.
    // ============================================================
    float    rtm_vesc_speed_diff_kmh;    // Phase C: max GPS vs VESC speed diff; 5-50 km/h; default 20.0
    float    vesc_erpm_per_kmh;          // ERPM per km/h (vehicle-specific); default 0.0 (0=skip VESC check)
    uint16_t rtm_rx_enabled;             // RX-side RTM master enable; 0=off, 1=on; default 1
    uint16_t rtm_rx_override_steering;   // Allow RTM to override steering; 0=off, 1=on; default 1
    uint16_t rtm_compass_required;       // Require valid compass for RTM arming; 0=no, 1=yes; default 1
    uint16_t rtm_stop_distance_m;        // Hard stop radius in metres; RTM stops when within this dist of TX; 1-50; default 10

    // V2.5-Evo - 2026-04-29 - BUNDLE B: VESC UART TIMEOUT
    // Set to 6s (down from the original hardcoded 20s) to minimise stale VESC data.
    // At 20s the TX display would show a valid battery % and FET temp for up to 20s after
    // the VESC UART connection dropped — misleading the rider. 6s matches the typical
    // VESC polling cadence (data_src=2 polls every ~1s) with room for 5 missed packets.
    // Minimum 5s: going lower causes false N/A during normal VESC dropout transients
    // (e.g. heavy regen braking briefly interrupts UART). Maximum 60s for diagnostic use.
    uint16_t vesc_timeout_s;  // 5-60 s; default 6; how long without a VESC UART packet before bat/temp shown as N/A

    // V2.5-Evo - 2026-04-30 - BUNDLE E: GPS POLLING RATE
    uint16_t gps_update_hz;   // 1-10 Hz; default 2; how often per second to drain the GPS UART (2=500ms, 5=200ms)

    // V2.5-Evo - 2026-04-30 - RTM APPROACH DECEL ZONE
    // Distance from TX at which the approach throttle ramp begins during active RTM.
    // Throttle cap = thr × (dist − rtm_stop_distance_m) / (rtm_approach_zone_m − rtm_stop_distance_m)
    // Result: full throttle at the outer edge; cap reaches 0 at rtm_stop_distance_m; Gate 9 hard stop still applies.
    // Set to 0 to disable the decel zone and use Gate 9 hard stop only.
    uint16_t rtm_approach_zone_m;  // 0=disabled, 5-100 m; default 15; outer edge of RTM approach decel zone

    // ============================================================
    // V2.5-Evo - 2026-05-06 - D3: RTM HEADING SOURCE SELECTION
    //
    // These two parameters control which heading source RTM steering uses.
    // Bench-test data (?magtest) confirmed compass-only steering is unsafe
    // on this hardware: motor current biases compass heading by 100° or more
    // even at 20% throttle. GPS course-over-ground (COG) is unaffected by
    // motor EMI and is the preferred heading source whenever the buggy is
    // moving fast enough for COG to be reliable (~3 km/h default).
    //
    // Modes:
    //   0 = GPS COG only — compass disabled for steering (safest if compass biased)
    //   1 = Hybrid (DEFAULT) — GPS COG primary, compass snapshot at low speed
    //   2 = Compass only — DIAGNOSTIC USE, DO NOT USE ON WATER. Bench tests confirm
    //                      motor current biases compass by 100° or more during
    //                      operation. Setting this on water risks RTM steering
    //                      the buggy in the wrong direction. For non-EMI builds
    //                      that have proven clean compass behavior under load only.
    //
    // !!! Adding these fields changes sizeof(confStruct) 160→164,         !!!
    // !!! and bumps SW_VERSION 29→30. SPIFFS resets ALL settings to       !!!
    // !!! defaults again (the second time, because D3-Fix changes layout). !!!
    // !!! After flashing:                                                  !!!
    // !!!   1) Re-pair TX and RX                                           !!!
    // !!!   2) Re-configure all settings via web UI                        !!!
    // !!!   3) Re-calibrate compass (runcal)                               !!!
    // !!!   4) Verify rtm_use_compass = 1 (hybrid default)                 !!!
    // !!!   5) Verify rtm_cog_min_speed_kmh = 3                            !!!
    // ============================================================
    uint16_t rtm_use_compass;        // 0=GPS COG only; 1=Hybrid (default); 2=Compass only DIAGNOSTIC ONLY DO NOT USE ON WATER
    uint16_t rtm_cog_min_speed_kmh;  // Min GPS speed for COG to be primary heading source; 1-15 km/h; default 3

    // ============================================================
    // V2.5-Evo - 2026-05-22 - SW32: TWO-PHASE RTM THROTTLE CONTROL
    //
    // Phase 1 (Align): when |heading_error| > rtm_align_threshold_deg, throttle is
    //   suppressed to ~5% so the buggy pivots toward the target without driving away.
    //   At near-zero throttle, motor current is minimal — compass bias is also reduced,
    //   so hybrid heading mode has cleaner compass snapshot data during alignment.
    //
    // Phase 2 (Run): once aligned, throttle is governed by GPS speed so behaviour is
    //   consistent across different boogies regardless of motor/prop curve.
    //   rtm_target_speed_kmh == 0 disables the governor (approach decel zone only).
    //
    // sizeof grows 164 → 172. SW_VERSION 31 → 32. First flash resets SPIFFS config.
    // ============================================================
    float    rtm_target_speed_kmh;      // Phase 2 run speed cap (GPS-based); 0=disabled; 0-20 km/h; default 4.0
    uint16_t rtm_align_threshold_deg;   // Phase 1→2 transition: heading error below which run phase begins; 10-90°; default 45

    // V2.5-Evo - 2026-06-05 - SW33: MOTOR RAMPING (seconds). Time for a motor output to rise
    // 0->full. Applied to BOTH motor channels — smooths the throttle AND prevents a single motor
    // from taking off (throttle- or steering-driven). Fall is instant (release/failsafe/e-stop/
    // straightening drop immediately). NOTE: this also ramps differential steering — a sharp turn
    // builds over this time. 0 = instant/off. sizeof grows 172->176; SW_VERSION 32->33; SPIFFS resets.
    float    motor_ramp_s;              // 0=off/instant, 0-4 s; default 0.75

    // V2.5-Evo - 2026-07-20 - SW34 reserved slots (added together so only ONE config wipe is needed).
    // NONE of these three are read by v1 code — they are storage slots so v2 features are code-only, no re-wipe.
    // fm_engage_dist_m: RESERVED fixed engage-distance override for the FM separation latch. 0 = auto
    //   (current behavior: d_engage = kFmEngageFactor * (min_dist_m + band), computed live in RTMState.ino).
    //   v1 does NOT read this; wiring it live (with 0=auto sentinel) is a later, separately-audited commit.
    float    fm_engage_dist_m;         // 0 = auto (do not read in v1); range 0-50 m
    // auton_runtime_cap_s: RESERVED shared RTM/FM autonomous-runtime cap. 0 = disabled (matches TX rtm_max_runtime_s default). Not read by v1.
    uint16_t auton_runtime_cap_s;      // 0 = disabled; range 0-3600 s
    // fm_steer_reposition_en: RESERVED Option C — continuous steer-driven repositioning of the follow ANGLE
    //   around the rider's radius. Distinct from the rtm_steer_exit_on_input blend. disabled till v2 — v1 must NOT read it.
    uint16_t fm_steer_reposition_en;   // 0 = off (disabled till v2); range 0-1
};
static_assert(sizeof(confStruct) == 184, "confStruct size mismatch — expected 184 bytes. Update this assert if you change the struct.");  // 176->184: +fm_engage_dist_m(float 4) +auton_runtime_cap_s(u16 2) +fm_steer_reposition_en(u16 2), all naturally aligned, no tail pad (2026-07-20 SW34)  // 172->176 motor_ramp_s float (2026-06-05 SW33)  // 112->128 Phase A; 128->136 Phase B; 136->152 P7 RTM; 152->156 Bundle B; 156 unchanged BundleE; 156->160 rtm_approach_zone_m (uint16_t + 2-byte tail pad) (2026-04-30); D3 rtm_use_compass + rtm_cog_min_speed_kmh (2x uint8_t) fill the 2-byte tail pad — sizeof stays 160 (2026-05-06); D3-Fix: uint8_t→uint16_t for ConfigService compatibility, sizeof unchanged at 164 (2026-05-06); Bundle 1: dummy_delete_me renamed to rtm_steer_response in-place, sizeof unchanged at 164 (2026-05-08)
confStruct usrConf;
  //The orginal confs were:  ##// confStruct defaultConf = {SW_VERSION, 1, 0, 0, 50, 0, 0, 1500, 2000, 1500, 2000, 1000, 10, 0, 1, 0, 0, 0, 0, 0, 25.0f, 10.0f, 10.0f, 5.0f, 35.0f, 45.0f, 45.0f, 0.0095554f, 0.0, 1000, 1, 0, {0, 0, 0}, {0, 0, 0}, {'1','2','3','4','5','6','7','8'}};
  // Factory default configuration.
confStruct defaultConf = {SW_VERSION, 2, 22, 1, 50 /*steering_influence: conventional default (0-100)*/, 0 /*steering_inverted: 0 = conventional default; a fresh build MUST verify steering direction wheels-up (FM steers toward rider) before trusting FM.*/, 0, 1000, 2000, 1000, 2000, 1000, 10, 0, 1, 2, 2, 1, 2, 1, 25.0f, 10.0f, 10.0f, 8.0f, 35.0f, 45.0f, 45.0f, 0.0095554f, 0.0f, 3000, 0, 0, {0, 0, 0}, {0, 0, 0}, {'1','2','3','4','5','6','7','8'}, // wifi_password below: documented DEFAULT AP password "12345678" — change before use
  // V2.5-Evo - 2026-04-22 - Compass calibration fields (previously implicit zeros).
  // Made explicit here so gps_chip_type can follow. Safe neutral values:
  // offsets=0 (no bias), scales=1.0f (unity gain = no correction applied).
  0, 0,   // mag_offset_x, mag_offset_y (neutral zero bias; re-derived via 'runcal')
  1.0f, 1.0f, // mag_scale_x, mag_scale_y (unity gain = no correction until calibrated)
  // V2.5-Evo - 2026-04-22 - GPS chip type: 1 = BN-880 (GPS+compass). RX default.
  1,          // gps_chip_type (1 = BN-880 + compass; run 'runcal' after first boot)
  // V2.5-Evo - 2026-04-22 - Phase A GPS anti-spoofing defaults
  2.0f,       // gps_max_hdop:           max HDOP for valid reading (range 0.5-5.0)
  3.0f,       // gps_max_accel_g:        max implied acceleration (range 1.0-10.0 G)
  80.0f,      // gps_max_teleport_kmh:       max teleport-implied speed (range 50-500 km/h; default lowered 200→80 2026-04-30)
  3,          // gps_suspect_threshold:  consecutive failures before GPS rejected (range 1-10)
  // V2.5-Evo - 2026-04-24 - Phase B GPS handshake anti-spoofing defaults
  500.0f,     // gps_max_pair_dist_m:    max TX-RX pairing distance (range 50-2000 m)
  50.0f,      // gps_max_speed_diff_kmh: max TX-RX speed difference (range 10-200 km/h)
  // V2.5-Evo - 2026-04-25 - Priority 7 RTM Phase C + RX safety defaults
  20.0f,      // rtm_vesc_speed_diff_kmh: max GPS vs VESC speed diff (5-50 km/h)
  0.0f,       // vesc_erpm_per_kmh: 0 = skip Phase C VESC check until calibrated
  1,          // rtm_rx_enabled: 1 = RTM enabled on RX side
  1,          // rtm_rx_override_steering: 1 = RTM may override steering
  1,          // rtm_compass_required: 1 = compass required for RTM arming
  // V2.5-Evo - 2026-04-26 - CRITICAL FIX: rtm_stop_distance_m was missing from defaultConf; zero-init
  // would have set it to 0, making Gate 9 check (dist_m < 0.0f) never fire — permanently
  // disabling the hard stop that prevents the buggy from hitting the user.
  10,  // rtm_stop_distance_m: safe default 10 m (>= 8 m GPS floor); RTM hard-stop radius
  // V2.5-Evo - 2026-04-29 - Bundle B: vesc_timeout_s replaces hardcoded 20s VESC connection timeout
  6,          // vesc_timeout_s: seconds without VESC UART packet before bat/temp shown as N/A (range 5-60s; default 6s)
  // V2.5-Evo - 2026-04-30 - Bundle E: gps_update_hz replaces hardcoded 1Hz GPS poll cadence
  2,          // gps_update_hz: GPS NMEA polling rate in Hz (range 1-10 Hz; default 2 Hz = 500ms interval)
  // V2.5-Evo - 2026-04-30 - RTM approach decel zone default
  12,         // rtm_approach_zone_m: outer edge of RTM throttle decel zone (0=disabled, 5-100 m)
  // V2.5-Evo - 2026-05-06 - D3: RTM heading source selection defaults
  1,          // rtm_use_compass: 1 = Hybrid (GPS COG primary, compass snapshot at low speed). 0=COG only, 2=compass only DIAGNOSTIC.
  3,          // rtm_cog_min_speed_kmh: GPS speed threshold below which compass snapshot is used; 1-15 km/h; default 3
  // V2.5-Evo - 2026-05-22 - SW32: Two-phase RTM throttle defaults
  4.0f,       // rtm_target_speed_kmh: Phase 2 GPS speed cap; 4 km/h default; 0=disabled
  45,         // rtm_align_threshold_deg: heading error threshold for Phase 1→2 transition; 45° default
  // V2.5-Evo - 2026-06-05 - SW33: motor ramping (secs) default
  0.75f,      // motor_ramp_s: motors ramp 0->full over 0.75s (0=instant/off, 0-4s); also ramps steering
  // V2.5-Evo - 2026-07-20 - SW34 reserved slots (not read by v1)
  0.0f,       // fm_engage_dist_m: 0 = auto (RTMState computes d_engage live; reserved for v2)
  0,          // auton_runtime_cap_s: 0 = disabled
  0           // fm_steer_reposition_en: 0 = off (Option C, disabled till v2)
};

#include "../Common/ConfigServiceEngine.h"

// Web config globals
#ifdef WIFI_ENABLED
volatile bool web_cfg_service_enabled = false;
volatile bool web_cfg_pending_save = false;
volatile bool web_cfg_radio_reinit_required = false;
volatile uint32_t web_cfg_req_total = 0;
volatile uint32_t web_cfg_req_ok = 0;
volatile uint32_t web_cfg_req_err = 0;
volatile uint8_t web_cfg_debug_mode = 1; // 0=off, 1=some, 2=full
volatile uint32_t web_cfg_ap_startup_timeout_ms = 60000; // 0 disables timeout
String web_cfg_last_err = "";
#endif
volatile bool config_version_error = false;

// ============================================================
// V2.5-Evo - 2026-04-24 - TX GPS COORDINATES (received via 0xF3 meta-packet)
//
// Written by processMetaGpsPacket() in Radio.ino at 2Hz whenever TX sends
// a GPS meta-packet and RX successfully validates it.
// Read by Phase B anti-spoofing (Priority 6) to check TX-RX proximity.
//
// rx_tx_gps_timestamp == 0 means no meta-packet has ever been received.
// Use (millis() - rx_tx_gps_timestamp) > usrConf.tx_gps_stale_timeout_ms
// to detect a stale TX GPS reading before trusting lat/lng.
// ============================================================
double        rx_tx_gps_lat       = 0.0;  // TX latitude (degrees, WGS84)
double        rx_tx_gps_lng       = 0.0;  // TX longitude (degrees, WGS84)
unsigned long rx_tx_gps_timestamp = 0;    // millis() when last meta-packet received; 0 = never

// V2.5-Evo - 2026-04-25 - P7 RTM/FM runtime state (set by Radio.ino meta-packet handlers)
// rtm_rx_active: true = TX signalled RTM active; safety gates in RTMState.ino may override.
// rtm_rx_emergency_stop: true = safety gate failed; calcPWM() forces throttle to 0.
// rtm_steer_override: bearing-derived steering value (0-255, 127=straight ahead).
// fm_mode_runtime: TX-side FM mode override (0-3); 0xFF = use SPIFFS default.
// V2.5-Evo - 2026-04-25 - P7 fix: use std::atomic for safe access across FreeRTOS task
// preemption. generatePWM (task) and RTMState.ino loop() both run on the single-core
// ESP32-C3; std::atomic gives an indivisible read/write + compiler barrier so a higher-
// priority task can't observe a torn value. (seq_cst, matching the rfInterrupt pattern.)
std::atomic<bool>    rtm_rx_active         {false};
std::atomic<bool>    rtm_rx_emergency_stop {false};
std::atomic<uint8_t> rtm_steer_override    {127};
std::atomic<uint8_t> fm_mode_runtime       {0xFF};

// V2.5-Evo - 2026-07-20 - R2: millis() when the last 0xF2 FM-mode declaration arrived from the
// TX; 0 = none has ever arrived this session. The TX refreshes its declaration every 30 s while
// armed, so RTMState.ino expires the mode after kFmModeAgeMs (95 s, ~3 missed keepalives) and
// returns FM to IDLE. Without this the RX kept a declared mode forever, which meant a lost
// disarm burst left the RX armed for the rest of the session with no way to discover it.
// Written by Radio.ino's meta-packet handler (triggeredReceive task), read by RTMState.ino's
// runFmLoop() (loop) — std::atomic for the same single-core preemption reason as the flags above.
std::atomic<unsigned long> fm_mode_last_rx_ms {0};
std::atomic<uint8_t> rtm_approach_cap      {255};  // V2.5-Evo - 2026-04-30 - approach decel cap (0-255); 255=no cap; computed by RTMState.ino during active RTM; applied by calcPWM()

// V2.5-Evo - 2026-07-19 - P3 Follow-Me (FM) autonomous-following runtime flags.
// fm_rx_active : true while FM is actively steering. Gates the steering override in calcPWM()
//                using the SAME pattern as rtm_rx_active (RTM and FM are mutually exclusive, so
//                they safely share rtm_steer_override as the steering command).
// fm_throttle_cap : FM's own subtract-only throttle cap (0-255; 255 = no cap). Applied in calcPWM()
//                alongside rtm_approach_cap (lowest cap wins). Deliberately a SEPARATE global from
//                rtm_approach_cap so RTM's per-tick housekeeping (which rewrites rtm_approach_cap=255
//                whenever RTM is inactive) can never transiently clear an FM cap in the window between
//                runRtmLoop() and runFmLoop() on the single-core ESP32-C3. seq_cst, same as the RTM
//                atomics — an indivisible read/write the 100Hz generatePWM task cannot tear.
std::atomic<bool>    fm_rx_active     {false};
std::atomic<uint8_t> fm_throttle_cap  {255};

#include "../Common/SPIFFSEngine.h"

// --- Global VESC Logger Struct ---
struct vesc_struct {
  int16_t fetTemp = 0;
  int32_t motCur = 0;
  int32_t batCur = 0;
  int16_t duty = 0;
  int32_t erpm = 0;
  int16_t batVolt = 0;
  int32_t wh_raw = 0;          // session Wh×10 from VESC float32_auto; 0 = unavailable
  uint8_t fault_code = 0;
  unsigned long last_packet = 0;
};
extern vesc_struct vesc;

struct __attribute__((packed)) VescLogData {
    uint32_t timestamp;           // Local Timestamp in ms
    int16_t current_motor;       // Motor Current in 0.01A
    int16_t current_battery;     // Battery Current in 0.01A
    int8_t duty_cycle;           // Duty cycle in %
    uint16_t voltage;             // Voltage in 0.1V
    int16_t ERPM;                // ERPM / 10
    int8_t temp_mos;             // MOSFET temperature in °C
    uint8_t fault_code;           // Error code
    uint16_t speed;               // Speed in 0.1 km/h
    float latitude;               // Latitude in degrees
    float longitude;              // Longitude in degrees
    uint32_t datetime;            // UTC datetime as unix timestamp
    // V2.5-Evo - 2026-05-06 - LOG-EXT-1: heading source debug fields.
    // Populated by convertToLogData() in Logger.ino (LOG-EXT-2).
    // All ×10 fields use 0xFFFF as the "invalid/no data" sentinel.
    // rtm_heading_chosen_dx10 uses int16, -1 sentinel for "no source".
    uint8_t  thr_received_log;          // TX throttle last received (0-255)
    uint8_t  rtm_source;                // 0=NONE, 1=GPS_COG, 2=COMPASS_SNAPSHOT, 3=COMPASS_LIVE (legacy mode)
    uint8_t  rtm_confidence;            // 0=NONE, 1=LOW, 2=MEDIUM, 3=HIGH
    uint8_t  rtm_rx_active_log;         // RTM engagement state (0/1)
    uint8_t  gps_phase_b_ok_log;        // Phase B anti-spoofing handshake state (0/1)
    uint8_t  rtm_steer_override_log;    // Current steering command 0-255 (127 = straight ahead)
    int16_t  rtm_heading_chosen_dx10;   // getRtmHeading() output × 10 deg; -1 if no valid source
    uint16_t compass_live_dx10;         // Live compass heading × 10 deg (0xFFFF = invalid/uncalibrated)
    uint16_t compass_snap_dx10;         // Clean compass snapshot × 10 deg (0xFFFF = no snapshot yet)
    uint16_t snap_age_s;                // Snapshot age in seconds (0xFFFF = no snapshot yet)
    uint16_t gps_course_dx10;           // GPS course-over-ground × 10 deg (0xFFFF = no fix or invalid)
    uint16_t cog_age_ms_div10;          // GPS course age in 10ms units (0xFFFF = no fix yet)
    // V2.5-Evo - 2026-05-08 - Bundle 1: heading controller tuning telemetry fields.
    // 0x7FFF (32767) is the "no data" sentinel — NOT 0x0000.
    int16_t heading_error_dx10;   // Heading error in 0.1° units (signed; -1800..+1800).
                                  // 0x7FFF = no valid heading source. Positive = need turn right.
    int16_t d_error_dx10;         // Rate-of-change of heading error in 0.1°/s units.
                                  // 0x7FFF = no prior sample (first cycle). For tuning Kd.
    // V2.5-Evo - 2026-05-11 - E7 Fix: BREmote remote_error code for cross-correlation with VESC/motor data.
    // 0 = no error, 71 = E71 water ingress (see checkWetness() in System.ino).
    uint8_t error_code_log;       // telemetry.error_code at log time. 0 = no BREmote error.
    // V2.5-Evo - 2026-07-19 - FM triage: steering byte actually applied to the motor mix by
    // calcPWM() (g_effective_steer), NOT just the commanded rtm_steer_override. Reveals the
    // actuation gap — a valid heading error can log a non-127 rtm_steer_override_log while this
    // stays 127 because the throttle-release gate suppressed it. 127 = straight ahead.
    uint8_t effective_steer_log;
    // V2.5-Evo - 2026-07-24 - F9: owner-requested range telemetry — RX→TX distance + LoRa link quality.
    // Appended at the tail so existing CSV column order is preserved (old parsers ignore trailing columns).
    uint16_t tx_distance_dx10;    // RX→TX distance × 10 m (0.1 m resolution, capped ~164 m); 0xFFFF = N/A (no valid GPS pair)
    int16_t  rssi_dbm;            // last control-packet RSSI in dBm (rounded); 0x7FFF = N/A (failsafe — no recent packet)
    int16_t  snr_dx10;            // last control-packet SNR × 10 dB; 0x7FFF = N/A (failsafe — no recent packet)
};
static_assert(sizeof(VescLogData) == 59, "VescLogData size mismatch — check binary log compat.");  // 29 base; +18 LOG-EXT-1 (2026-05-06); +4 Bundle 1 tuning fields (2026-05-08); +1 error_code_log E7 fix (2026-05-11); +1 effective_steer_log FM triage (2026-07-19); +6 F9 distance+RSSI+SNR (2026-07-24)
#define ENABLE_WEB_LOG_DOWNLOAD // Enable log download endpoints

#ifdef WIFI_ENABLED
#include "../Common/WebConfigEngine.h"
#endif

#ifdef WIFI_ENABLED
void webCfgNotifyRxConnected();
#else
inline void webCfgNotifyRxConnected() {}  // No-op stub when WiFi disabled
#endif

// V2.5-Evo - 2026-05-16 - feat(telemetry): expand LoRa packet 8→19 bytes + 0xF4 aux meta-packet
//Telemetry to send, MUST BE 8-bit!!
// V2.5-Evo - 2026-04-27 - P8: rtm_distance at index 5; encoding: 0-99=tenths of m, 100-254=(value-90) whole m, 255=N/A.
struct __attribute__((packed)) TelemetryPacket {
    uint8_t foil_bat = 0xFF;          // index 0 — battery % 0-100
    uint8_t foil_temp = 0xFF;         // index 1 — FET temp degC
    uint8_t foil_speed = 0xFF;        // index 2 — speed km/h
    uint8_t error_code = 0;           // index 3 — fault flags
    uint8_t foil_power = 0xFF;        // index 4 — power (watts/50); 0xFF = N/A
    uint8_t rtm_distance = 0xFF;      // index 5 — RX→TX distance; see encoding above; 0xFF = N/A
    uint8_t foil_motor_amps = 0xFF;   // index 6 — motor current whole amps; 0xFF = N/A
    uint8_t foil_voltage = 0xFF;      // index 7 — battery voltage V×2 (0.5V res); 0xFF = N/A
    uint8_t foil_duty = 0xFF;         // index 8 — duty cycle 0-100%; 0xFF = N/A
    uint8_t foil_erpm_lo = 0xFF;      // index 9 — |ERPM|÷100 low byte; 0xFFFF when both=0xFF means N/A
    uint8_t foil_erpm_hi = 0xFF;      // index 10 — |ERPM|÷100 high byte
    uint8_t foil_wh_lo = 0xFF;        // index 11 — session Wh×10 low byte; 0xFFFF when both=0xFF means N/A
    uint8_t foil_wh_hi = 0xFF;        // index 12 — session Wh×10 high byte
    uint8_t rx_heading = 0xFF;        // index 13 — GPS COG÷2 (0-179→0-358°); 0xFF = N/A
    uint8_t fm_heading_err = 127;     // index 14 — bearing error+127; 127 = no data
    uint8_t fm_status = 0;            // index 15 — [7]=aux2_on [6]=aux1_on [5]=vesc_online [4]=rx_wetness [3:2]=heading_conf [1]=rtm_active [0]=fm_active
    uint8_t fm_flags = 0;             // index 16 — Follow-Me engagement sub-state (assembled in RTMState.ino runRtmLoop): [3]=fault-stop-sticky [2]=armed-not-ready [1]=engaged [0]=armed. Was reserved_tx_imu (unused reserved byte).
    uint8_t rx_bearing_to_tx = 0xFF;  // index 17 — bearing from buggy toward rider÷2; 0xFF = N/A
    uint8_t link_quality = 0;         // index 18 (must be last)
} telemetry;

/*
** FreeROTS/Task handles
*/
const int maxTasks = 10;
TaskStatus_t taskStats[maxTasks];

// Task handles
TaskHandle_t generatePWMHandle = NULL;
TaskHandle_t triggeredReceiveHandle = NULL;
TaskHandle_t checkConnStatusHandle = NULL;
extern TaskHandle_t loopTaskHandle;

// Semaphore for triggered task
SemaphoreHandle_t triggerReceiveSemaphore;

// Mutex protecting Wire/AW9523 — accessed by multiple FreeRTOS tasks that preempt each
// other on the single ESP32-C3 core: generatePWM, checkConnStatus, checkWetness,
// checkButtons, setUartMux, blinkErr, blinkBind, readCompassRaw, loggerLoop, triggerBlink.
// Created in initHardware() before Wire.begin() so startupAW() can safely use it.
SemaphoreHandle_t i2cMutex;

/*
** Variables
*/
std::atomic<bool> rfInterrupt{false};
volatile bool rxIsrState = 0;
volatile int unpairedBlink = 0;
volatile unsigned long last_packet = 0;
volatile uint8_t telemetry_index = 0;

volatile uint8_t payload_buffer[10];
volatile uint8_t payload_received = 0;

const unsigned long PAIRING_TIMEOUT = 10000;
const uint8_t MAX_ADDRESS_CONFLICTS = 5;

rmt_channel_handle_t tx_channel = NULL;
rmt_encoder_handle_t copy_encoder = NULL;
rmt_symbol_word_t pulse_symbol;

volatile int alternatePWMChannel = 0;
volatile bool PWM_active = 0;
volatile uint16_t PWM0_time = 0;
volatile uint16_t PWM1_time = 0;

volatile uint8_t thr_received = 0;
volatile uint8_t steering_received = 127;

// V2.5-Evo - 2026-07-19 - FM triage: the steering byte calcPWM() actually applied to the motor
// mix this loop (rtm_steer_override while RTM active + override enabled + thr>=25, otherwise the
// user's steering_received). Written by calcPWM() (generatePWM task, 100Hz) and read by the logger
// (loggerTask). Single-byte volatile — atomic on ESP32-C3, same pattern as thr_received. Logged so
// the actuation gap is visible: rtm_steer_override can command a turn while this stays neutral
// because the throttle-release gate suppressed it. 127 = straight ahead.
volatile uint8_t g_effective_steer = 127;

volatile unsigned long get_vesc_timer = 0;
volatile unsigned long last_uart_packet = 0;

volatile uint8_t bind_pin_state = 0;
volatile uint8_t rx_aux_flags = 0;   // set by 0xF4 meta-packet: bit0=strobe, bit3=find-me

float fbatVolt = 0.0;
float noload_offset = 0.0;
uint8_t bc_arr[101];
uint8_t percent_last_val = 0xFF;
uint8_t percent_last_thr = 1;
unsigned long percent_last_thr_change = 0;

// V2.5-Evo: ERPM added to VESC selective-get mask; payload length is 23 bytes.
// P7: ERPM is also read by Phase C RTM anti-spoofing (RTMState.ino) to verify
// VESC speed matches GPS speed during active RTM. gps_en + vesc_erpm_per_kmh>0 required.
#define VESC_MORE_VALUES
#ifdef VESC_MORE_VALUES
  #define VESC_PACK_LEN 27  // +4 bytes for watt_hours (float32_auto)
  uint8_t vescRelayBuffer[34];
#else
  #define VESC_PACK_LEN 9
  uint8_t vescRelayBuffer[15];
#endif

//SPI Pins
#define P_SPI_MISO 6
#define P_SPI_MOSI 7
#define P_SPI_SCK 10
//LORA Pins
#define P_LORA_DIO 3
#define P_LORA_BUSY 4
#define P_LORA_RST 5
#define P_LORA_NSS 8
//Misc Pins
#define P_PWM_OUT 9
#define P_U1_TX 18
#define P_U1_RX 19
#define P_UBAT_MEAS 0
#define P_I2C_SCL 1
#define P_I2C_SDA 2

//AW9523 Pins
#define AP_U1_MUX_0 8
#define AP_U1_MUX_1 9
#define AP_S_BIND 0
#define AP_S_AUX 10
#define AP_L_BIND 1
#define AP_L_AUX 11
#define AP_EN_BMS_MEAS 4
#define AP_BMS_MEAS 7
#define AP_EN_PWM0 13
#define AP_EN_PWM1 12
#define AP_EN_WET_MEAS 14
#define AP_WET_MEAS 15

//Debug options — comment out for release builds
//#define DEBUG_RX
//#define DEBUG_VESC

#if defined DEBUG_RX
   #define rxprint(x)    Serial.print(x)
   #define rxprintln(x)  Serial.println(x)
#else
   #define rxprint(x)
   #define rxprintln(x)
#endif

#ifdef DEBUG_VESC
#define VESC_DEBUG_PRINT(x) Serial.print(x)
#define VESC_DEBUG_PRINTLN(x) Serial.println(x)
#else
#define VESC_DEBUG_PRINT(x)
#define VESC_DEBUG_PRINTLN(x)
#endif

#include "../Common/RadioCommon.h"
#include "../Common/SystemCommon.h"