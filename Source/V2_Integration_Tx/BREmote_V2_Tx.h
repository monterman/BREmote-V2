// V2.5-Evo - 2026-10-07 - F4/F5 on the remote (port of P2-d): followme_mode documents 1-5 (4 front right, 5 front left;
//   no 6) and mag_fm_set gains bit 3 = station 4 and bit 4 = station 5 (range 1-31). Default and repair value stay 7,
//   the three rear stations. Ranges and comments only: same fields at the same offsets, sizeof stays 136, SW_VERSION
//   stays 27, no config reset.
// V2.5-Evo - 2026-10-07 - SOP-041 rule 3 resync: RAM fm_flags_unarmed_since_ms / fm_flags_unarmed_streak. No struct change.
// V2.5-Evo - 2026-10-07 - TX protocol round: TelemetryPacket gains index 19 rx_state_flags (20 bytes, appended; an old RX
//   never sends it and 0 means nothing to report), RX_STATE_* bits, FM_FLAG_RETURN_STANDING (fm_flags bit 6), and the
//   RAM stamps rx_rtm_fault_rise_ms / rx_rtm_arrived_rise_ms / rtm_start_sent_ms. No confStruct change: sizeof stays 136,
//   SW_VERSION stays 27.
// V2.5-Evo - 2026-10-07 - A-1 (TX part): fm_status_rtm_on_ms / fm_status_rtm_off_streak (the buggy's RTM bit per arrival).
//   RAM only: sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-10-07 - SOP-040 gesture rule: triggerReleased() / TRIGGER_RELEASED_MAX (thr_scaled < 10). No struct change.
// V2.5-Evo - 2026-10-07 - P-11: RAM flag ads_cal_in_progress (plausibility exempt only while checkCal() runs). No
//   confStruct change: sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-10-07 - P-1: comment only - the radio task no longer writes ads_input_fault.
// V2.5-Evo - 2026-10-07 - fm_display_mode DEFAULT 1 -> 2 (distance to the buggy in metres; owner ruling). defaultConf
//   value only - no struct change, sizeof stays 136, SW_VERSION stays 27; remotes keep their stored value.
// V2.5-Evo - 2026-10-07 - H-1 (TX part): fm_status_arrival_ms, rtm_stop_sent_ms, FM_STATUS_RTM_ACTIVE, and a note that
//   the meta-packet atomics are now the head of a 2-deep queue. RAM only: sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-10-07 - C-1: throttle-input health globals (last_ads_ok_ms, ads_input_fault, ads_thr_out_of_range,
//   ads_thr_good_samples), ADS_STALE_MS and the TX-local error code REMOTE_ERR_INPUT_FAULT (72). RAM only, no confStruct
//   change: sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-10-07 - R-6: new RAM flag fm_fault_latched (set by the telemetry unpack on the Follow-Me fault-stop
//   rising edge, cleared by runFmLoop()). A global, not a confStruct field: sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-10-07 - comments only: fm_warn_distance_m no longer drives a vibration (the Pattern 8 FM warning
//   haptic was removed, owner ruling); it remains the R5 proximity-bar full-scale. The FollowMeDistanceWarning.h include
//   stays for kFmDistanceTelemetryMaxM. No struct change: sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-09-30 - MagFix (delta audit of 98fb7a8) — SEVEN FOLLOW-UPS, NO STRUCT CHANGE. sizeof
//   (confStruct) STAYS 136 and SW_VERSION STAYS 27: the tail is full, and a bump would wipe the owner's
//   throttle calibration, so the one piece of new state (the Return-To-Me session override) is a RAM
//   variable, not a field. (1) A sample-gap guard in runMagGesture(): a loop() stall can no longer promote a
//   tap into the 2.5 s hold. (2) The Return-To-Me magnet toggle is now genuinely RAM only — it writes
//   rtm_enabled_session, not usrConf, so `?save` cannot make a session flip permanent. (3) The tap ceiling is
//   600 ms, because the real magnet sample interval is ~110 ms and never was the 20 ms the old comments
//   claimed. (4) The RTM-OFF confirm is Pattern 12 (three firm taps), not the Pattern 7 fault buzz. (5) The
//   station flash is a clamped 1.2 s, not gear_display_time. (6) The tow gate (fmIsEngaged()) needs
//   FM_FLAG_ARMED as well as FM_FLAG_ENGAGED, on 2 consecutive telemetry arrivals — new global
//   fm_engaged_streak. (7) The mode table below records that mode 4 has NO MAGNET DISARM, and the
//   stray-magnet claim is corrected: a stray tap CAN arm Follow-Me (the owner asked for that) but cannot
//   move a station and cannot produce motion; no_lock = 0 is the real mitigation.
// V2.5-Evo - 2026-09-30 - MagStations: new mag_mode 4 (MAG_ROLE_FMSET) — a magnet TAP (60-600 ms) steps
//   through the Follow-Me stations selected in the new mag_fm_set bitmask, and a 2.5 s hold toggles
//   rtm_enabled for the session. mag_fm_set OCCUPIES THE 2 TAIL PADDING BYTES mag_mode left behind, so
//   sizeof(confStruct) stays 136 and SW_VERSION stays 27: no SPIFFS reset, no lost throttle calibration.
//   Station changes are HARD-GATED to "Follow-Me actively following" — never while the rider is on the
//   rope under tow, because a buggy that repositions itself while the rider is attached and cannot steer
//   away is unacceptable. Stations 4 and 5 (front) do not exist in this firmware and are NOT added here.
//   ⚠ GPIO 9 (P_MAG) IS AN ESP32-C3 STRAPPING PIN. NEVER POWER THE REMOTE ON WITH THE MAGNET ATTACHED:
//   the chip samples GPIO 9 at reset and a magnet held there puts it into UART download mode, so the
//   remote will not boot. mag_seen_high cannot prevent this — strapping happens before any firmware runs.
// V2.5-Evo - 2026-09-19 - Stick during auto-steer: FM_FLAG_STEER_TAKEOVER (fm_flags bit 4) is the buggy's echo of its
//   steer_during_auto setting (1 = the stick takes over an automatic steering run instead of cancelling it) and
//   FM_FLAG_STEER_ACTIVE (bit 5) says a takeover is standing right now (display only). Gate 4 in RTMState.ino reads bit 4
//   under the link-fresh window; rtm_steer_exit_on_input is no longer read by the remote (kept so stored settings load).
//   Defines + comments only: TX struct untouched (136, SW27), no SPIFFS reset.
// V2.5-Evo - 2026-09-19 - Return gesture: FM_FLAG_RETURN_ON (fm_flags bit 7) is the RX's echo of its EFFECTIVE auto-return mode
//   (1 = when the rider stops, Follow-Me brings the buggy back). The return gesture in RTMState.ino reads it to set the
//   session override to the opposite value. Define only: no confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-09-17 - FM warning-distance haptic: includes ../Common/FollowMeDistanceWarning.h (pure header shared with
//   the host unit test). fm_warn_distance_m is now LIVE (Pattern 8) with a 164 m ceiling — the one-byte rtm_distance
//   telemetry saturates there. No confStruct change: sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-09-17 - fm_arm_window_s RENAMED IN PLACE to fm_arm_timeout_s (owner decision 2026-09-15): same slot,
//   same uint16_t, sizeof(confStruct) stays 136, SW_VERSION stays 27, no SPIFFS reset. Meaning: seconds armed with no
//   throttle before auto-disarm; 0 = never (new default, was 180). A remote that already stores 180 keeps 180 until
//   `?set fm_arm_timeout_s 0` + `?save` — the stored value keeps its meaning in seconds.
// V2.5-Evo - 2026-08-17 - defaultConf.rtm_double_squeeze_en 0 → 1: the factory default RTM arm gesture is now the
//   deliberate double squeeze, which is what the struct comment always documented. Default value + comments only —
//   confStruct UNCHANGED, sizeof stays 136, SW_VERSION stays 27, no SPIFFS reset, and units with a stored value keep it.
// V2.5-Evo - 2026-07-20 - BLE re-enable deep-fix (audit 2026-07-20-bremote-fw-audit-tx-ble-reenable-rootcause):
//   BLE_ENABLED turned back ON permanently with the single-core coexistence hardening applied —
//   heap-floor guard on init, relaxed connection interval, consolidated/back-pressured notify stream
//   moved off loop() into a dedicated task, an i2cMutex serializing the shared HT16K33+ADS1115 Wire bus,
//   and a WiFi-vs-BLE mutual-exclusion gate. Runtime/task/init changes only — confStruct UNCHANGED,
//   sizeof stays 136, SW_VERSION stays 27, no SPIFFS reset. Re-commenting #define BLE_ENABLED (below) is
//   still the instant, proven rollback.
// V2.5-Evo - 2026-07-20 - MagGesture: SW_VERSION 26 → 27; sizeof(confStruct) 132 → 136 (2 bytes are alignment tail padding).
//   New TX field mag_mode (uint16_t, 0-3, default 0 = off / Hall sensor not fitted) selects what the
//   magnet gesture arms: 1=FM (2s), 2=RTM (2s), 3=FM (2s) + RTM (5s). magGestureRole() + MAG_ROLE_*
//   defines added. bt_enabled is UNCHANGED (still 0-2) — BLE and the optional Hall sensor are
//   separate concerns and get separate fields.
//   ⚠ SW_VERSION bump RESETS the TX SPIFFS config to defaultConf on first flash.
// V2.5-Evo - 2026-07-20 - SW27 defaults bake: defaultConf carries the factory default
//   configuration — pairing unbound, calibration nominal, GPS/speed source and RTM/FM tuning
//   at generic defaults. The FM arm window (now fm_arm_timeout_s) was baked at 180 (see the
//   rationale comment at the field). No struct change: sizeof(confStruct) stays 136 and SW_VERSION stays 27.
//
// ============================================================
// V2.5-Evo - 2026-07-20 - defaultConf.mag_mode ships at 0 (magnet gesture off). This is the
// safe general default: not every remote has the DRV5032 Hall sensor + magnet fitted, and
// P_MAG (GPIO 9) reads an UNDEFINED state without it, which would let the magnet gesture
// mis-fire on hardware that was never meant to use it. Enable 1/2/3 per device via the web UI.
// ============================================================
// V2.5-Evo - 2026-07-20 - Batch T (FM v1.4): TelemetryPacket index 16 reserved_tx_imu repurposed as fm_flags
//   ([0]armed [1]engaged [2]armed-not-ready [3]fault-stop-sticky) + FM_FLAG_* / FM_LINK_HEALTHY_MS defines.
//   No struct size change (byte already present), no confStruct change — sizeof(confStruct) stays 136,
//   SW_VERSION stays 27, static_assert intact, SPIFFS config NOT reset by this flash.
// V2.5-Evo - 2026-05-13 - SW50: DISPLAY_MODE_AMP replaces INTBAT; TelemetryPacket +foil_motor_amps byte (index 6); link_quality→index 7
// V2.5-Evo - 2026-05-13 - SW48: DISP_LOCK/UNLOCK macros; mutex all bare display callers outside renderOperationalDisplay/updateBargraphs
// V2.5-Evo - 2026-05-13 - SW46: DISPLAY_MODE order — Temp(0)/Thr(1)/Speed(2)/Power(3)/Bat(4)/IntBat(5); THR centre, LEFT=Temp, RIGHT=Speed
// V2.5-Evo - 2026-05-13 - SW33: GPIO 9 repurposed as P_MAG digital Hall sensor (DRV5032FADBZR); removed from serialOff OUTPUT-LOW block; mag_seen_high boot guard added
// V2.5-Evo - 2026-05-13 - SW33b: BT dot test (C7 R1) driven by P_MAG Hall sensor; bt_dot_state + BT_DOT_* defines added
// V2.5-Evo - 2026-04-21 - Added TinyGPS++ include, gps_tx + tx_gps_speed globals, and P_U1_RX/P_U1_TX pin defines for TX GPS (BN-220 on Serial1)
// V2.5-Evo - 2026-04-22 - Fixed speed_src/volatile comments; defaults gps_en=0,speed_src=0; added gps_max_hdop field (HDOP*100, tail-padding slot, sizeof stays 92)
// V2.5-Evo - 2026-04-22 - Added gps_chip_type field (GPS module selector: 0=BN-220, 2=M10); sizeof 92→96
// V2.5-Evo - 2026-04-25 - P7: Added RTM meta-packet queue globals (rtm_meta_type/value/count) and RTM throttle cap (rtm_thr_cap_tx, rtm_tx_active)
// V2.5-Evo - 2026-04-27 - P8: Added rtm_display_mode, fm_warn_distance_m, rtm_steer_exit_on_input to confStruct; TelemetryPacket adds rtm_distance at index 5; rtm_max_runtime_s default 120→0
// V2.5-Evo - 2026-04-27 - P8.1: Added the FM arm window (now fm_arm_timeout_s) to confStruct; FM redesigned as arm/disarm toggle with mode memory; sizeof 124→128
// V2.5-Evo - 2026-04-28 - P9: Added dist_unit (fills 2-byte tail padding; sizeof stays 128); rtm_arm_dist_m RAM global
// V2.5-Evo - 2026-04-29 - Sleep: added sleep_timeout_s to confStruct; SW_VERSION 25→26
// V2.5-Evo - 2026-05-01 - Release: DEBUG_RX commented out for production build
// V2.5-Evo - 2026-05-01 - thr_expo1 repurposed as fm_display_mode (FM digit zone data selector, 1-4)
// V2.5-Evo - 2026-08-15 - Comments in this firmware no longer say "Core 0" / "Core 1". The ESP32-C3 is SINGLE-CORE:
//                          every xTaskCreatePinnedToCore() call here passes core 0, and there is no core 1 to pin to.
//                          The races these mutexes and atomics guard are TASK PREEMPTION on one core, not parallel
//                          execution on two — same hazard, different mechanism, and the old wording taught the wrong
//                          model. Actors are now named by task ("the loop task", "the sendData task") instead. The RX
//                          was corrected this way on 2026-05-12; the TX kept the stale wording until now.
// V2.5-Evo - 2026-05-02 - Added displayMutex SemaphoreHandle_t (displayBuffer race between the loop task and the bargraph task)
// V2.5-Evo - 2026-05-13 - SW32 M3: rtm_meta_type/value/count + rtm_thr_cap_tx + rtm_tx_active changed volatile→std::atomic<T>; release/acquire ordering in queue/consumer
// V2.5-Evo - 2026-05-13 - SW32: default display_mode changed 0→DISPLAY_MODE_THR (throttle % as boot display; field test feedback)
// V2.5-Evo - 2026-05-09 - Bundle 9-Final: Added USB CDC On Boot compile-time guard
// V2.5-Evo - 2026-07-20 - T2: FM arm window (now fm_arm_timeout_s) comment corrected 10-60s → 10-600s (validation range was already 10-600; comment was stale). No struct change.

// ============================================================
// V2.5-Evo - 2026-05-09 - Bundle 9-Final: USB CDC On Boot guard
//
// ESP32-C3 chip-level hardware default: GPIO 18 = USB D-, GPIO 19 = USB D+.
// This firmware uses those pins as UART for the BN-220 GPS via Serial1.
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
#error "TX firmware requires USB CDC On Boot = Disabled. ESP32-C3 USB peripheral claims GPIO 18/19 (used by Serial1 for GPS) when CDC On Boot is enabled. Set Tools -> USB CDC On Boot -> Disabled in Arduino IDE, OR pass :CDCOnBoot=default to arduino-cli's --fqbn argument. See file header for full explanation."
#endif

/*
** Includes
*/
#include <Arduino.h>
#include <atomic>
#include "../Common/FollowMeDistanceWarning.h"   // V2.5-Evo - 2026-09-17 - pure header, host-testable. Since 2026-10-07 the TX uses only kFmDistanceTelemetryMaxM from it (ConfigService.ino); the haptic it fed is removed
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <RadioLib.h> //V7.1.2 jan gromes
#include <Wire.h>
#include <Adafruit_ADS1X15.h> //V2.5.0 adafruit
#include <Ticker.h>
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"  // V2.5-Evo - 2026-07-20 - heap_caps_get_free_size() for the BLE heap-floor guard (audit §4.1)
#include "FS.h"
#include "SPIFFS.h"
#include "mbedtls/base64.h"
#include <Preferences.h>   // V2.5-Evo - 2026-10-07 - S-8: the last boot ID lives in NVS (not confStruct), see txBootIdInit()

// --- V2.5-Evo: TX GPS support (BN-220 on Serial1) ---
// Added for Priority 1: read TX GPS speed and drive the SP display mode
// when usrConf.speed_src selects a TX-GPS option (2=km/h, 3=knots, 5=mph).
// Library: TinyGPSPlus 1.0.3 by Mikal Hart (same version used on RX).
#include <TinyGPS++.h>

// V2.5-Evo - 2026-06-04 - BLE master kill-switch. Leave BLE_ENABLED UNDEFINED to
// fully exclude the NimBLE stack: no header, no init, no task, no loop calls.
// This was added because the BLE work crashed the TX display during water testing.
// The Hall-sensor BT status dot (bt_dot_state / BT_DOT_*) and the boot gesture flag
// (bt_session_forced) are intentionally left OUTSIDE this guard — they are independent of
// the NimBLE stack and stay compiled so the rest of the firmware is unchanged.
// V2.5-Evo - 2026-07-20 - RE-ENABLED PERMANENTLY. Root-caused (single-core CPU starvation +
// no-PSRAM heap collapse + un-back-pressured triple notify stream) and hardened per the audit report
// 2026-07-20-bremote-fw-audit-tx-ble-reenable-rootcause: heap-floor guard, relaxed conn interval,
// one back-pressured notify stream in its own Core-0 task, i2cMutex on the shared Wire bus, and a
// WiFi/BLE mutual-exclusion gate. ROLLBACK IS STILL INSTANT: re-comment the #define below to fully
// exclude the NimBLE stack again exactly as during the 2026-06-04 → 2026-07-20 water-test kill.
#define BLE_ENABLED

// NimBLE-Arduino: required for ExtTelem BLE GATT service (BLE_Ext.ino).
// NimBLEServer type must be visible here so the forward declaration in
// V2_Integration_Tx.ino can compile before BLE_Ext.ino is concatenated.
#ifdef BLE_ENABLED
#include <NimBLEDevice.h>

// V2.5-Evo - 2026-07-20 - BLE re-enable tuning constants (audit §4.1 / §4.3).
// -- Heap-floor guard (audit §4.1, mirrors foilIQ F-2) --
// bleInitTask reads free INTERNAL DRAM before NimBLEDevice::init(); if it is below this floor the
// whole BLE stack is skipped gracefully (no boot-loop) so the no-PSRAM C3 never inits into a NULL
// alloc. The audit's guidance was ~60-70 KB, tune on the bench; the conservative (higher) end is chosen.
// Same-family data point: the foilIQ S3 measured 81776 free before init → 11688 after → init FAILED,
// so a comfortable pre-init margin is essential. Bench-tune against the real WiFi-off riding heap.
// V2.5-Evo - 2026-07-20 - audit M2 (re-audit): this value is BENCH-TUNABLE and currently UNPROVEN on the
// C3. The foilIQ S3 above consumed ≈70 KB during NimBLE init and STILL failed — so 70 KB *free* is only
// a floor to attempt init, not a guarantee of a successful/stable init. Do NOT bump this blindly: a
// higher floor could block BLE from ever starting if the C3's true free-heap-at-init is modest. The real
// number is unknown until bench — read it off the prominent pre/post-init Serial.printf heap logs in
// bleInitTask (Init.ino) and initBLE() (BLE.ino) on the first bench run, THEN set:
//   BLE_HEAP_FLOOR_BYTES = measured init consumption + runtime notify headroom + safety margin.
#define BLE_HEAP_FLOOR_BYTES 71680u   // 70 KB internal-DRAM floor to even attempt NimBLE init (BENCH-TUNABLE, audit M2)
// -- Runtime heap floor (audit M2 re-audit) — the standing net the init floor cannot provide --
// BLE_HEAP_FLOOR_BYTES only guards NimBLEDevice::init() ONCE, at boot. It does nothing against a runtime
// H2 heap collapse under a live connection. This runtime floor is checked in bleServiceNotify() (the
// Core-0 notify path) before every push: if free INTERNAL DRAM drops below BLE_HEAP_RUNTIME_FLOOR_BYTES
// the periodic telemetry notifies are SUSPENDED (push skipped) while the connection + NimBLE stack stay
// fully up; they RESUME only once free heap climbs back above floor + BLE_HEAP_RUNTIME_HYSTERESIS_BYTES.
// Suspending pushes is always fail-safe — it only reduces BLE egress and NEVER touches the motor /
// throttle / steer path. The hysteresis prevents flapping at the threshold; state changes are logged
// once per transition. BENCH-TUNABLE (audit M2): once NimBLE is up the runtime free heap is expected to
// sit well above the init peak, so this floor is a collapse tripwire set well below BLE_HEAP_FLOOR_BYTES,
// not a normal operating point. Confirm/adjust from the ?printtasks + heap census under a ≥30-min live
// connection (LoRa 10 Hz + GPS).
#define BLE_HEAP_RUNTIME_FLOOR_BYTES      20480u   // 20 KB — suspend telemetry notifies below this (BENCH-TUNABLE)
#define BLE_HEAP_RUNTIME_HYSTERESIS_BYTES  8192u   // 8 KB recovery margin above the floor before resuming
// -- Relaxed connection interval (audit §4.3 — highest-leverage single-core mitigation) --
// Requested on connect via NimBLEServer::updateConnParams(). Longer interval = fewer controller
// wakeups = less core stolen from the display render + LoRa sendData path on the one C3 core.
// Units: interval steps are 1.25 ms, supervision timeout steps are 10 ms.
#define BLE_CONN_MIN_INTERVAL 32u     // 32 * 1.25 ms = 40 ms
#define BLE_CONN_MAX_INTERVAL 64u     // 64 * 1.25 ms = 80 ms
#define BLE_CONN_LATENCY      0u      // no slave latency — keep telemetry timely
#define BLE_CONN_TIMEOUT      200u    // 200 * 10 ms = 2000 ms supervision timeout
// -- Consolidated notify cadence (audit §4.4 / §4.5) --
// One telemetry stream is pushed every BLE_TELEM_INTERVAL_MS (was: ext-telem 200 ms + CSV 500 ms running
// concurrently). The dedicated notify task wakes every BLE_NOTIFY_TICK_MS; the finer tick lets the
// backpressure hold-off react promptly when the stack reports congestion.
#define BLE_TELEM_INTERVAL_MS 250u
#define BLE_NOTIFY_TICK_MS    50u
#endif

// Uncomment the line below to enable WiFi AP configuration mode
#define WIFI_ENABLED

#ifdef WIFI_ENABLED
#include <WiFi.h>
#include <WebServer.h>
#endif

#define SW_VERSION 27  // V2.5-Evo - 2026-07-20: mag_mode added to confStruct (magnet/Hall gesture role);
                       // sizeof 132→136. First flash of SW27 RESETS all TX SPIFFS settings to defaultConf.
                       // defaultConf carries the factory default configuration, so the reset writes
                       // known-safe values — see the defaultConf header block before changing any default.
                       // V2.5-Evo - 2026-04-29: sleep_timeout_s added to confStruct; first
                       // flash resets all TX SPIFFS settings to defaults — re-configure via WebUI
const char* CONF_FILE_PATH = "/data.txt";

//#define DELETE_SPIFFS_CONF_AT_STARTUP 1

// V2.5-Evo - 2026-04-28 - P9: Compact 3×7 font entry used by showFullScreenMessage() in Display.ino.
// Defined here so Arduino IDE's auto-prototype generator sees it before emitting the fc3x7GetChar prototype.
struct Fc3x7Entry { uint8_t col[3]; };

/*
** Structs
*/
// NOTE: Not packed — sizeof is 96 (V2.5-Evo, was 92 before gps_chip_type, 80 in V2). Float forces 4-byte struct alignment.
// Do not add __attribute__((packed)), it would break existing SPIFFS configs and the web config tool.
struct confStruct {
    //Version
    uint16_t version;

    uint16_t radio_preset; //1: 868MHz (EU), 2: 915MHz (US/AU)
    int16_t rf_power; //Tx power from -9 to 22

    //Calibration of Tog&Thr
    uint16_t cal_ok;
    uint16_t cal_offset;

    uint16_t thr_idle;
    uint16_t thr_pull;

    uint16_t tog_left;
    uint16_t tog_mid;
    uint16_t tog_right;

    //UI Threshold & Times
    uint16_t tog_deadzone; //Deadzone in the middle of toggle 500
    uint16_t tog_diff;  //Difference in toggle signal to register a UI input 30 
    uint16_t tog_block_time; //How long toogle button is in steering (*10ms)
    uint16_t trig_unlock_timeout; //Time after unlock until trigger times out (ms) 5000
    uint16_t lock_waittime; //Time toggle needs to be pressed to power off or lock system (ms) 2000
    uint16_t gear_change_waittime; //Time toggle needs to be pressed to change gear (ms) 100
    uint16_t gear_display_time; //How long the new gear is shown (ms) 1000
    uint16_t menu_timeout; //How long after last menu use until steering is reengaged (0 to disable) 10
    uint16_t err_delete_time; //How long the "E-" is shown after deleting an error. In this time, the user can also change gear, even if the error is still persistent (and therefore will be shown again after this time is over) 2000

    //UI Features
    uint16_t no_lock; //No locking function, as soon as remote is on, throttle is active
    uint16_t throttle_mode; // 0=gears, 1=no gears, 2=dynamic cap
    uint16_t max_gears; //Max user gears
    uint16_t startgear; //The gear that is set after poweron or unlock (0 to 9)
    uint16_t steer_enabled; //If steering feature is enabled
    
    uint16_t thr_expo; //Exponential function, 50 = linear
    uint16_t fm_display_mode;  // FM digit zone display: 1=TX speed, 2=distance to buggy (default since 2026-10-07),
                               // 3=buggy speed (RX telemetry), 4=throttle %; range 1-4

    uint16_t steer_expo; //currently unused

    // V2.5-Evo - 2026-08-18 - RENAMED IN PLACE: steer_expo1 -> gps_dyn_model. Same offset, same
    // uint16_t, so sizeof(confStruct) STAYS 136 and SW_VERSION STAYS 27 — this flash does NOT
    // reset the TX SPIFFS config, and testers keep throttle calibration and pairing.
    //
    // steer_expo1 was chosen over steer_expo specifically because its default is 0, and 0 means
    // "use the default, which is Sea" — exactly the behaviour every TX already has hard-coded.
    // So every remote in the field reads 0, resolves to Sea, and changes nothing. steer_expo
    // defaults to 50, which would have needed a clamp to avoid a nonsense dynModel on first boot.
    //
    // Reported by beta tester heiguga 2026-08-18: `?set gps_dyn_model 4` returned
    // ERR_UNKNOWN_KEY on the TX. The RX gained this setting in SW35 and the TX did not, but the
    // TX has its own GPS and the same hard-coded Sea model with the same 500 m altitude ceiling.
    uint16_t gps_dyn_model;            // 0 = default (Sea) | 4 = Automotive | 5 = Sea

    //System parameters
    float ubat_cal; //ADC to volt cal for bat meas, default 0.000185662

    // GPS features related flags
    uint16_t gps_en;           // GPS runtime enable flag (0=disabled, 1=enabled)
    // V2.5-Evo - 2026-10-07 - F4/F5: the station set is 1-5 and continuous round the rider. There is no 6:
    // a station directly ahead puts the buggy on the rider's line, where a failed motor stops it in his path.
    // Range only - same uint16_t at the same offset, so sizeof(confStruct) stays 136 and SW_VERSION stays 27.
    uint16_t followme_mode; // Follow-me starting station (0=disabled, 1=rear_right, 2=behind, 3=rear_left, 4=front_right, 5=front_left)
    uint16_t kalman_en;        // Kalman filter runtime enable flag (0=disabled, 1=enabled)
    uint16_t speed_src;   // 0=RX km/h, 1=RX knots, 2=TX km/h, 3=TX knots, 4=RX mph, 5=TX mph
    
    //Follow-me timeouts (transmitted to RX via META)
    uint16_t tx_gps_stale_timeout_ms; // TX GPS data stale timeout (ms)

    //Comms
    uint16_t paired;
    uint8_t own_address[3];
    uint8_t dest_address[3];
    // wifi_password is 8 chars with NO null terminator. The field is deliberately
    // undersized — a 9th byte would shift dynamic_power_start and break all existing
    // SPIFFS configs (sizeof 128 → 130 after compiler alignment padding).
    // WebConfigEngine.h softAP() call copies into a local char ap_pass[9] buffer
    // and appends '\0' before passing to WiFi.softAP() — see Common/WebConfigEngine.h.
    char wifi_password[8];      // WPA2 AP password, exactly 8 chars — null-terminated at call site only
    uint16_t dynamic_power_start;  // 10-100, starting cap for mode 2 (default 85)
    uint16_t dynamic_power_step; // 1-25, step size per toggle press in mode 2 (default 5)
    // V2.5-Evo - 2026-04-22 - HDOP quality gate for TX GPS. Stored as HDOP*100 to keep the struct
    // as uint16 throughout (e.g. 200 = HDOP 2.0). Placed at the end to reuse the 2 bytes of
    // tail padding that the float member forces; sizeof was 92 after this field.
    uint16_t gps_max_hdop;       // TX GPS HDOP threshold *100 (50-500 = HDOP 0.5-5.0; default 200 = HDOP 2.0)

    // V2.5-Evo - 2026-04-22 - GPS chip type selector. Determines which baud/rate/constellation
    // init sequence is used by initTxGPS(). TX hardware has no compass, so types 1 and 3
    // are rejected by cfgValidateCrossField(). Adding this field grows sizeof 92→96 (2 bytes
    // data + 2 bytes new tail padding). Old 92-byte SPIFFS configs fail the decodedLen check
    // and trigger a clean write of defaultConf — safe behavior.
    uint16_t gps_chip_type;      // 0=BN-220 (default, 9600→115200, 5Hz), 2=M10 (115200, 10Hz, all constellations); TX valid: 0 and 2 only

    // ============================================================
    // V2.5-Evo - 2026-04-25 - PRIORITY 7: RTM AND FM MODE PARAMETERS
    //
    // 12 new uint16_t fields — sizeof grows 96→120.
    // First flash of P7 firmware resets all TX settings to defaults.
    // After flashing: re-pair TX/RX, re-enter all settings via web UI.
    // ============================================================
    uint16_t rtm_enabled;              // RTM master enable; 0=off, 1=on; default 1
    uint16_t rtm_hold_duration_s;      // LEFT hold time to arm RTM; 3-10 s (floor lowered 4→3 2026-07-20); default 5
    uint16_t rtm_arm_window_s;         // Window to engage throttle after arming; 5-30 s; default 10
    uint16_t rtm_double_squeeze_en;    // RTM arm gesture; 1=double squeeze, 0=single squeeze held ~500 ms; default 1
    uint16_t rtm_throttle_start_pct;   // Initial throttle cap when RTM engages; 10-50 %; default 30
    uint16_t rtm_throttle_max_pct;     // Max throttle cap after ramp; 30-90 %; default 70
    uint16_t rtm_ramp_duration_s;      // Time to ramp throttle start→max; 2-15 s; default 5
    uint16_t rtm_disengage_distance_m; // Distance from TX at which RTM disengages (hard stop); 3-20 m; default 10
    uint16_t rtm_max_runtime_s;        // Maximum continuous RTM runtime; 30-300 s; default 120
    uint16_t rtm_gps_timeout_ms;       // TX GPS loss timeout before safety stop; 500-3000 ms; default 2000
    uint16_t fm_hold_duration_s;       // RIGHT hold time for FM mode cycle; 3-10 s (floor lowered 4→3 2026-07-20); default 5
    uint16_t fm_override_enabled;      // Allow TX to override RX follow-me mode; 0=off, 1=on; default 1

    // ============================================================
    // V2.5-Evo - 2026-04-27 - PRIORITY 8: DISPLAY, GESTURE & UX OVERHAUL
    //
    // 3 new uint16_t fields — sizeof grows 120→124 (118 data + 6 = 124; 124 % 4 == 0, no tail padding).
    // First flash of P8 firmware resets all TX settings to defaults.
    // ============================================================
    uint16_t rtm_display_mode;         // RTM/FM active info display: 0=distance(default), 1=speed, 2=alternating 2.5s each
    uint16_t fm_warn_distance_m;       // TX-RX distance used as the R5 proximity-bar full-scale while Follow-Me is engaged; 50-164 m; default 150.
                                       // V2.5-Evo - 2026-10-07: no longer drives a vibration - the Pattern 8 warning haptic was removed.
                                       // V2.5-Evo - 2026-09-17: ceiling 1000 → 164 = kFmDistanceTelemetryMaxM, the largest value
                                       // the one-byte rtm_distance telemetry can carry; a stored value above it is clamped on load.
    // V2.5-Evo - 2026-09-19 - DEPRECATED, no longer read by the remote. Whether the stick cancels
    // or takes over an automatic return is the BUGGY's steer_during_auto setting, echoed to the
    // remote in telemetry.fm_flags bit 4; Gate 4 (RTMState.ino) follows the buggy. Kept in the
    // struct so stored settings load unchanged (a rename-in-place to a reserved slot is wrong here:
    // fielded remotes store 1, violating "0 = unused"). Delete at the next TX struct bump. The "0 =
    // blend" behaviour its help text promised was never written.
    uint16_t rtm_steer_exit_on_input;  // DEPRECATED (2026-09-19): unread. Was 1=any steering input exits RTM; 0=blend

    // ============================================================
    // V2.5-Evo - 2026-04-27 - PRIORITY 8.1: FM UX REDESIGN
    //
    // 1 new uint16_t field — sizeof grows 124→128 (126 data + 2 tail padding; 126%4=2).
    // First flash of P8.1 firmware resets all TX settings to defaults.
    // ============================================================
    // V2.5-Evo - 2026-09-17 - renamed in place from fm_arm_window_s (same slot, same type, sizeof unchanged).
    uint16_t fm_arm_timeout_s;         // seconds armed with no throttle before auto-disarm; 0 = never (default); range 0-1800 s

    // ============================================================
    // V2.5-Evo - 2026-04-28 - PRIORITY 9: DISTANCE UNIT SELECTION
    //
    // dist_unit fills the 2-byte tail padding left by P8.1; sizeof stays 128.
    // No SPIFFS reset required — old configs read 0 here (tail padding was zero).
    // 0 (Metres) is the correct default, so no migration is needed.
    // ============================================================
    uint16_t dist_unit;               // Distance display unit: 0=Metres, 1=Feet; default 0

    // ============================================================
    // V2.5-Evo - 2026-04-29 - SLEEP TIMEOUT PARAMETER
    //
    // Adds sleep_timeout_s after dist_unit. sizeof grows 128→132
    // (130 data bytes + 2 tail padding; 130 % 4 == 2, float forces 4-byte alignment).
    // SW_VERSION bumped 25→26 — first flash resets all TX SPIFFS settings to defaults.
    // ============================================================
    uint16_t sleep_timeout_s;  // Inactivity sleep timeout; 0=disabled, 60-3600 s; default 300
                               // TX sleeps after this many seconds with no LoRa packet from RX.
                               // Set to 0 to disable auto-sleep entirely.
    // V2.5-Evo - 2026-05-15 - feature/bluetooth: bt_enabled fills the 2-byte tail padding left by
    // sleep_timeout_s. sizeof stays 132. SW_VERSION stays 26 — no SPIFFS reset on first flash.
    // Existing configs read 0 here (padding was zero) = BLE off until set via web UI.
    uint16_t bt_enabled;       // BLE mode: 0=always off, 1=Hall/session (default), 2=always on
    // ============================================================
    // V2.5-Evo - 2026-07-20 - MagGesture: SW_VERSION 26 → 27, sizeof(confStruct) 132 → 136.
    // ============================================================
    // Magnet/Hall gesture role. Selects what the magnet gesture arms when a magnet is held
    // against the case and then removed. Deliberately a SEPARATE field from bt_enabled:
    // BLE and the Hall sensor are independent concerns, and the DRV5032 on GPIO 9 is
    // OPTIONAL EXTRA HARDWARE that many remotes will never have fitted.
    //
    //   0 = off / sensor not fitted (DEFAULT — feature is opt-in; Hall behaves exactly as
    //       it did before this feature existed, i.e. BT dot only)
    //   1 = magnet arms FM   (single 2s threshold — one buzz at 2s, arms on removal)
    //   2 = magnet arms RTM  (single 2s threshold — one buzz at 2s, arms on removal)
    //   3 = magnet arms FM at 2s / RTM at 5s (full two-tier gesture; the tier is decided
    //       by how long the magnet was held, and arming fires on REMOVAL)
    //
    //   4 = magnet TAPS (60-600 ms) step through the Follow-Me stations listed in mag_fm_set, and a
    //       2.5 s hold toggles Return-To-Me on or off FOR THE SESSION ONLY — the stored value below
    //       is never written, so a power cycle brings it back. See the mag_mode 4 block below and
    //       runMagGesture(). A tap only moves a station while Follow-Me is ACTIVELY FOLLOWING; if
    //       Follow-Me is not armed yet, the first tap arms it at the stored default station.
    //       ⚠ MODE 4 HAS NO MAGNET DISARM. This is the one behavioural difference between mode 4 and
    //       modes 1/3 that a rider can be surprised by, so it is written down here: in modes 1 and 3
    //       the magnet is an arm↔DISARM toggle, while in mode 4 the magnet only ever ARMS Follow-Me
    //       or STEPS its station, and the hold only flips the Return-To-Me enable. To disarm
    //       Follow-Me in mode 4, use the toggle combo (LEFT tap → RIGHT hold) or select F0.
    //       (V2.5-Evo - 2026-09-30: documented after the audit  the silent capability change.)
    //
    // Valid range 0-4; default 0. Implemented by runMagGesture() in Hall.ino.
    uint16_t mag_mode;         // magnet/Hall gesture role; 0-4; default 0 (off / not fitted)

    // ============================================================
    // V2.5-Evo - 2026-09-30 - MagStations: mag_fm_set fills the 2 tail padding bytes that
    // mag_mode left behind, so sizeof(confStruct) STAYS 136 and SW_VERSION STAYS 27.
    // This flash does NOT reset the TX SPIFFS config: the owner keeps his throttle
    // calibration, his toggle calibration and his pairing. bt_enabled was added exactly
    // this way at SW26 (see the note above sleep_timeout_s) and this is the same trick.
    // ============================================================
    // Which Follow-Me stations a magnet TAP steps through when mag_mode == 4. One bit per
    // station, so "never send me to station 2" is simply bit 1 left clear:
    //   bit 0 = station 1 (rear right) | bit 1 = station 2 (behind) | bit 2 = station 3 (rear left)
    // V2.5-Evo - 2026-10-07 - F4/F5: the front pair exists on this remote now, so the mask grows two bits:
    //   bit 3 = station 4 (FRONT RIGHT) | bit 4 = station 5 (FRONT LEFT)
    // Valid range 1-31 (at least one station must be selected). The default stays 7 = the three REAR
    // stations: the magnet is a one-touch input with no confirmation before the fact, so sending the
    // buggy in front of the rider has to be something he ticked, not something he inherited. Every remote
    // in the field stores a value in 1-7, so the magnet keeps stepping the same three rear stations until
    // he opts in. A remote flashed from an older build reads 0 out of the old padding bytes;
    // cfgValidateCrossField() silently corrects 0 (and anything above 31) to 7 on load rather
    // than rejecting the config, because a rejection on the load path would write defaultConf
    // and wipe the calibration this whole field placement exists to protect. The repair value is 7
    // and not 31 for the same reason the default is.
    // (The note that used to sit here said stations 4 and 5 did not exist in this firmware and that
    // fmNextStationInSet() masked bits 3 and up off. Both bits now address reachable stations.)
    uint16_t mag_fm_set;       // magnet-tap station set, bitmask bit0=F1 bit1=F2 bit2=F3 bit3=F4 bit4=F5; 1-31; default 7 (the three rear stations)
};

// V2.5-Evo - 2026-07-20 - MagGesture: 132 → 136. mag_mode is a uint16_t (+2 bytes = 134), but the
// struct's alignment is 4 (it contains uint32_t/float members), so the compiler pads the tail back
// out to 136. Those 2 trailing padding bytes are the slot a future uint16_t field can occupy for
// free — exactly how bt_enabled was added at SW26 without changing sizeof.
// V2.5-Evo - 2026-09-30 - MagStations: mag_fm_set CLAIMED those 2 padding bytes. The data now fills
// all 136 bytes with no tail padding left, so sizeof is STILL 136 and SW_VERSION is STILL 27 — the
// config version check passes on the next boot and nothing in SPIFFS is reset. THE TAIL IS NOW FULL:
// the next uint16_t added to this struct grows sizeof to 140, which IS a version bump and IS a config
// wipe. Do not add one without saying so out loud.
static_assert(sizeof(confStruct) == 136, "confStruct size mismatch — expected 136 bytes (V2.5-Evo sleep_timeout_s + bt_enabled + mag_mode + mag_fm_set; no tail padding left). Update this assert if you change the struct.");  // pinned to exact size; catches both shrinkage and unexpected growth
confStruct usrConf;

// ============================================================
// V2.5-Evo - 2026-07-20 - MagGesture: mag_mode role decoding
// Single source of truth for what the magnet gesture arms.
// See the mag_mode field comment above for the mode table.
// ============================================================
#define MAG_ROLE_NONE  0   // magnet gesture dormant; Hall behaves exactly as it did pre-gesture
#define MAG_ROLE_FM    1   // single 2s threshold arms FM
#define MAG_ROLE_RTM   2   // single 2s threshold arms RTM
#define MAG_ROLE_BOTH  3   // two-tier: 2s → FM, 5s → RTM
// V2.5-Evo - 2026-09-30 - MagStations: the fourth role. A short TAP (60-600 ms; the ceiling was 400 ms
// until the 2026-09-30 MagFix — see kMagTapMaxMs in Hall.ino) steps through the Follow-Me stations
// selected in mag_fm_set, but ONLY while Follow-Me is actively following; a 2.5 s hold toggles
// Return-To-Me on or off for THIS SESSION (RAM, never SPIFFS). Roles 1-3 keep their own timings, buzzes
// and arm-on-removal behaviour. NOTE: mode 4 has no magnet disarm — see the mag_mode field comment.
#define MAG_ROLE_FMSET 4   // tap = next station in mag_fm_set (FM following only); 2.5s hold = RTM on/off (session)

// Returns what the magnet gesture should arm for the current mag_mode value.
// Inputs: usrConf.mag_mode (0-4). Output: MAG_ROLE_NONE / _FM / _RTM / _BOTH / _FMSET.
// No side effects. Mode 0 and anything out of range return NONE, so the gesture stays
// completely dormant for remotes with no Hall sensor fitted (the default).
static inline uint8_t magGestureRole()
{
  switch (usrConf.mag_mode)
  {
    case 1:  return MAG_ROLE_FM;
    case 2:  return MAG_ROLE_RTM;
    case 3:  return MAG_ROLE_BOTH;
    case 4:  return MAG_ROLE_FMSET;   // V2.5-Evo - 2026-09-30 - MagStations
    default: return MAG_ROLE_NONE;
  }
}

// ============================================================
// V2.5-Evo - 2026-07-20 - SW27: defaultConf holds the factory default configuration.
//
// WHY: the SW26→SW27 bump changes sizeof(confStruct) 132→136, so the first flash
// of SW27 discards the SPIFFS config and writes defaultConf verbatim. Whatever is
// in this initializer is exactly what the device ends up running, so it must hold
// known-safe, self-consistent defaults (pairing unbound, calibration nominal).
//
// ⚠ POSITIONAL INITIALIZER — the order below is confStruct declaration order.
// Inserting, removing or transposing a single entry silently shifts every value
// after it (corrupt calibration, wrong LoRa address) with NO compile error.
// If you add a struct field, add its initializer entry at the MATCHING position.
// ============================================================
confStruct defaultConf = {  // V2.5-Evo — factory default configuration
  SW_VERSION,    // version (27 — always SW_VERSION, never a hardcoded number)
  2,             // radio_preset (US 915MHz)
  22,            // rf_power (0-22 dBm)
  // --- TX CALIBRATION BLOCK — nominal raw ADC endpoints for the throttle and
  // toggle travel limits. These are Hall-sensor calibration values, not tuned
  // per unit; a fresh build should re-run throttle + toggle calibration on the
  // device to derive its own. With cal_ok=0 the firmware forces that calibration
  // on first boot. Do not "tidy" or round these.
  0,             // cal_ok (0 = force Hands-Off throttle calibration on first boot; field default)
  100,           // cal_offset
  15195,         // thr_idle   (nominal; recalibrate on device)
  11909,         // thr_pull   (nominal; recalibrate on device)
  12310,         // tog_left   (nominal; recalibrate on device)
  13806,         // tog_mid    (nominal; recalibrate on device)
  14908,         // tog_right  (nominal; recalibrate on device)
  // --- end calibration block ---
  500,           // tog_deadzone
  30,            // tog_diff
  200,           // tog_block_time   (wait to finish steering before Dynamic Throttle /was 500 5 secs)
  3500,          // trig_unlock_timeout
  2000,          // lock_waittime
  80,            // gear_change_waittime
  800,           // gear_display_time
  2,             // menu_timeout
  2000,          // err_delete_time
  0,             // no_lock
  2,             // throttle_mode
  6,             // max_gears
  0,             // startgear
  1,             // steer_enabled
  100,           // thr_expo (50 = linear; 100 = fully exponential — gentle at low throttle, aggressive at high;
                 //           0 = the opposite curve — aggressive at low throttle. See expoThrCurve() in Hall.ino)
  2,             // fm_display_mode (2 = distance to the buggy in metres; range 1-4). V2.5-Evo - 2026-10-07: default was 1 (TX speed)
  50,            // steer_expo
  0,             // gps_dyn_model (was steer_expo1; 0 = default = Sea, unchanged behaviour)
  0.000185662f,  // ubat_cal
  1,             // gps_en (1 = TX GPS enabled)
  2,             // followme_mode (2 = Behind — defensive FM geometry; 1=near_right, 3=near_left)
  1,             // kalman_en
  5,             // speed_src (5 = TX mph)
  3000,          // tx_gps_stale_timeout_ms
  0,             // paired (unbound — fresh unit must pair itself)
  {0, 0, 0},     // own_address (unbound)
  {0, 0, 0},     // dest_address (unbound)
  {'1','2','3','4','5','6','7','8'}, // wifi_password: documented DEFAULT AP password "12345678" — change before use (SOP-020)
  85,            // dynamic_power_start
  5,             // dynamic_power_step
  // V2.5-Evo - 2026-04-22 - default HDOP gate: 200 = HDOP 2.0. Fits in former tail-padding bytes.
  200,           // gps_max_hdop (200 = HDOP 2.0; existing configs read 0 here → validation rejects → defaults written)
  // V2.5-Evo - 2026-04-22 - GPS chip type: 0 = BN-220 (9600→115200, 5Hz). TX only supports 0 and 2.
  0,             // gps_chip_type (0=BN-220 default; old configs → decodedLen check fails → defaults written)
  // V2.5-Evo - 2026-04-25 - Priority 7 RTM/FM defaults
  1,    // rtm_enabled
  3,    // rtm_hold_duration_s (3-10 s; 3 = at floor)
  15,   // rtm_arm_window_s (5-30 s)
  // V2.5-Evo - 2026-08-17 - Was 0 (single squeeze). The struct comment beside this field always
  // documented "default 1", but defaultConf shipped 0, so every factory-reset TX armed RTM on the
  // EASIER gesture — a single squeeze held ~500 ms, which an accidental throttle pull can satisfy
  // on a machine that tows a person through water. Now 1: the deliberate double squeeze is the
  // shipped default, and firmware and comment finally agree.
  // Existing units are deliberately untouched: no confStruct change and no SW_VERSION bump, so a TX
  // with a stored value keeps it. This only affects units never configured, or factory-reset later.
  0,    // rtm_double_squeeze_en (0 = SINGLE squeeze held ~500 ms — OWNER'S DEFAULT 2026-10-02, set on his board and here so a config wipe cannot quietly restore the double; 1 = double squeeze, the upstream default)
  30,   // rtm_throttle_start_pct
  70,   // rtm_throttle_max_pct
  5,    // rtm_ramp_duration_s
  10,   // rtm_disengage_distance_m (consistency with RX stop dist 10 m + 8 m GPS floor; 3-20 m)
  0,    // rtm_max_runtime_s (0=disabled — safety gates handle all real scenarios; P8 changed from 120)
  2000, // rtm_gps_timeout_ms
  3,    // fm_hold_duration_s (3-10 s; 3 = at floor)
  1,    // fm_override_enabled
  // V2.5-Evo - 2026-04-27 - Priority 8 UX overhaul defaults
  0,    // rtm_display_mode (0=distance; set 1 for speed, 2 for alternating)
  150,  // fm_warn_distance_m (150m FM proximity warning threshold)
  1,    // rtm_steer_exit_on_input (1=steering exits RTM; 0=blend only)
  // V2.5-Evo - 2026-04-27 - Priority 8.1 FM UX redesign defaults
  // V2.5-Evo - 2026-09-17 - fm_arm_timeout_s ships at 0 = NEVER auto-disarm (owner decision
  // 2026-09-15). Follow-Me is meant to stay armed for the whole session; only the disarm
  // gesture, an RX fault, or remote power-off ends it. The previous 180 s default existed
  // because the arm had to survive float → takeoff → tow → whip and a 30 s window expired
  // mid-sequence — with 0 there is no window to outlast. If a timeout IS wanted, the same
  // reasoning still applies: keep it well above the longest float, so 180+ rather than 30.
  // Units are seconds, validated range 0-1800 (0 = never, up to 30 minutes).
  0,    // fm_arm_timeout_s (0 = never auto-disarm; range 0-1800 s)
  0,    // dist_unit (0 = Metres)
  // V2.5-Evo - 2026-04-29 - sleep timeout default
  300,  // sleep_timeout_s — 300s = 5 minutes; set to 0 to disable
  2,    // bt_enabled (0=off, 1=Hall/session, 2=always on)
  // V2.5-Evo - 2026-07-20 - MagGesture: the FIELD default is documented as 0 = OFF / Hall
  // sensor not fitted (see the mag_mode field comment in confStruct) — the general, optional-
  // hardware case, where the gesture must stay opt-in because a remote with no DRV5032 on
  // GPIO 9 has undefined P_MAG state. Enable 1/2/3 per device via the web UI.
  0,    // mag_mode (0 = off / Hall not fitted; enable per device via web UI)
  // V2.5-Evo - 2026-09-30 - MagStations: 7 = 0b111 = all three Follow-Me stations selected, which
  // is exactly what the remote already does when the toggle cycles stations. Shipping the
  // permissive value means enabling mag_mode 4 changes WHICH INPUT cycles the stations, never
  // which stations exist. Narrow it per rider in the web UI (three tick boxes).
  // V2.5-Evo - 2026-10-07 - F4/F5: the toggle now steps all five stations, but this default stays 7 = the
  // three REAR stations, so the front pair is opt-in on the magnet (bits 3 and 4, ticked in the web UI).
  7,    // mag_fm_set (bit0=F1, bit1=F2, bit2=F3, bit3=F4, bit4=F5; 7 = the three rear stations)
};


// V2.5-Evo - 2026-05-16 - feat(telemetry): expand LoRa packet 8→19 bytes + 0xF4 aux meta-packet
//Telemetry to receive, MUST BE 8-bit!!
// V2.5-Evo - 2026-04-27 - P8: Added rtm_distance at index 5; see encoding comment at RX side.
struct __attribute__((packed)) TelemetryPacket {
    uint8_t foil_bat = 0xFF;          // index 0 — battery % 0-100
    uint8_t foil_temp = 0xFF;         // index 1 — FET temp degC
    uint8_t foil_speed = 0xFF;        // index 2 — speed km/h
    uint8_t error_code = 0;           // index 3 — fault flags
    uint8_t foil_power = 0xFF;        // index 4 — power (watts/50); 0xFF = N/A
    uint8_t rtm_distance = 0xFF;      // index 5 — RX→TX distance; 0xFF = N/A
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
    uint8_t fm_flags = 0;             // index 16 — Follow-Me engagement sub-state from the RX FM brain.
                                      //   [0]=armed [1]=engaged [2]=armed-not-ready(RX: latch not proven) [3]=fault-stop(sticky 6s).
                                      //   V2.5-Evo - 2026-07-20 - Batch T: repurposed the unused reserved_tx_imu byte (was 0xFF).
                                      //   Default 0 (not 0xFF) so that before any RX packet arrives no FM bit reads as set — matches
                                      //   the RX-side default. Written by the generic index-addressed telemetry unpack in Radio.ino.
    uint8_t rx_bearing_to_tx = 0xFF;  // index 17 — bearing from buggy toward rider÷2; 0xFF = N/A
    uint8_t link_quality = 0;         // index 18 (was "must be last": it is rotated like every other index and read by name)
    // V2.5-Evo - 2026-10-07 - index 19 - rx_state_flags: how the BUGGY ended a return (bit map: RX_STATE_* below).
    // APPENDED, so every older index keeps its place. An older RX never sends index 19: the byte stays 0 and every
    // reader below treats 0 as "nothing to report", so a new remote with an old buggy behaves exactly as before.
    uint8_t rx_state_flags = 0;       // index 19
} telemetry;
static_assert(sizeof(TelemetryPacket) == 20, "TelemetryPacket must be 20 bytes (indices 0-19) to match the RX");

// ============================================================
// V2.5-Evo - 2026-10-07 - telemetry.rx_state_flags (index 19) bit map, as the RX assembles it (RX RTMState.ino).
//   bit0 RTM FAULT-STOP, sticky ~6 s: the buggy ended Return-To-Me on a fault -> "St" + stop buzz, cap 255 (SOP-039).
//   bit1 RTM ARRIVED, sticky ~6 s: the buggy ended Return-To-Me at its stop distance -> silent "St", near-zero cap
//        until one full release (SOP-040 arrival rule).
//   bit2 HAND-BACK CAP STANDING on the buggy (display only; the remote adds no cap for it).
//   bit3 the buggy holds this remote's boot ID (S-8).  bit4 the buggy's RTM refresh expiry is armed (H-1).
// The two sticky bits are acted on by their RISING EDGE (latched where the byte arrives, Radio.ino), so a bit left
// over from the previous run (sticky for 6 s) cannot end a new one.
// ============================================================
#define RX_STATE_RTM_FAULT     0x01
#define RX_STATE_RTM_ARRIVED   0x02
#define RX_STATE_HANDBACK_CAP  0x04
#define RX_STATE_BOOT_ID_HELD  0x08
#define RX_STATE_REFRESH_ARMED 0x10
// rx_rtm_fault_rise_ms / rx_rtm_arrived_rise_ms - millis() of the index-19 ARRIVAL on which that bit rose (it was
// clear on the previous arrival). 0 = never. Written only by waitForTelemetry (Radio.ino); read by runRtmLoop(),
// which acts only on a rise stamped after the current run went ACTIVE. One aligned word each, no tearing.
volatile unsigned long rx_rtm_fault_rise_ms   = 0;
volatile unsigned long rx_rtm_arrived_rise_ms = 0;
// rtm_start_sent_ms - millis() when an 0xF1/1 ("RTM active") packet last went on the air. Written by sendData; read
// by the loop task for the Q-2 confirmation count and the H-1 refresh start. Twin of rtm_stop_sent_ms.
volatile unsigned long rtm_start_sent_ms      = 0;

// ============================================================
// V2.5-Evo - 2026-07-20 - Batch T (FM design v1.4): telemetry.fm_flags (index 16) bit map.
// The RX FM brain assembles this byte; the new TX consumes it to drive the R5 display and the
// disarm-ownership rule. An OLD TX ignores index 16 entirely (it was a reserved byte).
// ============================================================
#define FM_FLAG_ARMED     0x01  // bit0: RX FM armed
#define FM_FLAG_ENGAGED   0x02  // bit1: RX FM engaged (actively steering / capping)
#define FM_FLAG_NOTREADY  0x04  // bit2: RX-side armed-not-ready (separation latch not yet proven)
#define FM_FLAG_FAULT     0x08  // bit3: RX fault-stop, sticky 6s (already surprise-gated on the RX)
// V2.5-Evo - 2026-09-19 - bits 4 and 5: the stick during auto-steer. Bit 4 is the buggy's echo of its
// steer_during_auto setting, sent every tick in every FM state: 0 = the stick CANCELS automatic steering (Gate 4 in
// RTMState.ino exits a classic return-to-me on a push, as it always has); 1 = the stick TAKES OVER (the buggy steers
// by the stick while it is deflected and resumes on centring; Gate 4 stands down so the remote does not exit the run
// the buggy is deliberately continuing). Bit 5 = a takeover is STANDING on this tick (display only). Both are read
// only under the FM_LINK_HEALTHY_MS window like the other flags - a stale packet reads as cancel, never as takeover.
// An old RX never sets either bit, so a new remote with an old buggy cancels everywhere, as before.
// (V2.5-Evo - 2026-10-07 - bit 6 is no longer free: see FM_FLAG_RETURN_STANDING below.)
// V2.5-Evo - 2026-10-07 - Q-5: Gate 4 now stands down whatever bit 4 says (steering never ends a return on the remote),
// so the remote no longer reads bit 4; it stays defined because the buggy still sends it.
#define FM_FLAG_STEER_TAKEOVER 0x10  // bit4: RX steer_during_auto is 1 (take over) - Gate 4 steer-exit stands down
#define FM_FLAG_STEER_ACTIVE   0x20  // bit5: a stick takeover is standing on the RX right now (display only)
// V2.5-Evo - 2026-09-19 - bit 7: the RX's EFFECTIVE auto-return mode (its stored fm_return_mode unless this remote has
// overridden it for the session). Read by returnGesture() (RTMState.ino) to flip the override to the OPPOSITE value, and
// shown as "Ar" (ON) / "AO" (OFF) at that moment. Bit 6 is reserved for the accepted-mode echo (which will need its own
// telemetry byte if it needs 3 bits).
#define FM_FLAG_RETURN_ON 0x80  // bit7: RX effective auto-return mode is ON
// V2.5-Evo - 2026-10-07 - bit 6 (audit S-7): AUTO-RETURN STANDING on the buggy (its Follow-Me is in FM_RETURN). With
// bit 1 (engaged) also set the buggy is RETURNING (moving toward the rider); with bit 1 clear it is WAITING (parked).
// An old RX never sets it, so the remote then draws following / armed exactly as before.
#define FM_FLAG_RETURN_STANDING 0x40  // bit6: auto-return standing (bit 1 tells returning from waiting)
// Link-health window: the TX treats the RX link as alive only while a packet has landed within
// this many ms (matches the existing `millis()-last_packet < 1000` failsafe window used for the
// bargraphs/vibration connectivity checks). Used by the FM readiness OR and the engaged gate.
#define FM_LINK_HEALTHY_MS 1000UL

// ============================================================
// V2.5-Evo - 2026-09-30 - fm_engaged_streak: HOW MANY CONSECUTIVE ARRIVALS of the fm_flags byte have
// said "Follow-Me armed AND engaged". Written only by the telemetry unpack in Radio.ino (the
// waitForTelemetry task, where the byte physically arrives); read by fmIsEngaged() in RTMState.ino.
//
// WHY IT EXISTS. The magnet tap may only move a Follow-Me station while the buggy is actively
// following - never while the rider is on the tow rope, attached and unable to steer away. That rule
// used to rest on ONE bit inside ONE CRC8-protected packet. This counter is the corroboration:
// fmIsEngaged() requires the claim twice before it believes it. Reset to 0 the moment an fm_flags byte
// arrives without both bits, so it tracks the CURRENT claim and never accumulates history.
//
// WHY IT COUNTS ARRIVALS OF THE BYTE AND NOT PACKETS OR LOOP TICKS. The RX sends ONE telemetry byte
// per packet (Radio.ino: ptr[rcvArray[3]] = rcvArray[4]) and rotates through the indices, so
// telemetry.fm_flags only changes when index 16 comes round. Counting received packets or loop
// iterations would just re-count the same cached byte over and over and would corroborate nothing.
//
// It saturates instead of wrapping: a uint8_t that rolled over would momentarily read 0 or 1 and
// close the gate under a perfectly healthy link.
// volatile: written by a task, read by the loop task and the bargraph task. Single-byte accesses are
// one instruction on this RISC-V core, so no read can tear.
// ============================================================
volatile uint8_t fm_engaged_streak = 0;   // consecutive fm_flags arrivals with ARMED+ENGAGED both set

// ============================================================
// V2.5-Evo - 2026-10-07 - fm_fault_latched: the RX Follow-Me FAULT-STOP edge, latched where it arrives
// (audit R-6). Set true by the telemetry unpack in Radio.ino (waitForTelemetry task) when an fm_flags byte
// arrives with FM_FLAG_FAULT set and the previous arrival had it clear. Cleared only by runFmLoop()
// (RTMState.ino, loop task) when it handles the edge.
// WHY: runFmLoop() used to look for the rising edge itself, by comparing telemetry.fm_flags tick to tick.
// The RX holds bit 3 for only ~6 s, and the blocking RTM arm ceremony can stall loop() for the arm window
// plus ~4 s, so a fault that rose and fell inside one stall was never seen: the remote stayed fm_armed
// and kept re-declaring Follow-Me to a buggy that had stopped it. A latch cannot be missed, however late
// loop() gets to it. volatile bool: one writer task, one reader/clearer task; single-byte accesses do not
// tear on this core, and the RX holds the bit for seconds, so two edges can never race one clear.
// ============================================================
volatile bool fm_fault_latched = false;

// ============================================================
// V2.5-Evo - 2026-10-07 - SOP-041 rule 3 RESYNC: "the remote thinks Follow-Me is armed, the buggy says it is not".
//   fm_flags_unarmed_since_ms - millis() of the FIRST of the current run of fm_flags arrivals with bit 0 (armed) clear;
//                               0 = the last arrival had bit 0 set.
//   fm_flags_unarmed_streak   - how many consecutive fm_flags arrivals had bit 0 clear (saturates).
// Written only by waitForTelemetry (Radio.ino) on the arrival of index 16; read by runFmLoop() (RTMState.ino).
// ============================================================
volatile unsigned long fm_flags_unarmed_since_ms = 0;
volatile uint8_t       fm_flags_unarmed_streak   = 0;

/*
** FreeRTOS/Task handles
*/
// Unused — serPrintTasks() uses uxTaskGetStackHighWaterMark() directly
//const int maxTasks = 10;
//TaskStatus_t taskStats[maxTasks];

// Task handles
TaskHandle_t sendDataHandle = NULL;
TaskHandle_t triggeredWaitForTelemetryHandle = NULL;
TaskHandle_t measBufCalcHandle = NULL;
TaskHandle_t updateBargraphsHandle = NULL;
TaskHandle_t vibrationTaskHandle = NULL;  // Finding 4-1: saved so ?printtasks can measure stack HWM

//TaskHandle_t triggeredReceiveHandle = NULL;
//TaskHandle_t checkConnStatusHandle = NULL;

extern TaskHandle_t loopTaskHandle;

// --- V2.5-Evo: TX GPS globals ---
// gps_tx   : TinyGPS++ parser instance fed by Serial1 (BN-220).
//            V2.5-Evo - 2026-06-07 - Audit #10 invariant: TinyGPS++ is NOT
//            reentrant. gps_tx.encode() (writes) and every .location/.speed/.hdop
//            read must happen from the Arduino loop() task ONLY — never from an
//            ISR or a second core. On this single-core C3 with a cooperative loop
//            the encode sites (GPS.ino getTxGPSLoop; RTMState.ino keepalive +
//            pre-arm drain) and all readers run sequentially, so no torn mid-
//            sentence read can occur; the NMEA checksum also discards any garbled
//            sentence before its fields become readable. Keep all gps_tx access on
//            the loop task to preserve this guarantee.
// tx_gps_speed : Current speed in the UNIT selected by usrConf.speed_src.
//                Sentinel 0xFF = no fix / no valid data (matches existing
//                telemetry.foil_speed "not available" convention so the
//                display helper can render "--" without extra logic).
//                Written only by getTxGPSLoop() in GPS.ino, read by
//                Display.ino and Hall.ino — all in the Arduino loop task.
//                volatile prevents the compiler from caching or reordering
//                these accesses; no cross-core synchronization is needed.
TinyGPSPlus gps_tx;
volatile uint8_t tx_gps_speed = 0xFF;
// --- End V2.5-Evo: TX GPS globals ---

/*
** Variables
*/
uint16_t displayBuffer[8];
SemaphoreHandle_t displayMutex;   // protects displayBuffer + updateDisplay() — created in initTasks() before tasks start
// SW48: convenience macros — use these in all code that writes displayBuffer or calls updateDisplay()
// from outside an already-held displayMutex context (i.e. NOT from inside renderOperationalDisplay
// or updateBargraphs which take the mutex themselves).
#define DISP_LOCK()   do { if(displayMutex) xSemaphoreTake(displayMutex, portMAX_DELAY); } while(0)
#define DISP_UNLOCK() do { if(displayMutex) xSemaphoreGive(displayMutex); } while(0)

// V2.5-Evo - 2026-07-20 - audit §4.6 (H4): serializes the SHARED I2C bus. The HT16K33 display (0x70)
// and the ADS1115 throttle/steer/battery ADC (0x48) sit on the same Wire (SDA=2/SCL=1). displayMutex
// protects the displayBuffer DATA structure; i2cMutex protects the physical BUS. They are separate:
// the lock order is always displayMutex (outer, optional) → i2cMutex (inner, leaf) on the render path,
// while the ADS path (measBufCalc, prio 6) takes ONLY i2cMutex — so no cycle, no deadlock. Created in
// initTasks() before any task starts; the macros no-op until then (startup is single-threaded anyway).
SemaphoreHandle_t i2cMutex;   // serializes every HT16K33 and ADS1115 Wire transaction — created in initTasks()
#define I2C_LOCK()   do { if(i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY); } while(0)
#define I2C_UNLOCK() do { if(i2cMutex) xSemaphoreGive(i2cMutex); } while(0)
// Unused — shadowed by local declarations in displayDigits(), scroll3Digits(), scroll4Digits()
//uint8_t digitBuffer[6];

std::atomic<bool> rfInterrupt{false};

volatile uint8_t local_link_quality = 0;

volatile unsigned long last_packet = 0;
volatile unsigned long num_sent_packets = 0;
volatile unsigned long num_rcv_packets = 0;

// Unused — replaced by TelemetryPacket struct
//volatile uint8_t vesc_bat = 0;
//volatile uint8_t vesc_temp = 0;
//volatile uint8_t remote_sq = 0;
volatile uint8_t remote_error = 0;
volatile bool remote_error_blocked = 0;

volatile bool in_setup = 0;
volatile bool config_version_error = false;

// Unused — replaced by local buffers in waitForTelemetry() and initiatePairing()
//volatile uint8_t payload_buffer[10];
//volatile uint8_t payload_received = 0;

// Pairing timeout in milliseconds
const unsigned long PAIRING_TIMEOUT = 5000;
// TODO: Use when address conflict detection is implemented
//const uint8_t MAX_ADDRESS_CONFLICTS = 5;            // Maximum number of address conflicts before giving up

//Ring Buffer for Hall Sensors
#define BUFFSZ 6
volatile uint16_t thr_raw[BUFFSZ];
volatile uint16_t tog_raw[BUFFSZ];
volatile uint16_t intbat_raw[BUFFSZ];

volatile int filter_count = 0;
volatile int bat_filter_count = 0;
volatile int last_channel = 0;

// ============================================================
// V2.5-Evo - 2026-10-07 - C-1: THROTTLE INPUT HEALTH (frozen-throttle fix)
// THE BUG: measureAndBuffer() (Analog.ino) only writes thr_raw[] when the ADS1115 reports a finished
// conversion. If the I2C bus or the ADS1115 fails (SDA held low after water ingress, a loose wire, a
// display fault that wedges the shared bus) no conversion ever finishes, the buffer stops changing,
// and calcFilter() keeps producing the LAST throttle value. sendData() keeps sending it, so the buggy
// keeps driving after the rider lets go. Only a hard power-off ended it.
// THE FIX: every finished conversion stamps last_ads_ok_ms. If none has finished for ADS_STALE_MS while
// hall sampling is on, or a raw throttle reading lands far outside the calibrated idle..pull band, the
// input is declared faulty: throttle 0, steering centred, toggle (and every gesture) blocked, and the
// remote shows error 72 as a blinking "St" with the stop buzz. The fault clears only after fresh, in-band
// readings fill the whole filter buffer AND they show the trigger released, so throttle can never jump
// back to a held position when the bus recovers.
// Writers: measBufCalc task (prio 6) for all four. V2.5-Evo - 2026-10-07 - P-1: the sendData task no longer
// sets ads_input_fault; when it sees the deadline missed it zeroes only the packet it is sending (a CPU stall
// must not latch a fault), and measBufCalc latches from its own per-pass deadline (adsTaskPassStale()).
// Every variable is one aligned word or byte, so no read can tear on this single-core RISC-V part.
// ============================================================
#define ADS_STALE_MS 50UL                       // ms with no finished ADS1115 conversion before the input is untrusted
#define REMOTE_ERR_INPUT_FAULT 72               // TX-local remote_error code: throttle/toggle input fault (screen: blinking "St")
volatile unsigned long last_ads_ok_ms = 0;      // millis() of the last finished ADS1115 conversion (any channel)
volatile bool    ads_input_fault      = false;  // latched: throttle input untrusted; outputs forced safe until recovery
volatile bool    ads_thr_out_of_range = false;  // set by measureAndBuffer() when a raw throttle sample is implausible
volatile uint8_t ads_thr_good_samples = 0;      // in-band throttle samples stored since the last bad event (saturates)
// V2.5-Evo - 2026-10-07 - P-11: true only while checkCal() (Hall.ino) is actually calibrating. The throttle plausibility
// check is skipped then (there is no trusted band yet). It used to be skipped whenever usrConf.cal_ok was 0, so a runtime
// `?set cal_ok 0` switched the check off for the whole session. Written by the loop task, read by the ADC task; one byte.
volatile bool    ads_cal_in_progress  = false;

volatile int gear = 0;
volatile uint8_t max_power_cap = 85;  // Runtime cap for throttle_mode 2

volatile uint8_t thr_scaled = 0;
volatile uint8_t tog_scaled = 0;
volatile uint8_t steer_scaled = 0;

// ============================================================
// V2.5-Evo - 2026-10-07 - SOP-040 GESTURE RULE: ONE DEFINITION OF "THE TRIGGER IS FULLY RELEASED"
// The Return-To-Me gestures (the magnet 2.5 s hold and the toggle RIGHT tap + LEFT hold) act only with the trigger
// fully released, and an RTM arrival keeps its throttle cap until the trigger has been fully released once. Both
// use this test. thr_scaled < 10 (of 255, ~4 %) is the release threshold the toggle gestures and the RTM arm
// ceremony already used (handleGearToggle(), runDoubleSqueezeArm()), so nothing that worked before changes meaning.
// Reads thr_scaled (written by the ADC task). No side effects.
// ============================================================
#define TRIGGER_RELEASED_MAX 10
static inline bool triggerReleased() { return thr_scaled < TRIGGER_RELEASED_MAX; }

volatile uint8_t thr_sent = 0;   // Post-expo+gear throttle actually sent over radio
volatile uint8_t steer_sent = 0; // Steering value actually sent over radio

// V2.5-Evo - 2026-04-25 - P7 RTM meta-packet burst queue.
// V2.5-Evo - 2026-05-13 - SW32 M3: changed volatile→std::atomic<T>.
// Loop task writes type/value with memory_order_relaxed, then stores count
// with memory_order_release. The sendData task loads count with memory_order_acquire
// before reading type/value. volatile prevented compiler caching but not CPU store-buffer
// reordering; std::atomic release/acquire prevents sendData from observing count>0
// while type/value are still stale in the loop task's store buffer.
std::atomic<uint8_t> rtm_meta_type  {0};    // 0xF1=RTM state, 0xF2=FM override
std::atomic<uint8_t> rtm_meta_value {0};    // for 0xF1: 0=inactive 1=active 2=refresh (H-1) 0x80|id=boot ID (S-8); for 0xF2: the mode byte
std::atomic<uint8_t> rtm_meta_count {0};    // bursts remaining; 0 = idle (value is always 0 or 3)
// V2.5-Evo - 2026-10-07 - H-1 / L-1: the three atomics above are now the HEAD of a 2-deep queue (a second slot
// lives in Radio.ino). rtm_meta_count == 0 still means "nothing queued at all", because the second slot is
// promoted into the head the moment the head empties. See queueMetaPacketBurst().

// ============================================================
// V2.5-Evo - 2026-10-07 - H-1 (TX part): "the buggy is still in Return-To-Me but this remote is not".
// The buggy reports its RTM state in telemetry.fm_status bit 1 (RX RTMState.ino: rtm_rx_active). If the
// remote missed sending, or the buggy missed hearing, the 0xF1/0 that ends a return (link loss, a remote
// reboot or sleep mid-return, all three burst packets lost), the buggy kept running RTM with the remote
// showing manual. runRtmLoop() now watches that bit and re-sends 0xF1/0 until it clears.
//   fm_status_arrival_ms - millis() when the fm_status byte (telemetry index 15) last ARRIVED. The byte is
//                          cached between arrivals, so only an arrival after our stop went out is evidence.
//                          Written by waitForTelemetry (Radio.ino), read by the loop task.
//   rtm_stop_sent_ms     - millis() when an 0xF1/0 packet last went on the air. Written by sendData.
// ============================================================
#define FM_STATUS_RTM_ACTIVE 0x02                 // telemetry.fm_status bit 1: the buggy has rtm_rx_active set
volatile unsigned long fm_status_arrival_ms = 0;
volatile unsigned long rtm_stop_sent_ms     = 0;

// ============================================================
// V2.5-Evo - 2026-10-07 - A-1 (TX part): "the buggy ended Return-To-Me but this remote still shows it".
// The buggy ends a manual return on its own (arrival at the stop distance, a Phase C failure, its 30 s takeover
// timeout) and says so only by clearing fm_status bit 1. The remote never read that bit while ACTIVE, so it kept
// the RTM screen and cap until a 4 s release. runRtmLoop() now ends its own RTM when the buggy, having confirmed
// RTM during this run, reports it OFF on 2 consecutive arrivals of the fm_status byte.
//   fm_status_rtm_on_ms      - millis() of the last fm_status ARRIVAL with bit 1 set (0 = never).
//   fm_status_rtm_off_streak - consecutive fm_status arrivals with bit 1 clear since the last set one (saturates).
// Written only by waitForTelemetry (Radio.ino), where the byte arrives; read by the loop task. Counted per ARRIVAL of
// index 15 (the cached byte only changes when its index comes round), like fm_engaged_streak. One word / one byte.
// ============================================================
volatile unsigned long fm_status_rtm_on_ms      = 0;
volatile uint8_t       fm_status_rtm_off_streak = 0;

// V2.5-Evo - 2026-04-25 - P7 RTM throttle cap.
// V2.5-Evo - 2026-05-13 - SW32 M3: changed volatile→std::atomic<T>.
// Written by the loop task via RTMState.ino; read by sendData via calcFinalThrottle().
// 255 = no cap (RTM not active). During RTM ACTIVE, set to the ramped cap value
// (30-70% of 255). Applied in calcFinalThrottle(). RTM can only subtract from
// user throttle — never add. Creator safety philosophy enforced here.
std::atomic<uint8_t> rtm_thr_cap_tx {255};
std::atomic<bool>    rtm_tx_active  {false};

// V2.5-Evo - 2026-06-05 - C-1: 2nd independent throttle gate during RTM arm ceremony.
// true only while rtm_tx_state == RTM_ARMED (the blocking arm window). Read by sendData()
// to hard-zero the throttle byte independently of rtm_thr_cap_tx. Defined in RTMState.ino.
bool rtmIsArming();

// V2.5-Evo - 2026-04-28 - P9 S4: RTM arm distance captured at engage moment.
// Used by R5 proximity bar to set the 100% reference distance.
// RAM only — never written to SPIFFS. Reset to 0.0f when RTM disengages.
float rtm_arm_dist_m = 0.0f;

//-1 = left, 1 = right input
volatile int tog_input = 0;

volatile float int_bat_volt = 0.0;

volatile bool mot_active = 0;
volatile bool system_locked = 1;

// V2.5-Evo - 2026-05-13 - SW46: THR at centre(1) — LEFT=Temp(0), RIGHT=Speed(2)→Power(3)→Amp(4)→UBat(5)→Bat(6)→wrap Temp.
// All switch() cases use named constants — only these #defines change.
// display mode cycle: 0=temp, 1=throttle, 2=speed, 3=power(kW), 4=motor amps(MA), 5=TX int bat, 6=foil bat
#define DISPLAY_MODE_TEMP    0
#define DISPLAY_MODE_THR     1
#define DISPLAY_MODE_SPEED   2
#define DISPLAY_MODE_POWER   3
#define DISPLAY_MODE_AMP     4
#define DISPLAY_MODE_INTBAT  5
#define DISPLAY_MODE_BAT     6
#define DISPLAY_MODE_COUNT   7
// V2.5-Evo - 2026-05-13 - SW32: throttle % (DISPLAY_MODE_THR) as default boot display.
// Field test feedback: throttle % is more useful at-a-glance than temperature on first unlock.
// User can still cycle all modes via toggle. Was 0 (DISPLAY_MODE_TEMP).
volatile uint8_t display_mode = DISPLAY_MODE_THR;

volatile uint16_t toggle_blocked_counter = 0;
volatile bool toggle_blocked_by_steer = 0;
volatile int in_menu = 0;

volatile uint8_t sq_graph = 0;
volatile uint8_t last_known_temp_graph = 0;
volatile uint8_t last_known_bat_graph = 0;
volatile bool blink_bargraphs = 0;

volatile bool exitChargeScreen = 0;

volatile bool followme_enabled = false;

volatile bool serialOff = false;
volatile bool mag_seen_high = false;  // set true when GPIO 9 first reads HIGH after boot; gates intentional activation
// BT dot test states — Hall sensor (P_MAG / GPIO 9) drives bt_dot_state; display renders at C7 R1
#define BT_DOT_OFF  0
#define BT_DOT_SLOW 1
#define BT_DOT_FAST 2
volatile uint8_t bt_dot_state = BT_DOT_OFF;
volatile bool bt_session_forced = false;  // set by LEFT-hold boot gesture; enables BLE for session regardless of bt_enabled
volatile bool display_activity_enabled = true;
volatile bool radio_activity_enabled = true;
volatile bool radio_driver_ready = false;
volatile bool hall_activity_enabled = true;

#ifdef WIFI_ENABLED
volatile bool web_cfg_service_enabled = false;
volatile bool web_cfg_pending_save = false;
volatile bool web_cfg_radio_reinit_required = false;
volatile uint32_t web_cfg_req_total = 0;
volatile uint32_t web_cfg_req_ok = 0;
volatile uint32_t web_cfg_req_err = 0;
volatile uint8_t web_cfg_debug_mode = 1; // 0=off, 1=some, 2=full
volatile uint32_t web_cfg_ap_startup_timeout_ms = 120000; // 0 disables timeout
String web_cfg_last_err = "";
#endif

#include "../Common/ConfigServiceEngine.h"

/*
** Defines
*/
#define ADS1115_ADDRESS 0x48
#define DISPLAY_ADDRESS 0x70

//I2C Pins
#define P_I2C_SCL 1
#define P_I2C_SDA 2
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
#define P_MOT 0

// V2.5-Evo: GPS UART pins for TX (BN-220 on Serial1).
// Same numeric assignment as RX (P_U1_RX=18, P_U1_TX=19) — shared physical
// convention across TX and RX boards. Used by initTxGPS() and getTxGPSLoop()
// in Tx/GPS.ino. No UART mux on TX (unlike RX), so Serial1 talks to the
// GPS directly.
#define P_U1_RX 18
#define P_U1_TX 19

// Magnet sensor (DRV5032FADBZR, push-pull, digital) — LOW = magnet present, HIGH = no magnet
#define P_MAG 9
//ADC Pins (ADS1115 channel numbers, not GPIO)
#define P_HALL_THR  0
#define P_HALL_TOG  1
#define P_UBAT_MEAS 3
#define P_CHGSTAT   2

//Debug options — comment out for release builds
//#define DEBUG_RX

#if defined DEBUG_RX
   #define rxprint(x)    Serial.print(x)
   #define rxprintln(x)  Serial.println(x)
#else
   #define rxprint(x)
   #define rxprintln(x)
#endif

#define LET_A 10
#define LET_B 11
#define LET_C 12
#define LET_D 13
#define LET_E 14
#define LET_F 15
#define LET_H 16
#define LET_I 17
#define LET_L 18
#define LET_P 19
#define LET_T 20
#define LET_U 21
#define LET_V 22
#define LET_X 23
#define LET_Y 24
#define BLANK 25
#define DASH 26
#define LOWER_CELSIUS 27
#define TGT 28
#define TLT 29
#define LET_R 30
#define LET_N 31
#define LET_S 32
#define LET_M 33

                    //0                 //1                 //2                 //3                 //4
uint8_t num0[34][3]{ {0x1F, 0x11, 0x1F}, {0x00, 0x00, 0x1F}, {0x17, 0x15, 0x1D}, {0x11, 0x15, 0x1F}, {0x1C, 0x04, 0x1F},
                    //5                 //6                 //7                 //8                 //9
                    {0x1D, 0x15, 0x17}, {0x1F, 0x15, 0x17}, {0x10, 0x10, 0x1F}, {0x1F, 0x15, 0x1F}, {0x1D, 0x15, 0x1F},
                    //A                 //B                 //C                 //D                 //E                 //F
                    {0x1F, 0x14, 0x1F}, {0x1F, 0x15, 0x0A}, {0x1F, 0x11, 0x11}, {0x1F, 0x11, 0x0E}, {0x1F, 0x15, 0x11}, {0x1F, 0x14, 0x10},
                    //H                 //I                 //L                 //P                 //T
                    {0x1F, 0x04, 0x1F}, {0x11, 0x1F, 0x11}, {0x1F, 0x01, 0x01}, {0x1F, 0x14, 0x1C}, {0x10, 0x1F, 0x10},
                    //U                 //V                 //X                 //Y                 //Blank
                    {0x1F, 0x01, 0x1F}, {0x1E, 0x01, 0x1E}, {0x1B, 0x04, 0x1B}, {0x1C, 0x07, 0x1C}, {0x00, 0x00, 0x00},
                    //Dash              //LOWER_CELSIUS     //TGT (>)           //TLT(<)
                    {0x04, 0x04, 0x04}, {0x08, 0x07, 0x05}, {0x11, 0x0A, 0x04}, {0x04, 0x0A, 0x11},
                    // V2.5-Evo - 2026-10-01 - index 30 is a LOWERCASE r (owner's call: one r everywhere).
                    // It was {0x1F, 0x14, 0x13} = uppercase R, which is byte-for-byte the F glyph plus two
                    // bottom-right pixels - so "R0" read as "F0 with a small line" and the Return-To-Me
                    // confirmation was taken for a Follow-Me mode. Lowercase r is half height with one
                    // arch and cannot be confused with F. scrollFarStep() builds the >=100 m distance word
                    // from this glyph, so that readout now shows "FAr".
                    //r (30)              //N (31)              //S (32)              //M (33)
                    {0x07, 0x04, 0x04}, {0x1F, 0x10, 0x1F}, {0x1D, 0x15, 0x17}, {0x1F, 0x18, 0x1F}
                    };

uint8_t row_mapper[] = { 8,9,7,5,6,3,4,2,0,1 };
uint8_t col_mapper[] = { 1,2,4,3,5,6,7 };
//uint8_t row_mapper[] = { 1,0,2,4,3,6,5,7,9,8 };
//uint8_t col_mapper[] = { 7,6,4,5,3,2,1 };

#include "../Common/RadioCommon.h"
#include "../Common/SPIFFSEngine.h"
#ifdef WIFI_ENABLED
#include "../Common/WebConfigEngine.h"
#endif
#include "../Common/SystemCommon.h"

#ifdef WIFI_ENABLED
void webCfgNotifyTxUnlocked();
#else
inline void webCfgNotifyTxUnlocked() {}  // No-op stub when WiFi disabled
#endif
