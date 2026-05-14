// V2.5-Evo - 2026-05-13 - SW48: DISP_LOCK/UNLOCK macros; mutex all bare display callers outside renderOperationalDisplay/updateBargraphs
// V2.5-Evo - 2026-05-13 - SW46: DISPLAY_MODE order — Temp(0)/Thr(1)/Speed(2)/Power(3)/Bat(4)/IntBat(5); THR centre, LEFT=Temp, RIGHT=Speed
// V2.5-Evo - 2026-05-13 - SW33: GPIO 9 repurposed as P_MAG digital Hall sensor (DRV5032FADBZR); removed from serialOff OUTPUT-LOW block; mag_seen_high boot guard added
// V2.5-Evo - 2026-05-13 - SW33b: BT dot test (C7 R1) driven by P_MAG Hall sensor; bt_dot_state + BT_DOT_* defines added
// V2.5-Evo - 2026-04-21 - Added TinyGPS++ include, gps_tx + tx_gps_speed globals, and P_U1_RX/P_U1_TX pin defines for TX GPS (BN-220 on Serial1)
// V2.5-Evo - 2026-04-22 - Fixed speed_src/volatile comments; defaults gps_en=0,speed_src=0; added gps_max_hdop field (HDOP*100, tail-padding slot, sizeof stays 92)
// V2.5-Evo - 2026-04-22 - Added gps_chip_type field (GPS module selector: 0=BN-220, 2=M10); sizeof 92→96
// V2.5-Evo - 2026-04-25 - P7: Added RTM meta-packet queue globals (rtm_meta_type/value/count) and RTM throttle cap (rtm_thr_cap_tx, rtm_tx_active)
// V2.5-Evo - 2026-04-27 - P8: Added rtm_display_mode, fm_warn_distance_m, rtm_steer_exit_on_input to confStruct; TelemetryPacket adds rtm_distance at index 5; rtm_max_runtime_s default 120→0
// V2.5-Evo - 2026-04-27 - P8.1: Added fm_arm_window_s to confStruct; FM redesigned as arm/disarm toggle with mode memory; sizeof 124→128
// V2.5-Evo - 2026-04-28 - P9: Added dist_unit (fills 2-byte tail padding; sizeof stays 128); rtm_arm_dist_m RAM global
// V2.5-Evo - 2026-04-29 - Sleep: added sleep_timeout_s to confStruct; SW_VERSION 25→26
// V2.5-Evo - 2026-05-01 - Release: DEBUG_RX commented out for production build
// V2.5-Evo - 2026-05-01 - thr_expo1 repurposed as fm_display_mode (FM digit zone data selector, 1-4)
// V2.5-Evo - 2026-05-02 - Added displayMutex SemaphoreHandle_t (Core 0/Core 1 displayBuffer race fix)
// V2.5-Evo - 2026-05-13 - SW32 M3: rtm_meta_type/value/count + rtm_thr_cap_tx + rtm_tx_active changed volatile→std::atomic<T>; release/acquire ordering in queue/consumer
// V2.5-Evo - 2026-05-13 - SW32: default display_mode changed 0→DISPLAY_MODE_THR (throttle % as boot display; field test feedback)
// V2.5-Evo - 2026-05-09 - Bundle 9-Final: Added USB CDC On Boot compile-time guard

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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <RadioLib.h> //V7.1.2 jan gromes
#include <Wire.h>
#include <Adafruit_ADS1X15.h> //V2.5.0 adafruit
#include <Ticker.h>
#include "esp_task_wdt.h"
#include "FS.h"
#include "SPIFFS.h"
#include "mbedtls/base64.h"

// --- V3: TX GPS support (BN-220 on Serial1) ---
// Added for Priority 1: read TX GPS speed and drive the SP display mode
// when usrConf.speed_src selects a TX-GPS option (2=km/h, 3=knots, 5=mph).
// Library: TinyGPSPlus 1.0.3 by Mikal Hart (same version used on RX).
#include <TinyGPS++.h>

// Uncomment the line below to enable WiFi AP configuration mode
#define WIFI_ENABLED

#ifdef WIFI_ENABLED
#include <WiFi.h>
#include <WebServer.h>
#endif

#define SW_VERSION 26  // V2.5-Evo - 2026-04-29: sleep_timeout_s added to confStruct; first
                       // flash resets all TX SPIFFS settings to defaults — re-configure via WebUI
const char* CONF_FILE_PATH = "/data.txt";

//#define DELETE_SPIFFS_CONF_AT_STARTUP 1

// V2.5-Evo - 2026-04-28 - P9: Compact 3×7 font entry used by showFullScreenMessage() in Display.ino.
// Defined here so Arduino IDE's auto-prototype generator sees it before emitting the fc3x7GetChar prototype.
struct Fc3x7Entry { uint8_t col[3]; };

/*
** Structs
*/
// NOTE: Not packed — sizeof is 96 (V3, was 92 before gps_chip_type, 80 in V2). Float forces 4-byte struct alignment.
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
    uint16_t fm_display_mode;  // FM digit zone display: 1=TX speed (default), 2=distance to buggy,
                               // 3=buggy speed (RX telemetry), 4=throttle %; range 1-4

    uint16_t steer_expo; //currently unused
    uint16_t steer_expo1; //currently unused

    //System parameters
    float ubat_cal; //ADC to volt cal for bat meas, default 0.000185662

    // GPS features related flags
    uint16_t gps_en;           // GPS runtime enable flag (0=disabled, 1=enabled)
    uint16_t followme_mode; // Follow-me runtime mode flag (0=disabled, 1=behind, 2=near_right, 3=near_left)
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
    uint16_t rtm_hold_duration_s;      // LEFT hold time to arm RTM; 4-10 s; default 5
    uint16_t rtm_arm_window_s;         // Window to engage throttle after arming; 5-30 s; default 10
    uint16_t rtm_double_squeeze_en;    // Require double-squeeze (1) or 500ms hold (0); default 1
    uint16_t rtm_throttle_start_pct;   // Initial throttle cap when RTM engages; 10-50 %; default 30
    uint16_t rtm_throttle_max_pct;     // Max throttle cap after ramp; 30-90 %; default 70
    uint16_t rtm_ramp_duration_s;      // Time to ramp throttle start→max; 2-15 s; default 5
    uint16_t rtm_disengage_distance_m; // Distance from TX at which RTM disengages (hard stop); 3-20 m; default 10
    uint16_t rtm_max_runtime_s;        // Maximum continuous RTM runtime; 30-300 s; default 120
    uint16_t rtm_gps_timeout_ms;       // TX GPS loss timeout before safety stop; 500-3000 ms; default 2000
    uint16_t fm_hold_duration_s;       // RIGHT hold time for FM mode cycle; 4-10 s; default 5
    uint16_t fm_override_enabled;      // Allow TX to override RX follow-me mode; 0=off, 1=on; default 1

    // ============================================================
    // V2.5-Evo - 2026-04-27 - PRIORITY 8: DISPLAY, GESTURE & UX OVERHAUL
    //
    // 3 new uint16_t fields — sizeof grows 120→124 (118 data + 6 = 124; 124 % 4 == 0, no tail padding).
    // First flash of P8 firmware resets all TX settings to defaults.
    // ============================================================
    uint16_t rtm_display_mode;         // RTM/FM active info display: 0=distance(default), 1=speed, 2=alternating 2.5s each
    uint16_t fm_warn_distance_m;       // TX-RX distance to trigger FM proximity warning vibration; 50-1000m; default 150
    uint16_t rtm_steer_exit_on_input;  // 1=any steering input exits RTM (default); 0=blend/steering correction only

    // ============================================================
    // V2.5-Evo - 2026-04-27 - PRIORITY 8.1: FM UX REDESIGN
    //
    // 1 new uint16_t field — sizeof grows 124→128 (126 data + 2 tail padding; 126%4=2).
    // First flash of P8.1 firmware resets all TX settings to defaults.
    // ============================================================
    uint16_t fm_arm_window_s;          // FM auto-disarms after this many seconds with no throttle input; 10-60s; default 30

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
};

static_assert(sizeof(confStruct) == 132, "confStruct size mismatch — expected 132 bytes (V2.5-Evo sleep_timeout_s). Update this assert if you change the struct.");  // pinned to exact size; catches both shrinkage and unexpected growth
confStruct usrConf;
confStruct defaultConf = {  // V3 default configuration — tuned for monterman hardware
  SW_VERSION,    // version (26)
  2,             // radio_preset (US 915MHz)
  20,            // rf_power (20)
  1,             // cal_ok
  100,           // cal_offset
  15195,         // thr_idle
  11909,         // thr_pull
  12310,         // tog_left (swapped vs factory default — matches monterman hardware)
  13806,         // tog_mid
  14908,         // tog_right (swapped vs factory default — matches monterman hardware)
  500,           // tog_deadzone
  30,            // tog_diff
  200,           // tog_block_time   (wait to finish steering before Dynamic Throttle /was 500 5 secs)
  3500,          // trig_unlock_timeout
  2000,          // lock_waittime
  100,           // gear_change_waittime
  1000,          // gear_display_time
  10,            // menu_timeout
  2000,          // err_delete_time
  0,             // no_lock
  2,             // throttle_mode
  6,             // max_gears
  0,             // startgear
  1,             // steer_enabled
  100,           // thr_expo
  1,             // fm_display_mode (1=TX speed default; old configs had 0 here — ConfigService min:1 auto-corrects to 1)
  50,            // steer_expo
  0,             // steer_expo1
  0.000185662f,  // ubat_cal
  0,             // gps_en — V3: default off; opt in via web config (claiming Serial1 on every device is wrong)
  1,             // followme_mode
  1,             // kalman_en
  0,             // speed_src — V3: default RX km/h; TX GPS source requires explicit opt-in
  2000,          // tx_gps_stale_timeout_ms
  1,             // paired
  {0x46, 0xCB, 0xCC}, // own_address (Hex formatted)
  {0x46, 0xC9, 0xE0}, // dest_address (Hex formatted)
  {'1','2','3','4','5','6','7','8'}, // wifi_password
  85,            // dynamic_power_start
  5,             // dynamic_power_step
  // V2.5-Evo - 2026-04-22 - default HDOP gate: 200 = HDOP 2.0. Fits in former tail-padding bytes.
  200,           // gps_max_hdop (200 = HDOP 2.0; existing configs read 0 here → validation rejects → defaults written)
  // V2.5-Evo - 2026-04-22 - GPS chip type: 0 = BN-220 (9600→115200, 5Hz). TX only supports 0 and 2.
  0,             // gps_chip_type (0=BN-220 default; old configs → decodedLen check fails → defaults written)
  // V2.5-Evo - 2026-04-25 - Priority 7 RTM/FM defaults
  1,    // rtm_enabled
  5,    // rtm_hold_duration_s
  10,   // rtm_arm_window_s
  1,    // rtm_double_squeeze_en
  30,   // rtm_throttle_start_pct
  70,   // rtm_throttle_max_pct
  5,    // rtm_ramp_duration_s
  10,   // rtm_disengage_distance_m
  0,    // rtm_max_runtime_s (0=disabled — safety gates handle all real scenarios; P8 changed from 120)
  2000, // rtm_gps_timeout_ms
  5,    // fm_hold_duration_s
  1,    // fm_override_enabled
  // V2.5-Evo - 2026-04-27 - Priority 8 UX overhaul defaults
  0,    // rtm_display_mode (0=distance; set 1 for speed, 2 for alternating)
  150,  // fm_warn_distance_m (150m FM proximity warning threshold)
  1,    // rtm_steer_exit_on_input (1=steering exits RTM; 0=blend only)
  // V2.5-Evo - 2026-04-27 - Priority 8.1 FM UX redesign defaults
  30,   // fm_arm_window_s (30s before auto-disarm if no throttle input)
  0,    // dist_unit (0 = Metres)
  // V2.5-Evo - 2026-04-29 - sleep timeout default
  300,  // sleep_timeout_s — 300s = 5 minutes; set to 0 to disable
};


//Telemetry to receive, MUST BE 8-bit!!
// V2.5-Evo - 2026-04-27 - P8: Added rtm_distance at index 5; link_quality moved to index 6 (must remain last).
// Encoding: 0-99 = tenths of meter (0.0–9.9 m), 100-254 = meters offset (value-90 = actual m, so 100=10m, 199=109m), 255 = N/A.
struct __attribute__((packed)) TelemetryPacket {
    uint8_t foil_bat = 0xFF;      // index 0 — battery % 0-100
    uint8_t foil_temp = 0xFF;     // index 1 — FET temp degC
    uint8_t foil_speed = 0xFF;    // index 2 — speed km/h
    uint8_t error_code = 0;       // index 3 — fault flags
    // SCALE=watts/50  DECODE=foil_power*50  range 0-12750W at 50W resolution
    uint8_t foil_power = 0xFF;    // index 4 — power (watts/50); 0xFF = not available
    uint8_t rtm_distance = 0xFF;  // index 5 — RX→TX distance during RTM/FM; see encoding above; 0xFF = N/A
    //This must be the last entry
    uint8_t link_quality = 0;     // index 6 (must be last)
} telemetry;

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

// --- V3: TX GPS globals ---
// gps_tx   : TinyGPS++ parser instance fed by Serial1 (BN-220).
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
// --- End V3: TX GPS globals ---

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

volatile int gear = 0;
volatile uint8_t max_power_cap = 85;  // Runtime cap for throttle_mode 2

volatile uint8_t thr_scaled = 0;
volatile uint8_t tog_scaled = 0;
volatile uint8_t steer_scaled = 0;

volatile uint8_t thr_sent = 0;   // Post-expo+gear throttle actually sent over radio
volatile uint8_t steer_sent = 0; // Steering value actually sent over radio

// V2.5-Evo - 2026-04-25 - P7 RTM meta-packet burst queue.
// V2.5-Evo - 2026-05-13 - SW32 M3: changed volatile→std::atomic<T>.
// Loop task (Core 1) writes type/value with memory_order_relaxed, then stores count
// with memory_order_release. sendData task (Core 0) loads count with memory_order_acquire
// before reading type/value. volatile prevented compiler caching but not CPU store-buffer
// reordering; std::atomic release/acquire prevents Core 0 from observing count>0
// while type/value are still stale in Core 1's store buffer.
std::atomic<uint8_t> rtm_meta_type  {0};    // 0xF1=RTM state, 0xF2=FM override
std::atomic<uint8_t> rtm_meta_value {0};    // for 0xF1: 0=inactive 1=active; for 0xF2: 0-3 FM mode
std::atomic<uint8_t> rtm_meta_count {0};    // bursts remaining; 0 = idle (value is always 0 or 3)

// V2.5-Evo - 2026-04-25 - P7 RTM throttle cap.
// V2.5-Evo - 2026-05-13 - SW32 M3: changed volatile→std::atomic<T>.
// Written by loop task (Core 1) via RTMState.ino; read by sendData (Core 0) via calcFinalThrottle().
// 255 = no cap (RTM not active). During RTM ACTIVE, set to the ramped cap value
// (30-70% of 255). Applied in calcFinalThrottle(). RTM can only subtract from
// user throttle — never add. Creator safety philosophy enforced here.
std::atomic<uint8_t> rtm_thr_cap_tx {255};
std::atomic<bool>    rtm_tx_active  {false};

// V2.5-Evo - 2026-04-28 - P9 S4: RTM arm distance captured at engage moment.
// Used by R5 proximity bar to set the 100% reference distance.
// RAM only — never written to SPIFFS. Reset to 0.0f when RTM disengages.
float rtm_arm_dist_m = 0.0f;

//-1 = left, 1 = right input
volatile int tog_input = 0;

volatile float int_bat_volt = 0.0;

volatile bool mot_active = 0;
volatile bool system_locked = 1;

// V2.5-Evo - 2026-05-13 - SW46: THR at centre(1) — LEFT=Temp(0), RIGHT=Speed(2)→Power(3)→Bat(4)→IntBat(5)→wrap Temp.
// All switch() cases use named constants — only these #defines change.
// display mode cycle: 0=temp, 1=throttle, 2=speed, 3=power, 4=vesc bat, 5=int bat
#define DISPLAY_MODE_TEMP    0
#define DISPLAY_MODE_THR     1
#define DISPLAY_MODE_SPEED   2
#define DISPLAY_MODE_POWER   3
#define DISPLAY_MODE_BAT     4
#define DISPLAY_MODE_INTBAT  5
#define DISPLAY_MODE_COUNT   6
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

// V3: GPS UART pins for TX (BN-220 on Serial1).
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
                    //R (30)              //N (31)              //S (32)              //M (33)
                    {0x1F, 0x14, 0x13}, {0x1F, 0x10, 0x1F}, {0x1D, 0x15, 0x17}, {0x1F, 0x18, 0x1F}
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
