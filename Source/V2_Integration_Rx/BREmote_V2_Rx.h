// V3 - 2026-04-22 - Added gps_chip_type field to confStruct (GPS module selector); sizeof 108→112; updated defaultConf
// V3 - 2026-04-22 - Added Phase A GPS anti-spoofing params to confStruct; sizeof 112→128; updated defaultConf
// V3 - 2026-04-24 - Added rx_tx_gps_lat/lng/timestamp globals for 0xF3 meta-packet reception
// V3 - 2026-04-24 - Added Phase B GPS handshake params to confStruct; sizeof 128→136; updated defaultConf

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

#define SW_VERSION 3
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

    uint16_t dummy_delete_me;

    //UART config
    uint16_t data_src; //0: off, 1:analog, 2: VESC UART

    // GPS features related flags
    uint16_t gps_en;         // GPS runtime enable flag (0=disabled, 1=enabled)
    uint16_t followme_mode;  // Follow-me runtime mode flag (0=disabled, 1=behind, 2=near_right, 3=near_left)
    uint16_t kalman_en;      // Kalman filter runtime enable flag (0=disabled, 1=enabled)

    //Follow-me
    float boogie_vmax_in_followme_kmh; // Maximum boogie speed in follow-me mode (km/h)
    float min_dist_m; // minimum allowed distance to the foiler
    float followme_smoothing_band_m; // smoothing band above min distance
    float foiler_low_speed_kmh; // low-speed threshold for safety stop (hysteresis)
    float zone_angle_enter_deg; // Half-angle for zone entry (deg)
    float zone_angle_exit_deg;  // Half-angle for zone exit (deg)
    float near_diag_offset_deg; // Offset from behind for NEAR modes (deg)
    
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
    // V3 - 2026-04-22 - GPS CHIP TYPE SELECTOR
    //
    // !!! IMPORTANT: Adding this field changes sizeof(confStruct)  !!!
    // !!! from 108 bytes to 112 bytes.                             !!!
    // !!!                                                          !!!
    // !!! On first V3 boot, the SPIFFS config will fail the size  !!!
    // !!! check and ALL RX SETTINGS WILL RESET TO DEFAULTS.       !!!
    // !!!                                                          !!!
    // !!! After flashing V3 firmware for the first time, you must !!!
    // !!!   1) Re-pair TX and RX                                   !!!
    // !!!   2) Re-enter all config settings via web UI             !!!
    // !!!   3) Re-calibrate compass (run 'runcal' command)         !!!
    // ============================================================
    uint16_t gps_chip_type;  // 0=BN-220, 1=BN-880+compass (default), 2=M10 no compass, 3=M10+compass; range 0-3

    // ============================================================
    // V3 - 2026-04-22 - PHASE A GPS ANTI-SPOOFING PARAMETERS
    //
    // These four parameters control the always-on Phase A anti-
    // spoofing filter in GPS.ino. A reading is rejected if ANY
    // check fails. After gps_suspect_threshold consecutive
    // rejections, gps_rejected is set and RTM arming is blocked.
    //
    // !!! Adding these fields changes sizeof(confStruct) 112→128. !!!
    // !!! On first V3.1 boot, SPIFFS resets ALL settings to       !!!
    // !!! defaults. After flashing: re-pair TX/RX, re-configure   !!!
    // !!! all settings via web UI, re-run runcal.                 !!!
    // ============================================================
    float    gps_max_hdop;            // Max HDOP for a valid fix; range 0.5-5.0; default 2.0; dimensionless
    float    gps_max_accel_g;         // Max implied acceleration between readings; range 1.0-10.0G; default 3.0G
    float    gps_max_jump_kmh;        // Max position-implied speed (teleport check); range 50-500 km/h; default 200
    uint16_t gps_suspect_threshold;   // Consecutive failures before GPS marked rejected; range 1-10; default 3

    // ============================================================
    // V3 - 2026-04-24 - PHASE B GPS HANDSHAKE ANTI-SPOOFING PARAMETERS
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
};
static_assert(sizeof(confStruct) == 136, "confStruct size mismatch — expected 136 bytes (V3.2). Update this assert and SPIFFS migration logic if you change the struct.");  // V3 fix (N-1): pinned to exact size; catches both shrinkage and unexpected growth. 112→128 when Phase A added 2026-04-22; 128→136 when Phase B added 2026-04-24.
confStruct usrConf;
  //The orginal confs were:  ##// confStruct defaultConf = {SW_VERSION, 1, 0, 0, 50, 0, 0, 1500, 2000, 1500, 2000, 1000, 10, 0, 1, 0, 0, 0, 0, 0, 25.0f, 10.0f, 10.0f, 5.0f, 35.0f, 45.0f, 45.0f, 0.0095554f, 0.0, 1000, 1, 0, {0, 0, 0}, {0, 0, 0}, {'1','2','3','4','5','6','7','8'}};
  // V3 default configuration — tuned for monterman hardware
confStruct defaultConf = {SW_VERSION, 2, 20, 1, 50, 0, 0, 1000, 2000, 1000, 2000, 1000, 10, 0, 1, 0, 2, 1, 2, 1, 25.0f, 10.0f, 10.0f, 8.0f, 35.0f, 45.0f, 45.0f, 0.0095554f, 0.0f, 1000, 0, 1, {0x46, 0xC9, 0xE0}, {0x46, 0xCB, 0xCC}, {'1','2','3','4','5','6','7','8'},
  // V3 - 2026-04-22 - Compass calibration fields (previously implicit zeros).
  // Made explicit here so gps_chip_type can follow. Safe neutral values:
  // offsets=0 (no bias), scales=1.0f (unity gain = no correction applied).
  0, 0,      // mag_offset_x, mag_offset_y (no compass bias correction by default)
  1.0f, 1.0f, // mag_scale_x, mag_scale_y (unity gain — run 'runcal' to calibrate)
  // V3 - 2026-04-22 - GPS chip type: 1 = BN-880 (GPS+compass). RX default.
  1,          // gps_chip_type (1 = BN-880 + compass; run 'runcal' after first boot)
  // V3 - 2026-04-22 - Phase A GPS anti-spoofing defaults (see CLAUDE.md Section 11)
  2.0f,       // gps_max_hdop:           max HDOP for valid reading (range 0.5-5.0)
  3.0f,       // gps_max_accel_g:        max implied acceleration (range 1.0-10.0 G)
  200.0f,     // gps_max_jump_kmh:       max teleport-implied speed (range 50-500 km/h)
  3,          // gps_suspect_threshold:  consecutive failures before GPS rejected (range 1-10)
  // V3 - 2026-04-24 - Phase B GPS handshake anti-spoofing defaults (see CLAUDE.md Section 11)
  500.0f,     // gps_max_pair_dist_m:    max TX-RX pairing distance (range 50-2000 m)
  50.0f       // gps_max_speed_diff_kmh: max TX-RX speed difference (range 10-200 km/h)
};
  /// these equal to:  {"version":3,"radio_preset":2,"rf_power":20,"steering_type":1,"steering_influence":50,"steering_inverted":0,"trim":0,"pwm0_min":1000,"pwm0_max":2000,"pwm1_min":1000,"pwm1_max":2000,"failsafe_time":1000,"foil_num_cells":10,"bms_det_active":0,"wet_det_active":1,"dummy_delete_me":0,"data_src":2,"gps_en":1,"followme_mode":2,"kalman_en":1,"boogie_vmax_in_followme_kmh":25,"min_dist_m":10,"followme_smoothing_band_m":10,"foiler_low_speed_kmh":8,"zone_angle_enter_deg":35,"zone_angle_exit_deg":45,"near_diag_offset_deg":45,"ubat_cal":0.0095554,"ubat_offset":0,"tx_gps_stale_timeout_ms":1000,"logger_en":0,"paired":1,"own_address":"46:C9:E0","dest_address":"46:CB:CC","wifi_password":"12345678","mag_offset_x":0,"mag_offset_y":0,"mag_scale_x":1.0,"mag_scale_y":1.0,"gps_chip_type":1,"gps_max_hdop":2.0,"gps_max_accel_g":3.0,"gps_max_jump_kmh":200.0,"gps_suspect_threshold":3,"gps_max_pair_dist_m":500.0,"gps_max_speed_diff_kmh":50.0}
  ///

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
// V3 - 2026-04-24 - TX GPS COORDINATES (received via 0xF3 meta-packet)
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

#include "../Common/SPIFFSEngine.h"

// --- Global VESC Logger Struct ---
struct vesc_struct {
  int16_t fetTemp = 0;
  int32_t motCur = 0;
  int32_t batCur = 0;
  int16_t duty = 0;
  int32_t erpm = 0;
  int16_t batVolt = 0;
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
};
#define ENABLE_WEB_LOG_DOWNLOAD // Enable log download endpoints

#ifdef WIFI_ENABLED
#include "../Common/WebConfigEngine.h"
#endif

#ifdef WIFI_ENABLED
void webCfgNotifyRxConnected();
#else
inline void webCfgNotifyRxConnected() {}  // No-op stub when WiFi disabled
#endif

//Telemetry to send, MUST BE 8-bit!!
struct __attribute__((packed)) TelemetryPacket {
    uint8_t foil_bat = 0xFF;    // index 0 — battery % 0-100
    uint8_t foil_temp = 0xFF;   // index 1 — FET temp degC
    uint8_t foil_speed = 0xFF;  // index 2 — speed km/h
    uint8_t error_code = 0;     // index 3 — fault flags
    uint8_t foil_power = 0xFF;  // index 4 — power (watts/50); 0xFF = not available
    uint8_t link_quality = 0;   // index 5
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

volatile unsigned long get_vesc_timer = 0;
volatile unsigned long last_uart_packet = 0;

volatile uint8_t bind_pin_state = 0;

float fbatVolt = 0.0;
float noload_offset = 0.0;
uint8_t bc_arr[101];
uint8_t percent_last_val = 0xFF;
uint8_t percent_last_thr = 1;
unsigned long percent_last_thr_change = 0;

// V3: ERPM added to VESC selective-get mask; payload length is 23 bytes
#define VESC_MORE_VALUES
#ifdef VESC_MORE_VALUES
  #define VESC_PACK_LEN 23
  uint8_t vescRelayBuffer[30];
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

//Debug options
#define DEBUG_RX
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