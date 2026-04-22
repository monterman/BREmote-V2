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

// Uncomment the line below to enable WiFi AP configuration mode
#define WIFI_ENABLED

#ifdef WIFI_ENABLED
#include <WiFi.h>
#include <WebServer.h>
#endif

#define SW_VERSION 3
const char* CONF_FILE_PATH = "/data.txt";

//#define DELETE_SPIFFS_CONF_AT_STARTUP 1

/*
** Structs
*/
// NOTE: Not packed — sizeof is 92 (V3, was 80 in V2). Float forces 4-byte struct alignment.
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
    uint16_t thr_expo1; //currently unused

    uint16_t steer_expo; //currently unused
    uint16_t steer_expo1; //currently unused

    //System parameters
    float ubat_cal; //ADC to volt cal for bat meas, default 0.000185662

    // GPS features related flags
    uint16_t gps_en;           // GPS runtime enable flag (0=disabled, 1=enabled)
    uint16_t followme_mode; // Follow-me runtime mode flag (0=disabled, 1=behind, 2=near_right, 3=near_left)
    uint16_t kalman_en;        // Kalman filter runtime enable flag (0=disabled, 1=enabled)
    uint16_t speed_src;   //0: GPS RX kmh, 1: GPS RX knots, 2: GPS TX kmh, 3: GPS TX knots
    
    //Follow-me timeouts (transmitted to RX via META)
    uint16_t tx_gps_stale_timeout_ms; // TX GPS data stale timeout (ms)

    //Comms
    uint16_t paired;
    uint8_t own_address[3];
    uint8_t dest_address[3];
    char wifi_password[8];      // WPA2 AP password, exactly 8 chars (no null terminator)
    uint16_t dynamic_power_start;  // 10-100, starting cap for mode 2 (default 85)
    uint16_t dynamic_power_step; // 1-25, step size per toggle press in mode 2 (default 5)
};

static_assert(sizeof(confStruct) >= 80, "confStruct shrunk below V2 baseline");
confStruct usrConf;
confStruct defaultConf = {SW_VERSION, 1, 0, 0, 100, 0, 0, 0, 0, 0, 500, 30, 500, 5000, 2000, 100, 1000, 10, 2000, 0, 0, 10, 0, 1, 50, 0, 50, 0, 0.000185662f, 0, 0, 0, 0, 1000, 0, {0, 0, 0}, {0, 0, 0}, {'1','2','3','4','5','6','7','8'}, 85, 5};


//Telemetry to receive, MUST BE 8-bit!!
struct __attribute__((packed)) TelemetryPacket {
    uint8_t foil_bat = 0xFF;    // index 0 — battery % 0-100
    uint8_t foil_temp = 0xFF;   // index 1 — FET temp degC
    uint8_t foil_speed = 0xFF;  // index 2 — speed km/h
    uint8_t error_code = 0;     // index 3 — fault flags
    // SCALE=watts/50  DECODE=foil_power*50  range 0-12750W at 50W resolution
    uint8_t foil_power = 0xFF;  // index 4 — power (watts/50); 0xFF = not available
    //This must be the last entry
    uint8_t link_quality = 0;   // index 5 (was 4)
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

//TaskHandle_t triggeredReceiveHandle = NULL;
//TaskHandle_t checkConnStatusHandle = NULL;

extern TaskHandle_t loopTaskHandle;

/*
** Variables
*/
uint16_t displayBuffer[8];
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

//-1 = left, 1 = right input
volatile int tog_input = 0;

volatile float int_bat_volt = 0.0;

volatile bool mot_active = 0;
volatile bool system_locked = 1;

// Display mode cycle: 0=temp, 1=speed, 2=power, 3=vesc bat, 4=throttle, 5=int bat
#define DISPLAY_MODE_TEMP    0
#define DISPLAY_MODE_SPEED   1
#define DISPLAY_MODE_POWER   2
#define DISPLAY_MODE_BAT     3
#define DISPLAY_MODE_THR     4
#define DISPLAY_MODE_INTBAT  5
#define DISPLAY_MODE_COUNT   6
volatile uint8_t display_mode = 0;

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

//ADC Pins
#define P_HALL_THR  0
#define P_HALL_TOG  1
#define P_UBAT_MEAS 3 
#define P_CHGSTAT   2

//Debug options
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
    
                    //0                 //1                 //2                 //3                 //4
uint8_t num0[30][3]{ {0x1F, 0x11, 0x1F}, {0x00, 0x00, 0x1F}, {0x17, 0x15, 0x1D}, {0x11, 0x15, 0x1F}, {0x1C, 0x04, 0x1F},
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
