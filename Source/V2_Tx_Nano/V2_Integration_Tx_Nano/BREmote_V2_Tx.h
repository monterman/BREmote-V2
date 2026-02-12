/*
** Includes
*/
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <RadioLib.h> //V7.1.2 jan gromes
#include <Wire.h>
#include <Adafruit_ADS1X15.h> //V2.5.0 adafruit
#include <Adafruit_AW9523.h> //V1.0.5, BusIO 1.17.0
#include <Ticker.h>
#include "FS.h"
#include "SPIFFS.h"
#include "mbedtls/base64.h"

#define SW_VERSION 1
const char* CONF_FILE_PATH = "/data.txt";

//#define DELETE_SPIFFS_CONF_AT_STARTUP 1

/*
** Structs
*/
struct confStruct {
    //Version
    uint16_t version;

    uint16_t radio_preset;
    int16_t rf_power;

    //Calibration of Tog&Thr
    uint16_t cal_ok;
    uint16_t cal_offset;

    uint16_t thr_idle;//15690
    uint16_t thr_pull;//12110

    uint16_t tog_left;//16310
    uint16_t tog_mid;//13080
    uint16_t tog_right;//9480

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
    uint16_t no_gear; //Gears can't be switched
    uint16_t max_gears; //Max user gears
    uint16_t startgear; //The gear that is set after poweron or unlock (0 to 9)
    uint16_t steer_enabled; //If steering feature is enable
    
    uint16_t thr_expo; //Exponential function, 50 = linear
    uint16_t thr_expo1; //currently unused

    uint16_t steer_expo; //currently unused
    uint16_t steer_expo1; //currently unused

    //System parameters
    float ubat_cal; //ADC to volt cal for bat meas 0.185662

    //Comms
    uint16_t paired;
    uint8_t own_address[3];
    uint8_t dest_address[3];
};

confStruct usrConf;
confStruct defaultConf = {SW_VERSION,1,0,0,100,0,0,0,0,0,250,50,500,5000,2000,100,1000,10,2000,0,0,5,0,0,50,50,50,50,0.0002784957,0,{0, 0, 0}, {0, 0, 0}};

#define LED_BRIGHTNESS 3

//Telemetry to receive, MUST BE 8-bit!!
struct __attribute__((packed)) TelemetryPacket {
    uint8_t foil_bat = 0xFF;
    uint8_t foil_temp = 0xFF;
    uint8_t foil_speed = 0xFF;
    uint8_t error_code = 0;

    //This must be the last entry
    uint8_t link_quality = 0;
} telemetry;

/*
** FreeROTS/Task handles
*/
const int maxTasks = 10;
TaskStatus_t taskStats[maxTasks];

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

volatile bool rfInterrupt = false;

volatile uint8_t local_link_quality = 0;

volatile unsigned long last_packet = 0;
volatile unsigned long num_sent_packets = 0;
volatile unsigned long num_rcv_packets = 0;

volatile uint8_t vesc_bat = 0;
volatile uint8_t vesc_temp = 0;
volatile uint8_t remote_sq = 0;
volatile uint8_t remote_error = 0;
volatile bool remote_error_blocked = 0;

// Buffer for received data
volatile uint8_t payload_buffer[10];                // Maximum payload size is 10 bytes
volatile uint8_t payload_received = 0;              // Length of received payload

// Pairing timeout in milliseconds
const unsigned long PAIRING_TIMEOUT = 5000;
const uint8_t MAX_ADDRESS_CONFLICTS = 5;            // Maximum number of address conflicts before giving up

//Ring Buffer for Hall Sensors
#define BUFFSZ 4
volatile uint16_t thr_raw[BUFFSZ];
volatile uint16_t tog_raw[BUFFSZ];
volatile uint16_t intbat_raw[BUFFSZ];

volatile int filter_count = 0;
volatile int bat_filter_count = 0;
volatile int last_channel = -1;

volatile int gear = 0;

volatile uint8_t thr_scaled = 0;
volatile uint8_t tog_scaled = 0;
volatile uint8_t steer_scaled = 0;

//-1 = left, 1 = right input
volatile int tog_input = 0;

volatile float int_bat_volt = 0.0;

volatile bool mot_active = 0;
volatile bool system_locked = 1;

volatile uint16_t toggle_blocked_counter = 0;
volatile bool toggle_blocked_by_steer = 0;
volatile int in_menu = 0;

volatile uint8_t sq_graph = 0;
uint8_t tlm_last_update = 0xFF;

volatile uint8_t last_known_temp_graph = 0;
volatile uint8_t last_known_bat_graph = 0;
volatile bool blink_bargraphs = 0;

volatile bool exitChargeScreen = 0;
/* 
** Defines
*/
#define ADS1115_ADDRESS 0x48

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

//LED Assignment:
//##FRONT##
//LL5 - LR5
//LL4 - LR4
//LL3 - LR3
//LL2 - LR2
//LL1 - LR1
//##BACK##

//AW9523 Pins
#define AP_LL1 13 //1-5
#define AP_LL2 12 //1-4
#define AP_LL3 7 //0-7
#define AP_LL4 6 //0-6
#define AP_LL5 5 //0-5

#define AP_LR1 11 //1-3
#define AP_LR2 0 //0-0
#define AP_LR3 1 //0-1
#define AP_LR4 2 //0-2
#define AP_LR5 3 //0-3

//Debug options
//#define DEBUG_RX

#if defined DEBUG_RX
   #define rxprint(x)    Serial.print(x)
   #define rxprintln(x)  Serial.println(x)
#else
   #define rxprint(x)
   #define rxprintln(x)
#endif