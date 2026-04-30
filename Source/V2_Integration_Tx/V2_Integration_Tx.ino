// V3 - 2026-04-25 - P7: Simplified getTxGPSLoop() gate to gps_en only; added runRtmLoop() call in loop()
// V3 - 2026-04-24 - Call initTxGPS() in setup() after applyConfigSettings() so GPS UART is ready on boot
// V3 - 2026-04-21 - Added getTxGPSLoop() call in loop() and forward declarations for TX GPS functions
// V3 - 2026-04-27 - P8: loop() calls renderRtmInfoDisplay() instead of renderOperationalDisplay() when rtm_tx_active
// V2.5-Evo - 2026-04-29 - Sleep: dual-condition auto-sleep — user idle (no throttle/toggle
//   above deadzone) OR RX silent; both use sleep_timeout_s; pocket-safe thresholds applied
#include "BREmote_V2_Tx.h"
// Function prototypes — forward declarations to resolve Arduino IDE ordering issues
// Init & Setup Functions
void initHardware();
void initStorage();
void checkCharger();
void initTasks();
void runBootSequence();
void applyConfigSettings();
void initWatchdog();
// Loop & System Functions
void serPrintPackets(bool json);
void runMenu();
void renderOperationalDisplay();
void showFullScreenMessage(const char* msg, uint16_t duration_ms);
void deepSleep();
void checkSerial();
// Subsystem Activity Toggles
void setRadioActivityEnabled(bool enabled);
bool isRadioActivityEnabled();
void setHallActivityEnabled(bool enabled);
bool isHallActivityEnabled();
void setDisplayActivityEnabled(bool enabled);
bool isDisplayActivityEnabled();
// Throttle Functions
uint8_t calcFinalThrottle();
void throttleInit();
void throttleReset();
void throttleAdjustCap(int direction);
bool throttleUsesGears();
bool throttleForceToggleBlock();
uint8_t throttleGetCapPercent();
// TX GPS Functions (defined in GPS.ino, declared here as GPS.ino is concatenated after this file)
void initTxGPS();
void getTxGPSLoop();
// RTM & FM State Machine Functions (defined in RTMState.ino)
void runRtmLoop();
void runFmLoop();
void setRtmArmed();
void cycleFmMode();
void cycleFmModeArmed();
bool isFmArmed();
uint8_t calcRtmThrottleCap();
// RTM/FM Active Display (defined in Display.ino)
void renderRtmInfoDisplay();
// Cross-Tab Subsystem Initializers
void startupRadio();
void startupDisplay();
void initSPIFFS();
void getConfFromSPIFFS();
void getBCFromSPIFFS();
void ICACHE_RAM_ATTR packetReceived(void);
// FreeRTOS Task Functions
void sendData(void *parameter);
void waitForTelemetry(void *parameter);
void updateBargraphs(void *parameter);
void measBufCalc(void *parameter);
void vibrationTask(void *parameter);   // haptic feedback task — drives vibration motor patterns
// -----------------------------------------------------------------------

SX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO, P_LORA_RST, P_LORA_BUSY);
Adafruit_ADS1115 ads;
//Ticker ticksrc; // Unused — replaced by FreeRTOS tasks
// V2.5-Evo - 2026-04-29 - Sleep: SLEEP_TIMEOUT_MS removed; timeout now read from
// usrConf.sleep_timeout_s (SPIFFS). 0 = disabled. Default 300s = 5 minutes.

// Tracks time of last intentional user input for inactivity sleep.
// Reset when throttle is pulled above noise floor or toggle moves off center.
// Pocket-safe: ADC noise and accidental contact below these thresholds are ignored.
static unsigned long last_user_input_ms  = 0;
static uint8_t       last_sleep_steer    = 127;  // last known steer_scaled; 127=centre

void setup()
{
  in_setup = true;
  enterSetup();

  initHardware();
  initStorage();
  
  // SURGICAL STRIKE: Deletes only the old webpage, leaves configs 100% safe.  //Coment out after it works fins and no new changes//
  //SPIFFS.remove("/index.html"); 

  checkCharger();
  initTasks();
  runBootSequence();
  applyConfigSettings();
  // V3 - 2026-04-24 - Call initTxGPS() on boot so GPS UART is ready before loop() starts polling
  initTxGPS();
  initWatchdog();

  exitSetup();
  in_setup = false;

// ... (rest of the setup function remains the same)

  if(system_locked)
  {
    setRadioActivityEnabled(false);
  }
  else
  {
#ifdef WIFI_ENABLED
    webCfgNotifyTxUnlocked();
#endif
  }

  if(config_version_error)
  {
    serialOff = false;
  }

  delay(100);
  if(serialOff)
  {
    Serial.end();
    digitalWrite(20, LOW); pinMode(20, OUTPUT);
    digitalWrite(21, LOW); pinMode(21, OUTPUT);
    digitalWrite(9, LOW); pinMode(9, OUTPUT);
  }
}

void loop()
{
  if(config_version_error)
  {
    scroll3Digits(LET_E, 5, LET_V, 200);
    checkSerial();
    return;
  }

#ifdef WIFI_ENABLED
  webCfgLoop();
#endif

  // V3 - 2026-04-25 - P7: Run GPS loop whenever gps_en=1, regardless of speed_src.
  // RTM mode needs TX GPS position for meta-packets even when speed display uses
  // an RX-side source (speed_src 0/1/4). Non-blocking — safe every 110ms tick.
  if (usrConf.gps_en)
  {
    getTxGPSLoop();
  }

  runMenu();
  if(in_menu > 0) in_menu--;

  // V3 - 2026-04-25 - P7: RTM state machine (arming, active, cooldown) and FM mode cycle.
  runRtmLoop();
  // V3 - 2026-04-27 - P8.1: FM arm/disarm state machine (arm window + Gate 1 throttle-release)
  runFmLoop();

  checkSerial();

  // Update last_user_input_ms when intentional input is detected.
  // Throttle threshold: thr_scaled > 20 (~8% pull — above noise floor).
  // Toggle threshold: steer_scaled moved > 15 counts from centre (127) — deliberate movement.
  // Both thresholds are chosen to reject pocket pressure and ADC noise.
  if (thr_scaled > 20 || abs((int)steer_scaled - 127) > 15)
  {
    last_user_input_ms = millis();
  }
  last_sleep_steer = steer_scaled;

  // Auto-sleep: fires when EITHER condition is true for sleep_timeout_s seconds.
  // Primary:  no intentional user input (throttle or toggle above deadzone).
  // Fallback: no LoRa packet received from RX (RX off or out of range).
  // Both use the same timeout. sleep_timeout_s == 0 disables auto-sleep entirely.
  // Pocket-safe: thr_scaled ≤ 20 and steer within 15 counts of centre do not count as input.
  if (usrConf.sleep_timeout_s > 0)
  {
    uint32_t sleep_ms   = (uint32_t)usrConf.sleep_timeout_s * 1000UL;
    bool     user_idle  = (millis() - last_user_input_ms > sleep_ms);
    bool     rx_silent  = (millis() - last_packet        > sleep_ms);
    if (user_idle || rx_silent)
    {
      deepSleep();
    }
  }

  // V3 - 2026-04-27 - P8: Show RTM/FM info (distance/speed) when RTM is active; normal display otherwise
  if (rtm_tx_active)
    renderRtmInfoDisplay();
  else
    renderOperationalDisplay();
  
  //delay(110);
  vTaskDelay(pdMS_TO_TICKS(110));

} //End of loop()
