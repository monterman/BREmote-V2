// V3 - 2026-04-24 - Call initTxGPS() in setup() after applyConfigSettings() so GPS UART is ready on boot
// V3 - 2026-04-21 - Added getTxGPSLoop() call in loop() and forward declarations for TX GPS functions
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
static constexpr uint32_t SLEEP_TIMEOUT_MS = 5UL * 60UL * 1000UL;

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

  // V3 - 2026-04-21 - Poll TX GPS bytes each loop iteration (non-blocking).
  // Only runs when gps_en=1 AND a TX-GPS speed unit is selected (speed_src 2/3/5).
  // getTxGPSLoop() drains Serial1 into TinyGPS++ and updates tx_gps_speed.
  // It never blocks — safe to call every 110ms loop tick.
  if (usrConf.gps_en &&
      (usrConf.speed_src == 2 || usrConf.speed_src == 3 || usrConf.speed_src == 5))
  {
    getTxGPSLoop();
  }

  runMenu();
  if(in_menu > 0) in_menu--;

  checkSerial();

  if(millis() - last_packet > SLEEP_TIMEOUT_MS)
  {
    deepSleep();
  }

  renderOperationalDisplay();
  
  //delay(110);
  vTaskDelay(pdMS_TO_TICKS(110));

} //End of loop()
