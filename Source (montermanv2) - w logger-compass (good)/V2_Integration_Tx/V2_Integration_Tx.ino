#include "BREmote_V2_Tx.h"
// --- AFM-Gemini: Comprehensive Function Prototypes to bypass Arduino IDE bug ---
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
void vibrationTask(void *parameter);   // // AFM-Gemini: These new one is vibration motor and patterns
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
  // esp_task_wdt_reset(); // AFM-Gemini: Removed
  if(config_version_error)
  {
    scroll3Digits(LET_E, 5, LET_V, 200);
    checkSerial();
    return;
  }

#ifdef WIFI_ENABLED
  webCfgLoop();
#endif
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
