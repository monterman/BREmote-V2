#include "BREmote_V2_Tx.h"

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
  checkCharger();
  initTasks();
  runBootSequence();
  applyConfigSettings();
  initWatchdog();

  exitSetup();
  in_setup = false;

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
  esp_task_wdt_reset();
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
