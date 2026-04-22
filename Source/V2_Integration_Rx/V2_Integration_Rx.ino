#include "BREmote_V2_Rx.h"

SX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO, P_LORA_RST, P_LORA_BUSY);
Adafruit_AW9523 aw;
Ticker ticksrc;
TinyGPSPlus gps;

void setup()
{
  enterSetup();

  initHardware();
  initStorage();
  
  // ---> NEW: Initialize the QMC5883L Compass <---
  initCompass(); 

  // SURGICAL STRIKE: Deletes only the old webpage, leaves configs 100% safe.  //Coment out after it works fins and no new changes//
  //SPIFFS.remove("/index.html"); 

  runBootSequence();
  initTasks();
  initWatchdog();

  // Initialize the logger task and memory mutex
  initLogger();
  if (usrConf.logger_en == 1) {
    startLog();
  }

  exitSetup();
  PWM_active = 1;
}

unsigned long loop_timer = 0;
int wetness_counter = 0;

void loop()
{
  esp_task_wdt_reset();
#ifdef WIFI_ENABLED
  webCfgLoop();
#endif
  checkSerial();
  
  // Process Logger LED and button in main thread (AW9523 I2C is not ISR-safe)
  loggerLoop();
  
  if(millis()-loop_timer > 1000)
  {
    loop_timer = millis();
    
    if(usrConf.wet_det_active)
    {
      wetness_counter++;
      if(wetness_counter >= 10)
      {
        checkWetness();
        wetness_counter = 0;
      }
    }

    if(usrConf.gps_en)
    {
      getGPSLoop();
    }

    if(usrConf.data_src == 1)
    {
      getUbatLoop();
    }
    else if(usrConf.data_src == 2)
    {
      getVescLoop();
    }
  }

  vTaskDelay(pdMS_TO_TICKS(10));
}