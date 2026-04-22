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
  runBootSequence();
  initTasks();
  initWatchdog();

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

  //telemetry.foil_speed = (millis()/1000) % 100;

  vTaskDelay(pdMS_TO_TICKS(10));
}
