// V2.5-Evo - 2026-05-11 - Telemetry Fix: VESC moved to its own vesc_loop_timer (2Hz); checkButtons() added to loop() for runtime BIND compass cal
// V3 - 2026-05-03 - Removed commented-out SPIFFS.remove dead code (LOW audit cleanup)
// V3 - 2026-04-30 - Bundle E: GPS moved to its own gps_loop_timer (rate = gps_update_hz); removed from 1000ms gate
// V3 - 2026-04-25 - P7: Added runRtmLoop() call in loop(); forward declarations
#include "BREmote_V2_Rx.h"

SX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO, P_LORA_RST, P_LORA_BUSY);
Adafruit_AW9523 aw;
Ticker ticksrc;
TinyGPSPlus gps;

// V3 - 2026-04-25 - P7: RTM state machine and compass heading function
void runRtmLoop();
float getCompassHeading();

void setup()
{
  enterSetup();

  initHardware();
  initStorage();
  
  // ---> NEW: Initialize the QMC5883L Compass <---
  initCompass(); 

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
unsigned long gps_loop_timer  = 0;  // V3 - 2026-04-30 - Bundle E: separate GPS polling timer
unsigned long vesc_loop_timer = 0;  // V3 - 2026-05-11 - Telemetry Fix: separate VESC polling timer (2Hz)
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

  // Runtime button detection: BIND = compass cal. Boot-time pairing is guarded inside.
  checkButtons();

  // V3 - 2026-04-25 - P7: RTM state machine — safety gates, steering override, Phase C.
  // Runs at 10Hz regardless of the 1000ms GPS/VESC gate below.
  runRtmLoop();

  // GPS runs on its own configurable-rate timer, independent of the 1000ms VESC/wetness gate.
  // gps_update_hz=2 → 500ms interval; gps_update_hz=5 → 200ms interval.
  // Guard against zero (divide-by-zero): fall back to 500ms if gps_update_hz is unset.
  if(usrConf.gps_en)
  {
    uint32_t gps_interval_ms = (usrConf.gps_update_hz > 0) ? (1000UL / usrConf.gps_update_hz) : 500UL;
    if(millis() - gps_loop_timer >= gps_interval_ms)
    {
      gps_loop_timer = millis();
      getGPSLoop();
    }
  }

  // VESC at 2Hz, independent of GPS and wetness gate.
  if(usrConf.data_src == 2)
  {
    if(millis() - vesc_loop_timer >= 500)
    {
      vesc_loop_timer = millis();
      getVescLoop();
    }
  }

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

    if(usrConf.data_src == 1)
    {
      getUbatLoop();
    }
  }

  vTaskDelay(pdMS_TO_TICKS(10));
}