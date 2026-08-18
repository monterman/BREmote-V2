// *** LATEST: V2.5-Evo - 2026-07-25 - STAGE 0 (instrumentation only): loop() body wrapped in diagLoopBegin()/diagLoopEnd() so ?diag and the level-4 log record can report loop min/mean/max time. Cycle-counter based (~5 ns), measured before the existing vTaskDelay so the deliberate 10 ms sleep is not counted as work. No control-path statement added, removed or reordered; no confStruct change; SW_VERSION stays 34 ***
// V2.5-Evo - 2026-07-19 - P3 FM — runFmLoop() added to loop() after runRtmLoop(); Follow-Me autonomous following (see RTMState.ino). No confStruct change; SW_VERSION stays 33
// V2.5-Evo - 2026-06-04 - D1 — UART-mux read-back verify (setUartMux); skip VESC poll while throttle high; no confStruct change
// V2.5-Evo - 2026-05-14 - SW55 — GPS yields MUX to VESC on exit; rcv_err removed from receiveFromVESC; boot MUX starts on VESC
// V2.5-Evo - 2026-05-11 - Telemetry Fix: VESC moved to its own vesc_loop_timer (2Hz); checkButtons() added to loop() for runtime BIND compass cal
// V2.5-Evo - 2026-05-03 - Removed commented-out SPIFFS.remove dead code (LOW audit cleanup)
// V2.5-Evo - 2026-04-30 - Bundle E: GPS moved to its own gps_loop_timer (rate = gps_update_hz); removed from 1000ms gate
// V2.5-Evo - 2026-04-25 - P7: Added runRtmLoop() call in loop(); forward declarations
#include "BREmote_V2_Rx.h"

SX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO, P_LORA_RST, P_LORA_BUSY);
Adafruit_AW9523 aw;
Ticker ticksrc;
TinyGPSPlus gps;

// V2.5-Evo - 2026-04-25 - P7: RTM state machine and compass heading function
void runRtmLoop();
float getCompassHeading();
// V2.5-Evo - 2026-07-19 - P3 FM: Follow-Me state machine (RTMState.ino)
void runFmLoop();

void setup()
{
  // V2.5-Evo - 2026-08-18 - WEB-SERIAL-1, Phase 0. FIRST line of setup(), before anything prints.
  // Only creates the ring's recursive mutex - capture itself is already live, because the ring
  // and its head counter are .bss and were zeroed before any constructor ran. Prints that happen
  // ahead of this are still captured, unlocked, which is correct: nothing else is running yet.
  serialTeeInit();

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
unsigned long gps_loop_timer  = 0;  // V2.5-Evo - 2026-04-30 - Bundle E: separate GPS polling timer
unsigned long vesc_loop_timer = 0;  // V2.5-Evo - 2026-05-11 - Telemetry Fix: separate VESC polling timer (2Hz)
int wetness_counter = 0;

void loop()
{
  // V2.5-Evo - 2026-07-25 - STAGE 0 (instrumentation only): stamp the CPU cycle counter on the
  // first line and close the measurement on the last line of the body, BEFORE the vTaskDelay
  // below — so what gets recorded is the WORK this pass did, not the 10 ms it deliberately
  // sleeps afterwards. esp_cpu_get_cycle_count() is a single CSR read (~5 ns); micros() would
  // cost ~1 us, the same order as the thing being measured. Nothing between these two lines is
  // changed, reordered, added or removed.
  const uint32_t diag_loop_c0 = diagLoopBegin();

  esp_task_wdt_reset();
#ifdef WIFI_ENABLED
  webCfgLoop();
#endif
  checkSerial();
  
  // Process Logger LED and button in main thread (AW9523 I2C is not ISR-safe)
  loggerLoop();

  // Runtime button detection: BIND = compass cal. Boot-time pairing is guarded inside.
  checkButtons();

  // V2.5-Evo - 2026-04-25 - P7: RTM state machine — safety gates, steering override, Phase C.
  // Runs at 10Hz regardless of the 1000ms GPS/VESC gate below.
  runRtmLoop();

  // V2.5-Evo - 2026-07-19 - P3 FM: Follow-Me state machine — activation conditions, trailing
  // target-point steering, throttle cap chain. Also rate-limits itself to 10Hz internally.
  // MUST run after runRtmLoop(): RTM and FM are mutually exclusive and runFmLoop() reads
  // rtm_rx_active to decide whether RTM already owns the buggy this tick. FM keeps its own
  // throttle cap (fm_throttle_cap) rather than sharing rtm_approach_cap, so RTM's per-tick
  // "inactive -> cap 255" housekeeping can never transiently clear an FM cap in between.
  runFmLoop();

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
  // V2.5-Evo - 2026-07-18 - REGRESSION FIX: removed D1's "&& thr_received < 25" throttle gate.
  // D1 (2026-06-04) skipped the VESC poll while throttle >= 25 to dodge mux-switch EMI during driving,
  // but on a continuous-throttle vehicle (tow buggy) the poll then NEVER runs -> telemetry freezes at
  // the boot 0xFF default (dashes) for the whole ride. The EMI concern is already covered by the
  // bounded read-back-verify inside setUartMux() (System.ino), so the throttle-skip was redundant
  // belt-and-suspenders that cost all in-ride telemetry. Restore the SW55 unconditional 2Hz poll.
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

  // STAGE 0: close the loop-body timing measurement started at the top of loop().
  // Must stay the last statement before the vTaskDelay, otherwise the 10 ms sleep is counted
  // as work and every loop reads as "10 ms" regardless of what actually happened.
  diagLoopEnd(diag_loop_c0);

  vTaskDelay(pdMS_TO_TICKS(10));
}