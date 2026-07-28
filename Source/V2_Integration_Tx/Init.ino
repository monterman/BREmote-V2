// V2.5-Evo - 2026-04-21 - Added initTxGPS() call in applyConfigSettings() for TX GPS speed display
// V2.5-Evo - 2026-04-22 - Simplified initTxGPS() call site: speed_src guard moved into initTxGPS() itself
// V2.5-Evo - 2026-04-27 - P8: applyConfigSettings() always boots unlocked (lock feature removed)
// V2.5-Evo - 2026-04-27 - P8.1 Bug 1 fix: Restored no_lock=0/1 boot behavior; system_locked now conditional
// V2.5-Evo - 2026-04-29 - Fix 4-1: vibrationTask stack 1024→2048 words; handle saved for ?printtasks
// V2.5-Evo - 2026-05-02 - Create displayMutex before tasks start
// V2.5-Evo - 2026-05-13 - SW33: Added pinMode(P_MAG, INPUT) in initHardware() for DRV5032 Hall sensor on GPIO 9
// V2.5-Evo - 2026-07-20 - BLE re-enable deep-fix (Rex): bleInitTask now heap-floor-guards NimBLE init,
//   waits out the WiFi web-config AP (never WiFi+BLE together on the C3), and spawns the dedicated
//   Core-0 BLE notify task; initTasks() creates i2cMutex for the shared HT16K33+ADS1115 Wire bus.

// ===== Hardware Initialization =====

void initHardware()
{
  pinMode(P_MAG, INPUT);
  // V2.5-Evo - 2026-07-28 - probe the raw SDA/SCL line state BEFORE Wire takes the pins,
  // so a dead bus reports WHY (held low / floating / healthy) and not just "no ACK".
  i2cLineDiag();
  Wire.begin(P_I2C_SDA, P_I2C_SCL);
  Wire.setClock(400000);
  startupDisplay();
  startupADS();
}

// ===== Storage & Config =====

void initStorage()
{
  initSPIFFS();
  if(!ensureWebUiInSPIFFS())
  {
    Serial.println("WARNING: Web UI seed failed.");
  }
  getConfFromSPIFFS();
#ifdef WIFI_ENABLED
  webCfgInit();
#endif

  if(usrConf.max_gears <= 0) usrConf.max_gears = 1;

  // Radio init requires usrConf (radio_preset, rf_power) — must come after config load
  if(!config_version_error)
  {
    startupRadio();
    radio.setDio1Action(packetReceived);
  }
}

// ===== FreeRTOS Tasks =====

// V2.5-Evo - 2026-05-15 - feature/bluetooth: one-shot init task.
// Delays 5s to let LoRa and WiFi settle before starting BLE stack.
// Calls initBLE() then self-deletes. Pinned to Core 0 (ESP32-C3 is single-core) at lowest priority.
// V2.5-Evo - 2026-06-04 - Guarded by BLE_ENABLED (BREmote_V2_Tx.h). With BLE disabled this
// task is never compiled and never created, so NimBLEDevice::init() can never run.
// V2.5-Evo - 2026-07-20 - BLE re-enable deep-fix (Rex §4.1 heap guard, §4.7 WiFi/BLE mutual
//   exclusion, §4.5 dedicated notify task). Order of the added gates matters:
//     1. bail early if BLE isn't wanted (bt_enabled==0 and no boot gesture),
//     2. Rex §4.7 — wait until the WiFi web-config AP is DOWN. On the no-PSRAM C3 WiFi + BLE must
//        never co-reside (heap + 2.4 GHz coexistence). The AP only ever starts once, at boot
//        (webCfgInit() in initStorage()), and is torn down on unlock/ride; so waiting here is the
//        clean "BLE stands down while the owner is in web config, and comes up once he rides" rule.
//     3. Rex §4.1 — read free internal DRAM; skip NimBLE entirely (no boot-loop) if below the floor,
//     4. init, then spawn the Core-0 notify task that owns the periodic telemetry push (§4.5).
#ifdef BLE_ENABLED
static void bleInitTask(void* param)
{
  vTaskDelay(pdMS_TO_TICKS(5000));
  if (!(usrConf.bt_enabled > 0 || bt_session_forced))
  {
    vTaskDelete(NULL);   // BLE not requested this session — nothing to do
    return;
  }

  // Rex §4.7 — never run WiFi and BLE at the same time on the single-core, no-PSRAM C3.
  // Block until the web-config AP is gone (unlock/ride tears it down; a startup timeout also
  // stops it if no client ever connects). Negligible CPU: prio-1 task delaying 500 ms yields
  // fully to idle, so the native task watchdog keeps being fed while we wait.
#ifdef WIFI_ENABLED
  while (web_cfg_service_enabled)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
#endif

  // Rex §4.1 — heap-floor guard. The no-PSRAM C3 can init BLE into a NULL allocation under a tight
  // heap; refuse to even try below the floor and leave the rest of the firmware running normally.
  uint32_t heap_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  Serial.printf("BLE: free internal heap before init = %u bytes (floor %u)\n",
                (unsigned)heap_before, (unsigned)BLE_HEAP_FLOOR_BYTES);
  if (heap_before < BLE_HEAP_FLOOR_BYTES)
  {
    Serial.printf("BLE: SKIPPED — internal heap %u < floor %u; leaving BLE off, firmware continues.\n",
                  (unsigned)heap_before, (unsigned)BLE_HEAP_FLOOR_BYTES);
    bt_dot_state = BT_DOT_OFF;   // make sure the BT dot reflects "BLE not running"
    vTaskDelete(NULL);
    return;
  }

  initBLE();
  if (usrConf.bt_enabled == 2) bt_dot_state = BT_DOT_SLOW;  // always-on → pre-light the dot

  // Rex §4.5 — the periodic telemetry push lives here, NOT in loop(), so BLE cadence can't couple to
  // display-render timing. Prio 1 (below every app task, above idle), Core 0, modest stack. It only
  // reads telemetry/state and calls notify() — it never touches displayBuffer or the Wire bus.
  xTaskCreatePinnedToCore(bleNotifyTask, "BLE_Notify", 3072, NULL, 1, NULL, 0);

  vTaskDelete(NULL);
}
#endif

void initTasks()
{
  if(config_version_error) return;

  displayMutex = xSemaphoreCreateMutex();
  configASSERT(displayMutex != NULL);   // halt at boot if allocation fails — better than silent corruption
  // V2.5-Evo - 2026-07-20 - Rex §4.6: i2cMutex serializes the shared HT16K33 + ADS1115 Wire bus.
  // Created here, before any task starts, so the ADC task (measBufCalc) and the display render path
  // Wire bus is never torn mid-transaction between the ADC task and the display render path.
  i2cMutex = xSemaphoreCreateMutex();
  configASSERT(i2cMutex != NULL);
  xTaskCreatePinnedToCore(sendData, "Send_Data_100ms", 2048, NULL, 5, &sendDataHandle, 0);
  xTaskCreatePinnedToCore(waitForTelemetry, "wait_for_telem_triggered", 2048, NULL, 4, &triggeredWaitForTelemetryHandle, 0);
  xTaskCreatePinnedToCore(measBufCalc, "wait_for_telem_triggered_10ms", 2048, NULL, 6, &measBufCalcHandle, 0);
  xTaskCreatePinnedToCore(updateBargraphs, "wait_for_telem_triggered_200ms", 2048, NULL, 6, &updateBargraphsHandle, 0);
  xTaskCreatePinnedToCore(vibrationTask, "Vibration_Task_BG", 2048, NULL, 3, &vibrationTaskHandle, 0);
  // Finding 4-1: stack 1024→2048 words; handle saved so ?printtasks can report HWM
  // BLE init: one-shot task, 5s delayed, Core 0 (ESP32-C3 is single-core), priority 1, 4KB stack
  // V2.5-Evo - 2026-06-04 - BLE task creation guarded by BLE_ENABLED (BREmote_V2_Tx.h).
  // This is the single line that starts BLE; with the guard undefined it is never compiled,
  // so no BLE task is created and no NimBLE heap/CPU is consumed on the single-core C3.
#ifdef BLE_ENABLED
  xTaskCreatePinnedToCore(bleInitTask, "BLE_Init", 4096, NULL, 1, NULL, 0);
#endif
}

void initWatchdog()
{
  // V2.5-Evo: Watchdog handled natively by Arduino ESP32 Core.
  // Custom WDT init removed to prevent 1000ms panic reboots during TX unlock / WiFi shutdown.
  Serial.println("WDT: Handled by native Arduino Core");
}

// ===== Boot Sequence =====

void runBootSequence()
{
  bootAnimation();
  if(config_version_error) return;

  checkCal();
  checkStartupButtons();
  checkPairing();
}

// ===== Apply Config Settings =====

void applyConfigSettings()
{
  if(config_version_error)
  {
    system_locked = 1;
    return;
  }

  // V2.5-Evo - 2026-04-27 - P8.1 Bug 1 fix: Restored no_lock=0/1 behavior.
  // Always wait for throttle release first (safety gate — prevents accidental unlock during boot).
  // Then apply lock state based on SPIFFS config:
  //   no_lock=0 (locking enabled)  → system_locked stays 1; user must unlock manually via gesture
  //   no_lock=1 (locking disabled) → system_locked = 0; boots ready to use immediately
  while (thr_scaled > 10)
  {
    advanceArrow();
    delay(100);
  }
  if (usrConf.no_lock)
  {
    system_locked = 0;
#ifdef WIFI_ENABLED
    webCfgNotifyTxUnlocked();
#endif
  }

  throttleInit();

  // V2.5-Evo - 2026-04-21 - Initialize TX GPS (BN-220 on Serial1) if GPS is enabled.
  // V2.5-Evo - 2026-04-22 - speed_src guard removed from here; it now lives inside
  // initTxGPS() itself so all callers get consistent behavior automatically.
  // Called here because usrConf is fully loaded and this runs before
  // initWatchdog(), giving initTxGPS() ample margin for its ~450ms of delays.
  if (usrConf.gps_en)
  {
    initTxGPS();
  }
}
