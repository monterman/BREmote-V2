// V2.5-Evo - 2026-08-16 - initWatchdog() no longer returns early on config_version_error, so the task watchdog is armed on EVERY boot — including the first boot after a version bump, which used to run with no watchdog at all. A version mismatch is a self-healing condition (defaults are re-baked and re-read); a genuine config failure halts in spiffsErrorHalt() and never reaches this function. The flag itself is left set and untouched — it is shared with the TX, where it drives a whole-boot safe mode. No confStruct change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-07-25 - STAGE 1 (GPS repair): Serial1.setRxBufferSize(2048) added in runBootSequence() immediately BEFORE the first Serial1.begin(). The old setRxBufferSize(512) lived in configureGPS() (GPS.ino), which runs AFTER this begin() — and arduino-esp32 refuses a resize once the UART driver is installed, so the ring has always silently been the 256-byte default. No confStruct change, SW_VERSION stays 34.

// ===== Hardware Initialization =====

void initHardware()
{
  i2cMutex = xSemaphoreCreateMutex();
  Wire.begin(P_I2C_SDA, P_I2C_SCL);
  Wire.setClock(400000);
  startupAW();
}

// ===== Storage & Config =====

void initStorage()
{
  initSPIFFS();
  ensureWebUiInSPIFFS();
  getConfFromSPIFFS();
  getBCFromSPIFFS();
#ifdef WIFI_ENABLED
  webCfgInit();
#endif

  // Semaphore must exist before radio can receive (ISR uses it)
  triggerReceiveSemaphore = xSemaphoreCreateBinary();

  // Radio init requires usrConf (radio_preset, rf_power) — must come after config load
  startupRadio();
  radio.setPacketReceivedAction(packetReceived);
  radio.implicitHeader(6);
}

// ===== FreeRTOS Tasks =====

void initTasks()
{
  loopTaskHandle = xTaskGetCurrentTaskHandle();

  // V2.5-Evo fix (Bug 6): increased stack sizes from 2048 bytes — too small for tasks that call
  // RadioLib, RMT driver, AW9523 I2C, and handle interrupt nesting. Use ?printtasks to
  // monitor high-water marks after the fix; reduce if headroom proves excessive.
  //
  // L-1 NOTE (single-core): this RX runs on the single-core ESP32-C3 (HT-CT62). The `0` in
  // xTaskCreatePinnedToCore() is the only core. Comments elsewhere mentioning "Core 0/Core 1",
  // "dual-core" or "Xtensa" are legacy from the ESP32-S3 port — all tasks AND loop() run on the
  // one core. The volatile / std::atomic / mutex guards are still correct: they protect against
  // FreeRTOS *task preemption* and give compiler memory barriers, not against multiple cores.
  //Runs every 10ms to generate both PWM signals, high prio
  xTaskCreatePinnedToCore(generatePWM, "Generate_PWM_10ms", 4096, NULL, 10, &generatePWMHandle, 0);
  //Runs upon RF interrupt and reads packet & responds, medium-high prio
  xTaskCreatePinnedToCore(triggeredReceive, "RF_ReceiveTask_triggered", 4096, NULL, 5, &triggeredReceiveHandle, 0);
  //Checks if there is connection and blinks LED, low prio
  xTaskCreatePinnedToCore(checkConnStatus, "Check_conn_staus_200ms", 3072, NULL, 2, &checkConnStatusHandle, 0);
}

void initWatchdog()
{
  // ============================================================
  // V2.5-Evo - 2026-08-16 - THE WATCHDOG IS NOW ARMED ON EVERY BOOT.
  //
  // WHAT THE BUG WAS: this function opened with `if(config_version_error) return;`, so the task
  // watchdog was never started on any boot where the config stored in SPIFFS carried a different
  // SW_VERSION than the running firmware. That is precisely the FIRST boot after every flash of
  // a new build — and riders commonly go straight out on that boot without a second power cycle.
  // The one session most likely to meet a new-code bug was the one session with nothing watching
  // for a hang: no panic, no reboot, just a buggy whose control loop has stopped.
  //
  // WHY SKIPPING IT WAS NEVER RIGHT: a version mismatch is not a failure. Its whole consequence,
  // in getConfFromSPIFFS() (Common/SPIFFSEngine.h), is that defaultConf is re-baked into SPIFFS
  // and read back — an expected, recoverable, self-healing condition — so by the time execution
  // reaches this line usrConf holds a fully valid config. A GENUINE config failure cannot reach
  // this line at all: if the default re-bake fails, getConfFromSPIFFS() calls spiffsErrorHalt(),
  // which never returns (it blinks the AUX LED forever). The early return therefore protected
  // nothing and only disabled the watchdog on the boot that needed it most.
  //
  // WHY THE FLAG IS LEFT SET rather than cleared: config_version_error is set in Common/
  // SPIFFSEngine.h, which the TX compiles too, and the TX uses the flag as a whole-boot safe
  // mode — it skips the radio, skips initTasks(), skips the boot sequence, forces system_locked,
  // keeps serial on and scrolls "EV" instead of running loop(). Clearing it in the shared setter
  // would silently disarm all six of those. This function was its ONLY consumer on the RX, so
  // dropping the early return changes nothing else on this board and nothing at all on the TX.
  // ============================================================

  // V2.5-Evo fix (Bug 1): increased from 1000ms to 3000ms.
  // Peak loop load: getGPSLoop(300ms) + checkWetness(300ms) + getVescLoop(210ms) = ~810ms
  // before the loop task resets the WDT. 1000ms left only 190ms margin — a false trigger
  // under any combination of GPS + wetness + VESC in one iteration. 3000ms gives 3.7x margin
  // while still catching genuine hangs and deadlocks.
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 3000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };

  esp_err_t err = esp_task_wdt_init(&twdt_config);
  if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_add(NULL);  // loop task
    esp_task_wdt_add(generatePWMHandle);
    esp_task_wdt_add(checkConnStatusHandle);
    esp_task_wdt_add(triggeredReceiveHandle);
    // V2.5-Evo - 2026-07-31 - RX-WDT-1: publish the fact that the loop task is now SUBSCRIBED.
    // esp_task_wdt_reset() logs "task not found" at error level on every call from a task that
    // is not registered, and configureGPS() runs in runBootSequence() — BEFORE this function.
    // Without this flag the GPS code's feeds emitted hundreds of error lines during boot,
    // which is not merely ugly: that volume of spurious errors buries real ones.
    g_wdt_active = true;
    Serial.println("WDT: initialized");
  } else {
    Serial.println("WDT: init failed");
  }
}

// ===== Peripherals & Boot Sequence =====

void runBootSequence()
{
  checkButtons();
  rxIsrState = 1;
  radio.startReceive();

  initRMT();

  // ============================================================
  // V2.5-Evo - 2026-07-25 - STAGE 1: size the Serial1 RX ring BEFORE the first begin().
  //
  // WHAT THE BUG WAS: configureGPS() (GPS.ino) called Serial1.setRxBufferSize(512), but it runs
  // AFTER the begin() on the next line. arduino-esp32 rejects a resize once the UART driver is
  // installed — it logs "RX Buffer can't be resized when Serial is already running" and returns
  // 0 — so the ring has always been the 256-byte default, not 512. That was harmless only
  // because the UART mux was parked on the VESC and almost no GPS bytes were ever captured;
  // the moment STAGE 1 lets the GPS stream continuously it becomes the binding failure.
  //
  // WHY 2048: at 115200 baud a 256-byte ring holds only ~22 ms of airtime. checkWetness()
  // blocks loop() for ~300 ms every 10 s, and at 115200 roughly 750 bytes arrive in that
  // window — the ring would overflow and drop whole NMEA sentences on every wetness check.
  // 2048 bytes is ~178 ms of airtime, which rides out that stall with margin, and costs 1.75 kB
  // of heap once at boot.
  //
  // This value persists across the Serial1.end()/begin() baud-switch cycle that configureGPS()
  // performs for the BN-220/BN-880: the core keeps _rxBufferSize across end() and passes it to
  // every subsequent begin().
  // ============================================================
  Serial1.setRxBufferSize(2048);
  Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);

  aw.digitalWrite(AP_EN_PWM0, 1);
  aw.digitalWrite(AP_EN_PWM1, 1);

  configureGPS();
}
