// V3 - 2026-04-21 - Added initTxGPS() call in applyConfigSettings() for TX GPS speed display
// V3 - 2026-04-22 - Simplified initTxGPS() call site: speed_src guard moved into initTxGPS() itself

// ===== Hardware Initialization =====

void initHardware()
{
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

void initTasks()
{
  if(config_version_error) return;

  xTaskCreatePinnedToCore(sendData, "Send_Data_100ms", 2048, NULL, 5, &sendDataHandle, 0);
  xTaskCreatePinnedToCore(waitForTelemetry, "wait_for_telem_triggered", 2048, NULL, 4, &triggeredWaitForTelemetryHandle, 0);
  xTaskCreatePinnedToCore(measBufCalc, "wait_for_telem_triggered_10ms", 2048, NULL, 6, &measBufCalcHandle, 0);
  xTaskCreatePinnedToCore(updateBargraphs, "wait_for_telem_triggered_200ms", 2048, NULL, 6, &updateBargraphsHandle, 0);
  xTaskCreatePinnedToCore(vibrationTask, "Vibration_Task_BG", 1024, NULL, 3, NULL, 0);
}

void initWatchdog()
{
  // V3: Watchdog handled natively by Arduino ESP32 Core.
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

  if(usrConf.no_lock)
  {
    while(thr_scaled > 10)
    {
      advanceArrow();
      delay(100);
    }
    system_locked = 0;
#ifdef WIFI_ENABLED
    webCfgNotifyTxUnlocked();
#endif
  }

  throttleInit();

  // V3 - 2026-04-21 - Initialize TX GPS (BN-220 on Serial1) if GPS is enabled.
  // V3 - 2026-04-22 - speed_src guard removed from here; it now lives inside
  // initTxGPS() itself so all callers get consistent behavior automatically.
  // Called here because usrConf is fully loaded and this runs before
  // initWatchdog(), giving initTxGPS() ample margin for its ~450ms of delays.
  if (usrConf.gps_en)
  {
    initTxGPS();
  }
}
