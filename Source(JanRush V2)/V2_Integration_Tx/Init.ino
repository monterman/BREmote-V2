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
}

void initWatchdog()
{
  if(config_version_error) return;

  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };

  esp_err_t err = esp_task_wdt_init(&twdt_config);
  if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_add(NULL);  // loop task
    esp_task_wdt_add(sendDataHandle);
    esp_task_wdt_add(measBufCalcHandle);
    esp_task_wdt_add(updateBargraphsHandle);
    Serial.println("WDT: initialized");
  } else {
    Serial.println("WDT: init failed");
  }
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
}
