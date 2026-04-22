// ===== Hardware Initialization =====

void initHardware()
{
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

  //Runs every 10ms to generate both PWM signals, high prio
  xTaskCreatePinnedToCore(generatePWM, "Generate_PWM_10ms", 2048, NULL, 10, &generatePWMHandle, 0);
  //Runs upon RF interrupt and reads packet & responds, medium-high prio
  xTaskCreatePinnedToCore(triggeredReceive, "RF_ReceiveTask_triggered", 2048, NULL, 5, &triggeredReceiveHandle, 0);
  //Checks if there is connection and blinks LED, low prio
  xTaskCreatePinnedToCore(checkConnStatus, "Check_conn_staus_200ms", 2048, NULL, 2, &checkConnStatusHandle, 0);
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
    esp_task_wdt_add(generatePWMHandle);
    esp_task_wdt_add(checkConnStatusHandle);
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
  Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);

  aw.digitalWrite(AP_EN_PWM0, 1);
  aw.digitalWrite(AP_EN_PWM1, 1);

  configureGPS();
}
