const char* SYS_DEVICE_LABEL = "RX";

void startupAW()
{
  Serial.print("Starting AW9532...");

  if (! aw.begin(0x58)) {
    Serial.println("AW9523 not found!");
    while (1) delay(10);
  }

  aw.pinMode(AP_U1_MUX_0, OUTPUT);
  aw.pinMode(AP_U1_MUX_1, OUTPUT);
  aw.pinMode(AP_S_BIND, INPUT);
  aw.pinMode(AP_S_AUX, INPUT);
  aw.pinMode(AP_L_BIND, OUTPUT);
  aw.pinMode(AP_L_AUX, OUTPUT);
  aw.pinMode(AP_EN_BMS_MEAS, OUTPUT);
  aw.pinMode(AP_BMS_MEAS, INPUT);
  aw.pinMode(AP_EN_PWM0, OUTPUT);
  aw.pinMode(AP_EN_PWM1, OUTPUT);
  aw.pinMode(AP_EN_WET_MEAS, OUTPUT);
  aw.pinMode(AP_WET_MEAS, INPUT);

  aw.digitalWrite(AP_L_BIND, HIGH);
  aw.digitalWrite(AP_L_AUX, HIGH);
  aw.digitalWrite(AP_EN_BMS_MEAS, HIGH);

  Serial.println(" Done");
}

void setUartMux(int channel)
{
  if(channel == 0)
  {
    aw.digitalWrite(AP_U1_MUX_0, LOW);
    aw.digitalWrite(AP_U1_MUX_1, LOW);
  }
  if(channel == 1)
  {
    aw.digitalWrite(AP_U1_MUX_0, HIGH);
    aw.digitalWrite(AP_U1_MUX_1, LOW);
  }
}

void checkWetness()
{
  aw.digitalWrite(AP_EN_WET_MEAS, HIGH);
  vTaskDelay(pdMS_TO_TICKS(50));
  if(!aw.digitalRead(AP_WET_MEAS))
  {
    uint8_t amt = 0;
    for(uint8_t i = 0; i < 5; i++)
    {
      vTaskDelay(pdMS_TO_TICKS(50));
      amt += aw.digitalRead(AP_WET_MEAS);
    }
    if(amt == 0)
    {
      if(telemetry.error_code == 0)
      {
       telemetry.error_code = 7;
      }
    }
  }
  else
  {
    if(telemetry.error_code == 7)
    {
      telemetry.error_code = 0;
    }
  }
  aw.digitalWrite(AP_EN_WET_MEAS, LOW);
}

void getUbatLoop()
{
  uint16_t raw = analogRead(P_UBAT_MEAS);
  raw += analogRead(P_UBAT_MEAS);
  raw += analogRead(P_UBAT_MEAS);

  float vActual = (float)raw*usrConf.ubat_cal;

  telemetry.foil_bat = getUbatPercent(vActual);
}

uint8_t getUbatPercent(float pack_voltage)
{
  if(millis() - percent_last_thr_change > 5000)
  {
    uint8_t thr_state = (thr_received < 10);

    if( thr_state != percent_last_thr)
    {
      percent_last_thr = thr_state;
      percent_last_thr_change = millis();
      return percent_last_val;
    }

    uint16_t upackvolt = 0;

    if(thr_state)
    {
      float fpackvolt = ((((pack_voltage+usrConf.ubat_offset) / usrConf.foil_num_cells)-2.0-noload_offset) * 100.0);
      if(fpackvolt > 0) upackvolt = (uint16_t)fpackvolt;
      else upackvolt = 0;
    }
    else
    {
      upackvolt = (uint16_t)((((pack_voltage+usrConf.ubat_offset) / usrConf.foil_num_cells)-2.0) * 100.0);
    }

    uint8_t percent_rem = 100;
    while(bc_arr[100-percent_rem] > upackvolt && percent_rem > 0) percent_rem--;

    if(percent_rem < 100 && percent_rem > 0)
    {
      if((upackvolt-bc_arr[100-percent_rem]) > (bc_arr[100-percent_rem-1]-upackvolt))
      {
        percent_rem += 1;
      }
    }

    percent_last_val = percent_rem;
    return percent_rem;
  }
  else
  {
    return percent_last_val;
  }
}

void blinkErr(int num, uint8_t pin)
{
  for(int i = 0; i < num; i++)
  {
    aw.digitalWrite(pin, LOW);
    delay(200);
    aw.digitalWrite(pin, HIGH);
    delay(200);
  }
  delay(500);
  checkSerial();
}

void blinkBind(int num)
{
  for(int i = 0; i < num; i++)
  {
    aw.digitalWrite(AP_L_BIND, LOW);
    delay(50);
    aw.digitalWrite(AP_L_BIND, HIGH);
    delay(50);
  }
}

// ===== RX-Specific Command Handlers =====

struct SerialCommand {
  const char* name;
  const char* help;
  void (*handler)(const String& params);
};

void cmdSetConf(const String& params) { serSetConf(params); }
void cmdSetBC(const String& params) { serSetBC(params); }
void cmdClearConf(const String& params) { serClearConf(); }
void cmdClearBC(const String& params) { serClearBC(); }
void cmdApplyConf(const String& params) { serApplyConf(); }
void cmdPrintPWM(const String& params) { serPrintPWM(); }
void cmdPrintRSSI(const String& params) { serPrintRSSI(); }
void cmdPrintTasks(const String& params) { serPrintTasks(); }
void cmdPrintGPS(const String& params) { serPrintGPS(); }
void cmdPrintBat(const String& params) { serPrintBat(); }
void cmdPrintReceived(const String& params) { serPrintReceived(); }
void cmdTestBG(const String& params) { readTelemetryUntilQuit(); }
void cmdTestPercent(const String& params) { testPercent(); }

void cmdWifiStop(const String& params) {
#ifdef WIFI_ENABLED
  webCfgNotifyRxConnected();
  Serial.println("RX connected notified: AP will stop.");
#else
  Serial.println("ERR: WiFi disabled at compile time");
#endif
}

void cmdHelp(const String& params);

static const SerialCommand kCommands[] = {
  {"conf", "print current config", cmdConf},
  {"setconf", "<data> write Base64 config to SPIFFS", cmdSetConf},
  {"setbc", "<data> write Base64 battery cal to SPIFFS", cmdSetBC},
  {"set", "<key> <value> set config value", cmdSet},
  {"get", "<key> get config value", cmdGet},
  {"keys", "list all config keys", cmdKeys},
  {"applyconf", "reload config from SPIFFS", cmdApplyConf},
  {"save", "save config to SPIFFS", cmdSave},
  {"clearconf", "delete config from SPIFFS", cmdClearConf},
  {"clearbc", "delete battery cal from SPIFFS", cmdClearBC},
  {"reboot", "reboot the device", cmdReboot},
  {"printpwm", "print PWM values", cmdPrintPWM},
  {"printrssi", "print RSSI/SNR", cmdPrintRSSI},
  {"printreceived", "print received throttle/steering", cmdPrintReceived},
  {"printtasks", "print task stack usage", cmdPrintTasks},
  {"printgps", "print GPS info", cmdPrintGPS},
  {"printbat", "print battery voltage", cmdPrintBat},
  {"testbg", "test background telemetry", cmdTestBG},
  {"testpercent", "test percentage calculation", cmdTestPercent},
  {"wifi", "[on|off] WiFi/AP config service", cmdWifi},
  {"wifidbg", "[some|full|off] get/set wifi debug mode", cmdWifiDbg},
  {"wifips", "[<ms>|off] get/set AP startup timeout", cmdWifiPs},
  {"wifistop", "notify RX connected, stop AP", cmdWifiStop},
  {"wifiver", "print web UI version info", cmdWifiVer},
  {"wifiupd", "force web UI update to SPIFFS", cmdWifiUpd},
  {"wifistate", "wifi config state/counters", cmdWifiState},
  {"wifierr", "last wifi config error", cmdWifiErr},
  {"", "show this help", cmdHelp},
};

static const size_t kCommandCount = sizeof(kCommands) / sizeof(kCommands[0]);

void cmdHelp(const String& params) {
  Serial.println("Available commands (case-insensitive):");
  for (size_t i = 0; i < kCommandCount; i++) {
    Serial.print("?");
    Serial.print(kCommands[i].name);
    if (kCommands[i].help && strlen(kCommands[i].help) > 0) {
      Serial.print(" ");
      Serial.print(kCommands[i].help);
    }
    Serial.println();
  }
}

void checkSerial()
{
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Read input until newline

    // SECURITY FIX: Limit command length to prevent heap exhaustion
    if (command.length() > 512) {
      Serial.println("ERROR: Command too long (max 512 chars)");
      return;
    }

    // Trim leading and trailing spaces
    command.trim();

    // Process the command
    if (command.startsWith("?") || command.startsWith("?")) {
      // Find parameter separator - support both ":" and whitespace
      int separatorPos = -1;
      String params = "";

      // First try to find ":", then fall back to whitespace
      int colonPos = command.indexOf(':');
      int spacePos = command.indexOf(' ');

      if (colonPos > 0 && (spacePos < 0 || colonPos < spacePos)) {
        separatorPos = colonPos;
      } else if (spacePos > 0) {
        separatorPos = spacePos;
      }

      if (separatorPos > 0) {
        params = command.substring(separatorPos + 1);
        params.trim();
        command = command.substring(0, separatorPos);
      }

      // Remove leading "?" for table lookup
      String cmdName = command;
      if (cmdName.startsWith("?")) {
        cmdName = cmdName.substring(1);
      }

      // Commands that need original-case args
      if(cmdName != "setconf" && cmdName != "get" && cmdName != "set" && cmdName != "wifidbg" && cmdName != "wifips")
      {
        cmdName.toLowerCase();
        params.toLowerCase();
      }
      else
      {
        cmdName.toLowerCase();
      }

      // Lookup command in table
      bool found = false;
      for (size_t i = 0; i < kCommandCount; i++) {
        if (cmdName == kCommands[i].name) {
          kCommands[i].handler(params);
          found = true;
          break;
        }
      }

      if (!found) {
        Serial.println("Unknown command. Type '?' for help.");
      }
    }
    else {
      Serial.println("Unknown command. Type '?' for help.");
    }
  }
}

void testPercent()
{
  while(1)
  {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n'); // read until newline
      input.trim(); // remove spaces and newlines

      if (input.equalsIgnoreCase("quit")) {
        Serial.println("Quit command received. Stopping input loop.");
        break;
      }

      // Try to parse float
      float value = input.toFloat();

      // Validate: toFloat returns 0.0 if invalid, so check original string too
      if (input.length() == 0 || (value == 0.0f && !input.startsWith("0"))) {
        Serial.println("Invalid input. Please enter a float or 'quit'.");
      } else if (value >= 0.0f && value <= 100.0f) {
        Serial.println(getUbatPercent(value));
      } else {
        Serial.println("Value out of range (0.0 - 100.0).");
      }
    }
  }
}

void readTelemetryUntilQuit() {
    while (true) {
        if (Serial.available()) {
            String input = Serial.readStringUntil('\n');  // read line
            input.trim();  // remove CR/LF/whitespace

            if (input.equalsIgnoreCase("quit")) {
                Serial.println("Exiting telemetry read loop.");
                break; // stop the function
            }

            // Parse values separated by commas
            int firstComma = input.indexOf(',');
            int secondComma = input.indexOf(',', firstComma + 1);

            if (firstComma < 0 || secondComma < 0) {
                Serial.println("Error: Expected 3 values separated by commas.");
                continue; // wait for next line
            }

            String val1 = input.substring(0, firstComma);
            String val2 = input.substring(firstComma + 1, secondComma);
            String val3 = input.substring(secondComma + 1);

            int bat  = constrain(val1.toInt(), 0, 255);
            int temp = constrain(val2.toInt(), 0, 255);
            int link = constrain(val3.toInt(), 0, 255);

            telemetry.foil_bat     = (uint8_t)bat;
            telemetry.foil_temp    = (uint8_t)temp;
            telemetry.link_quality = (uint8_t)link;

            Serial.print("Updated telemetry -> ");
            Serial.print("Bat: "); Serial.print(telemetry.foil_bat);
            Serial.print(" Temp: "); Serial.print(telemetry.foil_temp);
            Serial.print(" Link: "); Serial.println(telemetry.link_quality);
        }
    }
}

void serPrintGPS()
{
  printSatelliteInfo();
}

void serPrintBat()
{
  while (true)
  {
    if(checkSerialQuit()) break;

    if(usrConf.data_src == 1)
    {
      getUbatLoop();
    }
    else if(usrConf.data_src == 2)
    {
      getVescLoop();
    }

    if(usrConf.data_src == 1)
    {
      uint16_t raw = analogRead(P_UBAT_MEAS);
      raw += analogRead(P_UBAT_MEAS);
      raw += analogRead(P_UBAT_MEAS);

      float vActual = (float)raw*usrConf.ubat_cal;
      Serial.print("Measured: ");
      Serial.print(vActual);
      Serial.print("V, offset: ");
      Serial.print(usrConf.ubat_offset);
      Serial.print("V, final: ");
      Serial.println(vActual + usrConf.ubat_offset);
    }
    else if(usrConf.data_src == 2)
    {
      getVescLoop();
      Serial.print("Measured: ");
      Serial.print(fbatVolt);
      Serial.print("V, offset: ");
      Serial.print(usrConf.ubat_offset);
      Serial.print("V, final: ");
      Serial.println(fbatVolt + usrConf.ubat_offset);
    }
    else
    {
      Serial.println("data_src not selected! Exiting...");
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void serSetBC(String data) {
  Serial.print("Setting batcal to: ");
  Serial.println(data);

  uint8_t* encodedData = new uint8_t[data.length()];
  for (size_t i = 0; i < data.length(); i++) {
    encodedData[i] = data[i];
  }

  // Save to SPIFFS
  File file = SPIFFS.open(BC_FILE_PATH, FILE_WRITE);
  if (!file) {
      Serial.println("Failed to open file for writing");
      delete[] encodedData;
      return;
  }
  file.write(encodedData, data.length());
  file.close();
  Serial.println("Batcal saved to SPIFFS as Base64");
  delete[] encodedData;
}

void serClearBC()
{
  Serial.println("Deleting batcal from SPIFFS");
  deleteBCFromSPIFFS();
}

void serPrintTasks()
{
  while (true)
  {
    if(checkSerialQuit()) break;

    Serial.println("\n=== Task Stack Usage ===");

    Serial.printf("receive stack left: %u words\n", uxTaskGetStackHighWaterMark(triggeredReceiveHandle));
    Serial.printf("pwm stack left: %u words\n", uxTaskGetStackHighWaterMark(generatePWMHandle));
    Serial.printf("check_conn stack left: %u words\n", uxTaskGetStackHighWaterMark(checkConnStatusHandle));
    Serial.printf("loop() stack left: %u words\n", uxTaskGetStackHighWaterMark(loopTaskHandle));

    Serial.println("========================\n");

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void serPrintRSSI()
{
  while (true)
  {
    if(checkSerialQuit()) break;

    // Print the variable
    if(millis() - last_packet < usrConf.failsafe_time)
    {
      Serial.print("RSSI: ");
      Serial.print(radio.getRSSI());
      Serial.print(", SNR: ");
      Serial.println(radio.getSNR());
    }
    else
    {
      Serial.print("Failsafe since (ms) ");
      Serial.println(millis()-last_packet);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void serPrintPWM()
{
  while (true)
  {
    if(checkSerialQuit()) break;

    // Print the variable
    Serial.print(PWM0_time);
    Serial.print(", ");
    Serial.println(PWM1_time);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void serPrintReceived()
{
  while (true)
  {
    if(checkSerialQuit()) break;

    // Print received throttle/steering in JSON format for test correlation
    Serial.print("{\"throttle\":");
    Serial.print(thr_received);
    Serial.print(",\"steering\":");
    Serial.print(steering_received);
    Serial.print(",\"rssi\":");
    Serial.print(radio.getRSSI());
    Serial.print(",\"snr\":");
    Serial.print(radio.getSNR());
    Serial.println("}");

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void checkButtons()
{
  if(!aw.digitalRead(AP_S_BIND))
  {
    if(!aw.digitalRead(AP_S_AUX))
    {
      delay(10);
      if(!aw.digitalRead(AP_S_AUX))
      {
        Serial.println("Deleting config and rebooting");
        deleteConfFromSPIFFS();
        delay(1000);
        ESP.restart();
      }
    }
    delay(10);
    if(!aw.digitalRead(AP_S_BIND))
    {
      //Start pairing
      waitForPairing();
    }
  }
}

void checkConnStatus(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(200);

  while (1)
  {
    esp_task_wdt_reset();
    if(usrConf.paired)
    {
      if(millis() - last_packet < usrConf.failsafe_time)
      {
        if(bind_pin_state != 1)
        {
          bind_pin_state = 1;
          aw.digitalWrite(AP_L_BIND, LOW);
        }
      }
      else
      {
        if(bind_pin_state)
        {
          bind_pin_state = 0;
          aw.digitalWrite(AP_L_BIND, HIGH);
        }
        else
        {
          bind_pin_state = 1;
          aw.digitalWrite(AP_L_BIND, LOW);
        }
      }
    }
    else
    {
      unpairedBlink++;
      if(unpairedBlink == 4)
      {
        unpairedBlink = 0;
        aw.digitalWrite(AP_L_BIND, LOW);
        vTaskDelay(pdMS_TO_TICKS(10));
        aw.digitalWrite(AP_L_BIND, HIGH);
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
