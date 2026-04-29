// V3 - 2026-04-24 - Added ?printgps, ?gpsraw, ?gpsreinit serial commands for TX GPS diagnostics
// V3 - 2026-04-22 - P4: signal-drop haptic warning (Pattern A) when sq_graph drops to 1 while connected
const char* SYS_DEVICE_LABEL = "TX";

void deepSleep()
{
  if(!isDisplayActivityEnabled())
  {
    setDisplayActivityEnabled(true);
  }
  displayDigits(LET_X, LET_X);
  updateDisplay();
  setBrightness(0x00);
  Serial.println("Going to sleep now");
  setRadioActivityEnabled(false);
  Serial.flush();
  esp_deep_sleep_start();
}

String checkHWConfig()
{
  //Not sure why this is necessary, otherwise pullup is too strong
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
  digitalWrite(18, LOW);

  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);
  digitalWrite(19, LOW);

  uint8_t pin_id = 0;

  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);

  pin_id |= (digitalRead(18)&0x01)<<0;
  pin_id |= (digitalRead(19)&0x01)<<4;

  pinMode(18, INPUT_PULLDOWN);
  pinMode(19, INPUT_PULLDOWN);

  pin_id |= (digitalRead(18)&0x01)<<1;
  pin_id |= (digitalRead(19)&0x01)<<5;

  if(pin_id == 0x11)
  {
    return "Unknown";
  }

  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);

  pin_id |= (digitalRead(19)&0x01)<<6;

  digitalWrite(18, LOW);

  pin_id |= (digitalRead(19)&0x01)<<7;

  pinMode(18, INPUT_PULLUP);
  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);

  pin_id |= (digitalRead(18)&0x01)<<2;

  digitalWrite(19, LOW);

  pin_id |= (digitalRead(18)&0x01)<<3;

  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);

  //Serial.println(pin_id, HEX);

  switch (pin_id)
  {
    case 0x77:
      // b8
      return "Beta_8M";
      break;
    case 0x44:
      // b9
      return "Beta_9M";
      break;
    case 0x40:
      // g18
      return "Gen1_8M";
      break;
    case 0x04:
      // g19
      return "Gen1_9M";
      break;
    case 0xF7:
      // g28
      return "Gen2_8M";
      break;
    case 0x7F:
      // g29
      return "Gen2_9M";
      break;
    case 0xFF:
      // o18
      return "Diy1_8M";
      break;
    case 0x00:
      // o19
      return "Diy1_9M";
      break;
    case 0x0F:
      // o28
      return "Diy2_8M";
      break;
    case 0xF0:
      // o29
      return "Diy2_9M";
      break;

    default:
      return "Unknown";
      break;
  }
}

void checkStartupButtons()
{
  if(thr_scaled > 100)
  {
    if(tog_input == 1)
    {
      //Delete SPIFFS
      Serial.println("Deleting conf from SPIFFS & rebooting...");
      deleteConfFromSPIFFS();
      scroll3Digits(LET_D, LET_E, LET_L, 200);
      scroll3Digits(LET_D, LET_E, LET_L, 200);
      scroll3Digits(LET_D, LET_E, LET_L, 200);
      ESP.restart();
    }
    else if (tog_input == -1)
    {
      //USB Mode
      Serial.println("USB Mode, type '?' for info...");
      while(1)
      {
        scroll3Digits(LET_U, 5, LET_B, 200);
        checkSerial();
      }
    }
  }
  else
  {
    if(tog_input == 1)
    {
      //Pairing
      usrConf.paired = 0;
      checkPairing();
    }
    else if (tog_input == -1)
    {
      //Calib
      usrConf.cal_ok = 0;
      checkCal();
    }
  }
}

// ===== TX-Specific Command Handlers =====

typedef void (*CmdHandler)(const String &args);

struct CmdEntry {
  const char *name;
  CmdHandler handler;
  const char *usage;
  const char *help;
};

void cmdSetConf(const String &args) {
  if(args.length() == 0) { Serial.println("ERR: usage: ?setconf <base64>"); return; }
  serSetConf(args);
}

void cmdApplyConf(const String &args) {
  serApplyConf();
}

void cmdClearSpiffs(const String &args) {
  serClearConf();
}

void cmdDisplay(const String &args) {
  if(args == "on")       { setDisplayActivityEnabled(true); Serial.println("Display activity enabled."); }
  else if(args == "off") { setDisplayActivityEnabled(false); Serial.println("Display activity disabled."); }
  else if(args == "")    { Serial.print("display="); Serial.println(isDisplayActivityEnabled() ? "ON" : "OFF"); }
  else                   Serial.println("ERR: usage: ?display on|off");
}

void cmdRadio(const String &args) {
  if(args == "on")       { setRadioActivityEnabled(true); Serial.println("Radio activity enabled."); }
  else if(args == "off") { setRadioActivityEnabled(false); Serial.println("Radio activity disabled."); }
  else if(args == "")    { Serial.print("radio="); Serial.println(isRadioActivityEnabled() ? "ON" : "OFF"); }
  else                   Serial.println("ERR: usage: ?radio on|off");
}

void cmdHall(const String &args) {
  if(args == "on")       { setHallActivityEnabled(true); Serial.println("Hall activity enabled."); }
  else if(args == "off") { setHallActivityEnabled(false); Serial.println("Hall activity disabled."); }
  else if(args == "")    { Serial.print("hall="); Serial.println(isHallActivityEnabled() ? "ON" : "OFF"); }
  else                   Serial.println("ERR: usage: ?hall on|off");
}

void cmdAll(const String &args) {
  if(args == "on") {
    setHallActivityEnabled(true);
    setRadioActivityEnabled(true);
    setDisplayActivityEnabled(true);
    Serial.println("All activity gateways enabled.");
  }
  else if(args == "off") {
    setDisplayActivityEnabled(false);
    setRadioActivityEnabled(false);
    setHallActivityEnabled(false);
    Serial.println("All activity gateways disabled.");
  }
  else Serial.println("ERR: usage: ?all on|off");
}

void cmdState(const String &args) {
  serPrintStatus(args == "json");
}

void cmdPrintRSSI(const String &args) {
  serPrintRSSI(args == "json");
}

void cmdPrintInputs(const String &args) {
  serPrintInputs(args == "json");
}

void cmdPrintTasks(const String &args) {
  serPrintTasks(args == "json");
}

void cmdPrintPackets(const String &args) {
  serPrintPackets(args == "json");
}

void cmdWifiStop(const String &args) {
#ifdef WIFI_ENABLED
  webCfgNotifyTxUnlocked();
  Serial.println("TX unlock notified: AP will stop.");
#else
  Serial.println("ERR: WiFi disabled at compile time");
#endif
}

void cmdExitChg(const String &args) {
  Serial.println(" Exit by user");
  exitChargeScreen = 1;
}

// ---- ?printgps : snapshot of all GPS state ----
void cmdPrintGPS(const String &args) {
  Serial.println("----- TX GPS Status -----");
  Serial.print("gps_en:             "); Serial.println(usrConf.gps_en);
  Serial.print("gps_chip_type:      "); Serial.println(usrConf.gps_chip_type);
  Serial.print("speed_src:          "); Serial.println(usrConf.speed_src);
  Serial.print("tx_gps_initialized: "); Serial.println(tx_gps_initialized ? "YES" : "NO");
  Serial.print("Serial1 available:  "); Serial.println(Serial1.available());
  Serial.print("Chars processed:    "); Serial.println(gps_tx.charsProcessed());
  Serial.print("Sentences failed:   "); Serial.println(gps_tx.failedChecksum());
  Serial.print("Location valid:     "); Serial.println(gps_tx.location.isValid() ? "YES" : "NO");
  Serial.print("Location age (ms):  "); Serial.println(gps_tx.location.age());
  Serial.print("Latitude:           "); Serial.println(gps_tx.location.lat(), 6);
  Serial.print("Longitude:          "); Serial.println(gps_tx.location.lng(), 6);
  Serial.print("Speed valid:        "); Serial.println(gps_tx.speed.isValid() ? "YES" : "NO");
  Serial.print("Satellites:         ");
  Serial.println(gps_tx.satellites.isValid() ? String(gps_tx.satellites.value()) : "invalid");
  Serial.print("HDOP:               ");
  Serial.println(gps_tx.hdop.isValid() ? String(gps_tx.hdop.value()) : "invalid");
  Serial.print("tx_gps_speed:       "); Serial.println(tx_gps_speed);
  Serial.println("-------------------------");
}

// ---- ?gpsraw : dump raw NMEA bytes from Serial1 for 5 seconds ----
// Tells us immediately if GPS module is alive and at what baud rate.
// See $GPRMC/$GPGGA lines = GPS alive and parsing correctly.
// See garbage/symbols = wrong baud rate.
// See nothing = wiring or power problem.
void cmdGpsRaw(const String &args) {
  Serial.println("--- Raw GPS bytes from Serial1 (5 seconds, type q to quit) ---");
  if (!tx_gps_initialized) {
    Serial.println("WARNING: tx_gps_initialized=false. Serial1 may not be configured.");
    Serial.println("Try ?gpsreinit first, then ?gpsraw again.");
  }
  unsigned long start = millis();
  unsigned long duration = 5000;
  // Optional: allow custom duration e.g. ?gpsraw 10 for 10 seconds
  if (args.length() > 0) {
    int sec = args.toInt();
    if (sec > 0 && sec <= 30) duration = sec * 1000UL;
  }
  while (millis() - start < duration) {
    if (checkSerialQuit()) break;
    while (Serial1.available()) {
      char c = (char)Serial1.read();
      Serial.print(c);
    }
    delay(10);
  }
  Serial.println("\n--- End raw GPS dump ---");
}

// ---- ?gpsreinit : re-run initTxGPS() without rebooting ----
// Useful for bench testing different chip types or baud rates.
// Change gps_chip_type via ?set gps_chip_type 2 then ?gpsreinit to test.
void cmdGpsReinit(const String &args) {
  Serial.println("Re-running initTxGPS()...");
  tx_gps_initialized = false;
  Serial1.end();
  delay(100);
  initTxGPS();
  Serial.print("tx_gps_initialized after reinit: ");
  Serial.println(tx_gps_initialized ? "YES" : "NO");
  Serial.println("Run ?gpsraw to check Serial1 output.");
}

// ===== Dispatch Table =====

const CmdEntry cmdTable[] = {
  { "conf",         cmdConf,         "",                "print info and usrConf" },
  { "setconf",      cmdSetConf,      "<data>",          "write B64 to SPIFFS" },
  { "applyconf",    cmdApplyConf,    "",                "apply SPIFFS config to RAM" },
  { "clearspiffs",  cmdClearSpiffs,  "",                "delete stored config" },
  { "get",          cmdGet,          "<key>",           "get config value by name" },
  { "set",          cmdSet,          "<key> <value>",   "set config value in RAM" },
  { "save",         cmdSave,         "",                "persist RAM config to SPIFFS" },
  { "keys",         cmdKeys,         "",                "list config field names" },
  { "wifi",         cmdWifi,         "[on|off]",        "WiFi/AP config service" },
  { "display",      cmdDisplay,      "[on|off]",        "display activity" },
  { "radio",        cmdRadio,        "[on|off]",        "radio activity" },
  { "hall",         cmdHall,         "[on|off]",        "hall sampling activity" },
  { "all",          cmdAll,          "[on|off]",        "all subsystems" },
  { "state",        cmdState,        "[json]",          "subsystem state overview" },
  { "printrssi",    cmdPrintRSSI,    "[json]",          "live RSSI/SNR (quit to stop)" },
  { "printinputs",  cmdPrintInputs,  "[json]",          "live input values (quit to stop)" },
  { "printtasks",   cmdPrintTasks,   "[json]",          "task stack usage (quit to stop)" },
  { "printpackets", cmdPrintPackets, "[json]",          "TX/RX packet counts" },
  { "wifidbg",      cmdWifiDbg,      "[some|full|off]", "get/set wifi debug mode" },
  { "wifips",       cmdWifiPs,       "[<ms>|off]",      "get/set AP startup timeout" },
  { "wifistop",     cmdWifiStop,     "",                "notify TX unlock, stop AP" },
  { "wifiver",      cmdWifiVer,      "",                "print web UI version info" },
  { "wifiupd",      cmdWifiUpd,      "",                "force web UI update to SPIFFS" },
  { "wifistate",    cmdWifiState,    "",                "wifi config state/counters" },
  { "wifierr",      cmdWifiErr,      "",                "last wifi config error" },
  { "reboot",       cmdReboot,       "",                "reboot the remote" },
  { "exitchg",      cmdExitChg,      "",                "exit charge screen" },
  { "printgps",    cmdPrintGPS,     "",                "TX GPS state and fix status" },
  { "gpsraw",      cmdGpsRaw,       "[sec]",           "dump raw Serial1 NMEA output (default 5s)" },
  { "gpsreinit",   cmdGpsReinit,    "",                "re-run initTxGPS() without rebooting" },
};
const size_t cmdTableSize = sizeof(cmdTable) / sizeof(cmdTable[0]);

// ===== Help (auto-generated from table) =====

void cmdHelp() {
  Serial.println("Possible commands:");
  Serial.println("");
  for(size_t i = 0; i < cmdTableSize; i++) {
    char line[40];
    if(strlen(cmdTable[i].usage) > 0)
      snprintf(line, sizeof(line), "  ?%s %s", cmdTable[i].name, cmdTable[i].usage);
    else
      snprintf(line, sizeof(line), "  ?%s", cmdTable[i].name);
    Serial.printf("%-30s - %s\n", line, cmdTable[i].help);
  }
}

// ===== Parser + Dispatcher =====

void parseCommand(const String &input, String &cmd, String &args)
{
  // Strip leading '?'
  String body = input.substring(1);

  // Handle both colon separator (legacy ?setconf:data) and space separator
  int colonPos = body.indexOf(':');
  int spacePos = body.indexOf(' ');

  int sep;
  if(colonPos >= 0 && (spacePos < 0 || colonPos < spacePos))
    sep = colonPos;  // colon comes first (legacy ?setconf:data)
  else
    sep = spacePos;

  if(sep < 0) {
    cmd = body;
    args = "";
  } else {
    cmd = body.substring(0, sep);
    args = body.substring(sep + 1);
    args.trim();
  }
}

void dispatchCommand(const String &input)
{
  if(input == "?") { cmdHelp(); return; }
  if(!input.startsWith("?")) {
    Serial.println("Unknown command. Type '?' for help.");
    return;
  }

  // Parse with original case preserved in args
  String cmd, args;
  parseCommand(input, cmd, args);
  // Lowercase the command name for matching
  cmd.toLowerCase();
  // Lowercase args copy for commands that compare against fixed keywords
  String argsLower = args;
  argsLower.toLowerCase();

  for(size_t i = 0; i < cmdTableSize; i++)
  {
    if(cmd == cmdTable[i].name)
    {
      // Commands that need original-case args: setconf, get, set, webdbg, wifips
      // Commands that compare against keywords (on/off/json): use lowered args
      if(cmd == "setconf" || cmd == "get" || cmd == "set" || cmd == "wifidbg" || cmd == "wifips")
        cmdTable[i].handler(args);
      else
        cmdTable[i].handler(argsLower);
      return;
    }
  }
  Serial.println("Unknown command. Type '?' for help.");
}

void checkSerial()
{
  if(serialOff) return;
  if(Serial.available() <= 0) return;

  String command = Serial.readStringUntil('\n');
  command.trim();

  dispatchCommand(command);
}

void serPrintTasks(bool json)
{
  while (true)
  {
    if(checkSerialQuit()) break;

    if(json)
    {
      Serial.printf("{\"sendData\":%u,\"telemetry\":%u,\"measBufCalc\":%u,\"bargraph\":%u,\"vibration\":%u,\"loop\":%u}\n",
        uxTaskGetStackHighWaterMark(sendDataHandle),
        uxTaskGetStackHighWaterMark(triggeredWaitForTelemetryHandle),
        uxTaskGetStackHighWaterMark(measBufCalcHandle),
        uxTaskGetStackHighWaterMark(updateBargraphsHandle),
        uxTaskGetStackHighWaterMark(vibrationTaskHandle),
        uxTaskGetStackHighWaterMark(loopTaskHandle));
    }
    else
    {
      Serial.println("\n=== Task Stack Usage ===");
      Serial.printf("sendData stack left: %u words\n", uxTaskGetStackHighWaterMark(sendDataHandle));
      Serial.printf("telemetry stack left: %u words\n", uxTaskGetStackHighWaterMark(triggeredWaitForTelemetryHandle));
      Serial.printf("measBufCalc stack left: %u words\n", uxTaskGetStackHighWaterMark(measBufCalcHandle));
      Serial.printf("bargraph stack left: %u words\n", uxTaskGetStackHighWaterMark(updateBargraphsHandle));
      Serial.printf("vibration stack left: %u words\n", uxTaskGetStackHighWaterMark(vibrationTaskHandle));
      Serial.printf("loop() stack left: %u words\n", uxTaskGetStackHighWaterMark(loopTaskHandle));
      Serial.println("========================\n");
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void serPrintPackets(bool json)
{
  if(json)
  {
    float ratio = num_sent_packets > 0 ? ((float)num_rcv_packets/(float)num_sent_packets)*100 : 0;
    Serial.printf("{\"sent\":%lu,\"received\":%lu,\"ratio\":%.2f}\n", num_sent_packets, num_rcv_packets, ratio);
  }
  else
  {
    Serial.print("Sent: ");
    Serial.println(num_sent_packets);
    Serial.print("Received: ");
    Serial.println(num_rcv_packets);
    Serial.print("Ratio: ");
    if(num_sent_packets > 0)
    {
      Serial.print(((float)num_rcv_packets/(float)num_sent_packets)*100);
      Serial.println(" %");
    }
    else
    {
      Serial.println("N/A");
    }
  }
}

void serPrintRSSI(bool json)
{
  while (true)
  {
    if(checkSerialQuit()) break;

    if(json)
    {
      if(!isRadioActivityEnabled())
        Serial.println("{\"error\":\"radio_disabled\"}");
      else if(millis()-last_packet < 1000)
        Serial.printf("{\"rssi\":%d,\"snr\":%.1f}\n", radio.getRSSI(), radio.getSNR());
      else
        Serial.printf("{\"failsafe_ms\":%lu}\n", millis()-last_packet);
    }
    else
    {
      if(!isRadioActivityEnabled())
      {
        Serial.println("Radio activity is disabled.");
      }
      else if(millis()-last_packet < 1000)
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
    }
    delay(50);
  }
}

void serPrintInputs(bool json)
{
  while (true)
  {
    if(checkSerialQuit()) break;
    if(in_menu > 0) in_menu--;

    if(json)
    {
      Serial.printf("{\"throttle\":%d,\"steering\":%d,\"thr_sent\":%d,\"steer_sent\":%d,\"toggle\":%d,\"toggle_input\":%d,\"locked\":%d,\"in_menu\":%d,\"steer_enabled\":%d,\"hall_enabled\":%d}\n",
        thr_scaled, steer_scaled, thr_sent, steer_sent, tog_scaled, tog_input,
        system_locked ? 1 : 0, in_menu, usrConf.steer_enabled,
        isHallActivityEnabled() ? 1 : 0);
    }
    else
    {
      Serial.print("Throttle: ");
      Serial.print(thr_scaled);
      Serial.print(", Steering: ");
      Serial.print(steer_scaled);
      Serial.print(", Toggle: ");
      Serial.print(tog_scaled);
      Serial.print(", ToggleInput: ");
      Serial.print(tog_input);
      Serial.print(", Locked: ");
      Serial.print(system_locked ? 1 : 0);
      Serial.print(", InMenu: ");
      Serial.print(in_menu);
      Serial.print(", SteerEn: ");
      Serial.print(usrConf.steer_enabled);
      Serial.print(", HallEn: ");
      Serial.println(isHallActivityEnabled() ? 1 : 0);
    }
    delay(50);
  }
}

void serPrintStatus(bool json)
{
  if(json)
  {
    Serial.printf("{\"hall\":\"%s\",\"radio\":\"%s\",\"display\":\"%s\",\"wifi\":\"%s\",\"locked\":%s,\"paired\":%s,\"throttle_mode\":%d,\"gear\":%d,\"max_gears\":%d,\"max_power_cap\":%d,\"error\":%d,\"last_pkt_ms\":%lu}\n",
      isHallActivityEnabled() ? "ON" : "OFF",
      isRadioActivityEnabled() ? "ON" : "OFF",
      isDisplayActivityEnabled() ? "ON" : "OFF",
#ifdef WIFI_ENABLED
      web_cfg_service_enabled ? "ON" : "OFF",
#else
      "DISABLED",
#endif
      system_locked ? "true" : "false",
      usrConf.paired ? "true" : "false",
      usrConf.throttle_mode, gear, usrConf.max_gears, max_power_cap, remote_error,
      millis() - last_packet);
  }
  else
  {
    Serial.println("--- Status ---");
    Serial.print("Hall:    "); Serial.println(isHallActivityEnabled() ? "ON" : "OFF");
    Serial.print("Radio:   "); Serial.println(isRadioActivityEnabled() ? "ON" : "OFF");
    Serial.print("Display: "); Serial.println(isDisplayActivityEnabled() ? "ON" : "OFF");
    Serial.print("WiFi AP: ");
#ifdef WIFI_ENABLED
    Serial.println(web_cfg_service_enabled ? "ON" : "OFF");
#else
    Serial.println("DISABLED");
#endif
    Serial.print("Locked:  "); Serial.println(system_locked ? "YES" : "NO");
    Serial.print("Paired:  "); Serial.println(usrConf.paired ? "YES" : "NO");
    Serial.print("Thr Mode: "); Serial.println(usrConf.throttle_mode == 0 ? "Gears" : usrConf.throttle_mode == 1 ? "No Gears" : "Dynamic Cap");
    if(usrConf.throttle_mode == 0) { Serial.print("Gear:    "); Serial.print(gear); Serial.print("/"); Serial.println(usrConf.max_gears); }
    if(usrConf.throttle_mode == 2) { Serial.print("Cap:     "); Serial.print(max_power_cap); Serial.println("%"); }
    Serial.print("Error:   "); Serial.println(remote_error);
    Serial.print("Last pkt (ms ago): "); Serial.println(millis() - last_packet);
    Serial.println("--------------");
  }
}

void checkCharger()
{
  uint8_t chg_err_cnt = 0;
  Serial.print("Checking if charging...");

  while(!exitChargeScreen)
  {
#ifdef WIFI_ENABLED
    webCfgLoop();
#endif
    ads.startADCReading(MUX_BY_CHANNEL[P_CHGSTAT],false);
    while(!ads.conversionComplete())
    {
#ifdef WIFI_ENABLED
      webCfgLoop();
#endif
      delay(1);
    }
    uint16_t chgstat = ads.getLastConversionResults();

    ads.startADCReading(MUX_BY_CHANNEL[P_UBAT_MEAS],false);
    while(!ads.conversionComplete())
    {
#ifdef WIFI_ENABLED
      webCfgLoop();
#endif
      delay(1);
    }
    uint16_t bat_volt = ads.getLastConversionResults();
    uint16_t c_bat_volt = (uint16_t)((float)bat_volt * usrConf.ubat_cal * 100.0);


    if(chgstat < 1000)
    {
      //Not charging
      Serial.println(" Done");
      serialOff = true;
      break;
    }
    else if(chgstat > 6000 && chgstat < 10000)
    {
      setBrightness(0x01);
      advanceChargeAnimation();
      uint8_t chglevel = map(c_bat_volt, 330, 420, 0, 10);
      displayHorzBargraph(7,chglevel);
      updateDisplay();
      checkSerial();
#ifdef WIFI_ENABLED
      webCfgLoop();
#endif
      delay(200);
    }
    else if(chgstat > 10000 && chgstat < 18000)
    {
      setBrightness(0x01);
      displayBuffer[1] = (displayBuffer[1] & 0xFF80) | 0x1F;  // I-1: preserve bit 7 (GPS dot)
      displayBuffer[4] = 0x1F;
      displayBuffer[2] = 0x3F;
      displayBuffer[3] = 0x3F;
      displayHorzBargraph(7,10);
      updateDisplay();
      checkSerial();
#ifdef WIFI_ENABLED
      webCfgLoop();
#endif
      delay(200);
    }
    else
    {
      chg_err_cnt++;
      Serial.print("Count: ");
      Serial.println(chg_err_cnt);
      Serial.print("Stat: ");
      Serial.println(chgstat);
#ifdef WIFI_ENABLED
      webCfgLoop();
#endif
      delay(10);
      if(chg_err_cnt > 10)
      {
        setBrightness(0x01);
        Serial.println("CHG ERR!");
        Serial.print("Stat: ");
        Serial.println(chgstat);
        int timeout = 0;
        while(timeout < 3)
        {
          timeout++;
          scroll3Digits(LET_E, LET_C, LET_H, 100);
        }
        exitChargeScreen = 1;
        serialOff = true;
      }
    }
  }
  displayHorzBargraph(7,0);
  setBrightness(0x0F);
}

volatile uint8_t current_vib_pattern = 0;  // active haptic pattern: 0=none, 1=2 short, 2=5 short, 3=5 long, 4=2 fast short (RTM arm/disarm)

void vibrationTask(void *parameter) {
  uint8_t last_error = 0;
  bool was_connected = true;
  bool bat_warning_sent = false;
  // Signal-drop warning state (Priority 4)
  uint8_t last_sq   = 0;     // sq_graph reading from previous loop iteration
  bool    sq_warned  = false; // true after Pattern A fired; cleared when signal recovers
  bool    last_con   = true;  // connection state from previous iteration

  while (1) {
    // --- 1. MONITOR SYSTEM STATES ---
    
    // Check for Radio Failsafe (Connection Lost)
    bool is_connected = (millis() - last_packet < 1000);
    if (was_connected && !is_connected) {
      current_vib_pattern = 2; // Pattern B: 5 Short (Urgent!)
    }
    was_connected = is_connected;

    // Check for Weak LoRa Signal (sq_graph drops to 1 while connected)
    // sq_graph == 1 means one bar of signal left — warn before full failsafe (sq_graph == 0).
    // Guard: only while connected; during failsafe, updateBargraphs() toggles sq_graph 0↔1
    // as a display artifact — that state is already covered by Pattern B above.
    // Re-arm when signal recovers above 1 so the warning can fire again on the next drop.
    {
      uint8_t cur_sq = sq_graph; // snapshot volatile once
      if (is_connected) {
        if (!last_con) {
          // Just reconnected — seed last_sq to suppress a spurious edge on reconnect
          last_sq   = cur_sq;
          sq_warned = false;
        } else if (last_sq > 1 && cur_sq == 1 && !sq_warned) {
          // Signal just dropped to 1 bar — fire Pattern A if nothing else is playing
          if (current_vib_pattern == 0) {
            current_vib_pattern = 1; // Pattern A: 2 Short (weak signal warning)
          }
          sq_warned = true;
        } else if (cur_sq > 1) {
          // Signal recovered — allow warning to fire again on next drop
          sq_warned = false;
        }
        last_sq = cur_sq;
      } else {
        // In failsafe — reset so warning re-arms on reconnect
        sq_warned = false;
      }
      last_con = is_connected;
    }

    // Check for Critical Errors (Like E71 Water Ingress)
    if (remote_error != last_error) {
      if (remote_error == 71) {
        current_vib_pattern = 3; // Pattern C: 5 Long (Emergency!)
      }
      last_error = remote_error;
    }

    // Check for Low VESC Battery (20% or less)
    if (telemetry.foil_bat != 0xFF && telemetry.foil_bat <= 20) {
      if (!bat_warning_sent) {
        current_vib_pattern = 1; // Pattern A: 2 Short (Warning)
        bat_warning_sent = true;
      }
    } else if (telemetry.foil_bat > 20) {
      bat_warning_sent = false; // Reset if battery is changed
    }

    // --- 2. EXECUTE THE PATTERNS ---
    
    if (current_vib_pattern == 1) { 
      // PATTERN A: 2 Short
      for(int i=0; i<2; i++) { 
        digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(150)); 
        digitalWrite(P_MOT, LOW);  vTaskDelay(pdMS_TO_TICKS(150)); 
      }
      current_vib_pattern = 0;
    }
    else if (current_vib_pattern == 2) { 
      // PATTERN B: 5 Short
      for(int i=0; i<5; i++) { 
        digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(150)); 
        digitalWrite(P_MOT, LOW);  vTaskDelay(pdMS_TO_TICKS(150)); 
      }
      current_vib_pattern = 0;
    }
    else if (current_vib_pattern == 3) {
      // PATTERN C: 5 Long
      for(int i=0; i<5; i++) {
        digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(500));
        digitalWrite(P_MOT, LOW);  vTaskDelay(pdMS_TO_TICKS(300));
      }
      current_vib_pattern = 0;
    }
    // V3 - 2026-04-27 - P8: Pattern 4 — 2 fast short pulses (RTM arm/disarm confirm)
    else if (current_vib_pattern == 4) {
      for (int i = 0; i < 2; i++) {
        digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(80));
        digitalWrite(P_MOT, LOW);  vTaskDelay(pdMS_TO_TICKS(80));
      }
      current_vib_pattern = 0;
    }

    // Sleep briefly to prevent hoarding the CPU
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}