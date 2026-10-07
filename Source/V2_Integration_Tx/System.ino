// V2.5-Evo - 2026-10-07 - R-7: Pattern 5 (one short blip) gains a caller - the mag_mode 4 hold refusal when
//   Return-To-Me cannot start. Comment only in this file; no pattern changed, no confStruct change.
// V2.5-Evo - 2026-09-30 - MagFix (Rex delta audit): haptic Pattern 12 added — THREE FIRM taps (130 ms on /
//   250 ms off, the Pattern 4 shape) = a mag_mode 4 magnet hold switched Return-To-Me OFF for the session.
//   It replaces the vib_stop_pending / Pattern 7 long buzz that confirm used to borrow: Pattern 7 means "a
//   FAULT stopped the system", its own contract says it does not fire on a deliberate disarm, and it preempts
//   every other pattern. ON stays Pattern 4 (two firm taps), so ON and OFF now differ only in tap count.
//   New pattern only — no other pattern changed, no confStruct change, sizeof stays 136, SW_VERSION 27.
// V2.5-Evo - 2026-09-30 - MagStations: haptic Pattern 11 added — N short taps, where N is the Follow-Me station
//   number a magnet tap just selected (count carried by the new volatile vib_pulse_count). Pattern 10 gained a second
//   caller (the mag_mode 4 2.5 s advisory) and its comment now says so. No other pattern changed, no confStruct change.
// V2.5-Evo - 2026-09-19 - Return gesture: vibrationTask Patterns 9 and 10 added. 9 = FOUR quick 80 ms taps (a trill) = the
//   auto-return override was SET for the session (RTM arm cancelled, override = opposite of the RX's echo); 10 = TWO medium
//   300 ms pulses = the override was CLEARED (back to the buggy's stored default). Both are new shapes: no existing pattern is
//   four quick taps or two medium pulses, so neither can be confused by feel with the two firm arm taps (4), the one long
//   STOP (7) or the single 300 ms warning (8). Both break early for a pending STOP.
// V2.5-Evo - 2026-09-17 - WarnDist: vibrationTask Pattern 8 added — ONE medium 300 ms pulse, the FM warning-distance
//   haptic. Queued by runFmLoop() (RTMState.ino) immediately when the buggy reaches fm_warn_distance_m and then every
//   2 s while it stays there; runs with the trigger released. Shorter than the 750 ms STOP, longer than the 150 ms advisory.
// V2.5-Evo - 2026-09-17 - CHG-ADS FIX: adsReadGuarded() took the ADS1115 mux constant as uint8_t, which truncated
//   every MUX_BY_CHANNEL[] value (0x4000..0x7000, bits 14:12) to 0 -> every charge-screen read was differential
//   AIN0-AIN1, never the charger-status or battery pin. The signed int16_t conversion result was also stored
//   unsigned. Signature is now (uint16_t mux, int16_t &out); chgstat/bat_volt are int16_t; a negative bat_volt
//   becomes 0 before the voltage bargraph. Analog.ino was never affected (it calls the library directly).
// V2.5-Evo - 2026-08-17 - StopBuzz FIX (vibrationTask): three delivery bugs in the haptics chain.
//   1. Pattern 7 (STOP) could be silently lost — every executor branch ended with an unconditional
//      current_vib_pattern = 0, wiping a stop queued while another pattern was mid-play (up to 4s
//      for Pattern 3). Stop requests now go through vib_stop_pending, are promoted ahead of
//      everything else each cycle, and cut a multi-pulse pattern short between pulses. Every other
//      branch now clears only if the pattern it played is still the one queued.
//   2. The weak-signal warning latched sq_warned even when the collision guard stopped the buzz
//      from ever being queued, so it was never retried. It now latches only on a real queue.
//   3. The low-battery warning wrote current_vib_pattern unconditionally, letting the LEAST
//      important alert overwrite a same-pass failsafe or E71 water-ingress buzz. Now guarded.
// V2.5-Evo - 2026-07-21 - DIAG: ?state (serPrintStatus) now prints a BLE status row in both the text and
//   json branches, guarded by BLE_ENABLED (fallback "DISABLED (compiled out)" when built without BLE).
// V2.5-Evo - 2026-07-20 - StopFeel: vibration Pattern 7 added — one 400ms LONG pulse = the dedicated
//   STOP/DISARM confirm for FM and RTM. Arm confirms stay Pattern 4 (2×80ms). Result by feel:
//   arm = two quick taps, stop = one long buzz. 400ms (not 350) chosen so it is unmistakable from the
//   single 150ms Pattern 5 advisory while the rider is moving and cannot look at the remote.
// V2.5-Evo - 2026-07-20 - MagGesture: vibration Patterns 5 and 6 added — 5 = one 150ms pulse (magnet 2s "release for FM" advisory);
//   6 = three 80ms pulses (magnet 5s "release for RTM" advisory). Pattern 6 exists so the RTM advisory is not
//   confused with the Pattern 4 arm confirm that follows it. Feel map: 1→2 = FM armed, 3→2 = RTM armed.
// V2.5-Evo - 2026-05-06 - DIAG: ?gpscoldreset command added
// V2.5-Evo - 2026-05-06 - FIX-HELP-1: corrected raw-GPS-dump help text from "type q to quit" to "type 'quit' to abort"
// V2.5-Evo - 2026-05-03 - Added reserved/warning comments (LOW audit cleanup)
// V2.5-Evo - 2026-04-24 - Added ?printgps, ?gpsraw, ?gpsreinit serial commands for TX GPS diagnostics
// V2.5-Evo - 2026-04-22 - P4: signal-drop haptic warning (Pattern A) when sq_graph drops to 1 while connected
const char* SYS_DEVICE_LABEL = "TX";

void deepSleep()
{
  if(!isDisplayActivityEnabled())
  {
    setDisplayActivityEnabled(true);
  }
  DISP_LOCK();
  displayDigits(LET_X, LET_X);
  updateDisplay();
  DISP_UNLOCK();
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
      // V2.5-Evo - 2026-05-15 - feature/bluetooth: LEFT toggle at boot enables BLE for session.
      // Replaces the old USB charging mode (removed — serial config works in normal mode).
      // bt_session_forced persists until reboot; activates BLE regardless of bt_enabled SPIFFS setting.
      bt_session_forced = true;
      Serial.println("BLE session forced via boot gesture");
      displayDigits(LET_B, LET_T);
      delay(1500);
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

// WARNING: this function uses blocking delay(). Do not call while RTM or FM is
// active — it will freeze the GPS/RTM/FM loop for the duration.
// ---- ?gpsraw : dump raw NMEA bytes from Serial1 for 5 seconds ----
// Tells us immediately if GPS module is alive and at what baud rate.
// See $GPRMC/$GPGGA lines = GPS alive and parsing correctly.
// See garbage/symbols = wrong baud rate.
// See nothing = wiring or power problem.
void cmdGpsRaw(const String &args) {
  Serial.println("--- Raw GPS bytes from Serial1 (5 seconds, type 'quit' to abort) ---");
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

// V2.5-Evo - 2026-05-06 - DIAG: ?gpscoldreset command handler
// ---- ?gpscoldreset : send UBX-CFG-RST cold-restart to GPS module ----
// Clears all ephemeris/almanac/clock data; forces fresh acquisition.
// Use when GPS appears stuck (PPS LED firing but no valid NMEA fix).
void cmdGpsColdReset(const String &args) {
  Serial.println("Sending UBX-CFG-RST cold-restart to GPS module...");
  extern void txGpsColdReset();
  txGpsColdReset();
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
  { "gpscoldreset", cmdGpsColdReset, "",              "send UBX-CFG-RST cold-restart to GPS (clear all cached data)" },
  // V2.5-Evo - 2026-07-29 - reads dynModel back OUT of the module. initTxGPS() checks the
  // UBX ACK, which proves the module accepted the write; this proves what it is running.
  // V2.5-Evo - 2026-07-30 - blocks the MAIN LOOP ~3s (display, RTM/FM, GPS parsing). It does
  // NOT stall the LoRa task — that runs at priority 5 on core 0 — so the RX failsafe is not
  // tripped. The earlier "stalls the LoRa cycle" note was wrong. 'quit' aborts.
  { "gpscfg",      cmdGpsCfg,       "",                "read back live GPS config (dynModel) - verifies initTxGPS() actually took; ~3s main-loop block, quit aborts" },
  // V2.5-Evo - 2026-07-29 - GPS-BAUD-1. M9/M10 dropped UBX-CFG-PRT, so the firmware cannot
  // always move a module's baud on its own. Bare = scan, <rate> = our uart only,
  // "set <rate>" = move the module and ask IT to save it (not usrConf — a confStruct
  // change would bump SW_VERSION and wipe the TX SPIFFS config).
  { "gpsbaud",     cmdGpsBaud,      "[set] [<rate>]",  "scan / set GPS baud; 'set' asks the module to save it (BBR always, flash only if fitted); ~2s block, quit aborts" },
  // V2.5-Evo - 2026-07-30 - GPS-SETUP-1. Run ONCE per assembled remote. Finds the module,
  // raises it to 115200, applies every setting ACK-verified, commits to the MODULE's own
  // non-volatile memory, then reads it back to prove it. Boot then only has to verify.
  { "gpssetup",    cmdGpsSetup,     "",                "ONE-TIME full GPS setup: find, raise to 115200, configure, save to the module permanently, verify (~15s, bench only)" },
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

// WARNING: this function uses blocking delay(). Do not call while RTM or FM is
// active — it will freeze the GPS/RTM/FM loop for the duration.
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

    // V2.5-Evo - 2026-06-05 - DIAG: raw Hall counts for magnet/calibration diagnosis
    uint16_t rawThr = 0, rawTog = 0;
    readFilteredInputs(rawThr, rawTog);

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
      Serial.print(isHallActivityEnabled() ? 1 : 0);
      Serial.print(", RAW_THR: ");
      Serial.print(rawThr);
      Serial.print(", RAW_TOG: ");
      Serial.println(rawTog);
    }
    delay(50);
  }
}

void serPrintStatus(bool json)
{
  if(json)
  {
    // Note: no closing brace here — the BLE fields are appended below so the JSON stays one object.
    Serial.printf("{\"hall\":\"%s\",\"radio\":\"%s\",\"display\":\"%s\",\"wifi\":\"%s\",\"locked\":%s,\"paired\":%s,\"throttle_mode\":%d,\"gear\":%d,\"max_gears\":%d,\"max_power_cap\":%d,\"error\":%d,\"last_pkt_ms\":%lu",
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
    // V2.5-Evo - 2026-07-21 - DIAG: BLE status appended to ?state json. State derived from bleRunning +
    // bleIsConnected(); notify reflects the runtime heap-floor tripwire (ble_notify_heap_suspended).
#ifdef BLE_ENABLED
    Serial.printf(",\"ble\":\"%s\",\"ble_conn\":%u,\"ble_heap\":%u,\"ble_notify\":\"%s\",\"bt_enabled\":%u",
      !bleRunning ? "OFF" : (bleIsConnected() ? "CONNECTED" : "ADVERTISING"),
      (unsigned)((bleRunning && bleServer) ? bleServer->getConnectedCount() : 0),
      (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
      ble_notify_heap_suspended ? "suspended" : "active",
      (unsigned)usrConf.bt_enabled);
#else
    Serial.printf(",\"ble\":\"DISABLED\"");
#endif
    Serial.printf("}\n");
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
    // V2.5-Evo - 2026-07-21 - DIAG: BLE status row. state=OFF(not init/skipped)/ADVERTISING/CONNECTED(n);
    // heap=free internal DRAM; notify reflects the runtime heap-floor tripwire; bt_enabled=SPIFFS config.
    Serial.print("BLE:     ");
#ifdef BLE_ENABLED
    if(!bleRunning) { Serial.print("OFF"); }
    else if(bleIsConnected()) { Serial.print("CONNECTED("); Serial.print(bleServer->getConnectedCount()); Serial.print(")"); }
    else { Serial.print("ADVERTISING"); }
    Serial.print("  heap="); Serial.print(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    Serial.print("  notify="); Serial.print(ble_notify_heap_suspended ? "SUSPENDED" : "active");
    Serial.print("  bt_enabled="); Serial.println(usrConf.bt_enabled);
#else
    Serial.println("DISABLED (compiled out)");
#endif
    Serial.println("--------------");
  }
}

// ============================================================
// adsReadGuarded - read one ADS1115 channel with a DEADLINE and a health check.
//
// 🚨 V2.5-Evo - 2026-07-30 - PH-1. checkCharger() used to spin on
// `while(!ads.conversionComplete()) delay(1);` with no timeout and no check of g_ads_ok.
// If the ADS1115 does not answer — I2C damage, a loose connector, water on the bus, which
// this board has already had (SDA HELD LOW, 2026-07-28) — conversionComplete() never becomes
// true and the remote HANGS AT BOOT, FOREVER. Not degraded: frozen, before the radio task is
// even created, with a blank screen and no serial output past "Checking if charging...".
//
// g_ads_ok already existed (Analog.ino) and already recorded whether the chip came up. It
// simply was not consulted here. A conversion at the default data rate takes ~8 ms, so a
// 200 ms deadline is ~25x margin and cannot false-trip on a healthy part.
//
// Returns false if the ADC is known-bad or does not answer in time. Callers must decide what
// a missing reading MEANS — never treat it as a valid zero.
//
// V2.5-Evo - 2026-09-17 - CHG-ADS FIX. The bug: muxChannel was uint8_t. The Adafruit
// MUX_BY_CHANNEL[] constants are uint16_t with the channel select in bits 14:12
// (0x4000, 0x5000, 0x6000, 0x7000), so the cast to uint8_t dropped them all to 0x0000 —
// the library's "differential AIN0 minus AIN1" mux. Every read made here therefore sampled
// the wrong pair of pins regardless of which channel the caller asked for, which is why the
// charge screen showed a nonsense pack voltage and never saw a real charger-status level.
// getLastConversionResults() also returns a SIGNED int16_t (a differential read can be
// negative); storing it in a uint16_t turned small negatives into ~65000. The fix: pass the
// mux word through untouched as uint16_t and hand the result back as int16_t.
// ============================================================
static bool adsReadGuarded(uint16_t muxChannel, int16_t &out)
{
  if (!g_ads_ok) return false;

  ads.startADCReading(muxChannel, false);
  uint32_t deadline = millis() + 200;

  while (!ads.conversionComplete())
  {
#ifdef WIFI_ENABLED
    webCfgLoop();
#endif
    if ((int32_t)(millis() - deadline) >= 0) return false;
    delay(1);
  }
  out = ads.getLastConversionResults();
  return true;
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
    int16_t chgstat = 0;   // CHG-ADS FIX 2026-09-17: signed, matches the ADS1115 conversion result
    if (!adsReadGuarded(MUX_BY_CHANNEL[P_CHGSTAT], chgstat))
    {
      // ============================================================
      // V2.5-Evo - 2026-07-30 - PH-1 + PH-2. The ADC is dead or mute, so we CANNOT know
      // whether we are charging. Two things must not happen here:
      //
      //   1. We must not wait on it — that is the boot hang described above.
      //   2. We must not fall through to the "not charging" branch, because that sets
      //      serialOff, and setup() then calls Serial.end() AND drives GPIO20/21
      //      (U0RXD/U0TXD) LOW as outputs. A broken ADC would therefore disable the one
      //      channel available to diagnose the broken ADC. That is precisely backwards:
      //      the failure would silence the tool needed to investigate it.
      //
      // So: leave the charge screen, boot normally, and KEEP SERIAL ON. Throttle is
      // unaffected by this decision — Analog.ino already fails throttle to zero when
      // g_ads_ok is false, which is the safe direction.
      // ============================================================
      Serial.println(" ADC NOT RESPONDING");
      Serial.println("CHG: !! ADS1115 did not answer. Cannot tell charging from not charging.");
      Serial.println("CHG: !! Skipping the charge screen and KEEPING SERIAL ON so this is");
      Serial.println("CHG: !! diagnosable. Check the I2C bus — run ?i2c. Throttle reads as");
      Serial.println("CHG: !! zero while the ADC is down, which is the safe direction.");
      serialOff = false;
      exitChargeScreen = 1;
      break;
    }

    int16_t bat_volt = 0;   // CHG-ADS FIX 2026-09-17: signed, matches the ADS1115 conversion result
    if (!adsReadGuarded(MUX_BY_CHANNEL[P_UBAT_MEAS], bat_volt))
    {
      Serial.println("CHG: !! ADS1115 stopped answering mid-read — leaving charge screen, "
                     "serial KEPT ON.");
      serialOff = false;
      exitChargeScreen = 1;
      break;
    }
    // CHG-ADS FIX 2026-09-17: a single-ended read cannot legitimately be negative, but noise on
    // an unloaded input can dip a count or two below zero. Clamp to 0 before the unsigned
    // conversion so a -1 does not become 65535 counts and pin the bargraph at full.
    if (bat_volt < 0) bat_volt = 0;
    uint16_t c_bat_volt = (uint16_t)((float)bat_volt * usrConf.ubat_cal * 100.0);


    if(chgstat < 1000)
    {
      //Not charging
      Serial.println(" Done");
      serialOff = true;
      break;
    }
    // V2.5-Evo - 2026-07-30 - PH-4: lower bound moved 6000 -> 1000, closing the dead band.
    //
    // USB is the ONLY way this remote charges, so USB present means charging. There is no
    // "plugged in but idle" state to distinguish. The band therefore only needs one boundary:
    // below 1000 is no external power, anything above it is charging.
    //
    // The old 6000 floor left 1000-6000 meaning nothing, and the owner's TX sits at ~2154 -
    // stably, across every sample (2153-2157). So a perfectly normal charging remote was
    // counted as eleven errors, shown "ECH", and had its serial port disabled. The reading was
    // never wrong; the threshold was.
    //
    // The 10000-18000 branch below is left alone - it is a distinct display state, not a
    // different verdict about whether power is present.
    else if(chgstat >= 1000 && chgstat < 10000)
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
        // ============================================================
        // V2.5-Evo - 2026-07-30 - PH-3. Observed live on the owner's TX: chgstat reads ~2058,
        // which lands in an UNHANDLED GAP. The branches above cover <1000 (not charging),
        // 6000-10000 and 10000-18000 (charging), leaving 1000-6000 and >18000 with no meaning
        // at all — so a reading in the gap is counted as an error eleven times and then
        // declared a fault.
        //
        // Two things were wrong with what happened next, and both are fixed here:
        //
        //   1. serialOff = true. setup() then calls Serial.end() AND drives GPIO20/21
        //      (U0RXD/U0TXD) low. So an unexplained charge reading DISABLED the only channel
        //      available to explain it — the same inversion PH-2 fixed for a mute ADC,
        //      arriving through a different branch. Serial now stays ON.
        //
        //   2. Three full "ECH" scroll passes at 14 frames x 100 ms is ~4.2 s of boot spent
        //      animating a condition the serial log already states precisely. Reduced to one
        //      pass (~1.4 s) — still unmistakable on the display, without dominating startup.
        //
        // ⚠️ NOT FIXED HERE, because it is a hardware question and guessing would be worse:
        // what a chgstat of ~2058 actually MEANS on this board. It may be a healthy
        // battery-full state, a weak USB supply, or a genuine charger fault. The thresholds
        // need widening or re-centring against measured values, not invented ones. The raw
        // number is printed above so it can be characterised.
        // ============================================================
        setBrightness(0x01);
        Serial.println("CHG ERR!");
        Serial.print("Stat: ");
        Serial.println(chgstat);
        Serial.println("CHG: !! that reading is in an UNMAPPED range (not <1000, not");
        Serial.println("CHG: !! 6000-10000, not 10000-18000). Booting normally and KEEPING");
        Serial.println("CHG: !! SERIAL ON so this stays diagnosable. Charging state unknown.");
        scroll3Digits(LET_E, LET_C, LET_H, 100);
        exitChargeScreen = 1;
        serialOff = false;   // never silence the diagnostic channel over a charge reading
      }
    }
  }
  displayHorzBargraph(7,0);
  setBrightness(0x0F);
}

volatile uint8_t current_vib_pattern = 0;  // active haptic pattern: 0=none, 1=2 short, 2=5 short, 3=5 long, 4=2 fast short (RTM/FM ARM confirm), 5=1 short (magnet 2s "release for FM" advisory), 6=3 fast short (magnet 5s "release for RTM" advisory), 7=1 long (UNCOMMANDED RTM/FM stop, or an arm refusal — request it via vib_stop_pending, never by writing 7 here), 8=1 medium 300ms (FM warning-distance reached; repeats every 2s from runFmLoop), 9=4 quick 80ms taps (return gesture: auto-return override SET for the session), 10=2 medium 300ms pulses (return gesture: override CLEARED, back to the stored default), 11=vib_pulse_count short taps (magnet station change: the tap count IS the station number — see the MagStations note below), 12=3 firm 130ms taps (magnet hold turned Return-To-Me OFF for the session — deliberately NOT the Pattern 7 stop buzz; see the note on Pattern 12 below)
// V2.5-Evo - 2026-09-30 - MagStations: how many taps Pattern 11 plays. It is the Follow-Me station number
// (1-3), so the rider counts the buzzes and knows where the buggy just went without looking at the display.
// WRITE THIS FIRST, then current_vib_pattern = 11 — the vibration task reads the count when it starts the
// pattern, and a count of 0 would silently play nothing. It is clamped to 1-5 inside the task so a stray
// value can never spin the motor for an unbounded time.
volatile uint8_t vib_pulse_count = 0;

// ============================================================
// STOP-BUZZ REQUEST FLAG - how Pattern 7 gets to actually play
// ============================================================
// Set true (never false) by any code that needs the long STOP buzz; cleared only by vibrationTask
// when it promotes the request into current_vib_pattern. Writing current_vib_pattern = 7 directly
// does not work reliably and must not be done:
//   - the request can arrive while another pattern is mid-play, and that pattern's executor ends by
//     zeroing current_vib_pattern, so the 7 is wiped before it is ever felt;
//   - the monitor checks at the top of the task can overwrite a queued 7 with a lower-priority
//     warning in the same pass.
// A separate flag cannot be overwritten by either, which is what makes "the stop buzz preempts
// everything" a true statement instead of an aspiration. Worst case on a lost race is one extra
// buzz; the failure mode of the old scheme was a MISSING stop buzz, which is the dangerous one.
// The rider is looking at the water, not the display — a stop he did not ask for must be felt.
volatile bool vib_stop_pending = false;

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
            // V2.5-Evo - 2026-08-17 - StopBuzz FIX 3: sq_warned used to be set HERE-but-outside,
            // i.e. also when the guard above had just stopped the buzz from being queued at all.
            // The warning was then latched as "already given" without ever having been felt, and
            // could not fire again until the signal first climbed back above 1 bar — so a drop
            // that happened to collide with any other pattern was swallowed permanently.
            // Latch only on a real queue.
            sq_warned = true;
          }
        } else if (cur_sq > 1) {
          // Signal recovered — allow warning to fire again on next drop
          sq_warned = false;
        }
        // V2.5-Evo - 2026-08-17 - StopBuzz FIX 3 (second half): hold last_sq at its pre-drop value
        // while a drop to 1 bar is still un-warned. The fire condition is an EDGE (>1 then ==1);
        // advancing last_sq to 1 unconditionally consumed that edge, so a deferred warning could
        // never be retried no matter what sq_warned said. Skipping the update leaves the edge
        // intact and the next pass (50ms later) tries again.
        if (!(cur_sq == 1 && !sq_warned)) last_sq = cur_sq;
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
    // V2.5-Evo - 2026-08-17 - StopBuzz FIX 4: this was an UNCONDITIONAL write, and it is the last
    // monitor check in the chain — so a failsafe (Pattern 2) or an E71 water-ingress (Pattern 3)
    // raised earlier in the SAME pass was overwritten by the least important alert in the system.
    // A priority inversion: "battery at 20%" is not urgent, "you have lost the link" and "water is
    // inside the remote" are. Guarded like every other informational site, and bat_warning_sent is
    // latched only when the buzz was really queued so it retries on the next pass instead of being
    // marked as given.
    if (telemetry.foil_bat != 0xFF && telemetry.foil_bat <= 20) {
      if (!bat_warning_sent && current_vib_pattern == 0) {
        current_vib_pattern = 1; // Pattern A: 2 Short (Warning)
        bat_warning_sent = true;
      }
    } else if (telemetry.foil_bat > 20) {
      bat_warning_sent = false; // Reset if battery is changed
    }

    // --- 1b. PROMOTE A PENDING STOP ---
    // A stop request outranks everything above: it is the only pattern that tells the rider the
    // system has stopped doing what he believes it is doing. Promoted here, AFTER all the monitor
    // checks and BEFORE the executor, so nothing in this pass can overwrite it.
    if (vib_stop_pending) {
      vib_stop_pending    = false;
      current_vib_pattern = 7;   // Pattern 7: one long buzz = STOP
    }

    // --- 2. EXECUTE THE PATTERNS ---
    // Each branch below ends by clearing ONLY the pattern it just played. It used to clear
    // unconditionally, which threw away anything queued while it was busy — including a stop.
    // Multi-pulse patterns also break out between pulses when a stop arrives, so the stop is felt
    // within one pulse rather than up to 4s later (Pattern 3 is five 500ms buzzes).

    if (current_vib_pattern == 1) {
      // PATTERN A: 2 Short
      for(int i=0; i<2; i++) {
        digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(150));
        digitalWrite(P_MOT, LOW);  vTaskDelay(pdMS_TO_TICKS(150));
        if (vib_stop_pending) break;   // a stop outranks a warning — cut it short
      }
      if (current_vib_pattern == 1) current_vib_pattern = 0;
    }
    else if (current_vib_pattern == 2) {
      // PATTERN B: 5 Short
      for(int i=0; i<5; i++) {
        digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(150));
        digitalWrite(P_MOT, LOW);  vTaskDelay(pdMS_TO_TICKS(150));
        if (vib_stop_pending) break;   // a stop outranks a failsafe alert — cut it short
      }
      if (current_vib_pattern == 2) current_vib_pattern = 0;
    }
    else if (current_vib_pattern == 3) {
      // PATTERN C: 5 Long
      for(int i=0; i<5; i++) {
        digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(500));
        digitalWrite(P_MOT, LOW);  vTaskDelay(pdMS_TO_TICKS(300));
        if (vib_stop_pending) break;   // longest pattern in the system (4s) — must yield to a stop
      }
      if (current_vib_pattern == 3) current_vib_pattern = 0;
    }
    // V2.5-Evo - 2026-04-27 - P8: Pattern 4 — ARM confirm (FM/RTM).
    // V2.5-Evo - 2026-07-20 - ArmFeel: was two 80ms taps with an 80ms gap — too fast, blurred into
    // one stutter and could be confused with the one-long-buzz STOP (Pattern 7). Now two firm 130ms
    // taps split by a clear 250ms gap so it reads UNMISTAKABLY as TWO events ("tap..tap" = arm) vs
    // Pattern 7's single sustained buzz ("buzzzzz" = stop). The 250ms gap is the key cue, and it stays
    // clear of the low-battery/weak-signal warning (Pattern 1 = two 150ms buzzes, 150ms gap).
    else if (current_vib_pattern == 4) {
      digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(130));
      digitalWrite(P_MOT, LOW);  vTaskDelay(pdMS_TO_TICKS(250));
      digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(130));
      digitalWrite(P_MOT, LOW);
      if (current_vib_pattern == 4) current_vib_pattern = 0;
    }
    // V2.5-Evo - 2026-07-20 - MagGesture: Pattern 5 — ONE short pulse.
    // Used only as the magnet-gesture 2s advisory ("release the magnet now and FM will arm").
    // Deliberately a single pulse so it cannot be confused by feel with Pattern 4 (two pulses),
    // which is the 5s RTM advisory and every arm/disarm confirm. 150ms matches the "short"
    // pulse length already used by Patterns A and B.
    // V2.5-Evo - 2026-10-07 - R-7: THIRD CALLER. runMagGesture() (mag_mode 4) plays it at the 2.5 s mark
    // INSTEAD of the Pattern 10 "let go now" cue when Return-To-Me cannot start (disabled or GPS off); removal
    // then shows "n0". One short blip is the least confusing existing shape for "no": it is not the two-medium
    // success cue (10), not the long fault buzz (7), and its other callers are the mag_mode 1-3 advisory (a
    // different mag_mode, so never on the same remote) and the fm_arm_timeout_s nudge (off by default).
    else if (current_vib_pattern == 5) {
      digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(150));
      digitalWrite(P_MOT, LOW);
      if (current_vib_pattern == 5) current_vib_pattern = 0;
    }
    // V2.5-Evo - 2026-07-20 - MagGesture: Pattern 6 — THREE short pulses.
    // Used only as the magnet-gesture 5s advisory ("release the magnet now and RTM will arm").
    // Why this is not Pattern 4: the RTM arm confirm fired by setRtmArmed() is itself Pattern 4,
    // so reusing it would give the rider two identical double-buzzes separated only by the
    // release — indistinguishable by feel. Three pulses makes both tiers unambiguous:
    //   1 pulse then 2 = FM armed;  3 pulses then 2 = RTM armed.
    // V2.5-Evo - 2026-07-20 - TapFeel: was 80ms on/80ms off — three taps that fast blur into one
    // buzz. Now 100ms on / 150ms off so they read UNMISTAKABLY as THREE distinct taps. Still lighter
    // and quicker than the two firm arm taps (Pattern 4 = 130ms/250ms), and the count (3 vs 2) keeps
    // them distinct regardless.
    else if (current_vib_pattern == 6) {
      for (int i = 0; i < 3; i++) {
        digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(P_MOT, LOW);  vTaskDelay(pdMS_TO_TICKS(150));
        if (vib_stop_pending) break;   // a stop outranks a magnet advisory — cut it short
      }
      if (current_vib_pattern == 6) current_vib_pattern = 0;
    }
    // V2.5-Evo - 2026-07-20 - StopFeel: Pattern 7 — ONE long 750ms pulse = the STOP confirm.
    // V2.5-Evo - 2026-08-17 - Trigger list corrected. It previously listed RTM Gate 3 and the FM
    // Gate 1 release backstop, which the same batch had just made SILENT — the comment described
    // behaviour that no longer existed.
    // THE RULE THAT DECIDES: A PURE TIMEOUT IS SILENT, A FAULT BUZZES. Mid-wave the rider has no
    // attention to spare for decoding a buzz, and the more buzzes there are the less each one is
    // read — so Pattern 7 is spent only where it tells him something he cannot otherwise know.
    // FIRES ON:
    //   - RTM Gate 1 (max runtime). A timer, and the ONE exception to the rule: it has no throttle
    //     precondition, so it can fire mid-squeeze, and rtmDisengage() lifts rtm_thr_cap_tx back to
    //     255 in the same instant — a real step from capped RTM throttle to raw manual throttle.
    //   - RTM Gate 2 (TX GPS stale). A fault: a sensor died and nothing the rider did explains it.
    //   - The FM RX fault-stop. A fault: the RX gave up steering on its own.
    //   - The two arm REFUSALS — RTM pre-arm distance reject, FM fundamental reject — where the
    //     rider would otherwise walk away believing the mode is armed when it is not.
    // DOES NOT FIRE ON:
    //   - RTM Gate 3 (throttle released 4s). A pure timeout his own released trigger caused, and
    //     "St" already shows it. (The FM Gate 1 release backstop that used to sit alongside it here
    //     was removed 2026-09-17 — it no longer exists, so it is not listed either way.)
    //   - Any deliberate disarm — gesture disarm, magnet-toggle disarm, steer-exit, F0 select.
    //   - The mag_mode 4 magnet hold that switches Return-To-Me OFF. V2.5-Evo - 2026-09-30 (Rex delta
    //     audit): that confirm DID route through vib_stop_pending for one commit and it should not have.
    //     It is a deliberate two-state decision, not a fault and not even a stop — nothing was running —
    //     and borrowing the fault buzz for it invited a rider with mag_mode 3 muscle memory to read
    //     "disarmed" when he had actually disabled Return-To-Me. It is Pattern 12 now (three firm taps).
    // Deliberately a single SUSTAINED buzz so the rider can tell "stopped/off" from the arm
    // confirm by feel alone while foiling. Distinct from every other pattern:
    //   - Pattern 4 (arm) is TWO 130ms taps split by a 250ms gap; Pattern 6 is THREE — this is ONE.
    //   - Pattern 5 (magnet advisory) is ONE short 150ms pulse — 750ms is 5× longer, so a
    //     single-pulse collision reads clearly as "long buzz" vs "short blip".
    // 750ms (owner-tuned up from 400ms) widens the margin for a by-feel signal the rider acts on
    // without looking. Single pulse only — cannot be confused with the 5×500ms Pattern 3.
    else if (current_vib_pattern == 7) {
      digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(750));
      digitalWrite(P_MOT, LOW);
      if (current_vib_pattern == 7) current_vib_pattern = 0;
    }
    // V2.5-Evo - 2026-09-17 - WarnDist: Pattern 8 — ONE medium 300 ms pulse = the FM warning-distance
    // haptic. runFmLoop() (RTMState.ino) queues it the moment the buggy reaches fm_warn_distance_m
    // and then every 2 s while it stays at or beyond it, including with the trigger released.
    // 300 ms sits between the 150 ms advisory (Pattern 5) and the 750 ms STOP (Pattern 7), so all
    // three stay distinct by feel. It is queued only when nothing else is playing and no STOP is
    // pending, and the pulse is bounded, so a STOP that arrives during it is promoted right after.
    else if (current_vib_pattern == 8) {
      digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(300));
      digitalWrite(P_MOT, LOW);
      if (current_vib_pattern == 8) current_vib_pattern = 0;
    }
    // V2.5-Evo - 2026-09-19 - Return gesture: Pattern 9 — FOUR quick 80 ms taps, 80 ms gaps (a trill).
    // Fired by returnGesture() when the RTM arm is cancelled and the auto-return override is SET
    // (to the opposite of what the buggy reported). Four fast taps are a new shape: the arm confirm
    // is two firm taps (4), the magnet advisory three (6), so the count and the speed both differ.
    else if (current_vib_pattern == 9) {
      for (int i = 0; i < 4; i++) {
        digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(80));
        digitalWrite(P_MOT, LOW);  vTaskDelay(pdMS_TO_TICKS(80));
        if (vib_stop_pending) break;   // a stop outranks a confirm — cut it short
      }
      if (current_vib_pattern == 9) current_vib_pattern = 0;
    }
    // V2.5-Evo - 2026-09-19 - Return gesture: Pattern 10 — TWO medium 300 ms pulses, 200 ms gap.
    // Fired by returnGesture() when the override is CLEARED (back to the buggy's stored default).
    // Two mediums: longer than the two firm arm taps (4: 130 ms), one more than the single 300 ms
    // warning (8), far shorter than the one 750 ms STOP (7).
    // V2.5-Evo - 2026-09-30 - MagStations: SECOND CALLER. runMagGesture() also fires Pattern 10 as the
    // 2.5 s advisory in mag_mode 4 — "let go now and Return-To-Me will toggle". It is the first half of
    // that gesture's signature: two mediums, then two firm taps (4) for ON or one long buzz (7) for OFF.
    // The two callers can never overlap — one is the toggle combo, the other needs a magnet held for
    // 2.5 s — and both mean "a deliberate two-state decision just happened", so the feel stays honest.
    else if (current_vib_pattern == 10) {
      for (int i = 0; i < 2; i++) {
        digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(300));
        digitalWrite(P_MOT, LOW);  vTaskDelay(pdMS_TO_TICKS(200));
        if (vib_stop_pending) break;   // a stop outranks a confirm — cut it short
      }
      if (current_vib_pattern == 10) current_vib_pattern = 0;
    }
    // V2.5-Evo - 2026-09-30 - MagStations: Pattern 11 — N short taps, where N IS THE ANSWER.
    // vib_pulse_count carries the Follow-Me station number the magnet tap just moved to (1-3), so the rider
    // counts taps instead of reading the display: two taps means the buggy is heading to station 2.
    // The pulse shape (100 ms on / 150 ms off) is copied from Pattern 6 — this firmware's existing
    // "counted taps" shape — so a count reads as a count and not as a new signal to learn. Pattern 11 with
    // a count of 3 is deliberately identical to Pattern 6: Pattern 6 only ever plays in mag_mode 3 and
    // Pattern 11 only in mag_mode 4, so one remote never produces both.
    // The count is clamped to 1-5 so a stray or uninitialised value cannot buzz forever.
    else if (current_vib_pattern == 11) {
      uint8_t n = vib_pulse_count;
      if (n < 1) n = 1;
      if (n > 5) n = 5;
      for (uint8_t i = 0; i < n; i++) {
        digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(P_MOT, LOW);  vTaskDelay(pdMS_TO_TICKS(150));
        if (vib_stop_pending) break;   // a stop outranks a confirm — cut it short
      }
      if (current_vib_pattern == 11) current_vib_pattern = 0;
    }
    // V2.5-Evo - 2026-09-30 - MagFix (Rex delta audit): Pattern 12 — THREE FIRM taps, the Pattern 4 shape
    // (130 ms on / 250 ms off). Fired by fmToggleRtmEnabledFromMagnet() when a mag_mode 4 magnet hold turns
    // Return-To-Me OFF for the session.
    // WHY THIS PATTERN EXISTS AT ALL. OFF used to raise vib_stop_pending, i.e. Pattern 7, the one long buzz.
    // Pattern 7 means "a FAULT stopped the system" and its own contract above says it DOES NOT FIRE ON any
    // deliberate disarm — and because it preempts every other pattern, it also outranked everything else the
    // remote might have been saying. The rider that hurts is the one carrying mag_mode 3 muscle memory, where
    // a magnet hold disarms and buzzes long: in mode 4 he would feel the same long buzz, read it as "disarmed",
    // and have actually switched Return-To-Me off with Follow-Me still armed.
    // WHY THREE FIRM TAPS. A deliberate two-state decision gets a bounded counted-tap confirm, which is the
    // shape this firmware already uses for exactly that. ON is Pattern 4 = TWO firm taps; OFF is the same
    // shape with THREE, so the pair differs only in count — the same 2-vs-3 discrimination Pattern 4 and
    // Pattern 6 already rely on — and neither is a single sustained buzz, so neither can be read as a stop.
    // NOT confusable with Pattern 6 (three LIGHT 100/150 taps, mag_mode 3 only — this is mag_mode 4 only, so
    // one remote never produces both) nor with Pattern 11 (light taps, and it confirms a station change, which
    // never follows the two-medium Pattern 10 advisory that precedes every RTM toggle) nor with Pattern 9
    // (four taps at 80/80 — twice the tempo and one more tap).
    // Bounded at three pulses, so like every confirm here it cannot buzz indefinitely, and it yields to a real
    // stop between pulses.
    else if (current_vib_pattern == 12) {
      for (uint8_t i = 0; i < 3; i++) {
        digitalWrite(P_MOT, HIGH); vTaskDelay(pdMS_TO_TICKS(130));
        digitalWrite(P_MOT, LOW);  vTaskDelay(pdMS_TO_TICKS(250));
        if (vib_stop_pending) break;   // a stop outranks a confirm — cut it short
      }
      if (current_vib_pattern == 12) current_vib_pattern = 0;
    }

    // Sleep briefly to prevent hoarding the CPU
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}