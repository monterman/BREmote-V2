// V2.5-Evo - 2026-05-11 - Compass Cal: runtime BIND press triggers compass calibration with LED feedback
// V3 - 2026-04-25 - P7: Added ?compassheading serial diagnostic command
// V2.5-Evo - 2026-05-05 - cmdMagTest: bench-test logger for compass EMI vs motor current
// V2.5-Evo - 2026-05-05 - cmdVescPing: VESC UART telemetry verification (?vescping)
// V2.5-Evo - 2026-05-06 - cmdVescRaw: raw VESC UART byte-dump probe (?vescraw)
#include <Wire.h>

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

// ===== I2C Scanner Function =====
void scanI2C() {
  byte error, address;
  int nDevices = 0;

  // V3 fix (Bug 7): do not call Wire.begin() here. Wire was already initialised to the
  // correct pins (SDA=%d SCL=%d) in initHardware(). Re-initialising mid-session resets
  // the I2C peripheral and can glitch an in-progress AW9523 transaction.
  Serial.printf("Scanning I2C bus (initialized on SDA:%d SCL:%d)...\n", P_I2C_SDA, P_I2C_SCL);

  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.print(address, HEX);
      
      // Give a helpful label if it matches known devices
      if (address == 0x58) Serial.print(" (AW9523 Expander)");
      if (address == 0x1E) Serial.print(" (HMC5883L Compass)");
      if (address == 0x0D) Serial.print(" (QMC5883L Compass)");
      
      Serial.println(" !");
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found.\n");
  } else {
    Serial.println("Scan complete.\n");
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

// ===== Logger Serial Command Handlers =====
void cmdStartLog(const String& params) { startLog(); }
void cmdStopLog(const String& params) { stopLog(); }
void cmdListLogs(const String& params) { listLogFiles(); }

void cmdDownloadLog(const String& params) { 
  if(params.length() == 0) {
    Serial.println("Error: Specify filename (e.g. ?download /filename.log)");
  } else {
    downloadLogFile(params.c_str()); 
  }
}

void cmdDeleteLog(const String& params) {
  if(params.length() == 0) {
    Serial.println("Error: Specify filename (e.g. ?deletelog /filename.log)");
  } else {
    deleteLogFile(params.c_str());
  }
}

void cmdLogRate(const String& params) {
  if (params.length() > 0) {
    float rate = params.toFloat();
    setLogRate(rate);
  } else {
    Serial.println("Error: Specify rate in Hz (e.g., ?lograte 1 or ?lograte 0.1)");
  }
}

// ===== Compass Serial Command Handlers =====
void cmdScanI2C(const String& params) { scanI2C(); }
void cmdPrintCompass(const String& params) { serPrintCompass(); }
void cmdCompassCal(const String& params) { runCompassCalibration(); }
void cmdPrintCompassHeading(const String& params) {
  Serial.println("Printing compass heading. Type 'quit' to exit.");
  while (true) {
    esp_task_wdt_reset();
    if (checkSerialQuit()) break;
    float h = getCompassHeading();
    if (h < 0.0f) Serial.println("Compass not detected or not calibrated");
    else Serial.printf("Heading: %.1f deg\n", h);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ============================================================
// cmdMagTest - Compass + Motor Current EMI Bench Test Logger
// ============================================================
//
// What it does:
//   Streams CSV data to Serial at 10 Hz for up to 120 seconds, capturing
//   raw magnetometer readings (X/Y/Z/magnitude/heading) alongside VESC
//   motor current, ERPM, and received throttle byte. Intended to let you
//   see how motor EMI shifts compass magnitude and heading as you increase
//   throttle on the stationary buggy.
//
// How to invoke:
//   Type '?magtest' in a serial terminal. Type 'quit' to abort early.
//
// Expected use:
//   Motor disconnected from load, buggy on bench. Bring throttle 0->100%
//   slowly. Save serial output as a .csv file and plot in Excel or Python.
//
// Output columns:
//   millis, magX, magY, magZ, magnitude, heading_deg,
//   vesc_erpm, vesc_motor_current_a, thr_received
//   heading_deg = -1.0 if compass not calibrated or I2C read failed.
//   vesc_erpm and vesc_motor_current_a = -1 if vescMutex take times out.
void cmdMagTest(const String& params) {
  extern SemaphoreHandle_t vescMutex; // declared in Logger.ino; guards vesc struct
  extern vesc_struct vesc;            // VESC telemetry struct; written by VESC.ino

  Serial.println("=== Compass + Motor Current Bench Test ===");
  Serial.println("Type 'quit' to abort. Runs up to 120 seconds.");
  Serial.println("Recommended: bring throttle 0->100% slowly while collecting data.");
  Serial.println();
  Serial.println("millis,magX,magY,magZ,magnitude,heading_deg,vesc_erpm,vesc_motor_current_a,thr_received");

  const uint32_t TEST_DURATION_MS = 120000UL;
  uint32_t start = millis();

  while ((millis() - start) < TEST_DURATION_MS) {
    esp_task_wdt_reset(); // prevent WDT timeout during the 120s blocking loop

    if (checkSerialQuit()) break;

    // Refresh raw magnetometer globals magX/magY/magZ from QMC5883L via I2C.
    // Result ignored — magnitude is computed below regardless; stale globals
    // are acceptable for a diagnostic logger if I2C briefly fails.
    readCompassRaw();
    float magnitude = sqrtf((float)magX * magX + (float)magY * magY + (float)magZ * magZ);

    // getCompassHeading() applies hard/soft iron correction and returns
    // degrees 0-360. Returns -1.0 if compass not detected or not calibrated.
    // Note: internally calls readCompassRaw() again, so magX/Y/Z may update;
    // at 10 Hz bench-test precision this one-sample gap is negligible.
    float heading = getCompassHeading();

    // Read VESC ERPM and motor current under mutex, exactly as runPhaseC() does
    // in RTMState.ino. motCur is stored in 0.01 A units; divide by 100 for amps.
    long  snap_erpm    = -1L;
    float snap_motor_a = -1.0f;
    if (vescMutex && xSemaphoreTake(vescMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      snap_erpm    = (long)vesc.erpm;
      snap_motor_a = (float)vesc.motCur / 100.0f;
      xSemaphoreGive(vescMutex);
    }

    // thr_received is volatile uint8_t — single-byte read is atomic on this arch.
    uint8_t snap_thr = thr_received;

    Serial.printf("%lu,%d,%d,%d,%.1f,%.1f,%ld,%.2f,%u\n",
                  millis(),
                  (int)magX, (int)magY, (int)magZ,
                  magnitude, heading,
                  snap_erpm, snap_motor_a,
                  (unsigned)snap_thr);

    vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz output rate
  }

  Serial.println("=== Test complete. Save serial output to a .csv file for analysis. ===");
}

// ============================================================
// cmdVescPing - VESC UART Telemetry Verification
// ============================================================
//
// What it does:
//   Reads the vesc struct (guarded by vescMutex) and the global
//   last_uart_packet timestamp at 2 Hz for up to 30 seconds, printing
//   a CSV line each iteration. The key diagnostic field is pkt_age_ms:
//   if it stays < ~1200 ms, the VESC is actively sending UART packets.
//   If it grows continuously past 1500 ms without resetting, the VESC is
//   silent — check wiring, baud rate, and the data_src SPIFFS parameter.
//
// How to invoke:
//   Type '?vescping' in a serial terminal. Type 'quit' to abort early.
//
// What to look for:
//   pkt_age_ms < 1200 throughout  → VESC UART healthy; motor current is real.
//   pkt_age_ms grows unboundedly  → VESC UART silent; struct values are stale.
//   motCur_a near 0 with healthy UART → unloaded motor, low current is normal.
void cmdVescPing(const String& params) {
  extern SemaphoreHandle_t vescMutex; // declared in Logger.ino; guards vesc struct
  extern vesc_struct vesc;            // VESC telemetry struct; written by VESC.ino

  Serial.println("=== VESC UART Verification ===");
  Serial.println("Type 'quit' to abort. Runs up to 30 seconds at 2 Hz.");
  Serial.println("Run with motor OFF first (baseline), then turn motor ON and observe.");
  Serial.println("If 'pkt_age_ms' keeps growing past ~1500 and never resets to ~0,");
  Serial.println("the VESC is NOT responding over UART (wiring or config issue).");
  Serial.println();
  Serial.println("millis,motCur_a,erpm,batVolt_v,fetTemp_c,pkt_age_ms,thr_received");

  const uint32_t TEST_DURATION_MS = 30000UL;
  uint32_t start = millis();

  while ((millis() - start) < TEST_DURATION_MS) {
    esp_task_wdt_reset(); // prevent WDT timeout during the 30s blocking loop

    if (checkSerialQuit()) break;

    // Read VESC struct fields under mutex — same pattern as runPhaseC() in RTMState.ino.
    // Units: motCur = 0.01 A, batVolt = 0.01 V, fetTemp = 0.1 °C.
    float  snap_motcur_a  = -1.0f;
    long   snap_erpm      = -1L;
    float  snap_batvolt_v = -1.0f;
    float  snap_fettemp_c = -1.0f;
    if (vescMutex && xSemaphoreTake(vescMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      snap_motcur_a  = (float)vesc.motCur  / 100.0f;
      snap_erpm      = (long)vesc.erpm;
      snap_batvolt_v = (float)vesc.batVolt / 100.0f;
      snap_fettemp_c = (float)vesc.fetTemp / 10.0f;
      xSemaphoreGive(vescMutex);
    }

    // last_uart_packet is a volatile unsigned long updated by getVescLoop() each time
    // a valid VESC packet arrives. Age tells us whether VESC is actively responding.
    unsigned long pkt_age_ms = millis() - last_uart_packet;

    // Single-byte volatile read — atomic on ESP32-C3, no mutex needed.
    uint8_t snap_thr = thr_received;

    Serial.printf("%lu,%.2f,%ld,%.2f,%.1f,%lu,%u\n",
                  millis(),
                  snap_motcur_a, snap_erpm,
                  snap_batvolt_v, snap_fettemp_c,
                  pkt_age_ms,
                  (unsigned)snap_thr);

    vTaskDelay(pdMS_TO_TICKS(500)); // 2 Hz output rate
  }

  Serial.println();
  Serial.println("=== Verification complete. ===");
  Serial.println("If pkt_age_ms stayed < 1200 throughout: VESC UART is healthy. Motor current is real.");
  Serial.println("If pkt_age_ms grew unboundedly: VESC UART is silent. Check wiring, baud, and data_src.");
}

// ============================================================
// cmdVescRaw - Raw VESC UART Byte-Dump Probe
// ============================================================
//
// What it does:
//   Bypasses the normal getVescLoop() pipeline entirely. Manually switches
//   the UART mux to channel 0 (VESC), sends a raw COMM_GET_VALUES short-frame
//   query, then dumps every byte received in hex for up to 200ms. Repeats
//   every 2 seconds for 15 iterations (30 seconds total).
//
//   This probes the physical UART path rather than the parsed struct, so it
//   reveals whether the VESC is reachable at the hardware level independently
//   of whether getVescLoop() parses the response correctly.
//
// Inputs:  params - unused
// Outputs: hex dump to Serial; no struct writes; no global state changes
// Side effects: switches UART mux to channel 0 each iteration (same as normal VESC operation)
//
// How to interpret output:
//   Zero bytes every iteration    -> VESC unreachable. Check mux IC channel 0,
//                                    VESC TX wire, and GND connection.
//   Bytes received, no 0x02 lead  -> Baud rate mismatch. Firmware uses 115200;
//                                    check VESC Tool App Configuration -> General -> UART Baud.
//   Response starts with 0x02     -> VESC is alive and responding. The issue
//                                    is in receiveFromVESC() parsing, not hardware.
void cmdVescRaw(const String& params) {
  Serial.println("=== VESC Raw UART Probe ===");
  Serial.println("Sends COMM_GET_VALUES every 2s, dumps received bytes in hex.");
  Serial.println("Type 'quit' to abort. Runs up to 30 seconds (15 attempts).");
  Serial.println();
  Serial.println("Expected outcomes:");
  Serial.println("  Zero bytes received  -> mux/wiring/baud issue (VESC unreachable)");
  Serial.println("  Garbage bytes        -> baud rate mismatch");
  Serial.println("  Frame starts with 02 -> VESC responding, parser failing elsewhere");
  Serial.println();
  Serial.println("  VESC short-frame format: [0x02][LEN][PAYLOAD][CRC16][0x03]");
  Serial.println();

  // Precomputed VESC short-frame for COMM_GET_VALUES (command ID 4).
  // [0x02 start][0x01 payload-len=1][0x04 COMM_GET_VALUES][0x40 CRC16_HI][0x07 CRC16_LO][0x03 end]
  // CRC16-CCITT (init=0) over single payload byte {0x04} = 0x4007.
  static const uint8_t getValuesQuery[] = { 0x02, 0x01, 0x04, 0x40, 0x07, 0x03 };

  const int MAX_ITERATIONS = 15;

  for (int iter = 1; iter <= MAX_ITERATIONS; iter++) {
    esp_task_wdt_reset(); // prevent WDT timeout during the 30s blocking loop

    if (checkSerialQuit()) break;

    // Switch mux to channel 0 (VESC) and allow it to settle
    setUartMux(0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Drain any stale bytes from a previous response before sending the query
    while (Serial1.available()) Serial1.read();

    // Send the raw COMM_GET_VALUES request directly — NOT via getVescLoop()
    Serial1.write(getValuesQuery, sizeof(getValuesQuery));
    Serial1.flush();

    Serial.printf("Iteration %d: sent 6 bytes, listening 200ms...\n", iter);

    // Collect every byte that arrives within 200ms
    uint8_t rxBuf[256];
    int rxCount = 0;
    uint32_t listenStart = millis();
    while ((millis() - listenStart) < 200 && rxCount < (int)sizeof(rxBuf)) {
      if (Serial1.available()) {
        rxBuf[rxCount++] = (uint8_t)Serial1.read();
      }
    }

    if (rxCount == 0) {
      Serial.println("  NO BYTES RECEIVED -- VESC unreachable on this iteration");
    } else {
      Serial.printf("  Received %d bytes:\n", rxCount);
      // Hex dump: rows of 16 bytes, two-digit hex, space-separated
      for (int i = 0; i < rxCount; i++) {
        if (i > 0 && (i % 16) == 0) Serial.println();
        if ((i % 16) == 0) Serial.print("  ");
        Serial.printf("%02X", rxBuf[i]);
        if ((i % 16) != 15 && i != rxCount - 1) Serial.print(" ");
      }
      Serial.println();
    }

    // Wait the remainder of the 2s cycle (~1700ms after 10ms mux + 200ms listen)
    vTaskDelay(pdMS_TO_TICKS(1700));
  }

  Serial.println();
  Serial.println("=== Probe complete. ===");
  Serial.println("Summary heuristic:");
  Serial.println("  All 15 iterations 0 bytes  -> wiring or mux. Check VESC TX wire, GND, mux IC.");
  Serial.println("  Most iterations 0 bytes    -> intermittent -- likely loose connection.");
  Serial.println("  Bytes received but no 0x02 -> baud mismatch. Try VESC Tool App config.");
  Serial.println("  0x02 0xXX received         -> VESC alive! Parser issue in receiveFromVESC().");
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
  
  // --- Logger Commands ---
  {"start", "start data logging", cmdStartLog},
  {"stop", "stop data logging", cmdStopLog},
  {"list", "list saved log files", cmdListLogs},
  {"download", "<filename> download log as CSV", cmdDownloadLog},
  {"deletelog", "<filename> delete specific log file", cmdDeleteLog},
  {"lograte", "<Hz> set log rate (e.g. 1 or 0.1)", cmdLogRate},
  
  // --- Hardware Diagnostics ---
  {"i2c", "scan I2C bus for compass", cmdScanI2C},
  {"printcompass", "print raw compass X/Y/Z", cmdPrintCompass},
  {"compasscal", "start 45s automated calibration", cmdCompassCal},
  {"compassheading", "print live compass heading in degrees", cmdPrintCompassHeading},
  {"magtest", "120s CSV log: compass X/Y/Z + VESC current vs throttle (bench EMI test)", cmdMagTest},
  {"vescping", "stream VESC fields + UART packet age (2Hz, up to 30s; verify VESC UART)", cmdVescPing},
  {"vescraw", "raw VESC UART byte dump (sends GET_VALUES, prints any bytes received as hex)", cmdVescRaw},

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
    
    String command = Serial.readStringUntil('\n');
    // Read input until newline

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
    esp_task_wdt_reset(); // V3 fix (I3): prevent WDT panic during blocking debug command
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
        esp_task_wdt_reset(); // V3 fix (I3): prevent WDT panic during blocking debug command
        if (Serial.available()) {
            String input = Serial.readStringUntil('\n'); // read line
            input.trim(); // remove CR/LF/whitespace

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
    esp_task_wdt_reset(); // V3 fix (I3): prevent WDT panic during blocking debug command
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
    esp_task_wdt_reset(); // V3 fix (I3): prevent WDT panic during blocking debug command
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
    esp_task_wdt_reset(); // V3 fix (I3): prevent WDT panic during blocking debug command
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
    esp_task_wdt_reset(); // V3 fix (I3): prevent WDT panic during blocking debug command
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
    esp_task_wdt_reset(); // V3 fix (I3): prevent WDT panic during blocking debug command
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
  // --- BOOT-TIME BIND / RESET LOGIC ---
  // Runs only on the first call (via runBootSequence() during setup). Static guard
  // prevents pairing and factory-reset from triggering during runtime calls from loop().
  static bool first_call = true;
  if (first_call) {
    first_call = false;
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

  // --- AUX BUTTON: LOGGER TOGGLE ---
  // Static variables remember their state between loops
  static bool aux_last_state = true;
  // true = HIGH (unpressed due to pullup)
  bool aux_current = aw.digitalRead(AP_S_AUX);
  // Detect a "falling edge" (button was just pressed down)
  if (aux_last_state == true && aux_current == false)
  {
    vTaskDelay(pdMS_TO_TICKS(50));
    // 50ms Debounce to prevent double-clicks
    if (aw.digitalRead(AP_S_AUX) == false)
    {
      if (isLoggingActive())
      {
        stopLog();
        blinkErr(2, AP_L_AUX); // Blink AUX LED 2 times to confirm STOP
      }
      else
      {
        startLog();
        blinkErr(5, AP_L_AUX); // Blink AUX LED 5 times to confirm START
      }

      // Wait for the user to let go of the button before continuing
      while(aw.digitalRead(AP_S_AUX) == false) { vTaskDelay(pdMS_TO_TICKS(10));
      }
    }
  }
  aux_last_state = aux_current;

  // --- RUNTIME BIND: COMPASS CALIBRATION ---
  // Short BIND press (falling edge, 50ms debounce) triggers 45s calibration.
  // blinkBind(5) = starting, blinkBind(2) = success, blinkBind(10) = compass not detected.
  // Boot-time pairing/reset cannot reach this block (guarded by first_call above).
  static bool bind_last_state = true;
  bool bind_current = aw.digitalRead(AP_S_BIND);
  if (bind_last_state == true && bind_current == false) {
    vTaskDelay(pdMS_TO_TICKS(50));
    if (aw.digitalRead(AP_S_BIND) == false) {
      blinkBind(5);
      extern bool compass_detected;  // global bool set by initCompass(); false if sensor absent
      runCompassCalibration();        // 45s collection, hard/soft-iron calc, auto-save to SPIFFS
      if (compass_detected) {
        blinkBind(2);
      } else {
        blinkBind(10);
      }
      while (aw.digitalRead(AP_S_BIND) == false) { vTaskDelay(pdMS_TO_TICKS(10)); }
    }
  }
  bind_last_state = bind_current;
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