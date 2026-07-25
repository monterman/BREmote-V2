// V2.5-Evo - 2026-07-25 - STAGE 0 PART D (instrumentation only): added ?diag (one-shot, non-blocking snapshot of GPS bytes/sentences per second, fix age now/mean/max, COG timestamp-updates vs VALUE-changes, mux switches + read-back failures, VESC poll success rate, and loop min/mean/max) and ?diagz (zero the counters to bracket a run). Unlike ?gpsdiag these do not loop or delay, so they are safe to run with RTM/FM active and need no refusal guard. setUartMux() also gained two counter increments (switches, read-back mismatches) — the corrective re-write itself is unchanged. No control path, no confStruct change, SW_VERSION stays 34.
// V2.5-Evo - 2026-07-19 - Rex hardening: cmdGpsDiag (?gpsdiag) refuses to run while RTM active — its ≤120s blocking loop would freeze runRtmLoop()/Phase A/B/convergence/Gate 9
// V2.5-Evo - 2026-07-19 - FM triage: cmdGpsDiag (?gpsdiag) — 2Hz GPS feed + RTM COG-valid sub-condition breakdown to diagnose why GPS COG heading never engages
// V2.5-Evo - 2026-05-11 - E7 Fix: checkWetness() debounced — requires 2 consecutive confirmed-wet calls to set E7; single clean read clears
// V2.5-Evo - 2026-05-11 - Compass Cal: runtime BIND press triggers compass calibration with LED feedback
// V2.5-Evo - 2026-04-25 - P7: Added ?compassheading serial diagnostic command
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

// V2.5-Evo - 2026-06-04 - D1: UART-mux read-back verify (audit).
//
// The AW9523 UART-mux select pins (AP_U1_MUX_0/1) and the PWM-enable pins share the
// same I2C expander. Under motor switching, MOSFET EMI corrupts I2C writes (documented
// SW48-SW55, checkWetness() note above): a setUartMux() write can be stripped, leaving
// Serial1 still routed to the GPS so the VESC query goes nowhere — the intermittent
// motor / blank-telemetry / startup-stall cluster.
//
// Mitigation: after writing the two mux bits, read them back (digitalRead reads the
// AW9523 INPUT register = actual pad level). If either bit does not match the intended
// state, re-assert ONCE and re-read ONCE. This is a BOUNDED verify-and-correct, NOT the
// SW51/SW52 rapid-retry loop that was reverted in SW54 for hammering the bus — at most a
// single corrective write pass per call. All I2C stays inside the i2cMutex critical section.
void setUartMux(int channel)
{
  // Intended pad levels for the two select bits per channel:
  //   channel 0 (VESC): MUX_0 = LOW,  MUX_1 = LOW
  //   channel 1 (GPS):  MUX_0 = HIGH, MUX_1 = LOW
  if(channel != 0 && channel != 1) return;
  const bool want_mux0 = (channel == 1);
  const bool want_mux1 = false;

  // V2.5-Evo - 2026-07-25 - STAGE 0: count every mux switch that actually drives the pins.
  // Diagnostic only — nothing reads this except ?diag. Counted here, outside the mutex, so the
  // I2C critical section is not made any longer than it already is.
  g_diag_mux_switches++;

  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  aw.digitalWrite(AP_U1_MUX_0, want_mux0);
  aw.digitalWrite(AP_U1_MUX_1, want_mux1);

  // Bounded verify-and-correct: one read-back, at most one corrective re-write.
  if(aw.digitalRead(AP_U1_MUX_0) != want_mux0 || aw.digitalRead(AP_U1_MUX_1) != want_mux1)
  {
    // V2.5-Evo - 2026-07-25 - STAGE 0: a read-back mismatch means the write did NOT stick —
    // the documented motor-EMI-corrupts-I2C failure. Counting it is the whole point: until now
    // this event corrected itself silently, so a session log could never show how often the
    // UART was pointed at the wrong peripheral. The corrective re-write below is UNCHANGED.
    g_diag_mux_errors++;
    aw.digitalWrite(AP_U1_MUX_0, want_mux0);
    aw.digitalWrite(AP_U1_MUX_1, want_mux1);
  }
  xSemaphoreGive(i2cMutex);
}

void checkWetness()
{
  // ============================================================
  // E7 PULSE-AND-SNOOZE — called every ~10s via wetness_counter
  //
  // Behavior (not a latch):
  //   1. 2 consecutive all-wet calls (~20s) → set E7 (TX vibrates + shows warning)
  //   2. On the very next call (~10s later) → auto-clear E7 (TX display clears)
  //   3. Snooze for 27 calls (~270s) — silent even if still wet
  //   4. After snooze expires → repeat from step 1 if still wet
  //   Total alarm-to-alarm cycle: ~300s = 5 minutes
  //
  // Genuine dry-out at any point resets everything immediately.
  // Prevents false triggers from motor EMI (which corrupts AW9523 I2C reads
  // at high current and makes all 5 samples return LOW).
  // ============================================================
  static uint8_t wet_strike   = 0;   // Consecutive all-wet calls; needs >=2 to trigger
  static uint8_t snooze_count = 0;   // Calls remaining in snooze window

  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  aw.digitalWrite(AP_EN_WET_MEAS, HIGH);
  xSemaphoreGive(i2cMutex);
  vTaskDelay(pdMS_TO_TICKS(50));

  uint8_t dry_count = 0;
  for (uint8_t i = 0; i < 5; i++)
  {
    vTaskDelay(pdMS_TO_TICKS(50));
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    if (aw.digitalRead(AP_WET_MEAS)) dry_count++;
    xSemaphoreGive(i2cMutex);
  }

  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  aw.digitalWrite(AP_EN_WET_MEAS, LOW);
  xSemaphoreGive(i2cMutex);

  // --- Genuine dry-out: clear everything immediately, regardless of state ---
  if (dry_count >= 4)
  {
    wet_strike         = 0;
    snooze_count       = 0;
    telemetry.error_code = 0;
    return;
  }

  // --- Auto-clear active E7 alarm; TX had ~10s to display and vibrate ---
  // Start the 5-minute snooze so the user isn't spammed while riding back.
  if (telemetry.error_code == 71)
  {
    telemetry.error_code = 0;
    snooze_count         = 27;  // 27 calls × ~10s = 270s snooze; +10s alarm +20s confirm = 300s total
    wet_strike           = 0;
    return;
  }

  // --- Snooze: tick down; stay silent even if still wet ---
  if (snooze_count > 0)
  {
    snooze_count--;
    return;
  }

  // --- Normal detection window ---
  if (dry_count == 0)
  {
    // All 5 samples LOW: wet or I2C corruption from motor EMI.
    // Require 2 consecutive calls (~20s) before alarming.
    if (++wet_strike >= 2)
    {
      telemetry.error_code = 71;  // TX sees this within 100ms (10Hz LoRa); alarm auto-clears next call
      wet_strike           = 0;
    }
  }
  else
  {
    // Mixed result (1–3 of 5 HIGH): inconclusive, likely transient EMI — reset strike.
    wet_strike = 0;
  }
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
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    aw.digitalWrite(pin, LOW);
    xSemaphoreGive(i2cMutex);
    delay(200);
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    aw.digitalWrite(pin, HIGH);
    xSemaphoreGive(i2cMutex);
    delay(200);
  }
  delay(500);
  checkSerial();
}

void blinkBind(int num)
{
  for(int i = 0; i < num; i++)
  {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    aw.digitalWrite(AP_L_BIND, LOW);
    xSemaphoreGive(i2cMutex);
    delay(50);
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    aw.digitalWrite(AP_L_BIND, HIGH);
    xSemaphoreGive(i2cMutex);
    delay(50);
  }
}

// ===== I2C Scanner Function =====
void scanI2C() {
  byte error, address;
  int nDevices = 0;

  // V2.5-Evo fix (Bug 7): do not call Wire.begin() here. Wire was already initialised to the
  // correct pins (SDA=%d SCL=%d) in initHardware(). Re-initialising mid-session resets
  // the I2C peripheral and can glitch an in-progress AW9523 transaction.
  Serial.printf("Scanning I2C bus (initialized on SDA:%d SCL:%d)...\n", P_I2C_SDA, P_I2C_SCL);

  for(address = 1; address < 127; address++ ) {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    xSemaphoreGive(i2cMutex);

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

void cmdLogStat(const String& params);
void cmdDeleteLog(const String& params) {
  if(params.length() == 0) {
    Serial.println("Error: Specify filename (e.g. ?deletelog /filename.log)");
  } else {
    deleteLogFile(params.c_str());
  }
}

void cmdDeleteAllLogs(const String& params) {
  deleteAllLogFiles();
}

void cmdLogRate(const String& params) {
  if (params.length() > 0) {
    float rate = params.toFloat();
    setLogRate(rate);
  } else {
    Serial.println("Error: Specify rate in Hz (e.g., ?lograte 1 or ?lograte 0.1)");
  }
}

void cmdLogStat(const String& params) {
  extern volatile bool logging_active;
  extern volatile bool log_pending;
  extern uint32_t      log_pending_since;
  extern String        currentLogFileName;

  Serial.println("=== Logger State ===");
  Serial.printf("logging_active  : %s\n", logging_active ? "true" : "false");
  Serial.printf("log_pending     : %s\n", log_pending    ? "true" : "false");
  if (log_pending) {
    Serial.printf("pending_age_ms  : %u\n", (unsigned)(millis() - log_pending_since));
  }
  Serial.printf("currentLogFile  : %s\n", currentLogFileName.length() ? currentLogFileName.c_str() : "(none)");
  Serial.printf("gps_en (config) : %u\n", usrConf.gps_en);
  Serial.printf("logger_en       : %u\n", usrConf.logger_en);
  Serial.println("--- GPS (TinyGPS++) ---");
  Serial.printf("location.isValid: %s\n", gps.location.isValid() ? "YES" : "NO");
  Serial.printf("date.isValid    : %s\n", gps.date.isValid()     ? "YES" : "NO");
  Serial.printf("satellites      : %u\n", gps.satellites.value());
  Serial.printf("hdop            : %.1f\n", gps.hdop.hdop());
  Serial.printf("lat/lng         : %.6f / %.6f\n", gps.location.lat(), gps.location.lng());
  Serial.printf("chars processed : %u\n", gps.charsProcessed());
  Serial.printf("sentences       : %u\n", gps.sentencesWithFix());
  Serial.printf("failed checksum : %u\n", gps.failedChecksum());
  Serial.println("--- SPIFFS ---");
  Serial.printf("total KB        : %u\n", (unsigned)(SPIFFS.totalBytes() / 1024));
  Serial.printf("used  KB        : %u\n", (unsigned)(SPIFFS.usedBytes()  / 1024));
  Serial.printf("free  KB        : %u\n", (unsigned)((SPIFFS.totalBytes() - SPIFFS.usedBytes()) / 1024));
  Serial.println("====================");
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
    // Units: motCur = 0.01 A, batVolt = 0.1 V, fetTemp = 0.1 °C.
    float  snap_motcur_a  = -1.0f;
    long   snap_erpm      = -1L;
    float  snap_batvolt_v = -1.0f;
    float  snap_fettemp_c = -1.0f;
    if (vescMutex && xSemaphoreTake(vescMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      snap_motcur_a  = (float)vesc.motCur  / 100.0f;
      snap_erpm      = (long)vesc.erpm;
      snap_batvolt_v = (float)vesc.batVolt / 10.0f;
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
  // [0x02 start][0x01 payload-len=1][0x04 COMM_GET_VALUES][0x40 CRC16_HI][0x84 CRC16_LO][0x03 end]
  // V2.5-Evo - 2026-07-19 - CRC FIX: low byte was 0x07 (WRONG) -> 0x84. CRC16-CCITT/XMODEM
  // (poly 0x1021, init 0) over payload {0x04} = 0x4084, NOT 0x4007. The bad CRC made the VESC
  // silently drop this query (no reply at ANY baud), so ?vescraw always printed "NO BYTES" even
  // with a perfectly healthy VESC — a false negative that masked good hardware for a whole session.
  // Bench-proven over FTDI (both FW 6.05 and 6.06): 02 01 04 40 84 03 -> full GET_VALUES reply.
  // (The real telemetry path getValuesSelective()->sendToVESC()->vesc_crc16() was always correct.)
  static const uint8_t getValuesQuery[] = { 0x02, 0x01, 0x04, 0x40, 0x84, 0x03 };

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

// ============================================================
// cmdGpsDiag - GPS Feed Diagnostic for the RTM COG Heading Source
// ============================================================
//
// What it does:
//   Streams one diagnostic line at 2 Hz for up to 120 seconds showing the RX GPS
//   feed state AND the exact values the RTM heading ladder (getRtmHeading() in
//   RTMState.ino) reads when it decides whether GPS course-over-ground (COG) is a
//   valid heading source. This answers the open field question from the Fable audit:
//   why does rtm_source=1 (GPS COG) never engage?
//
//   Each line reports, in order:
//     loc/date/time fix validity, satellites, HDOP,
//     chars/sentences/checksum counters (NMEA parse health),
//     raw gps.speed.kmph() vs the captured gps_last_speed_kmh that RTM actually reads,
//     the COG value + its age (ms) + the RX fix age (ms),
//     and a PASS/FAIL breakdown of the four cog_valid sub-conditions used by
//     getRtmHeading(): course captured, course in range, age < 1500ms,
//     speed >= rtm_cog_min_speed_kmh. Also prints gps_rejected + suspect count.
//
//   Reading the [cap rng fresh spd>=N] flags tells you WHICH condition blocks COG:
//     spd=0  -> speed never reaches the threshold (COG stays gated at low speed), OR
//              gps_last_speed_kmh is not propagating from the parser.
//     fresh=0-> course is captured but stale (>1.5s) — parse rate or mux starvation.
//     cap=0  -> course never captured at all — module not emitting RMC/VTG course.
//
// How to invoke:
//   Type '?gpsdiag' in a serial terminal (or the web-UI quick-commands dropdown).
//   Drive the buggy above rtm_cog_min_speed_kmh while watching COG_VALID flip to YES.
//   Type 'quit' to abort early.
//
// Inputs:  params - unused
// Side effects: read-only on GPS globals; does not change any control or RTM state.
void cmdGpsDiag(const String& params) {
  extern float         gps_last_speed_kmh;
  extern float         gps_last_course_deg;
  extern unsigned long gps_last_course_ms;
  extern unsigned long gps_last_ms;
  extern bool          gps_rejected;
  extern uint8_t       gps_suspect_count;
  extern std::atomic<bool> rtm_rx_active;

  // Guard: this handler blocks the loop task for up to 120s. If invoked mid-RTM it
  // would freeze runRtmLoop()/Phase A/B/convergence/Gate 9 for that whole window.
  // Refuse and return when RTM is engaged — this is a bench/idle diagnostic only.
  // (Same rtm_rx_active flag the logger gate reads via isLoggerGated().)
  if (rtm_rx_active.load()) {
    Serial.println("?gpsdiag refused: not while RTM active - bench/idle only");
    return;
  }

  Serial.println("=== GPS Diagnostic (RTM COG heading source) ===");
  Serial.printf("rtm_use_compass=%u  rtm_cog_min_speed_kmh=%u  gps_chip_type=%u\n",
                (unsigned)usrConf.rtm_use_compass,
                (unsigned)usrConf.rtm_cog_min_speed_kmh,
                (unsigned)usrConf.gps_chip_type);
  Serial.println("Drive above the COG min speed and watch COG_VALID flip to YES. Type 'quit' to stop.");
  Serial.println();

  const uint32_t TEST_DURATION_MS = 120000UL;
  uint32_t start = millis();

  while ((millis() - start) < TEST_DURATION_MS) {
    esp_task_wdt_reset(); // prevent WDT timeout during the blocking loop
    if (checkSerialQuit()) break;

    unsigned long now = millis();

    // Raw TinyGPS++ speed vs the captured value RTM actually reads (gps_last_speed_kmh).
    // If raw is high but rtm_kmh stays 0, speed is not propagating past Phase A.
    float raw_kmh = gps.speed.isValid() ? (float)gps.speed.kmph() : -1.0f;

    // COG age and RX fix age (ms). -1 = timestamp never set (no reading yet).
    long cog_age = (gps_last_course_ms > 0) ? (long)(now - gps_last_course_ms) : -1;
    long fix_age = (gps_last_ms > 0)        ? (long)(now - gps_last_ms)        : -1;

    // Reproduce the four cog_valid sub-conditions from getRtmHeading() (RTMState.ino).
    bool c_captured = (gps_last_course_ms > 0);
    bool c_range    = (gps_last_course_deg >= 0.0f);
    bool c_fresh    = c_captured && ((now - gps_last_course_ms) < 1500UL);
    bool c_speed    = (gps_last_speed_kmh >= (float)usrConf.rtm_cog_min_speed_kmh);
    bool cog_valid  = c_captured && c_range && c_fresh && c_speed;

    Serial.printf(
      "loc=%d date=%d time=%d sats=%u hdop=%.1f | chars=%u sent=%u cksum=%u | "
      "raw_kmh=%.1f rtm_kmh=%.1f cog=%.1f cog_age=%ld fix_age=%ld | "
      "COG_VALID=%s [cap=%d rng=%d fresh=%d spd>=%u:%d] rejected=%d suspect=%u\n",
      (int)gps.location.isValid(), (int)gps.date.isValid(), (int)gps.time.isValid(),
      (unsigned)gps.satellites.value(), gps.hdop.hdop(),
      (unsigned)gps.charsProcessed(), (unsigned)gps.sentencesWithFix(), (unsigned)gps.failedChecksum(),
      raw_kmh, gps_last_speed_kmh, gps_last_course_deg, cog_age, fix_age,
      cog_valid ? "YES" : "no",
      (int)c_captured, (int)c_range, (int)c_fresh,
      (unsigned)usrConf.rtm_cog_min_speed_kmh, (int)c_speed,
      (int)gps_rejected, (unsigned)gps_suspect_count);

    vTaskDelay(pdMS_TO_TICKS(500)); // 2 Hz output rate
  }

  Serial.println("=== GPS diagnostic complete ===");
}

// ============================================================
// V2.5-Evo - 2026-07-25 - STAGE 0 PART D: ?diag / ?diagz
//
// A snapshot of the previous call, so every rate below can be reported as a delta over a
// bounded window instead of an ever-growing since-boot average that hides a fault that
// started ten minutes ago.
// ============================================================
struct DiagSnapshot {
  uint32_t t_ms;
  uint32_t gps_bytes;
  uint32_t gps_sentences;
  uint32_t cog_ts_updates;
  uint32_t cog_val_changes;
  uint32_t fix_age_sum_ms;
  uint32_t fix_age_samples;
  uint32_t mux_switches;
  uint32_t mux_errors;
  uint32_t vesc_polls;
  uint32_t vesc_ok;
  uint32_t loop_count;
  uint32_t loop_us_sum;
  uint8_t  origin;   // 0 = boot (no previous call), 1 = previous ?diag, 2 = ?diagz
};
// Zero-initialised on purpose: the very first ?diag then differences against "boot", which is
// exactly right — its window is millis() and its deltas are the totals since power-on.
static DiagSnapshot g_diag_prev = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// ============================================================
// cmdDiag - one-shot RX instrumentation snapshot (?diag)
// ============================================================
//
// What it does:
//   Prints ONE report and returns. Every rate is a delta since the previous ?diag call (or
//   since ?diagz, or since boot on the first call), so two calls bracket a window of interest
//   without any streaming and without a 'quit' to remember.
//
// Why this one is safe to run while RTM or Follow-Me is active:
//   It has no loop, no vTaskDelay, and touches no shared hardware — no I2C, no radio, no UART
//   mux, no SPIFFS. It reads plain counters and prints about ten lines. Contrast ?gpsdiag,
//   which streams for up to 120 s and therefore had to be given a hard refusal guard while RTM
//   is engaged (it would freeze runRtmLoop/Phase A/B/convergence/Gate 9 for that whole window).
//   ?diag needs no such guard: the only time it holds the loop task is the few milliseconds it
//   takes to push its own text out of the UART.
//
// What is reset and what is not:
//   The extremes (worst fix age, shortest/longest loop) are read AND reset by each call,
//   because the maximum of a running total is meaningless as a delta — resetting is what makes
//   "max in this window" true. Running totals are never reset here, only differenced.
//
// Inputs:  params - unused
// Outputs: report on Serial
// Side effects: resets the diagnostic extremes; updates the snapshot used by the next call.
//   Changes no configuration and no control state whatsoever.
void cmdDiag(const String& params) {
  extern unsigned long gps_last_ms;

  const uint32_t now_ms = millis();

  // Snapshot every running total in one go, so all the rates below describe the same instant.
  DiagSnapshot cur;
  cur.t_ms            = now_ms;
  cur.gps_bytes       = g_diag_gps_bytes;
  cur.gps_sentences   = g_diag_gps_sentences;
  cur.cog_ts_updates  = g_diag_cog_ts_updates;
  cur.cog_val_changes = g_diag_cog_val_changes;
  cur.fix_age_sum_ms  = g_diag_fix_age_sum_ms;
  cur.fix_age_samples = g_diag_fix_age_samples;
  cur.mux_switches    = g_diag_mux_switches;
  cur.mux_errors      = g_diag_mux_errors;
  cur.vesc_polls      = g_diag_vesc_polls;
  cur.vesc_ok         = g_diag_vesc_ok;
  cur.loop_count      = g_diag_loop_count;
  cur.loop_us_sum     = g_diag_loop_us_sum;
  cur.origin          = 1;   // this call becomes "the previous ?diag" for the next one

  // Extremes: read then reset, so each report's max belongs to that report's window.
  const uint32_t fix_age_max = g_diag_fix_age_max_ms; g_diag_fix_age_max_ms = 0;
  const uint32_t loop_max_us = g_diag_loop_max_us;    g_diag_loop_max_us    = 0;
  const uint32_t loop_min_us = g_diag_loop_min_us;    g_diag_loop_min_us    = 0xFFFFFFFF;

  // Unsigned subtraction is correct across a counter or millis() wrap.
  const uint32_t d_ms        = cur.t_ms            - g_diag_prev.t_ms;
  const uint32_t d_bytes     = cur.gps_bytes       - g_diag_prev.gps_bytes;
  const uint32_t d_sent      = cur.gps_sentences   - g_diag_prev.gps_sentences;
  const uint32_t d_cog_ts    = cur.cog_ts_updates  - g_diag_prev.cog_ts_updates;
  const uint32_t d_cog_val   = cur.cog_val_changes - g_diag_prev.cog_val_changes;
  const uint32_t d_fix_sum   = cur.fix_age_sum_ms  - g_diag_prev.fix_age_sum_ms;
  const uint32_t d_fix_n     = cur.fix_age_samples - g_diag_prev.fix_age_samples;
  const uint32_t d_mux_sw    = cur.mux_switches    - g_diag_prev.mux_switches;
  const uint32_t d_mux_err   = cur.mux_errors      - g_diag_prev.mux_errors;
  const uint32_t d_vesc_p    = cur.vesc_polls      - g_diag_prev.vesc_polls;
  const uint32_t d_vesc_ok   = cur.vesc_ok         - g_diag_prev.vesc_ok;
  const uint32_t d_loop_n    = cur.loop_count      - g_diag_prev.loop_count;
  const uint32_t d_loop_us   = cur.loop_us_sum     - g_diag_prev.loop_us_sum;

  // Never divide by zero: two ?diag calls in the same millisecond still produce a finite rate.
  float win_s = (float)d_ms / 1000.0f;
  if (win_s < 0.001f) win_s = 0.001f;

  const char* origin_txt = (g_diag_prev.origin == 2) ? "since ?diagz"
                         : (g_diag_prev.origin == 1) ? "since previous ?diag"
                                                     : "since boot (first call)";

  const uint8_t lvl = logResolveLevel();
  const char*   lvl_txt = (lvl >= 4) ? "Deep" : "Developer";

  // "-1" in the lines below always means "no sample in this window", never a real measurement.
  const long  fix_age_now  = (gps_last_ms != 0) ? (long)(now_ms - (uint32_t)gps_last_ms) : -1L;
  const long  fix_age_mean = (d_fix_n > 0) ? (long)(d_fix_sum / d_fix_n) : -1L;
  const float loop_min_ms  = (loop_min_us == 0xFFFFFFFF) ? -1.0f : ((float)loop_min_us / 1000.0f);
  const float loop_mean_ms = (d_loop_n > 0) ? ((float)d_loop_us / (float)d_loop_n / 1000.0f) : -1.0f;
  const float loop_max_ms  = (float)loop_max_us / 1000.0f;
  const float vesc_pct     = (d_vesc_p > 0) ? (100.0f * (float)d_vesc_ok / (float)d_vesc_p) : -1.0f;

  char cog_frozen[40];
  if (g_diag_cog_change_ms == 0) {
    snprintf(cog_frozen, sizeof(cog_frozen), "no COG value seen yet");
  } else {
    snprintf(cog_frozen, sizeof(cog_frozen), "value frozen %u s",
             (unsigned)((now_ms - g_diag_cog_change_ms) / 1000UL));
  }

  Serial.println("=== ?diag - RX instrumentation snapshot ===");
  Serial.printf("window     : %.2f s %s\n", win_s, origin_txt);
  Serial.printf("log_level  : cfg %u -> level %u (%s), %u bytes/record\n",
                (unsigned)usrConf.log_level, (unsigned)lvl, lvl_txt,
                (unsigned)logRecordSizeForLevel(lvl));
  Serial.printf("GPS feed   : %.0f bytes/s, %.1f sentences/s   [window %u B, %u sentences]\n",
                (float)d_bytes / win_s, (float)d_sent / win_s,
                (unsigned)d_bytes, (unsigned)d_sent);
  Serial.printf("GPS fix age: now %ld ms, mean %ld ms, max %u ms   [%u samples]\n",
                fix_age_now, fix_age_mean, (unsigned)fix_age_max, (unsigned)d_fix_n);
  Serial.printf("COG        : %.1f timestamp-updates/s vs %.1f value-changes/s   [%s]\n",
                (float)d_cog_ts / win_s, (float)d_cog_val / win_s, cog_frozen);
  Serial.printf("UART mux   : %.1f switches/s, %u read-back failures   [%u total since boot]\n",
                (float)d_mux_sw / win_s, (unsigned)d_mux_err, (unsigned)cur.mux_errors);
  Serial.printf("VESC poll  : %u/%u ok (%.1f%%)\n",
                (unsigned)d_vesc_ok, (unsigned)d_vesc_p, vesc_pct);
  Serial.printf("loop()     : min %.2f ms, mean %.2f ms, max %.2f ms   [%u loops]\n",
                loop_min_ms, loop_mean_ms, loop_max_ms, (unsigned)d_loop_n);
  Serial.println("HOW TO READ IT:");
  Serial.println("  timestamp-updates high with value-changes near 0 = GPS is repeating a frozen");
  Serial.println("  heading. cog_age looks healthy in that state; it is not. Any COG-derived");
  Serial.println("  heading is stale, and RTM/FM steering built on it is steering on old news.");
  Serial.println("  -1 anywhere above means no sample in this window, not a real measurement.");
  Serial.println("  ?diagz zeroes the counters so a run can be bracketed cleanly.");
  Serial.println("===========================================");

  g_diag_prev = cur;
}

// ============================================================
// cmdDiagZ - zero the diagnostic counters (?diagz)
// ============================================================
//
// What it does:
//   Resets the running totals and the extremes so the next ?diag reports a clean window. Use it
//   to bracket a run: ?diagz, do the thing, ?diag.
//
// What it deliberately does NOT reset, and why:
//   g_diag_cog_change_ms   - a state timestamp ("when did the heading last actually move"),
//                            not a counter. Zeroing it would make the log and ?diag both claim
//                            "no COG value ever seen", which would be a lie.
//   g_diag_loop_max_us_log - owned by the logger, consumed once per level-4 record. Zeroing it
//                            here would silently steal a peak from an active session log.
//   g_diag_gps_sent_per_s  - a derived rate; getGPSLoop() refreshes it within one second.
//
// Inputs: params - unused. Outputs: confirmation on Serial. Side effects: as described above.
void cmdDiagZ(const String& params) {
  g_diag_gps_bytes       = 0;
  g_diag_gps_sentences   = 0;
  g_diag_cog_ts_updates  = 0;
  g_diag_cog_val_changes = 0;
  g_diag_fix_age_sum_ms  = 0;
  g_diag_fix_age_samples = 0;
  g_diag_fix_age_max_ms  = 0;
  g_diag_mux_switches    = 0;
  g_diag_mux_errors      = 0;
  g_diag_vesc_polls      = 0;
  g_diag_vesc_ok         = 0;
  g_diag_loop_count      = 0;
  g_diag_loop_us_sum     = 0;
  g_diag_loop_min_us     = 0xFFFFFFFF;
  g_diag_loop_max_us     = 0;

  DiagSnapshot fresh = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  fresh.t_ms   = millis();
  fresh.origin = 2;   // so the next ?diag says "since ?diagz"
  g_diag_prev  = fresh;

  Serial.println("?diagz: diagnostic counters zeroed. Run your test, then ?diag to read the window.");
  Serial.println("?diagz: the COG frozen-clock and the logger's own loop-peak are intentionally left alone.");
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
  {"deleteallogs", "delete all log files (skips active log)", cmdDeleteAllLogs},
  {"lograte", "<Hz> set log rate (e.g. 1 or 0.1)", cmdLogRate},
  {"logstat", "dump logger + GPS state (diagnose why logging fails)", cmdLogStat},
  
  // --- Hardware Diagnostics ---
  {"i2c", "scan I2C bus for compass", cmdScanI2C},
  {"gpsdiag", "2Hz GPS feed + RTM COG-valid breakdown (diagnose why GPS COG heading never engages)", cmdGpsDiag},
  {"diag", "one-shot snapshot: GPS bytes/sentences, fix age, COG updates vs value-changes, mux errors, VESC poll rate, loop min/mean/max (safe during RTM/FM)", cmdDiag},
  {"diagz", "zero the ?diag counters so a run can be bracketed", cmdDiagZ},
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
    esp_task_wdt_reset(); // V2.5-Evo fix (I3): prevent WDT panic during blocking debug command
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
        esp_task_wdt_reset(); // V2.5-Evo fix (I3): prevent WDT panic during blocking debug command
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
    esp_task_wdt_reset(); // V2.5-Evo fix (I3): prevent WDT panic during blocking debug command
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
    esp_task_wdt_reset(); // V2.5-Evo fix (I3): prevent WDT panic during blocking debug command
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
    esp_task_wdt_reset(); // V2.5-Evo fix (I3): prevent WDT panic during blocking debug command
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
    esp_task_wdt_reset(); // V2.5-Evo fix (I3): prevent WDT panic during blocking debug command
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
    esp_task_wdt_reset(); // V2.5-Evo fix (I3): prevent WDT panic during blocking debug command
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
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    bool s_bind_boot = !aw.digitalRead(AP_S_BIND);
    xSemaphoreGive(i2cMutex);
    if(s_bind_boot)
    {
      xSemaphoreTake(i2cMutex, portMAX_DELAY);
      bool s_aux_boot = !aw.digitalRead(AP_S_AUX);
      xSemaphoreGive(i2cMutex);
      if(s_aux_boot)
      {
        delay(10);
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        s_aux_boot = !aw.digitalRead(AP_S_AUX);
        xSemaphoreGive(i2cMutex);
        if(s_aux_boot)
        {
          Serial.println("Deleting config and rebooting");
          deleteConfFromSPIFFS();
          delay(1000);
          ESP.restart();
        }
      }
      delay(10);
      xSemaphoreTake(i2cMutex, portMAX_DELAY);
      s_bind_boot = !aw.digitalRead(AP_S_BIND);
      xSemaphoreGive(i2cMutex);
      if(s_bind_boot)
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
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  bool aux_current = aw.digitalRead(AP_S_AUX);
  xSemaphoreGive(i2cMutex);
  // Detect a "falling edge" (button was just pressed down)
  if (aux_last_state == true && aux_current == false)
  {
    vTaskDelay(pdMS_TO_TICKS(50));
    // 50ms Debounce to prevent double-clicks
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    bool aux_debounced = (aw.digitalRead(AP_S_AUX) == false);
    xSemaphoreGive(i2cMutex);
    if (aux_debounced)
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
      xSemaphoreTake(i2cMutex, portMAX_DELAY);
      bool aux_held = (aw.digitalRead(AP_S_AUX) == false);
      xSemaphoreGive(i2cMutex);
      while(aux_held) {
        vTaskDelay(pdMS_TO_TICKS(10));
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        aux_held = (aw.digitalRead(AP_S_AUX) == false);
        xSemaphoreGive(i2cMutex);
      }
    }
  }
  aux_last_state = aux_current;

  // --- RUNTIME BIND: COMPASS CALIBRATION ---
  // Short BIND press (falling edge, 50ms debounce) triggers 45s calibration.
  // blinkBind(5) = starting, blinkBind(2) = success, blinkBind(10) = compass not detected.
  // Boot-time pairing/reset cannot reach this block (guarded by first_call above).
  static bool bind_last_state = true;
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  bool bind_current = aw.digitalRead(AP_S_BIND);
  xSemaphoreGive(i2cMutex);
  if (bind_last_state == true && bind_current == false) {
    vTaskDelay(pdMS_TO_TICKS(50));
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    bool bind_debounced = (aw.digitalRead(AP_S_BIND) == false);
    xSemaphoreGive(i2cMutex);
    if (bind_debounced) {
      blinkBind(5);
      extern bool compass_detected;  // global bool set by initCompass(); false if sensor absent
      runCompassCalibration();        // 45s collection, hard/soft-iron calc, auto-save to SPIFFS
      if (compass_detected) {
        blinkBind(2);
      } else {
        blinkBind(10);
      }
      xSemaphoreTake(i2cMutex, portMAX_DELAY);
      bool bind_held = (aw.digitalRead(AP_S_BIND) == false);
      xSemaphoreGive(i2cMutex);
      while (bind_held) {
        vTaskDelay(pdMS_TO_TICKS(10));
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        bind_held = (aw.digitalRead(AP_S_BIND) == false);
        xSemaphoreGive(i2cMutex);
      }
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
          xSemaphoreTake(i2cMutex, portMAX_DELAY);
          aw.digitalWrite(AP_L_BIND, LOW);
          xSemaphoreGive(i2cMutex);
        }
      }
      else
      {
        if(bind_pin_state)
        {
          bind_pin_state = 0;
          xSemaphoreTake(i2cMutex, portMAX_DELAY);
          aw.digitalWrite(AP_L_BIND, HIGH);
          xSemaphoreGive(i2cMutex);
        }
        else
        {
          bind_pin_state = 1;
          xSemaphoreTake(i2cMutex, portMAX_DELAY);
          aw.digitalWrite(AP_L_BIND, LOW);
          xSemaphoreGive(i2cMutex);
        }
      }
    }
    else
    {
      unpairedBlink++;
      if(unpairedBlink == 4)
      {
        unpairedBlink = 0;
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        aw.digitalWrite(AP_L_BIND, LOW);
        xSemaphoreGive(i2cMutex);
        vTaskDelay(pdMS_TO_TICKS(10));
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        aw.digitalWrite(AP_L_BIND, HIGH);
        xSemaphoreGive(i2cMutex);
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}