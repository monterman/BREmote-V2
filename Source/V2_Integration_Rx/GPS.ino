// V2.5-Evo - 2026-05-06 - D1: Capture GPS course-over-ground (gps_last_course_deg/ms) for future RTM heading source
// V3 - 2026-04-30 - Rename: gps_max_jump_kmh → gps_max_teleport_kmh (clarity)
// V3 - 2026-04-30 - Bundle E: replaced 300ms blocking serial drain with non-blocking while(available()) drain
// V3 - 2026-04-24 - Added Phase B FIELD SERVICE NOTE (sizeof confStruct 128→136)
// V3 - 2026-04-22 - Added gps_chip_type branch: type 0/1=BN-220/BN-880 (9600→115200, 5Hz), type 2/3=M10 (115200 direct, 10Hz, all constellations)
// V3 - 2026-04-22 - Added Phase A GPS anti-spoofing: HDOP check, teleport check, acceleration check (gpsPhaseACheck)

// ============================================================
// FIELD SERVICE NOTE
//
// V3 - 2026-04-22: sizeof(confStruct) changed from 108 to 112
// bytes (gps_chip_type added). On the first V3 flash, SPIFFS
// will detect the size mismatch and reset ALL settings to
// defaults. After flashing, you must:
//   1) Re-pair TX and RX
//   2) Re-configure all settings via the web UI
//   3) Re-calibrate compass via the 'runcal' serial command
//
// V3 - 2026-04-22: sizeof(confStruct) changed from 112 to 128
// bytes (Phase A anti-spoofing params added). On the first
// flash after this change, SPIFFS will again detect the size
// mismatch and reset ALL settings to defaults. After flashing:
//   1) Re-pair TX and RX
//   2) Re-configure all settings via the web UI
//   3) Re-calibrate compass via the 'runcal' serial command
//   4) Verify anti-spoofing defaults in "GPS & Follow-Me"
//      section of the web UI (HDOP 2.0, Accel 3.0G, Jump 200
//      km/h, Threshold 3) — adjust if needed.
//
// Also verify that gps_chip_type in the web config matches
// the physical GPS module connected to this board.
// ============================================================

// ============================================================
// V3 - 2026-04-24: sizeof(confStruct) changed from 128 to 136
// bytes (Phase B GPS handshake params added). On the first
// flash after this change, SPIFFS will again detect the size
// mismatch and reset ALL settings to defaults. After flashing:
//   1) Re-pair TX and RX
//   2) Re-configure all settings via the web UI
//   3) Re-calibrate compass via the 'runcal' serial command
//   4) Verify Phase B defaults in "GPS & Follow-Me" section of
//      the web UI (Pair Dist 500 m, Speed Diff 50 km/h)
// ============================================================

// ============================================================
// PHASE A GPS ANTI-SPOOFING STATE
// File-scope variables that persist between getGPSLoop() calls.
// These are NOT static so the RTM state machine (GPS.ino future)
// can read gps_rejected without an accessor function.
// ============================================================
uint8_t       gps_suspect_count  = 0;     // consecutive suspicious readings; resets on any clean reading
bool          gps_rejected       = false; // true = GPS marked rejected; blocks RTM arming
double        gps_last_lat       = 0.0;   // last accepted latitude (degrees)
double        gps_last_lng       = 0.0;   // last accepted longitude (degrees)
float         gps_last_speed_kmh  = 0.0;   // last accepted speed (km/h)
unsigned long gps_last_ms         = 0;     // millis() timestamp of last accepted reading
float         gps_last_course_deg = -1.0f; // Last valid GPS course over ground (degrees, 0–360 clockwise from North). -1.0f = no valid reading yet.
unsigned long gps_last_course_ms  = 0;     // millis() timestamp of last valid course update. 0 = no valid reading yet.

// ============================================================
// gpsPhaseACheck - Phase A GPS anti-spoofing validation
// ============================================================
//
// What it does:
//   Validates one GPS reading against three independent checks:
//
//   1) HDOP check: reject if HDOP > usrConf.gps_max_hdop
//      (poor satellite geometry = untrustworthy position)
//
//   2) Teleport check: reject if the position change since the
//      last accepted reading implies travel faster than
//      usrConf.gps_max_teleport_kmh km/h (physically impossible)
//
//   3) Acceleration check: reject if the speed change since the
//      last accepted reading implies acceleration >
//      usrConf.gps_max_accel_g G (physically impossible for a
//      ground vehicle)
//
//   Checks 2 and 3 are skipped on the very first accepted
//   reading (gps_last_ms == 0) because there is no history to
//   compare against.
//
// Inputs:
//   cur_lat, cur_lng - current GPS position (degrees, from TinyGPS++)
//   cur_speed_kmh    - current GPS speed (km/h, from gps.speed.kmph())
//
// Returns:
//   true  = all checks passed (safe to accept this reading)
//   false = at least one check failed (treat as suspicious)
//
// Side effects:
//   Reads module-level state: gps_last_lat, gps_last_lng,
//   gps_last_ms, gps_last_speed_kmh.
//   Reads usrConf.gps_max_hdop, gps_max_teleport_kmh, gps_max_accel_g.
//   Prints diagnostics to Serial when a check fails.
// ============================================================
static bool gpsPhaseACheck(double cur_lat, double cur_lng, float cur_speed_kmh) {
  // ---- Check 1: HDOP ----
  // TinyGPS++ gps.hdop.value() returns HDOP * 100 as an integer.
  // Divide by 100.0 to get the real HDOP float for comparison.
  if (gps.hdop.isValid() && (float)gps.hdop.value() / 100.0f > usrConf.gps_max_hdop) {
    Serial.printf("GPS [PhA] HDOP %.1f exceeds max %.1f — reading rejected\n",
                  (float)gps.hdop.value() / 100.0f, usrConf.gps_max_hdop);
    return false;
  }

  // ---- Checks 2 & 3: require at least one prior accepted reading ----
  if (gps_last_ms > 0) {
    float dt_s = (float)(millis() - gps_last_ms) / 1000.0f;

    // Guard against near-zero dt (duplicate call, millis() wrap) to avoid division by zero.
    if (dt_s > 0.05f) {

      // ---- Check 2: teleport ----
      // TinyGPSPlus::distanceBetween() returns metres between two lat/lng pairs.
      float dist_m      = (float)TinyGPSPlus::distanceBetween(
                                   gps_last_lat, gps_last_lng, cur_lat, cur_lng);
      float implied_kmh = (dist_m / dt_s) * 3.6f;
      if (implied_kmh > usrConf.gps_max_teleport_kmh) {
        Serial.printf("GPS [PhA] Teleport %.0f km/h exceeds max %.0f km/h — reading rejected\n",
                      implied_kmh, usrConf.gps_max_teleport_kmh);
        return false;
      }

      // ---- Check 3: acceleration ----
      // Convert speed delta from km/h to m/s, then divide by dt to get m/s², then by 9.81 for G.
      float delta_v_ms = fabsf(cur_speed_kmh - gps_last_speed_kmh) / 3.6f;
      float accel_g    = (delta_v_ms / dt_s) / 9.81f;
      if (accel_g > usrConf.gps_max_accel_g) {
        Serial.printf("GPS [PhA] Accel %.2f G exceeds max %.2f G — reading rejected\n",
                      accel_g, usrConf.gps_max_accel_g);
        return false;
      }
    }
  }

  return true;  // all checks passed
}

// ============================================================
// configureGPS - Initialize GPS hardware on Serial1 (UART1)
// ============================================================
//
// What it does:
//   Sends UBX binary commands to configure the GPS module
//   attached to Serial1. The specific init sequence depends on
//   usrConf.gps_chip_type:
//     0 = BN-220 (no compass): 9600→115200, 5Hz
//     1 = BN-880 (GPS+compass): same GPS init as BN-220;
//         compass (QMC5883L) is initialized separately by
//         initCompass() in Compass.ino.
//     2 = M10 (no compass): 115200 native, 10Hz, all constellations
//     3 = M10 (GPS+compass): same GPS init as type 2; compass
//         initialized separately by initCompass().
//
// Inputs:
//   Reads usrConf.gps_chip_type.
//
// Outputs:
//   None. Side effect: Serial1 is configured and left open.
//
// Side effects:
//   - Calls setUartMux(1) to switch UART mux to GPS path.
//   - Calls Serial1.begin()/end()/begin() for BN-220/BN-880.
//   - Blocks ~450 ms for BN-220/BN-880; ~250 ms for M10.
// ============================================================
void configureGPS() {
  // Route UART1 to the GPS connector (MUX position 1).
  // This must happen before any Serial1 traffic regardless of chip type.
  setUartMux(1);

  // V3 fix (I-2): Increase RX buffer to 512 bytes before any Serial1.begin().
  // At 10Hz (M10) the GPS emits multiple NMEA sentences per cycle; the default
  // 256-byte buffer can overflow between loop ticks, causing sentence fragments
  // that confuse TinyGPS++. setRxBufferSize() MUST be called before begin().
  Serial1.setRxBufferSize(512);

  // V3 - 2026-04-22 - Branch on GPS chip type. Each chip type has a
  // different factory baud rate and supported feature set.
  switch (usrConf.gps_chip_type)
  {
    // --------------------------------------------------------
    // Types 0 and 1: BN-220 / BN-880 — factory 9600, switch to 115200, 5Hz
    // Both use the same GPS init sequence. For type 1, initCompass()
    // in Compass.ino handles the QMC5883L separately.
    // --------------------------------------------------------
    case 0:
    case 1:
    {
      // UBX-CFG-PRT: switch GPS UART to 115200 baud, 8N1, UBX+NMEA.
      // Fletcher-8 checksum (0xC0, 0x7E) pre-calculated over class→payload.
      byte setBaud[] = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
        0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00,
        0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E
      };

      // UBX-CFG-RATE: measurement rate 200 ms = 5 Hz, GPS time reference.
      // Fletcher-8 checksum (0xDE, 0x6A) pre-calculated.
      byte setRate5Hz[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00,
        0x01, 0x00, 0xDE, 0x6A
      };

      // Step 1: open at factory default baud so the GPS hears the baud command.
      Serial.println("GPS [BN-220/880]: Connecting at 9600...");
      Serial1.begin(9600, SERIAL_8N1, P_U1_RX, P_U1_TX);
      delay(200);

      // Step 2: send baud-change command; short delay lets UART finish shifting.
      Serial1.write(setBaud, sizeof(setBaud));
      Serial1.flush();
      delay(50);

      // Step 3: reopen our side at 115200 to match the GPS after it switches.
      Serial1.end();
      delay(100);
      Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);
      Serial.println("GPS [BN-220/880]: Baud switched to 115200");

      // Step 4: send the 5Hz measurement-rate command.
      delay(100);
      Serial1.write(setRate5Hz, sizeof(setRate5Hz));
      Serial1.flush();
      Serial.println("GPS [BN-220/880]: Config complete (115200, 5Hz)");
      break;
    }

    // --------------------------------------------------------
    // Types 2 and 3: M10 — 115200 native, 10Hz, all constellations
    // For type 3, initCompass() in Compass.ino handles QMC5883L separately.
    // --------------------------------------------------------
    case 2:
    case 3:
    {
      // M10 boots at 115200 by default — no baud-switch command needed.
      Serial.println("GPS [M10]: Connecting at 115200...");
      Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);
      delay(200);

      // UBX-CFG-RATE: measurement period 100 ms = 10 Hz, GPS time reference.
      // Fletcher-8 checksum: CK_A=0x7A, CK_B=0x12 (pre-calculated).
      byte setRate10Hz[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00,
        0x01, 0x00, 0x7A, 0x12
      };
      Serial1.write(setRate10Hz, sizeof(setRate10Hz));
      Serial1.flush();
      delay(50);

      // UBX-CFG-GNSS: enable GPS, Galileo, BDS (BeiDou), and GLONASS.
      // Payload: 4-byte header + 4 blocks of 8 bytes each (one per constellation).
      // Fletcher-8 checksum: CK_A=0xCE, CK_B=0xC0 (pre-calculated over class→payload).
      byte setGNSS[] = {
        0xB5, 0x62, 0x06, 0x3E, 0x24, 0x00,
        0x00, 0x00, 0xFF, 0x04,
        0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00,  // GPS (id=0)
        0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00,  // Galileo (id=2)
        0x03, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00,  // BDS (id=3)
        0x06, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00,  // GLONASS (id=6)
        0xCE, 0xC0
      };
      Serial1.write(setGNSS, sizeof(setGNSS));
      Serial1.flush();
      Serial.println("GPS [M10]: Config complete (115200, 10Hz, GPS+Galileo+BDS+GLONASS)");
      break;
    }

    default:
      // Unknown chip type — fall back to BN-880 sequence so the board
      // at least attempts to start. Log the unexpected value.
      // Note (N-2): RX falls back to BN-880 init rather than skipping entirely.
      // TX GPS.ino skips init on unknown types because getTxGPSLoop() guards on
      // tx_gps_initialized and the TX can survive without GPS (speed display only).
      // On RX, GPS feeds safety-critical follow-me and anti-spoofing; a partial
      // attempt at a known-good sequence is safer than leaving Serial1 unconfigured.
      Serial.print("GPS: unknown gps_chip_type=");
      Serial.println(usrConf.gps_chip_type);
      Serial.println("GPS: falling back to BN-880 init — check web config");
      {
        byte setBaud[] = {
          0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
          0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00,
          0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E
        };
        byte setRate5Hz[] = {
          0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00,
          0x01, 0x00, 0xDE, 0x6A
        };
        Serial1.begin(9600, SERIAL_8N1, P_U1_RX, P_U1_TX);
        delay(200);
        Serial1.write(setBaud, sizeof(setBaud));
        Serial1.flush();
        delay(50);
        Serial1.end();
        delay(100);
        Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);
        delay(100);
        Serial1.write(setRate5Hz, sizeof(setRate5Hz));
        Serial1.flush();
      }
      break;
  }
}

// Task for GPS reading
void getGPSLoop()
{
  setUartMux(1);
  vTaskDelay(pdMS_TO_TICKS(10));  
  //Serial1.end();
  //Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);
  //while(!Serial1) vTaskDelay(pdMS_TO_TICKS(10));
  
  // Flush the serial buffer to get fresh data
  Serial1.flush();
  
  // Reset buffer for new reading
  // Non-blocking drain: read all bytes currently in the UART buffer and return immediately.
  // The GPS module sends NMEA sentences at its configured rate (5-10Hz); calling getGPSLoop()
  // on its own timer (V2_Integration_Rx.ino gps_loop_timer) ensures each call arrives after
  // at least one sentence interval, so nothing is missed and loop() is never blocked.
  bool newData = false;
  while (Serial1.available())
  {
    char c = Serial1.read();
    if (gps.encode(c)) newData = true;
  }

  // V3 - 2026-04-25 - Fix: use isValid() not isUpdated() for speed check — isUpdated() fails when stationary blocking Phase B
  if (!newData || !gps.location.isValid() || !gps.speed.isValid()) {
    // No valid fix or no valid speed — not a spoof event, just no usable data.
    telemetry.foil_speed = 0xFF;  // 0xFF = no data (V3 fix N-4: 99 collides with real speed)
  } else {
    double cur_lat   = gps.location.lat();
    double cur_lng   = gps.location.lng();
    float  cur_speed = (float)gps.speed.kmph();

    if (gpsPhaseACheck(cur_lat, cur_lng, cur_speed)) {
      // Reading passed all Phase A checks — accept it.
      // Reset suspicion state and update the "last known good" snapshot.
      gps_suspect_count  = 0;
      gps_rejected       = false;
      gps_last_lat       = cur_lat;
      gps_last_lng       = cur_lng;
      gps_last_speed_kmh = cur_speed;
      gps_last_ms        = millis();

      // V2.5-Evo - 2026-05-06 - Capture GPS course-over-ground for use as heading source.
      // gps.course.deg() is unreliable when the buggy is stationary or moving very slowly
      // (typically < 3 km/h), but at higher speeds it is the most accurate heading source
      // available — unaffected by motor-current EMI that biases the compass.
      // We capture it here unconditionally when valid; consumers (RTM steering) will gate
      // on speed and age before using the value.
      if (gps.course.isValid()) {
        gps_last_course_deg = (float)gps.course.deg();
        gps_last_course_ms  = millis();
      }

      // Cap at 254: 0xFF (255) is the reserved "no data" sentinel.
      telemetry.foil_speed = (cur_speed >= 254.0f) ? 254 : (uint8_t)cur_speed;
    } else {
      // Reading failed at least one Phase A check — track consecutive failures.
      if (gps_suspect_count < 255) gps_suspect_count++;
      if (gps_suspect_count >= usrConf.gps_suspect_threshold && !gps_rejected) {
        gps_rejected = true;
        Serial.printf("GPS [PhA] REJECTED after %u consecutive failures — RTM arming blocked\n",
                      (unsigned)gps_suspect_count);
      }
      // Do not expose spoofed/suspicious data via telemetry.
      telemetry.foil_speed = 0xFF;
    }
  }
}

// Function to print satellite information
void printSatelliteInfo() {
  Serial.println("----- GPS Satellite Status -----");
  Serial.print("Satellites in view: ");
  Serial.println(gps.satellites.value());
  
  Serial.print("HDOP (Horizontal Dilution of Precision): ");
  if (gps.hdop.isValid()) {
    Serial.print(gps.hdop.value());
    Serial.println(" (Lower is better, <1 Excellent, 1-2 Good, 2-5 Moderate, 5-10 Fair, >10 Poor)");
  } else {
    Serial.println("Invalid");
  }
  
  Serial.print("Location validity: ");
  Serial.println(gps.location.isValid() ? "Valid" : "Invalid");
  
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    if (gps.altitude.isValid()) {
      Serial.print(gps.altitude.meters());
      Serial.println(" meters");
    } else {
      Serial.println("Invalid");
    }
  }
  
  Serial.print("Date/Time validity: ");
  Serial.println(gps.date.isValid() && gps.time.isValid() ? "Valid" : "Invalid");
  
  if (gps.date.isValid() && gps.time.isValid()) {
    char dateTime[30];
    sprintf(dateTime, "%04d-%02d-%02d %02d:%02d:%02d UTC", 
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
    Serial.print("Date/Time: ");
    Serial.println(dateTime);
  }
  
  Serial.print("Course validity: ");
  Serial.println(gps.course.isValid() ? "Valid" : "Invalid");
  
  if (gps.course.isValid()) {
    Serial.print("Course: ");
    Serial.print(gps.course.deg());
    Serial.println(" degrees");
  }
  
  Serial.print("Chars processed: ");
  Serial.println(gps.charsProcessed());
  Serial.print("Sentences with fix: ");
  Serial.println(gps.sentencesWithFix());
  Serial.print("Failed checksum: ");
  Serial.println(gps.failedChecksum());
  
  Serial.println("-------------------------------");
}