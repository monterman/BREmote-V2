// V2.5-Evo - 2026-05-06 - DIAG: GSV/GLL/VTG disable commented out; txGpsColdReset() added
// V2.5-Evo - 2026-05-06 - FIX-GPS-1: dual-baud init in initTxGPS() to prevent UART RX lockout from retained-config baud mismatch
// V2.5-Evo - 2026-04-21 - New TX GPS module: UBX init (115200/5Hz) and non-blocking speed polling for speed_src 2/3/5
// V2.5-Evo - 2026-04-22 - Added speed_src guard to initTxGPS(); 512-byte RX buffer; NMEA sentence filtering (GPGSV/GPGLL/GPVTG disabled); HDOP gate in getTxGPSLoop()
// V2.5-Evo - 2026-04-22 - Added gps_chip_type branch: type 0=BN-220 (existing path), type 2=M10 (115200 direct, 10Hz, all constellations)
// V2.5-Evo - 2026-05-03 - Decouple TX GPS init from speed_src (M1 audit fix).
//                   GPS now inits whenever gps_en=1 — Phase B anti-spoofing
//                   no longer silently broken for RX-speed users.

// ============================================================
// TX GPS - Reads GPS data over Serial1 on the TX (handheld
//          remote). Feeds the SP (Speed) display mode when the
//          user selects a TX-GPS option via the web config:
//              speed_src == 2  -> km/h
//              speed_src == 3  -> knots
//              speed_src == 5  -> mph
//          All other speed_src values (0, 1, 4) continue to use
//          the speed that arrives in the LoRa telemetry packet
//          from the RX (handled in Radio.ino, untouched here).
//
//          Supported GPS chips (selected via usrConf.gps_chip_type):
//              0 = BN-220 (factory 9600 baud → 115200, 5Hz)
//              2 = M10    (native 115200, 10Hz, multi-constellation)
//          Types 1 and 3 (compass variants) are rejected on TX
//          because the TX hardware has no compass connector.
//
// Design goals:
//   - Never block the main loop or the 10Hz LoRa TX cycle
//   - Never stress the watchdog
//   - Fail safely: if the GPS is missing or loses fix, publish
//     the sentinel 0xFF so the display helper renders "--"
//   - Nothing hardcoded that a user might want to change:
//     enable flag, chip type, stale timeout, and display unit
//     all come from usrConf (SPIFFS), per the project's standing rule
// ============================================================

// Has initTxGPS() actually configured Serial1 yet? Used by
// getTxGPSLoop() to avoid poking an uninitialized UART (e.g.
// if the user enables gps_en via web config without rebooting).
static bool tx_gps_initialized = false;

// ============================================================
// initTxGPS - Initialize Serial1 GPS based on usrConf.gps_chip_type
// ============================================================
//
// What it does:
//   Branches on usrConf.gps_chip_type and sends the appropriate
//   UBX binary commands to the GPS module attached to Serial1:
//     Type 0 (BN-220):
//       1) UBX-CFG-PRT : switch from factory 9600 baud to 115200
//       2) UBX-CFG-RATE: set measurement rate to 5Hz (200ms period)
//       3) UBX-CFG-MSG : disable GPGSV, GPGLL, GPVTG sentences
//     Type 2 (M10):
//       1) UBX-CFG-RATE: set measurement rate to 10Hz (100ms period)
//       2) UBX-CFG-GNSS: enable GPS+Galileo+BDS+GLONASS constellations
//       3) UBX-CFG-MSG : disable GPGSV, GPGLL, GPVTG sentences
//     Types 1/3 (compass variants): rejected by cfgValidateCrossField()
//       before this is reached — TX has no compass hardware.
//     Unknown types: skipped with a Serial error message.
//
// Inputs:
//   - usrConf.gps_en        : master enable; returns early if 0.
//   - usrConf.gps_chip_type : selects the init sequence (0 or 2 on TX).
//   (speed_src no longer gates init — GPS runs whenever gps_en=1.)
//
// Outputs:
//   None (no return value).
//
// Side effects:
//   - Calls Serial1.setRxBufferSize(512) before any begin().
//   - For type 0: dual-baud: begin(115200)/end()/begin(9600)/end()/begin(115200).
//     Blocks ~750ms across five delays.
//   - For type 2: same dual-baud sequence. Blocks ~750ms.
//   - Sets tx_gps_initialized = true on success. No UBX ACK is
//     verified — the flag means "init attempted".
//
// Reboot-required behavior (documented for the user):
//   Changing gps_en or gps_chip_type via the web config at runtime
//   does NOT reinitialize the GPS. A reboot is required.
// ============================================================
void initTxGPS()
{
  // Master switch: if GPS is disabled in config, do nothing.
  // This keeps the UART free and avoids boot delay on units not using GPS.
  if (!usrConf.gps_en)
  {
    Serial.println("TX GPS: disabled (gps_en=0), skipping init");
    return;
  }

  // TX GPS initializes whenever gps_en=1, regardless of speed_src.
  // Anti-spoofing (Phase B) always needs TX GPS — it compares RX vs TX position,
  // independent of which speed source the user has selected for display.

  // V2.5-Evo - 2026-04-22 - Increase RX buffer to 512 bytes before any begin().
  // At 115200 baud / 5-10Hz the GPS emits several NMEA sentences per cycle;
  // 256 bytes can overflow between loop ticks and cause sentence fragments
  // that confuse TinyGPS++. setRxBufferSize() MUST be called before begin().
  Serial1.setRxBufferSize(512);

  // V2.5-Evo - 2026-04-22 - Branch on GPS chip type. Each chip type requires a
  // different init sequence (factory baud, rate, constellation config).
  // TX hardware has no compass, so types 1/3 are blocked by ConfigService.
  switch (usrConf.gps_chip_type)
  {
    // --------------------------------------------------------
    // Type 0: BN-220 — factory 9600 baud, switch to 115200, 5Hz
    // --------------------------------------------------------
    case 0:
    {
      // UBX-CFG-PRT: configure the GPS UART for 115200 baud, 8N1,
      // with both UBX and NMEA protocols enabled in each direction.
      // The final two bytes are a pre-calculated Fletcher-8 checksum.
      byte setBaud[] = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
        0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00,
        0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E
      };

      // UBX-CFG-RATE: measurement rate 200 ms = 5 Hz, time reference GPS (0x01).
      byte setRate5Hz[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00,
        0x01, 0x00, 0xDE, 0x6A
      };

      // V2.5-Evo - 2026-05-06 - FIX-GPS-1: dual-baud init.
      // Standard u-blox best practice. Handles both possible GPS startup states:
      //   (a) Factory default 9600 (first ever power-on)
      //   (b) Retained 115200 in battery-backed memory (after a prior successful init)
      // Without dual-baud, scenario (b) floods the GPS at wrong baud → 100+ frame errors
      // → u-blox firmware disables UART RX → GPS becomes unresponsive to all commands.
      // Confirmed bug 2026-05-06 via "$GNTXT ... UART RX was disabled" diagnostic message.
      //
      // Total wrong-baud bytes sent in either path: ~28. Well under the 100-frame-error
      // threshold that triggers the UART RX disable.

      // Step 1: open at 115200. If GPS was already at 115200 from a previous boot,
      // this command is delivered cleanly and confirms the config.
      Serial.println("TX GPS [BN-220]: dual-baud init, attempt 115200 first...");
      Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);
      delay(200);

      // Send UBX-CFG-PRT targeting 115200. If GPS is at 115200 → accepted.
      // If GPS is at 9600 → looks like garbage (~28 bytes), well under threshold.
      Serial1.write(setBaud, sizeof(setBaud));
      Serial1.flush();
      delay(100);

      // Step 2: close and reopen at 9600 to handle the factory-default case.
      Serial1.end();
      delay(100);
      Serial.println("TX GPS [BN-220]: dual-baud init, attempt 9600 fallback...");
      Serial1.begin(9600, SERIAL_8N1, P_U1_RX, P_U1_TX);
      delay(200);

      // Send same UBX-CFG-PRT at 9600. If GPS was at factory default 9600 → accepted,
      // GPS switches to 115200. If GPS is already at 115200 (from step 1) → garbage at
      // 115200 receiver, ~28 bytes, under threshold.
      Serial1.write(setBaud, sizeof(setBaud));
      Serial1.flush();
      delay(100);

      // Step 3: GPS should now be at 115200 regardless of starting state.
      // Reopen our side at 115200 to match.
      Serial1.end();
      delay(100);
      Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);
      Serial.println("TX GPS [BN-220]: now at 115200, sending rate config...");

      // Step 4: send the 5Hz measurement-rate command at the final baud.
      delay(100);
      Serial1.write(setRate5Hz, sizeof(setRate5Hz));
      Serial1.flush();
      delay(50);

      // Step 5: disable verbose NMEA sentences we don't parse to reduce buffer load.
      // We only need GPRMC (speed, lat/lon) and GPGGA (fix quality, HDOP).
      // UBX-CFG-MSG: B5 62 06 01 03 00 [NMEA class F0] [msg id] [rate=0] CK_A CK_B
      byte disableGPGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};  // satellite view
      byte disableGPGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11};  // lat/lon (dup of GPRMC)
      byte disableGPVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19};  // track/speed (dup)
      // V2.5-Evo - 2026-05-06 - DIAG: NMEA filter temporarily DISABLED to keep GSV
      // (satellites in view) visible during fix-acquisition troubleshooting. Re-enable
      // these three writes once GPS fix issue is resolved if buffer pressure becomes a concern.
      // Serial1.write(disableGPGSV, sizeof(disableGPGSV)); Serial1.flush();
      // Serial1.write(disableGPGLL, sizeof(disableGPGLL)); Serial1.flush();
      // Serial1.write(disableGPVTG, sizeof(disableGPVTG)); Serial1.flush();

      tx_gps_initialized = true;
      Serial.println("TX GPS [BN-220]: init complete (115200, 5Hz, GSV/GLL/VTG enabled for diagnostics)");
      break;
    }

    // --------------------------------------------------------
    // Type 2: M10 — boots at 115200 natively, 10Hz, all constellations
    // --------------------------------------------------------
    case 2:
    {
      // UBX-CFG-PRT: configure UART1 for 115200 baud, 8N1, UBX+NMEA both directions.
      // Same byte sequence as case 0 — valid for both NEO-M8 and M10.
      // Redefined here because case 0's setBaud is block-scoped and out of reach.
      byte setBaud[] = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
        0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00,
        0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E
      };

      // V2.5-Evo - 2026-05-06 - FIX-GPS-1: dual-baud init for M10.
      // M10 defaults to 115200 but battery-backed config may have saved a different rate.
      // Same dual-baud sequence as BN-220 — covers both the native-115200 and any saved-rate case.

      // Step 1: open at 115200. If M10 is already here, setBaud is accepted cleanly.
      Serial.println("TX GPS [M10]: dual-baud init, attempt 115200 first...");
      Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);
      delay(200);

      Serial1.write(setBaud, sizeof(setBaud));
      Serial1.flush();
      delay(100);

      // Step 2: close and reopen at 9600 to cover the factory/saved-9600 case.
      Serial1.end();
      delay(100);
      Serial.println("TX GPS [M10]: dual-baud init, attempt 9600 fallback...");
      Serial1.begin(9600, SERIAL_8N1, P_U1_RX, P_U1_TX);
      delay(200);

      Serial1.write(setBaud, sizeof(setBaud));
      Serial1.flush();
      delay(100);

      // Step 3: GPS is now at 115200 regardless of starting state.
      // Reopen our side to match.
      Serial1.end();
      delay(100);
      Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);
      Serial.println("TX GPS [M10]: now at 115200, sending rate/GNSS config...");

      // UBX-CFG-RATE: measurement period 100 ms = 10 Hz, GPS time reference.
      // Fletcher-8 checksum: CK_A=0x7A, CK_B=0x12 (pre-calculated).
      byte setRate10Hz[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00,
        0x01, 0x00, 0x7A, 0x12
      };
      delay(100);
      Serial1.write(setRate10Hz, sizeof(setRate10Hz));
      Serial1.flush();
      delay(50);

      // UBX-CFG-GNSS: enable GPS, Galileo, BDS (BeiDou), and GLONASS.
      // This gives the M10 all available constellations for best fix accuracy.
      // Payload: 4-byte header (msgver=0, numTrkChHw=0, numTrkChUse=0xFF, numConfigBlocks=4)
      // + 4 blocks of 8 bytes each (gnssId, resTrkCh, maxTrkCh, reserved, flags).
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

      // V2.5-Evo fix (I-1): Disable verbose NMEA sentences on M10 path — same filtering applied
      // on BN-220 path. At 10Hz the omission is worse: ~800 bytes/s of GPGSV/GPGLL/GPVTG
      // that TinyGPS++ never uses, needlessly filling the 512-byte RX buffer.
      // UBX-CFG-MSG: B5 62 06 01 03 00 [NMEA class F0] [msg id] [rate=0] CK_A CK_B
      byte disableGPGSV_m10[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};  // satellite view
      byte disableGPGLL_m10[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11};  // lat/lon (dup of GPRMC)
      byte disableGPVTG_m10[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19};  // track/speed (dup)
      // V2.5-Evo - 2026-05-06 - DIAG: NMEA filter temporarily DISABLED to keep GSV
      // (satellites in view) visible during fix-acquisition troubleshooting. Re-enable
      // these three writes once GPS fix issue is resolved if buffer pressure becomes a concern.
      // Serial1.write(disableGPGSV_m10, sizeof(disableGPGSV_m10)); Serial1.flush();
      // Serial1.write(disableGPGLL_m10, sizeof(disableGPGLL_m10)); Serial1.flush();
      // Serial1.write(disableGPVTG_m10, sizeof(disableGPVTG_m10)); Serial1.flush();

      tx_gps_initialized = true;
      Serial.println("TX GPS [M10]: init complete (115200, 10Hz, GPS+Galileo+BDS+GLONASS, GSV/GLL/VTG enabled for diagnostics)");
      break;
    }

    default:
      // gps_chip_type values 1 and 3 (compass variants) are rejected by
      // cfgValidateCrossField() on TX. Any other value is an unknown chip type.
      Serial.print("TX GPS: unknown gps_chip_type=");
      Serial.println(usrConf.gps_chip_type);
      Serial.println("TX GPS: init skipped — check web config");
      // tx_gps_initialized stays false; getTxGPSLoop() will safely do nothing.
      break;
  }
}

// ============================================================
// txGpsColdReset - Send UBX-CFG-RST cold-restart to GPS module
// ============================================================
//
// V2.5-Evo - 2026-05-06 - DIAG: send UBX-CFG-RST cold-restart clear-all to GPS.
// Used by ?gpscoldreset serial command. Forces the GPS module to discard all
// ephemeris/almanac/clock data and acquire from scratch. Useful when the chip
// appears stuck (e.g. PPS LED firing but NMEA empty due to stale cache).
// bbr_mask=0xFFFF (clear all BBR), reset_mode=0x02 (controlled GPS-subsystem
// software reset). Checksum CK_A=0x0E, CK_B=0x61 verified by Fletcher-8.
// (Note: prompt specified mode=0x01 but checksum 0x0E/0x61 is correct for 0x02.)
void txGpsColdReset() {
  if (!tx_gps_initialized) {
    Serial.println("TX GPS: not initialized; run ?gpsreinit first");
    return;
  }
  byte coldReset[] = {
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00,
    0xFF, 0xFF, 0x02, 0x00,
    0x0E, 0x61
  };
  Serial1.write(coldReset, sizeof(coldReset));
  Serial1.flush();
  Serial.println("TX GPS: cold-restart command sent. Wait 30-60s for fresh acquisition.");
}

// ============================================================
// getTxGPSLoop - Drain pending GPS bytes and update tx_gps_speed
// ============================================================
//
// What it does:
//   Reads every byte currently sitting in the Serial1 RX buffer,
//   feeds each one to the TinyGPS++ parser (gps_tx), and — if
//   we have a valid and fresh fix — publishes the current speed
//   to the global tx_gps_speed in the unit chosen by
//   usrConf.speed_src.
//
//   If there is no fix, or the fix is older than the user's
//   configured stale-timeout, tx_gps_speed is set to the
//   sentinel 0xFF so the display helpers render "--" (matching
//   the existing convention used for telemetry.foil_speed).
//
// Inputs (all from usrConf, SPIFFS-backed):
//   - usrConf.speed_src              : 2=km/h, 3=knots, 5=mph
//   - usrConf.tx_gps_stale_timeout_ms: max allowed fix age (ms)
//
// Outputs:
//   Writes the global volatile uint8_t tx_gps_speed.
//
// Side effects:
//   - Consumes bytes from Serial1 (the RX buffer).
//   - Does NOT call delay(), does NOT block, does NOT yield.
//     It simply drains whatever is already there and returns.
//     Intended to be called once per main-loop iteration.
//
// Safety:
//   - Returns immediately if initTxGPS() never ran, so we never
//     touch an uninitialized UART.
//   - The caller (loop() in V2_Integration_Tx.ino) additionally
//     gates this call on usrConf.gps_en and on speed_src being
//     one of the TX-GPS options, so we don't waste cycles when
//     a TX-GPS display unit isn't selected.
// ============================================================
void getTxGPSLoop()
{
  // Guard: do nothing until initTxGPS() has set up Serial1.
  if (!tx_gps_initialized) return;

  // Pull every available byte into the parser. Non-blocking:
  // Serial1.available() only reports bytes already in the
  // buffer; we never wait for new ones.
  while (Serial1.available())
  {
    gps_tx.encode((char)Serial1.read());
  }

  // Decide whether the data is trustworthy enough to publish.
  // A fix is considered "current" when all three hold:
  //   1) TinyGPS++ has ever seen a valid location (isValid())
  //   2) TinyGPS++ has ever seen a valid speed reading
  //   3) the last location update is newer than the user's
  //      configured stale timeout (usrConf.tx_gps_stale_timeout_ms)
  const bool have_current_fix =
         gps_tx.location.isValid()
      && gps_tx.speed.isValid()
      && gps_tx.location.age() < usrConf.tx_gps_stale_timeout_ms;

  if (!have_current_fix)
  {
    tx_gps_speed = 0xFF;   // "no data" sentinel → display renders "--"
    return;
  }

  // V2.5-Evo - 2026-04-22 - HDOP quality gate (N-3 fix): reject fixes with poor
  // satellite geometry before publishing speed. Both usrConf.gps_max_hdop
  // and gps_tx.hdop.value() are stored as HDOP*100, so no float math is
  // needed — direct uint16 comparison. A value of 0 skips the check (out
  // of range 50-500, so it can only appear if config validation was bypassed).
  if (usrConf.gps_max_hdop > 0 &&
      (!gps_tx.hdop.isValid() || gps_tx.hdop.value() > usrConf.gps_max_hdop))
  {
    tx_gps_speed = 0xFF;   // poor signal quality → display renders "--"
    return;
  }

  // Convert the current speed into the unit the user selected
  // in the web config. TinyGPS++ provides the conversion helpers
  // natively so we don't carry any magic numbers here.
  float speed_val = 0.0f;
  switch (usrConf.speed_src)
  {
    case 2: speed_val = gps_tx.speed.kmph();  break;   // km/h
    case 3: speed_val = gps_tx.speed.knots(); break;   // knots
    case 5: speed_val = gps_tx.speed.mph();   break;   // mph
    default:
      // speed_src was flipped to a non-TX-GPS option between
      // the caller's gate and this line. Defensive: publish
      // the sentinel rather than a stale value.
      tx_gps_speed = 0xFF;
      return;
  }

  // Clamp into a byte. We reserve 0xFF (255) as the sentinel,
  // so the max displayable value is 254. No realistic foil,
  // buggy, or hand-held speed approaches 254 in any unit.
  if (speed_val < 0.0f)   speed_val = 0.0f;
  if (speed_val > 254.0f) speed_val = 254.0f;

  tx_gps_speed = (uint8_t)speed_val;
}
