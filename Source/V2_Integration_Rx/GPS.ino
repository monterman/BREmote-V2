// V2.5-Evo - 2026-07-25 - STAGE 1 (GPS repair, invert the UART parking): the UART mux now RESTS on GPS instead of on the VESC. getGPSLoop() no longer switches the mux at all — it is a pure drain, because the line is already pointed at the GPS when it is called; the 10 ms vTaskDelay it used to wait after switching is DELETED (it was the settle for a Serial1.end()/begin() cycle that has been commented out three lines below it since SW55 — the 74HC4052 analog mux settles in under 3 us and setUartMux() is synchronous, so the correct post-switch delay is zero). configureGPS() now ends on setUartMux(1) so the board BOOTS parked on GPS. WHY: the old scheme parked on VESC and gave GPS a ~10 ms listening window every 500 ms against a module that bursts ~500 B every 200 ms, so ~73% of windows landed entirely in the silence between bursts — measured on the owner's board at 7 GPS bytes/s and 0.1 sentences/s against an expected ~2500 B/s and ~28 sentences/s. That starved GPS course-over-ground, which is what made Follow-Me fall back to the EMI-biased compass and steer the wrong way. Mux switches DROP from 6/s to 4/s (only getVescLoop() switches now). setUartMux() itself, the VESC receive timeout and the 20 ms VESC settle are all UNTOUCHED. No confStruct change, sizeof stays 184, SW_VERSION stays 34, SPIFFS config is NOT reset by this flash.
// V2.5-Evo - 2026-07-25 - STAGE 1: the Serial1 RX ring is now sized in Init.ino BEFORE the first Serial1.begin() and raised 512 -> 2048. The setRxBufferSize(512) that used to live in configureGPS() ran AFTER Init.ino had already begun the port, and arduino-esp32 REFUSES a resize once the driver is installed (logs an error, returns 0) — so the real ring has always been the 256 B default. Harmless while nothing was being captured; a guaranteed data-loss bug the moment GPS actually flows.
// V2.5-Evo - 2026-07-25 - STAGE 0 (instrumentation only): getGPSLoop() now counts GPS bytes and complete NMEA sentences, tracks when the COG VALUE actually changes (separately from gps_last_course_ms, which only tracks the TIMESTAMP and therefore stayed "fresh" straight through a frozen-heading failure), and samples the RX fix age once per poll. gps_last_course_deg / gps_last_course_ms and every heading decision are untouched — the new state is written alongside them and read only by the level-4 log record and ?diag.
// V2.5-Evo - 2026-05-14 - SW55: setUartMux(0) at end of getGPSLoop() and configureGPS() — GPS resets MUX to VESC on exit; GPS always has priority
// V2.5-Evo - 2026-05-06 - D1: Capture GPS course-over-ground (gps_last_course_deg/ms) for future RTM heading source
// V2.5-Evo - 2026-04-30 - Rename: gps_max_jump_kmh → gps_max_teleport_kmh (clarity)
// V2.5-Evo - 2026-04-30 - Bundle E: replaced 300ms blocking serial drain with non-blocking while(available()) drain
// V2.5-Evo - 2026-04-24 - Added Phase B FIELD SERVICE NOTE (sizeof confStruct 128→136)
// V2.5-Evo - 2026-04-22 - Added gps_chip_type branch: type 0/1=BN-220/BN-880 (9600→115200, 5Hz), type 2/3=M10 (115200 direct, 10Hz, all constellations)
// V2.5-Evo - 2026-04-22 - Added Phase A GPS anti-spoofing: HDOP check, teleport check, acceleration check (gpsPhaseACheck)

// ============================================================
// FIELD SERVICE NOTE
//
// V2.5-Evo - 2026-04-22: sizeof(confStruct) changed from 108 to 112
// bytes (gps_chip_type added). On the first V2.5-Evo flash, SPIFFS
// will detect the size mismatch and reset ALL settings to
// defaults. After flashing, you must:
//   1) Re-pair TX and RX
//   2) Re-configure all settings via the web UI
//   3) Re-calibrate compass via the 'runcal' serial command
//
// V2.5-Evo - 2026-04-22: sizeof(confStruct) changed from 112 to 128
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
// V2.5-Evo - 2026-04-24: sizeof(confStruct) changed from 128 to 136
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
//   - Calls setUartMux(1) to switch UART mux to GPS path, and LEAVES it there
//     (V2.5-Evo 2026-07-25 STAGE 1: GPS is now the mux's resting position).
//   - Calls Serial1.begin()/end()/begin() for BN-220/BN-880.
//   - Blocks ~450 ms for BN-220/BN-880; ~250 ms for M10.
// ============================================================
// ============================================================
// ubxSendAcked - send a UBX config message and CONFIRM the module accepted it
//
// V2.5-Evo - 2026-07-28 - GPS-ACK-1.
//
// WHAT WAS WRONG: every UBX write in this firmware was fire-and-forget. Nothing read the
// ACK, so a dropped or rejected frame was indistinguishable from a successful one. That is
// not theoretical — on 2026-07-28 the newly-added ?gpscfg readback showed the RX still in
// dynModel 0 (Portable) after a flash that "sent" dynModel 5 (Sea). The GSV/GLL/VTG filters
// in the SAME block had landed. The NAV5 frame is 44 bytes and was being fired immediately
// after CFG-RATE with no gap; CFG-RATE retimes the module's whole output engine, and the
// larger frame arriving mid-reconfiguration was simply dropped.
//
// WHY NOT JUST ADD A DELAY: a delay would have fixed this one write and left every other
// config write in the firmware — rate, GNSS, the NMEA filters, and anything added later —
// still unverified. The defect is not "NAV5 needs 20 ms", it is "we configure blind".
//
// WHAT THIS DOES: u-blox answers every CFG message with UBX-ACK-ACK (0x05 0x01) or
// UBX-ACK-NAK (0x05 0x00), carrying the class and id of the message being answered. This
// sends, waits for that ACK, and retries on NAK or timeout. Ordering and inter-frame timing
// stop mattering because a dropped frame is simply re-sent until confirmed.
//
// Returns true if the module ACKed. Bounded: tries x (300 ms + 50 ms) worst case.
// ============================================================
// ============================================================
// UBX helpers — RX port of the TX GPS work of 2026-07-29/30.
//
// PORTED, NOT COPIED. Three RX-specific constraints shaped this:
//
//   1. THE RX HAS A WATCHDOG AND THE TX DOES NOT. Init.ino arms a 3000 ms task WDT with
//      trigger_panic = true and subscribes four tasks including the loop task. Every wait
//      loop below therefore calls esp_task_wdt_reset(), following the convention already used
//      throughout this firmware (Compass.ino, Logger.ino, System.ino). A verbatim copy of the
//      TX code would panic-reboot the buggy in the middle of a GPS command.
//      configureGPS() itself is safe unfed: setup() runs runBootSequence() -> configureGPS()
//      BEFORE initWatchdog(), so nothing is watching yet. The feeds matter for the serial
//      commands, which run in the watched loop task.
//   2. THE UART IS MUXED. setUartMux(1) points Serial1 at the GPS; getVescLoop() borrows it.
//      Since STAGE 1 the mux RESTS on GPS, so anything here asserts channel 1 and leaves it.
//   3. THE RX RING IS SIZED IN Init.ino, deliberately, before the first begin(). Nothing here
//      may call setRxBufferSize() — the core refuses it once the driver is installed. The
//      2048-byte value survives end()/begin(), which is what makes gpsOpenAt() safe.
// ============================================================

// Outcome of a UBX config write. NAK is distinguished from silence on purpose: NAK is an
// answer (the module understood and refused), silence is not.
static const uint8_t UBX_ACK     = 0;
static const uint8_t UBX_NAK     = 1;
static const uint8_t UBX_NOREPLY = 2;

// Which configuration dialect the module speaks.
static const uint8_t UBX_DIALECT_LEGACY = 0;
static const uint8_t UBX_DIALECT_VALSET = 1;
static const uint8_t UBX_DIALECT_MUTE   = 2;

static const uint8_t GPS_SAW_NMEA = 0x02;

// Set true by initWatchdog() once the loop task is actually SUBSCRIBED to the task WDT.
// esp_task_wdt_reset() logs an error on every call from an unregistered task, and
// configureGPS() runs BEFORE initWatchdog(), so the feeds below must be gated on this.
volatile bool g_wdt_active = false;

// Feed the watchdog, but only when there is one watching us. Used in every wait loop here.
static inline void gpsFeedWdt()
{
  if (g_wdt_active) esp_task_wdt_reset();
}

// Configuration-interface key IDs (u-blox M9/M10), verified against the M10 SPG 5.10
// interface description §4.9. Needed because M10 REMOVED the legacy CFG messages: its whole
// UBX-CFG class is CFG-CFG, CFG-RST, CFG-VALDEL, CFG-VALGET, CFG-VALSET. An M100 Pro or
// M100-5883 fitted to this RX would otherwise reject every write and sit in Portable.
static const uint32_t KEY_NAVSPG_DYNMODEL    = 0x20110021UL;  // E1, Table 23: SEA = 5
static const uint32_t KEY_RATE_MEAS          = 0x30210001UL;  // U2, milliseconds
static const uint32_t KEY_MSGOUT_NMEA_GLL_U1 = 0x209100caUL;  // U1, rate on UART1
static const uint32_t KEY_MSGOUT_NMEA_GSV_U1 = 0x209100c5UL;  // U1
static const uint32_t KEY_MSGOUT_NMEA_VTG_U1 = 0x209100b1UL;  // U1
static const uint32_t KEY_UART1_BAUDRATE     = 0x40520001UL;  // U4

static const uint32_t kGpsBauds[]   = { 115200, 38400, 9600, 57600, 19200 };
static const uint8_t  kGpsBaudCount = sizeof(kGpsBauds) / sizeof(kGpsBauds[0]);
static const uint32_t GPS_BAUD_PREFERRED = 115200;

static uint32_t gps_current_baud = 115200;

// How long to wait for an ACK, scaled to link speed. A fixed 300 ms silently assumed 115200;
// at 9600 the reply queues behind a GSV burst (~280 B ≈ 292 ms) and the wait became a coin
// flip. Observed on the TX as an intermittent rate-write failure.
static uint16_t gpsAckWindowMs()
{
  uint32_t baud = gps_current_baud ? gps_current_baud : 115200UL;
  uint32_t w    = 150UL + (5000000UL / baud);
  if (w < 300)  w = 300;
  if (w > 1200) w = 1200;
  return (uint16_t)w;
}

// Reopen Serial1 at a known baud. Keeps gps_current_baud honest and re-asserts the mux.
static void gpsOpenAt(uint32_t baud)
{
  setUartMux(1);
  Serial1.end();
  delay(10);
  Serial1.begin(baud, SERIAL_8N1, P_U1_RX, P_U1_TX);   // ring size persists from Init.ino
  gps_current_baud = baud;
  delay(60);
  while (Serial1.available()) Serial1.read();
}

// LISTEN at this baud. TRANSMITS NOTHING — see the TX's GPS-BAUD-4. Sending at an unconfirmed
// baud produces framing errors, and ~100 of those make u-blox disable its receiver until the
// module loses power. That happened on the TX bench on 2026-07-30. A u-blox module streams
// NMEA from power-on, so hearing "$G.." is proof enough and costs nothing.
static uint8_t gpsProbeAt(uint32_t baud, uint16_t window_ms)
{
  gpsOpenAt(baud);

  uint8_t  seen = 0;
  byte     prev = 0;
  uint32_t deadline = millis() + window_ms;

  while ((int32_t)(millis() - deadline) < 0)
  {
    gpsFeedWdt();
    if (!Serial1.available()) { delay(1); continue; }
    byte c = Serial1.read();
    if (prev == '$' && c == 'G') { seen |= GPS_SAW_NMEA; break; }
    prev = c;
  }
  return seen;
}

// Hunt the candidate list. Always leaves Serial1 at a known speed — the answering baud, or
// fallback_baud. Never strands the port on whatever was tried last.
static uint32_t gpsDetectBaud(uint16_t window_ms, uint32_t fallback_baud)
{
  for (uint8_t i = 0; i < kGpsBaudCount; i++)
    if (gpsProbeAt(kGpsBauds[i], window_ms)) return kGpsBauds[i];

  gpsOpenAt(fallback_baud);
  return 0;
}

// Fletcher-8 over class..payload, into the last two bytes. Needed for runtime-built frames.
static void ubxAppendChecksum(byte *frame, size_t frameLen)
{
  byte ckA = 0, ckB = 0;
  for (size_t i = 2; i < frameLen - 2; i++) { ckA += frame[i]; ckB += ckA; }
  frame[frameLen - 2] = ckA;
  frame[frameLen - 1] = ckB;
}

static uint8_t ubxSendAckedT(const byte *msg, size_t len, uint8_t tries);

// Set ONE config item via UBX-CFG-VALSET (0x06 0x8A) — the modern interface M9/M10 require.
// Layers RAM|BBR normally; persist adds Flash, and only ?gpssetup asks for that.
static uint8_t ubxValset(uint32_t key, const byte *val, uint8_t valLen, uint8_t tries,
                         bool persist = false)
{
  if (valLen == 0 || valLen > 8) return UBX_NOREPLY;

  byte f[6 + 4 + 4 + 8 + 2];
  const uint16_t payload = 4 + 4 + valLen;
  size_t n = 0;

  f[n++] = 0xB5; f[n++] = 0x62; f[n++] = 0x06; f[n++] = 0x8A;
  f[n++] = (byte)(payload & 0xFF); f[n++] = (byte)(payload >> 8);
  f[n++] = 0x00;
  f[n++] = persist ? (0x01 | 0x02 | 0x04) : (0x01 | 0x02);
  f[n++] = 0x00; f[n++] = 0x00;
  f[n++] = (byte)( key        & 0xFF);
  f[n++] = (byte)((key >>  8) & 0xFF);
  f[n++] = (byte)((key >> 16) & 0xFF);
  f[n++] = (byte)((key >> 24) & 0xFF);
  for (uint8_t i = 0; i < valLen; i++) f[n++] = val[i];
  n += 2;

  ubxAppendChecksum(f, n);
  return ubxSendAckedT(f, n, tries);
}

// Apply one setting in whichever dialect the module speaks. Returns a short status string.
// Never enforces: a module that cannot be configured still navigates on its defaults, and the
// outcome is REPORTED rather than made fatal.
static const char *gpsApplyCfg(uint8_t dialect,
                               const byte *legacy, size_t legacyLen,
                               uint32_t key, const byte *val, uint8_t valLen,
                               bool persist = false)
{
  if (dialect == UBX_DIALECT_VALSET)
    return (ubxValset(key, val, valLen, 2, persist) == UBX_ACK) ? "OK/valset" : "REJECTED";

  if (dialect == UBX_DIALECT_MUTE) {
    Serial1.write(legacy, legacyLen);
    Serial1.flush();
    delay(20);
    return "no-ACK";
  }

  uint8_t r = ubxSendAckedT(legacy, legacyLen, 2);
  if (r == UBX_ACK) return "OK";
  if (r == UBX_NAK)
    return (ubxValset(key, val, valLen, 2, persist) == UBX_ACK) ? "OK/valset" : "REJECTED";
  return "no-ACK";
}

static bool ubxSendAcked(const byte *msg, size_t len, uint8_t tries)
{
  const byte wantCls = msg[2];   // class of the message we are sending
  const byte wantId  = msg[3];   // id    of the message we are sending

  for (uint8_t t = 0; t < tries; t++)
  {
    while (Serial1.available()) Serial1.read();   // clear stale NMEA so the parser starts clean
    Serial1.write(msg, len);
    Serial1.flush();

    uint32_t deadline = millis() + 300;
    uint8_t  state = 0;
    byte     cls = 0, id = 0, ackCls = 0, ackId = 0;
    uint16_t len_ = 0, idx = 0;

    while ((int32_t)(millis() - deadline) < 0)
    {
      if (!Serial1.available()) { delay(1); continue; }
      byte c = Serial1.read();

      switch (state)
      {
        case 0: state = (c == 0xB5) ? 1 : 0; break;
        case 1: state = (c == 0x62) ? 2 : 0; break;
        case 2: cls = c; state = 3; break;
        case 3: id  = c; state = 4; break;
        case 4: len_ = c; state = 5; break;
        case 5:
          len_ |= ((uint16_t)c << 8);
          idx = 0;
          // ACK-ACK = 0x05/0x01, ACK-NAK = 0x05/0x00, payload is always 2 bytes.
          state = (cls == 0x05 && len_ == 2) ? 6 : 0;
          break;
        case 6:
          if (idx == 0) { ackCls = c; idx = 1; }
          else
          {
            ackId = c;
            // Only trust an ACK that names the message we actually sent — the module may
            // be ACKing something else queued ahead of us.
            if (ackCls == wantCls && ackId == wantId)
            {
              if (id == 0x01) return true;    // ACK-ACK
              break;                          // ACK-NAK: stop reading, fall through to retry
            }
            state = 0;                        // an ACK for a different message; keep looking
          }
          break;
      }
      if (state == 6 && idx == 1 && ackCls == wantCls && ackId == wantId && id == 0x00) break;
    }
    delay(50);   // brief settle before re-sending
  }
  return false;
}

// ------------------------------------------------------------
// ubxSendAckedT - the tri-state successor to ubxSendAcked() above.
//
// V2.5-Evo - 2026-07-30 - RX port. Three differences from the original, each earned on the TX:
//
//   1. Returns UBX_ACK / UBX_NAK / UBX_NOREPLY instead of a bool. The distinction is what
//      makes M9/M10 support possible: a NAK means "I do not speak this dialect", which is the
//      signal to re-send the same setting through CFG-VALSET. The bool version could not tell
//      "refused" from "silent" and so could not fall back.
//   2. RETRIES ONLY ON SILENCE, never on NAK. A NAK is a definite answer; re-sending an
//      identical frame can only earn an identical refusal. The original retried it, which on
//      an M10 — where five separate messages are all refused — wasted seconds of boot.
//   3. The deadline SCALES WITH BAUD and every wait feeds the watchdog.
//
// The original ubxSendAcked() is left in place and untouched: it is what shipped in f7fbcbd
// and is still referenced by the legacy call sites below until they are migrated.
// ------------------------------------------------------------
static uint8_t ubxSendAckedT(const byte *msg, size_t len, uint8_t tries)
{
  const byte wantCls = msg[2];
  const byte wantId  = msg[3];

  for (uint8_t t = 0; t < tries; t++)
  {
    setUartMux(1);
    while (Serial1.available()) Serial1.read();
    Serial1.write(msg, len);
    Serial1.flush();

    uint32_t deadline = millis() + gpsAckWindowMs();
    uint8_t  state = 0;
    byte     cls = 0, id = 0, ackCls = 0;
    uint16_t plen = 0, idx = 0;

    while ((int32_t)(millis() - deadline) < 0)
    {
      gpsFeedWdt();
      if (!Serial1.available()) { delay(1); continue; }
      byte c = Serial1.read();

      switch (state)
      {
        case 0: state = (c == 0xB5) ? 1 : 0; break;
        case 1: state = (c == 0x62) ? 2 : 0; break;
        case 2: cls = c; state = 3; break;
        case 3: id  = c; state = 4; break;
        case 4: plen = c; state = 5; break;
        case 5:
          plen |= ((uint16_t)c << 8);
          idx = 0;
          state = (cls == 0x05 && plen == 2) ? 6 : 0;   // ACK-ACK 0x05/0x01, ACK-NAK 0x05/0x00
          break;
        case 6:
          if (idx == 0) { ackCls = c; idx = 1; }
          else
          {
            // Only trust an ACK naming the message we actually sent.
            if (ackCls == wantCls && c == wantId)
              return (id == 0x01) ? UBX_ACK : UBX_NAK;
            state = 0;
          }
          break;
      }
    }
    delay(50);
  }
  return UBX_NOREPLY;
}

// Commit the module's current settings to its own non-volatile memory. UBX-CFG-CFG (0x06 0x09)
// is one of the five messages M10 kept, so one frame covers both generations. saveMask 0xFFFF
// = every section; deviceMask 0x17 = BBR + Flash + EEPROM + SPI. An ACK means "saved to what I
// have" — many BN-880 boards carry only battery-backed RAM, not flash.
static bool gpsSaveConfig()
{
  byte f[21] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x17,
    0x00, 0x00
  };
  ubxAppendChecksum(f, sizeof(f));
  return ubxSendAckedT(f, sizeof(f), 3) == UBX_ACK;
}

void configureGPS() {
  // Route UART1 to the GPS connector (MUX position 1).
  // This must happen before any Serial1 traffic regardless of chip type.
  setUartMux(1);

  // V2.5-Evo - 2026-07-25 - STAGE 1: the Serial1 RX ring buffer is NO LONGER sized here.
  //
  // WHAT THE BUG WAS: this function used to call Serial1.setRxBufferSize(512) on this line,
  // but runBootSequence() (Init.ino) has already called Serial1.begin() by the time we get
  // here. arduino-esp32 rejects a resize once the UART driver is installed — it logs
  // "RX Buffer can't be resized when Serial is already running" and returns 0 — so the ring
  // has silently stayed at the 256-byte default ever since. 256 bytes is only ~22 ms of
  // airtime at 115200 baud, and checkWetness() blocks loop() for ~300 ms every 10 s, during
  // which ~750 bytes arrive: guaranteed silent loss of whole NMEA sentences.
  //
  // WHAT THE FIX DOES: the size is now set in Init.ino immediately BEFORE the first
  // Serial1.begin(), and raised to 2048 bytes (~178 ms of airtime, enough to ride out the
  // wetness stall). The value survives the Serial1.end()/begin() cycle used by the
  // BN-220/BN-880 baud switch below, because the core keeps _rxBufferSize across end()
  // and passes it to every subsequent begin().

  // V2.5-Evo - 2026-04-22 - Branch on GPS chip type. Each chip type has a
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

      // ============================================================
      // V2.5-Evo - 2026-07-31 - RX GPS-BAUD-2: LISTEN BEFORE DANCING.
      //
      // The dance below is now the FALLBACK, not the opening move. It is effective, but it
      // always transmits ~28 bytes at a baud the module is NOT using, and u-blox counts
      // framing errors: past roughly 100 it DISABLES its receiver until the module loses
      // power. 28 per boot looks safe in isolation — but the counter does NOT reset when the
      // ESP32 reboots, only when the GPS itself is depowered. A bench session of reflashes and
      // ?reboot cycles therefore ACCUMULATES, and on 2026-07-30 that is exactly what disabled
      // the TX's GPS receiver mid-session: NMEA still streaming, every UBX write ignored.
      //
      // Listening costs nothing and skips the dance entirely whenever the module is already
      // talking — which is every boot after the first. If it is heard below 115200, CFG-PRT is
      // sent AT THE MODULE'S OWN BAUD, so it is parsed correctly and produces ZERO framing
      // errors instead of being sprayed blind at a guess.
      //
      // This matters more here than on the TX: the RX GPS feeds RTM and Follow-Me, the modes
      // that drive the buggy.
      // ============================================================
      {
        uint32_t seen = gpsDetectBaud(1100, 115200);
        if (seen) {
          Serial.printf("GPS [BN-220/880]: heard the module at %lu — skipping the dance\n",
                        (unsigned long)seen);
          if (seen != GPS_BAUD_PREFERRED) {
            Serial.printf("GPS [BN-220/880]: moving %lu -> %lu ... ",
                          (unsigned long)seen, (unsigned long)GPS_BAUD_PREFERRED);
            // CFG-PRT at the module's ACTUAL baud, so it is understood rather than guessed at.
            Serial1.write(setBaud, sizeof(setBaud));
            Serial1.flush();
            delay(100);
            gpsOpenAt(GPS_BAUD_PREFERRED);
            Serial.println(gpsProbeAt(GPS_BAUD_PREFERRED, 1100) ? "OK" : "failed, rescanning");
          }
          // The rate write lives in the dance we just skipped, so send it here. Blind, but at
          // a baud we have CONFIRMED by listening — which is the whole distinction that makes
          // it safe. dynModel and the NMEA filter are handled after the switch.
          delay(100);
          Serial1.write(setRate5Hz, sizeof(setRate5Hz));
          Serial1.flush();
          delay(50);
          Serial.println("GPS [BN-220/880]: Config complete (115200, 5Hz)");
          break;
        }
        Serial.println("GPS [BN-220/880]: nothing heard — falling back to the dual-baud dance");
      }

      // Step 1: open at factory default baud so the GPS hears the baud command.
      Serial.println("GPS [BN-220/880]: Connecting at 9600...");
      Serial1.begin(9600, SERIAL_8N1, P_U1_RX, P_U1_TX);
      gps_current_baud = 9600;
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

  // ============================================================
  // V2.5-Evo - 2026-07-27 - GPS-CFG-1: DYNAMIC MODEL + NMEA SENTENCE FILTER
  //
  // Applied to EVERY chip type (including the default fallback) because both changes are
  // chip-agnostic: the BN-220 and BN-880 are u-blox M8, the M10 accepts the same legacy
  // UBX-CFG-* messages, and this is placed after the switch so no path can miss it.
  // Serial1 is at 115200 in all branches by the time we get here.
  //
  // --- WHY dynModel ---
  // Every one of these modules ships in dynModel 0 (Portable), which tells the receiver's
  // own Kalman filter to accept solutions up to 310 m/s horizontal and 50 m/s VERTICAL,
  // because a "portable" device might be in an aircraft. Nothing here ever leaves the
  // surface of a lake. The cost of that permissiveness is not theoretical: the foilIQ
  // logger (same u-blox family, sitting on the same buggy) recorded bogus 254 km/h speeds
  // and 4800 m altitudes emitted as HIGH-CONFIDENCE fixes — 5-7 satellites, HDOP < 3 — on
  // 2026-07-24, one day before the tow-buggy session where FM veered and RTM would not arm.
  //
  // dynModel 5 (Sea) constrains the filter to ~25 m/s horizontal and ~0 m/s vertical and
  // pins altitude near the surface. That kills those solutions AT THE SOURCE rather than
  // relying on gpsPhaseACheck() to catch them downstream — which matters because Phase A
  // does not merely discard a bad reading, it increments gps_suspect_count, and at
  // gps_suspect_threshold it sets gps_rejected, which BLOCKS RTM ARMING. A module inventing
  // occasional 254 km/h fixes is therefore not just noise; it is a plausible mechanism for
  // RTM refusing to arm in the field.
  //
  // Removing the vertical degree of freedom also improves the horizontal solution, and
  // course-over-ground is derived from the horizontal velocity solution — which is exactly
  // what runFmLoop() steers on.
  //
  // ⚠️ ALTITUDE CEILING: dynModel 5 is valid to 500 m. The Great Lakes sit at 75-183 m
  // (Michigan 176 m), so there is ~3x margin. If this buggy is ever run on a mountain lake
  // above 500 m, this MUST change to dynModel 4 (Automotive) or fixes will degrade.
  // Owner confirmed 2026-07-27 that it will not be: hard-coded deliberately, not configurable.
  //
  // Payload is the u-blox DEFAULT field set with only dynModel changed, copied verbatim from
  // the foilIQ WaveShare firmware where it is field-proven. mask=0x0001 means only dynModel
  // is applied, but carrying real defaults rather than zeros is the safer form. Checksum
  // 0x86/0x51 independently recomputed and verified 2026-07-27.
  // ============================================================
  static const byte setNav5Sea[] = {
    0xB5,0x62,0x06,0x24,0x24,0x00,0x01,0x00,0x05,0x03,
    0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,
    0xFA,0x00,0xFA,0x00,0x64,0x00,0x5E,0x01,0x00,0x3C,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x86,0x51
  };
  // ============================================================
  // V2.5-Evo - 2026-07-30 - RX GPS-BAUD-1: LISTEN, THEN DECIDE THE DIALECT.
  //
  // Two things this replaces, both proven on the TX the same day:
  //
  //   1. TRANSMITTING AT AN UNCONFIRMED BAUD. The chip branches above end with a blind
  //      dual-baud dance and then this block used to fire a 44-byte NAV5 write regardless.
  //      If the module is not where we assume, every one of those bytes is a framing error,
  //      and ~100 of them make u-blox DISABLE ITS RECEIVER until the module loses power. That
  //      bricked the TX's GPS mid-session on 2026-07-30: NMEA still streaming, every write
  //      ignored. So nothing is sent until NMEA has actually been HEARD.
  //
  //   2. ASSUMING THE LEGACY DIALECT. u-blox M10 REMOVED the legacy CFG messages, so an
  //      M100 Pro or M100-5883 — the natural BN-880 upgrade, and already proven on the
  //      owner's drones — would reject every write here and sit in dynModel 0 (Portable).
  //      On the RX that feeds RTM and Follow-Me, i.e. the modes that drive the buggy. A NAK
  //      now means "wrong dialect" and the same setting is re-sent via CFG-VALSET.
  //
  // dynModel doubles as the dialect probe: it is the one setting that must not be missed, so
  // its answer is the most useful thing to branch on.
  // ============================================================
  bool     baud_confirmed = false;
  uint32_t heard = gpsDetectBaud(1100, GPS_BAUD_PREFERRED);
  if (heard) {
    Serial.printf("GPS: heard the module at %lu baud\n", (unsigned long)heard);
    baud_confirmed = true;
  }

  if (!baud_confirmed) {
    Serial.println("GPS: !! no NMEA at any baud — module not detected.");
    Serial.println("GPS: !! NOTHING was sent. Transmitting at an unconfirmed baud is what");
    Serial.println("GPS: !! disables a u-blox receiver, so silence is the safe response.");
    Serial.println("GPS: !! Check wiring and 3.3V, then run ?gpsbaud.");
    setUartMux(1);
    return;
  }

  uint8_t     dialect;
  const char *nav5_status;

  // ============================================================
  // V2.5-Evo - 2026-07-31 - RX GPS-ACK-2: SETTLE, THEN ASK TWICE.
  //
  // Observed on hardware: a boot immediately after a reset reported
  //   dynModel=Sea no-ACK | GSV no-ACK | GLL no-ACK | VTG no-ACK
  // while ?gpscfg moments later read back dynModel : 5 (Sea) and ?gpsbaud reported UBX alive.
  // So the module was healthy and correctly configured the whole time — it simply was not
  // ready to ANSWER yet when we asked.
  //
  // The cause is that listening succeeds too early to be a good readiness signal. A u-blox
  // module starts streaming NMEA within milliseconds of power-up, long before its UBX command
  // handler is serving requests, and gpsProbeAt() returns on the FIRST sentence — so hearing
  // NMEA proves the baud, not that the module is listening back.
  //
  // Two changes, because either alone is a guess: settle first, then ask a second time if the
  // first round hears nothing. Cheap in the healthy case (the first probe answers and neither
  // path is taken) and it removes a false "Portable!" warning that would otherwise send
  // someone chasing a fault that does not exist.
  // ============================================================
  delay(300);
  uint8_t probe = ubxSendAckedT(setNav5Sea, sizeof(setNav5Sea), 3);

  if (probe == UBX_NOREPLY) {
    Serial.println("GPS: no answer on the first round — settling and asking again...");
    delay(700);
    probe = ubxSendAckedT(setNav5Sea, sizeof(setNav5Sea), 3);
  }

  if (probe == UBX_ACK) {
    dialect     = UBX_DIALECT_LEGACY;
    nav5_status = "OK";
  } else if (probe == UBX_NAK) {
    dialect = UBX_DIALECT_VALSET;                  // M9/M10
    const byte sea = 5;                            // M10 SPG 5.10 Table 23: SEA = 5
    nav5_status = (ubxValset(KEY_NAVSPG_DYNMODEL, &sea, 1, 3) == UBX_ACK)
                  ? "OK/valset" : "REJECTED";
  } else {
    dialect     = UBX_DIALECT_MUTE;                // heard NMEA but no ACK — write blind, at
    Serial1.write(setNav5Sea, sizeof(setNav5Sea)); // a baud we DID confirm, so this is safe
    Serial1.flush();
    delay(20);
    nav5_status = "no-ACK";
  }
  bool nav5_ok = (strcmp(nav5_status, "OK") == 0 || strcmp(nav5_status, "OK/valset") == 0);

  // ============================================================
  // --- WHY the NMEA filter ---
  // TinyGPS++ reads GGA and RMC. It does not parse GSV, GLL or VTG at all — those sentences
  // are clocked across the wire, buffered, parsed to a dead end and discarded. GSV is the
  // worst: with multiple constellations it is 6-10 sentences per epoch on its own.
  //
  // On THIS board that waste is not free. Serial1 is time-shared with the VESC through the
  // 74HC4052 mux, and the GPS competes for both the line and the 2048-byte ring buffer —
  // the exact resource whose scarcity caused the STAGE 1 starvation above.
  //
  // This is not a speculative optimisation. The TX has shipped this identical filter since
  // 2026-06-05 (V2_Integration_Tx/GPS.ino, "L-2"), where it is recorded as resolving
  // "Audit #5 (GPS chatter choking the link)". The RX simply never received the same fix.
  //
  // GSA is deliberately left ENABLED to match the TX's proven configuration exactly rather
  // than diverge; it is one sentence per epoch and is available as further headroom if the
  // ?diag byte counters show it is worth trimming.
  //
  // Verify the effect with ?diag: g_diag_gps_bytes should fall sharply while
  // g_diag_gps_sent_per_s (complete PARSED sentences) holds steady.
  // ============================================================
  static const byte disableGLL[] = { 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2A };
  static const byte disableGSV[] = { 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x38 };
  static const byte disableVTG[] = { 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x46 };
  // Routed through gpsApplyCfg() so an M9/M10 gets the CFG-VALSET equivalent when it rejects
  // the legacy frame. GSV first — it is the largest burst and therefore the main source of
  // ACK delay for everything that follows it.
  static const byte off = 0;                       // U1 output rate 0 = disabled (VALSET path)
  const char *gsv_status = gpsApplyCfg(dialect, disableGSV, sizeof(disableGSV),
                                       KEY_MSGOUT_NMEA_GSV_U1, &off, 1);
  const char *gll_status = gpsApplyCfg(dialect, disableGLL, sizeof(disableGLL),
                                       KEY_MSGOUT_NMEA_GLL_U1, &off, 1);
  const char *vtg_status = gpsApplyCfg(dialect, disableVTG, sizeof(disableVTG),
                                       KEY_MSGOUT_NMEA_VTG_U1, &off, 1);

  // Report per-write, so a rejected config is visible at boot instead of silently shipping.
  Serial.printf("GPS config [%s]: dynModel=Sea %s | GSV %s | GLL %s | VTG %s\n",
                dialect == UBX_DIALECT_LEGACY ? "legacy CFG (u-blox 6/7/8)"
              : dialect == UBX_DIALECT_VALSET ? "CFG-VALSET (u-blox M9/M10)"
                                              : "UNVERIFIED - module sends no ACK",
                nav5_status, gsv_status, gll_status, vtg_status);

  if (dialect == UBX_DIALECT_MUTE)
    Serial.println("GPS: !! module never answered a config write. Settings were sent blind at "
                   "a confirmed baud and CANNOT be verified — run ?gpscfg.");

  if (!nav5_ok)
    Serial.println("GPS: !! dynModel NOT applied — module stays in Portable, which permits "
                   "310 m/s / 50 m/s solutions. Run ?gpscfg to confirm.");

  // V2.5-Evo - 2026-07-25 - STAGE 1: leave the MUX on GPS (channel 1) after init — the board
  // now BOOTS parked on GPS. This replaces the SW55 setUartMux(0) that parked on the VESC.
  // GPS is the peripheral that streams continuously and cannot be asked to repeat itself;
  // the VESC is polled and answers on demand, so the VESC is the visitor (getVescLoop()
  // switches to channel 0 for its poll and hands the line straight back).
  setUartMux(1);
}

// ============================================================
// cmdGpsCfg (?gpscfg) - READ BACK what the GPS module actually has
//
// V2.5-Evo - 2026-07-28. configureGPS() writes UBX config and NEVER checks the ACK — that
// is true of every UBX write in this firmware and of upstream. So "we sent dynModel=Sea"
// and "the module is in Sea" have been indistinguishable. This command closes that gap by
// POLLING the module and printing what it reports.
//
// Worth having beyond this one setting: any future UBX change (rate, constellations, an
// NMEA filter) can be silently rejected — wrong checksum, unsupported on that chip, or the
// module still at the old baud — and nothing anywhere would say so.
//
// Polls two things:
//   UBX-CFG-NAV5 (0x06 0x24) -> dynModel, byte 2 of the payload
//   UBX-CFG-MSG  (0x06 0x01) for NMEA GSV (class 0xF0 id 0x03) -> its per-port rates,
//                             which is the direct check that the GSV filter took
//
// Blocking, but bounded: 1.5 s per poll, so ~3 s worst case. Same shape as ?gpsdiag, and
// safe to run on the bench. Leaves the mux on GPS, which is its resting position anyway.
// ============================================================
static const char *dynModelName(uint8_t m)
{
  switch (m) {
    case 0:  return "Portable  <-- FACTORY DEFAULT, not what we want";
    case 2:  return "Stationary";
    case 3:  return "Pedestrian";
    case 4:  return "Automotive";
    case 5:  return "Sea  <-- CORRECT for this buggy";
    case 6:  return "Airborne <1g";
    case 7:  return "Airborne <2g";
    case 8:  return "Airborne <4g";
    case 9:  return "Wrist";
    case 10: return "Bike";
    default: return "unknown";
  }
}

// Send a UBX frame, then wait for a reply of the given class/id. Returns payload length,
// or 0 on timeout. Payload is copied into out[] up to outMax.
static uint16_t ubxPoll(const byte *req, size_t reqLen,
                        byte wantCls, byte wantId,
                        byte *out, uint16_t outMax)
{
  while (Serial1.available()) Serial1.read();   // discard stale NMEA
  Serial1.write(req, reqLen);
  Serial1.flush();

  uint32_t deadline = millis() + 1500;
  uint8_t  state = 0;
  byte     cls = 0, id = 0;
  uint16_t len = 0, idx = 0;

  while ((int32_t)(millis() - deadline) < 0)
  {
    if (!Serial1.available()) { delay(1); continue; }
    byte c = Serial1.read();

    switch (state) {
      case 0: state = (c == 0xB5) ? 1 : 0; break;
      case 1: state = (c == 0x62) ? 2 : 0; break;
      case 2: cls = c; state = 3; break;
      case 3: id  = c; state = 4; break;
      case 4: len = c;              state = 5; break;
      case 5: len |= ((uint16_t)c << 8); idx = 0;
              // Not the frame we asked for (an ACK, or unrelated) — resync.
              state = (cls == wantCls && id == wantId && len <= outMax) ? 6 : 0;
              if (state == 6 && len == 0) return 0;
              break;
      case 6:
        out[idx++] = c;
        if (idx >= len) return len;   // checksum bytes follow; we do not need them
        break;
    }
  }
  return 0;
}

void cmdGpsCfg(const String &args)
{
  (void)args;   // no arguments — signature matches the command-table dispatcher
  setUartMux(1);
  delay(5);

  Serial.println("----- GPS live config (polled from the module) -----");

  // UBX-CFG-NAV5 poll. Checksum 0x2A/0x84 over class,id,len.
  static const byte pollNav5[] = { 0xB5,0x62,0x06,0x24,0x00,0x00,0x2A,0x84 };
  byte pl[40];
  uint16_t n = ubxPoll(pollNav5, sizeof(pollNav5), 0x06, 0x24, pl, sizeof(pl));

  if (n >= 3) {
    Serial.println("  dialect  : legacy UBX-CFG (u-blox 6/7/8)");
    Serial.printf("  dynModel : %u  (%s)\n", pl[2], dynModelName(pl[2]));
    Serial.printf("  fixMode  : %u  (1=2D 2=3D 3=auto)\n", pl[3]);
  } else {
    // V2.5-Evo - 2026-07-30 - RX port: fall through to the MODERN interface before declaring
    // the module dead. On an M9/M10 the legacy CFG-NAV5 poll above does not exist, so a
    // "NO REPLY" here would be a false alarm on perfectly healthy hardware — exactly the kind
    // of lying diagnostic that sends someone power-cycling a working module.
    byte pollDyn[16] = {
      0xB5, 0x62, 0x06, 0x8B, 0x08, 0x00,
      0x00, 0x00, 0x00, 0x00,
      (byte)( KEY_NAVSPG_DYNMODEL        & 0xFF),
      (byte)((KEY_NAVSPG_DYNMODEL >>  8) & 0xFF),
      (byte)((KEY_NAVSPG_DYNMODEL >> 16) & 0xFF),
      (byte)((KEY_NAVSPG_DYNMODEL >> 24) & 0xFF),
      0x00, 0x00
    };
    ubxAppendChecksum(pollDyn, sizeof(pollDyn));
    n = ubxPoll(pollDyn, sizeof(pollDyn), 0x06, 0x8B, pl, sizeof(pl));

    if (n >= 9) {
      // CFG-VALGET response: version | layer | position(2) | key(4) | value(...)
      Serial.println("  dialect  : CFG-VALSET/VALGET (u-blox M9/M10)");
      Serial.printf("  dynModel : %u  (%s)\n", pl[8], dynModelName(pl[8]));
    } else {
    Serial.println("  dynModel : NO REPLY (neither CFG-NAV5 nor CFG-VALGET answered)");
    Serial.println("             module silent, wrong baud, or a clone implementing neither.");
    Serial.println("             If ?diag shows GPS bytes flowing, the config write is being");
    Serial.println("             rejected rather than the module being dead. Try ?gpsbaud.");
    }
  }

  // UBX-CFG-MSG poll for NMEA-GSV. Checksum 0xFC/0x14.
  static const byte pollGsv[] = { 0xB5,0x62,0x06,0x01,0x02,0x00,0xF0,0x03,0xFC,0x14 };
  n = ubxPoll(pollGsv, sizeof(pollGsv), 0x06, 0x01, pl, sizeof(pl));

  if (n >= 3) {
    // payload: msgClass, msgID, then one rate per port. Any non-zero = still emitting.
    bool on = false;
    for (uint16_t i = 2; i < n; i++) if (pl[i]) on = true;
    Serial.printf("  GSV      : %s", on ? "STILL ENABLED  <-- filter did NOT take" : "disabled (good)");
    Serial.print("   rates:");
    for (uint16_t i = 2; i < n; i++) Serial.printf(" %u", pl[i]);
    Serial.println();
  } else {
    Serial.println("  GSV      : NO REPLY");
  }

  Serial.printf("  chip_type: %u (0=BN-220 1=BN-880 2/3=M10)\n", usrConf.gps_chip_type);
  Serial.println("  Expect dynModel=5 (Sea) and GSV disabled. Anything else means the");
  Serial.println("  configureGPS() write did not stick — see ?diag for byte/sentence flow.");
  Serial.println("---------------------------------------------------");
}

// ============================================================
// Which dialect does the module speak? Decided by which POLL it answers — read-only, so safe
// to call before we have earned the right to change anything.
// ============================================================
static uint8_t gpsDetectDialect()
{
  byte pl[64];

  static const byte pollNav5[] = { 0xB5,0x62,0x06,0x24,0x00,0x00,0x2A,0x84 };
  if (ubxPoll(pollNav5, sizeof(pollNav5), 0x06, 0x24, pl, sizeof(pl)) >= 3)
    return UBX_DIALECT_LEGACY;

  byte pollDyn[16] = {
    0xB5, 0x62, 0x06, 0x8B, 0x08, 0x00,
    0x00, 0x00, 0x00, 0x00,
    (byte)( KEY_NAVSPG_DYNMODEL        & 0xFF),
    (byte)((KEY_NAVSPG_DYNMODEL >>  8) & 0xFF),
    (byte)((KEY_NAVSPG_DYNMODEL >> 16) & 0xFF),
    (byte)((KEY_NAVSPG_DYNMODEL >> 24) & 0xFF),
    0x00, 0x00
  };
  ubxAppendChecksum(pollDyn, sizeof(pollDyn));
  if (ubxPoll(pollDyn, sizeof(pollDyn), 0x06, 0x8B, pl, sizeof(pl)) >= 9)
    return UBX_DIALECT_VALSET;

  return UBX_DIALECT_MUTE;
}

// ============================================================
// cmdGpsBaud (?gpsbaud) - listen-only baud scan, and the UBX-alive check
//
// V2.5-Evo - 2026-07-30 - RX port. The scan LISTENS ONLY; UBX is polled once afterwards, at
// the baud that answered. That ordering is the whole point: transmitting at an unconfirmed
// baud produces framing errors, and ~100 of those make u-blox disable its receiver until the
// module is power-cycled.
//
// The "NMEA yes / UBX dead" combination below is the signature of exactly that state, and it
// is worth recognising by name — it looks like dead hardware and is not.
//
// ⚠️ Blocks the main loop up to ~6 s. Every wait feeds the watchdog (3 s, panic-enabled on
// this board), but RTM/FM and VESC polling are frozen meanwhile. Bench use only.
// ============================================================
void cmdGpsBaud(const String &args)
{
  (void)args;
  Serial.println("----- GPS baud scan (listen-only) -----");
  Serial.println("  baud     NMEA");

  uint32_t found = 0;
  for (uint8_t i = 0; i < kGpsBaudCount; i++) {
    uint8_t s = gpsProbeAt(kGpsBauds[i], 1100);
    Serial.printf("  %-7lu  %s\n", (unsigned long)kGpsBauds[i],
                  (s & GPS_SAW_NMEA) ? "yes" : "-");
    if (s) { found = kGpsBauds[i]; break; }
  }

  if (found) {
    gpsOpenAt(found);
    delay(250);                                    // settle before polling; 60 ms is not enough
    uint8_t d = gpsDetectDialect();
    Serial.printf("  -> NMEA at %lu; UBX input: %s\n", (unsigned long)found,
                  d == UBX_DIALECT_LEGACY ? "alive — legacy UBX-CFG (u-blox 6/7/8)"
                : d == UBX_DIALECT_VALSET ? "alive — CFG-VALSET (u-blox M9/M10)"
                                          : "DEAD — module is NOT accepting UBX");
    if (d == UBX_DIALECT_MUTE) {
      Serial.println("     u-blox disables its UART receiver after ~100 framing errors and");
      Serial.println("     stays that way until it LOSES POWER. Unplug USB and cut power to");
      Serial.println("     the RX; a reboot or re-flash will not clear it.");
    }
  } else {
    gpsOpenAt(GPS_BAUD_PREFERRED);
    Serial.println("  -> NOTHING answered. That is wiring or power, not configuration.");
    Serial.printf("     Serial1 restored to %lu.\n", (unsigned long)GPS_BAUD_PREFERRED);
  }
  setUartMux(1);
  Serial.println("---------------------------------------");
}

// ============================================================
// cmdGpsSetup (?gpssetup) - configure the GPS ONCE and make it permanent
//
// Run once on the bench when an RX is built or its GPS is swapped. Finds the module, moves it
// to 115200, applies every setting with the ACK checked, asks the module to save to its own
// non-volatile memory, then reads it back to prove it.
//
// Stored in the MODULE, never in usrConf: a confStruct field would push sizeof past its pinned
// 184 bytes and force a SW_VERSION bump, which RESETS the RX SPIFFS config — pairing, compass
// calibration, every setting, plus the logs. The module has its own memory; it costs nothing
// there and follows the module if it is moved.
//
// ⚠️ Blocks the main loop up to ~20 s. Watchdog is fed throughout. Bench use only — never with
// the buggy powered for a run.
// ============================================================
void cmdGpsSetup(const String &args)
{
  (void)args;
  Serial.println("===== RX GPS one-time setup =====");

  Serial.println("[1/5] locating module...");
  uint32_t cur = gpsDetectBaud(1100, GPS_BAUD_PREFERRED);
  if (!cur) {
    Serial.println("  FAILED: nothing answered at any baud. Check wiring and 3.3V.");
    setUartMux(1);
    return;
  }
  Serial.printf("  found at %lu baud\n", (unsigned long)cur);

  delay(250);
  uint8_t dialect = gpsDetectDialect();
  Serial.printf("[2/5] dialect: %s\n",
                dialect == UBX_DIALECT_LEGACY ? "legacy UBX-CFG (u-blox 6/7/8)"
              : dialect == UBX_DIALECT_VALSET ? "CFG-VALSET (u-blox M9/M10)"
                                              : "UNKNOWN — answers no config poll");
  if (dialect == UBX_DIALECT_MUTE) {
    Serial.println("  Nothing can be verified, so nothing will be written. Writing blind is");
    Serial.println("  what this whole change exists to stop. Power-cycle the RX and retry.");
    setUartMux(1);
    return;
  }

  Serial.println("[3/5] applying config (persisted)...");
  static const byte setNav5Sea[] = {
    0xB5,0x62,0x06,0x24,0x24,0x00,0x01,0x00,0x05,0x03,
    0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,
    0xFA,0x00,0xFA,0x00,0x64,0x00,0x5E,0x01,0x00,0x3C,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x86,0x51
  };
  static const byte disableGSV[] = { 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x38 };
  static const byte disableGLL[] = { 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2A };
  static const byte disableVTG[] = { 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x46 };
  static const byte off = 0;
  const byte sea = 5;

  const char *s_nav5 = gpsApplyCfg(dialect, setNav5Sea, sizeof(setNav5Sea),
                                   KEY_NAVSPG_DYNMODEL, &sea, 1, true);
  const char *s_gsv  = gpsApplyCfg(dialect, disableGSV, sizeof(disableGSV),
                                   KEY_MSGOUT_NMEA_GSV_U1, &off, 1, true);
  const char *s_gll  = gpsApplyCfg(dialect, disableGLL, sizeof(disableGLL),
                                   KEY_MSGOUT_NMEA_GLL_U1, &off, 1, true);
  const char *s_vtg  = gpsApplyCfg(dialect, disableVTG, sizeof(disableVTG),
                                   KEY_MSGOUT_NMEA_VTG_U1, &off, 1, true);
  Serial.printf("  dynModel=Sea %s | GSV %s | GLL %s | VTG %s\n",
                s_nav5, s_gsv, s_gll, s_vtg);

  Serial.print("[4/5] asking the module to save (BBR, plus flash if fitted)... ");
  Serial.println(gpsSaveConfig() ? "OK" : "NOT CONFIRMED");

  Serial.println("[5/5] verifying by readback:");
  cmdGpsCfg("");
  Serial.println("Reboot the RX so configureGPS() runs against the saved config.");
  Serial.println("=================================");
}

// ============================================================
// getGPSLoop - Drain the GPS UART and update GPS state
// ============================================================
//
// V2.5-Evo - 2026-07-25 - STAGE 1: this function no longer touches the UART mux at all.
//
// WHAT THE BUG WAS: it used to call setUartMux(1), wait 10 ms, drain whatever had arrived in
// those 10 ms, and then hand the line back to the VESC with setUartMux(0). Polled every 500 ms
// against a GPS that transmits ~500 bytes every 200 ms (43 ms of actual transmission), that
// listening window caught about 2% of the stream, and ~73% of the windows opened and closed
// entirely inside the silence between bursts. Measured on the owner's board: 7 GPS bytes/s and
// 0.1 complete sentences/s, against an expected ~2500 bytes/s and ~28 sentences/s.
//
// WHY THE 10 ms IS GONE: it was the settle for the Serial1.end()/begin() cycle that was
// commented out immediately below it — dead code guarding dead code. The 74HC4052 analog mux
// settles in under 3 microseconds and setUartMux() is synchronous (it returns only after the
// I2C write and its read-back verify), so by the time it returns the switch has already
// happened. The correct post-switch delay is zero.
//
// WHAT IT IS NOW: a pure drain. The mux already rests on GPS, so every byte the module sends
// lands in the Serial1 ring buffer continuously and this function just empties it.
// ============================================================
void getGPSLoop()
{
  // Non-blocking drain: read all bytes currently in the UART buffer and return immediately.
  // The GPS module sends NMEA sentences at its configured rate (5-10Hz); calling getGPSLoop()
  // on its own timer (V2_Integration_Rx.ino gps_loop_timer) ensures each call arrives after
  // at least one sentence interval, so nothing is missed and loop() is never blocked.
  bool newData = false;
  while (Serial1.available())
  {
    char c = Serial1.read();
    // STAGE 0: bare increment, nothing else added to this tight loop — one load/add/store per byte.
    g_diag_gps_bytes++;
    // gps.encode() returns true only when a COMPLETE, checksum-valid sentence has been assembled,
    // so this counter is "sentences parsed", not "sentences seen".
    if (gps.encode(c)) { newData = true; g_diag_gps_sentences++; }
  }

  // ============================================================
  // STAGE 0: collapse the raw sentence counter into "sentences parsed in the last full second"
  // for the level-4 log record. Updated at most once per second; getGPSLoop() runs at
  // gps_update_hz (1-10 Hz) so the window always closes on time. A value of 0 here while the
  // buggy is powered and outdoors is the signature of a dead GPS feed.
  // The (cur >= base) test keeps this correct after ?diagz zeroes the underlying counter mid-window.
  // ============================================================
  {
    static uint32_t diag_sent_win_ms   = 0;
    static uint32_t diag_sent_win_base = 0;
    uint32_t diag_now_ms = millis();
    uint32_t diag_cur    = g_diag_gps_sentences;
    if (diag_sent_win_ms == 0)
    {
      diag_sent_win_ms   = diag_now_ms;
      diag_sent_win_base = diag_cur;
    }
    else if ((diag_now_ms - diag_sent_win_ms) >= 1000UL)
    {
      uint32_t diag_d = (diag_cur >= diag_sent_win_base) ? (diag_cur - diag_sent_win_base) : diag_cur;
      g_diag_gps_sent_per_s = (diag_d > 255UL) ? 255 : (uint8_t)diag_d;
      diag_sent_win_ms   = diag_now_ms;
      diag_sent_win_base = diag_cur;
    }
  }

  // V2.5-Evo - 2026-04-25 - Fix: use isValid() not isUpdated() for speed check — isUpdated() fails when stationary blocking Phase B
  if (!newData || !gps.location.isValid() || !gps.speed.isValid()) {
    // No valid fix or no valid speed — not a spoof event, just no usable data.
    telemetry.foil_speed = 0xFF;  // 0xFF = no data (V2.5-Evo fix N-4: 99 collides with real speed)
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

        // ============================================================
        // V2.5-Evo - 2026-07-25 - STAGE 0 PART C: COG VALUE-CHANGE TRACKING
        //
        // The two lines above are UNCHANGED and remain the only thing the heading path reads.
        // Everything below writes separate diagnostic state that no control code touches.
        //
        // WHY IT IS HERE: gps_last_course_ms is refreshed on every valid course sentence, even
        // when the module keeps repeating the same heading. The existing cog_age_ms_div10 log
        // column is derived from that timestamp, so it reported a healthy, fresh COG straight
        // through the exact failure that cost the owner control — the VALUE was frozen on one
        // heading while its timestamp kept ticking. Tracking the value separately is the only
        // way a session log can tell "GPS is updating" from "GPS is repeating itself".
        // ============================================================
        {
          static float diag_cog_prev_deg = -1.0f;   // -1.0f = no COG value captured yet this session
          float diag_cog_now = (float)gps.course.deg();
          uint32_t diag_t = millis();
          if (diag_t == 0) diag_t = 1;              // 0 is the "never captured" sentinel — never store a literal 0

          if (diag_cog_prev_deg < 0.0f)
          {
            // First COG ever seen: this is the BASELINE, not a change. Start the frozen-clock
            // here so "frozen for N s" is measured from first sighting rather than from boot.
            diag_cog_prev_deg    = diag_cog_now;
            g_diag_cog_change_ms = diag_t;
          }
          else if (fabsf(diag_cog_now - diag_cog_prev_deg) > kDiagCogChangeDeg)
          {
            // A plain absolute difference is the right test for "did the number move": a
            // 359.9 -> 0.1 wrap is a genuine 0.2 degree turn AND a genuine value change, and
            // either way it is not a frozen reading. No angle normalisation needed.
            diag_cog_prev_deg    = diag_cog_now;
            g_diag_cog_change_ms = diag_t;
            g_diag_cog_val_changes++;
          }
        }
        g_diag_cog_ts_updates++;
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

  // ============================================================
  // STAGE 0: sample the RX fix age once per GPS poll. Sampling here rather than only on a
  // successful update is what makes staleness visible: if readings stop arriving, gps_last_ms
  // stops advancing and successive samples grow, so the mean and max in ?diag climb.
  // gps_last_ms == 0 means no reading has ever been accepted — skipped, not counted as age 0.
  // ============================================================
  if (gps_last_ms != 0)
  {
    uint32_t diag_fix_age = (uint32_t)(millis() - gps_last_ms);
    g_diag_fix_age_sum_ms += diag_fix_age;
    g_diag_fix_age_samples++;
    if (diag_fix_age > g_diag_fix_age_max_ms) g_diag_fix_age_max_ms = diag_fix_age;
  }

  // V2.5-Evo - 2026-07-25 - STAGE 1: the trailing setUartMux(0) that used to be here is DELETED.
  // Handing the line back to the VESC on the way out is exactly what starved the GPS: the module
  // kept transmitting into a mux that was pointed somewhere else for ~98% of every second.
  // The mux now rests on GPS between polls and only getVescLoop() ever moves it.
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