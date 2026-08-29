// V2.5-Evo - 2026-07-24 - F9: cache last control-packet RSSI/SNR (g_last_rssi_dbm/g_last_snr_db) at receive so the logger task can add distance+link-quality CSV columns without racing the radio SPI bus. No confStruct/SW_VERSION change.
// V2.5-Evo - 2026-07-20 - FM engagement semantics: processFmOverridePacket() stamps fm_mode_last_rx_ms on every 0xF2 so RTMState.ino can expire an unrefreshed FM declaration (95 s). This handler is now the ONLY path that can arm FM — the RX no longer auto-arms from usrConf.followme_mode.
// V2.5-Evo - 2026-05-12 - Fix Phase B recovery: recheck gate reduces from 30s to 2s when gps_phase_b_ok=false, eliminating up to 30s RTM motor block after any TX GPS gap
// V2.5-Evo - 2026-05-03 - Removed if(0) dead code; checkAndAdjustAddress() TODO noted
// V2.5-Evo - 2026-04-29 - Bundle C: startTransmit() return value checked and logged on error
// V2.5-Evo - 2026-04-24 - Added GPS meta-packet reception: gps_meta_pending state, processMetaGpsPacket(), triggeredReceive() 2-path state machine
// V2.5-Evo - 2026-04-24 - Added Phase B GPS handshake check: gpsPhaseBCheck() called from processMetaGpsPacket()
// V2.5-Evo - 2026-04-25 - P7: Added processRtmStatePacket(), processFmOverridePacket(); dispatch 0xF1/0xF2 in triggeredReceive()

// V2.5-Evo - 2026-07-24 - F9: cache last control-packet link quality for the logger CSV.
// radio.getRSSI()/getSNR() read SX1262 registers over SPI and are only valid right after a reception
// in triggeredReceive(). The logger runs in a separate FreeRTOS task, so it must NOT touch the radio
// SPI bus concurrently — it reads these cached copies instead (see extern in Logger.ino convertToLogData()).
float g_last_rssi_dbm = 0.0f;   // dBm; snapshot of the last control packet's RSSI
float g_last_snr_db   = 0.0f;   // dB;  snapshot of the last control packet's SNR

void radioErrorHalt(int type)
{
  if(type == 1) while(1) blinkErr(2, AP_L_BIND);
  while(1) blinkErr(3, AP_L_BIND);
}

void radioInitSuccess()
{
  // No extra init needed on RX side
}

void startupRadio()
{
  initRadioHardware();
}

void ICACHE_RAM_ATTR packetReceived(void)
{
  if(rxIsrState && !rfInterrupt)
  {
    rfInterrupt = true;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(triggerReceiveSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  else
  {
    rfInterrupt = true;
  }
}

// ============================================================
// V2.5-Evo - 2026-08-05 - SAFETY: may this RX obey a control packet at all?
// ============================================================
// THE BUG THIS CLOSES: the only gate on an inbound control packet was the
// 3-byte destination-address compare in triggeredReceive(). usrConf.paired was
// never consulted anywhere in the control path.
//
// Why that mattered: unpairing ZEROES own_address (Common/WebConfigEngine.h), so
// an unpaired RX listens on 00:00:00 — the one address every unbound unit in the
// world shares. An unpaired TX transmits to 00:00:00 for the same reason. So two
// unpaired units in radio range would match, pass CRC, and drive the motor,
// with nothing in between. No pairing, no consent, no gesture.
//
// TWO conditions, because either one alone still leaves the hole open:
//   !paired             - never bound, or deliberately unpaired
//   own_address all 00  - the factory/unpaired address. Even if `paired` were
//                         somehow true (corrupt config, a half-finished bind),
//                         00:00:00 must never be commandable.
//
// Deliberately placed so it gates the WHOLE control path, not just throttle:
// every writer of thr_received AND every writer of last_packet (the failsafe
// feed) lives inside that address block, including the 0xF1/0xF2/0xF4 meta
// packets. Gating only the throttle would still have let a stranger hold the
// failsafe open, which is the more dangerous half.
//
// Costs nothing on the wire, changes nothing for a paired board.
//
// NOTE ON PAIRING: waitForPairing() sets usrConf.paired = false while it runs,
// and the receive task IS live at that point (initTasks() is called in setup(),
// checkButtons() first runs from loop()). That is correct and intended — during
// a bind the board must not also be drivable.
static bool rxMayAcceptControl()
{
  if (!usrConf.paired) return false;
  if (usrConf.own_address[0] == 0 &&
      usrConf.own_address[1] == 0 &&
      usrConf.own_address[2] == 0) return false;
  return true;
}

// Function for waiting node to pair
bool waitForPairing()
{
  usrConf.paired = false;

  uint8_t responsePacket[8];
  unsigned long startTime = millis();

  while (millis() - startTime < PAIRING_TIMEOUT)
  {
    radio.implicitHeader(5);
    radio.startReceive();
    rxprintln("Waiting for pairing packet...");
    uint8_t buffer[15];

    while(!rfInterrupt && millis() - startTime < PAIRING_TIMEOUT) blinkBind(2);
    delay(10);
    if (rfInterrupt && radio.readData(buffer, 15) == RADIOLIB_ERR_NONE)
    {
      rfInterrupt = false;
      rxprintln("Received response");
      #ifdef DEBUG_RX
      printHexArray(buffer, 15);
      #endif

      if (buffer[0] == 0xAB)
      {
        rxprintln("1st byte matches");
        // Verify CRC of received packet directly
        uint8_t receivedCRC = buffer[4];
        uint8_t calculatedCRC = esp_crc8(buffer, 4);

        if (receivedCRC == calculatedCRC)
        {
          rxprintln("CRC ok");
          // Store potential partner's address
          uint8_t temp_addr[3];
          memcpy(temp_addr, buffer + 1, 3);

          // Prepare response packet
          responsePacket[0] = 0xBA;
          memcpy(responsePacket + 1, temp_addr, 3);
          memcpy(responsePacket + 4, usrConf.own_address, 3);

          // Calculate CRC;
          responsePacket[7] = esp_crc8(responsePacket, 7);

          delay(100);
          rxprintln("Sending response: ");
          #ifdef DEBUG_RX
          printHexArray(responsePacket, 8);
          #endif
          // Send response
          radio.implicitHeader(8);
          {
            int16_t _txErr = radio.startTransmit(responsePacket, 8);
            if (_txErr != RADIOLIB_ERR_NONE)
              Serial.printf("[Radio] startTransmit error %d at line %d\n", _txErr, __LINE__);
          }
          delay(10);
          radio.startReceive();
          rfInterrupt = false;

          while (millis() - startTime < PAIRING_TIMEOUT)
          {
            while(!rfInterrupt && millis() - startTime < PAIRING_TIMEOUT) delay(10);
            delay(10);
            if (rfInterrupt && radio.readData(buffer, 15) == RADIOLIB_ERR_NONE)
            {
              rfInterrupt = false;
              rxprintln("Received response");
              if (buffer[0] == 0xAC && memcmp(buffer + 1, usrConf.own_address, 3) == 0)
              {
                rxprintln("Address matches");
                // Verify CRC of final packet directly
                receivedCRC = buffer[7];
                calculatedCRC = esp_crc8(buffer, 7);

                if (receivedCRC == calculatedCRC)
                {
                  rxprintln("CRC ok, pairing success");
                  // Save partner's address
                  memcpy(usrConf.dest_address, buffer + 4, 3);
                  usrConf.paired = true;
                  saveConfToSPIFFS(usrConf);
                  aw.digitalWrite(AP_L_BIND, LOW);
                  return true;
                }
              }
            }
            else rfInterrupt = false;
          }
        }
      }
    }
    else rfInterrupt = false;
  }
  return false;
}

// ============================================================
// PHASE B GPS HANDSHAKE STATE
//
// gps_phase_b_ok: set true when Phase B distance and speed checks
// both pass; set false when either fails. Initialized false so
// RTM arming is blocked until the first successful handshake.
// NOT static — the RTM state machine (Priority 7) reads this flag.
//
// The static variables below are internal to gpsPhaseBCheck() and
// track timing and the previous TX position snapshot across calls.
// ============================================================
bool gps_phase_b_ok = false;  // Phase B handshake result; false = RTM arming blocked

// Last time Phase B check ran (ms). 0 = never run this session.
static unsigned long gps_phase_b_last_check_ms = 0;

// Previous TX GPS position snapshot used to compute TX implied speed.
// Updated each time Phase B check runs. 0 = no prior snapshot.
static double        gps_phase_b_prev_tx_lat = 0.0;
static double        gps_phase_b_prev_tx_lng = 0.0;
static unsigned long gps_phase_b_prev_tx_ms  = 0;

// ============================================================
// gpsPhaseBCheck - Phase B GPS handshake anti-spoofing validation
// ============================================================
//
// What it does:
//   Called after every successful 0xF3 GPS meta-packet decode.
//   Validates that TX and RX are physically plausible partners:
//
//   1) Distance check: Haversine distance between the last accepted
//      RX GPS position and the just-received TX GPS position must
//      be < usrConf.gps_max_pair_dist_m. Catches a spoofed TX GPS
//      report placing TX far from RX.
//
//   2) Speed consistency check: TX implied speed, computed from two
//      consecutive Phase B check positions, must differ from RX GPS
//      speed by < usrConf.gps_max_speed_diff_kmh. Catches GPS
//      replay attacks that report implausible TX movement.
//      Skipped on the very first check (no prior TX snapshot yet).
//
//   Time-gated: runs only on the first call after boot and every
//   30 seconds thereafter, regardless of how often meta-packets
//   arrive (they come at 2Hz).
//
//   Skipped entirely if:
//   - GPS disabled (usrConf.gps_en == 0)
//   - RX has no valid GPS reading (gps_last_ms == 0)
//
// Inputs:
//   Reads globals: rx_tx_gps_lat/lng (just updated by processMetaGpsPacket),
//   gps_last_lat/lng/speed_kmh/ms (from GPS.ino), usrConf fields.
//
// Side effects:
//   Sets gps_phase_b_ok (true = pass, false = fail).
//   Updates gps_phase_b_last_check_ms and gps_phase_b_prev_tx_* snapshot.
//   Prints diagnostics to Serial.
// ============================================================
static void gpsPhaseBCheck()
{
  // ---- Prerequisite: GPS must be enabled in config ----
  if (!usrConf.gps_en)
  {
    // GPS disabled — Phase B cannot run; do not change gps_phase_b_ok.
    return;
  }

  // ---- Prerequisite: RX must have at least one valid accepted GPS reading ----
  if (gps_last_ms == 0)
  {
    rxprintln("GPS [PhB] Skipped — RX has no valid GPS reading yet");
    return;
  }

  // ---- Time gate: 30s when Phase B good (anti-spoofing); 2s when failed (recovery) ----
  unsigned long now = millis();
  unsigned long recheck_ms = gps_phase_b_ok ? 30000UL : 2000UL;
  if (gps_phase_b_last_check_ms != 0 &&
      (now - gps_phase_b_last_check_ms) < recheck_ms)
  {
    return;  // Not due yet; skip silently
  }
  gps_phase_b_last_check_ms = now;

  // ---- Check 1: TX-RX Haversine distance ----
  // TinyGPSPlus::distanceBetween() returns metres between two WGS84 lat/lng pairs.
  float dist_m = (float)TinyGPSPlus::distanceBetween(
      gps_last_lat, gps_last_lng,
      rx_tx_gps_lat, rx_tx_gps_lng);

  if (dist_m > usrConf.gps_max_pair_dist_m)
  {
    Serial.printf("GPS [PhB] FAIL distance: %.0f m > max %.0f m — RTM arming blocked\n",
                  dist_m, (double)usrConf.gps_max_pair_dist_m);
    gps_phase_b_ok = false;
    // Update snapshot so the next check has a fresh reference point.
    gps_phase_b_prev_tx_lat = rx_tx_gps_lat;
    gps_phase_b_prev_tx_lng = rx_tx_gps_lng;
    gps_phase_b_prev_tx_ms  = now;
    return;
  }

  // ---- Check 2: TX-RX speed consistency ----
  // Compute TX implied speed from the position change since the last Phase B snapshot.
  // Skip on the very first run because there is no prior snapshot to measure from.
  if (gps_phase_b_prev_tx_ms > 0)
  {
    float dt_s = (float)(now - gps_phase_b_prev_tx_ms) / 1000.0f;

    // Guard against near-zero dt (shouldn't happen due to 30s gate, but be safe).
    if (dt_s > 0.1f)
    {
      float tx_delta_m   = (float)TinyGPSPlus::distanceBetween(
          gps_phase_b_prev_tx_lat, gps_phase_b_prev_tx_lng,
          rx_tx_gps_lat, rx_tx_gps_lng);
      float tx_speed_kmh = (tx_delta_m / dt_s) * 3.6f;
      float speed_diff   = fabsf(tx_speed_kmh - gps_last_speed_kmh);

      if (speed_diff > usrConf.gps_max_speed_diff_kmh)
      {
        Serial.printf("GPS [PhB] FAIL speed: TX %.1f km/h, RX %.1f km/h, diff %.1f km/h > max %.1f km/h\n",
                      tx_speed_kmh, gps_last_speed_kmh, speed_diff,
                      (double)usrConf.gps_max_speed_diff_kmh);
        gps_phase_b_ok = false;
        gps_phase_b_prev_tx_lat = rx_tx_gps_lat;
        gps_phase_b_prev_tx_lng = rx_tx_gps_lng;
        gps_phase_b_prev_tx_ms  = now;
        return;
      }
    }
  }

  // ---- All checks passed ----
  Serial.printf("GPS [PhB] PASS: dist %.0f m (max %.0f m)\n",
                dist_m, (double)usrConf.gps_max_pair_dist_m);
  gps_phase_b_ok = true;
  gps_phase_b_prev_tx_lat = rx_tx_gps_lat;
  gps_phase_b_prev_tx_lng = rx_tx_gps_lng;
  gps_phase_b_prev_tx_ms  = now;
}

// V2.5-Evo - 2026-04-25 - P7: Handle 0xF1 RTM state meta-packet from TX.
// pkt: 6-byte buffer. byte[3]=0xF1, byte[4]: 0=RTM deactivate, 1=RTM activate.
// Sets rtm_rx_active. Safety gates in RTMState.ino may override during active RTM.
static void processRtmStatePacket(const uint8_t *pkt)
{
  uint8_t new_state = pkt[4];
  if (new_state == 0)
  {
    rtm_rx_active         = false;
    rtm_rx_emergency_stop = false;
    Serial.println("RTM [RX] deactivated by TX");
  }
  else if (new_state == 1)
  {
    // RTM state machine in RTMState.ino will run safety gates on next iteration.
    rtm_rx_active = true;
    Serial.println("RTM [RX] activation requested by TX");
  }
}

// V2.5-Evo - 2026-04-25 - P7: Handle 0xF2 FM override meta-packet from TX.
// pkt: 6-byte buffer. byte[3]=0xF2, byte[4]: FM mode 0-3.
// Updates runtime FM mode without writing SPIFFS.
// V2.5-Evo - 2026-07-20 - R0/R2: 0xFF (the reboot value) no longer means "use the SPIFFS
// default" — it now means "the TX has made no declaration this session", which RTMState.ino
// treats as FM_IDLE. This handler is therefore the ONLY way FM can ever become armed.
// Each packet also stamps fm_mode_last_rx_ms so the RX can expire a declaration that stops
// being refreshed (the TX repeats 0xF2/mode every 30 s while armed).
static void processFmOverridePacket(const uint8_t *pkt)
{
  uint8_t mode = pkt[4] & 0x03;  // clamp to 0-3
  fm_mode_runtime = mode;
  fm_mode_last_rx_ms.store(millis(), std::memory_order_relaxed);
  Serial.printf("FM [RX] mode override: %d\n", mode);
}

// V2.5-Evo - 2026-04-24 - GPS meta-packet state and handler for 0xF3 protocol

// gps_meta_pending: set true when a 0xF3 announcement (6-byte) is received.
// On the NEXT wakeup of triggeredReceive, read 14 bytes (GPS data) instead
// of the normal 6-byte control packet.
static bool gps_meta_pending = false;

// ============================================================
// processMetaGpsPacket - Decode a received 14-byte GPS data packet
// ============================================================
//
// What it does:
//   Validates destination address, CRC8 (over bytes 0-12 stored in byte 13),
//   packet type (0xF3) and subtype (0x02). On success, extracts TX lat/lng
//   stored as int32_t microdegrees (little-endian) and writes the three
//   rx_tx_gps_* globals declared in BREmote_V2_Rx.h.
//
// Inputs:
//   pkt - pointer to a 14-byte buffer containing the received GPS data packet
//
// Side effects:
//   On success: updates rx_tx_gps_lat, rx_tx_gps_lng, rx_tx_gps_timestamp.
//   Always: prints diagnostics to Serial.
// ============================================================
// ============================================================
// gpsPhaseATxCheck - Phase A validation for the RIDER's position
// ============================================================
// V2.5-Evo - 2026-08-27 - PHASE-A-TX-1.
//
// WHAT WAS MISSING, and it was a gap in a feature that was supposed to be symmetric. Phase A
// anti-spoofing (HDOP / teleport / acceleration) has existed since 2026-04-22, but
// gpsPhaseACheck() has exactly ONE call site - GPS.ino, on the RX's OWN GPS. The rider's
// position arrives over the radio and was written straight into rx_tx_gps_lat/lng with no
// plausibility test of any kind. Phase B cross-checks the two tracks and the FM controller
// EMA-filters the rider, but nothing ever asked "could a human have moved that far in that
// time?" about the very position every Follow-Me and RTM distance is computed from.
//
// IT IS NOT THEORETICAL. Beta tester heiguga/robertzach's 37-minute level-4 log contains nine
// position spikes; the worst moves the rider 24 -> 52 m in 0.34 s, an implied 301 km/h. The
// teleport limit is usrConf.gps_max_teleport_kmh, default 80. That spike is 3.7x over it and
// would have been rejected outright.
//
// WHY IT COSTS NOTHING IN TRACKING LATENCY, which is the objection worth answering up front:
// this REJECTS, it does not SMOOTH. A smoothing filter delays every sample, good ones included -
// which is why computeFmTarget() carries a lag anchor pushing the target forward by up to 2x
// d_follow purely to cancel the EMA's own delay. A rejection test adds nothing to a good fix:
// it passes through on the same tick it always did. Only an already-impossible reading is
// dropped.
//
// WHAT IS AND IS NOT CHECKED. The 0xF3/0x02 meta packet carries lat/lng only - no HDOP, no
// speed - so check 1 (HDOP) has no input and is skipped; the TX runs its own Phase A on its own
// module before transmitting, which is where HDOP is properly judged. Checks 2 and 3 are both
// reconstructed from consecutive positions.
//
// THREE GUARDS, ALL BIASED TOWARD ACCEPTING. A wrongly-rejected fix ages the rider's position
// and can trip FM's Gate 4 staleness timeout, so a false reject costs more than a missed spike -
// and a missed spike is caught downstream anyway by the 3-fix separation dwell. Phase A here is
// a second net, never the only one.
//   DT FLOOR   - rx_tx_gps_timestamp is stamped at PACKET RECEIPT, not at fix time, so after a
//     link dropout two packets can arrive back-to-back with a genuine position change between
//     them. The raw dt is then tiny and the implied speed enormous: good data rejected as a
//     teleport. Flooring dt makes the implied speed SMALLER, i.e. more permissive.
//   MAX GAP    - after a long silence the baseline is too old to say anything useful, so the
//     reading is accepted and the baseline re-seeded rather than judged against stale data.
//   REJECT RUN - if a bad position ever becomes the baseline, every good fix afterwards looks
//     like a teleport away from it and the rider would be frozen out permanently. After
//     kTxPhaseAMaxRejects consecutive rejections the checker assumes ITSELF to be the problem,
//     accepts, and re-seeds.
// ============================================================
// REX PA-4: this floor and usrConf.gps_max_teleport_kmh MULTIPLY into a blind spot -
// (gps_max_teleport_kmh / 3.6) * kTxPhaseADtFloorS metres can always slip through regardless of
// arrival timing. At the 80 km/h default that is 5.56 m: below the follow station, comparable to
// ordinary GPS scatter, and 5x smaller than the worst spike in the beta logs. The validator allows
// gps_max_teleport_kmh up to 500, where the blind spot becomes 34.7 m and would swallow that spike
// whole. Do not raise the teleport limit without re-reading this line.
static const float    kTxPhaseADtFloorS   = 0.25f;  // s; smallest dt used for the speed maths
static const float    kTxPhaseAMaxGapS    = 10.0f;  // s; older than this, re-seed instead of judge
static const uint8_t  kTxPhaseAMaxRejects = 3;      // consecutive rejects before force-accepting

static double        tx_pa_prev_lat   = 0.0;
static double        tx_pa_prev_lng   = 0.0;
static unsigned long tx_pa_prev_ms    = 0;
static uint8_t       tx_pa_reject_run = 0;

uint16_t gps_tx_phase_a_rejects = 0;   // lifetime counter, surfaced by ?diag


// Previously accepted rider position, kept as the RAW int32 microdegrees off the wire rather than
// as reconstructed doubles: integer comparison is exact by construction and needs no reasoning
// about float representation. These feed rx_tx_gps_fix_seq, which is defined and fully explained
// in BREmote_V2_Rx.h beside the other rx_tx_gps_* globals.
static int32_t tx_seq_prev_lat_ud = 0;
static int32_t tx_seq_prev_lng_ud = 0;
static bool    tx_seq_seeded      = false;

static bool gpsPhaseATxCheck(double cur_lat, double cur_lng, unsigned long now_ms)
{
  // No baseline yet - nothing to compare against. Seed and accept.
  if (tx_pa_prev_ms == 0) {
    tx_pa_prev_lat = cur_lat;
    tx_pa_prev_lng = cur_lng;
    tx_pa_prev_ms  = now_ms;
    return true;
  }

  float dt_raw = (float)(now_ms - tx_pa_prev_ms) / 1000.0f;

  // Baseline too old to judge against, or the clock went backwards - re-seed, do not reject.
  if (dt_raw > kTxPhaseAMaxGapS || dt_raw <= 0.0f) {
    tx_pa_prev_lat = cur_lat;
    tx_pa_prev_lng = cur_lng;
    tx_pa_prev_ms  = now_ms;
    return true;
  }

  float dt_s = (dt_raw < kTxPhaseADtFloorS) ? kTxPhaseADtFloorS : dt_raw;

  float dist_m      = (float)TinyGPSPlus::distanceBetween(
                               tx_pa_prev_lat, tx_pa_prev_lng, cur_lat, cur_lng);
  float implied_kmh = (dist_m / dt_s) * 3.6f;

  // ---- Check 2: teleport. THE ONLY CHECK, DELIBERATELY. ----
  // REX PA-2 KILLED CHECK 3 (acceleration), and the reasoning is worth keeping because the same
  // trap will catch the next person who tries to add it back.
  //
  // Acceleration needs two successive SPEED measurements. What is available here is two successive
  // position differences over a RECEIVE clock, not a fix clock - so computing acceleration from it
  // differentiates the packet cadence twice and measures an artifact.
  //
  // Concretely: the TX transmits at 2 Hz off a 1 Hz GPS, so consecutive packets alternate REAL MOVE
  // / REPEATED POSITION. On a move packet the implied speed is 2v; on the repeat it is 0. The old
  // guard `tx_pa_prev_kmh > 0.0f` was false after a repeat, so the check SKIPPED every packet
  // carrying information and RAN it on every packet carrying an artifact - judging a synthetic 2v -> 0 step that no
  // rider ever performed. It crossed gps_max_accel_g (3.0) at a rider speed of 26.5 km/h at 2 Hz,
  // and at the 3.0 Hz rate actually measured in the beta logs it tripped at 11.8 km/h - which is
  // WELL INSIDE ordinary riding speed, and barely above the 10 km/h a rider must already exceed
  // for Follow-Me to engage at all. It would have rejected good rider fixes during normal use,
  // in exactly the speed band where the separation dwell is trying to accumulate them.
  //
  // The teleport check alone catches the observed spikes with 3.7x margin (worst logged spike
  // implies 301 km/h against an 80 km/h limit), and it needs only ONE position difference. Since
  // R4-1 the caller does not even invoke this function on a re-broadcast, so the alternating
  // pattern never reaches it and the dt it measures is always between two DISTINCT positions.
  bool bad = (implied_kmh > usrConf.gps_max_teleport_kmh);

  if (bad) {
    tx_pa_reject_run++;
    if (gps_tx_phase_a_rejects < 65535) gps_tx_phase_a_rejects++;
    Serial.printf("GPS [PhA-TX] rider position rejected: %.0f m in %.2f s = %.0f km/h "
                  "(max %.0f), run %u/%u\n",
                  (double)dist_m, (double)dt_raw, (double)implied_kmh,
                  (double)usrConf.gps_max_teleport_kmh,
                  (unsigned)tx_pa_reject_run, (unsigned)kTxPhaseAMaxRejects);

    // REJECT RUN escape: the baseline is more likely wrong than the whole world.
    if (tx_pa_reject_run >= kTxPhaseAMaxRejects) {
      Serial.println("GPS [PhA-TX] reject run hit the limit - re-seeding the baseline and "
                     "accepting. The stored reference was probably the bad one.");
      tx_pa_prev_lat   = cur_lat;
      tx_pa_prev_lng   = cur_lng;
      tx_pa_prev_ms    = now_ms;
      tx_pa_reject_run = 0;
      return true;
    }
    return false;    // baseline deliberately NOT advanced - next fix is judged from the last good one
  }

  tx_pa_prev_lat   = cur_lat;
  tx_pa_prev_lng   = cur_lng;
  tx_pa_prev_ms    = now_ms;
  tx_pa_reject_run = 0;
  return true;
}

static void processMetaGpsPacket(uint8_t *pkt)
{
  if (memcmp(pkt, usrConf.own_address, 3) != 0)
  {
    rxprintln("META GPS: address mismatch, discarding");
    return;
  }

  // CRC covers bytes 0-12; result stored in byte 13
  if (pkt[13] != esp_crc8(pkt, 13))
  {
    rxprintln("META GPS: CRC fail, discarding");
    return;
  }

  if (pkt[3] != 0xF3 || pkt[4] != 0x02)
  {
    rxprintln("META GPS: unexpected type/subtype, discarding");
    return;
  }

  // Extract lat/lng as int32_t microdegrees stored little-endian.
  // memcpy avoids strict-aliasing UB that a direct pointer cast would cause.
  int32_t lat_ud, lng_ud;
  memcpy(&lat_ud, pkt + 5, 4);
  memcpy(&lng_ud, pkt + 9, 4);

  double   cand_lat = (double)lat_ud / 1e6;
  double   cand_lng = (double)lng_ud / 1e6;
  unsigned long now = millis();

  // REX R4-1 / Q6: is this a genuinely NEW position, or the TX re-broadcasting the last one?
  // The TX transmits at 2 Hz off a 1 Hz GPS (3.0 Hz measured against 1 Hz in the beta logs), so
  // roughly every other packet carries a position the RX has already seen. Compared on the RAW
  // integers, which is exact.
  const bool is_new_fix = (!tx_seq_seeded) ||
                          (lat_ud != tx_seq_prev_lat_ud) || (lng_ud != tx_seq_prev_lng_ud);

  // PHASE-A-TX-1: judge the rider's position before it becomes the number every FM and RTM
  // distance is computed from. On rejection NOTHING is written - not the position and not the
  // timestamp - so the previous good fix simply ages, exactly as it does when a packet is lost.
  // That is the honest representation: we do not have a fresh rider position this tick.
  //
  // A RE-BROADCAST IS NOT JUDGED AT ALL. It carries no new evidence, so evaluating it would only
  // advance the Phase A baseline clock and shorten the dt seen by the next genuine fix - which
  // inflates that fix's implied speed and risks rejecting good data. Skipping keeps the teleport
  // check measuring between DISTINCT positions, which is the only interval where speed means
  // anything.
  if (is_new_fix) {
    if (!gpsPhaseATxCheck(cand_lat, cand_lng, now)) {
      return;
    }
    tx_seq_prev_lat_ud = lat_ud;
    tx_seq_prev_lng_ud = lng_ud;
    tx_seq_seeded      = true;
  }

  rx_tx_gps_lat       = cand_lat;
  rx_tx_gps_lng       = cand_lng;
  rx_tx_gps_timestamp = now;

  // REX S-1: the counter is published LAST, after the position it advertises. Bumping it before
  // the writes meant a loop-task read landing in between saw "a new fix is available" with the
  // PREVIOUS position still in place. Harmless for today's only consumer, which reads the counter
  // and nothing else - but this is now the SSOT for "is there a new rider fix", and the next
  // consumer will read lat/lng on the strength of it. Same publish-last discipline the timestamp
  // already follows two lines up, for the same reason.
  if (is_new_fix) rx_tx_gps_fix_seq++;

  #ifdef DEBUG_RX
  Serial.printf("META GPS received: lat=%.6f lng=%.6f\n",
                rx_tx_gps_lat, rx_tx_gps_lng);
  #endif

  // Run Phase B anti-spoofing check against the freshly received TX GPS position.
  // gpsPhaseBCheck() is time-gated (first call + every 30s) and self-throttles.
  gpsPhaseBCheck();
}

void triggeredReceive(void *parameter) {
  while (1)
  {
    // Feed WDT on every iteration regardless of packet activity.
    // portMAX_DELAY would block indefinitely when TX is off, preventing WDT reset.
    // 2000ms timeout: short enough to feed the 3000ms WDT, long enough to avoid
    // busy-looping when the radio is quiet.
    // V2.5-Evo - 2026-07-31 - RX-WDT-2: gated, same reason as PWM.ino — this task starts
    // before initWatchdog() subscribes it. Feed behaviour after subscription is unchanged.
    if (g_wdt_active) esp_task_wdt_reset();
    if (xSemaphoreTake(triggerReceiveSemaphore, pdMS_TO_TICKS(2000)) == pdTRUE)
    {
      if (gps_meta_pending)
      {
        // ---- GPS data packet path (14 bytes) ----
        // A 0xF3 announcement arrived on the previous wakeup.
        // Radio is already in implicitHeader(14) + startReceive mode.
        // TX has sent the 14-byte GPS coordinate packet; read and decode it.
        gps_meta_pending = false;  // clear before any early return

        uint8_t gpsArray[14];
        if (radio.readData(gpsArray, 14) == RADIOLIB_ERR_NONE)
        {
          processMetaGpsPacket(gpsArray);
        }
        else
        {
          rxprintln("META GPS: readData error");
        }
        // Fall through to common exit below (implicitHeader(6) + startReceive + rfInterrupt=false)
      }
      else
      {
        // ---- Normal 6-byte control packet path ----
        uint8_t rcvArray[6];
        if (radio.readData(rcvArray, 6) == RADIOLIB_ERR_NONE)
        {
          rxprint("Received packet: ");
          #ifdef DEBUG_RX
          printHexArray(rcvArray, 6);
          #endif

          // V2.5-Evo - 2026-08-05 - SAFETY GATE. Checked BEFORE the address compare,
          // because on an unpaired board the address IS 00:00:00 and would match a
          // stranger. See rxMayAcceptControl() above for the full reasoning.
          if (!rxMayAcceptControl())
          {
            rxprintln("Ignored: RX is not paired — hold BIND at boot to pair");
          }
          else if (memcmp(rcvArray, usrConf.own_address, 3) == 0)
          {
            rxprintln("Address matches");

            if (rcvArray[5] == esp_crc8(rcvArray, 5))
            {
              rxprintln("CRC ok");

              if (rcvArray[3] == 0xF1)
              {
                // ---- RTM state meta-packet ----
                // TX signals RTM active (1) or inactive (0). No telemetry reply.
                last_packet = millis();  // meta-packet proves TX is alive
                processRtmStatePacket(rcvArray);
              }
              else if (rcvArray[3] == 0xF2)
              {
                // ---- FM override meta-packet ----
                // TX cycles follow-me mode. No telemetry reply.
                last_packet = millis();  // meta-packet proves TX is alive
                processFmOverridePacket(rcvArray);
              }
              else if (rcvArray[3] == 0xF4)
              {
                // ---- Aux control meta-packet ----
                last_packet = millis();
                rx_aux_flags = rcvArray[4];
              }
              else if (rcvArray[3] == 0xF3)
              {
                // ---- GPS announcement ----
                // TX will send a 14-byte GPS data packet ~10ms from now.
                // Switch radio to 14-byte mode immediately — SPI writes take
                // <2ms, so we will be ready well before the GPS data arrives.
                rxprintln("META GPS: announcement, switching to 14-byte mode");
                gps_meta_pending = true;
                radio.implicitHeader(14);
                radio.startReceive();
                rfInterrupt = false;
                // TX does not expect a telemetry reply here; skip the common exit.
                continue;
              }
              else
              {
                // ---- Normal throttle/steering control packet ----
                last_packet = millis();
#ifdef WIFI_ENABLED
                webCfgNotifyRxConnected();
#endif
                rxprint("RSSI: ");
                rxprint(radio.getRSSI());
                rxprint(", SNR: ");
                rxprintln(radio.getSNR());

                thr_received      = rcvArray[3];
                steering_received = rcvArray[4];

                // V2.5-Evo - 2026-07-24 - F9: snapshot RSSI/SNR while the radio SPI bus is valid (just after
                // this reception) so the logger task can read them without touching the radio concurrently.
                g_last_rssi_dbm = radio.getRSSI();
                g_last_snr_db   = radio.getSNR();
                telemetry.link_quality = getLinkQuality(g_last_rssi_dbm, g_last_snr_db);

                rxprintln("Sending response");

                uint8_t sendArray[6];
                memcpy(sendArray, usrConf.dest_address, 3);
                uint8_t* ptr = (uint8_t*)&telemetry;
                sendArray[3] = telemetry_index;
                sendArray[4] = ptr[telemetry_index];
                telemetry_index++;
                if(telemetry_index >= sizeof(TelemetryPacket))
                {
                  telemetry_index = 0;
                }
                sendArray[5] = esp_crc8(sendArray, 5);

                #ifdef DEBUG_RX
                printHexArray(sendArray, 6);
                #endif

                vTaskDelay(pdMS_TO_TICKS(10));
                radio.implicitHeader(6);
                {
                  int16_t _txErr = radio.startTransmit(sendArray, 6);
                  if (_txErr != RADIOLIB_ERR_NONE)
                    Serial.printf("[Radio] startTransmit error %d at line %d\n", _txErr, __LINE__);
                }
                vTaskDelay(pdMS_TO_TICKS(10));
              }
            }
          }
        }
        else
        {
          rxprintln("Rx err");
        }
      }

      // Common exit: restore 6-byte receive mode and clear stale interrupt flag.
      // The GPS announcement path uses 'continue' and never reaches here.
      radio.implicitHeader(6);
      radio.startReceive();
      rfInterrupt = false;
    }
    else
    {
      // ============================================================
      // Feature C - SX1262 self-heal on the semaphore-timeout branch
      // ============================================================
      // V2.5-Evo - 2026-07-14 - Feature C: re-arm the radio when the 2000ms semaphore times out.
      // The bug: the took-semaphore branch above re-arms the SX1262 (implicitHeader(6)+startReceive)
      // on every packet, but the timeout branch previously did NOTHING except loop back to feed the
      // WDT. If the SX1262 wedges or drops its DIO IRQ, no packet ever fires the semaphore again, so
      // the receiver is never re-armed and the link stays dead until a power-cycle.
      // The fix: on timeout, re-run the same re-arm sequence used at the common exit so a wedged
      // radio self-heals. startReceive() resets the FIFO/IRQ state, so it doubles as the buffer flush.
      // Motor-safety: this runs only after 2000ms of RF silence, by which point the RX failsafe has
      // long since zeroed the motor (PWM.ino:13 gates on millis()-last_packet). This branch never
      // writes thr_received, last_packet, or any PWM state — motor-safe by construction.
      radio.implicitHeader(6);
      radio.startReceive();
      rfInterrupt = false;
    }
  }
}

// getLinkQuality() is now in ../Common/RadioCommon.h
