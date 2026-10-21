// V2.5-Evo - 2026-10-21 - E1-E4 (owner order): waitForTelemetry() reads the number of the remote in control from a reply at
//   index 20 (new-style link only; never stored as telemetry) and opens the REFUSED window with it. No confStruct change.
// V2.5-Evo - 2026-10-21 - text only: the PAIR offer line says the -60 dBm gate is about 5 m (calculated), not about 1 m.
// V2.5-Evo - 2026-10-21 - STEP-3 AUDIT P3-6 d: waitForTelemetry() reads local_link_quality from this remote's accepted reply
//   only, not from every frame seen in the 40 ms window. No confStruct change.
// V2.5-Evo - 2026-10-20 - MULTI-REMOTE REMOTE STEP (SW29), the radio side (design v3.1 sections 5, 9, 11; audit C2, C14, C16-C18,
//   W-2, W-7): (1) EVERY UPLINK IS BUILT BY mtxBuildUplink() (Common/MultiTx.h, host test T15): an old-style record
//   (mtx_link_proto 0, every migrated pairing) gives today's bytes exactly - the buggy's address, the same type and value,
//   LOCKED as control 0/127 - and a new-style record (3) puts the link ID in bytes 0-2 and sends LOCKED as 0xF1/0x10;
//   the take-back and the summon leave the queue as internal types the builder turns into 0xF1/0x5A and 0xF1/0x3C
//   (proto 3 only). (2) THE RATE (C16): on a new-style link a remote that has lost control and is not being used sends
//   one cycle in ten; a miss there shifts the next send by 33 or 66 ms only. (3) THE REPLY WAIT IS BOUNDED (C17): 40 ms
//   from the send, someone else's frame re-arms the receive, and no radio call is made after the bound; the reply's
//   status (new-style link) is recorded in g_mtx_tx. (4) PAIRING (C14, W-2, C10): 0xAD + this chip's ID, an OFFER only
//   at -60 dBm or stronger, -9 dBm while pairing with the stored power back on every exit (explicitly BEFORE the paired
//   store on success, and never as SPI to a sleeping radio), the record written with paired last. (5) W-7: enabling the
//   radio configures it and sets the stored power BEFORE the flag. (6) C18: the PRNG is seeded from this chip's ID.
//   (7) The rider-GPS stream goes out only while this remote is in control (old-style link: always, as today).
//   No confStruct change in this file.
// V2.5-Evo - 2026-10-12 - audit 2026-10-11 delta A-1: sendData() sends the v2 rider GPS packet - announcement 0xF3/0x03,
//   then 15 bytes with this remote's GPS FIX COUNTER (Common/RiderGpsPacket.h riderGpsTxFixStep / riderGpsBuildV2), so the
//   buggy can tell a still rider with a good fix from a dead GPS. Same 2 Hz, same 12.864 ms on air as the 14-byte v1.
//   Flash with the matching RX (an old RX rejects v2: no rider GPS, fail-safe). No confStruct change, SW_VERSION stays 27.
// V2.5-Evo - 2026-10-11 - audit 2026-10-11 D-3: the Follow-Me fault edge also captures fm_flags bit 0 from the SAME arrival (fm_fault_latched_armed), written before the latch. No confStruct change, SW_VERSION stays 27.
// V2.5-Evo - 2026-10-09 - comment only (audit "magnet station step" L-4): the fm_engaged_streak note no longer says
//   the magnet tap needs it (the tap gates on fm_armed since 2026-10-09). No code change.
// V2.5-Evo - 2026-10-07 - T-3: an rx_state_flags (index 19) end bit counts only when set on 2 consecutive arrivals.
//   T-5: meta cycles advance gps_cycle (capped at 4) so GPS stays at 2 Hz through meta packets. T-8: sendData()
//   stamps tx_boot_id_sent_ms when a boot ID goes on the air. No packet format change, no confStruct change.
// V2.5-Evo - 2026-10-07 - SOP-041 rule 3 resync: the fm_flags unpack counts arrivals with bit 0 (armed) clear.
// V2.5-Evo - 2026-10-07 - Q-9: sendData() holds the 100 ms cadence (no collision backoff) while the throttle input is
//   stale or faulted, so the zeroed packets go out at full rate. Can only make the zero arrive sooner.
// V2.5-Evo - 2026-10-07 - TX protocol round: queueMetaPacketIfFree() / metaQueuePending() for the boot ID (S-8) and the
//   RTM refresh (H-1) - free slot only, never update or evict a pending burst; sendData() stamps rtm_start_sent_ms on
//   every 0xF1/1 and keeps at most 3 non-control cycles in a row (meta budget); waitForTelemetry() latches the rising
//   edges of rx_state_flags (index 19) bits 0/1. No packet format change on the remote's side of the link.
// V2.5-Evo - 2026-10-07 - A-1: waitForTelemetry() records each fm_status arrival's RTM bit (fm_status_rtm_on_ms,
//   fm_status_rtm_off_streak) so the remote can follow the buggy ending RTM. No packet format change.
// V2.5-Evo - 2026-10-07 - P-8: the meta queue's second slot follows an explicit priority 0xF1 > 0xF2 > 0xF4; a burst
//   never evicts an equal- or higher-priority one. No packet format change.
// V2.5-Evo - 2026-10-07 - P-1: comment only - sendData()'s input-fault check zeroes the current packet and no longer latches.
// V2.5-Evo - 2026-10-07 - H-1 / L-1: the meta-packet queue is 2 deep (same type updates in place; 0xF1 has priority
//   for the second slot); sendData() stamps rtm_stop_sent_ms on every 0xF1/0 sent; waitForTelemetry() stamps
//   fm_status_arrival_ms. No packet format change.
// V2.5-Evo - 2026-10-07 - C-1: sendData() sends zero throttle and centred steering when adsInputFaultNow() reports a
//   dead or implausible throttle input (no ADS1115 conversion for 50 ms). Can only lower throttle. No format change.
// V2.5-Evo - 2026-10-07 - R-6: the telemetry unpack also latches the Follow-Me fault-stop rising edge (fm_flags bit 3)
//   into fm_fault_latched on the arrival of the byte, so runFmLoop() can act on it even after a long loop() stall
//   (the blocking RTM arm ceremony). Flag only - no packet format change, no confStruct change, sizeof stays 136.
// V2.5-Evo - 2026-09-30 - MagFix (delta audit): the telemetry unpack in waitForTelemetry() now counts
//   consecutive arrivals of the fm_flags byte that claim Follow-Me ARMED + ENGAGED (fm_engaged_streak).
//   fmIsEngaged() requires two of them before it lets a magnet tap move a Follow-Me station, so the
//   "never reposition the buggy while the rider is on the rope" rule no longer rests on a single
//   CRC8-protected bit. Counter only - no packet format change, no confStruct change, sizeof stays 136.
// V2.5-Evo - 2026-05-03 - Added reserved/warning comments (LOW audit cleanup)
// V2.5-Evo - 2026-04-24 - Added 0xF3 GPS meta-packet burst at 2Hz in sendData(); THR capped at 0xF2
// V2.5-Evo - 2026-04-25 - P7: Added RTM/FM meta-packet queue consumer in sendData(); cap 0xF2→0xF0; queueMetaPacketBurst()
// V2.5-Evo - 2026-05-13 - SW32 L3: stale checkAndAdjustAddress TODO block removed (function never implemented)
// V2.5-Evo - 2026-05-13 - SW32 M3: queueMetaPacketBurst uses release/relaxed; sendData consumer uses acquire load
// V2.5-Evo - 2026-04-29 - Bundle A: radio_preset max clamped to 2; dead foil_speed != 99 sentinel removed
// V2.5-Evo - 2026-10-20 - audit W-7 and design 11.4: (W-7) THE FLAG IS SET LAST ON THE WAY UP. It used to be published
//   first, so the sendData task could transmit while the radio was still being reconfigured below; now the radio is
//   configured, then the flag lets sendData use it. On the way down the flag still goes first (sendData stops using the
//   radio before it sleeps). (11.4) The stored rf_power is set again on every enable: the pairing guard's -9 dBm
//   survives the SX126x's sleep, and a pairing attempt that ended with the radio asleep could not restore it (the
//   guard never sends SPI to a sleeping radio). Same value as initRadioHardware() set at boot, so nothing changes on air.
void setRadioActivityEnabled(bool enabled)
{
  if(!enabled)
  {
    radio_activity_enabled = false;
    if(!radio_driver_ready) return;
    rfInterrupt = false;
    radio.sleep();
    return;
  }

  if(!radio_driver_ready) { radio_activity_enabled = true; return; }

  radio.setDio2AsRfSwitch(true);
  radio.setCRC(0);
  radio.setRxBandwidth(250);
  radio.setOutputPower((int8_t)usrConf.rf_power);   // design 11.4: the stored power, every time
  radio.implicitHeader(6);
  radio.startReceive();
  radio_activity_enabled = true;                    // W-7: published last
}

bool isRadioActivityEnabled()
{
  return radio_activity_enabled;
}

void radioErrorHalt(int type)
{
  if(type == 1) while(1) scroll4Digits(LET_E, LET_H, LET_F, LET_P, 200);
  if(type == 2) while(1) scroll4Digits(LET_E, LET_H, LET_F, LET_C, 200);
  while(1) scroll4Digits(LET_E, LET_H, LET_F, LET_I, 200);
}

void radioInitSuccess()
{
  radio_driver_ready = true;
  setRadioActivityEnabled(radio_activity_enabled);
}

void startupRadio()
{
  initRadioHardware();
}

void ICACHE_RAM_ATTR packetReceived(void) 
{
  // we sent or received a packet, set the flag
  rfInterrupt = true;
}

// ============================================================
// V2.5-Evo - 2026-10-20 - PAIRING, THE REMOTE'S HALF (design v3.1 11.2, 11.4; audit C10, C14, W-2)
// ============================================================
// RIGHT held at power-on (checkStartupButtons() -> checkPairing()) calls this until it succeeds or the radio is turned
// off. A NEW-STYLE REQUEST EVERY TIME: [0xAD][this chip's ID 3][CRC 0-3]. A buggy on the multi-remote firmware answers
// [0xDA][this remote's ID 3][its own ID 3][CRC 0-6]; an older buggy ignores 0xAD, so this remote shows "EP" until its
// pre-flash backup is restored (no ?pairlegacy, design Q8). The OFFER is taken only if it names this remote, carries a
// non-zero buggy ID and arrives at -60 dBm or stronger (mtxPairOfferOk(), host test T13); a weaker one is ignored with
// one line and the remote keeps listening. On an OFFER: [0xAE][the buggy's ID 3][this chip's ID 3][CRC 0-6], three
// times, 300 ms apart (the buggy enrols on the first), then the record: dest_address = the buggy's ID, own_address =
// this chip's ID, mtx_link_proto = 3, and paired = 1 LAST (V-11: the sendData task starts transmitting the moment it
// reads paired, so everything it needs is in place before).
// THE POWER (C14): -9 dBm for the whole attempt, from TxPairPowerGuard, armed after the entry check so that exit never
// touches the power. The stored rf_power comes back on every other exit:
//   - SUCCESS (W-2): explicitly, BEFORE the record is written - after the paired store the sendData task owns the radio,
//     and an SPI write from here could interleave with its transmit. Nothing in this function touches the radio after it.
//   - the timeout: the guard's destructor, the radio being awake.
//   - the two "radio switched off during the attempt" exits: the destructor sees the radio asleep and does NOTHING (an
//     SPI write would wake the SX126x into standby and draw current on a locked remote); setRadioActivityEnabled(true)
//     puts the stored power back when the radio is next switched on (the -9 dBm survives the sleep).
//   Today the two radio-off exits cannot even be reached (every caller of setRadioActivityEnabled() runs on the loop task,
//   which is the task blocked here); the rule is there for the day one is.
// TxPairPowerGuard - RAII for the pairing power. ok: -9 dBm was really set (a refusal ends the attempt at once, the radio
//   keeps the stored power). restoreNow(): the success path's explicit restore; the destructor then writes nothing.
struct TxPairPowerGuard {
  bool    ok;
  bool    done;
  int16_t err;
  TxPairPowerGuard() : done(false) { err = radio.setOutputPower((int8_t)kMtxPairPowerDbm); ok = (err == RADIOLIB_ERR_NONE); }
  void restoreNow() { radio.setOutputPower((int8_t)usrConf.rf_power); done = true; }
  ~TxPairPowerGuard() { if (!done && isRadioActivityEnabled()) radio.setOutputPower((int8_t)usrConf.rf_power); }
};

// initiatePairing - one pairing attempt of up to PAIRING_TIMEOUT (5 s), a request per second. Returns: true once paired
//   (the record is in usrConf; checkPairing() saves it). Side effects: the radio (power, header length, mode),
//   usrConf.paired / dest_address / own_address / mtx_link_proto, g_mtx_tx reset on success, Serial.
bool initiatePairing()
{
  if(!isRadioActivityEnabled()) return false;     // exit 1: before the guard - the power is never touched

  TxPairPowerGuard power_guard;                   // -9 dBm from here (C14)
  if (!power_guard.ok)
  {
    Serial.printf("PAIR: the radio REFUSED the pairing power of %d dBm (error %d) - not paired; try again\n",
                  (int)kMtxPairPowerDbm, (int)power_guard.err);
    return false;                                 // the destructor puts the stored power back
  }

  rxprintln("Initiating Pairing...");
  usrConf.paired = false;

  uint8_t pairingPacket[8];  // the request (5 B) and the confirmation (8 B)
  unsigned long startTime = millis();

  // The new-style request: 0xAD + this chip's ID (design 5.6). own_address is no longer sent - it is not the identity.
  pairingPacket[0] = 0xAD;
  memcpy(pairingPacket + 1, g_efuse_id, 3);
  pairingPacket[4] = esp_crc8(pairingPacket, 4);

  while (millis() - startTime < PAIRING_TIMEOUT)
  {
    if(!isRadioActivityEnabled()) return false;   // exit 2: radio asleep - the guard writes nothing
    unsigned long responseTime = millis();
    rxprintln("Sending pairing request packet: ");
    #ifdef DEBUG_RX
    printHexArray(pairingPacket,5);
    #endif
    // Send pairing request
    radio.implicitHeader(5);
    {
      int16_t _txErr = radio.startTransmit(pairingPacket, 5);
      if (_txErr != RADIOLIB_ERR_NONE)
        Serial.printf("[Radio] startTransmit error %d at line %d\n", _txErr, __LINE__);
    }
    delay(10);
    radio.implicitHeader(8);
    radio.startReceive();
    rfInterrupt = false;
    // Wait for response
    uint8_t responseBuffer[15];

    while(millis() - responseTime < 1000)
    {
      if(!isRadioActivityEnabled()) return false; // exit 3: radio asleep - the guard writes nothing
      while(!rfInterrupt && millis() - responseTime < 1000) delay(10);
      delay(10);

      if (rfInterrupt && radio.readData(responseBuffer, 15) == RADIOLIB_ERR_NONE)
      {
        rfInterrupt = false;
        const float rssi = radio.getRSSI();
        rxprintln("Received response");
        // The OFFER for THIS remote: [0xDA][this chip's ID][the buggy's ID][CRC 0-6]. Anything else is ignored.
        if (responseBuffer[0] == 0xDA && responseBuffer[7] == esp_crc8(responseBuffer, 7) &&
            mtxIdEqual(responseBuffer + 1, g_efuse_id))
        {
          if (!mtxPairOfferOk(responseBuffer + 1, g_efuse_id, responseBuffer + 4, rssi))
          {
            // C10: the RSSI gate (or a zero buggy ID). Keep listening; the remote is moved closer and tries again.
            Serial.printf("PAIR: offer from %02X:%02X:%02X ignored - RSSI %.0f dBm below %d (bring the remote closer: pair at arm's length; the gate is about 5 m, calculated)\n",
                          responseBuffer[4], responseBuffer[5], responseBuffer[6], (double)rssi, (int)kMtxPairMinRssiDbm);
            continue;
          }
          uint8_t rx_id[3];
          memcpy(rx_id, responseBuffer + 4, 3);

          // The confirmation: [0xAE][the buggy's ID][this chip's ID][CRC 0-6], three times.
          pairingPacket[0] = 0xAE;
          memcpy(pairingPacket + 1, rx_id, 3);
          memcpy(pairingPacket + 4, g_efuse_id, 3);
          pairingPacket[7] = esp_crc8(pairingPacket, 7);

          delay(100);
          rxprintln("Sending response: ");
          #ifdef DEBUG_RX
          printHexArray(pairingPacket, 8);
          #endif
          radio.implicitHeader(8);
          for(int i = 0; i < 3; i++)
          {
            {
              int16_t _txErr = radio.startTransmit(pairingPacket, 8);
              if (_txErr != RADIOLIB_ERR_NONE)
                Serial.printf("[Radio] startTransmit error %d at line %d\n", _txErr, __LINE__);
            }
            delay(300);
          }

          // W-2: the stored power back NOW, before the record - after `paired` the radio belongs to sendData.
          power_guard.restoreNow();
          // The record, in one critical section, paired LAST (V-11). A new link starts with no status heard.
          portENTER_CRITICAL(&g_mtx_tx_mux);
          memcpy(usrConf.dest_address, rx_id, 3);
          memcpy(usrConf.own_address, g_efuse_id, 3);
          usrConf.mtx_link_proto = kMtxProtoV3;
          mtxTxStatusInit(&g_mtx_tx);
          mtx_tx_events = 0;
          std::atomic_signal_fence(std::memory_order_seq_cst);
          usrConf.paired = true;
          portEXIT_CRITICAL(&g_mtx_tx_mux);
          // Nothing below touches the radio.
          Serial.printf("PAIR: paired with buggy %02X:%02X:%02X (new-style link), offer RSSI %.0f dBm\n",
                        rx_id[0], rx_id[1], rx_id[2], (double)rssi);
          return true;
        }
      }
      else rfInterrupt = false;
    }
  }
  return false;                                   // exit 5: the timeout - the guard puts the stored power back
}

void checkPairing()
{
  if(!isRadioActivityEnabled())
  {
    Serial.println("Radio activity is disabled, pairing skipped.");
    return;
  }

  if(!usrConf.paired)
  {
    Serial.println("Not Paired!");
    while(!usrConf.paired && isRadioActivityEnabled())
    {
      displayDigits(LET_E, LET_P);
      updateDisplay();
      uint8_t pair_animation = 0;
      while(!tog_input)
      {
        pair_animation++;
        if(pair_animation > 10) pair_animation = 0;
        if(pair_animation > 8)
        {
          displayDigits(TLT, TLT);
        }
        else
        {
          displayDigits(LET_E, LET_P);
        }
        updateDisplay();
        delay(100);
      }
      displayDigits(LET_P, LET_A);
      updateDisplay();
      initiatePairing();
    }
    if(!isRadioActivityEnabled())
    {
      Serial.println("Pairing aborted because radio activity was disabled.");
      return;
    }
    Serial.println("Pairing Done.");
    
    Serial.print("Own Address: ");
    for (int i = 0; i < 3; i++) {
        Serial.print(usrConf.own_address[i], HEX);
        Serial.print(i < 2 ? ":" : "\n");
    }

    Serial.print("Destination Address: ");
    for (int i = 0; i < 3; i++) {
        Serial.print(usrConf.dest_address[i], HEX);
        Serial.print(i < 2 ? ":" : "\n");
    }

    if(usrConf.paired == true)
    {
      scroll4Digits(5, LET_A, LET_V, LET_E, 120);
      scroll4Digits(5, LET_A, LET_V, LET_E, 120);
      saveConfToSPIFFS(usrConf);
    }
  }
}


// V2.5-Evo - 2026-04-24 - Added 0xF3 GPS meta-packet burst at 2Hz for Phase B anti-spoofing.
//                   THR capped at 0xF2: 0xF3 is reserved as the GPS meta-packet marker.
// V2.5-Evo - 2026-07-14 - Feature A: adaptive RF collision backoff (adapted from Ludwig 2.2.7).
// V2.5-Evo - 2026-10-07 - H-1 / L-1: meta queue consumer, defined further down this file.
static bool metaQueueTake(uint8_t &type, uint8_t &value);
void sendData(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // GPS meta-packet cycle counter. Incremented each control cycle.
  // Wraps at 5 → resets to 0. The GPS meta-packet fires on the cycle where it resets to 0
  // (i.e., every 5 control cycles = 2Hz at the 100ms base cadence).
  static uint8_t gps_cycle = 0;

  // ---------------------------------------------------------------------
  // V2.5-Evo - 2026-10-07 - META BUDGET: AT MOST 3 NON-CONTROL CYCLES IN A ROW
  // Every meta packet (0xF1/0xF2/0xF4) and every GPS meta cycle (0xF3) replaces a control packet, and the buggy
  // holds the last throttle command until the next one arrives. The 2-deep queue can hold two 3-packet bursts, and
  // a GPS cycle can fall right after them, so the control stream could pause for 600-700 ms. THE RULE: after 3
  // consecutive non-control cycles (300 ms) the next cycle is ALWAYS a control packet; the pending meta / GPS
  // packet simply goes one cycle later. Counted here, reset by every control packet.
  // ---------------------------------------------------------------------
  static uint8_t non_control_run = 0;
  const uint8_t  kMaxNonControlRun = 3;

  // -----------------------------------------------------------------------
  // Feature A — adaptive RF collision backoff (adapted from Ludwig 2.2.7).
  // Identifier names kept 1:1 with Ludwig for cross-fork diffability.
  //   base_interval         - nominal control cadence in ms (100 normal, 200 when degraded)
  //   consecutive_misses    - run of cycles with no telemetry reply (drives the 100->200 drop)
  //   consecutive_successes - run of clean replies at 200ms (drives the 200->100 recovery)
  // -----------------------------------------------------------------------
  static uint32_t base_interval         = 100;
  static uint8_t  consecutive_misses    = 0;
  static uint8_t  consecutive_successes = 0;

  // Feature A req#4: seed the PRNG per-unit from own_address so two identical units do NOT produce
  // the same jitter sequence — a shared seed de-syncs nothing. own_address is persistent+unique.
  // V2.5-Evo - 2026-10-20 - C18 (design 9.14): seeded from THIS CHIP'S ID (g_efuse_id) instead. own_address is a
  // config value and a copied config copies it, so two remotes with one config hopped in step; the chip ID cannot be
  // copied. On a remote whose stored own_address is its chip ID (the default bake) the seed is the same number.
  // NOTE (HW RNG dependency): if the ESP32-C3 Arduino core's random() already draws from esp_random()
  // (hardware RNG), this seed is redundant-but-harmless; if random() uses the newlib PRNG, this
  // per-unit seed is what actually makes two colliding TXs de-correlate.
  randomSeed(((uint32_t)g_efuse_id[0] << 16) |
             ((uint32_t)g_efuse_id[1] << 8)  |
              (uint32_t)g_efuse_id[2]);

  // V2.5-Evo - 2026-10-20 - C16: the slow rate's cycle counter (mtxTxCycleSends()). sendData task only.
  static uint8_t slow_cycle = 0;

  while(1)
  {
    // Feature A — per-cycle one-shot slot-jitter (0/33/66ms); 0 = no jitter this cycle.
    uint32_t extra_delay = 0;

    // Feature A req#3 — suspend the backoff while an autonomous mode is active (FM armed or RTM
    // active). The design requires GPS meta-packets >=2Hz during active FM steering, and the
    // every-5th-cycle GPS meta only stays at 2Hz while the cadence is a flat 100ms. Holding
    // base_interval at 100 and applying no jitter here preserves the >=2Hz meta floor (§12 rules
    // 1-4: the collision heuristic must never starve the FM/anti-spoofing data path).
    // Trade-off: two units both in FM/RTM will not de-sync until the mode disarms — the floor wins.
    // isFmArmed() (accessor) used instead of raw fm_armed: fm_armed lives in RTMState.ino, which
    // Arduino concatenates AFTER Radio.ino, so the raw variable is not yet declared here.
    // V2.5-Evo - 2026-10-07 - Q-9: THE GAP - at the degraded 200 ms cadence a dead throttle input took up to ~320 ms to
    // reach the buggy as a zero (vs ~250 ms at 100 ms). THE FIX: while adsInputFaultNow() reports the input stale or
    // faulted, the backoff is suspended exactly as during FM/RTM, so every zeroed packet goes out at the 100 ms cadence.
    // The first zeroed packet can still be one degraded cycle away: worst case about 50 ms (deadline) + 200 ms.
    bool backoff_allowed = !(isFmArmed() || rtm_tx_active.load(std::memory_order_relaxed) || adsInputFaultNow());
    if (!backoff_allowed)
    {
      base_interval         = 100;
      consecutive_misses    = 0;
      consecutive_successes = 0;
    }
    if(usrConf.paired && isRadioActivityEnabled())
    {
      // ---- V2.5-Evo - 2026-10-20 - MULTI-REMOTE: this cycle's link record and rate (design 5.3, 9.4; C16) ----
      // Every frame below is built from the link record by mtxBuildUplink(): an old-style record (proto 0) gives
      // today's bytes exactly. On a new-style link the remote that has LOST control (3 non-owner replies or 2 SEIZED)
      // and is not being used goes SLOW - one cycle in ten transmits, the rest are skipped whole (no send, no reply
      // wait, not scored) - so a spare costs the owner and the buggy almost no airtime. It is fast again at once on
      // an owner status, FREE plus input, a pending meta, a take-back or summon gesture, or 5 s with no reply
      // (mtxTxFast()). An old-style link is never slow. The rider-GPS stream is an owner-only action (design 9.5): on a
      // new-style link it goes out only while the latest status says this remote controls (OWNER or OWNER_SOLO).
      MtxLinkRec link;
      txLinkRec(&link);
      bool slow      = false;
      bool stream_ok = true;
      if (link.proto == kMtxProtoV3)
      {
        MtxTxStatus st;
        mtxTxStatusGet(&st);
        slow      = !mtxTxFast(&st, millis(), rtm_meta_count.load(std::memory_order_acquire) > 0, mtx_gesture_armed, mtx_input_ms);
        stream_ok = st.owner;
      }
      mtx_tx_slow = slow;
      if (slow)
      {
        // The degraded 200 ms cadence and its counters do not apply to a slow sender (V-10: never the 100 -> 200 step).
        base_interval         = 100;
        consecutive_misses    = 0;
        consecutive_successes = 0;
      }
      if (!mtxTxCycleSends(slow, &slow_cycle))
      {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));   // a skipped slow cycle: nothing on the air
        continue;
      }

      // ---- Meta-packet burst path (highest priority, preempts GPS and control packets) ----
      // Sends one 6-byte meta-packet per iteration until count reaches 0.
      // 3 bursts × 100ms cycle = 300ms total. Type/value written before count by loop task.
      // V2.5-Evo - 2026-05-13 - SW32 M3: acquire load on count pairs with release store in
      // queueMetaPacketBurst(); guarantees type/value are visible before count reads as >0.
      // V2.5-Evo - 2026-10-07 - H-1 / L-1: the packet now comes from the 2-deep queue (metaQueueTake(), which
      // also counts it off). Same one-packet-per-cycle cadence as before.
      uint8_t meta_type = 0, meta_value = 0;
      // V2.5-Evo - 2026-10-20 - the packet comes from the builder (C2): proto 0 -> [dest][type][value][CRC] exactly as
      // before; proto 3 -> the link ID in bytes 0-2; the internal SEIZE / SUMMON types -> 0xF1/0x5A / 0xF1/0x3C, proto 3
      // only. A packet the builder refuses (SEIZE / SUMMON on an old-style record) is dropped and this cycle sends control.
      uint8_t metaPkt[6];
      if (non_control_run < kMaxNonControlRun && metaQueueTake(meta_type, meta_value) &&   // V2.5-Evo - 2026-10-07 - meta budget
          mtxBuildUplinkFromQueue(meta_type, meta_value, &link, metaPkt, esp_crc8))
      {

        rxprint("RTM meta-pkt: ");
        #ifdef DEBUG_RX
        printHexArray(metaPkt, 6);
        #endif

        radio.implicitHeader(6);
        {
          int16_t _txErr = radio.startTransmit(metaPkt, 6);
          if (_txErr != RADIOLIB_ERR_NONE)
            Serial.printf("[Radio] startTransmit error %d at line %d\n", _txErr, __LINE__);
        }
        tx_last_send_ms = millis();   // V2.5-Evo - 2026-10-20 - C17: the reply window starts here
        // V2.5-Evo - 2026-10-07 - H-1: remember when an "RTM off" went on the air, so the loop task can tell a
        // fresh "buggy still in RTM" report from the cached one that predates this stop.
        if (meta_type == 0xF1 && meta_value == 0) rtm_stop_sent_ms = millis();
        // V2.5-Evo - 2026-10-07 - Q-2 / H-1: and when an "RTM on" did, so the loop task knows when the 0xF1/1 burst
        // has drained (its confirmation count and the refresh start are timed from the last one).
        if (meta_type == 0xF1 && meta_value == 1) rtm_start_sent_ms = millis();
        // V2.5-Evo - 2026-10-07 - T-8: and when a boot ID did (0xF1 value 0x80|id), so txBootIdTick() can tell a boot ID
        // that really went out from one a state burst overwrote in the queue before it was sent.
        if (meta_type == 0xF1 && (meta_value & 0x80)) { const unsigned long t = millis(); tx_boot_id_sent_ms = (t != 0) ? t : 1; }
        non_control_run++;   // V2.5-Evo - 2026-10-07 - meta budget, see below
        // V2.5-Evo - 2026-10-07 - T-5: THE BUG - gps_cycle advanced only on non-meta cycles, so every meta packet pushed
        // the next GPS packet back by 100 ms (about 1.8 Hz during an active return with its once-a-second refresh).
        // THE FIX: a meta cycle advances the GPS counter too, but never past 4: a GPS packet that falls due on a meta
        // cycle goes out on the next free cycle instead of being skipped.
        if (gps_cycle < 4) gps_cycle++;
        num_sent_packets++;
        vTaskDelay(pdMS_TO_TICKS(10));
        radio.implicitHeader(6);
        rfInterrupt = false;
        radio.startReceive();
        xTaskNotifyGive(triggeredWaitForTelemetryHandle);
        // Feature A req#3 — meta-packet bursts (RTM/FM/aux) always run at the base 100ms cadence.
        // Bursts get no telemetry reply (RX does not reply to 0xF1/0xF2/0xF4), so they must never
        // be scored as misses, and RTM/FM bursts must stay prompt — the backoff never slows them.
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
        continue;
      }

      gps_cycle++;
      if (gps_cycle >= 5) gps_cycle = 0;

      // Send GPS meta-packet when: counter reached 0, GPS is enabled in config,
      // and TinyGPS++ reports a valid fix that is not stale.
      // V2.5-Evo - 2026-10-20 - and only while this remote is in control (stream_ok; always true on an old-style link),
      // with the link's own address for both packets (mtxUplinkAddr(): proto 0 -> dest_address, as before).
      uint8_t gps_addr[3];
      const bool gps_addr_ok = mtxUplinkAddr(&link, gps_addr);
      bool send_gps_meta = (gps_cycle == 0)
                        && usrConf.gps_en
                        && gps_tx.location.isValid()
                        && gps_tx.location.age() < usrConf.tx_gps_stale_timeout_ms
                        && stream_ok && gps_addr_ok;
      // V2.5-Evo - 2026-10-07 - meta budget: a GPS cycle due right after 3 non-control cycles waits one cycle
      // (gps_cycle 4 -> the next pass makes it 0 again), so the >= 2 Hz GPS rate loses at most one 100 ms slot.
      if (send_gps_meta && non_control_run >= kMaxNonControlRun)
      {
        send_gps_meta = false;
        gps_cycle     = 4;
      }
      if (send_gps_meta) non_control_run++;
      else               non_control_run = 0;   // this cycle sends a control packet

      if (send_gps_meta)
      {
        // ---------------------------------------------------------------
        // GPS meta-packet burst (replaces one control packet per 500ms)
        //
        // Step 1: 6-byte announcement.
        // Primes RX to switch radio to implicitHeader(14) before the data arrives.
        // byte3=0xF3 is the meta-packet type marker. byte4=0x01 = GPS upcoming.
        // V2.5-Evo - 2026-10-12 - audit delta A-1: byte4 = 0x03 (kRiderGpsAnnounceV2) = a 15-byte v2 GPS packet follows,
        // with this remote's GPS FIX COUNTER (Common/RiderGpsPacket.h). A new buggy switches to implicitHeader(15); an old
        // buggy reads any 0xF3 announcement as v1, listens for 14 bytes and rejects the v2 body (CRC position + subtype),
        // so it has no rider GPS - fail-safe; flash both boards together.
        // ---------------------------------------------------------------
        uint8_t announcePkt[6];
        // V2.5-Evo - 2026-10-20 - built by mtxBuildUplink() (C2): [address][0xF3][0x03][CRC], the address the link's own.
        mtxBuildUplink(MTX_B_META, &link, kRiderGpsType, kRiderGpsAnnounceV2, announcePkt, esp_crc8);   // 0xF3 / 0x03 (was 0x01)

        rxprint("Sending GPS announcement: ");
        #ifdef DEBUG_RX
        printHexArray(announcePkt, 6);
        #endif

        radio.implicitHeader(6);
        {
          int16_t _txErr = radio.startTransmit(announcePkt, 6);
          if (_txErr != RADIOLIB_ERR_NONE)
            Serial.printf("[Radio] startTransmit error %d at line %d\n", _txErr, __LINE__);
        }
        num_sent_packets++;
        vTaskDelay(pdMS_TO_TICKS(10));  // wait for 6-byte TX to complete; RX switches mode during this window

        // ---------------------------------------------------------------
        // Step 2: 14-byte GPS data packet.
        // lat/lng as int32_t microdegrees (degrees × 1e6), little-endian.
        // Precision: ±0.111 m — sufficient for Phase B 500 m distance check.
        // V2.5-Evo - 2026-10-12 - audit delta A-1: NOW 15 BYTES (v2) - [dest 3][0xF3][0x04][lat 4][lng 4][fix counter]
        // [crc8 over 0-13], built by riderGpsBuildV2(). THE BUG IT SERVES: the buggy could only tell a new rider fix from
        // a re-broadcast by a change of coordinates, so a rider lying still looked like a dead GPS and Follow-Me switched
        // itself off after 30 s. The FIX COUNTER moves once per NEW GPS SOLUTION (riderGpsTxFixStep(): a new UTC
        // time-of-fix committed together with the position) - never per transmission, never per position change, and
        // not on sentences that carry a time but no fix. Same airtime as 14 bytes (12.864 ms at SF6/BW250/CR4-7).
        // Read order: the position, then the two ages, then the time - all inside this task, which the loop task (the
        // only TinyGPS writer) cannot interrupt. If this task preempted the parser between its time and position
        // commits, the position is a whole epoch older than the time: no step, and the old position goes out with the
        // old counter - a consistent re-broadcast; the next packet steps.
        // ---------------------------------------------------------------
        static RiderGpsTxFixState tx_fix_state = {false, 0, 0};   // sendData task only
        uint8_t gpsPkt[kRiderGpsLenV2];

        int32_t lat_ud = (int32_t)(gps_tx.location.lat() * 1e6);
        int32_t lng_ud = (int32_t)(gps_tx.location.lng() * 1e6);
        const uint32_t loc_age_ms  = gps_tx.location.age();
        const uint32_t time_age_ms = gps_tx.time.age();
        const uint8_t  fix_ctr     = riderGpsTxFixStep(&tx_fix_state, gps_tx.time.isValid(), gps_tx.time.value(),
                                                       loc_age_ms, time_age_ms, kRiderGpsSameEpochMs);
        riderGpsBuildV2(gpsPkt, gps_addr, lat_ud, lng_ud, fix_ctr, esp_crc8);   // V2.5-Evo - 2026-10-20 - the link's address

        rxprint("Sending GPS data: ");
        #ifdef DEBUG_RX
        printHexArray(gpsPkt, kRiderGpsLenV2);
        #endif

        radio.implicitHeader(kRiderGpsLenV2);
        {
          int16_t _txErr = radio.startTransmit(gpsPkt, kRiderGpsLenV2);
          if (_txErr != RADIOLIB_ERR_NONE)
            Serial.printf("[Radio] startTransmit error %d at line %d\n", _txErr, __LINE__);
        }
        tx_last_send_ms = millis();   // V2.5-Evo - 2026-10-20 - C17: the reply window starts at the last packet
        // 14-byte packet needs slightly more air time than 6-byte at SF6/BW250
        // (V2.5-Evo - 2026-10-12 - A-1: the 15-byte v2 packet takes the same 12.864 ms; the 15 ms wait still covers it.)
        vTaskDelay(pdMS_TO_TICKS(15));
      }
      else
      {
        // ---------------------------------------------------------------
        // Normal 6-byte control packet
        //
        // THR capped at 0xF2 (242): 0xF3 is the GPS meta-packet marker and
        // must never appear in the THR field of a control packet.
        // 0xF2 = 94.9% max throttle — imperceptible difference from uncapped 95.3%.
        // ---------------------------------------------------------------
        // V2.5-Evo - 2026-10-20 - BUILT BY mtxBuildUplink() (C2): CTRL = [address][throttle, clamped at 0xF0][steering]
        // [CRC]; LOCKED = control 0/127 on an old-style record (exactly today's locked packet) and 0xF1/0x10 on a
        // new-style one (the buggy then knows the remote is locked, not just at rest). thr_sent / steer_sent keep
        // their meaning - what the throttle and steering ARE in this packet - so a locked remote reports 0 / 127.
        uint8_t sendArray[6];
        uint8_t thr_out   = 0;
        uint8_t steer_out = 127;

        if(!system_locked)
        {
          uint8_t thr = calcFinalThrottle();
          // V2.5-Evo - 2026-06-05 - C-1: 2nd independent gate — hard-zero throttle during the RTM
          // arm ceremony, regardless of rtm_thr_cap_tx. Both gates must fail to pass throttle while arming.
          if (rtmIsArming()) thr = 0;
          // V2.5-Evo - 2026-10-07 - C-1: frozen-throttle guard at the point of sending. If the ADS1115 has
          // finished no conversion for ADS_STALE_MS (or the input fault is already latched), send zero throttle
          // and centred steering. The ADC task forces the same values, but it can be stuck inside a slow I2C
          // transaction when the bus fails, so the radio checks the deadline itself (adsInputFaultNow(),
          // Analog.ino). Can only zero the throttle, never raise it.
          // V2.5-Evo - 2026-10-07 - P-1: a missed deadline here zeroes THIS packet only; it no longer latches
          // the fault (a CPU stall is not an input failure). The ADC task latches from its own deadline.
          bool input_fault = adsInputFaultNow();
          if (input_fault) thr = 0;
          // V2.5-Evo - 2026-04-25 - P7: cap at 0xF0 (240=94.1%) to reserve 0xF1-0xFF for all meta-packet types.
          // 0xF1=RTM state, 0xF2=FM override, 0xF3=GPS coord. Was 0xF2 cap before P7.
          // 0xF1 and 0xF2 are intentionally reserved packet type bytes — do not assign.
          thr_out   = (thr > 0xF0) ? 0xF0 : thr;
          steer_out = input_fault ? 127 : steer_scaled;
        }

        thr_sent   = thr_out;
        steer_sent = steer_out;

        const bool built = mtxBuildUplink(system_locked ? MTX_B_LOCKED : MTX_B_CTRL, &link, thr_out, steer_out, sendArray, esp_crc8);

        rxprint("Sending: ");
        #ifdef DEBUG_RX
        printHexArray(sendArray, 6);
        #endif

        if (built)   // false only for a link record the validator never lets through (protocol not 0 or 3)
        {
          radio.implicitHeader(6);
          {
            int16_t _txErr = radio.startTransmit(sendArray, 6);
            if (_txErr != RADIOLIB_ERR_NONE)
              Serial.printf("[Radio] startTransmit error %d at line %d\n", _txErr, __LINE__);
          }
          tx_last_send_ms = millis();   // V2.5-Evo - 2026-10-20 - C17: the reply window starts here
          num_sent_packets++;
          vTaskDelay(pdMS_TO_TICKS(10));
        }
      }

      // Common exit for both GPS meta and normal paths:
      // return to 6-byte receive mode and wake waitForTelemetry.
      // After a GPS meta burst, RX sends a normal telemetry reply after processing
      // the GPS data packet — waitForTelemetry will receive it as usual.
      radio.implicitHeader(6);
      rfInterrupt = false;
      radio.startReceive();
      xTaskNotifyGive(triggeredWaitForTelemetryHandle);

      // ---------------------------------------------------------------
      // Feature A req#2/#7 — collision miss/success evaluation.
      // Runs ONLY on a normal control cycle: GPS-meta cycles and RTM/FM/aux bursts get no
      // telemetry reply, so scoring them would false-trigger a miss. backoff_allowed also gates
      // it off during active FM/RTM (req#3, >=2Hz meta floor).
      // The 30ms reply window lets the RX telemetry reply land and update last_packet BEFORE we
      // test it — without it, last_packet would still be from the previous cycle and every cycle
      // would read as a miss (audit caution #1). Window + >40ms threshold match Ludwig 1:1.
      // ---------------------------------------------------------------
      if (!send_gps_meta && backoff_allowed && last_packet > 0)
      {
        vTaskDelay(pdMS_TO_TICKS(30));  // bounded reply window (matches Ludwig's 30ms)

        if (millis() - last_packet > 40 && slow)
        {
          // V2.5-Evo - 2026-10-20 - C16 / V-10: a miss while SLOW shifts the next send by 33 or 66 ms - never 0 (a slow
          // sender must not stay in the owner's slot) and never the 100 -> 200 ms downgrade below. No print: at one
          // send a second it would fill the log.
          extra_delay = mtxTxSlowMissHopMs((uint32_t)random(0, 2));
        }
        else if (millis() - last_packet > 40)
        {
          // === PACKET MISSED === telemetry reply did not land this cycle (collision or obstacle).
          consecutive_misses++;
          consecutive_successes = 0;  // reset success streak

          // After 3 consecutive missed replies, downgrade to the 200ms base cadence.
          if (base_interval == 100 && consecutive_misses >= 3)
          {
            base_interval = 200;
            Serial.println("Change to 200ms interval");
          }

          // Slot-jitter (0/33/66ms). Added via vTaskDelayUntil below, it permanently phase-shifts
          // this TX into a new slot so two colliding units de-sync. random() is per-unit seeded.
          extra_delay = random(0, 3) * 33;
          Serial.print("Added random: ");
          Serial.println(extra_delay);
        }
        else
        {
          // === PACKET RECEIVED CLEANLY ===
          consecutive_misses = 0;

          // If degraded to 200ms, count clean replies and promote back to 100ms after ~50s.
          if (base_interval == 200)
          {
            consecutive_successes++;
            if (consecutive_successes >= 250)
            {
              base_interval = 100;
              consecutive_successes = 0;
              Serial.println("Change to 100ms interval");
            }
          }
        }
      }
    }
    // Feature A — variable cadence = base_interval + one-shot slot-jitter. Because vTaskDelayUntil
    // computes from xLastWakeTime, adding extra_delay permanently shifts this TX's phase into the
    // new slot (the de-sync mechanism). Worst case 200 + 66 = 266ms << 3000ms WDT.
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(base_interval + extra_delay));
  }
}


// ============================================================
// V2.5-Evo - 2026-10-20 - THE REPLY WAIT IS BOUNDED (design 9.12, audit C17) AND READS THE STATUS (design 9.3)
// ============================================================
// THE BUG: after every send this task waited for the radio's interrupt with NO bound. A cycle with no reply (a meta,
// a GPS packet, a lost reply) left it waiting until the next interrupt, which could be the NEXT send's own TX-done - it
// then read a frame while sendData was using the radio. With several remotes on one channel there is also other
// traffic: another remote's frame or reply would end the wait as a mismatch.
// THE FIX: the window is 40 ms from the send (tx_last_send_ms, stamped by sendData). A frame that is not this remote's
// reply (address or CRC wrong) re-arms the receive and the wait goes on; the first matching reply ends it; at the bound
// it stops, and NO radio call is made after it (sendData touches the radio again no earlier than the next cycle, at
// least 100 ms after the send). A frame is judged on the time it was SEEN, so a reply noticed just inside the window is
// kept even if reading it ends just outside. The Feature A 30 ms window and its > 40 ms test are unchanged.
// THE ADDRESS: an old-style record answers to own_address, as before; a new-style one to this chip's ID
// (mtxTxReplyAddr()). THE STATUS (new-style link only): byte 3 = status in bits 7-5, index in bits 4-0; the status goes
// to g_mtx_tx (mtxTxOnReply(): owner / lost owner / the SEIZED notice, edges for the loop), and REFUSED starts the 1.5 s
// E-code window. On an old-style record byte 3 IS the index, read exactly as before.
// V2.5-Evo - 2026-10-21 - E1-E4: on a new-style link a reply at index 20 carries the number of the remote in control
// (mtxTxOnNumber()); the window then shows E1-E4 (E9 while the number is unknown).
void waitForTelemetry(void *parameter)
{
  while (1) 
  {
    //wait until called
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if(!isRadioActivityEnabled()) continue;

    const unsigned long send_ms    = tx_last_send_ms;
    const bool          v3         = txIsV3();
    const uint8_t*      reply_addr = mtxTxReplyAddr(usrConf.mtx_link_proto, g_efuse_id, usrConf.own_address);
    uint8_t rcvArray[6];
    bool    ours = false;
    while (isRadioActivityEnabled() && mtxReplyWaitOpen(millis(), send_ms))
    {
      if (!rfInterrupt) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
      const unsigned long seen_ms = millis();
      const int16_t rd = radio.readData(rcvArray, 6);
      rfInterrupt = false;
      rxprint("Received packet: ");
      #ifdef DEBUG_RX
      printHexArray(rcvArray,6);
      #endif
      const bool match = (rd == RADIOLIB_ERR_NONE) && memcmp(rcvArray, reply_addr, 3) == 0 &&
                         rcvArray[5] == esp_crc8(rcvArray, 5);
      const uint8_t verdict = mtxReplyWaitOnFrame(seen_ms, send_ms, match);
      // V2.5-Evo - 2026-10-21 - P3-6 d: the link quality is read from THIS remote's accepted reply only. It was read from
      // every frame seen in the window, before the match - so on a channel shared with other remotes, another remote's frame
      // or reply (or a corrupted one) set this remote's bars and Pattern A. Same radio call, same moment (straight after the
      // read of that frame), one call fewer for a frame that is not ours.
      if (verdict == MTX_RW_ACCEPT) { local_link_quality = getLinkQuality(radio.getRSSI(), radio.getSNR()); ours = true; break; }
      if (verdict == MTX_RW_STOP) break;
      radio.startReceive();   // someone else's frame, still inside the window: listen again
    }
    if (ours)
    {
      {   // (these two scopes keep the reply processing below at its old indentation; its statements are unchanged
        {   //  apart from reading the index from idx)
          rxprintln("Address matches, CRC ok");
          num_rcv_packets++;

          // The index (and, on a new-style link, the status) of this reply.
          uint8_t idx = rcvArray[3];
          if (v3)
          {
            const uint8_t       s   = mtxReplyStatusOf(rcvArray[3]);
            const unsigned long now = millis();
            idx = mtxReplyIndexOf(rcvArray[3]);
            portENTER_CRITICAL(&g_mtx_tx_mux);
            const uint8_t ev = mtxTxOnReply(&g_mtx_tx, s, now);
            // V2.5-Evo - 2026-10-21 - E1-E4: index 20 is not telemetry - byte 4 is the number of the remote in control
            // (the store below skips it: only indices under 20 are telemetry). A REFUSED reply always carries it, so the
            // E window opens with the number from this very frame; the digit is written before the window's stamp.
            if (idx == kMtxIdxInControl) mtxTxOnNumber(&g_mtx_tx, rcvArray[4], now);
            mtx_tx_events = (uint8_t)(mtx_tx_events | ev);
            if (s == MTX_S_REFUSED)
            {
              mtx_e9_digit = mtxTxRefusalDigit(&g_mtx_tx, now);
              const unsigned long u = now + kMtxE9HoldMs; mtx_e9_until_ms = (u != 0) ? u : 1;
            }
            portEXIT_CRITICAL(&g_mtx_tx_mux);
            // V2.5-Evo - 2026-10-20 - X-8: on EVERY edge to OWNER (001: in control with others paired - never one remote's 000)
            // the boot ID is queued HERE, at once, before the loop can start anything that blocks (RTMState.ino).
            if ((ev & MTX_TXE_BECAME_OWNER) && s == MTX_S_OWNER) txBootIdOwnerEdge();
          }

          uint8_t* ptr = (uint8_t*)&telemetry;  
          if (idx < sizeof(TelemetryPacket))
          {
            ptr[idx] = rcvArray[4];
          }

          // V2.5-Evo - 2026-10-07 - H-1: stamp each ARRIVAL of the fm_status byte (index 15). The cached byte only
          // changes when its index comes round, so runRtmLoop() acts on its RTM bit only when it arrived after
          // the last 0xF1/0 went out.
          if (idx == offsetof(TelemetryPacket, fm_status))
          {
            fm_status_arrival_ms = millis();
            // V2.5-Evo - 2026-10-07 - A-1: count this arrival's RTM bit for runRtmLoop() (see BREmote_V2_Tx.h).
            if (rcvArray[4] & FM_STATUS_RTM_ACTIVE)
            {
              fm_status_rtm_on_ms      = millis();
              fm_status_rtm_off_streak = 0;
            }
            else if (fm_status_rtm_off_streak < 255)
            {
              fm_status_rtm_off_streak = (uint8_t)(fm_status_rtm_off_streak + 1);   // explicit RMW, not ++ on a volatile
            }
          }

          // ---- V2.5-Evo - 2026-09-30 - FOLLOW-ME "ENGAGED" CORROBORATION COUNTER ----
          // V2.5-Evo - 2026-10-09 - audit L-4: the magnet tap no longer uses this streak. Since 2026-10-09 it steps the
          // station in any armed state (fm_armed, Hall.ino / RTMState.ino); fmIsEngaged() and this streak still feed the
          // display (engaged screen, R5 bar) and fmIsReturning(). The history below describes the old tap gate.
          // WHY THIS IS HERE. The magnet tap may only move a Follow-Me station while the buggy is
          // actively following - never while the rider is on the tow rope, where he is attached to the
          // buggy and cannot steer away from it. That rule used to be carried by ONE bit in ONE packet,
          // and a CRC8 lets roughly 1 in 256 random corruptions through, so a single bad-but-valid
          // packet could have opened the gate. fmIsEngaged() (RTMState.ino) now demands the claim on
          // two consecutive arrivals, and this is where they are counted.
          // THIS IS THE ONLY PLACE IT CAN BE COUNTED. The RX sends one telemetry byte per packet and
          // rotates through the indices, so telemetry.fm_flags only changes when its own index arrives.
          // Counting anywhere else - per received packet, or per loop tick - would re-count the same
          // cached byte and corroborate nothing at all.
          // Both bits are required: ENGAGED without ARMED alongside it is not a state the RX sends.
          // Any arrival that does not make the full claim resets the streak to 0, so it always
          // describes the CURRENT claim and never accumulates stale history. It saturates rather than
          // wrapping, because a rollover would read 0 or 1 for a moment and shut the gate on a
          // perfectly healthy link.
          if (idx == offsetof(TelemetryPacket, fm_flags))
          {
            const uint8_t want = (uint8_t)(FM_FLAG_ARMED | FM_FLAG_ENGAGED);
            if ((rcvArray[4] & want) == want)
            {
              // Written as an explicit read-modify-write, not ++: a ++ on a volatile is deprecated in
              // C++20 and compiling with --warnings all would flag it.
              if (fm_engaged_streak < 255) fm_engaged_streak = (uint8_t)(fm_engaged_streak + 1);
            }
            else
            {
              fm_engaged_streak = 0;
            }

            // V2.5-Evo - 2026-10-07 - R-6: latch the Follow-Me FAULT-STOP rising edge HERE, on the arrival
            // of the byte, so it survives any loop() stall (see fm_fault_latched in BREmote_V2_Tx.h).
            // Compared against the previous ARRIVAL of this byte (not against telemetry.fm_flags, which was
            // already overwritten above). runFmLoop() clears the latch when it acts on it.
            static uint8_t fm_flags_prev_arrival = 0;
            if ((rcvArray[4] & FM_FLAG_FAULT) && !(fm_flags_prev_arrival & FM_FLAG_FAULT))
            {
              // V2.5-Evo - 2026-10-11 - D-3: capture bit 0 from THIS arrival, written before the latch is raised, so the
              // reader always sees the armed bit that belongs to the edge it is handling.
              fm_fault_latched_armed = (rcvArray[4] & FM_FLAG_ARMED) != 0;
              fm_fault_latched = true;
            }
            fm_flags_prev_arrival = rcvArray[4];

            // V2.5-Evo - 2026-10-07 - SOP-041 rule 3: count consecutive arrivals with the buggy's ARMED bit clear, and
            // stamp when that run began (see fm_flags_unarmed_since_ms in BREmote_V2_Tx.h; read by runFmLoop()).
            if (rcvArray[4] & FM_FLAG_ARMED)
            {
              fm_flags_unarmed_streak   = 0;
              fm_flags_unarmed_since_ms = 0;
            }
            else
            {
              if (fm_flags_unarmed_streak == 0)
              {
                const unsigned long t = millis();
                fm_flags_unarmed_since_ms = (t != 0) ? t : 1;
              }
              if (fm_flags_unarmed_streak < 255) fm_flags_unarmed_streak = (uint8_t)(fm_flags_unarmed_streak + 1);
            }
          }

          // V2.5-Evo - 2026-10-07 - A-1 / Q-3: index 19 (rx_state_flags) - latch the RISING EDGE of the buggy's two sticky
          // "how Return-To-Me ended" bits on the ARRIVAL of the byte, like the Follow-Me fault edge above. Both bits are
          // held ~6 s by the buggy, so a bit still standing from the previous run is not a new end: only an arrival that
          // shows the bit after one that did not is stamped. runRtmLoop() acts on a stamp later than its own ACTIVE start.
          // An older RX never sends index 19 (it rotates 19 indices), so nothing is ever stamped and the fm_status
          // fallback in runRtmLoop() keeps working exactly as before. The byte itself was stored above (index < 20).
          // V2.5-Evo - 2026-10-07 - T-3: THE BUG - one arrival was enough, so a single corrupted byte (or a bit left over
          // from a run that ended during a link gap) could end the current run: an arrival -> cap 0, a fault -> "St".
          // THE FIX: the bit must be SET ON 2 CONSECUTIVE ARRIVALS of index 19 (the buggy holds it ~6 s, two to three
          // arrivals); the rise is stamped on the second one. A clear arrival resets the count; the count saturates, so
          // a bit that stays set is stamped once. If an edge is missed this way, the fm_status fallback still ends the run.
          if (idx == offsetof(TelemetryPacket, rx_state_flags))
          {
            static uint8_t rx_fault_streak   = 0;   // consecutive arrivals with bit 0 set (this task only)
            static uint8_t rx_arrived_streak = 0;   // consecutive arrivals with bit 1 set (this task only)
            const unsigned long t = millis();
            if (rcvArray[4] & RX_STATE_RTM_FAULT)   { if (rx_fault_streak   < 255) rx_fault_streak++;   } else rx_fault_streak   = 0;
            if (rcvArray[4] & RX_STATE_RTM_ARRIVED) { if (rx_arrived_streak < 255) rx_arrived_streak++; } else rx_arrived_streak = 0;
            if (rx_fault_streak   == 2) rx_rtm_fault_rise_ms   = (t != 0) ? t : 1;
            if (rx_arrived_streak == 2) rx_rtm_arrived_rise_ms = (t != 0) ? t : 1;
          }

          // Speed conversion: RX sends speed in km/h; convert to the unit selected in web config.
          // 0xFF = no GPS data sentinel (V2.5-Evo fix: old V2 sentinel 99 km/h removed — collided with real speed)
          if (idx == 2 && telemetry.foil_speed != 0xFF)
          {
              if (usrConf.speed_src == 1) {
                  // Option 1: GPS RX knots (km/h * 0.539957)
                  telemetry.foil_speed = (uint8_t)(telemetry.foil_speed * 0.539957f);
              } 
              else if (usrConf.speed_src == 4) {
                  // Option 4: GPS RX mph (km/h * 0.621371)
                  telemetry.foil_speed = (uint8_t)(telemetry.foil_speed * 0.621371f);
              }
              // Options 0 (RX km/h), 2 (TX km/h), 3 (TX knots), and 5 (TX mph) 
              // are either untouched here or handled elsewhere by the TX GPS logic.
          }
          // ------------------------------------------

          // V2.5-Evo - 2026-05-15 - E71 fix: bidirectional sync — set on 71, clear when RX clears it.
          // One-way write (only on non-zero) left remote_error latched on TX after RX auto-cleared the alarm.
          if (telemetry.error_code == 71) {
            remote_error = 71;   // E71 water ingress — arm TX display and haptic
          } else if (remote_error == 71) {
            remote_error = 0;    // RX auto-cleared E71 — mirror the clear to TX
          }
          last_packet = millis();
        }
      }
    }
  }
}

// getLinkQuality() is now in ../Common/RadioCommon.h

// V2.5-Evo - 2026-04-25 - P7: Queue a 3-burst meta-packet transmission.
// V2.5-Evo - 2026-05-13 - SW32 M3: explicit memory ordering — type/value stored relaxed
// (count is the guard), count stored with memory_order_release so the sendData-task acquire load
// in sendData() is guaranteed to observe the correct type/value before acting on count>0.
// Called from loop task (RTM/FM state machines in RTMState.ino).
// sendData() FreeRTOS task consumes the queue.
// type: 0xF1=RTM state, 0xF2=FM override, 0xF4=aux control
// value: for 0xF1: 0=inactive 1=active; for 0xF2: 0-3 FM mode; for 0xF4: aux flags byte
// ============================================================
// V2.5-Evo - 2026-10-07 - H-1 / L-1: TWO-DEEP META-PACKET QUEUE
// THE BUG: the queue was ONE slot. Any new burst overwrote the one still going out, so a keepalive or a
// Follow-Me disarm could wipe a just-queued 0xF1 Return-To-Me state change (only the call sites' own "wait
// until empty" checks prevented it), and deepSleep() could not queue "RTM off" and "FM off" together.
// THE FIX: a second slot. Each packet TYPE is a state channel, so the rules are:
//   1. a type already queued (head or second slot) is UPDATED IN PLACE: the newest value wins and its three
//      sends restart (exactly what the single slot did for a repeat of the same type);
//   2. otherwise it goes into the first empty slot, head first;
//   3. if both slots hold OTHER types, the new burst takes the second slot only if its type has a HIGHER
//      priority than the one queued there: 0xF1 > 0xF2 > 0xF4 (metaTypePriority(), V2.5-Evo - 2026-10-07 - P-8).
//      A higher type is never evicted.
// The head is the rtm_meta_* atomics sendData() already read; the second slot is promoted the moment the
// head empties, inside the same critical section, so rtm_meta_count == 0 still means "queue empty" for the
// existing callers that wait for an empty queue.
// Callers: loop task (RTM/FM state machines), possibly the BLE/web tasks via sendAuxCommand(). Consumer:
// sendData task through metaQueueTake(). A critical section (interrupts off for a few instructions on this
// single-core part) makes every queue change atomic.
// ============================================================
static portMUX_TYPE metaQueueMux     = portMUX_INITIALIZER_UNLOCKED;
static uint8_t      meta_next_type   = 0;   // second slot: packet type (0xF1/0xF2/0xF4)
static uint8_t      meta_next_value  = 0;   // second slot: value byte
static uint8_t      meta_next_count  = 0;   // second slot: sends remaining, 0 = empty

// metaTypePriority - V2.5-Evo - 2026-10-07 - P-8: explicit priority of a meta packet type for the second slot.
// 0xF1 (Return-To-Me state) > 0xF2 (Follow-Me declaration / stop) > 0xF4 (aux) > anything else.
// THE BUG: rule 3 below used to let ANY type take the second slot unless it held an 0xF1, so an 0xF4 (aux) or a
// new 0xF1 could evict a queued 0xF2 - including a Follow-Me "off". Latent (there is no 0xF4 caller today), but
// the rule now says what it means: a burst may only replace a LOWER-priority burst, never an equal or higher one.
// Input: type. Output: 3/2/1/0. No side effects.
// V2.5-Evo - 2026-10-20 - the shared table (Common/MultiTx.h mtxMetaTypePriority(), host test T32): the same 3 / 2 / 1 / 0
// for 0xF1 / 0xF2 / 0xF4 / anything else, and 4 for the two internal types (the take-back SEIZE, the SUMMON), which
// have their own queue types so rule 1 below can never merge them with a queued RTM 0xF1.
static uint8_t metaTypePriority(uint8_t type)
{
  return mtxMetaTypePriority(type);
}

void queueMetaPacketBurst(uint8_t type, uint8_t value)
{
  portENTER_CRITICAL(&metaQueueMux);
  const uint8_t head_count = rtm_meta_count.load(std::memory_order_relaxed);
  const uint8_t head_type  = rtm_meta_type.load(std::memory_order_relaxed);
  if (head_count > 0 && head_type == type)
  {
    rtm_meta_value.store(value, std::memory_order_relaxed);   // rule 1, head
    rtm_meta_count.store(3, std::memory_order_release);
  }
  else if (meta_next_count > 0 && meta_next_type == type)
  {
    meta_next_value = value;                                    // rule 1, second slot
    meta_next_count = 3;
  }
  else if (head_count == 0)
  {
    rtm_meta_type.store(type, std::memory_order_relaxed);      // rule 2, head
    rtm_meta_value.store(value, std::memory_order_relaxed);
    rtm_meta_count.store(3, std::memory_order_release);        // release: type/value visible before count
  }
  else if (meta_next_count == 0 || metaTypePriority(type) > metaTypePriority(meta_next_type))
  {
    meta_next_type  = type;                                     // rule 2 / rule 3 (P-8), second slot
    meta_next_value = value;
    meta_next_count = 3;
  }
  // else: both slots busy with other types and the second holds an equal- or higher-priority burst - this
  // burst is dropped. Its senders re-send (the FM keepalive retries, aux is a user action, the H-1 watch
  // re-sends 0xF1/0 on the next report).
  portEXIT_CRITICAL(&metaQueueMux);
}

// ============================================================
// V2.5-Evo - 2026-10-07 - LOWEST-PRIORITY META PACKETS: THE BOOT ID (S-8) AND THE RTM REFRESH (H-1)
// Both are 0xF1 packets, and queueMetaPacketBurst() rule 1 updates a queued burst OF THE SAME TYPE in place. So a boot
// ID or a refresh queued the normal way would REPLACE a pending 0xF1/0 ("RTM off") or 0xF1/1 ("RTM on") and the buggy
// would never hear the state change - the exact failure the 2-deep queue exists to prevent. THE RULE: these two go
// only into a FREE slot, never update or evict anything, and are refused while any burst of their type is pending.
// They repeat on their own schedule (boot ID every 10 s, refresh every 1 s), so a refused one is simply retried.
// Inputs: type, value, count (sends; 1 for the repeating packets - every meta packet replaces a control packet).
// Returns: true if queued. Side effects: the queue only. Any task; critical section like queueMetaPacketBurst().
// ============================================================
bool queueMetaPacketIfFree(uint8_t type, uint8_t value, uint8_t count)
{
  bool queued = false;
  if (count == 0) return false;
  portENTER_CRITICAL(&metaQueueMux);
  const uint8_t head_count = rtm_meta_count.load(std::memory_order_relaxed);
  const uint8_t head_type  = rtm_meta_type.load(std::memory_order_relaxed);
  const bool    same_type  = (head_count > 0 && head_type == type) || (meta_next_count > 0 && meta_next_type == type);
  if (!same_type)
  {
    if (head_count == 0)
    {
      rtm_meta_type.store(type, std::memory_order_relaxed);
      rtm_meta_value.store(value, std::memory_order_relaxed);
      rtm_meta_count.store(count, std::memory_order_release);
      queued = true;
    }
    else if (meta_next_count == 0)
    {
      meta_next_type  = type;
      meta_next_value = value;
      meta_next_count = count;
      queued = true;
    }
  }
  portEXIT_CRITICAL(&metaQueueMux);
  return queued;
}

// metaQueuePending - V2.5-Evo - 2026-10-07 - is a packet of this type (and, if any_value is false, this exact value)
// still waiting to go out in either slot? Used to tell "the 0xF1/1 burst has drained" (Q-2, H-1 refresh start).
// Inputs: type, value, any_value. Output: true = pending. No side effects. Any task.
bool metaQueuePending(uint8_t type, uint8_t value, bool any_value)
{
  bool pending = false;
  portENTER_CRITICAL(&metaQueueMux);
  if (rtm_meta_count.load(std::memory_order_relaxed) > 0 && rtm_meta_type.load(std::memory_order_relaxed) == type &&
      (any_value || rtm_meta_value.load(std::memory_order_relaxed) == value)) pending = true;
  if (meta_next_count > 0 && meta_next_type == type && (any_value || meta_next_value == value)) pending = true;
  portEXIT_CRITICAL(&metaQueueMux);
  return pending;
}

// metaQueueTake - hand sendData() the next meta packet to transmit, if any.
// Outputs: type/value of the packet. Returns true if there is one. Side effects: counts it off the head
// and promotes the second slot when the head empties. Called only by the sendData task.
static bool metaQueueTake(uint8_t &type, uint8_t &value)
{
  bool have = false;
  portENTER_CRITICAL(&metaQueueMux);
  uint8_t c = rtm_meta_count.load(std::memory_order_acquire);
  if (c > 0)
  {
    type  = rtm_meta_type.load(std::memory_order_relaxed);
    value = rtm_meta_value.load(std::memory_order_relaxed);
    have  = true;
    c--;
    if (c == 0 && meta_next_count > 0)
    {
      rtm_meta_type.store(meta_next_type, std::memory_order_relaxed);
      rtm_meta_value.store(meta_next_value, std::memory_order_relaxed);
      rtm_meta_count.store(meta_next_count, std::memory_order_release);
      meta_next_count = 0;
    }
    else
    {
      rtm_meta_count.store(c, std::memory_order_release);
    }
  }
  portEXIT_CRITICAL(&metaQueueMux);
  return have;
}

// Queue a 0xF4 aux control burst to RX (3× for reliability).
// flags: bit0=strobe/light, bit1=horn(reserved), bit2=aux3(reserved), bit3=find-me flash, bits4-7=reserved.
void sendAuxCommand(uint8_t flags) {
  queueMetaPacketBurst(0xF4, flags);
}
