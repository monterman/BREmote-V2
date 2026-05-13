// V2.5-Evo - 2026-05-12 - Fix Phase B recovery: recheck gate reduces from 30s to 2s when gps_phase_b_ok=false, eliminating up to 30s RTM motor block after any TX GPS gap
﻿// V2.5-Evo - 2026-05-03 - Removed if(0) dead code; checkAndAdjustAddress() TODO noted
// V2.5-Evo - 2026-04-29 - Bundle C: startTransmit() return value checked and logged on error
// V2.5-Evo - 2026-04-24 - Added GPS meta-packet reception: gps_meta_pending state, processMetaGpsPacket(), triggeredReceive() 2-path state machine
// V2.5-Evo - 2026-04-24 - Added Phase B GPS handshake check: gpsPhaseBCheck() called from processMetaGpsPacket()
// V2.5-Evo - 2026-04-25 - P7: Added processRtmStatePacket(), processFmOverridePacket(); dispatch 0xF1/0xF2 in triggeredReceive()

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
// Updates runtime FM mode without writing SPIFFS. Resets to 0xFF (use SPIFFS) on RX reboot.
static void processFmOverridePacket(const uint8_t *pkt)
{
  uint8_t mode = pkt[4] & 0x03;  // clamp to 0-3
  fm_mode_runtime = mode;
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

  rx_tx_gps_lat       = (double)lat_ud / 1e6;
  rx_tx_gps_lng       = (double)lng_ud / 1e6;
  rx_tx_gps_timestamp = millis();

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
    esp_task_wdt_reset();
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

          if (memcmp(rcvArray, usrConf.own_address, 3) == 0)
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

                telemetry.link_quality = getLinkQuality(radio.getRSSI(), radio.getSNR());

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
  }
}

// getLinkQuality() is now in ../Common/RadioCommon.h
