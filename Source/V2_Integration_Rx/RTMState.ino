// V3 - 2026-04-25 - P7: RX RTM state machine, 10 safety gates, Phase C anti-spoofing.
// V3 - 2026-04-27 - P8: runRtmLoop() encodes RX→TX distance into telemetry.rtm_distance (index 5)
// V2.5-Evo - 2026-04-28 - P9 Bug1A/1B/1C: Gate9 zero-guard; always-compute dist before gates
// V2.5-Evo - 2026-04-28 - Security: Gate 1 resets rtm_steer_override=127 on throttle release
// V2.5-Evo - 2026-04-29 - Fix 6-1: Gate 4 + Phase C check 3 now use
//   usrConf.tx_gps_stale_timeout_ms instead of hardcoded 2000ms
// V2.5-Evo - 2026-04-29 - Fix 6-2: runRtmLoop() revokes gps_phase_b_ok
//   when TX GPS age exceeds 2× tx_gps_stale_timeout_ms
//
// The RTM state machine runs in loop() at ~10Hz (100ms rate-limit).
// When rtm_rx_active is set true by a 0xF1 meta-packet, this module:
//   1. Checks all 10 safety gates every iteration (any fail → emergency stop).
//   2. Computes compass bearing toward TX GPS position.
//   3. Converts bearing error to a steering override (0-255, 127=straight).
//   4. Runs Phase C: convergence check, VESC ERPM speed check, TX GPS freshness.
//
// All outputs are written to volatile globals read by calcPWM() and triggeredReceive().

// REPLACE WITH (add the extern line above it):
extern bool gps_phase_b_ok;   // V3 - P7 fix: defined in Radio.ino (Phase B section)
// ---- Phase C convergence tracking ----
static double        rtm_prev_dist_m = -1.0;   // distance to TX at last Phase C check
static unsigned long rtm_phase_c_ms  = 0;       // last Phase C check time

// ---- Safety gate check ----
// Returns true if ALL gates pass. Sets rtm_rx_emergency_stop=true and prints reason on any failure.
// Gate 1 (throttle released) returns false WITHOUT setting emergency_stop — motor is already 0.
static bool checkRtmSafetyGates()
{
  unsigned long now = millis();

  // Gate 1 (ABSOLUTE): user must be physically holding throttle > 10%.
  // Creator safety philosophy — this gate CANNOT be waived.
  if (thr_received < 25)
  {
    // Throttle released — this is normal; do not emergency-stop, just return false.
    // SAFETY FIX (2026-04-28 audit): reset steer override to straight (127) before returning.
    // Without this reset, the last bearing-derived value persists in rtm_steer_override.
    // calcPWM() applies that stale value to differential motor math even with thr=0:
    //   steering_offset_1 ≈ +286 at override=200 → PWM1_time=1286µs (motor spins ~28%)
    //   despite the user not holding the throttle — a hard safety violation.
    // Belt-and-suspenders companion fix is in PWM.ino calcPWM() (Task 1B).
    rtm_steer_override = 127;
    return false;
  }

  // Gate 2: Phase A GPS not rejected on RX
  if (gps_rejected)
  {
    Serial.println("RTM [RX] STOP: Phase A GPS rejected");
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 3: Phase B handshake passed
  if (!gps_phase_b_ok)
  {
    Serial.println("RTM [RX] STOP: Phase B handshake not passed");
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 4: valid TX GPS fix (age < usrConf.tx_gps_stale_timeout_ms)
  // Finding 6-1: was hardcoded 2000ms — now reads from SPIFFS so the
  // WebUI setting actually takes effect. Default is 1000ms.
  if (rx_tx_gps_timestamp == 0 ||
      (now - rx_tx_gps_timestamp) > (uint32_t)usrConf.tx_gps_stale_timeout_ms)
  {
    Serial.println("RTM [RX] STOP: TX GPS stale or never received");
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 5: valid RX GPS fix (age < 6000ms = 3× TX GPS timeout)
  if (gps_last_ms == 0 || (now - gps_last_ms) > 6000UL)
  {
    Serial.println("RTM [RX] STOP: RX GPS stale");
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 6: valid compass (if required)
  if (usrConf.rtm_compass_required)
  {
    float h = getCompassHeading();
    if (h < 0.0f)
    {
      Serial.println("RTM [RX] STOP: Compass not available");
      rtm_rx_emergency_stop = true;
      return false;
    }
  }

  // Gate 7: LoRa link healthy
  if (millis() - last_packet > usrConf.failsafe_time)
  {
    Serial.println("RTM [RX] STOP: LoRa link lost");
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 9: hard stop distance (buggy within stop distance of TX).
  // Guard: rtm_stop_distance_m==0 means SPIFFS held the pre-fix zero default.
  // Use 3m (firmware hard minimum) to keep Gate 9 active regardless of stored config.
  uint16_t stop_dist_m = (usrConf.rtm_stop_distance_m > 0) ? usrConf.rtm_stop_distance_m : 3u;
  float dist_m = (float)TinyGPSPlus::distanceBetween(
      gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);
  if (dist_m < (float)stop_dist_m)
  {
    Serial.printf("RTM [RX] STOP: within hard stop (%.0f m < %u m)\n",
                  dist_m, stop_dist_m);
    rtm_rx_emergency_stop = true;
    return false;
  }

  return true;
}

// ---- Compute RTM steering override ----
// Uses compass heading + TX GPS bearing to derive rtm_steer_override (0-255, 127=straight).
static void updateRtmSteering()
{
  if (!usrConf.rtm_rx_override_steering) return;

  float compass_heading = getCompassHeading();
  if (compass_heading < 0.0f)
  {
    rtm_steer_override = 127;  // straight ahead if no compass
    return;
  }

  // Bearing from RX GPS to TX GPS position (0-360, clockwise from North)
  double bearing_deg = TinyGPSPlus::courseTo(
      gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);

  // Heading error: positive = need to turn right, negative = turn left
  float heading_error = (float)(bearing_deg - compass_heading);
  while (heading_error >  180.0f) heading_error -= 360.0f;
  while (heading_error < -180.0f) heading_error += 360.0f;

  // Clamp to ±90° (full lock at 90° off course; ignore U-turns)
  float clamped = heading_error;
  if (clamped >  90.0f) clamped =  90.0f;
  if (clamped < -90.0f) clamped = -90.0f;

  // Map to 0-255 (127 = straight, >127 = right, <127 = left)
  rtm_steer_override = (uint8_t)(127.0f + (clamped / 90.0f) * 127.0f);

  #ifdef DEBUG_RX
  Serial.printf("RTM steer: bear=%.1f head=%.1f err=%.1f ovr=%d\n",
                (float)bearing_deg, compass_heading, heading_error, (int)(uint8_t)rtm_steer_override);
  #endif
}

// ---- Phase C anti-spoofing (runs during active RTM, every 5s) ----
static void runPhaseC()
{
  if (!rtm_rx_active || rtm_rx_emergency_stop) return;

  unsigned long now = millis();
  if (now - rtm_phase_c_ms < 5000UL) return;
  rtm_phase_c_ms = now;

  float dist_m = (float)TinyGPSPlus::distanceBetween(
      gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);

  // Phase C check 1: convergence — distance to TX must be decreasing
  if (rtm_prev_dist_m >= 0.0f && dist_m >= rtm_prev_dist_m)
  {
    Serial.printf("RTM [PhC] FAIL convergence: dist %.0f m (was %.0f m) — not closing\n",
                  dist_m, rtm_prev_dist_m);
    rtm_rx_emergency_stop = true;
    rtm_rx_active = false;
    return;
  }
  rtm_prev_dist_m = dist_m;

  // Phase C check 2: VESC ERPM vs GPS speed (only if vesc_erpm_per_kmh is configured)
  if (usrConf.vesc_erpm_per_kmh > 0.0f)
  {
    extern vesc_struct vesc;
    // vescMutex guards vesc.erpm against concurrent writes from the VESC task.
    extern SemaphoreHandle_t vescMutex;
    if (xSemaphoreTake(vescMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
      float vesc_speed_kmh = (float)abs(vesc.erpm) / usrConf.vesc_erpm_per_kmh;
      xSemaphoreGive(vescMutex);
      float speed_diff = fabsf(vesc_speed_kmh - gps_last_speed_kmh);
      if (speed_diff > usrConf.rtm_vesc_speed_diff_kmh)
      {
        Serial.printf("RTM [PhC] FAIL VESC speed: VESC=%.1f km/h GPS=%.1f km/h diff=%.1f\n",
                      vesc_speed_kmh, gps_last_speed_kmh, speed_diff);
        rtm_rx_emergency_stop = true;
        rtm_rx_active = false;
        return;
      }
    }
  }

  // Phase C check 3: TX GPS freshness
  // Finding 6-1: was hardcoded 2000ms — now reads from SPIFFS.
  // NOTE: structurally redundant — Gate 4 already enforces this before
  // runPhaseC() is called. Retained as belt-and-suspenders only.
  if (rx_tx_gps_timestamp == 0 ||
      (millis() - rx_tx_gps_timestamp) > (uint32_t)usrConf.tx_gps_stale_timeout_ms)
  {
    Serial.println("RTM [PhC] FAIL TX GPS freshness");
    rtm_rx_emergency_stop = true;
    rtm_rx_active = false;
    return;
  }

  Serial.printf("RTM [PhC] PASS: dist=%.0f m, converging\n", dist_m);
}

// ---- Main RTM loop — call from RX loop() ----
void runRtmLoop()
{
  // Rate-limit to 10Hz (compass I2C + TinyGPS math takes ~2ms per call)
  static unsigned long last_rtm_ms = 0;
  unsigned long now = millis();
  if (now - last_rtm_ms < 100UL) return;
  last_rtm_ms = now;

  // Finding 6-2: auto-expire Phase B approval when TX GPS goes stale.
  // gpsPhaseBCheck() sets gps_phase_b_ok=true on pass and never clears it —
  // it only runs on meta-packet receipt every ~30s. If TX GPS drops,
  // rx_tx_gps_timestamp stops updating and gps_phase_b_ok stays true
  // indefinitely. Gate 4 catches this during active RTM, but an RTM arm
  // attempt immediately after TX GPS loss could still pass Gate 3.
  // Revoke Phase B if TX GPS is older than 2× the configured stale threshold.
  {
    unsigned long phase_b_stale = (uint32_t)usrConf.tx_gps_stale_timeout_ms * 2UL;
    if (rx_tx_gps_timestamp == 0 ||
        (now - rx_tx_gps_timestamp) > phase_b_stale)
    {
      gps_phase_b_ok = false;
    }
  }

  // ---- Always compute RX→TX distance when GPS data is available ----
  // Populates telemetry.rtm_distance for TX at all times (not only during active RTM).
  // Bug 1B: TX can perform a pre-arm distance check before second-squeeze confirmation.
  // Bug 1C: Distance is available the moment RTM activates, no telemetry cycle delay.
  {
    bool gps_rx_ok = (gps_last_ms > 0) && ((millis() - gps_last_ms) < 6000UL);
    bool gps_tx_ok = (rx_tx_gps_timestamp > 0) && ((millis() - rx_tx_gps_timestamp) < 5000UL);

    if (gps_rx_ok && gps_tx_ok)
    {
      float d = (float)TinyGPSPlus::distanceBetween(
          gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);

      if (d < 10.0f)
      {
        // 0-99: tenths of metre (d=0.0..9.9 m)
        telemetry.rtm_distance = (uint8_t)(d * 10.0f);
      }
      else
      {
        // 100-254: whole metres offset by 90 (100=10m, 254=164m cap)
        uint8_t whole_m = (uint8_t)(d > 164.0f ? 164.0f : d);
        telemetry.rtm_distance = 90u + whole_m;
      }
    }
    else if (!rtm_rx_active)
    {
      telemetry.rtm_distance = 0xFF;  // mark N/A only when inactive AND no GPS
    }
    // If rtm_rx_active but GPS data just expired, keep last known value (don't set 0xFF)
  }

  if (!usrConf.rtm_rx_enabled)
  {
    rtm_rx_active         = false;
    rtm_rx_emergency_stop = false;
    return;
  }

  if (!rtm_rx_active)
  {
    rtm_rx_emergency_stop    = false;
    rtm_prev_dist_m          = -1.0;
    rtm_phase_c_ms           = 0;
    // telemetry.rtm_distance already set above (0xFF if no GPS, else live distance)
    return;
  }

  // RTM active: run all gates
  if (!checkRtmSafetyGates())
  {
    // Gate 1 (throttle released) returns false without setting emergency_stop.
    // All other gates set emergency_stop=true. If not emergency, just return.
    return;
  }

  // All gates pass: clear emergency stop, update steering
  rtm_rx_emergency_stop = false;
  updateRtmSteering();

  // Phase C (every 5s)
  runPhaseC();
}
