// V2.5-Evo - 2026-05-08 - Bundle 1: P+D+filter steering controller; preset table; bearing filter for FM path-following
// V2.5-Evo - 2026-05-06 - D5: getRtmHeading() layered heading source; updateRtmSteering() rewritten; Gate 6 accepts any source; updateCompassSnapshot() called from runRtmLoop top
// V3 - 2026-05-03 - C1/M2 audit fix: gps_tx_ok uses timestamp age on both paths; 0.0 lat/lng sentinel removed
// V3 - 2026-05-01 - Fix D: gps_tx_ok relaxed for FM/idle; never reset rtm_distance to 0xFF when RTM inactive
// V3 - 2026-05-01 - Fix C: FM bar keep-last-known on GPS dropout; only 0xFF if TX GPS never received
// V3 - 2026-05-01 - Fix B: encode rtm_distance always when GPS valid; feeds FM bar and enables correct pre-arm block within stop distance
// V3 - 2026-04-30 - Gate 9 clean disengagement (handoff to manual, no emergency stop); re-arm fix (0xFF when inactive); approach decel zone computation
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

extern bool gps_phase_b_ok;   // V3 - P7 fix: defined in Radio.ino (Phase B section)
// V2.5-Evo - 2026-05-06 - D5: extern declarations for D1+D2 capture globals.
extern float         gps_last_course_deg;       // From GPS.ino (D1) — last valid GPS course-over-ground (0-360 deg, -1.0 if none)
extern unsigned long gps_last_course_ms;        // From GPS.ino (D1) — millis() of last course update (0 if none)
extern float         compass_snapshot_heading;  // From Compass.ino (D2) — clean compass heading captured during motor-idle (0-360 deg, -1.0 if none)
extern unsigned long compass_snapshot_ms;       // From Compass.ino (D2) — millis() of snapshot capture (0 if none)
extern void          updateCompassSnapshot();   // From Compass.ino (D2) — captures clean compass heading when motor idle
// ============================================================
// RTM/FM STEERING CONTROLLER PRESETS — Bundle 1 (2026-05-08)
//
// PID-style controller: output = Kp * clamped_error - Kd * d(error)/dt
// Plus a low-pass filter on TARGET POSITION (lat/lng) for FM path-following
// — surfer's high-frequency bottom turns are smoothed out, buggy follows
// the surfer's path rather than chasing every wobble.
//
// For RTM (TX stationary), filter τ is set very low so behavior is essentially
// unfiltered — the filter doesn't hurt because there's nothing to smooth.
//
// 5 presets cover flat-water-to-heavy-surf range. Operator picks via WebUI
// before each session based on conditions. Default = Normal (index 2).
// ============================================================
struct SteerPreset {
  float error_clamp_deg;     // Saturation: heading error clamped to ±this before P/D math.
  float kp;                  // Proportional gain (PID Kp). 1.0 = baseline.
  float kd;                  // Derivative gain (PID Kd). 0.0 disables D term entirely.
  float target_filter_tau_s; // Low-pass filter time constant on target position (seconds).
                             // 0.5 ≈ no smoothing for RTM; 1-5 path-following for FM.
};

static const SteerPreset kSteerPresets[5] = {
  // {clamp,    Kp,   Kd,   tau_s }
  {  150.0f,  0.70f, 0.50f, 5.00f },  // 0 Very Soft   — heavy surf, aggressive surfer
  {  120.0f,  0.85f, 0.40f, 3.00f },  // 1 Soft        — choppy normal session
  {   90.0f,  1.00f, 0.30f, 2.00f },  // 2 Normal      — DEFAULT, mixed conditions
  {   60.0f,  1.20f, 0.20f, 1.00f },  // 3 Sharp       — calm water, RC use
  {   45.0f,  1.40f, 0.10f, 0.50f },  // 4 Very Sharp  — glass-flat, no waves
};

// ---- Bundle 1 module-level state for P+D controller and bearing filter ----
static float         prev_heading_error_deg    = 0.0f;
static unsigned long prev_steering_update_ms   = 0;
static double        tx_pos_filtered_lat       = 0.0;  // Filtered TX lat (degrees)
static double        tx_pos_filtered_lng       = 0.0;  // Filtered TX lng (degrees)
static bool          tx_pos_filter_initialized = false;

// Non-static globals exported to Logger.ino via extern (Bundle 1 tuning telemetry).
// 0x7FFF is the "no data" sentinel per CLAUDE.md Section 14 (non-zero).
int16_t g_heading_error_dx10 = 0x7FFF;  // Last heading error × 10 deg; 0x7FFF = no data
int16_t g_d_error_dx10       = 0x7FFF;  // Last derivative × 10 deg/s; 0x7FFF = no data

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

  // Gate 6: valid heading source (any source, per usrConf.rtm_use_compass mode)
  // V2.5-Evo - 2026-05-06 - D5: was compass-only check; now accepts GPS COG OR
  // compass snapshot OR live compass per the configured heading mode.
  // The legacy field name rtm_compass_required is preserved as the gate enable/disable.
  // When set to 1 (default), at least one valid heading source must exist.
  // When set to 0, this gate is bypassed (advanced/manual users only).
  if (usrConf.rtm_compass_required)
  {
    float h_unused;
    uint8_t conf_unused;
    if (!getRtmHeading(&h_unused, &conf_unused))
    {
      Serial.println("RTM [RX] STOP: No valid heading source (GPS COG too slow + compass snapshot stale)");
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

  // Gate 9: hard stop distance — buggy reached TX position.
  // This is a NORMAL RTM completion, not a safety failure (unlike Gates 2-8).
  // Clean disengagement: set rtm_rx_active=false and leave rtm_rx_emergency_stop=false
  // so calcPWM() passes user throttle through immediately (seamless manual handoff).
  // rtm_approach_cap reset to 255 so manual throttle is uncapped.
  // The inactive path in runRtmLoop() will set telemetry.rtm_distance=0xFF on the next
  // tick, clearing the TX pre-arm block so re-arm works after the buggy has moved away.
  // Guard: rtm_stop_distance_m==0 means SPIFFS held the pre-fix zero default;
  // use 3m (firmware hard minimum) to keep Gate 9 active regardless of stored config.
  uint16_t stop_dist_m = (usrConf.rtm_stop_distance_m > 0) ? usrConf.rtm_stop_distance_m : 3u;
  float dist_m = (float)TinyGPSPlus::distanceBetween(
      gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);
  if (dist_m < (float)stop_dist_m)
  {
    Serial.printf("RTM [RX] Gate 9: reached stop distance (%.1f m < %u m) — clean handoff to manual\n",
                  dist_m, stop_dist_m);
    rtm_rx_active         = false;   // disarm — enter inactive path next tick
    rtm_rx_emergency_stop = false;   // no emergency; motor returns to user throttle immediately
    rtm_approach_cap      = 255;     // clear decel cap so manual throttle is uncapped
    return false;
  }

  return true;
}

// V2.5-Evo - 2026-05-06 - D5: Layered heading source for RTM steering.
//
// Returns the best available heading (deg, 0-360 clockwise from North) based on
// usrConf.rtm_use_compass mode and current sensor state. Three modes:
//   0 = GPS COG only — no compass fallback. Safest choice for builds where compass
//       is biased by motor current (this hardware's bench-tested behavior).
//   1 = Hybrid (DEFAULT) — GPS COG primary; compass snapshot when buggy is too slow
//       for COG to be reliable. Compass snapshot is updated only when motor is idle
//       (thr_received < 25), so it represents an unbiased reading.
//   2 = Compass only — DIAGNOSTIC ONLY. Should NOT be used on water on builds with
//       known motor EMI. Available for non-EMI builds with proven clean compass
//       behavior under load.
//
// Confidence levels (output param):
//   3 = HIGH:   GPS COG, fresh and above min_speed threshold
//   2 = MEDIUM: compass snapshot < 1000ms old, or compass-only mode (legacy)
//   1 = LOW:    compass snapshot 1000-3000ms old (degraded — caller should reduce steering authority)
//   0 = NONE:   no valid heading source — caller must hold straight (rtm_steer_override = 127)
//
// Returns true if heading is valid (any non-zero confidence), false if no source.
//
// SAFETY: This function is read-only on globals; it never modifies sensor state.
//         Caller (updateRtmSteering) must handle confidence=0 as a hold-straight
//         scenario, not as a steering command.
static bool getRtmHeading(float* out_heading, uint8_t* out_confidence)
{
  uint16_t mode           = usrConf.rtm_use_compass;
  uint16_t cog_min_speed  = usrConf.rtm_cog_min_speed_kmh;
  unsigned long now       = millis();

  // ---- Mode 2: Compass only (legacy/diagnostic) ----
  // Use compass directly; valid only if compass returns non-error.
  // SAFETY: This mode SHOULD NOT be used on water — see field-service note in BREmote_V2_Rx.h.
  if (mode == 2) {
    float h = getCompassHeading();
    if (h < 0.0f) {
      *out_heading = -1.0f;
      *out_confidence = 0;
      return false;
    }
    *out_heading = h;
    *out_confidence = 2;  // MEDIUM — known biased under load but user opted in
    return true;
  }

  // ---- GPS COG (preferred for modes 0 and 1) ----
  // Valid if: course was captured (ms > 0), course is in valid range,
  //           course age < 1500ms, GPS speed >= cog_min_speed_kmh.
  bool cog_valid = (gps_last_course_ms > 0) &&
                   (gps_last_course_deg >= 0.0f) &&
                   ((now - gps_last_course_ms) < 1500UL) &&
                   (gps_last_speed_kmh >= (float)cog_min_speed);

  if (cog_valid) {
    *out_heading = gps_last_course_deg;
    *out_confidence = 3;  // HIGH
    return true;
  }

  // ---- Mode 0: GPS COG only — no fallback ----
  // If COG is invalid (slow speed or stale), return no source.
  // updateRtmSteering() will hold straight.
  if (mode == 0) {
    *out_heading = -1.0f;
    *out_confidence = 0;
    return false;
  }

  // ---- Mode 1 (Hybrid): try compass snapshot ----
  // Snapshot is captured by updateCompassSnapshot() in Compass.ino during motor-idle.
  // Age determines confidence:
  //   < 1000ms : MEDIUM (likely still fresh)
  //   1000-3000ms : LOW (degraded; reduce steering authority)
  //   > 3000ms : NONE (too stale)
  if (compass_snapshot_heading >= 0.0f && compass_snapshot_ms > 0) {
    unsigned long age_ms = now - compass_snapshot_ms;
    if (age_ms < 1000UL) {
      *out_heading = compass_snapshot_heading;
      *out_confidence = 2;  // MEDIUM
      return true;
    } else if (age_ms < 3000UL) {
      *out_heading = compass_snapshot_heading;
      *out_confidence = 1;  // LOW — caller should reduce steering authority
      return true;
    }
  }

  // ---- No valid heading source ----
  *out_heading = -1.0f;
  *out_confidence = 0;
  return false;
}

// ---- Compute RTM steering override (Bundle 1: P+D + bearing filter) ----
// V2.5-Evo - 2026-05-08 - Bundle 1: Replaced fixed ±90° clamp with preset-driven P+D controller.
// Added first-order low-pass filter on TX target position for FM path-following smoothness.
// Heading source still comes from getRtmHeading() (GPS COG primary, snapshot fallback).
// LOW-confidence sources reduce steering authority by 50% (unchanged from D5).
// Filter state + D-term reset on invalid heading to satisfy CLAUDE.md Section 12 rule 2.
static void updateRtmSteering()
{
  if (!usrConf.rtm_rx_override_steering) {
    rtm_steer_override = 127;
    g_heading_error_dx10 = 0x7FFF;
    g_d_error_dx10 = 0x7FFF;
    return;
  }

  float current_heading;
  uint8_t confidence;
  bool valid = getRtmHeading(&current_heading, &confidence);

  if (!valid) {
    // No valid heading — hold straight. Reset filter + D-term state so we don't
    // resume with stale data on next cycle. (Per CLAUDE.md Section 12 rule 2.)
    rtm_steer_override = 127;
    prev_heading_error_deg = 0.0f;
    tx_pos_filter_initialized = false;
    g_heading_error_dx10 = 0x7FFF;
    g_d_error_dx10 = 0x7FFF;
    return;
  }

  // Lookup active preset — clamp index defensively
  uint16_t idx = usrConf.rtm_steer_response;
  if (idx > 4) idx = 2;  // fallback to Normal on bad config
  const SteerPreset &p = kSteerPresets[idx];

  // ---- Bearing-target low-pass filter (for FM path-following) ----
  // First-order exponential moving average on TX position.
  // alpha = dt / (tau + dt). dt is loop period, tau is preset's filter time constant.
  unsigned long now = millis();
  float dt_s = (prev_steering_update_ms == 0) ? 0.1f : ((now - prev_steering_update_ms) / 1000.0f);
  if (dt_s <= 0.0f || dt_s > 1.0f) dt_s = 0.1f;  // sanity clamp
  prev_steering_update_ms = now;

  if (!tx_pos_filter_initialized) {
    tx_pos_filtered_lat = rx_tx_gps_lat;
    tx_pos_filtered_lng = rx_tx_gps_lng;
    tx_pos_filter_initialized = true;
  } else if (p.target_filter_tau_s > 0.0f) {
    float alpha = dt_s / (p.target_filter_tau_s + dt_s);
    tx_pos_filtered_lat += alpha * (rx_tx_gps_lat - tx_pos_filtered_lat);
    tx_pos_filtered_lng += alpha * (rx_tx_gps_lng - tx_pos_filtered_lng);
  } else {
    tx_pos_filtered_lat = rx_tx_gps_lat;
    tx_pos_filtered_lng = rx_tx_gps_lng;
  }

  // Bearing from RX GPS to FILTERED TX position
  double bearing_deg = TinyGPSPlus::courseTo(
      gps_last_lat, gps_last_lng, tx_pos_filtered_lat, tx_pos_filtered_lng);

  // Heading error (signed, wrapped to ±180°)
  float heading_error = (float)(bearing_deg - current_heading);
  while (heading_error >  180.0f) heading_error -= 360.0f;
  while (heading_error < -180.0f) heading_error += 360.0f;

  // Saturate (preset clamp angle)
  float clamped = heading_error;
  if (clamped >  p.error_clamp_deg) clamped =  p.error_clamp_deg;
  if (clamped < -p.error_clamp_deg) clamped = -p.error_clamp_deg;

  // P term (normalized to ±127 at full clamp)
  float p_term = (clamped / p.error_clamp_deg) * 127.0f * p.kp;

  // D term (rate of change of error). Skip on first cycle (prev=0 baseline).
  float d_error = (heading_error - prev_heading_error_deg) / dt_s;
  float d_term = p.kd * d_error;
  prev_heading_error_deg = heading_error;

  // Confidence: LOW conf reduces total authority by 50% (preserves D5 behavior)
  float authority = (confidence == 1) ? 0.5f : 1.0f;

  float output = 127.0f + authority * (p_term - d_term);
  if (output < 0.0f)   output = 0.0f;
  if (output > 254.0f) output = 254.0f;
  rtm_steer_override = (uint8_t)output;

  // Export for logger (with sentinel-safe conversion)
  g_heading_error_dx10 = (int16_t)(heading_error * 10.0f);
  g_d_error_dx10       = (int16_t)(d_error * 10.0f);
  if (g_heading_error_dx10 == 0x7FFF) g_heading_error_dx10 = 0x7FFE;  // avoid sentinel collision
  if (g_d_error_dx10       == 0x7FFF) g_d_error_dx10       = 0x7FFE;

  #ifdef DEBUG_RX
  Serial.printf("RTM steer[%u]: bear=%.1f head=%.1f err=%.1f d_err=%.1f P=%.1f D=%.1f auth=%.2f ovr=%d\n",
                idx, (float)bearing_deg, current_heading, heading_error, d_error,
                p_term, d_term, authority, (int)rtm_steer_override);
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
  // V2.5-Evo - 2026-05-06 - D5: Always update the compass snapshot, regardless
  // of RTM state or rate-limit gate. Snapshot only updates when motor is idle
  // (thr_received < 25, checked inside updateCompassSnapshot()), so this is cheap
  // and safe to call every iteration. The snapshot is consumed by getRtmHeading()
  // as the low-speed fallback heading source in Hybrid mode.
  updateCompassSnapshot();

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

  // ---- Distance computation: telemetry encoding + approach decel cap ----
  // Distance is always encoded when both GPS sources are valid — feeds the TX R5 proximity
  // bar during RTM and FM modes, and enables the TX pre-arm check to correctly block
  // re-arm while within rtm_disengage_distance_m (correct safety behaviour after Gate 9).
  // Approach decel cap is only computed during active RTM; reset to 255 otherwise.
  {
    bool gps_rx_ok = (gps_last_ms > 0) && ((millis() - gps_last_ms) < 6000UL);
    // C1/M2 audit fix: both active and inactive paths now require a fresh
    // rx_tx_gps_timestamp instead of the 0.0 lat/lng sentinel.
    // The 0.0 sentinel accepted any stale coordinate — field logs confirmed
    // GPS timestamps froze for 50+ seconds in urban environments, causing
    // RTM distance to read near-zero while actually 20m+ away.
    // Active RTM: 5s max age (tight — buggy is moving, staleness is dangerous).
    // Inactive/FM: 10s max age (tolerates brief meta-packet gaps without
    //              suppressing the FM bar; still rejects genuinely stale GPS).
    bool gps_tx_ok = (rx_tx_gps_timestamp > 0) &&
                     ((millis() - rx_tx_gps_timestamp) < (rtm_rx_active ? 5000UL : 10000UL));

    if (gps_rx_ok && gps_tx_ok)
    {
      float d = (float)TinyGPSPlus::distanceBetween(
          gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);

      // Always encode real distance when both GPS sources are valid.
      // 0-99: tenths of metre (0.0-9.9 m); 100-254: whole metres offset by 90 (10-164 m)
      if (d < 10.0f)
      {
        telemetry.rtm_distance = (uint8_t)(d * 10.0f);
      }
      else
      {
        uint8_t whole_m = (uint8_t)(d > 164.0f ? 164.0f : d);
        telemetry.rtm_distance = 90u + whole_m;
      }

      if (rtm_rx_active)
      {
        // Approach decel zone: linearly ramp the throttle cap as the buggy closes in.
        // At rtm_approach_zone_m (outer edge): cap = 255 (full user throttle).
        // At rtm_stop_distance_m (Gate 9 edge):  cap = 0  (buggy coasts to stop naturally).
        // Between those two distances: linear interpolation.
        // Gate 9 still fires as the absolute safety floor.
        // rtm_approach_zone_m == 0 disables the feature (hard stop only).
        if (usrConf.rtm_approach_zone_m > 0)
        {
          uint16_t stop_m     = (usrConf.rtm_stop_distance_m > 0) ? usrConf.rtm_stop_distance_m : 3u;
          float    approach_m = (float)usrConf.rtm_approach_zone_m;
          if (approach_m > (float)stop_m && d < approach_m)
          {
            float cap_frac = (d - (float)stop_m) / (approach_m - (float)stop_m);
            if (cap_frac < 0.0f) cap_frac = 0.0f;
            if (cap_frac > 1.0f) cap_frac = 1.0f;
            rtm_approach_cap = (uint8_t)(cap_frac * 255.0f);
          }
          else
          {
            rtm_approach_cap = 255;  // outside zone: no cap
          }
        }
        else
        {
          rtm_approach_cap = 255;  // feature disabled: no cap
        }
      }
      else
      {
        rtm_approach_cap = 255;  // RTM inactive: no approach cap
      }
    }
    else if (!rtm_rx_active)
    {
      // FM/idle: never actively write 0xFF here. The struct field initialises to 0xFF;
      // Fix B above updates it to real distance once gps_rx_ok && gps_tx_ok is satisfied.
      // Actively resetting to 0xFF on any GPS hiccup caused the FM bar to stay dark.
      // rtm_approach_cap must be 255 when RTM is inactive — no throttle capping outside RTM.
      rtm_approach_cap = 255;
    }
    // GPS conditions failed (RTM active or inactive): keep last known distance and cap.
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
    rtm_approach_cap         = 255;   // belt-and-suspenders: ensure cap is always clear when inactive
    // Bundle 1: reset filter + D-term state so re-arm starts fresh (not from last session)
    tx_pos_filter_initialized = false;
    prev_heading_error_deg    = 0.0f;
    prev_steering_update_ms   = 0;
    g_heading_error_dx10      = 0x7FFF;
    g_d_error_dx10            = 0x7FFF;
    // telemetry.rtm_distance already set to 0xFF by the block above (inactive path)
    return;
  }

  // RTM active: run all gates
  if (!checkRtmSafetyGates())
  {
    // Gate 1: throttle released — no emergency stop, motor already at 0.
    // Gate 9: stop distance reached — clean disengagement, rtm_rx_active set false, no emergency stop.
    // Gates 2-8: safety failure — rtm_rx_emergency_stop=true, calcPWM() forces throttle to 0.
    return;
  }

  // All gates pass: clear emergency stop, update steering
  rtm_rx_emergency_stop = false;
  updateRtmSteering();

  // Phase C (every 5s)
  runPhaseC();
}
