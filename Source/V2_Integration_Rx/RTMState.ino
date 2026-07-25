// V2.5-Evo - 2026-07-25 - F3-c (RX FM): the 8 m engage-distance floor guarded ONLY the manual fm_engage_dist_m value. The AUTO branch — kFmEngageFactor (1.5) x (min_dist_m + followme_smoothing_band_m) — used its product raw, and neither of those two SPIFFS fields has a lower bound, so a small tuning such as min_dist 1 + band 1 produced d_engage = 3 m: BELOW the measured 20 ft / 6.10 m tow rope, letting Follow-Me latch and engage with the rider still ON the rope. Same hazard the floor exists to prevent, reached through the other branch. FIX: each branch now only computes its candidate and ONE kFmEngageDistFloorM clamp is applied to the final d_engage regardless of origin. The clamp can only RAISE d_engage (engage later, never earlier), and it is a no-op at the owner's 4+2 tuning, which yields 9.0 m. Also swept the stale "6.7-7.6 m" tow-rope prose in this file to the measured 6.10 m. No confStruct change, sizeof stays 184, SW_VERSION stays 34.
// V2.5-Evo - 2026-07-25 - F3-b (RX FM; comment + shared-constant move, no behaviour change): kFmEngageDistFloorM is no longer DEFINED in this file. It now has exactly one definition, in BREmote_V2_Rx.h, raised 5.0 -> 8.0 m — 5.0 m sat below the tow rope it exists to clear (the owner's rope is 20 ft = 6.10 m), so a manual fm_engage_dist_m of 5.0-6.1 m was legal and let FM engage with the rider still ON the rope. ConfigService.ino's duplicate bare 5.0f literal is gone with it; both the validator and the read-site clamp in runFmLoop() now reference the one shared constant. The clamp itself is unchanged in shape — legacy stored values still clamp UP to the floor, now 8.0 m. No confStruct change, sizeof stays 184, SW_VERSION stays 34.
// V2.5-Evo - 2026-07-25 - Batch A follow-up (Rex A3 NO-GO: F1/F3/F5/F7), RX FM only. (F1) the A3 divergence detector was a BARE THRESHOLD on a 3000 ms dwell while the engage ramp is 3500 ms, so it could fire BEFORE the ramp even finished and aborted ordinary engagements (at engage, dist_m is typically 13-21 m against an 18 m ceiling, and the align cap of 13/255 means the gap GROWS first). It now mirrors runPhaseC()'s actual shape: the distance at dwell start is captured (fm_diverge_start_dist_m) and the fault is raised at dwell expiry ONLY if the buggy is not closing (dist_m >= start - kFmDivergeCloseEpsM 2.0 m); if it has closed by more than that it IS following, just far, so the timer clears and no fault fires. Plus an engage grace: the detector is skipped and its dwell parked for kFmEngageRampMs + kFmDivergeMs (6.5 s) after every engagement so the buggy is allowed to ramp and align before it is judged. (F3) fm_engage_dist_m gained a 5 m floor — a stored value of, say, 3 m IS the engage distance in metres and is SHORTER than the 6.7-7.6 m tow rope, which defeated the separation interlock entirely; cfgValidateCrossField() now accepts only 0 (auto) or >= 5.0 m, and the use site clamps defensively so a pre-existing stored value cannot slip through. (F5) corrected the A2 comment that claimed d_engage feeds the distance Schmitt hysteresis - it does not; the Schmitt uses min_dist / min_dist+band. (F7) the divergence Serial.printf now runs AFTER fm_throttle_cap = 0 so UART backpressure can never defer the hard stop. All compile-time constants; no confStruct change, sizeof stays 184, SW_VERSION stays 34.
// V2.5-Evo - 2026-07-25 - Batch A (A2+A3), RX FM only. (A2) fm_engage_dist_m is now READ: >0 sets the FM engage distance directly in metres (rope x ~1.15), 0 keeps the previous auto behaviour (kFmEngageFactor x d_follow) bit-for-bit; latch, dwell and Schmitt hysteresis untouched. (A3) FM divergence FAULT — runFmLoop() gained the upper distance bound it never had: condition 8 is a lower bound only, and runPhaseC()'s convergence check is RTM-only (called from runRtmLoop, never runFmLoop), so a wrong heading let FM steer away indefinitely. dist_m > kFmDivergeFactor(2.0) x D_engage sustained kFmDivergeMs(3000) while FM_ACTIVE now routes through the EXISTING fault path (FM_STOPPING ramp -> FM_IDLE, re-arm required, same haptic/St semantics as conditions 2-7). Non-blocking first-exceed timestamp, cleared on any condition/state/data-trust change. Subtract-only: adds no throttle, extends no engagement, does not touch the deadman. All compile-time constants; no confStruct change; SW_VERSION stays 34.
// V2.5-Evo - 2026-07-19 - P3 FM (DESIGN_FOLLOW_ME.md sections 4-7): Follow-Me autonomous following. Adds runFmLoop() 10Hz state machine (IDLE/ARMED/ACTIVE/DEMOTED incl. the missing 0xFF->usrConf.followme_mode fallback — SUPERSEDED 2026-07-20, see R0 below), all 9 activation/hold conditions with Schmitt hysteresis on distance and side-zone, the lag-anchor trailing target-point geometry, and the 5-stage subtract-only throttle cap chain. Reuses the existing EMA filter / P+D / heading ladder / authority / wrap pipeline unchanged - updateRtmSteering() only gains a target selector (RTM = rider position, FM = trailing point). telemetry.fm_status bit0 now reports FM engaged rather than FM mode selected. No confStruct change; SW_VERSION stays 33.
// V2.5-Evo - 2026-07-20 - FM engagement semantics (R0/R1/R2): (R0) BOTH 0xFF->usrConf.followme_mode fallbacks removed — 0xFF now means FM_IDLE always, killing the latently-armed factory boot; usrConf.followme_mode is the TX arm-gesture seed only. (R1) separation latch: FM's FIRST entry into ACTIVE now also requires dist > kFmEngageFactor(1.5) x d_follow sustained kFmSepDwellMs(2000) — the tow rope (6.7-7.6 m) is longer than the old engage distance, so FM could engage mid-tow; existing Schmitt hysteresis governs after the latch sets. (R2) two clears: thr_received<25 for kFmThrReleaseClearMs(10 s) clears the latch (ARMED-unlatched, mode memory kept); no 0xF2 refresh for kFmModeAgeMs(95 s) -> FM_IDLE. P3 geometry/cap/steering untouched. No confStruct change; SW_VERSION stays 33.
// V2.5-Evo - 2026-07-20 - FM control "brain" (Fable v1.4): (A) holds-vs-faults — condition 1=DEADMAN (throttle, never a fault), 8/9=HOLD (cap 0, stays ARMED, auto-resume, +kFmSpeedHystKmh speed hysteresis), 2-7=FAULT (FM_STOPPING ramp 0->255 over kFmStopRampMs -> FM_IDLE, re-arm required); heading loss (cond 6) is now ALWAYS a fault regardless of rtm_compass_required. (C) steer-cancel while ACTIVE -> ARMED-UNLATCHED (latch cleared, no alarm) guarded by kFmEngageGraceMs grace + kFmSteerPersistMs persistence; ARMED has no steer-cancel by construction. (D) fm_flags telemetry byte (repurposed reserved_tx_imu): armed/engaged/armed-not-ready/fault-stop-sticky(kFmFaultStickyMs). FM_DEMOTED renamed FM_HOLD; FM_STOPPING added. All compile-time constants; no confStruct change; SW_VERSION stays 33.
// V2.5-Evo - 2026-07-19 - Rex hardening: reset D-term continuity statics (prev_heading_src_valid/prev_heading_error_deg/prev_steering_update_ms) in the override-disabled early return so an off->on toggle can't differentiate a stale error across the gap
// V2.5-Evo - 2026-07-19 - FM triage (Fable audit §5): (1) no-fix engagement guard — getRtmHeading() returns confidence 0 unless a fresh RX GPS fix exists, so RTM/FM cannot report confidence 2 or engage with datetime_unix=0; (2) D-term differentiated only across consecutive same heading-source samples — skip the step on a source switch (COG<->compass) or a compass-snapshot re-snap to kill the ±300°/s Kd spikes
// V2.5-Evo - 2026-05-22 - SW32: Two-phase RTM throttle — align phase suppresses throttle until heading < rtm_align_threshold_deg; run phase GPS speed governor
// V2.5-Evo - 2026-05-11 - Phase C fix: VESC ERPM check now verifies data freshness via vesc.last_packet before comparing to GPS speed
// V2.5-Evo - 2026-05-08 - Bundle 1: P+D+filter steering controller; preset table; bearing filter for FM path-following
// V2.5-Evo - 2026-05-06 - D5: getRtmHeading() layered heading source; updateRtmSteering() rewritten; Gate 6 accepts any source; updateCompassSnapshot() called from runRtmLoop top
// V2.5-Evo - 2026-05-03 - C1/M2 audit fix: gps_tx_ok uses timestamp age on both paths; 0.0 lat/lng sentinel removed
// V2.5-Evo - 2026-05-01 - Fix D: gps_tx_ok relaxed for FM/idle; never reset rtm_distance to 0xFF when RTM inactive
// V2.5-Evo - 2026-05-01 - Fix C: FM bar keep-last-known on GPS dropout; only 0xFF if TX GPS never received
// V2.5-Evo - 2026-05-01 - Fix B: encode rtm_distance always when GPS valid; feeds FM bar and enables correct pre-arm block within stop distance
// V2.5-Evo - 2026-04-30 - Gate 9 clean disengagement (handoff to manual, no emergency stop); re-arm fix (0xFF when inactive); approach decel zone computation
// V2.5-Evo - 2026-04-25 - P7: RX RTM state machine, 10 safety gates, Phase C anti-spoofing.
// V2.5-Evo - 2026-04-27 - P8: runRtmLoop() encodes RX→TX distance into telemetry.rtm_distance (index 5)
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

extern bool gps_phase_b_ok;   // V2.5-Evo - P7 fix: defined in Radio.ino (Phase B section)
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
// D-term source-continuity tracking (2026-07-19 FM triage). We only differentiate
// heading_error across two samples that came from the SAME continuous heading source.
// prev_heading_src_id is a discriminator that changes on a source switch (COG<->compass)
// AND on a compass-snapshot re-snap; prev_heading_src_valid gates the very first sample.
static uint32_t      prev_heading_src_id       = 0;
static bool          prev_heading_src_valid    = false;
static double        tx_pos_filtered_lat       = 0.0;  // Filtered TX lat (degrees)
static double        tx_pos_filtered_lng       = 0.0;  // Filtered TX lng (degrees)
static bool          tx_pos_filter_initialized = false;

// Non-static globals exported to Logger.ino via extern (Bundle 1 tuning telemetry).
// 0x7FFF is the "no data" sentinel (non-zero).
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
  // use 10m (firmware hard minimum) to keep Gate 9 active regardless of stored config.
  uint16_t stop_dist_m = (usrConf.rtm_stop_distance_m > 0) ? usrConf.rtm_stop_distance_m : 10u;
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
//   1 = LOW:    compass snapshot 1000-8000ms old (degraded — caller should reduce steering authority)
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

  // ---- No-fix engagement guard (2026-07-19 FM triage, Fable audit §5) ----
  // A heading source is only meaningful for RTM/FM steering when the RX has a real
  // GPS position fix: the steering bearing is computed from gps_last_lat/lng, which are
  // 0,0 with no fix. The Fable log showed RTM reporting confidence=2 from a compass
  // snapshot while datetime_unix=0 (no fix) — that must never engage or be reported.
  // Require a fresh RX GPS fix (same age window as Gate 5) before granting ANY confidence.
  // A stationary buggy that HAS a fix still passes (gps_last_ms updates while stopped).
  if (gps_last_ms == 0 || (now - gps_last_ms) > 6000UL) {
    *out_heading = -1.0f;
    *out_confidence = 0;
    return false;
  }

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
  //   1000-8000ms : LOW (degraded; reduce steering authority) — SW45: extended from 3000ms for stationary RTM arm
  //   > 8000ms : NONE (too stale)
  if (compass_snapshot_heading >= 0.0f && compass_snapshot_ms > 0) {
    unsigned long age_ms = now - compass_snapshot_ms;
    if (age_ms < 1000UL) {
      *out_heading = compass_snapshot_heading;
      *out_confidence = 2;  // MEDIUM
      return true;
    } else if (age_ms < 8000UL) {
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

// ============================================================
// FOLLOW-ME (FM) AUTONOMOUS FOLLOWING - state, tuning constants, geometry
// V2.5-Evo - 2026-07-19 - P3 (DESIGN_FOLLOW_ME.md sections 4-7)
//
// WHAT FM DOES, IN PLAIN ENGLISH
// After the whip, the rider lets go of the rope and surfs the wave. FM makes the
// buggy trail the rider at a set distance and angle, steering itself, so the rider
// can keep their eyes on the wave instead of on the buggy.
//
// THE SAFETY RULES THIS CODE OBEYS (identical to RTM, non-negotiable):
//   1. The buggy ONLY moves while the rider physically holds the throttle trigger
//      (thr_received >= 25). FM never creates motion on its own.
//   2. FM ONLY steers and SUBTRACTS throttle. It can never add throttle. The human
//      trigger stays the one and only throttle source.
//   3. Releasing the trigger stops the buggy immediately (unchanged base architecture).
//   4. Every failure path - GPS, compass, LoRa, bad geometry - drives the motor to 0
//      by writing fm_throttle_cap = 0.
//
// FM writes exactly two things that can reach the motor: fm_throttle_cap (a cap that
// can only reduce throttle) and rtm_steer_override (steering only, and only while the
// trigger is held). Nothing else in this module touches the motor path.
// ============================================================

// ---- FM tuning constants (not user-configurable; the 8 SPIFFS FM params cover tuning) ----

// Rider speed below which a course derived from GPS positions is too noisy to trust.
// Measured: course noise roughly doubles below ~3 mph. Below this we drop to the
// degraded "hold station" geometry (no diagonal) rather than chase a bad course.
static const float    kFmCourseValidSpeedKmh = 5.0f;   // km/h

// How much faster than the rider the buggy is allowed to run while closing the gap.
// Feeds throttle cap 3 (speed governor): target = min(boogie_vmax, rider_speed + this).
static const float    kFmClosingMarginKmh    = 5.0f;   // km/h

// Align-phase throttle cap (~5% of 255). While the heading error is large the buggy
// should pivot toward the target, not drive away from it. Same value RTM's align phase uses.
static const uint8_t  kFmAlignCap            = 13;     // 0-255

// Engage ramp length. On every entry into FM_ACTIVE the throttle cap ramps 0 -> full
// over this time so re-engagement is always a smooth build, never a throttle jump.
static const uint32_t kFmEngageRampMs        = 3500;   // ms

// Minimum time between rider course/speed samples. The rider's position arrives at 2 Hz,
// so we need a baseline of a few hundred ms for a stable course rather than differentiating
// two nearly identical filtered positions and getting noise.
static const float    kFmMotionBaselineS     = 0.4f;   // seconds

// ---- Engagement-semantics constants (V2.5-Evo - 2026-07-20) ----
// These four numbers implement the separation latch and its clears. They are deliberately
// compile-time only: no new confStruct fields, no SW_VERSION bump, no SPIFFS reset.

// How much further than the steady-state follow distance the rider must get before FM is
// allowed to engage for the first time. D_engage = kFmEngageFactor * d_follow.
// WHY THIS EXISTS: before this change the engage distance EQUALLED the follow distance
// (d_follow = min_dist_m + band). The tow rope MEASURES 20 ft = 6.10 m, which is LONGER than
// d_follow at the intended 4+2 = 6 m tuning — so FM could engage while the rider was still
// on the rope, i.e. autonomous steering mid-tow. 1.5x gives 9 m at 4+2 tuning: 9 m clears
// the measured 6.10 m rope by ~1.48x.
// V2.5-Evo - 2026-07-25 - F3-c prose fix: this comment used to quote a "6.7-7.6 m" tow rope.
// That number was an early estimate and it contradicted the MEASURED 6.10 m figure that
// BREmote_V2_Rx.h, ConfigService.ino and the web UI text all use. There is one rope length in
// this project and it is 6.10 m; the estimate is gone so the documentation stops arguing with itself.
// IMPORTANT: this factor is NOT the only thing protecting the engage distance. min_dist_m and
// followme_smoothing_band_m have no lower bound of their own, so a small tuning can make this
// product tiny — which is why kFmEngageDistFloorM is applied to the RESULT of this multiplication
// as well, not just to a manually typed fm_engage_dist_m. See the F3-c clamp in runFmLoop().
static const float    kFmEngageFactor        = 1.5f;   // multiplier on d_follow

// How long the rider must stay beyond D_engage before the separation latch sets.
// The rider position arrives at 2 Hz, so 2000 ms = 4 consecutive independent GPS fixes.
// WHY: the logs contain single-fix GPS spikes implying 41-144 mph. A spike moves the
// apparent distance for one fix; it cannot SUSTAIN it for four. The dwell converts a
// noise-triggerable threshold into one that needs real, persistent separation.
static const uint32_t kFmSepDwellMs          = 2000;   // ms

// How long the throttle may stay released before the separation latch is cleared.
// WHY: this is the session boundary. To rig the next tow the rider necessarily lets go of
// the trigger, so the latch dies with the run and the next tow starts unlatched — FM cannot
// engage again until separation has been re-proven. A rider linking waves keeps the trigger
// held and therefore keeps the latch. Same 25-count threshold FM condition 1 uses.
static const uint32_t kFmThrReleaseClearMs   = 10000;  // ms

// How long the RX keeps a TX-declared FM mode alive without a refresh.
// The TX re-sends 0xF2/mode every 30 s while armed, so 95 s is ~3 missed keepalives.
// WHY: without this the RX stored the declared mode FOREVER. If the TX's disarm burst
// (0xF2/0) was lost in the air, the RX stayed armed for the rest of the session with no
// way to find out. This is the backstop that expires a declaration nobody is refreshing.
static const uint32_t kFmModeAgeMs           = 95000;  // ms

// ---- A3 holds-vs-faults + steer-cancel constants (V2.5-Evo - 2026-07-20) ----
// Compile-time only, like the four above: no confStruct fields, no SW_VERSION bump, no SPIFFS reset.

// Condition 9 (rider speed) RESUME hysteresis. FM HOLDs when the rider drops below
// foiler_low_speed_kmh and only resumes once they are back above foiler_low_speed_kmh +
// kFmSpeedHystKmh. WHY: falling below speed is normal and recurring, so it is a HOLD not a fault;
// without the +2 km/h gap a rider hovering at the threshold would flap HOLD<->ACTIVE every fix.
// Mirrors the distance Schmitt band.
static const float    kFmSpeedHystKmh        = 2.0f;   // km/h

// FAULT stop ramp. On a fault (conditions 2-7) FM hands throttle back to the rider by ramping the
// cap 0 -> 255 over this window, then drops to FM_IDLE (re-arm required). WHY the ramp: the rider
// may still be holding the trigger, so returning full manual throttle instantly would lurch.
static const uint32_t kFmStopRampMs          = 2000;   // ms

// How long the surprise-gated fault-stop notification stays sticky in fm_flags bit 3. WHY:
// telemetry rotates every ~2.4 s, so a one-shot notification could land between rotations and
// never reach the TX. 6 s guarantees the TX sees it and can fire the St + stop buzz exactly once.
static const uint32_t kFmFaultStickyMs       = 6000;   // ms

// Engage grace: FM ignores steer-cancel for this long after engaging. WHY: technique 2 (whip by
// steering the buggy away) means the rider is often still feeding the tail of that steering input
// at the instant FM engages; without the grace that tail would immediately cancel the fresh FM.
static const uint32_t kFmEngageGraceMs       = 2000;   // ms

// Steering persistence filter for steer-cancel. A steering deflection must be sustained beyond
// the deadband for this long before it cancels an ACTIVE FM. WHY: a momentary blip (chop, a bump)
// must not drop FM; only a deliberate, held steer is a manual-control declaration.
static const uint32_t kFmSteerPersistMs      = 500;    // ms

// Steering deadband for steer-cancel, in raw steering counts either side of 127 (centre). A
// deflection smaller than this is treated as centred and never counts toward steer-cancel.
static const uint8_t  kFmSteerCancelDeadband = 40;     // counts from 127

// ---- FM divergence-fault constants (V2.5-Evo - 2026-07-25 - A3) ----
// Compile-time only, like every other kFm* above: no confStruct fields, no SW_VERSION bump, no
// SPIFFS reset.
//
// WHAT WAS MISSING. While FM is ACTIVE the distance condition (condition 8, in runFmLoop) is
// "dist_m >= min_dist" — a LOWER bound only. It answers "is the buggy far enough away to be safe?"
// and nothing else. If the steering is wrong — a mirrored steering_inverted, a compass 180 out, a
// bad course estimate — the buggy drives AWAY from the rider and that condition keeps passing more
// and more comfortably the further it gets. RTM does have a divergence net (runPhaseC's convergence
// check, "distance must be decreasing"), but runPhaseC() is only ever called from runRtmLoop() and
// never from runFmLoop(), so FM had no upper bound at all and would steer away indefinitely for as
// long as the rider held the trigger. These two numbers add that bound.
//
// WHY A CEILING AND NOT runPhaseC's "must be decreasing". RTM's rule is right for RTM: the buggy is
// commanded to close on a stationary rider, so any non-decreasing distance is wrong. FM is not
// closing — it deliberately holds station d_follow behind a MOVING rider, so distance legitimately
// rises and falls every wave and "must be decreasing" would fire constantly. What is never
// legitimate in FM is being far outside the follow geometry. So we keep runPhaseC's bookkeeping
// shape (a single sustained-condition timer, cleared the instant the condition stops holding) and
// change only the test itself.

// Multiple of D_engage beyond which the buggy is running away rather than following. 2x D_engage is
// ~18 m at the owner's 9 m engage setting: far outside any legitimate follow geometry (the target
// point sits d_follow, ~6 m, behind the rider) yet far enough out that a normal catch-up transient
// or ordinary GPS scatter never reaches it. Scales automatically with fm_engage_dist_m (A2).
static const float    kFmDivergeFactor       = 2.0f;   // multiplier on D_engage

// How long the distance must stay beyond that limit before it counts as divergence. Rider position
// arrives at 2 Hz, so 3000 ms is ~6 consecutive independent fixes — the same spike-proofing argument
// as kFmSepDwellMs. A single bad fix cannot trip it; a genuinely diverging buggy trips it in 3 s.
static const uint32_t kFmDivergeMs           = 3000;   // ms

// V2.5-Evo - 2026-07-25 - F1: how much closer the buggy must have got over the dwell window to be
// judged "following, just far" rather than "running away".
// WHY THIS EXISTS AT ALL. The first cut of this detector was a BARE THRESHOLD: beyond the ceiling for
// kFmDivergeMs = a fault, full stop. That is wrong for two reasons that together aborted ordinary
// engagements. (1) The engage ramp is kFmEngageRampMs = 3500 ms, LONGER than the 3000 ms dwell, so the
// fault could fire before the buggy had even been given full throttle. (2) During align the cap is
// kFmAlignCap = 13/255 (~5%), so the buggy pivots on the spot and the distance GROWS before it starts
// to shrink — while at the engagement instant dist_m is typically 13-21 m against an 18 m ceiling
// (2 x 9 m). The result was FM aborting ~3 s into most real engagements and forcing a mid-session
// re-arm. THE FIX: judge the DERIVATIVE, not the level — exactly what runPhaseC()'s convergence check
// does ("dist_m >= rtm_prev_dist_m" -> not closing -> fail). We snapshot the distance when the dwell
// starts and, at dwell expiry, only fault if the buggy has NOT closed by more than this epsilon.
// WHY 2.0 m. The buggy's closing speed is capped by cap 3, the speed governor, at rider speed +
// kFmClosingMarginKmh = 5 km/h = 1.39 m/s, so over the 3 s dwell a genuinely closing buggy recovers
// up to ~4.2 m — comfortably more than 2 m. 2 m is meanwhile larger than ordinary GPS scatter at these
// distances, so noise alone cannot fake "closing" and cancel a real divergence.
static const float    kFmDivergeCloseEpsM    = 2.0f;   // metres of closure over the dwell

// V2.5-Evo - 2026-07-25 - F3-b: the hard floor for the MANUAL fm_engage_dist_m override,
// kFmEngageDistFloorM, is NOT defined here any more. It used to sit in this block as 5.0f while
// ConfigService.ino carried a SECOND bare 5.0f literal, on the false premise that the Arduino
// concatenation order stopped the two files sharing a constant. It now has exactly one definition,
// in BREmote_V2_Rx.h — raised there to 8.0 m, because 5.0 m was below the tow rope it exists to
// clear (the owner's rope is 20 ft = 6.10 m). Both the config validator and the read-site clamp in
// runFmLoop() below reference that one constant. See BREmote_V2_Rx.h for the full rationale.

// ---- FM state machine (DESIGN_FOLLOW_ME.md section 4) ----
//   FM_IDLE    : FM off (mode 0), RTM owns the buggy, or GPS/FM disabled.
//                No throttle cap (255) and no steering override - fully manual buggy.
//   FM_ARMED   : a mode (1-3) is selected and all monitoring runs, but FM has not engaged
//                yet. The throttle chain is INACTIVE (cap 255) so the rider still has full
//                manual control of the buggy while FM waits for the follow geometry.
//   FM_ACTIVE  : every activation condition holds. Steering override on, throttle cap chain on.
//   FM_HOLD    : FM was ACTIVE and a HOLD condition dropped out - condition 8 (distance / stop
//                radius) or 9 (rider below foiler_low_speed_kmh), or the trigger was released
//                (DEADMAN). These are geometry / throttle pauses, NOT faults: the motor stops
//                (cap 0), the declaration stays ARMED, no alarm sounds, and FM auto-resumes to
//                FM_ACTIVE through the engage ramp once the conditions restore AND the separation
//                latch is set. Falling below speed is a normal, recurring part of riding, so it
//                must never force a re-arm. Kept distinct from FM_ARMED because the two carry
//                different caps (0 vs 255): before FM ever engaged the rider keeps manual throttle,
//                but once FM has held control a paused hold must stop the buggy. (Was FM_DEMOTED.)
//   FM_STOPPING: FM was engaged and a FAULT dropped out - conditions 2-7 (Phase A/B, TX/RX GPS
//                stale, heading invalid, LoRa). Something actually broke, so autonomy ends for
//                this run: the throttle cap ramps 0 -> 255 over kFmStopRampMs (throttle always
//                returns, never a lurch under a held trigger), then FM drops to FM_IDLE and a
//                fresh TX declaration is required to re-arm. A surprise-gated St + stop buzz fires
//                (fm_flags bit 3) only if the trigger was held at the fault instant.
enum FmState : uint8_t { FM_IDLE = 0, FM_ARMED = 1, FM_ACTIVE = 2, FM_HOLD = 3, FM_STOPPING = 4 };
static FmState fm_state = FM_IDLE;

// ---- FM rider tracking state ----
// fm_filt_* is FM's EMA-filtered rider position. It uses the SAME first-order filter
// formula and the SAME preset time constant as RTM's tx_pos_filtered_* (see
// updateRtmSteering) - the filter must keep ignoring the rider's carves so the buggy
// follows the low-passed path instead of mirroring bottom turns (measured p95 turn rate
// 49 deg/s at ~5 m radius). FM keeps its own copy because tracking has to stay warm while
// FM is only ARMED, whereas RTM's filter only runs while RTM is actively steering.
static double        fm_filt_lat         = 0.0;
static double        fm_filt_lng         = 0.0;
static bool          fm_filt_init        = false;
static unsigned long fm_filt_prev_ms     = 0;     // last EMA update (for the filter dt)

// Previous filtered position, used as the baseline for deriving rider course and speed.
static double        fm_prev_filt_lat    = 0.0;
static double        fm_prev_filt_lng    = 0.0;
static unsigned long fm_prev_filt_ms     = 0;

// Rider motion derived from successive FILTERED positions.
// fm_rider_course_deg is -1.0f when the rider is too slow for a trustworthy course.
static float         fm_rider_course_deg = -1.0f;  // 0-360 deg clockwise from North, or -1 = invalid
static float         fm_rider_speed_kmh  = 0.0f;   // km/h

// Side-zone Schmitt state: true = apply the diagonal offset, false = sit directly behind.
static bool          fm_diagonal_engaged = false;

// millis() at the moment FM entered FM_ACTIVE. Drives the engage ramp. 0 = not engaged.
static unsigned long fm_engage_ms        = 0;

// ---- Separation latch state (V2.5-Evo - 2026-07-20) ----
// fm_sep_latched: true once the rider has been proven genuinely separated from the buggy
//   (beyond D_engage for kFmSepDwellMs) during this throttle-hold session. FM may only make
//   its FIRST entry into FM_ACTIVE while this is true. Once latched, the existing distance
//   Schmitt hysteresis governs engage/re-engage as before, so the buggy is free to close back
//   to its normal 6 m station without fighting the interlock. The latch is NEVER cleared by
//   geometry alone — only by the throttle-release clear, a mode change, or entering FM_IDLE.
static bool          fm_sep_latched      = false;

// millis() when the rider first went beyond D_engage; 0 = not currently beyond it.
// Counts the dwell that defeats single-fix GPS spikes.
static unsigned long fm_sep_over_since_ms = 0;

// millis() when thr_received first dropped below 25; 0 = throttle currently held.
// Counts the kFmThrReleaseClearMs window that clears the latch at the end of a run.
static unsigned long fm_thr_low_since_ms  = 0;

// ---- A3 fault-stop + steer-cancel state (V2.5-Evo - 2026-07-20) ----
// millis() when FM entered FM_STOPPING; drives the 0 -> 255 fault ramp. 0 = not stopping.
static unsigned long fm_stop_ms          = 0;

// millis() of the last SURPRISING fault stop (a fault that occurred while the trigger was held).
// Drives the sticky fm_flags bit 3 for kFmFaultStickyMs so the TX cannot miss the stop
// notification across the ~2.4 s telemetry rotation. 0 = no recent surprising fault. Deliberately
// NOT cleared by fmEnterIdle() — the notification must survive the transition into FM_IDLE.
static unsigned long fm_fault_alarm_ms   = 0;

// millis() when the rider's steering first exceeded kFmSteerCancelDeadband while FM was ACTIVE;
// 0 = steering currently centred. Counts the kFmSteerPersistMs persistence filter for steer-cancel.
static unsigned long fm_steer_input_since_ms = 0;

// V2.5-Evo - 2026-07-25 - A3: millis() when dist_m first exceeded kFmDivergeFactor x D_engage while
// FM was ACTIVE; 0 = not currently beyond it. Counts the kFmDivergeMs dwell for the divergence fault.
// Reset discipline is copied from runPhaseC's rtm_prev_dist_m and from fm_sep_over_since_ms: cleared
// the moment the condition stops holding, whenever the distance is untrustworthy (trigger released,
// GPS stale/rejected, link down), whenever FM is not ACTIVE, and on entry to FM_IDLE. A half-finished
// proof is never carried across a data gap or a state change.
static unsigned long fm_diverge_since_ms = 0;

// V2.5-Evo - 2026-07-25 - F1: the buggy-to-rider distance in metres captured at the instant
// fm_diverge_since_ms started, i.e. the baseline the closure test compares against at dwell expiry.
// -1.0f = no dwell running / no baseline. This is the FM twin of runPhaseC's rtm_prev_dist_m: it turns
// the detector from "are you far?" (a level) into "are you failing to close?" (a derivative), which is
// what actually distinguishes a buggy running away from one that is following from further back than
// we would like. Cleared in lockstep with fm_diverge_since_ms everywhere, so a baseline can never
// outlive its own dwell or be compared against a distance from a different engagement.
static float         fm_diverge_start_dist_m = -1.0f;

// The computed trailing target point FM steers toward. Written by computeFmTarget() and
// read by updateRtmSteering() when fm_rx_active is set.
static double        fm_target_lat       = 0.0;
static double        fm_target_lng       = 0.0;

// ---- Compute RTM steering override (Bundle 1: P+D + bearing filter) ----
// V2.5-Evo - 2026-05-08 - Bundle 1: Replaced fixed ±90° clamp with preset-driven P+D controller.
// Added first-order low-pass filter on TX target position for FM path-following smoothness.
// Heading source still comes from getRtmHeading() (GPS COG primary, snapshot fallback).
// LOW-confidence sources reduce steering authority by 50% (unchanged from D5).
// Filter state + D-term reset on invalid heading to satisfy the heading-filter rule.
static void updateRtmSteering()
{
  if (!usrConf.rtm_rx_override_steering) {
    rtm_steer_override = 127;
    // Reset D-term continuity statics here too: with override disabled we produce no
    // steering samples, so on a later off->on toggle the D-term must not differentiate
    // a fresh heading error against a stale pre-toggle sample across the gap (Kd spike).
    prev_heading_src_valid  = false;
    prev_heading_error_deg  = 0.0f;
    prev_steering_update_ms = 0;
    g_heading_error_dx10 = 0x7FFF;
    g_d_error_dx10 = 0x7FFF;
    return;
  }

  float current_heading;
  uint8_t confidence;
  bool valid = getRtmHeading(&current_heading, &confidence);

  if (!valid) {
    // No valid heading — hold straight. Reset filter + D-term state so we don't
    // resume with stale data on next cycle. (project rule)
    rtm_steer_override = 127;
    prev_heading_error_deg = 0.0f;
    prev_heading_src_valid = false;   // no same-source prior sample to differentiate against
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

  // ---- Steering target selection: RTM aims at the rider, FM aims behind the rider ----
  // V2.5-Evo - 2026-07-19 - P3 FM. Two callers now share this controller:
  //   RTM (fm_rx_active == false): steer straight at the rider's EMA-filtered position.
  //                                This branch is the original code, unchanged.
  //   FM  (fm_rx_active == true) : steer at the trailing target point that runFmLoop() already
  //                                computed via computeFmTarget(). FM does its own EMA filtering
  //                                in updateFmRiderTracking() (it has to keep tracking while only
  //                                ARMED), so we skip RTM's filter here rather than filtering the
  //                                same rider position twice and adding a second lag.
  // Everything downstream of this block - heading error, +/-180 wrap, preset clamp, P+D, and
  // confidence-scaled authority - is shared by both modes and is untouched.
  double steer_target_lat, steer_target_lng;
  if (fm_rx_active) {
    steer_target_lat = fm_target_lat;
    steer_target_lng = fm_target_lng;
  } else {
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
    steer_target_lat = tx_pos_filtered_lat;
    steer_target_lng = tx_pos_filtered_lng;
  }

  // Bearing from RX GPS to the selected steering target
  double bearing_deg = TinyGPSPlus::courseTo(
      gps_last_lat, gps_last_lng, steer_target_lat, steer_target_lng);

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

  // ---- D term: differentiate ONLY across consecutive same-source samples ----
  // Fable audit §5: when the heading source switches (GPS COG <-> compass) or the
  // compass snapshot re-snaps, heading_error steps by tens of degrees in a single
  // 100ms tick. Differentiating across that step injects a false ±300°/s rate into
  // Kd and commands a violent phantom turn. Build a source id that changes on a
  // source switch AND on a snapshot re-snap; skip the D term (d_error=0) whenever
  // the id differs from the previous sample so we never differentiate through a step.
  uint32_t heading_src_id;
  if (confidence == 3) {
    heading_src_id = 1;    // GPS COG — updates smoothly with motion, safe to differentiate
  } else if (usrConf.rtm_use_compass == 2) {
    heading_src_id = 2;    // live compass-only (diagnostic) — continuous reading
  } else {
    // Hybrid compass snapshot: the value is held constant until re-snapped. Fold the
    // snapshot timestamp into the id (high bit set so it can never collide with 1/2)
    // so each re-snap is treated as a new source and the D step across it is skipped.
    heading_src_id = 0x80000000UL | ((uint32_t)compass_snapshot_ms & 0x7FFFFFFFUL);
  }

  float d_error;
  if (prev_heading_src_valid && heading_src_id == prev_heading_src_id) {
    d_error = (heading_error - prev_heading_error_deg) / dt_s;
  } else {
    d_error = 0.0f;  // source switched or snapshot re-snapped — do not differentiate across the step
  }
  float d_term = p.kd * d_error;
  prev_heading_error_deg = heading_error;
  prev_heading_src_id    = heading_src_id;
  prev_heading_src_valid = true;

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
  // V2.5-Evo - 2026-05-11 - Freshness guard: vesc.last_packet is read inside the same mutex
  // that protects vesc.erpm. If the VESC data is older than vesc_timeout_s (e.g. VESC dropped
  // during heavy regen braking), skip the check rather than comparing stale ERPM to live GPS
  // speed — a false FAIL here would abort RTM mid-run. Phase C check 1 (convergence) is the
  // primary safety gate and remains active regardless.
  if (usrConf.vesc_erpm_per_kmh > 0.0f)
  {
    extern vesc_struct vesc;
    extern SemaphoreHandle_t vescMutex;
    if (xSemaphoreTake(vescMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
      unsigned long vesc_age_ms = millis() - vesc.last_packet;
      float vesc_speed_kmh = (float)abs(vesc.erpm) / usrConf.vesc_erpm_per_kmh;
      xSemaphoreGive(vescMutex);

      if (vesc_age_ms > (unsigned long)usrConf.vesc_timeout_s * 1000UL)
      {
        // Stale VESC data — skip rather than falsely fail. Log for diagnostics.
        Serial.printf("RTM [PhC] SKIP VESC check: data age %lu ms > timeout %d s\n",
                      vesc_age_ms, (int)usrConf.vesc_timeout_s);
      }
      else
      {
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

  // ---- Extended telemetry: rx_heading, fm_heading_err, fm_status ----
  // rx_heading: GPS COG÷2 (0-179 maps to 0-358°); 0xFF = no valid COG
  if (gps_last_course_deg >= 0.0f && gps_last_course_ms > 0 &&
      (now - gps_last_course_ms) < 3000UL) {
    telemetry.rx_heading = (uint8_t)((uint16_t)(gps_last_course_deg) / 2);
  } else {
    telemetry.rx_heading = 0xFF;
  }

  // fm_heading_err: bearing error + 127 bias; 127 = no data
  if (g_heading_error_dx10 == 0x7FFF) {
    telemetry.fm_heading_err = 127;
  } else {
    int16_t e = g_heading_error_dx10 / 10;
    if (e < -126) e = -126;
    if (e >  126) e =  126;
    telemetry.fm_heading_err = (uint8_t)(e + 127);
  }

  // fm_status: [7]=aux2_on [6]=aux1_on [5]=vesc_online [4]=rx_wetness [3:2]=heading_conf [1]=rtm_active [0]=fm_active
  {
    uint8_t st = 0;
    // V2.5-Evo - 2026-07-19 - P3 FM: bit 0 now reports FM actually ENGAGED and steering
    // (fm_rx_active), not merely "a mode is selected". Previously any selected mode 1-3 set
    // this bit, so the TX could show FM as live while FM was only armed and waiting for the
    // follow geometry. fm_rx_active is set by runFmLoop() only in FM_ACTIVE.
    if (fm_rx_active)  st |= (1 << 0);
    if (rtm_rx_active) st |= (1 << 1);
    float h_unused; uint8_t conf;
    getRtmHeading(&h_unused, &conf);
    st |= (conf & 0x03) << 2;
    if (telemetry.error_code == 71) st |= (1 << 4);
    bool vesc_ok = (millis() - last_uart_packet) <
                   ((uint32_t)usrConf.vesc_timeout_s * 1000UL);
    if (vesc_ok)                 st |= (1 << 5);
    if (rx_aux_flags & (1 << 0)) st |= (1 << 6);
    if (rx_aux_flags & (1 << 1)) st |= (1 << 7);
    telemetry.fm_status = st;
  }

  // fm_flags (index 16): coherent Follow-Me engagement sub-state for the TX display (A2/A3/arming).
  // V2.5-Evo - 2026-07-20 - repurposed the former reserved_tx_imu byte. Kept SEPARATE from
  // fm_status (whose 8 bits are already full with aux/vesc/wetness/heading_conf/rtm_active) so no
  // working telemetry is disturbed and a TX still on the old firmware simply ignores this byte.
  // Bit map (the TX renders these in a later pass):
  //   [0] armed           - a live TX declaration is held (FM_ARMED / FM_ACTIVE / FM_HOLD). Scanner.
  //   [1] engaged         - FM is actively following (FM_ACTIVE). Grow-with-far distance bar.
  //   [2] armed-not-ready - armed but not yet engage-eligible on RX facts (no separation latch yet).
  //                         The TX ORs its own TX-local readiness (own GPS fix/age, pairing, last
  //                         reply age) on top, then renders blink-in-place (not ready) vs sweep (ready).
  //   [3] fault-stop      - a FAULT ended FM while the trigger was held; sticky kFmFaultStickyMs so
  //                         the TX cannot miss it across the ~2.4 s rotation and fires St + stop buzz.
  // The four A3 disarm-ownership facts (armed drops, engaged drops, fault-sticky rises) let the TX
  // detect an RX-side fault and clear its own fm_armed so display and engagement cannot disagree.
  {
    uint8_t f = 0;
    FmState s = fm_state;
    if (s == FM_ARMED || s == FM_ACTIVE || s == FM_HOLD)    f |= (1 << 0);
    if (s == FM_ACTIVE)                                     f |= (1 << 1);
    if ((s == FM_ARMED || s == FM_HOLD) && !fm_sep_latched) f |= (1 << 2);
    if (fm_fault_alarm_ms != 0 && (now - fm_fault_alarm_ms) < kFmFaultStickyMs) f |= (1 << 3);
    telemetry.fm_flags = f;
  }

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

      // rx_bearing_to_tx: compass bearing from buggy toward rider position÷2 (0-179); 0xFF = N/A
      {
        double btx = TinyGPSPlus::courseTo(
            gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);
        telemetry.rx_bearing_to_tx = (uint8_t)((uint16_t)(btx) / 2);
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
          uint16_t stop_m     = (usrConf.rtm_stop_distance_m > 0) ? usrConf.rtm_stop_distance_m : 10u;
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
    prev_heading_src_valid    = false;   // FM triage: no same-source prior sample after disarm
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

  // Two-phase RTM throttle control (SW32):
  // Phase 1 (Align): heading error > rtm_align_threshold_deg → ~5% throttle cap.
  //   Buggy pivots toward target without driving away. At near-zero throttle, motor
  //   current is minimal so compass bias is reduced — hybrid heading mode gets cleaner
  //   snapshot data during alignment, benefiting builds with BN-880 compass installed.
  // Phase 2 (Run): heading OK → GPS speed governor keeps speed at rtm_target_speed_kmh
  //   regardless of buggy power curve. Behaviour is consistent across different boogies.
  //   rtm_target_speed_kmh == 0 disables the governor (approach decel zone only).
  {
    float abs_err = (g_heading_error_dx10 != 0x7FFF) ?
        fabsf((float)g_heading_error_dx10 / 10.0f) : 180.0f;

    if (abs_err > (float)usrConf.rtm_align_threshold_deg) {
      // Phase 1 — Align: ~5% throttle — differential steers; buggy barely moves forward
      const uint8_t kAlignCap = 13;
      if (rtm_approach_cap > kAlignCap) rtm_approach_cap = kAlignCap;
    } else if (usrConf.rtm_target_speed_kmh > 0.0f) {
      // Phase 2 — Run: proportional GPS speed governor (full cap at target, zero cap at rest)
      float speed_frac = gps_last_speed_kmh / usrConf.rtm_target_speed_kmh;
      if (speed_frac > 1.0f) speed_frac = 1.0f;
      uint8_t speed_cap = (uint8_t)((1.0f - speed_frac) * 255.0f);
      if (rtm_approach_cap > speed_cap) rtm_approach_cap = speed_cap;
    }
  }

  // Phase C (every 5s)
  runPhaseC();
}

// ============================================================
// FOLLOW-ME GEOMETRY HELPERS
// V2.5-Evo - 2026-07-19 - P3 (DESIGN_FOLLOW_ME.md section 6)
// ============================================================

// ------------------------------------------------------------
// projectPoint - move a lat/lng a given distance along a given compass bearing
// ------------------------------------------------------------
// What it does:
//   Standard spherical "destination point given start, bearing and distance" formula.
//   Used to place the lag anchor ahead of the rider and the trailing target behind them.
//
// Inputs:
//   lat, lng     - start position in degrees (WGS84)
//   bearing_deg  - direction to travel, degrees CLOCKWISE FROM NORTH (0=N, 90=E, 180=S, 270=W).
//                  This is the same bearing convention TinyGPSPlus::courseTo(), the compass,
//                  and GPS course-over-ground all use on this board.
//   dist_m       - distance to travel in metres
//
// Outputs:
//   *out_lat, *out_lng - the resulting position in degrees
//
// Side effects: none (pure function).
//
// Precision note: all maths is double, matching the project rule that FM position maths must
// never drop to float - float carries only ~7 significant digits, which is visible error at the
// sub-10 m distances FM steers by.
// ------------------------------------------------------------
static void projectPoint(double lat, double lng, float bearing_deg, float dist_m,
                         double* out_lat, double* out_lng)
{
  const double kEarthRadiusM = 6371000.0;

  double br     = (double)bearing_deg * M_PI / 180.0;   // bearing in radians
  double ang    = (double)dist_m / kEarthRadiusM;       // angular distance in radians
  double lat_r  = lat * M_PI / 180.0;
  double lng_r  = lng * M_PI / 180.0;

  double sin_lat = sin(lat_r);
  double cos_lat = cos(lat_r);
  double sin_ang = sin(ang);
  double cos_ang = cos(ang);

  double new_lat_r = asin(sin_lat * cos_ang + cos_lat * sin_ang * cos(br));
  double new_lng_r = lng_r + atan2(sin(br) * sin_ang * cos_lat,
                                   cos_ang - sin_lat * sin(new_lat_r));

  *out_lat = new_lat_r * 180.0 / M_PI;
  *out_lng = new_lng_r * 180.0 / M_PI;
}

// ------------------------------------------------------------
// fmAngleDiff - smallest absolute angle between two compass bearings
// ------------------------------------------------------------
// Inputs:  a, b - bearings in degrees (any range)
// Returns: the absolute difference wrapped into 0-180 degrees
// Side effects: none.
// ------------------------------------------------------------
static float fmAngleDiff(float a, float b)
{
  float d = a - b;
  while (d >  180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return fabsf(d);
}

// ------------------------------------------------------------
// updateFmRiderTracking - EMA-filter the rider position and derive course + speed
// ------------------------------------------------------------
// What it does (DESIGN_FOLLOW_ME.md section 6 steps 1-2):
//   1. Low-pass filters the raw rider position that arrives from the TX at 2 Hz in the 0xF3
//      meta-packet. Same first-order EMA and same preset time constant RTM uses, so the buggy
//      follows the rider's smoothed path and ignores individual carves.
//   2. Derives the rider's course and speed from two successive FILTERED positions.
//      An EMA does not change steady-state velocity (it only adds lag), so differentiating the
//      filtered track gives a clean speed while rejecting per-sample GPS jitter.
//      Below kFmCourseValidSpeedKmh the course is marked invalid (-1) and FM falls back to the
//      degraded hold-station geometry instead of chasing a meaningless heading.
//
// Called every FM tick (10 Hz) whether FM is ARMED, DEMOTED or ACTIVE, so that rider motion is
// already warm the instant the activation conditions are met.
//
// Inputs:  reads rx_tx_gps_lat/lng/timestamp, usrConf.rtm_steer_response
// Outputs: writes fm_filt_lat/lng, fm_rider_course_deg, fm_rider_speed_kmh and the
//          fm_prev_filt_* baseline.
// Side effects: none outside those module globals. Never touches the motor path.
// ------------------------------------------------------------
static void updateFmRiderTracking()
{
  // No rider position has ever arrived - nothing to track. Leave the last known motion alone.
  if (rx_tx_gps_timestamp == 0) return;

  unsigned long now = millis();

  // Use the active steering preset's filter time constant - the same tau RTM filters with.
  uint16_t idx = usrConf.rtm_steer_response;
  if (idx > 4) idx = 2;                       // defensive fallback to Normal on bad config
  float tau = kSteerPresets[idx].target_filter_tau_s;

  // Filter timestep, sanity-clamped exactly the way updateRtmSteering() clamps it.
  float dt_s = (fm_filt_prev_ms == 0) ? 0.1f : ((now - fm_filt_prev_ms) / 1000.0f);
  if (dt_s <= 0.0f || dt_s > 1.0f) dt_s = 0.1f;
  fm_filt_prev_ms = now;

  // ---- Step 1: EMA filter of the raw rider position ----
  if (!fm_filt_init) {
    fm_filt_lat  = rx_tx_gps_lat;
    fm_filt_lng  = rx_tx_gps_lng;
    fm_filt_init = true;
  } else if (tau > 0.0f) {
    float alpha = dt_s / (tau + dt_s);
    fm_filt_lat += alpha * (rx_tx_gps_lat - fm_filt_lat);
    fm_filt_lng += alpha * (rx_tx_gps_lng - fm_filt_lng);
  } else {
    fm_filt_lat = rx_tx_gps_lat;
    fm_filt_lng = rx_tx_gps_lng;
  }

  // ---- Step 2: derive rider course + speed from successive filtered positions ----
  if (fm_prev_filt_ms == 0) {
    // First sample - just seed the baseline, no motion available yet.
    fm_prev_filt_lat = fm_filt_lat;
    fm_prev_filt_lng = fm_filt_lng;
    fm_prev_filt_ms  = now;
    return;
  }

  float dt2 = (now - fm_prev_filt_ms) / 1000.0f;
  if (dt2 < kFmMotionBaselineS) return;   // baseline too short for a stable course - wait

  float d_m = (float)TinyGPSPlus::distanceBetween(
      fm_prev_filt_lat, fm_prev_filt_lng, fm_filt_lat, fm_filt_lng);

  fm_rider_speed_kmh = (d_m / dt2) * 3.6f;

  if (fm_rider_speed_kmh >= kFmCourseValidSpeedKmh) {
    fm_rider_course_deg = (float)TinyGPSPlus::courseTo(
        fm_prev_filt_lat, fm_prev_filt_lng, fm_filt_lat, fm_filt_lng);
  } else {
    // Too slow for a trustworthy course - degrade to hold-station geometry (no diagonal).
    fm_rider_course_deg = -1.0f;
  }

  fm_prev_filt_lat = fm_filt_lat;
  fm_prev_filt_lng = fm_filt_lng;
  fm_prev_filt_ms  = now;
}

// ------------------------------------------------------------
// computeFmTarget - compute the point behind the rider that the buggy should steer to
// ------------------------------------------------------------
// What it does (DESIGN_FOLLOW_ME.md section 6 steps 3-4 and 6):
//
//   d_follow = min_dist_m + followme_smoothing_band_m   (owner default 4 + 2 = 6 m)
//
//   1. LAG ANCHOR. A first-order filter always trails a moving target by roughly v * tau
//      (13-18 m at 15-20 mph with tau = 2 s - larger than the follow gap itself, and it grows
//      with speed). If we simply sat d_follow behind the filtered position the buggy would fall
//      further behind the faster the rider went. So we first push an anchor point FORWARD along
//      the rider's course by min(v_rider * tau, 2 * d_follow), which cancels the filter lag and
//      makes the geometry speed-independent. The cap at 2 * d_follow stops a bad speed estimate
//      from throwing the anchor far up the track. This is deliberately NOT solved by shrinking
//      tau: the filter has to keep ignoring the rider's carves.
//
//   2. TRAILING POINT. target = anchor + d_follow at bearing (course + 180 + offset).
//
//   3. SIDE-ZONE SCHMITT. The diagonal offset is only applied while the buggy is reasonably
//      lined up behind the rider. We measure how far off the directly-behind axis the buggy
//      currently sits and run a Schmitt trigger on it (engage below zone_angle_enter_deg,
//      release above zone_angle_exit_deg, owner defaults 35 / 45). Outside the zone we fall
//      back to pure-behind. Without this hysteresis an unstable rider course could whip the
//      target point across the wake from one side to the other.
//
//   DEGRADED MODE. If the rider is too slow for a valid course, there is no meaningful "behind".
//   We hold station instead: put the target d_follow from the rider along the current
//   rider->buggy bearing, with no diagonal. The buggy holds its distance without manoeuvring
//   around what may be a rider in the water.
//
// ============================================================
// !!! OFFSET SIGN CONVENTION - VERIFY BEFORE WATER !!!
//
// This whole board works in ONE bearing convention: degrees CLOCKWISE FROM NORTH
// (0 = North, 90 = East, 180 = South, 270 = West). TinyGPSPlus::courseTo(),
// getCompassHeading() and GPS course-over-ground all return that convention, and
// projectPoint() above consumes it. Adding degrees to a bearing therefore rotates
// CLOCKWISE.
//
// Given that, with the rider travelling along course C:
//   directly behind the rider  = bearing C + 180
//   the rider's right-hand side = bearing C + 90   (clockwise from their heading)
//   the rider's left-hand side  = bearing C - 90
//
// So a "behind and to the rider's RIGHT" spot lies between C+180 and C+90, i.e. at
// C + 180 - near_diag_offset_deg. A "behind and to the rider's LEFT" spot lies between
// C+180 and C+270, i.e. at C + 180 + near_diag_offset_deg.
//
// Written as target_bearing = C + 180 + offset, that gives:
//   mode 1 Near-Right -> offset = -near_diag_offset_deg     (NEGATIVE)
//   mode 2 Behind     -> offset = 0
//   mode 3 Near-Left  -> offset = +near_diag_offset_deg     (POSITIVE)
//
// which matches DESIGN_FOLLOW_ME.md section 6 step 4 exactly. The single line that sets
// this sign is the "offset = -/+ usrConf.near_diag_offset_deg" pair below.
//
// ASSUMPTION BEING MADE: "Near-Right" means to the RIDER'S right as the rider faces along
// their direction of travel (not the buggy's right, and not the right of someone watching
// from the beach). Default mode 1 exists because the owner rides left-foot-forward, which
// puts the right side on their open/visible side - consistent with rider-relative.
//
// This must be confirmed against Tools/FollowMe Settings Visualizer.html before any water
// test. If the visualizer shows the mirror image, flip ONLY the two signs below - no other
// part of the geometry depends on this choice.
// ============================================================
//
// Inputs:  reads fm_filt_lat/lng, fm_rider_course_deg, fm_rider_speed_kmh, gps_last_lat/lng,
//          usrConf FM params, fm_mode_runtime
// Outputs: *out_lat / *out_lng - the target point to steer at
// Side effects: updates the fm_diagonal_engaged Schmitt latch.
// ------------------------------------------------------------
static void computeFmTarget(double* out_lat, double* out_lng)
{
  // Follow gap = hard-stop radius plus the smoothing band (owner default 4 + 2 = 6 m).
  float d_follow = usrConf.min_dist_m + usrConf.followme_smoothing_band_m;
  if (d_follow < 0.5f) d_follow = 0.5f;   // guard against a degenerate config

  // ---- Degraded mode: no trustworthy rider course - hold station at distance ----
  if (fm_rider_course_deg < 0.0f) {
    float b_rider_to_buggy = (float)TinyGPSPlus::courseTo(
        fm_filt_lat, fm_filt_lng, gps_last_lat, gps_last_lng);
    projectPoint(fm_filt_lat, fm_filt_lng, b_rider_to_buggy, d_follow, out_lat, out_lng);
    return;
  }

  float course = fm_rider_course_deg;

  // ---- Lag anchor: push forward along the rider's course to cancel the filter lag ----
  uint16_t idx = usrConf.rtm_steer_response;
  if (idx > 4) idx = 2;
  float tau     = kSteerPresets[idx].target_filter_tau_s;
  float v_ms    = fm_rider_speed_kmh / 3.6f;
  float lag_m   = v_ms * tau;
  float max_lag = 2.0f * d_follow;
  if (lag_m > max_lag) lag_m = max_lag;
  if (lag_m < 0.0f)    lag_m = 0.0f;

  double anchor_lat, anchor_lng;
  projectPoint(fm_filt_lat, fm_filt_lng, course, lag_m, &anchor_lat, &anchor_lng);

  // ---- Side-zone Schmitt: is the buggy lined up enough behind the rider to use the diagonal? ----
  float b_rider_to_buggy = (float)TinyGPSPlus::courseTo(
      fm_filt_lat, fm_filt_lng, gps_last_lat, gps_last_lng);
  float off_axis = fmAngleDiff(b_rider_to_buggy, course + 180.0f);

  if (!fm_diagonal_engaged && off_axis < usrConf.zone_angle_enter_deg) {
    fm_diagonal_engaged = true;
  } else if (fm_diagonal_engaged && off_axis > usrConf.zone_angle_exit_deg) {
    fm_diagonal_engaged = false;
  }

  // ---- Trailing point. See the OFFSET SIGN CONVENTION block above before touching these signs. ----
  // V2.5-Evo - 2026-07-20 - R0: the "0xFF falls back to usrConf.followme_mode" line was removed
  // here. 0xFF means the TX has never declared a mode this session; runFmLoop() now sends that
  // straight to FM_IDLE, so this function cannot be reached with m == 0xFF. If it somehow were,
  // neither branch below matches and the offset stays 0 (plain Behind) — the safe geometry.
  uint8_t m = fm_mode_runtime.load(std::memory_order_relaxed);

  float offset = 0.0f;                                          // mode 2 Behind
  if (fm_diagonal_engaged) {
    if (m == 1)      offset = -(float)usrConf.near_diag_offset_deg;   // mode 1 Near-Right
    else if (m == 3) offset = +(float)usrConf.near_diag_offset_deg;   // mode 3 Near-Left
  }

  float target_bearing = course + 180.0f + offset;
  projectPoint(anchor_lat, anchor_lng, target_bearing, d_follow, out_lat, out_lng);
}

// ------------------------------------------------------------
// checkFmFaultConditions - the FM FAULT conditions (2-7)
// ------------------------------------------------------------
// What it does (DESIGN_FOLLOW_ME.md section 5, A3 holds-vs-faults classification):
//   Evaluates the six FAULT conditions only - the ones that mean something actually BROKE, so FM
//   must end for the run and a fresh declaration is required to re-arm. Two conditions are handled
//   by the CALLER, not here, because they are not faults:
//     - Condition 1 (throttle >= 25) is the DEADMAN. A trigger release is never a fault (treating
//       it as one would end FM on every release, worse than the original bug); the caller reads it
//       as thr_held and the motor is already 0 by the base architecture when it is low.
//     - Conditions 8 (distance) and 9 (rider speed) are geometric HOLDs: they pause FM (cap 0) but
//       keep it ARMED and auto-resume. The caller evaluates them as dist_ok / speed_ok.
//   Like RTM, any one of these six failing means FM must not be steering. This function does NOT
//   set rtm_rx_emergency_stop - FM stops the motor through its own fm_throttle_cap so the two
//   systems can never fight over one flag.
//
// Returns: true only if all six fault conditions hold.
// Side effects: none (read-only on all globals).
// ------------------------------------------------------------
static bool checkFmFaultConditions()
{
  unsigned long now = millis();

  // 2. Phase A: the RX's own GPS has not been rejected as implausible/spoofed.
  if (gps_rejected) return false;

  // 3. Phase B: the TX<->RX cross-validation handshake is currently passing.
  if (!gps_phase_b_ok) return false;

  // 4. The rider's (TX) GPS position is fresh.
  if (rx_tx_gps_timestamp == 0 ||
      (now - rx_tx_gps_timestamp) > (uint32_t)usrConf.tx_gps_stale_timeout_ms) return false;

  // 5. The buggy's (RX) GPS position is fresh (same 6 s window as RTM gate 5).
  if (gps_last_ms == 0 || (now - gps_last_ms) > 6000UL) return false;

  // 6. A valid heading source exists. V2.5-Evo - 2026-07-20 - A3: FM ALWAYS requires a heading
  //    source, regardless of rtm_compass_required. That flag was an RTM-arming convenience; a
  //    missing heading source in FM means the buggy would steer blind at the ~5% align cap, which
  //    is a FAULT, not something to silently permit. This is the R4 heading-source-loss fix.
  {
    float h_unused; uint8_t conf_unused;
    if (!getRtmHeading(&h_unused, &conf_unused)) return false;
  }

  // 7. The LoRa link is healthy.
  if (now - last_packet > usrConf.failsafe_time) return false;

  return true;
}

// ------------------------------------------------------------
// fmComputeThrottleCap - the FM throttle cap chain
// ------------------------------------------------------------
// What it does (DESIGN_FOLLOW_ME.md section 7):
//   Runs five independent caps and returns the LOWEST. Every cap can only ever reduce the
//   rider's throttle - none of them can raise it. calcPWM() then applies the result with a
//   plain "if (cap < throttle) throttle = cap", so the human trigger stays the only throttle
//   source and FM can only subtract.
//
//   Cap 1 Hard stop      - dist < min_dist_m. Handled by the caller: that condition demotes FM
//                          out of FM_ACTIVE entirely and forces cap 0, so by the time we get
//                          here the buggy is always outside the stop radius.
//   Cap 2 Approach ramp  - linear 255 -> 0 across the smoothing band, same shape as RTM's
//                          approach decel zone. The buggy coasts down as it closes on the rider.
//   Cap 3 Speed governor - hold the buggy toward min(boogie_vmax, rider_speed + closing margin),
//                          measured against the buggy's own GPS speed. Same proportional form as
//                          RTM's run-phase governor, so behaviour is consistent across boogies.
//   Cap 4 Align phase    - while the heading error is large, clamp to ~5% so the buggy pivots
//                          toward the target instead of driving away from it.
//   Cap 5 Engage ramp    - 0 -> full over kFmEngageRampMs on every entry into FM_ACTIVE, so
//                          engaging and re-engaging is always a smooth build, never a jump.
//
// Inputs:  dist_m - current buggy-to-rider distance in metres; now - millis() for this tick
// Returns: the winning cap, 0-255
// Side effects: none.
// ------------------------------------------------------------
static uint16_t fmComputeThrottleCap(float dist_m, unsigned long now)
{
  uint16_t cap   = 255;                                     // start uncapped, take the lowest
  float min_dist = usrConf.min_dist_m;
  float band     = usrConf.followme_smoothing_band_m;

  // ---- Cap 2: approach ramp across the smoothing band ----
  if (band > 0.01f && dist_m < (min_dist + band)) {
    float frac = (dist_m - min_dist) / band;                // 1.0 at the outer edge, 0.0 at the stop radius
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    uint16_t c = (uint16_t)(frac * 255.0f);
    if (c < cap) cap = c;
  }

  // ---- Cap 3: speed governor ----
  float gov = fm_rider_speed_kmh + kFmClosingMarginKmh;
  if (gov > usrConf.boogie_vmax_in_followme_kmh) gov = usrConf.boogie_vmax_in_followme_kmh;
  if (gov > 0.1f) {
    float speed_frac = gps_last_speed_kmh / gov;            // buggy's own GPS speed vs the target
    if (speed_frac > 1.0f) speed_frac = 1.0f;
    if (speed_frac < 0.0f) speed_frac = 0.0f;
    uint16_t c = (uint16_t)((1.0f - speed_frac) * 255.0f);
    if (c < cap) cap = c;
  }

  // ---- Cap 4: align phase ----
  float abs_err = (g_heading_error_dx10 != 0x7FFF) ?
      fabsf((float)g_heading_error_dx10 / 10.0f) : 180.0f;  // no heading data -> treat as worst case
  if (abs_err > (float)usrConf.rtm_align_threshold_deg && kFmAlignCap < cap) {
    cap = kFmAlignCap;
  }

  // ---- Cap 5: engage ramp ----
  if (fm_engage_ms > 0) {
    unsigned long elapsed = now - fm_engage_ms;
    if (elapsed < kFmEngageRampMs) {
      uint16_t c = (uint16_t)(((float)elapsed / (float)kFmEngageRampMs) * 255.0f);
      if (c < cap) cap = c;
    }
  }

  return cap;
}

// ------------------------------------------------------------
// fmEnterIdle - drop FM fully out of the control path
// ------------------------------------------------------------
// Used when FM is switched off (mode 0), when RTM arms and takes the buggy, or when GPS/RTM
// is disabled in config. Clears the throttle cap back to 255 so the rider's manual throttle
// passes through completely untouched, drops the steering override, and cold-starts the rider
// tracking so the next arm begins from a clean filter rather than a stale position.
//
// Deliberately does NOT touch rtm_steer_override or the shared P+D statics - if we are here
// because RTM just armed, RTM now owns those.
// ------------------------------------------------------------
static void fmEnterIdle()
{
  fm_state            = FM_IDLE;
  fm_rx_active        = false;
  fm_throttle_cap     = 255;          // no cap - fully manual buggy
  fm_diagonal_engaged = false;
  fm_engage_ms        = 0;
  fm_filt_init        = false;
  fm_filt_prev_ms     = 0;
  fm_prev_filt_ms     = 0;
  fm_rider_course_deg = -1.0f;
  fm_rider_speed_kmh  = 0.0f;

  // V2.5-Evo - 2026-07-20 - R1/R2: leaving FM entirely drops the separation proof with it.
  // Whatever put us here (mode 0, RTM preemption, mode-age expiry, GPS/FM disabled) ends the
  // declaration, so the next arm must re-prove separation from scratch before FM may engage.
  fm_sep_latched       = false;
  fm_sep_over_since_ms = 0;
  fm_thr_low_since_ms  = 0;

  // V2.5-Evo - 2026-07-20 - A3: clear the steer-cancel persistence timer and the fault-ramp clock.
  // fm_fault_alarm_ms is deliberately NOT reset here: the surprise-gated stop notification must
  // stay sticky for kFmFaultStickyMs even after FM has dropped into FM_IDLE.
  fm_steer_input_since_ms = 0;
  fm_stop_ms              = 0;

  // V2.5-Evo - 2026-07-25 - A3: leaving FM drops any part-accumulated divergence proof with it, so
  // the next engagement starts its 3 s window from scratch rather than inheriting a stale timer.
  // F1: the closure baseline is cleared in the same breath — a distance measured during the previous
  // engagement must never be the yardstick for the next one.
  fm_diverge_since_ms     = 0;
  fm_diverge_start_dist_m = -1.0f;
}

// ------------------------------------------------------------
// runFmLoop - the Follow-Me state machine. Call from loop().
// ------------------------------------------------------------
// What it does (DESIGN_FOLLOW_ME.md sections 4-7):
//   Runs at 10 Hz, the same cadence as runRtmLoop(). Every tick it re-evaluates all nine
//   activation/hold conditions and moves FM between IDLE / ARMED / ACTIVE / DEMOTED. While
//   ACTIVE it computes the trailing target, hands it to the shared steering controller, and
//   recomputes the throttle cap chain.
//
//   Mutual exclusion with RTM is absolute: if rtm_rx_active is set, FM drops to IDLE and stops
//   writing anything into the control path. RTM arming therefore silently disarms FM, which is
//   the existing documented behaviour.
//
// Inputs:  fm_mode_runtime (0xFF = no TX declaration this session = FM_IDLE), fm_mode_last_rx_ms,
//          all GPS/link globals, the eight FM SPIFFS parameters.
// Outputs: fm_rx_active, fm_throttle_cap, rtm_steer_override (via updateRtmSteering),
//          fm_target_lat/lng.
// Side effects: MOTOR-RELEVANT. fm_throttle_cap can reduce throttle and rtm_steer_override can
//   redirect steering, but only ever through calcPWM()'s existing subtract-only chain and only
//   while the rider is holding the trigger.
// ------------------------------------------------------------
void runFmLoop()
{
  // Rate-limit to 10 Hz (matches runRtmLoop; the geometry maths costs ~1 ms per call).
  static unsigned long last_fm_ms = 0;
  unsigned long now = millis();
  if (now - last_fm_ms < 100UL) return;
  last_fm_ms = now;

  // Keep the rider filter and derived motion warm on every tick, in every state, so the
  // instant the conditions are met we already have a trustworthy course and speed.
  updateFmRiderTracking();

  // ---- Resolve the active mode ----
  // V2.5-Evo - 2026-07-20 - R0: the "0xFF falls back to usrConf.followme_mode" line is GONE.
  // WHAT THE BUG WAS: 0xFF means "the TX has not declared an FM mode this session". Falling
  // back to the SPIFFS value meant a factory RX (defaultConf.followme_mode = 1) booted
  // LATENTLY ARMED — no gesture, no declaration, and nothing on the display to say so. A rider
  // holding the trigger beyond the engage distance would have handed steering to FM without
  // ever asking for it. WHAT THE FIX DOES: 0xFF now means FM_IDLE, always, and 0xFF is greater
  // than 3 so the mode gate below catches it. usrConf.followme_mode keeps exactly one job —
  // it is the value the TX's arm gesture SEEDS from (TX RTMState.ino). It is never again an
  // RX-side auto-arm source. Autonomous steering now always requires a live human declaration.
  uint8_t m = fm_mode_runtime.load(std::memory_order_relaxed);

  // ---- R2(b): expire a declaration nobody is refreshing ----
  // The TX re-sends 0xF2/mode every 30 s while armed. If no refresh has arrived for
  // kFmModeAgeMs (95 s, ~3 missed keepalives), the declaration is stale — most likely the
  // TX disarmed and its 0xF2/0 burst was lost in the air, or the TX is gone. Drop to FM_IDLE
  // and reset the runtime mode to 0xFF so re-arming requires a fresh declaration.
  if (m >= 1 && m <= 3) {
    unsigned long mode_ms = fm_mode_last_rx_ms.load(std::memory_order_relaxed);
    if (mode_ms == 0 || (now - mode_ms) > kFmModeAgeMs) {
      Serial.println("FM [RX] mode declaration expired (no 0xF2 refresh) -> IDLE");
      fm_mode_runtime.store(0xFF, std::memory_order_relaxed);
      fmEnterIdle();
      return;
    }
  }

  // ---- FM_IDLE: FM off / never declared (0xFF), RTM owns the buggy, or GPS/RTM disabled ----
  if (!usrConf.gps_en || !usrConf.rtm_rx_enabled || rtm_rx_active || m < 1 || m > 3) {
    fmEnterIdle();
    return;
  }

  // ---- FM_STOPPING: a FAULT ended FM; ramp throttle back to manual, then go IDLE (A3) ----
  // V2.5-Evo - 2026-07-20 - A3 FAULT semantics. Once a fault has stopped FM this run, autonomy is
  // over until a fresh declaration. We do NOT re-check the conditions here: even if the fault
  // clears mid-ramp, FM stays down and requires re-arm (silent resume after an anomaly is exactly
  // the unrequested autonomy this architecture forbids). We only ramp the throttle cap back up so
  // the rider regains manual control smoothly, then drop to FM_IDLE.
  // MOTOR SAFETY: the cap only ever RISES toward 255 (subtract-only, never adds throttle); the
  // rider's held trigger stays the sole throttle source, and starting the ramp from 0 means no
  // lurch. (RTM preemption / GPS-off / mode-off above still abort straight to IDLE.)
  if (fm_state == FM_STOPPING) {
    fm_rx_active       = false;
    rtm_steer_override = 127;
    unsigned long stop_elapsed = now - fm_stop_ms;
    if (stop_elapsed >= kFmStopRampMs) {
      // Ramp done: require a fresh TX declaration to re-arm (mirror the mode-age expiry path so the
      // TX must re-send 0xF2/mode; the TX also learns of the fault via fm_flags and clears its own
      // fm_armed - see runRtmLoop's fm_flags bit 3).
      fm_mode_runtime.store(0xFF, std::memory_order_relaxed);
      fmEnterIdle();
      return;
    }
    fm_throttle_cap = (uint8_t)(((float)stop_elapsed / (float)kFmStopRampMs) * 255.0f);
    return;
  }

  // ---- R2(a): throttle-release clear — the end-of-run session boundary ----
  // The trigger being released for kFmThrReleaseClearMs (10 s) means this run is over: the
  // rider is swimming, resting, or rigging for the next tow. Clear the separation latch so
  // the next tow starts unproven and FM cannot engage on the rope. The declared MODE survives
  // (the rider still intends to use FM) — only the geometric proof is discarded.
  // MOTOR SAFETY: restoring cap 255 here cannot produce motion. We only reach this branch
  // because the trigger has been released for 10 s, and the deadman already holds the motor
  // at 0 while it is released. When the rider next squeezes, the latch is clear, so FM is not
  // eligible to engage and the buggy is fully manual — the safe steady state.
  if (thr_received < 25) {
    if (fm_thr_low_since_ms == 0) {
      fm_thr_low_since_ms = now;
    } else if ((now - fm_thr_low_since_ms) >= kFmThrReleaseClearMs) {
      if (fm_sep_latched || fm_state == FM_HOLD) {
        Serial.println("FM [RX] throttle released 10s -> separation latch cleared, ARMED-unlatched");
      }
      fm_sep_latched       = false;
      fm_sep_over_since_ms = 0;
      if (fm_state != FM_IDLE) {
        fm_state        = FM_ARMED;
        fm_throttle_cap = 255;   // back to fully manual; trigger is released, so no motion
        fm_rx_active    = false;
      }
    }
  } else {
    fm_thr_low_since_ms = 0;
  }

  // ---- Evaluate the conditions, split by A3 class ----
  // DEADMAN = condition 1 (throttle). FAULT = conditions 2-7. HOLD = conditions 8-9.
  bool  thr_held = (thr_received >= 25);       // condition 1 (DEADMAN — never a fault)
  bool  fault_ok = checkFmFaultConditions();   // conditions 2-7 (FAULT)
  bool  hard_ok  = thr_held && fault_ok;       // both needed for a trustworthy distance / latch
  bool  speed_ok = false;                      // condition 9 (HOLD — rider moving)
  bool  dist_ok  = false;                      // condition 8 (HOLD — follow geometry)
  float dist_m   = 0.0f;
  // V2.5-Evo - 2026-07-25 - A3: sustained divergence while ACTIVE. Classed as a FAULT (same family
  // as conditions 2-7), so it is routed through the SAME FM_STOPPING path below — never its own.
  bool  diverge_fault = false;
  // V2.5-Evo - 2026-07-25 - F7: the numbers the divergence message prints, captured at detection but
  // PRINTED LATER — in the fault branch, after fm_throttle_cap = 0. WHAT THE BUG WAS: the message was
  // printed at the moment of detection, which is upstream of the cap write, so a full UART TX buffer
  // could block inside Serial.printf() and delay the hard stop by however long the host took to drain
  // it. The motor must reach 0 first and the explanation can wait; nothing else reads these.
  float diverge_limit_m = 0.0f;   // the ceiling (kFmDivergeFactor x D_engage) that was exceeded, m
  float diverge_start_m = 0.0f;   // the distance captured when the dwell started, m

  if (hard_ok) {
    // Both GPS sources are guaranteed fresh here by conditions 4 and 5.
    dist_m = (float)TinyGPSPlus::distanceBetween(
        gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);

    float min_dist = usrConf.min_dist_m;
    float band     = usrConf.followme_smoothing_band_m;

    // Condition 9 (HOLD) with RESUME hysteresis. Below foiler_low_speed_kmh the rider may be down
    // or swimming, and the buggy must not manoeuvre around them — but a fall is normal and
    // recurring, so this is a HOLD (stays ARMED), never a fault. When already ACTIVE, stay down to
    // the plain threshold; when trying to (re)engage from HOLD/ARMED, require foiler_low_speed_kmh
    // + kFmSpeedHystKmh so FM cannot flap on and off at the speed line (mirrors the distance Schmitt).
    if (fm_state == FM_ACTIVE)
      speed_ok = (fm_rider_speed_kmh >= usrConf.foiler_low_speed_kmh);
    else
      speed_ok = (fm_rider_speed_kmh >= (usrConf.foiler_low_speed_kmh + kFmSpeedHystKmh));

    // Condition 8: Schmitt hysteresis on distance so FM cannot flap at the band edge.
    //   to ENGAGE  : the rider must be beyond min_dist + band
    //   to STAY ON : hold until the rider is inside min_dist
    if (fm_state == FM_ACTIVE) dist_ok = (dist_m >= min_dist);
    else                       dist_ok = (dist_m >  (min_dist + band));

    // ---- R1: separation latch (the tow interlock) ----
    // Before FM may engage for the first time this run, the rider must be proven genuinely
    // OFF THE ROPE: beyond D_engage (9 m at the 4+2 tuning, clearing the measured 20 ft /
    // 6.10 m rope) continuously for kFmSepDwellMs. The dwell is what makes this spike-proof:
    // a one-fix GPS glitch cannot hold the distance high across 4 consecutive 2 Hz fixes.
    // Once latched it STAYS latched until a §R2 clear, so the buggy may close back to its
    // normal 6 m station and re-engage on the ordinary Schmitt hysteresis without ever having
    // to re-prove separation mid-wave.
    // Same degenerate-config guard computeFmTarget() applies to d_follow: a zeroed min_dist
    // and band would otherwise make D_engage 0 and the latch would set on the first tick,
    // silently disabling the whole interlock.
    float d_follow_e = min_dist + band;
    if (d_follow_e < 0.5f) d_follow_e = 0.5f;

    // V2.5-Evo - 2026-07-25 - A2: honour the fm_engage_dist_m override.
    // WHAT WAS WRONG: the field has existed in confStruct since SW34 and ConfigService validates it
    // (0-50 m), and the web UI shows a row for it — but no code anywhere ever READ it, so turning the
    // knob changed nothing. WHAT THIS DOES: when set above zero, the stored value IS the engage
    // distance, in METRES. It is NOT the rope length — MEASURE the rope and set at least a metre
    // beyond it (the measured 20 ft / 6.10 m rope -> 8 m, which is also the enforced floor).
    // 0 = auto: D_engage = kFmEngageFactor (1.5) x d_follow. The 0.1f compare is the
    // float "is this really zero" guard, not a second threshold — ConfigService already clamps the
    // range to 0-50 m.
    // V2.5-Evo - 2026-07-25 - F3-c amendment: the auto branch is no longer bit-for-bit the pre-A2
    // behaviour — it is now floored at kFmEngageDistFloorM as well (see the F3-c block below). At
    // the 4+2 tuning auto yields 9.0 m, above the floor, so nothing changes at the shipped setting.
    //
    // V2.5-Evo - 2026-07-25 - F5 comment correction. The A2 note here used to claim D_engage feeds
    // "the distance Schmitt hysteresis". IT DOES NOT, and saying so was misleading about what this
    // knob actually moves. The condition-8 Schmitt a dozen lines above works purely off min_dist and
    // min_dist + band; it never looks at D_engage. The two things that DO consume D_engage are: (1)
    // the separation latch immediately below, together with its kFmSepDwellMs dwell, and (2) the A3
    // divergence ceiling further down, which is kFmDivergeFactor x D_engage. Nothing else reads it.
    //
    // V2.5-Evo - 2026-07-25 - F3 defensive floor. WHAT THE BUG WAS: ConfigService accepted any value
    // in 0-50 m with no lower bound above zero, so a stored 3 m was legal — and 3 m is SHORTER than
    // the tow rope. Since this value IS the engage distance, that setting let FM engage while the
    // rider was still on the rope: the exact scenario the separation latch exists to prevent.
    // V2.5-Evo - 2026-07-25 - F3-b: that floor was 5.0 m, which was itself below the rope it exists
    // to clear (the owner's rope is 20 ft = 6.10 m), so 5.0-6.1 m stayed legal and stayed on-rope.
    // It is now kFmEngageDistFloorM = 8.0 m, defined once in BREmote_V2_Rx.h and shared with the
    // validator — no duplicated literal.
    // WHAT THE FIX DOES: cfgValidateCrossField() refuses to STORE anything in (0, kFmEngageDistFloorM),
    // and the clamp below is the belt-and-braces companion for a value already sitting in SPIFFS from
    // before that rule existed — such a config is never re-validated, so without the clamp it would
    // still reach the latch. Behaviour is unchanged: a legacy stored value clamps UP to the floor.
    // 0 (auto) is still accepted by the validator and still takes the auto branch below.
    //
    // V2.5-Evo - 2026-07-25 - F3-c: THE FLOOR NOW GUARDS BOTH BRANCHES, NOT JUST THE TYPED VALUE.
    // WHAT THE BUG WAS: kFmEngageDistFloorM used to be applied INSIDE the manual branch only. The
    // auto branch (fm_engage_dist_m = 0) computed kFmEngageFactor (1.5) x (min_dist_m +
    // followme_smoothing_band_m) and used that product RAW. Those two SPIFFS values have no lower
    // bound of their own — min_dist_m 1 m with a 1 m smoothing band is a perfectly storable tuning,
    // and it yields d_engage = 1.5 x 2 = 3 m. Three metres is BELOW the measured 20 ft / 6.10 m tow
    // rope, so Follow-Me could set the separation latch and engage with the rider still ON the rope:
    // the exact hazard the floor exists to prevent, simply reached down the other branch. The floor
    // was guarding the number the rider TYPES while leaving the number the firmware COMPUTES open.
    // WHAT THE FIX DOES: each branch now only computes its candidate, and ONE clamp is applied to the
    // final d_engage regardless of which branch produced it. The tow rope is a physical fact about
    // this buggy, not a property of the config path, so the safety limit belongs on the result.
    // WHY IT IS SAFE IN BOTH DIRECTIONS: the clamp can only ever RAISE d_engage, never lower it. A
    // larger d_engage means the rider must separate FURTHER before FM may engage — it can only make
    // engagement later and harder, never earlier or easier.
    // NO-OP AT THE OWNER'S TUNING: min_dist_m 4 + followme_smoothing_band_m 2 = 6 m, x 1.5 = 9.0 m,
    // already above the 8.0 m floor. Nothing changes at the shipped setting; the clamp only bites on
    // a small-geometry tuning that would otherwise have produced an on-rope engage distance.
    // KNOCK-ON, CHECKED: the A3 divergence ceiling further down is kFmDivergeFactor x d_engage, so a
    // floored d_engage raises that ceiling in the same proportion — the detector becomes MORE
    // permissive, never less, and cannot be made to fire spuriously by this change.
    float d_engage;
    if (usrConf.fm_engage_dist_m > 0.1f) {
      d_engage = usrConf.fm_engage_dist_m;      // manual: the stored value IS the engage distance, in metres
    } else {
      d_engage = kFmEngageFactor * d_follow_e;  // auto: derived from the follow geometry
    }
    // F3-c: single tow-rope safety floor, applied to the COMPUTED value as well as the typed one.
    if (d_engage < kFmEngageDistFloorM) d_engage = kFmEngageDistFloorM;
    if (dist_m > d_engage) {
      if (fm_sep_over_since_ms == 0) {
        fm_sep_over_since_ms = now;
      } else if (!fm_sep_latched && (now - fm_sep_over_since_ms) >= kFmSepDwellMs) {
        fm_sep_latched = true;
        Serial.printf("FM [RX] separation latch SET: dist=%.1f m > D_engage=%.1f m sustained %lu ms\n",
                      dist_m, d_engage, (unsigned long)kFmSepDwellMs);
      }
    } else {
      fm_sep_over_since_ms = 0;   // fell back inside D_engage - the dwell restarts from scratch
    }

    // ---- A3: DIVERGENCE FAULT — the upper bound FM never had ----
    // V2.5-Evo - 2026-07-25. Condition 8 above is a lower bound only, so a buggy steering the WRONG
    // WAY satisfies it more and more comfortably the further it runs. This adds the missing ceiling:
    // while FM is ACTIVE, being further than kFmDivergeFactor x D_engage from the rider AND FAILING
    // TO CLOSE for kFmDivergeMs is not "following badly", it is "not following", and it is a FAULT.
    //
    // V2.5-Evo - 2026-07-25 - F1: this used to be a BARE THRESHOLD (beyond the ceiling for the dwell
    // = fault) and that aborted legitimate engagements. Two reasons, and they compound. First, the
    // engage ramp is kFmEngageRampMs = 3500 ms but the dwell is only kFmDivergeMs = 3000 ms, so the
    // fault could fire BEFORE the buggy had finished being given throttle. Second, while the heading
    // error is still large, cap 4 pins the throttle at kFmAlignCap = 13/255 (~5%) so the buggy pivots
    // in place and the gap GROWS before it starts to shrink — and at the engagement instant dist_m is
    // typically 13-21 m against an 18 m ceiling (2 x a 9 m D_engage). Net effect: FM aborted ~3 s into
    // most real engagements and forced a re-arm mid-session.
    //
    // THE FIX IS IN TWO PARTS, and both are needed:
    //
    //   1. TEST THE DERIVATIVE, NOT THE LEVEL. This is what runPhaseC() actually does — its
    //      convergence check fails on "dist_m >= rtm_prev_dist_m", i.e. on NOT CLOSING, never on being
    //      far. We do the same in FM's own terms: snapshot the distance when the dwell starts
    //      (fm_diverge_start_dist_m) and, at dwell expiry, fault only if the buggy has failed to close
    //      by more than kFmDivergeCloseEpsM. If it HAS closed by more than that, it is following — just
    //      from further back than we would like — so the timer is cleared and no fault fires. The next
    //      tick starts a fresh window, so a buggy that is beyond the ceiling and genuinely closing is
    //      re-tested every 3 s and keeps passing for exactly as long as it keeps closing.
    //      (Why "must be DECREASING" from runPhaseC is not copied verbatim: RTM closes on a stationary
    //      rider so any non-decrease is wrong, whereas FM holds station behind a MOVING rider and the
    //      distance legitimately rises and falls every wave. The epsilon is what carries that across.)
    //
    //   2. ENGAGE GRACE. The detector is skipped entirely, and its dwell parked, for
    //      kFmEngageRampMs + kFmDivergeMs (3500 + 3000 = 6500 ms) after every entry into FM_ACTIVE.
    //      The buggy must be allowed to finish ramping AND aligning before its geometry is judged;
    //      judging it mid-ramp measures the ramp, not the steering. Parking the dwell (rather than
    //      letting it run) guarantees the first post-grace window is a full, clean kFmDivergeMs.
    //      NOTE: this is deliberately NOT kFmEngageGraceMs (2000 ms) — that constant is the
    //      steer-cancel grace and is a different, shorter window for a different purpose.
    //
    // Bookkeeping is otherwise the same shape runPhaseC() uses: one timer plus one baseline, evaluated
    // every tick, both cleared the instant the condition stops holding, so nothing can accumulate
    // across a gap. The dwell is a plain first-exceed timestamp compared against millis() — no delay(),
    // no blocking, no extra loop.
    //
    // Only evaluated while fm_state == FM_ACTIVE (FM actually has control this tick). ARMED, HOLD and
    // STOPPING are all states in which FM is not steering, so distance says nothing about divergence.
    //
    // SAFETY: this branch only ever sets a flag that REMOVES eligibility. It cannot raise the
    // throttle cap, cannot extend engagement, and does not touch the deadman. Both parts of the fix
    // make the detector STRICTLY LESS likely to fire, never more — a missed divergence still leaves
    // every other fault condition and the deadman in place, and the rider can always let go.
    bool in_engage_grace = (fm_engage_ms != 0) &&
                           ((now - fm_engage_ms) < (kFmEngageRampMs + kFmDivergeMs));

    if (in_engage_grace) {
      // Ramping and/or aligning — not judgeable yet. Park the window so it starts fresh afterwards.
      fm_diverge_since_ms     = 0;
      fm_diverge_start_dist_m = -1.0f;
    }
    else if (fm_state == FM_ACTIVE && dist_m > (kFmDivergeFactor * d_engage)) {
      if (fm_diverge_since_ms == 0) {
        // First tick beyond the ceiling: start the dwell and record what we are closing FROM.
        fm_diverge_since_ms     = now;
        fm_diverge_start_dist_m = dist_m;
      } else if ((now - fm_diverge_since_ms) >= kFmDivergeMs) {
        if (dist_m >= (fm_diverge_start_dist_m - kFmDivergeCloseEpsM)) {
          // Beyond the ceiling for the full dwell and NOT closing — this is divergence.
          // F7: the numbers are stashed and printed later, after fm_throttle_cap = 0.
          diverge_fault   = true;
          diverge_limit_m = kFmDivergeFactor * d_engage;
          diverge_start_m = fm_diverge_start_dist_m;
        } else {
          // It has closed by more than the epsilon: the buggy IS following, just far. No fault —
          // clear the window so the next tick opens a fresh one from the current distance.
          fm_diverge_since_ms     = 0;
          fm_diverge_start_dist_m = -1.0f;
        }
      }
    } else {
      // Back inside the limit, or FM is not ACTIVE — the dwell restarts from scratch.
      fm_diverge_since_ms     = 0;
      fm_diverge_start_dist_m = -1.0f;
    }
  } else {
    // No trustworthy distance this tick (trigger released, GPS stale/rejected, link down).
    // Restart the dwell rather than carrying a half-finished proof across a data gap.
    fm_sep_over_since_ms = 0;
    // A3: same discipline for the divergence dwell — never judge divergence on data we do not trust.
    // F1: the closure baseline goes with it; a baseline must never outlive the dwell that set it.
    fm_diverge_since_ms     = 0;
    fm_diverge_start_dist_m = -1.0f;
  }

  // The separation latch gates eligibility. Without it FM stays ARMED and the buggy stays
  // fully manual, no matter how well the other conditions read.
  // V2.5-Evo - 2026-07-25 - A3: !diverge_fault joins the same AND chain. It can only ever REMOVE
  // eligibility, so the worst case of a false positive is FM handing control back to the rider.
  bool can_be_active = hard_ok && speed_ok && dist_ok && fm_sep_latched && !diverge_fault;

  if (can_be_active) {
    // ---- Steer-cancel while ACTIVE (A3 PART 2 / R-steering) ----
    // If the rider is ALREADY following and applies a sustained steering input, they have taken
    // manual control: exit to ARMED and CLEAR the separation latch, so FM cannot silently resume —
    // it may only re-engage after a fresh >D_engage / kFmSepDwellMs separation is re-proven. This
    // is a rider DECLARATION, not a fault, so no alarm fires and the mode is kept. Two guards stop
    // the tail of the whip-separation steering from killing a just-engaged FM:
    //   1. kFmEngageGraceMs grace after engaging (ignore steer-cancel entirely for the first 2 s);
    //   2. kFmSteerPersistMs persistence past the deadband (a brief blip never cancels).
    // ARMED (not following) has NO steer-cancel path at all — that is what makes the whip-by-
    // steering separation safe: steering while merely armed is just steering. v1 policy = cancel;
    // the advanced steer-adjust-while-following blend (rtm_steer_exit_on_input == 0) is DEFERRED —
    // that param lives on the TX, not the RX, this pass.
    if (fm_state == FM_ACTIVE) {
      int sdev = (int)steering_received - 127;
      if (sdev < 0) sdev = -sdev;
      if (sdev >= (int)kFmSteerCancelDeadband) {
        if (fm_steer_input_since_ms == 0) fm_steer_input_since_ms = now;
      } else {
        fm_steer_input_since_ms = 0;
      }
      bool past_grace = (fm_engage_ms != 0) && ((now - fm_engage_ms) >= kFmEngageGraceMs);
      bool persisted  = (fm_steer_input_since_ms != 0) &&
                        ((now - fm_steer_input_since_ms) >= kFmSteerPersistMs);
      if (past_grace && persisted) {
        Serial.println("FM [RX] steer-cancel -> ARMED-UNLATCHED (separation latch cleared, no alarm)");
        fm_state                = FM_ARMED;
        fm_sep_latched          = false;   // deliberate: no silent resume, separation must re-prove
        fm_sep_over_since_ms    = 0;
        fm_steer_input_since_ms = 0;
        fm_rx_active            = false;
        rtm_steer_override      = 127;
        fm_engage_ms            = 0;
        fm_throttle_cap         = 255;      // manual; trigger may be held, but the latch is now clear
        return;
      }
    }

    // ---- FM_ACTIVE ----
    if (fm_state != FM_ACTIVE) {
      // Entering FM_ACTIVE from ARMED or HOLD.
      fm_engage_ms            = now;    // start the engage ramp - re-engagement is never a jump
      fm_diagonal_engaged     = false;  // re-evaluate which side we are on for this engagement
      fm_steer_input_since_ms = 0;      // ignore any pre-engagement deflection; the grace starts now

      // Reset the shared P+D derivative continuity. Without this the controller would
      // differentiate a fresh heading error against a stale pre-engagement sample across the
      // gap and command a violent phantom turn on the first tick.
      prev_heading_src_valid  = false;
      prev_heading_error_deg  = 0.0f;
      prev_steering_update_ms = 0;

      fm_state = FM_ACTIVE;
      Serial.printf("FM [RX] ENGAGE mode %u: dist=%.1f m rider=%.1f km/h course=%.0f\n",
                    (unsigned)m, dist_m, fm_rider_speed_kmh, fm_rider_course_deg);
    }

    fm_rx_active = true;                                   // gate the steering override on
    computeFmTarget(&fm_target_lat, &fm_target_lng);       // where behind the rider to sit
    updateRtmSteering();                                   // shared P+D controller, unchanged
    fm_throttle_cap = (uint8_t)fmComputeThrottleCap(dist_m, now);
  }
  else {
    // ---- Not eligible to steer — classify the drop (A3 DEADMAN / HOLD / FAULT) ----
    fm_rx_active            = false;
    rtm_steer_override      = 127;   // hand steering straight back to the rider
    fm_engage_ms            = 0;     // any re-engagement ramps from zero again
    fm_steer_input_since_ms = 0;

    bool was_engaged = (fm_state == FM_ACTIVE || fm_state == FM_HOLD);

    if ((!fault_ok || diverge_fault) && was_engaged) {
      // ---- FAULT (conditions 2-7, plus A3 divergence): something actually broke while FM had control ----
      // End autonomy for the run: enter FM_STOPPING, which ramps the throttle cap 0 -> 255 over the
      // next kFmStopRampMs (handled at the top of runFmLoop), then drops to FM_IDLE — re-arm
      // required. Fire the stop notification (sticky fm_flags bit 3, drives St + stop buzz on the
      // TX) ONLY if the trigger was held at this instant: a fault after release is not surprising,
      // and the bar going dark carries it. R4: heading loss is one of these faults now.
      // V2.5-Evo - 2026-07-25 - A3: sustained divergence enters through THIS branch and no other, so
      // it inherits the proven fault semantics unchanged — hard stop to cap 0 now, the same ramp back
      // to manual, the same haptic/St notification, and the same mandatory re-arm. Steering away is
      // exactly as much of a "something broke" event as losing the compass, and a silent auto-resume
      // after it would be the unrequested autonomy this architecture forbids.
      if (thr_held) fm_fault_alarm_ms = now;
      fm_stop_ms      = now;
      fm_state        = FM_STOPPING;
      fm_throttle_cap = 0;         // subtract-only hard stop; the ramp begins next tick
      // V2.5-Evo - 2026-07-25 - F7: ALL fault logging happens BELOW this line, never above it. The
      // divergence detail used to print at the point of detection, which is upstream of the cap write
      // — so if the USB CDC TX buffer was full (host not draining) Serial.printf() could block and
      // defer the hard stop for as long as the host took. Motor to 0 first, explain afterwards.
      if (diverge_fault) {
        Serial.printf("FM [RX] DIVERGENCE FAULT: dist=%.1f m (was %.1f m at dwell start, closed <%.1f m) > limit %.1f m sustained %lu ms — not closing\n",
                      (double)dist_m, (double)diverge_start_m, (double)kFmDivergeCloseEpsM,
                      (double)diverge_limit_m, (unsigned long)kFmDivergeMs);
      }
      Serial.printf("FM [RX] FAULT -> STOPPING (ramp %lu ms) -> IDLE, re-arm required (thr_held=%d)\n",
                    (unsigned long)kFmStopRampMs, (int)thr_held);
    } else if (was_engaged) {
      // ---- HOLD (cond 8/9) or DEADMAN (cond 1): a geometry / throttle pause, NOT a fault ----
      // Motor stops (cap 0) but FM stays ARMED (declaration held) and auto-resumes to FM_ACTIVE
      // once the conditions restore AND the latch is set. No alarm. This is what lets a rider
      // fall, slow, or close on the buggy repeatedly without ever having to re-arm.
      fm_state        = FM_HOLD;
      fm_throttle_cap = 0;         // subtract-only hard stop: motor to 0
    } else {
      // ---- FM_ARMED: never engaged this arm cycle — fully manual buggy ----
      // The throttle chain stays INACTIVE (cap 255) so the rider keeps full manual control while
      // FM waits for the follow geometry. ARMED has no steer-cancel path (see the ACTIVE branch).
      fm_state        = FM_ARMED;
      fm_throttle_cap = 255;
    }
  }
}
