// V2.5-Evo - 2026-07-19 - P3 FM (DESIGN_FOLLOW_ME.md sections 4-7): Follow-Me autonomous following. Adds runFmLoop() 10Hz state machine (IDLE/ARMED/ACTIVE/DEMOTED incl. the missing 0xFF->usrConf.followme_mode fallback — SUPERSEDED 2026-07-20, see R0 below), all 9 activation/hold conditions with Schmitt hysteresis on distance and side-zone, the lag-anchor trailing target-point geometry, and the 5-stage subtract-only throttle cap chain. Reuses the existing EMA filter / P+D / heading ladder / authority / wrap pipeline unchanged - updateRtmSteering() only gains a target selector (RTM = rider position, FM = trailing point). telemetry.fm_status bit0 now reports FM engaged rather than FM mode selected. No confStruct change; SW_VERSION stays 33.
// V2.5-Evo - 2026-07-20 - FM engagement semantics (R0/R1/R2): (R0) BOTH 0xFF->usrConf.followme_mode fallbacks removed — 0xFF now means FM_IDLE always, killing the latently-armed factory boot; usrConf.followme_mode is the TX arm-gesture seed only. (R1) separation latch: FM's FIRST entry into ACTIVE now also requires dist > kFmEngageFactor(1.5) x d_follow sustained kFmSepDwellMs(2000) — the tow rope (6.7-7.6 m) is longer than the old engage distance, so FM could engage mid-tow; existing Schmitt hysteresis governs after the latch sets. (R2) two clears: thr_received<25 for kFmThrReleaseClearMs(10 s) clears the latch (ARMED-unlatched, mode memory kept); no 0xF2 refresh for kFmModeAgeMs(95 s) -> FM_IDLE. P3 geometry/cap/steering untouched. No confStruct change; SW_VERSION stays 33.
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
// (d_follow = min_dist_m + band). The tow rope is 6.7-7.6 m long, which is LONGER than
// d_follow at the intended 4+2 = 6 m tuning — so FM could engage while the rider was still
// on the rope, i.e. autonomous steering mid-tow. 1.5x gives 9 m at 4+2 tuning: 9 m clears
// the 7.6 m maximum rope with ~18% margin.
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

// ---- FM state machine (DESIGN_FOLLOW_ME.md section 4) ----
//   FM_IDLE    : FM off (mode 0), RTM owns the buggy, or GPS/FM disabled.
//                No throttle cap (255) and no steering override - fully manual buggy.
//   FM_ARMED   : a mode (1-3) is selected and all monitoring runs, but FM has not engaged
//                yet. The throttle chain is INACTIVE (cap 255) so the rider still has full
//                manual control of the buggy while FM waits for the follow geometry.
//   FM_ACTIVE  : every activation condition holds. Steering override on, throttle cap chain on.
//   FM_DEMOTED : FM was ACTIVE and a condition dropped out. This is the section-4 "FM_ARMED with
//                throttle cap 0" demotion state: the motor stops, the mode stays selected, and
//                FM re-engages through the engage ramp once conditions restore - never a jump.
//                Kept as a distinct state from FM_ARMED because the two carry different caps
//                (255 vs 0): before FM has ever engaged the rider must keep manual throttle,
//                but once FM has taken control a fault must stop the buggy.
enum FmState : uint8_t { FM_IDLE = 0, FM_ARMED = 1, FM_ACTIVE = 2, FM_DEMOTED = 3 };
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
// checkFmHardConditions - activation/hold conditions 1-7 (the fault conditions)
// ------------------------------------------------------------
// What it does (DESIGN_FOLLOW_ME.md section 5 conditions 1-7):
//   Mirrors RTM's safety gates 1-7 exactly, and for the same reasons. Any one of these
//   failing means FM must not be steering, and the caller drives the throttle cap to 0.
//   Unlike RTM this function does NOT set rtm_rx_emergency_stop - FM stops the motor through
//   its own fm_throttle_cap so the two systems can never fight over one flag.
//
// Returns: true only if all seven hold.
// Side effects: none (read-only on all globals).
// ------------------------------------------------------------
static bool checkFmHardConditions()
{
  unsigned long now = millis();

  // 1. ABSOLUTE: the rider must physically be holding the throttle trigger.
  //    Below 25 the motor is already 0 - this is normal, not a fault.
  if (thr_received < 25) return false;

  // 2. Phase A: the RX's own GPS has not been rejected as implausible/spoofed.
  if (gps_rejected) return false;

  // 3. Phase B: the TX<->RX cross-validation handshake is currently passing.
  if (!gps_phase_b_ok) return false;

  // 4. The rider's (TX) GPS position is fresh.
  if (rx_tx_gps_timestamp == 0 ||
      (now - rx_tx_gps_timestamp) > (uint32_t)usrConf.tx_gps_stale_timeout_ms) return false;

  // 5. The buggy's (RX) GPS position is fresh (same 6 s window as RTM gate 5).
  if (gps_last_ms == 0 || (now - gps_last_ms) > 6000UL) return false;

  // 6. A valid heading source exists, honouring rtm_compass_required as the gate enable.
  if (usrConf.rtm_compass_required) {
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
      if (fm_sep_latched || fm_state == FM_DEMOTED) {
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

  // ---- Evaluate the nine activation / hold conditions ----
  bool  hard_ok  = checkFmHardConditions();   // conditions 1-7 (faults)
  bool  speed_ok = false;                     // condition 9 (rider moving)
  bool  dist_ok  = false;                     // condition 8 (follow geometry)
  float dist_m   = 0.0f;

  if (hard_ok) {
    // Both GPS sources are guaranteed fresh here by conditions 4 and 5.
    dist_m = (float)TinyGPSPlus::distanceBetween(
        gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);

    float min_dist = usrConf.min_dist_m;
    float band     = usrConf.followme_smoothing_band_m;

    // Condition 9: the rider must actually be moving. Below the threshold the rider may be
    // down or swimming, and the buggy must not manoeuvre around them.
    speed_ok = (fm_rider_speed_kmh >= usrConf.foiler_low_speed_kmh);

    // Condition 8: Schmitt hysteresis on distance so FM cannot flap at the band edge.
    //   to ENGAGE  : the rider must be beyond min_dist + band
    //   to STAY ON : hold until the rider is inside min_dist
    if (fm_state == FM_ACTIVE) dist_ok = (dist_m >= min_dist);
    else                       dist_ok = (dist_m >  (min_dist + band));

    // ---- R1: separation latch (the tow interlock) ----
    // Before FM may engage for the first time this run, the rider must be proven genuinely
    // OFF THE ROPE: beyond D_engage = 1.5 x d_follow (9 m at the 4+2 tuning, clearing the
    // 7.6 m rope) continuously for kFmSepDwellMs. The dwell is what makes this spike-proof:
    // a one-fix GPS glitch cannot hold the distance high across 4 consecutive 2 Hz fixes.
    // Once latched it STAYS latched until a §R2 clear, so the buggy may close back to its
    // normal 6 m station and re-engage on the ordinary Schmitt hysteresis without ever having
    // to re-prove separation mid-wave.
    // Same degenerate-config guard computeFmTarget() applies to d_follow: a zeroed min_dist
    // and band would otherwise make D_engage 0 and the latch would set on the first tick,
    // silently disabling the whole interlock.
    float d_follow_e = min_dist + band;
    if (d_follow_e < 0.5f) d_follow_e = 0.5f;
    float d_engage = kFmEngageFactor * d_follow_e;
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
  } else {
    // No trustworthy distance this tick (trigger released, GPS stale/rejected, link down).
    // Restart the dwell rather than carrying a half-finished proof across a data gap.
    fm_sep_over_since_ms = 0;
  }

  // The separation latch gates eligibility. Without it FM stays ARMED and the buggy stays
  // fully manual, no matter how well the other nine conditions read.
  bool can_be_active = hard_ok && speed_ok && dist_ok && fm_sep_latched;

  if (can_be_active) {
    // ---- FM_ACTIVE ----
    if (fm_state != FM_ACTIVE) {
      // Entering FM_ACTIVE from ARMED or DEMOTED.
      fm_engage_ms        = now;    // start the engage ramp - re-engagement is never a jump
      fm_diagonal_engaged = false;  // re-evaluate which side we are on for this engagement

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
    // ---- Not eligible to steer: FM_ARMED (never engaged) or FM_DEMOTED (was engaged) ----
    fm_rx_active       = false;
    rtm_steer_override = 127;   // hand steering straight back to the rider
    fm_engage_ms       = 0;     // any re-engagement ramps from zero again

    if (fm_state == FM_ACTIVE || fm_state == FM_DEMOTED) {
      // FM had taken control, so a dropped condition must STOP the buggy. The rider is not
      // watching it - failing open to full manual throttle here would be unsafe.
      if (fm_state == FM_ACTIVE) {
        Serial.printf("FM [RX] DEMOTE -> cap 0 (hard=%d speed=%d dist=%d) mode held, will re-engage via ramp\n",
                      (int)hard_ok, (int)speed_ok, (int)dist_ok);
      }
      fm_state        = FM_DEMOTED;
      fm_throttle_cap = 0;      // subtract-only hard stop: motor to 0
    } else {
      // FM has never engaged this arm cycle. The throttle chain stays INACTIVE so the rider
      // keeps full manual control of the buggy while FM waits for the follow geometry.
      fm_state        = FM_ARMED;
      fm_throttle_cap = 255;
    }
  }
}
