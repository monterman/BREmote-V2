// V2.5-Evo - 2026-10-12 - audit 2026-10-11 delta A-1: fmLongLossStep() takes the rider's fix as the caller's verdict
//   (rider_ok - the one rule the RX's pause and fm_flags bit 2 also use) plus the rider SOLUTION counter for its reset
//   streak; FmLongLossState loses fix_ms. See section 3.
// V2.5-Evo - 2026-10-11 - FOLLOW-ME NEVER SWITCHES ITSELF OFF (owner ruling, audit 2026-10-11 "four ride fixes" change 1 +
//   §12 D-1..D-4): the pure rules behind the Follow-Me PAUSE, the 30 s LONG-LOSS clock and the HEADING CREEP.
//   Header-only, no Arduino dependencies, so Tools/tests/follow_me_pause_test.cpp exercises the exact code the RX runs.
//   Nothing here writes a cap, a steering byte or a state: the RX (RTMState.ino runFmLoopBody()) reads the verdicts
//   and does the writes, in the F7 order (cap writes before prints).
//
//   1. fmEngagedEndClass   - what an engaged Follow-Me that cannot follow this tick must do: HOLD (geometry / trigger),
//                            PAUSE (data loss - stays declared, cap 0 under a held trigger, resumes by itself), or a
//                            STEERING END (divergence or a new heading disagreement - sticky cap, ARMED-unlatched).
//   2. fmPauseReasonBit    - which missing input to name for a pause (positions and link before heading).
//   3. fmLongLossStep      - the 30 s clock: link OR rider fix OR buggy fix missing; heading never counts.
//                            (2026-10-12, A-1: the rider fix is the caller's one-rule verdict.)
//   4. fmCreepStep / fmCreepCap / fmCreepStartDistM - the straight-ahead creep that earns a stopped, COG-only buggy its
//                            GPS course (D-4), bounded exactly like RTM's BOOTSTRAP-1.
#ifndef BREMOTE_FOLLOW_ME_PAUSE_H
#define BREMOTE_FOLLOW_ME_PAUSE_H

#include <stdint.h>
#include <math.h>
#include "AutoReturnRules.h"   // kFmCond* bits, stampAgeMs(), stampStale()

// ============================================================
// 1. fmEngagedEndClass - classify a tick on which an ENGAGED Follow-Me (FM_ACTIVE or FM_HOLD) may not steer.
// ============================================================
// Inputs: was_engaged - fm_state was FM_ACTIVE or FM_HOLD; failing_mask - fmFailingConditionsMask() (kFmCond* bits);
//         heading_counts - true when a missing heading is a reason this tick cannot follow (no creep grant standing);
//         diverge_fault - the A3 divergence verdict this tick; hdg_disagree_edge - the compass-vs-COG latch ROSE this tick.
// Returns: FMEE_NONE (not engaged), FMEE_STEER_END, FMEE_PAUSE or FMEE_HOLD. Side effects: none (pure).
// WHY THIS ORDER: a steering end means the steering itself is wrong (D-3), so it outranks a data gap on the same tick.
// A data gap (any of conditions 2-7, heading only when it counts) is a PAUSE: the declaration and the separation latch
// survive and Follow-Me resumes by itself through the engage ramp. Everything else is the ordinary geometry / trigger
// HOLD it has always been. Heading is NEVER an end of anything: at worst it is a pause.
static const uint8_t FMEE_NONE      = 0;
static const uint8_t FMEE_HOLD      = 1;
static const uint8_t FMEE_PAUSE     = 2;
static const uint8_t FMEE_STEER_END = 3;
static inline uint8_t fmEngagedEndClass(bool was_engaged, uint8_t failing_mask, bool heading_counts,
                                        bool diverge_fault, bool hdg_disagree_edge)
{
  if (!was_engaged) return FMEE_NONE;
  if (diverge_fault || hdg_disagree_edge) return FMEE_STEER_END;
  const uint8_t m = heading_counts ? failing_mask : (uint8_t)(failing_mask & (uint8_t)~kFmCondHeading);
  if (m != 0) return FMEE_PAUSE;
  return FMEE_HOLD;
}

// ============================================================
// 2. fmPauseReasonBit - the ONE missing input a pause is named after.
// ============================================================
// Inputs: failing_mask (kFmCond* bits). Returns: a single kFmCond* bit, or 0 when nothing is missing.
// Order: the buggy's own GPS rejected, the handshake, the rider's GPS, the buggy's GPS, the link, and the heading
// LAST - a stopped COG-only buggy always lacks a heading, so naming it first would hide the input that is really gone.
static inline uint8_t fmPauseReasonBit(uint8_t failing_mask)
{
  static const uint8_t order[6] = { kFmCondPhaseA, kFmCondPhaseB, kFmCondTxStale, kFmCondRxStale, kFmCondLink, kFmCondHeading };
  for (int i = 0; i < 6; ++i) if (failing_mask & order[i]) return order[i];
  return 0;
}

// ============================================================
// 3. fmLongLossStep - the 30 s long-loss clock (D-2, confirmed OR of three inputs).
// ============================================================
// What it counts: the link (any packet within failsafe_time), the RIDER'S FIX (rider_ok, see below) and the BUGGY'S FIX
// (passed in as buggy_ok, gps_last_ms within 6 s). The heading and the Phase A / Phase B checks are NOT inputs: they
// pause Follow-Me but carry no timer (Phase B is re-checked only every 30 s and would trip this clock by itself).
// V2.5-Evo - 2026-10-12 - audit delta A-1: THE RIDER'S FIX IS THE CALLER'S VERDICT NOW, NOT THIS CLOCK'S OWN. It used to
// track rx_tx_gps_fix_seq here - a counter that moves only when the rider's COORDINATES change - while the pause and the
// remote's not-ready look judged the rider by ANY packet. A rider lying still (identical coordinates) therefore looked
// lost to this clock alone: no pause, no not-ready look, and "St" 30 s later out of nowhere. Now the RX passes ONE
// verdict, the same one its pause mask and fm_flags bit 2 use - "a new rider GPS SOLUTION within
// tx_gps_stale_timeout_ms" (Common/RiderGpsPacket.h: the remote's fix counter moved; an old remote: the position moved) -
// and rider_seq is that solution counter, read here only to count new solutions in the reset streak. So a still rider
// whose GPS keeps solving never starts this clock, and when it does count, the rider sees the pause the whole time.
// Starts on the first running tick on which ANY input is missing. Resets only after all three have been good together
// for reset_ms with at least reset_fixes NEW rider solutions in that streak. Fires at fire_ms of clock age.
// running - the RX is in FM_ARMED, FM_ACTIVE or FM_HOLD (IDLE, STOPPING, RETURN and the RTM yield do not run it):
//           while false the clock and the good streak are zeroed. The solution counter is read whatever `running`
//           says (its first reading is only a seed, never a new solution).
// Returns: FMLL_OFF (not running), FMLL_OK (no clock), FMLL_COUNTING, FMLL_FIRED (exactly once; the clock is zeroed).
// Side effects: *s only. Every age is stampAgeMs() (never negative), and 0 is reserved for "never".
struct FmLongLossState {
  uint32_t since_ms;        // clock start, 0 = no clock running
  uint32_t good_since_ms;   // start of the all-good streak that may reset it, 0 = none
  uint8_t  good_fixes;      // new rider solutions counted in that streak
  uint32_t seq;             // the last rider solution counter seen
  bool     seq_seen;        // seq holds a real reading
};
static const uint8_t FMLL_OFF      = 0;
static const uint8_t FMLL_OK       = 1;
static const uint8_t FMLL_COUNTING = 2;
static const uint8_t FMLL_FIRED    = 3;
static inline uint8_t fmLongLossStep(FmLongLossState* s, uint32_t now_ms, bool running,
                                     bool link_ok, bool rider_ok, uint32_t rider_seq, bool buggy_ok,
                                     uint32_t fire_ms, uint32_t reset_ms, uint8_t reset_fixes)
{
  const uint32_t stamp = (now_ms != 0) ? now_ms : 1u;
  bool new_fix = false;
  if (!s->seq_seen)                { s->seq = rider_seq; s->seq_seen = true; }   // first reading: not a new solution
  else if (rider_seq != s->seq)    { s->seq = rider_seq; new_fix = true; }

  if (!running) { s->since_ms = 0; s->good_since_ms = 0; s->good_fixes = 0; return FMLL_OFF; }

  const bool all_ok = link_ok && rider_ok && buggy_ok;
  if (s->since_ms == 0) {
    if (all_ok) return FMLL_OK;
    s->since_ms = stamp; s->good_since_ms = 0; s->good_fixes = 0;
    return FMLL_COUNTING;
  }
  if (all_ok) {
    if (s->good_since_ms == 0) { s->good_since_ms = stamp; s->good_fixes = new_fix ? 1 : 0; }
    else if (new_fix && s->good_fixes < 255) s->good_fixes = (uint8_t)(s->good_fixes + 1);
    if (stampAgeMs(now_ms, s->good_since_ms) >= reset_ms && s->good_fixes >= reset_fixes) {
      s->since_ms = 0; s->good_since_ms = 0; s->good_fixes = 0;
      return FMLL_OK;
    }
  } else {
    s->good_since_ms = 0; s->good_fixes = 0;
  }
  if (stampAgeMs(now_ms, s->since_ms) >= fire_ms) {
    s->since_ms = 0; s->good_since_ms = 0; s->good_fixes = 0;
    return FMLL_FIRED;
  }
  return FMLL_COUNTING;
}

// ============================================================
// 4. THE HEADING CREEP (D-4) - a COG-only buggy at a standstill has no heading, so it can never be steered; the creep
//    drives it STRAIGHT AHEAD, slowly and briefly, under the held trigger, until a GPS course appears.
// ============================================================
// fmCreepStartDistM - how far the rider must be (fresh fix) before a creep may START.
// Inputs: d_engage_m, min_dist_m; course_known - a good buggy heading was seen within the last 30 s; course_deg - that
//         heading; bearing_deg - the bearing from the buggy to the rider now; margin_m (5), guard_deg (30).
// Returns: max(D_engage, min_dist + margin), raised to D_engage + margin when the last good course points within
//          guard_deg of the rider (after following, the nose usually points at him).
static inline float fmCreepStartDistM(float d_engage_m, float min_dist_m, bool course_known, float course_deg,
                                      float bearing_deg, float margin_m, float guard_deg)
{
  float start = d_engage_m;
  if (min_dist_m + margin_m > start) start = min_dist_m + margin_m;
  if (course_known) {
    float diff = fmodf(fabsf(course_deg - bearing_deg), 360.0f);
    if (diff > 180.0f) diff = 360.0f - diff;
    if (diff <= guard_deg && (d_engage_m + margin_m) > start) start = d_engage_m + margin_m;
  }
  return start;
}

// fmCreepCap - the creep's own throttle ceiling: max_cap x (1 - buggy_kmh / fade_kmh), clamped to 0..max_cap.
// Inputs: buggy_kmh (< 0 = unknown -> 0), max_cap (60), fade_kmh (6 km/h = 3.7 mph). Returns: 0..max_cap.
// The RX takes min() of this with the engage ramp and the approach ramp: it is a CAP, the trigger stays the source.
static inline uint8_t fmCreepCap(float buggy_kmh, uint8_t max_cap, float fade_kmh)
{
  if (buggy_kmh < 0.0f || fade_kmh <= 0.0f) return 0;
  float frac = 1.0f - (buggy_kmh / fade_kmh);
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;
  return (uint8_t)((float)max_cap * frac);
}

// fmCreepStep - one tick of the creep grant.
// Inputs: thr_held - the trigger byte is >= 25 (the RELEASE test only: a link gap that freezes the byte is not a
//         release, so it does not re-arm a creep); heading_ok - a heading exists this tick;
//         eligible - every Follow-Me engage / stay-on condition except the heading holds, INCLUDING a fresh control
//         packet (positions fresh, separation latched, distance and rider speed met, no divergence, no sticky cap, no
//         return candidate);
//         dist_m - buggy to rider, metres; start_m - fmCreepStartDistM(); abort_m - min_dist + 1 m; max_ms (3000).
// Returns: FMCR_NONE, FMCR_GRANTED (creep this tick), FMCR_ABORT (rider too close / a condition fell / data lost),
//          FMCR_TIMEOUT (no course after max_ms). Only GRANTED lets the RX treat the tick as steerable.
// Rules: at most ONE creep per squeeze (used, cleared only by a release); a creep that ends in any way - a course
// arriving, an abort, a timeout - spends it. A course arriving is success: the RX simply follows from that tick.
struct FmCreepState {
  uint32_t start_ms;   // 0 = not creeping
  bool     used;       // a creep already ran in this squeeze
};
static const uint8_t FMCR_NONE    = 0;
static const uint8_t FMCR_GRANTED = 1;
static const uint8_t FMCR_ABORT   = 2;
static const uint8_t FMCR_TIMEOUT = 3;
static inline uint8_t fmCreepStep(FmCreepState* s, uint32_t now_ms, bool thr_held, bool heading_ok, bool eligible,
                                  float dist_m, float start_m, float abort_m, uint32_t max_ms)
{
  if (!thr_held) { s->start_ms = 0; s->used = false; return FMCR_NONE; }   // a release re-arms one creep
  if (heading_ok) { s->start_ms = 0; return FMCR_NONE; }                  // a course exists: no creep needed
  if (s->start_ms == 0) {
    if (s->used || !eligible || dist_m < start_m) return FMCR_NONE;
    s->start_ms = (now_ms != 0) ? now_ms : 1u;
    s->used     = true;
    return FMCR_GRANTED;
  }
  if (!eligible || dist_m < abort_m) { s->start_ms = 0; return FMCR_ABORT; }
  if (stampAgeMs(now_ms, s->start_ms) >= max_ms) { s->start_ms = 0; return FMCR_TIMEOUT; }
  return FMCR_GRANTED;
}

#endif // BREMOTE_FOLLOW_ME_PAUSE_H
