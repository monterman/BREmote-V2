// V2.5-Evo - 2026-10-31 - R2-S SHIPS AS S1 ALONE (R2-S ruling, Revision 1.1): the RX calls fmSteerResponseStep() with s2_on false.
//   S2 (as first built and S2-w) stays here behind that flag, host-tested, because the gate-run record uses it.
// V2.5-Evo - 2026-10-31 - R2-S ruling, S2-w: FmSteerRespCfg gains s2_away_frac and s2_tickwise - S2 qualifies on 80 % of
//   its span at 150 deg or more, its rotation toward the target is summed tick by tick, and its span is 3.0 s. S1 unchanged.
// V2.5-Evo - 2026-10-31 - Follow-Me package 2 unit R2-S (design audit 2026-10-31 §E-0): fmSteerResponseStep() - THE WRONG-WAY
//   NET, judged against the buggy's own commanded turn (S1: the command held on one side while the COG turns the other way;
//   S2: pointing away from the target and not coming round). It only removes eligibility. See section 6.
// V2.5-Evo - 2026-10-30 - unit 1 of the 2026-10-30 package (Revision 2 §10, Revision 2.2 §B): fmCreepStep() gains the
//   closing abort - it stores the rider's 2-fix-mean distance at the grant and aborts once that mean is 2.0 m or more below
//   it on 2 consecutive distinct rider fixes (no mean: today's rules). FmCreepState gains start_mean_m, close_n, close_fix.
//   See section 4.
// V2.5-Evo - 2026-10-30 - audit 2026-10-20 R2 (finding F-1): fmDivergeDwellStep() - the divergence net's dwell, moved here
//   from runFmLoopBody(). Between the ceiling and 3 x D_engage a dwell ends Follow-Me only if the gap OPENED by more
//   than 2 m (running away); beyond 3 x D_engage it is the old rule (must have closed by 2 m). See section 5.
// V2.5-Evo - 2026-10-12 - audit 2026-10-11 delta A-3: fmLongLossStickyRiseOpen() - the 30 s end from a HOLD under a held
//   trigger arms the sticky cap with its rise CLOSED (stays 0 until one release). See section 3b.
// V2.5-Evo - 2026-10-12 - audit 2026-10-11 delta A-2: fmHdgDisagreeSample() / fmHdgDisagreeEnds() - the heading-disagree
//   end's edge is sampled at the top of every tick and ends only an ACTIVE engagement. See section 3c.
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
//   3b. fmLongLossStickyRiseOpen - (2026-10-12, A-3) the 30 s end from a held HOLD arms the sticky cap with its rise closed.
//   3c. fmHdgDisagreeSample / fmHdgDisagreeEnds - (2026-10-12, A-2) the heading-disagree end's edge, sampled every tick,
//                            ending only an ACTIVE engagement.
//   4. fmCreepStep / fmCreepCap / fmCreepStartDistM - the straight-ahead creep that earns a stopped, COG-only buggy its
//                            GPS course (D-4), bounded exactly like RTM's BOOTSTRAP-1.
//   5. fmDivergeDwellStep  - (2026-10-30, R2) the divergence net's 3 s dwell: running away inside 3 x D_engage, not
//                            closing beyond it.
//   6. fmSteerResponseStep - (2026-10-31, R2-S) the wrong-way net: the buggy does not turn the way it is told (S1), or
//                            points away from its target and does not come round (S2). Also run report-only on manual ticks.
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
// 3b. fmLongLossStickyRiseOpen - V2.5-Evo - 2026-10-12 - audit 2026-10-11 delta A-3 (owner ruling (a)).
// ============================================================
// How the sticky cap is armed when the 30 s long-loss end switches Follow-Me off from an ENGAGED state (in ARMED it is
// not armed at all - accepted deviation 1). THE CASE: the rider holds the trigger through a long pause, so the buggy is
// in HOLD, held at 0. The end arms the sticky cap at that 0 - and with the rise OPEN it then climbed to 60/255 in about
// 0.35 s: a stopped buggy restarting on the rider's steering without a new squeeze (and with the rider's fix lost, the
// GPS slow-down near him cannot act). THE RULE: from HOLD with the trigger held the rise starts CLOSED, so the cap stays
// at 0 until one release below 25 counts, and the buggy is plain manual after it. From ACTIVE the rise opens exactly as
// before (the cap in force, at most 60). Only this end uses it: manual, RTM and auto-return ends are unchanged.
// Inputs: from_hold (fm_state == FM_HOLD on the firing tick), thr_held (thr_received >= 25 this tick).
// Returns: true = open the clock-only rise (stickyCapArm()); false = arm it closed. Side effects: none (pure).
static inline bool fmLongLossStickyRiseOpen(bool from_hold, bool thr_held)
{
  return !(from_hold && thr_held);
}

// ============================================================
// 3c. THE HEADING-DISAGREE END'S EDGE - V2.5-Evo - 2026-10-12 - audit 2026-10-11 delta A-2.
// ============================================================
// fmHdgDisagreeSample - the ONE sampler of the D-3 edge detector. Returns true when the compass-vs-COG latch is up now
// and was down at the previous sample; *prev always ends equal to latch_now.
// THE BUG IT CLOSES: the previous value was stored only on ticks that reached the end classification, so a latch that
// rose during an auto-return, the RTM yield or IDLE (ticks that return early) was read as a FRESH edge on the first
// ordinary tick afterwards - an engagement ended for a rise it never saw. THE RX NOW SAMPLES AT THE TOP OF EVERY
// FOLLOW-ME TICK, before any early return (a rise since the previous tick is consumed by whichever tick comes next,
// early-returning or not), and once more right after the tick's own heading call (a rise inside it).
// Inputs: *prev (the stored previous sample), latch_now. Returns: rose since the previous sample. Side effects: *prev.
static inline bool fmHdgDisagreeSample(bool* prev, bool latch_now)
{
  const bool rose = latch_now && !*prev;
  *prev = latch_now;
  return rose;
}

// fmHdgDisagreeEnds - a rise ends the engagement ONLY while FM_ACTIVE (A-2). In HOLD nothing is steering and the cap is
// 0; ending there armed the sticky cap at 0 and let it climb to 60 under the held trigger - a stopped buggy restarting
// without a new squeeze, the old surge pattern. In HOLD the rise is simply consumed: the compass stays withdrawn ("COG,
// period", D-3) and Follow-Me resumes on the GPS course by its ordinary rules. No new stop, no pause change.
// Inputs: rose_this_tick (either sample above), active (fm_state == FM_ACTIVE). Returns: end now. Side effects: none.
static inline bool fmHdgDisagreeEnds(bool rose_this_tick, bool active)
{
  return rose_this_tick && active;
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
// V2.5-Evo - 2026-10-30 - unit 1 of the 2026-10-30 package (Revision 2 §10): THE CLOSING ABORT. The creep runs straight
//   with no heading, so waves can swing its nose toward the rider, or the rider can converge on it; it aborted only at
//   min_dist + 1 m (about 20 ft). Now the grant stores mean_m - the mean of the rider's 2 newest fixes' distances
//   (Common/FollowMeConverge.h fmCvRange()) - and the creep aborts once that mean is close_abort_m (2.0 m) or more below it.
//   A false abort only ends the creep (one per squeeze, as before). OWNER RULE: with no mean (mean_m < 0: the fix history
//   is missing or stale) the closing test is skipped - today's behaviour, never a looser one.
// V2.5-Evo - 2026-10-30 - Revision 2.2 §B: ON close_fixes (2) CONSECUTIVE DISTINCT FIXES. One noisy fix moved the 2-fix mean
//   2 m on about 11 % of 3 s creeps over a steady range (SD 1.0 m), ending a good creep. The test now counts distinct rider
//   fixes (mean_fix_id: a new value is a new fix - the RX passes fmCvNewestMs()) and aborts only when the mean stands
//   2 m closer on close_fixes of them in a row; a fix back above the line restarts the count.
struct FmCreepState {
  uint32_t start_ms;       // 0 = not creeping
  bool     used;           // a creep already ran in this squeeze
  float    start_mean_m;   // the 2-fix-mean distance when it was granted, < 0 = none (no closing abort)
  uint8_t  close_n;        // V2.5-Evo - 2026-10-30 - Rev 2.2 §B: consecutive distinct fixes with the mean 2 m closer
  uint32_t close_fix;      // the fix id last counted
};
static const uint8_t FMCR_NONE    = 0;
static const uint8_t FMCR_GRANTED = 1;
static const uint8_t FMCR_ABORT   = 2;
static const uint8_t FMCR_TIMEOUT = 3;
// V2.5-Evo - 2026-10-30 - unit 1: mean_m / close_abort_m added (the closing abort above). Pass mean_m < 0 for "no mean".
// V2.5-Evo - 2026-10-30 - Rev 2.2 §B: mean_fix_id (which rider fix the mean is from) and close_fixes (2) added.
static inline uint8_t fmCreepStep(FmCreepState* s, uint32_t now_ms, bool thr_held, bool heading_ok, bool eligible,
                                  float dist_m, float start_m, float abort_m, uint32_t max_ms,
                                  float mean_m, float close_abort_m, uint32_t mean_fix_id, uint8_t close_fixes)
{
  if (!thr_held) { s->start_ms = 0; s->used = false; return FMCR_NONE; }   // a release re-arms one creep
  if (heading_ok) { s->start_ms = 0; return FMCR_NONE; }                  // a course exists: no creep needed
  if (s->start_ms == 0) {
    if (s->used || !eligible || dist_m < start_m) return FMCR_NONE;
    s->start_ms     = (now_ms != 0) ? now_ms : 1u;
    s->used         = true;
    s->start_mean_m = (mean_m >= 0.0f) ? mean_m : -1.0f;
    s->close_n      = 0;
    s->close_fix    = mean_fix_id;
    return FMCR_GRANTED;
  }
  if (!eligible || dist_m < abort_m) { s->start_ms = 0; return FMCR_ABORT; }
  if (s->start_mean_m >= 0.0f && mean_m >= 0.0f && mean_fix_id != s->close_fix) {   // unit 1: closing, per distinct fix
    s->close_fix = mean_fix_id;
    if (mean_m <= s->start_mean_m - close_abort_m) { if (s->close_n < 255) s->close_n++; }
    else s->close_n = 0;
    if (s->close_n >= close_fixes) { s->start_ms = 0; return FMCR_ABORT; }
  }
  if (stampAgeMs(now_ms, s->start_ms) >= max_ms) { s->start_ms = 0; return FMCR_TIMEOUT; }
  return FMCR_GRANTED;
}

// ============================================================
// 5. fmDivergeDwellStep - ONE TICK OF THE DIVERGENCE NET'S DWELL (V2.5-Evo - 2026-10-30 - audit 2026-10-20 R2, F-1)
// ============================================================
// What it does: judges whether an ACTIVE Follow-Me buggy beyond its distance ceiling is running away from the rider.
// THE BUG IT FIXES (F-1): beyond the ceiling (2 x D_engage, 79 ft at the owner's 12 m) the old rule ended Follow-Me
//   unless the buggy had CLOSED by kFmDivergeCloseEpsM (2 m) over the 3 s dwell - a closing speed of +1.5 mph. Its
//   design note assumed the buggy really runs at rider + 3.1 mph; the speed governor never gets there (it is zero at its
//   own target), so a buggy pacing the rider correctly at 80-110 ft was "not closing" and was switched off: 8 of 14
//   stretches on 19 Oct, 4 of 9 engagements on 30 Oct - each one the stop buzz and then the 24 % sticky cap under a held
//   trigger (53 s at about 5 mph on 30 Oct).
// THE RULE NOW: between the ceiling and far_m (3 x D_engage, 118 ft) a dwell ends Follow-Me only if the gap OPENED by
//   more than open_eps_m (2 m: a +1.5 mph opening rate) - "running away", not "not closing fast enough". Beyond far_m
//   it is the old rule (must have closed by close_eps_m). Which rule applies is read at the dwell's end, from the
//   distance then, so a buggy that has slipped past far_m is judged by the old rule at once. If far_m is not above the
//   ceiling (a front station can raise the ceiling) the old rule applies everywhere beyond the ceiling.
// STILL CAUGHT: wrong steering (an inverted or 180 deg course opens the gap at about the rider's speed or more), and a
//   rider outrunning the buggy by more than 1.5 mph. A buggy holding a constant 80-118 ft is left following; the
//   trigger, the stick takeover and the 30 s long-loss clock still apply to it.
// Bookkeeping is the shape the RX always used: one start stamp plus the distance it started from, both cleared the
//   instant the buggy is not judged (parked for the engage grace, a pivot or a stick takeover; not ACTIVE; no
//   trustworthy distance) or is back inside the ceiling, and cleared after a dwell that does not end Follow-Me, so the
//   next tick opens a fresh one. On a verdict the state is LEFT standing (the RX's steering end clears it and prints
//   d->start_m first).
// Inputs: *d - the RX's fm_diverge_since_ms / fm_diverge_start_dist_m (0 / -1 = no dwell); now_ms; judged - FM_ACTIVE
//         and not parked; dist_m - buggy to rider; ceiling_m - the divergence ceiling in force; far_m - 3 x D_engage;
//         dwell_ms (3000); close_eps_m (2.0, beyond far_m); open_eps_m (2.0, between the ceiling and far_m).
// Returns: FMDV_NONE, FMDV_NOT_CLOSING (beyond far_m, the old rule) or FMDV_OPENING (inside the band, running away).
// Side effects: *d only. Nothing here writes a cap: a verdict only REMOVES eligibility (the RX's steering end).
struct FmDivergeDwell {
  uint32_t since_ms;   // dwell start, 0 = no dwell
  float    start_m;    // distance at the dwell start, -1 = none
};
static const uint8_t FMDV_NONE        = 0;
static const uint8_t FMDV_NOT_CLOSING = 1;
static const uint8_t FMDV_OPENING     = 2;
static inline uint8_t fmDivergeDwellStep(FmDivergeDwell* d, uint32_t now_ms, bool judged, float dist_m,
                                         float ceiling_m, float far_m, uint32_t dwell_ms,
                                         float close_eps_m, float open_eps_m)
{
  if (!judged || !(dist_m > ceiling_m)) { d->since_ms = 0; d->start_m = -1.0f; return FMDV_NONE; }
  if (d->since_ms == 0) { d->since_ms = now_ms; d->start_m = dist_m; return FMDV_NONE; }   // first tick beyond
  if ((uint32_t)(now_ms - d->since_ms) < dwell_ms) return FMDV_NONE;                     // still dwelling
  if (dist_m <= far_m) {
    if (dist_m > d->start_m + open_eps_m) return FMDV_OPENING;          // R2: running away
  } else {
    if (dist_m >= d->start_m - close_eps_m) return FMDV_NOT_CLOSING;    // beyond far_m: the old rule
  }
  d->since_ms = 0; d->start_m = -1.0f;                                  // following: a fresh window next tick
  return FMDV_NONE;
}

// ============================================================
// 6. fmSteerResponseStep - R2-S, THE WRONG-WAY NET (V2.5-Evo - 2026-10-31 - design audit 2026-10-31 §E-0, Follow-Me package 2)
// ============================================================
// WHAT IT JUDGES: not where the station is, but whether the buggy TURNS THE WAY IT IS TOLD. A healthy buggy's distance to
//   a moving station grows in every rider turn, so a distance net must wait; a mirrored-steering buggy is told "right" and
//   turns left, at any speed, at any station, and that shows within a few seconds. This net only REMOVES eligibility: the
//   RX ends Follow-Me through R2's existing steering end (sticky cap, needs-D_engage, stop buzz). It never adds throttle.
// THE TWO RULES (all thresholds from *cfg; the RX's numbers are kFmSteerRespCfg):
//   S1 "TURNS THE WRONG WAY": the steering command (the controller's byte - 127, BEFORE the mixer's steering_inverted) has
//      been on ONE side of centre by at least cmd_min counts on every sample of the last lead_ms + win_ms (0.5 s lead-in, so
//      the VESC ramp and the buggy's momentum have begun to respond, then a 2.0 s window); in the window the buggy's COG was
//      valid on at least valid_frac of the samples and the applied throttle was at least the caller's limit on at least
//      valid_frac of them; and over the window the COG rotated rot_min_deg or more THE OPPOSITE WAY (unwrapped).
//      Sign convention: command > 0 = steer right = the COG should increase (clockwise).
//   S2 "POINTS AWAY AND DOES NOT COME ROUND" (when s2_on): for the last s2_ms every sample has a heading error to the target
//      of at least s2_err_deg, COG and throttle valid on valid_frac of them, and the COG has rotated less than s2_rot_deg
//      toward the side the target was on at the start of that span.
//      V2.5-Evo - 2026-10-31 - S2-w: "every sample" is now s2_away_frac of them (0.8), the rotation is summed tick by tick
//      (s2_tickwise), and the span is 3.0 s - see the S2 block below. The first-built S2 is s2_away_frac 1.0, s2_tickwise false. This catches the steady mirrored state, where the error
//      sits near 180 deg and the command chatters from side to side, so S1's "one side throughout" never holds.
// NOT JUDGED (judged == false): outside FM_ACTIVE, a stick takeover and 1 s after it, the RTM yield, the first 0.5 s after
//   the ACTIVE edge - the caller decides. A sample that is not judged CLEARS the history, so every verdict rests on an
//   unbroken judged span (the first verdict can come lead_ms + win_ms after judging starts). No engage grace beyond that:
//   the grace exists because a distance net measures the ramp; a response-sign test does not.
// TIME-BASED: the windows are in milliseconds, so the RX's 10 Hz tick and a 3 Hz log replay run the same rule. The history
//   holds kFmSrLen samples (6.4 s at 10 Hz); a span is judged only if the history reaches back over all of it.
// Inputs: *r (state); now_ms; judged; cmd_off - command - 127 (counts, -127..+127); cog_ok / cog_deg - the buggy's GPS
//   course and whether it may be used (moving >= 5 km/h, fresh, not frozen); thr_ok - the applied throttle is at least the
//   caller's limit (30 counts); err_ok / err_deg - the signed heading error to the target (target bearing - heading,
//   -180..180; err_ok false = none this tick); *cfg; s2_on.
// Returns: FMSR_NONE, FMSR_WRONG_WAY (S1) or FMSR_POINTING_AWAY (S2). S1 is tested first.
// Side effects: *r only. Nothing here writes a cap, a steering byte or a state.
static const uint8_t FMSR_NONE          = 0;
static const uint8_t FMSR_WRONG_WAY     = 1;   // S1
static const uint8_t FMSR_POINTING_AWAY = 2;   // S2
struct FmSteerRespCfg {
  int16_t  cmd_min;      // counts beyond centre that count as "steering to one side" (32 = 25 %)
  uint32_t lead_ms;      // S1 lead-in with the command already on that side (500)
  uint32_t win_ms;       // S1 window (2000)
  float    rot_min_deg;  // S1: the wrong-way rotation that ends it (15)
  float    valid_frac;   // COG / throttle valid on at least this share of the window's samples (0.8)
  float    s2_err_deg;   // S2: |heading error to the target| at least this (150)
  uint32_t s2_ms;        // S2 span (V2.5-Evo - 2026-10-31 - S2-w: 3000; as first built 4000)
  float    s2_rot_deg;   // S2: rotation toward the target's side under this (15)
  // V2.5-Evo - 2026-10-31 - S2-w (design audit 2026-10-31, R2-S ruling): S2 made robust to the 180 deg wrap.
  float    s2_away_frac; // S2: the share of the span's samples that must have an error of at least s2_err_deg (S2-w 0.8;
                         //     as first built 1.0 = every sample)
  bool     s2_tickwise;  // S2: rotation toward the target summed TICK BY TICK, each course change signed by the older
                         //     sample's error (S2-w true); false = the whole span against the side of its first sample
};
static const uint8_t kFmSrLen = 64;
struct FmSteerRespSample {
  uint32_t t_ms;
  int8_t   side;         // +1 / -1: the command is beyond cmd_min on that side; 0: not
  bool     cog_ok;
  bool     thr_ok;
  bool     err_ok;
  float    cog_unw;      // the unwrapped COG at this sample (held through invalid samples)
  float    err_deg;
};
struct FmSteerResp {
  FmSteerRespSample s[kFmSrLen];
  uint8_t  n;            // samples held, 0..kFmSrLen
  uint8_t  head;         // index of the newest
  bool     have_cog;     // last_cog holds a valid COG
  float    last_cog;     // the last valid COG (deg)
  float    unw;          // the running unwrapped COG (deg)
};

// fmSteerRespReset - forget the history (a sample that is not judged, or a fresh engagement). Side effects: *r.
static inline void fmSteerRespReset(FmSteerResp* r)
{
  r->n = 0; r->head = 0; r->have_cog = false; r->last_cog = 0.0f; r->unw = 0.0f;
}

// fmSrAt - the sample k back from the newest (0 = newest). The caller keeps k < r->n.
static inline const FmSteerRespSample* fmSrAt(const FmSteerResp* r, uint8_t k)
{
  return &r->s[(uint8_t)((r->head + kFmSrLen - k) % kFmSrLen)];
}

// fmSrWindow - over the samples no older than span_ms (newest first): how many there are, how many have a valid COG and a
// valid throttle, the unwrapped COG of the oldest and the newest valid-COG sample (*rot = newest - oldest, 0 with fewer than
// two), and whether the history reaches back over the whole span (an older sample exists, or the oldest kept is exactly
// span_ms old). Returns the count. Pure.
static inline uint8_t fmSrWindow(const FmSteerResp* r, uint32_t now_ms, uint32_t span_ms, uint8_t* n_cog, uint8_t* n_thr,
                                 float* rot, bool* covered, uint8_t* oldest_k)
{
  uint8_t n = 0, nc = 0, nt = 0; bool have_new = false; float u_new = 0.0f, u_old = 0.0f;
  *covered = false; *oldest_k = 0;
  for (uint8_t k = 0; k < r->n; k++) {
    const FmSteerRespSample* p = fmSrAt(r, k);
    const uint32_t age = (uint32_t)(now_ms - p->t_ms);
    if (age > span_ms) { *covered = true; break; }   // an older sample: the history spans the whole window
    if (age == span_ms) *covered = true;
    n++; *oldest_k = k;
    if (p->cog_ok) { nc++; if (!have_new) { have_new = true; u_new = p->cog_unw; } u_old = p->cog_unw; }
    if (p->thr_ok) nt++;
  }
  *n_cog = nc; *n_thr = nt;
  *rot = (nc >= 2) ? (u_new - u_old) : 0.0f;
  return n;
}

static inline uint8_t fmSteerResponseStep(FmSteerResp* r, uint32_t now_ms, bool judged, int16_t cmd_off, bool cog_ok,
                                          float cog_deg, bool thr_ok, bool err_ok, float err_deg,
                                          const FmSteerRespCfg* cfg, bool s2_on)
{
  if (!judged) { fmSteerRespReset(r); return FMSR_NONE; }
  // The unwrapped COG: each valid sample adds its shortest signed change from the last valid one.
  if (cog_ok) {
    if (r->have_cog) {
      float d = cog_deg - r->last_cog;
      while (d > 180.0f) d -= 360.0f;
      while (d < -180.0f) d += 360.0f;
      r->unw += d;
    }
    r->last_cog = cog_deg; r->have_cog = true;
  }
  r->head = (uint8_t)((r->head + 1) % kFmSrLen);
  if (r->n < kFmSrLen) r->n++;
  FmSteerRespSample* p = &r->s[r->head];
  p->t_ms    = now_ms;
  p->side    = (cmd_off >= cfg->cmd_min) ? (int8_t)1 : (cmd_off <= -cfg->cmd_min) ? (int8_t)-1 : (int8_t)0;
  p->cog_ok  = cog_ok;
  p->thr_ok  = thr_ok;
  p->err_ok  = err_ok;
  p->cog_unw = r->unw;
  p->err_deg = err_deg;

  // ---- S1: one side throughout the lead-in and the window, and the COG turned the other way ----
  {
    const int8_t side = p->side;
    uint8_t nc = 0, nt = 0, ok_k = 0; float rot_all = 0.0f; bool covered = false;
    const uint8_t n_span = fmSrWindow(r, now_ms, cfg->lead_ms + cfg->win_ms, &nc, &nt, &rot_all, &covered, &ok_k);
    bool one_side = (side != 0) && covered && n_span > 0;
    for (uint8_t k = 0; one_side && k < n_span; k++) if (fmSrAt(r, k)->side != side) one_side = false;
    if (one_side) {
      float rot = 0.0f; bool cov_w = false;
      const uint8_t n_w = fmSrWindow(r, now_ms, cfg->win_ms, &nc, &nt, &rot, &cov_w, &ok_k);
      const bool enough = (n_w > 0) && ((float)nc >= cfg->valid_frac * (float)n_w) && ((float)nt >= cfg->valid_frac * (float)n_w);
      if (enough && (float)side * rot <= -cfg->rot_min_deg) return FMSR_WRONG_WAY;
    }
  }
  // ---- S2: pointing away over the span and not coming round ----
  // V2.5-Evo - 2026-10-31 - S2-w (design audit 2026-10-31, R2-S ruling). THE GAP: every mirrored run ends in the same state -
  // pointing away from its target with the command chattering across the 180 deg wrap - and S2 as first built needed EVERY
  // sample of a 4 s span at 150 deg or more, so each chatter dip or station switch restarted it (gate 1 failed at F4 / F5).
  // S2-w: (1) the span qualifies when at least s2_away_frac (80 %) of its samples have an error of 150 deg or more; (2) the
  // rotation toward the target is summed tick by tick - each consecutive pair of valid-course samples adds (newer course -
  // older course) x the sign of the OLDER sample's error, only where the older sample has an error - because once samples
  // from both sides of 180 deg are admitted, "the side at the start of the span" would count a healthy buggy turning toward a
  // target that read -179 then +179 as not coming round; (3) the span is 3.0 s. With s2_away_frac 1.0 and s2_tickwise false
  // this block is the first-built S2 exactly.
  if (s2_on) {
    uint8_t nc = 0, nt = 0, old_k = 0; float rot = 0.0f; bool covered = false;
    const uint8_t n_span = fmSrWindow(r, now_ms, cfg->s2_ms, &nc, &nt, &rot, &covered, &old_k);
    uint8_t n_away = 0;
    for (uint8_t k = 0; k < n_span; k++) {
      const FmSteerRespSample* q = fmSrAt(r, k);
      if (q->err_ok && fabsf(q->err_deg) >= cfg->s2_err_deg) n_away++;
    }
    const bool away = covered && n_span > 0 && (float)n_away >= cfg->s2_away_frac * (float)n_span;
    if (away) {
      const bool enough = ((float)nc >= cfg->valid_frac * (float)n_span) && ((float)nt >= cfg->valid_frac * (float)n_span);
      float toward = 0.0f;
      if (cfg->s2_tickwise) {
        // oldest to newest over the span: each consecutive pair of valid-course samples, signed by the older one's error
        const FmSteerRespSample* older = nullptr;
        for (int k = (int)old_k; k >= 0; k--) {
          const FmSteerRespSample* q = fmSrAt(r, (uint8_t)k);
          if (!q->cog_ok) continue;
          if (older != nullptr && older->err_ok) toward += (q->cog_unw - older->cog_unw) * ((older->err_deg >= 0.0f) ? 1.0f : -1.0f);
          older = q;
        }
      } else {
        toward = ((fmSrAt(r, old_k)->err_deg >= 0.0f) ? 1.0f : -1.0f) * rot;   // the side the target was on at the start
      }
      if (enough && toward < cfg->s2_rot_deg) return FMSR_POINTING_AWAY;
    }
  }
  return FMSR_NONE;
}

#endif // BREMOTE_FOLLOW_ME_PAUSE_H
