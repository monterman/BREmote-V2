// V2.5-Evo - 2026-10-30 - Host test, unit 1 of the 2026-10-30 Follow-Me package (Revision 2 §5 and §10, as amended by
//   Revision 2.1 §B and cut to STOPS ONLY by Revision 2.2 §B / §C / §D): THE FAST-CLOSURE CEILING
//   (Common/FollowMeConverge.h) with its stop qualifier, its divergence park, the heading creep's closing abort, and the
//   gates of Revision 2.2 §C in a closed-loop simulation. The two stop legs run side by side on the same seeds:
//     S-a - the Rev 2.1 leg: his raw speed 30 % down within 1.5 s, one estimate;
//     S-b - noise-proofed: at or under 60 % of his 2 s maximum on 2 consecutive estimates;
//   both with closing >= 1 m/s, a 2 s maximum >= 2.5 m/s, exit at 80 % on 2 estimates or closing under 0.5 m/s for 1 s.
//   Section 9 applies the selection rule (ship S-a if it passes every HARD gate, else S-b; else neither).
//   STATUS (Revision 2.3: S-b ships, RTMState.ino kFmCvQualCfg == kQualSb): section 1 (the pure rules) is ASSERTED, and
//   section 9 ASSERTS every HARD gate for S-b, the shipped leg. S-a (rejected) is still run beside it and reported, so
//   the record of why it was rejected stays reproducible.
//   1. Unit checks: the range history, closing rate, v_max, v_los, the verdict, the re-rise limit, the park, the rider's
//      raw speed, both stop legs and both exits, the inputs-bad fallback (no ceiling - today's behaviour, never looser),
//      and the creep's closing abort on 2 distinct fixes (with its false-abort rate on a steady range, reported).
//   2-8. A CLOSED-LOOP SIMULATION of Follow-Me ACTIVE, restated in runFmLoopBody()'s order, on today's flown governor and
//      side zone, flown against S-a and S-b (and the unqualified ceiling where a gate compares with it):
//        2 G-STOP (HARD) rider stops dead (-9 mph/s): >= 16.4 ft at 9.8-14 mph everywhere and at 8 mph F1/F3/F4/F5; 8 mph
//                 F2 no worse than flown on both fits (Rev 2.3; the owner's residual under option A); the STOP EFFECT
//                 >= min(16.4, unqualified - 1.5 ft) per phase;
//        3-5 HARSH, REALISTIC, PAIRS (HARD, "no harm"): closest >= min(16.4, flown - 0.5 ft) and band entry <= flown +
//                 0.3 mph per case; G-UT4 R2 ends <= flown, none in the park. F4/F5 cases with an R2 end are "manual"
//                 (reported; the front-station work, not in this package, owns them). Own-line labels and the cost
//                 against the unqualified ceiling reported;
//        6 G-UT5 STOP-AND-GO (HARD) 8 -> 3 mph, 1 s, back to 8: the ceiling gone <= 3 s after his speed is back to 80 %;
//          G-UT6(a) PICKUP (HARD) he turns back, rides at the buggy and slows at -2 mph/s to stop 10 / 20 / 30 ft from it:
//                 no harm, and the buggy's distance 2-8 s after he stops never more than 1.5 ft FARTHER than flown
//                 (Rev 2.3; nearer ends are judged by the closest-approach no-harm rule); the FLY-BY (report);
//        7 G-UT6(b) (HARD) under 1 % binding: steady, catch-up from behind, carving +/-45 / 60 / 75 / 90 deg; F1-F5;
//                 8-14 mph;
//        8 red edits (simulation level).
//      Plant: hard gates on a coast-down tau of 1.52 s (the p75 of the 13 measured coasts); 1.33 s (median) and 2.4 s
//      (the old speed-response fit) reported. The controller's tau_c is kFmCvCoastS = 1.6 s.
//   THE SIMULATION: 10 Hz FM ticks. The rider's fix at 3 Hz, reported 1 s late, with a 2 m slow bias (rotating over
//   90 s) and +/-3 ft white noise; the RX's 2 s position filter, speed and course over >= 0.4 s, the raw rider speed over
//   >= 1 s; the range history sampled per new rider solution (the RX's rule); conditions 8 / 9 and the raw-speed return
//   candidate (HOLD, cap 0); R2's divergence dwell (fmDivergeDwellStep) with the grace, the pivot suspension and - in the
//   unit-1 arms - the closure park, and the steering end (sticky cap 60, steering straight); the station geometry by
//   ctlTick() (the station test's restatement of computeFmTarget(), below, with the pre-R3 side zone, the shield, PG-2/4,
//   H-1); caps 2, 3 (restated from fmComputeThrottleCap(), the flown law plus unit 1), 4 (align 50 above 45 deg) and 5
//   (the engage ramp); the trigger held at max throughout; the rider scripted (he holds his line). The buggy: steady speed
//   for a throttle from the two fits (19 Oct: 15.5 mph, n 2; 30 Oct: 16.2 mph, n 1.36), speeding up over 2.4 s and
//   coasting down over the plant tau; yaw-limited steering. Not modelled: the RETURN leg, the pivot's own motion.
//   Owner's RX1 config: min_dist 5 m (16.4 ft), band 4 m, D_engage 12 m, near_diag 60, zone 35 / 45, foiler_low 8 km/h,
//   vmax 40 km/h, align 45 deg / cap 50, steer response 2 (2 s), auto-return on, a heading always available.
// Build and run:  g++ -std=c++17 -O2 -Wall -Wextra -o follow_me_converge_test follow_me_converge_test.cpp && ./follow_me_converge_test
//   ./follow_me_converge_test quick    - sections 1-2 only
//   ./follow_me_converge_test verbose  - every U-turn case on its own line (closest, band entry, own-line label)
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "../../Source/Common/FollowMeStation.h"
#include "../../Source/Common/FollowMeEngage.h"
#include "../../Source/Common/FollowMePause.h"
#include "../../Source/Common/FollowMeConverge.h"


// ======================================================================================
// THE STATION CHAIN: restated verbatim from Tools/tests/follow_me_station_test.cpp (its constants, Ctl, Tick, ctlTick(),
// engageEdge(), degradedTick()), with three outputs added to Tick for the cap chain (st_al / st_cr / outward).
// ======================================================================================
static const float kFrontDefault   = 45.0f;    // derived angle at ahead == side (13 m x 13 m)
static const float kFrontMin       = 35.0f;    // kFmFrontAngleMinDeg - floor on the DERIVED angle
static const float kFrontMax       = 45.0f;    // kFmFrontAngleMaxDeg - ceiling on the DERIVED angle (80 until audit M-12: ahead >= side now)
static const float kLateralMin     = 13.0f;    // kFmFrontLateralMinM  (carve 8 m + relative GPS 5 m)
static const float kPassLateral    = 10.1f;    // kFmPassLateralM      (rope 7.1 m + 3 m)
static const float kExtraDefault   = 7.0f;     // kFmFrontAheadExtraDefaultM
static const float kExtraMin       = 4.0f;     // kFmFrontAheadExtraMinM
static const float kExtraMax       = 10.0f;    // kFmFrontAheadExtraMaxM
static const float kNearDiag       = 45.0f;    // usrConf.near_diag_offset_deg, owner default
static const float kSlewRate       = 15.0f;    // kFmStationRateDegPerS (provisional)
static const float kSideBand       = 2.0f;     // kFmSideHysteresisM (provisional, audit H-3)


static const float kShHorizonS  = 5.0f;    // kFmShieldHorizonS
static const float kShHalfDeg   = 30.0f;   // the fixed half-angle before audit M-16 b (still used by the geometry-only sections)
static const float kShMinKmh    = 12.0f;   // kFmShieldMinSpeedKmh (the cone's ON threshold)
static const float kShOffKmh    = 10.0f;   // kFmShieldConeOffKmh  (the cone's OFF threshold, audit M-15)
static const float kShCircleM   = 3.0f;    // kFmShieldCircleMinM
static const float kThetaMinM   = 3.0f;    // kFmThetaMinSepM
static const float kSettleDeg   = 5.0f;    // kFmStationSettleDeg
static const float kAbortDeg    = 60.0f;   // kFmTransitAbortCourseDeg
static const float kZoneEnter   = 35.0f;   // usrConf.zone_angle_enter_deg, owner default
static const float kZoneExit    = 45.0f;   // usrConf.zone_angle_exit_deg, owner default
static const uint8_t kPrRetreat = 10, kPrOutward = 9, kPrTransit = 7, kPrFront = 8;
static const float kLookStepM   = 1.0f;    // kFmLookStepM (audit M-14): a bigger one-tick lookahead change is a step

struct Ctl {
  float live = 0.0f;  bool aborted = false;  int rside = 0;
  bool transit = false;  float tcourse = -1.0f;
  bool escape = false;  int kind_prev = 0;  bool pending = false;  bool diag = false;
  bool walking = false;  int shside = 0;
  // V2.5-Evo - 2026-10-06 - audits M-15 / M-14 / M-18: the cone gate's Schmitt state, last tick's hold state,
  // and the lag shortfall the shield is extended by (an input - the RX computes it from the preset's tau).
  bool cone_on = false;  bool held_prev = false;  float shortfall = 0.0f;
  float look_prev = 0.0f;                                // the clipped lookahead last tick (M-14 output check)
  int ticks = 0;                                         // ticks run on this state (the first has no previous lookahead)
  // V2.5-Evo - 2026-10-06 - the settled-escape latch: a millis() stand-in (advanced by dt each tick) and the timer.
  uint32_t now_ms = 0;  uint32_t settled_since = 0;
};
struct Tick {
  bool snapped = false, escape = false, h1 = false, sh_reseed = false, step = false, latched = false;
  int kind = 0;  uint8_t profile = 0;
  float b_al = 0, b_cr = 0;        // the buggy, anchor frame
  float sh_al = 0, sh_cr = 0;      // the segment end the shield tested
  float aim_al = 0, aim_cr = 0;    // the aim steered at, anchor frame
  FmShield sh{};
  float hold = 0, half = 0;        // V2.5-Evo - 2026-10-06 - the hold band and the half-angle used this tick
  bool cone_toggled = false;
  float look = 0, dlook = 0;       // the clipped lookahead this tick, and its change from last tick
  bool settled_latched = false;    // the 3 s settled-escape latch fired this tick
  // V2.5-Evo - 2026-10-30 - added for the closure simulation: the station point (anchor frame, before the lookahead)
  // and PG-4's outward escape - the RX's fm_station_along_m / fm_station_cross_m and fm_aim_outward.
  float st_al = 0, st_cr = 0;  bool outward = false;
};
static float g_dlook_max_quiet = 0.0f;   // the largest one-tick lookahead change on a tick nothing else flagged

static Tick ctlTick(Ctl &c, uint8_t m_decl, float near_diag, float d_follow, uint16_t extra_stored,
                    float min_dist, float speed_kmh, float course, float b_al_f, float b_cr,
                    float lag_m, float dt, float band)
{
  Tick t;
  c.now_ms += (uint32_t)(dt * 1000.0f + 0.5f);
  float pass = kPassLateral;  if (pass < min_dist) pass = min_dist;
  const float latmin = fmFrontSideFloorM(kLateralMin, pass, min_dist, kSideBand);   // audit M-20, as the RX
  const float extra = fmFrontAheadExtraM(extra_stored, kExtraDefault, kExtraMin, kExtraMax);
  float ahead, phi, rf;
  fmFrontStationGeom(d_follow, extra, latmin, kFrontMin, kFrontMax, &ahead, &phi, &rf, nullptr);
  const float lim = fmStationLimitDeg(phi);

  const float brg_rel = atan2f(b_cr, b_al_f) / BREMOTE_FMS_DEG2RAD;   // bearing of the buggy, off the course
  const float off_axis = fabsf(fmsWrap180(brg_rel - 180.0f));
  if (!c.diag && off_axis < kZoneEnter) c.diag = true;
  else if (c.diag && off_axis > kZoneExit) c.diag = false;
  const float meas = fmMeasuredStationDeg(course, course + brg_rel);

  uint8_t m_eff = m_decl;
  if (c.aborted && (m_decl == 4 || m_decl == 5)) {
    if (c.rside != 0) m_eff = (uint8_t)((c.rside > 0) ? 1 : 3);
    else              m_eff = (uint8_t)((m_decl == 4) ? 3 : 1);   // 2026-10-20: 4 front-left -> 3, 5 front-right -> 1
  }
  if (c.pending) {                                                   // M-11
    c.pending = false;
    const float sd = fmEngageSeedDeg(m_eff, true, meas);
    if (sd != c.live) { c.live = sd; t.step = true; if (sd != 0.0f) c.walking = true; }   // + M-17
  }
  float psi_target = fmStationPresetDeg(m_eff, near_diag, phi);
  if ((m_eff == 1 || m_eff == 3) && !c.diag) psi_target = 0.0f;
  psi_target = fmClampStationDeg(psi_target, phi);

  const bool want_front = fabsf(psi_target) > 90.0f;                // G-3
  if (want_front && fabsf(psi_target - c.live) > kSettleDeg) {
    if (!c.transit) { c.transit = true; c.tcourse = course; }
  } else {
    c.transit = false;
  }
  if (c.transit && c.tcourse >= 0.0f && fabsf(fmsWrap180(course - c.tcourse)) > kAbortDeg) {
    c.aborted = true; c.transit = false;
    m_eff = (uint8_t)((m_decl == 4) ? 3 : (m_decl == 5) ? 1 : m_decl);   // 2026-10-20: same side, as the RX
    psi_target = fmStationNearestRearPresetDeg(m_decl, near_diag);
    if ((m_eff == 1 || m_eff == 3) && !c.diag) psi_target = 0.0f;
    psi_target = fmClampStationDeg(psi_target, phi);
  }

  const int h1 = fmFrontRetreatSide(c.live, fabsf(psi_target) > 90.0f, b_cr, band);   // H-1 + H-3
  if (h1 != 0) {
    t.h1 = true; t.step = true;
    c.live = fmRetreatSeedDeg(h1, meas, lim);
    c.transit = false;
    if (m_decl == 4 || m_decl == 5) {
      c.aborted = true; c.rside = h1;
      m_eff = (uint8_t)((h1 > 0) ? 1 : 3);
      psi_target = fmStationPresetDeg(m_eff, near_diag, phi);
      if (!c.diag) psi_target = 0.0f;
      psi_target = fmClampStationDeg(psi_target, phi);
    }
  }

  const float cm  = fmStationTargetCeilingDeg(psi_target, b_cr, pass, lim);               // PG-2
  const float pet = (psi_target >= 0.0f) ? cm : -cm;
  // V2.5-Evo - 2026-10-06 - audits M-16 b / M-15 / M-18 / M-16 c, in the RX's order.
  const bool cone_on = fmShieldConeOn(c.cone_on, speed_kmh, kShMinKmh, kShOffKmh);
  t.cone_toggled = (cone_on != c.cone_on);
  c.cone_on = cone_on;
  t.half = fmShieldHalfAngleDeg(speed_kmh);
  t.sh   = fmShieldMake(speed_kmh, kShHorizonS, t.half, cone_on ? kShOffKmh : kShMinKmh, kShCircleM, min_dist, d_follow);
  fmShieldExtendForLag(t.sh, c.shortfall);
  t.hold = fmShieldHoldBandM(rf, phi, t.half, band);
  t.b_al = b_al_f - lag_m;
  t.b_cr = b_cr;
  const float infl = c.escape ? t.hold : 0.0f;
  bool snap = fmRearSnap(m_eff, c.live, near_diag);
  if (snap && c.walking) {                                    // a walk's first snap answers to the shield
    float s_al, s_cr;
    fmStationAlongCrossM(pet, d_follow, &s_al, &s_cr);
    if (fmShieldSegmentHits(t.sh, t.b_al, b_cr, s_al, s_cr, infl)) snap = false;
  }
  c.walking = !snap;
  if (!snap && (t.cone_toggled || c.escape != c.held_prev)) t.step = true;   // M-14
  c.held_prev = c.escape;
  c.live = snap ? pet : fmStationSlewStep(c.live, pet, kSlewRate, dt);
  c.live = fmClampStationDeg(c.live, phi);
  const float r = snap ? d_follow : fmStationRadiusM(c.live, d_follow, rf, near_diag, pass, phi);
  float sa, sc;
  fmStationAlongCrossM(c.live, r, &sa, &sc);
  const bool outward = fmAimOutwardNeeded(c.live, b_cr, sc, pass);
  t.st_al = sa; t.st_cr = sc; t.outward = outward;

  int kind = 0;
  if (outward) {
    const float lat = fmOutwardAimLateralM(c.live, b_cr, pass, d_follow);
    t.aim_al = t.sh_al = t.b_al;
    t.aim_cr = t.sh_cr = lat;
    kind = 1;
    t.profile = h1 ? kPrRetreat : kPrOutward;
    c.look_prev = 0.0f;                                                 // M-14: no lookahead on this aim
  } else {
    float look = 0.0f;
    const float a_live = fabsf(c.live);
    if (a_live > 90.0f && lim > 90.0f) {
      float f = (a_live - 90.0f) / (lim - 90.0f);
      if (f > 1.0f) f = 1.0f;
      look = 1.0f * d_follow * f;
    }
    if (look > 0.0f && !snap) look = fmShieldClipLookaheadM(t.sh, infl, t.b_al, b_cr, sa, sc, look);
    t.look  = look;                                                     // M-14: an output step is an aim step
    t.dlook = fabsf(look - c.look_prev);
    if (!snap && t.dlook > kLookStepM) t.step = true;
    c.look_prev = look;
    t.aim_al = t.sh_al = sa + look;
    t.aim_cr = t.sh_cr = sc;
    if (!snap) {
      const float dx = t.aim_al - t.b_al, dy = t.aim_cr - t.b_cr;
      if (sqrtf(dx * dx + dy * dy) < kThetaMinM) {                     // G-5 twin: the carrot moves,
        kind = 3;                                                       // the shield keeps the station
        t.aim_al = t.b_al + kThetaMinM;
        t.aim_cr = b_cr;
      }
    }
    if (snap)                            t.profile = 2;                 // a rear label - stable while snapped
    else if (m_eff >= 1 && m_eff <= 3)   t.profile = kPrTransit;
    else if (fabsf(pet) > 90.0f)         t.profile = c.transit ? kPrTransit : kPrFront;
    else                                 t.profile = kPrTransit;
    if (h1) t.profile = kPrRetreat;
  }

  bool esc = false, settled = false;                                   // finishAim
  if (!snap) {
    const FmShieldDecision d = fmShieldDecide(t.sh, c.escape, c.shside, band, t.hold, c.live, t.b_al, b_cr,
                                              t.sh_al, t.sh_cr, pass, d_follow);
    if (d.escape) {
      esc = true; kind = 2;
      if (c.escape && c.shside != 0 && d.side != c.shside) t.step = true;           // M-13
      c.shside = d.side;
      if (d.reseed) {
        const float rs_lim = (m_eff >= 1 && m_eff <= 3) ? 90.0f : lim;            // L-17
        c.live = fmRetreatSeedDeg(d.side, meas, rs_lim); t.step = true; t.sh_reseed = true;
      }
      settled = (m_decl == 4 || m_decl == 5) && !c.aborted && !c.transit;         // the 3 s latch
      if (fmShieldSettledEscapeExpired(settled, c.now_ms, &c.settled_since, 3000u)) {
        c.aborted = true; c.rside = d.side; t.settled_latched = true;
      }
      if ((m_decl == 4 || m_decl == 5) && !c.aborted && c.transit) {
        c.aborted = true; c.rside = d.side; c.transit = false; t.latched = true;
      }
      t.aim_al = t.b_al;
      t.aim_cr = d.lateral_m;
      t.profile = kPrOutward;
    }
  }
  c.escape = esc;
  if (!esc) c.shside = 0;
  if (!settled) c.settled_since = 0;
  if (kind != c.kind_prev) t.step = true;
  c.kind_prev = kind;
  t.snapped = snap; t.escape = esc; t.kind = kind;
  if (c.ticks > 0 && !t.step && t.dlook > g_dlook_max_quiet) g_dlook_max_quiet = t.dlook;
  c.ticks++;
  return t;
}

// V2.5-Evo - 2026-10-06 - the ACTIVE edge as the RX runs it (station part only): the seed (or pending, with no
// course), every shield latch cleared, and - audit M-17 - walking when the seed moved the live angle off 0.
static void engageEdge(Ctl &c, uint8_t mode, bool course_valid, float meas)
{
  if (course_valid) { c.live = fmEngageSeedDeg(mode, true, meas); c.pending = false; }
  else              { c.live = 0.0f; c.pending = true; }
  c.escape = false; c.shside = 0; c.walking = (c.live != 0.0f);
  c.cone_on = false; c.held_prev = false; c.look_prev = 0.0f; c.settled_since = 0;
  c.transit = false; c.tcourse = -1.0f; c.aborted = false; c.rside = 0; c.diag = false;
}

// V2.5-Evo - 2026-10-06 - one NO-COURSE (degraded) tick of computeFmTarget(), station part only: the M-19 abort
// rule, and the shield state that branch clears. The live angle is frozen there.
static void degradedTick(Ctl &c, uint8_t m_decl)
{
  if (fmNoCourseAbortLatches(m_decl, c.aborted, c.live, c.tcourse >= 0.0f)) c.aborted = true;
  c.transit = false; c.escape = false; c.shside = 0; c.kind_prev = 4;
  c.cone_on = false; c.held_prev = false; c.look_prev = 0.0f; c.settled_since = 0;
}

// ======================================================================================
// THE RX'S NUMBERS (RTMState.ino) AND THE OWNER'S RX1 CONFIG
// ======================================================================================
static const float    kMinDistO   = 5.0f, kBandO = 4.0f, kDFollowO = 9.0f, kNearO = 60.0f, kDEngageO = 12.0f;
static const float    kFoilerLowO = 8.0f, kSpeedHystO = 2.0f, kAlignDegO = 45.0f, kAlignCapO = 50.0f, kTauO = 2.0f;
static const float    kRawMaxO    = 4.0f, kEngageFloorO = 9.5f;
static const uint32_t kEngageRampMsO = 1500, kJudgeGraceMsO = 6500;
static const float    kGainO = 0.5f, kGapMaxO = 15.0f, kMarginO = 5.0f, kFloorO = 3.0f, kAlongGainO = 0.5f, kCourseMinO = 5.0f;
static const float    kDivFactorO = 2.0f, kDivFarO = 3.0f, kDivEpsO = 2.0f;
static const uint32_t kDivMsO = 3000;
static const float    kPivSuspendO = 45.0f, kPivHystO = 10.0f, kPivWorseO = 15.0f, kPivEpsO = 2.0f;
static const uint32_t kPivStallMsO = 2000, kPivMaxMsO = 15000;
static const float    kStickyO = 60.0f, kCogMinKmhO = 3.0f;
// unit 1 (RTMState.ino kFmCv*, kFmCvQualCfg, kFmCourseValidSpeedKmh)
static const float    kCvLatencyS = 1.0f, kCvCoastS = 1.6f, kCvCosMin = 0.25f;
static const uint32_t kCvReriseMs = 1500, kCvGapMs = 250, kCvParkMs = 6500;
// Revision 2.2 §B, the two stop legs: S-a (as built in Rev 2.1: 30 % down within 1.5 s, one estimate) and S-b (at or under
// 60 % of the 2 s maximum, on 2 consecutive estimates). Both: closing >= 1 m/s, the 2 s maximum >= 2.5 m/s, exit at 80 % on
// 2 estimates or closing under 0.5 m/s for 1 s. RTMState.ino kFmCvQualCfg holds the shipped one.
static const FmCvQualCfg kQualSa = { 0.70f, 1500, 1, 1.0f, 2.5f, 2000, 0.80f, 2, 0.5f, 1000 };
static const FmCvQualCfg kQualSb = { 0.60f, 2000, 2, 1.0f, 2.5f, 2000, 0.80f, 2, 0.5f, 1000 };
static const float    kCourseValidKmh = 5.0f;
static float          g_vmax = 40.0f;   // boogie_vmax_in_followme_kmh
// ---- the simulation ----
static const double kDt = 0.1, kMph = 0.44704, kPi = 3.14159265358979323846;
static const double kYawGain = 1.5, kYawMax = 45.0, kTauUp = 2.4;
static const double kTauHard = 1.52, kTauMed = 1.33, kTauOld = 2.4;   // the plant's coast-down: hard / reported / reported
static const float  kFixNoise = 0.646f, kBiasM = 2.0f, kSpdNoise = 0.483f;
static const int    kLatencyTicks = 10;      // the remote's fix is 1 s late
static const double kFloorFt = 16.4;         // the gates' floor: min_dist
static const double kMPerDeg = 111319.49;    // the sim's metres <-> degrees at the equator (FollowMeConverge.h's constant)
static long g_checks = 0;
#define CHECK(x) do { assert(x); g_checks++; } while (0)

struct Rng {
  uint32_t s;
  float u() { s = s * 1103515245u + 12345u; return (float)((s >> 8) & 0xFFFFu) / 65535.0f; }
  float sym(float a) { return (u() * 2.0f - 1.0f) * a; }
  float gauss() { float a = 0.0f; for (int i = 0; i < 12; i++) a += u(); return a - 6.0f; }   // SD 1 (Irwin-Hall)
};
struct V2 { double x, y; };   // x east, y north, metres
static double wrapd(double a) { while (a > 180.0) a -= 360.0; while (a <= -180.0) a += 360.0; return a; }
static double brgDeg(V2 a, V2 b) { double d = atan2(b.x - a.x, b.y - a.y) * 180.0 / kPi; if (d < 0.0) d += 360.0; return d; }
static double distM(V2 a, V2 b) { return hypot(b.x - a.x, b.y - a.y); }
static V2 proj(V2 p, double brg, double d) { const double r = brg * kPi / 180.0; return { p.x + d * sin(r), p.y + d * cos(r) }; }
// A point relative to an origin, in a course's frame: along (ahead +), cross (right +).
static void frameOf(V2 origin, double course, V2 p, float *al, float *cr)
{
  const double c = course * kPi / 180.0, dx = p.x - origin.x, dy = p.y - origin.y;
  *al = (float)(dx * sin(c) + dy * cos(c));
  *cr = (float)(dx * cos(c) - dy * sin(c));
}
static V2 fromFrame(V2 origin, double course, double al, double cr)
{
  const double c = course * kPi / 180.0;
  return { origin.x + al * sin(c) + cr * cos(c), origin.y + al * cos(c) - cr * sin(c) };
}

struct Model { const char *name; double v_top_mph; double n; };
static const Model kM19 = { "19 Oct fit", 15.5, 2.0 };
static const Model kM30 = { "30 Oct fit", 16.2, 1.36 };
static double steadyMs(const Model &m, double cmd) { if (cmd <= 0.0) return 0.0; return m.v_top_mph * kMph * pow(cmd / 255.0, 1.0 / m.n); }

// One arm of the comparison: the flown code, unit 1, or a variant of it.
struct Arm {
  const char *name;
  bool  cv;          // the ceiling is in
  bool  qual;        // the stop qualifier is in (false: the ceiling acts on any closure - the "unqualified" ceiling)
  float coast_s;     // the controller's tau_c
  int   red;         // 0 none; 1 the range and motion taken from the FILTERED rider; 2 no latency margin; 3 no park
  FmCvQualCfg q;
};
static const Arm kFlown = { "flown",                  false, false, kCvCoastS, 0, kQualSa };
static const Arm kSa    = { "S-a",                    true,  true,  kCvCoastS, 0, kQualSa };
static const Arm kSb    = { "S-b",                    true,  true,  kCvCoastS, 0, kQualSb };
static const Arm kNoQ   = { "unqualified ceiling",    true,  false, kCvCoastS, 0, kQualSa };

enum { ST_ACTIVE = 1, ST_HOLD = 2, ST_ENDED = 3 };   // ST_ENDED: a steering end - ARMED with the sticky cap, steering straight

struct Sim {
  const Model *m; uint8_t mode; Arm arm; Rng rng; double tau_down;
  double t; uint32_t now;
  V2 rider; double rc, rv;                 // rider truth
  V2 hist[kLatencyTicks + 1]; int hist_head;
  double bias_phase;
  V2 fix; long fix_idx; uint32_t sol_seq;
  V2 filt, prev_filt; double prev_filt_t; float vf, course;
  V2 raw_base; double raw_base_t; float raw;
  V2 buggy; double bh, bv;                 // buggy truth
  int state; uint32_t engage_ms; Ctl c;
  FmDivergeDwell dv;
  float sticky;
  uint32_t piv_since, piv_stall; float piv_best; bool piv_failed; float last_err; uint8_t last_cap;
  FmCvTrack track; FmCvRerise rer; FmCvQual q; uint32_t cv_last_bind;
  bool bound, coasted, qualified;          // this tick: the ceiling lowered cap 3 / by a coast; the qualifier stood
  long binds, active_ticks;
  int steer_ends, r2_verdicts, hold_entries, ends_parked; bool in_hold;
};

static void simInit(Sim &s, const Model *m, uint8_t mode, const Arm &arm, double tau_down, uint32_t seed, double mph)
{
  s = Sim();
  s.m = m; s.mode = mode; s.arm = arm; s.rng.s = seed; s.tau_down = tau_down;
  s.t = 0.0; s.now = 100000;
  s.rider = { 0.0, 0.0 }; s.rc = 0.0; s.rv = mph * kMph;
  for (int i = 0; i <= kLatencyTicks; i++) s.hist[i] = proj(s.rider, 180.0, s.rv * kDt * (kLatencyTicks - i));
  s.hist_head = kLatencyTicks;
  s.bias_phase = (seed % 360) * kPi / 180.0;
  s.fix = s.hist[0]; s.fix_idx = 0; s.sol_seq = 1;
  s.filt = proj(s.fix, 180.0, s.rv * (kTauO + kDt)); s.prev_filt = proj(s.filt, 180.0, s.rv * 0.4); s.prev_filt_t = -0.4;
  s.vf = (float)(s.rv * 3.6); s.course = (s.vf >= kCourseMinO) ? 0.0f : -1.0f;
  s.raw_base = proj(s.fix, 180.0, s.rv); s.raw_base_t = -1.0; s.raw = s.vf;
  s.state = ST_ACTIVE; s.engage_ms = s.now - kEngageRampMsO;   // engaged a moment ago: the ramp done, the grace still on
  s.dv.since_ms = 0; s.dv.start_m = -1.0f;
  s.last_cap = 255; s.piv_best = 180.0f;
  fmCvTrackReset(&s.track);
  fmCvQualReset(&s.q);
}

// The front-station geometry as ctlTick() resolves it.
static void frontGeomO(float *phi, float *rf, float *pass)
{
  float p = kPassLateral; if (p < kMinDistO) p = kMinDistO;
  const float latmin = fmFrontSideFloorM(kLateralMin, p, kMinDistO, kSideBand);
  const float extra = fmFrontAheadExtraM(0, kExtraDefault, kExtraMin, kExtraMax);
  float ahead;
  fmFrontStationGeom(kDFollowO, extra, latmin, kFrontMin, kFrontMax, &ahead, phi, rf, nullptr);
  *pass = p;
}

// Put the buggy on its station - rear: the station line, back_m from the rider; front: the settled front station - moving
// with the rider, with the station state a settled engagement would have.
static void placeOnStation(Sim &s, double back_m)
{
  float phi, rf, pass; frontGeomO(&phi, &rf, &pass);
  if (s.mode == 4 || s.mode == 5) {
    s.c.live = fmStationPresetDeg(s.mode, kNearO, phi); s.c.walking = true;
    const float r = fmStationRadiusM(s.c.live, kDFollowO, rf, kNearO, pass, phi);
    float al, cr; fmStationAlongCrossM(s.c.live, r, &al, &cr);
    s.buggy = fromFrame(s.rider, s.rc, al, cr);
  } else {
    const float psi = fmStationPresetDeg(s.mode, kNearO, phi);
    float al, cr; fmStationAlongCrossM(psi, kDFollowO, &al, &cr);
    const double back = -sqrt(fmax(back_m * back_m - (double)cr * cr, 1.0));
    s.buggy = fromFrame(s.rider, s.rc, back, cr);
    s.c.live = psi; s.c.diag = true;
  }
  s.bh = s.rc; s.bv = s.rv;
}

// Cap 3 as fmComputeThrottleCap() computes it: the flown law and, with arm.cv, the fast-closure ceiling (unit 1) with -
// with arm.qual - its abnormal-closure qualifier, in the RX's order.
static uint16_t cap3Rx(Sim &s, float dist, bool frame_valid, const Tick &t, float b_al_f, float b_cr, float buggy_kmh,
                       bool *bind_out)
{
  bool theta_ok = false; float theta = 0.0f;
  if (s.course >= 0.0f && distM(s.buggy, s.filt) > kThetaMinM) {
    theta = (float)fabs(wrapd(s.course - brgDeg(s.buggy, s.filt))); theta_ok = true;
  }
  float gap = (dist - kDFollowO) * kGainO; if (gap < 0.0f) gap = 0.0f; if (gap > kGapMaxO) gap = kGapMaxO;
  const bool outward = frame_valid && t.outward;
  const bool front   = frame_valid && fabsf(s.c.live) > 90.0f;
  float gov;
  if (outward)    gov = s.vf;
  else if (front) gov = s.vf + fmStationAlongGovKmh(t.st_al, b_al_f, kAlongGainO, kGapMaxO);
  else            gov = s.vf + kMarginO + (theta_ok ? gap : 0.0f);
  bool bind = false;
  FmCvVerdict v = { false, false, -1.0f, -1.0f };
  s.qualified = false;
  if (s.arm.cv) {
    float r = 0.0f, cdot = 0.0f;
    const bool closing_ok = fmCvClosing(&s.track, s.now, 3000u, &r, &cdot);
    float mps = 0.0f, crs = 0.0f; bool crs_ok = false;
    const bool mv_ok = closing_ok && fmCvRiderMotion(&s.track, s.now, 3000u, kCourseValidKmh / 3.6f, &mps, &crs, &crs_ok);
    bool qualified = closing_ok;
    if (s.arm.qual) qualified = fmCvQualStep(&s.q, s.now, closing_ok, fmCvNewestMs(&s.track), cdot, mv_ok, mps, &s.arm.q);
    s.qualified = qualified;
    const bool cog_ok = (s.bv * 3.6 >= kCogMinKmhO);
    float cosv = 0.0f;
    const float vlos = fmCvLosMps(buggy_kmh, cog_ok, (float)s.bh, (float)brgDeg(s.buggy, s.fix), &cosv);
    v = fmCvDecide(closing_ok && qualified, r, cdot, vlos, cog_ok, cosv, (s.arm.red == 2) ? 0.0f : kCvLatencyS,
                   kMinDistO + kBandO, kBandO / s.arm.coast_s, s.arm.coast_s, kCvCosMin);
    if (v.gov_max_kmh >= 0.0f && gov > v.gov_max_kmh) { gov = v.gov_max_kmh; bind = true; }
  }
  if (gov < 0.0f) gov = 0.0f;
  if (gov > g_vmax) gov = g_vmax;
  float phi, rf, pass; frontGeomO(&phi, &rf, &pass);
  const bool bypass = frame_valid && !t.escape && !outward && fmFadeBypass(true, s.c.live, b_al_f, b_cr, pass);
  if (theta_ok && theta > 90.0f && !bypass) { float f = (180.0f - theta) / 90.0f; if (f < 0.0f) f = 0.0f; gov *= f; }
  uint16_t c3;
  if (gov <= kFloorO) c3 = 0;
  else { float sf = buggy_kmh / gov; if (sf > 1.0f) sf = 1.0f; if (sf < 0.0f) sf = 0.0f; c3 = (uint16_t)((1.0f - sf) * 255.0f); }
  s.coasted = false;
  if (s.arm.cv) {
    if (v.coast) { c3 = 0; bind = true; s.coasted = true; }
    c3 = fmCvCap3Step(&s.rer, s.now, c3, bind, kCvReriseMs, kCvGapMs);
  }
  *bind_out = bind;
  return c3;
}

// One 100 ms FM tick in runFmLoopBody()'s order, then the buggy moves. The caller sets the rider's rc / rv first.
static void simTick(Sim &s)
{
  s.t += kDt; s.now += 100;
  s.bound = false; s.coasted = false;
  s.rider = proj(s.rider, s.rc, s.rv * kDt);
  s.hist_head = (s.hist_head + 1) % (kLatencyTicks + 1);
  s.hist[s.hist_head] = s.rider;
  // ---- the rider's fix: 3 Hz, 1 s late, slow bias + white noise ----
  const long idx = (long)floor(s.t * 3.0 + 1e-9);
  bool new_fix = false;
  if (idx != s.fix_idx) {
    s.fix_idx = idx;
    const V2 late = s.hist[(s.hist_head + 1) % (kLatencyTicks + 1)];   // the oldest entry: 1 s ago
    const double ph = s.bias_phase + 2.0 * kPi * s.t / 90.0;
    s.fix = { late.x + kBiasM * cos(ph) + s.rng.sym(kFixNoise), late.y + kBiasM * sin(ph) + s.rng.sym(kFixNoise) };
    s.sol_seq++; new_fix = true;
  }
  // ---- updateFmRiderTracking() / updateFmRawRiderSpeed() ----
  const double a = kDt / (kTauO + kDt);
  s.filt.x += a * (s.fix.x - s.filt.x); s.filt.y += a * (s.fix.y - s.filt.y);
  if (s.t - s.prev_filt_t >= 0.4 - 1e-9) {
    s.vf = (float)(distM(s.prev_filt, s.filt) / (s.t - s.prev_filt_t) * 3.6);
    s.course = (s.vf >= kCourseMinO) ? (float)brgDeg(s.prev_filt, s.filt) : -1.0f;
    s.prev_filt = s.filt; s.prev_filt_t = s.t;
  }
  if (new_fix && s.t - s.raw_base_t >= 1.0 - 1e-9) {
    s.raw = (float)(distM(s.raw_base, s.fix) / (s.t - s.raw_base_t) * 3.6);
    s.raw_base = s.fix; s.raw_base_t = s.t;
  }
  const float buggy_kmh = (float)(s.bv * 3.6) + s.rng.sym(kSpdNoise);
  const float dist = (float)distM(s.fix, s.buggy);
  // ---- unit 1: one range sample per new rider solution (the distance and the fix itself, as degrees) ----
  {
    const V2 rp = (s.arm.red == 1) ? s.filt : s.fix;
    fmCvTrackSample(&s.track, s.now, s.sol_seq, (float)distM(rp, s.buggy), rp.y / kMPerDeg, rp.x / kMPerDeg);
  }
  // ---- conditions 8 / 9, the return candidate ----
  const bool active = (s.state == ST_ACTIVE);
  const bool speed_ok = active ? (s.vf >= kFoilerLowO) : (s.vf >= kFoilerLowO + kSpeedHystO);
  const bool dist_ok  = active ? (dist >= kMinDistO)
                               : followMeMayEngage(dist, kMinDistO, kBandO, kEngageFloorO, kDEngageO, false, false);
  const bool cand     = s.raw >= 0.0f && s.raw < kRawMaxO;
  // ---- the pivot suspension (on the last steering tick's heading error), then R2's dwell ----
  const bool grace = s.engage_ms != 0 && (uint32_t)(s.now - s.engage_ms) < kJudgeGraceMsO;
  bool pivoting = false;
  {
    const float exit_deg = (kAlignDegO > kPivSuspendO) ? kAlignDegO : kPivSuspendO, enter_deg = exit_deg + kPivHystO;
    const bool band = (s.piv_since != 0) ? (s.last_err > exit_deg) : (s.last_err > enter_deg);
    if (active && band) {
      if (s.piv_since == 0) { s.piv_since = s.now; s.piv_best = s.last_err; s.piv_stall = s.now; s.piv_failed = false; }
      if (grace) { s.piv_stall = s.now; if (s.last_err < s.piv_best) s.piv_best = s.last_err; }
      else if (s.last_err < s.piv_best - kPivEpsO) { s.piv_best = s.last_err; s.piv_stall = s.now; }
      else if (!s.piv_failed && (s.last_err > s.piv_best + kPivWorseO || (uint32_t)(s.now - s.piv_stall) >= kPivStallMsO)) s.piv_failed = true;
      pivoting = !s.piv_failed && (uint32_t)(s.now - s.piv_since) < kPivMaxMsO;
    } else { s.piv_since = 0; s.piv_best = 180.0f; s.piv_stall = 0; s.piv_failed = false; }
  }
  bool diverge = false;
  {
    float phi, rf, pass; frontGeomO(&phi, &rf, &pass);
    float ceil_m = kDivFactorO * kDEngageO;
    const float r_st = (fabsf(s.c.live) > 90.0f) ? fmStationRadiusM(s.c.live, kDFollowO, rf, kNearO, pass, phi) : kDFollowO;
    if (r_st + kBandO > ceil_m) ceil_m = r_st + kBandO;
    const bool park = s.arm.cv && s.arm.red != 3 && fmCvParks(s.cv_last_bind, s.now, kCvParkMs);
    const bool judged = active && !grace && !pivoting && !park;
    diverge = fmDivergeDwellStep(&s.dv, s.now, judged, dist, ceil_m, kDivFarO * kDEngageO, kDivMsO, kDivEpsO, kDivEpsO) != FMDV_NONE;
    if (diverge) s.r2_verdicts++;
  }
  // ---- can_be_active and the branches ----
  double cmd = 0.0, yaw = 0.0;
  const bool eligible = (s.state != ST_ENDED) && speed_ok && dist_ok && !diverge && !cand;
  if (eligible) {
    if (s.state != ST_ACTIVE) {                                         // the ACTIVE edge
      s.engage_ms = s.now; s.state = ST_ACTIVE; s.dv.since_ms = 0; s.dv.start_m = -1.0f;
      const bool cvalid = s.course >= 0.0f;
      engageEdge(s.c, s.mode, cvalid, cvalid ? fmMeasuredStationDeg(s.course, (float)brgDeg(s.filt, s.buggy)) : 0.0f);
    }
    s.in_hold = false;
    V2 aim; Tick tk; bool frame_valid = false;
    float b_al_f = 0.0f, b_cr = 0.0f;
    if (s.course >= 0.0f) {
      double lag = s.vf / 3.6 * kTauO; if (lag > 2.0 * kDFollowO) lag = 2.0 * kDFollowO;
      frameOf(s.filt, s.course, s.buggy, &b_al_f, &b_cr);
      tk = ctlTick(s.c, s.mode, kNearO, kDFollowO, 0, kMinDistO, s.vf, s.course, b_al_f, b_cr, (float)lag, (float)kDt, kSideBand);
      aim = fromFrame(proj(s.filt, s.course, lag), s.course, tk.aim_al, tk.aim_cr);
      frame_valid = true;
    } else {
      degradedTick(s.c, s.mode);
      aim = proj(s.filt, brgDeg(s.filt, s.buggy), kDFollowO);
    }
    const double err = wrapd(brgDeg(s.buggy, aim) - s.bh);
    uint16_t cap = 255;
    if (dist < kMinDistO + kBandO) { float f = (dist - kMinDistO) / kBandO; if (f < 0.0f) f = 0.0f; if (f > 1.0f) f = 1.0f;
                                     const uint16_t c2 = (uint16_t)(f * 255.0f); if (c2 < cap) cap = c2; }
    bool bind = false;
    const uint16_t c3 = cap3Rx(s, dist, frame_valid, tk, b_al_f, b_cr, buggy_kmh, &bind);
    if (c3 < cap) cap = c3;
    if (fabs(err) > kAlignDegO && kAlignCapO < cap) cap = (uint16_t)kAlignCapO;
    const uint32_t el = (uint32_t)(s.now - s.engage_ms);
    if (el < kEngageRampMsO) { const uint16_t c5 = (uint16_t)((float)el / (float)kEngageRampMsO * 255.0f); if (c5 < cap) cap = c5; }
    if (bind) { s.cv_last_bind = s.now; s.binds++; s.bound = true; }
    s.active_ticks++;
    cmd = cap; s.last_cap = (uint8_t)cap;
    yaw = kYawGain * err; if (yaw > kYawMax) yaw = kYawMax; if (yaw < -kYawMax) yaw = -kYawMax;
    s.last_err = (float)fabs(err);
  } else if (diverge && s.state == ST_ACTIVE) {
    // G-UT4: an end while the ceiling binds or inside its park would be the unit's doing (zero unless the park is removed)
    if (s.arm.cv && s.cv_last_bind != 0 && (uint32_t)(s.now - s.cv_last_bind) < kCvParkMs) s.ends_parked++;
    s.steer_ends++; s.state = ST_ENDED; s.sticky = (s.last_cap < kStickyO) ? s.last_cap : kStickyO;
  } else if (s.state == ST_ENDED) {
    s.sticky += 255.0f * (float)kDt / 1.5f; if (s.sticky > kStickyO) s.sticky = kStickyO;   // the sticky cap climbs to 60
    cmd = s.sticky;
  } else {
    if (!s.in_hold) { s.hold_entries++; s.in_hold = true; }
    s.state = ST_HOLD; cmd = 0.0;
  }
  // ---- the buggy ----
  const double v_ss = steadyMs(*s.m, cmd);
  s.bv += (v_ss - s.bv) * (kDt / ((v_ss > s.bv) ? kTauUp : s.tau_down));
  s.bh += yaw * kDt;
  s.buggy = proj(s.buggy, s.bh, s.bv * kDt);
}

// ======================================================================================
// THE SCENARIOS
// ======================================================================================
// The outcome of one U-turn / pair run (one phase).
struct Out {
  double closest_ft;      // truth, rider to buggy
  double v30_mph;         // G-UT1: the buggy's speed toward the rider when the range first crosses 30 ft (-1 = never)
  double ref_ft;          // G-UT2: the rider's path to a reference buggy coasting at cap 0 from his turn start
  int    steer_ends, r2, ends_parked;   // G-UT4 (from the turn on; steer_ends -1 = R2 had ended it before the turn)
  bool   pre_bound;       // the ceiling had bound before the turn (the arms are then no longer identical at the turn)
  double lift_s;          // G-UT5: how long the ceiling still bound after the range opened (s; -99 = n/a)
  double t_event;         // the closure event: the first tick after his turn starts with the TRUE range closing >= 3 m/s
                          // (over 0.5 s), from the turn start (s; -1 = none)
  bool   pre_event_bound; // the ceiling bound between the turn start and the event (the arms may then differ at it)
  double t_bind, t_qual;  // the first bind / qualification at or after the event, from it (s; -1 = never)
  long   binds, ticks;
};

// A U-turn - or, with run_s >= 0, a wave-linking pair: turn toward, run run_s, turn back the other way. The rider at mph,
// 180 deg in turn_s, dir -1 left / +1 right; the buggy back_ft behind on its station line (F4/F5: settled on the station).
static Out runTurn(const Model *m, uint8_t mode, const Arm &arm, double tau_down, double mph, double turn_s, int dir,
                   double back_ft, int phase, double run_s)
{
  Sim s; simInit(s, m, mode, arm, tau_down, 7001u + 131u * (uint32_t)phase + (uint32_t)(mph * 10) + mode * 17u, mph);
  placeOnStation(s, back_ft * 0.3048);
  const int warm = 10 + 4 * phase;                                    // 1.0 - 4.6 s straight first: the 10 phases
  for (int i = 0; i < warm; i++) simTick(s);
  const int ends0 = s.steer_ends, r20 = s.r2_verdicts, park0 = s.ends_parked;
  const bool ended_before = (s.state == ST_ENDED);
  Out o = { 1e9, -1.0, -1.0, 0, 0, 0, s.binds > 0, -99.0, -1.0, false, -1.0, -1.0, 0, 0 };
  V2 ref = s.buggy; double ref_v = s.bv; const double ref_h = s.bh;   // the reference buggy: coasting from here
  const double t0 = s.t;
  double ref_min = 1e9, prev_d = 1e9, open_t = -1.0, t_min = 0.0, last_bind_t = -1.0;
  std::vector<double> bind_t;
  bool crossed30 = false;
  double dq[5]; for (int k = 0; k < 5; k++) dq[k] = distM(s.rider, s.buggy);
  const int turn_ticks = (int)(turn_s * 10.0 + 0.5);
  const int run_ticks  = (run_s >= 0.0) ? (int)(run_s * 10.0 + 0.5) : 0;
  const int total = turn_ticks + ((run_s >= 0.0) ? run_ticks + turn_ticks : 0) + 200;
  for (int i = 0; i < total; i++) {
    if (i < turn_ticks) s.rc += dir * 180.0 / turn_ticks;
    else if (run_s >= 0.0 && i >= turn_ticks + run_ticks && i < 2 * turn_ticks + run_ticks) s.rc -= dir * 180.0 / turn_ticks;
    simTick(s);
    {
      const double dn = distM(s.rider, s.buggy), closing = (dq[i % 5] - dn) / 0.5;
      dq[i % 5] = dn;
      if (o.t_event < 0.0 && i >= 5 && closing >= 3.0) o.t_event = s.t - t0;
    }
    if (s.bound) {
      last_bind_t = s.t; bind_t.push_back(s.t);
      if (o.t_event < 0.0) o.pre_event_bound = true;
      else if (o.t_bind < 0.0) o.t_bind = s.t - t0 - o.t_event;
    }
    if (o.t_event >= 0.0 && s.qualified && arm.qual && o.t_qual < 0.0) o.t_qual = s.t - t0 - o.t_event;
    ref_v *= exp(-kDt / tau_down);
    ref = proj(ref, ref_h, ref_v * kDt);
    const double rd = distM(s.rider, ref); if (rd < ref_min) ref_min = rd;
    const double d = distM(s.rider, s.buggy);
    if (d / 0.3048 < o.closest_ft) { o.closest_ft = d / 0.3048; t_min = s.t; }
    if (!crossed30 && d < kDFollowO) {
      crossed30 = true;
      const double cosv = cos((s.bh - brgDeg(s.buggy, s.rider)) * kPi / 180.0);
      o.v30_mph = (cosv > 0.0) ? s.bv * cosv / kMph : 0.0;
    }
    if (run_s >= 0.0 && i >= 2 * turn_ticks + run_ticks && open_t < 0.0 && d > prev_d + 0.05) open_t = s.t;
    prev_d = d;
  }
  o.ref_ft = ref_min / 0.3048;
  o.steer_ends = ended_before ? -1 : (s.steer_ends - ends0);
  o.r2 = s.r2_verdicts - r20;
  o.ends_parked = s.ends_parked - park0;
  // G-UT5: pairs - the range opening after the second turn; a single U-turn - after the closest approach. The ceiling's
  // last bind within 10 s of that moment, measured from it.
  const double t_open = (run_s >= 0.0) ? open_t : t_min;
  if (t_open >= 0.0) {
    double last = -1.0;
    for (double bt : bind_t) if (bt >= t_open && bt <= t_open + 10.0) last = bt;
    o.lift_s = (last >= 0.0) ? (last - t_open) : 0.0;
  }
  (void)last_bind_t;
  o.binds = s.binds; o.ticks = s.active_ticks;
  return o;
}

// The rider stops dead (-9 mph/s) after 150 s of steady following from 30 m back (F4/F5: from the settled station);
// 10 phases (0.7 s apart). Per phase: the closest approach (ft) and the first bind / qualification from the stop's start.
struct StopOut { double closest[10]; double t_bind[10]; double t_qual[10]; bool pre_bound[10]; double worst; };
static StopOut runStop(const Model *m, uint8_t mode, const Arm &arm, double tau_down, double mph)
{
  StopOut so; so.worst = 1e9;
  Sim base; simInit(base, m, mode, arm, tau_down, 555u + (uint32_t)(mph * 10) + mode * 7u, mph);
  if (mode == 4 || mode == 5) placeOnStation(base, 0.0);
  else placeOnStation(base, 30.0);
  base.engage_ms = 0;
  for (int i = 0; i < 1500; i++) simTick(base);
  for (int k = 0; k < 10; k++) {
    Sim s = base;
    for (int i = 0; i < 7 * k; i++) simTick(s);
    so.pre_bound[k] = s.binds > 0;
    so.t_bind[k] = -1.0; so.t_qual[k] = -1.0;
    const double t0 = s.t;
    double cmin = 1e9;
    for (int i = 0; i < 200; i++) {
      s.rv -= 9.0 * kMph * kDt; if (s.rv < 0.0) s.rv = 0.0;
      simTick(s);
      if (s.bound && so.t_bind[k] < 0.0) so.t_bind[k] = s.t - t0;
      if (s.qualified && arm.qual && so.t_qual[k] < 0.0) so.t_qual[k] = s.t - t0;
      const double d = distM(s.rider, s.buggy) / 0.3048; if (d < cmin) cmin = d;
    }
    so.closest[k] = cmin;
    if (cmin < so.worst) so.worst = cmin;
  }
  return so;
}

// The FLY-BY (Revision 2.2 §D: report only; it was the Rev 2.1 "pickup"): the rider rides at mph straight toward a buggy
// that is at rest or creeping toward him at v0_mph, on a line passing abeam_ft to its side, from 150 ft; Follow-Me ACTIVE
// (engaged long ago). He holds his line and speed.
struct PickOut { double closest_ft, max_bind_ft, max_coast_ft; long binds; };
static PickOut runFlyBy(const Model *m, uint8_t mode, const Arm &arm, double tau_down, double mph, double abeam_ft,
                        double v0_mph, int phase)
{
  Sim s; simInit(s, m, mode, arm, tau_down, 3301u + 97u * (uint32_t)phase + (uint32_t)(mph * 10) + mode * 13u, mph);
  s.buggy = { abeam_ft * 0.3048, 150.0 * 0.3048 };
  s.bh = 180.0; s.bv = v0_mph * kMph;
  s.engage_ms = 0;
  engageEdge(s.c, mode, true, fmMeasuredStationDeg(s.course, (float)brgDeg(s.filt, s.buggy)));
  PickOut o = { 1e9, -1.0, -1.0, 0 };
  for (int i = 0; i < 600; i++) {
    simTick(s);
    const double d = distM(s.rider, s.buggy) / 0.3048;
    if (d < o.closest_ft) o.closest_ft = d;
    if (s.bound) { o.binds++; if (d > o.max_bind_ft) o.max_bind_ft = d; }
    if (s.coasted && d > o.max_coast_ft) o.max_coast_ft = d;
    if (s.rider.y > s.buggy.y + 60.0 * 0.3048) break;                // he is 60 ft past it
  }
  return o;
}

// G-UT6(b): 60 s measured after 60 s settling. sc 0 steady, 1 catch-up from 200 ft behind (no settling: a fresh
// engagement, the ramp done and the 6.5 s grace still on, as simInit() leaves it), 2 carving +/-amp deg with an 8 s
// period. Returns bind ticks, ACTIVE ticks and (catch-up) the largest closing speed (mph, truth, over 1 s).
struct SteadyOut { long binds, ticks; double max_close_mph; };
static SteadyOut runSteady(const Model *m, uint8_t mode, const Arm &arm, double tau_down, double mph, int sc, double amp)
{
  Sim s; simInit(s, m, mode, arm, tau_down, 909u + mode + (uint32_t)(mph * 10) + (uint32_t)amp * 3u, mph);
  placeOnStation(s, (sc == 1) ? 200.0 * 0.3048 : 30.0);
  if (sc != 1) s.engage_ms = 0;
  if (sc != 1) for (int i = 0; i < 600; i++) simTick(s);
  const long b0 = s.binds, t0 = s.active_ticks;
  const double ts = s.t;
  double dq[10]; for (int i = 0; i < 10; i++) dq[i] = distM(s.rider, s.buggy);
  double maxc = 0.0;
  for (int i = 0; i < 600; i++) {
    if (sc == 2) s.rc = amp * sin(2.0 * kPi * (s.t - ts) / 8.0);
    simTick(s);
    const double d = distM(s.rider, s.buggy);
    const double c = (dq[i % 10] - d) / 1.0 / kMph;                  // closing over the last 1 s
    if (i >= 10 && c > maxc) maxc = c;
    dq[i % 10] = d;
  }
  return { s.binds - b0, s.active_ticks - t0, maxc };
}


// G-UT6(a), THE PICKUP (Revision 2.2 §D): Follow-Me ACTIVE behind the rider (his station, settled for 20 s plus the
// phase); he turns back (180 deg in turn_s, dir -1 left / +1 right), rides toward the buggy, and slows steadily at
// -2 mph/s to stop about stop_ft from it, as he does to grab the rope (he starts slowing when the true distance is
// stop_ft plus his braking distance). Returns the closest approach, the buggy's distance from him over 2-8 s after he
// stops (per tick, for the no-standoff comparison), and the ceiling's binds.
struct PickupOut { double closest_ft; double after_ft[61]; bool stopped; long binds; double max_bind_ft, max_coast_ft; };
static PickupOut runPickup(const Model *m, uint8_t mode, const Arm &arm, double tau_down, double mph, double turn_s, int dir,
                           double stop_ft, int phase)
{
  Sim s; simInit(s, m, mode, arm, tau_down, 5501u + 61u * (uint32_t)phase + (uint32_t)(mph * 10) + mode * 19u, mph);
  placeOnStation(s, 40.0 * 0.3048);
  for (int i = 0; i < 200 + 4 * phase; i++) simTick(s);
  PickupOut o; o.closest_ft = 1e9; o.stopped = false; o.binds = 0; o.max_bind_ft = -1.0; o.max_coast_ft = -1.0;
  for (int k = 0; k < 61; k++) o.after_ft[k] = -1.0;
  const int turn_ticks = (int)(turn_s * 10.0 + 0.5);
  const double a = 2.0 * kMph;                                       // -2 mph/s
  bool aimed = false, braking = false; double t_stop = -1.0;
  for (int i = 0; i < 900; i++) {
    if (i < turn_ticks) s.rc += dir * 180.0 / turn_ticks;
    else {
      if (!aimed) { s.rc = brgDeg(s.rider, s.buggy); aimed = true; }  // then he holds that line
      const double d = distM(s.rider, s.buggy);
      if (!braking && d <= stop_ft * 0.3048 + s.rv * s.rv / (2.0 * a)) braking = true;
      if (braking) { s.rv -= a * kDt; if (s.rv < 0.0) s.rv = 0.0; }
    }
    simTick(s);
    const double dft = distM(s.rider, s.buggy) / 0.3048;
    if (dft < o.closest_ft) o.closest_ft = dft;
    if (s.bound) { o.binds++; if (dft > o.max_bind_ft) o.max_bind_ft = dft; }
    if (s.coasted && dft > o.max_coast_ft) o.max_coast_ft = dft;
    if (braking && s.rv <= 0.0 && t_stop < 0.0) { t_stop = s.t; o.stopped = true; }
    if (t_stop >= 0.0) {
      const int k = (int)floor((s.t - t_stop - 2.0) * 10.0 + 0.5);
      if (k >= 0 && k <= 60) o.after_ft[k] = dft;
      if (s.t - t_stop > 8.05) break;
    }
  }
  return o;
}

// G-UT5's STOP-AND-GO (Revision 2.2 §C): from steady following at 8 mph, the rider drops to 3 mph at -9 mph/s, holds 1 s
// and goes back to 8 mph at +3 mph/s. Returns how long the ceiling still bound after his true speed was back to >= 80 %
// of 8 mph (s; 0 = not at all), the closest approach and the binds.
struct StopGoOut { double lift_s, closest_ft; long binds; };
static StopGoOut runStopGo(const Model *m, uint8_t mode, const Arm &arm, double tau_down, int phase)
{
  const double v0 = 8.0 * kMph;
  Sim s; simInit(s, m, mode, arm, tau_down, 6601u + 37u * (uint32_t)phase + mode * 23u, 8.0);
  if (mode == 4 || mode == 5) placeOnStation(s, 0.0); else placeOnStation(s, 30.0);
  s.engage_ms = 0;
  for (int i = 0; i < 600 + 7 * phase; i++) simTick(s);
  StopGoOut o = { 0.0, 1e9, 0 };
  int stage = 0; double hold_until = 0.0, t_back = -1.0;
  for (int i = 0; i < 300; i++) {
    if (stage == 0) { s.rv -= 9.0 * kMph * kDt; if (s.rv <= 3.0 * kMph) { s.rv = 3.0 * kMph; stage = 1; hold_until = s.t + 1.0; } }
    else if (stage == 1) { if (s.t >= hold_until) stage = 2; }
    else if (stage == 2) { s.rv += 3.0 * kMph * kDt; if (s.rv >= v0) { s.rv = v0; stage = 3; } }
    simTick(s);
    if (stage >= 2 && t_back < 0.0 && s.rv >= 0.8 * v0) t_back = s.t;
    const double dft = distM(s.rider, s.buggy) / 0.3048; if (dft < o.closest_ft) o.closest_ft = dft;
    if (s.bound) { o.binds++; if (t_back >= 0.0 && s.t - t_back > o.lift_s) o.lift_s = s.t - t_back; }
  }
  return o;
}

// ======================================================================================
// THE GATE RUNNERS
// ======================================================================================
static const Model *kFits[2] = { &kM19, &kM30 };
static const char *modeName(uint8_t m)
{
  static const char *n[6] = { "?", "F1", "F2", "F3", "F4", "F5" };
  return (m <= 5) ? n[m] : "?";
}
static const Arm *kVar[2] = { &kSa, &kSb };   // the two stop legs, run side by side (Revision 2.2 §B)
// The no-harm rule (Revision 2.2 §C): outside the floor a result may move; inside it, it may not get worse.
static double noHarmFloor(double flown_ft) { return (flown_ft - 0.5 < kFloorFt) ? flown_ft - 0.5 : kFloorFt; }

// The HARD verdicts, per variant, for section 9.
struct Hard { const char *gate; bool pass[2]; char detail[2][200]; };
static Hard g_hard[16];
static int  g_nhard = 0;
static Hard &hardRow(const char *gate)
{
  for (int i = 0; i < g_nhard; i++) if (strcmp(g_hard[i].gate, gate) == 0) return g_hard[i];
  Hard &h = g_hard[g_nhard < 15 ? g_nhard++ : 15];
  h.gate = gate; h.pass[0] = h.pass[1] = true; h.detail[0][0] = h.detail[1][0] = 0;
  return h;
}
static void hard(const char *gate, int v, bool pass, const char *detail)
{
  Hard &h = hardRow(gate);
  h.pass[v] = pass;
  snprintf(h.detail[v], sizeof(h.detail[v]), "%s", detail);
}

// One U-turn / pair case: the turn, the speed, the station distance.
struct TurnSpec { double mph, turn_s; int dir; double back_ft, run_s; };
// Per-variant totals over a set.
struct SetAcc {
  int cases = 0, harm = 0, band = 0, manual = 0, own = 0, mixed = 0, lift_over = 0;
  double lo = 1e9, v30 = 0.0, worst_margin = 1e9, worst_band = -1e9, cost = 1e9, lift = 0.0;
  long ends = 0, parked = 0, binds = 0;
  char worst_case[96] = "";
};

// set: 0 harsh, 1 realistic, 2 wave-linking pairs. One line per fit and station (verbose: one per case). The flown arm and
// both variants run on the same seeds; with with_noq the unqualified ceiling runs too (the cost report).
static void runSet(int set, double tau, bool verbose, bool with_noq, SetAcc acc[2], long *ends_flown, double *lo_flown,
                   double *v30_flown)
{
  for (int fi = 0; fi < 2; fi++) for (uint8_t mode = 1; mode <= ((set == 1) ? 5 : 3); mode++) {
    std::vector<TurnSpec> specs;
    if (set == 0) {
      for (int ts = 2; ts <= 4; ts += 2) for (int dir = -1; dir <= 1; dir += 2) specs.push_back({ 10.0, (double)ts, dir, 82.0, -1.0 });
    } else if (set == 1) {
      const double mph[4] = { 8.0, 10.0, 12.0, 14.0 }, back[3] = { 100.0, 150.0, 200.0 };
      for (int si = 0; si < 4; si++) for (int ts = 6; ts <= 10; ts += 2) for (int dir = -1; dir <= 1; dir += 2)
        for (int bi = 0; bi < ((mode >= 4) ? 1 : 3); bi++) specs.push_back({ mph[si], (double)ts, dir, back[bi], -1.0 });
    } else {
      for (int si = 0; si < 2; si++) for (int ts = 4; ts <= 6; ts += 2) for (int run = 5; run <= 15; run += 5)
        for (int dir = -1; dir <= 1; dir += 2) specs.push_back({ si ? 12.0 : 10.0, (double)ts, dir, 82.0, (double)run });
    }
    SetAcc a[2]; long ef_line = 0; double lo_f_line = 1e9;
    for (const TurnSpec &sp : specs) {
      Out f[10], u[2][10], nq[10];
      for (int k = 0; k < 10; k++) {
        f[k] = runTurn(kFits[fi], mode, kFlown, tau, sp.mph, sp.turn_s, sp.dir, sp.back_ft, k, sp.run_s);
        for (int v = 0; v < 2; v++) u[v][k] = runTurn(kFits[fi], mode, *kVar[v], tau, sp.mph, sp.turn_s, sp.dir, sp.back_ft, k, sp.run_s);
        if (with_noq) nq[k] = runTurn(kFits[fi], mode, kNoQ, tau, sp.mph, sp.turn_s, sp.dir, sp.back_ft, k, sp.run_s);
      }
      double cf = 1e9, vf = 0.0, cq = 1e9; long ef = 0; bool ended = false; int n_own = 0;
      for (int k = 0; k < 10; k++) {
        if (f[k].closest_ft < cf) cf = f[k].closest_ft;
        if (f[k].v30_mph > vf) vf = f[k].v30_mph;
        if (f[k].steer_ends > 0) ef += f[k].steer_ends;
        if (f[k].steer_ends != 0) ended = true;
        if (with_noq && nq[k].closest_ft < cq) cq = nq[k].closest_ft;
        if (f[k].ref_ft < kFloorFt) n_own++;
      }
      // "Manual" (Rev 2.2 §C): an F4/F5 case in which R2 ended Follow-Me in some phase - the buggy then runs under the sticky
      // cap on his centred stick. Reported, not judged here; the front-station work (not in this package) owns them.
      const bool manual_case = (mode >= 4) && ended;
      ef_line += ef; if (cf < lo_f_line) lo_f_line = cf;
      *ends_flown += ef;
      if (cf < *lo_flown) *lo_flown = cf;
      if (vf > *v30_flown) *v30_flown = vf;
      double cu[2], vu[2];
      for (int v = 0; v < 2; v++) {
        cu[v] = 1e9; vu[v] = 0.0; long eu = 0, pk = 0, bn = 0; double lift = 0.0; bool ended_u = false;
        for (int k = 0; k < 10; k++) {
          if (u[v][k].closest_ft < cu[v]) cu[v] = u[v][k].closest_ft;
          if (u[v][k].v30_mph > vu[v]) vu[v] = u[v][k].v30_mph;
          if (u[v][k].steer_ends > 0) eu += u[v][k].steer_ends;
          if (u[v][k].steer_ends != 0) ended_u = true;
          pk += u[v][k].ends_parked; bn += u[v][k].binds;
          if (u[v][k].lift_s > lift) lift = u[v][k].lift_s;
        }
        SetAcc &s = a[v];
        s.cases++;
        if (cu[v] < s.lo) s.lo = cu[v];
        if (vu[v] > s.v30) s.v30 = vu[v];
        s.ends += eu; s.parked += pk; s.binds += bn;
        if (lift > s.lift) s.lift = lift;
        if (lift > 3.0) s.lift_over++;
        if (n_own == 10) s.own++; else if (n_own > 0) s.mixed++;
        if (with_noq && cu[v] - cq < s.cost) s.cost = cu[v] - cq;
        if (manual_case || ((mode >= 4) && ended_u)) { s.manual++; continue; }
        const double margin = cu[v] - noHarmFloor(cf);
        if (margin < s.worst_margin) {
          s.worst_margin = margin;
          snprintf(s.worst_case, sizeof(s.worst_case), "%s %s %.0f mph %.0f s %s %.0f ft%s", kFits[fi]->name, modeName(mode), sp.mph,
                   sp.turn_s, sp.dir < 0 ? "left" : "right", sp.back_ft, (sp.run_s >= 0.0) ? " (pair)" : "");
        }
        if (margin < -1e-9) s.harm++;
        const double bm = vu[v] - (vf + 0.3);
        if (bm > s.worst_band) s.worst_band = bm;
        if (bm > 1e-9) s.band++;
      }
      if (verbose) {
        printf("    %s %s %4.1f mph %2.0f s %-5s %3.0f ft%s: closest flown %5.1f / S-a %5.1f / S-b %5.1f",
               kFits[fi]->name, modeName(mode), sp.mph, sp.turn_s, sp.dir < 0 ? "left" : "right", sp.back_ft,
               (sp.run_s >= 0.0) ? ((sp.run_s < 7.0) ? " run  5" : (sp.run_s < 12.0) ? " run 10" : " run 15") : "", cf, cu[0], cu[1]);
        if (with_noq) printf(" (unqualified %5.1f)", cq);
        printf("  band entry %4.1f / %4.1f / %4.1f mph  %s%s\n", vf, vu[0], vu[1],
               (n_own == 10) ? "OWN LINE" : (n_own > 0) ? "own line in some phases" : "clear", manual_case ? "  MANUAL (R2 end)" : "");
      }
    }
    printf("  %s %s: %3d cases  closest flown %5.1f, S-a %5.1f, S-b %5.1f  band entry flown -> S-a / S-b worst %+.1f / %+.1f mph"
           "  no-harm misses %d / %d  band misses %d / %d  R2 ends %ld / %ld / %ld (in park %ld / %ld)  manual %d\n",
           kFits[fi]->name, modeName(mode), a[0].cases, lo_f_line, a[0].lo, a[1].lo, a[0].worst_band + 0.3, a[1].worst_band + 0.3,
           a[0].harm, a[1].harm, a[0].band, a[1].band, ef_line, a[0].ends, a[1].ends, a[0].parked, a[1].parked, a[0].manual);
    for (int v = 0; v < 2; v++) {
      SetAcc &t = acc[v], &s = a[v];
      t.cases += s.cases; t.harm += s.harm; t.band += s.band; t.manual += s.manual; t.own += s.own; t.mixed += s.mixed;
      t.lift_over += s.lift_over; t.ends += s.ends; t.parked += s.parked; t.binds += s.binds;
      if (s.lo < t.lo) t.lo = s.lo;
      if (s.v30 > t.v30) t.v30 = s.v30;
      if (s.worst_margin < t.worst_margin) { t.worst_margin = s.worst_margin; snprintf(t.worst_case, sizeof(t.worst_case), "%s", s.worst_case); }
      if (s.worst_band > t.worst_band) t.worst_band = s.worst_band;
      if (s.cost < t.cost) t.cost = s.cost;
      if (s.lift > t.lift) t.lift = s.lift;
    }
  }
}

// The stop-effect gate (Revision 2.2 §C): on the phases where the unqualified ceiling had not bound before the stop,
// closest >= min(16.4 ft, unqualified - 1.5 ft).
struct StopAcc { int n = 0, fails = 0, skipped = 0; double worst = 1e9; };
static void stopEffect(StopAcc &a, const StopOut &v, const StopOut &nq)
{
  for (int p = 0; p < 10; p++) {
    if (nq.pre_bound[p] || v.pre_bound[p]) { a.skipped++; continue; }
    const double lim = (nq.closest[p] - 1.5 < kFloorFt) ? nq.closest[p] - 1.5 : kFloorFt;
    const double m = v.closest[p] - lim;
    a.n++;
    if (m < a.worst) a.worst = m;
    if (m < -1e-9) a.fails++;
  }
}

int main(int argc, char **argv)
{
  setvbuf(stdout, nullptr, _IONBF, 0);
  const bool quick   = (argc > 1 && strcmp(argv[1], "quick") == 0);
  const bool verbose = (argc > 1 && strcmp(argv[1], "verbose") == 0);
  // ======================================================================================
  // 1. UNIT CHECKS OF THE PURE RULES
  // ======================================================================================
  {
    // A sample at x east / y north metres, as degrees (the sim's frame at the equator).
    auto samp = [](FmCvTrack *t, uint32_t ms, uint32_t seq, float d, double x, double y) {
      fmCvTrackSample(t, ms, seq, d, y / kMPerDeg, x / kMPerDeg);
    };
    FmCvTrack t; fmCvTrackReset(&t);
    float r = 0, cd = 0, sp = 0, crs = 0; bool cok = false;
    samp(&t, 1000, 7, 20.0f, 0.0, 0.0);                                // the first reading only seeds the counter
    CHECK(t.count == 0 && fmCvNewestMs(&t) == 0);
    for (int i = 1; i <= 6; i++) samp(&t, 1000 + 333 * i, 7 + i, 20.0f - 1.0f * i, 0.0, 1.0 * i);   // closing 3 m/s, north 3 m/s
    CHECK(t.count == 6 && fmCvNewestMs(&t) == 2998);
    CHECK(fmCvClosing(&t, 3000, 3000, &r, &cd));
    CHECK(fabsf(r - 14.5f) < 0.01f && fabsf(cd - 3.0f) < 0.05f);
    CHECK(fmCvRiderMotion(&t, 3000, 3000, kCourseValidKmh / 3.6f, &sp, &crs, &cok));
    CHECK(fabsf(sp - 3.0f) < 0.05f && cok && (crs < 0.5f || crs > 359.5f));   // his raw speed and course: 3 m/s north
    samp(&t, 3100, 13, 5.0f, 0.0, 0.0);                                // the same solution again: no sample
    CHECK(t.count == 6);
    CHECK(!fmCvClosing(&t, 3000 + 3001, 3000, &r, &cd));               // the newest fix stale: no ceiling ...
    CHECK(!fmCvRiderMotion(&t, 3000 + 3001, 3000, 1.39f, &sp, &crs, &cok) && !cok);   // ... and no motion
    FmCvTrack sx; fmCvTrackReset(&sx);                                 // fixes 600 ms apart: the baseline is 1.2 s
    samp(&sx, 1000, 1, 30.0f, 0.0, 0.0);
    for (int i = 1; i <= 6; i++) samp(&sx, 1000 + 600 * i, 1 + i, 30.0f - 1.8f * i, 0.0, 1.8 * i);   // closing 3 m/s, north 3 m/s
    CHECK(fmCvClosing(&sx, 4700, 3000, &r, &cd) && fabsf(cd - 3.0f) < 0.05f);
    CHECK(fmCvRiderMotion(&sx, 4700, 3000, 1.39f, &sp, &crs, &cok) && fabsf(sp - 3.0f) < 0.05f);
    FmCvTrack e; fmCvTrackReset(&e);
    samp(&e, 1000, 1, 10.0f, 0.0, 0.0);
    for (int i = 1; i <= 6; i++) samp(&e, 1000 + 333 * i, 1 + i, 10.0f, 1.333 * i, 0.0);   // east at 4 m/s
    CHECK(fmCvRiderMotion(&e, 3000, 3000, 1.39f, &sp, &crs, &cok) && fabsf(sp - 4.0f) < 0.05f && cok && fabsf(crs - 90.0f) < 0.5f);
    FmCvTrack u; fmCvTrackReset(&u);
    samp(&u, 1000, 1, 10.0f, 0.0, 0.0);
    for (int i = 1; i <= 3; i++) samp(&u, 1000 + 333 * i, 1 + i, 10.0f, 0.0, 0.0);
    CHECK(!fmCvClosing(&u, 2000, 3000, &r, &cd));                      // under 4 samples: no ceiling
    FmCvTrack w; fmCvTrackReset(&w);
    samp(&w, 1000, 1, 10.0f, 0.0, 0.0);
    for (int i = 1; i <= 6; i++) samp(&w, 1000 + 2500 * i, 1 + i, 10.0f, 0.0, 0.0);   // fixes 2.5 s apart
    CHECK(!fmCvClosing(&w, 16000, 3000, &r, &cd));                     // no clean ~1 s baseline: no ceiling
    CHECK(!fmCvRiderMotion(&w, 16000, 3000, 1.39f, &sp, &crs, &cok));
    // v_max: v_band at or inside the band, e^(t_b / tau) outside it, none when not closing.
    const float vb = kBandO / kCvCoastS;
    CHECK(fabsf(vb - 2.5f) < 1e-6f);                                   // Rev 2.1 §B: 4 m / 1.6 s = 2.5 m/s (5.6 mph)
    CHECK(fabsf(fmCvVmaxMps(9.0f, 3.0f, 1.0f, 9.0f, vb, kCvCoastS) - vb) < 1e-4f);
    CHECK(fabsf(fmCvVmaxMps(9.0f + 3.0f + 3.0f * kCvCoastS, 3.0f, 1.0f, 9.0f, vb, kCvCoastS) - vb * expf(1.0f)) < 1e-3f);
    CHECK(fmCvVmaxMps(30.0f, 0.0f, 1.0f, 9.0f, vb, kCvCoastS) < 0.0f);
    CHECK(fmCvVmaxMps(30.0f, -2.0f, 1.0f, 9.0f, vb, kCvCoastS) < 0.0f);
    CHECK(fmCvVmaxMps(30.0f, 1e-6f, 1.0f, 9.0f, vb, kCvCoastS) > 1e3f);   // a tiny closure: huge, never inf
    // v_los: toward, away, no COG.
    float c = 0;
    CHECK(fabsf(fmCvLosMps(36.0f, true, 0.0f, 0.0f, &c) - 10.0f) < 1e-3f && fabsf(c - 1.0f) < 1e-5f);
    CHECK(fmCvLosMps(36.0f, true, 180.0f, 0.0f, &c) == 0.0f);          // moving away
    CHECK(fmCvLosMps(36.0f, false, 0.0f, 0.0f, &c) == 0.0f && c == 0.0f);   // no COG: unknown
    // The verdict: inputs bad -> nothing; not closing -> nothing; faster than the ceiling -> coast; else a clamp.
    FmCvVerdict v = fmCvDecide(false, 10.0f, 9.0f, 10.0f, true, 1.0f, 1.0f, 9.0f, vb, kCvCoastS, kCvCosMin);
    CHECK(!v.active && !v.coast && v.gov_max_kmh < 0.0f);
    v = fmCvDecide(true, 30.0f, -1.0f, 10.0f, true, 1.0f, 1.0f, 9.0f, vb, kCvCoastS, kCvCosMin);
    CHECK(!v.active);
    v = fmCvDecide(true, 25.0f, 9.0f, 4.5f, true, 1.0f, 1.0f, 9.0f, vb, kCvCoastS, kCvCosMin);
    CHECK(v.active && v.coast);
    v = fmCvDecide(true, 25.0f, 9.0f, 1.0f, true, 1.0f, 1.0f, 9.0f, vb, kCvCoastS, kCvCosMin);
    CHECK(v.active && !v.coast && v.gov_max_kmh > 0.0f);
    v = fmCvDecide(true, 25.0f, 9.0f, 0.1f, true, 0.2f, 1.0f, 9.0f, vb, kCvCoastS, kCvCosMin);
    CHECK(v.active && !v.coast && v.gov_max_kmh < 0.0f);               // cos under 0.25: no target clamp
    v = fmCvDecide(true, 25.0f, 9.0f, 0.0f, false, 0.0f, 1.0f, 9.0f, vb, kCvCoastS, kCvCosMin);
    CHECK(v.active && !v.coast && v.gov_max_kmh < 0.0f);               // no COG: nothing to clamp
    // The re-rise limit: a clamp to 0, then 255 per 1.5 s; never limits a fall; a gap starts fresh.
    FmCvRerise rr = { 0, 0.0f, false };
    CHECK(fmCvCap3Step(&rr, 1000, 200, false, 1500, 250) == 200);
    CHECK(fmCvCap3Step(&rr, 1100, 0, true, 1500, 250) == 0);
    uint16_t o3 = fmCvCap3Step(&rr, 1200, 200, false, 1500, 250);
    CHECK(o3 == 17);                                                   // 255 x 0.1 / 1.5
    for (uint32_t tt = 1300; tt <= 2500; tt += 100) o3 = fmCvCap3Step(&rr, tt, 200, false, 1500, 250);
    CHECK(o3 == 200 && !rr.rising);
    CHECK(fmCvCap3Step(&rr, 2600, 50, false, 1500, 250) == 50);        // a fall is never limited
    rr.rising = true; rr.cap = 0.0f;
    CHECK(fmCvCap3Step(&rr, 4000, 180, false, 1500, 250) == 180);      // after a 1.4 s gap: fresh
    CHECK(fmCvParks(1000, 7499, 6500) && !fmCvParks(1000, 7500, 6500) && !fmCvParks(0, 2000, 6500));

    // ---- THE STOP QUALIFIER (Rev 2.2 §B). Arguments: now, est_ok, est_ms, c-dot, spd_ok, spd, cfg. ----
    FmCvQual q; fmCvQualReset(&q);
    const FmCvQualCfg *qa = &kQualSa, *qb = &kQualSb;
    // S-a: one estimate 30 % down within 1.5 s, closing >= 1 m/s, from a 2 s maximum >= 2.5 m/s.
    CHECK(!fmCvQualStep(&q, 1000, true, 1000, 0.2f, true, 4.0f, qa));
    CHECK(!fmCvQualStep(&q, 1333, true, 1333, 0.6f, true, 4.0f, qa));
    CHECK(fmCvQualStep(&q, 1666, true, 1666, 1.2f, true, 2.7f, qa) && q.leg == FMCVQ_STOP && fabsf(q.ref_mps - 4.0f) < 1e-6f);
    fmCvQualReset(&q);                                                  // the same drop closing under 1 m/s: no
    fmCvQualStep(&q, 1000, true, 1000, 0.2f, true, 4.0f, qa);
    fmCvQualStep(&q, 1333, true, 1333, 0.6f, true, 4.0f, qa);
    CHECK(!fmCvQualStep(&q, 1666, true, 1666, 0.9f, true, 2.7f, qa));
    fmCvQualReset(&q);                                                  // a slow slow-down (over 1.5 s): no
    fmCvQualStep(&q, 1000, true, 1000, 0.2f, true, 4.0f, qa);
    fmCvQualStep(&q, 2700, true, 2700, 0.6f, true, 3.4f, qa);
    CHECK(!fmCvQualStep(&q, 3000, true, 3000, 1.2f, true, 2.7f, qa));
    fmCvQualReset(&q);                                                  // the speed floor: from 2.3 m/s, no
    fmCvQualStep(&q, 1000, true, 1000, 0.2f, true, 2.3f, qa);
    fmCvQualStep(&q, 1333, true, 1333, 0.6f, true, 2.3f, qa);
    CHECK(!fmCvQualStep(&q, 1666, true, 1666, 1.2f, true, 1.4f, qa));
    fmCvQualReset(&q);                                                  // he has stopped already (0 of 0): no
    for (uint32_t k = 0; k < 5; k++) CHECK(!fmCvQualStep(&q, 1000 + 333 * k, true, 1000 + 333 * k, 1.2f, true, 0.0f, qa));
    fmCvQualReset(&q);                                                  // no speed estimate: the leg cannot fire
    fmCvQualStep(&q, 1000, true, 1000, 0.2f, true, 4.0f, qa);
    CHECK(!fmCvQualStep(&q, 1333, true, 1333, 1.2f, false, 0.0f, qa));
    // S-b: at or under 60 % of the 2 s maximum on 2 consecutive estimates.
    fmCvQualReset(&q);
    fmCvQualStep(&q, 1000, true, 1000, 0.2f, true, 4.0f, qb);
    fmCvQualStep(&q, 1333, true, 1333, 0.6f, true, 4.0f, qb);
    CHECK(!fmCvQualStep(&q, 1666, true, 1666, 1.2f, true, 2.3f, qb));  // one estimate low: not yet
    CHECK(fmCvQualStep(&q, 2000, true, 2000, 1.2f, true, 2.0f, qb) && fabsf(q.ref_mps - 4.0f) < 1e-6f);   // the second
    fmCvQualReset(&q);                                                  // low once, then back up: no
    fmCvQualStep(&q, 1000, true, 1000, 0.2f, true, 4.0f, qb);
    fmCvQualStep(&q, 1333, true, 1333, 0.6f, true, 4.0f, qb);
    fmCvQualStep(&q, 1666, true, 1666, 1.2f, true, 2.3f, qb);
    CHECK(!fmCvQualStep(&q, 2000, true, 2000, 1.2f, true, 3.0f, qb));
    CHECK(!fmCvQualStep(&q, 2333, true, 2333, 1.2f, true, 2.3f, qb));
    fmCvQualReset(&q);                                                  // 65 % twice: S-b no, S-a (70 %) yes
    fmCvQualStep(&q, 1000, true, 1000, 0.2f, true, 4.0f, qb);
    fmCvQualStep(&q, 1333, true, 1333, 1.2f, true, 2.6f, qb);
    CHECK(!fmCvQualStep(&q, 1666, true, 1666, 1.2f, true, 2.6f, qb));
    fmCvQualReset(&q);
    fmCvQualStep(&q, 1000, true, 1000, 0.2f, true, 4.0f, qa);
    CHECK(fmCvQualStep(&q, 1333, true, 1333, 1.2f, true, 2.6f, qa));
    // Exit on his speed back: >= 80 % of the 2 s maximum on 2 estimates in a row.
    CHECK(fmCvQualStep(&q, 1666, true, 1666, 1.2f, true, 3.3f, qa));   // one estimate back (82.5 % of 4.0): still
    CHECK(fmCvQualStep(&q, 2000, true, 2000, 1.2f, true, 3.0f, qa));   // under 80 %: the count restarts
    CHECK(fmCvQualStep(&q, 2333, true, 2333, 1.2f, true, 3.3f, qa));
    CHECK(!fmCvQualStep(&q, 2666, true, 2666, 1.2f, true, 3.4f, qa) && q.leg == FMCVQ_NONE);   // two in a row: gone
    // Exit on the closure ending: closing under 0.5 m/s for 1 s.
    fmCvQualReset(&q);
    fmCvQualStep(&q, 1000, true, 1000, 0.2f, true, 4.0f, qa);
    CHECK(fmCvQualStep(&q, 1333, true, 1333, 1.2f, true, 2.6f, qa));
    CHECK(fmCvQualStep(&q, 1666, true, 1666, 0.4f, true, 1.0f, qa));   // the slow run starts
    CHECK(fmCvQualStep(&q, 2600, true, 2333, 0.3f, true, 0.5f, qa));   // 0.93 s: still
    CHECK(!fmCvQualStep(&q, 2700, true, 2333, 0.3f, true, 0.5f, qa));  // 1.03 s: gone
    fmCvQualReset(&q);                                                  // a slow run broken before 1 s: stays
    fmCvQualStep(&q, 1000, true, 1000, 0.2f, true, 4.0f, qa);
    fmCvQualStep(&q, 1333, true, 1333, 1.2f, true, 2.6f, qa);
    fmCvQualStep(&q, 1666, true, 1666, 0.4f, true, 1.0f, qa);
    fmCvQualStep(&q, 2000, true, 2000, 0.6f, true, 0.8f, qa);
    CHECK(fmCvQualStep(&q, 2800, true, 2333, 0.4f, true, 0.5f, qa));
    // Inputs bad: no qualification, no history (today's behaviour).
    CHECK(!fmCvQualStep(&q, 2900, false, 2333, 1.2f, true, 4.0f, qa) && !q.qualified && q.h_n == 0);

    // ---- THE CREEP'S CLOSING ABORT (FollowMePause.h, Rev 2.2 §B: 2 m closer on 2 consecutive distinct fixes) ----
    FmCreepState cs = { 0, false, -1.0f, 0, 0 };
    CHECK(fmCreepStep(&cs, 100, true, false, true, 20.0f, 15.0f, 6.0f, 3000, 20.0f, 2.0f, 1, 2) == FMCR_GRANTED);
    CHECK(fmCreepStep(&cs, 200, true, false, true, 19.0f, 15.0f, 6.0f, 3000, 17.9f, 2.0f, 2, 2) == FMCR_GRANTED);   // 1 fix 2.1 m closer
    CHECK(fmCreepStep(&cs, 300, true, false, true, 19.0f, 15.0f, 6.0f, 3000, 17.9f, 2.0f, 2, 2) == FMCR_GRANTED);   // the same fix again
    CHECK(fmCreepStep(&cs, 400, true, false, true, 18.0f, 15.0f, 6.0f, 3000, 18.0f, 2.0f, 3, 2) == FMCR_ABORT);     // 2nd fix, 2.0 m
    FmCreepState cb = { 0, false, -1.0f, 0, 0 };                       // a fix back above the line restarts the count
    CHECK(fmCreepStep(&cb, 100, true, false, true, 20.0f, 15.0f, 6.0f, 3000, 20.0f, 2.0f, 1, 2) == FMCR_GRANTED);
    CHECK(fmCreepStep(&cb, 200, true, false, true, 19.0f, 15.0f, 6.0f, 3000, 17.9f, 2.0f, 2, 2) == FMCR_GRANTED);
    CHECK(fmCreepStep(&cb, 300, true, false, true, 19.0f, 15.0f, 6.0f, 3000, 18.5f, 2.0f, 3, 2) == FMCR_GRANTED);
    CHECK(fmCreepStep(&cb, 400, true, false, true, 19.0f, 15.0f, 6.0f, 3000, 17.9f, 2.0f, 4, 2) == FMCR_GRANTED);
    CHECK(fmCreepStep(&cb, 500, true, false, true, 19.0f, 15.0f, 6.0f, 3000, 17.0f, 2.0f, 5, 2) == FMCR_ABORT);
    FmCreepState cn = { 0, false, -1.0f, 0, 0 };
    CHECK(fmCreepStep(&cn, 100, true, false, true, 20.0f, 15.0f, 6.0f, 3000, -1.0f, 2.0f, 1, 2) == FMCR_GRANTED);
    CHECK(fmCreepStep(&cn, 200, true, false, true, 15.0f, 15.0f, 6.0f, 3000, 10.0f, 2.0f, 2, 2) == FMCR_GRANTED);   // no start mean
    CHECK(fmCreepStep(&cn, 300, true, false, true, 15.0f, 15.0f, 6.0f, 3000, 10.0f, 2.0f, 3, 2) == FMCR_GRANTED);   // : today's rules
    // Its false-abort rate on a STEADY range (reported; Rev 2.2 target <= 3 %): fixes at 3 Hz with SD 1.0 m about 15 m,
    // the 2-fix mean, a creep of up to 3 s (kBootstrapMaxMs), white and correlated (AR rho 0.78, the rope data) noise.
    for (int ar = 0; ar <= 1; ar++) {
      Rng g; g.s = 4242u + ar;
      const float rho = ar ? 0.78f : 0.0f;
      long aborts = 0; const long n = 100000;
      for (long i = 0; i < n; i++) {
        float e0 = g.gauss(), prev = 15.0f + e0;
        FmCreepState st = { 0, false, -1.0f, 0, 0 };
        uint8_t res = FMCR_NONE;
        for (int j = 1; j <= 11 && res != FMCR_ABORT && res != FMCR_TIMEOUT; j++) {
          e0 = rho * e0 + sqrtf(1.0f - rho * rho) * g.gauss();
          const float d = 15.0f + e0, mean = 0.5f * (d + prev);
          prev = d;
          res = fmCreepStep(&st, (uint32_t)(1000 + 333 * j), true, false, true, 15.0f, 12.0f, 6.0f, 3000, mean, 2.0f, (uint32_t)j, 2);
        }
        if (res == FMCR_ABORT) aborts++;
      }
      printf("1. creep closing abort (2 fixes) on a steady range, SD 1.0 m %s: %.2f %% of 3 s creeps end early (%ld of %ld)%s\n",
             ar ? "correlated (rho 0.78)" : "white", 100.0 * aborts / n, aborts, n, (100.0 * aborts / n <= 3.0) ? "" : "  OVER 3 %");
    }
    printf("1. unit checks passed (%ld)\n", g_checks);
  }

  // ======================================================================================
  // 2. G-STOP AND THE STOP EFFECT (HARD at the 1.52 s plant; 1.33 and 2.4 reported)
  // ======================================================================================
  const double stop_mph[5] = { 8.0, 9.8, 10.9, 12.5, 14.0 };
  const double taus[3] = { kTauHard, kTauMed, kTauOld };
  {
    int fails[2] = { 0, 0 }, f2fail[2] = { 0, 0 }; double lo[2] = { 1e9, 1e9 }, f2m[2] = { 1e9, 1e9 };
    char ftxt[2][200] = { "", "" }, f2txt[2][200] = { "", "" };
    StopAcc se[2];
    for (int ti = 0; ti < 3; ti++) {
      printf("\n2. G-STOP (%s): rider stops dead at -9 mph/s from steady following, plant tau %.2f s, worst of 10 phases, "
             "closest ft flown / S-a / S-b\n", ti ? "report" : "HARD", taus[ti]);
      for (int fi = 0; fi < 2; fi++) for (uint8_t mode = 1; mode <= 5; mode++) {
        printf("  %s %s:", kFits[fi]->name, modeName(mode));
        for (int k = 0; k < 5; k++) {
          const StopOut f = runStop(kFits[fi], mode, kFlown, taus[ti], stop_mph[k]);
          StopOut b[2];
          for (int v = 0; v < 2; v++) b[v] = runStop(kFits[fi], mode, *kVar[v], taus[ti], stop_mph[k]);
          printf("  %4.1f mph %6.1f / %6.1f / %6.1f", stop_mph[k], f.worst, b[0].worst, b[1].worst);
          if (ti != 0) continue;
          const StopOut nq = runStop(kFits[fi], mode, kNoQ, taus[ti], stop_mph[k]);
          for (int v = 0; v < 2; v++) {
            stopEffect(se[v], b[v], nq);
            if (mode == 2 && k == 0) {                                 // 8 mph F2 (was >= flown + 2.0 ft before Rev 2.3)
              const double m = b[v].worst - f.worst;                   // Rev 2.3: no worse than flown
              if (m < f2m[v]) f2m[v] = m;
              const size_t L = strlen(f2txt[v]);
              snprintf(f2txt[v] + L, sizeof(f2txt[v]) - L, "%s%s %.1f -> %.1f ft", L ? "; " : "", fi ? "30 Oct" : "19 Oct", f.worst, b[v].worst);
              if (m < -1e-9) f2fail[v]++;
            } else {
              if (b[v].worst < lo[v]) lo[v] = b[v].worst;
              if (b[v].worst < kFloorFt) {
                fails[v]++;
                const size_t L = strlen(ftxt[v]);
                snprintf(ftxt[v] + L, sizeof(ftxt[v]) - L, "%s%s %s %.1f mph %.1f ft", L ? "; " : "", fi ? "30 Oct" : "19 Oct",
                         modeName(mode), stop_mph[k], b[v].worst);
              }
            }
          }
        }
        printf("\n");
      }
      if (quick && ti == 0) break;
    }
    for (int v = 0; v < 2; v++) {
      char d[200];
      snprintf(d, sizeof(d), "lowest %.1f ft; under 16.4: %d%s%s", lo[v], fails[v], fails[v] ? " - " : "", ftxt[v]);
      hard("G-STOP >= 16.4 ft (9.8-14 mph all; 8 mph F1/F3/F4/F5)", v, fails[v] == 0, d);
      hard("G-STOP 8 mph F2 no worse than flown, both fits (Rev 2.3)", v, f2fail[v] == 0, f2txt[v]);
      snprintf(d, sizeof(d), "%d phases compared (%d skipped: a bind before the stop); worst margin %+.1f ft; misses %d",
               se[v].n, se[v].skipped, se[v].worst, se[v].fails);
      hard("Stop effect >= min(16.4, unqualified - 1.5 ft)", v, se[v].fails == 0, d);
    }
  }
  if (quick) return 0;

  // ======================================================================================
  // 3-5. THE U-TURN SETS: NO HARM (HARD)
  // ======================================================================================
  const char *set_name[3] = { "HARSH", "REALISTIC", "WAVE-LINKING PAIRS" };
  for (int set = 0; set < 3; set++) {
    for (int ti = 0; ti < 3; ti++) {
      printf("\n%d. %s, plant tau %.2f s (%s): no-harm = closest >= min(16.4, flown - 0.5) ft and band entry <= flown + "
             "0.3 mph, per case at its worst of 10 phases; F4/F5 cases with an R2 end are 'manual' (reported, unit 2's)\n",
             3 + set, set_name[set], taus[ti], ti ? "report" : "HARD");
      if (set == 0) printf("   cases: 10 mph, 180 deg in 2 / 4 s, left / right, 82 ft back, F1-F3\n");
      if (set == 1) printf("   cases: 8/10/12/14 mph, 6/8/10 s, left / right, 100/150/200 ft back (F4/F5 settled)\n");
      if (set == 2) printf("   cases: 10 / 12 mph, 180 deg in 4 / 6 s toward, run 5/10/15 s, 180 deg back, 82 ft back, F1-F3\n");
      SetAcc acc[2]; long ends_f = 0; double lo_f = 1e9, v30_f = 0.0;
      runSet(set, taus[ti], verbose && ti == 0, ti == 0, acc, &ends_f, &lo_f, &v30_f);
      for (int v = 0; v < 2; v++) {
        printf("   TOTAL %s: %d cases (%d manual); closest flown %.1f, %s %.1f; no-harm misses %d (worst margin %+.1f ft, %s); "
               "band entry max %.1f (flown %.1f), misses %d; own line %d + %d some phases; R2 ends %ld (flown %ld), in park %ld; "
               "ceiling still binding after the range opened: worst %.1f s (report)",
               kVar[v]->name, acc[v].cases, acc[v].manual, lo_f, kVar[v]->name, acc[v].lo, acc[v].harm, acc[v].worst_margin,
               acc[v].worst_case, acc[v].v30, v30_f, acc[v].band, acc[v].own, acc[v].mixed, acc[v].ends, ends_f, acc[v].parked, acc[v].lift);
        if (ti == 0) printf("; closest vs the unqualified ceiling worst %+.1f ft (report)", acc[v].cost);
        printf("; bind ticks %ld\n", acc[v].binds);
        if (ti == 0) {
          char d[200];
          snprintf(d, sizeof(d), "%d misses of %d (worst %+.1f ft at %s); band entry misses %d", acc[v].harm, acc[v].cases - acc[v].manual,
                   acc[v].worst_margin, acc[v].worst_case, acc[v].band);
          hard(set == 0 ? "U-turns no harm, harsh" : set == 1 ? "U-turns no harm, realistic" : "U-turns no harm, pairs", v,
               acc[v].harm == 0 && acc[v].band == 0, d);
          snprintf(d, sizeof(d), "R2 ends %ld vs flown %ld; %ld in the park", acc[v].ends, ends_f, acc[v].parked);
          hard(set == 0 ? "G-UT4 ends <= flown, none parked, harsh" : set == 1 ? "G-UT4 ends <= flown, none parked, realistic"
                                                                             : "G-UT4 ends <= flown, none parked, pairs",
               v, acc[v].ends <= ends_f && acc[v].parked == 0, d);
        }
      }
    }
  }

  // ======================================================================================
  // 6. G-UT5 STOP-AND-GO (HARD) and G-UT6(a) THE PICKUP (HARD), the FLY-BY (report)
  // ======================================================================================
  {
    printf("\n6a. G-UT5 STOP-AND-GO (HARD), plant tau %.2f s: 8 -> 3 mph at -9 mph/s, 1 s, back to 8 mph at +3 mph/s; the "
           "ceiling gone <= 3 s after his speed is back to >= 80 %%; worst of 10 phases\n", kTauHard);
    double lift[2] = { 0, 0 }, clo[3] = { 1e9, 1e9, 1e9 }; long bn[2] = { 0, 0 };
    for (int fi = 0; fi < 2; fi++) for (uint8_t mode = 1; mode <= 5; mode++) for (int k = 0; k < 10; k++) {
      const StopGoOut f = runStopGo(kFits[fi], mode, kFlown, kTauHard, k);
      if (f.closest_ft < clo[2]) clo[2] = f.closest_ft;
      for (int v = 0; v < 2; v++) {
        const StopGoOut o = runStopGo(kFits[fi], mode, *kVar[v], kTauHard, k);
        if (o.lift_s > lift[v]) lift[v] = o.lift_s;
        if (o.closest_ft < clo[v]) clo[v] = o.closest_ft;
        bn[v] += o.binds;
      }
    }
    for (int v = 0; v < 2; v++) {
      char d[200];
      snprintf(d, sizeof(d), "still binding %.1f s after his speed was back (worst); %ld bind ticks; closest %.1f ft (flown %.1f)",
               lift[v], bn[v], clo[v], clo[2]);
      printf("  %s: %s\n", kVar[v]->name, d);
      hard("G-UT5 stop-and-go: ceiling gone <= 3 s after his speed is back", v, lift[v] <= 3.0 + 1e-9, d);
    }

    printf("\n6b. G-UT6(a) PICKUP (HARD, Rev 2.2 §D), plant tau %.2f s: Follow-Me ACTIVE behind him; he turns back (6 / 10 s, "
           "left / right), rides toward the buggy at 5.5 / 6.5 / 8 mph and slows at -2 mph/s to stop 10 / 20 / 30 ft from it; "
           "pass = closest no-harm, and the buggy's distance 2-8 s after he stops never more than 1.5 ft farther than flown "
           "(Rev 2.3)\n", kTauHard);
    int n = 0, harm[2] = { 0, 0 }, stand[2] = { 0, 0 }, unstopped = 0;
    double dmax[2] = { 0, 0 }, dnear[2] = { 0, 0 }, marg[2] = { 1e9, 1e9 }, bf[2] = { -1, -1 }, cf[2] = { -1, -1 }; long bnd[2] = { 0, 0 };
    const double mphs[3] = { 5.5, 6.5, 8.0 }, stops[3] = { 10.0, 20.0, 30.0 };
    for (int fi = 0; fi < 2; fi++) for (uint8_t mode = 1; mode <= 3; mode++) for (int si = 0; si < 3; si++) for (int di = 0; di < 3; di++)
      for (int ts = 6; ts <= 10; ts += 4) for (int dir = -1; dir <= 1; dir += 2) {
        PickupOut f[10], u[2][10];
        double cfl = 1e9, cu[2] = { 1e9, 1e9 };
        for (int k = 0; k < 10; k++) {
          f[k] = runPickup(kFits[fi], mode, kFlown, kTauHard, mphs[si], ts, dir, stops[di], k);
          if (!f[k].stopped) unstopped++;
          if (f[k].closest_ft < cfl) cfl = f[k].closest_ft;
          for (int v = 0; v < 2; v++) {
            u[v][k] = runPickup(kFits[fi], mode, *kVar[v], kTauHard, mphs[si], ts, dir, stops[di], k);
            if (u[v][k].closest_ft < cu[v]) cu[v] = u[v][k].closest_ft;
            bnd[v] += u[v][k].binds;
            if (u[v][k].max_bind_ft > bf[v]) bf[v] = u[v][k].max_bind_ft;
            if (u[v][k].max_coast_ft > cf[v]) cf[v] = u[v][k].max_coast_ft;
            // Rev 2.3: only FARTHER counts here (a standoff); a nearer end is judged by the no-harm rule on the closest.
            double dk = 0.0, dn = 0.0;
            for (int j = 0; j <= 60; j++) {
              if (f[k].after_ft[j] < 0.0 || u[v][k].after_ft[j] < 0.0) continue;
              const double dd = u[v][k].after_ft[j] - f[k].after_ft[j];
              if (dd > dk) dk = dd;
              if (-dd > dn) dn = -dd;
            }
            if (dk > dmax[v]) dmax[v] = dk;
            if (dn > dnear[v]) dnear[v] = dn;
            if (dk > 1.5 + 1e-9) stand[v]++;
          }
        }
        n++;
        for (int v = 0; v < 2; v++) { const double m = cu[v] - noHarmFloor(cfl); if (m < marg[v]) marg[v] = m; if (m < -1e-9) harm[v]++; }
      }
    for (int v = 0; v < 2; v++) {
      char d[200];
      snprintf(d, sizeof(d), "%d cases: no-harm misses %d (worst margin %+.1f ft); after-stop end farther than flown by up to "
               "%.2f ft (%d phases over 1.5 ft), nearer by up to %.2f ft; binds %ld, farthest bind %.1f, coast %.1f ft", n, harm[v],
               marg[v], dmax[v], stand[v], dnear[v], bnd[v], bf[v], cf[v]);
      printf("  %s: %s\n", kVar[v]->name, d);
      hard("G-UT6(a) pickup: no harm, end <= flown + 1.5 ft farther (Rev 2.3)", v, harm[v] == 0 && stand[v] == 0, d);
    }
    if (unstopped) printf("  (flown phases where he never came to a stop in the window: %d)\n", unstopped);

    printf("\n6c. FLY-BY (report, the Rev 2.1 'pickup'): 5.5 / 6.5 / 8 mph straight past a buggy at rest or creeping 2 mph, "
           "5 / 10 / 20 ft abeam, from 150 ft\n");
    for (int v = 0; v < 2; v++) {
      double dm = 0.0, b = -1.0, co = -1.0; long nb = 0;
      const double ab[3] = { 5.0, 10.0, 20.0 };
      for (int fi = 0; fi < 2; fi++) for (uint8_t mode = 1; mode <= 3; mode++) for (int si = 0; si < 3; si++) for (int ai = 0; ai < 3; ai++)
        for (int v0 = 0; v0 <= 2; v0 += 2) for (int k = 0; k < 10; k++) {
          const PickOut f = runFlyBy(kFits[fi], mode, kFlown, kTauHard, mphs[si], ab[ai], v0, k);
          const PickOut o = runFlyBy(kFits[fi], mode, *kVar[v], kTauHard, mphs[si], ab[ai], v0, k);
          if (fabs(o.closest_ft - f.closest_ft) > dm) dm = fabs(o.closest_ft - f.closest_ft);
          if (o.max_bind_ft > b) b = o.max_bind_ft;
          if (o.max_coast_ft > co) co = o.max_coast_ft;
          nb += o.binds;
        }
      printf("  %s: largest |closest - flown| %.2f ft; bind ticks %ld; farthest bind %.1f ft, farthest coast %.1f ft (-1 none)\n",
             kVar[v]->name, dm, nb, b, co);
    }
  }

  // ======================================================================================
  // 7. G-UT6(b): HOW OFTEN THE CEILING BINDS WHEN NOTHING IS WRONG (HARD: < 1 % everywhere)
  // ======================================================================================
  {
    printf("\n7. G-UT6(b) (HARD), plant tau %.2f s: ceiling-binding ticks of ACTIVE ticks (60 s after 60 s of settling), "
           "worst over 8 / 10 / 12 / 14 mph; steady, catch-up from 200 ft, carving +/-45 / 60 / 75 / 90 deg (8 s period)\n", kTauHard);
    for (int v = 0; v < 2; v++) {
      double w[6] = { 0, 0, 0, 0, 0, 0 };
      for (int fi = 0; fi < 2; fi++) for (uint8_t mode = 1; mode <= 5; mode++) {
        double x[6] = { 0, 0, 0, 0, 0, 0 };
        for (int si = 0; si < 4; si++) {
          const double mph = 8.0 + 2.0 * si;
          for (int sc = 0; sc < 6; sc++) {
            const SteadyOut o = runSteady(kFits[fi], mode, *kVar[v], kTauHard, mph, (sc < 2) ? sc : 2, (sc < 2) ? 0.0 : 45.0 + 15.0 * (sc - 2));
            const double pc = o.ticks ? 100.0 * o.binds / o.ticks : 0.0;
            if (pc > x[sc]) x[sc] = pc;
          }
        }
        printf("  %s %s %s: steady %5.2f %%  catch-up %5.2f %%  carving 45 %5.2f %%  60 %5.2f %%  75 %5.2f %%  90 %5.2f %%\n",
               kVar[v]->name, kFits[fi]->name, modeName(mode), x[0], x[1], x[2], x[3], x[4], x[5]);
        for (int sc = 0; sc < 6; sc++) if (x[sc] > w[sc]) w[sc] = x[sc];
      }
      char d[200];
      snprintf(d, sizeof(d), "worst: steady %.2f, catch-up %.2f, carving 45 %.2f / 60 %.2f / 75 %.2f / 90 %.2f %%", w[0], w[1], w[2], w[3], w[4], w[5]);
      bool ok = true; for (int sc = 0; sc < 6; sc++) if (w[sc] >= 1.0) ok = false;
      hard("G-UT6(b) under 1 % binding: steady, catch-up, carving 45-90", v, ok, d);
    }
  }

  // ======================================================================================
  // 8. RED EDITS (simulation level), on S-a: each should make some gate worse
  // ======================================================================================
  {
    printf("\n8. RED EDITS on S-a, plant tau %.2f s: G-STOP worst, 8 mph F2 worst (19 Oct fit), carving max over all angles\n", kTauHard);
    Arm r1 = kSa; r1.name = "RED range + speed from the filter"; r1.red = 1;
    Arm r2 = kSa; r2.name = "RED no latency margin";             r2.red = 2;
    Arm r4 = kSa; r4.name = "RED no qualifier";                  r4.qual = false;
    Arm r5 = kSa; r5.name = "RED no speed floor";                r5.q.floor_mps = 0.0f;
    Arm r6 = kSa; r6.name = "RED no closing test";               r6.q.stop_cdot_mps = -99.0f;
    Arm r7 = kSa; r7.name = "RED never qualifies";               r7.q.stop_ratio = -1.0f;
    const Arm *arms[7] = { &kSa, &r1, &r2, &r4, &r5, &r6, &r7 };
    for (int i = 0; i < 7; i++) {
      double lo = 1e9, f2 = 1e9, carve = 0.0;
      for (int fi = 0; fi < 2; fi++) for (uint8_t mode = 1; mode <= 5; mode++) for (int k = 0; k < 5; k++) {
        const double c = runStop(kFits[fi], mode, *arms[i], kTauHard, stop_mph[k]).worst;
        if (mode == 2 && k == 0 && fi == 0) f2 = c; else if (c < lo) lo = c;
      }
      for (int fi = 0; fi < 2; fi++) for (uint8_t mode = 1; mode <= 5; mode++) for (int si = 0; si < 4; si++)
        for (int amp = 45; amp <= 90; amp += 15) {
          const SteadyOut o = runSteady(kFits[fi], mode, *arms[i], kTauHard, 8.0 + 2.0 * si, 2, amp);
          const double pc = o.ticks ? 100.0 * o.binds / o.ticks : 0.0; if (pc > carve) carve = pc;
        }
      printf("  %-34s G-STOP worst (but 8 mph F2) %5.1f ft; 8 mph F2 %5.1f ft; carving max %5.2f %%\n", arms[i]->name, lo, f2, carve);
    }
  }

  // ======================================================================================
  // 9. THE HARD GATES (Rev 2.2 §C, re-rated by Rev 2.3), ASSERTED FOR S-b - THE SHIPPED LEG
  // ======================================================================================
  printf("\n9. HARD GATES (Rev 2.2 §C / Rev 2.3), plant tau %.2f s, controller tau_c %.1f s; S-b ships and is asserted, S-a "
         "(rejected) is reported\n", kTauHard, (double)kCvCoastS);
  int pass_all[2] = { 1, 1 }, npass[2] = { 0, 0 };
  for (int i = 0; i < g_nhard; i++) {
    for (int v = 0; v < 2; v++) {
      printf("  %-4s %-3s %-74s %s\n", g_hard[i].pass[v] ? "PASS" : "FAIL", kVar[v]->name, g_hard[i].gate, g_hard[i].detail[v]);
      if (g_hard[i].pass[v]) npass[v]++; else pass_all[v] = 0;
    }
  }
  printf("  S-a (rejected): %d of %d HARD gates pass; S-b (shipped): %d of %d (%ld unit checks asserted)\n", npass[0], g_nhard,
         npass[1], g_nhard, g_checks);
  (void)pass_all;
  for (int i = 0; i < g_nhard; i++) assert(g_hard[i].pass[1]);      // Rev 2.3: every HARD gate of the shipped leg
  printf("follow_me_converge_test: all %d HARD gates of S-b (the shipped leg) asserted\n", g_nhard);
  return 0;
}
