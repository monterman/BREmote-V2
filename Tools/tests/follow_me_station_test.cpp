// V2.5-Evo - 2026-10-06 - SETTLED-ESCAPE LATCH + M-13 + M-17 + L-17 (L-20): ctlTick() gains the 3 s settled-escape latch (a
//   millis() stand-in in Ctl), the M-13 side-flip step, the M-17 walking-on-seed rule and the L-17 rear re-seed clamp; new
//   engageEdge() restates the ACTIVE edge. New sections 29-32: the latch at phi 35 (fires at 3.1 s, never under 3 s), the psi 0
//   side flip, near_diag 90 (the seed's first snap refused through the rider), and the rear re-seed clamp.
// V2.5-Evo - 2026-10-06 - SHIELD SIZING (audits M-16 b/c, M-15, M-14, M-18, L-20): ctlTick() follows the RX - the speed-scaled
//   half-angle, the 12 / 10 km/h cone Schmitt, the lag capsule (Ctl::shortfall), the hold band, and the M-14 aim steps (input
//   edges + a > 1 m one-tick lookahead change). New sections 23-28: the half-angle curve against the band data, the Schmitt, the
//   settled-station clearance sweep against the HELD cone (855,470 configs, plus the report table and the capsule contacts), the
//   capsule geometry, the clip after a release, and a 12 km/h crossing sweep with lag and varying speed. stdout is unbuffered.
// V2.5-Evo - 2026-10-06 - RIDER SHIELD + SIDE BAND + AIM STEP + PENDING SEED + 45 DEG CEILING (audits H-2, H-3, M-10, L-11,
//   M-11, M-12, L-10): kFrontMax 80 -> 45 (ahead >= side) with section 15 asserting the 13 m ahead floor; every
//   fmFrontRetreatSide() call passes the 2 m band; new sections 16-22 - the band, the shield geometry, the D-term gate,
//   a restated non-degraded computeFmTarget() tick (ctlTick) and the kinematic simulations: an S-turn during a rear-half
//   walk, F4 engaged with the buggy ahead-left, a moving buggy with +/-1 m cross noise, and the pending seed from standstill.
// V2.5-Evo - 2026-10-06 - FRONT STATIONS BY OFFSET: section 1 and the radius inputs of sections 4 and 12
//   now come from fmFrontStationGeom() (side = the lateral floor exactly, ahead = d_follow + extra) instead
//   of the removed fmFrontRadiusM() / fmFrontAngleEffDeg(); new section 15 covers the side/ahead rule, the
//   35-80 deg derived-angle band and the fm_front_ahead_extra_m resolver (0 -> 7, 1-3 -> 4, > 10 -> 7).
// V2.5-Evo - 2026-10-02 - P2 host test for the Follow-Me station model.
//   Runs on the PC (see SOP-012 "Host-side unit tests"), exercises the exact header the RX compiles,
//   and is written so every assertion names the invariant it is defending.
//
//   g++ -std=c++17 -O1 -Wall -Wextra follow_me_station_test.cpp -o fms && ./fms
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../../Source/Common/FollowMeStation.h"

// The RX's own P2 constants (RTMState.ino), repeated here so the test states the numbers it claims.
// kFrontDefault: a representative front angle. It was kFmFrontAngleDefaultDeg until 2026-10-06; it is
// now what the side/ahead rule DERIVES whenever ahead == side, e.g. d_follow 6 + extra 7 = 13 m ahead.
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

static bool near_eq(float a, float b, float tol) { return fabsf(a - b) <= tol; }

// V2.5-Evo - 2026-10-06 - the read-site resolution, exactly as computeFmTarget() runs it: resolve the
// stored extra, then the side/ahead geometry with the RX's own angle band.
static void frontGeom(float d_follow, uint16_t stored_extra, float side,
                      float *ahead, float *phi, float *r, float *side_out = nullptr)
{
  const float extra = fmFrontAheadExtraM(stored_extra, kExtraDefault, kExtraMin, kExtraMax);
  fmFrontStationGeom(d_follow, extra, side, kFrontMin, kFrontMax, ahead, phi, r, side_out);
}

// V2.5-Evo - 2026-10-06 - one REAR-MODE tick of computeFmTarget()'s station chain, in the order the RX
// runs it (preset -> Schmitt gate -> clamp -> PG-2 ceiling -> fmRearSnap -> snap or slew -> clamp).
// Every step is the header function the RX calls; only the sequencing is restated here.
static float rearTick(uint8_t m_eff, float near_diag, bool schmitt, float phi_eff, float live,
                      bool *snapped)
{
  const float lim = fmStationLimitDeg(phi_eff);
  float psi_target = fmStationPresetDeg(m_eff, near_diag, phi_eff);
  if ((m_eff == 1 || m_eff == 3) && !schmitt) psi_target = 0.0f;
  psi_target = fmClampStationDeg(psi_target, phi_eff);
  const float ceil_mag = fmStationTargetCeilingDeg(psi_target, 0.0f, kPassLateral, lim);
  const float psi_eff_target = (psi_target >= 0.0f) ? ceil_mag : -ceil_mag;
  const bool snap = fmRearSnap(m_eff, live, near_diag);
  if (snapped) *snapped = snap;
  live = snap ? psi_eff_target : fmStationSlewStep(live, psi_eff_target, kSlewRate, 0.1f);
  return fmClampStationDeg(live, phi_eff);
}

// The 37f8b49 (SW36) rear offset: target bearing = course + 180 + offset.
static float sw36Offset(uint8_t mode, float near_diag, bool schmitt)
{
  if (mode == 1 && schmitt) return -near_diag;
  if (mode == 3 && schmitt) return +near_diag;
  return 0.0f;
}

// ======================================================================================
// V2.5-Evo - 2026-10-06 - ONE NON-DEGRADED computeFmTarget() TICK, RESTATED IN THE RX'S ORDER
// (audits H-2, H-3, M-10, L-11, M-11). Every decision is the header function the RX calls; only the
// sequencing is restated, exactly as rearTick() does for the rear chain:
//   m_eff -> pending seed -> preset / Schmitt gate / clamp -> G-3 -> H-1 (with the band) -> PG-2 -> snap
//   or slew -> radius -> PG-4 -> aim (outward, or station + clipped lookahead, or the G-5 twin) ->
//   finishAim (the shield: re-seed, F4/F5 outbound latch, go-around, the escape hold, the aim kind).
// Frames: the buggy comes in from the FILTERED rider position (b_al_f, b_cr) like fm_buggy_*; the anchor
// is lag_m ahead of it; every output coordinate is in the anchor frame.
// ======================================================================================
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
};
static float g_dlook_max_quiet = 0.0f;   // the largest one-tick lookahead change on a tick nothing else flagged

static Tick ctlTick(Ctl &c, uint8_t m_decl, float near_diag, float d_follow, uint16_t extra_stored,
                    float min_dist, float speed_kmh, float course, float b_al_f, float b_cr,
                    float lag_m, float dt, float band)
{
  Tick t;
  c.now_ms += (uint32_t)(dt * 1000.0f + 0.5f);
  float pass = kPassLateral;  if (pass < min_dist) pass = min_dist;
  float latmin = kLateralMin; if (latmin < pass) latmin = pass;
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
    else              m_eff = (uint8_t)((m_decl == 4) ? 1 : 3);
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
    m_eff = (uint8_t)((m_decl == 4) ? 1 : (m_decl == 5) ? 3 : m_decl);
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

// THE PROPERTY THE SHIELD EXISTS FOR, checked on one tick: the line the controller answers for (the
// go-around while escaping, else the segment the shield tested) does not touch the shield - unless the
// buggy is already inside the shield, or within the band of the rider's line (the accepted residual).
// Snapped rear stations are the SW36 path and are not checked. Returns true when the property holds.
static bool shieldPropertyHolds(const Tick &t, float band)
{
  if (t.snapped) return true;
  const float ea = t.escape ? t.aim_al : t.sh_al;
  const float ec = t.escape ? t.aim_cr : t.sh_cr;
  if (!fmShieldSegmentHits(t.sh, t.b_al, t.b_cr, ea, ec, 0.0f)) return true;
  const bool inside = fmShieldSegmentHits(t.sh, t.b_al, t.b_cr, t.b_al, t.b_cr, 0.0f);
  return inside || fabsf(t.b_cr) <= band + 0.001f;
}

// A tiny kinematic world for the simulations: East/North metres, bearings clockwise from North.
struct World {
  float rx = 0, ry = 0, rc = 0;          // rider position and course
  float bx = 0, by = 0, bh = 0;          // buggy position and heading
};
static void toRiderFrame(const World &w, float *along, float *cross)
{
  const float c = w.rc * BREMOTE_FMS_DEG2RAD;
  const float dx = w.bx - w.rx, dy = w.by - w.ry;
  *along = dx * sinf(c) + dy * cosf(c);
  *cross = dx * cosf(c) - dy * sinf(c);
}
// The buggy steers at the aim (anchor frame; the anchor is the rider here) with a yaw-rate limit.
static void buggyStep(World &w, float aim_al, float aim_cr, float v_ms, float yaw_dps, float dt)
{
  const float c = w.rc * BREMOTE_FMS_DEG2RAD;
  const float ax = w.rx + aim_al * sinf(c) + aim_cr * cosf(c);
  const float ay = w.ry + aim_al * cosf(c) - aim_cr * sinf(c);
  const float want = atan2f(ax - w.bx, ay - w.by) / BREMOTE_FMS_DEG2RAD;
  float err = fmsWrap180(want - w.bh);
  const float mx = yaw_dps * dt;
  if (err >  mx) err =  mx;
  if (err < -mx) err = -mx;
  w.bh += err;
  w.bx += v_ms * dt * sinf(w.bh * BREMOTE_FMS_DEG2RAD);
  w.by += v_ms * dt * cosf(w.bh * BREMOTE_FMS_DEG2RAD);
}
static void riderStep(World &w, float v_ms, float dt)
{
  w.rx += v_ms * dt * sinf(w.rc * BREMOTE_FMS_DEG2RAD);
  w.ry += v_ms * dt * cosf(w.rc * BREMOTE_FMS_DEG2RAD);
}
static void placeBuggy(World &w, float along, float cross, float heading)
{
  const float c = w.rc * BREMOTE_FMS_DEG2RAD;
  w.bx = w.rx + along * sinf(c) + cross * cosf(c);
  w.by = w.ry + along * cosf(c) - cross * sinf(c);
  w.bh = heading;
}

int main()
{
  setvbuf(stdout, nullptr, _IONBF, 0);   // V2.5-Evo - 2026-10-06 - a failing assert must not swallow the lines before it
  // ======================================================================================
  // 1. THE FRONT STATION FROM ITS OFFSETS: SIDE = THE FLOOR, AHEAD = d_follow + EXTRA
  // ======================================================================================
  {
    float ahead, phi, r;
    // The owner's example: d_follow 9 + default extra 7 -> 16 m ahead, 13 m side, 39.1 deg, 20.6 m.
    frontGeom(9.0f, 0, kLateralMin, &ahead, &phi, &r);
    assert(near_eq(ahead, 16.0f, 0.001f));
    assert(near_eq(phi, 39.09f, 0.02f));
    assert(near_eq(r, 20.62f, 0.02f));
    assert(near_eq(r * sinf(phi * BREMOTE_FMS_DEG2RAD), kLateralMin, 0.01f));   // side is the floor, exactly

    // Extra 4 at d_follow 9 reproduces the old 45 deg station: 13 m ahead x 13 m side, r 18.38.
    frontGeom(9.0f, 4, kLateralMin, &ahead, &phi, &r);
    assert(near_eq(ahead, 13.0f, 0.001f) && near_eq(phi, 45.0f, 0.01f) && near_eq(r, 18.38f, 0.01f));

    // Factory tuning 4 + 2 = 6 m with the default 7: 13 m ahead, the same 45 deg station.
    frontGeom(6.0f, 0, kLateralMin, &ahead, &phi, &r);
    assert(near_eq(ahead, 13.0f, 0.001f) && near_eq(phi, 45.0f, 0.01f));

    // A long follow distance does NOT widen the side and does NOT drop the angle under 35: ahead is
    // capped at 13 / tan35 = 18.57 m. d_follow 9 + extra 10 = 19 is just over that cap.
    frontGeom(9.0f, 10, kLateralMin, &ahead, &phi, &r);
    assert(near_eq(ahead, 18.57f, 0.01f) && near_eq(phi, kFrontMin, 0.001f));
    assert(near_eq(r * sinf(phi * BREMOTE_FMS_DEG2RAD), kLateralMin, 0.01f));
    frontGeom(22.0f, 0, kLateralMin, &ahead, &phi, &r);
    assert(near_eq(ahead, 18.57f, 0.01f) && near_eq(r, 22.67f, 0.02f));       // the old 35 deg radius
    // Past d_follow 22.67 m the radius cannot go inside the follow distance: r = d_follow, the angle
    // stays 35, and side grows with it - the safe direction, and reported, never hidden.
    float sideOut;
    frontGeom(40.0f, 0, kLateralMin, &ahead, &phi, &r, &sideOut);
    assert(near_eq(r, 40.0f, 0.001f) && near_eq(phi, kFrontMin, 0.001f));
    assert(near_eq(sideOut, 40.0f * sinf(kFrontMin * BREMOTE_FMS_DEG2RAD), 0.01f) && sideOut > kLateralMin);
    assert(near_eq(ahead, 40.0f * cosf(kFrontMin * BREMOTE_FMS_DEG2RAD), 0.01f));

    // min_dist_m raising the pass minimum above 13 m raises the side with it. V2.5-Evo - 2026-10-06 -
    // audit M-12: and ahead with it, because ahead is never less than side (16 -> 20 m at side 20).
    frontGeom(9.0f, 0, 20.0f, &ahead, &phi, &r);
    assert(near_eq(ahead, 20.0f, 0.001f) && near_eq(phi, 45.0f, 0.01f));
    assert(near_eq(r * sinf(phi * BREMOTE_FMS_DEG2RAD), 20.0f, 0.01f));
  }

  // ======================================================================================
  // 2. THE HARD FLOOR: NO INPUT CAN PUT A STATION DEAD AHEAD
  // ======================================================================================
  {
    // Sweep every angle the field could ever hold, including values no validator should allow.
    for (int phi_i = 1; phi_i <= 90; phi_i++) {
      const float phi   = (float)phi_i;
      const float limit = fmStationLimitDeg(phi);
      assert(limit <= 180.0f - 1.0f);                 // never 180: dead ahead is unreachable
      assert(limit >= 90.0f);
      for (int psi_i = -720; psi_i <= 720; psi_i += 3) {
        const float c = fmClampStationDeg((float)psi_i, phi);
        assert(fabsf(c) <= limit + 0.001f);
      }
    }
    // The two shipped angles, named.
    assert(near_eq(fmStationLimitDeg(45.0f), 135.0f, 0.001f));
    assert(near_eq(fmStationLimitDeg(35.0f), 145.0f, 0.001f));
    // A nonsense angle is clamped into (0, 90], so the limit stays inside [90, 179].
    assert(fmStationLimitDeg(0.0f)    <= 179.0f);
    assert(fmStationLimitDeg(-10.0f)  <= 179.0f);
    assert(fmStationLimitDeg(1000.0f) >= 90.0f);
  }

  // ======================================================================================
  // 3. THE PRESET TABLE, AND THE SIGN CONVENTION
  // ======================================================================================
  {
    const float phi = kFrontDefault;
    assert(near_eq(fmStationPresetDeg(1, kNearDiag, phi), +45.0f,  0.001f));   // rear-right
    assert(near_eq(fmStationPresetDeg(2, kNearDiag, phi),   0.0f,  0.001f));   // behind
    assert(near_eq(fmStationPresetDeg(3, kNearDiag, phi), -45.0f,  0.001f));   // rear-left
    assert(near_eq(fmStationPresetDeg(4, kNearDiag, phi), +135.0f, 0.001f));   // front-right
    assert(near_eq(fmStationPresetDeg(5, kNearDiag, phi), -135.0f, 0.001f));   // front-left
    // There is no mode 6, and no mode 6 behaviour: anything unknown is "directly behind".
    assert(near_eq(fmStationPresetDeg(6,    kNearDiag, phi), 0.0f, 0.001f));
    assert(near_eq(fmStationPresetDeg(0,    kNearDiag, phi), 0.0f, 0.001f));
    assert(near_eq(fmStationPresetDeg(0xFF, kNearDiag, phi), 0.0f, 0.001f));
    // Every preset is inside the floor by construction (the clamp is a no-op on all five).
    for (uint8_t m = 1; m <= 5; m++) {
      const float p = fmStationPresetDeg(m, kNearDiag, phi);
      assert(near_eq(fmClampStationDeg(p, phi), p, 0.001f));
    }
    // At the 35 deg minimum the front presets move out to +/-145, still never 180.
    assert(near_eq(fmStationPresetDeg(4, kNearDiag, kFrontMin), +145.0f, 0.001f));
    assert(near_eq(fmStationPresetDeg(5, kNearDiag, kFrontMin), -145.0f, 0.001f));
    // A near_diag the validator would never store cannot push a REAR preset ahead of the rider.
    assert(fabsf(fmStationPresetDeg(1, 170.0f, phi)) <= 90.0f);
    // The retreat target keeps the side: 4 -> rear-right, 5 -> rear-left.
    assert(near_eq(fmStationNearestRearPresetDeg(4, kNearDiag), +45.0f, 0.001f));
    assert(near_eq(fmStationNearestRearPresetDeg(5, kNearDiag), -45.0f, 0.001f));
    assert(near_eq(fmStationNearestRearPresetDeg(2, kNearDiag),   0.0f, 0.001f));
  }

  // ======================================================================================
  // 4. THE RADIUS SCHEDULE, AND PG-1 (THE STATION'S OWN CROSS-TRACK FLOOR)
  // ======================================================================================
  {
    const float phi = kFrontDefault;

    // ---- d_follow 9, extra 4: 13 x 13 m, r_front 18.38 (V2.5-Evo - 2026-10-06: from fmFrontStationGeom) ----
    {
      const float d = 9.0f;
      float ah, ph, r;
      frontGeom(d, 4, kLateralMin, &ah, &ph, &r);
      assert(near_eq(ph, phi, 0.01f));
      // Rear presets sit at exactly d_follow - today's geometry, unchanged.
      assert(near_eq(fmStationRadiusM(  0.0f, d, r, kNearDiag, kPassLateral, phi), d, 0.001f));
      assert(near_eq(fmStationRadiusM(+45.0f, d, r, kNearDiag, kPassLateral, phi), d, 0.001f));
      assert(near_eq(fmStationRadiusM(-45.0f, d, r, kNearDiag, kPassLateral, phi), d, 0.001f));
      // Front presets sit at exactly r_front, both sides.
      assert(near_eq(fmStationRadiusM(+135.0f, d, r, kNearDiag, kPassLateral, phi), r, 0.01f));
      assert(near_eq(fmStationRadiusM(-135.0f, d, r, kNearDiag, kPassLateral, phi), r, 0.01f));
      // Abeam: the ramp alone gives about 13.7 m at this tuning, comfortably over the pass minimum.
      const float rAbeam = fmStationRadiusM(90.0f, d, r, kNearDiag, kPassLateral, phi);
      assert(near_eq(rAbeam, 13.69f, 0.05f));
      assert(rAbeam >= kPassLateral);
    }

    // ---- Factory tuning 4 + 2 = 6, default extra 7 (13 x 13 m): abeam 12.2 m, still over the pass minimum ----
    {
      const float d = 6.0f;
      float ah, ph, r;
      frontGeom(d, 0, kLateralMin, &ah, &ph, &r);
      assert(near_eq(ph, phi, 0.01f));
      const float rAbeam = fmStationRadiusM(90.0f, d, r, kNearDiag, kPassLateral, phi);
      assert(near_eq(rAbeam, 12.19f, 0.05f));
      assert(rAbeam >= kPassLateral);
    }

    // ---- Degenerate tuning d_follow 0.5: the RAMP alone would be 9.44 m at abeam, UNDER the
    //      10.1 m pass minimum. PG-1's floor is what stops the station inviting a close pass. ----
    // V2.5-Evo - 2026-10-06: at d 0.5 + extra 4 the derived station is raised to 13 m ahead x 13 m side (45 deg,
    //   r 18.38 - audit M-12; it was 4.5 m ahead at 70.9 deg under the old 80 deg ceiling); the ramp is still
    //   under the minimum at abeam.
    {
      const float d = 0.5f;
      float ah, ph, r;
      frontGeom(d, 4, kLateralMin, &ah, &ph, &r);
      const float lim0 = fmStationLimitDeg(ph);
      const float ramp_only = d + (r - d) * ((90.0f - kNearDiag) / (lim0 - kNearDiag));
      assert(ramp_only < kPassLateral);                       // the hazard is real at this tuning
      const float rAbeam = fmStationRadiusM(90.0f, d, r, kNearDiag, kPassLateral, ph);
      assert(near_eq(rAbeam, kPassLateral, 0.01f));           // floored to exactly the minimum
    }

    // ---- PG-1 as a sweep: every station from abeam to the floor keeps cross >= pass_lateral,
    //      at every one of these tunings and at every legal extra (V2.5-Evo - 2026-10-06: the angle
    //      and radius are DERIVED per tuning by fmFrontStationGeom, as the RX does). ----
    const float dFollows[5] = { 0.5f, 2.0f, 6.0f, 9.0f, 20.0f };
    for (int ai = 4; ai <= 10; ai++) {
      for (int di = 0; di < 5; di++) {
        const float d = dFollows[di];
        float ah, a, r;
        frontGeom(d, (uint16_t)ai, kLateralMin, &ah, &a, &r);
        const float lim = fmStationLimitDeg(a);
        assert(r >= d);                                        // the radius never goes inward
        float prev = -1.0f;
        for (float psi = 0.0f; psi <= lim + 0.001f; psi += 1.0f) {
          const float rr = fmStationRadiusM(psi, d, r, kNearDiag, kPassLateral, a);
          assert(rr >= d - 0.001f);                            // never inside the follow station
          assert(rr >= prev - 0.001f);                         // monotone non-decreasing in |psi|
          prev = rr;
          float along, cross;
          fmStationAlongCrossM(psi, rr, &along, &cross);
          if (psi >= 90.0f) {
            assert(cross >= kPassLateral - 0.02f);             // PG-1
          }
          // And the mirror image on the left is identical in magnitude.
          float along_l, cross_l;
          fmStationAlongCrossM(-psi, fmStationRadiusM(-psi, d, r, kNearDiag, kPassLateral, a),
                               &along_l, &cross_l);
          assert(near_eq(along_l, along, 0.01f));
          assert(near_eq(cross_l, -cross, 0.01f));
        }
        // At the front limit the clearance is the full lateral invariant, not just the pass minimum.
        const float rLim = fmStationRadiusM(lim, d, r, kNearDiag, kPassLateral, a);
        float alongL, crossL;
        fmStationAlongCrossM(lim, rLim, &alongL, &crossL);
        assert(crossL >= kLateralMin - 0.02f);
        assert(near_eq(crossL, kLateralMin, 0.02f));           // 2026-10-06: and never MORE than the floor
        assert(near_eq(alongL, ah, 0.02f));                    // ahead is what the geometry said
        assert(alongL > 0.0f);                                 // a front station really is ahead
      }
    }
  }

  // ======================================================================================
  // 5. THE RIDER-FRAME COORDINATES, AND THE SIGNS THE WHOLE DESIGN RESTS ON
  // ======================================================================================
  {
    float along, cross;
    // Directly behind.
    fmStationAlongCrossM(0.0f, 9.0f, &along, &cross);
    assert(near_eq(along, -9.0f, 0.001f) && near_eq(cross, 0.0f, 0.001f));
    // Rear-right: behind and to the RIGHT.
    fmStationAlongCrossM(+45.0f, 9.0f, &along, &cross);
    assert(along < 0.0f && cross > 0.0f);
    // Rear-left: behind and to the LEFT.
    fmStationAlongCrossM(-45.0f, 9.0f, &along, &cross);
    assert(along < 0.0f && cross < 0.0f);
    // Abeam right: level with the rider, to the right.
    fmStationAlongCrossM(+90.0f, 13.7f, &along, &cross);
    assert(near_eq(along, 0.0f, 0.001f) && near_eq(cross, 13.7f, 0.001f));
    // Front-right at the default: 13.0 m ahead and 13.0 m right - the invariant as a coordinate.
    fmStationAlongCrossM(+135.0f, 18.38f, &along, &cross);
    assert(near_eq(along, 13.0f, 0.02f) && near_eq(cross, 13.0f, 0.02f));

    // The buggy's own coordinates, from a bearing and a distance.
    // Rider heading North (course 0). Buggy due South of them = directly behind.
    fmBuggyAlongCrossM(0.0f, 180.0f, 10.0f, &along, &cross);
    assert(near_eq(along, -10.0f, 0.01f) && near_eq(cross, 0.0f, 0.01f));
    // Buggy due East = abeam to the rider's right.
    fmBuggyAlongCrossM(0.0f, 90.0f, 10.0f, &along, &cross);
    assert(near_eq(along, 0.0f, 0.01f) && near_eq(cross, 10.0f, 0.01f));
    // Buggy due North = dead ahead (the station model never commands this; the test proves the sign).
    fmBuggyAlongCrossM(0.0f, 0.0f, 10.0f, &along, &cross);
    assert(near_eq(along, 10.0f, 0.01f) && near_eq(cross, 0.0f, 0.01f));
    // Same geometry with the rider heading West, to prove the frame rotates with the course.
    fmBuggyAlongCrossM(270.0f, 180.0f, 10.0f, &along, &cross);
    assert(near_eq(along, 0.0f, 0.01f) && near_eq(cross, -10.0f, 0.01f));

    // G-4: the measured station angle is the exact inverse of the bearing convention.
    for (float course = 0.0f; course < 360.0f; course += 17.0f) {
      for (float psi = -140.0f; psi <= 140.0f; psi += 13.0f) {
        const float bearing = course + 180.0f - psi;
        assert(near_eq(fmsWrap180(fmMeasuredStationDeg(course, bearing) - psi), 0.0f, 0.01f));
      }
    }
  }

  // ======================================================================================
  // 6. PG-2: THE STATION CANNOT GO AHEAD OF THE RIDER UNTIL THE BUGGY IS WIDE
  // ======================================================================================
  {
    const float lim = fmStationLimitDeg(kFrontDefault);     // 135

    // A REAR target is never gated.
    assert(near_eq(fmStationTargetCeilingDeg(+45.0f, 0.0f, kPassLateral, lim), 45.0f, 0.001f));
    assert(near_eq(fmStationTargetCeilingDeg(  0.0f, 0.0f, kPassLateral, lim),  0.0f, 0.001f));

    // A FRONT target with the buggy ON the rider's line: held at abeam, never beyond.
    assert(near_eq(fmStationTargetCeilingDeg(+135.0f, 0.0f, kPassLateral, lim), 90.0f, 0.001f));
    // Buggy wide but NOT YET wide enough (10.0 m against a 10.1 m minimum): still held.
    assert(near_eq(fmStationTargetCeilingDeg(+135.0f, 10.0f, kPassLateral, lim), 90.0f, 0.001f));
    // Buggy wide enough, on the right side: released to the full front station.
    assert(near_eq(fmStationTargetCeilingDeg(+135.0f, 10.1f, kPassLateral, lim), 135.0f, 0.001f));
    assert(near_eq(fmStationTargetCeilingDeg(+135.0f, 25.0f, kPassLateral, lim), 135.0f, 0.001f));
    // WRONG SIDE IS NOT WIDE. A buggy 25 m to the LEFT does not authorise a station to the RIGHT.
    assert(near_eq(fmStationTargetCeilingDeg(+135.0f, -25.0f, kPassLateral, lim), 90.0f, 0.001f));
    assert(near_eq(fmStationTargetCeilingDeg(-135.0f, +25.0f, kPassLateral, lim), 90.0f, 0.001f));
    // Mirror image for the left front station.
    assert(near_eq(fmStationTargetCeilingDeg(-135.0f, -10.1f, kPassLateral, lim), 135.0f, 0.001f));
    assert(near_eq(fmStationTargetCeilingDeg(-135.0f,   0.0f, kPassLateral, lim),  90.0f, 0.001f));
    // The ceiling never exceeds the hard floor, whatever is asked for.
    assert(fmStationTargetCeilingDeg(+179.0f, 100.0f, kPassLateral, lim) <= lim + 0.001f);
  }

  // ======================================================================================
  // 7. THE SLEW: PACED, AND IT NEVER CROSSES DEAD AHEAD
  // ======================================================================================
  {
    const float dt = 0.1f;                                   // the 10 Hz FM tick
    const float step = kSlewRate * dt;                       // 1.5 deg per tick

    // One step is exactly the rate.
    assert(near_eq(fmStationSlewStep(0.0f, 135.0f, kSlewRate, dt), step, 0.001f));
    assert(near_eq(fmStationSlewStep(0.0f, -135.0f, kSlewRate, dt), -step, 0.001f));
    // It arrives exactly, never overshoots.
    assert(near_eq(fmStationSlewStep(134.9f, 135.0f, kSlewRate, dt), 135.0f, 0.001f));
    assert(near_eq(fmStationSlewStep(135.0f, 135.0f, kSlewRate, dt), 135.0f, 0.001f));
    // A zero rate or a zero dt freezes it (this is also how the controller parks the station).
    assert(near_eq(fmStationSlewStep(90.0f, 135.0f, 0.0f, dt), 90.0f, 0.001f));
    assert(near_eq(fmStationSlewStep(90.0f, 135.0f, kSlewRate, 0.0f), 90.0f, 0.001f));

    // A SIDE CHANGE GOES THROUGH 0 (behind the rider), NEVER THROUGH 180 (dead ahead).
    // F4 (+135) to F5 (-135), walked tick by tick: psi must visit 0 and must never exceed 135.
    float psi = +135.0f;
    bool saw_behind = false;
    int ticks = 0;
    for (; ticks < 5000; ticks++) {
      psi = fmStationSlewStep(psi, -135.0f, kSlewRate, dt);
      assert(fabsf(psi) <= 135.0f + 0.001f);                 // never leaves the legal arc
      if (fabsf(psi) < 2.0f) saw_behind = true;
      if (near_eq(psi, -135.0f, 0.001f)) break;
    }
    assert(saw_behind);
    assert(ticks < 5000);
    // 270 deg at 15 deg/s = 18 s = 180 ticks of 100 ms.
    assert(ticks >= 179 && ticks <= 181);

    // Coming home from a front station passes through abeam and settles behind, monotonically.
    psi = +135.0f;
    float prev = psi;
    for (int i = 0; i < 200; i++) {
      psi = fmStationSlewStep(psi, 0.0f, kSlewRate, dt);
      assert(psi <= prev + 0.001f);
      prev = psi;
    }
    assert(near_eq(psi, 0.0f, 0.001f));
  }

  // ======================================================================================
  // 8. PG-3: THE AIM LINE NEVER CROSSES THE RIDER'S LINE
  // ======================================================================================
  {
    // Both endpoints wide on the same side -> proven clear.
    assert(fmAimClearsRiderLine(+10.1f, +13.0f, kPassLateral));
    assert(fmAimClearsRiderLine(+25.0f, +13.0f, kPassLateral));
    assert(fmAimClearsRiderLine(-10.1f, -13.0f, kPassLateral));
    // Opposite sides -> NOT clear, however wide each is: the line crosses the rider's line.
    assert(!fmAimClearsRiderLine(-25.0f, +13.0f, kPassLateral));
    assert(!fmAimClearsRiderLine(+25.0f, -13.0f, kPassLateral));
    // Either endpoint inside the minimum -> not clear.
    assert(!fmAimClearsRiderLine(+10.0f, +13.0f, kPassLateral));
    assert(!fmAimClearsRiderLine(+13.0f,  +9.0f, kPassLateral));
    assert(!fmAimClearsRiderLine(  0.0f, +13.0f, kPassLateral));

    // THE AFFINE ARGUMENT, CHECKED NUMERICALLY. Walk the segment from the buggy to the station and
    // confirm the minimum cross-track offset really is at an endpoint - that is what makes the two
    // endpoint tests above a proof about the whole line rather than about two points.
    const float cases[4][4] = {
      //  buggy_along, buggy_cross, station_along, station_cross
      {  -20.0f, +10.1f,  +13.0f, +13.0f },     // the pass itself, at the legal minimum
      {   -5.0f, +25.0f,  +13.0f, +13.0f },     // a wide buggy coming forward
      {  +30.0f, +13.0f,  +13.0f, +13.0f },     // too far ahead, coming back
      {  -20.0f, -10.1f,  +13.0f, -13.0f },     // the left-hand mirror
    };
    for (int c = 0; c < 4; c++) {
      const float bx = cases[c][0], by = cases[c][1];
      const float sx = cases[c][2], sy = cases[c][3];
      assert(fmAimClearsRiderLine(by, sy, kPassLateral));
      float worst = 1e9f;
      for (int k = 0; k <= 100; k++) {
        const float t = (float)k / 100.0f;
        const float cross = by + t * (sy - by);
        (void)(bx + t * (sx - bx));             // along is irrelevant to the cross-track minimum
        if (fabsf(cross) < worst) worst = fabsf(cross);
      }
      assert(worst >= kPassLateral - 0.001f);   // every point of the aim line clears the line
    }

    // PG-4: the escape test. Only ever true while the station is STRICTLY AHEAD.
    assert(!fmAimOutwardNeeded(+45.0f, 0.0f, 0.0f, kPassLateral));          // rear station: never
    assert(!fmAimOutwardNeeded(  0.0f, 0.0f, 0.0f, kPassLateral));
    // ABEAM IS NOT AHEAD, and that boundary is load-bearing. psi = 90 is the WAITING waypoint of the
    // two-waypoint pass - the station level with the rider and pass_lateral to the side, which is
    // where the buggy is sent BECAUSE it is not yet wide enough to go in front. If abeam counted as
    // ahead, the manoeuvre that earns the pass would trip the escape that exists to abort it, and
    // the G-4 engage seed (clamped to +/-90) could fire the escape on the very first tick.
    assert(!fmAimOutwardNeeded(+90.0f, 0.0f, +13.7f, kPassLateral));
    assert(!fmAimOutwardNeeded(-90.0f, 0.0f, -13.7f, kPassLateral));
    assert(!fmStationIsFront(+90.0f) && !fmStationIsFront(-90.0f));
    assert( fmStationIsFront(+90.1f) &&  fmStationIsFront(-90.1f));
    // One degree past abeam it IS ahead, so the escape arms immediately.
    assert( fmAimOutwardNeeded(+91.0f, 0.0f, +13.7f, kPassLateral));
    assert( fmAimOutwardNeeded(+135.0f, +2.0f, +13.0f, kPassLateral));      // rider turned into us
    assert( fmAimOutwardNeeded(+135.0f, -20.0f, +13.0f, kPassLateral));     // wrong side
    assert(!fmAimOutwardNeeded(+135.0f, +11.0f, +13.0f, kPassLateral));     // still lawfully wide
    // Deliberately independent of the along-track term: the affine argument never used it, so its
    // failure test does not either. A buggy BEHIND the rider with a front station and no clearance
    // is just as much a corridor break as one ahead.
    assert( fmAimOutwardNeeded(+135.0f, +1.0f, +13.0f, kPassLateral));

    // The escape waypoint is always further out than the buggy, and never inside the minimum.
    for (float bc = -30.0f; bc <= 30.0f; bc += 1.0f) {
      const float lat = fmOutwardAimLateralM(+135.0f, bc, kPassLateral, 9.0f);
      assert(lat > 0.0f);                                    // on the station's side
      assert(lat >= kPassLateral);                           // never inside the pass minimum
      assert(lat >= fabsf(bc) + 9.0f - 0.001f);              // genuinely outward, with margin
      const float latL = fmOutwardAimLateralM(-135.0f, bc, kPassLateral, 9.0f);
      assert(near_eq(latL, -lat, 0.001f));
    }
  }

  // ======================================================================================
  // 9. THE GOVERNOR BYPASS IS A GEOMETRY PREDICATE, AND IT IS PARTIAL
  // ======================================================================================
  {
    // The only case that bypasses the fade: course valid, station ahead, buggy measurably ahead,
    // and passing beside the line by at least the pass minimum.
    assert( fmFadeBypass(true,  +135.0f, +5.0f, +13.0f, kPassLateral));
    assert( fmFadeBypass(true,  -135.0f, +5.0f, -13.0f, kPassLateral));
    // No course -> no "ahead" exists -> ordinary fade.
    assert(!fmFadeBypass(false, +135.0f, +5.0f, +13.0f, kPassLateral));
    // Station not ahead -> ordinary fade (this is every rear station, i.e. today's behaviour).
    assert(!fmFadeBypass(true,   +45.0f, +5.0f, +13.0f, kPassLateral));
    assert(!fmFadeBypass(true,    +0.0f, +5.0f, +13.0f, kPassLateral));
    assert(!fmFadeBypass(true,   +90.0f, +5.0f, +13.0f, kPassLateral));   // abeam is not "ahead"
    // Commanded ahead but NOT measurably ahead -> ordinary fade. The predicate reads the buggy's
    // measured position, never the mode it was asked for.
    assert(!fmFadeBypass(true,  +135.0f,  0.0f, +13.0f, kPassLateral));
    assert(!fmFadeBypass(true,  +135.0f, -5.0f, +13.0f, kPassLateral));
    // Ahead but passing too close to the line -> ordinary fade. This is the term that makes it a
    // pass rather than an overtake, and it is the one Rex will attack hardest.
    assert(!fmFadeBypass(true,  +135.0f, +5.0f, +10.0f, kPassLateral));
    assert(!fmFadeBypass(true,  +135.0f, +5.0f,   0.0f, kPassLateral));
    // The cross term is a magnitude, so a left-hand pass with a right-hand station still bypasses
    // only on clearance - the side agreement is PG-2's job, upstream of this predicate.
    assert( fmFadeBypass(true,  +135.0f, +5.0f, -13.0f, kPassLateral));
    // Sweep: along <= 0 or |cross| < pass_lateral is ALWAYS false, at every station angle.
    for (float psi = -145.0f; psi <= 145.0f; psi += 5.0f) {
      for (float along = -20.0f; along <= 20.0f; along += 2.0f) {
        for (float cross = -20.0f; cross <= 20.0f; cross += 2.0f) {
          const bool b = fmFadeBypass(true, psi, along, cross, kPassLateral);
          if (along <= 0.0f || fabsf(cross) < kPassLateral || fabsf(psi) <= 90.0f) assert(!b);
          else assert(b);
        }
      }
    }
  }

  // ======================================================================================
  // 10. THE ALONG-TRACK GOVERNOR TERM: SIGNED, CLAMPED, AND IT CAN SUBTRACT
  // ======================================================================================
  {
    const float gain = 0.5f, maxk = 15.0f;
    // At station: no adjustment.
    assert(near_eq(fmStationAlongGovKmh(13.0f, 13.0f, gain, maxk), 0.0f, 0.001f));
    // Behind station: positive, so the buggy may catch up.
    assert(near_eq(fmStationAlongGovKmh(13.0f, 3.0f, gain, maxk), +5.0f, 0.001f));
    // AHEAD of station: NEGATIVE, so the buggy gives throttle back instead of running on.
    assert(near_eq(fmStationAlongGovKmh(13.0f, 23.0f, gain, maxk), -5.0f, 0.001f));
    // Clamped both ways at the same ceiling the radial gap term uses.
    assert(near_eq(fmStationAlongGovKmh(13.0f, -200.0f, gain, maxk), +maxk, 0.001f));
    assert(near_eq(fmStationAlongGovKmh(13.0f, +200.0f, gain, maxk), -maxk, 0.001f));
  }

  // ======================================================================================
  // 11. fmRearSnap - THE REAR-STATION STEP TEST (audit L-8: the shipped predicate, not a copy)
  // ======================================================================================
  {
    // Rear modes inside the band snap; the band edge is inclusive.
    for (uint8_t m = 1; m <= 3; m++) {
      assert( fmRearSnap(m,   0.0f, 45.0f));
      assert( fmRearSnap(m, +45.0f, 45.0f));
      assert( fmRearSnap(m, -45.0f, 45.0f));
      assert(!fmRearSnap(m, +45.1f, 45.0f));               // coming home from the front: walks
      assert(!fmRearSnap(m, -135.0f, 45.0f));
    }
    // Front modes and unknown modes never snap.
    assert(!fmRearSnap(4, 0.0f, 45.0f) && !fmRearSnap(5, 0.0f, 45.0f));
    assert(!fmRearSnap(0, 0.0f, 45.0f) && !fmRearSnap(0xFF, 0.0f, 45.0f));
    // near_diag 0: only exactly-behind snaps (no step exists anyway).
    assert( fmRearSnap(2, 0.0f, 0.0f) && !fmRearSnap(2, 0.1f, 0.0f));
    // near_diag >= 90 is clamped to 90 on both sides, as the presets are.
    assert( fmRearSnap(1, 90.0f, 170.0f) && !fmRearSnap(1, 90.1f, 170.0f));
    // Negative near_diag is clamped to 0.
    assert( fmRearSnap(2, 0.0f, -5.0f) && !fmRearSnap(2, 1.0f, -5.0f));
    // A snapped preset sits exactly on the band edge (the same float and clamp as the preset).
    for (int nd = 0; nd <= 180; nd++) {
      const float p1 = fmStationPresetDeg(1, (float)nd, kFrontDefault);
      const float p3 = fmStationPresetDeg(3, (float)nd, kFrontDefault);
      assert(fmRearSnap(1, p1, (float)nd) && fmRearSnap(3, p3, (float)nd));
    }
  }

  // ======================================================================================
  // 12. AUDIT H-1: THE BUGGY ON THE OTHER SIDE OF THE RIDER'S LINE - RETREAT ON THE BUGGY'S SIDE
  // ======================================================================================
  {
    // ---- the side test itself ----
    assert(fmFrontRetreatSide(+135.0f, true,  -13.0f, kSideBand) == -1);   // station right, buggy left
    assert(fmFrontRetreatSide(-135.0f, true,  +13.0f, kSideBand) == +1);   // mirror
    assert(fmFrontRetreatSide(+135.0f, true,  +2.0f, kSideBand)  ==  0);   // same side: PG-4 outward handles it
    assert(fmFrontRetreatSide(+135.0f, true,   0.0f, kSideBand)  ==  0);   // on the line counts as station side
    assert(fmFrontRetreatSide(+120.0f, false, -5.0f, kSideBand)  == -1);   // walking home from the front, rear target
    // The abeam WAITING waypoint: counts only while a front station is wanted.
    assert(fmFrontRetreatSide(+90.0f,  true,  -3.0f, kSideBand)  == -1);
    assert(fmFrontRetreatSide(-90.0f,  true,  +3.0f, kSideBand)  == +1);
    assert(fmFrontRetreatSide(+90.0f,  false, -3.0f, kSideBand)  ==  0);   // a rear preset at near_diag 90: SW36 path
    // Behind abeam it never fires - that is every rear station.
    for (float psi = -90.0f; psi <= 90.0f; psi += 0.5f) {
      for (float c = -30.0f; c <= 30.0f; c += 1.0f) {
        assert(fmFrontRetreatSide(psi, false, c, kSideBand) == 0);
        if (fabsf(psi) < 90.0f) assert(fmFrontRetreatSide(psi, true, c, kSideBand) == 0);
      }
    }

    // ---- the seed: the measured angle, on the buggy's side, clamped to the hard floor ----
    assert(near_eq(fmRetreatSeedDeg(-1, -135.0f, 135.0f), -135.0f, 0.001f));
    assert(near_eq(fmRetreatSeedDeg(-1, -170.0f, 135.0f), -135.0f, 0.001f));   // never past the floor
    assert(near_eq(fmRetreatSeedDeg(+1, -0.0001f, 135.0f), +0.0001f, 0.0001f)); // sign forced to the side
    assert(near_eq(fmRetreatSeedDeg(-1, +30.0f, 135.0f),  -30.0f, 0.001f));

    // ---- THE AUDIT SCENARIO: rider turns 90 deg RIGHT toward a settled F4 (+135) station ----
    const float dF   = 9.0f;
    const float phi  = kFrontDefault;
    const float lim  = fmStationLimitDeg(phi);
    // V2.5-Evo - 2026-10-06: d_follow 9 + extra 4 = 13 m ahead x 13 m side, the scenario's 45 deg station.
    float aF, phF, rF;
    frontGeom(dF, 4, kLateralMin, &aF, &phF, &rF);
    assert(near_eq(phF, phi, 0.01f));
    // Before the turn: course North, buggy settled on F4 = 13 m ahead, 13 m right = bearing 45.
    float along, cross;
    fmBuggyAlongCrossM(0.0f, 45.0f, rF, &along, &cross);
    assert(cross > kPassLateral);                               // lawfully wide on the right
    // After the turn: course East. Same world position, bearing 45 from the rider.
    const float course = 90.0f;
    fmBuggyAlongCrossM(course, 45.0f, rF, &along, &cross);
    assert(along > 0.0f && cross < 0.0f);                       // ahead-LEFT of the new course
    // The pre-fix escape put its waypoint on the station's (right) side: across the line, ahead of him.
    assert(fmOutwardAimLateralM(+135.0f, cross, kPassLateral, dF) > 0.0f);
    // The fix: the side test fires toward the buggy's side, and the seed is on that side.
    const int side = fmFrontRetreatSide(+135.0f, true, cross, kSideBand);
    assert(side == -1);
    float live = fmRetreatSeedDeg(side, fmMeasuredStationDeg(course, 45.0f), lim);
    assert(live < 0.0f && fabsf(live) <= lim + 0.001f);
    // F4 retreats to the rear preset on the BUGGY'S side (mode 3); the side-zone Schmitt is disengaged
    // with the buggy ahead, so the target is directly behind (0), and the walk must stay on the left.
    for (int sch = 0; sch <= 1; sch++) {
      float lv = live;
      bool snapped = false;
      int ticks = 0;
      for (; ticks < 400; ticks++) {
        lv = rearTick(3, kNearDiag, sch != 0, phi, lv, &snapped);
        assert(lv <= 0.0f + 0.001f);                            // never on the far side of the line
        const float rr = fmStationRadiusM(lv, dF, rF, kNearDiag, kPassLateral, phi);
        float sa, sc;
        fmStationAlongCrossM(lv, rr, &sa, &sc);
        assert(sc <= 0.001f);                                   // the station point stays left
        // If the escape stands this tick, its waypoint is on the buggy's side too.
        if (fmAimOutwardNeeded(lv, cross, sc, kPassLateral)) {
          assert(fmOutwardAimLateralM(lv, cross, kPassLateral, dF) < 0.0f);
        }
        // And the side test never flips it back while the buggy stays left.
        assert(fmFrontRetreatSide(lv, false, cross, kSideBand) == 0);
        if (snapped) break;
      }
      assert(snapped);                                          // it does get home
      assert(ticks <= 70);                                      // 135 -> 45 at 15 deg/s = 60 ticks
    }

    // ---- the JIBE: rider reverses course with F4 settled; the buggy ends up BEHIND-left ----
    {
      const float c2 = 180.0f;
      fmBuggyAlongCrossM(c2, 45.0f, rF, &along, &cross);
      assert(along < 0.0f && cross < 0.0f);
      assert(fmFrontRetreatSide(+135.0f, true, cross, kSideBand) == -1);   // fires without any along-track term
      const float sd = fmRetreatSeedDeg(-1, fmMeasuredStationDeg(c2, 45.0f), lim);
      assert(sd < 0.0f && fabsf(sd) <= 90.0f);                  // measured behind-left: a rear angle
    }
  }

  // ======================================================================================
  // 13. REAR-ONLY RIDING IS STILL SW36 (37f8b49): BEARING AND RADIUS, BITWISE
  // ======================================================================================
  {
    long cases = 0;
    const float phis[3] = { 35.0f, 45.0f, 80.0f };
    for (int pi = 0; pi < 3; pi++) {
      const float phi = phis[pi];
      for (int nd = 0; nd <= 90; nd++) {
        for (uint8_t m = 1; m <= 3; m++) {
          for (int sch = 0; sch <= 1; sch++) {
            // Every live angle rear-only riding can leave behind: 0 (the ACTIVE edge) and every
            // rear preset of every mode/Schmitt combination at this near_diag.
            const float priors[3] = { 0.0f, +(float)nd, -(float)nd };
            for (int k = 0; k < 3; k++) {
              bool snapped = false;
              const float live = rearTick(m, (float)nd, sch != 0, phi, priors[k], &snapped);
              assert(snapped);                                  // one tick, no walking
              assert(fmFrontRetreatSide(live, false, -20.0f, kSideBand) == 0);   // H-1 never fires
              assert(fmFrontRetreatSide(live, false, +20.0f, kSideBand) == 0);
              for (float course = 0.0f; course < 360.0f; course += 7.5f) {
                const float b_new  = course + 180.0f - live;
                const float b_sw36 = course + 180.0f + sw36Offset(m, (float)nd, sch != 0);
                assert(memcmp(&b_new, &b_sw36, sizeof(float)) == 0);   // bitwise
                cases++;
              }
            }
          }
        }
      }
    }
    // Mode / Schmitt sequences: every tick steps, from any rear state to any other.
    float live = 0.0f;
    unsigned seed = 12345u;
    for (int i = 0; i < 200000; i++) {
      seed = seed * 1103515245u + 12345u;
      const uint8_t m  = (uint8_t)(1 + (seed >> 16) % 3);
      const bool   sch = ((seed >> 20) & 1u) != 0;
      const float  nd  = (float)((seed >> 8) % 91);
      bool snapped = false;
      live = rearTick(m, nd, sch, kFrontDefault, live, &snapped);
      // Changing near_diag mid-run may leave the angle outside a SMALLER band (audit L-6 i); that walk
      // is the accepted difference. Otherwise every tick snaps to the SW36 offset.
      if (snapped) {
        const float off = sw36Offset(m, nd, sch);
        assert(live == -off);                                   // psi = -offset, exactly
      }
      cases++;
    }
    printf("rear-only SW36 equivalence: %ld cases\n", cases);
  }

  // ======================================================================================
  // 14. AUDIT M-8: THE ACTIVE-EDGE SEED - HOLD -> SWITCH -> SQUEEZE, EVERY MODE PAIR
  // ======================================================================================
  {
    // The rule.
    assert(fmEngageSeedDeg(2, true,  +150.0f) == +90.0f);
    assert(fmEngageSeedDeg(1, true,  -100.0f) == -90.0f);
    assert(fmEngageSeedDeg(3, true,   +90.0f) ==   0.0f);   // abeam is not ahead: SW36
    assert(fmEngageSeedDeg(3, true,   +30.0f) ==   0.0f);
    assert(fmEngageSeedDeg(4, true,   +30.0f) == +30.0f);   // G-4 unchanged
    assert(fmEngageSeedDeg(5, true,  +170.0f) == +90.0f);
    assert(fmEngageSeedDeg(2, false, +150.0f) ==   0.0f);   // no course
    assert(fmEngageSeedDeg(0, true,  +150.0f) ==   0.0f);

    const float dF  = 9.0f, phi = kFrontDefault;
    long seq = 0;
    for (uint8_t prev = 1; prev <= 5; prev++) {            // the mode before the release
      for (uint8_t next = 1; next <= 5; next++) {          // the mode switched to while floating
        for (int nd_i = 0; nd_i < 3; nd_i++) {
          const float nd = (nd_i == 0) ? 0.0f : (nd_i == 1) ? kNearDiag : 90.0f;
          for (float meas = -179.0f; meas <= 180.0f; meas += 1.0f) {
            // The previous engagement's live angle is irrelevant: the ACTIVE edge overwrites it.
            (void)prev;
            const float seed = fmEngageSeedDeg(next, true, meas);
            assert(fabsf(seed) <= 90.0f);                  // never begins ahead of abeam
            float bc;                                      // the buggy's cross-track side = sign(sin(meas))
            fmStationAlongCrossM(meas, 10.0f, nullptr, &bc);
            // Dead ahead (meas = 180, |bc| ~ 1e-7) has no side; judge only a buggy measurably off the line.
            const bool sided = fabsf(bc) > 0.001f;
            if (seed != 0.0f && sided) assert((seed > 0.0f) == (bc > 0.0f));   // on the buggy's side
            // H-1 can never fire on the engage tick: the seed is on the buggy's side.
            const bool fw = (next == 4 || next == 5);
            if (sided) assert(fmFrontRetreatSide(seed, fw, bc, kSideBand) == 0);
            if (next <= 3) {
              if (fabsf(meas) <= 90.0f) assert(seed == 0.0f);          // SW36: snap on tick 1
              // Walk it: monotone, on the buggy's side, then one snap behind the rider.
              for (int sch = 0; sch <= 1; sch++) {
                float lv = seed, prev_a = fabsf(seed);
                bool snapped = false;
                for (int t = 0; t < 200 && !snapped; t++) {
                  const float before = lv;
                  lv = rearTick(next, nd, sch != 0, phi, lv, &snapped);
                  if (!snapped) {
                    assert(fabsf(lv) <= prev_a + 0.001f);  // walking toward behind, never outward
                    assert(lv == 0.0f || (lv > 0.0f) == (seed > 0.0f));
                    prev_a = fabsf(lv);
                  } else {
                    assert(fabsf(before) <= (nd > 90.0f ? 90.0f : nd) + 0.001f);  // the step is from inside the rear band
                  }
                }
                assert(snapped);
                // Radius and side during the walk are the station schedule's; the station stays behind abeam.
                (void)dF;
              }
            }
            seq++;
          }
        }
      }
    }
    printf("M-8 HOLD->switch->squeeze sequences: %ld\n", seq);
  }

  // ======================================================================================
  // 15. V2.5-Evo - 2026-10-06 - FRONT STATIONS BY OFFSET: SIDE = FLOOR, AHEAD = FORMULA, ANGLE BAND
  // ======================================================================================
  {
    // ---- the stored-field resolver: 0 -> 7, 1-3 -> 4, 4-10 as is, above 10 -> 7 ----
    assert(fmFrontAheadExtraM(0, kExtraDefault, kExtraMin, kExtraMax) == 7.0f);   // every fielded SW36 board
    for (uint16_t v = 1; v <= 3; v++) assert(fmFrontAheadExtraM(v, kExtraDefault, kExtraMin, kExtraMax) == 4.0f);
    for (uint16_t v = 4; v <= 10; v++) assert(fmFrontAheadExtraM(v, kExtraDefault, kExtraMin, kExtraMax) == (float)v);
    for (uint32_t v = 11; v <= 65535; v++) {
      assert(fmFrontAheadExtraM((uint16_t)v, kExtraDefault, kExtraMin, kExtraMax) == 7.0f);
    }
    // An old Front Station Angle still in the blob (0 or 35-80 deg) never reads as a distance.
    assert(fmFrontAheadExtraM(45, kExtraDefault, kExtraMin, kExtraMax) == 7.0f);
    assert(fmFrontAheadExtraM(35, kExtraDefault, kExtraMin, kExtraMax) == 7.0f);
    assert(fmFrontAheadExtraM(80, kExtraDefault, kExtraMin, kExtraMax) == 7.0f);

    // ---- the geometry, swept: every d_follow 0.5..60 m, every stored extra 0..12, four side floors ----
    const float ahead_hi_13 = kLateralMin / tanf(kFrontMin * BREMOTE_FMS_DEG2RAD);   // 18.57 m
    assert(near_eq(ahead_hi_13, 18.566f, 0.01f));
    const float sides[4] = { kLateralMin, 15.0f, 20.0f, 30.0f };
    long cases = 0;
    for (int si = 0; si < 4; si++) {
      const float side = sides[si];
      const float hi = side / tanf(kFrontMin * BREMOTE_FMS_DEG2RAD);
      const float lo = side / tanf(kFrontMax * BREMOTE_FMS_DEG2RAD);
      for (int stored = 0; stored <= 12; stored++) {
        const float extra = fmFrontAheadExtraM((uint16_t)stored, kExtraDefault, kExtraMin, kExtraMax);
        float prev_ahead = -1.0f;
        const float r_cap = side / sinf(kFrontMin * BREMOTE_FMS_DEG2RAD);   // 22.67 m at side 13
        for (float d = 0.5f; d <= 60.0f; d += 0.5f) {
          float ahead, phi, r, sd;
          frontGeom(d, (uint16_t)stored, side, &ahead, &phi, &r, &sd);
          assert(near_eq(r * sinf(phi * BREMOTE_FMS_DEG2RAD), sd, 0.01f));   // side output is the real one
          assert(r >= d - 0.0001f);                                     // never inside the follow distance
          if (d <= r_cap) {
            // SIDE IS THE FLOOR, EXACTLY: never less (clearance) and never more (owner: "no more than 13 m").
            assert(near_eq(sd, side, 0.01f));
            // AHEAD IS THE FORMULA, held only by the angle band.
            const float want = d + extra;
            if (want <= hi && want >= lo) assert(near_eq(ahead, want, 0.0001f));
            else if (want > hi)           assert(near_eq(ahead, hi, 0.001f) && ahead < want);
            else                          assert(near_eq(ahead, lo, 0.001f));
            // phi / r CONSISTENT with side and ahead.
            assert(near_eq(phi, atan2f(side, ahead) / BREMOTE_FMS_DEG2RAD, 0.01f));
            assert(near_eq(r, sqrtf(side * side + ahead * ahead), 0.001f));
          } else {
            // Long follow distance: r = d_follow at the 35 deg angle, side WIDER than the floor.
            assert(near_eq(r, d, 0.0001f) && near_eq(phi, kFrontMin, 0.0001f));
            assert(sd >= side - 0.001f);
          }
          assert(ahead >= lo - 0.001f);                                 // genuinely ahead of abeam
          assert(near_eq(r * cosf(phi * BREMOTE_FMS_DEG2RAD), ahead, 0.01f));
          // THE ANGLE BAND: dead ahead stays unreachable (35 deg floor), and the station is never abeam.
          assert(phi >= kFrontMin - 0.0001f && phi <= kFrontMax + 0.0001f);
          assert(fmStationLimitDeg(phi) <= 180.0f - kFrontMin + 0.0001f);
          // Ahead never shrinks as the follow distance grows.
          assert(ahead >= prev_ahead - 0.0001f);
          prev_ahead = ahead;
          // THE FULL CHAIN: preset 4/5 at the derived angle, radius schedule at the limit, station point.
          for (uint8_t m = 4; m <= 5; m++) {
            const float pre = fmStationPresetDeg(m, kNearDiag, phi);
            assert(near_eq(fmClampStationDeg(pre, phi), pre, 0.0001f));
            const float pass = (side > kLateralMin) ? side : kPassLateral;   // the RX: side > 13 only when min_dist_m raised the pass minimum to it
            const float rr = fmStationRadiusM(pre, d, r, kNearDiag, pass, phi);
            float sa, sc;
            fmStationAlongCrossM(pre, rr, &sa, &sc);
            assert(near_eq(fabsf(sc), sd, 0.02f));                      // the station is `side` to the side
            assert(fabsf(sc) >= side - 0.02f);                          // and never inside the floor
            assert(near_eq(sa, ahead, 0.02f));                          // and `ahead` ahead
            assert((m == 4) ? (sc > 0.0f) : (sc < 0.0f));               // F4 right, F5 left
          }
          cases++;
        }
      }
    }
    // Degenerate inputs stay inside the band.
    float ah, ph, rr;
    fmFrontStationGeom(-5.0f, -3.0f, 0.0f, kFrontMin, kFrontMax, &ah, &ph, &rr, nullptr);
    assert(ph >= kFrontMin - 0.0001f && ph <= kFrontMax + 0.0001f && ah > 0.0f && rr > 0.0f);
    fmFrontStationGeom(9.0f, 7.0f, 13.0f, kFrontMin, kFrontMax, nullptr, nullptr, nullptr, nullptr);   // null-safe
    // ---- V2.5-Evo - 2026-10-06 - audit M-12: AHEAD IS NEVER LESS THAN SIDE (the 45 deg ceiling) ----
    {
      float a, p, r;
      frontGeom(6.0f, 4, kLateralMin, &a, &p, &r);       // d 6 + 4 = 10 -> raised to 13 ahead
      assert(near_eq(a, 13.0f, 0.001f) && near_eq(p, 45.0f, 0.01f) && near_eq(r, 18.38f, 0.01f));
      frontGeom(0.5f, 4, kLateralMin, &a, &p, &r);       // the degenerate d 0.5: 13 ahead too
      assert(near_eq(a, 13.0f, 0.001f) && near_eq(p, 45.0f, 0.01f));
      frontGeom(9.0f, 0, kLateralMin, &a, &p, &r);       // the owner's d 9 + 7 = 16: untouched
      assert(near_eq(a, 16.0f, 0.001f) && near_eq(p, 39.09f, 0.02f));
      for (int stored = 0; stored <= 12; stored++) {
        for (float d = 0.5f; d <= 22.5f; d += 0.5f) {
          for (int si = 0; si < 4; si++) {
            float sd;
            frontGeom(d, (uint16_t)stored, sides[si], &a, &p, &r, &sd);
            assert(a >= sd - 0.001f);                     // never less far ahead than to the side
            assert(p <= 45.0f + 0.0001f);
          }
        }
      }
    }
    printf("front station side/ahead: %ld cases\n", cases);
  }

  // ======================================================================================
  // 16. V2.5-Evo - 2026-10-06 - AUDIT H-3: THE 2 m SCHMITT BAND ON THE SIDE TEST
  // ======================================================================================
  {
    assert(fmFrontRetreatSide(+135.0f, true, -1.9f, kSideBand) ==  0);   // inside the band: keep the side
    assert(fmFrontRetreatSide(+135.0f, true, -2.1f, kSideBand) == -1);   // beyond it: retreat
    assert(fmFrontRetreatSide(-135.0f, true, +1.9f, kSideBand) ==  0);
    assert(fmFrontRetreatSide(-135.0f, true, +2.1f, kSideBand) == +1);
    assert(fmFrontRetreatSide(+135.0f, true, -0.1f, 0.0f)      == -1);   // band 0 = the old strict test
    assert(fmFrontRetreatSide(+135.0f, true, -0.1f, -5.0f)     == -1);   // a negative band reads as 0
    assert(fmBuggySideWithBand(+1, -1.9f, kSideBand) == +1);
    assert(fmBuggySideWithBand(-1, +1.9f, kSideBand) == -1);
    assert(fmBuggySideWithBand(+1, -2.1f, kSideBand) == -1);
    assert(fmBuggySideWithBand(-1, +2.1f, kSideBand) == +1);
    assert(fmBuggySideWithBand( 0, -0.5f, kSideBand) == -1);              // no committed side: the buggy's sign
    assert(fmBuggySideWithBand( 0,  0.0f, kSideBand) == +1);
    assert(fmBuggySideWithBand( 0, +0.5f, kSideBand) == +1);

    // ---- A MOVING BUGGY WITH +/-1 m CROSS-TRACK NOISE, the station held ahead (front wanted) ----
    // The buggy drifts from 1 m right of the rider's line to 5 m left of it while closing from 13 m to 9 m
    // ahead; every fix carries uniform +/-1 m of cross noise. Strict: the side flips on the noise and
    // every flip re-seeds the station across the line. Band: exactly one retreat, when it is really left.
    int reseeds[2] = { 0, 0 };
    for (int bi = 0; bi < 2; bi++) {
      const float band = (bi == 0) ? kSideBand : 0.0f;
      float live = +135.0f;
      unsigned seed = 777u;
      for (int i = 0; i <= 200; i++) {
        seed = seed * 1103515245u + 12345u;
        const float noise = ((float)((seed >> 8) % 2001u) / 1000.0f) - 1.0f;   // -1 .. +1 m
        const float along = 13.0f - 0.02f * (float)i;
        const float cross = (1.0f - 0.03f * (float)i) + noise;
        const float meas  = fmMeasuredStationDeg(0.0f, atan2f(cross, along) / BREMOTE_FMS_DEG2RAD);
        const int side = fmFrontRetreatSide(live, true, cross, band);
        if (side != 0) { live = fmRetreatSeedDeg(side, meas, 135.0f); reseeds[bi]++; }
      }
    }
    printf("H-3 moving buggy, +/-1 m noise: %d re-seed(s) with the 2 m band, %d strict\n", reseeds[0], reseeds[1]);
    assert(reseeds[0] == 1);
    assert(reseeds[1] >= 5);                                   // the chatter the band removes is real

    // ---- the same track through the FULL chain (ctlTick): F4 settled, then the buggy drifts across ----
    for (int bi = 0; bi < 2; bi++) {
      const float band = (bi == 0) ? kSideBand : 0.0f;
      Ctl c; c.live = +135.0f;
      unsigned seed = 777u;
      int side_changes = 0, last_side = 0, reseed_ticks = 0;
      for (int i = 0; i <= 200; i++) {
        seed = seed * 1103515245u + 12345u;
        const float noise = ((float)((seed >> 8) % 2001u) / 1000.0f) - 1.0f;
        const float along = 13.0f - 0.02f * (float)i;
        const float cross = (1.0f - 0.03f * (float)i) + noise;
        const Tick t = ctlTick(c, 4, kNearDiag, 6.0f, 0, 4.0f, 18.0f, 0.0f, along, cross, 0.0f, 0.1f, band);
        if (t.h1 || t.sh_reseed) { reseed_ticks++; assert(t.step); }    // M-10: every re-seed skips D
        const int side = (c.live > 0.0f) ? +1 : (c.live < 0.0f) ? -1 : 0;
        if (side != 0 && last_side != 0 && side != last_side) side_changes++;
        if (side != 0) last_side = side;
        if (bi == 0) assert(shieldPropertyHolds(t, band));
      }
      printf("H-3 full chain, band %.0f m: %d re-seed tick(s), %d station side change(s)\n",
             (double)band, reseed_ticks, side_changes);
      if (bi == 0) assert(side_changes <= 1 && reseed_ticks <= 2);
      else         assert(side_changes >= 3);
    }
  }

  // ======================================================================================
  // 17. V2.5-Evo - 2026-10-06 - AUDIT H-2: THE SHIELD GEOMETRY
  // ======================================================================================
  {
    // ---- speed gating, and the circle never smaller than min_dist_m ----
    FmShield s = fmShieldMake(11.9f, kShHorizonS, kShHalfDeg, kShMinKmh, kShCircleM, 4.0f, 100.0f);
    assert(s.cone_len_m == 0.0f && near_eq(s.circle_r_m, 4.0f, 0.0001f));        // floating: circle only
    s = fmShieldMake(12.0f, kShHorizonS, kShHalfDeg, kShMinKmh, kShCircleM, 2.0f, 100.0f);
    assert(near_eq(s.cone_len_m, 16.667f, 0.01f) && near_eq(s.circle_r_m, 3.0f, 0.0001f));
    s = fmShieldMake(18.0f, kShHorizonS, kShHalfDeg, kShMinKmh, kShCircleM, 4.0f, 100.0f);
    assert(near_eq(s.cone_len_m, 25.0f, 0.01f) && near_eq(s.cone_tan, 0.57735f, 0.0001f));

    // ---- the circle ----
    const FmShield slow = fmShieldMake(8.0f, kShHorizonS, kShHalfDeg, kShMinKmh, kShCircleM, 4.0f, 100.0f);
    assert( fmShieldSegmentHits(slow, -10.0f, 3.9f, 10.0f, 3.9f, 0.0f));          // passes 3.9 m off: inside 4
    assert(!fmShieldSegmentHits(slow, -10.0f, 4.1f, 10.0f, 4.1f, 0.0f));          // 4.1 m off: clear
    assert(!fmShieldSegmentHits(slow, -10.0f, 4.1f, 10.0f, 4.1f, 2.0f));          // the hold never widens the circle
    assert(!fmShieldSegmentHits(slow, -10.0f, 4.0f, 10.0f, 4.0f, 0.0f));          // exactly at the radius: clear
    {                                                                              // capped at the follow distance
      const FmShield tiny = fmShieldMake(8.0f, kShHorizonS, kShHalfDeg, kShMinKmh, kShCircleM, 2.0f, 2.5f);
      assert(near_eq(tiny.circle_r_m, 2.5f, 0.0001f));
      assert(!fmShieldSegmentHits(tiny, -6.0f, 0.0f, -2.5f, 0.0f, 0.0f));         // its own rear station is reachable
    }
    // The widened cone never reaches behind the rider: a rear station at 4 m directly behind stays clear.
    assert(!fmShieldSegmentHits(s, -8.0f, 0.0f, -4.0f, 0.0f, 2.0f));
    assert(!fmShieldSegmentHits(slow,  -9.0f, -9.0f, -6.0f, -9.0f, 0.0f));        // nowhere near

    // ---- the cone (18 km/h: 25 m deep, +/-30 deg) ----
    assert( fmShieldSegmentHits(s, 10.0f, -8.0f, 10.0f, 8.0f, 0.0f));             // straight across his path
    assert(!fmShieldSegmentHits(slow, 10.0f, -8.0f, 10.0f, 8.0f, 0.0f));          // not foiling: circle only
    assert(!fmShieldSegmentHits(s, 30.0f, -8.0f, 30.0f, 8.0f, 0.0f));             // beyond 5 s of travel
    assert(!fmShieldSegmentHits(s, 10.0f,  7.0f, 20.0f, 13.0f, 0.0f));            // beside the cone the whole way
    assert(!fmShieldSegmentHits(s, 10.0f,  6.5f, 10.0f, 6.5f, 0.0f));             // a point 0.7 m outside ...
    assert( fmShieldSegmentHits(s, 10.0f,  6.5f, 10.0f, 6.5f, 2.0f));             // ... is inside the 2 m hold
    assert( fmShieldSegmentHits(s, 10.0f,  5.0f, 10.0f, 5.0f, 0.0f));             // a point inside

    // ---- the segment test against brute-force sampling, 20,000 random segments x 4 shields ----
    {
      const float speeds[4] = { 0.0f, 12.0f, 18.0f, 30.0f };
      unsigned seed = 4242u;
      long checked = 0;
      for (int si = 0; si < 4; si++) {
        const FmShield sh = fmShieldMake(speeds[si], kShHorizonS, kShHalfDeg, kShMinKmh, kShCircleM, 4.0f, 100.0f);
        for (int k = 0; k < 20000; k++) {
          float v[4];
          for (int j = 0; j < 4; j++) {
            seed = seed * 1103515245u + 12345u;
            v[j] = ((float)((seed >> 8) % 8001u) / 100.0f) - 40.0f;            // -40 .. +40 m
          }
          bool sampled = false, near = false;
          for (int n = 0; n <= 400; n++) {
            const float u = (float)n / 400.0f;
            const float x = v[0] + u * (v[2] - v[0]), y = v[1] + u * (v[3] - v[1]);
            const bool inC = (x * x + y * y) <  sh.circle_r_m * sh.circle_r_m;
            const bool inT = sh.cone_len_m > 0.0f && x >= 0.0f && x <= sh.cone_len_m &&
                             fabsf(y) <= x * sh.cone_tan;
            if (inC || inT) sampled = true;
            const float rc = sh.circle_r_m + 0.3f;
            const bool nC = (x * x + y * y) <= rc * rc;
            const bool nT = sh.cone_len_m > 0.0f && x >= -0.6f && x <= sh.cone_len_m + 0.3f &&
                            fabsf(y) <= (x + 0.6f) * sh.cone_tan;
            if (nC || nT) near = true;
          }
          const bool hit = fmShieldSegmentHits(sh, v[0], v[1], v[2], v[3], 0.0f);
          if (sampled) assert(hit);                    // never misses a real contact
          if (hit)     assert(near);                   // and never reports one far from the shield
          checked++;
        }
      }
      printf("shield segment test vs sampling: %ld segments\n", checked);
    }

    // ---- the widened (hold) shield, point by point: the circle unchanged, the cone sides out by 2 m
    //      measured square to each side, its base 2 m deeper, and nothing behind the rider ----
    {
      const float w0 = 2.0f * sqrtf(1.0f + s.cone_tan * s.cone_tan);
      long pts = 0;
      for (float x = -10.0f; x <= 40.0f; x += 0.5f) {
        for (float y = -30.0f; y <= 30.0f; y += 0.5f) {
          const bool inC = (x * x + y * y) < s.circle_r_m * s.circle_r_m;
          const bool inW = x >= 0.0f && x <= s.cone_len_m + 2.0f && fabsf(y) <= x * s.cone_tan + w0;
          assert(fmShieldSegmentHits(s, x, y, x, y, 2.0f) == (inC || inW));
          pts++;
        }
      }
      printf("shield hold region, point by point: %ld points\n", pts);
    }

    // ---- the half-width ----
    assert(near_eq(fmShieldHalfWidthM(s, 10.0f), 5.7735f, 0.001f));
    assert(near_eq(fmShieldHalfWidthM(s,  1.0f), sqrtf(15.0f), 0.001f));          // the circle wins near him
    assert(near_eq(fmShieldHalfWidthM(s, -2.0f), sqrtf(12.0f), 0.001f));          // behind: circle only
    assert(fmShieldHalfWidthM(s, 30.0f) == 0.0f && fmShieldHalfWidthM(slow, 10.0f) == 0.0f);

    // ---- the lookahead clip: the owner's d_follow 9 + 7 F4 (16 m x 13 m), lookahead 9 m ----
    {
      float ah, ph, r;
      frontGeom(9.0f, 0, kLateralMin, &ah, &ph, &r);
      float sa, sc;
      fmStationAlongCrossM(fmStationLimitDeg(ph), fmStationRadiusM(fmStationLimitDeg(ph), 9.0f, r, kNearDiag,
                           kPassLateral, ph), &sa, &sc);
      assert(near_eq(sa, 16.0f, 0.02f) && near_eq(sc, 13.0f, 0.02f));
      // The settled station itself is outside the cone at every speed (39 deg > 30 deg) ...
      const FmShield fast = fmShieldMake(40.0f, kShHorizonS, kShHalfDeg, kShMinKmh, kShCircleM, 4.0f, 100.0f);
      assert(!fmShieldSegmentHits(fast, sa, sc, sa, sc, 0.0f));
      // ... but the full lookahead (25 x 13 m, 27.5 deg) is inside it at 18 km/h, so it is shortened.
      assert(fmShieldSegmentHits(s, sa, sc, sa + 9.0f, sc, 0.0f));
      const float lc = fmShieldClipLookaheadM(s, 0.0f, sa, sc, sa, sc, 9.0f);
      assert(lc > 6.3f && lc < 6.53f);                                            // 13 / tan30 = 22.5 m
      assert(!fmShieldSegmentHits(s, sa, sc, sa + lc, sc, 0.0f));
      assert( fmShieldSegmentHits(s, sa, sc, sa + lc + 0.05f, sc, 0.0f));
      // At 14 km/h (19.4 m deep) the full 9 m is clear and is left alone.
      const FmShield s14 = fmShieldMake(14.0f, kShHorizonS, kShHalfDeg, kShMinKmh, kShCircleM, 4.0f, 100.0f);
      assert(fmShieldClipLookaheadM(s14, 0.0f, sa, sc, sa, sc, 9.0f) == 9.0f);
      // The factory 13 x 13 m station with its 6 m lookahead (19 x 13 m, 34 deg) is never clipped.
      assert(fmShieldClipLookaheadM(fast, 0.0f, 13.0f, 13.0f, 13.0f, 13.0f, 6.0f) == 6.0f);
      // A clip can only shorten, and 0 in is 0 out.
      assert(fmShieldClipLookaheadM(s, 0.0f, sa, sc, sa, sc, 0.0f) == 0.0f);
    }

    // ---- THE GO-AROUND PROOF, swept: a buggy outside the shield and beyond the band is never sent
    //      through it, at four speeds, over a 71 x 61 m grid ----
    {
      const float speeds[4] = { 0.0f, 12.0f, 18.0f, 30.0f };
      long n = 0;
      for (int si = 0; si < 4; si++) {
        const FmShield sh = fmShieldMake(speeds[si], kShHorizonS, kShHalfDeg, kShMinKmh, kShCircleM, 4.0f, 100.0f);
        for (float a = -30.0f; a <= 40.0f; a += 1.0f) {
          for (float c = -30.0f; c <= 30.0f; c += 1.0f) {
            if (fabsf(c) <= kSideBand) continue;
            if (fmShieldSegmentHits(sh, a, c, a, c, 0.0f)) continue;              // already inside
            const int side = (c > 0.0f) ? +1 : -1;
            const float lat = fmShieldGoAroundLateralM(sh, side, a, c, kPassLateral, 6.0f);
            assert((lat > 0.0f) == (side > 0));
            assert(fabsf(lat) >= kPassLateral && fabsf(lat) >= fabsf(c) + 6.0f - 0.001f);
            assert(!fmShieldSegmentHits(sh, a, c, a, lat, 0.0f));
            n++;
          }
        }
      }
      printf("shield go-around proof: %ld buggy positions\n", n);
    }

    // ---- fmShieldDecide ----
    {
      // Audit H-2's own example: the buggy 10 m ahead / 3 m RIGHT, a station walking at psi -70 (left).
      float sa, sc;
      fmStationAlongCrossM(-70.0f, 12.0f, &sa, &sc);
      FmShieldDecision d = fmShieldDecide(s, false, 0, kSideBand, kSideBand, -70.0f, 10.0f, 3.0f, sa, sc, kPassLateral, 6.0f);
      assert(d.escape && d.side == +1 && d.reseed && d.lateral_m > 0.0f);
      // Inside the band the committed (station) side stands: no re-seed across the line on noise.
      d = fmShieldDecide(s, false, 0, kSideBand, kSideBand, -70.0f, 10.0f, 1.5f, sa, sc, kPassLateral, 6.0f);
      assert(d.escape && d.side == -1 && !d.reseed && d.lateral_m < 0.0f);
      // Directly behind (psi 0), near the line: the buggy's own sign, and NO re-seed - a station exactly
      // behind the rider has no side to be wrong about (re-seeding would send it back out ahead).
      d = fmShieldDecide(s, false, 0, kSideBand, kSideBand, 0.0f, 10.0f, -0.5f, -9.0f, 0.0f, kPassLateral, 6.0f);
      assert(d.escape && d.side == -1 && !d.reseed);
      // ... and while that escape stands, its side is held against noise inside the band.
      d = fmShieldDecide(s, true, -1, kSideBand, kSideBand, 0.0f, 10.0f, +1.5f, -9.0f, 0.0f, kPassLateral, 6.0f);
      assert(d.escape && d.side == -1 && !d.reseed);
      d = fmShieldDecide(s, true, -1, kSideBand, kSideBand, 0.0f, 10.0f, +2.5f, -9.0f, 0.0f, kPassLateral, 6.0f);
      assert(d.escape && d.side == +1);                       // beyond the band it follows the buggy
      // A clear aim: nothing to do.
      d = fmShieldDecide(s, false, 0, kSideBand, kSideBand, +45.0f, -9.0f, 7.0f, -6.0f, 6.0f, kPassLateral, 6.0f);
      assert(!d.escape && !d.reseed && d.lateral_m == 0.0f);
      // THE HOLD: an aim 1 m outside the cone does not start an escape, but holds one that stands.
      d = fmShieldDecide(s, false, 0, kSideBand, kSideBand, +100.0f, 12.0f, 8.0f, 12.0f, 7.93f, kPassLateral, 6.0f);
      assert(!d.escape);
      d = fmShieldDecide(s, true, 0, kSideBand, kSideBand, +100.0f, 12.0f, 8.0f, 12.0f, 7.93f, kPassLateral, 6.0f);
      assert(d.escape);
    }
  }

  // ======================================================================================
  // 18. V2.5-Evo - 2026-10-06 - AUDITS M-10 + L-11: THE D-TERM GATE (fmDTermContinuous)
  // ======================================================================================
  {
    const uint32_t src = 1;
    // Two consecutive H-1 re-seeds on opposite sides both carry kProfRetreat. On the label alone the
    // second one WAS differentiated - a station step of up to 270 deg as one tick of steering rate:
    assert( fmDTermContinuous(true, src, src, kPrRetreat, kPrRetreat, false));   // what the labels allowed
    assert(!fmDTermContinuous(true, src, src, kPrRetreat, kPrRetreat, true));    // the one-shot step skips it
    // The G-5 twin switching on or off under one label (kProfTransit both sides): skipped.
    assert(!fmDTermContinuous(true, src, src, kPrTransit, kPrTransit, true));
    // An ordinary continuous tick still differentiates; a source or label change still skips.
    assert( fmDTermContinuous(true, src, src, kPrTransit, kPrTransit, false));
    assert(!fmDTermContinuous(true, 2u, src, kPrTransit, kPrTransit, false));
    assert(!fmDTermContinuous(true, src, src, kPrFront, kPrTransit, false));
    assert(!fmDTermContinuous(false, src, src, kPrTransit, kPrTransit, false));
    // With no aim step the gate is exactly the 2026-09-17 three-term test - rear-only riding unchanged.
    for (int v = 0; v <= 1; v++)
      for (uint32_t a = 1; a <= 2; a++)
        for (uint32_t b = 1; b <= 2; b++)
          for (uint8_t p = 1; p <= 10; p++)
            for (uint8_t q = 1; q <= 10; q++)
              assert(fmDTermContinuous(v != 0, a, b, p, q, false) == ((v != 0) && a == b && p == q));

    // Through the chain: back-to-back retreats (the strict test on a buggy straddling the line) - every
    // re-seed tick carries the step, so no two consecutive re-seeds are ever differentiated.
    Ctl c; c.live = +135.0f;
    int consecutive = 0, prev_reseed = 0;
    for (int i = 0; i < 60; i++) {
      const float cross = (i % 2 == 0) ? -0.6f : +0.6f;
      const Tick t = ctlTick(c, 2, kNearDiag, 6.0f, 0, 4.0f, 8.0f, 0.0f, 12.0f, cross, 0.0f, 0.1f, 0.0f);
      const int rs = (t.h1 || t.sh_reseed) ? 1 : 0;
      if (rs) assert(t.step);
      if (rs && prev_reseed) consecutive++;
      prev_reseed = rs;
    }
    assert(consecutive >= 3);                          // the strict test really does re-seed back to back
  }

  // ======================================================================================
  // 19. V2.5-Evo - 2026-10-06 - AUDIT H-2: AN S-TURN DURING A REAR-HALF WALK (kinematic simulation)
  // ======================================================================================
  {
    // The rider is foiling at 18 km/h on F4, settled 16 m ahead x 13 m right (d_follow 9 + 7), and
    // switches to mode 2 (behind) while carving S-turns of +/-40 deg every 8 s. The station walks home
    // round the right; the turns sweep his path across the buggy. Every tick: the shield property holds,
    // every re-seed and aim swap carries the D-term step, and the escape never chatters.
    const float vR = 18.0f / 3.6f, dt = 0.1f;
    const float amps[3] = { 25.0f, 40.0f, 60.0f };
    for (int ai = 0; ai < 3; ai++) {
      World w;
      w.rc = 0.0f;
      placeBuggy(w, 16.0f, 13.0f, 0.0f);
      Ctl c;
      {
        float ah, ph, r;
        frontGeom(9.0f, 0, kLateralMin, &ah, &ph, &r);
        c.live = fmStationLimitDeg(ph);                                          // settled F4
      }
      int escapes = 0, edges = 0, short_dwell = 0, last_edge = -100, reseeds = 0, snapped_at = -1;
      bool prev_esc = false;
      float min_dist = 1e9f;
      for (int i = 0; i < 300; i++) {
        w.rc = amps[ai] * sinf(2.0f * 3.14159265f * (float)i * dt / 8.0f);
        if (w.rc < 0.0f) w.rc += 360.0f;
        float al, cr;
        toRiderFrame(w, &al, &cr);
        const Tick t = ctlTick(c, 2, kNearDiag, 9.0f, 0, 4.0f, 18.0f, w.rc, al, cr, 0.0f, dt, kSideBand);
        assert(shieldPropertyHolds(t, kSideBand));
        if (t.h1 || t.sh_reseed) { reseeds++; assert(t.step); }
        if (t.escape) escapes++;
        if (t.escape != prev_esc) {
          if (i - last_edge < 3) short_dwell++;
          last_edge = i; edges++;
          assert(t.step);                                                         // aim swap: D skipped
        }
        prev_esc = t.escape;
        if (t.snapped && snapped_at < 0) snapped_at = i;
        const float d = sqrtf(al * al + cr * cr);
        if (d < min_dist) min_dist = d;
        buggyStep(w, t.aim_al, t.aim_cr, vR * 1.2f, 90.0f, dt);
        riderStep(w, vR, dt);
      }
      printf("S-turn +/-%.0f deg during the walk home: %d escape ticks, %d edges (%d under 3 ticks), "
             "%d re-seeds, snapped home at tick %d, closest %.1f m\n",
             (double)amps[ai], escapes, edges, short_dwell, reseeds, snapped_at, (double)min_dist);
      assert(snapped_at > 0);                                                     // it does get home
      assert(short_dwell <= 2);                                                   // no tick-by-tick chatter
    }

    // AUDIT H-2's THIRD PATH: an M-8 seed, then a turn TOWARD the buggy. Mode 3 engaged with the buggy
    // 10 m ahead / 6 m left, so the station is seeded at -90 (the buggy's side) and walks home on the left.
    // The rider then turns left at 45 deg/s for 2 s - toward the buggy - which swings it to his RIGHT,
    // across his line from the walking station. Before the shield, the aim from there to the left-hand
    // station crossed his line about 7 m ahead of him. Now: re-seeded onto the buggy's side, go-around
    // on that side, and the property holds on every tick.
    {
      const float vR = 18.0f / 3.6f, dt = 0.1f;
      World w;
      w.rc = 0.0f;
      placeBuggy(w, 10.0f, -6.0f, 0.0f);
      Ctl c; c.live = -90.0f;
      int reseeds = 0;
      for (int i = 0; i < 150; i++) {
        const float tt = (float)i * dt;
        w.rc = (tt < 2.0f) ? -45.0f * tt : -90.0f;
        float al, cr;
        toRiderFrame(w, &al, &cr);
        const Tick t = ctlTick(c, 3, kNearDiag, 9.0f, 0, 4.0f, 18.0f, w.rc + 360.0f, al, cr, 0.0f, dt, kSideBand);
        assert(shieldPropertyHolds(t, kSideBand));
        if (t.sh_reseed || t.h1) {
          reseeds++;
          assert(t.step);
          assert((c.live > 0.0f) == (cr > 0.0f));             // onto the side the buggy is on
        }
        buggyStep(w, t.aim_al, t.aim_cr, vR * 1.2f, 90.0f, dt);
        riderStep(w, vR, dt);
      }
      assert(reseeds >= 1);
      printf("M-8 seed then a turn toward the buggy: %d re-seed(s) onto the buggy's side, property held\n", reseeds);
    }

    // A WALK HOME ENDS IN A SNAP ONLY WHEN THE SNAPPED AIM IS CLEAR. Mode 2 walking home from +60 with the
    // buggy held 12 m ahead, 0.5 m off the rider's line (inside the cone): the walk reaches the rear band,
    // but the snap to "directly behind" would aim straight through him, so the station keeps walking under
    // the shield. Once the buggy is behind the rider the very next tick snaps - the SW36 station.
    {
      Ctl c; c.live = +60.0f;
      int blocked = 0;
      for (int i = 0; i < 80; i++) {
        const Tick t = ctlTick(c, 2, kNearDiag, 6.0f, 0, 4.0f, 18.0f, 0.0f, 12.0f, 0.5f, 0.0f, 0.1f, kSideBand);
        assert(!t.snapped);
        assert(shieldPropertyHolds(t, kSideBand));
        if (fabsf(c.live) <= kNearDiag) blocked++;
      }
      assert(blocked > 0);                                      // it really was inside the band, held
      const Tick t = ctlTick(c, 2, kNearDiag, 6.0f, 0, 4.0f, 18.0f, 0.0f, -8.0f, 1.0f, 0.0f, 0.1f, kSideBand);
      assert(t.snapped && c.live == 0.0f && !c.walking);
      // From here it is the SW36 path: the snapped station is never re-tested, wherever the buggy goes.
      const Tick t2 = ctlTick(c, 2, kNearDiag, 6.0f, 0, 4.0f, 18.0f, 0.0f, 12.0f, 0.5f, 0.0f, 0.1f, kSideBand);
      assert(t2.snapped && !t2.escape);
    }

    // Audit H-2's example as one tick: buggy 10 m ahead / 3 m right, station walking at psi -70 (mode 3).
    {
      Ctl c; c.live = -70.0f;
      const Tick t = ctlTick(c, 3, kNearDiag, 9.0f, 0, 4.0f, 18.0f, 0.0f, 10.0f, 3.0f, 0.0f, 0.1f, kSideBand);
      assert(t.escape && t.sh_reseed && t.step);
      assert(c.live > 0.0f);                                   // re-seeded onto the buggy's (right) side
      assert(t.aim_cr > 3.0f);                                 // going round on the right, not across
      assert(shieldPropertyHolds(t, kSideBand));
      // Below 12 km/h the same geometry is the circle only: the buggy may reposition freely.
      Ctl c2; c2.live = -70.0f;
      const Tick t2 = ctlTick(c2, 3, kNearDiag, 9.0f, 0, 4.0f, 10.0f, 0.0f, 10.0f, 3.0f, 0.0f, 0.1f, kSideBand);
      assert(!t2.escape);
    }
  }

  // ======================================================================================
  // 20. V2.5-Evo - 2026-10-06 - AUDIT H-2: F4 ENGAGED WITH THE BUGGY AHEAD-LEFT
  // ======================================================================================
  {
    const float vR = 18.0f / 3.6f, dt = 0.1f;
    const float starts[3][2] = { { 12.0f, -6.0f }, { 12.0f, -9.0f }, { 20.0f, -4.0f } };
    for (int k = 0; k < 3; k++) {
      World w;
      w.rc = 0.0f;
      placeBuggy(w, starts[k][0], starts[k][1], 0.0f);
      Ctl c;
      float al, cr;
      toRiderFrame(w, &al, &cr);
      // The ACTIVE edge: the G-4 seed, the measured angle clamped into the rear half (-90 here).
      c.live = fmEngageSeedDeg(4, true, fmMeasuredStationDeg(0.0f, atan2f(cr, al) / BREMOTE_FMS_DEG2RAD));
      assert(c.live == -90.0f);
      bool latched = false;
      int latched_side = 0;
      float max_live = -1e9f;
      for (int i = 0; i < 200; i++) {
        toRiderFrame(w, &al, &cr);
        const Tick t = ctlTick(c, 4, kNearDiag, 9.0f, 0, 4.0f, 18.0f, 0.0f, al, cr, 0.0f, dt, kSideBand);
        assert(shieldPropertyHolds(t, kSideBand));
        if (t.latched) { latched = true; latched_side = c.rside; }
        if (c.live > max_live) max_live = c.live;
        buggyStep(w, t.aim_al, t.aim_cr, vR * 1.2f, 90.0f, dt);
        riderStep(w, vR, dt);
      }
      printf("F4 engaged, buggy ahead-left (%.0f, %.0f): abort %s%s, station max psi %.1f deg\n",
             (double)starts[k][0], (double)starts[k][1], latched ? "LATCHED" : "not needed",
             latched ? ((latched_side < 0) ? " to the LEFT" : " to the RIGHT") : "", (double)max_live);
      if (k == 0) {
        // Inside the cone at engagement: the walk round the back is refused on the first tick, the abort
        // latches on the buggy's (left) side, and the station never crosses to the right.
        assert(latched && latched_side == -1);
        assert(max_live <= 0.001f);
      }
    }
  }

  // ======================================================================================
  // 21. V2.5-Evo - 2026-10-06 - AUDIT M-11: THE PENDING SEED FROM STANDSTILL
  // ======================================================================================
  {
    // HOLD -> switch to 2 -> squeeze, while stopped: the ACTIVE edge has no course, so live = 0 and the
    // seed is pending. First tick with a course (6 km/h - below the shield's foiling speed):
    {
      Ctl c; c.pending = true;
      const Tick t = ctlTick(c, 2, kNearDiag, 6.0f, 0, 4.0f, 6.0f, 0.0f, 12.0f, 5.0f, 0.0f, 0.1f, kSideBand);
      assert(!c.pending);                                     // consumed on that one tick
      assert(t.step);                                         // the seed moved the aim: D skipped
      assert(c.live > 85.0f && c.live <= 90.0f);              // +90 on the buggy's (right) side, walking home
      assert(!t.snapped);
    }
    {
      Ctl c; c.pending = true;                                // buggy ahead-LEFT: -90
      ctlTick(c, 1, kNearDiag, 6.0f, 0, 4.0f, 6.0f, 0.0f, 10.0f, -4.0f, 0.0f, 0.1f, kSideBand);
      assert(c.live < -85.0f && c.live >= -90.0f);
    }
    {
      // Buggy BEHIND the rider: the seed is 0, exactly where the edge left it - the SW36 snap, no step.
      for (uint8_t m = 1; m <= 3; m++) {
        Ctl c; c.pending = true; c.kind_prev = 0;
        const Tick t = ctlTick(c, m, kNearDiag, 6.0f, 0, 4.0f, 6.0f, 0.0f, -8.0f, 1.0f, 0.0f, 0.1f, kSideBand);
        assert(t.snapped && !t.step);
        assert(c.live == -sw36Offset(m, kNearDiag, c.diag));
      }
    }
    {
      // F4 selected while stopped: the no-course branch has already abandoned it (F4 reads as F1), so
      // the pending seed follows the REAR rule on the effective mode: buggy ahead-left -> -90.
      Ctl c; c.pending = true; c.aborted = true;
      ctlTick(c, 4, kNearDiag, 6.0f, 0, 4.0f, 6.0f, 0.0f, 10.0f, -4.0f, 0.0f, 0.1f, kSideBand);
      assert(c.live < -85.0f);
    }
  }

  // ======================================================================================
  // 22. V2.5-Evo - 2026-10-06 - REAR-ONLY RIDING THROUGH THE NEW CHAIN IS STILL SW36 (37f8b49)
  // ======================================================================================
  {
    // Every rear tick through ctlTick - shield, band, aim kind and all - snaps to the SW36 offset, never
    // escapes, never sets the aim step, at foiling speed, at random buggy positions, any near_diag.
    Ctl c;
    unsigned seed = 99u;
    long n = 0;
    for (int i = 0; i < 100000; i++) {
      seed = seed * 1103515245u + 12345u;
      const uint8_t m  = (uint8_t)(1 + (seed >> 16) % 3);
      const float   nd = (float)((seed >> 8) % 91);
      seed = seed * 1103515245u + 12345u;
      const float al = ((float)((seed >> 8) % 4001u) / 100.0f) - 30.0f;          // -30 .. +10 m
      seed = seed * 1103515245u + 12345u;
      const float cr = ((float)((seed >> 8) % 4001u) / 100.0f) - 20.0f;          // -20 .. +20 m
      if (fabsf(c.live) > nd) c.live = 0.0f;   // only states rear-only riding can leave (see section 13)
      const Tick t = ctlTick(c, m, nd, 6.0f, 0, 4.0f, 18.0f, 0.0f, al, cr, 0.0f, 0.1f, kSideBand);
      assert(t.snapped && !t.escape && !t.h1 && !t.sh_reseed && t.kind == 0 && !t.step);
      assert(c.live == -sw36Offset(m, nd, c.diag));
      n++;
    }
    printf("rear-only through the shield chain, SW36 equivalence: %ld ticks\n", n);
  }

  // ======================================================================================
  // 23. V2.5-Evo - 2026-10-06 - AUDIT M-16 (b): THE SPEED-SCALED CONE HALF-ANGLE
  // ======================================================================================
  {
    // The anchors asked for, exactly.
    assert(fmShieldHalfAngleDeg(12.0f) == 30.0f && fmShieldHalfAngleDeg(15.0f) == 30.0f);
    assert(near_eq(fmShieldHalfAngleDeg(17.5f), 28.0f, 1e-4f));
    assert(near_eq(fmShieldHalfAngleDeg(22.5f), 21.0f, 1e-4f));
    assert(near_eq(fmShieldHalfAngleDeg(27.5f), 15.0f, 1e-4f));
    assert(fmShieldHalfAngleDeg(40.0f) == 15.0f && fmShieldHalfAngleDeg(0.0f) == 30.0f);
    assert(fmShieldHalfAngleDeg(-5.0f) == 30.0f && fmShieldHalfAngleDeg(NAN) == 30.0f);
    // Continuous, monotone, inside [15, 30], and NEVER NARROWER THAN THE DATA anywhere in a band (the
    // owner's 30 deg cap stands in for the 40 deg the 10-15 km/h band measured).
    float prev = fmShieldHalfAngleDeg(0.0f);
    long n = 0;
    for (int i = 0; i <= 6000; i++) {
      const float v = 0.01f * (float)i;
      const float h = fmShieldHalfAngleDeg(v);
      assert(h <= kFmShieldHalfAngleMaxDeg && h >= 15.0f);
      assert(h <= prev + 1e-4f);                                 // never widens with speed
      assert(fabsf(h - prev) <= 0.01f * (7.0f / 2.5f) + 1e-3f);  // no step: at most the steepest knot slope
      float data;                                                // the band's p90 angle at 5 s
      if (v < 10.0f)       data = 0.0f;                          // not foiling: no data, no cone
      else if (v < 15.0f)  data = 30.0f;                         // 40 measured, capped at 30 (owner ruling)
      else if (v <= 20.0f) data = 28.0f;
      else if (v <= 25.0f) data = 21.0f;
      else                 data = 15.0f;
      assert(h >= data - 1e-4f);
      prev = h;
      n++;
    }
    printf("M-16 b half-angle curve: %ld speeds, 30/28/21/15 deg held across each band\n", n);
  }

  // ======================================================================================
  // 24. V2.5-Evo - 2026-10-06 - AUDIT M-15: THE CONE GATE IS A SCHMITT (on at 12, off below 10)
  // ======================================================================================
  {
    assert(!fmShieldConeOn(false, 11.99f, kShMinKmh, kShOffKmh));
    assert( fmShieldConeOn(false, 12.0f,  kShMinKmh, kShOffKmh));
    assert( fmShieldConeOn(true,  10.0f,  kShMinKmh, kShOffKmh));   // on stays on down to 10
    assert(!fmShieldConeOn(true,   9.99f, kShMinKmh, kShOffKmh));
    assert(!fmShieldConeOn(false, 11.0f,  kShMinKmh, kShOffKmh));   // off stays off up to 12
    // The rider hovering at the takeoff edge: 11.5 +/- 1.2 km/h of noise. A single threshold flicks the cone
    // tens of times; the Schmitt only once on, and never off while he stays above 10.
    unsigned seed = 31u;
    bool sch = false, single = false;
    int t_sch = 0, t_single = 0;
    for (int i = 0; i < 600; i++) {
      seed = seed * 1103515245u + 12345u;
      const float v = 11.5f + (((float)((seed >> 8) % 2401u) / 1000.0f) - 1.2f);
      const bool s2 = fmShieldConeOn(sch, v, kShMinKmh, kShOffKmh);
      const bool s1 = (v >= kShMinKmh);
      if (s2 != sch) t_sch++;
      if (s1 != single) t_single++;
      sch = s2; single = s1;
    }
    printf("M-15 hovering at 11.5 +/- 1.2 km/h: cone toggles %d with the Schmitt, %d with one threshold\n",
           t_sch, t_single);
    assert(t_sch == 1 && t_single >= 20);
  }

  // ======================================================================================
  // 25. V2.5-Evo - 2026-10-06 - AUDIT M-16 (c): THE HOLD BAND, AND A SETTLED F4/F5 STATION CLEARS THE
  //     HELD CONE AT EVERY CONFIG
  // ======================================================================================
  {
    // The formula: min(2 m, 0.5 r sin(phi - half)), floored at 0.
    assert(near_eq(fmShieldHoldBandM(22.67f, 35.0f, 30.0f, kSideBand), 0.5f * 22.67f * sinf(5.0f * BREMOTE_FMS_DEG2RAD), 1e-4f));
    assert(fmShieldHoldBandM(18.38f, 45.0f, 15.0f, kSideBand) == kSideBand);          // capped at 2 m
    assert(fmShieldHoldBandM(18.38f, 30.0f, 30.0f, kSideBand) == 0.0f);               // no clearance: no hold
    assert(fmShieldHoldBandM(18.38f, 25.0f, 30.0f, kSideBand) == 0.0f);               // floored at 0
    assert(fmShieldHoldBandM(18.38f, 45.0f, 30.0f, 0.0f) == 0.0f);

    // THE SWEEP. The RX's own geometry chain for every config: min_dist_m and the smoothing band (so
    // d_follow AND the side floor move together, as they do on the board), every stored extra 0-10, every
    // steering preset (the lag shortfall the capsule adds), and the rider at 10-35 km/h WITH THE CONE ON
    // (12-35 asked; 10-12 is where the Schmitt keeps it on). The settled station - F4 and F5 at their
    // presets, at the radius the schedule gives them - must be OUTSIDE the shield with the cone widened
    // by the hold band. A failing config is printed before the assert fires.
    const float taus[5] = { 5.0f, 3.0f, 2.0f, 1.0f, 0.5f };        // kSteerPresets[].target_filter_tau_s
    const float mds[11] = { 1.0f, 2.0f, 3.0f, 4.0f, 6.0f, 8.0f, 10.1f, 12.0f, 13.0f, 15.0f, 20.0f };
    const float sbs[7]  = { 0.0f, 1.0f, 2.0f, 3.0f, 5.0f, 8.0f, 12.0f };
    // The cone and the circle/capsule are judged SEPARATELY, because they answer different findings: the
    // held CONE is M-16's requirement and must clear at every config (asserted). The CAPSULE (M-18) is
    // reported: when min_dist_m >= 13 the side floor IS min_dist_m, which is also the circle radius, so a
    // capsule reaching level with the station puts the station exactly on its edge (zero margin). Those
    // configs are counted, characterised and printed - not hidden - and the characterisation is asserted.
    long cfgs = 0, cone_fails = 0, cap_contacts = 0;
    float worst = 1e9f, worst_phi = 0, worst_h = 0, worst_d = 0, worst_v = 0;
    float cap_md_min = 1e9f, cap_tau_min = 1e9f, cap_v_min = 1e9f, cap_clear_min = 1e9f;
    for (int mi = 0; mi < 11; mi++) for (int bi = 0; bi < 7; bi++) for (int ex = 0; ex <= 10; ex++) {
      const float md = mds[mi], d = md + sbs[bi];
      float pass = kPassLateral; if (pass < md) pass = md;
      float latmin = kLateralMin; if (latmin < pass) latmin = pass;
      float ah, phi, rf;
      frontGeom(d, (uint16_t)ex, latmin, &ah, &phi, &rf);
      const float lim = fmStationLimitDeg(phi);
      for (int vi = 0; vi <= 100; vi++) {
        const float v    = 10.0f + 0.25f * (float)vi;
        const float h    = fmShieldHalfAngleDeg(v);
        const float hold = fmShieldHoldBandM(rf, phi, h, kSideBand);
        for (int ti = 0; ti < 5; ti++) {
          const float raw = (v / 3.6f) * taus[ti];
          float lag = raw; if (lag > 2.0f * d) lag = 2.0f * d;
          FmShield sh = fmShieldMake(v, kShHorizonS, h, kShOffKmh, kShCircleM, md, d);
          fmShieldExtendForLag(sh, raw - lag);
          FmShield cone_only = sh;  cone_only.circle_r_m = 0.0f;  cone_only.circle_len_m = 0.0f;
          FmShield circ_only = sh;  circ_only.cone_len_m = 0.0f;
          for (int side = -1; side <= 1; side += 2) {
            const float psi = side * lim;
            const float rr  = fmStationRadiusM(psi, d, rf, kNearDiag, pass, phi);
            float sa, sc;
            fmStationAlongCrossM(psi, rr, &sa, &sc);
            const float margin = rr * sinf((phi - h) * BREMOTE_FMS_DEG2RAD) - hold;   // to the held cone's side
            if (fmShieldSegmentHits(cone_only, sa, sc, sa, sc, hold) || !(margin > 0.0f)) {
              if (cone_fails < 10) printf("  M-16 HELD-CONE FAIL: md %.1f d %.1f extra %d v %.2f tau %.1f: phi %.2f h %.2f hold %.2f margin %.3f\n",
                                          (double)md, (double)d, ex, (double)v, (double)taus[ti], (double)phi, (double)h,
                                          (double)hold, (double)margin);
              cone_fails++;
            }
            if (fmShieldSegmentHits(circ_only, sa, sc, sa, sc, 0.0f)) {
              cap_contacts++;
              if (md < cap_md_min) cap_md_min = md;
              if (taus[ti] < cap_tau_min) cap_tau_min = taus[ti];
              if (v < cap_v_min) cap_v_min = v;
              const float L = sh.circle_len_m;
              const float cx = (sa < 0.0f) ? sa : ((sa > L) ? sa - L : 0.0f);
              const float clear = sqrtf(cx * cx + sc * sc) - sh.circle_r_m;
              if (clear < cap_clear_min) cap_clear_min = clear;
            }
            if (margin < worst) { worst = margin; worst_phi = phi; worst_h = h; worst_d = d; worst_v = v; }
            cfgs++;
          }
        }
      }
    }
    printf("M-16 settled F4/F5 vs the HELD cone: %ld configs, %ld fail, tightest margin %.2f m "
           "(phi %.1f, half-angle %.1f, d_follow %.1f, %.2f km/h)\n",
           cfgs, cone_fails, (double)worst, (double)worst_phi, (double)worst_h, (double)worst_d, (double)worst_v);
    printf("M-18 REPORT - settled F4/F5 touching the lag CAPSULE: %ld of %ld configs; all have min_dist >= %.1f m, "
           "tau >= %.1f s, rider >= %.2f km/h; worst clearance %.3f m (the station on the edge)\n",
           cap_contacts, cfgs, (double)cap_md_min, (double)cap_tau_min, (double)cap_v_min, (double)cap_clear_min);
    assert(cone_fails == 0);
    // The capsule contacts are exactly the case named above, and nothing else: min_dist >= 13 (side floor ==
    // circle radius), the two soft presets only, and the station ON the edge (zero clearance, not inside).
    if (cap_contacts > 0) {
      assert(cap_md_min >= kLateralMin - 0.001f);
      assert(cap_tau_min >= 3.0f);
      assert(cap_clear_min > -0.01f);
    }

    // THE TABLE for the report: min_dist 4, smoothing 2 / 5 / 8 (d_follow 6 / 9 / 12), stored extra
    // 0 (= the 7 m default) / 4 / 7 / 10. clear = r sin(phi - h) to the cone's side; margin = clear - hold.
    printf("M-16 settled-station clearance table (side 13 m; h = half-angle, all metres):\n");
    printf("   d  extra ahead   phi  |  15 km/h:  h  clear hold margin |  20 km/h:  h  clear hold margin |  25 km/h:  h  clear hold margin\n");
    const float dts[3] = { 6.0f, 9.0f, 12.0f };
    const int   exs[4] = { 0, 4, 7, 10 };
    for (int di = 0; di < 3; di++) for (int ei = 0; ei < 4; ei++) {
      float ah, phi, rf;
      frontGeom(dts[di], (uint16_t)exs[ei], kLateralMin, &ah, &phi, &rf);
      printf("  %2.0f  %2d%s %5.2f %5.2f |", (double)dts[di], exs[ei], exs[ei] == 0 ? "=7" : "  ",
             (double)ah, (double)phi);
      const float vs[3] = { 15.0f, 20.0f, 25.0f };
      for (int vi = 0; vi < 3; vi++) {
        const float h = fmShieldHalfAngleDeg(vs[vi]);
        const float hold = fmShieldHoldBandM(rf, phi, h, kSideBand);
        const float clear = rf * sinf((phi - h) * BREMOTE_FMS_DEG2RAD);
        printf("          %4.1f %5.2f %4.2f %5.2f  |", (double)h, (double)clear, (double)hold, (double)(clear - hold));
        assert(clear - hold > 0.0f);
      }
      printf("\n");
    }
  }

  // ======================================================================================
  // 26. V2.5-Evo - 2026-10-06 - AUDIT M-18: THE CAPSULE (the lag anchor capped short of the rider)
  // ======================================================================================
  {
    // Very Soft (tau 5 s), d_follow 6 (lag cap 12 m), 11 km/h: the real rider is 15.3 m beyond the filtered
    // position but the anchor only 12 m - 3.3 m short. A line crossing his course 2 m in front of HIM is
    // 5.3 m from the anchor: the plain 4 m circle missed it, the capsule does not.
    const float v = 11.0f, tau = 5.0f, d = 6.0f;
    const float raw = (v / 3.6f) * tau, lag = 2.0f * d, sf = raw - lag;
    assert(near_eq(sf, 3.28f, 0.01f));
    FmShield plain = fmShieldMake(v, kShHorizonS, 30.0f, kShMinKmh, kShCircleM, 4.0f, d);
    FmShield cap = plain;
    fmShieldExtendForLag(cap, sf);
    assert(near_eq(cap.circle_len_m, sf, 1e-5f) && cap.cone_len_m == 0.0f);   // cone still off at 11
    assert(!fmShieldSegmentHits(plain, sf + 2.0f, -8.0f, sf + 2.0f, 8.0f, 0.0f));
    assert( fmShieldSegmentHits(cap,   sf + 2.0f, -8.0f, sf + 2.0f, 8.0f, 0.0f));
    // The cone is deepened by the same amount, so its reach past the real rider is speed x horizon.
    FmShield f = fmShieldMake(18.0f, kShHorizonS, 28.0f, kShMinKmh, kShCircleM, 4.0f, d);
    const float len0 = f.cone_len_m;
    fmShieldExtendForLag(f, 7.0f);
    assert(near_eq(f.cone_len_m, len0 + 7.0f, 1e-4f) && f.circle_len_m == 7.0f);
    // No shortfall: nothing changes (bitwise), which is the Normal preset at ordinary speeds.
    FmShield g = fmShieldMake(18.0f, kShHorizonS, 28.0f, kShMinKmh, kShCircleM, 4.0f, d);
    fmShieldExtendForLag(g, 0.0f);
    fmShieldExtendForLag(g, -3.0f);
    assert(g.circle_len_m == 0.0f && g.cone_len_m == len0);
    // The rider's own rear station (d_follow behind the anchor) is still reachable: the capsule only grows ahead.
    assert(!fmShieldSegmentHits(cap, -d - 6.0f, 0.0f, -d, 0.0f, 0.0f));
    // Half-width: R all along the core, rounding off past both ends.
    assert(near_eq(fmShieldHalfWidthM(cap, 1.5f), 4.0f, 1e-4f));
    assert(near_eq(fmShieldHalfWidthM(cap, sf + 1.0f), sqrtf(15.0f), 1e-3f));
    assert(near_eq(fmShieldHalfWidthM(cap, -1.0f), sqrtf(15.0f), 1e-3f));

    // The capsule segment test against brute-force sampling, 20,000 random segments x 4 shields, and the
    // go-around proof on each (its cross-section is still one interval).
    {
      const float sfs[4] = { 0.5f, 3.3f, 8.0f, 20.0f };
      const float vs[4]  = { 8.0f, 11.0f, 18.0f, 30.0f };
      unsigned seed = 5151u;
      long checked = 0, ga = 0;
      for (int si = 0; si < 4; si++) {
        FmShield sh = fmShieldMake(vs[si], kShHorizonS, fmShieldHalfAngleDeg(vs[si]), kShMinKmh, kShCircleM, 4.0f, 100.0f);
        fmShieldExtendForLag(sh, sfs[si]);
        const float L = sh.circle_len_m;
        for (int k = 0; k < 20000; k++) {
          float p[4];
          for (int j = 0; j < 4; j++) {
            seed = seed * 1103515245u + 12345u;
            p[j] = ((float)((seed >> 8) % 8001u) / 100.0f) - 40.0f;
          }
          bool sampled = false, near = false;
          for (int nn = 0; nn <= 400; nn++) {
            const float u = (float)nn / 400.0f;
            const float x = p[0] + u * (p[2] - p[0]), y = p[1] + u * (p[3] - p[1]);
            const float cx = (x < 0.0f) ? x : ((x > L) ? x - L : 0.0f);
            const bool inC = (cx * cx + y * y) < sh.circle_r_m * sh.circle_r_m;
            const bool inT = sh.cone_len_m > 0.0f && x >= 0.0f && x <= sh.cone_len_m && fabsf(y) <= x * sh.cone_tan;
            if (inC || inT) sampled = true;
            const float rc = sh.circle_r_m + 0.3f;
            const bool nC = (cx * cx + y * y) <= rc * rc;
            const bool nT = sh.cone_len_m > 0.0f && x >= -0.6f && x <= sh.cone_len_m + 0.3f &&
                            fabsf(y) <= (x + 0.6f) * sh.cone_tan;
            if (nC || nT) near = true;
          }
          const bool hit = fmShieldSegmentHits(sh, p[0], p[1], p[2], p[3], 0.0f);
          if (sampled) assert(hit);
          if (hit)     assert(near);
          checked++;
        }
        for (int ai = -30; ai <= 50; ai++) {
          for (int ci = -30; ci <= 30; ci++) {
            const float a = (float)ai, c = (float)ci;
            if (fabsf(c) <= kSideBand) continue;
            if (fmShieldSegmentHits(sh, a, c, a, c, 0.0f)) continue;
            const int side = (c > 0.0f) ? +1 : -1;
            const float lat = fmShieldGoAroundLateralM(sh, side, a, c, kPassLateral, 6.0f);
            assert(!fmShieldSegmentHits(sh, a, c, a, lat, 0.0f));
            ga++;
          }
        }
      }
      printf("M-18 capsule segment test vs sampling: %ld segments; go-around proof: %ld buggy positions\n", checked, ga);
    }
  }

  // ======================================================================================
  // 27. V2.5-Evo - 2026-10-06 - AUDIT M-14: THE CLIP AFTER A RELEASE, AND A CONE TOGGLE, ARE AIM STEPS
  // ======================================================================================
  {
    // The owner's F4 (d_follow 9 + 7: 16 m ahead x 13 m side, 39.1 deg) settled at 18 km/h (half-angle 28):
    // the 9 m lookahead (25 x 13 m, 27.5 deg) is clipped. A buggy swung inside the cone starts an escape;
    // it then returns to the station and the escape ends. On the release tick the clip still runs against
    // the HELD cone; the tick after, against the plain one - the lookahead jumps. Both ticks skip the D term.
    float ah, ph, rf;
    frontGeom(9.0f, 0, kLateralMin, &ah, &ph, &rf);
    const float lim = fmStationLimitDeg(ph);
    float sa, sc;
    fmStationAlongCrossM(lim, rf, &sa, &sc);
    Ctl c; c.live = lim; c.cone_on = true;
    Tick t = ctlTick(c, 4, kNearDiag, 9.0f, 0, 4.0f, 18.0f, 0.0f, sa, sc, 0.0f, 0.1f, kSideBand);
    assert(!t.escape && t.kind == 0);
    const float look_plain = t.aim_al - sa;
    assert(look_plain > 4.0f && look_plain < 9.0f);                  // clipped, not zero
    t = ctlTick(c, 4, kNearDiag, 9.0f, 0, 4.0f, 18.0f, 0.0f, 20.0f, 6.0f, 0.0f, 0.1f, kSideBand);
    assert(t.escape && t.step);                                     // inside the cone: escape, aim swap
    t = ctlTick(c, 4, kNearDiag, 9.0f, 0, 4.0f, 18.0f, 0.0f, sa, sc, 0.0f, 0.1f, kSideBand);
    assert(!t.escape && t.step);                                    // release: kind change
    const float look_held = t.aim_al - sa;
    assert(look_held < look_plain - 0.5f);                          // clipped against the held cone
    t = ctlTick(c, 4, kNearDiag, 9.0f, 0, 4.0f, 18.0f, 0.0f, sa, sc, 0.0f, 0.1f, kSideBand);
    assert(!t.escape && near_eq(t.aim_al - sa, look_plain, 0.02f)); // the jump ...
    assert(t.step);                                                 // ... is flagged (M-14)
    t = ctlTick(c, 4, kNearDiag, 9.0f, 0, 4.0f, 18.0f, 0.0f, sa, sc, 0.0f, 0.1f, kSideBand);
    assert(!t.step);                                                // and steady state is not
    printf("M-14 clip after a release: %.2f m held -> %.2f m plain, both ticks flagged\n",
           (double)look_held, (double)look_plain);
    // The cone switching on under a settled station: the clip goes from the full 9 m to clipped - flagged.
    Ctl c2; c2.live = lim;
    t = ctlTick(c2, 4, kNearDiag, 9.0f, 0, 4.0f, 9.0f, 0.0f, sa, sc, 0.0f, 0.1f, kSideBand);
    assert(!c2.cone_on && near_eq(t.aim_al - sa, 9.0f, 0.01f));
    t = ctlTick(c2, 4, kNearDiag, 9.0f, 0, 4.0f, 9.0f, 0.0f, sa, sc, 0.0f, 0.1f, kSideBand);
    assert(!t.step);
    t = ctlTick(c2, 4, kNearDiag, 9.0f, 0, 4.0f, 18.0f, 0.0f, sa, sc, 0.0f, 0.1f, kSideBand);
    assert(c2.cone_on && t.cone_toggled && t.step && t.aim_al - sa < 8.9f);
  }

  // ======================================================================================
  // 28. V2.5-Evo - 2026-10-06 - AUDITS M-15 / M-14 / M-18 / L-20: A 12 km/h CROSSING SWEEP WITH LAG AND
  //     VARYING SPEED (kinematic simulation)
  // ======================================================================================
  {
    // The rider's speed swings 9-15 km/h (a 6 s cycle plus +/-0.7 km/h noise, low-passed as the RX's speed is -
    // it comes off the EMA-filtered track, never a raw per-fix value) with a +/-8 deg wobble, at the
    // Very Soft preset (tau 5 s, so the lag push hits its 2 x d_follow cap and the capsule is in play), and
    // the buggy runs two jobs: the F4 transit from behind-right, and mode 2 walking home from a settled F4.
    // Every tick: the shield property holds; every cone toggle on a non-snapped station carries the aim
    // step; and the aim never jumps more than 2 m in the world without the step.
    const float dt = 0.1f, tau = 5.0f, d = 9.0f;
    for (int job = 0; job < 2; job++) {
      World w;
      w.rc = 0.0f;
      float ah, ph, rf;
      frontGeom(d, 0, kLateralMin, &ah, &ph, &rf);
      Ctl c;
      uint8_t mode;
      if (job == 0) { placeBuggy(w, -9.0f, 4.0f, 0.0f); c.live = 25.0f; mode = 4; }
      else          { placeBuggy(w, ah, 13.0f, 0.0f);   c.live = fmStationLimitDeg(ph); mode = 2; }
      unsigned seed = 2026u + (unsigned)job;
      int toggles = 0, unflagged_jumps = 0, escapes = 0;
      float px = 0, py = 0, max_sf = 0.0f, max_jump = 0.0f, v_filt = 9.0f;
      bool have_prev = false;
      for (int i = 0; i < 400; i++) {
        seed = seed * 1103515245u + 12345u;
        const float noise = ((float)((seed >> 8) % 1401u) / 1000.0f) - 0.7f;
        v_filt += 0.15f * ((12.0f + 3.0f * sinf(2.0f * 3.14159265f * (float)i * dt / 6.0f) + noise) - v_filt);
        const float v = v_filt;
        w.rc = 8.0f * sinf(2.0f * 3.14159265f * (float)i * dt / 9.0f);
        if (w.rc < 0.0f) w.rc += 360.0f;
        const float raw = (v / 3.6f) * tau;
        const float lag = (raw > 2.0f * d) ? 2.0f * d : raw;
        c.shortfall = raw - lag;
        if (c.shortfall > max_sf) max_sf = c.shortfall;
        float al, cr;
        toRiderFrame(w, &al, &cr);
        const Tick t = ctlTick(c, mode, kNearDiag, d, 0, 4.0f, v, w.rc, al, cr, lag, dt, kSideBand);
        assert(shieldPropertyHolds(t, kSideBand));
        if (t.cone_toggled) { toggles++; if (!t.snapped) assert(t.step); }
        if (t.escape) escapes++;
        // the aim in the world (anchor frame -> world: the anchor is lag ahead of the filtered rider)
        const float cr_ = w.rc * BREMOTE_FMS_DEG2RAD;
        const float ax = w.rx + (t.aim_al + lag) * sinf(cr_) + t.aim_cr * cosf(cr_);
        const float ay = w.ry + (t.aim_al + lag) * cosf(cr_) - t.aim_cr * sinf(cr_);
        if (have_prev && !t.step && !t.snapped) {
          const float jmp = sqrtf((ax - px) * (ax - px) + (ay - py) * (ay - py));
          if (jmp > max_jump) max_jump = jmp;
          if (jmp > 2.0f) unflagged_jumps++;
        }
        px = ax; py = ay; have_prev = true;
        buggyStep(w, t.aim_al + lag, t.aim_cr, (v / 3.6f) * 1.2f, 90.0f, dt);
        riderStep(w, v / 3.6f, dt);
      }
      printf("12 km/h crossing sweep, %s: %d cone toggles (Schmitt), %d escape ticks, lag shortfall up to %.1f m, "
             "%d unflagged aim jumps > 2 m (largest unflagged move %.2f m)\n",
             job == 0 ? "F4 transit" : "mode 2 walk home from F4",
             toggles, escapes, (double)max_sf, unflagged_jumps, (double)max_jump);
      assert(unflagged_jumps == 0);
      assert(toggles <= 14);                       // about one on and one off per 6 s cycle, not per tick
    }
  }

  // ======================================================================================
  // 29. V2.5-Evo - 2026-10-06 - THE 3 s SETTLED-ESCAPE LATCH (owner-approved rule), AT PHI 35
  // ======================================================================================
  {
    // The timer itself.
    uint32_t since = 0;
    assert(!fmShieldSettledEscapeExpired(true, 1000, &since, 3000) && since == 1000);
    assert(!fmShieldSettledEscapeExpired(true, 4000, &since, 3000));            // exactly 3 s: not MORE than
    assert( fmShieldSettledEscapeExpired(true, 4001, &since, 3000) && since == 0);
    since = 0;
    assert(!fmShieldSettledEscapeExpired(true, 0, &since, 3000) && since == 1);  // millis 0 is not the sentinel
    assert(!fmShieldSettledEscapeExpired(false, 2000, &since, 3000) && since == 0);   // a break resets it
    since = 0xFFFFFC00u;                                                          // across the millis() wrap:
    assert(!fmShieldSettledEscapeExpired(true, 0x00000100u, &since, 3000));      // 1,280 ms in
    assert( fmShieldSettledEscapeExpired(true, 0x00000C00u, &since, 3000));      // 4,096 ms in

    // F4 settled at the 35 deg cap (d_follow 9 + extra 10: 18.6 m ahead x 13 m side, r 22.7 m), rider at
    // 16 km/h (half-angle 29.2, cone 22.2 m). The buggy, a little wide of its station (24 m out), sees his
    // course estimate swing 10 deg toward it: it now sits 25 deg off his course, inside the cone, while still
    // 10.1 m off his line (so PG-2 keeps the station where it is - a buggy NOT that wide is pulled back to abeam,
    // re-enters transit, and the old outbound rule latches within a few ticks). The go-around starts on a
    // SETTLED station, so nothing latches at once ...
    float ah, ph, rf;
    frontGeom(9.0f, 10, kLateralMin, &ah, &ph, &rf);
    assert(near_eq(ph, 35.0f, 0.01f));
    const float lim = fmStationLimitDeg(ph);
    const float b_al = 24.0f * cosf(25.0f * BREMOTE_FMS_DEG2RAD), b_cr = 24.0f * sinf(25.0f * BREMOTE_FMS_DEG2RAD);
    assert(b_cr >= kPassLateral);
    float s_al, s_cr;
    fmStationAlongCrossM(lim, rf, &s_al, &s_cr);
    {
      Ctl c; c.live = lim; c.cone_on = true;
      int latched_at = -1;
      for (int i = 1; i <= 40 && latched_at < 0; i++) {
        const Tick t = ctlTick(c, 4, kNearDiag, 9.0f, 10, 4.0f, 16.0f, 0.0f, b_al, b_cr, 0.0f, 0.1f, kSideBand);
        assert(t.escape);
        assert(shieldPropertyHolds(t, kSideBand));
        if (t.settled_latched) latched_at = i;
      }
      // ... until it has stood for MORE than 3 s: tick 1 starts the run (100 ms), tick 32 is 3.1 s later.
      printf("settled-escape latch at phi 35: latched on tick %d (%.1f s), side %s\n", latched_at,
             (double)((latched_at - 1) * 0.1f), (c.rside > 0) ? "RIGHT" : "LEFT");
      assert(latched_at == 32);
      assert(c.aborted && c.rside == +1);                         // the buggy's (right) side
      // From here it is F1's rear station, walked home on the right; the property holds all the way.
      for (int i = 0; i < 120; i++) {
        const Tick t = ctlTick(c, 4, kNearDiag, 9.0f, 10, 4.0f, 16.0f, 0.0f, -6.0f, 9.0f, 0.0f, 0.1f, kSideBand);
        assert(shieldPropertyHolds(t, kSideBand));
        assert(c.live >= -0.001f);                                // never crosses to the left
      }
    }
    // A wobble SHORTER than 3 s never latches: 2.5 s in the cone, back on station (the escape clears the HELD
    // cone - the M-16 c guarantee), 2.5 s in again. Two runs, neither long enough.
    {
      Ctl c; c.live = lim; c.cone_on = true;
      bool latched = false, cleared = false;
      for (int i = 0; i < 25; i++)
        latched |= ctlTick(c, 4, kNearDiag, 9.0f, 10, 4.0f, 16.0f, 0.0f, b_al, b_cr, 0.0f, 0.1f, kSideBand).settled_latched;
      for (int i = 0; i < 5; i++) {
        const Tick t = ctlTick(c, 4, kNearDiag, 9.0f, 10, 4.0f, 16.0f, 0.0f, s_al, s_cr, 0.0f, 0.1f, kSideBand);
        if (!t.escape) cleared = true;
      }
      for (int i = 0; i < 25; i++)
        latched |= ctlTick(c, 4, kNearDiag, 9.0f, 10, 4.0f, 16.0f, 0.0f, b_al, b_cr, 0.0f, 0.1f, kSideBand).settled_latched;
      assert(cleared && !latched && !c.aborted);
    }
  }

  // ======================================================================================
  // 30. V2.5-Evo - 2026-10-06 - AUDIT M-13: A GO-AROUND THAT CHANGES SIDES AT PSI 0 IS AN AIM STEP
  // ======================================================================================
  {
    // F5 selected with the station walking through directly-behind (psi 0, dt 0 holds it there) and the buggy
    // 10 m ahead in the cone, inside the 2 m band. The escape starts on the buggy's sign (left) and the outbound
    // F5 latches (the old rule, unchanged). The station is still exactly behind, so it has no side.
    Ctl c; c.live = 0.0f; c.cone_on = true;
    Tick t = ctlTick(c, 5, kNearDiag, 6.0f, 0, 4.0f, 18.0f, 0.0f, 10.0f, -1.5f, 0.0f, 0.0f, kSideBand);
    assert(t.escape && c.shside == -1 && c.live == 0.0f);
    t = ctlTick(c, 5, kNearDiag, 6.0f, 0, 4.0f, 18.0f, 0.0f, 10.0f, -1.5f, 0.0f, 0.0f, kSideBand);
    assert(t.escape && t.step);                                   // the hold switched on (M-14 input edge)
    t = ctlTick(c, 5, kNearDiag, 6.0f, 0, 4.0f, 18.0f, 0.0f, 10.0f, -1.5f, 0.0f, 0.0f, kSideBand);
    assert(t.escape && !t.step);                                  // steady: no step
    // The buggy is pushed 2.5 m RIGHT: beyond the band the go-around follows it - and the waypoint jumps
    // across his line. No re-seed (psi 0), same kind, same label: only M-13's flag can tell the D term.
    t = ctlTick(c, 5, kNearDiag, 6.0f, 0, 4.0f, 18.0f, 0.0f, 10.0f, +2.5f, 0.0f, 0.0f, kSideBand);
    assert(t.escape && c.shside == +1 && !t.sh_reseed && t.kind == 2 && c.live == 0.0f);
    assert(t.step);
    assert(shieldPropertyHolds(t, kSideBand));
    t = ctlTick(c, 5, kNearDiag, 6.0f, 0, 4.0f, 18.0f, 0.0f, 10.0f, +2.5f, 0.0f, 0.0f, kSideBand);
    assert(t.escape && !t.step);
    printf("M-13 go-around side flip at psi 0: flagged, no re-seed\n");
  }

  // ======================================================================================
  // 31. V2.5-Evo - 2026-10-06 - AUDIT M-17: near_diag 90 - THE SEED'S FIRST SNAP ANSWERS TO THE SHIELD
  // ======================================================================================
  {
    // Mode 2 engaged (HOLD -> switch -> squeeze) with the buggy 12 m ahead, 3 m right of the rider's line,
    // near_diag 90. The M-8 seed is +90 - already inside the rear band at nd 90 - so without M-17 the first
    // tick snapped to 0 (directly behind), aiming the buggy straight back through the rider.
    const float meas = fmMeasuredStationDeg(0.0f, atan2f(3.0f, 12.0f) / BREMOTE_FMS_DEG2RAD);
    for (int pend = 0; pend <= 1; pend++) {
      Ctl c;
      engageEdge(c, 2, pend == 0, meas);
      if (pend == 0) assert(c.live == 90.0f && c.walking);       // M-17: off 0 = walking
      else           assert(c.live == 0.0f && c.pending && !c.walking);
      const Tick t = ctlTick(c, 2, 90.0f, 6.0f, 0, 4.0f, 18.0f, 0.0f, 12.0f, 3.0f, 0.0f, 0.1f, kSideBand);
      assert(!t.snapped);                                         // the snap through the rider is refused
      assert(shieldPropertyHolds(t, kSideBand));
      // The pre-M-17 behaviour, for the record: walking false, the same tick snaps.
      Ctl o;
      o.live = 90.0f; o.walking = false;
      const Tick to = ctlTick(o, 2, 90.0f, 6.0f, 0, 4.0f, 18.0f, 0.0f, 12.0f, 3.0f, 0.0f, 0.1f, kSideBand);
      assert(to.snapped && o.live == 0.0f);
      float a, cc;
      fmStationAlongCrossM(0.0f, 6.0f, &a, &cc);
      assert(fmShieldSegmentHits(to.sh, to.b_al, to.b_cr, a, cc, 0.0f));   // ... and that line hit the shield
    }
    // Once the buggy is behind him, the next tick snaps - the SW36 station.
    Ctl c;
    engageEdge(c, 2, true, meas);
    ctlTick(c, 2, 90.0f, 6.0f, 0, 4.0f, 18.0f, 0.0f, 12.0f, 3.0f, 0.0f, 0.1f, kSideBand);
    const Tick t = ctlTick(c, 2, 90.0f, 6.0f, 0, 4.0f, 18.0f, 0.0f, -8.0f, 1.0f, 0.0f, 0.1f, kSideBand);
    assert(t.snapped && c.live == 0.0f);
    // A seed of 0 is not walking: rear-only engagements are exactly SW36.
    Ctl z;
    engageEdge(z, 1, true, 20.0f);
    assert(z.live == 0.0f && !z.walking);
    printf("M-17 near_diag 90: seed +90 walks (shield-checked) instead of snapping through the rider\n");
  }

  // ======================================================================================
  // 32. V2.5-Evo - 2026-10-06 - AUDIT L-17: A REAR STATION'S SHIELD RE-SEED STOPS AT ABEAM
  // ======================================================================================
  {
    // Mode 3 walking at -70, the buggy 10 m ahead / 3 m right (audit H-2's example): its measured angle is
    // +163 deg. The re-seed puts the station on the buggy's side - at +90, not at the front floor.
    Ctl c; c.live = -70.0f;
    const Tick t = ctlTick(c, 3, kNearDiag, 9.0f, 0, 4.0f, 18.0f, 0.0f, 10.0f, 3.0f, 0.0f, 0.1f, kSideBand);
    assert(t.escape && t.sh_reseed && c.live == 90.0f);
    // Every rear mode, every buggy position ahead: a shield re-seed never leaves the rear half.
    long n = 0;
    for (uint8_t m = 1; m <= 3; m++) {
      for (int ai = 2; ai <= 30; ai += 2) for (int ci = -12; ci <= 12; ci++) {
        if (ci == 0) continue;
        Ctl k; k.live = (ci > 0) ? -60.0f : 60.0f;               // walking on the far side
        const Tick u = ctlTick(k, m, kNearDiag, 9.0f, 0, 4.0f, 18.0f, 0.0f, (float)ai, (float)ci, 0.0f, 0.1f, kSideBand);
        if (u.sh_reseed) { assert(fabsf(k.live) <= 90.0f + 1e-4f); n++; }
      }
    }
    // F4/F5 keep the front floor (as H-1): the clamp is for the rear modes only.
    Ctl f; f.live = -70.0f; f.aborted = false;
    const Tick tf = ctlTick(f, 4, kNearDiag, 9.0f, 0, 4.0f, 18.0f, 0.0f, 10.0f, 3.0f, 0.0f, 0.1f, kSideBand);
    if (tf.sh_reseed) assert(f.live > 90.0f);
    printf("L-17 rear re-seeds clamped to +/-90: %ld re-seeds checked\n", n);
    assert(n > 50);
  }

  printf("largest one-tick lookahead change on an unflagged tick, all simulations: %.3f m\n", (double)g_dlook_max_quiet);
  printf("follow_me_station_test: all assertions passed\n");
  return 0;
}
