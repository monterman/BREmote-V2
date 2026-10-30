// V2.5-Evo - 2026-10-30 - Host test for Common/FollowMeGovernor.h (audit 2026-10-20 R1, finding F-2): Follow-Me cap 3, the
//   speed governor. Runs the exact header fmComputeThrottleCap() (RTMState.ino) calls.
//   0. The refactor changed nothing outside R1: the old inline cap-3 code, restated verbatim, against the header with the
//      side-offset gate off - bit for bit over two million random inputs, every branch.
//   1. Unit checks: the R1 gate (51 deg at a 9 m station and min_dist 5), the gap term's clamps, the fade to 0 head-on,
//      the floor.
//   2-5. A CLOSED-LOOP SIMULATION of a buggy following a rider (10 Hz FM ticks, the rider's fix at 3 Hz, the RX's 2 s
//      position filter and speed / course derivation, the lag anchor, the side zone (fmSideZoneStep), the rear station,
//      caps 2 / 3 / 4, condition 8, condition 9 and the raw-speed return candidate, a yaw-limited buggy and a fitted
//      throttle-to-speed model). The audit (2026-10-20 R1) lists the cases:
//        2. steady gap: modes 1/3 <= 50 ft at 9.8 mph and <= 60 ft at 10.9 mph; mode 2 within 2 ft of today;
//        3. the rider stops dead (-9 mph/s) from steady state: closest approach >= 23 ft in modes 1/3, no worse in mode 2;
//        4. a head-on U-turn at 10 mph from 82 ft: closest approach no worse than today by more than 2 ft - asserted for
//           2 s and 4 s turns; slower 6 s / 8 s turns toward the station's side do NOT hold it and are printed instead;
//        5. cap 3 within +/-20 counts at a fixed geometry with the GPS noise.
//   THE NOISE: the rider's fix moves by up to 3 ft (a 6 ft band - the audit's "6 ft"; its R2 pacing case of 82-115 ft
//   inside the 79-118 ft band is exactly that band) and the buggy's GPS speed by up to +/-0.3 mph, both uniform, white.
//   THE BUGGY: holding speed v takes throttle 255 x (v / v_top)^n; the speed relaxes to that over 2.4 s (which also makes
//   a cap-0 coast from 9.8 mph about 10.5 m = 34 ft, the audit's "30-40 ft"). Two fits, speeds and throttle only:
//     the audit's 2-point fit of 19 Oct, v_top 15.5 mph, n 2 (about 100/255 holds 9.6 mph);
//     the 30 Oct ride, v_top 16.2 mph (top speed in manual), n 1.36 (127/255 held 9.7 mph while following).
//   Owner's RX1 config: min_dist 5 m, band 4 m (9 m station), near_diag 60, zone 35 / 45, foiler_low_speed 8 km/h,
//   boogie_vmax_in_followme 40 km/h, align threshold 45 deg with cap 50, steer response 2 (2 s), auto-return on.
// Build and run:  g++ -std=c++17 -O1 -Wall -Wextra -o follow_me_governor_test follow_me_governor_test.cpp && ./follow_me_governor_test
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "../../Source/Common/FollowMeGovernor.h"
#include "../../Source/Common/FollowMeStation.h"

// ---- The RX's constants (RTMState.ino), stated here so the test says which numbers it claims ----
static const float kGain        = 0.5f;    // kFmGapGainKmhPerM
static const float kGapMax      = 15.0f;   // kFmGapMaxKmh
static const float kGainSide    = 1.5f;    // kFmGapGainSideKmhPerM (R1)
static const float kGapMaxSide  = 25.0f;   // kFmGapMaxSideKmh (R1)
static const float kSideMargin  = 2.0f;    // kFmGapSideMarginM (R1)
static const float kMargin      = 5.0f;    // kFmClosingMarginKmh
static const float kFloor       = 3.0f;    // kFmGovFloorKmh
static const float kCourseMin   = 5.0f;    // kFmCourseValidSpeedKmh
static const float kZoneEnter   = 35.0f, kZoneExit = 45.0f;                    // owner's zone_angle_enter / exit
static const float kZoneStEnter = 15.0f, kZoneStExit = 25.0f, kZoneStMax = 90.0f;   // kFmZoneStation* (R3)
static const float kRawMaxKmh   = 4.0f;    // kFmReturnRiderRawMaxKmh
// ---- The owner's RX1 config ----
static const float kMinDist     = 5.0f;    // min_dist_m
static const float kBand        = 4.0f;    // followme_smoothing_band_m
static const float kDFollow     = kMinDist + kBand;
static const float kNearDiag    = 60.0f;   // near_diag_offset_deg
static const float kFoilerLow   = 8.0f;    // foiler_low_speed_kmh
static const float kVmax        = 40.0f;   // boogie_vmax_in_followme_kmh (set 2026-10-13)
static const float kAlignDeg    = 45.0f;   // rtm_align_threshold_deg
static const float kAlignCap    = 50.0f;   // fm_align_cap
static const float kTau         = 2.0f;    // rtm_steer_response 2 -> target_filter_tau_s
// ---- The simulation ----
static const double kDt         = 0.1;     // FM tick
static const double kTauBuggy   = 2.4;     // s, speed response (and cap-0 coast: v x 2.4 s)
static const double kYawGain    = 1.5;     // deg/s per deg of heading error
static const double kYawMax     = 45.0;    // deg/s
static const float  kFixNoiseM  = 0.646f;  // per axis -> |noise| <= 0.914 m (3 ft): a 6 ft band
static const float  kSpdNoiseKmh = 0.483f; // +/-0.3 mph
static const double kMph        = 0.44704; // m/s per mph
static const double kPi         = 3.14159265358979323846;

static int g_checks = 0;
#define CHECK(x) do { assert(x); g_checks++; } while (0)

// ======================================================================================
// 0. THE OLD INLINE CAP-3 CODE, RESTATED VERBATIM (RTMState.ino fmComputeThrottleCap() before 2026-10-30)
// ======================================================================================
// Only the variable plumbing differs: globals became parameters. Returns the cap 3 contribution as the old code
// computed it against an incoming `cap` of 255.
static uint16_t oldCap3(float dist_m, float min_dist, float band, bool theta_ok, float theta, bool aim_outward,
                        bool frame_valid, float station_live_deg, float along_kmh, float rider_kmh, float vmax,
                        bool fade_bypass, float buggy_kmh)
{
  uint16_t cap = 255;
  float d_follow_g = min_dist + band;
  if (d_follow_g < 0.5f) d_follow_g = 0.5f;
  float gap_kmh = (dist_m - d_follow_g) * kGain;
  if (gap_kmh < 0.0f)    gap_kmh = 0.0f;
  if (gap_kmh > kGapMax) gap_kmh = kGapMax;
  float gov;
  if (aim_outward) {
    gov = rider_kmh;
  } else if (frame_valid && fabsf(station_live_deg) > 90.0f) {
    gov = rider_kmh + along_kmh;
  } else {
    gov = rider_kmh + kMargin + (theta_ok ? gap_kmh : 0.0f);
  }
  if (gov < 0.0f) gov = 0.0f;
  if (gov > vmax) gov = vmax;
  if (theta_ok && theta > 90.0f && !fade_bypass) {
    float frac = (180.0f - theta) / 90.0f;
    if (frac < 0.0f) frac = 0.0f;
    gov *= frac;
  }
  if (gov <= kFloor) {
    cap = 0;
  } else {
    float speed_frac = buggy_kmh / gov;
    if (speed_frac > 1.0f) speed_frac = 1.0f;
    if (speed_frac < 0.0f) speed_frac = 0.0f;
    uint16_t c = (uint16_t)((1.0f - speed_frac) * 255.0f);
    if (c < cap) cap = c;
  }
  return cap;
}

// The NEW path exactly as fmComputeThrottleCap() now runs it (gate -> gap -> target -> fmGovCap).
static uint16_t newCap3(float dist_m, float min_dist, float band, bool theta_ok, float theta, bool aim_outward,
                        bool frame_valid, float station_live_deg, float station_cross_m, float along_kmh,
                        float rider_kmh, float vmax, bool fade_bypass, float buggy_kmh, bool r1)
{
  float d_follow_g = min_dist + band;
  if (d_follow_g < 0.5f) d_follow_g = 0.5f;
  const bool  side_offset = r1 && fmGovSideOffset(frame_valid, station_cross_m, min_dist, kSideMargin);
  const float gap_kmh     = side_offset ? fmGovGapKmh(dist_m, d_follow_g, kGainSide, kGapMaxSide)
                                        : fmGovGapKmh(dist_m, d_follow_g, kGain, kGapMax);
  const bool  front = frame_valid && fabsf(station_live_deg) > 90.0f;
  const float gov   = fmGovTargetKmh(aim_outward, front, front ? along_kmh : 0.0f, rider_kmh, kMargin, theta_ok, gap_kmh);
  uint16_t cap = 255;
  const uint16_t c = fmGovCap(gov, vmax, theta_ok, theta, fade_bypass, kFloor, buggy_kmh);
  if (c < cap) cap = c;
  return cap;
}

struct Rng {
  uint32_t s;
  float u() { s = s * 1103515245u + 12345u; return (float)((s >> 8) & 0xFFFFu) / 65535.0f; }
  float sym(float a) { return (u() * 2.0f - 1.0f) * a; }
};

// ======================================================================================
// THE SIMULATED WORLD - East / North metres, bearings clockwise from North
// ======================================================================================
struct V2 { double x, y; };
static double wrap180d(double a) { while (a > 180.0) a -= 360.0; while (a <= -180.0) a += 360.0; return a; }
static double bearingDeg(V2 a, V2 b) { double d = atan2(b.x - a.x, b.y - a.y) * 180.0 / kPi; if (d < 0.0) d += 360.0; return d; }
static double distM(V2 a, V2 b) { return hypot(b.x - a.x, b.y - a.y); }
static V2 project(V2 p, double brg_deg, double d) { const double r = brg_deg * kPi / 180.0; return { p.x + d * sin(r), p.y + d * cos(r) }; }

struct Model { const char *name; double v_top_mph; double n; };
static const Model kModelAudit = { "audit fit 19 Oct (v/15.5 mph)^2", 15.5, 2.0 };
static const Model kModel1030  = { "30 Oct fit (v/16.2 mph)^1.36", 16.2, 1.36 };
static double holdThrottle(const Model &m, double v_ms) { return 255.0 * pow((v_ms / kMph) / m.v_top_mph, m.n); }
static double steadySpeedMs(const Model &m, double cmd) { if (cmd <= 0.0) return 0.0; return m.v_top_mph * kMph * pow(cmd / 255.0, 1.0 / m.n); }

struct Sim {
  const Model *model; uint8_t mode; bool r1; bool noise; Rng rng;
  float vmax;                                       // boogie_vmax_in_followme_kmh (the owner's 40 unless a case says)
  double t;
  V2 rider; double rcourse, rv;                     // rider truth: position, course (deg), speed (m/s)
  V2 fix; long fix_idx;                             // the rider's last fix (3 Hz, noisy)
  V2 filt, prev_filt; bool filt_init; double prev_filt_t;
  float vf_kmh, course_f;                           // fm_rider_speed_kmh, fm_rider_course_deg (-1 = invalid)
  V2 raw_base; double raw_base_t; float raw_kmh;    // fm_rider_raw_kmh
  V2 buggy; double bh, bv;                          // buggy truth: position, heading, speed
  bool active, diag; int flips; long ticks_align;
  uint16_t cap3, cap; double cmd; bool hold_now;
};

static void simInit(Sim &s, const Model *m, uint8_t mode, bool r1, bool noise, uint32_t seed, double rider_mph,
                    double buggy_along, double buggy_cross)
{
  s.model = m; s.mode = mode; s.r1 = r1; s.noise = noise; s.rng.s = seed; s.vmax = kVmax;
  s.t = 0.0; s.rider = { 0.0, 0.0 }; s.rcourse = 0.0; s.rv = rider_mph * kMph;
  s.fix = s.rider; s.fix_idx = 0;
  // The filter is pre-warmed to its steady state (it trails the held fix by v x (tau + dt)), so a run starts following.
  s.filt = project(s.rider, 180.0, s.rv * (kTau + kDt)); s.filt_init = true;
  s.prev_filt = project(s.filt, 180.0, s.rv * 0.4); s.prev_filt_t = -0.4;
  s.vf_kmh = (float)(s.rv * 3.6); s.course_f = (s.vf_kmh >= kCourseMin) ? 0.0f : -1.0f;
  s.raw_base = project(s.rider, 180.0, s.rv * 1.0); s.raw_base_t = -1.0; s.raw_kmh = s.vf_kmh;
  s.buggy = { buggy_cross, buggy_along }; s.bh = 0.0; s.bv = s.rv;
  s.active = true; s.diag = true; s.flips = 0; s.ticks_align = 0; s.cap3 = 255; s.cap = 255; s.cmd = 255.0;
  s.hold_now = false;
}

// One 100 ms tick. Rider truth first (the caller sets rcourse / rv), then the RX's view and its decision, then the buggy.
// free_geom: the buggy's position is NOT integrated (section 5 holds the geometry fixed and only reads cap 3).
static void simTick(Sim &s, bool free_geom = false)
{
  s.t += kDt;
  s.rider = project(s.rider, s.rcourse, s.rv * kDt);
  // ---- the rider's fix at 3 Hz ----
  const long idx = (long)floor(s.t * 3.0 + 1e-9);
  bool new_fix = false;
  if (idx != s.fix_idx) {
    s.fix_idx = idx;
    s.fix = s.rider;
    if (s.noise) { s.fix.x += s.rng.sym(kFixNoiseM); s.fix.y += s.rng.sym(kFixNoiseM); }
    new_fix = true;
  }
  // ---- updateFmRiderTracking(): EMA on the held fix, speed and course over >= 0.4 s ----
  const double alpha = kDt / (kTau + kDt);
  s.filt.x += alpha * (s.fix.x - s.filt.x);
  s.filt.y += alpha * (s.fix.y - s.filt.y);
  if (s.t - s.prev_filt_t >= 0.4 - 1e-9) {
    s.vf_kmh   = (float)(distM(s.prev_filt, s.filt) / (s.t - s.prev_filt_t) * 3.6);
    s.course_f = (s.vf_kmh >= kCourseMin) ? (float)bearingDeg(s.prev_filt, s.filt) : -1.0f;
    s.prev_filt = s.filt; s.prev_filt_t = s.t;
  }
  // ---- updateFmRawRiderSpeed(): per new fix, over a >= 1 s baseline ----
  if (new_fix && s.t - s.raw_base_t >= 1.0 - 1e-9) {
    s.raw_kmh = (float)(distM(s.raw_base, s.fix) / (s.t - s.raw_base_t) * 3.6);
    s.raw_base = s.fix; s.raw_base_t = s.t;
  }
  // ---- the buggy's GPS (position exact; all distance noise is on the rider's side), speed +/-0.3 mph ----
  float buggy_kmh = (float)(s.bv * 3.6);
  if (s.noise) buggy_kmh += s.rng.sym(kSpdNoiseKmh);
  const float dist_m = (float)distM(s.fix, s.buggy);

  // ---- conditions 8 and 9 and the return candidate (auto-return on): any of them -> HOLD, cap 0, steering centred ----
  if (s.active) {
    const bool speed_ok = s.vf_kmh >= kFoilerLow;
    const bool dist_ok  = dist_m >= kMinDist;
    const bool cand     = s.raw_kmh >= 0.0f && s.raw_kmh < kRawMaxKmh;
    if (!speed_ok || !dist_ok || cand) s.active = false;
  }
  s.hold_now = !s.active;
  double yaw = 0.0;
  if (!s.active) {
    s.cap3 = 0; s.cap = 0; s.cmd = 0.0;
  } else {
    // ---- computeFmTarget(): degraded, or the rear station round the anchor with the R3 side zone ----
    V2 aim; bool frame_valid = false; float st_cross = 0.0f, psi_live = 0.0f;
    if (s.course_f < 0.0f) {
      aim = project(s.filt, bearingDeg(s.filt, s.buggy), kDFollow);
    } else {
      double lag = s.vf_kmh / 3.6 * kTau;
      if (lag > 2.0 * kDFollow) lag = 2.0 * kDFollow;
      const V2 anchor = project(s.filt, s.course_f, lag);
      const float psi_zone = fmMeasuredStationDeg(s.course_f, (float)bearingDeg(anchor, s.buggy));
      const float sp = fmStationNearestRearPresetDeg(s.mode, kNearDiag);
      const bool d2 = fmSideZoneStep(s.diag, psi_zone, (sp > 0.0f) ? 1 : ((sp < 0.0f) ? -1 : 0), kNearDiag,
                                     kZoneEnter, kZoneExit, kZoneStEnter, kZoneStExit, kZoneStMax);
      if (d2 != s.diag) s.flips++;
      s.diag = d2;
      float psi = fmStationPresetDeg(s.mode, kNearDiag, 45.0f);
      if ((s.mode == 1 || s.mode == 3) && !s.diag) psi = 0.0f;
      aim = project(anchor, s.course_f + 180.0 - psi, kDFollow);
      float al, cr;
      fmStationAlongCrossM(psi, kDFollow, &al, &cr);
      st_cross = cr; psi_live = psi; frame_valid = true;
    }
    const double err = wrap180d(bearingDeg(s.buggy, aim) - s.bh);
    const bool aligning = fabs(err) > kAlignDeg;
    // ---- fmComputeThrottleCap(): caps 2, 3 and 4 (cap 5's engage ramp ended long before) ----
    uint16_t cap = 255;
    if (dist_m < kMinDist + kBand) {
      float frac = (dist_m - kMinDist) / kBand;
      if (frac < 0.0f) frac = 0.0f;
      if (frac > 1.0f) frac = 1.0f;
      const uint16_t c = (uint16_t)(frac * 255.0f);
      if (c < cap) cap = c;
    }
    bool theta_ok = false; float theta = 0.0f;
    if (s.course_f >= 0.0f && distM(s.buggy, s.filt) > 3.0) {
      theta = (float)fabs(wrap180d(s.course_f - bearingDeg(s.buggy, s.filt)));
      theta_ok = true;
    }
    s.cap3 = newCap3(dist_m, kMinDist, kBand, theta_ok, theta, false, frame_valid, psi_live,
                     st_cross, 0.0f, s.vf_kmh, s.vmax, false, buggy_kmh, s.r1);
    if (s.cap3 < cap) cap = s.cap3;
    if (aligning) { if (kAlignCap < cap) cap = (uint16_t)kAlignCap; s.ticks_align++; }
    s.cap = cap; s.cmd = cap;                           // full trigger: min(trigger, cap) = cap
    yaw = kYawGain * err;
    if (yaw >  kYawMax) yaw =  kYawMax;
    if (yaw < -kYawMax) yaw = -kYawMax;
  }
  if (free_geom) return;
  // ---- the buggy ----
  const double v_ss = steadySpeedMs(*s.model, s.cmd);
  s.bv += (v_ss - s.bv) * (kDt / kTauBuggy);
  s.bh += yaw * kDt;
  s.buggy = project(s.buggy, s.bh, s.bv * kDt);
}

struct Steady { double gap_ft, drift_ft_s, cap_mean, buggy_mph, align_pct; int flips; };

// Rider straight at rider_mph; the buggy starts 30 m back on the rider's line at his speed. 240 s; the last 60 s judged.
static Steady runSteady(const Model *m, uint8_t mode, bool r1, double rider_mph, uint32_t seed, float vmax = kVmax)
{
  Sim s;
  simInit(s, m, mode, r1, true, seed, rider_mph, -30.0, 0.0);
  s.vmax = vmax;
  double sum_gap = 0.0, sum_cap = 0.0, sum_v = 0.0, g_a = 0.0, g_b = 0.0;
  long n = 0, na = 0, nb = 0, align0 = 0;
  for (int i = 0; i < 2400; i++) {
    simTick(s);
    if (i == 1800) align0 = s.ticks_align;
    if (i >= 1800) { sum_gap += distM(s.rider, s.buggy); sum_cap += s.cap; sum_v += s.bv; n++; }
    if (i >= 1800 && i < 1900) { g_a += distM(s.rider, s.buggy); na++; }
    if (i >= 2300)             { g_b += distM(s.rider, s.buggy); nb++; }
  }
  Steady r;
  r.gap_ft = sum_gap / n / 0.3048;
  r.drift_ft_s = ((g_b / nb) - (g_a / na)) / 50.0 / 0.3048;
  r.cap_mean = sum_cap / n;
  r.buggy_mph = sum_v / n / kMph;
  r.align_pct = 100.0 * (double)(s.ticks_align - align0) / (double)n;
  r.flips = s.flips;
  CHECK(s.active);                                      // nothing in a straight run may HOLD
  return r;
}

int main()
{
  setvbuf(stdout, nullptr, _IONBF, 0);

  // ======================================================================================
  // 0. THE REFACTOR IS BIT-FOR-BIT OUTSIDE R1
  // ======================================================================================
  {
    Rng r{ 20261030u };
    long diffs = 0;
    for (int i = 0; i < 2000000; i++) {
      const float dist  = r.u() * 200.0f;
      const float mind  = r.u() * 12.0f, band = r.u() * 8.0f;
      const bool  th_ok = r.u() < 0.8f;
      const float theta = r.u() * 180.0f;
      const bool  outw  = r.u() < 0.1f;
      const bool  fv    = r.u() < 0.8f;
      const float psi   = r.sym(150.0f);
      const float along = r.sym(15.0f);
      const float rider = r.u() * 45.0f;
      const float vmax  = r.u() * 100.0f;
      const bool  byp   = r.u() < 0.2f;
      const float buggy = r.u() * 50.0f - 1.0f;
      const float cross = r.sym(20.0f);
      const uint16_t a = oldCap3(dist, mind, band, th_ok, theta, outw, fv, psi, along, rider, vmax, byp, buggy);
      const uint16_t b = newCap3(dist, mind, band, th_ok, theta, outw, fv, psi, cross, along, rider, vmax, byp, buggy, false);
      if (a != b) diffs++;
      // R1 on: the side-offset gain can only raise the target, so cap 3 can only bite less - never below today's cap -
      // and it is still a cap (0..255). Outside the gate it is today's to the bit.
      const uint16_t c = newCap3(dist, mind, band, th_ok, theta, outw, fv, psi, cross, along, rider, vmax, byp, buggy, true);
      CHECK(c <= 255);
      CHECK(c >= a);
      if (!fmGovSideOffset(fv, cross, mind, kSideMargin)) CHECK(c == a);
    }
    printf("0. refactor: 2,000,000 random inputs, old inline vs header with R1 off: %ld differences\n", diffs);
    CHECK(diffs == 0);
  }

  // ======================================================================================
  // 1. UNIT CHECKS
  // ======================================================================================
  {
    float al, cr;
    fmStationAlongCrossM(51.1f, kDFollow, &al, &cr);
    CHECK(fmGovSideOffset(true, cr, kMinDist, kSideMargin));            // 51.1 deg at 9 m: 7.004 m out
    fmStationAlongCrossM(-51.1f, kDFollow, &al, &cr);
    CHECK(fmGovSideOffset(true, cr, kMinDist, kSideMargin));            // either side
    fmStationAlongCrossM(50.9f, kDFollow, &al, &cr);
    CHECK(!fmGovSideOffset(true, cr, kMinDist, kSideMargin));           // 50.9 deg: 6.99 m, below the gate
    fmStationAlongCrossM(60.0f, kDFollow, &al, &cr);
    CHECK(fmGovSideOffset(true, cr, kMinDist, kSideMargin));            // the owner's 60 deg: 7.79 m
    CHECK(!fmGovSideOffset(false, cr, kMinDist, kSideMargin));          // no frame this tick: never
    CHECK(!fmGovSideOffset(true, 0.0f, kMinDist, kSideMargin));         // directly behind: never
    CHECK(fmGovGapKmh(5.0f, 9.0f, kGainSide, kGapMaxSide) == 0.0f);     // inside the station: no allowance
    CHECK(fabsf(fmGovGapKmh(19.0f, 9.0f, kGainSide, kGapMaxSide) - 15.0f) < 1e-4f);
    CHECK(fmGovGapKmh(200.0f, 9.0f, kGainSide, kGapMaxSide) == kGapMaxSide);
    CHECK(fmGovGapKmh(200.0f, 9.0f, kGain, kGapMax) == kGapMax);
    // Head-on, the fade takes the whole target to zero whatever the gap term: cap 0.
    CHECK(fmGovCap(fmGovTargetKmh(false, false, 0.0f, 16.0f, kMargin, true, kGapMaxSide), kVmax, true, 180.0f, false,
                   kFloor, 0.0f) == 0);
    CHECK(fmGovCap(2.9f, kVmax, false, 0.0f, false, kFloor, 0.0f) == 0);   // under the floor: 0, never "no cap"
    CHECK(fmGovCap(40.0f, kVmax, false, 0.0f, false, kFloor, 40.0f) == 0); // at the target: 0
    printf("1. unit checks passed (gate at 51 deg for a 9 m station and min_dist 5 m)\n");
  }

  // ======================================================================================
  // 2. STEADY GAP, CLOSED LOOP (audit R1 test a / b), plus the 30 Oct speeds
  // ======================================================================================
  const double speeds[] = { 6.0, 7.0, 8.0, 9.0, 9.8, 10.9, 12.0, 12.5 };
  const int nsp = (int)(sizeof(speeds) / sizeof(speeds[0]));
  const Model *models[2] = { &kModelAudit, &kModel1030 };
  for (int mi = 0; mi < 2; mi++) {
    printf("\n2. steady following, %s, vmax %.0f km/h, 240 s, last 60 s:\n", models[mi]->name, (double)kVmax);
    printf("   rider |  mode 1 today          |  mode 1 R1             |  mode 3 R1  |  mode 2 today | mode 2 R1\n");
    printf("   mph   |  gap ft  drift  cap    |  gap ft  drift  cap    |  gap ft     |  gap ft       | gap ft\n");
    for (int k = 0; k < nsp; k++) {
      const Steady t1 = runSteady(models[mi], 1, false, speeds[k], 101u + k);
      const Steady r1 = runSteady(models[mi], 1, true,  speeds[k], 101u + k);
      const Steady r3 = runSteady(models[mi], 3, true,  speeds[k], 101u + k);
      const Steady t2 = runSteady(models[mi], 2, false, speeds[k], 101u + k);
      const Steady r2 = runSteady(models[mi], 2, true,  speeds[k], 101u + k);
      const bool steady1 = fabs(r1.drift_ft_s) < 0.3;
      printf("   %5.1f | %6.0f %+6.2f %5.0f  | %6.0f %+6.2f %5.0f  | %6.0f      | %6.0f        | %6.0f%s\n",
             speeds[k], t1.gap_ft, t1.drift_ft_s, t1.cap_mean, r1.gap_ft, r1.drift_ft_s, r1.cap_mean, r3.gap_ft,
             t2.gap_ft, r2.gap_ft, steady1 ? "" : "   <- R1: no steady gap, the buggy falls behind");
      // (b) mode 2 is untouched by R1: the same run to the bit.
      CHECK(fabs(r2.gap_ft - t2.gap_ft) <= 2.0);
      // R1 never leaves modes 1/3 further back than today (the gate can only raise cap 3).
      CHECK(r1.gap_ft <= t1.gap_ft + 1.0);
      CHECK(r3.gap_ft <= t1.gap_ft + 1.0);
      // Modes 1 and 3 are mirror images.
      CHECK(fabs(r1.gap_ft - r3.gap_ft) <= 2.0);
      // R3: the station does not hunt while following in a straight line.
      CHECK(r1.flips <= 1 && r3.flips <= 1);
      if (mi == 0 && fabs(speeds[k] - 9.8) < 1e-6) {
        printf("   audit fit, 9.8 mph: mode 1/3 R1 %.0f ft (<= 50 asked), today %.0f ft; buggy %.1f mph, align cap %.0f %% of ticks\n",
               r1.gap_ft, t1.gap_ft, r1.buggy_mph, r1.align_pct);
        CHECK(r1.gap_ft <= 50.0 && r3.gap_ft <= 50.0);                   // (a)
      }
      if (mi == 0 && fabs(speeds[k] - 10.9) < 1e-6) {
        printf("   audit fit, 10.9 mph: mode 1/3 R1 %.0f ft (<= 60 asked), today %.0f ft\n", r1.gap_ft, t1.gap_ft);
        CHECK(r1.gap_ft <= 60.0 && r3.gap_ft <= 60.0);                   // (a)
      }
    }
    // The ceiling the 40 km/h vmax clamp sets on any target: the speed at which cap 3 at a 40 km/h target equals the
    // throttle that holds it. Above it no gap term can help (R4's question).
    double lo = 0.0, hi = 30.0;
    for (int it = 0; it < 60; it++) {
      const double mid = 0.5 * (lo + hi);
      const double cap = 255.0 * (1.0 - (mid * 3.6) / kVmax);
      if (cap > holdThrottle(*models[mi], mid)) lo = mid; else hi = mid;
    }
    printf("   with the %.0f km/h vmax clamp the governor can hold at most %.1f mph under this model\n",
           (double)kVmax, lo / kMph);
    // INFORMATION ONLY, for the R4 question - not R1's spec and not a recommendation: the same runs with the vmax clamp
    // raised to 60 km/h (a setting, boogie_vmax_in_followme_kmh). Nothing is asserted on them.
    for (int k = 6; k < nsp; k++) {
      const Steady a = runSteady(models[mi], 1, true, speeds[k], 101u + k, 60.0f);
      printf("   (vmax 60 km/h, information only) %.1f mph, mode 1 R1: gap %.0f ft, drift %+.2f ft/s%s\n", speeds[k],
             a.gap_ft, a.drift_ft_s, (fabs(a.drift_ft_s) < 0.3) ? "" : " - still falls behind");
    }
  }

  // ======================================================================================
  // 3. THE RIDER STOPS DEAD (audit R1 test c): -9 mph/s from steady state
  // ======================================================================================
  printf("\n3. rider stops at -9 mph/s from steady following; closest approach (truth) over the next 20 s:\n");
  for (int mi = 0; mi < 2; mi++) {
    for (int k = 0; k < 2; k++) {
      const double u = (k == 0) ? 9.8 : 10.9;
      double ca[3][2];
      const uint8_t modes[3] = { 1, 3, 2 };
      for (int md = 0; md < 3; md++) {
        for (int g = 0; g < 2; g++) {
          Sim s;
          simInit(s, models[mi], modes[md], g == 1, true, 555u + k, u, -30.0, 0.0);
          for (int i = 0; i < 1500; i++) simTick(s);    // 150 s: settled
          double min_d = 1e9, hold_t = -1.0;
          const double t0 = s.t;
          for (int i = 0; i < 200; i++) {
            s.rv -= 9.0 * kMph * kDt;
            if (s.rv < 0.0) s.rv = 0.0;
            simTick(s);
            if (s.hold_now && hold_t < 0.0) hold_t = s.t - t0;
            const double d = distM(s.rider, s.buggy);
            if (d < min_d) min_d = d;
          }
          ca[md][g] = min_d / 0.3048;
          if (md == 0 && g == 1)
            printf("   %s, %.1f mph: Follow-Me let go %.1f s after the rider began to stop\n", models[mi]->name, u, hold_t);
        }
      }
      printf("   %s, %.1f mph: mode 1 today %.0f ft / R1 %.0f ft; mode 3 R1 %.0f ft; mode 2 today %.0f ft / R1 %.0f ft\n",
             models[mi]->name, u, ca[0][0], ca[0][1], ca[1][1], ca[2][0], ca[2][1]);
      CHECK(ca[0][1] >= 23.0 && ca[1][1] >= 23.0);   // modes 1/3 with R1: past the stopped rider at >= 23 ft
      CHECK(ca[2][1] >= ca[2][0] - 0.01);            // mode 2: no worse than today (R1 is off there)
    }
  }

  // ======================================================================================
  // 4. HEAD-ON U-TURN AT 10 mph FROM 82 ft (audit R1 test d)
  // ======================================================================================
  // The rider turns 180 deg at a steady rate - in 2, 4, 6 or 8 s, so his return line passes the buggy anywhere from a
  // few feet to tens of feet off - either way, with the buggy 82 ft (25 m) back on its station line. Closest approach
  // over 20 s, R1 against today in the same world. Also reported: how fast the gap was closing when the rider first came
  // inside 50 ft (the audit expected about 5 % more with R1).
  // ASSERTED for the 2 s and 4 s turns (a head-on U-turn, 90 and 45 deg/s at 10 mph): R1 no more than 2 ft closer than
  // today. NOT ASSERTED, PRINTED, for the 6 s and 8 s turns: a slower turn TOWARD the station's side meets a buggy that
  // R1 has brought up faster from 82 ft, and R1 as specified loses up to about 8 ft there (the cap is already 0 when they
  // meet - it is the extra catch-up momentum). Left in plain view for the auditor; R1 is implemented as specified.
  printf("\n4. rider U-turns at 10 mph with the buggy 82 ft back on its station line (turns of 2/4/6/8 s, left and right):\n");
  int not_asserted_fails = 0;
  for (int mi = 0; mi < 2; mi++) {
    for (int mode = 1; mode <= 3; mode++) {
      double worst_delta = -1e9, worst_today = 0.0, worst_r1 = 0.0, close_today = 0.0, close_r1 = 0.0;
      double min_today = 1e9, min_r1 = 1e9;
      int worst_secs = 0, worst_dir = 0;
      for (int secs = 2; secs <= 8; secs += 2) {
        for (int dir = -1; dir <= 1; dir += 2) {
          double ca[2], closing[2] = { 0.0, 0.0 };
          for (int g = 0; g < 2; g++) {
            Sim s;
            float al, cr;
            fmStationAlongCrossM((mode == 2) ? 0.0f : ((mode == 1) ? kNearDiag : -kNearDiag), kDFollow, &al, &cr);
            const double back = -sqrt(25.0 * 25.0 - (double)cr * cr);
            simInit(s, models[mi], (uint8_t)mode, g == 1, true, 909u, 10.0, back, cr);
            for (int i = 0; i < 10; i++) simTick(s);      // 1 s on the line
            double min_d = 1e9, prev_d = distM(s.rider, s.buggy);
            bool inside50 = false;
            const int turn_ticks = secs * 10;
            for (int i = 0; i < 200; i++) {
              if (i < turn_ticks) s.rcourse += dir * (180.0 / turn_ticks);
              simTick(s);
              const double d = distM(s.rider, s.buggy);
              if (!inside50 && d < 50.0 * 0.3048) { inside50 = true; closing[g] = (prev_d - d) / kDt / kMph; }
              prev_d = d;
              if (d < min_d) min_d = d;
            }
            ca[g] = min_d / 0.3048;
          }
          if (secs <= 4) {
            CHECK(ca[1] >= ca[0] - 2.0);                  // (d): no worse than today by more than 2 ft
          } else if (ca[1] < ca[0] - 2.0) {
            not_asserted_fails++;
            printf("   NOT ASSERTED (for the auditor): %s, mode %d, %d s turn %s - today %.1f ft, R1 %.1f ft (%.1f ft closer)\n",
                   models[mi]->name, mode, secs, dir < 0 ? "left" : "right", ca[0], ca[1], ca[0] - ca[1]);
          }
          if (ca[0] - ca[1] > worst_delta) {
            worst_delta = ca[0] - ca[1]; worst_today = ca[0]; worst_r1 = ca[1]; worst_secs = secs; worst_dir = dir;
          }
          if (ca[0] < min_today) min_today = ca[0];
          if (ca[1] < min_r1)    min_r1 = ca[1];
          if (secs == 4 && dir == -1) { close_today = closing[0]; close_r1 = closing[1]; }
        }
      }
      printf("   %-34s mode %d: closest today %4.1f / R1 %4.1f ft; worst R1 loss %+4.1f ft (%d s turn %s: %4.1f -> %4.1f); "
             "closing at 50 ft (4 s left turn) %.1f -> %.1f mph\n",
             models[mi]->name, mode, min_today, min_r1, -worst_delta, worst_secs, worst_dir < 0 ? "left" : "right",
             worst_today, worst_r1, close_today, close_r1);
    }
  }
  printf("   %d slower-turn case(s) over the 2 ft line, reported above and not asserted\n", not_asserted_fails);

  // ======================================================================================
  // 5. CAP 3 AT A FIXED GEOMETRY WITH THE NOISE (audit R1 test e): within +/-20 counts
  // ======================================================================================
  // The buggy is held on its station line at the steady gap, moving with the rider; only the measurements are noisy.
  // The reference is the same run without noise. 60 s after a 30 s warm-up.
  printf("\n5. cap 3 at a fixed geometry, rider 9.8 mph, noise on (reference: the same run without noise):\n");
  for (int g = 0; g < 2; g++) {
    for (int mode = 1; mode <= 2; mode++) {
      const double gap = (g == 1 && mode == 1) ? 12.7 : 20.0;   // R1's steady gap beside the rider, today's otherwise
      float al, cr;
      fmStationAlongCrossM((mode == 1) ? kNearDiag : 0.0f, kDFollow, &al, &cr);
      const double back = -sqrt(gap * gap - (double)cr * cr);
      Sim a, b;
      simInit(a, &kModelAudit, (uint8_t)mode, g == 1, true,  77u, 9.8, back, cr);
      simInit(b, &kModelAudit, (uint8_t)mode, g == 1, false, 77u, 9.8, back, cr);
      int max_dev = 0;
      long sum = 0, n = 0;
      for (int i = 0; i < 900; i++) {
        a.buggy = project(a.rider, 0.0, 0.0); a.buggy.x += cr; a.buggy.y += back + a.rv * kDt;
        b.buggy = a.buggy;
        simTick(a, true);
        simTick(b, true);
        if (i >= 300) {
          const int dev = (int)a.cap3 - (int)b.cap3;
          const int adev = (dev < 0) ? -dev : dev;
          if (adev > max_dev) max_dev = adev;
          sum += b.cap3; n++;
        }
      }
      printf("   %s, mode %d, gap %.1f m: cap 3 about %ld, largest deviation %d counts\n",
             g == 1 ? "R1   " : "today", mode, gap, sum / n, max_dev);
      CHECK(max_dev <= 20);
    }
  }

  printf("\nfollow_me_governor_test: all %d checks passed\n", g_checks);
  return 0;
}
