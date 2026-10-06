// V2.5-Evo - 2026-10-02 - P2: the Follow-Me STATION MODEL - one live station angle around the
//   rider, five presets, and the pass geometry that lets the buggy take a station AHEAD of the
//   rider without ever crossing the rider's line.
//
//   Pure, header-only, no Arduino dependencies, so the host unit test in
//   Tools/tests/follow_me_station_test.cpp exercises the exact arithmetic the RX runs. The
//   controller (RTMState.ino) supplies every constant; nothing is hard-coded here.
//
// ---- THE STATION ANGLE psi ----
//   psi is measured AROUND THE RIDER, in degrees, 0 = directly behind the rider,
//   POSITIVE = the rider's RIGHT (clockwise seen from above). The bearing of the station from the
//   rider is therefore `course + 180 - psi`, which reproduces the board's existing convention
//   exactly: today's mode 1 (near-right) is `course + 180 - near_diag_offset_deg`, i.e. psi = +45.
//
//   Station numbering is CONTINUOUS and there is NO dead-ahead station, ever:
//     1 = rear-right   psi = +near_diag_offset_deg   (45 by default)
//     2 = behind       psi =  0
//     3 = rear-left    psi = -near_diag_offset_deg
//     4 = front-right  psi = +(180 - front_angle)    (+135 at the default 45 deg front angle)
//     5 = front-left   psi = -(180 - front_angle)
//   There is no mode 6 and |psi| can never reach 180.
//
// ---- WHY THERE IS NO DEAD-AHEAD STATION (the load-bearing safety rationale) ----
//   A buggy directly ahead of the rider has to ACCELERATE as the rider closes on it, and if its
//   motor fails it stops dead ON THE RIDER'S LINE and the rider hits it. The owner has already hit
//   this buggy once - a steering error, the foil struck it, the propeller broke, he limped home.
//   Off-axis, a dead motor leaves the buggy BESIDE the line, not on it, and the rider passes it.
//   The off-axis station also keeps the trailing rope off the rider's line. That is why the front
//   stations are a PAIR either side of dead-ahead with a hard floor between them, and why no
//   reachable combination of config values can put the station inside that floor.
//
// ---- THE PASS-GEOMETRY INVARIANTS (PG-1..PG-5) - what Rex can check ----
//   PG-1  The station's own CROSS-TRACK offset from the rider's course line is at least
//         pass_lateral_m whenever the station is ahead of the rider (|psi| >= 90). Enforced by the
//         radius floor inside fmStationRadiusM(), not by the preset table, so it holds at every
//         tuning including degenerate ones.
//   PG-2  psi may not exceed 90 deg in magnitude unless the BUGGY'S OWN measured cross-track offset
//         is at least pass_lateral_m AND ON THE SAME SIDE as the station it is heading for. This is
//         fmStationTargetCeilingDeg(): the station does not move ahead of the rider until the buggy
//         is physically wide of the rider's line. It is Rex's two-waypoint pass, expressed as a
//         ceiling on one variable instead of a second state machine.
//   PG-3  Cross-track offset is an AFFINE function of position along a straight line, so its minimum
//         over a segment is attained at one of the two endpoints. PG-1 and PG-2 together put BOTH
//         endpoints of the buggy->station aim segment at cross >= pass_lateral_m on the SAME side,
//         therefore EVERY point of the aim line is at cross >= pass_lateral_m. The aim line can
//         never cross the rider's course line while a front station is engaged, so it can never be
//         aimed through the rider. fmAimClearsRiderLine() is that test, stated once.
//   PG-4  If PG-3's precondition is ever broken anyway - the rider turns, so the buggy's cross
//         collapses while the station is still ahead - fmAimOutwardNeeded() reports it and the
//         controller steers OUTWARD (a waypoint further from the line than the buggy is), withdraws
//         the closing allowance and the convergence fade, and raises a geometry warning. It never
//         steers across. V2.5-Evo - 2026-10-06 (audit H-1): if the buggy is on the OTHER side of the
//         rider's line from the station, fmFrontRetreatSide() reports it and the controller re-seeds
//         the live angle on the BUGGY'S side and retreats to the rear preset there.
//   PG-5  Nothing in this header writes throttle. The station angle and the radius only move the
//         TARGET POINT. The min_dist_m cap-0, the boogie_vmax clamp and the subtract-only cap chain
//         are untouched in every mode.
#ifndef BREMOTE_FOLLOW_ME_STATION_H
#define BREMOTE_FOLLOW_ME_STATION_H

#include <math.h>
#include <stdint.h>

#ifndef BREMOTE_FMS_DEG2RAD
#define BREMOTE_FMS_DEG2RAD 0.01745329251994329577f
#endif

// fmsWrap180 - fold any angle into (-180, +180].
static inline float fmsWrap180(float a)
{
  while (a >  180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

// fmStationIsFront - is this station strictly AHEAD of the rider?
//
// STRICT, and the boundary matters. The abeam line (|psi| == 90) is the WAITING waypoint of the
// two-waypoint pass: the station sits level with the rider and pass_lateral_m to the side, and the
// buggy is sent out there precisely because it is NOT yet wide enough to go in front. So abeam is
// not "ahead": PG-4's escape and the fade bypass must both read false there, or the manoeuvre that
// gets the buggy wide would be mistaken for the pass it is a prerequisite for. The same strict test
// is used by fmFadeBypass(), so the two can never disagree.
//
// PG-1's radius floor is deliberately NOT written in terms of this function - it applies from abeam
// outward (a >= 90 inside fmStationRadiusM), because the abeam waypoint is exactly where the
// station's own clearance has to be guaranteed.
static inline bool fmStationIsFront(float psi_deg)
{
  return fabsf(psi_deg) > 90.0f;
}

// fmFrontRadiusM - the radius of a FRONT station, in metres.
//
// Two requirements, and the larger wins:
//   1. Robert's water-tested shape: at least radius_factor (2.0) x the follow distance, so the
//      front station is genuinely further out than the rear one.
//   2. THE LATERAL INVARIANT: r * sin(front_angle) >= lateral_min_m. At the default 45 deg and the
//      owner's 9 m follow distance that is max(18.0, 13/sin45) = 18.4 m, giving exactly 13.0 m of
//      clearance beside the rider's line. At the 35 deg minimum it is max(18.0, 13/sin35) = 22.7 m
//      (about 75 ft): a RIDER WHO ASKS FOR A TIGHTER ANGLE GETS A BIGGER RADIUS, never less
//      clearance. lateral_min_m is Rex B2's 13 m = an 8 m carve plus 5 m of relative GPS error.
//
// Inputs: d_follow_m > 0; radius_factor >= 1; front_angle_deg in (0, 90]; lateral_min_m >= 0.
// Returns: the radius in metres, never below d_follow_m.
static inline float fmFrontRadiusM(float d_follow_m, float radius_factor,
                                   float front_angle_deg, float lateral_min_m)
{
  if (d_follow_m < 0.5f)      d_follow_m = 0.5f;
  if (radius_factor < 1.0f)   radius_factor = 1.0f;
  if (front_angle_deg < 1.0f) front_angle_deg = 1.0f;
  if (front_angle_deg > 90.0f) front_angle_deg = 90.0f;

  float r = radius_factor * d_follow_m;
  const float s = sinf(front_angle_deg * BREMOTE_FMS_DEG2RAD);
  if (s > 0.0001f) {
    const float r_lat = lateral_min_m / s;
    if (r_lat > r) r = r_lat;
  }
  if (r < d_follow_m) r = d_follow_m;
  return r;
}

// fmFrontAngleEffDeg - the front angle actually used, after the lateral invariant is enforced THE
// OTHER WAY. fmFrontRadiusM() honours the rider's angle by growing the radius; if the radius is
// ever bounded for some other reason, the ANGLE has to rise instead so the clearance is kept:
// phi_eff = max(phi, asin(lateral_min / r_front)). With the radius computed by the function above
// this is a no-op (asin(lateral_min / r_front) <= phi by construction); it exists so that a future
// radius ceiling cannot silently shrink the clearance. At a 20 m bounded radius it reads 40.5 deg,
// which is Rex's B2 recommendation arrived at from the other side.
static inline float fmFrontAngleEffDeg(float front_angle_deg, float r_front_m, float lateral_min_m)
{
  if (front_angle_deg < 1.0f)  front_angle_deg = 1.0f;
  if (front_angle_deg > 90.0f) front_angle_deg = 90.0f;
  if (r_front_m <= 0.0f || lateral_min_m <= 0.0f) return front_angle_deg;

  float ratio = lateral_min_m / r_front_m;
  if (ratio > 1.0f) ratio = 1.0f;              // clearance unreachable at this radius: 90 deg, i.e. abeam
  const float need_deg = asinf(ratio) / BREMOTE_FMS_DEG2RAD;
  return (need_deg > front_angle_deg) ? need_deg : front_angle_deg;
}

// fmStationLimitDeg - the HARD FLOOR, as a ceiling on |psi|: 180 - the effective front angle.
// This is the one number that makes dead-ahead unreachable. At 45 deg it is 135, at the 35 deg
// minimum it is 145, and it can never be 180 because the angle can never be 0.
static inline float fmStationLimitDeg(float front_angle_eff_deg)
{
  if (front_angle_eff_deg < 1.0f)  front_angle_eff_deg = 1.0f;
  if (front_angle_eff_deg > 90.0f) front_angle_eff_deg = 90.0f;
  return 180.0f - front_angle_eff_deg;
}

// fmClampStationDeg - clamp a station angle into the legal arc [-limit, +limit].
// Deliberately a CLAMP on the psi axis and not a wrap: psi is a bounded coordinate, not a circle,
// and wrapping it is exactly how a station would end up on the far side of dead-ahead.
static inline float fmClampStationDeg(float psi_deg, float front_angle_eff_deg)
{
  const float limit = fmStationLimitDeg(front_angle_eff_deg);
  if (psi_deg >  limit) return  limit;
  if (psi_deg < -limit) return -limit;
  return psi_deg;
}

// fmStationPresetDeg - the station angle a MODE asks for.
//   1 = rear-right  +near_diag      2 = behind 0        3 = rear-left  -near_diag
//   4 = front-right +(180 - phi)    5 = front-left -(180 - phi)
// Any other mode (0, 0xFF, a corrupt byte) returns 0 = directly behind, the safe geometry - the
// same fallback computeFmTarget() has always had.
// front_angle_eff_deg must be the EFFECTIVE angle, so the front presets land exactly on the floor
// and fmClampStationDeg() is then a no-op on them.
static inline float fmStationPresetDeg(uint8_t mode, float near_diag_deg, float front_angle_eff_deg)
{
  if (near_diag_deg < 0.0f)  near_diag_deg = 0.0f;
  if (near_diag_deg > 90.0f) near_diag_deg = 90.0f;   // a rear preset is never ahead of the rider
  const float front = fmStationLimitDeg(front_angle_eff_deg);
  switch (mode) {
    case 1: return +near_diag_deg;
    case 2: return 0.0f;
    case 3: return -near_diag_deg;
    case 4: return +front;
    case 5: return -front;
    default: return 0.0f;
  }
}

// fmStationNearestRearPresetDeg - where a front station RETREATS to when the geometry stops
// supporting it (the rider turns mid-transit, or the course goes invalid). 4 -> rear-right,
// 5 -> rear-left, everything else is already rear. Never the opposite side: swapping sides under
// an abort would walk the station across the rider's wake at the worst possible moment.
static inline float fmStationNearestRearPresetDeg(uint8_t mode, float near_diag_deg)
{
  if (near_diag_deg < 0.0f)  near_diag_deg = 0.0f;
  if (near_diag_deg > 90.0f) near_diag_deg = 90.0f;
  switch (mode) {
    case 1: case 4: return +near_diag_deg;
    case 3: case 5: return -near_diag_deg;
    default:        return 0.0f;
  }
}

// fmStationRadiusM - how far from the rider the station sits, as a function of psi.
//
// A schedule, not a constant: d_follow in the rear-diagonal band, rising linearly to r_front at the
// front limit. The ramp is what carries the buggy outward as the station walks round, so by the time
// the station reaches abeam the radius is already 12-14 m at ordinary tunings.
//
// PG-1 IS ENFORCED HERE. From the abeam line outward the result is floored so that
// r * sin|psi| >= pass_lateral_m - the station's own cross-track offset. The ramp alone does NOT
// guarantee that: a degenerate d_follow of 0.5 m gives a ramp value of 9.4 m at abeam, under a
// 10.1 m pass minimum, and without this floor the station would invite the buggy to pass the rider
// closer than the rope is long.
//
// Inputs:  psi_deg - the station angle (the caller clamps it first; values beyond the limit are
//                    treated as the limit for the floor's divisor).
//          d_follow_m, r_front_m (>= d_follow_m), near_diag_deg - the schedule's shape.
//          pass_lateral_m - the pass minimum (rope + 3 m, raised to min_dist_m if that is larger).
//          front_angle_eff_deg - sets the top of the ramp AND the smallest sine the floor may use.
// Returns: metres, monotonically non-decreasing in |psi|, never below d_follow_m.
static inline float fmStationRadiusM(float psi_deg, float d_follow_m, float r_front_m,
                                     float near_diag_deg, float pass_lateral_m,
                                     float front_angle_eff_deg)
{
  if (d_follow_m < 0.5f) d_follow_m = 0.5f;
  if (r_front_m < d_follow_m) r_front_m = d_follow_m;
  if (near_diag_deg < 0.0f)  near_diag_deg = 0.0f;
  if (near_diag_deg > 90.0f) near_diag_deg = 90.0f;

  const float limit = fmStationLimitDeg(front_angle_eff_deg);
  float a = fabsf(psi_deg);
  if (a > limit) a = limit;

  float a0 = near_diag_deg;                 // flat out to the rear-diagonal band
  if (a0 >= limit) a0 = 0.0f;               // degenerate near_diag: ramp from directly behind

  float r;
  if (a <= a0)            r = d_follow_m;
  else if (a >= limit)    r = r_front_m;
  else                    r = d_follow_m + (r_front_m - d_follow_m) * ((a - a0) / (limit - a0));

  // ---- PG-1: the pass-lateral floor, from abeam outward ----
  // The station's cross-track offset is r * sin|psi|, so requiring it to reach pass_lateral_m is a
  // floor on r of pass_lateral_m / sin|psi|. The divisor is bounded below by sin(phi_eff) because
  // |psi| is clamped to the limit; the max() is the belt for a caller that passes an unclamped
  // angle, and it is what stops the floor running away to infinity as psi approaches 180.
  if (a >= 90.0f) {
    const float s_min = sinf((180.0f - limit) * BREMOTE_FMS_DEG2RAD);   // = sin(phi_eff)
    float s = sinf(a * BREMOTE_FMS_DEG2RAD);
    if (s < s_min) s = s_min;
    if (s > 0.0001f) {
      const float r_floor = pass_lateral_m / s;
      if (r_floor > r) r = r_floor;
    }
  }

  if (r < d_follow_m) r = d_follow_m;
  return r;
}

// fmStationAlongCrossM - the station point in the RIDER'S FRAME.
//   along_m  positive = AHEAD of the rider along their course   = -r * cos(psi)
//   cross_m  positive = to the rider's RIGHT                    = +r * sin(psi)
// Check the signs against the convention: psi = 0 (behind) gives along = -r, cross = 0; psi = +135
// at r = 18.4 gives along = +13.0, cross = +13.0 - the 45 deg front-right station, 13 m ahead and
// 13 m to the right, which is the lateral invariant stated as a coordinate.
static inline void fmStationAlongCrossM(float psi_deg, float r_m, float *along_m, float *cross_m)
{
  const float rad = psi_deg * BREMOTE_FMS_DEG2RAD;
  if (along_m) *along_m = -r_m * cosf(rad);
  if (cross_m) *cross_m =  r_m * sinf(rad);
}

// fmBuggyAlongCrossM - the BUGGY in the rider's frame, from quantities the RX already has.
// Inputs: course_deg - the rider's course over ground (degrees clockwise from North);
//         bearing_rider_to_buggy_deg - TinyGPSPlus::courseTo(rider, buggy);
//         dist_m - the separation.
// Outputs: along_m positive = the buggy is AHEAD of the rider; cross_m positive = to the rider's
//          RIGHT. Same frame and same signs as fmStationAlongCrossM(), which is what makes PG-3's
//          endpoint comparison legitimate.
static inline void fmBuggyAlongCrossM(float course_deg, float bearing_rider_to_buggy_deg,
                                      float dist_m, float *along_m, float *cross_m)
{
  const float a = fmsWrap180(bearing_rider_to_buggy_deg - course_deg) * BREMOTE_FMS_DEG2RAD;
  if (along_m) *along_m = dist_m * cosf(a);
  if (cross_m) *cross_m = dist_m * sinf(a);
}

// fmMeasuredStationDeg - G-4: the station angle the buggy is ACTUALLY at right now, so an
// engagement starts from where the buggy is rather than from a remembered side.
// Inverts the convention: bearing = course + 180 - psi, so psi = course + 180 - bearing.
static inline float fmMeasuredStationDeg(float course_deg, float bearing_rider_to_buggy_deg)
{
  return fmsWrap180(course_deg + 180.0f - bearing_rider_to_buggy_deg);
}

// fmStationTargetCeilingDeg - PG-2, THE SLEW GATE, as a ceiling on |psi| rather than a freeze.
//
// A rear target imposes no ceiling beyond itself. A FRONT target lets psi walk freely out to the
// abeam line - which is what carries the buggy wide, because the radius schedule grows along the
// way - and no further until the buggy's OWN measured cross-track offset has reached pass_lateral_m
// ON THE SIDE THE STATION IS GOING TO. Wrong side counts as not wide: a buggy 12 m to the left does
// not authorise a station 13 m to the right.
//
// Returned as a magnitude; the caller applies the sign of psi_target.
static inline float fmStationTargetCeilingDeg(float psi_target_deg, float buggy_cross_m,
                                              float pass_lateral_m, float station_limit_deg)
{
  const float want = fabsf(psi_target_deg);
  if (want <= 90.0f) return want;                       // rear target: nothing to gate

  const bool wide_on_target_side = (psi_target_deg > 0.0f)
                                     ? (buggy_cross_m >=  pass_lateral_m)
                                     : (buggy_cross_m <= -pass_lateral_m);
  if (!wide_on_target_side) return 90.0f;               // hold the station abeam and wait

  return (want < station_limit_deg) ? want : station_limit_deg;
}

// fmStationSlewStep - move psi_live one tick toward psi_target at no more than rate_deg_s.
//
// NO WRAPPING, and that is the point: psi is a bounded coordinate on a LINE, so interpolating it
// from +135 to -135 necessarily passes through 0 (behind the rider) and never through 180 (dead
// ahead). A side change therefore walks round the back, which is both the shorter safe path and the
// only legal one.
//
// rate_deg_s must be no greater than the buggy's measured yaw rate at the align cap: a station that
// walks faster than the buggy can turn makes the buggy lag, then overshoot, then demand exactly the
// correction input the whole design exists to avoid.
static inline float fmStationSlewStep(float psi_live_deg, float psi_target_deg,
                                      float rate_deg_s, float dt_s)
{
  if (rate_deg_s <= 0.0f || dt_s <= 0.0f) return psi_live_deg;
  const float step = rate_deg_s * dt_s;
  const float d    = psi_target_deg - psi_live_deg;
  if (fabsf(d) <= step) return psi_target_deg;
  return psi_live_deg + ((d > 0.0f) ? step : -step);
}

// fmAimClearsRiderLine - PG-3, stated as one test.
//
// Cross-track offset is affine along a straight segment, so its extreme values over the segment are
// its endpoint values. If both endpoints sit at least pass_lateral_m from the rider's course line
// ON THE SAME SIDE, every point between them does too - so the aim line never reaches the rider's
// line, let alone the rider.
//
// Returns true when the buggy->station aim line is proven clear. A false here is not by itself a
// hazard (the buggy may simply still be behind with a rear station); it is the precondition the
// front-station path must hold, which is why the controller only consults it while the station is
// ahead and routes a false into fmAimOutwardNeeded()'s outward escape.
static inline bool fmAimClearsRiderLine(float buggy_cross_m, float station_cross_m,
                                        float pass_lateral_m)
{
  if (station_cross_m >= pass_lateral_m && buggy_cross_m >= pass_lateral_m) return true;
  if (station_cross_m <= -pass_lateral_m && buggy_cross_m <= -pass_lateral_m) return true;
  return false;
}

// fmAimOutwardNeeded - PG-4: the station is ahead of the rider but the buggy is no longer wide
// enough on that side to make PG-3's argument, so the aim must be pushed OUTWARD instead of across.
// Independent of whether the buggy is ahead or behind: the affine argument never needed the
// along-track term, so neither does its failure test. That makes this condition strictly broader -
// i.e. more conservative - than a version gated on the buggy being ahead.
static inline bool fmAimOutwardNeeded(float psi_live_deg, float buggy_cross_m,
                                      float station_cross_m, float pass_lateral_m)
{
  if (!fmStationIsFront(psi_live_deg)) return false;
  return !fmAimClearsRiderLine(buggy_cross_m, station_cross_m, pass_lateral_m);
}

// fmOutwardAimLateralM - how far from the rider's course line the escape waypoint is placed, as a
// SIGNED cross-track offset on the station's side. At least pass_lateral_m, and at least the
// buggy's own offset plus clear_m, so the waypoint is always genuinely further out than the buggy
// is (the buggy turns away from the line, never along it) and never so close to the buggy that the
// bearing to it is noise - the G-5 hazard kFmThetaMinSepM names.
static inline float fmOutwardAimLateralM(float psi_live_deg, float buggy_cross_m,
                                         float pass_lateral_m, float clear_m)
{
  if (clear_m < 0.0f) clear_m = 0.0f;
  const bool right = (psi_live_deg > 0.0f);
  const float out  = fabsf(buggy_cross_m) + clear_m;
  float lat = (out > pass_lateral_m) ? out : pass_lateral_m;
  return right ? lat : -lat;
}

// V2.5-Evo - 2026-10-06 - fmRearSnap - the rear-station step test (moved here from RTMState.ino so the
// host test exercises the shipped predicate, not a copy of it).
// True when the effective mode is a REAR one (1-3) AND the live angle is already inside the rear band
// |psi| <= near_diag (clamped 0-90 exactly as fmStationPresetDeg() clamps it): the live angle is then
// set straight to its target in one tick, the SW36 behaviour. False for a station still outside the
// band - one coming home from ahead of abeam - which keeps walking so it never steps across the line.
static inline bool fmRearSnap(uint8_t m_eff, float psi_live_deg, float near_diag_deg)
{
  if (near_diag_deg < 0.0f)  near_diag_deg = 0.0f;
  if (near_diag_deg > 90.0f) near_diag_deg = 90.0f;
  return (m_eff >= 1 && m_eff <= 3) && (fabsf(psi_live_deg) <= near_diag_deg);
}

// V2.5-Evo - 2026-10-06 - fmFrontRetreatSide - the side test PG-4 was missing (audit H-1).
// PG-3's endpoint argument assumes the buggy is on the STATION'S side of the rider's line. A rider who
// turns toward a settled front station (or a jibe) breaks that: the buggy is now on the OTHER side, and
// both the outward escape and the abeam waiting waypoint - always placed on the station's side - would
// aim it across the rider's line, ahead of him.
// Returns the side the BUGGY is on (+1 right, -1 left) when the station is ahead of abeam, or holding
// abeam while a front station is wanted, AND the buggy is measured strictly on the other side of the
// rider's line. Returns 0 otherwise (no retreat). A buggy exactly on the line (cross == 0) counts as the
// station's side, where PG-4's outward escape already steers it off the line on that side.
// Never true for a rear-only engagement: there |psi_live| <= 90 and no front station is wanted.
static inline int fmFrontRetreatSide(float psi_live_deg, bool front_wanted, float buggy_cross_m)
{
  const float a = fabsf(psi_live_deg);
  const bool at_or_ahead = (a > 90.0f) || (front_wanted && a >= 90.0f);
  if (!at_or_ahead) return 0;
  if (psi_live_deg > 0.0f && buggy_cross_m < 0.0f) return -1;
  if (psi_live_deg < 0.0f && buggy_cross_m > 0.0f) return +1;
  return 0;
}

// V2.5-Evo - 2026-10-06 - fmEngageSeedDeg - the live station angle at an ACTIVE edge (a first squeeze,
// or HOLD -> ACTIVE after a release - which is how the rider normally changes mode: stop, float, switch,
// squeeze). Audit M-8 / owner ruling R-1b.
//   Modes 4/5: G-4 unchanged - the measured angle, clamped into the rear half (+/-90), so an engagement
//              can never BEGIN with a front station already granted.
//   Modes 1-3: 0, as SW36 (the first tick snaps to the preset) - UNLESS the buggy is measured ahead of
//              abeam (|psi_meas| > 90). Then +/-90 on the BUGGY'S side, outside the rear band, so the
//              station walks home on that side instead of stepping straight to a rear preset (possibly
//              on the far side) with the buggy still ahead of the rider.
//   No course, or any other mode: 0, directly behind.
static inline float fmEngageSeedDeg(uint8_t mode, bool course_valid, float psi_meas_deg)
{
  if (!course_valid) return 0.0f;
  if (mode == 4 || mode == 5) {
    if (psi_meas_deg >  90.0f) return  90.0f;
    if (psi_meas_deg < -90.0f) return -90.0f;
    return psi_meas_deg;
  }
  if (mode >= 1 && mode <= 3) {
    if (psi_meas_deg >  90.0f) return  90.0f;
    if (psi_meas_deg < -90.0f) return -90.0f;
    return 0.0f;
  }
  return 0.0f;
}

// V2.5-Evo - 2026-10-06 - fmRetreatSeedDeg - where the live angle is RE-SEEDED when H-1 retreats: the
// buggy's MEASURED station angle, forced onto the buggy's side (so a rounding disagreement between the
// angle and the cross-track sign can never put it on the far side) and clamped to the hard floor. The
// station then walks home to the rear preset on that side; every point of that walk is on the buggy's
// side, so the aim never has to cross the rider's line to reach it.
static inline float fmRetreatSeedDeg(int side, float psi_meas_deg, float station_limit_deg)
{
  float a = fabsf(psi_meas_deg);
  if (a > station_limit_deg) a = station_limit_deg;
  return (side < 0) ? -a : a;
}

// fmFadeBypass - the GOVERNOR-2 convergence-fade bypass, and invariant 2 of the plan: it is a
// GEOMETRY PREDICATE, never a mode flag. There is no `mode == 4 || mode == 5` term anywhere in it.
//
// The fade exists because past 90 deg the rider is closing the gap themselves, so the buggy needs
// less of its own speed - fading to zero head-on. That is right for every REAR station. It is wrong
// for exactly one situation: the buggy is lawfully ahead of the rider, wide of their line, holding a
// front station - where fading to zero would park it in front of an approaching rider.
//
// All four terms, and all four measured, not assumed:
//   course_valid    - there is a trustworthy rider course at all (below kFmCourseValidSpeedKmh
//                     there is no "ahead" and the degraded hold-station geometry runs instead).
//   |psi_live| > 90 - the COMMANDED station is ahead of the rider.
//   along_m > 0     - the buggy MEASURABLY is ahead of the rider, not merely commanded to be.
//   |cross_m| >= pass_lateral_m - and it is passing BESIDE the rider's line, by at least the rope
//                     plus 3 m. This is the term that makes the bypass a pass and not an overtake.
// Any one false -> the ordinary fade applies, with no dwell and no memory.
static inline bool fmFadeBypass(bool course_valid, float psi_live_deg,
                                float along_m, float cross_m, float pass_lateral_m)
{
  if (!course_valid) return false;
  if (fabsf(psi_live_deg) <= 90.0f) return false;
  if (along_m <= 0.0f) return false;
  return fabsf(cross_m) >= pass_lateral_m;
}

// fmStationAlongGovKmh - the along-track speed term for a FRONT station.
//
// GOVERNOR-2's gap term answers "how far outside my station am I?" with a RADIAL distance, which is
// the right question behind the rider and the wrong one ahead of them: a station 18.4 m out reads as
// 9 m of permanent gap at a 9 m follow distance and would grant a standing closing allowance for
// ever. Ahead of the rider the quantity that matters is signed ALONG-TRACK error - behind station,
// speed up; ahead of station, slow down - which is what this returns.
//
// Inputs: station_along_m, buggy_along_m in the rider's frame; gain_kmh_per_m; max_kmh (the same
//         ceiling the radial gap term uses).
// Returns: a SIGNED km/h adjustment, clamped to +/- max_kmh. Negative is the whole point: it is what
//          makes a buggy that has run too far ahead give throttle back.
static inline float fmStationAlongGovKmh(float station_along_m, float buggy_along_m,
                                         float gain_kmh_per_m, float max_kmh)
{
  float v = (station_along_m - buggy_along_m) * gain_kmh_per_m;
  if (v >  max_kmh) v =  max_kmh;
  if (v < -max_kmh) v = -max_kmh;
  return v;
}

#endif
