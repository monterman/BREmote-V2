// V2.5-Evo - 2026-10-06 - AUDIT M-20 + L-25: fmFrontSideFloorM() - the front side floor is now
//   max(13, pass minimum, min_dist_m + 2), resolved once here for every RX site. No change for min_dist_m <= 11 m.
//   L-26: the fmShieldMake input comment no longer says the half-angle is a fixed 30 deg.
// V2.5-Evo - 2026-10-06 - COMMENTS ONLY (audit L-19): the derived front angle is 35-45 deg (was written 35..80); the
//   shield block no longer claims a settled F4/F5 station is "never in the rider's likely path" - it is 5 deg outside
//   the cone at the 35 deg cap, which is why the hold band and the 3 s latch exist.
// V2.5-Evo - 2026-10-06 - AUDIT M-19: fmNoCourseAbortLatches() - F4/F5 selected while stopped is no longer abandoned on
//   the first tick: no latch while |psi_live| <= 90 and no transit has started.
// V2.5-Evo - 2026-10-06 - SETTLED-ESCAPE TIMER: fmShieldSettledEscapeExpired() - a shield escape on a SETTLED F4/F5
//   station lasting more than 3 s latches the abort (the RX owns the state and the latch).
// V2.5-Evo - 2026-10-06 - SHIELD SIZING (audits M-16 b/c, M-15, M-18): fmShieldHalfAngleDeg() + the PROVISIONAL
//   kFmShieldHalfAngle* knot table (the cone half-angle by rider speed, 30 deg falling to 15 deg, capped at 30 by owner
//   ruling, never narrower than the 31-session data); fmShieldHoldBandM() (the escape hold = min(2 m, half the
//   settled front station's clearance from the cone)); fmShieldConeOn() (a 12 / 10 km/h Schmitt on the cone);
//   fmShieldExtendForLag() + FmShield::circle_len_m (the circle becomes a capsule reaching the real rider when the
//   lag anchor is capped short of him); fmShieldDecide() takes the hold separately from the 2 m side band.
// V2.5-Evo - 2026-10-06 - THE RIDER SHIELD (audit H-2) + SIDE BAND (audit H-3): new FmShield / fmShieldMake /
//   fmShieldSegmentHits / fmShieldHalfWidthM / fmShieldClipLookaheadM / fmShieldDecide - a circle round the rider
//   always, plus a cone ahead of him along his course while he is foiling - and fmBuggySideWithBand(); the H-1 side
//   test fmFrontRetreatSide() takes a band_m (a buggy within band_m of the rider's line keeps the committed side).
//   Comment fixes: the derived front angle is now 35-45 deg (audit M-12), the retreat side (audit L-9), and the
//   stale angle + radius wording (audit L-13). Pure arithmetic, no Arduino dependency, host-tested.
// V2.5-Evo - 2026-10-06 - FRONT STATIONS BY OFFSET: fmFrontStationGeom() / fmFrontAheadExtraM() replace
//   fmFrontRadiusM() / fmFrontAngleEffDeg(). F4/F5 sit `side` (the lateral floor, exactly) to the side
//   and d_follow + fm_front_ahead_extra_m ahead; phi and r are derived. phi stays in 35..80 deg.
//   [CORRECTED 2026-10-06, audits M-12 / L-19: 35..45 deg - the ceiling went 80 -> 45 so ahead is never less than side.]
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
//     4 = front-right  psi = +(180 - phi)    phi = the angle DERIVED from the side and ahead offsets
//     5 = front-left   psi = -(180 - phi)    (fmFrontStationGeom, 35-45 deg; +/-135 when ahead == side)
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
// ---- THE PASS-GEOMETRY INVARIANTS (PG-1..PG-5) - what an audit can check ----
//   PG-1  The station's own CROSS-TRACK offset from the rider's course line is at least
//         pass_lateral_m whenever the station is ahead of the rider (|psi| >= 90). Enforced by the
//         radius floor inside fmStationRadiusM(), not by the preset table, so it holds at every
//         tuning including degenerate ones.
//   PG-2  psi may not exceed 90 deg in magnitude unless the BUGGY'S OWN measured cross-track offset
//         is at least pass_lateral_m AND ON THE SAME SIDE as the station it is heading for. This is
//         fmStationTargetCeilingDeg(): the station does not move ahead of the rider until the buggy
//         is physically wide of the rider's line. It is the audit's two-waypoint pass, expressed as a
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
//   SHIELD V2.5-Evo - 2026-10-06 (audit H-2, the owner's "bubble" rule). PG-1..PG-4 protect the rider's
//         LINE only while a station is ahead of abeam. The shield protects the rider himself at every
//         station angle the controller walks (every non-snapped station): a circle round him always,
//         plus a cone ahead of him along his course, speed x horizon long, while he is foiling. The
//         buggy's aim line (buggy -> aim point, whatever the aim point is this tick) must not pass
//         through it; if it would, the controller re-seeds the station onto the buggy's own side and
//         steers outward on that side until the line clears (fmShieldDecide). PG-1..PG-5 are unchanged.
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

// V2.5-Evo - 2026-10-06 - fmFrontAheadExtraM / fmFrontStationGeom REPLACE fmFrontRadiusM and
// fmFrontAngleEffDeg. OWNER RULE: a front station is placed by two OFFSETS from the rider, not by an
// angle and a radius - SIDEWAYS = the lateral clearance floor exactly (13 m, or the pass minimum if
// that is larger), never more; AHEAD = d_follow + fm_front_ahead_extra_m. The angle and the radius
// the rest of this header works in are DERIVED from those two numbers.

// fmFrontAheadExtraM - resolve the stored fm_front_ahead_extra_m (whole metres) to the metres used.
//   0                   -> default_m (7). Every board in the field reads 0 in these bytes.
//   1 .. min_m-1        -> min_m (4): the nearest legal value.
//   above max_m (10)    -> default_m. Not "max": the only way such a value can be here is a blob
//                          written while these two bytes were fm_front_angle_deg (0 or 35-80 deg),
//                          and an angle is not a distance - it reads as "use the default".
// Inputs: the raw u16 and the three bounds (the controller supplies them). Returns metres.
static inline float fmFrontAheadExtraM(uint16_t stored_m, float default_m, float min_m, float max_m)
{
  if (stored_m == 0) return default_m;
  const float x = (float)stored_m;
  if (x > max_m) return default_m;
  if (x < min_m) return min_m;
  return x;
}

// fmFrontStationGeom - the FRONT station from its two offsets, in the rider's frame.
//   side  = side_m, EXACTLY. The caller passes the lateral clearance floor (13 m carve + GPS, raised
//           to the pass minimum when min_dist_m makes that larger, and since audit M-20 to min_dist_m + 2 m:
//           fmFrontSideFloorM). Never narrowed; widened only in
//           the one case named below (a follow distance over 22.7 m).
//   ahead = d_follow_m + ahead_extra_m, then held inside the angle band (below).
//   phi   = atan(side / ahead), degrees off dead ahead, measured at the rider.
//   r     = sqrt(side^2 + ahead^2).
// So r * sin(phi) == side and r * cos(phi) == ahead: the station point fmStationAlongCrossM() makes
// at psi = 180 - phi, radius r is exactly `ahead` ahead and `side` to the side.
// Example: d_follow 9 + extra 7 -> ahead 16, side 13 -> phi 39.1 deg, r 20.6 m.
//
// THE ANGLE BAND, AND WHY IT IS HELD BY MOVING `AHEAD`, NOT `SIDE`. The owner's rule fixes side, so
// the only free number is ahead:
//   phi >= phi_min_deg (35, the owner's minimum): ahead <= side / tan(35) = 18.6 m at side 13. A long
//          follow distance or a large extra is capped there. Two reasons. (1) The "dead ahead is
//          unreachable" floor (|psi| <= 180 - phi) stays where the owner set it. (2) An error of theta
//          in the rider's course estimate swings the station about the rider and moves its true
//          side offset by about ahead x sin(theta) - so the clearance a course error can eat grows
//          with `ahead`, not with `side`. Holding ahead <= 1.43 x side keeps that ratio exactly where
//          the old 35 deg minimum held it (10 deg of course error costs at most 3.2 m of the 13).
//   phi <= phi_max_deg: ahead >= side / tan(phi_max), so a front station is always genuinely ahead
//          of abeam. V2.5-Evo - 2026-10-06 - audit M-12: the RX passes 45 (was 80), so ahead >= side -
//          13 m at side 13. WHY: the 13 m side floor is a LATERAL argument, and a rider carving 90 deg
//          toward the station on an 8 m radius also moves about 8 m FORWARD, so against a stopped buggy
//          the clearance is about sqrt((ahead - 8)^2 + 5^2): 5.4 m at 10 m ahead, 7.1 m at 13. At d_follow
//          6 + extra 4 (10 m) the old 80 deg ceiling allowed 10 m ahead; 45 raises it to 13.
//
// THE ONE CASE WHERE SIDE IS NOT EXACTLY THE FLOOR. If d_follow is longer than the capped radius
// (side / sin(35) = 22.7 m, about 74 ft, at side 13), the radius is raised to d_follow, because the
// station radius schedule (fmStationRadiusM) never puts any station inside the follow distance and
// is monotone from the rear station out to the front one. The angle stays at 35, so side and ahead
// both grow in proportion: side > the floor, the SAFE direction. This function returns those real
// numbers (*side_out_m, *ahead_m, *r_m are the station fmStationRadiusM will actually produce), so
// nothing prints a 13 m that is not true. The owner's 9 m follow distance is nowhere near it.
// Inputs: d_follow_m (floored 0.5), ahead_extra_m (floored 0), side_m (floored 0.5),
//         phi_min_deg / phi_max_deg (clamped to 1..89, max >= min).
// Outputs (any may be null): *ahead_m, *phi_deg, *r_m, *side_out_m. No side effects.
static inline void fmFrontStationGeom(float d_follow_m, float ahead_extra_m, float side_m,
                                      float phi_min_deg, float phi_max_deg,
                                      float *ahead_m, float *phi_deg, float *r_m, float *side_out_m)
{
  if (d_follow_m < 0.5f)    d_follow_m = 0.5f;
  if (ahead_extra_m < 0.0f) ahead_extra_m = 0.0f;
  if (side_m < 0.5f)        side_m = 0.5f;
  if (phi_min_deg < 1.0f)   phi_min_deg = 1.0f;
  if (phi_min_deg > 89.0f)  phi_min_deg = 89.0f;
  if (phi_max_deg < phi_min_deg) phi_max_deg = phi_min_deg;
  if (phi_max_deg > 89.0f)  phi_max_deg = 89.0f;

  float ahead = d_follow_m + ahead_extra_m;
  const float ahead_hi = side_m / tanf(phi_min_deg * BREMOTE_FMS_DEG2RAD);   // phi >= phi_min
  const float ahead_lo = side_m / tanf(phi_max_deg * BREMOTE_FMS_DEG2RAD);   // phi <= phi_max
  if (ahead > ahead_hi) ahead = ahead_hi;
  if (ahead < ahead_lo) ahead = ahead_lo;

  float phi = atan2f(side_m, ahead) / BREMOTE_FMS_DEG2RAD;
  if (phi < phi_min_deg) phi = phi_min_deg;      // float rounding at the cap only (< 1e-4 deg)
  if (phi > phi_max_deg) phi = phi_max_deg;

  float r    = sqrtf(side_m * side_m + ahead * ahead);
  float side = side_m;
  if (r < d_follow_m) {                           // only past d_follow 22.7 m at side 13 (see above)
    r     = d_follow_m;
    ahead = r * cosf(phi * BREMOTE_FMS_DEG2RAD);
    side  = r * sinf(phi * BREMOTE_FMS_DEG2RAD);
  }

  if (ahead_m)    *ahead_m    = ahead;
  if (phi_deg)    *phi_deg    = phi;
  if (r_m)        *r_m        = r;
  if (side_out_m) *side_out_m = side;
}

// V2.5-Evo - 2026-10-06 - audit M-20 + L-25: fmFrontSideFloorM - THE FRONT STATION'S SIDE FLOOR, resolved
// in ONE place (it was written out by hand at three sites in the RX).
//     side floor = max(lateral_min_m, pass_lateral_m, min_dist_m + margin_m)
// WHY THE THIRD TERM. Before it, the floor was max(13 m, pass minimum), and the pass minimum is min_dist_m
// once that is over 10.1 m. So with min_dist_m above 11 m a settled F4/F5 station sat only 0-2 m outside
// the min_dist_m circle round the rider - and EXACTLY on it from 13 m up. That circle is two things at once:
// the shield's circle (capsule) and the min_dist_m hard-stop radius (cap 0). A station on its edge makes
// shield escapes start and stop, and cap 0 flick on and off, ahead of a fast rider. Adding the margin
// (the RX passes kFmSideHysteresisM, 2 m - the same band that holds an escape) keeps the station at least
// that far outside the circle at every min_dist_m. It only ever WIDENS the side (the safe direction).
// For min_dist_m up to lateral_min_m - margin_m (11 m) nothing changes: the owner's 4 m gives 13 m, the
// same float, bit for bit.
// Inputs: lateral_min_m (13, carve + GPS), pass_lateral_m (the pass minimum, already raised to min_dist_m),
//         min_dist_m (usrConf.min_dist_m), margin_m (2). Returns metres. No side effects.
static inline float fmFrontSideFloorM(float lateral_min_m, float pass_lateral_m, float min_dist_m,
                                      float margin_m)
{
  float side = lateral_min_m;
  if (side < pass_lateral_m) side = pass_lateral_m;
  const float clear = min_dist_m + margin_m;
  if (side < clear) side = clear;
  return side;
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
// 5 -> rear-left, everything else is already rear. This function never picks the opposite side:
// swapping sides under an abort would walk the station across the rider's wake at the worst moment.
// V2.5-Evo - 2026-10-06 - audit L-9: the controller DOES pick the opposite side in one case - when the
// H-1 retreat or the shield finds the BUGGY measurably on the other side of the rider's line, it goes
// to the rear preset on the buggy's side (fm_front_retreat_side), because the buggy is already there.
// That choice is made in RTMState.ino, not here.
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
// front limit (r_front = hypot(side, ahead) from fmFrontStationGeom - V2.5-Evo - 2026-10-06, audit
// L-13: it is derived from the two offsets, not set as a radius). The ramp is what carries the buggy
// outward as the station walks round, so by the time the station reaches abeam the radius is already
// about 12-15 m at ordinary tunings.
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
// V2.5-Evo - 2026-10-06 - audit H-3: A SCHMITT BAND, NOT A STRICT SIGN TEST. The strict test flipped on
// about +/-1 m of cross-track noise; every flip re-seeded the station to the other side, and the buggy
// held ahead of the rider near his line with bang-bang steering. Now the buggy must be more than band_m
// on the OTHER side before the retreat fires; inside +/-band_m the committed side (the station's side)
// stands, and PG-4's outward escape steers off the line on that side. Once retreated, the station is on
// the new side, so flipping back needs the buggy more than band_m on the opposite side again. The cost
// is at most band_m of MEASURED line crossing, inside the GPS error the design already assumes.
// Inputs: band_m (the RX passes kFmSideHysteresisM, 2 m; negative is read as 0 = the old strict test).
static inline int fmFrontRetreatSide(float psi_live_deg, bool front_wanted, float buggy_cross_m,
                                     float band_m)
{
  if (band_m < 0.0f) band_m = 0.0f;
  const float a = fabsf(psi_live_deg);
  const bool at_or_ahead = (a > 90.0f) || (front_wanted && a >= 90.0f);
  if (!at_or_ahead) return 0;
  if (psi_live_deg > 0.0f && buggy_cross_m < -band_m) return -1;
  if (psi_live_deg < 0.0f && buggy_cross_m >  band_m) return +1;
  return 0;
}

// V2.5-Evo - 2026-10-06 - fmBuggySideWithBand - which side of the rider's line the BUGGY counts as being on,
// with the same Schmitt band as fmFrontRetreatSide (audit H-3), for the shield's go-around.
//   buggy more than band_m right -> +1; more than band_m left -> -1;
//   inside the band -> the COMMITTED side (the side the station is on, sign of psi_live), so cross-track
//   noise near the line can never flip the go-around from one side to the other;
//   inside the band with no committed side (psi_live exactly 0, directly behind, and no escape standing)
//   -> the buggy's own sign, 0 counting as right. fmShieldDecide then remembers that side for as long as
//   the escape stands, which commits it.
// Inputs: committed_side (+1, -1 or 0), buggy_cross_m (rider frame, + = right), band_m (>= 0).
static inline int fmBuggySideWithBand(int committed_side, float buggy_cross_m, float band_m)
{
  if (band_m < 0.0f) band_m = 0.0f;
  if (buggy_cross_m >  band_m) return +1;
  if (buggy_cross_m < -band_m) return -1;
  if (committed_side > 0) return +1;
  if (committed_side < 0) return -1;
  return (buggy_cross_m < 0.0f) ? -1 : +1;
}

// ==========================================================================================
// V2.5-Evo - 2026-10-06 - THE RIDER SHIELD (audit H-2, the owner's "bubble" rule)
//
// WHAT IT IS. A region around the rider that the buggy's AIM LINE (the straight segment from the buggy
// to the point it is steering at this tick) may not pass through:
//   - a CIRCLE round the rider, always: radius = the larger of circle_min_m (3 m) and min_dist_m, but
//     never more than the follow distance (the rider's own rear station must stay outside it);
//   - a CONE ahead of the rider, along his course, only while he is foiling (speed >= min_speed_kmh,
//     12 km/h): depth = speed x horizon_s (5 s), half-angle half_angle_deg. V2.5-Evo - 2026-10-06 - audit
//     M-16 / M-15 / M-18: the RX passes the SPEED-SCALED half-angle (fmShieldHalfAngleDeg, 30 deg falling
//     to 15 deg), switches the cone with a 12 / 10 km/h Schmitt (fmShieldConeOn), and turns the circle into
//     a capsule when the lag anchor falls short of the rider (fmShieldExtendForLag).
// Below the foiling speed only the circle applies: the rider is floating or slow and watching, and the
// buggy may reposition freely around him.
//
// WHY A CONE AND NOT A STRIP. 31 foil sessions (24,128 foiling seconds, 1 Hz wrist GPS) show a straight-
// line projection is only good for about 2 s; at 5 s the rider is 13 m off it one time in ten (p90). As
// an ANGLE from the rider that p90 miss is nearly constant beyond 2-3 s - 28 deg at 15-20 km/h - so a
// cone of about 30 deg holds where he really goes; a strip does not.
//
// THE CONE IS A TRIANGLE, deliberately. Apex at the rider, the base at `depth` along his course, sides
// at +/-half-angle. That contains the circular sector of the same radius (the triangle reaches
// depth / cos(half-angle) along its edges), so it errs larger, never smaller, and a triangle is a convex
// polygon whose segment test is exact (three half-planes, Liang-Barsky clipping below).
//
// THE FRAME. Every coordinate here is in the rider's frame: along = ahead-positive along his course,
// cross = right-positive, origin = the rider. The RX uses its lag-compensated rider estimate (the
// filtered position pushed forward by v x tau, the same anchor every station is placed around), so the
// shield and the stations agree on where the rider is.
//
// Front stations sit OUTSIDE the cone by design - but V2.5-Evo - 2026-10-06 - audit L-19: NOT BY MUCH, and
// "never in the rider's likely path" overstated it. At the 35 deg cap and a 30 deg cone the station is 5 deg
// outside, 1.98 m from the cone's side at r 22.7 m; ordinary wobble (median 6 deg/s, p90 21) and a +/-10 deg
// course error both reach that. What actually holds now: the cone narrows with speed (fmShieldHalfAngleDeg),
// the escape hold is at most half the station's clearance (fmShieldHoldBandM), so an escape near a settled
// station can always end, and the RX latches the F4/F5 abort if one stands for more than 3 s. The shield
// governs the TRANSIT, the walks home, and the steering lookahead.
// ==========================================================================================
struct FmShield {
  float circle_r_m;   // radius of the always-on circle round the rider, metres
  float cone_len_m;   // depth of the cone ahead of the rider along his course, metres; 0 = cone off
  float cone_tan;     // tan(half-angle) of the cone
  // V2.5-Evo - 2026-10-06 - audit M-18: the circle is swept this far AHEAD along the course, making it a
  // capsule (a stadium: every point within circle_r_m of the segment from the anchor to circle_len_m
  // ahead of it). 0 = the plain circle. Set only by fmShieldExtendForLag(); fmShieldMake() leaves it 0.
  float circle_len_m;
};

// ==========================================================================================
// V2.5-Evo - 2026-10-06 - audit M-16 (b): THE SPEED-SCALED CONE HALF-ANGLE. 🔴 PROVISIONAL.
//
// WHAT. The half-angle of the shield's cone as a function of the rider's speed, instead of one fixed
// 30 deg. From the same 31 foil sessions (24,128 foiling seconds, 1 Hz wrist GPS): the p90 sideways
// miss of a straight-line projection at 5 s, expressed as an angle from the rider, by speed band -
//      10-15 km/h: 40 deg     15-20 km/h: 28 deg     20-25 km/h: 21 deg     25+ km/h: 15 deg
// The faster the rider, the straighter he goes, so the cone can be narrower - which is what gives a
// settled F4/F5 station (35-45 deg off dead ahead) room outside it.
//
// THE CURVE (the knot table below, linear between knots, flat beyond both ends):
//   km/h  12    15    17.5   20    22.5   25    27.5 and up
//   deg   30    30    28     28    21     21    15
// - CONTINUOUS AND MONOTONE (never increases with speed, no steps), so the lookahead clip and the aim
//   built from it move continuously as the rider's speed changes.
// - NEVER NARROWER THAN THE DATA AT ANY SPEED. Each band's value holds across its WHOLE band, not only at
//   its centre: 28 deg all the way to 20 km/h and 21 deg all the way to 25 km/h, and the drop to the next
//   band happens inside that next band (whose own value is lower). Linear interpolation straight between
//   band centres (17.5 -> 22.5) would dip under 28 deg at 18-20 km/h; this curve does not. The anchors
//   asked for - 30 at 15, 28 at 17.5, 21 at 22.5, 15 at 27.5 - are all on it exactly.
// - CAPPED AT 30 deg (owner ruling 2026-10-06): the 10-15 km/h data says 40 deg, but a cone that wide at
//   the takeoff edge would swallow the 35 deg front stations. The 3 m circle covers the slow rider.
//   Below 15 km/h the curve is therefore flat at 30 deg, which is also the low-end clamp at 12 km/h
//   (the cone only exists from 12 km/h, or 10 km/h once on - see fmShieldConeOn).
// Revisit with activity-tagged sessions and 10 Hz vest data (both expected to narrow it further).
// ==========================================================================================
static const float kFmShieldHalfAngleMaxDeg = 30.0f;   // deg; owner cap - never wider than this
static const int   kFmShieldHalfAngleKnots  = 7;
static const float kFmShieldHalfAngleKnotKmh[kFmShieldHalfAngleKnots] = { 12.0f, 15.0f, 17.5f, 20.0f, 22.5f, 25.0f, 27.5f };
static const float kFmShieldHalfAngleKnotDeg[kFmShieldHalfAngleKnots] = { 30.0f, 30.0f, 28.0f, 28.0f, 21.0f, 21.0f, 15.0f };

// fmShieldHalfAngleDeg - the cone half-angle for this rider speed, from the table above.
// Input: rider speed, km/h (any value; non-finite reads as the slow end). Output: degrees, in
// [15, 30], continuous and non-increasing in speed. No side effects.
static inline float fmShieldHalfAngleDeg(float speed_kmh)
{
  const int n = kFmShieldHalfAngleKnots;
  float h;
  if (!(speed_kmh > kFmShieldHalfAngleKnotKmh[0])) {
    h = kFmShieldHalfAngleKnotDeg[0];
  } else if (speed_kmh >= kFmShieldHalfAngleKnotKmh[n - 1]) {
    h = kFmShieldHalfAngleKnotDeg[n - 1];
  } else {
    h = kFmShieldHalfAngleKnotDeg[n - 1];
    for (int i = 1; i < n; i++) {
      if (speed_kmh <= kFmShieldHalfAngleKnotKmh[i]) {
        const float x0 = kFmShieldHalfAngleKnotKmh[i - 1], x1 = kFmShieldHalfAngleKnotKmh[i];
        const float y0 = kFmShieldHalfAngleKnotDeg[i - 1], y1 = kFmShieldHalfAngleKnotDeg[i];
        h = y0 + (y1 - y0) * ((speed_kmh - x0) / (x1 - x0));
        break;
      }
    }
  }
  if (h > kFmShieldHalfAngleMaxDeg) h = kFmShieldHalfAngleMaxDeg;
  return h;
}

// V2.5-Evo - 2026-10-06 - audit M-15: fmShieldConeOn - THE CONE GATE, A SCHMITT. The cone switches ON at
// on_kmh (12) and stays on until the rider drops BELOW off_kmh (10). A single 12 km/h threshold let the
// cone flick on and off with the rider's speed noise near takeoff: the escape dropped, the aim swapped
// and the lookahead clip jumped, tick after tick. Chosen over "keep the cone while an escape stands"
// because that would hold the escape but still let the cone toggle under every NON-escaping aim, and
// every toggle moves the lookahead clip (audit M-14); the Schmitt removes both, and between 10 and 12 km/h
// it only ever keeps MORE shield, never less.
// Inputs: prev_on (the gate's state last tick), the rider speed, the two thresholds. Returns the new state.
static inline bool fmShieldConeOn(bool prev_on, float speed_kmh, float on_kmh, float off_kmh)
{
  if (speed_kmh >= on_kmh) return true;
  return prev_on && speed_kmh >= off_kmh;
}

// V2.5-Evo - 2026-10-06 - audit M-16 (c): fmShieldHoldBandM - how far the cone is widened while an escape
// stands (the Schmitt hold), tied to the front station's own clearance from the cone:
//     hold = min(max_m, 0.5 x r x sin(phi - half_angle)), floored at 0
// r x sin(phi - half) is the settled front station's perpendicular distance from the cone's side (the
// station sits r from the rider at phi off his course; the side is at half_angle). A fixed 2 m hold was
// LARGER than that distance at the 35 deg cap (1.98 m at r 22.7 m, 30 deg), so once any escape started
// near a settled F4/F5 station it could never end while the rider stayed foiling. Half the clearance is
// always strictly less than the clearance, so the settled station is always OUTSIDE the held cone, by at
// least the hold band itself (host-asserted at every config). The difference phi - half is clamped to
// 0..90 (past 90 the station is behind the cone's side and sin would fold back).
// Inputs: r_m, phi_deg (the front station, fmFrontStationGeom), half_deg (fmShieldHalfAngleDeg), max_m (2).
static inline float fmShieldHoldBandM(float r_m, float phi_deg, float half_deg, float max_m)
{
  float d = phi_deg - half_deg;
  if (!(d > 0.0f) || !(r_m > 0.0f) || !(max_m > 0.0f)) return 0.0f;
  if (d > 90.0f) d = 90.0f;
  float b = 0.5f * r_m * sinf(d * BREMOTE_FMS_DEG2RAD);
  if (b > max_m) b = max_m;
  if (!(b > 0.0f)) b = 0.0f;
  return b;
}

// fmShieldMake - build this tick's shield from the rider's (filtered) speed and the four constants.
// Inputs: rider_speed_kmh; horizon_s (5); half_angle_deg (the RX passes fmShieldHalfAngleDeg(speed),
//         15-30 deg - audit L-26; clamped 1..89 here); min_speed_kmh (12, or 10 while the cone is on);
//         circle_min_m (3); min_dist_m (usrConf.min_dist_m - the circle grows to it if it is larger);
//         d_follow_m - the circle is never larger than the follow distance. At any sane tuning this
//         changes nothing (owner default: circle 4 m, follow 6 m); it exists so a degenerate tuning
//         (min_dist_m under 3 m with no smoothing band) cannot put the rider's own follow station
//         inside the circle, where no aim at it could ever clear and a walk home could never end.
// Returns the shield. No side effects. A negative or non-finite radius input reads as 0.
static inline FmShield fmShieldMake(float rider_speed_kmh, float horizon_s, float half_angle_deg,
                                    float min_speed_kmh, float circle_min_m, float min_dist_m,
                                    float d_follow_m)
{
  FmShield s;
  float r = circle_min_m;
  if (min_dist_m > r) r = min_dist_m;
  if (d_follow_m > 0.0f && r > d_follow_m) r = d_follow_m;
  if (!(r > 0.0f)) r = 0.0f;
  s.circle_r_m = r;
  if (half_angle_deg < 1.0f)  half_angle_deg = 1.0f;
  if (half_angle_deg > 89.0f) half_angle_deg = 89.0f;
  s.cone_tan = tanf(half_angle_deg * BREMOTE_FMS_DEG2RAD);
  s.cone_len_m = 0.0f;
  if (rider_speed_kmh >= min_speed_kmh && horizon_s > 0.0f) {
    s.cone_len_m = (rider_speed_kmh / 3.6f) * horizon_s;
  }
  s.circle_len_m = 0.0f;   // V2.5-Evo - 2026-10-06 - audit M-18: a plain circle unless extended for lag
  return s;
}

// V2.5-Evo - 2026-10-06 - audit M-18: fmShieldExtendForLag - cover the REAL rider when the lag anchor falls
// short of him.
// WHY. The shield is drawn round the ANCHOR: the filtered rider position pushed forward by v x tau to undo
// the filter's lag. That push is capped at 2 x d_follow, so at the Soft and Very Soft steering presets
// (tau 3 s and 5 s) a fast rider is really up to shortfall_m = v x tau - (the push actually applied) further
// AHEAD than the anchor. At Very Soft he sat outside his own 3 m circle from about 10.5 km/h, and the cone's
// reach past him shrank to about 12 m.
// WHAT. The circle becomes a CAPSULE from the anchor to shortfall_m ahead of it along the course (the same
// radius all the way), so it holds both the anchor and the real rider; and the cone, still drawn from the
// anchor, is made shortfall_m deeper, so it still reaches speed x horizon past the real rider. The anchor
// cone contains the cone drawn from the real rider (same angle, apex further back), so this only ever
// ADDS shield. A shortfall of 0 or less changes nothing (the Normal preset at ordinary speeds).
// Inputs: the shield (modified in place), shortfall_m (metres, along the course). No other side effects.
static inline void fmShieldExtendForLag(FmShield &s, float shortfall_m)
{
  if (!(shortfall_m > 0.0f)) return;
  s.circle_len_m = shortfall_m;
  if (s.cone_len_m > 0.0f) s.cone_len_m += shortfall_m;
}

// fmsCoreDist2 - squared distance from the point (along, cross) to the capsule's core: the segment of the
// course line from 0 to len_m ahead of the anchor. Internal to the capsule test.
static inline float fmsCoreDist2(float along_m, float cross_m, float len_m)
{
  const float x = (along_m < 0.0f) ? along_m : ((along_m > len_m) ? (along_m - len_m) : 0.0f);
  return x * x + cross_m * cross_m;
}

// fmsPointSegDist2 - squared distance from the point (px, py) to the segment A -> B. Internal.
static inline float fmsPointSegDist2(float px, float py, float a_al, float a_cr, float b_al, float b_cr)
{
  const float dx = b_al - a_al, dy = b_cr - a_cr;
  const float l2 = dx * dx + dy * dy;
  float t = 0.0f;
  if (l2 > 1e-9f) {
    t = ((px - a_al) * dx + (py - a_cr) * dy) / l2;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
  }
  const float qx = a_al + t * dx - px, qy = a_cr + t * dy - py;
  return qx * qx + qy * qy;
}

// fmsClipLB - one Liang-Barsky step: keep the part of the segment parameter range [*u0, *u1] that
// satisfies p * u <= q. Returns false when nothing is left. Internal to fmShieldSegmentHits.
static inline bool fmsClipLB(float p, float q, float *u0, float *u1)
{
  if (fabsf(p) < 1e-9f) return q >= 0.0f;        // parallel to this edge: all in or all out
  const float r = q / p;
  if (p < 0.0f) {                                 // entering
    if (r > *u1) return false;
    if (r > *u0) *u0 = r;
  } else {                                        // leaving
    if (r < *u0) return false;
    if (r < *u1) *u1 = r;
  }
  return true;
}

// fmShieldSegmentHits - does the segment A -> B (rider frame) touch the shield?
// THE CIRCLE is tested STRICTLY (closer than the radius), so a station sitting exactly at the radius -
// the follow distance can equal it at a minimal tuning - is not a contact.
// THE CONE is closed (touching counts - the conservative choice). inflate_m widens the CONE ONLY: its two
// sides move outward by inflate_m (perpendicular) and its base inflate_m further ahead, but it never
// reaches behind the rider (along >= 0 always). The controller uses that as a Schmitt band: an escape
// that is standing is held until the aim clears the WIDENED cone, so an aim grazing its edge cannot flick
// the escape on and off every tick. The circle is never widened: the rider's own rear station sits at the
// follow distance, only a smoothing band outside the circle, and a widened circle could swallow it and
// hold an escape that can never clear.
// Inputs: the shield, the two endpoints, inflate_m (>= 0). Returns true on contact. No side effects.
static inline bool fmShieldSegmentHits(const FmShield &s, float a_al, float a_cr,
                                       float b_al, float b_cr, float inflate_m)
{
  if (!(inflate_m > 0.0f)) inflate_m = 0.0f;
  const float dx = b_al - a_al, dy = b_cr - a_cr;

  // ---- the circle: the closest point of the segment to the rider ----
  const float R = s.circle_r_m;
  if (R > 0.0f && s.circle_len_m > 0.0f) {
    // V2.5-Evo - 2026-10-06 - audit M-18: THE CAPSULE (the circle swept from the anchor to circle_len_m
    // ahead). Contact = the segment comes closer than R to the core segment [0, L] of the course line.
    // Two segments that do not cross are closest at one of the four endpoints, so: (1) does A -> B cross
    // the core; (2) either end of A -> B against the core; (3) either end of the core against A -> B.
    const float L  = s.circle_len_m;
    const float R2 = R * R;
    if (((a_cr <= 0.0f && b_cr >= 0.0f) || (a_cr >= 0.0f && b_cr <= 0.0f)) && fabsf(dy) > 1e-9f) {
      const float x = a_al + dx * (a_cr / (a_cr - b_cr));
      if (x >= 0.0f && x <= L) return true;
    }
    if (fmsCoreDist2(a_al, a_cr, L) < R2) return true;
    if (fmsCoreDist2(b_al, b_cr, L) < R2) return true;
    if (fmsPointSegDist2(0.0f, 0.0f, a_al, a_cr, b_al, b_cr) < R2) return true;
    if (fmsPointSegDist2(L,    0.0f, a_al, a_cr, b_al, b_cr) < R2) return true;
  } else if (R > 0.0f) {
    const float l2 = dx * dx + dy * dy;
    float t = 0.0f;
    if (l2 > 1e-9f) {
      t = -(a_al * dx + a_cr * dy) / l2;
      if (t < 0.0f) t = 0.0f;
      if (t > 1.0f) t = 1.0f;
    }
    const float px = a_al + t * dx, py = a_cr + t * dy;
    if (px * px + py * py < R * R) return true;
  }

  // ---- the cone: a triangle (widened by inflate_m), four half-planes ----
  if (!(s.cone_len_m > 0.0f)) return false;
  const float k  = s.cone_tan;
  const float w0 = inflate_m * sqrtf(1.0f + k * k);         // = inflate / cos(half-angle): sides out by inflate
  const float xm = s.cone_len_m + inflate_m;                 // the base
  float u0 = 0.0f, u1 = 1.0f;
  //  -along <= 0                           (never behind the rider)
  if (!fmsClipLB(-dx, a_al, &u0, &u1)) return false;
  //   along <= xm
  if (!fmsClipLB(dx, xm - a_al, &u0, &u1)) return false;
  //   cross - k along <= w0                (inside the right-hand side)
  if (!fmsClipLB(dy - k * dx, w0 - (a_cr - k * a_al), &u0, &u1)) return false;
  //  -cross - k along <= w0                (inside the left-hand side)
  if (!fmsClipLB(-dy - k * dx, w0 - (-a_cr - k * a_al), &u0, &u1)) return false;
  return u0 <= u1;
}

// fmShieldHalfWidthM - how far either side of the rider's course line the (un-inflated) shield reaches
// at a given along-track position: the circle's half-chord, or the cone's half-width, whichever is
// larger; 0 where neither reaches. The shield is symmetric about the course line and both parts are
// convex, so at any along-track position its cross-section is the single interval [-w, +w]. That is
// what makes the go-around provable: a point further out than w on one side is outside the shield.
// V2.5-Evo - 2026-10-06 - audit M-18: with the capsule, the circle part is R wide all along [0, L] and rounds
// off past either end; still symmetric and convex, so the single-interval argument is unchanged.
static inline float fmShieldHalfWidthM(const FmShield &s, float along_m)
{
  float w = 0.0f;
  const float L  = (s.circle_len_m > 0.0f) ? s.circle_len_m : 0.0f;
  const float da = (along_m < 0.0f) ? -along_m : ((along_m > L) ? (along_m - L) : 0.0f);
  if (da < s.circle_r_m) {
    w = sqrtf(s.circle_r_m * s.circle_r_m - da * da);
  }
  if (s.cone_len_m > 0.0f && along_m >= 0.0f && along_m <= s.cone_len_m) {
    const float cw = along_m * s.cone_tan;
    if (cw > w) w = cw;
  }
  return w;
}

// fmShieldClipLookaheadM - G-2's steering lookahead pushes the aim along the rider's course, which can
// carry it INTO the cone even though the station point itself is outside (example: d_follow 9 + extra 7
// puts F4 at 16 m ahead x 13 m side, 39 deg; the full 9 m lookahead puts the aim at 25 m x 13 m, 27.5 deg
// - inside a 30 deg cone once the rider is past about 16 km/h). Returns the largest lookahead in
// [0, look_m] for which the segment buggy -> (station_along + look, station_cross) clears the shield
// (inflated by inflate_m): look_m itself when that already clears, 0 when even no lookahead clears (the
// caller's escape then decides), otherwise found by bisection to within about 1 cm. The result only
// ever SHORTENS the lookahead; it never moves the aim across the course line (the lookahead is parallel
// to it), so PG-3 is unaffected.
static inline float fmShieldClipLookaheadM(const FmShield &s, float inflate_m,
                                           float buggy_along_m, float buggy_cross_m,
                                           float station_along_m, float station_cross_m, float look_m)
{
  if (!(look_m > 0.0f)) return 0.0f;
  if (!fmShieldSegmentHits(s, buggy_along_m, buggy_cross_m,
                           station_along_m + look_m, station_cross_m, inflate_m)) return look_m;
  if (fmShieldSegmentHits(s, buggy_along_m, buggy_cross_m,
                          station_along_m, station_cross_m, inflate_m)) return 0.0f;
  float lo = 0.0f, hi = look_m;                    // lo clears, hi hits
  for (int i = 0; i < 12; i++) {
    const float mid = 0.5f * (lo + hi);
    if (fmShieldSegmentHits(s, buggy_along_m, buggy_cross_m,
                            station_along_m + mid, station_cross_m, inflate_m)) hi = mid;
    else                                                                       lo = mid;
  }
  return lo;
}

// fmShieldGoAroundLateralM - the go-around waypoint, as a SIGNED cross-track offset on `side`, placed at
// the BUGGY'S OWN along-track position (the caller builds it that way, as PG-4's outward waypoint is).
// It is at least min_lateral_m from the rider's line (the RX passes the pass minimum), at least clear_m
// further out than the buggy, and at least clear_m beyond the shield's edge at that along-track position.
// PROOF THAT THE AIM LINE THEN CLEARS: the aim segment runs straight across the course at one along-track
// position, where the shield's cross-section is [-w, +w]. If the buggy is on `side` and outside the shield
// (side x cross > w), every point from the buggy out to the waypoint is further out than w, so none is
// inside. The only ways the segment can still touch the shield are the two the design accepts: the buggy
// is already inside it (then this is the shortest way out, square to the rider's line), or the buggy is
// within the Schmitt band on the far side of the line (at most the band of measured crossing).
static inline float fmShieldGoAroundLateralM(const FmShield &s, int side, float buggy_along_m,
                                             float buggy_cross_m, float min_lateral_m, float clear_m)
{
  if (clear_m < 0.0f) clear_m = 0.0f;
  const float sg = (side < 0) ? -1.0f : 1.0f;
  float own = sg * buggy_cross_m;
  if (own < 0.0f) own = 0.0f;
  float lat = own + clear_m;
  const float edge = fmShieldHalfWidthM(s, buggy_along_m) + clear_m;
  if (edge > lat) lat = edge;
  if (min_lateral_m > lat) lat = min_lateral_m;
  return sg * lat;
}

// fmShieldDecide - the per-tick shield verdict for a NON-SNAPPED station (the RX never consults it for a
// rear station snapped to its preset - that path is the SW36 one, unchanged by owner ruling).
// Inputs: the shield; prev_escape (an escape was standing last tick: test with the cone widened
//         by hold_m, the Schmitt hold); prev_side (the side that standing escape went round on, 0 if
//         none); band_m (kFmSideHysteresisM, the SIDE band); hold_m (V2.5-Evo - 2026-10-06 - audit M-16 c:
//         the cone hold, fmShieldHoldBandM - it was band_m, 2 m, until then); psi_live_deg (the station angle, whose sign is the
//         COMMITTED side); the buggy and this tick's aim point, rider frame; min_lateral_m and clear_m
//         for the go-around (fmShieldGoAroundLateralM).
// Outputs: .escape   - the aim line touches the shield: steer at the go-around waypoint instead;
//          .side     - the side to go round on (fmBuggySideWithBand: the buggy's side beyond the band,
//                      else the committed side - the station's side, or, for a station exactly behind
//                      the rider, the side the standing escape already chose, so noise cannot flip it);
//          .reseed   - the station is on the OTHER side of the rider's line from `side`: re-seed it onto
//                      `side` - the H-1 retreat mechanism, extended to every non-snapped station. Never
//                      for a station exactly behind (psi 0): it has no side to be wrong about, and moving
//                      it to the buggy's measured angle would send a station walking home back out ahead;
//          .lateral_m - the go-around waypoint's signed cross-track offset.
// No side effects.
struct FmShieldDecision {
  bool  escape;
  int   side;
  bool  reseed;
  float lateral_m;
};
static inline FmShieldDecision fmShieldDecide(const FmShield &s, bool prev_escape, int prev_side,
                                              float band_m, float hold_m, float psi_live_deg,
                                              float buggy_along_m, float buggy_cross_m,
                                              float aim_along_m, float aim_cross_m,
                                              float min_lateral_m, float clear_m)
{
  FmShieldDecision d;
  d.escape    = fmShieldSegmentHits(s, buggy_along_m, buggy_cross_m, aim_along_m, aim_cross_m,
                                    prev_escape ? hold_m : 0.0f);
  const int station_side = (psi_live_deg > 0.0f) ? +1 : (psi_live_deg < 0.0f) ? -1 : 0;
  const int committed    = (station_side != 0) ? station_side
                         : (prev_escape && prev_side != 0) ? ((prev_side > 0) ? +1 : -1) : 0;
  d.side      = fmBuggySideWithBand(committed, buggy_cross_m, band_m);
  d.reseed    = d.escape && station_side != 0 && station_side != d.side;
  d.lateral_m = d.escape ? fmShieldGoAroundLateralM(s, d.side, buggy_along_m, buggy_cross_m,
                                                    min_lateral_m, clear_m)
                         : 0.0f;
  return d;
}

// V2.5-Evo - 2026-10-06 - fmShieldSettledEscapeExpired - THE SETTLED-ESCAPE TIMER (owner-approved rule, audit
// M-16 / design choice 7). A shield escape on an F4/F5 station that is OUTBOUND (in transit) latches the abort at
// once. One on a SETTLED station is allowed to ride out a wobble - but if it lasts MORE than limit_ms (the RX
// passes kFmShieldSettledEscapeMs, 3000) the rider's path and the station keep meeting, and the caller latches
// the F4/F5 abort to the rear preset on the buggy's side; the rider re-selects to try again.
// Inputs: active - a shield escape stands THIS tick on a settled F4/F5 station; now_ms (millis); *since_ms - when
//         the current unbroken run of such ticks began, 0 = not timing (state the caller owns); limit_ms.
// Returns true on the one tick the run has lasted more than limit_ms. *since_ms goes back to 0 whenever the run
// breaks (any tick with active false) and on expiry. A start at millis() == 0 is stored as 1 (0 is the sentinel).
static inline bool fmShieldSettledEscapeExpired(bool active, uint32_t now_ms, uint32_t *since_ms,
                                                uint32_t limit_ms)
{
  if (!active) { *since_ms = 0; return false; }
  if (*since_ms == 0) { *since_ms = (now_ms != 0) ? now_ms : 1u; return false; }
  if ((uint32_t)(now_ms - *since_ms) > limit_ms) { *since_ms = 0; return true; }
  return false;
}

// V2.5-Evo - 2026-10-06 - audit M-19 (owner-approved): fmNoCourseAbortLatches - does a tick with NO rider course
// abandon a front station?
// BUG: G-3's no-course clause latched the F4/F5 abort on any tick without a course. The owner changes mode
// stopped (HOLD -> switch -> squeeze), so the first ACTIVE tick has no course, F4/F5 was abandoned before it ever
// started, and F4/F5 as a Starting Station was dead - reachable only by re-selecting while moving.
// FIX: no latch while the station is still in the rear half (|psi_live| <= 90) AND no transit has started this
// engagement - there is nothing ahead of the rider to lose, and the degraded hold keeps the buggy at the follow
// distance on its current bearing from him (as it does for every mode) until a course exists. From the first tick with a course the pending seed,
// PG-2, H-1 and the shield govern the walk out, exactly as if F4/F5 had been selected while moving. A station
// already ahead of abeam, or one whose transit has begun, still latches as before.
// Inputs: m_decl (declared mode), aborted (latched already), psi_live_deg, transit_started (a transit began in
// this engagement - the RX passes fm_transit_start_course_deg >= 0). Returns true = latch the abort now.
static inline bool fmNoCourseAbortLatches(uint8_t m_decl, bool aborted, float psi_live_deg, bool transit_started)
{
  if (!(m_decl == 4 || m_decl == 5) || aborted) return false;
  if (fabsf(psi_live_deg) <= 90.0f && !transit_started) return false;
  return true;
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

// V2.5-Evo - 2026-10-06 - fmDTermContinuous - may the steering D term differentiate this heading-error
// sample against the previous one? Only when both come from the same continuous heading source, the
// same commanded target geometry (profile label), AND - audit M-10 - the aim did not step this tick
// (aim_stepped: a re-seed of the station angle or a swap of the aim point, whatever the labels say).
// Moved here from updateRtmSteering() (same condition plus the new term) so the host test runs it.
// Returns true = differentiate; false = skip D for this one sample.
static inline bool fmDTermContinuous(bool prev_valid, uint32_t src_id, uint32_t prev_src_id,
                                     uint8_t profile, uint8_t prev_profile, bool aim_stepped)
{
  return prev_valid && src_id == prev_src_id && profile == prev_profile && !aim_stepped;
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
