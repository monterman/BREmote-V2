// V2.5-Evo - 2026-10-30 - NEW FILE: Follow-Me cap 3, THE SPEED GOVERNOR, as pure functions (audit 2026-10-20 R1,
//   finding F-2). Moved out of fmComputeThrottleCap() (RTMState.ino) so Tools/tests/follow_me_governor_test.cpp can run
//   the exact law the RX runs inside a closed-loop simulation of the buggy following a rider. The arithmetic is the
//   inline code it replaces, in the same order, term for term; R1 adds only one thing: the radial gap term's gain and
//   ceiling are passed in, and the RX passes the larger pair only while the COMMANDED station sits far enough to the side
//   of the rider's line (fmGovSideOffset()).
//   Header-only, no Arduino dependencies. Every constant comes from the caller; none is defined here.
//
// WHAT THIS CAN AND CANNOT DO TO THE MOTOR: every function returns a speed or a CAP. The RX takes the lowest of its five
// caps and calcPWM() applies min(trigger, cap), so the rider's trigger stays the only throttle source and Follow-Me can
// only subtract. A bigger target speed here means a cap that bites LESS, never throttle the rider is not commanding.
//
//   1. fmGovSideOffset - R1's gate: is the commanded station at least min_dist + margin to the side of the rider's line?
//   2. fmGovGapKmh     - GOVERNOR-2's radial gap term: gain x metres outside the follow station, 0 .. max.
//   3. fmGovTargetKmh  - the target speed: (a) PG-4 escape = keep-up only, (b) a front station = keep-up + the signed
//                        along-track term, (c) every rear station = keep-up + closing margin + the gap term (withdrawn
//                        when theta is untrustworthy, audit GV-3).
//   4. fmGovCap        - the target clamped to boogie_vmax (GV-1, before the fade), the convergence fade past 90 deg
//                        (unless the front-station bypass stands), the floor (GV-2: at or under it the cap is 0), and the
//                        P-law cap = 255 x (1 - buggy speed / target).
#ifndef BREMOTE_FOLLOW_ME_GOVERNOR_H
#define BREMOTE_FOLLOW_ME_GOVERNOR_H

#include <stdint.h>
#include <math.h>

// ============================================================
// 1. fmGovSideOffset - R1: may the larger gap gain apply this tick?
// ============================================================
// WHY A GATE AT ALL (audit 2026-10-20 R1, worst case): the P-law's droop is the only stopping margin a buggy DIRECTLY
// BEHIND the rider has - if he stops dead it is the headway that keeps the coasting buggy off him - so the stronger
// gain must not shorten it. A buggy on a station well to the side passes a stopped rider at about that sideways offset
// instead (26 ft at the owner's 60 deg, 9 m station), so there it may close the gap faster.
// Inputs: frame_valid - the RX's fm_frame_valid: true only on a tick whose station was placed this tick (a valid
//         rider course; false in the degraded hold-station branch, where fm_station_cross_m is not refreshed);
//         station_cross_m - the commanded station's cross-track offset from the rider's line (fm_station_cross_m,
//         either side); min_dist_m - usrConf.min_dist_m; margin_m - the RX's extra (2 m).
// Returns: true = use the side-offset gain and ceiling. Side effects: none.
// At a 9 m follow distance and min_dist 5 m this needs |9 sin(near_diag)| >= 7 m, i.e. near_diag >= 51 deg.
static inline bool fmGovSideOffset(bool frame_valid, float station_cross_m, float min_dist_m, float margin_m)
{
  return frame_valid && (fabsf(station_cross_m) >= (min_dist_m + margin_m));
}

// ============================================================
// 2. fmGovGapKmh - the radial gap term (GOVERNOR-2)
// ============================================================
// Inputs: dist_m - buggy to rider (raw positions); d_follow_m - the follow station radius (min_dist + band, already
//         floored at 0.5 by the caller); gain_kmh_per_m; max_kmh. Returns: (dist - d_follow) x gain, clamped 0 .. max.
static inline float fmGovGapKmh(float dist_m, float d_follow_m, float gain_kmh_per_m, float max_kmh)
{
  float gap_kmh = (dist_m - d_follow_m) * gain_kmh_per_m;
  if (gap_kmh < 0.0f)    gap_kmh = 0.0f;
  if (gap_kmh > max_kmh) gap_kmh = max_kmh;
  return gap_kmh;
}

// ============================================================
// 3. fmGovTargetKmh - the governor's target speed before the vmax clamp and the fade
// ============================================================
// Inputs: aim_outward - PG-4 escape standing this tick (fm_aim_outward); front_station - fm_frame_valid and the live
//         station ahead of abeam; along_kmh - fmStationAlongGovKmh() for that front station (read only when
//         front_station); rider_kmh - fm_rider_speed_kmh; margin_kmh - kFmClosingMarginKmh; theta_ok - the theta bearing
//         is trustworthy; gap_kmh - fmGovGapKmh(). Returns: km/h. Side effects: none.
static inline float fmGovTargetKmh(bool aim_outward, bool front_station, float along_kmh,
                                   float rider_kmh, float margin_kmh, bool theta_ok, float gap_kmh)
{
  if (aim_outward)   return rider_kmh;                                          // (a)
  if (front_station) return rider_kmh + along_kmh;                              // (b)
  return rider_kmh + margin_kmh + (theta_ok ? gap_kmh : 0.0f);                  // (c)
}

// ============================================================
// 4. fmGovCap - cap 3 itself
// ============================================================
// Inputs: gov_kmh - fmGovTargetKmh(); vmax_kmh - usrConf.boogie_vmax_in_followme_kmh; theta_ok / theta_deg - the angle
//         between the rider's course and the buggy -> rider bearing (0 = he runs away from us, 180 = straight at us);
//         fade_bypass - fm_fade_bypass; floor_kmh - kFmGovFloorKmh; buggy_kmh - gps_last_speed_kmh.
// Returns: the cap, 0..255. Side effects: none.
// The order is the one the RX has always used: clamp to [0, vmax] FIRST (GV-1), then the fade, then the floor (a target
// at or under it is a cap of 0, never "no cap" - audit G-1), then the P-law, whose speed fraction is clamped to 0..1.
static inline uint16_t fmGovCap(float gov_kmh, float vmax_kmh, bool theta_ok, float theta_deg, bool fade_bypass,
                                float floor_kmh, float buggy_kmh)
{
  float gov = gov_kmh;
  if (gov < 0.0f)     gov = 0.0f;
  if (gov > vmax_kmh) gov = vmax_kmh;
  if (theta_ok && theta_deg > 90.0f && !fade_bypass) {
    float frac = (180.0f - theta_deg) / 90.0f;                // 1.0 abeam, 0.0 straight at us
    if (frac < 0.0f) frac = 0.0f;
    gov *= frac;
  }
  if (gov <= floor_kmh) return 0;
  float speed_frac = buggy_kmh / gov;                         // the buggy's own GPS speed vs the target
  if (speed_frac > 1.0f) speed_frac = 1.0f;
  if (speed_frac < 0.0f) speed_frac = 0.0f;
  return (uint16_t)((1.0f - speed_frac) * 255.0f);
}

#endif // BREMOTE_FOLLOW_ME_GOVERNOR_H
