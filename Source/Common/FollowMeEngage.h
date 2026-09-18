// V2.5-Evo - 2026-09-18 - Follow-Me engagement rules that do not depend on the trigger (P1-a).
//   Pure, header-only, no Arduino dependencies, so the host unit test in Tools/tests exercises the
//   exact code the RX runs. Three rules live here:
//     1. the ENGAGE FLOOR  - the distance the rider must be strictly beyond for Follow-Me to move
//        from ARMED/HOLD into ACTIVE, never inside the tow rope whatever the tuning;
//     2. the NEEDS-D_ENGAGE rule - after a long trigger release (or an RTM yield) the first
//        re-engagement must clear the full separation distance again, because a latch that can now
//        be earned while swimming must not let the rope (7.1 m) re-engage the buggy on the re-rig;
//     3. (2026-09-18, review fix) the LIVE-STREAK rule - while rule 2 applies, the first squeeze
//        also needs the separation streak standing on that tick, so a latch earned far away cannot
//        be spent close in.
#ifndef BREMOTE_FOLLOW_ME_ENGAGE_H
#define BREMOTE_FOLLOW_ME_ENGAGE_H

#include <stdint.h>

// followMeEngageThresholdM - the distance (metres) the rider must be STRICTLY BEYOND for Follow-Me
// to enter FM_ACTIVE from FM_ARMED or FM_HOLD. (The stay-on bound while ACTIVE is a different,
// lower edge - dist >= min_dist - and is not computed here.)
//
// Inputs:
//   min_dist_m, band_m  - the follow geometry; min_dist + band is the ordinary Schmitt "to engage"
//                         edge that has always applied.
//   floor_m             - kFmEngageDistFloorM (9.5 m since 2026-09-18, review fix; 8.0 m before,
//                         derived for a 6.1 m rope): the tow-rope floor. Before this rule the
//                         engage edge at the factory 4 + 2 tuning was 6 m, BELOW the 7.1 m rope.
//   d_engage_m          - the separation-latch distance in force (manual fm_engage_dist_m, or
//                         1.5 x d_follow, already clamped up to the floor by the caller).
//   needs_dengage       - true when the first re-engagement must clear the full separation distance
//                         (set after a trigger release of kFmReleaseDengageMs or more, and by the RTM
//                         yield; cleared on the ARMED/HOLD -> ACTIVE edge).
// Returns: the threshold in metres.
// Side effects: none (pure).
//
// The threshold can only RISE: max(min_dist + band, floor), then max(that, d_engage) when
// needs_dengage is set. A manual d_engage that happens to sit below min_dist + band can therefore
// never LOWER the edge - the rule adds a requirement, it never relaxes one.
static inline float followMeEngageThresholdM(float min_dist_m, float band_m, float floor_m,
                                             float d_engage_m, bool needs_dengage)
{
  float threshold_m = min_dist_m + band_m;
  if (threshold_m < floor_m) threshold_m = floor_m;
  if (needs_dengage && d_engage_m > threshold_m) threshold_m = d_engage_m;
  return threshold_m;
}

// followMeSeparationStreakStands - is the rider beyond D_engage RIGHT NOW, and have they been for
// the same evidence the separation latch demands (kFmSepDwellFixes distinct rider fixes over at
// least kFmSepDwellFloorMs)? The caller's streak counter (fm_sep_fix_count / fm_sep_over_since_ms)
// counts distinct fixes only while the distance is beyond D_engage and is zeroed the moment it is
// not, so a standing streak is proof about the CURRENT position, not a memory of an old one.
// Inputs: fix_count, over_since_ms - the live streak (over_since_ms 0 = not beyond D_engage now).
//         now_ms, dwell_fixes, dwell_floor_ms - the current time and the latch's own two constants.
// Returns: true when the live streak would itself satisfy the latch.
// Side effects: none (pure). Unsigned subtraction survives a millis() wrap.
static inline bool followMeSeparationStreakStands(uint8_t fix_count, uint32_t over_since_ms,
                                                  uint32_t now_ms, uint8_t dwell_fixes,
                                                  uint32_t dwell_floor_ms)
{
  return over_since_ms != 0 && fix_count >= dwell_fixes &&
         (uint32_t)(now_ms - over_since_ms) >= dwell_floor_ms;
}

// followMeMayEngage - condition 8 for a Follow-Me that is not yet ACTIVE: strictly beyond the
// threshold above, and - while needs_dengage is set - with the separation streak standing NOW.
// Strict, exactly like the Schmitt edge it replaces (dist > min_dist + band).
//
// WHY THE STREAK TERM (code-review finding on the edge-triggered release clear): once the proof can
// be re-earned off the trigger beyond 10 s, a latch earned while swimming at 20 m would otherwise
// survive the swim back to the rope, and the only thing left between that latch and an engagement
// on the first squeeze would be a single-tick dist > D_engage. Demanding the live streak means the
// first squeeze after a long release engages only if the rider is beyond D_engage on this tick AND
// has been for the last kFmSepDwellFixes distinct fixes - which at 7.1 m on the rope is impossible,
// and which a fresh whip satisfies within ~1 s of separating.
//
// Inputs: dist_m - the buggy-to-rider distance this tick; streak_stands - from
//         followMeSeparationStreakStands(); the rest as for the threshold.
// Returns: true when the distance alone permits an engagement. The trigger, the separation
//          latch and the fault conditions are separate terms the caller ANDs on top.
static inline bool followMeMayEngage(float dist_m, float min_dist_m, float band_m, float floor_m,
                                     float d_engage_m, bool needs_dengage, bool streak_stands)
{
  if (dist_m <= followMeEngageThresholdM(min_dist_m, band_m, floor_m, d_engage_m, needs_dengage)) {
    return false;
  }
  return !needs_dengage || streak_stands;
}

// followMeReleaseNeedsDengage - the rule that SETS needs_dengage from the trigger-release timer:
// the trigger has been continuously released (thr_received < 25) for grace_ms or longer.
// Inputs: thr_low_since_ms - millis() when the trigger first dropped below 25; 0 = currently held.
//         now_ms, grace_ms - the current time and kFmReleaseDengageMs (2000).
// Returns: true once the release has lasted grace_ms. Unsigned subtraction so a millis() wrap
//          cannot produce a false negative.
static inline bool followMeReleaseNeedsDengage(uint32_t thr_low_since_ms, uint32_t now_ms,
                                               uint32_t grace_ms)
{
  return thr_low_since_ms != 0 && (uint32_t)(now_ms - thr_low_since_ms) >= grace_ms;
}

#endif
