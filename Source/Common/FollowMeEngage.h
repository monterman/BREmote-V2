// V2.5-Evo - 2026-10-31 - G-2 (Follow-Me package 2): new followMeAutoEngageDistM() - the AUTO engage distance
//   (fm_engage_dist_m = 0) is max(13.5 m, 1.5 x d_follow), so it never falls below a 7 m rope + 6.5 m of GPS error and never
//   drops below what auto gave before. Pure; RTMState.ino's auto branch calls it.
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
//                         followMeAutoEngageDistM() below, already clamped up to the floor by the caller).
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

// followMeAutoEngageDistM - V2.5-Evo - 2026-10-31 - G-2 (design audit 2026-10-31 §2 / §G): the engage distance when
// fm_engage_dist_m is 0 (auto), before the caller's tow-rope floor.
// WHAT WAS WRONG: auto was factor x d_follow alone. At a 4 + 2 tuning that is 9.0 m, floored to 9.5 m, which sits only
// 2.4 m past a 7.1 m rope - the rope's measured on-rope p90 - because GPS error is a fixed number of metres, not a
// proportion. THE RULE NOW: rope length + 6.5 m; auto assumes a typical 7 m rope, so auto_m = 13.5 m.
// WHY max() AND NOT A STRAIGHT REPLACEMENT: the factory geometry (defaultConf: min_dist 10 + band 10 = 20 m) gives
// 1.5 x 20 = 30 m today, and D_engage also sets R2's divergence distances (2 x and 3 x D_engage) and the creep start.
// Replacing 30 m by 13.5 m there would let Follow-Me engage earlier and pull R2's ceiling from 60 m to the 30 m station
// geometry. So auto only ever RISES: max(auto_m, factor x d_follow) - 13.5 m wherever d_follow <= 9 m, today's value above.
// Inputs: d_follow_m - min_dist + band, as the caller guards it; factor - kFmEngageFactor (1.5); auto_m - kFmEngageAutoM.
// Returns: metres. Side effects: none (pure).
static inline float followMeAutoEngageDistM(float d_follow_m, float factor, float auto_m)
{
  const float prop_m = factor * d_follow_m;
  return (prop_m > auto_m) ? prop_m : auto_m;
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

// V2.5-Evo - 2026-10-30 - E-1 (unit 2 of the 2026-10-30 package, Revision 2 §6 item 4): THE BUMPLESS ENGAGE. Cap 5, the
// engage ramp, as a pure rule. WHY: on an ARMED -> ACTIVE edge with the trigger held the rider is already towing on his
// own throttle, and cap 5 restarting at 0 cut it to nothing for a moment - a throttle dip at take-over. Now the ramp
// starts from floor - the throttle the buggy was actually getting (calcPWM()'s g_applied_thr_cmd, post-caps, post-ramp,
// pre-mixer) - and rises at the existing slope (255 per ramp_ms). From HOLD or a pause the caller passes floor 0: today's
// ramp to the bit. The floor is never above what the rider's own trigger gave on the tick before, and the result is a
// CAP (min() with everything else in calcPWM()), so this can never add throttle the trigger did not ask for.
// Inputs: elapsed_ms since the engagement; ramp_ms (kFmEngageRampMs); floor (0-255). Returns: cap 5, 255 once the ramp
// has run (elapsed >= ramp_ms, or the slope has reached 255).
static inline uint16_t fmEngageRampCap(uint32_t elapsed_ms, uint32_t ramp_ms, uint8_t floor)
{
  if (elapsed_ms >= ramp_ms) return 255;
  const uint32_t c = (uint32_t)floor + (uint32_t)(uint16_t)(((float)elapsed_ms / (float)ramp_ms) * 255.0f);
  return (uint16_t)((c > 255u) ? 255u : c);
}

#endif
