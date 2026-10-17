// V2.5-Evo - 2026-10-17 - audit C-1: kSteerScaleMinSetting (50). Settings 0-50 (the factory default 50 and below;
//   0 = no motor-split steering) are never scaled and mix exactly as before at every throttle; only settings 51-99
//   follow the curve described in the line below. Settings 100 and above still return 100. Host-tested.
// V2.5-Evo - 2026-10-17 - THROTTLE-SCALED STEERING INFLUENCE (owner request, RX1): throttleScaledSteeringInfluence()
//   and its two compile-time constants kSteerFullPivotThr (77 = 30 %) / kSteerScaleEndThr (153 = 60 %). Below 30 %
//   throttle the mixer is handed 100 % (the one-motor pivot), from 60 % up the stored steering_influence, linear in
//   between. A setting of 100 returns 100 at every throttle, so a board at 100 is bit-identical to before. The mixer
//   itself is unchanged. Pure, host-tested in Tools/tests/differential_mixer_test.cpp.
#pragma once

#include <stdint.h>

struct DifferentialMotorMix {
  uint8_t motor0;
  uint8_t motor1;
};

// Redistribute the effective throttle symmetrically between both drive motors. The turn term is
// relative to throttle, so steering cannot create motor command from zero gas. Before upper-end
// saturation motor0 + motor1 is exactly 2 * throttle; saturation can only reduce that sum, never
// increase it. Steering 0 and 255 have equal authority around the exact neutral byte 127.
//
// The result remains in normalized 0..255 motor-command space. Mapping each command into its own
// calibrated PWM range afterward preserves the same relative power request for unequal PWM ranges.
static inline DifferentialMotorMix mixThrottleRelativeDifferential(
    uint8_t throttle,
    uint8_t steering,
    uint16_t steering_influence_pct,
    bool steering_inverted)
{
  if (steering_influence_pct > 100u) steering_influence_pct = 100u;

  const int steering_delta = (int)steering - 127;
  const int steering_span = (steering_delta < 0) ? 127 : 128;
  const int steering_magnitude = (steering_delta < 0) ? -steering_delta : steering_delta;
  const int divisor = 100 * steering_span;
  const int32_t numerator = (int32_t)throttle *
      (int32_t)steering_influence_pct * (int32_t)steering_magnitude;
  int turn = (int)((numerator + (divisor / 2)) / divisor);

  if (steering_delta < 0) turn = -turn;
  if (steering_inverted) turn = -turn;

  int motor0 = (int)throttle - turn;
  int motor1 = (int)throttle + turn;
  if (motor0 < 0) motor0 = 0;
  if (motor0 > 255) motor0 = 255;
  if (motor1 < 0) motor1 = 0;
  if (motor1 > 255) motor1 = 255;

  DifferentialMotorMix result = {(uint8_t)motor0, (uint8_t)motor1};
  return result;
}

// ============================================================
// V2.5-Evo - 2026-10-17 - THROTTLE-SCALED STEERING INFLUENCE - full pivot at low throttle, the
// stored setting at high throttle
// ============================================================
//
// WHY THIS EXISTS. The mixer above is a plain fraction of the throttle: at influence 100 and full
// lock the inside motor is 0 at EVERY throttle. That is the quick pivot the owner relies on at low
// throttle (stick hard over, release the trigger, squeeze to about 20 %, release, squeeze again),
// and it is also what flips the buggy at speed: every tow flip in the 2026-10-10 log, at 9-14 mph,
// had one motor near 255 and the other near 0 under high throttle, and the roll then built over
// 1-3 s. Lowering steering_influence alone (the owner chose 80) would soften the high-throttle
// split but would weaken the low-throttle pivot by the same fraction, because the mixer scales
// every throttle alike. So the influence handed to the mixer now depends on the throttle the mixer
// is given: full strength low, the stored setting high, a straight line between.
//
// NO confStruct FIELD. The two thresholds are compile-time, like the kPivot* constants in
// BREmote_V2_Rx.h, so this flash does not change sizeof(confStruct) and does not wipe the stored
// settings. steering_influence keeps its meaning above 60 % throttle, which is where it matters for
// the flips.
//
// THE THRESHOLDS, in mixer throttle counts (0-255, the same T the mixer receives), for settings 51-99:
//   kSteerFullPivotThr = 77 = 30 % of 255 (76.5, rounded up). At or below it the influence is 100,
//     the one-motor pivot. The owner pumps the trigger to about 20 % (51 counts) when he pivots,
//     so 30 % leaves 10 points of margin for a squeeze that goes a little deeper than he means.
//   kSteerScaleEndThr = 153 = 60 % of 255 (exactly). At or above it the influence is the stored
//     setting, unchanged from before. The flips happened under high throttle; from 60 % up they get
//     the full softening of the setting, and between 30 % and 60 % they get part of it.
//   Between them the influence falls in a straight line from 100 to the setting over the 76 counts,
//     so there is no step anywhere for the rider to feel. With the owner's 80: 100 % up to 30 %
//     throttle, 90 % at 45 %, 80 % from 60 %.
//
// WHAT IT CANNOT DO. The result is always between the setting and 100, so it can only make the
// split between the two motors larger or equal - it never touches the throttle, and the mixer still
// keeps motor0 + motor1 <= 2T. At T = 0 the mixer's turn term is 0 whatever the influence, so the
// stopped state is unchanged.
//
// THE FLOOR: SETTINGS 0-50 ARE NEVER SCALED (kSteerScaleMinSetting, audit C-1). The curve is for a
// board that was at 100 and has been stepped down to soften the high-throttle split while keeping
// the low-throttle pivot - that is a setting between 51 and 99. A setting of 50 or below is
// returned as it is at every throttle, so those boards mix exactly as before this change:
//   - 0 keeps its meaning, no motor-split steering at all. Scaled, it would have pivoted fully
//     below 30 % throttle, and a stick that does not centre exactly would then turn the buggy.
//   - 50 is the factory default (steering_type 1 boards ship with it). Scaled, every such board
//     would have had up to twice the split below 30 % throttle after a flash, and up to twice the
//     Follow-Me / return-to-me steering gain whenever a cap held the throttle at 30 % or less -
//     a handling change nobody asked for.
// With the floor, every scaled setting is at least 51, and for all of them both motor commands rise
// or hold as the throttle rises, at every steering byte and both inversions (host-tested for every
// setting 0-100): squeezing harder never takes speed off either motor.
static const uint8_t kSteerFullPivotThr = 77;    // throttle counts; at or below = influence 100 (one-motor pivot), settings 51-99
static const uint8_t kSteerScaleEndThr  = 153;   // throttle counts; at or above = the stored steering_influence
// Settings at or below this are never scaled: 0-50 (the factory default 50; 0 = no motor-split
// steering) are returned unchanged at every throttle, so those boards mix exactly as before.
static const uint8_t kSteerScaleMinSetting = 50;   // %, highest setting that is NOT scaled

static_assert(kSteerFullPivotThr < kSteerScaleEndThr,
              "the full-pivot point must sit below the point where the stored setting takes over, or the "
              "blend window is empty and the division below has nothing to divide by");

// throttleScaledSteeringInfluence - the steering influence to hand the mixer for this throttle.
// Inputs:  throttle    - the throttle byte the mixer is about to be given (0-255; in calcPWM() this
//                        is pivot_thr, the ramped and pivot-assisted throttle)
//          setting_pct - the stored steering_influence, % (0-100; anything above 100 counts as 100)
// Outputs: the influence in %, always between the setting and 100 inclusive:
//            setting >= 100            -> 100 at every throttle (bit-identical to the plain mixer)
//            setting <= 50             -> the setting at every throttle (bit-identical to the plain mixer)
//          and for settings 51-99:
//            throttle <= 77 (30 %)     -> 100 (the one-motor pivot)
//            throttle >= 153 (60 %)    -> the setting
//            in between                -> 100 down to the setting in a straight line, rounded to the
//                                         nearest whole %, never rising as the throttle rises
// Side effects: none. Pure integer arithmetic, no globals.
static inline uint16_t throttleScaledSteeringInfluence(uint8_t throttle, uint16_t setting_pct)
{
  if (setting_pct >= 100u) return 100u;
  if (setting_pct <= kSteerScaleMinSetting) return setting_pct;   // audit C-1: 0-50 are never scaled
  if (throttle <= kSteerFullPivotThr) return 100u;
  if (throttle >= kSteerScaleEndThr) return setting_pct;

  // How far into the window the throttle is, and how much influence the whole window takes away.
  // drop = (100 - setting) x into / window, rounded to nearest (+ window / 2 before dividing). The
  // numerator only grows with the throttle, so the result never rises; at into == window it would
  // be exactly 100 - setting, so the line meets the setting at 60 % with no step.
  const uint32_t window = (uint32_t)(kSteerScaleEndThr - kSteerFullPivotThr);   // 76 counts
  const uint32_t into   = (uint32_t)(throttle - kSteerFullPivotThr);           // 1..75 here
  const uint32_t drop   = ((uint32_t)(100u - setting_pct) * into + (window / 2u)) / window;
  return (uint16_t)(100u - drop);
}
