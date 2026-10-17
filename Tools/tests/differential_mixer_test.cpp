// V2.5-Evo - 2026-10-17 - THROTTLE-SCALED STEERING INFLUENCE: tests for throttleScaledSteeringInfluence() appended at
//   the end of main() - setting 100 bit-identical to the plain mixer, the anchor points at setting 80, monotonic and
//   bounded for every setting 0-100, the full-lock pivot at 20 % throttle, the 2T power bound, motor commands that never
//   fall as the throttle rises for settings 48-100. The original tests above are unchanged.
// Build and run:  g++ -std=c++17 -Wall -Wextra -o differential_mixer_test differential_mixer_test.cpp && ./differential_mixer_test
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../../Source/Common/DifferentialMixer.h"

// The composition calcPWM() performs on the steering_type 1 branch (minus the align pivot boost,
// which replaces the influence afterwards and is not part of this helper).
static DifferentialMotorMix mixScaled(uint8_t throttle, uint8_t steering, uint16_t setting, bool inverted)
{
  return mixThrottleRelativeDifferential(throttle, steering,
                                         throttleScaledSteeringInfluence(throttle, setting), inverted);
}

int main()
{
  // Released throttle is structurally neutral for every steering byte and inversion direction.
  for (int steering = 0; steering <= 255; ++steering)
  {
    DifferentialMotorMix normal = mixThrottleRelativeDifferential(
        0, (uint8_t)steering, 100, false);
    DifferentialMotorMix inverted = mixThrottleRelativeDifferential(
        0, (uint8_t)steering, 100, true);
    assert(normal.motor0 == 0 && normal.motor1 == 0);
    assert(inverted.motor0 == 0 && inverted.motor1 == 0);
  }

  // Neutral steering and zero influence leave the throttle identical on both motors.
  for (int throttle = 0; throttle <= 255; ++throttle)
  {
    DifferentialMotorMix neutral = mixThrottleRelativeDifferential(
        (uint8_t)throttle, 127, 100, false);
    DifferentialMotorMix disabled = mixThrottleRelativeDifferential(
        (uint8_t)throttle, 255, 0, false);
    assert(neutral.motor0 == throttle && neutral.motor1 == throttle);
    assert(disabled.motor0 == throttle && disabled.motor1 == throttle);
  }

  // At 50% influence, full steering redistributes half of the live throttle in either direction.
  DifferentialMotorMix right = mixThrottleRelativeDifferential(100, 255, 50, false);
  DifferentialMotorMix left = mixThrottleRelativeDifferential(100, 0, 50, false);
  assert(right.motor0 == 50 && right.motor1 == 150);
  assert(left.motor0 == 150 && left.motor1 == 50);

  // Doubling gas doubles the steering delta; inversion swaps the two motor requests.
  DifferentialMotorMix low = mixThrottleRelativeDifferential(50, 255, 50, false);
  DifferentialMotorMix inverted = mixThrottleRelativeDifferential(100, 255, 50, true);
  DifferentialMotorMix saturated = mixThrottleRelativeDifferential(200, 255, 50, false);
  DifferentialMotorMix clampedInfluence = mixThrottleRelativeDifferential(100, 255, 150, false);
  assert(low.motor0 == 25 && low.motor1 == 75);
  assert(inverted.motor0 == right.motor1 && inverted.motor1 == right.motor0);
  assert(saturated.motor0 == 100 && saturated.motor1 == 255);
  assert(clampedInfluence.motor0 == 0 && clampedInfluence.motor1 == 200);

  // Below upper saturation, steering only redistributes power: the command sum stays exactly 2*T.
  for (int throttle = 0; throttle <= 127; ++throttle)
  {
    for (int steering = 0; steering <= 255; ++steering)
    {
      DifferentialMotorMix mixed = mixThrottleRelativeDifferential(
          (uint8_t)throttle, (uint8_t)steering, 100, false);
      assert((int)mixed.motor0 + (int)mixed.motor1 == 2 * throttle);
    }
  }

  // Across the full input space, steering can never increase aggregate requested motor power.
  const uint16_t influences[] = {0, 1, 35, 50, 100, 150};
  for (unsigned i = 0; i < sizeof(influences) / sizeof(influences[0]); ++i)
  {
    for (int throttle = 0; throttle <= 255; ++throttle)
    {
      for (int steering = 0; steering <= 255; ++steering)
      {
        DifferentialMotorMix mixed = mixThrottleRelativeDifferential(
            (uint8_t)throttle, (uint8_t)steering, influences[i], false);
        DifferentialMotorMix mixedInverted = mixThrottleRelativeDifferential(
            (uint8_t)throttle, (uint8_t)steering, influences[i], true);
        assert((int)mixed.motor0 + (int)mixed.motor1 <= 2 * throttle);
        assert((int)mixedInverted.motor0 + (int)mixedInverted.motor1 <= 2 * throttle);
        assert(mixedInverted.motor0 == mixed.motor1);
        assert(mixedInverted.motor1 == mixed.motor0);
      }
    }
  }

  // ---- V2.5-Evo - 2026-10-17 - throttle-scaled steering influence ----------------------------

  // The thresholds are the documented 30 % and 60 % of 255.
  assert(kSteerFullPivotThr == 77);
  assert(kSteerScaleEndThr == 153);

  // A setting of 100 (and anything above it, which the mixer clamps to 100 anyway) is bit-identical
  // to the plain mixer for every throttle, every steering byte and both inversions.
  const uint16_t at_or_above_100[] = {100, 101, 150, 65535};
  for (unsigned i = 0; i < sizeof(at_or_above_100) / sizeof(at_or_above_100[0]); ++i)
  {
    for (int throttle = 0; throttle <= 255; ++throttle)
    {
      assert(throttleScaledSteeringInfluence((uint8_t)throttle, at_or_above_100[i]) == 100);
      for (int steering = 0; steering <= 255; ++steering)
      {
        for (int inv = 0; inv <= 1; ++inv)
        {
          DifferentialMotorMix plain = mixThrottleRelativeDifferential(
              (uint8_t)throttle, (uint8_t)steering, at_or_above_100[i], inv != 0);
          DifferentialMotorMix scaled = mixScaled(
              (uint8_t)throttle, (uint8_t)steering, at_or_above_100[i], inv != 0);
          assert(plain.motor0 == scaled.motor0 && plain.motor1 == scaled.motor1);
        }
      }
    }
  }

  // The anchor points at the owner's setting of 80: full strength up to 30 %, the setting from 60 %.
  assert(throttleScaledSteeringInfluence(0,   80) == 100);
  assert(throttleScaledSteeringInfluence(51,  80) == 100);   // 20 %, the owner's pivot squeeze
  assert(throttleScaledSteeringInfluence(77,  80) == 100);   // 30 %, last count of full strength
  assert(throttleScaledSteeringInfluence(78,  80) == 100);   // first count into the window rounds to 100
  assert(throttleScaledSteeringInfluence(115, 80) == 90);    // 45 %, the middle of the window
  assert(throttleScaledSteeringInfluence(152, 80) == 80);    // last count inside the window rounds to 80
  assert(throttleScaledSteeringInfluence(153, 80) == 80);    // 60 %, the setting takes over
  assert(throttleScaledSteeringInfluence(255, 80) == 80);

  // For every setting 0-100: the result is never below the setting, never above 100, never rises as
  // the throttle rises, is 100 at and below 30 %, and is the setting at and above 60 %.
  for (int setting = 0; setting <= 100; ++setting)
  {
    uint16_t prev = 100;
    for (int throttle = 0; throttle <= 255; ++throttle)
    {
      const uint16_t inf = throttleScaledSteeringInfluence((uint8_t)throttle, (uint16_t)setting);
      assert(inf >= setting && inf <= 100);
      assert(inf <= prev);
      if (throttle <= kSteerFullPivotThr) assert(inf == 100);
      if (throttle >= kSteerScaleEndThr)  assert(inf == setting);
      prev = inf;
    }
  }

  // The owner's pivot: full lock at 20 % throttle with setting 80 gives exactly the setting-100 pair,
  // (0, 102) one way and the mirror the other way, and the inverted mirror with steering_inverted.
  {
    DifferentialMotorMix right80  = mixScaled(51, 255, 80, false);
    DifferentialMotorMix right100 = mixThrottleRelativeDifferential(51, 255, 100, false);
    DifferentialMotorMix left80   = mixScaled(51, 0, 80, false);
    DifferentialMotorMix left100  = mixThrottleRelativeDifferential(51, 0, 100, false);
    DifferentialMotorMix rightInv = mixScaled(51, 255, 80, true);
    assert(right80.motor0 == 0   && right80.motor1 == 102);
    assert(right80.motor0 == right100.motor0 && right80.motor1 == right100.motor1);
    assert(left80.motor0  == 102 && left80.motor1  == 0);
    assert(left80.motor0  == left100.motor0  && left80.motor1  == left100.motor1);
    assert(rightInv.motor0 == 102 && rightInv.motor1 == 0);
  }

  // At and above 60 % the setting applies exactly as it did before this change, and at and below
  // 30 % every setting mixes exactly like 100.
  for (int setting = 0; setting <= 100; ++setting)
  {
    for (int throttle = 0; throttle <= 255; ++throttle)
    {
      if (throttle > kSteerFullPivotThr && throttle < kSteerScaleEndThr) continue;
      const uint16_t expect = (throttle <= kSteerFullPivotThr) ? 100 : (uint16_t)setting;
      for (int steering = 0; steering <= 255; ++steering)
      {
        DifferentialMotorMix scaled = mixScaled((uint8_t)throttle, (uint8_t)steering, (uint16_t)setting, false);
        DifferentialMotorMix plain  = mixThrottleRelativeDifferential(
            (uint8_t)throttle, (uint8_t)steering, expect, false);
        assert(scaled.motor0 == plain.motor0 && scaled.motor1 == plain.motor1);
      }
    }
  }

  // It never adds power: across every setting, throttle, steering byte and inversion the command sum
  // stays <= 2T, and below upper saturation it is exactly 2T. Inversion still swaps the two motors.
  for (int setting = 0; setting <= 100; ++setting)
  {
    for (int throttle = 0; throttle <= 255; ++throttle)
    {
      for (int steering = 0; steering <= 255; ++steering)
      {
        DifferentialMotorMix mixed    = mixScaled((uint8_t)throttle, (uint8_t)steering, (uint16_t)setting, false);
        DifferentialMotorMix mixedInv = mixScaled((uint8_t)throttle, (uint8_t)steering, (uint16_t)setting, true);
        assert((int)mixed.motor0 + (int)mixed.motor1 <= 2 * throttle);
        if (throttle <= 127) assert((int)mixed.motor0 + (int)mixed.motor1 == 2 * throttle);
        assert(mixedInv.motor0 == mixed.motor1 && mixedInv.motor1 == mixed.motor0);
      }
    }
  }

  // Squeezing harder never lowers either motor for settings 48-100 (the owner's 80 included), at
  // every steering byte and both inversions. Below 48 the faster motor can dip slightly inside the
  // 30-60 % window; that limit is documented beside the constants.
  for (int setting = 48; setting <= 100; ++setting)
  {
    for (int steering = 0; steering <= 255; ++steering)
    {
      for (int inv = 0; inv <= 1; ++inv)
      {
        DifferentialMotorMix prev = mixScaled(0, (uint8_t)steering, (uint16_t)setting, inv != 0);
        for (int throttle = 1; throttle <= 255; ++throttle)
        {
          DifferentialMotorMix cur = mixScaled((uint8_t)throttle, (uint8_t)steering, (uint16_t)setting, inv != 0);
          assert(cur.motor0 >= prev.motor0 && cur.motor1 >= prev.motor1);
          prev = cur;
        }
      }
    }
  }

  // Print the owner's curve at setting 80, full lock right, so the run leaves a readable record.
  {
    const int pts[] = {0, 51, 77, 96, 115, 134, 153, 204, 255};
    printf("setting 80, full lock (steering 255), not inverted:\n");
    printf("  throttle  influence  motor0  motor1   | plain mixer at 80: motor0  motor1\n");
    for (unsigned i = 0; i < sizeof(pts) / sizeof(pts[0]); ++i)
    {
      const uint8_t t = (uint8_t)pts[i];
      DifferentialMotorMix m = mixScaled(t, 255, 80, false);
      DifferentialMotorMix p = mixThrottleRelativeDifferential(t, 255, 80, false);
      printf("  %3u (%3u%%)    %3u      %3u     %3u    |                    %3u     %3u\n",
             (unsigned)t, (unsigned)((t * 100u + 127u) / 255u),
             (unsigned)throttleScaledSteeringInfluence(t, 80), (unsigned)m.motor0, (unsigned)m.motor1,
             (unsigned)p.motor0, (unsigned)p.motor1);
    }
    printf("differential_mixer_test: all assertions passed\n");
  }

  return 0;
}
