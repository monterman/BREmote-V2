#include <assert.h>
#include <stdint.h>

#include "../../Source/Common/DifferentialMixer.h"

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

  return 0;
}
