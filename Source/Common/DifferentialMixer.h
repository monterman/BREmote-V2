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
