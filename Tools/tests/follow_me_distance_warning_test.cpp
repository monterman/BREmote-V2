#include <assert.h>

#include "../../Source/Common/FollowMeDistanceWarning.h"

int main()
{
  assert(followMeDecodeDistanceM(0u) == 0.0f);
  assert(followMeDecodeDistanceM(99u) == 9.9f);
  assert(followMeDecodeDistanceM(100u) == 10.0f);
  assert(followMeDecodeDistanceM(240u) == 150.0f);
  assert(followMeDecodeDistanceM(254u) == 164.0f);
  assert(followMeDecodeDistanceM(0xFFu) < 0.0f);

  // Equality is intentional: reaching the configured distance starts the warning.
  assert(!followMeDistanceWarningActive(true, true, true, 239u, 150u));
  assert(followMeDistanceWarningActive(true, true, true, 240u, 150u));
  assert(followMeDistanceWarningActive(true, true, true, 254u, 150u));

  // No warning from an incomplete FM declaration, a stale link or missing distance data.
  assert(!followMeDistanceWarningActive(false, true, true, 240u, 150u));
  assert(!followMeDistanceWarningActive(true, false, true, 240u, 150u));
  assert(!followMeDistanceWarningActive(true, true, false, 240u, 150u));
  assert(!followMeDistanceWarningActive(true, true, true, 0xFFu, 150u));
  assert(!followMeDistanceWarningActive(true, true, true, 254u, 165u));

  // Fire immediately, then at 2-second intervals. Unsigned subtraction preserves millis wrap.
  assert(followMeWarningPulseDue(true, false, 0u, 100u, 2000u));
  assert(!followMeWarningPulseDue(true, true, 100u, 2099u, 2000u));
  assert(followMeWarningPulseDue(true, true, 100u, 2100u, 2000u));
  assert(followMeWarningPulseDue(true, true, 0xFFFFFF00u, 0x000006D0u, 2000u));
  assert(!followMeWarningPulseDue(false, false, 0u, 5000u, 2000u));

  return 0;
}
