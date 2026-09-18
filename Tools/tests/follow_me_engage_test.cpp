#include <assert.h>

#include "../../Source/Common/FollowMeEngage.h"

// The RX's own dwell constants (RTMState.ino): 3 distinct rider fixes over at least 350 ms.
static const uint8_t  kDwellFixes   = 3;
static const uint32_t kDwellFloorMs = 350;

int main()
{
  const float floorM = 9.5f;   // kFmEngageDistFloorM (9.5 m since 2026-09-18: 7.1 m rope x 1.31, rounded up)

  // ---- Owner's tuning: min_dist 5 + band 4 = 9 m station, manual fm_engage_dist_m 12, rope 7.1 m ----
  const float ownerMin = 5.0f, ownerBand = 4.0f, ownerDengage = 12.0f;

  // A standing streak (3 fixes over 1 s) and no streak at all, for the engage tests below.
  const bool streakUp   = followMeSeparationStreakStands(3, 1000u, 2000u, kDwellFixes, kDwellFloorMs);
  const bool streakDown = followMeSeparationStreakStands(0, 0u,    2000u, kDwellFixes, kDwellFloorMs);
  assert(streakUp);
  assert(!streakDown);

  // The rope (7.1 m) never engages: not after a 2 s release (needs_dengage -> 12 m) ...
  assert(!followMeMayEngage(7.1f, ownerMin, ownerBand, floorM, ownerDengage, true, streakUp));
  // ... and not without one either (the 9 m station edge still stands above the rope).
  assert(!followMeMayEngage(7.1f, ownerMin, ownerBand, floorM, ownerDengage, false, streakUp));

  // The owner's short-release edge is max(5 + 4, 9.5) = 9.5 m: the 9 m station edge is now under
  // the floor. The edge is STRICT, exactly like the Schmitt "dist > min_dist + band" it replaces,
  // so the sample must sit beyond the 9.5 m threshold.
  assert(followMeEngageThresholdM(ownerMin, ownerBand, floorM, ownerDengage, false) == 9.5f);
  assert(followMeMayEngage(9.6f, ownerMin, ownerBand, floorM, ownerDengage, false, streakDown));
  assert(!followMeMayEngage(9.5f, ownerMin, ownerBand, floorM, ownerDengage, false, streakDown));
  assert(!followMeMayEngage(9.1f, ownerMin, ownerBand, floorM, ownerDengage, false, streakDown));   // was the edge before the 9.5 m floor

  // After a long release the same 9.6 m is NOT enough: the full 12 m separation is required again.
  assert(followMeEngageThresholdM(ownerMin, ownerBand, floorM, ownerDengage, true) == 12.0f);
  assert(!followMeMayEngage(9.6f, ownerMin, ownerBand, floorM, ownerDengage, true, streakUp));
  assert(!followMeMayEngage(12.0f, ownerMin, ownerBand, floorM, ownerDengage, true, streakUp));
  assert(followMeMayEngage(12.1f, ownerMin, ownerBand, floorM, ownerDengage, true, streakUp));

  // ---- The live-streak rule (review fix on the edge-triggered 10 s release clear) ----
  // Scenario 1: a 15 s release at 20 m. The 10 s clear fired once at 10 s and the streak then
  // re-accumulated off the trigger (3 fixes beyond 12 m by ~11 s); at the first squeeze (15 s) the
  // rider is still at 20 m with the streak standing, so the proof stands and the squeeze engages.
  {
    const bool streak15s = followMeSeparationStreakStands(9, 11000u, 15000u, kDwellFixes, kDwellFloorMs);
    assert(streak15s);
    assert(followMeMayEngage(20.0f, ownerMin, ownerBand, floorM, ownerDengage, true, streak15s));
  }
  // Scenario 2: the latch was earned at 20 m, then the rider swam back to the rope (7.1 m). The
  // RX zeroes the streak the moment the distance falls inside D_engage, so at the squeeze the
  // streak is down - and 7.1 m is inside the 12 m edge anyway. The latch being set changes nothing
  // here: the latch is a separate AND term the caller applies; this function refuses on its own.
  {
    const bool streakAtRope = followMeSeparationStreakStands(0, 0u, 40000u, kDwellFixes, kDwellFloorMs);
    assert(!streakAtRope);
    assert(!followMeMayEngage(7.1f, ownerMin, ownerBand, floorM, ownerDengage, true, streakAtRope));
    // And even a momentary drift back out past 12 m does not engage until the streak stands again:
    // one fresh fix beyond D_engage is not three.
    const bool oneFix = followMeSeparationStreakStands(1, 41000u, 41100u, kDwellFixes, kDwellFloorMs);
    assert(!oneFix);
    assert(!followMeMayEngage(12.5f, ownerMin, ownerBand, floorM, ownerDengage, true, oneFix));
  }
  // Scenario 3: a fresh 3-fix streak beyond D_engage is impossible on the rope side. The streak
  // counter only counts while dist > D_engage, and 7.1 m never is, so its inputs stay at zero
  // however long the rider sits there - followMeSeparationStreakStands() can never go true.
  {
    assert(!followMeSeparationStreakStands(0, 0u, 100000u, kDwellFixes, kDwellFloorMs));
    assert(!followMeMayEngage(7.1f, ownerMin, ownerBand, floorM, ownerDengage, true, false));
    assert(!followMeMayEngage(7.1f, ownerMin, ownerBand, floorM, ownerDengage, true, true));   // even a (fictional) streak cannot beat the edge
  }
  // The streak's own boundaries: fixes and floor are both needed, and over_since 0 means "not beyond now".
  assert(!followMeSeparationStreakStands(2, 1000u, 5000u, kDwellFixes, kDwellFloorMs));   // two fixes
  assert(!followMeSeparationStreakStands(3, 1000u, 1349u, kDwellFixes, kDwellFloorMs));   // 349 ms
  assert(followMeSeparationStreakStands(3, 1000u, 1350u, kDwellFixes, kDwellFloorMs));    // exactly the floor
  assert(!followMeSeparationStreakStands(3, 0u, 5000u, kDwellFixes, kDwellFloorMs));      // not beyond D_engage now
  assert(followMeSeparationStreakStands(3, 0xFFFFFF00u, 0x00000200u, kDwellFixes, kDwellFloorMs));   // millis() wrap
  // Without needs_dengage the streak is not consulted (a short release keeps the ordinary edge).
  assert(followMeMayEngage(9.6f, ownerMin, ownerBand, floorM, ownerDengage, false, false));

  // ---- Factory tuning: min_dist 4 + band 2 = 6 m; auto d_engage 1.5 x 6 = 9.0 m, which the RX
  //      clamps up to the 9.5 m floor before it gets here (F3-c) ----
  const float factMin = 4.0f, factBand = 2.0f, factDengage = 9.5f;

  // 6 m never engages: the 9.5 m floor sits above the 6 m station edge (which is below the rope).
  // 9.5 m is refused (strict edge), 9.6 m engages.
  assert(followMeEngageThresholdM(factMin, factBand, floorM, factDengage, false) == 9.5f);
  assert(!followMeMayEngage(6.0f, factMin, factBand, floorM, factDengage, false, streakUp));
  assert(!followMeMayEngage(7.1f, factMin, factBand, floorM, factDengage, false, streakUp));
  assert(!followMeMayEngage(8.1f, factMin, factBand, floorM, factDengage, false, streakUp));   // engaged under the old 8 m floor
  assert(!followMeMayEngage(9.5f, factMin, factBand, floorM, factDengage, false, streakUp));
  assert(followMeMayEngage(9.6f, factMin, factBand, floorM, factDengage, false, streakDown));

  // With needs_dengage the factory tuning asks for the (clamped) 9.5 m D_engage, plus the streak.
  assert(followMeEngageThresholdM(factMin, factBand, floorM, factDengage, true) == 9.5f);
  assert(!followMeMayEngage(9.5f, factMin, factBand, floorM, factDengage, true, streakUp));
  assert(followMeMayEngage(9.6f, factMin, factBand, floorM, factDengage, true, streakUp));
  assert(!followMeMayEngage(9.6f, factMin, factBand, floorM, factDengage, true, streakDown));

  // ---- The threshold can only rise: a manual D_engage below the station edge never lowers it ----
  assert(followMeEngageThresholdM(10.0f, 5.0f, floorM, 9.5f, true) == 15.0f);
  assert(followMeEngageThresholdM(10.0f, 5.0f, floorM, 9.5f, false) == 15.0f);

  // ---- The release rule: continuous release of kFmEngageGraceMs (2000) or more sets needs_dengage ----
  assert(!followMeReleaseNeedsDengage(0u, 5000u, 2000u));           // trigger held (timer idle)
  assert(!followMeReleaseNeedsDengage(1000u, 2999u, 2000u));        // 1999 ms released: not yet
  assert(followMeReleaseNeedsDengage(1000u, 3000u, 2000u));         // exactly 2000 ms: set
  assert(followMeReleaseNeedsDengage(1000u, 30000u, 2000u));        // long release: still set
  assert(followMeReleaseNeedsDengage(0xFFFFFF00u, 0x00000700u, 2000u));   // millis() wrap

  return 0;
}
