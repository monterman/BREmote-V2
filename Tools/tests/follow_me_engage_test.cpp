#include <assert.h>

#include "../../Source/Common/FollowMeEngage.h"

int main()
{
  const float floorM = 8.0f;   // kFmEngageDistFloorM

  // ---- Owner's tuning: min_dist 5 + band 4 = 9 m station, manual fm_engage_dist_m 12, rope 7.1 m ----
  const float ownerMin = 5.0f, ownerBand = 4.0f, ownerDengage = 12.0f;

  // The rope (7.1 m) never engages: not after a 2 s release (needs_dengage -> 12 m) ...
  assert(!followMeMayEngage(7.1f, ownerMin, ownerBand, floorM, ownerDengage, true));
  // ... and not without one either (the 9 m station edge still stands above the rope).
  assert(!followMeMayEngage(7.1f, ownerMin, ownerBand, floorM, ownerDengage, false));

  // 9 m engages at the owner's tuning without a long release. The edge is STRICT, exactly like the
  // Schmitt "dist > min_dist + band" it replaces, so the sample must sit beyond the 9.0 m threshold.
  assert(followMeEngageThresholdM(ownerMin, ownerBand, floorM, ownerDengage, false) == 9.0f);
  assert(followMeMayEngage(9.1f, ownerMin, ownerBand, floorM, ownerDengage, false));
  assert(!followMeMayEngage(9.0f, ownerMin, ownerBand, floorM, ownerDengage, false));

  // After a long release the same 9.1 m is NOT enough: the full 12 m separation is required again.
  assert(followMeEngageThresholdM(ownerMin, ownerBand, floorM, ownerDengage, true) == 12.0f);
  assert(!followMeMayEngage(9.1f, ownerMin, ownerBand, floorM, ownerDengage, true));
  assert(!followMeMayEngage(12.0f, ownerMin, ownerBand, floorM, ownerDengage, true));
  assert(followMeMayEngage(12.1f, ownerMin, ownerBand, floorM, ownerDengage, true));

  // ---- Factory tuning: min_dist 4 + band 2 = 6 m, auto d_engage 1.5 x 6 = 9 m ----
  const float factMin = 4.0f, factBand = 2.0f, factDengage = 9.0f;

  // 6 m never engages: the 8 m floor now sits above the 6 m station edge (was 6 m, below the rope).
  assert(followMeEngageThresholdM(factMin, factBand, floorM, factDengage, false) == 8.0f);
  assert(!followMeMayEngage(6.0f, factMin, factBand, floorM, factDengage, false));
  assert(!followMeMayEngage(7.1f, factMin, factBand, floorM, factDengage, false));
  assert(!followMeMayEngage(8.0f, factMin, factBand, floorM, factDengage, false));
  assert(followMeMayEngage(8.1f, factMin, factBand, floorM, factDengage, false));

  // With needs_dengage the factory tuning asks for the 9 m auto D_engage.
  assert(followMeEngageThresholdM(factMin, factBand, floorM, factDengage, true) == 9.0f);
  assert(!followMeMayEngage(8.5f, factMin, factBand, floorM, factDengage, true));
  assert(followMeMayEngage(9.5f, factMin, factBand, floorM, factDengage, true));

  // ---- The threshold can only rise: a manual D_engage below the station edge never lowers it ----
  assert(followMeEngageThresholdM(10.0f, 5.0f, floorM, 8.0f, true) == 15.0f);
  assert(followMeEngageThresholdM(10.0f, 5.0f, floorM, 8.0f, false) == 15.0f);

  // ---- The release rule: continuous release of kFmEngageGraceMs (2000) or more sets needs_dengage ----
  assert(!followMeReleaseNeedsDengage(0u, 5000u, 2000u));           // trigger held (timer idle)
  assert(!followMeReleaseNeedsDengage(1000u, 2999u, 2000u));        // 1999 ms released: not yet
  assert(followMeReleaseNeedsDengage(1000u, 3000u, 2000u));         // exactly 2000 ms: set
  assert(followMeReleaseNeedsDengage(1000u, 30000u, 2000u));        // long release: still set
  assert(followMeReleaseNeedsDengage(0xFFFFFF00u, 0x00000700u, 2000u));   // millis() wrap

  return 0;
}
