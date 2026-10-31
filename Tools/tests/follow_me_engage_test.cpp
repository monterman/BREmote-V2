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

  // ---- V2.5-Evo - 2026-10-30 - E-1 (unit 2, Revision 2 §6 item 4): the bumpless engage ramp, cap 5 ----
  // (1) Floor 0 is today's ramp TO THE BIT: the RX's old expression, every millisecond of a 1500 ms ramp and past it
  //     (today: no cap 5 at all once elapsed >= the ramp, i.e. 255 under min()).
  for (uint32_t el = 0; el <= 4000u; el++) {
    const uint16_t old = (el < 1500u) ? (uint16_t)(((float)el / (float)1500u) * 255.0f) : 255u;
    assert(fmEngageRampCap(el, 1500u, 0) == old);
  }
  // (2) With a floor: never above it at t = 0, never below it, never above 255, never falling, the existing slope.
  for (int f = 0; f <= 255; f++) {
    assert(fmEngageRampCap(0u, 1500u, (uint8_t)f) == (uint16_t)f);   // at the edge: exactly what the buggy was getting
    uint16_t prev = 0;
    for (uint32_t el = 0; el <= 2000u; el += 10u) {
      const uint16_t c = fmEngageRampCap(el, 1500u, (uint8_t)f);
      assert(c >= (uint16_t)f && c <= 255u && c >= prev);
      const uint16_t slope = (el < 1500u) ? (uint16_t)(((float)el / (float)1500u) * 255.0f) : 255u;
      assert(c == ((f + slope > 255) ? 255u : (uint16_t)(f + slope)));   // floor + today's slope, held at 255
      prev = c;
    }
  }

  // ---- V2.5-Evo - 2026-10-31 - G-2: the AUTO engage distance, followMeAutoEngageDistM() (RTMState.ino's auto branch) ----
  // kFmEngageFactor 1.5, kFmEngageAutoM 13.5 (RTMState.ino). Auto is max(13.5, 1.5 x d_follow): 13.5 m at d_follow <= 9 m,
  // the old 1.5 x d_follow above - it only ever rises, so no board engages earlier than before.
  {
    const float kFactor = 1.5f, kAutoM = 13.5f;
    assert(followMeAutoEngageDistM(4.0f + 2.0f, kFactor, kAutoM) == 13.5f);     // a 4 + 2 tuning: was 9.0 (floored to 9.5)
    assert(followMeAutoEngageDistM(ownerMin + ownerBand, kFactor, kAutoM) == 13.5f);   // the owner's 5 + 4: 13.5 either way
    assert(followMeAutoEngageDistM(10.0f + 10.0f, kFactor, kAutoM) == 30.0f);   // the factory 10 + 10 (defaultConf): kept at 30
    assert(followMeAutoEngageDistM(0.5f, kFactor, kAutoM) == 13.5f);            // the degenerate-geometry guard's 0.5 m
    for (int i = 1; i <= 400; i++) {                                            // d_follow 0.1-40 m
      const float df = 0.1f * (float)i, a = followMeAutoEngageDistM(df, kFactor, kAutoM);
      assert(a >= kAutoM && a >= kFactor * df);                                 // never below 13.5, never below the old auto
      assert(a == kAutoM || a == kFactor * df);                                 // and nothing else
      assert(a > floorM);                                                       // the 9.5 m floor never bites on auto
    }
    // The rope never engages at auto, even where the station edge is the floor: after a release the edge is 13.5 m.
    assert(followMeEngageThresholdM(4.0f, 2.0f, floorM, followMeAutoEngageDistM(6.0f, kFactor, kAutoM), true) == 13.5f);
    assert(!followMeMayEngage(13.5f, 4.0f, 2.0f, floorM, 13.5f, true, streakUp));
    assert(followMeMayEngage(13.6f, 4.0f, 2.0f, floorM, 13.5f, true, streakUp));
  }

  return 0;
}
