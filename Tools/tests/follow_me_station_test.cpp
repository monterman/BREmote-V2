// V2.5-Evo - 2026-10-02 - P2 host test for the Follow-Me station model.
//   Runs on the PC (see SOP-012 "Host-side unit tests"), exercises the exact header the RX compiles,
//   and is written so every assertion names the invariant it is defending.
//
//   g++ -std=c++17 -O1 -Wall -Wextra follow_me_station_test.cpp -o fms && ./fms
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../../Source/Common/FollowMeStation.h"

// The RX's own P2 constants (RTMState.ino), repeated here so the test states the numbers it claims.
static const float kFrontDefault   = 45.0f;    // kFmFrontAngleDefaultDeg
static const float kFrontMin       = 35.0f;    // kFmFrontAngleMinDeg (owner override of Rex's 40)
static const float kLateralMin     = 13.0f;    // kFmFrontLateralMinM  (carve 8 m + relative GPS 5 m)
static const float kPassLateral    = 10.1f;    // kFmPassLateralM      (rope 7.1 m + 3 m)
static const float kRadiusFactor   = 2.0f;     // kFmFrontRadiusFactor
static const float kNearDiag       = 45.0f;    // usrConf.near_diag_offset_deg, owner default
static const float kSlewRate       = 15.0f;    // kFmStationRateDegPerS (provisional)

static bool near_eq(float a, float b, float tol) { return fabsf(a - b) <= tol; }

int main()
{
  // ======================================================================================
  // 1. THE FRONT RADIUS AND THE LATERAL INVARIANT, ENFORCED BOTH WAYS
  // ======================================================================================
  {
    // Owner's tuning: min_dist 5 + band 4 = 9 m follow distance.
    const float dFollow = 9.0f;

    // 45 deg default -> max(2 x 9, 13/sin45) = max(18.0, 18.38) = 18.38 m, cross exactly 13.0 m.
    const float r45 = fmFrontRadiusM(dFollow, kRadiusFactor, kFrontDefault, kLateralMin);
    assert(near_eq(r45, 18.38f, 0.05f));
    assert(r45 * sinf(kFrontDefault * BREMOTE_FMS_DEG2RAD) >= kLateralMin - 0.01f);

    // 35 deg minimum -> 13/sin35 = 22.67 m (about 75 ft). A TIGHTER ANGLE BUYS A BIGGER RADIUS,
    // never less clearance: this is the owner's override of Rex's 40 deg recommendation, kept safe
    // by the radius rather than by refusing the angle.
    const float r35 = fmFrontRadiusM(dFollow, kRadiusFactor, kFrontMin, kLateralMin);
    assert(near_eq(r35, 22.67f, 0.05f));
    assert(r35 * sinf(kFrontMin * BREMOTE_FMS_DEG2RAD) >= kLateralMin - 0.01f);
    assert(r35 > r45);

    // Factory tuning 4 + 2 = 6 m: the lateral invariant binds, not the 2x factor.
    const float rFactory = fmFrontRadiusM(6.0f, kRadiusFactor, kFrontDefault, kLateralMin);
    assert(near_eq(rFactory, 18.38f, 0.05f));

    // A rider who tightens the follow distance does NOT shrink the clearance.
    const float rTight = fmFrontRadiusM(2.0f, kRadiusFactor, kFrontDefault, kLateralMin);
    assert(rTight * sinf(kFrontDefault * BREMOTE_FMS_DEG2RAD) >= kLateralMin - 0.01f);

    // A rider who widens it does not lose the 2x shape.
    const float rWide = fmFrontRadiusM(20.0f, kRadiusFactor, kFrontDefault, kLateralMin);
    assert(near_eq(rWide, 40.0f, 0.01f));

    // ---- The invariant the OTHER way: a bounded radius raises the ANGLE ----
    // With the radius computed above, phi_eff == phi (a no-op).
    assert(near_eq(fmFrontAngleEffDeg(kFrontDefault, r45, kLateralMin), kFrontDefault, 0.01f));
    assert(near_eq(fmFrontAngleEffDeg(kFrontMin,     r35, kLateralMin), kFrontMin,     0.01f));
    // If the radius were ever bounded at 20 m, 35 deg is no longer enough and the angle rises to
    // asin(13/20) = 40.54 deg - Rex's B2 figure, arrived at from the other side.
    assert(near_eq(fmFrontAngleEffDeg(kFrontMin, 20.0f, kLateralMin), 40.54f, 0.05f));
    // And the clearance is then honoured by the angle: 20 * sin(40.54) = 13.0 m.
    assert(20.0f * sinf(40.54f * BREMOTE_FMS_DEG2RAD) >= kLateralMin - 0.02f);
  }

  // ======================================================================================
  // 2. THE HARD FLOOR: NO INPUT CAN PUT A STATION DEAD AHEAD
  // ======================================================================================
  {
    // Sweep every angle the field could ever hold, including values no validator should allow.
    for (int phi_i = 1; phi_i <= 90; phi_i++) {
      const float phi   = (float)phi_i;
      const float limit = fmStationLimitDeg(phi);
      assert(limit <= 180.0f - 1.0f);                 // never 180: dead ahead is unreachable
      assert(limit >= 90.0f);
      for (int psi_i = -720; psi_i <= 720; psi_i += 3) {
        const float c = fmClampStationDeg((float)psi_i, phi);
        assert(fabsf(c) <= limit + 0.001f);
      }
    }
    // The two shipped angles, named.
    assert(near_eq(fmStationLimitDeg(45.0f), 135.0f, 0.001f));
    assert(near_eq(fmStationLimitDeg(35.0f), 145.0f, 0.001f));
    // A nonsense angle is clamped into (0, 90], so the limit stays inside [90, 179].
    assert(fmStationLimitDeg(0.0f)    <= 179.0f);
    assert(fmStationLimitDeg(-10.0f)  <= 179.0f);
    assert(fmStationLimitDeg(1000.0f) >= 90.0f);
  }

  // ======================================================================================
  // 3. THE PRESET TABLE, AND THE SIGN CONVENTION
  // ======================================================================================
  {
    const float phi = kFrontDefault;
    assert(near_eq(fmStationPresetDeg(1, kNearDiag, phi), +45.0f,  0.001f));   // rear-right
    assert(near_eq(fmStationPresetDeg(2, kNearDiag, phi),   0.0f,  0.001f));   // behind
    assert(near_eq(fmStationPresetDeg(3, kNearDiag, phi), -45.0f,  0.001f));   // rear-left
    assert(near_eq(fmStationPresetDeg(4, kNearDiag, phi), +135.0f, 0.001f));   // front-right
    assert(near_eq(fmStationPresetDeg(5, kNearDiag, phi), -135.0f, 0.001f));   // front-left
    // There is no mode 6, and no mode 6 behaviour: anything unknown is "directly behind".
    assert(near_eq(fmStationPresetDeg(6,    kNearDiag, phi), 0.0f, 0.001f));
    assert(near_eq(fmStationPresetDeg(0,    kNearDiag, phi), 0.0f, 0.001f));
    assert(near_eq(fmStationPresetDeg(0xFF, kNearDiag, phi), 0.0f, 0.001f));
    // Every preset is inside the floor by construction (the clamp is a no-op on all five).
    for (uint8_t m = 1; m <= 5; m++) {
      const float p = fmStationPresetDeg(m, kNearDiag, phi);
      assert(near_eq(fmClampStationDeg(p, phi), p, 0.001f));
    }
    // At the 35 deg minimum the front presets move out to +/-145, still never 180.
    assert(near_eq(fmStationPresetDeg(4, kNearDiag, kFrontMin), +145.0f, 0.001f));
    assert(near_eq(fmStationPresetDeg(5, kNearDiag, kFrontMin), -145.0f, 0.001f));
    // A near_diag the validator would never store cannot push a REAR preset ahead of the rider.
    assert(fabsf(fmStationPresetDeg(1, 170.0f, phi)) <= 90.0f);
    // The retreat target keeps the side: 4 -> rear-right, 5 -> rear-left.
    assert(near_eq(fmStationNearestRearPresetDeg(4, kNearDiag), +45.0f, 0.001f));
    assert(near_eq(fmStationNearestRearPresetDeg(5, kNearDiag), -45.0f, 0.001f));
    assert(near_eq(fmStationNearestRearPresetDeg(2, kNearDiag),   0.0f, 0.001f));
  }

  // ======================================================================================
  // 4. THE RADIUS SCHEDULE, AND PG-1 (THE STATION'S OWN CROSS-TRACK FLOOR)
  // ======================================================================================
  {
    const float phi = kFrontDefault;

    // ---- Owner's tuning: d_follow 9, r_front 18.38 ----
    {
      const float d = 9.0f;
      const float r = fmFrontRadiusM(d, kRadiusFactor, phi, kLateralMin);
      // Rear presets sit at exactly d_follow - today's geometry, unchanged.
      assert(near_eq(fmStationRadiusM(  0.0f, d, r, kNearDiag, kPassLateral, phi), d, 0.001f));
      assert(near_eq(fmStationRadiusM(+45.0f, d, r, kNearDiag, kPassLateral, phi), d, 0.001f));
      assert(near_eq(fmStationRadiusM(-45.0f, d, r, kNearDiag, kPassLateral, phi), d, 0.001f));
      // Front presets sit at exactly r_front, both sides.
      assert(near_eq(fmStationRadiusM(+135.0f, d, r, kNearDiag, kPassLateral, phi), r, 0.01f));
      assert(near_eq(fmStationRadiusM(-135.0f, d, r, kNearDiag, kPassLateral, phi), r, 0.01f));
      // Abeam: the ramp alone gives about 13.7 m at this tuning, comfortably over the pass minimum.
      const float rAbeam = fmStationRadiusM(90.0f, d, r, kNearDiag, kPassLateral, phi);
      assert(near_eq(rAbeam, 13.69f, 0.05f));
      assert(rAbeam >= kPassLateral);
    }

    // ---- Factory tuning 4 + 2 = 6: abeam 12.2 m, still over the pass minimum ----
    {
      const float d = 6.0f;
      const float r = fmFrontRadiusM(d, kRadiusFactor, phi, kLateralMin);
      const float rAbeam = fmStationRadiusM(90.0f, d, r, kNearDiag, kPassLateral, phi);
      assert(near_eq(rAbeam, 12.19f, 0.05f));
      assert(rAbeam >= kPassLateral);
    }

    // ---- Degenerate tuning d_follow 0.5: the RAMP alone would be 9.44 m at abeam, UNDER the
    //      10.1 m pass minimum. PG-1's floor is what stops the station inviting a close pass. ----
    {
      const float d = 0.5f;
      const float r = fmFrontRadiusM(d, kRadiusFactor, phi, kLateralMin);
      const float ramp_only = d + (r - d) * ((90.0f - kNearDiag) / (135.0f - kNearDiag));
      assert(ramp_only < kPassLateral);                       // the hazard is real at this tuning
      const float rAbeam = fmStationRadiusM(90.0f, d, r, kNearDiag, kPassLateral, phi);
      assert(near_eq(rAbeam, kPassLateral, 0.01f));           // floored to exactly the minimum
    }

    // ---- PG-1 as a sweep: every station from abeam to the floor keeps cross >= pass_lateral,
    //      at every one of these tunings and at both front angles. ----
    const float dFollows[5] = { 0.5f, 2.0f, 6.0f, 9.0f, 20.0f };
    const float angles[2]   = { kFrontDefault, kFrontMin };
    for (int ai = 0; ai < 2; ai++) {
      const float a   = angles[ai];
      const float lim = fmStationLimitDeg(a);
      for (int di = 0; di < 5; di++) {
        const float d = dFollows[di];
        const float r = fmFrontRadiusM(d, kRadiusFactor, a, kLateralMin);
        assert(r >= d);                                        // the radius never goes inward
        float prev = -1.0f;
        for (float psi = 0.0f; psi <= lim + 0.001f; psi += 1.0f) {
          const float rr = fmStationRadiusM(psi, d, r, kNearDiag, kPassLateral, a);
          assert(rr >= d - 0.001f);                            // never inside the follow station
          assert(rr >= prev - 0.001f);                         // monotone non-decreasing in |psi|
          prev = rr;
          float along, cross;
          fmStationAlongCrossM(psi, rr, &along, &cross);
          if (psi >= 90.0f) {
            assert(cross >= kPassLateral - 0.02f);             // PG-1
          }
          // And the mirror image on the left is identical in magnitude.
          float along_l, cross_l;
          fmStationAlongCrossM(-psi, fmStationRadiusM(-psi, d, r, kNearDiag, kPassLateral, a),
                               &along_l, &cross_l);
          assert(near_eq(along_l, along, 0.01f));
          assert(near_eq(cross_l, -cross, 0.01f));
        }
        // At the front limit the clearance is the full lateral invariant, not just the pass minimum.
        const float rLim = fmStationRadiusM(lim, d, r, kNearDiag, kPassLateral, a);
        float alongL, crossL;
        fmStationAlongCrossM(lim, rLim, &alongL, &crossL);
        assert(crossL >= kLateralMin - 0.02f);
        assert(alongL > 0.0f);                                 // a front station really is ahead
      }
    }
  }

  // ======================================================================================
  // 5. THE RIDER-FRAME COORDINATES, AND THE SIGNS THE WHOLE DESIGN RESTS ON
  // ======================================================================================
  {
    float along, cross;
    // Directly behind.
    fmStationAlongCrossM(0.0f, 9.0f, &along, &cross);
    assert(near_eq(along, -9.0f, 0.001f) && near_eq(cross, 0.0f, 0.001f));
    // Rear-right: behind and to the RIGHT.
    fmStationAlongCrossM(+45.0f, 9.0f, &along, &cross);
    assert(along < 0.0f && cross > 0.0f);
    // Rear-left: behind and to the LEFT.
    fmStationAlongCrossM(-45.0f, 9.0f, &along, &cross);
    assert(along < 0.0f && cross < 0.0f);
    // Abeam right: level with the rider, to the right.
    fmStationAlongCrossM(+90.0f, 13.7f, &along, &cross);
    assert(near_eq(along, 0.0f, 0.001f) && near_eq(cross, 13.7f, 0.001f));
    // Front-right at the default: 13.0 m ahead and 13.0 m right - the invariant as a coordinate.
    fmStationAlongCrossM(+135.0f, 18.38f, &along, &cross);
    assert(near_eq(along, 13.0f, 0.02f) && near_eq(cross, 13.0f, 0.02f));

    // The buggy's own coordinates, from a bearing and a distance.
    // Rider heading North (course 0). Buggy due South of them = directly behind.
    fmBuggyAlongCrossM(0.0f, 180.0f, 10.0f, &along, &cross);
    assert(near_eq(along, -10.0f, 0.01f) && near_eq(cross, 0.0f, 0.01f));
    // Buggy due East = abeam to the rider's right.
    fmBuggyAlongCrossM(0.0f, 90.0f, 10.0f, &along, &cross);
    assert(near_eq(along, 0.0f, 0.01f) && near_eq(cross, 10.0f, 0.01f));
    // Buggy due North = dead ahead (the station model never commands this; the test proves the sign).
    fmBuggyAlongCrossM(0.0f, 0.0f, 10.0f, &along, &cross);
    assert(near_eq(along, 10.0f, 0.01f) && near_eq(cross, 0.0f, 0.01f));
    // Same geometry with the rider heading West, to prove the frame rotates with the course.
    fmBuggyAlongCrossM(270.0f, 180.0f, 10.0f, &along, &cross);
    assert(near_eq(along, 0.0f, 0.01f) && near_eq(cross, -10.0f, 0.01f));

    // G-4: the measured station angle is the exact inverse of the bearing convention.
    for (float course = 0.0f; course < 360.0f; course += 17.0f) {
      for (float psi = -140.0f; psi <= 140.0f; psi += 13.0f) {
        const float bearing = course + 180.0f - psi;
        assert(near_eq(fmsWrap180(fmMeasuredStationDeg(course, bearing) - psi), 0.0f, 0.01f));
      }
    }
  }

  // ======================================================================================
  // 6. PG-2: THE STATION CANNOT GO AHEAD OF THE RIDER UNTIL THE BUGGY IS WIDE
  // ======================================================================================
  {
    const float lim = fmStationLimitDeg(kFrontDefault);     // 135

    // A REAR target is never gated.
    assert(near_eq(fmStationTargetCeilingDeg(+45.0f, 0.0f, kPassLateral, lim), 45.0f, 0.001f));
    assert(near_eq(fmStationTargetCeilingDeg(  0.0f, 0.0f, kPassLateral, lim),  0.0f, 0.001f));

    // A FRONT target with the buggy ON the rider's line: held at abeam, never beyond.
    assert(near_eq(fmStationTargetCeilingDeg(+135.0f, 0.0f, kPassLateral, lim), 90.0f, 0.001f));
    // Buggy wide but NOT YET wide enough (10.0 m against a 10.1 m minimum): still held.
    assert(near_eq(fmStationTargetCeilingDeg(+135.0f, 10.0f, kPassLateral, lim), 90.0f, 0.001f));
    // Buggy wide enough, on the right side: released to the full front station.
    assert(near_eq(fmStationTargetCeilingDeg(+135.0f, 10.1f, kPassLateral, lim), 135.0f, 0.001f));
    assert(near_eq(fmStationTargetCeilingDeg(+135.0f, 25.0f, kPassLateral, lim), 135.0f, 0.001f));
    // WRONG SIDE IS NOT WIDE. A buggy 25 m to the LEFT does not authorise a station to the RIGHT.
    assert(near_eq(fmStationTargetCeilingDeg(+135.0f, -25.0f, kPassLateral, lim), 90.0f, 0.001f));
    assert(near_eq(fmStationTargetCeilingDeg(-135.0f, +25.0f, kPassLateral, lim), 90.0f, 0.001f));
    // Mirror image for the left front station.
    assert(near_eq(fmStationTargetCeilingDeg(-135.0f, -10.1f, kPassLateral, lim), 135.0f, 0.001f));
    assert(near_eq(fmStationTargetCeilingDeg(-135.0f,   0.0f, kPassLateral, lim),  90.0f, 0.001f));
    // The ceiling never exceeds the hard floor, whatever is asked for.
    assert(fmStationTargetCeilingDeg(+179.0f, 100.0f, kPassLateral, lim) <= lim + 0.001f);
  }

  // ======================================================================================
  // 7. THE SLEW: PACED, AND IT NEVER CROSSES DEAD AHEAD
  // ======================================================================================
  {
    const float dt = 0.1f;                                   // the 10 Hz FM tick
    const float step = kSlewRate * dt;                       // 1.5 deg per tick

    // One step is exactly the rate.
    assert(near_eq(fmStationSlewStep(0.0f, 135.0f, kSlewRate, dt), step, 0.001f));
    assert(near_eq(fmStationSlewStep(0.0f, -135.0f, kSlewRate, dt), -step, 0.001f));
    // It arrives exactly, never overshoots.
    assert(near_eq(fmStationSlewStep(134.9f, 135.0f, kSlewRate, dt), 135.0f, 0.001f));
    assert(near_eq(fmStationSlewStep(135.0f, 135.0f, kSlewRate, dt), 135.0f, 0.001f));
    // A zero rate or a zero dt freezes it (this is also how the controller parks the station).
    assert(near_eq(fmStationSlewStep(90.0f, 135.0f, 0.0f, dt), 90.0f, 0.001f));
    assert(near_eq(fmStationSlewStep(90.0f, 135.0f, kSlewRate, 0.0f), 90.0f, 0.001f));

    // A SIDE CHANGE GOES THROUGH 0 (behind the rider), NEVER THROUGH 180 (dead ahead).
    // F4 (+135) to F5 (-135), walked tick by tick: psi must visit 0 and must never exceed 135.
    float psi = +135.0f;
    bool saw_behind = false;
    int ticks = 0;
    for (; ticks < 5000; ticks++) {
      psi = fmStationSlewStep(psi, -135.0f, kSlewRate, dt);
      assert(fabsf(psi) <= 135.0f + 0.001f);                 // never leaves the legal arc
      if (fabsf(psi) < 2.0f) saw_behind = true;
      if (near_eq(psi, -135.0f, 0.001f)) break;
    }
    assert(saw_behind);
    assert(ticks < 5000);
    // 270 deg at 15 deg/s = 18 s = 180 ticks of 100 ms.
    assert(ticks >= 179 && ticks <= 181);

    // Coming home from a front station passes through abeam and settles behind, monotonically.
    psi = +135.0f;
    float prev = psi;
    for (int i = 0; i < 200; i++) {
      psi = fmStationSlewStep(psi, 0.0f, kSlewRate, dt);
      assert(psi <= prev + 0.001f);
      prev = psi;
    }
    assert(near_eq(psi, 0.0f, 0.001f));
  }

  // ======================================================================================
  // 8. PG-3: THE AIM LINE NEVER CROSSES THE RIDER'S LINE
  // ======================================================================================
  {
    // Both endpoints wide on the same side -> proven clear.
    assert(fmAimClearsRiderLine(+10.1f, +13.0f, kPassLateral));
    assert(fmAimClearsRiderLine(+25.0f, +13.0f, kPassLateral));
    assert(fmAimClearsRiderLine(-10.1f, -13.0f, kPassLateral));
    // Opposite sides -> NOT clear, however wide each is: the line crosses the rider's line.
    assert(!fmAimClearsRiderLine(-25.0f, +13.0f, kPassLateral));
    assert(!fmAimClearsRiderLine(+25.0f, -13.0f, kPassLateral));
    // Either endpoint inside the minimum -> not clear.
    assert(!fmAimClearsRiderLine(+10.0f, +13.0f, kPassLateral));
    assert(!fmAimClearsRiderLine(+13.0f,  +9.0f, kPassLateral));
    assert(!fmAimClearsRiderLine(  0.0f, +13.0f, kPassLateral));

    // THE AFFINE ARGUMENT, CHECKED NUMERICALLY. Walk the segment from the buggy to the station and
    // confirm the minimum cross-track offset really is at an endpoint - that is what makes the two
    // endpoint tests above a proof about the whole line rather than about two points.
    const float cases[4][4] = {
      //  buggy_along, buggy_cross, station_along, station_cross
      {  -20.0f, +10.1f,  +13.0f, +13.0f },     // the pass itself, at the legal minimum
      {   -5.0f, +25.0f,  +13.0f, +13.0f },     // a wide buggy coming forward
      {  +30.0f, +13.0f,  +13.0f, +13.0f },     // too far ahead, coming back
      {  -20.0f, -10.1f,  +13.0f, -13.0f },     // the left-hand mirror
    };
    for (int c = 0; c < 4; c++) {
      const float bx = cases[c][0], by = cases[c][1];
      const float sx = cases[c][2], sy = cases[c][3];
      assert(fmAimClearsRiderLine(by, sy, kPassLateral));
      float worst = 1e9f;
      for (int k = 0; k <= 100; k++) {
        const float t = (float)k / 100.0f;
        const float cross = by + t * (sy - by);
        (void)(bx + t * (sx - bx));             // along is irrelevant to the cross-track minimum
        if (fabsf(cross) < worst) worst = fabsf(cross);
      }
      assert(worst >= kPassLateral - 0.001f);   // every point of the aim line clears the line
    }

    // PG-4: the escape test. Only ever true while the station is AHEAD.
    assert(!fmAimOutwardNeeded(+45.0f, 0.0f, 0.0f, kPassLateral));          // rear station: never
    assert(!fmAimOutwardNeeded(  0.0f, 0.0f, 0.0f, kPassLateral));
    assert( fmAimOutwardNeeded(+135.0f, +2.0f, +13.0f, kPassLateral));      // rider turned into us
    assert( fmAimOutwardNeeded(+135.0f, -20.0f, +13.0f, kPassLateral));     // wrong side
    assert(!fmAimOutwardNeeded(+135.0f, +11.0f, +13.0f, kPassLateral));     // still lawfully wide
    // Deliberately independent of the along-track term: the affine argument never used it, so its
    // failure test does not either. A buggy BEHIND the rider with a front station and no clearance
    // is just as much a corridor break as one ahead.
    assert( fmAimOutwardNeeded(+135.0f, +1.0f, +13.0f, kPassLateral));

    // The escape waypoint is always further out than the buggy, and never inside the minimum.
    for (float bc = -30.0f; bc <= 30.0f; bc += 1.0f) {
      const float lat = fmOutwardAimLateralM(+135.0f, bc, kPassLateral, 9.0f);
      assert(lat > 0.0f);                                    // on the station's side
      assert(lat >= kPassLateral);                           // never inside the pass minimum
      assert(lat >= fabsf(bc) + 9.0f - 0.001f);              // genuinely outward, with margin
      const float latL = fmOutwardAimLateralM(-135.0f, bc, kPassLateral, 9.0f);
      assert(near_eq(latL, -lat, 0.001f));
    }
  }

  // ======================================================================================
  // 9. THE GOVERNOR BYPASS IS A GEOMETRY PREDICATE, AND IT IS PARTIAL
  // ======================================================================================
  {
    // The only case that bypasses the fade: course valid, station ahead, buggy measurably ahead,
    // and passing beside the line by at least the pass minimum.
    assert( fmFadeBypass(true,  +135.0f, +5.0f, +13.0f, kPassLateral));
    assert( fmFadeBypass(true,  -135.0f, +5.0f, -13.0f, kPassLateral));
    // No course -> no "ahead" exists -> ordinary fade.
    assert(!fmFadeBypass(false, +135.0f, +5.0f, +13.0f, kPassLateral));
    // Station not ahead -> ordinary fade (this is every rear station, i.e. today's behaviour).
    assert(!fmFadeBypass(true,   +45.0f, +5.0f, +13.0f, kPassLateral));
    assert(!fmFadeBypass(true,    +0.0f, +5.0f, +13.0f, kPassLateral));
    assert(!fmFadeBypass(true,   +90.0f, +5.0f, +13.0f, kPassLateral));   // abeam is not "ahead"
    // Commanded ahead but NOT measurably ahead -> ordinary fade. The predicate reads the buggy's
    // measured position, never the mode it was asked for.
    assert(!fmFadeBypass(true,  +135.0f,  0.0f, +13.0f, kPassLateral));
    assert(!fmFadeBypass(true,  +135.0f, -5.0f, +13.0f, kPassLateral));
    // Ahead but passing too close to the line -> ordinary fade. This is the term that makes it a
    // pass rather than an overtake, and it is the one Rex will attack hardest.
    assert(!fmFadeBypass(true,  +135.0f, +5.0f, +10.0f, kPassLateral));
    assert(!fmFadeBypass(true,  +135.0f, +5.0f,   0.0f, kPassLateral));
    // The cross term is a magnitude, so a left-hand pass with a right-hand station still bypasses
    // only on clearance - the side agreement is PG-2's job, upstream of this predicate.
    assert( fmFadeBypass(true,  +135.0f, +5.0f, -13.0f, kPassLateral));
    // Sweep: along <= 0 or |cross| < pass_lateral is ALWAYS false, at every station angle.
    for (float psi = -145.0f; psi <= 145.0f; psi += 5.0f) {
      for (float along = -20.0f; along <= 20.0f; along += 2.0f) {
        for (float cross = -20.0f; cross <= 20.0f; cross += 2.0f) {
          const bool b = fmFadeBypass(true, psi, along, cross, kPassLateral);
          if (along <= 0.0f || fabsf(cross) < kPassLateral || fabsf(psi) <= 90.0f) assert(!b);
          else assert(b);
        }
      }
    }
  }

  // ======================================================================================
  // 10. THE ALONG-TRACK GOVERNOR TERM: SIGNED, CLAMPED, AND IT CAN SUBTRACT
  // ======================================================================================
  {
    const float gain = 0.5f, maxk = 15.0f;
    // At station: no adjustment.
    assert(near_eq(fmStationAlongGovKmh(13.0f, 13.0f, gain, maxk), 0.0f, 0.001f));
    // Behind station: positive, so the buggy may catch up.
    assert(near_eq(fmStationAlongGovKmh(13.0f, 3.0f, gain, maxk), +5.0f, 0.001f));
    // AHEAD of station: NEGATIVE, so the buggy gives throttle back instead of running on.
    assert(near_eq(fmStationAlongGovKmh(13.0f, 23.0f, gain, maxk), -5.0f, 0.001f));
    // Clamped both ways at the same ceiling the radial gap term uses.
    assert(near_eq(fmStationAlongGovKmh(13.0f, -200.0f, gain, maxk), +maxk, 0.001f));
    assert(near_eq(fmStationAlongGovKmh(13.0f, +200.0f, gain, maxk), -maxk, 0.001f));
  }

  printf("follow_me_station_test: all assertions passed\n");
  return 0;
}
