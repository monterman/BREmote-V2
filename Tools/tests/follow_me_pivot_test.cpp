// V2.5-Evo - 2026-10-31 - Host test, Follow-Me package 2 unit H (design audit 2026-10-31 §H; the 2026-10-31 B-1 / F'
//   close-out §4): THE SELF-TUNING PIVOT ON VESC 2's GYRO, Common/FollowMePivot.h - the exact code RTMState.ino calls.
//   1. Unit checks on made-up sample streams: the IMU rate and its capability gates, the bounds, the learning (steps,
//      settle, trigger, delivered throttle, the change-of-side rule, capability loss), the per-boot memory and its decay,
//      NOT TURNING, the yaw-sign calibration, WRONG WAY.
//   2. The RX's numbers and call sites (source text, run from Tools/tests).
//   3. THE KILL SWITCH (red edit FM_PIVOT_LEARN_ENABLED 0): every RX source file, with the switch's blocks taken out the
//      way the preprocessor takes them out at 0, is token-for-token the file at the base commit (576879a), comments and
//      whitespace aside; every other RX / Common file is byte-identical. So the 0 build is the base firmware.
//   4. Simulation (the pivot plant is NOT modelled to bench fidelity - the unit is accepted on the bench; this checks the
//      rules against a plausible plant): (a) the calibration on rides, (b) THE FALSE-FIRE GATE (D-CD1): NOT TURNING and
//      WRONG WAY on healthy pivots with noise, 10 seeds x 10 pivots per run, HARD zero; (c) the red edits - calibration
//      skipped with an inverted mount must false-fire, bounds removed must show a cap above 80, mirrored steering during
//      pivots must be ended by WRONG WAY; (d) report-only rows.
//   5. `follow_me_pivot_test --replay <fixture.csv>`: the shadow replay of both checks over a level-6 ride log reduced by
//      the extractor to relative time, states, bytes, the IMU yaw / status / age, the course and 0/1 speed and distance
//      classes (no positions, no clock time, no speeds). Report, plus a zero-fire verdict on the Follow-Me rows.
//   `--base <sha>` changes section 3's base commit (for the fm-stations port).
// Build and run (Git Bash, from Tools/tests, PATH and TMP as in the host-test note of the compile procedure):
//   g++ -std=c++17 -O2 -Wall -Wextra -o follow_me_pivot_test follow_me_pivot_test.cpp && ./follow_me_pivot_test
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

#include "../../Source/Common/FollowMePivot.h"

static long g_checks = 0;
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "CHECK FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } g_checks++; } while (0)
// A simulation GATE: recorded and reported, and the run goes on, so every gate of the one run is on the record; any failed
// gate makes the test exit non-zero at the end. (Unit checks, CHECK, still stop at once.)
static int g_hard_failed = 0;
#define HARD(x, label) do { if (!(x)) { printf("    >>> HARD GATE FAILED: %s\n", label); g_hard_failed++; } else { g_checks++; } } while (0)

// The RX's numbers (RTMState.ino kFmPivotCfg), stated here; section 2 checks the RX holds the same initializer.
static const FmPivotCfg kCfg = { 600, 200.0f, 150, 1100, 250, 0.5f, 8, 1.6f, 128, 30.0f, 0.7f, 1.3f, 10, 1000, 0.8f, 32, 500, 2000, 10.0f, 10.0f, 5.0f, 2, 8, 10, 1500, 5.0f };
static const char *kCfgText = "static const FmPivotCfg kFmPivotCfg = { 600, 200.0f, 150, 1100, 250, 0.5f, 8, 1.6f, 128, 30.0f, 0.7f, 1.3f, 10, 1000, 0.8f, 32, 500, 2000, 10.0f, 10.0f, 5.0f, 2, 8, 10, 1500, 5.0f };";
static const char *g_base = "576879a";

// ======================================================================================
// small helpers
// ======================================================================================
static uint32_t T0 = 4000000000u;   // near the 32-bit wrap: every age in the module is computed modulo 2^32

// A tiny deterministic random source (xorshift), so every run is reproducible.
struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ull + 1ull) {}
  uint64_t next() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
  double uni() { return (double)(next() >> 11) / 9007199254740992.0; }
  double gauss() { double u1 = uni(), u2 = uni(); if (u1 < 1e-12) u1 = 1e-12; return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2); }
};

static double wrapd(double d) { while (d > 180.0) d -= 360.0; while (d < -180.0) d += 360.0; return d; }

// ======================================================================================
// 1. UNIT CHECKS
// ======================================================================================
static void testRate()
{
  FmPvRate r; fmPvRateReset(&r);
  // never parsed: nothing taken, not capable
  CHECK(!fmPvIngest(&r, false, T0 + 100, 10.0f, true, &kCfg));
  CHECK(!fmPvCapable(&r, T0 + 100, &kCfg));
  // the first sample: taken, but a rate needs a pair
  CHECK(fmPvIngest(&r, true, T0 + 100, 10.0f, true, &kCfg));
  CHECK(!r.rate_ok && !fmPvCapable(&r, T0 + 150, &kCfg));
  // the same sample seen again: not new
  CHECK(!fmPvIngest(&r, true, T0 + 100, 10.0f, true, &kCfg));
  // a pair 500 ms apart, +10 deg: 20 deg/s, capable up to 600 ms after the newer sample
  CHECK(fmPvIngest(&r, true, T0 + 600, 20.0f, true, &kCfg));
  CHECK(r.rate_ok && fabsf(r.rate_dps - 20.0f) < 1e-3f && r.t0_ms == T0 + 100 && r.t1_ms == T0 + 600);
  CHECK(fmPvCapable(&r, T0 + 1200, &kCfg) && !fmPvCapable(&r, T0 + 1201, &kCfg));
  CHECK(fmPvCapable(&r, T0 + 590, &kCfg));                          // a stamp newer than now reads "just now"
  // through the +/-180 wrap: 170 -> -170 in 500 ms is +20 deg = +40 deg/s, and the unwrapped yaw keeps counting
  FmPvRate w; fmPvRateReset(&w);
  fmPvIngest(&w, true, T0, 170.0f, true, &kCfg);
  fmPvIngest(&w, true, T0 + 500, -170.0f, true, &kCfg);
  CHECK(w.rate_ok && fabsf(w.rate_dps - 40.0f) < 1e-3f && fabsf(w.yaw_unw - 20.0f) < 1e-3f);
  // 200 deg/s is believed, 220 is not (and the chain breaks)
  fmPvIngest(&w, true, T0 + 1000, -70.0f, true, &kCfg);             // +100 deg in 0.5 s = 200
  CHECK(w.rate_ok && fabsf(w.rate_dps - 200.0f) < 1e-3f);
  const uint32_t seg = w.seg;
  fmPvIngest(&w, true, T0 + 1500, 40.0f, true, &kCfg);              // +110 deg in 0.5 s = 220
  CHECK(!w.rate_ok && w.seg == seg + 1 && !fmPvCapable(&w, T0 + 1500, &kCfg));
  // a pair too close (100 ms) or too far apart (1200 ms): no rate
  fmPvIngest(&w, true, T0 + 1600, 41.0f, true, &kCfg);
  CHECK(!w.rate_ok);
  fmPvIngest(&w, true, T0 + 2800, 45.0f, true, &kCfg);
  CHECK(!w.rate_ok);
  fmPvIngest(&w, true, T0 + 3300, 50.0f, true, &kCfg);              // the next good pair is believed again
  CHECK(w.rate_ok && fabsf(w.rate_dps - 10.0f) < 1e-3f);
  // an implausible sample (|acc| < 0.5 g / not finite): no rate on it, and none on the pair after it either
  fmPvIngest(&w, true, T0 + 3800, 55.0f, false, &kCfg);
  CHECK(!w.rate_ok && !w.have);
  fmPvIngest(&w, true, T0 + 4300, 60.0f, true, &kCfg);
  CHECK(!w.rate_ok);
  fmPvIngest(&w, true, T0 + 4800, NAN, true, &kCfg);
  CHECK(!w.rate_ok && !w.have);
  printf("1a. the IMU rate and its gates (never parsed, stale > 600 ms, implausible / non-finite, > 200 deg/s, pair spacing, the wrap): passed\n");
}

static void testBounds()
{
  uint8_t lo, hi;
  fmPvBounds(50, true, &kCfg, &lo, &hi);  CHECK(lo == 25 && hi == 80);    // Andres's 50: 25-80
  fmPvBounds(50, false, &kCfg, &lo, &hi); CHECK(lo == 25 && hi == 50);    // not calibrated: down only
  fmPvBounds(80, true, &kCfg, &lo, &hi);  CHECK(lo == 40 && hi == 128);   // never above 128
  fmPvBounds(13, true, &kCfg, &lo, &hi);  CHECK(lo == 8 && hi == 20);     // the factory 13
  fmPvBounds(13, false, &kCfg, &lo, &hi); CHECK(lo == 8 && hi == 13);
  fmPvBounds(8, true, &kCfg, &lo, &hi);   CHECK(lo == 8 && hi == 12);
  fmPvBounds(30, true, &kCfg, &lo, &hi);  CHECK(lo == 15 && hi == 48);
  for (int ac = 8; ac <= 80; ac++) {        // every legal fm_align_cap: inside [8, 128], 1.6 x, never above the cap uncalibrated
    fmPvBounds((uint8_t)ac, true, &kCfg, &lo, &hi);
    CHECK(lo >= 8 && hi <= 128 && hi <= (int)(1.6 * ac + 1e-6) && lo <= ac && hi >= ac);
    fmPvBounds((uint8_t)ac, false, &kCfg, &lo, &hi);
    CHECK(hi == ac);
  }
  printf("1b. the bounds: 25-80 at 50 (25-50 uncalibrated), 40-128 at 80, 8-20 at 13, every cap 8-80 inside [8, 128]: passed\n");
}

// A scripted learning pivot: ticks every 100 ms, IMU samples every 500 ms (first at +250 ms), the yaw from rate(t) (deg/s),
// the command, trigger and delivered throttle from lambdas. Returns the cap out per tick through `caps`.
struct LearnRig {
  FmPvRate r; FmPvLearn L; uint32_t t; double yaw; uint32_t next_imu;
  LearnRig() { fmPvRateReset(&r); fmPvLearnReset(&L); t = T0; yaw = 0.0; next_imu = T0 + 250; }
};
template <class Rate, class Cmd, class Trig, class App, class Cap>
static FmPvOut learnTicks(LearnRig *g, int n_ticks, bool env, uint8_t ac, int8_t sign, Rate rate, Cmd cmd, Trig trig, App app,
                          Cap capable_override, std::vector<int> *caps, int *nt_fires, const FmPivotCfg *cfg = &kCfg)
{
  FmPvOut o; o.cap = ac; o.state = 0; o.nt_fired = false; o.stepped = false;
  for (int i = 0; i < n_ticks; i++) {
    const double ts = (double)(uint32_t)(g->t - T0) / 1000.0;
    g->yaw += rate(ts) * 0.1;
    bool newer = false;
    if ((int32_t)(g->t - g->next_imu) >= 0) {
      newer = fmPvIngest(&g->r, true, g->next_imu, (float)wrapd(g->yaw), true, cfg);
      g->next_imu += 500;
    }
    bool capable = fmPvCapable(&g->r, g->t, cfg);
    if (!capable_override(ts)) capable = false;
    o = fmPvLearnStep(&g->L, g->t, env, ac, (uint8_t)trig(ts), (uint8_t)app(ts, o.cap), (int16_t)cmd(ts), capable, sign, newer,
                      &g->r, cfg);
    if (caps) caps->push_back(o.cap);
    if (nt_fires && o.nt_fired) (*nt_fires)++;
    g->t += 100;
  }
  return o;
}
static bool yes(double) { return true; }

static void testLearn()
{
  auto full   = [](double) { return 255; };
  auto right  = [](double) { return 127; };
  auto deliv  = [](double, uint8_t cap) { return (int)cap; };          // the buggy gets exactly the cap
  // ---- a slow buggy (10 deg/s), calibrated: up 10 per sample after the first second, to 80, never above ----
  {
    LearnRig g; std::vector<int> caps; int nt = 0;
    learnTicks(&g, 60, true, 50, 1, [](double) { return 10.0; }, right, full, deliv, yes, &caps, &nt);
    CHECK(caps[0] == 50 && caps[4] == 50);                             // the first 0.5 s: held to 50 (the lead-in)
    int mx = 0; for (int c : caps) if (c > mx) mx = c;
    CHECK(mx == 80 && caps.back() == 80 && nt == 0);                   // 10 deg/s turns 15+ deg in 2 s: not NOT TURNING
    // the first step: the pair (0.75, 1.25) - wholly inside the pivot, ends after 1 s, after the lead-in
    CHECK(caps[12] == 50 && caps[13] == 60);
  }
  // ---- the same, NOT calibrated: it may only learn down - it stays 50 ----
  {
    LearnRig g; std::vector<int> caps;
    learnTicks(&g, 60, true, 50, 0, [](double) { return 10.0; }, right, full, deliv, yes, &caps, nullptr);
    for (int c : caps) CHECK(c == 50);
  }
  // ---- a fast buggy (50 deg/s): down 10 per sample to the floor 25 (calibrated or not) ----
  for (int s = 0; s <= 1; s++) {
    LearnRig g; std::vector<int> caps;
    learnTicks(&g, 60, true, 50, (int8_t)s, [](double) { return 50.0; }, right, full, deliv, yes, &caps, nullptr);
    CHECK(caps.back() == 25);
    int mn = 255; for (int c : caps) if (c < mn) mn = c;
    CHECK(mn == 25);
  }
  // ---- in the band (30 deg/s): no step ----
  {
    LearnRig g; std::vector<int> caps;
    learnTicks(&g, 60, true, 50, 1, [](double) { return 30.0; }, right, full, deliv, yes, &caps, nullptr);
    for (int c : caps) CHECK(c == 50);
  }
  // ---- never above the trigger: trigger 60 -> out <= 60, and no step up once the trigger is the limit ----
  {
    LearnRig g; std::vector<int> caps;
    learnTicks(&g, 60, true, 50, 1, [](double) { return 5.0; }, right, [](double) { return 60; }, deliv, yes, &caps, nullptr);
    for (int c : caps) CHECK(c <= 60);
    CHECK(g.L.cap == 60 && caps.back() == 60);
  }
  // ---- another cap holds the buggy lower (the throttle it got is 30 while the cap is 50): no step up ----
  {
    LearnRig g; std::vector<int> caps;
    learnTicks(&g, 60, true, 50, 1, [](double) { return 10.0; }, right, full, [](double, uint8_t) { return 30; }, yes, &caps, nullptr);
    CHECK(g.L.cap == 50);
  }
  // ---- the command changes side mid-pivot (CHATTER rule): above 50 never again in that pivot, no step up ----
  {
    LearnRig g; std::vector<int> caps;
    learnTicks(&g, 30, true, 50, 1, [](double) { return 10.0; }, right, full, deliv, yes, &caps, nullptr);
    CHECK(caps.back() > 50);
    const int16_t before = g.L.cap;
    caps.clear();
    learnTicks(&g, 40, true, 50, 1, [](double) { return 10.0; }, [](double) { return -127; }, full, deliv, yes, &caps, nullptr);
    for (int c : caps) CHECK(c <= 50);
    CHECK(g.L.cap == before && g.L.flipped);
  }
  // ---- the IMU goes stale mid-pivot: fm_align_cap exactly on those ticks, the learned value kept, then back ----
  {
    LearnRig g; std::vector<int> caps;
    learnTicks(&g, 30, true, 50, 1, [](double) { return 10.0; }, right, full, deliv, yes, &caps, nullptr);
    const int16_t learned = g.L.cap;
    CHECK(learned > 50);
    caps.clear();
    learnTicks(&g, 10, true, 50, 1, [](double) { return 10.0; }, right, full, deliv, [](double) { return false; }, &caps, nullptr);
    for (int c : caps) CHECK(c == 50);
    CHECK(g.L.cap == learned && g.L.in_pivot);
    caps.clear();
    learnTicks(&g, 3, true, 50, 1, [](double) { return 10.0; }, right, full, deliv, yes, &caps, nullptr);
    CHECK(caps.back() >= learned);
  }
  // ---- outside the envelope: fm_align_cap exactly, every tick ----
  {
    LearnRig g; std::vector<int> caps;
    learnTicks(&g, 40, false, 50, 1, [](double) { return 10.0; }, right, full, deliv, yes, &caps, nullptr);
    for (int c : caps) CHECK(c == 50);
    CHECK(!g.L.in_pivot);
  }
  // ---- the per-boot memory: the next pivot starts at 50 + 0.8 x (last - 50), toward fm_align_cap ----
  {
    LearnRig g;
    learnTicks(&g, 60, true, 50, 1, [](double) { return 10.0; }, right, full, deliv, yes, nullptr, nullptr);
    CHECK(g.L.cap == 80);
    learnTicks(&g, 2, false, 50, 1, [](double) { return 0.0; }, right, full, deliv, yes, nullptr, nullptr);
    CHECK(!g.L.in_pivot && g.L.have_last && g.L.last == 80);
    const int expect[] = { 74, 69, 65, 62, 59, 57, 55, 54, 53, 52, 51, 50, 50 };
    for (int k = 0; k < 13; k++) {     // short pivots (no step): each starts 0.8 of the way closer to 50
      learnTicks(&g, 1, true, 50, 1, [](double) { return 30.0; }, right, full, deliv, yes, nullptr, nullptr);
      CHECK(g.L.cap == expect[k]);
      learnTicks(&g, 1, false, 50, 1, [](double) { return 0.0; }, right, full, deliv, yes, nullptr, nullptr);
    }
    // learned DOWN: 25 -> the next pivot starts at 50 - 0.8 x 25 = 30
    LearnRig h;
    learnTicks(&h, 60, true, 50, 1, [](double) { return 50.0; }, right, full, deliv, yes, nullptr, nullptr);
    learnTicks(&h, 1, false, 50, 1, [](double) { return 0.0; }, right, full, deliv, yes, nullptr, nullptr);
    learnTicks(&h, 1, true, 50, 1, [](double) { return 30.0; }, right, full, deliv, yes, nullptr, nullptr);
    CHECK(h.L.cap == 30);
    // the memory above fm_align_cap is held to 50 when the sign is not calibrated any more
    LearnRig u;
    learnTicks(&u, 60, true, 50, 1, [](double) { return 10.0; }, right, full, deliv, yes, nullptr, nullptr);
    learnTicks(&u, 1, false, 50, 1, [](double) { return 0.0; }, right, full, deliv, yes, nullptr, nullptr);
    learnTicks(&u, 1, true, 50, 0, [](double) { return 30.0; }, right, full, deliv, yes, nullptr, nullptr);
    CHECK(u.L.cap == 50);
    // a tick gap (> 250 ms with no call) ends the pivot too
    LearnRig q;
    learnTicks(&q, 30, true, 50, 1, [](double) { return 10.0; }, right, full, deliv, yes, nullptr, nullptr);
    const int16_t was = q.L.cap;
    q.t += 400;
    learnTicks(&q, 1, true, 50, 1, [](double) { return 30.0; }, right, full, deliv, yes, nullptr, nullptr);
    CHECK(q.L.in_pivot && q.L.last == was && q.L.cap == (int16_t)(50 + (int16_t)(0.8f * (float)(was - 50))));
  }
  printf("1c. the learning: +10 / -10 per sample after 1 s, 25-80 at 50, down only uncalibrated, never above the trigger, no step when\n"
         "    another cap holds the throttle, held to 50 after a change of side, fm_align_cap exactly when not capable or outside;\n"
         "    the memory 80 -> 74 -> 69 ... -> 50 and 25 -> 30: passed\n");
}

static void testNotTurning()
{
  auto full  = [](double) { return 255; };
  auto right = [](double) { return 127; };
  auto deliv = [](double, uint8_t cap) { return (int)cap; };
  // not turning at all: fires once, when 2 s of samples after the lead-in are held (samples at 0.25 + 0.5 k s; the window
  // starts at 0.75 s, so the 2.75 s sample decides) - the cap back to 50, and no step up after it in this pivot
  {
    LearnRig g; std::vector<int> caps; int nt = 0;
    learnTicks(&g, 80, true, 50, 1, [](double) { return 0.0; }, right, full, deliv, yes, &caps, &nt);
    CHECK(nt == 1);
    CHECK(caps[27] > 50 && caps[28] == 50);                             // fired on the tick at 2.8 s
    for (size_t i = 28; i < caps.size(); i++) CHECK(caps[i] == 50);
    CHECK(g.L.nt_latched);
  }
  // 11 deg in 2 s: not NOT TURNING; 9 deg in 2 s: NOT TURNING
  {
    LearnRig g; int nt = 0;
    learnTicks(&g, 80, true, 50, 0, [](double) { return 5.5; }, right, full, deliv, yes, nullptr, &nt);
    CHECK(nt == 0);
    LearnRig h; int nt2 = 0;
    learnTicks(&h, 80, true, 50, 0, [](double) { return 4.5; }, right, full, deliv, yes, nullptr, &nt2);
    CHECK(nt2 == 1);
  }
  // the throttle the buggy got dips under fm_align_cap on the 2.0 s tick: the window restarts at the first sample after the
  // dip (2.25 s; the pair (1.75, 2.25) straddles it, so its older sample cannot start it) and fires on the 4.25 s sample
  {
    auto dip = [](double ts, uint8_t cap) { return (ts > 1.95 && ts < 2.05) ? 30 : (int)cap; };
    LearnRig g; std::vector<int> caps; int nt = 0;
    learnTicks(&g, 43, true, 50, 0, [](double) { return 0.0; }, right, full, dip, yes, &caps, &nt);
    CHECK(nt == 0);                                                    // ticks 0.0 - 4.2 s: not yet
    learnTicks(&g, 1, true, 50, 0, [](double) { return 0.0; }, right, full, dip, yes, &caps, &nt);
    CHECK(nt == 1);                                                    // the 4.3 s tick
  }
  // under fm_align_cap (a light trigger of 40): not judged at all
  {
    LearnRig g; int nt = 0;
    learnTicks(&g, 100, true, 50, 0, [](double) { return 0.0; }, right, [](double) { return 40; },
               [](double, uint8_t cap) { return (int)(cap < 40 ? cap : 40); }, yes, nullptr, &nt);
    CHECK(nt == 0);
  }
  // the command changes side every 1.5 s: never 2 s on one side - not judged (a U-turn's reversal is not "not turning")
  {
    LearnRig g; int nt = 0;
    learnTicks(&g, 150, true, 50, 0, [](double) { return 0.0; }, [](double ts) { return (((int)(ts / 1.5)) % 2) ? -127 : 127; },
               full, deliv, yes, nullptr, &nt);
    CHECK(nt == 0);
  }
  printf("1d. NOT TURNING: fires once at 2.8 s on a buggy that does not turn, cap back to 50 and latched; 11 deg / 2 s no, 9 deg yes;\n"
         "    a throttle dip under the cap or a change of side restarts it; under the cap it is not judged: passed\n");
}

// Calibration helper: feed pairs 500 ms apart with the given yaw and course changes per pair.
static void calFeed(FmPvRate *r, FmPvCal *k, uint32_t *t, double *yaw, double *cog, double dyaw, double dcog, bool cog_ok,
                    int pairs)
{
  for (int i = 0; i < pairs; i++) {
    *t += 500; *yaw += dyaw; *cog += dcog;
    if (fmPvIngest(r, true, *t, (float)wrapd(*yaw), true, &kCfg))
      fmPvCalStep(k, r, cog_ok, (float)fmod(*cog + 3600.0, 360.0), &kCfg);
  }
}

static void testCalibration()
{
  // 8 turns with the yaw and the course turning the same way (each 3 pairs at 20 deg/s, then straight): sign +1
  {
    FmPvRate r; fmPvRateReset(&r); FmPvCal k; fmPvCalReset(&k);
    uint32_t t = T0; double yaw = 0, cog = 0;
    calFeed(&r, &k, &t, &yaw, &cog, 0, 0, true, 2);
    for (int n = 0; n < 8; n++) {
      CHECK(k.sign == 0);
      calFeed(&r, &k, &t, &yaw, &cog, (n % 2) ? 10.0 : -10.0, (n % 2) ? 10.0 : -10.0, true, 3);
      calFeed(&r, &k, &t, &yaw, &cog, 0, 0, true, 2);
    }
    CHECK(k.sign == 1 && k.turns == 8);
    // three turns against: 7 of the last 10 agree - not calibrated any more
    for (int n = 0; n < 3; n++) {
      calFeed(&r, &k, &t, &yaw, &cog, -10.0, 10.0, true, 3);
      calFeed(&r, &k, &t, &yaw, &cog, 0, 0, true, 2);
    }
    uint8_t p, m; fmPvCalCounts(&k, &p, &m);
    CHECK(k.sign == 0 && p == 7 && m == 3);
  }
  // the VESC mounted the other way up: the yaw turns against the course - sign -1
  {
    FmPvRate r; fmPvRateReset(&r); FmPvCal k; fmPvCalReset(&k);
    uint32_t t = T0; double yaw = 0, cog = 0;
    for (int n = 0; n < 10; n++) {
      calFeed(&r, &k, &t, &yaw, &cog, -8.0, 8.0, true, 3);
      calFeed(&r, &k, &t, &yaw, &cog, 0, 0, true, 2);
    }
    CHECK(k.sign == -1);
  }
  // no vote: the course turning but the yaw not (frozen or a wrongly mounted IMU), under 8 km/h (cog_ok false), a one-pair
  // "turn", a course turning under 10 deg/s
  {
    FmPvRate r; fmPvRateReset(&r); FmPvCal k; fmPvCalReset(&k);
    uint32_t t = T0; double yaw = 0, cog = 0;
    for (int n = 0; n < 10; n++) { calFeed(&r, &k, &t, &yaw, &cog, 0.5, 10.0, true, 3); calFeed(&r, &k, &t, &yaw, &cog, 0, 0, true, 2); }
    CHECK(k.turns == 0);
    for (int n = 0; n < 10; n++) { calFeed(&r, &k, &t, &yaw, &cog, 10.0, 10.0, false, 3); calFeed(&r, &k, &t, &yaw, &cog, 0, 0, false, 2); }
    CHECK(k.turns == 0);
    for (int n = 0; n < 10; n++) { calFeed(&r, &k, &t, &yaw, &cog, 10.0, 10.0, true, 1); calFeed(&r, &k, &t, &yaw, &cog, 0, 0, true, 2); }
    CHECK(k.turns == 0);
    for (int n = 0; n < 10; n++) { calFeed(&r, &k, &t, &yaw, &cog, 4.0, 4.0, true, 4); calFeed(&r, &k, &t, &yaw, &cog, 0, 0, true, 2); }
    CHECK(k.turns == 0 && k.sign == 0);
  }
  // an S-turn (the course changes direction without a pause) is two turns
  {
    FmPvRate r; fmPvRateReset(&r); FmPvCal k; fmPvCalReset(&k);
    uint32_t t = T0; double yaw = 0, cog = 0;
    calFeed(&r, &k, &t, &yaw, &cog, 0, 0, true, 1);
    calFeed(&r, &k, &t, &yaw, &cog, 10.0, 10.0, true, 3);
    calFeed(&r, &k, &t, &yaw, &cog, -10.0, -10.0, true, 3);
    calFeed(&r, &k, &t, &yaw, &cog, 0, 0, true, 1);
    CHECK(k.turns == 2 && k.vote[0] == 1 && k.vote[1] == 1);
  }
  printf("1e. the yaw-sign calibration: +1 after 8 agreeing turns, -1 for an inverted mount, lost at 7 of 10; no vote without yaw\n"
         "    movement, under 8 km/h, on a one-pair turn or a course turning under 10 deg/s; an S-turn is two turns: passed\n");
}

static void testWrongWay()
{
  // Ticks every 100 ms, samples every 500 ms from +250 ms; the command, the yaw rate (the IMU's own sign) and the sign given.
  // The first second only warms the IMU stream up (not judged), as on the RX, where the stream runs before a pivot begins;
  // the times below are from the first judged tick (1.0 s).
  auto run = [](int8_t sign, double rate_imu, int cmd, int n_ticks, bool judged_all, long *t_fire) {
    FmPvRate r; fmPvRateReset(&r); FmPvWw w = { 0, 0, 0, 0 };
    uint32_t t = T0, next = T0 + 250; double yaw = 0;
    for (int i = 0; i < n_ticks + 10; i++) {
      yaw += rate_imu * 0.1;
      bool nw = false;
      if ((int32_t)(t - next) >= 0) { nw = fmPvIngest(&r, true, next, (float)wrapd(yaw), true, &kCfg); next += 500; }
      const bool judged = (i >= 10) && (judged_all || (i != 25));
      if (fmPvWwStep(&w, t, judged, (int16_t)cmd, sign, fmPvCapable(&r, t, &kCfg), nw, &r, &kCfg)) { *t_fire = (long)(t - T0) - 1000; return true; }
      t += 100;
    }
    *t_fire = -1; return false;
  };
  long tf = 0;
  // calibrated +1, steered right, yawing left at 10 deg/s: the run starts on the pair (0.75, 1.25) after the lead-in and
  // covers 1.5 s at the 2.25 s sample - the verdict on the tick at 2.3 s
  CHECK(run(1, -10.0, 60, 60, true, &tf) && tf == 2300);
  CHECK(run(1, 10.0, -60, 60, true, &tf) && tf == 2300);       // steered left, yawing right
  CHECK(!run(1, 10.0, 60, 300, true, &tf));                    // healthy: never
  CHECK(!run(1, -4.0, 60, 300, true, &tf));                    // 4 deg/s against: under 5
  CHECK(run(1, -5.0, 60, 60, true, &tf));                      // 5 deg/s: yes
  CHECK(!run(0, -10.0, 60, 300, true, &tf));                   // not calibrated: off
  CHECK(!run(1, -10.0, 31, 300, true, &tf));                   // under 32 counts: not "steering to one side"
  CHECK(run(-1, 10.0, 60, 60, true, &tf) && tf == 2300);       // inverted mount, sign -1: the IMU's +10 is a LEFT yaw
  CHECK(!run(-1, -10.0, 60, 300, true, &tf));                  // ... and its -10 a right one: healthy
  CHECK(run(1, -10.0, 60, 60, false, &tf) && tf == 3800);      // one tick not judged (1.5 s) clears it: lead-in from 1.6 s, 3.8 s
  printf("1f. WRONG WAY: 2.3 s after the command settles (0.5 s lead-in + 1.5 s at 5 deg/s or more), either side, the inverted\n"
         "    mount through its sign; never healthy, never uncalibrated, not under 32 counts or 5 deg/s; a gap restarts it: passed\n");
}

// ======================================================================================
// 2. THE RX's NUMBERS AND CALL SITES (source text)
// ======================================================================================
static std::string readFile(const char *path)
{
  FILE *f = fopen(path, "rb");
  if (!f) return std::string();
  std::string s; char buf[65536]; size_t k;
  while ((k = fread(buf, 1, sizeof(buf), f)) > 0) s.append(buf, k);
  fclose(f);
  std::string o; o.reserve(s.size());
  for (size_t i = 0; i < s.size(); i++) if (s[i] != '\r') o.push_back(s[i]);   // CRLF working copy -> LF
  return o;
}
static size_t countOf(const std::string &s, const std::string &n)
{
  size_t c = 0, p = 0;
  while ((p = s.find(n, p)) != std::string::npos) { c++; p += n.size(); }
  return c;
}

static void testRxSource()
{
  const std::string rtm = readFile("../../Source/V2_Integration_Rx/RTMState.ino");
  const std::string hdr = readFile("../../Source/V2_Integration_Rx/BREmote_V2_Rx.h");
  const std::string vesc = readFile("../../Source/V2_Integration_Rx/VESC.ino");
  CHECK(!rtm.empty() && !hdr.empty() && !vesc.empty());
  CHECK(countOf(rtm, kCfgText) == 1);
  CHECK(rtm.find("static const float    kFmPvMaxKmh  = 6.437f;") != std::string::npos);   // 4 mph
  CHECK(rtm.find("static const float    kFmPvCalKmh  = 8.0f;") != std::string::npos);
  // cap 4 takes this tick's learned cap only when the learn step ran this tick; else fm_align_cap
  CHECK(countOf(rtm, "const uint16_t c = fm_pv_cap4_tick ? (uint16_t)fm_pv_cap4 : (uint16_t)fmAlignCapValue();") == 1);
  // the learn step on the ACTIVE path runs right before the cap chain, once
  const size_t a = rtm.find("fmPivotLearnTick(now, true, dist_m);");
  CHECK(a != std::string::npos && countOf(rtm, "fmPivotLearnTick(now, true, dist_m);") == 1);
  const size_t b = rtm.find("fm_throttle_cap = (uint8_t)fmComputeThrottleCap(dist_m, now);", a);
  CHECK(b != std::string::npos && b - a < 200);
  // WRONG WAY ends Follow-Me through R2-S's path: it sets sr_fault, and the end path names code 12
  CHECK(countOf(rtm, "sr_fault     = true;   // R2-S's end path, code 12") == 1);
  CHECK(rtm.find("fmPvWwStep(&fm_pv_ww, (uint32_t)now, pv_judged,") != std::string::npos);
  CHECK(rtm.find("fm_last_stop_reason = FM_STOP_WRONG_WAY;") != std::string::npos);
  // the wrapper: sample tick before the body, the fallback learn step after it, the prints last
  const size_t s0 = rtm.find("fmPivotSampleTick(now);\n#endif\n\n  runFmLoopBody(now);");
  CHECK(s0 != std::string::npos);
  CHECK(rtm.find("if (!fm_pv_stepped) fmPivotLearnTick(now, false, 0.0f);", s0) != std::string::npos);
  // the build switch: defined in BREmote_V2_Rx.h, default 1, ahead of every Common/ header but SerialTee
  const size_t d = hdr.find("#ifndef FM_PIVOT_LEARN_ENABLED\n#define FM_PIVOT_LEARN_ENABLED 1\n#endif");
  CHECK(d != std::string::npos && d < hdr.find("#include \"../Common/DifferentialMixer.h\""));
  CHECK(countOf(hdr, "#define FM_PIVOT_LEARN_ENABLED") == 1 && countOf(rtm, "#define FM_PIVOT_LEARN_ENABLED") == 0);
  // H-0: the IMU poll's gate 1 also opens while Follow-Me is armed (and only that line changes)
  CHECK(vesc.find("if (!logImuPollWanted() && !fmPivotImuPollWanted()) { g_imu2_miss_streak = 0; return; }") != std::string::npos);
  CHECK(rtm.find("return (fm_state != FM_IDLE) && !rtm_rx_active.load();") != std::string::npos);
  printf("2. RTMState.ino holds kFmPivotCfg, 4 mph and 8 km/h; cap 4, the learn step before the chain, WRONG WAY into R2-S's end;\n"
         "   the switch defaults to 1 in BREmote_V2_Rx.h before the Common/ headers; VESC.ino's gate 1 adds Follow-Me armed\n");
}

// ======================================================================================
// 3. THE KILL SWITCH (red edit FM_PIVOT_LEARN_ENABLED 0): the 0 build is the base commit's code
// ======================================================================================
// stripSwitch - what the preprocessor keeps of `src` when FM_PIVOT_LEARN_ENABLED is 0 (and defined, as -D...=0 does):
// "#if FM_PIVOT_LEARN_ENABLED" and "#ifndef FM_PIVOT_LEARN_ENABLED" bodies dropped, their "#else" bodies kept,
// "#if !FM_PIVOT_LEARN_ENABLED" bodies kept; every other directive passes through. Returns false on a construct it does not
// handle (an #elif inside the switch's block, or unbalanced blocks).
static bool stripSwitch(const std::string &src, std::string *out)
{
  struct Fr { bool sw; bool keep; };
  std::vector<Fr> st;
  auto keeping = [&]() { for (const Fr &f : st) if (f.sw && !f.keep) return false; return true; };
  size_t p = 0;
  while (p < src.size()) {
    size_t e = src.find('\n', p); if (e == std::string::npos) e = src.size();
    const std::string line = src.substr(p, e - p);
    p = e + 1;
    size_t i = 0; while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) i++;
    const std::string t = line.substr(i);
    auto starts = [&](const char *k) { return t.compare(0, strlen(k), k) == 0; };
    if (starts("#if FM_PIVOT_LEARN_ENABLED") || starts("#ifndef FM_PIVOT_LEARN_ENABLED")) { st.push_back({ true, false }); continue; }
    if (starts("#if !FM_PIVOT_LEARN_ENABLED")) { st.push_back({ true, true }); continue; }
    if (starts("#if") ) { const bool k = keeping(); st.push_back({ false, true }); if (k) { out->append(line); out->push_back('\n'); } continue; }
    if (starts("#elif")) { if (!st.empty() && st.back().sw) return false; if (keeping()) { out->append(line); out->push_back('\n'); } continue; }
    if (starts("#else")) {
      if (st.empty()) return false;
      if (st.back().sw) { st.back().keep = !st.back().keep; continue; }
      if (keeping()) { out->append(line); out->push_back('\n'); }
      continue;
    }
    if (starts("#endif")) {
      if (st.empty()) return false;
      const Fr f = st.back(); st.pop_back();
      if (!f.sw && keeping()) { out->append(line); out->push_back('\n'); }
      continue;
    }
    if (keeping()) { out->append(line); out->push_back('\n'); }
  }
  return st.empty();
}
// codeTokens - the code with every comment taken out (string and character literals respected) and all whitespace runs
// collapsed to one space: two files with equal results compile to the same thing.
static std::string codeTokens(const std::string &s)
{
  std::string o; o.reserve(s.size());
  size_t i = 0; const size_t n = s.size();
  bool space = false;
  auto put = [&](char c) {
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v') { space = true; return; }
    if (space && !o.empty()) o.push_back(' ');
    space = false; o.push_back(c);
  };
  while (i < n) {
    const char c = s[i];
    if (c == '/' && i + 1 < n && s[i + 1] == '/') { while (i < n && s[i] != '\n') i++; space = true; continue; }
    if (c == '/' && i + 1 < n && s[i + 1] == '*') { i += 2; while (i + 1 < n && !(s[i] == '*' && s[i + 1] == '/')) i++; i += 2; space = true; continue; }
    if (c == '"' || c == '\'') {
      const char q = c; put(c); i++;
      while (i < n && s[i] != q) { if (s[i] == '\\' && i + 1 < n) { o.push_back(s[i]); i++; } o.push_back(s[i]); i++; }
      if (i < n) { o.push_back(s[i]); i++; }
      continue;
    }
    put(c); i++;
  }
  return o;
}
#ifdef _WIN32
static const char *kNullRedirect = " 2>NUL";        // popen() runs cmd.exe here
#else
static const char *kNullRedirect = " 2>/dev/null";
#endif
static bool gitShow(const char *base, const std::string &path, std::string *out)
{
  const std::string cmd = std::string("git -C ../.. show \"") + base + ":" + path + "\"" + kNullRedirect;
  FILE *p = popen(cmd.c_str(), "rb");
  if (!p) return false;
  char buf[65536]; size_t k; out->clear();
  while ((k = fread(buf, 1, sizeof(buf), p)) > 0) out->append(buf, k);
  const int rc = pclose(p);
  std::string o; for (char c : *out) if (c != '\r') o.push_back(c);
  *out = o;
  return rc == 0;
}
static std::vector<std::string> gitLsFiles(const char *base, const char *dir)
{
  std::vector<std::string> v;
  const std::string cmd = std::string("git -C ../.. ls-tree -r --name-only ") + base + " -- " + dir + kNullRedirect;
  FILE *p = popen(cmd.c_str(), "r");
  if (!p) return v;
  char line[1024];
  while (fgets(line, sizeof(line), p)) { std::string s(line); while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back(); if (!s.empty()) v.push_back(s); }
  pclose(p);
  return v;
}

static void testKillSwitch()
{
  int files = 0, stripped = 0;
  const char *dirs[2] = { "Source/V2_Integration_Rx", "Source/Common" };
  for (const char *dir : dirs) {
    const std::vector<std::string> list = gitLsFiles(g_base, dir);
    CHECK(!list.empty());                                       // the base commit must be reachable from this worktree
    for (const std::string &path : list) {
      // source files only (the folder also holds released binaries, which this unit does not touch)
      const size_t dot = path.rfind('.');
      const std::string ext = (dot == std::string::npos) ? "" : path.substr(dot);
      if (ext != ".ino" && ext != ".h" && ext != ".cpp" && ext != ".c" && ext != ".hpp") continue;
      std::string old_src;
      CHECK(gitShow(g_base, path, &old_src));
      const std::string now_src = readFile(("../../" + path).c_str());
      CHECK(!now_src.empty() || old_src.empty());
      files++;
      if (now_src == old_src) continue;                         // untouched (CRLF aside)
      std::string kept;
      CHECK(stripSwitch(now_src, &kept));
      const std::string a = codeTokens(kept), b = codeTokens(old_src);
      if (a != b) {
        size_t k = 0; while (k < a.size() && k < b.size() && a[k] == b[k]) k++;
        fprintf(stderr, "KILL SWITCH: %s differs from %s at token offset %zu:\n  now:  ...%s...\n  base: ...%s...\n", path.c_str(), g_base,
                k, a.substr(k > 80 ? k - 80 : 0, 200).c_str(), b.substr(k > 80 ? k - 80 : 0, 200).c_str());
      }
      CHECK(a == b);
      stripped++;
    }
  }
  // the one new file is only ever included under the switch
  const std::string hdr = readFile("../../Source/V2_Integration_Rx/BREmote_V2_Rx.h");
  std::string kept; CHECK(stripSwitch(hdr, &kept));
  CHECK(hdr.find("#include \"../Common/FollowMePivot.h\"") != std::string::npos && codeTokens(kept).find("FollowMePivot.h") == std::string::npos);
  printf("3. KILL SWITCH (red edit FM_PIVOT_LEARN_ENABLED 0): %d RX / Common files at %s; %d changed, and each of those with the\n"
         "   switch's blocks taken out is the base file token for token (comments aside); FollowMePivot.h only under the switch\n",
         files, g_base, stripped);
}

// ======================================================================================
// 4. SIMULATION
// ======================================================================================
// THE PIVOT PLANT (plausible, not bench-measured): the throttle the buggy gets (applied, after the caps and a 0.5 s
// output ramp) passes the VESC's own ramp (0.4 s up / 0.2 s down, full scale) and its 8-count input deadband; with the
// one-motor pivot the hull's yaw-rate target is G x (throttle - 8) x (command - 127) / 127 (mirrored: minus), reached with
// a first-order momentum tau. G is set per run from the buggy's yaw rate at full lock and its own fm_align_cap (8-48 deg/s;
// measured 10-16 deg/s at cap 50). THE CONTROLLER: the Normal preset's P term (error clamp 90 deg, kp 1) on a heading 0.3 s
// late with +/-2 deg noise; aligning above 45 deg (the pivot boost requested); cap 5 the 1.5 s engage ramp when the pivot
// starts at the ACTIVE edge. THE IMU: polled every 500 ms +/-25 ms, 3 % of polls lost, yaw = mount x heading + 0.3 deg
// noise + a slow drift. The RX's tick order: sample tick, WRONG WAY (last tick's command), steering, learn step, cap chain.
struct SimCfg {
  uint8_t ac;            // fm_align_cap
  double  rate_at_cap;   // deg/s at full lock and fm_align_cap (sets G)
  double  tau_s;         // yaw momentum
  int     target;        // 0 static, 1 swinging +swing for 4 s, 2 swinging -swing, 3 swinging across behind
  double  swing_dps;
  int     trig;          // 0 full, 1 pumping 255 / 128 each second, 2 light 100, 3 the owner's pump to 51
  bool    at_edge;       // the pivot starts at the ACTIVE edge (engage ramp from 0)
  int     mount;         // +1 / -1: the IMU's yaw against clockwise
  int8_t  sign;          // the calibration sign the module holds (0 = not calibrated)
  bool    mirrored;      // red edit (c): the buggy turns against its command
  bool    carry_hi;      // start the run with the learned memory at the top of its bounds
  bool    h_on;          // false = unit H compiled out (cap 4 = fm_align_cap, no WRONG WAY)
  const FmPivotCfg *cfg;
};
struct RunOut {
  int pivots, nt, ww, aligned, timeouts;
  int bound_viol;        // ticks the cap left its bounds (or went above the trigger / 128 / fm_align_cap uncalibrated)
  int max_learned;       // the highest learned cap seen
  int max_cap_after_flip;// the highest cap in force after the command first changed side in a pivot
  bool any_flip;
  double ww_t;           // the first WRONG WAY: seconds from its pivot's start (-1 none)
  double first_side_s;   // the first pivot: how long the command stayed on its first side
  double flip_t;         // the first pivot: when the command first changed side (-1 = never, while the pivot ran)
  double nt_abs_cmd;     // report only: the sum over NOT TURNING fires of |command - 127| on the firing tick ...
  double nt_abs_err;     // ... and of |heading error| (deg)
};

static int trigAt(int kind, double ts)
{
  if (kind == 0) return 255;
  if (kind == 1) return (((int)ts) % 2) ? 128 : 255;
  if (kind == 2) return 100;
  return 51;
}

static RunOut simRun(const SimCfg &c, uint64_t seed, int n_pivots, double e0_fixed)
{
  Rng rng(seed);
  RunOut o; memset(&o, 0, sizeof(o)); o.ww_t = -1.0; o.first_side_s = -1.0; o.flip_t = -1.0; o.max_cap_after_flip = 0;
  const FmPivotCfg *cfg = c.cfg;
  FmPvRate rate; fmPvRateReset(&rate);
  FmPvLearn L; fmPvLearnReset(&L);
  FmPvWw W = { 0, 0, 0, 0 };
  uint8_t lo, hi; fmPvBounds(c.ac, c.sign != 0, cfg, &lo, &hi);
  if (c.carry_hi) { L.have_last = true; L.last = hi; }
  const double G = c.rate_at_cap / (double)(c.ac > 8 ? c.ac - 8 : 1);
  double h = rng.uni() * 360.0, r = 0.0, thr_v = 0.0, applied_f = 0.0, drift = (rng.uni() - 0.5) * 0.2, drift_acc = 0.0;
  uint32_t t = T0 + (uint32_t)(rng.uni() * 1000.0);
  uint32_t next_imu = t + 100;
  float imu_yaw = 0.0f; uint32_t imu_ms = 0; bool imu_ok = false;
  std::vector<double> hist(64, h);   // heading history at 10 ms, for the controller's 0.3 s lag
  size_t hp = 0;
  for (int pv = 0; pv < n_pivots; pv++) {
    const double e0 = (e0_fixed > -999.0) ? e0_fixed : ((rng.uni() < 0.5) ? -1.0 : 1.0) * (50.0 + rng.uni() * 130.0);
    const double b0 = h + e0;
    const uint32_t pv_start = t;
    const bool edge = c.at_edge || pv == 0;
    const uint32_t engage_ms = edge ? t : t - 60000u;
    bool pivot_prev = false, aligned_once = false, ended = false;
    uint32_t aligned_since = 0;
    int last_side = 0; bool flipped = false; bool first_side_open = (pv == 0); int first_side = 0;
    int cmd_prev = 0, cmd = 0;
    uint8_t fm_cap = 0;
    o.pivots++;
    for (int step = 0; step < 2500 && !ended; step++) {   // 25 s at 10 ms
      const double ts = (double)(uint32_t)(t - pv_start) / 1000.0;
      // ---- the target bearing ----
      double b = b0;
      const double sw = (ts < 4.0) ? ts : 4.0;
      if (c.target == 1) b = b0 + c.swing_dps * sw;
      if (c.target == 2) b = b0 - c.swing_dps * sw;
      if (c.target == 3) { const double side = (e0 >= 0) ? 1.0 : -1.0; b = b0 + side * c.swing_dps * sw; }   // swings on across behind
      // ---- the 10 Hz Follow-Me tick ----
      if (step % 10 == 0) {
        const bool newer = fmPvIngest(&rate, imu_ok, imu_ms, imu_yaw, true, cfg);
        const bool capable = fmPvCapable(&rate, t, cfg);
        // WRONG WAY, on last tick's command, inside hard_ok: ACTIVE, 0.5 s past the edge, last tick a commanded pivot
        if (c.h_on) {
          const bool judged = ((uint32_t)(t - engage_ms) >= 500u) && pivot_prev;
          if (fmPvWwStep(&W, t, judged, (int16_t)cmd_prev, c.sign, capable, newer, &rate, cfg)) {
            o.ww++;
            if (o.ww_t < 0.0) o.ww_t = ts;
            fmPvMarkEvent(&L, FMPV_WRONG_WAY, t);
            ended = true;                      // Follow-Me ends (sticky cap, ARMED): this pivot is over
            fmPvLearnStep(&L, t, false, c.ac, 255, 0, 0, capable, c.sign, false, &rate, cfg);
            break;
          }
        }
        // the steering: the heading 0.3 s late with noise, the P term
        const double h_meas = hist[(hp + 64 - 30) % 64] + 2.0 * rng.gauss();
        const double e = wrapd(b - h_meas);
        double ec = e; if (ec > 90.0) ec = 90.0; if (ec < -90.0) ec = -90.0;
        int u = (int)lround(127.0 + ec / 90.0 * 127.0); if (u < 0) u = 0; if (u > 254) u = 254;
        cmd = u - 127;
        const bool aligning = fabs(e) > 45.0;
        // the learn step (H on) or fm_align_cap (H off), then the cap chain: cap 4 while aligning, cap 5 the engage ramp
        const int trig = trigAt(c.trig, ts);
        uint8_t cap4 = c.ac;
        if (c.h_on) {
          const FmPvOut po = fmPvLearnStep(&L, t, aligning, c.ac, (uint8_t)trig, (uint8_t)lround(applied_f), (int16_t)cmd, capable,
                                           c.sign, newer, &rate, cfg);
          cap4 = po.cap;
          if (po.nt_fired) { o.nt++; o.nt_abs_cmd += fabs((double)cmd); o.nt_abs_err += fabs(e); }
          if (aligning && capable) {
            // the bounds, the trigger, 128, and fm_align_cap while uncalibrated
            uint8_t l2, h2; fmPvBounds(c.ac, c.sign != 0, cfg, &l2, &h2);
            const int lim = (h2 < trig) ? h2 : trig;
            if ((int)po.cap > lim || (c.sign == 0 && po.cap > c.ac)) o.bound_viol++;
            if (cfg->cap_ceiling <= 128 && po.cap > 128) o.bound_viol++;
            if (L.cap > o.max_learned) o.max_learned = L.cap;
          }
        }
        int cap = aligning ? cap4 : 255;
        if (edge) { const double ramp = 255.0 * (double)(uint32_t)(t - engage_ms) / 1500.0; if (ramp < cap) cap = (int)ramp; }
        fm_cap = (uint8_t)((cap < 0) ? 0 : (cap > 255 ? 255 : cap));
        pivot_prev = aligning;
        // the command's side, for the change-of-side metrics
        const int side = (cmd >= 32) ? 1 : (cmd <= -32) ? -1 : 0;
        if (side != 0) {
          if (first_side_open && first_side == 0) first_side = side;
          if (last_side != 0 && side != last_side) {
            flipped = true; o.any_flip = true;
            if (first_side_open) { o.first_side_s = ts; o.flip_t = ts; first_side_open = false; }
          }
          last_side = side;
        }
        if (flipped && aligning && (int)cap4 > o.max_cap_after_flip) o.max_cap_after_flip = cap4;
        // the pivot is over once the buggy has pointed inside the align band for 1 s
        if (!aligning) { if (aligned_since == 0) aligned_since = t | 1u; aligned_once = true; } else aligned_since = 0;
        if (aligned_since != 0 && (uint32_t)(t - aligned_since) >= 1000u) ended = true;
        cmd_prev = cmd;
      }
      // ---- the IMU poll (getVescLoop, later in the same loop pass: the next tick sees the sample) ----
      if ((int32_t)(t - next_imu) >= 0) {
        if (rng.uni() >= 0.03) {
          drift_acc += drift * 0.5;
          imu_yaw = (float)wrapd((double)c.mount * h + drift_acc + 0.3 * rng.gauss());
          imu_ms = t; imu_ok = true;
        }
        next_imu = t + 475 + (uint32_t)(rng.uni() * 50.0);
      }
      // ---- the 100 Hz PWM pass: the trigger under the caps, the 0.5 s output ramp, the motor ----
      const int trig = trigAt(c.trig, ts);
      const double target = (trig < fm_cap) ? trig : fm_cap;
      if (target > applied_f) { applied_f += 255.0 * 0.01 / 0.5; if (applied_f > target) applied_f = target; } else applied_f = target;
      // the VESC's own ramp and deadband
      if (applied_f > thr_v) { thr_v += 255.0 * 0.01 / 0.4; if (thr_v > applied_f) thr_v = applied_f; }
      else { thr_v -= 255.0 * 0.01 / 0.2; if (thr_v < applied_f) thr_v = applied_f; }
      double eff = thr_v - 8.0; if (eff < 0.0) eff = 0.0;
      const double defl = (double)cmd / 127.0;
      double turn = G * eff;                                // the one-motor pivot: rate at full lock
      if (fm_cap == 255 && turn > 25.0) turn = 25.0;        // aligned: driving off at the normal influence (bounded)
      const double rstar = (c.mirrored ? -1.0 : 1.0) * turn * defl;
      r += (rstar - r) * 0.01 / c.tau_s;
      h += r * 0.01;
      hp = (hp + 1) % 64; hist[hp] = h;
      t += 10;
    }
    if (pv == 0 && first_side_open) o.first_side_s = (double)(uint32_t)(t - pv_start) / 1000.0;   // never changed side
    if (aligned_once && !o.ww) o.aligned++;
    if (!aligned_once && !o.ww) o.timeouts++;
    // between pivots: 1.5 s with the hull settled (no turn) - the learn step outside the envelope ends the pivot (its cap
    // remembered); the IMU keeps sampling, as it does on the RX, so the next pivot starts with a live rate stream
    r = 0.0;
    for (int k = 0; k < 15; k++) {
      const bool newer = fmPvIngest(&rate, imu_ok, imu_ms, imu_yaw, true, cfg);
      fmPvLearnStep(&L, t, false, c.ac, 255, 0, 0, fmPvCapable(&rate, t, cfg), c.sign, newer, &rate, cfg);
      fmPvWwStep(&W, t, false, 0, c.sign, false, false, &rate, cfg);
      for (int s10 = 0; s10 < 10; s10++) {
        if ((int32_t)(t - next_imu) >= 0) {
          if (rng.uni() >= 0.03) {
            drift_acc += drift * 0.5;
            imu_yaw = (float)wrapd((double)c.mount * h + drift_acc + 0.3 * rng.gauss());
            imu_ms = t; imu_ok = true;
          }
          next_imu = t + 475 + (uint32_t)(rng.uni() * 50.0);
        }
        hp = (hp + 1) % 64; hist[hp] = h;
        t += 10;
      }
    }
    if (o.ww && c.mirrored) break;
  }
  return o;
}

// 4a. THE CALIBRATION ON RIDES: turns of 10-30 deg/s at 10-25 km/h with straights between (heading wander under 1 deg/s),
// the course 0.4 s late with 2 or 5 deg of noise per sample, the hull ahead of the track by 0.4 s of its turn rate (the
// skid); some slow turning stretches under 8 km/h that must not vote. Returns the turn count when the sign first equalled
// the mount (-1 never), whether it was ever the wrong sign, the final sign and the votes.
struct CalOut { int turns_to_cal; bool ever_wrong; int sign_end; uint32_t votes; };
static CalOut calRide(uint64_t seed, int mount, bool straight_only, double minutes, double cog_sigma)
{
  Rng rng(seed);
  FmPvRate rate; fmPvRateReset(&rate); FmPvCal k; fmPvCalReset(&k);
  CalOut o = { -1, false, 0, 0 };
  double track = rng.uni() * 360.0, track_rate = 0.0, speed = 15.0;
  std::vector<double> hist(64, track); size_t hp = 0;
  uint32_t t = T0, next_imu = T0 + 100;
  double seg_left = 0.0, drift = (rng.uni() - 0.5) * 0.2, drift_acc = 0.0;
  const int steps = (int)(minutes * 6000.0);
  for (int s = 0; s < steps; s++) {
    if (seg_left <= 0.0) {
      const double p = rng.uni();
      if (straight_only || p < 0.45) { track_rate = (rng.uni() - 0.5) * 2.0; seg_left = 3.0 + rng.uni() * 7.0; speed = 10.0 + rng.uni() * 15.0; }
      else if (p < 0.85) { track_rate = ((rng.uni() < 0.5) ? -1.0 : 1.0) * (10.0 + rng.uni() * 20.0); seg_left = 2.0 + rng.uni() * 4.0; speed = 10.0 + rng.uni() * 15.0; }
      else { track_rate = ((rng.uni() < 0.5) ? -1.0 : 1.0) * (6.0 + rng.uni() * 20.0); seg_left = 2.0 + rng.uni() * 4.0; speed = 3.0 + rng.uni() * 4.5; }
    }
    seg_left -= 0.01;
    track += track_rate * 0.01;
    hp = (hp + 1) % 64; hist[hp] = track;
    if ((int32_t)(t - next_imu) >= 0) {
      drift_acc += drift * 0.5;
      const double hull = track + 0.4 * track_rate;   // the skid: the hull turned ahead of the track by 0.4 s of its rate
      if (rng.uni() >= 0.03) {
        const float yaw = (float)wrapd((double)mount * hull + drift_acc + 0.3 * rng.gauss());
        if (fmPvIngest(&rate, true, t, yaw, true, &kCfg)) {
          const double cog = hist[(hp + 64 - 40) % 64] + cog_sigma * rng.gauss();   // the course 0.4 s late, noisy
          const bool cog_ok = speed >= 8.0;
          fmPvCalStep(&k, &rate, cog_ok, (float)fmod(cog + 36000.0, 360.0), &kCfg);
          if (k.sign != 0 && k.sign != mount) o.ever_wrong = true;
          if (k.sign == mount && o.turns_to_cal < 0) o.turns_to_cal = (int)k.turns;
        }
      }
      next_imu = t + 475 + (uint32_t)(rng.uni() * 50.0);
    }
    t += 10;
  }
  o.sign_end = k.sign; o.votes = k.turns;
  return o;
}

static void testSimulation()
{
  // ---- 4a. calibration ----
  {
    int worst = 0, never = 0, wrong = 0, runs = 0;
    for (int mount = -1; mount <= 1; mount += 2)
      for (uint64_t seed = 1; seed <= 10; seed++)
        for (double sig = 2.0; sig <= 5.01; sig += 3.0) {
          const CalOut co = calRide(seed * 7 + (uint64_t)(mount + 1) * 1000 + (uint64_t)sig, mount, false, 10.0, sig);
          runs++;
          if (co.ever_wrong) wrong++;
          if (co.turns_to_cal < 0) never++; else if (co.turns_to_cal > worst) worst = co.turns_to_cal;
        }
    printf("4a. calibration on simulated rides (10 min, both mounts, course noise 2 and 5 deg, 10 seeds = %d rides): wrong sign %d (HARD 0),\n"
           "    not calibrated by the end %d (HARD 0), worst turns to calibrate %d\n", runs, wrong, never, worst);
    HARD(wrong == 0, "4a calibration: never the wrong sign");
    HARD(never == 0, "4a calibration: calibrated by the end of every ride");
    int cal_straight = 0, votes_straight = 0;
    for (uint64_t seed = 1; seed <= 10; seed++) {
      const CalOut co = calRide(seed + 500, 1, true, 10.0, 5.0);
      if (co.sign_end != 0) cal_straight++;
      votes_straight += (int)co.votes;
    }
    printf("    straight riding only (heading wander under 1 deg/s, course noise 5 deg, 10 x 10 min): calibrated %d (HARD 0), votes %d\n",
           cal_straight, votes_straight);
    HARD(cal_straight == 0, "4a calibration: straight riding never calibrates");
  }
  // ---- 4b. THE FALSE-FIRE GATE (D-CD1): healthy pivots with noise, 10 seeds, 10 pivots per run ----
  const uint8_t caps[3] = { 50, 30, 80 };
  const double rates[6] = { 8.0, 12.0, 16.0, 24.0, 32.0, 48.0 };
  const int targets[4] = { 0, 1, 2, 3 };
  long runs = 0, pivots = 0, nt = 0, ww = 0, viol = 0, aligned = 0, timeouts = 0; int max_learned = 0;
  long nt50 = 0, ww50 = 0, runs50 = 0;
  // report only: where the NOT TURNING fires fall (by the buggy's rate at full lock, trigger, target, cap, calibration,
  // engagement edge), and the command and heading error on the firing tick
  long nt_rate[6] = { 0 }, nt_trig[4] = { 0 }, nt_tg[4] = { 0 }, nt_cap[3] = { 0 }, nt_cal[2] = { 0 }, nt_edge[2] = { 0 };
  double nt_cmd = 0.0, nt_err = 0.0;
  for (uint8_t ac : caps)
    for (double rt : rates)
      for (int tg : targets)
        for (int trig = 0; trig < 4; trig++)
          for (int edge = 0; edge < 2; edge++)
            for (int mount = -1; mount <= 1; mount += 2)
              for (int calib = 0; calib < 2; calib++)
                for (uint64_t seed = 1; seed <= 10; seed++) {
                  SimCfg c = { ac, rt, 0.3, tg, (tg == 3) ? 25.0 : 15.0, trig, edge == 1, mount, (int8_t)(calib ? mount : 0),
                               false, false, true, &kCfg };
                  const RunOut o = simRun(c, seed * 1000003ull + (uint64_t)(ac * 7 + (int)rt * 13 + tg * 17 + trig * 19 + edge * 23 + calib * 29 + (mount + 1) * 31),
                                          10, -1000.0);
                  runs++; pivots += o.pivots; nt += o.nt; ww += o.ww; viol += o.bound_viol; aligned += o.aligned; timeouts += o.timeouts;
                  if (o.max_learned > max_learned) max_learned = o.max_learned;
                  if (ac == 50) { runs50++; nt50 += o.nt; ww50 += o.ww; }
                  {
                    int ri = 0; while (ri < 5 && rates[ri] != rt) ri++;
                    const int ci = (ac == 50) ? 0 : (ac == 30) ? 1 : 2;
                    nt_rate[ri] += o.nt; nt_trig[trig] += o.nt; nt_tg[tg] += o.nt; nt_cap[ci] += o.nt; nt_cal[calib] += o.nt; nt_edge[edge] += o.nt;
                    nt_cmd += o.nt_abs_cmd; nt_err += o.nt_abs_err;
                  }
                }
  printf("4b. FALSE-FIRE GATE (D-CD1), healthy pivots with noise: %ld runs x 10 pivots = %ld pivots (fm_align_cap 30 / 50 / 80; 8-48 deg/s\n"
         "    at the cap; target static, swinging +/-15 deg/s, swinging 25 deg/s across behind; full, pumped 255/128, 100 and 51 triggers;\n"
         "    at the ACTIVE edge and mid-engagement; both mounts, calibrated and not; 10 seeds):\n"
         "    NOT TURNING %ld, WRONG WAY %ld (HARD 0 / 0; at 50: %ld / %ld in %ld runs); bound violations %ld (HARD 0); highest learned %d;\n"
         "    pivots that reached the target %ld, timed out %ld\n", runs, pivots, nt, ww, nt50, ww50, runs50, viol, max_learned, aligned, timeouts);
  if (nt > 0) {
    printf("    report only - the NOT TURNING fires by the buggy's rate at full lock (8/12/16/24/32/48 deg/s): %ld %ld %ld %ld %ld %ld;\n"
           "    by trigger (full / pumped / 100 / 51): %ld %ld %ld %ld; by target (static / +15 / -15 / across behind): %ld %ld %ld %ld;\n"
           "    by fm_align_cap (50 / 30 / 80): %ld %ld %ld; uncalibrated / calibrated: %ld %ld; mid-engagement / at the edge: %ld %ld;\n"
           "    on the firing tick, mean |command - 127| %.1f counts (127 = full lock) and mean |heading error| %.1f deg\n",
           nt_rate[0], nt_rate[1], nt_rate[2], nt_rate[3], nt_rate[4], nt_rate[5], nt_trig[0], nt_trig[1], nt_trig[2], nt_trig[3],
           nt_tg[0], nt_tg[1], nt_tg[2], nt_tg[3], nt_cap[0], nt_cap[1], nt_cap[2], nt_cal[0], nt_cal[1], nt_edge[0], nt_edge[1],
           nt_cmd / (double)nt, nt_err / (double)nt);
  }
  HARD(nt == 0, "4b D-CD1: NOT TURNING never fires on a healthy pivot");
  HARD(ww == 0, "4b D-CD1: WRONG WAY never fires on a healthy pivot");
  HARD(viol == 0 && max_learned <= 128, "4b bounds: the cap never leaves its bounds, the trigger or 128");
  // ---- 4c. THE RED EDITS ----
  // (a) calibration skipped: the module holds +1 while the VESC is mounted the other way up - healthy pivots must false-fire
  {
    long rr = 0, fires = 0;
    for (double rt : rates) for (int tg : targets) for (uint64_t seed = 1; seed <= 10; seed++) {
      SimCfg c = { 50, rt, 0.3, tg, 15.0, 0, false, -1, 1, false, false, true, &kCfg };
      const RunOut o = simRun(c, seed * 7919ull + (uint64_t)(rt * 3 + tg), 10, -1000.0);
      rr++; fires += o.ww;
    }
    printf("4c(a). RED EDIT calibration skipped (sign forced +1, VESC mounted the other way up): %ld false WRONG WAY ends over %ld\n"
           "       healthy runs of 10 pivots (must be > 0: the calibration is load-bearing)\n", fires, rr);
    HARD(fires > 0, "4c(a) red edit calibration skipped: must false-fire");
  }
  // (b) bounds removed: a slow buggy, calibrated - a cap above 80 must be seen
  {
    FmPivotCfg nb = kCfg; nb.lo_frac = 0.0f; nb.cap_floor = 0; nb.hi_frac = 100.0f; nb.cap_ceiling = 255;
    int mx = 0;
    for (uint64_t seed = 1; seed <= 10; seed++) {
      SimCfg c = { 50, 8.0, 0.3, 0, 15.0, 0, false, 1, 1, false, false, true, &nb };
      const RunOut o = simRun(c, seed, 10, -1000.0);
      if (o.max_learned > mx) mx = o.max_learned;
    }
    SimCfg c = { 50, 8.0, 0.3, 0, 15.0, 0, false, 1, 1, false, false, true, &kCfg };
    int mx_ok = 0; for (uint64_t seed = 1; seed <= 10; seed++) { const RunOut o = simRun(c, seed, 10, -1000.0); if (o.max_learned > mx_ok) mx_ok = o.max_learned; }
    printf("4c(b). RED EDIT bounds removed (slow buggy, 8 deg/s at 50, calibrated): highest learned cap %d (must be > 80); with the\n"
           "       bounds: %d\n", mx, mx_ok);
    HARD(mx > 80 && mx_ok <= 80, "4c(b) red edit bounds removed: a cap above 80 must be seen (and none with the bounds)");
  }
  // (c) mirrored steering during pivots, calibrated: WRONG WAY ends it.
  //   HARD 1: every run whose command did not change side in its first 4.5 s is ended by WRONG WAY within 4.5 s of its start.
  //   HARD 2: in every run, once the command has changed side the cap in force is never above fm_align_cap (CHATTER).
  //   Recorded: the runs that change side sooner (a target near 180 deg, a fast buggy) and are not ended by WRONG WAY.
  {
    long rr = 0, ended = 0, hard_class = 0, hard_ended = 0, chatter = 0, chatter_ended = 0, over_after_flip = 0;
    double t_worst = 0.0, t_sum = 0.0; long t_n = 0;
    long hard_by_e0[19] = { 0 }, ended_by_e0[19] = { 0 }, runs_by_e0[19] = { 0 };
    long miss_rate[6] = { 0 }, miss_trig[4] = { 0 }, miss_cap[3] = { 0 }, miss_edge[2] = { 0 }, miss_late = 0;   // report only: HARD 1's misses
    for (uint8_t ac : caps) for (double rt : rates) for (int trig = 0; trig < 4; trig++) for (int edge = 0; edge < 2; edge++)
      for (int carry = 0; carry < 2; carry++) for (int e10 = 5; e10 <= 18; e10++) for (int sd = -1; sd <= 1; sd += 2)
        for (uint64_t seed = 1; seed <= 10; seed++) {
          SimCfg c = { ac, rt, 0.3, 0, 15.0, trig, edge == 1, 1, 1, true, carry == 1, true, &kCfg };
          const double e0 = (double)sd * (e10 == 18 ? 179.0 : e10 * 10.0);
          const RunOut o = simRun(c, seed * 104729ull + (uint64_t)(ac + (int)rt * 5 + trig * 11 + edge * 13 + carry * 17 + e10 * 19 + (sd + 1) * 23), 1, e0);
          rr++; runs_by_e0[e10]++;
          if (o.ww) { ended++; ended_by_e0[e10]++; t_sum += o.ww_t; t_n++; if (o.ww_t > t_worst) t_worst = o.ww_t; }
          if (o.max_cap_after_flip > ac) over_after_flip++;
          const bool flipped_early = (o.flip_t >= 0.0 && o.flip_t < 4.5);
          if (!flipped_early) {
            hard_class++; hard_by_e0[e10]++;
            if (o.ww && o.ww_t < 4.5) hard_ended++;
            else {
              int ri = 0; while (ri < 5 && rates[ri] != rt) ri++;
              miss_rate[ri]++; miss_trig[trig]++; miss_cap[(ac == 50) ? 0 : (ac == 30) ? 1 : 2]++; miss_edge[edge]++;
              if (o.ww) miss_late++;
            }
          }
          else { chatter++; if (o.ww) chatter_ended++; }
        }
    printf("4c(c). RED EDIT mirrored steering during pivots (calibrated; caps 30/50/80; 8-48 deg/s; 4 triggers; edge / mid; memory at\n"
           "       the top or not; start error 50-179 deg both sides; 10 seeds = %ld runs): WRONG WAY ended %ld (%.1f %%), mean %.2f s,\n"
           "       worst %.2f s after the pivot began\n", rr, ended, 100.0 * ended / (double)rr, t_n ? t_sum / t_n : 0.0, t_worst);
    printf("       HARD 1 - command on one side for its first 4.5 s: %ld runs, ended by WRONG WAY within 4.5 s: %ld\n", hard_class, hard_ended);
    printf("       (by start error, runs in that class: ");
    for (int e10 = 5; e10 <= 18; e10++) printf("%d: %ld%s", (e10 == 18 ? 179 : e10 * 10), hard_by_e0[e10], e10 < 18 ? ", " : ")\n");
    if (hard_ended < hard_class)
      printf("       report only - HARD 1's %ld misses (%ld of them ended later): by the buggy's rate at full lock (8/12/16/24/32/48 deg/s)\n"
             "       %ld %ld %ld %ld %ld %ld; by trigger (full / pumped / 100 / 51) %ld %ld %ld %ld; by fm_align_cap (50 / 30 / 80) %ld %ld %ld;\n"
             "       mid-engagement / at the edge %ld %ld\n", hard_class - hard_ended, miss_late,
             miss_rate[0], miss_rate[1], miss_rate[2], miss_rate[3], miss_rate[4], miss_rate[5],
             miss_trig[0], miss_trig[1], miss_trig[2], miss_trig[3], miss_cap[0], miss_cap[1], miss_cap[2], miss_edge[0], miss_edge[1]);
    printf("       HARD 2 - the cap above fm_align_cap after a change of side: %ld runs (must be 0)\n", over_after_flip);
    printf("       RECORDED - changed side sooner (the steady mirrored state near 180 deg): %ld runs, of which WRONG WAY ended %ld;\n"
           "       the rest run at fm_align_cap or less, as the flown code does, until the release, the pivot stall rule and R2.\n", chatter, chatter_ended);
    printf("       by start error (deg): ");
    for (int e10 = 5; e10 <= 18; e10++) printf("%d: %ld/%ld ended%s", (e10 == 18 ? 179 : e10 * 10), ended_by_e0[e10], runs_by_e0[e10], e10 < 18 ? ", " : "\n");
    HARD(hard_class > 0 && hard_ended == hard_class, "4c(c) HARD 1: WRONG WAY ends every one-sided mirrored pivot within 4.5 s");
    HARD(over_after_flip == 0, "4c(c) HARD 2: never above fm_align_cap after a change of side");
    // the same mirrored runs with unit H compiled out: nothing in this unit can end them (the flown behaviour)
    long off_ended = 0, off_rr = 0;
    for (uint64_t seed = 1; seed <= 10; seed++) for (int e10 = 5; e10 <= 18; e10 += 4) {
      SimCfg c = { 50, 16.0, 0.3, 0, 15.0, 0, false, 1, 1, true, false, false, &kCfg };
      const RunOut o = simRun(c, seed + (uint64_t)e10 * 100, 1, (double)e10 * 10.0);
      off_rr++; off_ended += o.ww;
    }
    printf("       with FM_PIVOT_LEARN_ENABLED 0 the same pivots: %ld of %ld ended by this net (it does not exist: R2 and the release only)\n",
           off_ended, off_rr);
    HARD(off_ended == 0, "4c(c) with the switch at 0 nothing in this unit ends a run");
  }
  // ---- 4d. report only ----
  {
    long nt_slow = 0, piv_slow = 0, nt_13 = 0, piv_13 = 0, nt_tau = 0, ww_tau = 0, piv_tau = 0, nt_stuck = 0, piv_stuck = 0;
    for (uint64_t seed = 1; seed <= 10; seed++) for (int tg : targets) {
      SimCfg c1 = { 50, 5.0, 0.3, tg, 15.0, 0, false, 1, 1, false, false, true, &kCfg };
      RunOut o = simRun(c1, seed * 31 + tg, 10, -1000.0); nt_slow += o.nt; piv_slow += o.pivots;
      SimCfg c2 = { 13, 12.0, 0.3, tg, 15.0, 0, false, 1, 1, false, false, true, &kCfg };
      o = simRun(c2, seed * 37 + tg, 10, -1000.0); nt_13 += o.nt; piv_13 += o.pivots;
      SimCfg c3 = { 50, 12.0, 0.6, tg, 15.0, 0, false, 1, 1, false, false, true, &kCfg };
      o = simRun(c3, seed * 41 + tg, 10, -1000.0); nt_tau += o.nt; ww_tau += o.ww; piv_tau += o.pivots;
      SimCfg c4 = { 50, 1.0, 0.3, tg, 15.0, 0, false, 1, 1, false, false, true, &kCfg };
      o = simRun(c4, seed * 43 + tg, 3, -1000.0); nt_stuck += o.nt; piv_stuck += o.pivots;
    }
    printf("4d. report only: a 5 deg/s buggy at 50 - NOT TURNING in %ld of %ld pivots; the factory fm_align_cap 13 (12 deg/s at full lock,\n"
           "    5 counts past the VESC deadband) - %ld of %ld; momentum tau 0.6 s - NOT TURNING %ld, WRONG WAY %ld of %ld; the positive\n"
           "    control, a stuck buggy (1 deg/s, weed) - NOT TURNING in %ld of %ld pivots\n",
           nt_slow, piv_slow, nt_13, piv_13, nt_tau, ww_tau, piv_tau, nt_stuck, piv_stuck);
    HARD(nt_stuck > 0, "4d positive control: NOT TURNING fires on a stuck buggy");
  }
}

// ======================================================================================
// 5. THE SHADOW REPLAY over a reduced level-6 ride log (argument mode)
// ======================================================================================
struct Row { uint32_t t; int st, aligning, boost, cmd, eff, thr, cap, gate, tko, slow, fast8, cog, cog_ok, band_ok, imu_st, imu_age, yaw_cdeg, ac; };
static int stretch(int b) { if (b <= 0) return 0; int v = (b * 255 + 120) / 240; return v > 255 ? 255 : v; }

static int replay(const char *path)
{
  FILE *f = fopen(path, "r");
  if (!f) { printf("replay: cannot open %s\n", path); return 2; }
  std::vector<Row> rows; char line[512];
  while (fgets(line, sizeof(line), f)) {
    if (line[0] == '#' || line[0] == 't') continue;
    Row r;
    if (sscanf(line, "%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", &r.t, &r.st, &r.aligning, &r.boost, &r.cmd, &r.eff,
               &r.thr, &r.cap, &r.gate, &r.tko, &r.slow, &r.fast8, &r.cog, &r.cog_ok, &r.band_ok, &r.imu_st, &r.imu_age, &r.yaw_cdeg, &r.ac) == 19)
      rows.push_back(r);
  }
  fclose(f);
  // The log is 3 Hz: the RX's 250 ms tick-gap rule would end every pivot between two rows, so the replay allows 600 ms.
  FmPivotCfg rc = kCfg; rc.tick_gap_ms = 600;
  FmPvRate rate; fmPvRateReset(&rate); FmPvCal cal; fmPvCalReset(&cal);
  FmPvLearn La, Lm; fmPvLearnReset(&La); fmPvLearnReset(&Lm);
  FmPvWw Wa = { 0, 0, 0, 0 }, Wm = { 0, 0, 0, 0 };
  bool ever = false, prev_active = false, pivot_prev = false, man_prev = false;
  uint32_t edge_ms = 0, last_stamp = 0;
  long n_active = 0, n_env = 0, n_judged = 0, n_samples = 0, n_rate = 0, nt_a = 0, ww_a = 0;
  long n_man = 0, n_man_env = 0, n_man_judged = 0, nt_m = 0, ww_m = 0, cal_changes = 0; int8_t last_sign = 0;
  long rows_cal = 0;
  for (size_t i = 0; i < rows.size(); i++) {
    const Row &r = rows[i];
    const uint32_t t = r.t + 100000u;   // keep every stamp positive
    // the IMU's newest sample as the row shows it: status 1 = a plausible sample, 3 = an implausible one; 2 / none = no new one
    bool newer = false;
    if (r.imu_st == 1 || r.imu_st == 3) {
      ever = true;
      uint32_t stamp = t - (uint32_t)(r.imu_age > 0 ? r.imu_age : 0);
      if (last_stamp != 0 && (uint32_t)(stamp > last_stamp ? stamp - last_stamp : last_stamp - stamp) < 60u) stamp = last_stamp;   // the same sample, seen again
      newer = fmPvIngest(&rate, ever, stamp, (float)r.yaw_cdeg / 100.0f, r.imu_st == 1, &rc);
      if (newer) { last_stamp = stamp; n_samples++; if (rate.rate_ok) n_rate++; }
    }
    if (newer) {
      fmPvCalStep(&cal, &rate, r.cog_ok && r.fast8, (float)r.cog / 10.0f, &rc);
      if (cal.sign != last_sign) { cal_changes++; last_sign = cal.sign; }
    }
    if (cal.sign != 0) rows_cal++;
    const bool capable = fmPvCapable(&rate, t, &rc);
    const bool active = (r.st == 2);
    if (active && !prev_active) edge_ms = t;
    prev_active = active;
    if (active) n_active++;
    const int trig = stretch(r.thr);
    // ---- the Follow-Me net, as the RX runs it ----
    const bool judged = active && !r.tko && (uint32_t)(t - edge_ms) >= 500u && pivot_prev;
    if (judged) n_judged++;
    if (fmPvWwStep(&Wa, t, judged, (int16_t)(r.cmd - 127), cal.sign, capable, newer, &rate, &rc)) {
      ww_a++; printf("   FOLLOW-ME ROW %zu: WRONG WAY would have fired (cmd %d, sign %d)\n", i, r.cmd - 127, cal.sign);
    }
    const bool pivot_cmd = active && r.aligning && r.boost && r.slow;
    const bool env = pivot_cmd && r.band_ok;
    if (env) n_env++;
    const int applied = (!r.gate) ? 0 : ((active && r.cap < trig) ? r.cap : trig);
    const FmPvOut po = fmPvLearnStep(&La, t, env, (uint8_t)(r.ac > 0 ? r.ac : 50), (uint8_t)trig, (uint8_t)applied,
                                     (int16_t)(r.cmd - 127), capable, cal.sign, newer, &rate, &rc);
    if (po.nt_fired) { nt_a++; printf("   FOLLOW-ME ROW %zu: NOT TURNING would have fired\n", i); }
    pivot_prev = pivot_cmd;
    // ---- report only: the same two checks on MANUAL low-speed pivots, the stick as the command ----
    const bool manual = (r.st != 2) && (r.st != 5);
    if (manual) n_man++;
    const bool man_pivot = manual && r.slow && (r.eff - 127 >= 32 || r.eff - 127 <= -32) && r.gate;
    if (man_pivot) n_man_env++;
    const bool man_judged = man_pivot && man_prev;
    if (man_judged) n_man_judged++;
    if (fmPvWwStep(&Wm, t, man_judged, (int16_t)(r.eff - 127), cal.sign, capable, newer, &rate, &rc)) ww_m++;
    const FmPvOut pm = fmPvLearnStep(&Lm, t, man_pivot, 50, (uint8_t)trig, (uint8_t)(r.gate ? trig : 0), (int16_t)(r.eff - 127),
                                     capable, cal.sign, newer, &rate, &rc);
    if (pm.nt_fired) nt_m++;
    man_prev = man_pivot;
  }
  uint8_t p = 0, m = 0; fmPvCalCounts(&cal, &p, &m);
  printf("5. SHADOW REPLAY of %s: %zu rows; IMU samples %ld (believed rates %ld)\n", path, rows.size(), n_samples, n_rate);
  printf("   yaw-sign calibration: %lu turns voted, the last %u: %u with the course, %u against -> sign %+d (calibrated on %ld rows, %ld changes)\n",
         (unsigned long)cal.turns, (unsigned)cal.n, (unsigned)p, (unsigned)m, (int)cal.sign, rows_cal, cal_changes);
  printf("   FOLLOW-ME: %ld ACTIVE rows; in the learning envelope %ld; WRONG WAY judged %ld; NOT TURNING %ld, WRONG WAY %ld (D-CD1: 0 / 0)\n",
         n_active, n_env, n_judged, nt_a, ww_a);
  printf("   report only, MANUAL low-speed pivots (stick >= 32 counts, under 4 mph): %ld manual rows, %ld pivot rows, %ld judged;\n"
         "   NOT TURNING %ld, WRONG WAY %ld\n", n_man, n_man_env, n_man_judged, nt_m, ww_m);
  return (nt_a == 0 && ww_a == 0) ? 0 : 1;
}

int main(int argc, char **argv)
{
  const char *replay_path = nullptr;
  bool units_only = false;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--base") && i + 1 < argc) g_base = argv[++i];
    else if (!strcmp(argv[i], "--replay") && i + 1 < argc) replay_path = argv[++i];
    else if (!strcmp(argv[i], "--units")) units_only = true;   // sections 1-3 only (no simulation gates)
  }
  if (replay_path) return replay(replay_path);
  testRate();
  testBounds();
  testLearn();
  testNotTurning();
  testCalibration();
  testWrongWay();
  testRxSource();
  testKillSwitch();
  if (!units_only) testSimulation();
  if (g_hard_failed) {
    printf("follow_me_pivot_test: %d HARD GATE(S) FAILED (see the >>> lines); %ld checks and gates passed\n", g_hard_failed, g_checks);
    return 1;
  }
  printf("follow_me_pivot_test: all %ld checks passed\n", g_checks);
  return 0;
}
