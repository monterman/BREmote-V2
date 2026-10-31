// V2.5-Evo - 2026-10-31 - R2-S SHIPS AS S1 ALONE (the R2-S ruling, Revision 1.1): section 2 also checks that the RX calls the
//   rule with S2 OFF, and gate 3 is asserted for S1 alone (the RX's call). S2's code stays in the header, host-tested here.
// V2.5-Evo - 2026-10-31 - the R2-S ruling: both variants (kCfg as first built, kCfgW = S2-w), S2-w's unit checks and red edits
//   (section 1b, with the healthy "target sweeps behind" track), and gate 3 over both committed fixtures (30 Oct and 19 Oct).
// V2.5-Evo - 2026-10-31 - Host test, Follow-Me package 2 unit R2-S (design audit 2026-10-31 §E-0): THE WRONG-WAY NET,
//   Common/FollowMePause.h fmSteerResponseStep() - S1 (the command held on one side while the COG turns the other way) and
//   S2 (pointing away from the target and not coming round).
//   1. Unit checks on made-up tracks: when each rule fires and when it must not (the other way round, a command that
//      changes side, under the command threshold, COG or throttle valid on too few samples, 14 deg against 15 deg, the
//      north wrap, a judged gap, S2 switched off, an error that dips under 150 deg), at 10 Hz and at 3 Hz (time-based).
//   2. The RX's numbers: RTMState.ino must hold exactly kCfg below (source-text check, run from Tools/tests).
//   3. Gate 3, the LOG REPLAY (D-R1): the rule over every row of fixtures/r2s_replay_2026-10-30.csv, derived from RX1's
//      30 Oct deep log with NO positions, NO clock time and NO speed (courses, steering bytes, trigger / cap bytes, the
//      heading error, and derived 0/1 flags). FM_ACTIVE rows with the RX's exclusions: zero events (HARD). Manual rows
//      (shadow mode: the stick as the command, S2 off): events reported on and off the rope, per hour of riding.
//      `follow_me_steer_response_test <fixture.csv>` replays another fixture made by the same extractor (report only).
//   The closed-loop gates (red edit (c), the healthy sets) are section 10 of follow_me_converge_test.cpp.
// Build and run (Git Bash, from Tools/tests, PATH and TMP as in the host-test note of the compile procedure):
//   g++ -std=c++17 -O2 -Wall -Wextra -o follow_me_steer_response_test follow_me_steer_response_test.cpp && ./follow_me_steer_response_test
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

#include "../../Source/Common/FollowMePause.h"

static long g_checks = 0;
#define CHECK(x) do { assert(x); g_checks++; } while (0)

// The RX's numbers (RTMState.ino kFmSteerRespCfg), stated here; section 2 checks the RX holds the same initializer.
// V2.5-Evo - 2026-10-31 - the two variants of the R2-S ruling: kCfg = R2-S as first built (7928269: S2 on every sample of
// 4 s, against the side of the span's first sample); kCfgW = S2-w (80 % of a 3 s span at 150 deg or more, the rotation
// toward the target summed tick by tick). kShip is the one RTMState.ino ships (the ruling's accept rule), and kCfgText the
// exact line it must hold.
static const FmSteerRespCfg kCfg  = { 32, 500, 2000, 15.0f, 0.8f, 150.0f, 4000, 15.0f, 1.0f, false };
static const FmSteerRespCfg kCfgW = { 32, 500, 2000, 15.0f, 0.8f, 150.0f, 3000, 15.0f, 0.8f, true };
static const FmSteerRespCfg *kShip = &kCfg;   // the RX's numbers (its S2 fields are unused: the RX passes s2_on false)
static const char *kCfgText = "static const FmSteerRespCfg kFmSteerRespCfg = { 32, 500, 2000, 15.0f, 0.8f, 150.0f, 4000, 15.0f, 1.0f, false };";

// One scripted run: dt_ms per sample, n samples; per sample the command offset, the COG and the flags come from the
// lambdas. Returns the time (ms from the start) of the first verdict, or -1, and that verdict.
template <class Cmd, class Cog, class OkC, class OkT, class Err>
static long runTrack(uint32_t dt_ms, int n, Cmd cmd, Cog cog, OkC cog_ok, OkT thr_ok, Err err, bool s2_on, uint8_t *verdict,
                     const FmSteerRespCfg *cfg = &kCfg)
{
  FmSteerResp r; fmSteerRespReset(&r);
  const uint32_t t0 = 4000000000u;   // near the 32-bit wrap: every age is computed modulo 2^32
  for (int i = 0; i < n; i++) {
    const uint32_t t = t0 + (uint32_t)i * dt_ms;
    float e = 0.0f; const bool e_ok = err(i, &e);
    const uint8_t v = fmSteerResponseStep(&r, t, true, (int16_t)cmd(i), cog_ok(i), (float)cog(i), thr_ok(i), e_ok, e, cfg, s2_on);
    if (v != FMSR_NONE) { *verdict = v; return (long)((uint32_t)(t - t0)); }
  }
  *verdict = FMSR_NONE;
  return -1;
}
static bool always(int) { return true; }
static bool noErr(int, float *e) { *e = 0.0f; return false; }

static void testUnits()
{
  uint8_t v = 0;
  // S1 fires: the command held right (+60) while the COG turns LEFT at 10 deg/s - the mirrored buggy.
  long t = runTrack(100, 60, [](int) { return 60; }, [](int i) { return 90.0 - 1.0 * i; }, always, always, noErr, true, &v);
  CHECK(v == FMSR_WRONG_WAY && t == 2500);              // the first tick whose history spans lead-in + window (2.5 s)
  // ... and to the left, with the COG turning right.
  t = runTrack(100, 60, [](int) { return -60; }, [](int i) { return 90.0 + 1.0 * i; }, always, always, noErr, true, &v);
  CHECK(v == FMSR_WRONG_WAY && t == 2500);
  // A healthy buggy turning the way it is told never fires, however long the command is held.
  t = runTrack(100, 600, [](int) { return 60; }, [](int i) { return 90.0 + 1.0 * i; }, always, always, noErr, true, &v);
  CHECK(t < 0);
  // Under the threshold (31 counts) - not "steering to one side".
  t = runTrack(100, 100, [](int) { return 31; }, [](int i) { return 90.0 - 1.0 * i; }, always, always, noErr, false, &v);
  CHECK(t < 0);
  // Exactly 32 counts is.
  t = runTrack(100, 100, [](int) { return 32; }, [](int i) { return 90.0 - 1.0 * i; }, always, always, noErr, false, &v);
  CHECK(v == FMSR_WRONG_WAY && t == 2500);
  // The command changes side once every 2 s - never one side for 2.5 s.
  t = runTrack(100, 300, [](int i) { return ((i / 20) % 2) ? -60 : 60; }, [](int i) { return 90.0 - 1.0 * i; }, always, always,
               noErr, false, &v);
  CHECK(t < 0);
  // 14 deg the wrong way over the 2 s window: not enough; 15 deg is.
  t = runTrack(100, 60, [](int) { return 60; }, [](int i) { return 90.0 - 0.7 * i; }, always, always, noErr, false, &v);
  CHECK(t < 0);
  t = runTrack(100, 60, [](int) { return 60; }, [](int i) { return 90.0 - 0.75 * i; }, always, always, noErr, false, &v);
  CHECK(v == FMSR_WRONG_WAY);
  // COG valid on fewer than 80 % of the window's samples (every 4th missing = 75 %): not judged.
  t = runTrack(100, 120, [](int) { return 60; }, [](int i) { return 90.0 - 1.0 * i; }, [](int i) { return (i % 4) != 0; },
               always, noErr, false, &v);
  CHECK(t < 0);
  // ... every 5th missing (80 %): judged.
  t = runTrack(100, 120, [](int) { return 60; }, [](int i) { return 90.0 - 1.0 * i; }, [](int i) { return (i % 5) != 0; },
               always, noErr, false, &v);
  CHECK(v == FMSR_WRONG_WAY);
  // The throttle under the limit on too many samples (a buggy without thrust cannot be judged on its turning).
  t = runTrack(100, 120, [](int) { return 60; }, [](int i) { return 90.0 - 1.0 * i; }, always, [](int i) { return (i % 3) != 0; },
               noErr, false, &v);
  CHECK(t < 0);
  // The north wrap: 10 deg falling through 0 to 340 deg is a 30 deg LEFT turn, not +330.
  t = runTrack(100, 60, [](int) { return 60; }, [](int i) { double c = 10.0 - 1.0 * i; return (c < 0.0) ? c + 360.0 : c; },
               always, always, noErr, false, &v);
  CHECK(v == FMSR_WRONG_WAY && t == 2500);
  // ... and a right turn through north is healthy.
  t = runTrack(100, 600, [](int) { return 60; }, [](int i) { return fmod(350.0 + 1.0 * i, 360.0); }, always, always, noErr,
               false, &v);
  CHECK(t < 0);
  // TIME-BASED: the same mirrored track sampled at 3 Hz fires once 2.5 s are covered (the sample at 2.667 s).
  t = runTrack(333, 20, [](int) { return 60; }, [](int i) { return 90.0 - 3.33 * i; }, always, always, noErr, false, &v);
  CHECK(v == FMSR_WRONG_WAY && t == 2664);
  // A judged gap clears the history: after one unjudged sample the 2.5 s start again.
  {
    FmSteerResp r; fmSteerRespReset(&r);
    uint8_t out = FMSR_NONE; long first = -1;
    for (int i = 0; i < 60; i++) {
      const bool judged = (i != 20);
      out = fmSteerResponseStep(&r, 1000u + 100u * i, judged, 60, true, (float)(90.0 - i), true, false, 0.0f, &kCfg, false);
      if (out != FMSR_NONE) { first = i; break; }
    }
    CHECK(out == FMSR_WRONG_WAY && first == 46);           // 25 samples after the gap at 20 (21 .. 46)
  }
  // S2: the steady mirrored state - the error near 180 deg, the command chattering, no net rotation.
  auto err180 = [](int i, float *e) { *e = (i % 2) ? 175.0f : -178.0f; return true; };
  t = runTrack(100, 80, [](int i) { return (i % 2) ? 90 : -90; }, [](int i) { return 200.0 + ((i % 2) ? 2.0 : -2.0); }, always,
               always, err180, true, &v);
  CHECK(v == FMSR_POINTING_AWAY && t == 4000);
  // ... switched off (S2 removed): nothing (S1 cannot see it - the command changes side).
  t = runTrack(100, 80, [](int i) { return (i % 2) ? 90 : -90; }, [](int i) { return 200.0 + ((i % 2) ? 2.0 : -2.0); }, always,
               always, err180, false, &v);
  CHECK(t < 0);
  // S2 does not fire on a buggy that IS coming round toward the target (16 deg in 4 s toward the error's side).
  auto err170 = [](int, float *e) { *e = 170.0f; return true; };
  t = runTrack(100, 80, [](int) { return 0; }, [](int i) { return 10.0 + 0.4 * i; }, always, always, err170, true, &v);
  CHECK(t < 0);
  // ... but 14 deg in 4 s toward it is "not coming round".
  t = runTrack(100, 80, [](int) { return 0; }, [](int i) { return 10.0 + 0.35 * i; }, always, always, err170, true, &v);
  CHECK(v == FMSR_POINTING_AWAY && t == 4000);
  // An error that dips under 150 deg once inside the 4 s: not "pointing away for 4 s".
  auto errDip = [](int i, float *e) { *e = (i == 30) ? 140.0f : 170.0f; return true; };
  t = runTrack(100, 80, [](int) { return 0; }, [](int) { return 10.0; }, always, always, errDip, true, &v);
  CHECK(v == FMSR_POINTING_AWAY && t == 7100);           // the first 4 s span without the dip: samples 31-71
  // No heading error at all (err_ok false): S2 is never judged.
  t = runTrack(100, 100, [](int) { return 0; }, [](int) { return 10.0; }, always, always, noErr, true, &v);
  CHECK(t < 0);
  // The red edit of gate 2, at unit level: with the opposite-rotation test removed (rot_min -1000) S1 fires on any
  // constant command - a healthy buggy turning the right way included.
  {
    FmSteerRespCfg red = kCfg; red.rot_min_deg = -1000.0f;
    t = runTrack(100, 60, [](int) { return 60; }, [](int i) { return 90.0 + 1.0 * i; }, always, always, noErr, false, &v, &red);
    CHECK(v == FMSR_WRONG_WAY && t == 2500);
  }
  printf("1. unit checks: %ld passed\n", g_checks);
}

// V2.5-Evo - 2026-10-31 - S2-w, the ruling's change, and its red edits at unit level.
static void testS2w()
{
  uint8_t v = 0; long t;
  // The steady mirrored state (the error chattering across the wrap, the command too): S2-w fires once 3 s are covered.
  auto err180 = [](int i, float *e) { *e = (i % 2) ? 175.0f : -178.0f; return true; };
  t = runTrack(100, 80, [](int i) { return (i % 2) ? 90 : -90; }, [](int i) { return 200.0 + ((i % 2) ? 2.0 : -2.0); }, always,
               always, err180, true, &v, &kCfgW);
  CHECK(v == FMSR_POINTING_AWAY && t == 3000);
  // PART 1: one dip under 150 deg inside the span no longer restarts it (80 %).
  auto errDip = [](int i, float *e) { *e = (i == 20) ? 140.0f : 170.0f; return true; };
  t = runTrack(100, 80, [](int) { return 0; }, [](int) { return 10.0; }, always, always, errDip, true, &v, &kCfgW);
  CHECK(v == FMSR_POINTING_AWAY && t == 3000);
  {   // RED, part 1 reverted (every sample): the dip holds it off until it leaves the span (samples 21-51)
    FmSteerRespCfg c = kCfgW; c.s2_away_frac = 1.0f;
    t = runTrack(100, 80, [](int) { return 0; }, [](int) { return 10.0; }, always, always, errDip, true, &v, &c);
    CHECK(v == FMSR_POINTING_AWAY && t == 5100);
  }
  // Coming round still clears it: 16.5 deg toward the target in 3 s; 13.5 deg is "not coming round".
  auto err170 = [](int, float *e) { *e = 170.0f; return true; };
  t = runTrack(100, 80, [](int) { return 0; }, [](int i) { return 10.0 + 0.55 * i; }, always, always, err170, true, &v, &kCfgW);
  CHECK(t < 0);
  t = runTrack(100, 80, [](int) { return 0; }, [](int i) { return 10.0 + 0.45 * i; }, always, always, err170, true, &v, &kCfgW);
  CHECK(v == FMSR_POINTING_AWAY && t == 3000);
  // PART 2 - THE TARGET SWEEPS BEHIND (the ruling's red-edit track, healthy): the target behind the buggy across the wrap
  // (-179 deg on the first sample, +179 .. +175 after), the buggy turning toward it at 15 deg/s with the command on that side.
  auto sweep = [](int i, float *e) { *e = (i == 0) ? -179.0f : 179.0f - 0.05f * i; return true; };
  auto sweepCmd = [](int i) { return (i == 0) ? -127 : 127; };
  auto sweepCog = [](int i) { return 30.0 + 1.5 * i; };
  t = runTrack(100, 80, sweepCmd, sweepCog, always, always, sweep, true, &v, &kCfgW);
  CHECK(t < 0);                                          // S2-w: turning toward the target tick by tick - no end
  {   // RED, part 2 reverted (the start-side rule): the healthy sweep fires falsely as soon as 3 s are covered
    FmSteerRespCfg c = kCfgW; c.s2_tickwise = false;
    t = runTrack(100, 80, sweepCmd, sweepCog, always, always, sweep, true, &v, &c);
    CHECK(v == FMSR_POINTING_AWAY && t == 3000);
  }
  // FINDING: R2-S AS FIRST BUILT (kCfg) false-fires on the same healthy sweep (its start-side rule, at 4 s). The healthy
  // gate-2 sets never produce this geometry, which is why gate 2 passed it; recorded here, not hidden.
  t = runTrack(100, 80, sweepCmd, sweepCog, always, always, sweep, true, &v, &kCfg);
  CHECK(v == FMSR_POINTING_AWAY && t == 4000);
  printf("1b. S2-w unit checks (chatter, the 80 %% dip, coming round, the target sweeping behind; red edits parts 1 and 2): "
         "passed - and R2-S as first built false-fires on the sweep track (recorded)\n");
}

// 2. The RX holds the same numbers.
static void testRxNumbers()
{
  FILE *f = fopen("../../Source/V2_Integration_Rx/RTMState.ino", "rb");
  CHECK(f != nullptr);
  std::string text; char buf[65536]; size_t k;
  while ((k = fread(buf, 1, sizeof(buf), f)) > 0) text.append(buf, k);
  fclose(f);
  CHECK(text.find(kCfgText) != std::string::npos);
  // S1 ALONE: the judge's call passes s2_on false - the call from "sr_verdict = fmSteerResponseStep(" to its ");".
  const size_t c0 = text.find("const uint8_t sr_verdict = fmSteerResponseStep(&fm_sr_active,");
  CHECK(c0 != std::string::npos);
  const size_t c1 = text.find(");", c0);
  CHECK(c1 != std::string::npos);
  const std::string call = text.substr(c0, c1 - c0);
  CHECK(call.find("&kFmSteerRespCfg, false") != std::string::npos);
  CHECK(call.find("&kFmSteerRespCfg, true") == std::string::npos);
  CHECK(text.find("fmSteerResponseStep(&fm_sr_active", c1) == std::string::npos);   // one judge call only
  printf("2. RTMState.ino holds kFmSteerRespCfg and calls the judge with S2 OFF (S1 alone ships)\n");
}

// ======================================================================================
// 3. THE LOG REPLAY (D-R1)
// ======================================================================================
struct Row { uint32_t t; int st, cmd, eff, thr, cap, gate, cog, cog_ok, herr, tko, yld, rope; };
static const int kStActive = 2, kStReturn = 5;           // RTMState.ino FmState
static const uint32_t kTakeoverAfterMs = 1000, kEdgeMs = 500;

// The applied throttle (calcPWM()'s g_applied_thr_cmd, post-cap, pre-mixer) as the log lets us rebuild it: the trigger byte
// stretched 0..240 -> 0..255 (thrByteToCmd()), held to Follow-Me's cap while ACTIVE, 0 with the motor gate shut. The ramp is
// ignored (it only lowers the value, and only for 1.5 s after an engagement).
static int appliedThr(const Row &r)
{
  if (!r.gate) return 0;
  int a = (r.thr * 255 + 120) / 240;
  if (r.st == kStActive && r.cap < a) a = r.cap;
  return a;
}

struct ReplayOut {
  long rows, active_rows, active_judged, active_events, active_s1, active_s2;
  long manual_rows, manual_judged_rope[3], shadow_events_rope[3];
  double manual_riding_s_rope[3];   // manual time with thrust and a usable COG, per rope class
};

static bool loadFixture(const char *path, std::vector<Row> *rows)
{
  FILE *f = fopen(path, "r");
  if (!f) return false;
  char line[512];
  while (fgets(line, sizeof(line), f)) {
    if (line[0] == '#' || line[0] == 't') continue;
    Row r;
    if (sscanf(line, "%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", &r.t, &r.st, &r.cmd, &r.eff, &r.thr, &r.cap, &r.gate, &r.cog,
               &r.cog_ok, &r.herr, &r.tko, &r.yld, &r.rope) == 13) rows->push_back(r);
  }
  fclose(f);
  return true;
}

static ReplayOut replay(const std::vector<Row> &rows, const FmSteerRespCfg *cfg, bool print, bool s2_on = false)
{
  ReplayOut o; memset(&o, 0, sizeof(o));
  FmSteerResp act, sh; fmSteerRespReset(&act); fmSteerRespReset(&sh);
  uint32_t edge_ms = 0, tko_end_ms = 0; bool prev_active = false, prev_tko = false, tko_seen = false;
  for (size_t i = 0; i < rows.size(); i++) {
    const Row &r = rows[i];
    o.rows++;
    const bool active = (r.st == kStActive);
    if (active && !prev_active) edge_ms = r.t;
    if (prev_tko && !r.tko) { tko_end_ms = r.t; tko_seen = true; }
    prev_active = active; prev_tko = (r.tko != 0);
    const bool cog_ok = r.cog_ok != 0;
    const bool thr_ok = appliedThr(r) >= 30;
    const bool err_ok = r.herr != 32767;
    // ---- the real net: FM_ACTIVE, not a takeover or 1 s after one, not the RTM yield, not the first 0.5 s ----
    if (active) o.active_rows++;
    const bool after_tko = tko_seen && (uint32_t)(r.t - tko_end_ms) < kTakeoverAfterMs;
    const bool judged = active && !r.tko && !after_tko && !r.yld && (uint32_t)(r.t - edge_ms) >= kEdgeMs;
    if (judged) o.active_judged++;
    const uint8_t v = fmSteerResponseStep(&act, r.t, judged, (int16_t)(r.cmd - 127), cog_ok, r.cog / 10.0f, thr_ok, err_ok,
                                          r.herr / 10.0f, cfg, s2_on);
    if (v != FMSR_NONE) {
      o.active_events++; if (v == FMSR_WRONG_WAY) o.active_s1++; else o.active_s2++;
      if (print) printf("   ACTIVE EVENT at t %.1f s: %s (cmd %d, cog %.1f, herr %.1f)\n", r.t / 1000.0, v == FMSR_WRONG_WAY ? "S1" : "S2",
             r.cmd, r.cog / 10.0, r.herr / 10.0);
      fmSteerRespReset(&act);            // the RX ends Follow-Me here: the next judged span starts fresh
    }
    // ---- shadow mode: manual ticks (not Follow-Me ACTIVE / RETURN, not RTM), the stick as the command, S2 off ----
    const bool manual = !active && r.st != kStReturn && !r.yld;
    if (manual) o.manual_rows++;
    const int rc = (r.rope >= 0 && r.rope <= 2) ? r.rope : 2;
    if (manual) {
      o.manual_judged_rope[rc]++;
      if (thr_ok && cog_ok && i > 0) o.manual_riding_s_rope[rc] += (double)(uint32_t)(r.t - rows[i - 1].t) / 1000.0;
    }
    const uint8_t sv = fmSteerResponseStep(&sh, r.t, manual, (int16_t)(r.eff - 127), cog_ok, r.cog / 10.0f, thr_ok, false, 0.0f,
                                           cfg, false);
    if (sv != FMSR_NONE) {
      o.shadow_events_rope[rc]++;
      if (print) printf("   shadow event (manual, %s) at t %.1f s: stick %d, cog %.1f\n", rc == 1 ? "on the rope" : rc == 0 ? "off the rope" :
             "rope unknown", r.t / 1000.0, r.eff, r.cog / 10.0);
      fmSteerRespReset(&sh);             // one event per episode: report-only, nothing else happens
    }
  }
  return o;
}

static void printReplay(const char *label, const ReplayOut &o)
{
  printf("   %s: %ld rows; FM_ACTIVE %ld rows (%ld judged): %ld events (S1 %ld, S2 %ld)\n", label, o.rows, o.active_rows,
         o.active_judged, o.active_events, o.active_s1, o.active_s2);
  const char *rn[3] = { "off the rope", "on the rope ", "rope unknown" };
  for (int k = 0; k < 3; k++) {
    const double h = o.manual_riding_s_rope[k] / 3600.0;
    char rate[32];
    if (h > 0.0) snprintf(rate, sizeof(rate), "%.1f per 10 h", o.shadow_events_rope[k] / h * 10.0);
    else         snprintf(rate, sizeof(rate), "-");
    printf("   shadow, manual %s: %ld rows, %.2f h riding (thrust + COG); %ld events (%s)\n", rn[k], o.manual_judged_rope[k], h,
           o.shadow_events_rope[k], rate);
  }
}

int main(int argc, char **argv)
{
  testUnits();
  testS2w();
  testRxNumbers();
  // GATE 3 (HARD): zero events in the FM_ACTIVE rows of both committed fixtures, for both variants. An argument replays one
  // other fixture made by the same extractor (report only).
  struct Fix { const char *path; size_t rows; long active; };
  const Fix fixes[2] = { { "fixtures/r2s_replay_2026-10-30.csv", 3933, 180 }, { "fixtures/r2s_replay_2026-10-19.csv", 5990, 618 } };
  const int nfix = (argc > 1) ? 1 : 2;
  for (int fx = 0; fx < nfix; fx++) {
    const char *path = (argc > 1) ? argv[1] : fixes[fx].path;
    std::vector<Row> rows;
    CHECK(loadFixture(path, &rows));
    printf("3. LOG REPLAY (D-R1) of %s\n", path);
    const ReplayOut o  = replay(rows, kShip, true, false);         // S1 alone: what the RX runs
    printReplay("S1 alone (ships)", o);
    const ReplayOut ob = replay(rows, &kCfg, false, true);         // the run record: S2 as first built, and S2-w
    const ReplayOut ow = replay(rows, &kCfgW, false, true);
    printf("   record - with S2 as first built: %ld events; with S2-w: %ld events\n", ob.active_events, ow.active_events);
    if (argc <= 1) {
      CHECK(rows.size() == fixes[fx].rows);
      CHECK(o.active_rows == fixes[fx].active);
      CHECK(o.active_events == 0);                                 // GATE 3 (HARD), S1 alone
    }
    FmSteerRespCfg red = *kShip; red.rot_min_deg = -1000.0f;   // the opposite-rotation test removed (report)
    const ReplayOut r = replay(rows, &red, false);
    printf("   red edit, opposite-rotation test removed: FM_ACTIVE events %ld; shadow events off / on / unknown rope %ld / %ld / %ld\n",
           r.active_events, r.shadow_events_rope[0], r.shadow_events_rope[1], r.shadow_events_rope[2]);
  }
  printf("follow_me_steer_response_test: all %ld checks passed\n", g_checks);
  return 0;
}
