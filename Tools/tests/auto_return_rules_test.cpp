// Host test for Common/AutoReturnRules.h - the auto-return / return-to-me end rules (2026-10-07).
// Build and run:  g++ -std=c++17 -Wall -Wextra -o auto_return_rules_test auto_return_rules_test.cpp && ./auto_return_rules_test
// Ticks are 100 ms like runRtmLoop() / runFmLoop(). Constants are the RX's: release below 8 counts,
// H-2 limit 1500 ms (step bound 200 ms), D-1 stale 10000 ms / recover 1000 ms, S-6 15000 / 2000 / 2 deg / 45000.

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../../Source/Common/AutoReturnRules.h"

static void testHandbackCap()
{
  bool clr = true;
  // No cap standing: the throttle passes untouched and nothing is cleared.
  assert(handbackCapStep(kHandbackNone, 200, 200, 8, &clr) == 200 && !clr);
  // A cap of 0 (arrival at the stop distance) with the trigger still at 100 %: the motor stays at 0.
  assert(handbackCapStep(0, 255, 255, 8, &clr) == 0 && !clr);
  // A cap of 40 with the rider at 30: 30 (subtract-only, never raised to the cap).
  assert(handbackCapStep(40, 30, 30, 8, &clr) == 30 && !clr);
  // Easing to 9 counts is NOT a full release: still capped, not cleared.
  assert(handbackCapStep(0, 9, 9, 8, &clr) == 0 && !clr);
  // Fully released (below 8): cleared; this tick's output is still min(effective, cap).
  assert(handbackCapStep(0, 3, 3, 8, &clr) == 0 && clr);
  assert(handbackCapStep(5, 0, 0, 8, &clr) == 0 && clr);
  // Never above the input, for every cap / throttle pair.
  for (int cap = 0; cap <= 255; ++cap)
    for (int thr = 0; thr <= 255; ++thr) {
      const uint8_t out = handbackCapStep((uint8_t)cap, (uint8_t)thr, (uint8_t)thr, 8, &clr);
      assert(out <= thr);
      if (cap != 255) assert(out <= cap);
    }
}

static void testRtmGateFault()
{
  RtmGateFaultState s = {0, 0};
  uint32_t t = 1000;
  // Held and failing: fires once 1500 ms have accumulated (first tick adds nothing).
  int fired_at = -1;
  for (int i = 0; i < 30; ++i, t += 100) {
    if (rtmGateFaultStep(&s, t, true, true, 1500, 200)) { fired_at = i; break; }
  }
  assert(fired_at == 15);
  // Feathering: 1 s held-failing, 1 s released, 1 s held-failing -> the release PAUSES, it fires.
  rtmGateFaultReset(&s);
  t = 50000;
  bool fired = false;
  for (int i = 0; i < 10; ++i, t += 100) fired |= rtmGateFaultStep(&s, t, true, true, 1500, 200);
  assert(!fired);
  for (int i = 0; i < 10; ++i, t += 100) fired |= rtmGateFaultStep(&s, t, false, false, 1500, 200);
  assert(!fired);
  for (int i = 0; i < 10; ++i, t += 100) fired |= rtmGateFaultStep(&s, t, true, true, 1500, 200);
  assert(fired);
  // A passing tick under a held trigger resets the count.
  rtmGateFaultReset(&s);
  t = 90000;
  for (int i = 0; i < 12; ++i, t += 100) assert(!rtmGateFaultStep(&s, t, true, true, 1500, 200));
  assert(!rtmGateFaultStep(&s, t, true, false, 1500, 200)); t += 100;
  for (int i = 0; i < 12; ++i, t += 100) assert(!rtmGateFaultStep(&s, t, true, true, 1500, 200));
  // A loop stall cannot jump the count: one 5 s gap adds at most 200 ms.
  rtmGateFaultReset(&s);
  assert(!rtmGateFaultStep(&s, 200000, true, true, 1500, 200));
  assert(!rtmGateFaultStep(&s, 205000, true, true, 1500, 200));
  assert(s.accum_ms == 200);
}

static void testBootId()
{
  uint8_t stored = kTxBootIdNone;
  assert(!txBootIdIsBootIdValue(0x00) && !txBootIdIsBootIdValue(0x01) && !txBootIdIsBootIdValue(0x02));
  assert(txBootIdIsBootIdValue(0x80) && txBootIdIsBootIdValue(0xFF));
  assert(!txBootIdStep(&stored, 0x80 | 42));   // first ID: stored, no reboot
  assert(stored == 42);
  assert(!txBootIdStep(&stored, 0x80 | 42));   // repeat: no reboot
  assert(txBootIdStep(&stored, 0x80 | 7));     // different: the remote rebooted
  assert(stored == 7);
  assert(!txBootIdStep(&stored, 0x80 | 7));
  assert(txBootIdStep(&stored, 0x80 | 0));     // ID 0 is a valid ID
}

static void testRefreshExpiry()
{
  // Today's remote never refreshes: no expiry, however long the run.
  assert(!rtmRefreshExpired(true, false, 0, 600000, 5000));
  // A refreshing remote: fine within 5 s, expired after.
  assert(!rtmRefreshExpired(true, true, 10000, 15000, 5000));
  assert(rtmRefreshExpired(true, true, 10000, 15001, 5000));
  // RTM not active: never.
  assert(!rtmRefreshExpired(false, true, 0, 600000, 5000));
  // N-1: the radio task stamped the refresh AFTER the loop read its clock (last = now + 1, + 50): fresh,
  // never "49 days old". The unsigned difference used to wrap and expire the run at once.
  assert(!rtmRefreshExpired(true, true, 20001, 20000, 5000));
  assert(!rtmRefreshExpired(true, true, 20050, 20000, 5000));
  // Across the 32-bit millis() wrap: still measured correctly both ways.
  assert(!rtmRefreshExpired(true, true, 0xFFFFFF00u, 0x00000100u, 5000));          // 512 ms old
  assert(rtmRefreshExpired(true, true, 0xFFFFFF00u, 0x00000100u + 5000u, 5000));   // 5512 ms old
  assert(!rtmRefreshExpired(true, true, 0x00000010u, 0xFFFFFFF0u, 5000));          // stamped 32 ms "after" now
}

// N-1 / N-6 / N-7: the signed stamp age every loop-task comparison now uses.
static void testStampAge()
{
  assert(stampAgeMs(1000, 400) == 600);
  assert(stampAgeMs(1000, 1000) == 0);
  assert(stampAgeMs(1000, 1001) == 0);              // stamp newer than the clock: "just now", not 49 days
  assert(stampAgeMs(1000, 1500) == 0);
  assert(stampAgeMs(0x00000005u, 0xFFFFFFFBu) == 10); // across the wrap
  // stampStale: 0 = never = stale; strictly older than the limit = stale; newer than now = fresh.
  assert(stampStale(5000, 0, 1000));
  assert(!stampStale(5000, 4000, 1000));             // exactly at the limit: not stale (the old `>` test)
  assert(stampStale(5000, 3999, 1000));
  assert(!stampStale(5000, 5001, 1000));             // the race the audit found: stamp > now
  assert(!stampStale(5000, 5100, 0));
}

// N-6: the failing-conditions mask, from plain inputs.
static void testFailingMask()
{
  const uint32_t now = 100000;
  // Everything fresh and good: 0.
  assert(fmFailingMaskFrom(now, false, true, now - 500, 3000, now - 200, 6000, true, now - 100, 1000) == 0);
  // THE AUDIT CASE: the radio task stamped the rider fix and the link AFTER `now` (stamp = now + 1).
  // They must read fresh - no false TxStale / Link bit.
  assert(fmFailingMaskFrom(now, false, true, now + 1, 3000, now - 200, 6000, true, now + 1, 1000) == 0);
  // A real rider-GPS gap while the link keeps refreshing after `now`: ONLY the rider bit, so a parked
  // return waits (tolerated) even without a boot ID - before the fix the false Link bit made it a fault.
  const uint8_t m = fmFailingMaskFrom(now, false, true, now - 4000, 3000, now - 200, 6000, true, now + 3, 1000);
  assert(m == kFmCondTxStale);
  assert(fmReturnParkedTolerates(m, true, false, false));
  // Never-received stamps fail (0 = never), exactly as the old `stamp == 0 ||` tests.
  assert(fmFailingMaskFrom(now, false, true, 0, 3000, 0, 6000, true, 0, 1000) ==
         (kFmCondTxStale | kFmCondRxStale | kFmCondLink));
  // Each condition maps to its own bit.
  assert(fmFailingMaskFrom(now, true,  true,  now, 3000, now, 6000, true,  now, 1000) == kFmCondPhaseA);
  assert(fmFailingMaskFrom(now, false, false, now, 3000, now, 6000, true,  now, 1000) == kFmCondPhaseB);
  assert(fmFailingMaskFrom(now, false, true,  now, 3000, now - 6001, 6000, true, now, 1000) == kFmCondRxStale);
  assert(fmFailingMaskFrom(now, false, true,  now, 3000, now, 6000, false, now, 1000) == kFmCondHeading);
  assert(fmFailingMaskFrom(now, false, true,  now, 3000, now, 6000, true,  now - 1001, 1000) == kFmCondLink);
}

static void testDistBlank()
{
  DistBlankState s = {0, 0, true};
  uint32_t t = 1000;
  // At boot it is blanked; valid inputs publish after 1 s continuous.
  int published_at = -1;
  for (int i = 0; i < 20; ++i, t += 100) {
    if (!distBlankStep(&s, t, true, 10000, 1000)) { published_at = i; break; }
  }
  assert(published_at == 10);
  // A short hiccup (5 s) keeps the last value: no blanking.
  for (int i = 0; i < 50; ++i, t += 100) assert(!distBlankStep(&s, t, false, 10000, 1000));
  t += 100; assert(!distBlankStep(&s, t, true, 10000, 1000));
  // More than 10 s stale: blanked.
  bool blanked = false;
  for (int i = 0; i < 110; ++i, t += 100) blanked = distBlankStep(&s, t, false, 10000, 1000);
  assert(blanked);
  // One valid tick does not unblank (hysteresis), a flapping input stays blanked.
  for (int i = 0; i < 20; ++i, t += 100) assert(distBlankStep(&s, t, (i % 3) != 0, 10000, 1000));
  // 1 s continuous valid: published again.
  bool pub = false;
  for (int i = 0; i < 11 && !pub; ++i, t += 100) pub = !distBlankStep(&s, t, true, 10000, 1000);
  assert(pub);

  // The RX's setting: stale_ms 0 (its inputs already tolerate a 10 s old rider fix). The first invalid
  // tick blanks; one valid tick does not unblank; 1 s of valid inputs does.
  DistBlankState r = {0, 0, true};
  t = 500000;
  for (int i = 0; i < 11; ++i, t += 100) (void)distBlankStep(&r, t, true, 0, 1000);
  assert(!distBlankStep(&r, t, true, 0, 1000)); t += 100;
  assert(distBlankStep(&r, t, false, 0, 1000)); t += 100;
  assert(distBlankStep(&r, t, true, 0, 1000));  t += 100;
  pub = false;
  for (int i = 0; i < 11 && !pub; ++i, t += 100) pub = !distBlankStep(&r, t, true, 0, 1000);
  assert(pub);
}

static void testParkedTolerance()
{
  // Remote GPS stale, parked, released: wait.
  assert(fmReturnParkedTolerates(kFmCondTxStale, true, false, false));
  // Link down: waits only when the remote sends a boot ID (S-8 pairing).
  assert(!fmReturnParkedTolerates(kFmCondLink, true, false, false));
  assert(fmReturnParkedTolerates(kFmCondLink, true, false, true));
  // Handshake revoked together with remote staleness: wait. Handshake alone (a real mismatch): end.
  assert(fmReturnParkedTolerates(kFmCondPhaseB | kFmCondTxStale, true, false, false));
  assert(!fmReturnParkedTolerates(kFmCondPhaseB, true, false, true));
  // The buggy's own sensors always end it.
  assert(!fmReturnParkedTolerates(kFmCondRxStale, true, false, true));
  assert(!fmReturnParkedTolerates(kFmCondHeading | kFmCondTxStale, true, false, true));
  assert(!fmReturnParkedTolerates(kFmCondPhaseA, true, false, true));
  // Moving, or the trigger held: never (motion needs every input).
  assert(!fmReturnParkedTolerates(kFmCondTxStale, false, false, true));
  assert(!fmReturnParkedTolerates(kFmCondTxStale, true, true, true));
  // Nothing failing is not a tolerance question.
  assert(!fmReturnParkedTolerates(0, true, false, true));
}

static void testPivotSuspend()
{
  FmReturnPivotState s; fmReturnPivotReset(&s);
  const uint32_t base = 10000;
  // Inside the first 15 s: suspended whatever the error does.
  assert(fmReturnPivotSuspend(&s, base + 100, true, 120.0f, base, 15000, 2000, 2.0f, 45000));
  // A slow pivot, error falling 3 deg per second from 160: past the old 15 s limit (at 20 s the
  // error is 100, still aligning) it stays suspended because it is still making progress.
  float err = 160.0f;
  bool at20 = false;
  for (uint32_t t = base + 200; t <= base + 20000; t += 100) {
    if ((t - base) % 1000 == 0) err -= 3.0f;
    const bool v = fmReturnPivotSuspend(&s, t, true, err, base, 15000, 2000, 2.0f, 45000);
    if (t == base + 20000) at20 = v;
  }
  assert(at20);
  // A stalled error past 15 s: judged (not suspended) once 2 s pass without progress.
  fmReturnPivotReset(&s);
  bool v = true;
  for (uint32_t t = base + 100; t <= base + 18000; t += 100) v = fmReturnPivotSuspend(&s, t, true, 90.0f, base, 15000, 2000, 2.0f, 45000);
  assert(!v);
  // The ceiling: even steady progress is judged after 45 s.
  fmReturnPivotReset(&s);
  err = 179.0f;
  for (uint32_t t = base + 100; t <= base + 46000; t += 100) {
    err -= 0.05f;   // 0.5 deg/s: 2 deg every 4 s -> no progress within 2 s -> would stop at 15 s anyway
    v = fmReturnPivotSuspend(&s, t, true, err, base, 15000, 2000, 2.0f, 45000);
  }
  assert(!v);
  fmReturnPivotReset(&s);
  err = 179.0f;
  for (uint32_t t = base + 100; t <= base + 46000; t += 100) {
    err -= 0.25f;   // 2.5 deg/s: progress every second
    if (err < 46.0f) err = 179.0f;   // synthetic: keep it aligning and "improving" to test the ceiling
    v = fmReturnPivotSuspend(&s, t, true, err, base, 15000, 2000, 2.0f, 45000);
  }
  assert(!v);
  // Not aligning: never suspended, and the episode resets.
  assert(!fmReturnPivotSuspend(&s, base + 50000, false, 10.0f, base, 15000, 2000, 2.0f, 45000));
  assert(s.progress_ms == 0);
}

static void testCandidateMayForm()
{
  // ACTIVE / HOLD: as before, no extra requirement.
  assert(fmReturnCandidateMayForm(true, false, false, 2.0f, 12.0f));
  // ARMED with no engagement this run: never.
  assert(!fmReturnCandidateMayForm(false, true, false, 40.0f, 12.0f));
  // ARMED after an engagement, beyond D_engage: yes. On the rope (inside D_engage): no.
  assert(fmReturnCandidateMayForm(false, true, true, 40.0f, 12.0f));
  assert(!fmReturnCandidateMayForm(false, true, true, 7.1f, 12.0f));
  assert(!fmReturnCandidateMayForm(false, true, true, 12.0f, 12.0f));
  // Any other state: no.
  assert(!fmReturnCandidateMayForm(false, false, true, 40.0f, 12.0f));
}

int main()
{
  testHandbackCap();
  testRtmGateFault();
  testBootId();
  testRefreshExpiry();
  testStampAge();
  testFailingMask();
  testDistBlank();
  testParkedTolerance();
  testPivotSuspend();
  testCandidateMayForm();
  printf("auto_return_rules_test: all tests passed\n");
  return 0;
}
