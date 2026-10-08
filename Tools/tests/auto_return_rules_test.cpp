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

// N-5 backstop (2026-10-07): the cap also clears after the trigger stays below 25 for 1.0 s without a break. calcPWM()
// runs every 10 ms; this walks the same timer at 10 ms steps.
static void testHandbackBackstop()
{
  uint32_t since = 0;
  assert(kHandbackBackstopThr == 25 && kHandbackBackstopMs == 1000);
  // A remote idling at 12 counts (above the 8-count release, below 25): no instant clear...
  bool clr = true;
  assert(handbackCapStep(0, 12, 12, 8, &clr) == 0 && !clr);
  // ...but the backstop clears after exactly 1.0 s low, not before.
  uint32_t t = 5000;
  for (int i = 0; i < 100; ++i, t += 10) assert(!handbackBackstopStep(t, 12, 25, 1000, &since));
  assert(handbackBackstopStep(t, 12, 25, 1000, &since));   // t = start + 1000 ms
  // A squeeze to 25 (the threshold itself) at 0.9 s restarts the run.
  since = 0; t = 20000;
  for (int i = 0; i < 90; ++i, t += 10) assert(!handbackBackstopStep(t, 20, 25, 1000, &since));
  assert(!handbackBackstopStep(t, 25, 25, 1000, &since) && since == 0); t += 10;
  for (int i = 0; i < 100; ++i, t += 10) assert(!handbackBackstopStep(t, 0, 25, 1000, &since));
  assert(handbackBackstopStep(t, 0, 25, 1000, &since));
  // Held trigger: never clears, however long.
  since = 0; t = 40000;
  for (int i = 0; i < 1000; ++i, t += 10) assert(!handbackBackstopStep(t, 200, 25, 1000, &since));
  // millis() == 0 is not mistaken for "no run", and the timer survives the 32-bit wrap.
  since = 0;
  assert(!handbackBackstopStep(0, 3, 25, 1000, &since) && since == 1);
  since = 0; t = 0xFFFFFF00u;
  assert(!handbackBackstopStep(t, 3, 25, 1000, &since));
  assert(handbackBackstopStep(t + 1000u, 3, 25, 1000, &since));
}

// N-2 / N-3 / N-4: every RTM end (Phase C, Gate 9, H-1, H-2, S-8) arms kRtmEndHandbackCap. Phase C sequence:
// tick 0 the FAIL raises the emergency stop and arms the cap; tick 1 the inactive path DROPS the emergency
// stop - the trigger is still held at 100 %, and the motor must stay at 0; a feathered 9 counts is still not a
// release; one full release clears it; the next squeeze is plain manual.
static void testRtmEndHandback()
{
  assert(kRtmEndHandbackCap == 0);
  uint8_t cap = kHandbackNone;
  bool clr = false;
  // Before the fix nothing was armed: after the e-stop dropped, the held 255 came straight back.
  assert(handbackCapStep(cap, 255, 255, 8, &clr) == 255);
  // With the fix: armed FIRST at the Phase C end (handbackCapArm only lowers: min(none, 0) = 0).
  cap = (kRtmEndHandbackCap < cap) ? kRtmEndHandbackCap : cap;
  // e-stop tick (effective already 0) and the e-stop-dropped tick (effective = the held trigger): both 0.
  assert(handbackCapStep(cap, 0, 255, 8, &clr) == 0 && !clr);
  assert(handbackCapStep(cap, 255, 255, 8, &clr) == 0 && !clr);
  // Feathering (9 counts) is not a full release.
  assert(handbackCapStep(cap, 9, 9, 8, &clr) == 0 && !clr);
  // Full release: cleared.
  assert(handbackCapStep(cap, 0, 0, 8, &clr) == 0 && clr);
  if (clr) cap = kHandbackNone;
  // Next squeeze: manual, uncapped (the ramp in calcPWM() softens it from 0).
  assert(handbackCapStep(cap, 200, 200, 8, &clr) == 200 && !clr);
  // N-3: Gate 9 with the approach zone off - rtm_approach_cap was 255, which used to be what was armed
  // (no cap at all). kRtmEndHandbackCap holds the motor at 0 instead.
  const uint8_t zone_off_cap = 255;
  assert(handbackCapStep(zone_off_cap, 255, 255, 8, &clr) == 255);       // the old arm: nothing held
  assert(handbackCapStep(kRtmEndHandbackCap, 255, 255, 8, &clr) == 0);   // the new arm
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

// N-12: the refresh expiry counts link-fresh time only.
static void testRefreshExpiryOnLink()
{
  // Link fresh for 20 s, last refresh 6 s ago: expired (the remote talks to us but no longer says RTM).
  assert(rtmRefreshExpiredOnLink(true, true, 14000, 20000, 5000, true, 1000));
  // Link DOWN: never (that is gate 7 / H-2's job), however old the refresh.
  assert(!rtmRefreshExpiredOnLink(true, true, 1000, 60000, 5000, false, 0));
  // Link just back after a 10 s gap (fresh since 1 s), refresh 12 s old: NOT yet - the remote gets 5 s
  // of link to send its next refresh.
  assert(!rtmRefreshExpiredOnLink(true, true, 18000, 30000, 5000, true, 29000));
  // ... and if it still has not refreshed after 5 s of fresh link: expired.
  assert(rtmRefreshExpiredOnLink(true, true, 18000, 34001, 5000, true, 29000));
  // A refresh arriving while the link is back: fine.
  assert(!rtmRefreshExpiredOnLink(true, true, 33000, 34001, 5000, true, 29000));
  // Stamp newer than now (N-1) and link-fresh-since newer than now: fresh / not long enough.
  assert(!rtmRefreshExpiredOnLink(true, true, 40001, 40000, 5000, true, 1000));
  assert(!rtmRefreshExpiredOnLink(true, true, 1000, 40000, 5000, true, 40002));
  // Today's remote (no refresh ever seen): never.
  assert(!rtmRefreshExpiredOnLink(true, false, 0, 600000, 5000, true, 1));
}

// N-8(b): a sticky telemetry window counts down only while the link is fresh.
static void testStickyWindow()
{
  StickyWindowState s = {0, 0, 0};
  uint32_t t = 10000;
  // No alarm yet: never set.
  assert(!stickyWindowStep(&s, t, 0, true, 6000)); t += 100;
  // An alarm raised at a time the link is DOWN (a link-loss fault): set, and it stays set through 20 s of
  // link loss - the old wall-clock window was gone after 6 s, before the remote heard anything.
  const uint32_t alarm = t;
  assert(stickyWindowStep(&s, t, alarm, false, 6000)); t += 100;
  for (int i = 0; i < 200; ++i, t += 100) assert(stickyWindowStep(&s, t, alarm, false, 6000));
  // Link back: 6 s of fresh link, then it drops.
  int shown = 0;
  for (int i = 0; i < 100; ++i, t += 100) if (stickyWindowStep(&s, t, alarm, true, 6000)) ++shown;
  assert(shown >= 59 && shown <= 61);
  assert(!stickyWindowStep(&s, t, alarm, true, 6000)); t += 100;
  // A gap in the middle pauses the count: 3 s fresh, 5 s down, then 3 s fresh more.
  StickyWindowState w = {0, 0, 0};
  t = 50000;
  const uint32_t a2 = t;
  assert(stickyWindowStep(&w, t, a2, true, 6000)); t += 100;
  for (int i = 0; i < 29; ++i, t += 100) assert(stickyWindowStep(&w, t, a2, true, 6000));
  for (int i = 0; i < 50; ++i, t += 100) assert(stickyWindowStep(&w, t, a2, false, 6000));
  bool still = true; int more = 0;
  for (int i = 0; i < 40 && still; ++i, t += 100) { still = stickyWindowStep(&w, t, a2, true, 6000); if (still) ++more; }
  assert(more >= 29 && more <= 31);
  // A NEW alarm (different stamp) restarts a full window.
  const uint32_t a3 = t;
  assert(stickyWindowStep(&w, t, a3, true, 6000)); t += 100;
  assert(w.left_ms == 6000 || w.left_ms == 5900);
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

// N-10: no return motion after a tolerated link gap until a boot ID has been received again.
static void testReturnGap()
{
  FmReturnGapState g = {false, 0};
  // Today's remote (no boot ID ever): a gap never makes it wait (the link loss is a fault for it anyway).
  assert(!fmReturnGapStep(&g, true, 0, false));
  assert(!fmReturnGapStep(&g, false, 0, false));
  // A boot-ID remote: link down -> waiting; link back with no new boot ID -> still waiting.
  g = {false, 0};
  uint8_t seq = 5;
  assert(!fmReturnGapStep(&g, false, seq, true));
  assert(fmReturnGapStep(&g, true, seq, true));
  assert(fmReturnGapStep(&g, true, seq, true));
  assert(fmReturnGapStep(&g, false, seq, true));      // control packets are back, the remote has not said who it is
  assert(fmReturnGapStep(&g, false, seq, true));
  ++seq;                                               // its boot ID arrives (same ID: same remote)
  assert(!fmReturnGapStep(&g, false, seq, true));
  assert(!fmReturnGapStep(&g, false, seq, true));
  // The packet that ends the gap may itself be the boot ID: it counts.
  assert(fmReturnGapStep(&g, true, seq, true));
  ++seq;
  assert(!fmReturnGapStep(&g, false, seq, true));
  // Counter wrap 255 -> 0 still counts as "moved".
  g = {false, 0};
  assert(fmReturnGapStep(&g, true, 255, true));
  assert(!fmReturnGapStep(&g, false, 0, true));
}

// N-13 + owner rule "waiting means it will work": the parked verdict.
static void testParkVerdict()
{
  // All good: GO.
  assert(fmReturnParkVerdict(0, true, false, true, false) == FMRPV_GO);
  assert(fmReturnParkVerdict(0, false, true, true, false) == FMRPV_GO);
  // Parked, released, the remote's side missing: NOT READY (bit 2) - never "waiting".
  assert(fmReturnParkVerdict(kFmCondTxStale, true, false, false, false) == FMRPV_NOT_READY);
  assert(fmReturnParkVerdict(kFmCondLink, true, false, true, false) == FMRPV_NOT_READY);
  assert(fmReturnParkVerdict(kFmCondLink | kFmCondTxStale | kFmCondPhaseB, true, false, true, false) == FMRPV_NOT_READY);
  // Link down for a remote without a boot ID: a fault, as before (St).
  assert(fmReturnParkVerdict(kFmCondLink, true, false, false, false) == FMRPV_FAULT);
  // N-10: link back, boot ID not heard again: NOT READY while released...
  assert(fmReturnParkVerdict(0, true, false, true, true) == FMRPV_NOT_READY);
  // ... and a squeeze then is a fault (St), never motion.
  assert(fmReturnParkVerdict(0, true, true, true, true) == FMRPV_FAULT);
  // A squeeze while not ready: fault (St).
  assert(fmReturnParkVerdict(kFmCondTxStale, true, true, true, false) == FMRPV_FAULT);
  // The buggy's own sensors while parked: the return ends at once (St + buzz), never "waiting".
  assert(fmReturnParkVerdict(kFmCondRxStale, true, false, true, false) == FMRPV_FAULT);
  assert(fmReturnParkVerdict(kFmCondHeading, true, false, true, false) == FMRPV_FAULT);
  assert(fmReturnParkVerdict(kFmCondPhaseA | kFmCondTxStale, true, false, true, false) == FMRPV_FAULT);
  assert(fmReturnParkVerdict(kFmCondPhaseB, true, false, true, false) == FMRPV_FAULT);   // a real handshake mismatch
  // Moving with anything failing: fault.
  assert(fmReturnParkVerdict(kFmCondTxStale, false, true, true, false) == FMRPV_FAULT);
}

// N-9: a stall is an arrival only in the final crawl.
// P-3 (owner ruling, 2026-10-07): a handshake failing ONLY on the pairing distance is waited through while parked
// and released (NOT_READY); a speed-check failure, a held trigger or a moving return is still a FAULT; the buggy's
// own sensors still end it whatever the distance flag says.
static void testParkVerdictDistance()
{
  // Default argument: unchanged behaviour (a lone handshake failure is a fault).
  assert(fmReturnParkVerdict(kFmCondPhaseB, true, false, true, false) == FMRPV_FAULT);
  // Distance-only, parked, released: wait.
  assert(fmReturnParkVerdict(kFmCondPhaseB, true, false, true, false, true) == FMRPV_NOT_READY);
  assert(fmReturnParkVerdict(kFmCondPhaseB, true, false, false, false, true) == FMRPV_NOT_READY);
  // Speed-check failure (flag false): fault.
  assert(fmReturnParkVerdict(kFmCondPhaseB, true, false, true, false, false) == FMRPV_FAULT);
  // Distance-only but the trigger held, or the return moving: fault.
  assert(fmReturnParkVerdict(kFmCondPhaseB, true, true, true, false, true) == FMRPV_FAULT);
  assert(fmReturnParkVerdict(kFmCondPhaseB, false, false, true, false, true) == FMRPV_FAULT);
  // Distance-only together with the buggy's own sensors failing: fault.
  assert(fmReturnParkVerdict(kFmCondPhaseB | kFmCondHeading, true, false, true, false, true) == FMRPV_FAULT);
  assert(fmReturnParkVerdict(kFmCondPhaseB | kFmCondRxStale, true, false, true, false, true) == FMRPV_FAULT);
  // Distance-only together with the rider's GPS stale: still a wait (as before).
  assert(fmReturnParkVerdict(kFmCondPhaseB | kFmCondTxStale, true, false, true, false, true) == FMRPV_NOT_READY);
  // Link down without a boot ID: still a fault whatever the distance flag says.
  assert(fmReturnParkVerdict(kFmCondPhaseB | kFmCondLink, true, false, false, false, true) == FMRPV_FAULT);
}

static void testStallIsArrival()
{
  const float stop = 3.0f;
  // The final crawl: inside the zone, within stop + 3 m, buggy stopped.
  assert(fmReturnStallIsArrival(true, 5.5f, stop, 180, false, 0.3f));
  // Further out but the approach ramp had it crawling (cap 30) and the buggy stopped: arrival.
  assert(fmReturnStallIsArrival(true, 7.0f, stop, 30, false, 0.5f));
  // THE AUDIT CASE: circling / mirrored at 10 m, cap about 198: a fault, not a silent arrival.
  assert(!fmReturnStallIsArrival(true, 10.0f, stop, 198, false, 0.8f));
  // Low cap only because it is aligning (a stalled pivot at the align cap): a fault.
  assert(!fmReturnStallIsArrival(true, 10.0f, stop, 13, true, 0.2f));
  // Still moving (>= 1.5 km/h) or speed unknown: a fault even close in.
  assert(!fmReturnStallIsArrival(true, 4.0f, stop, 10, false, 2.0f));
  assert(!fmReturnStallIsArrival(true, 4.0f, stop, 10, false, -1.0f));
  // Outside the zone (or the zone off): always a fault.
  assert(!fmReturnStallIsArrival(false, 4.0f, stop, 10, false, 0.1f));
  // The boundary: stop + 3 m exactly counts; 1.5 km/h exactly does not; cap 40 counts, 41 does not.
  assert(fmReturnStallIsArrival(true, 6.0f, stop, 200, false, 1.0f));
  assert(!fmReturnStallIsArrival(true, 6.0f, stop, 200, false, 1.5f));
  assert(fmReturnStallIsArrival(true, 9.0f, stop, 40, false, 1.0f));
  assert(!fmReturnStallIsArrival(true, 9.0f, stop, 41, false, 1.0f));

  // P-4 (2026-10-07): the cap passed in is the APPROACH-RAMP term at dist_m, fmReturnApproachRampCap().
  // Ramp shape: 255 outside the zone / ramp off / zone not wider than the stop radius; 0 at the stop radius.
  assert(fmReturnApproachRampCap(13.0f, stop, 12.0f) == 255);
  assert(fmReturnApproachRampCap(12.0f, stop, 12.0f) == 255);
  assert(fmReturnApproachRampCap(5.0f, stop, 0.0f) == 255);
  assert(fmReturnApproachRampCap(5.0f, 12.0f, 12.0f) == 255);
  assert(fmReturnApproachRampCap(3.0f, stop, 12.0f) == 0);
  assert(fmReturnApproachRampCap(2.0f, stop, 12.0f) == 0);
  assert(fmReturnApproachRampCap(7.5f, stop, 12.0f) == 127);   // half way: (4.5 / 9) x 255 = 127.5
  // THE AUDIT CASE: a stall at 8 m in a 12 m zone (stop 3). The previous tick's TOTAL cap could be 13 there (an
  // align cap, or a re-squeeze's engage ramp) and used to read as a crawl - a silent arrival. The ramp term at
  // 8 m is 141: a fault (St), as it must be.
  const uint8_t r8 = fmReturnApproachRampCap(8.0f, stop, 12.0f);
  assert(r8 == 141);
  assert(!fmReturnStallIsArrival(true, 8.0f, stop, r8, false, 0.3f));
  // The real final crawl still counts: 4.4 m in the same zone gives a ramp term of 39.
  const uint8_t r44 = fmReturnApproachRampCap(4.4f, stop, 12.0f);
  assert(r44 <= kFmReturnCrawlCapMax);
  assert(fmReturnStallIsArrival(true, 4.4f, stop, r44, false, 0.3f));
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
  testRtmEndHandback();
  testHandbackBackstop();
  testRtmGateFault();
  testBootId();
  testRefreshExpiry();
  testRefreshExpiryOnLink();
  testStickyWindow();
  testStampAge();
  testFailingMask();
  testDistBlank();
  testParkedTolerance();
  testReturnGap();
  testParkVerdict();
  testParkVerdictDistance();
  testStallIsArrival();
  testPivotSuspend();
  testCandidateMayForm();
  printf("auto_return_rules_test: all tests passed\n");
  return 0;
}
