// V2.5-Evo - 2026-10-17 - HEADING LATCH WRITTEN ONCE PER CHANGE (audit F1-F5): host test for
//   Source/Common/HeadingLatchPersist.h. A 100 ms tick simulator drives the latch path the way RTMState.ino does:
//   K heading calls per tick (K = 1-3, the call sites of getRtmHeading()), each one either convicting (the set site,
//   hdgLatchSet(true)), acquitting (the clear site, hdgLatchSet(false) only while the latch stands) or measuring
//   nothing; then, once at the end of the tick, the service (hdgPersistDue / apply / hdgPersistResult), gated on a
//   scripted trigger. A fake apply(want) stands in for the SPIFFS write / remove: it counts calls and returns a
//   scripted result.
//   T1 one store for a 92 s disagreement (pre-fix count printed for contrast); T2 a rise under a held trigger is stored
//   on the first released tick; T3 one remove for 300 s of agreement, then nothing; T4 a restored latch never
//   rewrites; T5 a failing store is retried at >= 5000 ms and the warning prints once; T6 rise then clear under a
//   held trigger stores only the clear; T7 the cal clear overrides a pending set; T8 the retry gap across the
//   32-bit millis() wrap.
// Build and run (Git Bash, PATH and TMP as in the host-test note of the compile procedure):
//   from the repo root:  g++ -std=c++17 -Wall -Wextra -I Source -I Source/Common -o heading_latch_persist_test Tools/tests/heading_latch_persist_test.cpp && ./heading_latch_persist_test
//   or from Tools/tests: g++ -std=c++17 -Wall -Wextra -o heading_latch_persist_test heading_latch_persist_test.cpp && ./heading_latch_persist_test

#include <cstdio>
#include <cstdint>

#include "../../Source/Common/HeadingLatchPersist.h"

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static const uint32_t kRetry = 5000;   // the RX's kHdgPersistRetryMs
static const uint32_t kTick  = 100;    // the RX's 10 Hz loop

// ============================================================
// THE MODEL - the RX's latch, its request, and a fake file system
// ============================================================
// What one heading call measured: nothing comparable, a disagreement past the dwell (convict), or an agreement past
// the dwell (acquit). The dwell bookkeeping itself is not part of the header and is not modelled; a verdict here is
// "this call reached the set / clear site".
enum Verdict { V_NONE = 0, V_CONVICT = 1, V_ACQUIT = 2 };

struct Rx {
  volatile bool latch;      // heading_disagree_fault
  HdgPersist    p;          // hdg_persist
  int  requests;            // hdgLatchSet() calls that returned true
  int  warnings;            // WARNING lines the service would print (first failure of a request)
  // the fake file system
  int  applies;             // apply() calls, successful or not (each is SPIFFS I/O on the RX)
  int  fail_remaining;      // apply() fails this many more times, then succeeds
  bool last_value;          // want of the most recent apply() call
  bool file_present;        // what the file system holds after the last successful apply()
  uint32_t attempt_ms[16];  // clock of each apply() call (first 16)
};

static void rxInit(Rx* rx, bool latch, bool want, bool dirty, bool file_present)
{
  rx->latch = latch;
  rx->p.want = want; rx->p.dirty = dirty; rx->p.fail_ms = 0;
  rx->requests = 0; rx->warnings = 0;
  rx->applies = 0; rx->fail_remaining = 0; rx->last_value = false; rx->file_present = file_present;
  for (int i = 0; i < 16; ++i) rx->attempt_ms[i] = 0;
}

// The clock of apply() call i, or 0xFFFFFFFF when it was not recorded (so a broken header fails a CHECK cleanly
// instead of reading past the array).
static uint32_t attemptAt(const Rx* rx, int i)
{
  return (i >= 0 && i < 16 && i < rx->applies) ? rx->attempt_ms[i] : 0xFFFFFFFFu;
}

// headingDisagreePersist(): the scripted SPIFFS write / remove.
static bool fakeApply(Rx* rx, bool want, uint32_t now)
{
  if (rx->applies < 16) rx->attempt_ms[rx->applies] = now;
  rx->applies++;
  rx->last_value = want;
  if (rx->fail_remaining > 0) { rx->fail_remaining--; return false; }
  rx->file_present = want;
  return true;
}

// One heading call: the set site (every convicting call after the dwell reaches it) and the clear site (the whole
// body under `if (heading_disagree_fault)`), exactly as RTMState.ino calls the header.
static void headingCall(Rx* rx, Verdict v)
{
  if (v == V_CONVICT) {
    if (hdgLatchSet(&rx->latch, &rx->p, true)) rx->requests++;
  } else if (v == V_ACQUIT) {
    if (rx->latch) { if (hdgLatchSet(&rx->latch, &rx->p, false)) rx->requests++; }
  }
}

// headingDisagreeClearAfterCal(): an unconditional clear request (its own "nothing to forgive" test passes here).
static void calClear(Rx* rx)
{
  if (hdgLatchSet(&rx->latch, &rx->p, false)) rx->requests++;
}

// headingDisagreePersistService(): once per tick, after the tick's heading calls.
static void service(Rx* rx, uint32_t now, bool released)
{
  if (!hdgPersistDue(&rx->p, now, released, kRetry)) return;
  const bool first = (rx->p.fail_ms == 0);
  const bool ok    = fakeApply(rx, rx->p.want, now);
  hdgPersistResult(&rx->p, now, ok);
  if (!ok && first) rx->warnings++;
}

// One 10 Hz tick: K heading calls with the same verdict, then the service. Returns the number of heading calls that
// reached the set / clear site, which is what the pre-fix code turned into SPIFFS calls one for one.
static int tick(Rx* rx, uint32_t now, int K, Verdict v, bool released)
{
  for (int k = 0; k < K; ++k) headingCall(rx, v);
  service(rx, now, released);
  return (v == V_NONE) ? 0 : K;
}

// ============================================================
// T1 - a 92 s disagreement makes ONE request and ONE store
// ============================================================
static void testT1(Rx* rx)
{
  rxInit(rx, false, false, false, false);
  int prefix_calls = 0;
  for (uint32_t t = 0; t < 97000; t += kTick) {
    const Verdict v = (t >= 5000) ? V_CONVICT : V_NONE;
    prefix_calls += tick(rx, t, 3, v, true);
  }
  CHECK(rx->latch == true);
  CHECK(rx->requests == 1);
  CHECK(rx->applies == 1);
  CHECK(attemptAt(rx, 0) == 5000);
  CHECK(rx->last_value == true && rx->file_present == true);
  CHECK(rx->p.dirty == false && rx->p.fail_ms == 0);
  CHECK(prefix_calls == 2760);
  std::printf("T1 convict 5.0-97 s, K=3, released: %d request, %d apply at t=%lu ms (value %s); pre-fix rewrites: %d\n",
              rx->requests, rx->applies, (unsigned long)attemptAt(rx, 0), rx->last_value ? "true" : "false", prefix_calls);
}

// ============================================================
// T2 - a rise while the trigger is held is stored on the first released tick, not before
// ============================================================
static void testT2()
{
  Rx rx; rxInit(&rx, false, false, false, false);
  int applies_before_6s = -1;
  for (uint32_t t = 0; t < 97000; t += kTick) {
    const bool released = !(t >= 4500 && t < 6000);
    const Verdict v = (t >= 5000) ? V_CONVICT : V_NONE;
    if (t == 6000) applies_before_6s = rx.applies;   // read before this tick runs
    tick(&rx, t, 3, v, released);
  }
  CHECK(applies_before_6s == 0);
  CHECK(rx.requests == 1);
  CHECK(rx.applies == 1);
  CHECK(attemptAt(&rx, 0) == 6000);
  CHECK(rx.last_value == true && rx.p.dirty == false);
  std::printf("T2 as T1, trigger held 4.5-6.0 s: %d applies before 6.0 s, %d apply at t=%lu ms\n",
              applies_before_6s, rx.applies, (unsigned long)attemptAt(&rx, 0));
}

// ============================================================
// T3 - after T1, 300 s of agreement makes ONE clear request and ONE remove, then nothing
// ============================================================
static void testT3(Rx* rx)
{
  const int req0 = rx->requests, app0 = rx->applies;
  int prefix_lookups = 0;
  int applies_after_first_tick = -1;
  for (uint32_t t = 97000; t < 397000; t += kTick) {
    prefix_lookups += tick(rx, t, 3, V_ACQUIT, true);
    if (t == 97000) applies_after_first_tick = rx->applies;
  }
  CHECK(rx->latch == false);
  CHECK(rx->requests - req0 == 1);
  CHECK(rx->applies - app0 == 1);
  CHECK(attemptAt(rx, app0) == 97000);
  CHECK(rx->last_value == false && rx->file_present == false);
  CHECK(applies_after_first_tick == rx->applies);   // nothing after the first acquitting tick
  CHECK(rx->p.dirty == false);
  std::printf("T3 acquit 300 s after T1, K=3: %d clear request, %d apply(false) at t=%lu ms, then 0 applies; pre-fix lookups: %d\n",
              rx->requests - req0, rx->applies - app0, (unsigned long)attemptAt(rx, app0), prefix_lookups);
}

// ============================================================
// T4 - a latch restored at boot (latch true, want true, dirty false) never rewrites
// ============================================================
static void testT4()
{
  Rx rx; rxInit(&rx, true, true, false, true);
  for (uint32_t t = 0; t < 92000; t += kTick) tick(&rx, t, 3, V_CONVICT, true);
  CHECK(rx.latch == true);
  CHECK(rx.requests == 0);
  CHECK(rx.applies == 0);
  CHECK(rx.p.want == true && rx.p.dirty == false && rx.p.fail_ms == 0);
  // A same-value hdgLatchSet() leaves the request untouched, even a failed one waiting for its retry.
  HdgPersist q = { true, true, 1234 };
  volatile bool l = true;
  CHECK(hdgLatchSet(&l, &q, true) == false);
  CHECK(l == true && q.want == true && q.dirty == true && q.fail_ms == 1234);
  std::printf("T4 restored latch, convict 92 s, K=3: %d requests, %d applies\n", rx.requests, rx.applies);
}

// ============================================================
// T5 - a store that fails 3 times is retried no sooner than 5000 ms apart, then succeeds; one warning
// ============================================================
static void testT5()
{
  Rx rx; rxInit(&rx, false, false, false, false);
  rx.fail_remaining = 3;
  for (uint32_t t = 0; t < 30000; t += kTick) {
    const Verdict v = (t == 1000) ? V_CONVICT : V_NONE;
    tick(&rx, t, 1, v, true);
  }
  CHECK(rx.applies == 4);
  bool gaps_ok = true;
  for (int i = 1; i < 4; ++i) if ((uint32_t)(attemptAt(&rx, i) - attemptAt(&rx, i - 1)) < kRetry) gaps_ok = false;
  CHECK(gaps_ok);
  CHECK(rx.p.dirty == false && rx.p.fail_ms == 0);
  CHECK(rx.file_present == true);
  CHECK(rx.warnings == 1);
  std::printf("T5 3 failures then success: %d attempts at %lu / %lu / %lu / %lu ms, %d warning, dirty %s\n",
              rx.applies, (unsigned long)attemptAt(&rx, 0), (unsigned long)attemptAt(&rx, 1),
              (unsigned long)attemptAt(&rx, 2), (unsigned long)attemptAt(&rx, 3), rx.warnings,
              rx.p.dirty ? "true" : "false");
}

// ============================================================
// T6 - rise at t, clear at t + 0.3 s, trigger held throughout, release at t + 1.0 s: one apply, value false
// ============================================================
static void testT6()
{
  Rx rx; rxInit(&rx, false, false, false, false);
  const uint32_t t0 = 10000;
  for (uint32_t t = 0; t < 20000; t += kTick) {
    const bool released = !(t >= t0 && t < t0 + 1000);
    Verdict v = V_NONE;
    if (t == t0)       v = V_CONVICT;
    if (t == t0 + 300) v = V_ACQUIT;
    tick(&rx, t, 1, v, released);
  }
  CHECK(rx.requests == 2);
  CHECK(rx.applies == 1);
  CHECK(attemptAt(&rx, 0) == t0 + 1000);
  CHECK(rx.last_value == false && rx.file_present == false);
  CHECK(rx.latch == false && rx.p.dirty == false);
  std::printf("T6 rise + clear under a held trigger: %d apply at t0+%lu ms, value %s\n",
              rx.applies, (unsigned long)(attemptAt(&rx, 0) - t0), rx.last_value ? "true" : "false");
}

// ============================================================
// T7 - a set pending under a held trigger, then the cal clear, then release: one apply, value false
// ============================================================
static void testT7()
{
  Rx rx; rxInit(&rx, false, false, false, false);
  const uint32_t t0 = 10000;
  for (uint32_t t = 0; t < 20000; t += kTick) {
    const bool released = !(t >= t0 && t < t0 + 2000);
    const Verdict v = (t == t0) ? V_CONVICT : V_NONE;
    if (t == t0 + 500) calClear(&rx);   // ?compasscal / ?magalign, between ticks
    tick(&rx, t, 1, v, released);
  }
  CHECK(rx.requests == 2);
  CHECK(rx.applies == 1);
  CHECK(attemptAt(&rx, 0) == t0 + 2000);
  CHECK(rx.last_value == false && rx.file_present == false);
  CHECK(rx.latch == false && rx.p.dirty == false);
  std::printf("T7 pending set, then cal clear, then release: %d apply at t0+%lu ms, value %s\n",
              rx.applies, (unsigned long)(attemptAt(&rx, 0) - t0), rx.last_value ? "true" : "false");
}

// ============================================================
// T8 - the retry gap across the 32-bit wrap of millis()
// ============================================================
static void testT8()
{
  const HdgPersist p = { true, true, 0xFFFFF000u };
  CHECK(hdgPersistDue(&p, 0x00000388u, true, kRetry) == true);    // 4096 + 904 = 5000 ms later
  CHECK(hdgPersistDue(&p, 0x00000387u, true, kRetry) == false);   // 4999 ms later
  CHECK(hdgPersistDue(&p, 0x00000388u, false, kRetry) == false);  // due, but the trigger is held
  const HdgPersist clean = { true, false, 0 };
  CHECK(hdgPersistDue(&clean, 0x00000388u, true, kRetry) == false);   // nothing owed
  // A failure at clock 0 is stored as 1, so 0 keeps meaning "no failure".
  HdgPersist z = { true, true, 0 };
  hdgPersistResult(&z, 0, false);
  CHECK(z.fail_ms == 1 && z.dirty == true);
  std::printf("T8 wrap: fail_ms 0xFFFFF000, due at 0x00000388 (5000 ms): %s, at 0x00000387 (4999 ms): %s\n",
              hdgPersistDue(&p, 0x00000388u, true, kRetry) ? "yes" : "no",
              hdgPersistDue(&p, 0x00000387u, true, kRetry) ? "yes" : "no");
}

int main()
{
  Rx rx;            // T1 and T3 share one board: T3 runs "after T1"
  testT1(&rx);
  testT2();
  testT3(&rx);
  testT4();
  testT5();
  testT6();
  testT7();
  testT8();
  if (fails) { std::printf("heading_latch_persist_test: %d of %d checks FAILED\n", fails, checks); return 1; }
  std::printf("heading_latch_persist_test: all %d checks passed\n", checks);
  return 0;
}
