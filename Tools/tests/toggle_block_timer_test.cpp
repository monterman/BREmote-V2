// V2.5-Evo - 2026-10-21 - host test for Source/Common/ToggleBlockTimer.h: the remote's "toggle is steering" wait
//   (tog_block_time) timed in real milliseconds. calcFilter() (Hall.ino) calls toggleBlockStep() once per pass of the
//   10 ms ADC task with: hold = trigger squeezed (or the forced block), remote_error, centred = steer_scaled == 127,
//   block_10ms = usrConf.tog_block_time, now = millis().
//   1. The wait is REAL time: 200 frees the toggle at 2.0 s and 300 at 3.0 s, whether the passes come every 10 ms,
//      every 27 ms or irregularly. The old pass counter (modelled below, line for line from the old calcFilter())
//      matches it within one pass at exactly 10 ms, and stretches to 5.4 s at 27 ms passes.
//   2. Unchanged rules: a squeeze blocks and restarts the wait; a toggle off centre restarts it; once served, toggle
//      movement does not block again (only a squeeze does); a remote error frees the toggle at once and the wait
//      stays served after the error clears; 0 never frees it (except the error).
//   3. Boot (zero-init) state, the millis() roll-over, and the largest values (5000 = 50 s, 65535) do not overflow.
// Build and run (Git Bash, from Tools/tests, PATH and TMP as in SOP-012's host-test note):
//   g++ -std=c++17 -Wall -Wextra -o toggle_block_timer_test toggle_block_timer_test.cpp && ./toggle_block_timer_test

#include <cstdio>
#include <cstdint>

#include "../../Source/Common/ToggleBlockTimer.h"

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

// The remote as calcFilter() drives it: the timer memory plus the block flag.
struct Remote {
  ToggleBlockTimer t = {0u, false};
  bool blocked = false;
  bool pass(bool hold, bool err, bool centred, uint16_t T, uint32_t now) {
    blocked = toggleBlockStep(&t, blocked, hold, err, centred, T, now);
    return blocked;
  }
};

// The OLD calcFilter() block, line for line, as a reference model (counts passes).
struct OldRemote {
  uint16_t counter = 0;
  bool blocked = false;
  void pass(bool hold, bool err, bool centred, uint16_t T) {
    if (hold) { blocked = true; counter = 0; }
    else if (!err) {
      if (counter < T) { if (!centred) counter = 0; else counter++; }
      else { if (T != 0) blocked = false; }
    }
    else { counter = T; blocked = false; }
  }
};

// Squeeze at time t0 (one pass), then release with the toggle centred, passes every `period` ms.
// Returns the time (ms after the squeeze) of the first pass that frees the toggle, or UINT32_MAX if none
// within `limit` ms.
static uint32_t timeToFree(uint16_t T, uint32_t period, uint32_t t0, uint32_t limit) {
  Remote r;
  r.pass(true, false, true, T, t0);
  for (uint32_t dt = period; dt <= limit; dt += period) {
    if (!r.pass(false, false, true, T, t0 + dt)) return dt;
  }
  return UINT32_MAX;
}
static uint32_t oldTimeToFree(uint16_t T, uint32_t period, uint32_t limit) {
  OldRemote r;
  r.pass(true, false, true, T);
  for (uint32_t dt = period; dt <= limit; dt += period) {
    r.pass(false, false, true, T);
    if (!r.blocked) return dt;
  }
  return UINT32_MAX;
}

int main() {
  // ---- 1. Real time, whatever the pass rate ----
  CHECK(timeToFree(200, 10, 1000, 60000) == 2000);    // 200 = 2.0 s at 10 ms passes
  CHECK(timeToFree(300, 10, 1000, 60000) == 3000);    // 300 = 3.0 s (the owner's planned value)
  // At 27 ms passes the toggle is freed on the first pass at or after 2.0 s: 75 x 27 = 2025 ms.
  CHECK(timeToFree(200, 27, 1000, 60000) == 2025);
  CHECK(timeToFree(300, 27, 1000, 60000) == 3024);    // 112 x 27
  CHECK(timeToFree(110, 10, 1000, 60000) == 1100);
  CHECK(timeToFree(1, 10, 1000, 60000) == 10);        // smallest non-zero value: one pass
  // Exact boundary: at 1999 ms still steering, at 2000 ms free.
  {
    Remote r;
    r.pass(true, false, true, 200, 5000);
    CHECK(r.pass(false, false, true, 200, 6999) == true);
    CHECK(r.pass(false, false, true, 200, 7000) == false);
  }
  // Irregular passes (5-40 ms apart) - freed on the first pass at or after 2000 ms, never before.
  {
    Remote r;
    r.pass(true, false, true, 200, 0);
    uint32_t now = 0, freed_at = 0;
    const uint32_t gaps[] = {5, 40, 12, 33, 9, 27, 18};
    for (int i = 0; freed_at == 0 && i < 1000; i++) {
      now += gaps[i % 7];
      if (!r.pass(false, false, true, 200, now)) freed_at = now;
      else CHECK(now < 2000);
    }
    CHECK(freed_at >= 2000 && freed_at < 2040);
  }
  // The old pass counter: one pass later than the new timer at exactly 10 ms, and 5.4 s at 27 ms passes.
  CHECK(oldTimeToFree(200, 10, 60000) == 2010);
  CHECK(oldTimeToFree(200, 27, 60000) == 5427);       // (200 + 1) x 27 - what a slow pass rate did to "2 s"

  // ---- 2. Unchanged rules ----
  // A squeeze in the middle of the wait restarts it from the release.
  {
    Remote r;
    r.pass(true, false, true, 200, 0);
    for (uint32_t t = 10; t <= 1500; t += 10) CHECK(r.pass(false, false, true, 200, t));
    CHECK(r.pass(true, false, true, 200, 1510));       // squeeze again at 1.51 s
    for (uint32_t t = 1520; t < 3510; t += 10) CHECK(r.pass(false, false, true, 200, t));
    CHECK(r.pass(false, false, true, 200, 3510) == false);   // 2.0 s after the last squeeze pass
  }
  // The toggle off centre (steering) restarts the wait; it is measured from the last off-centre pass.
  {
    Remote r;
    r.pass(true, false, true, 200, 0);
    for (uint32_t t = 10; t <= 1900; t += 10) CHECK(r.pass(false, false, true, 200, t));
    CHECK(r.pass(false, false, false, 200, 1910));     // a steering move (or an early gear try) at 1.91 s
    CHECK(r.pass(false, false, false, 200, 1920));
    for (uint32_t t = 1930; t < 3920; t += 10) CHECK(r.pass(false, false, true, 200, t));
    CHECK(r.pass(false, false, true, 200, 3920) == false);
  }
  // Steering move AND early tries never free the toggle while the wait runs (the purpose of the block).
  {
    Remote r;
    r.pass(true, false, true, 300, 0);
    uint32_t t = 0;
    for (int tries = 0; tries < 5; tries++) {
      for (int k = 0; k < 250; k++) { t += 10; CHECK(r.pass(false, false, true, 300, t)); }   // 2.5 s centred
      for (int k = 0; k < 20; k++)  { t += 10; CHECK(r.pass(false, false, false, 300, t)); }  // 0.2 s push
    }
  }
  // Once served, moving the toggle does not block it again - it is the gear/menu button now. A squeeze does.
  {
    Remote r;
    r.pass(true, false, true, 200, 0);
    CHECK(r.pass(false, false, true, 200, 2000) == false);
    CHECK(r.pass(false, false, false, 200, 2010) == false);   // gear push
    CHECK(r.pass(false, false, false, 200, 9000) == false);
    CHECK(r.pass(true, false, false, 200, 9010) == true);     // squeeze: steering again
    CHECK(r.pass(false, false, true, 200, 9020) == true);
    CHECK(r.pass(false, false, true, 200, 11010) == false);   // 2.0 s after the squeeze pass
  }
  // A remote error frees the toggle at once; after it clears the wait stays served until the next squeeze.
  {
    Remote r;
    r.pass(true, false, true, 200, 0);
    CHECK(r.pass(false, false, true, 200, 500) == true);
    CHECK(r.pass(false, true, false, 200, 510) == false);     // error showing, toggle off centre: free anyway
    CHECK(r.pass(false, false, false, 200, 520) == false);    // error dismissed: still free
    CHECK(r.pass(true, false, true, 200, 600) == true);       // squeeze blocks again
  }
  // The hold wins over an error, exactly as the old if/else order did (the caller already folds the
  // forced-block's !remote_error into hold).
  {
    Remote r;
    CHECK(r.pass(true, true, true, 200, 100) == true);
  }
  // 0 = never free the toggle after a squeeze, however long it rests centred; a remote error still frees it.
  {
    Remote r;
    r.pass(true, false, true, 0, 0);
    for (uint32_t t = 10; t <= 120000; t += 10) { if (!r.pass(false, false, true, 0, t)) { CHECK(false); break; } }
    CHECK(timeToFree(0, 10, 0, 120000) == UINT32_MAX);
    CHECK(r.pass(false, true, true, 0, 120010) == false);
    CHECK(r.pass(false, false, true, 0, 120020) == false);
    OldRemote o;                                               // the old code agrees
    o.pass(true, false, true, 0);
    for (int i = 0; i < 12000; i++) o.pass(false, false, true, 0);
    CHECK(o.blocked);
  }
  // Same verdicts as the old counter at exactly 10 ms passes, pass by pass, over a mixed scripted session
  // (allowing the old code's one-pass lag at the moment of freeing).
  {
    Remote n; OldRemote o;
    uint32_t t = 0, mismatches = 0, lag_ok = 0;
    for (int i = 0; i < 20000; i++) {
      t += 10;
      const int ph = i % 1000;
      const bool hold = ph < 100;                              // 1 s squeeze each 10 s
      const bool centred = !(ph >= 200 && ph < 220) && !(ph >= 700 && ph < 705);   // a steer during the wait, a gear push after
      const bool err = (i / 1000 == 15) && ph >= 150 && ph < 160;                   // one error during one wait
      const bool nb = n.pass(hold, err, centred, 200, t);
      o.pass(hold, err, centred, 200);
      if (nb != o.blocked) { if (!nb && o.blocked) lag_ok++; else mismatches++; }
    }
    CHECK(mismatches == 0);
    CHECK(lag_ok <= 20);                                       // at most one pass per freeing
  }

  // ---- 3. Boot state, roll-over, extremes ----
  {
    Remote r;                                                  // zero-init, not blocked: the toggle is a button at boot
    CHECK(r.pass(false, false, true, 200, 1500) == false);
    CHECK(r.pass(false, false, false, 200, 1510) == false);
    CHECK(r.pass(true, false, true, 200, 1520) == true);       // first squeeze
  }
  {
    Remote r;                                                  // squeeze just before millis() rolls over
    r.pass(true, false, true, 200, 0xFFFFFF00u);
    CHECK(r.pass(false, false, true, 200, 0xFFFFFF00u + 1999u) == true);
    CHECK(r.pass(false, false, true, 200, 0xFFFFFF00u + 2000u) == false);   // wrapped: 0x6D0
  }
  CHECK(timeToFree(5000, 10, 0, 100000) == 50000);            // config maximum = 50 s
  CHECK(timeToFree(65535, 100, 0, 700000) == 655400);         // uint16 maximum: 655.35 s, first 100 ms pass after

  if (fails) { std::printf("%d of %d checks FAILED\n", fails, checks); return 1; }
  std::printf("toggle_block_timer_test: all %d checks passed\n", checks);
  return 0;
}
