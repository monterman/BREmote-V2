// V2.5-Evo - 2026-10-29 - delta audit LOW-1 / LOW-2 / LOW-4: the Conn model follows the new checkConnStatus() pass -
//   the end-of-pass count restart (bindLedRefreshPatternSeen()), the link-lost blink skipping its write while a pattern
//   runs, and the refresh's direction (CONFIG0) write, with the pin's direction now modelled (Led::out). New checks:
//   section 0, the restart rule; section 2, a direction lost at boot is repaired in 5 passes (the old code never);
//   section 3, at least 0.8 s dark after every pattern; section 5, with the remote off every pattern is shown intact,
//   the blink keeps the old code's phase, and the LED is lit within 1 s when the link comes back.
// V2.5-Evo - 2026-10-29 - host test for Source/Common/BindLedRefresh.h: the receiver's BIND LED is lit again about
//   once a second while the link is up, and never in the middle of a deliberate BIND LED pattern.
//   The Conn model below is System.ino's checkConnStatus() pass, line for line (the old code is the same model with the
//   refresh switched off). Patterns are modelled as the LED levels the RX functions write, with the pattern flag set for
//   their whole length: blinkBind(n), blinkBindPattern() (FULL, PARTIAL) and blinkErr(n, AP_L_BIND).
//   1. Steady link: the LED is lit on the first fresh pass; then one refresh every 5 passes (1.0 s), each one a lit write.
//   2. A lost LED-on write or a stray HIGH while connected: the old code leaves the LED dark for good; the new code
//      lights it again within 5 passes.
//   3. Every pattern, started at every 10 ms phase of the 200 ms pass grid and at every phase of the refresh count,
//      with the pass running before or after the pattern's own step: the LED shows exactly the pattern for its whole
//      length, and is lit again at most 1.0 s after the pattern ends. The BIND-button calibration sequence end to end.
//   4. Unchanged: the link-lost blink and the unpaired blip are the old code pass for pass; with no lost writes and
//      no patterns the LED level is the old code's on every pass; link recovery restarts the count.
// Build and run (Git Bash, from Tools/tests, PATH and TMP as in SOP-012's host-test note):
//   g++ -std=c++17 -Wall -Wextra -o bind_led_refresh_test bind_led_refresh_test.cpp && ./bind_led_refresh_test

#include <cstdio>
#include <cstdint>
#include <vector>

#include "../../Source/Common/BindLedRefresh.h"

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

// The BIND LED (AP_L_BIND, active LOW): lit = LOW written.
struct Led {
  bool lit = false;    // the level (OUTPUT0 bit 1) is LOW; startupAW() writes it HIGH (dark)
  bool out = true;     // the pin is an OUTPUT (CONFIG0 bit 1 = 0); false = left an input, so nothing shows
  int  writes = 0;     // level (OUTPUT0) writes
  int  cfg_writes = 0; // direction (CONFIG0) writes
  bool shown() const { return out && lit; }
};

// checkConnStatus(), one 200 ms pass, line for line. refresh = false is the old code (f0e5d2d: no refresh, no direction
// write, no count restart, and a link-lost blink that writes through a pattern).
struct Conn {
  bool    refresh = true;
  uint8_t bind_pin_state = 0;
  uint8_t passes = 0;           // the task's local counter
  int     unpairedBlink = 0;
  int     refreshes = 0, skips = 0, blink_skips = 0;
  // lose = the LED write(s) made on this pass are lost on the bus (the library cannot tell).
  void pass(Led &led, bool paired, bool fresh, bool pattern_running, bool lose = false) {
    auto write    = [&](bool lit) { led.writes++; if (!lose) led.lit = lit; };
    auto cfgWrite = [&]() { led.cfg_writes++; if (!lose) led.out = true; };
    if (paired) {
      if (fresh) {
        const bool was_lit = (bind_pin_state == 1);
        if (bind_pin_state != 1) { bind_pin_state = 1; write(true); }
        if (refresh && bindLedRefreshDue(&passes, was_lit)) {
          if (bindLedRefreshMayWrite(pattern_running)) { write(true); refreshes++; }   // hold 1: the level
          else skips++;
          if (bindLedRefreshMayWrite(pattern_running)) cfgWrite();                     // hold 2: the direction (LOW-4)
        }
      } else {
        // LOW-2: the new code skips the blink's write while a pattern runs, and toggles bind_pin_state either way.
        const bool may = !refresh || bindLedRefreshMayWrite(pattern_running);
        if (bind_pin_state) { bind_pin_state = 0; if (may) write(false); else blink_skips++; }
        else                { bind_pin_state = 1; if (may) write(true);  else blink_skips++; }
      }
    } else {
      unpairedBlink++;
      if (unpairedBlink == 4) { unpairedBlink = 0; write(true); write(false); }   // 10 ms blip, ends dark
    }
    if (refresh) bindLedRefreshPatternSeen(&passes, pattern_running);             // LOW-1: end of every pass
  }
};

// A BIND LED pattern: the levels it writes, in order, and how long each is held. Every pattern ends dark.
struct Seg { bool lit; uint32_t ms; };
typedef std::vector<Seg> Pattern;

static Pattern blinkBindP(int n) {                 // blinkBind(n): 50 on / 50 off
  Pattern p;
  for (int i = 0; i < n; i++) { p.push_back({true, 50}); p.push_back({false, 50}); }
  return p;
}
static Pattern blinkBindPatternP(int count, uint32_t on, uint32_t off) {   // 400 ms dark lead-in, then count x on/off
  Pattern p;
  p.push_back({false, 400});
  for (int i = 0; i < count; i++) { p.push_back({true, on}); p.push_back({false, off}); }
  return p;
}
static Pattern blinkErrP(int n) {                  // blinkErr(n, AP_L_BIND): 200 on / 200 off, then 500 dark
  Pattern p;
  for (int i = 0; i < n; i++) { p.push_back({true, 200}); p.push_back({false, 200}); }
  p.push_back({false, 500});
  return p;
}
static uint32_t total(const Pattern &p) { uint32_t t = 0; for (const Seg &s : p) t += s.ms; return t; }
static bool levelAt(const Pattern &p, uint32_t dt) {
  for (const Seg &s : p) { if (dt < s.ms) return s.lit; dt -= s.ms; }
  return false;
}
static bool segStartsAt(const Pattern &p, uint32_t dt, bool *lit) {
  uint32_t t = 0;
  for (const Seg &s : p) { if (dt == t) { *lit = s.lit; return true; } t += s.ms; }
  return false;
}

// Run the link fresh and paired, in 10 ms steps, with one pattern starting at t_start. The pass runs every 200 ms;
// pass_first = the pass runs before the pattern's step in the same 10 ms slot (the other interleaving otherwise).
// Checks the LED equals the pattern at every step inside it, and returns how long after the pattern's end the LED was
// dark (0 = lit at once), or UINT32_MAX if it never came back within 5 s.
static uint32_t runPattern(const Pattern &p, uint32_t t_start, bool pass_first, bool *inside_ok) {
  Led led; Conn c;
  const uint32_t len = total(p), t_end = t_start + len;
  *inside_ok = true;
  uint32_t dark_after = UINT32_MAX;
  for (uint32_t t = 0; t <= t_end + 5000; t += 10) {
    const bool running = (t >= t_start && t < t_end);
    // The flag as the pass sees it. Pass first in the slot: the pattern has not yet set the flag at its start, and
    // has not yet cleared it at its end. Pass second: the pattern's step for this slot has already happened.
    const bool flag_seen = pass_first ? (t > t_start && t <= t_end) : running;
    auto patternStep = [&]() {
      bool lit;
      if (running && segStartsAt(p, t - t_start, &lit)) { led.writes++; led.lit = lit; }
    };
    auto passStep = [&]() { if (t % 200 == 0) c.pass(led, true, true, flag_seen); };
    if (pass_first) { passStep(); patternStep(); } else { patternStep(); passStep(); }
    if (running && led.lit != levelAt(p, t - t_start)) *inside_ok = false;
    if (t >= t_end && led.lit && dark_after == UINT32_MAX) dark_after = t - t_end;
  }
  return dark_after;
}

int main() {
  // ---- 0. The rule itself ----
  {
    uint8_t n = 0;
    CHECK(bindLedRefreshDue(&n, false) == false && n == 0);      // the LED-on pass restarts the count
    CHECK(bindLedRefreshDue(&n, true) == false && n == 1);
    CHECK(bindLedRefreshDue(&n, true) == false && n == 2);
    CHECK(bindLedRefreshDue(&n, true) == false && n == 3);
    CHECK(bindLedRefreshDue(&n, true) == false && n == 4);
    CHECK(bindLedRefreshDue(&n, true) == true  && n == 0);      // 5th lit pass: due
    n = 3;
    CHECK(bindLedRefreshDue(&n, false) == false && n == 0);      // link came up again: restart
    n = 200;                                                     // cannot happen; treated as due, then every 5
    CHECK(bindLedRefreshDue(&n, true) == true && n == 0);
    CHECK(bindLedRefreshMayWrite(false) == true);
    CHECK(bindLedRefreshMayWrite(true) == false);
    CHECK(kBindLedRefreshPasses == 5);
    n = 3;                                                       // LOW-1: the end-of-pass restart
    bindLedRefreshPatternSeen(&n, false); CHECK(n == 3);         // no pattern: the count is left alone
    bindLedRefreshPatternSeen(&n, true);  CHECK(n == 0);         // a pattern seen: restart
    n = 4;                                                       // due pass that sees the pattern: due resets to 0,
    CHECK(bindLedRefreshDue(&n, true) == true && n == 0);        // the write is skipped (MayWrite false), and the
    bindLedRefreshPatternSeen(&n, true);  CHECK(n == 0);         // pass still ends at 0
    n = 0;                                                       // order matters: Due THEN PatternSeen ends at 0;
    CHECK(bindLedRefreshDue(&n, true) == false && n == 1);       // the other order would end the pass at 1
    bindLedRefreshPatternSeen(&n, true);  CHECK(n == 0);
  }

  // ---- 1. Steady link: lit on the first fresh pass, then a refresh every 5 passes ----
  {
    Led led; Conn c;
    c.pass(led, true, true, false);
    CHECK(led.lit && led.writes == 1);
    for (int i = 1; i <= 300; i++) {
      const int before = led.writes;
      c.pass(led, true, true, false);
      CHECK(led.lit);
      CHECK((led.writes - before) == ((i % 5 == 0) ? 1 : 0));   // passes 5, 10, 15 ... after the LED-on pass
    }
    CHECK(c.refreshes == 60);                                    // one a second for a minute
    CHECK(led.writes == 61);
    CHECK(led.cfg_writes == 60);                                 // LOW-4: each refresh writes the direction once too
  }

  // ---- 2. A lost LED-on write, and a stray HIGH while connected ----
  {
    Led led_old, led_new; Conn o, n; o.refresh = false;
    o.pass(led_old, true, true, false, true);                    // the one LED-on write is lost
    n.pass(led_new, true, true, false, true);
    CHECK(!led_old.lit && !led_new.lit);
    int lit_at = -1;
    for (int i = 1; i <= 300; i++) {
      o.pass(led_old, true, true, false);
      n.pass(led_new, true, true, false);
      CHECK(!led_old.lit);                                       // old code: dark for the whole minute
      if (led_new.lit && lit_at < 0) lit_at = i;
    }
    CHECK(lit_at == 5);                                          // new code: lit 1.0 s later
  }
  {
    Led led; Conn c;
    for (int i = 0; i < 7; i++) c.pass(led, true, true, false);
    led.lit = false;                                             // something drove the pin HIGH, no pattern flag
    int dark = 0;
    while (!led.lit && dark < 100) { c.pass(led, true, true, false); dark++; }
    CHECK(dark >= 1 && dark <= 5);
  }
  // LOW-4: the pin left an INPUT at boot (a failed CONFIG0 read inside a later pinMode() wrote pin 1 back as an input).
  // No level write can light it. The old code never does; the new refresh writes the direction too, on pass 5.
  {
    Led led_old, led_new; Conn o, n; o.refresh = false;
    led_old.out = false; led_new.out = false;
    int shown_at = -1;
    for (int i = 0; i < 300; i++) {
      o.pass(led_old, true, true, false);
      n.pass(led_new, true, true, false);
      CHECK(!led_old.shown());                                   // old code: dark for the whole minute
      if (led_new.shown() && shown_at < 0) shown_at = i;
    }
    CHECK(shown_at == 5);                                        // pass 0 lights the level only; pass 5 is the refresh
    CHECK(led_new.shown());
  }
  // LOW-4: the direction lost later while connected (one refresh's writes lost as well): repaired by the next refresh.
  {
    Led led; Conn c;
    for (int i = 0; i < 5; i++) c.pass(led, true, true, false);
    led.out = false;
    c.pass(led, true, true, false, true);                        // pass 5: the refresh's two writes are lost
    CHECK(!led.shown());
    int k = 0;
    while (!led.shown() && k < 20) { c.pass(led, true, true, false); k++; }
    CHECK(k == 5);                                               // the next refresh, 1.0 s later
  }

  // ---- 3. Patterns are never overwritten, and the LED is lit again 0.8-1.0 s after them ----
  // The pattern list is shared with section 5.
  struct Named { const char *name; Pattern p; };
  const Named pats[] = {
    {"blinkBind(5) cal start", blinkBindP(5)},
    {"blinkBind(10) REJECTED", blinkBindP(10)},
    {"blinkBind(2) heartbeat", blinkBindP(2)},
    {"FULL",                   blinkBindPatternP(2, 70, 130)},
    {"PARTIAL",                blinkBindPatternP(3, 400, 300)},
    {"blinkErr(2, BIND)",      blinkErrP(2)},
    {"blinkErr(3, BIND)",      blinkErrP(3)},
  };
  {
    for (const Named &np : pats) {
      bool all_inside = true;
      uint32_t worst = 0, best = UINT32_MAX;
      // Start after the link has been up a while, at every 10 ms offset across one full refresh period (1 s),
      // which covers every phase of both the 200 ms pass grid and the 5-pass count.
      for (uint32_t off = 0; off < 1000; off += 10) {
        for (int order = 0; order < 2; order++) {
          bool inside_ok = false;
          const uint32_t d = runPattern(np.p, 2000 + off, order == 0, &inside_ok);
          if (!inside_ok) all_inside = false;
          if (d > worst) worst = d;
          if (d < best) best = d;
        }
      }
      if (!all_inside) std::printf("  pattern %s was drawn over\n", np.name);
      if (worst > 1000) std::printf("  pattern %s: dark %u ms after it ended\n", np.name, (unsigned)worst);
      // LOW-1: the shortest dark tail too. Before the count restart it could be 0 ms (lit on the very next pass), and
      // a relight within a pattern's own gap length reads as one more flash.
      if (best < 800) std::printf("  pattern %s: lit again only %u ms after it ended\n", np.name, (unsigned)best);
      CHECK(all_inside);
      CHECK(worst <= 1000);
      CHECK(best >= 800);
    }
  }
  // The BIND-button calibration while connected: blinkBind(5), the 45 s collection (no pattern: the task drives the LED,
  // as the original design note says), then FULL. Lit during the collection within 1 s; FULL intact; lit after it.
  {
    Led led; Conn c;
    const Pattern start = blinkBindP(5), full = blinkBindPatternP(2, 70, 130);
    const uint32_t t0 = 3070, t1 = t0 + total(start), t2 = t1 + 45000, t3 = t2 + total(full);
    bool start_ok = true, full_ok = true, lit_in_collection = true, lit_after = false;
    bool start_tail_dark = true, full_tail_dark = true;          // LOW-1: dark for 0.8 s after each pattern
    for (uint32_t t = 0; t <= t3 + 1200; t += 10) {
      const bool in_start = (t >= t0 && t < t1), in_full = (t >= t2 && t < t3);
      bool lit;
      if (in_start && segStartsAt(start, t - t0, &lit)) led.lit = lit;
      if (in_full && segStartsAt(full, t - t2, &lit)) led.lit = lit;
      if (t % 200 == 0) c.pass(led, true, true, in_start || in_full);
      if (in_start && led.lit != levelAt(start, t - t0)) start_ok = false;
      if (in_full && led.lit != levelAt(full, t - t2)) full_ok = false;
      if (t >= t1 + 1000 && t < t2 && !led.lit) lit_in_collection = false;
      if (t >= t3 + 1000 && led.lit) lit_after = true;
      if (t >= t1 && t < t1 + 800 && led.lit) start_tail_dark = false;
      if (t >= t3 && t < t3 + 800 && led.lit) full_tail_dark = false;
    }
    CHECK(start_ok);
    CHECK(full_ok);
    CHECK(lit_in_collection);
    CHECK(lit_after);
    CHECK(start_tail_dark);
    CHECK(full_tail_dark);
  }

  // ---- 4. Unchanged behaviour ----
  // Mixed session, no lost writes and no patterns: the LED level is the old code's on every pass, and every extra
  // write the new code makes is a refresh (a lit write on a fresh pass with the LED already lit).
  {
    Led lo, ln; Conn o, n; o.refresh = false;
    uint32_t seed = 12345u, level_mismatch = 0, extra_not_refresh = 0;
    bool fresh = true;
    for (int i = 0; i < 20000; i++) {
      seed = seed * 1103515245u + 12345u;
      if ((seed >> 16) % 23 == 0) fresh = !fresh;               // link drops and returns at random
      const bool paired = !(i >= 9000 && i < 9500);             // a stretch unpaired
      const int wo = lo.writes, wn = ln.writes, rn = n.refreshes;
      o.pass(lo, paired, fresh, false);
      n.pass(ln, paired, fresh, false);
      if (lo.lit != ln.lit) level_mismatch++;
      if ((ln.writes - wn) - (lo.writes - wo) != (n.refreshes - rn)) extra_not_refresh++;
      if (o.bind_pin_state != n.bind_pin_state) level_mismatch++;
    }
    CHECK(level_mismatch == 0);
    CHECK(extra_not_refresh == 0);
    CHECK(n.refreshes > 0);
  }
  // Link lost: the blink is the old code's, pass for pass, and no refresh is made.
  {
    Led lo, ln; Conn o, n; o.refresh = false;
    for (int i = 0; i < 12; i++) { o.pass(lo, true, true, false); n.pass(ln, true, true, false); }
    const int r = n.refreshes;
    bool same = true;
    for (int i = 0; i < 100; i++) {
      o.pass(lo, true, false, false); n.pass(ln, true, false, false);
      if (lo.lit != ln.lit || lo.writes != ln.writes - r) same = false;
    }
    CHECK(same);
    CHECK(n.refreshes == r);
  }
  // Unpaired: the blip is the old code's, and no refresh is ever made.
  {
    Led lo, ln; Conn o, n; o.refresh = false;
    bool same = true;
    for (int i = 0; i < 400; i++) {
      o.pass(lo, false, (i % 3) == 0, false); n.pass(ln, false, (i % 3) == 0, false);
      if (lo.lit != ln.lit || lo.writes != ln.writes) same = false;
    }
    CHECK(same);
    CHECK(n.refreshes == 0 && n.skips == 0);
  }
  // Link recovery on the dark half of the blink: the LED-on write runs and the count restarts from it.
  {
    Led led; Conn c;
    for (int i = 0; i < 8; i++) c.pass(led, true, true, false);
    c.pass(led, true, false, false);                             // stale: bind_pin_state 1 -> 0, dark
    CHECK(!led.lit && c.bind_pin_state == 0);
    const int w = led.writes;
    c.pass(led, true, true, false);                              // fresh again: LED-on write
    CHECK(led.lit && led.writes == w + 1 && c.passes == 0);
    for (int i = 1; i < 5; i++) { c.pass(led, true, true, false); CHECK(led.writes == w + 1); }
    c.pass(led, true, true, false);
    CHECK(led.writes == w + 2);                                  // first refresh 5 passes after it
  }
  // Link recovery on the lit half of the blink: no LED-on write (already lit); the next refresh within 5 passes.
  {
    Led led; Conn c;
    for (int i = 0; i < 8; i++) c.pass(led, true, true, false);
    c.pass(led, true, false, false);                             // 1 -> 0, dark
    c.pass(led, true, false, false);                             // 0 -> 1, lit
    CHECK(led.lit && c.bind_pin_state == 1);
    int k = 0;
    const int w = led.writes;
    while (led.writes == w && k < 10) { c.pass(led, true, true, false); k++; }
    CHECK(k >= 1 && k <= 5);
    CHECK(led.lit);
  }

  // ---- 5. The remote off: the link-lost blink no longer draws over a pattern (LOW-2) ----
  // A BIND-button calibration with the remote off or out of range: the link is stale, so the blink writes on every pass.
  // Every pattern, started at every 10 ms offset across two blink periods (both blink phases and every pass-grid
  // phase), with the pass before or after the pattern's step in its slot. The old code runs alongside on the same flag.
  //   A. The remote off throughout. The new code shows exactly the pattern; bind_pin_state is the old code's on every
  //      pass (the blink keeps its phase); from the first pass after the pattern the LED is the old code's again (the
  //      blink carries on); and the old code chops every pattern at some phase (the case is real: this test bites).
  //   B. The remote comes back as the pattern ends. The LED is lit at most 1 s later and stays lit - also when the
  //      blink's skipped LOW write left it dark with bind_pin_state at 1, so no LED-on write is made.
  for (const Named &np : pats) {
    const Pattern &p = np.p;
    const uint32_t len = total(p);
    bool inside_ok = true, phase_same = true, resumes_same = true, old_chopped = false, back_ok = true;
    int  blink_skips = 0;
    for (uint32_t off = 0; off < 400; off += 10) {
      for (int order = 0; order < 2; order++) {
        const bool pass_first = (order == 0);
        const uint32_t t_start = 2000 + off, t_end = t_start + len;
        for (int run = 0; run < 2; run++) {                      // 0 = A, 1 = B
          Led ln, lo; Conn n, o; o.refresh = false;
          for (uint32_t t = 0; t <= t_end + 2000; t += 10) {
            const bool running   = (t >= t_start && t < t_end);
            const bool flag_seen = pass_first ? (t > t_start && t <= t_end) : running;   // as in runPattern()
            const bool fresh     = (t < 1000) || (run == 1 && t >= t_end);               // link up, lost at 1 s
            auto patternStep = [&]() {
              bool lit;
              if (running && segStartsAt(p, t - t_start, &lit)) { ln.writes++; ln.lit = lit; lo.writes++; lo.lit = lit; }
            };
            auto passStep = [&]() {
              if (t % 200 != 0) return;
              n.pass(ln, true, fresh, flag_seen);
              o.pass(lo, true, fresh, flag_seen);
              if (run == 0 && n.bind_pin_state != o.bind_pin_state) phase_same = false;
            };
            if (pass_first) { passStep(); patternStep(); } else { patternStep(); passStep(); }
            if (running) {
              const bool want = levelAt(p, t - t_start);
              if (ln.lit != want) inside_ok = false;
              if (run == 0 && lo.lit != want) old_chopped = true;
            }
            if (run == 0 && t >= t_end + 200 && ln.lit != lo.lit) resumes_same = false;
            if (run == 1 && t >= t_end + 1000 && !ln.shown()) back_ok = false;
          }
          blink_skips += n.blink_skips;
        }
      }
    }
    if (!inside_ok)    std::printf("  remote off: pattern %s was drawn over by the link-lost blink\n", np.name);
    if (!phase_same)   std::printf("  remote off: pattern %s moved the blink's phase\n", np.name);
    if (!resumes_same) std::printf("  remote off: the blink did not carry on after pattern %s\n", np.name);
    if (!back_ok)      std::printf("  remote back: LED not lit within 1 s after pattern %s\n", np.name);
    CHECK(inside_ok);
    CHECK(phase_same);
    CHECK(resumes_same);
    CHECK(old_chopped);
    CHECK(back_ok);
    CHECK(blink_skips > 0);
  }

  if (fails) { std::printf("%d of %d checks FAILED\n", fails, checks); return 1; }
  std::printf("bind_led_refresh_test: all %d checks passed\n", checks);
  return 0;
}
