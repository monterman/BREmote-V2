// V2.5-Evo - 2026-10-12 - audit 2026-10-11 delta A-2: section 2c - the heading-disagree end fires only while ACTIVE (no
//   end from HOLD) and its edge cannot fire late after RETURN / the RTM yield / IDLE.
// V2.5-Evo - 2026-10-12 - audit 2026-10-11 delta A-1: the clock takes the rider verdict from the caller; new section 2b
//   runs the RX's one rule end to end (still rider with a moving fix counter never pauses or starts the clock; a repeated
//   counter pauses visibly and fires after 30 s; an old v1 remote; a link pull resets on new solutions).
// V2.5-Evo - 2026-10-11 - Host test for Common/FollowMePause.h (audit 2026-10-11 change 1, owner rulings D-1..D-4):
//   pause vs exit classification, the 30 s long-loss clock (start / reset / fire), the heading creep (grant / abort /
//   timeout, one per squeeze), and two sequence models built on Common/AutoReturnRules.h: the sticky cap armed at every
//   Follow-Me exit, and "no Follow-Me exit raises the cap toward a held trigger" (the 10 Oct C-1 surge).
// Build and run:  g++ -std=c++17 -Wall -Wextra -o follow_me_pause_test follow_me_pause_test.cpp && ./follow_me_pause_test
// Ticks are 100 ms like runFmLoop(). Constants are the RX's: clock 30000 ms, reset 1000 ms + 2 fixes, creep 60/255 fading
// by 6 km/h, 3000 ms, start max(D_engage, min_dist + 5), abort min_dist + 1, sticky ceiling 60 over a 1500 ms ramp.

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../../Source/Common/FollowMePause.h"
#include "../../Source/Common/RiderGpsPacket.h"   // V2.5-Evo - 2026-10-12 - A-1: the rider GPS one-rule chain

static const uint32_t kFire = 30000, kReset = 1000;
static const uint8_t  kResetFixes = 2;
static const uint32_t kRiderStale = 3000;   // a typical tx_gps_stale_timeout_ms

// ---- 1. pause vs exit classification ----
static void testClassification()
{
  // Not engaged: nothing to classify, whatever is missing.
  assert(fmEngagedEndClass(false, kFmCondLink, true, true, true) == FMEE_NONE);
  // Engaged, all data present, no steering verdict: the ordinary HOLD (geometry / trigger).
  assert(fmEngagedEndClass(true, 0, true, false, false) == FMEE_HOLD);
  // Every one of conditions 2-7 missing alone is a PAUSE - never an end. Heading included (H-1).
  const uint8_t each[6] = { kFmCondPhaseA, kFmCondPhaseB, kFmCondTxStale, kFmCondRxStale, kFmCondHeading, kFmCondLink };
  for (int i = 0; i < 6; ++i) assert(fmEngagedEndClass(true, each[i], true, false, false) == FMEE_PAUSE);
  // A missing heading that does not count (a creep grant stands) is not a pause by itself.
  assert(fmEngagedEndClass(true, kFmCondHeading, false, false, false) == FMEE_HOLD);
  assert(fmEngagedEndClass(true, kFmCondHeading | kFmCondLink, false, false, false) == FMEE_PAUSE);
  // Divergence or a NEW heading disagreement is a steering end, and outranks a data gap on the same tick (D-3).
  assert(fmEngagedEndClass(true, 0, true, true, false) == FMEE_STEER_END);
  assert(fmEngagedEndClass(true, 0, true, false, true) == FMEE_STEER_END);
  assert(fmEngagedEndClass(true, kFmCondLink, true, true, false) == FMEE_STEER_END);

  // Pause reason: positions and link before the heading, in the documented order.
  assert(fmPauseReasonBit(0) == 0);
  assert(fmPauseReasonBit(kFmCondHeading) == kFmCondHeading);
  assert(fmPauseReasonBit(kFmCondHeading | kFmCondLink) == kFmCondLink);
  assert(fmPauseReasonBit(kFmCondHeading | kFmCondTxStale | kFmCondLink) == kFmCondTxStale);
  assert(fmPauseReasonBit(0x3F) == kFmCondPhaseA);
}

// ---- 2. the long-loss clock ----
// Drives the clock at 100 ms ticks. new_fix = a new rider GPS SOLUTION arrived this tick.
// V2.5-Evo - 2026-10-12 - audit delta A-1: the rider input is the caller's verdict, modelled here the way the RX makes it
// - a new solution within kRiderStale (rx_tx_gps_solution_ms) - and the counter is the solution counter.
struct Sim { FmLongLossState s; uint32_t t; uint32_t seq; uint32_t sol_ms; };
static uint8_t tick(Sim* m, bool running, bool link_ok, bool new_fix, bool buggy_ok)
{
  m->t += 100;
  if (new_fix) { m->seq++; m->sol_ms = m->t; }
  const bool rider_ok = !stampStale(m->t, m->sol_ms, kRiderStale);
  return fmLongLossStep(&m->s, m->t, running, link_ok, rider_ok, m->seq, buggy_ok, kFire, kReset, kResetFixes);
}

static void testLongLoss()
{
  // Healthy: link up, a rider fix every 300 ms, buggy fix good -> never fires (and, with the rider verdict now the
  // caller's own fresh-solution test, never even opens).
  {
    Sim m = {{0, 0, 0, 0, false}, 1000, 7, 0};
    for (int i = 0; i < 1000; ++i) {
      const uint8_t v = tick(&m, true, true, (i % 3) == 0, true);
      assert(v != FMLL_FIRED);
      assert(v == FMLL_OK);
    }
  }
  // Link down for 35 s while armed: counts from the first missing tick, fires exactly at 30 s, once.
  {
    Sim m = {{0, 0, 0, 0, false}, 1000, 7, 0};
    for (int i = 0; i < 20; ++i) tick(&m, true, true, (i % 3) == 0, true);
    int fired_at = -1;
    for (int i = 0; i < 350; ++i) {
      const uint8_t v = tick(&m, true, false, (i % 3) == 0, true);
      if (v == FMLL_FIRED) { assert(fired_at < 0); fired_at = i; }
    }
    assert(fired_at == 300);   // the clock starts on tick 0 and fires at 300 x 100 ms = 30 s of age
  }
  // A 3 s link pull: counts, then resets once all three are good for >= 1 s with >= 2 new rider fixes. No fire.
  {
    Sim m = {{0, 0, 0, 0, false}, 1000, 7, 0};
    for (int i = 0; i < 20; ++i) tick(&m, true, true, (i % 3) == 0, true);
    for (int i = 0; i < 30; ++i) assert(tick(&m, true, false, (i % 3) == 0, true) == FMLL_COUNTING);
    uint8_t v = FMLL_COUNTING; int good = 0;
    while (v == FMLL_COUNTING) { v = tick(&m, true, true, (good % 3) == 0, true); ++good; assert(good < 30); }
    assert(v == FMLL_OK && m.s.since_ms == 0);
    assert(good >= 10);        // never before 1 s of good data
  }
  // Good data for 1 s but only ONE new rider fix: the clock does NOT reset (2 fixes needed), and it still fires at 30 s.
  {
    Sim m = {{0, 0, 0, 0, false}, 1000, 7, 0};
    for (int i = 0; i < 20; ++i) tick(&m, true, true, (i % 3) == 0, true);
    tick(&m, true, false, false, true);                    // clock starts
    int i = 1; bool fired = false;
    for (; i < 400 && !fired; ++i) {
      // the link is fine from here, the buggy fine, but the rider sends one fix at i == 5 and then nothing
      const uint8_t v = tick(&m, true, true, i == 5, true);
      fired = (v == FMLL_FIRED);
    }
    assert(fired && i == 301);
  }
  // The rider's fix is judged by NEW SOLUTIONS: packets that bring none (re-broadcasts) are a loss.
  {
    Sim m = {{0, 0, 0, 0, false}, 1000, 7, 0};
    for (int i = 0; i < 20; ++i) tick(&m, true, true, (i % 3) == 0, true);
    int fired_at = -1;
    for (int i = 0; i < 400 && fired_at < 0; ++i) if (tick(&m, true, true, false, true) == FMLL_FIRED) fired_at = i;
    // the last fix was at most 3 ticks before; stale after 3000 ms more, then 30 s of clock
    assert(fired_at >= 300 && fired_at <= 335);
  }
  // The buggy's own fix missing counts too (OR, not AND).
  {
    Sim m = {{0, 0, 0, 0, false}, 1000, 7, 0};
    for (int i = 0; i < 20; ++i) tick(&m, true, true, (i % 3) == 0, true);
    int fired_at = -1;
    for (int i = 0; i < 400 && fired_at < 0; ++i) if (tick(&m, true, true, (i % 3) == 0, false) == FMLL_FIRED) fired_at = i;
    assert(fired_at == 300);
  }
  // Not running (IDLE / RETURN / yield): never counts, and leaving the running states zeroes a running clock.
  {
    Sim m = {{0, 0, 0, 0, false}, 1000, 7, 0};
    for (int i = 0; i < 500; ++i) assert(tick(&m, false, false, false, false) == FMLL_OFF);
    for (int i = 0; i < 100; ++i) tick(&m, true, false, false, true);
    assert(m.s.since_ms != 0);
    assert(tick(&m, false, false, false, true) == FMLL_OFF && m.s.since_ms == 0);
  }
  // millis() == 0 is not "no clock", and the clock survives the 32-bit wrap.
  {
    FmLongLossState s = {0, 0, 0, 0, false};
    uint32_t t = 0xFFFFFFFFu - 5000u, seq = 1;
    for (int i = 0; i < 20; ++i) { t += 100; if (i % 3 == 0) ++seq; fmLongLossStep(&s, t, true, true, true, seq, true, kFire, kReset, kResetFixes); }
    assert(s.since_ms == 0);
    int fired_at = -1;
    for (int i = 0; i < 400 && fired_at < 0; ++i) {
      t += 100;
      if (fmLongLossStep(&s, t, true, false, true, seq, true, kFire, kReset, kResetFixes) == FMLL_FIRED) fired_at = i;
    }
    assert(fired_at == 300);
  }
}

// ---- 2b. A-1 (V2.5-Evo - 2026-10-12): ONE RULE FOR "THE RIDER'S GPS IS ALIVE", end to end ----
// The RX chain, modelled with the exact functions it calls: a packet -> riderGpsClassify / NoteCounter / Commit
// (Common/RiderGpsPacket.h; Phase A assumed passing - a still rider's 0 m always passes) -> the solution stamp ->
// fmFailingMaskFrom() condition 4 (fmFailingConditionsMaskHdg) -> that ONE bit feeds the pause (fmEngagedEndClass), the
// remote's not-ready look (fm_not_ready_data = !fault_ok) and the long-loss clock (rider_ok). Ticks are 100 ms; the
// remote sends a GPS packet every 3rd or 4th tick (0.3-0.4 s, as in the 10 Oct log); link and buggy fix stay good.
struct OneRule {
  RiderGpsRxState rx; FmLongLossState ll; uint32_t t; uint32_t sol_ms; uint32_t sol_seq; uint8_t ctr;
};
// One 100 ms tick. pkt: a GPS packet arrives this tick; v2: with a counter; advance: the remote's counter moved;
// lat: the rider's latitude (microdegrees). Returns the clock verdict; *mask_out = this tick's conditions 2-7.
static uint8_t oneRuleTick(OneRule* m, bool pkt, bool v2, bool advance, int32_t lat, bool link_ok, uint8_t* mask_out)
{
  m->t += 100;
  if (pkt) {
    if (advance) m->ctr++;
    RiderGpsFields f = {lat, -80234567, v2, v2 ? m->ctr : (uint8_t)0};
    const uint8_t c = riderGpsClassify(&m->rx, &f);
    riderGpsNoteCounter(&m->rx, &f);
    if (c == RGF_NEW) { riderGpsCommit(&m->rx, &f); m->sol_ms = m->t; m->sol_seq++; }
  }
  const uint8_t mask = fmFailingMaskFrom(m->t, false, true, m->sol_ms, kRiderStale, m->t, 6000, true,
                                         link_ok ? m->t : 1u, 500);
  *mask_out = mask;
  const bool rider_ok = (mask & kFmCondTxStale) == 0;
  const bool link     = (mask & kFmCondLink) == 0;
  return fmLongLossStep(&m->ll, m->t, true, link, rider_ok, m->sol_seq, true, kFire, kReset, kResetFixes);
}

static void testRiderGpsOneRule()
{
  // (1) THE 10 OCT CASE, v2 remote: a rider lying still - identical coordinates on every packet - whose GPS keeps
  //     solving (the counter moves). For 120 s: never a pause, never not-ready, the clock never even opens.
  {
    OneRule m = {{false, 0, 0, false, 0}, {0, 0, 0, 0, false}, 1000, 0, 0, 0};
    for (int i = 0; i < 1200; ++i) {
      uint8_t mask = 0;
      const uint8_t v = oneRuleTick(&m, (i % 3) == 0 || (i % 7) == 0, true, true, 26123456, true, &mask);
      if (i > 0) {
        assert(mask == 0);                                                           // condition 4 holds: no pause ...
        assert(fmEngagedEndClass(true, mask, true, false, false) == FMEE_HOLD);     // ... (HOLD = the rider is slow)
        assert(v == FMLL_OK);                                                        // ... and no clock
      }
    }
  }
  // (2) A FROZEN remote GPS: packets keep coming with the SAME counter (re-broadcasts). The pause and the not-ready
  //     look rise on the very tick the clock starts (no invisible countdown), and the clock fires 30 s later.
  {
    OneRule m = {{false, 0, 0, false, 0}, {0, 0, 0, 0, false}, 1000, 0, 0, 0};
    uint8_t mask = 0;
    for (int i = 0; i < 30; ++i) oneRuleTick(&m, (i % 3) == 0, true, true, 26123456, true, &mask);   // healthy
    assert(mask == 0);
    int start = -1, fired = -1;
    for (int i = 0; i < 700 && fired < 0; ++i) {
      const uint8_t v = oneRuleTick(&m, (i % 3) == 0, true, false, 26123456, true, &mask);
      const bool paused = fmEngagedEndClass(true, mask, true, false, false) == FMEE_PAUSE;
      const bool not_ready = (mask & (uint8_t)~kFmCondHeading) != 0;                  // fm_not_ready_data = !fault_ok
      if (v == FMLL_COUNTING || v == FMLL_FIRED) { assert(paused && not_ready); if (start < 0) start = i; }
      else assert(!paused && !not_ready);                                             // never one without the other
      if (v == FMLL_FIRED) fired = i;
    }
    assert(start >= 28 && start <= 33);        // the last new solution aged past the 3 s stale limit
    assert(fired == start + 300);              // then exactly 30 s of clock
  }
  // (3) An OLD (v1) remote and a still rider: the coordinate rule, as before - judged lost after the stale limit - but
  //     now VISIBLY: the pause and the not-ready look stand for the whole 30 s.
  {
    OneRule m = {{false, 0, 0, false, 0}, {0, 0, 0, 0, false}, 1000, 0, 0, 0};
    uint8_t mask = 0; int fired = -1, start = -1;
    for (int i = 0; i < 700 && fired < 0; ++i) {
      const uint8_t v = oneRuleTick(&m, (i % 3) == 0, false, false, 26123456, true, &mask);
      if (v == FMLL_COUNTING || v == FMLL_FIRED) { assert(mask & kFmCondTxStale); if (start < 0) start = i; }
      if (v == FMLL_FIRED) fired = i;
    }
    assert(start > 0 && fired == start + 300);
  }
  // (4) A v2 still rider and a 3 s LINK PULL: the clock opens on the link, and RESETS once the link is back, because
  //     the reset streak counts new SOLUTIONS - it used to count distinct POSITIONS, which a still rider never makes, so
  //     this case fired at 30 s.
  {
    OneRule m = {{false, 0, 0, false, 0}, {0, 0, 0, 0, false}, 1000, 0, 0, 0};
    uint8_t mask = 0;
    for (int i = 0; i < 30; ++i) oneRuleTick(&m, (i % 3) == 0, true, true, 26123456, true, &mask);
    for (int i = 0; i < 30; ++i) assert(oneRuleTick(&m, false, true, true, 26123456, false, &mask) == FMLL_COUNTING);
    bool reset = false;
    for (int i = 0; i < 600; ++i) {
      const uint8_t v = oneRuleTick(&m, (i % 3) == 0, true, true, 26123456, true, &mask);
      assert(v != FMLL_FIRED);
      if (v == FMLL_OK) { reset = true; assert(i >= 9); }   // never before 1 s of good data
    }
    assert(reset);
  }
  // (5) A moving rider on a v2 remote: same as ever - solutions every packet, no pause, no clock.
  {
    OneRule m = {{false, 0, 0, false, 0}, {0, 0, 0, 0, false}, 1000, 0, 0, 0};
    for (int i = 0; i < 600; ++i) {
      uint8_t mask = 0;
      const uint8_t v = oneRuleTick(&m, (i % 3) == 0, true, true, 26123456 + i * 10, true, &mask);
      if (i > 0) assert(mask == 0 && v == FMLL_OK);
    }
  }
}

// ---- 2c. A-2 (V2.5-Evo - 2026-10-12): the heading-disagree end - ACTIVE only, edge sampled on every tick ----
// A model of runFmLoopBody(): sample at the TOP of every tick (latch_top: the latch as the tick starts - it may have
// risen in runRtmLoop()'s heading call just before); an early-returning tick (RETURN, the RTM yield, IDLE) stops there
// - and a RETURN tick's own heading call may still raise the latch afterwards (latch_end); an ordinary tick samples again
// after its heading call (latch_end) and ends the engagement only if the edge stands and the state is FM_ACTIVE.
// Returns true when the tick ENDS the engagement (the D-3 steering end).
static bool hdgTick(bool* prev, bool latch_top, bool latch_end, bool early, bool active)
{
  const bool at_entry = fmHdgDisagreeSample(prev, latch_top);
  if (early) return false;
  const bool edge = fmHdgDisagreeSample(prev, latch_end) || at_entry;
  const bool ends = fmHdgDisagreeEnds(edge, active);
  // ... which is what fmEngagedEndClass() is given (data all present, no divergence):
  assert((fmEngagedEndClass(true, 0, true, false, ends) == FMEE_STEER_END) == ends);
  return ends;
}

static void testHdgDisagreeEnd()
{
  // ACTIVE: a rise between ticks (in the RTM loop's heading call) ends it once; a standing latch never again.
  { bool p = false;
    assert(!hdgTick(&p, false, false, false, true));
    assert(hdgTick(&p, true, true, false, true));
    for (int i = 0; i < 50; ++i) assert(!hdgTick(&p, true, true, false, true)); }
  // ACTIVE: a rise inside this tick's own heading call ends it on this tick.
  { bool p = false;
    assert(hdgTick(&p, false, true, false, true)); }
  // HOLD (A-2): a rise is consumed with NO end - and it does not come back as an edge when Follow-Me resumes ACTIVE.
  { bool p = false;
    assert(!hdgTick(&p, false, false, false, false));
    assert(!hdgTick(&p, true, true, false, false));                     // rose between ticks, in HOLD
    for (int i = 0; i < 50; ++i) assert(!hdgTick(&p, true, true, false, true));   // resumes ACTIVE on COG: no end
    bool q = false;
    assert(!hdgTick(&q, false, true, false, false));                    // rose inside a HOLD tick's heading call
    assert(!hdgTick(&q, true, true, false, true)); }
  // LATE EDGE (A-2): the latch rises during a run of RETURN / yield / IDLE ticks - including inside the LAST RETURN
  // tick's own heading call, after its top sample. The next tick is never ACTIVE at its evaluation (a return exits to
  // ARMED / HOLD / STOPPING / IDLE, the yield to ARMED), so it consumes the rise with no end; every ACTIVE tick after
  // that sees no edge. The old code stored the previous value only on ordinary ticks and ended here.
  { bool p = false;
    for (int i = 0; i < 10; ++i) assert(!hdgTick(&p, false, false, true, false));   // RETURN ticks, latch down
    assert(!hdgTick(&p, false, true, true, false));                                 // rises inside the last one
    assert(!hdgTick(&p, true, true, false, false));                                 // exit tick (HOLD / ARMED)
    for (int i = 0; i < 50; ++i) assert(!hdgTick(&p, true, true, false, true)); }   // ACTIVE again: never fires
  { bool p = false;
    for (int i = 0; i < 5; ++i) assert(!hdgTick(&p, false, false, true, false));    // yield / IDLE
    for (int i = 0; i < 5; ++i) assert(!hdgTick(&p, true, true, true, false));      // rose between yield ticks
    assert(!hdgTick(&p, true, true, false, false));                                 // first ordinary tick (ARMED)
    for (int i = 0; i < 50; ++i) assert(!hdgTick(&p, true, true, false, true)); }
  // ... while a latch that clears and RISES AGAIN during a later ACTIVE run is a new edge and ends that run.
  { bool p = false;
    assert(!hdgTick(&p, true, true, true, false));
    assert(!hdgTick(&p, false, false, false, true));                                // cleared (agreement / ?compasscal)
    assert(hdgTick(&p, true, true, false, true)); }
}

// ---- 3. the heading creep ----
static void testCreep()
{
  // Start distance: max(D_engage, min_dist + 5), raised to D_engage + 5 when the last course points at the rider.
  assert(fmCreepStartDistM(12.0f, 4.0f, false, 0.0f, 0.0f, 5.0f, 30.0f) == 12.0f);
  assert(fmCreepStartDistM(9.5f, 7.0f, false, 0.0f, 0.0f, 5.0f, 30.0f) == 12.0f);
  assert(fmCreepStartDistM(12.0f, 4.0f, true, 350.0f, 10.0f, 5.0f, 30.0f) == 17.0f);   // 20 deg off across north
  assert(fmCreepStartDistM(12.0f, 4.0f, true, 100.0f, 10.0f, 5.0f, 30.0f) == 12.0f);   // 90 deg off: not pointing at him

  // Cap: 60 at a standstill, fading to 0 by 6 km/h, 0 when the speed is unknown.
  assert(fmCreepCap(0.0f, 60, 6.0f) == 60);
  assert(fmCreepCap(3.0f, 60, 6.0f) == 30);
  assert(fmCreepCap(6.0f, 60, 6.0f) == 0);
  assert(fmCreepCap(9.0f, 60, 6.0f) == 0);
  assert(fmCreepCap(-1.0f, 60, 6.0f) == 0);
  for (int v = 0; v <= 200; ++v) assert(fmCreepCap(v / 10.0f, 60, 6.0f) <= 60);

  const float start = 12.0f, abort_m = 5.0f;
  // Grant, then success when a course appears; spent for the squeeze.
  {
    FmCreepState c = {0, false};
    assert(fmCreepStep(&c, 1000, true, false, true, 11.9f, start, abort_m, 3000) == FMCR_NONE);   // too close to start
    assert(fmCreepStep(&c, 1100, true, false, true, 12.0f, start, abort_m, 3000) == FMCR_GRANTED);
    assert(fmCreepStep(&c, 1200, true, false, true, 11.0f, start, abort_m, 3000) == FMCR_GRANTED); // closing is fine above abort
    assert(fmCreepStep(&c, 1300, true, true,  true, 11.0f, start, abort_m, 3000) == FMCR_NONE && c.start_ms == 0);
    // heading lost again in the same squeeze: no second creep
    assert(fmCreepStep(&c, 1400, true, false, true, 20.0f, start, abort_m, 3000) == FMCR_NONE);
    // a release re-arms one creep
    assert(fmCreepStep(&c, 1500, false, false, true, 20.0f, start, abort_m, 3000) == FMCR_NONE && !c.used);
    assert(fmCreepStep(&c, 1600, true, false, true, 20.0f, start, abort_m, 3000) == FMCR_GRANTED);
  }
  // Timeout at 3 s exactly, then no retry until a release.
  {
    FmCreepState c = {0, false};
    uint32_t t = 5000; uint8_t v = fmCreepStep(&c, t, true, false, true, 15.0f, start, abort_m, 3000);
    assert(v == FMCR_GRANTED);
    int n = 0;
    while ((v = fmCreepStep(&c, t += 100, true, false, true, 15.0f, start, abort_m, 3000)) == FMCR_GRANTED) ++n;
    assert(v == FMCR_TIMEOUT && n == 29);
    assert(fmCreepStep(&c, t += 100, true, false, true, 15.0f, start, abort_m, 3000) == FMCR_NONE);
  }
  // Abort at once inside min_dist + 1, on any condition falling (data loss, rider stale, sticky cap...), and
  // a link gap that freezes the trigger byte (thr_held stays true, eligible false) aborts without re-arming.
  {
    FmCreepState c = {0, false};
    assert(fmCreepStep(&c, 100, true, false, true, 15.0f, start, abort_m, 3000) == FMCR_GRANTED);
    assert(fmCreepStep(&c, 200, true, false, true, 4.9f, start, abort_m, 3000) == FMCR_ABORT);
    assert(fmCreepStep(&c, 300, true, false, true, 15.0f, start, abort_m, 3000) == FMCR_NONE);   // spent
    FmCreepState d = {0, false};
    assert(fmCreepStep(&d, 100, true, false, true, 15.0f, start, abort_m, 3000) == FMCR_GRANTED);
    assert(fmCreepStep(&d, 200, true, false, false, 15.0f, start, abort_m, 3000) == FMCR_ABORT);
    assert(d.used);
  }
  // Never granted while not eligible, or with a heading.
  {
    FmCreepState c = {0, false};
    assert(fmCreepStep(&c, 100, true, false, false, 50.0f, start, abort_m, 3000) == FMCR_NONE && !c.used);
    assert(fmCreepStep(&c, 200, true, true,  true,  50.0f, start, abort_m, 3000) == FMCR_NONE && !c.used);
  }
}

// ---- 4. sequence models on the sticky cap (Common/AutoReturnRules.h) ----
// A Follow-Me exit under a held trigger: the cap in force is `cap_before`; the exit arms the sticky cap at
// stickyArmValue(cap_before, 60), THEN Follow-Me's own cap is lifted to 255. The motor sees min(trigger, fm cap, sticky).
// Model the RX's 10 Hz stickyCapUpdate() with no GPS (rider unknown) and the trigger held at 240 for 10 s.
static void testStickyAtEveryExit()
{
  const uint8_t trigger = 240;
  const uint8_t caps[] = { 0, 5, 13, 60, 61, 122, 200, 255 };
  for (uint8_t cap_before : caps) {
    uint8_t sticky = stickyArmValue(cap_before, 60);
    bool rise_open = true;
    assert(sticky <= 60 && sticky <= cap_before);
    const uint8_t fm_cap = 255;   // fmEnterIdle() / ARMED after the arm
    uint8_t prev_out = (cap_before < trigger) ? cap_before : trigger;
    for (int i = 0; i < 100; ++i) {
      sticky = stickyReturnCapStep(sticky, &rise_open, 100, false, -1.0f, 10.0f, 30.0f, 60, 1500);
      uint8_t out = trigger;
      if (fm_cap < out) out = fm_cap;
      if (sticky < out) out = sticky;
      // NO FM EXIT RAISES THE CAP TOWARD THE HELD TRIGGER: never above the 60 ceiling ...
      assert(out <= 60);
      // ... and never more than the clock-only rise per tick (255 counts per 1.5 s = 17 per 100 ms).
      assert(out <= prev_out + 17 || out <= cap_before);
      prev_out = out;
    }
    // With no GPS the cap holds at the ceiling (owner: "if there is no GPS, at least cap it").
    assert(sticky == 60);
  }
  // GPS knows the rider is inside the stop distance: the cap can only go down, to 0.
  {
    uint8_t sticky = stickyArmValue(200, 60); bool rise_open = true;
    for (int i = 0; i < 50; ++i) sticky = stickyReturnCapStep(sticky, &rise_open, 100, true, 8.0f, 10.0f, 30.0f, 60, 1500);
    assert(sticky == 0);
    // and a GPS reading far away later never raises it again
    for (int i = 0; i < 50; ++i) sticky = stickyReturnCapStep(sticky, &rise_open, 100, true, 80.0f, 10.0f, 30.0f, 60, 1500);
    assert(sticky == 0);
  }
}

int main()
{
  testClassification();
  testLongLoss();
  testRiderGpsOneRule();   // V2.5-Evo - 2026-10-12 - A-1
  testHdgDisagreeEnd();    // V2.5-Evo - 2026-10-12 - A-2
  testCreep();
  testStickyAtEveryExit();
  printf("follow_me_pause_test: all tests passed\n");
  return 0;
}
