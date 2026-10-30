// V2.5-Evo - 2026-10-12 - audit 2026-10-11 delta A-3: section 5 - the 30 s end from a HOLD under a held trigger arms the
//   sticky cap with its rise closed (0 until one release); from ACTIVE unchanged.
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

// ---- 5. A-3 (V2.5-Evo - 2026-10-12, owner ruling (a)): the 30 s end from a HOLD under a held trigger ----
// The RX arms the sticky cap with stickyCapArmRise(cap, fmLongLossStickyRiseOpen(from_hold, thr_held)) and steps it with
// stickyReturnCapStep() at 10 Hz (stickyCapUpdate()). The motor sees min(trigger, sticky) until calcPWM() clears it on
// one release below 25 counts.
static void testLongLossStickyFromHold()
{
  const uint8_t trigger = 240;
  // The rule itself: closed only from HOLD with the trigger held.
  assert(!fmLongLossStickyRiseOpen(true, true));
  assert(fmLongLossStickyRiseOpen(true, false));    // released this very tick: calcPWM() clears the cap on its next pass
  assert(fmLongLossStickyRiseOpen(false, true));    // from ACTIVE: as before
  assert(fmLongLossStickyRiseOpen(false, false));
  // FROM HOLD, TRIGGER HELD: HOLD's cap is 0, the rise starts CLOSED -> the buggy stays at 0 for as long as the trigger
  // is held - 10 s with no GPS, and 10 s with GPS putting the rider far away (the distance term can only lower).
  for (int gps = 0; gps < 2; ++gps) {
    uint8_t sticky = stickyArmValue(0, 60);
    bool rise_open = fmLongLossStickyRiseOpen(true, true);
    for (int i = 0; i < 100; ++i) {
      sticky = stickyReturnCapStep(sticky, &rise_open, 100, gps == 1, 80.0f, 10.0f, 30.0f, 60, 1500);
      const uint8_t out = (sticky < trigger) ? sticky : trigger;
      assert(out == 0 && !rise_open);
    }
  }
  // (One release below 25 counts then clears it in calcPWM() - PWM.ino, unchanged and not modelled here.)
  // FROM ACTIVE (the cap in force, e.g. 200): unchanged - armed at the 60 ceiling, the rise open, holds 60 with no GPS.
  {
    uint8_t sticky = stickyArmValue(200, 60);
    bool rise_open = fmLongLossStickyRiseOpen(false, true);
    for (int i = 0; i < 100; ++i) sticky = stickyReturnCapStep(sticky, &rise_open, 100, false, -1.0f, 10.0f, 30.0f, 60, 1500);
    assert(sticky == 60);
  }
  // FROM ACTIVE at a low cap (e.g. 0 in an engage ramp's first tick): the rise opens and climbs to 60 as before.
  {
    uint8_t sticky = stickyArmValue(0, 60);
    bool rise_open = fmLongLossStickyRiseOpen(false, true);
    int ticks = 0;
    while (sticky < 60 && ticks < 50) { sticky = stickyReturnCapStep(sticky, &rise_open, 100, false, -1.0f, 10.0f, 30.0f, 60, 1500); ++ticks; }
    assert(sticky == 60 && ticks == 4);   // 0 -> 17 -> 34 -> 51 -> 60
  }
}

// ---- 6. V2.5-Evo - 2026-10-30 - audit 2026-10-20 R2 (finding F-1): the divergence dwell ----
// fmDivergeDwellStep() at the owner's numbers: D_engage 12 m, so the ceiling is 24 m (79 ft) and the band's far edge 36 m
// (118 ft); dwell 3000 ms; 2 m either way. Ticks are 100 ms; the rider's fix (and so the distance) moves at 3 Hz.
// THE NOISE: up to +/-3 ft (0.914 m) on each distance sample, uniform, held between fixes - the audit's "6 ft" as a band
// (its pacing case, 82-115 ft inside the 79-118 ft band, is exactly that band shrunk by 3 ft each side).
static const float    kDvCeil = 24.0f, kDvFar = 36.0f, kDvEps = 2.0f, kDvNoise = 0.9144f;
static const uint32_t kDvDwell = 3000;
static float dvU(uint32_t *s) { *s = *s * 1103515245u + 12345u; return (float)((*s >> 8) & 0xFFFFu) / 65535.0f; }

// The pre-R2 rule, restated verbatim from runFmLoopBody() (2026-10-29): park / dwell / reset, "must have closed 2 m".
static bool oldDivergeStep(uint32_t *since, float *start, uint32_t now, bool parked, bool active, float dist, float ceil_m)
{
  if (parked) { *since = 0; *start = -1.0f; return false; }
  if (active && dist > ceil_m) {
    if (*since == 0) { *since = now; *start = dist; return false; }
    if ((uint32_t)(now - *since) >= kDvDwell) {
      if (dist >= (*start - kDvEps)) return true;
      *since = 0; *start = -1.0f;
    }
    return false;
  }
  *since = 0; *start = -1.0f;
  return false;
}

// Runs a distance trajectory through the dwell. dist_at(t_s) gives the TRUE distance; noise (if on) is added per fix.
// judge_from_s: the grace / pivot window - the dwell is parked before it. Returns the verdict and when / where it fired.
struct DvRun { uint8_t verdict; float t_s; float true_m; float read_m; };
template <typename F>
static DvRun dvRun(F dist_at, float judge_from_s, float max_s, bool noise, uint32_t seed)
{
  FmDivergeDwell d = { 0, -1.0f };
  uint32_t rs = seed;
  float n = 0.0f;
  long fix_prev = -1;
  for (int i = 1; i <= (int)(max_s * 10.0f); ++i) {
    const float t = i * 0.1f;
    const long fix = (long)floorf(t * 3.0f + 1e-4f);
    if (fix != fix_prev) { fix_prev = fix; n = noise ? (dvU(&rs) * 2.0f - 1.0f) * kDvNoise : 0.0f; }
    const float truth = dist_at(t);
    const float read  = truth + n;
    const bool judged = (t >= judge_from_s);
    const uint8_t v = fmDivergeDwellStep(&d, (uint32_t)(i * 100), judged, read, kDvCeil, kDvFar, kDvDwell, kDvEps, kDvEps);
    if (v != FMDV_NONE) return { v, t, truth, read };
  }
  return { FMDV_NONE, -1.0f, -1.0f, -1.0f };
}

static void testDivergeDwell()
{
  // (0) With the band empty (far edge not above the ceiling - a front station that raised the ceiling) it is the old rule
  //     to the bit: random sequences of parking, state and distance, verdicts and state compared on every tick.
  {
    uint32_t rs = 4711u;
    long verdicts = 0;
    for (int run = 0; run < 2000; ++run) {
      FmDivergeDwell d = { 0, -1.0f };
      uint32_t o_since = 0; float o_start = -1.0f;
      float dist = 10.0f + dvU(&rs) * 40.0f;
      for (int i = 1; i <= 400; ++i) {
        dist += (dvU(&rs) - 0.5f) * 3.0f;
        const bool parked = dvU(&rs) < 0.05f, active = dvU(&rs) < 0.97f;
        const bool o = oldDivergeStep(&o_since, &o_start, (uint32_t)(i * 100), parked, active, dist, kDvCeil);
        const uint8_t v = fmDivergeDwellStep(&d, (uint32_t)(i * 100), !parked && active, dist, kDvCeil, kDvCeil,
                                             kDvDwell, kDvEps, kDvEps);
        assert(o == (v != FMDV_NONE));
        assert(v != FMDV_OPENING);                       // no band, no in-band verdict
        assert(o_since == d.since_ms && o_start == d.start_m);
        if (o) { verdicts++; o_since = 0; o_start = -1.0f; d.since_ms = 0; d.start_m = -1.0f; }   // the RX's steering end
      }
    }
    printf("R2 (0) empty band = the old rule: 2000 random runs identical (%ld verdicts)\n", verdicts);
  }
  // Unit: the band and the far edge.
  {
    FmDivergeDwell d = { 0, -1.0f };
    assert(fmDivergeDwellStep(&d, 100, true, 25.0f, kDvCeil, kDvFar, kDvDwell, kDvEps, kDvEps) == FMDV_NONE);  // starts
    assert(d.since_ms == 100 && d.start_m == 25.0f);
    assert(fmDivergeDwellStep(&d, 3000, true, 25.0f, kDvCeil, kDvFar, kDvDwell, kDvEps, kDvEps) == FMDV_NONE);  // 2.9 s
    assert(fmDivergeDwellStep(&d, 3100, true, 25.0f, kDvCeil, kDvFar, kDvDwell, kDvEps, kDvEps) == FMDV_NONE);  // pacing
    assert(d.since_ms == 0 && d.start_m == -1.0f);                                                            // fresh
    d = { 100, 25.0f };
    assert(fmDivergeDwellStep(&d, 3100, true, 27.0f, kDvCeil, kDvFar, kDvDwell, kDvEps, kDvEps) == FMDV_NONE);  // +2.0: not more
    d = { 100, 25.0f };
    assert(fmDivergeDwellStep(&d, 3100, true, 27.1f, kDvCeil, kDvFar, kDvDwell, kDvEps, kDvEps) == FMDV_OPENING);
    assert(d.since_ms == 100);                                                            // left standing for the RX
    d = { 100, 37.0f };
    assert(fmDivergeDwellStep(&d, 3100, true, 36.5f, kDvCeil, kDvFar, kDvDwell, kDvEps, kDvEps) == FMDV_NOT_CLOSING);  // beyond 36
    d = { 100, 39.0f };
    assert(fmDivergeDwellStep(&d, 3100, true, 36.9f, kDvCeil, kDvFar, kDvDwell, kDvEps, kDvEps) == FMDV_NONE);  // closed 2.1
    d = { 100, 30.0f };
    assert(fmDivergeDwellStep(&d, 3100, false, 40.0f, kDvCeil, kDvFar, kDvDwell, kDvEps, kDvEps) == FMDV_NONE);  // parked
    assert(d.since_ms == 0);
    d = { 100, 30.0f };
    assert(fmDivergeDwellStep(&d, 3100, true, 23.9f, kDvCeil, kDvFar, kDvDwell, kDvEps, kDvEps) == FMDV_NONE);  // inside
    assert(d.since_ms == 0);
  }
  // (a) WRONG STEERING. An inverted-steering buggy turns until it points away from its aim and drives off; the rider keeps
  //     going. Modelled in 2-D: rider 9.8 mph north; the buggy turns the WRONG way at up to 45 deg/s and runs at 5 mph
  //     (the align cap). Two cases, with and without the noise band:
  //     (a1) inverted from the engagement, the buggy 20 m back: judging opens at the grace end (6.5 s) or a later pivot
  //          window end (8 / 10 / 12 s) - by then it is past 36 m, so it is the old rule that ends it;
  //     (a2) a fault MID-RUN, inside the band: following correctly at 25 m (82 ft) until 10 s, then inverted. The RX's
  //          pivot net parks the judgement while the heading error rises from 55 deg (entry) until it has worsened 15 deg
  //          past its best (PV-2), then a fresh window opens - and the R2 band rule must end it.
  //     Either way it must end within 3 s (+ a tick) of judging opening with the buggy beyond the ceiling.
  {
    const float vr = 9.8f * 0.44704f, vb = 5.0f * 0.44704f;
    float worst = 0.0f;
    int in_band_ends = 0, runs_a = 0;
    for (int scen = 0; scen < 5; ++scen) {                       // 0-3: (a1) at the four openings; 4: (a2)
      const float opens_a1[4] = { 6.5f, 8.0f, 10.0f, 12.0f };
      static float traj[1201], errs[1201];
      float rx = 0.0f, ry = 0.0f, bx = 0.0f, by = (scen < 4) ? -20.0f : -25.0f, bh = 0.0f;
      const float t_fault = (scen < 4) ? 0.0f : 10.0f;
      traj[0] = ry - by; errs[0] = 0.0f;
      for (int i = 1; i <= 1200; ++i) {
        const float t = i * 0.1f;
        ry += vr * 0.1f;
        const float want = atan2f(rx - bx, ry - by) * 57.2957795f;
        float err = want - bh;
        while (err > 180.0f) err -= 360.0f;
        while (err < -180.0f) err += 360.0f;
        float turn = err;                                        // correct steering ...
        float v = vr;                                            // ... pacing the rider
        if (t >= t_fault) {
          turn = -err;                                           // inverted
          if (fabsf(err) < 1.0f) turn = 4.5f;                    // a dead-ahead start still diverges
          v = vb;
        }
        if (turn >  4.5f) turn =  4.5f;
        if (turn < -4.5f) turn = -4.5f;
        bh += turn;
        bx += v * 0.1f * sinf(bh / 57.2957795f);
        by += v * 0.1f * cosf(bh / 57.2957795f);
        traj[i] = sqrtf((rx - bx) * (rx - bx) + (ry - by) * (ry - by));
        errs[i] = fabsf(err);
      }
      // When judging opens: (a1) the given opening; (a2) the end of the pivot park (error from 55 up to best + 15 = 70).
      float open_t = 0.0f, park_from = -1.0f;
      if (scen < 4) {
        open_t = opens_a1[scen];
      } else {
        for (int i = 0; i <= 1200; ++i) if (i * 0.1f >= t_fault && errs[i] > 55.0f) { park_from = i * 0.1f; break; }
        for (int i = 0; i <= 1200; ++i) if (park_from >= 0.0f && i * 0.1f >= park_from && errs[i] > 70.0f) { open_t = i * 0.1f; break; }
        assert(park_from > 0.0f && open_t > park_from);
      }
      for (int nz = 0; nz < 2; ++nz) {
        for (uint32_t seed = 1; seed <= (nz ? 50u : 1u); ++seed) {
          FmDivergeDwell d = { 0, -1.0f };
          uint32_t rs = seed;
          float n = 0.0f;
          long fix_prev = -1;
          DvRun r = { FMDV_NONE, -1.0f, -1.0f, -1.0f };
          float first_beyond_judged = -1.0f;
          for (int i = 1; i <= 1200; ++i) {
            const float t = i * 0.1f;
            const long fix = (long)floorf(t * 3.0f + 1e-4f);
            if (fix != fix_prev) { fix_prev = fix; n = nz ? (dvU(&rs) * 2.0f - 1.0f) * kDvNoise : 0.0f; }
            const float read = traj[i] + n;
            const bool parked = (scen < 4) ? (t < open_t) : (park_from >= 0.0f && t >= park_from && t < open_t);
            if (!parked && t >= open_t && read > kDvCeil && first_beyond_judged < 0.0f) first_beyond_judged = t;
            const uint8_t v = fmDivergeDwellStep(&d, (uint32_t)(i * 100), !parked, read, kDvCeil, kDvFar, kDvDwell,
                                                 kDvEps, kDvEps);
            if (v != FMDV_NONE && (scen < 4 || t >= t_fault)) { r = { v, t, traj[i], read }; break; }
            assert(v == FMDV_NONE);                              // (a2): pacing before the fault never ends
          }
          assert(r.verdict != FMDV_NONE && first_beyond_judged >= 0.0f);
          const float lat = r.t_s - first_beyond_judged;
          if (lat > worst) worst = lat;
          assert(lat <= 3.0f + 0.1f + 1e-3f);
          if (scen == 4) { runs_a++; if (r.verdict == FMDV_OPENING) in_band_ends++; }
        }
      }
    }
    printf("R2 (a) inverted steering: ends at most %.1f s after judging opens beyond the ceiling (<= 3 s + a tick asked); "
           "mid-run fault from 82 ft: %d of %d runs ended by the band rule, the rest by the old rule (the gap passes "
           "118 ft inside the first dwell)\n", (double)worst, in_band_ends, runs_a);
  }
  // (b) PACING at a constant 82-115 ft with the 6 ft noise band at 3 Hz: no end in 120 s - 200 seeds per gap, plus the
  //     adversarial worst case (every dwell starts at -3 ft and ends at +3 ft: 1.83 m, under the 2 m line).
  {
    const float gaps_ft[6] = { 82.0f, 90.0f, 98.0f, 106.0f, 112.0f, 115.0f };
    long runs = 0;
    for (int g = 0; g < 6; ++g) {
      const float gap = gaps_ft[g] * 0.3048f;
      for (uint32_t seed = 1; seed <= 200; ++seed) {
        const DvRun r = dvRun([&](float) { return gap; }, 0.0f, 120.0f, true, seed * 7919u + g);
        assert(r.verdict == FMDV_NONE);
        runs++;
      }
      FmDivergeDwell d = { 0, -1.0f };
      for (int i = 1; i <= 1200; ++i) {
        const float n = ((i / 30) % 2 == 0) ? -kDvNoise : kDvNoise;   // flips every 3 s, against the dwell
        assert(fmDivergeDwellStep(&d, (uint32_t)(i * 100), true, gap + n, kDvCeil, kDvFar, kDvDwell, kDvEps, kDvEps)
               == FMDV_NONE);
      }
    }
    printf("R2 (b) pacing 82-115 ft with the 6 ft band: %ld runs x 120 s, no end (the old rule would end every one)\n", runs);
    // The old rule on the same pacing: it ends - that is F-1.
    uint32_t since = 0; float start = -1.0f; bool ended = false;
    for (int i = 1; i <= 1200 && !ended; ++i) ended = oldDivergeStep(&since, &start, (uint32_t)(i * 100), false, true, 30.0f, kDvCeil);
    assert(ended);
  }
  // (c) FALLING BEHIND at 0.7 mph from 20 m: no end while the gap is inside 118 ft, then the old rule ends it - at the
  //     first dwell end past 36 m. Noise-free asserted. With the 6 ft band the 0.94 m opening per dwell can read as more
  //     than 2 m; how often that ends it early is reported (the audit's optional 3-fix averaging is not implemented).
  {
    const float rate = 0.7f * 0.44704f;
    const DvRun r = dvRun([&](float t) { return 20.0f + rate * t; }, 0.0f, 200.0f, false, 1);
    const float cross36 = (kDvFar - 20.0f) / rate;
    assert(r.verdict == FMDV_NOT_CLOSING);
    assert(r.true_m > kDvFar);
    assert(r.t_s - cross36 <= 3.0f + 0.1f + 1e-3f);
    int early = 0;
    float earliest = 1e9f;
    for (uint32_t seed = 1; seed <= 500; ++seed) {
      const DvRun q = dvRun([&](float t) { return 20.0f + rate * t; }, 0.0f, 200.0f, true, seed);
      assert(q.verdict != FMDV_NONE);
      if (q.true_m <= kDvFar) { early++; if (q.true_m < earliest) earliest = q.true_m; }
    }
    printf("R2 (c) falling behind 0.7 mph: ends at %.1f m (%.0f ft) by the old rule, %.1f s after passing 118 ft; with the "
           "6 ft band %d of 500 runs end inside 118 ft (earliest %.0f ft) - noise, reported\n",
           (double)r.true_m, (double)(r.true_m / 0.3048f), (double)(r.t_s - cross36), early,
           (double)((early ? earliest : 0.0f) / 0.3048f));
  }
  // (d) THE RIDER IS 2 mph FASTER: the gap opens 2.7 m per dwell - over the 2 m line - so it ends, inside the band.
  {
    const float rate = 2.0f * 0.44704f;
    const DvRun r = dvRun([&](float t) { return 20.0f + rate * t; }, 0.0f, 200.0f, false, 1);
    assert(r.verdict == FMDV_OPENING);
    assert(r.true_m <= kDvFar);
    const float cross24 = (kDvCeil - 20.0f) / rate;
    assert(r.t_s - cross24 <= 3.0f + 0.2f + 1e-3f);
    int ended_in_band = 0;
    for (uint32_t seed = 1; seed <= 500; ++seed) {
      const DvRun q = dvRun([&](float t) { return 20.0f + rate * t; }, 0.0f, 200.0f, true, seed);
      assert(q.verdict != FMDV_NONE);                            // always ends
      if (q.verdict == FMDV_OPENING) ended_in_band++;
    }
    printf("R2 (d) rider 2 mph faster: ends %.1f s after passing 79 ft (running away); with the 6 ft band all 500 runs end, "
           "%d inside the band\n", (double)(r.t_s - cross24), ended_in_band);
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
  testLongLossStickyFromHold();   // V2.5-Evo - 2026-10-12 - A-3
  testDivergeDwell();             // V2.5-Evo - 2026-10-30 - audit 2026-10-20 R2
  printf("follow_me_pause_test: all tests passed\n");
  return 0;
}
