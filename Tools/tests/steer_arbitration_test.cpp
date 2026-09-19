// Host test for Common/SteerArbitration.h - the stick-takeover arbitration during automatic steering.
// The RX's constants (RTMState.ino kSteerTakeover*): engage deadband 40 counts, release deadband 20,
// engage persistence 500 ms, release persistence 200 ms, grace 2000 ms from the owner's motion start,
// maximum takeover 20000 ms. Ticks are 100 ms like runFmLoop() / runRtmLoop().

#include <assert.h>
#include <stdint.h>

#include "../../Source/Common/SteerArbitration.h"

static const SteerTakeoverParams P = { 40u, 20u, 500u, 200u, 2000u, 20000u };

// Feed one constant steering byte for `ticks` ticks starting at t0 (100 ms apart), owner active with
// the given motion start. Returns the last verdict; *t_out is the tick time of the first edge verdict
// (ENGAGED / RELEASED / TIMED_OUT) or of the last tick if none; *edges counts edge verdicts seen.
static uint8_t feed(SteerTakeoverState* s, uint32_t t0, uint32_t ticks, uint32_t owner_since,
                    uint8_t byte, uint32_t* t_out, uint32_t* edges)
{
  uint8_t v = STO_INACTIVE;
  *edges = 0;
  *t_out = t0 + (ticks - 1) * 100u;
  bool first_edge = true;
  for (uint32_t i = 0; i < ticks; ++i) {
    const uint32_t now = t0 + i * 100u;
    v = steerTakeoverStep(s, now, true, owner_since, byte, &P);
    if (v == STO_ENGAGED || v == STO_RELEASED || v == STO_TIMED_OUT) {
      (*edges)++;
      if (first_edge) { *t_out = now; first_edge = false; }
    }
  }
  return v;
}

// A state that has seen the centre and whose owner's grace has long passed: the common starting
// point for the engage/release cases. Owner motion began at 1000; the stick was centred at 1000-1500.
static SteerTakeoverState primed(uint32_t* t_after)
{
  SteerTakeoverState s = {0, 0, 0, false, false};
  uint32_t t, e;
  uint8_t v = feed(&s, 1000u, 6u, 1000u, 127u, &t, &e);   // centred for 0.5 s
  assert(v == STO_INACTIVE && e == 0);
  assert(s.centre_seen && !s.active);
  // idle at centre until 4000 so the 2 s grace from 1000 is comfortably served
  v = feed(&s, 1600u, 25u, 1000u, 127u, &t, &e);
  assert(v == STO_INACTIVE && e == 0);
  *t_after = 4100u;
  return s;
}

int main()
{
  // ---- The deflection helper is symmetric about 127 ----
  assert(steerTakeoverDeflection(127u) == 0u);
  assert(steerTakeoverDeflection(167u) == 40u && steerTakeoverDeflection(87u) == 40u);
  assert(steerTakeoverDeflection(255u) == 128u && steerTakeoverDeflection(0u) == 127u);

  uint32_t t, e;

  // ---- Engage at EXACTLY 500 ms sustained at 40 counts; nothing at 490 ms ----
  {
    SteerTakeoverState s = primed(&t);
    const uint32_t t0 = t;
    uint8_t v = feed(&s, t0, 5u, 1000u, 167u, &t, &e);      // ticks at 0,100,200,300,400 ms of deflection
    assert(v == STO_INACTIVE && e == 0 && !s.active);        // 400 ms: not yet
    v = steerTakeoverStep(&s, t0 + 490u, true, 1000u, 167u, &P);
    assert(v == STO_INACTIVE && !s.active);                  // 490 ms: not yet
    v = steerTakeoverStep(&s, t0 + 500u, true, 1000u, 167u, &P);
    assert(v == STO_ENGAGED && s.active && s.active_since_ms == t0 + 500u);   // 500 ms: engaged, once
    v = steerTakeoverStep(&s, t0 + 600u, true, 1000u, 167u, &P);
    assert(v == STO_ACTIVE);                                 // the edge is reported once
  }
  // The same on the other side of centre (87 = 40 counts left).
  {
    SteerTakeoverState s = primed(&t);
    uint8_t v = feed(&s, t, 6u, 1000u, 87u, &t, &e);
    assert(v == STO_ENGAGED && e == 1 && s.active);
  }
  // 39 counts (166 / 88) is inside the band and never engages, however long it is held.
  {
    SteerTakeoverState s = primed(&t);
    uint8_t v = feed(&s, t, 100u, 1000u, 166u, &t, &e);     // 10 s at 39
    assert(v == STO_INACTIVE && e == 0 && !s.active && s.deflect_since_ms == 0);
  }

  // ---- No engage without centre-seen: a stick that was never read centred cannot take over ----
  {
    SteerTakeoverState s = {0, 0, 0, false, false};
    uint8_t v = feed(&s, 1000u, 100u, 1000u, 167u, &t, &e); // 10 s deflected, grace long served
    assert(v == STO_INACTIVE && e == 0 && !s.active && !s.centre_seen);
    assert(s.deflect_since_ms == 1000u);                     // the persistence stands; only centre-seen is missing
    // A drifted centre of 25 counts (152) counts as neither centred nor deflected: still nothing.
    v = feed(&s, 11000u, 50u, 1000u, 152u, &t, &e);
    assert(v == STO_INACTIVE && !s.centre_seen && s.deflect_since_ms == 0);
    // Once read centred, a fresh 500 ms deflection engages.
    v = steerTakeoverStep(&s, 16000u, true, 1000u, 127u, &P);
    assert(v == STO_INACTIVE && s.centre_seen);
    v = feed(&s, 16100u, 6u, 1000u, 167u, &t, &e);
    assert(v == STO_ENGAGED && t == 16600u);
  }

  // ---- No engage inside the owner's 2 s grace; engages once the grace is served ----
  {
    SteerTakeoverState s = {0, 0, 0, false, false};
    // owner motion began at 5000; the stick was centred at 5000, then hard over from 5100
    uint8_t v = steerTakeoverStep(&s, 5000u, true, 5000u, 127u, &P);
    assert(v == STO_INACTIVE && s.centre_seen);
    v = feed(&s, 5100u, 19u, 5000u, 200u, &t, &e);           // 5100 .. 6900: inside the grace
    assert(v == STO_INACTIVE && e == 0 && !s.active);
    v = steerTakeoverStep(&s, 7000u, true, 5000u, 200u, &P); // grace served at 7000; persistence long served
    assert(v == STO_ENGAGED && s.active);
  }
  // A missing motion clock (owner_since 0) can never engage, whatever else holds.
  {
    SteerTakeoverState s = {0, 0, 0, false, false};
    steerTakeoverStep(&s, 1000u, true, 0u, 127u, &P);
    uint8_t v = feed(&s, 1100u, 100u, 0u, 200u, &t, &e);
    assert(v == STO_INACTIVE && e == 0 && !s.active);
  }

  // ---- Release at 200 ms inside 20 counts; the controller resumes; centre-seen is kept ----
  {
    SteerTakeoverState s = primed(&t);
    uint8_t v = feed(&s, t, 6u, 1000u, 167u, &t, &e);
    assert(v == STO_ENGAGED);
    const uint32_t t0 = t + 100u;
    v = steerTakeoverStep(&s, t0, true, 1000u, 130u, &P);            // 3 counts: centred, timer starts
    assert(v == STO_ACTIVE && s.centre_since_ms == t0);
    v = steerTakeoverStep(&s, t0 + 100u, true, 1000u, 130u, &P);
    assert(v == STO_ACTIVE);
    v = steerTakeoverStep(&s, t0 + 200u, true, 1000u, 130u, &P);
    assert(v == STO_RELEASED && !s.active && s.centre_seen && s.active_since_ms == 0);
    v = steerTakeoverStep(&s, t0 + 300u, true, 1000u, 130u, &P);
    assert(v == STO_INACTIVE);                                       // the edge is reported once
    // A second takeover needs the full 500 ms of deflection again.
    v = feed(&s, t0 + 400u, 5u, 1000u, 167u, &t, &e);
    assert(v == STO_INACTIVE && !s.active);
    v = steerTakeoverStep(&s, t0 + 900u, true, 1000u, 167u, &P);
    assert(v == STO_ENGAGED);
  }
  // 19 counts (146) releases; 20 counts (147) does not - the release band is strictly inside 20.
  {
    SteerTakeoverState s = primed(&t);
    feed(&s, t, 6u, 1000u, 167u, &t, &e);
    uint8_t v = feed(&s, t + 100u, 50u, 1000u, 147u, &t, &e);       // 5 s at exactly 20
    assert(v == STO_ACTIVE && e == 0 && s.active && s.centre_since_ms == 0);
    v = feed(&s, t + 100u, 3u, 1000u, 146u, &t, &e);                 // 19: released after 200 ms
    assert(v == STO_RELEASED && e == 1);
  }

  // ---- 20-39 counts neither engages nor releases ----
  {
    SteerTakeoverState s = primed(&t);
    uint8_t v = feed(&s, t, 100u, 1000u, 157u, &t, &e);              // 30 counts, 10 s
    assert(v == STO_INACTIVE && e == 0 && !s.active);
    v = feed(&s, t + 100u, 6u, 1000u, 167u, &t, &e);                 // now 40: engages
    assert(v == STO_ENGAGED);
    v = feed(&s, t + 100u, 100u, 1000u, 157u, &t, &e);               // back to 30, 10 s: still standing
    assert(v == STO_ACTIVE && e == 0 && s.active);
  }

  // ---- Chatter: 20/60 alternating at 10 Hz never engages ----
  {
    SteerTakeoverState s = primed(&t);
    uint32_t now = t;
    for (uint32_t i = 0; i < 300u; ++i, now += 100u) {              // 30 s of chop
      const uint8_t byte = (i & 1u) ? 187u : 147u;                    // 60 / 20 counts
      const uint8_t v = steerTakeoverStep(&s, now, true, 1000u, byte, &P);
      assert(v == STO_INACTIVE && !s.active);
    }
  }
  // ---- Chatter: 10/30 alternating while active never releases ----
  {
    SteerTakeoverState s = primed(&t);
    feed(&s, t, 6u, 1000u, 167u, &t, &e);
    assert(s.active);
    uint32_t now = t + 100u;
    for (uint32_t i = 0; i < 150u; ++i, now += 100u) {              // 15 s of chop, inside the 20 s cap
      const uint8_t byte = (i & 1u) ? 157u : 137u;                    // 30 / 10 counts
      const uint8_t v = steerTakeoverStep(&s, now, true, 1000u, byte, &P);
      assert(v == STO_ACTIVE && s.active);
    }
  }

  // ---- Timeout at 20 s: STO_TIMED_OUT once, active false, centre-seen cleared ----
  {
    SteerTakeoverState s = primed(&t);
    uint8_t v = feed(&s, t, 6u, 1000u, 167u, &t, &e);
    assert(v == STO_ENGAGED);
    const uint32_t t_eng = t;
    v = feed(&s, t_eng + 100u, 199u, 1000u, 167u, &t, &e);           // up to 19.9 s of hold
    assert(v == STO_ACTIVE && e == 0 && s.active);
    v = steerTakeoverStep(&s, t_eng + 20000u, true, 1000u, 167u, &P);
    assert(v == STO_TIMED_OUT && !s.active && !s.centre_seen && s.active_since_ms == 0);
    // Still deflected afterwards: nothing engages, because the centre has not been seen again.
    v = feed(&s, t_eng + 20100u, 100u, 1000u, 167u, &t, &e);
    assert(v == STO_INACTIVE && e == 0 && !s.active);
    // Centre it once, deflect again: a new takeover after the usual 500 ms.
    steerTakeoverStep(&s, t_eng + 30200u, true, 1000u, 127u, &P);
    v = feed(&s, t_eng + 30300u, 6u, 1000u, 167u, &t, &e);
    assert(v == STO_ENGAGED);
  }
  // A release that completes on the same tick as the timeout is a release (the rider centred).
  {
    SteerTakeoverState s = primed(&t);
    feed(&s, t, 6u, 1000u, 167u, &t, &e);
    const uint32_t t_eng = t;
    feed(&s, t_eng + 100u, 197u, 1000u, 167u, &t, &e);               // to 19.7 s
    steerTakeoverStep(&s, t_eng + 19800u, true, 1000u, 127u, &P);
    steerTakeoverStep(&s, t_eng + 19900u, true, 1000u, 127u, &P);
    uint8_t v = steerTakeoverStep(&s, t_eng + 20000u, true, 1000u, 127u, &P);
    assert(v == STO_RELEASED && s.centre_seen);
  }

  // ---- Owner drop zeroes the state, mid-takeover included ----
  {
    SteerTakeoverState s = primed(&t);
    feed(&s, t, 6u, 1000u, 167u, &t, &e);
    assert(s.active);
    uint8_t v = steerTakeoverStep(&s, t + 100u, false, 1000u, 167u, &P);   // trigger released / mode ended
    assert(v == STO_INACTIVE);
    assert(s.deflect_since_ms == 0 && s.centre_since_ms == 0 && s.active_since_ms == 0);
    assert(!s.centre_seen && !s.active);
    // The next owner starts from nothing: centre-seen and the grace must be earned again.
    v = feed(&s, t + 200u, 100u, t + 200u, 167u, &t, &e);
    assert(v == STO_INACTIVE && e == 0 && !s.active);
  }

  // ---- millis() wrap: the persistence and the timeout survive the 32-bit rollover ----
  {
    SteerTakeoverState s = {0, 0, 0, false, false};
    const uint32_t base = 0xFFFFF000u;                               // 4096 ms before the wrap
    steerTakeoverStep(&s, base, true, base, 127u, &P);               // centred; motion begins here
    uint8_t v = feed(&s, base + 2000u, 6u, base, 167u, &t, &e);      // grace served at +2000; wraps at +4096
    assert(v == STO_ENGAGED && t == base + 2500u);
    v = feed(&s, base + 2600u, 200u, base, 167u, &t, &e);            // crosses 0 while active
    assert(v == STO_TIMED_OUT && t == base + 2500u + 20000u);
  }
  // A stamp at millis() == 0 is stored as 1, never as the 0 sentinel.
  {
    SteerTakeoverState s = {0, 0, 0, false, false};
    steerTakeoverStep(&s, 0u, true, 1u, 127u, &P);
    assert(s.centre_since_ms == 1u);
    steerTakeoverStep(&s, 0u, true, 1u, 200u, &P);
    assert(s.deflect_since_ms == 1u);
  }

  // ---- Mode 0 is byte-identical: the selector equals the old `auto_steer ? override : stick` ----
  // A scripted vector of (rtm_rx_active, fm_rx_active, override_enabled, thr, stick) samples; the
  // old selector is written out exactly as calcPWM() had it, the new one goes through the header
  // with the atomic held false (the setting at 0 never writes it true). Every sample must agree.
  {
    struct Sample { bool rtm; bool fm; bool ovr; uint8_t thr; uint8_t stick; uint8_t override_byte; };
    static const Sample vec[] = {
      { false, false, true,   0, 127, 127 }, { false, false, true, 200, 200, 127 },
      { true,  false, true,  24, 200,  90 }, { true,  false, true,  25, 200,  90 },
      { true,  false, true, 255,  10, 210 }, { true,  false, false, 255, 10, 210 },
      { false, true,  true,  25, 167, 140 }, { false, true,  true,  24, 167, 140 },
      { false, true,  false, 200, 167, 140 }, { true,  true,  true, 100,   0, 254 },
      { true,  true,  true,   0,   0, 254 }, { false, false, false, 0,  87,  60 },
    };
    for (unsigned i = 0; i < sizeof(vec) / sizeof(vec[0]); ++i) {
      const Sample& x = vec[i];
      const bool    auto_steer_old = ((x.rtm || x.fm) && x.ovr && x.thr >= 25);
      const uint8_t old_byte       = auto_steer_old ? x.override_byte : x.stick;
      const bool    auto_owner     = ((x.rtm || x.fm) && x.ovr && x.thr >= 25);   // the unchanged test
      const bool    takeover       = false;                                         // mode 0: never published true
      const uint8_t new_byte       = steerTakeoverSelectsStick(auto_owner, takeover) ? x.stick : x.override_byte;
      assert(new_byte == old_byte);
      assert(steerTakeoverSelectsStick(auto_owner, takeover) == !auto_steer_old);
    }
    // And with a takeover standing the stick wins ONLY while there is an owner: no owner, no change.
    assert(steerTakeoverSelectsStick(true,  true)  == true);    // owner + takeover: the stick
    assert(steerTakeoverSelectsStick(true,  false) == false);   // owner, no takeover: the controller
    assert(steerTakeoverSelectsStick(false, true)  == true);    // no owner: the stick, as always
    assert(steerTakeoverSelectsStick(false, false) == true);
  }

  return 0;
}
