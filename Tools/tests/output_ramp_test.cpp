// Host test for Common/OutputRamp.h - the THROTTLE rise-limit calcPWM() applies before the mixer.
//
// V2.5-Evo - 2026-09-24 - rewritten with the header. The old test drove a two-channel ramp on the PWM
// microseconds and proved it byte-identical to the inline block calcPWM() used to carry. That block is
// gone: it rate-limited PWM0_time / PWM1_time, and on a differential drive the difference between
// those two IS the turn, so it slewed steering too. The ramp is now a single channel on the throttle
// (0-255 counts), ahead of the mixer, with a Q12 accumulator. The equivalence test is therefore
// replaced by what actually matters now: the ramp TIME the rider gets for each configured value, which
// the whole-count step in this smaller domain could not deliver.
//
// The RX's generatePWM task runs at 100 Hz, so tick_hz = 100 throughout.

#include <assert.h>
#include <stdint.h>

#include "../../Source/Common/OutputRamp.h"

static const float HZ = 100.0f;

// ticksToFull - how many ticks 0 -> 255 takes at a given ramp setting, driven exactly as calcPWM()
// drives it: the target is held at full and the helper is stepped until the output reaches 255.
static uint32_t ticksToFull(float ramp_s)
{
  OutputRampState s = {0};
  uint32_t ticks = 0;
  uint8_t out = 0;
  while (out < 255) {
    out = throttleRampStep(&s, 255, ramp_s, HZ);
    ticks++;
    assert(ticks < 100000);            // no setting may fail to converge
  }
  return ticks;
}

int main()
{
  // ---- The per-tick step: full scale / (ramp x ticks), truncated, floored at 1 unit ----
  assert(THROTTLE_RAMP_Q12_FULL == 1044480UL);
  assert(throttleRampStepQ12(0.20f, HZ) == 52224UL);    // 1044480 / 20
  assert(throttleRampStepQ12(1.00f, HZ) == 10444UL);    // 10444.8 truncates DOWN - the slow, safe side
  assert(throttleRampStepQ12(2.00f, HZ) ==  5222UL);    // 5222.4
  assert(throttleRampStepQ12(4.00f, HZ) ==  2611UL);    // 2611.2 - still three orders above the floor
  assert(throttleRampStepQ12(0.005f, HZ) == THROTTLE_RAMP_Q12_FULL);   // shorter than a tick = instant

  // ---- The ramp TIME the rider actually gets. This is the whole point of the Q12 accumulator: a
  // whole-count step in the 0-255 domain floors at 1 count = 255 ticks, so every setting from 1.275 s
  // up produced the same 2.55 s and 4.00 s ran 36% FAST. Every case here is within ONE tick of
  // nominal and never below it. ----
  assert(ticksToFull(0.20f) ==  20);    // 0.20 s exactly
  assert(ticksToFull(0.50f) ==  51);    // 0.51 s
  assert(ticksToFull(0.75f) ==  76);    // 0.76 s
  assert(ticksToFull(1.00f) == 101);    // 1.01 s  - the owner's current setting
  assert(ticksToFull(1.50f) == 151);    // 1.51 s
  assert(ticksToFull(2.00f) == 201);    // 2.01 s  - the owner's setting earlier today
  assert(ticksToFull(3.00f) == 301);    // 3.01 s
  assert(ticksToFull(4.00f) == 401);    // 4.01 s  - the configured maximum, honoured

  // ---- Sweep the whole configured range: never faster than asked, never more than one tick slow ----
  for (int centis = 20; centis <= 400; ++centis) {
    const float    ramp_s  = (float)centis / 100.0f;
    const uint32_t ticks   = ticksToFull(ramp_s);
    assert(ticks >= (uint32_t)centis);            // never faster than configured (the safe direction)
    assert(ticks <= (uint32_t)centis + 1);        // and never more than one 10 ms tick slow
  }

  // ---- Rise: never past the target; fall: instant, on the same tick ----
  {
    OutputRampState s = {0};
    assert(throttleRampStep(&s, 0, 1.0f, HZ) == 0);           // at rest the memory is 0
    uint8_t out = 0;
    for (int i = 0; i < 101; ++i) {
      const uint8_t prev = out;
      out = throttleRampStep(&s, 100, 1.0f, HZ);
      assert(out <= 100 && out >= prev);                      // monotone rise, never past the target
    }
    assert(out == 100);                                       // and it does arrive
    assert(throttleRampStep(&s, 40, 1.0f, HZ) == 40);         // a lower target is taken at once
    assert(throttleRampStep(&s, 0,  1.0f, HZ) == 0);          // release / failsafe / e-stop: instant
  }

  // ---- The memory parks at 0 with the trigger released, so a re-engage is a fresh full ramp ----
  {
    OutputRampState s = {0};
    for (int i = 0; i < 50; ++i) throttleRampStep(&s, 255, 1.0f, HZ);   // half way up
    assert(throttleRampStep(&s, 0, 1.0f, HZ) == 0);
    assert(s.out_q12 == 0);                                   // no banked ramp credit
    uint32_t ticks = 0; uint8_t out = 0;
    while (out < 255) { out = throttleRampStep(&s, 255, 1.0f, HZ); ticks++; }
    assert(ticks == 101);                                     // the full ramp again, not a shortcut
  }

  // ---- Continuity across a rate change: the throttle carries on from where it is, at the new rate.
  // This is how the RX switches between motor_ramp_s and auto_ramp_s mid-rise. ----
  {
    OutputRampState s = {0};
    for (int i = 0; i < 50; ++i) throttleRampStep(&s, 255, 2.00f, HZ);  // 50 ticks of the slow ramp
    const uint32_t mid = s.out_q12;
    assert(mid == 50UL * 5222UL);
    const uint8_t fast = throttleRampStep(&s, 255, 0.50f, HZ);          // the automatic ramp takes over
    assert(s.out_q12 == mid + 20889UL);                                 // +1 fast step, no jump either way
    (void)fast;
    const uint8_t slow = throttleRampStep(&s, 255, 2.00f, HZ);          // and back to the slow one
    assert(s.out_q12 == mid + 20889UL + 5222UL);
    (void)slow;
  }

  // ---- Off: pass-through, AND the memory keeps tracking, so a later ramp resumes with no dip.
  // (The pre-header RX skipped the whole block while the ramp was off and dipped on re-enable.) ----
  {
    OutputRampState s = {0};
    assert(throttleRampStep(&s, 180, 0.0f, HZ) == 180);
    assert(s.out_q12 == 180UL * THROTTLE_RAMP_Q12_ONE);
    const uint8_t out = throttleRampStep(&s, 200, 2.00f, HZ);           // ramp switched on mid-run
    assert(out == 181);                                                 // continues from 180, not from 0
  }

  // ---- The output can never exceed the target, at any ramp, from any state: this is what keeps the
  // RTM / Follow-Me throttle caps binding after the ramp. ----
  {
    static const float ramps[] = { 0.0f, 0.20f, 0.75f, 1.00f, 2.00f, 4.00f };
    static const uint8_t targets[] = { 0, 1, 25, 60, 127, 200, 254, 255, 13, 0, 255, 0 };
    for (unsigned r = 0; r < sizeof(ramps) / sizeof(ramps[0]); ++r) {
      OutputRampState s = {0};
      for (unsigned i = 0; i < sizeof(targets) / sizeof(targets[0]); ++i) {
        for (int rep = 0; rep < 40; ++rep) {
          const uint8_t out = throttleRampStep(&s, targets[i], ramps[r], HZ);
          assert(out <= targets[i]);
        }
      }
    }
  }

  return 0;
}
