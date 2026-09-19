// Host test for Common/OutputRamp.h - the motor output rise-limit calcPWM() applies.
// Channel ranges as the owner's calibration: 1000-2000 us on both. The RX's generatePWM task runs at
// 100 Hz, so tick_hz = 100: a 2.0 s ramp is 5 us per tick, a 0.5 s ramp is 20 us per tick.

#include <assert.h>
#include <stdint.h>

#include "../../Source/Common/OutputRamp.h"

static const uint16_t MIN0 = 1000, MAX0 = 2000, MIN1 = 1000, MAX1 = 2000;
static const float    HZ   = 100.0f;

// The RX's ORIGINAL inline block, copied verbatim in shape, so the header can be shown to produce
// the same bytes tick for tick while the ramp is on.
struct OldRamp { uint16_t pwm0_ramp, pwm1_ramp; bool init; };
static void oldStep(OldRamp* o, uint16_t* PWM0_time, uint16_t* PWM1_time, float motor_ramp_s)
{
  if (motor_ramp_s > 0.001f)
  {
    if (!o->init) { o->pwm0_ramp = MIN0; o->pwm1_ramp = MIN1; o->init = true; }
    float s0 = (float)(MAX0 - MIN0) / (motor_ramp_s * 100.0f); if (s0 < 1.0f) s0 = 1.0f;
    float s1 = (float)(MAX1 - MIN1) / (motor_ramp_s * 100.0f); if (s1 < 1.0f) s1 = 1.0f;
    uint16_t step0 = (uint16_t)s0, step1 = (uint16_t)s1;
    if (*PWM0_time > o->pwm0_ramp + step0) o->pwm0_ramp += step0; else o->pwm0_ramp = *PWM0_time;
    if (*PWM1_time > o->pwm1_ramp + step1) o->pwm1_ramp += step1; else o->pwm1_ramp = *PWM1_time;
    *PWM0_time = o->pwm0_ramp;
    *PWM1_time = o->pwm1_ramp;
  }
}

static void step(OutputRampState* s, uint16_t t0, uint16_t t1, float ramp_s, uint16_t* o0, uint16_t* o1)
{
  outputRampStep(s, t0, t1, MIN0, MAX0, MIN1, MAX1, ramp_s, HZ, o0, o1);
}

int main()
{
  uint16_t o0, o1;

  // ---- The per-tick step: span / (ramp x ticks), floored at 1 us ----
  assert(outputRampStepPerTick(1000, 2000, 2.0f, HZ) == 5);
  assert(outputRampStepPerTick(1000, 2000, 0.5f, HZ) == 20);
  assert(outputRampStepPerTick(1000, 2000, 0.75f, HZ) == 13);    // 13.33 truncates, as the RX always did
  assert(outputRampStepPerTick(1000, 1050, 4.0f, HZ) == 1);      // 0.125 -> the 1 us floor

  // ---- Rise: 0 -> full over the ramp time, never past the target; fall: instant ----
  {
    OutputRampState s = {0, 0, false};
    step(&s, MIN0, MIN1, 2.0f, &o0, &o1);                         // seeded at the minimums
    assert(o0 == 1000 && o1 == 1000 && s.init);
    uint32_t ticks = 0;
    do { step(&s, 2000, 2000, 2.0f, &o0, &o1); ticks++; assert(o0 <= 2000 && o1 <= 2000); } while (o0 < 2000);
    assert(ticks == 200 && o1 == 2000);                           // 1000 us at 5 us/tick = 200 ticks = 2.0 s
    step(&s, 1000, 1500, 2.0f, &o0, &o1);
    assert(o0 == 1000 && o1 == 1500);                             // a lower target is taken at once
    step(&s, 1503, 1502, 2.0f, &o0, &o1);
    assert(o0 == 1005 && o1 == 1502);                             // ch0 rises one step; ch1's target within one step is reached exactly
  }

  // ---- Continuity across a rate change: the output carries on from where it is, at the new step ----
  {
    OutputRampState s = {0, 0, false};
    step(&s, MIN0, MIN1, 2.0f, &o0, &o1);
    for (int i = 0; i < 50; ++i) step(&s, 2000, 2000, 2.0f, &o0, &o1);   // 0.5 s at 5 us/tick
    assert(o0 == 1250);
    step(&s, 2000, 2000, 0.5f, &o0, &o1);                          // the automatic ramp takes over mid-rise
    assert(o0 == 1270 && o1 == 1270);                              // +20 from 1250, no jump either way
    step(&s, 2000, 2000, 2.0f, &o0, &o1);                          // and back to the slow one
    assert(o0 == 1275);
  }

  // ---- Off: pass-through, and the memory keeps tracking so a later ramp resumes with no dip ----
  {
    OutputRampState s = {0, 0, false};
    step(&s, 1800, 1700, 0.0f, &o0, &o1);
    assert(o0 == 1800 && o1 == 1700 && s.out0 == 1800 && s.out1 == 1700);
    step(&s, 1900, 1900, 2.0f, &o0, &o1);                          // ramp switched on mid-run
    assert(o0 == 1805 && o1 == 1705);                              // continues from the tracked output, not from the minimum
  }
  // The pre-header behaviour for comparison: the old block skipped tracking while off, so the same
  // sequence resumed from the seed (a dip to 1000, then a rise) - the quirk this header closes.
  {
    OldRamp o = {0, 0, false};
    uint16_t p0 = 1800, p1 = 1700;
    oldStep(&o, &p0, &p1, 0.0f);
    p0 = 1900; p1 = 1900;
    oldStep(&o, &p0, &p1, 2.0f);
    assert(p0 == 1005 && p1 == 1005);
  }

  // ---- Byte-identical to the old inline block while the ramp is on: a scripted target sequence ----
  {
    static const uint16_t seq0[] = { 1000, 1300, 2000, 2000, 1200, 1210, 1215, 2000, 1000, 1000, 1990, 2000, 1500 };
    static const uint16_t seq1[] = { 1000, 1000, 1400, 2000, 2000, 1000, 1005, 1006, 1300, 1000, 2000, 2000, 1500 };
    static const float    ramps[] = { 0.75f, 2.0f, 0.2f, 4.0f, 1.0f };
    for (unsigned r = 0; r < sizeof(ramps) / sizeof(ramps[0]); ++r) {
      OutputRampState s = {0, 0, false};
      OldRamp o = {0, 0, false};
      for (unsigned i = 0; i < sizeof(seq0) / sizeof(seq0[0]); ++i) {
        for (int rep = 0; rep < 7; ++rep) {                         // hold each target for 7 ticks
          uint16_t p0 = seq0[i], p1 = seq1[i];
          oldStep(&o, &p0, &p1, ramps[r]);
          step(&s, seq0[i], seq1[i], ramps[r], &o0, &o1);
          assert(o0 == p0 && o1 == p1);
        }
      }
    }
  }

  return 0;
}
