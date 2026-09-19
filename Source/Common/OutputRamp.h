// V2.5-Evo - 2026-09-19 - The motor output rise-limit (the "motor ramp"), pure and host-testable.
//   calcPWM() (RX PWM.ino) rise-limits BOTH motor outputs so 0 -> full takes ramp_s seconds: a violent
//   throttle yank and a single motor taking off (throttle- or steering-driven) both build over the ramp
//   instead of landing in one 10 ms tick. This header holds the step that used to be inline there,
//   arithmetic unchanged, so Tools/tests/output_ramp_test.cpp runs the exact code the RX runs:
//     RISE     - per tick the output may climb by at most step = span / (ramp_s x tick_hz), never
//                less than 1 us per tick, and never past the target.
//     FALL     - instant. A lower target is taken on the same tick, so a release, the failsafe, an
//                RTM e-stop and a straightening all drop the motor immediately.
//     CONTINUITY across a rate change - the memory holds the last OUTPUT, and the step is recomputed
//                on every tick from whatever ramp_s the caller passes, so switching from one ramp to
//                another mid-rise continues from the current output at the new rate: no jump either way.
//                The RX selects between two ramps (the slow manual motor_ramp_s and the automatic
//                modes' auto_ramp_s) exactly this way.
//     OFF      - ramp_s at or below zero passes the targets straight through - AND the memory keeps
//                tracking them. Before this header the RX skipped the whole block while the ramp was
//                off, so the memory went stale and a later non-zero ramp resumed from an old value
//                (a momentary dip). Tracking on every tick closes that.
//     INIT     - the first call seeds the memory at the channel minimums (the stopped state), as before.
//   Nothing here reads config or time; the caller passes the targets, the channel ranges, the ramp and
//   the tick rate, and applies the terminal effective_thr == 0 clamp AFTER this (it stays the last writer).
#ifndef BREMOTE_OUTPUT_RAMP_H
#define BREMOTE_OUTPUT_RAMP_H

#include <stdint.h>

// The ramp's memory: the last OUTPUT of each channel, and whether it has been seeded.
struct OutputRampState {
  uint16_t out0;     // last output of channel 0 (us)
  uint16_t out1;     // last output of channel 1 (us)
  bool     init;     // false until the first call seeds out0/out1 at the channel minimums
};

// outputRampStepPerTick - how far one channel may rise in one tick.
// Inputs:  min_us / max_us - the channel's calibrated range; ramp_s - the rise time 0 -> full;
//          tick_hz - how often the caller runs (100 for the RX's generatePWM task).
// Returns: the per-tick step in us, at least 1. Side effects: none.
// The arithmetic is the RX's original, unchanged: (uint16_t)max(1.0f, span / (ramp_s * tick_hz)).
static inline uint16_t outputRampStepPerTick(uint16_t min_us, uint16_t max_us, float ramp_s, float tick_hz)
{
  float st = (float)(max_us - min_us) / (ramp_s * tick_hz);
  if (st < 1.0f) st = 1.0f;
  return (uint16_t)st;
}

// outputRampStep - advance both channels by one tick.
// Inputs:  s - the memory (updated in place); target0 / target1 - this tick's computed outputs (us);
//          min0 / max0 / min1 / max1 - the two channels' calibrated ranges; ramp_s - the rise time in
//          force on this tick (<= 0 = off); tick_hz - the caller's rate.
// Outputs: *out0 / *out1 - the rise-limited outputs to apply.
// Side effects: only on *s. With the ramp off the outputs are the targets and the memory tracks
//          them; with it on, each channel rises by at most one step toward its target and snaps
//          down to a lower target at once. Unseeded memory is seeded at (min0, min1) first.
static inline void outputRampStep(OutputRampState* s,
                                  uint16_t target0, uint16_t target1,
                                  uint16_t min0, uint16_t max0, uint16_t min1, uint16_t max1,
                                  float ramp_s, float tick_hz,
                                  uint16_t* out0, uint16_t* out1)
{
  if (!s->init) { s->out0 = min0; s->out1 = min1; s->init = true; }

  if (ramp_s > 0.001f) {
    const uint16_t step0 = outputRampStepPerTick(min0, max0, ramp_s, tick_hz);
    const uint16_t step1 = outputRampStepPerTick(min1, max1, ramp_s, tick_hz);
    if (target0 > s->out0 + step0) s->out0 += step0; else s->out0 = target0;
    if (target1 > s->out1 + step1) s->out1 += step1; else s->out1 = target1;
  } else {
    s->out0 = target0;   // ramp off: pass through, and keep tracking so a later ramp resumes from here
    s->out1 = target1;
  }

  *out0 = s->out0;
  *out1 = s->out1;
}

#endif
