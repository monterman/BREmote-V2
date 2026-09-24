// V2.5-Evo - 2026-09-19 - The motor rise-limit (the "motor ramp"), pure and host-testable.
// V2.5-Evo - 2026-09-24 - REWRITTEN: this is now a SINGLE-CHANNEL ramp on the THROTTLE (0-255 command
//   counts), not a two-channel ramp on the finished PWM microseconds, and it accumulates in Q12 fixed
//   point. Two things forced the change and both are safety-shaped:
//
//   (1) WHERE IT RUNS. The old version rise-limited PWM0_time and PWM1_time - the two finished motor
//       outputs - and on a differential drive the DIFFERENCE between those two outputs IS the turn, so
//       it slewed every steering input as well (on steering_type 2 it slewed the steering servo itself,
//       which carries no throttle at all). The owner's standing rule is that steering is never ramped:
//       the ramp exists for the throttle, for a soft tow start that pulls a rider off a shoulder
//       without snatching the rope, and a turn must land on the very next 10 ms pass. calcPWM() now
//       calls this BEFORE the differential mixer, on the throttle, so the mixer splits a rise-limited
//       throttle with no limit of its own.
//
//   (2) RESOLUTION. 0-255 is roughly four times coarser than the ~1000-count PWM span the old ramp
//       worked in, and a whole-count step could not express a slow ramp at a 10 ms tick: it floors at
//       1 count = 2.55 s, so every setting from 1.275 s up collapsed onto one ramp and 4.00 s ran 36 %
//       FAST - the unsafe direction. The memory therefore holds 1/4096 of a count (Q12) in a uint32_t.
//       Full scale is 255 x 4096 = 1044480; the step is that over (ramp_s x tick_hz), TRUNCATED and
//       never rounded, so the error is only ever on the slow side. Swept 0.20-4.00 s in 0.01 s
//       increments: worst case +1 tick (+10 ms), zero cases faster than configured.
//
//   The contract is otherwise the one the two-channel version had, and the host test still checks it:
//     RISE     - per tick the throttle may climb by at most one step, and never past the target.
//     FALL     - instant. A lower target is taken on the same tick, so a trigger release, the
//                failsafe, an RTM/FM emergency stop and a straightening all drop the motors at once.
//                This is also the ONLY downward path, which is why the ramp can neither stall nor jump.
//     CONTINUITY across a rate change - the memory holds the last OUTPUT and the step is recomputed on
//                every tick from whatever ramp_s the caller passes, so switching from one ramp to the
//                other mid-rise continues from the current throttle at the new rate: no jump either
//                way. The RX selects between the slow manual motor_ramp_s and the automatic modes'
//                auto_ramp_s exactly this way.
//     OFF      - ramp_s at or below 0.001 passes the target straight through AND the memory keeps
//                tracking it. Before this header the RX skipped the whole block while the ramp was
//                off, so the memory went stale and a later non-zero ramp resumed from an old value
//                (a momentary dip). Tracking on every tick closes that.
//     INIT     - none needed. The stopped state in this domain is throttle 0, which is the zero-init
//                value of the memory, so the first squeeze after boot behaves like every later one.
//                (The old two-channel version needed a seed flag only because its stopped state was
//                PWM_min, not 0.)
//   Nothing here reads config or time: the caller passes the target, the ramp and the tick rate, maps
//   the result into each channel's PWM range, and applies the terminal effective_thr == 0 clamp AFTER
//   this - that clamp stays the last writer.
#ifndef BREMOTE_OUTPUT_RAMP_H
#define BREMOTE_OUTPUT_RAMP_H

#include <stdint.h>

// One throttle count in the accumulator's units, and full scale (throttle 255).
#define THROTTLE_RAMP_Q12_ONE   4096UL
#define THROTTLE_RAMP_Q12_FULL  (255UL * THROTTLE_RAMP_Q12_ONE)   // 1044480

// The ramp's memory: the last OUTPUT throttle, in 1/4096 of a count. Zero-init is the stopped state.
struct OutputRampState {
  uint32_t out_q12;
};

// throttleRampStepQ12 - how far the throttle may rise in one tick, in 1/4096 of a count.
// Inputs:  ramp_s - the 0 -> full rise time in seconds (caller guarantees > 0 here);
//          tick_hz - how often the caller runs (100 for the RX's generatePWM task).
// Returns: the per-tick step, at least 1 unit and at most full scale. Side effects: none.
// Truncated, never rounded: a truncated step can only make the ramp slower than configured, and
// slower is the safe direction. The 1-unit floor is the successor of the old whole-count floor and
// exists only so a corrupt out-of-range config value can never stall the ramp at zero; at 1/4096 of a
// count it is unreachable in the validated 0.2-4.0 s range, where the step runs 2611..52224. A step
// at or above full scale means "shorter than one tick", i.e. instant.
static inline uint32_t throttleRampStepQ12(float ramp_s, float tick_hz)
{
  const float st = (float)THROTTLE_RAMP_Q12_FULL / (ramp_s * tick_hz);
  if (!(st > 0.0f))                              return 1UL;   // also catches NaN
  if (st >= (float)THROTTLE_RAMP_Q12_FULL)       return THROTTLE_RAMP_Q12_FULL;
  const uint32_t s = (uint32_t)st;
  return (s == 0UL) ? 1UL : s;
}

// throttleRampStep - advance the throttle by one tick.
// Inputs:  s - the memory (updated in place); target - this tick's permitted throttle, 0-255, AFTER
//          every cap; ramp_s - the rise time in force on this tick (<= 0.001 = off); tick_hz.
// Returns: the rise-limited throttle to feed the mixer, 0-255. Always <= target, so every throttle
//          cap the caller applied before this still bounds the output.
// Side effects: only on *s.
static inline uint8_t throttleRampStep(OutputRampState* s, uint8_t target, float ramp_s, float tick_hz)
{
  const uint32_t target_q12 = (uint32_t)target * THROTTLE_RAMP_Q12_ONE;

  if (ramp_s > 0.001f) {
    const uint32_t step = throttleRampStepQ12(ramp_s, tick_hz);
    if (target_q12 > s->out_q12 + step) s->out_q12 += step;
    else                                s->out_q12 = target_q12;   // instant fall; the only way down
  } else {
    s->out_q12 = target_q12;   // ramp off: pass through, and keep tracking so a later ramp has no dip
  }

  return (uint8_t)(s->out_q12 / THROTTLE_RAMP_Q12_ONE);
}

#endif
