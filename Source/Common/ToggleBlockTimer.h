// V2.5-Evo - 2026-10-21 - The remote's "toggle is steering" wait, timed in real milliseconds. Pure and host-testable
//   (Tools/tests/toggle_block_timer_test.cpp). Owner-ordered (option B: "make the setting mean real time").
//
// ============================================================
// TOGGLE BLOCK TIMER - when may the thumb toggle go back to being a gear / menu button after a ride?
// ============================================================
//
// WHAT IT DOES
//   On the remote the thumb toggle has two jobs. While the trigger is squeezed (and steering is on) it STEERS the
//   buggy. With the trigger released it is the gear / cap / menu / gesture button. Right after a ride the rider is
//   often still steering, so the toggle must not turn back into a button the instant the trigger is let go:
//   a steering move would otherwise become a gear change, a lock or a Follow-Me / Return-To-Me gesture.
//   The toggle therefore stays steering until BOTH of these have held, without a break, for usrConf.tog_block_time
//   x 10 ms: the trigger released, and the toggle resting in its centre deadband. Any squeeze, and any toggle
//   movement out of the centre, restarts the wait. A remote error frees the toggle at once (it is needed to dismiss
//   the error). tog_block_time 0 means "never free it": once the trigger has been squeezed the toggle stays
//   steering (inherited behaviour, kept as is).
//
// WHAT CHANGED (V2.5-Evo - 2026-10-21)
//   calcFilter() (Hall.ino) used to count its own passes: +1 per pass of the 10 ms ADC task while the toggle was
//   centred, unlock at tog_block_time passes. That is only real time if every pass is exactly 10 ms apart, and
//   nothing guaranteed that. The owner observed about 5-6 s at the default 200 and chose to have the setting mean
//   real time. (Whether the passes ran slow or the wait kept restarting was not measured on the remote.) The wait
//   is now timed with millis(), so 200 is 2.0 s and 300 is 3.0 s whatever the pass rate. The unit (x 10 ms), the field,
//   its offset and its range are unchanged: no confStruct change, no SW_VERSION bump, no config reset.
//   The hold, reset, error and 0 rules are exactly the old ones; only "how long" is measured differently.
//
// Nothing here reads config, globals or the clock: the caller passes them in. Called once per calcFilter() pass, on
// the measBufCalc task only (the only writer of the state).
#ifndef BREMOTE_TOGGLE_BLOCK_TIMER_H
#define BREMOTE_TOGGLE_BLOCK_TIMER_H

#include <stdint.h>

// The wait's memory. Zero-init is fine: "not served yet, started at 0".
struct ToggleBlockTimer {
  uint32_t since_ms;   // millis() of the last pass that restarted the wait (a squeeze, or the toggle off centre)
  bool     done;       // true once the wait has been served (or a remote error freed the toggle); cleared by a squeeze
};

// toggleBlockStep - one calcFilter() pass of the toggle block.
// Inputs:
//   t            - the timer memory (read and updated)
//   blocked      - the current block flag (toggle_blocked_by_steer): true = the toggle steers
//   hold         - the existing block condition: trigger squeezed with steering on and no menu / lock, or the forced
//                  block of throttle_mode 1 + no_lock 1
//   remote_error - a remote error is showing
//   centred      - steer_scaled == 127 on this pass, i.e. the toggle sits inside its tog_deadzone band
//   block_10ms   - usrConf.tog_block_time, in units of 10 ms (0 = never free the toggle)
//   now_ms       - millis()
// Output: the new block flag. Side effects: updates *t only.
// Wrap-safe: the elapsed time is an unsigned subtraction, so the millis() roll-over after 49.7 days is harmless.
static inline bool toggleBlockStep(ToggleBlockTimer *t, bool blocked, bool hold, bool remote_error,
                                   bool centred, uint16_t block_10ms, uint32_t now_ms)
{
  // Trigger squeezed: the toggle steers, and the wait starts again from now.
  if (hold)
  {
    t->since_ms = now_ms;
    t->done     = false;
    return true;
  }

  // A remote error is showing: free the toggle at once and count the wait as served (old code: counter = limit).
  if (remote_error)
  {
    t->done = true;
    return false;
  }

  // Trigger released. Until the wait is served, any toggle movement out of the centre restarts it; once the toggle
  // has stayed centred for block_10ms x 10 ms the wait is served. After that the toggle is a button again and its
  // movements no longer matter here - only a new squeeze (hold) blocks it again. 0 is never served.
  if (!t->done)
  {
    if (!centred)
    {
      t->since_ms = now_ms;
    }
    else if (block_10ms != 0u && (uint32_t)(now_ms - t->since_ms) >= (uint32_t)block_10ms * 10u)
    {
      t->done = true;
    }
  }

  if (t->done && block_10ms != 0u) return false;
  return blocked;
}

#endif
