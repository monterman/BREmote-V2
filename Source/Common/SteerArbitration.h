// V2.5-Evo - 2026-09-19 - The stick during automatic steering: the TAKEOVER arbitration, pure and host-testable.
//   Whenever the buggy steers itself - Follow-Me following, auto-return (FM_RETURN) or a classic
//   return-to-me - the rider's steering stick is thrown away and, today, a held push CANCELS the
//   automatic steering. The RX setting steer_during_auto selects a second behaviour: the stick TAKES
//   OVER the steering byte while it is deflected and hands it back when centred, cancelling nothing.
//   This header decides WHEN a takeover stands. It is called once per 10 Hz tick by the mode that owns
//   the steering (RX RTMState.ino), with that mode's own grace clock, and the result is published to
//   calcPWM() through one atomic - the 100 Hz motor path never decides anything itself.
//     ENGAGE   - the stick has been read within release_deadband (20) of centre at least once since
//                the owner started ("centre-seen": a remote whose rest position has drifted can never
//                take over silently), the owner's grace (2 s from the start of its motion, the same
//                grace the cancel uses) has passed, and the stick has been at or beyond
//                engage_deadband (40 counts from 127) for engage_persist_ms (500 ms) without a break.
//     ACTIVE   - the stick steers. A byte between the two deadbands (20-39) neither engages nor
//                releases: that is the hysteresis that keeps chop from flipping the source.
//     RELEASE  - the stick has been inside release_deadband for release_persist_ms (200 ms): the
//                automatic steering resumes. Nothing was cancelled.
//     TIMEOUT  - a takeover that stands for max_active_ms (20 s) ends; the caller runs its CANCEL
//                path (a rider who has steered for 20 s is driving manually, and a stick that never
//                comes back to centre is most likely a drifted centre). Centre-seen is cleared, so
//                the stick must be read centred again before another takeover.
//     NO OWNER - no mode is steering, the trigger is below 25 counts, or steering override is off:
//                the memory is zeroed and the verdict is INACTIVE. Every stop, every release of the
//                trigger and every change of owner therefore ends a takeover within one tick.
//   Unsigned subtraction survives a millis() wrap; a timestamp of 0 is stored as 1 because 0 means
//   "not running". Tools/tests/steer_arbitration_test.cpp runs the exact code the RX runs, and also
//   shows that with the setting at 0 the steering-source selector calcPWM() uses is byte-identical to
//   the one it used before this header existed.
#ifndef BREMOTE_STEER_ARBITRATION_H
#define BREMOTE_STEER_ARBITRATION_H

#include <stdint.h>

// The arbitration's tuning, passed in by the caller (the RX's kSteerTakeover* constants).
struct SteerTakeoverParams {
  uint8_t  engage_deadband;     // counts from 127 at or beyond which the stick counts as deflected (40)
  uint8_t  release_deadband;    // counts from 127 below which the stick counts as centred (20)
  uint32_t engage_persist_ms;   // the deflection must stand this long to engage (500)
  uint32_t release_persist_ms;  // the centre must stand this long to release (200)
  uint32_t grace_ms;            // no engage this long after the owner's motion started (2000)
  uint32_t max_active_ms;       // a takeover standing this long times out into the cancel path (20000)
};

// The arbitration's memory between ticks. All zero = nothing seen, nothing standing.
struct SteerTakeoverState {
  uint32_t deflect_since_ms;    // millis() the stick was first read at/beyond engage_deadband; 0 = not deflected
  uint32_t centre_since_ms;     // millis() the stick was first read inside release_deadband; 0 = not centred
  uint32_t active_since_ms;     // millis() the takeover engaged; 0 = not active
  bool     centre_seen;         // the stick has been read centred at least once for this owner
  bool     active;              // a takeover stands: calcPWM() applies the stick, not the controller
};

// What one tick decided. The two edges and the timeout are returned exactly once each.
enum SteerTakeoverVerdict : uint8_t {
  STO_INACTIVE  = 0,   // no takeover (no owner, or the stick has not earned one)
  STO_ACTIVE    = 1,   // a takeover stands (not the first tick)
  STO_ENGAGED   = 2,   // the takeover engaged on THIS tick (edge)
  STO_RELEASED  = 3,   // the takeover released on THIS tick (edge): the controller resumes
  STO_TIMED_OUT = 4    // the takeover timed out on THIS tick (edge, once): the caller must CANCEL
};

// steerTakeoverDeflection - how far the stick is from centre, in counts.
// Inputs: the steering byte (0-255, 127 = centre). Returns: |byte - 127|, 0-128. Side effects: none.
static inline uint8_t steerTakeoverDeflection(uint8_t steering_byte)
{
  return (steering_byte >= 127) ? (uint8_t)(steering_byte - 127) : (uint8_t)(127 - steering_byte);
}

// steerTakeoverStep - advance the arbitration by one tick.
// Inputs:  s - the memory (updated in place); now_ms - millis(); owner_active - a mode is applying
//          its steering override on this tick (mode steering AND trigger >= 25 AND override enabled);
//          owner_since_ms - millis() the owner's current motion began (its grace base), 0 = no motion
//          clock yet, which can never engage; steering_byte - the stick as received; p - the tuning.
// Returns: one SteerTakeoverVerdict. STO_ENGAGED / STO_RELEASED / STO_TIMED_OUT are single-tick edges;
//          s->active is the level the caller publishes.
// Side effects: only on *s. No owner zeroes everything (a takeover never survives a stop, a trigger
//          release or a change of owner). A release keeps centre_seen (the stick IS centred); a
//          timeout clears it (the stick must be proven centred again).
static inline uint8_t steerTakeoverStep(SteerTakeoverState* s, uint32_t now_ms, bool owner_active,
                                        uint32_t owner_since_ms, uint8_t steering_byte,
                                        const SteerTakeoverParams* p)
{
  if (!owner_active) {
    s->deflect_since_ms = 0;
    s->centre_since_ms  = 0;
    s->active_since_ms  = 0;
    s->centre_seen      = false;
    s->active           = false;
    return STO_INACTIVE;
  }

  const uint32_t stamp = (now_ms != 0) ? now_ms : 1u;   // 0 is the "not running" sentinel
  const uint8_t  dev   = steerTakeoverDeflection(steering_byte);

  // Where the stick is this tick: centred, deflected, or in the hysteresis band between the two.
  if (dev < p->release_deadband) {
    s->centre_seen = true;
    if (s->centre_since_ms == 0) s->centre_since_ms = stamp;
    s->deflect_since_ms = 0;
  } else if (dev >= p->engage_deadband) {
    if (s->deflect_since_ms == 0) s->deflect_since_ms = stamp;
    s->centre_since_ms = 0;
  } else {
    s->deflect_since_ms = 0;   // between the bands: neither timer runs
    s->centre_since_ms  = 0;
  }

  if (!s->active) {
    const bool grace_served = (owner_since_ms != 0) && ((uint32_t)(now_ms - owner_since_ms) >= p->grace_ms);
    const bool persisted    = (s->deflect_since_ms != 0) &&
                              ((uint32_t)(now_ms - s->deflect_since_ms) >= p->engage_persist_ms);
    if (s->centre_seen && grace_served && persisted) {
      s->active          = true;
      s->active_since_ms = stamp;
      return STO_ENGAGED;
    }
    return STO_INACTIVE;
  }

  // Active: release on a sustained centre, else time out, else keep standing.
  if (s->centre_since_ms != 0 && (uint32_t)(now_ms - s->centre_since_ms) >= p->release_persist_ms) {
    s->active           = false;
    s->active_since_ms  = 0;
    s->deflect_since_ms = 0;
    return STO_RELEASED;
  }
  if ((uint32_t)(now_ms - s->active_since_ms) >= p->max_active_ms) {
    s->active           = false;
    s->active_since_ms  = 0;
    s->deflect_since_ms = 0;
    s->centre_since_ms  = 0;
    s->centre_seen      = false;   // must be read centred again before the next takeover
    return STO_TIMED_OUT;
  }
  return STO_ACTIVE;
}

// steerTakeoverSelectsStick - the steering-source selector calcPWM() applies, as one expression.
// Inputs:  auto_owner - the existing test ((rtm_rx_active || fm_rx_active) && override enabled &&
//          thr >= 25); takeover_active - the published atomic. Returns: true when the STICK byte is
//          applied, false when the controller's override byte is. Side effects: none.
// With takeover_active always false (the setting at 0) this is exactly !auto_owner, the selector
// calcPWM() has always used; the host test proves it sample by sample.
static inline bool steerTakeoverSelectsStick(bool auto_owner, bool takeover_active)
{
  return !(auto_owner && !takeover_active);
}

#endif
