// V2.5-Evo - 2026-10-07 - PARKED AUTO-RETURN (audit N-9, N-10, N-13 + owner rule "waiting means it will work"):
//   fmReturnGapStep() (no return motion after a tolerated link gap until a boot ID is heard again),
//   fmReturnParkVerdict() (go / not-ready / fault for a parked return), fmReturnStallIsArrival() (a stall is an
//   arrival only in the final crawl).
// V2.5-Evo - 2026-10-07 - LINK-TIME CLOCKS (audit N-8(b), N-12): stickyWindowStep() holds a sticky telemetry bit
//   for 6 s of LINK-FRESH time; rtmRefreshExpiredOnLink() ends RTM only after the link has been fresh for the
//   whole expiry with no refresh.
// V2.5-Evo - 2026-10-07 - kRtmEndHandbackCap (audit N-2, N-3, N-4): every end of manual return-to-me (Gate 9,
//   Phase C, the refresh expiry, the gate-fault timeout, a remote reboot) arms the hand-back cap at 0.
// V2.5-Evo - 2026-10-07 - SIGNED STAMP AGES (audit N-1, N-6, N-7): rtmRefreshExpired() treats a refresh stamped
//   AFTER the caller read the clock (age <= 0) as fresh instead of 49 days old; new stampAgeMs() / stampStale()
//   clamp a negative age to 0 for every loop-task comparison against a stamp the radio task writes; new
//   fmFailingMaskFrom() is the pure, host-tested core of the RX's failing-conditions mask.
// V2.5-Evo - 2026-10-07 - Auto-return / return-to-me END RULES (audits S-1..S-8, A-1, H-1, H-2, M-1, D-1 and the
//   owner's "arrival hand-back keeps a throttle cap" rule). Pure, header-only, no Arduino dependencies, so the host
//   unit test Tools/tests/auto_return_rules_test.cpp exercises the exact code the RX runs. Every function here is a
//   DECISION: it reads numbers and returns a verdict. None of them writes a motor output; the RX applies the verdicts
//   through its existing subtract-only chain (calcPWM(): effective = min(trigger, every cap)).
//
//   1. handbackCapStep        - the arrival hand-back cap: the mode ends, its cap stays until ONE full release.
//   2. rtmGateFaultStep        - H-2: RTM gates 2-7 failing ~1.5 s under a held trigger end RTM instead of
//                                leaving it half on with the motor dead.
//   3. txBootIdStep            - S-8: the remote's random boot ID; a change means the remote was switched off and on.
//   4. rtmRefreshExpired       - H-1: the RX ends RTM when a remote that PROVED it refreshes stops refreshing.
//   5. distBlankStep           - D-1: the distance telemetry byte goes to "unknown" (0xFF) when its inputs are stale.
//   6. fmReturnParkedTolerates - S-2: a PARKED auto-return waits through a silent / stale remote.
//   7. fmReturnPivotSuspend    - S-6: the not-closing net stays parked while the heading error is still falling.
//   8. fmReturnCandidateMayForm- S-3: the auto-return candidate may form from ARMED after an engagement this run.
//   0. stampAgeMs / stampStale - N-1/N-6/N-7: the age of a stamp another task writes, never negative (defined first,
//                                because rtmRefreshExpired() uses it).
//   9. fmFailingMaskFrom       - N-6: the failing Follow-Me conditions 2-7 as a mask, from plain inputs.
#ifndef BREMOTE_AUTO_RETURN_RULES_H
#define BREMOTE_AUTO_RETURN_RULES_H

#include <stdint.h>

// ============================================================
// 0. THE AGE OF A STAMP ANOTHER TASK WRITES (audits N-1, N-6, N-7)
// ============================================================
// THE BUG: the loop task reads millis() once at the top of its 10 Hz tick (`now`), and the radio task
// (higher priority) can preempt it afterwards and stamp a timestamp with a LATER millis() - last_packet,
// rx_tx_gps_timestamp, fm_mode_last_rx_ms, rtm_refresh_last_ms. `now - stamp` in unsigned arithmetic then
// wraps to about 4.29 billion ms, so a packet that arrived a microsecond ago reads as 49 days stale: a false
// link loss, a false stale rider fix, a false 95 s mode expiry, a false RTM refresh expiry. On a 10 Hz link
// the window is hit every few seconds to minutes.
// THE FIX: take the difference as a SIGNED 32-bit number. A stamp newer than `now` gives a negative age,
// which is clamped to 0 - "just now", which is the truth. Differences up to 24.8 days stay correct, far
// beyond any session.
// stampAgeMs - Inputs: now_ms (the caller's clock), stamp_ms (millis() of the event). Returns: the age in
//   ms, 0 when the stamp is newer than now_ms. Side effects: none (pure).
static inline uint32_t stampAgeMs(uint32_t now_ms, uint32_t stamp_ms)
{
  const int32_t age = (int32_t)(now_ms - stamp_ms);
  return (age > 0) ? (uint32_t)age : 0u;
}

// stampStale - Inputs: now_ms; stamp_ms (0 = never happened); limit_ms. Returns: true when the event never
//   happened or is older than limit_ms (strictly, like every `age > limit` test it replaces). Side effects: none.
static inline bool stampStale(uint32_t now_ms, uint32_t stamp_ms, uint32_t limit_ms)
{
  if (stamp_ms == 0) return true;
  return stampAgeMs(now_ms, stamp_ms) > limit_ms;
}

// ============================================================
// 1. THE ARRIVAL HAND-BACK CAP
// ============================================================
// The owner's rule (2026-10-07): manual return-to-me (Gate 9) and auto-return end the same way on
// arrival. The MODE ends and steering goes back to the rider at once, but the throttle cap in force
// at arrival (the approach cap, near 0 at the stop distance) STAYS until the rider lets go of the
// trigger fully once. A rider still at 100 % cannot ram the buggy into himself; after one full
// release the throttle is plain manual.
//
// kHandbackNone is "no hand-back cap standing" (255 = no cap anywhere in calcPWM()).
static const uint8_t kHandbackNone = 255;

// handbackCapStep - apply a standing hand-back cap to this tick's throttle and say whether the
// rider's full release has now cleared it.
// Inputs:  cap         - the standing cap (kHandbackNone = none).
//          effective   - the throttle after every other cap this tick (0-255).
//          trigger     - the raw trigger byte from the remote (thr_received).
//          release_below - the trigger is "fully released" strictly below this many counts.
// Outputs: *clear_out  - true when the cap must be cleared (the rider let go fully this tick).
// Returns: the throttle after the cap: min(effective, cap). On the releasing tick the cap is still
//          applied (the trigger is already below release_below, so it cannot matter), and it is gone
//          from the next tick on.
// Side effects: none (pure). Subtract-only by construction: the result is never above `effective`.
static inline uint8_t handbackCapStep(uint8_t cap, uint8_t effective, uint8_t trigger,
                                      uint8_t release_below, bool* clear_out)
{
  *clear_out = false;
  if (cap == kHandbackNone) return effective;
  if (trigger < release_below) *clear_out = true;
  return (cap < effective) ? cap : effective;
}

// V2.5-Evo - 2026-10-07 - audits N-2, N-3, N-4: EVERY END OF MANUAL RETURN-TO-ME ARMS THE HAND-BACK CAP AT 0.
// The RX used to arm "the cap in force" at some RTM ends and nothing at others:
//   - Phase C (N-2) armed nothing: the emergency stop was dropped on the next tick and the held trigger came
//     back with no cap - against the documented rx_state_flags bit 0 contract;
//   - Gate 9 (N-3), the remote reboot and the refresh expiry (N-4) armed rtm_approach_cap, which is 255 when
//     the approach zone is 0 or smaller than the stop distance, or was already reset to 255 that tick.
// One number for all of them (Gate 9, Phase C, H-1, H-2, S-8): 0. The rider lets go fully once, then the
// throttle is plain manual. Auto-return arrival keeps its own rule (the cap in force, near 0 at the stop radius).
static const uint8_t kRtmEndHandbackCap = 0;

// ============================================================
// 2. H-2: RTM GATES 2-7 FAILING UNDER A HELD TRIGGER END RTM
// ============================================================
// Gates 2-7 used to raise the emergency stop and leave rtm_rx_active TRUE, so RTM stayed half on
// with the motor dead and the remote still showing RTM. Feathering the trigger never escaped it.
// The fix ends RTM after limit_ms of failing gates with the trigger held. "Continuously" here means
// "with no PASSING evidence in between": a release PAUSES the count (gates are not evaluated while
// the trigger is released) and a squeeze picks it up again, so feathering cannot reset it - only a
// tick on which the gates pass under a held trigger resets it.
struct RtmGateFaultState {
  uint32_t accum_ms;   // held-and-failing time accumulated since the last passing tick
  uint32_t last_ms;    // millis() of the previous held-and-failing tick; 0 = the previous tick was not one
};

// rtmGateFaultStep - one 10 Hz tick of the H-2 accumulator.
// Inputs:  now_ms; thr_held (trigger >= 25); gates_failing (a gate 2-7 failed THIS tick, which can
//          only be known while the trigger is held); limit_ms (1500); max_dt_ms bounds one step so a
//          stalled loop cannot jump the count (200 ms = two ticks).
// Returns: true on the tick the accumulated time reaches limit_ms (the caller ends RTM).
// Side effects: updates *s only.
static inline bool rtmGateFaultStep(RtmGateFaultState* s, uint32_t now_ms, bool thr_held,
                                    bool gates_failing, uint32_t limit_ms, uint32_t max_dt_ms)
{
  if (!thr_held) {                 // released: pause, do not reset
    s->last_ms = 0;
    return false;
  }
  if (!gates_failing) {            // held and passing: real evidence the gates are fine
    s->accum_ms = 0;
    s->last_ms  = 0;
    return false;
  }
  const uint32_t stamp = (now_ms != 0) ? now_ms : 1u;
  if (s->last_ms != 0) {
    uint32_t dt = (uint32_t)(stamp - s->last_ms);
    if (dt > max_dt_ms) dt = max_dt_ms;
    s->accum_ms += dt;
  }
  s->last_ms = stamp;
  return s->accum_ms >= limit_ms;
}

static inline void rtmGateFaultReset(RtmGateFaultState* s) { s->accum_ms = 0; s->last_ms = 0; }

// ============================================================
// 3. S-8: THE REMOTE'S BOOT ID
// ============================================================
// WIRE FORMAT (designed here; the remote side is not built yet). The remote picks a random 7-bit ID
// at every power-on and sends it as an 0xF1 meta packet with the VALUE byte 0x80 | id (0x80-0xFF).
// Why 0xF1 and those values: every RX firmware in the field handles 0xF1 values 0 and 1 only and
// ignores every other value (processRtmStatePacket), so an older RX simply ignores the ID - and a new
// type byte is NOT safe, because an old RX reads any unknown type byte as a THROTTLE value.
// The RX keeps the last ID it heard (kTxBootIdNone until the first one). A DIFFERENT ID means the
// remote was switched off and on: the RX cancels any standing return (SOP-040 rule 5).
static const uint8_t kTxBootIdNone  = 0xFF;   // no ID heard since the RX booted
static const uint8_t kTxBootIdFlag  = 0x80;   // 0xF1 value bit 7 = "this is a boot ID"

static inline bool txBootIdIsBootIdValue(uint8_t f1_value) { return (f1_value & kTxBootIdFlag) != 0; }

// txBootIdStep - fold one received boot-ID value into the stored ID.
// Inputs:  *stored (kTxBootIdNone = none yet); f1_value (the 0xF1 value byte, 0x80-0xFF).
// Returns: true when the remote REBOOTED (an ID was stored and this one differs). The first ID ever
//          heard is stored and returns false - there is nothing to compare it with.
// Side effects: *stored becomes this ID.
static inline bool txBootIdStep(uint8_t* stored, uint8_t f1_value)
{
  const uint8_t id = (uint8_t)(f1_value & 0x7F);
  const bool changed = (*stored != kTxBootIdNone) && (*stored != id);
  *stored = id;
  return changed;
}

// ============================================================
// 4. H-1: RTM REFRESH EXPIRY (behind a remote that proves it refreshes)
// ============================================================
// WIRE FORMAT (designed here; the remote side is not built yet). While RTM is ACTIVE the remote
// re-sends 0xF1 with the VALUE 0x02 ("RTM still active") about once a second. Old RX firmware
// ignores value 2. On the new RX a refresh never ACTIVATES RTM (only 0xF1/1 does), so a refresh that
// arrives after the buggy ended RTM by itself cannot switch it back on.
// The expiry only applies to an RTM run in which at least one refresh has been heard: today's remote
// sends 0xF1/1 once and never refreshes, and an unconditional expiry would kill every legitimate
// return after N seconds.
static const uint8_t kRtmRefreshValue = 0x02;

// rtmRefreshExpired - has a refreshing remote gone quiet?
// Inputs:  rtm_active; refresh_seen (a refresh arrived during THIS run); last_refresh_ms; now_ms;
//          expiry_ms.
// Returns: true when RTM must end. Side effects: none (pure).
// V2.5-Evo - 2026-10-07 - N-1: the age is SIGNED (stampAgeMs). The radio task stamps last_refresh_ms and can
// do so after the loop read now_ms; the unsigned difference then wrapped and the run "expired" at once -
// roughly every 0.5-2 min once a remote refreshes. An age of 0 or less is fresh.
static inline bool rtmRefreshExpired(bool rtm_active, bool refresh_seen, uint32_t last_refresh_ms,
                                     uint32_t now_ms, uint32_t expiry_ms)
{
  if (!rtm_active || !refresh_seen) return false;
  return stampAgeMs(now_ms, last_refresh_ms) > expiry_ms;
}

// rtmRefreshExpiredOnLink - V2.5-Evo - 2026-10-07 - audit N-12: the expiry counts only time the link is UP.
// rtmRefreshExpired() alone also counted link-down time, so a remote behind a wave for a few seconds came back
// to an RTM already ended - the refresh it was about to send had no chance to arrive. The expiry is for "other
// packets keep arriving but the refresh does not" (the remote left RTM and its 0xF1/0 was lost), so it now
// needs BOTH: no refresh for expiry_ms, AND the link fresh without a break for expiry_ms.
// Inputs: as rtmRefreshExpired(), plus link_fresh (last_packet within failsafe_time this tick) and
//         link_fresh_since_ms (millis() the link became fresh again; 0 = not fresh).
// Returns: true when RTM must end. Side effects: none (pure).
static inline bool rtmRefreshExpiredOnLink(bool rtm_active, bool refresh_seen, uint32_t last_refresh_ms,
                                           uint32_t now_ms, uint32_t expiry_ms,
                                           bool link_fresh, uint32_t link_fresh_since_ms)
{
  if (!link_fresh || link_fresh_since_ms == 0) return false;
  if (stampAgeMs(now_ms, link_fresh_since_ms) <= expiry_ms) return false;
  return rtmRefreshExpired(rtm_active, refresh_seen, last_refresh_ms, now_ms, expiry_ms);
}

// ============================================================
// 4b. N-8(b): A STICKY TELEMETRY BIT COUNTS DOWN ONLY WHILE THE LINK IS UP
// ============================================================
// fm_flags bit 3 and rx_state_flags bits 0 / 1 are held for 6 s so the remote, which receives one telemetry
// byte per control packet (a full rotation is about 2 s), cannot miss them. But the clock ran during a link
// loss too, and the commonest fault - a link-loss fault - happens exactly then: by the time the link came
// back the 6 s were gone and the remote never saw "St" (audit N-8). Now the window counts down only on ticks
// with a fresh link, so the remote gets 6 s of LINK time to read it.
struct StickyWindowState {
  uint32_t seen_stamp;   // the alarm stamp this window was started for (0 = none yet)
  uint32_t left_ms;      // link-fresh time still to show the bit (0 = not showing)
  uint32_t last_ms;      // millis() of the previous step
};

// stickyWindowStep - one tick. Inputs: now_ms; alarm_stamp_ms - the millis() the alarm was raised (the
//   existing *_alarm_ms stamp; 0 = never; a DIFFERENT non-zero value starts a fresh full window);
//   link_fresh; window_ms. Returns: true while the bit must be set. Side effects: updates *s only.
static inline bool stickyWindowStep(StickyWindowState* s, uint32_t now_ms, uint32_t alarm_stamp_ms,
                                    bool link_fresh, uint32_t window_ms)
{
  const uint32_t dt = (s->last_ms != 0) ? (uint32_t)(now_ms - s->last_ms) : 0u;
  s->last_ms = (now_ms != 0) ? now_ms : 1u;
  if (alarm_stamp_ms != 0 && alarm_stamp_ms != s->seen_stamp) {
    s->seen_stamp = alarm_stamp_ms;
    s->left_ms    = window_ms;
    return window_ms > 0;
  }
  if (s->left_ms == 0) return false;
  if (link_fresh) s->left_ms = (dt >= s->left_ms) ? 0u : (s->left_ms - dt);
  return s->left_ms > 0;
}

// ============================================================
// 5. D-1: THE DISTANCE TELEMETRY BYTE GOES TO "UNKNOWN" WHEN ITS INPUTS ARE STALE
// ============================================================
// The RX used to keep the last distance forever when either GPS went stale, so the remote showed a
// frozen number with no warning (SOP-041 rule 4: old data is never shown as live). It also must not
// blank on every short hiccup (that is why the active write of 0xFF was taken out once: the FM bar
// went dark). So: blank after stale_ms with no valid distance, and once blanked, publish again only
// after the inputs have been valid continuously for recover_ms (the hysteresis). The RX passes
// stale_ms = 0: its "valid" already allows a rider fix up to 10 s old, which is the hiccup tolerance;
// another 10 s on top kept a frozen number up for about 20 s.
struct DistBlankState {
  uint32_t last_good_ms;   // millis() of the last tick with a valid distance; 0 = none since boot
  uint32_t good_since_ms;  // while blanked: millis() the inputs became valid again; 0 = not valid
  bool     blanked;        // the byte is held at 0xFF
};

// distBlankStep - one tick of the distance-byte freshness rule.
// Inputs:  now_ms; inputs_valid (both GPS sources fresh this tick); stale_ms (10000); recover_ms (1000).
// Returns: true when the caller must publish 0xFF this tick; false = publish the real distance (when
//          inputs_valid) or leave the byte as it is (when not valid and not yet stale).
// Side effects: updates *s only. Starts blanked (initialise blanked = true), so the very first
// distance after boot is published recover_ms after the inputs first become valid.
static inline bool distBlankStep(DistBlankState* s, uint32_t now_ms, bool inputs_valid,
                                 uint32_t stale_ms, uint32_t recover_ms)
{
  const uint32_t stamp = (now_ms != 0) ? now_ms : 1u;
  if (s->blanked) {
    if (!inputs_valid) { s->good_since_ms = 0; return true; }
    if (s->good_since_ms == 0) s->good_since_ms = stamp;
    if ((uint32_t)(stamp - s->good_since_ms) >= recover_ms) {
      s->blanked       = false;
      s->good_since_ms = 0;
      s->last_good_ms  = stamp;
      return false;
    }
    return true;
  }
  if (inputs_valid) { s->last_good_ms = stamp; return false; }
  if (s->last_good_ms == 0 || (uint32_t)(stamp - s->last_good_ms) > stale_ms) {
    s->blanked       = true;
    s->good_since_ms = 0;
    return true;
  }
  return false;
}

// ============================================================
// 6. S-2: A PARKED AUTO-RETURN WAITS THROUGH A SILENT OR STALE REMOTE
// ============================================================
// Failing Follow-Me conditions as bits (condition numbers from checkFmFaultConditions()).
static const uint8_t kFmCondPhaseA  = 1u << 0;   // 2: the buggy's own GPS rejected
static const uint8_t kFmCondPhaseB  = 1u << 1;   // 3: the TX<->RX handshake failing
static const uint8_t kFmCondTxStale = 1u << 2;   // 4: the rider's (remote's) GPS stale
static const uint8_t kFmCondRxStale = 1u << 3;   // 5: the buggy's own GPS stale
static const uint8_t kFmCondHeading = 1u << 4;   // 6: no heading source
static const uint8_t kFmCondLink    = 1u << 5;   // 7: the radio link down

// fmReturnParkedTolerates - may a standing auto-return stay PARKED through these failures?
// Inputs:  failing_mask - the failing conditions this tick (0 = all hold);
//          parked       - no held-trigger motion is under way (fm_return_motion_ms == 0);
//          thr_held     - the trigger is held this tick;
//          link_tolerance_allowed - the remote sends a boot ID (S-8), so a remote switched off and on
//                         is recognised and cancels the return by itself. Without that, a link loss
//                         keeps ending the return exactly as before, because the link-loss fault is
//                         today's ONLY way a remote reboot cancels a parked return.
// Returns: true = stay parked (no fault, no motion); false = the caller ends the return on a fault.
// THE RULE: only the REMOTE side may be missing - its GPS (4), the link (7), and the handshake (3),
// which the RX revokes by itself once the remote's GPS has been stale for 2x the stale timeout, so it
// follows 4 or 7 and is tolerated only together with one of them. The buggy's OWN sensors (2, 5, 6)
// still end the return. And MOTION still needs every input: a held trigger is never tolerated.
static inline bool fmReturnParkedTolerates(uint8_t failing_mask, bool parked, bool thr_held,
                                           bool link_tolerance_allowed)
{
  if (failing_mask == 0) return false;                 // nothing failing: not a tolerance question
  if (!parked || thr_held) return false;               // motion needs every input fresh
  if (failing_mask & (kFmCondPhaseA | kFmCondRxStale | kFmCondHeading)) return false;   // the buggy's own sensors
  if ((failing_mask & kFmCondLink) && !link_tolerance_allowed) return false;
  if ((failing_mask & kFmCondPhaseB) && !(failing_mask & (kFmCondTxStale | kFmCondLink))) return false;
  return true;
}

// ============================================================
// 7. S-6: THE NOT-CLOSING NET STAYS PARKED WHILE THE PIVOT IS STILL MAKING PROGRESS
// ============================================================
// The return's not-closing net used to be suspended while aligning for at most 15 s from the
// motion start; a slow pivot at the align cap could take longer and trip a false "St". Now the
// suspension continues past suspend_max_ms for as long as the heading error keeps FALLING (by at
// least eps_deg within every stall_ms), up to an absolute ceiling_ms, so a stalled or wrong-way
// pivot is still judged.
struct FmReturnPivotState {
  float    best_err_deg;   // the smallest heading error seen in this episode
  uint32_t progress_ms;    // millis() the error last improved by eps_deg; 0 = no episode
};

static inline void fmReturnPivotReset(FmReturnPivotState* s) { s->best_err_deg = 180.0f; s->progress_ms = 0; }

// fmReturnPivotSuspend - one tick. Inputs: now_ms; aligning (the error is above the align threshold);
// err_deg (|heading error|, 180 when unknown); judge_base_ms (the motion start, re-based on every
// takeover release); suspend_max_ms (15000), stall_ms (2000), eps_deg (2), ceiling_ms (45000).
// Returns: true while the net must stay parked for the pivot. Side effects: updates *s only.
static inline bool fmReturnPivotSuspend(FmReturnPivotState* s, uint32_t now_ms, bool aligning,
                                        float err_deg, uint32_t judge_base_ms,
                                        uint32_t suspend_max_ms, uint32_t stall_ms,
                                        float eps_deg, uint32_t ceiling_ms)
{
  if (!aligning) { fmReturnPivotReset(s); return false; }
  const uint32_t stamp = (now_ms != 0) ? now_ms : 1u;
  if (s->progress_ms == 0) {
    s->best_err_deg = err_deg;
    s->progress_ms  = stamp;
  } else if (err_deg < (s->best_err_deg - eps_deg)) {
    s->best_err_deg = err_deg;
    s->progress_ms  = stamp;
  }
  const uint32_t elapsed = (uint32_t)(stamp - judge_base_ms);
  if (elapsed < suspend_max_ms) return true;
  if (elapsed < ceiling_ms && (uint32_t)(stamp - s->progress_ms) < stall_ms) return true;
  return false;
}

// ============================================================
// 8. S-3: THE AUTO-RETURN CANDIDATE MAY FORM FROM ARMED AFTER AN ENGAGEMENT THIS RUN
// ============================================================
// The candidate used to form only in FM_ACTIVE or FM_HOLD, i.e. only if the rider stopped WITH THE
// TRIGGER HELD. The normal flow is "release, surf or stop, later squeeze" - and a release turns HOLD
// into ARMED (HOLD-ESCAPE-2) - so auto-return never started. Now it may also form from ARMED when
// Follow-Me has engaged since the declaration (engaged_this_run), and only beyond D_engage: a rider
// stopped close to the buggy (on the rope for the next tow) must never become a return candidate.
// Inputs: active_or_hold (fm_state is FM_ACTIVE or FM_HOLD), armed (fm_state is FM_ARMED),
//         engaged_this_run, dist_m, d_engage_m.
// Returns: true when a candidate may form this tick. Side effects: none (pure).
static inline bool fmReturnCandidateMayForm(bool active_or_hold, bool armed, bool engaged_this_run,
                                            float dist_m, float d_engage_m)
{
  if (active_or_hold) return true;
  return armed && engaged_this_run && (dist_m > d_engage_m);
}

// ============================================================
// 9. N-6: THE FAILING FOLLOW-ME CONDITIONS 2-7 AS A MASK, FROM PLAIN INPUTS
// ============================================================
// The RX's fmFailingConditionsMask() gathers the globals and calls this, so the host test runs the exact
// decision. THE BUG IT CLOSES (audit N-6): the mask used the loop tick's `now` against stamps the radio task
// writes; a stamp newer than `now` wrapped and set a false "rider GPS stale" or "link down" bit. With a real
// rider-GPS gap that turned a parked return that should WAIT into a fault ("St") about half the time.
// Every age here is stampAgeMs(), never negative. A stamp of 0 means "never" and counts as failing, exactly
// as the old `stamp == 0 ||` tests did.
// Inputs:  now_ms - read by the caller AFTER it has read nothing else time-related (the RX passes millis());
//          phase_a_rejected (2); phase_b_ok (3); tx_fix_ms + tx_stale_ms (4); rx_fix_ms + rx_stale_ms (5);
//          heading_ok (6); link_ms + link_timeout_ms (7).
// Returns: the kFmCond* bits of every failing condition (0 = all hold). Side effects: none (pure).
static inline uint8_t fmFailingMaskFrom(uint32_t now_ms, bool phase_a_rejected, bool phase_b_ok,
                                        uint32_t tx_fix_ms, uint32_t tx_stale_ms,
                                        uint32_t rx_fix_ms, uint32_t rx_stale_ms,
                                        bool heading_ok, uint32_t link_ms, uint32_t link_timeout_ms)
{
  uint8_t mask = 0;
  if (phase_a_rejected)                                mask |= kFmCondPhaseA;
  if (!phase_b_ok)                                     mask |= kFmCondPhaseB;
  if (stampStale(now_ms, tx_fix_ms, tx_stale_ms))      mask |= kFmCondTxStale;
  if (stampStale(now_ms, rx_fix_ms, rx_stale_ms))      mask |= kFmCondRxStale;
  if (!heading_ok)                                     mask |= kFmCondHeading;
  if (stampStale(now_ms, link_ms, link_timeout_ms))    mask |= kFmCondLink;
  return mask;
}

// ============================================================
// 10. N-10: AFTER A TOLERATED LINK GAP, NO RETURN MOTION UNTIL THE REMOTE HAS SAID WHO IT IS AGAIN
// ============================================================
// A parked return waits through a link loss only for a remote that sends a boot ID (S-2 + S-8), because a
// changed boot ID is what cancels the return if the remote was switched off and on. But a remote rebooted
// DURING the gap comes back with control packets first; until its boot ID arrives the buggy cannot tell it
// from the remote it was waiting for, and a squeeze could resume the return by itself (SOP-040 rule 5, audit
// N-10). So: once a link gap is seen while a boot ID has ever been heard, return motion waits until a boot
// ID has been received AFTER the gap (the RX counts every boot-ID packet, changed or not). Today's remote
// sends no boot ID, a link loss is not tolerated for it at all, and nothing changes for it.
struct FmReturnGapState {
  bool    awaiting;   // a gap was seen; no boot ID received since
  uint8_t mark;       // the boot-ID receive counter on the last tick the link was down
};

// fmReturnGapStep - one tick. Inputs: link_failing (condition 7 this tick); bootid_rx_seq (the counter of
//   boot-ID packets received); bootid_ever (a boot ID has been heard since the RX booted).
// Returns: true while return motion must wait for a boot ID. Side effects: updates *g only.
static inline bool fmReturnGapStep(FmReturnGapState* g, bool link_failing, uint8_t bootid_rx_seq,
                                   bool bootid_ever)
{
  if (link_failing) {
    g->mark     = bootid_rx_seq;
    g->awaiting = bootid_ever;
  } else if (g->awaiting && bootid_rx_seq != g->mark) {
    g->awaiting = false;
  }
  return g->awaiting;
}

// ============================================================
// 11. N-13 + OWNER RULE "WAITING MEANS IT WILL WORK": WHAT A PARKED RETURN DOES THIS TICK
// ============================================================
// SOP-040 (2026-10-07): the waiting screen is a promise. If the BUGGY's own side cannot do the return
// (its GPS 2/5, its heading 6, a real handshake mismatch 3), the return ends at once with "St" and the stop
// buzz. If only the REMOTE's side is briefly missing (its GPS 4, the link 7, or the boot ID after a gap,
// N-10), the return stays parked and the remote shows NOT READY (fm_flags bit 2). A squeeze while not ready
// gives "St" - it ends the return, as before - and MOTION always needs every input.
enum FmReturnParkVerdict : uint8_t {
  FMRPV_GO        = 0,   // nothing failing: the ordinary tick runs (a squeeze moves the buggy)
  FMRPV_NOT_READY = 1,   // stay parked, cap 0, show not-ready; nothing ends
  FMRPV_FAULT     = 2    // end the return on a fault (St + stop buzz)
};

// fmReturnParkVerdict - Inputs: failing_mask (fmFailingMaskFrom); parked (no held-trigger motion under
//   way); thr_held; link_tolerance_allowed (a boot ID has been heard); awaiting_bootid (fmReturnGapStep).
// Returns: one FmReturnParkVerdict. Side effects: none (pure).
static inline uint8_t fmReturnParkVerdict(uint8_t failing_mask, bool parked, bool thr_held,
                                          bool link_tolerance_allowed, bool awaiting_bootid)
{
  if (failing_mask == 0 && !awaiting_bootid) return FMRPV_GO;
  if (!parked || thr_held) return FMRPV_FAULT;                   // motion needs every input, and the boot ID
  if (failing_mask == 0) return FMRPV_NOT_READY;                  // only the post-gap boot ID is missing
  return fmReturnParkedTolerates(failing_mask, parked, thr_held, link_tolerance_allowed)
             ? FMRPV_NOT_READY : FMRPV_FAULT;
}

// ============================================================
// 12. N-9: A RETURN THAT STOPS CLOSING IS AN "ARRIVAL" ONLY IN THE FINAL CRAWL
// ============================================================
// S-6 treated ANY not-closing verdict inside rtm_approach_zone_m as an arrival (hand-back, no alarm). Inside
// a 12 m zone that also covered a buggy circling or steering mirrored at 10 m with a cap near 200 - a
// failure, ended in silence (audit N-9). The arrival reading is now kept for the case it was written for:
// the last metres, where the approach ramp has the cap near 0 and the motor (or the VESC deadband) stops the
// buggy short of the stop radius. Everything else is RETURN_NOT_CLOSING with "St" + the stop buzz.
static const float   kFmReturnCrawlExtraM    = 3.0f;   // m beyond the stop radius that still counts as the final crawl
static const uint8_t kFmReturnCrawlCapMax    = 40;     // a previous-tick cap at or below this is a crawl (about 16 %)
static const float   kFmReturnCrawlSpeedKmh  = 1.5f;   // the buggy must be (nearly) stopped, by its own GPS

// fmReturnStallIsArrival - Inputs: in_zone (inside rtm_approach_zone_m, zone enabled); dist_m; stop_m;
//   prev_cap (the return cap on the previous tick); aligning (the align cap, not the approach ramp, is what
//   holds the cap low - a stalled pivot is not an arrival); buggy_kmh (own GPS speed; < 0 = unknown).
// Returns: true = treat the stall as an arrival; false = a not-closing fault. Side effects: none (pure).
static inline bool fmReturnStallIsArrival(bool in_zone, float dist_m, float stop_m, uint8_t prev_cap,
                                          bool aligning, float buggy_kmh)
{
  if (!in_zone) return false;
  if (buggy_kmh < 0.0f || buggy_kmh >= kFmReturnCrawlSpeedKmh) return false;
  const bool close    = dist_m <= (stop_m + kFmReturnCrawlExtraM);
  const bool crawling = (prev_cap <= kFmReturnCrawlCapMax) && !aligning;
  return close || crawling;
}

#endif // BREMOTE_AUTO_RETURN_RULES_H
