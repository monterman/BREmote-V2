// V2.5-Evo - 2026-10-29 - delta audit LOW-1 / LOW-2 / LOW-4: bindLedRefreshPatternSeen() restarts the count on every
//   pass that sees a pattern running, so the LED stays dark at least 0.8 s after every pattern; the link-lost blink and
//   the refresh's new direction (CONFIG0) write ask bindLedRefreshMayWrite() too. No confStruct change, sizeof stays
//   200, SW_VERSION stays 36.
// V2.5-Evo - 2026-10-29 - The receiver's BIND LED is lit again about once a second while the link is up. Pure and
//   host-testable (Tools/tests/bind_led_refresh_test.cpp). Owner-approved for RX1. RX only. No confStruct change,
//   sizeof stays 200, SW_VERSION stays 36.
//
// ============================================================
// BIND LED REFRESH - when does checkConnStatus() light the BIND LED again while the link stays up?
// ============================================================
//
// WHAT WAS WRONG
//   checkConnStatus() (System.ino, a 200 ms task) lit the BIND LED (AP_L_BIND LOW = lit) only on the one pass where
//   the link came up (bind_pin_state 0 -> 1). Nothing lit it again for as long as the link stayed fresh. So the LED
//   stayed dark, with the remote driving the buggy normally, until the link next dropped and came back, whenever:
//     - that single write was lost. It is one read-modify-write of the AW9523's OUTPUT0 register under the 3 ms
//       Wire ceiling, and the library's digitalWrite() does not report a failure; or
//     - something else drove the pin HIGH while connected. Every BIND LED pattern ends with the LED dark:
//       blinkBind(), the compass FULL and PARTIAL patterns (blinkBindPattern()) and blinkErr() on the BIND LED. At
//       runtime they run from the BIND-button compass calibration, ?compasscal and ?magalign - so after any
//       calibration done with the remote on, the LED stayed dark.
//   Seen on RX1 on 2026-10-29: BIND LED fully off while the remote drove the buggy; the AUX LED worked.
//
// WHAT IT DOES NOW
//   While paired with a fresh link, every kBindLedRefreshPasses-th pass on which the LED was ALREADY lit
//   (5 x 200 ms = about once a second) writes AP_L_BIND LOW again. The count restarts on the pass that lights the
//   LED when the link comes up, so the first refresh is 1 s after it. A lost write, or a stray HIGH, is repaired
//   within about a second.
//   A deliberate pattern is NEVER overwritten: the rider reads the compass FULL / PARTIAL / REJECTED patterns off
//   this LED, and a refresh in the middle of one would change what it says. Each pattern function sets a flag for
//   as long as it runs, and the refresh skips its write while the flag is set (bindLedRefreshMayWrite(), asked with
//   i2cMutex held - see System.ino). A skipped refresh is not tried again early: the next one is 5 passes later, so
//   the LED is lit again at most about 1 s after a pattern ends.
//   V2.5-Evo - 2026-10-29 - delta audit LOW-1: ...and at LEAST 0.8 s after it ends. The count used to run on through a
//   pattern, so the first refresh after it could come anywhere from 0 to 1 s later; a relight 50-300 ms after the
//   pattern's last dark gap looked like one more flash (FULL could read "blip, blip, long"). Now every pass that sees
//   the pattern flag restarts the count (bindLedRefreshPatternSeen()), so the first refresh comes 5 passes after the
//   last pass inside the pattern: 0.8-1.0 s after the pattern ends. Every pattern that can run while the refresh runs
//   lasts 0.5 s or more, so at least one pass always sees it.
//   The unpaired blip and the write on the pass the link comes up are unchanged.
//   V2.5-Evo - 2026-10-29 - delta audit LOW-2: the link-lost blink (remote off or out of range) now also skips its LED
//   write while a pattern runs, but still toggles its state, so it keeps its phase and carries on after the pattern.
//   It used to write every 200 ms straight through a calibration result and could cut a PARTIAL pulse short.
//
// COST
//   One port-0 read-modify-write a second while connected (two I2C transactions, under 1 ms at 100 kHz, inside one
//   i2cMutex hold - the same hold every existing BIND LED write makes). The library writes OUTPUT0 + pin / 8, one byte, so this never touches port 1 (the PPM enables)
//   or the CONFIG1 register the PWM task swaps. Port 0 carries no motor signal: the BIND button and BMS sense
//   (inputs), the BIND LED, and the BMS-measure enable (set HIGH at boot; nothing measures it).
//   V2.5-Evo - 2026-10-29 - delta audit LOW-4: the refresh also writes the pin's DIRECTION - the port-0 direction
//   register, CONFIG0, as one absolute byte - in a SECOND short hold of one I2C transaction (the register pointer and
//   one data byte, so it can never run on into CONFIG1). A level write cannot light an LED whose pin was left an
//   input. Each hold is still at most two transactions. Why and how: System.ino, awPort0DirWriteLocked().
//
// Nothing here reads globals, config or the clock: the caller passes them in. Called by checkConnStatus() only (the
// counter is that task's own local variable, so it has one writer).
#ifndef BREMOTE_BIND_LED_REFRESH_H
#define BREMOTE_BIND_LED_REFRESH_H

#include <stdint.h>

// checkConnStatus() passes between two refreshes of the lit BIND LED. The task runs every 200 ms, so 5 = 1.0 s.
static const uint8_t kBindLedRefreshPasses = 5;

// bindLedRefreshDue - one checkConnStatus() pass while paired with a fresh link (the branch that keeps the LED lit).
// Inputs:
//   passes  - the pass counter (read and updated); the task's own variable, 0 when the task starts.
//   was_lit - bind_pin_state was already 1 when this pass began, so the LED-on write is NOT made on this pass.
//             false = this is the pass that lights the LED because the link has just come up: the count restarts.
// Returns: true = write AP_L_BIND LOW again on this pass (unless a pattern is running, see below).
// Side effects: updates *passes only. A counter above the limit (it cannot happen) is treated as due and restarts.
// Not called while the link is stale or the board is unpaired, so the count pauses there. If the link comes back on
// the lit half of the link-lost blink, no LED-on write is made and the count resumes - the LED is already lit then,
// and the next refresh comes within 1 s.
// V2.5-Evo - 2026-10-29 - (delta audit LOW-2) "already lit" is now what bind_pin_state says, not always what the LED
// shows: if the blink's LOW write was skipped for a pattern, the LED can be dark with bind_pin_state at 1. The refresh
// is what lights it then: within 1 s of the link coming back, and never sooner than 0.8 s after the pattern ended
// (bindLedRefreshPatternSeen()).
static inline bool bindLedRefreshDue(uint8_t *passes, bool was_lit)
{
  if (!was_lit)
  {
    *passes = 0;
    return false;
  }
  *passes = (uint8_t)(*passes + 1);
  if (*passes < kBindLedRefreshPasses) return false;
  *passes = 0;
  return true;
}

// bindLedRefreshMayWrite - may the due refresh write the LED now?
// Inputs: pattern_running - blinkBind(), blinkBindPattern() or blinkErr() on the BIND LED is running.
// Returns: true = write AP_L_BIND LOW now; false = skip this refresh and leave the pattern alone.
// The caller asks this with i2cMutex HELD. The pattern sets its flag before its first take of the mutex, so either
// the refresh's write lands entirely before the pattern's first write (and the pattern draws over it), or the
// refresh sees the flag. It can never land between two of the pattern's writes.
// V2.5-Evo - 2026-10-29 - asked the same way, in its own hold, before the refresh's direction (CONFIG0) write (delta
// audit LOW-4: if the pin had been left an input, turning it into an output inside a pattern could cut a visible pulse
// short), and before each link-lost blink write (LOW-2).
static inline bool bindLedRefreshMayWrite(bool pattern_running)
{
  return !pattern_running;
}

// bindLedRefreshPatternSeen - V2.5-Evo - 2026-10-29 - delta audit LOW-1. Called at the END of every checkConnStatus()
// pass, whatever the link state, with the pattern flag as it reads at that moment.
// Inputs:
//   passes          - the pass counter (read and updated), as for bindLedRefreshDue().
//   pattern_running - blinkBind(), blinkBindPattern() or blinkErr() on the BIND LED is running.
// Returns: nothing. Side effects: *passes = 0 while a pattern runs. The count therefore restarts on the last pass that
//   sees the pattern, and the first refresh after it comes 5 passes (1.0 s) after that pass: 0.8-1.0 s after the
//   pattern ends, never sooner.
// It must run AFTER bindLedRefreshDue() on the same pass, so that pass ends with the count at 0. Run before it, the
// count would end the pass at 1 and the dark gap after a pattern could shrink to 0.6 s.
static inline void bindLedRefreshPatternSeen(uint8_t *passes, bool pattern_running)
{
  if (pattern_running) *passes = 0;
}

#endif
