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
//   The link-lost blink, the unpaired blip and the write on the pass the link comes up are unchanged.
//
// COST
//   One port-0 read-modify-write a second while connected (two I2C transactions, under 1 ms at 100 kHz, inside one
//   i2cMutex hold - the same hold every existing BIND LED write makes). The library writes OUTPUT0 + pin / 8, one byte, so this never touches port 1 (the PPM enables)
//   or the CONFIG1 register the PWM task swaps. Port 0 carries no motor signal: the BIND button and BMS sense
//   (inputs), the BIND LED, and the BMS-measure enable (set HIGH at boot; nothing measures it).
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
static inline bool bindLedRefreshMayWrite(bool pattern_running)
{
  return !pattern_running;
}

#endif
