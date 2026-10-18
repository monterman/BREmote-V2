// V2.5-Evo - 2026-10-17 - HEADING LATCH PERSISTENCE (new, pure): the stored copy of the compass-vs-GPS-course latch
//   (/hdg_fault) is REQUESTED on a latch change and written later, once, with the trigger released, by the RX's
//   headingDisagreePersistService() (RTMState.ino). Header-only, no Arduino dependencies, so
//   Tools/tests/heading_latch_persist_test.cpp exercises the exact code the RX runs.
//
//   THE BUG THIS FIXES. The RX rewrote /hdg_fault (open, print, close) on EVERY heading call after the 5 s
//   disagreement dwell, not once when the latch rose: about 10 rewrites a second per call site, for as long as the
//   compass and the GPS course kept disagreeing (2,205 rewrites, about 1.7 MB, in one ride). Every rewrite leaves
//   deleted pages behind, so SPIFFS garbage collection ran over and over: it erased the blocks that still held the
//   deleted ride log (the log could not be recovered) and it stalled the loop task, and each flash erase freezes the
//   whole CPU, the motor task included. On the agreeing side, every heading call after 5 s of agreement looked the
//   file up by name, latched or not.
//
//   WHAT THIS DOES. Changing the latch and asking for the stored copy to follow are one step (hdgLatchSet), and that
//   step does nothing when the latch already has the value, so a standing verdict never asks again. The RX applies
//   the request later (hdgPersistDue / hdgPersistResult) from one place that runs after the tick's control writes,
//   only while the trigger is released, and retries a failed write no faster than the gap the caller passes in.
//
//   Nothing here touches SPIFFS, Serial, millis(), a cap or a steering byte: the caller passes every input and does
//   the file work itself.
#ifndef BREMOTE_HEADING_LATCH_PERSIST_H
#define BREMOTE_HEADING_LATCH_PERSIST_H

#include <stdint.h>

// ============================================================
// HdgPersist - what the stored copy of the latch should say, and whether it may not say it yet
// ============================================================
// want    : true = the marker file should exist (latch set); false = it should not exist (latch clear).
// dirty   : true = the file may not match `want` yet, so a write or a remove is still owed.
// fail_ms : the caller's clock (ms) at the last FAILED attempt for this request; 0 = no failure yet, so the first
//           attempt is due at once. A failure at clock 0 is stored as 1, so 0 always means "none".
struct HdgPersist { bool want; bool dirty; uint32_t fail_ms; };

// ============================================================
// hdgLatchSet - change the latch, and ask for the stored copy to follow, ONLY on a real change
// ============================================================
// Inputs:  latch - the RAM latch (the RX passes &heading_disagree_fault); p - the persistence request;
//          value - the verdict the caller has just proven (true = set, false = clear).
// Returns: true if the latch changed, false if it already held that value.
// Side effects: on a change only - writes *latch, sets p->want = value and p->dirty = true, and zeroes p->fail_ms so
//          the new request is tried at once and a failure of it is reported once. With no change it writes nothing,
//          which is the fix: a caller may call it on every tick that proves the same verdict and no new store is
//          requested.
// A new request replaces any pending one (a clear made while a set is still waiting cancels that set), so the file
// always ends up describing the latest latch value, never an older one.
static inline bool hdgLatchSet(volatile bool* latch, HdgPersist* p, bool value)
{
  if (*latch == value) return false;
  *latch     = value;
  p->want    = value;
  p->dirty   = true;
  p->fail_ms = 0;
  return true;
}

// ============================================================
// hdgPersistDue - is a write / remove of the stored copy due right now?
// ============================================================
// Inputs:  p - the request; now - the caller's millisecond clock; trigger_released - true when the motor cannot be
//          live (the RX passes thr_received < 25, the same test its deferred Serial notices use);
//          retry_gap_ms - the least time from a failed attempt to the next one.
// Returns: true when something is owed, the trigger is released, and either nothing has failed yet or retry_gap_ms
//          has passed since the last failure. Wrap-safe: the age is an unsigned 32-bit difference, so it stays right
//          across the clock's 49.7-day rollover.
// Side effects: none (pure).
// WHY THE TRIGGER TEST. A SPIFFS write can erase a flash sector, and an erase freezes the whole CPU, the motor task
// included. Writing only with the trigger released means such a freeze can only ever hold a zero-throttle command.
static inline bool hdgPersistDue(const HdgPersist* p, uint32_t now, bool trigger_released, uint32_t retry_gap_ms)
{
  if (!p->dirty || !trigger_released) return false;
  return (p->fail_ms == 0) || ((uint32_t)(now - p->fail_ms) >= retry_gap_ms);
}

// ============================================================
// hdgPersistResult - record how an attempt went
// ============================================================
// Inputs:  p - the request; now - the caller's clock at the attempt; ok - true if the file now matches p->want.
// Outputs: none.
// Side effects: on success, clears dirty and fail_ms (nothing is owed). On failure, stamps fail_ms = now (1 when now
//          is 0, so 0 keeps meaning "no failure") and leaves dirty set, so the next attempt waits for the caller's
//          retry gap. want is never changed here.
static inline void hdgPersistResult(HdgPersist* p, uint32_t now, bool ok)
{
  if (ok) { p->dirty = false; p->fail_ms = 0; }
  else    { p->fail_ms = (now != 0) ? now : 1; }
}

#endif  // BREMOTE_HEADING_LATCH_PERSIST_H
