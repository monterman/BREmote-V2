// V2.5-Evo - 2026-09-19 - The FM_RETURN entry proof, pure and host-testable.
//   Auto-return inside Follow-Me (FM_RETURN) may only start once BOTH the rider and the buggy have
//   genuinely stopped, judged from raw positions and a proven-zero throttle cap - never from a
//   filtered speed and never from a state name. This header holds the three decisions:
//     1. CANDIDATE   - the rider's RAW displacement speed over the last ~1 s is below a sanity cap
//                      (4 km/h). A fallen rider drifting at 2-3 km/h in a current still qualifies;
//                      a swimmer at 4.5 km/h never does.
//     2. WINDOW      - the throttle cap has been read back as ZERO for at least 3 s (the hull coasts
//                      for seconds after the cap drops) AND the buggy's own GPS speed is below 3 km/h;
//                      only then does a 4 s relative-displacement window open.
//     3. CONFIRMED   - over that window the RELATIVE displacement of rider and buggy (the vector
//                      difference of the two raw displacements, in metres) divided by the window
//                      length is below a 3 km/h band. Both drifting together in the same water
//                      passes; the buggy blown downwind while the rider sits in the water does not
//                      (fails safe: no return), and a scalar speed difference would sit inside the
//                      0.4-1.8 km/h per-receiver GPS noise at standstill, so positions are used.
//   The caller (RX RTMState.ino runFmLoopBody) computes the displacements with TinyGPSPlus from the
//   raw positions it snapshots when the window opens; nothing here touches GPS, time or state.
//   Tools/tests/follow_me_return_proof_test.cpp runs the exact code the RX runs.
#ifndef BREMOTE_FOLLOW_ME_RETURN_PROOF_H
#define BREMOTE_FOLLOW_ME_RETURN_PROOF_H

#include <stdint.h>
#include <math.h>

// The proof's tuning, passed in by the caller (the RX's kFmReturn* constants).
struct FmReturnProofParams {
  float    rider_raw_max_kmh;   // candidate: raw rider speed must be below this (4.0)
  uint32_t cap_zero_min_ms;     // cap proven 0 this long before the window may open (3000)
  float    buggy_max_kmh;       // buggy's own GPS speed must be below this for the window to open (3.0)
  uint32_t window_ms;           // relative-displacement window length (4000)
  float    rel_band_kmh;        // relative displacement / window below this confirms (3.0)
};

// The proof's memory between ticks. Zero = nothing proven.
struct FmReturnProofState {
  uint32_t cap_zero_since_ms;   // millis() when the cap was first read back as 0; 0 = not proven
  uint32_t window_since_ms;     // millis() when the current window opened; 0 = no window open
};

// What one tick of the proof decided. Codes are stable: they go into the log's block-reason and ?diag.
enum FmReturnProofVerdict : uint8_t {
  FMRP_IDLE          = 0,   // not a candidate, or the cap is not zero: the proof is reset
  FMRP_CAP_DWELL     = 1,   // candidate, cap zero, still inside the 3 s dwell (or the buggy still moving)
  FMRP_WINDOW_OPENED = 2,   // the window opened on THIS tick: the caller must snapshot the raw positions now
  FMRP_WINDOW_OPEN   = 3,   // the window is running
  FMRP_CONFIRMED     = 4,   // the window completed inside the band: enter FM_RETURN
  FMRP_WINDOW_FAILED = 5    // the window completed outside the band: it reopens on the next tick
};

// followMeReturnCandidate - is the rider stopped, judged on RAW displacement speed?
// Inputs:  rider_raw_kmh - the rider's raw displacement speed over the last ~1 s; a negative value
//          means "unknown" (no fresh fix) and is never a candidate. rider_raw_max_kmh - the cap.
// Returns: true when 0 <= speed < cap. Side effects: none.
static inline bool followMeReturnCandidate(float rider_raw_kmh, float rider_raw_max_kmh)
{
  return rider_raw_kmh >= 0.0f && rider_raw_kmh < rider_raw_max_kmh;
}

// followMeRelativeKmh - the relative displacement rate of rider and buggy over a window.
// Inputs:  the rider's and the buggy's displacement since the window opened, as north/east metres
//          (dn, de); elapsed_ms - the window length actually elapsed.
// Returns: |(rider - buggy)| / elapsed, in km/h. An empty window returns a huge value (never confirms).
// Side effects: none.
static inline float followMeRelativeKmh(float rider_dn_m, float rider_de_m,
                                        float buggy_dn_m, float buggy_de_m, uint32_t elapsed_ms)
{
  if (elapsed_ms == 0) return 1.0e9f;
  const float dn = rider_dn_m - buggy_dn_m;
  const float de = rider_de_m - buggy_de_m;
  return sqrtf(dn * dn + de * de) / ((float)elapsed_ms / 1000.0f) * 3.6f;
}

// followMeReturnProofStep - advance the proof by one tick.
// Inputs:  s - the proof memory (updated in place); now_ms - millis(); candidate - from
//          followMeReturnCandidate() this tick; cap_zero - the throttle cap READ BACK as zero this
//          tick (or the trigger released, which the RX treats the same way) - never assumed from a
//          state; buggy_kmh - the buggy's own GPS speed (negative = unknown, never opens a window);
//          rider_dn/de_m, buggy_dn/de_m - displacements since the window opened (ignored unless a
//          window is running); p - the tuning.
// Returns: one FmReturnProofVerdict. FMRP_WINDOW_OPENED is the caller's cue to snapshot the raw
//          positions; FMRP_CONFIRMED is the only verdict that may enter FM_RETURN.
// Side effects: only on *s. A candidate or cap-zero drop resets BOTH timers (a half-finished proof
//          never survives a squeeze); a failed window reopens from scratch on the next tick while
//          the cap-zero dwell stands. Unsigned subtraction survives a millis() wrap.
static inline uint8_t followMeReturnProofStep(FmReturnProofState* s, uint32_t now_ms,
                                              bool candidate, bool cap_zero, float buggy_kmh,
                                              float rider_dn_m, float rider_de_m,
                                              float buggy_dn_m, float buggy_de_m,
                                              const FmReturnProofParams* p)
{
  if (!candidate || !cap_zero) {
    s->cap_zero_since_ms = 0;
    s->window_since_ms   = 0;
    return FMRP_IDLE;
  }
  if (s->cap_zero_since_ms == 0) s->cap_zero_since_ms = (now_ms != 0) ? now_ms : 1u;   // 0 means "not proven"
  if ((uint32_t)(now_ms - s->cap_zero_since_ms) < p->cap_zero_min_ms) {
    s->window_since_ms = 0;
    return FMRP_CAP_DWELL;
  }
  if (!(buggy_kmh >= 0.0f && buggy_kmh < p->buggy_max_kmh)) {
    s->window_since_ms = 0;          // the hull is still moving: the window cannot open (or stay open)
    return FMRP_CAP_DWELL;
  }
  if (s->window_since_ms == 0) {
    s->window_since_ms = (now_ms != 0) ? now_ms : 1u;
    return FMRP_WINDOW_OPENED;
  }
  const uint32_t elapsed = (uint32_t)(now_ms - s->window_since_ms);
  if (elapsed < p->window_ms) return FMRP_WINDOW_OPEN;
  const float rel_kmh = followMeRelativeKmh(rider_dn_m, rider_de_m, buggy_dn_m, buggy_de_m, elapsed);
  s->window_since_ms = 0;            // one verdict per window
  return (rel_kmh < p->rel_band_kmh) ? FMRP_CONFIRMED : FMRP_WINDOW_FAILED;
}

#endif
