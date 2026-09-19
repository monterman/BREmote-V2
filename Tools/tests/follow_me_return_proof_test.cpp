// Host test for Common/FollowMeReturnProof.h - the FM_RETURN entry proof.
// The RX's constants (RTMState.ino kFmReturn*): rider raw max 4.0 km/h, cap-zero dwell 3000 ms,
// buggy max 3.0 km/h, window 4000 ms, relative band 3.0 km/h. Ticks are 100 ms like runFmLoop().

#include <assert.h>
#include <stdint.h>
#include <math.h>

#include "../../Source/Common/FollowMeReturnProof.h"

static const FmReturnProofParams P = { 4.0f, 3000u, 3.0f, 4000u, 3.0f };

// Displacement (metres) a body covers in window_ms at speed_kmh.
static float dispM(float speed_kmh, uint32_t window_ms) { return speed_kmh / 3.6f * ((float)window_ms / 1000.0f); }

// Run the proof from t0 with a constant candidate / cap / buggy speed, feeding a rider displacement
// that grows linearly to rider_final_m north and a buggy displacement to buggy_final_m north at the
// end of the window (the caller in the RX snapshots on WINDOW_OPENED and measures from there, so the
// displacement handed in is "since the window opened"). Returns the first verdict >= CONFIRMED or
// the last verdict seen if none, plus the tick time at which it happened.
static uint8_t run(FmReturnProofState* s, uint32_t t0, uint32_t ticks, bool candidate, bool cap_zero,
                   float buggy_kmh, float rider_final_m, float buggy_final_m, uint32_t* t_out)
{
  uint32_t opened_at = 0;
  uint8_t  v = FMRP_IDLE;
  for (uint32_t i = 0; i < ticks; ++i) {
    const uint32_t now = t0 + i * 100u;
    float rdn = 0.0f, bdn = 0.0f;
    if (opened_at != 0) {
      const float frac = (float)(now - opened_at) / (float)P.window_ms;
      rdn = rider_final_m * frac;
      bdn = buggy_final_m * frac;
    }
    v = followMeReturnProofStep(s, now, candidate, cap_zero, buggy_kmh, rdn, 0.0f, bdn, 0.0f, &P);
    if (v == FMRP_WINDOW_OPENED) opened_at = now;
    if (v == FMRP_CONFIRMED || v == FMRP_WINDOW_FAILED) { *t_out = now; return v; }
  }
  *t_out = t0 + (ticks - 1) * 100u;
  return v;
}

int main()
{
  // ---- The candidate is judged on RAW rider speed, and only below 4 km/h ----
  assert(followMeReturnCandidate(0.0f, P.rider_raw_max_kmh));
  assert(followMeReturnCandidate(2.5f, P.rider_raw_max_kmh));    // a fallen rider drifting in a current
  assert(followMeReturnCandidate(3.99f, P.rider_raw_max_kmh));
  assert(!followMeReturnCandidate(4.0f, P.rider_raw_max_kmh));   // the cap itself is not a candidate
  assert(!followMeReturnCandidate(4.5f, P.rider_raw_max_kmh));   // a swimmer
  assert(!followMeReturnCandidate(-1.0f, P.rider_raw_max_kmh));  // unknown (no fresh fix) is never a candidate

  // ---- The relative rate: a vector, in km/h over the window ----
  assert(fabsf(followMeRelativeKmh(dispM(2.9f, 4000u), 0.0f, 0.0f, 0.0f, 4000u) - 2.9f) < 0.01f);
  assert(fabsf(followMeRelativeKmh(0.0f, 0.0f, dispM(3.1f, 4000u), 0.0f, 4000u) - 3.1f) < 0.01f);
  // Both drift 5 m north together: relative 0. Opposite directions: relative 10 m / 4 s = 9 km/h.
  assert(followMeRelativeKmh(5.0f, 0.0f, 5.0f, 0.0f, 4000u) < 0.001f);
  assert(fabsf(followMeRelativeKmh(5.0f, 0.0f, -5.0f, 0.0f, 4000u) - 9.0f) < 0.01f);
  // A scalar difference would call "3 m north vs 3 m east" zero; the vector does not (4.24 m / 4 s = 3.8 km/h).
  assert(followMeRelativeKmh(3.0f, 0.0f, 0.0f, 3.0f, 4000u) > 3.0f);
  assert(followMeRelativeKmh(0.0f, 0.0f, 0.0f, 0.0f, 0u) > 1.0e8f);   // an empty window never confirms

  uint32_t t;

  // ---- No confirmation while the cap is not zero, however long the rider is stopped ----
  {
    FmReturnProofState s = {0, 0};
    uint8_t v = run(&s, 1000u, 300u, true, false, 0.0f, 0.0f, 0.0f, &t);   // 30 s, cap never zero
    assert(v == FMRP_IDLE);
    assert(s.cap_zero_since_ms == 0 && s.window_since_ms == 0);
  }

  // ---- None inside 3 s of cap zero: the window cannot even open before the dwell ----
  {
    FmReturnProofState s = {0, 0};
    uint8_t v = run(&s, 1000u, 30u, true, true, 0.0f, 0.0f, 0.0f, &t);   // 2.9 s of cap zero
    assert(v == FMRP_CAP_DWELL);
    assert(s.cap_zero_since_ms == 1000u && s.window_since_ms == 0);
    // one more tick reaches 3.0 s: the window opens now (the caller snapshots), still nothing confirmed
    v = followMeReturnProofStep(&s, 4000u, true, true, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &P);
    assert(v == FMRP_WINDOW_OPENED);
    assert(s.window_since_ms == 4000u);
    // and the earliest possible confirmation is 4 s after that: 3 s + 4 s = 7 s from cap zero
    v = followMeReturnProofStep(&s, 7900u, true, true, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &P);
    assert(v == FMRP_WINDOW_OPEN);
    v = followMeReturnProofStep(&s, 8000u, true, true, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &P);
    assert(v == FMRP_CONFIRMED);
  }

  // ---- The buggy must itself be below 3 km/h before the window opens (hull coast) ----
  {
    FmReturnProofState s = {0, 0};
    uint8_t v = run(&s, 1000u, 100u, true, true, 3.5f, 0.0f, 0.0f, &t);   // 10 s, buggy still coasting at 3.5
    assert(v == FMRP_CAP_DWELL);
    assert(s.cap_zero_since_ms == 1000u && s.window_since_ms == 0);
    // unknown buggy speed (negative) never opens the window either
    v = followMeReturnProofStep(&s, 12000u, true, true, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, &P);
    assert(v == FMRP_CAP_DWELL);
    // the hull settles: the window opens on the first tick below 3 km/h, with the dwell already served
    v = followMeReturnProofStep(&s, 12100u, true, true, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, &P);
    assert(v == FMRP_WINDOW_OPENED);
    // and if the hull speeds up again mid-window, the window is dropped (not paused)
    v = followMeReturnProofStep(&s, 14000u, true, true, 3.2f, 0.0f, 0.0f, 0.0f, 0.0f, &P);
    assert(v == FMRP_CAP_DWELL);
    assert(s.window_since_ms == 0 && s.cap_zero_since_ms == 1000u);
  }

  // ---- Relative 2.9 km/h inside a 3.0 band confirms; 3.1 does not (and reopens) ----
  {
    FmReturnProofState s = {0, 0};
    uint8_t v = run(&s, 1000u, 200u, true, true, 0.0f, dispM(2.9f, P.window_ms), 0.0f, &t);
    assert(v == FMRP_CONFIRMED);
    assert(t == 8000u);   // cap zero at 1 s + 3 s dwell + 4 s window
  }
  {
    FmReturnProofState s = {0, 0};
    uint8_t v = run(&s, 1000u, 200u, true, true, 0.0f, dispM(3.1f, P.window_ms), 0.0f, &t);
    assert(v == FMRP_WINDOW_FAILED);
    assert(t == 8000u);
    assert(s.window_since_ms == 0 && s.cap_zero_since_ms != 0);   // the dwell stands, the window reopens
    v = followMeReturnProofStep(&s, 8100u, true, true, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &P);
    assert(v == FMRP_WINDOW_OPENED);
  }
  // Both drifting together at 2.9 km/h (rider AND buggy): relative 0 -> confirms.
  {
    FmReturnProofState s = {0, 0};
    const float d = dispM(2.9f, P.window_ms);
    uint8_t v = run(&s, 1000u, 200u, true, true, 2.9f, d, d, &t);
    assert(v == FMRP_CONFIRMED);
  }
  // The buggy blown 3.1 km/h downwind while the rider sits still: relative 3.1 -> never confirms.
  {
    FmReturnProofState s = {0, 0};
    uint8_t v = run(&s, 1000u, 200u, true, true, 2.5f, 0.0f, dispM(3.1f, P.window_ms), &t);
    assert(v == FMRP_WINDOW_FAILED);
  }

  // ---- Rider raw 4.5 km/h never confirms, whatever the cap and the buggy do ----
  {
    FmReturnProofState s = {0, 0};
    const bool cand = followMeReturnCandidate(4.5f, P.rider_raw_max_kmh);
    assert(!cand);
    uint8_t v = run(&s, 1000u, 300u, cand, true, 0.0f, 0.0f, 0.0f, &t);
    assert(v == FMRP_IDLE);
    assert(s.cap_zero_since_ms == 0 && s.window_since_ms == 0);
  }

  // ---- A squeeze (cap no longer zero) mid-proof resets BOTH timers; the proof restarts from scratch ----
  {
    FmReturnProofState s = {0, 0};
    uint8_t v = run(&s, 1000u, 50u, true, true, 0.0f, 0.0f, 0.0f, &t);   // 5 s in: dwell served, window open
    assert(v == FMRP_WINDOW_OPEN && s.window_since_ms == 4000u);
    v = followMeReturnProofStep(&s, 6000u, true, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &P);
    assert(v == FMRP_IDLE && s.cap_zero_since_ms == 0 && s.window_since_ms == 0);
    v = followMeReturnProofStep(&s, 6100u, true, true, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &P);
    assert(v == FMRP_CAP_DWELL && s.cap_zero_since_ms == 6100u);
  }

  // ---- The rider moving off (candidate drops) resets the proof the same way ----
  {
    FmReturnProofState s = {0, 0};
    run(&s, 1000u, 50u, true, true, 0.0f, 0.0f, 0.0f, &t);
    uint8_t v = followMeReturnProofStep(&s, 6000u, false, true, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &P);
    assert(v == FMRP_IDLE && s.cap_zero_since_ms == 0 && s.window_since_ms == 0);
  }

  // ---- millis() wrap: the dwell and the window survive the 32-bit rollover ----
  {
    FmReturnProofState s = {0, 0};
    uint8_t v = followMeReturnProofStep(&s, 0xFFFFF000u, true, true, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &P);
    assert(v == FMRP_CAP_DWELL);
    v = followMeReturnProofStep(&s, 0xFFFFF000u + 3000u, true, true, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &P);   // wraps past 0
    assert(v == FMRP_WINDOW_OPENED);
    v = followMeReturnProofStep(&s, 0xFFFFF000u + 7000u, true, true, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &P);
    assert(v == FMRP_CONFIRMED);
  }
  // A cap-zero edge exactly at millis() == 0 is still "proven" (stored as 1, never as the 0 sentinel).
  {
    FmReturnProofState s = {0, 0};
    followMeReturnProofStep(&s, 0u, true, true, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &P);
    assert(s.cap_zero_since_ms == 1u);
  }

  return 0;
}
