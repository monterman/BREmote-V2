// V2.5-Evo - 2026-10-30 - NEW FILE, unit 1 of the 2026-10-30 Follow-Me package (Revision 2 §5): THE FAST-CLOSURE CEILING.
//   Follow-Me slows the buggy by itself ONLY when the rider stops dead in front of it (Revision 2.2 §B: stops only; the
//   U-turn case needs the remote's own GPS course and speed, next package), so that it reaches its normal stopping band at
//   a speed from which an unpowered coast ends at the hard stop (min_dist). It speeds up again by itself when he goes
//   again. No standoff: the stop point stays min_dist and the approach band; steering is not touched.
//   Header-only, no Arduino dependencies. Every constant comes from the caller. Nothing here writes a cap: the RX applies
//   the verdict INSIDE cap 3 (a lower target, or cap 3 = 0 to coast) and takes min() with its other caps as always, and
//   calcPWM() applies min(trigger, cap) - so the human trigger stays the only throttle source and this can only subtract.
//
//   THE INPUTS ARE THE RAW RIDER FIXES, NOT THE 2 s FILTERED TRACK. The filtered course lags about 2 s, which is the
//   flown code's late convergence detection. Each NEW rider GPS solution adds one sample: that fix's distance to the
//   buggy. r is the mean of the 2 newest samples; c-dot is how fast that mean fell over about 1 s (+ = closing).
//   OWNER RULE (a feature that relies on a sensor is gated on that sensor being present and good): the caller passes
//   inputs_ok only when the rider's fix is fresh within tx_gps_stale_timeout_ms (the newest sample's age is checked here
//   too), the buggy's own fix is good (Phase A) and the handshake stands (Phase B); v_los needs a valid buggy COG. With
//   any input bad the verdict is "no ceiling" - today's behaviour - never a looser cap: this header only ever lowers one.
//
//   1. FmCvTrack / fmCvTrackReset / fmCvTrackSample - the per-solution distance history (the newest 8 samples).
//   2. fmCvClosing     - r and c-dot from the history, or false when there is not a clean ~1 s baseline.
//   3. fmCvVmaxMps     - the ceiling on the buggy's speed toward the rider, from r and c-dot.
//   4. fmCvLosMps      - the buggy's own speed toward the rider (GPS speed x cos(COG vs the bearing to him)).
//   5. fmCvDecide      - the verdict: no ceiling, a lower governor target, or coast (cap 3 = 0).
//   6. fmCvCap3Step    - after a clamp, cap 3 may rise again at no more than 255 per kFmCvReriseMs.
//   7. fmCvParks       - the divergence net is parked while the ceiling binds and kFmCvParkMs after it last bound.
//   8. fmCvRiderMotion - the rider's raw speed and course, from the same fix pairs as c-dot.
//   9. FmCvQual / fmCvQualStep - V2.5-Evo - 2026-10-30 - Revision 2.1 §D, cut to STOPS ONLY by Revision 2.2 §B: the
//      ceiling may act only while the rider is measurably stopping in front of the buggy (his raw speed well below its
//      recent maximum while the gap closes). The running-at (U-turn) leg is deleted: with the rider's position 1 s late
//      at 3 Hz it could not tell a carve from a U-turn, bound while he surfed, and gained under 1 ft in U-turns.
#ifndef BREMOTE_FOLLOW_ME_CONVERGE_H
#define BREMOTE_FOLLOW_ME_CONVERGE_H

#include <stdint.h>
#include <math.h>

// ============================================================
// 1. THE PER-SOLUTION DISTANCE HISTORY
// ============================================================
static const uint8_t kFmCvTrackLen = 8;   // about 2.7 s at 3 Hz: enough for a 1 s baseline plus a missed fix or two
struct FmCvTrack {
  uint32_t t_ms[kFmCvTrackLen];   // when each sample was taken (the tick the new solution was seen)
  float    d_m[kFmCvTrackLen];    // that fix's distance to the buggy, metres
  double   lat[kFmCvTrackLen];    // V2.5-Evo - 2026-10-30 - Rev 2.1 §D: that fix itself (degrees), for his raw speed and course
  double   lng[kFmCvTrackLen];
  uint8_t  count;                 // valid samples, 0..kFmCvTrackLen
  uint8_t  head;                  // index of the newest sample
  uint32_t last_seq;              // the rider solution counter at the last sample
  bool     seq_seen;              // last_seq holds a real reading
};

// fmCvTrackReset - forget every sample (a data gap, or Follow-Me leaving its states). Side effects: *t.
static inline void fmCvTrackReset(FmCvTrack *t)
{
  t->count = 0; t->head = 0; t->last_seq = 0; t->seq_seen = false;
  for (uint8_t i = 0; i < kFmCvTrackLen; i++) { t->t_ms[i] = 0; t->d_m[i] = 0.0f; t->lat[i] = 0.0; t->lng[i] = 0.0; }
}

// fmCvTrackSample - add a sample when the rider's solution counter moved. The first reading only seeds the counter.
// Inputs: now_ms; seq - the rider GPS solution counter (rx_tx_gps_solution_seq); dist_m - buggy to the rider's newest fix
// (negative = no trustworthy distance this tick: the sample is skipped); lat / lng - that fix (rx_tx_gps_lat / lng).
// Side effects: *t.
static inline void fmCvTrackSample(FmCvTrack *t, uint32_t now_ms, uint32_t seq, float dist_m, double lat, double lng)
{
  if (!t->seq_seen) { t->last_seq = seq; t->seq_seen = true; return; }
  if (seq == t->last_seq) return;
  t->last_seq = seq;
  if (!(dist_m >= 0.0f)) return;
  t->head = (uint8_t)((t->head + 1) % kFmCvTrackLen);
  t->t_ms[t->head] = now_ms;
  t->d_m[t->head]  = dist_m;
  t->lat[t->head]  = lat;
  t->lng[t->head]  = lng;
  if (t->count < kFmCvTrackLen) t->count++;
}

// fmCvNewestMs - when the newest sample was taken (0 = none). A new value means a new estimate.
static inline uint32_t fmCvNewestMs(const FmCvTrack *t)
{
  return (t->count == 0) ? 0u : t->t_ms[t->head];
}

// fmCvRange - r alone: the mean of the 2 newest samples, if the newest is no older than max_age_ms.
// Returns false (and leaves *r_m) when there are fewer than 2 samples or the newest is stale.
static inline bool fmCvRange(const FmCvTrack *t, uint32_t now_ms, uint32_t max_age_ms, float *r_m)
{
  if (t->count < 2) return false;
  if ((uint32_t)(now_ms - t->t_ms[t->head]) > max_age_ms) return false;
  const uint8_t p = (uint8_t)((t->head + kFmCvTrackLen - 1) % kFmCvTrackLen);
  *r_m = 0.5f * (t->d_m[t->head] + t->d_m[p]);
  return true;
}

// ============================================================
// 2. fmCvClosing - r and c-dot
// ============================================================
// r = the mean of the 2 newest samples. c-dot = (the mean of an older adjacent pair - r) / the time between the two
// pairs' midpoints, using the older pair whose midpoint is nearest 1.0 s before the newest pair's, accepted only if that
// gap is 0.6-2.0 s (a clean ~1 s baseline). Positive = closing.
// Inputs: max_age_ms - the newest sample may be no older than this (the RX passes tx_gps_stale_timeout_ms).
// Returns: true with *r_m and *cdot_mps set, or false (no ceiling: not enough history, a stale fix, or no clean baseline).
// fmCvPickBaseline - V2.5-Evo - 2026-10-30 - the pair choice fmCvClosing() and fmCvRiderMotion() share (moved out of
// fmCvClosing() unchanged, so both use the SAME two pairs). Returns false (no estimate) under 4 samples, with the newest
// older than max_age_ms, or with no older pair 0.6-2.0 s back; else *k_out = how many samples back the older pair starts
// (it is samples k and k + 1 back from the newest, 0 = newest) and *dt_s = the time between the two pairs' midpoints.
static inline bool fmCvPickBaseline(const FmCvTrack *t, uint32_t now_ms, uint32_t max_age_ms, uint8_t *k_out, float *dt_s)
{
  if (t->count < 4) return false;
  if ((uint32_t)(now_ms - t->t_ms[t->head]) > max_age_ms) return false;
  const uint8_t h0 = t->head, h1 = (uint8_t)((h0 + kFmCvTrackLen - 1) % kFmCvTrackLen);
  const uint32_t t_now = t->t_ms[h1] + (uint32_t)((t->t_ms[h0] - t->t_ms[h1]) / 2u);
  float best_err = 1e9f, best_dt = 0.0f;
  uint8_t best_k = 0;
  // The older pair is the samples k and k + 1 back from the newest (0 = newest), never overlapping the newest pair.
  for (uint8_t k = 2; k + 1 < t->count; k++) {
    const uint8_t a = (uint8_t)((h0 + kFmCvTrackLen - k) % kFmCvTrackLen);
    const uint8_t b = (uint8_t)((h0 + kFmCvTrackLen - k - 1) % kFmCvTrackLen);
    const uint32_t t_pair = t->t_ms[b] + (uint32_t)((t->t_ms[a] - t->t_ms[b]) / 2u);
    const float dt = (float)(uint32_t)(t_now - t_pair) / 1000.0f;
    const float err = fabsf(dt - 1.0f);
    if (err < best_err) { best_err = err; best_dt = dt; best_k = k; }
  }
  if (!(best_dt >= 0.6f && best_dt <= 2.0f)) return false;
  *k_out = best_k;
  *dt_s  = best_dt;
  return true;
}

static inline bool fmCvClosing(const FmCvTrack *t, uint32_t now_ms, uint32_t max_age_ms, float *r_m, float *cdot_mps)
{
  uint8_t k = 0; float dt = 0.0f;
  if (!fmCvPickBaseline(t, now_ms, max_age_ms, &k, &dt)) return false;
  const uint8_t h0 = t->head, h1 = (uint8_t)((h0 + kFmCvTrackLen - 1) % kFmCvTrackLen);
  const uint8_t a = (uint8_t)((h0 + kFmCvTrackLen - k) % kFmCvTrackLen);
  const uint8_t b = (uint8_t)((h0 + kFmCvTrackLen - k - 1) % kFmCvTrackLen);
  const float r_now = 0.5f * (t->d_m[h0] + t->d_m[h1]);
  *r_m = r_now;
  *cdot_mps = (0.5f * (t->d_m[a] + t->d_m[b]) - r_now) / dt;
  return true;
}

// ============================================================
// 3. fmCvVmaxMps - the ceiling on the buggy's speed toward the rider
// ============================================================
// r_b = min_dist + band (the approach band's outer edge); v_band = band / tau_c (the speed from which an unpowered coast
// with time constant tau_c ends exactly at the hard stop). LATENCY MARGIN: r_eff = r - c-dot x latency_s - a margin on a
// measured distance for the remote's report delay, used only to LOWER the ceiling; nothing here predicts a rider
// position for steering (GPS rule 4).
//   c-dot > 0 and r_eff > r_b: v_max = v_band x e^(t_b / tau_c), t_b = (r_eff - r_b) / c-dot (the time left before the
//   band, at the measured closure; the buggy may arrive faster the more time it has to coast down);
//   c-dot > 0 and r_eff <= r_b: v_max = v_band;
//   c-dot <= 0: no ceiling (returns a negative number).
// The exponent is held at or below 12 so a tiny closure gives a huge (non-binding) ceiling, never an overflow.
static inline float fmCvVmaxMps(float r_m, float cdot_mps, float latency_s, float r_b_m, float v_band_mps, float tau_c_s)
{
  if (!(cdot_mps > 0.0f)) return -1.0f;
  const float r_eff = r_m - cdot_mps * latency_s;
  if (r_eff <= r_b_m) return v_band_mps;
  float x = ((r_eff - r_b_m) / cdot_mps) / tau_c_s;
  if (x > 12.0f) x = 12.0f;
  return v_band_mps * expf(x);
}

// ============================================================
// 4. fmCvLosMps - the buggy's own speed toward the rider
// ============================================================
// v_los = GPS speed x cos(COG - bearing buggy -> rider), and 0 when the COG is not valid or the buggy moves away.
// Inputs: speed_kmh (gps_last_speed_kmh); cog_ok - the buggy's COG is fresh, moving and not frozen; cog_deg;
//         bearing_deg - from the buggy to the rider's newest fix.
// Outputs: *cos_out = that cosine when cog_ok, else 0 (unknown). Returns metres per second, >= 0.
static inline float fmCvLosMps(float speed_kmh, bool cog_ok, float cog_deg, float bearing_deg, float *cos_out)
{
  *cos_out = 0.0f;
  if (!cog_ok || !(speed_kmh > 0.0f)) return 0.0f;
  const float c = cosf((cog_deg - bearing_deg) * 0.0174532925f);
  *cos_out = c;
  if (c <= 0.0f) return 0.0f;
  return speed_kmh / 3.6f * c;
}

// ============================================================
// 5. fmCvDecide - the verdict for this tick
// ============================================================
struct FmCvVerdict {
  bool  active;        // a ceiling stands (the rider is closing)
  bool  coast;         // the buggy is already faster toward him than the ceiling: cap 3 = 0
  float gov_max_kmh;   // the most the governor target may be (km/h along the buggy's track), < 0 = no target clamp
  float vmax_mps;      // the ceiling itself, for ?diag (< 0 = none)
};
// Inputs: inputs_ok (see the file header); r_m / cdot_mps from fmCvClosing(); v_los_mps / cos_los from fmCvLosMps()
//   (cos_known = the COG was valid); the constants. Returns the verdict. Side effects: none.
// The target clamp is v* <= v_max / cos, applied only while cos > cos_min (a buggy moving mostly sideways or away is not
// what the ceiling is about, and dividing by a small cosine would give no limit anyway).
static inline FmCvVerdict fmCvDecide(bool inputs_ok, float r_m, float cdot_mps, float v_los_mps, bool cos_known,
                                     float cos_los, float latency_s, float r_b_m, float v_band_mps, float tau_c_s,
                                     float cos_min)
{
  FmCvVerdict v = { false, false, -1.0f, -1.0f };
  if (!inputs_ok) return v;
  const float vmax = fmCvVmaxMps(r_m, cdot_mps, latency_s, r_b_m, v_band_mps, tau_c_s);
  if (vmax < 0.0f) return v;
  v.active = true;
  v.vmax_mps = vmax;
  if (v_los_mps > vmax) { v.coast = true; return v; }
  if (cos_known && cos_los > cos_min) v.gov_max_kmh = vmax / cos_los * 3.6f;
  return v;
}

// ============================================================
// 6. fmCvCap3Step - coast, and the re-rise limit after a clamp
// ============================================================
// cap3 is cap 3 as computed this tick WITH the ceiling's target clamp already in it; clamped = the ceiling lowered it
// this tick (the target clamp bound, or a coast - the caller then passes cap3 = 0). While a clamp has happened and cap 3
// has not yet got back to its unclamped value, it may rise by no more than 255 counts per rerise_ms; it may always
// fall. A gap of more than gap_ms since the last call (Follow-Me was not computing cap 3 - HOLD, a pause, a creep, an
// RTM yield) starts fresh: the engage ramp (cap 5) already governs that rise.
struct FmCvRerise {
  uint32_t last_ms;   // the last call, 0 = never
  float    cap;       // cap 3 as this function returned it last time
  bool     rising;    // a clamp happened and cap 3 is still being let back up
};
static inline uint16_t fmCvCap3Step(FmCvRerise *s, uint32_t now_ms, uint16_t cap3, bool clamped, uint32_t rerise_ms,
                                    uint32_t gap_ms)
{
  if (s->last_ms == 0 || (uint32_t)(now_ms - s->last_ms) > gap_ms) { s->rising = false; s->cap = (float)cap3; }
  const float dt_ms = (s->last_ms == 0) ? 0.0f : (float)(uint32_t)(now_ms - s->last_ms);
  float out = (float)cap3;
  if (clamped) s->rising = true;
  if (s->rising) {
    const float lim = s->cap + 255.0f * dt_ms / (float)rerise_ms;
    if (out > lim) out = lim;
    if (!clamped && out >= (float)cap3) s->rising = false;
  }
  s->cap = out;
  s->last_ms = (now_ms != 0) ? now_ms : 1u;
  return (uint16_t)out;
}

// ============================================================
// 7. fmCvParks - the divergence net is parked during and after a closure clamp
// ============================================================
// Why (Revision 2 §5): in a wave-linking pair the rider turns away again while the slowed buggy must turn round; the
// gap then opens inside R2's band and R2's "running away" verdict would end Follow-Me (sticky cap, needs a release).
// Inputs: last_bind_ms - when the ceiling last bound (0 = not this engagement); park_ms - kFmCvParkMs (6500).
static inline bool fmCvParks(uint32_t last_bind_ms, uint32_t now_ms, uint32_t park_ms)
{
  return last_bind_ms != 0 && (uint32_t)(now_ms - last_bind_ms) < park_ms;
}

// ============================================================
// 8. fmCvRiderMotion - the rider's raw speed and course
// ============================================================
// V2.5-Evo - 2026-10-30 - Revision 2.1 §D. From the SAME two fix pairs as c-dot (fmCvPickBaseline()): the move of the
// mean of the 2 newest fixes from the mean of the older pair, over the time between them. Raw fixes only, nothing
// filtered or extrapolated (GPS rules 1-4). The local flat-earth step is exact to millimetres over a few metres.
// Inputs: course_min_mps - under this speed the course is not trusted (the RX passes kFmCourseValidSpeedKmh / 3.6).
// Outputs: *spd_mps, *course_deg (0-360 clockwise from north), *course_ok. Returns false when there is no estimate.
static inline bool fmCvRiderMotion(const FmCvTrack *t, uint32_t now_ms, uint32_t max_age_ms, float course_min_mps,
                                   float *spd_mps, float *course_deg, bool *course_ok)
{
  *course_ok = false;
  uint8_t k = 0; float dt = 0.0f;
  if (!fmCvPickBaseline(t, now_ms, max_age_ms, &k, &dt)) return false;
  const uint8_t h0 = t->head, h1 = (uint8_t)((h0 + kFmCvTrackLen - 1) % kFmCvTrackLen);
  const uint8_t a = (uint8_t)((h0 + kFmCvTrackLen - k) % kFmCvTrackLen);
  const uint8_t b = (uint8_t)((h0 + kFmCvTrackLen - k - 1) % kFmCvTrackLen);
  const double lat_n = 0.5 * (t->lat[h0] + t->lat[h1]), lng_n = 0.5 * (t->lng[h0] + t->lng[h1]);
  const double lat_o = 0.5 * (t->lat[a] + t->lat[b]),   lng_o = 0.5 * (t->lng[a] + t->lng[b]);
  const double m_per_deg = 111319.49;   // 2 x pi x 6378137 m / 360
  const double north = (lat_n - lat_o) * m_per_deg;
  const double east  = (lng_n - lng_o) * m_per_deg * cos(0.5 * (lat_n + lat_o) * 0.017453292519943295);
  const float d = (float)sqrt(north * north + east * east);
  *spd_mps = d / dt;
  if (*spd_mps >= course_min_mps) {
    float c = (float)(atan2(east, north) * 57.29577951308232);
    if (c < 0.0f) c += 360.0f;
    *course_deg = c;
    *course_ok  = true;
  }
  return true;
}


// ============================================================
// 9. THE STOP QUALIFIER (Revision 2.1 §D as cut down by Revision 2.2 §B: STOPS ONLY)
// ============================================================
// WHY STOPS ONLY: the rider's fix reaches the buggy about 1 s late at 3 Hz and the buggy has no brake, so a carve toward
// the buggy and the first 2 s of a U-turn look the same in that signal; a trigger early enough to help in a U-turn fired
// in carves (the deleted running-at leg bound on up to 14 % of carving ticks) and gained under 1 ft in U-turns. A rider
// STOPPING in front of the buggy is different: his speed falls, and that is measurable early. So the ceiling may act only
// while this stands:
//   QUALIFY: his raw speed at or under stop_ratio x its maximum over the stop_window_ms before the estimate, on
//            stop_estimates consecutive estimates (each new rider fix is one estimate), AND c-dot >= stop_cdot_mps on the
//            qualifying estimate, AND his maximum over the floor_window_ms before it >= floor_mps (2.5 m/s = v_band: below
//            that the approach band already handles the closure, and a rider already stopped is not "stopping").
//              S-a (Rev 2.1 as built): stop_ratio 0.70 (a 30 % drop), stop_window_ms 1500, stop_estimates 1;
//              S-b (noise-proofed):    stop_ratio 0.60, stop_window_ms 2000, stop_estimates 2.
//   EXIT:    his raw speed back to >= exit_ratio x that 2 s maximum on exit_estimates consecutive estimates, OR c-dot
//            under exit_cdot_mps for exit_slow_ms.
// OWNER RULE: est_ok false (no closing estimate, or any input the caller judges bad) drops the qualification and the
// speed history - no ceiling, today's behaviour. With no speed estimate the leg cannot fire. The qualifier only ever
// ENABLES the v_max rule, which can only lower cap 3.
struct FmCvQualCfg {
  float    stop_ratio;        // S-a 0.70, S-b 0.60
  uint32_t stop_window_ms;    // S-a 1500, S-b 2000
  uint8_t  stop_estimates;    // S-a 1, S-b 2
  float    stop_cdot_mps;     // 1.0
  float    floor_mps;         // 2.5
  uint32_t floor_window_ms;   // 2000
  float    exit_ratio;        // 0.80
  uint8_t  exit_estimates;    // 2
  float    exit_cdot_mps;     // 0.5
  uint32_t exit_slow_ms;      // 1000
};
static const uint8_t FMCVQ_NONE = 0;   // not qualified
static const uint8_t FMCVQ_STOP = 2;   // the stop leg qualified it (2: the code the deleted two-leg version used for it)
static const uint8_t kFmCvQualHistLen = 8;
struct FmCvQual {
  bool     qualified;
  uint8_t  leg;                        // FMCVQ_*: ?diag
  uint32_t last_est_ms;                // the newest sample time of the last estimate looked at, 0 = none
  uint8_t  stop_n;                     // estimates in a row meeting the speed test
  uint8_t  exit_n;                     // while qualified: estimates in a row with his speed back
  uint32_t slow_since_ms;              // while qualified: the first of a run of estimates under exit_cdot_mps, 0 = none
  float    ref_mps;                    // his maximum over the floor window when it qualified (the exit test), < 0 = none
  uint32_t h_ms[kFmCvQualHistLen];     // his recent speed estimates: when ...
  float    h_mps[kFmCvQualHistLen];    // ... and how fast
  uint8_t  h_n, h_head;
};

// fmCvQualReset - not qualified, no history. Side effects: *q.
static inline void fmCvQualReset(FmCvQual *q)
{
  q->qualified = false; q->leg = FMCVQ_NONE; q->last_est_ms = 0; q->stop_n = 0; q->exit_n = 0; q->slow_since_ms = 0;
  q->ref_mps = -1.0f; q->h_n = 0; q->h_head = 0;
  for (uint8_t i = 0; i < kFmCvQualHistLen; i++) { q->h_ms[i] = 0; q->h_mps[i] = 0.0f; }
}

// fmCvQualMax - his highest speed estimate taken within window_ms before est_ms (the estimate itself excluded), or -1.
static inline float fmCvQualMax(const FmCvQual *q, uint32_t est_ms, uint32_t window_ms)
{
  float m = -1.0f;
  for (uint8_t i = 0; i < q->h_n; i++) {
    const uint8_t j = (uint8_t)((q->h_head + kFmCvQualHistLen - i) % kFmCvQualHistLen);
    if (q->h_ms[j] != est_ms && (uint32_t)(est_ms - q->h_ms[j]) <= window_ms && q->h_mps[j] > m) m = q->h_mps[j];
  }
  return m;
}

// fmCvQualStep - one Follow-Me tick of the qualifier.
// Inputs: now_ms; est_ok - a closing estimate exists and every input is good (see OWNER RULE above); est_ms - the newest
//   sample's time (fmCvNewestMs(): a new value is a new estimate, so the tests count estimates, not ticks); cdot_mps (from
//   fmCvClosing()); spd_ok / spd_mps (from fmCvRiderMotion()); cfg - the RX's constants. Returns: qualified this tick.
// Side effects: *q.
static inline bool fmCvQualStep(FmCvQual *q, uint32_t now_ms, bool est_ok, uint32_t est_ms, float cdot_mps,
                                bool spd_ok, float spd_mps, const FmCvQualCfg *cfg)
{
  if (!est_ok) { fmCvQualReset(q); return false; }
  if (est_ms != q->last_est_ms) {                                   // a new estimate
    q->last_est_ms = est_ms;
    const float pre   = fmCvQualMax(q, est_ms, cfg->stop_window_ms);  // the drop's reference
    const float floor = fmCvQualMax(q, est_ms, cfg->floor_window_ms);
    if (spd_ok) {
      q->h_head = (uint8_t)((q->h_head + 1) % kFmCvQualHistLen);
      q->h_ms[q->h_head] = est_ms; q->h_mps[q->h_head] = spd_mps;
      if (q->h_n < kFmCvQualHistLen) q->h_n++;
    }
    if (!q->qualified) {
      const bool low = spd_ok && pre > 0.0f && spd_mps <= cfg->stop_ratio * pre && floor >= cfg->floor_mps;
      if (low) { if (q->stop_n < 255) q->stop_n++; } else q->stop_n = 0;
      if (q->stop_n >= cfg->stop_estimates && cdot_mps >= cfg->stop_cdot_mps) {
        q->qualified = true; q->leg = FMCVQ_STOP; q->ref_mps = floor; q->exit_n = 0; q->slow_since_ms = 0;
      }
    } else {
      if (spd_ok && spd_mps >= cfg->exit_ratio * q->ref_mps) { if (q->exit_n < 255) q->exit_n++; } else q->exit_n = 0;
      if (cdot_mps < cfg->exit_cdot_mps) { if (q->slow_since_ms == 0) q->slow_since_ms = (est_ms != 0) ? est_ms : 1u; }
      else q->slow_since_ms = 0;
      if (q->exit_n >= cfg->exit_estimates) {                       // his speed is back: going again
        q->qualified = false; q->leg = FMCVQ_NONE; q->slow_since_ms = 0; q->stop_n = 0; q->exit_n = 0;
      }
    }
  }
  if (q->qualified && q->slow_since_ms != 0 && (uint32_t)(now_ms - q->slow_since_ms) >= cfg->exit_slow_ms) {
    q->qualified = false; q->leg = FMCVQ_NONE; q->slow_since_ms = 0; q->stop_n = 0; q->exit_n = 0;   // closure has ended
  }
  return q->qualified;
}

#endif // BREMOTE_FOLLOW_ME_CONVERGE_H
