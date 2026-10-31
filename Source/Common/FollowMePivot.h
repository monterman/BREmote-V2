// V2.5-Evo - 2026-10-31 - UNIT H FIX PASS (the delta check of unit H, rules P-1 and N-1): P-1 - the learned cap may go above
//   fm_align_cap only once the gyro has CONFIRMED the hull turns the commanded way, and never after a pair showing it turn
//   AGAINST the command (two per-pivot latches); N-1 - NOT TURNING's 10 deg is scaled by the smallest steering deflection
//   over its window (5 deg at half lock). See fmPvLearnStep(). No new constant.
// V2.5-Evo - 2026-10-31 - Follow-Me package 2 unit H (design audit 2026-10-31 §H, with the three conditions of the
//   2026-10-31 B-1 / F' close-out §4): THE SELF-TUNING PIVOT ON VESC 2's GYRO. New file. Pure, header-only: no Arduino,
//   no FreeRTOS, no globals. RTMState.ino calls these functions and does every write itself, in the F7 order;
//   Tools/tests/follow_me_pivot_test.cpp runs this exact code on the PC.
//
// ============================================================
// WHAT IT IS FOR - in plain words
// ============================================================
// When Follow-Me has to turn the buggy round to face its station, it holds the throttle down to fm_align_cap (50 on
// Andres's buggy) and asks the mixer for the one-motor pivot. That cap is a fixed number, so one buggy pivots briskly at
// it and another (heavier, weed on a prop, a different motor) barely turns. VESC 2 carries a gyro, and its yaw tells the
// RX how fast the hull is really turning. This unit uses that to tune the pivot throttle while the pivot runs, inside
// tight bounds, and it adds two checks that the gyro makes possible:
//   NOT TURNING - the pivot is commanded at fm_align_cap or more for 2 s and the hull has not turned 10 deg: stop asking
//                 for more (back to fm_align_cap). Weed, a fouled prop, a stalled motor, a buggy held by something.
//   WRONG WAY   - once the gyro's sign is known, the hull keeps turning AGAINST the command: Follow-Me ends, through the
//                 same steering end as the wrong-way net R2-S (code 12). It is the standstill half of R2-S: R2-S needs the
//                 GPS course, which a buggy pivoting at rest does not have.
//
// WHAT IT CAN NEVER DO (the owner's rules, and why each holds here):
//   - It never adds throttle. Everything it produces is a CAP: RTMState.ino puts it in the place of fm_align_cap in the
//     Follow-Me cap chain (cap 4), and calcPWM() applies min(trigger, every cap). Letting go still stops the buggy on the
//     next 10 ms pass. The cap is also clamped here to the trigger itself, and never exceeds 128 (50 %) - so the "fast
//     pivot" stays the low-speed, under-half-throttle tool Andres asked for.
//   - Bounds: [max(8, fm_align_cap / 2), min(128, 1.6 x fm_align_cap)] = 25-80 at fm_align_cap 50.
//   - Above fm_align_cap ONLY once the gyro's yaw sign is calibrated (WRONG WAY armed). Before that it can only learn
//     DOWNWARD. (Close-out §4 condition 3: the H-1 lesson applied to this unit itself.)
//   - Above fm_align_cap ONLY while the command has stayed on one side of centre since the pivot began and for at least
//     lead_ms. A pivot whose command changes side runs at fm_align_cap or less until it ends. (See CHATTER below.)
//   - Above fm_align_cap ONLY once VESC 2's gyro has confirmed, in this pivot, that the hull turns the way it is steered,
//     and never again in that pivot once it has seen the hull turn against the command (P-1, fmPvLearnStep()).
//   - With any capability gate failing - the IMU sample older than 600 ms, never parsed, not finite / |acc| < 0.5 g, a
//     rate above 200 deg/s - it returns fm_align_cap exactly: today's behaviour, to the bit.
//   - It runs only inside the envelope the caller decides: Follow-Me ACTIVE (not RTM, not RETURN, not the heading creep),
//     aligning with the pivot boost, the buggy under 4 mph on a fresh fix, and never inside min_dist + band of the rider.
//   - Learned per boot, in RAM only. No confStruct field, no settings wipe.
//
// ============================================================
// THE PIECES (each a small state machine the RX owns as a static)
// ============================================================
//   1. fmPvIngest / fmPvCapable - VESC 2's IMU samples (about 2 Hz) turned into a yaw RATE between two consecutive
//      samples (unwrapped, from the AHRS yaw and the two receipt times), and the capability gates.
//   2. fmPvBounds - the cap's bounds for this fm_align_cap and calibration state.
//   3. fmPvCalStep - THE YAW-SIGN CALIBRATION: does the gyro's yaw turn the same way as the GPS course? Each turn the
//      buggy makes at 8 km/h or more with the course turning at 10 deg/s or more votes once; 8 of the last 10 votes
//      agreeing gives the sign (+1, or -1 for a VESC mounted the other way up). Fewer: not calibrated, WRONG WAY off.
//   4. fmPvWwStep - WRONG WAY.
//   5. fmPvLearnStep - H-1, the learned cap, and NOT TURNING.
//
// SIGN CONVENTION. Every "side" here is the steering command's: command - 127 > 0 = steer right = the heading should
// INCREASE (clockwise), the same convention as R2-S (Common/FollowMePause.h section 6) - the controller's byte, BEFORE the
// mixer's steering_inverted. The gyro's own yaw sign depends on how VESC 2 is mounted; the calibration's sign turns it into
// "clockwise positive". Learning uses |rate| only, so it does not need the sign at all.
//
// CHATTER (spec silent; the choice made here). A buggy whose steering is mirrored, starting with its target almost
// directly behind, does not hold the command on one side: the error sits near 180 deg and the command flips across the
// wrap every second or so. Neither WRONG WAY ("against the command for 1.5 s") nor NOT TURNING ("commanded one way for
// 2 s") can see that state, exactly as R2-S's S1 cannot. So the cap above fm_align_cap is withdrawn for the rest of any
// pivot whose command has changed side: such a pivot runs at today's fm_align_cap (or the lower learned value). A healthy
// pivot changes side only when its target swings across behind it (a rider U-turning past it); it then finishes at
// fm_align_cap, as every pivot did before this unit.
#ifndef BREMOTE_FOLLOW_ME_PIVOT_H
#define BREMOTE_FOLLOW_ME_PIVOT_H

#include <stdint.h>
#include <math.h>

// ============================================================
// THE NUMBERS - the RX's are kFmPivotCfg in RTMState.ino (the host test checks that line)
// ============================================================
struct FmPivotCfg {
  // capability gates - H does nothing (fm_align_cap, no checks) unless all hold
  uint32_t imu_fresh_ms;   // ms; the newest IMU sample no older than this (600)
  float    rate_max_dps;   // deg/s; a rate above this is not believed (200)
  uint32_t pair_min_ms;    // ms; two samples closer together than this give no rate (150)
  uint32_t pair_max_ms;    // ms; ... nor two further apart than this - more than one lost poll at 2 Hz (1100)
  uint32_t tick_gap_ms;    // ms; a caller that has not ticked for longer than this has lost the thread: start afresh (250)
  // H-1, the learned cap
  float    lo_frac;        // the lower bound is max(cap_floor, lo_frac x fm_align_cap) ... (0.5)
  uint8_t  cap_floor;      // ... (8 counts)
  float    hi_frac;        // the upper bound is min(cap_ceiling, hi_frac x fm_align_cap) ... (1.6)
  uint8_t  cap_ceiling;    // ... (128 counts = 50 %: the fast pivot stays a low-speed, under-half-throttle tool)
  float    omega_dps;      // deg/s; the target pivot rate (30)
  float    up_frac;        // below up_frac x omega -> one step up (0.7: under 21 deg/s)
  float    down_frac;      // above down_frac x omega -> one step down (1.3: over 39 deg/s)
  uint8_t  step;           // counts per step (10)
  uint32_t settle_ms;      // ms; no step in a pivot's first second (1000)
  float    carry;          // the next pivot starts at fm_align_cap + carry x (last learned - fm_align_cap) (0.8)
  // the commanded side (NOT TURNING, WRONG WAY, the step up, the cap above fm_align_cap)
  int16_t  cmd_min;        // counts beyond centre that count as steering to one side (32 = 25 %, as R2-S)
  uint32_t lead_ms;        // ms; the command on that side this long before anything is judged on it (500, as R2-S)
  // NOT TURNING
  uint32_t nt_ms;          // ms; the window (2000)
  float    nt_deg;         // deg; the hull turned less than this over the window -> NOT TURNING (10)
  // the yaw-sign calibration (the "8 km/h or more" speed test is the caller's: cog_ok)
  float    cal_cog_dps;    // deg/s; a sample pair counts as turning when the course turned at least this fast (10)
  float    cal_min_deg;    // deg; a turn votes only when the yaw AND the course each moved at least this much (5)
  uint8_t  cal_min_pairs;  // sample pairs; a turn is at least this long (2: about 1 s) - one noisy pair never votes
  uint8_t  cal_need;       // votes that must agree ... (8)
  uint8_t  cal_of;         // ... of the last this many turns (10; at most kFmPvCalMax)
  // WRONG WAY
  uint32_t ww_ms;          // ms; the hull turning against the command this long ... (1500)
  float    ww_dps;         // deg/s; ... at least this fast (5)
};

// fmPvWrap180 - an angle difference folded into -180..+180 deg. Pure.
static inline float fmPvWrap180(float d)
{
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// fmPvSide - which side of centre a command offset is on: +1 right, -1 left, 0 inside cmd_min. Pure.
static inline int8_t fmPvSide(int16_t cmd_off, int16_t cmd_min)
{
  if (cmd_off >= cmd_min)  return (int8_t)1;
  if (cmd_off <= -cmd_min) return (int8_t)-1;
  return (int8_t)0;
}

// ============================================================
// 1. THE IMU RATE STREAM AND THE CAPABILITY GATES
// ============================================================
// One VESC 2 IMU sample arrives about every 500 ms (VESC.ino pollVesc2ImuIfDue(), inside the VESC mux visit). The rate is
// the change of the AHRS yaw between two consecutive samples over the time between their receipt, unwrapped through
// +/-180 deg. A rate is believed only when both samples are plausible (finite, |acc| >= 0.5 g - vescImuPlausible()), they
// are 150-1100 ms apart, and it is 200 deg/s or less. yaw_unw is the running unwrapped yaw over the chain of believed
// pairs; seg counts the breaks in that chain.
struct FmPvRate {
  bool     have;       // a previous plausible sample is held (t_ms, yaw_deg)
  uint32_t seen_ms;    // the receipt stamp of the newest sample taken - the sample's identity; 0 = none yet
  uint32_t t_ms;       // the previous sample's stamp
  float    yaw_deg;    // ... and its yaw (deg)
  float    yaw_unw;    // the running unwrapped yaw (deg)
  bool     rate_ok;    // the newest pair gave a believable rate
  float    rate_dps;   // that rate, deg/s, in the IMU's OWN sign
  float    dyaw_deg;   // that pair's yaw change (deg)
  uint32_t t0_ms;      // that pair's older sample ...
  uint32_t t1_ms;      // ... and newer sample
  uint32_t seg;        // +1 at every break in the chain of believed pairs
};

// fmPvRateReset - forget the stream. Side effects: *r.
static inline void fmPvRateReset(FmPvRate* r)
{
  r->have = false; r->seen_ms = 0; r->t_ms = 0; r->yaw_deg = 0.0f; r->yaw_unw = 0.0f;
  r->rate_ok = false; r->rate_dps = 0.0f; r->dyaw_deg = 0.0f; r->t0_ms = 0; r->t1_ms = 0; r->seg = 0;
}

// fmPvIngest - take the IMU's newest sample if it is new.
// Inputs: ever_ok - the IMU has answered this session; sample_ms - the newest sample's receipt stamp; yaw_deg - its yaw;
//         plausible - finite with |acc| >= 0.5 g (the caller's vescImuPlausible()); *c.
// Returns: true when a NEW sample was taken on this call (rate_ok then says whether it made a believable rate).
// Side effects: *r only.
static inline bool fmPvIngest(FmPvRate* r, bool ever_ok, uint32_t sample_ms, float yaw_deg, bool plausible,
                              const FmPivotCfg* c)
{
  if (!ever_ok) return false;
  if (r->seen_ms != 0 && sample_ms == r->seen_ms) return false;   // the same sample as before
  r->seen_ms = (sample_ms != 0) ? sample_ms : 1;
  if (!plausible || !isfinite(yaw_deg)) {
    r->have = false; r->rate_ok = false; r->seg++;
    return true;
  }
  bool ok = false;
  if (r->have) {
    const uint32_t dt = (uint32_t)(sample_ms - r->t_ms);
    if (dt >= c->pair_min_ms && dt <= c->pair_max_ms) {
      const float d    = fmPvWrap180(yaw_deg - r->yaw_deg);
      const float rate = d * 1000.0f / (float)dt;
      if (fabsf(rate) <= c->rate_max_dps) {
        ok = true;
        r->rate_dps = rate; r->dyaw_deg = d; r->t0_ms = r->t_ms; r->t1_ms = sample_ms;
        r->yaw_unw += d;
      }
    }
  }
  if (!ok) r->seg++;
  r->rate_ok = ok;
  r->have = true; r->t_ms = sample_ms; r->yaw_deg = yaw_deg;
  return true;
}

// fmPvCapable - may H act on this tick? The newest pair was believed and its newer sample is no older than imu_fresh_ms.
// A stamp newer than now (taken after the caller read its clock) reads as "just now". Pure.
static inline bool fmPvCapable(const FmPvRate* r, uint32_t now_ms, const FmPivotCfg* c)
{
  if (!r->rate_ok) return false;
  const int32_t age = (int32_t)(now_ms - r->t1_ms);
  return (age <= 0) || ((uint32_t)age <= c->imu_fresh_ms);
}

// ============================================================
// 2. THE BOUNDS
// ============================================================
// fmPvBounds - [max(cap_floor, lo_frac x align_cap), min(cap_ceiling, hi_frac x align_cap)] = [max(8, cap / 2),
// min(128, 1.6 x cap)] with the RX's numbers, and not above align_cap until the yaw sign is calibrated (close-out §4
// condition 3). At 50: 25-80 calibrated, 25-50 not; at 80: 40-128; at 13: 8-20. Both bounds round DOWN.
// Inputs: align_cap (fmAlignCapValue(): 8-80), calibrated, *c. Outputs: *lo, *hi. Pure.
static inline void fmPvBounds(uint8_t align_cap, bool calibrated, const FmPivotCfg* c, uint8_t* lo, uint8_t* hi)
{
  float lf = c->lo_frac * (float)align_cap;
  float hf = c->hi_frac * (float)align_cap;
  if (lf < 0.0f) lf = 0.0f;
  if (hf > 255.0f) hf = 255.0f;
  if (hf < 0.0f) hf = 0.0f;
  uint16_t l = (uint16_t)lf;
  if (l < c->cap_floor) l = c->cap_floor;
  uint16_t h = (uint16_t)hf;
  if (h > c->cap_ceiling) h = c->cap_ceiling;
  if (!calibrated && h > align_cap) h = align_cap;
  if (h < l) h = l;   // only for an align cap under 8, which fmAlignCapValue() never returns
  *lo = (uint8_t)l; *hi = (uint8_t)h;
}

// ============================================================
// 3. THE YAW-SIGN CALIBRATION
// ============================================================
// Call once per NEW IMU sample (fmPvIngest() returned true), with the buggy's GPS course at that moment.
// A sample PAIR qualifies when its rate was believed, the course was usable at BOTH samples (cog_ok: fresh, not frozen,
// the buggy at 8 km/h or more - the caller's test) and the course turned at cal_cog_dps or faster over the pair. Consecutive
// qualifying pairs turning the same way make one TURN; a pair that does not qualify, or turns the other way, closes it. A
// closed turn of at least cal_min_pairs pairs in which the yaw and the course each moved cal_min_deg or more votes once:
// +1 if the yaw moved the same way as the course, -1 if the other way. A turn in which the yaw hardly moved does not vote
// (a frozen or wrongly-mounted IMU never calibrates). sign: +1 / -1 once cal_need of the last cal_of votes agree, else 0.
// Since boot, RAM only; the sign is re-derived after every vote, so it can also be LOST if the votes stop agreeing.
static const uint8_t kFmPvCalMax = 16;
struct FmPvCal {
  int8_t   vote[kFmPvCalMax];  // ring of the last cal_of votes
  uint8_t  n;                  // votes held
  uint8_t  head;               // where the next vote goes
  uint8_t  pairs;              // the turn being collected: qualifying pairs so far ...
  int8_t   dir;                // ... the way its course turns (+1 / -1) ...
  float    dyaw;               // ... the yaw change over it ...
  float    dcog;               // ... and the course change over it
  bool     cog_have;           // the course at the previous IMU sample was usable ...
  float    cog_prev;           // ... and its value
  int8_t   sign;               // 0 = not calibrated, +1 / -1
  uint32_t turns;              // turns that voted since boot (?diag)
};

// fmPvCalReset - forget everything (boot state). Side effects: *k.
static inline void fmPvCalReset(FmPvCal* k)
{
  for (uint8_t i = 0; i < kFmPvCalMax; i++) k->vote[i] = 0;
  k->n = 0; k->head = 0; k->pairs = 0; k->dir = 0; k->dyaw = 0.0f; k->dcog = 0.0f;
  k->cog_have = false; k->cog_prev = 0.0f; k->sign = 0; k->turns = 0;
}

// fmPvCalCounts - of the votes held, how many say +1 and how many -1. Pure.
static inline void fmPvCalCounts(const FmPvCal* k, uint8_t* plus, uint8_t* minus)
{
  uint8_t p = 0, m = 0;
  for (uint8_t i = 0; i < k->n; i++) { if (k->vote[i] > 0) p++; else if (k->vote[i] < 0) m++; }
  *plus = p; *minus = m;
}

// fmPvCalClose - end the turn being collected; it votes if it was long enough and both moved enough. Side effects: *k.
static inline void fmPvCalClose(FmPvCal* k, const FmPivotCfg* c)
{
  if (k->pairs >= c->cal_min_pairs && fabsf(k->dyaw) >= c->cal_min_deg && fabsf(k->dcog) >= c->cal_min_deg) {
    const uint8_t of = (c->cal_of > kFmPvCalMax) ? kFmPvCalMax : (c->cal_of == 0 ? (uint8_t)1 : c->cal_of);
    k->vote[k->head] = (k->dyaw * k->dcog > 0.0f) ? (int8_t)1 : (int8_t)-1;
    k->head = (uint8_t)((k->head + 1u) % of);
    if (k->n < of) k->n++;
    k->turns++;
    uint8_t p = 0, m = 0;
    fmPvCalCounts(k, &p, &m);
    k->sign = (p >= c->cal_need) ? (int8_t)1 : (m >= c->cal_need) ? (int8_t)-1 : (int8_t)0;
  }
  k->pairs = 0; k->dir = 0; k->dyaw = 0.0f; k->dcog = 0.0f;
}

// fmPvCalStep - one NEW IMU sample. Inputs: *r (fmPvIngest() just took the sample), cog_ok / cog_deg (the course now),
// *c. Side effects: *k only.
static inline void fmPvCalStep(FmPvCal* k, const FmPvRate* r, bool cog_ok, float cog_deg, const FmPivotCfg* c)
{
  bool   qual = false;
  int8_t dir  = 0;
  float  dcog = 0.0f;
  if (r->rate_ok && cog_ok && k->cog_have && r->t1_ms == r->seen_ms) {
    dcog = fmPvWrap180(cog_deg - k->cog_prev);
    const float dt_s = (float)(uint32_t)(r->t1_ms - r->t0_ms) / 1000.0f;
    if (dt_s > 0.0f && fabsf(dcog) / dt_s >= c->cal_cog_dps) { qual = true; dir = (dcog > 0.0f) ? (int8_t)1 : (int8_t)-1; }
  }
  if (!qual || (k->pairs > 0 && dir != k->dir)) fmPvCalClose(k, c);
  if (qual) {
    if (k->pairs < 255) k->pairs++;
    k->dir = dir; k->dyaw += r->dyaw_deg; k->dcog += dcog;
  }
  k->cog_have = cog_ok;
  if (cog_ok) k->cog_prev = cog_deg;
}

// ============================================================
// 4. WRONG WAY
// ============================================================
// Judged only on ticks the caller says (judged): Follow-Me ACTIVE in a commanded pivot (aligning with the boost, under
// 4 mph, not the creep), not a stick takeover or the second after one, past the first 0.5 s of the engagement, the
// distance and the trigger good (R2-S's hard_ok). Also only with the sign calibrated, H capable, and the command on one
// side. Anything else - or a tick gap - clears it.
// Rule: after the command has been on one side for lead_ms, every new sample pair whose yaw rate (made clockwise-positive
// by the calibrated sign) runs AGAINST that side at ww_dps or more extends a run; a pair that does not ends it. When the
// run covers ww_ms (from the older sample of its first pair to the newer sample of its latest) -> WRONG WAY.
// Returns true on the verdict (and clears itself). Side effects: *w only. Nothing here writes a cap, a byte or a state.
struct FmPvWw {
  int8_t   side;       // the side the command is on (0 = none)
  uint32_t side_ms;    // since when
  uint32_t opp_ms;     // the start of the run of against-the-command pairs; 0 = none
  uint32_t last_ms;    // the last call; 0 = never
};

// fmPvWwReset - clear the run and the side (the tick-gap memory is kept). Side effects: *w.
static inline void fmPvWwReset(FmPvWw* w)
{
  w->side = 0; w->side_ms = 0; w->opp_ms = 0;
}

static inline bool fmPvWwStep(FmPvWw* w, uint32_t now_ms, bool judged, int16_t cmd_off, int8_t sign, bool capable,
                              bool new_sample, const FmPvRate* r, const FmPivotCfg* c)
{
  const bool gap = (w->last_ms != 0) && ((uint32_t)(now_ms - w->last_ms) > c->tick_gap_ms);
  w->last_ms = (now_ms != 0) ? now_ms : 1;
  const int8_t side = fmPvSide(cmd_off, c->cmd_min);
  if (!judged || sign == 0 || !capable || side == 0 || gap) { fmPvWwReset(w); return false; }
  if (side != w->side) { w->side = side; w->side_ms = now_ms; w->opp_ms = 0; }
  if (new_sample && r->rate_ok && r->t1_ms == r->seen_ms) {
    const float cw = (float)sign * r->rate_dps;                                   // clockwise-positive yaw rate
    const bool after_lead = (int32_t)(r->t0_ms - (w->side_ms + c->lead_ms)) >= 0;  // the whole pair after the lead-in
    if (after_lead && (float)side * cw <= -c->ww_dps) {
      if (w->opp_ms == 0) w->opp_ms = (r->t0_ms != 0) ? r->t0_ms : 1;
    } else {
      w->opp_ms = 0;
    }
    if (w->opp_ms != 0 && (uint32_t)(r->t1_ms - w->opp_ms) >= c->ww_ms) { fmPvWwReset(w); return true; }
  }
  return false;
}

// ============================================================
// 5. H-1, THE LEARNED CAP, AND NOT TURNING
// ============================================================
// Call EXACTLY ONCE PER TICK, every tick (env false when not in the envelope). Inputs:
//   env       - this tick is inside the learning envelope (Follow-Me ACTIVE, aligning with the boost, the buggy under
//               4 mph on a fresh fix, at least min_dist + band from the rider; not RTM, RETURN or the creep)
//   align_cap - fmAlignCapValue() (8-80);  trig_cmd - the trigger as a command, thrByteToCmd(thr_received) (0-255)
//   applied   - the throttle calcPWM() last applied, after every cap and the ramp (g_applied_thr_cmd)
//   cmd_off   - this tick's steering command - 127;  capable - fmPvCapable();  sign - the calibration's (0 = none)
//   new_sample, *r - this tick's IMU sample, if one arrived
// Behaviour:
//   PIVOT     - the first in-envelope tick starts one: the cap starts at align_cap + carry x (last - align_cap), or at
//               align_cap for the first pivot this boot, inside the bounds. It ends (and its cap is remembered) on the
//               first tick outside the envelope, or after a tick gap. A capability loss does NOT end it: on those ticks
//               H does nothing (align_cap out, no step, no check) and the pivot carries on when the IMU is back.
//   STEPS     - on each new believed sample pair lying wholly inside the pivot and ending settle_ms or more after its
//               start: |rate| above down_frac x omega -> one step DOWN; below up_frac x omega -> one step UP, but only
//               while NOT TURNING has not fired in this pivot, the command has stayed on one side (the CHATTER rule)
//               since lead_ms before the pair, the trigger is above the cap (otherwise raising it changes nothing) and
//               the throttle the buggy actually got is within 2 counts of it (another cap is not holding it lower).
//               A step up that would take the cap ABOVE align_cap also needs the direction CONFIRMED and never AGAINST
//               (P-1, below); without that it stops at align_cap. Clamped to the bounds after every step and on every tick.
//   P-1       - V2.5-Evo - 2026-10-31 - THE BOOST NEEDS PROOF OF DIRECTION (delta check of unit H, rule P-1). Two per-pivot
//               latches, cleared when a pivot starts and ends. On each new believed pair lying wholly after the lead-in on
//               the current side, with the yaw sign calibrated, BEFORE its step: side x sign x rate >= +ww_dps sets
//               CONFIRMED (the hull turns the way it is told); <= -ww_dps sets AGAINST (it turns the other way). Learning
//               on |rate| alone let a buggy with mirrored steering reach 1.6 x align_cap while it pivoted the wrong way.
//   NOT TURNING - while the command stays on one side, past lead_ms, with the applied throttle at align_cap or more on
//               every tick: if the yaw moved less than nt_deg x d over the last nt_ms -> the cap goes back to align_cap, no
//               more steps up in this pivot, the event is reported. Once per pivot.
//               V2.5-Evo - 2026-10-31 - N-1: d is the SMALLEST |command - 127| / 127 recorded over the window's samples,
//               clamped to 0.5-1.0: 10 deg at full lock, 5 deg at half lock. The P term gives about half lock at the tail
//               of every pivot, and a healthy 8 deg/s buggy turns only about 8.7 deg in 2 s there; the smallest deflection
//               is the safe one, because the hull's rate lags a rising command.
//   OUT       - the cap for cap 4: align_cap outside the envelope or when not capable; otherwise the learned cap, held to
//               align_cap unless the command has stayed on one side for lead_ms with no change of side in this pivot
//               (CHATTER) AND the direction is CONFIRMED and not AGAINST (P-1), and never above the trigger.
// Returns FmPvOut. Side effects: *L only.
static const uint8_t FMPV_NONE        = 0;   // H is not acting this tick
static const uint8_t FMPV_PIVOT       = 1;   // in a learning pivot, no step this sample
static const uint8_t FMPV_UP          = 2;   // stepped up
static const uint8_t FMPV_DOWN        = 3;   // stepped down
static const uint8_t FMPV_NOT_TURNING = 4;   // NOT TURNING fired
static const uint8_t FMPV_WRONG_WAY   = 5;   // WRONG WAY ended Follow-Me (the caller marks it: fmPvMarkEvent())
static const uint32_t kFmPvEventHoldMs = 500;   // ms an event code stays the reported state, so a 3 Hz log row sees it
static const uint8_t  kFmPvRingLen     = 8;     // NOT TURNING's yaw history: 8 samples = 4 s at 2 Hz

struct FmPvLearn {
  bool     in_pivot;     // a learning pivot is running
  uint32_t start_ms;     // since when
  int16_t  cap;          // the learned cap, counts
  bool     have_last;    // a pivot has ended this boot ...
  int16_t  last;         // ... with this cap
  bool     nt_latched;   // NOT TURNING fired in this pivot
  bool     flipped;      // the command changed side in this pivot (CHATTER rule)
  bool     confirmed;    // P-1: a pair in this pivot showed the hull turning the commanded way at ww_dps or more
  bool     against;      // P-1: a pair in this pivot showed it turning AGAINST the command at ww_dps or more
  int8_t   seen_side;    // the last non-zero side seen in this pivot
  int8_t   side;         // the side the command is on now (0 = none)
  uint32_t side_ms;      // since when
  uint8_t  rn;                       // NOT TURNING's history: samples held, oldest first ...
  uint32_t rt[kFmPvRingLen];
  float    ry[kFmPvRingLen];
  uint8_t  rc[kFmPvRingLen];         // ... and |command - 127| on the tick each was recorded (N-1)
  uint32_t nt_from_ms;               // ... none older than this: the last tick its preconditions broke
  uint8_t  event;        // the last event code ...
  uint32_t event_ms;     // ... and when
  uint32_t last_ms;      // the last call; 0 = never
};

struct FmPvOut {
  uint8_t cap;           // cap 4 for this tick
  uint8_t state;         // FMPV_* for the log and ?diag
  bool    nt_fired;      // NOT TURNING fired on this tick
  bool    stepped;       // a step (up or down) was taken on this tick
};

// fmPvLearnReset - boot state. Side effects: *L.
static inline void fmPvLearnReset(FmPvLearn* L)
{
  L->in_pivot = false; L->start_ms = 0; L->cap = 0; L->have_last = false; L->last = 0;
  L->nt_latched = false; L->flipped = false; L->confirmed = false; L->against = false; L->seen_side = 0; L->side = 0;
  L->side_ms = 0;
  L->rn = 0; for (uint8_t i = 0; i < kFmPvRingLen; i++) { L->rt[i] = 0; L->ry[i] = 0.0f; L->rc[i] = 0; }
  L->nt_from_ms = 0;
  L->event = FMPV_NONE; L->event_ms = 0; L->last_ms = 0;
}

// fmPvMarkEvent - record an event code for the log / ?diag (WRONG WAY is marked by the caller). Side effects: *L.
static inline void fmPvMarkEvent(FmPvLearn* L, uint8_t code, uint32_t now_ms)
{
  L->event = code; L->event_ms = (now_ms != 0) ? now_ms : 1;
}

// fmPvState - the reported state: a recent event (within kFmPvEventHoldMs), else PIVOT while acting, else NONE. Pure.
static inline uint8_t fmPvState(const FmPvLearn* L, uint32_t now_ms, bool acting)
{
  if (L->event != FMPV_NONE && L->event_ms != 0 && (uint32_t)(now_ms - L->event_ms) < kFmPvEventHoldMs) return L->event;
  return acting ? FMPV_PIVOT : FMPV_NONE;
}

// fmPvEndPivot - remember the pivot's cap and close it. Side effects: *L.
static inline void fmPvEndPivot(FmPvLearn* L)
{
  if (L->in_pivot) { L->last = L->cap; L->have_last = true; }
  L->in_pivot = false; L->nt_latched = false; L->flipped = false; L->confirmed = false; L->against = false;
  L->seen_side = 0; L->side = 0; L->side_ms = 0; L->rn = 0;
}

static inline FmPvOut fmPvLearnStep(FmPvLearn* L, uint32_t now_ms, bool env, uint8_t align_cap, uint8_t trig_cmd,
                                    uint8_t applied, int16_t cmd_off, bool capable, int8_t sign, bool new_sample,
                                    const FmPvRate* r, const FmPivotCfg* c)
{
  FmPvOut o; o.cap = align_cap; o.state = FMPV_NONE; o.nt_fired = false; o.stepped = false;
  const bool gap = (L->last_ms != 0) && ((uint32_t)(now_ms - L->last_ms) > c->tick_gap_ms);
  L->last_ms = (now_ms != 0) ? now_ms : 1;
  if (L->in_pivot && (gap || !env)) fmPvEndPivot(L);
  if (!env) { o.state = fmPvState(L, now_ms, false); return o; }

  uint8_t lo = 0, hi = 0;
  fmPvBounds(align_cap, sign != 0, c, &lo, &hi);
  if (!L->in_pivot) {
    int16_t start = align_cap;
    if (L->have_last) start = (int16_t)(align_cap + (int16_t)(c->carry * (float)(L->last - (int16_t)align_cap)));   // toward align_cap
    L->in_pivot = true; L->start_ms = now_ms; L->cap = start;
    L->nt_latched = false; L->flipped = false; L->confirmed = false; L->against = false;
    L->seen_side = 0; L->side = 0; L->side_ms = 0; L->rn = 0;
    L->nt_from_ms = now_ms;
  }
  if (L->cap < (int16_t)lo) L->cap = lo;
  if (L->cap > (int16_t)hi) L->cap = hi;

  // The commanded side, the change-of-side latch (CHATTER) and the NOT TURNING history's own preconditions.
  const int8_t side = fmPvSide(cmd_off, c->cmd_min);
  if (side != 0) {
    if (L->seen_side != 0 && side != L->seen_side) L->flipped = true;
    L->seen_side = side;
  }
  if (side != L->side) { L->side = side; L->side_ms = now_ms; L->rn = 0; L->nt_from_ms = now_ms; }
  const bool settled = (L->side != 0) && ((uint32_t)(now_ms - L->side_ms) >= c->lead_ms);
  if (!capable || applied < align_cap || L->side == 0) { L->rn = 0; L->nt_from_ms = now_ms; }   // NOT TURNING needs every tick commanded at the cap

  if (!capable) {                                   // the gates failed: today's behaviour, exactly
    o.cap = align_cap; o.state = fmPvState(L, now_ms, false);
    return o;
  }

  if (new_sample && r->rate_ok && r->t1_ms == r->seen_ms) {
    const float w = fabsf(r->rate_dps);
    // ---- the step ----
    const bool in_pivot_pair = ((int32_t)(r->t0_ms - L->start_ms) >= 0) &&
                               ((uint32_t)(r->t1_ms - L->start_ms) >= c->settle_ms);
    const bool pair_on_side  = (L->side != 0) && ((int32_t)(r->t0_ms - (L->side_ms + c->lead_ms)) >= 0);
    // ---- P-1: does the hull turn the way it is told? (before this pair's step) ----
    if (pair_on_side && sign != 0) {
      const float with_cmd = (float)L->side * (float)sign * r->rate_dps;   // > 0: turning the commanded way
      if (with_cmd >= c->ww_dps)  L->confirmed = true;
      if (with_cmd <= -c->ww_dps) L->against   = true;
    }
    const bool direction_ok = L->confirmed && !L->against;
    if (in_pivot_pair) {
      if (w > c->down_frac * c->omega_dps) {
        L->cap = (int16_t)(L->cap - c->step); o.stepped = true; fmPvMarkEvent(L, FMPV_DOWN, now_ms);
      } else if (w < c->up_frac * c->omega_dps && !L->nt_latched && !L->flipped && pair_on_side &&
                 trig_cmd > L->cap && (int16_t)applied + 2 >= L->cap) {
        int16_t next = (int16_t)(L->cap + c->step);
        // P-1: above align_cap only with the direction confirmed and never against; otherwise the step stops at align_cap
        if (next > (int16_t)align_cap && !direction_ok) next = (L->cap < (int16_t)align_cap) ? (int16_t)align_cap : L->cap;
        if (next != L->cap) { L->cap = next; o.stepped = true; fmPvMarkEvent(L, FMPV_UP, now_ms); }
      }
      if (L->cap < (int16_t)lo) L->cap = lo;
      if (L->cap > (int16_t)hi) L->cap = hi;
    }
    // ---- NOT TURNING ----
    if (!L->nt_latched && pair_on_side && applied >= align_cap) {
      // the window may start at the pair's older sample only if that sample lies inside the unbroken span
      const int ac_i = (cmd_off < 0) ? -(int)cmd_off : (int)cmd_off;
      const uint8_t abs_cmd = (uint8_t)((ac_i > 127) ? 127 : ac_i);
      if (L->rn == 0 && (int32_t)(r->t0_ms - L->nt_from_ms) >= 0) {
        L->rt[0] = r->t0_ms; L->ry[0] = r->yaw_unw - r->dyaw_deg; L->rc[0] = abs_cmd; L->rn = 1;
      }
      if (L->rn == kFmPvRingLen) {
        for (uint8_t i = 1; i < kFmPvRingLen; i++) { L->rt[i - 1] = L->rt[i]; L->ry[i - 1] = L->ry[i]; L->rc[i - 1] = L->rc[i]; }
        L->rn--;
      }
      L->rt[L->rn] = r->t1_ms; L->ry[L->rn] = r->yaw_unw; L->rc[L->rn] = abs_cmd; L->rn++;
      // the newest held sample that is at least nt_ms older than this one
      int k = -1;
      for (int i = (int)L->rn - 2; i >= 0; i--) {
        if ((uint32_t)(r->t1_ms - L->rt[i]) >= c->nt_ms) { k = i; break; }
      }
      // N-1: the threshold scaled by the smallest deflection over the window (entries k .. newest), clamped 0.5-1.0
      float d = 1.0f;
      if (k >= 0) {
        uint8_t mn = 127;
        for (int i = k; i < (int)L->rn; i++) if (L->rc[i] < mn) mn = L->rc[i];
        d = (float)mn / 127.0f;
        if (d < 0.5f) d = 0.5f;
        if (d > 1.0f) d = 1.0f;
      }
      if (k >= 0 && fabsf(r->yaw_unw - L->ry[k]) < c->nt_deg * d) {
        L->cap = align_cap; L->nt_latched = true; L->rn = 0;
        o.nt_fired = true; fmPvMarkEvent(L, FMPV_NOT_TURNING, now_ms);
        if (L->cap < (int16_t)lo) L->cap = lo;
        if (L->cap > (int16_t)hi) L->cap = hi;
      }
    }
  }

  // ---- the cap out ----
  int16_t out = L->cap;
  if (out > (int16_t)align_cap && (!settled || L->flipped)) out = align_cap;   // CHATTER: above align_cap only on one side
  if (out > (int16_t)align_cap && !(L->confirmed && !L->against)) out = align_cap;   // P-1: ... and only once the gyro confirms it
  if (out > (int16_t)trig_cmd) out = trig_cmd;                                 // never above the trigger
  if (out < 0) out = 0;
  o.cap = (uint8_t)out;
  o.state = fmPvState(L, now_ms, true);
  return o;
}

// ============================================================
// BENCH ROWS (the unit's acceptance is bench and water work - the host test checks the rules, not the pivot plant)
// ============================================================
//   B-H1  Follow-Me armed (or a level-6 log recording), on the bench: turn the buggy clockwise by hand, then counter-
//         clockwise. ?diag "FM pivot" shows the IMU rate's sign each way. On the first ride (8 or more turns at 5 mph or
//         more) ?diag must read "yaw sign +1" if clockwise read positive, "-1" if it read negative, with 8+ of 10 agreeing.
//   B-H2  The RX GPS sentence rate while Follow-Me is armed (the IMU now polled at 2 Hz) stays at 90 % or more of its
//         configured rate: ?diag's GPS sentence rate armed against disarmed.
//   B-H3  The bench spin plus 10 pivots: ?diag's learned range since boot stays inside its bounds (25-80 at 50), and
//         NOT TURNING and WRONG WAY stay at 0 on the healthy buggy.
#endif // BREMOTE_FOLLOW_ME_PIVOT_H
