// V2.5-Evo - 2026-10-21 - owner rule (taking control): T40 - on a remote that is not in control only a squeeze from zero
//   or a COMPLETED arm gesture (its SUMMON) takes control; a tap, a gear change, the display cycle, an early release,
//   a lock or any other meta never does, even from a claimable owner. T39 (full audit F-1) the boot-ID packets at an
//   owner edge.
// V2.5-Evo - 2026-10-21 - E1-E4 (owner order): T38 - the number of the remote in control at reply index 20: the wire, the
//   statuses that show it, who counts as in control (owner, claim token), the send rule (REFUSED always, a change at once,
//   else every other reply), the model's replies with two and three remotes (take-back, token, nobody -> E9), SINGLE never,
//   the remote's hold and freshness, and a random three-remote property (never another remote's number).
// V2.5-Evo - 2026-10-21 - STEP-3 AUDIT P3-1: T16 and T29 judge the take-back / summon answer on the FIRST owner arrival: a
//   grant at send + 0.4 s polled at send + 3 s is OWNER, no owner status is TIMEOUT, the window edge, a control that came
//   and went, the millis() wrap, and the late summon grant stays late.
//   P3-4: T21 checks mtxListChangeActions() (zero whenever MULTI is involved, the takeover limit once on MULTI -> SINGLE) and
//   that the remaining remote's held trigger then meets the limit (~25 % rising to full over 4 s) instead of full authority.
// V2.5-Evo - 2026-10-20 - REMOTE STEP (TX SW29): T34 the activity repeat (X-1 with the owner's widening: from the first
//   press, every 2 s or less, through a combo gap and the RTM ceremony, owner-only, never in SINGLE, never with RTM
//   active or an 0xF1/1 queued), T35 the E9-or-St choice for a refused gesture, T36 the ownership dot and the non-owner
//   test the summon path uses. T21 also checks the receiver's released_rebind counter (step-2 audit Z-7 a). T37 the
//   remote's validator pieces its cfgValidateCrossField() runs on every intake path (the link record, the three clamps).
// V2.5-Evo - 2026-10-20 - RECEIVER STEP: X-10 (c) "subtract-only whatever the order" - a model of one motor-task pass
//   (pwmPass: the trigger byte read first, the sticky return cap, the takeover limit) can be slotted between ANY two
//   hook calls of a frame; T21 (a lock under a running return) and T26 (a claim with a same-packet grant) try every
//   slot. T21 also covers the receiver's MTX_REL_REBIND release.
// V2.5-Evo - 2026-10-20 - STEP 2 HEADER FIXES (step-1 code audit Y-1..Y-5, Y-7 b/c): every event now also goes into the
//   Sim's log as "ev<N>", so the order tests see where each line falls (Y-4). New cases: T5 the take-back claim notice
//   (Y-7 b); T13 the pairing bounds guard (Y-2); T21 the hook-table boot check (Y-1); T25 the sparse X-2 disarm edge
//   (Y-5 a); T26 the claim line after the release (Y-4), R14's own zero-proof freshness and motor-output gate with a
//   third remote keeping "present" (Y-5 b), and monotonic times (Y-5 c); T27 the summon line after the release (Y-4);
//   T33 the per-slot stamp expiry across 2^31 ms (Y-3) and the owner's SEIZE / SUMMON restarting its silence (Y-7 c).
// V2.5-Evo - 2026-10-20 - Host test for Common/MultiTx.h (multi-remote v3.2, code step 1) and the stickyWindowClose()
//   helper in Common/AutoReturnRules.h. T1-T18, T20-T33 (T19 lives in config_migrate_test.cpp). Every ID here is MADE UP
//   (the 5C:xx:xx family the public design document uses); no real board ID or MAC is in this file.
//
// The receiver's per-packet rules are run through the REAL mtxOnFrame() with a simulated firmware behind the hooks
// (struct Sim): the hooks do what the receiver's existing code does - the control branch writes the throttle and stamps
// the failsafe, the 0xF1 / 0xF2 / 0xF4 handlers stamp last_packet, the 0xF3 announce switches to 15 bytes. T22 runs the
// same frames through a reference copy of today's triggeredReceive() branch (struct Ref) and compares the writes.
//
// Optional local-only input (audit W-4): pass a file path as the first argument to check the owner's own boards.
// Lines: "<banner MAC, 12 hex digits> <expected chip ID, 6 hex>" and "link <tx id 6 hex> <rx id 6 hex> <L 6 hex>".
// Without it the owner-table check prints SKIPPED. Keep that file out of the repository.
//
// Build and run (per the host-test procedure):
//   g++ -std=c++17 -O1 -Wall -Wextra multi_tx_test.cpp -o multi_tx_test && ./multi_tx_test [local-ids.txt]

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

#include "../../Source/Common/MultiTx.h"
#include "../../Source/Common/AutoReturnRules.h"

// ============================================================
// Harness
// ============================================================
static int g_checks = 0, g_fails = 0;
#define CHECK(cond) do { g_checks++; if (!(cond)) { g_fails++; printf("  FAIL line %d: %s\n", __LINE__, #cond); } } while (0)

static int g_test_checks0 = 0, g_test_fails0 = 0;
static void testBegin() { g_test_checks0 = g_checks; g_test_fails0 = g_fails; }
static void testEnd(const char* name)
{
  printf("%-4s %s (%d checks)\n", name, (g_fails == g_test_fails0) ? "ok  " : "FAIL", g_checks - g_test_checks0);
}

// The exact CRC both boards use (Common/SystemCommon.h esp_crc8(), copied: that header needs Arduino).
static uint8_t crc8(uint8_t* data, uint8_t length)
{
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

// A deterministic generator for the property checks.
static uint32_t g_rng = 0x2545F491u;
static uint32_t rnd() { g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5; return g_rng; }

static void idSet(uint8_t id[3], uint8_t a, uint8_t b, uint8_t c) { id[0] = a; id[1] = b; id[2] = c; }

// Made-up IDs (the public document's example family).
static const uint8_t kR1[3] = { 0x5C, 0x10, 0x20 };   // buggy 1
static const uint8_t kR2[3] = { 0x5C, 0x77, 0x31 };   // buggy 2
static const uint8_t kA[3]  = { 0x5C, 0x4A, 0x7B };   // remote A
static const uint8_t kB[3]  = { 0x5C, 0x91, 0x0E };   // remote B
static const uint8_t kC[3]  = { 0x5C, 0x23, 0xD4 };   // remote C

// ============================================================
// The simulated receiver firmware behind MtxRxHooks
// ============================================================
struct Sim {
  MtxRxEnv env;
  uint16_t failsafe_ms;
  uint8_t  thr, steer;
  uint32_t last_packet, last_control_packet;
  uint32_t mtx_grant_ms;
  uint8_t  seq;
  uint32_t gps_ts; int32_t gps_lat; bool gps_pending; uint8_t gps_len;
  bool     phb_ok; uint32_t phb_last;
  uint8_t  boot_id;
  bool     sticky_cap;
  uint8_t  aux;
  uint32_t now;
  // X-10 (c): one motor-task pass (pwmPass) can be slotted in after log entry number pwm_after (-1 = never), i.e.
  // between any two hook calls of a frame - the PWM task outranks the radio task and may run at any such point.
  int      pwm_after;
  uint8_t  sticky_val;                          // the value the sticky return cap is armed at (RTM's cap at the time)
  std::vector<int> pwm_in, pwm_out, pwm_grant;  // per pass: trigger byte read, byte handed on, limit stamp set
  std::vector<std::string> log;
  std::vector<std::pair<int, int> > replies;    // slot, status
  // V2.5-Evo - 2026-10-21 - E1-E4 (T38): what the receiver's reply hook (RX Radio.ino mtxHkReply()) puts at index 20 - per
  // reply the number sent (1-4, 0 = nobody) or -1 for a telemetry byte, from the same two notes per remote it keeps.
  uint8_t  num_told[kMtxSlots], num_turn[kMtxSlots];
  std::vector<int> reply_num;
  std::vector<std::pair<int, int> > events;     // ev, arg
  std::vector<int> event_slots;
  MtxRxState st;
  MtxRxCfg   cfg;
  MtxRxHooks h;
};

static Sim* S(void* c) { return (Sim*)c; }

// pwmPass - X-10 (c): the parts of one calcPWM() pass (RX PWM.ino) an ownership change can touch, in its order: the
// trigger byte is read FIRST; the sticky return cap (armed by the synthetic "RTM off" while a return ran) applies as a
// min() and clears on a pass with the trigger below 25; then the takeover limit (64 rising over 4 s). Records the
// byte read, the byte handed on to the ramp, and whether a limit stamp was set. Side effects: s->sticky_cap, records.
static void pwmPass(Sim* s)
{
  const uint8_t thr_read = s->thr;
  uint8_t eff = thr_read;
  if (s->sticky_cap) { if (s->sticky_val < eff) eff = s->sticky_val; if (thr_read < 25) s->sticky_cap = false; }
  const uint32_t g = s->mtx_grant_ms;
  const uint8_t lim = mtxTakeoverCap(s->now, g, 64, 4000);
  if (lim < eff) eff = lim;
  s->pwm_in.push_back(thr_read); s->pwm_out.push_back(eff); s->pwm_grant.push_back(g != 0 ? 1 : 0);
}
// slog - every hook logs through here, so a motor pass can be slotted in after any chosen step (pwm_after).
static void slog(Sim* s, const std::string& what)
{
  s->log.push_back(what);
  if ((int)s->log.size() - 1 == s->pwm_after) pwmPass(s);
}

static void hkReadEnv(void* c, MtxRxEnv* out) { *out = S(c)->env; }
static void hkZero(void* c) { S(c)->thr = 0; S(c)->steer = 127; slog(S(c), "zero"); }
static void hkRtmOff(void* c)
{
  // processRtmStatePacket(0xF1/0): the sticky return cap is armed only if RTM was running.
  if (S(c)->env.rtm_rx_active) S(c)->sticky_cap = true;
  S(c)->env.rtm_rx_active = false;
  slog(S(c), "rtm_off");
}
static void hkFmOff(void* c) { S(c)->env.fm_mode_runtime = 0; slog(S(c), "fm_off"); }
static void hkGpsReset(void* c)
{
  Sim* s = S(c);
  s->gps_ts = 0; s->phb_ok = false; s->phb_last = 0;            // the coordinates (gps_lat) are NOT cleared
  if (s->gps_pending) { s->gps_pending = false; s->gps_len = 6; }
  slog(s, "gps_reset");
}
static void hkBootReset(void* c) { S(c)->boot_id = 0; slog(S(c), "boot_reset"); }
static void hkLimit(void* c, uint32_t g) { S(c)->mtx_grant_ms = g; slog(S(c), "limit"); }
static void hkSeq(void* c) { S(c)->seq++; slog(S(c), "seq"); }
static void hkApply(void* c, int8_t, const uint8_t f[6])
{
  // Today's control branch (RX Radio.ino): last_packet, throttle, steering, THEN the failsafe stamp.
  Sim* s = S(c);
  s->last_packet = s->now; s->thr = f[3]; s->steer = f[4]; s->last_control_packet = s->now;
  slog(s, "apply");
}
static void hkMeta(void* c, int8_t, const uint8_t f[6])
{
  // Today's meta handlers: 0xF1 / 0xF2 / 0xF4 stamp last_packet; 0xF3 switches to 15 bytes and stamps nothing.
  Sim* s = S(c);
  if (f[3] == 0xF1) {
    s->last_packet = s->now;
    if (f[4] == 0) { if (s->env.rtm_rx_active) s->sticky_cap = true; s->env.rtm_rx_active = false; }
    else if (f[4] == 1) s->env.rtm_rx_active = true;
    else if (f[4] >= 0x80) s->boot_id = f[4];
  } else if (f[3] == 0xF2) {
    s->last_packet = s->now;
    uint8_t m = f[4] & 0x07; if (m > 5) m = 0;
    s->env.fm_mode_runtime = m;
  } else if (f[3] == 0xF4) {
    s->last_packet = s->now; s->aux = f[4];
  } else if (f[3] == 0xF3) {
    s->gps_pending = true; s->gps_len = 15;
  }
  slog(s, "meta");
}
static void hkReply(void* c, int8_t slot, uint8_t status)
{
  Sim* s = S(c);
  s->replies.push_back(std::make_pair((int)slot, (int)status));
  // E1-E4: as mtxHkReply()'s MULTI branch (the SINGLE branch never carries the number).
  int num = -1;
  if (!s->cfg.single && slot >= 0 && slot < (int8_t)kMtxSlots)
  {
    const uint8_t n = mtxInControlNumber(&s->st);
    if (mtxReplyNumberDue(status, n, &s->num_told[slot], &s->num_turn[slot])) num = n;
  }
  s->reply_num.push_back(num);
}
static void hkEvent(void* c, uint8_t ev, int8_t slot, uint8_t arg)
{
  S(c)->events.push_back(std::make_pair((int)ev, (int)arg));
  S(c)->event_slots.push_back(slot);
  slog(S(c), "ev" + std::to_string((int)ev));            // Y-4: the order tests see where each line falls
}

static void simInit(Sim* s, bool single, uint8_t valid_mask)
{
  s->env.pwm_active = true; s->env.fm_mode_runtime = 0xFF; s->env.rtm_rx_active = false; s->env.fm_rx_active = false;
  s->failsafe_ms = 1000;
  s->thr = 0; s->steer = 127; s->last_packet = 0; s->last_control_packet = 0; s->mtx_grant_ms = 0; s->seq = 0;
  s->gps_ts = 0; s->gps_lat = 0; s->gps_pending = false; s->gps_len = 6; s->phb_ok = false; s->phb_last = 0;
  s->boot_id = 0; s->sticky_cap = false; s->aux = 0; s->now = 0;
  s->pwm_after = -1; s->sticky_val = 40; s->pwm_in.clear(); s->pwm_out.clear(); s->pwm_grant.clear();
  s->log.clear(); s->replies.clear(); s->events.clear(); s->event_slots.clear();
  memset(s->num_told, kMtxNumberUntold, sizeof(s->num_told)); memset(s->num_turn, 0, sizeof(s->num_turn)); s->reply_num.clear();
  mtxRxStateInit(&s->st);
  s->cfg.single = single; s->cfg.valid_mask = valid_mask; s->cfg.claim_s = 10; s->cfg.failsafe_ms = 1000; s->cfg.seize_en = true;
  s->h.ctx = s; s->h.read_env = hkReadEnv; s->h.zero_outputs = hkZero; s->h.synthetic_rtm_off = hkRtmOff;
  s->h.synthetic_fm_off = hkFmOff; s->h.reset_rider_gps = hkGpsReset; s->h.reset_boot_id = hkBootReset;
  s->h.publish_limit = hkLimit; s->h.owner_changed = hkSeq; s->h.apply_owner = hkApply; s->h.meta = hkMeta;
  s->h.reply = hkReply; s->h.event = hkEvent;
}
static void simClear(Sim* s) { s->log.clear(); s->replies.clear(); s->events.clear(); s->event_slots.clear(); s->reply_num.clear(); }

// One frame from slot at time t.
static void fr(Sim* s, int8_t slot, uint8_t b3, uint8_t b4, uint32_t t, bool legacy = false)
{
  s->now = t;
  const uint8_t f[6] = { 0, 0, 0, b3, b4, 0 };
  mtxOnFrame(&s->st, &s->cfg, &s->h, slot, legacy, f, t);
}
// The radio task's 2 s semaphore timeout (no frame).
static void wake(Sim* s, uint32_t t) { s->now = t; mtxTimers(&s->st, &s->cfg, &s->h, t); }
// The loop task's Follow-Me tick: fm_rx_active follows the declaration (fmEnterIdle() when not declared).
static void loopTick(Sim* s) { s->env.fm_rx_active = mtxFmDeclared(s->env.fm_mode_runtime); }

static int lastStatus(const Sim* s) { return s->replies.empty() ? -1 : s->replies.back().second; }
static bool logHas(const Sim* s, const char* what)
{
  for (size_t i = 0; i < s->log.size(); i++) if (s->log[i] == what) return true;
  return false;
}
static int logPos(const Sim* s, const char* what)
{
  for (size_t i = 0; i < s->log.size(); i++) if (s->log[i] == what) return (int)i;
  return -1;
}
static int countEv(const Sim* s, int ev)
{
  int n = 0;
  for (size_t i = 0; i < s->events.size(); i++) if (s->events[i].first == ev) n++;
  return n;
}
static uint8_t claimable(Sim* s, uint32_t t)
{
  MtxRxEnv e = s->env;
  return mtxClaimableReason(&s->st, &s->cfg, &e, t);
}

// Grant slot to an owner the normal way: zero, zero, squeeze. Ends at t+200.
static void grantBySqueeze(Sim* s, int8_t slot, uint32_t t)
{
  fr(s, slot, 0, 127, t); fr(s, slot, 0, 127, t + 100); fr(s, slot, 60, 127, t + 200);
}

// ============================================================
// T1 - link ID and the one identity function (W-4, W-5)
// ============================================================
static uint64_t macU64(const uint8_t m[6])   // the uint64 getEfuseMac() returns: mac[0] in the LOW byte
{
  uint64_t v = 0;
  for (int i = 5; i >= 0; i--) v = (v << 8) | m[i];
  return v;
}
// A copy of the default-address bake as it stands in Common/SPIFFSEngine.h, to prove efuseIdFromMac() is the same.
static void bakeCopy(uint64_t mac, uint8_t out[3])
{
  uint8_t mac_address[6];
  for (int i = 0; i < 6; i++) mac_address[i] = (mac >> (8 * i)) & 0xFF;
  out[0] = crc8(mac_address, 4); out[1] = mac_address[4]; out[2] = mac_address[5];
}
static bool parseHex(const char* p, int digits, uint64_t* out)
{
  uint64_t v = 0;
  for (int i = 0; i < digits; i++) {
    const char ch = p[i]; int d;
    if (ch >= '0' && ch <= '9') d = ch - '0'; else if (ch >= 'a' && ch <= 'f') d = ch - 'a' + 10;
    else if (ch >= 'A' && ch <= 'F') d = ch - 'A' + 10; else return false;
    v = (v << 4) | (uint64_t)d;
  }
  *out = v; return true;
}
static void id3(uint64_t v, uint8_t out[3]) { out[0] = (uint8_t)(v >> 16); out[1] = (uint8_t)(v >> 8); out[2] = (uint8_t)v; }

static void t1(const char* local_path)
{
  testBegin();
  // The public example: buggy 5C:10:20, remote A 5C:4A:7B, remote B 5C:91:0E -> L 00:5A:5B and 00:81:2E.
  uint8_t L[3], back[3];
  mtxIdXor(kA, kR1, L);  CHECK(L[0] == 0x00 && L[1] == 0x5A && L[2] == 0x5B);
  mtxIdXor(kB, kR1, L);  CHECK(L[0] == 0x00 && L[1] == 0x81 && L[2] == 0x2E);
  // Round trip, never zero for tx != rx, and the uplink address (L) is never the reply address (T) when rx != 0.
  for (int i = 0; i < 20000; i++) {
    uint8_t t[3], r[3];
    idSet(t, (uint8_t)rnd(), (uint8_t)rnd(), (uint8_t)rnd());
    idSet(r, (uint8_t)rnd(), (uint8_t)rnd(), (uint8_t)rnd());
    mtxIdXor(t, r, L); mtxIdXor(L, r, back);
    CHECK(mtxIdEqual(back, t));
    if (!mtxIdEqual(t, r)) CHECK(!mtxIdIsZero(L));
    if (!mtxIdIsZero(r))   CHECK(!mtxIdEqual(L, t));
  }
  // W-5 byte order, from a synthetic MAC 02:11:22:33:44:55 (mac[0] = 0x02).
  const uint8_t m[6] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x55 };
  const uint64_t u = macU64(m);
  CHECK(u == 0x0000554433221102ULL);                     // the banner would print 554433221102 - the MAC backwards
  uint8_t id[3], ref[3];
  efuseIdFromMac(u, crc8, id);
  uint8_t first4[4] = { 0x02, 0x11, 0x22, 0x33 };
  CHECK(id[0] == crc8(first4, 4) && id[1] == 0x44 && id[2] == 0x55);
  // W-5: the one function equals the default-address bake for any MAC.
  for (int i = 0; i < 20000; i++) {
    const uint64_t mac = ((uint64_t)rnd() << 16) ^ (uint64_t)rnd();
    efuseIdFromMac(mac & 0xFFFFFFFFFFFFULL, crc8, id); bakeCopy(mac & 0xFFFFFFFFFFFFULL, ref);
    CHECK(mtxIdEqual(id, ref));
  }
  // W-4: the owner's own boards only from a local file.
  FILE* fp = local_path ? fopen(local_path, "r") : NULL;
  if (!fp) {
    printf("     T1 owner table: SKIPPED (no local input)\n");
  } else {
    char line[128]; int n = 0;
    while (fgets(line, sizeof(line), fp)) {
      if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
      uint64_t a, b, c;
      if (strncmp(line, "link ", 5) == 0) {
        if (parseHex(line + 5, 6, &a) && parseHex(line + 12, 6, &b) && parseHex(line + 19, 6, &c)) {
          uint8_t ta[3], rb[3], lc[3], l2[3]; id3(a, ta); id3(b, rb); id3(c, lc);
          mtxIdXor(ta, rb, l2); CHECK(mtxIdEqual(l2, lc)); n++;
        }
      } else if (parseHex(line, 12, &a) && parseHex(line + 13, 6, &b)) {
        uint8_t want[3]; id3(b, want); efuseIdFromMac(a, crc8, id); CHECK(mtxIdEqual(id, want)); n++;
      }
    }
    fclose(fp);
    printf("     T1 owner table: %d local lines checked\n", n);
  }
  testEnd("T1");
}

// ============================================================
// T2 - classification across two buggies (the only multi-buggy coverage)
// ============================================================
static MtxRxBinding bindingV3(const uint8_t rx[3], const uint8_t* const* ids, int n)
{
  MtxRxBinding b; memset(&b, 0, sizeof(b));
  b.paired = 1; memcpy(b.own_address, rx, 3);
  for (int i = 0; i < n; i++) { memcpy(&b.mtx_id[i * 3], ids[i], 3); b.mtx_flags[i] = kMtxFlagValid; }
  return b;
}
static void t2()
{
  testBegin();
  const uint8_t* ids[2] = { kA, kB };
  const MtxRxBinding b1 = bindingV3(kR1, ids, 2), b2 = bindingV3(kR2, ids, 2);
  uint8_t L[3];
  mtxIdXor(kA, kR1, L); CHECK(mtxLookupSlot(&b1, kR1, L) == 0); CHECK(mtxLookupSlot(&b2, kR2, L) == kMtxSlotUnknown);
  mtxIdXor(kB, kR1, L); CHECK(mtxLookupSlot(&b1, kR1, L) == 1); CHECK(mtxLookupSlot(&b2, kR2, L) == kMtxSlotUnknown);
  mtxIdXor(kA, kR2, L); CHECK(mtxLookupSlot(&b2, kR2, L) == 0); CHECK(mtxLookupSlot(&b1, kR1, L) == kMtxSlotUnknown);
  mtxIdXor(kB, kR2, L); CHECK(mtxLookupSlot(&b2, kR2, L) == 1); CHECK(mtxLookupSlot(&b1, kR1, L) == kMtxSlotUnknown);
  // A reply (bytes 0-2 = a remote's own ID) never classifies as an entry, on either buggy.
  CHECK(mtxLookupSlot(&b1, kR1, kA) == kMtxSlotUnknown); CHECK(mtxLookupSlot(&b1, kR1, kB) == kMtxSlotUnknown);
  CHECK(mtxLookupSlot(&b2, kR2, kA) == kMtxSlotUnknown); CHECK(mtxLookupSlot(&b2, kR2, kB) == kMtxSlotUnknown);
  // Gate 1: address 00:00:00.
  const uint8_t z[3] = { 0, 0, 0 };
  CHECK(mtxLookupSlot(&b1, kR1, z) == kMtxSlotGate1);
  // The legacy alias (bytes 0-2 = the buggy's own address) only as the sole entry.
  CHECK(mtxLookupSlot(&b1, kR1, kR1) == kMtxSlotUnknown);           // two new-style entries: no alias
  MtxRxBinding lg; memset(&lg, 0, sizeof(lg)); lg.paired = 1; memcpy(lg.own_address, kR1, 3);
  memcpy(lg.mtx_id, kA, 3); lg.mtx_flags[0] = kMtxFlagValid | kMtxFlagLegacy; memcpy(lg.dest_address, kA, 3);
  CHECK(mtxLookupSlot(&lg, kR1, kR1) == 0);
  // The implicit legacy entry of a migrated board.
  MtxRxBinding im; memset(&im, 0, sizeof(im)); im.paired = 1; memcpy(im.own_address, kR1, 3); memcpy(im.dest_address, kA, 3);
  CHECK(mtxImplicitLegacy(&im, kR1)); CHECK(mtxLookupSlot(&im, kR1, kR1) == kMtxSlotImplicit);
  CHECK(mtxRxMode(&im, kR1) == MTX_MODE_SINGLE);
  CHECK(mtxRxMode(&b1, kR1) == MTX_MODE_MULTI);
  // Never formed next to a list entry, nor when own_address is not the chip ID.
  MtxRxBinding im2 = im; memcpy(im2.mtx_id, kB, 3); im2.mtx_flags[0] = kMtxFlagValid;
  CHECK(!mtxImplicitLegacy(&im2, kR1));
  CHECK(!mtxImplicitLegacy(&im, kR2));
  // Every random frame for buggy 1 is unknown on buggy 2 unless it decodes exactly to a listed ID there.
  for (int i = 0; i < 20000; i++) {
    uint8_t a[3]; idSet(a, (uint8_t)rnd(), (uint8_t)rnd(), (uint8_t)rnd());
    const int8_t s = mtxLookupSlot(&b2, kR2, a);
    uint8_t t[3]; mtxIdXor(a, kR2, t);
    if (s >= 0) CHECK(mtxIdEqual(t, kA) || mtxIdEqual(t, kB));
  }
  testEnd("T2");
}

// ============================================================
// T3 - kind decode (+ SUMMON, T32)
// ============================================================
static void t3()
{
  testBegin();
  for (int b3 = 0; b3 <= 0xF0; b3++) for (int b4 = 0; b4 < 256; b4 += 17) CHECK(mtxKind((uint8_t)b3, (uint8_t)b4) == MTX_K_CTRL);
  for (int b3 = 0xF5; b3 <= 0xFF; b3++) for (int b4 = 0; b4 < 256; b4++) {
    CHECK(mtxKind((uint8_t)b3, (uint8_t)b4) == MTX_K_DROP);
    CHECK(mtxFrameKind((uint8_t)b3, (uint8_t)b4, true, false) == MTX_K_DROP);    // SINGLE too
    CHECK(mtxFrameKind((uint8_t)b3, (uint8_t)b4, false, false) == MTX_K_DROP);   // MULTI
  }
  CHECK(mtxKind(0xF1, 0x10) == MTX_K_LOCKED);
  CHECK(mtxKind(0xF1, 0x5A) == MTX_K_SEIZE);
  CHECK(mtxKind(0xF1, 0x3C) == MTX_K_SUMMON);
  for (int b4 = 0; b4 < 256; b4++) {
    if (b4 == 0x10 || b4 == 0x5A || b4 == 0x3C) continue;
    CHECK(mtxKind(0xF1, (uint8_t)b4) == MTX_K_META);                       // incl. 0x11, 0x5B, 0x3B, 0x3D, 0, 1, 2, 0x80+
  }
  for (int b3 = 0xF2; b3 <= 0xF4; b3++) for (int b4 = 0; b4 < 256; b4++) CHECK(mtxKind((uint8_t)b3, (uint8_t)b4) == MTX_K_META);
  // The neighbours stay plain metas.
  CHECK(mtxKind(0xF1, 0x11) == MTX_K_META && mtxKind(0xF1, 0x5B) == MTX_K_META);
  CHECK(mtxKind(0xF1, 0x3B) == MTX_K_META && mtxKind(0xF1, 0x3D) == MTX_K_META);
  // Legacy entry: LOCKED / SEIZE / SUMMON dropped, in both modes.
  CHECK(mtxFrameKind(0xF1, 0x10, true, true) == MTX_K_DROP && mtxFrameKind(0xF1, 0x5A, true, true) == MTX_K_DROP);
  CHECK(mtxFrameKind(0xF1, 0x3C, true, true) == MTX_K_DROP && mtxFrameKind(0xF1, 0x3C, false, true) == MTX_K_DROP);
  // SINGLE: LOCKED -> control (0/127 is applied, T22); SEIZE and SUMMON dropped.
  CHECK(mtxFrameKind(0xF1, 0x10, true, false) == MTX_K_CTRL);
  CHECK(mtxFrameKind(0xF1, 0x5A, true, false) == MTX_K_DROP);
  CHECK(mtxFrameKind(0xF1, 0x3C, true, false) == MTX_K_DROP);
  // MULTI: as decoded.
  CHECK(mtxFrameKind(0xF1, 0x10, false, false) == MTX_K_LOCKED);
  CHECK(mtxFrameKind(0xF1, 0x5A, false, false) == MTX_K_SEIZE);
  CHECK(mtxFrameKind(0xF1, 0x3C, false, false) == MTX_K_SUMMON);
  testEnd("T3");
}

// ============================================================
// T4 - grant (R1)
// ============================================================
static void t4()
{
  testBegin();
  Sim s;
  simInit(&s, false, 0x03);
  grantBySqueeze(&s, 1, 1000);
  CHECK(s.st.owner == 1); CHECK(lastStatus(&s) == MTX_S_OWNER);
  CHECK(logPos(&s, "limit") >= 0 && logPos(&s, "limit") < logPos(&s, "apply"));   // the limit before the first byte
  CHECK(s.thr == 60 && s.mtx_grant_ms == 1200);
  // A squeeze held throughout never takes control.
  simInit(&s, false, 0x03);
  for (int i = 0; i < 30; i++) fr(&s, 1, 60, 127, 1000 + 100 * i);
  CHECK(s.st.owner == kMtxNone); CHECK(s.thr == 0); CHECK(lastStatus(&s) == MTX_S_FREE);   // FREE, never E9
  // zero, 9, squeeze: no.
  simInit(&s, false, 0x03);
  fr(&s, 1, 0, 127, 1000); fr(&s, 1, 9, 127, 1100); fr(&s, 1, 60, 127, 1200);
  CHECK(s.st.owner == kMtxNone);
  // Zero proof older than 3.0 s: no (3100 ms); exactly 3000: yes.
  simInit(&s, false, 0x03);
  fr(&s, 1, 0, 127, 1000); fr(&s, 1, 0, 127, 1100); fr(&s, 1, 60, 127, 4200);
  CHECK(s.st.owner == kMtxNone);
  simInit(&s, false, 0x03);
  fr(&s, 1, 0, 127, 1000); fr(&s, 1, 0, 127, 1100); fr(&s, 1, 60, 127, 4100);
  CHECK(s.st.owner == 1);
  // Follow-Me 0 and 0xFF allowed; 1-5 refused; RTM and fm_rx_active refused; no PWM refused.
  for (int m = 0; m < 256; m++) {
    simInit(&s, false, 0x03); s.env.fm_mode_runtime = (uint8_t)m;
    grantBySqueeze(&s, 0, 1000);
    CHECK((s.st.owner == 0) == !(m >= 1 && m <= 5));
  }
  simInit(&s, false, 0x03); s.env.rtm_rx_active = true; grantBySqueeze(&s, 0, 1000); CHECK(s.st.owner == kMtxNone);
  simInit(&s, false, 0x03); s.env.fm_rx_active = true;  grantBySqueeze(&s, 0, 1000); CHECK(s.st.owner == kMtxNone);
  simInit(&s, false, 0x03); s.env.pwm_active = false;   grantBySqueeze(&s, 0, 1000); CHECK(s.st.owner == kMtxNone);
  // LOCKED never counts as zero.
  simInit(&s, false, 0x03);
  fr(&s, 1, 0, 127, 1000); fr(&s, 1, 0xF1, 0x10, 1100); fr(&s, 1, 60, 127, 1200);
  CHECK(s.st.owner == kMtxNone);
  simInit(&s, false, 0x03);
  fr(&s, 1, 0xF1, 0x10, 1000); fr(&s, 1, 0xF1, 0x10, 1100); fr(&s, 1, 60, 127, 1200);
  CHECK(s.st.owner == kMtxNone);
  // The grant is read BEFORE the streak update: the squeeze itself does not reset the proof it is judged on.
  simInit(&s, false, 0x03);
  fr(&s, 1, 0, 127, 1000); fr(&s, 1, 0, 127, 1100);
  CHECK(s.st.zero_streak[1] == 2);
  fr(&s, 1, 60, 127, 1200);
  CHECK(s.st.owner == 1 && s.st.zero_streak[1] == 0);
  // An invalid slot never acts.
  simInit(&s, false, 0x03); grantBySqueeze(&s, 2, 1000); CHECK(s.st.owner == kMtxNone && s.replies.empty());
  testEnd("T4");
}

// ============================================================
// T5 - the master's take-back (R8)
// ============================================================
static void t5()
{
  testBegin();
  Sim s;
  simInit(&s, false, 0x03);
  grantBySqueeze(&s, 1, 1000);                         // the guest (slot 1) owns and drives
  fr(&s, 1, 120, 127, 1300);
  fr(&s, 0, 0, 127, 1400); fr(&s, 0, 0, 127, 1500);    // the master at zero
  CHECK(lastStatus(&s) == MTX_S_SEIZABLE);
  simClear(&s);
  fr(&s, 0, 0xF1, 0x5A, 1600);
  CHECK(s.st.owner == 0); CHECK(s.thr == 0);            // the guest's throttle zeroed at once
  CHECK(logPos(&s, "zero") == 0); CHECK(s.mtx_grant_ms == 1600); CHECK(s.replies.empty());   // no reply to SEIZE
  fr(&s, 1, 120, 127, 1700); CHECK(lastStatus(&s) == MTX_S_SEIZED);
  // Negatives.
  const int cases = 6;
  for (int c = 0; c < cases; c++) {
    simInit(&s, false, 0x03);
    grantBySqueeze(&s, 1, 1000);
    fr(&s, 0, 0, 127, 1400); fr(&s, 0, 0, 127, 1500);
    uint32_t t = 1600;
    if (c == 0) s.cfg.seize_en = false;
    if (c == 1) s.env.fm_mode_runtime = 2;
    if (c == 2) s.env.rtm_rx_active = true;
    if (c == 3) s.env.pwm_active = false;
    if (c == 4) t = 2600;                               // zero proof 1100 ms old
    if (c == 5) { fr(&s, 0, 10, 127, 1550); }           // the master not at zero
    fr(&s, 0, 0xF1, 0x5A, t);
    CHECK(s.st.owner == 1);
  }
  // A guest can never seize.
  simInit(&s, false, 0x03);
  grantBySqueeze(&s, 0, 1000);
  fr(&s, 1, 0, 127, 1400); fr(&s, 1, 0, 127, 1500); fr(&s, 1, 0xF1, 0x5A, 1600);
  CHECK(s.st.owner == 0);
  // Nobody owns: nothing to take back.
  simInit(&s, false, 0x03);
  fr(&s, 0, 0, 127, 1400); fr(&s, 0, 0, 127, 1500); fr(&s, 0, 0xF1, 0x5A, 1600);
  CHECK(s.st.owner == kMtxNone);
  // Y-7 (b): a guest that was SILENT at the take-back and comes back after the 5 s hold still hears SEIZED ("tA")
  // on its first frame - the take-back leaves a claim notice, as a claim does - and the hold restarts from that frame.
  simInit(&s, false, 0x03);
  grantBySqueeze(&s, 1, 1000); fr(&s, 1, 0, 127, 1300);   // the guest owns, then goes quiet at 1300
  fr(&s, 0, 0, 127, 1400); fr(&s, 0, 0, 127, 1500);
  fr(&s, 0, 0xF1, 0x5A, 1600);
  CHECK(s.st.owner == 0 && s.st.claim_notice[1]);
  fr(&s, 0, 0, 127, 7500);                                // the master keeps sending
  fr(&s, 1, 0, 127, 7600);                                // the guest is back 6 s later
  CHECK(lastStatus(&s) == MTX_S_SEIZED && s.st.seized_until_ms[1] == 7600 + 5000 && !s.st.claim_notice[1]);
  testEnd("T5");
}

// ============================================================
// T7 - no timer releases anything (replaced by T23-T25 and T33)
// ============================================================
static void t7()
{
  testBegin();
  Sim s;
  for (int variant = 0; variant < 4; variant++) {
    simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000);
    if (variant == 1) { fr(&s, 0, 0xF2, 0x82, 1300); loopTick(&s); }    // Follow-Me armed
    if (variant == 2) { fr(&s, 0, 0xF1, 1, 1300); }                       // RTM on
    simClear(&s);
    const uint8_t seq0 = s.seq;
    for (uint32_t t = 2000; t < 400000; t += 2000) {
      wake(&s, t);
      if (variant == 3) fr(&s, 1, 0, 127, t + 1);                         // another remote present the whole time
    }
    CHECK(s.st.owner == 0); CHECK(!logHas(&s, "zero")); CHECK(s.seq == seq0);
    CHECK(countEv(&s, MTX_EV_RELEASED) == 0);
  }
  testEnd("T7");
}

// ============================================================
// T8 - the takeover limit
// ============================================================
static void t8()
{
  testBegin();
  CHECK(mtxTakeoverCap(5000, 0, 64, 4000) == 255);
  CHECK(mtxTakeoverCap(5000, 1000, 64, 0) == 255);
  CHECK(mtxTakeoverCap(5000, 1000, 255, 4000) == 255);
  const uint32_t g = 10000;
  CHECK(mtxTakeoverCap(g, g, 64, 4000) == 64);
  CHECK(mtxTakeoverCap(g + 1000, g, 64, 4000) == 111);
  CHECK(mtxTakeoverCap(g + 2000, g, 64, 4000) == 159);
  CHECK(mtxTakeoverCap(g + 3000, g, 64, 4000) == 207);
  CHECK(mtxTakeoverCap(g + 4000, g, 64, 4000) == 255);
  CHECK(mtxTakeoverCap(g - 50, g, 64, 4000) == 64);                 // a negative age counts as "just granted"
  uint8_t prev = 0;
  for (uint32_t dt = 0; dt <= 5000; dt++) {
    const uint8_t c = mtxTakeoverCap(g + dt, g, 64, 4000);
    CHECK(c >= prev); prev = c;
    for (int x = 0; x < 256; x += 51) CHECK((x < c ? x : c) <= x);  // min(x, cap) never raises x
  }
  // Across the millis() wrap.
  const uint32_t gw = 0xFFFFFF00u;
  CHECK(mtxTakeoverCap(gw + 1000, gw, 64, 4000) == 111);
  CHECK(mtxTakeoverExpired(gw + 4000, gw, 4000) && !mtxTakeoverExpired(gw + 3999, gw, 4000));
  // A grant at millis() == 0 is stored as 1 (0 means "no limit").
  Sim s; simInit(&s, false, 0x03);
  fr(&s, 0, 0, 127, 0xFFFFFF00u); fr(&s, 0, 0, 127, 0xFFFFFF80u); fr(&s, 0, 60, 127, 0);
  CHECK(s.st.owner == 0 && s.mtx_grant_ms == 1);
  testEnd("T8");
}

// ============================================================
// T9 - status pack / unpack; S = 000 always in SINGLE
// ============================================================
struct __attribute__((packed)) TelemetryMirror { uint8_t b[20]; };   // the 20-byte TelemetryPacket
static_assert(sizeof(TelemetryMirror) <= 32, "TelemetryPacket must stay within 32 bytes (5-bit index)");
static void t9()
{
  testBegin();
  for (int s = 0; s < 8; s++) for (int i = 0; i < 20; i++) {
    const uint8_t b3 = mtxReplyByte3((uint8_t)s, (uint8_t)i);
    CHECK(mtxReplyIndexOf(b3) == i);
    CHECK(mtxReplyStatusOf(b3) == ((s == 7) ? MTX_S_BUSY : s));
  }
  for (int i = 0; i < 20; i++) CHECK(mtxReplyByte3(MTX_S_OWNER_SOLO, (uint8_t)i) == i);   // byte-identical to today
  Sim s; simInit(&s, true, 0x01);
  for (int i = 0; i < 50; i++) fr(&s, 0, (uint8_t)(i * 5), 127, 1000 + 100 * i);
  fr(&s, 0, 0xF1, 0x10, 7000);
  bool all000 = !s.replies.empty();
  for (size_t i = 0; i < s.replies.size(); i++) all000 = all000 && s.replies[i].second == MTX_S_OWNER_SOLO;
  CHECK(all000);
  testEnd("T9");
}

// ============================================================
// T10 - the reply masks; per-slot link quality
// ============================================================
static void t10()
{
  testBegin();
  for (int idx = 0; idx < 20; idx++) for (int raw = 0; raw < 256; raw += 7) {
    const uint8_t r = (uint8_t)raw, i = (uint8_t)idx;
    CHECK(mtxReplyValue(i, r, true, false, true, 33) == r);           // SINGLE: unchanged, every index
    const uint8_t own = mtxReplyValue(i, r, false, true, false, 33);
    const uint8_t non = mtxReplyValue(i, r, false, false, false, 33);
    const uint8_t chg = mtxReplyValue(i, r, false, true, true, 33);
    if (i == 18) { CHECK(own == 33 && non == 33 && chg == 33); continue; }
    CHECK(own == r);
    CHECK(chg == non);                                                // O-1: the full non-owner mask until rebuilt
    switch (i) {
      case 5: CHECK(non == 0xFF); break;
      case 14: CHECK(non == 127); break;
      case 15: CHECK(non == (r & 0xFC)); break;
      case 16: CHECK(non == 0); break;
      case 17: CHECK(non == 0xFF); break;
      case 19: CHECK(non == 0); break;
      default: CHECK(non == r); break;
    }
  }
  // The new owner's fault bits (16 bit 3, 19 bits 0-1) are clear while the change is pending.
  CHECK((mtxReplyValue(16, 0xFF, false, true, true, 0) & 0x08) == 0);
  CHECK((mtxReplyValue(19, 0xFF, false, true, true, 0) & 0x03) == 0);
  testEnd("T10");
}

// ============================================================
// T11 - per-remote telemetry rotation
// ============================================================
static void t11()
{
  testBegin();
  uint8_t idx_owner = 0, idx_guest = 0, shared = 0;
  bool seen_o[20] = {}, seen_g[20] = {}, seen_shared_o[20] = {};
  int t_all_owner = -1;
  for (int k = 0; k < 400; k++) {             // 40 s: 10 cycles per second, cycle 9 is the guest
    const bool guest = (k % 10) == 9;
    if (guest) { seen_g[mtxTelIndexNext(&idx_guest)] = true; shared = (uint8_t)((shared + 1) % 20); }
    else {
      seen_o[mtxTelIndexNext(&idx_owner)] = true;
      seen_shared_o[shared] = true; shared = (uint8_t)((shared + 1) % 20);
      bool all = true; for (int i = 0; i < 20; i++) all = all && seen_o[i];
      if (all && t_all_owner < 0) t_all_owner = k;
    }
  }
  bool all_o = true, all_g = true, all_shared = true;
  for (int i = 0; i < 20; i++) { all_o = all_o && seen_o[i]; all_g = all_g && seen_g[i]; all_shared = all_shared && seen_shared_o[i]; }
  CHECK(all_o && all_g);
  CHECK(t_all_owner >= 0 && t_all_owner <= 25);   // the owner sees all 20 within about 2.2 s
  CHECK(!all_shared);                              // with ONE shared rotation the owner would miss index 9, 19, ...
  testEnd("T11");
}

// ============================================================
// T12 - entry, binding and identity validation
// ============================================================
static void t12()
{
  testBegin();
  // Identity: a copied / edited own_address drops the binding.
  MtxRxBinding b; const uint8_t* ids[2] = { kA, kB };
  b = bindingV3(kR2, ids, 2);                                    // the stored address is buggy 2's ...
  uint8_t n = mtxValidateRxBinding(&b, kR1);                     // ... but the chip is buggy 1
  CHECK((n & MTX_NOTE_ID_MISMATCH) != 0); CHECK(mtxIdEqual(b.own_address, kR1)); CHECK(b.paired == 0);
  CHECK(mtxValidEntryCount(&b) == 0 && mtxIdIsZero(b.dest_address));
  MtxRxBinding z; memset(&z, 0, sizeof(z)); z.paired = 1; memcpy(z.dest_address, kA, 3);
  n = mtxValidateRxBinding(&z, kR1);
  CHECK((n & MTX_NOTE_ID_NOT_SET) != 0 && z.paired == 0 && mtxIdIsZero(z.dest_address));
  // The migrated single-remote case, own == chip: the implicit legacy entry is kept.
  MtxRxBinding m; memset(&m, 0, sizeof(m)); m.paired = 1; memcpy(m.own_address, kR1, 3); memcpy(m.dest_address, kA, 3);
  n = mtxValidateRxBinding(&m, kR1);
  CHECK(n == 0 && m.paired == 1 && mtxImplicitLegacy(&m, kR1));
  // paired == 0 clears the list and dest.
  b = bindingV3(kR1, ids, 2); b.paired = 0; memcpy(b.dest_address, kA, 3);
  n = mtxValidateRxBinding(&b, kR1);
  CHECK((n & MTX_NOTE_UNPAIRED) != 0 && mtxValidEntryCount(&b) == 0 && mtxIdIsZero(b.dest_address));
  // Entries: 000000, the buggy's own ID, a duplicate.
  const uint8_t zero[3] = { 0, 0, 0 };
  const uint8_t* bad[4] = { zero, kR1, kA, kA };
  b = bindingV3(kR1, bad, 4);
  n = mtxValidateRxBinding(&b, kR1);
  CHECK((n & MTX_NOTE_ENTRY_CLEARED) != 0);
  CHECK(!mtxEntryValid(&b, 0) && !mtxEntryValid(&b, 1) && mtxEntryValid(&b, 2) && !mtxEntryValid(&b, 3));
  // A legacy entry next to a new-style one is removed; dest goes to 000000.
  b = bindingV3(kR1, ids, 2); b.mtx_flags[1] |= kMtxFlagLegacy; memcpy(b.dest_address, kB, 3);
  n = mtxValidateRxBinding(&b, kR1);
  CHECK(mtxEntryValid(&b, 0) && !mtxEntryValid(&b, 1) && mtxIdIsZero(b.dest_address));
  // Two legacy entries: the second goes.
  b = bindingV3(kR1, ids, 2); b.mtx_flags[0] |= kMtxFlagLegacy; b.mtx_flags[1] |= kMtxFlagLegacy;
  n = mtxValidateRxBinding(&b, kR1);
  CHECK(mtxEntryValid(&b, 0) && !mtxEntryValid(&b, 1));
  // A clean list is untouched.
  b = bindingV3(kR1, ids, 2); MtxRxBinding before = b;
  CHECK(mtxValidateRxBinding(&b, kR1) == 0 && memcmp(&b, &before, sizeof(b)) == 0);
  // The remote's link record.
  for (int p = 0; p < 8; p++) {
    uint16_t paired = 1, proto = (uint16_t)p;
    const uint8_t note = mtxValidateTxLink(&paired, &proto);
    if (p == 0 || p == 3) CHECK(note == 0 && paired == 1 && proto == p);
    else CHECK(note == 1 && paired == 0 && proto == 0);
  }
  uint16_t pd = 0, pr = 3; mtxValidateTxLink(&pd, &pr); CHECK(pr == 0);
  testEnd("T12");
}

// ============================================================
// T13 - the pairing window
// ============================================================
static void t13()
{
  testBegin();
  const uint8_t* ids[1] = { kC };
  MtxRxBinding cur = bindingV3(kR1, ids, 1);
  MtxPairWindow w; mtxPairOpen(&w, &cur, 1000);
  CHECK(w.close_at_ms == 61000);
  CHECK(mtxPairOnRequest(&w, false, kA, kR1, -50.0f) == MTX_PR_OFFER);
  CHECK(mtxPairOnRequest(&w, false, kA, kR1, -60.0f) == MTX_PR_OFFER);       // the gate is inclusive
  CHECK(mtxPairOnRequest(&w, false, kA, kR1, -60.5f) == MTX_PR_RSSI);
  const uint8_t zero[3] = { 0, 0, 0 };
  CHECK(mtxPairOnRequest(&w, false, zero, kR1, -40.0f) == MTX_PR_ZERO);
  CHECK(mtxPairOnRequest(&w, false, kR1, kR1, -40.0f) == MTX_PR_RX_ID);
  CHECK(mtxPairOnRequest(&w, true,  zero, kR1, -40.0f) == MTX_PR_ZERO);
  CHECK(mtxPairOnConfirm(&w, false, kA) == 0); CHECK(w.close_at_ms == 81000);
  CHECK(mtxPairOnConfirm(&w, false, kA) == 0); CHECK(w.n == 1 && w.close_at_ms == 81000);   // a refresh does not extend
  CHECK(mtxPairOnRequest(&w, true, kB, kR1, -40.0f) == MTX_PR_LEGACY_LATE);                 // 0xAB after an enrolment
  CHECK(mtxPairOnConfirm(&w, false, kB) == 1); CHECK(w.close_at_ms == 101000);
  CHECK(mtxPairOnConfirm(&w, false, kC) == 2); CHECK(w.close_at_ms == 121000);              // 120 s cap from the open
  const uint8_t d4[3] = { 0x5C, 0x44, 0x01 }, d5[3] = { 0x5C, 0x55, 0x02 };
  CHECK(mtxPairOnConfirm(&w, false, d4) == 3); CHECK(w.close_at_ms == 121000 && w.closed && w.close_reason == MTX_PC_FULL);
  CHECK(mtxPairOnRequest(&w, false, d5, kR1, -40.0f) == MTX_PR_CLOSED);
  MtxRxBinding out = cur;
  CHECK(mtxPairCommit(&w, &out, kR1));
  CHECK(out.paired == 1 && mtxIdEqual(out.own_address, kR1) && mtxIdIsZero(out.dest_address));
  CHECK(mtxIdEqual(mtxEntryId(&out, 0), kA) && mtxIdEqual(mtxEntryId(&out, 1), kB) && mtxIdEqual(mtxEntryId(&out, 2), kC));
  CHECK(mtxValidEntryCount(&out) == 4);                     // rebuilt from this window, never appended to the old list
  // Full list: a new ID refused, a listed one may refresh.
  MtxPairWindow f; mtxPairOpen(&f, &cur, 0);
  mtxPairOnConfirm(&f, false, kA); mtxPairOnConfirm(&f, false, kB); mtxPairOnConfirm(&f, false, kC);
  f.n = 4; memcpy(&f.ids[9], d4, 3); f.closed = false;
  CHECK(mtxPairOnRequest(&f, false, d5, kR1, -40.0f) == MTX_PR_FULL);
  CHECK(mtxPairOnRequest(&f, false, kB, kR1, -40.0f) == MTX_PR_OFFER);
  // A legacy enrolment is the sole entry and closes the window at once.
  MtxPairWindow l; mtxPairOpen(&l, &cur, 0);
  CHECK(mtxPairOnRequest(&l, true, kA, kR1, -45.0f) == MTX_PR_OFFER);
  mtxPairOnConfirm(&l, true, kA);
  CHECK(l.closed && l.close_reason == MTX_PC_LEGACY);
  CHECK(mtxPairOnRequest(&l, false, kB, kR1, -45.0f) == MTX_PR_CLOSED);
  MtxRxBinding lo; memset(&lo, 0, sizeof(lo));
  CHECK(mtxPairCommit(&l, &lo, kR1) && mtxIdEqual(lo.dest_address, kA) && (lo.mtx_flags[0] & kMtxFlagLegacy));
  // Abort: nobody enrolled -> the old binding back, nothing written.
  MtxPairWindow a; mtxPairOpen(&a, &cur, 0);
  CHECK(!mtxPairPoll(&a, 59999, false)); CHECK(mtxPairPoll(&a, 60000, false) && a.close_reason == MTX_PC_TIME);
  MtxRxBinding ab; memset(&ab, 0xEE, sizeof(ab));
  CHECK(!mtxPairCommit(&a, &ab, kR1) && memcmp(&ab, &cur, sizeof(ab)) == 0);
  MtxPairWindow bp; mtxPairOpen(&bp, &cur, 0);
  CHECK(mtxPairPoll(&bp, 100, true) && bp.close_reason == MTX_PC_BIND);
  // Y-2: the bounds guard. A new ID is never enrolled past the fourth slot or into a closed window: kMtxNone, and
  // nothing in the window changes (before the guard a fifth confirm wrote past ids[] into flags[] and `closed`).
  { MtxPairWindow g; mtxPairOpen(&g, &cur, 0);
    mtxPairOnConfirm(&g, false, kA); mtxPairOnConfirm(&g, false, kB); mtxPairOnConfirm(&g, false, kC);
    mtxPairOnConfirm(&g, false, d4);
    CHECK(g.n == 4 && g.closed);
    uint8_t ids0[kMtxSlots * 3], fl0[kMtxSlots]; memcpy(ids0, g.ids, sizeof(ids0)); memcpy(fl0, g.flags, sizeof(fl0));
    const uint32_t close0 = g.close_at_ms; const uint8_t reason0 = g.close_reason;
    CHECK(mtxPairOnConfirm(&g, false, d5) == kMtxNone);                     // closed and full
    g.closed = false;                                                       // a caller that ignored "closed":
    CHECK(mtxPairOnConfirm(&g, false, d5) == kMtxNone);                     // still refused on the count
    CHECK(mtxPairOnConfirm(&g, true, d5) == kMtxNone);                      // a legacy one too
    CHECK(g.n == 4 && !g.closed && g.close_at_ms == close0 && g.close_reason == reason0);
    CHECK(memcmp(g.ids, ids0, sizeof(ids0)) == 0 && memcmp(g.flags, fl0, sizeof(fl0)) == 0);
    CHECK(mtxPairOnConfirm(&g, false, kB) == 1 && g.n == 4);               // a refresh still answers its slot
    MtxRxBinding gb; memset(&gb, 0, sizeof(gb));
    CHECK(mtxPairCommit(&g, &gb, kR1) && mtxValidEntryCount(&gb) == 4); }
  // ... and a window closed with room (a legacy enrolment closes it at once) enrols nothing more.
  { MtxPairWindow lc; mtxPairOpen(&lc, &cur, 0);
    mtxPairOnConfirm(&lc, true, kA);
    CHECK(lc.closed && lc.n == 1);
    CHECK(mtxPairOnConfirm(&lc, false, kB) == kMtxNone && lc.n == 1 && lc.flags[1] == 0 && mtxIdIsZero(&lc.ids[3])); }
  // The remote's OFFER gate.
  CHECK(mtxPairOfferOk(kA, kA, kR1, -60.0f));
  CHECK(!mtxPairOfferOk(kB, kA, kR1, -40.0f));
  CHECK(!mtxPairOfferOk(kA, kA, zero, -40.0f));
  CHECK(!mtxPairOfferOk(kA, kA, kR1, -61.0f));
  testEnd("T13");
}

// ============================================================
// T14 - old/new mixes (M1-M15), with an SW36 classifier model and a pre-R-3 model
// ============================================================
// SW36 (today): accepts a frame only if paired and bytes 0-2 equal its own_address (and the CRC). Byte 3 0xF1-0xF4 is
// a meta (values outside 0/1/2/0x80+ ignored); anything else, 0xF5-0xFF included, is THROTTLE.
static bool sw36Accepts(const uint8_t own[3], const uint8_t f[6]) { return memcmp(f, own, 3) == 0 && f[5] == crc8((uint8_t*)f, 5); }
static bool sw36IsThrottle(const uint8_t f[6]) { return !(f[3] >= 0xF1 && f[3] <= 0xF4); }
static void t14()
{
  testBegin();
  uint8_t f[6];
  // M2: a migrated remote (proto 0) on an old buggy sends today's frames, byte for byte.
  MtxLinkRec leg; leg.proto = kMtxProtoLegacy; memcpy(leg.dest_address, kR1, 3); memcpy(leg.efuse_id, kA, 3);
  for (int thr = 0; thr < 256; thr += 3) {
    CHECK(mtxBuildUplink(MTX_B_CTRL, &leg, (uint8_t)thr, 99, f, crc8));
    uint8_t today[6] = { kR1[0], kR1[1], kR1[2], (uint8_t)(thr > 0xF0 ? 0xF0 : thr), 99, 0 }; today[5] = crc8(today, 5);
    CHECK(memcmp(f, today, 6) == 0 && sw36Accepts(kR1, f));
  }
  CHECK(mtxBuildUplink(MTX_B_LOCKED, &leg, 0, 0, f, crc8) && f[3] == 0 && f[4] == 127 && sw36Accepts(kR1, f));
  CHECK(!mtxBuildUplink(MTX_B_SEIZE, &leg, 0, 0, f, crc8) && !mtxBuildUplink(MTX_B_SUMMON, &leg, 0, 0, f, crc8));
  // M8 / F20: an old buggy never accepts a link-ID frame (the made-up batch: IDs start 0x5C, link IDs 0x00).
  MtxLinkRec v3; v3.proto = kMtxProtoV3; memcpy(v3.dest_address, kR1, 3); memcpy(v3.efuse_id, kA, 3);
  const uint8_t kinds[5] = { MTX_B_CTRL, MTX_B_LOCKED, MTX_B_SEIZE, MTX_B_SUMMON, MTX_B_META };
  for (int k = 0; k < 5; k++) {
    CHECK(mtxBuildUplink(kinds[k], &v3, (kinds[k] == MTX_B_META) ? 0xF2 : 50, 1, f, crc8));
    CHECK(!sw36Accepts(kR1, f) && !sw36Accepts(kR2, f) && !sw36Accepts(kA, f));
  }
  // Pre-R-3 model (2026-04-25 .. 2026-09-28): any accepted 0xF1 frame refreshed the gate. The builder never produces a
  // LOCKED / SEIZE / SUMMON code with a plain address, so such a buggy can never be held open by one.
  for (int p = 0; p < 2; p++) {
    MtxLinkRec r = p ? v3 : leg;
    for (int k = 1; k <= 5; k++) for (int b = 0; b < 256; b++) {
      if (!mtxBuildUplink((uint8_t)k, &r, (k == MTX_B_META) ? 0xF1 : 0, (uint8_t)b, f, crc8)) continue;
      const uint8_t kind = mtxKind(f[3], f[4]);
      if (kind == MTX_K_LOCKED || kind == MTX_K_SEIZE || kind == MTX_K_SUMMON) CHECK(r.proto == kMtxProtoV3 && !sw36Accepts(kR1, f));
      CHECK(f[3] < 0xF5);                                        // nothing ever reads as throttle 245+
    }
  }
  // M4: a migrated v3 buggy keeps an old remote only where its stored address is its chip ID.
  MtxRxBinding mig; memset(&mig, 0, sizeof(mig)); mig.paired = 1; memcpy(mig.own_address, kR1, 3); memcpy(mig.dest_address, kA, 3);
  MtxRxBinding mig2 = mig;
  mtxValidateRxBinding(&mig, kR1);  CHECK(mtxRxMode(&mig, kR1) == MTX_MODE_SINGLE);
  mtxValidateRxBinding(&mig2, kR2); CHECK(mtxRxMode(&mig2, kR2) == MTX_MODE_UNPAIRED);
  // M7: two old remotes cloned to one address act as the one legacy entry (today's hazard, unchanged).
  CHECK(mtxLookupSlot(&mig, kR1, kR1) == kMtxSlotImplicit);
  // M9 / M10: another buggy's remote, or a remote re-paired elsewhere, is unknown here (T2).
  const uint8_t* ids[1] = { kA };
  MtxRxBinding old_list = bindingV3(kR1, ids, 1);
  uint8_t Lnew[3]; mtxIdXor(kA, kR2, Lnew);                       // A now builds L for buggy 2
  CHECK(mtxLookupSlot(&old_list, kR1, Lnew) == kMtxSlotUnknown);
  // M13 / M14: identity from the chip; a config imported from another board loses its binding (T12).
  MtxRxBinding imp = bindingV3(kR2, ids, 1);
  mtxValidateRxBinding(&imp, kR1); CHECK(imp.paired == 0 && mtxRxMode(&imp, kR1) == MTX_MODE_UNPAIRED);
  // M5 / M15: a legacy enrolment is the sole entry and closes the window (T13).
  MtxPairWindow w; mtxPairOpen(&w, &old_list, 0); mtxPairOnConfirm(&w, true, kA);
  CHECK(w.closed && mtxPairOnRequest(&w, false, kB, kR1, -40.0f) == MTX_PR_CLOSED);
  // M6: full v3 - the round trip of a link-ID frame on its own buggy.
  MtxRxBinding v3list = bindingV3(kR1, ids, 1);
  CHECK(mtxBuildUplink(MTX_B_CTRL, &v3, 40, 127, f, crc8) && mtxLookupSlot(&v3list, kR1, f) == 0);
  // SW36 reads every 0xF5-0xFF byte as throttle: the builder never makes one.
  uint8_t t35[6] = { kR1[0], kR1[1], kR1[2], 0xF5, 0, 0 }; CHECK(sw36IsThrottle(t35));
  testEnd("T14");
}

// ============================================================
// T15 - the remote's frame builder (+ T32 builder rows)
// ============================================================
static void t15()
{
  testBegin();
  MtxLinkRec v3; v3.proto = kMtxProtoV3; memcpy(v3.dest_address, kR1, 3); memcpy(v3.efuse_id, kA, 3);
  uint8_t L[3]; mtxIdXor(kA, kR1, L);
  uint8_t f[6];
  CHECK(mtxBuildUplink(MTX_B_CTRL, &v3, 0xFF, 200, f, crc8) && f[3] == 0xF0 && f[4] == 200);   // clamp
  CHECK(memcmp(f, L, 3) == 0 && f[5] == crc8(f, 5));                                               // the CRC covers L
  CHECK(mtxBuildUplink(MTX_B_LOCKED, &v3, 0, 0, f, crc8) && f[3] == 0xF1 && f[4] == 0x10 && memcmp(f, L, 3) == 0);
  CHECK(mtxBuildUplink(MTX_B_SEIZE,  &v3, 0, 0, f, crc8) && f[3] == 0xF1 && f[4] == 0x5A && memcmp(f, L, 3) == 0);
  CHECK(mtxBuildUplink(MTX_B_SUMMON, &v3, 0, 0, f, crc8) && f[3] == 0xF1 && f[4] == 0x3C && memcmp(f, L, 3) == 0);
  CHECK(mtxBuildUplink(MTX_B_META, &v3, 0xF3, 0x03, f, crc8) && f[3] == 0xF3 && memcmp(f, L, 3) == 0);   // GPS announce
  // META can never smuggle one of the three codes, nor any type outside 0xF1-0xF4.
  CHECK(!mtxBuildUplink(MTX_B_META, &v3, 0xF1, 0x10, f, crc8));
  CHECK(!mtxBuildUplink(MTX_B_META, &v3, 0xF1, 0x5A, f, crc8));
  CHECK(!mtxBuildUplink(MTX_B_META, &v3, 0xF1, 0x3C, f, crc8));
  for (int t = 0; t < 256; t++) {
    const bool ok = mtxBuildUplink(MTX_B_META, &v3, (uint8_t)t, 0x01, f, crc8);
    CHECK(ok == (t >= 0xF1 && t <= 0xF4));
  }
  // Every protocol other than 0 and 3 builds nothing.
  for (int p = 0; p < 16; p++) {
    MtxLinkRec r = v3; r.proto = (uint16_t)p;
    const bool ok = mtxBuildUplink(MTX_B_CTRL, &r, 10, 127, f, crc8);
    CHECK(ok == (p == 0 || p == 3));
    uint8_t a[3]; CHECK(mtxUplinkAddr(&r, a) == (p == 0 || p == 3));
  }
  // Exhaustive: no output with byte 3 >= 0xF5; the internal queue types never appear as byte 3.
  for (int p = 0; p < 2; p++) {
    MtxLinkRec r = v3; r.proto = p ? kMtxProtoV3 : kMtxProtoLegacy;
    for (int k = 0; k < 8; k++) for (int a = 0; a < 256; a++) for (int b = 0; b < 256; b += 15) {
      if (!mtxBuildUplink((uint8_t)k, &r, (uint8_t)a, (uint8_t)b, f, crc8)) continue;
      CHECK(f[3] < 0xF5);
      if (k != MTX_B_CTRL) CHECK(f[3] != kMtxQueueSeize && f[3] != kMtxQueueSummon);
    }
    for (int q = 0; q < 256; q++) {
      if (!mtxBuildUplinkFromQueue((uint8_t)q, 0x00, &r, f, crc8)) continue;
      CHECK(f[3] >= 0xF1 && f[3] <= 0xF4);                    // a queued item is always a meta-range byte 3
    }
  }
  CHECK(mtxBuildUplinkFromQueue(kMtxQueueSeize, 0, &v3, f, crc8) && f[3] == 0xF1 && f[4] == 0x5A);
  CHECK(mtxBuildUplinkFromQueue(kMtxQueueSummon, 0, &v3, f, crc8) && f[3] == 0xF1 && f[4] == 0x3C);
  // A failed build leaves out untouched.
  memset(f, 0xAB, 6); MtxLinkRec bad = v3; bad.proto = 7;
  CHECK(!mtxBuildUplink(MTX_B_CTRL, &bad, 1, 1, f, crc8) && f[0] == 0xAB && f[5] == 0xAB);
  testEnd("T15");
}

// ============================================================
// T16 - the remote: status, rate, auto-lock, take-back
// ============================================================
static void t16()
{
  testBegin();
  MtxTxStatus t; mtxTxStatusInit(&t);
  CHECK(mtxTxFast(&t, 1000, false, false, 0));                         // never a reply -> fast
  CHECK(mtxTxOnReply(&t, MTX_S_OWNER, 1000) & MTX_TXE_BECAME_OWNER); CHECK(t.owner && !t.lost_owner);
  CHECK(mtxTxOnReply(&t, MTX_S_BUSY, 1100) == 0); CHECK(!t.owner && !t.lost_owner);
  CHECK(mtxTxOnReply(&t, MTX_S_BUSY, 1200) == 0);
  CHECK(mtxTxOnReply(&t, MTX_S_BUSY, 1300) & MTX_TXE_LOST_OWNER); CHECK(t.lost_owner);
  CHECK(mtxTxOnReply(&t, MTX_S_OWNER_SOLO, 1400) & MTX_TXE_BECAME_OWNER); CHECK(!t.lost_owner);
  // 2 SEIZED in a row: "tA" and lost owner on the second.
  CHECK(mtxTxOnReply(&t, MTX_S_SEIZED, 1500) == 0);
  const uint8_t e2 = mtxTxOnReply(&t, MTX_S_SEIZED, 1600);
  CHECK((e2 & MTX_TXE_SEIZED_NOTICE) && (e2 & MTX_TXE_LOST_OWNER));
  // The reserved status reads as BUSY.
  mtxTxStatusInit(&t); mtxTxOnReply(&t, 7, 100); CHECK(t.last[0] == MTX_S_BUSY);
  // Rate: slow only with all conditions; F-4: input makes it fast only while FREE.
  mtxTxStatusInit(&t);
  for (int i = 0; i < 3; i++) mtxTxOnReply(&t, MTX_S_BUSY, 1000 + 100 * i);
  CHECK(!mtxTxFast(&t, 1300, false, false, 0));
  CHECK(!mtxTxFast(&t, 1300, false, false, 1250));                      // input while BUSY: still slow
  CHECK(mtxTxFast(&t, 1300, true, false, 0));                           // meta pending
  CHECK(mtxTxFast(&t, 1300, false, true, 0));                           // gesture armed
  CHECK(mtxTxFast(&t, 6300, false, false, 0));                          // no valid reply for 5 s
  mtxTxOnReply(&t, MTX_S_FREE, 1400);
  CHECK(!mtxTxFast(&t, 1500, false, false, 0));                         // FREE, no input
  CHECK(mtxTxFast(&t, 1500, false, false, 1450));                       // FREE + input -> fast
  CHECK(mtxTxFast(&t, 4450, false, false, 1450) && !mtxTxFast(&t, 4451, false, false, 1450));   // for 3 s
  for (int st = 3; st <= 6; st++) { mtxTxOnReply(&t, (uint8_t)st, 5000); CHECK(!mtxTxFast(&t, 5010, false, false, 5005)); }
  // Slow skip: 1 cycle in 10 transmits; the hop is 33 or 66, never 0.
  uint8_t ctr = 0; int sends = 0;
  for (int i = 0; i < 100; i++) sends += mtxTxCycleSends(true, &ctr) ? 1 : 0;
  CHECK(sends == 10);
  for (int i = 0; i < 100; i++) CHECK(mtxTxCycleSends(false, &ctr));
  for (int i = 0; i < 1000; i++) { const uint8_t hop = mtxTxSlowMissHopMs(rnd()); CHECK(hop == 33 || hop == 66); }
  // The auto-lock timeout selection.
  mtxTxStatusInit(&t);
  CHECK(mtxTxAutolockTimeoutS(&t, 120, 0) == 0);                        // never a reply
  mtxTxOnReply(&t, MTX_S_OWNER_SOLO, 10); CHECK(mtxTxAutolockTimeoutS(&t, 120, 0) == 0);
  mtxTxOnReply(&t, MTX_S_OWNER, 20);      CHECK(mtxTxAutolockTimeoutS(&t, 120, 30) == 30);
  for (int st = 2; st <= 6; st++) { mtxTxOnReply(&t, (uint8_t)st, 30); CHECK(mtxTxAutolockTimeoutS(&t, 120, 0) == 120); }
  CHECK(mtxTxAutolockDue(120, 130000, 10000, false) && !mtxTxAutolockDue(120, 129999, 10000, false));
  CHECK(!mtxTxAutolockDue(0, 999999, 0, false) && !mtxTxAutolockDue(120, 999999, 0, true));
  CHECK(mtxTxAutolockSkip(true, false, false, false, false, false) && mtxTxAutolockSkip(false, true, false, false, false, false));
  CHECK(mtxTxAutolockSkip(false, false, true, false, false, false) && mtxTxAutolockSkip(false, false, false, false, false, true));
  CHECK(!mtxTxAutolockSkip(false, false, false, false, false, false));
  // The take-back verdict: the last two SEIZABLE, the newer at most 5 s old.
  mtxTxStatusInit(&t);
  mtxTxOnReply(&t, MTX_S_SEIZABLE, 1000); CHECK(!mtxTxSeizeVerdict(&t, 1000));
  mtxTxOnReply(&t, MTX_S_SEIZABLE, 2000); CHECK(mtxTxSeizeVerdict(&t, 7000) && !mtxTxSeizeVerdict(&t, 7001));
  // The gesture: tap, second press within 400 ms, hold 1200 ms -> FIRE; fast from the first press.
  MtxSeizeGesture g; mtxSeizeGestureInit(&g);
  uint32_t now = 3000;
  CHECK(mtxSeizeGestureStep(&g, now, true, 0, false, &t, 1200) & MTX_SGO_ARMED);
  CHECK(!(mtxSeizeGestureStep(&g, now + 300, true, 0, false, &t, 1200) & MTX_SGO_SUPPRESS));   // the tap is not suppressed
  mtxSeizeGestureStep(&g, now + 350, false, 0, false, &t, 1200);                                // tap released
  CHECK(mtxSeizeGestureStep(&g, now + 600, true, 0, false, &t, 1200) & MTX_SGO_SUPPRESS);       // second press
  mtxTxOnReply(&t, MTX_S_SEIZABLE, now + 1000);
  CHECK(!(mtxSeizeGestureStep(&g, now + 1700, true, 0, false, &t, 1200) & MTX_SGO_FIRE));
  uint8_t o = mtxSeizeGestureStep(&g, now + 1800, true, 0, false, &t, 1200);
  CHECK((o & MTX_SGO_FIRE) && (o & MTX_SGO_SUPPRESS));
  CHECK(mtxSeizeGestureStep(&g, now + 2500, true, 0, false, &t, 1200) & MTX_SGO_SUPPRESS);      // still held: no display cycle
  CHECK(mtxSeizeGestureStep(&g, now + 2600, false, 0, false, &t, 1200) == 0 && g.phase == MTX_SG_IDLE);
  // Aborts: throttle, early release, lock, a status change, the verdict at completion.
  for (int c = 0; c < 5; c++) {
    mtxTxStatusInit(&t); mtxTxOnReply(&t, MTX_S_SEIZABLE, 9000); mtxTxOnReply(&t, MTX_S_SEIZABLE, 9500);
    mtxSeizeGestureInit(&g); now = 10000;
    mtxSeizeGestureStep(&g, now, true, 0, false, &t, 1200);
    mtxSeizeGestureStep(&g, now + 200, false, 0, false, &t, 1200);
    mtxSeizeGestureStep(&g, now + 400, true, 0, false, &t, 1200);
    uint8_t thr = 0; bool locked = false, down = true;
    if (c == 0) thr = 5;
    if (c == 1) down = false;
    if (c == 2) locked = true;
    if (c == 3) mtxTxOnReply(&t, MTX_S_FREE, now + 500);
    if (c == 4) { mtxSeizeGestureStep(&g, now + 900, true, 0, false, &t, 1200); now += 5000; }   // verdict stale at the end
    o = mtxSeizeGestureStep(&g, now + 1700, down, thr, locked, &t, 1200);
    CHECK((o & MTX_SGO_ABORT) && !(o & MTX_SGO_FIRE));
  }
  // Not SEIZABLE: a RIGHT press is a normal press; no gesture.
  mtxTxStatusInit(&t); mtxTxOnReply(&t, MTX_S_BUSY, 100); mtxSeizeGestureInit(&g);
  CHECK(mtxSeizeGestureStep(&g, 200, true, 0, false, &t, 1200) == 0);
  // A long first press is a hold, not a tap.
  mtxTxStatusInit(&t); mtxTxOnReply(&t, MTX_S_SEIZABLE, 100); mtxTxOnReply(&t, MTX_S_SEIZABLE, 150); mtxSeizeGestureInit(&g);
  mtxSeizeGestureStep(&g, 200, true, 0, false, &t, 1200);
  CHECK(mtxSeizeGestureStep(&g, 1200, true, 0, false, &t, 1200) == 0 && g.phase == MTX_SG_WAIT_RELEASE);
  // After a fire: an owner status within 1 s -> confirm, else "St".
  mtxTxStatusInit(&t); mtxTxOnReply(&t, MTX_S_SEIZABLE, 100);
  CHECK(mtxTxOwnerWait(&t, 500, 900, kMtxSeizeConfirmMs) == MTX_W_WAIT);
  mtxTxOnReply(&t, MTX_S_OWNER, 950);
  CHECK(mtxTxOwnerWait(&t, 500, 960, kMtxSeizeConfirmMs) == MTX_W_OWNER);
  mtxTxStatusInit(&t); mtxTxOnReply(&t, MTX_S_OWNER, 400);                 // an owner status from BEFORE the send
  CHECK(mtxTxOwnerWait(&t, 500, 1501, kMtxSeizeConfirmMs) == MTX_W_TIMEOUT);
  // V2.5-Evo - 2026-10-21 - P3-1: judged on the FIRST owner arrival, however late the poll. The take-back fires while RIGHT
  // is still held (loop() blocked); the buggy grants at send + 0.4 s and then says OWNER at 10 Hz.
  mtxTxStatusInit(&t); mtxTxOnReply(&t, MTX_S_SEIZABLE, 900); mtxTxOnReply(&t, MTX_S_SEIZABLE, 950);
  for (uint32_t a = 1400; a <= 3900; a += 100) mtxTxOnReply(&t, MTX_S_OWNER, a);
  CHECK(t.owner && t.owner_since_ms == 1400 && t.last_ms[0] == 3900);
  CHECK(mtxTxOwnerWait(&t, 1000, 4000, kMtxSeizeConfirmMs) == MTX_W_OWNER);   // polled at send + 3 s: still OWNER
  CHECK(mtxTxOwnerWait(&t, 1000, 1450, kMtxSeizeConfirmMs) == MTX_W_OWNER);   // polled at once: the same answer
  CHECK(mtxAgeMs(t.last_ms[0], 1000) > kMtxSeizeConfirmMs);                   // (the newest status alone said "too late")
  // No owner status at all: WAIT inside the window, TIMEOUT after it, however late the poll.
  mtxTxStatusInit(&t); for (uint32_t a = 900; a <= 3900; a += 100) mtxTxOnReply(&t, MTX_S_SEIZABLE, a);
  CHECK(mtxTxOwnerWait(&t, 1000, 2000, kMtxSeizeConfirmMs) == MTX_W_WAIT);
  CHECK(mtxTxOwnerWait(&t, 1000, 4000, kMtxSeizeConfirmMs) == MTX_W_TIMEOUT);
  // The window edge: a first owner status exactly at send + 1 s counts, 1 ms later it does not (polled late in both).
  mtxTxStatusInit(&t); mtxTxOnReply(&t, MTX_S_SEIZABLE, 900); mtxTxOnReply(&t, MTX_S_OWNER, 2000);
  CHECK(mtxTxOwnerWait(&t, 1000, 3000, kMtxSeizeConfirmMs) == MTX_W_OWNER);
  mtxTxStatusInit(&t); mtxTxOnReply(&t, MTX_S_SEIZABLE, 900); mtxTxOnReply(&t, MTX_S_OWNER, 2001);
  CHECK(mtxTxOwnerWait(&t, 1000, 3000, kMtxSeizeConfirmMs) == MTX_W_TIMEOUT);
  // Control that came and went before the poll is not reported as a success; a second edge re-stamps the arrival.
  mtxTxStatusInit(&t); mtxTxOnReply(&t, MTX_S_SEIZABLE, 900); mtxTxOnReply(&t, MTX_S_OWNER, 1400);
  mtxTxOnReply(&t, MTX_S_BUSY, 1500); mtxTxOnReply(&t, MTX_S_BUSY, 1600); mtxTxOnReply(&t, MTX_S_BUSY, 1700);
  CHECK(!t.owner && mtxTxOwnerWait(&t, 1000, 4000, kMtxSeizeConfirmMs) == MTX_W_TIMEOUT);
  mtxTxOnReply(&t, MTX_S_OWNER, 1800);
  CHECK(t.owner_since_ms == 1800 && mtxTxOwnerWait(&t, 1000, 4000, kMtxSeizeConfirmMs) == MTX_W_OWNER);
  // Across the millis() wrap.
  { const uint32_t s0 = 0xFFFFFF00u;                                    // the grant lands 144 ms after the wrap
    mtxTxStatusInit(&t); mtxTxOnReply(&t, MTX_S_SEIZABLE, s0 - 100);
    for (uint32_t k = 400; k <= 2900; k += 100) mtxTxOnReply(&t, MTX_S_OWNER, s0 + k);
    CHECK(mtxTxOwnerWait(&t, s0, s0 + 3000, kMtxSeizeConfirmMs) == MTX_W_OWNER); }
  testEnd("T16");
}

// ============================================================
// T17 - link-fresh windows; Pattern B while slow
// ============================================================
static void t17()
{
  testBegin();
  CHECK(mtxTxLinkFresh(1999, 1000, false) && !mtxTxLinkFresh(2000, 1000, false));
  CHECK(mtxTxLinkFresh(3499, 1000, true) && !mtxTxLinkFresh(3500, 1000, true));
  CHECK(!mtxTxLinkFresh(5, 0, false));
  CHECK(mtxTxLinkFresh(0x00000100u, 0xFFFFFF00u, false));                 // across the wrap
  CHECK(mtxTxPatternBAllowed(false) && !mtxTxPatternBAllowed(true));
  testEnd("T17");
}

// ============================================================
// T18 - the bounded telemetry wait
// ============================================================
static void t18()
{
  testBegin();
  const uint32_t send = 5000;
  CHECK(mtxReplyWaitOnFrame(send + 10, send, false) == MTX_RW_RELISTEN);     // someone else's reply: keep listening
  CHECK(mtxReplyWaitOnFrame(send + 30, send, true) == MTX_RW_ACCEPT);        // ours after a foreign frame
  CHECK(mtxReplyWaitOnFrame(send + 39, send, true) == MTX_RW_ACCEPT);
  CHECK(mtxReplyWaitOnFrame(send + 40, send, true) == MTX_RW_STOP);          // at the bound: nothing, no radio call
  CHECK(mtxReplyWaitOnFrame(send + 45, send, false) == MTX_RW_STOP);
  CHECK(!mtxReplyWaitOpen(send + 40, send) && mtxReplyWaitOpen(send + 39, send));
  CHECK(mtxReplyWaitOpen(0x00000010u, 0xFFFFFFF0u) && !mtxReplyWaitOpen(0x00000018u, 0xFFFFFFF0u));   // wrap
  // A simulated wait loop never touches the radio at or after the bound.
  int radio_calls_after = 0;
  for (uint32_t t = send; t < send + 100; t += 3) {
    const uint8_t r = mtxReplyWaitOnFrame(t, send, (t == send + 42));
    if (r == MTX_RW_RELISTEN && !mtxReplyWaitOpen(t, send)) radio_calls_after++;
    if (r == MTX_RW_STOP || r == MTX_RW_ACCEPT) break;
  }
  CHECK(radio_calls_after == 0);
  const uint8_t own[3] = { 0x5C, 0x01, 0x02 };
  CHECK(mtxTxReplyAddr(3, kA, own) == kA && mtxTxReplyAddr(0, kA, own) == own);
  testEnd("T18");
}

// ============================================================
// T20 - gate 0 and every unpair path
// ============================================================
static void t20()
{
  testBegin();
  const uint8_t* ids[2] = { kA, kB };
  MtxRxBinding b = bindingV3(kR1, ids, 2);
  CHECK(mtxMayAcceptControl(&b, kR1));
  MtxRxBinding p = b; p.paired = 0; CHECK(!mtxMayAcceptControl(&p, kR1));
  MtxRxBinding o = b; memset(o.own_address, 0, 3); CHECK(!mtxMayAcceptControl(&o, kR1));
  MtxRxBinding e = b; memset(e.mtx_flags, 0, 4); CHECK(!mtxMayAcceptControl(&e, kR1));
  // reset_bind (paired = 0, then the validator), ?set paired 0, factory defaults: the list is gone.
  MtxRxBinding r = b; r.paired = 0; mtxValidateRxBinding(&r, kR1);
  CHECK(mtxValidEntryCount(&r) == 0 && !mtxMayAcceptControl(&r, kR1) && mtxRxMode(&r, kR1) == MTX_MODE_UNPAIRED);
  MtxRxBinding f; memset(&f, 0, sizeof(f)); mtxValidateRxBinding(&f, kR1);
  CHECK(mtxIdEqual(f.own_address, kR1) && f.paired == 0 && !mtxMayAcceptControl(&f, kR1));
  // An import made on THIS board keeps its list; one from another board does not.
  MtxRxBinding same = b; CHECK(mtxValidateRxBinding(&same, kR1) == 0 && mtxValidEntryCount(&same) == 2);
  MtxRxBinding other = bindingV3(kR2, ids, 2); mtxValidateRxBinding(&other, kR1); CHECK(mtxValidEntryCount(&other) == 0);
  testEnd("T20");
}

// ============================================================
// T21 - the release model, the gate model, the owner-change hook on the real stickyWindowStep()
// ============================================================
static void t21()
{
  testBegin();
  Sim s; simInit(&s, false, 0x03);
  grantBySqueeze(&s, 0, 1000);
  fr(&s, 0, 0xF1, 1, 1300);                                 // RTM on
  fr(&s, 0, 0xF2, 0x82, 1400); loopTick(&s);                // Follow-Me armed
  fr(&s, 0, 0xF3, 0x03, 1500);                              // a GPS announce left pending
  fr(&s, 0, 200, 127, 1600);                                // driving at 200
  simClear(&s);
  const uint32_t lcp = s.last_control_packet;
  fr(&s, 0, 0xF1, 0x10, 1700);                              // the owner locks -> release(LOCK)
  // The order: throttle 0 / steering centre FIRST, then RTM off, then Follow-Me off, then the resets, then the counter.
  CHECK(logPos(&s, "zero") == 0);
  CHECK(logPos(&s, "zero") < logPos(&s, "rtm_off") && logPos(&s, "rtm_off") < logPos(&s, "fm_off"));
  CHECK(logPos(&s, "fm_off") < logPos(&s, "gps_reset") && logPos(&s, "gps_reset") < logPos(&s, "boot_reset"));
  CHECK(logPos(&s, "boot_reset") < logPos(&s, "seq"));
  CHECK(s.thr == 0 && s.steer == 127 && s.st.owner == kMtxNone);
  CHECK(s.last_control_packet == lcp);                      // no failsafe stamp: the gate shuts on its own clock
  CHECK(s.sticky_cap);                                      // the slow ceiling of the running return
  CHECK(!s.gps_pending && s.gps_len == 6);                  // 6-byte receive mode restored
  CHECK(countEv(&s, MTX_EV_RELEASED) == 1 && s.events[0].second == MTX_REL_LOCK);
  // The gate model: shut at failsafe after the owner's last control packet, whatever the others send at 10 Hz.
  bool reopened = false;
  for (uint32_t t = 1700; t < 4000; t += 100) {
    fr(&s, 1, 0, 127, t);                                   // the other remote at zero (no grant: it never squeezes)
    const bool open = mtxAgeMs(t, s.last_control_packet) < s.failsafe_ms;
    if (t >= lcp + s.failsafe_ms) reopened = reopened || open;
  }
  CHECK(!reopened);
  // The owner-change hook: stickyWindowClose() on the real stickyWindowStep().
  StickyWindowState w; memset(&w, 0, sizeof(w));
  CHECK(stickyWindowStep(&w, 1000, 900, true, 6000));       // an alarm opens a window
  CHECK(stickyWindowStep(&w, 1100, 900, true, 6000));
  stickyWindowClose(&w, 900);                               // the owner changed
  CHECK(!stickyWindowStep(&w, 1200, 900, true, 6000));      // a running window shows nothing any more
  StickyWindowState w2; memset(&w2, 0, sizeof(w2));
  stickyWindowClose(&w2, 950);                              // raised before the change, never shown yet
  CHECK(!stickyWindowStep(&w2, 1300, 950, true, 6000));     // ... never shows
  CHECK(stickyWindowStep(&w2, 1400, 1350, true, 6000));     // a stamp raised AFTER the change: a full window
  int shown = 0; for (uint32_t t = 1500; t < 9000; t += 100) shown += stickyWindowStep(&w2, t, 1350, true, 6000) ? 1 : 0;
  CHECK(shown >= 58 && shown <= 61);
  StickyWindowState w3; memset(&w3, 0, sizeof(w3)); stickyWindowClose(&w3, 0);
  CHECK(!stickyWindowStep(&w3, 100, 0, true, 6000));        // stamp 0 is safe
  // X-10 (c), the lock: the owner locks while its RTM return runs, so the release arms the sticky return cap. A motor
  // pass may run between ANY two steps of that frame; at every such point every pass is subtract-only, and from the
  // throttle zero on it hands on 0 and clears the cap (the trigger byte is now 0, below 25).
  { int points = 0;
    for (int k = 0; k < 40; k++) {
      Sim q; simInit(&q, false, 0x03);
      grantBySqueeze(&q, 0, 1000); fr(&q, 0, 0xF1, 1, 1300); fr(&q, 0, 200, 127, 1400);
      simClear(&q); q.pwm_after = k;
      fr(&q, 0, 0xF1, 0x10, 1500);                               // the lock: release(LOCK)
      const bool inserted = !q.pwm_in.empty();
      const int zpos = logPos(&q, "zero");
      q.pwm_after = -1; pwmPass(&q);                             // the next 10 ms pass
      for (size_t i = 0; i < q.pwm_in.size(); i++) CHECK(q.pwm_out[i] <= q.pwm_in[i]);
      if (inserted && k >= zpos) CHECK(q.pwm_out[0] == 0);
      CHECK(q.pwm_out.back() == 0 && !q.sticky_cap);
      if (!inserted) break;
      points++;
    }
    CHECK(points >= 6); }
  // The receiver's MTX_REL_REBIND (the paired list changed under an owner): the same tested order, no lock / take-back
  // / claim counted, one line with the reason.
  { Sim q; simInit(&q, false, 0x03);
    grantBySqueeze(&q, 0, 1000); fr(&q, 0, 0xF1, 1, 1300);
    simClear(&q);
    mtxRelease(&q.st, &q.h, MTX_REL_REBIND, 1400);
    CHECK(logPos(&q, "zero") == 0 && logPos(&q, "zero") < logPos(&q, "rtm_off") && logPos(&q, "rtm_off") < logPos(&q, "fm_off"));
    CHECK(logPos(&q, "fm_off") < logPos(&q, "gps_reset") && logPos(&q, "boot_reset") < logPos(&q, "seq"));
    CHECK(q.st.owner == kMtxNone && q.thr == 0 && q.sticky_cap);
    CHECK(q.st.n.released_lock == 0 && q.st.n.released_seized == 0 && q.st.n.released_claim == 0);
    CHECK(q.st.n.released_rebind == 1);                                     // Z-7 (a): counted on its own
    CHECK(countEv(&q, MTX_EV_RELEASED) == 1 && q.events.back().second == MTX_REL_REBIND); }
  // V2.5-Evo - 2026-10-21 - P3-4: what a paired-list change does to the outputs (mtxRxRefresh() runs this after the REBIND
  // release, before that wake's frame). Z-1: zero whenever MULTI is on either side; P3-4: the limit once on MULTI -> SINGLE.
  CHECK(mtxListChangeActions(MTX_MODE_MULTI, MTX_MODE_SINGLE)      == (MTX_LC_ZERO | MTX_LC_LIMIT));
  CHECK(mtxListChangeActions(MTX_MODE_MULTI, MTX_MODE_MULTI)       == MTX_LC_ZERO);
  CHECK(mtxListChangeActions(MTX_MODE_MULTI, MTX_MODE_UNPAIRED)    == MTX_LC_ZERO);   // nobody is obeyed: no limit needed
  CHECK(mtxListChangeActions(MTX_MODE_SINGLE, MTX_MODE_MULTI)      == MTX_LC_ZERO);   // the next owner claims from zero anyway
  CHECK(mtxListChangeActions(MTX_MODE_UNPAIRED, MTX_MODE_MULTI)    == MTX_LC_ZERO);
  CHECK(mtxListChangeActions(MTX_MODE_SINGLE, MTX_MODE_SINGLE)     == 0);             // one remote before and after: today's
  CHECK(mtxListChangeActions(MTX_MODE_SINGLE, MTX_MODE_UNPAIRED)   == 0);
  CHECK(mtxListChangeActions(MTX_MODE_UNPAIRED, MTX_MODE_SINGLE)   == 0);
  CHECK(mtxListChangeActions(MTX_MODE_UNPAIRED, MTX_MODE_UNPAIRED) == 0);
  // The edge itself: two remotes, nobody in control, remote 1 holding its trigger at 200 (no zero proof, so no grant). A
  // config write removes remote 0: remote 1 is now the single remote and its next frame takes today's SINGLE path at once.
  { Sim q; simInit(&q, false, 0x03);
    fr(&q, 1, 200, 127, 1000); fr(&q, 1, 200, 127, 1100);
    CHECK(q.st.owner == kMtxNone && q.mtx_grant_ms == 0);
    const uint8_t lc = mtxListChangeActions(MTX_MODE_MULTI, MTX_MODE_SINGLE);   // mtxRxRefresh(), in its order:
    q.now = 1150;
    if (lc & MTX_LC_ZERO)  q.h.zero_outputs(q.h.ctx);                            // 1. the zero (Z-1)
    if (lc & MTX_LC_LIMIT) q.h.publish_limit(q.h.ctx, mtxStampOf(q.now));        // 2. the limit (P3-4)
    CHECK(q.thr == 0 && q.steer == 127 && q.mtx_grant_ms == 1150);
    pwmPass(&q); CHECK(q.pwm_out.back() == 0);                                   // a motor pass in between: zero
    mtxRxStateInit(&q.st); q.cfg.single = true; q.cfg.valid_mask = 0x01;          // 3. the fresh SINGLE model
    fr(&q, 0, 200, 127, 1200);                                                    // the held trigger is applied at once ...
    CHECK(q.thr == 200);
    q.now = 1200; pwmPass(&q);
    CHECK(q.pwm_out.back() == mtxTakeoverCap(1200, 1150, 64, 4000) && q.pwm_out.back() <= 70);   // ... but at ~25 %
    q.now = 3150; pwmPass(&q); CHECK(q.pwm_out.back() == 159);                    // 2 s in
    q.now = 5150; pwmPass(&q); CHECK(q.pwm_out.back() == 200);                    // 4 s: the rider's own 200, no limit left
    // Before P3-4 (no limit published) the same frame was handed on at 200 on the very first pass.
    Sim r; simInit(&r, true, 0x01); fr(&r, 0, 200, 127, 1200); r.now = 1200; pwmPass(&r);
    CHECK(r.thr == 200 && r.pwm_out.back() == 200); }
  // Y-1: the boot check. The Sim's table is complete; with any one hook missing it is not.
  { Sim q; simInit(&q, false, 0x03);
    CHECK(mtxRxHooksComplete(&q.h) && !mtxRxHooksComplete(NULL));
    MtxRxHooks x = q.h; x.read_env = NULL;          CHECK(!mtxRxHooksComplete(&x));
    x = q.h; x.zero_outputs = NULL;                  CHECK(!mtxRxHooksComplete(&x));
    x = q.h; x.synthetic_rtm_off = NULL;             CHECK(!mtxRxHooksComplete(&x));
    x = q.h; x.synthetic_fm_off = NULL;              CHECK(!mtxRxHooksComplete(&x));
    x = q.h; x.reset_rider_gps = NULL;               CHECK(!mtxRxHooksComplete(&x));
    x = q.h; x.reset_boot_id = NULL;                 CHECK(!mtxRxHooksComplete(&x));
    x = q.h; x.publish_limit = NULL;                 CHECK(!mtxRxHooksComplete(&x));
    x = q.h; x.owner_changed = NULL;                 CHECK(!mtxRxHooksComplete(&x));
    x = q.h; x.apply_owner = NULL;                   CHECK(!mtxRxHooksComplete(&x));
    x = q.h; x.meta = NULL;                          CHECK(!mtxRxHooksComplete(&x));
    x = q.h; x.reply = NULL;                         CHECK(!mtxRxHooksComplete(&x));
    x = q.h; x.event = NULL;                         CHECK(!mtxRxHooksComplete(&x)); }
  testEnd("T21");
}

// ============================================================
// T22 - single remote, and a second remote off or locked: today's writes, packet for packet
// ============================================================
struct Ref {   // today's triggeredReceive() branch for a frame addressed to this buggy with a good CRC
  uint8_t thr, steer; uint32_t last_packet, lcp; uint8_t fm_mode; bool rtm; bool gps_pending; uint8_t aux; uint32_t now;
};
static void refInit(Ref* r) { r->thr = 0; r->steer = 127; r->last_packet = 0; r->lcp = 0; r->fm_mode = 0xFF; r->rtm = false; r->gps_pending = false; r->aux = 0; r->now = 0; }
static void refFrame(Ref* r, uint8_t b3, uint8_t b4, uint32_t t)
{
  if (b3 == 0xF1)      { r->last_packet = t; if (b4 == 0) r->rtm = false; else if (b4 == 1) r->rtm = true; }
  else if (b3 == 0xF2) { r->last_packet = t; uint8_t m = b4 & 7; if (m > 5) m = 0; r->fm_mode = m; }
  else if (b3 == 0xF4) { r->last_packet = t; r->aux = b4; }
  else if (b3 == 0xF3) { r->gps_pending = true; }
  else                 { r->last_packet = t; r->thr = b3; r->steer = b4; r->lcp = t; }
}
static bool same(const Sim* s, const Ref* r)
{
  return s->thr == r->thr && s->steer == r->steer && s->last_packet == r->last_packet && s->last_control_packet == r->lcp
      && s->env.fm_mode_runtime == r->fm_mode && s->env.rtm_rx_active == r->rtm && s->gps_pending == r->gps_pending;
}

// One silence script. mode: 0 = FM declared m, 1 = FM never declared, 2 = RTM on with the trigger held,
// 3 = RTM on with the trigger released. other: -1 none, 0 = never heard, 1 = LOCKED at 1 Hz.
static bool runScript(bool single, bool legacy, int other, int mode, uint8_t m, uint32_t silence_ms, int* mismatches)
{
  Sim s; Ref r;
  simInit(&s, single, single ? 0x01 : 0x03); refInit(&r);
  s.env.fm_mode_runtime = 0xFF;
  uint32_t t = 100000;
  // Before the script (MULTI): the owner took the free buggy the normal way. Copy that into the reference.
  if (!single) { grantBySqueeze(&s, 0, t - 1000); r.thr = s.thr; r.steer = s.steer; r.last_packet = s.last_packet; r.lcp = s.last_control_packet; }
  const uint8_t seq0 = s.seq;
  simClear(&s);
  std::vector<std::pair<uint8_t, uint8_t> > pre;
  if (mode == 0) pre.push_back(std::make_pair((uint8_t)0xF2, (uint8_t)(m | 0x80)));
  if (mode >= 2) pre.push_back(std::make_pair((uint8_t)0xF1, (uint8_t)1));
  const uint8_t drive = (mode == 2) ? 180 : 0;
  int bad = 0;
  auto send = [&](uint8_t b3, uint8_t b4) {
    fr(&s, 0, b3, b4, t, legacy); refFrame(&r, b3, b4, t); loopTick(&s);
    if (!same(&s, &r)) bad++;
  };
  for (size_t i = 0; i < pre.size(); i++) { send(pre[i].first, pre[i].second); t += 100; }
  for (int i = 0; i < 30; i++) { send(drive, 127); t += 100; }
  // The silence: the radio task wakes on its 2 s timeout; the other remote (if any) sends LOCKED at 1 Hz.
  const uint32_t end = t + silence_ms;
  for (uint32_t q = t; q < end; q += 100) {
    if (other == 1 && ((q - t) % 1000) == 0) { fr(&s, 1, 0xF1, 0x10, q); if (!same(&s, &r)) bad++; }
    if (((q - t) % 2000) == 0) { wake(&s, q); if (!same(&s, &r)) bad++; }
  }
  t = end;
  for (int i = 0; i < 20; i++) { send(drive, 127); t += 100; }
  if (mode == 0 && (t % 3) == 0) send(0xF2, m);              // a keepalive
  *mismatches += bad;
  bool ok = (bad == 0) && s.seq == seq0 && !logHas(&s, "zero") && !logHas(&s, "limit")
         && countEv(&s, MTX_EV_CLAIMABLE) == 0 && countEv(&s, MTX_EV_RELEASED) == 0;
  if (single) for (size_t i = 0; i < s.replies.size(); i++) ok = ok && s.replies[i].second == MTX_S_OWNER_SOLO;
  if (!single) ok = ok && s.st.owner == 0;
  return ok;
}

static void t22()
{
  testBegin();
  const uint32_t silences[5] = { 500, 5000, 10000, 30000, 100000 };
  int mismatches = 0;
  for (int si = 0; si < 5; si++) {
    for (int mode = 0; mode < 4; mode++) {
      const int mmax = (mode == 0) ? 5 : 1;
      for (int m = 1; m <= mmax; m++) {
        CHECK(runScript(true,  false, -1, mode, (uint8_t)m, silences[si], &mismatches));   // SINGLE, a new-style remote
        CHECK(runScript(true,  true,  -1, mode, (uint8_t)m, silences[si], &mismatches));   // SINGLE, a legacy remote
        CHECK(runScript(false, false,  0, mode, (uint8_t)m, silences[si], &mismatches));   // MULTI, the other never heard
        CHECK(runScript(false, false,  1, mode, (uint8_t)m, silences[si], &mismatches));   // MULTI, the other LOCKED at 1 Hz
      }
    }
  }
  CHECK(mismatches == 0);
  // SINGLE: LOCKED is today's 0/127 control packet, stamp included.
  { Sim s; Ref r; simInit(&s, true, 0x01); refInit(&r);
    fr(&s, 0, 100, 140, 1000); refFrame(&r, 100, 140, 1000);
    fr(&s, 0, 0xF1, 0x10, 1100); refFrame(&r, 0, 127, 1100);
    CHECK(same(&s, &r) && s.last_control_packet == 1100 && lastStatus(&s) == MTX_S_OWNER_SOLO); }
  // SINGLE: a SEIZE or SUMMON writes nothing and gets no reply.
  { Sim s; simInit(&s, true, 0x01);
    fr(&s, 0, 0, 127, 1000);
    const uint32_t lp = s.last_packet; const size_t nr = s.replies.size();
    fr(&s, 0, 0xF1, 0x5A, 1100); fr(&s, 0, 0xF1, 0x3C, 1200);
    CHECK(s.last_packet == lp && s.replies.size() == nr && s.seq == 0 && !logHas(&s, "zero") && !logHas(&s, "limit"));
    CHECK(claimable(&s, 1200) == MTX_CL_NO); }
  // X-4: MULTI, the second remote off. (1) lock -> nobody owns -> a squeeze from zero is a grant under the limit.
  { Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000);
    fr(&s, 0, 0xF1, 0x10, 2000);                             // lock: released
    CHECK(s.st.owner == kMtxNone);
    simClear(&s);
    fr(&s, 0, 0, 127, 5000); fr(&s, 0, 0, 127, 5100); fr(&s, 0, 90, 127, 5200);
    CHECK(s.st.owner == 0 && logHas(&s, "limit") && s.mtx_grant_ms == 5200);
    CHECK(mtxTakeoverCap(5200, s.mtx_grant_ms, 64, 4000) == 64); }
  // (2) lock -> nobody owns -> a summon: a grant, then the owner-path arm; no Follow-Me / RTM write before the new
  //     owner's own metas, and the rest of the burst is a silent no-op (X-5).
  { Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000);
    fr(&s, 0, 0xF1, 0x10, 2000);
    fr(&s, 0, 0, 127, 5000); fr(&s, 0, 0, 127, 5100);
    simClear(&s);
    const uint8_t fm0 = s.env.fm_mode_runtime; const bool rtm0 = s.env.rtm_rx_active;
    fr(&s, 0, 0xF1, 0x3C, 5150); fr(&s, 0, 0xF1, 0x3C, 5250); fr(&s, 0, 0xF1, 0x3C, 5350);
    CHECK(s.st.owner == 0 && s.st.n.summons_refused == 0 && s.st.n.summons == 1);
    CHECK(!logHas(&s, "zero") && !logHas(&s, "rtm_off") && !logHas(&s, "fm_off"));      // nothing to release
    CHECK(s.env.fm_mode_runtime == fm0 && s.env.rtm_rx_active == rtm0 && s.replies.empty());
    fr(&s, 0, 0, 127, 5400); fr(&s, 0, 0xF1, 1, 5500);       // the new owner's own arm
    CHECK(s.env.rtm_rx_active); }
  testEnd("T22");
}

// ============================================================
// T23 - claimable, but never released
// ============================================================
static void t23()
{
  testBegin();
  for (int fsv = 0; fsv < 2; fsv++) {
    Sim s; simInit(&s, false, 0x03);
    if (fsv) s.cfg.failsafe_ms = 10000;
    const uint32_t thr_ms = fsv ? 11000 : 10000;
    grantBySqueeze(&s, 0, 1000);
    fr(&s, 0, 0xF2, 0x82, 1300); loopTick(&s);              // Follow-Me armed
    const uint32_t last = 1400; fr(&s, 0, 0, 127, last);    // the owner's last frame
    simClear(&s);
    const uint8_t seq0 = s.seq;
    for (uint32_t t = last + 100; t < last + thr_ms - 50; t += 100) fr(&s, 1, 0, 127, t);   // another remote present
    CHECK(claimable(&s, last + thr_ms - 1) == MTX_CL_NO);
    CHECK(claimable(&s, last + thr_ms) == MTX_CL_SILENT);
    fr(&s, 1, 0, 127, last + thr_ms);
    CHECK(lastStatus(&s) == MTX_S_FREE && countEv(&s, MTX_EV_CLAIMABLE) == 1);
    for (uint32_t t = last + thr_ms + 100; t < last + 100000; t += 100) fr(&s, 1, 0, 127, t);
    CHECK(s.st.owner == 0 && s.seq == seq0 && !logHas(&s, "zero") && s.env.fm_mode_runtime == 2);
    CHECK(countEv(&s, MTX_EV_RELEASED) == 0);
  }
  testEnd("T23");
}

// ============================================================
// T24 - the owner comes back before a claim
// ============================================================
static void t24()
{
  testBegin();
  for (int meta = 0; meta < 2; meta++) {
    Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000);
    fr(&s, 0, 0, 127, 1300);
    for (uint32_t t = 1400; t <= 13300; t += 100) fr(&s, 1, 0, 127, t);
    CHECK(s.st.claimable == MTX_CL_SILENT);
    simClear(&s);
    const uint32_t g0 = s.mtx_grant_ms; const uint8_t seq0 = s.seq;
    if (meta) fr(&s, 0, 0xF2, 0x02, 13800);                  // a keepalive alone ends the silence
    else      fr(&s, 0, 0, 127, 13800);
    CHECK(s.st.owner == 0 && s.st.claimable == MTX_CL_NO);
    CHECK(countEv(&s, MTX_EV_NOT_CLAIMABLE) == 1 && s.events.back().second == MTX_NC_HEARD);
    CHECK(!logHas(&s, "limit") && s.mtx_grant_ms == g0 && s.seq == seq0);
    CHECK(mtxTakeoverCap(13800, s.mtx_grant_ms, 64, 4000) == 255);   // no limit
    fr(&s, 1, 0, 127, 13900); CHECK(lastStatus(&s) == MTX_S_BUSY);
  }
  testEnd("T24");
}

// ============================================================
// T25 - the idle rule (with X-1 and X-2)
// ============================================================
static void ownerAtZeroWithOther(Sim* s, uint32_t from, uint32_t to)
{
  for (uint32_t t = from; t < to; t += 100) { fr(s, 0, 0, 127, t); fr(s, 1, 0, 127, t + 50); }
}
static void t25()
{
  testBegin();
  // Manual, owner at zero 10 s with another remote present -> IDLE.
  { Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000); fr(&s, 0, 0, 127, 1300);
    ownerAtZeroWithOther(&s, 1400, 11150);                   // the last input was the grant squeeze at 1200
    CHECK(claimable(&s, 11199) == MTX_CL_NO);
    CHECK(claimable(&s, 11200) == MTX_CL_IDLE);
    ownerAtZeroWithOther(&s, 11150, 11400);
    CHECK(claimable(&s, 11400) == MTX_CL_IDLE && s.st.claimable == MTX_CL_IDLE);
    // Silence is not time at zero: an owner heard again after 10 s of silence is not claimable on its first frame.
    { Sim q; simInit(&q, false, 0x03);
      grantBySqueeze(&q, 0, 1000); fr(&q, 0, 0, 127, 1300);
      for (uint32_t t = 1400; t <= 15000; t += 100) fr(&q, 1, 0, 127, t);
      CHECK(q.st.claimable == MTX_CL_SILENT);
      fr(&q, 0, 0, 127, 15050);
      CHECK(q.st.claimable == MTX_CL_NO && claimable(&q, 15050) == MTX_CL_NO);
      ownerAtZeroWithOther(&q, 15100, 25000);
      CHECK(claimable(&q, 25049) == MTX_CL_NO && claimable(&q, 25050) == MTX_CL_IDLE); }
    // The owner's own squeeze while claimable: its own throttle, no limit; claimable ends.
    simClear(&s);
    fr(&s, 0, 150, 127, 11500);
    CHECK(s.thr == 150 && !logHas(&s, "limit") && s.st.owner == 0 && s.st.claimable == MTX_CL_NO);
    CHECK(s.events.size() == 1 && s.events[0].first == MTX_EV_NOT_CLAIMABLE && s.events[0].second == MTX_NC_INPUT); }
  // Armed: never idle at 10, 60 or 100 s; silence still counts.
  for (int arm = 0; arm < 3; arm++) {
    Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000);
    if (arm == 0) { fr(&s, 0, 0xF2, 0x83, 1300); loopTick(&s); }
    if (arm == 1) fr(&s, 0, 0xF1, 1, 1300);
    if (arm == 2) s.env.fm_rx_active = true;
    ownerAtZeroWithOther(&s, 1400, 101400);
    CHECK(s.st.owner == 0 && countEv(&s, MTX_EV_CLAIMABLE) == 0);
    for (uint32_t t = 101400; t < 112000; t += 100) fr(&s, 1, 0, 127, t);   // now the owner goes silent
    CHECK(claimable(&s, 112000) == MTX_CL_SILENT);
  }
  // The predicate itself (not only the X-2 tracker): an armed owner is never IDLE, however old its idle base.
  { Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000); fr(&s, 0, 0, 127, 1300); fr(&s, 1, 0, 127, 30000);
    s.st.owner_heard_ms = 29950; s.st.owner_idle_base_ms = 1300; s.st.owner_armed_seen = false;
    MtxRxEnv e = s.env;
    CHECK(mtxClaimableReason(&s.st, &s.cfg, &e, 30000) == MTX_CL_IDLE);
    e.fm_mode_runtime = 1; CHECK(mtxClaimableReason(&s.st, &s.cfg, &e, 30000) == MTX_CL_NO);
    e = s.env; e.rtm_rx_active = true; CHECK(mtxClaimableReason(&s.st, &s.cfg, &e, 30000) == MTX_CL_NO);
    e = s.env; e.fm_rx_active = true;  CHECK(mtxClaimableReason(&s.st, &s.cfg, &e, 30000) == MTX_CL_NO);
    e = s.env; s.st.owner_armed_seen = true; CHECK(mtxClaimableReason(&s.st, &s.cfg, &e, 30000) == MTX_CL_NO); }
  // Input restarts the clock: a byte >= 8, or steering 24 off centre; 7 and 23 do not.
  for (int c = 0; c < 4; c++) {
    Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000); fr(&s, 0, 0, 127, 1300);
    ownerAtZeroWithOther(&s, 1400, 6000);
    const uint8_t b3 = (c == 0) ? 8 : (c == 1) ? 7 : 0;
    const uint8_t b4 = (c == 2) ? (uint8_t)(127 + 24) : (c == 3) ? (uint8_t)(127 - 23) : 127;
    fr(&s, 0, b3, b4, 6000);
    ownerAtZeroWithOther(&s, 6100, 11500);
    const bool restarted = (c == 0 || c == 2);
    CHECK((claimable(&s, 11500) == MTX_CL_NO) == restarted);
  }
  // X-2: armed for 60 s, disarmed at 60 s -> not claimable until 70 s.
  { Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000);
    fr(&s, 0, 0xF2, 0x82, 1300); loopTick(&s);
    ownerAtZeroWithOther(&s, 1400, 61300);
    fr(&s, 0, 0xF2, 0x00, 61300); loopTick(&s);              // the rider's own disarm at about 60 s
    ownerAtZeroWithOther(&s, 61400, 71300);
    CHECK(claimable(&s, 71250) == MTX_CL_NO);
    ownerAtZeroWithOther(&s, 71300, 71600);
    CHECK(claimable(&s, 71600) == MTX_CL_IDLE); }
  // Y-5 (a): X-2's DISARM-EDGE restart, with SPARSE evaluations. The dense script above passes without it, because
  // its last armed evaluation is 50 ms before the disarm. Here frames come once a second: the last armed evaluation
  // is at T, the loop disarms at T+900 with no frame, the next evaluation is at T+1000 - so the idle clock must start
  // at T+1000 (late by one evaluation), never at T (early). Not claimable at T+10999, IDLE at T+11000.
  { Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000); fr(&s, 0, 0, 127, 1300);
    s.env.fm_rx_active = true;                                 // Follow-Me active on the loop
    const uint32_t T = 30000;
    for (uint32_t t = 2000; t <= T; t += 1000) { fr(&s, 0, 0, 127, t); fr(&s, 1, 0, 127, t); }
    s.env.fm_rx_active = false;                                // T+900: the loop disarms, no frame
    for (uint32_t t = T + 1000; t <= T + 10000; t += 1000) { fr(&s, 0, 0, 127, t); fr(&s, 1, 0, 127, t); }
    CHECK(claimable(&s, T + 10999) == MTX_CL_NO);
    CHECK(s.st.owner_idle_base_ms == T + 1000);
    fr(&s, 0, 0, 127, T + 11000); fr(&s, 1, 0, 127, T + 11000);
    CHECK(claimable(&s, T + 11000) == MTX_CL_IDLE && s.st.claimable == MTX_CL_IDLE); }
  // X-1 option (a): during a held combo or the RTM ceremony the remote repeats 0xF1/0 every 3 s with throttle 0: never idle.
  { Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000); fr(&s, 0, 0, 127, 1300);
    uint32_t t = 1400;
    for (; t < 31400; t += 100) {
      if (((t - 1400) % 3000) == 0) fr(&s, 0, 0xF1, 0, t); else fr(&s, 0, 0, 127, t);
      fr(&s, 1, 0, 127, t + 50);
    }
    CHECK(countEv(&s, MTX_EV_CLAIMABLE) == 0);
    fr(&s, 0, 0xF1, 1, t);                                   // the ceremony ends: RTM on, armed from here
    CHECK(s.env.rtm_rx_active); }
  // X-1: 0xF2 with the fresh bit restarts the clock; a keepalive (no fresh bit), the boot ID, the refresh do not.
  CHECK(mtxOwnerActivity(0xF2, 0x81) && !mtxOwnerActivity(0xF2, 0x01) && !mtxOwnerActivity(0xF2, 0x00));
  CHECK(mtxOwnerActivity(0xF1, 0) && mtxOwnerActivity(0xF1, 1) && !mtxOwnerActivity(0xF1, 2) && !mtxOwnerActivity(0xF1, 0x85));
  CHECK(!mtxOwnerActivity(0xF1, 0x10) && !mtxOwnerActivity(0xF3, 0x03) && !mtxOwnerActivity(0xF4, 0x01));
  CHECK(mtxOwnerActivity(8, 127) && !mtxOwnerActivity(7, 127) && mtxOwnerActivity(0, 103) && !mtxOwnerActivity(0, 104));
  testEnd("T25");
}

// ============================================================
// T26 - the claim by squeeze and the claim token
// ============================================================
static void silentOwnerClaimable(Sim* s, uint32_t* t)
{
  grantBySqueeze(s, 0, 1000); fr(s, 0, 0, 127, 1300);
  for (*t = 1400; *t <= 11400; *t += 100) fr(s, 1, 0, 127, *t);
}
static void t26()
{
  testBegin();
  // fm_rx_active false -> the grant on the same packet.
  { Sim s; simInit(&s, false, 0x03); uint32_t t; silentOwnerClaimable(&s, &t);
    CHECK(s.st.claimable == MTX_CL_SILENT);
    fr(&s, 1, 0, 127, t); simClear(&s);
    fr(&s, 1, 90, 127, t + 100);
    CHECK(s.st.owner == 1 && lastStatus(&s) == MTX_S_OWNER && s.thr == 90);
    // The order: the release (zero first) ... the counter, then the grant (limit before the first byte).
    CHECK(logPos(&s, "zero") == 0 && logPos(&s, "zero") < logPos(&s, "rtm_off"));
    CHECK(logPos(&s, "seq") < logPos(&s, "limit") && logPos(&s, "limit") < logPos(&s, "apply"));
    CHECK(s.st.claim_notice[0] && s.st.n.claims == 1 && s.st.n.released_claim == 1);
    // Y-4: the claim line (ev6) comes after the release (zero ... seq) and after the grant (limit); nothing is
    // logged before the throttle zero.
    CHECK(logPos(&s, "ev6") > logPos(&s, "seq") && logPos(&s, "ev6") > logPos(&s, "limit")); }
  // fm_rx_active true -> a token; the grant on s's next control frame once Follow-Me idled, at any throttle.
  { Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000); fr(&s, 0, 0xF2, 0x81, 1300); loopTick(&s);
    for (uint32_t t = 1400; t <= 11400; t += 100) fr(&s, 1, 0, 127, t);
    fr(&s, 1, 0, 127, 11500); simClear(&s);
    fr(&s, 1, 90, 127, 11600);                                // the claim: FM still active on the loop
    CHECK(s.st.owner == kMtxNone && s.st.token.slot == 1 && lastStatus(&s) == MTX_S_FREE && s.thr == 0);
    CHECK(s.env.fm_mode_runtime == 0 && s.env.fm_rx_active);
    // A third remote cannot take the buggy while the token stands, and hears BUSY.
    Sim* p = &s; p->cfg.valid_mask = 0x07;
    fr(&s, 2, 0, 127, 11650); fr(&s, 2, 0, 127, 11700); fr(&s, 2, 80, 127, 11750);
    CHECK(s.st.owner == kMtxNone && lastStatus(&s) == MTX_S_BUSY);
    fr(&s, 1, 200, 127, 11760);                               // FM not idle yet: still waiting (Y-5 c: time runs forward)
    CHECK(s.st.owner == kMtxNone && lastStatus(&s) == MTX_S_FREE);
    loopTick(&s);                                             // the FM tick: fmEnterIdle()
    fr(&s, 1, 200, 127, 11800);
    CHECK(s.st.owner == 1 && s.thr == 200 && s.st.token.slot == kMtxNone);
    CHECK(mtxTakeoverCap(11800, s.mtx_grant_ms, 64, 4000) == 64); }   // the limit from the grant
  // The token lapses after 1000 ms (and across the wrap).
  for (int wrap = 0; wrap < 2; wrap++) {
    Sim s; simInit(&s, false, 0x03);
    const uint32_t b = wrap ? 0xFFFFD000u : 0;
    grantBySqueeze(&s, 0, b + 1000); fr(&s, 0, 0xF2, 0x81, b + 1300); loopTick(&s);
    for (uint32_t t = b + 1400; t != b + 11500; t += 100) fr(&s, 1, 0, 127, t);
    fr(&s, 1, 90, 127, b + 11500);
    CHECK(s.st.token.slot == 1);
    wake(&s, b + 12500); CHECK(s.st.token.slot == 1);         // 1000 ms: still held
    wake(&s, b + 12501); CHECK(s.st.token.slot == kMtxNone && s.st.n.tokens_lapsed == 1);
    loopTick(&s);
    fr(&s, 1, 200, 127, b + 12600);                           // R1 from now on: needs zero proof again
    CHECK(s.st.owner == kMtxNone && lastStatus(&s) == MTX_S_FREE);
  }
  // A held trigger with no zero proof on a claimable owner: FREE, never a claim, never E9. (While the owner is not yet
  // claimable the same held trigger is REFUSED, E9.)
  { Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000); fr(&s, 0, 50, 127, 1300);    // the owner switched off mid-throttle at 1300
    for (uint32_t t = 1400; t < 11300; t += 100) { fr(&s, 1, 120, 127, t); CHECK(lastStatus(&s) == MTX_S_REFUSED); }
    for (uint32_t t = 11300; t < 20000; t += 100) { fr(&s, 1, 120, 127, t); CHECK(lastStatus(&s) == MTX_S_FREE); }
    CHECK(s.st.owner == 0 && s.st.n.claims == 0); }
  // Y-5 (b): R14's OWN zero-proof freshness and its motor-output gate. With two remotes the freshness is implied by
  // "present" (a claimer silent 3.1 s is not present, so the owner is not claimable); a THIRD remote keeps "present"
  // here, so the only thing that can refuse the claim is the claimer's own proof or the motor output.
  //   c = 0: proof 3000 ms old -> claim;  c = 1: proof 3100 ms old -> no claim;  c = 2: motor output off -> no claim.
  for (int c = 0; c < 3; c++) {
    Sim s; simInit(&s, false, 0x07);
    grantBySqueeze(&s, 0, 1000); fr(&s, 0, 0, 127, 1300);    // the owner goes silent at 1300
    for (uint32_t t = 1400; t <= 11600; t += 100) fr(&s, 2, 0, 127, t);   // slot 2 keeps "present"
    CHECK(s.st.claimable == MTX_CL_SILENT);
    fr(&s, 1, 0, 127, 11650); fr(&s, 1, 0, 127, 11700);      // slot 1: zero proof, the last zero at 11700
    const uint32_t sq = 11700 + ((c == 1) ? 3100u : 3000u);
    for (uint32_t t = 11800; t < sq; t += 100) fr(&s, 2, 0, 127, t);
    if (c == 2) s.env.pwm_active = false;
    fr(&s, 1, 90, 127, sq);
    CHECK((s.st.n.claims == 1) == (c == 0));
    if (c == 0) CHECK(s.st.owner == 1 && s.thr == 90);
    else        CHECK(s.st.owner == 0 && s.st.token.slot == kMtxNone && !logHas(&s, "zero") && lastStatus(&s) == MTX_S_FREE);
  }
  // X-10 (c), the claim: "subtract-only whatever the order". The owner's RTM return was running when it fell silent,
  // so the claim's synthetic "RTM off" arms the sticky return cap (value 40), and the claimer is granted on the SAME
  // packet at 90. A motor pass may run between ANY two steps of that frame. At every such point: every pass is
  // subtract-only (out <= the byte it read); the claimer's 90 is never seen without the takeover limit (and so never
  // passes above 64 in its first second); the cap stays at most until the claimer's first pass below 25.
  { int points = 0, raced = 0;
    for (int k = 0; k < 60; k++) {
      Sim q; simInit(&q, false, 0x03);
      grantBySqueeze(&q, 0, 1000); fr(&q, 0, 0xF1, 1, 1300);    // a return running
      fr(&q, 0, 120, 127, 1400);                                // the owner's last frame: trigger held, then silent
      for (uint32_t t = 1500; t <= 11500; t += 100) fr(&q, 1, 0, 127, t);
      CHECK(q.st.claimable == MTX_CL_SILENT);
      simClear(&q); q.pwm_after = k;
      fr(&q, 1, 90, 127, 11600);                                // the claim and the same-packet grant
      CHECK(q.st.owner == 1 && q.thr == 90);
      const bool inserted = !q.pwm_in.empty();
      q.pwm_after = -1; pwmPass(&q);                            // the next 10 ms pass
      if (q.sticky_cap) raced++;                                // no pass fell between the zero and the grant
      for (size_t i = 0; i < q.pwm_in.size(); i++) {
        CHECK(q.pwm_out[i] <= q.pwm_in[i]);
        if (q.pwm_in[i] == 90) CHECK(q.pwm_grant[i] == 1 && q.pwm_out[i] <= 64);
      }
      fr(&q, 1, 10, 127, 11700); pwmPass(&q);                   // the claimer below 25 once
      CHECK(!q.sticky_cap && q.pwm_out.back() <= 10);
      if (!inserted) break;
      points++;
    }
    CHECK(points >= 10 && raced >= 1); }                        // the race exists, and it stays subtract-only
  // ... and the predicate itself, on a hand-built state with no mtxTimers() between (Y-3's expiry would otherwise
  // clear a proof older than 3 s first, so this pins mtxClaimOk()'s own 3000 ms rule and its PWM term directly).
  { Sim s; simInit(&s, false, 0x07);
    s.st.owner = 0; s.st.owner_heard_ms = 1000; s.st.owner_idle_base_ms = 1000; s.st.owner_last_thr = 50;
    s.st.ctrl_heard_ms[2] = 13900;                           // slot 2 present
    s.st.zero_streak[1] = 2; s.st.last_zero_ms[1] = 11000;
    MtxRxEnv e = s.env;
    CHECK(mtxClaimableReason(&s.st, &s.cfg, &e, 14000) == MTX_CL_SILENT);
    CHECK(mtxClaimOk(&s.st, &s.cfg, &e, 1, 90, 14000));     // 3000 ms old: accepted
    CHECK(!mtxClaimOk(&s.st, &s.cfg, &e, 1, 90, 14001));    // 3001 ms: refused
    e.pwm_active = false;
    CHECK(!mtxClaimOk(&s.st, &s.cfg, &e, 1, 90, 14000));    // motor output not running: refused
    e = s.env; s.st.zero_streak[1] = 1;
    CHECK(!mtxClaimOk(&s.st, &s.cfg, &e, 1, 90, 14000)); }  // one zero frame only: refused
  testEnd("T26");
}

// ============================================================
// T27 - the summon gates on the buggy (+ X-5)
// ============================================================
static void t27()
{
  testBegin();
  // From a claimable owner: accepted, no reply; the burst's frames 2-3 are no-ops, nothing counted refused.
  { Sim s; simInit(&s, false, 0x03); uint32_t t; silentOwnerClaimable(&s, &t);
    simClear(&s);
    fr(&s, 1, 0xF1, 0x3C, t); fr(&s, 1, 0xF1, 0x3C, t + 100); fr(&s, 1, 0xF1, 0x3C, t + 200);
    CHECK(s.st.owner == 1 && s.replies.empty() && s.st.n.summons == 1 && s.st.n.summons_refused == 0);
    CHECK(countEv(&s, MTX_EV_TAKES_CONTROL) == 1);
    // Y-4: the throttle zero is the first thing logged; the summon line (ev9) comes after the release (seq), the
    // grant (limit) and the claim line (ev6).
    CHECK(logPos(&s, "zero") == 0 && logPos(&s, "ev9") > logPos(&s, "seq") && logPos(&s, "ev9") > logPos(&s, "limit"));
    CHECK(logPos(&s, "ev9") > logPos(&s, "ev6")); }
  // From a token holder: frames 2-3 never restamp the token (X-5).
  { Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000); fr(&s, 0, 0xF2, 0x81, 1300); loopTick(&s);
    for (uint32_t t = 1400; t <= 11400; t += 100) fr(&s, 1, 0, 127, t);
    fr(&s, 1, 0xF1, 0x3C, 11450);
    CHECK(s.st.token.slot == 1 && s.st.token.ms == 11450);
    fr(&s, 1, 0xF1, 0x3C, 11550); fr(&s, 1, 0xF1, 0x3C, 11650);
    CHECK(s.st.token.ms == 11450 && s.st.n.summons == 1 && s.st.n.summons_refused == 0);
    // The guard inside mtxClaim() itself (no path reaches it today): a second claim by the holder never restamps.
    mtxClaim(&s.st, &s.h, 1, 11700);
    CHECK(s.st.token.slot == 1 && s.st.token.ms == 11450); }
  // Zero proof 1100 ms old -> refused; 1000 ms -> accepted.
  for (int c = 0; c < 2; c++) {
    Sim s; simInit(&s, false, 0x03); uint32_t t; silentOwnerClaimable(&s, &t);
    const uint32_t last_zero = t - 100;
    fr(&s, 1, 0xF1, 0x3C, last_zero + (c ? 1000 : 1100));
    CHECK((s.st.owner == 1) == (c == 1));
    CHECK(s.st.n.summons_refused == (c ? 0 : 1));
  }
  // A token held by another slot -> refused.
  { Sim s; simInit(&s, false, 0x07);
    grantBySqueeze(&s, 0, 1000); fr(&s, 0, 0xF2, 0x81, 1300); loopTick(&s);
    for (uint32_t t = 1400; t <= 11400; t += 100) { fr(&s, 1, 0, 127, t); fr(&s, 2, 0, 127, t + 10); }
    fr(&s, 1, 90, 127, 11450);                                 // slot 1 claims: a token
    fr(&s, 2, 0xF1, 0x3C, 11460);
    CHECK(s.st.token.slot == 1 && s.st.n.summons_refused == 1); }
  // From nobody: the token or the grant, no release, no notice.
  { Sim s; simInit(&s, false, 0x03);
    fr(&s, 1, 0, 127, 1000); fr(&s, 1, 0, 127, 1100); simClear(&s);
    fr(&s, 1, 0xF1, 0x3C, 1200);
    CHECK(s.st.owner == 1 && !logHas(&s, "zero") && !s.st.claim_notice[0] && !s.st.claim_notice[1]); }
  // No PWM, or SINGLE -> nothing.
  { Sim s; simInit(&s, false, 0x03); s.env.pwm_active = false;
    fr(&s, 1, 0, 127, 1000); fr(&s, 1, 0, 127, 1100); fr(&s, 1, 0xF1, 0x3C, 1200);
    CHECK(s.st.owner == kMtxNone && s.st.n.summons_refused == 1); }
  testEnd("T27");
}

// ============================================================
// T28 - a summon against an owner who is commanding is refused
// ============================================================
static void t28()
{
  testBegin();
  for (int c = 0; c < 2; c++) {                                // FM armed at zero, or driving at 40 %
    Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000);
    if (c == 0) { fr(&s, 0, 0xF2, 0x81, 1300); loopTick(&s); }
    const uint8_t drive = c ? 100 : 0;
    simClear(&s);
    for (uint32_t t = 1400; t < 101400; t += 100) {
      fr(&s, 0, drive, 127, t); fr(&s, 1, 0, 127, t + 30);
      if (((t - 1400) % 5000) == 0) fr(&s, 1, 0xF1, 0x3C, t + 60);
    }
    CHECK(s.st.owner == 0 && !logHas(&s, "zero") && s.st.n.summons_refused == 20 && s.st.n.summons == 0);
  }
  // Silent 9.9 s -> refused; 10.0 s -> accepted. (The owner's last frame carries throttle, so only silence counts.)
  for (int c = 0; c < 2; c++) {
    Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000); fr(&s, 0, 50, 127, 1300);
    uint32_t t = 1400; for (; t < 11200; t += 100) fr(&s, 1, 0, 127, t);
    fr(&s, 1, 0xF1, 0x3C, c ? 11300 : 11200);
    CHECK((s.st.owner == 1) == (c == 1));
  }
  testEnd("T28");
}

// ============================================================
// T29 - the remote's summon gate; a remote without GPS
// ============================================================
static void t29()
{
  testBegin();
  MtxTxStatus t; mtxTxStatusInit(&t);
  MtxSummonIn in; in.gps_en = true; in.loc_valid = true; in.loc_age_ms = 200; in.gps_stale_ms = 3000;
  in.trigger_released = true; in.feature_enabled = true; in.locked = false; in.in_setup = false;
  CHECK(mtxSummonVerdict(&t, &in, 1000) == MTX_SV_NOT_FREE);                  // never a reply
  for (int st = 0; st < 8; st++) {
    mtxTxStatusInit(&t); mtxTxOnReply(&t, (uint8_t)st, 1000);
    CHECK((mtxSummonVerdict(&t, &in, 1100) == MTX_SV_OK) == (st == MTX_S_FREE));
  }
  mtxTxStatusInit(&t); mtxTxOnReply(&t, MTX_S_FREE, 1000);
  CHECK(mtxSummonVerdict(&t, &in, 6000) == MTX_SV_OK && mtxSummonVerdict(&t, &in, 6001) == MTX_SV_NOT_FREE);
  MtxSummonIn x = in; x.gps_en = false;        CHECK(mtxSummonVerdict(&t, &x, 1100) == MTX_SV_NO_FIX);
  x = in; x.loc_valid = false;                 CHECK(mtxSummonVerdict(&t, &x, 1100) == MTX_SV_NO_FIX);   // a module, no fix
  x = in; x.loc_age_ms = 3000;                 CHECK(mtxSummonVerdict(&t, &x, 1100) == MTX_SV_NO_FIX);   // stale
  x = in; x.loc_age_ms = 2999;                 CHECK(mtxSummonVerdict(&t, &x, 1100) == MTX_SV_OK);
  x = in; x.loc_age_ms = 0xFFFFFFFFu;          CHECK(mtxSummonVerdict(&t, &x, 1100) == MTX_SV_NO_FIX);   // TinyGPS "never"
  x = in; x.trigger_released = false;          CHECK(mtxSummonVerdict(&t, &x, 1100) == MTX_SV_TRIGGER);
  x = in; x.feature_enabled = false;           CHECK(mtxSummonVerdict(&t, &x, 1100) == MTX_SV_DISABLED);
  x = in; x.locked = true;                     CHECK(mtxSummonVerdict(&t, &x, 1100) == MTX_SV_LOCKED);
  x = in; x.in_setup = true;                   CHECK(mtxSummonVerdict(&t, &x, 1100) == MTX_SV_SETUP);
  // The fix test is the stream gate itself.
  for (int i = 0; i < 2000; i++) {
    const bool en = rnd() & 1, v = rnd() & 1; const uint32_t age = rnd() % 6000; const uint16_t stale = (uint16_t)(rnd() % 6000);
    CHECK(mtxTxFixFresh(en, v, age, stale) == (en && v && age < stale));
  }
  // The remote's action: build and queue only on OK; on every refusal the builder is never called.
  MtxLinkRec v3; v3.proto = kMtxProtoV3; memcpy(v3.dest_address, kR1, 3); memcpy(v3.efuse_id, kB, 3);
  int built = 0;
  for (int c = 0; c < 7; c++) {
    MtxSummonIn y = in;
    if (c == 1) y.gps_en = false;
    if (c == 2) y.loc_valid = false;
    if (c == 3) y.trigger_released = false;
    if (c == 4) y.feature_enabled = false;
    if (c == 5) y.locked = true;
    if (c == 6) y.in_setup = true;
    uint8_t f[6];
    if (mtxSummonVerdict(&t, &y, 1100) == MTX_SV_OK && mtxBuildUplinkFromQueue(kMtxQueueSummon, 0, &v3, f, crc8)) built++;
  }
  CHECK(built == 1);
  // Then: an owner status within 1500 ms -> the arm runs; none -> "St".
  mtxTxStatusInit(&t); mtxTxOnReply(&t, MTX_S_FREE, 900);
  CHECK(mtxTxOwnerWait(&t, 1000, 1300, kMtxSummonWaitMs) == MTX_W_WAIT);
  mtxTxOnReply(&t, MTX_S_OWNER, 1350);
  CHECK(mtxTxOwnerWait(&t, 1000, 1360, kMtxSummonWaitMs) == MTX_W_OWNER);
  mtxTxStatusInit(&t); mtxTxOnReply(&t, MTX_S_FREE, 900); mtxTxOnReply(&t, MTX_S_FREE, 1900);
  CHECK(mtxTxOwnerWait(&t, 1000, 2500, kMtxSummonWaitMs) == MTX_W_WAIT);      // 1500 ms is still inside
  CHECK(mtxTxOwnerWait(&t, 1000, 2501, kMtxSummonWaitMs) == MTX_W_TIMEOUT);
  // A late owner status (after the wait) is not taken as the answer to this summon.
  mtxTxOnReply(&t, MTX_S_OWNER, 2600);
  CHECK(mtxTxOwnerWait(&t, 1000, 2600, kMtxSummonWaitMs) == MTX_W_TIMEOUT);
  // V2.5-Evo - 2026-10-21 - P3-1: a toggle summon whose button is held past the combo. Granted at send + 0.4 s, polled
  // at send + 3 s: the arm still runs (it used to read TIMEOUT: "St" while in control).
  mtxTxStatusInit(&t); mtxTxOnReply(&t, MTX_S_FREE, 900);
  for (uint32_t a = 1400; a <= 3900; a += 100) mtxTxOnReply(&t, MTX_S_OWNER, a);
  CHECK(mtxTxOwnerWait(&t, 1000, 4000, kMtxSummonWaitMs) == MTX_W_OWNER);
  // No owner status: TIMEOUT at send + 3 s.
  mtxTxStatusInit(&t); for (uint32_t a = 900; a <= 3900; a += 100) mtxTxOnReply(&t, MTX_S_FREE, a);
  CHECK(mtxTxOwnerWait(&t, 1000, 4000, kMtxSummonWaitMs) == MTX_W_TIMEOUT);
  // The late grant stays a late grant however late the poll: first owner at send + 1.6 s, polled at send + 3 s.
  mtxTxStatusInit(&t); mtxTxOnReply(&t, MTX_S_FREE, 900);
  for (uint32_t a = 2600; a <= 3900; a += 100) mtxTxOnReply(&t, MTX_S_OWNER, a);
  CHECK(mtxTxOwnerWait(&t, 1000, 4000, kMtxSummonWaitMs) == MTX_W_TIMEOUT);
  testEnd("T29");
}

// ============================================================
// T30 - rider GPS after a claim
// ============================================================
static void t30()
{
  testBegin();
  Sim s; simInit(&s, false, 0x03);
  grantBySqueeze(&s, 0, 1000);
  s.gps_ts = 1250; s.gps_lat = 26123456; s.phb_ok = true; s.phb_last = 1250;   // the old owner's fix
  fr(&s, 0, 0, 127, 1300);
  for (uint32_t t = 1400; t <= 11400; t += 100) fr(&s, 1, 0, 127, t);
  fr(&s, 1, 90, 127, 11500);                                  // claimed
  CHECK(s.st.owner == 1);
  // The fix's stamp is cleared; the coordinates are not (every reader gates on the stamp).
  CHECK(s.gps_ts == 0 && s.gps_lat == 26123456);
  CHECK(stampStale(11500, s.gps_ts, 3000));                   // an RTM Gate 4 model fails until the new owner's fix
  CHECK(!s.phb_ok && s.phb_last == 0);                        // Phase B runs on the new owner's first GPS packet
  const bool phase_b_due_now = (s.phb_last == 0);
  CHECK(phase_b_due_now);
  // GPS data: only from the owner whose announce switched to 15 bytes, with that slot's address.
  uint8_t a0[3], a1[3]; mtxIdXor(kA, kR1, a0); mtxIdXor(kB, kR1, a1);
  CHECK(!mtxGpsDataOk(&s.st, &s.cfg, 0, a0, a0));             // the old owner: rejected
  CHECK(mtxGpsDataOk(&s.st, &s.cfg, 1, a1, a1));              // the new owner
  CHECK(!mtxGpsDataOk(&s.st, &s.cfg, 1, a0, a1));             // wrong address
  // The old owner's announce never switches the buggy to 15 bytes.
  fr(&s, 0, 0xF3, 0x03, 11600);
  CHECK(!s.gps_pending && countEv(&s, MTX_EV_NON_OWNER_META) == 1);
  fr(&s, 1, 0xF3, 0x03, 11700);
  CHECK(s.gps_pending && s.gps_len == 15);
  // SINGLE: the address test alone (today).
  MtxRxCfg c1 = s.cfg; c1.single = true;
  CHECK(mtxGpsDataOk(&s.st, &c1, kMtxNone, a0, a0));
  // Phase B distance: a model of today's check with the maximum pairing distance.
  const float max_m = 500.0f; CHECK(!(510.0f <= max_m)); CHECK(490.0f <= max_m);
  testEnd("T30");
}

// ============================================================
// T31 - status priority; the claim notice; X-6
// ============================================================
static void t31()
{
  testBegin();
  // The old owner's first frame after a silent claim: SEIZED, the hold restarted from THAT frame.
  { Sim s; simInit(&s, false, 0x03); uint32_t t; silentOwnerClaimable(&s, &t);
    fr(&s, 1, 0, 127, t); fr(&s, 1, 90, 127, t + 100);
    CHECK(s.st.owner == 1 && s.st.claim_notice[0]);
    fr(&s, 0, 0, 127, t + 20000);                             // back 20 s later
    CHECK(lastStatus(&s) == MTX_S_SEIZED && s.st.seized_until_ms[0] == t + 20000 + 5000);
    fr(&s, 0, 0, 127, t + 24999); CHECK(lastStatus(&s) == MTX_S_SEIZED);
    fr(&s, 0, 0, 127, t + 25000); CHECK(lastStatus(&s) != MTX_S_SEIZED);
    // Two SEIZED in a row on the remote -> "tA".
    MtxTxStatus x; mtxTxStatusInit(&x); mtxTxOnReply(&x, MTX_S_OWNER, 0);
    CHECK(!(mtxTxOnReply(&x, MTX_S_SEIZED, 1) & MTX_TXE_SEIZED_NOTICE));
    CHECK(mtxTxOnReply(&x, MTX_S_SEIZED, 2) & MTX_TXE_SEIZED_NOTICE); }
  // The priority, rule by rule.
  { Sim s; simInit(&s, false, 0x07); MtxRxEnv e = s.env;
    s.st.owner = 1; s.st.owner_heard_ms = 1000; s.st.owner_idle_base_ms = 1000; s.st.owner_last_thr = 50;
    s.st.claim_notice[2] = true; s.st.token.slot = 0; s.st.token.ms = 1000;
    CHECK(mtxNonOwnerStatus(&s.st, &s.cfg, &e, 2, true, 1000) == MTX_S_SEIZED);   // SEIZED > BUSY (token)
    s.st.seized_on[2] = false; s.st.owner = kMtxNone;
    CHECK(mtxNonOwnerStatus(&s.st, &s.cfg, &e, 2, true, 1000) == MTX_S_BUSY);     // BUSY (token) > FREE
    s.st.token.slot = kMtxNone;
    CHECK(mtxNonOwnerStatus(&s.st, &s.cfg, &e, 2, true, 1000) == MTX_S_FREE);     // FREE > REFUSED
    s.st.owner = 1;
    CHECK(mtxNonOwnerStatus(&s.st, &s.cfg, &e, 2, true, 1000) == MTX_S_REFUSED);  // REFUSED > SEIZABLE
    CHECK(mtxNonOwnerStatus(&s.st, &s.cfg, &e, 0, true, 1000) == MTX_S_REFUSED);
    CHECK(mtxNonOwnerStatus(&s.st, &s.cfg, &e, 0, false, 1000) == MTX_S_SEIZABLE);
    CHECK(mtxNonOwnerStatus(&s.st, &s.cfg, &e, 2, false, 1000) == MTX_S_BUSY);
    e.fm_mode_runtime = 3; CHECK(mtxNonOwnerStatus(&s.st, &s.cfg, &e, 0, false, 1000) == MTX_S_BUSY);   // armed: no SEIZABLE
    e.fm_mode_runtime = 0xFF; s.cfg.seize_en = false; CHECK(mtxNonOwnerStatus(&s.st, &s.cfg, &e, 0, false, 1000) == MTX_S_BUSY);
    s.cfg.seize_en = true; s.st.owner = 0; CHECK(mtxNonOwnerStatus(&s.st, &s.cfg, &e, 0, false, 1000) == MTX_S_BUSY); }
  // X-6: during a token, a LOCKED remote that is not the holder hears BUSY, never FREE; the holder's lock ends the token.
  { Sim s; simInit(&s, false, 0x07);
    grantBySqueeze(&s, 0, 1000); fr(&s, 0, 0xF2, 0x81, 1300); loopTick(&s);
    for (uint32_t t = 1400; t <= 11400; t += 100) fr(&s, 1, 0, 127, t);
    fr(&s, 1, 90, 127, 11450);
    CHECK(s.st.token.slot == 1);
    fr(&s, 2, 0xF1, 0x10, 11500); CHECK(lastStatus(&s) == MTX_S_BUSY);
    fr(&s, 0, 0xF1, 0x10, 11550); CHECK(lastStatus(&s) == MTX_S_SEIZED);   // the returning old owner locks first: still told
    fr(&s, 1, 0xF1, 0x10, 11600);
    CHECK(s.st.token.slot == kMtxNone && lastStatus(&s) != MTX_S_OWNER); }
  testEnd("T31");
}

// ============================================================
// T32 - SUMMON on the wire; the meta queue
// ============================================================
// A copy of the remote's queueMetaPacketBurst() rules 1-3 (TX Radio.ino), with mtxMetaTypePriority().
struct Q { uint8_t ht, hv, hc, nt, nv, nc; };
static void qBurst(Q* q, uint8_t type, uint8_t value)
{
  if (q->hc > 0 && q->ht == type)      { q->hv = value; q->hc = 3; }
  else if (q->nc > 0 && q->nt == type) { q->nv = value; q->nc = 3; }
  else if (q->hc == 0)                 { q->ht = type; q->hv = value; q->hc = 3; }
  else if (q->nc == 0 || mtxMetaTypePriority(type) > mtxMetaTypePriority(q->nt)) { q->nt = type; q->nv = value; q->nc = 3; }
}
static void t32()
{
  testBegin();
  CHECK(mtxMetaTypePriority(kMtxQueueSummon) == 4 && mtxMetaTypePriority(kMtxQueueSeize) == 4);
  CHECK(mtxMetaTypePriority(0xF1) == 3 && mtxMetaTypePriority(0xF2) == 2 && mtxMetaTypePriority(0xF4) == 1);
  CHECK(kMtxQueueSummon != 0xF1 && kMtxQueueSeize != 0xF1 && kMtxQueueSummon != kMtxQueueSeize);
  CHECK(kMtxQueueSummon < kMtxZeroThr && kMtxQueueSeize < kMtxZeroThr);   // a leak would be a zero-throttle byte
  // Never merged with a queued RTM 0xF1 (rule 1 is "same type"), in either order.
  Q q; memset(&q, 0, sizeof(q));
  qBurst(&q, 0xF1, 1); qBurst(&q, kMtxQueueSummon, 0);
  CHECK(q.ht == 0xF1 && q.hv == 1 && q.nt == kMtxQueueSummon);
  memset(&q, 0, sizeof(q));
  qBurst(&q, kMtxQueueSummon, 0); qBurst(&q, 0xF1, 0);
  CHECK(q.ht == kMtxQueueSummon && q.nt == 0xF1 && q.nv == 0);
  // Priority 4 takes the second slot from a 0xF2.
  memset(&q, 0, sizeof(q));
  qBurst(&q, 0xF1, 0); qBurst(&q, 0xF2, 0x81); qBurst(&q, kMtxQueueSummon, 0);
  CHECK(q.nt == kMtxQueueSummon);
  // The builder rows and the decode (T3, T15).
  MtxLinkRec v3; v3.proto = kMtxProtoV3; memcpy(v3.dest_address, kR1, 3); memcpy(v3.efuse_id, kB, 3);
  uint8_t f[6], L[3]; mtxIdXor(kB, kR1, L);
  CHECK(mtxBuildUplinkFromQueue(kMtxQueueSummon, 0, &v3, f, crc8));
  CHECK(memcmp(f, L, 3) == 0 && f[3] == 0xF1 && f[4] == 0x3C && f[5] == crc8(f, 5) && mtxKind(f[3], f[4]) == MTX_K_SUMMON);
  MtxLinkRec leg = v3; leg.proto = 0;
  CHECK(!mtxBuildUplinkFromQueue(kMtxQueueSummon, 0, &leg, f, crc8));
  // An L-addressed SUMMON on the SW36 classifier model: never accepted. On the v3 buggy: the summoner's slot.
  CHECK(mtxBuildUplinkFromQueue(kMtxQueueSummon, 0, &v3, f, crc8) && !sw36Accepts(kR1, f));
  const uint8_t* ids[2] = { kA, kB }; MtxRxBinding b = bindingV3(kR1, ids, 2);
  CHECK(mtxLookupSlot(&b, kR1, f) == 1);
  testEnd("T32");
}

// ============================================================
// T33 - claim thresholds, the wrap, the clamp
// ============================================================
static void t33()
{
  testBegin();
  for (uint16_t cs = 6; cs <= 60; cs++) for (uint16_t fs = 100; fs <= 10000; fs += 100) {
    const uint32_t want = ((uint32_t)cs * 1000u > (uint32_t)fs + 1000u) ? (uint32_t)cs * 1000u : (uint32_t)fs + 1000u;
    CHECK(mtxClaimMs(cs, fs) == want);
    CHECK(mtxClaimMs(cs, fs) > fs);                                // never while the old gate could be open
  }
  CHECK(mtxClaimMs(10, 1000) == 10000 && mtxClaimMs(10, 10000) == 11000 && mtxClaimMs(6, 9000) == 10000);
  // Across the wrap: silence, idle and the token.
  { Sim s; simInit(&s, false, 0x03);
    const uint32_t b = 0xFFFFE000u;                               // about 8 s before the wrap
    grantBySqueeze(&s, 0, b); fr(&s, 0, 50, 127, b + 300);        // switched off mid-throttle: silence only
    for (uint32_t t = b + 400; t != b + 10300; t += 100) fr(&s, 1, 0, 127, t);
    CHECK(claimable(&s, b + 10299) == MTX_CL_NO);
    CHECK(claimable(&s, b + 10300) == MTX_CL_SILENT); }
  { Sim s; simInit(&s, false, 0x03);
    const uint32_t b = 0xFFFFE000u;
    grantBySqueeze(&s, 0, b); fr(&s, 0, 0, 127, b + 300);
    for (uint32_t t = b + 400; t != b + 10500; t += 100) { fr(&s, 0, 0, 127, t); fr(&s, 1, 0, 127, t + 50); }
    CHECK(claimable(&s, b + 10500) == MTX_CL_IDLE); }
  // A stamp written after the caller read the clock reads as "just now", never 49 days.
  CHECK(mtxAgeMs(1000, 1005) == 0 && mtxAgeMs(5, 0xFFFFFFF0u) == 21);
  // The validator clamp.
  uint16_t cs, se, cap, ms;
  const uint16_t in_cs[6] = { 0, 5, 6, 60, 61, 65535 }, out_cs[6] = { 6, 6, 6, 60, 60, 60 };
  for (int i = 0; i < 6; i++) { cs = in_cs[i]; se = 1; cap = 64; ms = 4000; mtxClampRxConfig(&cs, &se, &cap, &ms); CHECK(cs == out_cs[i]); }
  cs = 10; se = 7; cap = 300; ms = 20000; mtxClampRxConfig(&cs, &se, &cap, &ms); CHECK(se == 1 && cap == 255 && ms == 10000);
  uint16_t al = 3, hold = 900, mal = 10; mtxClampTxConfig(&al, &hold, &mal); CHECK(al == 5 && hold == 1000 && mal == 30);
  al = 0; hold = 2000; mal = 0; mtxClampTxConfig(&al, &hold, &mal); CHECK(al == 0 && hold == 1800 && mal == 0);
  al = 999; mal = 999; mtxClampTxConfig(&al, &hold, &mal); CHECK(al == 180 && mal == 600);
  // Y-3: the per-slot stamps expire, so none can alias as "just now" after 2^31 ms (24.8 days) of uptime.
  // (1) Zero proof: a remote sends two zeros and falls silent; the radio task wakes every 2 s for 2^31 ms; then it
  //     squeezes. Before the expiry the 2^31-ms-old proof read as age 0 and this squeeze took the buggy.
  { Sim s; simInit(&s, false, 0x03);
    fr(&s, 1, 0, 127, 1000); fr(&s, 1, 0, 127, 1100);
    wake(&s, 4100);
    CHECK(s.st.zero_streak[1] == 2 && s.st.last_zero_ms[1] == 1100);        // 3000 ms: kept
    wake(&s, 4101);
    CHECK(s.st.zero_streak[1] == 0 && s.st.last_zero_ms[1] == 0 && s.st.ctrl_heard_ms[1] == 0);
    const uint32_t T = 1100u + 0x80000000u + 200u;
    for (uint32_t t = 6000; t < T; t += 2000) wake(&s, t);
    fr(&s, 1, 60, 127, T);
    CHECK(s.st.owner == kMtxNone && lastStatus(&s) == MTX_S_FREE && s.thr == 0); }
  // (2) "Present": a remote heard once is not "on" 3 s later, and never comes back as "on" at the aliasing age.
  { Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000);
    fr(&s, 1, 0, 127, 1300);
    CHECK(mtxPresent(&s.st, &s.cfg, 4300));
    wake(&s, 4301);
    CHECK(s.st.ctrl_heard_ms[1] == 0 && !mtxPresent(&s.st, &s.cfg, 4301));
    CHECK(!mtxPresent(&s.st, &s.cfg, 1300u + 0x80000000u + 500u)); }
  // Y-7 (c): ANY frame from the owner restarts its silence clock - its own SUMMON or SEIZE frame too (today's
  // remote never sends either while it owns). The owner's last control frame carries throttle, so only silence counts.
  for (int kind = 0; kind < 2; kind++) {
    Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000); fr(&s, 0, 50, 127, 1300);
    for (uint32_t t = 1400; t < 11200; t += 100) fr(&s, 1, 0, 127, t);
    fr(&s, 0, 0xF1, kind ? 0x5A : 0x3C, 11200);             // the owner's own SEIZE / SUMMON: refused, but heard
    fr(&s, 1, 0, 127, 11300);
    CHECK(s.st.owner_heard_ms == 11200 && claimable(&s, 11300) == MTX_CL_NO && lastStatus(&s) == MTX_S_BUSY);
    CHECK(s.st.owner == 0 && !logHas(&s, "zero"));
  }
  testEnd("T33");
}

// ============================================================
// T34 - the remote's activity repeat (X-1 option (a), Y-6; owner 2026-10-20: busy on any button / toggle press)
// ============================================================
static MtxTxActIn actIn()
{
  MtxTxActIn in; memset(&in, 0, sizeof(in));
  in.v3 = true; in.owner_multi = true;   // in control, others paired, nothing else in the way
  return in;
}
static void t34()
{
  testBegin();
  // A closed window is never due; the first press makes one due AT ONCE.
  { MtxTxActivity a; mtxTxActivityInit(&a); MtxTxActIn in = actIn();
    CHECK(!mtxTxActivityDue(&a, &in, 1000));
    mtxTxActivityNote(&a, 1000);
    CHECK(mtxTxActivityDue(&a, &in, 1000)); mtxTxActivitySent(&a, 1000);
    CHECK(!mtxTxActivityDue(&a, &in, 2999));                             // the period
    CHECK(mtxTxActivityDue(&a, &in, 3000)); mtxTxActivitySent(&a, 3000);
    CHECK(!mtxTxActivityDue(&a, &in, 3999));
    CHECK(!mtxTxActivityDue(&a, &in, 4000));                             // 3 s after the last press: closed
    CHECK(a.until_ms == 0 && a.last_ms == 0);
    mtxTxActivityNote(&a, 4500);                                         // a new press: due at once again
    CHECK(mtxTxActivityDue(&a, &in, 4500)); }
  // A queue that refused (a pending 0xF1) is retried on the next tick, never skipped.
  { MtxTxActivity a; mtxTxActivityInit(&a); MtxTxActIn in = actIn();
    mtxTxActivityNote(&a, 100);
    CHECK(mtxTxActivityDue(&a, &in, 100));                               // not marked sent: the queue refused
    CHECK(mtxTxActivityDue(&a, &in, 200)); }
  // The ceremony keeps the window open with no press; the period holds throughout.
  { MtxTxActivity a; mtxTxActivityInit(&a); MtxTxActIn in = actIn(); in.arming = true;
    int sends = 0; uint32_t last = 0, maxgap = 0;
    for (uint32_t t = 10000; t <= 40000; t += 100)
      if (mtxTxActivityDue(&a, &in, t)) { if (sends) maxgap = (t - last > maxgap) ? t - last : maxgap; last = t; sends++; mtxTxActivitySent(&a, t); }
    CHECK(sends >= 15 && maxgap == 2000); }
  // The gates: never in SINGLE / old-style / non-owner / locked / setup / RTM active / 0xF1/1 queued / throttle >= 8.
  for (int g = 0; g < 8; g++) {
    MtxTxActivity a; mtxTxActivityInit(&a); MtxTxActIn in = actIn(); in.arming = true;
    if (g == 0) in.v3 = false;
    if (g == 1) in.owner_multi = false;
    if (g == 2) in.locked = true;
    if (g == 3) in.in_setup = true;
    if (g == 4) in.rtm_active = true;
    if (g == 5) in.rtm_on_queued = true;
    if (g == 6) in.thr_byte = kMtxZeroThr;
    if (g == 7) in.thr_byte = 200;
    mtxTxActivityNote(&a, 500);
    bool any = false;
    for (uint32_t t = 500; t < 20000; t += 50) any = any || mtxTxActivityDue(&a, &in, t);
    CHECK(!any);
  }
  // A press-and-combo script: a RIGHT tap, a 2.9 s gap, a 3 s LEFT hold, then the ceremony 15 s: one repeat at the
  // first press, and never more than 2.5 s between two repeats until the ceremony ends (RTM goes active).
  { MtxTxActivity a; mtxTxActivityInit(&a); MtxTxActIn in = actIn();
    std::vector<uint32_t> sent;
    for (uint32_t t = 0; t <= 22000; t += 10) {
      const bool tap  = (t >= 1000 && t < 1400);
      const bool hold = (t >= 4300 && t < 7300);
      in.arming = (t >= 7300 && t < 22000);
      if (tap || hold) mtxTxActivityNote(&a, t);
      if (mtxTxActivityDue(&a, &in, t)) { sent.push_back(t); mtxTxActivitySent(&a, t); }
    }
    CHECK(!sent.empty() && sent[0] == 1000);
    bool ok = true;
    for (size_t i = 1; i < sent.size(); i++) ok = ok && (sent[i] - sent[i - 1] <= 2500);
    CHECK(ok && sent.back() >= 20000); }
  // RTM goes active: from that tick on, nothing - the ceremony's 0xF1/1 is the last word.
  { MtxTxActivity a; mtxTxActivityInit(&a); MtxTxActIn in = actIn(); in.arming = true;
    CHECK(mtxTxActivityDue(&a, &in, 100)); mtxTxActivitySent(&a, 100);
    in.arming = false; in.rtm_active = true;
    bool any = false;
    for (uint32_t t = 100; t < 10000; t += 50) any = any || mtxTxActivityDue(&a, &in, t);
    CHECK(!any); }
  // Across the millis() wrap.
  { MtxTxActivity a; mtxTxActivityInit(&a); MtxTxActIn in = actIn();
    const uint32_t t0 = 0xFFFFF000u;
    mtxTxActivityNote(&a, t0);
    CHECK(mtxTxActivityDue(&a, &in, t0)); mtxTxActivitySent(&a, t0);
    CHECK(!mtxTxActivityDue(&a, &in, t0 + 1999));
    CHECK(mtxTxActivityDue(&a, &in, t0 + 2000));                         // past the wrap, still 2 s
    CHECK(!mtxTxActivityDue(&a, &in, t0 + 3000)); }
  testEnd("T34");
}

// ============================================================
// T35 - a refused arm gesture: E9 against a busy owner, "St" otherwise (owner 2026-10-20)
// ============================================================
static void t35()
{
  testBegin();
  MtxTxStatus t; mtxTxStatusInit(&t);
  CHECK(mtxTxRefusalSignal(&t, 1000) == MTX_RS_ST);                      // no reply ever: St
  const uint8_t busy[4] = { MTX_S_BUSY, MTX_S_SEIZABLE, MTX_S_REFUSED, MTX_S_SEIZED };
  for (int i = 0; i < 4; i++) {
    mtxTxStatusInit(&t); mtxTxOnReply(&t, busy[i], 1000);
    CHECK(mtxTxRefusalSignal(&t, 1000) == MTX_RS_E9);
    CHECK(mtxTxRefusalSignal(&t, 6000) == MTX_RS_E9);                    // 5 s: still fresh
    CHECK(mtxTxRefusalSignal(&t, 6001) == MTX_RS_ST);                    // stale: not evidence of a busy owner
  }
  const uint8_t other[3] = { MTX_S_FREE, MTX_S_OWNER, MTX_S_OWNER_SOLO };
  for (int i = 0; i < 3; i++) { mtxTxStatusInit(&t); mtxTxOnReply(&t, other[i], 1000); CHECK(mtxTxRefusalSignal(&t, 1100) == MTX_RS_ST); }
  testEnd("T35");
}

// ============================================================
// T36 - the ownership dot (24a0371) and the non-owner test of the summon path
// ============================================================
static void t36()
{
  testBegin();
  MtxTxStatus t; mtxTxStatusInit(&t);
  CHECK(mtxTxDotState(&t, 1000, true) == MTX_DOT_OFF && !mtxTxNotOwner(&t, true));   // no reply yet
  struct { uint8_t s, dot; bool not_owner; } rows[] = {
    { MTX_S_OWNER_SOLO, MTX_DOT_OFF,   false },   // one remote: today's screen, today's paths
    { MTX_S_OWNER,      MTX_DOT_SOLID, false },
    { MTX_S_FREE,       MTX_DOT_PULSE, true  },
    { MTX_S_BUSY,       MTX_DOT_SLOW,  true  },
    { MTX_S_SEIZABLE,   MTX_DOT_SLOW,  true  },
    { MTX_S_REFUSED,    MTX_DOT_SLOW,  true  },
    { MTX_S_SEIZED,     MTX_DOT_SLOW,  true  },
  };
  for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
    mtxTxStatusInit(&t); mtxTxOnReply(&t, rows[i].s, 2000);
    CHECK(mtxTxDotState(&t, 2000, true) == rows[i].dot);
    CHECK(mtxTxDotState(&t, 7000, true) == rows[i].dot);                 // 5 s: still shown
    CHECK(mtxTxDotState(&t, 7001, true) == MTX_DOT_OFF);                 // no reply for 5 s: off
    CHECK(mtxTxDotState(&t, 2000, false) == MTX_DOT_OFF);                // an old-style link: always off
    CHECK(mtxTxNotOwner(&t, true) == rows[i].not_owner);
    CHECK(!mtxTxNotOwner(&t, false));                                    // an old-style link: today's paths
  }
  testEnd("T36");
}

// ============================================================
// T37 - the remote's config rules (TX cfgValidateCrossField(): mtxValidateTxLink() + mtxClampTxConfig())
// ============================================================
static void t37()
{
  testBegin();
  // The link record: 0 and 3 stay; anything else unpairs (with the note); unpaired -> 0.
  for (uint32_t pr = 0; pr <= 0xFFFF; pr += (pr < 8 ? 1 : 997)) {
    uint16_t paired = 1, proto = (uint16_t)pr;
    const uint8_t note = mtxValidateTxLink(&paired, &proto);
    if (pr == 0 || pr == 3) CHECK(note == 0 && paired == 1 && proto == pr);
    else                    CHECK(note == 1 && paired == 0 && proto == 0);
  }
  { uint16_t paired = 0, proto = 3; CHECK(mtxValidateTxLink(&paired, &proto) == 0 && proto == 0); }
  // The clamps: never a rejection (the load path), always inside the rows' ranges afterwards.
  for (uint32_t v = 0; v <= 0xFFFF; v += (v < 2000 ? 1 : 61)) {
    uint16_t a = (uint16_t)v, h = (uint16_t)v, s = (uint16_t)v;
    mtxClampTxConfig(&a, &h, &s);
    CHECK(a == 0 || (a >= 5 && a <= 180));
    CHECK(h >= 1000 && h <= 1800);
    CHECK(s == 0 || (s >= 30 && s <= 600));
    if (v == 0)                CHECK(a == 0 && s == 0);                 // 0 = off stays off
    if (v >= 5 && v <= 180)    CHECK(a == v);                           // in range: unchanged
    if (v >= 1 && v <= 4)      CHECK(a == 5);                           // the 1-4 gap is raised, as 039b908 asked
    if (v >= 30 && v <= 600)   CHECK(s == v);
    if (v >= 1 && v <= 29)     CHECK(s == 30);
    if (v >= 1000 && v <= 1800) CHECK(h == v);
  }
  testEnd("T37");
}

// ============================================================
// T38 - E1-E4 (owner order 2026-10-21): the number of the remote in control, at reply index 20
// ============================================================
// txFeed - one remote's replies since reply number `from`, into that remote's MtxTxStatus, as waitForTelemetry() does
// (TX Radio.ino): the status first, then, for a reply at index 20, the number.
static void txFeed(MtxTxStatus* t, const Sim* s, size_t from, int slot, uint32_t now)
{
  for (size_t i = from; i < s->replies.size(); i++) {
    if (s->replies[i].first != slot) continue;
    mtxTxOnReply(t, (uint8_t)s->replies[i].second, now);
    if (s->reply_num[i] >= 0) mtxTxOnNumber(t, (uint8_t)s->reply_num[i], now);
  }
}
// numsTo - what the replies to one slot carried since reply number `from`: the number, or -1 for a telemetry byte.
static std::vector<int> numsTo(const Sim* s, size_t from, int slot)
{
  std::vector<int> v;
  for (size_t i = from; i < s->replies.size(); i++) if (s->replies[i].first == slot) v.push_back(s->reply_num[i]);
  return v;
}
static void t38()
{
  testBegin();
  // (1) The wire: index 20 is not a telemetry index (the rotation never reaches it) and packs with every status.
  CHECK(kMtxIdxInControl >= kMtxTelemetryLen && kMtxIdxInControl <= 0x1F);
  { uint8_t idx = 0; bool hit = false;
    for (int i = 0; i < 1000; i++) hit = hit || (mtxTelIndexNext(&idx) >= kMtxTelemetryLen);
    CHECK(!hit); }
  for (uint8_t st = 0; st < MTX_S_RESERVED; st++) {
    const uint8_t b3 = mtxReplyByte3(st, kMtxIdxInControl);
    CHECK(mtxReplyStatusOf(b3) == st && mtxReplyIndexOf(b3) == kMtxIdxInControl);
  }
  // (2) The statuses that show the number are exactly the ones whose refusal is the E code (mtxTxRefusalSignal()).
  for (uint8_t st = 0; st < MTX_S_RESERVED; st++) {
    const bool want = (st == MTX_S_BUSY || st == MTX_S_SEIZABLE || st == MTX_S_REFUSED || st == MTX_S_SEIZED);
    CHECK(mtxStatusShowsNumber(st) == want);
    MtxTxStatus t; mtxTxStatusInit(&t); mtxTxOnReply(&t, st, 1000);
    CHECK((mtxTxRefusalSignal(&t, 1000) == MTX_RS_E9) == want);
  }
  CHECK(!mtxStatusShowsNumber(MTX_S_RESERVED));                          // never sent by a buggy
  // (3) Who is in control: the owner (slot + 1), else a claim token's holder, else nobody (0).
  { MtxRxState st; mtxRxStateInit(&st);
    CHECK(mtxInControlNumber(&st) == kMtxNumberNone);
    for (int8_t o = 0; o < (int8_t)kMtxSlots; o++) { st.owner = o; CHECK(mtxInControlNumber(&st) == (uint8_t)(o + 1)); }
    st.owner = kMtxNone;
    for (int8_t k = 0; k < (int8_t)kMtxSlots; k++) { st.token.slot = k; st.token.ms = 5; CHECK(mtxInControlNumber(&st) == (uint8_t)(k + 1)); }
    st.owner = 2; st.token.slot = 1;
    CHECK(mtxInControlNumber(&st) == 3); }                               // the owner first
  // (4) The send rule: REFUSED always; a changed number at once; otherwise every other reply; any other status never,
  //     and it resets the note (the remote drops its number on it).
  { uint8_t told = kMtxNumberUntold, turn = 0;
    const struct { uint8_t s, n; bool due; } seq[] = {
      { MTX_S_BUSY, 1, true }, { MTX_S_BUSY, 1, false }, { MTX_S_BUSY, 1, true }, { MTX_S_BUSY, 2, true },
      { MTX_S_BUSY, 2, false }, { MTX_S_REFUSED, 2, true }, { MTX_S_REFUSED, 2, true }, { MTX_S_REFUSED, 2, true },
      { MTX_S_SEIZABLE, 2, false }, { MTX_S_SEIZABLE, 2, true }, { MTX_S_FREE, 0, false }, { MTX_S_BUSY, 2, true },
      { MTX_S_OWNER, 3, false }, { MTX_S_OWNER_SOLO, 3, false }, { MTX_S_SEIZED, 3, true }, { MTX_S_SEIZED, 0, true },
    };
    bool ok = true;
    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++) ok = ok && (mtxReplyNumberDue(seq[i].s, seq[i].n, &told, &turn) == seq[i].due);
    CHECK(ok);
    told = kMtxNumberUntold; turn = 0; int n = 0, run = 0, maxrun = 0;
    for (int i = 0; i < 100; i++) { if (mtxReplyNumberDue(MTX_S_BUSY, 4, &told, &turn)) { n++; run = 0; } else if (++run > maxrun) maxrun = run; }
    CHECK(n == 50 && maxrun == 1); }                                     // half the rotation stays telemetry
  // (5) Through the real model, two remotes. The master (slot 0) rides; the guest squeezes: every reply is REFUSED with
  //     "1", and the rider in control never gets the number. Then the guest lets go: BUSY, the number every other reply.
  { Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000); fr(&s, 0, 120, 127, 1300);
    simClear(&s);
    for (uint32_t t = 1400; t < 2400; t += 100) { fr(&s, 0, 120, 127, t); fr(&s, 1, 90, 127, t + 50); }
    bool ok = true; int n = 0;
    for (size_t i = 0; i < s.replies.size(); i++) {
      if (s.replies[i].first == 0) ok = ok && s.replies[i].second == MTX_S_OWNER && s.reply_num[i] == -1;
      else { ok = ok && s.replies[i].second == MTX_S_REFUSED && s.reply_num[i] == 1; n++; }
    }
    CHECK(ok && n == 10);
    MtxTxStatus g; mtxTxStatusInit(&g); txFeed(&g, &s, 0, 1, 2400);
    CHECK(mtxTxRefusalDigit(&g, 2400) == 1);                              // the guest's screen: E1
    simClear(&s);
    for (uint32_t t = 2400; t < 4400; t += 100) { fr(&s, 0, 120, 127, t); fr(&s, 1, 0, 127, t + 50); }
    const std::vector<int> v = numsTo(&s, 0, 1);
    int nums = 0, run = 0, maxrun = 0; ok = true;
    for (size_t i = 0; i < v.size(); i++) {
      if (v[i] >= 0) { ok = ok && v[i] == 1; nums++; run = 0; } else if (++run > maxrun) maxrun = run;
    }
    for (size_t i = 0; i < s.replies.size(); i++) if (s.replies[i].first == 1) ok = ok && s.replies[i].second == MTX_S_BUSY;
    CHECK(ok && v.size() == 20 && nums == 10 && maxrun == 1);
    mtxTxStatusInit(&g); txFeed(&g, &s, 0, 1, 4350);
    CHECK(mtxTxRefusalDigit(&g, 4350) == 1);                              // a summon refused now: E1
    CHECK(mtxTxRefusalDigit(&g, g.number_ms + 5000) == 1 && mtxTxRefusalDigit(&g, g.number_ms + 5001) == 9); }
  // (6) The master's take-back, two remotes: the master hears SEIZABLE with "2" (the guest); after the take-back the guest
  //     hears SEIZED with "1" on its very next reply.
  { Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 1, 1000); fr(&s, 1, 120, 127, 1300);
    simClear(&s);
    fr(&s, 0, 0, 127, 1400); fr(&s, 0, 0, 127, 1500);
    CHECK(s.replies.size() == 2 && s.replies[0].second == MTX_S_SEIZABLE && s.reply_num[0] == 2 && s.reply_num[1] == -1);
    fr(&s, 1, 120, 127, 1550);
    fr(&s, 0, 0xF1, 0x5A, 1600);
    CHECK(s.st.owner == 0);
    simClear(&s);
    fr(&s, 1, 120, 127, 1700);
    CHECK(lastStatus(&s) == MTX_S_SEIZED && s.reply_num.back() == 1);
    fr(&s, 0, 0, 127, 1750);
    CHECK(lastStatus(&s) == MTX_S_OWNER && s.reply_num.back() == -1); }
  // (7) Three remotes: a bystander is told a change of the remote in control on its very next reply.
  { Sim s; simInit(&s, false, 0x07);
    grantBySqueeze(&s, 1, 1000); fr(&s, 1, 120, 127, 1300);
    simClear(&s);
    fr(&s, 2, 0, 127, 1350); fr(&s, 2, 0, 127, 1450);
    CHECK(s.replies.size() == 2 && s.replies[0].second == MTX_S_BUSY && s.reply_num[0] == 2 && s.reply_num[1] == -1);
    fr(&s, 0, 0, 127, 1500); fr(&s, 0, 0, 127, 1600); fr(&s, 1, 120, 127, 1650);
    fr(&s, 0, 0xF1, 0x5A, 1700);                                          // the master takes back from the guest
    CHECK(s.st.owner == 0);
    fr(&s, 2, 0, 127, 1750);                                              // C's next reply: still BUSY, now "1"
    CHECK(lastStatus(&s) == MTX_S_BUSY && s.reply_num.back() == 1);
    MtxTxStatus c; mtxTxStatusInit(&c); txFeed(&c, &s, 0, 2, 1750);
    CHECK(mtxTxRefusalDigit(&c, 1750) == 1); }
  // (8) A claim token: its holder counts as in control for everyone else; once it lapses nobody is, and a SEIZED remote
  //     falls back to E9 ("busy, number unknown") - one of the two cases where the fallback is reached.
  { Sim s; simInit(&s, false, 0x07);
    grantBySqueeze(&s, 0, 1000); fr(&s, 0, 0xF2, 0x81, 1300); loopTick(&s);   // A owns, Follow-Me armed
    for (uint32_t t = 1400; t <= 11400; t += 100) { fr(&s, 1, 0, 127, t); fr(&s, 2, 0, 127, t + 10); }   // A silent
    fr(&s, 1, 0, 127, 11500);
    simClear(&s);
    fr(&s, 1, 90, 127, 11600);                                            // B claims; Follow-Me still on the loop: a token
    CHECK(s.st.owner == kMtxNone && s.st.token.slot == 1 && lastStatus(&s) == MTX_S_FREE && s.reply_num.back() == -1);
    fr(&s, 2, 0, 127, 11650);
    CHECK(lastStatus(&s) == MTX_S_BUSY && s.reply_num.back() == 2);       // C: E2
    fr(&s, 0, 0, 127, 11700);
    CHECK(lastStatus(&s) == MTX_S_SEIZED && s.reply_num.back() == 2);     // A, back: tA, and a summon would show E2
    MtxTxStatus a; mtxTxStatusInit(&a); txFeed(&a, &s, 0, 0, 11700);
    CHECK(mtxTxRefusalDigit(&a, 11700) == 2);
    wake(&s, 12601);
    CHECK(s.st.token.slot == kMtxNone && s.st.owner == kMtxNone);
    const size_t from = s.replies.size();
    fr(&s, 0, 0, 127, 12700);
    CHECK(lastStatus(&s) == MTX_S_SEIZED && s.reply_num.back() == 0);     // nobody in control
    txFeed(&a, &s, from, 0, 12700);
    CHECK(mtxTxRefusalSignal(&a, 12700) == MTX_RS_E9 && mtxTxRefusalDigit(&a, 12700) == 9); }
  // (9) SINGLE: no reply ever carries the number - today's reply.
  { Sim s; simInit(&s, true, 0x01);
    for (uint32_t t = 1000; t < 3000; t += 100) fr(&s, 0, ((t / 100) % 2) ? 0 : 90, 127, t);
    bool none = !s.replies.empty();
    for (size_t i = 0; i < s.replies.size(); i++) none = none && s.reply_num[i] == -1 && s.replies[i].second == MTX_S_OWNER_SOLO;
    CHECK(none); }
  // (10) The remote: the number is held only with a status that shows it, for at most 5 s; FREE or OWNER drops it; a
  //      value outside 1-4 leaves it unknown; across the millis() wrap.
  { MtxTxStatus t; mtxTxStatusInit(&t);
    CHECK(mtxTxRefusalDigit(&t, 1000) == 9);
    mtxTxOnReply(&t, MTX_S_BUSY, 1000); mtxTxOnNumber(&t, 3, 1000);
    CHECK(mtxTxRefusalDigit(&t, 1000) == 3 && mtxTxRefusalDigit(&t, 6000) == 3 && mtxTxRefusalDigit(&t, 6001) == 9);
    mtxTxOnReply(&t, MTX_S_BUSY, 1500);                                   // a telemetry reply keeps it
    CHECK(mtxTxRefusalDigit(&t, 1500) == 3);
    mtxTxOnReply(&t, MTX_S_FREE, 1600);  CHECK(mtxTxRefusalDigit(&t, 1600) == 9);
    mtxTxOnReply(&t, MTX_S_BUSY, 1700);  CHECK(mtxTxRefusalDigit(&t, 1700) == 9);   // until a number comes again
    mtxTxOnNumber(&t, 1, 1700);          CHECK(mtxTxRefusalDigit(&t, 1700) == 1);
    mtxTxOnReply(&t, MTX_S_OWNER, 1800); CHECK(t.number == 0);
    mtxTxOnNumber(&t, 2, 1800);          CHECK(t.number == 0);            // never held with an owner status
    const uint8_t bad[4] = { 0, 5, 9, 255 };
    for (int i = 0; i < 4; i++) {
      mtxTxOnReply(&t, MTX_S_REFUSED, 2000); mtxTxOnNumber(&t, 4, 2000); CHECK(mtxTxRefusalDigit(&t, 2000) == 4);
      mtxTxOnNumber(&t, bad[i], 2001);     CHECK(t.number == 0 && mtxTxRefusalDigit(&t, 2001) == 9);
    }
    mtxTxStatusInit(&t);
    const uint32_t w = 0xFFFFF000u;
    mtxTxOnReply(&t, MTX_S_SEIZED, w); mtxTxOnNumber(&t, 2, w);
    CHECK(mtxTxRefusalDigit(&t, w + 5000u) == 2 && mtxTxRefusalDigit(&t, w + 5001u) == 9); }
  // (11) Property, three remotes, random frames (squeezes, zeros, locks, take-backs, summons, Follow-Me, timer wakes): right
  //      after every reply that shows the E code, that remote's digit is the true number of the remote in control, or 9
  //      when nobody is in control - never another remote's number.
  { Sim s; simInit(&s, false, 0x07);
    MtxTxStatus tx[3]; for (int i = 0; i < 3; i++) mtxTxStatusInit(&tx[i]);
    uint32_t t = 1000; size_t seen = 0; int checked = 0, numbered = 0; bool ok = true;
    for (int step = 0; step < 60000; step++) {
      t += 10 + rnd() % 90;
      const int8_t slot = (int8_t)(rnd() % 3);
      const uint32_t r = rnd() % 100;
      uint8_t b3 = 0, b4 = 127;
      if (r < 40)      { b3 = 0; }
      else if (r < 75) { b3 = (uint8_t)(12 + rnd() % 200); }
      else if (r < 82) { b3 = 0xF1; b4 = 0x10; }                          // LOCKED
      else if (r < 86) { b3 = 0xF1; b4 = 0x5A; }                          // SEIZE
      else if (r < 90) { b3 = 0xF1; b4 = 0x3C; }                          // SUMMON
      else if (r < 94) { b3 = 0xF2; b4 = (uint8_t)(0x80 | (1 + rnd() % 5)); }
      else             { b3 = 0xF1; b4 = 0; }
      if (rnd() % 7 == 0)  loopTick(&s);
      if (rnd() % 50 == 0) wake(&s, t);
      fr(&s, slot, b3, b4, t);
      for (; seen < s.replies.size(); seen++) {
        const int sl = s.replies[seen].first;
        const uint8_t st = (uint8_t)s.replies[seen].second;
        mtxTxOnReply(&tx[sl], st, t);
        if (s.reply_num[seen] >= 0) { mtxTxOnNumber(&tx[sl], (uint8_t)s.reply_num[seen], t); numbered++; }
        if (mtxStatusShowsNumber(st)) {
          const uint8_t truth = mtxInControlNumber(&s.st);
          ok = ok && mtxTxRefusalDigit(&tx[sl], t) == (truth ? truth : 9);
          checked++;
        }
      }
    }
    CHECK(ok && checked > 1000 && numbered > 500); }
  testEnd("T38");
}

// ============================================================
// T39 - full audit F-1: the boot ID at an owner edge (one burst per edge; one packet after a squeeze)
// ============================================================
// A model of the remote's boot-ID path around an owner edge, with the REAL pure decisions of the fix
// (mtxTxBootIdEdgeSends(), mtxTxBootIdBeltDue()). The meta queue: another 0xF1 burst (qo, e.g. an "RTM off") and the boot
// ID (q), sends left; queueMetaPacketIfFree() refuses while any 0xF1 is pending. sendData: one meta packet per 100 ms
// cycle, never more than 3 in a row (then a control frame). The loop's txBootIdTick() is MIRRORED here statement for
// statement from TX RTMState.ino (in control, link fresh) - the fixed version, and the version before the fix for the
// "before" counts; `literal_now` is the variant that counts the edge at the tick's own time instead of its stamp.
struct BootSim {
  uint32_t last_ms, edge_ms, sent_ms;
  bool     requeue, fixed, literal_now;
  int      q, qo;                 // boot-ID sends left / another 0xF1 burst's sends left
  int      on_air;                // boot-ID packets sent
  int      meta_run, max_meta_run;   // non-control cycles in a row
  std::vector<uint32_t> sends;    // the times the boot-ID packets went out
};
static bool bsQueue(BootSim* b, uint8_t n) { if (n == 0 || b->q > 0 || b->qo > 0) return false; b->q = n; return true; }
// The owner edge (telemetry task): txBootIdOwnerEdge(). Before the fix: always 3, never stamped.
static void bsEdge(BootSim* b, uint32_t now, uint8_t thr)
{
  if (!b->fixed) { if (!bsQueue(b, 3)) b->requeue = true; return; }
  const uint8_t sends = mtxTxBootIdEdgeSends(b->q > 0, thr);
  if (sends == 0 || bsQueue(b, sends)) { b->requeue = false; b->edge_ms = mtxStampOf(now); } else b->requeue = true;
}
// The summon's belt (loop task, at its answer): txBootIdSummonBelt(). Before the fix it called the edge again.
static void bsBelt(BootSim* b, uint32_t now, uint8_t thr)
{
  if (!b->fixed) { bsEdge(b, now, thr); return; }
  if (mtxTxBootIdBeltDue(b->edge_ms, b->last_ms)) bsEdge(b, now, thr);
}
// txBootIdTick() in control with a fresh link (mirror). The fixed version first counts the edge's stamp.
static void bsTick(BootSim* b, uint32_t now)
{
  if (b->fixed) { const uint32_t e = b->edge_ms; if (e != 0) { b->edge_ms = 0; b->last_ms = b->literal_now ? mtxStampOf(now) : e; } }
  if (b->requeue) { b->requeue = false; b->last_ms = 0; }
  if (b->last_ms != 0 && b->q == 0 && (int32_t)(b->sent_ms - b->last_ms) < 0) b->last_ms = 0;   // T-8
  if (b->last_ms != 0 && (now - b->last_ms) < 10000u) return;
  const uint8_t sends = (b->last_ms == 0) ? 3 : 1;
  if (bsQueue(b, sends)) b->last_ms = mtxStampOf(now);
}
// One run from an edge at t0 (a reply that says OWNER, answering a frame with throttle byte thr). The loop ticks every
// 110 ms but not before t0 + block_ms (a held toggle, a blocking St); a summon's belt runs at t0 + belt_at (-1 = none; it
// is polled inside a held toggle too). blocker: another 0xF1 burst already queued at the edge. edge = false: no edge at
// all (one remote, an old-style link): the tick alone, from a fresh link.
static BootSim bsRun(bool fixed, bool literal_now, bool edge, uint8_t thr, int belt_at, uint32_t block_ms, bool blocker,
                     uint32_t span_ms)
{
  BootSim b; b.last_ms = b.edge_ms = b.sent_ms = 0; b.requeue = false; b.fixed = fixed; b.literal_now = literal_now;
  b.q = 0; b.qo = blocker ? 3 : 0; b.on_air = 0; b.meta_run = 0; b.max_meta_run = 0;
  const uint32_t t0 = 100000;
  if (edge) bsEdge(&b, t0, thr);
  uint32_t next_tick = t0 + 5, next_send = t0 + 60;
  for (uint32_t t = t0; t <= t0 + span_ms; t++) {
    if (belt_at >= 0 && t == t0 + (uint32_t)belt_at) bsBelt(&b, t, thr);
    if (t == next_send) {
      if ((b.qo > 0 || b.q > 0) && b.meta_run < 3) {
        if (b.qo > 0) b.qo--; else { b.q--; b.on_air++; b.sent_ms = t; b.sends.push_back(t); }
        if (++b.meta_run > b.max_meta_run) b.max_meta_run = b.meta_run;
      } else {
        b.meta_run = 0;                                                    // a control frame
      }
      next_send += 100;
    }
    if (t == next_tick) { if (t >= t0 + block_ms) bsTick(&b, t); next_tick += 110; }
  }
  return b;
}
static void t39()
{
  testBegin();
  // The pure decisions: 0 if one is already queued; ONE after a squeeze (trigger byte 8 or more); 3 with the trigger
  // released (a summon, a take-back). The belt only while neither the edge nor the tick has queued one.
  for (int thr = 0; thr < 256; thr++) {
    CHECK(mtxTxBootIdEdgeSends(true, (uint8_t)thr) == 0);
    CHECK(mtxTxBootIdEdgeSends(false, (uint8_t)thr) == ((thr >= kMtxZeroThr) ? 1 : 3));
  }
  CHECK(mtxTxBootIdBeltDue(0, 0) && !mtxTxBootIdBeltDue(5, 0) && !mtxTxBootIdBeltDue(0, 5) && !mtxTxBootIdBeltDue(5, 5));
  // (1) A squeeze claim (the trigger at 90), the loop free: before the fix six packets, two runs of three non-control
  //     cycles (the F-1 finding); now ONE packet, one 100 ms cycle, and no burst from the tick.
  { const BootSim a = bsRun(false, false, true, 90, -1, 0, false, 3000);
    CHECK(a.on_air == 6 && a.max_meta_run == 3);
    const BootSim b = bsRun(true, false, true, 90, -1, 0, false, 3000);
    CHECK(b.on_air == 1 && b.max_meta_run == 1); }
  // (2) A take-back (trigger released) with RIGHT still held, so the loop is blocked 1.5 s: three packets. Counting the
  //     edge at the tick's own late time instead of its stamp would read the burst as overwritten (T-8) and send it again.
  { CHECK(bsRun(false, false, true, 0, -1, 1500, false, 3000).on_air == 6);
    const BootSim b = bsRun(true, false, true, 0, -1, 1500, false, 3000);
    CHECK(b.on_air == 3 && b.max_meta_run == 3);
    CHECK(bsRun(true, true, true, 0, -1, 1500, false, 3000).on_air == 6); }   // why it is a stamp
  // (3) A toggle summon: its answer (and the belt) is polled inside the held toggle at +10 ms, the loop blocked 2 s.
  { CHECK(bsRun(false, false, true, 0, 10, 2000, false, 3000).on_air == 6);
    CHECK(bsRun(true, false, true, 0, 10, 2000, false, 3000).on_air == 3); }
  // (4) A summon whose belt runs after the edge burst has gone out (+450 ms, loop blocked to +1000): before the fix a
  //     THIRD burst (nine packets); now three.
  { CHECK(bsRun(false, false, true, 0, 450, 1000, false, 4000).on_air == 9);
    CHECK(bsRun(true, false, true, 0, 450, 1000, false, 4000).on_air == 3); }
  // (5) The edge's queue refused (an RTM-off burst pending): the belt (after it drained) or the tick sends it, once.
  { CHECK(bsRun(true, false, true, 0, 10, 2000, true, 4000).on_air == 3);    // belt refused too: the tick, after the block
    CHECK(bsRun(true, false, true, 0, 400, 2000, true, 4000).on_air == 3);   // the belt, once the queue is free
    CHECK(bsRun(true, false, true, 90, -1, 0, true, 4000).on_air == 3); }    // a squeeze, refused: the tick's 3, as before
  // (6) After the edge the 10 s repeats run as before: one packet, the first no sooner than 10 s after the edge.
  { const BootSim b = bsRun(true, false, true, 90, -1, 0, false, 25000);
    CHECK(b.on_air == 3 && b.sends.size() == 3 && b.sends[1] - b.sends[0] >= 9900 && b.sends[2] - b.sends[1] >= 9900); }
  // (7) No edge (one remote, an old-style link): the fixed tick is the old tick - 3 at the fresh link, then 1 every 10 s.
  { const BootSim a = bsRun(false, false, false, 0, -1, 0, false, 25000);
    const BootSim b = bsRun(true, false, false, 0, -1, 0, false, 25000);
    CHECK(a.on_air == 5 && b.on_air == 5 && a.sends == b.sends); }
  testEnd("T39");
}

// ============================================================
// T40 - owner rule 2026-10-21: on a remote that is NOT in control, only a squeeze from zero or a COMPLETED Return-To-Me /
//        Follow-Me arm gesture (the SUMMON it then sends) takes control. A toggle tap, a gear change, the display cycle, a
//        gesture let go before its hold time, a lock or any other meta never does.
// ============================================================
// On the remote the SUMMON exists only as the internal queue type kMtxQueueSummon, queued only by mtxTxSummonTry() (TX
// MultiRemote.ino). That is called only where an arm gesture is COMPLETE (code-read): the toggle combos after their full
// hold (Hall.ino handleGearToggle(), and returnGesture() in RTMState.ino, reached from there), and the magnet on removal
// after its full band (Hall.ino runMagGesture()). A tap, a gear change, the display cycle or a combo released early reaches
// none of those sites; such a remote sends control frames with the trigger at zero only (the toggle is not steering then),
// and a lock sends LOCKED frames plus its "RTM off" / "Follow-Me off". The gesture handler itself is board code (pins,
// delays) and is not host-testable. What is: (1) nothing but that queue type can put 0xF1/0x3C on the air; (2) on the
// buggy no other frame such a remote can send takes control, even from a claimable owner, while the SUMMON and a squeeze
// from zero do; (3) from an owner who is not claimable not even those.
// t40Script - 5 s of what a remote that is not in control (slot 1) sends for one kind of gesture, from t; the owner (slot 0)
// stays silent (owner_idle = false) or sends zeros (true) meanwhile. Returns the time after it.
static uint32_t t40Script(Sim* s, int script, uint32_t t, bool owner_idle)
{
  static const uint8_t kSteer[5] = { 0, 60, 127, 200, 255 };
  for (int i = 0; i < 50; i++, t += 100) {
    if (owner_idle) fr(s, 0, 0, 127, t);
    const uint32_t u = t + 50;
    switch (script) {
      case 0:  fr(s, 1, 0, 127, u); break;                                  // tap, gear change, display cycle, early release
      case 1:  fr(s, 1, (uint8_t)(i % 12), kSteer[i % 5], u); break;        // the trigger touched under a squeeze, steering
      case 2:  if (i == 10)      fr(s, 1, 0xF1, 0x00, u);                   // a lock: LOCKED, then its RTM-off / FM-off
               else if (i == 11) fr(s, 1, 0xF2, 0x00, u);
               else              fr(s, 1, 0xF1, 0x10, u);
               break;
      default: {                                                            // every other meta, between zero frames
        static const uint8_t kMeta[8][2] = { {0xF1, 0x00}, {0xF1, 0x01}, {0xF1, 0x02}, {0xF1, 0x85}, {0xF2, 0x81},
                                             {0xF2, 0x03}, {0xF4, 0x01}, {0xF3, 0x03} };
        if (i % 2) fr(s, 1, kMeta[(i / 2) % 8][0], kMeta[(i / 2) % 8][1], u); else fr(s, 1, 0, 127, u);
        break;
      }
    }
  }
  return t;
}
static void t40()
{
  testBegin();
  // (1) The builder: no queue type or value but kMtxQueueSummon ever gives 0xF1/0x3C, on either link; a control frame is
  //     always control; the SUMMON itself only on the new-style link.
  { MtxLinkRec v3; v3.proto = kMtxProtoV3; memcpy(v3.dest_address, kR1, 3); memcpy(v3.efuse_id, kB, 3);
    MtxLinkRec v0 = v3; v0.proto = kMtxProtoLegacy;
    bool never = true; uint8_t f[6];
    for (int ty = 0; ty < 256; ty++) for (int va = 0; va < 256; va++) {
      if (ty != kMtxQueueSummon) {
        if (mtxBuildUplinkFromQueue((uint8_t)ty, (uint8_t)va, &v3, f, crc8)) never = never && !(f[3] == kMtxMetaRtm && f[4] == kMtxSummonValue);
        if (mtxBuildUplinkFromQueue((uint8_t)ty, (uint8_t)va, &v0, f, crc8)) never = never && !(f[3] == kMtxMetaRtm && f[4] == kMtxSummonValue);
      }
      if (mtxBuildUplink(MTX_B_CTRL, &v3, (uint8_t)ty, (uint8_t)va, f, crc8)) never = never && mtxKind(f[3], f[4]) == MTX_K_CTRL;
    }
    CHECK(never);
    CHECK(mtxBuildUplinkFromQueue(kMtxQueueSummon, 0, &v3, f, crc8) && f[3] == kMtxMetaRtm && f[4] == kMtxSummonValue);
    CHECK(!mtxBuildUplinkFromQueue(kMtxQueueSummon, 0, &v0, f, crc8)); }
  // (2) A claimable owner - silent 10 s, or at zero 10 s with nothing armed: every script leaves it in control (no claim,
  //     no release, no token, the outputs never zeroed); then the completed gesture's SUMMON, or a squeeze from zero, takes
  //     control - so the owner WAS claimable throughout.
  for (int idle = 0; idle < 2; idle++)
    for (int script = 0; script < 4; script++)
      for (int finish = 0; finish < 2; finish++) {
        Sim s; simInit(&s, false, 0x03); uint32_t t;
        if (!idle) { silentOwnerClaimable(&s, &t); }
        else {
          grantBySqueeze(&s, 0, 1000);
          for (t = 1300; t <= 11400; t += 100) { fr(&s, 0, 0, 127, t); fr(&s, 1, 0, 127, t + 50); }
        }
        CHECK(claimable(&s, t) == (idle ? MTX_CL_IDLE : MTX_CL_SILENT));
        simClear(&s);
        t = t40Script(&s, script, t, idle != 0);
        CHECK(s.st.owner == 0 && s.st.token.slot == kMtxNone && s.st.n.claims == 0 && s.st.n.summons == 0);
        CHECK(s.st.n.released_claim == 0 && !logHas(&s, "zero"));
        // The rider of slot 1 now completes an arm gesture (finish 0: its SUMMON burst) or squeezes from zero (finish 1).
        if (idle) fr(&s, 0, 0, 127, t);
        fr(&s, 1, 0, 127, t + 50); fr(&s, 1, 0, 127, t + 150);
        if (finish == 0) { fr(&s, 1, kMtxMetaRtm, kMtxSummonValue, t + 250); CHECK(s.st.n.summons == 1); }
        else             { fr(&s, 1, 90, 127, t + 250); }
        CHECK(s.st.n.claims == 1 && s.st.n.released_claim == 1 && (s.st.owner == 1 || s.st.token.slot == 1));
      }
  // (3) An owner who is NOT claimable (riding, or at zero for under 10 s): nothing takes control - not the SUMMON, not a
  //     squeeze (the master's take-back, a separate owner-approved gesture, is T5).
  for (int ride = 0; ride < 2; ride++) {
    Sim s; simInit(&s, false, 0x03);
    grantBySqueeze(&s, 0, 1000);
    uint32_t t = 1300;
    for (int i = 0; i < 30; i++, t += 100) { fr(&s, 0, ride ? 120 : 0, 127, t); fr(&s, 1, 0, 127, t + 50); }
    fr(&s, 1, kMtxMetaRtm, kMtxSummonValue, t + 60); fr(&s, 1, 90, 127, t + 70);
    CHECK(s.st.owner == 0 && s.st.n.claims == 0 && s.st.n.summons_refused == 1 && !logHas(&s, "zero"));
  }
  testEnd("T40");
}

int main(int argc, char** argv)
{
  printf("multi_tx_test - Common/MultiTx.h (v3.2 step 1)\n");
  t1(argc > 1 ? argv[1] : NULL);
  t2(); t3(); t4(); t5();
  printf("T6   -> covered by T31 (the v3.2 status priority)\n");
  t7(); t8(); t9(); t10(); t11(); t12(); t13(); t14(); t15(); t16(); t17(); t18();
  printf("T19  -> Tools/tests/config_migrate_test.cpp\n");
  t20(); t21(); t22(); t23(); t24(); t25(); t26(); t27(); t28(); t29(); t30(); t31(); t32(); t33();
  t34(); t35(); t36(); t37(); t38(); t39(); t40();
  printf("%s: %d checks, %d failed\n", g_fails ? "FAILED" : "PASSED", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
