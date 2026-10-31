// V2.5-Evo - 2026-10-31 - v3 (design audit 2026-10-31, Follow-Me package 2 unit A): the 18-byte packet with the remote's
//   own course and speed. New: testV3Framing (round trip of every code range and the sentinels; a v3 body cut to 14 bytes
//   is refused by the package-1 buggy for all 256 values of byte 13, the CRC collision included; cut to 15 bytes it never
//   parses as v2; bit flips), testAirTime (the A-T1 wait), testTxVelocity (the remote's same-epoch, HDOP, speed-bound,
//   course-under-1-m/s and NaN rules), testRxVelCheck (V-1 to V-3), testPublishOrder (the publish-last order and the
//   solution tag) and testPairings (the four remote / buggy pairings). The package-1 buggy is a FROZEN COPY of its parser
//   (fm-bundle 0464187) below. One line of testFraming changed: subtype 0x05 now promises 18 bytes, so the "any other
//   subtype is 14" loop skips 0x05 as well as 0x03. Every v1 / v2 assertion is otherwise unchanged. All coordinates here
//   are made-up numbers, none from a log.
// V2.5-Evo - 2026-10-12 - Host test for Common/RiderGpsPacket.h (audit 2026-10-11 delta A-1): the v1 / v2 framing both
//   ways, the old buggy's view of a v2 packet, the remote's fix counter (one step per NEW solution, never per send, never
//   per position change, never on a time-only sentence) and the buggy's NEW / REPEAT / DROP classification.
// Build and run:  g++ -std=c++17 -Wall -Wextra -o rider_gps_packet_test rider_gps_packet_test.cpp && ./rider_gps_packet_test

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>               // V2.5-Evo - 2026-10-31 - v3 tests: NAN / INFINITY
#include <initializer_list>     // V2.5-Evo - 2026-10-31 - v3 tests: range-for over value lists

#include "../../Source/Common/RiderGpsPacket.h"

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

static const uint8_t kDest[3] = { 0x46, 0xC9, 0xE9 };

// A v1 packet exactly as every remote before this change builds it (TX Radio.ino, 2026-04-24).
static void buildV1(uint8_t out[14], int32_t lat, int32_t lng)
{
  memcpy(out, kDest, 3); out[3] = 0xF3; out[4] = 0x02;
  memcpy(out + 5, &lat, 4); memcpy(out + 9, &lng, 4);
  out[13] = crc8(out, 13);
}

// The OLD buggy's acceptance test (RX Radio.ino processMetaGpsPacket before this change), on the 14 bytes it reads.
static bool oldBuggyAccepts(uint8_t* pkt14)
{
  if (memcmp(pkt14, kDest, 3) != 0) return false;
  if (pkt14[13] != crc8(pkt14, 13)) return false;
  if (pkt14[3] != 0xF3 || pkt14[4] != 0x02) return false;
  return true;
}

static void testFraming()
{
  // The announcement promises the length: 0x03 -> 15 (v2), 0x01 -> 14 (v1), anything else -> 14, as every buggy read it.
  assert(riderGpsDataLen(0x03) == 15);
  assert(riderGpsDataLen(0x01) == 14);
  for (int s = 0; s < 256; ++s) if (s != 0x03 && s != 0x05) assert(riderGpsDataLen((uint8_t)s) == 14);   // V2.5-Evo - 2026-10-31 - 0x05 = v3 (18)

  const int32_t lat = 26123456, lng = -80234567;
  // v2 round trip, every counter value.
  for (int c = 0; c < 256; ++c) {
    uint8_t p[15];
    riderGpsBuildV2(p, kDest, lat, lng, (uint8_t)c, crc8);
    assert(p[3] == 0xF3 && p[4] == 0x04 && p[13] == (uint8_t)c && p[14] == crc8(p, 14));
    RiderGpsFields f = {0, 0, false, 0};
    assert(riderGpsParse(p, 15, crc8, &f) == RGP_OK);
    assert(f.lat_ud == lat && f.lng_ud == lng && f.has_ctr && f.ctr == (uint8_t)c);
    // NEW REMOTE + OLD BUGGY: the old buggy listens for 14 bytes and reads the first 14 of the v2 body. It must reject
    // every one of them (byte 13 is the counter, not a CRC; and the subtype is 0x04) - it never takes a v2 position.
    assert(!oldBuggyAccepts(p));
    // A v2 body read as 14 bytes by the NEW buggy (announcement lost or corrupted): rejected too.
    assert(riderGpsParse(p, 14, crc8, &f) != RGP_OK);
  }
  // OLD REMOTE + NEW BUGGY: a v1 packet parses, with no counter.
  {
    uint8_t p[14]; buildV1(p, lat, lng);
    RiderGpsFields f = {0, 0, true, 9};
    assert(riderGpsParse(p, 14, crc8, &f) == RGP_OK);
    assert(f.lat_ud == lat && f.lng_ud == lng && !f.has_ctr && f.ctr == 0);
    assert(oldBuggyAccepts(p));   // and the old buggy, of course
  }
  // Corruption: a flipped bit anywhere fails the CRC; a wrong type / subtype under a good CRC fails the type test.
  {
    uint8_t p[15]; riderGpsBuildV2(p, kDest, lat, lng, 7, crc8);
    RiderGpsFields f;
    for (int i = 0; i < 15; ++i) for (int b = 0; b < 8; ++b) {
      uint8_t q[15]; memcpy(q, p, 15); q[i] ^= (uint8_t)(1u << b);
      assert(riderGpsParse(q, 15, crc8, &f) != RGP_OK);
    }
    uint8_t q[15]; memcpy(q, p, 15); q[4] = 0x02; q[14] = crc8(q, 14);   // a v1 subtype in a 15-byte body
    assert(riderGpsParse(q, 15, crc8, &f) == RGP_TYPE);
    memcpy(q, p, 15); q[3] = 0xF2; q[14] = crc8(q, 14);
    assert(riderGpsParse(q, 15, crc8, &f) == RGP_TYPE);
    assert(riderGpsParse(p, 13, crc8, &f) == RGP_LEN);
    assert(riderGpsParse(p, 6, crc8, &f) == RGP_LEN);
  }
}

static void testTxCounter()
{
  RiderGpsTxFixState s = {false, 0, 0};
  // First solution: counts.
  assert(riderGpsTxFixStep(&s, true, 12000000, 5, 5, kRiderGpsSameEpochMs) == 1);
  // The same solution sent again (2 Hz radio off a 1 Hz module, or a stalled module): no step, however often.
  for (int i = 0; i < 20; ++i) assert(riderGpsTxFixStep(&s, true, 12000000, 300 + i * 500, 300 + i * 500, kRiderGpsSameEpochMs) == 1);
  // A new solution with the SAME coordinates (a still rider) is still a new solution: the counter does not look at
  // positions at all.
  assert(riderGpsTxFixStep(&s, true, 12000020, 10, 10, kRiderGpsSameEpochMs) == 2);
  // A time-only sentence (fix lost: the module keeps sending a fresh time, no position): no step - the position is
  // older than the time by more than one epoch.
  assert(riderGpsTxFixStep(&s, true, 12000040, 120, 20, kRiderGpsSameEpochMs) == 2);
  assert(riderGpsTxFixStep(&s, true, 12000100, 700, 0, kRiderGpsSameEpochMs) == 2);
  // ... and when the fix comes back, the first solution counts.
  assert(riderGpsTxFixStep(&s, true, 12000120, 3, 3, kRiderGpsSameEpochMs) == 3);
  // A remote read that caught the time committed but not yet the position (it preempted the parser between the two):
  // the position is a whole epoch old -> no step now; the next packet (both committed) steps.
  assert(riderGpsTxFixStep(&s, true, 12000140, 200, 0, kRiderGpsSameEpochMs) == 3);
  assert(riderGpsTxFixStep(&s, true, 12000140, 1, 1, kRiderGpsSameEpochMs) == 4);
  // RMC and GGA of one epoch a few ms apart: the position may be slightly older than the time - still one step only.
  assert(riderGpsTxFixStep(&s, true, 12000160, 12, 4, kRiderGpsSameEpochMs) == 5);
  assert(riderGpsTxFixStep(&s, true, 12000160, 2, 2, kRiderGpsSameEpochMs) == 5);
  // No valid time: never a step.
  assert(riderGpsTxFixStep(&s, false, 12000180, 1, 1, kRiderGpsSameEpochMs) == 5);
  // Wrap: 255 -> 0 and keeps counting.
  RiderGpsTxFixState w = {true, 0, 255};
  assert(riderGpsTxFixStep(&w, true, 1, 0, 0, kRiderGpsSameEpochMs) == 0);
  assert(riderGpsTxFixStep(&w, true, 2, 0, 0, kRiderGpsSameEpochMs) == 1);
}

static RiderGpsFields v2(int32_t lat, int32_t lng, uint8_t c) { RiderGpsFields f = {lat, lng, true, c}; return f; }
static RiderGpsFields v1(int32_t lat, int32_t lng)            { RiderGpsFields f = {lat, lng, false, 0}; return f; }

// One packet through the buggy's rules, Phase A passing. Returns the class; counts distinct positions and solutions.
static uint8_t rx(RiderGpsRxState* s, RiderGpsFields f, int* distinct, int* solutions)
{
  const uint8_t c = riderGpsClassify(s, &f);
  riderGpsNoteCounter(s, &f);
  if (c == RGF_NEW) { if (riderGpsCommit(s, &f)) ++*distinct; ++*solutions; }
  return c;
}

static void testRxClassify()
{
  // v2, a STILL rider: identical coordinates, the counter moving -> every packet is a new solution, none is a new
  // distinct position (so the separation proof still cannot be fed by a still rider).
  {
    RiderGpsRxState s = {false, 0, 0, false, 0}; int d = 0, n = 0;
    for (int i = 0; i < 50; ++i) assert(rx(&s, v2(100, 200, (uint8_t)(i * 1)), &d, &n) == RGF_NEW);
    assert(n == 50 && d == 1);
  }
  // v2 re-broadcast: same counter, same coordinates -> REPEAT (no evidence), forever.
  {
    RiderGpsRxState s = {false, 0, 0, false, 0}; int d = 0, n = 0;
    assert(rx(&s, v2(100, 200, 9), &d, &n) == RGF_NEW);
    for (int i = 0; i < 100; ++i) assert(rx(&s, v2(100, 200, 9), &d, &n) == RGF_REPEAT);
    assert(n == 1 && d == 1);
  }
  // v2, the counter repeats with OTHER coordinates (a read that straddled an update) -> DROP; the next counter is new.
  {
    RiderGpsRxState s = {false, 0, 0, false, 0}; int d = 0, n = 0;
    assert(rx(&s, v2(100, 200, 9), &d, &n) == RGF_NEW);
    assert(rx(&s, v2(105, 200, 9), &d, &n) == RGF_DROP);
    assert(s.lat_ud == 100);                                  // nothing written
    assert(rx(&s, v2(105, 200, 10), &d, &n) == RGF_NEW);
    assert(n == 2 && d == 2);
  }
  // v2, Phase A rejects a solution: its counter is still noted, so its repeat is a DROP (not judged again), and the
  // reference stays the last ACCEPTED position.
  {
    RiderGpsRxState s = {false, 0, 0, false, 0}; int d = 0, n = 0;
    assert(rx(&s, v2(100, 200, 1), &d, &n) == RGF_NEW);
    RiderGpsFields bad = v2(900000, 200, 2);
    assert(riderGpsClassify(&s, &bad) == RGF_NEW);
    riderGpsNoteCounter(&s, &bad);                           // Phase A rejected: no commit
    assert(riderGpsClassify(&s, &bad) == RGF_DROP);
    assert(s.lat_ud == 100 && s.seeded);
  }
  // v1, the old coordinate rule: a still rider is a REPEAT (no evidence), a moving one NEW.
  {
    RiderGpsRxState s = {false, 0, 0, false, 0}; int d = 0, n = 0;
    assert(rx(&s, v1(100, 200), &d, &n) == RGF_NEW);
    for (int i = 0; i < 10; ++i) assert(rx(&s, v1(100, 200), &d, &n) == RGF_REPEAT);
    assert(rx(&s, v1(101, 200), &d, &n) == RGF_NEW);
    assert(n == 2 && d == 2);
  }
  // A v1 packet ends a v2 run: the next v2 packet is judged fresh even with the old counter value.
  {
    RiderGpsRxState s = {false, 0, 0, false, 0}; int d = 0, n = 0;
    assert(rx(&s, v2(100, 200, 5), &d, &n) == RGF_NEW);
    assert(rx(&s, v1(100, 200), &d, &n) == RGF_REPEAT);
    assert(rx(&s, v2(100, 200, 5), &d, &n) == RGF_NEW);
  }
  // The very first packet is always new (seeds the reference), v1 or v2.
  {
    RiderGpsRxState s = {false, 0, 0, false, 0};
    RiderGpsFields a = v2(0, 0, 0), b = v1(0, 0);
    assert(riderGpsClassify(&s, &a) == RGF_NEW);
    assert(riderGpsClassify(&s, &b) == RGF_NEW);
  }
}

// ============================================================
// V2.5-Evo - 2026-10-31 - v3 TESTS (design audit 2026-10-31, unit A)
// ============================================================
static long g_v3_checks = 0;   // how many v3 checks ran (printed at the end)
#define CHECK(c) do { ++g_v3_checks; assert(c); } while (0)

// THE PACKAGE-1 BUGGY (fm-bundle 0464187, the buggy in the field today): riderGpsDataLen() and riderGpsParse() exactly as
// they were before v3, FROZEN here so the pairing tests keep judging that buggy whatever the header does next.
static uint8_t p1DataLen(uint8_t announce_subtype) { return (announce_subtype == 0x03) ? 15 : 14; }
static uint8_t p1Parse(uint8_t* pkt, uint8_t len, RiderGpsFields* f)
{
  if (len != 14 && len != 15) return RGP_LEN;
  const uint8_t crc_at = (uint8_t)(len - 1);
  if (pkt[crc_at] != crc8(pkt, crc_at)) return RGP_CRC;
  const uint8_t want_sub = (len == 15) ? 0x04 : 0x02;
  if (pkt[3] != 0xF3 || pkt[4] != want_sub) return RGP_TYPE;
  memcpy(&f->lat_ud, pkt + 5, 4);
  memcpy(&f->lng_ud, pkt + 9, 4);
  f->has_ctr = (len == 15);
  f->ctr     = f->has_ctr ? pkt[13] : (uint8_t)0;
  return RGP_OK;
}

static const int32_t kLatV3 = 1000000, kLngV3 = -2000000;   // made-up microdegrees, not from any log

// One v3 packet built, parsed and its velocity read back: every byte and field must survive the trip.
static void v3RoundTrip(uint16_t crs, uint16_t spd, uint8_t age, uint8_t ctr)
{
  const uint32_t w = riderGpsVelPack(crs, spd, age);
  uint8_t p[18];
  riderGpsBuildV3(p, kDest, kLatV3, kLngV3, ctr, w, crc8);
  CHECK(p[3] == 0xF3 && p[4] == 0x06 && p[13] == ctr && p[17] == crc8(p, 17));
  RiderGpsFields f = {0, 0, false, 0};
  CHECK(riderGpsParse(p, 18, crc8, &f) == RGP_OK);
  CHECK(f.lat_ud == kLatV3 && f.lng_ud == kLngV3 && f.has_ctr && f.ctr == ctr);   // the position is never touched by the velocity
  const uint32_t got = riderGpsParsedVel(p, 18);
  CHECK(got == w);
  uint16_t c2 = 0, s2 = 0; uint8_t a2 = 0;
  riderGpsVelUnpack(got, &c2, &s2, &a2);
  CHECK(c2 == crs && s2 == spd && a2 == age);
}

static void testV3Framing()
{
  CHECK(riderGpsDataLen(0x05) == 18 && riderGpsDataLen(0x03) == 15 && riderGpsDataLen(0x01) == 14);
  CHECK(kRiderGpsLenMax == 18);
  // (The two cut-body tests run first, so a wrong data subtype is caught by the test that is about it.)

  // A v3 BODY CUT TO 14 BYTES (what a package-1 or v1-era buggy reads after announcement 0x05): refused for all 256 values
  // of byte 13. Exactly one of them makes byte 13 equal the CRC of bytes 0-12, and that one is refused BY THE SUBTYPE.
  {
    int crc_hits = 0;
    for (int b = 0; b < 256; ++b) {
      uint8_t p[18];
      riderGpsBuildV3(p, kDest, kLatV3, kLngV3, (uint8_t)b, riderGpsVelPack(100, 200, 2), crc8);
      RiderGpsFields f;
      if (p[13] == crc8(p, 13)) { ++crc_hits; CHECK(p1Parse(p, 14, &f) == RGP_TYPE); }
      CHECK(p1Parse(p, p1DataLen(0x05), &f) != RGP_OK);
      CHECK(!oldBuggyAccepts(p));
      CHECK(riderGpsParse(p, 14, crc8, &f) != RGP_OK);
    }
    CHECK(crc_hits == 1);   // the CRC-collision case was exercised
  }
  // NEVER PARSES AS v2: a v3 body cut to 15 bytes is refused by the package-1 parser and by this one for all 256 values
  // of byte 14, the CRC collision included. (Red edit: data subtype 0x04 for v3 makes the collision parse.)
  {
    int crc_hits = 0;
    for (int b = 0; b < 256; ++b) {
      uint8_t p[18];
      riderGpsBuildV3(p, kDest, kLatV3, kLngV3, 7, riderGpsVelPack((uint16_t)b, 200, 2), crc8);
      CHECK(p[14] == (uint8_t)b);
      RiderGpsFields f;
      if (p[14] == crc8(p, 14)) ++crc_hits;
      CHECK(p1Parse(p, 15, &f) != RGP_OK);
      CHECK(riderGpsParse(p, 15, crc8, &f) != RGP_OK);
    }
    CHECK(crc_hits == 1);
  }
  // Round trip: every course code (with a spread of speeds and ages), every speed code, every age code, sentinels too.
  // The data subtype on the wire is pinned to 0x06 here (v3RoundTrip()).
  for (int c = 0; c < 1024; ++c)
    for (int s : {0, 1, 511, 1022, 1023})
      for (int a : {0, 7, 14, 15}) v3RoundTrip((uint16_t)c, (uint16_t)s, (uint8_t)a, (uint8_t)c);
  for (int s = 0; s < 1024; ++s)
    for (int c : {0, 999, 1000, 1022, 1023}) v3RoundTrip((uint16_t)c, (uint16_t)s, 3, (uint8_t)s);
  for (int a = 0; a < 16; ++a) v3RoundTrip(500, 500, (uint8_t)a, 9);
  CHECK(riderGpsVelPack(1023, 1023, 15) == kRiderVelNoneW24);
  CHECK(riderGpsVelPack(0xFFFF, 0xFFFF, 0xFF) == kRiderVelNoneW24);   // each code is masked to its own width
  { uint8_t p[15]; riderGpsBuildV2(p, kDest, kLatV3, kLngV3, 5, crc8); CHECK(riderGpsParsedVel(p, 15) == kRiderVelNoneW24); }
  // Corruption: any single flipped bit fails the CRC; a v1 or v2 subtype in an 18-byte body under a good CRC is a type
  // failure; 16 and 17 bytes are not a length any version has.
  {
    uint8_t p[18]; riderGpsBuildV3(p, kDest, kLatV3, kLngV3, 7, riderGpsVelPack(321, 456, 3), crc8);
    RiderGpsFields f;
    for (int i = 0; i < 18; ++i) for (int b = 0; b < 8; ++b) {
      uint8_t q[18]; memcpy(q, p, 18); q[i] ^= (uint8_t)(1u << b);
      CHECK(riderGpsParse(q, 18, crc8, &f) != RGP_OK);
    }
    for (int sub : {0x02, 0x04}) {
      uint8_t q[18]; memcpy(q, p, 18); q[4] = (uint8_t)sub; q[17] = crc8(q, 17);
      CHECK(riderGpsParse(q, 18, crc8, &f) == RGP_TYPE);
    }
    CHECK(riderGpsParse(p, 16, crc8, &f) == RGP_LEN && riderGpsParse(p, 17, crc8, &f) == RGP_LEN);
  }
}

// FINDING A-T1: the remote's wait after the data packet covers the v3 air time with margin. The header refuses to compile
// with a shorter wait (static_assert); this checks the numbers it rests on.
// V2.5-Evo - 2026-10-31 - delta audit P2A-2: the guarantee is the wait LESS ONE TICK (vTaskDelay can return just after
// n - 1 ms), against air time + 2 ms: 17 ms >= 16.656 ms. 15 ms (14 ms guaranteed) still fails it.
static void testAirTime()
{
  static_assert(riderGpsAirTimeUs(18) == 14656, "v3 air time");
  CHECK(riderGpsAirTimeUs(6) == 7488);                                       // the announcement
  CHECK(riderGpsAirTimeUs(14) == 12864 && riderGpsAirTimeUs(15) == 12864);   // v1 and v2: 4 blocks
  CHECK(riderGpsAirTimeUs(16) == 14656 && riderGpsAirTimeUs(18) == 14656);   // 5 blocks
  CHECK(riderGpsAirTimeUs(19) == 16448);                                     // a 19th byte would cost a 6th block
  CHECK(kRiderGpsDataWaitMs == 18);
  CHECK((kRiderGpsDataWaitMs - 1u) * 1000u >= riderGpsAirTimeUs(kRiderGpsLenV3) + 2000u);
  CHECK((kRiderGpsDataWaitMs - 1u) * 1000u - riderGpsAirTimeUs(kRiderGpsLenV3) == 2344u);   // the real margin, 2.344 ms
  CHECK(!((15u - 1u) * 1000u >= riderGpsAirTimeUs(kRiderGpsLenV3) + 2000u));                // the old 15 ms would not pass the assert
}

// Unpack helper for the remote-rule tests.
static void unpack3(uint32_t w, uint16_t* c, uint16_t* s, uint8_t* a) { riderGpsVelUnpack(w, c, s, a); }

// THE REMOTE'S RULE (riderGpsTxVelocity): same epoch, HDOP (the good-fix gate), speed bound, course under 1 m/s, NaN.
static void testTxVelocity()
{
  CHECK(riderGpsVelEpochWindowMs(100) == 50 && riderGpsVelEpochWindowMs(200) == 100);   // 10 Hz, 5 Hz
  CHECK(riderGpsVelEpochWindowMs(0) == 50 && riderGpsVelEpochWindowMs(1000) == 500);
  const uint32_t W = 100;
  uint16_t c, s; uint8_t a;
  // A good fix: 5 m/s on 90 deg, speed and course 40 ms old, the position 10 ms old.
  unpack3(riderGpsTxVelocity(true, true, 5.0f, 40, true, 90.0f, 40, 10, W), &c, &s, &a);
  CHECK(s == 250 && c == 250 && a == 0);
  // SAME EPOCH: the speed may be up to W older or younger than the position, not more.
  unpack3(riderGpsTxVelocity(true, true, 5.0f, 110, true, 90.0f, 110, 10, W), &c, &s, &a);
  CHECK(s == 250 && c == 250);
  unpack3(riderGpsTxVelocity(true, true, 5.0f, 111, true, 90.0f, 111, 10, W), &c, &s, &a);
  CHECK(s == kRiderVelSpdNone && c == kRiderVelCrsNone && a == 0);   // both sentinels; the age still goes
  unpack3(riderGpsTxVelocity(true, true, 5.0f, 10, true, 90.0f, 10, 111, W), &c, &s, &a);
  CHECK(s == kRiderVelSpdNone && c == kRiderVelCrsNone && a == 2);
  // The COURSE epoch: a course one epoch older than the speed goes as its sentinel; the speed stands.
  unpack3(riderGpsTxVelocity(true, true, 5.0f, 40, true, 90.0f, 141, 10, W), &c, &s, &a);
  CHECK(s == 250 && c == kRiderVelCrsNone);
  // HDOP / GOOD FIX: no good fix, no velocity. Speed not valid: none.
  unpack3(riderGpsTxVelocity(false, true, 5.0f, 40, true, 90.0f, 40, 10, W), &c, &s, &a);
  CHECK(s == kRiderVelSpdNone && c == kRiderVelCrsNone);
  unpack3(riderGpsTxVelocity(true, false, 5.0f, 40, true, 90.0f, 40, 10, W), &c, &s, &a);
  CHECK(s == kRiderVelSpdNone && c == kRiderVelCrsNone);
  // SPEED BOUND: 20.44 m/s is the last real code; faster, negative, NaN or infinite goes as none.
  unpack3(riderGpsTxVelocity(true, true, 20.44f, 40, true, 90.0f, 40, 10, W), &c, &s, &a);
  CHECK(s == 1022 && c == 250);
  for (float bad : {20.45f, 30.0f, -0.1f, (float)NAN, (float)INFINITY}) {
    unpack3(riderGpsTxVelocity(true, true, bad, 40, true, 90.0f, 40, 10, W), &c, &s, &a);
    CHECK(s == kRiderVelSpdNone && c == kRiderVelCrsNone);
  }
  // COURSE UNDER 1.0 m/s: the speed goes, the course does not.
  unpack3(riderGpsTxVelocity(true, true, 0.9f, 40, true, 45.0f, 40, 10, W), &c, &s, &a);
  CHECK(s == 45 && c == kRiderVelCrsNone);
  unpack3(riderGpsTxVelocity(true, true, 0.0f, 40, true, 0.0f, 40, 10, W), &c, &s, &a);   // a still rider, an empty RMC course
  CHECK(s == 0 && c == kRiderVelCrsNone);
  unpack3(riderGpsTxVelocity(true, true, 1.0f, 40, true, 0.0f, 40, 10, W), &c, &s, &a);
  CHECK(s == 50 && c == 0);
  // Course NaN / infinite / not valid: none; the speed stands.
  for (float bad : {(float)NAN, (float)INFINITY}) {
    unpack3(riderGpsTxVelocity(true, true, 5.0f, 40, true, bad, 40, 10, W), &c, &s, &a);
    CHECK(s == 250 && c == kRiderVelCrsNone);
  }
  unpack3(riderGpsTxVelocity(true, true, 5.0f, 40, false, 90.0f, 40, 10, W), &c, &s, &a);
  CHECK(s == 250 && c == kRiderVelCrsNone);
  // Course encoding and wrap: never the out-of-range 1000.
  CHECK(riderGpsVelCrsCode(0.0f) == 0 && riderGpsVelCrsCode(0.36f) == 1 && riderGpsVelCrsCode(90.0f) == 250);
  CHECK(riderGpsVelCrsCode(359.8f) == 999 && riderGpsVelCrsCode(359.9f) == 0 && riderGpsVelCrsCode(360.0f) == 0);
  CHECK(riderGpsVelCrsCode(-0.5f) == 999 && riderGpsVelCrsCode(720.36f) == 1);
  for (int i = 0; i < 36000; ++i) CHECK(riderGpsVelCrsCode(i * 0.01f) <= kRiderVelCrsMaxCode);
  // AGE: floor of age / 50, 15 at 750 ms and up or unknown (TinyGPS++ 0xFFFFFFFF).
  CHECK(riderGpsVelAgeCode(0) == 0 && riderGpsVelAgeCode(49) == 0 && riderGpsVelAgeCode(50) == 1);
  CHECK(riderGpsVelAgeCode(749) == 14 && riderGpsVelAgeCode(750) == 15 && riderGpsVelAgeCode(0xFFFFFFFFu) == 15);
}

// THE BUGGY'S GATES (riderGpsVelCheck): V-1 the codes, V-2 the speed limit, V-3 the tie to the raw position track.
static void testRxVelCheck()
{
  const float kMax = 80.0f / 3.6f;   // the default gps_max_teleport_kmh, in m/s
  RiderGpsVelVerdict v;
  // No velocity sent (a v1 / v2 solution, or both sentinels): not a reject, nothing usable.
  v = riderGpsVelCheck(kRiderVelNoneW24, kMax, false, 0.0f);
  CHECK(v.verdict == RGV_NONE && !v.spd_ok && !v.crs_ok && v.age_code == 15);
  v = riderGpsVelCheck(riderGpsVelPack(1023, 1023, 4), kMax, true, 5.0f);
  CHECK(v.verdict == RGV_NONE && v.age_code == 4 && !v.v3_checked);
  // V-1: a course without a speed, or a course code 1000-1022 (with or without a speed): the whole velocity is refused.
  v = riderGpsVelCheck(riderGpsVelPack(250, 1023, 1), kMax, false, 0.0f);
  CHECK(v.verdict == RGV_V1 && !v.spd_ok && !v.crs_ok);
  for (int cbad = 1000; cbad <= 1022; ++cbad) {
    v = riderGpsVelCheck(riderGpsVelPack((uint16_t)cbad, 250, 1), kMax, false, 0.0f);
    CHECK(v.verdict == RGV_V1 && !v.spd_ok && !v.crs_ok);
    v = riderGpsVelCheck(riderGpsVelPack((uint16_t)cbad, 1023, 1), kMax, false, 0.0f);
    CHECK(v.verdict == RGV_V1);
  }
  // A real speed with no course (the remote's rule under 1 m/s): the speed stands, no course.
  v = riderGpsVelCheck(riderGpsVelPack(1023, 250, 2), kMax, false, 0.0f);
  CHECK(v.verdict == RGV_OK && v.spd_ok && fabsf(v.spd_mps - 5.0f) < 1e-4f && !v.crs_ok && v.age_code == 2);
  // Speed and course: both usable from 2.0 m/s.
  v = riderGpsVelCheck(riderGpsVelPack(250, 250, 2), kMax, false, 0.0f);
  CHECK(v.verdict == RGV_OK && v.spd_ok && v.crs_ok && fabsf(v.crs_deg - 90.0f) < 1e-3f);
  v = riderGpsVelCheck(riderGpsVelPack(250, 99, 2), kMax, false, 0.0f);    // 1.98 m/s: the course is not used, no reject
  CHECK(v.verdict == RGV_OK && v.spd_ok && !v.crs_ok);
  v = riderGpsVelCheck(riderGpsVelPack(250, 100, 2), kMax, false, 0.0f);   // 2.00 m/s
  CHECK(v.verdict == RGV_OK && v.crs_ok);
  // V-2: speed above the rider-position limit.
  v = riderGpsVelCheck(riderGpsVelPack(250, 250, 2), 5.0f, false, 0.0f);   // exactly at a 5 m/s limit
  CHECK(v.verdict == RGV_OK);
  v = riderGpsVelCheck(riderGpsVelPack(250, 251, 2), 5.0f, false, 0.0f);
  CHECK(v.verdict == RGV_V2 && !v.spd_ok && !v.crs_ok);
  v = riderGpsVelCheck(riderGpsVelPack(250, 1022, 2), kMax, false, 0.0f);  // 20.44 m/s is under the 22.2 m/s default
  CHECK(v.verdict == RGV_OK);
  // V-3: |v3 - raw| <= max(1.5 m/s, 0.4 x raw), only when a raw estimate exists.
  v = riderGpsVelCheck(riderGpsVelPack(250, 695, 2), kMax, true, 10.0f);   // 13.9 vs 10: inside 4.0
  CHECK(v.verdict == RGV_OK && v.v3_checked);
  v = riderGpsVelCheck(riderGpsVelPack(250, 705, 2), kMax, true, 10.0f);   // 14.1 vs 10: outside
  CHECK(v.verdict == RGV_V3 && !v.spd_ok && !v.crs_ok);
  v = riderGpsVelCheck(riderGpsVelPack(250, 305, 2), kMax, true, 10.0f);   // 6.1 vs 10: inside
  CHECK(v.verdict == RGV_OK);
  v = riderGpsVelCheck(riderGpsVelPack(250, 295, 2), kMax, true, 10.0f);   // 5.9 vs 10: outside
  CHECK(v.verdict == RGV_V3);
  v = riderGpsVelCheck(riderGpsVelPack(250, 170, 2), kMax, true, 2.0f);    // 3.4 vs 2: inside the 1.5 floor
  CHECK(v.verdict == RGV_OK);
  v = riderGpsVelCheck(riderGpsVelPack(250, 180, 2), kMax, true, 2.0f);    // 3.6 vs 2: outside
  CHECK(v.verdict == RGV_V3);
  v = riderGpsVelCheck(riderGpsVelPack(250, 705, 2), kMax, false, 10.0f);  // no raw estimate: V-3 not judged
  CHECK(v.verdict == RGV_OK && !v.v3_checked);
  v = riderGpsVelCheck(riderGpsVelPack(250, 250, 2), kMax, true, (float)NAN);   // a NaN raw speed fails closed
  CHECK(v.verdict == RGV_V3);
}

// PUBLISH ORDER AND THE SOLUTION TAG. Radio.ino publishes a new solution n as two stores: the word riderGpsVelTag(n, w)
// FIRST, then the solution counter = n. The loop task reads the counter, then the word. Every interleaving of the two
// writer stores with the two reads: a velocity is never given to the wrong solution, and a reader that has seen the new
// counter always gets that solution's velocity on the same read (publish-last). Wraps of the 8-bit tag included.
static void testPublishOrder()
{
  const uint32_t w_old = riderGpsVelPack(10, 20, 1), w_new = riderGpsVelPack(30, 40, 2);
  for (uint32_t n0 : {0u, 1u, 254u, 255u, 256u, 1000u, 0xFFFFFFFEu}) {
    // where each writer store lands: 0 = before the reader's first read, 1 = between its reads, 2 = after both
    for (int at_word = 0; at_word < 3; ++at_word)
      for (int at_seq = at_word; at_seq < 3; ++at_seq) {   // the word is always stored first
        uint32_t word = riderGpsVelTag(n0, w_old), seq = n0;
        if (at_word == 0) word = riderGpsVelTag(n0 + 1, w_new);
        if (at_seq  == 0) seq  = n0 + 1;
        const uint32_t r_seq = seq;                          // read 1: the counter
        if (at_word == 1) word = riderGpsVelTag(n0 + 1, w_new);
        if (at_seq  == 1) seq  = n0 + 1;
        const uint32_t r_word = word;                        // read 2: the word
        uint32_t got = 0;
        const bool ok = riderGpsVelForSeq(r_word, r_seq, &got);
        if (ok) CHECK(got == ((r_seq == n0) ? w_old : w_new));   // never another solution's velocity
        if (r_seq == n0 + 1) CHECK(ok);                           // the new counter always brings its velocity
      }
    // The word of solution n0 read against counter n0 + 1 (what the REVERSED store order would let a reader see) is
    // refused by the tag, not mis-paired.
    uint32_t got = 0;
    CHECK(!riderGpsVelForSeq(riderGpsVelTag(n0, w_old), n0 + 1, &got));
    CHECK(riderGpsVelForSeq(riderGpsVelTag(n0, w_old), n0, &got) && got == w_old);
  }
  // The initial word belongs to counter 0 and carries no velocity.
  uint32_t got = 0;
  CHECK(riderGpsVelForSeq(kRiderVelWordInit, 0, &got) && got == kRiderVelNoneW24);
}

// THE FOUR PAIRINGS (design audit 2026-10-31 §A.5), fail-safe in every one.
static void testPairings()
{
  // 1. v3 remote + package-2 buggy: the announcement promises 18 bytes; the packet parses with its counter and velocity.
  {
    CHECK(kRiderGpsAnnounceV3 == 0x05 && riderGpsDataLen(kRiderGpsAnnounceV3) == 18);
    uint8_t p[18]; const uint32_t w = riderGpsVelPack(250, 250, 1);
    riderGpsBuildV3(p, kDest, kLatV3, kLngV3, 42, w, crc8);
    RiderGpsFields f;
    CHECK(riderGpsParse(p, riderGpsDataLen(kRiderGpsAnnounceV3), crc8, &f) == RGP_OK);
    CHECK(f.has_ctr && f.ctr == 42 && riderGpsParsedVel(p, 18) == w);
    const RiderGpsVelVerdict v = riderGpsVelCheck(riderGpsParsedVel(p, 18), 80.0f / 3.6f, false, 0.0f);
    CHECK(v.verdict == RGV_OK && v.spd_ok && v.crs_ok);
    // WHETHER OR NOT A VELOCITY ARRIVES, the buggy's position path sees exactly what a v2 packet gives it: for every
    // counter and a spread of velocity words (good, none, out of range), the parsed fields equal those of the v2 packet
    // with the same position and counter, and the NEW / REPEAT / DROP classification and the commit run the same.
    for (int c = 0; c < 256; ++c)
      for (uint32_t wv : {riderGpsVelPack(250, 250, 1), kRiderVelNoneW24, riderGpsVelPack(1010, 1023, 15), 0u}) {
        uint8_t p3[18], p2[15];
        riderGpsBuildV3(p3, kDest, kLatV3 + c, kLngV3 - c, (uint8_t)c, wv, crc8);
        riderGpsBuildV2(p2, kDest, kLatV3 + c, kLngV3 - c, (uint8_t)c, crc8);
        RiderGpsFields f3 = {0, 0, false, 0}, f2 = {0, 0, false, 0};
        CHECK(riderGpsParse(p3, 18, crc8, &f3) == RGP_OK && riderGpsParse(p2, 15, crc8, &f2) == RGP_OK);
        CHECK(f3.lat_ud == f2.lat_ud && f3.lng_ud == f2.lng_ud && f3.has_ctr == f2.has_ctr && f3.ctr == f2.ctr);
        RiderGpsRxState s3 = {true, kLatV3, kLngV3, true, 7}, s2 = s3;
        CHECK(riderGpsClassify(&s3, &f3) == riderGpsClassify(&s2, &f2));
        riderGpsNoteCounter(&s3, &f3); riderGpsNoteCounter(&s2, &f2);
        CHECK(riderGpsCommit(&s3, &f3) == riderGpsCommit(&s2, &f2));
        CHECK(memcmp(&s3, &s2, sizeof(s3)) == 0);
      }
  }
  // 2. v2 remote (and v1 remote) + package-2 buggy: exactly what the package-1 buggy decides, and no velocity.
  {
    for (int s = 0; s < 256; ++s) if (s != 0x05) CHECK(riderGpsDataLen((uint8_t)s) == p1DataLen((uint8_t)s));
    for (int c = 0; c < 256; ++c) {
      uint8_t p[15]; riderGpsBuildV2(p, kDest, kLatV3 + c, kLngV3 - c, (uint8_t)c, crc8);
      RiderGpsFields a = {0, 0, false, 0}, b = {0, 0, false, 0};
      CHECK(riderGpsParse(p, 15, crc8, &a) == RGP_OK && p1Parse(p, 15, &b) == RGP_OK);
      CHECK(a.lat_ud == b.lat_ud && a.lng_ud == b.lng_ud && a.has_ctr == b.has_ctr && a.ctr == b.ctr);
      CHECK(riderGpsVelCheck(riderGpsParsedVel(p, 15), 22.2f, false, 0.0f).verdict == RGV_NONE);
      for (int i = 0; i < 15; ++i) {   // every single-byte corruption: the same verdict from both parsers
        uint8_t q[15]; memcpy(q, p, 15); q[i] ^= 0x5A;
        RiderGpsFields fa, fb;
        CHECK(riderGpsParse(q, 15, crc8, &fa) == p1Parse(q, 15, &fb));
      }
    }
    uint8_t p[14]; buildV1(p, kLatV3, kLngV3);
    RiderGpsFields a, b;
    CHECK(riderGpsParse(p, 14, crc8, &a) == RGP_OK && p1Parse(p, 14, &b) == RGP_OK);
    CHECK(a.lat_ud == b.lat_ud && a.lng_ud == b.lng_ud && !a.has_ctr && !b.has_ctr);
    CHECK(riderGpsParsedVel(p, 14) == kRiderVelNoneW24);
  }
  // 3. v3 remote + package-1 buggy (and any v1-era buggy): refused every time; the CRC lets some through by chance and
  //    the subtype refuses those.
  {
    int crc_hits = 0;
    for (int i = 0; i < 4096; ++i) {
      uint8_t p[18];
      riderGpsBuildV3(p, kDest, kLatV3 + i * 7, kLngV3 - i * 13, (uint8_t)i,
                      riderGpsVelPack((uint16_t)(i % 1000), (uint16_t)(i % 1023), (uint8_t)(i % 16)), crc8);
      RiderGpsFields f;
      CHECK(p1DataLen(kRiderGpsAnnounceV3) == 14);
      const uint8_t r = p1Parse(p, p1DataLen(kRiderGpsAnnounceV3), &f);
      if (r == RGP_TYPE) ++crc_hits;
      CHECK(r != RGP_OK);
      CHECK(!oldBuggyAccepts(p));
    }
    CHECK(crc_hits > 0);
  }
  // 4. v2 / v1 remote + old buggy: unchanged.
  {
    uint8_t p2[15]; riderGpsBuildV2(p2, kDest, kLatV3, kLngV3, 3, crc8);
    RiderGpsFields f;
    CHECK(p1Parse(p2, p1DataLen(0x03), &f) == RGP_OK && f.ctr == 3);
    uint8_t p1[14]; buildV1(p1, kLatV3, kLngV3);
    CHECK(p1Parse(p1, p1DataLen(0x01), &f) == RGP_OK && oldBuggyAccepts(p1));
  }
}

int main()
{
  testFraming();
  testTxCounter();
  testRxClassify();
  // V2.5-Evo - 2026-10-31 - v3
  testV3Framing();
  testAirTime();
  testTxVelocity();
  testRxVelCheck();
  testPublishOrder();
  testPairings();
  printf("rider_gps_packet_test: all tests passed (v1/v2 asserts as before, plus %ld v3 checks)\n", g_v3_checks);
  return 0;
}
