// V2.5-Evo - 2026-10-12 - Host test for Common/RiderGpsPacket.h (audit 2026-10-11 delta A-1): the v1 / v2 framing both
//   ways, the old buggy's view of a v2 packet, the remote's fix counter (one step per NEW solution, never per send, never
//   per position change, never on a time-only sentence) and the buggy's NEW / REPEAT / DROP classification.
// Build and run:  g++ -std=c++17 -Wall -Wextra -o rider_gps_packet_test rider_gps_packet_test.cpp && ./rider_gps_packet_test

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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

// SYNTHETIC test data: made-up values, not from any real board.
static const uint8_t kDest[3] = { 0xA1, 0xB2, 0xC3 };

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
  for (int s = 0; s < 256; ++s) if (s != 0x03) assert(riderGpsDataLen((uint8_t)s) == 14);

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

int main()
{
  testFraming();
  testTxCounter();
  testRxClassify();
  printf("rider_gps_packet_test: all tests passed\n");
  return 0;
}
