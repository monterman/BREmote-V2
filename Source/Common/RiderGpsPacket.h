// V2.5-Evo - 2026-10-12 - RIDER GPS PACKET v2: THE REMOTE'S GPS FIX COUNTER (audit 2026-10-11 delta A-1, owner ruling:
//   the strong version, remote + buggy). Header-only, no Arduino dependencies, so Tools/tests/rider_gps_packet_test.cpp
//   and Tools/tests/follow_me_pause_test.cpp exercise the exact code both boards run.
//
// THE BUG IT CLOSES. The buggy judged "the rider's GPS is alive" by whether the rider's COORDINATES changed. A rider
// lying still (10 Oct log, 816-821 s: 0 km/h, identical coordinates for about 5 s, packets every 0.3-0.4 s) therefore
// looked exactly like a dead GPS re-sending its last position, and after 30 s the long-loss clock switched Follow-Me
// off - silently, because the pause and the remote's not-ready look used a different rule (any packet at all).
//
// THE FIX. Every GPS packet now says which GPS SOLUTION its position belongs to: a 1-byte counter that moves once per
// NEW solution from the module - a new UTC time-of-fix committed together with a position - never per transmission and
// never per position change. A still rider with a healthy GPS keeps moving the counter; a frozen module, a lost fix or
// a re-broadcast of the last packet does not. Owner rule: a rider whose GPS has a good fix is never "lost" just because
// he is not moving.
//
// ============================================================
// WIRE FORMAT (remote -> buggy). Unchanged first step: the 6-byte announcement [dest 3][0xF3][subtype][crc8 over 0-4].
// ============================================================
//   v1 (every remote before this change): announcement subtype 0x01, then 14 bytes
//        [0-2 dest][3 0xF3][4 0x02][5-8 lat][9-12 lng][13 crc8 over 0-12]
//   v2 (this change):                     announcement subtype 0x03, then 15 bytes
//        [0-2 dest][3 0xF3][4 0x04][5-8 lat][9-12 lng][13 fix counter][14 crc8 over 0-13]
//   lat / lng: int32 microdegrees, little-endian, exactly as v1 (project GPS rule 3: never a float on the wire).
//
// AIRTIME at this link's settings (Common/RadioCommon.h: SF6, BW 250 kHz, CR 4/7, implicit header, LoRa CRC off,
// preamble 8; RadioLib 7.1.2 getTimeOnAir()): symbol 0.256 ms; payload blocks = ceil((8 x len - 24) / 24). 14 bytes
// -> 4 blocks, 15 bytes -> 4 blocks: both 50.25 symbols = 12.864 ms. THE EXTRA BYTE COSTS NOTHING ON AIR (a 16th byte
// would add a 5th block, +1.792 ms). The announcement stays 6 bytes, 7.488 ms. Rate unchanged (2 Hz).
//
// COMPATIBILITY - fail-safe in every pairing:
//   new remote + new buggy: v2, the fix-counter rule.
//   old remote + new buggy: v1, the old coordinate rule (a still rider is judged as before, but now the pause and the
//                           remote's not-ready look show it while the 30 s clock counts - one rule for all three).
//   new remote + old buggy: an old buggy reads ANY 0xF3 announcement as v1 and listens for 14 bytes. A v2 packet then
//                           fails BOTH of its tests on every packet: byte 13 (the counter) is not the CRC of bytes 0-12,
//                           and the subtype is 0x04, not 0x02. It never accepts a v2 position, so it has no rider GPS:
//                           Follow-Me and return-to-me stay not ready, manual driving is untouched. Flash the pair.
//   old remote + old buggy: unchanged.
// ============================================================
#ifndef BREMOTE_RIDER_GPS_PACKET_H
#define BREMOTE_RIDER_GPS_PACKET_H

#include <stdint.h>
#include <string.h>

static const uint8_t  kRiderGpsType       = 0xF3;   // the meta-packet type byte (byte 3) of announcement and data
static const uint8_t  kRiderGpsAnnounceV1 = 0x01;   // announcement subtype: a 14-byte v1 data packet follows
static const uint8_t  kRiderGpsAnnounceV2 = 0x03;   // announcement subtype: a 15-byte v2 data packet follows
static const uint8_t  kRiderGpsDataV1     = 0x02;   // data subtype (byte 4), v1
static const uint8_t  kRiderGpsDataV2     = 0x04;   // data subtype (byte 4), v2 - deliberately NOT 0x02 (old buggies)
static const uint8_t  kRiderGpsLenV1      = 14;     // bytes
static const uint8_t  kRiderGpsLenV2      = 15;     // bytes
static const uint8_t  kRiderGpsLenMax     = 15;     // receive buffer size
// The remote: a new time-of-fix counts as a new solution only if the position was committed in the SAME NMEA epoch -
// the position may be at most this much older than the time. A module that loses its fix keeps sending sentences with
// a fresh time and no position (TinyGPS commits the time alone), and that must not move the counter. 50 ms is below
// the 100 ms epoch of a 10 Hz module and far above the microseconds between the two commits of one sentence.
static const uint32_t kRiderGpsSameEpochMs = 50;    // ms

// The CRC both boards already use for every packet (esp_crc8() in Common/SystemCommon.h), passed in so this header
// needs no Arduino code.
typedef uint8_t (*RiderGpsCrc8Fn)(uint8_t* data, uint8_t length);

// ============================================================
// riderGpsDataLen - the data length an announcement promises.
// ============================================================
// Input: the announcement's subtype byte (byte 4). Returns: 15 for v2 (0x03), else 14 - any other 0xF3 announcement is
// v1, exactly as every buggy has read it since 2026-04-24. Side effects: none.
static inline uint8_t riderGpsDataLen(uint8_t announce_subtype)
{
  return (announce_subtype == kRiderGpsAnnounceV2) ? kRiderGpsLenV2 : kRiderGpsLenV1;
}

// ============================================================
// riderGpsBuildV2 - the remote fills one 15-byte v2 data packet.
// ============================================================
// Inputs: dest (the buggy's 3-byte address), lat_ud / lng_ud (int32 microdegrees), fix_ctr (riderGpsTxFixStep()),
// crc8. Output: out[0..14]. Side effects: none beyond out.
static inline void riderGpsBuildV2(uint8_t out[15], const uint8_t dest[3], int32_t lat_ud, int32_t lng_ud,
                                   uint8_t fix_ctr, RiderGpsCrc8Fn crc8)
{
  out[0] = dest[0]; out[1] = dest[1]; out[2] = dest[2];
  out[3] = kRiderGpsType;
  out[4] = kRiderGpsDataV2;
  memcpy(out + 5, &lat_ud, 4);   // bytes 5-8, little-endian on both boards (and on the host test)
  memcpy(out + 9, &lng_ud, 4);   // bytes 9-12
  out[13] = fix_ctr;
  out[14] = crc8(out, 14);       // CRC over bytes 0-13
}

// ============================================================
// riderGpsParse - the buggy decodes a v1 (14-byte) or v2 (15-byte) data packet. The address test stays with the caller.
// ============================================================
// Inputs: pkt, len (the length the announcement promised), crc8. Output: *f on success.
// Returns: RGP_OK, RGP_LEN (not 14 or 15), RGP_CRC (last byte is not the CRC of the rest), RGP_TYPE (byte 3 not 0xF3,
// or byte 4 not the subtype of THAT length - a v2 body is never read as v1 or the other way round).
struct RiderGpsFields {
  int32_t lat_ud;    // microdegrees
  int32_t lng_ud;    // microdegrees
  bool    has_ctr;   // true = v2: ctr below is the remote's fix counter
  uint8_t ctr;       // the fix counter (v2), 0 for v1
};
static const uint8_t RGP_OK   = 0;
static const uint8_t RGP_LEN  = 1;
static const uint8_t RGP_CRC  = 2;
static const uint8_t RGP_TYPE = 3;
static inline uint8_t riderGpsParse(uint8_t* pkt, uint8_t len, RiderGpsCrc8Fn crc8, RiderGpsFields* f)
{
  if (len != kRiderGpsLenV1 && len != kRiderGpsLenV2) return RGP_LEN;
  const uint8_t crc_at = (uint8_t)(len - 1);
  if (pkt[crc_at] != crc8(pkt, crc_at)) return RGP_CRC;
  const uint8_t want_sub = (len == kRiderGpsLenV2) ? kRiderGpsDataV2 : kRiderGpsDataV1;
  if (pkt[3] != kRiderGpsType || pkt[4] != want_sub) return RGP_TYPE;
  memcpy(&f->lat_ud, pkt + 5, 4);
  memcpy(&f->lng_ud, pkt + 9, 4);
  f->has_ctr = (len == kRiderGpsLenV2);
  f->ctr     = f->has_ctr ? pkt[13] : (uint8_t)0;
  return RGP_OK;
}

// ============================================================
// riderGpsTxFixStep - THE REMOTE'S FIX COUNTER: +1 once per NEW GPS solution.
// ============================================================
// A new solution = the module's UTC time-of-fix (TinyGPS time.value(), hhmmsscc) differs from the last counted one AND
// the position was committed in the same epoch (loc_age_ms no more than same_epoch_ms older than time_age_ms). So:
//   - the same solution sent twice (2 Hz radio off a 1 Hz module, or a module that stalls) -> no step;
//   - sentences that carry a time but no fix (TinyGPS commits the time alone) -> no step: the position is old;
//   - an invalid time -> no step (never seen with a valid position on NMEA; failing closed is the safe direction);
//   - a 5 or 10 Hz module -> one step per transmitted packet (the solutions between packets are never sent anyway).
// Inputs: *s; time_valid; fix_time; loc_age_ms / time_age_ms (TinyGPS age()s, read in the same pass); same_epoch_ms.
// Returns: the counter to send (wraps 255 -> 0; the buggy only asks "did it change"). Side effects: *s only.
struct RiderGpsTxFixState {
  bool     seen;        // last_time holds a counted solution
  uint32_t last_time;   // the time-of-fix of the last counted solution (hhmmsscc)
  uint8_t  ctr;         // the counter
};
static inline uint8_t riderGpsTxFixStep(RiderGpsTxFixState* s, bool time_valid, uint32_t fix_time,
                                        uint32_t loc_age_ms, uint32_t time_age_ms, uint32_t same_epoch_ms)
{
  const bool same_epoch = (loc_age_ms <= time_age_ms) || ((loc_age_ms - time_age_ms) <= same_epoch_ms);
  if (time_valid && same_epoch && (!s->seen || fix_time != s->last_time)) {
    s->seen      = true;
    s->last_time = fix_time;
    s->ctr       = (uint8_t)(s->ctr + 1);
  }
  return s->ctr;
}

// ============================================================
// THE BUGGY'S SIDE: is this packet a NEW solution, a re-broadcast, or something to drop?
// ============================================================
// riderGpsClassify - Inputs: *s (what the buggy last accepted / saw), *f (this packet, parsed). Returns:
//   RGF_NEW    - v2: the counter moved (or none seen yet). v1: the position differs from the last accepted (the R4-1
//                rule of 2026-08-29). The caller judges it with Phase A and, if accepted, calls riderGpsCommit().
//                A v2 solution with IDENTICAL coordinates is new: a still rider, speed 0, judged like any other.
//   RGF_REPEAT - the same solution again (v2: same counter AND same coordinates as the last accepted; v1: same
//                coordinates). No new evidence: the position is re-written and the receipt stamp refreshed exactly as
//                before, but the rider's GPS is NOT proven alive by it.
//   RGF_DROP   - v2 only: the counter repeats but the coordinates differ from the last accepted one - a remote read
//                that straddled a GPS update, or the repeat of a solution Phase A just rejected. Nothing is written.
// Side effects: none (pure). riderGpsNoteCounter() records the counter afterwards, whatever Phase A decides, so the
// repeat of a rejected solution is not judged a second time.
struct RiderGpsRxState {
  bool    seeded;    // lat_ud / lng_ud hold the last ACCEPTED solution
  int32_t lat_ud;
  int32_t lng_ud;
  bool    ctr_seen;  // ctr holds the counter of the last v2 packet that parsed
  uint8_t ctr;
};
static const uint8_t RGF_DROP   = 0;
static const uint8_t RGF_REPEAT = 1;
static const uint8_t RGF_NEW    = 2;
static inline uint8_t riderGpsClassify(const RiderGpsRxState* s, const RiderGpsFields* f)
{
  const bool same_pos = s->seeded && (f->lat_ud == s->lat_ud) && (f->lng_ud == s->lng_ud);
  if (f->has_ctr) {
    if (!s->ctr_seen || f->ctr != s->ctr) return RGF_NEW;
    return same_pos ? RGF_REPEAT : RGF_DROP;
  }
  return same_pos ? RGF_REPEAT : RGF_NEW;
}

// riderGpsNoteCounter - remember this packet's counter (v2) or forget it (v1: a v1 packet ends any v2 run, so the next
// v2 packet is judged fresh). Call for every packet that parsed, before Phase A. Side effects: *s only.
static inline void riderGpsNoteCounter(RiderGpsRxState* s, const RiderGpsFields* f)
{
  s->ctr_seen = f->has_ctr;
  s->ctr      = f->has_ctr ? f->ctr : (uint8_t)0;
}

// riderGpsCommit - an ACCEPTED new solution becomes the reference. Returns true when its position differs from the
// previous reference (or there was none): the separation proof's DISTINCT-position counter (rx_tx_gps_fix_seq) moves
// only then, so the proof still needs real distinct positions, exactly as before. Side effects: *s only.
static inline bool riderGpsCommit(RiderGpsRxState* s, const RiderGpsFields* f)
{
  const bool distinct = !s->seeded || (f->lat_ud != s->lat_ud) || (f->lng_ud != s->lng_ud);
  s->seeded = true;
  s->lat_ud = f->lat_ud;
  s->lng_ud = f->lng_ud;
  return distinct;
}

#endif // BREMOTE_RIDER_GPS_PACKET_H
