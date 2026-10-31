// V2.5-Evo - 2026-10-31 - RIDER GPS PACKET v3: THE REMOTE'S OWN GPS COURSE AND SPEED (design audit 2026-10-31, Follow-Me
//   package 2 unit A - LOGGING ONLY). The remote already decodes its speed and course from the RMC sentence on every fix
//   (its speed display uses them); they were simply never sent. v3 adds them in 3 bytes (an 18-byte packet, one more LoRa
//   block, +1.792 ms on air), with the fix's age at send. The buggy stores them behind plausibility gates and shows them
//   in ?diag and the level-6 log; NOTHING in Follow-Me, return-to-me or the motor path reads them yet. New here: the v3
//   constants, riderGpsBuildV3(), riderGpsParse() accepting 18 bytes, riderGpsAirTimeUs() and the remote's post-data wait
//   (finding A-T1), the velocity word (riderGpsVelPack / Unpack), the remote's rule riderGpsTxVelocity(), the buggy's
//   publish tag (riderGpsVelTag / riderGpsVelForSeq) and its plausibility gates riderGpsVelCheck() (V-1 to V-3). The v1
//   and v2 parsing branches are unchanged. Header-only, no Arduino dependencies (rider_gps_packet_test.cpp).
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
//   v2 (2026-10-12):                      announcement subtype 0x03, then 15 bytes
//        [0-2 dest][3 0xF3][4 0x04][5-8 lat][9-12 lng][13 fix counter][14 crc8 over 0-13]
//   v3 (V2.5-Evo - 2026-10-31):           announcement subtype 0x05, then 18 bytes
//        [0-2 dest][3 0xF3][4 0x06][5-8 lat][9-12 lng][13 fix counter][14-16 velocity word][17 crc8 over 0-16]
//        The data subtype is 0x06 - deliberately neither 0x02 nor 0x04 - so that NO older buggy can ever take a v3 body
//        for its own format, whatever the CRC does (see COMPATIBILITY).
//   lat / lng: int32 microdegrees, little-endian, exactly as v1 (project GPS rule 3: never a float on the wire).
//   The velocity word (v3 only, bytes 14-16) is ONE 24-bit little-endian word w = crs10 | (spd10 << 10) | (age4 << 20):
//     crs10  course over ground, 0.36 deg per count, 0-999 (0-359.64 deg). 1023 = no course; 1000-1022 are read as no
//            course too (fail closed). The remote sends no course below 1.0 m/s.
//     spd10  speed over ground, 0.02 m/s per count, 0-1022 (0-20.44 m/s, 45.7 mph). 1023 = no speed.
//     age4   the fix's age when the packet was built, 50 ms per count, 0-14 (0-749 ms). 15 = 750 ms or more, or unknown.
//   "No data" is never 0 (project rule: zero is the zero-initialisation value and would pass a "> 0" test).
//
// AIRTIME at this link's settings (Common/RadioCommon.h: SF6, BW 250 kHz, CR 4/7, implicit header, LoRa CRC off,
// preamble 8; RadioLib 7.1.2 getTimeOnAir()): symbol 0.256 ms; payload blocks = ceil((8 x len - 24) / 24). 14 bytes
// -> 4 blocks, 15 bytes -> 4 blocks: both 50.25 symbols = 12.864 ms. THE EXTRA BYTE COSTS NOTHING ON AIR (a 16th byte
// would add a 5th block, +1.792 ms). The announcement stays 6 bytes, 7.488 ms. Rate unchanged (2 Hz).
// V2.5-Evo - 2026-10-31 - v3: 18 bytes is the LARGEST packet that still fits in 5 blocks (57.25 symbols = 14.656 ms), so
// the 3 velocity bytes cost one block, +1.792 ms per GPS packet, about +0.36 % of the channel time at 2 Hz. The remote's
// wait after the data packet is sized from this (riderGpsAirTimeUs(), kRiderGpsDataWaitMs: finding A-T1).
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
// V2.5-Evo - 2026-10-31 - v3, the same rule one step on (all four pairings):
//   v3 remote + package-2 buggy: v3 - the velocity is stored where its gates pass (logging only in unit A).
//   v2 remote + package-2 buggy: the v2 path is byte-identical; there is no velocity, so nothing changes.
//   v3 remote + v2 (package 1) or v1 buggy: such a buggy reads announcement 0x05 as v1 and listens for 14 bytes. The
//                           first 4 LoRa blocks of a v3 body decode exactly as they were sent, so it gets bytes 0-13.
//                           Byte 13 (the fix counter) equals the CRC of bytes 0-12 by chance 1 time in 256 - and then
//                           byte 4 is 0x06, not 0x02, so the packet is REJECTED, DETERMINISTICALLY, never by CRC luck.
//                           No rider GPS: Follow-Me and return-to-me show not ready, manual driving is untouched. The
//                           5th block is still on air for 1.8 ms after that buggy's RxDone; it carries no preamble, so it
//                           can never be taken for a packet.
//   v2 / v1 remote + old buggy: unchanged.
// ============================================================
#ifndef BREMOTE_RIDER_GPS_PACKET_H
#define BREMOTE_RIDER_GPS_PACKET_H

#include <stdint.h>
#include <string.h>
#include <math.h>   // V2.5-Evo - 2026-10-31 - v3: isfinite() / lroundf() for the velocity rules

static const uint8_t  kRiderGpsType       = 0xF3;   // the meta-packet type byte (byte 3) of announcement and data
static const uint8_t  kRiderGpsAnnounceV1 = 0x01;   // announcement subtype: a 14-byte v1 data packet follows
static const uint8_t  kRiderGpsAnnounceV2 = 0x03;   // announcement subtype: a 15-byte v2 data packet follows
static const uint8_t  kRiderGpsAnnounceV3 = 0x05;   // V2.5-Evo - 2026-10-31 - announcement subtype: an 18-byte v3 data packet follows
static const uint8_t  kRiderGpsDataV1     = 0x02;   // data subtype (byte 4), v1
static const uint8_t  kRiderGpsDataV2     = 0x04;   // data subtype (byte 4), v2 - deliberately NOT 0x02 (old buggies)
static const uint8_t  kRiderGpsDataV3     = 0x06;   // V2.5-Evo - 2026-10-31 - data subtype (byte 4), v3 - deliberately neither 0x02 nor 0x04
static const uint8_t  kRiderGpsLenV1      = 14;     // bytes
static const uint8_t  kRiderGpsLenV2      = 15;     // bytes
static const uint8_t  kRiderGpsLenV3      = 18;     // V2.5-Evo - 2026-10-31 - bytes (the largest packet that fits in 5 LoRa blocks)
static const uint8_t  kRiderGpsLenMax     = 18;     // receive buffer size (V2.5-Evo - 2026-10-31 - v3: was 15)
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
// V2.5-Evo - 2026-10-31 - v3: 18 for 0x05. Every other subtype keeps its old answer.
static inline uint8_t riderGpsDataLen(uint8_t announce_subtype)
{
  if (announce_subtype == kRiderGpsAnnounceV3) return kRiderGpsLenV3;   // V2.5-Evo - 2026-10-31 - v3
  return (announce_subtype == kRiderGpsAnnounceV2) ? kRiderGpsLenV2 : kRiderGpsLenV1;
}

// ============================================================
// V2.5-Evo - 2026-10-31 - AIR TIME AND THE REMOTE'S WAIT AFTER THE DATA PACKET (finding A-T1)
// ============================================================
// riderGpsAirTimeUs - the time a packet of len bytes spends on air at THIS link's settings (SF6, BW 250 kHz, CR 4/7,
// implicit header, LoRa CRC off, preamble 8 - Common/RadioCommon.h and setRadioActivityEnabled()), in microseconds.
// RadioLib 7.1.2's getTimeOnAir() for SF5/SF6: symbols = preamble 8 + 6.25 + 8 + 7 x ceil((8 x len - 24) / 24), one
// symbol = 2^6 / 250 kHz = 256 us. Counted in quarter symbols (89 + 28 per block) so it stays an exact integer.
// 6 B -> 7488 us, 14 / 15 B -> 12864 us, 18 B -> 14656 us. Valid for len >= 3. Pure; usable in a static_assert.
static constexpr uint32_t riderGpsAirTimeUs(uint32_t len)
{
  return (89u + 28u * ((8u * len - 24u + 23u) / 24u)) * 64u;
}
// THE BUG IT CLOSES (finding A-T1, design-time). The remote waited vTaskDelay(15) after starting the data packet - sized
// for the 12.864 ms v2 packet. vTaskDelay(15) waits 14-15 ms, and a v3 packet needs 14.656 ms on air plus the radio's
// SetTx start-up, so the next implicitHeader(6) / startReceive() could cut the 5th block: CRC fail at the buggy, and a
// new pair would have NO rider GPS at all. THE FIX: the wait is ceil(air time) + 3 ms = 18 ms (the GPS cycle then uses
// about 30 ms of its 100 ms slot), and the compiler refuses any value shorter than that - so the old 15 cannot come back.
static const uint32_t kRiderGpsDataWaitMs = 18;   // ms, the remote's wait after it starts the v3 data packet
static_assert(kRiderGpsDataWaitMs * 1000u >= riderGpsAirTimeUs(kRiderGpsLenV3) + 3000u,
              "the remote's post-data wait must cover the v3 packet's air time plus 3 ms (finding A-T1)");

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
// V2.5-Evo - 2026-10-31 - THE VELOCITY WORD (v3, bytes 14-16): constants, pack and unpack
// ============================================================
// Layout and units in the WIRE FORMAT block above. The sentinels are the all-ones codes, never 0.
static const uint16_t kRiderVelCrsNone    = 1023;      // course code: no course
static const uint16_t kRiderVelCrsMaxCode = 999;       // course codes 0-999 are real (0.36 deg each); 1000-1022 = no course
static const uint16_t kRiderVelSpdNone    = 1023;      // speed code: no speed
static const uint16_t kRiderVelSpdMaxCode = 1022;      // speed codes 0-1022 are real (0.02 m/s each)
static const uint8_t  kRiderVelAgeNone    = 15;        // age code: 750 ms or more, or unknown
static const float    kRiderVelCrsDegPerCode = 0.36f;  // deg per course count
static const float    kRiderVelSpdMpsPerCode = 0.02f;  // m/s per speed count
static const float    kRiderVelSpdMaxMps  = 20.44f;    // m/s; faster is implausible here and goes as "no speed"
static const uint32_t kRiderVelAgeMsPerCode = 50;      // ms per age count
static const float    kRiderVelCrsMinTxMps = 1.0f;     // m/s; the remote sends no course below this (meaningless that slow,
                                                       // and TinyGPS++ may commit an EMPTY RMC course field as a "valid" 0.00)
static const uint32_t kRiderVelNoneW24    = 0xFFFFFFu; // the 24-bit word with all three sentinels (no velocity, age unknown)

// riderGpsVelPack - three codes -> the 24-bit word. Each code is masked to its own width. Pure.
static inline uint32_t riderGpsVelPack(uint16_t crs, uint16_t spd, uint8_t age)
{
  return ((uint32_t)(crs & 0x3FFu)) | ((uint32_t)(spd & 0x3FFu) << 10) | ((uint32_t)(age & 0x0Fu) << 20);
}
// riderGpsVelUnpack - the 24-bit word (any higher bits ignored) -> three codes. Pure.
static inline void riderGpsVelUnpack(uint32_t w, uint16_t *crs, uint16_t *spd, uint8_t *age)
{
  *crs = (uint16_t)(w & 0x3FFu);
  *spd = (uint16_t)((w >> 10) & 0x3FFu);
  *age = (uint8_t)((w >> 20) & 0x0Fu);
}

// ============================================================
// V2.5-Evo - 2026-10-31 - riderGpsBuildV3 - the remote fills one 18-byte v3 data packet.
// ============================================================
// Inputs: dest, lat_ud / lng_ud (int32 microdegrees), fix_ctr (riderGpsTxFixStep()), vel_w24 (riderGpsTxVelocity()),
// crc8. Output: out[0..17]. Bytes 0-13 are laid out exactly as v2 apart from the subtype (byte 4 = 0x06). Side effects:
// none beyond out.
static inline void riderGpsBuildV3(uint8_t out[18], const uint8_t dest[3], int32_t lat_ud, int32_t lng_ud,
                                   uint8_t fix_ctr, uint32_t vel_w24, RiderGpsCrc8Fn crc8)
{
  out[0] = dest[0]; out[1] = dest[1]; out[2] = dest[2];
  out[3] = kRiderGpsType;
  out[4] = kRiderGpsDataV3;
  memcpy(out + 5, &lat_ud, 4);   // bytes 5-8, little-endian
  memcpy(out + 9, &lng_ud, 4);   // bytes 9-12
  out[13] = fix_ctr;
  out[14] = (uint8_t)(vel_w24 & 0xFFu);           // the velocity word, little-endian
  out[15] = (uint8_t)((vel_w24 >> 8) & 0xFFu);
  out[16] = (uint8_t)((vel_w24 >> 16) & 0xFFu);
  out[17] = crc8(out, 17);       // CRC over bytes 0-16
}

// ============================================================
// riderGpsParse - the buggy decodes a v1 (14-byte) or v2 (15-byte) data packet. The address test stays with the caller.
// ============================================================
// Inputs: pkt, len (the length the announcement promised), crc8. Output: *f on success.
// Returns: RGP_OK, RGP_LEN (not 14 or 15), RGP_CRC (last byte is not the CRC of the rest), RGP_TYPE (byte 3 not 0xF3,
// or byte 4 not the subtype of THAT length - a v2 body is never read as v1 or the other way round).
// V2.5-Evo - 2026-10-31 - v3: an 18-byte packet with subtype 0x06 parses too (has_ctr true, the counter in byte 13, as
// v2); its velocity word is read with riderGpsParsedVel(). Each length still accepts ONLY its own subtype, so no body of
// one version can ever pass as another. The v1 and v2 branches are unchanged.
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
  if (len != kRiderGpsLenV1 && len != kRiderGpsLenV2 && len != kRiderGpsLenV3) return RGP_LEN;   // V2.5-Evo - 2026-10-31 - + v3
  const uint8_t crc_at = (uint8_t)(len - 1);
  if (pkt[crc_at] != crc8(pkt, crc_at)) return RGP_CRC;
  const uint8_t want_sub = (len == kRiderGpsLenV3) ? kRiderGpsDataV3          // V2.5-Evo - 2026-10-31 - v3
                         : (len == kRiderGpsLenV2) ? kRiderGpsDataV2 : kRiderGpsDataV1;
  if (pkt[3] != kRiderGpsType || pkt[4] != want_sub) return RGP_TYPE;
  memcpy(&f->lat_ud, pkt + 5, 4);
  memcpy(&f->lng_ud, pkt + 9, 4);
  f->has_ctr = (len == kRiderGpsLenV2) || (len == kRiderGpsLenV3);   // V2.5-Evo - 2026-10-31 - v3 carries the counter too
  f->ctr     = f->has_ctr ? pkt[13] : (uint8_t)0;
  return RGP_OK;
}

// V2.5-Evo - 2026-10-31 - riderGpsParsedVel - the velocity word of a packet riderGpsParse() has just accepted: bytes
// 14-16 of an 18-byte v3 packet, or kRiderVelNoneW24 (no velocity) for v1 / v2. Call only after RGP_OK. Pure.
static inline uint32_t riderGpsParsedVel(const uint8_t *pkt, uint8_t len)
{
  if (len != kRiderGpsLenV3) return kRiderVelNoneW24;
  return (uint32_t)pkt[14] | ((uint32_t)pkt[15] << 8) | ((uint32_t)pkt[16] << 16);
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

// ============================================================
// V2.5-Evo - 2026-10-31 - THE REMOTE'S SIDE OF v3: which course and speed go into the packet
// ============================================================
// riderGpsVelEpochWindowMs - W_v, how far apart the speed's and the position's commit times may be and still be one GPS
// epoch: max(kRiderGpsSameEpochMs, meas_ms / 2) = 50 ms at 10 Hz, 100 ms at 5 Hz. TinyGPS++ commits the position on
// BOTH GGA and RMC but the speed and course on RMC only, so the position is normally the GGA's (a few ms to ~75 ms at
// 9600 baud after the RMC); this window covers that spacing. Input: the module's measurement period in ms. Pure.
static inline uint32_t riderGpsVelEpochWindowMs(uint32_t meas_ms)
{
  const uint32_t half = meas_ms / 2u;
  return (half > kRiderGpsSameEpochMs) ? half : kRiderGpsSameEpochMs;
}
// riderGpsAgeDiffMs - |a - b| for two TinyGPS++ ages (unsigned, either may be the larger). Pure.
static inline uint32_t riderGpsAgeDiffMs(uint32_t a, uint32_t b) { return (a > b) ? (a - b) : (b - a); }
// riderGpsVelAgeCode - a fix age in ms -> its 4-bit code (floor of age / 50; 15 = 750 ms or more, or unknown - TinyGPS++
// returns 0xFFFFFFFF for a value never seen). Pure.
static inline uint8_t riderGpsVelAgeCode(uint32_t age_ms)
{
  const uint32_t c = age_ms / kRiderVelAgeMsPerCode;
  return (c >= kRiderVelAgeNone) ? kRiderVelAgeNone : (uint8_t)c;
}
// riderGpsVelCrsCode - a course in degrees (finite) -> 0-999. Wrapped into 0-360 first; a course that rounds to 1000
// (359.82 deg and up) is 0 deg, never the out-of-range 1000. Pure.
static inline uint16_t riderGpsVelCrsCode(float deg)
{
  float d = fmodf(deg, 360.0f);
  if (d < 0.0f) d += 360.0f;
  long c = lroundf(d / kRiderVelCrsDegPerCode);
  if (c >= 1000) c -= 1000;
  if (c < 0) c = 0;
  return (uint16_t)c;
}
// riderGpsVelSpdCode - a speed in m/s (finite, 0 to kRiderVelSpdMaxMps) -> 0-1022. Pure.
static inline uint16_t riderGpsVelSpdCode(float mps)
{
  long c = lroundf(mps * 50.0f);   // 1 / 0.02 m/s per count
  if (c < 0) c = 0;
  if (c > (long)kRiderVelSpdMaxCode) c = kRiderVelSpdMaxCode;
  return (uint16_t)c;
}

// ============================================================
// riderGpsTxVelocity - the remote's velocity word for the packet it is about to send (design audit 2026-10-31 §A.3)
// ============================================================
// What it does: decides whether this fix's speed and course may go on air and encodes them with the fix's age.
// THE SPEED (and with it any course) is sent only when ALL of these hold, else both sentinels:
//   - good_fix: the remote's own trustworthy-fix gate passes (txGpsGoodFix(): location and speed valid, fresh, and the
//     gps_max_hdop gate - the HDOP gate the remote's positions have never had; positions are still sent exactly as before);
//   - the speed is valid, finite, not negative and at most kRiderVelSpdMaxMps (20.44 m/s);
//   - SAME EPOCH as the position: |speed age - position age| <= epoch_ms (riderGpsVelEpochWindowMs()). A speed from an
//     older sentence than the position - a fix lost and regained, or this read landing between two commits of the
//     parser - is not sent.
// THE COURSE is sent only when the speed is, AND it is valid and finite, AND the speed is at least kRiderVelCrsMinTxMps
//   (1.0 m/s: below that a course means nothing, and TinyGPS++ may commit an EMPTY RMC course field as a "valid" 0.00),
//   AND its own age is within epoch_ms of the speed's (TinyGPS++ commits the speed, then the course: a read between the
//   two would pair a new speed with the previous epoch's course). Otherwise the course alone goes as its sentinel.
// THE AGE is always the position fix's age at send (loc_age_ms), the remote-side delay of the very fix being sent.
// Inputs: the TinyGPS++ values read in ONE pass in the radio task, which the parser (loop task) cannot interrupt.
// Returns: the 24-bit word (riderGpsVelPack()). Side effects: none.
static inline uint32_t riderGpsTxVelocity(bool good_fix, bool spd_valid, float spd_mps, uint32_t spd_age_ms,
                                          bool crs_valid, float crs_deg, uint32_t crs_age_ms,
                                          uint32_t loc_age_ms, uint32_t epoch_ms)
{
  const uint8_t age = riderGpsVelAgeCode(loc_age_ms);
  uint16_t spd = kRiderVelSpdNone, crs = kRiderVelCrsNone;
  const bool spd_ok = good_fix && spd_valid && isfinite(spd_mps) && spd_mps >= 0.0f && spd_mps <= kRiderVelSpdMaxMps &&
                      riderGpsAgeDiffMs(spd_age_ms, loc_age_ms) <= epoch_ms;
  if (spd_ok) {
    spd = riderGpsVelSpdCode(spd_mps);
    const bool crs_ok = crs_valid && isfinite(crs_deg) && spd_mps >= kRiderVelCrsMinTxMps &&
                        riderGpsAgeDiffMs(crs_age_ms, spd_age_ms) <= epoch_ms;
    if (crs_ok) crs = riderGpsVelCrsCode(crs_deg);
  }
  return riderGpsVelPack(crs, spd, age);
}

// ============================================================
// V2.5-Evo - 2026-10-31 - THE BUGGY'S SIDE OF v3: publishing, and the plausibility gates
// ============================================================
// PUBLISH-LAST WITH AN OWNER TAG. The radio task stores the velocity of each accepted NEW solution as ONE aligned 32-bit
// word, (that solution's sequence number & 0xFF) << 24 | w24, BEFORE it bumps the solution counter
// (rx_tx_gps_solution_seq). A reader reads the counter FIRST, then the word, and takes the velocity only if the word's
// tag matches the counter it read (riderGpsVelForSeq()). So it can never pair one solution's position with another
// solution's velocity, and a 32-bit word cannot be torn on this single-core part: a reader that lands between the two
// stores sees the old counter with the new word - a tag mismatch - and simply waits one tick.
// A REPEAT (the same solution again) writes no word: it carries no new velocity. A solution Phase A rejects writes no
// word either: its velocity goes with its position. A v1 / v2 solution publishes kRiderVelNoneW24 (no velocity).
static const uint32_t kRiderVelWordInit = kRiderVelNoneW24;   // tag 0 = solution counter 0 (none yet), no velocity

// riderGpsVelTag - the published word for solution number solution_seq. Pure.
static inline uint32_t riderGpsVelTag(uint32_t solution_seq, uint32_t w24)
{
  return ((solution_seq & 0xFFu) << 24) | (w24 & 0xFFFFFFu);
}
// riderGpsVelForSeq - true (and *w24 set) when the published word belongs to solution solution_seq; false = it does not
// (no velocity for that solution on this read). Pure.
static inline bool riderGpsVelForSeq(uint32_t word, uint32_t solution_seq, uint32_t *w24)
{
  if (((word >> 24) & 0xFFu) != (solution_seq & 0xFFu)) return false;
  *w24 = word & 0xFFFFFFu;
  return true;
}

// riderGpsVelCheck - THE PLAUSIBILITY GATES (design audit 2026-10-31 §A.4). A failure invalidates the VELOCITY ONLY: the
// position it came with was judged by Phase A and stays exactly as accepted, so a velocity failure can never age the
// rider, pause Follow-Me or start the long-loss clock.
//   V-1 THE CODES. Each code must be a real value or its own sentinel, and a course never comes without a speed (the
//       remote sends both sentinels when it has no speed). A course code 1000-1022 cannot come from a remote: it fails
//       closed, and since its top bits share byte 15 with the speed, the WHOLE velocity is refused. A course sentinel with
//       a real speed is normal (the remote sends no course under 1.0 m/s): the speed stands, the course does not. The
//       course is USED only at a v3 speed of at least kRiderVelCrsMinUseMps (2.0 m/s, 4.5 mph); below that crs_ok is false
//       without a reject.
//   V-2 THE SPEED LIMIT. speed <= max_speed_mps (the caller passes gps_max_teleport_kmh / 3.6, the rider-position limit).
//   V-3 THE TIE TO THE POSITION TRACK. When the buggy's raw rider track has a speed estimate (raw_ok - fmCvRiderMotion()),
//       |v3 speed - raw speed| <= max(1.5 m/s, 0.4 x raw speed). Speed only: a course test would fail falsely in a 90 deg/s
//       turn, where the 1 s chord lags by 45 deg. With no raw estimate V-3 is not judged (v3_checked false).
// Inputs: w24; max_speed_mps; raw_ok / raw_mps. Returns the verdict. Side effects: none.
static const uint8_t RGV_OK   = 0;   // the speed is good (and the course too when crs_ok)
static const uint8_t RGV_NONE = 1;   // the remote sent no velocity (both sentinels, or a v1 / v2 solution) - not a reject
static const uint8_t RGV_V1   = 2;   // refused by V-1 (a code out of range, or a course without a speed)
static const uint8_t RGV_V2   = 3;   // refused by V-2 (faster than the rider-position limit)
static const uint8_t RGV_V3   = 4;   // refused by V-3 (disagrees with the raw position track)
static const float   kRiderVelCrsMinUseMps = 2.0f;   // m/s; V-1: the course is used only at or above this
static const float   kRiderVelV3AbsMps     = 1.5f;   // m/s; V-3 tolerance floor
static const float   kRiderVelV3Frac       = 0.4f;   // V-3 tolerance, fraction of the raw speed
struct RiderGpsVelVerdict {
  uint8_t verdict;     // RGV_*
  bool    spd_ok;      // spd_mps is a usable speed
  float   spd_mps;     // m/s (0 when !spd_ok)
  bool    crs_ok;      // crs_deg is a usable course
  float   crs_deg;     // degrees, 0-359.64 (0 when !crs_ok)
  uint8_t age_code;    // the fix's age at send, 50 ms per count; 15 = 750 ms or more / unknown
  bool    v3_checked;  // V-3 was judged (a raw estimate existed)
};
static inline RiderGpsVelVerdict riderGpsVelCheck(uint32_t w24, float max_speed_mps, bool raw_ok, float raw_mps)
{
  RiderGpsVelVerdict v = { RGV_NONE, false, 0.0f, false, 0.0f, kRiderVelAgeNone, false };
  uint16_t crs = 0, spd = 0; uint8_t age = 0;
  riderGpsVelUnpack(w24, &crs, &spd, &age);
  v.age_code = age;
  const bool crs_real = (crs <= kRiderVelCrsMaxCode);
  const bool crs_bad  = !crs_real && (crs != kRiderVelCrsNone);   // 1000-1022
  if (spd == kRiderVelSpdNone) {                                    // no speed sent
    v.verdict = (crs_real || crs_bad) ? RGV_V1 : RGV_NONE;          // a course without a speed is not a remote's packet
    return v;
  }
  if (spd > kRiderVelSpdMaxCode || crs_bad) { v.verdict = RGV_V1; return v; }   // (spd > 1022 cannot occur: 1023 is caught above)
  const float s = (float)spd * kRiderVelSpdMpsPerCode;
  if (!(s <= max_speed_mps)) { v.verdict = RGV_V2; return v; }
  if (raw_ok) {
    v.v3_checked = true;
    const float tol = (kRiderVelV3Frac * raw_mps > kRiderVelV3AbsMps) ? kRiderVelV3Frac * raw_mps : kRiderVelV3AbsMps;
    if (!(fabsf(s - raw_mps) <= tol)) { v.verdict = RGV_V3; return v; }   // NaN fails closed
  }
  v.verdict = RGV_OK;
  v.spd_ok  = true;
  v.spd_mps = s;
  if (crs_real && s >= kRiderVelCrsMinUseMps) {
    v.crs_ok  = true;
    v.crs_deg = (float)crs * kRiderVelCrsDegPerCode;
  }
  return v;
}

#endif // BREMOTE_RIDER_GPS_PACKET_H
