// V2.5-Evo - 2026-10-20 - REMOTE STEP (TX SW29, step 3): new section 17 for the remote - the activity repeat (X-1 with the
//   owner's widening: any button or toggle press while in control), the E9-or-St choice for a refused gesture, and the
//   ownership-dot mapping (24a0371). The receiver gains released_rebind (step-2 audit Z-7 a), and hook rule (f) states
//   that the claimable edge is evaluated after the reply (Z-7 b). Host-tested (multi_tx_test.cpp T34-T36).
// V2.5-Evo - 2026-10-20 - RECEIVER STEP (RX SW37): the receiver now includes this header (through Common/
//   SPIFFSEngine.h, for efuseIdFromMac(), and BREmote_V2_Rx.h) and runs mtxOnFrame() behind its hook table in
//   Radio.ino. MTX_REL_REBIND added. The remote still only gets efuseIdFromMac() from it, via SPIFFSEngine.h.
// V2.5-Evo - 2026-10-20 - STEP 2 HEADER FIXES (step-1 code audit Y-1..Y-5, Y-7 b/c): the receiver hook contract is
//   written out on MtxRxHooks, with mtxRxHooksComplete() for the boot check (Y-1); mtxPairOnConfirm() refuses a fifth
//   enrolment or a closed window (Y-2); mtxTimers() expires the per-slot zero proof and "control heard" stamps after
//   3 s, so neither can alias as "just now" after 24.8 days of uptime (Y-3); the claim and summon lines are emitted
//   after the release and the grant (Y-4); a master take-back leaves a claim notice (Y-7 b); the owner's own SEIZE or
//   SUMMON frame restarts its silence clock, as any owner frame does (Y-7 c). Host-tested (multi_tx_test.cpp).
// V2.5-Evo - 2026-10-20 - MULTI-REMOTE v3.2, CODE STEP 1: the pure model of one buggy and up to four remotes (claim
//   model, summon, master take-back, takeover limit). Header-only, no Arduino code, and NOT included by any firmware
//   file yet: that starts with the receiver step. So no board behaviour changes. Host-tested by
//   Tools/tests/multi_tx_test.cpp (T1-T18, T20-T33) and Tools/tests/config_migrate_test.cpp (T19).
//   The design is docs/MULTI_TX_DESIGN.md (v3.2). Section numbers below ("design 7.2") point into the design notes.
//
// ============================================================
// WHAT THIS HEADER IS
// ============================================================
// Every function here is a DECISION or a fixed ORDER of steps. None of them touches a radio, a pin, a task or a
// global: the firmware passes its state in, and for the receiver's per-packet rules it passes a table of small
// functions (MtxRxHooks) that do the real work - zero the throttle, run the existing RTM / Follow-Me handlers, send the
// reply. So the ORDER of a release, a grant and a claim lives here, once, and the host test runs that same order.
//
// THE SAFETY ARGUMENT IN ONE PARAGRAPH. Only the owner's control frame reaches apply_owner(), the one place that
// writes the throttle and stamps the motor failsafe. Control passes only from zero throttle (zero proof read BEFORE
// this frame updates it), never while Follow-Me or Return-To-Me is armed, never before the motor output runs, and the
// takeover limit is published before the new owner's first byte. A release zeroes the throttle FIRST and stamps
// nothing, so the motor gate shuts on its own clock. No timer releases anything. With one remote paired (SINGLE)
// every frame takes today's path unchanged.
//
// Single-core ESP32-C3: the receiver's ownership state has one writer, the radio task. The PWM task reads the grant
// stamp, the loop task reads the owner and the change counter - those are the firmware's atomics, not this header's.
// ============================================================
#ifndef BREMOTE_MULTI_TX_H
#define BREMOTE_MULTI_TX_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// ============================================================
// 1. CONSTANTS - compile-time, not settings
// ============================================================
static const uint8_t  kMtxSlots          = 4;      // remotes one buggy can list (slot 0 = the master, paired first)
static const int8_t   kMtxNone           = -1;     // "nobody owns" / "no slot"
static const int8_t   kMtxSlotUnknown    = -1;     // mtxLookupSlot(): no paired remote sent this frame
static const int8_t   kMtxSlotGate1      = -2;     // mtxLookupSlot(): address 00:00:00, dropped before the lookup
static const int8_t   kMtxSlotImplicit   = 4;      // mtxLookupSlot(): the implicit legacy entry of a migrated board
static const uint8_t  kMtxTelemetryLen   = 20;     // TelemetryPacket indices 0-19 (unchanged)

// Throttle and steering bytes (0-255 counts; steering centre 127).
static const uint8_t  kMtxZeroThr        = 8;      // a throttle byte below this is "at zero"
static const uint8_t  kMtxInputThr       = 12;     // a throttle byte at or above this is a squeeze (grant, claim, E9)
static const uint8_t  kMtxSteerCentre    = 127;
static const uint8_t  kMtxSteerInput     = 24;     // steering this far off centre is input
static const uint8_t  kMtxZeroPkts       = 2;      // zero packets needed before a grant, a claim or a take-back

// Timing (ms).
static const uint32_t kMtxPresentMs      = 3000;   // "another remote is on" window, and the grant/claim zero-proof age
static const uint32_t kMtxSeizeZeroMs    = 1000;   // zero-proof age for a take-back and a summon
static const uint32_t kMtxSeizedHoldMs   = 5000;   // how long a remote whose control was taken hears SEIZED
static const uint32_t kMtxClaimGrantMs   = 1000;   // life of a claim token
static const uint32_t kMtxClaimFloorMs   = 1000;   // the silence before a claim is at least failsafe_time + this
static const uint32_t kMtxSeizeVerdictMs = 5000;   // the newer SEIZABLE (take-back) or FREE (summon) status must be this fresh
static const uint32_t kMtxSeizeTapGapMs  = 400;    // take-back: tap release -> second press
static const uint32_t kMtxTapMaxMs       = 1000;   // a tap is a press shorter than this (the remote's COMBO_TAP_MAX_MS)
static const uint32_t kMtxFastHoldMs     = 3000;   // a remote stays fast this long after input while FREE
static const uint32_t kMtxNoReplyFastMs  = 5000;   // no valid reply this long -> fast
static const uint32_t kMtxLinkFreshMs    = 1000;   // "connected" window at the normal rate
static const uint32_t kMtxLinkFreshSlowMs= 2500;   // "connected" window while slow
static const uint32_t kMtxReplyWaitMs    = 40;     // the bound of the remote's telemetry wait after a send
static const uint32_t kMtxE9HoldMs       = 1500;   // E9 shows this long after the last REFUSED
static const uint32_t kMtxSeizeConfirmMs = 1000;   // take-back: an owner status within this -> confirm buzz, else "St"
static const uint32_t kMtxSummonWaitMs   = 1500;   // summon: an owner status within this -> the arm runs, else "St"
static const uint32_t kMtxPairWindowMs   = 60000;  // pairing window
static const uint32_t kMtxPairExtendMs   = 20000;  // + this per new enrolment
static const uint32_t kMtxPairMaxMs      = 120000; // never longer than this from the open
static const uint32_t kMtxConfirmWaitMs  = 1500;   // pairing: wait for the confirm
static const int8_t   kMtxPairPowerDbm   = -9;     // both ends while pairing
static const int16_t  kMtxPairMinRssiDbm = -60;    // pairing RSSI gate, both ends (about 5 m or closer)

// Remote-side rate.
static const uint8_t  kMtxSlowCycles       = 10;   // slow = 1 transmit in 10 cycles of 100 ms
static const uint8_t  kMtxLoseOwnerReplies = 3;    // non-owner replies in a row before "lost owner"
static const uint8_t  kMtxLoseOwnerSeized  = 2;    // SEIZED replies in a row before "lost owner" (and "tA")
static const uint8_t  kMtxSeizeAbortThr    = 5;    // take-back gesture aborts at this thr_scaled

// Uplink byte 3 / byte 4 codes (design 5.1 and 8.1).
static const uint8_t  kMtxCtrlMax        = 0xF0;   // the remote clamps throttle here (unchanged)
static const uint8_t  kMtxMetaRtm        = 0xF1;
static const uint8_t  kMtxMetaFm         = 0xF2;
static const uint8_t  kMtxMetaGps        = 0xF3;
static const uint8_t  kMtxMetaAux        = 0xF4;
static const uint8_t  kMtxDropFirst      = 0xF5;   // 0xF5-0xFF: never sent, dropped from every remote
static const uint8_t  kMtxLockedValue    = 0x10;   // 0xF1/0x10 = LOCKED
static const uint8_t  kMtxSeizeValue     = 0x5A;   // 0xF1/0x5A = SEIZE (master take-back)
static const uint8_t  kMtxSummonValue    = 0x3C;   // 0xF1/0x3C = SUMMON
static const uint8_t  kMtxFmFreshBit     = 0x80;   // 0xF2 byte 4 bit 7: a declaration made by a rider gesture
// Remote meta-queue types that never appear on the air: the builder maps them to their codes. They are deliberately
// below kMtxZeroThr, so even a slip that sent one as byte 3 would be a zero-throttle byte, never a squeeze.
static const uint8_t  kMtxQueueSeize     = 0x01;
static const uint8_t  kMtxQueueSummon    = 0x02;

// Link record protocols (remote config mtx_link_proto).
static const uint16_t kMtxProtoLegacy    = 0;      // today's frames, the buggy's address in bytes 0-2
static const uint16_t kMtxProtoV3        = 3;      // link-ID frames

// Settings: defaults and ranges (the validators clamp to these).
static const uint16_t kMtxClaimSDefault   = 10, kMtxClaimSMin = 6, kMtxClaimSMax = 60;        // RX mtx_claim_s, s
static const uint16_t kMtxSeizeEnDefault  = 1;                                                // RX mtx_seize_en, 0-1
static const uint16_t kMtxTakeoverCapDefault = 64, kMtxTakeoverCapMax = 255;                  // RX mtx_takeover_cap, counts
static const uint16_t kMtxTakeoverMsDefault  = 4000, kMtxTakeoverMsMax = 10000;               // RX mtx_takeover_ms, ms
static const uint16_t kMtxAutolockTimeoutSMin = 5, kMtxAutolockTimeoutSMax = 180;             // TX autolock_timeout_s (0 = off)
static const uint16_t kMtxSeizeHoldMsDefault = 1200, kMtxSeizeHoldMsMin = 1000, kMtxSeizeHoldMsMax = 1800;  // TX
static const uint16_t kMtxAutolockSDefault = 120, kMtxAutolockSMin = 30, kMtxAutolockSMax = 600;            // TX mtx_autolock_s (0 = off)

// ============================================================
// 2. SMALL HELPERS
// ============================================================
// mtxAgeMs - the age of a stamp, never negative: the signed difference, a stamp newer than now reads as 0 (the same
//   rule as stampAgeMs() in AutoReturnRules.h, which this header does not include so the remote can use it).
//   Inputs: now_ms, stamp_ms. Returns: ms. Side effects: none.
static inline uint32_t mtxAgeMs(uint32_t now_ms, uint32_t stamp_ms)
{
  const int32_t age = (int32_t)(now_ms - stamp_ms);
  return (age > 0) ? (uint32_t)age : 0u;
}

// mtxStampOf - a stamp is never stored as 0, because 0 means "never". Input: now_ms. Returns: now_ms, or 1 for 0.
static inline uint32_t mtxStampOf(uint32_t now_ms) { return (now_ms != 0) ? now_ms : 1u; }

// mtxWithin - did the event happen, no more than limit_ms ago? Inputs: now, stamp (0 = never), limit. Returns: bool.
static inline bool mtxWithin(uint32_t now_ms, uint32_t stamp_ms, uint32_t limit_ms)
{
  return stamp_ms != 0 && mtxAgeMs(now_ms, stamp_ms) <= limit_ms;
}

// mtxFmDeclared - Follow-Me is armed only for a declared mode 1-5. 0 (disarmed) and 0xFF (nothing declared since
//   boot) are "not armed", so a remote can take control after a buggy boot. Input: fm_mode_runtime. Returns: bool.
static inline bool mtxFmDeclared(uint8_t mode) { return mode >= 1 && mode <= 5; }

// mtxSteerOff - how far the steering byte is from centre (0-128). Input: steering byte. Returns: counts.
static inline uint8_t mtxSteerOff(uint8_t steer)
{
  return (steer >= kMtxSteerCentre) ? (uint8_t)(steer - kMtxSteerCentre) : (uint8_t)(kMtxSteerCentre - steer);
}

// mtxFrameHasInput - does a control frame carry input for the REFUSED (E9) answer? A squeeze (>= 12) or steering
//   24 or more off centre. Inputs: throttle and steering bytes. Returns: bool. Side effects: none.
static inline bool mtxFrameHasInput(uint8_t thr, uint8_t steer)
{
  return thr >= kMtxInputThr || mtxSteerOff(steer) >= kMtxSteerInput;
}

// ============================================================
// 3. IDENTITY AND THE LINK ID (design section 4)
// ============================================================
// The CRC both boards already use for every packet (esp_crc8() in Common/SystemCommon.h), passed in so this header
// needs no Arduino code.
typedef uint8_t (*MtxCrc8Fn)(uint8_t* data, uint8_t length);

// efuseIdFromMac - THE one identity formula: a board's 3-byte chip ID from its 48-bit eFuse MAC. It is the formula the
//   default-address bake in Common/SPIFFSEngine.h has used since V2 (audit W-5: one function, so the bake and the
//   multi-remote identity can never drift apart; the bake switches to this call with the receiver step).
//   BYTE ORDER: ESP.getEfuseMac() returns the MAC in a uint64 with mac[0] in the LOW byte. The boot banner prints the
//   same uint64 with %012llX, high byte first, so the banner reads the MAC backwards.
//   Inputs: mac (the uint64 as getEfuseMac() returns it); crc8. Output: out[3] = {crc8(mac[0..3]), mac[4], mac[5]}.
//   Side effects: none.
static inline void efuseIdFromMac(uint64_t mac, MtxCrc8Fn crc8, uint8_t out[3])
{
  uint8_t m[6];
  for (int i = 0; i < 6; i++) m[i] = (uint8_t)((mac >> (8 * i)) & 0xFF);
  out[0] = crc8(m, 4);
  out[1] = m[4];
  out[2] = m[5];
}

// mtxIdIsZero / mtxIdEqual / mtxIdXor - 3-byte ID helpers. mtxIdXor gives the link ID L = T XOR R, and recovers
//   T = L XOR R on the buggy (XOR is its own inverse). Side effects: none (mtxIdXor writes out only).
static inline bool mtxIdIsZero(const uint8_t id[3]) { return (id[0] | id[1] | id[2]) == 0; }
static inline bool mtxIdEqual(const uint8_t a[3], const uint8_t b[3])
{
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}
static inline void mtxIdXor(const uint8_t a[3], const uint8_t b[3], uint8_t out[3])
{
  out[0] = (uint8_t)(a[0] ^ b[0]);
  out[1] = (uint8_t)(a[1] ^ b[1]);
  out[2] = (uint8_t)(a[2] ^ b[2]);
}

// ============================================================
// 4. UPLINK KINDS (design 5.1, 8.1)
// ============================================================
static const uint8_t MTX_K_DROP   = 0;   // never acted on, no reply
static const uint8_t MTX_K_CTRL   = 1;   // throttle 0x00-0xF0 + steering
static const uint8_t MTX_K_LOCKED = 2;   // 0xF1/0x10
static const uint8_t MTX_K_SEIZE  = 3;   // 0xF1/0x5A
static const uint8_t MTX_K_SUMMON = 4;   // 0xF1/0x3C
static const uint8_t MTX_K_META   = 5;   // 0xF1 (any other value), 0xF2, 0xF3, 0xF4

// mtxKind - the kind of a 6-byte uplink from its bytes 3 and 4. Exactly the three 0xF1 pairs are LOCKED, SEIZE and
//   SUMMON; every other 0xF1 value stays a plain meta, as on every buggy since 2026-04-25. 0xF5-0xFF is DROP in every
//   mode (a buggy at the base commit reads them as throttle, so no remote may ever send them). Returns: MTX_K_*.
static inline uint8_t mtxKind(uint8_t b3, uint8_t b4)
{
  if (b3 <= kMtxCtrlMax)   return MTX_K_CTRL;
  if (b3 >= kMtxDropFirst) return MTX_K_DROP;
  if (b3 == kMtxMetaRtm)
  {
    if (b4 == kMtxLockedValue) return MTX_K_LOCKED;
    if (b4 == kMtxSeizeValue)  return MTX_K_SEIZE;
    if (b4 == kMtxSummonValue) return MTX_K_SUMMON;
  }
  return MTX_K_META;
}

// mtxFrameKind - the kind the buggy acts on, after the mode and entry rules (design 7.1, 7.2, 8.1):
//   - a legacy entry never sends LOCKED / SEIZE / SUMMON, so those codes from it are dropped;
//   - in SINGLE, LOCKED is handled as control 0/127 (today's locked packet, stamp included; the caller rewrites the
//     bytes), and SEIZE and SUMMON are dropped.
//   Inputs: b3, b4; single (exactly one paired remote); legacy_entry. Returns: MTX_K_*. Side effects: none.
static inline uint8_t mtxFrameKind(uint8_t b3, uint8_t b4, bool single, bool legacy_entry)
{
  const uint8_t k = mtxKind(b3, b4);
  if (k == MTX_K_LOCKED || k == MTX_K_SEIZE || k == MTX_K_SUMMON)
  {
    if (legacy_entry) return MTX_K_DROP;
    if (single)       return (k == MTX_K_LOCKED) ? MTX_K_CTRL : MTX_K_DROP;
  }
  return k;
}

// ============================================================
// 5. THE REPLY STATUS (design 5.5): byte 3 = bits 7-5 status, bits 4-0 telemetry index
// ============================================================
static const uint8_t MTX_S_OWNER_SOLO = 0;   // you control and are the only remote: the reply is byte-identical to today's
static const uint8_t MTX_S_OWNER      = 1;   // you control; other remotes are paired
static const uint8_t MTX_S_FREE       = 2;   // nobody controls, or the remote in control can be claimed
static const uint8_t MTX_S_BUSY       = 3;   // another remote controls
static const uint8_t MTX_S_SEIZABLE   = 4;   // a guest controls; you are the master and may take it back
static const uint8_t MTX_S_REFUSED    = 5;   // another remote controls and your input is ignored -> E9
static const uint8_t MTX_S_SEIZED     = 6;   // your control was taken (take-back or claim) -> "tA"
static const uint8_t MTX_S_RESERVED   = 7;   // never sent; read as BUSY

// mtxReplyByte3 / mtxReplyStatusOf / mtxReplyIndexOf / mtxStatusIsOwner - pack and read the reply's byte 3.
//   mtxReplyStatusOf reads the reserved 7 as BUSY. Side effects: none.
static inline uint8_t mtxReplyByte3(uint8_t status, uint8_t idx)
{
  return (uint8_t)(((status & 0x07) << 5) | (idx & 0x1F));
}
static inline uint8_t mtxReplyStatusOf(uint8_t b3)
{
  const uint8_t s = (uint8_t)(b3 >> 5);
  return (s == MTX_S_RESERVED) ? MTX_S_BUSY : s;
}
static inline uint8_t mtxReplyIndexOf(uint8_t b3) { return (uint8_t)(b3 & 0x1F); }
static inline bool    mtxStatusIsOwner(uint8_t s) { return s == MTX_S_OWNER_SOLO || s == MTX_S_OWNER; }

// ============================================================
// 6. REPLY VALUES: THE MASKS AND THE PER-REMOTE ROTATION (design 7.6, 7.7)
// ============================================================
static const uint8_t kMtxIdxRtmDistance  = 5;
static const uint8_t kMtxIdxFmHeadingErr = 14;
static const uint8_t kMtxIdxFmStatus     = 15;
static const uint8_t kMtxIdxFmFlags      = 16;
static const uint8_t kMtxIdxBearing      = 17;
static const uint8_t kMtxIdxLinkQuality  = 18;
static const uint8_t kMtxIdxRxStateFlags = 19;

// mtxReplyValue - the byte 4 a reply carries for index idx.
//   SINGLE: the telemetry byte unchanged (byte-identical to today).
//   MULTI:  index 18 is the link quality of THAT remote's own frames. A remote that is not in control gets the
//           owner's Follow-Me / Return-To-Me fields blanked (5 -> 0xFF, 14 -> 127, 15 bits 0-1 cleared, 16 -> 0,
//           17 -> 0xFF, 19 -> 0), so a guest's screen never shows someone else's mode. The NEW owner gets the same
//           mask until the loop task has rebuilt the bytes after an owner change (owner_change_pending; audit O-1:
//           one rule, not a partial one), so it never sees the previous rider's "St".
//   Inputs: idx, raw (the telemetry byte), single, to_owner, owner_change_pending, slot_link_quality.
//   Returns: the byte to send. Side effects: none.
static inline uint8_t mtxReplyValue(uint8_t idx, uint8_t raw, bool single, bool to_owner, bool owner_change_pending,
                                    uint8_t slot_link_quality)
{
  if (single) return raw;
  if (idx == kMtxIdxLinkQuality) return slot_link_quality;
  if (to_owner && !owner_change_pending) return raw;
  switch (idx)
  {
    case kMtxIdxRtmDistance:  return 0xFF;
    case kMtxIdxFmHeadingErr: return 127;
    case kMtxIdxFmStatus:     return (uint8_t)(raw & (uint8_t)~0x03u);
    case kMtxIdxFmFlags:      return 0;
    case kMtxIdxBearing:      return 0xFF;
    case kMtxIdxRxStateFlags: return 0;
    default:                  return raw;
  }
}

// mtxTelIndexNext - one remote's own telemetry rotation 0..19 (one counter per remote, so a remote answered once a
//   second still sees every index). Input/output: *idx (that remote's counter). Returns: the index to send now.
static inline uint8_t mtxTelIndexNext(uint8_t* idx)
{
  const uint8_t i = (*idx < kMtxTelemetryLen) ? *idx : (uint8_t)0;
  *idx = (uint8_t)((i + 1) % kMtxTelemetryLen);
  return i;
}

// ============================================================
// 7. THE BUGGY'S REMOTE LIST: VALIDATION, MODE, GATE 0, LOOKUP (design 7.1, 7.2, 14.3)
// ============================================================
// The receiver config fields this model reads and writes (the firmware copies them in and out of usrConf).
struct MtxRxBinding {
  uint16_t paired;
  uint8_t  own_address[3];
  uint8_t  dest_address[3];
  uint8_t  mtx_id[kMtxSlots * 3];     // remote IDs in pairing order; slot 0 = the master
  uint8_t  mtx_flags[kMtxSlots];      // per slot: bit 0 valid, bit 1 legacy
};
static const uint8_t kMtxFlagValid  = 0x01;
static const uint8_t kMtxFlagLegacy = 0x02;

// Validator notes (bits): the firmware prints one NOTE line per bit.
static const uint8_t MTX_NOTE_ID_NOT_SET   = 0x01;  // own_address was 00:00:00 -> set to the chip ID, binding dropped
static const uint8_t MTX_NOTE_ID_MISMATCH  = 0x02;  // own_address was not this chip's ID -> binding dropped
static const uint8_t MTX_NOTE_ENTRY_CLEARED= 0x04;  // an entry was 00:00:00, the buggy's own ID, a duplicate, a second legacy,
                                                    //   or a legacy entry next to a new-style one
static const uint8_t MTX_NOTE_UNPAIRED     = 0x08;  // paired == 0: the list and dest_address were cleared

static inline bool mtxEntryValid(const MtxRxBinding* b, uint8_t i)  { return (b->mtx_flags[i] & kMtxFlagValid) != 0; }
static inline bool mtxEntryLegacy(const MtxRxBinding* b, uint8_t i) { return (b->mtx_flags[i] & kMtxFlagLegacy) != 0; }
static inline const uint8_t* mtxEntryId(const MtxRxBinding* b, uint8_t i) { return &b->mtx_id[i * 3]; }
static inline void mtxEntryClear(MtxRxBinding* b, uint8_t i)
{
  memset(&b->mtx_id[i * 3], 0, 3);
  b->mtx_flags[i] = 0;
}

// mtxValidateRxBinding - the receiver's binding rules, in this order, on every config intake path (boot load,
//   migration, ?set, ?setconf, the web import):
//   1. IDENTITY: own_address must be this chip's ID. If not, it is set to the chip ID and paired = 0, so a binding made
//      for another identity (a copied or edited config) is never kept.
//   2. UNPAIR: paired == 0 clears the list and dest_address. This one rule covers every unpair path.
//   3. ENTRIES: an entry that is 00:00:00, equals the buggy's own ID, duplicates an earlier entry, is a second legacy
//      entry, or is a legacy entry while any new-style entry is valid, is cleared. While any new-style entry is valid,
//      dest_address = 00:00:00 (replies go to each remote's own ID).
//   Inputs: b (in and out); efuse_id (this chip's ID). Returns: MTX_NOTE_* bits. Side effects: *b only.
static inline uint8_t mtxValidateRxBinding(MtxRxBinding* b, const uint8_t efuse_id[3])
{
  uint8_t notes = 0;
  if (!mtxIdEqual(b->own_address, efuse_id))
  {
    notes |= mtxIdIsZero(b->own_address) ? MTX_NOTE_ID_NOT_SET : MTX_NOTE_ID_MISMATCH;
    memcpy(b->own_address, efuse_id, 3);
    b->paired = 0;
  }
  if (b->paired == 0)
  {
    bool had = !mtxIdIsZero(b->dest_address);
    for (uint8_t i = 0; i < kMtxSlots; i++) had = had || b->mtx_flags[i] != 0 || !mtxIdIsZero(mtxEntryId(b, i));
    memset(b->mtx_id, 0, sizeof(b->mtx_id));
    memset(b->mtx_flags, 0, sizeof(b->mtx_flags));
    memset(b->dest_address, 0, 3);
    if (had) notes |= MTX_NOTE_UNPAIRED;
    return notes;
  }
  // Rule 3, pass 1: zero, own ID, duplicate of an earlier surviving entry.
  for (uint8_t i = 0; i < kMtxSlots; i++)
  {
    if (!mtxEntryValid(b, i)) continue;
    const uint8_t* id = mtxEntryId(b, i);
    bool bad = mtxIdIsZero(id) || mtxIdEqual(id, b->own_address);
    for (uint8_t j = 0; j < i && !bad; j++) bad = mtxEntryValid(b, j) && mtxIdEqual(mtxEntryId(b, j), id);
    if (bad) { mtxEntryClear(b, i); notes |= MTX_NOTE_ENTRY_CLEARED; }
  }
  // Rule 3, pass 2: no legacy entry next to a new-style entry, and never two legacy entries.
  bool any_v3 = false;
  for (uint8_t i = 0; i < kMtxSlots; i++) any_v3 = any_v3 || (mtxEntryValid(b, i) && !mtxEntryLegacy(b, i));
  bool seen_legacy = false;
  for (uint8_t i = 0; i < kMtxSlots; i++)
  {
    if (!mtxEntryValid(b, i) || !mtxEntryLegacy(b, i)) continue;
    if (any_v3 || seen_legacy) { mtxEntryClear(b, i); notes |= MTX_NOTE_ENTRY_CLEARED; }
    else seen_legacy = true;
  }
  if (any_v3) memset(b->dest_address, 0, 3);
  return notes;
}

// mtxValidateTxLink - the remote's link-record rules: a protocol other than 0 or 3 unpairs (returns 1, the caller
//   prints "link record invalid - pair again"); an unpaired remote has protocol 0. Inputs/outputs: *paired, *proto.
static inline uint8_t mtxValidateTxLink(uint16_t* paired, uint16_t* proto)
{
  uint8_t note = 0;
  if (*proto != kMtxProtoLegacy && *proto != kMtxProtoV3) { *proto = kMtxProtoLegacy; *paired = 0; note = 1; }
  if (*paired == 0) *proto = kMtxProtoLegacy;
  return note;
}

// mtxValidEntryCount - valid entries in the list. Side effects: none.
static inline uint8_t mtxValidEntryCount(const MtxRxBinding* b)
{
  uint8_t n = 0;
  for (uint8_t i = 0; i < kMtxSlots; i++) if (mtxEntryValid(b, i)) n++;
  return n;
}

// mtxImplicitLegacy - a board migrated from the single-remote firmware: paired, no list, a dest_address, and its own
//   address is its chip ID. It acts as the one legacy entry (SINGLE). Never formed while any list entry exists.
static inline bool mtxImplicitLegacy(const MtxRxBinding* b, const uint8_t rx_id[3])
{
  return b->paired != 0 && mtxValidEntryCount(b) == 0 && !mtxIdIsZero(b->dest_address)
      && mtxIdEqual(b->own_address, rx_id);
}

// mtxMayAcceptControl - gate 0: paired, an own address, and at least one entry (or the implicit legacy entry).
static inline bool mtxMayAcceptControl(const MtxRxBinding* b, const uint8_t rx_id[3])
{
  return b->paired != 0 && !mtxIdIsZero(b->own_address)
      && (mtxValidEntryCount(b) >= 1 || mtxImplicitLegacy(b, rx_id));
}

static const uint8_t MTX_MODE_UNPAIRED = 0;
static const uint8_t MTX_MODE_SINGLE   = 1;   // exactly one remote: today's code path
static const uint8_t MTX_MODE_MULTI    = 2;   // 2-4 remotes, all new-style: the ownership rules

// mtxRxMode - which rules apply. Inputs: b, rx_id. Returns: MTX_MODE_*. Side effects: none.
static inline uint8_t mtxRxMode(const MtxRxBinding* b, const uint8_t rx_id[3])
{
  if (!mtxMayAcceptControl(b, rx_id)) return MTX_MODE_UNPAIRED;
  return (mtxValidEntryCount(b) >= 2) ? MTX_MODE_MULTI : MTX_MODE_SINGLE;
}

// mtxLookupSlot - which paired remote sent this frame (design 7.2 gate 1 and the lookup).
//   Address 00:00:00 -> kMtxSlotGate1 (no valid link ID is zero). Otherwise T = address XOR rx_id:
//   - T == 00:00:00 is a legacy frame (bytes 0-2 = the buggy's own address): the legacy entry, only as the sole entry
//     (the explicit legacy slot, or kMtxSlotImplicit);
//   - else the valid new-style entry whose ID is T.
//   A frame built for another buggy, and any reply (bytes 0-2 = a remote's own ID), decode to an ID that is not on the
//   list: kMtxSlotUnknown. Inputs: b, rx_id, addr (bytes 0-2). Returns: the slot or a negative code. Side effects: none.
static inline int8_t mtxLookupSlot(const MtxRxBinding* b, const uint8_t rx_id[3], const uint8_t addr[3])
{
  if (mtxIdIsZero(addr)) return kMtxSlotGate1;
  uint8_t t[3];
  mtxIdXor(addr, rx_id, t);
  const uint8_t n = mtxValidEntryCount(b);
  if (mtxIdIsZero(t))
  {
    if (n == 0) return mtxImplicitLegacy(b, rx_id) ? kMtxSlotImplicit : kMtxSlotUnknown;
    if (n == 1)
      for (uint8_t i = 0; i < kMtxSlots; i++)
        if (mtxEntryValid(b, i) && mtxEntryLegacy(b, i)) return (int8_t)i;
    return kMtxSlotUnknown;
  }
  for (uint8_t i = 0; i < kMtxSlots; i++)
    if (mtxEntryValid(b, i) && !mtxEntryLegacy(b, i) && mtxIdEqual(mtxEntryId(b, i), t)) return (int8_t)i;
  return kMtxSlotUnknown;
}

// ============================================================
// 8. THE REMOTE'S ONE UPLINK BUILDER (design 5.3, 8.3)
// ============================================================
// The only place a remote can produce LOCKED, SEIZE or SUMMON, and only with a link-ID address.
struct MtxLinkRec {
  uint16_t proto;            // mtx_link_proto: 0 or 3
  uint8_t  dest_address[3];  // the buggy's ID (proto 3) or its address (proto 0)
  uint8_t  efuse_id[3];      // this remote's chip ID
};
static const uint8_t MTX_B_CTRL   = 1;   // a = throttle, b = steering
static const uint8_t MTX_B_LOCKED = 2;
static const uint8_t MTX_B_SEIZE  = 3;
static const uint8_t MTX_B_SUMMON = 4;
static const uint8_t MTX_B_META   = 5;   // a = type 0xF1-0xF4, b = value (0xF3 + subtype is the rider-GPS announce)

// mtxUplinkAddr - bytes 0-2 for this link: proto 3 -> L = own chip ID XOR the buggy's ID; proto 0 -> dest_address.
//   Any other protocol -> false (nothing may be sent). Also the address the 15-byte rider-GPS packet carries.
static inline bool mtxUplinkAddr(const MtxLinkRec* r, uint8_t out[3])
{
  if (r->proto == kMtxProtoV3)     { mtxIdXor(r->efuse_id, r->dest_address, out); return true; }
  if (r->proto == kMtxProtoLegacy) { memcpy(out, r->dest_address, 3); return true; }
  return false;
}

// mtxBuildUplink - build one 6-byte uplink, address and type together, with the CRC over bytes 0-4.
//   proto 3: L on every kind. proto 0: today's frames - LOCKED becomes control 0/127, SEIZE and SUMMON build nothing.
//   CTRL clamps the throttle at 0xF0. META accepts only 0xF1-0xF4, and never an 0xF1 value that is one of the three
//   codes (so nothing but this function's own LOCKED / SEIZE / SUMMON rows can emit them). No output ever has byte 3
//   >= 0xF5. Inputs: kind, r, a, b, crc8. Output: out[6], written only on success. Returns: false = send nothing.
static inline bool mtxBuildUplink(uint8_t kind, const MtxLinkRec* r, uint8_t a, uint8_t b, uint8_t out[6],
                                  MtxCrc8Fn crc8)
{
  uint8_t f[6];
  if (!mtxUplinkAddr(r, f)) return false;
  const bool v3 = (r->proto == kMtxProtoV3);
  switch (kind)
  {
    case MTX_B_CTRL:   f[3] = (a > kMtxCtrlMax) ? kMtxCtrlMax : a; f[4] = b; break;
    case MTX_B_LOCKED: if (v3) { f[3] = kMtxMetaRtm; f[4] = kMtxLockedValue; } else { f[3] = 0; f[4] = kMtxSteerCentre; } break;
    case MTX_B_SEIZE:  if (!v3) return false; f[3] = kMtxMetaRtm; f[4] = kMtxSeizeValue;  break;
    case MTX_B_SUMMON: if (!v3) return false; f[3] = kMtxMetaRtm; f[4] = kMtxSummonValue; break;
    case MTX_B_META:
      if (a < kMtxMetaRtm || a > kMtxMetaAux) return false;
      if (a == kMtxMetaRtm && (b == kMtxLockedValue || b == kMtxSeizeValue || b == kMtxSummonValue)) return false;
      f[3] = a; f[4] = b;
      break;
    default: return false;
  }
  f[5] = crc8(f, 5);
  memcpy(out, f, 6);
  return true;
}

// mtxBuildUplinkFromQueue - the meta queue's entry to the builder: the two internal queue types become SEIZE and
//   SUMMON, every other type is a plain meta. Inputs: qtype, qvalue, r, crc8. Output: out[6]. Returns: as above.
static inline bool mtxBuildUplinkFromQueue(uint8_t qtype, uint8_t qvalue, const MtxLinkRec* r, uint8_t out[6],
                                           MtxCrc8Fn crc8)
{
  if (qtype == kMtxQueueSeize)  return mtxBuildUplink(MTX_B_SEIZE,  r, 0, 0, out, crc8);
  if (qtype == kMtxQueueSummon) return mtxBuildUplink(MTX_B_SUMMON, r, 0, 0, out, crc8);
  return mtxBuildUplink(MTX_B_META, r, qtype, qvalue, out, crc8);
}

// mtxMetaTypePriority - the meta queue's second-slot priority: SEIZE and SUMMON 4, then 0xF1 3, 0xF2 2, 0xF4 1 (the
//   remote's metaTypePriority() today, plus the two internal types). Because those have their own queue types, the
//   queue's "same type -> update in place" rule can never merge them with a queued RTM 0xF1.
static inline uint8_t mtxMetaTypePriority(uint8_t type)
{
  if (type == kMtxQueueSeize || type == kMtxQueueSummon) return 4;
  if (type == kMtxMetaRtm) return 3;
  if (type == kMtxMetaFm)  return 2;
  if (type == kMtxMetaAux) return 1;
  return 0;
}

// ============================================================
// 9. THE TAKEOVER LIMIT (design section 8)
// ============================================================
// mtxTakeoverCap - the throttle limit after control passes: cap0 at the grant, rising evenly to 255 over T_ms.
//   255 (no limit) when grant_ms is 0 (never granted, or cleared), T_ms is 0 or cap0 is 255. A grant stamp newer than
//   now counts as "just granted" (cap0). Inputs: now, grant_ms, cap0 (0-255), T_ms (0-10000). Returns: 0-255.
//   Subtract-only by construction: the caller applies min(throttle, this). Side effects: none.
static inline uint8_t mtxTakeoverCap(uint32_t now, uint32_t grant_ms, uint8_t cap0, uint32_t T_ms)
{
  if (grant_ms == 0 || T_ms == 0 || cap0 >= 255) return 255;
  const uint32_t dt = mtxAgeMs(now, grant_ms);
  if (dt >= T_ms) return 255;
  return (uint8_t)(cap0 + ((uint32_t)(255 - cap0) * dt) / T_ms);
}

// mtxTakeoverExpired - true once the limit has run out, so the caller may clear the grant stamp (compare-exchange).
static inline bool mtxTakeoverExpired(uint32_t now, uint32_t grant_ms, uint32_t T_ms)
{
  return grant_ms != 0 && (T_ms == 0 || mtxAgeMs(now, grant_ms) >= T_ms);
}

// ============================================================
// 10. THE BUGGY'S OWNERSHIP RULES (design 7.2-7.5 as amended by v3.2 section 3)
// ============================================================
struct MtxRxCfg {
  bool     single;        // SINGLE: today's path for every frame
  uint8_t  valid_mask;    // MULTI: bit i = slot i is a valid new-style entry (slot 0 = the master)
  uint16_t claim_s;       // mtx_claim_s, 6-60 s: the silence and the idle time before the owner is claimable
  uint16_t failsafe_ms;   // failsafe_time, 100-10000 ms
  bool     seize_en;      // mtx_seize_en: the master may take control back from a guest
};

// The firmware state the rules read, fetched fresh through MtxRxHooks.read_env (it changes under the rules: the
// synthetic "RTM off" clears rtm_rx_active at once; fm_rx_active is the loop task's flag).
struct MtxRxEnv {
  bool    pwm_active;       // the motor output runs (end of setup())
  uint8_t fm_mode_runtime;  // 0, 1-5 or 0xFF
  bool    rtm_rx_active;
  bool    fm_rx_active;
};

// mtxArmed - Follow-Me or Return-To-Me armed on the buggy. mtxGrantGatesOk - control may pass now: the motor output
//   runs and nothing is armed. Side effects: none.
static inline bool mtxArmed(const MtxRxEnv* e)
{
  return mtxFmDeclared(e->fm_mode_runtime) || e->rtm_rx_active || e->fm_rx_active;
}
static inline bool mtxGrantGatesOk(const MtxRxEnv* e) { return e->pwm_active && !mtxArmed(e); }

struct MtxClaimToken {
  int8_t   slot;   // kMtxNone = no token
  uint32_t ms;     // stamp, never 0 while held
};

struct MtxRxCounters {
  uint16_t grants, claims, summons, summons_refused, tokens_lapsed, non_owner_meta;
  uint16_t released_lock, released_seized, released_claim, dropped;
  uint16_t released_rebind;   // V2.5-Evo - 2026-10-20 - Z-7 (a): releases because the paired list changed (receiver)
};

// The ownership state. The radio task is its only writer.
struct MtxRxState {
  int8_t   owner;                         // kMtxNone or slot 0-3
  uint32_t heard_ms[kMtxSlots];           // any frame from the slot
  uint32_t ctrl_heard_ms[kMtxSlots];      // control frames only ("present"); LOCKED never counts
  uint8_t  zero_streak[kMtxSlots];        // control frames in a row below kMtxZeroThr
  uint32_t last_zero_ms[kMtxSlots];       // the last of them
  bool     seized_on[kMtxSlots];          // the slot hears SEIZED until seized_until_ms
  uint32_t seized_until_ms[kMtxSlots];
  bool     claim_notice[kMtxSlots];       // control was claimed from this slot; SEIZED restarts on its next frame
  uint8_t  last_status[kMtxSlots];        // the last status sent to each slot (?mtx)
  uint32_t owner_heard_ms;                // any frame from the owner (silence clock)
  uint32_t owner_idle_base_ms;            // the idle clock's base: owner activity, and the end of an armed period
  bool     owner_armed_seen;              // the last evaluation saw Follow-Me / RTM armed
  uint8_t  owner_last_thr;                // the owner's last throttle byte
  MtxClaimToken token;
  uint32_t grant_ms;                      // the last published grant stamp (?mtx; the PWM task reads the firmware's copy)
  uint8_t  claimable;                     // the last evaluated MTX_CL_* - for the edge lines and ?mtx only, NEVER an authority
  uint32_t claimable_since_ms;
  MtxRxCounters n;
};

// mtxRxStateInit - nobody owns, no token, every clock "never". Side effects: *st only.
static inline void mtxRxStateInit(MtxRxState* st)
{
  memset(st, 0, sizeof(*st));
  st->owner = kMtxNone;
  st->token.slot = kMtxNone;
}

// The firmware supplies these; every one must be set. All are called on the radio task.
// ============================================================
// THE HOOK CONTRACT (step-1 code audit Y-1). The host test proves the ORDER of every release, grant and claim; it
// cannot see the hook bodies. The model's guarantees carry into the firmware only if every body keeps these rules:
//  (a) THE OWNER ATOMIC. No hook is told "publish the owner", yet the loop task reads the owner and the change
//      counter. owner_changed() is called right after st->owner has its new value (NONE in a release, s in a
//      grant), so owner_changed() publishes st->owner to the firmware's owner atomic FIRST and only then moves the
//      change counter. A loop that sees the counter move therefore also sees the new owner.
//  (b) read_env() reads the LIVE atomics on every call (PWM_active, fm_mode_runtime, rtm_rx_active, fm_rx_active),
//      never a copy taken earlier: mtxClaim() reads it again after the synthetic "RTM off" to decide grant or token.
//  (c) zero_outputs() is two single-byte stores (thr_received = 0, steering_received = 127) and then the compiler
//      barrier; it stamps NO failsafe clock, so the motor gate shuts on its own at failsafe_time. publish_limit()
//      is the grant-stamp store plus the same barrier, so the limit is visible before any byte of the new owner.
//  (d) event() never blocks and never prints on the radio task: it queues, and the loop task prints. In SINGLE it
//      is never reached except for a dropped frame, and it prints nothing for those either (no "MTX:" line ever
//      appears with one remote).
//  (e) No hook calls back into this model or writes MtxRxState. The radio task is the state's only writer.
//  (f) reply() yields (two 10 ms task delays around the send). Every state change a DECISION reads is made before it
//      is called, and mtxOnFrame() calls it last of the decisions for every CTRL and LOCKED frame. V2.5-Evo -
//      2026-10-20 - Z-7 (b): one thing runs after it - the claimable EDGE (mtxClaimableEdge(), the end of mtxOnFrame()).
//      That writes only st->claimable / claimable_since_ms (display and ?mtx state, never an authority) and queues one
//      line, so nothing the reply carried and no motor state can change behind the reply.
//  (g) One millis() per wake, read after the receive semaphore was taken, passed as `now` to every call for that
//      wake; mtxTimers() also runs on the 2 s semaphore timeout, so token expiry never waits for a frame.
//  (h) The config is clamped (mtxClampRxConfig()) and validated before MtxRxCfg is built: a claim_s of 0 would make
//      an owner at zero claimable at once. valid_mask carries new-style entries only (never a legacy one), and
//      mtxOnFrame() is never called with a negative slot: the caller drops every negative mtxLookupSlot() result.
// ============================================================
struct MtxRxHooks {
  void* ctx;
  void (*read_env)(void* ctx, MtxRxEnv* out);
  void (*zero_outputs)(void* ctx);                       // release step 1: thr_received = 0, steering_received = 127, barrier
  void (*synthetic_rtm_off)(void* ctx);                  // release step 2: processRtmStatePacket() with 0xF1/0
  void (*synthetic_fm_off)(void* ctx);                   // release step 3: processFmOverridePacket() with 0xF2/0
  void (*reset_rider_gps)(void* ctx);                    // release step 4: rider fix stamps, Phase A/B, 6-byte receive mode
  void (*reset_boot_id)(void* ctx);                      // release step 5: rx_tx_boot_id = none
  void (*publish_limit)(void* ctx, uint32_t grant_ms);   // grant step 1: mtx_grant_ms store + fence, BEFORE any granted byte
  void (*owner_changed)(void* ctx);                      // mtx_owner_change_seq += 1 (the loop's sticky-window hook)
  void (*apply_owner)(void* ctx, int8_t slot, const uint8_t f[6]);   // today's control branch: the ONLY throttle writer
  void (*meta)(void* ctx, int8_t slot, const uint8_t f[6]);          // today's 0xF1-0xF4 handlers (they stamp last_packet)
  void (*reply)(void* ctx, int8_t slot, uint8_t status);             // build and send the reply to that slot
  void (*event)(void* ctx, uint8_t ev, int8_t slot, uint8_t arg);    // one serial line / counter
};

// mtxRxHooksComplete - the boot check (Y-1): true only if every function pointer is set. A firmware whose table is
//   incomplete must not run mtxOnFrame() at all (it drops every frame instead, so the motor gate stays shut).
//   Input: h. Returns: bool. Side effects: none.
static inline bool mtxRxHooksComplete(const MtxRxHooks* h)
{
  return h != NULL && h->read_env != NULL && h->zero_outputs != NULL && h->synthetic_rtm_off != NULL
      && h->synthetic_fm_off != NULL && h->reset_rider_gps != NULL && h->reset_boot_id != NULL
      && h->publish_limit != NULL && h->owner_changed != NULL && h->apply_owner != NULL && h->meta != NULL
      && h->reply != NULL && h->event != NULL;
}

// Events (MtxRxHooks.event).
static const uint8_t MTX_EV_DROPPED        = 1;   // arg: the raw MTX_K_* of the dropped frame
static const uint8_t MTX_EV_RELEASED       = 2;   // slot: old owner; arg: MTX_REL_*
static const uint8_t MTX_EV_TAKES_CONTROL  = 3;   // slot: new owner
static const uint8_t MTX_EV_CLAIMABLE      = 4;   // slot: owner; arg: MTX_CL_SILENT / MTX_CL_IDLE
static const uint8_t MTX_EV_NOT_CLAIMABLE  = 5;   // slot: owner; arg: MTX_NC_*
static const uint8_t MTX_EV_CLAIM          = 6;   // slot: claimer; arg: the old owner's slot, or 0xFF from nobody
static const uint8_t MTX_EV_TOKEN          = 7;   // slot: holder (the grant waits for Follow-Me to idle)
static const uint8_t MTX_EV_TOKEN_LAPSED   = 8;   // slot: holder; arg: 0 = timed out, 1 = the holder locked
static const uint8_t MTX_EV_SUMMON         = 9;   // slot: summoner (accepted)
static const uint8_t MTX_EV_SUMMON_REFUSED = 10;  // slot: summoner
static const uint8_t MTX_EV_NON_OWNER_META = 11;  // slot: sender; arg: byte 3

static const uint8_t MTX_REL_LOCK   = 1;
static const uint8_t MTX_REL_SEIZED = 2;
static const uint8_t MTX_REL_CLAIM  = 3;
// V2.5-Evo - 2026-10-20 - receiver step: the paired-remote list changed under a running owner (a config import or
// ?applyconf rebuilt it, or ?set paired 0 cleared it). The slot numbers may now name other remotes, so the receiver
// releases with the same tested order before it starts again from "nobody owns".
static const uint8_t MTX_REL_REBIND = 4;

static const uint8_t MTX_CL_NO     = 0;
static const uint8_t MTX_CL_SILENT = 1;
static const uint8_t MTX_CL_IDLE   = 2;

static const uint8_t MTX_NC_HEARD  = 1;   // the silent owner was heard again
static const uint8_t MTX_NC_INPUT  = 2;   // the idle owner gave input (or armed)
static const uint8_t MTX_NC_ALONE  = 3;   // nobody else is on any more

// mtxSlotValid - a valid MULTI slot. Side effects: none.
static inline bool mtxSlotValid(const MtxRxCfg* c, int8_t s)
{
  return s >= 0 && s < (int8_t)kMtxSlots && ((c->valid_mask >> s) & 1u) != 0;
}

// mtxClaimMs - the silence before the owner is claimable: max(mtx_claim_s, failsafe_time + 1 s), so a claim can never
//   start while the old owner's motor gate could still be open. Inputs: claim_s, failsafe_ms. Returns: ms.
static inline uint32_t mtxClaimMs(uint16_t claim_s, uint16_t failsafe_ms)
{
  const uint32_t a = (uint32_t)claim_s * 1000u;
  const uint32_t b = (uint32_t)failsafe_ms + kMtxClaimFloorMs;
  return (a > b) ? a : b;
}

// mtxPresent - another remote is on: a valid slot other than the owner sent a CONTROL frame within 3 s. A locked or
//   switched-off remote is not present. Side effects: none.
static inline bool mtxPresent(const MtxRxState* st, const MtxRxCfg* c, uint32_t now)
{
  for (int8_t s = 0; s < (int8_t)kMtxSlots; s++)
    if (s != st->owner && mtxSlotValid(c, s) && mtxWithin(now, st->ctrl_heard_ms[s], kMtxPresentMs)) return true;
  return false;
}

// mtxOwnerActivity - THE X-1 ACTIVITY PREDICATE (audit X-1, option (a)): does this frame FROM THE OWNER show the rider
//   is using the remote? It restarts the idle clock.
//   - a control frame with a throttle byte >= 8 or steering >= 24 off centre;
//   - 0xF1/0 or 0xF1/1: the remote's "RTM armed, not yet active" and "RTM on". Under option (a) the remote repeats
//     0xF1/0 every 3 s or less while a toggle combo is held or the RTM arm ceremony runs (from the first combo press),
//     because the gesture itself sends throttle 0 and steering 127, and the ceremony forces the throttle byte to 0;
//     without this an owner who is arming looks idle. (The remote half: it must never send that 0xF1/0 while RTM is
//     ACTIVE, which would end the return.)
//   - 0xF2 with the fresh bit: a Follow-Me declaration made by a rider gesture.
//   Never activity: the 30 s keepalive, the boot ID, the refresh, aux, the GPS announce.
//   Inputs: b3, b4 of an owner frame. Returns: bool. Side effects: none.
static inline bool mtxOwnerActivity(uint8_t b3, uint8_t b4)
{
  if (b3 <= kMtxCtrlMax) return b3 >= kMtxZeroThr || mtxSteerOff(b4) >= kMtxSteerInput;
  if (b3 == kMtxMetaRtm) return b4 == 0 || b4 == 1;
  if (b3 == kMtxMetaFm)  return (b4 & kMtxFmFreshBit) != 0;
  return false;
}

// mtxIdleTrack - (audit X-2) time spent armed never counts toward idle. While Follow-Me or RTM is armed the idle base
//   keeps restarting, and the first evaluation that sees it disarmed restarts it once more, so idle needs a full
//   mtx_claim_s at zero AFTER the disarm (late by at most one evaluation, never early). Run on every evaluation, before
//   mtxClaimableReason(). Inputs: st, e, now. Side effects: st->owner_idle_base_ms / owner_armed_seen.
static inline void mtxIdleTrack(MtxRxState* st, const MtxRxEnv* e, uint32_t now)
{
  if (st->owner == kMtxNone) return;
  if (mtxArmed(e))
  {
    st->owner_armed_seen   = true;
    st->owner_idle_base_ms = mtxStampOf(now);
  }
  else if (st->owner_armed_seen)
  {
    st->owner_armed_seen   = false;
    st->owner_idle_base_ms = mtxStampOf(now);
  }
}

// mtxClaimableReason - is the owner claimable now, and why (design v3.2 3.1)? Evaluated against the current clock
//   every time; the stored copy in st is for the edge lines only.
//   claimable = (MULTI) && an owner && another remote present && (silent || idle), where
//     silent = nothing at all from the owner for mtxClaimMs();
//     idle   = nothing armed (and not armed at the last evaluation) && no owner activity for claim_s && the owner's
//              last throttle byte below 8.
//   Idle never makes an armed owner claimable; only silence does. Returns: MTX_CL_*. Side effects: none.
static inline uint8_t mtxClaimableReason(const MtxRxState* st, const MtxRxCfg* c, const MtxRxEnv* e, uint32_t now)
{
  if (c->single || st->owner == kMtxNone) return MTX_CL_NO;
  if (!mtxPresent(st, c, now)) return MTX_CL_NO;
  if (mtxAgeMs(now, st->owner_heard_ms) >= mtxClaimMs(c->claim_s, c->failsafe_ms)) return MTX_CL_SILENT;
  if (!mtxArmed(e) && !st->owner_armed_seen && st->owner_last_thr < kMtxZeroThr
      && mtxAgeMs(now, st->owner_idle_base_ms) >= (uint32_t)c->claim_s * 1000u) return MTX_CL_IDLE;
  return MTX_CL_NO;
}

// Token and zero-proof helpers. Side effects: none.
static inline bool mtxTokenHeld(const MtxRxState* st) { return st->token.slot != kMtxNone; }
static inline bool mtxTokenHeldByOther(const MtxRxState* st, int8_t s) { return mtxTokenHeld(st) && st->token.slot != s; }
static inline bool mtxZeroProofOk(const MtxRxState* st, int8_t s, uint32_t now, uint32_t fresh_ms)
{
  return st->zero_streak[s] >= kMtxZeroPkts && mtxWithin(now, st->last_zero_ms[s], fresh_ms);
}

// mtxGrantOk - R1: a remote may take a FREE buggy (nobody owns). Read BEFORE this frame's zero-proof update.
//   Needs: nobody owns; a valid slot; the motor output runs; this frame is a squeeze (>= 12); 2 or more zero frames
//   before it, the last within 3 s; nothing armed; no claim token held by another slot. Side effects: none.
static inline bool mtxGrantOk(const MtxRxState* st, const MtxRxCfg* c, const MtxRxEnv* e, int8_t s, uint8_t thr,
                              uint32_t now)
{
  return st->owner == kMtxNone && mtxSlotValid(c, s) && e->pwm_active && thr >= kMtxInputThr
      && mtxZeroProofOk(st, s, now, kMtxPresentMs) && !mtxArmed(e) && !mtxTokenHeldByOther(st, s);
}

// mtxClaimOk - R14: a squeeze claim of a claimable owner. Read BEFORE this frame's zero-proof update. The same zero
//   proof as a grant, and no Follow-Me / RTM gate on purpose: the claim's release disarms them. Side effects: none.
static inline bool mtxClaimOk(const MtxRxState* st, const MtxRxCfg* c, const MtxRxEnv* e, int8_t s, uint8_t thr,
                              uint32_t now)
{
  return st->owner != kMtxNone && s != st->owner && mtxSlotValid(c, s) && e->pwm_active && thr >= kMtxInputThr
      && mtxZeroProofOk(st, s, now, kMtxPresentMs) && !mtxTokenHeldByOther(st, s)
      && mtxClaimableReason(st, c, e, now) != MTX_CL_NO;
}

// mtxSummonOk - R15: a SUMMON from a remote that is not in control. Needs MULTI, a valid slot that is not the owner,
//   nobody owning or a claimable owner (never against an owner who is commanding), the motor output, zero proof within
//   1 s, and no token held by another slot. Side effects: none.
static inline bool mtxSummonOk(const MtxRxState* st, const MtxRxCfg* c, const MtxRxEnv* e, int8_t s, uint32_t now)
{
  if (c->single || !mtxSlotValid(c, s) || s == st->owner) return false;
  if (st->owner != kMtxNone && mtxClaimableReason(st, c, e, now) == MTX_CL_NO) return false;
  return e->pwm_active && mtxZeroProofOk(st, s, now, kMtxSeizeZeroMs) && !mtxTokenHeldByOther(st, s);
}

// mtxSeizeOk - R8: the master's take-back. Slot 0 only, a guest owns, take-back enabled, the motor output, nothing
//   armed, the master at zero within 1 s. Side effects: none.
static inline bool mtxSeizeOk(const MtxRxState* st, const MtxRxCfg* c, const MtxRxEnv* e, int8_t m, uint32_t now)
{
  return !c->single && m == 0 && mtxSlotValid(c, m) && st->owner != kMtxNone && st->owner != m && c->seize_en
      && e->pwm_active && !mtxArmed(e) && mtxZeroProofOk(st, m, now, kMtxSeizeZeroMs);
}

// mtxNonOwnerStatus - the answer to a remote that is not in control (design v3.2 3.7; audit X-6: LOCKED frames use it
//   too, so nobody but the token holder hears FREE while a token stands). First match wins:
//   1. SEIZED: control was claimed from it (the notice; the 5 s hold restarts from this frame), or within the hold;
//   2. BUSY: a claim token is held by another slot;
//   3. FREE: nobody owns, or the owner is claimable;
//   4. REFUSED: this frame carries input (E9);
//   5. SEIZABLE: it is the master, a guest owns, take-back enabled, nothing armed;
//   6. BUSY.
//   Inputs: st, c, e, s, frame_has_input (false for LOCKED), now. Returns: MTX_S_*. Side effects: the claim notice
//   and the SEIZED hold of slot s.
static inline uint8_t mtxNonOwnerStatus(MtxRxState* st, const MtxRxCfg* c, const MtxRxEnv* e, int8_t s,
                                        bool frame_has_input, uint32_t now)
{
  if (st->claim_notice[s])
  {
    st->claim_notice[s]    = false;
    st->seized_on[s]       = true;
    st->seized_until_ms[s] = now + kMtxSeizedHoldMs;
    return MTX_S_SEIZED;
  }
  if (st->seized_on[s])
  {
    if ((int32_t)(st->seized_until_ms[s] - now) > 0) return MTX_S_SEIZED;
    st->seized_on[s] = false;
  }
  if (mtxTokenHeldByOther(st, s)) return MTX_S_BUSY;
  if (st->owner == kMtxNone || mtxClaimableReason(st, c, e, now) != MTX_CL_NO) return MTX_S_FREE;
  if (frame_has_input) return MTX_S_REFUSED;
  if (s == 0 && st->owner != 0 && c->seize_en && !mtxArmed(e)) return MTX_S_SEIZABLE;
  return MTX_S_BUSY;
}

// mtxZeroProofStep - after the decisions: a control byte below 8 extends the slot's zero streak, anything else ends
//   it. Side effects: st->zero_streak[s] / last_zero_ms[s].
static inline void mtxZeroProofStep(MtxRxState* st, int8_t s, uint8_t thr, uint32_t now)
{
  if (thr < kMtxZeroThr)
  {
    if (st->zero_streak[s] < 255) st->zero_streak[s]++;
    st->last_zero_ms[s] = mtxStampOf(now);
  }
  else
  {
    st->zero_streak[s] = 0;
  }
}

// mtxOwnerHeard - any frame from the owner restarts the silence clock. SILENCE IS NOT TIME AT ZERO: an owner that was
//   not heard for at least claim_s is not "at zero" - the buggy could not see its trigger - so when it is heard again
//   its idle clock restarts too. Without this, an owner coming back from a silence at zero would be idle-claimable on
//   its very first frame, while the design says the owner coming back ends "claimable" at once (design 3.2, T24).
//   Later, never earlier. Inputs: st, c, now. Side effects: st->owner_heard_ms, st->owner_idle_base_ms.
static inline void mtxOwnerHeard(MtxRxState* st, const MtxRxCfg* c, uint32_t now)
{
  if (mtxAgeMs(now, st->owner_heard_ms) >= (uint32_t)c->claim_s * 1000u) st->owner_idle_base_ms = mtxStampOf(now);
  st->owner_heard_ms = mtxStampOf(now);
}

// mtxApplyOwner - the owner's control frame: the silence and idle clocks, then today's control branch (the hook).
static inline void mtxApplyOwner(MtxRxState* st, const MtxRxCfg* c, const MtxRxHooks* h, int8_t s, const uint8_t f[6],
                                 uint32_t now)
{
  mtxOwnerHeard(st, c, now);
  st->owner_last_thr = f[3];
  if (mtxOwnerActivity(f[3], f[4])) st->owner_idle_base_ms = mtxStampOf(now);
  h->apply_owner(h->ctx, s, f);
}

// mtxRelease - the owner loses control (lock, master take-back, claim). THE ORDER IS THE SAFETY RULE:
//   1. throttle 0 and steering centre FIRST, with no failsafe stamp, so the motor gate stays open at neutral only until
//      failsafe_time and then shuts on its own clock;
//   2. "RTM off" through today's handler (it arms the slow ceiling of a return that was running);
//   3. "Follow-Me off" through today's handler;
//   4. the rider-GPS state reset; 5. the remote boot ID reset;
//   6. nobody owns, then the owner-change counter (the loop closes the end windows), then one line.
//   No timer ever calls this. Inputs: st, h, reason (MTX_REL_*), now. Side effects: st, and the hooks.
static inline void mtxRelease(MtxRxState* st, const MtxRxHooks* h, uint8_t reason, uint32_t now)
{
  (void)now;
  const int8_t o = st->owner;
  h->zero_outputs(h->ctx);
  h->synthetic_rtm_off(h->ctx);
  h->synthetic_fm_off(h->ctx);
  h->reset_rider_gps(h->ctx);
  h->reset_boot_id(h->ctx);
  st->owner              = kMtxNone;
  st->owner_armed_seen   = false;
  st->claimable          = MTX_CL_NO;
  st->claimable_since_ms = 0;
  h->owner_changed(h->ctx);
  if (reason == MTX_REL_LOCK)   st->n.released_lock++;
  if (reason == MTX_REL_SEIZED) st->n.released_seized++;
  if (reason == MTX_REL_CLAIM)  st->n.released_claim++;
  if (reason == MTX_REL_REBIND) st->n.released_rebind++;   // V2.5-Evo - 2026-10-20 - Z-7 (a)
  h->event(h->ctx, MTX_EV_RELEASED, o, reason);
}

// mtxGrant - a remote takes control. THE ORDER: 1. the takeover limit is published (stamp never 0) BEFORE anything
//   the new owner sends can be applied; 2. the owner and its clocks; 3. the rider-GPS and boot-ID reset again (clean
//   slate); 4. the owner-change counter, one line. The caller applies the frame only after this returns. A token
//   ends here. Inputs: st, h, s, now. Side effects: st, and the hooks.
static inline void mtxGrant(MtxRxState* st, const MtxRxHooks* h, int8_t s, uint32_t now)
{
  const uint32_t g = mtxStampOf(now);
  st->grant_ms = g;
  h->publish_limit(h->ctx, g);
  st->owner              = s;
  st->owner_heard_ms     = g;
  st->owner_idle_base_ms = g;
  st->owner_armed_seen   = false;
  st->owner_last_thr     = 0;
  st->token.slot         = kMtxNone;
  st->token.ms           = 0;
  st->claimable          = MTX_CL_NO;
  st->claimable_since_ms = 0;
  h->reset_rider_gps(h->ctx);
  h->reset_boot_id(h->ctx);
  h->owner_changed(h->ctx);
  st->n.grants++;
  h->event(h->ctx, MTX_EV_TAKES_CONTROL, s, 0);
}

// mtxClaim - a deliberate claim by slot s (a squeeze from zero, R14, or a summon, R15). If someone owns: the claim
//   notice (that remote hears SEIZED when it is next heard) and the release. Then the grant at once if nothing is armed
//   any more, else a claim token for s (Follow-Me idles on the loop's next tick; the token lets the grant land on s's
//   next control frame without a second zero proof). An existing token of s is never restamped (audit X-5).
//   V2.5-Evo - 2026-10-20 - Y-4: the claim line is emitted LAST, after the release and the grant (or the token).
//   It used to come first, so a slow event hook would have delayed the throttle zero; C6 is "zero first".
//   Inputs: st, h, s, now. Side effects: st, and the hooks.
static inline void mtxClaim(MtxRxState* st, const MtxRxHooks* h, int8_t s, uint32_t now)
{
  const int8_t o = st->owner;
  st->n.claims++;
  if (o != kMtxNone)
  {
    st->claim_notice[o]    = true;
    st->seized_on[o]       = true;
    st->seized_until_ms[o] = now + kMtxSeizedHoldMs;
    mtxRelease(st, h, MTX_REL_CLAIM, now);
  }
  MtxRxEnv e;
  h->read_env(h->ctx, &e);
  if (mtxGrantGatesOk(&e))
  {
    mtxGrant(st, h, s, now);
  }
  else if (st->token.slot != s)
  {
    st->token.slot = s;
    st->token.ms   = mtxStampOf(now);
    h->event(h->ctx, MTX_EV_TOKEN, s, 0);
  }
  h->event(h->ctx, MTX_EV_CLAIM, s, (o == kMtxNone) ? (uint8_t)0xFF : (uint8_t)o);   // Y-4: after the release
}

// mtxClaimableEdge - one line on each change of the claimable state (R9, R10). Side effects: st->claimable*, events.
static inline void mtxClaimableEdge(MtxRxState* st, const MtxRxCfg* c, const MtxRxHooks* h, const MtxRxEnv* e,
                                    uint32_t now)
{
  const uint8_t r = mtxClaimableReason(st, c, e, now);
  if (r == st->claimable) return;
  const uint8_t was = st->claimable;
  st->claimable          = r;
  st->claimable_since_ms = (r != MTX_CL_NO) ? mtxStampOf(now) : 0;
  if (r != MTX_CL_NO)
  {
    h->event(h->ctx, MTX_EV_CLAIMABLE, st->owner, r);
  }
  else
  {
    const uint8_t cause = !mtxPresent(st, c, now) ? MTX_NC_ALONE : (was == MTX_CL_SILENT ? MTX_NC_HEARD : MTX_NC_INPUT);
    h->event(h->ctx, MTX_EV_NOT_CLAIMABLE, st->owner, cause);
  }
}

// mtxTimers - run on every radio-task wake (each frame, before acting on it, and the 2 s semaphore timeout). It
//   RELEASES NOTHING: it clears a claim token older than 1 s (R18), keeps the idle clock (X-2) and prints the
//   claimable edges. Motor safety never depends on it - the gate shuts on its own clock. Side effects: st, events.
//   V2.5-Evo - 2026-10-20 - Y-3: THE STAMPS EXPIRE. mtxAgeMs() reads a stamp 2^31 ms or more old (24.8 days) as
//   age 0, so a remote whose last two frames were zeros and that was then unheard for 24.8 days of uptime would
//   have been granted on its first squeeze with no fresh zeros, and a remote heard once that long ago would have
//   counted as "present". Each slot's zero proof and its "control heard" stamp are therefore cleared here once they
//   are older than kMtxPresentMs (3 s, the longest age any rule accepts). This runs on every wake (at least every
//   2 s), so no stamp ever reaches the aliasing age. Nothing a rule can see changes: every reader already treats
//   an age above 3 s as "not fresh". The owner's own clocks are left alone: they alias toward "not claimable".
static inline void mtxTimers(MtxRxState* st, const MtxRxCfg* c, const MtxRxHooks* h, uint32_t now)
{
  if (c->single) return;
  for (uint8_t i = 0; i < kMtxSlots; i++)
  {
    if (st->last_zero_ms[i] != 0 && mtxAgeMs(now, st->last_zero_ms[i]) > kMtxPresentMs)
    {
      st->zero_streak[i]  = 0;
      st->last_zero_ms[i] = 0;
    }
    if (st->ctrl_heard_ms[i] != 0 && mtxAgeMs(now, st->ctrl_heard_ms[i]) > kMtxPresentMs) st->ctrl_heard_ms[i] = 0;
  }
  if (mtxTokenHeld(st) && mtxAgeMs(now, st->token.ms) > kMtxClaimGrantMs)
  {
    const int8_t holder = st->token.slot;
    st->token.slot = kMtxNone;
    st->token.ms   = 0;
    st->n.tokens_lapsed++;
    h->event(h->ctx, MTX_EV_TOKEN_LAPSED, holder, 0);
  }
  MtxRxEnv e;
  h->read_env(h->ctx, &e);
  mtxIdleTrack(st, &e, now);
  mtxClaimableEdge(st, c, h, &e, now);
}

// mtxOnFrame - THE PER-PACKET RULES for one 6-byte frame that passed the CRC, gate 0 and the lookup (design 7.2 with
//   v3.2 3.8). The caller has found slot s (mtxLookupSlot) and says whether it is the legacy entry.
//   SINGLE: today's path - control (and LOCKED as 0/127) to apply_owner + reply 000, metas to meta, SEIZE / SUMMON /
//           0xF5-0xFF dropped. No ownership state is touched.
//   MULTI:
//     CTRL   - owner: applied. A token holder once nothing is armed: grant, then applied (R16). A squeeze from zero
//              on a claimable owner: claim (R14). A squeeze from zero on a free buggy: grant (R1). Otherwise the
//              non-owner status. The grant and claim tests are read BEFORE the zero-proof update. Every CTRL replies.
//     LOCKED - never zero proof; the holder's token ends (X-6); the owner's lock releases (R5); the non-owner status
//              (X-6). Replies.
//     SEIZE  - the master's take-back (R8): SEIZED for the guest, release, grant. No reply.
//     SUMMON - the owner or the token holder: a silent no-op (X-5). Otherwise a claim if mtxSummonOk (R15), else
//              dropped and counted "summon refused" (R19). No reply.
//     META   - the owner's: today's handlers; the silence clock; the X-1 activity. Anyone else's: dropped.
//   Inputs: st, c, h, s, legacy_entry, f[6], now. Side effects: st, and the hooks.
static inline void mtxOnFrame(MtxRxState* st, const MtxRxCfg* c, const MtxRxHooks* h, int8_t s, bool legacy_entry,
                              const uint8_t f[6], uint32_t now)
{
  const uint8_t raw = mtxKind(f[3], f[4]);
  const uint8_t k   = mtxFrameKind(f[3], f[4], c->single, legacy_entry);
  if (k == MTX_K_DROP)
  {
    st->n.dropped++;
    h->event(h->ctx, MTX_EV_DROPPED, s, raw);
    return;
  }

  if (c->single)
  {
    if (k == MTX_K_CTRL)
    {
      uint8_t g[6];
      memcpy(g, f, 6);
      if (raw == MTX_K_LOCKED) { g[3] = 0; g[4] = kMtxSteerCentre; }   // today's locked packet, stamp included
      h->apply_owner(h->ctx, s, g);
      h->reply(h->ctx, s, MTX_S_OWNER_SOLO);
    }
    else
    {
      h->meta(h->ctx, s, f);
    }
    return;
  }

  if (!mtxSlotValid(c, s))
  {
    st->n.dropped++;
    h->event(h->ctx, MTX_EV_DROPPED, s, raw);
    return;
  }
  st->heard_ms[s] = mtxStampOf(now);
  mtxTimers(st, c, h, now);
  MtxRxEnv e;
  h->read_env(h->ctx, &e);

  if (k == MTX_K_CTRL)
  {
    const uint8_t thr = f[3];
    const bool tok = (st->token.slot == s);
    const bool ok  = mtxGrantOk(st, c, &e, s, thr, now);    // BEFORE the zero-proof update
    const bool cok = mtxClaimOk(st, c, &e, s, thr, now);    // BEFORE the zero-proof update
    mtxZeroProofStep(st, s, thr, now);
    st->ctrl_heard_ms[s] = mtxStampOf(now);
    uint8_t status;
    if (st->owner == s)
    {
      mtxApplyOwner(st, c, h, s, f, now);
      status = MTX_S_OWNER;
    }
    else if (tok && mtxGrantGatesOk(&e))
    {
      mtxGrant(st, h, s, now);                              // R16
      mtxApplyOwner(st, c, h, s, f, now);
      status = MTX_S_OWNER;
    }
    else if (cok)
    {
      mtxClaim(st, h, s, now);                              // R14
      if (st->owner == s) { mtxApplyOwner(st, c, h, s, f, now); status = MTX_S_OWNER; }
      else                { status = MTX_S_FREE; }
    }
    else if (ok)
    {
      mtxGrant(st, h, s, now);                              // R1
      mtxApplyOwner(st, c, h, s, f, now);
      status = MTX_S_OWNER;
    }
    else
    {
      status = mtxNonOwnerStatus(st, c, &e, s, mtxFrameHasInput(f[3], f[4]), now);
    }
    st->last_status[s] = status;
    h->reply(h->ctx, s, status);
  }
  else if (k == MTX_K_LOCKED)
  {
    st->zero_streak[s] = 0;                                 // a locked frame is never zero proof
    if (st->token.slot == s)
    {
      st->token.slot = kMtxNone;                            // X-6: a lock ends the holder's claim
      st->token.ms   = 0;
      h->event(h->ctx, MTX_EV_TOKEN_LAPSED, s, 1);
    }
    if (st->owner == s) mtxRelease(st, h, MTX_REL_LOCK, now);   // R5
    MtxRxEnv e2;
    h->read_env(h->ctx, &e2);
    const uint8_t status = mtxNonOwnerStatus(st, c, &e2, s, false, now);
    st->last_status[s] = status;
    h->reply(h->ctx, s, status);
  }
  else if (k == MTX_K_SEIZE)
  {
    // V2.5-Evo - 2026-10-20 - Y-7 (c): any frame from the owner restarts its silence clock (design 3.1), its own
    // SEIZE too (never sent by today's remote; refused below, since an owner cannot take back from itself).
    if (s == st->owner) mtxOwnerHeard(st, c, now);
    if (mtxSeizeOk(st, c, &e, s, now))
    {
      const int8_t o = st->owner;
      st->seized_on[o]       = true;
      st->seized_until_ms[o] = now + kMtxSeizedHoldMs;
      // V2.5-Evo - 2026-10-20 - Y-7 (b): the claim notice, as a claim leaves it. A guest that was silent at the
      // take-back and comes back after the 5 s hold still hears SEIZED ("tA") on its first frame, not BUSY.
      st->claim_notice[o]    = true;
      mtxRelease(st, h, MTX_REL_SEIZED, now);
      mtxGrant(st, h, s, now);
    }
    else
    {
      st->n.dropped++;
      h->event(h->ctx, MTX_EV_DROPPED, s, raw);
    }
  }
  else if (k == MTX_K_SUMMON)
  {
    if (s == st->owner || st->token.slot == s)
    {
      // X-5: the rest of a 3-frame burst, or a repeat - already in control or already holding the claim.
      // V2.5-Evo - 2026-10-20 - Y-7 (c): from the owner it is still an owner frame, so the silence clock restarts.
      if (s == st->owner) mtxOwnerHeard(st, c, now);
    }
    else if (mtxSummonOk(st, c, &e, s, now))
    {
      st->n.summons++;
      mtxClaim(st, h, s, now);                              // R15
      h->event(h->ctx, MTX_EV_SUMMON, s, 0);                // Y-4: after the release and the grant
    }
    else
    {
      st->n.summons_refused++;                              // R19
      h->event(h->ctx, MTX_EV_SUMMON_REFUSED, s, 0);
    }
  }
  else   // MTX_K_META
  {
    if (st->owner == s)
    {
      mtxOwnerHeard(st, c, now);
      if (mtxOwnerActivity(f[3], f[4])) st->owner_idle_base_ms = mtxStampOf(now);
      h->meta(h->ctx, s, f);
    }
    else
    {
      st->n.non_owner_meta++;
      h->event(h->ctx, MTX_EV_NON_OWNER_META, s, f[3]);
    }
  }

  // The owner coming back ends "claimable" on this very frame, not one frame later.
  MtxRxEnv e3;
  h->read_env(h->ctx, &e3);
  mtxClaimableEdge(st, c, h, &e3, now);
}

// mtxGpsDataOk - the 15-byte rider-GPS data packet is accepted only from the slot whose announce switched the radio to
//   15 bytes, with that slot's frame address, and only while that slot owns (design 5.4). SINGLE: the address test
//   alone (today's rule). Inputs: st, c, announce_slot, pkt_addr (bytes 0-2), slot_addr (that slot's uplink address).
static inline bool mtxGpsDataOk(const MtxRxState* st, const MtxRxCfg* c, int8_t announce_slot,
                                const uint8_t pkt_addr[3], const uint8_t slot_addr[3])
{
  if (!mtxIdEqual(pkt_addr, slot_addr)) return false;
  if (c->single) return true;
  return announce_slot >= 0 && announce_slot == st->owner;
}

// ============================================================
// 11. THE REMOTE: STATUS, RATE, LINK, AUTO-LOCK (design 9.3, 9.4, 9.6, 9.11)
// ============================================================
struct MtxTxStatus {
  uint8_t  last[2];        // the last two valid statuses, newest first
  uint32_t last_ms[2];     // their arrival stamps
  uint8_t  count;          // 0, 1 or 2
  uint8_t  non_owner_run;  // non-owner replies in a row
  uint8_t  seized_run;     // SEIZED replies in a row
  bool     owner;          // the newest status says "you control"
  bool     lost_owner;     // 3 non-owner replies in a row, or 2 SEIZED: slow down, disarm locally
};
static const uint8_t MTX_TXE_BECAME_OWNER  = 0x01;
static const uint8_t MTX_TXE_LOST_OWNER    = 0x02;   // rising edge: disarm the remote's own FM/RTM state and screen
static const uint8_t MTX_TXE_SEIZED_NOTICE = 0x04;   // 2 SEIZED in a row: "tA" for 3 s + one buzz

static inline void mtxTxStatusInit(MtxTxStatus* t) { memset(t, 0, sizeof(*t)); }

// mtxTxOnReply - one valid reply's status (the reserved 7 reads as BUSY). One owner status is enough to be owner.
//   Returns: MTX_TXE_* bits. Side effects: *t.
static inline uint8_t mtxTxOnReply(MtxTxStatus* t, uint8_t status, uint32_t now)
{
  const uint8_t s = (status == MTX_S_RESERVED) ? MTX_S_BUSY : (uint8_t)(status & 0x07);
  t->last[1] = t->last[0];   t->last_ms[1] = t->last_ms[0];
  t->last[0] = s;            t->last_ms[0] = mtxStampOf(now);
  if (t->count < 2) t->count++;
  uint8_t ev = 0;
  if (mtxStatusIsOwner(s))
  {
    if (!t->owner) ev |= MTX_TXE_BECAME_OWNER;
    t->owner = true; t->lost_owner = false; t->non_owner_run = 0; t->seized_run = 0;
    return ev;
  }
  t->owner = false;
  if (t->non_owner_run < 255) t->non_owner_run++;
  if (s == MTX_S_SEIZED) { if (t->seized_run < 255) t->seized_run++; }
  else t->seized_run = 0;
  if (s == MTX_S_SEIZED && t->seized_run == kMtxLoseOwnerSeized) ev |= MTX_TXE_SEIZED_NOTICE;
  if (!t->lost_owner && (t->non_owner_run >= kMtxLoseOwnerReplies || t->seized_run >= kMtxLoseOwnerSeized))
  {
    t->lost_owner = true;
    ev |= MTX_TXE_LOST_OWNER;
  }
  return ev;
}

// mtxTxFast - the normal 100 ms rate, or slow (1 in 10)? Fast unless ALL of: lost owner, no meta pending, no gesture
//   armed (the take-back, or a summon gesture that started while FREE), a valid reply within 5 s, and no input in the
//   last 3 s WHILE THE LATEST STATUS IS FREE. While it is BUSY, SEIZABLE, REFUSED or SEIZED, input does not make the
//   remote fast (it cannot take control then). Inputs: t, now, meta_pending, gesture_armed, last_input_ms (any
//   throttle >= 8, steering beyond the deadzone, or a toggle press). Returns: true = fast. Side effects: none.
static inline bool mtxTxFast(const MtxTxStatus* t, uint32_t now, bool meta_pending, bool gesture_armed,
                             uint32_t last_input_ms)
{
  if (!t->lost_owner) return true;
  if (meta_pending || gesture_armed) return true;
  if (t->count == 0 || !mtxWithin(now, t->last_ms[0], kMtxNoReplyFastMs)) return true;
  if (t->last[0] == MTX_S_FREE && mtxWithin(now, last_input_ms, kMtxFastHoldMs)) return true;
  return false;
}

// mtxTxCycleSends - while slow, only 1 cycle in 10 transmits; skipped cycles do not transmit, do not wait for a reply
//   and are not scored. Inputs: slow; *ctr (the remote's cycle counter). Returns: true = transmit this cycle.
static inline bool mtxTxCycleSends(bool slow, uint8_t* ctr)
{
  if (!slow) { *ctr = 0; return true; }
  const bool send = (*ctr == 0);
  *ctr = (uint8_t)((*ctr + 1) % kMtxSlowCycles);
  return send;
}

// mtxTxSlowMissHopMs - while slow, a missed reply shifts the next send by 33 or 66 ms, never 0 and never the
//   100 -> 200 ms downgrade, so a slow sender cannot sit in the owner's slot. Input: a random number. Returns: ms.
static inline uint8_t mtxTxSlowMissHopMs(uint32_t rnd) { return (rnd & 1u) ? (uint8_t)66 : (uint8_t)33; }

// mtxTxLinkFresh / mtxTxPatternBAllowed - "connected" means a valid reply under 1 s ago (2.5 s while slow); while slow
//   the urgent link-lost buzz (Pattern B) is not played. Side effects: none.
static inline bool mtxTxLinkFresh(uint32_t now, uint32_t last_packet_ms, bool slow)
{
  if (last_packet_ms == 0) return false;
  return mtxAgeMs(now, last_packet_ms) < (slow ? kMtxLinkFreshSlowMs : kMtxLinkFreshMs);
}
static inline bool mtxTxPatternBAllowed(bool slow) { return !slow; }

// mtxTxAutolockTimeoutS - which auto-lock timeout applies: a non-owner multi-remote status (FREE, BUSY, SEIZABLE,
//   REFUSED, SEIZED) -> mtx_autolock_s (default 120 s); owner, single remote, or no reply ever -> autolock_timeout_s
//   (default 0 = off). Returns: seconds (0 = off). Side effects: none.
static inline uint16_t mtxTxAutolockTimeoutS(const MtxTxStatus* t, uint16_t mtx_autolock_s, uint16_t autolock_timeout_s)
{
  if (t->count == 0 || mtxStatusIsOwner(t->last[0])) return autolock_timeout_s;
  return mtx_autolock_s;
}

// mtxTxAutolockSkip / mtxTxAutolockDue - auto-lock never runs while Follow-Me or RTM is armed, the take-back gesture
//   is armed, in a menu or in setup, or with no_lock set; otherwise it is due after timeout_s without input.
static inline bool mtxTxAutolockSkip(bool fm_armed, bool rtm_armed, bool gesture_armed, bool in_menu, bool in_setup,
                                     bool no_lock)
{
  return fm_armed || rtm_armed || gesture_armed || in_menu || in_setup || no_lock;
}
static inline bool mtxTxAutolockDue(uint16_t timeout_s, uint32_t now, uint32_t last_input_ms, bool skip)
{
  if (timeout_s == 0 || skip) return false;
  return mtxAgeMs(now, last_input_ms) >= (uint32_t)timeout_s * 1000u;
}

// mtxTxOwnerWait - after a SEIZE or SUMMON burst: did an owner status arrive after the send, within window_ms?
//   Returns: MTX_W_OWNER, MTX_W_WAIT (keep waiting) or MTX_W_TIMEOUT ("St"). Side effects: none.
static const uint8_t MTX_W_WAIT    = 0;
static const uint8_t MTX_W_OWNER   = 1;
static const uint8_t MTX_W_TIMEOUT = 2;
static inline uint8_t mtxTxOwnerWait(const MtxTxStatus* t, uint32_t sent_ms, uint32_t now, uint32_t window_ms)
{
  if (t->count > 0 && mtxStatusIsOwner(t->last[0]) && (int32_t)(t->last_ms[0] - sent_ms) >= 0
      && mtxAgeMs(t->last_ms[0], sent_ms) <= window_ms) return MTX_W_OWNER;
  return (mtxAgeMs(now, sent_ms) > window_ms) ? MTX_W_TIMEOUT : MTX_W_WAIT;
}

// ============================================================
// 12. THE MASTER'S TAKE-BACK: VERDICT AND GESTURE (design 9.7)
// ============================================================
// mtxTxSeizeVerdict - the last two valid statuses were both SEIZABLE and the newer is at most 5 s old. Read at the
//   COMPLETION of the hold. Side effects: none.
static inline bool mtxTxSeizeVerdict(const MtxTxStatus* t, uint32_t now)
{
  return t->count >= 2 && t->last[0] == MTX_S_SEIZABLE && t->last[1] == MTX_S_SEIZABLE
      && mtxWithin(now, t->last_ms[0], kMtxSeizeVerdictMs);
}

// The gesture: trigger released, a RIGHT tap (< 1 s), RIGHT pressed again within 400 ms and held for hold_ms.
struct MtxSeizeGesture {
  uint8_t  phase;
  uint32_t t_ms;
  bool     suppress;   // the second press must not cycle the display, until it is released
};
static const uint8_t MTX_SG_IDLE = 0, MTX_SG_TAP = 1, MTX_SG_GAP = 2, MTX_SG_HOLD = 3, MTX_SG_WAIT_RELEASE = 4;
static const uint8_t MTX_SGO_ARMED    = 0x01;   // the gesture is in progress: be fast, skip auto-lock
static const uint8_t MTX_SGO_SUPPRESS = 0x02;   // this press must not run the 2 s RIGHT-hold display cycle
static const uint8_t MTX_SGO_FIRE     = 0x04;   // queue the SEIZE burst now
static const uint8_t MTX_SGO_ABORT    = 0x08;   // the gesture was aborted (throttle, early release, lock, status, verdict)

static inline void mtxSeizeGestureInit(MtxSeizeGesture* g) { memset(g, 0, sizeof(*g)); }

// mtxSeizeGestureStep - one poll of the take-back gesture. Inputs: g; now; right_down (the RIGHT toggle is pressed);
//   thr_scaled; locked; t (the remote's statuses); hold_ms (mtx_seize_hold_ms). Returns: MTX_SGO_* bits.
//   Starts only at a RIGHT press while the latest status is SEIZABLE (and then makes the remote fast at once). Aborts on
//   thr_scaled >= 5, a lock, a status other than SEIZABLE, or a release before hold_ms. At hold_ms the verdict is read
//   again: FIRE if it stands, else ABORT. Side effects: *g.
static inline uint8_t mtxSeizeGestureStep(MtxSeizeGesture* g, uint32_t now, bool right_down, uint8_t thr_scaled,
                                          bool locked, const MtxTxStatus* t, uint32_t hold_ms)
{
  const bool seizable = (t->count > 0 && t->last[0] == MTX_S_SEIZABLE);
  if (g->phase == MTX_SG_TAP || g->phase == MTX_SG_GAP || g->phase == MTX_SG_HOLD)
  {
    if (thr_scaled >= kMtxSeizeAbortThr || locked || !seizable)
    {
      const uint8_t out = (uint8_t)(MTX_SGO_ABORT | (g->suppress ? MTX_SGO_SUPPRESS : 0));
      g->phase = right_down ? MTX_SG_WAIT_RELEASE : MTX_SG_IDLE;
      if (!right_down) g->suppress = false;
      return out;
    }
  }
  switch (g->phase)
  {
    case MTX_SG_IDLE:
      if (right_down && seizable && thr_scaled < kMtxSeizeAbortThr && !locked)
      {
        g->phase = MTX_SG_TAP; g->t_ms = mtxStampOf(now); g->suppress = false;
        return MTX_SGO_ARMED;
      }
      return 0;
    case MTX_SG_TAP:
      if (right_down)
      {
        if (mtxAgeMs(now, g->t_ms) >= kMtxTapMaxMs) { g->phase = MTX_SG_WAIT_RELEASE; return 0; }   // a hold, not a tap
        return MTX_SGO_ARMED;
      }
      g->phase = MTX_SG_GAP; g->t_ms = mtxStampOf(now);
      return MTX_SGO_ARMED;
    case MTX_SG_GAP:
      if (right_down)
      {
        if (mtxAgeMs(now, g->t_ms) <= kMtxSeizeTapGapMs)
        {
          g->phase = MTX_SG_HOLD; g->t_ms = mtxStampOf(now); g->suppress = true;
          return (uint8_t)(MTX_SGO_ARMED | MTX_SGO_SUPPRESS);
        }
        g->phase = MTX_SG_TAP; g->t_ms = mtxStampOf(now);   // too late for a second press: a new first press
        return MTX_SGO_ARMED;
      }
      if (mtxAgeMs(now, g->t_ms) > kMtxSeizeTapGapMs) { g->phase = MTX_SG_IDLE; return 0; }   // lapsed quietly
      return MTX_SGO_ARMED;
    case MTX_SG_HOLD:
      if (!right_down)
      {
        g->phase = MTX_SG_IDLE; g->suppress = false;
        return MTX_SGO_ABORT;                                // released before the hold time
      }
      if (mtxAgeMs(now, g->t_ms) >= hold_ms)
      {
        g->phase = MTX_SG_WAIT_RELEASE;
        return (uint8_t)(MTX_SGO_SUPPRESS | (mtxTxSeizeVerdict(t, now) ? MTX_SGO_FIRE : MTX_SGO_ABORT));
      }
      return (uint8_t)(MTX_SGO_ARMED | MTX_SGO_SUPPRESS);
    default:   // MTX_SG_WAIT_RELEASE
      if (right_down) return g->suppress ? MTX_SGO_SUPPRESS : (uint8_t)0;
      g->phase = MTX_SG_IDLE; g->suppress = false;
      return 0;
  }
}

// ============================================================
// 13. THE SUMMON GATE ON THE REMOTE (design v3.2 3.4.2)
// ============================================================
struct MtxSummonIn {
  bool     gps_en;            // usrConf.gps_en
  bool     loc_valid;         // gps_tx.location.isValid()
  uint32_t loc_age_ms;        // gps_tx.location.age()
  uint16_t gps_stale_ms;      // usrConf.tx_gps_stale_timeout_ms
  bool     trigger_released;  // triggerReleased()
  bool     feature_enabled;   // RTM: rtmEnabledEffective(); Follow-Me: fm_override_enabled
  bool     locked;            // system_locked
  bool     in_setup;          // a settings / calibration screen (not the gesture handler's own menu state)
};
static const uint8_t MTX_SV_OK       = 0;
static const uint8_t MTX_SV_NOT_FREE = 1;   // the latest status is not FREE, or is older than 5 s
static const uint8_t MTX_SV_NO_FIX   = 2;   // no GPS, a module without a fix, or a stale fix: manual control only
static const uint8_t MTX_SV_TRIGGER  = 3;   // the trigger is not released
static const uint8_t MTX_SV_DISABLED = 4;   // the feature is off or gps_en is 0
static const uint8_t MTX_SV_LOCKED   = 5;
static const uint8_t MTX_SV_SETUP    = 6;

// mtxTxFixFresh - THE SAME TEST that lets the remote send its rider-GPS stream: gps_en, a valid fix, and an age under
//   tx_gps_stale_timeout_ms. A remote that would not stream may not summon. Side effects: none.
static inline bool mtxTxFixFresh(bool gps_en, bool loc_valid, uint32_t loc_age_ms, uint16_t gps_stale_ms)
{
  return gps_en && loc_valid && loc_age_ms < (uint32_t)gps_stale_ms;
}

// mtxSummonVerdict - may this remote (not in control) summon with its RTM or Follow-Me gesture? In order: the status
//   (FREE, at most 5 s old), a fresh own fix, then today's gesture preconditions. Any refusal: "St" + the stop buzz and
//   NOTHING is sent. Inputs: t, in, now. Returns: MTX_SV_*. Side effects: none.
static inline uint8_t mtxSummonVerdict(const MtxTxStatus* t, const MtxSummonIn* in, uint32_t now)
{
  if (!(t->count > 0 && t->last[0] == MTX_S_FREE && mtxWithin(now, t->last_ms[0], kMtxSeizeVerdictMs)))
    return MTX_SV_NOT_FREE;
  if (!mtxTxFixFresh(in->gps_en, in->loc_valid, in->loc_age_ms, in->gps_stale_ms)) return MTX_SV_NO_FIX;
  if (!in->trigger_released)                   return MTX_SV_TRIGGER;
  if (!in->feature_enabled || !in->gps_en)     return MTX_SV_DISABLED;
  if (in->locked)                              return MTX_SV_LOCKED;
  if (in->in_setup)                            return MTX_SV_SETUP;
  return MTX_SV_OK;
}

// ============================================================
// 14. THE REMOTE'S BOUNDED TELEMETRY WAIT (design 9.12)
// ============================================================
static const uint8_t MTX_RW_ACCEPT   = 1;   // our reply: take it
static const uint8_t MTX_RW_RELISTEN = 2;   // someone else's frame: startReceive() again and keep waiting
static const uint8_t MTX_RW_STOP     = 3;   // the 40 ms bound has passed: stop, and do NOT touch the radio again

// mtxReplyWaitOpen - the wait is open strictly before send_ms + 40. After that no radio call may be made.
static inline bool mtxReplyWaitOpen(uint32_t now, uint32_t send_ms)
{
  return (int32_t)(now - (send_ms + kMtxReplyWaitMs)) < 0;
}
// mtxReplyWaitOnFrame - a frame arrived at now; frame_ours = its address and CRC match this remote's reply.
static inline uint8_t mtxReplyWaitOnFrame(uint32_t now, uint32_t send_ms, bool frame_ours)
{
  if (!mtxReplyWaitOpen(now, send_ms)) return MTX_RW_STOP;
  return frame_ours ? MTX_RW_ACCEPT : MTX_RW_RELISTEN;
}
// mtxTxReplyAddr - the address a reply to this remote carries: its chip ID (proto 3) or its own_address (proto 0).
static inline const uint8_t* mtxTxReplyAddr(uint16_t proto, const uint8_t efuse_id[3], const uint8_t own_address[3])
{
  return (proto == kMtxProtoV3) ? efuse_id : own_address;
}

// ============================================================
// 15. PAIRING (design 11.1, 11.2): THE BUGGY'S WINDOW AND THE REMOTE'S OFFER GATE
// ============================================================
struct MtxPairWindow {
  uint32_t     open_ms;
  uint32_t     close_at_ms;
  uint8_t      n;                       // enrolled in THIS window (the list is rebuilt, never appended)
  uint8_t      ids[kMtxSlots * 3];
  uint8_t      flags[kMtxSlots];
  bool         closed;
  uint8_t      close_reason;
  MtxRxBinding snapshot;                // the binding before the window, restored on an abort
};
static const uint8_t MTX_PR_OFFER       = 0;   // answer (0xDA, or 0xBA for a legacy request)
static const uint8_t MTX_PR_ZERO        = 1;   // the ID is 00:00:00
static const uint8_t MTX_PR_RX_ID       = 2;   // the ID is the buggy's own
static const uint8_t MTX_PR_FULL        = 3;   // 4 enrolled and this one is not among them
static const uint8_t MTX_PR_RSSI        = 4;   // weaker than -60 dBm
static const uint8_t MTX_PR_LEGACY_LATE = 5;   // a legacy request after any enrolment (a legacy remote may only be the one remote)
static const uint8_t MTX_PR_CLOSED      = 6;
static const uint8_t MTX_PC_TIME = 1, MTX_PC_FULL = 2, MTX_PC_LEGACY = 3, MTX_PC_BIND = 4;

// mtxPairOpen - open the window: snapshot the binding, empty list, 60 s. Side effects: *w.
static inline void mtxPairOpen(MtxPairWindow* w, const MtxRxBinding* current, uint32_t now)
{
  memset(w, 0, sizeof(*w));
  w->snapshot    = *current;
  w->open_ms     = now;
  w->close_at_ms = now + kMtxPairWindowMs;
}

static inline int8_t mtxPairFind(const MtxPairWindow* w, const uint8_t id[3])
{
  for (uint8_t i = 0; i < w->n; i++) if (mtxIdEqual(&w->ids[i * 3], id)) return (int8_t)i;
  return kMtxNone;
}

// mtxPairOnRequest - answer a pairing request or refuse it (one serial line per refusal). Inputs: w, legacy (0xAB vs
//   0xAD), id, rx_id, rssi_dbm. Returns: MTX_PR_*. Side effects: none.
static inline uint8_t mtxPairOnRequest(const MtxPairWindow* w, bool legacy, const uint8_t id[3], const uint8_t rx_id[3],
                                       float rssi_dbm)
{
  if (w->closed) return MTX_PR_CLOSED;
  if (legacy && w->n > 0) return MTX_PR_LEGACY_LATE;
  if (mtxIdIsZero(id)) return MTX_PR_ZERO;
  if (mtxIdEqual(id, rx_id)) return MTX_PR_RX_ID;
  if (w->n >= kMtxSlots && mtxPairFind(w, id) == kMtxNone) return MTX_PR_FULL;
  if (rssi_dbm < (float)kMtxPairMinRssiDbm) return MTX_PR_RSSI;
  return MTX_PR_OFFER;
}

// mtxPairOnConfirm - the matching confirm arrived (it names this buggy and that remote, within 1.5 s): enrol, or
//   refresh an ID already enrolled in this window. A NEW enrolment extends the window by 20 s, never past 120 s from
//   the open. A legacy enrolment, or the fourth, closes it. Returns: the slot. Side effects: *w.
//   V2.5-Evo - 2026-10-20 - Y-2: THE BOUNDS GUARD. A new ID is never enrolled into a closed window or past the
//   fourth slot: it returns kMtxNone and writes nothing. Before this the function relied on its caller, and a fifth
//   confirm would have written past ids[] into flags[] and `closed`, and the commit would have copied 15 bytes into
//   the 12-byte list. A refresh of an ID already enrolled still answers its slot and writes nothing.
static inline int8_t mtxPairOnConfirm(MtxPairWindow* w, bool legacy, const uint8_t id[3])
{
  const int8_t found = mtxPairFind(w, id);
  if (found != kMtxNone) return found;
  if (w->closed || w->n >= kMtxSlots) return kMtxNone;
  const uint8_t i = w->n++;
  memcpy(&w->ids[i * 3], id, 3);
  w->flags[i] = (uint8_t)(kMtxFlagValid | (legacy ? kMtxFlagLegacy : 0));
  uint32_t ext = w->close_at_ms + kMtxPairExtendMs;
  if (mtxAgeMs(ext, w->open_ms) > kMtxPairMaxMs) ext = w->open_ms + kMtxPairMaxMs;
  w->close_at_ms = ext;
  if (legacy)               { w->closed = true; w->close_reason = MTX_PC_LEGACY; }
  else if (w->n >= kMtxSlots) { w->closed = true; w->close_reason = MTX_PC_FULL; }
  return (int8_t)i;
}

// mtxPairPoll - close on the window time or a fresh BIND press. Returns: true once closed. Side effects: *w.
static inline bool mtxPairPoll(MtxPairWindow* w, uint32_t now, bool bind_pressed_again)
{
  if (w->closed) return true;
  if (bind_pressed_again)                    { w->closed = true; w->close_reason = MTX_PC_BIND; }
  else if ((int32_t)(now - w->close_at_ms) >= 0) { w->closed = true; w->close_reason = MTX_PC_TIME; }
  return w->closed;
}

// mtxPairCommit - at close: with at least one enrolment, the new list in enrolment order (slot 0 = the first = the
//   master), dest_address = the legacy remote's address if it is the sole entry, else 00:00:00, own_address = rx_id,
//   paired = 1 -> returns true (the caller saves once). With none: the snapshot is restored -> false, nothing written.
static inline bool mtxPairCommit(const MtxPairWindow* w, MtxRxBinding* b, const uint8_t rx_id[3])
{
  if (w->n == 0) { *b = w->snapshot; return false; }
  memset(b->mtx_id, 0, sizeof(b->mtx_id));
  memset(b->mtx_flags, 0, sizeof(b->mtx_flags));
  memcpy(b->mtx_id, w->ids, (size_t)w->n * 3);
  memcpy(b->mtx_flags, w->flags, w->n);
  if (w->n == 1 && (w->flags[0] & kMtxFlagLegacy)) memcpy(b->dest_address, w->ids, 3);
  else memset(b->dest_address, 0, 3);
  memcpy(b->own_address, rx_id, 3);
  b->paired = 1;
  return true;
}

// mtxPairOfferOk - the remote takes an OFFER (0xDA) only if it names this remote, carries a non-zero buggy ID and is at
//   least -60 dBm (the caller has checked the CRC). Side effects: none.
static inline bool mtxPairOfferOk(const uint8_t named_tx[3], const uint8_t my_id[3], const uint8_t rx_id[3],
                                  float rssi_dbm)
{
  return mtxIdEqual(named_tx, my_id) && !mtxIdIsZero(rx_id) && rssi_dbm >= (float)kMtxPairMinRssiDbm;
}

// ============================================================
// 16. SETTING CLAMPS (the validators' range rule)
// ============================================================
static inline uint16_t mtxClampU16(uint16_t v, uint16_t lo, uint16_t hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

// mtxClampRxConfig - mtx_claim_s 6-60 s, mtx_seize_en 0-1, mtx_takeover_cap 0-255, mtx_takeover_ms 0-10000 ms.
static inline void mtxClampRxConfig(uint16_t* claim_s, uint16_t* seize_en, uint16_t* takeover_cap, uint16_t* takeover_ms)
{
  *claim_s      = mtxClampU16(*claim_s, kMtxClaimSMin, kMtxClaimSMax);
  *seize_en     = (*seize_en != 0) ? 1 : 0;
  *takeover_cap = mtxClampU16(*takeover_cap, 0, kMtxTakeoverCapMax);
  *takeover_ms  = mtxClampU16(*takeover_ms, 0, kMtxTakeoverMsMax);
}

// mtxClampTxConfig - autolock_timeout_s 0 or 5-180 s, mtx_seize_hold_ms 1000-1800 ms, mtx_autolock_s 0 or 30-600 s.
static inline void mtxClampTxConfig(uint16_t* autolock_timeout_s, uint16_t* seize_hold_ms, uint16_t* mtx_autolock_s)
{
  if (*autolock_timeout_s != 0)
    *autolock_timeout_s = mtxClampU16(*autolock_timeout_s, kMtxAutolockTimeoutSMin, kMtxAutolockTimeoutSMax);
  *seize_hold_ms = mtxClampU16(*seize_hold_ms, kMtxSeizeHoldMsMin, kMtxSeizeHoldMsMax);
  if (*mtx_autolock_s != 0)
    *mtx_autolock_s = mtxClampU16(*mtx_autolock_s, kMtxAutolockSMin, kMtxAutolockSMax);
}

// ============================================================
// 17. THE REMOTE IN CONTROL: THE ACTIVITY REPEAT, THE REFUSAL SIGNAL, THE OWNERSHIP DOT (step 3, TX SW29)
// ============================================================
// V2.5-Evo - 2026-10-20 - THE ACTIVITY REPEAT (audit X-1 option (a), Y-6; owner 2026-10-20: "the remote in control is
//   busy on throttle or any button / toggle press"). A rider who presses buttons at zero throttle, or arms Return-To-Me
//   through the toggle and its ceremony, sends throttle 0 / steering 127 the whole time, so to the buggy he looks idle
//   and another remote could claim him mid-gesture. The remote in control therefore repeats the "RTM armed, not yet
//   active" meta 0xF1/0 - which every buggy since P7 treats as a no-op while RTM is not running, and which the buggy's
//   mtxOwnerActivity() counts as activity - while a press or a ceremony is in progress.
//   THE RULES (step-1 audit Q3 items 1-6, step-2 audit section 6):
//   - from the FIRST press (the window opens at once, the first repeat is due at once), through the press, the gap to
//     a combo's second press (the window stays open kMtxActHoldMs after the last press) and the whole RTM ceremony;
//   - at most every kMtxActRepeatMs (2.0 s; the audit: 2.5 s or less, so one lost packet never leaves a 6 s gap);
//   - only on a new-style link (proto 3), only while the latest status is OWNER (S = 001: in control with other
//     remotes paired) - never in SINGLE (S = 000), so one remote stays byte-identical; never for a non-owner, whose
//     metas the buggy drops anyway (a non-owner claims only by a squeeze from zero);
//   - never while locked or in setup, never while RTM is ACTIVE on this remote (an 0xF1/0 would end the return), never
//     while an 0xF1/1 is queued, and not while the throttle byte is 8 or more (the control frames already show it).
//   The caller queues it with queueMetaPacketIfFree() (one packet, a FREE slot only, refused while any 0xF1 is
//   pending), so a repeat can never rewrite a queued 0xF1/1 into an 0xF1/0; and the ceremony sets RTM active before it
//   queues 0xF1/1, so no repeat can follow it either.
static const uint32_t kMtxActRepeatMs = 2000;   // repeat period while the window is open (<= 2.5 s)
static const uint32_t kMtxActHoldMs   = 3000;   // the window stays open this long after the last press (= the combo gap)

struct MtxTxActivity {
  uint32_t until_ms;   // the window is open while now is before this; 0 = closed
  uint32_t last_ms;    // the last repeat queued in this window; 0 = none yet (the next one is due at once)
};

struct MtxTxActIn {
  bool    v3;              // mtx_link_proto == 3
  bool    owner_multi;     // the latest valid status is OWNER (001)
  bool    locked;          // system_locked
  bool    in_setup;
  bool    rtm_active;      // Return-To-Me ACTIVE on this remote
  bool    rtm_on_queued;   // an 0xF1/1 is waiting in the meta queue
  bool    arming;          // the RTM arm ceremony is running: the window is open throughout
  uint8_t thr_byte;        // the last control throttle byte sent
};

static inline void mtxTxActivityInit(MtxTxActivity* a) { a->until_ms = 0; a->last_ms = 0; }

// mtxTxActivityNote - a press: a toggle press, a toggle held, a magnet arriving or held. Opens the window (the first
//   repeat is then due at once) or keeps it open for kMtxActHoldMs more. Inputs: a, now. Side effects: *a.
static inline void mtxTxActivityNote(MtxTxActivity* a, uint32_t now)
{
  const bool open = (a->until_ms != 0) && (int32_t)(now - a->until_ms) < 0;
  if (!open) a->last_ms = 0;
  a->until_ms = mtxStampOf(now + kMtxActHoldMs);
}

// mtxTxActivityDue - should one 0xF1/0 be queued now? Closes an expired window (a ceremony keeps it open). Inputs: a,
//   in, now. Returns: true = queue one now (then call mtxTxActivitySent() if the queue took it). Side effects: *a.
static inline bool mtxTxActivityDue(MtxTxActivity* a, const MtxTxActIn* in, uint32_t now)
{
  const bool open = in->arming || ((a->until_ms != 0) && (int32_t)(now - a->until_ms) < 0);
  if (!open) { a->until_ms = 0; a->last_ms = 0; return false; }
  if (!in->v3 || !in->owner_multi || in->locked || in->in_setup || in->rtm_active || in->rtm_on_queued) return false;
  if (in->thr_byte >= kMtxZeroThr) return false;
  if (a->last_ms != 0 && mtxAgeMs(now, a->last_ms) < kMtxActRepeatMs) return false;
  return true;
}
static inline void mtxTxActivitySent(MtxTxActivity* a, uint32_t now) { a->last_ms = mtxStampOf(now); }

// V2.5-Evo - 2026-10-20 - THE REFUSAL SIGNAL (owner 2026-10-20: "if it cannot engage, show stop. That's the regular
//   response always"). A Return-To-Me or Follow-Me arm gesture that cannot engage shows "St" with the stop buzz -
//   EXCEPT a takeover try against a remote that is busy in control: that shows E9, the same answer a squeeze gets. A
//   summon is such a try: refused because the latest status says another remote is in control and not claimable
//   (BUSY, SEIZABLE, REFUSED) or has just taken control from this one (SEIZED), it is E9. A stale status (no reply for
//   5 s) is not evidence of a busy owner: "St". Inputs: t, now. Returns: MTX_RS_*. Side effects: none.
static const uint8_t MTX_RS_ST = 0;
static const uint8_t MTX_RS_E9 = 1;
static inline uint8_t mtxTxRefusalSignal(const MtxTxStatus* t, uint32_t now)
{
  if (t->count == 0 || !mtxWithin(now, t->last_ms[0], kMtxSeizeVerdictMs)) return MTX_RS_ST;
  const uint8_t s = t->last[0];
  return (s == MTX_S_BUSY || s == MTX_S_SEIZABLE || s == MTX_S_REFUSED || s == MTX_S_SEIZED) ? MTX_RS_E9 : MTX_RS_ST;
}

// V2.5-Evo - 2026-10-20 - THE OWNERSHIP DOT (24a0371, design 9.10): what C7 R2 shows when no Follow-Me dot needs it.
//   SOLID = you control and others are paired (OWNER); PULSE = control is available now (FREE); SLOW = someone else
//   controls (BUSY, SEIZABLE, REFUSED, SEIZED); OFF = an old-style link, one remote (OWNER_SOLO - so a single remote's
//   screen is today's), no reply yet, or none for 5 s (no link: nothing to act on). Inputs: t, now, v3. Returns:
//   MTX_DOT_*. Side effects: none.
static const uint8_t MTX_DOT_OFF   = 0;
static const uint8_t MTX_DOT_SOLID = 1;
static const uint8_t MTX_DOT_SLOW  = 2;
static const uint8_t MTX_DOT_PULSE = 3;
static inline uint8_t mtxTxDotState(const MtxTxStatus* t, uint32_t now, bool v3)
{
  if (!v3 || t->count == 0 || !mtxWithin(now, t->last_ms[0], kMtxNoReplyFastMs)) return MTX_DOT_OFF;
  switch (t->last[0])
  {
    case MTX_S_OWNER:    return MTX_DOT_SOLID;
    case MTX_S_FREE:     return MTX_DOT_PULSE;
    case MTX_S_BUSY:
    case MTX_S_SEIZABLE:
    case MTX_S_REFUSED:
    case MTX_S_SEIZED:   return MTX_DOT_SLOW;
    default:             return MTX_DOT_OFF;   // OWNER_SOLO: one remote, today's screen
  }
}

// mtxTxNotOwner - is this remote a NON-OWNER on a multi-remote buggy right now (the summon path, design v3.2 3.4.2)?
//   True only on a new-style link whose latest valid status is not "you control". An old-style link, one remote
//   (OWNER_SOLO), or no reply yet keep today's paths (the buggy re-judges every frame anyway). Side effects: none.
static inline bool mtxTxNotOwner(const MtxTxStatus* t, bool v3)
{
  return v3 && t->count > 0 && !mtxStatusIsOwner(t->last[0]);
}

#endif // BREMOTE_MULTI_TX_H
