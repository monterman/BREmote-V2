// V2.5-Evo - 2026-10-24 - THE THROTTLE BYTE'S FULL SCALE (throttle-byte audit T-1, fix (b)): new header. kThrCtrlMax,
//   the one 240 both boards use; thrTriggerToByte(), the remote's 0..255 -> 0..240 scaling that replaces the clip at
//   0xF0; thrByteToCmd(), the receiver's 0..240 -> 0..255 stretch at the head of calcPWM(). Pure, no Arduino code,
//   host-tested (Tools/tests/throttle_byte_test.cpp). Both main headers (BREmote_V2_Tx.h, BREmote_V2_Rx.h) include it
//   directly, next to RadioCommon.h, before any .ino code.
//
// ============================================================
// THE THROTTLE BYTE - what the remote sends, and what the receiver makes of it
// ============================================================
// A control frame carries the throttle in byte 3. Bytes 0xF1-0xF4 in that position are not throttle: they are the
// meta-packet types (0xF1 return-to-me state, 0xF2 Follow-Me, 0xF3 rider GPS, 0xF4 aux; 0xF5-0xFF are never sent by
// this firmware). So the highest throttle byte a frame may carry is 0xF0 = 240, which is 94.1 % of the byte.
//
// THE BUG THIS HEADER FIXES (bench 2026-10-24: 367 samples at 1941 us, never higher; the same clip on every branch).
// The remote CLIPPED its 0..255 throttle at 240, and the receiver kept 255 as its full scale: map(240, 0, 255, 1000,
// 2000) = 1941 us. So a full squeeze never reached 2000 us in a straight line, and the top 15 counts of trigger travel
// (about 6 %) all sent the same 240 and did nothing.
//
// THE FIX, ONE LINE ON EACH BOARD:
//   - the remote SCALES 0..255 onto 0..240 (thrTriggerToByte()) instead of clipping, so every trigger position has
//     its own byte;
//   - the receiver STRETCHES 0..240 back onto 0..255 (thrByteToCmd()) at the head of calcPWM(), before any cap.
//   Together: within 1 count of the trigger at every position (never below it), 0 -> 0, and 255 -> 255 = 2000 us.
//
// WHAT IT DOES NOT CHANGE:
//   - the frame, its length and its airtime;
//   - the receiver's thr_received: it stays the byte as received, so every release, deadman, hand-back and idle test
//     reads the byte it reads today. Those bytes are about 6 % smaller for the same squeeze: "below 8" is still a
//     trigger of 7 or less, the deadman's 25 needs a trigger of 27, and "released" (below 25) reads true up to 26;
//   - every receiver cap (the return-to-me approach cap, the Follow-Me cap, the arrival hand-back cap, the sticky
//     return cap): each is a min() applied AFTER the stretch, so a cap of c still gives map(c) exactly;
//   - every remote cap (gears, the dynamic cap, the return-to-me ramp): applied before the scaling, so the pair gives
//     their output of today to within 1 count.
//
// MIXED FLEET: flash every remote first, then the receiver. A new remote on an old receiver is 5.9 % SOFTER at every
// position (the safe direction); an old remote on a new receiver is 6.25 % hotter. This receiver has no re-press lock,
// so its only soft start in that hot state is the motor ramp: keep the remote-first order.
#ifndef BREMOTE_THROTTLE_BYTE_H
#define BREMOTE_THROTTLE_BYTE_H

#include <stdint.h>

static const uint8_t kThrCtrlMax = 0xF0;   // 240: the highest throttle byte a control frame may carry (0xF1-0xF4 are meta types)

// thrTriggerToByte - THE REMOTE: the throttle (calcFinalThrottle(), 0-255, after the arming and input-fault zeros)
//   scaled onto the control frame's 0..240, rounded to nearest.
//   Input: thr, 0-255. Returns: 0-240. 0 -> 0, 1 -> 1 (a squeeze is never sent as zero), 255 -> 240, never decreasing,
//   never above kThrCtrlMax. Side effects: none. Arithmetic: at most 255 x 240 + 127 = 61327, inside 16 bits.
static inline uint8_t thrTriggerToByte(uint8_t thr)
{
  return (uint8_t)(((uint16_t)thr * kThrCtrlMax + 127u) / 255u);
}

// thrByteToCmd - THE RECEIVER: the control frame's throttle byte stretched back onto the 0-255 motor command that
//   calcPWM() caps, ramps, mixes and maps.
//   Input: b, the byte as received, 0-255. Returns: 0-255. 0 -> 0, never decreasing, never below b, 240 or more -> 255.
//   On this branch the receiver dispatches 0xF1-0xF4 as meta packets, but takes 0xF5-0xFF as throttle (only a remote on
//   other firmware sends them); they give 255, no higher, which is no more than they gave before.
//   Side effects: none. Arithmetic: at most 239 x 255 + 120 = 61065, inside 16 bits.
//   0 -> 0 is the safety property: calcPWM()'s SAFETY-NEUTRAL-1 still sees effective_thr == 0 exactly when the byte is 0.
static inline uint8_t thrByteToCmd(uint8_t b)
{
  if (b >= kThrCtrlMax) return 255;                                               // 240 or more = full output
  return (uint8_t)(((uint16_t)b * 255u + kThrCtrlMax / 2u) / kThrCtrlMax);        // rounded; 0 -> 0, never below b
}

#endif // BREMOTE_THROTTLE_BYTE_H
