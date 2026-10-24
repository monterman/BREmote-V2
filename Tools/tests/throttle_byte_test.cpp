// V2.5-Evo - 2026-10-24 - throttle-byte audit T-1, fix (b): host test for Source/Common/ThrottleByte.h. The remote's
//   scaling and the receiver's stretch over all 256 values (0 -> 0, never decreasing, rounded, never a meta type, never
//   below the input), the round trip (0 -> 0, 255 -> 255, within 1 count: 241 exact, 15 one count up), the one
//   constant, and the PWM the pair gives against the old clip (full squeeze 1941 -> 2000 us).
// Build and run (Git Bash, from Tools/tests, PATH and TMP as in the host-test note of the compile procedure):
//   g++ -std=c++17 -Wall -Wextra -o throttle_byte_test throttle_byte_test.cpp && ./throttle_byte_test

#include <cstdio>
#include <cstdint>

#include "../../Source/Common/ThrottleByte.h"

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

// Arduino's map(), copied, as the receiver's efoil branch uses it with PWM 1000-2000.
static long arduinoMap(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
static int pwmUs(uint8_t cmd) { return (int)arduinoMap(cmd, 0, 255, 1000, 2000); }

// ---- The remote: thrTriggerToByte(), all 256 throttles ----
static void testRemote()
{
  CHECK(kThrCtrlMax == 0xF0);
  bool ok = true;
  for (int t = 0; t < 256; t++) {
    const uint8_t b = thrTriggerToByte((uint8_t)t);
    ok = ok && b <= kThrCtrlMax;                                            // never a meta type (0xF1-0xF4) or above
    ok = ok && (t == 0 ? b == 0 : b >= 1);                                  // 0 stays 0; a squeeze is never sent as 0
    const int err17 = 17 * (int)b - 16 * t;                                 // b against 240t/255 = 16t/17, in 17ths
    ok = ok && err17 >= -8 && err17 <= 8;                                   // rounded to nearest
    if (t > 0) ok = ok && b >= thrTriggerToByte((uint8_t)(t - 1));          // never decreasing
  }
  CHECK(ok);
  CHECK(thrTriggerToByte(0) == 0 && thrTriggerToByte(1) == 1 && thrTriggerToByte(254) == 239 &&
        thrTriggerToByte(255) == kThrCtrlMax);
}

// ---- The receiver: thrByteToCmd(), all 256 bytes ----
static void testReceiver()
{
  bool ok = true;
  for (int b = 0; b < 256; b++) {
    const uint8_t c = thrByteToCmd((uint8_t)b);
    ok = ok && c >= b;                                                      // never below its input
    if (b >= kThrCtrlMax) ok = ok && c == 255;                              // 240 or more = full output (0xF5-0xFF too)
    else {
      const int err = 240 * (int)c - 255 * b;                               // c against 255b/240, in 240ths
      ok = ok && c < 255 && err > -120 && err <= 120;                       // full only from 240; rounded (half up)
    }
    if (b > 0) {
      const uint8_t p = thrByteToCmd((uint8_t)(b - 1));
      ok = ok && c >= p;                                                    // never decreasing ...
      if (b <= kThrCtrlMax) ok = ok && c > p;                               // ... and every byte 0..240 has its own command
    }
  }
  CHECK(ok);
  CHECK(thrByteToCmd(0) == 0 && thrByteToCmd(1) == 1 && thrByteToCmd(8) == 9 && thrByteToCmd(239) == 254 &&
        thrByteToCmd(240) == 255 && thrByteToCmd(255) == 255);
}

// ---- The pair: the round trip, all 256 throttles: 0 -> 0, 255 -> 255, never decreasing, within 1 count ----
static void testRoundTrip()
{
  bool ok = true;
  int exact = 0, plus1 = 0;
  for (int t = 0; t < 256; t++) {
    const uint8_t r = thrByteToCmd(thrTriggerToByte((uint8_t)t));
    ok = ok && (r == t || r == t + 1);                                      // within 1 count, and never below the throttle
    if (r == t) exact++; else plus1++;
    if (t > 0) ok = ok && r >= thrByteToCmd(thrTriggerToByte((uint8_t)(t - 1)));
  }
  CHECK(ok && exact == 241 && plus1 == 15);
  CHECK(thrByteToCmd(thrTriggerToByte(0)) == 0 && thrByteToCmd(thrTriggerToByte(255)) == 255);
}

// ---- What the rider gets, PWM 1000-2000 us: the old clip against the pair (the audit's section 4 table) ----
static void testPwmTable()
{
  struct Row { uint8_t thr; int before; int fixed; };
  const Row rows[] = {
    { 255, 1941, 2000 },   // full squeeze: the finding (bench: 1941, never higher)
    { 240, 1941, 1941 },   // trigger 240: the old full squeeze, now 94 % of the travel
    { 128, 1501, 1501 },   // half trigger
    { 229, 1898, 1901 },   // gear 9 of 10
    { 216, 1847, 1847 },   // dynamic cap 85 % at a full squeeze (255 x 85 / 100)
    { 178, 1698, 1701 },   // the remote's return-to-me cap at 70 % (70 x 255 / 100 = 178)
    {   0, 1000, 1000 },   // released
  };
  for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
    const uint8_t clip = (rows[i].thr > kThrCtrlMax) ? kThrCtrlMax : rows[i].thr;   // the old remote line
    CHECK(pwmUs(clip) == rows[i].before);                                           // the old receiver: 255 = full
    CHECK(pwmUs(thrByteToCmd(thrTriggerToByte(rows[i].thr))) == rows[i].fixed);
  }
}

int main()
{
  testRemote();
  testReceiver();
  testRoundTrip();
  testPwmTable();
  if (fails) { std::printf("throttle_byte_test: %d of %d checks FAILED\n", fails, checks); return 1; }
  std::printf("throttle_byte_test: all %d checks passed\n", checks);
  return 0;
}
