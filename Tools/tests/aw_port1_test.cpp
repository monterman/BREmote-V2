// V2.5-Evo - 2026-10-14 - full-system audit FS-1 / FS-13 / FS-14 + link-layer audit F-1: host test for
//   Source/Common/AwPort1.h (the AW9523 port-1 OUTPUT writer's plan) and Source/Common/UartMuxVerify.h (the UART-mux
//   read-back verdict). The writer model below is System.ino's awPort1WriteLocked() line for line: a shadow of the
//   owned output levels, one checked read, nothing written on a failed read, and both PPM enables forced HIGH once
//   awPort1EnableMotorsLocked() has run (before that, carried as read).
//   1. For EVERY kind of port-1 write the RX makes (mux to channel 0 / 1, AUX LED on / off, wet-measure enable on / off,
//      the boot enable write, the P-6 re-assert) and EVERY one of the 256 possible OUTPUT1 read values, armed and not:
//      enables HIGH when armed and as read before, owned bits exactly as intended, the two input pins carried over,
//      nothing written on a failed read.
//   2. Sequences: the wet-measure enable is HIGH only inside checkWetness()'s window; a skipped write is repaired by
//      the next successful one from any caller; a noise-corrupted read can no longer move the mux, flip the LED or
//      clear an enable.
//   3. muxReadbackVerdict(): a failed read is never a pass (all 256 leftover bytes); a good read passes exactly when
//      both select bits match.
// Build and run (Git Bash, from Tools/tests, PATH and TMP as in SOP-012's host-test note):
//   g++ -std=c++17 -Wall -Wextra -o aw_port1_test aw_port1_test.cpp && ./aw_port1_test

#include <cstdio>
#include <cstdint>

#include "../../Source/Common/AwPort1.h"
#include "../../Source/Common/UartMuxVerify.h"

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

// The RX's AW9523 port 1 (BREmote_V2_Rx.h AP_* pins 8-15 -> bits 0-7).
static const uint8_t MUX0 = 1u << 0;   // AP_U1_MUX_0 = 8
static const uint8_t MUX1 = 1u << 1;   // AP_U1_MUX_1 = 9
static const uint8_t BTN  = 1u << 2;   // AP_S_AUX = 10 (input)
static const uint8_t LED  = 1u << 3;   // AP_L_AUX = 11 (active LOW: 1 = off)
static const uint8_t EN1  = 1u << 4;   // AP_EN_PWM1 = 12
static const uint8_t EN0  = 1u << 5;   // AP_EN_PWM0 = 13
static const uint8_t WET  = 1u << 6;   // AP_EN_WET_MEAS = 14
static const uint8_t SENS = 1u << 7;   // AP_WET_MEAS = 15 (input)
static const uint8_t EN    = EN0 | EN1;
static const uint8_t OWNED = MUX0 | MUX1 | LED | WET;
static const uint8_t INPUTS = BTN | SENS;

// System.ino's writer, as a host model.
struct Writer {
  uint8_t shadow = LED;      // boot intent: LED off, wet off, mux 0/0
  bool    armed  = false;
  bool write(bool read_ok, uint8_t cur, uint8_t set_mask, uint8_t set_bits, uint8_t *out) {
    shadow = port1ShadowApply(shadow, set_mask, set_bits, OWNED);
    const uint8_t owned = (uint8_t)(OWNED | (armed ? EN : 0));
    const uint8_t force = armed ? EN : (uint8_t)0;
    return port1WritePlan(read_ok, cur, shadow, owned, force, out);
  }
  bool setPin(bool read_ok, uint8_t cur, uint8_t bit, bool level, uint8_t *out) {
    if ((bit & OWNED) == 0) return false;
    return write(read_ok, cur, bit, level ? bit : 0, out);
  }
  bool enableMotors(bool read_ok, uint8_t cur, uint8_t *out) { armed = true; return write(read_ok, cur, 0, 0, out); }
};

enum Kind { K_MUX0, K_MUX1, K_LED_ON, K_LED_OFF, K_WET_ON, K_WET_OFF, K_BOOT_ENABLES, K_P6_REASSERT, K_COUNT };

static bool doKind(Writer &w, Kind k, bool read_ok, uint8_t cur, uint8_t *out) {
  switch (k) {
    case K_MUX0:         return w.write(read_ok, cur, MUX0 | MUX1, 0, out);
    case K_MUX1:         return w.write(read_ok, cur, MUX0 | MUX1, MUX0, out);
    case K_LED_ON:       return w.setPin(read_ok, cur, LED, false, out);   // active LOW
    case K_LED_OFF:      return w.setPin(read_ok, cur, LED, true, out);
    case K_WET_ON:       return w.setPin(read_ok, cur, WET, true, out);
    case K_WET_OFF:      return w.setPin(read_ok, cur, WET, false, out);
    case K_BOOT_ENABLES: return w.enableMotors(read_ok, cur, out);
    case K_P6_REASSERT:  return w.write(read_ok, cur, 0, 0, out);
    default:             return false;
  }
}

// ---- 1. every writer kind x every read value x armed / not ----
static void testEveryKindEveryRead() {
  for (int k = 0; k < K_COUNT; k++) {
    for (int armed = 0; armed < 2; armed++) {
      for (int cur = 0; cur < 256; cur++) {
        // Start each case from a known, non-trivial intent: channel 1, LED on, wet off.
        Writer w; w.armed = (armed != 0);
        uint8_t dummy;
        w.write(true, 0, MUX0 | MUX1, MUX0, &dummy);
        w.write(true, 0, LED, 0, &dummy);
        const uint8_t before = w.shadow;

        uint8_t out = 0xA5;
        const bool wrote = doKind(w, (Kind)k, true, (uint8_t)cur, &out);
        CHECK(wrote);
        const bool now_armed = w.armed;   // K_BOOT_ENABLES arms
        // Enables: forced HIGH once armed, carried exactly as read before.
        if (now_armed) CHECK((out & EN) == EN);
        else           CHECK((out & EN) == ((uint8_t)cur & EN));
        // Inputs: carried over as read.
        CHECK((out & INPUTS) == ((uint8_t)cur & INPUTS));
        // Owned bits: exactly the shadow, whatever was read.
        CHECK((out & OWNED) == (w.shadow & OWNED));
        // And the shadow is the intended result of this request on top of the earlier intent.
        uint8_t want = before;
        switch ((Kind)k) {
          case K_MUX0:    want = (uint8_t)(want & ~(MUX0 | MUX1)); break;
          case K_MUX1:    want = (uint8_t)((want & ~(MUX0 | MUX1)) | MUX0); break;
          case K_LED_ON:  want = (uint8_t)(want & ~LED); break;
          case K_LED_OFF: want = (uint8_t)(want | LED); break;
          case K_WET_ON:  want = (uint8_t)(want | WET); break;
          case K_WET_OFF: want = (uint8_t)(want & ~WET); break;
          default: break;
        }
        CHECK(w.shadow == want);
        // The wet probe is ON only after an explicit wet-on request.
        CHECK(((out & WET) != 0) == ((Kind)k == K_WET_ON));

        // A failed read writes NOTHING, whatever byte the bus left behind - and keeps the request in the shadow.
        Writer w2; w2.armed = (armed != 0);
        uint8_t untouched = 0x5A;
        CHECK(!doKind(w2, (Kind)k, false, (uint8_t)cur, &untouched));
        CHECK(untouched == 0x5A);
      }
    }
  }
}

// ---- 2. sequences ----
static void testSequences() {
  // FS-1: an acknowledged read corrupted to all zeros no longer clears the enables, moves the mux or lights the LED.
  {
    Writer w; uint8_t out;
    w.enableMotors(true, 0x38, &out);
    w.write(true, 0x38, MUX0 | MUX1, MUX0, &out);            // channel 1, LED off
    CHECK(out == (uint8_t)(MUX0 | LED | EN));
    CHECK(w.setPin(true, 0x00, LED, true, &out));            // LED write, read corrupted to 0x00
    CHECK(out == (uint8_t)(MUX0 | LED | EN));                // enables HIGH, channel 1, LED off, wet off
    CHECK(w.write(true, 0xFF, 0, 0, &out));                  // read corrupted to 0xFF (both inputs read high)
    CHECK((out & ~INPUTS) == (uint8_t)(MUX0 | LED | EN));    // wet probe NOT powered, mux NOT on channel 3
  }
  // FS-14: the wet-measure enable is HIGH only inside checkWetness()'s window, even if other writers run in it.
  {
    Writer w; uint8_t out;
    w.enableMotors(true, 0, &out);
    w.setPin(true, 0, WET, true, &out);          CHECK(out & WET);
    w.write(true, 0, MUX0 | MUX1, MUX0, &out);   CHECK(out & WET);       // a mux write inside the window keeps it
    w.setPin(true, 0, WET, false, &out);         CHECK(!(out & WET));
    w.setPin(true, 0, LED, false, &out);         CHECK(!(out & WET));    // and it stays off afterwards
  }
  // A skipped write (failed read) is repaired by the next successful write from ANY caller.
  {
    Writer w; uint8_t out = 0;
    w.enableMotors(true, 0, &out);
    CHECK(!w.setPin(false, 0, WET, false, &out));            // the "wet off" write is skipped...
    CHECK(w.write(true, WET, MUX0 | MUX1, MUX0, &out));      // ...the chip still has the probe on...
    CHECK(!(out & WET) && (out & EN) == EN);                 // ...and the next mux write turns it off
  }
  // Before the boot enable point the enables are left exactly as read (no earlier opto turn-on than before).
  {
    Writer w; uint8_t out;
    CHECK(w.setPin(true, 0x00, LED, true, &out) && (out & EN) == 0);   // startupAW()'s LED write
    CHECK(w.enableMotors(true, 0x00, &out) && (out & EN) == EN);       // runBootSequence()
    CHECK(w.setPin(true, 0x00, LED, true, &out) && (out & EN) == EN);  // and from then on, always
  }
  // The enables and the input pins can never be requested as owned outputs.
  {
    Writer w; uint8_t out = 0x11;
    CHECK(!w.setPin(true, 0, EN0, false, &out) && out == 0x11);
    CHECK(!w.setPin(true, 0, BTN, true, &out) && out == 0x11);
    CHECK(port1ShadowApply(0, 0xFF, 0xFF, OWNED) == OWNED);
  }
}

// ---- 3. the read-back verdict ----
static void testVerdict() {
  for (int v = 0; v < 256; v++) {
    CHECK(!muxReadbackVerdict(false, (uint8_t)v, 0, 1, true, false));
    CHECK(!muxReadbackVerdict(false, (uint8_t)v, 0, 1, false, false));
    for (int w = 0; w < 4; w++) {
      const bool w0 = (w & 1) != 0, w1 = (w & 2) != 0;
      const bool expect = ((v & 1) != 0) == w0 && ((v & 2) != 0) == w1;
      CHECK(muxReadbackVerdict(true, (uint8_t)v, 0, 1, w0, w1) == expect);
    }
  }
}

int main() {
  testEveryKindEveryRead();
  testSequences();
  testVerdict();
  if (fails) { std::printf("aw_port1_test: %d of %d checks FAILED\n", fails, checks); return 1; }
  std::printf("aw_port1_test: all %d checks passed\n", checks);
  return 0;
}
