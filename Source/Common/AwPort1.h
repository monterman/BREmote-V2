// V2.5-Evo - 2026-10-14 - full-system audit FS-1 (HIGH) + FS-13 (MEDIUM) + FS-14 (LOW), owner-approved for the RX1
//   build: ONE writer for the AW9523's port-1 OUTPUT register (OUTPUT1, RX pins 8-15), as pure functions, host-tested in
//   Tools/tests/aw_port1_test.cpp over every possible read value for every kind of write the RX makes.
//
// WHAT WAS WRONG. Port 1 holds both PPM enables (pins 12 and 13), HIGH by design and never driven LOW by any code: the
// enable swap picks a motor through CONFIG1 (direction), not OUTPUT1. But every other port-1 output - the UART-mux
// selects (8, 9), the AUX LED (11), the wet-measure enable (14) - was written with the library's digitalWrite(), a
// read-modify-write of the WHOLE register. A read corrupted by motor noise but acknowledged by the chip was written
// straight back, enables included: an enable read as 0 was written as 0, and stayed 0 through every later
// read-modify-write. One motor (or both) then got no pulses until a reboot - one motor cut under a held trigger is a
// yaw. (A read that fails outright writes all ones instead; that cannot clear an enable, but it parks the mux on
// channel 3 and leaves the wet probe powered.)
//
// WHAT THE WRITER DOES NOW (System.ino awPort1WriteLocked()):
//   - a SHADOW byte holds the intended level of every port-1 OUTPUT pin the RX owns (mux 0/1, AUX LED, both enables,
//     wet-measure enable). A write request first updates the shadow (port1ShadowApply());
//   - then ONE checked read of OUTPUT1: if it fails, NOTHING is written (port1WritePlan() returns false). The shadow
//     keeps the request, and the next successful port-1 write - from any caller - applies it;
//   - otherwise ONE write of port1WritePlan()'s byte: every owned bit from the shadow, both enables FORCED HIGH whatever
//     the shadow or the read say, and only the two INPUT pins' bits (AUX button 10, wet sense 15) carried over as read.
//   So a corrupted read can no longer clear an enable, move the mux, flip the LED or power the wet probe (FS-14): the
//   read only decides whether the bus is answering. The wet-measure enable is LOW in the shadow except between
//   checkWetness()'s own HIGH and LOW writes.
// Forcing the enables HIGH restores the designed state and can never ADD thrust: a motor is pulsed only by the PWM task,
// under the trigger; an enable passes only what the RMT sends, and a high-Z (parked) channel ignores its OUTPUT bit.
// No Arduino dependency.
#pragma once
#include <stdint.h>

// port1ShadowApply - fold a write request into the shadow of the intended OUTPUT1 levels.
// Inputs: shadow (current intended levels), set_mask (the bits this request owns), set_bits (their wanted levels),
// owned_mask (every port-1 OUTPUT bit the RX owns). Returns: the new shadow; bits outside owned_mask are always 0, so
// an INPUT pin can never be recorded as "owned". Side effects: none.
static inline uint8_t port1ShadowApply(uint8_t shadow, uint8_t set_mask, uint8_t set_bits, uint8_t owned_mask)
{
  return (uint8_t)(((shadow & (uint8_t)~set_mask) | (set_bits & set_mask)) & owned_mask);
}

// port1WritePlan - decide whether to write OUTPUT1 at all, and what.
// Inputs: read_ok (the OUTPUT1 read was acknowledged and returned its byte), current (that byte), shadow (intended
// levels of the owned bits), owned_mask, enables_mask (both PPM enable bits), out.
// Returns: false = write NOTHING (the read failed); true = write *out, which is the shadow for every owned bit, both
// enables forced HIGH, and the not-owned (INPUT) bits as read. Side effects: *out, only when it returns true.
static inline bool port1WritePlan(bool read_ok, uint8_t current, uint8_t shadow, uint8_t owned_mask,
                                  uint8_t enables_mask, uint8_t *out)
{
  if (!read_ok) return false;
  *out = (uint8_t)((current & (uint8_t)~owned_mask) | (shadow & owned_mask) | enables_mask);
  return true;
}
