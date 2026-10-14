// V2.5-Evo - 2026-10-14 - link-layer audit F-1 (ported to the RX1 build with FS-1, owner-approved): the UART-mux
//   read-back verdict as a pure function, host-tested in Tools/tests/aw_port1_test.cpp. setUartMux() (RX System.ino)
//   feeds it ONE checked read of the AW9523 INPUT register that holds both mux select pins. Any I2C failure is "not
//   verified": the library's digitalRead() cannot report a failed read and returns HIGH, which is exactly channel 1's
//   wanted MUX_0 level, so a write that did not stick plus a failed read used to pass as "on channel 1".
//   No Arduino dependency. The port-1 OUTPUT write plan (FS-1 / FS-13 / FS-14) is in ./AwPort1.h.
#pragma once
#include <stdint.h>

// muxReadbackVerdict - does one checked read of the mux port confirm the wanted channel?
// Inputs:  read_ok      - the I2C read was acknowledged AND returned its byte (false on ANY I2C error);
//          port         - the byte read (ignored when read_ok is false);
//          bit0, bit1   - the bit positions of the MUX_0 / MUX_1 select pins inside that port (0-7);
//          want0, want1 - the wanted pad levels for the channel being selected.
// Returns: true only when the read succeeded AND both select bits match. Side effects: none.
static inline bool muxReadbackVerdict(bool read_ok, uint8_t port, uint8_t bit0, uint8_t bit1, bool want0, bool want1)
{
  if (!read_ok) return false;                         // an I2C error is never read as a level
  const bool m0 = ((port >> bit0) & 1u) != 0;
  const bool m1 = ((port >> bit1) & 1u) != 0;
  return (m0 == want0) && (m1 == want1);
}
