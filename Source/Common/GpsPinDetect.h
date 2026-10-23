// V2.5-Evo - 2026-10-23 - GPS-PIN-1: which pin does the remote's GPS transmit on? The pure decision behind the TX's
//   power-up pin check (TX GPS.ino gpsDetectDataPin()). Header-only, no Arduino dependencies, so
//   Tools/tests/gps_pin_detect_test.cpp exercises the exact code the remote runs. TX only today; the RX does not use it.
//
// THE BUG IT CLOSES. The TX firmware always read its GPS on G18 and transmitted to it on G19 (BREmote_V2_Tx.h P_U1_RX 18,
// P_U1_TX 19). Factory-made remotes (Ludwig / OpenFoil) are wired the other way round: the GPS transmits into G19, which
// is what Ludwig's own code expects. On those remotes our firmware heard NOTHING ("Serial1 available 0", "Chars processed
// 0", ?gpsbaud finds no baud) while Ludwig's GPS test firmware got a fix on the same hardware. The remotes are potted, so
// the wires cannot be moved: the firmware has to find the GPS where it is.
//
// THE FIX. Before the GPS UART is opened, the TX watches BOTH pins as plain inputs (nothing driven) for about a second
// and counts signal edges on each. A GPS transmits continuously from power-on, NMEA or binary UBX, at any baud, so its
// transmit line toggles hundreds of times per second; the other line (the GPS's receive input) stays still. This file
// turns the two edge counts into a verdict. It does not care about the protocol or the baud, on purpose: the module may
// be NMEA or UBX-only, at 9600 or 115200.
//
//   edges only on the default receive pin (G18) -> DEFAULT  : RX=G18, TX=G19 (the documented DIY wiring)
//   edges only on the default transmit pin (G19) -> SWAPPED : RX=G19, TX=G18 (factory wiring - OpenFoil)
//   edges on neither                             -> NONE    : keep the default (no GPS, no power, or a dead module)
//   edges on both                                -> BOTH    : cannot tell, keep the default (today's behaviour)
//
// "Edges on a pin" means at least min_edges of them. Below that it is noise, not data. The smallest real stream is far
// above it: every UART byte has at least two edges (the start bit falls, the stop bit rises), and even one short NMEA
// sentence or one UBX message is 40-100+ bytes.
// ============================================================
#ifndef BREMOTE_GPS_PIN_DETECT_H
#define BREMOTE_GPS_PIN_DETECT_H

#include <stdint.h>

// Verdicts. Plain uint8_t constants rather than an enum, the same choice GPS.ino makes for its UBX constants: the
// Arduino builder hoists auto-generated prototypes above types declared in a .ino, so plain integers are the safe form.
static const uint8_t GPS_PIN_VERDICT_NONE    = 0;  // no data on either pin -> default pins
static const uint8_t GPS_PIN_VERDICT_DEFAULT = 1;  // data only on the default receive pin -> default pins
static const uint8_t GPS_PIN_VERDICT_SWAPPED = 2;  // data only on the default transmit pin -> pins swapped
static const uint8_t GPS_PIN_VERDICT_BOTH    = 3;  // data on both -> ambiguous -> default pins

// Noise floor, in edges per check window (about 1.1 s). Range: any value >= 1; 32 is used. A real GPS stream gives
// hundreds to tens of thousands of edges in that window; an idle, pulled-up line gives 0.
static const uint32_t kGpsPinMinEdges = 32u;

// ------------------------------------------------------------
// gpsPinVerdict - turn the two edge counts into a verdict.
// Inputs:  edges_on_default_rx - edges counted on the pin the firmware reads by default (G18 on the TX)
//          edges_on_default_tx - edges counted on the pin the firmware drives by default (G19 on the TX)
//          min_edges           - noise floor; a pin "has data" at or above it. 0 is treated as 1, so a pin with no
//                                edges at all can never count as having data.
// Output:  GPS_PIN_VERDICT_NONE / _DEFAULT / _SWAPPED / _BOTH.
// No side effects.
// ------------------------------------------------------------
static inline uint8_t gpsPinVerdict(uint32_t edges_on_default_rx, uint32_t edges_on_default_tx, uint32_t min_edges)
{
  if (min_edges == 0u) min_edges = 1u;
  const bool data_on_rx = edges_on_default_rx >= min_edges;
  const bool data_on_tx = edges_on_default_tx >= min_edges;
  if (data_on_rx && data_on_tx) return GPS_PIN_VERDICT_BOTH;
  if (data_on_rx)               return GPS_PIN_VERDICT_DEFAULT;
  if (data_on_tx)               return GPS_PIN_VERDICT_SWAPPED;
  return GPS_PIN_VERDICT_NONE;
}

// ------------------------------------------------------------
// gpsPinRxFor / gpsPinTxFor - the pin the UART should RECEIVE on / TRANSMIT on for a verdict.
// Only SWAPPED changes anything; every other verdict, including an unknown value, keeps the defaults, so a corrupt
// verdict can never move the UART somewhere it was not proven to belong.
// Inputs: verdict, and the two default pin numbers. Output: a pin number. No side effects.
// ------------------------------------------------------------
static inline uint8_t gpsPinRxFor(uint8_t verdict, uint8_t default_rx, uint8_t default_tx)
{
  return (verdict == GPS_PIN_VERDICT_SWAPPED) ? default_tx : default_rx;
}

static inline uint8_t gpsPinTxFor(uint8_t verdict, uint8_t default_rx, uint8_t default_tx)
{
  return (verdict == GPS_PIN_VERDICT_SWAPPED) ? default_rx : default_tx;
}

// ------------------------------------------------------------
// gpsPinVerdictLabel - short plain-English name for the boot line and ?printgps. No side effects.
// ------------------------------------------------------------
static inline const char* gpsPinVerdictLabel(uint8_t verdict)
{
  switch (verdict) {
    case GPS_PIN_VERDICT_DEFAULT: return "DIY wiring";
    case GPS_PIN_VERDICT_SWAPPED: return "factory wiring - OpenFoil";
    case GPS_PIN_VERDICT_BOTH:    return "signal on both pins - cannot tell, default used";
    default:                      return "no signal on either pin - default used";
  }
}

#endif
