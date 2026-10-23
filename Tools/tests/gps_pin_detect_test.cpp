// V2.5-Evo - 2026-10-23 - Host test for Common/GpsPinDetect.h (GPS-PIN-1): the G18-or-G19 verdict the TX uses to find its
//   GPS, and the pin mapping that follows from it. Signal only on G18, only on G19, on neither, on both, and the noise floor.
// Build and run:  g++ -std=c++17 -Wall -Wextra -o gps_pin_detect_test gps_pin_detect_test.cpp && ./gps_pin_detect_test

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../../Source/Common/GpsPinDetect.h"

// The TX defaults (BREmote_V2_Tx.h): the firmware receives on G18 and transmits on G19 unless the check says otherwise.
static const uint8_t kRx = 18, kTx = 19;

static void testSinglePin()
{
  // Signal only on G18: the DIY wiring, nothing moves.
  assert(gpsPinVerdict(5000u, 0u, kGpsPinMinEdges) == GPS_PIN_VERDICT_DEFAULT);
  assert(gpsPinRxFor(GPS_PIN_VERDICT_DEFAULT, kRx, kTx) == 18);
  assert(gpsPinTxFor(GPS_PIN_VERDICT_DEFAULT, kRx, kTx) == 19);

  // Signal only on G19: the factory wiring, the UART swaps.
  assert(gpsPinVerdict(0u, 5000u, kGpsPinMinEdges) == GPS_PIN_VERDICT_SWAPPED);
  assert(gpsPinRxFor(GPS_PIN_VERDICT_SWAPPED, kRx, kTx) == 19);
  assert(gpsPinTxFor(GPS_PIN_VERDICT_SWAPPED, kRx, kTx) == 18);

  // The smallest real stream (one short UBX message: ~100 bytes, >= 2 edges each) is far above the floor.
  assert(gpsPinVerdict(0u, 200u, kGpsPinMinEdges) == GPS_PIN_VERDICT_SWAPPED);
  // The edge cap the firmware detaches at (512) is a clear verdict too.
  assert(gpsPinVerdict(512u, 0u, kGpsPinMinEdges) == GPS_PIN_VERDICT_DEFAULT);
}

static void testNeitherAndBoth()
{
  // No signal anywhere (no module, no power): default pins.
  assert(gpsPinVerdict(0u, 0u, kGpsPinMinEdges) == GPS_PIN_VERDICT_NONE);
  assert(gpsPinRxFor(GPS_PIN_VERDICT_NONE, kRx, kTx) == 18);
  assert(gpsPinTxFor(GPS_PIN_VERDICT_NONE, kRx, kTx) == 19);

  // Signal on both: cannot tell, default pins (what every remote did before the check existed).
  assert(gpsPinVerdict(800u, 900u, kGpsPinMinEdges) == GPS_PIN_VERDICT_BOTH);
  assert(gpsPinVerdict(kGpsPinMinEdges, kGpsPinMinEdges, kGpsPinMinEdges) == GPS_PIN_VERDICT_BOTH);
  assert(gpsPinRxFor(GPS_PIN_VERDICT_BOTH, kRx, kTx) == 18);
  assert(gpsPinTxFor(GPS_PIN_VERDICT_BOTH, kRx, kTx) == 19);
}

static void testNoiseFloor()
{
  const uint32_t m = kGpsPinMinEdges;
  assert(m >= 2u);   // a single stray edge must never count as data

  // Just below the floor is noise, on either pin.
  assert(gpsPinVerdict(m - 1u, 0u, m) == GPS_PIN_VERDICT_NONE);
  assert(gpsPinVerdict(0u, m - 1u, m) == GPS_PIN_VERDICT_NONE);
  assert(gpsPinVerdict(m - 1u, m - 1u, m) == GPS_PIN_VERDICT_NONE);
  // At the floor it is data.
  assert(gpsPinVerdict(m, 0u, m) == GPS_PIN_VERDICT_DEFAULT);
  assert(gpsPinVerdict(0u, m, m) == GPS_PIN_VERDICT_SWAPPED);

  // A little noise on the quiet pin does not spoil a clear verdict on the busy one.
  assert(gpsPinVerdict(3000u, m - 1u, m) == GPS_PIN_VERDICT_DEFAULT);
  assert(gpsPinVerdict(m - 1u, 3000u, m) == GPS_PIN_VERDICT_SWAPPED);

  // A floor of 0 is treated as 1: zero edges is never data, one edge is.
  assert(gpsPinVerdict(0u, 0u, 0u) == GPS_PIN_VERDICT_NONE);
  assert(gpsPinVerdict(1u, 0u, 0u) == GPS_PIN_VERDICT_DEFAULT);
  assert(gpsPinVerdict(0u, 1u, 0u) == GPS_PIN_VERDICT_SWAPPED);

  // Counter extremes do not wrap into a wrong verdict.
  assert(gpsPinVerdict(0xFFFFFFFFu, 0u, m) == GPS_PIN_VERDICT_DEFAULT);
  assert(gpsPinVerdict(0u, 0xFFFFFFFFu, m) == GPS_PIN_VERDICT_SWAPPED);
}

static void testMappingIsAlwaysAPair()
{
  // Whatever the verdict - including a corrupt one - RX and TX are the two default pins, one each, never the same pin.
  // Only SWAPPED may move them.
  for (int v = 0; v < 256; ++v) {
    const uint8_t rx = gpsPinRxFor((uint8_t)v, kRx, kTx);
    const uint8_t tx = gpsPinTxFor((uint8_t)v, kRx, kTx);
    assert(rx != tx);
    assert((rx == 18 && tx == 19) || (rx == 19 && tx == 18));
    if (v != GPS_PIN_VERDICT_SWAPPED) assert(rx == 18 && tx == 19);
  }
}

static void testLabels()
{
  assert(strcmp(gpsPinVerdictLabel(GPS_PIN_VERDICT_DEFAULT), "DIY wiring") == 0);
  assert(strcmp(gpsPinVerdictLabel(GPS_PIN_VERDICT_SWAPPED), "factory wiring") == 0);
  assert(gpsPinVerdictLabel(GPS_PIN_VERDICT_NONE) != NULL);
  assert(gpsPinVerdictLabel(GPS_PIN_VERDICT_BOTH) != NULL);
  assert(gpsPinVerdictLabel(200) != NULL);   // unknown value still prints something
}

int main()
{
  testSinglePin();
  testNeitherAndBoth();
  testNoiseFloor();
  testMappingIsAlwaysAPair();
  testLabels();
  printf("gps_pin_detect_test: all checks passed\n");
  return 0;
}
