#ifndef BREMOTE_FOLLOW_ME_DISTANCE_WARNING_H
#define BREMOTE_FOLLOW_ME_DISTANCE_WARNING_H

#include <stdint.h>

// The existing one-byte RX->TX distance telemetry represents 0.0-9.9 m in 0.1 m steps and
// 10-164 m in whole metres. 0xFF is reserved for no data, so 164 m is the largest configurable
// warning threshold that can be evaluated truthfully without changing the packet ABI.
static const uint16_t kFmDistanceTelemetryMaxM = 164u;

static inline float followMeDecodeDistanceM(uint8_t encoded_distance)
{
  if (encoded_distance == 0xFFu) return -1.0f;
  if (encoded_distance < 100u) return (float)encoded_distance / 10.0f;
  return (float)(encoded_distance - 90u);
}

// The warning belongs to the live FM declaration, not to trigger posture: releasing the trigger
// stops motor authority but must not hide that the buggy has reached the configured separation.
// Requiring both TX- and RX-armed state plus a fresh link prevents stale/default telemetry from
// producing a warning before the two sides agree that FM is live.
static inline bool followMeDistanceWarningActive(
    bool tx_fm_armed,
    bool rx_fm_armed,
    bool link_fresh,
    uint8_t encoded_distance,
    uint16_t warning_distance_m)
{
  if (!tx_fm_armed || !rx_fm_armed || !link_fresh) return false;
  if (warning_distance_m == 0u || warning_distance_m > kFmDistanceTelemetryMaxM) return false;

  const float distance_m = followMeDecodeDistanceM(encoded_distance);
  return distance_m >= 0.0f && distance_m >= (float)warning_distance_m;
}

static inline bool followMeWarningPulseDue(
    bool warning_active,
    bool pulse_already_sent,
    uint32_t last_pulse_ms,
    uint32_t now_ms,
    uint32_t period_ms)
{
  return warning_active &&
      (!pulse_already_sent || (uint32_t)(now_ms - last_pulse_ms) >= period_ms);
}

#endif
