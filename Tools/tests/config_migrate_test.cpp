// Host test for Common/ConfigMigrate.h - the SW35 -> SW36 prefix migration, on a REAL image.
//
// The image below is the owner's RX1 config as exported by ?conf on 2026-09-18 (SW35, 192 bytes,
// from Logs testing/conf-backups/rx1-conf-backup-2026-09-18-postalign.txt). The test decodes it,
// migrates it exactly the way the RX does at boot (cfgPrefixMigrateImage with the factory-default
// image as the tail source, then the version stamp), and asserts:
//   - every SW35 byte survives at its offset (the whole 192-byte prefix is identical),
//   - the named fields the owner would notice losing carry their known values (pairing, addresses,
//     compass calibration and orientation, throttle calibration, RTM stop distance, tuning),
//   - the three SW36 fields equal their factory defaults (1 / 13 / 80) and the tail pad is 0,
//   - nothing from the default image leaks into the prefix,
//   - and the gate refuses every other (size, version) pair.
// The two mirror structs pin the layouts (sizeof 192 and 200, checked at compile time); if the
// firmware struct ever changes shape this test must change with it.

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#include "../../Source/Common/ConfigMigrate.h"

// ---- Layout mirrors of Source/V2_Integration_Rx/BREmote_V2_Rx.h confStruct ----
// Field order and types copied one-to-one; uint16/int16/float/uint8/char have the same alignment
// on this host as on the RV32 target, so the offsets and the padding agree.
struct ConfSw35 {
  uint16_t version;
  uint16_t radio_preset;
  int16_t  rf_power;
  uint16_t steering_type;
  uint16_t steering_influence;
  uint16_t steering_inverted;
  int16_t  trim;
  uint16_t PWM0_min, PWM0_max, PWM1_min, PWM1_max;
  uint16_t failsafe_time;
  uint16_t foil_num_cells;
  uint16_t bms_det_active, wet_det_active;
  uint16_t rtm_steer_response;
  uint16_t data_src;
  uint16_t gps_en, followme_mode, kalman_en;
  float    boogie_vmax_in_followme_kmh, min_dist_m, followme_smoothing_band_m, foiler_low_speed_kmh;
  float    zone_angle_enter_deg, zone_angle_exit_deg, near_diag_offset_deg;
  float    ubat_cal, ubat_offset;
  uint16_t tx_gps_stale_timeout_ms;
  uint16_t logger_en;
  uint16_t paired;
  uint8_t  own_address[3];
  uint8_t  dest_address[3];
  char     wifi_password[8];
  int16_t  mag_offset_x, mag_offset_y;
  float    mag_scale_x, mag_scale_y;
  uint16_t gps_chip_type;
  float    gps_max_hdop, gps_max_accel_g, gps_max_teleport_kmh;
  uint16_t gps_suspect_threshold;
  float    gps_max_pair_dist_m, gps_max_speed_diff_kmh;
  float    rtm_vesc_speed_diff_kmh, vesc_erpm_per_kmh;
  uint16_t rtm_rx_enabled, rtm_rx_override_steering, rtm_compass_required, rtm_stop_distance_m;
  uint16_t vesc_timeout_s;
  uint16_t gps_update_hz;
  uint16_t rtm_approach_zone_m;
  uint16_t rtm_use_compass, rtm_cog_min_speed_kmh;
  float    rtm_target_speed_kmh;
  uint16_t rtm_align_threshold_deg;
  float    motor_ramp_s;
  float    fm_engage_dist_m;
  uint16_t gps_dyn_model;
  uint16_t log_level;
  uint16_t mag_orientation;
  uint16_t rsvd_u16_1;
  float    rsvd_f32_1;
};
static_assert(sizeof(ConfSw35) == 192, "SW35 mirror must be 192 bytes");
static_assert(offsetof(ConfSw35, rsvd_f32_1) == 188, "SW35 tail offset");

struct ConfSw36 {
  ConfSw35 sw35;                 // byte-exact prefix
  uint16_t fm_return_mode;       // 192
  uint16_t fm_align_cap;         // 194
  uint16_t fm_align_influence;   // 196
                                 // 198-199: alignment pad
};
static_assert(sizeof(ConfSw36) == 200, "SW36 mirror must be 200 bytes (198 + 2 pad)");
static_assert(offsetof(ConfSw36, fm_return_mode) == 192, "SW36 first appended field at 192");
static_assert(offsetof(ConfSw36, fm_align_influence) == 196, "SW36 last appended field at 196");

static const size_t   kLegacyLen = sizeof(ConfSw35);   // 192
static const uint16_t kLegacySw  = 35;
static const size_t   kTargetLen = sizeof(ConfSw36);   // 200
static const uint16_t kTargetSw  = 36;

// ---- The owner's real SW35 image (?conf, 2026-09-18) ----
static const char kBlobB64[] =
  "IwACABYAAQA3AAEAAADoA9AH6APQB+gDCgAAAAEAAgACAAEAAgABADMz20EAAKBAAACAQAAAAEEAAAxCAAA0QgAAcEJBjhw8AAAAALgLAAABAEbJ6UbLzDEyMzQ1Njc4FwRC/w4pTz9yfKe/AQAAAAAAAEAAAEBAAACgQgMAAAAAAPpDAABIQgAAoEEAAAAAAQABAAEAAwAGAAoADAABAAMAAAAAAIBALQAAAAAAAEAAAEBBAAAEALQAAAAAAAAA";

// Minimal Base64 decoder (RFC 4648, '=' padding). Returns decoded length, 0 on a bad character.
static size_t b64decode(const char* in, uint8_t* out, size_t out_cap)
{
  size_t n = 0; uint32_t acc = 0; int bits = 0;
  for (const char* p = in; *p; ++p) {
    char c = *p; int v;
    if (c >= 'A' && c <= 'Z') v = c - 'A';
    else if (c >= 'a' && c <= 'z') v = c - 'a' + 26;
    else if (c >= '0' && c <= '9') v = c - '0' + 52;
    else if (c == '+') v = 62;
    else if (c == '/') v = 63;
    else if (c == '=') break;
    else return 0;
    acc = (acc << 6) | (uint32_t)v; bits += 6;
    if (bits >= 8) { bits -= 8; if (n >= out_cap) return 0; out[n++] = (uint8_t)((acc >> bits) & 0xFF); }
  }
  return n;
}

static bool near(float a, float b, float eps) { return fabsf(a - b) <= eps; }

int main()
{
  // ---- Decode the real image and check it is exactly what the firmware would see ----
  uint8_t blob[256];
  const size_t blobLen = b64decode(kBlobB64, blob, sizeof(blob));
  assert(blobLen == kLegacyLen);
  const uint16_t blobVersion = (uint16_t)(blob[0] | ((uint16_t)blob[1] << 8));
  assert(blobVersion == kLegacySw);

  ConfSw35 old;
  memcpy(&old, blob, sizeof(old));
  // Spot-check the decode against the ?conf listing in the backup file.
  assert(old.version == 35 && old.radio_preset == 2 && old.rf_power == 22);
  assert(old.steering_type == 1 && old.steering_influence == 55 && old.steering_inverted == 1);
  assert(old.paired == 1);
  assert(old.rtm_stop_distance_m == 3);
  assert(old.mag_orientation == 180);

  // ---- The factory-default image, poisoned everywhere it must NOT be read from ----
  // Every byte 0xA5 except the SW36 tail: the three new fields at their defaults and a zero pad.
  // If a single 0xA5 shows up in the migrated prefix, the migration read the wrong source.
  ConfSw36 defaults;
  memset(&defaults, 0xA5, sizeof(defaults));
  defaults.sw35.version       = kTargetSw;
  defaults.fm_return_mode     = 1;
  defaults.fm_align_cap       = 13;
  defaults.fm_align_influence = 80;
  memset((uint8_t*)&defaults + 198, 0, 2);   // the tail pad, zero as in a zero-initialised global

  // ---- The gate: exactly this pair and nothing else ----
  assert(cfgLegacyPairMatches(blobLen, blobVersion, kLegacyLen, kLegacySw, true));
  assert(!cfgLegacyPairMatches(blobLen, blobVersion, kLegacyLen, kLegacySw, false));   // build not the target
  assert(!cfgLegacyPairMatches(184, 34, kLegacyLen, kLegacySw, true));                 // the SW34 blob: one version too old
  assert(!cfgLegacyPairMatches(192, 34, kLegacyLen, kLegacySw, true));                 // legacy length, wrong version
  assert(!cfgLegacyPairMatches(200, 36, kLegacyLen, kLegacySw, true));                 // already current
  assert(!cfgLegacyPairMatches(191, 35, kLegacyLen, kLegacySw, true));                 // one byte short

  // ---- Migrate exactly as the RX does, then stamp the version as the firmware does ----
  ConfSw36 migrated;
  memset(&migrated, 0x5A, sizeof(migrated));
  assert(cfgPrefixMigrateImage(blob, blobLen, kLegacyLen,
                               (const uint8_t*)&defaults, kTargetLen, (uint8_t*)&migrated));
  // The whole prefix is the blob, byte for byte - before the stamp the version is still 35.
  assert(memcmp(&migrated, blob, kLegacyLen) == 0);
  migrated.sw35.version = kTargetSw;
  // After the stamp: everything except the two version bytes is still the blob.
  assert(memcmp((const uint8_t*)&migrated + 2, blob + 2, kLegacyLen - 2) == 0);
  assert(migrated.sw35.version == 36);

  // ---- The three new fields equal their defaults; the pad is 0; no 0xA5 anywhere in the prefix ----
  assert(migrated.fm_return_mode == 1);
  assert(migrated.fm_align_cap == 13);
  assert(migrated.fm_align_influence == 80);
  const uint8_t* mb = (const uint8_t*)&migrated;
  assert(mb[198] == 0 && mb[199] == 0);
  for (size_t i = 2; i < kLegacyLen; ++i) assert(mb[i] == blob[i]);

  // ---- Every field the owner would notice losing, by name and value (from the ?conf listing) ----
  const ConfSw35& m = migrated.sw35;
  assert(m.paired == 1);
  assert(m.own_address[0] == 0x46 && m.own_address[1] == 0xC9 && m.own_address[2] == 0xE9);
  assert(m.dest_address[0] == 0x46 && m.dest_address[1] == 0xCB && m.dest_address[2] == 0xCC);
  assert(memcmp(m.wifi_password, "12345678", 8) == 0);
  assert(m.steering_type == 1 && m.steering_influence == 55 && m.steering_inverted == 1 && m.trim == 0);
  assert(m.PWM0_min == 1000 && m.PWM0_max == 2000 && m.PWM1_min == 1000 && m.PWM1_max == 2000);
  assert(m.failsafe_time == 1000 && m.foil_num_cells == 10);
  assert(m.bms_det_active == 0 && m.wet_det_active == 1);
  assert(m.rtm_steer_response == 2 && m.data_src == 2 && m.gps_en == 1 && m.followme_mode == 2 && m.kalman_en == 1);
  assert(near(m.boogie_vmax_in_followme_kmh, 27.4f, 0.01f));
  assert(near(m.min_dist_m, 5.0f, 0.001f) && near(m.followme_smoothing_band_m, 4.0f, 0.001f));
  assert(near(m.foiler_low_speed_kmh, 8.0f, 0.001f));
  assert(near(m.zone_angle_enter_deg, 35.0f, 0.001f) && near(m.zone_angle_exit_deg, 45.0f, 0.001f));
  assert(near(m.near_diag_offset_deg, 60.0f, 0.001f));
  assert(near(m.ubat_cal, 0.0095554f, 1e-7f) && near(m.ubat_offset, 0.0f, 1e-6f));
  assert(m.tx_gps_stale_timeout_ms == 3000 && m.logger_en == 0);
  assert(m.mag_offset_x == 1047 && m.mag_offset_y == -190);
  assert(near(m.mag_scale_x, 0.81f, 0.005f) && near(m.mag_scale_y, -1.31f, 0.005f));   // the mirrored frame survives
  assert(m.gps_chip_type == 1 && m.gps_dyn_model == 0);
  assert(near(m.gps_max_hdop, 2.0f, 0.001f) && near(m.gps_max_accel_g, 3.0f, 0.001f));
  assert(near(m.gps_max_teleport_kmh, 80.0f, 0.001f) && m.gps_suspect_threshold == 3);
  assert(near(m.gps_max_pair_dist_m, 500.0f, 0.001f) && near(m.gps_max_speed_diff_kmh, 50.0f, 0.001f));
  assert(near(m.rtm_vesc_speed_diff_kmh, 20.0f, 0.001f) && near(m.vesc_erpm_per_kmh, 0.0f, 1e-6f));
  assert(m.rtm_rx_enabled == 1 && m.rtm_rx_override_steering == 1 && m.rtm_compass_required == 1);
  assert(m.rtm_stop_distance_m == 3);      // the owner's 3 m RTM stop - now also the FM_RETURN stop radius
  assert(m.vesc_timeout_s == 6);           // the stored value, not the newer 10 s default: nothing is "corrected"
  assert(m.gps_update_hz == 10 && m.rtm_approach_zone_m == 12);
  assert(m.rtm_use_compass == 1 && m.rtm_cog_min_speed_kmh == 3);
  assert(near(m.rtm_target_speed_kmh, 4.0f, 0.001f) && m.rtm_align_threshold_deg == 45);
  assert(near(m.motor_ramp_s, 2.0f, 0.001f));
  assert(near(m.fm_engage_dist_m, 12.0f, 0.001f));
  assert(m.log_level == 4);
  assert(m.mag_orientation == 180);
  assert(m.rsvd_u16_1 == 0 && near(m.rsvd_f32_1, 0.0f, 1e-6f));

  // ---- Refusals: the image is never reinterpreted when the sizes do not say it is a prefix ----
  ConfSw36 scratch;
  memset(&scratch, 0x5A, sizeof(scratch));
  assert(!cfgPrefixMigrateImage(blob, 184, kLegacyLen, (const uint8_t*)&defaults, kTargetLen, (uint8_t*)&scratch));  // wrong blob length
  assert(!cfgPrefixMigrateImage(blob, blobLen, 0, (const uint8_t*)&defaults, kTargetLen, (uint8_t*)&scratch));      // empty legacy size
  assert(!cfgPrefixMigrateImage(blob, blobLen, kLegacyLen, (const uint8_t*)&defaults, 100, (uint8_t*)&scratch));    // target smaller than the prefix
  assert(!cfgPrefixMigrateImage(NULL, blobLen, kLegacyLen, (const uint8_t*)&defaults, kTargetLen, (uint8_t*)&scratch));
  assert(!cfgPrefixMigrateImage(blob, blobLen, kLegacyLen, NULL, kTargetLen, (uint8_t*)&scratch));
  assert(!cfgPrefixMigrateImage(blob, blobLen, kLegacyLen, (const uint8_t*)&defaults, kTargetLen, NULL));
  const uint8_t* sb = (const uint8_t*)&scratch;
  for (size_t i = 0; i < kTargetLen; ++i) assert(sb[i] == 0x5A);   // a refused migration writes nothing

  // A same-size migration (legacy_len == target_len) is legal and is a plain copy.
  assert(cfgPrefixMigrateImage(blob, blobLen, kLegacyLen, (const uint8_t*)&defaults, kLegacyLen, (uint8_t*)&scratch));
  assert(memcmp(&scratch, blob, kLegacyLen) == 0);

  return 0;
}
