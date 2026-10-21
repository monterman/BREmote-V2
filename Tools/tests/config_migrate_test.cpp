// V2.5-Evo - 2026-10-20 - SYNTHETIC IMAGE: the test no longer embeds a real board's config. The image is built below
//   from made-up values (made-up addresses, settings that match no board), and goes through the same Base64 round trip
//   a ?conf export takes. Same coverage as before: decode, the byte-exact prefix, every named field, the defaults tail,
//   the gate and the refusals. A real config export carries the pairing and chip-derived addresses, so it is never
//   pasted into tracked source.
// Host test for Common/ConfigMigrate.h - the SW35 -> SW36 prefix migration.
//
// The image below is a SYNTHETIC SW35 config (192 bytes), built field by field from made-up values over a byte
// pattern (so the struct padding is non-zero too), then Base64-encoded and decoded the way a ?conf export is read.
// The test migrates it exactly the way the RX does at boot (cfgPrefixMigrateImage with the factory-default image as
// the tail source, then the version stamp), and asserts:
//   - every SW35 byte survives at its offset (the whole 192-byte prefix is identical),
//   - the named fields the owner would notice losing carry their values (pairing, addresses, compass calibration and
//     orientation, throttle calibration, RTM stop distance, tuning),
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
  uint16_t rsvd_u16_1;           // keeps the SW35 name on purpose: this mirrors the SW35 image AS IT WAS; the live RX struct
                                 // renamed this slot in place to steer_during_auto on 2026-09-19 (same offset, same u16)
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

// ---- The SYNTHETIC SW35 image ----
// Made-up values only. The two addresses are the made-up IDs the public design document uses (5C:10:20 and
// 5C:4A:7B); no real board has them. Every other value is chosen to differ from the factory defaults, so a field that
// was silently reset would show.
static const uint8_t kSynOwn[3]  = { 0x5C, 0x10, 0x20 };
static const uint8_t kSynDest[3] = { 0x5C, 0x4A, 0x7B };

// fillSyntheticSw35 - build the synthetic image. Input: none. Output: *c, every byte set (the pattern first, so the
// padding bytes are non-zero and the byte-exact prefix check covers them too). Side effects: none.
static void fillSyntheticSw35(ConfSw35* c)
{
  uint8_t* p = (uint8_t*)c;
  for (size_t i = 0; i < sizeof(*c); ++i) p[i] = (uint8_t)(0x11 + 7 * i);   // the pattern under every field and pad
  c->version = 35;          c->radio_preset = 1;        c->rf_power = 14;
  c->steering_type = 2;     c->steering_influence = 41; c->steering_inverted = 0; c->trim = -3;
  c->PWM0_min = 1010; c->PWM0_max = 1990; c->PWM1_min = 1020; c->PWM1_max = 1980;
  c->failsafe_time = 1200;  c->foil_num_cells = 12;
  c->bms_det_active = 1;    c->wet_det_active = 0;
  c->rtm_steer_response = 3; c->data_src = 1; c->gps_en = 1; c->followme_mode = 3; c->kalman_en = 0;
  c->boogie_vmax_in_followme_kmh = 21.5f; c->min_dist_m = 6.5f; c->followme_smoothing_band_m = 3.5f;
  c->foiler_low_speed_kmh = 7.25f;
  c->zone_angle_enter_deg = 30.0f; c->zone_angle_exit_deg = 40.0f; c->near_diag_offset_deg = 55.0f;
  c->ubat_cal = 0.0101f; c->ubat_offset = 0.125f;
  c->tx_gps_stale_timeout_ms = 2500; c->logger_en = 1;
  c->paired = 1;
  memcpy(c->own_address, kSynOwn, 3);
  memcpy(c->dest_address, kSynDest, 3);
  memcpy(c->wifi_password, "synthpw1", 8);
  c->mag_offset_x = 321; c->mag_offset_y = -123;
  c->mag_scale_x = 0.95f; c->mag_scale_y = -1.05f;                 // a mirrored frame, as a real calibration can be
  c->gps_chip_type = 2;
  c->gps_max_hdop = 1.5f; c->gps_max_accel_g = 2.5f; c->gps_max_teleport_kmh = 70.0f;
  c->gps_suspect_threshold = 4;
  c->gps_max_pair_dist_m = 400.0f; c->gps_max_speed_diff_kmh = 40.0f;
  c->rtm_vesc_speed_diff_kmh = 15.0f; c->vesc_erpm_per_kmh = 123.5f;
  c->rtm_rx_enabled = 1; c->rtm_rx_override_steering = 0; c->rtm_compass_required = 1; c->rtm_stop_distance_m = 9;
  c->vesc_timeout_s = 8;
  c->gps_update_hz = 5;
  c->rtm_approach_zone_m = 11;
  c->rtm_use_compass = 0; c->rtm_cog_min_speed_kmh = 4;
  c->rtm_target_speed_kmh = 4.5f;
  c->rtm_align_threshold_deg = 35;
  c->motor_ramp_s = 1.5f;
  c->fm_engage_dist_m = 10.5f;
  c->gps_dyn_model = 2;
  c->log_level = 3;
  c->mag_orientation = 90;
  c->rsvd_u16_1 = 0;
  c->rsvd_f32_1 = 0.0f;
}

// b64encode - RFC 4648 Base64 with '=' padding, the form ?conf prints. Inputs: in / len; out (cap bytes).
// Returns: the string length, 0 if out is too small. Side effects: writes out (NUL-terminated).
static size_t b64encode(const uint8_t* in, size_t len, char* out, size_t cap)
{
  static const char* k = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  const size_t need = 4 * ((len + 2) / 3) + 1;
  if (cap < need) return 0;
  size_t o = 0;
  for (size_t i = 0; i < len; i += 3) {
    const uint32_t a = in[i];
    const uint32_t b = (i + 1 < len) ? in[i + 1] : 0;
    const uint32_t c = (i + 2 < len) ? in[i + 2] : 0;
    const uint32_t v = (a << 16) | (b << 8) | c;
    out[o++] = k[(v >> 18) & 63];
    out[o++] = k[(v >> 12) & 63];
    out[o++] = (i + 1 < len) ? k[(v >> 6) & 63] : '=';
    out[o++] = (i + 2 < len) ? k[v & 63] : '=';
  }
  out[o] = 0;
  return o;
}

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
  // ---- Build the synthetic image, export it as ?conf would, and decode it back ----
  ConfSw35 synth;
  fillSyntheticSw35(&synth);
  char b64[512];
  assert(b64encode((const uint8_t*)&synth, sizeof(synth), b64, sizeof(b64)) == 256);   // 192 bytes -> 256 chars
  uint8_t blob[256];
  const size_t blobLen = b64decode(b64, blob, sizeof(blob));
  assert(memcmp(blob, &synth, sizeof(synth)) == 0);                                     // the round trip is exact
  assert(blobLen == kLegacyLen);
  const uint16_t blobVersion = (uint16_t)(blob[0] | ((uint16_t)blob[1] << 8));
  assert(blobVersion == kLegacySw);

  ConfSw35 old;
  memcpy(&old, blob, sizeof(old));
  // Spot-check the decode against the values the image was built from.
  assert(old.version == 35 && old.radio_preset == 1 && old.rf_power == 14);
  assert(old.steering_type == 2 && old.steering_influence == 41 && old.steering_inverted == 0);
  assert(old.paired == 1);
  assert(old.rtm_stop_distance_m == 9);
  assert(old.mag_orientation == 90);

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

  // ---- Every field the owner would notice losing, by name and value (the values the image was built from) ----
  const ConfSw35& m = migrated.sw35;
  assert(m.paired == 1);
  assert(memcmp(m.own_address, kSynOwn, 3) == 0);     // the identity fields survive the migration untouched
  assert(memcmp(m.dest_address, kSynDest, 3) == 0);
  assert(memcmp(m.wifi_password, "synthpw1", 8) == 0);
  assert(m.steering_type == 2 && m.steering_influence == 41 && m.steering_inverted == 0 && m.trim == -3);
  assert(m.PWM0_min == 1010 && m.PWM0_max == 1990 && m.PWM1_min == 1020 && m.PWM1_max == 1980);
  assert(m.failsafe_time == 1200 && m.foil_num_cells == 12);
  assert(m.bms_det_active == 1 && m.wet_det_active == 0);
  assert(m.rtm_steer_response == 3 && m.data_src == 1 && m.gps_en == 1 && m.followme_mode == 3 && m.kalman_en == 0);
  assert(near(m.boogie_vmax_in_followme_kmh, 21.5f, 0.001f));
  assert(near(m.min_dist_m, 6.5f, 0.001f) && near(m.followme_smoothing_band_m, 3.5f, 0.001f));
  assert(near(m.foiler_low_speed_kmh, 7.25f, 0.001f));
  assert(near(m.zone_angle_enter_deg, 30.0f, 0.001f) && near(m.zone_angle_exit_deg, 40.0f, 0.001f));
  assert(near(m.near_diag_offset_deg, 55.0f, 0.001f));
  assert(near(m.ubat_cal, 0.0101f, 1e-7f) && near(m.ubat_offset, 0.125f, 1e-6f));
  assert(m.tx_gps_stale_timeout_ms == 2500 && m.logger_en == 1);
  assert(m.mag_offset_x == 321 && m.mag_offset_y == -123);
  assert(near(m.mag_scale_x, 0.95f, 0.0005f) && near(m.mag_scale_y, -1.05f, 0.0005f));   // the mirrored frame survives
  assert(m.gps_chip_type == 2 && m.gps_dyn_model == 2);
  assert(near(m.gps_max_hdop, 1.5f, 0.001f) && near(m.gps_max_accel_g, 2.5f, 0.001f));
  assert(near(m.gps_max_teleport_kmh, 70.0f, 0.001f) && m.gps_suspect_threshold == 4);
  assert(near(m.gps_max_pair_dist_m, 400.0f, 0.001f) && near(m.gps_max_speed_diff_kmh, 40.0f, 0.001f));
  assert(near(m.rtm_vesc_speed_diff_kmh, 15.0f, 0.001f) && near(m.vesc_erpm_per_kmh, 123.5f, 1e-4f));
  assert(m.rtm_rx_enabled == 1 && m.rtm_rx_override_steering == 0 && m.rtm_compass_required == 1);
  assert(m.rtm_stop_distance_m == 9);      // the RTM stop distance - also the FM_RETURN stop radius
  assert(m.vesc_timeout_s == 8);           // the stored value, not the newer 10 s default: nothing is "corrected"
  assert(m.gps_update_hz == 5 && m.rtm_approach_zone_m == 11);
  assert(m.rtm_use_compass == 0 && m.rtm_cog_min_speed_kmh == 4);
  assert(near(m.rtm_target_speed_kmh, 4.5f, 0.001f) && m.rtm_align_threshold_deg == 35);
  assert(near(m.motor_ramp_s, 1.5f, 0.001f));
  assert(near(m.fm_engage_dist_m, 10.5f, 0.001f));
  assert(m.log_level == 3);
  assert(m.mag_orientation == 90);
  // The old-named slot reads 0 on the migrated image: a migrated board reads steer_during_auto 0 = cancel, the
  // behaviour-preserving default the rename-in-place rules require (2026-09-19).
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
