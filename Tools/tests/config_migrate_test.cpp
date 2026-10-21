// V2.5-Evo - 2026-10-20 - Y-5 (c) (step-1 code audit): the SW35 -> SW36 part no longer uses assert(). assert() is
//   compiled out under -DNDEBUG, and two of these checks CALLED the code under test inside the assert (the Base64
//   encode and the migration itself), so a -DNDEBUG build would have skipped the migration and checked nothing. Every
//   check is now MCHECK(), which always evaluates its argument and counts, like T19's own T19CHECK(); main() returns
//   failure if either counter failed.
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

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "../../Source/Common/ConfigMigrate.h"

// MCHECK - one check of the SW35 -> SW36 part: ALWAYS evaluates its argument (unlike assert(), which -DNDEBUG removes
// together with any call inside it), counts it, and prints the line of a failure. Side effects: the two counters.
static int g_mig_checks = 0, g_mig_fails = 0;
#define MCHECK(c) do { g_mig_checks++; if (!(c)) { g_mig_fails++; printf("  SW35->36 FAIL line %d: %s\n", __LINE__, #c); } } while (0)

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

// ============================================================
// T19 - V2.5-Evo - 2026-10-20 - multi-remote v3.2 step 1: the SW36 -> SW37 (RX) and SW27 -> SW29 (TX) prefix
//   migrations and the receiver's identity rule, on SYNTHETIC images. Real ?conf exports are read only at run time,
//   from paths given on the command line (the gitignored backup folder); without them the real-image part prints
//   SKIPPED. Nothing from a real image is printed. The SW37 / SW29 layouts below are the design's (v3.1 section 14);
//   the receiver and remote steps must match these offsets.
// ============================================================
#include "../../Source/Common/MultiTx.h"
#include <stdio.h>

// The exact CRC both boards use (Common/SystemCommon.h esp_crc8(), copied: that header needs Arduino).
static uint8_t crc8(uint8_t* data, uint8_t length)
{
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

// The RX struct as it stands at SW36: bytes 198-199 are fm_front_ahead_extra_m since 2026-10-02 (no pad left).
struct ConfSw36Full {
  ConfSw35 sw35;
  uint16_t fm_return_mode;          // 192
  uint16_t fm_align_cap;            // 194
  uint16_t fm_align_influence;      // 196
  uint16_t fm_front_ahead_extra_m;  // 198
};
static_assert(sizeof(ConfSw36Full) == 200, "SW36 RX struct is 200 bytes with no pad");
// SW37 = SW36 + the multi-remote tail (design v3.1 14.1, mtx_claim_s per v3.2 section 7).
struct ConfSw37 {
  ConfSw36Full sw36;                // 0-199, byte-exact prefix
  uint8_t  mtx_id[12];              // 200
  uint8_t  mtx_flags[4];            // 212
  uint16_t mtx_claim_s;             // 216, default 10
  uint16_t mtx_seize_en;            // 218, default 1
  uint16_t mtx_takeover_cap;        // 220, default 64
  uint16_t mtx_takeover_ms;         // 222, default 4000
  uint16_t mtx_rsvd[4];             // 224, 0
};
static_assert(sizeof(ConfSw37) == 232, "SW37 RX struct is 232 bytes");
static_assert(offsetof(ConfSw37, mtx_id) == 200 && offsetof(ConfSw37, mtx_flags) == 212, "SW37 list offsets");
static_assert(offsetof(ConfSw37, mtx_claim_s) == 216 && offsetof(ConfSw37, mtx_seize_en) == 218, "SW37 settings offsets");
static_assert(offsetof(ConfSw37, mtx_takeover_cap) == 220 && offsetof(ConfSw37, mtx_takeover_ms) == 222, "SW37 limit offsets");
static_assert(offsetof(ConfSw37, mtx_rsvd) == 224, "SW37 reserved offset");
// TX: SW27 is 136 bytes with no pad (kept opaque here: the migration does not read it); SW29 appends 8 bytes.
struct ConfTx29Tail {
  uint16_t autolock_timeout_s;      // 136, default 0
  uint16_t mtx_link_proto;          // 138, default 0 (a migrated pairing is a legacy record)
  uint16_t mtx_seize_hold_ms;       // 140, default 1200
  uint16_t mtx_autolock_s;          // 142, default 120
};
struct ConfTx29 { uint8_t sw27[136]; ConfTx29Tail tail; };
static_assert(sizeof(ConfTx29) == 144 && offsetof(ConfTx29, tail) == 136, "SW29 TX struct is 144 bytes");

static int g_t19_checks = 0, g_t19_fails = 0;
#define T19CHECK(c) do { g_t19_checks++; if (!(c)) { g_t19_fails++; printf("  T19 FAIL line %d: %s\n", __LINE__, #c); } } while (0)

// The receiver's binding fields, read from / written back into a migrated SW37 image.
static MtxRxBinding bindingOf(const ConfSw37& c)
{
  MtxRxBinding b;
  b.paired = c.sw36.sw35.paired;
  memcpy(b.own_address, c.sw36.sw35.own_address, 3);
  memcpy(b.dest_address, c.sw36.sw35.dest_address, 3);
  memcpy(b.mtx_id, c.mtx_id, 12);
  memcpy(b.mtx_flags, c.mtx_flags, 4);
  return b;
}
static void putBinding(ConfSw37* c, const MtxRxBinding& b)
{
  c->sw36.sw35.paired = b.paired;
  memcpy(c->sw36.sw35.own_address, b.own_address, 3);
  memcpy(c->sw36.sw35.dest_address, b.dest_address, 3);
  memcpy(c->mtx_id, b.mtx_id, 12);
  memcpy(c->mtx_flags, b.mtx_flags, 4);
}

// The factory-default SW37 image: 0xA5 everywhere the migration must NOT read, the tail at its defaults.
static void defaults37(ConfSw37* d)
{
  memset(d, 0xA5, sizeof(*d));
  d->sw36.sw35.version = 37;
  memset(d->mtx_id, 0, 12); memset(d->mtx_flags, 0, 4);
  d->mtx_claim_s = 10; d->mtx_seize_en = 1; d->mtx_takeover_cap = 64; d->mtx_takeover_ms = 4000;
  memset(d->mtx_rsvd, 0, sizeof(d->mtx_rsvd));
}

// One RX SW36 image (len 200) through the SW37 migration and the identity rule: with a chip ID equal to the stored
// address the binding is kept (and, if paired, the implicit legacy entry forms); with other_id it is dropped, and
// nothing but the binding changes. Inputs: blob/len, other_id, expect_paired. Side effects: the T19 counters.
static void t19Rx(const uint8_t* blob, size_t len, const uint8_t other_id[3], bool expect_paired)
{
  ConfSw37 d; defaults37(&d);
  T19CHECK(cfgLegacyPairMatches(len, (uint16_t)(blob[0] | (blob[1] << 8)), 200, 36, true));
  ConfSw37 m; memset(&m, 0x5A, sizeof(m));
  T19CHECK(cfgPrefixMigrateImage(blob, len, 200, (const uint8_t*)&d, sizeof(ConfSw37), (uint8_t*)&m));
  T19CHECK(memcmp(&m, blob, 200) == 0);                                    // the whole SW36 prefix, byte for byte
  m.sw36.sw35.version = 37;
  T19CHECK(memcmp((const uint8_t*)&m + 200, (const uint8_t*)&d + 200, 32) == 0);   // the tail = the defaults
  T19CHECK(m.mtx_claim_s == 10 && m.mtx_seize_en == 1 && m.mtx_takeover_cap == 64 && m.mtx_takeover_ms == 4000);
  // Identity: the chip ID equals the stored address -> nothing changes, the implicit legacy entry forms (SINGLE).
  MtxRxBinding b = bindingOf(m);
  uint8_t own[3]; memcpy(own, b.own_address, 3);
  const MtxRxBinding before = b;
  const uint8_t n_same = mtxValidateRxBinding(&b, own);
  if (expect_paired) {
    T19CHECK(n_same == 0 && memcmp(&b, &before, sizeof(b)) == 0);                       // validation passes, unchanged
    T19CHECK(mtxImplicitLegacy(&b, own) && mtxRxMode(&b, own) == MTX_MODE_SINGLE);
    T19CHECK(mtxLookupSlot(&b, own, own) == kMtxSlotImplicit);
  }
  // Identity: another chip ID -> the binding is dropped, and only the binding.
  MtxRxBinding x = bindingOf(m);
  const uint8_t notes = mtxValidateRxBinding(&x, other_id);
  T19CHECK((notes & (MTX_NOTE_ID_MISMATCH | MTX_NOTE_ID_NOT_SET)) != 0);
  T19CHECK(x.paired == 0 && mtxIdIsZero(x.dest_address) && mtxIdEqual(x.own_address, other_id));
  T19CHECK(mtxRxMode(&x, other_id) == MTX_MODE_UNPAIRED);
  ConfSw37 kept = m; putBinding(&kept, x);
  const ConfSw35& a = m.sw36.sw35; const ConfSw35& k = kept.sw36.sw35;
  T19CHECK(memcmp(&a, &k, offsetof(ConfSw35, paired)) == 0);                 // everything before the binding
  T19CHECK(memcmp((const uint8_t*)&a + offsetof(ConfSw35, wifi_password), (const uint8_t*)&k + offsetof(ConfSw35, wifi_password),
                  sizeof(ConfSw35) - offsetof(ConfSw35, wifi_password)) == 0);   // everything after it (compass cal, tuning)
  T19CHECK(memcmp(&m.sw36.fm_return_mode, &kept.sw36.fm_return_mode, 8) == 0);
}

// One TX SW27 image (len 136) through the SW29 migration: prefix byte-exact, tail = defaults, a legacy link record.
static void t19Tx(const uint8_t* blob, size_t len)
{
  ConfTx29 d; memset(&d, 0xA5, sizeof(d));
  d.sw27[0] = 29; d.sw27[1] = 0;
  d.tail.autolock_timeout_s = 0; d.tail.mtx_link_proto = 0; d.tail.mtx_seize_hold_ms = 1200; d.tail.mtx_autolock_s = 120;
  T19CHECK(cfgLegacyPairMatches(len, (uint16_t)(blob[0] | (blob[1] << 8)), 136, 27, true));
  ConfTx29 m; memset(&m, 0x5A, sizeof(m));
  T19CHECK(cfgPrefixMigrateImage(blob, len, 136, (const uint8_t*)&d, sizeof(ConfTx29), (uint8_t*)&m));
  T19CHECK(memcmp(&m, blob, 136) == 0);
  T19CHECK(memcmp(&m.tail, &d.tail, sizeof(d.tail)) == 0);
  T19CHECK(m.tail.mtx_link_proto == kMtxProtoLegacy && m.tail.mtx_seize_hold_ms == 1200 && m.tail.mtx_autolock_s == 120);
  uint16_t paired = 1, proto = m.tail.mtx_link_proto;
  T19CHECK(mtxValidateTxLink(&paired, &proto) == 0 && paired == 1 && proto == 0);   // the pairing survives as proto 0
}

// Read one ?conf export: the "Encoded Data Read: <base64>" line, and the banner "MAC: <12 hex>" if present.
static size_t readConfExport(const char* path, uint8_t* out, size_t cap, uint64_t* mac, bool* has_mac)
{
  FILE* f = fopen(path, "r");
  if (!f) return 0;
  char line[1024]; size_t n = 0; *has_mac = false;
  while (fgets(line, sizeof(line), f)) {
    const char* p = strstr(line, "Encoded Data Read: ");
    if (p && n == 0) {
      char b64[1024]; size_t j = 0;
      for (const char* q = p + 19; *q && *q != '\r' && *q != '\n' && j + 1 < sizeof(b64); ++q) b64[j++] = *q;
      b64[j] = 0;
      n = b64decode(b64, out, cap);
    }
    const char* mp = strstr(line, "MAC: ");
    if (mp && !*has_mac) {
      uint64_t v = 0; int d = 0;
      for (const char* q = mp + 5; *q && d < 12; ++q) {
        const char c = *q; int h;
        if (c >= '0' && c <= '9') h = c - '0'; else if (c >= 'A' && c <= 'F') h = c - 'A' + 10;
        else if (c >= 'a' && c <= 'f') h = c - 'a' + 10; else break;
        v = (v << 4) | (uint64_t)h; d++;
      }
      if (d == 12) { *mac = v; *has_mac = true; }
    }
  }
  fclose(f);
  return n;
}

static void t19(int argc, char** argv)
{
  // ---- Synthetic RX SW36 image: the synthetic SW35 values plus the four SW36 fields, padding-free ----
  ConfSw36Full s36;
  fillSyntheticSw35(&s36.sw35);
  s36.sw35.version = 36;
  s36.fm_return_mode = 0; s36.fm_align_cap = 21; s36.fm_align_influence = 66; s36.fm_front_ahead_extra_m = 7;
  const uint8_t other[3] = { 0x5C, 0x10, 0x21 };                          // a made-up chip ID, not the stored address
  t19Rx((const uint8_t*)&s36, sizeof(s36), other, true);
  // An SW35 blob is not migratable onto SW37; an SW37 blob is current, not a legacy pair.
  T19CHECK(!cfgLegacyPairMatches(192, 35, 200, 36, true));
  T19CHECK(!cfgLegacyPairMatches(232, 37, 200, 36, true));
  T19CHECK(!cfgLegacyPairMatches(200, 35, 200, 36, true));
  // ---- Synthetic TX SW27 image ----
  uint8_t tx27[136];
  for (size_t i = 0; i < sizeof(tx27); ++i) tx27[i] = (uint8_t)(0x21 + 5 * i);
  tx27[0] = 27; tx27[1] = 0;
  t19Tx(tx27, sizeof(tx27));
  T19CHECK(!cfgLegacyPairMatches(136, 26, 136, 27, true));
  // ---- Real exports, local only: paths on the command line. Nothing from them is printed. ----
  int real = 0;
  for (int i = 1; i < argc; i++) {
    uint8_t img[512]; uint64_t mac = 0; bool has_mac = false;
    const size_t n = readConfExport(argv[i], img, sizeof(img), &mac, &has_mac);
    const uint16_t ver = (n >= 2) ? (uint16_t)(img[0] | (img[1] << 8)) : 0;
    if (n == 200 && ver == 36) {
      uint8_t notme[3] = { (uint8_t)(img[82] ^ 0xFF), img[83], img[84] };  // certainly not the stored address
      t19Rx(img, n, notme, img[80] != 0);
      if (has_mac) {
        uint8_t chip[3]; efuseIdFromMac(mac, crc8, chip);
        MtxRxBinding b; memset(&b, 0, sizeof(b)); b.paired = (uint16_t)(img[80] | (img[81] << 8));
        memcpy(b.own_address, img + 82, 3); memcpy(b.dest_address, img + 85, 3);
        mtxValidateRxBinding(&b, chip);
        printf("     T19 real RX image %d: migrated; at its first SW37 boot its binding is %s\n", i,
               b.paired ? "KEPT (stored address = chip ID)" : "DROPPED (stored address is not its chip ID) - pair again");
      } else {
        printf("     T19 real RX image %d: migrated (no banner MAC in the file, identity prediction skipped)\n", i);
      }
      real++;
    } else if (n == 136 && ver == 27) {
      t19Tx(img, n);
      printf("     T19 real TX image %d: migrated, legacy link record\n", i);
      real++;
    } else {
      printf("     T19 file %d: not an SW36 RX or SW27 TX export - not used\n", i);
    }
  }
  if (real == 0) printf("     T19 real images: SKIPPED (no local images)\n");
  printf("T19  %s (%d checks)\n", g_t19_fails ? "FAIL" : "ok  ", g_t19_checks);
}

int main(int argc, char** argv)
{
  // ---- Build the synthetic image, export it as ?conf would, and decode it back ----
  ConfSw35 synth;
  fillSyntheticSw35(&synth);
  char b64[512];
  MCHECK(b64encode((const uint8_t*)&synth, sizeof(synth), b64, sizeof(b64)) == 256);   // 192 bytes -> 256 chars
  uint8_t blob[256];
  const size_t blobLen = b64decode(b64, blob, sizeof(blob));
  MCHECK(memcmp(blob, &synth, sizeof(synth)) == 0);                                     // the round trip is exact
  MCHECK(blobLen == kLegacyLen);
  const uint16_t blobVersion = (uint16_t)(blob[0] | ((uint16_t)blob[1] << 8));
  MCHECK(blobVersion == kLegacySw);

  ConfSw35 old;
  memcpy(&old, blob, sizeof(old));
  // Spot-check the decode against the values the image was built from.
  MCHECK(old.version == 35 && old.radio_preset == 1 && old.rf_power == 14);
  MCHECK(old.steering_type == 2 && old.steering_influence == 41 && old.steering_inverted == 0);
  MCHECK(old.paired == 1);
  MCHECK(old.rtm_stop_distance_m == 9);
  MCHECK(old.mag_orientation == 90);

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
  MCHECK(cfgLegacyPairMatches(blobLen, blobVersion, kLegacyLen, kLegacySw, true));
  MCHECK(!cfgLegacyPairMatches(blobLen, blobVersion, kLegacyLen, kLegacySw, false));   // build not the target
  MCHECK(!cfgLegacyPairMatches(184, 34, kLegacyLen, kLegacySw, true));                 // the SW34 blob: one version too old
  MCHECK(!cfgLegacyPairMatches(192, 34, kLegacyLen, kLegacySw, true));                 // legacy length, wrong version
  MCHECK(!cfgLegacyPairMatches(200, 36, kLegacyLen, kLegacySw, true));                 // already current
  MCHECK(!cfgLegacyPairMatches(191, 35, kLegacyLen, kLegacySw, true));                 // one byte short

  // ---- Migrate exactly as the RX does, then stamp the version as the firmware does ----
  ConfSw36 migrated;
  memset(&migrated, 0x5A, sizeof(migrated));
  MCHECK(cfgPrefixMigrateImage(blob, blobLen, kLegacyLen,
                               (const uint8_t*)&defaults, kTargetLen, (uint8_t*)&migrated));
  // The whole prefix is the blob, byte for byte - before the stamp the version is still 35.
  MCHECK(memcmp(&migrated, blob, kLegacyLen) == 0);
  migrated.sw35.version = kTargetSw;
  // After the stamp: everything except the two version bytes is still the blob.
  MCHECK(memcmp((const uint8_t*)&migrated + 2, blob + 2, kLegacyLen - 2) == 0);
  MCHECK(migrated.sw35.version == 36);

  // ---- The three new fields equal their defaults; the pad is 0; no 0xA5 anywhere in the prefix ----
  MCHECK(migrated.fm_return_mode == 1);
  MCHECK(migrated.fm_align_cap == 13);
  MCHECK(migrated.fm_align_influence == 80);
  const uint8_t* mb = (const uint8_t*)&migrated;
  MCHECK(mb[198] == 0 && mb[199] == 0);
  for (size_t i = 2; i < kLegacyLen; ++i) MCHECK(mb[i] == blob[i]);

  // ---- Every field the owner would notice losing, by name and value (the values the image was built from) ----
  const ConfSw35& m = migrated.sw35;
  MCHECK(m.paired == 1);
  MCHECK(memcmp(m.own_address, kSynOwn, 3) == 0);     // the identity fields survive the migration untouched
  MCHECK(memcmp(m.dest_address, kSynDest, 3) == 0);
  MCHECK(memcmp(m.wifi_password, "synthpw1", 8) == 0);
  MCHECK(m.steering_type == 2 && m.steering_influence == 41 && m.steering_inverted == 0 && m.trim == -3);
  MCHECK(m.PWM0_min == 1010 && m.PWM0_max == 1990 && m.PWM1_min == 1020 && m.PWM1_max == 1980);
  MCHECK(m.failsafe_time == 1200 && m.foil_num_cells == 12);
  MCHECK(m.bms_det_active == 1 && m.wet_det_active == 0);
  MCHECK(m.rtm_steer_response == 3 && m.data_src == 1 && m.gps_en == 1 && m.followme_mode == 3 && m.kalman_en == 0);
  MCHECK(near(m.boogie_vmax_in_followme_kmh, 21.5f, 0.001f));
  MCHECK(near(m.min_dist_m, 6.5f, 0.001f) && near(m.followme_smoothing_band_m, 3.5f, 0.001f));
  MCHECK(near(m.foiler_low_speed_kmh, 7.25f, 0.001f));
  MCHECK(near(m.zone_angle_enter_deg, 30.0f, 0.001f) && near(m.zone_angle_exit_deg, 40.0f, 0.001f));
  MCHECK(near(m.near_diag_offset_deg, 55.0f, 0.001f));
  MCHECK(near(m.ubat_cal, 0.0101f, 1e-7f) && near(m.ubat_offset, 0.125f, 1e-6f));
  MCHECK(m.tx_gps_stale_timeout_ms == 2500 && m.logger_en == 1);
  MCHECK(m.mag_offset_x == 321 && m.mag_offset_y == -123);
  MCHECK(near(m.mag_scale_x, 0.95f, 0.0005f) && near(m.mag_scale_y, -1.05f, 0.0005f));   // the mirrored frame survives
  MCHECK(m.gps_chip_type == 2 && m.gps_dyn_model == 2);
  MCHECK(near(m.gps_max_hdop, 1.5f, 0.001f) && near(m.gps_max_accel_g, 2.5f, 0.001f));
  MCHECK(near(m.gps_max_teleport_kmh, 70.0f, 0.001f) && m.gps_suspect_threshold == 4);
  MCHECK(near(m.gps_max_pair_dist_m, 400.0f, 0.001f) && near(m.gps_max_speed_diff_kmh, 40.0f, 0.001f));
  MCHECK(near(m.rtm_vesc_speed_diff_kmh, 15.0f, 0.001f) && near(m.vesc_erpm_per_kmh, 123.5f, 1e-4f));
  MCHECK(m.rtm_rx_enabled == 1 && m.rtm_rx_override_steering == 0 && m.rtm_compass_required == 1);
  MCHECK(m.rtm_stop_distance_m == 9);      // the RTM stop distance - also the FM_RETURN stop radius
  MCHECK(m.vesc_timeout_s == 8);           // the stored value, not the newer 10 s default: nothing is "corrected"
  MCHECK(m.gps_update_hz == 5 && m.rtm_approach_zone_m == 11);
  MCHECK(m.rtm_use_compass == 0 && m.rtm_cog_min_speed_kmh == 4);
  MCHECK(near(m.rtm_target_speed_kmh, 4.5f, 0.001f) && m.rtm_align_threshold_deg == 35);
  MCHECK(near(m.motor_ramp_s, 1.5f, 0.001f));
  MCHECK(near(m.fm_engage_dist_m, 10.5f, 0.001f));
  MCHECK(m.log_level == 3);
  MCHECK(m.mag_orientation == 90);
  // The old-named slot reads 0 on the migrated image: a migrated board reads steer_during_auto 0 = cancel, the
  // behaviour-preserving default the rename-in-place rules require (2026-09-19).
  MCHECK(m.rsvd_u16_1 == 0 && near(m.rsvd_f32_1, 0.0f, 1e-6f));

  // ---- Refusals: the image is never reinterpreted when the sizes do not say it is a prefix ----
  ConfSw36 scratch;
  memset(&scratch, 0x5A, sizeof(scratch));
  MCHECK(!cfgPrefixMigrateImage(blob, 184, kLegacyLen, (const uint8_t*)&defaults, kTargetLen, (uint8_t*)&scratch));  // wrong blob length
  MCHECK(!cfgPrefixMigrateImage(blob, blobLen, 0, (const uint8_t*)&defaults, kTargetLen, (uint8_t*)&scratch));      // empty legacy size
  MCHECK(!cfgPrefixMigrateImage(blob, blobLen, kLegacyLen, (const uint8_t*)&defaults, 100, (uint8_t*)&scratch));    // target smaller than the prefix
  MCHECK(!cfgPrefixMigrateImage(NULL, blobLen, kLegacyLen, (const uint8_t*)&defaults, kTargetLen, (uint8_t*)&scratch));
  MCHECK(!cfgPrefixMigrateImage(blob, blobLen, kLegacyLen, NULL, kTargetLen, (uint8_t*)&scratch));
  MCHECK(!cfgPrefixMigrateImage(blob, blobLen, kLegacyLen, (const uint8_t*)&defaults, kTargetLen, NULL));
  const uint8_t* sb = (const uint8_t*)&scratch;
  for (size_t i = 0; i < kTargetLen; ++i) MCHECK(sb[i] == 0x5A);   // a refused migration writes nothing

  // A same-size migration (legacy_len == target_len) is legal and is a plain copy.
  MCHECK(cfgPrefixMigrateImage(blob, blobLen, kLegacyLen, (const uint8_t*)&defaults, kLegacyLen, (uint8_t*)&scratch));
  MCHECK(memcmp(&scratch, blob, kLegacyLen) == 0);

  printf("SW35->36 %s (%d checks)\n", g_mig_fails ? "FAIL" : "ok  ", g_mig_checks);

  // ---- T19: the multi-remote migrations and the identity rule (real exports only from the command line) ----
  t19(argc, argv);
  return (g_mig_fails || g_t19_fails) ? 1 : 0;
}
