// V2.5-Evo - 2026-10-08 - LOG LEVEL 6 (IMU): new file. The VESC IMU reply parser and the 22 B log block
//   encoder, pure and host-testable (no Arduino, no FreeRTOS, no globals).
//
//   WHAT IT IS FOR: log level 6 records VESC 2's IMU (roll / pitch / yaw, gyro, accelerometer) so the
//   launch-boost tilt thresholds in VESC 2's script can be tuned from real rides, and so buggy heel in
//   turns and gybes can be read off the log. The RX polls VESC 2 through VESC 1 over CAN
//   (COMM_FORWARD_CAN carrying COMM_GET_IMU_DATA) - VESC.ino pollVesc2ImuIfDue() does the I/O, this
//   header only turns bytes into numbers and numbers into a log block.
//
//   THE REPLY (verified against vedderb/bldc comm/commands.c, case COMM_GET_IMU_DATA, in release_6_06,
//   and the same shape in release_5_03 / 6_02 / 6_05):
//     [65 COMM_GET_IMU_DATA][mask u16 big-endian, echoed][one float32_auto per set mask bit][controller id u8]
//   Mask bits, in the order the floats are appended: 0-2 roll, pitch, yaw (imu_get_rpy(), RADIANS - the
//   AHRS returns atan2f / asinf), 3-5 acc x, y, z (g), 6-8 gyro x, y, z (deg/s), 9-11 magnetometer,
//   12-15 quaternion. We ask for bits 0-8 (mask 0x01FF), so the payload is 1 + 2 + 9 x 4 + 1 = 40 bytes.
//   The trailing controller-ID byte lets the parser prove the reply came from VESC 2 itself.
//
//   Tools/tests/vesc_imu_test.cpp runs this exact code on the PC.
#ifndef BREMOTE_VESC_IMU_H
#define BREMOTE_VESC_IMU_H

#include <stdint.h>
#include <math.h>

// Defined in Source/V2_Integration_Rx/vesc_buffer.cpp (VESC's own float encoding, exponent bias 126).
// Declared here, identically to vesc_buffer.h, so this header stays free of any sketch-folder include path.
float buffer_get_float32_auto(const uint8_t *buffer, int32_t *index);

// ============================================================
// THE LOG BLOCK - 22 bytes, used twice in the level-6 record (VESC 2's IMU, then the reserved RX IMU)
// ============================================================
// Every value field has a non-zero "no data" sentinel (project struct rule: zero must never read as
// valid). OK is status 1, not 0, for the same reason. Real values are clamped strictly inside their
// sentinel, so a reading can never be mistaken for "no data".
struct __attribute__((packed)) ImuLogBlock {
  uint16_t age_ms;        // ms since the source's last validated sample; 0xFFFF = never; capped 0xFFFE
  int16_t  roll_cdeg;     // degrees x 100, clamped +-18000; 0x7FFF = N/A
  int16_t  pitch_cdeg;    // degrees x 100, clamped +-18000; 0x7FFF = N/A
  int16_t  yaw_cdeg;      // degrees x 100, clamped +-18000; 0x7FFF = N/A (6-axis yaw drifts; short-term use only)
  int16_t  gyro_x_ddps;   // deg/s x 10, clamped +-32000; 0x7FFF = N/A
  int16_t  gyro_y_ddps;   // deg/s x 10 (Y = pitch rate with X to the nose: the boost script's 80 dps axis)
  int16_t  gyro_z_ddps;   // deg/s x 10 (Z = yaw / turn rate)
  int16_t  acc_x_mg;      // milli-g, clamped +-32000; 0x7FFF = N/A
  int16_t  acc_y_mg;      // milli-g
  int16_t  acc_z_mg;      // milli-g
  uint8_t  status;        // 1 OK, 2 STALE, 3 IMPLAUSIBLE, 0xFF NO SOURCE; 0 = never written (read as N/A)
  uint8_t  rsvd;          // 0. RX block: future source code (1 = RX I2C IMU, 2 = VESC 1 IMU over UART). Not a CSV column.
};
static_assert(sizeof(ImuLogBlock) == 22, "ImuLogBlock must be 22 bytes - the level-6 record and the PC log reader depend on it");

// Status codes written into ImuLogBlock.status.
static const uint8_t kImuStatusOk          = 1;     // fresh and plausible: values are real
static const uint8_t kImuStatusStale       = 2;     // last sample older than the stale limit: real age kept, values N/A
static const uint8_t kImuStatusImplausible = 3;     // reply valid but |acc| < 0.5 g or a value is not finite (IMU off / not detected): values N/A
static const uint8_t kImuStatusNoSource    = 0xFF;  // never answered this session, or no such IMU in this firmware

// Sentinels and clamps (all real values are clamped strictly below the 0x7FFF sentinel).
static const uint16_t kImuAgeNever   = 0xFFFF;
static const uint16_t kImuAgeCap     = 0xFFFE;
static const int16_t  kImuNaI16      = 0x7FFF;
static const int32_t  kImuRpyClamp   = 18000;   // +-180.00 deg
static const int32_t  kImuWideClamp  = 32000;   // +-3200.0 deg/s and +-32.000 g - the LSM6DS3 tops out at 2000 dps / 16 g, so real data never reaches it

// ============================================================
// THE REQUEST / REPLY CONSTANTS
// ============================================================
static const uint8_t  kVescImuCommId    = 65;       // COMM_GET_IMU_DATA (vesc_datatypes.h; static_assert in VESC.ino)
static const uint16_t kVescImuMask      = 0x01FF;   // bits 0-8: roll, pitch, yaw, acc xyz, gyro xyz
static const int      kVescImuReplyLen  = 40;       // id 1 + mask 2 + 9 floats x 4 + controller id 1
static const bool     kImuRpyIsRadians  = true;     // imu_get_rpy() returns radians (AHRS atan2f / asinf); flip ONLY if the bench shows pitch ~1150 for a 20 deg lift
static const float    kImuMinAccG       = 0.5f;     // a working accelerometer at rest reads ~1 g; below 0.5 g the IMU is off or not detected

// One parsed sample, already in the units the log uses (degrees, g, deg/s). Axes are VESC 2's own
// frame as set in VESC Tool (X to the nose) - no sign flips here, so thresholds read off the log apply
// 1:1 to the script's get-imu-rpy / get-imu-gyro values.
struct VescImuSample {
  float rpy_deg[3];    // roll, pitch, yaw, degrees
  float acc_g[3];      // x, y, z, g
  float gyro_dps[3];   // x, y, z, deg/s
};

// ============================================================
// vescImuAccMagG - length of the acceleration vector, in g
// Inputs: s - a sample. Outputs: |acc| in g (non-finite if any component is). Side effects: none.
// ============================================================
static inline float vescImuAccMagG(const VescImuSample* s)
{
  return sqrtf(s->acc_g[0] * s->acc_g[0] + s->acc_g[1] * s->acc_g[1] + s->acc_g[2] * s->acc_g[2]);
}

// ============================================================
// vescImuPlausible - is this sample something the log may print as real?
// Inputs: s - a sample. Outputs: true when all nine values are finite and |acc| >= kImuMinAccG.
// Side effects: none. A powered IMU at rest reads ~1 g; an IMU that is disabled or not detected in
// VESC 2's App Settings answers with zeros, which this rejects (status 3, values -999 in the CSV).
// ============================================================
static inline bool vescImuPlausible(const VescImuSample* s)
{
  for (int i = 0; i < 3; ++i) {
    if (!isfinite(s->rpy_deg[i]) || !isfinite(s->acc_g[i]) || !isfinite(s->gyro_dps[i])) return false;
  }
  return vescImuAccMagG(s) >= kImuMinAccG;
}

// ============================================================
// vescImuParse - validate and decode one COMM_GET_IMU_DATA reply payload
//
// Inputs:  msg       - the reply PAYLOAD (receiveFromVESC() output: frame bytes already stripped)
//          len       - payload length receiveFromVESC() returned
//          expect_id - the CAN controller ID the reply must carry in its last byte (2 = VESC 2)
//          out       - where to write the sample; written ONLY on success
// Outputs: true when the reply is ours and every value is finite; false = treat as a missed poll.
// Checks, in order: length >= 40, command id 65, the exact mask 0x01FF echoed, controller id at byte
//   39 == expect_id (so VESC 1, or a stray frame, can never be stored as VESC 2), nine finite floats.
// Side effects: none.
// ============================================================
static inline bool vescImuParse(const uint8_t* msg, int len, uint8_t expect_id, VescImuSample* out)
{
  if (msg == 0 || out == 0) return false;
  if (len < kVescImuReplyLen) return false;
  if (msg[0] != kVescImuCommId) return false;
  if (msg[1] != (uint8_t)(kVescImuMask >> 8) || msg[2] != (uint8_t)(kVescImuMask & 0xFF)) return false;
  if (msg[kVescImuReplyLen - 1] != expect_id) return false;

  // The floats follow the mask in bit order: roll, pitch, yaw, acc x, y, z, gyro x, y, z.
  VescImuSample s;
  int32_t idx = 3;
  for (int i = 0; i < 3; ++i) s.rpy_deg[i]  = buffer_get_float32_auto(msg, &idx);
  for (int i = 0; i < 3; ++i) s.acc_g[i]    = buffer_get_float32_auto(msg, &idx);
  for (int i = 0; i < 3; ++i) s.gyro_dps[i] = buffer_get_float32_auto(msg, &idx);

  for (int i = 0; i < 3; ++i) {
    if (!isfinite(s.rpy_deg[i]) || !isfinite(s.acc_g[i]) || !isfinite(s.gyro_dps[i])) return false;
  }
  if (kImuRpyIsRadians) {
    for (int i = 0; i < 3; ++i) s.rpy_deg[i] *= (180.0f / 3.14159265358979f);
  }
  *out = s;
  return true;
}

// ============================================================
// imuClampScale - one engineering value -> one clamped, rounded int16 log field
// Inputs: v - the value, scale - the log's multiplier (100 for degrees, 10 for deg/s, 1000 for g),
//         lim - the clamp (kImuRpyClamp or kImuWideClamp). v must be finite (the caller checks).
// Outputs: round(v x scale), clamped to +-lim - never the 0x7FFF sentinel. Side effects: none.
// The clamp is applied in float BEFORE rounding so an absurd value cannot overflow lroundf().
// ============================================================
static inline int16_t imuClampScale(float v, float scale, int32_t lim)
{
  float x = v * scale;
  if (x >  (float)lim) x =  (float)lim;
  if (x < -(float)lim) x = -(float)lim;
  return (int16_t)lroundf(x);
}

// ============================================================
// imuLogNoSource - write a block that says "no IMU": every sentinel, status 0xFF
// Inputs: b - the block. Outputs: none. Side effects: none beyond *b.
// ============================================================
static inline void imuLogNoSource(ImuLogBlock* b)
{
  b->age_ms      = kImuAgeNever;
  b->roll_cdeg   = kImuNaI16;
  b->pitch_cdeg  = kImuNaI16;
  b->yaw_cdeg    = kImuNaI16;
  b->gyro_x_ddps = kImuNaI16;
  b->gyro_y_ddps = kImuNaI16;
  b->gyro_z_ddps = kImuNaI16;
  b->acc_x_mg    = kImuNaI16;
  b->acc_y_mg    = kImuNaI16;
  b->acc_z_mg    = kImuNaI16;
  b->status      = kImuStatusNoSource;
  b->rsvd        = 0;
}

// ============================================================
// imuLogEncode - turn the last stored sample into one log block, deciding freshness at log time
//
// Inputs:  b        - the block to fill
//          s        - the last validated sample (ignored unless ever_ok)
//          ever_ok  - false = the source has never answered this session
//          age_ms   - ms since that sample was stored
//          stale_ms - older than this is STALE (values not printed as live)
// Outputs: none (b is filled). Side effects: none.
// Order: the sentinels are written FIRST, so any early return leaves a block that reads "no data":
//   never answered -> status 0xFF, age 0xFFFF
//   older than stale_ms -> status 2, the real age, values N/A
//   not plausible (non-finite or |acc| < 0.5 g) -> status 3, the real age, values N/A
//   otherwise -> status 1 and every value
// ============================================================
static inline void imuLogEncode(ImuLogBlock* b, const VescImuSample* s, bool ever_ok, uint32_t age_ms, uint32_t stale_ms)
{
  imuLogNoSource(b);
  if (!ever_ok || s == 0) return;

  b->age_ms = (uint16_t)((age_ms > kImuAgeCap) ? kImuAgeCap : age_ms);
  if (age_ms > stale_ms) { b->status = kImuStatusStale; return; }
  if (!vescImuPlausible(s)) { b->status = kImuStatusImplausible; return; }

  b->roll_cdeg   = imuClampScale(s->rpy_deg[0],  100.0f,  kImuRpyClamp);
  b->pitch_cdeg  = imuClampScale(s->rpy_deg[1],  100.0f,  kImuRpyClamp);
  b->yaw_cdeg    = imuClampScale(s->rpy_deg[2],  100.0f,  kImuRpyClamp);
  b->gyro_x_ddps = imuClampScale(s->gyro_dps[0], 10.0f,   kImuWideClamp);
  b->gyro_y_ddps = imuClampScale(s->gyro_dps[1], 10.0f,   kImuWideClamp);
  b->gyro_z_ddps = imuClampScale(s->gyro_dps[2], 10.0f,   kImuWideClamp);
  b->acc_x_mg    = imuClampScale(s->acc_g[0],    1000.0f, kImuWideClamp);
  b->acc_y_mg    = imuClampScale(s->acc_g[1],    1000.0f, kImuWideClamp);
  b->acc_z_mg    = imuClampScale(s->acc_g[2],    1000.0f, kImuWideClamp);
  b->status      = kImuStatusOk;
}

#endif // BREMOTE_VESC_IMU_H
