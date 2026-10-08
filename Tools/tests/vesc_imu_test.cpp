// V2.5-Evo - 2026-10-08 - LOG LEVEL 6 (IMU): host test for Common/VescImu.h - the COMM_GET_IMU_DATA
// reply parser and the 22 B log block encoder. Replies are built with VESC's own encoder
// (buffer_append_float32_auto), so the round trip exercises the exact bytes a VESC sends.
//
// vesc_buffer.cpp is pulled in with #include (rather than linked on the command line) so this file
// builds with the same one-file-per-test loop as every other test here:
//   g++ -std=c++17 -O1 -Wall -Wextra vesc_imu_test.cpp -o vesc_imu_test.exe

#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "../../Source/V2_Integration_Rx/vesc_buffer.cpp"
#include "../../Source/Common/VescImu.h"

static const float kPi = 3.14159265358979f;

// Build a 40-byte reply payload exactly as VESC FW 6.06 commands.c does: id, mask u16, the nine
// float32_auto values in mask-bit order (roll, pitch, yaw RADIANS; acc g; gyro dps), controller id.
static int buildReply(uint8_t* buf, uint16_t mask, const float rpy_rad[3], const float acc[3],
                      const float gyro[3], uint8_t ctrl_id)
{
  int32_t i = 0;
  buf[i++] = 65;
  buffer_append_uint16(buf, mask, &i);
  for (int k = 0; k < 3; ++k) buffer_append_float32_auto(buf, rpy_rad[k], &i);
  for (int k = 0; k < 3; ++k) buffer_append_float32_auto(buf, acc[k], &i);
  for (int k = 0; k < 3; ++k) buffer_append_float32_auto(buf, gyro[k], &i);
  buf[i++] = ctrl_id;
  return (int)i;
}

static bool near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

int main()
{
  // ---- the layout the log and the PC reader depend on ----
  assert(sizeof(ImuLogBlock) == 22);
  assert(kVescImuReplyLen == 40);

  // ---- a good reply round-trips, radians become degrees ----
  const float rpy_rad[3] = { 15.0f * kPi / 180.0f, -20.0f * kPi / 180.0f, 90.0f * kPi / 180.0f };
  const float acc[3]     = { 0.12f, -0.34f, 0.98f };
  const float gyro[3]    = { 1.5f, -80.25f, 33.0f };
  uint8_t msg[48];
  int len = buildReply(msg, 0x01FF, rpy_rad, acc, gyro, 2);
  assert(len == 40);

  VescImuSample s;
  memset(&s, 0, sizeof(s));
  assert(vescImuParse(msg, len, 2, &s));
  assert(near(s.rpy_deg[0], 15.0f, 0.01f));
  assert(near(s.rpy_deg[1], -20.0f, 0.01f));
  assert(near(s.rpy_deg[2], 90.0f, 0.01f));
  assert(near(s.acc_g[0], 0.12f, 1e-5f) && near(s.acc_g[1], -0.34f, 1e-5f) && near(s.acc_g[2], 0.98f, 1e-5f));
  assert(near(s.gyro_dps[0], 1.5f, 1e-4f) && near(s.gyro_dps[1], -80.25f, 1e-4f) && near(s.gyro_dps[2], 33.0f, 1e-4f));
  // a newer firmware that appends bytes after the ID would still be read by the fixed offsets
  assert(vescImuParse(msg, 43, 2, &s));

  // ---- rejections: each one leaves `out` untouched ----
  VescImuSample keep;
  memset(&keep, 0x5A, sizeof(keep));
  VescImuSample probe = keep;
  assert(!vescImuParse(msg, 39, 2, &probe));                       // short (no controller-ID byte)
  assert(!vescImuParse(msg, len, 1, &probe));                      // controller ID is not VESC 2
  assert(memcmp(&probe, &keep, sizeof(keep)) == 0);
  uint8_t bad[48];
  memcpy(bad, msg, sizeof(bad)); bad[0] = 50;                      // wrong command id
  assert(!vescImuParse(bad, len, 2, &probe));
  int mlen = buildReply(bad, 0x00FF, rpy_rad, acc, gyro, 2);       // mask mismatch (echoed mask is not ours)
  assert(!vescImuParse(bad, mlen, 2, &probe));
  // a non-finite float (exponent 255 decodes to infinity) is a miss, never stored
  memcpy(bad, msg, sizeof(bad));
  bad[3 + 4 * 4] = 0x7F; bad[3 + 4 * 4 + 1] = 0x80; bad[3 + 4 * 4 + 2] = 0; bad[3 + 4 * 4 + 3] = 0;   // acc y = +inf
  assert(!vescImuParse(bad, len, 2, &probe));
  assert(memcmp(&probe, &keep, sizeof(keep)) == 0);
  assert(!vescImuParse(0, len, 2, &probe));

  // ---- encoder: never answered -> NO SOURCE, every sentinel ----
  ImuLogBlock b;
  memset(&b, 0, sizeof(b));
  imuLogEncode(&b, &s, false, 0, 1500);
  assert(b.status == 0xFF && b.age_ms == 0xFFFF);
  assert(b.roll_cdeg == 0x7FFF && b.pitch_cdeg == 0x7FFF && b.yaw_cdeg == 0x7FFF);
  assert(b.gyro_x_ddps == 0x7FFF && b.gyro_y_ddps == 0x7FFF && b.gyro_z_ddps == 0x7FFF);
  assert(b.acc_x_mg == 0x7FFF && b.acc_y_mg == 0x7FFF && b.acc_z_mg == 0x7FFF);
  assert(b.rsvd == 0);

  // ---- fresh and plausible -> OK (1, never 0) and scaled values ----
  imuLogEncode(&b, &s, true, 412, 1500);
  assert(b.status == 1 && b.age_ms == 412);
  assert(b.roll_cdeg == 1500 && b.pitch_cdeg == -2000 && b.yaw_cdeg == 9000);
  assert(b.gyro_x_ddps == 15 && b.gyro_y_ddps == -803 && b.gyro_z_ddps == 330);   // -80.25 x 10 rounds away from zero
  assert(b.acc_x_mg == 120 && b.acc_y_mg == -340 && b.acc_z_mg == 980);

  // ---- exactly at the stale limit is still fresh; one ms past it is STALE with the real age ----
  imuLogEncode(&b, &s, true, 1500, 1500);
  assert(b.status == 1);
  imuLogEncode(&b, &s, true, 1501, 1500);
  assert(b.status == 2 && b.age_ms == 1501 && b.roll_cdeg == 0x7FFF && b.acc_z_mg == 0x7FFF);
  imuLogEncode(&b, &s, true, 200000, 1500);                        // age caps at 0xFFFE, never the 0xFFFF "never" sentinel
  assert(b.status == 2 && b.age_ms == 0xFFFE);

  // ---- implausible: IMU off (all zeros), |acc| just under 0.5 g, or a NaN -> 3, values N/A ----
  VescImuSample z;
  memset(&z, 0, sizeof(z));
  imuLogEncode(&b, &z, true, 100, 1500);
  assert(b.status == 3 && b.age_ms == 100 && b.pitch_cdeg == 0x7FFF && b.gyro_y_ddps == 0x7FFF);
  VescImuSample low = s;
  low.acc_g[0] = 0.0f; low.acc_g[1] = 0.0f; low.acc_g[2] = 0.49f;
  imuLogEncode(&b, &low, true, 100, 1500);
  assert(b.status == 3);
  low.acc_g[2] = 0.5f;                                             // 0.5 g itself is accepted
  imuLogEncode(&b, &low, true, 100, 1500);
  assert(b.status == 1);
  VescImuSample nan_s = s;
  nan_s.gyro_dps[2] = NAN;
  imuLogEncode(&b, &nan_s, true, 100, 1500);
  assert(b.status == 3 && b.gyro_z_ddps == 0x7FFF);

  // ---- clamp edges never land on the 0x7FFF sentinel ----
  VescImuSample big = s;
  big.rpy_deg[0] = 400.0f;  big.rpy_deg[1] = -400.0f;              // outside +-180 (cannot happen, still clamped)
  big.gyro_dps[0] = 5000.0f; big.gyro_dps[1] = -5000.0f;            // beyond +-3200 dps
  big.acc_g[0] = 40.0f; big.acc_g[1] = -40.0f; big.acc_g[2] = 1.0f; // beyond +-32 g
  imuLogEncode(&b, &big, true, 10, 1500);
  assert(b.status == 1);
  assert(b.roll_cdeg == 18000 && b.pitch_cdeg == -18000);
  assert(b.gyro_x_ddps == 32000 && b.gyro_y_ddps == -32000);
  assert(b.acc_x_mg == 32000 && b.acc_y_mg == -32000);
  assert(b.roll_cdeg != 0x7FFF && b.gyro_x_ddps != 0x7FFF && b.acc_x_mg != 0x7FFF);
  big.rpy_deg[0] = 1.0e30f;                                        // huge but finite: clamped before rounding
  imuLogEncode(&b, &big, true, 10, 1500);
  assert(b.roll_cdeg == 18000);

  // ---- the reserved RX block: always NO SOURCE ----
  memset(&b, 0, sizeof(b));
  imuLogNoSource(&b);
  assert(b.status == 0xFF && b.age_ms == 0xFFFF && b.acc_z_mg == 0x7FFF && b.rsvd == 0);

  // ---- a zero-initialised block never reads as OK ----
  ImuLogBlock zero;
  memset(&zero, 0, sizeof(zero));
  assert(zero.status != 1);

  printf("vesc_imu_test: all checks passed\n");
  return 0;
}
