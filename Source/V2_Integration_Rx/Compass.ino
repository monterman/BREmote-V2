// V2.5-Evo - 2026-07-22 - runCompassCalibration() now clamps mag_scale_x/y to [0.1, 10.0] (the kCfgFields validator range) before cmdSave(), so a pathological high-asymmetry cal can never save a scale that fails config-load validation and wipes config+pairing on next boot. Value-clamp only — no confStruct/SW_VERSION change.
// V2.5-Evo - 2026-07-21 - Compass hardening: (1) getCompassHeading()/updateCompassSnapshot() now reject the UNCALIBRATED degenerate default (offset 0/0 + scale 1/1), not just scale==0, so FM/RTM never steer on a garbage heading; (2) runCompassCalibration() requires compass at entry and aborts on a no-sample collection (maxX<=minX) WITHOUT saving, so a botched cal can never overwrite a good one. Pure code checks — no confStruct field added, no SW_VERSION bump.
// V2.5-Evo - 2026-05-06 - D2: Add updateCompassSnapshot() and snapshot globals (clean heading captured during motor-idle for future RTM heading source)
// V2.5-Evo - 2026-04-25 - P7: Added getCompassHeading() function
#include <Wire.h>
#include <esp_task_wdt.h> // <-- Added to feed the Watchdog

// ============================================================
// Supported magnetometers — detected at boot by I2C address
// ============================================================
// The I2C address IS the part identification, so no config field is needed and
// no confStruct/SW_VERSION change is required: one image drives either module.
//
//   0x0D = QMC5883L  — Beitian BN-880, HGLRC M100 Pro
//   0x2C = QMC5883P  — HGLRC M100-5883
//   0x1E = HMC5883L  — very old BN-880 stock. NOT supported; reported, not driven.
//
// ⚠ These are different silicon, not a revision. The data block starts at a
// DIFFERENT register (L: 0x00, P: 0x01), so pointing the L driver at a P returns
// CHIPID,XL,XH,YL,YH,ZL — every axis shifted one byte. That yields a smoothly
// varying, entirely wrong heading with no error anywhere. On a Follow-Me buggy
// that is a safety-grade failure, which is why the read path branches on the part
// rather than just on the address.
#define QMC5883L_ADDR 0x0D
#define QMC5883P_ADDR 0x2C
#define HMC5883L_ADDR 0x1E

enum CompassChip : uint8_t {
  COMPASS_NONE     = 0,
  COMPASS_QMC5883L = 1,
  COMPASS_QMC5883P = 2
};

int16_t magX = 0, magY = 0, magZ = 0;
bool compass_detected = false;

CompassChip compass_chip     = COMPASS_NONE;  // which part answered at boot
uint8_t     compass_addr     = QMC5883L_ADDR; // its I2C address
uint8_t     compass_data_reg = 0x00;          // first data register: L=0x00, P=0x01

// Human-readable part name for ?i2c / ?magtest / boot log.
const char* compassChipName() {
  switch (compass_chip) {
    case COMPASS_QMC5883L: return "QMC5883L";
    case COMPASS_QMC5883P: return "QMC5883P";
    default:               return "none";
  }
}

// Single-register write. Takes the I2C mutex itself, matching the style below.
static bool compassWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  bool ok = (Wire.endTransmission() == 0);
  xSemaphoreGive(i2cMutex);
  return ok;
}

// Single-register read. Returns false on any bus error; value in *out.
static bool compassReadReg(uint8_t addr, uint8_t reg, uint8_t *out) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { xSemaphoreGive(i2cMutex); return false; }
  bool ok = (Wire.requestFrom((uint8_t)addr, (uint8_t)1, (uint8_t)true) == 1);
  if (ok) *out = Wire.read();
  xSemaphoreGive(i2cMutex);
  return ok;
}

// Does anything ACK at this address?
static bool compassProbe(uint8_t addr) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(addr);
  bool ack = (Wire.endTransmission() == 0);
  xSemaphoreGive(i2cMutex);
  return ack;
}
float         compass_snapshot_heading = -1.0f; // Last "clean" compass heading captured while motor was idle (degrees, 0–360 clockwise from North). -1.0f = no valid snapshot yet.
unsigned long compass_snapshot_ms      = 0;     // millis() timestamp of last snapshot capture. 0 = no snapshot yet.

// We link to your existing save command to automate the process
extern void cmdSave(const String& params);

void initCompass() {
  Wire.setTimeOut(20);

  compass_detected = false;
  compass_chip     = COMPASS_NONE;

  // ---- Detect which part is fitted -------------------------------------
  // The ACK is the primary evidence; the chip-ID register only confirms it.
  // Deliberately that way round: the L's ID value is 0xFF, which is also what an
  // idle-high bus reads, so ID-first would be a weak signature. The P's 0x80 is
  // unambiguous, so 0x2C is probed first.
  if (compassProbe(QMC5883P_ADDR)) {
    uint8_t id = 0;
    if (compassReadReg(QMC5883P_ADDR, 0x00, &id) && id == 0x80) {
      compass_chip = COMPASS_QMC5883P;
    } else {
      // ACKed at 0x2C but the ID did not read back as expected. Trust the ACK —
      // nothing else in this system lives at 0x2C — and say so.
      compass_chip = COMPASS_QMC5883P;
      Serial.printf("Compass: 0x2C ACKed but chip ID read 0x%02X (expected 0x80) — driving as QMC5883P anyway.\n", id);
    }
    compass_addr     = QMC5883P_ADDR;
    compass_data_reg = 0x01;                 // P data block starts at 0x01
  }
  else if (compassProbe(QMC5883L_ADDR)) {
    compass_chip     = COMPASS_QMC5883L;
    compass_addr     = QMC5883L_ADDR;
    compass_data_reg = 0x00;                 // L data block starts at 0x00
  }
  else if (compassProbe(HMC5883L_ADDR)) {
    // Old Honeywell part on very early BN-880 stock. Different register map
    // again; not driven here. Report it clearly rather than failing silently,
    // so the owner knows the module is alive and simply unsupported.
    Serial.println("WARNING: HMC5883L found at 0x1E — NOT supported. Fit a BN-880 (QMC5883L) or M100-5883 (QMC5883P).");
    return;
  }
  else {
    Serial.println("WARNING: no compass found at 0x2C / 0x0D / 0x1E.");
    Serial.println("         Check gps_chip_type is 1 or 3 (a compass-equipped module) and the I2C wiring.");
    return;
  }

  // ---- Configure it ----------------------------------------------------
  if (compass_chip == COMPASS_QMC5883P) {
    // QST QMC5883P datasheet §7.2 continuous-mode example. Matches iNav.
    // NOTE: 0x29 = 0x06 sets the axis signs and is MANDATORY. It appears only in
    // the application examples, not in the register-map table. ArduPilot's driver
    // has this transposed (writes 0x29 into register 0x06, a read-only data
    // register) — do not copy it.
    compassWriteReg(QMC5883P_ADDR, 0x0B, 0x80);  // soft reset
    delay(30);
    compassWriteReg(QMC5883P_ADDR, 0x29, 0x06);  // axis sign — mandatory
    compassWriteReg(QMC5883P_ADDR, 0x0B, 0x08);  // CR2: set/reset on, range 8 G
    compassWriteReg(QMC5883P_ADDR, 0x0A, 0xC7);  // CR1: OSR2/OSR1 max, ODR 50 Hz, MODE continuous (11)
    compass_detected = true;
    Serial.println("QMC5883P compass detected at 0x2C. Init OK (8 G, 50 Hz, continuous).");
  }
  else {
    // QMC5883L — unchanged from the long-standing BN-880 path. 0x15 =
    // OSR 512, range 8 G, ODR 50 Hz, MODE continuous (01).
    compassWriteReg(QMC5883L_ADDR, 0x0B, 0x01);  // SET/RESET period FBR
    compassWriteReg(QMC5883L_ADDR, 0x09, 0x15);  // CR1
    compass_detected = true;
    Serial.println("QMC5883L compass detected at 0x0D. Init OK (8 G, 50 Hz, continuous).");
  }

  // A stored calibration cannot cross a part change: mag_offset_* are raw counts,
  // and the two parts differ in sensitivity at 8 G (L 3000 vs P 3750 LSB/G) and in
  // axis frame. Heading itself is atan2(y,x) so a uniform scale error cancels — the
  // offsets do not. Re-run ?compasscal after swapping modules.
  Serial.println("If you have just changed GPS/compass module, run ?compasscal before trusting FM/RTM heading.");
}

bool readCompassRaw() {
  if (!compass_detected) return false;

  // compass_addr and compass_data_reg were set by initCompass() from the detected
  // part. The data-register base is the difference that matters: QMC5883L starts
  // at 0x00, QMC5883P at 0x01. Reading from the wrong base does not error — it
  // shifts every axis by one byte and returns a plausible, wrong heading.
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(compass_addr);
  Wire.write(compass_data_reg);
  if (Wire.endTransmission(false) != 0) {
    xSemaphoreGive(i2cMutex);
    return false;
  }

  uint8_t bytesReceived = Wire.requestFrom(compass_addr, (uint8_t)6, (uint8_t)true);

  if (bytesReceived == 6) {
    // Both parts: X,Y,Z, LSB first, 16-bit two's complement. Read into locals
    // first so a partial/torn read can never leave magX updated and magY stale.
    uint8_t b[6];
    for (uint8_t i = 0; i < 6; i++) b[i] = Wire.read();
    xSemaphoreGive(i2cMutex);

    magX = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
    magY = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
    magZ = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
    return true;
  }

  xSemaphoreGive(i2cMutex);
  return false;
}

void serPrintCompass() {
  Serial.println("Printing Raw Compass Data. Type 'quit' to exit.");
  
  while (true) {
    esp_task_wdt_reset(); // <-- FEED THE WATCHDOG! Prevent 7-second crash.

    if(checkSerialQuit()) break;

    if (readCompassRaw()) {
      Serial.print("Raw X: "); Serial.print(magX);
      Serial.print("\tRaw Y: "); Serial.print(magY);
      Serial.print("\tRaw Z: "); Serial.println(magZ);
    } else {
      Serial.println("I2C Read Error. Did a wire come loose?");
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

void runCompassCalibration() {
  // V2.5-Evo - 2026-07-21 - Require the compass hardware at entry. Without this, a cal started
  // with no sensor present (loose wire, wrong hardware, BIND pressed by mistake) collects zero
  // samples and — via the min/max init values below — silently bakes a garbage 0/0/1/1 cal over
  // a known-good one. Abort loudly and keep the existing calibration.
  if (!compass_detected) {
    Serial.println("\nERROR: Compass not detected. Calibration aborted (existing cal kept).");
    blinkBind(10);
    return;
  }

  int16_t minX = 32767, maxX = -32768;
  int16_t minY = 32767, maxY = -32768;

  uint32_t startTime = millis();
  uint32_t duration = 45000; // 45 seconds
  uint32_t lastPrintTime = 0;

  Serial.println("\n--- COMPASS CALIBRATION STARTED ---");
  Serial.println(">>> ROTATE BUGGY SLOWLY 360 DEGREES (2 FULL CIRCLES) <<<");
  Serial.println("You have 45 seconds. Type 'quit' to abort.");

  // Clear any stale serial inputs
  while(Serial.available()) Serial.read();

  while (millis() - startTime < duration) {
    esp_task_wdt_reset(); // <-- FEED THE WATCHDOG! Prevent 7-second crash.

    // Allow user to abort if needed
    if (checkSerialQuit()) {
      Serial.println("\nCalibration ABORTED by user.");
      return;
    }

    if (readCompassRaw()) {
      if (magX < minX) minX = magX;
      if (magX > maxX) maxX = magX;
      if (magY < minY) minY = magY;
      if (magY > maxY) maxY = magY;
    }
    
    // Print a countdown every 5 seconds
    if (millis() - lastPrintTime >= 5000) {
      lastPrintTime = millis();
      Serial.printf("Calibrating... %d seconds left\n", (int)((duration - (millis() - startTime)) / 1000));
    }
    
    // Tiny delay to yield to FreeRTOS
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }

  // V2.5-Evo - 2026-07-21 - Abort if NO valid samples were captured. If readCompassRaw() never
  // succeeded during the 45s window, min/max stay at their init values (min=32767, max=-32768),
  // so max<=min. The old avgDeltaX==0 guard below NEVER fired in this case because
  // avgDeltaX=(maxX-minX)/2=-32767.5, not 0 — so a no-sample cal computed offset 0/0 + scale
  // 1.0/1.0 and cmdSave() clobbered the good calibration. Detect the degenerate range HERE and
  // abort BEFORE writing usrConf, so a botched cal can never overwrite a good one.
  if (maxX <= minX || maxY <= minY) {
    Serial.println("\nERROR: No valid compass samples captured. Calibration aborted (existing cal kept).");
    blinkBind(10);
    return;
  }

  // Phase 1: Calculate Hard Iron Offsets (The Center)
  usrConf.mag_offset_x = (maxX + minX) / 2;
  usrConf.mag_offset_y = (maxY + minY) / 2;

  // Phase 2: Calculate Soft Iron Scaling (The Shape)
  float avgDeltaX = (maxX - minX) / 2.0;
  float avgDeltaY = (maxY - minY) / 2.0;
  float avgDelta = (avgDeltaX + avgDeltaY) / 2.0;

  if (avgDeltaX == 0 || avgDeltaY == 0) {
     Serial.println("\nERROR: No valid compass data received. Calibration failed.");
     return;
  }

  usrConf.mag_scale_x = avgDelta / avgDeltaX;
  usrConf.mag_scale_y = avgDelta / avgDeltaY;

  // V2.5-Evo - 2026-07-22 - Clamp mag_scale to the kCfgFields validation range [0.1, 10.0] BEFORE
  // saving. A pathological cal (extreme axis asymmetry, ~19:1) can compute a scale > 10.0, which
  // would then FAIL validateConfig() on the next boot → readConfFromSPIFFS() rejects the blob →
  // full config + pairing wipe. Clamping at the writer keeps the cal output inside the validator's
  // domain so a valid cal can never self-wipe the config. Clamp before the print below so the
  // reported scales match what is actually saved.
  if (usrConf.mag_scale_x < 0.1f)  usrConf.mag_scale_x = 0.1f;
  if (usrConf.mag_scale_x > 10.0f) usrConf.mag_scale_x = 10.0f;
  if (usrConf.mag_scale_y < 0.1f)  usrConf.mag_scale_y = 0.1f;
  if (usrConf.mag_scale_y > 10.0f) usrConf.mag_scale_y = 10.0f;

  Serial.println("\n--- CALIBRATION COMPLETE ---");
  Serial.printf("Saved Center Offsets: X=%d, Y=%d\n", usrConf.mag_offset_x, usrConf.mag_offset_y);
  Serial.printf("Saved Shape Scales:   X=%.2f, Y=%.2f\n", usrConf.mag_scale_x, usrConf.mag_scale_y);
  
  // Automate the save command to SPIFFS
  cmdSave("");
  Serial.println("Success! Calibration permanently saved to hardware.");
}

// V2.5-Evo - 2026-04-25 - P7: Compute calibrated compass heading in degrees.
//
// What it does:
//   Reads raw magnetometer via readCompassRaw(), applies hard-iron offset
//   correction (mag_offset_x/y) and soft-iron scale correction (mag_scale_x/y),
//   then returns the 2D heading angle via atan2f.
//
// Returns:
//   Heading in degrees, 0=North, 90=East, 180=South, 270=West (clockwise).
//   Returns -1.0f if compass is not detected or never calibrated (scale=0).
//
// Note: if heading is consistently wrong by a fixed offset, adjust physical
//   mounting or add a calibration offset parameter in a future revision.
//   If left/right are swapped, negate cal_y below.
float getCompassHeading()
{
  if (!compass_detected) return -1.0f;

  // V2.5-Evo - 2026-07-21 - Reject an UNCALIBRATED compass, not just scale==0.
  // Bug: a never-calibrated (or botched-cal) compass carries the degenerate default
  // offset 0/0 + scale 1/1 — the scale is 1, NOT 0 — so the old scale==0 check passed it
  // and returned atan2(magY*1, magX*1) with the hard-iron offset (0) never subtracted: a
  // biased/garbage heading that FM/RTM then steered on. Treat offset 0/0 + scale ~1/1 as
  // uncalibrated. Keep the scale==0 reject too (means the field was never written at all).
  if (usrConf.mag_scale_x == 0.0f || usrConf.mag_scale_y == 0.0f) return -1.0f;
  if (usrConf.mag_offset_x == 0 && usrConf.mag_offset_y == 0 &&
      fabsf(usrConf.mag_scale_x - 1.0f) < 1e-4f && fabsf(usrConf.mag_scale_y - 1.0f) < 1e-4f) return -1.0f;

  // Return -1 on I2C failure — stale magX/magY from a previous read would give a wrong heading.
  if (!readCompassRaw()) return -1.0f;

  float cal_x = ((float)magX - (float)usrConf.mag_offset_x) * usrConf.mag_scale_x;
  float cal_y = ((float)magY - (float)usrConf.mag_offset_y) * usrConf.mag_scale_y;

  float heading = atan2f(cal_y, cal_x) * (180.0f / M_PI);
  if (heading < 0.0f) heading += 360.0f;

  return heading;
}

// ============================================================
// updateCompassSnapshot - Capture a clean compass heading during motor-idle
// ============================================================
//
// What it does:
//   Attempts to capture a fresh, unbiased compass heading and store it in
//   compass_snapshot_heading / compass_snapshot_ms. Only updates when the
//   motor is effectively idle, because motor current induces a magnetic field
//   that biases the QMC5883L by 100°+ at even moderate throttle. Idle moments
//   give a usable reference heading unaffected by drive-current EMI.
//
// Inputs:
//   Reads thr_received (global volatile, declared extern here to avoid header change).
//   Calls getCompassHeading() for a fresh calibrated heading.
//
// Side effects:
//   Updates compass_snapshot_heading and compass_snapshot_ms when all conditions
//   are met (compass detected, calibrated, motor idle, valid heading returned).
//   No side effects if any condition fails — snapshot is left unchanged.
//
// When called:
//   Called every ~100ms from runRtmLoop() in RTMState.ino (top of loop).
//   Safe to call from any FreeRTOS task context — getCompassHeading() takes
//   a few ms (I2C reads) but does not block indefinitely.
// ============================================================
void updateCompassSnapshot()
{
  // Gate 1: compass hardware must be present
  if (!compass_detected) return;

  // V2.5-Evo - 2026-07-21 - Gate 2: compass must be calibrated. UNCALIBRATED = the degenerate
  // default offset 0/0 + scale 1/1 (scale is 1, NOT 0) — the old scale==0 check missed it and let
  // a biased/garbage heading become the RTM snapshot. Reject scale==0 AND the 0/0 + ~1/1 default.
  if (usrConf.mag_scale_x == 0.0f || usrConf.mag_scale_y == 0.0f) return;
  if (usrConf.mag_offset_x == 0 && usrConf.mag_offset_y == 0 &&
      fabsf(usrConf.mag_scale_x - 1.0f) < 1e-4f && fabsf(usrConf.mag_scale_y - 1.0f) < 1e-4f) return;

  // Gate 3: motor must be idle. thr_received < 25 means the user is not pressing
  // the throttle trigger enough to spin the motor. Threshold matches RTM Gate 1
  // in RTMState.ino so snapshot and RTM arming use a consistent idle definition.
  extern volatile uint8_t thr_received;
  if (thr_received >= 25) return;

  // Take a fresh heading reading. Skip update on I2C failure or uncalibrated result.
  float h = getCompassHeading();
  if (h < 0.0f) return;

  compass_snapshot_heading = h;
  compass_snapshot_ms      = millis();
}