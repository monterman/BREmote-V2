// V2.5-Evo - 2026-08-17 - REPORT THE ERROR THE CARDINAL SNAP LEAVES BEHIND. mag_orientation can only ever hold 0, 90, 180 or 270, so a module glued in at any other angle keeps the difference between where it actually sits and the cardinal it snapped to as a CONSTANT heading error - up to 45 deg - that no calibration can remove. Both numbers were already printed side by side ("measured 251 deg, stored 270 deg") but nothing anywhere said they were meant to match, so a rider saw two numbers and no reason to care. This is not hypothetical: a tester's published run measured 251.25 deg, snapped to 270, and his four post-correction cardinal errors then averaged exactly -18.75 deg - the residual, showing up as a fixed bias he could not calibrate away and could not have diagnosed from the output. Both ?compasscal and ?magalign now compute that residual as a SHORTEST ANGULAR DISTANCE (so 359 deg snapping to 0 is 1 deg out, not 359) and print a warning naming the remaining error and the physical fix. New file-scope constant kMountSquareTolDeg = 9.6 deg = 3x the ~3.2 deg idle noise this file documents twice, so noise alone can never trip it; deliberately NOT raised to cover the 15-20 deg human aim slop kNorthTolDeg allows for, because that band is exactly where the real failure sat. ?magalign's existing 25 deg warning is REPLACED by this one - at 25 deg it would have said nothing about the tester's 18.75 deg. Reporting only: the snap, the stored value, orientation_stored, the FULL/PARTIAL verdict and the 2/3/10 BIND LED patterns are all untouched, and no threshold that already existed (kNorthTolDeg, kMinTurnDeg, kMinIronTurnDeg) moved. No confStruct field added, no SW_VERSION bump, sizeof stays 192.
// V2.5-Evo - 2026-08-17 - ?compasscal and ?magalign are now the rider's way OUT of a heading-disagreement degradation. RTMState.ino's heading_disagree_fault latch stopped being cleared at engagement boundaries (it was forgiveness without evidence — a compass mounted 90 deg out is just as wrong on the next run), so while it stands the whole session runs on GPS course only. The escape routes have to be evidence that the compass was actually FIXED, and there are exactly two here: a FULL ?compasscal, and a completed ?magalign. Both call headingDisagreeClearAfterCal() at the point where the new mounting numbers have been written and saved, which drops the latch and restores hybrid heading immediately — no reboot, and no coasting to re-prove anything. A PARTIAL cal deliberately does NOT clear it: a 300-400 deg run saves the iron calibration but keeps the OLD mag_orientation, which is usually the very thing that caused the disagreement, so promoting it to "fixed" would hand the compass straight back to the steering while it was still wrong. Aborted and failed runs do not clear it either — they write nothing at all. The clear function is a file-scope static in RTMState.ino, forward-declared below in the same way Logger.ino declares headingDisagreeLatched(); Arduino compiles the whole sketch as one translation unit and concatenates Compass.ino ahead of RTMState.ino, so the declaration is what makes the call legal. Both commands run in the loop task (System.ino dispatches them), so the latch keeps its single-writer property. No confStruct field added, no SW_VERSION bump, sizeof stays 192.
// V2.5-Evo - 2026-08-16 - Two follow-ups. (1) MID-COMMAND SAFETY: ?printcompass, ?compasscal and ?magalign now abort if Return-to-Me or Follow-Me becomes engaged WHILE they are running, not only when one is already engaged at dispatch (see rxAbortIfEngaged() in System.ino). All three abort points sit BEFORE the first usrConf write, so an abandoned run leaves no half-written calibration and the existing one untouched. (2) PARTIAL CREDIT IS NOW REPORTED AS PARTIAL: a ?compasscal run that turned 300-400 deg saves the iron calibration but does NOT re-measure mounting orientation or handedness, and a run that turns far enough but finishes off north saves iron calibration and handedness but still no orientation. Both used to print the identical "--- CALIBRATION COMPLETE --- / Success!" as a full run and blink the identical 2-flash BIND pattern. A rider who had just re-mounted the module and walked a sloppy circle was therefore told it worked while mag_orientation still held the OLD mounting angle - a heading wrong by exactly the mounting delta. runCompassCalibration() now records its outcome in compass_cal_result (FAILED / PARTIAL / FULL) for the BIND LED in System.ino, and prints an explicit PARTIAL report naming what was and was not updated. NEITHER THRESHOLD MOVED - 300 deg still gates the iron save and 400 deg still gates orientation and handedness, exactly as adjudicated; only the reporting changed. No confStruct field added, no SW_VERSION bump.
// V2.5-Evo - 2026-08-16 - Four ?compasscal / init hardening fixes: (1) an under-rotated re-run no longer silently DESTROYS a stored mirror correction — the previous sign of mag_scale_y is preserved when handedness is not re-derived, and the message now says so; (2) ?compasscal aborts without saving anything unless the buggy was actually turned (a 45 s hold used to bake offsets from a noise blob over a good cal); (3) the QMC5883P init writes — including the MANDATORY axis-sign write — are now checked, and a failed init leaves the compass reported as NOT detected instead of producing systematically wrong headings; (4) the uncalibrated-reject test is now identical in getCompassHeading(), updateCompassSnapshot() and runMagAlign() — magnitude-compared, so a legitimately MIRRORED module (negative mag_scale_y) is never mistaken for uncalibrated. Code checks only — no confStruct field added, no SW_VERSION bump.
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

// V2.5-Evo - 2026-08-16 - the shared "has RTM/FM engaged while this command was running?" test.
// Defined in System.ino beside rxRefuseIfEngaged(), which asks the same question at dispatch
// time. Declared here explicitly rather than leaning on the sketch's generated prototypes, so
// the dependency between these two files is written down where a reader will see it.
extern bool rxAbortIfEngaged(const char *what);

// V2.5-Evo - 2026-08-17 - clears the heading-disagreement latch in RTMState.ino after the rider has
// physically re-measured the compass mounting. Defined there, next to the flag it owns, so this
// file never touches that state directly.
// NOT 'extern': it is a file-scope 'static' in RTMState.ino (internal linkage), so the declaration
// that names it here has to be 'static' too. That is enough, because Arduino compiles every .ino in
// the sketch as ONE translation unit and concatenates this file ahead of RTMState.ino — the same
// forward-declaration pattern Logger.ino uses for headingDisagreeLatched().
// CALL IT ONLY FROM A RUN THAT ACTUALLY RE-MEASURED THE MOUNTING: CAL_FULL, or a completed
// ?magalign. A PARTIAL cal keeps the old mag_orientation and must leave the latch alone.
static void headingDisagreeClearAfterCal(const char *what);

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
    //
    // V2.5-Evo - 2026-08-16 - Check every write. These four writes used to be fire-and-forget.
    // The bug: if the axis-sign write (0x29 = 0x06) does not land, the part reports its axes in a
    // different frame from the one the stored calibration was measured in — so EVERY heading is
    // systematically wrong, with nothing anywhere to detect it. There is no readback and no error;
    // the compass simply lies, and Follow-Me steers on the lie. The fix is to notice at boot: if
    // any init write is not acknowledged, the compass is reported as NOT detected and stays
    // disabled. Failing loudly at boot costs the rider a look at the wiring; failing silently
    // costs them a buggy that steers to the wrong point of the compass.
    bool init_ok = true;
    init_ok &= compassWriteReg(QMC5883P_ADDR, 0x0B, 0x80);  // soft reset
    delay(30);
    init_ok &= compassWriteReg(QMC5883P_ADDR, 0x29, 0x06);  // axis sign — mandatory
    init_ok &= compassWriteReg(QMC5883P_ADDR, 0x0B, 0x08);  // CR2: set/reset on, range 8 G
    init_ok &= compassWriteReg(QMC5883P_ADDR, 0x0A, 0xC7);  // CR1: OSR2/OSR1 max, ODR 50 Hz, MODE continuous (11)

    if (!init_ok) {
      // compass_detected is already false at this point and is deliberately left that way.
      // compass_chip is cleared too so ?i2c / ?magtest do not advertise a working part.
      compass_chip = COMPASS_NONE;
      Serial.println("ERROR: QMC5883P ACKed at 0x2C but one or more init writes FAILED.");
      Serial.println("       Compass DISABLED - a half-configured QMC5883P returns readings that");
      Serial.println("       look fine and are systematically wrong (the 0x29 axis-sign write is");
      Serial.println("       mandatory). Check I2C wiring, pull-ups and 3.3V, then reboot.");
      return;
    }

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

    // V2.5-Evo - 2026-08-16 - also stop if RTM/FM engages mid-stream. Read-only command, so the
    // exit needs no tidying up: the i2cMutex is taken and given inside readCompassRaw().
    if(checkSerialQuit() || rxAbortIfEngaged("?printcompass")) break;

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

// ============================================================
// Compass calibration OUTCOME - runtime state, deliberately not configuration
// ============================================================
//
// What it is:
//   What the LAST ?compasscal run actually achieved. Written by runCompassCalibration() on every
//   path through it; read by the runtime BIND-button handler in System.ino to choose the LED
//   pattern it blinks when the run finishes.
//
// Why it exists:
//   A calibration has three genuinely different endings and, until now, only two of them could
//   be told apart - and not reliably. The middle one is the dangerous one: a run that turned far
//   enough to sweep the field (>= 300 deg) but not far enough to close a revolution (< 400 deg)
//   saves a NEW iron calibration while leaving the OLD mounting orientation in place. If the
//   module has just been re-mounted, those two now describe different physical mountings and
//   every heading is wrong by the difference. It printed "Success!" and blinked the full-success
//   pattern, so nothing anywhere told the rider. Naming the outcome is what fixes that.
//
// It is a plain global on purpose: it describes the run that has just happened, not a setting.
// It must not survive a reboot and it must not cost a confStruct field - adding one would change
// sizeof(confStruct) and force a SW_VERSION bump, which wipes the RX SPIFFS config, the pairing
// and this very calibration on the next boot.
enum CompassCalResult : uint8_t {
  CAL_FAILED  = 0,   // nothing written, nothing saved - the previous calibration is untouched
  CAL_PARTIAL = 1,   // iron calibration saved, but mounting ORIENTATION was NOT re-measured
  CAL_FULL    = 2    // iron calibration, handedness and mounting orientation all updated + saved
};

CompassCalResult compass_cal_result = CAL_FAILED;

// ============================================================
// kMountSquareTolDeg - how far off a cardinal a mounting angle may sit before it is worth saying so
// ============================================================
// V2.5-Evo - 2026-08-17 - mag_orientation is a snapped value: 0, 90, 180 or 270 and nothing in
// between. Whatever is thrown away by that snap does not average out and does not wash out - it
// stays as a fixed offset on every heading the compass ever reports. This is the line above which
// the rider is told about it.
//
// Why 9.6 and not a round 10: it is 3x the ~3.2 deg idle noise spread this file documents in two
// places (the calibration tolerance block below, and the ?magalign averaging note). Three
// standard-ish deviations means sensor noise ALONE can effectively never push a genuinely square
// mount over this line, which is the whole requirement - a warning that cries wolf on good runs
// gets ignored on the bad one. It is deliberately NOT raised to also swallow the 15-20 deg of
// human aim slop that kNorthTolDeg allows for, because that is precisely the band where the real
// failure lives (a measured 251.25 deg snapping to 270 = 18.75 deg of permanent error). Hiding
// everything below 20 deg would hide the exact case this warning exists for. The two causes are
// separated in words instead: aim error moves between runs, a mounting angle repeats.
//
// File scope on purpose, not beside kMinTurnDeg / kNorthTolDeg inside runCompassCalibration():
// ?magalign snaps the same way and needs the same number, and one threshold in one place is what
// keeps the two commands from drifting apart.
//
// Units: degrees. Range of the residual it is compared against is 0-45 by construction.
static const float kMountSquareTolDeg = 9.6f;

void runCompassCalibration() {
  // V2.5-Evo - 2026-08-16 - assume nothing was achieved until the epilogue proves otherwise.
  // Setting it FAILED once, here, means every early return below - no compass, no samples, barely
  // turned, no valid data, user typed 'quit', or RTM/FM engaged mid-run - reports the truth
  // without each of those paths having to remember to say so. Only the epilogue promotes it.
  compass_cal_result = CAL_FAILED;

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


  // V2.5-Evo - 2026-08-16 - North-to-north calibration. One run, three results:

  //   1. hard/soft iron cal  - as before, from the min/max sweep

  //   2. mounting HANDEDNESS - from the SIGN of accumulated rotation. The rider is told to

  //                            turn clockwise, so a mirrored frame counts backwards.

  //   3. mounting ROTATION   - from the FIRST sample, taken while pointing north.

  //

  // The first sample is kept RAW and converted to a heading only AFTER the new offsets and

  // scales exist, so orientation comes from calibrated numbers, not the biased pre-cal frame.

  bool    have_first = false;

  int16_t firstX = 0, firstY = 0, lastX = 0, lastY = 0;

  float   prev_raw_hdg = 0.0f;

  bool    have_prev    = false;

  float   rot_accum    = 0.0f;   // signed degrees turned; +ve = heading rising


  uint32_t startTime = millis();
  uint32_t duration = 45000; // 45 seconds
  uint32_t lastPrintTime = 0;

  Serial.println("\n--- COMPASS CALIBRATION STARTED ---");
  Serial.println(">>> POINT THE FRONT OF THE BUGGY AT NORTH BEFORE YOU START <<<");

  Serial.println(">>> Then rotate it SLOWLY CLOCKWISE, TWO FULL CIRCLES        <<<");

  Serial.println(">>> and FINISH POINTING NORTH AGAIN.                        <<<");

  Serial.println("Clockwise matters: the turn direction is how mounting handedness");

  Serial.println("is detected. Ending on north is how the result is checked.");

  Serial.println("You have 45 seconds. Type \'quit\' to abort.");


  // Clear any stale serial inputs
  while(Serial.available()) Serial.read();

  while (millis() - startTime < duration) {
    esp_task_wdt_reset(); // <-- FEED THE WATCHDOG! Prevent 7-second crash.

    // Allow user to abort if needed
    if (checkSerialQuit()) {
      Serial.println("\nCalibration ABORTED by user.");
      return;
    }

    // V2.5-Evo - 2026-08-16 - abort if Return-to-Me or Follow-Me engages while the 45 s collection
    // is running. ?compasscal is refused at dispatch when the buggy is ALREADY engaged, but it can
    // legitimately start on an idle bench and then be overtaken by an arm from the remote - and a
    // 45 s freeze of runRtmLoop()/runFmLoop() means 45 s in which no safety gate is evaluated at
    // all. This is the safest possible place to give up: it is before the FIRST usrConf write, so
    // the samples gathered so far are simply discarded and the stored calibration is left exactly
    // as it was. compass_cal_result stays CAL_FAILED, so the BIND LED cannot claim a success.
    if (rxAbortIfEngaged("?compasscal")) {
      Serial.println("Calibration abandoned. Nothing was saved; the existing calibration is kept.");
      return;
    }

    if (readCompassRaw()) {
      if (magX < minX) minX = magX;
      if (magX > maxX) maxX = magX;
      if (magY < minY) minY = magY;
      if (magY > maxY) maxY = magY;


      if (!have_first) { firstX = magX; firstY = magY; have_first = true; }

      lastX = magX; lastY = magY;



      // Signed rotation from the RAW frame. A hard-iron bias makes a turn non-uniform but

      // not non-monotonic, so the SIGN of the total stays trustworthy.

      float raw_hdg = atan2f((float)magY, (float)magX) * (180.0f / M_PI);

      if (raw_hdg < 0.0f) raw_hdg += 360.0f;

      if (have_prev) {

        float d = raw_hdg - prev_raw_hdg;

        while (d > 180.0f)  d -= 360.0f;      // shortest-path wrap

        while (d < -180.0f) d += 360.0f;

        rot_accum += d;

      }

      prev_raw_hdg = raw_hdg;

      have_prev    = true;

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

  // V2.5-Evo - 2026-08-16 - Abort if the buggy was never actually TURNED. The bug: the min/max
  // check above only catches a run with NO samples. A run where the buggy sat still for the whole
  // 45 s collects thousands of samples, so min/max pass — but they bound a small noise blob, not
  // the earth's field. That gives offsets at the centre of the noise and scales near 1.0: numbers
  // that LOOK calibrated (the offsets are non-zero, so every downstream "is it calibrated?" check
  // waves them through) while heading becomes atan2(noise) — random. Saved over a good cal, this
  // is silent and dangerous, so a real sweep is now required BEFORE anything is written.
  //
  // The bar is deliberately low — one circle with slop, not the two that were asked for — in
  // keeping with the tolerance philosophy below: a rejected run only costs the rider another walk
  // round the buggy, while a bad stored cal is invisible and steers Follow-Me wrong. kMinTurnDeg
  // further down is the higher bar, and it gates handedness and orientation only.
  const float kMinIronTurnDeg = 300.0f;  // ~one full circle; less than this cannot sweep the field ellipse
  if (fabsf(rot_accum) < kMinIronTurnDeg) {
    Serial.printf("\nERROR: only %.0f deg of rotation seen - the buggy barely turned.\n", fabsf(rot_accum));
    Serial.println("       Calibration ABORTED. Nothing saved, existing calibration kept.");
    Serial.println("       The 45 s window has to be spent WALKING THE BUGGY ROUND: two full");
    Serial.println("       circles clockwise, starting and finishing with the nose on north.");
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

  // V2.5-Evo - 2026-08-16 - Remember the handedness that is about to be overwritten. The two
  // divisions below are always POSITIVE, so they wipe out the negative mag_scale_y that encodes a
  // mirrored module. That is only correct if this run actually re-derives handedness; if it does
  // not, the previous sign has to be put back (see the kMinTurnDeg block further down).
  const bool prev_frame_mirrored = (usrConf.mag_scale_y < 0.0f);

  usrConf.mag_scale_x = avgDelta / avgDeltaX;
  usrConf.mag_scale_y = avgDelta / avgDeltaY;

  // V2.5-Evo - 2026-07-22 - Clamp mag_scale to the kCfgFields validation range [0.1, 10.0] BEFORE
  // saving. A pathological cal (extreme axis asymmetry, ~19:1) can compute a scale > 10.0, which
  // would then FAIL validateConfig() on the next boot → readConfFromSPIFFS() rejects the blob →
  // full config + pairing wipe. Clamping at the writer keeps the cal output inside the validator's
  // domain so a valid cal can never self-wipe the config. Clamp before the print below so the
  // reported scales match what is actually saved.
  // V2.5-Evo - 2026-08-16 - Clamp MAGNITUDE, preserve SIGN. A negative mag_scale_y now carries

  // meaning - it encodes a mirrored sensor frame - so the old positive-only clamp would have

  // silently discarded the handedness correction. Validator range widened to match.

  auto clampMag = [](float v) {

    float s = (v < 0.0f) ? -1.0f : 1.0f;

    float m = fabsf(v);

    if (m < 0.1f)  m = 0.1f;

    if (m > 10.0f) m = 10.0f;

    return s * m;

  };

  usrConf.mag_scale_x = clampMag(usrConf.mag_scale_x);

  usrConf.mag_scale_y = clampMag(usrConf.mag_scale_y);




  // ============================================================

  // V2.5-Evo - 2026-08-16 - Derive HANDEDNESS and ROTATION from the same run.

  //

  // Tolerances are deliberately forgiving. A rejected calibration costs the rider a physical

  // re-run on their feet, so the bar is 'clearly wrong', not 'imperfect'. Idle noise on this

  // hardware is ~3.2 deg and a human aiming a buggy at north by eye is worth 15-20 deg, so

  // anything inside +/-40 deg is accepted. A bad run keeps the PREVIOUS orientation rather

  // than storing a guess - the hard/soft-iron cal is still saved either way.

  // ============================================================

  const float kMinTurnDeg  = 400.0f;  // want ~720; accept one sloppy turn plus margin

  const float kNorthTolDeg = 40.0f;   // start-vs-end closure, and aim slop at north



  bool orient_ok = true;

  // V2.5-Evo - 2026-08-16 - true ONLY where usrConf.mag_orientation is genuinely written. That
  // single fact is what separates a FULL calibration from a PARTIAL one, and there are two
  // different ways to miss it - too little rotation (orient_ok goes false above) and a turn that
  // does not close back on north (below) - so the epilogue reads THIS rather than trying to
  // re-derive the answer from the thresholds.
  bool orientation_stored = false;



  if (fabsf(rot_accum) < kMinTurnDeg) {

    // V2.5-Evo - 2026-08-16 - PUT THE OLD HANDEDNESS BACK. The bug this fixes: the scales above
    // are always recomputed positive, and the mirror sign was only ever re-applied inside the
    // validated branch below. So a rider with a MIRRORED module who did a sloppy re-run had the
    // mirror correction silently destroyed while being told handedness was "NOT updated" — it had
    // been updated, to the wrong value. The compass then read mirrored and hybrid mode steered on
    // it at low speed. This run did not measure handedness, so it must not change it.
    if (prev_frame_mirrored) usrConf.mag_scale_y = -usrConf.mag_scale_y;

    Serial.printf("\nNOTE: only %.0f deg of rotation seen (wanted ~720).\n", fabsf(rot_accum));

    Serial.println("      Iron calibration saved. Orientation NOT updated (previous kept).");

    Serial.printf("      Handedness NOT updated either - previous setting kept (%s).\n",

                  prev_frame_mirrored ? "MIRRORED" : "normal");

    Serial.println("      Re-run and turn through two full circles to set them.");

    orient_ok = false;

  }



  if (orient_ok) {

    // Turning CLOCKWISE, a correctly-handed sensor makes the heading RISE. If it fell, the

    // frame is mirrored - stored as a NEGATIVE mag_scale_y, because negating cal_y IS the

    // mirror correction and that field already exists. No extra storage needed.

    if (rot_accum < 0.0f) {

      usrConf.mag_scale_y = -usrConf.mag_scale_y;

      Serial.println("\nCompass frame is MIRRORED - corrected (mag_scale_y stored negative).");

    }



    // The first sample was taken pointing north. Convert it NOW, with the offsets and scales

    // just computed, so the answer comes from calibrated numbers.

    float fx = ((float)firstX - (float)usrConf.mag_offset_x) * usrConf.mag_scale_x;

    float fy = ((float)firstY - (float)usrConf.mag_offset_y) * usrConf.mag_scale_y;

    float h_start = atan2f(fy, fx) * (180.0f / M_PI);

    if (h_start < 0.0f) h_start += 360.0f;



    float lx = ((float)lastX - (float)usrConf.mag_offset_x) * usrConf.mag_scale_x;

    float ly = ((float)lastY - (float)usrConf.mag_offset_y) * usrConf.mag_scale_y;

    float h_end = atan2f(ly, lx) * (180.0f / M_PI);

    if (h_end < 0.0f) h_end += 360.0f;



    // Did the rider finish where they started? This is what stops a sloppy run baking in a

    // wrong orientation - the failure ArduPilot warns about, where a calibration 'appears to

    // succeed while leaving the compass in a very bad state'.

    float closure = h_end - h_start;

    while (closure > 180.0f)  closure -= 360.0f;

    while (closure < -180.0f) closure += 360.0f;



    if (fabsf(closure) > kNorthTolDeg) {

      Serial.printf("\nNOTE: finished %.0f deg from where it started.\n", closure);

      Serial.println("      Iron calibration saved. Orientation NOT updated (previous kept).");

      Serial.println("      Re-run, finishing with the nose back on north.");

    } else {

      // Pointing north when h_start was taken, so h_start IS the mounting rotation. Snap to

      // the nearest cardinal: a 3.2 deg noise floor cannot justify finer resolution.

      int snapped = ((int)((h_start + 45.0f) / 90.0f)) * 90;

      if (snapped >= 360) snapped -= 360;

      usrConf.mag_orientation = (uint16_t)snapped;

      orientation_stored = true;   // the one place a run earns FULL rather than PARTIAL

      Serial.printf("\nMounting orientation: measured %.0f deg, stored %d deg.\n", h_start, snapped);

      // V2.5-Evo - 2026-08-17 - Say out loud what the snap just threw away.
      //
      // The two numbers on the line above are supposed to be the same number. When they are not,
      // the difference is not rounding - it is a real, permanent heading error, because only the
      // snapped value gets stored and getCompassHeading() subtracts only that. Nothing anywhere
      // told the rider to compare them, so a module sitting 18.75 deg off a cardinal read 18.75
      // deg wrong forever and looked like a perfect calibration.
      //
      // SHORTEST ANGULAR DISTANCE, not a raw subtraction: h_start of 359 deg snaps to 0, and the
      // module is 1 deg off square, not 359. Same wrap normalisation the closure check above uses.
      float mount_residual = h_start - (float)snapped;
      while (mount_residual > 180.0f)  mount_residual -= 360.0f;
      while (mount_residual < -180.0f) mount_residual += 360.0f;

      if (fabsf(mount_residual) > kMountSquareTolDeg) {
        Serial.println("\nWARNING: the compass module is NOT mounted square to the buggy.");
        Serial.printf("         It sits %.1f deg from the nearest cardinal, and only 0, 90, 180\n",
                      fabsf(mount_residual));
        Serial.println("         or 270 deg can be stored - so that difference was discarded.");
        Serial.printf("         About %.1f deg of heading error will REMAIN no matter how well the\n",
                      fabsf(mount_residual));
        Serial.println("         compass is calibrated. Every heading it reports is wrong by that");
        Serial.println("         much, all the time, and Follow-Me steers wrong by that much too.");
        Serial.println("         FIX: re-mount the module SQUARE - its forward axis lined up with");
        Serial.println("         the nose of the buggy, or turned exactly 90, 180 or 270 deg from");
        Serial.println("         it - then run ?compasscal again.");
        Serial.println("         (Aim counts here too: if the nose was not truly on north when this");
        Serial.println("         run started, that error is included in the figure above. Aim error");
        Serial.println("         changes between runs, a mounting angle does not - so run it twice");
        Serial.println("         to tell the two apart.)");
      }

    }

  }

  // ============================================================
  // V2.5-Evo - 2026-08-16 - REPORT PARTIAL CREDIT AS PARTIAL.
  //
  // The bug: a run that saved the iron calibration but did NOT re-measure the mounting
  // orientation printed the identical "--- CALIBRATION COMPLETE --- / Success! Calibration
  // permanently saved to hardware." that a full run prints, and blinked the identical 2-flash
  // BIND pattern. Two different runs reach here that way: one that turned 300-400 deg (enough to
  // sweep the field, not enough to trust closure - orientation AND handedness both left alone),
  // and one that turned far enough but finished off north (handedness measured, orientation
  // still left alone).
  //
  // Why that matters: the rider who most needs to know is the one who has just RE-MOUNTED the
  // module. They walk a sloppy circle, are told "Success!", and walk away with an iron
  // calibration that matches the NEW mounting and a mag_orientation that still describes the OLD
  // one. Every heading is then wrong by exactly the mounting delta, and Follow-Me veers by that
  // same amount at close range - the failure this whole batch of work exists to remove.
  //
  // What did NOT change: neither threshold moved, and nothing about WHAT gets saved moved
  // either. A 300-400 deg run still saves its iron calibration, because a 300 deg arc can miss
  // at most one field extreme by <= 30 deg (about 4 deg of worst-case heading error), while
  // closure genuinely needs a completed revolution. Only the words and the LED changed.
  // ============================================================
  const bool cal_full = orientation_stored;

  Serial.println(cal_full ? "\n--- CALIBRATION COMPLETE ---"
                          : "\n--- CALIBRATION PARTIAL ---");
  Serial.printf("Saved Center Offsets: X=%d, Y=%d\n", usrConf.mag_offset_x, usrConf.mag_offset_y);
  Serial.printf("Saved Shape Scales:   X=%.2f, Y=%.2f\n", usrConf.mag_scale_x, usrConf.mag_scale_y);
  Serial.printf("Mounting Orientation: %u deg%s\n", usrConf.mag_orientation,

                usrConf.mag_scale_y < 0.0f ? "  (frame MIRRORED)" : "");

  if (!cal_full) {
    // Name precisely what this run did and did not change, so "partial" is not left as a mood.
    Serial.println("\nUPDATED    : iron calibration - the hard-iron offsets and soft-iron scales above.");
    Serial.printf("NOT UPDATED: mounting ORIENTATION - still %u deg, carried over from an earlier run.\n",
                  usrConf.mag_orientation);
    if (orient_ok) {
      Serial.println("UPDATED    : mounting HANDEDNESS - the turn direction was measured this run.");
    } else {
      Serial.println("NOT UPDATED: mounting HANDEDNESS - the previous setting was kept.");
    }
  }

  // Automate the save command to SPIFFS
  cmdSave("");

  if (cal_full) {
    Serial.println("Success! Calibration permanently saved to hardware.");
    compass_cal_result = CAL_FULL;

    // V2.5-Evo - 2026-08-17 - A FULL run is the evidence that clears a heading-disagreement
    // degradation. Iron calibration, handedness AND mounting orientation have all just been
    // re-measured and saved, so the compass this session had been refusing is not the same compass
    // any more — the numbers the verdict was formed against no longer exist. Clearing it here is
    // what lets a rider fix a mis-mounted module on the beach and get hybrid heading back in the
    // same session, with no reboot. It is placed AFTER cmdSave() above, so nothing is forgiven
    // until the new calibration is actually on the flash.
    // ONLY IN THIS BRANCH: the PARTIAL branch below keeps the OLD mag_orientation, which is the
    // usual cause of a 45+ deg disagreement, so it must not clear anything.
    headingDisagreeClearAfterCal("?compasscal");
  } else {
    Serial.println("PARTIAL calibration saved to hardware. This is NOT a full success.");
    Serial.println("IF YOU HAVE JUST RE-MOUNTED OR MOVED THE COMPASS MODULE, RUN ?compasscal AGAIN:");
    Serial.println("  the orientation above belongs to the module's OLD position, while the iron");
    Serial.println("  calibration just saved belongs to its NEW one. Until orientation is measured");
    Serial.println("  again, every heading is wrong by the difference between the two - and");
    Serial.println("  Follow-Me will veer by that same amount at close range.");
    Serial.println("  Start with the nose on north, turn TWO FULL CIRCLES CLOCKWISE, finish on north.");
    Serial.println("  (?magalign sets orientation on its own if the iron calibration is already good.)");
    compass_cal_result = CAL_PARTIAL;
  }
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
  // V2.5-Evo - 2026-08-16 - Compare the MAGNITUDE of the scales, so this test matches the one in
  // updateCompassSnapshot() and runMagAlign() exactly. It used to differ: only runMagAlign() took
  // fabsf(), so scale ±1 with offsets 0/0 was judged three different ways in three places. What
  // this test is for is the never-calibrated default (offsets 0/0 + scale 1/1) — the SIGN of
  // mag_scale_y is not part of that question, because a negative mag_scale_y is a legitimate,
  // deliberately stored value: it is how a MIRRORED module is corrected. A mirrored module that
  // HAS been calibrated has real non-zero offsets, so it passes this test and keeps working.
  if (usrConf.mag_scale_x == 0.0f || usrConf.mag_scale_y == 0.0f) return -1.0f;
  if (usrConf.mag_offset_x == 0 && usrConf.mag_offset_y == 0 &&
      fabsf(fabsf(usrConf.mag_scale_x) - 1.0f) < 1e-4f &&
      fabsf(fabsf(usrConf.mag_scale_y) - 1.0f) < 1e-4f) return -1.0f;

  // Return -1 on I2C failure — stale magX/magY from a previous read would give a wrong heading.
  if (!readCompassRaw()) return -1.0f;

  float cal_x = ((float)magX - (float)usrConf.mag_offset_x) * usrConf.mag_scale_x;
  float cal_y = ((float)magY - (float)usrConf.mag_offset_y) * usrConf.mag_scale_y;

  float heading = atan2f(cal_y, cal_x) * (180.0f / M_PI);
  if (heading < 0.0f) heading += 360.0f;


  // V2.5-Evo - 2026-08-16 - Apply the stored mounting rotation. The module can be glued in any

  // of four orientations; mag_orientation records which, measured by ?compasscal starting and

  // ending on north. A MIRRORED frame is NOT handled here - a negative mag_scale_y already

  // flipped cal_y before the atan2 above.

  if (usrConf.mag_orientation) {

    heading -= (float)usrConf.mag_orientation;

    if (heading < 0.0f)    heading += 360.0f;

    if (heading >= 360.0f) heading -= 360.0f;

  }


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
  // V2.5-Evo - 2026-08-16 - Magnitude-compared, identical to getCompassHeading() and runMagAlign().
  // A negative mag_scale_y is the stored MIRROR correction, not evidence of a missing calibration,
  // so the sign must not enter this test — see the longer note in getCompassHeading().
  if (usrConf.mag_scale_x == 0.0f || usrConf.mag_scale_y == 0.0f) return;
  if (usrConf.mag_offset_x == 0 && usrConf.mag_offset_y == 0 &&
      fabsf(fabsf(usrConf.mag_scale_x) - 1.0f) < 1e-4f &&
      fabsf(fabsf(usrConf.mag_scale_y) - 1.0f) < 1e-4f) return;

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
// ============================================================
// runMagAlign - set the compass mounting orientation on its own
// ============================================================
// V2.5-Evo - 2026-08-16 - ?magalign. The same orientation ?compasscal derives, but as a
// standalone step so it can be re-checked or corrected WITHOUT redoing the two-circle iron
// calibration - which is the slow, physical part.
//
// Point the nose of the buggy at magnetic north and run it. The buggy IS at heading 0, so
// whatever the compass reports IS the mounting rotation. Snapped to the nearest cardinal.
//
// Requires an existing iron calibration: mag_offset/mag_scale must already be real, because
// the reading is taken THROUGH them. Running this on an uncalibrated compass would measure
// the hard-iron bias and call it a mounting angle.
//
// This CANNOT detect a mirrored frame - a single heading tells you where zero is, not which
// way the numbers run. Mirroring is set by ?compasscal, from the direction of the turn.
void runMagAlign() {
  if (!compass_detected) {
    Serial.println("\nERROR: no compass detected. Nothing to align.");
    blinkBind(10);
    return;
  }

  // Same uncalibrated-default reject that getCompassHeading() uses.
  if (usrConf.mag_scale_x == 0.0f || usrConf.mag_scale_y == 0.0f ||
      (usrConf.mag_offset_x == 0 && usrConf.mag_offset_y == 0 &&
       fabsf(fabsf(usrConf.mag_scale_x) - 1.0f) < 1e-4f &&
       fabsf(fabsf(usrConf.mag_scale_y) - 1.0f) < 1e-4f)) {
    Serial.println("\nERROR: compass is not calibrated yet.");
    Serial.println("       Run ?compasscal first - it sets the iron calibration AND the");
    Serial.println("       orientation in one go. Use ?magalign only to re-check afterwards.");
    blinkBind(10);
    return;
  }

  Serial.println("\n--- COMPASS ORIENTATION (?magalign) ---");
  Serial.println(">>> POINT THE FRONT OF THE BUGGY AT MAGNETIC NORTH <<<");
  Serial.println("Hold it steady. Sampling for 5 seconds, starting now.");
  Serial.println("Motor OFF - motor current swamps the compass entirely.");

  while (Serial.available()) Serial.read();

  // Average over 5 s. Idle noise here is ~3.2 deg spread, so averaging costs nothing and
  // removes the odd outlier.
  float sum_sin = 0.0f, sum_cos = 0.0f;
  int   n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) {
    esp_task_wdt_reset();

    // V2.5-Evo - 2026-08-16 - abort if RTM/FM engages during the 5 s average. Like ?compasscal,
    // this can legitimately start on an idle bench and be overtaken by an arm from the remote.
    // The abort is before usrConf.mag_orientation is touched, so the stored orientation is left
    // exactly as it was and the averaged samples are simply thrown away.
    if (rxAbortIfEngaged("?magalign")) {
      Serial.println("Alignment abandoned. Mounting orientation unchanged, nothing saved.");
      return;
    }

    if (readCompassRaw()) {
      float cx = ((float)magX - (float)usrConf.mag_offset_x) * usrConf.mag_scale_x;
      float cy = ((float)magY - (float)usrConf.mag_offset_y) * usrConf.mag_scale_y;
      float h  = atan2f(cy, cx);
      sum_sin += sinf(h);   // circular mean - a plain average breaks across the 0/360 wrap
      sum_cos += cosf(h);
      n++;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  if (n < 20) {
    Serial.printf("\nERROR: only %d samples read. Orientation unchanged.\n", n);
    blinkBind(10);
    return;
  }

  float measured = atan2f(sum_sin / n, sum_cos / n) * (180.0f / M_PI);
  if (measured < 0.0f) measured += 360.0f;

  int snapped = ((int)((measured + 45.0f) / 90.0f)) * 90;
  if (snapped >= 360) snapped -= 360;

  // How far off the nearest cardinal was the reading? Large means the buggy was not
  // actually pointing north, or the module is mounted at an odd angle - either way the
  // rider should know rather than get a silent snap.
  // The two while() loops are what make this a SHORTEST ANGULAR DISTANCE rather than a raw
  // subtraction: a reading of 359 deg snaps to 0, and 359 - 0 must be read as 1 deg off square,
  // not 359. Same normalisation runCompassCalibration() uses for its closure and mount checks.
  float residual = measured - (float)snapped;
  while (residual > 180.0f)  residual -= 360.0f;
  while (residual < -180.0f) residual += 360.0f;

  uint16_t previous = usrConf.mag_orientation;
  usrConf.mag_orientation = (uint16_t)snapped;

  Serial.printf("\nMeasured heading while pointing north: %.1f deg (%d samples)\n", measured, n);
  Serial.printf("Mounting orientation stored: %u deg (was %u)\n", usrConf.mag_orientation, previous);
  // V2.5-Evo - 2026-08-17 - Same warning ?compasscal now prints, on the same threshold, because
  // this command performs the identical snap and leaves the identical permanent error behind.
  //
  // What changed: the trigger was a bare 25.0f and the text blamed the rider's aim. At 25 deg it
  // said NOTHING about a module sitting 18.75 deg off a cardinal - the real, measured case this
  // work came from - and even when it did fire, "mounted at an odd angle" never told the rider
  // that the angle costs them a fixed heading error forever. Threshold is now the shared
  // kMountSquareTolDeg (3x the ~3.2 deg idle noise averaged out just above) and the text names
  // the consequence and the physical fix. Aim is still offered as the alternative cause, because
  // here it genuinely is one: this reading is taken in a single 5 s hold pointing north.
  if (fabsf(residual) > kMountSquareTolDeg) {
    Serial.println("\nWARNING: the compass module is NOT mounted square to the buggy.");
    Serial.printf("         The reading sits %.1f deg from the nearest cardinal, and only 0, 90,\n",
                  fabsf(residual));
    Serial.println("         180 or 270 deg can be stored - so that difference was discarded.");
    Serial.printf("         About %.1f deg of heading error will REMAIN no matter how well the\n",
                  fabsf(residual));
    Serial.println("         compass is calibrated. Every heading it reports is wrong by that");
    Serial.println("         much, all the time, and Follow-Me steers wrong by that much too.");
    Serial.println("         FIX: re-mount the module SQUARE - its forward axis lined up with");
    Serial.println("         the nose of the buggy, or turned exactly 90, 180 or 270 deg from");
    Serial.println("         it - then run ?magalign again.");
    Serial.println("         (Aim counts here too: if the nose was not truly on magnetic north");
    Serial.println("         for those 5 seconds, that error is included in the figure above. Aim");
    Serial.println("         error changes between runs, a mounting angle does not - so run it");
    Serial.println("         twice to tell the two apart.)");
  }
  Serial.println("Note: ?magalign cannot detect a MIRRORED module - only ?compasscal can,");
  Serial.println("      from the direction of the turn.");

  cmdSave("");
  Serial.println("Saved.");

  // V2.5-Evo - 2026-08-17 - a completed ?magalign is the second piece of evidence that clears a
  // heading-disagreement degradation. Every failure path above returns before this line without
  // touching usrConf.mag_orientation, so reaching here means the mounting angle has genuinely been
  // re-measured against north and saved. That is the correction for the most common cause of a
  // 45+ deg COG-vs-compass gap on this build: a module glued in at 90/180/270 deg. It cannot detect
  // a MIRRORED frame — only ?compasscal can — so a mirrored module that is still wrong will simply
  // re-prove the disagreement the next time the rider coasts, which is the guard working as
  // intended rather than a reason to withhold the clear.
  headingDisagreeClearAfterCal("?magalign");

  blinkBind(2);
}
