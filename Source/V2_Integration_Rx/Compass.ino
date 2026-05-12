// V2.5-Evo - 2026-05-06 - D2: Add updateCompassSnapshot() and snapshot globals (clean heading captured during motor-idle for future RTM heading source)
// V2.5-Evo - 2026-04-25 - P7: Added getCompassHeading() function
#include <Wire.h>
#include <esp_task_wdt.h> // <-- Added to feed the Watchdog

#define QMC5883L_ADDR 0x0D

int16_t magX = 0, magY = 0, magZ = 0;
bool compass_detected = false;
float         compass_snapshot_heading = -1.0f; // Last "clean" compass heading captured while motor was idle (degrees, 0–360 clockwise from North). -1.0f = no valid snapshot yet.
unsigned long compass_snapshot_ms      = 0;     // millis() timestamp of last snapshot capture. 0 = no snapshot yet.

// We link to your existing save command to automate the process
extern void cmdSave(const String& params);

void initCompass() {
  Wire.setTimeOut(20);

  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(QMC5883L_ADDR);
  bool found = (Wire.endTransmission() == 0);
  xSemaphoreGive(i2cMutex);

  if (found) {
    compass_detected = true;
    Serial.println("QMC5883L Compass detected. Initializing...");

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(0x0B);
    Wire.write(0x01);
    Wire.endTransmission();
    xSemaphoreGive(i2cMutex);

    // 50Hz Data Rate for stability
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(0x09);
    Wire.write(0x15);
    Wire.endTransmission();
    xSemaphoreGive(i2cMutex);

    Serial.println("Compass Init OK (50Hz Mode).");
  } else {
    compass_detected = false;
    Serial.println("WARNING: QMC5883L Compass not found during init.");
  }
}

bool readCompassRaw() {
  if (!compass_detected) return false;

  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) {
    xSemaphoreGive(i2cMutex);
    return false;
  }

  uint8_t bytesReceived = Wire.requestFrom((uint8_t)QMC5883L_ADDR, (uint8_t)6, (uint8_t)true);

  if (bytesReceived == 6) {
    magX = (int16_t)(Wire.read() | (Wire.read() << 8));
    magY = (int16_t)(Wire.read() | (Wire.read() << 8));
    magZ = (int16_t)(Wire.read() | (Wire.read() << 8));
    xSemaphoreGive(i2cMutex);
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

  // Reject uncalibrated scale (default 1.0f after runcal is fine; 0.0f = never set)
  if (usrConf.mag_scale_x == 0.0f || usrConf.mag_scale_y == 0.0f) return -1.0f;

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

  // Gate 2: compass must be calibrated (default 0.0f means runcal was never run)
  if (usrConf.mag_scale_x == 0.0f || usrConf.mag_scale_y == 0.0f) return;

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