// V2.5-Evo - 2026-05-13 - SW35: Logger fix — ledSyncState change-only gate (was: portMAX_DELAY every loop); throttle activity gate removed (was blocking all field logging)
// V2.5-Evo - 2026-05-12 - Logger activity gate: block start/stop during RTM/FM/active throttle; single-blink rejection (Option B)
// V2.5-Evo - 2026-05-12 - Fix REAL-BUG-B: guard aw.*/AW9523 calls in loggerLoop() and triggerBlink() with i2cMutex (FreeRTOS preemption race with generatePWM task)
// V2.5-Evo - 2026-05-11 - E7 Fix: +1 CSV column (remote_error); 26→27 columns; error_code_log from telemetry.error_code
// V2.5-Evo - 2026-05-08 - Bundle 1: +2 CSV columns (heading_error_dx10, d_error_dx10); 24→26 columns; VescLogData +4 bytes; extern g_heading_error_dx10/g_d_error_dx10 from RTMState.ino
// V2.5-Evo - 2026-05-06 - FIX-LOGDL-2: serial ?download CSV updated for LOG-EXT-1 fields (24 columns); WDT reset + FreeRTOS yield added inside read loop to support files >30KB without crash
// V2.5-Evo - 2026-05-06 - LOG-EXT-2: convertToLogData populates 12 heading debug fields; inline-duplicate of getRtmHeading() (must stay in sync with RTMState.ino); default lograte changed 1Hz→5Hz at line 21 (manual user edit, do not revert)
// V2.5-Evo - 2026-05-03 - H4: deleteCandidates String[]→char[][] (no heap alloc);
//                   deleteLogFile() active-file guard added
#include <FS.h>
#include <SPIFFS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <TinyGPS++.h>
#include <Adafruit_AW9523.h> // Required for LED and Button

extern TinyGPSPlus gps;
extern Adafruit_AW9523 aw;   // Pull in the global AW9523 expander
extern SemaphoreHandle_t i2cMutex;

#define MIN_FREE_SPACE_KB 500  

// Task handles and configuration
static TaskHandle_t loggerTaskHandle = NULL;
static SemaphoreHandle_t fileMutex = NULL;
SemaphoreHandle_t vescMutex = NULL;         // V3 fix (Bug 2): non-static — visible to VESC.ino. Protects vesc struct against FreeRTOS preemption race between loggerTask (reader) and getVescLoop() (writer) on the single ESP32-C3 core.
static volatile bool logging_active = false; // V3 fix (Bug 3): volatile — loggerTask on Core 0 reads this in a while(true) loop; without volatile the compiler may cache the value in a register and never see startLog()/stopLog() writes from Core 1.
static uint32_t log_interval_ms = 200; // Default 5 Hz =200 (was 1 Hz =1000)
static File currentLogFile;
static String currentLogFileName = "";
static uint32_t last_space_check = 0;
static const uint32_t SPACE_CHECK_INTERVAL = 60000; 

// LED/Button State Machine Variables (Non-Blocking)
static unsigned long lastBtnTime = 0;
static bool lastBtnState = HIGH;
static int blinksRemaining = 0;
static unsigned long lastBlinkTime = 0;
static bool blinkState = false;
static int blinkSpeedMs = 0;

// Forward declarations
void loggerTask(void* parameter);

// Triggers the non-blocking blink sequence
void triggerBlink(int blinks, int speedMs) {
  blinksRemaining = blinks * 2; 
  blinkSpeedMs = speedMs;
  lastBlinkTime = millis();
  blinkState = true; 
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  aw.digitalWrite(AP_L_AUX, LOW); // Turn ON immediately (Active-Low)
  xSemaphoreGive(i2cMutex);
}

// Returns true when the logger button must be ignored (system is actively running).
// Prevents accidental start/stop during RTM, FM, or active manual throttle.
// FM extension: add || fm_rx_active.load() here when FM implements its active flag.
static bool isLoggerGated() {
  extern std::atomic<bool> rtm_rx_active;
  return rtm_rx_active.load();
}

// Safely handles UI updates from the main thread
void loggerLoop() {
  unsigned long now = millis();

  // 1. Process LED Blinks
  if (blinksRemaining > 0) {
    if (now - lastBlinkTime >= blinkSpeedMs) {
       lastBlinkTime = now;
       blinksRemaining--;
       
       if (blinksRemaining > 0) {
          blinkState = !blinkState;
          xSemaphoreTake(i2cMutex, portMAX_DELAY);
          aw.digitalWrite(AP_L_AUX, blinkState ? LOW : HIGH);
          xSemaphoreGive(i2cMutex);
       } else {
          // Blinking finished, set solid state based on logging status
          xSemaphoreTake(i2cMutex, portMAX_DELAY);
          aw.digitalWrite(AP_L_AUX, logging_active ? LOW : HIGH);
          xSemaphoreGive(i2cMutex);
       }
    }
  } else {
    // Sync LED only on state change — not every loop iteration (i2cMutex contention with generatePWM)
    static bool ledSyncState = false;
    if (logging_active != ledSyncState) {
      ledSyncState = logging_active;
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        aw.digitalWrite(AP_L_AUX, logging_active ? LOW : HIGH);
        xSemaphoreGive(i2cMutex);
      }
    }
  }

  // 2. Process Button
  if (now - lastBtnTime >= 50) { // 50ms polling & debounce
    lastBtnTime = now;
    bool currentBtnState = lastBtnState; // safe default if mutex unavailable
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      currentBtnState = aw.digitalRead(AP_S_AUX);
      xSemaphoreGive(i2cMutex);
    }
    
    // Detect press (HIGH to LOW)
    if (currentBtnState == LOW && lastBtnState == HIGH) {
       if (isLoggerGated()) {
          triggerBlink(1, 30); // Reject: system active — one fast flash, no action
       } else if (isLoggingActive()) {
          stopLog();
       } else {
          startLog();
       }
    }
    lastBtnState = currentBtnState;
  }
}

// Scale and convert data
VescLogData convertToLogData() {
  // V3 fix (Bug 2): zero-init so vesc fields stay 0 if vescMutex times out (hold time is <1µs, so timeout is effectively impossible)
  VescLogData data = {};
  data.timestamp = millis();

  // V3 fix (Bug 2): guard all vesc.* reads with vescMutex.
  // This function runs on Core 0 (loggerTask); getVescLoop() writes vesc on Core 1 (loop task).
  // Without the mutex, the ESP32-S3 dual-core pipeline can produce a torn log record where
  // some fields are from one VESC packet and some from the next.
  if (vescMutex && xSemaphoreTake(vescMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    data.current_motor   = (int16_t)constrain(vesc.motCur,    -30000, 30000);
    data.current_battery = (int16_t)constrain(vesc.batCur,    -30000, 30000);
    int32_t scaled_duty  = vesc.duty / 10;
    data.duty_cycle      = (int8_t)constrain(scaled_duty,       -101,   101);
    data.voltage         = (uint16_t)(abs(vesc.batVolt));
    data.ERPM            = (int16_t)constrain(vesc.erpm / 10, -30000, 30000);
    data.temp_mos        = (int8_t)constrain(vesc.fetTemp / 10, -120,   120);
    data.fault_code      = vesc.fault_code;
    xSemaphoreGive(vescMutex);
  }

  // gps.* is written by getGPSLoop (main loop task) and read here in loggerTask — FreeRTOS preemption race on the single ESP32-C3 core.
  // The race is benign: GPS updates at ~1Hz and log writes at 1Hz,
  // so collisions are rare and the worst case is one torn log record. TinyGPS++ is not thread-safe;
  // a gpsMutex would be the strict fix if every record must be clean.
  data.speed     = (uint16_t)(gps.speed.kmph() * 10);
  data.latitude  = gps.location.lat();
  data.longitude = gps.location.lng();
  data.datetime  = gps.time.value();

  // ============================================================
  // LOG-EXT-2: Populate heading source debug fields (LOG-EXT-1).
  // All externs are declared locally to keep this a single-file edit.
  // ============================================================
  {
    extern volatile uint8_t      thr_received;
    extern std::atomic<bool>     rtm_rx_active;
    extern std::atomic<uint8_t>  rtm_steer_override;
    extern bool                  gps_phase_b_ok;
    extern float                 gps_last_course_deg;
    extern unsigned long         gps_last_course_ms;
    extern float                 compass_snapshot_heading;
    extern unsigned long         compass_snapshot_ms;
    extern float                 gps_last_speed_kmh;
    extern float                 getCompassHeading();

    // Simple state reads
    data.thr_received_log       = thr_received;
    data.rtm_rx_active_log      = rtm_rx_active.load() ? 1 : 0;
    data.rtm_steer_override_log = rtm_steer_override.load();
    data.gps_phase_b_ok_log     = gps_phase_b_ok ? 1 : 0;

    // Live compass heading × 10 (0xFFFF = invalid/uncalibrated)
    float live_compass = getCompassHeading();
    if (live_compass >= 0.0f && live_compass < 360.0f) {
      data.compass_live_dx10 = (uint16_t)(live_compass * 10.0f);
    } else {
      data.compass_live_dx10 = 0xFFFF;
    }

    // Snapshot heading × 10 + snapshot age in seconds (0xFFFF = no snapshot)
    unsigned long now_ms = millis();
    if (compass_snapshot_heading >= 0.0f && compass_snapshot_ms > 0) {
      data.compass_snap_dx10 = (uint16_t)(compass_snapshot_heading * 10.0f);
      unsigned long age_s = (now_ms - compass_snapshot_ms) / 1000UL;
      data.snap_age_s = (uint16_t)((age_s > 0xFFFEUL) ? 0xFFFE : age_s);
    } else {
      data.compass_snap_dx10 = 0xFFFF;
      data.snap_age_s        = 0xFFFF;
    }

    // GPS COG × 10 + COG age in 10ms units (0xFFFF = no fix or invalid)
    if (gps_last_course_ms > 0 && gps_last_course_deg >= 0.0f && gps_last_course_deg < 360.0f) {
      data.gps_course_dx10 = (uint16_t)(gps_last_course_deg * 10.0f);
      unsigned long age_ms    = now_ms - gps_last_course_ms;
      unsigned long age_units = age_ms / 10UL;
      data.cog_age_ms_div10   = (uint16_t)((age_units > 0xFFFEUL) ? 0xFFFE : age_units);
    } else {
      data.gps_course_dx10  = 0xFFFF;
      data.cog_age_ms_div10 = 0xFFFF;
    }

    // ============================================================
    // CRITICAL MAINTENANCE: This block is an inline duplicate of
    // getRtmHeading() in RTMState.ino (D5). If you change the heading
    // source selection logic there, you MUST update this duplicate
    // to match, or log records will diverge from runtime behavior.
    // The duplicate exists to keep this a single-file edit per project rule.
    // ============================================================
    uint16_t mode          = usrConf.rtm_use_compass;
    uint16_t cog_min_speed = usrConf.rtm_cog_min_speed_kmh;
    uint8_t  src           = 0;       // 0 = NONE
    uint8_t  conf          = 0;       // 0 = NONE
    float    chosen        = -1.0f;

    if (mode == 2) {
      // Compass-only mode (DIAGNOSTIC)
      if (live_compass >= 0.0f) {
        src    = 3;     // COMPASS_LIVE
        conf   = 2;     // MEDIUM
        chosen = live_compass;
      }
    } else {
      // Modes 0 and 1: GPS COG primary
      bool cog_valid = (gps_last_course_ms > 0) &&
                       (gps_last_course_deg >= 0.0f) &&
                       ((now_ms - gps_last_course_ms) < 1500UL) &&
                       (gps_last_speed_kmh >= (float)cog_min_speed);
      if (cog_valid) {
        src    = 1;     // GPS_COG
        conf   = 3;     // HIGH
        chosen = gps_last_course_deg;
      } else if (mode == 1) {
        // Hybrid: fall back to compass snapshot
        if (compass_snapshot_heading >= 0.0f && compass_snapshot_ms > 0) {
          unsigned long snap_age_ms = now_ms - compass_snapshot_ms;
          if (snap_age_ms < 1000UL) {
            src    = 2;   // COMPASS_SNAPSHOT
            conf   = 2;   // MEDIUM
            chosen = compass_snapshot_heading;
          } else if (snap_age_ms < 3000UL) {
            src    = 2;   // COMPASS_SNAPSHOT
            conf   = 1;   // LOW
            chosen = compass_snapshot_heading;
          }
        }
      }
      // Mode 0 with no valid COG: src/conf stay 0 (hold straight)
    }

    data.rtm_source              = src;
    data.rtm_confidence          = conf;
    data.rtm_heading_chosen_dx10 = (chosen < 0.0f) ? -1 : (int16_t)(chosen * 10.0f);
  }

  // V2.5-Evo - 2026-05-08 - Bundle 1: heading controller tuning telemetry (from RTMState.ino globals)
  {
    extern int16_t g_heading_error_dx10;
    extern int16_t g_d_error_dx10;
    data.heading_error_dx10 = g_heading_error_dx10;
    data.d_error_dx10       = g_d_error_dx10;
  }

  // V2.5-Evo - 2026-05-11 - E7 Fix: log BREmote error code so E7 events are visible in CSV
  // rather than inferred from abrupt log restarts. 0 = no error, 7 = water ingress.
  data.error_code_log = telemetry.error_code;

  return data;
}

// Check and manage SPIFFS space
bool ensureFreeSpace() {
  size_t totalBytes = SPIFFS.totalBytes();
  size_t freeBytes = totalBytes - SPIFFS.usedBytes();

  if (freeBytes > (MIN_FREE_SPACE_KB * 1024)) return true;

  Serial.printf("Space low: %u KB free (need %d)\n", freeBytes / 1024, MIN_FREE_SPACE_KB);
  // V3 fix (Bug 4): removed the block that closed currentLogFile here.
  // The old code cleared currentLogFileName before building the candidate list, so the
  // active file lost its exclusion identity and could be deleted along with the old logs.
  // The active file stays open; SPIFFS allows deleting other files while one is held open.

  // char[20][32] instead of String[20] — avoids 20 heap allocations during
  // log cleanup. SPIFFS filenames max ~18 chars + slash + null, 32 is safe.
  char deleteCandidates[20][32];
  int candidateCount = 0;
  File root = SPIFFS.open("/");
  if (root && root.isDirectory()) {
    File file = root.openNextFile();
    while (file && candidateCount < 20) {
      String filename = String(file.name());
      String fullPath = "/" + filename;
      if (fullPath.endsWith(".log")) {
        if (fullPath == currentLogFileName) {
          // V3 fix (Bug 4): never delete the file we are currently writing to
        } else {
          strncpy(deleteCandidates[candidateCount++], fullPath.c_str(), 31);
          deleteCandidates[candidateCount-1][31] = '\0'; // null-terminate
        }
      }
      file.close();
      file = root.openNextFile();
    }
    root.close();
  }

  if (candidateCount == 0) return false;

  int deleted = 0;
  for (int i = 0; i < candidateCount && (SPIFFS.totalBytes() - SPIFFS.usedBytes()) < (MIN_FREE_SPACE_KB * 1024); i++) {
    for (int retry = 0; retry < 5; retry++) {
      vTaskDelay(pdMS_TO_TICKS(100 * (retry + 1)));
      if (SPIFFS.remove(deleteCandidates[i])) {
        deleted++;
        break;
      }
    }
  }

  freeBytes = SPIFFS.totalBytes() - SPIFFS.usedBytes();
  return freeBytes >= (MIN_FREE_SPACE_KB * 1024);
}

// Create new log file
bool createNewLogFile() {
  Serial.println("Waiting for GPS lock to timestamp log file...");
  uint32_t startWait = millis();
  
  while (!gps.location.isValid() || !gps.date.isValid()) {
    vTaskDelay(pdMS_TO_TICKS(500));
    if (millis() - startWait > 300000) {  
      Serial.println("GPS fix timeout!");
      return false;
    }
    if (!logging_active) return false;
  }

  Serial.println("GPS lock acquired!");
  if (!ensureFreeSpace()) return false;

  char filenameBuffer[30];
  snprintf(filenameBuffer, sizeof(filenameBuffer), "/%02d%02d%02d_%02d%02d%02d.log", 
           gps.date.month(), gps.date.day(), (gps.date.year() % 100), 
           gps.time.hour(), gps.time.minute(), gps.time.second());
           
  currentLogFileName = String(filenameBuffer);
  
  if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    currentLogFile = SPIFFS.open(currentLogFileName, FILE_WRITE);
    xSemaphoreGive(fileMutex);
    
    if (!currentLogFile) return false;
    Serial.printf("Created log file: %s\n", currentLogFileName.c_str());
    return true;
  }
  return false;
}

// Logger background task (SPIFFS writes only!)
void loggerTask(void* parameter) {
  while (true) {
    if (logging_active) {
      if (!currentLogFile || currentLogFileName.length() == 0) {
        if (!createNewLogFile()) {
          vTaskDelay(pdMS_TO_TICKS(100));
          continue;
        }
      }

      if (millis() - last_space_check > SPACE_CHECK_INTERVAL) {
        last_space_check = millis();
        if (!ensureFreeSpace()) continue;
      }

      VescLogData logData = convertToLogData();
      
      if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (currentLogFile) {
          currentLogFile.write((uint8_t*)&logData, sizeof(VescLogData));
          currentLogFile.flush();
        }
        xSemaphoreGive(fileMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(log_interval_ms));
  }
}

// Initialize logger and hardware pins
void initLogger() {
  Serial.println("Initializing data logger...");
  
  // CRITICAL FIX: Removed SPIFFS.begin() here to prevent crashes.
                
  fileMutex = xSemaphoreCreateMutex();
  if (fileMutex == NULL) {
    Serial.println("FATAL: Failed to create fileMutex!");
    return;
  }
  // V3 fix (Bug 2): must be created here in setup() — before loop() starts calling getVescLoop() on Core 1
  vescMutex = xSemaphoreCreateMutex();
  if (vescMutex == NULL) {
    Serial.println("FATAL: Failed to create vescMutex!");
    return;
  }

  // Setup the hardware pins on the AW9523
  aw.pinMode(AP_S_AUX, INPUT_PULLUP);
  aw.pinMode(AP_L_AUX, OUTPUT);
  aw.digitalWrite(AP_L_AUX, HIGH); // OFF

  // Start SPIFFS Background task
  xTaskCreatePinnedToCore(loggerTask, "DataLogger", 4096, NULL, 1, &loggerTaskHandle, 0);

  Serial.println("Data logger initialized successfully");
}

void startLog() {
  if (logging_active) return;
  Serial.println("Starting data logging...");
  logging_active = true;
  last_space_check = millis();
  
  triggerBlink(5, 80); // Fast start
}

void stopLog() {
  if (!logging_active) return;
  Serial.println("Stopping data logging...");
  logging_active = false;
  
  triggerBlink(2, 400); // Slow stop (400ms pulses)

  if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
    if (currentLogFile) {
      currentLogFile.close();
      Serial.printf("Closed log file: %s\n", currentLogFileName.c_str());
    }
    currentLogFileName = "";
    xSemaphoreGive(fileMutex);
  }
}

void setLogRate(float log_rate_Hz) {
  if (log_rate_Hz <= 0 || log_rate_Hz > 1000) return;
  log_interval_ms = (uint32_t)(1000.0 / log_rate_Hz);
  Serial.printf("Log rate set to %.2f Hz (interval: %u ms)\n", log_rate_Hz, log_interval_ms);
}

void listLogFiles() {
  File root = SPIFFS.open("/");
  if (!root || !root.isDirectory()) return;

  Serial.println("\n=== Available Log Files ===");
  Serial.println("Filename\t\tSize (KB)");
  Serial.println("--------------------------------------------");

  File file = root.openNextFile();
  int fileCount = 0;
  
  while (file) {
    String filename = String(file.name());
    if (filename.endsWith(".log")) {
      size_t fileSize = file.size();
      Serial.printf("%s\t%.2f\n", filename.c_str(), fileSize / 1024.0);
      fileCount++;
    }
    file = root.openNextFile();
  }
  Serial.printf("\nTotal log files: %d\n", fileCount);
}

void downloadLogFile(const char* filename) {
  String fullPath = String(filename);
  fullPath.trim();  
  while (fullPath.startsWith("/")) fullPath.remove(0, 1);
  fullPath = "/" + fullPath;
  
  if (!SPIFFS.exists(fullPath)) return;

  File file = SPIFFS.open(fullPath, FILE_READ);
  if (!file) return;

  Serial.println("\n=== BEGIN CSV DATA ===");
  // V2.5-Evo - 2026-05-11 - E7 Fix: header updated to 27 fields (+remote_error)
  Serial.println("timestamp_ms,motor_current_A,battery_current_A,duty_cycle_%,voltage_V,ERPM,temp_mos_C,fault_code,speed_kmh,latitude,longitude,datetime_unix,thr_received,rtm_source,rtm_confidence,rtm_rx_active,gps_phase_b_ok,rtm_steer_override,rtm_heading_chosen_dx10,compass_live_dx10,compass_snap_dx10,snap_age_s,gps_course_dx10,cog_age_ms_div10,heading_error_dx10,d_error_dx10,remote_error");

  VescLogData logData;
  uint16_t recordCount = 0;
  while (file.available()) {
    // V2.5-Evo - 2026-05-06 - FIX-LOGDL-2: feed WDT inside loop and yield to FreeRTOS.
    // Without these, files >~30KB cause WDT (3s timeout) to fire mid-download (Andres
    // confirmed crash at ~3 min / ~350KB on 050626_204204.log).
    esp_task_wdt_reset();

    size_t bytesRead = file.read((uint8_t*)&logData, sizeof(VescLogData));

    if (bytesRead == sizeof(VescLogData)) {
      Serial.printf("%u,%.2f,%.2f,%d,%.1f,%d,%u,%u,%.1f,%.6f,%.6f,%u,%u,%u,%u,%u,%u,%u,%d,%u,%u,%u,%u,%u,%d,%d,%u\n",
                    logData.timestamp,
                    logData.current_motor / 100.0f,
                    logData.current_battery / 100.0f,
                    (int16_t)logData.duty_cycle,
                    logData.voltage / 10.0f,
                    (int32_t)logData.ERPM * 10,
                    logData.temp_mos,
                    logData.fault_code,
                    logData.speed / 10.0f,
                    logData.latitude,
                    logData.longitude,
                    logData.datetime,
                    (unsigned)logData.thr_received_log,
                    (unsigned)logData.rtm_source,
                    (unsigned)logData.rtm_confidence,
                    (unsigned)logData.rtm_rx_active_log,
                    (unsigned)logData.gps_phase_b_ok_log,
                    (unsigned)logData.rtm_steer_override_log,
                    (int)logData.rtm_heading_chosen_dx10,
                    (unsigned)logData.compass_live_dx10,
                    (unsigned)logData.compass_snap_dx10,
                    (unsigned)logData.snap_age_s,
                    (unsigned)logData.gps_course_dx10,
                    (unsigned)logData.cog_age_ms_div10,
                    // Bundle 1: heading controller tuning columns (0x7FFF = no data sentinel)
                    (int)logData.heading_error_dx10,
                    (int)logData.d_error_dx10,
                    // E7 Fix: BREmote remote_error code (0 = none, 7 = E7 water ingress)
                    (unsigned)logData.error_code_log);

      // Yield to FreeRTOS every 50 records to keep other tasks responsive.
      if ((++recordCount % 50) == 0) {
        delay(1);
      }
    } else {
      break;
    }
  }
  file.close();
  Serial.println("=== END CSV DATA ===");
}

void deleteLogFile(const char* filename) {
  String fullPath = String(filename);
  fullPath.trim();
  while (fullPath.startsWith("/")) fullPath.remove(0, 1);
  fullPath = "/" + fullPath;
  
  // Do not delete the currently active log file
  if (logging_active && fullPath == currentLogFileName) {
    Serial.println("LOG: skipped delete of active log file");
    return;
  }

  if (SPIFFS.exists(fullPath)) {
    SPIFFS.remove(fullPath);
  }
}

bool isLoggingActive() {
  return logging_active;
}