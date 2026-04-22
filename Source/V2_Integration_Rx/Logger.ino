#include <FS.h>
#include <SPIFFS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <TinyGPS++.h>
#include <Adafruit_AW9523.h> // Required for LED and Button

extern TinyGPSPlus gps;
extern Adafruit_AW9523 aw;   // Pull in the global AW9523 expander

#define MIN_FREE_SPACE_KB 500  

// Task handles and configuration
static TaskHandle_t loggerTaskHandle = NULL;
static SemaphoreHandle_t fileMutex = NULL;
SemaphoreHandle_t vescMutex = NULL;         // V3 fix (Bug 2): non-static — visible to VESC.ino. Protects vesc struct against Core 0/1 data race between loggerTask (reader) and getVescLoop() (writer).
static volatile bool logging_active = false; // V3 fix (Bug 3): volatile — loggerTask on Core 0 reads this in a while(true) loop; without volatile the compiler may cache the value in a register and never see startLog()/stopLog() writes from Core 1.
static uint32_t log_interval_ms = 1000; // Default 1 Hz
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
  aw.digitalWrite(AP_L_AUX, LOW); // Turn ON immediately (Active-Low)
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
          aw.digitalWrite(AP_L_AUX, blinkState ? LOW : HIGH);
       } else {
          // Blinking finished, set solid state based on logging status
          aw.digitalWrite(AP_L_AUX, logging_active ? LOW : HIGH);
       }
    }
  } else {
    // Keep LED synced to state just in case
    aw.digitalWrite(AP_L_AUX, logging_active ? LOW : HIGH);
  }

  // 2. Process Button
  if (now - lastBtnTime >= 50) { // 50ms polling & debounce
    lastBtnTime = now;
    bool currentBtnState = aw.digitalRead(AP_S_AUX);
    
    // Detect press (HIGH to LOW)
    if (currentBtnState == LOW && lastBtnState == HIGH) {
       if (isLoggingActive()) {
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

  // gps.* is written by Core 1 (getGPSLoop feeds TinyGPS++ in the main loop task) and read here on Core 0.
  // This IS a cross-core access. The race is benign: GPS updates at ~1Hz and log writes at 1Hz,
  // so collisions are rare and the worst case is one torn log record. TinyGPS++ is not thread-safe;
  // a gpsMutex would be the strict fix if every record must be clean.
  data.speed     = (uint16_t)(gps.speed.kmph() * 10);
  data.latitude  = gps.location.lat();
  data.longitude = gps.location.lng();
  data.datetime  = gps.time.value();
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

  String deleteCandidates[20];
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
          deleteCandidates[candidateCount++] = fullPath;
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
    String targetFile = deleteCandidates[i];
    for (int retry = 0; retry < 5; retry++) {
      vTaskDelay(pdMS_TO_TICKS(100 * (retry + 1)));
      if (SPIFFS.remove(targetFile)) {
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
  Serial.println("timestamp_ms,motor_current_A,battery_current_A,duty_cycle_%,voltage_V,ERPM,temp_mos_C,fault_code,speed_kmh,latitude,longitude,datetime_unix");

  VescLogData logData;
  while (file.available()) {
    size_t bytesRead = file.read((uint8_t*)&logData, sizeof(VescLogData));
    
    if (bytesRead == sizeof(VescLogData)) {
      Serial.printf("%u,%.2f,%.2f,%d,%.1f,%d,%u,%u,%.1f,%.6f,%.6f,%u\n",
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
                    logData.datetime);
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
  
  if (SPIFFS.exists(fullPath)) {
    SPIFFS.remove(fullPath);
  }
}

bool isLoggingActive() {
  return logging_active;
}