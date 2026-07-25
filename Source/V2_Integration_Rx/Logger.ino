// V2.5-Evo - 2026-07-25 - STAGE 0 PART B+C (instrumentation only): every log file now opens with an 8-byte self-describing header (magic "BRLG", format version, log level, record size) so a reader can parse a VARIABLE record size instead of assuming sizeof(VescLogData); the level is latched once per FILE in createNewLogFile() so changing the setting mid-session cannot corrupt an open file; level 4 writes a 65-byte record (59-byte level-3 record + gps_sent_per_s, cog_frozen_s, mux_err_cnt, loop_max_ms); ?download reads the header, steps by header.record_size, refuses a file with no valid magic in plain English instead of emitting garbage, and formats rows through the single shared logFormatCsvRow() that the WiFi path also calls — so the two CSV outputs cannot drift apart again. No confStruct change, sizeof stays 184, SW_VERSION stays 34, no control path touched.
// V2.5-Evo - 2026-07-24 - F9: +3 CSV columns (tx_distance_m, rssi_dbm, snr_db); 28→31 columns; VescLogData +6 bytes; distance decoded from telemetry.rtm_distance, RSSI/SNR from Radio.ino cache (g_last_rssi_dbm/g_last_snr_db); appended for parser compat; no confStruct change, SW_VERSION unchanged
// V2.5-Evo - 2026-07-19 - Rex INFO: corrected stale "ESP32-S3 dual-core / Core 0/Core 1" wording in convertToLogData() vescMutex comment to ESP32-C3 single-core / FreeRTOS-preemption (comment-only)
// V2.5-Evo - 2026-07-19 - FM triage: +1 CSV column (effective_steer, the steering byte calcPWM actually applied); 27→28 columns; VescLogData +1 byte; no-fix guard mirrored into inline getRtmHeading() duplicate (src/conf forced NONE without a fresh RX GPS fix, matching RTMState.ino)
// V2.5-Evo - 2026-05-13 - SW43: GPS gate relaxed to location.isValid() only — date absent when UART mux fragments RMC; T_HHMMSS filename when time valid but date missing
// V2.5-Evo - 2026-05-13 - SW40: loggerLoop() button section removed — checkButtons() is the sole AUX handler; pending timeout 5min→15s start-anyway (was: give-up)
// V2.5-Evo - 2026-05-13 - SW38: log_pending state — GPS gate moved to startLog()/loggerLoop(); LED heartbeat (1 blink/3s) while waiting; auto-transitions to active on fix; 5-min timeout → 3 slow error blinks
// V2.5-Evo - 2026-05-13 - SW37: createNewLogFile() — no GPS wait; file created immediately; GPS name if fix available, millis fallback otherwise
// V2.5-Evo - 2026-05-13 - SW37: loggerTask() periodic close+reopen every 30s — forces SPIFFS directory entry finalization; limits power-loss data loss to last 30s
// V2.5-Evo - 2026-05-13 - SW36: createNewLogFile() GPS fallback — 10s timeout then millis-based filename (was: 300s then return false, silently killing all logs with no GPS fix)
// V2.5-Evo - 2026-05-13 - SW36: remaining portMAX_DELAY in triggerBlink() and blink-active loggerLoop() path → pdMS_TO_TICKS(10)
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
SemaphoreHandle_t vescMutex = NULL;         // V2.5-Evo fix (Bug 2): non-static — visible to VESC.ino. Protects vesc struct against FreeRTOS preemption race between loggerTask (reader) and getVescLoop() (writer) on the single ESP32-C3 core.
static volatile bool logging_active  = false; // V2.5-Evo fix (Bug 3): volatile — loggerTask on Core 0 reads this in a while(true) loop; without volatile the compiler may cache the value in a register and never see startLog()/stopLog() writes from Core 1.
static volatile bool log_pending     = false; // GPS not yet valid; waiting to transition to logging_active
static uint32_t      log_pending_since = 0;   // millis() when pending started
static uint32_t      log_heartbeat_ms  = 0;   // last heartbeat blink while pending
#define LOG_GPS_PENDING_TIMEOUT_MS (15000UL)   // 15s wait for GPS timestamp; then start anyway with millis filename
#define LOG_GPS_HEARTBEAT_MS       (3000UL)    // 1 quick blink every 3s while waiting for fix
// V2.5-Evo - 2026-07-14 - Default lowered 5 Hz → 3 Hz (333ms) for prop/max-speed testing (Andres):
// 3 Hz is plenty for speed/trend logging and stretches on-board session capacity vs 5 Hz. Bump back
// to 5 Hz at runtime for RTM/steering analysis via the serial command "?lograte 5" (cmdLogRate →
// setLogRate(), System.ino). Rate is NOT persisted — this boot static is the only default lever.
static uint32_t log_interval_ms = 333; // Default 3 Hz =333 (was 5 Hz =200; 1 Hz =1000)
static File currentLogFile;
static String currentLogFileName = "";
// V2.5-Evo - 2026-07-25 - STAGE 0 PART B: the log level and record size in force for the file
// currently open. Latched ONCE in createNewLogFile() from usrConf.log_level and written into
// that file's header, so a rider changing the setting over WiFi mid-session cannot produce a
// file whose records stop matching its own header. Defaults describe a level-3 file so these
// are never nonsense even before the first file is created.
static uint8_t  active_log_level   = 3;
static uint16_t active_record_size = (uint16_t)sizeof(VescLogData);
static uint32_t last_space_check = 0;
static const uint32_t SPACE_CHECK_INTERVAL = 60000; 

// LED Blink State Machine Variables
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
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    aw.digitalWrite(AP_L_AUX, LOW); // Turn ON immediately (Active-Low)
    xSemaphoreGive(i2cMutex);
  }
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

  // 0. GPS-pending state — heartbeat LED + auto-transition when fix arrives
  if (log_pending) {
    // Heartbeat: 1 quick blink every 3s — "waiting for GPS, not logging yet"
    if (now - log_heartbeat_ms >= LOG_GPS_HEARTBEAT_MS) {
      log_heartbeat_ms = now;
      triggerBlink(1, 80);
    }
    // SW43: gate on location only — date may be absent when mux contention fragments RMC sentences
    if (gps.location.isValid()) {
      // Fix acquired — transition to active
      log_pending = false;
      logging_active = true;
      last_space_check = millis();
      triggerBlink(5, 80); // "Logging started" confirmation
      Serial.println("GPS location fix acquired — log started");
    } else if (now - log_pending_since >= LOG_GPS_PENDING_TIMEOUT_MS) {
      // 15s timeout — start anyway with millis-based filename; GPS data fills in per-record once fix arrives
      log_pending = false;
      logging_active = true;
      last_space_check = millis();
      triggerBlink(3, 200); // 3 medium blinks = "starting without GPS fix"
      Serial.println("Log started without GPS fix (15s timeout — millis filename)");
    }
  }

  // 1. Process LED Blinks
  if (blinksRemaining > 0) {
    if (now - lastBlinkTime >= blinkSpeedMs) {
       lastBlinkTime = now;
       blinksRemaining--;
       
       if (blinksRemaining > 0) {
          blinkState = !blinkState;
          if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            aw.digitalWrite(AP_L_AUX, blinkState ? LOW : HIGH);
            xSemaphoreGive(i2cMutex);
          }
       } else {
          // Blinking finished, set solid state based on logging status
          if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            aw.digitalWrite(AP_L_AUX, logging_active ? LOW : HIGH);
            xSemaphoreGive(i2cMutex);
          }
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

  // Button handled by checkButtons() in System.ino — single handler, no duplicate reads here
}

// Scale and convert data
VescLogData convertToLogData() {
  // V2.5-Evo fix (Bug 2): zero-init so vesc fields stay 0 if vescMutex times out (hold time is <1µs, so timeout is effectively impossible)
  VescLogData data = {};
  data.timestamp = millis();

  // V2.5-Evo fix (Bug 2): guard all vesc.* reads with vescMutex.
  // This function runs in loggerTask; getVescLoop() writes vesc from the loop task.
  // The ESP32-C3 is single-core, so these never run in parallel — but FreeRTOS can
  // preempt loggerTask mid-read, so without the mutex a torn log record can still
  // occur where some fields are from one VESC packet and some from the next.
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
    extern unsigned long         gps_last_ms;
    extern volatile uint8_t      g_effective_steer;
    extern float                 getCompassHeading();

    // Simple state reads
    data.thr_received_log       = thr_received;
    data.rtm_rx_active_log      = rtm_rx_active.load() ? 1 : 0;
    data.rtm_steer_override_log = rtm_steer_override.load();
    data.gps_phase_b_ok_log     = gps_phase_b_ok ? 1 : 0;
    data.effective_steer_log    = g_effective_steer;   // FM triage: steering byte actually applied by calcPWM()

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
          } else if (snap_age_ms < 8000UL) {   // Audit #9: synced to RTMState's 8000ms window (was 3000) so logged confidence matches the steering logic
            src    = 2;   // COMPASS_SNAPSHOT
            conf   = 1;   // LOW
            chosen = compass_snapshot_heading;
          }
        }
      }
      // Mode 0 with no valid COG: src/conf stay 0 (hold straight)
    }

    // No-fix guard mirror of getRtmHeading() (RTMState.ino, 2026-07-19 FM triage): with no
    // fresh RX GPS fix, no heading source is valid for steering — the bearing is computed from
    // gps_last_lat/lng which are 0,0 without a fix. Force NONE so the log matches runtime
    // behaviour (Fable audit: confidence=2 logged with datetime_unix=0).
    if (gps_last_ms == 0 || (now_ms - gps_last_ms) > 6000UL) {
      src = 0; conf = 0; chosen = -1.0f;
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

  // V2.5-Evo - 2026-07-24 - F9: owner-requested range telemetry (distance + link quality).
  // Lets a session log show achievable range at the current TX/RX radio settings.
  {
    // last_packet is already a global (volatile unsigned long) from BREmote_V2_Rx.h — no local extern needed.
    extern float         g_last_rssi_dbm;  // cached last-packet RSSI (Radio.ino, F9)
    extern float         g_last_snr_db;    // cached last-packet SNR  (Radio.ino, F9)

    // Distance: decode the SAME telemetry.rtm_distance byte RTMState.ino maintains for the TX bar.
    // Encoding (RTMState.ino ~line 858): 0-99 = tenths of a metre; 100-254 = whole metres offset by 90;
    // 0xFF = N/A. Re-expanded here to 0.1 m units so the CSV carries the identical RTM distance value.
    uint8_t dist_enc = telemetry.rtm_distance;
    if (dist_enc == 0xFF) {
      data.tx_distance_dx10 = 0xFFFF;                                  // N/A (no valid GPS pair)
    } else if (dist_enc <= 99) {
      data.tx_distance_dx10 = (uint16_t)dist_enc;                      // already tenths of a metre (0.0-9.9 m)
    } else {
      data.tx_distance_dx10 = (uint16_t)(((uint16_t)dist_enc - 90u) * 10u); // whole metres → tenths (10-164 m)
    }

    // Link quality: use the cached RSSI/SNR (never touch the radio SPI bus from this task). Mark N/A while
    // in failsafe (no control packet within failsafe_time) so a link drop reads as a clear gap, not stale data.
    if ((millis() - last_packet) < usrConf.failsafe_time) {
      data.rssi_dbm = (int16_t)lroundf(g_last_rssi_dbm);
      data.snr_dx10 = (int16_t)lroundf(g_last_snr_db * 10.0f);
    } else {
      data.rssi_dbm = 0x7FFF;   // N/A — failsafe
      data.snr_dx10 = 0x7FFF;   // N/A — failsafe
    }
  }

  return data;
}

// ============================================================
// V2.5-Evo - 2026-07-25 - STAGE 0 PART C
// fillLevel4Diag - add the level-4 ("Deep") diagnostic block to a log record
// ============================================================
//
// What it does:
//   Fills the four extra fields that level 4 appends to the standard level-3 record. It reads
//   the free-running diagnostic counters declared in BREmote_V2_Rx.h and does no I/O.
//
// Inputs:  rec - a VescLogDataL4 whose .base has already been filled by convertToLogData()
// Outputs: none (rec is filled in place)
// Side effects: resets g_diag_loop_max_us_log to 0 — that field is defined as "worst loop since
//   the PREVIOUS record", so consuming it here is what makes consecutive records comparable.
//   ?diag keeps its own separate peak (g_diag_loop_max_us) so the two never steal from each other.
//
// Sentinels: cog_frozen_s == 255 means no COG value has ever been captured this session (NOT
//   "0 seconds"); 254 means 254 seconds or longer. mux_err_cnt saturates at 0xFFFE.
static void fillLevel4Diag(VescLogDataL4 &rec)
{
  uint32_t now_ms = millis();

  // Sentences parsed in the last completed 1-second window (maintained by getGPSLoop()).
  rec.gps_sent_per_s = g_diag_gps_sent_per_s;

  // Seconds since the COG VALUE last moved. THE important field: cog_age_ms_div10 (already in
  // the level-3 record) is derived from gps_last_course_ms, which refreshes on every course
  // sentence even when the heading number never changes — so it read healthy right through the
  // frozen-COG failure. This one only advances when the value itself has genuinely stopped moving.
  if (g_diag_cog_change_ms == 0) {
    rec.cog_frozen_s = 255;                       // no COG value has EVER been seen this session
  } else {
    uint32_t frozen_s = (uint32_t)(now_ms - g_diag_cog_change_ms) / 1000UL;
    rec.cog_frozen_s = (frozen_s > 254UL) ? 254 : (uint8_t)frozen_s;
  }

  // Running session total of AW9523 UART-mux read-back mismatches (motor EMI corrupting I2C).
  // Logged as a running total rather than a delta so any single record answers "how bad is it
  // by now", and any two records answer "how many happened between these two".
  uint32_t mux_err = g_diag_mux_errors;
  rec.mux_err_cnt = (mux_err > 0xFFFEUL) ? 0xFFFE : (uint16_t)mux_err;

  // Worst loop() body since the previous record, in ms, rounded to nearest, then reset.
  uint32_t max_us = g_diag_loop_max_us_log;
  g_diag_loop_max_us_log = 0;
  uint32_t max_ms = (max_us + 500UL) / 1000UL;
  rec.loop_max_ms = (max_ms > 0xFFFEUL) ? 0xFFFE : (uint16_t)max_ms;
}

// Check and manage SPIFFS space
bool ensureFreeSpace() {
  size_t totalBytes = SPIFFS.totalBytes();
  size_t freeBytes = totalBytes - SPIFFS.usedBytes();

  if (freeBytes > (MIN_FREE_SPACE_KB * 1024)) return true;

  Serial.printf("Space low: %u KB free (need %d)\n", freeBytes / 1024, MIN_FREE_SPACE_KB);
  // V2.5-Evo fix (Bug 4): removed the block that closed currentLogFile here.
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
          // V2.5-Evo fix (Bug 4): never delete the file we are currently writing to
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

// Create new log file — no GPS wait; file created immediately on startLog()
bool createNewLogFile() {
  if (!ensureFreeSpace()) return false;

  char filenameBuffer[30];
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
    // Full GPS timestamp: MMDDYY_HHMMSS.log
    snprintf(filenameBuffer, sizeof(filenameBuffer), "/%02d%02d%02d_%02d%02d%02d.log",
             gps.date.month(), gps.date.day(), (gps.date.year() % 100),
             gps.time.hour(), gps.time.minute(), gps.time.second());
    Serial.println("GPS timestamp filename");
  } else if (gps.location.isValid() && gps.time.isValid()) {
    // Location + time but no date (RMC fragmented by UART mux): T_HHMMSS_<ms>.log
    snprintf(filenameBuffer, sizeof(filenameBuffer), "/T_%02d%02d%02d_%u.log",
             gps.time.hour(), gps.time.minute(), gps.time.second(), (unsigned)(millis() % 100000));
    Serial.println("GPS time-only filename (no date from RMC)");
  } else {
    snprintf(filenameBuffer, sizeof(filenameBuffer), "/ms%u.log", (unsigned)millis());
    Serial.println("No GPS fix — millis filename");
  }

  currentLogFileName = String(filenameBuffer);
  
  if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    currentLogFile = SPIFFS.open(currentLogFileName, FILE_WRITE);

    // ============================================================
    // V2.5-Evo - 2026-07-25 - STAGE 0 PART B: latch the level and stamp the file header.
    //
    // The level is read from config exactly ONCE, here, and immediately written into the file.
    // Every record appended afterwards is built to match. That is what makes a file
    // self-describing: whatever the rider does to the setting later, this file's header and
    // this file's records always agree with each other.
    //
    // The header is written only on creation. The periodic close+reopen in loggerTask() uses
    // FILE_APPEND, so it never rewrites or duplicates it.
    // ============================================================
    if (currentLogFile) {
      active_log_level   = logResolveLevel();
      active_record_size = logRecordSizeForLevel(active_log_level);

      LogFileHeader hdr;
      hdr.magic       = LOG_FILE_MAGIC;
      hdr.format_ver  = LOG_FILE_FORMAT_VER;
      hdr.log_level   = active_log_level;
      hdr.record_size = active_record_size;
      currentLogFile.write((uint8_t*)&hdr, sizeof(hdr));
      currentLogFile.flush();
    }
    xSemaphoreGive(fileMutex);

    if (!currentLogFile) return false;
    Serial.printf("Created log file: %s (log_level %u, %u bytes/record)\n",
                  currentLogFileName.c_str(),
                  (unsigned)active_log_level,
                  (unsigned)active_record_size);
    return true;
  }
  return false;
}

// Logger background task (SPIFFS writes only!)
void loggerTask(void* parameter) {
  static uint32_t last_reopen_ms = 0;
  const uint32_t  REOPEN_INTERVAL_MS = 30000; // Close+reopen every 30s — forces SPIFFS directory finalization
  while (true) {
    if (logging_active) {
      if (!currentLogFile || currentLogFileName.length() == 0) {
        if (!createNewLogFile()) {
          vTaskDelay(pdMS_TO_TICKS(100));
          continue;
        }
        last_reopen_ms = millis();
      }

      if (millis() - last_space_check > SPACE_CHECK_INTERVAL) {
        last_space_check = millis();
        if (!ensureFreeSpace()) continue;
      }

      // Periodic close+reopen: finalizes SPIFFS directory entry so abrupt power-off
      // only loses data since the last reopen, not the entire session.
      if (millis() - last_reopen_ms >= REOPEN_INTERVAL_MS) {
        last_reopen_ms = millis();
        if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
          if (currentLogFile) {
            currentLogFile.close();
            currentLogFile = SPIFFS.open(currentLogFileName, FILE_APPEND);
          }
          xSemaphoreGive(fileMutex);
        }
      }

      // ============================================================
      // V2.5-Evo - 2026-07-25 - STAGE 0 PART C: build the record for the level this FILE was
      // created at (active_log_level), not for whatever the config says right now. Both tiers
      // are assembled into one byte buffer so there is still exactly ONE write path holding
      // fileMutex — the mutex block below is unchanged apart from taking a length instead of
      // a hardcoded sizeof().
      // ============================================================
      uint8_t  rec_buf[sizeof(VescLogDataL4)];
      uint16_t rec_len;
      if (active_log_level >= 4) {
        VescLogDataL4 logData4;
        logData4.base = convertToLogData();
        fillLevel4Diag(logData4);
        memcpy(rec_buf, &logData4, sizeof(logData4));
        rec_len = (uint16_t)sizeof(logData4);
      } else {
        VescLogData logData = convertToLogData();
        memcpy(rec_buf, &logData, sizeof(logData));
        rec_len = (uint16_t)sizeof(logData);
      }

      if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (currentLogFile) {
          currentLogFile.write(rec_buf, rec_len);
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
  // V2.5-Evo fix (Bug 2): must be created here in setup() — before loop() starts calling getVescLoop() on Core 1
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
  if (logging_active || log_pending) return;
  Serial.println("Log requested...");
  // SW43: gate on location only — date may be absent when mux contention fragments RMC sentences
  if (gps.location.isValid()) {
    logging_active = true;
    last_space_check = millis();
    triggerBlink(5, 80); // "Logging started" confirmation
    Serial.println("Log started — GPS location fix available");
  } else {
    log_pending     = true;
    log_pending_since = millis();
    log_heartbeat_ms  = millis();
    triggerBlink(1, 400); // Single slow blink: "acknowledged, waiting for GPS"
    Serial.println("Log pending — waiting for GPS location fix (up to 15s)");
  }
}

void stopLog() {
  if (!logging_active && !log_pending) return;
  Serial.println("Stopping data logging...");
  log_pending    = false;
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

  // ============================================================
  // V2.5-Evo - 2026-07-25 - STAGE 0 PART B: read the self-describing file header FIRST.
  //
  // Records are no longer a fixed size — a level-4 file writes 65-byte records where a level-3
  // file writes 59 — so the reader must be told the size by the file rather than assuming
  // sizeof(VescLogData). Stepping by the wrong size does not fail loudly; it walks off the
  // record boundary and prints thousands of lines of plausible-looking nonsense, which is worse
  // than no data at all. Hence: no valid header, no output.
  //
  // Files written before this change have no header (their first 4 bytes are a millis()
  // timestamp), so the magic test rejects them with a plain-English explanation. Those files
  // were ALREADY undecodable after the 53 -> 59 byte record change (F9, 2026-07-24); this only
  // makes the failure visible instead of silent.
  // ============================================================
  LogFileHeader hdr;
  if (file.size() < sizeof(LogFileHeader) ||
      file.read((uint8_t*)&hdr, sizeof(hdr)) != sizeof(hdr) ||
      hdr.magic != LOG_FILE_MAGIC) {
    file.close();
    Serial.println("LOG: this file has no BRLG header, so its record layout is unknown.");
    Serial.println("LOG: it was written before the self-describing log format (or is corrupt).");
    Serial.println("LOG: nothing printed — a wrong record size produces convincing garbage. Delete it with ?deletelog.");
    return;
  }
  if (hdr.format_ver != LOG_FILE_FORMAT_VER ||
      hdr.record_size < (uint16_t)sizeof(VescLogData) ||
      hdr.record_size > (uint16_t)sizeof(VescLogDataL4)) {
    file.close();
    Serial.printf("LOG: unsupported log format (header version %u, %u bytes/record).\n",
                  (unsigned)hdr.format_ver, (unsigned)hdr.record_size);
    Serial.printf("LOG: this firmware reads header version %u with %u-%u bytes/record. Nothing printed.\n",
                  (unsigned)LOG_FILE_FORMAT_VER,
                  (unsigned)sizeof(VescLogData), (unsigned)sizeof(VescLogDataL4));
    return;
  }

  Serial.println("\n=== BEGIN CSV DATA ===");
  // V2.5-Evo - 2026-07-19 - FM triage: header updated to 28 fields (+effective_steer)
  // V2.5-Evo - 2026-07-24 - F9: header updated to 31 fields (+tx_distance_m, +rssi_dbm, +snr_db). N/A sentinels: distance -1.0, rssi -999, snr -99.0
  // V2.5-Evo - 2026-07-25 - STAGE 0: the column list lives ONCE in BREmote_V2_Rx.h and the WiFi
  // download path emits the same macro, so the two can no longer drift. The header printed must
  // match the level the file was actually RECORDED at (from its own header), not the level the
  // config happens to be set to now.
  Serial.println((hdr.log_level >= 4) ? LOG_CSV_HEADER_L4 : LOG_CSV_HEADER_L3);

  uint8_t  rec_buf[sizeof(VescLogDataL4)];
  char     row[LOG_CSV_ROW_BUF];
  uint16_t recordCount = 0;
  while (file.available()) {
    // V2.5-Evo - 2026-05-06 - FIX-LOGDL-2: feed WDT inside loop and yield to FreeRTOS.
    // Without these, files >~30KB cause WDT (3s timeout) to fire mid-download (Andres
    // confirmed crash at ~3 min / ~350KB on 050626_204204.log).
    esp_task_wdt_reset();

    // Step by the size THIS file declares. A short read means the tail is truncated (power cut
    // mid-write): stop cleanly rather than formatting a partial record.
    size_t bytesRead = file.read(rec_buf, (size_t)hdr.record_size);

    if (bytesRead == (size_t)hdr.record_size) {
      logFormatCsvRow(row, sizeof(row), rec_buf, hdr.record_size, hdr.log_level);
      Serial.print(row);

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

void deleteAllLogFiles() {
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  int deleted = 0, skipped = 0;
  while (file) {
    String fname = String("/") + file.name();
    file = root.openNextFile();  // advance before remove
    if (!fname.endsWith(".log")) continue;
    if (logging_active && fname == currentLogFileName) { skipped++; continue; }
    SPIFFS.remove(fname);
    deleted++;
  }
  Serial.printf("LOG: deleted %d log file(s)", deleted);
  if (skipped) Serial.printf(", skipped %d active", skipped);
  Serial.println();
}

bool isLoggingActive() {
  return logging_active;
}