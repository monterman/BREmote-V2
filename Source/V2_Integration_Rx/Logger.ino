// V2.5-Evo - 2026-10-06 - LOG FORMAT 3 (see BREmote_V2_Rx.h): fillVesc2Block() fills the VESC 2 block in the LEVEL-4 record now (owner ruling: both VESCs at level 4), so loggerTask() calls it for level 4 AND level 5 (logData5.l4). ?download refuses a file of another log format with a plain-English message that says which firmware wrote it and what to do (download before flashing; the PC log reader still decodes it). No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-10-06 - VESC 2 OVER CAN, part 2 (see BREmote_V2_Rx.h): loggerTask() fills the new 14 B VESC 2 block at the tail of the level-5 record (fillVesc2Block()), deciding freshness at log time against VESC 2's own age stamp: never answered or older than kVesc2StaleMs = every value field written as its N/A sentinel, age logged as the real age. Level 5 only; levels 3 and 4 unchanged. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-10-03 - I2C ENABLE-SWAP STARVATION, STEP 5 of 5 (Rex delta audit L-3; the code fix is in PWM.ino): fillLevel4Diag()'s two swap-failure delta statics are SEEDED FROM THE LIVE COUNTERS ON THE FIRST CALL instead of starting at 0. Starting at 0 made the first level-4 row of a session report the whole boot's accumulated failure count as if it had happened inside that one ~333 ms row, and because the nibbles saturate at 15 a board that had seen any swap failures before logging started wrote a first row reading 15/15 - "pinned" - from a healthy bus. Seeded inside loggerTask, so the single-writer contract is untouched: this task still reads the PWM task's volatiles and writes only its own statics. ZERO log bytes, no LOG_FILE_FORMAT_VER bump (stays 2), record sizes stay 62 / 90 / 112, no confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-10-03 - I2C ENABLE-SWAP STARVATION, STEP 4 of 4 (see Compass.ino, System.ino, PWM.ino): convertToLogData() STOPS READING THE COMPASS OVER I2C. It used to call getCompassHeading() on EVERY log record - ~2.89 Hz at the default rate - which is two chained Wire transactions inside one portMAX_DELAY mutex hold, from loggerTask, with NO throttle gate, NO rtm_use_compass gate and NO heading-latch gate. So whenever logging was on, the RX hit the compass about three times a second at ANY throttle, including full throttle with a rider on the rope, contending for the same mutex as the 100 Hz enable swap that decides which of the two motors receives the single PPM output; and loggerTask is not registered with the task watchdog, so a hold that never returned would not have panicked anything. This was the compass starver nobody had found. IT IS NOT AN ARBITRATION PROBLEM: the logger and the compass were never two competing bus users - the logger's I2C traffic WAS a compass read - so the fix is a cache, not a referee. getCompassHeading() (Compass.ino) now publishes its last successful result, and this file reads that, with an age test: past kCompassLiveCacheMaxAgeMs the column records the 0xFFFF invalid sentinel it has always had. Zero transactions, zero mutex takes, zero contention. The column's contract is unchanged - same field, same size, same scaling, same 0xFFFF-means-invalid semantics - and the mode-2 heading mirror below is unaffected because in that mode getRtmHeading() reads live at 10 Hz, so the cached value is never more than ~100 ms behind what the controller actually used. No record-size change, no LOG_FILE_FORMAT_VER bump, no confStruct change: sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-10-03 - I2C ENABLE-SWAP STARVATION, STEP 2 of 4 - THE INSTRUMENT (see BREmote_V2_Rx.h, PWM.ino, System.ino): fillLevel4Diag() ORs two 4-bit DELTAS of the enable-swap failure counters into bits 23-30 of the fm_gate_flags word it already copies - how many swaps lost the mutex since the previous row, per channel, saturating at 15. NO NEW COLUMN AND NO NEW BYTES, deliberately: the project's own rule beside LOG_FILE_FORMAT_VER says appending to the BASE record forces a format bump, and it would move every field above byte 62 and break offsetof(VescLogDataL5, rider_lat) == 90, making EVERY EXISTING LOG FILE undecodable. Riding in the existing u32 costs nothing, needs no bump (format stays 2), covers levels 4 AND 5, and leaves record sizes at 62 / 90 / 112. The delta is computed against loggerTask-LOCAL statics, so this task reads the PWM task's two volatiles and writes only its own - the single-writer contract that makes the FmLogSnapshot hand-off safe is preserved exactly. The cur < prev case (?diagz zeroing the counters mid-session) is handled rather than wrapping. Strictly additive and masked to its own eight bits: no FM gate verdict, no other column, no control path and no record size is touched. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-10-01 - M-2 FIX, part 3 of 4 (see BREmote_V2_Rx.h, PWM.ino, System.ino): convertToLogData() fills the two motor-gate columns appended to the BASE record, VescLogData (59 -> 62 B, so VescLogDataL4 is 90 B and VescLogDataL5 112 B): ctrl_pkt_age_ms = millis() - last_control_packet, capped at 0xFFFE, and motor_gate_open = the calcPWM() observer g_motor_gate_open (PWM.ino), the same route motor0_cmd / motor1_cmd take. THE BASE RECORD ON PURPOSE: the tiers are cumulative and levels 0-3 all record as level 3, so these two columns appear at EVERY log level 0 through 5 - there is no stored log_level that could miss them, and an incident is never re-runnable at a higher level. The gate is copied, never recomputed: this file already holds one CRITICAL MAINTENANCE inline duplicate (the heading ladder) and must not acquire a second. OLD LOGS: a pre-flash file cannot be decoded by this build - record_size says how far to step but cannot express that every block above byte 59 moved 3 bytes - so LOG_FILE_FORMAT_VER went 1 -> 2 and the format_ver test already in this file's download guard refuses them in plain English instead of mis-decoding them. Download anything worth keeping BEFORE flashing. ?download's record-size ceiling and the record buffers are sizeof()-derived and follow automatically. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-25 - MANUAL PIVOT ASSIST (log only): fillLevel4Diag() ORs the assist's depth, 0-15, into bits 19-22 of the fm_gate_flags word it already copies - one 4-bit field, no new column, no record-size change, so every existing 87 B / 109 B log file and both readers are unaffected. The value comes from the calcPWM() observer g_pivot_assist_q4 (PWM.ino), the same route motor0_cmd / motor1_cmd take, because the assist's controller is the 100 Hz PWM task and g_fm_log_snapshot must keep its single loop-task writer. Strictly additive and masked to its own four bits: no FM gate verdict, no other column and no control path is touched. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - DEEP LOG level 5: loggerTask() builds a VescLogDataL5 (109 B) when the file was created at level 5 - convertToLogData() for the base, ONE snapshot copy per row (logTakeFmSnapshot(), one critical section) shared by fillLevel4Diag() and the new fillLevel5Extra(), so a row never mixes two ticks between its level-4 and level-5 columns. Record buffers and ?download's record-size ceiling are sizeof(VescLogDataL5) now; the header/row tiers stay offset-selected. AUX button and logger_en semantics unchanged. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - DEEP LOG (B)+(C): fillLevel4Diag() fills the three fields appended to VescLogDataL4 (83 -> 87 B): fm_rider_raw_dx10 from the snapshot (RTMState.ino publishes fm_rider_raw_kmh x 10, 0xFFFF = unknown), motor0_cmd / motor1_cmd from the two calcPWM() observers g_motor0_cmd / g_motor1_cmd (PWM.ino). ?download's record-size ceiling is sizeof(VescLogDataL4) as before, so it follows the bump; the header/row tiers are chosen by block offset in BREmote_V2_Rx.h, so 83 B files written since 2026-09-17 still print their own 48 columns. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - DEEP LOG (A): fillLevel4Diag() copies the snapshot's return_reason into the byte that was fm_pad (now VescLogDataL4.fm_return_reason) - the sticky FmReturnReason latch, published by RTMState.ino every tick. Record size unchanged at 83; the two new gate bits (17 aligning, 18 boost) ride inside fm_gate_flags, which was already copied. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-17 - DEEP-LOG FM AUDIT COLUMNS (P0-g): fillLevel4Diag() now also copies g_fm_log_snapshot — filled by runFmLoop() once per tick — into the 18-byte Follow-Me block appended to VescLogDataL4 (65 -> 83 B). The copy runs inside taskENTER_CRITICAL(&g_fm_log_mux) so a row can never mix two ticks; the logger recomputes NO gate. ?download picks the CSV header by the file's own record_size (logCsvHeaderFor), so 65 B level-4 files written before this change still print their 35 columns. No confStruct change, sizeof stays 192, SW_VERSION stays 35, no control path touched.
// V2.5-Evo - 2026-08-17 - CRITICAL MAINTENANCE contract honoured again, same day, because the controller moved again (log mirror only): a standing heading-disagreement latch no longer just withdraws the compass, it drops the WHOLE ladder to the mode-0 path — GPS course only — so getRtmHeading() now returns NONE below rtm_cog_min_speed_kmh instead of serving a held COG, and returns NONE in mode 2 instead of serving the live compass. Two branches of this duplicate had to follow or the CSV would contradict the controller in exactly the state a rider would be reporting: (1) the COG-HOLD branch is now gated on !headingDisagreeLatched(), so a degraded tick logs src 0 / conf 0 rather than claiming a held GPS course the controller did not serve; (2) the mode-2 branch is gated the same way, so a diagnostic-mode session whose compass has been proven wrong logs NONE rather than src 3 / conf 2 COMPASS_LIVE. The compass-snapshot branch keeps the gate it was given this morning. Everything else about the mirror is unchanged, and the ORDER still matches the controller exactly: disagreement veto, live COG, held COG, compass snapshot. STRICTLY READ-ONLY: two more calls to the same read-only accessor from loggerTask, no controller state written, no timing changed. NO new column, no new rtm_source value, no record-size change — existing logs stay parseable. No confStruct change, no VescLogData change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-08-17 - CRITICAL MAINTENANCE contract honoured again (log mirror only): convertToLogData()'s duplicate heading ladder now reads the heading-disagreement LATCH, not just the per-tick verdict. getRtmHeading() withdraws the compass fallback entirely once the compass has been caught disagreeing with GPS course for kHeadingDisagreeMs and returns NONE, but this mirror kept falling through to the compass branch and logging rtm_source = 2 / rtm_confidence = 2 on those ticks — the CSV asserted a compass heading at MEDIUM confidence for ticks on which the controller had refused the compass and was holding straight. That was unreachable in an RTM run until the latch clears became edge-triggered; it is reachable now, so the log would actively lie about the heading source in exactly the new refusal case a rider will report. The compass-snapshot branch is gated on !headingDisagreeLatched(), positioned exactly where the controller checks it — BELOW the last-good-COG hold (a held COG is still logged src 1 / conf 2 while the fault stands, since a held GPS course is not the sensor under suspicion) and ABOVE the compass fallback — so a gated tick falls out of the else-if chain as src 0 / conf 0 = NONE, the pair getRtmHeading() actually returns. Strictly READ-ONLY: one call to a read-only accessor from loggerTask, no controller state written, no timing changed. NO new column, no new rtm_source value, no record-size change — existing logs stay parseable. No confStruct change, no VescLogData change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-08-16 - MID-RUN ABORT for ?download and ?deleteallogs, plus the MISSING WATCHDOG FEED in deleteAllLogFiles(). Both commands now ask rxAbortIfEngaged() inside their per-item loop, so an RTM/FM engagement that begins AFTER the command started stops it instead of freezing every safety gate for the rest of it — on ?download that was MINUTES (the code's own note records ~3 min for a ~350 kB file), which is by far the largest blind spot of any command on this board. ?download checks at the RECORD boundary, so no half-formatted row reaches the wire, closes the file on the way out, and prints a DIFFERENT end-of-transfer marker: "=== END CSV DATA ===" is never printed after an abort, because every reader treats that line as "the whole file arrived" and printing it over a truncated stream would silently pass a partial log off as complete; the abort marker names itself and carries the record count actually sent. ?deleteallogs checks at the WHOLE-FILE boundary before each SPIFFS.remove(), so every file is either fully deleted or fully untouched, and it now reports how many were ACTUALLY deleted instead of claiming completion over a partial run. Separately, its loop gained the watchdog feed it never had: initWatchdog() now arms the 3000 ms panic WDT on the first boot after a version bump where it previously did not, and on a full SPIFFS the garbage collection each remove() triggers can walk the loop past the timeout and panic-reboot mid-delete. The feed is gated on g_wdt_active, the same guard PWM.ino and Radio.ino use. NO change to the log file format, record layout or column set — existing logs stay parseable — and no change to either command's behaviour or timing when it runs to completion. No confStruct change, no VescLogData change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-08-16 - CRITICAL MAINTENANCE contract honoured (log mirror only; no new column, no new rtm_source value, no record-size change, no control path touched): the 2026-08-16 COG-hold change was mirrored into convertToLogData()'s STAGE 2 guards but NOT into the hold itself, so on every tick the controller was serving the last good COG (confidence 2, up to kCogHoldMs after cog_valid drops) the log recorded COMPASS_SNAPSHOT or NONE instead — the rtm_source / rtm_confidence columns misdescribed the exact source transitions these logs are being read to diagnose. The duplicate ladder now re-serves cog_last_good_deg / cog_last_good_ms in the same position and on the same terms the controller does (after the disagreement veto and the live-COG branch, gated on mode 1 and !cog_frozen_moving, ahead of the compass fallback) and logs it as src 1 / conf 2 — GPS_COG at MEDIUM, the pair getRtmHeading() actually returns, so held and live COG stay distinguishable without touching the CSV format. Strictly READ-ONLY: this mirror runs in loggerTask and writes no controller state, no timing, nothing. The 5 s escalation latch is still deliberately not read here. No confStruct change, no VescLogData change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-07-25 - STAGE 2 (log mirror only, no new column, no record-size change): the inline getRtmHeading() duplicate in convertToLogData() now applies the same two heading-trust guards RTMState.ino gained — guard 1 (COG rejected when its VALUE has been frozen longer than kRtmCogFrozenMs while gps_last_speed_kmh >= rtm_cog_min_speed_kmh, and no compass promotion in that state) and guard 2 (per-tick COG-vs-compass-snapshot disagreement beyond kHeadingDisagreeDeg = no source at all). Without this, rtm_source/rtm_confidence would keep logging "GPS COG, HIGH" for ticks where the controller was actually holding straight, which is precisely the blindness Stage 0 was built to end. The 5 s escalation latch is deliberately NOT read here: this runs in loggerTask and the mirror stays side-effect-free and per-tick. No confStruct change, no VescLogData change, sizeof stays 184, SW_VERSION stays 34, no control path touched.
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

// V2.5-Evo - 2026-08-16 - the shared "has RTM/FM engaged while this command was running?" test.
// Defined in System.ino beside rxRefuseIfEngaged(), which asks the same question at dispatch
// time. Declared here explicitly rather than leaning on the sketch's generated prototypes, so
// the dependency between these two files is written down where a reader will see it.
// Used by the two long-running log commands below, ?download and ?deleteallogs.
extern bool rxAbortIfEngaged(const char *what);

// V2.5-Evo - 2026-08-16 - true only once initWatchdog() has actually SUBSCRIBED the loop task
// to the task WDT (defined in GPS.ino). esp_task_wdt_reset() logs an error on every call from
// an unsubscribed task, so feeds are gated on this — the same guard PWM.ino and Radio.ino use.
extern volatile bool g_wdt_active;

// V2.5-Evo - 2026-08-17 - the heading-disagreement latch, read by the log mirror in
// convertToLogData() so the CSV cannot claim a compass heading the controller has refused.
// Defined in RTMState.ino, which Arduino concatenates AFTER this file, so it needs a
// declaration here — the same reason cog_last_good_deg is declared ahead of its use below.
// NOT 'extern': the accessor is a file-scope 'static' in RTMState.ino (internal linkage), so
// the declaration that names it here has to be 'static' too. That is enough, because Arduino
// compiles every .ino in the sketch as ONE translation unit — this is the same forward
// declaration RTMState.ino already makes of it ~240 lines above its own definition, just
// placed earlier in the same unit. It is declared at file scope rather than inside
// convertToLogData() with the extern block because C++ forbids 'static' on a block-scope
// function declaration.
// READ-ONLY, and it can only ever be read: the accessor returns the flag and cannot set,
// clear or age it, which is exactly why the mirror is allowed to consult it at all.
static bool headingDisagreeLatched();

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
    // V2.5-Evo - 2026-10-03 - STEP 4: getCompassHeading() is NO LONGER CALLED FROM THIS TASK. The
    // two cache globals below replace it; they are defined in Compass.ino, which the Arduino build
    // concatenates ahead of this file (the same dependency the gps_* externs above rely on, written
    // out for the same reason - so a reader sees it where they will look for it).
    extern float                 compass_live_cache_deg;   // last heading getCompassHeading() computed, deg; -1.0 = none yet
    extern unsigned long         compass_live_cache_ms;    // millis() of that computation; 0 = never
    // V2.5-Evo - 2026-08-16 - the last-good COG the controller holds across a short COG dropout
    // (RTMState.ino, kCogHoldMs). READ-ONLY here: the logger mirrors the controller's choice, it
    // never participates in making it, so nothing in this file may ever assign to these two.
    extern float                 cog_last_good_deg;   // last COG accepted at HIGH confidence, deg; -1.0 = none yet
    extern unsigned long         cog_last_good_ms;    // millis() of that acceptance; 0 = none yet

    // Simple state reads
    data.thr_received_log       = thr_received;
    data.rtm_rx_active_log      = rtm_rx_active.load() ? 1 : 0;
    data.rtm_steer_override_log = rtm_steer_override.load();
    data.gps_phase_b_ok_log     = gps_phase_b_ok ? 1 : 0;
    data.effective_steer_log    = g_effective_steer;   // FM triage: steering byte actually applied by calcPWM()

    // Live compass heading × 10 (0xFFFF = invalid/uncalibrated)
    // ============================================================
    // V2.5-Evo - 2026-10-03 - STEP 4: THIS LINE USED TO CALL getCompassHeading(). IT NO LONGER
    // ISSUES ANY I2C TRAFFIC AT ALL.
    //
    // WHAT WAS WRONG: getCompassHeading() -> readCompassRaw() -> two chained Wire transactions
    // inside ONE i2cMutex hold taken with portMAX_DELAY, performed on EVERY log record - ~2.89 Hz
    // at the default rate - from loggerTask, with no throttle gate, no rtm_use_compass gate and no
    // heading-latch gate. So whenever logging was on, the RX hit the compass about three times a
    // second AT ANY THROTTLE, including full throttle with a rider on the rope, competing for the
    // same mutex as the 100 Hz enable swap that decides which of the two motors receives the single
    // PPM output. A swap that loses that race starves one channel completely. loggerTask is also
    // NOT registered with the task watchdog, so a hold that never returned would not have panicked
    // anything either. This was the compass starver nobody had found.
    //
    // THE FIX IS A CACHE, NOT AN ARBITRATION. The logger and the compass were never two competing
    // bus users - the logger's I2C traffic WAS a compass read - so there is nothing to arbitrate.
    // getCompassHeading() publishes its last successful result in Compass.ino; recording a heading
    // no longer requires fetching one. Zero transactions, zero mutex takes, zero contention with
    // the motor path, and no behaviour anywhere else changes because no control path reads this.
    //
    // THE AGE TEST IS THE HONEST PART. Past kCompassLiveCacheMaxAgeMs the column records its
    // existing 0xFFFF invalid sentinel rather than a value that no longer describes this row. That
    // is not a loss: at real throttle there IS no trustworthy live compass heading - the snapshot
    // path is gated off above thr_received 25 precisely because motor current biases this part by
    // 100 degrees or more - so a fresh read on the motor path was buying a number that was already
    // known to be wrong, at the price of starving a motor. In the one mode where the live value is
    // genuinely load-bearing (rtm_use_compass 2, where getRtmHeading() reads live at 10 Hz) the
    // cache is never more than ~100 ms old, so the mode-2 mirror below logs exactly what the
    // controller used.
    //
    // THE COLUMN'S CONTRACT IS UNCHANGED: same field, same size, same scaling, same
    // 0xFFFF-means-invalid semantics. No record-size change and no format bump.
    // ============================================================
    float live_compass = -1.0f;
    if (compass_live_cache_ms != 0 &&
        (millis() - compass_live_cache_ms) <= kCompassLiveCacheMaxAgeMs) {
      live_compass = compass_live_cache_deg;
    }
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
      // V2.5-Evo - 2026-08-17 - mirror of the mode-2 refusal added to getRtmHeading(): mode 2 has no
      // GPS course to degrade to, so a compass that has been PROVEN to disagree is withheld outright
      // and the controller returns NONE. Without this gate the log would record src 3 / conf 2
      // (COMPASS_LIVE, MEDIUM) for ticks on which the controller was holding straight with no
      // heading at all. Read-only, like every other test in this mirror.
      if (live_compass >= 0.0f && !headingDisagreeLatched()) {
        src    = 3;     // COMPASS_LIVE
        conf   = 2;     // MEDIUM
        chosen = live_compass;
      }
    } else {
      // Modes 0 and 1: GPS COG primary
      // V2.5-Evo - 2026-07-25 - STAGE 2 mirror of the two heading-trust guards added to
      // getRtmHeading(). Kept in lockstep on purpose: if the log said "GPS COG, HIGH" on a tick
      // where the controller had rejected COG and was holding straight, the log would be actively
      // misleading about the exact failure Stage 0 was built to expose.
      bool cog_captured = (gps_last_course_ms > 0) && (gps_last_course_deg >= 0.0f);
      bool cog_fresh_ts = cog_captured && ((now_ms - gps_last_course_ms) < 1500UL);
      bool cog_moving   = (gps_last_speed_kmh >= (float)cog_min_speed);

      // GUARD 1 mirror: a COG whose VALUE has not moved for kRtmCogFrozenMs while the buggy is
      // moving is a repeated number, not a heading. Speed-gated, so a stationary buggy reporting a
      // constant (and correct) course is never flagged.
      bool cog_frozen_moving = false;
      if (cog_moving && cog_fresh_ts) {
        unsigned long cog_change_ms = (unsigned long)g_diag_cog_change_ms;
        cog_frozen_moving = (cog_change_ms != 0) &&
                            ((now_ms - cog_change_ms) >= (unsigned long)kRtmCogFrozenMs);
      }

      bool cog_valid = cog_fresh_ts && cog_moving && !cog_frozen_moving;

      // GUARD 2 mirror (per-tick verdict only). The 5 s escalation latch lives in RTMState.ino and
      // is NOT read here: this function runs in loggerTask, so the mirror must stay side-effect-free
      // and must not depend on loop-task timing. A logged NONE therefore reflects the disagreement
      // that was measurable at this instant, which is what rtm_source/rtm_confidence describe.
      bool compare_possible = (mode == 1) && cog_valid &&
                              (compass_snapshot_heading >= 0.0f) && (compass_snapshot_ms > 0) &&
                              ((now_ms - compass_snapshot_ms) < (unsigned long)kHeadingCompareSnapMs);
      bool disagree_now = false;
      if (compare_possible) {
        float d = gps_last_course_deg - compass_snapshot_heading;   // wrap-correct across 0/360
        while (d >  180.0f) d -= 360.0f;
        while (d < -180.0f) d += 360.0f;
        disagree_now = (fabsf(d) > kHeadingDisagreeDeg);
      }

      if (disagree_now) {
        // Both sources distrusted — src/conf stay 0, matching getRtmHeading() returning NONE.
      } else if (cog_valid) {
        src    = 1;     // GPS_COG
        conf   = 3;     // HIGH
        chosen = gps_last_course_deg;
      } else if (mode == 1 && !cog_frozen_moving &&
                 cog_last_good_deg >= 0.0f && cog_last_good_ms > 0 &&
                 ((now_ms - cog_last_good_ms) < (unsigned long)kCogHoldMs)) {
        // V2.5-Evo - 2026-08-17 - THE LATCH GATE IS DELIBERATELY NOT HERE, and the note that used to
        // sit here claiming otherwise was written against a controller that has since been corrected.
        // The disagreement latch is evidence about the COMPASS. This branch serves a last-good GPS
        // COURSE, which the latch says nothing about — so withdrawing it on compass evidence was
        // outside the guard's charter, and the controller no longer does it: its fault check sits
        // BELOW this hold and above the compass fallback. Mirror that exactly. Gating here would log
        // src 0 / conf 0 for ticks the controller genuinely spent steering on a held course.
        // The two branches that DO carry the gate — mode 2 and the compass fallback — both consume
        // the compass, which is the thing the latch has evidence against.
        // COG-HOLD mirror (V2.5-Evo - 2026-08-16). WHAT WAS WRONG: the 2026-08-16 controller change
        // added a step to getRtmHeading() that this duplicate never got — before falling back to the
        // compass it re-serves the LAST GOOD COG for up to kCogHoldMs at confidence 2, which is what
        // stops RTM flapping to an EMI-biased compass every time speed dips under the COG floor.
        // Because it was missing here, every tick of every hold logged COMPASS_SNAPSHOT or NONE
        // while the controller was in fact steering on held COG: the two columns we are reading
        // field logs to understand described the wrong source at exactly the transitions in question.
        // WHY src=1/conf=2 IS THE HONEST PAIR, with no new column and no new enum value: the value
        // being steered on IS a GPS course (src 1), and confidence 2 is precisely what
        // getRtmHeading() returns for it — a real measurement, but a stale one. A live COG still
        // logs 1/3, so held and live remain distinguishable in the CSV without changing its format.
        // ORDER MATTERS and mirrors the controller exactly: after the disagreement veto and the live
        // COG branch, gated on mode 1 (mode 0 returns NONE before the hold, mode 2 never reaches
        // here) and on !cog_frozen_moving (a frozen COG is refused before the hold is consulted),
        // and ahead of the compass fallback.
        // STRICTLY SIDE-EFFECT-FREE: three reads and nothing else. This runs in loggerTask; the two
        // globals are written only by getRtmHeading() in the loop task, and this mirror must never
        // write them or any other controller state, nor change any timing.
        src    = 1;     // GPS_COG — held, not live
        conf   = 2;     // MEDIUM — real course, but stale
        chosen = cog_last_good_deg;
      } else if (mode == 1 && !cog_frozen_moving && !headingDisagreeLatched()) {
        // Hybrid: fall back to compass snapshot — but NOT when guard 1 has just proven the COG
        // frozen while moving. In that state one source is provably dead and the other cannot be
        // cross-checked, so the controller holds straight instead of promoting the survivor, and
        // the log must say the same thing.
        // V2.5-Evo - 2026-08-17 - ...and NOT while the heading-disagreement latch stands.
        // WHAT WAS WRONG: this mirror read only the per-tick verdict (disagree_now) and never the
        // SUSTAINED fault. getRtmHeading() withdraws the compass fallback outright once the
        // compass has been caught disagreeing with GPS course for kHeadingDisagreeMs, and returns
        // NONE. On exactly those ticks this duplicate still fell through to here and wrote
        // rtm_source = 2 / rtm_confidence = 2 — the CSV reported a compass heading at MEDIUM
        // confidence for ticks on which the controller had refused the compass and was holding
        // straight. The log contradicted the controller in precisely the refusal a rider would be
        // reporting, which is the one case these two columns exist to explain.
        // WHY IT MATTERS NOW: the clears became edge-triggered, so the latch can stand across an
        // engagement for the first time — this branch is reachable during a real run, not just in
        // theory.
        // POSITION MIRRORS THE CONTROLLER EXACTLY. V2.5-Evo - 2026-08-17: that position moved. The
        // controller now leaves at the mode-0 return, which is ABOVE both the held-COG hold and
        // this compass fallback, so a standing fault withdraws BOTH — the same gate therefore also
        // appears on the hold branch above, and the two branches fall out of the else-if chain
        // together as src 0 / conf 0 = NONE, which is what getRtmHeading() returns. A live COG is
        // untouched by any of it and still logs src 1 / conf 3.
        // SIDE-EFFECT-FREE: one read of a read-only accessor. This runs in loggerTask and writes
        // no controller state and changes no timing. No new column, no new rtm_source value, no
        // record-size change — existing logs stay parseable.
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
      // Mode 0 with no valid COG, or a frozen-while-moving COG in any mode: src/conf stay 0 (hold straight)
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

  // V2.5-Evo - 2026-10-01 - M-2: THE MOTOR GATE, the one thing no instrument on this board could
  // see. Everything else in this record that touches the link keys on last_packet ("is the remote
  // alive?") - including the rssi_dbm / snr_dx10 N/A sentinels immediately above - while the motor
  // gate in generatePWM() keys on last_control_packet ("do I have a fresh throttle command?"). The
  // two diverge exactly when it matters: through a meta-packet-only stretch the link flag, the RSSI
  // and the BIND LED all read healthy while the motors are not being driven at all. On 2026-09-29
  // that cost a half-day, with remote_error sitting at 0 through two confirmed outages of 5.4 s and
  // 8.75 s. These two columns close it.
  {
    // How stale the last THROTTLE command was on this row. Unsigned subtraction is correct across a
    // millis() wrap. Capped at 0xFFFE so the column cannot wrap its own u16 encoding - anything past
    // ~65.5 s of control silence is one state, "long gone", and the gate has been shut for all of it.
    // No N/A sentinel: this is always a real measurement. Before the first control packet it reads as
    // uptime, which is exactly what the gate itself sees at that moment.
    unsigned long ctrl_age_ms = millis() - last_control_packet;
    data.ctrl_pkt_age_ms = (uint16_t)((ctrl_age_ms > 0xFFFEUL) ? 0xFFFEUL : ctrl_age_ms);

    // The gate's OWN verdict, straight from the calcPWM() observer (g_motor_gate_open, PWM.ino) -
    // a single-byte volatile written at 100 Hz on the generatePWM task, read atomically here, the
    // identical route motor0_cmd / motor1_cmd already take. COPIED, NOT RECOMPUTED: the gate
    // expression exists exactly twice (the gate itself and its documented mirror in calcPWM) and
    // this file must not become a third site to keep in step. The observer also carries PWM_active,
    // so a 0 with a SMALL age means the PWM path is down rather than the link.
    data.motor_gate_open = g_motor_gate_open ? 1 : 0;
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
//          s   - the controller's snapshot for this row (logTakeFmSnapshot(), V2.5-Evo - 2026-09-19:
//                taken ONCE by the caller so the level-4 and level-5 columns of a row come from the
//                same tick)
// Outputs: none (rec is filled in place)
// Side effects: resets g_diag_loop_max_us_log to 0 — that field is defined as "worst loop since
//   the PREVIOUS record", so consuming it here is what makes consecutive records comparable.
//   ?diag keeps its own separate peak (g_diag_loop_max_us) so the two never steal from each other.
//
// Sentinels: cog_frozen_s == 255 means no COG value has ever been captured this session (NOT
//   "0 seconds"); 254 means 254 seconds or longer. mux_err_cnt saturates at 0xFFFE.
// ============================================================
// V2.5-Evo - 2026-09-19 - logTakeFmSnapshot - copy the controller's snapshot out, once per row
// Inputs: none. Outputs: a copy of g_fm_log_snapshot. Side effects: one short critical section
// (taskENTER_CRITICAL) around a struct copy - no I/O, no computation inside it. Was inline in
// fillLevel4Diag(); moved out so a level-5 row takes the snapshot exactly once.
// ============================================================
static FmLogSnapshot logTakeFmSnapshot()
{
  FmLogSnapshot s;
  taskENTER_CRITICAL(&g_fm_log_mux);
  s = g_fm_log_snapshot;
  taskEXIT_CRITICAL(&g_fm_log_mux);
  return s;
}

static void fillLevel4Diag(VescLogDataL4 &rec, const FmLogSnapshot &s)
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

  // V2.5-Evo - 2026-09-17 - P0-g: the Follow-Me audit block. A straight COPY of the snapshot the
  // controller published on its last 10 Hz tick — the logger recomputes nothing. (V2.5-Evo -
  // 2026-09-19: the copy itself now happens in logTakeFmSnapshot(), once per row, in the caller.)
  rec.fm_gate_flags       = s.gate_flags;
  rec.fm_distance_dx10    = s.distance_dx10;
  rec.fm_d_engage_dx10    = s.d_engage_dx10;
  rec.fm_rider_speed_dx10 = s.rider_speed_dx10;
  rec.fm_sep_fix_count    = s.sep_fix_count;
  rec.fm_mode             = s.mode;
  rec.fm_state            = s.state;
  rec.fm_block_reason     = s.block_reason;
  rec.fm_throttle_cap     = s.throttle_cap;
  rec.fm_station_deg_x10  = s.station_deg_x10;
  rec.fm_return_reason    = s.return_reason;   // DEEP LOG (A) 2026-09-19: was the always-0 pad; sticky, read it where fm_state changes
  rec.fm_rider_raw_dx10   = s.rider_raw_dx10;  // DEEP LOG (B) 2026-09-19: raw rider speed x 10, 0xFFFF = unknown

  // V2.5-Evo - 2026-09-19 - DEEP LOG (C): the two mixer outputs, straight from the calcPWM() observers.
  // Single-byte volatiles written at 100 Hz on the generatePWM task; a read here is atomic on this
  // core, and the pair may straddle one 10 ms pass at worst - fine for a 3-5 Hz diagnostic row.
  rec.motor0_cmd          = g_motor0_cmd;
  rec.motor1_cmd          = g_motor1_cmd;

  // V2.5-Evo - 2026-09-25 - MANUAL PIVOT ASSIST: its depth rides in bits 19-22 of the gate word
  // that was just copied above, so the assist adds NO column and the record size is unchanged.
  // This is an OR into the copied value, not a recomputation: the controller for those four bits is
  // calcPWM(), not runFmLoop(), so the value cannot come through g_fm_log_snapshot (one writer, the
  // loop task) without giving that struct a second writer on another task. It comes through the same
  // route motor0_cmd / motor1_cmd just used - a single-byte volatile observer written by the PWM task
  // - and the mask keeps it strictly inside its own four bits, so no FM gate verdict can be altered
  // by it. The assist is never active while Follow-Me or RTM is, so these bits and bits 0-18 are
  // never both meaningful on the same row.
  rec.fm_gate_flags |= ((uint32_t)(g_pivot_assist_q4 & 0x0FU)) << FM_LOG_GATE_PIVOT_ASSIST_SHIFT;

  // V2.5-Evo - 2026-10-03 - I2C ENABLE-SWAP FAILURES: two more 4-bit fields in bits 23-30 of the
  // same gate word, on the same mechanism the pivot assist just used. ZERO added bytes, no
  // record-size change, no LOG_FILE_FORMAT_VER bump, and every log file written before today still
  // parses - which is the whole reason they ride in here instead of becoming a column. Appending to
  // the BASE record would move every field above byte 62 and break the
  // offsetof(VescLogDataL5, rider_lat) == 90 assertion, forcing format 2 -> 3 and making every
  // existing log undecodable; see the restated rule beside LOG_FILE_FORMAT_VER.
  //
  // A DELTA, NOT A TOTAL, and that is what the local snapshot is for. g_swap_fail_ch0/ch1 are
  // monotonic session counters owned by the generatePWM task. Subtracting a loggerTask-LOCAL
  // snapshot taken at the previous row keeps the single-writer contract intact: this task reads
  // those two volatiles and writes only its own statics, never the PWM task's variables. That is
  // the same discipline the FmLogSnapshot hand-off uses and the reason this can be added safely.
  // Four bits means "how bad was this row" (saturating at 15), while the exact cumulative figures
  // live on ?diag. At 100 Hz a 333 ms row holds at most ~33 swaps, so 15 does not span a total
  // outage - a row reading 15/15 means "pinned", which is all the CSV has to say.
  //
  // THE cur < prev CASE IS REAL AND HANDLED: ?diagz zeroes the counters mid-session, so the next
  // row would otherwise compute a huge wrapped delta. When that happens the current value IS the
  // count since the reset, so it is used directly.
  {
    static uint16_t prev_swap_fail_ch0 = 0;
    static uint16_t prev_swap_fail_ch1 = 0;
    // V2.5-Evo - 2026-10-03 - STEP 5 (Rex L-3): SEED THE SNAPSHOT ON THE FIRST CALL. Starting from 0
    // made the FIRST level-4 row of a session report the whole boot's accumulated failure count as
    // if it had all happened inside that one ~333 ms row - and since the nibbles saturate at 15, a
    // board that had seen any swap failures before logging started wrote a first row reading 15/15,
    // i.e. "pinned", from a perfectly healthy bus. Seeded HERE, inside loggerTask, so the
    // single-writer contract described above is untouched: this task still writes only its own
    // statics and nothing outside loggerTask touches them. RESIDUAL, stated rather than hidden:
    // these statics outlive a logger stop/start within one boot, so the first row after the AUX
    // button re-enables logging still reports the delta accumulated while logging was off. Closing
    // that needs a cross-task reset from startLog(), which is not worth breaking the single-writer
    // rule for one cosmetic row.
    static bool     prev_swap_seeded   = false;

    const uint16_t cur0 = g_swap_fail_ch0;
    const uint16_t cur1 = g_swap_fail_ch1;
    if(!prev_swap_seeded) { prev_swap_fail_ch0 = cur0; prev_swap_fail_ch1 = cur1; prev_swap_seeded = true; }
    const uint16_t d0   = (cur0 >= prev_swap_fail_ch0) ? (uint16_t)(cur0 - prev_swap_fail_ch0) : cur0;
    const uint16_t d1   = (cur1 >= prev_swap_fail_ch1) ? (uint16_t)(cur1 - prev_swap_fail_ch1) : cur1;
    prev_swap_fail_ch0  = cur0;
    prev_swap_fail_ch1  = cur1;

    rec.fm_gate_flags |= ((uint32_t)((d0 > 15U) ? 15U : d0)) << FM_LOG_GATE_SWAP_FAIL_CH0_SHIFT;
    rec.fm_gate_flags |= ((uint32_t)((d1 > 15U) ? 15U : d1)) << FM_LOG_GATE_SWAP_FAIL_CH1_SHIFT;
  }
}

// ============================================================
// V2.5-Evo - 2026-09-19 - fillLevel5Extra - add the level-5 ("Everything") block to a log record
//
// What it does: copies the level-5 fields of the controller's snapshot into the 22-byte block that
// VescLogDataL5 appends after the level-4 record. Straight copies - every value was published by
// fmPublishLogSnapshot() (RTMState.ino) on the controller's last 10 Hz tick; the logger derives
// nothing. The two reserved bytes are written 0 here (the steer-takeover branch fills them).
// Inputs:  rec - a VescLogDataL5 whose .l4 has already been filled (convertToLogData + fillLevel4Diag)
//          s   - the same snapshot fillLevel4Diag() was given, so the row is one tick throughout
// Outputs: none (rec is filled in place). Side effects: none - no I/O, no globals written.
// ============================================================
static void fillLevel5Extra(VescLogDataL5 &rec, const FmLogSnapshot &s)
{
  rec.rider_lat               = s.l5_rider_lat;
  rec.rider_lng               = s.l5_rider_lng;
  rec.rider_fix_seq           = s.l5_rider_fix_seq;
  rec.rider_fix_age_div10     = s.l5_rider_fix_age_div10;
  rec.rtm_approach_cap        = s.l5_rtm_approach_cap;
  rec.rtm_phase               = s.l5_rtm_phase;
  rec.align_cap               = s.l5_align_cap;
  rec.align_influence         = s.l5_align_influence;
  rec.mix_influence           = s.l5_mix_influence;
  rec.fm_return_override      = s.l5_fm_return_override;
  rec.fm_flags_sent           = s.l5_fm_flags_sent;
  rec.fm_keepalive_age_div100 = s.l5_fm_keepalive_age_div100;
  rec.l5_rsvd_takeover_active = 0;
  rec.l5_rsvd_takeover_end    = 0;
}

// ============================================================
// V2.5-Evo - 2026-10-06 - fillVesc2Block - add the VESC 2 block to a level-4 (or level-5) log record
// (LOG FORMAT 3: the block moved from the tail of VescLogDataL5 into VescLogDataL4, so a level-5 row passes
// its .l4 and both levels carry it.)
//
// What it does: copies VESC 2's last validated telemetry (vesc2, written by pollVesc2IfDue() in
// VESC.ino) into the 14-byte block at the tail of VescLogDataL4, in the same units and clamps the
// base record uses for VESC 1. FRESHNESS IS DECIDED HERE, at log time, against VESC 2's OWN age
// stamp: if VESC 2 has never answered this session, or its last reply is older than kVesc2StaleMs,
// every value field gets its "no data" sentinel instead of the old numbers - stale data is never
// written as live (project GPS/telemetry rule 1, by analogy). vesc2_age_ms is always the real age
// (0xFFFF only for "never"), so a reader can see WHY a row is N/A.
// Inputs:  rec - a VescLogDataL4 whose earlier blocks are already filled
// Outputs: none (rec is filled in place).
// Side effects: takes vescMutex for a struct copy (the same 50 ms bound convertToLogData() uses);
//   if the take fails the block is written as N/A rather than torn. No I/O, no globals written.
// ============================================================
static void fillVesc2Block(VescLogDataL4 &rec)
{
  // Start from "no data" in every field; only a fresh copy overwrites it.
  rec.vesc2_age_ms          = 0xFFFF;
  rec.vesc2_current_motor   = 0x7FFF;
  rec.vesc2_current_battery = 0x7FFF;
  rec.vesc2_duty_cycle      = 0x7F;
  rec.vesc2_voltage         = 0xFFFF;
  rec.vesc2_ERPM            = 0x7FFF;
  rec.vesc2_temp_mos        = 0x7F;
  rec.vesc2_temp_motor      = 0x7F;
  rec.vesc2_fault_code      = 0xFF;

  vesc2_struct v2;
  if (!(vescMutex && xSemaphoreTake(vescMutex, pdMS_TO_TICKS(50)) == pdTRUE)) return;
  v2 = vesc2;
  xSemaphoreGive(vescMutex);

  if (!v2.ever_ok) return;   // never answered: age stays 0xFFFF, values N/A

  const uint32_t age = millis() - v2.last_ok_ms;
  rec.vesc2_age_ms = (uint16_t)((age > 0xFFFEUL) ? 0xFFFEUL : age);
  if (age > kVesc2StaleMs) return;   // stale: real age logged, values N/A

  // Clamps keep every real value strictly inside its field and off its sentinel
  // (+-30000 < 0x7FFF, +-101 / +-120 < 0x7F, a fault code is never 0xFF).
  rec.vesc2_current_motor   = (int16_t)constrain(v2.motCur, -30000, 30000);
  rec.vesc2_current_battery = (int16_t)constrain(v2.batCur, -30000, 30000);
  rec.vesc2_duty_cycle      = (int8_t)constrain(v2.duty / 10, -101, 101);
  rec.vesc2_voltage         = (uint16_t)constrain((int32_t)v2.batVolt, 0, 0xFFFE);
  rec.vesc2_ERPM            = (int16_t)constrain(v2.erpm / 10, -30000, 30000);
  rec.vesc2_temp_mos        = (int8_t)constrain(v2.fetTemp / 10, -120, 120);
  rec.vesc2_temp_motor      = (int8_t)constrain(v2.motorTemp / 10, -120, 120);
  rec.vesc2_fault_code      = (v2.fault_code == 0xFF) ? 0xFE : v2.fault_code;
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
      // created at (active_log_level), not for whatever the config says right now. All tiers
      // are assembled into one byte buffer so there is still exactly ONE write path holding
      // fileMutex — the mutex block below is unchanged apart from taking a length instead of
      // a hardcoded sizeof().
      // ============================================================
      // V2.5-Evo - 2026-09-19 - level 5 added: the same shape one tier up. The controller snapshot
      // is taken ONCE per row and handed to both fill functions, so a level-5 row is one tick
      // throughout. The buffer is sized for the largest record this firmware writes (level 5).
      uint8_t  rec_buf[sizeof(VescLogDataL5)];
      uint16_t rec_len;
      if (active_log_level >= 5) {
        VescLogDataL5 logData5;
        logData5.l4.base = convertToLogData();
        const FmLogSnapshot snap = logTakeFmSnapshot();
        fillLevel4Diag(logData5.l4, snap);
        fillVesc2Block(logData5.l4);   // V2.5-Evo - 2026-10-06 - VESC 2 over CAN, with its own freshness test (in the level-4 record since log format 3)
        fillLevel5Extra(logData5, snap);
        memcpy(rec_buf, &logData5, sizeof(logData5));
        rec_len = (uint16_t)sizeof(logData5);
      } else if (active_log_level >= 4) {
        VescLogDataL4 logData4;
        logData4.base = convertToLogData();
        const FmLogSnapshot snap = logTakeFmSnapshot();
        fillLevel4Diag(logData4, snap);
        fillVesc2Block(logData4);      // V2.5-Evo - 2026-10-06 - LOG FORMAT 3: level 4 carries both VESCs (owner ruling)
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
  // V2.5-Evo - 2026-10-06 - LOG FORMAT 3: a file from OLDER firmware gets its own plain-English answer. Its bytes
  // are not garbage - they follow an older layout this firmware no longer decodes (a format-2 126 B record and a
  // format-3 126 B record are the same size with different contents), so nothing is printed rather than wrong
  // columns, and the rider is told what happened and where the file CAN be read.
  if (hdr.format_ver < LOG_FILE_FORMAT_VER) {
    file.close();
    Serial.printf("LOG: this log was recorded by older firmware (log format %u). This firmware writes log format %u\n",
                  (unsigned)hdr.format_ver, (unsigned)LOG_FILE_FORMAT_VER);
    Serial.println("LOG: and cannot turn an older file into CSV, so nothing was printed. The file is NOT damaged.");
    Serial.println("LOG: Logs should be downloaded BEFORE flashing new firmware. To read this one, flash the previous");
    Serial.println("LOG: firmware and download it, or decode the raw file on a PC with Tools/logreader/bremote_log.py.");
    return;
  }
  if (hdr.format_ver != LOG_FILE_FORMAT_VER ||
      hdr.record_size < (uint16_t)sizeof(VescLogData) ||
      hdr.record_size > (uint16_t)sizeof(VescLogDataL5)) {   // V2.5-Evo - 2026-09-19 - level 5 is the largest record now
    file.close();
    Serial.printf("LOG: unsupported log format (header version %u, %u bytes/record).\n",
                  (unsigned)hdr.format_ver, (unsigned)hdr.record_size);
    Serial.printf("LOG: this firmware reads header version %u with %u-%u bytes/record. Nothing printed.\n",
                  (unsigned)LOG_FILE_FORMAT_VER,
                  (unsigned)sizeof(VescLogData), (unsigned)sizeof(VescLogDataL5));
    return;
  }

  Serial.println("\n=== BEGIN CSV DATA ===");
  // V2.5-Evo - 2026-07-19 - FM triage: header updated to 28 fields (+effective_steer)
  // V2.5-Evo - 2026-07-24 - F9: header updated to 31 fields (+tx_distance_m, +rssi_dbm, +snr_db). N/A sentinels: distance -1.0, rssi -999, snr -99.0
  // V2.5-Evo - 2026-07-25 - STAGE 0: the column list lives ONCE in BREmote_V2_Rx.h and the WiFi
  // download path emits the same macro, so the two can no longer drift. The header printed must
  // match the level the file was actually RECORDED at (from its own header), not the level the
  // config happens to be set to now.
  // V2.5-Evo - 2026-09-17 - and by the file's RECORD SIZE too, so a 65 B level-4 file written
  // before the Follow-Me block gets its own 35-column header, not the 45-column one.
  Serial.println(logCsvHeaderFor(hdr.log_level, hdr.record_size));

  uint8_t  rec_buf[sizeof(VescLogDataL5)];   // V2.5-Evo - 2026-09-19 - sized for the largest record (level 5)
  char     row[LOG_CSV_ROW_BUF];
  uint16_t recordCount = 0;
  bool     aborted     = false;   // V2.5-Evo - 2026-08-16 - true = stopped by an RTM/FM engagement
  while (file.available()) {
    // V2.5-Evo - 2026-05-06 - FIX-LOGDL-2: feed WDT inside loop and yield to FreeRTOS.
    // Without these, files >~30KB cause WDT (3s timeout) to fire mid-download (Andres
    // confirmed crash at ~3 min / ~350KB on 050626_204204.log).
    esp_task_wdt_reset();

    // ============================================================
    // V2.5-Evo - 2026-08-16 - MID-RUN ABORT: stop if RTM or Follow-Me engages mid-transfer.
    //
    // This is the longest-running command on the RX — the note directly above records a
    // measured ~3 min for a ~350 kB file — so an engagement part-way through bought MINUTES in
    // which no safety gate was evaluated at all (no Gate 9 stop-distance hard stop, no Phase
    // A/B/C) while generatePWM carried on applying the last steering override and throttle cap.
    // The dispatch gate only closes the front door; rtm_rx_active is set from the radio task on
    // an arm packet and can turn true at any instant after this command was correctly allowed
    // to start on an idle buggy.
    //
    // Checked per record, at the RECORD BOUNDARY: cheap enough to be free here (two atomic
    // loads against the read + format + serial print that follow it), and leaving on a boundary
    // means a half-formatted row never reaches the wire.
    //
    // CLEANUP: this command only READS — no SPIFFS writes, no mux movement, no baud change — so
    // the only obligations are closing the file (the existing close below covers every exit) and
    // telling the receiving end that the stream is incomplete. See the marker after the loop.
    // ============================================================
    if (rxAbortIfEngaged("?download")) { aborted = true; break; }

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

  // ============================================================
  // V2.5-Evo - 2026-08-16 - END-OF-TRANSFER MARKER: a complete download and one that was
  // stopped part-way must NOT look the same on the wire.
  //
  // "=== END CSV DATA ===" is what every reader — a human scrolling the terminal, or the web
  // serial tool scraping between the BEGIN and END lines — treats as "the whole file arrived".
  // Printing it after an aborted stream would silently pass a truncated log off as a complete
  // one, and a partial session log that is believed to be whole is worse than no log: the gap
  // is invisible, so the missing records read as a period where nothing happened.
  //
  // So the abort path prints its OWN distinct terminator instead, naming itself and carrying
  // the number of records actually sent. Two lines, neither containing the normal marker, so
  // the difference is unmistakable at a glance and machine-detectable by an exact line match.
  // ============================================================
  if (aborted) {
    Serial.printf("=== CSV DATA TRUNCATED - ABORTED AFTER %u RECORDS ===\n", (unsigned)recordCount);
    Serial.println("=== THIS DOWNLOAD IS INCOMPLETE - re-run ?download once disarmed ===");
  } else {
    Serial.println("=== END CSV DATA ===");
  }
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
  bool aborted = false;   // V2.5-Evo - 2026-08-16 - true = stopped by an RTM/FM engagement
  while (file) {
    // ============================================================
    // V2.5-Evo - 2026-08-16 - FEED THE WATCHDOG. This loop had no feed at all.
    //
    // WHAT THE BUG IS: SPIFFS runs garbage collection on remove(), and on a nearly-full
    // filesystem holding several large logs that can take hundreds of milliseconds per file.
    // Enough files and the loop walks straight past the 3000 ms task-WDT timeout, which is
    // armed with trigger_panic = true — so the board PANIC-REBOOTS part-way through the delete
    // rather than finishing it.
    //
    // WHY IT MATTERS NOW: this was previously unreachable in practice because initWatchdog()
    // did not arm the WDT in the state this command was typically run in. It now arms on the
    // first boot after a version bump, so the loop below runs watched where it used to run
    // unwatched. The hazard did not change; its reachability did.
    //
    // Gated on g_wdt_active for the same reason PWM.ino and Radio.ino gate theirs: calling
    // esp_task_wdt_reset() from a task that is not subscribed logs an error on EVERY call, and
    // the RX has no display and no LED — the boot log is its only diagnostic surface.
    // ============================================================
    if (g_wdt_active) esp_task_wdt_reset();

    // ============================================================
    // V2.5-Evo - 2026-08-16 - MID-RUN ABORT: stop if RTM or Follow-Me engages mid-delete.
    //
    // Duration here scales with how many logs are stored, so on a full board this is the
    // difference between a moment and many seconds during which no safety gate is evaluated
    // while generatePWM keeps applying the last steering override and throttle cap.
    //
    // Checked at the TOP of the iteration, BEFORE this file's SPIFFS.remove(), so we always
    // leave on a WHOLE-FILE boundary: every file is either fully deleted or fully untouched,
    // never half-removed. A partial run is inherently partial, which is why the report below
    // states what was actually done instead of claiming completion.
    // ============================================================
    if (rxAbortIfEngaged("?deleteallogs")) { aborted = true; break; }

    String fname = String("/") + file.name();
    file = root.openNextFile();  // advance before remove
    if (!fname.endsWith(".log")) continue;
    if (logging_active && fname == currentLogFileName) { skipped++; continue; }
    SPIFFS.remove(fname);
    deleted++;
  }
  // V2.5-Evo - 2026-08-16 - report what ACTUALLY happened. A partial delete must not print the
  // same line as a complete one: the operator would otherwise be told every log was erased while
  // files are still on the board, and would go hunting a storage fault that does not exist.
  if (aborted) {
    Serial.printf("LOG: STOPPED PART-WAY — %d log file(s) deleted before the abort", deleted);
  } else {
    Serial.printf("LOG: deleted %d log file(s)", deleted);
  }
  if (skipped) Serial.printf(", skipped %d active", skipped);
  Serial.println();
  if (aborted) {
    Serial.println("LOG: log files REMAIN on the board — re-run ?deleteallogs once disarmed.");
  }
}

bool isLoggingActive() {
  return logging_active;
}