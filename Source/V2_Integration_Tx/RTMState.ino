// V3 - 2026-04-25 - P7: TX RTM and FM state machines.
// RTM: left-hold gesture → arm → squeeze(s) → active → cooldown → idle
// FM:  right-hold gesture → cycle FM mode 0→1→2→3→0 → send 0xF2 meta-packet
// V3 - 2026-04-27 - P8: setRtmArmed shows "rn" ×2 (static, 3s total); showFmMode shows F0-F3;
//   added setRtmDisarmed(); steer-exit gate in ACTIVE; rtm_max_runtime_s=0 disables runtime gate
// V3 - 2026-04-27 - fix: extern declaration for current_vib_pattern (defined in System.ino)
// V3 - 2026-04-27 - P8.1 Bug 2 fix: FM mode display uses scroll3Digits("FM[n]") — digit "1" as second
//   character of displayDigits() renders as a barely-visible horizontal bar, so all modes looked like "F"
// V2.5-Evo - 2026-04-28 - P9: Bug1B pre-arm check; Bug1D all-exit Pattern4/StP; S2 FM full-screen confirms
// V2.5-Evo - 2026-04-28 - P9 S4: rtm_arm_dist_m captured at engage; reset on disengage (R5 proximity bar)
// V2.5-Evo - 2026-04-28 - Chg5: runDoubleSqueezeArm() blocking double-squeeze ceremony; "A rM"→"A r"; "St P"→"St"
// V2.5-Evo - 2026-04-28 - ChgB/C/D/E: SPIFFS seed on first arm; cycle 1→2→3→1 (skip F0); "FM" confirm; 30s keepalive
// V2.5-Evo - 2026-04-28 - ChgDZ: persistent "r n" blinks use displayDigitZone() to preserve R5 proximity bar
// V2.5-Evo - 2026-04-28 - Bug2: setRtmArmed() clears fm_armed — RTM and FM are mutually exclusive
// V2.5-Evo - 2026-04-28 - Bug3: rtmDisengage() clears displayBuffer[6] (R5) to prevent FM phantom pixel
// V2.5-Evo - 2026-04-28 - Bug4: runDoubleSqueezeArm() rewritten — handles single+double squeeze; removes "A r"; RTM_ARMED dead code
// V2.5-Evo - 2026-04-28 - Task2: fmSilentDisarm() for arm-window expiry; cycleFmMode() cycles on armed+no-throttle; "F n" display
// V2.5-Evo - 2026-04-28 - Task3: bobbing advanceArrow() in RTM arm wait loops; delay(250) after unlockAnimation(); Pattern4 after animation; clear on timeout
// V2.5-Evo - 2026-04-28 - TaskA: rtm_arm_gps_timeout_override — 4× GPS staleness threshold during blocking arm ceremony;
//   cleared by rtmDisengage() and ceremony timeout paths; Gate 2 reads it via ternary.
// V2.5-Evo - 2026-04-29 - Fix 1-4: setRtmArmed() calls fmSilentDisarm() so RX
//   receives 0xF2/0 when RTM preempts FM — prevents stale fm_mode_runtime on RX
//   TODO: remove when runDoubleSqueezeArm() is refactored to non-blocking.
// V2.5-Evo - 2026-04-29 - Fix 4-3: fm_armed declared volatile (read core 0 / write core 1)
// V2.5-Evo - 2026-04-29 - Fix 2-1: pre-arm rejection path now clears rtm_arm_gps_timeout_override
// V2.5-Evo - 2026-04-29 - F0: FM cycle extended to 1→2→3→0; landing on 0 disarms FM (RAM-only hand-off mode)
// V2.5-Evo - 2026-04-29 - Display: F0-F3 confirms and FM arm confirm now use large-font
//   displayDigits(LET_F, mode) instead of showFullScreenMessage() compact font
// V2.5-Evo - 2026-04-29 - Display: "St" confirm now uses large-font displayDigits(LET_S, LET_T)
//   in all three call sites (rtmDisengage, runDoubleSqueezeArm rejection, fmInternalDisarm)

extern volatile uint8_t current_vib_pattern;
extern float rtm_arm_dist_m;  // defined in BREmote_V2_Tx.h — captured at RTM engage moment

// ============================================================
// RTM STATE MACHINE
// ============================================================

typedef enum { RTM_IDLE, RTM_ARMED, RTM_SQUEEZE_WAIT, RTM_ACTIVE, RTM_COOLDOWN } RtmTxState;

static RtmTxState  rtm_tx_state        = RTM_IDLE;
static unsigned long rtm_arm_start_ms  = 0;   // when ARMED state was entered
static unsigned long rtm_active_start_ms = 0; // when ACTIVE state was entered
static unsigned long rtm_release_ms    = 0;   // when throttle was last released during ACTIVE
static unsigned long rtm_cooldown_ms   = 0;   // when COOLDOWN state was entered
static unsigned long rtm_squeeze_ms    = 0;   // when SQUEEZE_WAIT was entered
// File-scope so setRtmArmed() can reset it. Inside a switch case it can't be reached
// externally; stale value after arm-window timeout would skip the 500ms hold check.
static unsigned long rtm_hold_start    = 0;   // single-mode: when thr_scaled first crossed 30%

// Temporary 4× multiplier on the GPS staleness threshold used by Gate 2 in runRtmLoop().
// runDoubleSqueezeArm() blocks loop() for up to rtm_arm_window_s seconds, freezing GPS
// polling. Without this, Gate 2 fires immediately on the first runRtmLoop() call after
// the ceremony because GPS age >> rtm_gps_timeout_ms. Set at ceremony start; cleared by
// rtmDisengage() (covers all RTM_ACTIVE exit paths) and the two ceremony timeout returns.
// TODO: remove when arm ceremony is refactored to non-blocking.
static uint32_t rtm_arm_gps_timeout_override = 0;

// FM session-init and keepalive state (Changes B + E)
static bool          fm_session_init_done = false;  // Change B: true once last_fm_mode seeded from SPIFFS this session
static unsigned long fm_last_sync_ms      = 0;      // Change E: millis() of last 0xF2 keepalive; 0 when FM disarmed

// ---- Compute the current throttle cap for the ramp ----
// Returns 0-255. During ACTIVE, ramps from rtm_throttle_start_pct→max over rtm_ramp_duration_s.
uint8_t calcRtmThrottleCap()
{
  if (rtm_tx_state != RTM_ACTIVE) return 255;  // no cap outside ACTIVE
  unsigned long elapsed = millis() - rtm_active_start_ms;
  float t = (float)elapsed / ((float)usrConf.rtm_ramp_duration_s * 1000.0f);
  if (t > 1.0f) t = 1.0f;
  float pct = (float)usrConf.rtm_throttle_start_pct
            + t * (float)(usrConf.rtm_throttle_max_pct - usrConf.rtm_throttle_start_pct);
  return (uint8_t)(pct * 255.0f / 100.0f);
}

// ---- Called by handleGearToggle() when RTM combo gesture completes ----
// Bug2: fm_armed cleared first — RTM and FM are mutually exclusive.
// Bug4: runDoubleSqueezeArm() now handles both single and double squeeze, fully blocking.
//       On return rtm_tx_state is RTM_ACTIVE or RTM_IDLE; RTM_ARMED case in runRtmLoop() is dead code.
void setRtmArmed()
{
  if (!usrConf.rtm_enabled || !usrConf.gps_en) return;
  fmSilentDisarm();                // Bug2 + Finding 1-4: disarm FM and notify RX via 0xF2/0 before RTM arms
  rtm_tx_state     = RTM_ARMED;
  rtm_arm_start_ms = millis();
  rtm_hold_start   = 0;
  rtm_tx_active    = false;
  rtm_thr_cap_tx   = 255;
  queueMetaPacketBurst(0xF1, 0);   // tell RX: RTM armed but not yet active
  current_vib_pattern = 4;         // Pattern 4: 2 fast short = RTM arm confirm
  runDoubleSqueezeArm();            // Bug4: handles both single and double squeeze
}

// ---- Called to disengage RTM from the gesture layer (user-initiated) ----
// Pattern 4 + "St P" now handled inside rtmDisengage().
static void setRtmDisarmed()
{
  rtmDisengage();
}

// ---- Disengage RTM: return to COOLDOWN, notify RX, confirm with haptic + display ----
// V2.5-Evo - 2026-04-28 - P9 Bug1D: Pattern 4 and "St P" moved here so ALL exit paths
// (steer-exit, GPS stale, max runtime, throttle release) fire the confirm consistently.
static void rtmDisengage()
{
  rtm_tx_state    = RTM_COOLDOWN;
  rtm_cooldown_ms = millis();
  rtm_tx_active   = false;
  displayBuffer[6] = 0x0000;     // Bug3: clear R5 proximity bar row — updateR5ProximityBar() left
                                 // stale data here; without clearing, FM mode sees a phantom pixel
  rtm_thr_cap_tx  = 255;
  rtm_arm_dist_m  = 0.0f;        // reset R5 bar reference (defined in BREmote_V2_Tx.h)
  rtm_arm_gps_timeout_override = 0;  // clear GPS timeout multiplier — ceremony fully over
  queueMetaPacketBurst(0xF1, 0);  // tell RX: RTM inactive

  // Fire Pattern 4 BEFORE the blocking display so vibration runs during the 2s flash
  current_vib_pattern = 4;  // 2 fast short = RTM disengage confirm

  // Large-font stop confirm: LET_S(32) renders as "5", LET_T(20) renders as "t".
  // "5t" appearance is intentional — matches large-font style of F0-F3 confirms.
  displayDigits(LET_S, LET_T);
  updateDisplay();
  delay(2000);
}

// ---- Decode telemetry.rtm_distance to metres ----
// Returns -1.0f if no valid distance available (telemetry.rtm_distance == 0xFF).
// Used by pre-arm check (Bug 1B) and R5 proximity bar.
static float decodeRtmDistanceM()
{
  uint8_t d = telemetry.rtm_distance;
  if (d == 0xFF) return -1.0f;
  if (d < 100)   return d / 10.0f;     // tenths of metre (0.0–9.9 m)
  return (float)(d - 90);              // whole metres (10–164 m)
}

// ============================================================
// V2.5-Evo - 2026-04-28 - Bug4: Full rewrite. Handles both single and double squeeze.
// Always called blocking from setRtmArmed(). Uses rtm_arm_start_ms as shared arm-window ref.
// "A r" and "rn ×2" ceremony removed. Arm confirmation is unlockAnimation() + "r n" 2s.
//
// Single (rtm_double_squeeze_en==0):
//   blink "r n" → thr >30% held 500ms → unlockAnimation()+P4 → "r n" 2s → dist check → ACTIVE
//
// Double (rtm_double_squeeze_en==1):
//   blink "r n" → 1st thr >30% held 500ms → unlockAnimation() → blank 800ms →
//   2nd thr >30% held 500ms → unlockAnimation()+P4 → "r n" 2s → dist check → ACTIVE
//
// On return: rtm_tx_state == RTM_ACTIVE (success) or RTM_IDLE (timeout / rejected).
// ============================================================
static void runDoubleSqueezeArm()
{
  // Relax the GPS staleness threshold (Gate 2) for the duration of this blocking ceremony.
  // loop() is suspended here for up to rtm_arm_window_s seconds, so GPS age accumulates.
  // rtmDisengage() clears this on every RTM_ACTIVE exit path.
  rtm_arm_gps_timeout_override = (uint32_t)usrConf.rtm_gps_timeout_ms * 4UL;

  // Show "r n" while waiting for first squeeze
  displayDigitZone("r n");
  advanceArrow();   // prime arrow before loop; advanceArrow() calls updateDisplay() internally

  // Wait for first squeeze: thr > 30% (thr_scaled > 76) held for 500ms continuous
  bool          first_ok = false;
  unsigned long hold_ms  = 0;
  while (millis() - rtm_arm_start_ms < (unsigned long)usrConf.rtm_arm_window_s * 1000UL)
  {
    advanceArrow();   // bob arrow every 100ms while waiting for squeeze
    if (thr_scaled > 76)
    {
      if (hold_ms == 0) hold_ms = millis();
      if (millis() - hold_ms >= 500UL) { first_ok = true; hold_ms = 0; break; }
    }
    else { hold_ms = 0; }
    delay(100);
    checkSerial();
  }
  if (!first_ok)
  {
    rtm_arm_gps_timeout_override = 0;  // ceremony aborted — restore normal GPS threshold
    for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000;
    updateDisplay();
    rtm_tx_state = RTM_IDLE;
    return;
  }

  if (!usrConf.rtm_double_squeeze_en)
  {
    // Single-squeeze: unlock, pause, then Pattern 4 + "r n" arm confirm
    unlockAnimation();
    delay(250);
    current_vib_pattern = 4;   // Pattern 4 after visual unlock completes
    displayDigitZone("r n");
    updateDisplay();
    delay(2000);
  }
  else
  {
    // Double-squeeze: first unlock (no P4 yet), pause, then black screen, then wait for second squeeze
    unlockAnimation();
    delay(250);
    for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000;
    updateDisplay();
    delay(800);

    bool second_ok = false;
    hold_ms = 0;
    advanceArrow();   // prime arrow for second wait
    while (millis() - rtm_arm_start_ms < (unsigned long)usrConf.rtm_arm_window_s * 1000UL)
    {
      advanceArrow();   // bob arrow every 100ms while waiting for second squeeze
      if (thr_scaled > 76)
      {
        if (hold_ms == 0) hold_ms = millis();
        if (millis() - hold_ms >= 500UL) { second_ok = true; hold_ms = 0; break; }
      }
      else { hold_ms = 0; }
      delay(100);
      checkSerial();
    }
    if (!second_ok)
    {
      rtm_arm_gps_timeout_override = 0;  // ceremony aborted — restore normal GPS threshold
      for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000;
      updateDisplay();
      rtm_tx_state = RTM_IDLE;
      return;
    }

    // Second squeeze confirmed: unlock, pause, then Pattern 4 + "r n" arm confirm
    unlockAnimation();
    delay(250);
    current_vib_pattern = 4;   // Pattern 4 after visual unlock completes
    displayDigitZone("r n");
    updateDisplay();
    delay(2000);
  }

  // Pre-arm distance check: reject if already inside the disengage threshold
  float prearm_m = decodeRtmDistanceM();
  if (prearm_m >= 0.0f && prearm_m <= (float)usrConf.rtm_disengage_distance_m)
  {
    rtm_arm_gps_timeout_override = 0;  // Finding 2-1: only exit path that left
                                        // the 4× override stale; all other exits
                                        // (timeouts + rtmDisengage) already clear it
    current_vib_pattern = 4;
    // Large-font stop confirm on arm rejection.
    displayDigits(LET_S, LET_T);
    updateDisplay();
    delay(2000);
    rtm_tx_state  = RTM_IDLE;
    rtm_tx_active = false;
    queueMetaPacketBurst(0xF1, 0);
    return;
  }

  // Activate RTM
  rtm_tx_state        = RTM_ACTIVE;
  rtm_active_start_ms = millis();
  rtm_tx_active       = true;
  rtm_release_ms      = 0;
  rtm_arm_dist_m      = decodeRtmDistanceM();
  if (rtm_arm_dist_m < 0.0f) rtm_arm_dist_m = 0.0f;
  queueMetaPacketBurst(0xF1, 1);
}

// ---- Called from loop() every ~110ms ----
void runRtmLoop()
{
  if (!usrConf.rtm_enabled || !usrConf.gps_en) return;

  unsigned long now = millis();

  switch (rtm_tx_state)
  {
    // ---- IDLE: nothing to do; setRtmArmed() transitions out ----
    case RTM_IDLE:
      break;

    // ---- ARMED: dead code — Bug4 moved all arm logic into runDoubleSqueezeArm() ----
    // setRtmArmed() now calls runDoubleSqueezeArm() blocking for both single and double squeeze.
    // On return rtm_tx_state is RTM_ACTIVE or RTM_IDLE — this case is never reached.
    case RTM_ARMED:
      rtm_tx_state = RTM_IDLE;
      break;

    // ---- SQUEEZE_WAIT: dead code path (Change 5) ----
    // Double-squeeze arm is now fully blocking in runDoubleSqueezeArm(), called from setRtmArmed().
    // This state can no longer be entered; retained for enum completeness only.
    case RTM_SQUEEZE_WAIT:
      rtm_tx_state = RTM_IDLE;
      break;

    // ---- ACTIVE: RTM running ----
    case RTM_ACTIVE:
    {
      // Update throttle cap for ramp
      rtm_thr_cap_tx = calcRtmThrottleCap();

      // Gate 1: max runtime (0 = disabled — safety gates handle all real scenarios)
      if (usrConf.rtm_max_runtime_s > 0 &&
          now - rtm_active_start_ms > (unsigned long)usrConf.rtm_max_runtime_s * 1000UL)
      {
        rtmDisengage();
        break;
      }

      // Gate 2: TX GPS freshness — use 4× relaxed threshold in the cycles immediately after
      // the blocking arm ceremony (GPS age may be high; rtm_arm_gps_timeout_override > 0
      // until rtmDisengage() clears it on any RTM_ACTIVE exit path).
      {
        uint32_t gps_thr = (rtm_arm_gps_timeout_override > 0)
                           ? rtm_arm_gps_timeout_override
                           : (uint32_t)usrConf.rtm_gps_timeout_ms;
        if (gps_tx.location.age() > gps_thr)
        {
          rtmDisengage();
          break;
        }
      }

      // Gate 3: throttle release > 10s → disengage
      // V2.5-Evo - 2026-04-28 - P9 Bug1D: now calls rtmDisengage() so Pattern 4 + "St P" fire.
      if (thr_scaled < 10)
      {
        if (rtm_release_ms == 0) rtm_release_ms = now;
        if (now - rtm_release_ms > 10000UL)
        {
          rtmDisengage();
          break;
        }
      }
      else
      {
        rtm_release_ms = 0;
      }

      // Gate 4: steering exit (P8 — if enabled, any significant steering input exits RTM)
      if (usrConf.rtm_steer_exit_on_input && toggle_blocked_by_steer &&
          abs((int)steer_scaled - 127) > 20)
      {
        setRtmDisarmed();
        break;
      }

      // Display handled by renderRtmInfoDisplay() in loop() when rtm_tx_active==true
      break;
    }

    // ---- COOLDOWN: wait 2s then return to IDLE ----
    // showFullScreenMessage("St P", 2000) was called at disengage moment; by the time
    // this state is polled, the 2s has already elapsed → transition to IDLE immediately.
    case RTM_COOLDOWN:
      if (now - rtm_cooldown_ms > 2000UL)
      {
        rtm_tx_state = RTM_IDLE;
      }
      break;
  }
}

// ============================================================
// FM STATE MACHINE
// V3 - 2026-04-27 - P8.1: FM redesigned as arm/disarm toggle with persistent mode memory.
//
// ARM (LEFT tap + RIGHT hold 5s, first time or after disarm):
//   - Arms at last_fm_mode (RAM; defaults to F1 on power cycle — never resets to F0/disabled)
//   - Blinks "F[mode]" x2 on display; fires Pattern 4 (2 fast buzzes) as arm confirm
//   - FM active: user engages throttle to ride
//
// CHANGE MODE while armed (LEFT hold 2s, intercepted by Hall.ino):
//   - Cycles F0→F1→F2→F3→F0; stays armed; sends new mode to RX; resets arm timer
//
// DISARM (any of):
//   - Same combo again (LEFT tap + RIGHT hold 5s) — toggle
//   - Throttle release for 3s after first throttle input — Gate 1
//   - Arm window expires (fm_arm_window_s) before any throttle input — auto-disarm
// ============================================================

volatile bool        fm_armed         = false;  // FM arm state; RAM only, cleared on power cycle. Not static — extern'd by Display.ino (R5 bar)
                                                 // volatile: read by updateBargraphs() on core 0, written by loop() on core 1
static uint8_t       last_fm_mode     = 1;      // last active FM mode (1-3); defaults F1; RAM only
static unsigned long fm_arm_ms        = 0;      // time of arm, or time of last throttle >10 while armed
static bool          fm_throttle_seen = false;  // becomes true once thr_scaled>10 after arming

// Returns true if FM is currently armed; called by Hall.ino to intercept LEFT hold 2s
bool isFmArmed() { return fm_armed; }

// Silent disarm: clears FM state and notifies RX, but shows no display and fires no haptic.
// Used when arm-window expires before any throttle input — nothing active to confirm.
static void fmSilentDisarm()
{
  fm_armed         = false;
  fm_throttle_seen = false;
  fm_last_sync_ms  = 0;
  queueMetaPacketBurst(0xF2, 0);   // mode 0 = FM disabled on RX
}

// Internal disarm: clears state, notifies RX, fires haptic, shows "St" full-screen.
// V2.5-Evo - 2026-04-28 - P9 S2: showFmMode() removed; disarm shows blocking stop message.
// V2.5-Evo - 2026-04-28 - ChgE: fm_last_sync_ms reset to 0 on disarm so keepalive timer clears.
static void fmDisarm()
{
  fm_armed         = false;
  fm_throttle_seen = false;
  fm_last_sync_ms  = 0;            // Change E: clear keepalive timer
  queueMetaPacketBurst(0xF2, 0);   // mode 0 = FM disabled on RX (followme_mode=0)
  current_vib_pattern = 4;         // Pattern 4: 2 fast buzzes = disarm confirm
  // Large-font stop confirm on FM disarm.
  displayDigits(LET_S, LET_T);
  updateDisplay();
  delay(2000);
}

// Called by handleGearToggle() combo (LEFT tap + RIGHT hold 5s) — toggles arm/disarm.
// On arm: seeds last_fm_mode from SPIFFS on first arm this session (Change B); fires Pattern 4;
// shows "FM" confirm (Change D, 6 cols ≤ C0-C5); sends 0xF2 to RX; starts keepalive timer (Change E).
void cycleFmMode()
{
  if (!usrConf.fm_override_enabled || !usrConf.gps_en) return;

  if (fm_armed)
  {
    if (fm_throttle_seen)
    {
      // User already rode — treat gesture as disarm toggle
      fmDisarm();
    }
    else
    {
      // No throttle yet — cycle to next mode (1→2→3→0 where 0 = disarm)
      last_fm_mode = (last_fm_mode < 3) ? last_fm_mode + 1 : 0;

      if (last_fm_mode == 0)
      {
        // F0: FM disabled — disarm with brief visual confirm and return to normal display.
        // Sends 0xF2/0 to RX (FM off), fires Pattern 4, resets mode to SPIFFS default.
        // This is RAM-only; power cycle restores usrConf.followme_mode.
        current_vib_pattern = 4;
        // Large-font F0 disarm confirm: LET_F + 0. Shorter hold (1s) — this is a disarm, not a mode select.
        displayDigits(LET_F, 0);
        updateDisplay();
        delay(1000);
        queueMetaPacketBurst(0xF2, 0);       // tell RX: FM disabled
        fm_armed         = false;
        fm_throttle_seen = false;
        fm_last_sync_ms  = 0;
        // Reset mode to SPIFFS default so next arm starts at configured mode, not 0
        last_fm_mode = (usrConf.followme_mode >= 1 && usrConf.followme_mode <= 3)
                       ? usrConf.followme_mode : 1;
        return;
      }

      // Large-font mode confirm: LET_F + mode digit (1/2/3). snprintf no longer needed.
      displayDigits(LET_F, last_fm_mode);
      updateDisplay();
      delay(2000);
      queueMetaPacketBurst(0xF2, last_fm_mode);
      fm_last_sync_ms = millis();
      fm_arm_ms       = millis();   // reset arm window — user is actively choosing a mode
    }
    return;
  }

  // V2.5-Evo - 2026-04-28 - Change B: On first arm this session, seed last_fm_mode from SPIFFS.
  // usrConf.followme_mode is the user's configured starting mode (range 1-3; 0 is invalid here).
  // After seeding, fm_session_init_done prevents overriding any mode the user cycled to mid-session.
  if (!fm_session_init_done)
  {
    if (usrConf.followme_mode >= 1 && usrConf.followme_mode <= 3)
      last_fm_mode = usrConf.followme_mode;
    fm_session_init_done = true;
  }

  // Arm at last used mode (never arms at F0 = disabled; last_fm_mode defaults to 1)
  fm_armed         = true;
  fm_arm_ms        = millis();
  fm_throttle_seen = false;
  current_vib_pattern = 4;         // Pattern 4: 2 fast buzzes = arm confirm
  fm_last_sync_ms  = millis();     // Change E: start keepalive timer from now (avoids immediate re-sync)

  // V2.5-Evo - 2026-04-29 - Display: show actual mode being armed (F1/F2/F3) in large font
  // instead of the generic "FM" text. Uses large num0[] font via LET_F(15) + mode digit.
  displayDigits(LET_F, last_fm_mode);
  updateDisplay();
  delay(2000);

  queueMetaPacketBurst(0xF2, last_fm_mode);
}

// Called by handleGearToggle(-1) simple LEFT hold 2s when FM is armed (Hall.ino checks isFmArmed()).
// Cycles mode 1→2→3→1 (Change C: skips mode 0 = disabled); stays armed; resets arm timer.
void cycleFmModeArmed()
{
  if (!fm_armed) return;
  // Cycle 1→2→3→0 where 0 = disarm (FM disabled RAM-only state for hand-off to inexperienced user).
  last_fm_mode = (last_fm_mode < 3) ? last_fm_mode + 1 : 0;

  if (last_fm_mode == 0)
  {
    // F0: FM disabled — disarm with brief visual confirm and return to normal display.
    // Sends 0xF2/0 to RX (FM off), fires Pattern 4, resets mode to SPIFFS default.
    // This is RAM-only; power cycle restores usrConf.followme_mode.
    current_vib_pattern = 4;
    // Large-font F0 disarm confirm: LET_F + 0. Shorter hold (1s) — this is a disarm, not a mode select.
    displayDigits(LET_F, 0);
    updateDisplay();
    delay(1000);
    queueMetaPacketBurst(0xF2, 0);       // tell RX: FM disabled
    fm_armed         = false;
    fm_throttle_seen = false;
    fm_last_sync_ms  = 0;
    // Reset mode to SPIFFS default so next arm starts at configured mode, not 0
    last_fm_mode = (usrConf.followme_mode >= 1 && usrConf.followme_mode <= 3)
                   ? usrConf.followme_mode : 1;
    return;
  }

  // Large-font mode confirm: LET_F + mode digit (1/2/3). snprintf no longer needed.
  displayDigits(LET_F, last_fm_mode);
  updateDisplay();
  delay(2000);
  queueMetaPacketBurst(0xF2, last_fm_mode);
  fm_last_sync_ms = millis();              // reset keepalive — just synced
  fm_arm_ms       = millis();             // reset arm window — user is actively choosing a mode
}

// Called from loop() every ~110ms.
// Handles arm-window auto-disarm and Gate 1 (throttle-release disarm).
void runFmLoop()
{
  if (!fm_armed) return;

  unsigned long now = millis();

  // Arm-window auto-disarm: if user never applied throttle since arming, disarm after fm_arm_window_s
  if (!fm_throttle_seen)
  {
    if (now - fm_arm_ms > (unsigned long)usrConf.fm_arm_window_s * 1000UL)
    {
      fmSilentDisarm();   // arm window expired before first throttle — no confirm needed
      return;
    }
  }

  // Track throttle engagement; while riding, keep arm timer alive
  if (thr_scaled > 10)
  {
    fm_throttle_seen = true;
    fm_arm_ms = now;  // reset — Gate 1 timer starts from last throttle input
  }

  // Gate 1: throttle release for 3s after first engagement → disarm FM
  // 3s grace allows brief stops without losing FM (e.g., obstacle avoidance)
  if (fm_throttle_seen && thr_scaled < 5)
  {
    if (now - fm_arm_ms > 3000UL)
    {
      fmDisarm();
      return;
    }
  }

  // V2.5-Evo - 2026-04-28 - Change E: Send 0xF2 keepalive every 30s while FM is armed.
  // Ensures RX stays in the correct FM mode after any transient packet loss.
  if (fm_last_sync_ms > 0 && now - fm_last_sync_ms >= 30000UL)
  {
    queueMetaPacketBurst(0xF2, last_fm_mode);
    fm_last_sync_ms = now;
  }
}
