// V3 - 2026-04-25 - P7: TX RTM and FM state machines.
// RTM: left-hold gesture → arm → squeeze(s) → active → cooldown → idle
// FM:  right-hold gesture → cycle FM mode 0→1→2→3→0 → send 0xF2 meta-packet
// V3 - 2026-04-27 - P8: setRtmArmed shows "rn" ×2 (static, 3s total); showFmMode shows F0-F3;
//   added setRtmDisarmed(); steer-exit gate in ACTIVE; rtm_max_runtime_s=0 disables runtime gate
// V3 - 2026-04-27 - fix: extern declaration for current_vib_pattern (defined in System.ino)
// V3 - 2026-04-27 - P8.1 Bug 2 fix: FM mode display uses scroll3Digits("FM[n]") — digit "1" as second
//   character of displayDigits() renders as a barely-visible horizontal bar, so all modes looked like "F"

extern volatile uint8_t current_vib_pattern;

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
// Transitions from IDLE to ARMED; shows "rn" static ×2 (1.5 s each = 3 s total);
// fires Pattern 4 haptic confirm (2 fast short pulses).
void setRtmArmed()
{
  if (!usrConf.rtm_enabled || !usrConf.gps_en) return;
  rtm_tx_state     = RTM_ARMED;
  rtm_arm_start_ms = millis();
  rtm_hold_start   = 0;    // reset single-mode hold timer each time we arm
  rtm_tx_active    = false;
  rtm_thr_cap_tx   = 255;
  queueMetaPacketBurst(0xF1, 0);  // tell RX: RTM armed but not yet active
  current_vib_pattern = 4;        // Pattern 4: 2 fast short = RTM arm confirm
  // Show "rn" (return now) as 2 static passes of 1.5 s each
  for (int i = 0; i < 2; i++)
  {
    displayDigits(LET_R, LET_N);
    updateDisplay();
    delay(1500);
  }
}

// ---- Called to disengage RTM from the gesture layer (user-initiated) ----
// Transitions to COOLDOWN via rtmDisengage() and fires Pattern 4 haptic confirm.
static void setRtmDisarmed()
{
  rtmDisengage();
  current_vib_pattern = 4;  // Pattern 4: 2 fast short = RTM disarm confirm
}

// ---- Disengage RTM: return to COOLDOWN, notify RX ----
static void rtmDisengage()
{
  rtm_tx_state   = RTM_COOLDOWN;
  rtm_cooldown_ms = millis();
  rtm_tx_active  = false;
  rtm_thr_cap_tx = 255;
  queueMetaPacketBurst(0xF1, 0);  // tell RX: RTM inactive
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

    // ---- ARMED: wait for first (or only) squeeze ----
    case RTM_ARMED:
      // Arm window timeout
      if (now - rtm_arm_start_ms > (unsigned long)usrConf.rtm_arm_window_s * 1000UL)
      {
        rtm_tx_state = RTM_IDLE;
        break;
      }
      // Blink "rn" every ~500ms while waiting for squeeze
      if ((now / 500) % 2 == 0)
      {
        displayDigits(LET_R, LET_N);
        updateDisplay();
      }

      if (usrConf.rtm_double_squeeze_en)
      {
        // Wait for first squeeze (thr_scaled > 10%)
        if (thr_scaled > 25)
        {
          displayDigits(LET_A, LET_R);  // "AR" = first squeeze detected
          updateDisplay();
          // Wait for user to release throttle (block briefly — this is the trigger ack)
          unsigned long rel_wait = millis();
          while (thr_scaled > 5 && millis() - rel_wait < 2000) delay(20);
          rtm_tx_state  = RTM_SQUEEZE_WAIT;
          rtm_squeeze_ms = millis();
        }
      }
      else
      {
        // Single mode: throttle > 30% for 500ms continuous
        if (thr_scaled > 76)
        {
          if (rtm_hold_start == 0) rtm_hold_start = now;
          if (now - rtm_hold_start >= 500)
          {
            rtm_hold_start    = 0;
            rtm_tx_state      = RTM_ACTIVE;
            rtm_active_start_ms = now;
            rtm_tx_active     = true;
            rtm_release_ms    = 0;
            queueMetaPacketBurst(0xF1, 1);
          }
        }
        else
        {
          rtm_hold_start = 0;
        }
      }
      break;

    // ---- SQUEEZE_WAIT: waiting for second squeeze (double-squeeze mode only) ----
    case RTM_SQUEEZE_WAIT:
      // 5s window
      if (now - rtm_squeeze_ms > 5000UL)
      {
        rtm_tx_state = RTM_IDLE;
        break;
      }
      // Blink "RY" (ready for second squeeze)
      if ((now / 300) % 2 == 0)
      {
        displayDigits(LET_R, LET_Y);
        updateDisplay();
      }
      if (thr_scaled > 25)
      {
        // Second squeeze → ACTIVE
        rtm_tx_state      = RTM_ACTIVE;
        rtm_active_start_ms = now;
        rtm_tx_active     = true;
        rtm_release_ms    = 0;
        queueMetaPacketBurst(0xF1, 1);
      }
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

      // Gate 2: TX GPS freshness
      if (gps_tx.location.age() > usrConf.rtm_gps_timeout_ms)
      {
        rtmDisengage();
        break;
      }

      // Gate 3: throttle release > 10s → disengage
      if (thr_scaled < 10)
      {
        if (rtm_release_ms == 0) rtm_release_ms = now;
        if (now - rtm_release_ms > 10000UL)
        {
          rtm_tx_state   = RTM_IDLE;
          rtm_tx_active  = false;
          rtm_thr_cap_tx = 255;
          queueMetaPacketBurst(0xF1, 0);
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

    // ---- COOLDOWN: show "Stp" for 2s ----
    case RTM_COOLDOWN:
      scroll3Digits(LET_S, LET_T, LET_P, 50);
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

static bool          fm_armed         = false;  // FM arm state; RAM only, cleared on power cycle
static uint8_t       last_fm_mode     = 1;      // last active FM mode (1-3); defaults F1; RAM only
static unsigned long fm_arm_ms        = 0;      // time of arm, or time of last throttle >10 while armed
static bool          fm_throttle_seen = false;  // becomes true once thr_scaled>10 after arming

// Returns true if FM is currently armed; called by Hall.ino to intercept LEFT hold 2s
bool isFmArmed() { return fm_armed; }

// Show FM mode code briefly: scrolls "FM0"…"FM3" across the display (~700ms).
// Uses scroll3Digits instead of displayDigits because digit "1" as the second character
// of a 2-char display renders as a single thin horizontal bar — nearly invisible next to "F".
// scroll3Digits shows all three characters (F, M, mode) in a clear scrolling animation.
static void showFmMode(uint8_t mode)
{
  scroll3Digits(LET_F, LET_M, mode, 50);
}

// Internal disarm: clears state, notifies RX, fires haptic, scrolls "FM-"
static void fmDisarm()
{
  fm_armed         = false;
  fm_throttle_seen = false;
  queueMetaPacketBurst(0xF2, 0);          // mode 0 = FM disabled on RX (followme_mode=0)
  current_vib_pattern = 4;                // Pattern 4: 2 fast buzzes = disarm confirm
  scroll3Digits(LET_F, LET_M, DASH, 50); // scroll "FM-" = FM off (~700ms)
}

// Called by handleGearToggle() combo (LEFT tap + RIGHT hold 5s) — toggles arm/disarm.
// On arm: uses last_fm_mode (default F1); blinks mode x2; fires Pattern 4; sends 0xF2 to RX.
void cycleFmMode()
{
  if (!usrConf.fm_override_enabled || !usrConf.gps_en) return;

  if (fm_armed)
  {
    // Already armed → disarm (toggle)
    fmDisarm();
    return;
  }

  // Arm at last used mode (never arms at F0 = disabled; last_fm_mode defaults to 1)
  fm_armed         = true;
  fm_arm_ms        = millis();
  fm_throttle_seen = false;
  current_vib_pattern = 4;         // Pattern 4: 2 fast buzzes = arm confirm

  // Scroll "FM[mode]" x2 (~700ms each = ~1.4s total) as arm confirm
  for (int i = 0; i < 2; i++)
  {
    scroll3Digits(LET_F, LET_M, last_fm_mode, 50);
  }

  queueMetaPacketBurst(0xF2, last_fm_mode);
}

// Called by handleGearToggle(-1) simple LEFT hold 2s when FM is armed (Hall.ino checks isFmArmed()).
// Cycles mode F0→F1→F2→F3→F0; stays armed; resets arm timer.
void cycleFmModeArmed()
{
  if (!fm_armed) return;
  last_fm_mode = (last_fm_mode + 1) % 4;
  showFmMode(last_fm_mode);                // show new mode briefly
  queueMetaPacketBurst(0xF2, last_fm_mode);
  fm_arm_ms = millis();                    // reset arm window — user is actively choosing a mode
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
      fmDisarm();
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
}
