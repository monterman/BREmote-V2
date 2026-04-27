// V3 - 2026-04-25 - P7: TX RTM and FM state machines.
// RTM: left-hold gesture → arm → squeeze(s) → active → cooldown → idle
// FM:  right-hold gesture → cycle FM mode 0→1→2→3→0 → send 0xF2 meta-packet
// V3 - 2026-04-27 - P8: setRtmArmed shows "rn" ×2 (static, 3s total); showFmMode shows F0-F3;
//   added setRtmDisarmed(); steer-exit gate in ACTIVE; rtm_max_runtime_s=0 disables runtime gate
// V3 - 2026-04-27 - fix: extern declaration for current_vib_pattern (defined in System.ino)

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
// ============================================================

static uint8_t fm_current_mode = 0;  // current TX-side FM selection (0-3)

// Show the current FM mode number on display: "F0", "F1", "F2", "F3"
// V3 - 2026-04-27 - P8: Changed from named modes ("0ff"/"bEh"/"nR"/"nL") to F0-F3 for clarity
static void showFmMode(uint8_t mode)
{
  displayDigits(LET_F, mode);  // mode is 0-3, a valid num0[] digit
  updateDisplay();
  delay(500);
}

// Called by handleGearToggle(+1) long press when fm_override_enabled.
// Cycles FM mode and queues 0xF2 meta-packet burst.
void cycleFmMode()
{
  if (!usrConf.fm_override_enabled || !usrConf.gps_en) return;

  // Enter FM with brief "FM" flash
  displayDigits(LET_F, LET_M);
  updateDisplay();
  delay(500);

  // Cycle once
  fm_current_mode = (fm_current_mode + 1) % 4;
  showFmMode(fm_current_mode);

  // Watch for additional right-holds within 2s to keep cycling
  unsigned long last_action = millis();
  while (millis() - last_action < 2000UL)
  {
    if (tog_input == 1)  // right hold
    {
      fm_current_mode = (fm_current_mode + 1) % 4;
      showFmMode(fm_current_mode);
      last_action = millis();
      while (tog_input == 1) delay(20);  // wait for release
    }
    delay(50);
  }

  // User stopped cycling — send the selected mode
  queueMetaPacketBurst(0xF2, fm_current_mode);
}
