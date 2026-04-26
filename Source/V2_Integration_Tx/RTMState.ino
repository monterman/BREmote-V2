// V3 - 2026-04-25 - P7: TX RTM and FM state machines.
// RTM: left-hold gesture → arm → squeeze(s) → active → cooldown → idle
// FM:  right-hold gesture → cycle FM mode 0→1→2→3→0 → send 0xF2 meta-packet

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

// ---- Called by handleGearToggle() when left long-press threshold is reached ----
// Transitions from IDLE to ARMED and shows "rtn" on display.
void setRtmArmed()
{
  if (!usrConf.rtm_enabled || !usrConf.gps_en) return;
  rtm_tx_state     = RTM_ARMED;
  rtm_arm_start_ms = millis();
  rtm_hold_start   = 0;    // reset single-mode hold timer each time we arm
  rtm_tx_active    = false;
  rtm_thr_cap_tx   = 255;
  queueMetaPacketBurst(0xF1, 0);  // tell RX: RTM not yet active (just armed on TX)
  scroll3Digits(LET_R, LET_T, LET_N, 100);
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
      // Show "rtn" on display (blink every ~500ms while waiting)
      if ((now / 500) % 2 == 0)
      {
        scroll3Digits(LET_R, LET_T, LET_N, 50);
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

      // Gate 1: max runtime
      if (now - rtm_active_start_ms > (unsigned long)usrConf.rtm_max_runtime_s * 1000UL)
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

      // Display "rtn" while active
      scroll3Digits(LET_R, LET_T, LET_N, 30);
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

// Show the current FM mode abbreviation on display
static void showFmMode(uint8_t mode)
{
  switch (mode)
  {
    case 0: scroll3Digits(0, LET_F, LET_F, 150);     break;  // "0ff"
    case 1: scroll3Digits(LET_B, LET_E, LET_H, 150); break;  // "bEh"
    case 2: displayDigits(LET_N, LET_R); updateDisplay(); delay(500); break;  // "nR"
    case 3: displayDigits(LET_N, LET_L); updateDisplay(); delay(500); break;  // "nL"
  }
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
