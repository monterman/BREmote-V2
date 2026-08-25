// V2.5-Evo - 2026-08-25 - Follow-Me F4 In Front added to both FM gesture cycles, starting-mode validation and display/meta declaration. TX still only selects and declares the mode; all forward-pacer geometry and safety gates live on the RX. The existing 0xF2 byte already carries the new value, so there is no packet or confStruct change and SW_VERSION stays 27.
// V2.5-Evo - 2026-04-25 - P7: TX RTM and FM state machines.
// RTM: left-hold gesture → arm → squeeze(s) → active → cooldown → idle
// FM:  right-hold gesture → cycle FM mode 0→1→2→3→4→0 → send 0xF2 meta-packet
// V2.5-Evo - 2026-04-27 - P8: setRtmArmed shows "rn" ×2 (static, 3s total); showFmMode shows F0-F3;
//   added setRtmDisarmed(); steer-exit gate in ACTIVE; rtm_max_runtime_s=0 disables runtime gate
// V2.5-Evo - 2026-04-27 - fix: extern declaration for current_vib_pattern (defined in System.ino)
// V2.5-Evo - 2026-04-27 - P8.1 Bug 2 fix: FM mode display uses scroll3Digits("FM[n]") — digit "1" as second
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
// V2.5-Evo - 2026-04-29 - Fix 4-3: fm_armed declared volatile (read by a task / written by loop())
// V2.5-Evo - 2026-04-29 - Fix 2-1: pre-arm rejection path now clears rtm_arm_gps_timeout_override
// V2.5-Evo - 2026-04-29 - F0: FM cycle extended to 1→2→3→0; landing on 0 disarms FM (RAM-only hand-off mode)
// V2.5-Evo - 2026-04-29 - Display: F0-F3 confirms and FM arm confirm now use large-font
//   displayDigits(LET_F, mode) instead of showFullScreenMessage() compact font
// V2.5-Evo - 2026-04-29 - Display: "St" confirm now uses large-font displayDigits(LET_S, LET_T)
//   in all three call sites (rtmDisengage, runDoubleSqueezeArm rejection, fmInternalDisarm)
// V2.5-Evo - 2026-04-29 - Bug fix: RTM arm always failed — two root causes:
//   BugA: decodeRtmDistanceM() returned 0.0m for zero-init telemetry (d==0x00 now → -1.0f)
//   BugB: GPS age exceeded 4× override on double-squeeze; drain Serial1 before RTM_ACTIVE
// V2.5-Evo - 2026-04-29 - GPS dot fix: gpsKeepAliveDelay() replaces bare delay() in
//   ceremony and confirms; drains Serial1 to keep gps_tx.location.age() fresh and
//   prevent GPS dot blinking during blocking display holds.
// V2.5-Evo - 2026-05-02 - Gate 3 throttle-release timeout reduced 10000→4000ms (10s was too long for tow buggy field use)
// V2.5-Evo - 2026-05-10 - SAFETY FIX: zero rtm_thr_cap_tx during arm ceremony to prevent motor runaway (see setRtmArmed)
// V2.5-Evo - 2026-07-20 - T1: Gate 1 throttle-release disarm is now the named constant
//   kFmGate1ReleaseMs, raised 3000 -> 30000 ms (whip gap is 10-25 s off-trigger). Threshold and dead band unchanged.
// V2.5-Evo - 2026-07-20 - StopFeel: every STOP/DISARM confirm now fires Pattern 7 (one 400ms long pulse)
//   instead of Pattern 4 (two 80ms taps), so arm and disarm feel different by touch. Changed sites:
//   rtmDisengage() (RTM disengage), fmDisarm() (FM disarm), the runDoubleSqueezeArm() pre-arm "St"
//   rejection, and both F0-disarm paths (cycleFmMode / cycleFmModeArmed). ARM confirms are UNCHANGED and
//   stay Pattern 4: setRtmArmed(), the two runDoubleSqueezeArm() squeeze confirms, and the cycleFmMode()
//   arm path. fmSilentDisarm() stays silent (arm-window expiry is not a commanded stop).
// V2.5-Evo - 2026-08-17 - StopBuzz FIX: the 2026-08-16 haptic cut was applied inside rtmDisengage()
//   and fmDisarm() — the SHARED sinks — so it also silenced every stop the rider did NOT ask for
//   (RTM Gate 1 max-runtime, Gate 2 GPS-stale, Gate 3 throttle-release, FM Gate 1 release backstop,
//   FM RX fault-stop). That is the exact inverse of the intent. Both functions now take a
//   `commanded` flag: true = the rider asked for it (stay silent), false = a safety gate stopped
//   the system (fire the long STOP buzz). Every call site is classified explicitly below.
//   All Pattern 7 requests now go through vib_stop_pending (System.ino) so a stop buzz can never be
//   swallowed by a pattern that happens to be mid-play.
// V2.5-Evo - 2026-08-17 - StopBuzz REVISION (supersedes the classification in the entry above):
//   the rule is now A PURE TIMEOUT IS SILENT, A FAULT BUZZES. On a wave the rider has no attention
//   to spare for decoding a buzz, and the more buzzes there are the less each one is read, so
//   Pattern 7 is spent on faults only. Moved to commanded = true (SILENT): FM Gate 1 (throttle
//   released 30s) and RTM Gate 3 (throttle released 4s) - both can only fire BECAUSE the trigger
//   was already released, so nothing steps under the rider and the display already says "St".
//   Still commanded = false (BUZZ): RTM Gate 2 (TX GPS lost) and the FM RX fault-stop - faults, not
//   timers - plus RTM Gate 1 (max runtime), the one timer that can fire MID-SQUEEZE, where
//   restoring rtm_thr_cap_tx to 255 un-clamps Throttle.ino and hands the rider raw manual throttle
//   with no gesture behind it. The two arm REFUSALS (RTM pre-arm distance, FM fundamental reject)
//   are unchanged and still buzz.

extern volatile uint8_t current_vib_pattern;
extern volatile bool    vib_stop_pending;   // set true to REQUEST the Pattern 7 STOP buzz (defined in
                                            // System.ino). Never write current_vib_pattern = 7 directly:
                                            // the flag is what makes the stop buzz preempt and survive.
extern float rtm_arm_dist_m;  // defined in BREmote_V2_Tx.h — captured at RTM engage moment

// ---- GPS-aware blocking delay ----
// Replaces bare delay() calls in the arm ceremony and mode confirms.
// Drains Serial1 in 10ms chunks so gps_tx.location.age() stays fresh.
// Prevents the GPS dot from blinking during blocking display animations.
// Safe to call from loop() only — gps_tx.encode() must not be
// called concurrently from core 0.
static void gpsKeepAliveDelay(uint32_t ms)
{
  unsigned long start = millis();
  while (millis() - start < ms)
  {
    while (Serial1.available()) gps_tx.encode(Serial1.read());
    delay(10);
  }
}

// ============================================================
// RTM STATE MACHINE
// ============================================================

typedef enum { RTM_IDLE, RTM_ARMED, RTM_SQUEEZE_WAIT, RTM_ACTIVE, RTM_COOLDOWN } RtmTxState;

static RtmTxState  rtm_tx_state        = RTM_IDLE;

// V2.5-Evo - 2026-06-05 - C-1: 2nd independent throttle gate. True only during the blocking
// RTM arm ceremony (RTM_ARMED), cleared on every exit (→RTM_ACTIVE success / →RTM_IDLE fail).
// sendData() reads this to hard-zero the throttle byte regardless of rtm_thr_cap_tx.
bool rtmIsArming() { return rtm_tx_state == RTM_ARMED; }
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
  // SAFETY FIX: sendData() FreeRTOS task keeps running while loop() is blocked inside
  // runDoubleSqueezeArm(). With cap=255, every arm-squeeze byte goes straight to RX and
  // drives the motor at full duty with rtm_rx_active=0 (no RX gate suppression).
  // Hold cap=0 for the entire ceremony; all abort paths below restore it to 255;
  // success path leaves it at 0 so calcRtmThrottleCap() ramp takes over on first RTM_ACTIVE tick.
  rtm_thr_cap_tx   = 0;
  queueMetaPacketBurst(0xF1, 0);   // tell RX: RTM armed but not yet active
  if (current_vib_pattern == 0) current_vib_pattern = 4;         // Pattern 4: 2 fast short = RTM arm confirm
  runDoubleSqueezeArm();            // Bug4: handles both single and double squeeze
}

// ---- Called to disengage RTM from the gesture layer (user-initiated) ----
// "St" confirm handled inside rtmDisengage().
// Every caller of THIS wrapper is a deliberate rider action (the magnet toggle in Hall.ino and the
// Gate 4 steer-exit), so it always passes commanded = true → no STOP buzz.
static void setRtmDisarmed()
{
  rtmDisengage(true);
}

// ---- Disengage RTM: return to COOLDOWN, notify RX, confirm with haptic + display ----
// V2.5-Evo - 2026-04-28 - P9 Bug1D: Pattern 4 and "St P" moved here so ALL exit paths
// (steer-exit, GPS stale, max runtime, throttle release) fire the confirm consistently.
// INPUT: commanded — true = stay SILENT, false = fire the Pattern 7 STOP buzz. Since the 2026-08-17
//        revision the test is no longer "did the rider press something" but "is this a FAULT or a
//        TIMEOUT": true for the deliberate stops (magnet toggle, steer-exit) AND for Gate 3, a pure
//        timeout the rider's own released trigger caused; false for the GPS-stale fault and for
//        Gate 1 (max runtime), the only timer here that can fire while he is still squeezing.
// OUTPUT: none. SIDE EFFECTS: state → RTM_COOLDOWN, throttle cap restored to 255, 0xF1/0 sent to RX,
//        STOP buzz requested when uncommanded, and a BLOCKING 2s "St" display hold.
static void rtmDisengage(bool commanded)
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

  // Request the STOP confirm BEFORE the blocking display so the buzz runs during the 2s flash.
  // V2.5-Evo - 2026-08-16 - HAPTIC CUT: silent on a DELIBERATE disarm. You just did it, and the
  // display already says so - the stop confirm appears. A buzz confirming your own action is noise,
  // and it was the single most frequent buzz in the system.
  // V2.5-Evo - 2026-08-17 - StopBuzz: that cut was written here, at the SHARED sink, so it also
  // silenced every safety gate that ends up in this same function. The fix is not to buzz them all
  // back - it is to let the caller decide, on one rule: A PURE TIMEOUT IS SILENT, A FAULT BUZZES.
  // Mid-wave the rider has no attention to spare for decoding a buzz, and the more buzzes there
  // are the less each one is read. So Pattern 7 is spent only where it buys something:
  //   FAULT   -> Gate 2 (TX GPS lost). A sensor died; nothing the rider did explains the stop.
  //   TIMEOUT -> Gate 3 (trigger released 4s). His own hand caused it, "St" shows it. Silent.
  //   Gate 1 (max runtime) buzzes despite being a timer, and it is the ONE exception: it has no
  //   throttle precondition, so it can fire while he is still squeezing - and rtm_thr_cap_tx was
  //   restored to 255 a few lines above, which un-clamps Throttle.ino's `if (result >
  //   rtm_thr_cap_tx)` in the same instant. The cap is 30-90% by config range and can never BE
  //   255, so that step to raw manual throttle is always real, and the sendData task keeps
  //   transmitting it throughout the blocking 2s hold below. That transition earns a warning;
  //   the two release-driven timeouts, where the trigger is already at rest, do not.
  if (!commanded) vib_stop_pending = true;   // Pattern 7: one long buzz = a FAULT stopped the system

  // Large-font stop confirm: LET_S(32) renders as "5", LET_T(20) renders as "t".
  // "5t" appearance is intentional — matches large-font style of F0-F3 confirms.
  DISP_LOCK(); displayDigits(LET_S, LET_T); updateDisplay(); DISP_UNLOCK();
  gpsKeepAliveDelay(2000);
}

// ---- Decode telemetry.rtm_distance to metres ----
// Returns -1.0f if no valid distance available (telemetry.rtm_distance == 0xFF or 0x00).
// Used by pre-arm check (Bug 1B) and R5 proximity bar.
static float decodeRtmDistanceM()
{
  uint8_t d = telemetry.rtm_distance;
  // Bug A fix: treat 0x00 as "no data" same as 0xFF.
  // 0x00 is the zero-initialized default before any RX telemetry packet arrives.
  // Previously returned 0.0m → pre-arm check saw 0.0m ≤ disengage threshold → always rejected.
  if (d == 0xFF || d == 0x00) return -1.0f;
  if (d < 100) return d / 10.0f;      // tenths of metre (0.0–9.9 m)
  return (float)(d - 90);             // whole metres (10–164 m)
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
    rtm_thr_cap_tx = 255;              // restore throttle passthrough — arm aborted
    DISP_LOCK(); for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000; updateDisplay(); DISP_UNLOCK();
    rtm_tx_state = RTM_IDLE;
    return;
  }

  if (!usrConf.rtm_double_squeeze_en)
  {
    // Single-squeeze: unlock, pause, then Pattern 4 + "r n" arm confirm
    unlockAnimation();
    gpsKeepAliveDelay(750);   // V2.5-Evo - 2026-06-05: was 250 — +500ms so the "armed" buzz is clearly separated from the squeeze
    if (current_vib_pattern == 0) current_vib_pattern = 4;   // Pattern 4 after visual unlock completes
    DISP_LOCK(); displayDigitZone("r n"); updateDisplay(); DISP_UNLOCK();
    gpsKeepAliveDelay(2000);
  }
  else
  {
    // Double-squeeze: first unlock (no P4 yet), pause, then black screen, then wait for second squeeze
    unlockAnimation();
    gpsKeepAliveDelay(250);
    DISP_LOCK(); for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000; updateDisplay(); DISP_UNLOCK();
    gpsKeepAliveDelay(800);

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
      rtm_thr_cap_tx = 255;              // restore throttle passthrough — arm aborted
      DISP_LOCK(); for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000; updateDisplay(); DISP_UNLOCK();
      rtm_tx_state = RTM_IDLE;
      return;
    }

    // Second squeeze confirmed: unlock, pause, then Pattern 4 + "r n" arm confirm
    unlockAnimation();
    gpsKeepAliveDelay(750);   // V2.5-Evo - 2026-06-05: was 250 — +500ms so the "armed" buzz is clearly separated from the squeeze
    if (current_vib_pattern == 0) current_vib_pattern = 4;   // Pattern 4 after visual unlock completes
    DISP_LOCK(); displayDigitZone("r n"); updateDisplay(); DISP_UNLOCK();
    gpsKeepAliveDelay(2000);
  }

  // Pre-arm distance check: reject if already inside the disengage threshold
  float prearm_m = decodeRtmDistanceM();
  if (prearm_m >= 0.0f && prearm_m <= (float)usrConf.rtm_disengage_distance_m)
  {
    rtm_arm_gps_timeout_override = 0;  // restore GPS threshold — arm rejected
                                        // Finding 2-1: only exit path that previously left
                                        // the 4× override stale; all other exits
                                        // (timeouts + rtmDisengage) already clear it
    rtm_thr_cap_tx = 255;              // restore throttle passthrough — arm rejected
    // Pattern 7: one long buzz = the arm was REFUSED. Kept (an arm refusal is the rider believing
    // RTM is running when it is not), and routed through the pending flag like every other stop.
    vib_stop_pending = true;
    // Large-font stop confirm on arm rejection.
    DISP_LOCK(); displayDigits(LET_S, LET_T); updateDisplay(); DISP_UNLOCK();
    gpsKeepAliveDelay(2000);
    rtm_tx_state  = RTM_IDLE;
    rtm_tx_active = false;
    queueMetaPacketBurst(0xF1, 0);
    return;
  }

  // Bug B fix: drain Serial1 NMEA backlog before activating.
  // loop() was suspended for the entire ceremony; gps_tx.encode() was not called.
  // gps_tx.location.age() has accumulated to ceremony duration (7-10s for double-squeeze).
  // Gate 2 threshold is rtm_gps_timeout_ms × 4 = 8000ms — a normal user exceeds this.
  // Processing the queued sentences refreshes age to near-zero before the first Gate 2 check.
  // 300ms cap prevents blocking indefinitely if module is sending continuous data.
  {
    unsigned long drain_start = millis();
    while (Serial1.available() && millis() - drain_start < 300UL)
    {
      gps_tx.encode(Serial1.read());
    }
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
        // BUZZES — and it is the one timeout in this file that does. Every other timeout here can
        // only fire because the trigger was already released; this gate has no throttle
        // precondition, so it fires mid-squeeze, and rtmDisengage() lifts rtm_thr_cap_tx (30-90%
        // by config range) back to 255 in the same instant. That is a silent step from capped RTM
        // throttle to raw manual throttle with no gesture behind it. Off by default
        // (rtm_max_runtime_s = 0), so keeping this buzz costs nothing against buzz saturation.
        rtmDisengage(false);
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
          rtmDisengage(false);   // FAULT, not a timeout: the TX GPS died under RTM and nothing the
                                 // rider did explains the stop → buzz
          break;
        }
      }

      // Gate 3: throttle release timeout — 4s gives time to re-apply before RTM disengages.
      // Kept hardcoded (not SPIFFS) by design. Total re-arm window after release = ~6s (4s gate + 2s cooldown).
      // V2.5-Evo - 2026-04-28 - P9 Bug1D: now calls rtmDisengage() so Pattern 4 + "St P" fire.
      if (thr_scaled < 10)
      {
        if (rtm_release_ms == 0) rtm_release_ms = now;
        if (now - rtm_release_ms > 4000UL)
        {
          // SILENT (2026-08-17 rule: a pure timeout is silent, a fault buzzes). This gate can only
          // be reached because the rider let go and kept the trigger released for 4s — the stop
          // follows his own hand and "St" already shows it. It is also the moment the cap lift in
          // rtmDisengage() is harmless: thr_scaled < 10 means the shaped throttle sits far below
          // the RTM cap, so restoring the cap to 255 changes nothing he can feel. Buzzing here
          // spends the rider's attention on a non-event and devalues the fault buzz.
          rtmDisengage(true);
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
        setRtmDisarmed();   // COMMANDED: the rider deliberately steered out (opt-in gate) → silent
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
// V2.5-Evo - 2026-04-27 - P8.1: FM redesigned as arm/disarm toggle with persistent mode memory.
//
// ARM (LEFT tap + RIGHT hold 5s, first time or after disarm):
//   - Arms at last_fm_mode (RAM; defaults to F1 on power cycle — never resets to F0/disabled)
//   - Blinks "F[mode]" x2 on display; fires Pattern 4 (2 fast buzzes) as arm confirm
//   - FM active: user engages throttle to ride
//
// CHANGE MODE while armed (LEFT hold 2s, intercepted by Hall.ino):
//   - Cycles F0→F1→F2→F3→F4→F0; stays armed; sends new mode to RX; resets arm timer
//
// DISARM (any of):
//   - Same combo again (LEFT tap + RIGHT hold 5s) — toggle
//   - Throttle release for kFmGate1ReleaseMs (30s) after first throttle input — Gate 1
//   - Arm window expires (fm_arm_window_s) before any throttle input — auto-disarm
// ============================================================

// ---- Gate 1 throttle-release disarm window ----
// How long the throttle may stay fully released (thr_scaled < 5) after the rider has
// ridden at least once before FM hard-disarms itself on the TX.
// Units: milliseconds. Compile-time only — deliberately NOT a SPIFFS field (no confStruct change).
// V2.5-Evo - 2026-07-20 - was a bare 3000UL literal. Raised 3000 -> 30000 ms: across a
// whip the rider is fully off the trigger for 10-25 s, so the 3 s timer was disarming FM at
// exactly the moment it was supposed to engage. 30 s is the hard-disarm backstop in the
// escalation chain (RX latch-clear 2 s -> TX Gate 1 30 s -> RX mode-age 95 s).
// The threshold (thr_scaled < 5) and the 5-10 dead band are UNCHANGED — timer only.
static const unsigned long kFmGate1ReleaseMs = 30000UL;

volatile bool        fm_armed         = false;  // FM arm state; RAM only, cleared on power cycle. Not static — extern'd by Display.ino (R5 bar)
                                                 // volatile: read by updateBargraphs() (task), written by loop()
static uint8_t       last_fm_mode     = 1;      // last active FM mode (1-4); defaults F1; RAM only
static unsigned long fm_arm_ms        = 0;      // time of arm, or time of last throttle >10 while armed
static bool          fm_throttle_seen = false;  // becomes true once thr_scaled>10 after arming

// Returns true if FM is currently armed; called by Hall.ino to intercept LEFT hold 2s
bool isFmArmed() { return fm_armed; }

// V2.5-Evo - 2026-07-20 - Batch T: previous telemetry.fm_flags snapshot for bit3 (fault-stop)
// rising-edge detection in runFmLoop(). Updated every runFmLoop() tick so re-arming always
// starts from a fresh baseline (no stale edge). RAM only.
static uint8_t fm_flags_prev = 0;

// ============================================================
// V2.5-Evo - 2026-07-20 - Batch T (Fable FM v1.4): FM arm-time and display readiness gating.
// All inputs are TX-LOCAL (paired flag, own GPS fix/age, last-reply age) plus the RX's own
// armed-not-ready bit — instant, zero telemetry dependency, no new confStruct field. Called
// only from the loop task: fmFundamentalReject() from cycleFmMode(), fmArmedNotReady()
// from updateR5ProximityBar() via the render path. gps_tx access stays on the loop task (invariant).
// ============================================================

// FUNDAMENTAL arm-time reject — the three conditions under which arming would be a lie AND a
// rider genuinely mid-tow cannot be in, so false-refusal ≈ 0. Everything else (transient GPS
// staleness, a momentary packet gap, high HDOP) is NOT fundamental: it arms as NOT-READY, so a
// brief mid-tow glitch never blocks the magnet. Returns true = refuse the fresh arm.
bool fmFundamentalReject()
{
  if (!usrConf.paired)                return true;   // never paired
  if (last_packet == 0)               return true;   // no RX packet ever this session
  if (!gps_tx.location.isValid())     return true;   // no TX GPS fix ever this session (isValid latches on first fix)
  return false;
}

// ARMED-NOT-READY vs READY — the OR of the RX's armed-not-ready bit (bit2) with the TX-local
// transient readiness set. Any not-ready input → true (scanner blinks in place). None → false
// (scanner sweeps). Flips live each render, so the "now it's good" moment appears for free.
bool fmArmedNotReady()
{
  if (telemetry.fm_flags & FM_FLAG_NOTREADY)                          return true;  // RX half: latch not yet proven
  if (!usrConf.paired)                                               return true;  // pairing lost
  if (last_packet == 0 || (millis() - last_packet) >= FM_LINK_HEALTHY_MS) return true;  // RX telemetry not recent / link unhealthy
  if (!(gps_tx.location.isValid() &&
        gps_tx.location.age() < (unsigned long)usrConf.tx_gps_stale_timeout_ms))  return true;  // TX GPS fix missing or stale
  return false;  // all TX-local inputs fresh AND RX not-ready clear → READY → sweep
}

// Silent disarm: clears FM state and notifies RX, but shows no display and fires no haptic.
// Used when arm-window expires before any throttle input — nothing active to confirm.
static void fmSilentDisarm()
{
  fm_armed         = false;
  fm_throttle_seen = false;
  fm_last_sync_ms  = 0;
  queueMetaPacketBurst(0xF2, 0);   // mode 0 = FM disabled on RX
}

// Internal disarm: clears state, notifies RX, shows "St" full-screen, buzzes only on a FAULT.
// V2.5-Evo - 2026-04-28 - P9 S2: showFmMode() removed; disarm shows blocking stop message.
// V2.5-Evo - 2026-04-28 - ChgE: fm_last_sync_ms reset to 0 on disarm so keepalive timer clears.
// INPUT: commanded — true = stay SILENT, false = fire the Pattern 7 STOP buzz. Since the 2026-08-17
//        revision the test is "FAULT or TIMEOUT", not "did the rider press something": true for the
//        deliberate disarms (toggle combo, magnet toggle) AND for the Gate 1 release backstop,
//        which is a pure timeout; false only for the RX fault-stop.
// OUTPUT: none. SIDE EFFECTS: fm_armed cleared, keepalive stopped, 0xF2/0 sent to RX, STOP buzz
//        requested when uncommanded, and a BLOCKING 2s "St" display hold.
static void fmDisarm(bool commanded)
{
  fm_armed         = false;
  fm_throttle_seen = false;
  fm_last_sync_ms  = 0;            // Change E: clear keepalive timer
  queueMetaPacketBurst(0xF2, 0);   // mode 0 = FM disabled on RX (followme_mode=0)
  // V2.5-Evo - 2026-08-16 - HAPTIC CUT: silent on a DELIBERATE disarm. You just did it, and the
  // display already says so. A buzz confirming your own action is noise.
  // V2.5-Evo - 2026-08-17 - StopBuzz: the caller now decides, on the rule A PURE TIMEOUT IS SILENT,
  // A FAULT BUZZES. Exactly ONE FM path is a fault — the RX fault-stop in runFmLoop(), where the RX
  // gave up steering on its own and the rider has no way to know. The Gate 1 release backstop is a
  // timer, and it expires in the worst possible place: mid-whip, where the rider is legitimately
  // off the trigger for 10-25 s and is riding a wave with no attention to spare for decoding a
  // buzz. Buzz saturation is the real failure mode here — the more buzzes there are, the less any
  // one of them is read — so Pattern 7 is spent on faults only.
  if (!commanded) vib_stop_pending = true;   // Pattern 7: one long buzz = a FAULT stopped the system

  // Large-font stop confirm on FM disarm.
  DISP_LOCK(); displayDigits(LET_S, LET_T); updateDisplay(); DISP_UNLOCK();
  gpsKeepAliveDelay(2000);
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
      fmDisarm(true);   // COMMANDED: the rider made the disarm gesture → silent
    }
    else
    {
      // No throttle yet — cycle to next mode (1→2→3→4→0 where 0 = disarm)
      last_fm_mode = (last_fm_mode < 4) ? last_fm_mode + 1 : 0;

      if (last_fm_mode == 0)
      {
        // F0: FM disabled — disarm with brief visual confirm and return to normal display.
        // Sends 0xF2/0 to RX (FM off) and resets mode to SPIFFS default. No buzz — selecting F0 is
        // a deliberate disarm the rider is watching happen on the display (comment corrected
        // 2026-08-17: it said "fires Pattern 7", which stopped being true with the 08-16 cut).
        // This is RAM-only; power cycle restores usrConf.followme_mode.
        // V2.5-Evo - 2026-08-16 - HAPTIC CUT: silent on a DELIBERATE disarm. You just did it, and

        // the display already says so - RTM/FM stops being shown and the stop confirm appears. A buzz

        // confirming your own action is noise, and it was the single most frequent buzz in the system.

        // V2.5-Evo - 2026-08-17 - and after the StopBuzz revision Pattern 7 means ONE thing:
        // a FAULT stopped the system. Timeouts — including the ones the rider did not ask for —
        // are silent, so this deliberate F0 disarm is in the majority, not the exception.

        // Large-font F0 disarm confirm: LET_F + 0. Shorter hold (1s) — this is a disarm, not a mode select.
        DISP_LOCK();
        displayDigits(LET_F, 0);
        updateDisplay();
        DISP_UNLOCK();
        gpsKeepAliveDelay(1000);
        queueMetaPacketBurst(0xF2, 0);       // tell RX: FM disabled
        fm_armed         = false;
        fm_throttle_seen = false;
        fm_last_sync_ms  = 0;
        // Reset mode to SPIFFS default so next arm starts at configured mode, not 0
        last_fm_mode = (usrConf.followme_mode >= 1 && usrConf.followme_mode <= 4)
                       ? usrConf.followme_mode : 1;
        return;
      }

      // Large-font mode confirm: LET_F + mode digit (1/2/3/4). snprintf no longer needed.
      DISP_LOCK();
      displayDigits(LET_F, last_fm_mode);
      updateDisplay();
      DISP_UNLOCK();
      gpsKeepAliveDelay(2000);
      queueMetaPacketBurst(0xF2, last_fm_mode);
      fm_last_sync_ms = millis();
      fm_arm_ms       = millis();   // reset arm window — user is actively choosing a mode
    }
    return;
  }

  // V2.5-Evo - 2026-07-20 - Batch T (Fable FM v1.4): FUNDAMENTAL arm-time reject.
  // Reached only on a FRESH arm (fm_armed was false above). Covers BOTH entry points — the
  // toggle combo AND the magnet gesture both funnel through cycleFmMode() when disarmed. On a
  // fundamental not-ready state the arm DOES NOT TAKE: fire Pattern 7 (long stop buzz) + "St",
  // leave fm_armed false, and return. Marginal/transient conditions are deliberately NOT checked
  // here — they arm as NOT-READY (fmArmedNotReady()) so the magnet still arms through a glitch.
  if (fmFundamentalReject())
  {
    // Pattern 7: one long buzz = the arm was REFUSED. Kept (the rider would otherwise walk away
    // believing FM is armed), and routed through the pending flag like every other stop request.
    vib_stop_pending = true;
    DISP_LOCK(); displayDigits(LET_S, LET_T); updateDisplay(); DISP_UNLOCK();
    gpsKeepAliveDelay(2000);
    return;                          // arm refused — no fm_armed, no 0xF2, no keepalive
  }

  // V2.5-Evo - 2026-04-28 - Change B: On first arm this session, seed last_fm_mode from SPIFFS.
  // usrConf.followme_mode is the user's configured starting mode (range 1-4; 0 is invalid here).
  // After seeding, fm_session_init_done prevents overriding any mode the user cycled to mid-session.
  if (!fm_session_init_done)
  {
    if (usrConf.followme_mode >= 1 && usrConf.followme_mode <= 4)
      last_fm_mode = usrConf.followme_mode;
    fm_session_init_done = true;
  }

  // Arm at last used mode (never arms at F0 = disabled; last_fm_mode defaults to 1)
  fm_armed         = true;
  fm_arm_ms        = millis();
  fm_throttle_seen = false;
  if (current_vib_pattern == 0) current_vib_pattern = 4;         // Pattern 4: 2 fast buzzes = arm confirm
  fm_last_sync_ms  = millis();     // Change E: start keepalive timer from now (avoids immediate re-sync)

  // V2.5-Evo - 2026-04-29 - Display: show actual mode being armed (F1/F2/F3/F4) in large font
  // instead of the generic "FM" text. Uses large num0[] font via LET_F(15) + mode digit.
  DISP_LOCK();
  displayDigits(LET_F, last_fm_mode);
  updateDisplay();
  DISP_UNLOCK();
  gpsKeepAliveDelay(2000);

  queueMetaPacketBurst(0xF2, last_fm_mode);
}

// Called by handleGearToggle(-1) simple LEFT hold 2s when FM is armed (Hall.ino checks isFmArmed()).
// Cycles mode 1→2→3→4→0 (0 disables FM); stays armed for modes 1-4 and resets arm timer.
void cycleFmModeArmed()
{
  if (!fm_armed) return;
  // Cycle 1→2→3→4→0 where 0 = disarm (FM disabled RAM-only state for hand-off).
  last_fm_mode = (last_fm_mode < 4) ? last_fm_mode + 1 : 0;

  if (last_fm_mode == 0)
  {
    // F0: FM disabled — disarm with brief visual confirm and return to normal display.
    // Sends 0xF2/0 to RX (FM off) and resets mode to SPIFFS default. No buzz — selecting F0 is a
    // deliberate disarm the rider is watching happen on the display (comment corrected 2026-08-17:
    // it said "fires Pattern 4", which stopped being true with the 08-16 cut).
    // This is RAM-only; power cycle restores usrConf.followme_mode.
    // V2.5-Evo - 2026-08-16 - HAPTIC CUT: no buzz on a mode CYCLE. You are actively

    // pressing through modes and watching the display change; the arm buzz already fired

    // once when you armed. Repeating it per mode is what made the patterns unreadable.

    // Large-font F0 disarm confirm: LET_F + 0. Shorter hold (1s) — this is a disarm, not a mode select.
    DISP_LOCK();
    displayDigits(LET_F, 0);
    updateDisplay();
    DISP_UNLOCK();
    gpsKeepAliveDelay(1000);
    queueMetaPacketBurst(0xF2, 0);       // tell RX: FM disabled
    fm_armed         = false;
    fm_throttle_seen = false;
    fm_last_sync_ms  = 0;
    // Reset mode to SPIFFS default so next arm starts at configured mode, not 0
    last_fm_mode = (usrConf.followme_mode >= 1 && usrConf.followme_mode <= 4)
                   ? usrConf.followme_mode : 1;
    return;
  }

  // Large-font mode confirm: LET_F + mode digit (1/2/3/4). snprintf no longer needed.
  DISP_LOCK();
  displayDigits(LET_F, last_fm_mode);
  updateDisplay();
  DISP_UNLOCK();
  gpsKeepAliveDelay(2000);
  queueMetaPacketBurst(0xF2, last_fm_mode);
  fm_last_sync_ms = millis();              // reset keepalive — just synced
  fm_arm_ms       = millis();             // reset arm window — user is actively choosing a mode
}

// Called from loop() every ~110ms.
// Handles arm-window auto-disarm and Gate 1 (throttle-release disarm).
void runFmLoop()
{
  unsigned long now = millis();

  // V2.5-Evo - 2026-07-20 - Batch T (Fable FM v1.4): DISARM OWNERSHIP — the display can't lie.
  // The RX owns engagement; on an RX fault it stops FM and raises fm_flags bit3 (fault-stop),
  // held sticky ~6s so this ~110ms loop is guaranteed to catch the rising edge across the
  // ~2.4s telemetry rotation. On that rising edge, while WE still believe we are armed, the TX
  // must clear its own arm and STOP re-declaring 0xF2/mode — otherwise the 30s keepalive below
  // would re-arm the RX within 30s of a fault. fmDisarm() does exactly that: fm_armed=false
  // (so this function early-returns next tick and the keepalive never fires), queues 0xF2/0
  // (belt-and-suspenders — RX already idle), and shows the stop as "St" + Pattern 7. bit3 is
  // already surprise-gated on the RX, so it is only set when the alarm is warranted — no TX
  // re-gating needed. fm_flags_prev is updated every tick (armed or not) so a re-arm starts clean.
  uint8_t fm_flags_now = telemetry.fm_flags;
  bool fault_rising = (fm_flags_now & FM_FLAG_FAULT) && !(fm_flags_prev & FM_FLAG_FAULT);
  fm_flags_prev = fm_flags_now;
  if (fm_armed && fault_rising)
  {
    // FAULT — and since the 2026-08-17 revision this is the ONLY FM path that buzzes. The RX
    // faulted and stopped following by itself: the rider asked for nothing, no timer explains it,
    // and he has no other way to learn the buggy is no longer steering for him. → commanded =
    // false → Pattern 7. (The Gate 1 release backstop below is a timeout and is now silent.)
    fmDisarm(false);   // clears fm_armed + keepalive, sends 0xF2/0, "St" + Pattern 7 — TX & RX can't disagree
    return;
  }

  if (!fm_armed) return;

  // Arm-window auto-disarm: if user never applied throttle since arming, disarm after fm_arm_window_s
  if (!fm_throttle_seen)
  {
    if (now - fm_arm_ms > (unsigned long)usrConf.fm_arm_window_s * 1000UL)
    {
      // V2.5-Evo - 2026-07-20 - Batch T (A2 D1): the silent disarm is no longer fully silent.
      // The scanner going dark is the primary signal; add ONE short blip (Pattern 5, a single
      // 150ms pulse) as a NUDGE so a rider relying on the long arm window isn't left believing
      // he is still armed. Distinct by feel from the two-tap arm (Pattern 4) and the long stop
      // buzz (Pattern 7). Placed at the call site (not inside fmSilentDisarm) so it fires ONLY on
      // arm-window expiry, never on the RTM-preemption path that also calls fmSilentDisarm().
      fmSilentDisarm();   // arm window expired before first throttle — no blocking confirm
      if (current_vib_pattern == 0) current_vib_pattern = 5;   // nudge: one short blip
      return;
    }
  }

  // Track throttle engagement; while riding, keep arm timer alive
  if (thr_scaled > 10)
  {
    fm_throttle_seen = true;
    fm_arm_ms = now;  // reset — Gate 1 timer starts from last throttle input
  }

  // Gate 1: throttle released (thr_scaled < 5) continuously for kFmGate1ReleaseMs → disarm FM.
  // The grace period allows the rider to be off the trigger — during a whip that gap is
  // routinely 10-25 s — without losing the FM declaration they made during the tow.
  if (fm_throttle_seen && thr_scaled < 5)
  {
    if (now - fm_arm_ms > kFmGate1ReleaseMs)
    {
      // SILENT — the owner's ruling, 2026-08-17. A pure timeout, not a fault: it is only reachable
      // with the trigger released (thr_scaled < 5), which on this craft means the rider is mid-whip
      // riding a wave, where a buzz is both unreadable and unhelpful. "The more buzzing there is,
      // the less attention you pay to them" — so Pattern 7 never fires in the middle of a wave and
      // stays reserved for faults. Same class as RTM Gate 3, which is also silent. Nothing steps
      // under him either: FM never touches rtm_thr_cap_tx, and he is off the trigger, so the
      // disarm changes no throttle he is currently commanding.
      fmDisarm(true);
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
