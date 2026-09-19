// V2.5-Evo - 2026-09-19 - THE RETURN GESTURE (RIGHT tap + LEFT hold, the RTM arm combo) is now returnGesture(), a three-state
//   machine read at the instant the hold completes: (1) no override and RTM not armed -> arm RTM exactly as before (setRtmArmed(),
//   ceremony, gates, rtm_arm_window_s untouched); (2) RTM armed and still inside the arm window (the blocking ceremony is waiting for
//   the squeeze) -> the ceremony's own wait loops poll for the same gesture, CANCEL the arm (0xF1/0, TX -> IDLE) and set the
//   auto-return override to the OPPOSITE of the value the buggy echoes in fm_flags bit 7 (last_fm_return_mode = !echo), Pattern 9,
//   "Ar" (ON) / "AO" (OFF) for 2 s; (3) an override standing (either value) -> back to default (last_fm_return_mode = 0xFF), Pattern 10.
//   last_fm_return_mode (RAM, 0xFF = none, like last_fm_mode) is encoded into bits 5-6 of EVERY 0xF2 the remote sends
//   (fmEncodeModeByte(): cycleFmMode(), cycleFmModeArmed(), the 30 s keepalive, every 0xF2/0 disarm burst) and cleared on FM disarm
//   and power-up; a flip while armed asks the keepalive to go out now (fmRequestKeepaliveNow()) instead of in 30 s.
//   cycleFmModeArmed() now WRAPS 1 -> 2 -> 3 -> 1 and never lands on F0 (owner rule: nothing disarms Follow-Me deliberately except
//   the disarm gesture); cycleFmMode()'s pre-throttle F0 landing is kept. Unconditional Serial prints (the remote printed nothing
//   for any RTM/FM state change before) at the RTM ceremony timeouts, the distance reject, the activation, and at every 0xF2/0
//   call site with its reason. No confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-09-18 - KEEPALIVE vs BURST QUEUE (code-review finding, water-test blocker): the 30 s 0xF2 keepalive in runFmLoop() now
//   queues only when rtm_meta_count == 0 (the single-slot queueMetaPacketBurst() is empty), otherwise retries next tick without
//   advancing fm_last_sync_ms - it used to overwrite the 0xF1/1 RTM-activation burst on the first tick after the blocking arm
//   ceremony, leaving the RX unaware RTM was on. Belt: runDoubleSqueezeArm() refreshes fm_last_sync_ms on a successful arm.
//   No confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-09-18 - RTM no longer disarms Follow-Me (owner decision 27(c), always-armed): setRtmArmed() no longer calls
//   fmSilentDisarm() and sends no 0xF2/0 - fm_armed stays true, the 30 s keepalive keeps running, last_fm_mode is untouched, the RTM
//   ceremony is unchanged. The RX yields on its own side while rtm_rx_active is set and resumes ARMED (unlatched) when RTM ends.
//   The warning-distance haptic in runFmLoop() is gated on !rtm_tx_active so a return does not buzz the FM warning all the way in.
//   No confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-09-17 - WarnDist: runFmLoop() now drives the FM warning-distance haptic (Pattern 8, one 300 ms pulse,
//   repeated every 2 s) while TX armed AND RX FM_FLAG_ARMED AND link fresh AND decoded rtm_distance >= fm_warn_distance_m.
//   No throttle gate. Never overwrites a playing pattern or a pending STOP. Pure logic in Common/FollowMeDistanceWarning.h.
// V2.5-Evo - 2026-09-17 - ArmTimeout: fm_arm_window_s → fm_arm_timeout_s; the arm-window auto-disarm in runFmLoop() now
//   runs only when usrConf.fm_arm_timeout_s > 0 (0 = never, the new default). fm_throttle_seen is unchanged.
// V2.5-Evo - 2026-09-17 - Gate1-REMOVED (Rex A2-TX, owner decision 2026-09-14): the TX 30 s throttle-release
//   disarm (kFmGate1ReleaseMs) is gone. Follow-Me is meant to stay armed for the whole session; the RX 10 s
//   latch clear and the RX 95 s mode-age expiry remain the backstops. The 30 s 0xF2 keepalive is unchanged.
// V2.5-Evo - 2026-04-25 - P7: TX RTM and FM state machines.
// RTM: left-hold gesture → arm → squeeze(s) → active → cooldown → idle
// FM:  right-hold gesture → cycle FM mode 0→1→2→3→0 → send 0xF2 meta-packet
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

// ============================================================
// V2.5-Evo - 2026-09-19 - AUTO-RETURN SESSION OVERRIDE (the followme_mode pattern, RAM only)
//
// last_fm_return_mode: this remote's override of the buggy's stored fm_return_mode for THIS session.
//   0xFF = none (the buggy uses its own stored default), 0 = OFF, 1 = ON. Set and cleared only by
//   returnGesture(); cleared on every FM disarm and lost on power-up. It rides in bits 5-6 of every
//   0xF2 the remote sends (00 none, 01 OFF, 10 ON), so a lost packet is repaired by the next 30 s
//   keepalive and a remote power cycle returns the buggy to its stored default within one.
// Declared HERE, ahead of the RTM ceremony, because the ceremony's wait loops are where the second
// gesture state lands (see returnGestureCeremonyPoll()).
// ============================================================
static uint8_t       last_fm_return_mode  = 0xFF;   // 0xFF none, 0 OFF, 1 ON; RAM only

// fmEncodeModeByte - compose the 0xF2 value byte: bits 0-2 the FM mode, bits 5-6 the override.
// Inputs: mode 0-7 (0 = disarm, 1-3 the modes). Reads last_fm_return_mode. Output: the byte.
// Side effects: none. EVERY 0xF2 goes through here so the RX always sees the current override.
static uint8_t fmEncodeModeByte(uint8_t mode)
{
  uint8_t ret_bits = 0;                              // 00 = no override: the buggy keeps its stored default
  if (last_fm_return_mode == 0)      ret_bits = 1;   // 01 = OFF for the session
  else if (last_fm_return_mode == 1) ret_bits = 2;   // 10 = ON for the session
  return (uint8_t)((mode & 0x07) | (uint8_t)(ret_bits << 5));
}

// fmRequestKeepaliveNow - ask runFmLoop()'s 30 s 0xF2 keepalive to go out on its next tick.
// Used after the override changes while FM is armed, so the buggy learns the new value in ~100 ms
// (or as soon as the single-slot burst queue is free) instead of up to 30 s later. Deliberately
// NOT a direct queueMetaPacketBurst(): the gesture that sets the override has just queued the
// 0xF1/0 cancel burst, and a second burst would overwrite it (single slot, see the 2026-09-18
// keepalive note). The keepalive path already waits for an empty slot. No-op while FM is disarmed
// (fm_last_sync_ms == 0): the next arm's 0xF2 carries the override anyway.
// Inputs: none. Side effects: rewinds fm_last_sync_ms so the keepalive is due now (never to 0,
// which the keepalive reads as "disarmed").
static void fmRequestKeepaliveNow()
{
  if (fm_last_sync_ms == 0) return;
  unsigned long t = millis() - 30000UL;
  fm_last_sync_ms = (t == 0) ? 1UL : t;
}

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
// V2.5-Evo - 2026-09-18 - RTM NO LONGER DISARMS FOLLOW-ME (owner decision 27(c), always-armed).
//   From 2026-04-28 (Bug2) to 2026-09-18 this function called fmSilentDisarm() first, sending
//   0xF2/0 so the RX dropped Follow-Me to IDLE for the whole return (Finding 1-4's concern was a
//   stale fm_mode_runtime on the RX while RTM ran). Under the always-armed philosophy that was
//   wrong: the owner's loop is "stop -> bring the buggy back -> whip again" with no re-arm gesture.
//   WHAT REPLACED IT: the RX yields on its own side (RX RTMState.ino, runFmLoopBody): while
//   rtm_rx_active is set, Follow-Me parks in FM_ARMED, clears its separation latch, writes no cap
//   and no steering, and re-engages after the return only by re-proving separation. So fm_armed
//   stays true here, the 30 s 0xF2 keepalive keeps running, last_fm_mode is untouched, and the
//   RX-side mode can never be stale because the keepalive keeps refreshing it. RTM > FM: the RTM
//   ceremony below (rtm_hold_duration_s, double squeeze, rtm_arm_window_s) is exactly as it was.
// Bug4: runDoubleSqueezeArm() now handles both single and double squeeze, fully blocking.
//       On return rtm_tx_state is RTM_ACTIVE or RTM_IDLE; RTM_ARMED case in runRtmLoop() is dead code.
void setRtmArmed()
{
  if (!usrConf.rtm_enabled || !usrConf.gps_en) return;
  // (fmSilentDisarm() was called here until 2026-09-18 - see the header note above.)
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
// V2.5-Evo - 2026-09-19 - THE RETURN GESTURE INSIDE THE ARM CEREMONY (gesture state 2)
//
// The RTM arm is a BLOCKING ceremony: setRtmArmed() runs runDoubleSqueezeArm(), which holds loop()
// - and therefore checkButtons()/handleGearToggle() - for up to rtm_arm_window_s while it waits
// for the squeeze. rtm_tx_state == RTM_ARMED exists ONLY inside that ceremony, so "the gesture done
// again while RTM is armed and inside the arm window" can only be seen from inside it. These two
// helpers are that: the wait loops call returnGestureCeremonyPoll() every 100 ms; it re-implements
// handleGearToggle()'s combo on the live toggle input (calcFilter() keeps producing tog_input during
// the ceremony because in_menu is held non-zero) - a RIGHT tap shorter than COMBO_TAP_MAX_MS, then a
// LEFT hold of rtm_hold_duration_s begun within COMBO_WINDOW_MS of the tap, with the trigger released
// (thr_scaled < 10, the same action gate). The LEFT hold that STARTED the ceremony must be released
// first; it never counts. When it fires, ceremonyCancelForReturnGesture() aborts the arm exactly
// like a timeout does (cap 255, GPS override cleared, display cleared, RTM_IDLE, 0xF1/0) and then
// flips the override: last_fm_return_mode = the OPPOSITE of the auto-return mode the buggy last
// echoed in fm_flags bit 7, Pattern 9, "Ar" (ON) / "AO" (OFF) for 2 s, keepalive requested.
//
// INVARIANT (owner's miscount hazard): a miscounted gesture can never produce motion. Arming RTM
// only declares intent - the squeeze ceremony, RTM's gates and a held trigger are still required;
// cancelling the arm removes intent; the override only changes what a Follow-Me HOLD graduates to
// on the buggy, and that graduation itself moves nothing without the trigger. The gesture is
// off-throttle only (thr_scaled < 10 at the instant the hold completes) and the ceremony's own
// squeeze detection (thr_scaled > 76) is mutually exclusive with it.
// ============================================================

// returnGestureCeremonyPoll - RIGHT tap + LEFT hold detector for the blocking arm ceremony.
// Inputs:  reset - true to clear the detector at ceremony start (the arming LEFT hold is still
//          down at that moment and must be released before anything counts). Reads ctplus() /
//          ctminus() (Hall.ino), thr_scaled, usrConf.rtm_hold_duration_s, COMBO_TAP_MAX_MS,
//          COMBO_WINDOW_MS (Hall.ino).
// Returns: true exactly once per completed gesture. Side effects: its own static state only.
static bool returnGestureCeremonyPoll(bool reset)
{
  static bool          arm_hold_released = false;   // the LEFT hold that started the ceremony has been let go
  static bool          right_was_down    = false;
  static bool          left_was_down     = false;
  static unsigned long right_down_ms     = 0;
  static unsigned long right_tap_ms      = 0;       // millis() of the last RIGHT tap; 0 = none pending
  static unsigned long left_down_ms      = 0;
  if (reset) {
    arm_hold_released = false;
    right_was_down = left_was_down = false;
    right_down_ms = right_tap_ms = left_down_ms = 0;
    return false;
  }
  const unsigned long now   = millis();
  const bool          right = ctplus();
  const bool          left  = ctminus();

  if (!arm_hold_released) {
    if (!left) arm_hold_released = true;   // the arming hold is over; from here the toggle is a fresh input
    return false;
  }

  // RIGHT: a press shorter than COMBO_TAP_MAX_MS, on release, is a tap.
  if (right && !right_was_down) right_down_ms = now;
  if (!right && right_was_down && (now - right_down_ms) < COMBO_TAP_MAX_MS) right_tap_ms = now;
  right_was_down = right;

  // LEFT: time the hold from its press edge.
  if (left && !left_was_down) left_down_ms = now;
  left_was_down = left;

  if (left && right_tap_ms != 0 &&
      (left_down_ms - right_tap_ms) < COMBO_WINDOW_MS &&
      (now - left_down_ms) >= (unsigned long)usrConf.rtm_hold_duration_s * 1000UL &&
      thr_scaled < 10) {
    right_tap_ms = 0;   // consumed: the same hold cannot fire twice
    return true;
  }
  return false;
}

// ceremonyCancelForReturnGesture - abort the RTM arm ceremony and flip the auto-return override.
// Inputs: none (reads telemetry.fm_flags for the buggy's echo). Side effects: rtm_tx_state ->
//   RTM_IDLE, rtm_thr_cap_tx 255, rtm_arm_gps_timeout_override 0, display cleared, 0xF1/0 queued;
//   last_fm_return_mode set to the opposite of the echo; Pattern 9; a BLOCKING 2 s "Ar"/"AO" hold;
//   the keepalive requested (if FM is armed). Called only from runDoubleSqueezeArm().
static void ceremonyCancelForReturnGesture()
{
  // The abort, exactly as the ceremony's own timeout paths do it.
  rtm_arm_gps_timeout_override = 0;
  rtm_thr_cap_tx = 255;
  DISP_LOCK(); for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000; updateDisplay(); DISP_UNLOCK();
  rtm_tx_state  = RTM_IDLE;
  rtm_tx_active = false;
  queueMetaPacketBurst(0xF1, 0);   // tell RX: RTM not active (belt: it already heard 0xF1/0 at ceremony start)

  // The flip: opposite of what the buggy says it is doing NOW. With the stored default ON this is
  // the only way the gesture can turn auto-return OFF for a session, and vice versa.
  const bool echo_on = (telemetry.fm_flags & FM_FLAG_RETURN_ON) != 0;
  last_fm_return_mode = echo_on ? 0 : 1;
  Serial.printf("RETURN [TX] gesture during the RTM arm: arm cancelled (0xF1/0); auto-return override set %s for this session (buggy reported %s)\n",
                last_fm_return_mode ? "ON" : "OFF", echo_on ? "ON" : "OFF");
  if (current_vib_pattern == 0) current_vib_pattern = 9;   // Pattern 9: four quick taps = override SET
  // "Ar" = auto-return ON, "AO" = OFF (the 0 glyph is the O: the 3x5 font has no lowercase o).
  DISP_LOCK(); displayDigits(LET_A, last_fm_return_mode ? LET_R : 0); updateDisplay(); DISP_UNLOCK();
  gpsKeepAliveDelay(2000);
  fmRequestKeepaliveNow();   // the 0xF1/0 burst drains first; the keepalive then carries bits 5-6
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

  returnGestureCeremonyPoll(true);   // V2.5-Evo - 2026-09-19 - fresh detector; the arming hold is still down

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
    // V2.5-Evo - 2026-09-19 - the return gesture done again while waiting = cancel + flip (state 2).
    if (returnGestureCeremonyPoll(false)) { ceremonyCancelForReturnGesture(); return; }
    delay(100);
    checkSerial();
  }
  if (!first_ok)
  {
    rtm_arm_gps_timeout_override = 0;  // ceremony aborted — restore normal GPS threshold
    rtm_thr_cap_tx = 255;              // restore throttle passthrough — arm aborted
    DISP_LOCK(); for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000; updateDisplay(); DISP_UNLOCK();
    rtm_tx_state = RTM_IDLE;
    Serial.printf("RTM [TX] arm cancelled: no first squeeze within %u s\n", (unsigned)usrConf.rtm_arm_window_s);
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
      // V2.5-Evo - 2026-09-19 - the return gesture done again while waiting = cancel + flip (state 2).
      if (returnGestureCeremonyPoll(false)) { ceremonyCancelForReturnGesture(); return; }
      delay(100);
      checkSerial();
    }
    if (!second_ok)
    {
      rtm_arm_gps_timeout_override = 0;  // ceremony aborted — restore normal GPS threshold
      rtm_thr_cap_tx = 255;              // restore throttle passthrough — arm aborted
      DISP_LOCK(); for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000; updateDisplay(); DISP_UNLOCK();
      rtm_tx_state = RTM_IDLE;
      Serial.printf("RTM [TX] arm cancelled: no second squeeze within %u s\n", (unsigned)usrConf.rtm_arm_window_s);
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
    Serial.printf("RTM [TX] arm refused: buggy is %.1f m away, inside the %u m disengage distance\n",
                  (double)prearm_m, (unsigned)usrConf.rtm_disengage_distance_m);
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
  Serial.printf("RTM [TX] ACTIVE: 0xF1/1 queued (buggy %.1f m away)\n", (double)rtm_arm_dist_m);
  // V2.5-Evo - 2026-09-18 - Belt for the single-slot burst queue: this ceremony blocked loop() for
  // 8 s or more, so an armed Follow-Me's 30 s keepalive may be overdue the instant loop() resumes.
  // Restart its clock now so it cannot fall due on top of the 0xF1/1 just queued. The keepalive
  // itself also refuses to queue while a burst is in flight (see runFmLoop()); this only keeps it
  // from trying. Only while FM is armed (fm_last_sync_ms > 0) - never starts a keepalive on its own.
  if (fm_last_sync_ms > 0) fm_last_sync_ms = millis();
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
//   - Cycles F0→F1→F2→F3→F0; stays armed; sends new mode to RX; resets arm timer
//
// DISARM (any of):
//   - Same combo again (LEFT tap + RIGHT hold 5s) — toggle
//   - Arm timeout expires (fm_arm_timeout_s) before any throttle input — auto-disarm.
//     0 = never (the default since 2026-09-17): armed with no throttle stays armed indefinitely.
//   - RX fault-stop (fm_flags bit3 rising edge) — the TX follows the RX's decision
//
// V2.5-Evo - 2026-09-18 - NOT a disarm any more: arming RTM. Follow-Me stays armed through a
// Return-to-Me; the RX yields while RTM is active and resumes ARMED (unlatched) when it ends.
// See setRtmArmed(). While RTM runs, loop() renders the RTM display as before (Display.ino checks
// rtm_tx_active first on every FM-aware path), so the rider sees RTM, and FM reappears armed after.
//
// V2.5-Evo - 2026-09-17 - Gate1-REMOVED. There is no longer a TX-side throttle-release disarm.
// The old Gate 1 (kFmGate1ReleaseMs, 30 s off the trigger after the first squeeze) contradicted
// the always-armed philosophy: a rider who floats, swims or waits between runs for more than
// 30 s was silently disarmed and had to re-do the gesture. The RX already guards the motion
// path on its own — the escalation chain is now RX latch-clear 10 s -> RX mode-age 95 s — so
// the TX keeps its arm declaration until the rider disarms, the RX faults, or power is cut.
// ============================================================

volatile bool        fm_armed         = false;  // FM arm state; RAM only, cleared on power cycle. Not static — extern'd by Display.ino (R5 bar)
                                                 // volatile: read by updateBargraphs() (task), written by loop()
static uint8_t       last_fm_mode     = 1;      // last active FM mode (1-3); defaults F1; RAM only
static unsigned long fm_arm_ms        = 0;      // time of arm, or time of last throttle >10 while armed
static bool          fm_throttle_seen = false;  // becomes true once thr_scaled>10 after arming

// Returns true if FM is currently armed; called by Hall.ino to intercept LEFT hold 2s
bool isFmArmed() { return fm_armed; }

// V2.5-Evo - 2026-07-20 - Batch T: previous telemetry.fm_flags snapshot for bit3 (fault-stop)
// rising-edge detection in runFmLoop(). Updated every runFmLoop() tick so re-arming always
// starts from a fresh baseline (no stale edge). RAM only.
static uint8_t fm_flags_prev = 0;

// ============================================================
// V2.5-Evo - 2026-09-17 - WarnDist: FM warning-distance haptic scheduler state.
//
// WHAT IT DOES: while Follow-Me is live on BOTH sides (TX armed, RX reports FM_FLAG_ARMED, link
// fresh within FM_LINK_HEALTHY_MS) and the decoded RX→TX distance is at or beyond
// usrConf.fm_warn_distance_m, the remote gives one medium buzz (Pattern 8, 300 ms) immediately
// and then one every kFmDistanceWarningPeriodMs while the condition holds. It stops when the
// distance drops below the threshold, on FM disarm, or on link loss.
//
// WHY NO THROTTLE GATE: releasing the trigger withdraws motor authority, but it must not hide
// that the buggy has reached the configured separation — that is exactly when the rider, off the
// trigger and looking at the water, needs to be told. The warning belongs to the live FM
// declaration, not to trigger posture.
//
// The decision logic (followMeDistanceWarningActive / followMeWarningPulseDue) lives in
// Common/FollowMeDistanceWarning.h so the host unit test in Tools/tests exercises the same code.
// Geometry warnings (his fm_flags bits 6/7) are NOT adopted here — they come with the P2 station work.
// ============================================================
static const unsigned long kFmDistanceWarningPeriodMs = 2000UL;  // repeat interval while the condition holds
static unsigned long fm_warning_last_ms = 0;      // millis() of the last Pattern 8 actually queued
static bool          fm_warning_sent    = false;  // true once a pulse has been queued for the current episode

// Clears the scheduler so the next episode starts with an immediate pulse. Called every tick
// that FM is not armed and every tick the condition is false — runFmLoop() runs every ~110 ms,
// so every disarm path (gesture, F0, fault, silent) is covered within one tick.
static void fmResetWarningScheduler()
{
  fm_warning_last_ms = 0;
  fm_warning_sent    = false;
}

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
  last_fm_return_mode = 0xFF;      // V2.5-Evo - 2026-09-19 - the override ends with the declaration (bits 5-6 = 00)
  queueMetaPacketBurst(0xF2, fmEncodeModeByte(0));   // mode 0 = FM disabled on RX
}

// Internal disarm: clears state, notifies RX, shows "St" full-screen, buzzes only on a FAULT.
// V2.5-Evo - 2026-04-28 - P9 S2: showFmMode() removed; disarm shows blocking stop message.
// V2.5-Evo - 2026-04-28 - ChgE: fm_last_sync_ms reset to 0 on disarm so keepalive timer clears.
// INPUT: commanded — true = stay SILENT, false = fire the Pattern 7 STOP buzz. Since the 2026-08-17
//        revision the test is "FAULT or TIMEOUT", not "did the rider press something": true for the
//        deliberate disarms (toggle combo, magnet toggle); false only for the RX fault-stop.
//        (The Gate 1 release backstop that also passed true was removed 2026-09-17.)
// OUTPUT: none. SIDE EFFECTS: fm_armed cleared, keepalive stopped, 0xF2/0 sent to RX, STOP buzz
//        requested when uncommanded, and a BLOCKING 2s "St" display hold.
static void fmDisarm(bool commanded)
{
  fm_armed         = false;
  fm_throttle_seen = false;
  fm_last_sync_ms  = 0;            // Change E: clear keepalive timer
  last_fm_return_mode = 0xFF;      // V2.5-Evo - 2026-09-19 - the override ends with the declaration (bits 5-6 = 00)
  queueMetaPacketBurst(0xF2, fmEncodeModeByte(0));   // mode 0 = FM disabled on RX (followme_mode=0)
  // V2.5-Evo - 2026-08-16 - HAPTIC CUT: silent on a DELIBERATE disarm. You just did it, and the
  // display already says so. A buzz confirming your own action is noise.
  // V2.5-Evo - 2026-08-17 - StopBuzz: the caller now decides, on the rule A PURE TIMEOUT IS SILENT,
  // A FAULT BUZZES. Exactly ONE FM path is a fault today — the RX fault-stop in runFmLoop(), where
  // the RX gave up steering on its own and the rider has no way to know. Buzz saturation is the real
  // failure mode here — the more buzzes there are, the less any one of them is read — so Pattern 7
  // is spent on faults only.
  // V2.5-Evo - 2026-09-17 - History: from 2026-07-20 to 2026-09-17 a Gate 1 release backstop (a 30 s
  // off-the-trigger disarm timer) was also classified commanded=true for this same reason. It was
  // removed because Scenario A's rider may legitimately release the trigger for up to 30 s while
  // linking waves with the buggy already parked behind the break — the whip is a separate
  // engagement gesture, not part of this release window. The RX 10 s latch clear and 95 s
  // mode-age expiry are the backstops now.
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
      Serial.println("FM [TX] disarm: the disarm gesture after riding -> 0xF2/0");   // V2.5-Evo - 2026-09-19
      fmDisarm(true);   // COMMANDED: the rider made the disarm gesture → silent
    }
    else
    {
      // No throttle yet — cycle to next mode (1→2→3→0 where 0 = disarm)
      last_fm_mode = (last_fm_mode < 3) ? last_fm_mode + 1 : 0;

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
        Serial.println("FM [TX] disarm: cycled to F0 before riding -> 0xF2/0");   // V2.5-Evo - 2026-09-19
        last_fm_return_mode = 0xFF;          // V2.5-Evo - 2026-09-19 - the override ends with the declaration
        queueMetaPacketBurst(0xF2, fmEncodeModeByte(0));       // tell RX: FM disabled
        fm_armed         = false;
        fm_throttle_seen = false;
        fm_last_sync_ms  = 0;
        // Reset mode to SPIFFS default so next arm starts at configured mode, not 0
        last_fm_mode = (usrConf.followme_mode >= 1 && usrConf.followme_mode <= 3)
                       ? usrConf.followme_mode : 1;
        return;
      }

      // Large-font mode confirm: LET_F + mode digit (1/2/3). snprintf no longer needed.
      DISP_LOCK();
      displayDigits(LET_F, last_fm_mode);
      updateDisplay();
      DISP_UNLOCK();
      gpsKeepAliveDelay(2000);
      queueMetaPacketBurst(0xF2, fmEncodeModeByte(last_fm_mode));   // V2.5-Evo - 2026-09-19 - carries the override bits
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
  if (current_vib_pattern == 0) current_vib_pattern = 4;         // Pattern 4: 2 fast buzzes = arm confirm
  fm_last_sync_ms  = millis();     // Change E: start keepalive timer from now (avoids immediate re-sync)

  // V2.5-Evo - 2026-04-29 - Display: show actual mode being armed (F1/F2/F3) in large font
  // instead of the generic "FM" text. Uses large num0[] font via LET_F(15) + mode digit.
  DISP_LOCK();
  displayDigits(LET_F, last_fm_mode);
  updateDisplay();
  DISP_UNLOCK();
  gpsKeepAliveDelay(2000);

  queueMetaPacketBurst(0xF2, fmEncodeModeByte(last_fm_mode));   // V2.5-Evo - 2026-09-19 - carries the override bits
}

// Called by handleGearToggle(-1) simple LEFT hold 2s when FM is armed (Hall.ino checks isFmArmed()).
// Cycles mode 1→2→3→1 (Change C: skips mode 0 = disabled); stays armed; resets arm timer.
// V2.5-Evo - 2026-09-19 - IT WRAPS, AND NEVER LANDS ON F0 (owner rule: nothing disarms Follow-Me
// deliberately except the disarm gesture). From 2026-04-29 (F0) until today the third LEFT hold
// while armed reached 0 = disarm, so a rider stepping through the modes on the water could disarm
// by miscounting one hold. Disarm stays on the combo-after-throttle (cycleFmMode()) and the magnet
// toggle; cycleFmMode()'s pre-throttle F0 landing is kept, because that is the deliberate disarm
// path before a tow. No buzz on a mode cycle (2026-08-16 haptic cut).
void cycleFmModeArmed()
{
  if (!fm_armed) return;
  // Cycle 1→2→3→1: wrap, never 0.
  last_fm_mode = (last_fm_mode < 3) ? last_fm_mode + 1 : 1;

  // Large-font mode confirm: LET_F + mode digit (1/2/3). snprintf no longer needed.
  DISP_LOCK();
  displayDigits(LET_F, last_fm_mode);
  updateDisplay();
  DISP_UNLOCK();
  gpsKeepAliveDelay(2000);
  queueMetaPacketBurst(0xF2, fmEncodeModeByte(last_fm_mode));   // V2.5-Evo - 2026-09-19 - carries the override bits
  fm_last_sync_ms = millis();              // reset keepalive — just synced
  fm_arm_ms       = millis();             // reset arm window — user is actively choosing a mode
}

// ============================================================
// V2.5-Evo - 2026-09-19 - returnGesture - RIGHT tap + LEFT hold, the three-state return gesture.
//
// Called by handleGearToggle() (Hall.ino) when the combo hold completes, in place of the direct
// setRtmArmed() call it used to make. The STATE IS READ AT THE INSTANT THE HOLD COMPLETES, never
// remembered from a previous press:
//   (1) no override standing and RTM not armed -> ARM RTM exactly as before: setRtmArmed(), the
//       squeeze ceremony, the gates, rtm_arm_window_s, Pattern 4 - nothing about RTM changes.
//   (2) RTM armed and inside the arm window -> the ceremony is BLOCKING loop(), so this function
//       cannot run then; the ceremony's own wait loops poll for the gesture and land this state
//       from inside (returnGestureCeremonyPoll() / ceremonyCancelForReturnGesture() above):
//       cancel the arm (0xF1/0, TX -> IDLE) and set the override to the OPPOSITE of the value the
//       buggy echoes in fm_flags bit 7. Pattern 9, "Ar" / "AO".
//   (3) an override standing (either value) -> BACK TO DEFAULT: last_fm_return_mode = 0xFF, so the
//       next 0xF2 carries bits 5-6 = 00 and the buggy uses its stored fm_return_mode. Pattern 10,
//       nothing on the display. The next gesture after this one arms RTM again (state 1).
// WHY "OPPOSITE OF THE ECHO": with the buggy's stored default ON, "set the override ON" could never
// turn auto-return OFF for a session; the opposite of the effective value is useful under either
// default. A rider who wants ON permanently sets it on the buggy, not with the gesture.
//
// INVARIANT - A MISCOUNT CAN NEVER PRODUCE MOTION. Arming RTM only declares intent: the squeeze
// ceremony, RTM's own gates and a held trigger are all still required before anything moves, and
// the wrong-state outcome of a miscount is one of "RTM armed and waiting for a squeeze", "RTM arm
// cancelled" or "override changed" - none of which is a motor command. The override only changes
// what a Follow-Me HOLD graduates to on the buggy, and that graduation moves nothing without the
// trigger either. The gesture is off-throttle only (the calcFilter() gate hands the toggle to
// steering the moment the trigger rises, and handleGearToggle() aborts a hold on thr_scaled > 3).
//
// Inputs: last_fm_return_mode, rtm_tx_state (via setRtmArmed), usrConf.rtm_enabled / gps_en.
// Side effects: state 1 - everything setRtmArmed() does (blocking); state 3 - the override
//   cleared, Pattern 10, the keepalive requested (if FM is armed), one Serial line.
// ============================================================
void returnGesture()
{
  if (last_fm_return_mode != 0xFF)
  {
    // State 3: back to default.
    last_fm_return_mode = 0xFF;
    Serial.println("RETURN [TX] gesture: auto-return override cleared -> the buggy uses its stored default (next 0xF2 carries no override)");
    if (current_vib_pattern == 0) current_vib_pattern = 10;   // Pattern 10: two medium pulses = override CLEARED
    fmRequestKeepaliveNow();
    return;
  }
  // State 1: arm RTM as before. (State 2 is landed from inside the ceremony this starts.)
  if (usrConf.rtm_enabled && usrConf.gps_en)
    setRtmArmed();
}

// Called from loop() every ~110ms.
// Handles the RX fault-stop edge, the arm-window auto-disarm and the 30 s 0xF2 keepalive.
// (The TX Gate 1 throttle-release disarm was removed 2026-09-17 — see the FM section header.)
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
    // false → Pattern 7.
    Serial.println("FM [TX] disarm: the buggy reported a Follow-Me fault stop (fm_flags bit 3) -> 0xF2/0");   // V2.5-Evo - 2026-09-19
    fmDisarm(false);   // clears fm_armed + keepalive, sends 0xF2/0, "St" + Pattern 7 — TX & RX can't disagree
    return;
  }

  if (!fm_armed)
  {
    fmResetWarningScheduler();   // WarnDist: a fresh arm always starts with an immediate pulse
    return;
  }

  // V2.5-Evo - 2026-09-17 - WarnDist: FM warning-distance haptic. See the scheduler comment block
  // above for what it does and why there is deliberately no throttle gate. Pattern 8 is
  // informational: it is queued only when no other pattern is playing and no STOP is pending, so
  // it can never mask a fault buzz. If the haptic is busy the pulse is simply retried next tick
  // (fm_warning_sent stays as it was), so a due warning is deferred, never dropped.
  {
    const bool link_fresh = (last_packet != 0) && ((now - last_packet) < FM_LINK_HEALTHY_MS);
    // V2.5-Evo - 2026-09-18 - Follow-Me now stays armed THROUGH a Return-to-Me (setRtmArmed() no
    // longer disarms it), and the RX reports FM_FLAG_ARMED while it yields - so without this term
    // every RTM run would buzz the warning-distance pulse all the way in (the buggy is far by
    // definition when the rider calls it back). The warning belongs to a live Follow-Me that is
    // actually following; while RTM owns the buggy it is parked, and the RTM display says so.
    const bool distance_warning_now = followMeDistanceWarningActive(
        fm_armed && !rtm_tx_active,
        (fm_flags_now & FM_FLAG_ARMED) != 0,
        link_fresh,
        telemetry.rtm_distance,
        usrConf.fm_warn_distance_m);

    if (!distance_warning_now)
    {
      fmResetWarningScheduler();
    }
    else if (followMeWarningPulseDue(true, fm_warning_sent, fm_warning_last_ms, now, kFmDistanceWarningPeriodMs)
             && current_vib_pattern == 0 && !vib_stop_pending)
    {
      current_vib_pattern = 8;   // Pattern 8: one 300 ms pulse (System.ino vibrationTask)
      fm_warning_last_ms  = now;
      fm_warning_sent     = true;
    }
  }

  // Arm-timeout auto-disarm: if the rider never applied throttle since arming, disarm after
  // fm_arm_timeout_s. V2.5-Evo - 2026-09-17: gated on fm_arm_timeout_s > 0 — 0 means NEVER, and is
  // the default. An armed remote with no throttle now stays armed until the rider disarms it, the
  // RX faults, or power is cut (always-armed philosophy, owner decision 2026-09-15).
  if (!fm_throttle_seen && usrConf.fm_arm_timeout_s > 0)
  {
    if (now - fm_arm_ms > (unsigned long)usrConf.fm_arm_timeout_s * 1000UL)
    {
      // V2.5-Evo - 2026-07-20 - Batch T (A2 D1): the silent disarm is no longer fully silent.
      // The scanner going dark is the primary signal; add ONE short blip (Pattern 5, a single
      // 150ms pulse) as a NUDGE so a rider relying on the long arm window isn't left believing
      // he is still armed. Distinct by feel from the two-tap arm (Pattern 4) and the long stop
      // buzz (Pattern 7). Placed at the call site (not inside fmSilentDisarm) so it fires ONLY on
      // arm-window expiry, never on the RTM-preemption path that also called fmSilentDisarm()
      // (that path is gone since 2026-09-18 - RTM no longer disarms FM - so this is now its only caller).
      Serial.printf("FM [TX] disarm: armed %u s with no throttle (fm_arm_timeout_s) -> 0xF2/0\n", (unsigned)usrConf.fm_arm_timeout_s);   // V2.5-Evo - 2026-09-19
      fmSilentDisarm();   // arm window expired before first throttle — no blocking confirm
      if (current_vib_pattern == 0) current_vib_pattern = 5;   // nudge: one short blip
      return;
    }
  }

  // Track throttle engagement. fm_throttle_seen is what turns the arm combo into a disarm
  // toggle (cycleFmMode) once the rider has ridden, and what ends the arm-window check above.
  // fm_arm_ms is still refreshed here, but since the Gate 1 removal (2026-09-17) nothing reads
  // it once fm_throttle_seen is set — kept as-is rather than widening this change.
  if (thr_scaled > 10)
  {
    fm_throttle_seen = true;
    fm_arm_ms = now;
  }

  // V2.5-Evo - 2026-09-17 - Gate1-REMOVED. The block that lived here disarmed FM after
  // kFmGate1ReleaseMs (30 s) of thr_scaled < 5 once fm_throttle_seen was set. Deleted on the
  // owner's decision (Rex A2-TX): the TX keeps its arm across any length of release. The RX
  // side is unchanged and still owns the motion path — its 10 s latch clear and 95 s mode-age
  // expiry are the backstops.

  // V2.5-Evo - 2026-04-28 - Change E: Send 0xF2 keepalive every 30s while FM is armed.
  // Ensures RX stays in the correct FM mode after any transient packet loss.
  //
  // V2.5-Evo - 2026-09-18 - NEVER CLOBBER AN IN-FLIGHT BURST (code-review finding, water-test
  // blocker). queueMetaPacketBurst() is a SINGLE SLOT (Radio.ino): a new call overwrites
  // rtm_meta_type/value and restarts the 3-packet count, so whatever burst was still being sent is
  // simply lost on the air. WHY THAT BIT HERE: the toggle RTM arm blocks loop() inside
  // runDoubleSqueezeArm() for 8 s or more, during which this function does not run and
  // fm_last_sync_ms goes stale; the ceremony ends by queueing 0xF1/1 (RTM ACTIVE), loop() resumes,
  // and on its very first tick this keepalive - now 30 s overdue - queued 0xF2/mode on top of it
  // before sendData() (100 ms cycle) had sent a single 0xF1/1 packet. The TX then showed RTM
  // ACTIVE with rtm_thr_cap_tx ramping while the RX never learned RTM was on: no gate 9, no
  // approach cap, no Phase C, no RTM steering - the trigger drove the buggy straight ahead.
  // Measured ~30-50 % of toggle arms once Follow-Me started staying armed through a return.
  // THE FIX: queue the keepalive only when the slot is EMPTY; otherwise retry next tick and do NOT
  // advance fm_last_sync_ms, so the retry keeps coming every ~110 ms until the slot frees (a burst
  // drains in ~300 ms). The acquire load pairs with the release store in queueMetaPacketBurst().
  // Belt: runDoubleSqueezeArm() also refreshes fm_last_sync_ms at the end of a successful
  // ceremony, so the keepalive is not overdue on resume in the first place.
  // (A 2-deep queue and a TX check of fm_status bit 1 are the longer-term answer; not built here.)
  if (fm_last_sync_ms > 0 && now - fm_last_sync_ms >= 30000UL)
  {
    if (rtm_meta_count.load(std::memory_order_acquire) == 0)
    {
      queueMetaPacketBurst(0xF2, fmEncodeModeByte(last_fm_mode));   // V2.5-Evo - 2026-09-19 - carries the override bits
      fm_last_sync_ms = now;
    }
    // else: a burst (0xF1 RTM state, 0xF4 aux, or an earlier 0xF2) is still going out — retry next tick.
  }
}
