// V2.5-Evo - 2026-08-17 - StopBuzz FIX: fmDisarm() takes a `commanded` flag; the magnet toggle's two
//   disarm paths pass "commanded" (silent) because removing the magnet IS the rider asking. The
//   magnet ADVISORY buzzes (Patterns 5 and 6) are unchanged. The 2026-07-20 tag below is a dated
//   record: since the 2026-08-16 cut, a deliberate disarm no longer fires Pattern 7.
// V2.5-Evo - 2026-07-20 - StopFeel: comment-only sync — every STOP/DISARM confirm now fires Pattern 7
//   (one 400ms long buzz), not Pattern 4. Arm confirms still Pattern 4. Feel map + disarm-path comments
//   below updated to match; no code change in this file.
// V2.5-Evo - 2026-07-20 - MagGesture FIX2: the magnet is now a TOGGLE, mimicking the toggle-combo.
//   A >=2s hold + release toggles FM (mode 1/3) or RTM (mode 2/3-at-5s): if the mode is disarmed it
//   arms (as before); if it is ARMED it disarms via the SAME path the toggle uses — fmDisarm() for FM
//   (0xF2/0, Pattern 7, "St") and setRtmDisarmed()→rtmDisengage() for RTM (0xF1/0, Pattern 7, "St").
//   Was arm-only (no-op when already armed). Threshold, vibration and arm behaviour are UNCHANGED.
// V2.5-Evo - 2026-07-20 - MagGesture: magnet/Hall arm gesture (runMagGesture()) added — hold magnet
//   >=2s <5s then REMOVE = arm FM; hold >=5s then REMOVE = arm RTM. Advisory buzz at each threshold.
//   Reads P_MAG without touching the SW33b bt_dot_state machine.
//   Role selected by the new mag_mode SPIFFS field (0=off/not fitted default, 1=FM, 2=RTM, 3=FM+RTM).
// V2.5-Evo - 2026-05-16 - SW56: stop WiFi AP synchronously before unlockAnimation() — AP was running during frames, WiFi stack tasks preempted Core 0 causing last-frame stutter on first boot unlock only
// V2.5-Evo - 2026-07-18 - Arm-hold now SPIFFS-tunable: combo hold reads rtm_hold_duration_s (RTM LEFT-hold) / fm_hold_duration_s (FM RIGHT-hold) instead of a hardcoded 5000ms. Both 4-10s (ConfigService-clamped). No struct/SW_VERSION change.
// V2.5-Evo - 2026-07-20 - Hold-duration floor lowered 4s → 3s (comment only in this file; see handleGearToggle for why the floor is 3 not 2 — it must exceed the hardcoded 2000ms simple-hold). No code change here.
// V2.5-Evo - 2026-04-25 - P7: handleGearToggle() left-hold arms RTM; right-hold cycles FM
// V2.5-Evo - 2026-04-21 - Updated DISPLAY_MODE_SPEED availability check to support TX GPS speed sources
// V2.5-Evo - 2026-04-27 - P8: Gesture redesign — combo state machine; LEFT hold=display cycle; RIGHT+LEFT=RTM; LEFT+RIGHT=FM
// V2.5-Evo - 2026-04-27 - fix: COMBO_TAP_MAX_MS 500ms; tap detection was tied to gear_change_waittime (100ms — too tight)
// V2.5-Evo - 2026-04-27 - fix: restored correct gesture map — RIGHT hold=display cycle, LEFT hold=lock (P8 had them swapped)
// V2.5-Evo - 2026-04-28 - Change1: post-unlock delay 500→250ms; throttle-release settling 1000→500ms
// V2.5-Evo - 2026-05-06 - FIX-GESTURE-1: COMBO_TAP_MAX_MS 500ms→1000ms. In no_gears mode the 100ms display cycle confused users into holding the tap longer than 500ms, causing has_combo=false and LEFT-hold-5s to fall into the 2s LOCK branch instead of arming RTM (~70% failure rate per Andres field report).

// Returns true if the given display mode has a valid value
bool isDisplayModeAvailable(uint8_t mode)
{
  switch(mode) {
    case DISPLAY_MODE_TEMP:   return telemetry.foil_temp  != 0xFF;
    // V2.5-Evo - 2026-04-21 - When a TX-GPS speed unit is selected (speed_src 2/3/5),
    // SPEED mode is always available (shows "--" when no fix, live value otherwise).
    // For RX-sourced speed, availability still depends on the telemetry sentinel.
    case DISPLAY_MODE_SPEED:
      if (usrConf.speed_src == 2 || usrConf.speed_src == 3 || usrConf.speed_src == 5)
        return true;
      return telemetry.foil_speed != 0xFF;
    case DISPLAY_MODE_POWER:  return telemetry.foil_power != 0xFF;
    case DISPLAY_MODE_BAT:    return telemetry.foil_bat   != 0xFF;
    case DISPLAY_MODE_THR:    return true;
    case DISPLAY_MODE_AMP:    return telemetry.foil_motor_amps != 0xFF;
    case DISPLAY_MODE_INTBAT: return true;
    default: return false;
  }
}

// Cycle display_mode in given direction (+1 or -1), skipping unavailable modes
void cycleDisplayMode(int direction)
{
  uint8_t start = display_mode;
  uint8_t next = display_mode;
  for(uint8_t i = 0; i < DISPLAY_MODE_COUNT; i++) {
    next = (next + DISPLAY_MODE_COUNT + direction) % DISPLAY_MODE_COUNT;
    if(isDisplayModeAvailable(next)) break;
  }
  display_mode = next;
  DISP_LOCK();
  switch(display_mode) {
    case DISPLAY_MODE_TEMP:   displayDigits(LET_T, LET_P); break;
    case DISPLAY_MODE_SPEED:  displayDigits(5, LET_P); break;
    case DISPLAY_MODE_POWER:  displayDigits(LET_P, LET_V); break;
    case DISPLAY_MODE_BAT:    displayDigits(LET_B, LET_A); break;
    case DISPLAY_MODE_THR:    displayDigits(LET_T, LET_H); break;
    case DISPLAY_MODE_AMP:    displayDigits(LET_M, LET_A); break;
    case DISPLAY_MODE_INTBAT: displayDigits(LET_U, LET_B); break;
  }
  updateDisplay();
  DISP_UNLOCK();
  delay(500);
}

//100us
void calcFilter()
{
  // Not volatile — these are local stack variables, not shared between tasks
  uint32_t thr_sum = 0;
  uint32_t tog_sum = 0;
  uint32_t intbat_sum = 0;

  for(int i = 0; i < BUFFSZ; i++)
  {
    thr_sum += thr_raw[i];
    tog_sum += tog_raw[i];
    intbat_sum += intbat_raw[i];
  }

  uint16_t thr_filter = thr_sum / BUFFSZ;
  uint16_t tog_filter = tog_sum / BUFFSZ;
  uint16_t intbat_filter = intbat_sum / BUFFSZ;

  //Map Throttle (guard against div-by-zero from corrupted config)
  if(usrConf.thr_idle == usrConf.thr_pull)
  {
    thr_scaled = 0;
  }
  else if(usrConf.thr_idle < usrConf.thr_pull)
  {
    uint16_t thr_const = constrain(thr_filter, usrConf.thr_idle, usrConf.thr_pull);
    thr_scaled = (uint8_t)((long)(thr_const - usrConf.thr_idle) * 255 / (usrConf.thr_pull - usrConf.thr_idle));
  }
  else
  {
    uint16_t thr_const = constrain(thr_filter, usrConf.thr_pull, usrConf.thr_idle);
    thr_scaled = 255 - (uint8_t)((long)(thr_const - usrConf.thr_pull) * 255 / (usrConf.thr_idle - usrConf.thr_pull));
  }

  //Deadzone mid-toggle (clamp to prevent uint16_t underflow with bad config)
  uint16_t halfDead = usrConf.tog_deadzone / 2;
  uint16_t deadbandLower = (halfDead < usrConf.tog_mid) ? usrConf.tog_mid - halfDead : 0;
  uint32_t upperSum = (uint32_t)usrConf.tog_mid + halfDead;
  uint16_t deadbandUpper = (upperSum <= 0xFFFF) ? (uint16_t)upperSum : 0xFFFF;
  
  if(tog_filter >= deadbandLower && tog_filter <= deadbandUpper)
  {
    tog_scaled = 127;
  }
  else
  {
    //Map toggle (guard against div-by-zero from corrupted config)
    if(usrConf.tog_left == usrConf.tog_right)
    {
      tog_scaled = 127;
    }
    else if(usrConf.tog_left < usrConf.tog_right)
    {
      uint16_t tog_const = constrain(tog_filter, usrConf.tog_left, usrConf.tog_right);
      if (tog_const < deadbandLower) 
      {
        // Map from left to deadbandLower → 0 to 126
        tog_scaled = (uint8_t)((long)(tog_const - usrConf.tog_left) * 126 / (deadbandLower - usrConf.tog_left));
      }
      else
      {
        // Map from deadbandUpper to right → 128 to 255
        tog_scaled = (uint8_t)(128 + (long)(tog_const - deadbandUpper) * 127 / (usrConf.tog_right - deadbandUpper));
      }
    }
    else
    {
      uint16_t tog_const = constrain(tog_filter, usrConf.tog_right, usrConf.tog_left);
      if (tog_const > deadbandUpper)
      {
        // Map from deadbandUpper to left → 126 to 0
        tog_scaled = 126 - (uint8_t)((long)(tog_const - deadbandUpper) * 126 / (usrConf.tog_left - deadbandUpper));
      }
      else
      {
        // Map from right to deadbandLower → 255 to 128
        tog_scaled = 255 - (uint8_t)((long)(tog_const - usrConf.tog_right) * 127 / (deadbandLower - usrConf.tog_right));
      }
    }
  }

  //Calc Bat Voltage
  int_bat_volt = (float)intbat_filter * usrConf.ubat_cal;


  //Block toggle input when steering
  if((thr_scaled > 3 && system_locked == 0 && !in_menu && usrConf.steer_enabled)||(throttleForceToggleBlock() && !remote_error && !in_setup))
  {
    //If so, block steer and reset counter
    toggle_blocked_by_steer = 1;
    toggle_blocked_counter = 0;
  }
  else
  {
    if(!remote_error)
    {
      //If trigger was released, increment
      if(toggle_blocked_counter < usrConf.tog_block_time)
      {
        if(steer_scaled != 127)
        {
          toggle_blocked_counter = 0;
        }
        else
        {
          toggle_blocked_counter ++;
        }
      }
      //Until usrConf.tog_block_time reached, then unlock toggle
      else
      {
        if(usrConf.tog_block_time != 0)
        {
          toggle_blocked_by_steer = 0;
        }
      }
    }
    else
    {
      toggle_blocked_counter = usrConf.tog_block_time;
      toggle_blocked_by_steer = 0;
    }
  }
  //If in steer mode, update steering
  if(toggle_blocked_by_steer)
  {
    steer_scaled = tog_scaled;
    tog_input = 0;
  }
  else
  {
    steer_scaled = 127;
    if(tog_scaled > 127+ usrConf.tog_diff) tog_input = 1;
    else if(tog_scaled < 127- usrConf.tog_diff) tog_input = -1;
    else tog_input = 0;
  }
}

// Apply exponential throttle curve with x^2 shaping
uint8_t expoThrCurve(uint8_t thr_scaled_linear) 
{
  float x = thr_scaled_linear / 255.0f;  // Normalize input (0.0 to 1.0)
  float expo = usrConf.thr_expo;

  // Map expo 0–100 to -1.0 to +1.0 range (0 = strong negative, 50 = linear, 100 = strong positive)
  float expo_weight = (expo - 50.0f) / 50.0f;

  // Negative expo: blend toward 1 - (1 - x)^2 (flattening)
  // Positive expo: blend toward x^2 (sharpening)
  float curve;
  if (expo_weight < 0) {
      float neg_curve = 1.0f - (1.0f - x) * (1.0f - x); // Negative exponential
      curve = x + expo_weight * (x - neg_curve); // Blend away from linear
  } else {
      float pos_curve = x * x; // Positive exponential
      curve = x + expo_weight * (pos_curve - x); // Blend toward curve
  }

  // Scale back to 0–255 and clamp
  int result = (int)(curve * 255.0f + 0.5f); // Round
  if (result < 0) result = 0;
  if (result > 255) result = 255;

  return (uint8_t)result;
}

bool ctplus()
{
  return tog_input == 1;
}

bool ctminus()
{
  return tog_input == -1;
}

// ============================================================
// V2.5-Evo - 2026-04-27 - P8: COMBO GESTURE STATE MACHINE
//
// Direction convention (unchanged since V2):
//   Physical LEFT toggle → tog_input = -1 → handleGearToggle(-1) → direction = -1
//   Physical RIGHT toggle → tog_input = +1 → handleGearToggle(+1) → direction = +1
// NOTE: P8 initially set LEFT hold = display cycle and removed lock (wrong).
// Corrected: RIGHT hold = display cycle; LEFT hold = lock (matches user intent).
//
// Tap = press released before COMBO_TAP_MAX_MS (1000ms). Recorded as last_tap_dir.
// A tap that lasts 100ms–500ms will also fire a gear/cap change as a side effect
// (gear_change_waittime = 100ms), but the tap is still recorded for combo purposes.
// Combo = opposite tap within COMBO_WINDOW_MS followed by a long hold.
//
// Gesture map:
//   RIGHT hold 2s (simple)             → cycle telemetry display mode
//   LEFT hold 2s (simple)              → lock remote (unlock: left hold + throttle touch)
//   RIGHT tap → LEFT hold 5s (combo)   → arm RTM
//   LEFT tap → RIGHT hold 5s (combo)   → FM mode cycle
// ============================================================
static int           last_tap_dir   = 0;    // last recorded tap direction: +1=right, -1=left, 0=none
static unsigned long last_tap_ms    = 0;    // millis() when last tap was recorded
static const unsigned long COMBO_WINDOW_MS  = 3000UL;  // max gap between tap and hold for combo
// Separate from gear_change_waittime — gives users a comfortable ~500ms window to perform
// a tap without needing sub-100ms precision. A slightly-long tap may also adjust gear/cap
// (side effect) but still primes the combo correctly.
static const unsigned long COMBO_TAP_MAX_MS = 1000UL;

// direction: -1 = left toggle press, +1 = right toggle press
void handleGearToggle(int direction)
{
  bool (*isActive)() = (direction < 0) ? ctminus : ctplus;

  in_menu = usrConf.menu_timeout + 1;
  delay(50);
  unsigned long pushtime = millis();
  bool change_once       = 1;
  bool long_press_done   = false;

  // Combo valid only if opposite direction tap happened within the window
  bool has_combo = (last_tap_dir != 0) &&
                   (last_tap_dir != direction) &&
                   (millis() - last_tap_ms < COMBO_WINDOW_MS);

  // Combo holds: RTM arm (LEFT hold, direction<0) uses rtm_hold_duration_s;
  // FM cycle (RIGHT hold, direction>0) uses fm_hold_duration_s — both 3-10s, SPIFFS-tunable.
  // Simple holds = 2s (hardcoded below).
  //
  // V2.5-Evo - 2026-07-20 - The ConfigService floor is 3s (lowered from 4s), NOT 2s, and the
  // 3 is deliberate — do not lower it. The combo hold MUST exceed the hardcoded 2000ms simple
  // hold: a "tap" is any press < COMBO_TAP_MAX_MS (1000ms) and routine gear/cap changes are
  // taps, so an ordinary opposite-direction tap within COMBO_WINDOW_MS (3000ms) sets has_combo.
  // If the combo hold could equal 2s it would fire at the exact instant the user expects the
  // simple 2s action (lock / display-cycle), with no dead-time window to signal "this is the
  // other gesture". At 3s the user still gets ~1s of separation, so the two gestures stay
  // distinct.
  unsigned long long_press_ms;
  if (has_combo)
    long_press_ms = (unsigned long)(direction < 0 ? usrConf.rtm_hold_duration_s
                                                  : usrConf.fm_hold_duration_s) * 1000UL;
  else
    long_press_ms = 2000UL;

  while (isActive())
  {
    delay(10);

    if (millis() - pushtime > long_press_ms)
    {
      if (thr_scaled < 10)
      {
        if (has_combo)
        {
          if (direction < 0 && last_tap_dir == 1)
          {
            // RIGHT tap + LEFT hold 5s → arm RTM
            if (usrConf.rtm_enabled && usrConf.gps_en)
              setRtmArmed();
          }
          else if (direction > 0 && last_tap_dir == -1)
          {
            // LEFT tap + RIGHT hold 5s → FM mode cycle
            if (usrConf.fm_override_enabled && usrConf.gps_en)
              cycleFmMode();
          }
        }
        else if (direction > 0)
        {
          // Simple RIGHT hold 2s → cycle telemetry display mode
          cycleDisplayMode(1);
        }
        else if (direction < 0)
        {
          if (isFmArmed())
          {
            // FM armed: LEFT hold 2s → cycle FM mode (stays armed)
            cycleFmModeArmed();
          }
          else if (!usrConf.no_lock)
          {
            // FM not armed: LEFT hold 2s → lock remote
            system_locked = 1;
            DISP_LOCK(); displayLock(); DISP_UNLOCK();
          }
        }
        last_tap_dir   = 0;  // consume the tap after any long-press action
        long_press_done = true;
        in_menu = usrConf.menu_timeout;
      }
      while (isActive()) delay(100);
      break;
    }

    if (millis() - pushtime > usrConf.gear_change_waittime)
    {
      if (change_once)
      {
        switch (usrConf.throttle_mode)
        {
          case 0: // Gears
          default:
            if (direction < 0 && gear > 0) gear--;
            else if (direction > 0 && gear < usrConf.max_gears - 1) gear++;
            showNewGear();
            break;
          case 1: // No gears — cycle display
            cycleDisplayMode(direction);
            break;
          case 2: // Dynamic cap
            throttleAdjustCap(direction);
            showCapPercent();
            break;
        }
        change_once = 0;
        in_menu = usrConf.menu_timeout;
      }
    }
  }

  unsigned long held_ms = millis() - pushtime;

  // Record tap if released before COMBO_TAP_MAX_MS (1000ms). Decoupled from gear_change_waittime
  // (100ms) so that a tap that also fires a gear/cap change still primes the combo correctly.
  // Bug fix: old threshold was gear_change_waittime (100ms from pushtime after 50ms initial delay
  // = ~150ms total from press). This window was too tight — any tap over ~150ms total was silently
  // dropped and last_tap_dir was never set, so combos never triggered.
  if (!long_press_done && held_ms < COMBO_TAP_MAX_MS)
  {
    last_tap_dir = direction;  // +1 or -1
    last_tap_ms  = millis();
  }

  if (!long_press_done)
  {
    while (isActive()) delay(10);
    delay(50);
    while (millis() - pushtime < usrConf.gear_display_time)
    {
      runMenu();
      delay(10);
    }
    in_menu = usrConf.menu_timeout;
  }
}

// ============================================================
// V2.5-Evo - 2026-07-20 - MAGNET (HALL) ARM GESTURE
//
// WHAT IT DOES
//   Lets the rider arm Follow-Me or Return-To-Me by holding a magnet against the
//   potted case and then taking it away. Faster and more reliable in the water than
//   the tap+hold toggle combos, and it works through the sealed housing.
//
//   What the gesture arms is user-selectable via the mag_mode SPIFFS field (0-3).
//   magGestureRole() decodes it; see the mode table on mag_mode in BREmote_V2_Tx.h.
//   The Hall sensor is OPTIONAL EXTRA HARDWARE, so mag_mode defaults to 0 (off).
//
//   MAG_ROLE_BOTH (mag_mode 3) — the full two-tier gesture:
//     Magnet held        Feedback while holding          On magnet REMOVAL
//     -----------        ----------------------          -----------------
//     < 2s               none                            nothing (accident guard)
//     >= 2s and < 5s     ONE pulse at 2s   (Pattern 5)   arm FM   (cycleFmMode())
//     >= 5s              THREE pulses at 5s (Pattern 6)  arm RTM  (setRtmArmed())
//
//   Haptic feel map — the advisory and the confirm that follows it are always different,
//   so the rider can tell from the buzz alone which mode they just armed:
//     1 pulse  then 2 fast (Pattern 4) = FM armed
//     3 pulses then 2 fast (Pattern 4) = RTM armed
//   Disarm/stop (magnet re-toggle, or any FM/RTM stop) = ONE long 400ms buzz (Pattern 7) — a single
//   sustained buzz, deliberately unlike the two/three fast taps of an arm confirm, so the rider can
//   tell arm from stop by feel alone.
//
//   MAG_ROLE_FM (mag_mode 1) / MAG_ROLE_RTM (mag_mode 2) — single-tier. There is no second
//   tier to disambiguate, so the 2s threshold is the only one: one pulse (Pattern 5) at 2s,
//   arm that one mode on release. The 5s threshold is not used in these roles.
//
//   MAG_ROLE_NONE (mag_mode 0, the default) — dormant. The function returns immediately
//   and the Hall sensor behaves exactly as it did before this feature existed.
//
// WHY THE ACTION FIRES ON REMOVAL, NOT ON THE THRESHOLD
//   A 5s RTM hold necessarily passes through the 2s FM threshold on its way. If FM
//   armed at the 2s mark, the rider would get an FM arm they never asked for, and RTM
//   would then have to preempt it a few seconds later. So the 2s / 5s buzzes are purely
//   advisory — they mean "let go now and you will get X". The arming happens only when
//   the magnet actually leaves.
//
// TOGGLE (v2)
//   The gesture mimics what the toggle-combo can do: it both arms AND disarms. On removal, if the
//   selected mode is disarmed it arms; if it is armed it disarms through the toggle's own disarm
//   path — fmDisarm() for FM, setRtmDisarmed()/rtmDisengage() for RTM — so the haptic feel and the
//   RX effect are identical to the toggle-combo disarm. FM and RTM stay mutually exclusive: RTM
//   active/arming blocks any FM toggle, and arming RTM disarms FM first (setRtmArmed()).
//
// ARMING WHILE ON THE THROTTLE IS INTENTIONAL
//   Unlike the toggle combos, this gesture does NOT require a released throttle. The approved
//   FM design has the rider arm during the tow, while on the trigger — the toggle physically
//   cannot do that (it doubles as the steering control whenever thr_scaled > 3), which is a
//   large part of why this gesture exists. Arming only declares intent; it moves nothing.
//   FM and RTM each still enforce all of their own conditions, and neither can produce motion
//   without a held trigger. See the guard block in the removal branch below.
//
// HOW THIS AVOIDS DISTURBING THE BT STATUS DOT
//   The SW33b block in V2_Integration_Tx.ino loop() owns bt_dot_state and owns setting
//   mag_seen_high. This function only ever READS digitalRead(P_MAG) and mag_seen_high.
//   It keeps its own private debounce/timer state and writes nothing the dot machine uses,
//   so the dot behaves exactly as before.
//
// INPUTS:  P_MAG (GPIO 9, DRV5032FADBZR, LOW = magnet present), mag_seen_high boot guard
// OUTPUTS: none (void)
// SIDE EFFECTS: may call cycleFmMode()/setRtmArmed() to arm, or fmDisarm()/setRtmDisarmed() to
//   disarm — all of which BLOCK for several seconds (display confirms / squeeze ceremony) and may
//   fire current_vib_pattern. MUST therefore be called from loop() only — never from a FreeRTOS task.
// ============================================================

// current_vib_pattern is defined in System.ino, which the Arduino build concatenates
// AFTER Hall.ino, so it needs an extern here.
extern volatile uint8_t current_vib_pattern;
// rtmIsArming() is defined in RTMState.ino (also concatenated after this file).
bool rtmIsArming();
// fmDisarm() and setRtmDisarmed() are the toggle-combo's own disarm paths (both static in
// RTMState.ino, concatenated after this file). Declared static here — matching their definitions
// so the linkage agrees — so the magnet TOGGLE can fire the identical disarm the toggle uses
// (fmDisarm: 0xF2/0 + "St"; setRtmDisarmed→rtmDisengage: 0xF1/0 + "St" — both silent, because a
// gesture disarm is a stop the rider asked for).
// V2.5-Evo - 2026-08-17 - fmDisarm() now takes a `commanded` flag (true = the rider asked for the
// stop → silent, false = a safety gate stopped it → Pattern 7). The magnet toggle always passes
// true: removing the magnet IS the rider asking. setRtmDisarmed() is unchanged — it is the
// deliberate-stop wrapper and passes commanded = true internally.
static void fmDisarm(bool commanded);
static void setRtmDisarmed();

// ---- Gesture timing constants (compile-time only — deliberately NOT SPIFFS fields, no confStruct change) ----
static const uint32_t kMagFmHoldMs   = 2000UL;   // hold >= this and release before kMagRtmHoldMs → arm FM
static const uint32_t kMagRtmHoldMs  = 5000UL;   // hold >= this → arm RTM on release
// Software debounce on top of the DRV5032's own hysteresis. A marginal magnet position can
// still flutter the pin; the level must read the same for this long before it is accepted.
// 120ms is well under the 2000ms shortest meaningful hold, so it cannot mask a real gesture.
static const uint32_t kMagDebounceMs = 120UL;
static const uint32_t kMagPollMs     = 20UL;     // sampling interval — matches the SW33b dot poll rate
// Parked-magnet guard: if the magnet stays present longer than this, the rider is not making
// a gesture — the remote is stowed against something magnetic. The gesture is abandoned and
// removal does nothing. Without this, un-stowing the remote hours later would arm RTM.
static const uint32_t kMagMaxHoldMs  = 30000UL;

// ---- Called from loop() every cycle; self-rate-limits to kMagPollMs ----
void runMagGesture()
{
  // Private debounce + timing state. Kept separate from the SW33b dot machine's state
  // so the two never interfere.
  static uint32_t mag_next_poll_ms = 0;   // next millis() at which we sample the pin
  static bool     mag_raw_last     = false;  // previous raw sample (true = magnet present)
  static uint32_t mag_raw_since    = 0;   // millis() when the current raw run started
  static bool     mag_stable_low   = false;  // debounced level (true = magnet present)
  static uint32_t mag_hold_start   = 0;   // millis() of the accepted magnet-arrival edge
  static bool     fm_advised       = false;  // 2s advisory buzz already fired this hold
  static bool     rtm_advised      = false;  // 5s advisory buzz already fired this hold
  static bool     hold_abandoned   = false;  // parked-magnet guard tripped this hold

  // Role gate. With mag_mode == 0 (the default — no Hall sensor fitted) the gesture does not
  // exist: bail out before touching any state, so the Hall behaves exactly as it did before
  // this feature was added and a user without the optional magnet sees zero change.
  // Re-read every call (not cached) so a live web-UI config change takes effect immediately.
  uint8_t role = magGestureRole();
  if (role == MAG_ROLE_NONE)
  {
    // Reset state so switching roles mid-session cannot inherit a half-finished hold.
    mag_raw_last   = false;
    mag_stable_low = false;
    hold_abandoned = false;
    return;
  }

  uint32_t now = millis();
  if ((int32_t)(now - mag_next_poll_ms) < 0) return;
  mag_next_poll_ms = now + kMagPollMs;

  // Boot guard (SW33): until GPIO 9 has been seen HIGH at least once since power-up we cannot
  // tell "rider is holding a magnet" from "a magnet was already sitting there when it booted".
  // mag_seen_high is set by the bt_dot_state block in loop(); we only read it.
  if (!mag_seen_high)
  {
    mag_raw_last   = false;
    mag_stable_low = false;
    return;
  }

  // ---- Debounce: a level must persist for kMagDebounceMs before it is accepted ----
  bool raw_low = (digitalRead(P_MAG) == LOW);   // LOW = magnet present
  if (raw_low != mag_raw_last)
  {
    mag_raw_last  = raw_low;
    mag_raw_since = now;   // start timing this new run
  }

  bool edge_accepted = false;
  if (raw_low != mag_stable_low && (now - mag_raw_since) >= kMagDebounceMs)
  {
    mag_stable_low = raw_low;
    edge_accepted  = true;
  }

  // ---- Magnet arrived: start timing the hold ----
  // The hold clock is set to mag_raw_since (the real electrical edge), not to now, so the
  // debounce window is not silently subtracted from the rider's 2s / 5s hold.
  if (edge_accepted && mag_stable_low)
  {
    mag_hold_start = mag_raw_since;
    fm_advised     = false;
    rtm_advised    = false;
    hold_abandoned = false;
    return;
  }

  // ---- Magnet present: fire the advisory buzzes as each threshold is crossed ----
  if (mag_stable_low && !hold_abandoned)
  {
    uint32_t held = now - mag_hold_start;

    if (held >= kMagMaxHoldMs)
    {
      // Parked magnet — abandon this hold entirely; removal will do nothing.
      hold_abandoned = true;
      return;
    }
    // Advisories are ONLY hints about what removal would do. They arm nothing.
    // Guarded on current_vib_pattern == 0 so an advisory never stomps a warning
    // pattern (signal drop, low battery, E71) that is already playing.
    // The 5s tier exists only in MAG_ROLE_BOTH; the single-role modes stop at 2s.
    if (role == MAG_ROLE_BOTH && !rtm_advised && held >= kMagRtmHoldMs)
    {
      rtm_advised = true;
      // Pattern 6 = three fast buzzes = "release for RTM". Deliberately NOT Pattern 4:
      // Pattern 4 is the arm confirm that setRtmArmed() fires moments later, and two
      // identical double-buzzes back to back are indistinguishable by feel.
      if (current_vib_pattern == 0) current_vib_pattern = 6;
    }
    else if (!fm_advised && held >= kMagFmHoldMs)
    {
      fm_advised = true;
      // One short buzz = "release now". In MAG_ROLE_BOTH that means FM; in the
      // single-role modes it means whichever mode this remote is configured for.
      if (current_vib_pattern == 0) current_vib_pattern = 5;
    }
    return;
  }

  // ---- Magnet removed: this is where the arming actually happens ----
  if (edge_accepted && !mag_stable_low)
  {
    // Duration is measured to mag_raw_since (the real departure edge) for the same reason
    // the arrival edge is used above.
    uint32_t held = mag_raw_since - mag_hold_start;
    bool     was_abandoned = hold_abandoned;

    // Clear per-hold state before doing anything blocking.
    fm_advised     = false;
    rtm_advised    = false;
    hold_abandoned = false;

    if (was_abandoned) return;          // parked-magnet guard tripped
    if (held < kMagFmHoldMs) return;    // accident guard — too short to mean anything

    // ---- Common preconditions: states in which no arm gesture should be honoured at all ----
    if (system_locked) return;                      // remote locked — no arming from a stowed remote
    if (in_setup) return;                           // mid-calibration / setup
    if (remote_error && !remote_error_blocked) return;  // unacknowledged error on screen

    // NOTE — deliberately NO throttle-released check here, unlike handleGearToggle().
    // handleGearToggle() requires thr_scaled < 10 because the toggle IS the steering control
    // whenever the rider is on the trigger (see calcFilter(): thr_scaled > 3 sets
    // toggle_blocked_by_steer). That guard resolves an INPUT CONFLICT on the toggle; it is not
    // a safety rule. The magnet is an independent input with no such conflict, so the check
    // does not transfer.
    // It also must not transfer: the approved FM design has the rider arm DURING the tow, i.e.
    // while on throttle — something the toggle physically cannot do. Requiring a released
    // throttle here would remove the one capability that justifies this gesture existing.
    // Safety is unaffected: arming only DECLARES INTENT. It moves nothing. FM and RTM each
    // still require every one of their own conditions plus a held trigger before any motion.

    // ---- Decide which mode this hold asked for ----
    // MAG_ROLE_BOTH is the only two-tier role: >=5s means RTM, otherwise FM.
    // The single-role modes have one threshold (2s, already checked above), so the
    // hold length beyond 2s is irrelevant — they always arm their one configured mode.
    bool want_rtm;
    if (role == MAG_ROLE_BOTH)      want_rtm = (held >= kMagRtmHoldMs);
    else if (role == MAG_ROLE_RTM)  want_rtm = true;
    else                            want_rtm = false;   // MAG_ROLE_FM

    if (want_rtm)
    {
      // ---- toggle RTM ----
      // Bail out entirely if RTM isn't usable — same guard the toggle path applies.
      if (!(usrConf.rtm_enabled && usrConf.gps_en)) return;
      if (rtm_tx_active || rtmIsArming())
      {
        // RTM already active (or mid arm-ceremony) → DISARM through the toggle's own path.
        // setRtmDisarmed()→rtmDisengage(true) sends 0xF1/0 and shows "St", no buzz — identical
        // feel and RX effect to a toggle-combo disengage. (Mid-ceremony is only theoretical here:
        // runDoubleSqueezeArm() blocks loop(), so runMagGesture() cannot be entered while arming.)
        setRtmDisarmed();
      }
      else
      {
        // RTM disarmed → ARM. setRtmArmed() is only the gesture half of RTM arming: it disarms FM
        // (mutual exclusion), sets RTM_ARMED, zeroes rtm_thr_cap_tx, then runs the blocking
        // runDoubleSqueezeArm() throttle-squeeze ceremony — exactly as the toggle path does.
        setRtmArmed();
      }
    }
    else
    {
      // ---- toggle FM ----
      // Mutual exclusion: never touch FM while RTM is active or mid-ceremony.
      if (rtm_tx_active || rtmIsArming()) return;
      if (!(usrConf.fm_override_enabled && usrConf.gps_en)) return;
      if (isFmArmed())
      {
        // FM already armed → DISARM via the toggle-combo's own disarm path. fmDisarm(true) sends
        // 0xF2/0 and shows "St" with no buzz — so the magnet disarm feels and behaves exactly like the
        // toggle disarm the owner used successfully. (This is a hard disarm, not cycleFmMode()'s
        // arm/cycle/disarm behaviour: the magnet is a pure arm↔disarm toggle.)
        fmDisarm(true);   // COMMANDED: the magnet gesture IS the rider asking → silent
      }
      else
      {
        // FM disarmed → ARM. cycleFmMode() arms at last_fm_mode when FM is not currently armed.
        cycleFmMode();
      }
    }

    // setRtmArmed() / cycleFmMode() block for seconds. The magnet may have been re-applied
    // in the meantime, so resynchronise the debounce state to the pin as it is right now.
    // A new gesture then requires a fresh, fully debounced magnet-arrival edge.
    mag_raw_last     = (digitalRead(P_MAG) == LOW);
    mag_stable_low   = mag_raw_last;
    mag_raw_since    = millis();
    mag_hold_start   = millis();
    hold_abandoned   = mag_stable_low;  // magnet still there on return → treat as parked, not a new gesture
    mag_next_poll_ms = millis() + kMagPollMs;
  }
}

void runMenu()
{
  if(remote_error == 0 || remote_error_blocked == 1)
  {
    if(system_locked)
    {
      if(ctminus())
      {
        in_menu = usrConf.menu_timeout+1;
        delay(300);
        if(ctminus())
        {
          //To unlock, prompt user to touch throttle once
          advanceArrow();
          unsigned long timeout = millis();
          while(thr_scaled < 200 && (millis()-timeout < usrConf.trig_unlock_timeout ))
          {
            advanceArrow();
            delay(100);
          }
          if(millis()-timeout < usrConf.trig_unlock_timeout)
          {
            setHallActivityEnabled(true);
            setRadioActivityEnabled(true);
#ifdef WIFI_ENABLED
            webCfgNotifyTxUnlocked();  // stop WiFi AP before animation — eliminates WiFi-task preemption during frames
#endif
            unlockAnimation();
            delay(250);
            while(thr_scaled > 5)
            {
              delay(100);
            }
            delay(500);
            system_locked = 0;
            throttleReset();
            in_menu = usrConf.menu_timeout;
          }
        }
      }
    }
    //System is NOT locked
    else
    {
      if(ctminus())
      {
        handleGearToggle(-1);
      }
      else if(ctplus())
      {
        handleGearToggle(1);
      }
    }
  }
  //Handle errors
  else
  {
    if(ctminus() || ctplus())
    {
      in_menu = usrConf.menu_timeout+1;
      delay(500);
      if(ctminus() || ctplus())
      {
        remote_error = 0;
        displayError(DASH);
        while(ctminus() || ctplus()) delay(1);
        remote_error_blocked = 1;
        unsigned long pushtime = millis();
        while(millis() - pushtime < usrConf.err_delete_time)
        {
          runMenu();
          delay(10);
        }
        remote_error_blocked = 0;
        in_menu = usrConf.menu_timeout;
      }
    }
  }
}

void readFilteredInputs(uint16_t &thr_out, uint16_t &tog_out)
{
  uint32_t thr_sum = 0;
  uint32_t tog_sum = 0;
  for(int i = 0; i < BUFFSZ; i++)
  {
    thr_sum += thr_raw[i];
    tog_sum += tog_raw[i];
  }
  thr_out = thr_sum / BUFFSZ;
  tog_out = tog_sum / BUFFSZ;
}

void checkCal()
{
  #define CAL_MIN_DIFF 1000

  //Check if calibration is OK
  if(!usrConf.cal_ok)
  {
    Serial.println("Entering Calibration...");

    displayDigits(LET_E, LET_C);
    updateDisplay();
    delay(2000);

    Serial.println("Hands Off!");

    displayDigits(0, LET_F);
    updateDisplay();
    delay(3000);

    uint16_t thr_filter_raw, tog_filter_raw;
    readFilteredInputs(thr_filter_raw, tog_filter_raw);

    usrConf.thr_idle = thr_filter_raw;
    usrConf.tog_mid =  tog_filter_raw;

    Serial.println("Full throttle!");
    for(int i = 0; i < 30; i++)
    {
      advanceArrow();
      delay(100);
    }

    readFilteredInputs(thr_filter_raw, tog_filter_raw);

    usrConf.thr_pull = thr_filter_raw;

    Serial.println("Toggle left");
    displayDigits(TLT, TLT);
    updateDisplay();
    delay(3000);

    readFilteredInputs(thr_filter_raw, tog_filter_raw);

    usrConf.tog_left = tog_filter_raw;

    Serial.println("Toggle right");
    displayDigits(TGT, TGT);
    updateDisplay();
    delay(3000);

    readFilteredInputs(thr_filter_raw, tog_filter_raw);

    usrConf.tog_right = tog_filter_raw;

    //Check if cal values are in range

    bool cal_in_range = 1;

    if(usrConf.thr_idle < usrConf.thr_pull)
    {
      if(usrConf.thr_pull - usrConf.thr_idle > CAL_MIN_DIFF)
      {
        usrConf.thr_pull -= usrConf.cal_offset;
        usrConf.thr_idle += usrConf.cal_offset;
      }
      else
      {
        Serial.println("Throttle out of range!");
        cal_in_range = 0;
      }
    }
    else
    {
      if(usrConf.thr_idle - usrConf.thr_pull > CAL_MIN_DIFF)
      {
        usrConf.thr_pull += usrConf.cal_offset;
        usrConf.thr_idle -= usrConf.cal_offset;
      }
      else
      {
        Serial.println("Throttle out of range!");
        cal_in_range = 0;
      }
    }

    if(usrConf.tog_left > usrConf.tog_mid && usrConf.tog_mid > usrConf.tog_right)
    {
      if(usrConf.tog_left - usrConf.tog_mid > CAL_MIN_DIFF &&  usrConf.tog_mid - usrConf.tog_right > CAL_MIN_DIFF)
      {
        usrConf.tog_left -= usrConf.cal_offset;
        usrConf.tog_right += usrConf.cal_offset;
      }
      else
      {
        Serial.println("Toggle out of range!");
        cal_in_range = 0;
      }
    }
    else if(usrConf.tog_left < usrConf.tog_mid && usrConf.tog_mid < usrConf.tog_right)
    {
      if(usrConf.tog_mid - usrConf.tog_left > CAL_MIN_DIFF &&  usrConf.tog_right - usrConf.tog_mid > CAL_MIN_DIFF)
      {
        usrConf.tog_left += usrConf.cal_offset;
        usrConf.tog_right -= usrConf.cal_offset;
      }
      else
      {
        Serial.println("Toggle out of range!");
        cal_in_range = 0;
      }
    }
    else
    {
      Serial.println("Toggle out of range!");
      cal_in_range = 0;
    }

    Serial.print("THR_IDLE: ");
    Serial.println(usrConf.thr_idle);
    Serial.print("THR_PULL: ");
    Serial.println(usrConf.thr_pull);
    Serial.print("TOG_LEFT: ");
    Serial.println(usrConf.tog_left);
    Serial.print("TOG_MID: ");
    Serial.println(usrConf.tog_mid);
    Serial.print("TOG_RIGHT: ");
    Serial.println(usrConf.tog_right);

    if(cal_in_range)
    {
      usrConf.cal_ok = 1;
      Serial.println("Cal Done.");
      saveConfToSPIFFS(usrConf);
      scroll4Digits(5, LET_A, LET_V, LET_E, 120);
      scroll4Digits(5, LET_A, LET_V, LET_E, 120);
    }
    else
    {
      usrConf.cal_ok = 0;
      Serial.println("Cal Error!");
      displayDigits(LET_E, LET_C);
      updateDisplay();
      while(1) delay(100);
    }
  }
}
