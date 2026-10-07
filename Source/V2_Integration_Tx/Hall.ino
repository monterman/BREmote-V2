// V2.5-Evo - 2026-10-07 - SOP-040 gesture rule: the mag_mode 4 2.5 s hold acts only with the trigger fully released
//   (checked at 2.5 s and at removal; held -> ignored silently, one serial line). The toggle RIGHT tap + LEFT hold logs
//   when it is ignored because the trigger is held. Magnet taps unchanged. No confStruct change.
// V2.5-Evo - 2026-10-07 - P-11: checkCal() sets ads_cal_in_progress while it calibrates (plausibility exemption).
// V2.5-Evo - 2026-10-07 - SOP-040: the mag_mode 4 hold refusal (Return-To-Me disabled or GPS off) is now "St" + the
//   normal stop buzz on removal, replacing the "n0" screen and the Pattern 5 blip at 2.5 s. No confStruct change.
// V2.5-Evo - 2026-10-07 - H-1 (TX part): the LEFT-hold lock now flushes 0xF1/0 + 0xF2/0 (rtmFmStopFlush()).
// V2.5-Evo - 2026-10-07 - C-1: runMagGesture() ignores every magnet gesture while the throttle input fault is latched
//   (ads_input_fault). The toggle is already blocked by the ADC task (tog_input forced to 0). No confStruct change.
// V2.5-Evo - 2026-10-07 - comment only: a station step no longer buzzes (Pattern 11 removed, RTMState.ino); the tap
//   lockout note is updated. The 1 s lockout itself is unchanged.
// V2.5-Evo - 2026-10-07 - R-4: stale comments corrected (the 2.5 s hold "toggles Return-To-Me", Pattern 10 "will
//   toggle") and the fmToggleAutoReturnFromMagnet() prototype removed with the function. Comments only otherwise.
// V2.5-Evo - 2026-10-07 - R-3: runMenu() ignores the toggle after an RTM arm ceremony until it has been seen
//   centred (ceremony_toggle_latch, set by runDoubleSqueezeArm()), so a LEFT toggle still held for the in-ceremony
//   RIGHT tap -> LEFT hold can no longer fall through into a gear step, station change or lock when the ceremony
//   ends. No confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-10-07 - mag_mode 4 hold, two owner rulings (audits R-2 and R-7). The 2.5 s hold now decides at the
//   2.5 s mark what removal will do (magHoldVerdict(), latched): (1) a Return-To-Me ALREADY ACTIVE -> the hold is
//   ignored, no buzz, the return continues (it used to restart the arm ceremony with no "St" and no cooldown);
//   (2) Return-To-Me cannot start (disabled or GPS off) -> one short refusal blip (Pattern 5) instead of the
//   Pattern 10 success cue, and "n0" for 2 s on removal (it used to promise with Pattern 10 and then do nothing);
//   (3) otherwise Pattern 10 and the manual Return-To-Me ceremony, as before. No confStruct change, sizeof stays
//   136, SW_VERSION stays 27.
// V2.5-Evo - 2026-10-06 - mag_mode 4, two owner rulings. (1) M-1 TAP LOCKOUT: after a magnet tap actually steps the
//   Follow-Me station, any tap whose magnet ARRIVES within kMagStepLockoutMs (1000 ms) is ignored completely - no
//   step, no arm, no buzz. Since the F-label hold stopped blocking loop(), two quick taps (or one wobbly one) could
//   step two stations while the rider felt only one count. (2) THE 2.5 s HOLD ALWAYS STARTS MANUAL RETURN-TO-ME,
//   whatever the Follow-Me state (was: FM armed -> A1/A0 auto-return toggle). A1/A0 stays reachable from the "rn"
//   wait (RIGHT tap then LEFT hold). No confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-09-30 - MagFix (Rex delta audit of 98fb7a8): runMagGesture() gains a SAMPLE-GAP GUARD (H-1) —
//   a hold is abandoned if loop() stalled long enough that the magnet pin went unsampled, so a blocking disarm,
//   disengage or arm ceremony can no longer promote a 300 ms tap into the 2.5 s hold and silently turn
//   Return-To-Me off. Applies to EVERY mag_mode role, because all of them time their holds off wall-clock.
//   kMagTapMaxMs 400 -> 600 ms (M-2): the real sample interval is ~110 ms (one loop() iteration), not the
//   20 ms kMagPollMs suggests, so a genuine 300 ms tap could measure ~410 ms and be dropped. Every comment
//   that claimed 20 ms sampling, or reasoned from it, is corrected. The mode 4 RTM gate now reads the
//   effective enable (rtmEnabledEffective()) so a session override is honoured. The stray-magnet claim in
//   the header is corrected: a stray tap CAN arm Follow-Me (the owner asked for that), it just cannot move a
//   station and cannot produce motion — no_lock = 0 (boots locked) is the real mitigation.
//   Comments, timing and guard logic only: no confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-09-30 - MagStations: runMagGesture() gains mag_mode 4 (MAG_ROLE_FMSET). A short TAP (60-600 ms)
//   steps through the Follow-Me stations selected in the new mag_fm_set bitmask, and a 2.5 s hold toggles
//   Return-To-Me for the session. Roles 1-3 keep their own behaviour: same 120 ms debounce, same 2 s / 5 s
//   thresholds, same advisories, same arm-on-removal. See the MAG_ROLE_FMSET block inside runMagGesture().
//   (The 2026-09-30 MagFix entry above DOES touch roles 1-3 in one respect: the sample-gap guard protects
//   their holds too, because the wall-clock weakness it fixes was never specific to mode 4.)
//   — GPIO 9 (P_MAG) IS AN ESP32-C3 STRAPPING PIN. NEVER POWER THE REMOTE ON WITH THE MAGNET ATTACHED.
// V2.5-Evo - 2026-09-19 - RIGHT tap + LEFT hold now calls returnGesture() (RTMState.ino) instead of setRtmArmed() directly: the same
//   combo is a three-state machine (arm RTM as before / cancel the arm and flip the auto-return override for the session / back to
//   default). The gesture map comment below is updated. The magnet FM disarm prints its reason (the remote printed nothing for any
//   RTM/FM state change before). No confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-09-18 - comment only (review finding F8): the magnet-gesture header said "arming RTM disarms FM first"; corrected to the yield behaviour. No code change.
// V2.5-Evo - 2026-09-18 - comment only: the magnet RTM-arm branch said setRtmArmed() disarms FM; it no longer does (Follow-Me stays armed through a return since 2026-09-18). No code change.
// V2.5-Evo - 2026-09-17 - GestureAbort (Rex B7 case 2): handleGearToggle() now aborts any toggle hold the
//   moment the trigger is squeezed (thr_scaled > 3 with steer_enabled, the same gate calcFilter() uses to hand
//   the toggle to steering). Applies to the simple 2s holds, both combo holds and the post-action release wait. On abort
//   in_menu is zeroed so the next calcFilter() pass returns the toggle to steering, and a throttle_abort flag
//   stops every pending action: no gear/mode/lock step fires, no tap is recorded, no menu wait runs.
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
//   RIGHT tap → LEFT hold 5s (combo)   → the return gesture (returnGesture(), RTMState.ino):
//                                         arm RTM / cancel the arm + flip auto-return / back to default
//   LEFT tap → RIGHT hold 5s (combo)   → FM mode cycle
// ============================================================
// V2.5-Evo - 2026-09-19 - defined in RTMState.ino (concatenated after this file).
void returnGesture();
// V2.5-Evo - 2026-10-07 - H-1: defined in RTMState.ino (concatenated after this file); used by the lock branch.
void rtmFmStopFlush();
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

  // V2.5-Evo - 2026-09-17 - GestureAbort. While this handler runs, in_menu is non-zero, so
  // calcFilter() leaves the toggle in menu mode even if the rider squeezes the trigger — and
  // with the trigger squeezed the toggle IS the steering control. Before this fix a squeeze
  // mid-hold left the rider unable to steer until the hold finished or timed out, and a
  // gear/mode/lock action could still fire on top of it (Rex B7 failure case 2).
  //
  // The fix: any throttle above the calcFilter() steering threshold (thr_scaled > 3) abandons
  // the gesture immediately. in_menu = 0 lets the very next calcFilter() pass (10 ms task)
  // route the toggle back to steer_scaled, and throttle_abort guarantees nothing that was
  // pending — a long-press action, a gear step, a tap record, the post-tap menu wait — can
  // fire after the break. The thr_scaled < 10 action gate below is kept as a second, coarser
  // guard for the instant the hold timer expires.
  //
  // Gated on usrConf.steer_enabled exactly like the calcFilter() gate it mirrors: with steering
  // disabled the toggle is a gear/cap selector only, riders shift gears with the trigger
  // squeezed, and there is no steering role to hand the toggle back to.
  bool throttle_abort = false;

  while (isActive())
  {
    delay(10);

    if (thr_scaled > 3 && usrConf.steer_enabled)
    {
      throttle_abort = true;
      in_menu = 0;
      break;
    }

    if (millis() - pushtime > long_press_ms)
    {
      if (thr_scaled < 10)
      {
        if (has_combo)
        {
          if (direction < 0 && last_tap_dir == 1)
          {
            // RIGHT tap + LEFT hold 5s → the return gesture. V2.5-Evo - 2026-09-19: it arms RTM as
            // before when no auto-return override stands (the rtm_enabled / gps_en gate is inside),
            // and otherwise clears the override; the cancel-and-flip state is landed from inside
            // the arm ceremony. See returnGesture() in RTMState.ino.
            returnGesture();
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
            // V2.5-Evo - 2026-10-07 - H-1 (TX part): a locked remote sends zero throttle, but a buggy left in
            // Return-To-Me or Follow-Me would stay there. Send "RTM off" + "FM off" and let them go out
            // (>= 400 ms, RTMState.ino). Follow-Me is never armed here (that branch is above).
            rtmFmStopFlush();
          }
        }
        last_tap_dir   = 0;  // consume the tap after any long-press action
        long_press_done = true;
        in_menu = usrConf.menu_timeout;
      }
      else if (has_combo && direction < 0 && last_tap_dir == 1)
      {
        // V2.5-Evo - 2026-10-07 - SOP-040 gesture rule: the RIGHT tap + LEFT hold Return-To-Me gesture acts only
        // with the trigger fully released. It always did nothing here with the trigger held; now it says so on
        // serial (silently on the remote, as the rule asks).
        Serial.printf("RTM [TX] return gesture ignored: trigger held (thr %u) - it needs the trigger fully released\n",
                      (unsigned)thr_scaled);
      }
      // Release wait after the action. A squeeze here also hands the toggle straight back to
      // steering instead of holding the rider in menu mode until the toggle is centred.
      while (isActive())
      {
        if (thr_scaled > 3 && usrConf.steer_enabled)
        {
          throttle_abort = true;
          in_menu = 0;
          break;
        }
        delay(100);
      }
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
  // GestureAbort 2026-09-17: a hold abandoned by a squeeze is not a tap — recording one would
  // prime a combo the rider never asked for.
  if (!long_press_done && !throttle_abort && held_ms < COMBO_TAP_MAX_MS)
  {
    last_tap_dir = direction;  // +1 or -1
    last_tap_ms  = millis();
  }

  // GestureAbort 2026-09-17: skipped on a throttle abort — this block waits for toggle release
  // and then re-arms in_menu for gear_display_time, which would take steering away again.
  if (!long_press_done && !throttle_abort)
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
//   MAG_ROLE_FMSET (mag_mode 4) — V2.5-Evo - 2026-09-30 - MagStations. A DIFFERENT SHAPE OF GESTURE:
//     Magnet held        Feedback while holding          On magnet REMOVAL
//     -----------        ----------------------          -----------------
//     < 60ms             none                            nothing (debounce, not a gesture)
//     60-600ms (a TAP)   none (too short to buzz)        step to the next station in mag_fm_set,
//                                                        or ARM Follow-Me if it is not armed yet
//     600ms - 2.5s       none                            nothing (deliberate dead zone, see below)
//     >= 2.5s            TWO medium pulses (Pattern 10)  start the MANUAL Return-To-Me ("rn", squeeze to confirm)
//     (V2.5-Evo - 2026-10-06: the hold row used to read "toggle Return-To-Me on/off", and from 2026-10-02 it
//     toggled auto-return instead while Follow-Me was armed. Owner ruling: it ALWAYS starts the manual recall.)
//     V2.5-Evo - 2026-10-07 - two exceptions to that row (audits R-2, R-7): while a Return-To-Me is ALREADY
//     ACTIVE the hold is ignored (no buzz, no action); when Return-To-Me cannot start (disabled, or GPS off)
//     the 2.5 s buzz is ONE short blip (Pattern 5) instead of Pattern 10, and removal shows "n0" for 2 s.
//     V2.5-Evo - 2026-10-07 - SOP-040: superseded - no buzz while holding, and removal shows "St" with the stop buzz.
//     V2.5-Evo - 2026-10-07 - SOP-040 GESTURE RULE: the hold acts only with the trigger FULLY RELEASED (checked at
//     2.5 s and again at removal). With the trigger held it is ignored silently - no buzz, no "St", no ceremony.
//     The TAP row is unchanged: with the trigger held a tap still arms Follow-Me and still steps the station.
//     TAP LOCKOUT (V2.5-Evo - 2026-10-06, audit M-1): after a tap that actually stepped the station, a tap
//     whose magnet arrives within kMagStepLockoutMs (1 s) is ignored completely. See the constant.
//
//     MODE 4 HAS NO MAGNET DISARM. Roles 1 and 3 make the magnet an arm↔disarm toggle; in mode 4 the
//     magnet only ARMS Follow-Me or STEPS its station, and the hold only starts the manual Return-To-Me.
//     To disarm Follow-Me in mode 4, use the toggle combo (LEFT tap → RIGHT hold). This is a deliberate
//     difference.
//
//     WHY THE 600ms - 2.5s DEAD ZONE IS DELIBERATE. A tap is about 300 ms and the hold is 2500 ms, and the
//     tap CEILING sits at 600 ms, so the two bands stay more than 4x apart end-to-end. That is what makes
//     them impossible to confuse with cold hands, wet hands, gloves or one-handed in chop. A hold in the
//     dead zone is silent: the rider feels no buzz, nothing happens, and they simply tap again.
//
//     WHY THE TAP CEILING IS 600ms AND NOT 400ms (V2.5-Evo - 2026-09-30, Rex delta audit M-2). The original
//     400 ms ceiling was specified against a 20 ms sample rate that DOES NOT EXIST in this firmware. This
//     function is reached once per loop() iteration and loop() ends in vTaskDelay(110), so the real magnet
//     sample interval is ~110 ms, not 20 ms — kMagPollMs is only a floor, never the actual cadence. With
//     ±110 ms of quantisation a true 300 ms tap could measure ~410 ms and be silently dropped. 600 ms
//     absorbs that jitter and still leaves the tap band and the 2.5 s hold band far apart. A future option
//     (NOT done here) is to move the P_MAG sample into a dedicated 20 ms task and leave only the action in
//     loop(); that would make the sample rate match the original design instead of widening the window.
//
//     THE FIRST TAP IS NEVER A NO-OP, AND NEVER SAYS "YOU ARE ALREADY THERE":
//       Follow-Me not armed          -> ARM it at the stored default station (cycleFmMode())
//       Follow-Me actively following -> move to the next station set in mag_fm_set, wrapping
//       ...at a station NOT in the set -> move to the lowest station that IS in the set
//       already at the only set station -> NOTHING AT ALL, and no buzz (fmStepStationFromMagnet())
//
//     — THE HARD GATE: A STATION ONLY MOVES WHILE FOLLOW-ME IS ACTIVELY FOLLOWING.
//     Never while the rider is on the rope under tow. On the rope the rider is physically attached to the
//     buggy and cannot steer away from it, so a buggy that repositions itself on its own decision would
//     drag him. Once Follow-Me has ENGAGED the rope is slack — the rider is riding independently after
//     the whip and the buggy is trailing — so a station transit pulls nobody. fmIsEngaged() is the test.
//     Armed-but-not-engaged is exactly the tow state, and the tap is dead in it.
//
//     WHAT A STRAY MAGNET CAN AND CANNOT DO — CORRECTED V2.5-Evo - 2026-09-30 (Rex delta audit). An earlier
//     version of this comment claimed a stray magnet "does nothing at all unless Follow-Me is already
//     following". THAT WAS FALSE and is corrected here, because a comment that overstates a guard is worse
//     than no comment. The truth:
//       - A stray tap CANNOT move the buggy's station. That is hard-gated on fmIsEngaged() above, and it is
//         the case the safety rule is actually about.
//       - A stray tap CAN ARM Follow-Me, on an unlocked and paired remote that already has a GPS fix and a
//         live link. This is the owner's explicit requirement ("if fm mode never armed then yes it arms at
//         default mode ie fm3, then again tap goes to fm4"), so the behaviour stays.
//       - NO MOTION RESULTS FROM THAT ARM. Arming only declares intent: the RX still needs its separation
//         latch proven and the rider still has to hold the trigger before anything moves, and Follow-Me can
//         only ever SUBTRACT from the rider's throttle.
//       - THE REAL MITIGATION IS THE LOCK, NOT THE GATE. This whole function returns early while
//         system_locked is set, and the owner ships no_lock = 0, so a remote in a bag or a car boots LOCKED
//         and no magnet gesture of any kind is honoured until the rider unlocks it deliberately.
//     A REFUSED GESTURE IS COMPLETELY SILENT: buzzing for "I did nothing" would train the rider to expect
//     feedback from accidental contact. Counting taps was rejected as the guard because it has a measured
//     reliability cost on competitor water remotes.
//
// — HARDWARE HAZARD THE RIDER MUST KNOW: GPIO 9 (P_MAG) IS AN ESP32-C3 STRAPPING PIN.
//   A magnet held against the case at power-up or reset puts the chip into UART DOWNLOAD MODE, and the
//   remote will not boot normally. NEVER POWER THE REMOTE ON WITH THE MAGNET ATTACHED. The mag_seen_high
//   boot guard below cannot prevent this: strapping is sampled by the silicon at reset, before any
//   firmware runs. The guard only stops a magnet that was already resting there from firing a gesture.
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
//   RX effect are identical to the toggle-combo disarm. FM and RTM stay mutually exclusive on the
//   buggy: RTM active/arming blocks any FM toggle here, and on the RX an active RTM makes Follow-Me
//   yield. Since 2026-09-18 arming RTM no longer disarms FM first (setRtmArmed()) - Follow-Me
//   stays armed through the return and resumes ARMED (unlatched) when it ends.
//
// ARMING WHILE ON THE THROTTLE IS INTENTIONAL
//   (V2.5-Evo - 2026-10-07 - SOP-040: EXCEPT the mag_mode 4 2.5 s Return-To-Me hold, which now needs the trigger
//   fully released - see kMagHoldTriggerHeld. Roles 1-3 and the mode 4 tap are unchanged.)
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
extern volatile bool    vib_stop_pending;   // V2.5-Evo - 2026-10-07 - the stop-buzz request flag (System.ino)
// rtmIsArming() is defined in RTMState.ino (also concatenated after this file).
bool rtmIsArming();
// V2.5-Evo - 2026-09-30 - rtmEnabledEffective() is the ONE place that answers "is Return-To-Me enabled
// right now". It is the stored usrConf.rtm_enabled unless a session override stands (RAM only — see the
// RTM SESSION OVERRIDE block in RTMState.ino; since 2026-10-07 nothing writes that override, R-4). Every gate that used to read
// usrConf.rtm_enabled directly now calls this, so a session flip is honoured everywhere and can still
// never reach SPIFFS. Defined in RTMState.ino, concatenated after this file.
bool rtmEnabledEffective();
// (V2.5-Evo - 2026-10-07 - R-4: the fmToggleAutoReturnFromMagnet() prototype that sat here is gone with the
// function itself; the 2.5 s hold has always started the manual Return-To-Me since 2026-10-06.)
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
// kMagPollMs is a FLOOR, NOT THE ACTUAL SAMPLE RATE. V2.5-Evo - 2026-09-30 (Rex delta audit M-2): this
// function is reached once per loop() iteration and loop() ends in vTaskDelay(110), so the real interval
// between two P_MAG samples is ~110 ms. This constant only stops the gesture from re-sampling FASTER than
// 20 ms if loop() ever gets shorter; it has never made the sampling 20 ms, and no timing rationale in this
// file may be reasoned from "20 ms samples". Use kMagLoopPeriodMs below for anything that needs the real
// cadence. (The SW33b dot block in loop() does poll on its own 20 ms clock — that is a different sampler.)
static const uint32_t kMagPollMs     = 20UL;     // MINIMUM interval between samples, not the real one
// The real loop() period: V2_Integration_Tx.ino ends loop() with vTaskDelay(pdMS_TO_TICKS(110)). Everything
// in this file that has to know how often the pin is actually read uses this, not kMagPollMs.
static const uint32_t kMagLoopPeriodMs = 110UL;
// ---- V2.5-Evo - 2026-09-30 - SAMPLE-GAP GUARD (Rex delta audit H-1) ----
// THE BUG THIS FIXES. Hold length was computed purely from wall-clock (now - mag_hold_start) with no check
// that the pin had actually been READ across that interval. Any block of loop() longer than the hold
// threshold therefore promoted a short touch into a long hold: a 300 ms tap taken while an FM fault-stop
// disarm (2 s), an rtmDisengage() (2 s) or the RTM arm ceremony (up to rtm_arm_window_s = 15 s) was
// blocking loop() would come back measuring >2.5 s and silently toggle the Return-To-Me enable — a
// safety-relevant setting the rider would only discover when he next tried to recall the buggy.
// THE FIX. Remember when the last sample was actually accepted. If the gap since then is larger than a
// magnet hold could possibly hide behind, the pin went UNOBSERVED, so this hold is not evidence of
// anything: abandon it exactly as the parked-magnet guard does, and removal then does nothing at all.
// WHY A TIMESTAMP AND NOT A SAMPLE COUNT: a count still has to be compared against an expected count for
// the elapsed wall-clock, which needs a trustworthy cadence — the very thing that is missing here. One gap
// measurement is direct evidence that the pin was not read, with nothing inferred.
// WHY 5 LOOP PERIODS (550 ms): the real cadence is ~110 ms, so 5 periods clears ordinary jitter (display
// writes, GPS drain, a serial command) without false-tripping, while still catching EVERY blocker Rex
// listed — the shortest of them is 2000 ms, and even a gear-flash hold (gear_display_time, 800 ms default)
// is caught. THE COST OF A FALSE TRIP IS ZERO RISK: the gesture is abandoned silently and the rider taps
// again. APPLIES TO EVERY ROLE, not just mode 4: roles 1-3 measure their 2 s / 5 s holds off the same
// wall-clock and had the same weakness.
static const uint32_t kMagSampleGapMaxMs = 5UL * kMagLoopPeriodMs;   // 550 ms
// Parked-magnet guard: if the magnet stays present longer than this, the rider is not making
// a gesture — the remote is stowed against something magnetic. The gesture is abandoned and
// removal does nothing. Without this, un-stowing the remote hours later would arm RTM.
static const uint32_t kMagMaxHoldMs  = 30000UL;
// ---- V2.5-Evo - 2026-09-30 - MagStations: mag_mode 4 timing (compile-time only, no confStruct change) ----
// A TAP is 60-600 ms of magnet-present. 60 ms is the floor because anything shorter is indistinguishable
// from contact bounce. 600 ms is the ceiling (raised from 400 ms on 2026-09-30, Rex delta audit M-2): the
// 400 ms figure came from shipped double-tap windows (Android 300 ms, the OneButton library 400 ms), but
// those assume a fast sampler, and this gesture is sampled once per ~110 ms loop() iteration. With ±110 ms
// of quantisation a real 300 ms tap can measure ~410 ms, so a 400 ms ceiling dropped good taps silently.
// 600 ms swallows the jitter and still keeps the tap band and the 2500 ms hold band more than 4x apart.
static const uint32_t kMagTapMinMs   = 60UL;      // shorter than this = bounce, ignored
static const uint32_t kMagTapMaxMs   = 600UL;     // longer than this is not a tap (see the dead-zone note)
// The hold that starts the manual Return-To-Me (V2.5-Evo - 2026-10-07 - R-4: this comment said "toggles
// Return-To-Me", which it has not done since 2026-10-06). 2500 ms is more than 4x the tap CEILING (and ~8x a typical 300 ms
// tap), which is what makes the pair impossible to confuse. The advisory buzz fires the moment the hold
// crosses it, so the rider never has to estimate time: hold until you feel it, then take the magnet away.
static const uint32_t kMagRtmToggleHoldMs = 2500UL;
// Debounce for mag_mode 4 ONLY. The 120 ms used by roles 1-3 is longer than the whole 60 ms tap floor, so
// with it a tap could never be seen at all. 40 ms is the shortest window that still requires the level to
// survive into a following sample before it is believed — and since the real sample interval is ~110 ms
// (see kMagLoopPeriodMs), in practice that means one further loop iteration, so it is a real debounce and
// not a removal of one. (V2.5-Evo - 2026-09-30: the old comment here claimed "two full 20 ms samples",
// which described a sample rate this firmware has never run. Corrected, value unchanged.)
// Roles 1-3 keep their original 120 ms untouched — this constant is never applied to them.
static const uint32_t kMagTapDebounceMs = 40UL;
// ---- V2.5-Evo - 2026-10-06 - TAP LOCKOUT AFTER A STATION STEP (audit M-1, owner ruling: 1 second) ----
// THE PROBLEM. Until 2026-10-06 a station step blocked loop() for 1.2 s on its display flash, and that
// stall swallowed any further taps by accident. The F-label hold no longer blocks, so two quick taps -
// or one wobbly magnet that bounces off and back - stepped TWO stations (F1 -> F3) while the rider may
// have felt only one buzz count.
// THE RULE. After a tap that ACTUALLY moved the station, a tap whose magnet ARRIVES within this window
// is ignored completely: no step, no arm, no buzz, and the debounce state is rebuilt from the pin
// exactly as after any other mode 4 removal, so nothing is left half-armed. Judged on the ARRIVAL edge,
// not the removal, so a bounce that lands inside the window is dropped even if it lifts after it.
// WHY 1000 ms. A deliberate tap-remove-tap-remove still steps quickly (about one station a second),
// while a wobble or a bounce lands well inside it. (It also used to guarantee that the previous step's
// Pattern 11 count had finished before the next one queued; V2.5-Evo - 2026-10-07 - a station step no
// longer buzzes at all, so only the double-step protection remains.)
// Only taps are locked out. The 2.5 s hold is untouched: it cannot complete inside the window anyway.
static const uint32_t kMagStepLockoutMs = 1000UL;

// ---- V2.5-Evo - 2026-10-07 - WHAT A mag_mode 4 HOLD WILL DO (audits R-2 and R-7, owner rulings) ----
// The 2.5 s hold has three possible outcomes, decided ONCE, at the moment the hold crosses 2.5 s, so the
// buzz the rider feels while holding and the action on removal always agree:
//   kMagHoldStartRtm - start the manual Return-To-Me ceremony (Pattern 10 while holding, "rn" on removal).
//   kMagHoldIgnored  - a Return-To-Me is ALREADY ACTIVE. R-2: the hold is ignored completely - no buzz, no
//                      action - and the return carries on. (It used to re-enter the arm ceremony, which cut
//                      the buggy to 0 and restarted the return with no "St" and no cooldown.) Releasing the
//                      trigger still ends the return through Gate 3, exactly as before.
//   kMagHoldRefused  - Return-To-Me cannot start (RTM disabled or GPS off). R-7: the success cue used to
//                      play and then nothing happened. Now the rider gets a refusal instead: ONE short
//                      150 ms blip (Pattern 5) while holding, and "n0" ("no") on the display for 2 s on
//                      removal. Not Pattern 10 (that is the success cue) and not Pattern 7 (the long fault
//                      buzz - nothing faulted, the feature is simply off).
//                      V2.5-Evo - 2026-10-07 - SUPERSEDED by SOP-040 ("St" is the only "not working" signal):
//                      no buzz while holding; on removal "St" for 2 s with the normal stop buzz (Pattern 7).
//   kMagHoldTriggerHeld - V2.5-Evo - 2026-10-07 - SOP-040 GESTURE RULE: the trigger is NOT fully released
//                      (triggerReleased() false). The hold is ignored SILENTLY - no buzz while holding, nothing on
//                      removal, one serial line. Checked at the 2.5 s mark (so no Pattern 10 promise is made) AND
//                      again at removal (a squeeze after the buzz still cancels it). This makes the held-trigger
//                      RTM ceremony unreachable from the magnet: with the trigger held, nudging or steering a buggy
//                      never starts or ends a return. The magnet TAP is not affected: with the trigger held a tap
//                      still arms Follow-Me (owner exception) and still steps the station while following.
// Latched in runMagGesture() so a state change between the 2.5 s mark and the removal (for example Gate 3
// ending an active return while the magnet is still held) cannot turn an ignored hold into a new ceremony.
static const uint8_t kMagHoldStartRtm    = 0;
static const uint8_t kMagHoldIgnored     = 1;
static const uint8_t kMagHoldRefused     = 2;
static const uint8_t kMagHoldTriggerHeld = 3;   // V2.5-Evo - 2026-10-07 - SOP-040 gesture rule

// magHoldVerdict - decide what a mag_mode 4 2.5 s hold will do right now (see the block above).
// Inputs: rtm_tx_active, rtmIsArming(), triggerReleased(), rtmEnabledEffective(), usrConf.gps_en.
// Output: kMagHoldStartRtm, kMagHoldIgnored, kMagHoldTriggerHeld or kMagHoldRefused. No side effects, never blocks.
// Order matters: an active return is ignored first (R-2), then a held trigger is ignored silently (SOP-040) - so a
// held trigger never produces the "St" refusal either - and only then is a released-trigger refusal shown (R-7).
static uint8_t magHoldVerdict()
{
  if (rtm_tx_active || rtmIsArming())                return kMagHoldIgnored;      // R-2: the return continues
  if (!triggerReleased())                            return kMagHoldTriggerHeld;  // SOP-040: trigger held -> silent
  if (!(rtmEnabledEffective() && usrConf.gps_en))    return kMagHoldRefused;      // R-7: say no, clearly
  return kMagHoldStartRtm;
}

// gpsKeepAliveDelay() is defined in RTMState.ino (concatenated after this file). Declared static here to
// match its definition; used for the 2 s "n0" refusal hold below.
static void gpsKeepAliveDelay(uint32_t ms);

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
  static bool     hold_abandoned   = false;  // parked-magnet OR sample-gap guard tripped this hold
  // V2.5-Evo - 2026-09-30 - H-1: millis() of the last sample this function actually took. 0 = none yet.
  // The gap between consecutive samples is the evidence that the pin was really watched across a hold.
  static uint32_t mag_last_sample_ms = 0;
  // V2.5-Evo - 2026-10-06 - M-1 tap lockout: millis() of the last tap that ACTUALLY stepped the station,
  // and whether there has been one yet this power-up (a flag, not a 0 sentinel, so a step at any millis()
  // value counts). Loop task only, like every other static here.
  static uint32_t mag_last_step_ms   = 0;
  static bool     mag_step_seen      = false;
  // V2.5-Evo - 2026-10-07 - R-2 / R-7: what this mag_mode 4 hold will do, latched when it crosses 2.5 s
  // (kMagHold* above). Meaningful only while rtm_advised is true for the same hold.
  static uint8_t  mag_hold_verdict   = 0;

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

  // ---- V2.5-Evo - 2026-09-30 - SAMPLE-GAP GUARD (Rex delta audit H-1) ----
  // This sample is about to be taken, so first judge how long it has been since the last one. If loop()
  // was blocked longer than kMagSampleGapMaxMs then the pin was NOT watched over that stretch, and any
  // hold in progress cannot be trusted to be a hold at all — a brief touch could have started and ended
  // inside the blind window, or a touch that is still present could look far older than it is. Abandon it
  // the same way the parked-magnet guard does: no advisory, and removal does nothing.
  // The guard only judges a hold that is already in progress (mag_stable_low). A gap with no magnet
  // present is harmless — nothing was being timed — and the very first call has no previous sample to
  // compare against, which is why mag_last_sample_ms == 0 is excluded.
  // See the kMagSampleGapMaxMs comment for why the threshold is 5 loop periods and why this applies to
  // every mag_mode role, not only mode 4.
  uint32_t sample_gap = now - mag_last_sample_ms;
  bool     first_ever = (mag_last_sample_ms == 0);
  mag_last_sample_ms  = now;
  if (!first_ever && mag_stable_low && !hold_abandoned && sample_gap > kMagSampleGapMaxMs)
  {
    hold_abandoned = true;
    Serial.print("MAG [TX] hold abandoned: loop stalled ");   // V2.5-Evo - 2026-09-30 - H-1
    Serial.print(sample_gap);
    Serial.println(" ms, the magnet pin went unsampled - this hold is not trusted");
    return;
  }

  // Boot guard (SW33): until GPIO 9 has been seen HIGH at least once since power-up we cannot
  // tell "rider is holding a magnet" from "a magnet was already sitting there when it booted".
  // mag_seen_high is set by the bt_dot_state block in loop(); we only read it.
  if (!mag_seen_high)
  {
    mag_raw_last   = false;
    mag_stable_low = false;
    return;
  }

  // ---- Debounce: a level must persist for the role's debounce window before it is accepted ----
  // V2.5-Evo - 2026-09-30 - MagStations: roles 1-3 keep the original 120 ms exactly. MAG_ROLE_FMSET
  // needs 40 ms instead, because 120 ms is longer than the entire 60 ms tap floor — with it, a tap
  // could never be accepted at all and the gesture would simply not work.
  // V2.5-Evo - 2026-09-30 - comment corrected (Rex delta audit M-2): this used to claim 40 ms is "two full
  // 20 ms samples". It is not — the real sample interval is ~110 ms (see kMagLoopPeriodMs), so what 40 ms
  // actually requires is that the new level still be there on a LATER sample, i.e. one further loop
  // iteration. That is still a real debounce against pin flutter; only the arithmetic behind it was wrong.
  uint32_t debounce_ms = (role == MAG_ROLE_FMSET) ? kMagTapDebounceMs : kMagDebounceMs;

  bool raw_low = (digitalRead(P_MAG) == LOW);   // LOW = magnet present
  if (raw_low != mag_raw_last)
  {
    mag_raw_last  = raw_low;
    mag_raw_since = now;   // start timing this new run
  }

  bool edge_accepted = false;
  if (raw_low != mag_stable_low && (now - mag_raw_since) >= debounce_ms)
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
    // V2.5-Evo - 2026-09-30 - H-1, THE OTHER HALF OF THE SAME BUG. The guard at the top of this function
    // catches a stall that happens DURING a debounced hold, but a stall can also land across the ARRIVAL
    // itself: the pin is seen LOW on one sample, loop() blocks for seconds, the pin is seen LOW again, the
    // debounce accepts the edge - and because the hold clock is back-dated to mag_raw_since (the first of
    // those two samples) the "hold" is already seconds old before the rider has held anything. Worse, the
    // magnet could have been taken away and put back inside that blind window.
    // So: back-date the hold ONLY as far as a genuinely observed edge. If the edge being back-dated to is
    // older than the sample-gap limit, it was not observed - abandon the hold instead of trusting it.
    // In normal running the edge is one or two samples old (~110-230 ms) and this never trips.
    hold_abandoned = ((now - mag_raw_since) > kMagSampleGapMaxMs);
    return;
  }

  // ---- Magnet present: fire the advisory buzzes as each threshold is crossed ----
  // NO ADVISORY EVER FIRES ON AN ABANDONED HOLD. The !hold_abandoned test on this block is what
  // enforces it, and it matters more now than it did: since 2026-09-30 a hold can be abandoned by the
  // sample-gap guard at ANY time, including before the 2.5 s mark, not only at the 30 s parked-magnet
  // mark. An advisory is a promise ("let go now and X will happen"), and removal after an abandon does
  // nothing at all, so promising on a hold that can no longer deliver would be a lie the rider feels.
  // ONE RESIDUAL CASE, STATED HONESTLY: the 2.5 s advisory fires at 2.5 s and the parked-magnet guard
  // trips at 30 s, so a rider who keeps holding for another 27.5 s after the buzz has already been
  // promised something that will not happen. The promise was true when it was made; it expires only
  // because he kept holding far past the band. Nothing is re-buzzed on that expiry, deliberately —
  // a buzz meaning "never mind" is exactly the feedback-on-nothing this gesture avoids everywhere else.
  if (mag_stable_low && !hold_abandoned)
  {
    uint32_t held = now - mag_hold_start;

    if (held >= kMagMaxHoldMs)
    {
      // Parked magnet — abandon this hold entirely; removal will do nothing, and no further
      // advisory can fire because this whole block is gated on !hold_abandoned.
      hold_abandoned = true;
      return;
    }
    // Advisories are ONLY hints about what removal would do. They arm nothing.
    // Guarded on current_vib_pattern == 0 so an advisory never stomps a warning
    // pattern (signal drop, low battery, E71) that is already playing.
    // The 5s tier exists only in MAG_ROLE_BOTH; the single-role modes stop at 2s.
    // V2.5-Evo - 2026-09-30 - MagStations: MAG_ROLE_FMSET has its own single band at 2.5 s and does
    // NOT use the 2 s / 5 s thresholds at all, so it is handled first and returns. Pattern 10 (two
    // medium pulses) says "let go now and the manual Return-To-Me starts" (V2.5-Evo - 2026-10-07 - R-4:
    // it used to say "will toggle"; see the hold verdict below for when it is NOT played). Same buzz-announces-the-band
    // principle as the other roles: the rider holds until the pattern arrives, then takes the magnet
    // away — they never have to estimate 2.5 seconds.
    if (role == MAG_ROLE_FMSET)
    {
      if (!rtm_advised && held >= kMagRtmToggleHoldMs)
      {
        rtm_advised = true;
        // V2.5-Evo - 2026-10-07 - R-2 / R-7: decide now what removal will do, and only promise what it
        // will deliver. Pattern 10 only when the hold will really start the ceremony; Pattern 5 (one short
        // blip) when Return-To-Me cannot start; nothing at all while a return is already active.
        mag_hold_verdict = magHoldVerdict();
        if (mag_hold_verdict == kMagHoldStartRtm)
        {
          if (current_vib_pattern == 0) current_vib_pattern = 10;
        }
        // V2.5-Evo - 2026-10-07 - SOP-040 ("St" is the only refusal signal): kMagHoldRefused no longer plays the
        // Pattern 5 blip here; the refusal is "St" + the stop buzz on removal (see below).
        // kMagHoldRefused, kMagHoldIgnored and kMagHoldTriggerHeld: no buzz while holding.
      }
      return;
    }
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
    // V2.5-Evo - 2026-10-07 - R-2 / R-7: the mode 4 hold verdict latched at the 2.5 s mark. If no
    // advisory pass ran for this hold (it always does in practice - the sample that first sees the
    // magnet gone evaluates the same held value), decide it now instead.
    uint8_t  hold_verdict  = rtm_advised ? mag_hold_verdict : magHoldVerdict();

    // Clear per-hold state before doing anything blocking.
    fm_advised     = false;
    rtm_advised    = false;
    hold_abandoned = false;

    // Abandoned by EITHER guard — the parked-magnet 30 s limit or the sample-gap guard at the top of
    // this function (V2.5-Evo - 2026-09-30 - H-1). In both cases the hold length is not evidence of
    // anything the rider did, so removal does nothing, silently.
    if (was_abandoned) return;

    // ---- Common preconditions: states in which no gesture should be honoured at all ----
    // V2.5-Evo - 2026-09-30 - MagStations: moved ABOVE the roles 1-3 length check so mag_mode 4 gets
    // the same three refusals. Their meaning is unchanged for every role.
    if (system_locked) return;                      // remote locked — no gesture from a stowed remote
    if (in_setup) return;                           // mid-calibration / setup
    if (remote_error && !remote_error_blocked) return;  // unacknowledged error on screen
    // V2.5-Evo - 2026-10-07 - C-1: no magnet gesture while the throttle input is faulted. remote_error 72 above
    // already covers it; this line holds even if water-ingress E71 has replaced the 72 on screen.
    if (ads_input_fault) return;

    // ============================================================
    // V2.5-Evo - 2026-09-30 - MagStations: MAG_ROLE_FMSET (mag_mode 4) removal handling.
    // Entirely separate from the roles 1-3 branch below, which continues untouched.
    //
    // TAP (60-600 ms): if Follow-Me is not armed, arm it at the stored default station; if it is
    // ACTIVELY FOLLOWING, step to the next station in mag_fm_set. Anything else — armed but not yet
    // following, i.e. the rider on the rope under tow — does NOTHING AND SAYS NOTHING. That silence is
    // the point: a buzz meaning "I ignored you" teaches the rider to expect feedback from accidental
    // magnet contact, and the buggy must never reposition itself while he is attached to the rope.
    //
    // HOLD (>= 2.5 s): start the MANUAL Return-To-Me ceremony through setRtmArmed(), in every Follow-Me
    // state (V2.5-Evo - 2026-10-06, owner ruling; see the hold branch below).
    //
    // 600 ms - 2.5 s falls through both and does nothing: the deliberate dead zone that keeps the tap
    // band and the hold band more than 4x apart. See the band table in this function's header comment.
    // ============================================================
    if (role == MAG_ROLE_FMSET)
    {
      if (held >= kMagRtmToggleHoldMs)
      {
        // V2.5-Evo - 2026-10-06 - THE HOLD ALWAYS MEANS MANUAL RETURN-TO-ME (owner ruling, option A:
        // "I want the buggy to always return to me; if it stops working I use the manual one").
        // From 2026-10-02 this branch was state-aware: FM armed -> toggle auto-return ("A1"/"A0"),
        // FM not armed -> manual recall. That left the manual recall UNREACHABLE from the magnet in
        // exactly the state where the rider needs a fallback for auto-return. It now starts the manual
        // ceremony in every state - FM disarmed, FM armed, FM engaged.
        //
        // WHAT HAPPENS TO FOLLOW-ME: nothing on the remote. setRtmArmed() has not disarmed Follow-Me
        // since 2026-09-18 (RTMState.ino, header of setRtmArmed): fm_armed, the station and the 30 s
        // keepalive stay as they were. While the ceremony waits for the squeeze, the throttle byte is
        // forced to 0 (Radio.ino, rtmIsArming()). Once RTM goes ACTIVE (0xF1/1) the buggy makes
        // Follow-Me YIELD - parked ARMED, separation latch cleared - and it re-engages only by
        // re-proving separation after the return ends. RTM > FM, the existing precedence, unchanged.
        //
        // setRtmArmed() keeps the FULL ceremony: it re-checks rtmEnabledEffective() and gps_en itself,
        // then blinks "rn" and requires a >30% trigger squeeze held 500 ms before anything is armed.
        // The magnet is another DOORWAY to that ceremony, never a way past it - and it is the only
        // doorway reachable mid-tow, because calcFilter() hands the toggle to steering the moment the
        // trigger rises.
        //
        // WHERE A1/A0 (THE AUTO-RETURN TOGGLE) LIVES NOW: inside the "rn" wait. With the trigger
        // released, a RIGHT tap then a LEFT hold of rtm_hold_duration_s, all inside rtm_arm_window_s,
        // cancels the arm and flips auto-return for the session (returnGestureCeremonyPoll() and
        // ceremonyCancelForReturnGesture() in RTMState.ino). That detector does not need the arming
        // LEFT hold the toggle route starts with: on the magnet route the LEFT toggle is already up,
        // so it is live from the first poll. (fmToggleAutoReturnFromMagnet() was removed 2026-10-07, R-4.)
        //
        // V2.5-Evo - 2026-10-07 - R-2 / R-7: the verdict latched at 2.5 s decides (see magHoldVerdict()).
        // V2.5-Evo - 2026-10-07 - SOP-040 gesture rule: the trigger is checked AGAIN here, at the moment the action
        // would happen. Released at 2.5 s but squeezed by the time the magnet comes off -> ignored silently.
        if ((hold_verdict == kMagHoldStartRtm || hold_verdict == kMagHoldRefused) && !triggerReleased())
        {
          hold_verdict = kMagHoldTriggerHeld;
        }
        if (hold_verdict == kMagHoldTriggerHeld)
        {
          // SOP-040: the 2.5 s hold acts only with the trigger fully released. Nothing happens, no buzz.
          Serial.printf("MAG [TX] hold ignored: trigger held (thr %u) - the 2.5 s hold needs the trigger fully released\n",
                        (unsigned)thr_scaled);
        }
        else if (hold_verdict == kMagHoldIgnored)
        {
          // R-2: a Return-To-Me is already running. Ignore the hold; the return continues untouched.
          Serial.println("MAG [TX] hold ignored: Return-To-Me is already active, the return continues");
        }
        else if (hold_verdict == kMagHoldRefused)
        {
          // R-7: Return-To-Me cannot start.
          // V2.5-Evo - 2026-10-07 - SOP-040: "St" with the normal stop buzz is the ONLY refusal signal. This used
          // to be "n0" plus a Pattern 5 blip at 2.5 s; a rider knows "St" and never misses it, and reads the
          // context from what he was trying to do. BLOCKING 2 s, like every other "St"; the resync below covers it.
          Serial.printf("MAG [TX] hold refused: Return-To-Me cannot start (rtm enabled %d, gps_en %d)\n",
                        rtmEnabledEffective() ? 1 : 0, (int)usrConf.gps_en);
          vib_stop_pending = true;   // Pattern 7, the normal stop buzz
          DISP_LOCK(); displayDigits(LET_S, LET_T); updateDisplay(); DISP_UNLOCK();
          gpsKeepAliveDelay(2000);
        }
        else
        {
          setRtmArmed();
        }
      }
      else if (held >= kMagTapMinMs && held <= kMagTapMaxMs)
      {
        // V2.5-Evo - 2026-10-06 - M-1 TAP LOCKOUT, checked FIRST so a locked-out tap does nothing at all
        // (no step, no arm, no buzz). Judged on the arrival edge (mag_hold_start). Wrap-safe: unsigned
        // subtraction, and an arrival can never precede the step, because the step happens at the removal
        // of the previous contact and the debounce state is rebuilt just below it.
        if (mag_step_seen && (uint32_t)(mag_hold_start - mag_last_step_ms) < kMagStepLockoutMs)
        {
          Serial.print("MAG [TX] tap ignored: magnet arrived ");
          Serial.print((uint32_t)(mag_hold_start - mag_last_step_ms));
          Serial.print(" ms after the last station step (lockout ");
          Serial.print(kMagStepLockoutMs);
          Serial.println(" ms)");
        }
        else if (rtm_tx_active || rtmIsArming())
        {
          // Return-To-Me owns the buggy right now — never touch Follow-Me underneath it.
        }
        else if (!(usrConf.fm_override_enabled && usrConf.gps_en))
        {
          // Follow-Me is not usable on this remote at all — same guard the roles 1-3 FM path applies.
        }
        else if (!isFmArmed())
        {
          // Not armed — the first tap ARMS at the stored default station. cycleFmMode() does the
          // fundamental-readiness check, the Pattern 4 confirm and the F<n> display, exactly as it does
          // for the toggle combo and for mag_mode 1.
          cycleFmMode();
        }
        else if (fmIsEngaged())
        {
          // Actively following — the rope is slack, so a station transit is safe. This is the ONLY
          // state in which the magnet is allowed to move the buggy's station.
          // V2.5-Evo - 2026-10-06 - M-1: start the lockout only when the station really moved. A silent
          // no-op tap (already at the only set station) starts nothing.
          if (fmStepStationFromMagnet())
          {
            mag_last_step_ms = millis();
            mag_step_seen    = true;
          }
        }
        // else: armed but not following = on the rope. Nothing, and no feedback. See the block above.
      }
      // Re-synchronise the debounce state: the actions above can block for up to 2 s, so the magnet
      // may have been re-applied since. A new gesture needs a fresh, fully debounced arrival edge.
      // V2.5-Evo - 2026-09-30 - H-1: mag_last_sample_ms is re-based here too. The action we just ran IS
      // a long unsampled gap, and it is an ACCOUNTED-FOR one — the state above is rebuilt from the pin as
      // it reads right now — so the sample-gap guard must not also punish the next sample for it.
      mag_raw_last       = (digitalRead(P_MAG) == LOW);
      mag_stable_low     = mag_raw_last;
      mag_raw_since      = millis();
      mag_hold_start     = millis();
      hold_abandoned     = mag_stable_low;  // magnet still there on return -> parked, not a new gesture
      mag_next_poll_ms   = millis() + kMagPollMs;
      mag_last_sample_ms = millis();
      return;
    }

    if (held < kMagFmHoldMs) return;    // accident guard — too short to mean anything (roles 1-3)

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
      // V2.5-Evo - 2026-09-30 - reads the EFFECTIVE enable (stored value, or the RAM session override a
      // mag_mode 4 hold may have set) instead of usrConf.rtm_enabled directly.
      if (!(rtmEnabledEffective() && usrConf.gps_en)) return;
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
        // RTM disarmed → ARM. setRtmArmed() is only the gesture half of RTM arming: it sets
        // RTM_ARMED, zeroes rtm_thr_cap_tx, then runs the blocking runDoubleSqueezeArm()
        // throttle-squeeze ceremony — exactly as the toggle path does. (It no longer disarms FM:
        // since 2026-09-18 Follow-Me stays armed through a return and the RX yields instead.)
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
        Serial.println("FM [TX] disarm: magnet gesture -> 0xF2/0");   // V2.5-Evo - 2026-09-19
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
    // V2.5-Evo - 2026-09-30 - H-1: mag_last_sample_ms is re-based for the same reason as in the mode 4
    // branch above — this block IS the accounted-for unsampled gap, so the guard must not re-judge it.
    mag_raw_last       = (digitalRead(P_MAG) == LOW);
    mag_stable_low     = mag_raw_last;
    mag_raw_since      = millis();
    mag_hold_start     = millis();
    hold_abandoned     = mag_stable_low;  // magnet still there on return → treat as parked, not a new gesture
    mag_next_poll_ms   = millis() + kMagPollMs;
    mag_last_sample_ms = millis();
  }
}

// ---- V2.5-Evo - 2026-10-07 - R-3: IGNORE THE TOGGLE UNTIL IT IS RELEASED AFTER AN RTM CEREMONY ----
// THE BUG: the blocking RTM arm ceremony (runDoubleSqueezeArm(), RTMState.ino) can end - window expiry, an
// A1/A0 cancel, a refusal, or RTM going ACTIVE - while the rider is still holding the LEFT toggle for the
// in-ceremony RIGHT tap -> LEFT hold. That held toggle then reached handleGearToggle() as a brand-new press:
// a gear-down step at once and, after 2 s, a Follow-Me station change (FM armed) or a remote lock.
// THE FIX: the ceremony sets this latch when it starts; while it is set, runMenu() ignores the toggle, and
// the first pass that sees the toggle centred clears it. A toggle that is held when the ceremony ends can
// therefore never become a gear, station or lock action. Non-blocking: loop() keeps running, so RTM's gates
// run normally if the ceremony ended in RTM ACTIVE. Written by the loop task only (runDoubleSqueezeArm()
// and runMenu() both run on it). The toggle route is unaffected: handleGearToggle() already waits for its
// own release after the ceremony returns, so the latch is clear on the very next pass.
static bool ceremony_toggle_latch = false;

void runMenu()
{
  // V2.5-Evo - 2026-10-07 - R-3: see ceremony_toggle_latch above.
  if (ceremony_toggle_latch)
  {
    if (ctminus() || ctplus()) return;   // still held since the ceremony - not a new press
    ceremony_toggle_latch = false;       // released: from here the toggle is a fresh input again
  }

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
    // V2.5-Evo - 2026-10-07 - P-11: the throttle plausibility check (Analog.ino) is exempt ONLY while this block
    // calibrates; the flag is cleared as soon as calibration succeeds (failure halts below, locked).
    ads_cal_in_progress = true;
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
      ads_cal_in_progress = false;   // V2.5-Evo - 2026-10-07 - P-11: the new band is trusted from here on
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
