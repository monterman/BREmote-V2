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

  // Combo holds need 5s; simple holds need 2s
  unsigned long long_press_ms = has_combo ? 5000UL : 2000UL;

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
            unlockAnimation();
            delay(250);
            while(thr_scaled > 5)
            {
              delay(100);
            }
            delay(500);
            system_locked = 0;
#ifdef WIFI_ENABLED
            webCfgNotifyTxUnlocked();
#endif
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
