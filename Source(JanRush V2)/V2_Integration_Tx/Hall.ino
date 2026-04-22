// Returns true if the given display mode has a valid value
bool isDisplayModeAvailable(uint8_t mode)
{
  switch(mode) {
    case DISPLAY_MODE_TEMP:   return telemetry.foil_temp  != 0xFF;
    case DISPLAY_MODE_SPEED:  return telemetry.foil_speed != 0xFF;
    case DISPLAY_MODE_POWER:  return telemetry.foil_power != 0xFF;
    case DISPLAY_MODE_BAT:    return telemetry.foil_bat   != 0xFF;
    case DISPLAY_MODE_THR:    return true;
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
  switch(display_mode) {
    case DISPLAY_MODE_TEMP:   displayDigits(LET_T, LET_P); break;
    case DISPLAY_MODE_SPEED:  displayDigits(5, LET_P); break;
    case DISPLAY_MODE_POWER:  displayDigits(LET_P, LET_V); break;
    case DISPLAY_MODE_BAT:    displayDigits(LET_B, LET_A); break;
    case DISPLAY_MODE_THR:    displayDigits(LET_T, LET_H); break;
    case DISPLAY_MODE_INTBAT: displayDigits(LET_U, LET_B); break;
  }
  updateDisplay();
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

// direction: -1 = gear down (long press locks), +1 = gear up (long press follow-me)
void handleGearToggle(int direction)
{
  bool (*isActive)() = (direction < 0) ? ctminus : ctplus;

  in_menu = usrConf.menu_timeout+1;
  delay(50);
  unsigned long pushtime = millis();
  bool change_once = 1;

  while(isActive())
  {
    delay(10);
    if(millis() - pushtime > usrConf.lock_waittime)
    {
      if(thr_scaled < 10)
      {
        if(direction < 0)
        {
          //Long press minus: lock system
          if(!usrConf.no_lock)
          {
            system_locked = 1;
            displayLock();
          }
        }
        else
        {
          //Long press plus: cycle display mode
          cycleDisplayMode(1);
        }
        in_menu = usrConf.menu_timeout;
      }
      while(isActive())
      {
        delay(100);
      }
      break;
    }
    if(millis() - pushtime > usrConf.gear_change_waittime)
    {
      if(change_once)
      {
        switch(usrConf.throttle_mode)
        {
          case 0: // Gears
          default:
            if(direction < 0 && gear > 0) gear--;
            else if(direction > 0 && gear < usrConf.max_gears-1) gear++;
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
  if(!(millis() - pushtime > usrConf.lock_waittime))
  {
    while(isActive())
    {
      delay(10);
    }
    delay(50);
    while(millis() - pushtime < usrConf.gear_display_time)
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
            delay(500);
            while(thr_scaled > 5)
            {
              delay(100);
            }
            delay(1000);
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
