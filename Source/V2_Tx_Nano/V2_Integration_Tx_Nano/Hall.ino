//100us
void calcFilter()
{
  volatile uint16_t thr_filter = 0;
  volatile uint16_t tog_filter = 0;
  volatile uint16_t intbat_filter = 0;

  for(int i = 0; i < BUFFSZ; i++)
  {
    thr_filter += thr_raw[i]/BUFFSZ;
    tog_filter += tog_raw[i]/BUFFSZ;
    intbat_filter += intbat_raw[i]/BUFFSZ;
  }

  //Map Throttle
  if(usrConf.thr_idle < usrConf.thr_pull)
  {
    uint16_t thr_const = constrain(thr_filter, usrConf.thr_idle, usrConf.thr_pull);
    thr_scaled = (uint8_t)((long)(thr_const - usrConf.thr_idle) * 255 / (usrConf.thr_pull - usrConf.thr_idle));
  }
  else
  {
    uint16_t thr_const = constrain(thr_filter, usrConf.thr_pull, usrConf.thr_idle);
    thr_scaled = 255 - (uint8_t)((long)(thr_const - usrConf.thr_pull) * 255 / (usrConf.thr_idle - usrConf.thr_pull));
  }

  //Deadzone mid-toggle
  uint16_t deadbandLower = usrConf.tog_mid - (usrConf.tog_deadzone / 2);
  uint16_t deadbandUpper = usrConf.tog_mid + (usrConf.tog_deadzone / 2);
  
  if(tog_filter >= deadbandLower && tog_filter <= deadbandUpper)
  {
    tog_scaled = 127;
  }
  else
  {
    //Map toggle
    if(usrConf.tog_left < usrConf.tog_right)
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
  if((thr_scaled > 3 && system_locked == 0 && !in_menu && usrConf.steer_enabled)||(usrConf.no_gear && usrConf.no_lock && !remote_error))
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

void runMenu()
{
  if(remote_error == 0 || remote_error_blocked == 1)
  {
    if(system_locked)
    {/*
      if(ctminus())
      {
        in_menu = usrConf.menu_timeout+1;
        delay(300);
        if(ctminus())
        {
          //To unlock, promt user to touch throttle once
          advanceArrow();
          unsigned long timeout = millis();
          while(thr_scaled < 20 && (millis()-timeout < usrConf.trig_unlock_timeout ))
          {
            advanceArrow();
            delay(100);
          }
          if(millis()-timeout < usrConf.trig_unlock_timeout)
          {
            unlockAnimation();
            delay(500);
            while(thr_scaled > 5)
            {
              delay(100);
            }
            delay(1000);
            system_locked = 0;
            if(usrConf.startgear > usrConf.max_gears) usrConf.startgear = usrConf.max_gears;
            gear = usrConf.startgear+1;
            in_menu = usrConf.menu_timeout;
          }
        }
      }*/
    }
    //System is NOT locked
    else
    {
      if(ctminus())
      {
        in_menu = usrConf.menu_timeout+1;
        delay(50);
        unsigned long pushtime = millis();
        bool decrease_once = 1;
        //minus decreases gear, holding locks system
        while(ctminus())
        {
          delay(10);
          if(millis() - pushtime > usrConf.lock_waittime)
          {
            if(thr_scaled < 10)
            {
              /*if(!usrConf.no_lock)
              {
                system_locked = 1;
                displayLock();
              }*/
              in_menu = usrConf.menu_timeout;
            }
            while(ctminus())
            {
              delay(100);
            }
            break;
          }
          if(millis() - pushtime > usrConf.gear_change_waittime)
          {
            if(decrease_once)
            {
              if(!usrConf.no_gear)
              {
                if(gear > 5) gear --;
                showNewGear();
                decrease_once = 0;
              }
              in_menu = usrConf.menu_timeout;
            }
          }
        }
        if(!(millis() - pushtime > usrConf.lock_waittime))
        {
          while(ctminus())
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
      else if(ctplus())
      {
        in_menu = usrConf.menu_timeout+1;
        delay(50);
        unsigned long pushtime = millis();
        bool increase_once = 1;
        //Plus decreases gear, holding powers off
        while(ctplus())
        {
          delay(10);
          if(millis() - pushtime > usrConf.gear_change_waittime)
          {
            if(increase_once)
            {
              if(!usrConf.no_gear)
              {
                if(gear < usrConf.max_gears-1) gear ++;
                showNewGear();
                increase_once = 0;
              }
              in_menu = usrConf.menu_timeout;
            }
          }
        }
        if(!(millis() - pushtime > usrConf.lock_waittime))
        {
          while(ctplus())
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
    }
  }
  //Handle errors
  else
  {
    if(ctminus() | ctplus())
    {
      in_menu = usrConf.menu_timeout+1;
      delay(500);
      if(ctminus() | ctplus())
      {
        remote_error = 0;
        //displayError(DASH);
        //TODO
        while(ctminus() | ctplus()) delay(1);
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

void checkCal()
{
  #define CAL_MIN_DIFF 400

  //Check if calibration is OK
  if(!usrConf.cal_ok)
  {
    Serial.println("Entering Calibration...");
    delay(2000);

    Serial.println("Hands Off!");
    delay(3000);

    uint16_t thr_filter_raw = 0;
    uint16_t tog_filter_raw = 0;

    for(int i = 0; i < BUFFSZ; i++)
    {
      thr_filter_raw += thr_raw[i]/BUFFSZ;
      tog_filter_raw += tog_raw[i]/BUFFSZ;
    }

    usrConf.thr_idle = thr_filter_raw;
    usrConf.tog_mid =  tog_filter_raw;

    Serial.println("Full throttle!");
    delay(3000);

    thr_filter_raw = tog_filter_raw = 0;
    for(int i = 0; i < BUFFSZ; i++)
    {
      thr_filter_raw += thr_raw[i]/BUFFSZ;
      tog_filter_raw += tog_raw[i]/BUFFSZ;
    }

    usrConf.thr_pull = thr_filter_raw;

    Serial.println("Toggle left");
    delay(3000);

    thr_filter_raw = tog_filter_raw = 0;
    for(int i = 0; i < BUFFSZ; i++)
    {
      thr_filter_raw += thr_raw[i]/BUFFSZ;
      tog_filter_raw += tog_raw[i]/BUFFSZ;
    }

    usrConf.tog_left = tog_filter_raw;

    Serial.println("Toggle right");
    delay(3000);

    thr_filter_raw = tog_filter_raw = 0;
    for(int i = 0; i < BUFFSZ; i++)
    {
      thr_filter_raw += thr_raw[i]/BUFFSZ;
      tog_filter_raw += tog_raw[i]/BUFFSZ;
    }

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
      if(usrConf.tog_left - usrConf.tog_mid > CAL_MIN_DIFF/2 &&  usrConf.tog_mid - usrConf.tog_right > CAL_MIN_DIFF/2)
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
      if(usrConf.tog_mid - usrConf.tog_left > CAL_MIN_DIFF/3 &&  usrConf.tog_right - usrConf.tog_mid > CAL_MIN_DIFF/3)
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
    }
    else
    {
      usrConf.cal_ok = 0;
      Serial.println("Cal Error!");
      while(1) delay(100);
    }
  }
}