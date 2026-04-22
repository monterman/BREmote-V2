static void clearDisplayRaw()
{
  Wire.beginTransmission(DISPLAY_ADDRESS);
  uint8_t buffer[17];
  for (uint8_t i = 0; i < 17; i++) {
    buffer[i] = 0x00;
  }
  Wire.write(buffer,17);
  Wire.endTransmission();
}

void setDisplayActivityEnabled(bool enabled)
{
  if (enabled == display_activity_enabled) return;

  if (enabled)
  {
    if (!beginDisplay())
    {
      display_activity_enabled = false;
      return;
    }
    display_activity_enabled = true;
    initDisplay();
    updateDisplay();
    return;
  }

  clearDisplayRaw();
  Wire.beginTransmission(DISPLAY_ADDRESS);
  Wire.write(0x80); // display off
  Wire.endTransmission();
  display_activity_enabled = false;
}

bool isDisplayActivityEnabled()
{
  return display_activity_enabled;
}

void startupDisplay()
{
  Serial.print("Starting Display...");
  if(!beginDisplay())
  {
    Serial.println(" Failed");
    while(1) delay(100);
  }

  clearDisplayBuffer();
  clearDisplay();
  initDisplay();
  Serial.println(" Done");
}

bool beginDisplay()
{
  Wire.beginTransmission(DISPLAY_ADDRESS);
  return (0 == Wire.endTransmission());
}

// Unused — global digitBuffer was shadowed by locals everywhere
//void clearDigitBuffer()
//{
//  for(int i = 0; i < 6; i++)
//  {
//    digitBuffer[i] = 0x00;
//  }
//}

void displayDigits(uint8_t dig1, uint8_t dig2)
{
  if (dig1 > 29) dig1 = BLANK;
  if (dig2 > 29) dig2 = BLANK;

  //Delete whole number field
  for(int i = 1; i < 7; i++)
  {
    displayBuffer[i] &= 0xFF00;
  }

  uint8_t digitBuffer[7];

  for(int i = 0; i < 7; i++)
  {
    digitBuffer[i] = 0;
  }

  digitBuffer[0] = num0[dig1][0];
  digitBuffer[1] = num0[dig1][1];
  digitBuffer[2] = num0[dig1][2];

  digitBuffer[4] = num0[dig2][0];
  digitBuffer[5] = num0[dig2][1];
  digitBuffer[6] = num0[dig2][2];


  for(int j = 5; j >= 0; j--)
  {
    for(int i = 0; i < 7; i++)
    {
      displayBuffer[j] |= ((digitBuffer[i]>>(5-j))&0x01)<<i;
    }
  }
}

void initDisplay()
{
  if(!isDisplayActivityEnabled()) return;

  Wire.beginTransmission(DISPLAY_ADDRESS);
  //System Oscillator on
  Wire.write(0x21);
  Wire.endTransmission();

  Wire.beginTransmission(DISPLAY_ADDRESS);
  //On, no blinking
  Wire.write(0x81);
  Wire.endTransmission();
  
  setBrightness(0x0F);
}

void setBrightness(uint8_t level)
{
  if(!isDisplayActivityEnabled()) return;

  //Set brightness x00..x0F
  if(level > 0x0F) level = 0x0F;
  Wire.beginTransmission(DISPLAY_ADDRESS);
  //Full brightness
  Wire.write(0xE0 | level);
  Wire.endTransmission();
}

void updateDisplay()
{
  if(!isDisplayActivityEnabled()) return;

  //This is where the mapping takes place
  //displayBuffer keeps the desired matrix config,
  //but the connection is not 1:1
  //See the row_mapper[] and col_mapper[] arrays

  uint8_t sendBuffer[17];

  for(int i=0; i < 17; i++)
  {
    sendBuffer[i] = 0x00;
  }

  //Go through the 8 columns
  for(int j = 0; j < 7; j++)
  {
    //And map the 8 bits + 2 bits (in total 10)
    for( int k=0; k<8; k++)
    {
      sendBuffer[2*j+1] |= ((displayBuffer[col_mapper[j]]>>row_mapper[k])&0x01)<<k;
    }
    for( int k=0; k<2; k++)
    {
      sendBuffer[2*j+2] |= ((displayBuffer[col_mapper[j]]>>row_mapper[k+8])&0x01)<<k;
    }
  }

  Wire.beginTransmission(DISPLAY_ADDRESS);
  Wire.write(sendBuffer,17);
  Wire.endTransmission();
}

void clearDisplayBuffer()
{
  for(int i = 0; i < 8; i++)
  {
    displayBuffer[i] = 0x0000;
  }
}

void clearDisplay()
{  
  if(!isDisplayActivityEnabled()) return;
  clearDisplayRaw();
}

void displayVertBargraph(uint8_t location, uint8_t length, uint8_t offset)
{
  if(location > 9) location = 9;
  if(length > 7-offset) length = 7-offset;
  
  int i = 0;

  for(; i < length; i++)
  {
    displayBuffer[7-offset-i] |= 0x01<<location; 
  }
  for(; i < 7-offset; i++)
  {
    displayBuffer[7-offset-i] &= ~(0x01<<location); 
  }
}

void displayHorzBargraph(int location, int length)
{
  if(location > 7) location = 7;
  if(length > 10) length = 10;
      
  for(int i = 0; i < length; i++)
  {
    displayBuffer[location] |= 0x01<<i;
  }
  for(int i = length; i < 10; i++)
  {
    displayBuffer[location] &= ~(0x01<<i);
  }
}

void showNewGear()
{
  displayDigits(LET_L, gear);
  updateDisplay();
}

void showCapPercent()
{
  uint8_t cap = throttleGetCapPercent();
  uint8_t tens = cap / 10;
  uint8_t ones = cap % 10;
  if(cap >= 100)
  {
    // Show "FL" for full (100%)
    displayDigits(LET_F, LET_L);
  }
  else
  {
    displayDigits(tens, ones);
  }
  updateDisplay();
}

static void displayShowTwoDigitOrDash(uint8_t value)
{
  if(value != 0xFF)
  {
    displayDigits(value/10, value-10*(value/10));
  }
  else
  {
    displayDigits(DASH, DASH);
  }
}

void renderOperationalDisplay()
{
  if(remote_error == 0)
  {
    if(system_locked)
    {
      displayLock();
    }
    else
    {
      // If current mode is unavailable, silently advance to next available
      if(!isDisplayModeAvailable(display_mode))
      {
        for(uint8_t i = 0; i < DISPLAY_MODE_COUNT; i++) {
          display_mode = (display_mode + 1) % DISPLAY_MODE_COUNT;
          if(isDisplayModeAvailable(display_mode)) break;
        }
      }
      switch(display_mode) {
        case DISPLAY_MODE_TEMP:   displayShowTwoDigitOrDash(telemetry.foil_temp); break;
        case DISPLAY_MODE_SPEED:  displayShowTwoDigitOrDash(telemetry.foil_speed); break;
        case DISPLAY_MODE_POWER:  displayShowTwoDigitOrDash(telemetry.foil_power != 0xFF ? min((uint8_t)(telemetry.foil_power / 2), (uint8_t)99) : 0xFF); break;
        case DISPLAY_MODE_BAT:    displayShowTwoDigitOrDash(telemetry.foil_bat); break;
        case DISPLAY_MODE_THR:    displayShowTwoDigitOrDash(thr_scaled * 99 / 255); break;
        case DISPLAY_MODE_INTBAT: displayShowTwoDigitOrDash((uint8_t)(int_bat_volt * 10)); break;
        default: displayShowTwoDigitOrDash(telemetry.foil_temp); break;
      }
      updateDisplay();
    }
  }
  else
  {
    displayDigits(LET_E, remote_error < 10 ? remote_error : DASH);
    updateDisplay();
  }
}

void displayError(int err)
{
  displayDigits(LET_E, min(err, 29));  // Clamp to prevent array overflow (num0[30][3])
  updateDisplay();
}

void scroll3Digits(uint8_t dig1, uint8_t dig2, uint8_t dig3, int del)
{
  uint8_t digitBuffer[14];

  for(int i = 0; i < 14; i++)
  {
    digitBuffer[i] = 0;
  }

  digitBuffer[0] = num0[dig1][0];
  digitBuffer[1] = num0[dig1][1];
  digitBuffer[2] = num0[dig1][2];

  digitBuffer[4] = num0[dig2][0];
  digitBuffer[5] = num0[dig2][1];
  digitBuffer[6] = num0[dig2][2];

  digitBuffer[8] = num0[dig3][0];
  digitBuffer[9] = num0[dig3][1];
  digitBuffer[10] = num0[dig3][2];

  digitBuffer[12] = 0x04;

  for(int k = 0; k < 14; k++)
  {
    //Delete whole number field
    for(int i = 1; i < 7; i++)
    {
      displayBuffer[i] &= 0xFF00;
    }

    for(int j = 5; j >= 0; j--)
    {
      for(int i = 0; i < 7; i++)
      {
        displayBuffer[j] |= ((digitBuffer[(i+k)%14]>>(5-j))&0x01)<<i;
      }
    }
    updateDisplay();
    delay(del);
    checkSerial();
  }
}

void scroll4Digits(uint8_t dig1, uint8_t dig2, uint8_t dig3, uint8_t dig4, int del)
{
  uint8_t digitBuffer[18];

  for(int i = 0; i < 18; i++)
  {
    digitBuffer[i] = 0;
  }

  digitBuffer[0] = num0[dig1][0];
  digitBuffer[1] = num0[dig1][1];
  digitBuffer[2] = num0[dig1][2];

  digitBuffer[4] = num0[dig2][0];
  digitBuffer[5] = num0[dig2][1];
  digitBuffer[6] = num0[dig2][2];

  digitBuffer[8] = num0[dig3][0];
  digitBuffer[9] = num0[dig3][1];
  digitBuffer[10] = num0[dig3][2];

  digitBuffer[12] = num0[dig4][0];
  digitBuffer[13] = num0[dig4][1];
  digitBuffer[14] = num0[dig4][2];

  digitBuffer[16] = 0x04;

  for(int k = 0; k < 18; k++)
  {
    //Delete whole number field
    for(int i = 1; i < 7; i++)
    {
      displayBuffer[i] &= 0xFF00;
    }

    for(int j = 5; j >= 0; j--)
    {
      for(int i = 0; i < 7; i++)
      {
        displayBuffer[j] |= ((digitBuffer[(i+k)%18]>>(5-j))&0x01)<<i;
      }
    }
    updateDisplay();
    delay(del);
    checkSerial();
  }
}

void bootAnimation()
{
  scroll4Digits(LET_B, 0, 0, LET_T, 100);
  scroll4Digits(LET_B, 0, 0, LET_T, 100);
  displayDigits(LET_V,LET_I);
  updateDisplay();
  delay(500);

  uint8_t temp_volt = uint8_t(int_bat_volt*10);

  displayDigits(temp_volt/10,temp_volt-10*(temp_volt/10));
  updateDisplay();
  delay(1000);
}

uint8_t arrowPos = 0;
void advanceArrow()
{
  //Delete whole number field
  for(int i = 1; i < 7; i++)
  {
    displayBuffer[i] &= 0xFF00;
  }

  arrowPos ++;
  if(arrowPos >=2) arrowPos = 0;

  displayBuffer[0+arrowPos] |= 0x3E;
  displayBuffer[1+arrowPos] |= 0x3E;
  displayBuffer[2+arrowPos] |= 0x1C;
  displayBuffer[3+arrowPos] |= 0x1C;
  displayBuffer[4+arrowPos] |= 0x08;

  updateDisplay();
}

uint8_t chargeAnimationPos = 0;
void advanceChargeAnimation()
{
  //Delete whole number field
  for(int i = 1; i < 7; i++)
  {
    displayBuffer[i] &= 0xFF00;
  }

  displayBuffer[1] = 0x1F;
  displayBuffer[4] = 0x1F;
  
  chargeAnimationPos++;
  if(chargeAnimationPos > 4) chargeAnimationPos = 0;

  if(chargeAnimationPos == 0)
  {
    displayBuffer[2] = 0x21;
    displayBuffer[3] = 0x21;
  }
  else if(chargeAnimationPos == 1)
  {
    displayBuffer[2] = 0x23;
    displayBuffer[3] = 0x23;
  }
  else if(chargeAnimationPos == 2)
  {
    displayBuffer[2] = 0x27;
    displayBuffer[3] = 0x27;
  }
  else if(chargeAnimationPos == 3)
  {
    displayBuffer[2] = 0x2F;
    displayBuffer[3] = 0x2F;
  }
  else
  {
    displayBuffer[2] = 0x3F;
    displayBuffer[3] = 0x3F;
  }
}

void displayLock()
{
  //Delete whole number field
  for(int i = 1; i < 7; i++)
  {
    displayBuffer[i] &= 0xFF00;
  }

  displayBuffer[5] |= 0x7F;
  displayBuffer[4] |= 0x41;
  displayBuffer[3] |= 0x7F;
  displayBuffer[2] |= 0x22;
  displayBuffer[1] |= 0x1C;
  updateDisplay();
}

#define ANIMATION_DELAY 80
void unlockAnimation()
{
  for(int i = 1; i < 7; i++)
  {
    displayBuffer[i] &= 0xFF00;
  }

  displayBuffer[0] |= 0x3E;
  displayBuffer[1] |= 0x3E;
  displayBuffer[2] |= 0x1C;
  displayBuffer[3] |= 0x1C;
  displayBuffer[4] |= 0x08;
  updateDisplay();

  delay(ANIMATION_DELAY);

  displayBuffer[1] |= 0x3E;
  displayBuffer[2] |= 0x3E;
  displayBuffer[3] |= 0x1C;
  displayBuffer[4] |= 0x1C;
  displayBuffer[5] |= 0x08;
  updateDisplay();

  delay(ANIMATION_DELAY);

  displayBuffer[2] |= 0x3E;
  displayBuffer[3] |= 0x3E;
  displayBuffer[4] |= 0x1C;
  displayBuffer[5] |= 0x1C;
  updateDisplay();

  delay(ANIMATION_DELAY);

  displayBuffer[3] |= 0x3E;
  displayBuffer[4] |= 0x3E;
  displayBuffer[5] |= 0x1C;
  updateDisplay();

  delay(ANIMATION_DELAY);

  displayBuffer[4] |= 0x3E;
  displayBuffer[5] |= 0x3E;
  updateDisplay();

  delay(ANIMATION_DELAY);
  
  arrowPos = 0;
}

void updateBargraphs(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(200);

  while (1) 
  {
    esp_task_wdt_reset();
    if(millis()-last_packet < 1000)
    {
      if(telemetry.link_quality)
      {
        blink_bargraphs ^= 1;
        if(telemetry.foil_temp != 0xFF)
        {
          last_known_temp_graph = map( constrain(telemetry.foil_temp,20,81), 20, 70, 1, 5);
          displayVertBargraph(8,last_known_temp_graph,2);
        }
        else
        {
          if(blink_bargraphs)
          {
            displayVertBargraph(8,last_known_temp_graph,2);
          }
          else
          {
            displayVertBargraph(8,0,2);
          }
        }

        if(telemetry.foil_bat != 0xFF && telemetry.foil_bat > 5)
        {
          last_known_bat_graph = map( constrain(telemetry.foil_bat,5,100), 5, 95, 1, 10);
          displayHorzBargraph(7,last_known_bat_graph);
        }
        else
        {
          if(telemetry.foil_bat <= 5) last_known_bat_graph = 1;
          if(blink_bargraphs)
          {
            displayHorzBargraph(7,last_known_bat_graph);
          }
          else
          {
            displayHorzBargraph(7,0);
          }
        }

        sq_graph = map( telemetry.link_quality + local_link_quality, 0, 20, 0, 5);
      }
    }
    else
    {
      if(sq_graph)
      {
        sq_graph = 0;
      }
      else
      {
        sq_graph = 1;
      }
    }
    displayVertBargraph(9, sq_graph, 2);
    updateDisplay();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
