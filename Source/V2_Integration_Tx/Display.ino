// V3 - 2026-04-21 - Updated DISPLAY_MODE_SPEED case in renderOperationalDisplay() to show TX GPS speed when speed_src 2/3/5
// V3 - 2026-04-22 - Added GPS status dot at C7 R0 in updateBargraphs(); fixed digit-clear mask 0xFF00→0xFF80 to preserve C7
// V3 - 2026-04-27 - P8: Fixed displayDigits() clamp 29→33; ANIMATION_DELAY 80→40; ET handler; added renderRtmInfoDisplay()
// V2.5-Evo - 2026-04-28 - P9: fontCompact3x7 + showFullScreenMessage() + E71 full-screen flash
// V2.5-Evo - 2026-04-28 - P9 S3+S4: displayDistanceInUnits() + R5 proximity bar

extern bool fm_armed;  // defined in RTMState.ino — needed by updateR5ProximityBar()

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
  // V3 - 2026-04-27 - P8: Clamp raised 29→33. LET_R(30)/LET_N(31)/LET_S(32)/LET_M(33) were
  // added to num0[] after this clamp was written, causing them to silently render as BLANK.
  if (dig1 > 33) dig1 = BLANK;
  if (dig2 > 33) dig2 = BLANK;

  //Delete whole number field
  for(int i = 1; i < 7; i++)
  {
    displayBuffer[i] &= 0xFF80;  // preserve bit 7 (C7 = GPS status dot)
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
    // V3 - 2026-04-28 - P9: render "100" via fontCompact3x7 across C0-C8. Non-blocking.
    // Clears C0-C8 (R0-R6) then writes '1'(C0-C2), '0'(C3-C5), '0'(C6-C8). C9 unchanged.
    // Column bytes from fc3x7_1={0x40,0x7F,0x42}, fc3x7_0={0x3E,0x41,0x3E} (see font table).
    for (int row = 1; row <= 7; row++) displayBuffer[row] &= ~0x01FFu;
    static const uint8_t k100cols[9] = {0x40,0x7F,0x42, 0x3E,0x41,0x3E, 0x3E,0x41,0x3E};
    for (int col = 0; col < 9; col++) {
      uint8_t bits = k100cols[col];
      for (int row = 0; row < 7; row++)
        if (bits & (1u << row)) displayBuffer[row + 1] |= (1u << col);
    }
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

// ============================================================
// V2.5-Evo - 2026-04-28 - P9: COMPACT 3×7 FONT
// Source: docs/Dot_Matrix_Display_10x7_Render.html — fontCompact JavaScript object.
// Each entry: 3 bytes = 3 display columns (col[0]=leftmost, col[2]=rightmost).
// Each byte: 7 bits for 7 display rows — bit 0 = R0 (top), bit 6 = R6 (bottom).
// Space = 1 dark column, handled by the caller (not in this table).
// Supported chars: A E F M P S r t 0 1 2 3 7
// Bitmaps extracted from HTML row-major values by transposing bit2→col0, bit1→col1, bit0→col2.
// Fc3x7Entry struct defined in BREmote_V2_Tx.h (must precede Arduino auto-prototype emission).
// ============================================================

static const Fc3x7Entry fc3x7_A = {{0x7E, 0x09, 0x7E}};
// V3 - 2026-04-28 - P9 font fix: col[0]↔col[2] swapped for all asymmetric characters.
// Physical display maps software col[0] to the rightmost physical LED column, so the
// HTML fontCompact bit2 (visual-left) must go into col[2], bit0 (visual-right) into col[0].
static const Fc3x7Entry fc3x7_E = {{0x41, 0x49, 0x7F}};
static const Fc3x7Entry fc3x7_F = {{0x01, 0x09, 0x7F}};
static const Fc3x7Entry fc3x7_M = {{0x7F, 0x06, 0x7F}};  // symmetric — unchanged
static const Fc3x7Entry fc3x7_P = {{0x06, 0x09, 0x7F}};
static const Fc3x7Entry fc3x7_S = {{0x31, 0x49, 0x46}};
static const Fc3x7Entry fc3x7_r = {{0x04, 0x08, 0x7C}};
static const Fc3x7Entry fc3x7_t = {{0x04, 0x7F, 0x04}};  // symmetric — unchanged
static const Fc3x7Entry fc3x7_0 = {{0x3E, 0x41, 0x3E}};  // symmetric — unchanged
static const Fc3x7Entry fc3x7_1 = {{0x40, 0x7F, 0x42}};
static const Fc3x7Entry fc3x7_2 = {{0x46, 0x49, 0x71}};
static const Fc3x7Entry fc3x7_3 = {{0x36, 0x49, 0x41}};
static const Fc3x7Entry fc3x7_7 = {{0x07, 0x19, 0x61}};

// Returns pointer to 3-column bitmap for c, or nullptr for unsupported characters.
static const Fc3x7Entry* fc3x7GetChar(char c)
{
  switch (c) {
    case 'A': return &fc3x7_A;
    case 'E': return &fc3x7_E;
    case 'F': return &fc3x7_F;
    case 'M': return &fc3x7_M;
    case 'P': return &fc3x7_P;
    case 'S': return &fc3x7_S;
    case 'r': return &fc3x7_r;
    case 't': return &fc3x7_t;
    case '0': return &fc3x7_0;
    case '1': return &fc3x7_1;
    case '2': return &fc3x7_2;
    case '3': return &fc3x7_3;
    case '7': return &fc3x7_7;
    default:  return nullptr;
  }
}

// ============================================================
// V2.5-Evo - 2026-04-28 - P9: Full-screen one-time confirmation flash.
// Clears ALL 10 columns × 7 rows (including C8 temp bar, C9 signal bar, R5 proximity, R6 battery).
// Renders msg using fontCompact3x7. Space = 1 dark column. Each other char = 3 columns.
// Caller must ensure total columns ≤ 10. Holds for duration_ms (blocking — acceptable
// for one-time 2-second events). Other FreeRTOS tasks (vibrationTask, updateBargraphs)
// continue to run. On return, the next renderRtmInfoDisplay()/renderOperationalDisplay()
// call rebuilds displayBuffer naturally.
// ============================================================
void showFullScreenMessage(const char* msg, uint16_t duration_ms)
{
  for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000;
  clearDisplay();

  uint8_t col = 0;
  for (int ci = 0; msg[ci] != '\0' && col < 10; ci++)
  {
    if (msg[ci] == ' ')
    {
      col++;
    }
    else
    {
      const Fc3x7Entry* entry = fc3x7GetChar(msg[ci]);
      if (entry == nullptr) { col++; continue; }
      for (int fc = 0; fc < 3 && col < 10; fc++, col++)
      {
        uint8_t colBits = entry->col[fc];
        for (int row = 0; row < 7; row++)
        {
          if (colBits & (1u << row))
            displayBuffer[row + 1] |= (1u << col);
        }
      }
    }
  }

  updateDisplay();
  delay(duration_ms);
  for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000;
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
        // V3 - 2026-04-21 - Show TX GPS speed when a TX-GPS unit is selected (speed_src 2/3/5);
        // fall back to RX telemetry speed for all other speed_src values.
        // tx_gps_speed is already 0xFF when no fix, so displayShowTwoDigitOrDash renders "--" automatically.
        case DISPLAY_MODE_SPEED:
          if (usrConf.speed_src == 2 || usrConf.speed_src == 3 || usrConf.speed_src == 5)
            displayShowTwoDigitOrDash(tx_gps_speed);
          else
            displayShowTwoDigitOrDash(telemetry.foil_speed);
          break;
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
    // V2.5-Evo - 2026-04-28 - P9: E71 water ingress — full-screen blinking flash.
    // All existing E71 haptic (Pattern 3: 5×500ms) and detection logic are UNCHANGED.
    // Display-only change: "E 71" rendered full-screen at 250ms on/off until error clears.
    // E(3) + space(1) + 7(3) + 1(3) = 10 columns exactly.
    if (remote_error == 71)
    {
      static unsigned long e71_blink_ms    = 0;
      static bool          e71_blink_state = false;
      if (millis() - e71_blink_ms >= 250)
      {
        e71_blink_state = !e71_blink_state;
        e71_blink_ms    = millis();
        if (e71_blink_state)
        {
          for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000;
          const char* e71msg = "E 71";
          uint8_t col = 0;
          for (int ci = 0; e71msg[ci] && col < 10; ci++) {
            if (e71msg[ci] == ' ') { col++; continue; }
            const Fc3x7Entry* en = fc3x7GetChar(e71msg[ci]);
            if (!en) { col++; continue; }
            for (int fc = 0; fc < 3 && col < 10; fc++, col++) {
              uint8_t cb = en->col[fc];
              for (int r = 0; r < 7; r++) if (cb & (1u << r)) displayBuffer[r+1] |= (1u << col);
            }
          }
        }
        else
        {
          for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000;
        }
        updateDisplay();
      }
      return;
    }

    // V3 - 2026-04-27 - P8: ET error (code=20=LET_T) shows "--" and auto-clears after 3s.
    // ET is absent from V3 RX source; this guard is defensive for legacy or future paths.
    // System stays in manual mode; no RTM/FM engagement; no vibration on ET.
    static unsigned long et_show_ms = 0;
    if (remote_error == LET_T)
    {
      if (et_show_ms == 0) et_show_ms = millis();
      displayDigits(DASH, DASH);
      updateDisplay();
      if (millis() - et_show_ms > 3000UL)
      {
        remote_error = 0;
        et_show_ms   = 0;
      }
    }
    else
    {
      et_show_ms = 0;
      displayDigits(LET_E, remote_error < 10 ? remote_error : DASH);
      updateDisplay();
    }
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
      displayBuffer[i] &= 0xFF80;  // preserve bit 7 (C7 = GPS status dot)
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
      displayBuffer[i] &= 0xFF80;  // preserve bit 7 (C7 = GPS status dot)
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
  pinMode(P_MOT, OUTPUT);     // configure haptic motor pin for output
  digitalWrite(P_MOT, HIGH); // haptic motor ON — boot notification pulse

  scroll4Digits(LET_B, 0, 0, LET_T, 100);

  digitalWrite(P_MOT, LOW);  // haptic motor OFF
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
    displayBuffer[i] &= 0xFF80;  // preserve bit 7 (C7 = GPS status dot)
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
    displayBuffer[i] &= 0xFF80;  // preserve bit 7 (C7 = GPS status dot)
  }

  displayBuffer[1] = (displayBuffer[1] & 0xFF80) | 0x1F;  // I-1: preserve bit 7 (GPS dot)
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
    displayBuffer[i] &= 0xFF80;  // preserve bit 7 (C7 = GPS status dot)
  }

  displayBuffer[5] |= 0x7F;
  displayBuffer[4] |= 0x41;
  displayBuffer[3] |= 0x7F;
  displayBuffer[2] |= 0x22;
  displayBuffer[1] |= 0x1C;
  updateDisplay();
}

#define ANIMATION_DELAY 40  // V3 - 2026-04-27 - P8: 80→40ms (2× faster unlock down-arrow)
void unlockAnimation()
{
  for(int i = 1; i < 7; i++)
  {
    displayBuffer[i] &= 0xFF80;  // preserve bit 7 (C7 = GPS status dot)
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

// GPS rejection flag — set by future TX Phase A anti-spoofing (GPS.ino) via extern.
// Not static so GPS.ino can write it when that code is added.
volatile bool gps_rejected = false;

void updateBargraphs(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(200);
  while (1) 
  {
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
    // ---- GPS status dot  C7 R0 ----------------------------------------
    // Bit 7 of displayBuffer[1] (row R0, col C7).
    // Preserved across digit updates by the 0xFF80 clear mask above.
    static uint32_t gps_dot_ms    = 0;      // millis() of last blink toggle
    static bool     gps_dot_state = false;  // current dot on/off state

    if (!usrConf.gps_en) {
      // GPS disabled — dot off
      displayBuffer[1] &= ~(1u << 7);
    } else if (gps_rejected) {
      // GPS rejected (anti-spoofing) — fast blink 250 ms
      if (millis() - gps_dot_ms >= 250) {
        gps_dot_state = !gps_dot_state;
        gps_dot_ms    = millis();
      }
      if (gps_dot_state) displayBuffer[1] |=  (1u << 7);
      else               displayBuffer[1] &= ~(1u << 7);
    } else if (gps_tx.location.isValid() &&
               gps_tx.location.age() < usrConf.tx_gps_stale_timeout_ms) {
      // Valid fresh fix — solid on; reset timer so blink starts cleanly on state change
      gps_dot_state = true;
      gps_dot_ms    = millis();
      displayBuffer[1] |= (1u << 7);
    } else {
      // No fix or stale fix — slow blink 1 s (acquiring)
      if (millis() - gps_dot_ms >= 1000) {
        gps_dot_state = !gps_dot_state;
        gps_dot_ms    = millis();
      }
      if (gps_dot_state) displayBuffer[1] |=  (1u << 7);
      else               displayBuffer[1] &= ~(1u << 7);
    }
    // ---- End GPS status dot --------------------------------------------

    displayVertBargraph(9, sq_graph, 2);
    updateDisplay();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ============================================================
// V2.5-Evo - 2026-04-28 - P9 S3: Distance unit conversion for TX display.
// Inputs: dist_m in metres (float). Outputs: displayDigits() call + optional
// decimal dot at C3 R4 (displayBuffer[5] bit 3) for fractional values.
// Rules:
//   Metric (0):   <1 m → "00"; 1-99 m → whole metres; 100+ m → X.X km (dot)
//   Imperial (1): <4 ft → "00"; 4-99 ft → whole feet; 100+ ft → X.X mi (dot)
// displayDigits() must be called before setting decimal dot (it clears R4).
// ============================================================
static void displayDistanceInUnits(float dist_m)
{
  if (usrConf.dist_unit == 1)
  {
    // Imperial
    float dist_ft = dist_m * 3.28084f;
    if (dist_ft < 4.0f)
    {
      displayDigits(0, 0);
    }
    else if (dist_ft < 100.0f)
    {
      uint8_t ft = (uint8_t)dist_ft;
      if (ft > 99) ft = 99;
      displayDigits(ft / 10, ft % 10);
    }
    else
    {
      // Tenths of miles: 528 ft = 0.1 mi, cap at 9.9 mi
      float miles = dist_ft / 5280.0f;
      uint8_t whole = (uint8_t)miles;
      uint8_t frac  = (uint8_t)((miles - whole) * 10.0f + 0.5f);
      if (whole > 9) { whole = 9; frac = 9; }
      displayDigits(whole, frac);
      displayBuffer[5] |= (1u << 3);  // decimal dot C3 R4
    }
  }
  else
  {
    // Metric (default: dist_unit == 0)
    if (dist_m < 1.0f)
    {
      displayDigits(0, 0);
    }
    else if (dist_m < 100.0f)
    {
      uint8_t m = (uint8_t)dist_m;
      if (m > 99) m = 99;
      displayDigits(m / 10, m % 10);
    }
    else
    {
      // Tenths of km: 100 m = 1.0 km, cap at 9.9 km
      float km = dist_m / 100.0f;
      uint8_t whole = (uint8_t)km;
      uint8_t frac  = (uint8_t)((km - whole) * 10.0f + 0.5f);
      if (whole > 9) { whole = 9; frac = 9; }
      displayDigits(whole, frac);
      displayBuffer[5] |= (1u << 3);  // decimal dot C3 R4
    }
  }
}

// ============================================================
// V3 - 2026-04-27 - P8: RTM/FM ACTIVE INFO DISPLAY
// Called from loop() instead of renderOperationalDisplay() when rtm_tx_active==true.
// Modes: 0=distance to TX (default), 1=speed, 2=alternating 2.5s each.
// Distance telemetry encoding (rtm_distance byte from RX):
//   0-99  → tenths of meter (0.0–9.9 m), displayed as "X.X" with C3 decimal dot
//   100-254 → whole meters (value-90 = actual m; 100=10m, 199=109m, 254=164m max)
//   255   → N/A / RTM inactive on RX side → show "--"
// ============================================================
void renderRtmInfoDisplay()
{
  static unsigned long alt_last_switch_ms = 0;
  static uint8_t       alt_showing        = 0;  // 0=distance, 1=speed (used in mode 2)

  uint8_t mode = usrConf.rtm_display_mode;

  if (mode == 2)
  {
    if (alt_last_switch_ms == 0)
      alt_last_switch_ms = millis();  // start timer on first call; show distance first
    else if (millis() - alt_last_switch_ms >= 2500UL)
    {
      alt_showing        = alt_showing ? 0 : 1;
      alt_last_switch_ms = millis();
    }
    mode = alt_showing;
  }

  if (mode == 0)
  {
    // Distance mode — decode telemetry.rtm_distance then convert to selected unit
    uint8_t d = telemetry.rtm_distance;
    if (d == 0xFF)
    {
      displayDigits(DASH, DASH);
    }
    else
    {
      float actual_m;
      if (d < 100)
        actual_m = d / 10.0f;
      else
        actual_m = (float)(d - 90);

      displayDistanceInUnits(actual_m);
    }
  }
  else
  {
    // Speed mode (mode == 1 or speed half of mode 2)
    if (usrConf.speed_src == 2 || usrConf.speed_src == 3 || usrConf.speed_src == 5)
      displayShowTwoDigitOrDash(tx_gps_speed);
    else
      displayShowTwoDigitOrDash(telemetry.foil_speed);
  }

  updateR5ProximityBar();
  updateDisplay();
}

// ============================================================
// V2.5-Evo - 2026-04-28 - P9 S4: R5 PROXIMITY BAR
// Active during RTM or FM armed. Blinks 1000 ms on / 500 ms off.
// Suppressed during showFullScreenMessage() — buffer is cleared
// and this function is not called during blocking messages.
//
// RTM: square-root curve from arm distance to 0.
//   rtm_arm_dist_m captured at engage; pixels = round(sqrt(current/arm)*10), 0-10.
//   Bar fills C0→C9 (left to right) and shrinks from right as buggy closes in.
// FM:  stub — single center pixel C4 R5. Full FM bar deferred to Priority 10.
// ============================================================
void updateR5ProximityBar()
{
  static unsigned long r5_blink_ms    = 0;
  static bool          r5_blink_state = false;

  // Blink: 1000 ms on, 500 ms off
  unsigned long now = millis();
  if (r5_blink_state)
  {
    if (now - r5_blink_ms >= 1000UL) { r5_blink_state = false; r5_blink_ms = now; }
  }
  else
  {
    if (now - r5_blink_ms >= 500UL)  { r5_blink_state = true;  r5_blink_ms = now; }
  }

  displayBuffer[6] = 0x0000;  // clear R5 before every call

  if (!r5_blink_state) return;  // off phase — leave R5 dark

  if (rtm_tx_active)
  {
    if (rtm_arm_dist_m <= 0.0f) return;  // no valid arm distance — skip to avoid divide-by-zero

    uint8_t d = telemetry.rtm_distance;
    if (d == 0xFF) return;  // no distance data — leave R5 dark

    float current_m = (d < 100) ? d / 10.0f : (float)(d - 90);

    float ratio = current_m / rtm_arm_dist_m;
    if (ratio > 1.0f) ratio = 1.0f;
    uint8_t pixels = (uint8_t)(sqrtf(ratio) * 10.0f + 0.5f);
    if (pixels > 10) pixels = 10;

    for (uint8_t c = 0; c < pixels; c++)
      displayBuffer[6] |= (1u << c);
  }
  else if (fm_armed)
  {
    // FM stub: single center pixel C4 R5 — full FM bar deferred to Priority 10
    displayBuffer[6] |= (1u << 4);
  }
}
