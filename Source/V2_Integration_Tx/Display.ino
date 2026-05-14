// V2.5-Evo - 2026-05-13 - SW47: unlockAnimation() per-frame clear (smear→clean arrow); boot battery display 1s→3s; ANIMATION_DELAY 40→60ms
// V2.5-Evo - 2026-05-13 - SW33b: BT status dot at C7 R1 added to updateBargraphs(); blinks from bt_dot_state
// V2.5-Evo - 2026-05-05 - 30s digit cache for foil_temp/foil_bat to suppress telemetry-drop dashes
// V2.5-Evo - 2026-05-05 - PV display: foil_power rendered as kW with decimal point (X.Y kW)
// V2.5-Evo - 2026-05-03 - displayError() clamp corrected 29→33 (H6 audit fix)
// V2.5-Evo - 2026-05-01 - Fix A: lazy-capture rtm_arm_dist_m on first valid render if missed at arm time
// V2.5-Evo - 2026-04-30 - FM R5 bar: replaced linear fill with center-expanding from C4-C5
// V2.5-Evo - 2026-04-30 - Priority 10: FM R5 proximity bar implemented in updateR5ProximityBar(); called from renderOperationalDisplay() FM path
// V2.5-Evo - 2026-04-21 - Updated DISPLAY_MODE_SPEED case in renderOperationalDisplay() to show TX GPS speed when speed_src 2/3/5
// V2.5-Evo - 2026-04-22 - Added GPS status dot at C7 R0 in updateBargraphs(); fixed digit-clear mask 0xFF00→0xFF80 to preserve C7
// V2.5-Evo - 2026-04-27 - P8: Fixed displayDigits() clamp 29→33; ANIMATION_DELAY 80→40; ET handler; added renderRtmInfoDisplay()
// V2.5-Evo - 2026-04-28 - P9: fontCompact3x7 + showFullScreenMessage() + E71 full-screen flash
// V2.5-Evo - 2026-04-28 - P9 S3+S4: displayDistanceInUnits() + R5 proximity bar
// V2.5-Evo - 2026-04-28 - Reverted P9 col[0]↔col[2] swap: col[0] is left physical column, no swap needed
// V2.5-Evo - 2026-04-28 - Security: fixed displayDistanceInUnits() metric label (km→×100m) and imperial threshold (100ft→1000ft)
// V2.5-Evo - 2026-04-28 - Changes2/3/4/6/G: cap→99; fc3x7_n; E71→"E 7"
// V2.5-Evo - 2026-04-28 - chore: removed bootSelfTest() — restore fast startup
// V2.5-Evo - 2026-04-28 - ChangeD: persistent "FM" display in renderOperationalDisplay() when fm_armed
// V2.5-Evo - 2026-04-28 - ChgDZ: displayDigitZone() — safe persistent renderer, preserves R5/R6/C7/C8/C9
// V2.5-Evo - 2026-04-28 - Bug1: showFullScreenMessage() save+reassert R6 to beat updateBargraphs() FreeRTOS task
// V2.5-Evo - 2026-04-28 - Bug3: Removed dead fm_armed stub from updateR5ProximityBar() — was unreachable from call site
// V2.5-Evo - 2026-04-28 - Bug5: fc3x7_r + fc3x7_n bitmaps corrected 0x7C→0x1E/0x04/0x02 — shift up, avoid R5
// V2.5-Evo - 2026-04-29 - Fix 4-3: extern fm_armed updated to volatile to match RTMState.ino
// V2.5-Evo - 2026-04-29 - Display: fc3x7_F middle bar R3→R2 for visual consistency
// V2.5-Evo - 2026-05-01 - FM digit zone shows fm_display_mode data (1=TX speed, 2=dist, 3=buggy spd, 4=thr%)
// V2.5-Evo - 2026-05-02 - displayMutex applied to updateBargraphs (Core 0) and main loop render path (Core 1)

extern volatile bool fm_armed;  // defined in RTMState.ino — volatile: written by loop() core 1,
                                 // read by updateBargraphs() core 0; must match definition

// ============================================================
// FOIL DATA DIGIT CACHE - holds last-known foil_temp/foil_bat values
// for up to 30s so brief VESC UART silences don't flash "--" on the digit display.
// ============================================================
static uint8_t        last_known_foil_temp            = 0xFF;
static uint8_t        last_known_foil_bat             = 0xFF;
static unsigned long  foil_temp_last_valid_ms         = 0;
static unsigned long  foil_bat_last_valid_ms          = 0;
static const unsigned long FOIL_DATA_CACHE_TIMEOUT_MS = 30000;  // 30s before admitting stale

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
  // V2.5-Evo - 2026-04-27 - P8: Clamp raised 29→33. LET_R(30)/LET_N(31)/LET_S(32)/LET_M(33) were
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
    // V2.5-Evo - 2026-04-28: full-cap shows 99. fontCompact "100" used cols 0-8; cols 7-8 map
    // to hw ROW 2/0 which are physically unconnected on this display.
    displayDigits(9, 9);
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
// Supported chars: A E F M P S n r t 0 1 2 3 7
// Bitmaps extracted from HTML row-major values by transposing bit2→col0, bit1→col1, bit0→col2.
// Fc3x7Entry struct defined in BREmote_V2_Tx.h (must precede Arduino auto-prototype emission).
// ============================================================

static const Fc3x7Entry fc3x7_A = {{0x7E, 0x09, 0x7E}};
// col[0]=left physical column, col[1]=middle, col[2]=right. No swap needed.
// Bitmaps are direct from HTML fontCompact (bit2→col[0], bit1→col[1], bit0→col[2]).
static const Fc3x7Entry fc3x7_E = {{0x7F, 0x49, 0x41}};
static const Fc3x7Entry fc3x7_F = {{0x7F, 0x05, 0x01}};
// col[1] changed 0x09→0x05: middle bar moved from R3 (bit 3) to R2 (bit 2).
// Top bar (bit 0 = R0) and right top pixel (col[2] bit 0) unchanged.
static const Fc3x7Entry fc3x7_M = {{0x7F, 0x06, 0x7F}};  // symmetric — unchanged
static const Fc3x7Entry fc3x7_P = {{0x7F, 0x09, 0x06}};
static const Fc3x7Entry fc3x7_S = {{0x46, 0x49, 0x31}};
// V2.5-Evo - 2026-04-28 - Bug5: Bitmaps shifted up 1 row so characters render R1-R4 instead
// of R2-R4. Previous 0x7C reached bit 5 (R5, proximity bar row) in showFullScreenMessage().
// 0x1E = bits 1-4 = R1-R4 (stays clear of R5). Verified with 0x1F clip in displayDigitZone.
static const Fc3x7Entry fc3x7_n = {{0x1E, 0x02, 0x1E}};  // lowercase n: both legs R1-R4, bridge at R1
static const Fc3x7Entry fc3x7_r = {{0x1E, 0x04, 0x02}};  // lowercase r: left leg R1-R4, arch at R2, stub at R1
static const Fc3x7Entry fc3x7_t = {{0x04, 0x7F, 0x04}};  // symmetric — unchanged
static const Fc3x7Entry fc3x7_0 = {{0x3E, 0x41, 0x3E}};  // symmetric — unchanged
static const Fc3x7Entry fc3x7_1 = {{0x42, 0x7F, 0x40}};
static const Fc3x7Entry fc3x7_2 = {{0x71, 0x49, 0x46}};
static const Fc3x7Entry fc3x7_3 = {{0x41, 0x49, 0x36}};
static const Fc3x7Entry fc3x7_7 = {{0x61, 0x19, 0x07}};

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
    case 'n': return &fc3x7_n;
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
// V2.5-Evo - 2026-04-28 - ChgDZ: DIGIT ZONE RENDERER (persistent-safe).
// Renders msg into C0-C6 (bits 0-6), rows R0-R4 (displayBuffer[1..5]) only.
// Does NOT touch C7 (GPS dot), C8/C9 (temp/signal bars), R5 (proximity bar), R6 (battery bar).
// Clips font bitmaps to R0-R4 using (colBits & 0x1F) — drops rows R5-R6 silently.
// Characters and spacing: 3 cols per char, 1 col per space (same as showFullScreenMessage).
// Column output limited to C0-C6 (col < 7). Does NOT call updateDisplay() — caller handles that.
// ============================================================
static void displayDigitZone(const char* msg)
{
  // Clear C0-C6 in rows R0-R4 only; preserve C7 GPS dot, C8/C9 bargraphs, R5, R6
  for (int i = 1; i <= 5; i++) displayBuffer[i] &= 0xFF80;

  uint8_t col = 0;
  for (int ci = 0; msg[ci] != '\0' && col < 7; ci++)
  {
    if (msg[ci] == ' ')
    {
      col++;
    }
    else
    {
      const Fc3x7Entry* entry = fc3x7GetChar(msg[ci]);
      if (entry == nullptr) { col++; continue; }
      for (int fc = 0; fc < 3 && col < 7; fc++, col++)
      {
        uint8_t colBits = entry->col[fc] & 0x1F;  // clip to R0-R4 (5 rows, bits 0-4)
        for (int row = 0; row < 5; row++)
        {
          if (colBits & (1u << row))
            displayBuffer[row + 1] |= (1u << col);
        }
      }
    }
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

  // V2.5-Evo - 2026-04-28 - Bug1: Save R6 (battery bar row) before the hold and re-assert
  // it every 40ms. The updateBargraphs() FreeRTOS task runs every 200ms and calls
  // displayHorzBargraph(7,...) which writes displayBuffer[7] (R6) — clearing any R6 pixels
  // belonging to the message (e.g. the bottom of 'S'). Re-asserting on a 40ms cadence
  // ensures the message wins the race for the full hold duration.
  uint16_t fs_r6  = displayBuffer[7];
  unsigned long fs_end = millis() + duration_ms;
  while (millis() < fs_end)
  {
    displayBuffer[7] = fs_r6;
    updateDisplay();
    delay(40);
  }
  for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000;
}

// V2.5-Evo - 2026-05-05 - 30s cache for foil_temp/foil_bat digit display.
// Brief telemetry drops (UART mux contention, momentary VESC silence) cause
// foil_temp/foil_bat to flicker to 0xFF on RX. Without caching, the digit
// display would show '--' for sub-second drops, distracting the rider.
// Solution: hold last-known value for up to 30s before admitting stale.
// Bargraph blink behavior in updateBargraphs() is preserved as the
// 'no fresh data right now' signal — digits show context, bars show liveness.
void updateFoilDataCache() {
  if (telemetry.foil_temp != 0xFF) {
    last_known_foil_temp    = telemetry.foil_temp;
    foil_temp_last_valid_ms = millis();
  }
  if (telemetry.foil_bat != 0xFF) {
    last_known_foil_bat    = telemetry.foil_bat;
    foil_bat_last_valid_ms = millis();
  }
}

uint8_t getEffectiveFoilTemp() {
  if (telemetry.foil_temp != 0xFF) return telemetry.foil_temp;
  if (foil_temp_last_valid_ms == 0) return 0xFF;  // never had valid data
  if (millis() - foil_temp_last_valid_ms > FOIL_DATA_CACHE_TIMEOUT_MS) return 0xFF;
  return last_known_foil_temp;
}

uint8_t getEffectiveFoilBat() {
  if (telemetry.foil_bat != 0xFF) return telemetry.foil_bat;
  if (foil_bat_last_valid_ms == 0) return 0xFF;
  if (millis() - foil_bat_last_valid_ms > FOIL_DATA_CACHE_TIMEOUT_MS) return 0xFF;
  return last_known_foil_bat;
}

void renderOperationalDisplay()
{
  updateFoilDataCache();  // refresh digit cache once per render cycle, before mutex and switch
  xSemaphoreTake(displayMutex, portMAX_DELAY);  // Core 1 render — waits for Core 0 updateBargraphs to release
  // V2.5-Evo - 2026-04-28 - ChgDZ: Persistent "FM" while Follow-Me armed, RTM not active.
  // displayDigitZone() preserves R5 proximity bar, R6 battery bar, C7 GPS dot, C8/C9 bargraphs.
  // Previous hand-written render wrote through all 7 rows, destructively clearing R5/R6.
  if (fm_armed && !rtm_tx_active)
  {
    // V2.5-Evo - 2026-05-01 - FM digit zone: show data selected by fm_display_mode instead of static "FM" text.
    // R5 center-expanding bar already signals FM active — digit zone shows useful data instead.
    // Option 1: TX GPS speed in the unit selected by speed_src (0xFF = no fix → shows "--").
    // Option 2: Distance to buggy decoded from telemetry.rtm_distance (same encoding as RTM bar).
    // Option 3: Buggy speed from RX telemetry (0xFF = not available → shows "--").
    // Option 4: Current throttle percentage 0-100.
    switch (usrConf.fm_display_mode)
    {
      case 2:
      {
        uint8_t d = telemetry.rtm_distance;
        if (d == 0xFF)
          displayDigits(DASH, DASH);
        else
        {
          float actual_m = (d < 100) ? d / 10.0f : (float)(d - 90);
          displayDistanceInUnits(actual_m);
        }
        break;
      }
      case 3:
        displayShowTwoDigitOrDash(telemetry.foil_speed);
        break;
      case 4:
        displayShowTwoDigitOrDash((uint8_t)(calcFinalThrottle() * 100U / 255U));
        break;
      case 1:
      default:
        displayShowTwoDigitOrDash(tx_gps_speed);
        break;
    }
    updateR5ProximityBar();  // Priority 10: FM following-distance bar on R5
    updateDisplay();
    xSemaphoreGive(displayMutex);
    return;
  }

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
        case DISPLAY_MODE_TEMP:   displayShowTwoDigitOrDash(getEffectiveFoilTemp()); break;
        // V2.5-Evo - 2026-04-21 - Show TX GPS speed when a TX-GPS unit is selected (speed_src 2/3/5);
        // fall back to RX telemetry speed for all other speed_src values.
        // tx_gps_speed is already 0xFF when no fix, so displayShowTwoDigitOrDash renders "--" automatically.
        case DISPLAY_MODE_SPEED:
          if (usrConf.speed_src == 2 || usrConf.speed_src == 3 || usrConf.speed_src == 5)
            displayShowTwoDigitOrDash(tx_gps_speed);
          else
            displayShowTwoDigitOrDash(telemetry.foil_speed);
          break;
        // V2.5-Evo - 2026-05-05 - PV display: kW with decimal at C3R4.
        // Encoding chain: RX VESC.ino sets foil_power = watts/50 (byte 0-255).
        // On TX: foil_power/2 = watts/100 = kW × 10.
        // Render as 'X.Y' kW with decimal point. Range 0.0-9.9 kW (cap at 99).
        // Higher-power motors (>9.9 kW) would need RX-side encoding rescale — TODO.
        case DISPLAY_MODE_POWER:
          if (telemetry.foil_power == 0xFF) {
            displayShowTwoDigitOrDash(0xFF);                       // renders "--" (no data)
          } else {
            uint8_t pv_x10 = min((uint8_t)(telemetry.foil_power / 2), (uint8_t)99);
            displayDigits(pv_x10 / 10, pv_x10 % 10);              // leading zero shown: 0.4 kW → "0.4"
            displayBuffer[5] |= (1u << 3);                         // decimal dot C3 R4; auto-cleared by next displayDigits() call
          }
          break;
        case DISPLAY_MODE_BAT:    displayShowTwoDigitOrDash(getEffectiveFoilBat()); break;
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
    // Display-only change: "E 7" rendered full-screen at 250ms on/off until error clears.
    // E(3) + space(1) + 7(3) = 7 columns, all within C0-C6 (bits 7-9 are unconnected hw ROW lines).
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
          const char* e71msg = "E 7";
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
      xSemaphoreGive(displayMutex);
      return;
    }

    // V2.5-Evo - 2026-04-27 - P8: ET error (code=20=LET_T) shows "--" and auto-clears after 3s.
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
  xSemaphoreGive(displayMutex);  // release after all render paths complete
}

void displayError(int err)
{
  displayDigits(LET_E, min(err, 33));  // clamp to 33 — num0[] has 34 entries (indices 0–33); was 29, silently wrong for err 30–33
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
  delay(3000);  // SW47: was 1000ms; extended to push padlock appearance to ~5s after boot
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

// V2.5-Evo - 2026-05-13 - SW47: ANIMATION_DELAY 40→60ms; per-frame buffer clear added (was |=-only → smeared square)
#define ANIMATION_DELAY 60
// Helper: clear digit zone preserving C7 GPS dot and C8/C9 bargraphs (bit 7 = C7)
#define ANIM_CLEAR() for(int _i = 0; _i < 7; _i++) displayBuffer[_i] &= 0xFF80

void unlockAnimation()
{
  ANIM_CLEAR();
  displayBuffer[0] |= 0x3E;
  displayBuffer[1] |= 0x3E;
  displayBuffer[2] |= 0x1C;
  displayBuffer[3] |= 0x1C;
  displayBuffer[4] |= 0x08;
  updateDisplay();
  delay(ANIMATION_DELAY);

  ANIM_CLEAR();
  displayBuffer[1] |= 0x3E;
  displayBuffer[2] |= 0x3E;
  displayBuffer[3] |= 0x1C;
  displayBuffer[4] |= 0x1C;
  displayBuffer[5] |= 0x08;
  updateDisplay();
  delay(ANIMATION_DELAY);

  ANIM_CLEAR();
  displayBuffer[2] |= 0x3E;
  displayBuffer[3] |= 0x3E;
  displayBuffer[4] |= 0x1C;
  displayBuffer[5] |= 0x1C;
  updateDisplay();
  delay(ANIMATION_DELAY);

  ANIM_CLEAR();
  displayBuffer[3] |= 0x3E;
  displayBuffer[4] |= 0x3E;
  displayBuffer[5] |= 0x1C;
  updateDisplay();
  delay(ANIMATION_DELAY);

  ANIM_CLEAR();
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
    // Take mutex before writing displayBuffer. Timeout 50ms — if Core 1 is mid-render,
    // skip this 200ms cycle entirely (vTaskDelayUntil keeps timing aligned).
    // Note: 'continue' used here, NOT 'return' — returning from a FreeRTOS task function
    // permanently terminates the task; continue skips just this cycle and loops back.
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) != pdTRUE)
    {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);  // keep 200ms cadence aligned
      continue;
    }
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

    // ---- BT status dot  C7 R1 ----------------------------------------
    // Bit 7 of displayBuffer[2] (row R1, col C7) — one row below GPS dot.
    // Preserved across digit updates by the 0xFF80 clear mask.
    static uint32_t bt_dot_ms = 0;
    static bool     bt_dot_on = false;
    switch (bt_dot_state) {
      case BT_DOT_OFF:
        bt_dot_on = false;
        bt_dot_ms = millis();
        break;
      case BT_DOT_SLOW:
        if (millis() - bt_dot_ms >= 500) { bt_dot_on = !bt_dot_on; bt_dot_ms = millis(); }
        break;
      case BT_DOT_FAST:
        if (millis() - bt_dot_ms >= 200) { bt_dot_on = !bt_dot_on; bt_dot_ms = millis(); }
        break;
    }
    if (bt_dot_on) displayBuffer[2] |=  (1u << 7);
    else           displayBuffer[2] &= ~(1u << 7);
    // ---- End BT status dot --------------------------------------------

    displayVertBargraph(9, sq_graph, 2);
    updateDisplay();
    xSemaphoreGive(displayMutex);  // release before delay — don't hold mutex during 200ms sleep
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ============================================================
// V2.5-Evo - 2026-04-28 - P9 S3: Distance unit conversion for TX display.
// Inputs: dist_m in metres (float). Outputs: displayDigits() call + optional
// decimal dot at C3 R4 (displayBuffer[5] bit 3) for fractional values.
// Rules:
//   Metric (0):   <1 m → "00"; 1-99 m → whole metres; 100+ m → X.X ×100m (dot)
//   Imperial (1): <4 ft → "00"; 4-99 ft → whole feet; 100-999 ft → X.X ×100ft (dot); ≥1000 ft → X.X mi (dot)
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
    else if (dist_ft < 1000.0f)
    {
      // 100-999 ft: tenths of ×100ft (e.g., 1.5 = 150 ft). Covers full RTM op range (100-538 ft).
      // BUG FIX (2026-04-28 audit): old threshold was 100ft — all RTM distances showed 0.0-0.1 mi.
      float dist_100ft = dist_ft / 100.0f;
      uint8_t whole = (uint8_t)dist_100ft;
      uint8_t frac  = (uint8_t)((dist_100ft - whole) * 10.0f + 0.5f);
      if (whole > 9) { whole = 9; frac = 9; }
      displayDigits(whole, frac);
      displayBuffer[5] |= (1u << 3);  // decimal dot C3 R4
    }
    else
    {
      // ≥1000 ft: tenths of miles (528 ft = 0.1 mi), cap at 9.9 mi
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
      // 100 m+: tenths of ×100m (e.g., 1.0=100m, 5.0=500m, 9.9=990m). Cap at 9.9.
      // BUG FIX (2026-04-28 audit): variable was named 'km' but formula is dist_m/100 (hectometres, not km).
      float dist_100m = dist_m / 100.0f;
      uint8_t whole = (uint8_t)dist_100m;
      uint8_t frac  = (uint8_t)((dist_100m - whole) * 10.0f + 0.5f);
      if (whole > 9) { whole = 9; frac = 9; }
      displayDigits(whole, frac);
      displayBuffer[5] |= (1u << 3);  // decimal dot C3 R4
    }
  }
}

// ============================================================
// V2.5-Evo - 2026-04-27 - P8: RTM/FM ACTIVE INFO DISPLAY
// Called from loop() instead of renderOperationalDisplay() when rtm_tx_active==true.
// Modes: 0=distance to TX (default), 1=speed, 2=alternating 2.5s each.
// Distance telemetry encoding (rtm_distance byte from RX):
//   0-99  → tenths of meter (0.0–9.9 m), displayed as "X.X" with C3 decimal dot
//   100-254 → whole meters (value-90 = actual m; 100=10m, 199=109m, 254=164m max)
//   255   → N/A / RTM inactive on RX side → show "--"
// ============================================================
void renderRtmInfoDisplay()
{
  xSemaphoreTake(displayMutex, portMAX_DELAY);  // Core 1 render — waits for Core 0 updateBargraphs to release
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
  xSemaphoreGive(displayMutex);
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
// FM:  R5 is cleared; FM proximity bar deferred to Priority 10 (stub removed — Bug3).
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
    uint8_t d = telemetry.rtm_distance;
    if (d == 0xFF) return;  // no distance data — leave R5 dark

    float current_m = (d < 100) ? d / 10.0f : (float)(d - 90);

    // rtm_arm_dist_m is captured at arm-engage time but RX telemetry may still be 0xFF
    // at that instant (RX hasn't transitioned to active yet). Lazily capture it from the
    // first valid render call — buggy is still near arm distance at this point.
    if (rtm_arm_dist_m <= 0.0f) rtm_arm_dist_m = current_m;
    if (rtm_arm_dist_m <= 0.0f) return;  // buggy literally at 0 m at arm — skip to avoid divide-by-zero

    float ratio = current_m / rtm_arm_dist_m;
    if (ratio > 1.0f) ratio = 1.0f;
    uint8_t pixels = (uint8_t)(sqrtf(ratio) * 10.0f + 0.5f);
    if (pixels > 10) pixels = 10;

    for (uint8_t c = 0; c < pixels; c++)
      displayBuffer[6] |= (1u << c);
  }
  else if (fm_armed)
  {
    // FM proximity bar: center-expanding from C4+C5 outward.
    // Full bar (C0-C9, 10 pixels) = buggy right next to user (0 m).
    // Sweet-spot distance (half_width=1) = just C4+C5 lit (2 center pixels).
    // Bar grows outward symmetrically as buggy gets closer; dark when >= 30 m.
    // Uses the same telemetry.rtm_distance byte the RX always sends (RX→TX distance).
    uint8_t d = telemetry.rtm_distance;
    if (d == 0xFF) return;  // no distance data from RX — leave R5 dark

    float current_m = (d < 100) ? d / 10.0f : (float)(d - 90);

    const float FM_BAR_REF_M = 30.0f;  // reference distance: bar is dark at this distance or beyond
    float dist_ratio = current_m / FM_BAR_REF_M;
    if (dist_ratio > 1.0f) dist_ratio = 1.0f;

    // half_width: 0=dark, 1=C4+C5, 2=C3-C6, 3=C2-C7, 4=C1-C8, 5=C0-C9 (all 10 pixels)
    uint8_t half_width = (uint8_t)((1.0f - dist_ratio) * 5.0f + 0.5f);
    if (half_width > 5) half_width = 5;

    // Set bits symmetrically outward from C4 (bit 4) and C5 (bit 5)
    for (uint8_t i = 0; i < half_width; i++)
    {
      displayBuffer[6] |= (1u << (4 - i));  // left half:  C4, C3, C2, C1, C0
      displayBuffer[6] |= (1u << (5 + i));  // right half: C5, C6, C7, C8, C9
    }
  }
  // V2.5-Evo - 2026-04-28 - Bug3 history: dead FM stub was removed because this function
  // was only called from renderRtmInfoDisplay() (rtm_tx_active path). Now that
  // renderOperationalDisplay() also calls updateR5ProximityBar() for the FM path,
  // the fm_armed branch above is reachable. Priority 10 complete.
}
