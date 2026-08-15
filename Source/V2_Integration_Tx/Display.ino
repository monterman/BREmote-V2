// V2.5-Evo - 2026-07-25 - displayDistanceInUnits(): decimal dot = TRUE decimal always; >=100 m scrolls non-blocking "FAR" (old ×100-dot far branch deleted); metres rendered for both dist_unit settings (feet parked)
// V2.5-Evo - 2026-07-20 - Rex §4.6 (H4): every HT16K33 (0x70) Wire transaction below now takes i2cMutex (I2C_LOCK/UNLOCK) so it can't tear against the ADS1115 ADC read on the shared bus. displayMutex still guards displayBuffer; i2cMutex is the inner bus lock. No leaf-lock function calls another leaf-lock function, so there is no re-entrant deadlock.
// V2.5-Evo - 2026-07-20 - GPS dot: solid only on FM-grade fix (adds HDOP + speed-valid to match the publish gate); solid branch now calls shared txGpsGoodFix() (GPS.ino).
// V2.5-Evo - 2026-07-20 - BT dot: SOLID when a BLE device is actually connected (e.g. Waveshare); blink (SLOW/FAST) still means advertising-only. Display-layer override in updateBargraphs(), guarded by BLE_ENABLED; does not touch bt_dot_state.
// V2.5-Evo - 2026-05-14 - SW55: bootAnimation VI display 500→250ms; battery voltage display 500→1450ms; total boot to padlock ~4.5s
// V2.5-Evo - 2026-05-14 - SW53: unlockAnimation() rewritten 3-frame sweep R0→R6 (was 5 frames starting at invisible row); bootAnimation battery delay 3s→500ms
// V2.5-Evo - 2026-05-13 - SW48: advanceArrow/unlockAnimation/displayError mutex-wrapped; cycleDisplayMode/displayLock call sites fixed in Hall+RTMState+System
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
// V2.5-Evo - 2026-05-02 - displayMutex applied to updateBargraphs (bargraph task) and the main loop render path

extern volatile bool fm_armed;  // defined in RTMState.ino — volatile: written by loop(),
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
  uint8_t buffer[17];
  for (uint8_t i = 0; i < 17; i++) {
    buffer[i] = 0x00;
  }
  I2C_LOCK();
  Wire.beginTransmission(DISPLAY_ADDRESS);
  Wire.write(buffer,17);
  Wire.endTransmission();
  I2C_UNLOCK();
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
  I2C_LOCK();
  Wire.beginTransmission(DISPLAY_ADDRESS);
  Wire.write(0x80); // display off
  Wire.endTransmission();
  I2C_UNLOCK();
  display_activity_enabled = false;
}

bool isDisplayActivityEnabled()
{
  return display_activity_enabled;
}

// V2.5-Evo - 2026-07-27 - TX-DISPLAY-1. False means the HT16K33 never answered at boot.
// The remote now stays fully alive in that state instead of hanging in setup().
bool g_display_ok = false;

// ============================================================
// i2cBusRecover - free a slave that is holding SDA low
//
// An I2C slave interrupted mid-byte (power glitch, brownout, reset during a transfer) can
// keep SDA pulled LOW indefinitely, waiting to finish clocking out the byte it was sending.
// It survives an MCU reset because the SLAVE is the one stuck, not the master. The bus then
// looks permanently dead and no device will ACK.
//
// The cure is the standard one from the I2C spec / NXP AN10216: take the pins away from the
// peripheral, manually pulse SCL up to 9 times (one full byte + ACK) so the slave finishes
// its transfer and releases SDA, then generate a STOP so every device returns to idle.
//
// P_I2C_SDA is GPIO 2 — an ESP32-C3 strapping pin — so it is left as INPUT afterwards and
// the Wire driver is re-initialised with exactly the parameters Init.ino:17-18 uses
// (400 kHz), so bus speed is unchanged.
// ============================================================
// ============================================================
// i2cLineDiag - what is physically on SDA/SCL, before Wire ever runs
//
// V2.5-Evo - 2026-07-28. Five consecutive boots on 2026-07-27 showed neither the HT16K33
// (0x70) nor the ADS1115 (0x48) answering, with six bus-recovery attempts each time. That
// proves the bus is dead but NOT WHY, and the two causes need different repairs — which
// matters because this unit is heavily potted and cannot be inspected.
//
// HOW IT TELLS THEM APART: I2C idles high only because external pull-up resistors tie both
// lines to the peripherals' supply rail. So probe each line twice, before Wire.begin():
//
//   internal PULL-UP reads LOW   -> something is actively HOLDING the line down.
//                                   Short to ground, a latched-up device, or a dead pin.
//   internal PULL-DOWN reads HIGH-> an external pull-up is winning against the ~45k internal
//                                   pull-down. The resistors ARE powered: rail and wiring
//                                   are good, so the CHIPS are the problem.
//   pull-up HIGH + pull-down LOW -> nothing external is driving the line at all. FLOATING:
//                                   the pull-ups have lost their supply, or the line is open.
//
// Run BEFORE Wire.begin() so the pins are still plain GPIO and nothing we do can mask the
// real state. Cost is ~1 ms and it runs every boot.
// ============================================================
void i2cLineDiag()
{
  Serial.println("I2C line state (probed before Wire init):");

  const uint8_t pins[2]  = { (uint8_t)P_I2C_SDA, (uint8_t)P_I2C_SCL };
  const char   *names[2] = { "SDA (GPIO2)", "SCL (GPIO1)" };

  for (uint8_t i = 0; i < 2; i++)
  {
    pinMode(pins[i], INPUT_PULLUP);
    delayMicroseconds(500);
    bool up = digitalRead(pins[i]);

    pinMode(pins[i], INPUT_PULLDOWN);
    delayMicroseconds(500);
    bool dn = digitalRead(pins[i]);

    pinMode(pins[i], INPUT);   // leave floating for Wire.begin()

    const char *verdict;
    if (!up)     verdict = "HELD LOW  -> short to GND / latched device / damaged pin";
    else if (dn) verdict = "EXTERNAL PULL-UP PRESENT  -> rail + wiring OK, chips are at fault";
    else         verdict = "FLOATING  -> pull-ups unpowered or line OPEN (supply or broken trace)";

    Serial.printf("  %-12s  pullup=%d  pulldown=%d   %s\n",
                  names[i], (int)up, (int)dn, verdict);
  }

  Serial.println("  (both lines EXTERNAL PULL-UP = bus healthy, chips dead/unpowered)");
  Serial.println("  (both lines FLOATING        = the 3V3 rail feeding 0x70+0x48 is gone)");
}

// NOT static: Analog.ino calls this too, for the ADS1115 half of the same shared bus.
void i2cBusRecover()
{
  Wire.end();

  pinMode(P_I2C_SCL, OUTPUT);
  pinMode(P_I2C_SDA, INPUT_PULLUP);
  digitalWrite(P_I2C_SCL, HIGH);
  delayMicroseconds(10);

  // Up to 9 clocks, stopping as soon as the slave lets SDA go high.
  for (uint8_t i = 0; i < 9 && digitalRead(P_I2C_SDA) == LOW; i++) {
    digitalWrite(P_I2C_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(P_I2C_SCL, HIGH);
    delayMicroseconds(10);
  }

  // STOP condition: SDA rises while SCL is high.
  pinMode(P_I2C_SDA, OUTPUT);
  digitalWrite(P_I2C_SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(P_I2C_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(P_I2C_SDA, HIGH);
  delayMicroseconds(10);

  pinMode(P_I2C_SDA, INPUT);
  pinMode(P_I2C_SCL, INPUT);

  Wire.begin(P_I2C_SDA, P_I2C_SCL);
  Wire.setClock(400000);
}

// Print every address that ACKs. Called when the display never answers, so a dark remote
// reports WHAT is on the bus rather than just dying. 0x70 = HT16K33, 0x48 = ADS1115.
void txI2cScan()
{
  Serial.println("  I2C scan:");
  uint8_t n = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("    0x%02X ACK%s\n", addr,
                    addr == 0x70 ? "  <- HT16K33 display" :
                    addr == 0x48 ? "  <- ADS1115"         : "");
      n++;
    }
  }
  if (!n) Serial.println("    NOTHING responded — bus is dead or held low, not a chip fault.");
}

void startupDisplay()
{
  Serial.print("Starting Display...");
  // HT16K33 tPOR ≤ 1ms but PCB power-rail settle can take longer.
  // Retry for up to 100ms before declaring failure — prevents boot hang
  // on first power cycle when the display chip isn't ready yet.
  // ============================================================
  // V2.5-Evo - 2026-07-27 - TX-DISPLAY-1: BUS RECOVERY + NEVER BRICK ON A DARK SCREEN.
  //
  // FIELD FAILURE 2026-07-27: the TX died at the dock and would not boot afterwards.
  // Serial showed a CLEAN power-on (rst:0x1 POWERON, SW 27) reaching exactly:
  //     Starting Display... Failed
  // and then nothing. The radio, buttons, throttle and serial command handler never
  // started, because the old code ended in `while(1) delay(100)`. The battery was 4.0 V —
  // never the problem. Owner reported the screen DID come on a few times at home and then
  // stopped, which rules out a dead chip: this is intermittent, so it is timing or bus state.
  //
  // TWO DEFECTS, BOTH FIXED HERE:
  //
  // 1) THE RETRY WINDOW WAS FAR TOO SHORT. 20 tries x 5 ms = 100 ms total. The comment
  //    above openly admits "PCB power-rail settle can take longer" and then allows 100 ms
  //    anyway. Coming off the charger the rail ramps differently than on a cold battery
  //    boot, so the HT16K33 sometimes is not ready inside that window. Now ~3 s.
  //
  // 2) A STUCK BUS COULD NEVER CLEAR ITSELF. If power is interrupted mid-transaction — which
  //    is exactly what "it died while I was using it" means — an I2C slave can be left
  //    holding SDA LOW, and it keeps holding it ACROSS A RESET. Every subsequent probe then
  //    fails forever and no amount of replugging helps, which matches what the owner saw.
  //    The standard cure is to clock the stuck slave out by hand: pulse SCL up to 9 times
  //    with SDA released, then issue a STOP. That is i2cBusRecover() below, and it now runs
  //    between retry rounds.
  //
  // 3) AND THE REAL BUG: `while(1)` on a failed peripheral. A remote control must not brick
  //    itself because a display driver did not ACK. It now continues, records the failure in
  //    g_display_ok, and prints an I2C bus scan so the fault is DIAGNOSABLE over serial
  //    instead of presenting as a dead remote. Every display write already goes through
  //    beginDisplay()/I2C_LOCK paths that tolerate a missing device.
  //
  // NOTE: P_I2C_SDA is GPIO 2, an ESP32-C3 strapping pin. A slave holding it low at reset is
  // therefore doubly bad. Recovery releases it as INPUT before Wire is restarted.
  // ============================================================
  bool found = false;
  for (uint8_t round = 0; round < 6 && !found; round++)
  {
    for (int i = 0; i < 20 && !found; i++) {
      found = beginDisplay();
      if (!found) delay(25);
    }
    if (!found) {
      Serial.printf(" [try %u: no ACK at 0x%02X, recovering bus]", round + 1, DISPLAY_ADDRESS);
      i2cBusRecover();
    }
  }

  g_display_ok = found;

  if (!found) {
    Serial.println(" FAILED after ~3s + bus recovery.");
    Serial.println("  >> Continuing WITHOUT display (old firmware hung here forever).");
    Serial.println("  >> Radio, buttons and serial commands are UP. Use ?i2c to inspect the bus.");
    txI2cScan();
    return;
  }

  clearDisplayBuffer();
  clearDisplay();
  initDisplay();
  Serial.println(" Done");
}

bool beginDisplay()
{
  I2C_LOCK();
  Wire.beginTransmission(DISPLAY_ADDRESS);
  bool ok = (0 == Wire.endTransmission());
  I2C_UNLOCK();
  return ok;
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

  // Both init transactions under one i2cMutex hold. setBrightness() takes the mutex itself, so it is
  // called AFTER I2C_UNLOCK() to avoid re-entering the non-recursive mutex.
  I2C_LOCK();
  Wire.beginTransmission(DISPLAY_ADDRESS);
  //System Oscillator on
  Wire.write(0x21);
  Wire.endTransmission();

  Wire.beginTransmission(DISPLAY_ADDRESS);
  //On, no blinking
  Wire.write(0x81);
  Wire.endTransmission();
  I2C_UNLOCK();

  setBrightness(0x0F);
}

void setBrightness(uint8_t level)
{
  if(!isDisplayActivityEnabled()) return;

  //Set brightness x00..x0F
  if(level > 0x0F) level = 0x0F;
  I2C_LOCK();
  Wire.beginTransmission(DISPLAY_ADDRESS);
  //Full brightness
  Wire.write(0xE0 | level);
  Wire.endTransmission();
  I2C_UNLOCK();
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

  I2C_LOCK();
  Wire.beginTransmission(DISPLAY_ADDRESS);
  Wire.write(sendBuffer,17);
  Wire.endTransmission();
  I2C_UNLOCK();
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
  xSemaphoreTake(displayMutex, portMAX_DELAY);  // loop-task render — waits for the bargraph task to release
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
            // V2.5-Evo - 2026-07-25 - Dot raised R4 -> R3, in step with the distance readout so the
            // decimal point sits at the same height everywhere. C3 is the never-written separator column.
            displayBuffer[4] |= (1u << 3);                         // decimal dot C3 R3; auto-cleared by next displayDigits() call
          }
          break;
        case DISPLAY_MODE_BAT:    displayShowTwoDigitOrDash(getEffectiveFoilBat()); break;
        case DISPLAY_MODE_THR:    displayShowTwoDigitOrDash(thr_scaled * 99 / 255); break;
        case DISPLAY_MODE_AMP:    displayShowTwoDigitOrDash(min(telemetry.foil_motor_amps, (uint8_t)99)); break;
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
    // ET is absent from V2.5-Evo RX source; this guard is defensive for legacy or future paths.
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
  DISP_LOCK();
  displayDigits(LET_E, min(err, 33));  // clamp to 33 — num0[] has 34 entries (indices 0–33); was 29, silently wrong for err 30–33
  updateDisplay();
  DISP_UNLOCK();
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
  delay(250);   // SW55: trimmed — quick version flash before voltage

  uint8_t temp_volt = uint8_t(int_bat_volt*10);

  displayDigits(temp_volt/10,temp_volt-10*(temp_volt/10));
  updateDisplay();
  delay(1450);  // SW55: extended — user reads voltage before padlock; ~4.5s total boot time
}

uint8_t arrowPos = 0;
void advanceArrow()
{
  DISP_LOCK();
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
  DISP_UNLOCK();
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
// V2.5-Evo - 2026-05-13 - SW48: holds displayMutex for entire animation (300ms) — the bargraph task skips one cycle, which is fine
#define ANIMATION_DELAY 60
// Helper: clear digit zone preserving C7 GPS dot and C8/C9 bargraphs (bit 7 = C7)
#define ANIM_CLEAR() for(int _i = 0; _i < 7; _i++) displayBuffer[_i] &= 0xFF80

// SW53: paintbrush sweep. Arrow descends R0→R6 using |= without clearing between frames —
// each row the arrow passes through stays lit. Same pattern as advanceArrow().
// 3 frames × 60ms = 180ms. Final state: R0-R3 = 0x3E (wide), R4-R5 = 0x1C (mid), R6 = 0x08 (tip).
void unlockAnimation()
{
  DISP_LOCK();
  ANIM_CLEAR();                // clear once — frames paint on top without erasing
  displayBuffer[7] &= 0xFF80; // clear battery row cols C0-C6 so tip pixel has clean base

  // Frame 1 — arrow head at R0, tip at R4
  displayBuffer[1] |= 0x3E;
  displayBuffer[2] |= 0x3E;
  displayBuffer[3] |= 0x1C;
  displayBuffer[4] |= 0x1C;
  displayBuffer[5] |= 0x08;
  updateDisplay();
  delay(ANIMATION_DELAY);

  // Frame 2 — arrow moves to R1; R0 stays lit
  displayBuffer[2] |= 0x3E;
  displayBuffer[3] |= 0x3E;
  displayBuffer[4] |= 0x1C;
  displayBuffer[5] |= 0x1C;
  displayBuffer[6] |= 0x08;
  updateDisplay();
  delay(ANIMATION_DELAY);

  // Frame 3 — arrow tip reaches R6; all rows R0-R6 painted at full head width (0x3E)
  // R4/R5 widened to 0x3E (were 0x1C — missing C1 and C5).
  // R6 explicitly painted to 0x3E so rectangle is complete even when VESC battery bar is off.
  displayBuffer[3] |= 0x3E;
  displayBuffer[4] |= 0x3E;
  displayBuffer[5] |= 0x3E;
  displayBuffer[6] |= 0x3E;
  displayBuffer[7] |= 0x3E;
  updateDisplay();
  delay(ANIMATION_DELAY);

  arrowPos = 0;
  DISP_UNLOCK();
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
    // Take mutex before writing displayBuffer. Timeout 50ms — if the loop task is mid-render,
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
    } else if (txGpsGoodFix()) {
      // V2.5-Evo - 2026-07-20 - GPS dot: solid only on FM-grade fix (adds HDOP + speed-valid to match the publish gate)
      // canonical gate: GPS.ino getTxGPSLoop() / txGpsGoodFix()
      // Good FM-grade fix — solid on; reset timer so blink starts cleanly on state change
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
#ifdef BLE_ENABLED
    // V2.5-Evo - 2026-07-20 - BT dot: SOLID when a BLE device is actually connected (e.g. Waveshare); blink = advertising-only.
    if (bt_dot_state != BT_DOT_OFF && bleIsConnected()) bt_dot_on = true;
#endif
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
// V2.5-Evo - 2026-07-25 - FM/RTM distance readout: dot = TRUE decimal always; far case scrolls "FAR".
// BUG: the old metric branch used the C3 R4 decimal dot to mean "×100 m" at >=100 m
// (e.g. "1.7·" = 170 m). Riders read the dot as a real decimal, so 170 m looked like
// 1.7 m in the field and burned the user in FM. FIX: the dot now ALWAYS means a true
// decimal, and the far case (>=100 m) is shown as a scrolling word "FAR" instead of an
// ambiguous dotted number. The old ×100 dist_100m branch is deleted entirely.
//
// scrollFarStep() is a NON-BLOCKING version of the boot scroll (scroll4Digits): the boot
// scroll uses a blocking delay() per frame, which is unusable here because this renders on
// every loop tick during active FM/RTM — a blocking delay would stall throttle/telemetry on
// the single-core TX. Instead a static column offset is advanced by a millis() timer at the
// same ~100 ms cadence the boot scroll uses (del=100), rendering exactly one frame per call.
// It writes ONLY the digit zone rows R0-R4 (displayBuffer[1..5], cols C0-C6); it never touches
// the R5 proximity bar (displayBuffer[6]), R6 battery (displayBuffer[7]), C7 GPS dot or C8/C9
// bargraphs (bits 7-9 preserved by the 0xFF80 mask), so the FM/RTM proximity bar stays visible.
//
// Inputs: dist_m in metres (float). Metres are rendered for BOTH unit settings this version.
// Rules (metres):
//   <1 m     → "00"
//   0-9.9 m  → "X.X" with the true-decimal dot at C3 R3   (1.7 = 1.7 m)
//   10-99 m  → "XX" whole metres, no dot
//   >=100 m  → scrolling "FAR"
// displayDigits() must be called before setting the decimal dot (it clears R0-R5, R3 included).
// ============================================================
static uint8_t       far_scroll_pos = 0;   // current left-edge column offset into the FAR buffer
static unsigned long far_scroll_ms  = 0;   // millis() of last scroll step (0 = freshly reset)

// Reset the "FAR" scroll so it re-enters from the start next time >=100 m is shown.
// Called on every non-far metric path so leaving the far range clears the scroll state cleanly.
static inline void scrollFarReset()
{
  far_scroll_pos = 0;
  far_scroll_ms  = 0;
}

// Render exactly ONE non-blocking frame of the scrolling word "FAR" into the digit zone.
// Uses the large num0[] glyphs (same font / visual style as the boot scroll). Advances one
// column per ~100 ms via a millis() timer — no delay(), so loop() keeps servicing throttle
// and telemetry on the single-core TX.
static void scrollFarStep()
{
  // "FAR" column buffer: F(3) + gap(1) + A(3) + gap(1) + R(3) + trailing gap(4) = 15 cols.
  // The trailing gap separates repeats so it reads as a repeating scrolling word, not "FARFAR".
  const uint8_t FAR_LEN = 15;
  uint8_t buf[FAR_LEN];
  uint8_t p = 0;
  buf[p++] = num0[LET_F][0]; buf[p++] = num0[LET_F][1]; buf[p++] = num0[LET_F][2];
  buf[p++] = 0x00;
  buf[p++] = num0[LET_A][0]; buf[p++] = num0[LET_A][1]; buf[p++] = num0[LET_A][2];
  buf[p++] = 0x00;
  buf[p++] = num0[LET_R][0]; buf[p++] = num0[LET_R][1]; buf[p++] = num0[LET_R][2];
  buf[p++] = 0x00; buf[p++] = 0x00; buf[p++] = 0x00; buf[p++] = 0x00;

  // Non-blocking step: first frame after a reset shows offset 0; then +1 column per 100 ms.
  unsigned long now = millis();
  if (far_scroll_ms == 0)
    far_scroll_ms = now;
  else if (now - far_scroll_ms >= 100UL)
  {
    far_scroll_ms  = now;
    far_scroll_pos = (uint8_t)((far_scroll_pos + 1) % FAR_LEN);
  }

  // Clear digit zone rows R0-R4 (displayBuffer[1..5]), cols C0-C6 only. 0xFF80 preserves
  // C7 (GPS dot) + C8/C9 (bargraphs). R5 (displayBuffer[6]) and R6 (displayBuffer[7]) are
  // deliberately left untouched here — the caller's updateR5ProximityBar() rebuilds R5 after.
  for (int i = 1; i <= 5; i++) displayBuffer[i] &= 0xFF80;

  // Render the 7 visible columns C0-C6 at the current offset. Bit math matches scroll4Digits:
  // displayBuffer[j] is display row R(j-1); num0 bit (5-j) is that row's pixel for column i.
  // (j runs 5..1 only; j=0 would target the unused displayBuffer[0] with num0 bit 5, always 0.)
  for (int j = 5; j >= 1; j--)
    for (int i = 0; i < 7; i++)
      displayBuffer[j] |= ((buf[(i + far_scroll_pos) % FAR_LEN] >> (5 - j)) & 0x01) << i;
}

static void displayDistanceInUnits(float dist_m)
{
  // FEET NOT IMPLEMENTED on TX display this version — renders metres. Parked: full feet/yards
  // far-range solution. usrConf.dist_unit is retained in confStruct/SPIFFS (no schema change,
  // no config wipe); a future version can branch here on dist_unit==1. Metres are rendered for
  // both settings for now.
  if (dist_m < 1.0f)
  {
    scrollFarReset();
    displayDigits(0, 0);
  }
  else if (dist_m < 100.0f)
  {
    scrollFarReset();
    if (dist_m < 10.0f)
    {
      // 0-9.9 m: "X.X" with the C3 R3 dot meaning a TRUE decimal (1.7 = 1.7 m). Same dot
      // mechanism the kW readout uses. Work in integer tenths to avoid float-rounding glitches.
      uint16_t tenths = (uint16_t)(dist_m * 10.0f + 0.5f);  // 10..99 for 1.0-9.9 m
      if (tenths > 99) tenths = 99;                         // guard 9.95-9.99 rounding to 10.0
      displayDigits(tenths / 10, tenths % 10);
      // V2.5-Evo - 2026-07-25 - Decimal dot raised R4 -> R3 (owner: one row up off the bottom of the
      // digit zone, where it was hard to read). Row naming is this file's existing R0-R6, R0 = top:
      // R3 is the 4th row down. C3 is the separator column between the two digits and is NEVER written
      // by displayDigits() (digitBuffer[3] is left zero while [0..2] and [4..6] carry the glyphs), so
      // the dot cannot collide with a digit at any row. displayDigits() clears displayBuffer[1..6], so
      // R3 is blanked before the dot is set, exactly as R4 was. Keep in step with the kW readout dot so
      // the decimal point sits at the same height everywhere on the display.
      displayBuffer[4] |= (1u << 3);                        // true decimal dot at C3 R3
    }
    else
    {
      // 10-99 m: whole metres, no dot.
      uint8_t m = (uint8_t)(dist_m + 0.5f);
      if (m > 99) m = 99;
      displayDigits(m / 10, m % 10);
    }
  }
  else
  {
    // >=100 m: scrolling "FAR" (non-blocking). The old ×100-with-dot branch is deleted — the
    // dot now always means a true decimal, so a dotted far distance would misread as ones of m.
    scrollFarStep();
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
  xSemaphoreTake(displayMutex, portMAX_DELAY);  // loop-task render — waits for the bargraph task to release
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
// V2.5-Evo - 2026-07-20 - Batch T (Fable FM v1.4): FM path is now STATE-DRIVEN from
//   telemetry.fm_flags + TX-local state. Suppressed during showFullScreenMessage() (buffer
//   cleared, not called during blocking messages). All R5 writes stay inside displayBuffer[6]
//   under the caller's displayMutex (renderOperationalDisplay / renderRtmInfoDisplay hold it),
//   so no tearing; no blocking, no delays — pure presentation.
//
// RTM (unchanged): blinks 1000/500; square-root curve, GROW-WITH-FAR (full = at arm distance,
//   shrinks from the right as the buggy closes). rtm_arm_dist_m is the 100% reference.
//
// FM R5 states:
//   Disarmed (!fm_armed)                 → R5 fully OFF (absence = "not armed").
//   ARMED-READY  (armed, !engaged, ready)→ 3-px Knight-Rider scanner SWEEPS C0→C9→C0, one step
//                                           per ~200ms (matches the bargraph tick cadence).
//   ARMED-NOT-READY (armed, !engaged,    → same 3-px segment BLINKS IN PLACE (centered), no sweep
//     any not-ready per fmArmedNotReady())  — "armed, waiting on GPS/link"; flips to sweep live.
//   ENGAGED (fm_flags bit1, link fresh)  → static distance bar, GROW-WITH-FAR (same direction as
//                                           RTM for consistency), center-expanding, SPIFFS-scaled
//                                           full-scale from usrConf.fm_warn_distance_m (existing
//                                           field — no new confStruct field).
// ============================================================
void updateR5ProximityBar()
{
  static unsigned long r5_blink_ms    = 0;
  static bool          r5_blink_state = false;
  // Scanner sweep state (ARMED-READY): 3-px segment ping-pongs across C0..C9, one step per ~200ms.
  static unsigned long r5_scan_ms  = 0;
  static int8_t        r5_scan_pos = 0;   // left column of the 3-px segment; bounds 0..COLS-3
  static int8_t        r5_scan_dir = 1;   // +1 sweeping right, -1 sweeping left

  // Blink: 1000 ms on, 500 ms off — used by the RTM bar and the FM armed-not-ready blink-in-place.
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

  // ---- RTM proximity bar (unchanged): blinks, GROW-WITH-FAR (full = far) ----
  if (rtm_tx_active)
  {
    if (!r5_blink_state) return;  // off phase — leave R5 dark

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
    uint8_t pixels = (uint8_t)(sqrtf(ratio) * 10.0f + 0.5f);  // full at arm distance, shrinks as it closes
    if (pixels > 10) pixels = 10;

    for (uint8_t c = 0; c < pixels; c++)
      displayBuffer[6] |= (1u << c);
    return;
  }

  // ---- FM R5 row (Batch T): state-driven from fm_flags + TX-local readiness ----
  if (!fm_armed) return;  // Disarmed → R5 fully OFF

  uint8_t f = telemetry.fm_flags;
  bool link_recent = (last_packet != 0 && (now - last_packet) < FM_LINK_HEALTHY_MS);

  // ENGAGED → static distance bar, GROW-WITH-FAR, SPIFFS-scaled, center-expanding.
  // Reuses the RX→TX distance byte (telemetry.rtm_distance). Full-scale = fm_warn_distance_m so
  // the bar fills as the buggy falls behind and is full at the proximity-warn threshold. Same
  // GROW-WITH-FAR direction as the RTM bar above (one physical row, one meaning across modes).
  if (link_recent && (f & FM_FLAG_ENGAGED))
  {
    uint8_t d = telemetry.rtm_distance;
    if (d == 0xFF) return;  // no distance data — leave R5 dark

    float current_m = (d < 100) ? d / 10.0f : (float)(d - 90);
    float ref_m = (float)usrConf.fm_warn_distance_m;   // dynamic SPIFFS full-scale reference
    if (ref_m < 1.0f) ref_m = 30.0f;                   // guard against a zero/invalid config
    float dist_ratio = current_m / ref_m;
    if (dist_ratio > 1.0f) dist_ratio = 1.0f;

    // GROW-WITH-FAR: near → just the centre pair (C4+C5); far → full 10-px bar. Never fully dark
    // while engaged (half_width 1..5) so ENGAGED always reads distinct from the OFF/scanner states.
    uint8_t half_width = 1 + (uint8_t)(dist_ratio * 4.0f + 0.5f);
    if (half_width > 5) half_width = 5;
    for (uint8_t i = 0; i < half_width; i++)
    {
      displayBuffer[6] |= (1u << (4 - i));  // left  half: C4, C3, C2, C1, C0
      displayBuffer[6] |= (1u << (5 + i));  // right half: C5, C6, C7, C8, C9
    }
    return;
  }

  // ARMED (not engaged) → 3-px Knight-Rider scanner. READY = sweep, NOT-READY = blink in place.
  const uint8_t SCAN_COLS = 10;
  const uint8_t SCAN_SEG  = 3;   // 3-px segment (A3 correction), position bounds 0..COLS-3
  if (fmArmedNotReady())
  {
    // NOT-READY → segment blinks in place (centred), no sweep: "armed, waiting on GPS/link".
    if (!r5_blink_state) return;                         // off phase — dark
    const uint8_t center = (SCAN_COLS - SCAN_SEG) / 2;   // = 3 → C3, C4, C5
    for (uint8_t i = 0; i < SCAN_SEG; i++) displayBuffer[6] |= (1u << (center + i));
    return;
  }

  // READY → sweep the 3-px segment one step per ~200ms, ping-pong C0→C9→C0.
  if (now - r5_scan_ms >= 200UL)
  {
    r5_scan_ms = now;
    r5_scan_pos += r5_scan_dir;
    if (r5_scan_pos >= (int8_t)(SCAN_COLS - SCAN_SEG)) { r5_scan_pos = SCAN_COLS - SCAN_SEG; r5_scan_dir = -1; }
    else if (r5_scan_pos <= 0)                          { r5_scan_pos = 0;                    r5_scan_dir =  1; }
  }
  for (uint8_t i = 0; i < SCAN_SEG; i++) displayBuffer[6] |= (1u << (r5_scan_pos + i));
}
