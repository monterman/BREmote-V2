// V2.5-Evo - 2026-07-20 - Rex §4.6 (H4): the ADS1115 (0x48) shares the Wire bus with the HT16K33
// display (0x70). Every ADS transaction below is now wrapped in I2C_LOCK/I2C_UNLOCK so it can never
// interleave with a display write. Wrapping the bus access ONLY — no ADC scaling/logic is changed.
// During setup() (before initTasks() creates i2cMutex) the macros no-op, which is safe: startup is
// single-threaded. No display function is ever called while i2cMutex is held here, so there is no
// lock-ordering cycle with displayMutex.
// V2.5-Evo - 2026-07-20 - Rex M1 (re-audit): closed the one missed leaf-lock site — the ADS
// startADCReading in setHallActivityEnabled() is now I2C_LOCK/I2C_UNLOCK wrapped too.
// V2.5-Evo - 2026-07-27 - false means the ADS1115 never answered at boot. Diagnostic only;
// a missing ADS reads as zero throttle, which is the safe direction.
bool g_ads_ok = false;

void startupADS()
{
  Serial.print("Starting ADS1115...");
  I2C_LOCK();
  bool ok = ads.begin(ADS1115_ADDRESS);
  if(ok) ads.setGain(GAIN_ONE);
  I2C_UNLOCK();
  // V2.5-Evo - 2026-07-27 - TX-DISPLAY-1 (same class of defect as Display.ino).
  // This used to be `while (true) delay(100);` — an infinite hang in setup() on a failed
  // peripheral, which is what left the remote completely dead on 2026-07-27 (the display
  // hit its copy of this pattern first). One un-ACKed I2C device must not brick a remote
  // control: the radio, buttons and serial command handler all live downstream of here, and
  // without them the fault cannot even be diagnosed.
  //
  // The ADS1115 shares the bus with the HT16K33, so a stuck slave takes out BOTH. One bus
  // recovery + retry is attempted before giving up.
  //
  // SAFETY: the ADS reads the hall throttle. If it is genuinely absent, conversions return
  // zero, which is zero throttle — the safe direction. g_ads_ok records the state for
  // diagnostics; nothing here can command more throttle than the trigger asks for.
  if(!ok)
  {
    Serial.print(" no ACK, recovering bus...");
    i2cBusRecover();
    I2C_LOCK();
    ok = ads.begin(ADS1115_ADDRESS);
    if(ok) ads.setGain(GAIN_ONE);
    I2C_UNLOCK();
  }

  g_ads_ok = ok;

  if(!ok)
  {
    Serial.println(" FAILED — continuing anyway (old firmware hung here forever).");
    Serial.println("  >> Throttle will read ZERO until the ADS1115 answers. Use ?i2c.");
    return;
  }
  Serial.println(" Done");
}

void setHallActivityEnabled(bool enabled)
{
  hall_activity_enabled = enabled;

  if(enabled)
  {
    filter_count = 0;
    bat_filter_count = 0;
    last_channel = 0;
    // V2.5-Evo - 2026-07-20 - Rex M1 (re-audit): this ADS1115 startADCReading is a shared-Wire-bus
    // access that was NOT under i2cMutex. It is runtime-reachable on the loop task (prio 1) via
    // ?hall on (System.ino:216), the wake path (System.ino:224) and the unlock gesture (Hall.ino:726),
    // any of which the ADC task measBufCalc (prio 6) can preempt mid-transaction while it holds the bus
    // for its own ADS access. Wrap in I2C_LOCK/I2C_UNLOCK like every other ADS + HT16K33 site so the two
    // bus users can never interleave. Bus access ONLY — no ADC scaling/mux/logic change. Not re-entrant:
    // no caller of setHallActivityEnabled() holds i2cMutex, so this is a single leaf-lock take, matching
    // the blessed ordering (no displayMutex requested inside) — no new lock cycle.
    I2C_LOCK();
    ads.startADCReading(MUX_BY_CHANNEL[last_channel],false);
    I2C_UNLOCK();
    return;
  }

  thr_scaled = 0;
  tog_scaled = 127;
  steer_scaled = 127;
  tog_input = 0;
}

bool isHallActivityEnabled()
{
  return hall_activity_enabled;
}

void measBufCalc(void *parameter) 
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  while (1) 
  {
    if(isHallActivityEnabled())
    {
      measureAndBuffer();
      calcFilter();
    }
    else
    {
      thr_scaled = 0;
      tog_scaled = 127;
      steer_scaled = 127;
      tog_input = 0;
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
//3ms
// V2.5-Evo - 2026-07-20 - Rex §4.6: entire body wrapped in I2C_LOCK/I2C_UNLOCK — this is the ADS1115
// half of the shared-bus hardening. ADC scaling/logic is untouched; only the bus access is serialized.
void measureAndBuffer()
{
  I2C_LOCK();
  if (ads.conversionComplete())
  {
    if(last_channel == 0)
    {
      //Serial.print("Read Ch0, pos ");
      //Serial.println(filter_count);
      thr_raw[filter_count] = ads.getLastConversionResults();
      last_channel ++;
    }
    else if(last_channel == 1)
    {
      //Serial.print("Read Ch1, pos ");
      //Serial.println(filter_count);
      tog_raw[filter_count] = ads.getLastConversionResults();
      filter_count++;
      last_channel = 0;
      if(filter_count >= BUFFSZ)
      {
        filter_count = 0;
        last_channel = 3;
      }
    }
    else if(last_channel == 3)
    {
      //Serial.print("Read Ch3, pos ");
      //Serial.println(bat_filter_count);
      if(!mot_active) intbat_raw[bat_filter_count] = ads.getLastConversionResults();
      last_channel = 0;
      bat_filter_count++;
      if(bat_filter_count >= BUFFSZ)
      {
        bat_filter_count = 0;
      }
    }
    else
    {
      last_channel = 0;
    }
  }
  ads.startADCReading(MUX_BY_CHANNEL[last_channel],false);
  I2C_UNLOCK();
}
