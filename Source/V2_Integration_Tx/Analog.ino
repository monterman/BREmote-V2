// V2.5-Evo - 2026-07-20 - Rex §4.6 (H4): the ADS1115 (0x48) shares the Wire bus with the HT16K33
// display (0x70). Every ADS transaction below is now wrapped in I2C_LOCK/I2C_UNLOCK so it can never
// interleave with a display write. Wrapping the bus access ONLY — no ADC scaling/logic is changed.
// During setup() (before initTasks() creates i2cMutex) the macros no-op, which is safe: startup is
// single-threaded. No display function is ever called while i2cMutex is held here, so there is no
// lock-ordering cycle with displayMutex.
// V2.5-Evo - 2026-07-20 - Rex M1 (re-audit): closed the one missed leaf-lock site — the ADS
// startADCReading in setHallActivityEnabled() is now I2C_LOCK/I2C_UNLOCK wrapped too.
void startupADS()
{
  Serial.print("Starting ADS1115...");
  I2C_LOCK();
  bool ok = ads.begin(ADS1115_ADDRESS);
  if(ok) ads.setGain(GAIN_ONE);
  I2C_UNLOCK();
  if(!ok)
  {
    Serial.println(" Failed");
    while (true) delay(100);
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
