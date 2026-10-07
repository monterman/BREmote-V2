// V2.5-Evo - 2026-10-07 - C-1: frozen-throttle fix. Every finished ADS1115 conversion stamps last_ads_ok_ms; a
//   raw throttle sample far outside the calibrated band is rejected and flagged; measBufCalc() runs
//   adsInputFaultUpdate() after calcFilter(), which forces throttle 0 / steering centre / toggle blocked, raises
//   error 72 and the stop buzz, and only releases the fault after a fully refreshed buffer shows the trigger
//   released. adsInputFaultNow() lets sendData() apply the same deadline. No confStruct change.
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
  // V2.5-Evo - 2026-10-07 - C-1 CORRECTION: the "reads zero" claim above was not true. A missing ADS never
  // finishes a conversion, so the buffer stays at its boot zeros, and with this remote's inverted calibration
  // (idle > pull) calcFilter() clamps 0 counts to FULL throttle. What makes it safe now is the conversion
  // deadline in adsInputFaultUpdate(): no conversion for ADS_STALE_MS -> throttle 0 and error 72.
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
  // V2.5-Evo - 2026-10-07 - C-1: restart the conversion deadline when sampling is switched on, so the time
  // spent with sampling off is not mistaken for a dead ADC. Stamped BEFORE the enable is written, because
  // the ADC task (prio 6) can preempt this loop-task call between the two lines.
  if (enabled) last_ads_ok_ms = millis();
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

// ============================================================
// THROTTLE INPUT FAULT DETECTION - V2.5-Evo - 2026-10-07 - C-1
// See the C-1 block in BREmote_V2_Tx.h for the bug this closes.
// ============================================================

// vib_stop_pending is defined in System.ino, which the build concatenates AFTER this file.
extern volatile bool vib_stop_pending;

// A trigger reading at or below this (0-255 scale) counts as "released" for fault recovery. Same test the
// unlock path uses (Hall.ino waits while thr_scaled > 5), so "released" means fully let go, not feathered.
static const uint8_t kAdsReleasedMax = 5;

// adsInputStale - has the ADS1115 gone quiet for longer than ADS_STALE_MS while sampling is on?
// Inputs: last_ads_ok_ms, hall_activity_enabled, millis(). Output: true = no finished conversion in time.
// No side effects. Callable from any task. The stamp is read BEFORE millis() so a stamp written by the
// higher-priority ADC task in between can only make the age smaller, never wrap it negative.
static bool adsInputStale()
{
  if (!isHallActivityEnabled()) return false;
  unsigned long stamp = last_ads_ok_ms;
  return (millis() - stamp) > ADS_STALE_MS;
}

// adsInputFaultNow - should the throttle byte about to be sent be forced to 0?
// Called by sendData() (Radio.ino) for every control packet. It applies the same deadline as the ADC task,
// because when the bus is stuck the ADC task can sit inside one slow I2C transaction (the Wire timeout is
// 50 ms per transaction) and cannot zero thr_scaled itself. A missed deadline seen here LATCHES the fault;
// the ADC task announces it and owns the recovery.
// Inputs: ads_input_fault, adsInputStale(). Output: true = send zero throttle and centred steering.
// Side effects: may set ads_input_fault. Never blocks.
bool adsInputFaultNow()
{
  if (ads_input_fault) return true;
  if (adsInputStale())
  {
    ads_input_fault = true;
    return true;
  }
  return false;
}

// adsThrReadingPlausible - is this raw throttle sample inside the calibrated band plus a margin?
// A healthy Hall trigger reads between thr_idle and thr_pull, a little beyond each end because calibration
// pulls both ends in by cal_offset. A failed I2C read in the ADS library leaves its buffer half-stale and
// returns roughly 0..255; with this remote's inverted calibration (idle > pull) the old clamp turned that
// into FULL throttle. Such a reading is now a fault, never a clamp.
// Margin: half the calibrated span, at least 1000 counts (the calibration's own minimum span), which is far
// beyond any normal over-travel or temperature drift and far inside a garbage reading.
// Not applied while calibrating (cal_ok == 0: there is no trusted band yet), during boot (in_setup: the
// throttle byte is 0 because the remote boots locked, and the boot recalibration gesture must stay usable
// when the magnet has moved), or with a degenerate band (idle == pull: calcFilter() already outputs 0).
// Input: raw - one signed conversion result. Output: true = plausible. No side effects.
static bool adsThrReadingPlausible(int16_t raw)
{
  if (!usrConf.cal_ok || in_setup) return true;
  int32_t lo = (usrConf.thr_idle < usrConf.thr_pull) ? usrConf.thr_idle : usrConf.thr_pull;
  int32_t hi = (usrConf.thr_idle < usrConf.thr_pull) ? usrConf.thr_pull : usrConf.thr_idle;
  if (lo == hi) return true;
  int32_t margin = (hi - lo) / 2;
  if (margin < 1000) margin = 1000;
  return ((int32_t)raw >= lo - margin) && ((int32_t)raw <= hi + margin);
}

// adsInputFaultUpdate - latch, announce, enforce and clear the throttle input fault.
// Called by measBufCalc() right after calcFilter(), in the same task pass, so a lower-priority task (the
// radio) can never observe the unsafe intermediate value calcFilter() wrote.
// Inputs: adsInputStale(), ads_thr_out_of_range, ads_thr_good_samples, thr_scaled as calcFilter() just
//   computed it from the buffer. Outputs: none.
// Side effects: while faulted, forces thr_scaled = 0, tog_scaled = 127, steer_scaled = 127, tog_input = 0
//   (no steering, no toggle gesture, no lock/unlock, no error dismissal). On the rising edge: stop buzz
//   (vib_stop_pending, Pattern 7) and remote_error = 72 (unless water-ingress E71 is showing), one serial
//   line. On recovery: clears remote_error 72, one serial line.
// Recovery needs ALL of: conversions arriving again, no out-of-band sample, at least BUFFSZ fresh in-band
//   throttle samples since the fault (so every slot of the averaging buffer is new), and that fresh average
//   reading released (<= kAdsReleasedMax). A trigger still held when the bus comes back keeps throttle at 0
//   until it is let go, so throttle can never jump to a held position.
static void adsInputFaultUpdate()
{
  static bool announced = false;
  const uint8_t thr_fresh = thr_scaled;           // what calcFilter() just computed from the buffer
  const bool    stale     = adsInputStale();
  const bool    bad_now   = stale || ads_thr_out_of_range;
  ads_thr_out_of_range = false;                   // consumed

  if (bad_now)
  {
    ads_input_fault      = true;
    ads_thr_good_samples = 0;                     // the buffer must be refilled after every bad event
  }
  if (!ads_input_fault)
  {
    announced = false;
    return;
  }

  if (!announced)
  {
    announced            = true;
    ads_thr_good_samples = 0;                     // covers a fault latched by sendData() while this task was stuck
    vib_stop_pending     = true;                  // Pattern 7, the normal stop buzz (the motor is a GPIO, not I2C)
    if (remote_error != 71) remote_error = REMOTE_ERR_INPUT_FAULT;
    Serial.println(stale ? "INPUT [TX] FAULT: no ADS1115 conversion for >50 ms - throttle 0, steering centred, error 72"
                         : "INPUT [TX] FAULT: throttle reading far outside calibration - throttle 0, steering centred, error 72");
  }

  if (!bad_now && ads_thr_good_samples >= BUFFSZ && thr_fresh <= kAdsReleasedMax)
  {
    ads_input_fault = false;
    announced       = false;
    if (remote_error == REMOTE_ERR_INPUT_FAULT) remote_error = 0;
    Serial.println("INPUT [TX] recovered: fresh readings with the trigger released - throttle passed again");
    return;                                       // calcFilter()'s fresh, released values stand
  }

  // Still faulted. Re-assert the error if something else cleared it (the E71 mirror clears to 0).
  if (remote_error == 0) remote_error = REMOTE_ERR_INPUT_FAULT;
  thr_scaled   = 0;
  tog_scaled   = 127;
  steer_scaled = 127;
  tog_input    = 0;
}

void measBufCalc(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
  // V2.5-Evo - 2026-10-07 - C-1: start the conversion deadline when the task starts, not at millis() 0.
  last_ads_ok_ms = millis();

  while (1)
  {
    if(isHallActivityEnabled())
    {
      measureAndBuffer();
      calcFilter();
      adsInputFaultUpdate();   // V2.5-Evo - 2026-10-07 - C-1: may override the values calcFilter() just wrote
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
  // V2.5-Evo - 2026-10-07 - C-1: what the ADS library does on a failed read (Adafruit_ADS1X15 2.5.0 with
  // Adafruit_BusIO 1.17.4, read from the source): readRegister() ignores the I2C result and returns its
  // buffer, whose first byte still holds the register pointer it just wrote. So with the chip absent or SDA
  // held low, conversionComplete() reads 0x01xx (bit 15 clear) and is ALWAYS false - the buffer freezes, it is
  // never 0xFFFF - and a getLastConversionResults() that fails after a good poll returns 0x00xx (0..255).
  // The first case is caught by the conversion deadline, the second by the plausibility check below.
  if (ads.conversionComplete())
  {
    last_ads_ok_ms = millis();   // V2.5-Evo - 2026-10-07 - C-1: a conversion finished - the ADC is alive
    if(last_channel == 0)
    {
      //Serial.print("Read Ch0, pos ");
      //Serial.println(filter_count);
      // V2.5-Evo - 2026-10-07 - C-1: an implausible throttle sample is NOT stored (it would be averaged and
      // clamped into a throttle value); it raises the input fault instead. See adsThrReadingPlausible().
      int16_t thr_now = ads.getLastConversionResults();
      if (adsThrReadingPlausible(thr_now))
      {
        thr_raw[filter_count] = thr_now;
        if (ads_thr_good_samples < 255) ads_thr_good_samples = (uint8_t)(ads_thr_good_samples + 1);
      }
      else
      {
        ads_thr_out_of_range = true;
        ads_thr_good_samples = 0;
      }
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
