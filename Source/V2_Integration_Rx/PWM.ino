// V2.5-Evo - 2026-08-26 - FM manual steering takeover: a rider deflection outside kFmManualSteerDeadband immediately wins over the FM steering override without changing FM state, separation latch or throttle cap. Centring the stick hands steering back to FM. RTM behaviour is unchanged.
// V2.5-Evo - 2026-07-19 - P3 FM: calcPWM() applies fm_throttle_cap (subtract-only, lowest cap wins) and lets fm_rx_active gate the steering override alongside rtm_rx_active. Throttle can still only be reduced, never added, and the thr_received>=25 steering gate is unchanged.
// V2.5-Evo - 2026-07-19 - FM triage: calcPWM() records effective_steer into g_effective_steer (diagnostic observer only — no control-path change) so the logger can show the actuation gap
// V2.5-Evo - 2026-04-30 - calcPWM() applies rtm_approach_cap for RTM approach decel zone
// V2.5-Evo - 2026-04-25 - P7: calcPWM() applies RTM emergency stop and steering override via effective_thr/steer
// V2.5-Evo - 2026-04-28 - Security: gate steer override on thr_received>=25 (belt-and-suspenders)
void generatePWM(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  while (1)
  {
    // V2.5-Evo - 2026-07-31 - RX-WDT-2: gated. initTasks() creates this task BEFORE
    // initWatchdog() subscribes it, so the first few iterations were calling
    // esp_task_wdt_reset() unregistered — which logs "task not found" at ERROR level and put
    // five spurious errors in every boot log. Control flow is untouched: once g_wdt_active is
    // set the feed happens exactly as before, and before that there is no watchdog to feed.
    // The RX has no display and no LED, so the boot log is the only diagnostic surface it has;
    // fake errors in it are not free.
    if (g_wdt_active) esp_task_wdt_reset();
    calcPWM();

    if(PWM_active && millis()-last_packet < usrConf.failsafe_time)
    {

      // V2.5-Evo - 2026-07-22 - <Rex HIGH> WDT self-preservation on i2cMutex.
      // BUG: this WDT-registered top-priority PWM task took i2cMutex with portMAX_DELAY at the
      // two AW9523 enable-swap sites. i2cMutex is shared with compass/ADS1115/AW9523-LED/logger;
      // if the I2C bus wedged, this task blocked unbounded and the 3000ms WDT panic-rebooted the
      // RX mid-ride. FIX: bound the take to 10ms and skip the swap on timeout (never block >10ms).
      // CONSISTENCY: the enable swap sets up the enable line for the NEXT iteration's pulse, so
      // alternatePWMChannel is now advanced ONLY when the swap actually succeeds. On timeout the
      // channel index is left unchanged, so next cycle re-pulses the SAME, still-enabled motor
      // (correct VESC) instead of firing the other channel's pulse onto the current enable state.
      // Worst case under bus contention: one motor gets repeated pulses for a few 10ms cycles while
      // the other misses updates — no wrong-motor pulse, no unbounded block, no WDT trip. On timeout
      // we do NOT hold the mutex, so we do NOT give it.
      if(alternatePWMChannel)
      {
        generate_pulse(PWM0_time);
        vTaskDelay(pdMS_TO_TICKS(2));
        if(xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
          aw.pinMode(AP_EN_PWM0, INPUT);
          aw.pinMode(AP_EN_PWM1, OUTPUT);
          xSemaphoreGive(i2cMutex);
          alternatePWMChannel = 0;  // advance only on a successful enable swap
        }
        // timeout: keep alternatePWMChannel=1 so PWM0 (still enabled) re-pulses next cycle
      }
      else
      {
        generate_pulse(PWM1_time);
        vTaskDelay(pdMS_TO_TICKS(2));
        if(xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
          aw.pinMode(AP_EN_PWM1, INPUT);
          aw.pinMode(AP_EN_PWM0, OUTPUT);
          xSemaphoreGive(i2cMutex);
          alternatePWMChannel = 1;  // advance only on a successful enable swap
        }
        // timeout: keep alternatePWMChannel=0 so PWM1 (still enabled) re-pulses next cycle
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void calcPWM()
{
  // V2.5-Evo - 2026-04-25 - P7: Apply RTM overrides before any PWM calculation.
  // Emergency stop: any safety gate failure forces throttle to neutral regardless of user input.
  // Steering override: bearing-derived value replaces radio steering when RTM is active.
  // RTM can only subtract from user throttle (never add). Creator safety philosophy enforced.
  uint8_t effective_thr   = rtm_rx_emergency_stop ? 0 : thr_received;
  // Approach decel zone: cap effective_thr when RTM is guiding the buggy into the stop zone.
  // rtm_approach_cap is 255 (no cap) in manual mode and outside the approach zone.
  // Only RTMState.ino sets it below 255 — during active RTM when dist < rtm_approach_zone_m.
  // RTM can only subtract from user throttle, never add — creator safety philosophy enforced.
  if ((uint8_t)rtm_approach_cap < effective_thr)
  {
    effective_thr = rtm_approach_cap;
  }

  // V2.5-Evo - 2026-07-19 - P3 FM: apply the Follow-Me throttle cap.
  // Same subtract-only shape as rtm_approach_cap above — lowest cap wins, and the cap can only
  // ever reduce the rider's throttle, never raise it. fm_throttle_cap is 255 (no cap) whenever FM
  // is idle or merely armed; runFmLoop() drives it to 0 on any FM fault or geometric hold, and to
  // the cap-chain result while FM is actively following. Creator safety philosophy enforced: the
  // human trigger remains the sole throttle source.
  if ((uint8_t)fm_throttle_cap < effective_thr)
  {
    effective_thr = fm_throttle_cap;
  }

  // SAFETY FIX (2026-04-28 audit): also gate on thr_received>=25.
  // Gate 1 in RTMState.ino resets rtm_steer_override=127 on throttle release (Task 1A),
  // but that runs at 10Hz. This gate ensures the PWM task (100Hz) cannot apply a stale
  // bearing value during the up-to-100ms window before Gate 1 next fires.
  // V2.5-Evo - 2026-08-26 - FM manual steering takeover. A deliberate rider input wins immediately
  // in this 100 Hz task, while FM remains ACTIVE in the 10 Hz state machine. This preserves the FM
  // mode, separation latch and subtract-only throttle cap; centring the input seamlessly restores
  // the latest automatic steering command. RTM deliberately keeps its existing behaviour.
  int fm_manual_steer_dev = (int)steering_received - 127;
  if (fm_manual_steer_dev < 0) fm_manual_steer_dev = -fm_manual_steer_dev;
  bool fm_manual_steer = fm_rx_active &&
                         (fm_manual_steer_dev >= (int)kFmManualSteerDeadband);
  bool automatic_steer = rtm_rx_active || (fm_rx_active && !fm_manual_steer);

  // Autonomous steering never reaches the motors on a released trigger. When FM manual takeover
  // is selected, steering_received is used below even though FM remains active in the background.
  uint8_t effective_steer = (automatic_steer && usrConf.rtm_rx_override_steering &&
                             thr_received >= 25)
                            ? (uint8_t)rtm_steer_override
                            : steering_received;

  // V2.5-Evo - 2026-07-19 - FM triage: record the steering byte actually applied this loop for
  // the logger. Diagnostic observer only — this write does not alter any PWM/motor control path.
  g_effective_steer = effective_steer;

  if(usrConf.steering_type == 0)
  {
    //Efoil mode
    PWM0_time = constrain(map(effective_thr, 0, 255, usrConf.PWM0_min, usrConf.PWM0_max) + usrConf.trim, usrConf.PWM0_min, usrConf.PWM0_max);
    PWM1_time = constrain(map(effective_thr, 0, 255, usrConf.PWM1_min, usrConf.PWM1_max) - usrConf.trim, usrConf.PWM1_min, usrConf.PWM1_max);
  }
  else if(usrConf.steering_type == 1)
  {
    //Diff
    // Map throttle input to PWM range for each motor
    uint16_t throttle_0 = map(effective_thr, 0, 255, usrConf.PWM0_min, usrConf.PWM0_max);
    uint16_t throttle_1 = map(effective_thr, 0, 255, usrConf.PWM1_min, usrConf.PWM1_max);

    // Compute differential steering adjustment with influence factor
    int max_steering_offset_0 = map(usrConf.steering_influence, 0, 100, 0, (usrConf.PWM0_max - usrConf.PWM0_min));
    int max_steering_offset_1 = map(usrConf.steering_influence, 0, 100, 0, (usrConf.PWM1_max - usrConf.PWM1_min));

    // V2.5-Evo - 2026-06-05 - H-1 recentering: removed the +1 bias AND recentre so neutral steering (127)
    // maps to exactly 0 — both motors sit at PWM_min at rest, killing the ~2us map-quantization residual
    // that read 1000,1002 (127 mapped to -2 because 0-255 has no whole-number centre). No dead zone, smooth
    // steering. Neutral stability is handled upstream by the TX tog_deadzone.
    int center_off_0 = map(127, 0, 255, -max_steering_offset_0, max_steering_offset_0);
    int center_off_1 = map(127, 0, 255, -max_steering_offset_1, max_steering_offset_1);
    int steering_offset_0 = map(effective_steer, 0, 255, -max_steering_offset_0, max_steering_offset_0) - center_off_0;
    int steering_offset_1 = map(effective_steer, 0, 255, -max_steering_offset_1, max_steering_offset_1) - center_off_1;

    if(usrConf.steering_inverted)
    {
      PWM0_time = constrain(throttle_0 + usrConf.trim + steering_offset_0, usrConf.PWM0_min, usrConf.PWM0_max);
      PWM1_time = constrain(throttle_1 - usrConf.trim - steering_offset_1, usrConf.PWM1_min, usrConf.PWM1_max);
    }
    else
    {
      PWM0_time = constrain(throttle_0 + usrConf.trim - steering_offset_0, usrConf.PWM0_min, usrConf.PWM0_max);
      PWM1_time = constrain(throttle_1 - usrConf.trim + steering_offset_1, usrConf.PWM1_min, usrConf.PWM1_max);
    }
  }
  else if(usrConf.steering_type == 2)
  {
    //Servo
    PWM0_time = map(effective_thr, 0, 255, usrConf.PWM0_min, usrConf.PWM0_max);
    if(usrConf.steering_inverted)
    {
      PWM1_time = constrain(map(effective_steer, 0, 255, usrConf.PWM1_min, usrConf.PWM1_max)+usrConf.trim, usrConf.PWM1_min, usrConf.PWM1_max);
    }
    else
    {
      PWM1_time = constrain(map(effective_steer, 255, 0, usrConf.PWM1_min, usrConf.PWM1_max)+usrConf.trim, usrConf.PWM1_min, usrConf.PWM1_max);
    }
  }
  else
  {
    PWM_active = 0;
  }

  // ── SAFETY: MOTOR RAMPING (usrConf.motor_ramp_s, seconds) ──────────────────────
  // Rise-limit BOTH motor outputs so 0->full takes motor_ramp_s seconds. Prevents a violent throttle
  // yank AND a single motor taking off (throttle- or steering-driven). FALL is instant so release /
  // failsafe / RTM e-stop / straightening drop the motor immediately. By design this also ramps the
  // differential-steering response (a sharp turn builds over this time). 0 = instant/off.
  if (usrConf.motor_ramp_s > 0.001f)
  {
    static uint16_t pwm0_ramp = 0, pwm1_ramp = 0;
    static bool     ramp_init = false;
    if (!ramp_init) { pwm0_ramp = usrConf.PWM0_min; pwm1_ramp = usrConf.PWM1_min; ramp_init = true; }
    uint16_t step0 = (uint16_t)max(1.0f, (float)(usrConf.PWM0_max - usrConf.PWM0_min) / (usrConf.motor_ramp_s * 100.0f));
    uint16_t step1 = (uint16_t)max(1.0f, (float)(usrConf.PWM1_max - usrConf.PWM1_min) / (usrConf.motor_ramp_s * 100.0f));
    if (PWM0_time > pwm0_ramp + step0) pwm0_ramp += step0; else pwm0_ramp = PWM0_time;
    if (PWM1_time > pwm1_ramp + step1) pwm1_ramp += step1; else pwm1_ramp = PWM1_time;
    PWM0_time = pwm0_ramp;
    PWM1_time = pwm1_ramp;
  }

  // ============================================================
  // V2.5-Evo - 2026-07-27 - SAFETY-NEUTRAL-1: RELEASED THROTTLE == ABSOLUTE MINIMUM.
  //
  // THIS RUNS LAST, AFTER EVERY OTHER CALCULATION, ON PURPOSE. It is a structural
  // guarantee, not a correction of any one contributor: with the trigger released, both
  // outputs are the configured minimum and NOTHING is permitted to lift them.
  //
  // WHAT WAS WRONG — confirmed by arithmetic, not inference:
  //   throttle_0 = map(0, 0,255, PWM0_min, PWM0_max)  ->  PWM0_min
  //   PWM0_time  = constrain(throttle_0 + usrConf.trim - steering_offset_0, min, max)
  // so at ZERO throttle the output was PWM0_min + trim. Any non-zero trim parked a motor
  // above its minimum forever — trigger released, radio idle, buggy on the dock. The
  // asymmetry matches the field report exactly: POSITIVE trim lifts motor 0, NEGATIVE trim
  // lifts motor 1, so exactly ONE motor creeps. Owner observed one motor starting on its
  // own at the dock, 2026-07-27, and had been compensating by widening the VESC's own
  // deadband — i.e. masking an RX bug inside the ESC.
  //
  // AND THE ONE I DID NOT PROVE: trim is a CONSTANT offset, but the owner described the
  // behaviour as DRIFTING. Steering neutral is the mechanism that actually drifts —
  // steering_offset_0/1 recentre to exactly 0 ONLY when steering_received == 127 exactly,
  // and the H-1 comment above openly delegates that to the TX tog_deadzone. A TX whose
  // battery rail is sagging (the TX died at the dock the same day) shifts its ADC reference
  // and therefore its apparent stick centre. That lifts a motor and it drifts.
  //
  // WHY IT IS WRITTEN THIS WAY: patching trim alone would have fixed the cause I proved and
  // left the cause I suspect — plus ramp residue and whatever is added next — still able to
  // put throttle on a motor the rider is not asking for. Enforcing the invariant ONCE, at
  // the end, makes it independent of every upstream term. The creator safety philosophy
  // already written throughout this file ("the human trigger remains the sole throttle
  // source") is now enforced instead of merely assumed.
  //
  // The test is effective_thr, not thr_received: RTM e-stop and the FM/RTM caps all drive
  // effective_thr to 0, so an autonomous stop lands on true minimum too rather than on
  // minimum-plus-trim.
  //
  // NOTE: this is deliberately == 0 and NOT a deadband. If the TX ever sends a non-zero
  // throttle with the trigger released, that is a TX fault that must be found and fixed at
  // source, and swallowing it in a deadband here would hide it.
  // ============================================================
  if (effective_thr == 0)
  {
    PWM0_time = usrConf.PWM0_min;
    PWM1_time = usrConf.PWM1_min;
  }
}

void initRMT()
{
  // Initialize RMT TX channel
  rmt_tx_channel_config_t tx_chan_config = {
    .gpio_num = RMT_TX_GPIO_NUM,
    .clk_src = RMT_CLK_SRC_DEFAULT,  // Select APB clock (80MHz)
    .resolution_hz = 1000000,         // 1MHz, 1 tick = 1μs
    .mem_block_symbols = 64,
    .trans_queue_depth = 4,
  };

  tx_chan_config.flags.io_od_mode = 0; //open-drain
  // Create RMT TX channel
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_channel));
  // Create RMT encoder
  rmt_copy_encoder_config_t copy_encoder_config = {};
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &copy_encoder));
  // Enable RMT TX channel
  ESP_ERROR_CHECK(rmt_enable(tx_channel));
}

void generate_pulse(uint16_t pulse_width_us) 
{
  pulse_symbol.level0 = 1;
  pulse_symbol.duration0 = pulse_width_us;  // High time in microseconds
  pulse_symbol.level1 = 0;
  pulse_symbol.duration1 = 1;  // Low time in microseconds
  // Create a transmission that loops the same pattern (creates a continuous signal)
  rmt_transmit_config_t tx_config = {
    .loop_count = 1,  // Infinite loop
  };
  tx_config.flags.eot_level = 0; // End-of-transmission level (LOW)
  // Send the pulse pattern
  ESP_ERROR_CHECK(rmt_transmit(tx_channel, copy_encoder, &pulse_symbol, 
                              sizeof(pulse_symbol), &tx_config));
}
