// V2.5-Evo - 2026-09-19 - SEPARATE RAMPS (owner decision 14:00): the motor rise-limit now selects its seconds on the OWNER flags - (rtm_rx_active || fm_rx_active) && auto_ramp_s > 0 ? auto_ramp_s : motor_ramp_s - so Follow-Me following, FM_RETURN motion and classic RTM (a stick takeover included) ride the automatic modes' fast ramp while manual towing and every hand-back to the rider (ARMED-not-engaged, FM_STOPPING, a HOLD escape, Gate 9, the mode-0 cancel) ride the slow manual one. The step itself moved verbatim into Common/OutputRamp.h (pure, host-tested: rise <= target, instant fall, continuity across a rate change) and gains the stale-state fix: the memory tracks the output while the ramp is off, so a ramp switched on mid-session no longer dips. With auto_ramp_s 0 the selector always yields motor_ramp_s and the bytes are identical to before in any session where motor_ramp_s is not changed live. The terminal effective_thr == 0 clamp stays the last writer. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - STICK TAKEOVER: calcPWM()'s steering-source selector reads the steer_takeover_active atomic (BREmote_V2_Rx.h; published once per 10 Hz tick by RTMState.ino while the rider's stick has taken over an automatic steering run under usrConf.steer_during_auto 1). The unchanged auto_steer test becomes auto_owner; auto_cmd = auto_owner && !takeover selects rtm_steer_override, otherwise the stick; the pivot boost follows auto_cmd. Nothing else changes: no new writer of effective_thr, any cap, rtm_steer_override or a PWM time; the thr_received >= 25 gate, the efoil and servo branches, the ramp and the terminal effective_thr == 0 clamp are untouched. With the atomic false (the setting at 0, every fielded board) the selector is the old `auto_steer ? override : stick`, byte for byte (host test). No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - PIVOT BOOST: in steering_type 1, calcPWM() passes align_mixer_influence_override (BREmote_V2_Rx.h; published by RTMState.ino while Follow-Me align, FM_RETURN align or classic RTM Phase 1 align is turning the buggy to face its target) to mixThrottleRelativeDifferential() in place of usrConf.steering_influence when it is non-zero AND an autonomous steering override is being applied on this tick (the same gate as effective_steer). Manual steering, the efoil and servo branches, the ramp and the terminal effective_thr == 0 guard are untouched. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - C-4 FIX: throttle-relative differential mixer. BUG: in steering_type 1 the steering term was a fixed fraction of the FULL PWM span (steering_influence % of PWM_max-PWM_min) added AFTER the throttle map, so it was never scaled by effective_thr - a hard steer during a 13/255 crawl (influence 50) put one motor at ~55 % regardless of the RTM/FM throttle cap, i.e. steering could ADD power past the cap. FIX: calcPWM() now calls mixThrottleRelativeDifferential() (Common/DifferentialMixer.h, host-tested in Tools/tests/differential_mixer_test.cpp): turn = T x influence x |steer-127| / (100 x span), motor0 = T - turn, motor1 = T + turn, each clamped 0..255, then each motor command is map()ed into its own PWM range and trim is applied as the same symmetric post-map correction as before (+trim ch0, -trim ch1). Steering is now proportional to the permitted throttle - none at zero, less at low throttle, full authority at full throttle - and before upper saturation the two commands always sum to exactly 2T (power-neutral; saturation can only lower it). 127 is the exact neutral byte inside the mixer, so the 2026-06-05 H-1 recentring is no longer needed and is removed. steering_inverted semantics are unchanged: 0 -> steer > 127 slows motor0 / speeds motor1; 1 -> the mirror. Efoil and servo branches, the ramp and the terminal effective_thr==0 guard are untouched. No confStruct change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-09-18 - comment only (review finding F8): the steering-gate note said runFmLoop() forces FM to IDLE whenever rtm_rx_active is set; since 2026-09-18 it makes FM yield (FM_ARMED, fm_rx_active false, no cap/steer writes) instead. No code change.
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
  // V2.5-Evo - 2026-07-19 - P3 FM: FM steers through this same gate. RTM and FM are mutually
  // exclusive (since 2026-09-18 runFmLoop() makes FM YIELD whenever rtm_rx_active is set - it parks
  // in FM_ARMED with fm_rx_active false and writes neither cap nor steering until RTM ends; before
  // that it forced FM to IDLE), so they safely share
  // rtm_steer_override as the steering command. The thr_received>=25 condition is unchanged and
  // still applies to both — autonomous steering never reaches the motors on a released trigger.
  // V2.5-Evo - 2026-09-19 - STICK TAKEOVER (usrConf.steer_during_auto 1). Three names for one
  // decision, so the motor path stays readable:
  //   auto_owner - the test above, unchanged: an autonomous controller owns the steering this tick.
  //   takeover   - the rider's stick has taken that steering over (steer_takeover_active, decided
  //                at 10 Hz in RTMState.ino with 500 ms / 200 ms persistence and 40 / 20 count
  //                hysteresis, published through ONE atomic with one writer; this task only reads).
  //   auto_cmd   - the controller's byte is applied: an owner exists AND no takeover stands.
  // The selector is the ONLY thing the takeover changes here: which of the two EXISTING steering
  // sources feeds the mixer. effective_thr, every cap above, rtm_steer_override and the PWM times
  // have exactly the writers they had. A takeover with no owner is impossible (auto_owner carries
  // thr_received >= 25, so a released trigger makes the flag irrelevant on this very cycle and
  // effective_thr == 0 lands both outputs at PWM_min below), and with the setting at 0 the flag is
  // never true, so auto_cmd == auto_steer and the bytes are identical to before.
  const bool auto_owner = ((rtm_rx_active || fm_rx_active) && usrConf.rtm_rx_override_steering && thr_received >= 25);
  const bool takeover   = auto_owner && steer_takeover_active.load(std::memory_order_relaxed);
  const bool auto_cmd   = auto_owner && !takeover;
  uint8_t effective_steer = auto_cmd ? (uint8_t)rtm_steer_override : steering_received;

  // V2.5-Evo - 2026-09-19 - PIVOT BOOST. While an autonomous controller is in its align phase it
  // publishes usrConf.fm_align_influence here (see align_mixer_influence_override in the header);
  // the mixer then splits the permitted throttle harder between the two motors - at 100 one motor
  // stops and the other gets twice the align cap, a pivot on the spot - WITHOUT adding any power
  // (the mixer is throttle-relative: motor0 + motor1 <= 2T always). It is honoured only on ticks
  // where the autonomous steering override itself is honoured, so manual steering is never mixed
  // with a boosted influence, and 0 means "use steering_influence" exactly as before.
  // V2.5-Evo - 2026-09-19 - and that means auto_cmd, not auto_owner: a takeover mixes the rider's
  // stick at steering_influence (the "manual feel never changes" rule), the boost follows the
  // CONTROLLER's command only.
  uint16_t mix_influence = usrConf.steering_influence;
  {
    const uint8_t boost = align_mixer_influence_override.load(std::memory_order_relaxed);
    if (auto_cmd && boost != 0) mix_influence = boost;
  }

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
    // V2.5-Evo - 2026-09-19 - C-4 FIX: steering redistributes the PERMITTED throttle instead of adding
    // a gas-independent offset taken from the full PWM span. The old form was
    //   PWM0 = map(effective_thr) + trim -/+ steering_offset_0,  steering_offset = influence% of (PWM_max - PWM_min)
    // so at a 13/255 crawl with influence 50 a full-lock steer still moved one motor by 500 us — steering
    // could push a motor past every throttle cap (RTM approach, FM cap) that had just been applied above.
    // The mixer works in normalized 0..255 command space: turn = T x influence x |steer - 127| / (100 x span),
    // motor0 = T - turn, motor1 = T + turn (inversion flips the sign of turn), each clamped 0..255. At T = 0 the
    // turn term is zero for every steering byte, so steering can no longer create motor command from zero gas;
    // below upper saturation motor0 + motor1 == 2T exactly, so steering never adds aggregate power. 127 is the
    // exact neutral inside the mixer, which is why the former H-1 recentring (2026-06-05) is gone: there is no
    // map() residual to cancel. Mapping each command into its own calibrated PWM range afterwards preserves the
    // same relative power request even when the two channels have unequal ranges.
    // Inversion semantics are unchanged from the old branch: steering_inverted 0 -> a steer byte above 127 slows
    // motor 0 and speeds motor 1; steering_inverted 1 -> the mirror (motor 0 speeds, motor 1 slows).
    DifferentialMotorMix motor_mix = mixThrottleRelativeDifferential(
        effective_thr, effective_steer, mix_influence,   // V2.5-Evo - 2026-09-19 - steering_influence, or the align pivot boost
        usrConf.steering_inverted);
    int motor_0_pwm = map(motor_mix.motor0, 0, 255, usrConf.PWM0_min, usrConf.PWM0_max);
    int motor_1_pwm = map(motor_mix.motor1, 0, 255, usrConf.PWM1_min, usrConf.PWM1_max);

    // Trim stays a symmetric post-map correction with the same sign convention as before and as the efoil
    // branch: +trim on channel 0, -trim on channel 1, independent of steering_inverted. The final
    // effective_thr == 0 clamp below still owns the absolute stopped state and cannot be bypassed by trim.
    PWM0_time = constrain(motor_0_pwm + usrConf.trim, usrConf.PWM0_min, usrConf.PWM0_max);
    PWM1_time = constrain(motor_1_pwm - usrConf.trim, usrConf.PWM1_min, usrConf.PWM1_max);
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

  // ── SAFETY: MOTOR RAMPING (usrConf.motor_ramp_s / usrConf.auto_ramp_s, seconds) ──────────
  // Rise-limit BOTH motor outputs so 0->full takes the ramp's seconds. Prevents a violent throttle
  // yank AND a single motor taking off (throttle- or steering-driven). FALL is instant so release /
  // failsafe / RTM e-stop / straightening drop the motor immediately. By design this also ramps the
  // differential-steering response (a sharp turn builds over this time). 0 = instant/off.
  //
  // V2.5-Evo - 2026-09-19 - TWO RAMPS (owner decision 14:00). The slow motor_ramp_s exists for the
  // rider's shoulder on a MANUAL tow start, where the rope is loaded; in the automatic modes the rope
  // is not loaded and a fast ramp is what makes the buggy reactive. The ramp in force this tick is
  // selected on the OWNER flags - rtm_rx_active || fm_rx_active, whoever is CAPPING the throttle -
  // and deliberately NOT on auto_owner above, which also carries the trigger and the stick switch:
  //   - Follow-Me following, FM_RETURN moving and classic RTM ride auto_ramp_s (when it is non-zero;
  //     0 = inherit motor_ramp_s, today's behaviour). A stick takeover rides it too: the steering
  //     byte changes hands, the throttle is still the automatic owner's.
  //   - FM_ARMED-not-engaged towing, FM_STOPPING's hand-back ramp, a HOLD escape, Gate 9's handoff and
  //     the mode-0 steer-cancel un-clamp all ride motor_ramp_s: on every one of those the owner flag
  //     is already false, so every hand-back to the rider is softened by the slow slew (the shoulder
  //     case) - the fast ramp never lands on the rider's own throttle.
  // The step lives in Common/OutputRamp.h (pure, host-tested: rise <= target, instant fall,
  // continuity across a rate change, and the memory now TRACKS the output while the ramp is off, so a
  // ramp switched on mid-session resumes from the live output instead of the stale value the old
  // inline block kept - a momentary dip, fixed here). The memory holds the last OUTPUT and the step
  // is recomputed on every 100 Hz tick, so switching between the two ramps mid-rise continues from
  // the current output at the new rate - no jump either way. The terminal effective_thr == 0 clamp
  // below stays the last writer. RULE FOR P2 (not built): an automatic mode with the buggy AHEAD of
  // the rider (front_along_m > 0) uses motor_ramp_s - see auto_ramp_s in the header.
  {
    static OutputRampState motor_ramp = {0, 0, false};
    const bool  auto_ramp_owner = (rtm_rx_active || fm_rx_active) && (usrConf.auto_ramp_s > 0.001f);
    const float ramp_s = auto_ramp_owner ? usrConf.auto_ramp_s : usrConf.motor_ramp_s;
    uint16_t ramped0, ramped1;
    outputRampStep(&motor_ramp, PWM0_time, PWM1_time,
                   usrConf.PWM0_min, usrConf.PWM0_max, usrConf.PWM1_min, usrConf.PWM1_max,
                   ramp_s, 100.0f, &ramped0, &ramped1);
    PWM0_time = ramped0;
    PWM1_time = ramped1;
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
  //
  // V2.5-Evo - 2026-09-19 - The two contributors named above (the full-span steering_offset_0/1
  // and the H-1 recentring it needed) no longer exist: the throttle-relative mixer in the diff
  // branch makes the turn term zero whenever effective_thr is zero, for every steering byte and
  // both inversions, so the drift mechanism suspected above is closed by construction. The
  // history is kept as written. This clamp deliberately stays as defence in depth for trim,
  // ramp residue and whatever is added next — it is the last writer and therefore the stronger
  // stopped-state invariant.
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
