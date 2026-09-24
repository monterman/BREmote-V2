// V2.5-Evo - 2026-09-24 - RAMP BEFORE THE MIXER, STEERING IS INSTANT. WHAT WAS WRONG: the rise-limit sat AFTER the differential mixer and rate-limited PWM0_time / PWM1_time, i.e. the two finished MOTOR outputs - and on a differential drive the turn IS the difference between those two outputs, so the block rate-limited the steering as well. Its own comment admitted it ("By design this also ramps the differential-steering response (a sharp turn builds over this time)"). That is the one thing the owner has ruled out repeatedly: the ramp exists for the THROTTLE - a soft tow start that pulls a rider off a shoulder without snatching the rope - while steering must land on the very next 10 ms pass. On the servo branch it was worse: the block was rate-limiting the steering SERVO channel itself, which has no throttle in it at all. FIX: the rise-limit moves UP into the throttle domain (0-255 counts), applied to effective_thr AFTER every cap and BEFORE the mixer, producing ramped_thr. All three branches drive their motor channel(s) from ramped_thr; the mixer computes the turn from ramped_thr with no rate limit of its own, so a stick flick moves both motors on the next pass; the post-mixer block is deleted (keeping it would double-ramp). RISE ONLY - the fall branch snaps onto the target, so trigger release, failsafe, RTM/FM emergency stop and straightening still drop the output on the same tick. motor_ramp_s <= 0.001 still means instant/off. CAPS STILL BOUND THE OUTPUT: the ramp only ever approaches effective_thr from below, so ramped_thr <= effective_thr <= rtm_approach_cap / fm_throttle_cap on every tick - a cap DROP is instant, a cap RISE is slewed, exactly as before - and the pivot boost still only redistributes that permitted throttle. STEP - Q12 FIXED POINT, and why it has to be: the ramp accumulates in 1/4096 of a throttle count (full scale 255 x 4096 = 1044480, a uint32_t), step = full scale / (motor_ramp_s x 100 ticks per second). A whole-count integer step does not work in this domain - 255 counts is four times coarser than the ~1000-count PWM span the old post-mixer ramp used, so a whole-count step turned the owner's 1.00 s setting into 1.28 s and floored every setting from 1.275 s upward at one fixed 2.55 s (4.00 s would have run 36 % FAST, the wrong direction). With Q12 the entire configured 0.20-4.00 s range lands WITHIN ONE 10 ms TICK of nominal and always on the slow side, because truncation is the safe direction and the ramp is never faster than asked: 0.20 s -> 0.20 s; 0.50 s -> 0.51 s; 0.75 s -> 0.76 s; 1.00 s -> 1.01 s; 1.50 s -> 1.51 s; 2.00 s -> 2.01 s; 3.00 s -> 3.01 s; 4.00 s -> 4.01 s. The residual is the single tick it costs to land exactly on target - a flat +10 ms, which is +0 % at the 0.20 s end (20 ticks divide exactly), +2 % worst relative at 0.50 s, and +0.25 % at 4.00 s; measured by brute-force sweep of the whole range in 0.01 s steps, worst case +1 tick, zero cases faster than configured. The old max(1.0f, ...) WHOLE-count floor is gone; a 1-unit (1/4096 count) floor replaces it purely to guarantee forward progress against a corrupt out-of-range config value, and it cannot bite anywhere in the validated range, where the step runs 2611 (4.00 s) to 52224 (0.20 s). One shared ramp also means the two motors can no longer slew at different real rates when one channel's PWM span hits a floor. DEEP LOG: g_motor0_cmd / g_motor1_cmd now carry the ramp and are still exactly what their name says (post-mixer, pre-map motor commands); thr_received_log is still the raw TX byte and g_effective_steer is still the steering byte applied. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - DEEP LOG (C): calcPWM() records the two post-mixer, pre-map motor commands (motor_mix.motor0 / motor1, 0-255 counts) into g_motor0_cmd / g_motor1_cmd - diagnostic observers only, the g_effective_steer pattern: written on every pass, never read back into any control path. The steering_type 1 branch writes the mixer's values; the efoil and servo branches write 0 / 0. No PWM value, cap, gate or ramp is touched. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
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

  // -- SAFETY: THROTTLE RAMPING (usrConf.motor_ramp_s, seconds) -------------------
  // V2.5-Evo - 2026-09-24 - The ramp lives HERE now: in the throttle domain, after every cap and
  // before the differential mixer, so that it can only ever slow the THROTTLE down. It used to sit
  // after the mixer and rate-limit the two finished motor outputs, which by definition also
  // rate-limited the difference between them - the steering. Owner rule: never ramp steering. The
  // ramp is for a soft tow start; a turn has to be there on the next 10 ms pass.
  //
  // RISE ONLY. The else-branch snaps the memory straight onto the target, so every kind of stop -
  // trigger release, failsafe, RTM/FM emergency stop, a cap collapsing to 0, straightening - drops
  // the throttle on the same tick with no slew at all. That same else-branch is also why the ramp
  // can neither stall nor jump: it is the only way the memory can move DOWN, and it always moves it
  // the whole way in one tick.
  //
  // WHAT thr_ramp_q12 HOLDS WITH THE TRIGGER RELEASED: exactly 0. effective_thr is 0 so the target
  // is 0, the rise test (0 > thr_ramp_q12 + step) is false, the first released tick assigns 0 and it stays
  // there. Re-engaging always starts a fresh full ramp from 0 - the rider cannot bank ramp credit by
  // feathering the trigger, and there is no stale value left to jump to. 0 is also the power-on
  // value of the static, so the first squeeze after boot behaves like every later one; no init flag
  // is needed here (the old PWM-domain version needed one only because its resting value was
  // PWM_min, not 0).
  //
  // STEP, AND WHY IT IS FIXED POINT: 0 -> full must take motor_ramp_s seconds anywhere in the
  // configured 0.20-4.00 s range, and at a 10 ms tick a 4 s ramp needs 0.64 of a throttle count per
  // pass. A whole-count step cannot express that - it floors at 1 count, which is 2.55 s, so every
  // setting from 1.275 s up would collapse onto the same ramp and 4.00 s would run 36 % FAST. So the
  // accumulator carries 1/4096 of a count (Q12) in a uint32_t: full scale is 255 x 4096 = 1044480
  // and the step is that over (motor_ramp_s x 100). The step is TRUNCATED, never rounded, because
  // truncation can only make the ramp slower than configured, and slower is the safe direction.
  // Result across the whole range, always +1 tick at worst and never fast: 0.20 s -> 0.20 s,
  // 0.50 s -> 0.51 s, 0.75 s -> 0.76 s, 1.00 s -> 1.01 s, 2.00 s -> 2.01 s, 4.00 s -> 4.01 s.
  // Overflow: the accumulator never exceeds full scale, the largest in-range step is 52224, so the
  // rise test tops out near 1.1 M - three orders of magnitude inside a uint32_t.
  //
  // motor_ramp_s 0 (or <= 0.001) = off: ramped_thr is effective_thr, instant, exactly as before.
  uint8_t ramped_thr = effective_thr;
  if (usrConf.motor_ramp_s > 0.001f)
  {
    const uint32_t kThrRampFullQ12 = 255UL * 4096UL;   // 1044480 = throttle 255 in Q12 units
    static uint32_t thr_ramp_q12 = 0;                  // the ramp memory, in 1/4096 of a count

    const uint32_t target_q12 = (uint32_t)effective_thr * 4096UL;
    const float    step_f     = (float)kThrRampFullQ12 / (usrConf.motor_ramp_s * 100.0f);
    // A ramp shorter than one tick, or a corrupt sub-0.01 s config value, means "instant": clamp the
    // step to full scale so the rise test below always takes the else-branch. The 1-unit floor is
    // the successor of the old whole-count floor and exists only so a nonsense value can never stall
    // the ramp at zero; at 1/4096 of a count it is unreachable anywhere in the validated range.
    uint32_t step_q12 = (step_f >= (float)kThrRampFullQ12) ? kThrRampFullQ12 : (uint32_t)step_f;
    if (step_q12 == 0) step_q12 = 1;

    if (target_q12 > thr_ramp_q12 + step_q12) thr_ramp_q12 += step_q12;
    else                                      thr_ramp_q12 = target_q12;   // instant fall, and the
                                                                           // only downward path
    ramped_thr = (uint8_t)(thr_ramp_q12 >> 12);   // always <= effective_thr, so every cap still bounds it
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
  const bool auto_steer = ((rtm_rx_active || fm_rx_active) && usrConf.rtm_rx_override_steering && thr_received >= 25);
  uint8_t effective_steer = auto_steer ? (uint8_t)rtm_steer_override : steering_received;

  // V2.5-Evo - 2026-09-19 - PIVOT BOOST. While an autonomous controller is in its align phase it
  // publishes usrConf.fm_align_influence here (see align_mixer_influence_override in the header);
  // the mixer then splits the permitted throttle harder between the two motors - at 100 one motor
  // stops and the other gets twice the align cap, a pivot on the spot - WITHOUT adding any power
  // (the mixer is throttle-relative: motor0 + motor1 <= 2T always). It is honoured only on ticks
  // where the autonomous steering override itself is honoured, so manual steering is never mixed
  // with a boosted influence, and 0 means "use steering_influence" exactly as before.
  uint16_t mix_influence = usrConf.steering_influence;
  {
    const uint8_t boost = align_mixer_influence_override.load(std::memory_order_relaxed);
    if (auto_steer && boost != 0) mix_influence = boost;
  }

  // V2.5-Evo - 2026-07-19 - FM triage: record the steering byte actually applied this loop for
  // the logger. Diagnostic observer only — this write does not alter any PWM/motor control path.
  g_effective_steer = effective_steer;

  // V2.5-Evo - 2026-09-19 - DEEP LOG (C): the two motor commands the differential mixer produces this
  // pass, for the logger. Locals here, published once below the branch chain so the pair is written
  // together and reads 0 / 0 on the branches that have no mixer. Diagnostic observers only - the
  // g_effective_steer pattern; nothing in this function or anywhere else reads them back.
  uint8_t motor0_cmd_obs = 0;
  uint8_t motor1_cmd_obs = 0;

  if(usrConf.steering_type == 0)
  {
    //Efoil mode
    // V2.5-Evo - 2026-09-24 - ramped_thr, not effective_thr: the rise-limit now runs ahead of this
    // branch chain instead of behind it. This branch has no steering term at all, so both channels
    // simply follow the ramped throttle - the same trajectory the old post-map ramp produced (map()
    // is linear, so a linear rise in command counts is a linear rise in microseconds), with one
    // shared rate instead of two independently-floored per-channel rates.
    PWM0_time = constrain(map(ramped_thr, 0, 255, usrConf.PWM0_min, usrConf.PWM0_max) + usrConf.trim, usrConf.PWM0_min, usrConf.PWM0_max);
    PWM1_time = constrain(map(ramped_thr, 0, 255, usrConf.PWM1_min, usrConf.PWM1_max) - usrConf.trim, usrConf.PWM1_min, usrConf.PWM1_max);
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
    // V2.5-Evo - 2026-09-24 - the mixer is fed ramped_thr (the rise-limited throttle) and the LIVE
    // steering byte. The mixer is memoryless, so the split is recomputed from scratch every 10 ms:
    // the throttle it is allowed to share out builds over motor_ramp_s, while a stick flick changes
    // how that throttle is shared on the very next pass. Steering has no rate limit anywhere now.
    // Everything the C-4 note above says still holds with T = ramped_thr, including the key one: at
    // T = 0 the turn term is zero for every steering byte, and ramped_thr is 0 exactly when
    // effective_thr is 0 (the ramp's fall is instant), so the stopped state is unchanged.
    DifferentialMotorMix motor_mix = mixThrottleRelativeDifferential(
        ramped_thr, effective_steer, mix_influence,   // V2.5-Evo - 2026-09-19 - steering_influence, or the align pivot boost
        usrConf.steering_inverted);
    motor0_cmd_obs = motor_mix.motor0;   // DEEP LOG (C): post-mixer, pre-map counts, every pass of this branch
    motor1_cmd_obs = motor_mix.motor1;
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
    // V2.5-Evo - 2026-09-24 - throttle channel follows ramped_thr; the STEERING SERVO on channel 1
    // below is deliberately left alone. The deleted post-mixer block used to rate-limit PWM1_time
    // too, which on this steering_type meant the servo itself could only sweep at the ramp rate - a
    // ramped steering wheel, with no throttle in it at all. That is exactly the behaviour the owner
    // rules out; the servo now tracks the stick with no limiting.
    PWM0_time = map(ramped_thr, 0, 255, usrConf.PWM0_min, usrConf.PWM0_max);
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

  // DEEP LOG (C) 2026-09-19: publish the two observers (diagnostic only; see the declaration in the header).
  g_motor0_cmd = motor0_cmd_obs;
  g_motor1_cmd = motor1_cmd_obs;

  // V2.5-Evo - 2026-09-24 - the post-mixer MOTOR RAMPING block that used to sit here is gone. It
  // rate-limited PWM0_time / PWM1_time, and the difference between those two IS the differential
  // steering, so it slewed every turn as well - the behaviour the owner has ruled out. Its job now
  // happens before the mixer, on the throttle itself (see the ramp block above). Leaving it here as
  // well would double-ramp the throttle.

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
