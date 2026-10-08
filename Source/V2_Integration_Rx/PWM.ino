// V2.5-Evo - 2026-10-03 - I2C ENABLE-SWAP STARVATION, STEP 5 of 5 - THE SWAP NOW CHECKS THE BUS, NOT JUST THE SEMAPHORE (delta audit C-1; see Init.ino, System.ino, Logger.ino). WHAT WAS STILL BROKEN AFTER STEPS 1-4: Adafruit_AW9523::pinMode() returns void (Adafruit_AW9523.cpp:253-279), so the two enable writes could fail silently ON THE BUS while xSemaphoreTake() reported success - which is exactly what happens on a wedged bus with LOW contention, and STEP 4 made that case MORE reachable by removing contention. The enable lines never moved, alternatePWMChannel was advanced anyway, g_swap_fail_run was ZEROED and g_swap_ok_run incremented: the firmware reported a healthy bus and an OPEN motor gate while ONE VESC RECEIVED BOTH CHANNELS' WIDTHS ALTERNATELY AT 50 Hz AND THE OTHER RECEIVED NOTHING. That is the original asymmetric-thrust failure, undetected, with STEP 2's counters reading clean and STEP 3 unable to help because g_swap_starved could never become true. THE FIX: the two aw.pinMode() calls at each swap site are replaced by ONE CHECKED WRITE of the AW9523's CONFIG1 direction register (awSetEnableDirections() below) - both enables are port-1 pins (AP_EN_PWM0 13, AP_EN_PWM1 12), so both direction bits live in that single register - and the bus result is treated EXACTLY as a semaphore timeout already was: alternatePWMChannel is NOT advanced, the per-channel and consecutive failure counters increment, g_swap_ok_run is cleared. The state machine cannot tell the two failure kinds apart, so kSwapStarveTicks (25) and kSwapRecoverTicks (5) keep working unchanged and a silent bus failure now reaches the symmetric stop in 250 ms instead of never. COST, and it is in the right direction: the swap drops from four register read-modify-writes (~8-12 Wire transactions, ~28 bytes on the wire, ~2.5 ms at 100 kHz) to ONE 3-byte write (~0.3 ms), because half of what pinMode() did was rewriting the LEDMODE register identically 100 times a second. Every hold-time margin in the STEP 1-3 derivations widens by roughly 8x. NOT CHANGED: the pulse-then-2 ms-then-swap ordering, the swap being attempted on every link_ok tick, the channel index advancing only on a verified swap, and the count of gate-expression copies - still exactly TWO, and as of this commit textually identical again (audit L-2). ALSO IN THIS COMMIT: Init.ino's STEP 1 justification is corrected (audit M-2), ?diag names the third cause of a CLOSED motor gate and prints g_swap_starved (audit M-6), and the logger's delta snapshot is seeded on its first call (audit L-3). No confStruct change, sizeof stays 200, SW_VERSION stays 36, no log format change.
// V2.5-Evo - 2026-10-03 - I2C ENABLE-SWAP STARVATION, STEP 4 of 4 - COMMENT ONLY in this file, no code touched (see Compass.ino, Logger.ino, System.ino): the 2026-07-22 note listed the i2cMutex sharers as "compass/ADS1115/AW9523-LED/logger". THERE IS NO ADS1115 ON THE RX. It is a TX part (0x48, Display.ino / Analog.ino) and appears nowhere in the RX firmware or on the RX schematic; the RX's AP_BMS_MEAS is a plain AW9523 pin given pinMode(INPUT) in startupAW() and then never read anywhere, and getUbatLoop() measures the battery with the ESP32's own analogRead() on GPIO 0, with no I2C involved. The stale name cost real time - it sent the 2026-10-03 audit hunting for an ADC conversion wait held inside this lock, and no such holder exists on this board. The list now names the real sharers: the compass, the AW9523 LED and UART-mux writes, the button reads and the logger. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-10-03 - I2C ENABLE-SWAP STARVATION, STEP 3 of 4 - FAIL SYMMETRIC, NOT ASYMMETRIC. THIS IS THE SAFETY FIX AND IT IS CONFINED TO THIS FILE. WHAT WAS WRONG: when the enable swap loses i2cMutex, one motor keeps being pulsed and the other gets nothing; the starved VESC waits out its own timeout_msec (1000 ms) and releases while the other HOLDS THE RIDER'S COMMANDED THROTTLE. A differential-drive buggy in that state does not stop - it turns hard under power, toward the rider and the trailing rope. The owner has already been hit by this buggy once. WHAT IT DOES NOW: after kSwapStarveTicks (25 ticks = 250 ms) of consecutive failures, NEITHER channel is pulsed. Both VESCs time out together and the buggy COASTS - the failure direction this system already has and the rider has already met (trigger release, link failsafe, VESC timeout all produce it), rather than an uncommanded yaw at power he has no training for. 250 ms is one quarter of the VESC's 1000 ms timeout, so the symmetric stop lands ~750 ms BEFORE the starved VESC would have released and the asymmetry never becomes mechanical at all. HOW IT IS IMPLEMENTED, and this part is non-negotiable: as a TERM IN THE GATE EXPRESSION (!g_swap_starved, added at the pulse site AND at the calcPWM() mirror - one new && token each, still exactly TWO copies of that expression), NOT as a skipped generate_pulse() call. A skipped pulse would leave motor_gate_open true, M-3's ramp target would keep climbing on the rider's live throttle, and recovery would STEP STRAIGHT TO FULL COMMANDED THROTTLE - the M-3 snapback, reintroduced by the safety fix at the worst possible moment. As a gate term the ramp target goes to 0, recovery is a full soft ramp from 0 over motor_ramp_s, BOTH channels resume together with normal alternation, the effective_thr == 0 clamp is untouched, and ?diag / ?printpwm / the log's motor_gate_open column all report the starvation with no new code. RECOVERY IS A SCHMITT TRIGGER, NOT A LATCH: 25 failures in, kSwapRecoverTicks (5) consecutive successes out. One lucky swap must not resume output - because M-3 resets the ramp whenever the gate shuts, a flapping cut/resume would re-ramp every time and PIN THE THROTTLE near zero, which is F-1 of the 2026-10-02 M-3 delta audit. No hard latch either: a latch turns a transient into a dead ride a rider cannot recover from on the water. THE SWAP IS STILL ATTEMPTED ON EVERY link_ok TICK, starved or not - identical frequency to before, zero new I2C traffic - because an attempt left inside the pulse gate could never clear the counter and would have been an accidental permanent latch. No reordering of the pulse-then-2 ms-then-swap sequence. Two new file-scope volatiles and two new && tokens; no confStruct change, sizeof stays 200, SW_VERSION stays 36, no log format change.
// V2.5-Evo - 2026-10-03 - I2C ENABLE-SWAP STARVATION, STEP 2 of 4 - THE INSTRUMENT (see BREmote_V2_Rx.h, System.ino, Logger.ino): the two enable-swap timeout paths at :75-100 now COUNT. On a timeout the swap does not happen, alternatePWMChannel is deliberately left where it is, the still-enabled motor is re-pulsed and THE OTHER MOTOR RECEIVES NO PULSES AT ALL - the starved VESC then times out and releases while the other holds the rider's commanded throttle, which on a differential drive is uncommanded yaw at power. That path has existed and been correct since 2026-07-22 and was completely silent: three half-days of field work on an asymmetric-thrust failure could not establish whether it had ever fired, because ?printpwm reads PWM0_time / PWM1_time, which calcPWM() computes unconditionally whether a pulse leaves the board or not. ADDED: g_swap_fail_ch0++ / g_swap_fail_ch1++ (saturating at 0xFFFF) in the two else-branches, g_swap_fail_run++ beside them, and g_swap_fail_run = 0 on each successful swap. Read them the right way round - a run of ch0 failures means PWM0 kept its pulses and PWM1 was the starved channel. DIAGNOSTIC ONLY: three counter writes are the whole change; nothing reads them back, g_swap_fail_run is a PURE OBSERVER until STEP 3, and no gate, channel index, cap, ramp or PWM value moves. The pulse-then-2 ms-then-swap ordering is byte-for-byte unchanged. No confStruct change, sizeof stays 200, SW_VERSION stays 36, no log format change.
// V2.5-Evo - 2026-10-01 - M-2 FIX, part 2 of 4 (see BREmote_V2_Rx.h, Logger.ino, System.ino): calcPWM() PUBLISHES the motor gate's verdict into the diagnostic observer g_motor_gate_open. The gate's state had no instrument anywhere: the BIND LED, ?printrssi and the deep log's link flag all read last_packet ("is the remote alive?"), while the gate reads last_control_packet ("do I have a fresh throttle command?"), so a motor gated off displayed as connected with good signal. ?printpwm was worse than useless - it prints PWM0_time / PWM1_time, which calcPWM() computes UNCONDITIONALLY every 10 ms whether the gate lets a pulse out or not. ONE new line here: the local motor_gate_open that the M-3 ramp reset below already computes is copied to the observer, so ?diag, ?printpwm and the logger all read the gate's own verdict rather than each growing a third copy of the expression - there are still exactly TWO copies, the gate in generatePWM() and this mirror, and the warning at both sites still says so. The observer also carries PWM_active, which a bare control-packet age cannot show. DIAGNOSTIC ONLY: nothing reads it back, ramp_target still uses the local bool, and no gate, cap, ramp or PWM value changes - the control path is byte-for-byte identical. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-10-01 - M-3 FIX (the follow-up residual the ramp-placement audit left open, now water-critical by owner request): the throttle rate-limiter is RESET while the motor gate is shut. calcPWM() runs before the gate on every 10 ms pass, so through a link dropout - gate shut, no pulses leaving the board - the ramp kept winding its memory up toward the rider's last pre-dropout throttle byte, and a link that came back with the trigger still held stepped the motors straight to that value instead of easing in over motor_ramp_s. The same snapback a competitor's manual documents ("may jump to 100% if trigger still fully pressed"). FIX: calcPWM() mirrors the gate's own test and hands the ramp a target of 0 while it is shut, so the memory lands on 0 through the helper's existing instant-fall path - its only downward path - and the climb on recovery starts from 0, at motor_ramp_s whenever RTM and FM are inactive. No new state, no second timer (the gate's own failsafe_time already means a closure is never a micro glitch), nothing reaches inside OutputRampState, and Common/OutputRamp.h is not touched at all - so the separate ramp-OFF tracking fix inside the helper (rate 0 = track the target, no mid-session dip) is untouched: that one is about the RATE and lives in the helper, this one is about the GATE and lives in the caller. The gate line itself is unchanged, effective_thr and every cap are unchanged, and the terminal effective_thr == 0 clamp is still the last writer. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-28 - R-3 FIX, part 3 of 3 (see BREmote_V2_Rx.h and Radio.ino): the motor gate in generatePWM() tests last_control_packet instead of last_packet. last_packet is refreshed by the 0xF1 / 0xF2 / 0xF4 meta-packets too, and none of them carries a throttle byte, so after a dropout a meta-packet - most reachably the 30 s Follow-Me keepalive, which fires whenever FM is armed - could reopen this gate on the stale pre-dropout thr_received with the trigger released, and with the ramp already climbed back to it because calcPWM() runs before the gate. The gate now asks "do I have a fresh throttle command?" rather than "is the remote alive?". ONE token changed; everything else here is comment. last_packet and its four other readers are untouched. With a healthy 10 Hz link last_control_packet is refreshed every ~100 ms, so normal operation is byte-for-byte unchanged. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-25 - MANUAL PIVOT ASSIST, FLOOR CORRECTION (review findings P-7 / P-6): kPivotFloorQ8 128 -> 64 in BREmote_V2_Rx.h, because f = 1/2 is the exact reciprocal of the mixer's 2x gain into the outer motor at full lock and therefore cut nothing at full trigger. No code change in this file - the stale 0.50 literals are corrected to 0.25, the ramp-placement paragraph now quotes the real 75 % cut / 0.75 s recovery it would have cost as a pre-ramp cap, and the "byte-for-byte" claim at the mixer call is corrected to "within 8 ticks / 80 ms, monotonically and subtract-only" because the depth bleeds out rather than snapping to 0. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-25 - MANUAL PIVOT ASSIST. WHAT THE OWNER ASKED FOR: "should be natural but for when starting from [low] speed to pivot fast. There are many occasions where the buggy's nose is misaligned and you want to turn in the right direction before you take off." He already does it by hand - easing the trigger back to about half while holding the stick over tightens the turn, and at full throttle the arc is too wide - and steering_influence is already at its maximum 100, so no stored setting can reproduce it. WHAT THIS ADDS: calcPWM() multiplies the ramped throttle by a factor of 1.00 down to kPivotFloorQ8/256 (0.25 as shipped, which puts the outer motor at 126/255 at full trigger instead of the 254/255 an f of 0.50 would have left - see the min(2fT, 255) derivation in the header) while the buggy is slow AND the rider's stick is hard over, and feeds THAT to the differential mixer. SUBTRACT-ONLY BY CONSTRUCTION: the factor is never above 1.0, the result is taken only when it is strictly lower (`if (assisted < pivot_thr)`), and it is applied DOWNSTREAM of every cap and of the ramp, so pivot_thr <= ramped_thr <= effective_thr <= min(rtm_approach_cap, fm_throttle_cap) <= thr_received on every tick - the assist cannot raise a byte, lift a cap, or reach past the ramp's rise limit. MANUAL ONLY: gated off whenever rtm_rx_active || fm_rx_active, because the automatic modes have their own align pivot (fm_align_cap / fm_align_influence via align_mixer_influence_override) and the two must never fight; that machinery is not touched here. DIFF ONLY: gated on steering_type == 1, because the assist exists to change how the differential mixer splits the throttle - on the efoil and servo branches a throttle cut during a turn buys nothing, so they keep reading ramped_thr and their bytes are unchanged. NATURAL, NOT SWITCH-LIKE: two continuous blends (speed, stick deflection) multiply into a DEPTH, the depth is rate-limited to kPivotDepthRiseQ8 / kPivotDepthFallQ8 per tick so it walks rather than steps, and a Schmitt band (kPivotDepthArmQ8 to arm, 0 to release) stops it flickering at the edge of the window. PROVABLY INERT ABOVE THE WINDOW: depth 0 makes the factor exactly 256/256, and (thr * 256) >> 8 == thr for every byte, so fast riding and small stick angles produce the identical bytes they do today. Read 'inert' as WITHIN 80 ms, not on the same tick: the depth BLEEDS OUT over kPivotDepthFallQ8 (8 ticks), so crossing 5 km/h, straightening the stick, or RTM/FM taking over mid-pivot each reach bit-identical output within 80 ms rather than instantly - monotonically and subtract-only the whole way. The genuinely same-tick cases are a non-steering_type-1 board and any steady state outside the windows. SAFETY-NEUTRAL-1 is untouched and still last. All nine tuning numbers are compile-time kPivot* constants in BREmote_V2_Rx.h with their derivations: no confStruct field, so sizeof stays 200, SW_VERSION stays 36, and the owner's stored settings are NOT wiped by this flash.
// V2.5-Evo - 2026-09-24 - MERGE of the takeover side branch into the throttle-ramp move. The two branches rewrote the same block in opposite directions: this one moved the rise-limit AHEAD of the mixer onto the throttle (so steering is never slewed), the side branch kept it after the mixer but gave it TWO rates. Both survive. The ramp step lives in Common/OutputRamp.h as the side branch intended, but the header is rewritten as a SINGLE-CHANNEL 0-255 throttle ramp with the Q12 accumulator (the two-channel microsecond version and its min-1-us floor are gone), and calcPWM() calls it at the pre-mixer site with the side branch's selector: (rtm_rx_active || fm_rx_active) && auto_ramp_s > 0.001 ? auto_ramp_s : motor_ramp_s. The side branch's post-mixer call site is deleted - it would have double-ramped and re-ramped steering. Its stale-state fix is KEPT and is now the reason the ramp-off path is inside the helper: with the ramp off the memory tracks the target instead of going stale, so a ramp switched on mid-session no longer dips. Its steering work is untouched: auto_owner / takeover / auto_cmd, steer_takeover_active and the pivot boost following auto_cmd all merged as written. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-24 - RAMP BEFORE THE MIXER, STEERING IS INSTANT. WHAT WAS WRONG: the rise-limit sat AFTER the differential mixer and rate-limited PWM0_time / PWM1_time, i.e. the two finished MOTOR outputs - and on a differential drive the turn IS the difference between those two outputs, so the block rate-limited the steering as well. Its own comment admitted it ("By design this also ramps the differential-steering response (a sharp turn builds over this time)"). That is the one thing the owner has ruled out repeatedly: the ramp exists for the THROTTLE - a soft tow start that pulls a rider off a shoulder without snatching the rope - while steering must land on the very next 10 ms pass. On the servo branch it was worse: the block was rate-limiting the steering SERVO channel itself, which has no throttle in it at all. FIX: the rise-limit moves UP into the throttle domain (0-255 counts), applied to effective_thr AFTER every cap and BEFORE the mixer, producing ramped_thr. All three branches drive their motor channel(s) from ramped_thr; the mixer computes the turn from ramped_thr with no rate limit of its own, so a stick flick moves both motors on the next pass; the post-mixer block is deleted (keeping it would double-ramp). RISE ONLY - the fall branch snaps onto the target, so trigger release, failsafe, RTM/FM emergency stop and straightening still drop the output on the same tick. motor_ramp_s <= 0.001 still means instant/off. CAPS STILL BOUND THE OUTPUT: the ramp only ever approaches effective_thr from below, so ramped_thr <= effective_thr <= rtm_approach_cap / fm_throttle_cap on every tick - a cap DROP is instant, a cap RISE is slewed, exactly as before - and the pivot boost still only redistributes that permitted throttle. STEP - Q12 FIXED POINT, and why it has to be: the ramp accumulates in 1/4096 of a throttle count (full scale 255 x 4096 = 1044480, a uint32_t), step = full scale / (motor_ramp_s x 100 ticks per second). A whole-count integer step does not work in this domain - 255 counts is four times coarser than the ~1000-count PWM span the old post-mixer ramp used, so a whole-count step turned the owner's 1.00 s setting into 1.28 s and floored every setting from 1.275 s upward at one fixed 2.55 s (4.00 s would have run 36 % FAST, the wrong direction). With Q12 the entire configured 0.20-4.00 s range lands WITHIN ONE 10 ms TICK of nominal and always on the slow side, because truncation is the safe direction and the ramp is never faster than asked: 0.20 s -> 0.20 s; 0.50 s -> 0.51 s; 0.75 s -> 0.76 s; 1.00 s -> 1.01 s; 1.50 s -> 1.51 s; 2.00 s -> 2.01 s; 3.00 s -> 3.01 s; 4.00 s -> 4.01 s. The residual is the single tick it costs to land exactly on target - a flat +10 ms, which is +0 % at the 0.20 s end (20 ticks divide exactly), +2 % worst relative at 0.50 s, and +0.25 % at 4.00 s; measured by brute-force sweep of the whole range in 0.01 s steps, worst case +1 tick, zero cases faster than configured. The old max(1.0f, ...) WHOLE-count floor is gone; a 1-unit (1/4096 count) floor replaces it purely to guarantee forward progress against a corrupt out-of-range config value, and it cannot bite anywhere in the validated range, where the step runs 2611 (4.00 s) to 52224 (0.20 s). One shared ramp also means the two motors can no longer slew at different real rates when one channel's PWM span hits a floor. DEEP LOG: g_motor0_cmd / g_motor1_cmd now carry the ramp and are still exactly what their name says (post-mixer, pre-map motor commands); thr_received_log is still the raw TX byte and g_effective_steer is still the steering byte applied. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - DEEP LOG (C): calcPWM() records the two post-mixer, pre-map motor commands (motor_mix.motor0 / motor1, 0-255 counts) into g_motor0_cmd / g_motor1_cmd - diagnostic observers only, the g_effective_steer pattern: written on every pass, never read back into any control path. The steering_type 1 branch writes the mixer's values; the efoil and servo branches write 0 / 0. No PWM value, cap, gate or ramp is touched. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
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
// ============================================================
// V2.5-Evo - 2026-10-03 - THE ENABLE-SWAP STARVATION STATE, owned entirely by this file
// ============================================================
//
// g_swap_starved is the one control bit STEP 3 adds. true = the enable swap has failed
// kSwapStarveTicks times in a row, so NEITHER channel is pulsed: both VESCs time out together and
// the buggy COASTS, instead of one motor holding the rider's commanded throttle while the other
// has been released. It is read by exactly two CONTROL paths - the pulse test in generatePWM()
// and the gate mirror in calcPWM() - and it is written only here, by the swap outcome.
// V2.5-Evo - 2026-10-03 - STEP 5 (audit M-6): ?diag (System.ino) now also PRINTS it, read-only, on
// the "swap fails" line, because a CLOSED motor gate has three causes and this is the third one.
// That is an observer, not a control path: it prints the flag and sets nothing.
//
// g_swap_ok_run is the recovery counter and is internal to this state machine. Nothing outside
// this file reads it; the failure counters that DO have external readers (?diag, the deep log) are
// g_swap_fail_* in BREmote_V2_Rx.h.
//
// WHY BOTH ARE volatile: same reason as every other observer in this project. Single writer
// (generatePWM, which is also the only caller of calcPWM, so this is one task), single core, and
// volatile is the part that matters - the compiler must not cache them in a register across the
// two functions.
//
// IT IS A SCHMITT TRIGGER, NOT A SWITCH: kSwapStarveTicks (25) failures in to cut, and
// kSwapRecoverTicks (5) successes in a row to clear. Derivations for both are at their
// declarations in BREmote_V2_Rx.h. The short version of why recovery is not "the first successful
// swap": M-3 drives the throttle ramp's target to 0 whenever the motor gate is shut, so a gate
// that FLAPS costs a full re-ramp every time it reopens, and a marginal bus could have cut and
// resumed on alternate ticks and PINNED THE THROTTLE near the bottom of the ramp - the buggy would
// crawl rather than stop, a failure with no clear signature. That is F-1 of the 2026-10-02 M-3
// delta audit, and five consecutive good swaps is what closes it.
//
// NOT A LATCH, DELIBERATELY. A hard latch turns a transient bus glitch into a dead ride, with a
// rider on a rope and no way to restore propulsion without a power cycle he cannot perform on the
// water. That is a worse outcome than the fault being fixed.
//
// ONE HONEST EDGE CASE, STATED RATHER THAN PAPERED OVER: the swap is only attempted while link_ok,
// so if the link drops while starved, the state persists until the link returns and then clears
// 50 ms later. That is harmless - through a link dropout the link gate already holds the motors
// off and the ramp memory is already at 0, so the starved flag is not what is stopping anything,
// and the 50 ms it costs on recovery is inside the soft ramp. It is deliberately NOT cleared on a
// link drop: that would be a second clearing rule for one decision.
// AND A SECOND: ?diagz zeroes g_swap_fail_run, so running it during a failure run restarts the
// count and can delay a trip by up to 250 ms. It is a bench diagnostic typed by hand on the
// serial console and it cannot clear g_swap_starved (only five good swaps do that), so the
// consequence is bounded and acceptable.
volatile bool     g_swap_starved = false;   // true = enable swap starved; BOTH channels stop pulsing (fail symmetric)
volatile uint16_t g_swap_ok_run  = 0;       // consecutive SUCCESSFUL swaps; recovery needs kSwapRecoverTicks of them

// ============================================================
// V2.5-Evo - 2026-10-03 - STEP 5 - awSetEnableDirections(): ONE CHECKED I2C WRITE FOR THE SWAP
// ============================================================
//
// What it does: writes the AW9523's CONFIG1 register (0x05) - the DIRECTION bits for pins 8-15 -
//               so that exactly one of the two PPM enable pins is driven and the other is left
//               high-impedance, which is how this board selects which optocoupler is live.
// Inputs:       cfg1 - the COMPLETE CONFIG1 byte to write, one of the two kAwCfg1Pulse* constants
//               below. THE CALLER MUST ALREADY HOLD i2cMutex: this function does not take it, so
//               the swap's existing 10 ms bounded take stays the only take anywhere on this path.
// Outputs:      true only if the I2C transaction was acknowledged end to end (address, register
//               pointer and data byte). false on NACK, on a Wire.setTimeOut(3) timeout, or on any
//               other bus error - i.e. on exactly the silent failures pinMode() could not report.
// Side effects: one I2C transaction on Wire. No global is written, no delay, no mutex touched.
//
// WHY A RAW REGISTER WRITE AND NOT THE LIBRARY. Adafruit_AW9523::pinMode() returns void, so the bus
// outcome is unreachable - that is the audit's C-1 and the whole reason this function exists. The
// library's configureDirection() does return bool, but it writes CONFIG0 **and** CONFIG1 (two
// transactions, not one) and would therefore rewrite port 0's directions - the BIND button, the
// BIND LED, AP_EN_BMS_MEAS - from the top-priority motor task 100 times a second, and would need a
// second copy of the whole 16-bit direction map to do it, which is a second source of truth on the
// motor path. One CONFIG1 write touches only the register holding the two bits this swap owns.
//
// WHY DROPPING THE LEDMODE WRITE IS SAFE. pinMode() also wrote the LEDMODE bit for the pin on every
// call. startupAW() (System.ino) already puts both enable pins in GPIO mode at boot, and NOTHING in
// this firmware writes LEDMODE again - there is no configureLEDMode() and no aw.analogWrite() call
// anywhere on the RX - so those writes were re-asserting a constant 100 times a second. Dropping
// them is where most of the ~8x bus saving comes from.
//
// WHY THE WRITE IS ABSOLUTE AND NOT A TOGGLE, which is what makes even a MIS-REPORTED outcome safe:
// each constant is the full intended direction byte, so if a transaction ever lands on the part but
// is reported as failed (a timeout during the stop condition, say), the next tick re-asserts the
// identical byte for the same alternatePWMChannel value and the enable/index pairing is restored.
// The worst cost of that rare case is ONE tick - 10 ms, one pulse - of one channel's width on the
// other channel's enable, against the previous behaviour of 50 Hz cross-routing forever with every
// instrument on the board reading healthy.
static const uint8_t kAwCfg1Reg = AW9523_REG_CONFIG0 + 1;  // CONFIG1 = 0x05: direction bits, pins 8-15

// CONFIG1 bit n is pin (n + 8); 1 = INPUT (high-Z), 0 = OUTPUT (driven). The FIXED half of the byte
// is the port-1 direction map startupAW() (System.ino) sets at boot and that no runtime code
// changes: AP_S_AUX (10) and AP_WET_MEAS (15) are inputs; AP_U1_MUX_0 (8), AP_U1_MUX_1 (9),
// AP_L_AUX (11) and AP_EN_WET_MEAS (14) are outputs. (initLogger()'s aw.pinMode(AP_S_AUX,
// INPUT_PULLUP) matches none of the library's three mode branches and is a no-op, so pin 10 keeps
// the INPUT startupAW() gave it either way.) Built from the AP_* defines rather than written as a
// literal so that a pin-map edit in BREmote_V2_Rx.h cannot silently desynchronise this byte.
static const uint8_t kAwCfg1PortInputs = (uint8_t)((1u << (AP_S_AUX - 8)) | (1u << (AP_WET_MEAS - 8)));
// ...and the swap's own two bits: the channel about to be PULSED is driven, the other is high-Z.
static const uint8_t kAwCfg1PulsePwm1  = (uint8_t)(kAwCfg1PortInputs | (1u << (AP_EN_PWM0 - 8)));
static const uint8_t kAwCfg1PulsePwm0  = (uint8_t)(kAwCfg1PortInputs | (1u << (AP_EN_PWM1 - 8)));

static_assert(AP_EN_PWM0 >= 8 && AP_EN_PWM0 <= 15 && AP_EN_PWM1 >= 8 && AP_EN_PWM1 <= 15,
              "Both PPM enables must be AW9523 port-1 pins or they no longer share CONFIG1, and the "
              "single checked write in awSetEnableDirections() would move the wrong pin");

static bool awSetEnableDirections(uint8_t cfg1)
{
  // AW9523_DEFAULT_ADDR is 0x58, the address startupAW() passes to aw.begin().
  Wire.beginTransmission(AW9523_DEFAULT_ADDR);
  Wire.write(kAwCfg1Reg);
  Wire.write(cfg1);
  return (Wire.endTransmission() == 0);  // 0 = address, register and data all ACKed
}

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

    // V2.5-Evo - 2026-09-28 - R-3 FIX: the motor gate tests last_control_packet, NOT last_packet.
    // THE BUG: last_packet is refreshed by the 0xF1 / 0xF2 / 0xF4 meta-packets as well as by the
    // control packet, and none of the three meta-packets carries a throttle byte - thr_received has
    // exactly ONE writer, the control-packet branch in Radio.ino, and nothing anywhere zeroes it on
    // a failsafe. So: rider on the throttle, link drops past failsafe_time, motors cut, rider
    // releases the trigger, and then a meta-packet lands before the next control packet - the
    // Follow-Me 30 s keepalive is the most reachable one, and it fires whenever FM is armed,
    // including armed while the buggy is being towed by hand. That reopened THIS gate on the stale
    // pre-dropout throttle with the trigger released. Note calcPWM() above runs BEFORE the gate, so
    // the ramp has already climbed back to that stale value by the time the gate opens - the motors
    // do not ease in, they are already there.
    // V2.5-Evo - 2026-10-01 - M-3 FIX: that last sentence is no longer true. calcPWM() still runs
    // first, but it now MIRRORS this exact test and feeds the throttle ramp a target of 0 whenever
    // this gate is shut, so the ramp memory is at 0 when the gate reopens and the motors do ease
    // in. The gate itself is unchanged - not one token of the line below moved. If this expression
    // is ever edited, edit the mirror in calcPWM() in the same commit.
    // THE FIX: last_control_packet is stamped only by the control-packet branch, so this gate now
    // asks "do I have a FRESH THROTTLE COMMAND?" instead of "is the remote alive?". Those are two
    // different questions and this line was answering the wrong one. last_packet keeps its liveness
    // meaning and all four of its other readers (Logger.ino, RTMState.ino x2, System.ino) are
    // untouched - liveness is the correct question for every one of them.
    // INVARIANT: with a healthy 10 Hz control link last_control_packet is refreshed every ~100 ms,
    // far inside failsafe_time, so this gate behaves IDENTICALLY to today in normal operation. The
    // only behaviour that changes is the failure case above. Full write-up at the declaration in
    // BREmote_V2_Rx.h.
    // V2.5-Evo - 2026-10-03 - STEP 3: the gate test is HOISTED into link_ok and its MEANING does
    // not move. It is hoisted because the pulse and the swap now need it separately: the pulse must
    // stop while the swap is starved, and the swap must keep being ATTEMPTED so it can recover. The
    // full gate for PULSING is link_ok && !starved, and its mirror in calcPWM() carries the
    // identical pair of terms - there are still exactly TWO copies of that expression and the
    // warning at both sites still says so.
    // V2.5-Evo - 2026-10-03 - STEP 5 (audit L-2): the subexpression below is now CHARACTER-IDENTICAL
    // to its mirror in calcPWM(). The STEP 3 hoist had left this copy as millis()-x while the
    // mirror reads (millis() - x) - identical by precedence, but the entire maintenance rule at
    // both sites is "a human eyeballs the two lines for sameness", and a diff-by-eye that has to
    // forgive one cosmetic difference is a diff-by-eye that will forgive a real one. No behaviour
    // change: the added parentheses only make explicit the precedence the compiler already applied.
    const bool link_ok = (PWM_active && (millis() - last_control_packet) < usrConf.failsafe_time);
    if(link_ok)
    {

      // V2.5-Evo - 2026-07-22 - <audit HIGH> WDT self-preservation on i2cMutex.
      // BUG: this WDT-registered top-priority PWM task took i2cMutex with portMAX_DELAY at the
      // two AW9523 enable-swap sites. i2cMutex is shared with the compass, the AW9523 LED and
      // UART-mux writes, the button reads and the logger;
      // V2.5-Evo - 2026-10-03 - STEP 4: the line above used to say "compass/ADS1115/AW9523-LED/
      // logger". THERE IS NO ADS1115 ON THE RX - it is a TX part (0x48, Display.ino / Analog.ino)
      // and it appears nowhere in the RX firmware or on the RX schematic. The RX's AP_BMS_MEAS is a
      // plain AW9523 pin given pinMode(INPUT) in startupAW() (System.ino) and then never read
      // anywhere, and getUbatLoop() measures the battery with the ESP32's own analogRead() on
      // GPIO 0 - no I2C at all. The stale name cost real time: it sent the 2026-10-03 audit looking
      // for an ADC conversion wait held inside this lock, and there is no such holder on this board.
      // if the I2C bus wedged, this task blocked unbounded and the 3000ms WDT panic-rebooted the
      // RX mid-ride. FIX: bound the take to 10ms and skip the swap on timeout (never block >10ms).
      // CONSISTENCY: the enable swap sets up the enable line for the NEXT iteration's pulse, so
      // alternatePWMChannel is now advanced ONLY when the swap actually succeeds. On timeout the
      // channel index is left unchanged, so next cycle re-pulses the SAME, still-enabled motor
      // (correct VESC) instead of firing the other channel's pulse onto the current enable state.
      // Worst case under bus contention: one motor gets repeated pulses for a few 10ms cycles while
      // the other misses updates — no wrong-motor pulse, no unbounded block, no WDT trip. On timeout
      // we do NOT hold the mutex, so we do NOT give it.
      // V2.5-Evo - 2026-10-03 - STEP 2: COUNT THE FAILED SWAPS. Everything above this line is
      // unchanged; the three counter writes below are the entire change in this file.
      // WHY IT HAD TO BE DONE FIRST: the timeout path above has existed since 2026-07-22 and is
      // correct, but it was SILENT. A swap that fails leaves one motor being re-pulsed and gives
      // the other NOTHING, and nothing anywhere recorded that it had happened - so after three
      // half-days of field work on an asymmetric-thrust failure there was still no way to say
      // whether this mechanism had fired. ?printpwm cannot see it (calcPWM() computes PWM0_time /
      // PWM1_time unconditionally whether a pulse leaves the board or not), and the deep log had
      // no column for it. These counters are that missing observer.
      // DIAGNOSTIC ONLY AT THIS STEP: the increments are the only new statements, nothing reads
      // them back, and no gate, channel index, cap, ramp or PWM value is touched. g_swap_fail_run
      // in particular is a PURE OBSERVER here - it becomes a control input in STEP 3, in its own
      // commit, so that the two changes can be audited and reverted independently.
      // V2.5-Evo - 2026-10-03 - STEP 3: PULSE ONLY WHILE THE SWAP IS HEALTHY. The pulse is lifted
      // out of the two channel branches so this one test covers both; the ternary selects exactly
      // the width each branch used to pulse (channel index 1 -> PWM0_time, 0 -> PWM1_time) and the
      // pulse-then-2 ms-then-swap ordering on every tick that pulses is unchanged. On a starved
      // tick there is no pulse, so there is nothing for an enable change to corrupt.
      if(!g_swap_starved)
      {
        generate_pulse(alternatePWMChannel ? PWM0_time : PWM1_time);
        vTaskDelay(pdMS_TO_TICKS(2));
      }

      // V2.5-Evo - 2026-10-03 - STEP 3: THE SWAP IS ATTEMPTED ON EVERY link_ok TICK, INCLUDING
      // WHILE STARVED. This is not an optimisation, it is the thing that makes recovery possible:
      // if the attempt sat inside the pulse gate, then once the gate closed for starvation the swap
      // would never be retried, the success run could never accumulate, and the "no latch" promise
      // would be an accidental PERMANENT latch - unrecoverable propulsion loss mid-ride. The rate
      // is IDENTICAL to before (the swap already ran once per 10 ms tick whenever link_ok), so this
      // adds no I2C traffic anywhere.
      // V2.5-Evo - 2026-10-03 - STEP 5: THE SWAP CHECKS THE BUS, NOT JUST THE SEMAPHORE. swap_ok
      // is true only if the mutex was obtained AND the AW9523 acknowledged the CONFIG1 write. A bus
      // failure then falls through to the SAME else-branch a semaphore timeout already took, which
      // is the whole point: the state machine below must not be able to tell the two apart, or
      // kSwapStarveTicks / kSwapRecoverTicks would need a second set of rules to stay correct.
      // The mutex is released BEFORE the outcome is acted on, so the hold is still one transaction.
      if(alternatePWMChannel)
      {
        bool swap_ok = false;
        if(xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
          // One transaction: PWM0 high-Z, PWM1 driven - the pairing next tick's PWM1_time needs.
          swap_ok = awSetEnableDirections(kAwCfg1PulsePwm1);
          xSemaphoreGive(i2cMutex);
        }
        if(swap_ok)
        {
          alternatePWMChannel = 0;  // advance only on a VERIFIED enable swap
          g_swap_fail_run = 0;      // STEP 2: the run ends on any successful swap
          // STEP 3 hysteresis: kSwapRecoverTicks consecutive GOOD swaps to resume, not one.
          if(g_swap_ok_run < 0xFFFFU) g_swap_ok_run++;
          if(g_swap_ok_run >= kSwapRecoverTicks) g_swap_starved = false;
        }
        else
        {
          // mutex timeout OR a NACKed/timed-out CONFIG1 write: keep alternatePWMChannel=1 so PWM0
          // (still enabled) re-pulses next cycle. PWM0 keeps its pulses, so PWM1 is the channel
          // being starved on this tick - and that is true of both failure kinds, which is why they
          // share this branch and this counter.
          if(g_swap_fail_ch0 < 0xFFFFU) g_swap_fail_ch0++;   // saturate: a wrapped counter reads as healthy
          if(g_swap_fail_run < 0xFFFFU) g_swap_fail_run++;
          g_swap_ok_run = 0;        // STEP 3: one failure ends the recovery run
          if(g_swap_fail_run >= kSwapStarveTicks) g_swap_starved = true;
        }
      }
      else
      {
        bool swap_ok = false;
        if(xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
          // One transaction: PWM1 high-Z, PWM0 driven - the pairing next tick's PWM0_time needs.
          swap_ok = awSetEnableDirections(kAwCfg1PulsePwm0);
          xSemaphoreGive(i2cMutex);
        }
        if(swap_ok)
        {
          alternatePWMChannel = 1;  // advance only on a VERIFIED enable swap
          g_swap_fail_run = 0;      // STEP 2: the run ends on any successful swap
          if(g_swap_ok_run < 0xFFFFU) g_swap_ok_run++;
          if(g_swap_ok_run >= kSwapRecoverTicks) g_swap_starved = false;
        }
        else
        {
          // mutex timeout OR a NACKed/timed-out CONFIG1 write: keep alternatePWMChannel=0 so PWM1
          // (still enabled) re-pulses next cycle. PWM1 keeps its pulses, so PWM0 is the channel
          // being starved on this tick.
          if(g_swap_fail_ch1 < 0xFFFFU) g_swap_fail_ch1++;
          if(g_swap_fail_run < 0xFFFFU) g_swap_fail_run++;
          g_swap_ok_run = 0;
          if(g_swap_fail_run >= kSwapStarveTicks) g_swap_starved = true;
        }
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

  // -- SAFETY: THROTTLE RAMPING (usrConf.motor_ramp_s / usrConf.auto_ramp_s, seconds) ----------
  // V2.5-Evo - 2026-09-24 - The ramp lives HERE: in the throttle domain, after every cap and before
  // the differential mixer, so it can only ever slow the THROTTLE down. It used to sit after the
  // mixer and rate-limit the two finished motor outputs, and the difference between those two IS the
  // turn, so it slewed steering as well. Owner rule: never ramp steering. The ramp is for a soft tow
  // start; a turn has to be there on the next 10 ms pass. The step itself is in Common/OutputRamp.h
  // (pure, host-tested in Tools/tests/output_ramp_test.cpp): rise <= target, instant fall, continuity
  // across a rate change, pass-through-and-track while off, and a Q12 accumulator so the whole
  // 0.20-4.00 s range is honoured to within one 10 ms tick and is never faster than configured.
  //
  // WHICH RAMP (V2.5-Evo - 2026-09-19 - TWO RAMPS, owner decision 14:00). The slow motor_ramp_s
  // exists for the rider's shoulder on a MANUAL tow start, where the rope is loaded; in the automatic
  // modes the rope is not loaded and a fast ramp is what makes the buggy reactive. The ramp in force
  // this tick is selected on the OWNER flags - rtm_rx_active || fm_rx_active, whoever is CAPPING the
  // throttle - and deliberately NOT on auto_owner below, which also carries the trigger and the stick
  // switch:
  //   - Follow-Me following, FM_RETURN moving and classic RTM ride auto_ramp_s (when it is non-zero;
  //     0 = inherit motor_ramp_s, today's behaviour). A stick takeover rides it too: the steering
  //     byte changes hands, the throttle is still the automatic owner's.
  //   - FM_ARMED-not-engaged towing, FM_STOPPING's hand-back ramp, a HOLD escape, Gate 9's handoff and
  //     the mode-0 steer-cancel un-clamp all ride motor_ramp_s: on every one of those the owner flag
  //     is already false, so every hand-back to the rider is softened by the slow slew (the shoulder
  //     case) - the fast ramp never lands on the rider's own throttle.
  // RULE FOR P2 (not built): an automatic mode with the buggy AHEAD of the rider (front_along_m > 0)
  // uses motor_ramp_s - see auto_ramp_s in the header.
  //
  // RISE ONLY. The helper's else-branch snaps the memory straight onto the target, so every kind of
  // stop - trigger release, failsafe, RTM/FM emergency stop, a cap collapsing to 0, straightening -
  // drops the throttle on the same tick with no slew. That is also the only way the memory can move
  // DOWN, and it always moves it the whole way, so the ramp can neither stall nor jump.
  //
  // WHAT THE MEMORY HOLDS WITH THE TRIGGER RELEASED: exactly 0. effective_thr is 0, so the target is
  // 0 and the fall branch assigns it on the first released tick. Re-engaging always starts a fresh
  // full ramp from 0 - the rider cannot bank ramp credit by feathering the trigger, and there is no
  // stale value to jump to. 0 is the zero-init value too, so the first squeeze after boot behaves
  // like every later one.
  //
  // CAPS STILL BOUND THE OUTPUT: the ramp only approaches effective_thr from below, so
  // ramped_thr <= effective_thr <= rtm_approach_cap / fm_throttle_cap on every tick. A cap DROP is
  // instant, a cap RISE is slewed. Both ramps at 0 = off: the target passes straight through.
  //
  // V2.5-Evo - 2026-10-01 - M-3 FIX: THE RAMP IS RESET WHILE THE MOTOR GATE IS SHUT.
  //
  // WHAT WAS WRONG. calcPWM() runs BEFORE the motor gate in generatePWM() on every 10 ms pass, so
  // during a link dropout - gate shut, no pulses leaving the board - this ramp kept on winding its
  // memory UP toward the last throttle byte it had been given. thr_received has exactly one writer
  // (the control-packet branch in Radio.ino) and nothing zeroes it on a failsafe, so the target it
  // was climbing toward was the rider's pre-dropout command. When the link came back WITH THE
  // TRIGGER STILL HELD, the memory was already at the top: the motors did not ease in over
  // motor_ramp_s, they stepped straight to the commanded value. The soft-start the ramp exists to
  // provide was defeated by exactly the event that needs it most. This is the vendor-documented
  // snapback a competitor's manual warns about ("may jump to 100% if trigger still fully pressed"),
  // so it is a real field failure mode and not a theoretical one. Pre-existing behaviour - it was
  // ruled acceptable once (as the ramp-placement follow-up residual) and the owner has now asked
  // for it closed before his next water session.
  //
  // THE FIX, AND WHY IT IS A TARGET AND NOT A POKE AT THE MEMORY. While the gate is shut the ramp
  // is handed a target of 0 instead of effective_thr. The helper's fall branch is instant and is
  // the ONLY downward path it has, so the memory lands on exactly 0 on the first shut tick and
  // stays there for the rest of the dropout; on recovery the climb starts from 0, which is what a
  // hand-back to the rider should feel like. Nothing reaches inside OutputRampState - the helper
  // keeps its single-writer contract and its host test (Tools/tests/output_ramp_test.cpp) still
  // describes it exactly.
  //
  // THIS IS NOT THE RAMP-OFF TRACKING PATH, AND THE TWO MUST NOT BE CONFLATED. The 2026-09-24
  // merge kept a stale-state fix: with the ramp DISABLED (ramp_s <= 0.001) the helper's else-branch
  // assigns the target every tick so the memory TRACKS it, and a ramp switched on mid-session
  // therefore does not dip. That path is about the RATE being zero and it lives INSIDE the helper,
  // where it decides the update RULE. This fix is about the GATE being shut and it lives OUT HERE,
  // where it decides the TARGET. Two different conditions, two different places, neither touched by
  // the other: with the gate OPEN the target is effective_thr byte-for-byte as before, so ramp-off
  // tracking behaves identically to today; with the gate SHUT both branches of the helper converge
  // on 0, which is the intended reset in either rate setting.
  //
  // NO EXTRA HYSTERESIS ON PURPOSE. The gate only shuts after usrConf.failsafe_time without a
  // control packet (default 1000 ms, validated 100-10000), so by the time it shuts the motors have
  // already been dead for that whole period - a gate closure is a significant event, never a micro
  // glitch, and brief dropouts never shut it at all. A second timer here would be a second source
  // of truth for one decision. (At the 100 ms floor the gate already flaps and already cuts pulses
  // between packets on a 10 Hz link; this reset rides that same gate and adds no new flap case.)
  //
  // IT CANNOT FIRE WITH THE GATE OPEN - no mid-pull dip. The test below is the gate's own test, so
  // the target is 0 only on ticks where generatePWM() emits no pulse. The two are read a few
  // microseconds apart, and both orders of disagreement are safe: if this test sees the gate open
  // and the gate then sees it shut, the ramp advanced but nothing was pulsed; if a control packet
  // lands in between so that this test saw it shut and the gate sees it open, that tick pulses at
  // the ramp's 0 (PWM minimum, the same place the first tick of any squeeze starts from) and the
  // next tick climbs from 0 - the soft start, not a dip, and the motor had been dead for at least
  // failsafe_time anyway. On a healthy 10 Hz link millis() - last_control_packet sits near 100 ms
  // against a 1000 ms failsafe, so mid-pull the reset is simply unreachable.
  //
  // WHICH RAMP ON RECOVERY: motor_ramp_s, the slow manual one, whenever RTM and FM are inactive -
  // the selector above keys on rtm_rx_active || fm_rx_active, and a dropout past failsafe_time is
  // itself what stops both (RTM gate 7 raises rtm_rx_emergency_stop, FM fails eligibility with
  // FM_STOP_LINK), so a recovery into manual control is a hand-back and gets the shoulder-friendly
  // slew. That is deliberate and matches the standing rule that the fast auto_ramp_s never lands on
  // the rider's own throttle.
  //
  // NOT TOUCHED: effective_thr, every cap above it, and the terminal effective_thr == 0 clamp,
  // which is still the last writer of the stopped state. The only values that differ while the gate
  // is shut are ones nobody acts on - PWM0_time / PWM1_time (not pulsed in that branch) and the
  // g_motor0_cmd / g_motor1_cmd log observers and ?printpwm, which now read the minimum instead of a
  // stale command, i.e. they report what the motors are actually being given: nothing.
  static OutputRampState motor_ramp = {0};
  const bool  auto_ramp_owner = (rtm_rx_active || fm_rx_active) && (usrConf.auto_ramp_s > 0.001f);
  const float ramp_s          = auto_ramp_owner ? usrConf.auto_ramp_s : usrConf.motor_ramp_s;
  // Mirror of the motor gate in generatePWM() - keep the two expressions identical if either moves.
  // V2.5-Evo - 2026-10-03 - STEP 3: AND THE SECOND TERM IS WHY THIS IS A GATE TERM AND NOT A
  // SKIPPED PULSE. !g_swap_starved is added here as well as at the pulse site, so the two copies of
  // the expression stay identical - one new && token at each site, and still exactly TWO copies.
  // WHAT GOES WRONG IF THIS LINE IS NOT CHANGED: generatePWM() would stop pulsing while this mirror
  // still reported the gate OPEN, so ramp_target below would keep being handed the rider's live
  // throttle, the ramp memory would keep climbing, and on recovery THE MOTORS WOULD STEP STRAIGHT
  // TO THE COMMANDED VALUE. That is the exact M-3 snapback this file was changed to cure two days
  // ago ("may jump to 100% if trigger still fully pressed"), reintroduced by the safety fix and at
  // the worst possible moment: a rider on a rope, trigger held, buggy that just stopped.
  // WHAT THIS ONE TOKEN BUYS, all of it for free:
  //   - ramp_target goes to 0 while starved, the helper's instant-fall branch lands the memory on
  //     exactly 0, and recovery is a FULL SOFT RAMP FROM 0 over motor_ramp_s - the slow manual
  //     ramp, because RTM and FM are inactive on a hand-back. A ramp, not a step.
  //   - BOTH channels resume together and the channel alternation continues normally, because the
  //     swap never stopped being attempted and alternatePWMChannel is still advanced by it.
  //   - the effective_thr == 0 clamp at the end of this function is untouched and is still the
  //     last writer of the stopped state.
  //   - g_motor_gate_open below publishes 0 while starved, so ?diag's "motor gate" line,
  //     ?printpwm's (GATED ...) marker and the deep log's motor_gate_open column ALL report a
  //     starvation with zero new code. That dividend is the whole point of the M-2 observer, and
  //     it depends on this term being here: a future refactor that moves the starvation test out
  //     of this expression silently blinds all three instruments.
  const bool  motor_gate_open = (PWM_active && (millis() - last_control_packet) < usrConf.failsafe_time) && !g_swap_starved;
  // V2.5-Evo - 2026-10-01 - M-2: publish that verdict for the instruments (g_motor_gate_open,
  // declared in BREmote_V2_Rx.h). ?diag, ?printpwm and the deep log read this ONE value instead of
  // each evaluating the gate test again, which is why adding three readers does not add a third copy
  // of the expression above. DIAGNOSTIC OBSERVER ONLY - the g_effective_steer / g_motor0_cmd
  // pattern: written here on every pass, read back by nothing. ramp_target below still uses the
  // local bool, so the control path is unchanged.
  g_motor_gate_open           = motor_gate_open ? 1 : 0;
  const uint8_t ramp_target   = motor_gate_open ? effective_thr : 0;
  const uint8_t ramped_thr    = throttleRampStep(&motor_ramp, ramp_target, ramp_s, 100.0f);

  // -- MANUAL PIVOT ASSIST (kPivot* constants in BREmote_V2_Rx.h) -------------------------------
  // V2.5-Evo - 2026-09-25 - Slow buggy + hard-over stick = lower the throttle reaching the mixer,
  // which is exactly what the owner does by hand to tighten a standing-start pivot. It produces
  // pivot_thr, and ONLY the steering_type 1 (differential) branch below reads it.
  //
  // WHY IT SITS HERE, AFTER THE RAMP, AND NOT AS A THIRD CAP ABOVE IT. Both placements are
  // subtract-only, so this is a feel decision and it was decided on the EXIT from the pivot:
  //   - As a cap BEFORE the ramp, the cut would enter instantly (the ramp's fall is instant) but
  //     the RECOVERY would be slewed by the ramp. At the owner's motor_ramp_s of 1.00 s, coming
  //     back from the shipped 75 % cut costs 0.75 s of climb - so every pivot would end in three
  //     quarters of a second of sluggish exit, which is worse than having no pivot at all. Worse,
  //     the cut would also drag
  //     the ramp's MEMORY down, so the ramp would then have to re-climb ground it had already won.
  //   - AFTER the ramp, the assist is a pure multiply with no memory of its own in the throttle
  //     domain: the cut lands on the very next 10 ms pass, the ramp's memory keeps tracking the
  //     un-assisted capped throttle and never dips, and the recovery is governed by the assist's
  //     OWN release rate (kPivotDepthFallQ8, 0.08 s full travel) instead of borrowing the tow-start
  //     ramp's. The ramp keeps doing the only job it has - rise-limiting the rider's throttle for a
  //     soft tow start - and the assist owns its own exit. That is the crisp pivot exit he needs.
  // The ramp still BOUNDS the assist: pivot_thr <= ramped_thr, always, so nothing the assist does
  // can outrun the rise limit. And because the fall branch of the ramp is instant, effective_thr
  // 0 still gives ramped_thr 0 and therefore pivot_thr 0 on the same tick - the deadman is
  // untouched, and SAFETY-NEUTRAL-1 below is still the last writer.
  uint8_t pivot_thr = ramped_thr;
  {
    // THE SPEED SOURCE: GPS, and only GPS. Two reasons, and the second is the decisive one.
    //   1. There is no calibrated ERPM-to-km/h conversion on this board. usrConf.vesc_erpm_per_kmh
    //      is 0.0 - the factory default, meaning "not calibrated, skip the Phase C VESC check" -
    //      and the owner's own config backup of 2026-09-25 confirms it is still 0.0. An
    //      ERPM-derived speed would be a guess dressed as a measurement.
    //   2. Even calibrated, ERPM is the WRONG signal for this test. These are propeller drives:
    //      at a standstill with the trigger held the motors spin near their free-running speed
    //      while the buggy is not moving at all. That is the EXACT situation this assist exists
    //      for, and an ERPM-based speed test would read it as "fast" and refuse to engage. ERPM
    //      measures thrust, not ground speed.
    // GPS.ino is concatenated before this file, so these are its own definitions; the externs are
    // written out so the dependency is visible where a reader will look for it (the same habit
    // Logger.ino and V2_Integration_Rx.ino already follow).
    extern float         gps_last_speed_kmh;   // last ACCEPTED GPS speed, km/h (Phase A passed)
    extern unsigned long gps_last_ms;          // millis() of that reading; 0 = none this session
    extern bool          gps_rejected;         // Phase A anti-spoofing has rejected the GPS
    // Plain cross-task reads of 32-bit, naturally-aligned values on a single-core RV32 part - a
    // single load each, which cannot tear. They are inputs to a PERMISSION test whose failure
    // direction is "no assist", so even a one-tick-old value can only make the assist smaller or
    // absent, never larger.

    // THE ASSIST'S ONLY MEMORY: the depth, Q8, 0..256. Zero-init is the inert state, so the first
    // squeeze after boot behaves like every later one (the OutputRampState argument, same reason).
    // A function-static in calcPWM() is safe here for exactly the reason motor_ramp above is: this
    // function has ONE caller, generatePWM(), and therefore one task and no re-entry.
    static uint16_t pivot_depth_q8 = 0;

    uint16_t target_q8 = 0;   // what the two blends ask for this tick; 0 = no assist requested

    // THE FOUR HARD GATES. Every one of them fails toward NO assist, and three of them are the
    // speed source failing: GPS disabled, no accepted reading this session, a reading older than
    // kPivotSpeedMaxAgeMs, or a GPS the anti-spoofing has rejected. In all of those the assist is
    // not computed at all and the depth is walked back to 0 below.
    const bool manual_only = !(rtm_rx_active || fm_rx_active);   // RTM/FM own their own align pivot
    const bool diff_drive  = (usrConf.steering_type == 1);       // the mixer is what this acts on
    const bool speed_ok    = (usrConf.gps_en != 0) && !gps_rejected && (gps_last_ms != 0) &&
                             ((millis() - gps_last_ms) <= kPivotSpeedMaxAgeMs);

    if (manual_only && diff_drive && speed_ok)
    {
      const float kmh = gps_last_speed_kmh;

      // SPEED BLEND, 0..256. Deliberately written as "only if the reading is BELOW the zero point",
      // because a NaN fails every float comparison and therefore lands on 0 - the assist stays
      // inert with no explicit NaN test needed, fail-closed by structure. (A NEGATIVE reading is a
      // different case and is deliberately NOT treated as a fault: it falls into the full-blend
      // branch, i.e. it is read as "stopped", which is the only sensible reading of "slower than
      // zero" and is exactly what 0.0 km/h would have given. TinyGPS++ kmph() cannot return one.)
      uint16_t speed_q8 = 0;
      if (kmh < kPivotSpeedZeroKmh)
      {
        if (kmh <= kPivotSpeedFullKmh)
        {
          speed_q8 = 256;
        }
        else
        {
          speed_q8 = (uint16_t)(256.0f * (kPivotSpeedZeroKmh - kmh) /
                                         (kPivotSpeedZeroKmh - kPivotSpeedFullKmh));
        }
      }

      // STICK BLEND, 0..256, from the RIDER's stick. steering_received is the right byte to read
      // and not effective_steer: inside this gate no autonomous controller is active, so the two
      // are the same value, and naming the rider's own byte is what makes "manual only" textual.
      // 127 is the neutral byte the mixer itself uses, so the deflection here is the same quantity
      // the mixer will turn into its turn term.
      const int sdev = (int)steering_received - 127;
      const int defl = (sdev < 0) ? -sdev : sdev;
      uint16_t steer_q8 = 0;
      if (defl >= (int)kPivotSteerFullCounts)
      {
        steer_q8 = 256;
      }
      else if (defl > (int)kPivotSteerStartCounts)
      {
        steer_q8 = (uint16_t)(((uint32_t)(defl - (int)kPivotSteerStartCounts) * 256UL) /
                              (uint32_t)(kPivotSteerFullCounts - kPivotSteerStartCounts));
      }

      // The two blends MULTIPLY, so either one at zero is a hard zero: a fast buggy gets no assist
      // at any stick angle, and a small stick angle gets none at any speed. That product is what
      // makes requirement "fast and straight stays exactly as it is today" an arithmetic fact
      // rather than a tuning hope.
      target_q8 = (uint16_t)(((uint32_t)speed_q8 * (uint32_t)steer_q8) >> 8);
    }

    // HYSTERESIS - a Schmitt band on the depth's zero. Arm only when the blends ask for at least
    // one full rise step; release all the way to 0. The blends themselves are continuous, so there
    // is no threshold for the OUTPUT to toggle across; what this stops is the assist repeatedly
    // starting and stopping at the very edge of the window on GPS noise or a trembling stick.
    if (pivot_depth_q8 == 0 && target_q8 < kPivotDepthArmQ8) target_q8 = 0;

    // RATE LIMIT - the depth WALKS to its target, it never steps, and it always arrives exactly
    // (the "within one step" case assigns the target rather than overshooting). Coming on is
    // deliberately slower than letting go. Note that every exit path - the gates failing, RTM or FM
    // taking over mid-pivot, the rider straightening - lands here with target 0 and therefore
    // bleeds the assist out over 0.08 s instead of releasing it in one tick.
    if (target_q8 > pivot_depth_q8)
    {
      pivot_depth_q8 = ((uint16_t)(target_q8 - pivot_depth_q8) > kPivotDepthRiseQ8)
                     ? (uint16_t)(pivot_depth_q8 + kPivotDepthRiseQ8)
                     : target_q8;
    }
    else if (target_q8 < pivot_depth_q8)
    {
      pivot_depth_q8 = ((uint16_t)(pivot_depth_q8 - target_q8) > kPivotDepthFallQ8)
                     ? (uint16_t)(pivot_depth_q8 - kPivotDepthFallQ8)
                     : target_q8;
    }

    // THE CUT. factor_q8 = 256 - depth x (256 - floor) / 256, so 256 (x1.00) at depth 0 and
    // kPivotFloorQ8 (x0.25 as shipped) at depth 256. Integer throughout and the shift TRUNCATES,
    // so any rounding error is a slightly deeper cut - the subtract-only direction.
    // At depth 0 the multiply is (thr * 256) >> 8, which is thr for every byte 0-255: the assist is
    // then not "nearly" transparent, it is bit-identical.
    const uint16_t factor_q8 = (uint16_t)(256U -
        (uint16_t)(((uint32_t)pivot_depth_q8 * (uint32_t)(256U - kPivotFloorQ8)) >> 8));
    const uint8_t  assisted  = (uint8_t)(((uint32_t)ramped_thr * (uint32_t)factor_q8) >> 8);

    // SUBTRACT-ONLY, enforced and not merely derived: the assisted value is taken ONLY when it is
    // strictly lower than what the rider's throttle had already been reduced to. Same shape as the
    // rtm_approach_cap / fm_throttle_cap tests above - lowest wins - so even a future arithmetic
    // mistake in factor_q8 cannot raise a single count of throttle.
    if (assisted < pivot_thr) pivot_thr = assisted;

    // DEEP LOG: publish the depth for fm_gate_flags bits 19-22 (see g_pivot_assist_q4 in the
    // header). Round to nearest of 15; the Schmitt band above guarantees the internal depth is
    // either 0 or at least 16, so a running assist can never report itself as inert.
    g_pivot_assist_q4 = (uint8_t)(((uint32_t)pivot_depth_q8 * 15UL + 128UL) / 256UL);
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
    // V2.5-Evo - 2026-09-25 - MANUAL PIVOT ASSIST: the mixer is fed pivot_thr, which is ramped_thr
    // with the assist's subtract-only cut applied (see the assist block above). pivot_thr ==
    // ramped_thr bit for bit whenever the assist is inert, which is: any steady state above the
    // speed window, any steady state below the stick window, and every tick of a session on a board
    // that is not steering_type 1 (there the gate never opens at all). It is NOT bit-identical on the
    // tick a condition CHANGES - including the tick RTM or Follow-Me becomes active - because the
    // depth bleeds out over kPivotDepthFallQ8 rather than snapping to 0, so those cases converge to
    // byte-identical within 8 ticks / 80 ms, monotonically and subtract-only the whole way down. Everything the C-4 note above says still holds with T = pivot_thr, including the
    // stopped state: pivot_thr <= ramped_thr and the assist is a multiply, so T is 0 exactly when
    // effective_thr is 0. This is also the ONLY branch that reads pivot_thr; the efoil and servo
    // branches keep ramped_thr, because a throttle cut during a turn buys them nothing.
    DifferentialMotorMix motor_mix = mixThrottleRelativeDifferential(
        pivot_thr, effective_steer, mix_influence,   // V2.5-Evo - 2026-09-19 - steering_influence, or the align pivot boost
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
