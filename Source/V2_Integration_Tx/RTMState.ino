// V2.5-Evo - 2026-10-09 - mag_mode 4 tap steps the station in ANY armed state (owner ruling 2026-10-09: "the magnet is a
//   shortcut to the manual gesture, so it must work the same way"; audit "magnet station step while armed" M-1, L-2, L-3).
//   fmStepStationFromMagnet() now gates on fm_armed instead of fmIsEngaged(), so a tap steps while waiting, on the rope,
//   parked on auto-return or following, with the same effects as the LEFT 2 s hold (cycleFmModeArmed()): "F<n>" held
//   2 s, 0xF2 with the fresh bit, keepalive + arm timers reset, and NO buzz (the manual step gives none, so the audit's
//   confirm buzz is not added). New fmReturnMovingRaw(): the last raw fm_flags say returning (bits 6 + 1), no freshness
//   or streak test, so a moving auto-return behind a stale link still refuses the tap. Still walks mag_fm_set. Comments
//   on the old "engaged only" gate rewritten. No confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-10-08 - STICKY RETURN CAP, remote backstop (owner design): Gate 1 (max runtime), Gate 2 (TX GPS stale)
//   and the buggy's fault bit (and its fallback) now KEEP the RTM cap in force instead of lifting it to 255, until the raw
//   trigger drops below 26 once (rtm_end_sticky / rtm_cap_hold_sticky, rtmKeptCapReleased()). Arrival, the Q-2 refusal and
//   the ceremony holds keep the < 10 release. "St" + buzz unchanged; no display change. No confStruct change.
// V2.5-Evo - 2026-10-07 - T-6: stop-flush ceiling 1000 -> 1500 ms. T-7: a failed boot-ID NVS write is logged. T-8: a boot
//   ID overwritten in the queue before it was sent is re-armed (send stamp). No confStruct change.
// V2.5-Evo - 2026-10-07 - T-4: fmStepStationFromMagnet() refuses while the buggy confirms an auto-return RETURNING.
// V2.5-Evo - 2026-10-07 - T-9: the toggle-combo Follow-Me disarm (after riding) is ignored while the buggy confirms an
//   auto-return RETURNING (fmIsReturning()); serial line only. No confStruct change.
// V2.5-Evo - 2026-10-07 - T-1: the pre-arm distance refusal and the two squeeze timeouts keep the throttle cap at 0 while
//   the trigger is held (rtm_arrival_cap_hold) until one full release; released -> 255 as before. The 1 s LEFT cancel
//   is unchanged. No confStruct change.
// V2.5-Evo - 2026-10-07 - F4/F5 on the remote (port of P2-d, dropped by the merge): both toggle station steppers wrap
//   1 -> 2 -> 3 -> 4 -> 5 -> 1 (cycleFmMode() pre-throttle branch and cycleFmModeArmed()), the first-arm seed from
//   followme_mode accepts 1-5, and fmNextStationInSet() (magnet tap) masks 0x1F and walks modulo 5, so mag_fm_set bits
//   3 and 4 select the front pair. The 0xF2 encoding (bit 7 on gesture declarations) is unchanged. No confStruct change.
// V2.5-Evo - 2026-10-07 - S-8 follow-up: the first boot ID after a link gap (or at the first fresh link) is a 3-packet
//   burst, the 10 s repeats stay single packets. No confStruct change.
// V2.5-Evo - 2026-10-07 - S-7: fmIsReturning() - Follow-Me engaged (fmIsEngaged(), corroborated) AND fm_flags bit 6, i.e.
//   the buggy confirms auto-return RETURNING. Display only. No confStruct change.
// V2.5-Evo - 2026-10-07 - SOP-041 rule 3 resync: while this remote has Follow-Me armed and the link is fresh, if the
//   buggy reports fm_flags bit 0 (armed) clear on 2+ consecutive arrivals spanning at least 3 s (and starting more than
//   1.5 s after this remote's arm declaration), the remote disarms Follow-Me itself: "St" + the stop buzz (deferred to
//   the end of an active return, as F-1). No confStruct change.
// V2.5-Evo - 2026-10-07 - Q-5: Gate 4 (steer exit) always stands down - steering never ends a manual return (SOP-040);
//   one serial line per run when the stick is pushed. No confStruct change.
// V2.5-Evo - 2026-10-07 - Q-3 / Q-1 / Q-2 / Q-6: the remote ends its manual return when the BUGGY says how it ended:
//   rx_state_flags bit 0 (fault) -> "St" + stop buzz, cap 255; bit 1 (arrived) -> silent "St", cap 0 until one full
//   release; the fm_status bit-1 two-arrival end stays as the fallback (old RX). An RTM the buggy never confirms is
//   re-sent once, then refused ("St" + stop buzz, cap kept until a release), and is drawn as "rn" until confirmed
//   (rtmReturnConfirmed()). A release seen during the 2 s "St" lifts a kept cap at that sample. No confStruct change.
// V2.5-Evo - 2026-10-07 - TX protocol round: S-8 boot ID (txBootIdInit() at power-on, different from the last one kept
//   in NVS, then txBootIdTick() every 10 s on a fresh link), H-1 refresh (rtmRefreshTick(): one 0xF1/0x02 a second
//   while RTM_ACTIVE), M-1 0xF2 bit 7 on the four gesture declarations only. Both new 0xF1 values go into a free
//   queue slot only. No confStruct change.
// V2.5-Evo - 2026-10-07 - P-12: comments only - the single-slot queue notes (R-6 header, fmRequestKeepaliveNow(), the
//   runFmLoop() guard) now say the queue is 2-deep and why the waits are kept.
// V2.5-Evo - 2026-10-07 - defects b / c: during the RTM arm wait "rn" alternates with the arrow (it was overdrawn every
//   100 ms); fmDisarm() and fmSilentDisarm() clear the whole R5 row (no leftover C7-C9 pixels). No confStruct change.
// V2.5-Evo - 2026-10-07 - F-1 / F-7: a Follow-Me fault-stop edge handled during an active return disarms Follow-Me
//   silently at once and defers "St" + the stop buzz to the return's end (rtmDisengage()); a fresh Follow-Me arm clears a
//   stale fm_fault_latched. No confStruct change.
// V2.5-Evo - 2026-10-07 - A-1 (TX part): while RTM_ACTIVE, if the buggy (having confirmed RTM this run) reports fm_status
//   bit 1 OFF on 2 consecutive arrivals, the remote ends its own RTM: silent "St", and per SOP-040 the throttle cap stays
//   in force until the trigger is fully released once, then full manual. Hook left for the buggy's future RTM-fault bit.
//   No confStruct change.
// V2.5-Evo - 2026-10-07 - F-3 / F-8: the R-3 ceremony extension has a hard ceiling (window + COMBO_WINDOW_MS + hold);
//   a trigger release during the unlock animation between the two squeezes is counted. No confStruct change.
// V2.5-Evo - 2026-10-07 - P-6: the 1 s LEFT ceremony cancel needs a push past half of the calibrated left travel
//   (tog_mid -> tog_left), so an ordinary LEFT steer cannot cancel a return. No confStruct change.
// V2.5-Evo - 2026-10-07 - SOP-040 gesture rule: returnGesture() is ignored (serial line only) while Return-To-Me is active
//   on this remote or arming, and whenever the trigger is not fully released. No confStruct change.
// V2.5-Evo - 2026-10-07 - SOP-040: returnGesture() (the toggle RIGHT tap + LEFT hold) now shows "St" + the stop buzz when
//   Return-To-Me cannot start (disabled or GPS off); it was silent. No confStruct change.
// V2.5-Evo - 2026-10-07 - M-3 / F-2: inside the RTM arm ceremony a plain 1 s LEFT hold (trigger held or released)
//   cancels at once to full manual: 0xF1/0, cap 255, "St" + stop buzz. A RIGHT tap first still makes it A1/A0.
//   No magnet cancel. No confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-10-07 - H-1 (TX part): rtmRxStateWatch() re-sends 0xF1/0 while the buggy reports Return-To-Me
//   (fm_status bit 1) and this remote is not running one; rtmFmStopFlush() sends 0xF1/0 + 0xF2/0 and waits >= 400 ms,
//   used by deepSleep() and the lock gesture. No confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-10-07 - R-6: runFmLoop() takes the Follow-Me fault-stop edge from fm_fault_latched (set in Radio.ino on
//   the byte's arrival) instead of comparing telemetry.fm_flags tick to tick, so a fault that rose and fell during the
//   blocking RTM arm ceremony is still handled: Follow-Me disarms, "St", stop buzz (fmDisarm(false)). The handling
//   waits while the burst queue is busy (written when the queue was a single slot; it is 2-deep since the H-1 round
//   and the wait is kept - see runFmLoop()). Since the F-1 round a fault handled during an active return is
//   deferred instead. No confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-10-07 - FM warning-distance haptic REMOVED (owner ruling, minimal-buzz rule): runFmLoop() no longer
//   queues Pattern 8, and its scheduler state is gone. fm_warn_distance_m itself stays - the R5 proximity bar
//   uses it as the distance full-scale. No struct change.
// V2.5-Evo - 2026-10-07 - Station buzz REMOVED (owner ruling, minimal-buzz rule; audit R-4): fmStepStationFromMagnet()
//   no longer queues the N-tap Pattern 11. The F<n> display label is the only station-change confirm.
// V2.5-Evo - 2026-10-07 - R-4: the dead fmToggleRtmEnabledFromMagnet() and fmToggleAutoReturnFromMagnet() are
//   removed, together with their header comment; the A1/A0 invariant they carried now sits on
//   ceremonyCancelForReturnGesture(). rtm_enabled_session has no writer and is documented as such.
// V2.5-Evo - 2026-10-07 - R-3: rtm_arm_window_s itself is unchanged, but once a valid RIGHT tap -> LEFT hold has
//   started inside it, the ceremony keeps waiting until that sequence completes or ends
//   (return_gesture_in_progress, set by returnGestureCeremonyPoll()); no squeeze counts past the window. The
//   ceremony also sets ceremony_toggle_latch (Hall.ino) so a toggle still held when it exits is ignored until
//   released and can never become a gear, station or lock action. No confStruct change, sizeof stays 136,
//   SW_VERSION stays 27.
// V2.5-Evo - 2026-10-07 - R-1: in runDoubleSqueezeArm() a squeeze counts only after the trigger has been seen
//   released (thr_scaled < 10) since the ceremony started, and again between squeeze 1 and squeeze 2. A trigger
//   held through a magnet hold can no longer complete the ceremony on its own. No confStruct change, sizeof
//   stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-10-06 - mag_mode 4 rulings (see Hall.ino): fmStepStationFromMagnet() now returns true when the
//   station actually moved, so runMagGesture() can start its 1 s tap lockout (audit M-1). fmToggleAutoReturnFromMagnet()
//   is no longer called: the 2.5 s magnet hold always starts the manual Return-To-Me. Its body is unchanged. No
//   confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-10-06 - F-label hold: every "F<n>" confirm (cycleFmMode() arm + pre-throttle cycle,
//   cycleFmModeArmed(), fmStepStationFromMagnet()) now calls showFmLabelHeld() (Display.ino): held 2 s and
//   NON-blocking. Was a blocking gpsKeepAliveDelay() of 2 s (toggle paths) or 1.2 s (magnet tap). The 0xF2 in
//   the two cycleFmMode() paths now goes out immediately instead of after the 2 s hold. No confStruct change,
//   sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-09-30 - MagFix (delta audit of 98fb7a8), four changes here:
//   1. RAM-ONLY ENFORCED. fmToggleRtmEnabledFromMagnet() no longer writes usrConf.rtm_enabled - it writes the new
//      RAM rtm_enabled_session, and every gate now asks rtmEnabledEffective(). `?save` and the web-UI save persist
//      the live usrConf wholesale, so the old code let a session flip become permanent at the next save, which is
//      the opposite of the owner's instruction that the magnet is temporary and SPIFFS is deliberate.
//   2. THE TOW GATE IS CORROBORATED. fmIsEngaged() now needs FM_FLAG_ARMED as well as FM_FLAG_ENGAGED, and needs
//      them on 2 consecutive arrivals of the fm_flags byte (fm_engaged_streak, counted in Radio.ino). The rule
//      "never reposition the buggy while the rider is on the rope" no longer rests on one CRC8-protected bit.
//      Distance corroboration via rtm_distance was considered and rejected - see the note on the function.
//   3. THE STATION FLASH IS 1.2 s AND CLAMPED. It was usrConf.gear_display_time, a user field with a 65535 ms
//      ceiling, which meant a readable gear flash bought a multi-second loop() stall on every station change.
//   4. RTM-OFF NO LONGER USES THE STOP BUZZ. Pattern 7 means "a fault stopped the system" and preempts everything;
//      a deliberate two-state confirm must not borrow it. OFF is now Pattern 12, three firm taps.
//   Comments, RAM state and one new haptic pattern: no confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-09-30 - MagStations (mag_mode 4): fmIsEngaged() is the new FM-actively-following predicate and the
//   safety gate for the magnet tap; fmNextStationInSet() resolves the next station inside the mag_fm_set bitmask;
//   fmStepStationFromMagnet() performs the move (0xF2 + N-tap Pattern 11 + F<n> flash; the flash was
//   gear_display_time here, now a clamped 1.2 s — see the 2026-09-30 MagFix entry above) and
//   returns SILENTLY when the tap resolves to the station the buggy is already at — a tap either moves the buggy or
//   does nothing at all. fmToggleRtmEnabledFromMagnet() flips the Return-To-Me enable off-throttle only, for the
//   session only (it wrote usrConf.rtm_enabled here and now writes the RAM rtm_enabled_session instead).
//   cycleFmMode(), cycleFmModeArmed(), fmDisarm() and every RTM path are UNCHANGED. No confStruct size change:
//   mag_fm_set took the 2 tail padding bytes, so sizeof stays 136 and SW_VERSION stays 27.
// V2.5-Evo - 2026-09-19 - F0 REMOVED FROM THE PRE-THROTTLE CYCLE (owner ruling 12:45, supersedes the
//   "F0 landing is kept" call in the entry below). Bench testing today: the tap+hold combo cycling
//   stations before any throttle ran 1 -> 2 -> 3 -> 0 (disarm) and landed on it twice in five minutes
//   — F0 had no purpose as a cycle-stop once the disarm gesture exists. cycleFmMode()'s pre-throttle
//   branch now wraps 1 -> 2 -> 3 -> 1, the same wrap cycleFmModeArmed() already used, and can no
//   longer land on F0; the F0-disarm branch it used to take is gone (unreachable). To leave
//   Follow-Me off, don't arm it; the after-throttle disarm gesture and the magnet toggle disarm
//   exactly as before. Diagnostic only, no behaviour change: returnGestureCeremonyPoll() now prints
//   one RETURN [TX] line per detector edge (arming hold released, RIGHT tap registered, LEFT hold
//   started, gesture fired) so the next USB check of the in-ceremony second gesture is conclusive —
//   detector timing, thresholds and ordering are untouched. No confStruct change, sizeof stays 136,
//   SW_VERSION stays 27.
// V2.5-Evo - 2026-09-19 - GATE 4 FOLLOWS THE BUGGY (stick during auto-steer). The classic-RTM steer-exit no longer reads
//   usrConf.rtm_steer_exit_on_input; it reads the buggy's steer_during_auto echo, telemetry.fm_flags bit 4
//   (FM_FLAG_STEER_TAKEOVER), under the same FM_LINK_HEALTHY_MS window as the other flags. Bit clear, or link stale, or an old
//   RX that never sets it -> a push > 20 counts exits RTM exactly as before (setRtmDisarmed(), silent, "St"). Bit set -> the
//   buggy is letting the stick steer the return and the remote must not end it: the gate stands down, and says so once per
//   RTM run (rtm_gate4_takeover_printed, cleared at every RTM_ACTIVE entry). Gates 1-3 untouched. The one delta for a remote
//   that stored rtm_steer_exit_on_input 0: it now exits on a push when the buggy says cancel (none known; the owner's is 1).
//   TX struct untouched (136, SW27).
// V2.5-Evo - 2026-09-19 - THE RETURN GESTURE (RIGHT tap + LEFT hold, the RTM arm combo) is now returnGesture(), a three-state
//   machine read at the instant the hold completes: (1) no override and RTM not armed -> arm RTM exactly as before (setRtmArmed(),
//   ceremony, gates, rtm_arm_window_s untouched); (2) RTM armed and still inside the arm window (the blocking ceremony is waiting for
//   the squeeze) -> the ceremony's own wait loops poll for the same gesture, CANCEL the arm (0xF1/0, TX -> IDLE) and set the
//   auto-return override to the OPPOSITE of the value the buggy echoes in fm_flags bit 7 (last_fm_return_mode = !echo), Pattern 9,
//   "A1" (ON) / "A0" (OFF) for 2 s; (3) an override standing (either value) -> back to default (last_fm_return_mode = 0xFF), Pattern 10.
//   last_fm_return_mode (RAM, 0xFF = none, like last_fm_mode) is encoded into bits 5-6 of EVERY 0xF2 the remote sends
//   (fmEncodeModeByte(): cycleFmMode(), cycleFmModeArmed(), the 30 s keepalive, every 0xF2/0 disarm burst) and cleared on FM disarm
//   and power-up; a flip while armed asks the keepalive to go out now (fmRequestKeepaliveNow()) instead of in 30 s.
//   cycleFmModeArmed() now WRAPS 1 -> 2 -> 3 -> 1 and never lands on F0 (owner rule: nothing disarms Follow-Me deliberately except
//   the disarm gesture); cycleFmMode()'s pre-throttle F0 landing was kept at the time this line was
//   written — superseded later the same day, see the entry above: it also wraps without F0 now.
//   Unconditional Serial prints (the remote printed nothing
//   for any RTM/FM state change before) at the RTM ceremony timeouts, the distance reject, the activation, and at every 0xF2/0
//   call site with its reason. No confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-09-18 - KEEPALIVE vs BURST QUEUE (code-review finding, water-test blocker): the 30 s 0xF2 keepalive in runFmLoop() now
//   queues only when rtm_meta_count == 0 (the single-slot queueMetaPacketBurst() is empty), otherwise retries next tick without
//   advancing fm_last_sync_ms - it used to overwrite the 0xF1/1 RTM-activation burst on the first tick after the blocking arm
//   ceremony, leaving the RX unaware RTM was on. Belt: runDoubleSqueezeArm() refreshes fm_last_sync_ms on a successful arm.
//   No confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-09-18 - RTM no longer disarms Follow-Me (owner decision 27(c), always-armed): setRtmArmed() no longer calls
//   fmSilentDisarm() and sends no 0xF2/0 - fm_armed stays true, the 30 s keepalive keeps running, last_fm_mode is untouched, the RTM
//   ceremony is unchanged. The RX yields on its own side while rtm_rx_active is set and resumes ARMED (unlatched) when RTM ends.
//   The warning-distance haptic in runFmLoop() is gated on !rtm_tx_active so a return does not buzz the FM warning all the way in.
//   No confStruct change, sizeof stays 136, SW_VERSION stays 27.
// V2.5-Evo - 2026-09-17 - WarnDist: runFmLoop() now drives the FM warning-distance haptic (Pattern 8, one 300 ms pulse,
//   repeated every 2 s) while TX armed AND RX FM_FLAG_ARMED AND link fresh AND decoded rtm_distance >= fm_warn_distance_m.
//   No throttle gate. Never overwrites a playing pattern or a pending STOP. Pure logic in Common/FollowMeDistanceWarning.h.
// V2.5-Evo - 2026-09-17 - ArmTimeout: fm_arm_window_s → fm_arm_timeout_s; the arm-window auto-disarm in runFmLoop() now
//   runs only when usrConf.fm_arm_timeout_s > 0 (0 = never, the new default). fm_throttle_seen is unchanged.
// V2.5-Evo - 2026-09-17 - Gate1-REMOVED (audit A2-TX, owner decision 2026-09-14): the TX 30 s throttle-release
//   disarm (kFmGate1ReleaseMs) is gone. Follow-Me is meant to stay armed for the whole session; the RX 10 s
//   latch clear and the RX 95 s mode-age expiry remain the backstops. The 30 s 0xF2 keepalive is unchanged.
// V2.5-Evo - 2026-04-25 - P7: TX RTM and FM state machines.
// RTM: left-hold gesture → arm → squeeze(s) → active → cooldown → idle
// FM:  right-hold gesture → cycle FM mode 0→1→2→3→0 → send 0xF2 meta-packet
// V2.5-Evo - 2026-04-27 - P8: setRtmArmed shows "rn" ×2 (static, 3s total); showFmMode shows F0-F3;
//   added setRtmDisarmed(); steer-exit gate in ACTIVE; rtm_max_runtime_s=0 disables runtime gate
// V2.5-Evo - 2026-04-27 - fix: extern declaration for current_vib_pattern (defined in System.ino)
// V2.5-Evo - 2026-04-27 - P8.1 Bug 2 fix: FM mode display uses scroll3Digits("FM[n]") — digit "1" as second
//   character of displayDigits() renders as a barely-visible horizontal bar, so all modes looked like "F"
// V2.5-Evo - 2026-04-28 - P9: Bug1B pre-arm check; Bug1D all-exit Pattern4/StP; S2 FM full-screen confirms
// V2.5-Evo - 2026-04-28 - P9 S4: rtm_arm_dist_m captured at engage; reset on disengage (R5 proximity bar)
// V2.5-Evo - 2026-04-28 - Chg5: runDoubleSqueezeArm() blocking double-squeeze ceremony; "A rM"→"A r"; "St P"→"St"
// V2.5-Evo - 2026-04-28 - ChgB/C/D/E: SPIFFS seed on first arm; cycle 1→2→3→1 (skip F0); "FM" confirm; 30s keepalive
// V2.5-Evo - 2026-04-28 - ChgDZ: persistent "r n" blinks use displayDigitZone() to preserve R5 proximity bar
// V2.5-Evo - 2026-04-28 - Bug2: setRtmArmed() clears fm_armed — RTM and FM are mutually exclusive
// V2.5-Evo - 2026-04-28 - Bug3: rtmDisengage() clears displayBuffer[6] (R5) to prevent FM phantom pixel
// V2.5-Evo - 2026-04-28 - Bug4: runDoubleSqueezeArm() rewritten — handles single+double squeeze; removes "A r"; RTM_ARMED dead code
// V2.5-Evo - 2026-04-28 - Task2: fmSilentDisarm() for arm-window expiry; cycleFmMode() cycles on armed+no-throttle; "F n" display
// V2.5-Evo - 2026-04-28 - Task3: bobbing advanceArrow() in RTM arm wait loops; delay(250) after unlockAnimation(); Pattern4 after animation; clear on timeout
// V2.5-Evo - 2026-04-28 - TaskA: rtm_arm_gps_timeout_override — 4× GPS staleness threshold during blocking arm ceremony;
//   cleared by rtmDisengage() and ceremony timeout paths; Gate 2 reads it via ternary.
// V2.5-Evo - 2026-04-29 - Fix 1-4: setRtmArmed() calls fmSilentDisarm() so RX
//   receives 0xF2/0 when RTM preempts FM — prevents stale fm_mode_runtime on RX
//   TODO: remove when runDoubleSqueezeArm() is refactored to non-blocking.
// V2.5-Evo - 2026-04-29 - Fix 4-3: fm_armed declared volatile (read by a task / written by loop())
// V2.5-Evo - 2026-04-29 - Fix 2-1: pre-arm rejection path now clears rtm_arm_gps_timeout_override
// V2.5-Evo - 2026-04-29 - F0: FM cycle extended to 1→2→3→0; landing on 0 disarms FM (RAM-only hand-off mode)
// V2.5-Evo - 2026-04-29 - Display: F0-F3 confirms and FM arm confirm now use large-font
//   displayDigits(LET_F, mode) instead of showFullScreenMessage() compact font
// V2.5-Evo - 2026-04-29 - Display: "St" confirm now uses large-font displayDigits(LET_S, LET_T)
//   in all three call sites (rtmDisengage, runDoubleSqueezeArm rejection, fmInternalDisarm)
// V2.5-Evo - 2026-04-29 - Bug fix: RTM arm always failed — two root causes:
//   BugA: decodeRtmDistanceM() returned 0.0m for zero-init telemetry (d==0x00 now → -1.0f)
//   BugB: GPS age exceeded 4× override on double-squeeze; drain Serial1 before RTM_ACTIVE
// V2.5-Evo - 2026-04-29 - GPS dot fix: gpsKeepAliveDelay() replaces bare delay() in
//   ceremony and confirms; drains Serial1 to keep gps_tx.location.age() fresh and
//   prevent GPS dot blinking during blocking display holds.
// V2.5-Evo - 2026-05-02 - Gate 3 throttle-release timeout reduced 10000→4000ms (10s was too long for tow buggy field use)
// V2.5-Evo - 2026-05-10 - SAFETY FIX: zero rtm_thr_cap_tx during arm ceremony to prevent motor runaway (see setRtmArmed)
// V2.5-Evo - 2026-07-20 - T1: Gate 1 throttle-release disarm is now the named constant
//   kFmGate1ReleaseMs, raised 3000 -> 30000 ms (whip gap is 10-25 s off-trigger). Threshold and dead band unchanged.
// V2.5-Evo - 2026-07-20 - StopFeel: every STOP/DISARM confirm now fires Pattern 7 (one 400ms long pulse)
//   instead of Pattern 4 (two 80ms taps), so arm and disarm feel different by touch. Changed sites:
//   rtmDisengage() (RTM disengage), fmDisarm() (FM disarm), the runDoubleSqueezeArm() pre-arm "St"
//   rejection, and both F0-disarm paths (cycleFmMode / cycleFmModeArmed). ARM confirms are UNCHANGED and
//   stay Pattern 4: setRtmArmed(), the two runDoubleSqueezeArm() squeeze confirms, and the cycleFmMode()
//   arm path. fmSilentDisarm() stays silent (arm-window expiry is not a commanded stop).
// V2.5-Evo - 2026-08-17 - StopBuzz FIX: the 2026-08-16 haptic cut was applied inside rtmDisengage()
//   and fmDisarm() — the SHARED sinks — so it also silenced every stop the rider did NOT ask for
//   (RTM Gate 1 max-runtime, Gate 2 GPS-stale, Gate 3 throttle-release, FM Gate 1 release backstop,
//   FM RX fault-stop). That is the exact inverse of the intent. Both functions now take a
//   `commanded` flag: true = the rider asked for it (stay silent), false = a safety gate stopped
//   the system (fire the long STOP buzz). Every call site is classified explicitly below.
//   All Pattern 7 requests now go through vib_stop_pending (System.ino) so a stop buzz can never be
//   swallowed by a pattern that happens to be mid-play.
// V2.5-Evo - 2026-08-17 - StopBuzz REVISION (supersedes the classification in the entry above):
//   the rule is now A PURE TIMEOUT IS SILENT, A FAULT BUZZES. On a wave the rider has no attention
//   to spare for decoding a buzz, and the more buzzes there are the less each one is read, so
//   Pattern 7 is spent on faults only. Moved to commanded = true (SILENT): FM Gate 1 (throttle
//   released 30s) and RTM Gate 3 (throttle released 4s) - both can only fire BECAUSE the trigger
//   was already released, so nothing steps under the rider and the display already says "St".
//   Still commanded = false (BUZZ): RTM Gate 2 (TX GPS lost) and the FM RX fault-stop - faults, not
//   timers - plus RTM Gate 1 (max runtime), the one timer that can fire MID-SQUEEZE, where
//   restoring rtm_thr_cap_tx to 255 un-clamps Throttle.ino and hands the rider raw manual throttle
//   with no gesture behind it. The two arm REFUSALS (RTM pre-arm distance, FM fundamental reject)
//   are unchanged and still buzz.

extern volatile uint8_t current_vib_pattern;
// (V2.5-Evo - 2026-10-07 - the vib_pulse_count extern for Pattern 11 was removed with the station buzz.)
extern volatile bool    vib_stop_pending;   // set true to REQUEST the Pattern 7 STOP buzz (defined in
                                            // System.ino). Never write current_vib_pattern = 7 directly:
                                            // the flag is what makes the stop buzz preempt and survive.
extern float rtm_arm_dist_m;  // defined in BREmote_V2_Tx.h — captured at RTM engage moment

// ---- GPS-aware blocking delay ----
// Replaces bare delay() calls in the arm ceremony and mode confirms.
// Drains Serial1 in 10ms chunks so gps_tx.location.age() stays fresh.
// Prevents the GPS dot from blinking during blocking display animations.
// Safe to call from loop() only — gps_tx.encode() must not be
// called concurrently from core 0.
static void gpsKeepAliveDelay(uint32_t ms)
{
  unsigned long start = millis();
  while (millis() - start < ms)
  {
    while (Serial1.available()) gps_tx.encode(Serial1.read());
    delay(10);
  }
}

// ceremonyDelaySeeRelease - V2.5-Evo - 2026-10-07 - R-1: gpsKeepAliveDelay() that also watches the trigger.
// Used only for the two pauses between squeeze 1 and squeeze 2 in runDoubleSqueezeArm(), so a release that
// happens while the ceremony is pausing is not missed.
// Inputs:  ms - how long to wait. Reads thr_scaled (written by the 10 ms measBufCalc task).
// Returns: true if the trigger read released (thr_scaled < 10) at any 10 ms sample during the wait.
// Side effects: same as gpsKeepAliveDelay() (drains Serial1 into gps_tx). Loop task only. Blocks for ms.
static bool ceremonyDelaySeeRelease(uint32_t ms)
{
  bool seen = false;
  unsigned long start = millis();
  while (millis() - start < ms)
  {
    while (Serial1.available()) gps_tx.encode(Serial1.read());
    if (thr_scaled < 10) seen = true;
    delay(10);
  }
  return seen;
}

// ============================================================
// RTM STATE MACHINE
// ============================================================

typedef enum { RTM_IDLE, RTM_ARMED, RTM_SQUEEZE_WAIT, RTM_ACTIVE, RTM_COOLDOWN } RtmTxState;

static RtmTxState  rtm_tx_state        = RTM_IDLE;

// V2.5-Evo - 2026-06-05 - C-1: 2nd independent throttle gate. True only during the blocking
// RTM arm ceremony (RTM_ARMED), cleared on every exit (→RTM_ACTIVE success / →RTM_IDLE fail).
// sendData() reads this to hard-zero the throttle byte regardless of rtm_thr_cap_tx.
bool rtmIsArming() { return rtm_tx_state == RTM_ARMED; }
static unsigned long rtm_arm_start_ms  = 0;   // when ARMED state was entered
static unsigned long rtm_active_start_ms = 0; // when ACTIVE state was entered
static unsigned long rtm_release_ms    = 0;   // when throttle was last released during ACTIVE
static unsigned long rtm_cooldown_ms   = 0;   // when COOLDOWN state was entered
static unsigned long rtm_squeeze_ms    = 0;   // when SQUEEZE_WAIT was entered
// File-scope so setRtmArmed() can reset it. Inside a switch case it can't be reached
// externally; stale value after arm-window timeout would skip the 500ms hold check.
static unsigned long rtm_hold_start    = 0;   // single-mode: when thr_scaled first crossed 30%

// Temporary 4× multiplier on the GPS staleness threshold used by Gate 2 in runRtmLoop().
// runDoubleSqueezeArm() blocks loop() for up to rtm_arm_window_s seconds, freezing GPS
// polling. Without this, Gate 2 fires immediately on the first runRtmLoop() call after
// the ceremony because GPS age >> rtm_gps_timeout_ms. Set at ceremony start; cleared by
// rtmDisengage() (covers all RTM_ACTIVE exit paths) and the two ceremony timeout returns.
// TODO: remove when arm ceremony is refactored to non-blocking.
static uint32_t rtm_arm_gps_timeout_override = 0;

// V2.5-Evo - 2026-09-19 - Gate 4 prints once per RTM run when it stands down because the buggy
// says the stick takes over (fm_flags bit 4). Cleared at every RTM_ACTIVE entry.
static bool rtm_gate4_takeover_printed = false;

// ============================================================
// V2.5-Evo - 2026-10-07 - A-1 + SOP-040 "ARRIVAL HAND-BACK KEEPS A THROTTLE CAP"
// When the buggy ends the return itself (A-1, see rtmBuggyEndedCheck()), the MODE ends on the remote at once but
// the throttle cap that was in force stays in force until the trigger has been seen FULLY RELEASED once
// (triggerReleased()); only then is throttle uncapped manual. A rider still at full trigger when the buggy arrives
// cannot drive it into himself.
//   rtm_end_keep_cap     - set by the A-1 path immediately before rtmDisengage(): "keep rtm_thr_cap_tx as it is".
//                          Consumed (cleared) by rtmDisengage().
//   rtm_arrival_cap_hold - true while that kept cap is waiting for the release. Cleared by the release
//                          (rtmArrivalCapUpdate() or the "St" hold in rtmDisengage()) or by a new arm.
// Loop task only.
// ============================================================
static bool rtm_end_keep_cap     = false;
static bool rtm_arrival_cap_hold = false;
// V2.5-Evo - 2026-10-07 - Q-1: WHICH cap is kept. THE BUG: an arrival kept the ramp cap (30-70 %), so a rider still at
// full trigger got up to 70 % with the buggy a few metres away and pointing at him. THE FIX: the ending path sets the
// cap to keep here - 0 after an arrival (the buggy holds its own near-zero approach cap too), the cap in force after a
// Q-2 refusal - and rtmDisengage() applies min(current cap, this), so a kept cap can only ever go DOWN. Loop task only.
static uint8_t rtm_end_cap_value = 0;
// ============================================================
// V2.5-Evo - 2026-10-08 - THE STICKY RETURN CAP, REMOTE BACKSTOP (owner design: a return that ends early keeps its limit)
// When Return-To-Me ends EARLY - not by arrival - the remote keeps its RTM cap in force (the 30-70 % ramp cap) instead of
// lifting it to 255, until the RAW trigger drops below kRtmStickyReleaseRaw (26 of 255, 10.2 % of travel) once. The
// buggy keeps the real limit itself (its sticky return cap: governor, slow-down near the rider); this is the backstop
// for the governor's one weak moment (standing still) and for a buggy on older firmware. Why 26 and not the arrival
// release (< 10): the owner's rule is "below 10 % or released, full manual", and the buggy clears at 25 counts of its
// own byte (audit D-6 - the two bytes differ after the expo curve and the gear; the rider only ever gets the lower cap).
//   rtm_end_sticky      - set by an early-ending path together with rtm_end_keep_cap; consumed by rtmDisengage().
//   rtm_cap_hold_sticky - the kept cap standing is a sticky one: it clears at raw < 26, not at triggerReleased().
// Used by: Gate 1 (max runtime), Gate 2 (this remote's GPS stale), the buggy's fault bit and its fallback. Arrival and
// the Q-2 refusal keep the arrival rule. Loop task only.
// ============================================================
static const uint8_t kRtmStickyReleaseRaw = 26;
static bool rtm_end_sticky      = false;
static bool rtm_cap_hold_sticky = false;

// rtmKeptCapReleased - V2.5-Evo - 2026-10-08 - has the rider released enough to lift the kept cap? A sticky cap: the
// raw trigger below kRtmStickyReleaseRaw; any other kept cap: triggerReleased() (< 10) as before.
// Inputs: rtm_cap_hold_sticky, thr_scaled. Output: true = lift it. No side effects.
static bool rtmKeptCapReleased()
{
  return rtm_cap_hold_sticky ? (thr_scaled < kRtmStickyReleaseRaw) : triggerReleased();
}

// ============================================================
// V2.5-Evo - 2026-10-07 - Q-2: AN RTM THE BUGGY NEVER CONFIRMS
// THE BUG: if the buggy never heard 0xF1/1 (all three packets lost, RTM disabled on the buggy) it never reports RTM
// in fm_status bit 1, so the A-1 end could never fire: the remote showed the RTM screen and kept its cap while the
// buggy was in plain manual - a silent non-start that looked like a return (SOP-041 rules 1-2).
// THE FIX: count the fm_status ARRIVALS that land more than kRtmConfirmMarginMs after the last 0xF1/1 packet went
// out (and only once that burst has drained). The first two with bit 1 clear, and none set: re-send 0xF1/1 once.
// The next two still clear: end it as a REFUSAL - "St" + the stop buzz, the cap in force kept until one full release.
// Until the first confirmed arrival the screen shows "rn" (waiting), never the RTM distance screen.
// Loop task only. Reset at every RTM_ACTIVE entry (rtmUnconfirmedReset()).
// ============================================================
static const unsigned long kRtmConfirmMarginMs = 300UL;
static unsigned long rtm_q2_last_arrival_ms = 0;   // fm_status arrival already looked at
static uint8_t       rtm_q2_clear_count     = 0;   // clear arrivals counted since the start or the re-send
static bool          rtm_q2_requeued        = false;

// ============================================================
// V2.5-Evo - 2026-10-07 - F-1: A FOLLOW-ME FAULT THAT ARRIVES DURING AN ACTIVE RETURN IS ANNOUNCED AT THE RETURN'S END
// THE BUG: runFmLoop() handled a Follow-Me fault-stop edge with fmDisarm(false): "St" + the stop buzz and a 2 s
// BLOCKING hold. Handled while RTM was ACTIVE (typically ~300 ms after the ceremony's 0xF1/1), it showed "St" on
// top of a working return - a rider reads "stopped" when the buggy is coming back - and froze Gates 3/4 and the
// ramp cap for 2 s. THE FIX: while RTM is active only the SILENT half runs at once (fmSilentDisarm(): fm_armed
// false, keepalive stopped, 0xF2/0 queued - the 2-deep queue never lets it evict the 0xF1 burst), and this flag is
// set. Every RTM end goes through rtmDisengage(), which then upgrades its "St" to carry the stop buzz and clears
// the flag: ONE "St", ONE buzz, and the screen afterwards is the true state (normal screen, Follow-Me is off).
// If RTM ends on its own fault the buzz fires anyway and the flag is simply cleared. A fault handled when no
// return is active (including a ceremony that ended without going ACTIVE) takes today's fmDisarm(false) path.
// Loop task only.
// ============================================================
static bool fm_fault_deferred = false;

// FM session-init and keepalive state (Changes B + E)
static bool          fm_session_init_done = false;  // Change B: true once last_fm_mode seeded from SPIFFS this session
static unsigned long fm_last_sync_ms      = 0;      // Change E: millis() of last 0xF2 keepalive; 0 when FM disarmed

// ============================================================
// V2.5-Evo - 2026-09-19 - AUTO-RETURN SESSION OVERRIDE (the followme_mode pattern, RAM only)
//
// last_fm_return_mode: this remote's override of the buggy's stored fm_return_mode for THIS session.
//   0xFF = none (the buggy uses its own stored default), 0 = OFF, 1 = ON. Set and cleared only by
//   returnGesture(); cleared on every FM disarm and lost on power-up. It rides in bits 5-6 of every
//   0xF2 the remote sends (00 none, 01 OFF, 10 ON), so a lost packet is repaired by the next 30 s
//   keepalive and a remote power cycle returns the buggy to its stored default within one.
// Declared HERE, ahead of the RTM ceremony, because the ceremony's wait loops are where the second
// gesture state lands (see returnGestureCeremonyPoll()).
// ============================================================
static uint8_t       last_fm_return_mode  = 0xFF;   // 0xFF none, 0 OFF, 1 ON; RAM only

// ============================================================
// V2.5-Evo - 2026-09-30 - RETURN-TO-ME SESSION OVERRIDE (delta audit: the RAM-only claim enforced)
//
// THE BUG THIS FIXES. fmToggleRtmEnabledFromMagnet() used to write usrConf.rtm_enabled directly and call
// that "RAM only" because it never wrote SPIFFS itself. It is not RAM only: `?save` and the web-UI save
// both persist the LIVE usrConf wholesale, so one magnet hold on the water followed by any later config
// save made a session decision permanent. The owner's instruction was explicit — "No do not change spiffs
// with a toggle or magnet ... spiffs changing is deliberate and intentional. Magnet is temp."
//
// THE FIX, AND WHY IT LOOKS LIKE THIS. usrConf is now NEVER touched by the gesture. The session value
// lives here instead, exactly the way followme_mode (last_fm_mode) and the auto-return override
// (last_fm_return_mode, right above) already work — this file's established pattern for "a decision that
// lasts the session and dies at power-off".
//   rtm_enabled_session: 0xFF = no override, use the stored usrConf.rtm_enabled
//                        0    = Return-To-Me OFF for this session
//                        1    = Return-To-Me ON for this session
// TRI-STATE, NOT A BOOL, for one concrete reason: usrConf is loaded from SPIFFS in setup(), long after
// static initialisation, so a bool here could not be seeded with the stored value at declaration time
// without inventing a second "have I been seeded yet" flag. 0xFF IS that flag.
// A POWER CYCLE RETURNS THE STORED VALUE, with nothing to undo: this variable is RAM, it resets to 0xFF,
// and usrConf.rtm_enabled still holds whatever the rider last saved deliberately through the web UI.
//
// NO NEW STRUCT FIELD. A RAM variable is not a confStruct field: sizeof(confStruct) stays 136 and
// SW_VERSION stays 27, which matters because the TX struct tail is FULL and a version bump here would
// wipe the owner's throttle calibration.
// V2.5-Evo - 2026-10-07 - R-4: NO WRITER ANY MORE. Its only writer, fmToggleRtmEnabledFromMagnet(), was
// removed (dead since the 2026-10-06 hold ruling), so this stays 0xFF and rtmEnabledEffective() always
// returns the stored usrConf.rtm_enabled. Kept, with every reader, so the gates need no change.
// ============================================================
static uint8_t       rtm_enabled_session  = 0xFF;   // 0xFF none, 0 OFF, 1 ON; RAM only, dies at power-off

// rtmEnabledEffective - is Return-To-Me enabled right now?
//
// THE SINGLE SOURCE OF TRUTH for that question. Returns the session override when one is standing,
// otherwise the value stored in SPIFFS. Every gate that used to read usrConf.rtm_enabled directly calls
// this instead — setRtmArmed(), runRtmLoop(), returnGesture() here, and the magnet RTM branch in Hall.ino
// — so a session flip is honoured in all of them at once and cannot be half-applied.
// Not static: Hall.ino (concatenated BEFORE this file) declares and calls it.
// INPUTS: rtm_enabled_session, usrConf.rtm_enabled. OUTPUT: true = enabled. No side effects, never blocks.
bool rtmEnabledEffective()
{
  if (rtm_enabled_session == 0xFF) return (usrConf.rtm_enabled != 0);   // no override -> the stored value
  return (rtm_enabled_session != 0);
}

// fmEncodeModeByte - compose the 0xF2 value byte: bits 0-2 the FM mode, bits 5-6 the override.
// Inputs: mode 0-7 (0 = disarm, 1-5 the stations). Reads last_fm_return_mode. Output: the byte.
// Side effects: none. EVERY 0xF2 goes through here so the RX always sees the current override.
static uint8_t fmEncodeModeByte(uint8_t mode)
{
  uint8_t ret_bits = 0;                              // 00 = no override: the buggy keeps its stored default
  if (last_fm_return_mode == 0)      ret_bits = 1;   // 01 = OFF for the session
  else if (last_fm_return_mode == 1) ret_bits = 2;   // 10 = ON for the session
  return (uint8_t)((mode & 0x07) | (uint8_t)(ret_bits << 5));
}

// V2.5-Evo - 2026-10-07 - M-1: 0xF2 bit 7 = "FRESH DECLARATION". Set ONLY on a declaration the rider just made with a
// gesture (a Follow-Me arm, a mode/station change); NEVER on the 30 s keepalive and never on a disarm (mode 0). After a
// Follow-Me fault the buggy ignores a mode 1-5 without this bit, so a keepalive from a remote that missed the fault can
// no longer re-arm it, while a deliberate re-arm still can. Every RX built before this round ignores bit 7 (it reads the
// mode from bits 0-2 and the override from bits 5-6), so the bit is harmless to an older buggy.
static const uint8_t kFmFreshDeclBit = 0x80;

// fmRequestKeepaliveNow - ask runFmLoop()'s 30 s 0xF2 keepalive to go out on its next tick.
// Used after the override changes while FM is armed, so the buggy learns the new value in ~100 ms
// (or as soon as the burst queue is empty) instead of up to 30 s later. Deliberately NOT a direct
// queueMetaPacketBurst(): the gesture that sets the override has just queued the 0xF1/0 cancel burst.
// (V2.5-Evo - 2026-10-07 - P-12: this used to say a second burst would OVERWRITE it; that was true of the
// old single-slot queue. The queue is 2-deep since the H-1 round and never lets an 0xF2 evict an 0xF1, so
// nothing would be lost now; the keepalive path's wait for an empty queue is simply kept.) No-op while FM
// is disarmed (fm_last_sync_ms == 0): the next arm's 0xF2 carries the override anyway.
// Inputs: none. Side effects: rewinds fm_last_sync_ms so the keepalive is due now (never to 0,
// which the keepalive reads as "disarmed").
static void fmRequestKeepaliveNow()
{
  if (fm_last_sync_ms == 0) return;
  unsigned long t = millis() - 30000UL;
  fm_last_sync_ms = (t == 0) ? 1UL : t;
}

// ---- Compute the current throttle cap for the ramp ----
// Returns 0-255. During ACTIVE, ramps from rtm_throttle_start_pct→max over rtm_ramp_duration_s.
uint8_t calcRtmThrottleCap()
{
  if (rtm_tx_state != RTM_ACTIVE) return 255;  // no cap outside ACTIVE
  unsigned long elapsed = millis() - rtm_active_start_ms;
  float t = (float)elapsed / ((float)usrConf.rtm_ramp_duration_s * 1000.0f);
  if (t > 1.0f) t = 1.0f;
  float pct = (float)usrConf.rtm_throttle_start_pct
            + t * (float)(usrConf.rtm_throttle_max_pct - usrConf.rtm_throttle_start_pct);
  return (uint8_t)(pct * 255.0f / 100.0f);
}

// ---- Called by handleGearToggle() when RTM combo gesture completes ----
// V2.5-Evo - 2026-09-18 - RTM NO LONGER DISARMS FOLLOW-ME (owner decision 27(c), always-armed).
//   From 2026-04-28 (Bug2) to 2026-09-18 this function called fmSilentDisarm() first, sending
//   0xF2/0 so the RX dropped Follow-Me to IDLE for the whole return (Finding 1-4's concern was a
//   stale fm_mode_runtime on the RX while RTM ran). Under the always-armed philosophy that was
//   wrong: the owner's loop is "stop -> bring the buggy back -> whip again" with no re-arm gesture.
//   WHAT REPLACED IT: the RX yields on its own side (RX RTMState.ino, runFmLoopBody): while
//   rtm_rx_active is set, Follow-Me parks in FM_ARMED, clears its separation latch, writes no cap
//   and no steering, and re-engages after the return only by re-proving separation. So fm_armed
//   stays true here, the 30 s 0xF2 keepalive keeps running, last_fm_mode is untouched, and the
//   RX-side mode can never be stale because the keepalive keeps refreshing it. RTM > FM: the RTM
//   ceremony below (rtm_hold_duration_s, double squeeze, rtm_arm_window_s) is exactly as it was.
// Bug4: runDoubleSqueezeArm() now handles both single and double squeeze, fully blocking.
//       On return rtm_tx_state is RTM_ACTIVE or RTM_IDLE; RTM_ARMED case in runRtmLoop() is dead code.
void setRtmArmed()
{
  // V2.5-Evo - 2026-09-30 - the EFFECTIVE enable: the stored usrConf.rtm_enabled unless a mag_mode 4 hold
  // has overridden it for this session (see rtmEnabledEffective()). Was usrConf.rtm_enabled directly.
  if (!rtmEnabledEffective() || !usrConf.gps_en) return;
  // (fmSilentDisarm() was called here until 2026-09-18 - see the header note above.)
  rtm_tx_state     = RTM_ARMED;
  rtm_arm_start_ms = millis();
  rtm_hold_start   = 0;
  rtm_tx_active    = false;
  rtm_arrival_cap_hold = false;   // V2.5-Evo - 2026-10-07 - A-1: a new run owns the cap from here (it is set to 0 below)
  rtm_cap_hold_sticky  = false;   // V2.5-Evo - 2026-10-08 - and no sticky kept cap survives into it
  // SAFETY FIX: sendData() FreeRTOS task keeps running while loop() is blocked inside
  // runDoubleSqueezeArm(). With cap=255, every arm-squeeze byte goes straight to RX and
  // drives the motor at full duty with rtm_rx_active=0 (no RX gate suppression).
  // Hold cap=0 for the entire ceremony; all abort paths below restore it to 255;
  // success path leaves it at 0 so calcRtmThrottleCap() ramp takes over on first RTM_ACTIVE tick.
  rtm_thr_cap_tx   = 0;
  queueMetaPacketBurst(0xF1, 0);   // tell RX: RTM armed but not yet active
  if (current_vib_pattern == 0) current_vib_pattern = 4;         // Pattern 4: 2 fast short = RTM arm confirm
  runDoubleSqueezeArm();            // Bug4: handles both single and double squeeze
}

// ---- Called to disengage RTM from the gesture layer (user-initiated) ----
// "St" confirm handled inside rtmDisengage().
// Every caller of THIS wrapper is a deliberate rider action (the magnet toggle in Hall.ino and the
// lock gesture; V2.5-Evo - 2026-10-07 - Q-5: the Gate 4 steer-exit no longer calls it), so it always passes
// commanded = true → no STOP buzz.
static void setRtmDisarmed()
{
  rtmDisengage(true);
}

// ---- Disengage RTM: return to COOLDOWN, notify RX, confirm with haptic + display ----
// V2.5-Evo - 2026-04-28 - P9 Bug1D: Pattern 4 and "St P" moved here so ALL exit paths
// (steer-exit, GPS stale, max runtime, throttle release) fire the confirm consistently.
// INPUT: commanded — true = stay SILENT, false = fire the Pattern 7 STOP buzz. Since the 2026-08-17
//        revision the test is no longer "did the rider press something" but "is this a FAULT or a
//        TIMEOUT": true for the deliberate stops (magnet toggle, steer-exit) AND for Gate 3, a pure
//        timeout the rider's own released trigger caused; false for the GPS-stale fault and for
//        Gate 1 (max runtime), the only timer here that can fire while he is still squeezing.
// OUTPUT: none. SIDE EFFECTS: state → RTM_COOLDOWN, throttle cap restored to 255, 0xF1/0 sent to RX,
//        STOP buzz requested when uncommanded, and a BLOCKING 2s "St" display hold.
// V2.5-Evo - 2026-10-07 - A-1: if rtm_end_keep_cap is set (the buggy ended the return itself), the cap in force is
//        KEPT instead of restored to 255, rtm_arrival_cap_hold is raised, and the 2 s "St" hold watches the trigger:
//        a full release during it lifts the cap at the end of the hold; otherwise rtmArrivalCapUpdate() lifts it on
//        the first released sample afterwards.
// V2.5-Evo - 2026-10-08 - STICKY RETURN CAP: if rtm_end_sticky is set too (an early end: Gate 1, Gate 2, the buggy's
//        fault bit), the kept cap is a sticky one - rtm_cap_hold_sticky - and lifts at raw < 26 instead of < 10.
static void rtmDisengage(bool commanded)
{
  rtm_tx_state    = RTM_COOLDOWN;
  rtm_cooldown_ms = millis();
  rtm_tx_active   = false;
  displayBuffer[6] = 0x0000;     // Bug3: clear R5 proximity bar row — updateR5ProximityBar() left
                                 // stale data here; without clearing, FM mode sees a phantom pixel
  if (rtm_end_keep_cap)
  {
    rtm_end_keep_cap     = false;   // A-1: consumed
    rtm_arrival_cap_hold = true;    // the kept cap stands until a full release
    rtm_cap_hold_sticky  = rtm_end_sticky;   // V2.5-Evo - 2026-10-08 - a sticky one clears at raw < 26 instead
    rtm_end_sticky       = false;            // consumed
    // V2.5-Evo - 2026-10-07 - Q-1: keep the LOWER of the cap in force and the one the ending path asked for.
    if (rtm_end_cap_value < rtm_thr_cap_tx.load()) rtm_thr_cap_tx = rtm_end_cap_value;
  }
  else
  {
    rtm_arrival_cap_hold = false;
    rtm_cap_hold_sticky  = false;   // V2.5-Evo - 2026-10-08
    rtm_end_sticky       = false;
    rtm_thr_cap_tx  = 255;
  }
  rtm_arm_dist_m  = 0.0f;        // reset R5 bar reference (defined in BREmote_V2_Tx.h)
  rtm_arm_gps_timeout_override = 0;  // clear GPS timeout multiplier — ceremony fully over
  queueMetaPacketBurst(0xF1, 0);  // tell RX: RTM inactive

  // Request the STOP confirm BEFORE the blocking display so the buzz runs during the 2s flash.
  // V2.5-Evo - 2026-08-16 - HAPTIC CUT: silent on a DELIBERATE disarm. You just did it, and the
  // display already says so - the stop confirm appears. A buzz confirming your own action is noise,
  // and it was the single most frequent buzz in the system.
  // V2.5-Evo - 2026-08-17 - StopBuzz: that cut was written here, at the SHARED sink, so it also
  // silenced every safety gate that ends up in this same function. The fix is not to buzz them all
  // back - it is to let the caller decide, on one rule: A PURE TIMEOUT IS SILENT, A FAULT BUZZES.
  // Mid-wave the rider has no attention to spare for decoding a buzz, and the more buzzes there
  // are the less each one is read. So Pattern 7 is spent only where it buys something:
  //   FAULT   -> Gate 2 (TX GPS lost). A sensor died; nothing the rider did explains the stop.
  //   TIMEOUT -> Gate 3 (trigger released 4s). His own hand caused it, "St" shows it. Silent.
  //   Gate 1 (max runtime) buzzes despite being a timer, and it is the ONE exception: it has no
  //   throttle precondition, so it can fire while he is still squeezing - and rtm_thr_cap_tx was
  //   restored to 255 a few lines above, which un-clamps Throttle.ino's `if (result >
  //   rtm_thr_cap_tx)` in the same instant. The cap is 30-90% by config range and can never BE
  //   255, so that step to raw manual throttle is always real, and the sendData task keeps
  //   transmitting it throughout the blocking 2s hold below. That transition earns a warning;
  //   the two release-driven timeouts, where the trigger is already at rest, do not.
  // V2.5-Evo - 2026-10-07 - F-1: a Follow-Me fault deferred during this return is announced now - this "St" gets
  // the stop buzz even if the return itself ended quietly. If the return ended on its own fault it buzzes anyway.
  if (fm_fault_deferred)
  {
    fm_fault_deferred = false;
    if (commanded) Serial.println("FM [TX] the Follow-Me fault stop deferred during Return-To-Me is announced now: St + stop buzz");
    commanded = false;
  }
  if (!commanded) vib_stop_pending = true;   // Pattern 7: one long buzz = a FAULT stopped the system

  // Large-font stop confirm: LET_S(32) renders as "5", LET_T(20) renders as "t".
  // "5t" appearance is intentional — matches large-font style of F0-F3 confirms.
  DISP_LOCK(); displayDigits(LET_S, LET_T); updateDisplay(); DISP_UNLOCK();
  if (rtm_arrival_cap_hold)
  {
    // V2.5-Evo - 2026-10-07 - A-1: watch the trigger during the hold (the loop is blocked here).
    // V2.5-Evo - 2026-10-07 - Q-6: THE BUG - a release seen during the hold lifted the cap only when the 2 s ended, so a
    // rider who let go and squeezed again inside the hold stayed capped for up to 2 s. THE FIX: the cap is lifted AT THE
    // 10 ms SAMPLE where the release is seen; the "St" hold itself still runs its full 2 s.
    const unsigned long hold_start_ms = millis();
    while (millis() - hold_start_ms < 2000UL)
    {
      while (Serial1.available()) gps_tx.encode(Serial1.read());
      if (rtm_arrival_cap_hold && rtmKeptCapReleased())   // V2.5-Evo - 2026-10-08 - was triggerReleased(): a sticky cap clears at raw < 26
      {
        rtm_arrival_cap_hold = false;
        rtm_thr_cap_tx       = 255;
        Serial.println("RTM [TX] trigger released after the return ended -> throttle cap lifted, full manual");
      }
      delay(10);
    }
  }
  else
  {
    gpsKeepAliveDelay(2000);
  }
}

// rtmArrivalCapUpdate - V2.5-Evo - 2026-10-07 - A-1 / SOP-040: lift the cap kept after the buggy ended a return,
// on the first sample that shows the trigger fully released. Called every runRtmLoop() tick (~110 ms), before the
// RTM enable check so a disabled RTM cannot strand a cap. A new arm (setRtmArmed()) clears the hold itself.
// Inputs: rtm_arrival_cap_hold, triggerReleased(). Side effects: rtm_thr_cap_tx = 255 + one line on release.
// V2.5-Evo - 2026-10-08 - the release test is rtmKeptCapReleased(): raw < 26 for a sticky kept cap (an early end),
// triggerReleased() (< 10) for every other kept cap, exactly as before.
static void rtmArrivalCapUpdate()
{
  if (!rtm_arrival_cap_hold) return;
  if (!rtmKeptCapReleased()) return;
  rtm_arrival_cap_hold = false;
  rtm_thr_cap_tx       = 255;
  Serial.println("RTM [TX] trigger released after the return ended -> throttle cap lifted, full manual");
}

// rtmCapHoldDelay - V2.5-Evo - 2026-10-07 - T-1: a blocking wait (the 2 s "St" hold) that also lifts a kept cap.
// gpsKeepAliveDelay() that watches the trigger: if rtm_arrival_cap_hold is set and a full release is seen at a 10 ms
// sample, the cap goes back to 255 at that sample (the Q-6 rule), so a rider who lets go and squeezes again inside the
// hold is not left capped. Inputs: ms, rtm_arrival_cap_hold, triggerReleased(). Side effects: drains Serial1 into gps_tx;
// may set rtm_thr_cap_tx = 255 and print one line. Loop task only. BLOCKS for ms.
static void rtmCapHoldDelay(uint32_t ms)
{
  const unsigned long start_ms = millis();
  while (millis() - start_ms < ms)
  {
    while (Serial1.available()) gps_tx.encode(Serial1.read());
    rtmArrivalCapUpdate();
    delay(10);
  }
}

// rtmCeremonyKeepCapIfHeld - V2.5-Evo - 2026-10-07 - T-1: how an arm ceremony that did NOT go ACTIVE gives the throttle
// back. THE BUG: the pre-arm distance refusal (buggy already inside the disengage distance) and the two squeeze
// timeouts set the cap straight back to 255. A rider still squeezing to summon the buggy - which is right next to him in
// the refusal case - got his held trigger passed to the motors uncapped the moment the ceremony ended. THE FIX (the
// SOP-040 arrival rule): with the trigger NOT fully released, the cap stays 0 (it has been 0 for the whole ceremony) and
// rtm_arrival_cap_hold is raised, so only a full release gives full manual back (rtmArrivalCapUpdate(), or
// rtmCapHoldDelay() during a "St" hold). With the trigger released, the cap goes back to 255 at once, as before.
// The 1 s LEFT cancel (ceremonyCancelToManual()) is deliberately NOT routed here: it is the rider's own way back to full
// manual and keeps restoring 255 directly. Inputs: triggerReleased(), why (for the serial line). Side effects:
// rtm_thr_cap_tx, rtm_arrival_cap_hold, one serial line when the cap is kept. Loop task only.
static void rtmCeremonyKeepCapIfHeld(const char *why)
{
  if (triggerReleased())
  {
    rtm_arrival_cap_hold = false;
    rtm_thr_cap_tx       = 255;   // released: nothing to protect, full manual at once
    return;
  }
  rtm_thr_cap_tx       = 0;       // only ever lowers it: the ceremony already held it at 0
  rtm_arrival_cap_hold = true;
  rtm_cap_hold_sticky  = false;   // V2.5-Evo - 2026-10-08 - a ceremony hold clears on the full release (< 10), not the sticky rule
  Serial.printf("RTM [TX] %s with the trigger held -> throttle cap 0 until the trigger is fully released\n", why);
}

// rtmReturnConfirmed - V2.5-Evo - 2026-10-07 - Q-2 / SOP-041: is a manual return running AND confirmed by the buggy
// (an fm_status arrival with bit 1 set after this run went ACTIVE)? The screen draws the RTM returning look only while
// this is true; before it, "rn" (waiting). Inputs: rtm_tx_state, fm_status_rtm_on_ms, rtm_active_start_ms.
// Output: true = confirmed. No side effects. Not static: Display.ino (concatenated before this file) calls it from the
// loop task and from the bargraph task; every read is one aligned word, so no tearing.
bool rtmReturnConfirmed()
{
  if (rtm_tx_state != RTM_ACTIVE) return false;
  const unsigned long on_ms = fm_status_rtm_on_ms;
  return (on_ms != 0) && ((long)(on_ms - rtm_active_start_ms) > 0);
}

// rtmUnconfirmedReset - V2.5-Evo - 2026-10-07 - Q-2: start a fresh confirmation count at RTM_ACTIVE entry. Arrivals
// already received are ignored (rtm_q2_last_arrival_ms = the current stamp). Loop task only.
static void rtmUnconfirmedReset()
{
  rtm_q2_last_arrival_ms = fm_status_arrival_ms;
  rtm_q2_clear_count     = 0;
  rtm_q2_requeued        = false;
}

// rtmUnconfirmedCheck - V2.5-Evo - 2026-10-07 - Q-2: one tick of the "the buggy never confirmed RTM" rule (see the
// block at the top of this file). Call only while RTM_ACTIVE and not yet confirmed.
// Inputs: fm_status_arrival_ms, telemetry.fm_status, rtm_start_sent_ms, the meta queue, the rtm_q2_* state.
// Returns: true if it ENDED the run (refusal: rtmDisengage(false) has run - the caller must stop). Side effects: may
// re-queue 0xF1/1 once; one serial line per decision.
static bool rtmUnconfirmedCheck()
{
  const unsigned long arr = fm_status_arrival_ms;
  if (arr == rtm_q2_last_arrival_ms) return false;                          // no new fm_status arrival
  rtm_q2_last_arrival_ms = arr;
  const unsigned long sent = rtm_start_sent_ms;
  if (sent == 0 || (long)(sent - rtm_active_start_ms) < 0) return false;   // this run's 0xF1/1 not on the air yet
  if (metaQueuePending(0xF1, 1, false)) return false;                       // the 0xF1/1 burst is still draining
  if ((long)(arr - sent) <= (long)kRtmConfirmMarginMs) return false;       // too soon after it to be evidence
  if (telemetry.fm_status & FM_STATUS_RTM_ACTIVE) return false;            // bit set: that IS the confirmation
  if (rtm_q2_clear_count < 255) rtm_q2_clear_count++;
  if (rtm_q2_clear_count < 2) return false;
  if (!rtm_q2_requeued)
  {
    rtm_q2_requeued    = true;
    rtm_q2_clear_count = 0;
    queueMetaPacketBurst(0xF1, 1);   // state burst: replaces a queued refresh / boot ID in place, never a stop
    Serial.println("RTM [TX] the buggy has not confirmed Return-To-Me on 2 reports -> 0xF1/1 sent again");
    return false;
  }
  Serial.printf("RTM [TX] the buggy did not confirm Return-To-Me after the re-send -> St + stop buzz; throttle cap %u kept until the trigger is fully released\n",
                (unsigned)rtm_thr_cap_tx.load());
  rtm_end_cap_value = rtm_thr_cap_tx.load();   // keep the cap in force
  rtm_end_keep_cap  = true;
  rtmDisengage(false);                         // a refusal: "St" + the stop buzz (SOP-040)
  return true;
}

// ---- Decode telemetry.rtm_distance to metres ----
// Returns -1.0f if no valid distance available (telemetry.rtm_distance == 0xFF or 0x00).
// Used by pre-arm check (Bug 1B) and R5 proximity bar.
static float decodeRtmDistanceM()
{
  uint8_t d = telemetry.rtm_distance;
  // Bug A fix: treat 0x00 as "no data" same as 0xFF.
  // 0x00 is the zero-initialized default before any RX telemetry packet arrives.
  // Previously returned 0.0m → pre-arm check saw 0.0m ≤ disengage threshold → always rejected.
  if (d == 0xFF || d == 0x00) return -1.0f;
  if (d < 100) return d / 10.0f;      // tenths of metre (0.0–9.9 m)
  return (float)(d - 90);             // whole metres (10–164 m)
}

// ============================================================
// V2.5-Evo - 2026-09-19 - THE RETURN GESTURE INSIDE THE ARM CEREMONY (gesture state 2)
//
// The RTM arm is a BLOCKING ceremony: setRtmArmed() runs runDoubleSqueezeArm(), which holds loop()
// - and therefore checkButtons()/handleGearToggle() - for up to rtm_arm_window_s while it waits
// for the squeeze. rtm_tx_state == RTM_ARMED exists ONLY inside that ceremony, so "the gesture done
// again while RTM is armed and inside the arm window" can only be seen from inside it. These two
// helpers are that: the wait loops call returnGestureCeremonyPoll() every 100 ms; it re-implements
// handleGearToggle()'s combo on the live toggle input (calcFilter() keeps producing tog_input during
// the ceremony because in_menu is held non-zero) - a RIGHT tap shorter than COMBO_TAP_MAX_MS, then a
// LEFT hold of rtm_hold_duration_s begun within COMBO_WINDOW_MS of the tap, with the trigger released
// (thr_scaled < 10, the same action gate). The LEFT hold that STARTED the ceremony must be released
// first; it never counts. When it fires, ceremonyCancelForReturnGesture() aborts the arm exactly
// like a timeout does (cap 255, GPS override cleared, display cleared, RTM_IDLE, 0xF1/0) and then
// flips the override: last_fm_return_mode = the OPPOSITE of the auto-return mode the buggy last
// echoed in fm_flags bit 7, Pattern 9, "Ar" (ON) / "AO" (OFF) for 2 s, keepalive requested.
//
// INVARIANT (owner's miscount hazard): a miscounted gesture can never produce motion. Arming RTM
// only declares intent - the squeeze ceremony, RTM's gates and a held trigger are still required;
// cancelling the arm removes intent; the override only changes what a Follow-Me HOLD graduates to
// on the buggy, and that graduation itself moves nothing without the trigger. The gesture is
// off-throttle only (thr_scaled < 10 at the instant the hold completes) and the ceremony's own
// squeeze detection (thr_scaled > 76) is mutually exclusive with it.
// ============================================================

// returnGestureCeremonyPoll - RIGHT tap + LEFT hold detector for the blocking arm ceremony.
// Inputs:  reset - true to clear the detector at ceremony start (the arming LEFT hold is still
//          down at that moment and must be released before anything counts). Reads ctplus() /
//          ctminus() (Hall.ino), thr_scaled, usrConf.rtm_hold_duration_s, COMBO_TAP_MAX_MS,
//          COMBO_WINDOW_MS (Hall.ino).
// Returns: true exactly once per completed gesture. Side effects: its own static state only.
// V2.5-Evo - 2026-09-19 - DIAGNOSTIC ONLY, no behaviour change: one Serial.printf per detector
//   edge (arming hold released, RIGHT tap registered, LEFT hold started, gesture fired) in the
//   RETURN [TX] print style, added because the owner could not get the in-ceremony second gesture
//   to fire on the bench and the remote was not on USB. Detector timing, thresholds and ordering
//   are untouched.
// V2.5-Evo - 2026-10-07 - R-3: true while a valid RIGHT tap -> LEFT hold sequence is under way inside the
// ceremony (written by returnGestureCeremonyPoll() on every call). runDoubleSqueezeArm() keeps waiting past
// rtm_arm_window_s while it is true, so a sequence started inside the window is allowed to finish instead of
// being cut off mid-hold. It is bounded: a pending tap expires COMBO_WINDOW_MS after the tap, a started hold
// ends on release, completion or a squeeze, so the extension is at most COMBO_WINDOW_MS + rtm_hold_duration_s.
// (V2.5-Evo - 2026-10-07 - F-3: that bound did not hold - a fresh RIGHT tap renewed it - so the wait loops now also
// stop at the absolute ceiling in ceremonyExtensionAllowed().)
static bool return_gesture_in_progress = false;

// ceremonyExtensionAllowed - V2.5-Evo - 2026-10-07 - F-3: the R-3 extension's HARD CEILING.
// THE BUG: the bound above was not a real ceiling - every new RIGHT tap renewed the pending tap, so tapping RIGHT
// every 2 s kept the ceremony (and its zero throttle) going past rtm_arm_window_s indefinitely. THE FIX: the
// extension may never run past window + COMBO_WINDOW_MS + rtm_hold_duration_s from the ceremony start, which is
// the longest a RIGHT tap -> LEFT hold begun on the last instant of the window can legitimately take.
// Inputs: rtm_arm_start_ms, usrConf.rtm_arm_window_s / rtm_hold_duration_s, COMBO_WINDOW_MS (Hall.ino).
// Output: true = still inside the ceiling. No side effects.
static bool ceremonyExtensionAllowed()
{
  const unsigned long ceiling_ms = (unsigned long)usrConf.rtm_arm_window_s * 1000UL + COMBO_WINDOW_MS
                                 + (unsigned long)usrConf.rtm_hold_duration_s * 1000UL;
  return (millis() - rtm_arm_start_ms) < ceiling_ms;
}

static bool returnGestureCeremonyPoll(bool reset)
{
  static bool          arm_hold_released = false;   // the LEFT hold that started the ceremony has been let go
  static bool          right_was_down    = false;
  static bool          left_was_down     = false;
  static unsigned long right_down_ms     = 0;
  static unsigned long right_tap_ms      = 0;       // millis() of the last RIGHT tap; 0 = none pending
  static unsigned long left_down_ms      = 0;
  return_gesture_in_progress = false;   // V2.5-Evo - 2026-10-07 - R-3: recomputed below on every live call
  if (reset) {
    arm_hold_released = false;
    right_was_down = left_was_down = false;
    right_down_ms = right_tap_ms = left_down_ms = 0;
    return false;
  }
  const unsigned long now   = millis();
  const bool          right = ctplus();
  const bool          left  = ctminus();

  if (!arm_hold_released) {
    if (!left) {
      arm_hold_released = true;   // the arming hold is over; from here the toggle is a fresh input
      Serial.println("RETURN [TX] ceremony poll: arming hold released, detector live");   // V2.5-Evo - 2026-09-19 - diagnostic only
    }
    return false;
  }

  // RIGHT: a press shorter than COMBO_TAP_MAX_MS, on release, is a tap.
  if (right && !right_was_down) right_down_ms = now;
  if (!right && right_was_down && (now - right_down_ms) < COMBO_TAP_MAX_MS) {
    right_tap_ms = now;
    Serial.printf("RETURN [TX] ceremony poll: RIGHT tap registered (%lu ms)\n", (unsigned long)(now - right_down_ms));   // V2.5-Evo - 2026-09-19 - diagnostic only
  }
  right_was_down = right;

  // LEFT: time the hold from its press edge.
  if (left && !left_was_down) {
    left_down_ms = now;
    Serial.println("RETURN [TX] ceremony poll: LEFT hold started");   // V2.5-Evo - 2026-09-19 - diagnostic only
  }
  left_was_down = left;

  if (left && right_tap_ms != 0 &&
      (left_down_ms - right_tap_ms) < COMBO_WINDOW_MS &&
      (now - left_down_ms) >= (unsigned long)usrConf.rtm_hold_duration_s * 1000UL &&
      thr_scaled < 10) {
    right_tap_ms = 0;   // consumed: the same hold cannot fire twice
    Serial.println("RETURN [TX] ceremony poll: gesture fired (tap+hold combo complete)");   // V2.5-Evo - 2026-09-19 - diagnostic only
    return true;
  }

  // V2.5-Evo - 2026-10-07 - R-3: is a valid sequence under way? A RIGHT tap is pending and either (a) the
  // LEFT hold has started within COMBO_WINDOW_MS of it and is still down, or (b) LEFT is not down yet and
  // the tap is still young enough for a LEFT hold to qualify. The trigger must be released, because the
  // gesture can only fire with thr_scaled < 10; a squeeze ends the extension at once.
  if (right_tap_ms != 0 && thr_scaled < 10) {
    if (left) return_gesture_in_progress = (left_down_ms - right_tap_ms) < COMBO_WINDOW_MS;
    else      return_gesture_in_progress = (now - right_tap_ms) < COMBO_WINDOW_MS;
  }
  return false;
}

// ceremonyCancelForReturnGesture - abort the RTM arm ceremony and flip the auto-return override.
// Inputs: none (reads telemetry.fm_flags for the buggy's echo). Side effects: rtm_tx_state ->
//   RTM_IDLE, rtm_thr_cap_tx 255, rtm_arm_gps_timeout_override 0, display cleared, 0xF1/0 queued;
//   last_fm_return_mode set to the opposite of the echo; Pattern 9; a BLOCKING 2 s "A1"/"A0" hold;
//   the keepalive requested (if FM is armed). Called only from runDoubleSqueezeArm().
// V2.5-Evo - 2026-10-07 - INVARIANT (moved here from the removed fmToggleAutoReturnFromMagnet()): AN
//   AUTO-RETURN FLIP NEVER CHANGES FOLLOW-ME ARMING. This function and returnGesture() state 3 write
//   last_fm_return_mode only - never fm_armed, last_fm_mode or fm_throttle_seen - and the keepalive re-sends
//   the SAME station with only bits 5-6 changed. Field check 2026-10-05 (RX log RX1_2026-10-05_2346): after
//   the A0 the buggy stayed FM ARMED at station 2; Follow-Me ended later on a buggy DIVERGENCE fault-stop.
static void ceremonyCancelForReturnGesture()
{
  // The abort, exactly as the ceremony's own timeout paths do it.
  rtm_arm_gps_timeout_override = 0;
  rtm_thr_cap_tx = 255;
  DISP_LOCK(); for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000; updateDisplay(); DISP_UNLOCK();
  rtm_tx_state  = RTM_IDLE;
  rtm_tx_active = false;
  queueMetaPacketBurst(0xF1, 0);   // tell RX: RTM not active (belt: it already heard 0xF1/0 at ceremony start)

  // The flip: opposite of what the buggy says it is doing NOW. With the stored default ON this is
  // the only way the gesture can turn auto-return OFF for a session, and vice versa.
  const bool echo_on = (telemetry.fm_flags & FM_FLAG_RETURN_ON) != 0;
  last_fm_return_mode = echo_on ? 0 : 1;
  Serial.printf("RETURN [TX] gesture during the RTM arm: arm cancelled (0xF1/0); auto-return override set %s for this session (buggy reported %s)\n",
                last_fm_return_mode ? "ON" : "OFF", echo_on ? "ON" : "OFF");
  if (current_vib_pattern == 0) current_vib_pattern = 9;   // Pattern 9: four quick taps = override SET
  // V2.5-Evo - 2026-10-01 - "A1" = auto-return ON, "A0" = OFF. Was "Ar"/"AO", which
  // shared the r glyph with the MANUAL Return-To-Me readout ("r1"/"r0") and invited
  // exactly the confusion the owner flagged: RTM is the manual recall, auto-return is
  // the automatic one inside Follow-Me. One grammar now - letter = which feature,
  // digit = its state - and no glyph shared between the two.
  DISP_LOCK(); displayDigits(LET_A, last_fm_return_mode ? 1 : 0); updateDisplay(); DISP_UNLOCK();
  gpsKeepAliveDelay(2000);
  fmRequestKeepaliveNow();   // the 0xF1/0 burst drains first; the keepalive then carries bits 5-6
}

// ============================================================
// V2.5-Evo - 2026-10-07 - M-3 / F-2: A 1 s LEFT HOLD CANCELS THE ARM CEREMONY BACK TO FULL MANUAL
// THE BUG: while "rn" blinks the remote sends zero throttle for up to rtm_arm_window_s (15 s on the owner's
// remote) and there was no way out except waiting: the only in-ceremony gesture was the A1/A0 combo, and a
// rider who kept the trigger held through a magnet hold had no action at all that returned power.
// THE FIX: hold the toggle LEFT for kCeremonyLeftCancelMs (1 s) -> the arm is cancelled AT ONCE: 0xF1/0 to the
// buggy, throttle cap back to 255 (full manual), "St" with the normal stop buzz (vib_stop_pending, SOP-040).
// DETAILS THAT MATTER:
//   - LEFT is read from tog_scaled, not ctminus(): with the trigger squeezed calcFilter() hands the toggle to
//     steering and tog_input reads 0, but tog_scaled still shows the push. So the cancel works trigger held
//     or released - the held-trigger case is exactly F-2.
//   - The LEFT hold that STARTED the ceremony (toggle route) must be released first; it never counts.
//   - A RIGHT tap first makes the LEFT hold the A1/A0 auto-return switch instead (return_gesture_in_progress,
//     set by returnGestureCeremonyPoll() on the same tick), so that existing gesture still works.
//   - After the cancel the LEFT toggle may still be held; ceremony_toggle_latch (Hall.ino) keeps it from
//     becoming a lock or a gear step until it is centred.
//   - No magnet cancel (owner still deciding).
// ============================================================
static const unsigned long kCeremonyLeftCancelMs = 1000UL;

// ceremonyLeftPastHalf - V2.5-Evo - 2026-10-07 - P-6: is the toggle pushed LEFT by MORE THAN HALF of its
// calibrated left travel (tog_mid -> tog_left)?
// THE BUG: the cancel counted any LEFT push past tog_diff (a few percent). In a held-trigger ceremony the toggle
// is also the steering stick, so an ordinary LEFT steer held for 1 s cancelled the return the rider had asked for.
// THE FIX: a deliberate push past the halfway point is required. Read from the raw averaged toggle counts against
// the stored calibration, which works for either calibration direction (tog_left above or below tog_mid).
// Inputs: tog_raw[] via readFilteredInputs() (Hall.ino), usrConf.tog_left / tog_mid, ads_input_fault (a frozen
// buffer during an input fault is not a rider's push). Output: true = past half. No side effects. Loop task.
static bool ceremonyLeftPastHalf()
{
  if (ads_input_fault) return false;
  uint16_t thr_f = 0, tog_f = 0;
  readFilteredInputs(thr_f, tog_f);
  const int32_t span = (int32_t)usrConf.tog_left - (int32_t)usrConf.tog_mid;   // signed left travel
  if (span == 0) return false;                                                 // no calibration to judge by
  const int32_t d = (int32_t)tog_f - (int32_t)usrConf.tog_mid;                 // signed deflection now
  return (span > 0) ? (2 * d > span) : (2 * d < span);                         // same side, beyond half
}

// ceremonyLeftCancelPoll - 1 s LEFT-hold detector for the blocking arm ceremony. Call AFTER
// returnGestureCeremonyPoll(false) on the same tick.
// Inputs: reset - true at ceremony start. Reads tog_scaled, usrConf.tog_diff, ceremonyLeftPastHalf(),
//   return_gesture_in_progress.
// Returns: true once the hold has lasted kCeremonyLeftCancelMs. Side effects: its own static state only.
// V2.5-Evo - 2026-10-07 - P-6: "released" still means the toggle is back near centre (the old tog_diff test, so
// the arming LEFT hold must really be let go), but the 1 s hold only counts while the push is past half travel
// (ceremonyLeftPastHalf()); dropping below half restarts the second.
static bool ceremonyLeftCancelPoll(bool reset)
{
  static bool          released_seen = false;   // LEFT seen not pushed since the ceremony started
  static bool          holding       = false;
  static unsigned long hold_start_ms = 0;
  if (reset) { released_seen = false; holding = false; return false; }

  const bool left = ((int)tog_scaled < 127 - (int)usrConf.tog_diff);
  if (!left)                      { released_seen = true; holding = false; return false; }
  if (!released_seen)             return false;                       // still the arming hold
  if (return_gesture_in_progress) { holding = false; return false; }  // RIGHT tap first: this is A1/A0
  if (!ceremonyLeftPastHalf())    { holding = false; return false; }  // P-6: a light push / steer does not count
  if (!holding) { holding = true; hold_start_ms = millis(); }
  return (millis() - hold_start_ms) >= kCeremonyLeftCancelMs;
}

// ceremonyCancelToManual - end the arm ceremony and hand back full manual throttle at once.
// Inputs: none. Side effects: rtm_tx_state -> RTM_IDLE (so rtmIsArming() stops zeroing the throttle byte),
//   rtm_thr_cap_tx 255, GPS override cleared, 0xF1/0 queued, stop buzz requested, then a 2 s "St" hold
//   (BLOCKING, like every other "St"; the radio task passes the trigger during it). Loop task only.
static void ceremonyCancelToManual()
{
  rtm_arm_gps_timeout_override = 0;
  rtm_thr_cap_tx = 255;
  rtm_tx_state   = RTM_IDLE;
  rtm_tx_active  = false;
  queueMetaPacketBurst(0xF1, 0);
  Serial.println("RTM [TX] arm cancelled: LEFT hold 1 s -> full manual (0xF1/0)");
  vib_stop_pending = true;   // Pattern 7, the normal stop buzz
  DISP_LOCK(); displayDigits(LET_S, LET_T); updateDisplay(); DISP_UNLOCK();
  gpsKeepAliveDelay(2000);
}

// ============================================================
// V2.5-Evo - 2026-04-28 - Bug4: Full rewrite. Handles both single and double squeeze.
// Always called blocking from setRtmArmed(). Uses rtm_arm_start_ms as shared arm-window ref.
// "A r" and "rn ×2" ceremony removed. Arm confirmation is unlockAnimation() + "r n" 2s.
//
// Single (rtm_double_squeeze_en==0):
//   blink "r n" → thr >30% held 500ms → unlockAnimation()+P4 → "r n" 2s → dist check → ACTIVE
//
// Double (rtm_double_squeeze_en==1):
//   blink "r n" → 1st thr >30% held 500ms → unlockAnimation() → blank 800ms →
//   2nd thr >30% held 500ms → unlockAnimation()+P4 → "r n" 2s → dist check → ACTIVE
//
// On return: rtm_tx_state == RTM_ACTIVE (success) or RTM_IDLE (timeout / rejected).
// ============================================================
// ============================================================
// V2.5-Evo - 2026-10-07 - DEFECT b: "rn" MUST BE VISIBLE WHILE THE CEREMONY WAITS FOR THE SQUEEZE
// THE BUG: the wait drew "r n" once and then called advanceArrow() every 100 ms; advanceArrow() clears the whole
// digit zone and draws the bobbing arrow, so "rn" was overdrawn on the very first tick and the rider only ever saw
// the arrow (SOP-041 table: manual RTM window = "rn", must be visible). THE FIX: the two ALTERNATE in a fixed
// cycle - "r n" steady for kCeremonyRnShowMs, then the bobbing arrow (the "squeeze" prompt) for the rest of
// kCeremonyCycleMs. Same glyphs and same digit zone as before; nothing new is drawn and R5 / C7-C9 are untouched.
// Inputs: phase_start_ms - millis() when this wait began (the cycle starts on "r n"). Side effects: draws the
// digit zone (takes displayMutex via DISP_LOCK / advanceArrow()). Loop task only; called every ~100 ms.
// ============================================================
static const unsigned long kCeremonyRnShowMs = 1000UL;   // "r n" shown this long...
static const unsigned long kCeremonyCycleMs  = 1500UL;   // ...out of every cycle; the arrow fills the remaining 500 ms

static void ceremonyWaitFrame(unsigned long phase_start_ms)
{
  if (((millis() - phase_start_ms) % kCeremonyCycleMs) < kCeremonyRnShowMs)
  {
    DISP_LOCK(); displayDigitZone("r n"); updateDisplay(); DISP_UNLOCK();
  }
  else
  {
    advanceArrow();   // bob the arrow (it redraws the digit zone and calls updateDisplay() itself)
  }
}

static void runDoubleSqueezeArm()
{
  // Relax the GPS staleness threshold (Gate 2) for the duration of this blocking ceremony.
  // loop() is suspended here for up to rtm_arm_window_s seconds, so GPS age accumulates.
  // rtmDisengage() clears this on every RTM_ACTIVE exit path.
  rtm_arm_gps_timeout_override = (uint32_t)usrConf.rtm_gps_timeout_ms * 4UL;

  // Show "r n" while waiting for first squeeze
  // V2.5-Evo - 2026-10-07 - defect b: "r n" alternates with the arrow (ceremonyWaitFrame()); it used to be drawn
  // here and overdrawn at once by advanceArrow().
  const unsigned long wait1_start_ms = millis();
  ceremonyWaitFrame(wait1_start_ms);

  returnGestureCeremonyPoll(true);   // V2.5-Evo - 2026-09-19 - fresh detector; the arming hold is still down
  ceremonyLeftCancelPoll(true);      // V2.5-Evo - 2026-10-07 - M-3: fresh 1 s LEFT-hold cancel detector

  // V2.5-Evo - 2026-10-07 - R-3: from here on, a toggle must be seen centred before runMenu() (Hall.ino)
  // acts on it again. Set at the start so EVERY exit below (squeeze timeout, A1/A0 cancel, pre-arm refusal,
  // RTM ACTIVE) is covered without touching each return path; nothing reads it while this function blocks.
  ceremony_toggle_latch = true;

  // V2.5-Evo - 2026-10-07 - RELEASE-FIRST SQUEEZES (audit R-1). THE BUG: the waits below accepted any
  // trigger above 30% held for 500 ms and never asked whether the trigger had been let go first. A rider
  // who kept the trigger held through a magnet hold therefore completed the whole ceremony with no
  // deliberate squeeze at all: squeeze 1 was "met" about 0.6 s after the magnet came off, squeeze 2
  // about 1.3 s later, and RTM went ACTIVE about 5 s after the hold - with Follow-Me armed, possibly
  // while he was still on the rope. THE FIX: a squeeze only counts after the trigger has been SEEN
  // RELEASED (thr_scaled < 10, the same "released" test the toggle gestures use) since the ceremony
  // started, and again between squeeze 1 and squeeze 2. A deliberate squeeze - release, squeeze, hold
  // half a second - works exactly as before; the toggle route always starts released, so nothing
  // changes there. Throttle is still forced to 0 for the whole ceremony (rtm_thr_cap_tx = 0 and
  // rtmIsArming() in Radio.ino), so this only decides WHEN the ceremony may complete.
  bool          thr_released_seen = false;

  // Wait for first squeeze: thr > 30% (thr_scaled > 76) held for 500ms continuous
  bool          first_ok = false;
  unsigned long hold_ms  = 0;
  while (millis() - rtm_arm_start_ms < (unsigned long)usrConf.rtm_arm_window_s * 1000UL ||
         (return_gesture_in_progress && ceremonyExtensionAllowed()))   // R-3: a started tap -> hold may finish; F-3: hard ceiling
  {
    ceremonyWaitFrame(wait1_start_ms);   // V2.5-Evo - 2026-10-07 - defect b: "r n" / arrow alternate (was advanceArrow() only)
    if (thr_scaled < 10) thr_released_seen = true;   // V2.5-Evo - 2026-10-07 - R-1: release seen
    if (thr_released_seen && thr_scaled > 76 &&     // V2.5-Evo - 2026-10-07 - R-1: counts only after a release
        millis() - rtm_arm_start_ms < (unsigned long)usrConf.rtm_arm_window_s * 1000UL)   // R-3: never past the window
    {
      if (hold_ms == 0) hold_ms = millis();
      if (millis() - hold_ms >= 500UL) { first_ok = true; hold_ms = 0; break; }
    }
    else { hold_ms = 0; }
    // V2.5-Evo - 2026-09-19 - the return gesture done again while waiting = cancel + flip (state 2).
    if (returnGestureCeremonyPoll(false)) { ceremonyCancelForReturnGesture(); return; }
    // V2.5-Evo - 2026-10-07 - M-3 / F-2: a plain 1 s LEFT hold = cancel to full manual, "St" + stop buzz.
    if (ceremonyLeftCancelPoll(false)) { ceremonyCancelToManual(); return; }
    delay(100);
    checkSerial();
  }
  if (!first_ok)
  {
    rtm_arm_gps_timeout_override = 0;  // ceremony aborted — restore normal GPS threshold
    // V2.5-Evo - 2026-10-07 - T-1: was rtm_thr_cap_tx = 255 unconditionally; a held trigger now keeps cap 0 until released.
    rtmCeremonyKeepCapIfHeld("arm timed out (no first squeeze)");
    DISP_LOCK(); for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000; updateDisplay(); DISP_UNLOCK();
    rtm_tx_state = RTM_IDLE;
    Serial.printf("RTM [TX] arm cancelled: no first squeeze within %u s\n", (unsigned)usrConf.rtm_arm_window_s);
    return;
  }

  if (!usrConf.rtm_double_squeeze_en)
  {
    // Single-squeeze: unlock, pause, then Pattern 4 + "r n" arm confirm
    unlockAnimation();
    gpsKeepAliveDelay(750);   // V2.5-Evo - 2026-06-05: was 250 — +500ms so the "armed" buzz is clearly separated from the squeeze
    if (current_vib_pattern == 0) current_vib_pattern = 4;   // Pattern 4 after visual unlock completes
    DISP_LOCK(); displayDigitZone("r n"); updateDisplay(); DISP_UNLOCK();
    gpsKeepAliveDelay(2000);
  }
  else
  {
    // Double-squeeze: first unlock (no P4 yet), pause, then black screen, then wait for second squeeze
    // V2.5-Evo - 2026-10-07 - R-1: squeeze 2 needs its OWN release after squeeze 1. The two pauses
    // below watch the trigger too (ceremonyDelaySeeRelease()), so a rider who lets go and squeezes
    // again while the unlock animation is still playing is counted exactly as before.
    thr_released_seen = false;
    unlockAnimation();
    if (unlock_anim_release_seen) thr_released_seen = true;   // V2.5-Evo - 2026-10-07 - F-8: a release during the animation counts
    if (ceremonyDelaySeeRelease(250)) thr_released_seen = true;
    DISP_LOCK(); for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000; updateDisplay(); DISP_UNLOCK();
    if (ceremonyDelaySeeRelease(800)) thr_released_seen = true;

    bool second_ok = false;
    hold_ms = 0;
    const unsigned long wait2_start_ms = millis();   // V2.5-Evo - 2026-10-07 - defect b: second wait, cycle restarts on "r n"
    ceremonyWaitFrame(wait2_start_ms);
    while (millis() - rtm_arm_start_ms < (unsigned long)usrConf.rtm_arm_window_s * 1000UL ||
           (return_gesture_in_progress && ceremonyExtensionAllowed()))   // R-3: a started tap -> hold may finish; F-3: hard ceiling
    {
      ceremonyWaitFrame(wait2_start_ms);   // V2.5-Evo - 2026-10-07 - defect b: "r n" / arrow alternate (was advanceArrow() only)
      if (thr_scaled < 10) thr_released_seen = true;   // V2.5-Evo - 2026-10-07 - R-1: release seen
      if (thr_released_seen && thr_scaled > 76 &&     // V2.5-Evo - 2026-10-07 - R-1: counts only after a release
          millis() - rtm_arm_start_ms < (unsigned long)usrConf.rtm_arm_window_s * 1000UL)   // R-3: never past the window
      {
        if (hold_ms == 0) hold_ms = millis();
        if (millis() - hold_ms >= 500UL) { second_ok = true; hold_ms = 0; break; }
      }
      else { hold_ms = 0; }
      // V2.5-Evo - 2026-09-19 - the return gesture done again while waiting = cancel + flip (state 2).
      if (returnGestureCeremonyPoll(false)) { ceremonyCancelForReturnGesture(); return; }
      // V2.5-Evo - 2026-10-07 - M-3 / F-2: a plain 1 s LEFT hold = cancel to full manual, "St" + stop buzz.
      if (ceremonyLeftCancelPoll(false)) { ceremonyCancelToManual(); return; }
      delay(100);
      checkSerial();
    }
    if (!second_ok)
    {
      rtm_arm_gps_timeout_override = 0;  // ceremony aborted — restore normal GPS threshold
      // V2.5-Evo - 2026-10-07 - T-1: was rtm_thr_cap_tx = 255 unconditionally; a held trigger now keeps cap 0 until released.
      rtmCeremonyKeepCapIfHeld("arm timed out (no second squeeze)");
      DISP_LOCK(); for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000; updateDisplay(); DISP_UNLOCK();
      rtm_tx_state = RTM_IDLE;
      Serial.printf("RTM [TX] arm cancelled: no second squeeze within %u s\n", (unsigned)usrConf.rtm_arm_window_s);
      return;
    }

    // Second squeeze confirmed: unlock, pause, then Pattern 4 + "r n" arm confirm
    unlockAnimation();
    gpsKeepAliveDelay(750);   // V2.5-Evo - 2026-06-05: was 250 — +500ms so the "armed" buzz is clearly separated from the squeeze
    if (current_vib_pattern == 0) current_vib_pattern = 4;   // Pattern 4 after visual unlock completes
    DISP_LOCK(); displayDigitZone("r n"); updateDisplay(); DISP_UNLOCK();
    gpsKeepAliveDelay(2000);
  }

  // Pre-arm distance check: reject if already inside the disengage threshold
  float prearm_m = decodeRtmDistanceM();
  if (prearm_m >= 0.0f && prearm_m <= (float)usrConf.rtm_disengage_distance_m)
  {
    rtm_arm_gps_timeout_override = 0;  // restore GPS threshold — arm rejected
                                        // Finding 2-1: only exit path that previously left
                                        // the 4× override stale; all other exits
                                        // (timeouts + rtmDisengage) already clear it
    // V2.5-Evo - 2026-10-07 - T-1: THE BUG - this restored rtm_thr_cap_tx = 255 here, so a rider still squeezing to
    // summon a buggy that is already within the disengage distance had his held trigger passed straight to the motors
    // when the "St" ended. THE FIX (SOP-040 arrival rule): with the trigger held the cap stays 0 until one full release;
    // the "St" hold below watches for that release and lifts the cap at the sample it is seen.
    rtmCeremonyKeepCapIfHeld("arm refused (buggy inside the disengage distance)");
    // Pattern 7: one long buzz = the arm was REFUSED. Kept (an arm refusal is the rider believing
    // RTM is running when it is not), and routed through the pending flag like every other stop.
    vib_stop_pending = true;
    // Large-font stop confirm on arm rejection.
    DISP_LOCK(); displayDigits(LET_S, LET_T); updateDisplay(); DISP_UNLOCK();
    rtmCapHoldDelay(2000);   // T-1: was gpsKeepAliveDelay(2000); same 2 s, but a release lifts a kept cap at once
    rtm_tx_state  = RTM_IDLE;
    rtm_tx_active = false;
    queueMetaPacketBurst(0xF1, 0);
    Serial.printf("RTM [TX] arm refused: buggy is %.1f m away, inside the %u m disengage distance\n",
                  (double)prearm_m, (unsigned)usrConf.rtm_disengage_distance_m);
    return;
  }

  // Bug B fix: drain Serial1 NMEA backlog before activating.
  // loop() was suspended for the entire ceremony; gps_tx.encode() was not called.
  // gps_tx.location.age() has accumulated to ceremony duration (7-10s for double-squeeze).
  // Gate 2 threshold is rtm_gps_timeout_ms × 4 = 8000ms — a normal user exceeds this.
  // Processing the queued sentences refreshes age to near-zero before the first Gate 2 check.
  // 300ms cap prevents blocking indefinitely if module is sending continuous data.
  {
    unsigned long drain_start = millis();
    while (Serial1.available() && millis() - drain_start < 300UL)
    {
      gps_tx.encode(Serial1.read());
    }
  }

  // Activate RTM
  rtm_tx_state        = RTM_ACTIVE;
  rtm_active_start_ms = millis();
  rtm_tx_active       = true;
  rtm_release_ms      = 0;
  rtm_gate4_takeover_printed = false;   // V2.5-Evo - 2026-09-19 - Gate 4's stand-down notice is once per run
  rtmUnconfirmedReset();                // V2.5-Evo - 2026-10-07 - Q-2: fresh confirmation count for this run
  rtm_arm_dist_m      = decodeRtmDistanceM();
  if (rtm_arm_dist_m < 0.0f) rtm_arm_dist_m = 0.0f;
  queueMetaPacketBurst(0xF1, 1);
  Serial.printf("RTM [TX] ACTIVE: 0xF1/1 queued (buggy %.1f m away)\n", (double)rtm_arm_dist_m);
  // V2.5-Evo - 2026-09-18 - Belt for the single-slot burst queue: this ceremony blocked loop() for
  // 8 s or more, so an armed Follow-Me's 30 s keepalive may be overdue the instant loop() resumes.
  // Restart its clock now so it cannot fall due on top of the 0xF1/1 just queued. The keepalive
  // itself also refuses to queue while a burst is in flight (see runFmLoop()); this only keeps it
  // from trying. Only while FM is armed (fm_last_sync_ms > 0) - never starts a keepalive on its own.
  if (fm_last_sync_ms > 0) fm_last_sync_ms = millis();
}

// ============================================================
// V2.5-Evo - 2026-10-07 - H-1 (TX part): THE BUGGY IS STILL IN RETURN-TO-ME BUT THIS REMOTE IS NOT
// THE BUG: the buggy's RTM flag (rtm_rx_active) is set and cleared only by 0xF1 bursts, with no expiry. If
// the stop never arrived - link lost mid-return, the remote rebooted or slept mid-return, or all three burst
// packets were lost - the buggy stayed in RTM while the remote showed manual: with GPS fine a squeeze steered
// on its own with the stick ignored, with GPS dead the motor stayed at 0. Nothing on the remote could see it.
// THE FIX: the buggy reports the flag in telemetry.fm_status bit 1. While this remote is NOT running a return
// (IDLE or COOLDOWN, never ARMED or ACTIVE) and the link is fresh, a set bit that ARRIVED after our last 0xF1/0
// went on the air (plus kRtmStopEchoMarginMs for the buggy to process it) means the buggy did not hear the
// stop: queue 0xF1/0 again. The arrival test makes the retry follow the telemetry rotation (one try per fresh
// report, ~2-3 s) instead of re-sending every loop tick, and it keeps retrying until the bit clears.
// Runs whatever the remote's own RTM enable says, and while locked, because a stuck buggy is the problem
// either way. Can only ever send "RTM off". Loop task only.
// Inputs: rtm_tx_state, usrConf.paired, radio activity, last_packet, telemetry.fm_status,
//   fm_status_arrival_ms, rtm_stop_sent_ms. Side effects: may queue 0xF1/0 and print one line.
// ============================================================
static const unsigned long kRtmStopEchoMarginMs = 300UL;   // buggy handles 0xF1 at once; telemetry refresh is 100 ms

static void rtmRxStateWatch(unsigned long now)
{
  if (rtm_tx_state == RTM_ACTIVE || rtm_tx_state == RTM_ARMED) return;     // our own return, or its ceremony
  if (!usrConf.paired || !isRadioActivityEnabled()) return;
  if (last_packet == 0 || (now - last_packet) >= FM_LINK_HEALTHY_MS) return; // link not fresh: nothing to trust
  if ((telemetry.fm_status & FM_STATUS_RTM_ACTIVE) == 0) return;              // the buggy is not in RTM
  const unsigned long arrived = fm_status_arrival_ms;
  const unsigned long sent    = rtm_stop_sent_ms;
  if (arrived == 0) return;                                                   // never actually received the byte
  if (sent != 0 && (long)(arrived - sent) < (long)kRtmStopEchoMarginMs) return; // report predates our last stop
  Serial.println("RTM [TX] the buggy reports Return-To-Me active but this remote is not running it -> 0xF1/0");
  queueMetaPacketBurst(0xF1, 0);
  rtm_stop_sent_ms = now;   // hold off the next try until this burst is out and a fresh report arrives
}

// ============================================================
// V2.5-Evo - 2026-10-07 - H-1 (TX part): FLUSH "RTM OFF" AND "FOLLOW-ME OFF" BEFORE GOING QUIET
// THE BUG: deepSleep() switched the radio off with nothing sent, so a buggy in Return-To-Me or Follow-Me
// kept that state with no remote behind it (contrary to the 2026-07-20 "flush 0xF1/0 before radio teardown").
// THE FIX: queue 0xF1/0 and 0xF2/0 (both fit in the 2-deep queue) and wait until they have gone out:
// at least kStopFlushMinMs (400 ms), until the queue is empty, at most kStopFlushMaxMs. Two bursts of three
// packets at the 100 ms cadence take about 600 ms. Used by deepSleep() (System.ino) and the lock gesture
// (Hall.ino). The 0xF2 value goes through fmEncodeModeByte(0) like every other Follow-Me disarm.
// Does nothing if the remote is unpaired or its radio is off (nothing can be sent).
// BLOCKING up to kStopFlushMaxMs on the loop task. The radio and ADC tasks keep running; throttle is not
// affected (meta packets replace control packets for those cycles, exactly as any burst does).
// Not static: Hall.ino and System.ino call it.
// ============================================================
static const unsigned long kStopFlushMinMs = 400UL;
// V2.5-Evo - 2026-10-07 - T-6: THE BUG - at the degraded 200 ms cadence (plus jitter) two bursts and the forced control
// packets between them can take 0.8-1.0 s, so a 1000 ms ceiling could give up before the second burst was out. THE
// FIX: the ceiling is 1500 ms. The wait still ends as soon as the queue is empty (after the 400 ms minimum).
static const unsigned long kStopFlushMaxMs = 1500UL;

void rtmFmStopFlush()
{
  if (!usrConf.paired || !isRadioActivityEnabled()) return;
  queueMetaPacketBurst(0xF1, 0);
  queueMetaPacketBurst(0xF2, fmEncodeModeByte(0));
  unsigned long start = millis();
  while (millis() - start < kStopFlushMaxMs)
  {
    if (millis() - start >= kStopFlushMinMs && rtm_meta_count.load(std::memory_order_acquire) == 0) break;
    delay(10);
  }
  Serial.println("RTM [TX] stop flush: 0xF1/0 + 0xF2/0 sent before going quiet");
}

// ============================================================
// V2.5-Evo - 2026-10-07 - S-8: THIS REMOTE'S BOOT ID (so the buggy can tell that the remote was switched off and on)
// THE PROBLEM: a remote that is power-cycled mid-return looks to the buggy exactly like one that never went away, so a
// standing return-to-me or auto-return carried on with nobody's intent behind it (SOP-040 auto-return rule 5).
// THE FIX: at every power-on the remote picks a random 7-bit ID and sends it as an 0xF1 packet whose VALUE is
// 0x80 | id. The buggy remembers the last ID; a DIFFERENT one means the remote rebooted, and the buggy cancels any
// standing return. Every RX firmware handles 0xF1 values 0 and 1 only and ignores every other value, so an older buggy
// simply ignores the packet.
// The ID must differ from the previous power-on's (a repeat would hide the reboot), so the last one is kept in NVS
// (Preferences namespace "bremote_tx", key "boot_id") - NOT in confStruct, so no struct change and no SPIFFS write.
// If NVS cannot be opened the ID is still random (1 in 128 chance of a repeat) and one line says so.
// Timing: a 3-packet burst queued in setup() before the radio task starts, so it is the first thing sent (after the
// unlock on a remote that boots locked); then ONE packet whenever the link becomes fresh and every 10 s while it stays
// fresh. These are lowest priority: queueMetaPacketIfFree() puts them only into a free slot and never over a pending
// 0xF1/0 or 0xF1/1 (a refused one is retried on the next tick).
// ============================================================
static uint8_t       tx_boot_id         = 0;    // 0-127, chosen once per power-on
static unsigned long tx_boot_id_last_ms = 0;    // millis() of the last repeat queued; 0 = send at the next fresh link
static const unsigned long kTxBootIdRepeatMs = 10000UL;

// txBootIdInit - choose this power-on's boot ID (different from the last one stored in NVS), store it, and queue the
// first 0xF1/(0x80|id) burst. Called once from setup() before initTasks(). Inputs: NVS, esp_random().
// Side effects: one NVS write per power-on, one queued burst, one serial line. Not static: setup() calls it.
void txBootIdInit()
{
  Preferences prefs;
  uint8_t last = 0xFF;                                   // 0xFF = none stored (never a valid 7-bit ID)
  const bool nvs_ok = prefs.begin("bremote_tx", false);
  if (nvs_ok) last = prefs.getUChar("boot_id", 0xFF);
  uint8_t id = (uint8_t)(esp_random() & 0x7F);
  if (id == last) id = (uint8_t)((id + 1 + (esp_random() % 126)) & 0x7F);   // offset 1..126: can never equal last
  if (nvs_ok)
  {
    // V2.5-Evo - 2026-10-07 - T-7: a failed write was silent. putUChar() returns the bytes written (0 = failed); the
    // ID is still used, but the next power-on may pick the same one, so say so.
    if (prefs.putUChar("boot_id", id) == 0)
      Serial.println("RTM [TX] boot ID: NVS write failed - the next power-on may repeat this ID");
    prefs.end();
  }
  tx_boot_id = id;
  queueMetaPacketIfFree(0xF1, (uint8_t)(0x80 | id), 3);
  if (last == 0xFF) Serial.printf("RTM [TX] boot ID %u (none stored before) queued as 0xF1/0x%02X\n", (unsigned)id, (unsigned)(0x80 | id));
  else              Serial.printf("RTM [TX] boot ID %u (previous %u) queued as 0xF1/0x%02X\n", (unsigned)id, (unsigned)last, (unsigned)(0x80 | id));
  if (!nvs_ok) Serial.println("RTM [TX] boot ID: NVS not available - the ID is random but may repeat the last one");
}

// txBootIdTick - repeat the boot ID while the link is fresh (once when it becomes fresh, then every 10 s).
// Inputs: now, usrConf.paired, radio activity, last_packet. Side effects: may queue one 0xF1 packet. Loop task only.
static void txBootIdTick(unsigned long now)
{
  if (!usrConf.paired || !isRadioActivityEnabled()) return;
  // V2.5-Evo - 2026-10-07 - signed: waitForTelemetry can stamp last_packet after `now` was read (see rtmRefreshTick()).
  const bool link_fresh = (last_packet != 0) && ((long)(now - last_packet) < (long)FM_LINK_HEALTHY_MS);
  if (!link_fresh) { tx_boot_id_last_ms = 0; return; }   // send again as soon as the link comes back
  // V2.5-Evo - 2026-10-07 - T-8: THE BUG - the repeat timer was stamped when the boot ID was QUEUED, and a state burst
  // (0xF1/0 or 0xF1/1) queued after it replaces it in place, so a boot ID could be lost without ever going on the air
  // and the next try was 10 s away. THE FIX: once it has left the queue, check the send stamp (tx_boot_id_sent_ms,
  // written by sendData). Not sent since it was queued -> it was overwritten: re-arm now, as a 3-packet burst.
  if (tx_boot_id_last_ms != 0 && !metaQueuePending(0xF1, (uint8_t)(0x80 | tx_boot_id), false) &&
      (long)(tx_boot_id_sent_ms - tx_boot_id_last_ms) < 0)
  {
    tx_boot_id_last_ms = 0;
    Serial.println("RTM [TX] boot ID was replaced in the queue before it went out -> sent again");
  }
  if (tx_boot_id_last_ms != 0 && (now - tx_boot_id_last_ms) < kTxBootIdRepeatMs) return;
  // V2.5-Evo - 2026-10-07 - S-8 follow-up: right after a link gap the buggy may have lost the ID (an RX reboot) and a
  // parked auto-return reads not-ready until it hears one, so the first send after a gap is a full 3-packet burst;
  // the 10 s repeats on a steady link stay one packet each (every meta packet replaces a control packet).
  const uint8_t sends = (tx_boot_id_last_ms == 0) ? 3 : 1;
  if (queueMetaPacketIfFree(0xF1, (uint8_t)(0x80 | tx_boot_id), sends)) tx_boot_id_last_ms = (now != 0) ? now : 1;
}

// ============================================================
// V2.5-Evo - 2026-10-07 - H-1: THE RTM REFRESH ("this remote is still running Return-To-Me")
// THE PROBLEM: the buggy's RTM flag is set by one 0xF1/1 burst and has no expiry, so if every later 0xF1/0 is lost
// (link loss, a remote that went quiet) the buggy keeps returning with nobody's intent behind it.
// THE FIX: while RTM_ACTIVE this remote sends ONE 0xF1 packet with VALUE 0x02 about once a second, starting at least
// 1 s after the last 0xF1/1 packet went out (the activation burst has drained). The buggy ends RTM on a fault if a
// remote that has refreshed once stops refreshing for 5 s. A refresh never ACTIVATES RTM on the buggy, and an older RX
// ignores value 2. It stops on every RTM end, because it is only queued from the RTM_ACTIVE case; a refresh still
// queued when RTM ends is overwritten in place by the 0xF1/0 rtmDisengage() queues (same-type rule).
// Free slot only (queueMetaPacketIfFree()), so it can never replace a pending 0xF1/0 or 0xF1/1.
// ============================================================
static unsigned long rtm_refresh_last_ms = 0;   // millis() of the last refresh queued
static const unsigned long kRtmRefreshPeriodMs = 1000UL;

// rtmRefreshTick - queue the next refresh if one is due. Inputs: now, rtm_start_sent_ms, rtm_active_start_ms,
// rtm_refresh_last_ms, the queue. Side effects: may queue one 0xF1/0x02. Called only from the RTM_ACTIVE case.
static void rtmRefreshTick(unsigned long now)
{
  const unsigned long started = rtm_start_sent_ms;
  if (started == 0 || (long)(started - rtm_active_start_ms) < 0) return;   // this run's 0xF1/1 is not on the air yet
  if (metaQueuePending(0xF1, 1, false)) return;                             // the activation burst is still draining
  // V2.5-Evo - 2026-10-07 - signed: sendData (higher priority) can stamp `started` AFTER `now` was read; an unsigned
  // now - started would then wrap to ~49 days and send the refresh at once, on top of the activation burst.
  if ((long)(now - started) < (long)kRtmRefreshPeriodMs) return;           // >= 1 s after its last packet
  if ((long)(rtm_refresh_last_ms - rtm_active_start_ms) >= 0 &&
      (now - rtm_refresh_last_ms) < kRtmRefreshPeriodMs) return;           // one per second within this run
  if (queueMetaPacketIfFree(0xF1, 0x02, 1)) rtm_refresh_last_ms = now;
}

// ---- Called from loop() every ~110ms ----
void runRtmLoop()
{
  // V2.5-Evo - 2026-10-07 - S-8: boot ID repeats, whatever the RTM enable says (a reboot matters to auto-return too).
  txBootIdTick(millis());
  // V2.5-Evo - 2026-10-07 - H-1: runs BEFORE the enable check below, on purpose - see rtmRxStateWatch().
  rtmRxStateWatch(millis());
  // V2.5-Evo - 2026-10-07 - A-1: the kept arrival cap is lifted on a full release whatever the RTM enable says.
  rtmArrivalCapUpdate();

  // V2.5-Evo - 2026-09-30 - the EFFECTIVE enable (see rtmEnabledEffective()). A session override that
  // says OFF parks this whole state machine exactly as a stored 0 always did; it cannot leave a run
  // half-supervised, because the flip is refused outright while RTM is active or arming.
  if (!rtmEnabledEffective() || !usrConf.gps_en) return;

  unsigned long now = millis();

  switch (rtm_tx_state)
  {
    // ---- IDLE: nothing to do; setRtmArmed() transitions out ----
    case RTM_IDLE:
      break;

    // ---- ARMED: dead code — Bug4 moved all arm logic into runDoubleSqueezeArm() ----
    // setRtmArmed() now calls runDoubleSqueezeArm() blocking for both single and double squeeze.
    // On return rtm_tx_state is RTM_ACTIVE or RTM_IDLE — this case is never reached.
    case RTM_ARMED:
      rtm_tx_state = RTM_IDLE;
      break;

    // ---- SQUEEZE_WAIT: dead code path (Change 5) ----
    // Double-squeeze arm is now fully blocking in runDoubleSqueezeArm(), called from setRtmArmed().
    // This state can no longer be entered; retained for enum completeness only.
    case RTM_SQUEEZE_WAIT:
      rtm_tx_state = RTM_IDLE;
      break;

    // ---- ACTIVE: RTM running ----
    case RTM_ACTIVE:
    {
      // Update throttle cap for ramp
      rtm_thr_cap_tx = calcRtmThrottleCap();

      // V2.5-Evo - 2026-10-07 - A-1: THE BUGGY ENDED THE RETURN ITSELF. Checked first: if the buggy is no longer
      // returning, none of the remote's own gates below has anything left to supervise.
      // V2.5-Evo - 2026-10-07 - Q-3 / Q-1: THE BUGGY NOW SAYS HOW IT ENDED (telemetry.rx_state_flags, index 19, rising
      // edges latched in Radio.ino; acted on only when the rise came after this run went ACTIVE):
      //   bit 0 FAULT (Phase C, its gate timeout, the refresh expiry) -> rtmDisengage(false): "St" + the stop buzz,
      //         0xF1/0, cap 255 - a fault hands back full manual control (SOP-039 rule 2).
      //         V2.5-Evo - 2026-10-08 - now the cap in force is KEPT until the raw trigger drops below 26 (sticky cap).
      //   bit 1 ARRIVED (its stop distance)                            -> rtmDisengage(true): silent "St", 0xF1/0, and the
      //         throttle cap goes to 0 until the trigger is fully released once (SOP-040 arrival rule, Q-1).
      // FALLBACK (an older RX that sends no index 19, or a lost edge): the buggy CONFIRMED RTM during this run (an
      // fm_status arrival with bit 1 set after ACTIVE began) and then reported it OFF on 2 consecutive arrivals of the
      // byte. It ends as a fault if the last rx_state_flags shows bit 0, otherwise as an arrival (cap 0 until release).
      // Q-2: while the buggy has NOT confirmed, rtmUnconfirmedCheck() re-sends 0xF1/1 once, then refuses.
      {
        const bool rx_confirmed = (fm_status_rtm_on_ms != 0) &&
                                  ((long)(fm_status_rtm_on_ms - rtm_active_start_ms) > 0);
        const unsigned long fault_rise  = rx_rtm_fault_rise_ms;
        const unsigned long arrive_rise = rx_rtm_arrived_rise_ms;
        const bool rx_fault_now   = (fault_rise  != 0) && ((long)(fault_rise  - rtm_active_start_ms) > 0);
        const bool rx_arrived_now = (arrive_rise != 0) && ((long)(arrive_rise - rtm_active_start_ms) > 0);
        // V2.5-Evo - 2026-10-08 - STICKY RETURN CAP: a fault end no longer hands back full manual at once. The RTM cap in
        // force is KEPT (rtm_end_sticky) until the raw trigger drops below 26 once; the buggy holds its own live limit.
        if (rx_fault_now)
        {
          Serial.println("RTM [TX] the buggy ended Return-To-Me on a FAULT (rx_state_flags bit 0) -> St + stop buzz; steering is yours, the RTM throttle cap stays until the trigger drops below 10 %");
          rtm_end_cap_value = rtm_thr_cap_tx.load();   // V2.5-Evo - 2026-10-08 - keep the cap in force
          rtm_end_keep_cap  = true;
          rtm_end_sticky    = true;
          rtmDisengage(false);
          break;
        }
        if (rx_arrived_now)
        {
          Serial.println("RTM [TX] the buggy ARRIVED (rx_state_flags bit 1) -> silent St; throttle cap 0 until the trigger is fully released");
          rtm_end_cap_value = 0;
          rtm_end_keep_cap  = true;
          rtmDisengage(true);
          break;
        }
        if (rx_confirmed && fm_status_rtm_off_streak >= 2)
        {
          if (telemetry.rx_state_flags & RX_STATE_RTM_FAULT)
          {
            Serial.println("RTM [TX] the buggy ended Return-To-Me (fm_status bit 1 off on 2 arrivals, fault bit set) -> St + stop buzz; the RTM throttle cap stays until the trigger drops below 10 %");
            rtm_end_cap_value = rtm_thr_cap_tx.load();   // V2.5-Evo - 2026-10-08 - sticky: keep the cap in force
            rtm_end_keep_cap  = true;
            rtm_end_sticky    = true;
            rtmDisengage(false);
          }
          else
          {
            Serial.println("RTM [TX] the buggy ended Return-To-Me (fm_status bit 1 off on 2 arrivals) -> silent St; throttle cap 0 until the trigger is fully released");
            rtm_end_cap_value = 0;
            rtm_end_keep_cap  = true;
            rtmDisengage(true);
          }
          break;
        }
        if (!rx_confirmed && rtmUnconfirmedCheck()) break;   // Q-2 refusal ended the run
      }

      // Gate 1: max runtime (0 = disabled — safety gates handle all real scenarios)
      if (usrConf.rtm_max_runtime_s > 0 &&
          now - rtm_active_start_ms > (unsigned long)usrConf.rtm_max_runtime_s * 1000UL)
      {
        // BUZZES — and it is the one timeout in this file that does. Every other timeout here can
        // only fire because the trigger was already released; this gate has no throttle
        // precondition, so it fires mid-squeeze, and rtmDisengage() lifts rtm_thr_cap_tx (30-90%
        // by config range) back to 255 in the same instant. That is a silent step from capped RTM
        // throttle to raw manual throttle with no gesture behind it. Off by default
        // (rtm_max_runtime_s = 0), so keeping this buzz costs nothing against buzz saturation.
        // V2.5-Evo - 2026-10-08 - STICKY RETURN CAP: that silent step to raw manual throttle is gone. The cap in force is
        // KEPT until the raw trigger drops below 26 once (the buggy keeps the return's limit on 0xF1/0 too). The buzz stays.
        rtm_end_cap_value = rtm_thr_cap_tx.load();
        rtm_end_keep_cap  = true;
        rtm_end_sticky    = true;
        rtmDisengage(false);
        break;
      }

      // Gate 2: TX GPS freshness — use 4× relaxed threshold in the cycles immediately after
      // the blocking arm ceremony (GPS age may be high; rtm_arm_gps_timeout_override > 0
      // until rtmDisengage() clears it on any RTM_ACTIVE exit path).
      {
        uint32_t gps_thr = (rtm_arm_gps_timeout_override > 0)
                           ? rtm_arm_gps_timeout_override
                           : (uint32_t)usrConf.rtm_gps_timeout_ms;
        if (gps_tx.location.age() > gps_thr)
        {
          // V2.5-Evo - 2026-10-08 - STICKY RETURN CAP: keep the cap in force until the raw trigger drops below 26 once,
          // instead of lifting it to 255 under a held trigger (the buggy keeps the return's limit on 0xF1/0).
          rtm_end_cap_value = rtm_thr_cap_tx.load();
          rtm_end_keep_cap  = true;
          rtm_end_sticky    = true;
          rtmDisengage(false);   // FAULT, not a timeout: the TX GPS died under RTM and nothing the
                                 // rider did explains the stop → buzz
          break;
        }
      }

      // Gate 3: throttle release timeout — 4s gives time to re-apply before RTM disengages.
      // Kept hardcoded (not SPIFFS) by design. Total re-arm window after release = ~6s (4s gate + 2s cooldown).
      // V2.5-Evo - 2026-04-28 - P9 Bug1D: now calls rtmDisengage() so Pattern 4 + "St P" fire.
      if (thr_scaled < 10)
      {
        if (rtm_release_ms == 0) rtm_release_ms = now;
        if (now - rtm_release_ms > 4000UL)
        {
          // SILENT (2026-08-17 rule: a pure timeout is silent, a fault buzzes). This gate can only
          // be reached because the rider let go and kept the trigger released for 4s — the stop
          // follows his own hand and "St" already shows it. It is also the moment the cap lift in
          // rtmDisengage() is harmless: thr_scaled < 10 means the shaped throttle sits far below
          // the RTM cap, so restoring the cap to 255 changes nothing he can feel. Buzzing here
          // spends the rider's attention on a non-event and devalues the fault buzz.
          rtmDisengage(true);
          break;
        }
      }
      else
      {
        rtm_release_ms = 0;
      }

      // Gate 4: steering exit (P8 — any significant steering input exited RTM). SUPERSEDED 2026-10-07 by Q-5 below:
      // the gate never exits any more; the 2026-09-19 note that follows is kept as history.
      // V2.5-Evo - 2026-09-19 - THE GATE FOLLOWS THE BUGGY. usrConf.rtm_steer_exit_on_input is no
      // longer read (deprecated, see BREmote_V2_Tx.h). Whether the stick cancels or takes over an
      // automatic return is the buggy's steer_during_auto setting, echoed in fm_flags bit 4 and
      // read here only while the link is fresh (FM_LINK_HEALTHY_MS, as every other flag): bit
      // clear, link stale, or an old RX -> the push exits RTM exactly as before; bit set -> the
      // buggy is steering by the rider's stick and will resume on its own when the stick centres,
      // so the remote must NOT end the run - the gate stands down and says so once per run. During
      // the ~2 s handshake after a setting change the two boards may disagree for one telemetry
      // rotation; both directions fail to cancel or to ignore, never to an unguarded takeover.
      // V2.5-Evo - 2026-10-07 - Q-5: GATE 4 ALWAYS STANDS DOWN. THE BUG: with the buggy echoing steer_during_auto 0 (or a
      // stale link) a push past 20 counts ended the return - with the trigger HELD - and lifted the cap to 255 mid-squeeze.
      // Owner rule (SOP-040, SOP-039 rule 7): steering is an aid, never an exit; a manual return ends only by arriving, a
      // fault, the trigger released 4 s (Gate 3), the lock, or a remote power cycle. The remote now never exits on the
      // stick; whether the stick cancels or takes over the automatic STEERING is the buggy's own steer_during_auto
      // setting, applied on the buggy. One serial line per run when the stick is first pushed.
      if (toggle_blocked_by_steer && abs((int)steer_scaled - 127) > 20 && !rtm_gate4_takeover_printed)
      {
        rtm_gate4_takeover_printed = true;
        Serial.println("RTM [TX] Gate 4: stick pushed - steering never ends a return on this remote; the buggy's steer_during_auto decides how the stick steers it");
      }

      // V2.5-Evo - 2026-10-07 - H-1: still ACTIVE after every gate - send the once-a-second refresh if one is due.
      rtmRefreshTick(now);

      // Display handled by renderRtmInfoDisplay() in loop() when rtm_tx_active==true
      break;
    }

    // ---- COOLDOWN: wait 2s then return to IDLE ----
    // showFullScreenMessage("St P", 2000) was called at disengage moment; by the time
    // this state is polled, the 2s has already elapsed → transition to IDLE immediately.
    case RTM_COOLDOWN:
      if (now - rtm_cooldown_ms > 2000UL)
      {
        rtm_tx_state = RTM_IDLE;
      }
      break;
  }
}

// ============================================================
// FM STATE MACHINE
// V2.5-Evo - 2026-04-27 - P8.1: FM redesigned as arm/disarm toggle with persistent mode memory.
//
// ARM (LEFT tap + RIGHT hold 5s, first time or after disarm):
//   - Arms at last_fm_mode (RAM; defaults to F1 on power cycle — never resets to F0/disabled)
//   - Blinks "F[mode]" x2 on display; fires Pattern 4 (2 fast buzzes) as arm confirm
//   - FM active: user engages throttle to ride
//
// CHANGE MODE while armed (LEFT hold 2s, intercepted by Hall.ino):
//   - Cycles F1→F2→F3→F4→F5→F1 (F4/F5 since 2026-10-07), never F0 (owner rule 2026-09-19: nothing disarms Follow-Me by cycling);
//     stays armed; sends new mode to RX; resets arm timer
//
// DISARM (any of):
//   - Same combo again (LEFT tap + RIGHT hold 5s) — toggle
//   - Arm timeout expires (fm_arm_timeout_s) before any throttle input — auto-disarm.
//     0 = never (the default since 2026-09-17): armed with no throttle stays armed indefinitely.
//   - RX fault-stop (fm_flags bit3 rising edge) — the TX follows the RX's decision
//
// V2.5-Evo - 2026-09-18 - NOT a disarm any more: arming RTM. Follow-Me stays armed through a
// Return-to-Me; the RX yields while RTM is active and resumes ARMED (unlatched) when it ends.
// See setRtmArmed(). While RTM runs, loop() renders the RTM display as before (Display.ino checks
// rtm_tx_active first on every FM-aware path), so the rider sees RTM, and FM reappears armed after.
//
// V2.5-Evo - 2026-09-17 - Gate1-REMOVED. There is no longer a TX-side throttle-release disarm.
// The old Gate 1 (kFmGate1ReleaseMs, 30 s off the trigger after the first squeeze) contradicted
// the always-armed philosophy: a rider who floats, swims or waits between runs for more than
// 30 s was silently disarmed and had to re-do the gesture. The RX already guards the motion
// path on its own — the escalation chain is now RX latch-clear 10 s -> RX mode-age 95 s — so
// the TX keeps its arm declaration until the rider disarms, the RX faults, or power is cut.
// ============================================================

volatile bool        fm_armed         = false;  // FM arm state; RAM only, cleared on power cycle. Not static — extern'd by Display.ino (R5 bar)
                                                 // volatile: read by updateBargraphs() (task), written by loop()
static uint8_t       last_fm_mode     = 1;      // last active FM station (1-5 since the F4/F5 port: 4 front-right, 5 front-left); defaults F1; RAM only
static unsigned long fm_arm_ms        = 0;      // time of arm, or time of last throttle >10 while armed
static bool          fm_throttle_seen = false;  // becomes true once thr_scaled>10 after arming
// V2.5-Evo - 2026-10-07 - SOP-041 rule 3 resync: millis() of this remote's last Follow-Me ARM declaration (cycleFmMode()
// arm path). The buggy needs the 0xF2 burst plus one telemetry rotation before it can report bit 0, so "not armed"
// reports that began within kFmResyncDeclareMarginMs of the arm are not evidence. Loop task only.
static unsigned long fm_declare_ms    = 0;
static const unsigned long kFmResyncMs              = 3000UL;   // bit 0 clear for this long on a fresh link
static const unsigned long kFmResyncDeclareMarginMs = 1500UL;

// Returns true if FM is currently armed; called by Hall.ino to intercept LEFT hold 2s
bool isFmArmed() { return fm_armed; }

// V2.5-Evo - 2026-07-20 - Batch T: a previous telemetry.fm_flags snapshot (fm_flags_prev) lived here for the
// bit3 (fault-stop) rising-edge test in runFmLoop().
// V2.5-Evo - 2026-10-07 - R-6: removed. The edge is now latched on the byte's arrival in Radio.ino
// (fm_fault_latched, BREmote_V2_Tx.h), so a long loop() stall can no longer hide it.

// V2.5-Evo - 2026-10-07 - the FM warning-distance haptic (Pattern 8, one 300 ms pulse every 2 s while the buggy
// was at or beyond fm_warn_distance_m) is REMOVED by owner ruling (minimal-buzz rule), together with its
// scheduler state (kFmDistanceWarningPeriodMs, fm_warning_last_ms, fm_warning_sent, fmResetWarningScheduler()).
// usrConf.fm_warn_distance_m STAYS: the R5 proximity bar uses it as its distance full-scale (Display.ino,
// updateR5ProximityBar()), and ConfigService.ino still validates it. No struct change.

// ============================================================
// V2.5-Evo - 2026-07-20 - Batch T (FM design v1.4): FM arm-time and display readiness gating.
// All inputs are TX-LOCAL (paired flag, own GPS fix/age, last-reply age) plus the RX's own
// armed-not-ready bit — instant, zero telemetry dependency, no new confStruct field. Called
// only from the loop task: fmFundamentalReject() from cycleFmMode(), fmArmedNotReady()
// from updateR5ProximityBar() via the render path. gps_tx access stays on the loop task (invariant).
// ============================================================

// FUNDAMENTAL arm-time reject — the three conditions under which arming would be a lie AND a
// rider genuinely mid-tow cannot be in, so false-refusal ≈ 0. Everything else (transient GPS
// staleness, a momentary packet gap, high HDOP) is NOT fundamental: it arms as NOT-READY, so a
// brief mid-tow glitch never blocks the magnet. Returns true = refuse the fresh arm.
bool fmFundamentalReject()
{
  if (!usrConf.paired)                return true;   // never paired
  if (last_packet == 0)               return true;   // no RX packet ever this session
  if (!gps_tx.location.isValid())     return true;   // no TX GPS fix ever this session (isValid latches on first fix)
  return false;
}

// ARMED-NOT-READY vs READY — the OR of the RX's armed-not-ready bit (bit2) with the TX-local
// transient readiness set. Any not-ready input → true (scanner blinks in place). None → false
// (scanner sweeps). Flips live each render, so the "now it's good" moment appears for free.
bool fmArmedNotReady()
{
  if (telemetry.fm_flags & FM_FLAG_NOTREADY)                          return true;  // RX half: latch not yet proven
  if (!usrConf.paired)                                               return true;  // pairing lost
  if (last_packet == 0 || (millis() - last_packet) >= FM_LINK_HEALTHY_MS) return true;  // RX telemetry not recent / link unhealthy
  if (!(gps_tx.location.isValid() &&
        gps_tx.location.age() < (unsigned long)usrConf.tx_gps_stale_timeout_ms))  return true;  // TX GPS fix missing or stale
  return false;  // all TX-local inputs fresh AND RX not-ready clear → READY → sweep
}

// Silent disarm: clears FM state and notifies RX, but shows no display and fires no haptic.
// Used when arm-window expires before any throttle input — nothing active to confirm.
static void fmSilentDisarm()
{
  fm_armed         = false;
  fm_throttle_seen = false;
  fm_last_sync_ms  = 0;
  last_fm_return_mode = 0xFF;      // V2.5-Evo - 2026-09-19 - the override ends with the declaration (bits 5-6 = 00)
  queueMetaPacketBurst(0xF2, fmEncodeModeByte(0));   // mode 0 = FM disabled on RX
  // V2.5-Evo - 2026-10-07 - defect c: clear the whole R5 row here too (arm-timeout and the F-1 silent disarm), so no
  // C7-C9 scanner/bar pixel outlives Follow-Me. During an active return the RTM bar redraws R5 on the next render.
  DISP_LOCK(); displayBuffer[6] = 0x0000; DISP_UNLOCK();
}

// Internal disarm: clears state, notifies RX, shows "St" full-screen, buzzes only on a FAULT.
// V2.5-Evo - 2026-04-28 - P9 S2: showFmMode() removed; disarm shows blocking stop message.
// V2.5-Evo - 2026-04-28 - ChgE: fm_last_sync_ms reset to 0 on disarm so keepalive timer clears.
// INPUT: commanded — true = stay SILENT, false = fire the Pattern 7 STOP buzz. Since the 2026-08-17
//        revision the test is "FAULT or TIMEOUT", not "did the rider press something": true for the
//        deliberate disarms (toggle combo, magnet toggle); false only for the RX fault-stop.
//        (The Gate 1 release backstop that also passed true was removed 2026-09-17.)
// OUTPUT: none. SIDE EFFECTS: fm_armed cleared, keepalive stopped, 0xF2/0 sent to RX, STOP buzz
//        requested when uncommanded, and a BLOCKING 2s "St" display hold.
static void fmDisarm(bool commanded)
{
  fm_armed         = false;
  fm_throttle_seen = false;
  fm_last_sync_ms  = 0;            // Change E: clear keepalive timer
  last_fm_return_mode = 0xFF;      // V2.5-Evo - 2026-09-19 - the override ends with the declaration (bits 5-6 = 00)
  queueMetaPacketBurst(0xF2, fmEncodeModeByte(0));   // mode 0 = FM disabled on RX (followme_mode=0)
  // V2.5-Evo - 2026-08-16 - HAPTIC CUT: silent on a DELIBERATE disarm. You just did it, and the
  // display already says so. A buzz confirming your own action is noise.
  // V2.5-Evo - 2026-08-17 - StopBuzz: the caller now decides, on the rule A PURE TIMEOUT IS SILENT,
  // A FAULT BUZZES. Exactly ONE FM path is a fault today — the RX fault-stop in runFmLoop(), where
  // the RX gave up steering on its own and the rider has no way to know. Buzz saturation is the real
  // failure mode here — the more buzzes there are, the less any one of them is read — so Pattern 7
  // is spent on faults only.
  // V2.5-Evo - 2026-09-17 - History: from 2026-07-20 to 2026-09-17 a Gate 1 release backstop (a 30 s
  // off-the-trigger disarm timer) was also classified commanded=true for this same reason. It was
  // removed because Scenario A's rider may legitimately release the trigger for up to 30 s while
  // linking waves with the buggy already parked behind the break — the whip is a separate
  // engagement gesture, not part of this release window. The RX 10 s latch clear and 95 s
  // mode-age expiry are the backstops now.
  if (!commanded) vib_stop_pending = true;   // Pattern 7: one long buzz = a FAULT stopped the system

  // Large-font stop confirm on FM disarm.
  // V2.5-Evo - 2026-10-07 - DEFECT c: THE BUG - displayDigits() clears only C0-C6 of the R5 row (displayBuffer[6]),
  // and the Follow-Me R5 bar / scanner also lights C7-C9 there. Nothing redraws R5 once Follow-Me is off, so those
  // pixels stayed lit on the normal screen after a disarm - a leftover "following" look (SOP-041 rule 5). THE FIX:
  // clear the whole R5 row (as rtmDisengage() already does for the RTM bar) before the "St".
  DISP_LOCK(); displayBuffer[6] = 0x0000; displayDigits(LET_S, LET_T); updateDisplay(); DISP_UNLOCK();
  gpsKeepAliveDelay(2000);
}

// Called by handleGearToggle() combo (LEFT tap + RIGHT hold 5s) — toggles arm/disarm.
// On arm: seeds last_fm_mode from SPIFFS on first arm this session (Change B); fires Pattern 4;
// shows "FM" confirm (Change D, 6 cols ≤ C0-C5); sends 0xF2 to RX; starts keepalive timer (Change E).
void cycleFmMode()
{
  if (!usrConf.fm_override_enabled || !usrConf.gps_en) return;

  if (fm_armed)
  {
    if (fm_throttle_seen)
    {
      // V2.5-Evo - 2026-10-07 - T-9: THE BUG - this deliberate disarm also ended an auto-return that was under way
      // (the buggy dropped Follow-Me mid-return), but SOP-040 rule 4 says auto-return ends only by arriving, a fault or
      // a remote power cycle. THE FIX: while the buggy confirms it is RETURNING (fmIsReturning()) the gesture is ignored,
      // serial line only. Parked (waiting) or following, the disarm works exactly as before.
      if (fmIsReturning())
      {
        Serial.println("FM [TX] disarm gesture ignored: the buggy is on an auto-return back to you (it ends by arriving, a fault or a remote power cycle)");
        return;
      }
      // User already rode — treat gesture as disarm toggle
      Serial.println("FM [TX] disarm: the disarm gesture after riding -> 0xF2/0");   // V2.5-Evo - 2026-09-19
      fmDisarm(true);   // COMMANDED: the rider made the disarm gesture → silent
    }
    else
    {
      // No throttle yet — cycle to next mode. V2.5-Evo - 2026-09-19 (owner ruling 12:45): wraps
      // 1 -> 2 -> 3 -> 1 and never lands on F0 any more, the same wrap cycleFmModeArmed() uses.
      // F0 had no purpose as a cycle-stop — bench testing showed the tap+hold combo landing on it
      // (an unwanted disarm) twice in five minutes before any throttle. The F0-disarm branch this
      // wrap used to reach (display "F0", 0xF2/0, reset to SPIFFS default) is removed; it is
      // unreachable now. To leave Follow-Me off, don't arm it — the combo-after-throttle disarm
      // above and the magnet toggle disarm are unaffected.
      // V2.5-Evo - 2026-10-07 - F4/F5: the wrap is 1 -> 2 -> 3 -> 4 -> 5 -> 1. The stations run round the rider -
      // 1 rear-right, 2 behind, 3 rear-left, 4 FRONT-RIGHT, 5 FRONT-LEFT - and there is deliberately no 6: a station
      // directly ahead would put the buggy on the rider's line, where a failed motor stops it in his path.
      last_fm_mode = (last_fm_mode < 5) ? last_fm_mode + 1 : 1;

      // Large-font mode confirm: LET_F + mode digit (1-5). V2.5-Evo - 2026-10-06 - held 2 s by
      // showFmLabelHeld() (Display.ino) WITHOUT blocking loop(); was a blocking gpsKeepAliveDelay(2000).
      showFmLabelHeld(last_fm_mode);
      queueMetaPacketBurst(0xF2, (uint8_t)(fmEncodeModeByte(last_fm_mode) | kFmFreshDeclBit));   // V2.5-Evo - 2026-10-07 - M-1: a gesture -> bit 7
      fm_last_sync_ms = millis();
      fm_arm_ms       = millis();   // reset arm window — user is actively choosing a mode
    }
    return;
  }

  // V2.5-Evo - 2026-07-20 - Batch T (FM design v1.4): FUNDAMENTAL arm-time reject.
  // Reached only on a FRESH arm (fm_armed was false above). Covers BOTH entry points — the
  // toggle combo AND the magnet gesture both funnel through cycleFmMode() when disarmed. On a
  // fundamental not-ready state the arm DOES NOT TAKE: fire Pattern 7 (long stop buzz) + "St",
  // leave fm_armed false, and return. Marginal/transient conditions are deliberately NOT checked
  // here — they arm as NOT-READY (fmArmedNotReady()) so the magnet still arms through a glitch.
  if (fmFundamentalReject())
  {
    // Pattern 7: one long buzz = the arm was REFUSED. Kept (the rider would otherwise walk away
    // believing FM is armed), and routed through the pending flag like every other stop request.
    vib_stop_pending = true;
    DISP_LOCK(); displayDigits(LET_S, LET_T); updateDisplay(); DISP_UNLOCK();
    gpsKeepAliveDelay(2000);
    return;                          // arm refused — no fm_armed, no 0xF2, no keepalive
  }

  // V2.5-Evo - 2026-04-28 - Change B: On first arm this session, seed last_fm_mode from SPIFFS.
  // usrConf.followme_mode is the user's configured starting mode (range 1-5 since the F4/F5 port; 0 is
  // invalid here). After seeding, fm_session_init_done prevents overriding any mode the user cycled to
  // mid-session. A front starting station still needs the buggy to prove separation before it engages.
  if (!fm_session_init_done)
  {
    if (usrConf.followme_mode >= 1 && usrConf.followme_mode <= 5)
      last_fm_mode = usrConf.followme_mode;
    fm_session_init_done = true;
  }

  // V2.5-Evo - 2026-10-07 - F-7: THE BUG - a fault-stop edge latched while Follow-Me was disarmed stayed latched
  // until the next runFmLoop() pass, so an arm made in between (the toggle combo runs inside runMenu(), before
  // runFmLoop() in loop()) was disarmed again at once with "St". THE FIX: a fresh arm starts with no stale latch.
  // A fault that arrives AFTER this point latches again and is handled normally.
  fm_fault_latched = false;

  // Arm at last used mode (never arms at F0 = disabled; last_fm_mode defaults to 1)
  fm_declare_ms    = millis();   // V2.5-Evo - 2026-10-07 - SOP-041 rule 3 resync: evidence must come after this
  fm_armed         = true;
  fm_arm_ms        = millis();
  fm_throttle_seen = false;
  if (current_vib_pattern == 0) current_vib_pattern = 4;         // Pattern 4: 2 fast buzzes = arm confirm
  fm_last_sync_ms  = millis();     // Change E: start keepalive timer from now (avoids immediate re-sync)

  // V2.5-Evo - 2026-04-29 - Display: show actual mode being armed (F1/F2/F3) in large font
  // instead of the generic "FM" text. Uses large num0[] font via LET_F(15) + mode digit.
  // V2.5-Evo - 2026-10-06 - held 2 s by showFmLabelHeld() (Display.ino) WITHOUT blocking loop();
  // was a blocking gpsKeepAliveDelay(2000). The 0xF2 below now goes out straight away, not 2 s later.
  showFmLabelHeld(last_fm_mode);

  queueMetaPacketBurst(0xF2, (uint8_t)(fmEncodeModeByte(last_fm_mode) | kFmFreshDeclBit));   // V2.5-Evo - 2026-10-07 - M-1: the arm gesture -> bit 7
}

// Called by handleGearToggle(-1) simple LEFT hold 2s when FM is armed (Hall.ino checks isFmArmed()).
// Cycles mode 1→2→3→4→5→1 (F4/F5 since 2026-10-07; Change C: skips mode 0 = disabled); stays armed; resets arm timer.
// V2.5-Evo - 2026-09-19 - IT WRAPS, AND NEVER LANDS ON F0 (owner rule: nothing disarms Follow-Me
// deliberately except the disarm gesture). From 2026-04-29 (F0) until today the third LEFT hold
// while armed reached 0 = disarm, so a rider stepping through the modes on the water could disarm
// by miscounting one hold. Disarm stays on the combo-after-throttle (cycleFmMode()) and the magnet
// toggle; cycleFmMode()'s pre-throttle cycle now uses this same 1->2->3->1 wrap too (owner ruling
// 12:45 the same day — F0 had no purpose as a cycle-stop; see the header note at the top of this
// file). No buzz on a mode cycle (2026-08-16 haptic cut).
void cycleFmModeArmed()
{
  if (!fm_armed) return;
  // Cycle 1→2→3→4→5→1: wrap, never 0.
  // V2.5-Evo - 2026-10-07 - F4/F5: 1-3 becomes 1-5; the two extra steps are the FRONT stations (no 6, no
  // dead-ahead station). A miscounted hold can land on a front station the rider did not mean to pick - a wrong
  // station, never an unsafe one: the buggy must be measurably wide of his line before it goes ahead of him.
  last_fm_mode = (last_fm_mode < 5) ? last_fm_mode + 1 : 1;

  // Large-font mode confirm: LET_F + mode digit (1-5). V2.5-Evo - 2026-10-06 - held 2 s by
  // showFmLabelHeld() (Display.ino) WITHOUT blocking loop(); was a blocking gpsKeepAliveDelay(2000).
  showFmLabelHeld(last_fm_mode);
  queueMetaPacketBurst(0xF2, (uint8_t)(fmEncodeModeByte(last_fm_mode) | kFmFreshDeclBit));   // V2.5-Evo - 2026-10-07 - M-1: a gesture -> bit 7
  fm_last_sync_ms = millis();              // reset keepalive — just synced
  fm_arm_ms       = millis();             // reset arm window — user is actively choosing a mode
}

// ============================================================
// V2.5-Evo - 2026-09-30 - MagStations: the magnet-tap station stepper (mag_mode 4).
//
// WHY THESE FUNCTIONS EXIST SEPARATELY FROM cycleFmModeArmed()
//   cycleFmModeArmed() is the toggle's stepper: it walks 1 -> 2 -> 3 -> 4 -> 5 -> 1 unconditionally and it
//   holds the confirm on screen for 2 s. The magnet tap needs two things that one cannot give:
//   it must honour the rider's mag_fm_set station subset, and it must refuse to do anything at all
//   unless Follow-Me is ACTIVELY FOLLOWING. Rather than bolt both onto the shared path and risk
//   changing the toggle's behaviour, the magnet gets its own entry point. cycleFmModeArmed() and
//   cycleFmMode() are untouched.
//   V2.5-Evo - 2026-10-09 - SUPERSEDED in part (owner ruling: the magnet is a shortcut to the manual gesture). The
//   "only while actively following" gate is gone: the tap now steps in any armed state, like cycleFmModeArmed().
//   What still differs: the mag_fm_set walk, and the tap stays silent during Return-To-Me and while the buggy is
//   coming back on an auto-return. The magnet keeps its own entry point for those reasons.
// ============================================================

// fmIsEngaged - is Follow-Me actively following right now?
//
// V2.5-Evo - 2026-10-09 - NO LONGER the magnet tap's gate (the tap steps in any armed state now, see
// fmStepStationFromMagnet()). It still feeds fmIsReturning(), the FM status dots and the R5 bar. The text
// below is the history of when it was the tap's gate.
// This is THE safety gate for the magnet tap, and it is the SINGLE source of truth for "engaged":
// the magnet tap, the C7 R3/R4 FM status dots and the R5 proximity bar all call this one function
// (V2.5-Evo - 2026-09-30: the R5 bar used to repeat the test inline, which meant the display and the
// safety gate could drift apart at the next edit; it now calls here like everything else).
//
// WHY "ENGAGED" AND NOT "ARMED" IS THE GATE: Follow-Me is armed during the tow, while the rider is
// on the rope and physically attached to the buggy. A buggy that repositions itself then would pull
// a rider who cannot steer away from it. It ENGAGES after the whip, when the rope is slack and the
// rider is riding independently - a station transit there pulls nobody. So armed-but-not-engaged is
// exactly the tow state, and the magnet tap must be dead in it.
//
// ---- V2.5-Evo - 2026-09-30 - CORROBORATION (delta audit: the tow gate rested on ONE bit) ----
// WHAT WAS WRONG. The owner's hard rule - the buggy must never reposition while he is on the rope -
// was carried by a SINGLE un-debounced bit (FM_FLAG_ENGAGED) arriving over a 1-byte-per-packet
// telemetry stream protected only by CRC8. A CRC8 lets roughly 1 in 256 random corruptions through,
// so one bad-but-valid packet could open a window in which a tap moved a station while he was
// attached. Low probability, unacceptable consequence: the fix costs nothing, so it is worth making.
// WHAT IS REQUIRED NOW, all four at once:
//   1. this remote believes FM is armed        (fm_armed, TX-local, not from the radio at all)
//   2. a packet landed inside FM_LINK_HEALTHY_MS (a stale or absent link reads as NOT engaged)
//   3. the RX also says FM_FLAG_ARMED           - a lone ENGAGED bit with no ARMED bit alongside it is
//      not a state the RX ever sends, so it is corruption on its face
//   4. it said so on 2 CONSECUTIVE ARRIVALS of the fm_flags byte (fm_engaged_streak >= 2, counted in
//      Radio.ino where the byte is unpacked - see the streak block there for why it has to be counted
//      per ARRIVAL of index 16 and not per received packet or per loop tick)
// WHY THIS CANNOT BACKFIRE. Every added condition can only make the answer MORE conservative, and the
// conservative answer is "not engaged" = "the magnet tap does nothing". It never opens the gate. The
// only cost is latency: the gate now opens one telemetry rotation later than it used to, and the FM
// dots go solid one rotation later, which is a display nicety against a safety rule.
// DISTANCE CORROBORATION VIA rtm_distance WAS CONSIDERED AND DELIBERATELY NOT ADDED. telemetry
// .rtm_distance decodes 0x00 as "no data" (-1.0f) and saturates at ~164 m, and there is no honest
// whip-distance floor to test against: the rope is 10-25 m but the buggy is legitimately closer than
// that the instant after it engages, so any floor would either be so low it proves nothing or high
// enough to refuse taps the owner asked for. A gate that is sometimes wrong in the permissive
// direction and sometimes wrong in the restrictive one is worse than the two clean bits above.
//
// INPUTS: fm_armed, last_packet, telemetry.fm_flags, fm_engaged_streak.
// OUTPUT: true = following. No side effects, never blocks.
// Safe to call from the loop task and from the bargraph task: every read is single-instruction on this
// RISC-V core, so no read can tear; an inconsistent snapshot resolves to true or false, never garbage,
// and a spurious true would need all four conditions to hold genuinely.
bool fmIsEngaged()
{
  if (!fm_armed) return false;
  if (last_packet == 0 || (millis() - last_packet) >= FM_LINK_HEALTHY_MS) return false;
  uint8_t f = telemetry.fm_flags;
  if ((f & FM_FLAG_ENGAGED) == 0) return false;
  if ((f & FM_FLAG_ARMED)   == 0) return false;   // engaged without armed is not a state the RX sends
  if (fm_engaged_streak < 2)      return false;   // one corroborating arrival of the byte, minimum
  return true;
}

// fmIsReturning - V2.5-Evo - 2026-10-07 - S-7: is the buggy's AUTO-RETURN RETURNING (moving toward the rider) right now?
// The buggy says so with fm_flags bit 6 (auto-return standing) together with bit 1 (engaged, it moves); bit 6 without
// bit 1 is auto-return WAITING (parked), which is drawn exactly like Follow-Me armed. Built on fmIsEngaged(), so it
// carries the same corroboration (bits 0 + 1 on 2 consecutive arrivals, fresh link, this remote armed): the returning
// look is drawn only from buggy-confirmed telemetry (SOP-041 rule 1) and drops as soon as the link goes stale (rule 4).
// An older RX never sets bit 6, so this is always false with it and its return looks like following, as before.
// INPUTS: fmIsEngaged(), telemetry.fm_flags. OUTPUT: true = returning. No side effects; loop task and bargraph task.
bool fmIsReturning()
{
  if (!fmIsEngaged()) return false;
  return (telemetry.fm_flags & FM_FLAG_RETURN_STANDING) != 0;
}

// fmReturnMovingRaw - V2.5-Evo - 2026-10-09 - audit L-2: did the LAST fm_flags byte say the auto-return is moving back?
// Bits 6 (auto-return standing) and 1 (engaged) both set, read raw: no link-freshness test, no 2-arrival streak.
// fmIsReturning() errs toward "not returning" (stale link, short streak); once the magnet tap steps on fm_armed alone,
// that error would let a tap flash "F<n>" over the metres of a moving return. This test errs the other way, toward
// refusing the tap, which is the safe side for a display-only question (the RX return leg never reads the station).
// INPUTS: telemetry.fm_flags. OUTPUT: true = treat as returning. No side effects. Loop task.
bool fmReturnMovingRaw()
{
  const uint8_t want = (uint8_t)(FM_FLAG_RETURN_STANDING | FM_FLAG_ENGAGED);
  return (telemetry.fm_flags & want) == want;
}

// fmNextStationInSet - which station does a tap move to?
//
// INPUTS:  from = the station the buggy is at now (1-5; anything else counts as "outside the set")
//          mask = usrConf.mag_fm_set, bit0 = station 1 ... bit4 = station 5
// OUTPUT:  the station to move to (1-5), or 0 for "there is nowhere to go, do nothing".
// No side effects.
//
// Rules, straight from the design:
//   - from IS in the set     -> the next set station going up, wrapping 5 -> 1
//   - from is NOT in the set -> the LOWEST set station (the rider asked never to sit where he is)
//   - the set holds only from -> returns from, which the caller treats as DO NOTHING. A tap never
//     says "you are already there"; it either moves the buggy or it is silent.
// V2.5-Evo - 2026-10-07 - F4/F5: THE SET IS FIVE STATIONS NOW. The note that used to sit here said stations 4 and 5
// did not exist and that a bit for a station the buggy cannot reach would strand the tap. They exist, so the mask is
// 0x1F and the walk is modulo 5; both new bits address reachable stations. The tap's own gate is unchanged:
// fmIsEngaged() still has to hold, so a tap is dead while the rider is on the rope. mag_fm_set defaults to 7, so the
// magnet keeps stepping the three rear stations until the rider ticks bits 3 and 4.
// V2.5-Evo - 2026-10-09 - the tap's gate is fm_armed now, not fmIsEngaged(): a tap steps on the rope too (see
// fmStepStationFromMagnet() for why that is safe).
static uint8_t fmNextStationInSet(uint8_t from, uint16_t mask)
{
  mask &= 0x1F;                 // stations 1-5
  if (mask == 0) return 0;      // nothing selected - validated against, but fail quietly anyway

  // Current station outside the chosen set -> go to the lowest station that IS in it.
  if (from < 1 || from > 5 || !(mask & (1u << (from - 1))))
  {
    for (uint8_t s = 1; s <= 5; s++)
      if (mask & (1u << (s - 1))) return s;
    return 0;
  }

  // Current station inside the set -> walk forward and stop at the first set station.
  for (uint8_t step = 1; step <= 5; step++)
  {
    uint8_t s = (uint8_t)(((from - 1 + step) % 5) + 1);
    if (mask & (1u << (s - 1))) return s;
  }
  return 0;
}

// ---- V2.5-Evo - 2026-09-30 - THE STATION-FLASH HOLD: A DEDICATED, CLAMPED 1.2 s ----
// WHY THIS IS NOT usrConf.gear_display_time ANY MORE. Two reasons, and both matter.
//   1. THE OWNER DECIDED 1.2 s for this flash. Reusing the gear-flash field was a specification miss,
//      not a design choice, and 1.2 s is what he asked to see on the display for a station change.
//   2. gear_display_time IS A USER FIELD WITH A 65535 ms RANGE CEILING (see its row in
//      ConfigService.ino). A rider who sets it to 5000 for a comfortably readable gear flash would
//      have bought a 5 SECOND loop() stall on every magnet station change. That is not merely slow:
//      a long stall is exactly what the sample-gap guard in runMagGesture() exists to defend against,
//      so an unclamped blocking delay at this call site actively feeds the bug the audit  as H-1.
// BLOCKING CALL - freezes GPS polling, FreeRTOS task scheduling and Serial1 reads for its duration
// (the blocking-call rule). It is bounded at 1.2 s, it is far shorter than the 2 s the toggle's own
// station confirm already blocks for, and the throttle path is untouched by it: the throttle is read
// in measBufCalc at priority 6 and the radio byte is built in sendData at priority 5, while loop() -
// and therefore this delay - runs at priority 1 and cannot starve either of them.
// THE CLAMP IS KEPT EVEN THOUGH THE VALUE IS NOW A CONSTANT. It costs one comparison and it means no
// future edit to kMagStationFlashMs - or any later decision to feed a config field in here again -
// can hand this call site a multi-second block without someone also raising the ceiling on purpose.
// V2.5-Evo - 2026-10-06 - SUPERSEDED: the 1.2 s blocking flash and its clamp are gone. In the field
// (2026-10-05) the label read as under half a second and the owner asked for 2 s (1 s minimum), the
// same as every other "F<n>" confirm. The label is now held by showFmLabelHeld() (Display.ino), which
// does NOT block loop() at all, so there is no stall left to bound and kMagStationFlashMs /
// kMagStationFlashCapMs were removed. The H-1 reasoning above still holds: no config field feeds the
// hold time, it is the fixed kFmLabelHoldMs.

// fmStepStationFromMagnet - act on a magnet TAP (mag_mode 4).
//
// Called from runMagGesture() (Hall.ino) once a magnet tap (600 ms - 3 s since 2026-10-09) has been accepted AND
// fmIsEngaged() has returned true. It re-checks the gate itself so the safety rule lives with the
// action, not only with the caller.
// V2.5-Evo - 2026-10-09 - GATE CHANGED (owner ruling: the magnet is a shortcut to the manual LEFT 2 s hold, so it
// must work the same way). The gate is fm_armed, not fmIsEngaged(): the tap steps while waiting, on the rope, on a
// parked auto-return and while following. It is still refused while the buggy is coming back on an auto-return
// (fmIsReturning() or the raw fmReturnMovingRaw()). WHY THE ROPE IS SAFE: the RX never reads the station before it
// engages - in FM_ARMED it hands steering back and writes cap 255 without calling computeFmTarget() (RX RTMState.ino
// 7792, 7801-7806, 7955-7959). The rope guard is the RX separation latch (RX 7076-7096) and the 9.5 m D_engage floor
// (RX 7028-7031, 7118-7124), neither of which reads the station; stepping on the rope equals having armed at that
// station. The effects below are the same as cycleFmModeArmed(): "F<n>" held 2 s, 0xF2 with the fresh bit, keepalive
// and arm timers reset, and no buzz (the manual step gives none). The only difference is the mag_fm_set walk.
//
// CONFIRMATION - the display only, and only when something actually changed:
//   display : the existing "F<n>" large-font confirm, held 2 s without blocking (V2.5-Evo - 2026-10-06;
//             was a blocking 1.2 s)
// V2.5-Evo - 2026-10-07 - NO BUZZ ANY MORE (owner ruling, minimal-buzz rule; audit R-4 found the count
//   could also go silently missing behind another pattern). The N-tap Pattern 11 that used to say the
//   station number is removed; the F<n> label is the confirm.
// If the tap resolves to the station the buggy is already at, this function returns in silence: no
// flash, no packet. A confirmation for "nothing happened" teaches the rider to expect
// feedback from accidental magnet contact, which is the opposite of what we want.
//
// INPUTS: last_fm_mode, usrConf.mag_fm_set. (usrConf.gear_display_time is NO LONGER read here - see
//   the constants above for why.)
// SIDE EFFECTS: last_fm_mode updated, one 0xF2 burst to the buggy, keepalive + arm timers reset,
//   and a NON-blocking 2 s "F<n>" hold via showFmLabelHeld() (V2.5-Evo -
//   2026-10-06; was a blocking 1.2 s gpsKeepAliveDelay()). Loop task only - never from a FreeRTOS task.
// OUTPUT (V2.5-Evo - 2026-10-06, audit M-1): true only when the station actually moved, false for every
//   silent return. runMagGesture() starts its 1 s tap lockout on true only. Was void.
bool fmStepStationFromMagnet()
{
  if (!fm_armed) return false;                       // V2.5-Evo - 2026-10-09 - any armed state (was fmIsEngaged()); the gate lives with the action too
  if (fmIsReturning()) return false;                 // V2.5-Evo - 2026-10-07 - T-4: never during an auto-return coming back
  if (fmReturnMovingRaw()) return false;             // V2.5-Evo - 2026-10-09 - L-2: nor when the last raw flags say it is coming back

  uint8_t next = fmNextStationInSet(last_fm_mode, usrConf.mag_fm_set);
  if (next == 0 || next == last_fm_mode) return false;   // nowhere to go - stay silent

  last_fm_mode = next;
  Serial.print("FM [TX] magnet tap: station -> F");  // V2.5-Evo - 2026-09-30
  Serial.println(last_fm_mode);

  // Tell the buggy first, so the transit starts while the rider is still reading the confirm.
  queueMetaPacketBurst(0xF2, (uint8_t)(fmEncodeModeByte(last_fm_mode) | kFmFreshDeclBit));   // V2.5-Evo - 2026-10-07 - M-1: a gesture -> bit 7
  fm_last_sync_ms = millis();              // reset keepalive - just synced
  fm_arm_ms       = millis();              // reset arm window - the rider is actively choosing

  // V2.5-Evo - 2026-10-07 - the Pattern 11 station-count buzz that was queued here is removed (owner
  // ruling: no buzz on a station change). The display label below is the only confirm.

  // The same "F<n>" confirm the toggle path draws. V2.5-Evo - 2026-10-06 - held 2 s (was a blocking
  // 1.2 s) by showFmLabelHeld() in Display.ino, which does not block loop() - see the note above.
  showFmLabelHeld(last_fm_mode);
  return true;
}

// V2.5-Evo - 2026-10-07 - R-4: fmToggleRtmEnabledFromMagnet() and fmToggleAutoReturnFromMagnet() were
// REMOVED from here. Neither had a caller once the 2.5 s magnet hold started always meaning the manual
// Return-To-Me (2026-10-06). Auto-return (A1/A0) is flipped only by ceremonyCancelForReturnGesture() and
// cleared by returnGesture() state 3; rtm_enabled_session now has no writer and stays 0xFF (see its block).

// ============================================================
// V2.5-Evo - 2026-09-19 - returnGesture - RIGHT tap + LEFT hold, the three-state return gesture.
//
// Called by handleGearToggle() (Hall.ino) when the combo hold completes, in place of the direct
// setRtmArmed() call it used to make. The STATE IS READ AT THE INSTANT THE HOLD COMPLETES, never
// remembered from a previous press:
//   (1) no override standing and RTM not armed -> ARM RTM exactly as before: setRtmArmed(), the
//       squeeze ceremony, the gates, rtm_arm_window_s, Pattern 4 - nothing about RTM changes.
//   (2) RTM armed and inside the arm window -> the ceremony is BLOCKING loop(), so this function
//       cannot run then; the ceremony's own wait loops poll for the gesture and land this state
//       from inside (returnGestureCeremonyPoll() / ceremonyCancelForReturnGesture() above):
//       cancel the arm (0xF1/0, TX -> IDLE) and set the override to the OPPOSITE of the value the
//       buggy echoes in fm_flags bit 7. Pattern 9, "Ar" / "AO".
//   (3) an override standing (either value) -> BACK TO DEFAULT: last_fm_return_mode = 0xFF, so the
//       next 0xF2 carries bits 5-6 = 00 and the buggy uses its stored fm_return_mode. Pattern 10,
//       nothing on the display. The next gesture after this one arms RTM again (state 1).
// WHY "OPPOSITE OF THE ECHO": with the buggy's stored default ON, "set the override ON" could never
// turn auto-return OFF for a session; the opposite of the effective value is useful under either
// default. A rider who wants ON permanently sets it on the buggy, not with the gesture.
//
// INVARIANT - A MISCOUNT CAN NEVER PRODUCE MOTION. Arming RTM only declares intent: the squeeze
// ceremony, RTM's own gates and a held trigger are all still required before anything moves, and
// the wrong-state outcome of a miscount is one of "RTM armed and waiting for a squeeze", "RTM arm
// cancelled" or "override changed" - none of which is a motor command. The override only changes
// what a Follow-Me HOLD graduates to on the buggy, and that graduation moves nothing without the
// trigger either. The gesture is off-throttle only (the calcFilter() gate hands the toggle to
// steering the moment the trigger rises, and handleGearToggle() aborts a hold on thr_scaled > 3).
//
// Inputs: last_fm_return_mode, rtm_tx_state (via setRtmArmed), usrConf.rtm_enabled / gps_en.
// Side effects: state 1 - everything setRtmArmed() does (blocking); state 3 - the override
//   cleared, Pattern 10, the keepalive requested (if FM is armed), one Serial line.
// ============================================================
void returnGesture()
{
  // V2.5-Evo - 2026-10-07 - THE GESTURE IS IGNORED WHILE RETURN-TO-ME IS ACTIVE ON THIS REMOTE (audit: "toggle route
  // during RTM ACTIVE"; same rule as the magnet hold, R-2). THE BUG: this path called setRtmArmed() without looking
  // at rtm_tx_state, so a RIGHT tap + LEFT hold during an active return re-entered the arm ceremony: the buggy was
  // told 0xF1/0, throttle went to 0 and the return restarted with no "St" and no cooldown. Now: nothing happens,
  // one serial line, the return continues (Gate 3 still ends it on a release).
  if (rtm_tx_active || rtmIsArming())
  {
    Serial.println("RTM [TX] return gesture ignored: Return-To-Me is already active on this remote, the return continues");
    return;
  }
  // V2.5-Evo - 2026-10-07 - SOP-040 GESTURE RULE: the gesture acts only with the trigger fully released, in every
  // one of its states (arm, refusal, override clear). handleGearToggle() already gates its action on a released
  // trigger; this is the same rule held at the action itself. Ignored silently (serial line only).
  if (!triggerReleased())
  {
    Serial.printf("RTM [TX] return gesture ignored: trigger held (thr %u) - it needs the trigger fully released\n",
                  (unsigned)thr_scaled);
    return;
  }
  if (last_fm_return_mode != 0xFF)
  {
    // State 3: back to default.
    last_fm_return_mode = 0xFF;
    Serial.println("RETURN [TX] gesture: auto-return override cleared -> the buggy uses its stored default (next 0xF2 carries no override)");
    if (current_vib_pattern == 0) current_vib_pattern = 10;   // Pattern 10: two medium pulses = override CLEARED
    fmRequestKeepaliveNow();
    return;
  }
  // State 1: arm RTM as before. (State 2 is landed from inside the ceremony this starts.)
  // V2.5-Evo - 2026-09-30 - the EFFECTIVE enable (see rtmEnabledEffective()), so the toggle combo and the
  // magnet gesture agree about whether Return-To-Me is available this session. setRtmArmed() re-checks it.
  if (rtmEnabledEffective() && usrConf.gps_en)
  {
    setRtmArmed();
  }
  else
  {
    // V2.5-Evo - 2026-10-07 - SOP-040: manual Return-To-Me cannot start (disabled or GPS off). This refusal was
    // completely silent on the toggle route, so the rider could not tell the gesture failed. "St" with the
    // normal stop buzz is the one "not working" signal - the same refusal the magnet hold now gives.
    // BLOCKING 2 s like every other "St"; nothing was armed, throttle is untouched.
    Serial.printf("RTM [TX] arm refused: Return-To-Me cannot start (rtm enabled %d, gps_en %d)\n",
                  rtmEnabledEffective() ? 1 : 0, (int)usrConf.gps_en);
    vib_stop_pending = true;
    DISP_LOCK(); displayDigits(LET_S, LET_T); updateDisplay(); DISP_UNLOCK();
    gpsKeepAliveDelay(2000);
  }
}

// Called from loop() every ~110ms.
// Handles the RX fault-stop edge, the arm-window auto-disarm and the 30 s 0xF2 keepalive.
// (The TX Gate 1 throttle-release disarm was removed 2026-09-17 — see the FM section header.)
void runFmLoop()
{
  unsigned long now = millis();

  // V2.5-Evo - 2026-07-20 - Batch T (FM design v1.4): DISARM OWNERSHIP — the display can't lie.
  // The RX owns engagement; on an RX fault it stops FM and raises fm_flags bit3 (fault-stop),
  // held sticky ~6s so this ~110ms loop is guaranteed to catch the rising edge across the
  // ~2.4s telemetry rotation. On that rising edge, while WE still believe we are armed, the TX
  // must clear its own arm and STOP re-declaring 0xF2/mode — otherwise the 30s keepalive below
  // would re-arm the RX within 30s of a fault. fmDisarm() does exactly that: fm_armed=false
  // (so this function early-returns next tick and the keepalive never fires), queues 0xF2/0
  // (belt-and-suspenders — RX already idle), and shows the stop as "St" + Pattern 7. bit3 is
  // already surprise-gated on the RX, so it is only set when the alarm is warranted — no TX
  // re-gating needed. fm_flags_prev is updated every tick (armed or not) so a re-arm starts clean.
  // V2.5-Evo - 2026-10-07 - R-6: THE EDGE NOW COMES FROM A LATCH. Comparing telemetry.fm_flags tick to tick
  // (the fm_flags_prev code that was here) missed a fault whose ~6 s sticky bit rose and fell while loop()
  // was blocked in the RTM arm ceremony (up to the arm window + ~4 s). Radio.ino now latches the rising
  // edge on the byte's arrival (fm_fault_latched); it is handled here however late: the remote disarms
  // Follow-Me, shows "St" and fires the stop buzz through fmDisarm(false) -> vib_stop_pending, exactly as
  // an on-time edge always did. Latched while NOT armed -> simply cleared, as an edge was ignored before.
  // ONE GUARD: while a burst is still going out the latch is kept and retried next tick (~110 ms; a burst
  // drains in ~300 ms), the same rule the keepalive below follows. (V2.5-Evo - 2026-10-07 - P-12: this was
  // written for the old SINGLE-SLOT queue, where fmDisarm()'s 0xF2/0 would have overwritten a just-queued
  // 0xF1/1 and the buggy would never have learned RTM was on. The queue is 2-deep now and an 0xF2 can never
  // evict an 0xF1, so the wait is belt only. A fault handled while a return is ACTIVE no longer reaches this
  // branch at all - it takes the F-1 deferred path above it.)
  if (fm_fault_latched && !fm_armed)
  {
    fm_fault_latched = false;   // nothing armed on this side to disarm
  }
  else if (fm_fault_latched && rtm_tx_active)
  {
    // V2.5-Evo - 2026-10-07 - F-1: a return is running - do the SILENT half now and defer the "St" + buzz to the
    // return's end (see fm_fault_deferred). No blocking hold, so RTM's gates and ramp keep running.
    fm_fault_latched = false;
    Serial.println("FM [TX] the buggy reported a Follow-Me fault stop during Return-To-Me -> Follow-Me off now (0xF2/0, silent); St + stop buzz when the return ends");
    fmSilentDisarm();
    fm_fault_deferred = true;
    return;
  }
  else if (fm_fault_latched && rtm_meta_count.load(std::memory_order_acquire) == 0)
  {
    fm_fault_latched = false;
    // FAULT — and since the 2026-08-17 revision this is the ONLY FM path that buzzes. The RX
    // faulted and stopped following by itself: the rider asked for nothing, no timer explains it,
    // and he has no other way to learn the buggy is no longer steering for him. → commanded =
    // false → Pattern 7.
    Serial.println("FM [TX] disarm: the buggy reported a Follow-Me fault stop (fm_flags bit 3) -> 0xF2/0");   // V2.5-Evo - 2026-09-19
    fmDisarm(false);   // clears fm_armed + keepalive, sends 0xF2/0, "St" + Pattern 7 — TX & RX can't disagree
    return;
  }

  if (!fm_armed) return;   // V2.5-Evo - 2026-10-07 - the warning-scheduler reset that sat here went with Pattern 8

  // ============================================================
  // V2.5-Evo - 2026-10-07 - SOP-041 RULE 3: THE SCREEN FOLLOWS THE BUGGY - RESYNC A FOLLOW-ME THE BUGGY NO LONGER HAS
  // THE GAP: if the buggy dropped Follow-Me without a fault edge reaching this remote (its 95 s declaration expiry, an
  // RX reboot, an RX with Follow-Me unavailable, an edge lost to the link), the remote kept drawing Follow-Me armed and
  // kept re-declaring it every 30 s - a "wannabe" state the owner ruled out. THE FIX: when the buggy, on a fresh link,
  // reports its ARMED bit (fm_flags bit 0) clear on at least 2 consecutive arrivals of the byte spanning at least
  // kFmResyncMs, and that run began more than kFmResyncDeclareMarginMs after this remote's arm declaration, the remote
  // disarms Follow-Me itself and says so the one way the rider knows: "St" + the stop buzz (fmDisarm(false) - it also
  // sends 0xF2/0 so the two sides agree). During an active manual return the silent half runs now and the "St" + buzz
  // is deferred to the return's end, exactly like a Follow-Me fault (F-1). Signed elapsed time: the stamps are written
  // by the telemetry task and can be newer than `now`.
  // ============================================================
  {
    const unsigned long since      = fm_flags_unarmed_since_ms;
    const bool          link_fresh = (last_packet != 0) && ((long)(now - last_packet) < (long)FM_LINK_HEALTHY_MS);
    if (link_fresh && fm_flags_unarmed_streak >= 2 && since != 0 &&
        (long)(since - fm_declare_ms) > (long)kFmResyncDeclareMarginMs &&
        (long)(now - since) >= (long)kFmResyncMs)
    {
      if (rtm_tx_active)
      {
        Serial.println("FM [TX] the buggy reports Follow-Me NOT armed (fm_flags bit 0 clear 3 s) during Return-To-Me -> Follow-Me off now (0xF2/0); St + stop buzz when the return ends");
        fmSilentDisarm();
        fm_fault_deferred = true;
      }
      else
      {
        Serial.println("FM [TX] the buggy reports Follow-Me NOT armed (fm_flags bit 0 clear 3 s) -> disarm here too: St + stop buzz, 0xF2/0");
        fmDisarm(false);
      }
      return;
    }
  }

  // V2.5-Evo - 2026-10-07 - the FM warning-distance haptic block (Pattern 8 every 2 s beyond
  // fm_warn_distance_m) that ran here is REMOVED by owner ruling - see the note above fmFundamentalReject().

  // Arm-timeout auto-disarm: if the rider never applied throttle since arming, disarm after
  // fm_arm_timeout_s. V2.5-Evo - 2026-09-17: gated on fm_arm_timeout_s > 0 — 0 means NEVER, and is
  // the default. An armed remote with no throttle now stays armed until the rider disarms it, the
  // RX faults, or power is cut (always-armed philosophy, owner decision 2026-09-15).
  if (!fm_throttle_seen && usrConf.fm_arm_timeout_s > 0)
  {
    if (now - fm_arm_ms > (unsigned long)usrConf.fm_arm_timeout_s * 1000UL)
    {
      // V2.5-Evo - 2026-07-20 - Batch T (A2 D1): the silent disarm is no longer fully silent.
      // The scanner going dark is the primary signal; add ONE short blip (Pattern 5, a single
      // 150ms pulse) as a NUDGE so a rider relying on the long arm window isn't left believing
      // he is still armed. Distinct by feel from the two-tap arm (Pattern 4) and the long stop
      // buzz (Pattern 7). Placed at the call site (not inside fmSilentDisarm) so it fires ONLY on
      // arm-window expiry, never on the RTM-preemption path that also called fmSilentDisarm()
      // (that path is gone since 2026-09-18 - RTM no longer disarms FM - so this is now its only caller).
      Serial.printf("FM [TX] disarm: armed %u s with no throttle (fm_arm_timeout_s) -> 0xF2/0\n", (unsigned)usrConf.fm_arm_timeout_s);   // V2.5-Evo - 2026-09-19
      fmSilentDisarm();   // arm window expired before first throttle — no blocking confirm
      if (current_vib_pattern == 0) current_vib_pattern = 5;   // nudge: one short blip
      return;
    }
  }

  // Track throttle engagement. fm_throttle_seen is what turns the arm combo into a disarm
  // toggle (cycleFmMode) once the rider has ridden, and what ends the arm-window check above.
  // fm_arm_ms is still refreshed here, but since the Gate 1 removal (2026-09-17) nothing reads
  // it once fm_throttle_seen is set — kept as-is rather than widening this change.
  if (thr_scaled > 10)
  {
    fm_throttle_seen = true;
    fm_arm_ms = now;
  }

  // V2.5-Evo - 2026-09-17 - Gate1-REMOVED. The block that lived here disarmed FM after
  // kFmGate1ReleaseMs (30 s) of thr_scaled < 5 once fm_throttle_seen was set. Deleted on the
  // owner's decision (audit A2-TX): the TX keeps its arm across any length of release. The RX
  // side is unchanged and still owns the motion path — its 10 s latch clear and 95 s mode-age
  // expiry are the backstops.

  // V2.5-Evo - 2026-04-28 - Change E: Send 0xF2 keepalive every 30s while FM is armed.
  // Ensures RX stays in the correct FM mode after any transient packet loss.
  //
  // V2.5-Evo - 2026-09-18 - NEVER CLOBBER AN IN-FLIGHT BURST (code-review finding, water-test
  // blocker). queueMetaPacketBurst() is a SINGLE SLOT (Radio.ino): a new call overwrites
  // rtm_meta_type/value and restarts the 3-packet count, so whatever burst was still being sent is
  // simply lost on the air. WHY THAT BIT HERE: the toggle RTM arm blocks loop() inside
  // runDoubleSqueezeArm() for 8 s or more, during which this function does not run and
  // fm_last_sync_ms goes stale; the ceremony ends by queueing 0xF1/1 (RTM ACTIVE), loop() resumes,
  // and on its very first tick this keepalive - now 30 s overdue - queued 0xF2/mode on top of it
  // before sendData() (100 ms cycle) had sent a single 0xF1/1 packet. The TX then showed RTM
  // ACTIVE with rtm_thr_cap_tx ramping while the RX never learned RTM was on: no gate 9, no
  // approach cap, no Phase C, no RTM steering - the trigger drove the buggy straight ahead.
  // Measured ~30-50 % of toggle arms once Follow-Me started staying armed through a return.
  // THE FIX: queue the keepalive only when the slot is EMPTY; otherwise retry next tick and do NOT
  // advance fm_last_sync_ms, so the retry keeps coming every ~110 ms until the slot frees (a burst
  // drains in ~300 ms). The acquire load pairs with the release store in queueMetaPacketBurst().
  // Belt: runDoubleSqueezeArm() also refreshes fm_last_sync_ms at the end of a successful
  // ceremony, so the keepalive is not overdue on resume in the first place.
  // (A 2-deep queue and a TX check of fm_status bit 1 are the longer-term answer; not built here.)
  // V2.5-Evo - 2026-10-07 - H-1 / L-1: both now exist (queueMetaPacketBurst() in Radio.ino, rtmRxStateWatch()
  // above). This empty-queue wait is kept anyway: it costs at most ~600 ms and never loses a state burst.
  if (fm_last_sync_ms > 0 && now - fm_last_sync_ms >= 30000UL)
  {
    if (rtm_meta_count.load(std::memory_order_acquire) == 0)
    {
      queueMetaPacketBurst(0xF2, fmEncodeModeByte(last_fm_mode));   // V2.5-Evo - 2026-09-19 - carries the override bits
      fm_last_sync_ms = now;
    }
    // else: a burst (0xF1 RTM state, 0xF4 aux, or an earlier 0xF2) is still going out — retry next tick.
  }
}
