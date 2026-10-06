// V2.5-Evo - 2026-10-06 - COMMENT ONLY (audit L-19): the fm_front_ahead_extra_m comment gives the derived front angle as 35-45 deg (it still said 35-80). No code, no confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-10-06 - AUDIT M-12: kFmFrontAngleMaxDeg 80 -> 45, so a front station is never less far ahead than it is to the side (13 m ahead at the 13 m side floor). Comments: audit L-13 (stale angle + radius wording). Constant only - no confStruct change, sizeof STAYS 200, SW_VERSION STAYS 36.
// V2.5-Evo - 2026-10-06 - FRONT STATIONS BY OFFSET (owner rule): the u16 fm_front_angle_deg is RENAMED IN PLACE to fm_front_ahead_extra_m - same offset, same type, sizeof STAYS 200, SW_VERSION STAYS 36, NO CONFIG WIPE. F4/F5 now sit the lateral floor (13 m) exactly to the side and d_follow + this many metres ahead; the angle and radius are derived (FollowMeStation.h fmFrontStationGeom). 0 = default 7 m, legal 4-10 m. kFmFrontAngleDefaultDeg is replaced by kFmFrontAheadExtraDefaultM / MinM / MaxM; kFmFrontAngleMinDeg (35) and MaxDeg (80) now bound the DERIVED angle.
// V2.5-Evo - 2026-10-04 - COMMENTS ONLY, M-4: the fm_align_cap field comment now records that the VESC applies its OWN 3 % input deadband, which eats the first ~7.66 command counts - so below ~8 counts the motor does not turn at all, and every low-cap figure in these comments (the "13 is about 5 %" and the "at cap 13: motors 0 / 26" next to it included) overstates the thrust actually delivered. Measured against the owner's PPM map (span 852 us, l_current_max 95 A): 3 counts = inside the deadband = 0 A, 23 counts = 5.9 A. No declaration, default, struct, constant or range in this file is changed: sizeof(confStruct) stays 200, SW_VERSION stays 36, LOG_FILE_FORMAT_VER stays 2, record sizes stay 62 / 90 / 112.
// V2.5-Evo - 2026-10-03 - I2C ENABLE-SWAP STARVATION, STEP 2 of 4 - THE INSTRUMENT (see PWM.ino, System.ino, Logger.ino, Tools/logreader/bremote_log.py). The enable swap that time-multiplexes the single PPM output between the two motors has never had an observer on it, which is why a failure that starves one channel entirely could not be confirmed or ruled out from the field. THIS FILE ADDS, all of it diagnostic: (1) three saturating volatile uint16_t counters beside g_motor_gate_open - g_swap_fail_ch0 / g_swap_fail_ch1 (per-channel session totals of swaps that lost the mutex) and g_swap_fail_run (the CONSECUTIVE run length right now). volatile, not std::atomic, because this is the g_diag_mux_errors shape exactly: single writer in generatePWM(), single core, read by ?diag and the logger, zeroed by ?diagz - full reasoning at the declaration. (2) kSwapStarveTicks (25) and kSwapRecoverTicks (5) beside the kPivot* block, with their derivations, so STEP 3 needs no confStruct field; kSwapStarveTicks is explicitly COUPLED to the Wire.setTimeOut(3) that landed in STEP 1 and is not justified without it. (3) FM_LOG_GATE_SWAP_FAIL_CH0/CH1_SHIFT+MASK for bits 23-30 of the existing fm_gate_flags word, which were free. NOTHING READS g_swap_fail_run BACK AT THIS STEP - it becomes a control input only in STEP 3, as a separate commit. ZERO log bytes, NO LOG_FILE_FORMAT_VER bump (stays 2), record sizes stay 62 / 90 / 112, every existing log file still parses, and no confStruct change: sizeof stays 200, SW_VERSION stays 36. Cost: 6 bytes of RAM.
// V2.5-Evo - 2026-10-03 - RX WEB CONSOLE: #include "../Common/SerialTee.h" added immediately after <Arduino.h> and before every one of our own headers, which is where it has to be - it redefines what the word `Serial` means, so anything included ahead of it is silently left out of the capture. Include only; no declaration, default, struct or constant in this file is changed. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-10-02 - P2 (FRONT STATIONS F4/F5), part 1 of 3 (see ConfigService.ino and WebUiEmbedded.h): the config layer only - no controller change in this file's commit. Adds (1) the confStruct field fm_front_angle_deg (u16, 0 = use the 45 deg default, else 35-80 deg off dead ahead for the two front stations) IN THE 2 BYTES OF TAIL PADDING the SW36 append left behind, so sizeof(confStruct) STAYS 200, the static_assert is untouched, SW_VERSION STAYS 36 and THE OWNER'S STORED CONFIG IS NOT WIPED BY THIS FLASH - the same trick mag_fm_set used on the TX and bt_enabled before it. The P2 plan had allocated this field to the banked reserved slot rsvd_f32_1 as a float; that slot no longer exists (it became auto_ramp_s on 2026-09-19, and rsvd_u16_1 became steer_during_auto the same day), so the field is a u16 in WHOLE DEGREES instead - 1 degree is 0.32 m at the 18.4 m front radius, far inside the 5 m relative GPS error the design already assumes. (2) The four shared constants kFmFrontAngleDefaultDeg (45), kFmFrontAngleMinDeg (35, the owner's override of the review's 40), kFmFrontAngleMaxDeg (80) and kFmFrontLateralMinM (13 m = carve 8 + relative GPS 5), defined here because the ConfigService validator and the RTMState read site both need them and ConfigService.ino is concatenated first. (3) The include of ../Common/FollowMeStation.h, the pure station arithmetic. defaultConf.fm_front_angle_deg is 0, which is what every fielded board already reads out of those padding bytes, so the flash changes no behaviour until the rider sets the field. Dead ahead stays unreachable at every value: |station| is clamped at 180 - the effective angle.
// V2.5-Evo - 2026-10-01 - M-2 FIX, part 1 of 4 (see PWM.ino, Logger.ino, System.ino): THE MOTOR GATE BECOMES VISIBLE. There are now two link timestamps - last_packet ("is the remote alive?") and last_control_packet ("do I have a fresh throttle command?") - and the motor gate uses the second while EVERY instrument still read the first, so the motor could be gated off with the BIND LED solid, ?printrssi showing good signal and the log's link flag reading healthy, and nothing anywhere saying why the motors stopped. It cost a half-day on 2026-09-29: ?printpwm showed a live throttle value while the gate was shut, and remote_error stayed 0 through two confirmed outages of 5.4 s and 8.75 s. FOUR instruments, none of which could see the gate. THIS FILE ADDS: (1) the diagnostic observer g_motor_gate_open - calcPWM() publishes the verdict of the gate-mirror expression it already computes, so ?diag, ?printpwm and the logger read ONE published value instead of each growing a third copy of the gate test; (2) two columns at the tail of VescLogData, the BASE record: ctrl_pkt_age_ms (u16, capped 0xFFFE) and motor_gate_open (u8). They go in the base deliberately - the tiers are cumulative and levels 0-3 all record as level 3, so a field in the base appears at EVERY log level, 0 through 5, and an incident is never re-runnable at a higher level. sizeof(VescLogData) 59 -> 62, VescLogDataL4 87 -> 90, VescLogDataL5 109 -> 112, asserts updated. OLD LOGS DO NOT PARSE AFTER THIS FLASH and the per-file header does NOT rescue them: record_size tells a reader how far to step, but it cannot express that every block above byte 59 has moved 3 bytes, so a pre-flash level-4/5 file would have been accepted and silently mis-decoded - exactly the "convincing garbage" this format exists to prevent. LOG_FILE_FORMAT_VER is therefore bumped 1 -> 2, which both readers already test, so every pre-flash file is now refused in plain English instead. DOWNLOAD ANY LOGS WORTH KEEPING BEFORE FLASHING. This is the same accepted cost as the 51->52, 53->59 record growths. No confStruct change, sizeof(confStruct) stays 200, SW_VERSION stays 36, the owner's stored config is NOT wiped.
// V2.5-Evo - 2026-09-28 - R-3 FIX, part 1 of 3 (see Radio.ino and PWM.ino): adds the plain global last_control_packet - a SECOND link timestamp, stamped only by the normal control-packet branch and read only by the motor gate in PWM.ino. last_packet means "the remote is alive" and is refreshed by the 0xF1 / 0xF2 / 0xF4 meta-packets too, none of which carries a throttle byte; the motor gate was testing it and so could reopen on a meta-packet with a stale thr_received and the trigger released. All four of last_packet's other readers (Logger.ino's link flag, RTMState.ino's RTM failsafe stop and FM_STOP_LINK, System.ino's connection status) are deliberately left exactly as they are - liveness is the right question for them. It is a plain global, NOT a confStruct field: no struct change, sizeof stays 200, SW_VERSION stays 36, and this flash does NOT wipe the owner's stored config.
// V2.5-Evo - 2026-09-25 - MANUAL PIVOT ASSIST, FLOOR CORRECTION (review findings P-7 / P-6). kPivotFloorQ8 128 -> 64. WHY: at full lock the mixer's gain into the outer motor is exactly 2x, so a throttle-domain floor f lands the outer motor - and therefore ALL the thrust, since the inner motor is 0 - at min(2fT, 255). f = 1/2 is the EXACT RECIPROCAL of that gain, so it cancels it and cuts NOTHING at full trigger (outer 254 of 255): the single worst value the constant can take, worst precisely in the owner's trigger-pinned use case. 64 (f = 1/4) puts the outer motor at 126/255 at full trigger, a 51 % cut of total thrust. The min(2fT, 255) derivation, the floor table and the "do not go below 64 without water testing" bound are now written into the constants block; the old comment claimed the full-lock benefit was "reduced thrust", which at f = 1/2 and full trigger did not exist (254 vs 255) and would have hidden this from the next reader (P-7). Also corrects the "byte-for-byte no-op" wording: the depth BLEEDS OUT over 8 ticks, so RTM/FM taking over, crossing 5 km/h and straightening are inert WITHIN 80 ms, subtract-only throughout, not on the same tick (P-6). ONE code change - the constant; everything else is comment. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-25 - MANUAL PIVOT ASSIST (owner request: "a fast pivot will aid enormously in getting it right right away"). Adds, with NO confStruct field: (1) the nine compile-time kPivot* tuning constants below the heading-trust block - the assist is deliberately recompile-tuned, in the kFm* style, so this flash does NOT reset the owner's SPIFFS config (sizeof stays 200, SW_VERSION stays 36); (2) the diagnostic observer g_pivot_assist_q4 beside g_motor0_cmd / g_motor1_cmd - the assist depth quantised to 4 bits, written by calcPWM() at 100 Hz and read by fillLevel4Diag(); (3) FM_LOG_GATE_PIVOT_ASSIST_SHIFT / _MASK - bits 19-22 of the EXISTING fm_gate_flags u32, so the log gains no column and no record size changes. The control code itself is in PWM.ino: a subtract-only multiply applied to the ramped throttle in the steering_type 1 branch only, gated OFF whenever rtm_rx_active || fm_rx_active. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-24 - COMMENT-ONLY RESYNC after the throttle-ramp move in PWM.ino: motor_ramp_s ramps the THROTTLE ONLY and steering is never rate-limited, so the confStruct field comment, the defaultConf line and the g_motor0_cmd / g_motor1_cmd declaration (which claimed the observers were independent of the ramp - they now carry it) are corrected. No struct, no default value, no code and no field changed: sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - DEEP LOG level 5 ("everything", owner request for the test sessions): VescLogDataL5 = the complete 87 B level-4 record + a 22 B level-5 block = 109 B (static_assert 109): the rider's position as the RX holds it (rx_tx_gps_lat/lng as float, the distinct-fix counter, the fix age), the classic RTM phase code and rtm_approach_cap, the align cap / align influence / mixer influence in force this tick, the auto-return override state, telemetry.fm_flags as sent, the 0xF2 keepalive age, and two reserved bytes for the steer-takeover branch (l5_rsvd_takeover_active / _end) so its integration does not bump the size again. Every field is a COPY of published state, taken in fmPublishLogSnapshot() (loop task, one writer) and carried in FmLogSnapshot; nothing in the control path reads any of it. log_level 5 selects it (logResolveLevel / logRecordSizeForLevel), createNewLogFile() stamps record_size 109, logCsvHeaderFor() gains the L5 tier (65 columns) and logFormatCsvRow() prints every level-5 field with units. Capacity on the 1757 KB the filesystem reports: about 1 h 30 min at 3 Hz, about 55 min at 5 Hz (87 B level 4: about 1 h 55 min / 1 h 10 min). No confStruct change - the log_level field is the same u16, its validator max is raised 4 -> 5 - sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - DEEP LOG (B)+(C): VescLogDataL4 grows 83 -> 87 B (static_assert 87). (B) fm_rider_raw_dx10 u16 = the rider's RAW displacement speed x 10 km/h (RTMState.ino fm_rider_raw_kmh, the number the FM_RETURN candidate is judged on; the filtered EMA track was already logged as fm_rider_speed_dx10), sentinel 0xFFFF = unknown (< 0). (C) motor0_cmd / motor1_cmd u8 = the two post-mixer, pre-map motor commands out of calcPWM()'s steering_type 1 branch (g_motor0_cmd / g_motor1_cmd, two diagnostic observers written every 100 Hz tick, the g_effective_steer pattern; 0 for the efoil / servo branches). TIER FIX for the readers: logCsvHeaderFor() used to return the 65 B DIAG header for anything smaller than sizeof(VescLogDataL4), so after this bump every 83 B file written since 2026-09-17 would have printed with the wrong header; it now picks by the offsets of the blocks actually present (65 B DIAG / 83 B L4_83 / 85 B +raw / 87 B full) and logFormatCsvRow() guards each new field on record_size >= its offset + size. The 2026-09-17 'LAYOUT IS FINAL' note is history. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - DEEP LOG (A): the one unused byte of the level-4 Follow-Me block, fm_pad, becomes fm_return_reason - a straight copy of the controller's FmReturnReason latch (RTMState.ino fm_return_last_reason: 0 = no event since boot; 4 = RETURN entered, 6/7/8/11 = arrived / cancelled / timed out / steered, 1-3 = a candidate dropped), STICKY - it changes only on an event, so read it on the row where fm_state changes. Two new bits in the existing fm_gate_flags u32: bit 17 aligning (an autonomous controller is turning to face its target this tick) and bit 18 boost (the pivot-boost mixer influence is published this tick); bit 16 is left free because the takeover side branch already defines it. CSV gains fm_return_reason, fm_aligning, fm_boost (45 -> 48 columns for an 83 B file; the two bits are read out of fm_gate_flags). Record size unchanged at 83, static_assert stays 83, old 83 B files print 0 in all three new columns (the pad was always 0). No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - SEPARATE RAMPS, part 3: comment only - the deep-log bit 7 note names kFmJudgeGraceMs (6500, pinned in RTMState.ino when the engage ramp went 3500 -> 1500). No code change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - SEPARATE RAMPS, part 2: includes ../Common/OutputRamp.h (the pure motor rise-limit step calcPWM() now calls, arithmetic unchanged, host-tested). Include only: no confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - SEPARATE RAMPS (owner decision 14:00): the banked RESERVED slot rsvd_f32_1 is RENAMED IN PLACE to auto_ramp_s (float, 0 = inherit motor_ramp_s = today's behaviour byte-identical, else 0.2-4.0 s) - the motor rise-limit while an AUTOMATIC mode (Follow-Me following, FM_RETURN motion, classic RTM) caps the throttle; the slow motor_ramp_s stays the manual-tow ramp (the rider's shoulder on a loaded rope) and every hand-back to the rider. kAutoRampMinS / kAutoRampMaxS bounds, clamped on load. The P2 rule (buggy ahead of the rider -> manual ramp) is recorded at the field. No confStruct size change: sizeof stays 200, SW_VERSION stays 36, config is NOT reset by this flash.
// V2.5-Evo - 2026-09-19 - fix round 2: comments only - the takeover release band is 30 counts (was 20) and the maximum takeover is 10 s (was 20 s); the constants live in RTMState.ino. No code change here, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - STICK DURING AUTO-STEER (cancel or take over): the banked RESERVED slot rsvd_u16_1 is RENAMED IN PLACE to steer_during_auto (u16, 0-1, default 0) - same offset, same type; 0 = the stick CANCELS the automatic steering exactly as before (every fielded board holds 0 here), 1 = the stick TAKES OVER the steering byte while deflected and hands it back on centring, for Follow-Me following, auto-return (FM_RETURN) and classic return-to-me alike. Adds the steer_takeover_active atomic (the ONE takeover flag; single write site publishSteerTakeover() in RTMState.ino, read by calcPWM(), false on every tick with no auto-steer owner), includes ../Common/SteerArbitration.h (the pure, host-tested engage/release/timeout arbitration), adds FM_LOG_GATE_STEER_TAKEOVER (bit 16) to the deep-log gate word (existing u32, record size unchanged) and documents telemetry.fm_flags bits 4 (setting echo) and 5 (takeover standing) for the remote. No confStruct size change: sizeof stays 200, SW_VERSION stays 36, config is NOT reset by this flash.
// V2.5-Evo - 2026-09-19 - FM_RETURN + pivot boost: adds the align_mixer_influence_override atomic (0 = none; the ONE mixer influence override, written by publishAlignMixerInfluence() in RTMState.ino for FM align, FM_RETURN align and classic RTM Phase 1 align, read by calcPWM()), includes ../Common/FollowMeReturnProof.h (the pure, host-tested FM_RETURN entry proof), and adds FM_LOG_GATE_RETURN_WINDOW (bit 15) to the deep-log gate word next to the P1-b bit 11 it reserved - new bit in the existing u32, record size unchanged; fm_state gains the value 5 (RETURN). No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - 0xF2 return-mode override: adds the fm_return_mode_runtime atomic (0xFF = use the SPIFFS fm_return_mode; 0 / 1 = the remote's session override, carried in 0xF2 bits 5-6) next to fm_mode_runtime, and documents telemetry.fm_flags bit 7 as the RX's echo of its EFFECTIVE return mode for the remote's display and return gesture. Runtime globals + comments only: no confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - SW36: three Follow-Me fields APPENDED at the tail of confStruct - fm_return_mode (u16, 0-1, default 1: when the rider stops, Follow-Me graduates to FM_RETURN and brings the buggy back under the trigger), fm_align_cap (u16, 8-80, default 13: the throttle cap during the FM align phase and the FM_RETURN align/engage ramp, replacing the compile-time kFmAlignCap for FM paths), fm_align_influence (u16, 0-100, default 80: the mixer steering influence during FM align / FM_RETURN align only - 100 = one-motor pivot; 0 = use steering_influence). sizeof 192 -> 200 (192 + 3 x 2 = 198, padded to the 4-byte struct alignment), static_assert 200, SW_VERSION 35 -> 36. Config is NOT reset by this flash: the 192-byte SW35 blob is migrated by prefix at boot (Common/SPIFFSEngine.h, host-tested in Tools/tests/config_migrate_test.cpp) and the three new fields take their defaults.
// V2.5-Evo - 2026-09-19 - C-4 fix (throttle-relative differential mixer): includes ../Common/DifferentialMixer.h (pure mixer, adopted verbatim, host-tested in Tools/tests/differential_mixer_test.cpp) for calcPWM()'s steering_type 1 branch. Include only: no confStruct change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-09-18 - comment only (review finding F6): the fm_gate_flags bit-13 note names kFmReleaseDengageMs, the alias RTMState.ino now uses for the release -> needs-D_engage timer. No code change.
// V2.5-Evo - 2026-09-18 - kFmEngageDistFloorM raised 8.0 -> 9.5 m (code-review finding F3): the rope in use is 7.1 m (owner, 2026-08-28), not the 6.10 m the 8.0 m floor was derived from, so 8.0 m cleared it by only 0.9 m - inside GPS error. 7.1 x 1.31 = 9.3, rounded up. Validator, read-site clamp, trigger-free engage floor and the BOOTSTRAP-1 abort radius all read this one constant. fm_engage_dist_m field comment updated (0, or 9.5-50 m). Compile-time only: no confStruct change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-09-18 - P1-c (Follow-Me stays armed through a Return-to-Me): adds FM_LOG_GATE_YIELD_TO_RTM (bit 14) to the deep-log gate word - a new bit in the existing u32, record size unchanged. No confStruct change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-09-18 - P1-a (trigger-free Follow-Me engagement): includes ../Common/FollowMeEngage.h (pure engage floor + needs-D_engage rule, host-tested in Tools/tests/follow_me_engage_test.cpp) and adds FM_LOG_GATE_PROOF_OK (bit 12) / FM_LOG_GATE_NEEDS_DENGAGE (bit 13) to the deep-log gate word - new bits in the existing u32, record size unchanged, bit 11 stays reserved for P1-b. No confStruct change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-09-17 - DEEP-LOG FM AUDIT COLUMNS (comparison row 13, adapted onto our 65 B level-4 record): VescLogDataL4 gains an 18-byte Follow-Me block (fm_gate_flags u32, fm_distance_dx10, fm_d_engage_dx10, fm_rider_speed_dx10, fm_sep_fix_count, fm_mode, fm_state, fm_block_reason, fm_throttle_cap, fm_station_deg_x10, 1 B pad) -> sizeof 65 -> 83, static_assert 83. The P1/P2 fields (return_candidate / fade_bypass / transit bits, fm_station_deg_x10) are laid out NOW and zero-filled so the record never changes again. PUBLISH-FROM-CONTROLLER: runFmLoop() fills g_fm_log_snapshot under taskENTER_CRITICAL once per 10 Hz tick and Logger.ino copies it - the logger never recomputes a gate. Old 65 B level-4 files still parse: the file header's record_size selects the column set (logCsvHeaderFor) and logFormatCsvRow() guards each block on the bytes actually present. Deep logging at 3 Hz now holds about 2.0 h in 1757 KB (was about 2.5 h). Log record only: NO confStruct change, sizeof(confStruct) stays 192, SW_VERSION stays 35, SPIFFS config is NOT reset by this flash.
// V2.5-Evo - 2026-08-17 - COMMENT-ONLY size correction (no code, no struct, no SW_VERSION change): the mag_orientation block claimed "sizeof 184 -> 188". The finished struct is 192 — mag_orientation (2) plus the two reserved slots rsvd_u16_1 (2) and rsvd_f32_1 (4) that landed in the same SW34->35 edit, naturally aligned with no tail pad. The static_assert has always said 192; only the prose was wrong. Corrected in three places: the mag_orientation block, the "confStruct is 184 bytes" line in the log_level block (retensed as history), and the static_assert's own trailing history, which never recorded the 184->192 step and now does. Flagged as load-bearing rather than cosmetic because the SW34->35 config-backup migration is pinned to the exact counts 184 (legacy) and 192 (current) and disables itself if either stops matching. Every remaining "184" in this file sits inside a dated change-history entry and is correct AS HISTORY — the static_assert is the SSOT for the current size.
// V2.5-Evo - 2026-08-17 - defaultConf.vesc_timeout_s raised 6 -> 10 s, because a VESC cold restart takes roughly 8-9 seconds and 6 s is shorter than that: every restart blanked the rider's battery % and FET temperature to "N/A" on the TX while the VESC was merely booting. 10 s covers the restart and is still half the original hardcoded 20 s. VALUE-ONLY change to an existing field: no field added, moved, renamed or resized, and the 5-60 s validation range in ConfigService is untouched — so sizeof(confStruct) stays 192, the static_assert is unchanged, SW_VERSION stays 35 and the owner's SPIFFS config is NOT wiped by this flash. Because it is NOT wiped, a board with a stored vesc_timeout_s keeps its old value: it must be set on the device with `?set vesc_timeout_s 10` + `?save`. Second consumer checked — RTM Phase C check 2 uses the same field to decide when to SKIP its VESC-ERPM-vs-GPS-speed comparison; a longer timeout means fewer skips, and since a skipped check and a passed check have the identical outcome (no stop) the check's coverage can only widen, never shrink.
// V2.5-Evo - 2026-07-25 - STAGE 2 (heading-source trust guards): added the four shared compile-time constants the RTM/FM heading ladder needs — kRtmCogFrozenMs (3000), kHeadingDisagreeDeg (45.0), kHeadingDisagreeMs (5000), kHeadingCompareSnapMs (1000). They live HERE, once, for the same reason kFmEngageDistFloorM does: getRtmHeading() in RTMState.ino and its inline duplicate in Logger.ino both read them, and Logger.ino is concatenated BEFORE RTMState.ino, so a constant defined in RTMState.ino would be invisible to the logger mirror. Constants + comments only: no confStruct field added, moved, renamed or resized; sizeof(confStruct) stays 184, static_assert unchanged, SW_VERSION stays 34, SPIFFS config is NOT reset by this flash.
// V2.5-Evo - 2026-07-25 - STAGE 1 (GPS repair): defaultConf.gps_update_hz raised 2 -> 10 Hz, because the GPS module is configured for 5 Hz (BN-220/BN-880) or 10 Hz (M10) and, now that STAGE 1 leaves the UART mux parked on GPS, a 500 ms drain interval would leave ~1250 bytes pending per drain. VALUE-ONLY change to an existing field: no field added, moved, renamed or resized, and the 1-10 validation range in ConfigService is untouched — so sizeof(confStruct) stays 184, the static_assert is unchanged, SW_VERSION stays 34 and the owner's SPIFFS config is NOT wiped by this flash. Because it is NOT wiped, a board with a stored gps_update_hz keeps its old value: it must be set on the device with `?set gps_update_hz 10` + `?save`. No control-path field (throttle, steering, PWM, RTM/FM) is touched.
// V2.5-Evo - 2026-07-25 - STAGE 0 (instrumentation only, ZERO control-behaviour change): (A) the unused RESERVED slot fm_steer_reposition_en is RENAMED IN PLACE to log_level — same offset, same uint16_t, so sizeof(confStruct) stays 184, the static_assert is unchanged, SW_VERSION stays 34 and the owner's SPIFFS config is NOT wiped (same trick as dummy_delete_me -> rtm_steer_response, Bundle 1). 0 = unset (behaves exactly as level 3), 1 = Basic, 2 = VESC, 3 = Developer, 4 = Deep; only 3 and 4 are implemented, 1 and 2 are accepted by the validator and currently log as level 3. (B) added LogFileHeader — every log file now starts with an 8-byte self-describing header (magic/format/level/record size) so a reader can parse a variable record size. (C) added VescLogDataL4 = VescLogData + the 4 level-4 diagnostic fields, and the free-running diagnostic counters the ?diag command and the level-4 record read. Nothing here is read by throttle, steering, PWM, the mux schedule, or any FM/RTM logic.
// V2.5-Evo - 2026-07-25 - F3-b: kFmEngageDistFloorM (the FM engage-distance floor) now lives HERE, once, and is raised 5.0 -> 8.0 m. It used to be defined in RTMState.ino AND duplicated as a bare 5.0f literal in ConfigService.ino, on the false premise that the Arduino concatenation order stopped the two files sharing a constant — this header is included at the top of V2_Integration_Rx.ino, which is compiled first, so both see it. 5.0 m was below the hazard it names: the owner's tow rope is 20 ft = 6.10 m, so a manual fm_engage_dist_m of 5.0-6.1 m was storable and let FM engage with the rider still ON the rope. 8.0 m clears a 6.10 m rope by ~1.31x. One shared constant + comments only: no field added, moved or resized; sizeof(confStruct) stays 184, static_assert unchanged, SW_VERSION stays 34, SPIFFS config is NOT reset by this flash.
// V2.5-Evo - 2026-07-25 - A2: fm_engage_dist_m promoted from RESERVED/unread to LIVE — runFmLoop() now reads it as the FM engage distance in METRES (rope length x ~1.15), 0 = auto (unchanged legacy behaviour). Comment/semantics only: no field added, moved or resized; sizeof(confStruct) stays 184, static_assert unchanged, SW_VERSION stays 34, SPIFFS config is NOT reset by this flash.
// V2.5-Evo - 2026-07-24 - F9: VescLogData +6 bytes (tx_distance_dx10, rssi_dbm, snr_dx10) for owner-requested distance + link-quality CSV columns; sizeof 53->59; old SPIFFS logs misparse after this flash; NO confStruct change, SW_VERSION stays 34
// V2.5-Evo - 2026-07-20 - SW33->34 config bump + defaultConf bake: appended THREE reserved confStruct slots — fm_engage_dist_m (float, 0=auto), auton_runtime_cap_s (uint16_t, 0=disabled), fm_steer_reposition_en (uint16_t, 0=off). All three are default-off storage slots and are NOT read by v1 control law — bundled together so the v2 features that will read them need NO second config wipe. sizeof(confStruct) 176->184 (float+u16+u16, naturally aligned, no tail pad); static_assert updated to 184. defaultConf carries the factory default configuration (compass cal fields made explicit, neutral). Behavior-IDENTICAL control law — config-layer only, no FM/RTM logic change. SPIFFS config IS reset by this flash (struct size changed); this is the one intended config-wipe event.
// V2.5-Evo - 2026-07-20 - FM control brain (Fable v1.4): repurposed the unused reserved_tx_imu telemetry byte (index 16) as fm_flags — the coherent FM engagement sub-state the TX display consumes ([0]armed [1]engaged [2]armed-not-ready [3]fault-stop-sticky). No confStruct change, no telemetry-packet size change (byte was already present) — SW_VERSION stays 33, sizeof(confStruct) stays 176, SPIFFS config is NOT reset by this flash.
// V2.5-Evo - 2026-07-19 - P3 FM: added fm_rx_active + fm_throttle_cap runtime atomics for the Follow-Me state machine. No confStruct change (FM reuses the 8 existing FM params) — SW_VERSION stays 33, sizeof stays 176, SPIFFS config is NOT reset by this flash.
// V2.5-Evo - 2026-07-20 - FM engagement semantics: added fm_mode_last_rx_ms atomic (0xF2 declaration age, drives the 95 s mode-age expiry); R6 comment cleanup on the zone_angle_enter/exit + near_diag_offset block (described a non-existent engagement cone, wrong mode numbers, inverted signs, false "CURRENTLY UNUSED"). No confStruct change — sizeof stays 176, SW_VERSION stays 33, SPIFFS config is NOT reset by this flash.
// V2.5-Evo - 2026-07-19 - FM triage: log the steering byte actually applied by calcPWM() (g_effective_steer global + VescLogData.effective_steer_log); VescLogData sizeof 52→53; old SPIFFS logs misparse after this flash; no confStruct change, SW_VERSION unchanged
// V2.5-Evo - 2026-07-18 - FM mode mapping canonicalized to TX convention (1=Near-Right, 2=Behind, 3=Near-Left). Labels/comment only — no struct/SW_VERSION change. [2026-07-24 F4 correction: the earlier "2→1 preserves Near-Right default" note was stale — that edit never landed; defaultConf.followme_mode is and stays 2 (Behind), the shipped defensive default. All surfaces now agree on 2.]
// V2.5-Evo - 2026-05-22 - SW32: Two-phase RTM throttle: rtm_align_threshold_deg + rtm_target_speed_kmh; sizeof 164→172; SW_VERSION 31→32
// V2.5-Evo - 2026-05-09 - Bundle 9-Final: Added USB CDC On Boot compile-time guard
// V2.5-Evo - 2026-05-11 - E7 Fix: VescLogData +1 byte (error_code_log); sizeof 51→52; old SPIFFS logs misparse after this flash
// V2.5-Evo - 2026-05-08 - Bundle 1: RTM/FM steering preset system (rtm_steer_response 0-4); SW_VERSION 30→31; sizeof unchanged at 164; VescLogData +4 bytes for tuning telemetry
// V2.5-Evo - 2026-05-06 - LOG-EXT-1: VescLogData extended with heading source debug fields (12 fields, +18 bytes)
// V2.5-Evo - 2026-05-06 - D3-Fix: rtm_use_compass + rtm_cog_min_speed_kmh changed uint8_t→uint16_t for ConfigService CFG_U16 compatibility; SW_VERSION 29→30; sizeof 160→164
// V2.5-Evo - 2026-05-06 - D3: Added rtm_use_compass + rtm_cog_min_speed_kmh; sizeof stays 160 (fills tail pad); SW_VERSION 28→29
// V2.5-Evo - 2026-05-01 - Release: DEBUG_RX commented out for production build
// V2.5-Evo - 2026-04-30 - RTM approach decel zone: rtm_approach_zone_m SPIFFS param; rtm_approach_cap atomic global; sizeof 156→160
// V2.5-Evo - 2026-04-30 - Rename: gps_max_jump_kmh → gps_max_teleport_kmh (clarity)
// V2.5-Evo - 2026-04-29 - Bundle B: vesc_timeout_s SPIFFS param replaces hardcoded 20s VESC timeout
// V2.5-Evo - 2026-04-22 - Added gps_chip_type field to confStruct (GPS module selector); sizeof 108→112; updated defaultConf
// V2.5-Evo - 2026-04-22 - Added Phase A GPS anti-spoofing params to confStruct; sizeof 112→128; updated defaultConf
// V2.5-Evo - 2026-04-24 - Added rx_tx_gps_lat/lng/timestamp globals for 0xF3 meta-packet reception
// V2.5-Evo - 2026-04-24 - Added Phase B GPS handshake params to confStruct; sizeof 128→136; updated defaultConf
// V2.5-Evo - 2026-04-25 - P7: Added RTM Phase C + RX safety params; VESC_MORE_VALUES; sizeof 136→152
// V2.5-Evo - 2026-04-27 - P8: TelemetryPacket adds rtm_distance at index 5; link_quality moved to index 6
// V2.5-Evo - 2026-04-25 - P7: Added rtm_rx_active, rtm_rx_emergency_stop, rtm_steer_override, fm_mode_runtime globals
// V2.5-Evo - 2026-04-25 - P7 fix: Changed RTM volatile globals to std::atomic for cross-core safety (core 0 PWM task / core 1 loop task)

// ============================================================
// V2.5-Evo - 2026-05-09 - Bundle 9-Final: USB CDC On Boot guard
//
// ESP32-C3 chip-level hardware default: GPIO 18 = USB D-, GPIO 19 = USB D+.
// RX firmware uses GPIO 18/19 as UART for the BN-880 GPS via Serial1.
// If "USB CDC On Boot" is enabled at compile time, the ESP32-C3 USB
// peripheral claims GPIO 18/19 internally and Serial1.begin() silently
// fails — GPS init never reaches the module, no fix is ever acquired,
// hours of debugging follow.
//
// REQUIRED: Arduino IDE → Tools → USB CDC On Boot → Disabled
//   OR     arduino-cli --fqbn esp32:esp32:esp32c3:CDCOnBoot=default
//
// Debug Serial output goes via UART0 (GPIO 20/21) → CH340 USB-to-UART chip
// → USB connector. Same physical USB cable, same COM port, no debug loss.
// ============================================================
#if defined(ARDUINO_USB_CDC_ON_BOOT) && (ARDUINO_USB_CDC_ON_BOOT != 0)
#error "RX firmware requires USB CDC On Boot = Disabled. ESP32-C3 USB peripheral claims GPIO 18/19 (used by Serial1 for GPS) when CDC On Boot is enabled. Set Tools -> USB CDC On Boot -> Disabled in Arduino IDE, OR pass :CDCOnBoot=default to arduino-cli's --fqbn argument. See file header for full explanation."
#endif

/*
** Includes
*/
#include <Arduino.h>

// V2.5-Evo - 2026-10-03 - RX WEB CONSOLE. MUST sit immediately after <Arduino.h> and BEFORE every
// one of our own headers (DifferentialMixer, OutputRamp, ConfigServiceEngine, SPIFFSEngine,
// WebConfigEngine, RadioCommon, SystemCommon) and before all the .ino files. It redefines what the
// word `Serial` means, so anything included ahead of it binds to the real port and is silently
// left out of the capture - a hole in the record that looks like working code. The header's own
// comment explains why its position is load-bearing for static-initialisation order as well.
// Nothing about this include is optional or reorderable.
#include "../Common/SerialTee.h"

#include <atomic>
// V2.5-Evo - 2026-09-19 - C-4 fix: the differential-steering mixer calcPWM() (PWM.ino) uses in
// steering_type 1 is a pure function in ../Common/DifferentialMixer.h so
// Tools/tests/differential_mixer_test.cpp runs the exact code the RX runs. It is throttle-relative
// (turn term proportional to effective_thr), so steering can no longer add motor power past the
// RTM/FM throttle caps or create any motor command at zero gas. Header adopted verbatim; it defines
// no constants and reads no config of its own — influence and inversion are passed in from usrConf.
#include "../Common/DifferentialMixer.h"
// V2.5-Evo - 2026-09-19 - the motor rise-limit (the "motor ramp") calcPWM() applies is a pure function
// in ../Common/OutputRamp.h (host-tested in Tools/tests/output_ramp_test.cpp) so the two-ramp selector
// (motor_ramp_s for manual, auto_ramp_s for the automatic modes) shares one memory and one step.
// V2.5-Evo - 2026-09-24 - two corrections to the line above, both from the throttle-ramp move: it is
// applied BEFORE the mixer, not after (rate-limiting the two finished motor outputs also rate-limited
// the difference between them, which is the steering), and the arithmetic is NOT the old inline
// block's - the header is a single-channel 0-255 throttle ramp with a Q12 accumulator, because a
// whole-count step in that domain floors at 2.55 s and cannot express the configured 0.2-4.0 s range.
#include "../Common/OutputRamp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <RadioLib.h> //V7.1.2
#include <Wire.h>
#include <Adafruit_AW9523.h> //V1.0.5, BusIO 1.17.0
#include "driver/rmt_tx.h"
#define RMT_TX_GPIO_NUM  GPIO_NUM_9
#include <Ticker.h>
#include "esp_task_wdt.h"
// V2.5-Evo - 2026-07-25 - STAGE 0: esp_cpu_get_cycle_count() for loop() timing. It is a single
// CSR read (~5 ns) versus ~1 us for micros(), so measuring the loop does not distort the loop.
#include "esp_cpu.h"
#include "FS.h"
#include "SPIFFS.h"
#include "mbedtls/base64.h"

// Uncomment the line below to enable WiFi AP configuration mode
#define WIFI_ENABLED

#ifdef WIFI_ENABLED
#include <WiFi.h>
#include <WebServer.h>
#endif

#include "vesc_datatypes.h"
#include "vesc_buffer.h"
#include "vesc_crc.h"

#include <TinyGPS++.h> //TinyGPSPlus 1.0.3 Mikal Hart

// V2.5-Evo - 2026-08-29 - Minimum storable tx_gps_stale_timeout_ms.
// The staleness test is (age > timeout), so ZERO makes every rider fix stale within a
// millisecond: RTM Gate 4 fails permanently, Follow-Me condition 4 carries the identical test and
// fails with it, and BOTH autonomous modes are dead until the value is put back. The web UI used
// to describe 0 as "disabled (never stale)", which is the exact opposite of what it does, so a
// rider relaxing the check would have bricked the feature they were relaxing.
//
// 500 ms is a floor, not a recommendation. It is well clear of "instantly stale" while still
// allowing tighter-than-default tuning; at 500 ms a 1 Hz TX GPS is degraded but recoverable,
// where at 0 it is dead. Enforced by CLAMPING in cfgValidateCrossField() rather than by narrowing
// the kCfgFields range: that validator runs on the LOAD path too, and a range rejection there
// fails the load and falls back to defaults - which would wipe the config, pairing and compass
// calibration of anyone who had already stored a 0. A clamp corrects them silently instead.
static const uint16_t kTxGpsStaleFloorMs = 500;

// V2.5-Evo - 2026-09-19 - auto_ramp_s bounds: 0 = inherit motor_ramp_s, else kAutoRampMinS..kAutoRampMaxS
// (the same 4 s ceiling motor_ramp_s has). Enforced by CLAMPING in cfgValidateCrossField() for the
// same reason as the floor above: it runs on the LOAD path, and the slot this field reuses was a
// RESERVED float validated -1e6..1e6, so a stored blob may in principle hold anything. A value in
// (0, 0.2) is raised to 0.2 with a NOTE (a ramp that short is instant in all but name); below 0 or
// NaN becomes 0 (inherit) and above 4 becomes 4, silently. The kCfgFields row is 0-4.0 like
// motor_ramp_s, so a typed value above the ceiling is refused the way motor_ramp_s refuses it.
static const float kAutoRampMinS = 0.2f;   // s; smallest non-zero auto_ramp_s
static const float kAutoRampMaxS = 4.0f;   // s; the ceiling, shared with motor_ramp_s

#define SW_VERSION 36  // V2.5-Evo - 2026-10-02 - STILL 36, AND THE TAIL PAD IS NOW SPENT: fm_front_angle_deg (P2 front stations) took the 2 B alignment pad named below, so sizeof is still 200, this flash does NOT reset config, and the NEXT new scalar field is a real SW37 bump with a real wipe - there is no free space left in this struct and both banked reserved slots went on 2026-09-19 (steer_during_auto, auto_ramp_s). When SW37 comes, bank four slots. // V2.5-Evo - 2026-09-19 - 36 = fm_return_mode / fm_align_cap / fm_align_influence appended at the tail; sizeof 192->200 (198 + 2 B alignment pad). Config is NOT reset by this flash: the 192-byte SW35 blob is migrated by prefix (every stored value keeps its offset and its value) and the three new fields default (1 / 13 / 80) - see the LEGACY CONFIG BLOB MIGRATION block in Common/SPIFFSEngine.h. 35 = mag_orientation appended (compass mounting rotation); sizeof 184->192 (mag_orientation + 2 reserved slots banked for future no-bump features), config IS reset by this flash. 34 = added fm_engage_dist_m / auton_runtime_cap_s / fm_steer_reposition_en reserved slots + defaultConf carries factory default config (compass cal, near_diag_offset 45); first flash resets all RX SPIFFS config to defaults. NOTE (2026-07-25, STAGE 0 PART A): the third of those slots has since been RENAMED IN PLACE to log_level — same offset, same uint16_t, sizeof(confStruct) still 184 — so this stays 34 and NO further config wipe happens.
const char* CONF_FILE_PATH = "/data.txt";
const char* BC_FILE_PATH = "/batconf.txt";

/*
** Structs
*/
struct confStruct {
    //Version
    uint16_t version;
    
    uint16_t radio_preset; //1: 868MHz (EU), 2: 915MHz (US/AU)
    int16_t rf_power; //Tx power from -9 to 22

    uint16_t steering_type; //0: single motor, 1: diff motor, 2: servo
    uint16_t steering_influence; //How much (percentually) the steering influences the motor speeds
    uint16_t steering_inverted; //If steering is inverted or not
    int16_t trim; //Trim the steering

    //PWM min and max
    uint16_t PWM0_min;
    uint16_t PWM0_max;
    uint16_t PWM1_min;
    uint16_t PWM1_max;

    uint16_t failsafe_time; //Time after last packet until failsafe

    //Foil battery voltage settings
    uint16_t foil_num_cells; //Amount of cells in series e.g. 14 for a "14SxP" pack

    //Sensors
    uint16_t bms_det_active;
    uint16_t wet_det_active;

    uint16_t rtm_steer_response;  // Steering preset index 0-4 for RTM/FM heading controller.
                                  // 0 = Very Soft (big waves, aggressive surfer)
                                  // 1 = Soft (choppy water)
                                  // 2 = Normal (DEFAULT — mixed conditions)
                                  // 3 = Sharp (calm water, RC use)
                                  // 4 = Very Sharp (glass-flat, no waves)
                                  // Drives kSteerPresets[] table in RTMState.ino which
                                  // sets PID gains (Kp, Kd) + bearing filter time constant.

    //UART config
    uint16_t data_src; //0: off, 1:analog, 2: VESC UART

    // GPS features related flags
    uint16_t gps_en;         // GPS runtime enable flag (0=disabled, 1=enabled)
    uint16_t followme_mode;  // Follow-me mode (0=disabled, 1=near_right, 2=behind, 3=near_left) — canonical mapping, matches TX + README
    uint16_t kalman_en;      // Kalman filter runtime enable flag (0=disabled, 1=enabled)

    //Follow-me
    float boogie_vmax_in_followme_kmh; // Maximum boogie speed in follow-me mode (km/h)
    float min_dist_m; // minimum allowed distance to the foiler
    float followme_smoothing_band_m; // smoothing band above min distance
    float foiler_low_speed_kmh; // low-speed threshold for safety stop (hysteresis)
    // V2.5-Evo - 2026-07-20 - R6: comment block corrected. It previously described an
    // "engagement cone" gate that does not exist, marked all three params "CURRENTLY UNUSED"
    // (the FM geometry consumes all three), and gave the wrong mode numbers with the wrong
    // signs for the diagonal offset. The values and ranges themselves are unchanged.

    // DIAGONAL-BLEND SCHMITT — ENTER half-angle (degrees). This is NOT an engagement gate:
    // it decides whether the buggy is lined up closely enough BEHIND the rider to apply the
    // diagonal side offset, or whether it should just sit directly behind. Measured as the
    // angle between the rider->buggy bearing and "directly behind the rider".
    // Below this angle the diagonal offset is applied (see computeFmTarget in RTMState.ino).
    // Range: 5-90°. Default 35°.
    float zone_angle_enter_deg;

    // DIAGONAL-BLEND SCHMITT — EXIT half-angle (degrees). Once the diagonal is applied it is
    // dropped again only when the off-axis angle exceeds this value. MUST be > zone_angle_enter_deg
    // by 5-15° — the hysteresis stops an unstable rider course from whipping the target point
    // across the rider's wake from one side to the other.
    // Range: 10-95°. Default 45°.
    float zone_angle_exit_deg;

    // NEAR-MODE DIAGONAL OFFSET (degrees from "directly behind the rider").
    // Applied as target_bearing = rider_course + 180 + offset, in this board's one bearing
    // convention (degrees CLOCKWISE from North), where "Near-Right"/"Near-Left" mean the side
    // the buggy ends up on RELATIVE TO THE RIDER as the rider faces along their course:
    //   followme_mode=1 (Near Right): offset = -near_diag_offset_deg  (behind-and-right)
    //   followme_mode=2 (Behind)    : offset = 0
    //   followme_mode=3 (Near Left) : offset = +near_diag_offset_deg  (behind-and-left)
    // 0° = directly behind, 90° = beside the rider. Diagonal placement keeps the buggy out of
    // the rider's wake/spray path. Authoritative derivation: the OFFSET SIGN CONVENTION block
    // above computeFmTarget() in RTMState.ino.
    // Range: 0-90°. Default 45°.
    float near_diag_offset_deg;
    
    //System parameters
    float ubat_cal; //ADC to volt cal for bat meas
    float ubat_offset; //Offset to add to analog/vesc measurement

    uint16_t tx_gps_stale_timeout_ms; // TX GPS data stale timeout (ms)

    //Logger
    uint16_t logger_en; // BREmote Logger runtime enable flag (0=disabled, 1=enabled)

    //Comms
    uint16_t paired;
    uint8_t own_address[3];
    uint8_t dest_address[3];
    char wifi_password[8];  // WPA2 AP password, exactly 8 chars (no null terminator)

    // ---> NEW COMPASS CALIBRATION VARIABLES <---
    int16_t mag_offset_x;
    int16_t mag_offset_y;
    float mag_scale_x;
    float mag_scale_y;

    // ============================================================
    // V2.5-Evo - 2026-04-22 - GPS CHIP TYPE SELECTOR
    //
    // !!! IMPORTANT: Adding this field changed sizeof(confStruct)  !!!
    // !!! from 108 bytes to 112 bytes.                             !!!
    // !!! On first V2.5-Evo boot after this change, SPIFFS config  !!!
    // !!! failed the size check — ALL RX SETTINGS RESET TO        !!!
    // !!! DEFAULTS. One-time migration; complete on existing units. !!!
    // ============================================================
    uint16_t gps_chip_type;  // 0=BN-220, 1=BN-880+compass (default), 2=M10 no compass, 3=M10+compass; range 0-3

    // ============================================================
    // V2.5-Evo - 2026-04-22 - PHASE A GPS ANTI-SPOOFING PARAMETERS
    //
    // These four parameters control the always-on Phase A anti-
    // spoofing filter in GPS.ino. A reading is rejected if ANY
    // check fails. After gps_suspect_threshold consecutive
    // rejections, gps_rejected is set and RTM arming is blocked.
    //
    // !!! Adding these fields changed sizeof(confStruct) 112→128.  !!!
    // !!! On first V2.5-Evo boot after this change, SPIFFS reset  !!!
    // !!! ALL settings to defaults. One-time migration; complete.  !!!
    // ============================================================
    float    gps_max_hdop;            // Max HDOP for a valid fix; range 0.5-5.0; default 2.0; dimensionless
    float    gps_max_accel_g;         // Max implied acceleration between readings; range 1.0-10.0G; default 3.0G
    float    gps_max_teleport_kmh;        // Max position-implied speed for teleport check; range 50-500 km/h; default 80
    uint16_t gps_suspect_threshold;   // Consecutive failures before GPS marked rejected; range 1-10; default 3

    // ============================================================
    // V2.5-Evo - 2026-04-24 - PHASE B GPS HANDSHAKE ANTI-SPOOFING PARAMETERS
    //
    // These two parameters control Phase B, which runs every time a
    // 0xF3 GPS meta-packet is received from TX (at most every 30s).
    //
    // Distance check: TX-RX Haversine distance must be <
    //   gps_max_pair_dist_m or RTM arming is blocked.
    // Speed consistency check: TX implied speed (from consecutive
    //   meta-packet positions) must be within gps_max_speed_diff_kmh
    //   of RX GPS speed or arming is blocked.
    //
    // !!! Adding these fields changes sizeof(confStruct) 128→136. !!!
    // !!! On first flash after this change, SPIFFS resets ALL      !!!
    // !!! settings to defaults. After flashing:                    !!!
    // !!!   1) Re-pair TX and RX                                   !!!
    // !!!   2) Re-configure all settings via web UI                !!!
    // !!!   3) Re-calibrate compass (runcal)                       !!!
    // !!!   4) Verify Phase B defaults (500 m, 50 km/h)            !!!
    // ============================================================
    float gps_max_pair_dist_m;      // Max plausible TX-RX distance at handshake; range 50-2000 m; default 500 m
    float gps_max_speed_diff_kmh;   // Max TX-RX speed difference for handshake; range 10-200 km/h; default 50 km/h

    // ============================================================
    // V2.5-Evo - 2026-04-25 - PRIORITY 7: RTM PHASE C + RX SAFETY PARAMETERS
    //
    // sizeof grows 136->152. Layout:
    //   float rtm_vesc_speed_diff_kmh  (4)
    //   float vesc_erpm_per_kmh        (4)
    //   uint16_t rtm_rx_enabled        (2)
    //   uint16_t rtm_rx_override_steer (2)
    //   uint16_t rtm_compass_required  (2)
    //   uint16_t rtm_stop_distance_m   (2) — fills former 2-byte tail padding; sizeof stays 152
    //
    // First flash of P7 firmware resets all RX settings to defaults.
    // After flashing: re-pair TX/RX, re-enter all settings, re-run runcal.
    // ============================================================
    float    rtm_vesc_speed_diff_kmh;    // Phase C: max GPS vs VESC speed diff; 5-50 km/h; default 20.0
    float    vesc_erpm_per_kmh;          // ERPM per km/h (vehicle-specific); default 0.0 (0=skip VESC check)
    uint16_t rtm_rx_enabled;             // RX-side RTM master enable; 0=off, 1=on; default 1
    uint16_t rtm_rx_override_steering;   // Allow RTM to override steering; 0=off, 1=on; default 1
    uint16_t rtm_compass_required;       // Require valid compass for RTM arming; 0=no, 1=yes; default 1
    uint16_t rtm_stop_distance_m;        // Hard stop radius in metres; RTM stops when within this dist of TX; 1-50; default 10

    // V2.5-Evo - 2026-04-29 - BUNDLE B: VESC UART TIMEOUT
    // Set to 6s (down from the original hardcoded 20s) to minimise stale VESC data.
    // At 20s the TX display would show a valid battery % and FET temp for up to 20s after
    // the VESC UART connection dropped — misleading the rider. 6s matches the typical
    // VESC polling cadence (data_src=2 polls every ~1s) with room for 5 missed packets.
    // Minimum 5s: going lower causes false N/A during normal VESC dropout transients
    // (e.g. heavy regen braking briefly interrupts UART). Maximum 60s for diagnostic use.
    //
    // V2.5-Evo - 2026-08-17 - FACTORY DEFAULT RAISED 6 -> 10 s. The 5-60 s validation range is
    // NOT changed; only the value baked into defaultConf below.
    // WHY: a VESC cold restart takes roughly 8-9 seconds, which is LONGER than the 6 s window.
    // So every time the VESC rebooted, the rider's battery % and FET temperature blanked to
    // "N/A" on the TX display even though nothing was actually wrong — the VESC was simply
    // booting. 10 s covers that restart with about a second to spare, and it is still half the
    // original hardcoded 20 s, so Bundle B's intent (do not show a stale reading long after a
    // real disconnect) is preserved. The price is that a GENUINE VESC disconnect now holds the
    // last reading for up to 10 s instead of 6 s before it blanks.
    // SECOND CONSUMER, CHECKED: this same field also decides when RTM Phase C check 2 SKIPS its
    // VESC-ERPM-vs-GPS-speed comparison (runPhaseC() in RTMState.ino). A longer timeout means
    // FEWER skips — the check now runs on VESC data up to 10 s old instead of 6 s. That cannot
    // weaken the check, because a skipped check and a passed check have the identical outcome
    // (no stop); it only widens the window in which slightly staler ERPM is compared against a
    // live GPS speed. And it is moot on a stock board: the check only runs when
    // vesc_erpm_per_kmh > 0, whose factory default is 0.0 (disabled).
    uint16_t vesc_timeout_s;  // 5-60 s; default 10; how long without a VESC UART packet before bat/temp shown as N/A

    // V2.5-Evo - 2026-04-30 - BUNDLE E: GPS POLLING RATE
    // V2.5-Evo - 2026-07-25 - STAGE 1: factory default raised 2 -> 10 Hz (see defaultConf below).
    uint16_t gps_update_hz;   // 1-10 Hz; default 10; how often per second to drain the GPS UART (10=100ms, 5=200ms, 2=500ms)

    // V2.5-Evo - 2026-04-30 - RTM APPROACH DECEL ZONE
    // Distance from TX at which the approach throttle ramp begins during active RTM.
    // Throttle cap = thr × (dist − rtm_stop_distance_m) / (rtm_approach_zone_m − rtm_stop_distance_m)
    // Result: full throttle at the outer edge; cap reaches 0 at rtm_stop_distance_m; Gate 9 hard stop still applies.
    // Set to 0 to disable the decel zone and use Gate 9 hard stop only.
    uint16_t rtm_approach_zone_m;  // 0=disabled, 5-100 m; default 15; outer edge of RTM approach decel zone

    // ============================================================
    // V2.5-Evo - 2026-05-06 - D3: RTM HEADING SOURCE SELECTION
    //
    // These two parameters control which heading source RTM steering uses.
    // Bench-test data (?magtest) confirmed compass-only steering is unsafe
    // on this hardware: motor current biases compass heading by 100° or more
    // even at 20% throttle. GPS course-over-ground (COG) is unaffected by
    // motor EMI and is the preferred heading source whenever the buggy is
    // moving fast enough for COG to be reliable (~3 km/h default).
    //
    // Modes:
    //   0 = GPS COG only — compass disabled for steering (safest if compass biased)
    //   1 = Hybrid (DEFAULT) — GPS COG primary, compass snapshot at low speed
    //   2 = Compass only — DIAGNOSTIC USE, DO NOT USE ON WATER. Bench tests confirm
    //                      motor current biases compass by 100° or more during
    //                      operation. Setting this on water risks RTM steering
    //                      the buggy in the wrong direction. For non-EMI builds
    //                      that have proven clean compass behavior under load only.
    //
    // !!! Adding these fields changes sizeof(confStruct) 160→164,         !!!
    // !!! and bumps SW_VERSION 29→30. SPIFFS resets ALL settings to       !!!
    // !!! defaults again (the second time, because D3-Fix changes layout). !!!
    // !!! After flashing:                                                  !!!
    // !!!   1) Re-pair TX and RX                                           !!!
    // !!!   2) Re-configure all settings via web UI                        !!!
    // !!!   3) Re-calibrate compass (runcal)                               !!!
    // !!!   4) Verify rtm_use_compass = 1 (hybrid default)                 !!!
    // !!!   5) Verify rtm_cog_min_speed_kmh = 3                            !!!
    // ============================================================
    uint16_t rtm_use_compass;        // 0=GPS COG only; 1=Hybrid (default); 2=Compass only DIAGNOSTIC ONLY DO NOT USE ON WATER
    uint16_t rtm_cog_min_speed_kmh;  // Min GPS speed for COG to be primary heading source; 1-15 km/h; default 3

    // ============================================================
    // V2.5-Evo - 2026-05-22 - SW32: TWO-PHASE RTM THROTTLE CONTROL
    //
    // Phase 1 (Align): when |heading_error| > rtm_align_threshold_deg, throttle is
    //   suppressed to ~5% so the buggy pivots toward the target without driving away.
    //   At near-zero throttle, motor current is minimal — compass bias is also reduced,
    //   so hybrid heading mode has cleaner compass snapshot data during alignment.
    //
    // Phase 2 (Run): once aligned, throttle is governed by GPS speed so behaviour is
    //   consistent across different boogies regardless of motor/prop curve.
    //   rtm_target_speed_kmh == 0 disables the governor (approach decel zone only).
    //
    // sizeof grows 164 → 172. SW_VERSION 31 → 32. First flash resets SPIFFS config.
    // ============================================================
    float    rtm_target_speed_kmh;      // Phase 2 run speed cap (GPS-based); 0=disabled; 0-20 km/h; default 4.0
    uint16_t rtm_align_threshold_deg;   // Phase 1→2 transition: heading error below which run phase begins; 10-90°; default 45

    // V2.5-Evo - 2026-06-05 - SW33: MOTOR RAMPING (seconds). Time for a motor output to rise
    // 0->full. Applied to BOTH motor channels — smooths the throttle AND prevents a single motor
    // from taking off (throttle- or steering-driven). Fall is instant (release/failsafe/e-stop/
    // straightening drop immediately). NOTE: this also ramped differential steering — a sharp turn
    // built over this time. 0 = instant/off. sizeof grows 172->176; SW_VERSION 32->33; SPIFFS resets.
    //
    // V2.5-Evo - 2026-09-24 - THE STEERING SENTENCE ABOVE IS HISTORY. It was true because the
    // rise-limit sat AFTER the differential mixer, on PWM0_time / PWM1_time - and the difference
    // between those two outputs IS the turn, so it slewed steering too (on steering_type 2 it slewed
    // the steering servo itself, which carries no throttle at all). calcPWM() now rate-limits the
    // THROTTLE instead: effective_thr, after every cap, before the mixer. So this field ramps
    // THROTTLE ONLY - steering is never rate-limited and a turn lands on the next 10 ms pass. Fall
    // is still instant (release / failsafe / RTM / FM stop). The ramp accumulates in Q12 (1/4096 of
    // a throttle count), so the whole 0.20-4.00 s range is honoured to within one 10 ms tick and is
    // never faster than configured; the per-setting numbers are in PWM.ino's file header.
    float    motor_ramp_s;              // 0=off/instant, 0-4 s; default 0.75; THROTTLE only — steering is never ramped

    // V2.5-Evo - 2026-07-20 - SW34 reserved slots (added together so only ONE config wipe is needed).
    // They were all storage slots at SW34 so v2 features could be code-only, with no re-wipe.
    // V2.5-Evo - 2026-07-25 - A2 status update: fm_engage_dist_m is now LIVE (read by runFmLoop()).
    // V2.5-Evo - 2026-07-25 - STAGE 0 PART A status update: the third slot, fm_steer_reposition_en,
    // has been RENAMED IN PLACE to log_level and is now LIVE (read by the logger). auton_runtime_cap_s
    // is the only one of the three still RESERVED and still read by nothing.
    // fm_engage_dist_m: LIVE since 2026-07-25 (A2) — no longer RESERVED/unread. Fixed engage-distance
    //   override for the FM separation latch, read in RTMState.ino runFmLoop().
    //   0   = auto: d_engage = kFmEngageFactor (1.5) * (min_dist_m + followme_smoothing_band_m), the
    //         original behaviour, reproduced exactly.
    //   >0  = the engage distance itself, in METRES. This is NOT the rope length — set it to rope
    //         length x ~1.3 so the buggy clears the rope with margin (the 7.1 m rope -> 9.5).
    //         V2.5-Evo - 2026-07-25 - F3-b: legal non-zero values start at kFmEngageDistFloorM
    //         (defined below this struct); (0, floor) is clamped up by cfgValidateCrossField().
    //         V2.5-Evo - 2026-09-18 - the floor is 9.5 m (was 8.0, derived for a 6.10 m rope; the
    //         rope in use is 7.1 m). See the constant's block for the derivation and knock-ons.
    //   HOW THE RIDER PICKS THIS VALUE: measure your own tow rope and set this to about a third
    //   more than the rope length, so Follow-Me only engages once you have genuinely let go and
    //   separated. Example: a 7.1 m rope -> set 9.5 m or more. Setting it at or below your rope
    //   length lets FM engage while you are still on the rope. 9.5 m is the enforced minimum, not a
    //   recommendation — a longer rope needs a bigger number.
    float    fm_engage_dist_m;         // 0 = auto; >0 = fixed engage distance in metres; 0, or 9.5-50 m
    // V2.5-Evo - 2026-08-16 - RENAMED IN PLACE: auton_runtime_cap_s -> gps_dyn_model.
    // Same offset, same uint16_t, so sizeof(confStruct) stays 184, the static_assert is
    // unchanged, SW_VERSION stays 34 and NOBODY'S CONFIG IS WIPED. Same trick as
    // fm_steer_reposition_en -> log_level (2026-07-25) and dummy_delete_me ->
    // rtm_steer_response (Bundle 1). The old field was RESERVED and never read by v1, so
    // no behaviour is displaced. Every board in the field currently holds 0 here — which is
    // why 0 MUST mean "the previous hard-coded behaviour", i.e. Sea.
    //
    // u-blox NAV5 dynamic platform model, applied by configureGPS().
    //   0 = default -> Sea (5). What every existing board already does.
    //   4 = Automotive — REQUIRED above ~500 m; Sea's altitude ceiling is 500 m.
    //   5 = Sea (explicit) — best below 500 m: constrains the filter to ~25 m/s and pins
    //       altitude, which sharpens course-over-ground, and COG is what Follow-Me steers on.
    // Deliberately NOT offering dynModel 0 (Portable): it permits 310 m/s and is what produced
    // the bogus 254 km/h / 4800 m HIGH-CONFIDENCE fixes. No reason to expose it.
    uint16_t gps_dyn_model;            // 0 = default (Sea) | 4 = Automotive | 5 = Sea
    // ============================================================
    // V2.5-Evo - 2026-07-25 - STAGE 0 PART A: log_level (IN-PLACE RENAME, NO SPIFFS WIPE)
    //
    // This slot used to be fm_steer_reposition_en — a RESERVED Option-C storage slot added at
    // SW34 that no code ever read and that has therefore been sitting in every stored config as
    // a plain 0. It is RENAMED IN PLACE here: same position in the struct, same uint16_t type,
    // so sizeof(confStruct) stays 184, the static_assert below is unchanged, SW_VERSION stays 34
    // and the owner's saved settings are NOT reset by this flash. There is precedent for exactly
    // this move: dummy_delete_me -> rtm_steer_response (Bundle 1, 2026-05-08).
    //
    // WHY THE RENAME RATHER THAN A NEW FIELD: confStruct was 184 bytes with no tail padding, so
    // appending anything grows it, which fails the SPIFFS size check on the next boot and wipes
    // the rider's entire configuration (pairing, calibration, all tuning). Reusing a dead slot
    // that is already 0 everywhere costs nothing and wipes nothing.
    // (V2.5-Evo - 2026-08-17 - tense fix: the 184 above is the SW34-era size this rename was
    // written against. The struct is 192 bytes today — SW35 appended mag_orientation and two
    // reserved slots and spent the one config wipe. The static_assert below is the SSOT for the
    // current size; every "184" in this block is history, not a current fact.)
    //
    // WHAT THE FM v2 FEATURE DOES NOW: the Option-C "steer reposition" idea has NOT been
    // implemented and has NOT been abandoned — when FM v2 lands it must claim a FRESH field of
    // its own (which will be a deliberate, announced config-wipe event), not this slot.
    //
    // WHAT THE VALUES MEAN — how much detail each recorded log line carries:
    //   0 = UNSET. Behaves EXACTLY as level 3. This is what every existing config already
    //       stores, so nothing changes for anyone who never touches this setting.
    //   1 = Basic   — RESERVED for a future storage optimisation; CURRENTLY LOGS AS LEVEL 3.
    //   2 = VESC    — RESERVED for a future storage optimisation; CURRENTLY LOGS AS LEVEL 3.
    //   3 = Developer — the full 59-byte record this firmware has always written.
    //   4 = Deep      — Developer plus the 6-byte diagnostic block (GPS sentence rate, COG
    //       frozen-seconds, mux read-back error count, worst loop() time) = 65 bytes/record.
    // Levels 1 and 2 are ACCEPTED by the validator (so a rider can set them and a future
    // firmware will honour them) but are deliberately NOT silently ignored: they are documented
    // everywhere as falling back to level 3 until the smaller records are implemented.
    //
    // The level is latched once per log FILE (createNewLogFile() in Logger.ino) and written into
    // that file's header, so changing this setting mid-session never corrupts an open file.
    // ============================================================
    // V2.5-Evo - 2026-09-19 - level 5 = Everything: the 87 B Deep record plus a 22 B block (rider
    // position + freshness, RTM phase and approach cap, the align/mixer influences in force, the
    // auto-return override, fm_flags as sent, the keepalive age, two reserved bytes) = 109 B/record,
    // for test sessions only (about 1 h 30 min at 3 Hz). Same u16 slot; the validator max is 5 now
    // and cfgValidateCrossField() CLAMPS anything above 5 down to 5 on every path (a range rejection
    // on the load path would wipe the config). See VescLogDataL5 below.
    uint16_t log_level;                // 0 = unset (= level 3); 1 = Basic*, 2 = VESC*, 3 = Developer, 4 = Deep, 5 = Everything. (*accepted, currently logs as level 3.) Range 0-5.

    // V2.5-Evo - 2026-08-16 - SW34->35: mag_orientation. NEW FIELD, appended at the END of
    // confStruct so every existing offset is unchanged, and SW_VERSION 34 -> 35, which DOES reset
    // config on first boot. This is the one intended wipe for this bump.
    // V2.5-Evo - 2026-08-17 - SIZE CORRECTION: this comment said "sizeof 184 -> 188 (the uint16 plus
    // 2 bytes of 4-byte alignment padding)". THE FINISHED STRUCT IS 192 BYTES, not 188 — see the
    // static_assert below, which is the SSOT. 188 was the size mag_orientation alone would have
    // produced, but the two banked RESERVED slots (rsvd_u16_1 + rsvd_f32_1, documented under this
    // field) landed in the SAME edit and account for the other 4 bytes: 184 + 2 (mag_orientation)
    // + 2 (rsvd_u16_1) = 188, + 4 (rsvd_f32_1) = 192, naturally aligned with no tail pad.
    // THIS IS NOT COSMETIC: the SW34 -> SW35 config-backup migration is pinned to the exact byte
    // counts 184 (legacy) and 192 (current) and disables itself if either stops matching, so a
    // comment claiming 188 would have someone reasoning from the wrong number about code that
    // writes bytes into the live config.
    //
    // Mounting rotation of the compass module about the vertical axis, in degrees, applied to the
    // computed heading. Set by ?compasscal (which now starts and ends pointing north) or by
    // ?magalign. Snapped to the four cardinal values: a 3.2 deg idle noise floor cannot justify
    // finer resolution, and nobody glues a module at 37 degrees.
    //   0 / 90 / 180 / 270
    // MIRRORING is NOT stored here - a mirrored sensor frame is stored as a NEGATIVE mag_scale_y,
    // because negating cal_y is exactly the mirror fix and that field already exists.
    uint16_t mag_orientation;          // 0 | 90 | 180 | 270 degrees


    // ============================================================

    // V2.5-Evo - 2026-08-16 - RESERVED SLOTS, banked deliberately.

    //

    // SW34 banked three of these and TWO have already paid for themselves without costing

    // anyone a config reset: fm_steer_reposition_en became log_level (2026-07-25), and

    // auton_runtime_cap_s became gps_dyn_model (2026-08-16). The pattern works, so since

    // SW35 is spending a reset anyway, bank more now rather than charge riders again for

    // the next small setting.

    //

    // RULES for reusing one - the same ones that made the last two free:

    //   1. RENAME IN PLACE. Same offset, same type. Never insert, never reorder.

    //   2. sizeof(confStruct) must not change, so the static_assert stays untouched.

    //   3. SW_VERSION must NOT be bumped - that is the whole point.

    //   4. 0 must remain a safe, behaviour-preserving default, because that is what every

    //      board in the field already holds here.

    //   5. Update the kCfgFields row and both web UIs in the same commit.

    //

    // WHAT THESE CANNOT DO: anything needing an ARRAY. Multi-TX needs a table of paired

    // addresses, not a scalar, so it will need a real struct change and its own bump. These

    // slots are for scalars - a threshold, a mode, an enable flag, a coefficient.

    //

    // Candidates already visible: magnetic declination (compass reads magnetic, GPS COG is

    // true-referenced); a motor-EMI compass compensation coefficient; the autonomous-runtime

    // cap that gps_dyn_model displaced; further GPS or FM tuning values.

    // ============================================================

    // V2.5-Evo - 2026-09-19 - THE FIRST OF THESE IS CLAIMED: rsvd_u16_1 is RENAMED IN PLACE to
    // steer_during_auto, by the five rules above. Same offset, same uint16_t, sizeof stays 200, the
    // static_assert is untouched, SW_VERSION stays 36, and 0 - what every fielded board holds here -
    // is the behaviour-preserving default: the stick CANCELS automatic steering exactly as before.
    //
    // steer_during_auto - what the rider's steering stick does while the buggy is steering itself
    //   (Follow-Me following, auto-return FM_RETURN, and a classic return-to-me):
    //     0 = CANCEL (default). A held push (40 counts from centre for 0.5 s, after the first 2 s
    //         of a run) ends the automatic steering: following drops to ARMED-unlatched, an
    //         auto-return stops through HOLD, and the remote exits a classic return-to-me. Exactly
    //         the behaviour before this setting existed - the three cancel paths run unchanged.
    //     1 = TAKE OVER. The same push makes the stick steer the buggy while it is held; centring it
    //         (within 30 counts for 0.2 s) hands the steering back to the controller and nothing is
    //         cancelled. The takeover changes only WHICH steering byte calcPWM() applies: every
    //         throttle cap, the stop radius, the return proof and every safety gate keep running.
    //         A stick that has not been read centred at least once since the run began cannot take
    //         over (a remote whose centre has drifted must never steer silently); a takeover held
    //         for 10 s ends through the mode's cancel path. See Common/SteerArbitration.h.
    //   Range 0-1; the remote learns the value from telemetry.fm_flags bit 4 (its Gate 4 cancel
    //   stays for 0). Turn on with `?set steer_during_auto 1` + `?save`.
    uint16_t steer_during_auto;        // 0 = stick cancels auto-steer (as before); 1 = stick takes over while deflected, resumes on centring. Range 0-1; default 0




    // V2.5-Evo - 2026-09-19 - THE SECOND SLOT IS CLAIMED TOO: rsvd_f32_1 is RENAMED IN PLACE to
    // auto_ramp_s, by the same five rules. Same offset, same float, sizeof stays 200, the
    // static_assert is untouched, SW_VERSION stays 36, and 0 - what every fielded board holds
    // here - is the behaviour-preserving default: inherit motor_ramp_s, exactly as before.
    //
    // auto_ramp_s - the motor rise-limit (seconds, 0 -> full) while an AUTOMATIC mode is capping
    //   the throttle: Follow-Me following, auto-return (FM_RETURN) motion, classic return-to-me.
    //   WHY TWO RAMPS (owner decision 2026-09-19 14:00): the slow motor_ramp_s exists for the
    //   rider's shoulder on a MANUAL tow start - the rope is loaded and a hard yank hurts. In the
    //   automatic modes the rope is not loaded, so a fast ramp is what makes the buggy reactive:
    //   it catches up fast and pivots fast. The rider's own trigger stays the only throttle
    //   source in both; a ramp only ever delays a rise, never adds throttle, and the fall stays
    //   instant.
    //     0         = inherit motor_ramp_s (default; today's behaviour, byte-identical).
    //     0.2 - 4.0 = the automatic modes' own ramp (the owner's suggestion: try 0.5).
    //   WHO RIDES WHICH (the selector in PWM.ino calcPWM(), on the OWNER flags rtm_rx_active ||
    //   fm_rx_active - deliberately not the takeover's auto_owner, which also carries the trigger
    //   and the stick switch): the ramp belongs to whoever is capping the throttle. Following,
    //   RETURN moving and classic RTM ride the auto ramp - a stick takeover included, since the
    //   throttle is still the automatic owner's. FM_ARMED-not-engaged towing, FM_STOPPING's hand-
    //   back, a HOLD escape, Gate 9 and the mode-0 cancel un-clamp all ride the MANUAL ramp, so
    //   every hand-back to the rider is softened by the slow slew (the shoulder case).
    //   RULE FOR P2 (stations), NOT BUILT: any automatic mode with the buggy AHEAD of the rider -
    //   front_along_m > 0, the fade-bypass predicate - must use motor_ramp_s, not auto_ramp_s: with
    //   the buggy in front the rope can load again, and that is the shoulder case in another guise.
    //   Bounds: kAutoRampMinS / kAutoRampMaxS above; clamped on load, never rejected.
    float    auto_ramp_s;              // 0 = same as motor_ramp_s; else 0.2-4.0 s rise time while an auto mode caps the throttle. Default 0

    // ============================================================
    // V2.5-Evo - 2026-09-19 - SW35->36: AUTO-RETURN INSIDE FOLLOW-ME (three fields, APPENDED)
    //
    // Appended at the END so every existing offset is unchanged and the SW35 blob is a byte-exact
    // prefix of this struct - that is what lets the boot-path migration keep every stored value
    // (pairing, compass calibration, every tuning number) instead of re-baking defaults. The two
    // reserved slots above are deliberately left alone: the owner accepted the version bump so the
    // slots stay banked for a future no-bump feature.
    //
    // fm_return_mode - the buggy's POWER-ON DEFAULT for auto-return. It is read only while the
    //   remote has not overridden it for the session (fm_return_mode_runtime == 0xFF; the remote's
    //   return gesture pushes an override in 0xF2 bits 5-6, RAM only, gone on a remote power cycle).
    //     0 = when the rider stops, Follow-Me HOLDs (cap 0) as it always has.
    //     1 = when the rider stops, Follow-Me graduates to FM_RETURN: the buggy proves that both it
    //         and the rider have genuinely stopped, then creeps back to the rider - ONLY while the
    //         trigger is held - and stops at rtm_stop_distance_m.
    // fm_align_cap - throttle cap (0-255 scale) while the buggy turns to face its target: the FM
    //   align phase and the FM_RETURN align / engage ramp. Replaces the compile-time kFmAlignCap
    //   (13) for the Follow-Me paths. 13 is about 5 %; raise it only after the bench spin has
    //   measured the yaw rate at the throttle-relative mixer.
    //   V2.5-Evo - 2026-10-04 - M-4: the VESC applies its OWN 3 % input deadband, which eats the
    //   first ~7.66 command counts, so below ~8 counts the motor does not turn AT ALL - every
    //   low-cap figure in these comments overstates the thrust actually delivered.
    // fm_align_influence - the steering influence (percent) handed to the differential mixer during
    //   those same align phases ONLY, in place of steering_influence. 100 = one-motor pivot (at cap
    //   13: motors 0 / 26). 0 = no boost, use steering_influence. Ordinary following and manual
    //   driving always use steering_influence.
    // ============================================================
    uint16_t fm_return_mode;           // 0 = HOLD when the rider stops (as before); 1 = FM_RETURN. Range 0-1; default 1
    uint16_t fm_align_cap;             // throttle cap during FM align / FM_RETURN align+ramp, 0-255 scale; range 8-80; default 13
    uint16_t fm_align_influence;       // mixer steering influence during FM align / FM_RETURN align, %; range 0-100 (0 = use steering_influence); default 80

    // ============================================================
    // V2.5-Evo - 2026-10-02 - P2 (FRONT STATIONS): fm_front_angle_deg TAKES THE 2 TAIL PADDING
    // BYTES. NO SW_VERSION BUMP, NO CONFIG WIPE.
    //
    // WHY IT IS HERE AND NOT A FLOAT. The P2 plan allocated this field to the banked RESERVED slot
    // rsvd_f32_1, as a float with one decimal place. THAT SLOT IS GONE: it was renamed in place to
    // auto_ramp_s on 2026-09-19, and rsvd_u16_1 went to steer_during_auto the same day, so both
    // banked slots are spent. The remaining free space in this struct is the 2 bytes of tail padding
    // the SW36 append left behind (198 used, sizeof 200 for the 4-byte float alignment), which is
    // exactly one uint16_t. So this field is a uint16_t in WHOLE DEGREES, and sizeof(confStruct)
    // stays 200, the static_assert below is untouched, SW_VERSION stays 36, and the owner's stored
    // config is NOT wiped by this flash. It is the same trick mag_fm_set used on the TX (2026-09-30)
    // and bt_enabled before it.
    //
    // WHAT WHOLE DEGREES COST: nothing that matters. At the 18.4 m front radius, 1 degree of station
    // angle is 0.32 m of position - far inside the 5 m relative GPS error the 13 m lateral invariant
    // is built on top of. A tenths-of-a-degree encoding was considered and rejected: a rider typing
    // `?set fm_front_angle_deg 45` would be asking for 4.5 degrees, which the clamp would silently
    // turn into 35, and a setting that surprises the person reading it back is worse than one with
    // coarser resolution.
    //
    // fm_front_angle_deg - the angle of the FRONT stations (F4 front-right, F5 front-left) off dead
    //   ahead, in degrees, measured at the rider.
    //     0     = use kFmFrontAngleDefaultDeg (45). THIS IS WHAT EVERY BOARD IN THE FIELD READS
    //             HERE TODAY (the padding bytes of a stored blob are zero in practice), so 0 must
    //             mean the shipped default - rule 4 of the reserved-slot block above.
    //     35-80 = the angle itself. 35 is the owner's settable minimum (his override of the code
    //             review's 40 deg recommendation), accepted because the clearance is kept by the
    //             RADIUS instead of by the angle: see the invariant below.
    //   THE INVARIANT, ENFORCED BOTH WAYS AT THE READ SITE (not only in the validator):
    //     r_front = max(kFmFrontRadiusFactor x d_follow, kFmFrontLateralMinM / sin(phi))
    //     phi_eff = max(phi, asin(kFmFrontLateralMinM / r_front))
    //   so r_front x sin(phi_eff) >= kFmFrontLateralMinM (13 m) at every setting. A SMALLER ANGLE
    //   MAKES THE FRONT STATION FURTHER AWAY (35 deg -> 22.7 m, about 75 ft), NEVER CLOSER TO THE
    //   RIDER'S LINE. A rider who tightens d_follow does not shrink the clearance either.
    //   AND THERE IS NO DEAD-AHEAD STATION AT ANY VALUE: |station| is clamped at 180 - phi_eff, so
    //   the whole arc within phi_eff of dead ahead is unreachable. A buggy directly ahead must
    //   accelerate as the rider closes on it, and a failed motor stops it ON the rider's line - the
    //   owner has already hit this buggy once, broke the propeller and limped home. Off axis, a dead
    //   motor leaves the buggy beside the line, and the trailing rope stays off the line too.
    //   Validator: 0, or 35-80; anything in (0, 35) is CLAMPED UP to 35 and says so, never rejected
    //   (a range rejection on the LOAD path wipes the whole config - the 2026-09-03 lesson).
    // ============================================================
    // ============================================================
    // V2.5-Evo - 2026-10-06 - RENAMED IN PLACE: fm_front_angle_deg -> fm_front_ahead_extra_m.
    // EVERYTHING ABOVE ABOUT THE ANGLE IS HISTORY (kept as the record of why these 2 bytes exist).
    // Owner rule: the front stations are placed by SIDE + AHEAD offsets, not angle + radius, so the
    // angle is now derived and this field became obsolete. It is reused, not replaced: same offset,
    // same uint16_t, sizeof STAYS 200, SW_VERSION STAYS 36, the stored config is NOT wiped.
    //
    // fm_front_ahead_extra_m - how much further AHEAD of the rider a front station (F4/F5) sits,
    //   beyond the follow distance, in WHOLE METRES. ahead = d_follow + this. Sideways is NOT set
    //   here: it is the lateral floor exactly (kFmFrontLateralMinM 13 m, or the pass minimum if
    //   min_dist_m makes that larger) and never more.
    //     0    = use kFmFrontAheadExtraDefaultM (7). What every fielded board reads in these bytes
    //            (SW36 without the field: padding = 0), so the default is what they get.
    //     4-10 = the extra itself. Example: d_follow 9 + 7 = 16 m ahead, 13 m side -> 39 deg, 20.6 m.
    //   Clamped on load (cfgValidateCrossField), never rejected: 1-3 -> 4; above 10 -> 0 (default),
    //   because the only source of such a value is a blob that stored an ANGLE here (35-80).
    //   The derived angle is held at 35-45 deg off dead ahead (45 since audit M-12; this line said 35-80 until
    //   audit L-19) by moving `ahead` (side does not move,
    //   except that a follow distance over 22.7 m - longer than the capped radius - pushes the
    //   station out to d_follow at 35 deg, wider than 13 m: the safe direction),
    //   so dead ahead stays unreachable: |station| <= 180 - the derived angle.
    // ============================================================
    uint16_t fm_front_ahead_extra_m;   // metres ahead of d_follow for F4/F5; 0 = default 7; range 4-10. Default 0 (was fm_front_angle_deg, renamed in place 2026-10-06)
    // (the 2 bytes of tail padding are now spent: 200 used, sizeof 200, nothing left for free.)
};
static_assert(sizeof(confStruct) == 200, "confStruct size mismatch — expected 200 bytes (SW36: 192 + fm_return_mode/fm_align_cap/fm_align_influence 3 x u16 = 198, + fm_front_ahead_extra_m (u16, was fm_front_angle_deg) in the 2 B tail pad = 200, no padding left). Update this assert if you change the struct.");  // 2026-10-06: fm_front_angle_deg renamed in place to fm_front_ahead_extra_m, sizeof unchanged.  // 2026-10-02 P2: +fm_front_angle_deg(u16 2) took the 2 B TAIL PAD, so sizeof STAYS 200, SW_VERSION STAYS 36 and no config is wiped — the mag_fm_set trick (TX, 2026-09-30). The tail is now FULL: the next scalar is a real SW37 bump. // 192->200 SW35->36: +fm_return_mode(u16 2) +fm_align_cap(u16 2) +fm_align_influence(u16 2) appended at the tail, +2 B tail pad (2026-09-19). The SW35->36 boot-path migration in Common/SPIFFSEngine.h is pinned to 192 (legacy) and 200 (current) and disables itself if either stops matching.
// HISTORY of the size assert as it stood until 2026-09-19 (192 bytes), kept verbatim: 176->184: +fm_engage_dist_m(float 4) +auton_runtime_cap_s(u16 2) +fm_steer_reposition_en(u16 2), all naturally aligned, no tail pad (2026-07-20 SW34)  // 172->176 motor_ramp_s float (2026-06-05 SW33)  // 112->128 Phase A; 128->136 Phase B; 136->152 P7 RTM; 152->156 Bundle B; 156 unchanged BundleE; 156->160 rtm_approach_zone_m (uint16_t + 2-byte tail pad) (2026-04-30); D3 rtm_use_compass + rtm_cog_min_speed_kmh (2x uint8_t) fill the 2-byte tail pad — sizeof stays 160 (2026-05-06); D3-Fix: uint8_t→uint16_t for ConfigService compatibility, sizeof unchanged at 164 (2026-05-06); Bundle 1: dummy_delete_me renamed to rtm_steer_response in-place, sizeof unchanged at 164 (2026-05-08); STAGE 0 PART A: fm_steer_reposition_en renamed to log_level in-place — same offset, same uint16_t, sizeof STILL 184 and SW_VERSION STILL 34, so this flash does NOT reset SPIFFS config (2026-07-25); auton_runtime_cap_s renamed to gps_dyn_model in-place, sizeof STILL 184, SW_VERSION STILL 34 (2026-08-16); 184->192 SW34->35: +mag_orientation(u16 2) +rsvd_u16_1(u16 2) +rsvd_f32_1(float 4), appended at the tail, naturally aligned, no tail pad — the one intended config wipe for this bump (2026-08-16). THIS NUMBER IS THE SSOT: the SW34->35 config-backup migration is pinned to 184 (legacy) and 192 (current) and disables itself if either stops matching, so any prose elsewhere that disagrees with the 192 above is stale and must be corrected rather than trusted.
confStruct usrConf;
  //The orginal confs were:  ##// confStruct defaultConf = {SW_VERSION, 1, 0, 0, 50, 0, 0, 1500, 2000, 1500, 2000, 1000, 10, 0, 1, 0, 0, 0, 0, 0, 25.0f, 10.0f, 10.0f, 5.0f, 35.0f, 45.0f, 45.0f, 0.0095554f, 0.0, 1000, 1, 0, {0, 0, 0}, {0, 0, 0}, {'1','2','3','4','5','6','7','8'}};
  // Factory default configuration.
confStruct defaultConf = {SW_VERSION, 2, 22, 1, 50 /*steering_influence: conventional default (0-100)*/, 0 /*steering_inverted: 0 = conventional default; a fresh build MUST verify steering direction wheels-up (FM steers toward rider) before trusting FM.*/, 0, 1000, 2000, 1000, 2000, 1000, 10, 0, 1, 2, 2, 1, 2, 1, 25.0f, 10.0f, 10.0f, 8.0f, 35.0f, 45.0f, 45.0f, 0.0095554f, 0.0f, 3000, 0, 0, {0, 0, 0}, {0, 0, 0}, {'1','2','3','4','5','6','7','8'}, // wifi_password below: documented DEFAULT AP password "12345678" — change before use
  // V2.5-Evo - 2026-04-22 - Compass calibration fields (previously implicit zeros).
  // Made explicit here so gps_chip_type can follow. Safe neutral values:
  // offsets=0 (no bias), scales=1.0f (unity gain = no correction applied).
  0, 0,   // mag_offset_x, mag_offset_y (neutral zero bias; re-derived via 'runcal')
  1.0f, 1.0f, // mag_scale_x, mag_scale_y (unity gain = no correction until calibrated)
  // V2.5-Evo - 2026-04-22 - GPS chip type: 1 = BN-880 (GPS+compass). RX default.
  1,          // gps_chip_type (1 = BN-880 + compass; run 'runcal' after first boot)
  // V2.5-Evo - 2026-04-22 - Phase A GPS anti-spoofing defaults
  2.0f,       // gps_max_hdop:           max HDOP for valid reading (range 0.5-5.0)
  3.0f,       // gps_max_accel_g:        max implied acceleration (range 1.0-10.0 G)
  80.0f,      // gps_max_teleport_kmh:       max teleport-implied speed (range 50-500 km/h; default lowered 200→80 2026-04-30)
  3,          // gps_suspect_threshold:  consecutive failures before GPS rejected (range 1-10)
  // V2.5-Evo - 2026-04-24 - Phase B GPS handshake anti-spoofing defaults
  500.0f,     // gps_max_pair_dist_m:    max TX-RX pairing distance (range 50-2000 m)
  50.0f,      // gps_max_speed_diff_kmh: max TX-RX speed difference (range 10-200 km/h)
  // V2.5-Evo - 2026-04-25 - Priority 7 RTM Phase C + RX safety defaults
  20.0f,      // rtm_vesc_speed_diff_kmh: max GPS vs VESC speed diff (5-50 km/h)
  0.0f,       // vesc_erpm_per_kmh: 0 = skip Phase C VESC check until calibrated
  1,          // rtm_rx_enabled: 1 = RTM enabled on RX side
  1,          // rtm_rx_override_steering: 1 = RTM may override steering
  1,          // rtm_compass_required: 1 = compass required for RTM arming
  // V2.5-Evo - 2026-04-26 - CRITICAL FIX: rtm_stop_distance_m was missing from defaultConf; zero-init
  // would have set it to 0, making Gate 9 check (dist_m < 0.0f) never fire — permanently
  // disabling the hard stop that prevents the buggy from hitting the user.
  10,  // rtm_stop_distance_m: safe default 10 m (>= 8 m GPS floor); RTM hard-stop radius
  // V2.5-Evo - 2026-04-29 - Bundle B: vesc_timeout_s replaces hardcoded 20s VESC connection timeout
  // V2.5-Evo - 2026-08-17 - raised 6 -> 10 s. A VESC cold restart takes roughly 8-9 s, so at 6 s
  // the rider's battery % and FET temperature blanked to N/A on the TX across every restart even
  // though the VESC was only booting. 10 s covers it and is still half the original hardcoded 20 s.
  // VALUE-ONLY change: no field added, moved or resized, and the 5-60 range already in kCfgFields is
  // unchanged, so sizeof(confStruct) stays 192, SW_VERSION stays 35 and SPIFFS config is NOT reset.
  // NOTE: defaultConf only applies to units whose config is reset. A board with a STORED
  // vesc_timeout_s keeps its old value (6) after this flash — set it explicitly on the device with
  // `?set vesc_timeout_s 10` then `?save`.
  10,         // vesc_timeout_s: seconds without VESC UART packet before bat/temp shown as N/A (range 5-60s; default 10s)
  // V2.5-Evo - 2026-04-30 - Bundle E: gps_update_hz replaces hardcoded 1Hz GPS poll cadence
  // V2.5-Evo - 2026-07-25 - STAGE 1: default raised 2 -> 10 Hz. The GPS module is configured for
  // 5 Hz (BN-220/BN-880) or 10 Hz (M10), so draining twice a second left ~1250 bytes pending per
  // drain now that STAGE 1 lets the module stream continuously into the ring. 10 Hz drains every
  // ~100 ms, keeping the ring occupancy far below its 2048-byte capacity at all times.
  // VALUE-ONLY change: no field added, moved or resized; the 1-10 range already in kCfgFields is
  // unchanged, sizeof(confStruct) stays 184, SW_VERSION stays 34, SPIFFS config is NOT reset.
  // NOTE: defaultConf only applies to units whose config is reset. A board with a STORED
  // gps_update_hz keeps its old value (2) after this flash — set it explicitly on the device.
  10,         // gps_update_hz: GPS NMEA polling rate in Hz (range 1-10 Hz; default 10 Hz = 100ms interval)
  // V2.5-Evo - 2026-04-30 - RTM approach decel zone default
  12,         // rtm_approach_zone_m: outer edge of RTM throttle decel zone (0=disabled, 5-100 m)
  // V2.5-Evo - 2026-05-06 - D3: RTM heading source selection defaults
  1,          // rtm_use_compass: 1 = Hybrid (GPS COG primary, compass snapshot at low speed). 0=COG only, 2=compass only DIAGNOSTIC.
  3,          // rtm_cog_min_speed_kmh: GPS speed threshold below which compass snapshot is used; 1-15 km/h; default 3
  // V2.5-Evo - 2026-05-22 - SW32: Two-phase RTM throttle defaults
  4.0f,       // rtm_target_speed_kmh: Phase 2 GPS speed cap; 4 km/h default; 0=disabled
  45,         // rtm_align_threshold_deg: heading error threshold for Phase 1→2 transition; 45° default
  // V2.5-Evo - 2026-06-05 - SW33: motor ramping (secs) default
  0.75f,      // motor_ramp_s: THROTTLE ramps 0->full over 0.75s (0=instant/off, 0-4s); steering is never ramped
  // V2.5-Evo - 2026-07-20 - SW34 slots. 2026-07-25 A2: fm_engage_dist_m is now live; the other two stay reserved/unread.
  0.0f,       // fm_engage_dist_m: 0 = auto (RTMState computes d_engage from min_dist + band); >0 = fixed engage distance in metres
  0,          // gps_dyn_model: 0 = default -> Sea (was auton_runtime_cap_s, renamed in place 2026-08-16)

  // V2.5-Evo - 2026-07-25 - STAGE 0 PART A: this slot was fm_steer_reposition_en, renamed in place
  // to log_level. The default stays 0 on purpose — 0 means "unset" and behaves exactly as level 3
  // (Developer), which is the behaviour every unit already has, so nothing changes on flash.
  0,          // log_level: 0 = unset -> logs as level 3 (Developer). 1/2 accepted but currently log as 3; 4 = Deep.
  0,          // mag_orientation: 0 deg. Set by ?compasscal (north-to-north) or ?magalign.
  0,            // steer_during_auto: 0 = the stick CANCELS automatic steering (the tested behaviour); 1 = it takes over while deflected (was rsvd_u16_1, renamed in place 2026-09-19)


  0.0f,         // auto_ramp_s: 0 = the automatic modes ride motor_ramp_s (today's behaviour); 0.2-4.0 = their own, faster ramp (was rsvd_f32_1, renamed in place 2026-09-19)
  // V2.5-Evo - 2026-09-19 - SW36 auto-return defaults. These are also the values the boot-path
  // migration writes into a migrated SW35 config (the tail of the struct is copied from HERE).
  1,            // fm_return_mode: 1 = auto-return ON (owner decision, session log item 32); 0 = HOLD as before
  13,           // fm_align_cap: ~5 % throttle while turning to face the target (same number kFmAlignCap held)
  80,           // fm_align_influence: 80 % steering split during the align phases (owner decision 2026-09-19; 100 = one-motor pivot, 0 = use steering_influence)
  // V2.5-Evo - 2026-10-02 - P2. The default STAYS 0 on purpose and that is load-bearing: 0 means
  // "use kFmFrontAngleDefaultDeg (45)", and 0 is what every board in the field already reads out of
  // the tail padding this field now occupies. So a flash changes nothing until the rider sets it.
  // V2.5-Evo - 2026-10-06 - the field is now fm_front_ahead_extra_m; 0 still means "the default"
  // (7 m), for the same load-bearing reason.
  0             // fm_front_ahead_extra_m: 0 = default 7 m ahead of d_follow for F4/F5; else 4-10 m
};

// ============================================================
// V2.5-Evo - 2026-07-25 - F3-b: FM ENGAGE-DISTANCE FLOOR (one shared definition)
//
// Hard floor for a MANUAL fm_engage_dist_m override, in metres. 0 (auto) bypasses it entirely,
// because auto derives the engage distance from min_dist_m + followme_smoothing_band_m instead.
//
// WHAT THIS NUMBER GUARDS: since A2, fm_engage_dist_m IS the Follow-Me engage distance in metres —
// how far the rider must be from the buggy before FM may engage for the first time. The whole point
// of the separation latch is that the rider must be OFF the tow rope before FM engages.
//
// WHAT THE BUG WAS: the kCfgFields row for the field only range-checks 0-50 m, so a small value such
// as 3 m was accepted and stored. An engage distance shorter than the rope does not tune the
// interlock, it DEFEATS it — FM engages with the rider still on the rope, i.e. autonomous steering
// mid-tow. The first fix (F3) set this floor to 5.0 m, which was still BELOW the hazard it named:
// the owner's tow rope was then taken as 20 ft = 6.10 m, so 5.0-6.1 m was still storable and still
// on-rope. 8.0 m cleared a 6.10 m rope by ~1.31x, and it also sits above the factory follow geometry
// (min_dist 4 m + smoothing band 2 m = 6 m), so the interlock is a real gate rather than a no-op.
//
// V2.5-Evo - 2026-09-18 - RAISED 8.0 -> 9.5 m (code-review finding F3). The rope actually in use is
// 7.1 m (owner, 2026-08-28), not the 6.10 m the 8.0 m figure was derived from: 8.0 m cleared it by
// only 0.9 m, inside ordinary GPS error, so the floor no longer did the job its own comment claims.
// Same derivation, right rope: 7.1 x 1.31 = 9.3 m, rounded up to 9.5 m. The owner's manual 12 m is
// unaffected. Knock-ons, all in the conservative direction: the factory engage edge becomes
// max(4 + 2, 9.5) = 9.5 m; the owner's short-release edge becomes max(5 + 4, 9.5) = 9.5 m (was 9);
// the auto D_engage at factory tuning clamps 9.0 -> 9.5 m; a stored fm_engage_dist_m of 8.0-9.4 m
// now clamps up to 9.5 m at the read site; and the heading-blind RTM bootstrap abort radius in
// runRtmLoop() (max(min_dist_m, this)) grows 8 -> 9.5 m.
//
// WHERE IT IS USED — the same definition everywhere; there is no second copy:
//   1. cfgValidateCrossField() in ConfigService.ino — refuses to STORE anything in (0, 9.5) m.
//   2. runFmLoop() in RTMState.ino — defensive clamp UP to this floor for a value already sitting in
//      SPIFFS from before the rule existed (a stored config is never re-validated when it is loaded),
//      and the trigger-free engage floor (Common/FollowMeEngage.h, since 2026-09-18).
//   3. runRtmLoop() in RTMState.ino — the BOOTSTRAP-1 abort radius floor.
// All are .ino files, and this header is included at the top of V2_Integration_Rx.ino, which the
// Arduino build concatenates first — so the constant is visible to all. (An earlier comment claimed
// concatenation order prevented sharing and used that to justify a duplicated literal. It was wrong.)
// ============================================================
static const float kFmEngageDistFloorM = 9.5f;   // metres; smallest legal non-zero fm_engage_dist_m (7.1 m rope x 1.31, rounded up)

// ============================================================
// V2.5-Evo - 2026-10-02 - P2: THE FRONT-STATION ANGLE BOUNDS AND THE LATERAL MINIMUM
//
// These four live HERE, once, for exactly the reason kFmEngageDistFloorM does: TWO FILES READ THEM.
// cfgValidateCrossField() in ConfigService.ino needs the bounds to clamp a stored
// fm_front_angle_deg, and the read site in RTMState.ino needs the same numbers to resolve the angle
// and the radius. [V2.5-Evo - 2026-10-06 - audit L-13: since the offset change the validator clamps
// fm_front_ahead_extra_m with the kFmFrontAheadExtra* values instead, and the read sites derive the
// angle and radius from side + ahead.] ConfigService.ino is concatenated BEFORE RTMState.ino, so a constant defined in
// RTMState.ino would be invisible to the validator; this header is included at the top of
// V2_Integration_Rx.ino, which is concatenated first, so both see these. The rest of the P2 tuning
// (the slew rate, the radius factor, the transit constants) is read only by RTMState.ino and lives
// there with the other kFm* values.
//
// kFmFrontLateralMinM IS THE SAFETY NUMBER OF THIS WHOLE FEATURE. 13 m = an 8 m carve radius plus
// 5 m of TX-vs-RX relative GPS error (code review B2). It is the minimum distance a front station is
// allowed to sit from the rider's course line. [V2.5-Evo - 2026-10-06 - audit L-13: it is no longer
// honoured "by growing the radius against the rider's angle" - there is no angle setting now; the
// front station sits exactly this far to the side and the angle and radius are derived. See the
// FRONT STATIONS BY OFFSET note below.] The 5 m term is an assumption, not a measurement: measurement #5 in the plan (both boards
// static at the dock, 5 min of fm_distance_dx10) is what pins it, and the floor moves roughly
// 4 deg per metre of error, so this number must be revisited once that measurement exists.
//
// THE MINIMUM ANGLE IS 35, WHICH IS AN OWNER OVERRIDE ON THE RECORD. The code review recommended a
// 40 deg minimum derived at a 20 m radius. The owner overrode it to 35 and kept the 13 m invariant,
// which is self-consistent: at 35 deg the radius has to be 13/sin35 = 22.7 m (about 75 ft), and the
// clearance is identical. Recorded as an override, not as a rejection of the finding.
// ============================================================
// V2.5-Evo - 2026-10-06 - FRONT STATIONS BY OFFSET. The angle above is no longer a setting: F4/F5
// sit kFmFrontLateralMinM (or the pass minimum, if larger) EXACTLY to the side and d_follow +
// fm_front_ahead_extra_m ahead, and the angle is derived (FollowMeStation.h fmFrontStationGeom).
// kFmFrontAngleMinDeg / MaxDeg now bound that DERIVED angle, by capping `ahead` - side does not move
// (except past d_follow 22.7 m, where the radius is d_follow and side grows - see fmFrontStationGeom).
// At side 13 m the 35 deg minimum caps ahead at 18.6 m. kFmFrontAngleDefaultDeg is gone (nothing
// means "45 deg" any more); the three kFmFrontAheadExtra* values replace it. Read by both the
// validator (ConfigService.ino) and the read sites (RTMState.ino), hence here.
static const float kFmFrontAngleMinDeg       = 35.0f;   // degrees; floor on the DERIVED front angle (owner's 35, override of the review's 40)
// V2.5-Evo - 2026-10-06 - audit M-12: kFmFrontAngleMaxDeg 80 -> 45, so `ahead` is never less than `side`
// (13 m at side 13). WHAT WAS WRONG: at 80 the floor on ahead was side / tan80 = 2.3 m, so d_follow 6 +
// extra 4 put the station only 10 m ahead. The 13 m side floor is a LATERAL argument: a rider carving
// 90 deg toward the station on an 8 m radius also travels about 8 m FORWARD, and against a stopped buggy
// the clearance is about sqrt((ahead - 8)^2 + 5^2) - 5.4 m at 10 m ahead, 7.1 m at 13, 9.4 m at 16. With
// 45 the derived angle is 35-45 deg; d_follow 6 + 4 -> 13 m ahead x 13 m side at 45 deg.
static const float kFmFrontAngleMaxDeg       = 45.0f;   // degrees; ceiling on the DERIVED angle - ahead >= side, never closer ahead than to the side
static const float kFmFrontLateralMinM       = 13.0f;   // metres; carve 8 m + relative GPS 5 m. The front station's side offset IS this (or the pass minimum if larger), exactly
static const float kFmFrontAheadExtraDefaultM = 7.0f;   // metres; what fm_front_ahead_extra_m == 0 means
static const float kFmFrontAheadExtraMinM     = 4.0f;   // metres; smallest legal extra (1-3 are raised to this)
static const float kFmFrontAheadExtraMaxM     = 10.0f;  // metres; largest legal extra (above it reads as the default)

// V2.5-Evo - 2026-10-02 - P2: the station arithmetic (presets, the hard floor, the radius schedule
// and its pass-lateral floor, the rider-frame coordinates, the slew, the pass-geometry predicates
// PG-1..PG-4 and the fade-bypass predicate) is pure in ../Common/FollowMeStation.h so
// Tools/tests/follow_me_station_test.cpp runs the exact code computeFmTarget() and
// fmComputeThrottleCap() call. Every constant is passed in; the header defines none of its own.
#include "../Common/FollowMeStation.h"

// V2.5-Evo - 2026-09-18 - P1-a: the trigger-free engage floor and the needs-D_engage rule are pure
// functions in ../Common/FollowMeEngage.h so Tools/tests/follow_me_engage_test.cpp runs the exact
// code runFmLoop() calls. The floor constant above is passed in as an argument; the header defines
// no constants of its own.
#include "../Common/FollowMeEngage.h"
// V2.5-Evo - 2026-09-19 - P1-b: the FM_RETURN entry proof (candidate on raw rider speed, cap-zero
// dwell, buggy-stopped gate, relative-displacement window) is pure in ../Common/FollowMeReturnProof.h
// so Tools/tests/follow_me_return_proof_test.cpp runs the exact code runFmLoop() calls. The RX's
// kFmReturn* constants (RTMState.ino) are passed in; the header defines none of its own.
#include "../Common/FollowMeReturnProof.h"
// V2.5-Evo - 2026-09-19 - the stick-takeover arbitration (centre-seen, 40/500 ms engage, 20/200 ms
// release, 10 s timeout, zeroed with no owner) is pure in ../Common/SteerArbitration.h so
// Tools/tests/steer_arbitration_test.cpp runs the exact code the three auto-steer modes call. The
// RX's kSteerTakeover* constants (RTMState.ino) are passed in; the header defines none of its own.
#include "../Common/SteerArbitration.h"

// ============================================================
// V2.5-Evo - 2026-07-25 - STAGE 2: HEADING-SOURCE TRUST CONSTANTS (RTM + FM)
//
// WHY THESE EXIST — MEASURED ON THE BENCH, NOT THEORISED
//   ?diag on the repaired board reported:
//       COG : 7.4 timestamp-updates/s vs 0.0 value-changes/s   [value frozen 533 s]
//   The GPS module pushed a course update about seven times a second for nine minutes, and every
//   single one carried THE SAME NUMBER. getRtmHeading()'s freshness test is
//   (millis() - gps_last_course_ms) < 1500 ms, and that timestamp was being refreshed by every one
//   of those repeats — so the heading ladder saw a perfectly fresh, HIGH-confidence COG the whole
//   time. It was not fresh. It was one heading, repeated. That is the failure that made Follow-Me
//   steer the wrong way on the water: COG looked alive, was dead, and when it finally dropped out
//   the ladder handed steering to a compass this hardware is known to bias by 100 deg+ under motor
//   current. Two independent guards, both consuming these constants, close that hole.
//
// WHY THEY LIVE IN THIS HEADER AND NOT IN RTMState.ino
//   Exactly the kFmEngageDistFloorM story (see the block above). getRtmHeading() in RTMState.ino
//   and its inline duplicate in Logger.ino (convertToLogData, the block marked CRITICAL
//   MAINTENANCE) must apply the SAME rule or the log will report a heading source the controller
//   did not actually use. The Arduino build concatenates .ino files alphabetically after the main
//   sketch, so Logger.ino is compiled BEFORE RTMState.ino: a constant defined in RTMState.ino is
//   invisible to the logger mirror. This header is included at the top of V2_Integration_Rx.ino,
//   so one definition here is visible to both.
//
// NONE of these four is a confStruct field. They are deliberately compile-time: no SPIFFS slot,
// no web-UI row, no SW_VERSION bump, no config wipe.
// ============================================================

// GUARD 1 — how long the COG VALUE may sit still, WHILE THE BUGGY IS MOVING, before COG stops
// counting as a trustworthy heading source.
// WHY 3000 ms. The module emits course several times a second, and real course-over-ground is
// noisy: at the default rtm_cog_min_speed_kmh of 3 km/h, GPS course scatter is whole degrees, and
// the value-change tracker in GPS.ino counts any movement above kDiagCogChangeDeg = 0.05 deg. A
// genuinely live COG therefore re-stamps g_diag_cog_change_ms many times per second, and three
// full seconds of a bit-identical course while moving is not a quiet trajectory, it is a frozen
// register. The measured fault held one value for 533 s, so 3 s catches it ~178x over while
// leaving a wide margin against a false positive. It is also short enough to matter: at the FM
// speed governor's ceiling the buggy covers only a few metres in 3 s.
static const uint32_t kRtmCogFrozenMs       = 3000;   // ms of unchanged COG value (while moving) = COG not trusted

// GUARD 2 — how far apart the two independent heading estimates may be before BOTH are distrusted.
// WHY 45 deg. Below this, disagreement is explainable by things that are not faults: compass
// mounting offset, magnetic declination, the yaw the buggy accumulates between the compass
// snapshot and the live COG sample, and ordinary COG scatter at low speed. Beyond 45 deg the two
// sensors are no longer describing the same vehicle attitude, and at 45 deg of heading error the
// steering controller is already commanding half of full authority in a direction one of the two
// sources says is wrong. It is a "these cannot both be right" threshold, not a tuning knob.
static const float    kHeadingDisagreeDeg   = 45.0f;  // degrees, shortest angular distance

// GUARD 2 — how long that disagreement must persist before it is treated as a genuine FAULT rather
// than a transient. WHY 5000 ms. A single bad compass sample, one COG glitch or one hard carve can
// open a >45 deg gap for a moment; a broken sensor holds it. At the 10 Hz ladder cadence 5 s is ~50
// consecutive confirmations, the same "a spike cannot sustain it" argument kFmSepDwellMs and
// kFmDivergeMs are built on. It is deliberately LONGER than kRtmCogFrozenMs: guard 1 has hard
// evidence (a register that stopped moving) and may act fast; guard 2 only knows that one of two
// sources is lying, so it waits for proof before ending the run.
static const uint32_t kHeadingDisagreeMs    = 5000;   // ms of sustained disagreement = FAULT

// GUARD 2 — the oldest compass snapshot that may still be compared against a live COG.
// WHY THIS IS NEEDED AND WHY IT IS 1000 ms. compass_snapshot_heading is a HELD value: Compass.ino
// only re-captures it while the motor is idle (thr_received < 25), so during an FM/RTM run, with
// the trigger held, it freezes at the instant of the squeeze and simply ages. Comparing a live COG
// against a heading captured seconds ago measures how much the buggy has TURNED since, not whether
// the sensors agree — at a modest 10 deg/s yaw a 5 s-old snapshot is 50 deg out all by itself and
// would fake a disagreement every corner. 1000 ms is the same window getRtmHeading() already
// requires before it will call a snapshot MEDIUM confidence, i.e. the only compass data this
// firmware already treats as simultaneous with now. Outside it, guard 2 simply does not run —
// no comparison is better than a comparison of two different moments in time.
// V2.5-Evo - 2026-08-16 - How long a COG stays usable after cog_valid goes false. Bridges the
// noise-driven flicker between GPS course and compass that RTM's 4.0 km/h target creates
// against a 3 km/h COG floor. 3 s is ~3.3 m of travel at that speed - short enough that the
// held course is still true, long enough to cover the dips that caused the flapping.
static const uint32_t kCogHoldMs           = 3000;   // ms; hold last-good COG across a dropout
static const uint32_t kHeadingCompareSnapMs = 1000;   // ms; max compass-snapshot age for a valid comparison

// ============================================================
// V2.5-Evo - 2026-09-25 - MANUAL PIVOT ASSIST - THE TUNED CONSTANTS
//
// WHAT THE RIDER ASKED FOR, IN HIS WORDS
//   "should be natural but for when starting from [low] speed to pivot fast. There are many
//    occasions where the buggy's nose is misaligned and you want to turn in the right direction
//    before you take off. A fast pivot will aid enormously in getting it right right away."
//   He already does this by hand: on the water he found that easing the trigger back to roughly
//   half while holding the stick over tightened the U-turn, and that at full throttle the arc was
//   too wide. steering_influence is already at its maximum of 100, so no stored setting can
//   reproduce it. The assist does that same hand action automatically, and ONLY that: it lowers
//   the throttle that reaches the differential mixer while the buggy is slow and the stick is
//   hard over. It is SUBTRACT-ONLY (see PWM.ino) and it never touches a cap, the ramp's memory,
//   the deadman or the SAFETY-NEUTRAL-1 clamp.
//
// WHAT THE ARITHMETIC ACTUALLY SAYS - READ THIS BEFORE RETUNING kPivotFloorQ8
//   The mixer (Common/DifferentialMixer.h) is turn = T x influence x |steer-127| / (100 x span),
//   motor0 = T - turn, motor1 = T + turn, each clamped 0..255. At the owner's influence of 100:
//     FULL LOCK  (|steer-127| = span): turn == T, so the pair is (0, 2T) clamped -> (0, 255) for
//       ANY T from 128 to 255. T = 255 gives (0, 255); T = 127 gives (0, 254). Those are the SAME
//       pair, and that is the whole reason this constant needs a derivation instead of a guess.
//
//       THE MECHANISM: AT FULL LOCK THE MIXER'S GAIN INTO THE OUTER MOTOR IS EXACTLY 2x. The inner
//       motor is already 0, so with a throttle-domain floor f the outer motor - and therefore the
//       ENTIRE thrust the craft produces - lands at exactly
//                                  min(2 x f x T, 255).
//       Read that formula once and the trap is obvious: f = 1/2 is the EXACT RECIPROCAL of the
//       mixer's gain, so it cancels the gain and puts the outer motor at exactly T. For every
//       T >= 128 the un-assisted outer motor was already clamped at 255, so at f = 1/2 the assist's
//       entire effect is to bring 255 down to T - a cut of 255 - T, which is ZERO at full trigger
//       and only grows as the rider eases off. f = 1/2 is therefore the single WORST value this
//       constant can take, and it is worst precisely in the owner's use case: trigger pinned, stick
//       hard over, nose misaligned. It was the shipped value for a few hours on 2026-09-25 and is
//       now 64 (f = 1/4). THE TABLE, outer motor at FULL TRIGGER (T = 255), as the code computes it
//       (Q8 multiply, truncating):
//                kPivotFloorQ8 = 128  (f = 0.500) -> 254   =  0 % cut   <- annihilated
//                kPivotFloorQ8 =  96  (f = 0.375) -> 190   = 25 % cut
//                kPivotFloorQ8 =  64  (f = 0.250) -> 126   = 51 % cut   <- SHIPPED
//                kPivotFloorQ8 =  51  (f = 0.199) -> 100   = 61 % cut   <- too deep, see below
//
//       WHY TOTAL THRUST IS THE RIGHT QUANTITY TO CONTROL, and not the differential. At full lock
//       the inner motor is 0, so yaw and forward surge are THE SAME NUMBER and the yaw-to-thrust
//       ratio is already 1.0 - the maximum two forward-only motors can ever reach. The throttle
//       cannot improve that ratio; there is nothing left to improve. What the throttle DOES control
//       is whether the hull stays in the low-speed regime where it rotates freely on the spot rather
//       than carving a wide arc on a loaded skeg. And because the inner motor is 0, total thrust
//       EQUALS the outer motor command - so the quantity to control is exactly min(2fT, 255), the
//       formula above, and nothing else.
//     PARTIAL LOCK, e.g. 85 counts (two thirds): turn = 0.664T. T = 255 -> (86, 255): differential
//       169 on a total of 341, ratio 0.50, because the outer motor is clipped. T = 127 -> (43, 211):
//       differential 168 - the SAME yaw - on a total of only 254, ratio 0.66. Here the cut really
//       does buy headroom in the ordinary sense: identical turning moment for a quarter less
//       forward thrust.
//   So the assist earns its keep in both regimes, by two different mechanisms: at partial lock it
//   recovers genuine differential headroom, and at full lock it holds total thrust - which IS the
//   outer motor - down to min(2fT, 255) so the hull stays in the regime where it will rotate.
//
//   DO NOT GO BELOW 64 WITHOUT WATER TESTING. At f = 0.20 the outer motor sits at only ~100/255 at
//   full trigger, and the craft may not break out and rotate at all - a pivot that never starts is
//   a worse failure than a pivot that is too wide, because the rider cannot tell it from a fault.
//   64 is the first value below the f = 1/2 trap that still leaves half the outer motor's authority.
//
// WHY ALL NINE ARE COMPILE-TIME AND NOT confStruct FIELDS
//   Both banked reserved slots are spent (rsvd_u16_1 -> steer_during_auto, rsvd_f32_1 ->
//   auto_ramp_s), so any new field grows sizeof(confStruct) past 200, trips the version check and
//   WIPES the owner's stored settings on the next boot. He is on the water tomorrow morning. These
//   are therefore deliberate compile-time constants in the kFm* / kRtm* style already used all
//   over this header: every number is stated here with its derivation so it can be changed by
//   recompile, and a future owner decision can promote any of them to SPIFFS with a struct bump.
//
// THE SHAPE, IN ONE PARAGRAPH (the code is in PWM.ino; this is why the numbers are what they are)
//   Two continuous 0..1 blends - one on speed, one on stick deflection - are multiplied into a
//   DEPTH. The depth is rate-limited, and the throttle reaching the mixer is multiplied by
//   1 - depth x (1 - floor). Depth 0 is a bit-identical pass-through, which is what makes the
//   assist PROVABLY inert above the speed window and at small stick angles.
//
// HOW "INERT" SHOULD BE READ - IT IS WITHIN 80 ms, NOT ON THE SAME TICK
//   An earlier version of these comments called the disengaged cases "byte-for-byte no-ops". That
//   is true of the STEADY STATE and of the never-armed cases, and it is NOT true of the instant of
//   crossing, which matters when reading a log row or reasoning about a handover. The depth does not
//   snap to 0: it BLEEDS OUT over kPivotDepthFallQ8, i.e. up to 8 ticks / 80 ms. So for all three
//   disengagements - RTM or Follow-Me becoming active mid-pivot, the buggy crossing 5 km/h, and the
//   rider straightening the stick - the assist is inert WITHIN 80 ms, monotonically and
//   subtract-only the whole way down, never on the very tick the condition changes. That bleed is
//   deliberate (a one-tick release would be the snatch this design exists to avoid), and it is safe
//   in every case because the assist can only ever SUBTRACT: during those 80 ms the throttle is
//   climbing back toward what the rider and every cap already permit, never past it.
//   The two genuinely same-tick, byte-for-byte cases are: a board that is not steering_type 1 (the
//   gate never opens, so the depth is 0 for the whole session), and any steady state outside the
//   speed or stick window (depth already 0, factor exactly 256/256, (thr x 256) >> 8 == thr).
// ============================================================

// SPEED WINDOW. Below kPivotSpeedFullKmh the speed blend is 1.0; at and above kPivotSpeedZeroKmh
// it is exactly 0 and the assist cannot act at all; in between it is linear.
// WHY 2.5 AND 5.0 km/h. 5.0 km/h is a brisk walking pace and it is the owner's own "starting from
// low speed" boundary - above it he has said repeatedly that he likes the current feel and does
// not want it sharper, so that is where the assist must be arithmetically absent, not merely
// small. 2.5 km/h is set from measured data rather than taste: on the 2026-09-25 dock session
// (Logs testing/2026-09-25-water-fm/RX_092526_202904.csv) a stationary RX logged GPS speeds of
// 0.0-1.3 km/h, so 2.5 km/h clears this board's at-rest noise floor by ~2x and guarantees that a
// genuinely stopped buggy - the nose-misaligned case he described - gets the FULL assist rather
// than a noise-dependent fraction of it.
static const float    kPivotSpeedFullKmh   = 2.5f;   // km/h at or below which the speed blend is 1.0
static const float    kPivotSpeedZeroKmh   = 5.0f;   // km/h at or above which the assist is inert

// HOW OLD A SPEED READING MAY BE. The RX has exactly one calibrated speed: the GPS. See PWM.ino
// for why ERPM is not usable here. A speed that stops arriving must fail toward NO assist, so the
// reading is required to be fresher than this or the assist is skipped entirely.
// WHY 1500 ms. It is the same freshness window getRtmHeading() already applies to GPS course, so
// the firmware now has ONE definition of "a fresh GPS number" instead of two. It is also 4x
// tighter than the 6000 ms the FM_RETURN buggy-speed gate allows, and that margin matters in the
// unsafe direction: the longer this window, the further the buggy could have accelerated behind a
// stale "slow" reading. At the owner's gps_update_hz of 10 against a module that emits at 5 Hz,
// 1500 ms is still seven consecutive missed updates before the assist drops out.
static const uint32_t kPivotSpeedMaxAgeMs  = 1500;   // ms; older than this = no assist at all

// STICK WINDOW, in counts away from the neutral byte 127 (full lock is 127 counts one way, 128 the
// other). AT OR BELOW kPivotSteerStartCounts the stick blend is exactly 0; at or above
// kPivotSteerFullCounts it is 1.0; linear strictly between the two.
// WHY 85 AND 120. 85 counts is two thirds of full deflection - the "deliberate hard stick" the
// owner asked for - and it is 2.1x kFmSteerCancelDeadband (40 counts), the existing threshold for
// "the rider means this", so ordinary steering corrections and the Follow-Me cancel gesture both
// sit far below the assist's first count of authority. 120 counts (~94 %) rather than 127 is the
// margin for a TX that does not quite reach its rail: stick trim, tog_deadzone and ADC spread mean
// a hard-over stick can report 120-125, and the rider must be able to reach FULL assist without
// fighting his own remote for the last few counts.
static const uint8_t  kPivotSteerStartCounts = 85;   // counts from 127; below this the assist is inert
static const uint8_t  kPivotSteerFullCounts  = 120;  // counts from 127; at/above this the stick blend is 1.0

// HOW DEEP THE CUT GOES AT FULL ASSIST, as a Q8 fraction of the rider's own throttle (256 = 1.0).
// WHY 64 (= 25 %), AND WHY NOT THE 128 THE OWNER MEASURED BY HAND. 128 is f = 1/2, which is the
// exact reciprocal of the mixer's 2x gain into the outer motor at full lock, so it cancels that gain
// and cuts NOTHING at full trigger - the one value that is useless in exactly the situation the
// assist is for. Read the min(2fT, 255) block above; it has the derivation and the table. 64 puts
// the outer motor at 126/255 at full trigger (a 51 % cut of total thrust) instead of 254/255 (0 %).
// The owner's "trigger back to about half" was a correct OBSERVATION of a real effect and the wrong
// NUMBER to copy into the throttle domain, because his hand was moving T while this constant moves
// the outer motor through the 2x gain. DO NOT lower it below 64 without water testing - at f = 0.20
// the outer motor is ~100/255 and the craft may never break out and rotate.
static const uint16_t kPivotFloorQ8        = 64;     // Q8; throttle at full assist = this/256 of command

// HOW FAST THE ASSIST MAY COME ON AND LET GO, in Q8 depth units per 10 ms calcPWM() tick.
// The depth cannot step: it walks. That is the whole answer to "it must feel natural, not
// switch-like", and it is also what stops GPS speed noise or stick jitter from producing a snatch -
// the worst the assist can do to the throttle in one tick is this step x (1 - floor)/256, i.e. 12
// counts of 255 on the way in at the shipped floor of 64.
// WHY 16 IN (0.16 s full travel) AND 32 OUT (0.08 s). 0.16 s to full depth is quicker than a human
// can move a stick from two thirds to full lock, so the assist is never the thing the rider waits
// for, while still spreading the full 75 % cut over sixteen ticks rather than delivering it as the
// single step that would feel like a fault. The release is deliberately TWICE as fast, because a sluggish exit from a pivot is
// the one outcome that would be worse than no pivot at all: 0.08 s returns the rider's own throttle
// authority, and it is still gentler than what the mixer already does on the same stick movement -
// straightening from full lock today moves the inner motor from 0 to T in a single 10 ms pass.
static const uint16_t kPivotDepthRiseQ8    = 16;     // Q8 depth per tick coming on  (256/16 = 16 ticks = 0.16 s)
static const uint16_t kPivotDepthFallQ8    = 32;     // Q8 depth per tick letting go (256/32 =  8 ticks = 0.08 s)

// THE HYSTERESIS. Both blends are continuous, so there is no threshold anywhere for the output to
// toggle across - the classic chatter mechanism is absent by construction. What remains is the
// assist repeatedly starting and stopping at the very edge of the window as a noisy GPS speed or a
// trembling stick crosses back and forth. This is a Schmitt band on the depth's zero: the assist
// will not START until the blends together ask for at least this much depth, but once running it
// follows them all the way back down to exactly 0.
// WHY 16/256. It is one rise step, so the arm threshold and the smallest step the depth can take
// are the same number and the depth can never sit in the dead range 1-15. In throttle terms the
// largest jolt this band can ever hide is 16/256 x 75 % = under 5 % of the rider's command, which is
// below perception and is itself delivered over one tick.
static const uint16_t kPivotDepthArmQ8     = 16;     // Q8; minimum depth to START the assist (release is 0)

// -- I2C ENABLE-SWAP STARVATION: FAIL SYMMETRIC, NOT ASYMMETRIC -------------------------------
// V2.5-Evo - 2026-10-03 - the two thresholds of the swap-starvation state machine in PWM.ino.
// Declared here, with the kPivot* block, for the same two reasons: it is where this project keeps
// its derived tuning numbers, and it keeps them OUT of confStruct - sizeof stays 200, nothing is
// wiped, no web-UI field is owed.
//
// WHAT THE STATE MACHINE IS FOR. When the enable swap cannot get i2cMutex, one motor keeps being
// pulsed and the other gets nothing: the starved VESC eventually times out and releases while the
// other holds the rider's commanded throttle. A differential-drive buggy in that state does not
// stop - it turns hard under power, toward the rider and the tow rope. After kSwapStarveTicks
// consecutive failures the firmware therefore stops pulsing BOTH channels, so both VESCs time out
// together and the buggy COASTS. Loss of propulsion is the failure direction this system already
// has (trigger release, link failsafe, VESC timeout all produce it); uncommanded yaw at power is
// not, and the rider has no training for it.
//
// WHY 25 TICKS IN (250 ms), against all four of the clocks that matter:
//   - the PWM task's tick is 10 ms, so 25 ticks is 250 ms: no single tick and no short burst trips
//     it, and one timed-out take is 1/25 of the trip;
//   - the VESC's own timeout_msec is 1000 ms. 250 ms is ONE QUARTER of it, so the symmetric stop
//     lands ~750 ms BEFORE the starved VESC would have released - the asymmetry never becomes
//     mechanical at all. This is the load-bearing number;
//   - usrConf.failsafe_time is 1000 ms by default (validated 100-10000), so this nests INSIDE the
//     existing link authority rather than competing with it;
//   - on a healthy bus the worst instantaneous pile-up of every other holder is ~1.5 ms against a
//     10 ms take budget - not one failure, let alone 25. 25 can only be reached by repeated
//     physical-layer timeouts, which is precisely the condition it should fire on.
//
// 🔴 25 IS ONLY VALID BECAUSE Wire.setTimeOut(3) LANDED IN initHardware() (Init.ino, STEP 1 of this
// fix). At the OLD 20 ms per-transaction ceiling the worst plausible HEALTHY transient burst was
// setUartMux() on its corrective path (10 txn = 200 ms = 20 ticks) immediately followed by a
// compass read that also timed out (2 txn = 40 ms = 4 ticks) = 24 ticks - ONE TICK short of 25,
// which is far too thin a margin and would have made the correct value 40. At 3 ms the same burst
// is 36 ms = 4 ticks and 25 has ~6x margin. If Wire.setTimeOut() is ever raised, THIS NUMBER IS NO
// LONGER JUSTIFIED and must be re-derived; the two are coupled and a change to one silently
// invalidates the other.
static const uint16_t kSwapStarveTicks   = 25;   // consecutive FAILED enable swaps before both channels stop pulsing (10 ms/tick => 250 ms)

// WHY 5 TICKS OUT (50 ms), AND WHY RECOVERY IS NOT "the first successful swap".
// Recovery is a Schmitt trigger, deliberately asymmetric: 250 ms to cut, 50 ms to clear.
// THE HAZARD THIS CLOSES is F-1 of the 2026-10-02 M-3 delta audit: because M-3 drives the throttle
// ramp's target to 0 whenever the motor gate is shut, a gate that FLAPS costs a full re-ramp every
// time it reopens - and a marginal bus that produced cut/resume/cut/resume on alternate ticks would
// pin the throttle near the bottom of the ramp indefinitely. The buggy would crawl rather than stop,
// which is a failure mode with no clear signature and no obvious rider response. Requiring five
// consecutive GOOD swaps means one lucky swap in a storm of failures cannot resume output, so the
// oscillation cannot start.
// WHY NOT A HARD LATCH: a latch converts a transient glitch into a dead ride, with a rider on a rope
// and no way to restore propulsion without a power cycle he cannot perform on the water. That is a
// worse safety outcome than the fault being fixed. Auto-recovery is safe here specifically because
// the stop is implemented as a GATE TERM: the ramp memory is at 0 while it is shut, so recovery is a
// full soft ramp from 0 over motor_ramp_s, not a step back to the commanded throttle.
// WHY 5 AND NOT MORE: 50 ms is five motor ticks and half a control-packet interval at 10 Hz - below
// the threshold at which a rider could perceive the delay - while still being five independent
// pieces of evidence that the bus is working again.
static const uint16_t kSwapRecoverTicks  = 5;    // consecutive SUCCESSFUL enable swaps required to resume pulsing (10 ms/tick => 50 ms)

#include "../Common/ConfigServiceEngine.h"

// Web config globals
#ifdef WIFI_ENABLED
volatile bool web_cfg_service_enabled = false;
volatile bool web_cfg_pending_save = false;
volatile bool web_cfg_radio_reinit_required = false;
volatile uint32_t web_cfg_req_total = 0;
volatile uint32_t web_cfg_req_ok = 0;
volatile uint32_t web_cfg_req_err = 0;
volatile uint8_t web_cfg_debug_mode = 1; // 0=off, 1=some, 2=full
volatile uint32_t web_cfg_ap_startup_timeout_ms = 60000; // 0 disables timeout
String web_cfg_last_err = "";
#endif
volatile bool config_version_error = false;

// ============================================================
// V2.5-Evo - 2026-04-24 - TX GPS COORDINATES (received via 0xF3 meta-packet)
//
// Written by processMetaGpsPacket() in Radio.ino at 2Hz whenever TX sends
// a GPS meta-packet and RX successfully validates it.
// Read by Phase B anti-spoofing (Priority 6) to check TX-RX proximity.
//
// rx_tx_gps_timestamp == 0 means no meta-packet has ever been received.
// Use (millis() - rx_tx_gps_timestamp) > usrConf.tx_gps_stale_timeout_ms
// to detect a stale TX GPS reading before trusting lat/lng.
// ============================================================
double        rx_tx_gps_lat       = 0.0;  // TX latitude (degrees, WGS84)
double        rx_tx_gps_lng       = 0.0;  // TX longitude (degrees, WGS84)
unsigned long rx_tx_gps_timestamp = 0;    // millis() when last meta-packet received; 0 = never

// V2.5-Evo - 2026-08-29 - REX R4-1. Bumped once per DISTINCT rider position by
// processMetaGpsPacket(), and read by everything downstream that needs to tell a genuinely new fix
// from a re-broadcast. The TX transmits at 2 Hz off a 1 Hz GPS (3.0 Hz measured against 1 Hz in
// the beta logs), so roughly every other meta packet repeats a position already seen.
//
// WHY A COUNTER RATHER THAN COMPARING rx_tx_gps_lat/lng AT EACH CONSUMER. Comparing the position
// is correct arithmetic and WRONG CONCURRENCY: those are doubles, written on the radio task and
// read on the loop task, and a double is TWO 32-bit stores on RV32. A read preempted between them
// sees a half-written value, counts it as a new fix, then counts the complete value as another on
// the next tick - two phantom fixes injected into an interlock whose entire job is to require
// three. A uint32_t is a single naturally-aligned access on this core and cannot tear.
//
// volatile because the writer is the radio task and the reader is the loop task - the same
// discipline this project documents for logging_active.
volatile uint32_t rx_tx_gps_fix_seq = 0;   // one increment per distinct rider position

// V2.5-Evo - 2026-04-25 - P7 RTM/FM runtime state (set by Radio.ino meta-packet handlers)
// rtm_rx_active: true = TX signalled RTM active; safety gates in RTMState.ino may override.
// rtm_rx_emergency_stop: true = safety gate failed; calcPWM() forces throttle to 0.
// rtm_steer_override: bearing-derived steering value (0-255, 127=straight ahead).
// fm_mode_runtime: TX-side FM mode override (0-3); 0xFF = use SPIFFS default.
// V2.5-Evo - 2026-04-25 - P7 fix: use std::atomic for safe access across FreeRTOS task
// preemption. generatePWM (task) and RTMState.ino loop() both run on the single-core
// ESP32-C3; std::atomic gives an indivisible read/write + compiler barrier so a higher-
// priority task can't observe a torn value. (seq_cst, matching the rfInterrupt pattern.)
std::atomic<bool>    rtm_rx_active         {false};
std::atomic<bool>    rtm_rx_emergency_stop {false};
std::atomic<uint8_t> rtm_steer_override    {127};
std::atomic<uint8_t> fm_mode_runtime       {0xFF};

// V2.5-Evo - 2026-07-20 - R2: millis() when the last 0xF2 FM-mode declaration arrived from the
// TX; 0 = none has ever arrived this session. The TX refreshes its declaration every 30 s while
// armed, so RTMState.ino expires the mode after kFmModeAgeMs (95 s, ~3 missed keepalives) and
// returns FM to IDLE. Without this the RX kept a declared mode forever, which meant a lost
// disarm burst left the RX armed for the rest of the session with no way to discover it.
// Written by Radio.ino's meta-packet handler (triggeredReceive task), read by RTMState.ino's
// runFmLoop() (loop) — std::atomic for the same single-core preemption reason as the flags above.
std::atomic<unsigned long> fm_mode_last_rx_ms {0};

// V2.5-Evo - 2026-09-19 - the remote's SESSION override of fm_return_mode (auto-return inside
// Follow-Me). 0xFF = no override this session: the RX uses its stored usrConf.fm_return_mode.
// 0 / 1 = the remote's return gesture set it OFF / ON for the session; carried in bits 5-6 of every
// 0xF2 the remote sends (00 = none -> 0xFF here, 01 = OFF -> 0, 10 = ON -> 1, 11 = ignored), so a
// lost packet is repaired by the next 30 s keepalive and a remote power cycle (RAM lost there)
// falls back to the stored default within one keepalive. Cleared to 0xFF by fmEnterIdle() (which
// the 95 s mode-age expiry and a 0xF2/0 disarm both reach). The EFFECTIVE value - this when it is
// not 0xFF, else the SPIFFS field - is resolved once per tick in runFmLoop() and echoed to the
// remote in telemetry.fm_flags bit 7. Written by Radio.ino (triggeredReceive task), read by
// RTMState.ino (loop task): std::atomic for the same single-core preemption reason as above.
std::atomic<uint8_t> fm_return_mode_runtime {0xFF};
std::atomic<uint8_t> rtm_approach_cap      {255};  // V2.5-Evo - 2026-04-30 - approach decel cap (0-255); 255=no cap; computed by RTMState.ino during active RTM; applied by calcPWM()

// V2.5-Evo - 2026-07-19 - P3 Follow-Me (FM) autonomous-following runtime flags.
// fm_rx_active : true while FM is actively steering. Gates the steering override in calcPWM()
//                using the SAME pattern as rtm_rx_active (RTM and FM are mutually exclusive, so
//                they safely share rtm_steer_override as the steering command).
// fm_throttle_cap : FM's own subtract-only throttle cap (0-255; 255 = no cap). Applied in calcPWM()
//                alongside rtm_approach_cap (lowest cap wins). Deliberately a SEPARATE global from
//                rtm_approach_cap so RTM's per-tick housekeeping (which rewrites rtm_approach_cap=255
//                whenever RTM is inactive) can never transiently clear an FM cap in the window between
//                runRtmLoop() and runFmLoop() on the single-core ESP32-C3. seq_cst, same as the RTM
//                atomics — an indivisible read/write the 100Hz generatePWM task cannot tear.
std::atomic<bool>    fm_rx_active     {false};
std::atomic<uint8_t> fm_throttle_cap  {255};

// V2.5-Evo - 2026-09-19 - THE PIVOT BOOST: the steering influence the differential mixer should use
// INSTEAD of usrConf.steering_influence while an autonomous controller is turning the buggy to
// face its target (heading error above rtm_align_threshold_deg) at the align cap. 0 = no override
// (calcPWM() uses steering_influence). Non-zero = usrConf.fm_align_influence (100 = one-motor
// pivot at the align cap: motors 0 / 2 x cap). Published for: Follow-Me ACTIVE align (cap 4),
// FM_RETURN align, and classic RTM Phase 1 align (owner decision 2026-09-19 - with the
// throttle-relative mixer, RTM's align at cap 13 / influence 55 was 8-14 counts of differential).
// SINGLE WRITE SITE: publishAlignMixerInfluence() in RTMState.ino, called once at the end of every
// runRtmLoop() and runFmLoop() tick with RTM outranking FM; each loop's request starts at 0 on
// every tick, so any exit from an align branch clears it. Reader: calcPWM() (generatePWM task,
// 100 Hz), which applies it only on ticks where it is already applying an autonomous steering
// override (rtm_rx_active || fm_rx_active, override enabled, trigger held). std::atomic for the
// same single-core preemption reason as fm_throttle_cap above.
std::atomic<uint8_t> align_mixer_influence_override {0};

// V2.5-Evo - 2026-09-19 - THE STICK TAKEOVER: true while the rider's stick, not the controller, is
// the steering byte calcPWM() applies during an automatic steering run (Follow-Me following,
// FM_RETURN, classic RTM) - usrConf.steer_during_auto == 1 and the arbitration in
// Common/SteerArbitration.h has engaged. It changes ONLY the steering-byte selector: no cap, no
// effective_thr, no rtm_steer_override and no PWM time is written because of it, and the pivot
// boost above follows the CONTROLLER's byte (a takeover mixes the stick at steering_influence).
// SINGLE WRITE SITE: publishSteerTakeover() in RTMState.ino, called once at the end of every
// runRtmLoop() and runFmLoop() tick with RTM outranking FM; each loop's request starts at false on
// every tick, so any tick with no auto-steer owner (mode not steering, trigger below 25 counts,
// override off) publishes false - the flag can never outlive its owner by more than one 10 Hz
// tick, and calcPWM()'s own thr_received >= 25 test makes it irrelevant before that. Reader:
// calcPWM() (generatePWM task, 100 Hz). With steer_during_auto == 0 it is never written true.
// std::atomic for the same single-core preemption reason as fm_throttle_cap above.
std::atomic<bool> steer_takeover_active {false};

#include "../Common/SPIFFSEngine.h"

// ============================================================
// V2.5-Evo - 2026-07-25 - STAGE 0 DIAGNOSTIC COUNTERS (instrumentation only)
//
// WHAT THIS IS
//   A set of plain counters that the rest of the firmware bumps at the exact point where the
//   event happens. They exist so a full on-water session log can explain itself instead of
//   needing a 60-second bench capture to reproduce a fault that only happens on the water.
//
// WHAT READS THEM — exactly two consumers, both read-only:
//   1. the level-4 log record (fillLevel4Diag() in Logger.ino), and
//   2. the ?diag / ?diagz serial commands (System.ino).
//   NOTHING in the control path reads any of these. No throttle, steering, PWM, mux schedule,
//   FM or RTM decision depends on a single value below. Deleting this whole block would change
//   how the buggy drives by precisely nothing.
//
// WHY PLAIN volatile uint32_t AND NO MUTEX
//   A 32-bit aligned load or store is a single instruction on this RISC-V core, so a reader can
//   never observe a half-written value. The increment itself is a read-modify-write, so if two
//   execution contexts bump the SAME counter and one preempts the other mid-increment, one count
//   can be lost. That is accepted deliberately: these are diagnostics, one lost count in a
//   million is invisible in a rate, and taking a mutex inside the GPS byte loop or inside
//   setUartMux() would perturb the very timing we are trying to measure.
//
// COST
//   One load, one add, one store per event. The only counter that sits in a tight loop is
//   g_diag_gps_bytes (once per GPS UART byte) and it is a bare increment — no branch, no call,
//   no formatting. There is no printf anywhere in this instrumentation except inside ?diag,
//   which is a one-shot operator command.
// ============================================================

// --- GPS feed health ---
volatile uint32_t g_diag_gps_bytes      = 0;  // bytes pulled off Serial1 by getGPSLoop() since boot / ?diagz
volatile uint32_t g_diag_gps_sentences  = 0;  // COMPLETE, checksum-valid NMEA sentences parsed (gps.encode() returned true)
volatile uint8_t  g_diag_gps_sent_per_s = 0;  // sentences parsed during the last completed 1-second window; recomputed in getGPSLoop(). Saturates at 255.

// --- GPS course-over-ground (COG) ---
// THE POINT OF THESE TWO COUNTERS: gps_last_course_ms is refreshed on EVERY valid course
// sentence, even when the module keeps repeating the same heading. The existing cog_age_ms_div10
// log column is derived from that timestamp, so it read "fresh" straight through the failure that
// actually cost control — a COG VALUE frozen on one heading while its timestamp kept ticking.
// g_diag_cog_ts_updates counts the timestamp refreshes; g_diag_cog_val_changes counts only the
// times the NUMBER genuinely moved. When the first is high and the second is ~0, the GPS is
// repeating itself and any heading derived from COG is stale even though it looks current.
volatile uint32_t g_diag_cog_ts_updates  = 0;  // times gps_last_course_ms was refreshed
volatile uint32_t g_diag_cog_val_changes = 0;  // times the COG VALUE moved by more than kDiagCogChangeDeg
volatile uint32_t g_diag_cog_change_ms   = 0;  // millis() when the COG VALUE last moved. SENTINEL: 0 = no COG value has EVER been captured this session (never a valid timestamp — the writer substitutes 1 for a literal millis()==0).
static const float kDiagCogChangeDeg     = 0.05f;  // degrees; smallest COG movement counted as a real change. Below this is GPS quantisation/noise, not motion.

// --- RX GPS fix age (sampled once per GPS poll, so it tracks staleness even when updates stop) ---
volatile uint32_t g_diag_fix_age_sum_ms = 0;  // running sum of sampled fix ages, for the mean
volatile uint32_t g_diag_fix_age_samples = 0; // number of samples in that sum
volatile uint32_t g_diag_fix_age_max_ms = 0;  // worst fix age seen; ?diag reads then resets it

// --- AW9523 UART mux (the suspected EMI failure) ---
volatile uint32_t g_diag_mux_switches = 0;    // setUartMux() calls that actually drove the select pins
volatile uint32_t g_diag_mux_errors   = 0;    // read-back mismatches inside setUartMux() (a write that did not stick)

// --- VESC polling ---
volatile uint32_t g_diag_vesc_polls = 0;      // getVescLoop() query attempts
volatile uint32_t g_diag_vesc_ok    = 0;      // of those, replies that parsed and validated

// --- loop() timing (microseconds, derived from the CPU cycle counter) ---
volatile uint32_t g_diag_loop_count      = 0;          // completed loop() bodies
volatile uint32_t g_diag_loop_us_sum     = 0;          // running sum of loop-body durations, for the mean
volatile uint32_t g_diag_loop_min_us     = 0xFFFFFFFF; // shortest loop body; 0xFFFFFFFF = no sample yet
volatile uint32_t g_diag_loop_max_us     = 0;          // longest loop body — owned by ?diag, which reads then resets it
volatile uint32_t g_diag_loop_max_us_log = 0;          // longest loop body — owned by the LOGGER, which reads then resets it once per level-4 record (kept separate so ?diag and the log never steal each other's peak)

// ============================================================
// diagCpuMhz - CPU clock in MHz, cached after the first call
//
// Inputs:  none. Outputs: MHz (160 on a stock ESP32-C3).
// Side effects: none after the first call. Used only to turn a cycle delta into microseconds.
// ============================================================
static inline uint32_t diagCpuMhz()
{
  static uint32_t mhz = 0;
  if (mhz == 0)
  {
    mhz = getCpuFrequencyMhz();
    if (mhz == 0) mhz = 160;   // defensive: never divide by zero if the call ever returns 0
  }
  return mhz;
}

// ============================================================
// diagLoopBegin / diagLoopEnd - measure one pass through the loop() body
//
// diagLoopBegin() is called on the FIRST line of loop() and returns a raw cycle stamp.
// diagLoopEnd(stamp) is called on the LAST line of the loop() body, BEFORE its own
// vTaskDelay(10) — so what is measured is the WORK the loop did, not the 10 ms it then
// deliberately sleeps. A "loop_max_ms" of 40 therefore means the loop genuinely spent 40 ms
// doing something, which is what starves GPS drains and RTM/FM ticks.
//
// esp_cpu_get_cycle_count() is a single CSR read (~5 ns). micros() is ~1 us, which is the same
// order as the shortest thing being measured, so it is deliberately NOT used here.
// The 32-bit cycle counter wraps every ~26.8 s at 160 MHz; the unsigned subtraction below is
// correct across that wrap for any interval shorter than 26.8 s, and the 3 s watchdog
// guarantees a loop body is never anywhere near that long.
//
// Side effects: updates the loop-timing counters above. Nothing else. No allocation, no I/O.
// ============================================================
static inline uint32_t diagLoopBegin()
{
  return (uint32_t)esp_cpu_get_cycle_count();
}

static inline void diagLoopEnd(uint32_t start_cycles)
{
  uint32_t us = (uint32_t)((uint32_t)esp_cpu_get_cycle_count() - start_cycles) / diagCpuMhz();
  g_diag_loop_count++;
  g_diag_loop_us_sum += us;
  if (us > g_diag_loop_max_us)     g_diag_loop_max_us     = us;
  if (us > g_diag_loop_max_us_log) g_diag_loop_max_us_log = us;
  if (us < g_diag_loop_min_us)     g_diag_loop_min_us     = us;
}

// --- Global VESC Logger Struct ---
struct vesc_struct {
  int16_t fetTemp = 0;
  int32_t motCur = 0;
  int32_t batCur = 0;
  int16_t duty = 0;
  int32_t erpm = 0;
  int16_t batVolt = 0;
  int32_t wh_raw = 0;          // session Wh×10 from VESC float32_auto; 0 = unavailable
  uint8_t fault_code = 0;
  unsigned long last_packet = 0;
};
extern vesc_struct vesc;

struct __attribute__((packed)) VescLogData {
    uint32_t timestamp;           // Local Timestamp in ms
    int16_t current_motor;       // Motor Current in 0.01A
    int16_t current_battery;     // Battery Current in 0.01A
    int8_t duty_cycle;           // Duty cycle in %
    uint16_t voltage;             // Voltage in 0.1V
    int16_t ERPM;                // ERPM / 10
    int8_t temp_mos;             // MOSFET temperature in °C
    uint8_t fault_code;           // Error code
    uint16_t speed;               // Speed in 0.1 km/h
    float latitude;               // Latitude in degrees
    float longitude;              // Longitude in degrees
    uint32_t datetime;            // UTC datetime as unix timestamp
    // V2.5-Evo - 2026-05-06 - LOG-EXT-1: heading source debug fields.
    // Populated by convertToLogData() in Logger.ino (LOG-EXT-2).
    // All ×10 fields use 0xFFFF as the "invalid/no data" sentinel.
    // rtm_heading_chosen_dx10 uses int16, -1 sentinel for "no source".
    uint8_t  thr_received_log;          // TX throttle last received (0-255)
    uint8_t  rtm_source;                // 0=NONE, 1=GPS_COG, 2=COMPASS_SNAPSHOT, 3=COMPASS_LIVE (legacy mode)
    uint8_t  rtm_confidence;            // 0=NONE, 1=LOW, 2=MEDIUM, 3=HIGH
    uint8_t  rtm_rx_active_log;         // RTM engagement state (0/1)
    uint8_t  gps_phase_b_ok_log;        // Phase B anti-spoofing handshake state (0/1)
    uint8_t  rtm_steer_override_log;    // Current steering command 0-255 (127 = straight ahead)
    int16_t  rtm_heading_chosen_dx10;   // getRtmHeading() output × 10 deg; -1 if no valid source
    uint16_t compass_live_dx10;         // Live compass heading × 10 deg (0xFFFF = invalid/uncalibrated)
    uint16_t compass_snap_dx10;         // Clean compass snapshot × 10 deg (0xFFFF = no snapshot yet)
    uint16_t snap_age_s;                // Snapshot age in seconds (0xFFFF = no snapshot yet)
    uint16_t gps_course_dx10;           // GPS course-over-ground × 10 deg (0xFFFF = no fix or invalid)
    uint16_t cog_age_ms_div10;          // GPS course age in 10ms units (0xFFFF = no fix yet)
    // V2.5-Evo - 2026-05-08 - Bundle 1: heading controller tuning telemetry fields.
    // 0x7FFF (32767) is the "no data" sentinel — NOT 0x0000.
    int16_t heading_error_dx10;   // Heading error in 0.1° units (signed; -1800..+1800).
                                  // 0x7FFF = no valid heading source. Positive = need turn right.
    int16_t d_error_dx10;         // Rate-of-change of heading error in 0.1°/s units.
                                  // 0x7FFF = no prior sample (first cycle). For tuning Kd.
    // V2.5-Evo - 2026-05-11 - E7 Fix: BREmote remote_error code for cross-correlation with VESC/motor data.
    // 0 = no error, 71 = E71 water ingress (see checkWetness() in System.ino).
    uint8_t error_code_log;       // telemetry.error_code at log time. 0 = no BREmote error.
    // V2.5-Evo - 2026-07-19 - FM triage: steering byte actually applied to the motor mix by
    // calcPWM() (g_effective_steer), NOT just the commanded rtm_steer_override. Reveals the
    // actuation gap — a valid heading error can log a non-127 rtm_steer_override_log while this
    // stays 127 because the throttle-release gate suppressed it. 127 = straight ahead.
    uint8_t effective_steer_log;
    // V2.5-Evo - 2026-07-24 - F9: owner-requested range telemetry — RX→TX distance + LoRa link quality.
    // Appended at the tail so existing CSV column order is preserved (old parsers ignore trailing columns).
    uint16_t tx_distance_dx10;    // RX→TX distance × 10 m (0.1 m resolution, capped ~164 m); 0xFFFF = N/A (no valid GPS pair)
    int16_t  rssi_dbm;            // last control-packet RSSI in dBm (rounded); 0x7FFF = N/A (failsafe — no recent packet)
    int16_t  snr_dx10;            // last control-packet SNR × 10 dB; 0x7FFF = N/A (failsafe — no recent packet)
    // V2.5-Evo - 2026-10-01 - M-2 (diagnosability): THE MOTOR GATE. In the BASE record on purpose -
    // the tiers are cumulative and levels 0-3 all record as level 3, so a field here appears at every
    // log level 0 through 5, and you never get to re-run an incident at a higher level. Needed because
    // every other link field in this record keys on last_packet ("is the remote alive?") - including
    // the rssi_dbm / snr_dx10 N/A sentinels just above - while the motor gate keys on
    // last_control_packet ("do I have a fresh throttle command?"). A motor gated off for want of a
    // control packet therefore logged as a perfectly healthy link, with nothing recording the gate.
    uint16_t ctrl_pkt_age_ms;     // millis() - last_control_packet at log time, in ms; capped at 0xFFFE (>= ~65.5 s of control silence is one state)
    uint8_t  motor_gate_open;     // 1 = gate OPEN, pulses reaching the ESCs; 0 = SHUT, nothing leaving the board. Copy of g_motor_gate_open - calcPWM()'s own verdict, so it carries PWM_active too, which the age alone cannot show.
};
static_assert(sizeof(VescLogData) == 62, "VescLogData size mismatch — check binary log compat.");  // 29 base; +18 LOG-EXT-1 (2026-05-06); +4 Bundle 1 tuning fields (2026-05-08); +1 error_code_log E7 fix (2026-05-11); +1 effective_steer_log FM triage (2026-07-19); +6 F9 distance+RSSI+SNR (2026-07-24); +3 M-2 motor-gate block ctrl_pkt_age_ms+motor_gate_open (2026-10-01), 59->62

// ============================================================
// V2.5-Evo - 2026-07-25 - STAGE 0 PART C: LEVEL-4 ("Deep") LOG RECORD
//
// Tiers are ADDITIVE: level N is level N-1 plus a block. VescLogDataL4 starts with a complete,
// byte-identical VescLogData, so the first 62 bytes of a level-4 record decode with exactly the
// same code that decodes a level-3 record. That is what lets one CSV formatter serve both.
//
// The four fields answer the four open theories about the on-water failure:
//   gps_sent_per_s — is the GPS still delivering sentences at all, or has the feed died?
//   cog_frozen_s   — how long has the COG VALUE been stuck? (The existing cog_age_ms_div10
//                    column tracks the TIMESTAMP, which kept refreshing while the value was
//                    frozen, so it was blind to exactly this failure.)
//   mux_err_cnt    — how many AW9523 mux writes failed read-back? (motor EMI corrupting I2C)
//   loop_max_ms    — did the main loop stall long enough to starve the GPS drain / RTM tick?
// ============================================================
// ============================================================
// V2.5-Evo - 2026-09-17 - FOLLOW-ME AUDIT BLOCK (18 bytes, appended after the four diagnostics)
//
// WHY: a Follow-Me run that stops, holds, or refuses to engage leaves no record of WHICH gate did
// it. These columns are the controller's own verdicts, copied out once per tick — not recomputed
// by the logger from raw inputs — so a log row says what runFmLoop() actually decided at that
// instant (Rex positive finding on robertzach's snapshot design).
//
// LAYOUT (history: 'LAYOUT IS FINAL' as written 2026-09-17; the record grew 83 -> 87 B on 2026-09-19,
// see below). The P1 (return_candidate) and P2 (fade_bypass, transit, fm_station_deg_x10) fields
// were laid out then and written as zero. Old 65-byte and 83-byte level-4 files still parse: the file
// header's record_size tells the reader which blocks are present (logCsvHeaderFor / logFormatCsvRow
// pick by the OFFSET of each block, never by sizeof, so a bump never re-labels an older file).
//
// fm_gate_flags bits (1 = the condition held on this tick):
//   bit 0 thr_held          condition 1, the deadman (thr_received >= 25)
//   bit 1 fault_ok          conditions 2-7 all hold (Phase A/B, TX/RX GPS fresh, heading, link)
//   bit 2 speed_ok          condition 9, rider above foiler_low_speed_kmh (+hysteresis to engage)
//   bit 3 dist_ok           condition 8, distance Schmitt (beyond min_dist + band / inside min_dist)
//   bit 4 sep_latched       the separation proof (tow interlock) is standing
//   bit 5 diverge           the A3 divergence fault fired on this tick
//   bit 6 pivoting          PIVOT-SUSPEND-1 is suspending the divergence judgement
//   bit 7 in_grace          inside the post-engage grace (kFmJudgeGraceMs, 6500 ms; until 2026-09-19 kFmEngageRampMs + kFmDivergeMs)
//   bit 8 heading_disagree  the compass-vs-COG disagreement latch is standing
//   bit 9 fade_bypass       P2 — the GOVERNOR-2 fade bypass is standing this tick (live since the 2026-10-02 station work)
//   bit 10 transit          P2 — a station is in transit toward a FRONT preset this tick (live since the 2026-10-02 station work)
//   bit 11 return_candidate P1-b (live since 2026-09-19): a RETURN candidate stands this tick - the
//                           rider's raw speed is under the band and the entry proof is running
//   bit 15 return_window    P1-b: the proof's relative-displacement window is open this tick
//   V2.5-Evo - 2026-09-18 - P1-a (trigger-free engagement) adds two bits. Bit 11 stays reserved for
//   the auto-return proof (P1-b, not built); these are new bits in the same u32, no record change.
//   bit 12 proof_ok         conditions 2-7 hold, so the distance, the dwell and the latch were
//                           evaluated this tick WITHOUT the trigger (P1-a: proof_ok = fault_ok)
//   bit 13 needs_dengage    the next engagement must clear the full D_engage (set by a trigger
//                           release of kFmReleaseDengageMs or more; cleared on the ACTIVE edge)
//   bit 14 yield_to_rtm     P1-c: Return-to-Me is active and Follow-Me is parked in FM_ARMED,
//                           writing no cap and no steering. The only bit set on such a tick.
//   bit 16 steer_takeover   V2.5-Evo - 2026-09-19: the rider's stick has taken over the steering
//                           byte this tick (steer_during_auto 1, arbitration engaged) while
//                           Follow-Me was following or returning. New bit in the same u32, no
//                           record change. (V2.5-Evo - 2026-09-24: the two branches merged, so this
//                           is no longer "reserved" - it is live. Set straight into fm_log_gate_flags
//                           inside the body, not at publish time like 17/18.)
//   V2.5-Evo - 2026-09-19 - DEEP LOG (A): two bits set at PUBLISH time (fmPublishLogSnapshot) from
//   published state, on EVERY tick whatever path the body took - they are not gate verdicts.
//   bit 17 aligning         an autonomous controller (Follow-Me or Return-to-Me) is ACTIVE and in its
//                           align phase this tick: |heading error| > rtm_align_threshold_deg, the
//                           same fmHeadingAligning() test cap 4 / FM_RETURN / RTM Phase 1 use. Gated
//                           on fm_rx_active || rtm_rx_active on purpose: with nothing engaged the
//                           0x7FFF no-heading sentinel reads as 180 deg and the bit would be set on
//                           every idle tick
//   bit 18 boost            the pivot boost is published this tick: align_mixer_influence_override
//                           is non-zero, so calcPWM() mixes with fm_align_influence, not
//                           steering_influence
//   V2.5-Evo - 2026-09-25 - MANUAL PIVOT ASSIST: bits 19-22 are ONE 4-BIT FIELD, not four flags,
//   and they are the only thing this feature adds to the log - no new column, no record-size
//   change, the same "ride in the existing u32" mechanism bits 16-18 use.
//   bits 19-22 pivot_assist_q4  the MANUAL pivot assist's depth on this tick, 0-15 (g_pivot_assist_q4,
//                           written by calcPWM() at 100 Hz, copied in at fill time like motor0_cmd /
//                           motor1_cmd). 0 = the assist is inert and the throttle reaching the mixer
//                           is bit-identical to the un-assisted value; 15 = full assist. ONE field
//                           answers both tuning questions: non-zero says WHEN it was active, the
//                           value says HOW MUCH it cut - the throttle multiplier is
//                           1 - (value/15) x (1 - kPivotFloorQ8/256), i.e. 1.00 at 0 down to 0.25 at
//                           15 with the shipped floor of 64. To read the cut in counts, multiply by
//                           the same row's thr_received; to read what the OUTER motor actually got at
//                           full lock, that result doubles (clamped at 255) - see the min(2fT, 255)
//                           block beside the kPivot* constants. NOTE it is the assist's REQUEST, so at
//                           zero throttle a non-zero depth still cut nothing (0 x anything is 0).
//                           Never non-zero on a non-differential steering_type (the gate never opens).
//                           It CAN be non-zero for up to 8 rows-worth of ticks (80 ms) after
//                           rtm_rx_active || fm_rx_active goes true, because the depth bleeds out
//                           rather than snapping to 0 - so a handover row may legitimately show a
//                           falling assist depth alongside an FM/RTM verdict. It is still
//                           subtract-only throughout that bleed.
//   V2.5-Evo - 2026-10-03 - I2C ENABLE-SWAP FAILURES: bits 23-30 are TWO MORE 4-BIT FIELDS, on the
//   same "ride in the existing u32" mechanism. Zero added bytes, no record-size change, no
//   LOG_FILE_FORMAT_VER bump, and every log file written before today still parses.
//   bits 23-26 swap_fail_ch0_q4  FAILED enable swaps counted on ticks where PWM0 was the enabled
//                           channel, SINCE THE PREVIOUS LOG ROW, saturating at 15. A DELTA, not a
//                           total - computed in fillLevel4Diag() against a loggerTask-local
//                           snapshot of the monotonic g_swap_fail_ch0, so the logger never writes
//                           the PWM task's variables and the single-writer contract holds.
//   bits 27-30 swap_fail_ch1_q4  the same for PWM1.
//                           READ THEM THE RIGHT WAY ROUND: a non-zero ch0 field means PWM0 kept
//                           being re-pulsed, so PWM1 WAS THE STARVED CHANNEL. The field names the
//                           channel that kept its pulses.
//                           SATURATION IS DELIBERATE AND IS NOT A DEFECT: at 100 Hz there are at
//                           most ~33 swaps in a 333 ms row, so 15 does not cover a total outage.
//                           These fields are a RATE INDICATOR - a row reading 15/15 means "pinned",
//                           which is all the CSV needs to say - and ?diag carries the exact
//                           cumulative figures. Both fields read 0 on a healthy bus.
//                           Bit 31 is the PG-4 escape (FM_LOG_GATE_AIM_OUTWARD, see below).
// Bits 0-3, 5-7 are only evaluated on ticks that reach the condition block (FM_ARMED and beyond
// with a live declaration); on IDLE / STOPPING / early-exit ticks the whole word is 0 apart from
// bits 17-18, which describe the engaged controller and are therefore 0 on such ticks anyway.
// ============================================================
#define FM_LOG_GATE_THR_HELD          (1UL << 0)
#define FM_LOG_GATE_FAULT_OK          (1UL << 1)
#define FM_LOG_GATE_SPEED_OK          (1UL << 2)
#define FM_LOG_GATE_DIST_OK           (1UL << 3)
#define FM_LOG_GATE_SEP_LATCHED       (1UL << 4)
#define FM_LOG_GATE_DIVERGE           (1UL << 5)
#define FM_LOG_GATE_PIVOTING          (1UL << 6)
#define FM_LOG_GATE_IN_GRACE          (1UL << 7)
#define FM_LOG_GATE_HEADING_DISAGREE  (1UL << 8)
#define FM_LOG_GATE_FADE_BYPASS       (1UL << 9)    // P2, reserved
#define FM_LOG_GATE_TRANSIT           (1UL << 10)   // P2, reserved
#define FM_LOG_GATE_RETURN_CANDIDATE  (1UL << 11)   // P1-b (live 2026-09-19)
#define FM_LOG_GATE_PROOF_OK          (1UL << 12)   // P1-a
#define FM_LOG_GATE_NEEDS_DENGAGE     (1UL << 13)   // P1-a
#define FM_LOG_GATE_YIELD_TO_RTM      (1UL << 14)   // P1-c
#define FM_LOG_GATE_RETURN_WINDOW     (1UL << 15)   // P1-b (2026-09-19): the relative-displacement window is open
#define FM_LOG_GATE_STEER_TAKEOVER    (1UL << 16)   // 2026-09-19: the rider's stick has taken over the steering byte this tick
#define FM_LOG_GATE_ALIGNING          (1UL << 17)   // DEEP LOG (A) 2026-09-19: engaged controller is in its align phase this tick
#define FM_LOG_GATE_BOOST             (1UL << 18)   // DEEP LOG (A) 2026-09-19: the pivot-boost mixer influence is published this tick
// V2.5-Evo - 2026-09-25 - MANUAL PIVOT ASSIST: a 4-BIT FIELD, not a flag. Shift/mask rather than a
// single-bit macro so nobody can accidentally test it with the == pattern the flags above use.
// Reader: value = (fm_gate_flags & FM_LOG_GATE_PIVOT_ASSIST_MASK) >> FM_LOG_GATE_PIVOT_ASSIST_SHIFT.
#define FM_LOG_GATE_PIVOT_ASSIST_SHIFT 19            // bits 19-22 hold the manual pivot assist depth, 0-15
#define FM_LOG_GATE_PIVOT_ASSIST_MASK  (0xFUL << FM_LOG_GATE_PIVOT_ASSIST_SHIFT)
// V2.5-Evo - 2026-10-03 - I2C ENABLE-SWAP FAILURES: two more 4-BIT FIELDS, not eight flags. Same
// shift/mask shape as the pivot assist above and for the same reason - so nobody can test them with
// the single-bit == pattern bits 0-18 use.
// Reader: value = (fm_gate_flags & FM_LOG_GATE_SWAP_FAIL_CH0_MASK) >> FM_LOG_GATE_SWAP_FAIL_CH0_SHIFT.
// WHY THEY RIDE IN HERE RATHER THAN BECOMING A COLUMN: appending to the base record would move every
// field above byte 62 and break offsetof(VescLogDataL5, rider_lat) == 90, which forces
// LOG_FILE_FORMAT_VER 2 -> 3 and makes EVERY EXISTING LOG FILE undecodable - see the restated rule
// at the format-version define below. Appending to the L5 tail would avoid the bump but would cover
// level 5 only. These eight bits were free, cost ZERO bytes, need NO format bump, and are present at
// levels 4 AND 5. Bit 31 was left free here; the 2026-10-05 front-station integration took it for FM_LOG_GATE_AIM_OUTWARD.
#define FM_LOG_GATE_SWAP_FAIL_CH0_SHIFT 23           // bits 23-26: FAILED ch0 enable swaps since the previous log row, saturating at 15
#define FM_LOG_GATE_SWAP_FAIL_CH0_MASK  (0xFUL << FM_LOG_GATE_SWAP_FAIL_CH0_SHIFT)
#define FM_LOG_GATE_SWAP_FAIL_CH1_SHIFT 27           // bits 27-30: the same for ch1
#define FM_LOG_GATE_SWAP_FAIL_CH1_MASK  (0xFUL << FM_LOG_GATE_SWAP_FAIL_CH1_SHIFT)
// V2.5-Evo - 2026-10-02 - P2: bit 31 (was bit 23 on the fm-stations-front branch - MOVED at the
// 2026-10-05 integration onto SW36, because the I2C enable-swap fix had already claimed bits 23-30
// for its two 4-bit failure counters and left bit 31 as the only free bit. Left at 23 it would have
// OR-ed a 1 into the ch0 swap-failure count every tick the aim was outward, so the log would have
// reported enable-swap failures that never happened.) The PG-4 escape is standing this tick - a FRONT station is
// commanded but the buggy is not wide enough of the rider's line on that side to prove the aim line
// clear, so the aim is an OUTWARD waypoint and both the closing allowance and the convergence fade
// are withdrawn. Read it alongside bit 9 (fade bypass) and bit 10 (transit): 9 and 31 are mutually
// exclusive by construction, so a row with both set is a bug. New bit in the existing u32 - the log
// record size does not change and no column is added.
#define FM_LOG_GATE_AIM_OUTWARD        (1UL << 31)

struct __attribute__((packed)) VescLogDataL4 {
    VescLogData base;              // the complete level-3 record, unchanged and first — do not reorder
    uint8_t  gps_sent_per_s;       // complete NMEA sentences parsed in the last full second (saturates at 255)
    uint8_t  cog_frozen_s;         // seconds since the COG VALUE last changed. SENTINEL 255 = no COG value has ever been seen this session. 254 = 254 s or longer.
    uint16_t mux_err_cnt;          // running session total of setUartMux() I2C read-back mismatches (saturates at 0xFFFE)
    uint16_t loop_max_ms;          // worst loop() body duration since the PREVIOUS record, in ms, rounded to nearest; reset to 0 after every record. 0 = no loop completed since the last record, or every loop was under 0.5 ms.
    // ---- V2.5-Evo - 2026-09-17 - Follow-Me audit block (see the comment above). Byte 65 onward. ----
    uint32_t fm_gate_flags;        // FM_LOG_GATE_* bits, the controller's verdicts on this tick; 0 on ticks that never reached the condition block
    uint16_t fm_distance_dx10;     // buggy-to-rider distance x 10 m as the controller measured it; 0xFFFF = not trustworthy this tick (a fault condition; since P1-a the trigger no longer gates it)
    uint16_t fm_d_engage_dx10;     // the engage distance x 10 m in force this tick (manual or auto, after the tow-rope floor); 0xFFFF = not evaluated
    uint16_t fm_rider_speed_dx10;  // the rider's filtered speed x 10 km/h (fm_rider_speed_kmh)
    uint8_t  fm_sep_fix_count;     // rider GPS fixes counted toward the separation dwell (DWELL-1); OUR counter, not a dwell time
    uint8_t  fm_mode;              // fm_mode_runtime: 1-5 declared (4/5 = front stations), 0 off, 0xFF never declared this session  // V2.5-Evo - 2026-10-06 - was "1-3 declared"
    uint8_t  fm_state;             // FmState: 0 IDLE, 1 ARMED, 2 ACTIVE, 3 HOLD, 4 STOPPING, 5 RETURN (2026-09-19)
    uint8_t  fm_block_reason;      // FmStopReason (P0-e): the live stop latch, non-zero for the whole FM_STOPPING ramp; 0 = no fault stop in progress
    uint8_t  fm_throttle_cap;      // FM's subtract-only throttle cap this tick (0-255; 255 = no cap)
    int16_t  fm_station_deg_x10;   // P2 live station angle x 10 deg (0 = behind the rider, + = his right); written since the 2026-10-02 station work  // V2.5-Evo - 2026-10-06 - comment corrected (was "always 0")
    uint8_t  fm_return_reason;     // V2.5-Evo - 2026-09-19 - DEEP LOG (A): was fm_pad (always 0). FmReturnReason (RTMState.ino): why the last
                                   // RETURN candidate / RETURN ended. STICKY - a straight copy of fm_return_last_reason every tick, never
                                   // cleared: 0 = no event since boot; during a RETURN leg it reads 4 (ENTERED); the row where fm_state
                                   // leaves 5 carries the exit reason (6 arrived, 7 cancelled, 8 timeout, 9 fault, 10 left, 11 steered);
                                   // 1-3 = a candidate dropped before RETURN. Changes only on an event - read it at the row where the
                                   // state changes. Same offset, same size: the block stays 18 B / the record 83 B.
    // ---- V2.5-Evo - 2026-09-19 - DEEP LOG (B)+(C): four bytes appended, byte 83 onward. 83 -> 87 B. ----
    uint16_t fm_rider_raw_dx10;    // (B) the rider's RAW displacement speed x 10 km/h (fm_rider_raw_kmh: distance between two
                                   //     distinct rider fixes >= 1 s apart, no filter, no extrapolation) - the number the FM_RETURN
                                   //     candidate is judged on. SENTINEL 0xFFFF = unknown (no baseline yet, no rider fix for 3 s,
                                   //     or no meta-packet ever); saturates at 0xFFFE. fm_rider_speed_dx10 above is the FILTERED track.
    uint8_t  motor0_cmd;           // (C) g_motor0_cmd: motor 0 command out of the differential mixer, 0-255 counts, post-mixer pre-map
    uint8_t  motor1_cmd;           // (C) g_motor1_cmd: motor 1 command, same scale. Both 0 on a non-diff steering_type.
};
static_assert(sizeof(VescLogDataL4) == 90, "VescLogDataL4 size mismatch — expected 62 (VescLogData, which carries the 3 B M-2 motor-gate block since 2026-10-01) + 6 (level-4 diagnostics) + 18 (Follow-Me audit block, 2026-09-17) + 2 (fm_rider_raw_dx10) + 2 (motor0_cmd, motor1_cmd) (2026-09-19).");  // 83 -> 87: +fm_rider_raw_dx10 (u16) +motor0_cmd +motor1_cmd (u8 x 2), appended 2026-09-19

// ============================================================
// V2.5-Evo - 2026-09-19 - LEVEL-5 ("Everything") LOG RECORD - 109 B
//
// Owner request for the first water sessions with auto-return and the pivot boost: "dump anything
// you need into log 5; sessions will not be ultra long". Tiers stay ADDITIVE: the first 87 bytes are
// a byte-identical VescLogDataL4, so every level-4 column decodes with the same code, and the 22-byte
// level-5 block is appended after it. Like the level-4 Follow-Me block it is PUBLISHED by the
// controller (fmPublishLogSnapshot(), loop task, one writer, once per 10 Hz tick) and COPIED by the
// logger; nothing in the control path reads any of it. Packed like the other records; the file
// header's record_size (109) is what a reader steps by.
//
// WHAT THE BLOCK LETS A READER REBUILD:
//   rider_lat / rider_lng + the buggy's latitude/longitude in the base record -> the bearing to the
//   rider, the station angle (rider course from consecutive rider positions), and the geometry of
//   every align / return decision; rider_fix_seq / rider_fix_age -> whether that position was a
//   NEW fix or a repeat, and how old; rtm_phase + rtm_approach_cap -> which branch classic RTM took
//   and the cap it wrote; align_cap / align_influence / mix_influence -> the throttle cap and the
//   motor split the pivot boost actually asked for; fm_return_override / fm_flags_sent /
//   fm_keepalive_age -> what the remote declared and what the RX echoed back.
//
// BYTE OFFSETS (from the start of the record; the Python reader is built from this table):
//    87  rider_lat            f32   rider (TX) latitude, degrees, as the RX holds it (rx_tx_gps_lat cast to float:
//                                   ~0.5 m resolution at mid latitudes, the same precision the base record's own
//                                   latitude/longitude carry). 0.0 with rider_fix_age 0xFFFF = no meta-packet ever
//    91  rider_lng            f32   rider (TX) longitude, degrees
//    95  rider_fix_seq        u16   rx_tx_gps_fix_seq & 0xFFFF - bumps once per DISTINCT rider position (wraps)
//    97  rider_fix_age_div10  u16   (now - rx_tx_gps_timestamp) / 10 ms; 0xFFFF = never received; saturates 0xFFFE
//    99  rtm_approach_cap     u8    rtm_approach_cap as published (0-255; 255 = no cap)
//   100  rtm_phase            u8    RTM branch taken this tick: 0 inactive/disabled, 1 align (Phase 1), 2 run
//                                   (aligned; governor or free), 3 approach (aligned AND inside rtm_approach_zone_m,
//                                   the decel ramp is capping), 4 bootstrap-headless (no heading: BOOTSTRAP-1 or its
//                                   declined fallback), 5 gate-stop (a gate 2-7 failed: rtm_rx_emergency_stop),
//                                   6 trigger released (gate 1: armed, motor already 0), 7 Gate 9 handoff (one tick)
//   101  align_cap            u8    fmAlignCapValue() while fm_gate_flags bit 17 (aligning) is set, else 0
//   102  align_influence      u8    align_mixer_influence_override as published this tick (0 = no boost)
//   103  mix_influence        u8    the steering influence the differential mixer used this tick, %: the boost
//                                   when calcPWM() honours it (auto-steer with the trigger held), else
//                                   steering_influence - the same rule calcPWM() applies, from the same inputs
//   104  fm_return_override   u8    the remote's 0xF2 auto-return override: 0 none (stored default), 1 OFF, 2 ON
//   105  fm_flags_sent        u8    telemetry.fm_flags as sent to the remote ([7] effective auto-return, [3] fault
//                                   sticky, [2] armed-not-ready, [1] engaged, [0] armed)
//   106  fm_keepalive_age_div100 u8 (now - fm_mode_last_rx_ms) / 100 ms since the last 0xF2; 0xFF = none this
//                                   session; saturates 0xFE (25.4 s; the keepalive is every 30 s, expiry 95 s)
//   107  l5_rsvd_takeover_active u8 RESERVED, 0 here: steer_takeover_active for the steer-takeover branch
//   108  l5_rsvd_takeover_end    u8 RESERVED, 0 here: that branch's last_end code
//   109 = sizeof
// NOT INCLUDED, and why: the raw 0xF2 byte (Radio.ino decodes it into fm_mode_runtime and
// fm_return_mode_runtime and keeps no copy - both decoded values are here: fm_mode in the level-4
// block, fm_return_override above); a compass yaw rate (Compass.ino has no per-tick heading history
// to derive one from cheaply - derive it from compass_live_dx10 across consecutive rows instead).
// CAPACITY on the 1757 KB the filesystem reports (the logger keeps a 500 KB reserve on top): about
// 1 h 30 min at 3 Hz, about 55 min at 5 Hz. Level 4 at 87 B: about 1 h 55 min / 1 h 10 min.
// ============================================================
struct __attribute__((packed)) VescLogDataL5 {
    VescLogDataL4 l4;                    // the complete 87 B level-4 record, unchanged and first — do not reorder
    // ---- level-5 block, byte 87 onward (22 B) ----
    float    rider_lat;                  // rider latitude, deg (rx_tx_gps_lat)
    float    rider_lng;                  // rider longitude, deg (rx_tx_gps_lng)
    uint16_t rider_fix_seq;              // rx_tx_gps_fix_seq & 0xFFFF
    uint16_t rider_fix_age_div10;        // ms / 10 since the last meta-packet; 0xFFFF = never
    uint8_t  rtm_approach_cap;           // 0-255; 255 = no cap
    uint8_t  rtm_phase;                  // 0-7, see the table above
    uint8_t  align_cap;                  // 0 when not aligning
    uint8_t  align_influence;            // 0 = no boost published
    uint8_t  mix_influence;              // %, what the mixer used
    uint8_t  fm_return_override;         // 0 none / 1 OFF / 2 ON
    uint8_t  fm_flags_sent;              // telemetry.fm_flags
    uint8_t  fm_keepalive_age_div100;    // ms / 100; 0xFF = none
    uint8_t  l5_rsvd_takeover_active;    // RESERVED (takeover branch), 0
    uint8_t  l5_rsvd_takeover_end;       // RESERVED (takeover branch), 0
};
static_assert(sizeof(VescLogDataL5) == 112, "VescLogDataL5 size mismatch — expected 90 (VescLogDataL4) + 22 (level-5 block, 2026-09-19).");
static_assert(offsetof(VescLogDataL5, rider_lat) == 90, "level-5 block must start right after the 90 B level-4 record");

// ============================================================
// V2.5-Evo - 2026-09-17 - FmLogSnapshot: the controller -> logger hand-off
//
// runFmLoop() (loop task, 10 Hz) fills g_fm_log_snapshot exactly once per tick, inside
// taskENTER_CRITICAL(&g_fm_log_mux); fillLevel4Diag() (loggerTask) copies it out inside the same
// critical section. The two tasks share one core at the same priority and are time-sliced, so
// without the critical section an 18-byte copy could be torn mid-way and a row would mix two
// ticks. The section is a handful of instructions long — no I/O, no computation inside it.
// The logger NEVER recomputes a gate from raw inputs: what it writes is what the controller
// decided. Control code reads nothing from this struct.
// ============================================================
struct FmLogSnapshot {
    uint32_t gate_flags;
    uint16_t distance_dx10;
    uint16_t d_engage_dx10;
    uint16_t rider_speed_dx10;
    uint8_t  sep_fix_count;
    uint8_t  mode;
    uint8_t  state;
    uint8_t  block_reason;
    uint8_t  throttle_cap;
    int16_t  station_deg_x10;
    uint8_t  return_reason;    // V2.5-Evo - 2026-09-19 - DEEP LOG (A): fm_return_last_reason, sticky (see VescLogDataL4.fm_return_reason)
    uint16_t rider_raw_dx10;   // V2.5-Evo - 2026-09-19 - DEEP LOG (B): fm_rider_raw_kmh x 10; 0xFFFF = unknown
    // V2.5-Evo - 2026-09-19 - the level-5 block, same fields and sentinels as VescLogDataL5 (copied
    // by fillLevel5Extra() only when the file is level 5; published on every tick regardless).
    float    l5_rider_lat;
    float    l5_rider_lng;
    uint16_t l5_rider_fix_seq;
    uint16_t l5_rider_fix_age_div10;
    uint8_t  l5_rtm_approach_cap;
    uint8_t  l5_rtm_phase;
    uint8_t  l5_align_cap;
    uint8_t  l5_align_influence;
    uint8_t  l5_mix_influence;
    uint8_t  l5_fm_return_override;
    uint8_t  l5_fm_flags_sent;
    uint8_t  l5_fm_keepalive_age_div100;
};
FmLogSnapshot g_fm_log_snapshot = { 0, 0xFFFF, 0xFFFF, 0, 0, 0xFF, 0, 0, 255, 0, 0, 0xFFFF,
                                    0.0f, 0.0f, 0, 0xFFFF, 255, 0, 0, 0, 0, 0, 0, 0xFF };
portMUX_TYPE  g_fm_log_mux      = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// V2.5-Evo - 2026-07-25 - STAGE 0 PART B: SELF-DESCRIBING LOG FILE HEADER
//
// WHY THIS EXISTS: different log levels write different record sizes, so a log file that does
// not say what it is cannot be parsed — a reader that assumes sizeof(VescLogData) would walk
// straight off the record boundary and emit convincing garbage. Every log file now opens with
// this 8-byte header, and BOTH readers (the serial ?download path in Logger.ino and the WiFi
// /api/logs/download path in Common/WebConfigEngine.h) read it and parse accordingly.
//
// 8 bytes exactly, naturally aligned, no padding: uint32 + uint8 + uint8 + uint16. The first
// record therefore starts at file offset 8, which is still 4-byte aligned.
//
// OLD LOGS: files written before this change have no header. Their first 4 bytes are a millis()
// timestamp, which will not equal the magic, so both readers detect the missing magic and say so
// in plain language instead of emitting garbage. Those files were already undecodable after the
// 53 -> 59 byte record change (F9, 2026-07-24) — this just makes the failure honest.
// ============================================================
#define LOG_FILE_MAGIC       0x474C5242UL  // little-endian bytes on disk read "BRLG" (BREmote Log)
// V2.5-Evo - 2026-10-01 - M-2: bumped 1 -> 2, and the rule above it corrected. The old rule said
// "bump ONLY if the header layout itself changes", on the assumption that record_size alone could
// always describe a record. It cannot: the M-2 motor-gate block was appended to the BASE record, so
// every block above byte 59 of a level-4/5 record moved 3 bytes. record_size still says how far to
// STEP, but nothing in a format-version-1 file says which GENERATION of layout its bytes follow,
// and the pre-flash sizes (59/65/83/85/87/109) sit inside the new accepted range, so both readers
// would have taken an old level-4/5 file and silently mis-decoded every column above byte 59 -
// precisely the convincing garbage this self-describing header was created to prevent. Both readers
// (the serial ?download path in Logger.ino and the WiFi /api/logs/download path in
// Common/WebConfigEngine.h) ALREADY test format_ver against this constant and refuse a mismatch in
// plain English, so the bump makes the failure honest with no new logic at either reader.
// THE RULE, restated: bump this when the header layout changes OR when the RECORD layout changes in
// a way that moves an existing field - i.e. any time an older file's bytes can no longer be decoded
// by this build. Appending at the tail of the LARGEST record does not qualify; appending to the base
// record does. Pre-flash logs must be downloaded BEFORE flashing a build that bumps this.
#define LOG_FILE_FORMAT_VER  2             // 2 = M-2 motor-gate block in the base record (2026-10-01); 1 = the original STAGE 0 PART B layout (2026-07-25)

struct __attribute__((packed)) LogFileHeader {
    uint32_t magic;        // LOG_FILE_MAGIC — absent/mismatched means "not a BREmote log of this era"
    uint8_t  format_ver;   // LOG_FILE_FORMAT_VER — layout of THIS header
    uint8_t  log_level;    // the level the file was actually recorded at (3, 4 or 5 since 2026-09-19)
    uint16_t record_size;  // bytes per record in this file — the ONLY thing a reader may step by
};
static_assert(sizeof(LogFileHeader) == 8, "LogFileHeader must stay 8 bytes — readers step past it by sizeof().");

// ============================================================
// logResolveLevel - turn the stored config value into the level actually used
//
// Inputs:  usrConf.log_level. Outputs: 3, 4 or 5. Side effects: none.
//
// 0 (unset), 1 (Basic), 2 (VESC), 3 (Developer) and ANY out-of-range value all resolve to 3.
// V2.5-Evo - 2026-09-19 - 5 (Everything) resolves to 5.
// Levels 1 and 2 are reserved for a future storage optimisation (smaller records); they are
// accepted by the config validator so a rider can select them and a later firmware will honour
// them, but until those records exist they are documented — here, in the field comment, and in
// all three config UIs — as logging at level 3 rather than being silently dropped.
// ============================================================
static inline uint8_t logResolveLevel()
{
  if (usrConf.log_level == 5) return 5;
  return (usrConf.log_level == 4) ? 4 : 3;
}

// ============================================================
// logRecordSizeForLevel - bytes per record for a given level
// Inputs: level (3, 4 or 5). Outputs: record size in bytes. Side effects: none.
// ============================================================
static inline uint16_t logRecordSizeForLevel(uint8_t level)
{
  if (level >= 5) return (uint16_t)sizeof(VescLogDataL5);   // V2.5-Evo - 2026-09-19 - level 5, 109 B
  return (level >= 4) ? (uint16_t)sizeof(VescLogDataL4) : (uint16_t)sizeof(VescLogData);
}

// ============================================================
// V2.5-Evo - 2026-07-25 - STAGE 0 PART B: ONE CSV DEFINITION, TWO READERS
//
// The serial ?download path and the WiFi /api/logs/download path must emit the same columns,
// the same values, the same scaling and the same N/A sentinels. That parity was only just
// repaired (F-WEBCSV, 2026-07-25) after the WiFi path silently dropped 5 columns for months.
// Rather than keep two hand-synchronised copies and a comment asking future editors to be
// careful, the column list, the row format and the row FORMATTER now exist exactly once and
// both readers call them. Divergence is now structurally impossible, not merely discouraged.
//
// If you add a column: extend LOG_CSV_HEADER_L3 (or _L4), extend the matching format string,
// and add the argument in logFormatCsvRow(). Both readers pick it up with no further edits.
// ============================================================
#define LOG_CSV_HEADER_L3 "timestamp_ms,motor_current_A,battery_current_A,duty_cycle_%,voltage_V,ERPM,temp_mos_C,fault_code,speed_kmh,latitude,longitude,datetime_unix,thr_received,rtm_source,rtm_confidence,rtm_rx_active,gps_phase_b_ok,rtm_steer_override,rtm_heading_chosen_dx10,compass_live_dx10,compass_snap_dx10,snap_age_s,gps_course_dx10,cog_age_ms_div10,heading_error_dx10,d_error_dx10,remote_error,effective_steer,tx_distance_m,rssi_dbm,snr_db,ctrl_pkt_age_ms,motor_gate_open"
// V2.5-Evo - 2026-09-17 - two level-4 column sets: _L4_DIAG is the 65-byte layout written from
// 2026-07-25 to 2026-09-16 (four diagnostics); _L4 is the current 83-byte layout (diagnostics +
// the Follow-Me audit block). logCsvHeaderFor() picks by the file's own record_size.
#define LOG_CSV_HEADER_L4_DIAG LOG_CSV_HEADER_L3 ",gps_sent_per_s,cog_frozen_s,mux_err_cnt,loop_max_ms"
// V2.5-Evo - 2026-09-19 - DEEP LOG (A): +fm_return_reason (the sticky FmReturnReason latch), +fm_aligning and +fm_boost
// (bits 17 / 18 of fm_gate_flags, read out as their own 0/1 columns so a reader never has to mask the flag word).
// V2.5-Evo - 2026-09-19 - DEEP LOG (B)+(C): three tiers above DIAG now, each its own macro, each selected by the
// file's record_size: _L4_83 is the 83 B layout written from 2026-09-17 to the (B)+(C) bump (48 columns), _L4_RAW
// adds fm_rider_raw_kmh (85 B, never shipped on its own but a reader must still be able to name it), _L4 is the
// current 87 B layout with motor0_cmd / motor1_cmd (51 columns).
#define LOG_CSV_HEADER_L4_83 LOG_CSV_HEADER_L4_DIAG ",fm_gate_flags,fm_distance_m,fm_d_engage_m,fm_rider_speed_kmh,fm_sep_fix_count,fm_mode,fm_state,fm_block_reason,fm_throttle_cap,fm_station_deg,fm_return_reason,fm_aligning,fm_boost"
#define LOG_CSV_HEADER_L4_RAW LOG_CSV_HEADER_L4_83 ",fm_rider_raw_kmh"
#define LOG_CSV_HEADER_L4 LOG_CSV_HEADER_L4_RAW ",motor0_cmd,motor1_cmd"
// V2.5-Evo - 2026-09-19 - level 5: the 14 level-5 columns after the full level-4 set (65 columns, 109 B files).
#define LOG_CSV_HEADER_L5 LOG_CSV_HEADER_L4 ",rider_lat,rider_lng,rider_fix_seq,rider_fix_age_ms,rtm_approach_cap,rtm_phase,align_cap,align_influence,mix_influence,fm_return_override,fm_flags_sent,fm_keepalive_age_s,l5_rsvd_takeover_active,l5_rsvd_takeover_end"

// V2.5-Evo - 2026-10-01 - M-2: 31 level-3 columns -> 33 (+ctrl_pkt_age_ms, +motor_gate_open).
#define LOG_CSV_ROW_FMT_L3 "%u,%.2f,%.2f,%d,%.1f,%d,%u,%u,%.1f,%.6f,%.6f,%u,%u,%u,%u,%u,%u,%u,%d,%u,%u,%u,%u,%u,%d,%d,%u,%u,%.1f,%d,%.1f,%u,%u"
#define LOG_CSV_ROW_EXT_L4 ",%u,%u,%u,%u"
#define LOG_CSV_ROW_EXT_L4_FM ",%u,%.1f,%.1f,%.1f,%u,%u,%u,%u,%u,%.1f,%u,%u,%u"
#define LOG_CSV_ROW_EXT_L4_RAW ",%.1f"        // (B) fm_rider_raw_kmh; -1.0 = unknown
#define LOG_CSV_ROW_EXT_L4_MOTORS ",%u,%u"    // (C) motor0_cmd, motor1_cmd
#define LOG_CSV_ROW_EXT_L5 ",%.6f,%.6f,%u,%d,%u,%u,%u,%u,%u,%u,%u,%.1f,%u,%u"   // level 5: lat, lng, seq, age ms (-1 never), cap, phase, align cap, align infl, mix infl, return override, fm_flags, keepalive s (-1.0 none), rsvd x 2

// Row buffer size. Sizing arithmetic for the 31 level-3 columns is unchanged from F-WEBCSV:
//   ~178 field chars + 30 commas + newline + NUL = ~210 bytes for normal data, and a corrupt
//   latitude/longitude printed via "%.6f" can reach ~282. The 4 level-4 diagnostic columns add
//   at most 3+3+5+5 chars plus 4 commas = 20. The 10 Follow-Me columns (2026-09-17) add at most
//   ~60 more (a u32 flag word, three "%.1f" distances/speeds, five u8s, one signed "%.1f"); the
//   three 2026-09-19 (A) columns (a u8 and two 0/1 flags) at most 9 more, and the (B)+(C) columns (one
//   "%.1f" speed, two u8s) at most 15 more. The 14 level-5 columns add at most ~80 (two "%.6f"
//   coordinates, one "%.1f", eleven small integers, 14 commas). The two 2026-10-01 motor-gate
//   columns (a 5-digit capped age and a single 0/1) add at most 8: pathological total ~478. 640
//   clears the pathological ~362 by ~1.8x. It is a stack local in the Arduino loop task (8 KB
//   stack), which is where both readers run.
#define LOG_CSV_ROW_BUF 640

// ============================================================
// V2.5-Evo - 2026-09-17 - logCsvHeaderFor - the column header that matches a file's own layout
// Inputs: level and record_size, both from the file's LogFileHeader. Outputs: the header string.
// Side effects: none. Both readers (serial ?download, WiFi /api/logs/download) call this so a
// 65-byte level-4 file written before the Follow-Me block still gets exactly its 35 columns.
// V2.5-Evo - 2026-09-19 - DEEP LOG (B)+(C) TIER FIX: the second test used to be
// record_size < sizeof(VescLogDataL4), which was right only while sizeof was the ONE size above DIAG.
// After the 83 -> 87 bump every 83 B file on the board would have printed with the 35-column DIAG
// header - 13 columns short of its own contents. Each tier is now bounded by the OFFSET where the
// next block starts, so the record size is matched to the columns that are actually present and a
// future bump cannot re-label an older file. Order: 65 DIAG, 83 L4_83, 85 +raw, 87 full.
// ============================================================
static inline const char* logCsvHeaderFor(uint8_t level, uint16_t record_size)
{
  if (level < 4 || record_size < (uint16_t)offsetof(VescLogDataL4, fm_gate_flags)) return LOG_CSV_HEADER_L3;
  if (record_size < (uint16_t)offsetof(VescLogDataL4, fm_rider_raw_dx10))                              return LOG_CSV_HEADER_L4_DIAG;  // 65 B: diagnostics only
  if (record_size < (uint16_t)(offsetof(VescLogDataL4, fm_rider_raw_dx10) + sizeof(uint16_t)))         return LOG_CSV_HEADER_L4_83;    // 83 B: + Follow-Me block
  if (record_size < (uint16_t)(offsetof(VescLogDataL4, motor1_cmd) + sizeof(uint8_t)))                 return LOG_CSV_HEADER_L4_RAW;   // 85 B: + raw rider speed
  if (record_size < (uint16_t)sizeof(VescLogDataL5))                                                    return LOG_CSV_HEADER_L4;       // 87 B: + motor commands
  return LOG_CSV_HEADER_L5;                                                                                                             // 109 B: + the level-5 block (2026-09-19)
}

// ============================================================
// logFormatCsvRow - format ONE binary log record as one CSV line
//
// What it does:
//   Decodes rec_bytes (raw bytes straight off SPIFFS) into the level-3 fields, formats them,
//   and — when the file is level 4 and the record is big enough to contain it — appends the
//   4-column level-4 diagnostic block. Always terminates the line with a single '\n' and a NUL.
//
// Inputs:
//   out       - destination buffer (use LOG_CSV_ROW_BUF bytes)
//   out_len   - size of that buffer
//   rec_bytes - one raw record as read from the file, at least sizeof(VescLogData) bytes
//   rec_size  - bytes actually read for this record, taken from the FILE HEADER, never sizeof()
//   level     - log level from the file header (3, 4 or 5)
//
// Outputs: number of characters written (excluding the NUL); 0 on a bad argument.
// Side effects: none — reads nothing global, writes only into out.
//
// The record is copied with memcpy rather than cast in place: the caller's buffer is a byte
// array with no guaranteed alignment, and these structs are packed.
// ============================================================
static int logFormatCsvRow(char* out, size_t out_len, const uint8_t* rec_bytes, uint16_t rec_size, uint8_t level)
{
  if (out == NULL || out_len < 2) return 0;
  if (rec_bytes == NULL || rec_size < (uint16_t)sizeof(VescLogData)) { out[0] = '\0'; return 0; }

  VescLogData d;
  memcpy(&d, rec_bytes, sizeof(VescLogData));

  int n = snprintf(out, out_len, LOG_CSV_ROW_FMT_L3,
                   d.timestamp,
                   d.current_motor / 100.0f,
                   d.current_battery / 100.0f,
                   (int16_t)d.duty_cycle,
                   d.voltage / 10.0f,
                   (int32_t)d.ERPM * 10,
                   d.temp_mos,
                   d.fault_code,
                   d.speed / 10.0f,
                   d.latitude,
                   d.longitude,
                   d.datetime,
                   (unsigned)d.thr_received_log,
                   (unsigned)d.rtm_source,
                   (unsigned)d.rtm_confidence,
                   (unsigned)d.rtm_rx_active_log,
                   (unsigned)d.gps_phase_b_ok_log,
                   (unsigned)d.rtm_steer_override_log,
                   (int)d.rtm_heading_chosen_dx10,
                   (unsigned)d.compass_live_dx10,
                   (unsigned)d.compass_snap_dx10,
                   (unsigned)d.snap_age_s,
                   (unsigned)d.gps_course_dx10,
                   (unsigned)d.cog_age_ms_div10,
                   // Bundle 1: heading controller tuning columns (0x7FFF = no data sentinel)
                   (int)d.heading_error_dx10,
                   (int)d.d_error_dx10,
                   // E7 Fix: BREmote remote_error code (0 = none, 71 = E71 water ingress)
                   (unsigned)d.error_code_log,
                   // FM triage: steering byte actually applied by calcPWM() (vs commanded rtm_steer_override)
                   (unsigned)d.effective_steer_log,
                   // F9: distance (m) + link quality. N/A → distance -1.0, rssi -999, snr -99.0
                   (d.tx_distance_dx10 == 0xFFFF) ? -1.0f : (d.tx_distance_dx10 / 10.0f),
                   (d.rssi_dbm == 0x7FFF) ? -999 : (int)d.rssi_dbm,
                   (d.snr_dx10 == 0x7FFF) ? -99.0f : (d.snr_dx10 / 10.0f),
                   // V2.5-Evo - 2026-10-01 - M-2: the motor gate. ctrl_pkt_age_ms is how stale the
                   // last THROTTLE command was on this row (0xFFFE = capped, >= ~65.5 s); it has no
                   // N/A sentinel because it is always a real measurement - before the first control
                   // packet it simply reads as uptime, the same thing the gate itself sees.
                   // motor_gate_open is 1 = OPEN (pulses reaching the ESCs) / 0 = SHUT. Read them
                   // together: a 0 with a large age is a link failsafe, a 0 with a small age is
                   // PWM_active being down, and a 0 of any kind means the motor columns on this row
                   // describe arithmetic that never left the board.
                   (unsigned)d.ctrl_pkt_age_ms,
                   (unsigned)d.motor_gate_open);

  if (n < 0) { out[0] = '\0'; return 0; }
  if ((size_t)n >= out_len) n = (int)out_len - 1;   // snprintf truncated — keep the index inside the buffer

  // Level-4 blocks. Guarded on the RECORD SIZE as well as the level so a truncated or
  // mislabelled file can never make us read past the bytes we actually have.
  // V2.5-Evo - 2026-09-17 - two blocks now: the four diagnostics (present from byte 59, i.e. in
  // every 65 B or 83 B level-4 file) and the Follow-Me audit block (present only in 83 B files).
  // Only rec_size bytes are copied, into a zeroed struct, so a 65 B record never reads stale
  // buffer bytes as Follow-Me columns.
  if (level >= 4 && rec_size >= (uint16_t)offsetof(VescLogDataL4, fm_gate_flags) && (size_t)n < (out_len - 1))
  {
    VescLogDataL4 d4;
    memset(&d4, 0, sizeof(d4));
    memcpy(&d4, rec_bytes, (rec_size < (uint16_t)sizeof(VescLogDataL4)) ? rec_size : (uint16_t)sizeof(VescLogDataL4));
    int m = snprintf(out + n, out_len - (size_t)n, LOG_CSV_ROW_EXT_L4,
                     (unsigned)d4.gps_sent_per_s,
                     (unsigned)d4.cog_frozen_s,
                     (unsigned)d4.mux_err_cnt,
                     (unsigned)d4.loop_max_ms);
    if (m > 0)
    {
      n += m;
      if ((size_t)n >= out_len) n = (int)out_len - 1;
    }
    // V2.5-Evo - 2026-09-19 - DEEP LOG (B)+(C): the Follow-Me block is present when the record reaches
    // the byte after it (the offset of fm_rider_raw_dx10, 83) - NOT sizeof(), which is 87 now and
    // would have dropped the whole block for every 83 B file. The (B) and (C) fields are each guarded
    // on their own offset + size, so 83 / 85 / 87 B records print exactly the columns they carry.
    if (rec_size >= (uint16_t)offsetof(VescLogDataL4, fm_rider_raw_dx10) && (size_t)n < (out_len - 1))
    {
      // Follow-Me audit block. N/A distances print as -1.0 (same convention as tx_distance_m).
      int k = snprintf(out + n, out_len - (size_t)n, LOG_CSV_ROW_EXT_L4_FM,
                       (unsigned)d4.fm_gate_flags,
                       (d4.fm_distance_dx10 == 0xFFFF) ? -1.0f : (d4.fm_distance_dx10 / 10.0f),
                       (d4.fm_d_engage_dx10 == 0xFFFF) ? -1.0f : (d4.fm_d_engage_dx10 / 10.0f),
                       d4.fm_rider_speed_dx10 / 10.0f,
                       (unsigned)d4.fm_sep_fix_count,
                       (unsigned)d4.fm_mode,
                       (unsigned)d4.fm_state,
                       (unsigned)d4.fm_block_reason,
                       (unsigned)d4.fm_throttle_cap,
                       d4.fm_station_deg_x10 / 10.0f,
                       // DEEP LOG (A) 2026-09-19: the sticky return reason, then bits 17/18 of the flag word as 0/1
                       (unsigned)d4.fm_return_reason,
                       (unsigned)((d4.fm_gate_flags & FM_LOG_GATE_ALIGNING) ? 1 : 0),
                       (unsigned)((d4.fm_gate_flags & FM_LOG_GATE_BOOST)    ? 1 : 0));
      if (k > 0)
      {
        n += k;
        if ((size_t)n >= out_len) n = (int)out_len - 1;
      }
    }
    // (B) raw rider speed: present from 85 B. 0xFFFF (unknown) prints as -1.0.
    if (rec_size >= (uint16_t)(offsetof(VescLogDataL4, fm_rider_raw_dx10) + sizeof(uint16_t)) && (size_t)n < (out_len - 1))
    {
      int r = snprintf(out + n, out_len - (size_t)n, LOG_CSV_ROW_EXT_L4_RAW,
                       (d4.fm_rider_raw_dx10 == 0xFFFF) ? -1.0f : (d4.fm_rider_raw_dx10 / 10.0f));
      if (r > 0)
      {
        n += r;
        if ((size_t)n >= out_len) n = (int)out_len - 1;
      }
    }
    // (C) the two mixer outputs: present from 87 B.
    if (rec_size >= (uint16_t)(offsetof(VescLogDataL4, motor1_cmd) + sizeof(uint8_t)) && (size_t)n < (out_len - 1))
    {
      int c = snprintf(out + n, out_len - (size_t)n, LOG_CSV_ROW_EXT_L4_MOTORS,
                       (unsigned)d4.motor0_cmd, (unsigned)d4.motor1_cmd);
      if (c > 0)
      {
        n += c;
        if ((size_t)n >= out_len) n = (int)out_len - 1;
      }
    }
    // V2.5-Evo - 2026-09-19 - the level-5 block: present only in a full 109 B record. Sentinels print
    // as -1 (fix age never received) / -1.0 (no keepalive this session), the same convention as above.
    if (rec_size >= (uint16_t)sizeof(VescLogDataL5) && (size_t)n < (out_len - 1))
    {
      VescLogDataL5 d5;
      memcpy(&d5, rec_bytes, sizeof(VescLogDataL5));
      int f = snprintf(out + n, out_len - (size_t)n, LOG_CSV_ROW_EXT_L5,
                       d5.rider_lat,
                       d5.rider_lng,
                       (unsigned)d5.rider_fix_seq,
                       (d5.rider_fix_age_div10 == 0xFFFF) ? -1 : (int)d5.rider_fix_age_div10 * 10,
                       (unsigned)d5.rtm_approach_cap,
                       (unsigned)d5.rtm_phase,
                       (unsigned)d5.align_cap,
                       (unsigned)d5.align_influence,
                       (unsigned)d5.mix_influence,
                       (unsigned)d5.fm_return_override,
                       (unsigned)d5.fm_flags_sent,
                       (d5.fm_keepalive_age_div100 == 0xFF) ? -1.0f : (d5.fm_keepalive_age_div100 / 10.0f),
                       (unsigned)d5.l5_rsvd_takeover_active,
                       (unsigned)d5.l5_rsvd_takeover_end);
      if (f > 0)
      {
        n += f;
        if ((size_t)n >= out_len) n = (int)out_len - 1;
      }
    }
  }

  if ((size_t)n < (out_len - 1)) out[n++] = '\n';
  out[n] = '\0';
  return n;
}

#define ENABLE_WEB_LOG_DOWNLOAD // Enable log download endpoints

#ifdef WIFI_ENABLED
#include "../Common/WebConfigEngine.h"
#endif

#ifdef WIFI_ENABLED
void webCfgNotifyRxConnected();
#else
inline void webCfgNotifyRxConnected() {}  // No-op stub when WiFi disabled
#endif

// V2.5-Evo - 2026-05-16 - feat(telemetry): expand LoRa packet 8→19 bytes + 0xF4 aux meta-packet
//Telemetry to send, MUST BE 8-bit!!
// V2.5-Evo - 2026-04-27 - P8: rtm_distance at index 5; encoding: 0-99=tenths of m, 100-254=(value-90) whole m, 255=N/A.
struct __attribute__((packed)) TelemetryPacket {
    uint8_t foil_bat = 0xFF;          // index 0 — battery % 0-100
    uint8_t foil_temp = 0xFF;         // index 1 — FET temp degC
    uint8_t foil_speed = 0xFF;        // index 2 — speed km/h
    uint8_t error_code = 0;           // index 3 — fault flags
    uint8_t foil_power = 0xFF;        // index 4 — power (watts/50); 0xFF = N/A
    uint8_t rtm_distance = 0xFF;      // index 5 — RX→TX distance; see encoding above; 0xFF = N/A
    uint8_t foil_motor_amps = 0xFF;   // index 6 — motor current whole amps; 0xFF = N/A
    uint8_t foil_voltage = 0xFF;      // index 7 — battery voltage V×2 (0.5V res); 0xFF = N/A
    uint8_t foil_duty = 0xFF;         // index 8 — duty cycle 0-100%; 0xFF = N/A
    uint8_t foil_erpm_lo = 0xFF;      // index 9 — |ERPM|÷100 low byte; 0xFFFF when both=0xFF means N/A
    uint8_t foil_erpm_hi = 0xFF;      // index 10 — |ERPM|÷100 high byte
    uint8_t foil_wh_lo = 0xFF;        // index 11 — session Wh×10 low byte; 0xFFFF when both=0xFF means N/A
    uint8_t foil_wh_hi = 0xFF;        // index 12 — session Wh×10 high byte
    uint8_t rx_heading = 0xFF;        // index 13 — GPS COG÷2 (0-179→0-358°); 0xFF = N/A
    uint8_t fm_heading_err = 127;     // index 14 — bearing error+127; 127 = no data
    uint8_t fm_status = 0;            // index 15 — [7]=aux2_on [6]=aux1_on [5]=vesc_online [4]=rx_wetness [3:2]=heading_conf [1]=rtm_active [0]=fm_active
    uint8_t fm_flags = 0;             // index 16 — Follow-Me engagement sub-state (assembled in RTMState.ino runRtmLoop): [7]=effective auto-return mode echo (1 = ON; V2.5-Evo 2026-09-19, read by the remote's display and return gesture) [6]=reserved [5]=steer takeover STANDING this tick (the stick is steering an auto-steer run; display only) [4]=steer_during_auto echo (1 = take over: the remote's Gate 4 steer-exit stands down; 0 = cancel) - both V2.5-Evo 2026-09-19, sent every tick in every FM state [3]=fault-stop-sticky [2]=armed-not-ready [1]=engaged (FM_ACTIVE, or FM_RETURN while it moves) [0]=armed. Was reserved_tx_imu (unused reserved byte).
    uint8_t rx_bearing_to_tx = 0xFF;  // index 17 — bearing from buggy toward rider÷2; 0xFF = N/A
    uint8_t link_quality = 0;         // index 18 (must be last)
} telemetry;

/*
** FreeROTS/Task handles
*/
const int maxTasks = 10;
TaskStatus_t taskStats[maxTasks];

// Task handles
TaskHandle_t generatePWMHandle = NULL;
TaskHandle_t triggeredReceiveHandle = NULL;
TaskHandle_t checkConnStatusHandle = NULL;
extern TaskHandle_t loopTaskHandle;

// Semaphore for triggered task
SemaphoreHandle_t triggerReceiveSemaphore;

// Mutex protecting Wire/AW9523 — accessed by multiple FreeRTOS tasks that preempt each
// other on the single ESP32-C3 core: generatePWM, checkConnStatus, checkWetness,
// checkButtons, setUartMux, blinkErr, blinkBind, readCompassRaw, loggerLoop, triggerBlink.
// Created in initHardware() before Wire.begin() so startupAW() can safely use it.
SemaphoreHandle_t i2cMutex;

/*
** Variables
*/
std::atomic<bool> rfInterrupt{false};
volatile bool rxIsrState = 0;
volatile int unpairedBlink = 0;
volatile unsigned long last_packet = 0;

// ============================================================
// V2.5-Evo - 2026-09-28 - R-3 FIX: the motor gate's own timestamp.
// ============================================================
// THE BUG THIS CLOSES. last_packet answers "is the remote alive?" and it is refreshed by FIVE
// radio branches in Radio.ino: the normal control packet, and the three meta-packets 0xF1 (RTM
// state), 0xF2 (Follow-Me mode) and 0xF4 (aux). Only ONE of those five carries a throttle byte -
// thr_received has exactly one writer in the whole firmware, the control-packet branch - and
// nothing anywhere zeroes thr_received on a failsafe. But the motor gate in PWM.ino was testing
// last_packet, so it was answering the throttle question with the liveness answer.
//
// THE FAILURE THAT PRODUCES. Rider on the throttle; the link drops past usrConf.failsafe_time, so
// the gate closes and the motors cut; the rider lets go of the trigger, as anyone would. A
// meta-packet then arrives BEFORE the next control packet - the 0xF2 Follow-Me keepalive is the
// most reachable one, it repeats every 30 s whenever FM is armed, including armed while the buggy
// is being towed by hand - and it refreshes last_packet. The gate reopens on a stale thr_received
// from before the dropout, with the trigger released, and stays there until a real control packet
// lands. Worse, calcPWM() is called BEFORE the gate on every 10 ms pass, so the throttle ramp has
// already climbed back to that stale value by the time the gate opens: the motors do not ease in,
// they are already there. That is motor movement with no user throttle input, which the creator
// safety philosophy forbids outright.
//
// THE FIX, and why it is a SECOND timestamp rather than a change to the first. "The remote is
// alive" and "I have a fresh throttle command" are two different questions and the code was
// conflating them. Both are wanted - just by different readers:
//   - THIS variable is stamped ONLY in the control-packet branch (Radio.ino), and the only thing
//     that ACTS on it is the motor gate in PWM.ino. A meta-packet cannot open the motor gate, full
//     stop. V2.5-Evo - 2026-10-01 - M-2 added two READ-ONLY readers, neither in a control path:
//     cmdDiag() prints its age on ?diag's "motor gate" line, and convertToLogData() records that
//     age as the log column ctrl_pkt_age_ms. Both only subtract it from millis() and print/store
//     the result. The gate VERDICT they show comes from g_motor_gate_open, not from a second
//     evaluation of the gate test here.
//   - last_packet keeps its existing meaning and ALL FOUR of its other readers untouched:
//     Logger.ino's link flag, RTMState.ino's RTM failsafe stop and FM_STOP_LINK, and System.ino's
//     connection status. "The TX is alive" is the correct question for every one of those.
// The two alternatives were both rejected for having side effects we do not want: zeroing
// thr_received in the meta branches would stutter the throttle every time a keepalive lands during
// normal riding, and not refreshing last_packet in the meta branches would change FM/RTM stop
// timing and the logger's link flag.
//
// THE INVARIANT TO CHECK THIS AGAINST: on a healthy 10 Hz control link this is refreshed every
// ~100 ms, i.e. far inside usrConf.failsafe_time, so the motor gate behaves IDENTICALLY to today
// in normal operation. The only behaviour that changes is the failure case described above. The
// 0 initial value is also unchanged in effect: it matches last_packet's, so the first
// failsafe_time ms after boot read as "fresh" exactly as they always have.
//
// Written by the triggeredReceive task, read by the generatePWM task. volatile for the same reason
// last_packet is: single-core preemption, and the compiler must not cache it in a register across
// the gate's loop. A 32-bit naturally-aligned load/store cannot tear on this RV32 part.
volatile unsigned long last_control_packet = 0;

volatile uint8_t telemetry_index = 0;

volatile uint8_t payload_buffer[10];
volatile uint8_t payload_received = 0;

const unsigned long PAIRING_TIMEOUT = 10000;
const uint8_t MAX_ADDRESS_CONFLICTS = 5;

rmt_channel_handle_t tx_channel = NULL;
rmt_encoder_handle_t copy_encoder = NULL;
rmt_symbol_word_t pulse_symbol;

volatile int alternatePWMChannel = 0;
volatile bool PWM_active = 0;
volatile uint16_t PWM0_time = 0;
volatile uint16_t PWM1_time = 0;

volatile uint8_t thr_received = 0;
volatile uint8_t steering_received = 127;

// V2.5-Evo - 2026-07-19 - FM triage: the steering byte calcPWM() actually applied to the motor
// mix this loop (rtm_steer_override while RTM active + override enabled + thr>=25, otherwise the
// user's steering_received). Written by calcPWM() (generatePWM task, 100Hz) and read by the logger
// (loggerTask). Single-byte volatile — atomic on ESP32-C3, same pattern as thr_received. Logged so
// the actuation gap is visible: rtm_steer_override can command a turn while this stays neutral
// because the throttle-release gate suppressed it. 127 = straight ahead.
volatile uint8_t g_effective_steer = 127;

// V2.5-Evo - 2026-09-19 - DEEP LOG (C): the two motor commands the differential mixer produced this
// calcPWM() pass, in 0-255 command counts AFTER the mixer and BEFORE map() into each channel's PWM
// range (so the split is visible independent of PWM_min/max, trim and the effective_thr == 0 clamp,
// all of which act on the microsecond values afterwards). Written unconditionally on
// every pass of the steering_type == 1 branch from motor_mix.motor0/motor1; 0 / 0 for the efoil and
// servo branches, which have no mixer. DIAGNOSTIC OBSERVERS ONLY - the g_effective_steer pattern
// exactly: written by calcPWM() (generatePWM task, 100 Hz), read by fillLevel4Diag() (loggerTask),
// never read back into any control path. Single-byte volatiles, atomic on the ESP32-C3. A bench
// check: full-lock squeeze at 30 % throttle (76/255) with steering_influence 60 -> turn = 46 -> 30 / 122.
// V2.5-Evo - 2026-09-24 - the ramp is deliberately NO LONGER in that list of things these observers
// are independent of. motor_ramp_s used to rate-limit the finished microsecond values, downstream of
// every observer, so it was invisible in every log column; it now rate-limits the throttle BEFORE the
// mixer, so these two carry it and show the motor commands the ramp actually permitted on that tick.
// Their definition is otherwise unchanged - still post-mixer, pre-map, still diagnostic only. A rider
// squeezing from rest therefore shows these two climbing over motor_ramp_s while thr_received_log
// (the raw TX byte) jumps immediately: that gap IS the ramp, and it is the only place it is visible.
// V2.5-Evo - 2026-09-25 - and the MANUAL PIVOT ASSIST is now in that same list of things these two
// observers are NOT independent of: on the diff branch they are computed from the assisted throttle,
// so a row where g_pivot_assist_q4 below is non-zero shows the motor commands the assist actually
// permitted. Their definition is otherwise unchanged - still post-mixer, pre-map, still diagnostic.
volatile uint8_t g_motor0_cmd = 0;   // motor 0 command out of the mixer, 0-255; 0 when not the diff branch
volatile uint8_t g_motor1_cmd = 0;   // motor 1 command out of the mixer, 0-255; 0 when not the diff branch

// V2.5-Evo - 2026-09-25 - MANUAL PIVOT ASSIST observer: the assist's depth this calcPWM() pass,
// quantised to the 4 bits it rides in inside the deep log's existing fm_gate_flags word (bits
// 19-22, FM_LOG_GATE_PIVOT_ASSIST_SHIFT). 0 = inert, and inert means the throttle handed to the
// mixer is bit-identical to the un-assisted one; 15 = full assist, i.e. the throttle multiplied by
// kPivotFloorQ8/256. The intermediate depths 1-14 are genuine - the assist blends - but the values
// 1-15 can never appear "by accident" because the Schmitt band kPivotDepthArmQ8 keeps the internal
// depth at either 0 or at least one full rise step.
// DIAGNOSTIC OBSERVER ONLY - exactly the g_effective_steer / g_motor0_cmd pattern: written on every
// pass by calcPWM() (generatePWM task, 100 Hz), read by fillLevel4Diag() (loggerTask), and read back
// by NOTHING in any control path. Single-byte volatile, atomic on the ESP32-C3. The assist's real
// state is a uint16_t static inside calcPWM(); this is a lossy copy for the log, never the source.
volatile uint8_t g_pivot_assist_q4 = 0;   // manual pivot assist depth, 0 = inert, 15 = full

// V2.5-Evo - 2026-10-01 - M-2 (diagnosability): THE MOTOR GATE'S OWN VERDICT, published once per
// calcPWM() pass. 1 = the gate in generatePWM() is OPEN on this tick, so generate_pulse() is being
// called and the ESCs are being driven. 0 = SHUT, so NOTHING is leaving the board no matter what
// PWM0_time / PWM1_time happen to read - and calcPWM() computes those two unconditionally, which is
// exactly how ?printpwm came to show a live throttle value through a closed gate and cost a half-day
// on 2026-09-29.
// WHY A PUBLISHED VALUE AND NOT A THIRD COPY OF THE TEST: this is the value of the gate-mirror
// expression calcPWM() ALREADY computes for the M-3 ramp reset, so it carries PWM_active as well as
// the control-packet age, and the instruments that read it do not each grow their own copy of
// "PWM_active && millis() - last_control_packet < usrConf.failsafe_time". There are exactly TWO
// copies of that expression - the gate in generatePWM() and its documented mirror in calcPWM() - and
// the maintenance warning at both sites still names both. Adding readers here cannot make a third.
// DIAGNOSTIC OBSERVER ONLY - exactly the g_effective_steer / g_motor0_cmd pattern: written on every
// pass by calcPWM() (generatePWM task, 100 Hz), read by convertToLogData() (loggerTask) and by
// cmdDiag() / serPrintPWM() (loop task), and read back by NOTHING in any control path. Single-byte
// volatile, atomic on the ESP32-C3. Initialised 0 so a read before the PWM task's first pass reports
// the gate SHUT - the safe direction: an instrument must never claim the motors are live when it has
// not yet been told.
volatile uint8_t g_motor_gate_open = 0;   // 1 = motor gate OPEN this tick, 0 = SHUT (also 0 whenever PWM_active is down)

// V2.5-Evo - 2026-10-03 - I2C ENABLE-SWAP FAILURE COUNTERS. THE INSTRUMENT THIS CONTROL PATH HAS
// NEVER HAD, and the reason three separate half-days went into it blind.
//
// WHAT THEY COUNT. There is ONE PPM output on this board (GPIO 9, one RMT channel) and two motors.
// generatePWM() time-multiplexes them: pulse channel 0, wait 2 ms, swap the optocoupler enables
// over the AW9523 on I2C, pulse channel 1, swap back. That swap takes i2cMutex with a 10 ms bound,
// and when the take TIMES OUT the swap does not happen - correctly, because firing one channel's
// pulse onto the other channel's enable state would drive the wrong VESC. alternatePWMChannel is
// therefore left where it is and the SAME motor is pulsed again, which means THE OTHER MOTOR GETS
// NOTHING AT ALL. These two count exactly that event, per channel.
//
// READ THEM THE RIGHT WAY ROUND. g_swap_fail_ch0 counts failures on the tick where PWM0 was the
// enabled channel - so a run of ch0 failures means PWM0 is being re-pulsed and PWM1 IS THE STARVED
// ONE. ch1 is the mirror. The counter names the channel that KEPT its pulses, not the one that lost
// them, because that is the channel the code was looking at when it failed.
//
// SATURATING, NOT WRAPPING. A wrapped counter can read 0 after 65536 failures, which is the one
// value that must mean "healthy". They stop at 0xFFFF and stay there; ?diagz clears them.
//
// WHY volatile uint16_t AND NOT std::atomic. This is the g_diag_mux_switches / g_diag_mux_errors
// shape exactly (see those two above): a single-writer scalar observer, ++ from one task, read by
// ?diag and by the logger, zeroed by ?diagz. generatePWM() is the only task that can observe a swap
// outcome, so there is no second writer to tear against; the part is single-core, so there is no
// cache coherency to solve; and std::atomic<uint16_t>::fetch_add emits a barriered read-modify-write
// in the one task that must not be made heavier. ?diagz adds a second writer doing a plain = 0, so
// the worst case is ONE LOST INCREMENT around a zeroing - the identical, long-accepted behaviour of
// g_diag_mux_*. Stated here so nobody has to rediscover it.
volatile uint16_t g_swap_fail_ch0 = 0;   // enable-swap i2cMutex timeouts while PWM0 was the enabled channel (so PWM1 was starved). Saturates at 0xFFFF; cleared by ?diagz.
volatile uint16_t g_swap_fail_ch1 = 0;   // ...while PWM1 was enabled (so PWM0 was starved). Saturates at 0xFFFF; cleared by ?diagz.
volatile uint16_t g_swap_fail_run = 0;   // CONSECUTIVE failures as of right now; set to 0 by the next successful swap. Not a session total - it is the live run length.

volatile unsigned long get_vesc_timer = 0;
volatile unsigned long last_uart_packet = 0;

volatile uint8_t bind_pin_state = 0;
volatile uint8_t rx_aux_flags = 0;   // set by 0xF4 meta-packet: bit0=strobe, bit3=find-me

float fbatVolt = 0.0;
float noload_offset = 0.0;
uint8_t bc_arr[101];
uint8_t percent_last_val = 0xFF;
uint8_t percent_last_thr = 1;
unsigned long percent_last_thr_change = 0;

// V2.5-Evo: ERPM added to VESC selective-get mask; payload length is 23 bytes.
// P7: ERPM is also read by Phase C RTM anti-spoofing (RTMState.ino) to verify
// VESC speed matches GPS speed during active RTM. gps_en + vesc_erpm_per_kmh>0 required.
#define VESC_MORE_VALUES
#ifdef VESC_MORE_VALUES
  #define VESC_PACK_LEN 27  // +4 bytes for watt_hours (float32_auto)
  uint8_t vescRelayBuffer[34];
#else
  #define VESC_PACK_LEN 9
  uint8_t vescRelayBuffer[15];
#endif

//SPI Pins
#define P_SPI_MISO 6
#define P_SPI_MOSI 7
#define P_SPI_SCK 10
//LORA Pins
#define P_LORA_DIO 3
#define P_LORA_BUSY 4
#define P_LORA_RST 5
#define P_LORA_NSS 8
//Misc Pins
#define P_PWM_OUT 9
#define P_U1_TX 18
#define P_U1_RX 19
#define P_UBAT_MEAS 0
#define P_I2C_SCL 1
#define P_I2C_SDA 2

//AW9523 Pins
#define AP_U1_MUX_0 8
#define AP_U1_MUX_1 9
#define AP_S_BIND 0
#define AP_S_AUX 10
#define AP_L_BIND 1
#define AP_L_AUX 11
#define AP_EN_BMS_MEAS 4
#define AP_BMS_MEAS 7
#define AP_EN_PWM0 13
#define AP_EN_PWM1 12
#define AP_EN_WET_MEAS 14
#define AP_WET_MEAS 15

//Debug options — comment out for release builds
//#define DEBUG_RX
//#define DEBUG_VESC

#if defined DEBUG_RX
   #define rxprint(x)    Serial.print(x)
   #define rxprintln(x)  Serial.println(x)
#else
   #define rxprint(x)
   #define rxprintln(x)
#endif

#ifdef DEBUG_VESC
#define VESC_DEBUG_PRINT(x) Serial.print(x)
#define VESC_DEBUG_PRINTLN(x) Serial.println(x)
#else
#define VESC_DEBUG_PRINT(x)
#define VESC_DEBUG_PRINTLN(x)
#endif

#include "../Common/RadioCommon.h"
#include "../Common/SystemCommon.h"