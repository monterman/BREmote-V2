// V2.5-Evo - 2026-08-25 - RX RTM/FM D-term wrap fix. heading_error itself was normalized to +/-180 deg, but the derivative subtracted two normalized samples directly. Crossing the branch cut (for example +179 -> -179) therefore looked like a -358 deg step instead of the physical +2 deg change and Kd could saturate steering for one control tick. Normalize the same-source error delta to +/-180 before dividing by dt; source-switch/re-snap suppression, P term, gains, logging and config stay unchanged. No confStruct change; SW_VERSION stays 35.
// V2.5-Evo - 2026-08-17 - THREE FOLLOW-UPS TO THE PASS BELOW, ALL OF THEM NOTIFICATION, NONE OF THEM CONTROL. (1) THE DEGRADATION NOTICE COULD BE LOST ENTIRELY, NOT MERELY DEFERRED. headingDisagreeAnnounceDegraded() rightly returns without setting its one-shot flag while thr_received >= 25 — four Serial lines upstream of a hard stop would break the motor-to-zero-first rule — but its ONLY call site was inside the if (disagree_now) branch, so the retry needed another MEASURED disagreement. A measurement needs a live COG plus a compass snapshot younger than kHeadingCompareSnapMs, and that snapshot only refreshes while the trigger is released, so a dwell that completed inside the ~1 s window after a squeeze was silenced — and a rider who then finished the session under power and never coasted above rtm_cog_min_speed_kmh again rode the WHOLE SESSION with the compass withdrawn and Follow-Me refusing to engage, announced nowhere but a manual ?diag. getRtmHeading() now offers the notice on EVERY tick while the verdict stands, so the retry no longer depends on the evidence coming back; the deferral guard itself is untouched, and the print still cannot land between a proven fault and a motor-stopping write, because it can only fire below 25 counts where the deadman already holds the motor at 0. (2) A FAULT PROVEN WHILE COASTING NOW REACHES THE REMOTE. fm_fault_alarm_ms was set only if (thr_held), but the heading-disagree latch can only complete with the trigger RELEASED — so for this one fault the sticky fm_flags bit 3 never rose, the TX never learned the run had ended on a fault, and Follow-Me silently re-armed on the next keepalive into a blocked ARMED state whose only field signal was the not-ready flag. The alarm is now also set for a standing heading-disagree fault; every other fault keeps the surprise gating exactly as it was. (3) COMMENT-ONLY: the note in front of the restored FM fault term claimed a HOLD-parked Follow-Me would sit at cap 0 "for the rest of the session". The throttle-release clear rescues FM_HOLD back to FM_ARMED after 10 continuous seconds below 25 counts, so the accurate hazard is narrower — a rider FEATHERING the trigger restarts that timer on every squeeze, never accumulates the 10 s, and gets a dead motor on every squeeze with no explanation. Plus heading_disagree_fault is now volatile: it is read cross-task by Logger.ino through headingDisagreeLatched(), and as a file-scope static whose address never escapes the compiler may cache it. Read-only, log columns only, no control impact. No confStruct change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-08-17 - THREE CORRECTIONS TO THE DEGRADATION PASS BELOW, WHICH ASSUMED THE COMPASS WAS THE LIAR. The cross-check proves only that the compass and the GPS course CANNOT BOTH BE RIGHT. It does not say which one is wrong — the guard says so itself in guard 2's own header: "It deliberately does NOT pick a winner." Degrading the session onto GPS course alone picks one anyway, and it picks the compass as the culprit; if the RX's course is the bad source instead (marina multipath, a wrong dynamic platform model, a lagged COG at low speed) the degradation withdraws the GOOD sensor and operates on the bad one. (1) FOLLOW-ME REFUSES AGAIN — RELEASE BLOCKER. !heading_disagree_fault is back in can_be_active, and back in the FM fault-stop classifier it was deleted with, so a disagreement proven mid-engagement ends the run through the existing FM_STOPPING ramp (back to manual, re-arm required) instead of parking FM in FM_HOLD at cap 0 for the rest of the session. RTM KEEPS DEGRADING, and that part was sound: RTM's degraded behaviour is BOUNDED — with no heading the steering override is pinned at 127 and the align cap holds the throttle at 13/255 on the 180 deg sentinel — and the alternative was a genuinely dangerous half-armed state, buggy dead with the throttle at 0 while the TX still displayed RTM as ACTIVE. FOLLOW-ME IS NOT BOUNDED LIKE THAT: it engages autonomously at rider speed plus a margin, its steering authority is continuous, its only backstops are the divergence fault (about a 6.5 s grace plus a 3 s dwell) and the deadman — and the disagreement is UNMEASURABLE during the engagement, because compare_possible needs a compass snapshot younger than kHeadingCompareSnapMs and the snapshot only refreshes while thr_received < 25, so about a second into the run the comparison goes dormant and COG is served at confidence 3 unchallenged. Refusing to engage is the right answer to "one of my two heading sources is lying and I cannot tell which", and it is the same answer guard 1 already gives a few lines up: HOLDING STRAIGHT IS SAFER THAN STEERING ON THE SURVIVOR. The rider is not left guessing: the one-shot degradation notice still prints, the rate-limited "ARMED, NOT ENGAGING" explanation is restored, fm_flags bit 2 (armed-not-ready) carries the fault again so the TX cannot render "ready" for a Follow-Me that will not engage, and ?diag now reports the latch on demand. (2) THE kCogHoldMs COG HOLD SURVIVES DEGRADATION. The fault term moved back BELOW the hold, to the site it occupied before. The hold serves a last-good GPS COURSE, and the latch is evidence about the COMPASS, so withdrawing a GPS-derived value on compass evidence was outside this guard's charter — and it cost real behaviour twice over: a COG dropout longer than 1.5 s failed FM's condition 6 and forced a stop-and-re-arm the hold would have bridged, and on the RTM side the documented cog_valid flicker in the 3-4 km/h approach band (rtm_target_speed_kmh 4.0 against rtm_cog_min_speed_kmh 3) alternated the steering override between centre and bearing at 10 Hz. Degraded now means precisely "mode 1 minus the compass", which is what the evidence supports. (3) THE CLEAR IS AS STRICT AS THE SET. Setting the fault needs at least four measured samples spanning 5 s of continuity; clearing it took ONE agreeing tick. For a MIRRORED module the reported heading is theta - h_true, so the gap is 2*h_true - theta, which passes below 45 deg in two 45-deg-wide windows per revolution — a rider coasting near one of them cleared the latch instantly with the module still mirrored, and that is exactly the fault ?magalign cannot detect. Agreement must now hold continuously for kHeadingDisagreeMs with measured samples no more than kHeadingDisagreeGapMaxMs apart, mirroring the set. No new config field, no threshold retuned, no confStruct change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-08-17 - A HEADING DISAGREEMENT NOW DEGRADES THE SESSION INSTEAD OF REFUSING IT. WHAT SHIPPED BEFORE: a proven heading_disagree_fault withdrew the compass fallback (correct) and then refused to run on what was left (wrong). Gate 6 is enforced whenever usrConf.rtm_compass_required is set, which is the default, so on a stationary arm there was no heading at all, the gate raised rtm_rx_emergency_stop and the buggy sat dead with the throttle pinned at 0 — while gates 2-8 deliberately do NOT clear rtm_rx_active, so the TX went on displaying RTM as ACTIVE. Follow-Me had the twin of it: can_be_active carried !heading_disagree_fault, so FM sat in FM_ARMED for the rest of the session. The rider armed, nothing moved, and the remote said it had. WHAT IT DOES NOW: the COMPASS is the sensor that was caught lying; the GPS course is not, so refusing to run threw away a good source because a different one failed. A standing fault now degrades the session to COG-only — which is exactly usrConf.rtm_use_compass == 0, an existing, documented, supported mode — instead of refusing: getRtmHeading() takes the mode-0 path for heading SELECTION while the fault stands (live GPS course above rtm_cog_min_speed_kmh, nothing below it: no compass fallback and no kCogHoldMs hold, because mode 0 has neither), gate 6 is bypassed exactly as it is for a rider who has set rtm_compass_required = 0, FM's can_be_active and its fault-stop chain no longer carry the flag, and fm_flags bit 2 stops calling it armed-not-ready. The rider's experience becomes the ordinary COG-only one: get the buggy moving and the GPS course takes over. NOTHING IS WRITTEN TO usrConf — rtm_use_compass and rtm_compass_required are untouched, no SPIFFS write, no ?conf change; this is runtime degradation for the session only. THE COMPARISON ITSELF STILL RUNS IN THE CONFIGURED MODE (compare_possible keeps testing the stored mode, not the degraded one), because that measurement is the evidence route that clears the fault. AND BECAUSE THE FLAG NO LONGER BLOCKS ANYTHING, THE LATCH IS NOW STICKY: the disarm-edge clear in runRtmLoop() and the FM-idle clear in fmEnterIdle() are GONE — they forgave a proven fault with no evidence, so a mis-mounted compass had to be re-proven by coasting on every single run. Exactly three clear routes remain: a live measurement showing the two sources agreeing again, a SUCCESSFUL ?compasscal (CAL_FULL only) or ?magalign — the rider actually fixing the mounting, wired in Compass.ino — and a reboot. A PARTIAL or failed calibration does NOT clear it. The rider is told once on serial, the moment the degradation engages, what happened and how to fix it. Subtract-only throughout: the flag grants no heading, raises no cap, extends no engagement, and it now removes strictly MORE heading than before. No threshold retuned, no confStruct change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-08-17 - The heading-disagree backstop threw away the only evidence it ever gets. THE EVIDENCE WINDOW: a comparison needs a live COG *and* a compass snapshot less than kHeadingCompareSnapMs (1000 ms) old, and updateCompassSnapshot() only refreshes the snapshot while thr_received < 25, because a snapshot taken under motor current is worthless. RTM and FM both require a HELD trigger. So about one second into any engagement the comparison goes dormant and the kHeadingDisagreeMs (5 s) dwell can never complete inside a run — the ONLY time both inputs exist together is when the rider is coasting with the trigger released above rtm_cog_min_speed_kmh, or during the arm ceremony, where the TX pins the throttle byte to zero. The previous pass then added an ARM-EDGE clear that wiped heading_disagree_fault at exactly that boundary, i.e. it discarded the freshest evidence available anywhere in the sequence and left the guard unable to act on anything it had proven. FIX: the arm edges (rtm_rx_active false -> true in runRtmLoop, and the FM_IDLE -> armed edge in runFmLoop) now clear only the UNFINISHED dwell (heading_disagree_since_ms, heading_disagree_last_seen_ms). A PROVEN fault survives into the engagement, where getRtmHeading() consumes it by withdrawing the compass fallback. The DISARM edges (rtm_rx_active true -> false, and a genuine transition into FM idle via fmEnterIdle) still clear all three, so this is still not a power-cycle latch, and a live measurement showing the two sources agreeing again still clears it outright. Because a surviving fault can now refuse an engagement, both refusals were made to SAY SO instead of being silent: RTM gate 6 prints the real cause (a compass caught disagreeing with GPS course) rather than the misleading generic "No valid heading source", and FM — which would otherwise sit in FM_ARMED with the TX reading "ready" — now raises fm_flags bit 2 (armed-not-ready) and prints a rate-limited explanation. Subtract-only throughout: the flag grants no heading, raises no cap and extends no engagement. No threshold retuned, no confStruct change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-08-17 - SAFETY (release blocker): rtm_steer_override could survive an idle period and be applied on the first tick of the NEXT engagement. Only gate 1 (throttle released) and updateRtmSteering() ever wrote the neutral 127, so a GATE 9 HANDOFF WITH THE TRIGGER STILL HELD left the last bearing-derived value — say 210 — parked indefinitely: the inactive path in runRtmLoop() returned without touching it and fmEnterIdle() deliberately does not touch it either. calcPWM() applies the override whenever (rtm_rx_active || fm_rx_active) && rtm_rx_override_steering && thr_received >= 25, so on the next re-arm the 100 Hz generatePWM task could put that stale bearing on the motors as differential steering before runRtmLoop() recomputed it — with no gate 9 and no Phase C in the path. Normally a 100 ms window; UNBOUNDED if the re-arm lands while the loop task is frozen inside a deliberately non-abortable command (?gpssetup, ?wifiupd), since runRtmLoop() is then not running at all. FIX: runRtmLoop() now forces rtm_steer_override = 127 on every tick where BOTH rtm_rx_active and fm_rx_active are false. The !fm_rx_active term is load-bearing, not defensive — runRtmLoop() runs BEFORE runFmLoop() in loop(), so an unconditional reset would publish a neutral 127 for one preemption window per cycle throughout every Follow-Me engagement (RTM is inactive for all of one) and stutter FM steering to centre at 10 Hz; same hazard fm_throttle_cap has its own global to avoid. Subtract-only: writes the neutral value only, never a turn, never throttle. No confStruct change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-08-16 - Follow-up to the edge-clear fix below; three items, all in the heading-disagreement guard. (1) THE LATCH NOW ALSO CLEARS AT THE ARM EDGE, not only at the disarm edge. runRtmLoop() calls getRtmHeading() unconditionally every 100 ms to fill the fm_status telemetry byte, so the disagreement dwell runs during ORDINARY MANUAL RIDING with nothing armed: a rotated or uncalibrated compass plus five seconds of coasting above rtm_cog_min_speed_kmh with the trigger released (snapshot still refreshing, COG live) was enough to latch the fault before the rider had armed anything. Once both clears became edges, nothing forgave that. The rider then declared FM, separation latched, and FM sat in FM_ARMED for the rest of the session - silently, because fm_flags bit 2 (armed-not-ready) DROPS once separation latches so the TX reads "ready", and the HEADING DISAGREE FAULT printf only runs when FM was already engaged. RTM had the same dead end by a different road: gates 2-8 set rtm_rx_emergency_stop WITHOUT clearing rtm_rx_active, so a TX that dies mid-tow leaves rtm_rx_active stuck true, the disarm edge never fires, and the "fresh agreement" clear is unreachable because the emergency stop pins throttle at 0, so the buggy cannot move, so COG never goes valid. Both are answered by clearing at the START of an engagement as well as the end - the rtm_rx_active false -> true edge in runRtmLoop(), and the FM_IDLE -> armed edge in runFmLoop(). This cannot weaken the guard: the flag only ever SUBTRACTS eligibility, and a genuine disagreement re-proves itself within kHeadingDisagreeMs (5 s) of the run starting, so the cost is at most 5 s of protection at the very beginning of an engagement. (2) THE 15 s EVIDENCE CAP BOUNDED THE OTHER DEFECT BUT DID NOT REMOVE IT: it measures the SPAN from the start of the dwell, not the age of the last measurement, so one disagreeing tick at t=0 and one more at t=14 s passed the cap and then satisfied "held continuously for 5 s" immediately - a fault built from two isolated one-tick samples fourteen seconds apart. Persistence is now required to be CONTINUOUS: consecutive measured disagreements no more than kHeadingDisagreeGapMaxMs (2 s) apart, so at least four samples must span the dwell. The freeze-don't-clear behaviour the short COG handover needs is untouched. (3) Comment-only: the prose in front of the COG hold claimed the function hands out no heading at all once the latch sets. It still serves a HELD COG at confidence 2 for up to kCogHoldMs, deliberately - the latch is evidence against the COMPASS, and a held COG is not the compass - so the comment now states what the code does instead of asserting more. No threshold retuned, no ordering changed, no confStruct change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-08-16 - The heading-disagree backstop was INERT, and it was inert in the exact mode it was extended to protect. fmEnterIdle() zeroed heading_disagree_since_ms and heading_disagree_fault on every call, and runFmLoop() calls fmEnterIdle() on every 100 ms tick whenever FM is idle — a condition that INCLUDES rtm_rx_active. So for the whole duration of any RTM run the dwell timer and the latch were being wiped ten times a second: the kHeadingDisagreeMs (5 s) dwell could never accumulate, the fault could never be set, and the check that consumes it in getRtmHeading() (in front of the compass fallback) could never see anything. In RTM-only operation — the field case this was built for, a buggy that veered at close range and never came back — the advertised sticky backstop did nothing. FIX 1: the clear in fmEnterIdle() is now EDGE-triggered on a genuine transition into FM idle (a real end of an FM declaration), not re-applied every tick, so the latch survives across ticks WITHIN an engagement. FIX 2: RTM gained its own end-of-engagement clear at the top of runRtmLoop() — an edge on rtm_rx_active going true -> false — so a proven fault is forgiven when the rider disarms and cannot outlive the run that produced it (it is NOT a power-cycle latch). Deliberately an edge there too: clearing it at every tick RTM is inactive would recreate the same bug for Follow-Me, which only ever runs while RTM is inactive. FIX 3 (the freeze-age MEDIUM): the "no comparison possible" branch froze the dwell timestamp indefinitely, so two one-tick disagreements minutes apart escalated as one "sustained 5 s" fault; a part-finished proof older than kHeadingDisagreeEvidenceMaxMs (3x the dwell) is now discarded, while a PROVEN fault is untouched. Nothing here grants a heading, raises a cap or extends an engagement — the guard can still only subtract. cog_last_good_deg/ms are no longer static so Logger.ino can read (never write) them. No confStruct change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-07-25 - STAGE 2 (RX heading-source trust guards, RTM + FM): the bench ?diag on the repaired board read "COG : 7.4 timestamp-updates/s vs 0.0 value-changes/s [value frozen 533 s]" — the GPS repeated ONE course seven times a second for nine minutes while gps_last_course_ms kept refreshing, so getRtmHeading()'s 1500 ms freshness gate passed the whole time and the ladder steered on a dead number, then fell back to the EMI-biased compass. That is the Follow-Me veer. GUARD 1: the COG branch now also requires the course VALUE to have moved within kRtmCogFrozenMs (3 s), read from the existing Stage 0 tracker g_diag_cog_change_ms — and it is SPEED-GATED to the same gps_last_speed_kmh >= rtm_cog_min_speed_kmh term the branch already used, so a stationary buggy (whose constant course is correct, not faulty) is never touched. A COG frozen WHILE MOVING additionally suppresses the mode-1 compass promotion: one source is provably dead and the other unverifiable, so we hold straight instead of steering on the survivor. GUARD 2: when a live COG and a simultaneous (< kHeadingCompareSnapMs) compass snapshot are both available in hybrid mode, a shortest-angular-distance gap > kHeadingDisagreeDeg (45) returns NO heading (confidence 0) so updateRtmSteering() holds straight; sustained past kHeadingDisagreeMs (5 s) it sets heading_disagree_fault, which joins the EXISTING FM fault chain alongside A3's diverge_fault (FM_STOPPING -> cap 0 -> ramp -> FM_IDLE, re-arm required) — no new state, no new exit path. Both guards can only REMOVE eligibility: no throttle cap is raised, no engagement extended, the deadman (thr_received < 25) is untouched. Logger.ino's inline getRtmHeading() duplicate is updated in lockstep so rtm_source/rtm_confidence keep telling the truth. All compile-time constants (shared, in BREmote_V2_Rx.h); no confStruct change, sizeof stays 184, SW_VERSION stays 34.
// V2.5-Evo - 2026-07-25 - F3-c (RX FM): the 8 m engage-distance floor guarded ONLY the manual fm_engage_dist_m value. The AUTO branch — kFmEngageFactor (1.5) x (min_dist_m + followme_smoothing_band_m) — used its product raw, and neither of those two SPIFFS fields has a lower bound, so a small tuning such as min_dist 1 + band 1 produced d_engage = 3 m: BELOW the measured 20 ft / 6.10 m tow rope, letting Follow-Me latch and engage with the rider still ON the rope. Same hazard the floor exists to prevent, reached through the other branch. FIX: each branch now only computes its candidate and ONE kFmEngageDistFloorM clamp is applied to the final d_engage regardless of origin. The clamp can only RAISE d_engage (engage later, never earlier), and it is a no-op at the owner's 4+2 tuning, which yields 9.0 m. Also swept the stale "6.7-7.6 m" tow-rope prose in this file to the measured 6.10 m. No confStruct change, sizeof stays 184, SW_VERSION stays 34.
// V2.5-Evo - 2026-07-25 - F3-b (RX FM; comment + shared-constant move, no behaviour change): kFmEngageDistFloorM is no longer DEFINED in this file. It now has exactly one definition, in BREmote_V2_Rx.h, raised 5.0 -> 8.0 m — 5.0 m sat below the tow rope it exists to clear (the owner's rope is 20 ft = 6.10 m), so a manual fm_engage_dist_m of 5.0-6.1 m was legal and let FM engage with the rider still ON the rope. ConfigService.ino's duplicate bare 5.0f literal is gone with it; both the validator and the read-site clamp in runFmLoop() now reference the one shared constant. The clamp itself is unchanged in shape — legacy stored values still clamp UP to the floor, now 8.0 m. No confStruct change, sizeof stays 184, SW_VERSION stays 34.
// V2.5-Evo - 2026-07-25 - Batch A follow-up (Rex A3 NO-GO: F1/F3/F5/F7), RX FM only. (F1) the A3 divergence detector was a BARE THRESHOLD on a 3000 ms dwell while the engage ramp is 3500 ms, so it could fire BEFORE the ramp even finished and aborted ordinary engagements (at engage, dist_m is typically 13-21 m against an 18 m ceiling, and the align cap of 13/255 means the gap GROWS first). It now mirrors runPhaseC()'s actual shape: the distance at dwell start is captured (fm_diverge_start_dist_m) and the fault is raised at dwell expiry ONLY if the buggy is not closing (dist_m >= start - kFmDivergeCloseEpsM 2.0 m); if it has closed by more than that it IS following, just far, so the timer clears and no fault fires. Plus an engage grace: the detector is skipped and its dwell parked for kFmEngageRampMs + kFmDivergeMs (6.5 s) after every engagement so the buggy is allowed to ramp and align before it is judged. (F3) fm_engage_dist_m gained a 5 m floor — a stored value of, say, 3 m IS the engage distance in metres and is SHORTER than the 6.7-7.6 m tow rope, which defeated the separation interlock entirely; cfgValidateCrossField() now accepts only 0 (auto) or >= 5.0 m, and the use site clamps defensively so a pre-existing stored value cannot slip through. (F5) corrected the A2 comment that claimed d_engage feeds the distance Schmitt hysteresis - it does not; the Schmitt uses min_dist / min_dist+band. (F7) the divergence Serial.printf now runs AFTER fm_throttle_cap = 0 so UART backpressure can never defer the hard stop. All compile-time constants; no confStruct change, sizeof stays 184, SW_VERSION stays 34.
// V2.5-Evo - 2026-07-25 - Batch A (A2+A3), RX FM only. (A2) fm_engage_dist_m is now READ: >0 sets the FM engage distance directly in metres (rope x ~1.15), 0 keeps the previous auto behaviour (kFmEngageFactor x d_follow) bit-for-bit; latch, dwell and Schmitt hysteresis untouched. (A3) FM divergence FAULT — runFmLoop() gained the upper distance bound it never had: condition 8 is a lower bound only, and runPhaseC()'s convergence check is RTM-only (called from runRtmLoop, never runFmLoop), so a wrong heading let FM steer away indefinitely. dist_m > kFmDivergeFactor(2.0) x D_engage sustained kFmDivergeMs(3000) while FM_ACTIVE now routes through the EXISTING fault path (FM_STOPPING ramp -> FM_IDLE, re-arm required, same haptic/St semantics as conditions 2-7). Non-blocking first-exceed timestamp, cleared on any condition/state/data-trust change. Subtract-only: adds no throttle, extends no engagement, does not touch the deadman. All compile-time constants; no confStruct change; SW_VERSION stays 34.
// V2.5-Evo - 2026-07-19 - P3 FM (DESIGN_FOLLOW_ME.md sections 4-7): Follow-Me autonomous following. Adds runFmLoop() 10Hz state machine (IDLE/ARMED/ACTIVE/DEMOTED incl. the missing 0xFF->usrConf.followme_mode fallback — SUPERSEDED 2026-07-20, see R0 below), all 9 activation/hold conditions with Schmitt hysteresis on distance and side-zone, the lag-anchor trailing target-point geometry, and the 5-stage subtract-only throttle cap chain. Reuses the existing EMA filter / P+D / heading ladder / authority / wrap pipeline unchanged - updateRtmSteering() only gains a target selector (RTM = rider position, FM = trailing point). telemetry.fm_status bit0 now reports FM engaged rather than FM mode selected. No confStruct change; SW_VERSION stays 33.
// V2.5-Evo - 2026-07-20 - FM engagement semantics (R0/R1/R2): (R0) BOTH 0xFF->usrConf.followme_mode fallbacks removed — 0xFF now means FM_IDLE always, killing the latently-armed factory boot; usrConf.followme_mode is the TX arm-gesture seed only. (R1) separation latch: FM's FIRST entry into ACTIVE now also requires dist > kFmEngageFactor(1.5) x d_follow sustained kFmSepDwellMs(2000) — the tow rope (6.7-7.6 m) is longer than the old engage distance, so FM could engage mid-tow; existing Schmitt hysteresis governs after the latch sets. (R2) two clears: thr_received<25 for kFmThrReleaseClearMs(10 s) clears the latch (ARMED-unlatched, mode memory kept); no 0xF2 refresh for kFmModeAgeMs(95 s) -> FM_IDLE. P3 geometry/cap/steering untouched. No confStruct change; SW_VERSION stays 33.
// V2.5-Evo - 2026-07-20 - FM control "brain" (Fable v1.4): (A) holds-vs-faults — condition 1=DEADMAN (throttle, never a fault), 8/9=HOLD (cap 0, stays ARMED, auto-resume, +kFmSpeedHystKmh speed hysteresis), 2-7=FAULT (FM_STOPPING ramp 0->255 over kFmStopRampMs -> FM_IDLE, re-arm required); heading loss (cond 6) is now ALWAYS a fault regardless of rtm_compass_required. (C) steer-cancel while ACTIVE -> ARMED-UNLATCHED (latch cleared, no alarm) guarded by kFmEngageGraceMs grace + kFmSteerPersistMs persistence; ARMED has no steer-cancel by construction. (D) fm_flags telemetry byte (repurposed reserved_tx_imu): armed/engaged/armed-not-ready/fault-stop-sticky(kFmFaultStickyMs). FM_DEMOTED renamed FM_HOLD; FM_STOPPING added. All compile-time constants; no confStruct change; SW_VERSION stays 33.
// V2.5-Evo - 2026-07-19 - Rex hardening: reset D-term continuity statics (prev_heading_src_valid/prev_heading_error_deg/prev_steering_update_ms) in the override-disabled early return so an off->on toggle can't differentiate a stale error across the gap
// V2.5-Evo - 2026-07-19 - FM triage (Fable audit §5): (1) no-fix engagement guard — getRtmHeading() returns confidence 0 unless a fresh RX GPS fix exists, so RTM/FM cannot report confidence 2 or engage with datetime_unix=0; (2) D-term differentiated only across consecutive same heading-source samples — skip the step on a source switch (COG<->compass) or a compass-snapshot re-snap to kill the ±300°/s Kd spikes
// V2.5-Evo - 2026-05-22 - SW32: Two-phase RTM throttle — align phase suppresses throttle until heading < rtm_align_threshold_deg; run phase GPS speed governor
// V2.5-Evo - 2026-05-11 - Phase C fix: VESC ERPM check now verifies data freshness via vesc.last_packet before comparing to GPS speed
// V2.5-Evo - 2026-05-08 - Bundle 1: P+D+filter steering controller; preset table; bearing filter for FM path-following
// V2.5-Evo - 2026-05-06 - D5: getRtmHeading() layered heading source; updateRtmSteering() rewritten; Gate 6 accepts any source; updateCompassSnapshot() called from runRtmLoop top
// V2.5-Evo - 2026-05-03 - C1/M2 audit fix: gps_tx_ok uses timestamp age on both paths; 0.0 lat/lng sentinel removed
// V2.5-Evo - 2026-05-01 - Fix D: gps_tx_ok relaxed for FM/idle; never reset rtm_distance to 0xFF when RTM inactive
// V2.5-Evo - 2026-05-01 - Fix C: FM bar keep-last-known on GPS dropout; only 0xFF if TX GPS never received
// V2.5-Evo - 2026-05-01 - Fix B: encode rtm_distance always when GPS valid; feeds FM bar and enables correct pre-arm block within stop distance
// V2.5-Evo - 2026-04-30 - Gate 9 clean disengagement (handoff to manual, no emergency stop); re-arm fix (0xFF when inactive); approach decel zone computation
// V2.5-Evo - 2026-04-25 - P7: RX RTM state machine, 10 safety gates, Phase C anti-spoofing.
// V2.5-Evo - 2026-04-27 - P8: runRtmLoop() encodes RX→TX distance into telemetry.rtm_distance (index 5)
// V2.5-Evo - 2026-04-28 - P9 Bug1A/1B/1C: Gate9 zero-guard; always-compute dist before gates
// V2.5-Evo - 2026-04-28 - Security: Gate 1 resets rtm_steer_override=127 on throttle release
// V2.5-Evo - 2026-04-29 - Fix 6-1: Gate 4 + Phase C check 3 now use
//   usrConf.tx_gps_stale_timeout_ms instead of hardcoded 2000ms
// V2.5-Evo - 2026-04-29 - Fix 6-2: runRtmLoop() revokes gps_phase_b_ok
//   when TX GPS age exceeds 2× tx_gps_stale_timeout_ms
//
// The RTM state machine runs in loop() at ~10Hz (100ms rate-limit).
// When rtm_rx_active is set true by a 0xF1 meta-packet, this module:
//   1. Checks all 10 safety gates every iteration (any fail → emergency stop).
//   2. Computes compass bearing toward TX GPS position.
//   3. Converts bearing error to a steering override (0-255, 127=straight).
//   4. Runs Phase C: convergence check, VESC ERPM speed check, TX GPS freshness.
//
// All outputs are written to volatile globals read by calcPWM() and triggeredReceive().

extern bool gps_phase_b_ok;   // V2.5-Evo - P7 fix: defined in Radio.ino (Phase B section)
// V2.5-Evo - 2026-05-06 - D5: extern declarations for D1+D2 capture globals.
extern float         gps_last_course_deg;       // From GPS.ino (D1) — last valid GPS course-over-ground (0-360 deg, -1.0 if none)
extern unsigned long gps_last_course_ms;        // From GPS.ino (D1) — millis() of last course update (0 if none)
extern float         compass_snapshot_heading;  // From Compass.ino (D2) — clean compass heading captured during motor-idle (0-360 deg, -1.0 if none)
extern unsigned long compass_snapshot_ms;       // From Compass.ino (D2) — millis() of snapshot capture (0 if none)
extern void          updateCompassSnapshot();   // From Compass.ino (D2) — captures clean compass heading when motor idle
// ============================================================
// RTM/FM STEERING CONTROLLER PRESETS — Bundle 1 (2026-05-08)
//
// PID-style controller: output = Kp * clamped_error - Kd * d(error)/dt
// Plus a low-pass filter on TARGET POSITION (lat/lng) for FM path-following
// — surfer's high-frequency bottom turns are smoothed out, buggy follows
// the surfer's path rather than chasing every wobble.
//
// For RTM (TX stationary), filter τ is set very low so behavior is essentially
// unfiltered — the filter doesn't hurt because there's nothing to smooth.
//
// 5 presets cover flat-water-to-heavy-surf range. Operator picks via WebUI
// before each session based on conditions. Default = Normal (index 2).
// ============================================================
struct SteerPreset {
  float error_clamp_deg;     // Saturation: heading error clamped to ±this before P/D math.
  float kp;                  // Proportional gain (PID Kp). 1.0 = baseline.
  float kd;                  // Derivative gain (PID Kd). 0.0 disables D term entirely.
  float target_filter_tau_s; // Low-pass filter time constant on target position (seconds).
                             // 0.5 ≈ no smoothing for RTM; 1-5 path-following for FM.
};

static const SteerPreset kSteerPresets[5] = {
  // {clamp,    Kp,   Kd,   tau_s }
  {  150.0f,  0.70f, 0.50f, 5.00f },  // 0 Very Soft   — heavy surf, aggressive surfer
  {  120.0f,  0.85f, 0.40f, 3.00f },  // 1 Soft        — choppy normal session
  {   90.0f,  1.00f, 0.30f, 2.00f },  // 2 Normal      — DEFAULT, mixed conditions
  {   60.0f,  1.20f, 0.20f, 1.00f },  // 3 Sharp       — calm water, RC use
  {   45.0f,  1.40f, 0.10f, 0.50f },  // 4 Very Sharp  — glass-flat, no waves
};

// ---- Bundle 1 module-level state for P+D controller and bearing filter ----
static float         prev_heading_error_deg    = 0.0f;
static unsigned long prev_steering_update_ms   = 0;
// D-term source-continuity tracking (2026-07-19 FM triage). We only differentiate
// heading_error across two samples that came from the SAME continuous heading source.
// prev_heading_src_id is a discriminator that changes on a source switch (COG<->compass)
// AND on a compass-snapshot re-snap; prev_heading_src_valid gates the very first sample.
static uint32_t      prev_heading_src_id       = 0;
static bool          prev_heading_src_valid    = false;
static double        tx_pos_filtered_lat       = 0.0;  // Filtered TX lat (degrees)
static double        tx_pos_filtered_lng       = 0.0;  // Filtered TX lng (degrees)
static bool          tx_pos_filter_initialized = false;

// Non-static globals exported to Logger.ino via extern (Bundle 1 tuning telemetry).
// 0x7FFF is the "no data" sentinel (non-zero).
int16_t g_heading_error_dx10 = 0x7FFF;  // Last heading error × 10 deg; 0x7FFF = no data
int16_t g_d_error_dx10       = 0x7FFF;  // Last derivative × 10 deg/s; 0x7FFF = no data

// ---- Phase C convergence tracking ----
static double        rtm_prev_dist_m = -1.0;   // distance to TX at last Phase C check
static unsigned long rtm_phase_c_ms  = 0;       // last Phase C check time

// V2.5-Evo - 2026-08-17 - Read-only accessor for the heading-disagreement latch, forward-declared
// here so gate 6 below can ask whether the compass has been withdrawn from the heading ladder —
// while it has, that gate stands aside instead of refusing the run (see the note at the gate).
// Two other readers use the same accessor and neither is in the control path: Logger.ino's heading
// mirror, and (V2.5-Evo - 2026-08-17) the ?diag line in System.ino that lets a rider ask the board
// whether the latch is standing.
// WHY AN ACCESSOR AND NOT THE FLAG ITSELF: heading_disagree_fault is a file-scope static declared
// ~170 lines further down, alongside the long block that documents it. A variable is not visible
// before its declaration — unlike a function, which the Arduino builder auto-prototypes — and
// moving the state up here would separate it from its documentation for no benefit. The accessor
// is defined immediately after the flag. It only reads: it cannot set, clear or age anything.
static bool headingDisagreeLatched();

// ---- Safety gate check ----
// Returns true if ALL gates pass. Sets rtm_rx_emergency_stop=true and prints reason on any failure.
// Gate 1 (throttle released) returns false WITHOUT setting emergency_stop — motor is already 0.
static bool checkRtmSafetyGates()
{
  unsigned long now = millis();

  // Gate 1 (ABSOLUTE): user must be physically holding throttle > 10%.
  // Creator safety philosophy — this gate CANNOT be waived.
  if (thr_received < 25)
  {
    // Throttle released — this is normal; do not emergency-stop, just return false.
    // SAFETY FIX (2026-04-28 audit): reset steer override to straight (127) before returning.
    // Without this reset, the last bearing-derived value persists in rtm_steer_override.
    // calcPWM() applies that stale value to differential motor math even with thr=0:
    //   steering_offset_1 ≈ +286 at override=200 → PWM1_time=1286µs (motor spins ~28%)
    //   despite the user not holding the throttle — a hard safety violation.
    // Belt-and-suspenders companion fix is in PWM.ino calcPWM() (Task 1B).
    rtm_steer_override = 127;
    return false;
  }

  // Gate 2: Phase A GPS not rejected on RX
  if (gps_rejected)
  {
    Serial.println("RTM [RX] STOP: Phase A GPS rejected");
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 3: Phase B handshake passed
  if (!gps_phase_b_ok)
  {
    Serial.println("RTM [RX] STOP: Phase B handshake not passed");
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 4: valid TX GPS fix (age < usrConf.tx_gps_stale_timeout_ms)
  // Finding 6-1: was hardcoded 2000ms — now reads from SPIFFS so the
  // WebUI setting actually takes effect. Default is 1000ms.
  if (rx_tx_gps_timestamp == 0 ||
      (now - rx_tx_gps_timestamp) > (uint32_t)usrConf.tx_gps_stale_timeout_ms)
  {
    Serial.println("RTM [RX] STOP: TX GPS stale or never received");
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 5: valid RX GPS fix (age < 6000ms = 3× TX GPS timeout)
  if (gps_last_ms == 0 || (now - gps_last_ms) > 6000UL)
  {
    Serial.println("RTM [RX] STOP: RX GPS stale");
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 6: valid heading source (any source, per usrConf.rtm_use_compass mode)
  // V2.5-Evo - 2026-05-06 - D5: was compass-only check; now accepts GPS COG OR
  // compass snapshot OR live compass per the configured heading mode.
  // The legacy field name rtm_compass_required is preserved as the gate enable/disable.
  // When set to 1 (default), at least one valid heading source must exist.
  // When set to 0, this gate is bypassed (advanced/manual users only).
  //
  // V2.5-Evo - 2026-08-17 - AND IT IS ALSO BYPASSED WHILE THE HEADING-DISAGREEMENT LATCH STANDS.
  // WHAT THIS FIXES. getRtmHeading() withdraws the compass the moment that compass has been caught
  // disagreeing with the GPS course, which is right — but with this gate still enforced, a buggy
  // that is stationary or below rtm_cog_min_speed_kmh at arm time then had NO heading at all, so
  // the gate raised rtm_rx_emergency_stop and pinned the throttle at 0. Gates 2-8 deliberately
  // leave rtm_rx_active TRUE, so the TX went on showing RTM as active while the buggy was dead:
  // it half-armed. That is a refusal, and refusing throws away the GPS course — a source that was
  // never in doubt — because a different sensor failed.
  // WHAT IT DOES INSTEAD. A standing fault degrades the session to COG-only, and the point of
  // COG-only (usrConf.rtm_use_compass == 0) is that it is a real, documented mode with its own
  // documented arming behaviour: rtm_compass_required is what a COG-only rider turns off, because
  // demanding a heading before the craft is moving is exactly what a COG-only craft cannot supply.
  // So while the latch stands this gate behaves the way it does for that rider — it stands aside,
  // RTM arms, updateRtmSteering() holds the override at 127 (straight) for as long as there is no
  // heading, and the buggy acquires a real GPS course as soon as it is moving faster than
  // rtm_cog_min_speed_kmh. Every other gate is untouched, Phase C's convergence check still
  // watches the run, and the throttle stays whatever the rider's trigger asks for minus the caps.
  // SAFETY DIRECTION: this can only ever let RTM run WITHOUT a heading, never let it steer on a
  // heading it does not trust — the compass is still withdrawn inside getRtmHeading(), so nothing
  // downstream can steer on the sensor that was caught lying. No throttle is added: the align cap
  // (~5% of 255) applies for as long as no heading exists, because g_heading_error_dx10 stays at
  // its 0x7FFF sentinel and the align phase treats that as the worst case.
  // V2.5-Evo - 2026-08-17 - TWO PRECISIONS on the wording above. (1) "COG-only" is now exactly
  // "mode 1 minus the compass": the kCogHoldMs held-COG bridge is served as well as the live course,
  // because the hold is a GPS value and this latch is evidence about the compass. RTM's degraded
  // behaviour is unchanged in kind — it simply has one fewer gap in it. (2) THIS BYPASS IS RTM-ONLY
  // AND DELIBERATELY SO. Follow-Me does NOT degrade: can_be_active carries the fault, so FM declines
  // to engage while the latch stands. RTM may run headless because its headless behaviour is bounded
  // (override pinned at 127, align cap at 13/255, Phase C watching, rider's trigger the only
  // throttle); FM engages autonomously with continuous steering authority, which is not bounded in
  // the same way. See can_be_active in runFmLoop() for the full argument.
  if (usrConf.rtm_compass_required && !headingDisagreeLatched())
  {
    float h_unused;
    uint8_t conf_unused;
    if (!getRtmHeading(&h_unused, &conf_unused))
    {
      // Emergency stop BEFORE the print, as the 2026-08-17 pass established for this gate: the F7
      // rule (see runFmLoop) is motor to 0 first, explain afterwards, because a full USB CDC TX
      // buffer can block inside Serial for as long as the host takes to drain it.
      rtm_rx_emergency_stop = true;
      Serial.println("RTM [RX] STOP: No valid heading source (GPS COG too slow + compass snapshot stale)");
      return false;
    }
  }

  // Gate 7: LoRa link healthy
  if (millis() - last_packet > usrConf.failsafe_time)
  {
    Serial.println("RTM [RX] STOP: LoRa link lost");
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 9: hard stop distance — buggy reached TX position.
  // This is a NORMAL RTM completion, not a safety failure (unlike Gates 2-8).
  // Clean disengagement: set rtm_rx_active=false and leave rtm_rx_emergency_stop=false
  // so calcPWM() passes user throttle through immediately (seamless manual handoff).
  // rtm_approach_cap reset to 255 so manual throttle is uncapped.
  // The inactive path in runRtmLoop() will set telemetry.rtm_distance=0xFF on the next
  // tick, clearing the TX pre-arm block so re-arm works after the buggy has moved away.
  // Guard: rtm_stop_distance_m==0 means SPIFFS held the pre-fix zero default;
  // use 10m (firmware hard minimum) to keep Gate 9 active regardless of stored config.
  uint16_t stop_dist_m = (usrConf.rtm_stop_distance_m > 0) ? usrConf.rtm_stop_distance_m : 10u;
  float dist_m = (float)TinyGPSPlus::distanceBetween(
      gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);
  if (dist_m < (float)stop_dist_m)
  {
    Serial.printf("RTM [RX] Gate 9: reached stop distance (%.1f m < %u m) — clean handoff to manual\n",
                  dist_m, stop_dist_m);
    rtm_rx_active         = false;   // disarm — enter inactive path next tick
    rtm_rx_emergency_stop = false;   // no emergency; motor returns to user throttle immediately
    rtm_approach_cap      = 255;     // clear decel cap so manual throttle is uncapped
    return false;
  }

  return true;
}

// ============================================================
// V2.5-Evo - 2026-07-25 - STAGE 2: HEADING-DISAGREEMENT STATE (guard 2)
//
// Two module-private timers. They are NOT confStruct fields, NOT telemetry, NOT logged as new
// columns — a guard firing is already visible in an existing level-4 log through rtm_source,
// rtm_confidence and cog_frozen_s.
//
// heading_disagree_since_ms : millis() when the COG-vs-compass gap first exceeded
//   kHeadingDisagreeDeg and has held it continuously since; 0 = not currently disagreeing, or no
//   comparison was possible this tick. Same first-exceed-timestamp shape as fm_sep_over_since_ms
//   and fm_diverge_since_ms: plain millis() bookkeeping, no delay, no spin.
//
// heading_disagree_fault : set once that gap has persisted for kHeadingDisagreeMs. This is the
//   ESCALATION — a transient disagreement only invalidates the heading for that tick, but a
//   sustained one means the COMPASS is genuinely wrong (it is the sensor the ladder can promote,
//   and the one this hardware biases under motor current).
//
//   V2.5-Evo - 2026-08-17 - WHAT IT DOES NOW: RTM DEGRADES, FOLLOW-ME DECLINES. While it stands the
//   compass is withdrawn from the ladder, so RTM runs on GPS course alone; it is NOT a config
//   change (nothing writes rtm_use_compass or rtm_compass_required, nothing touches SPIFFS, the
//   rider's stored settings are exactly as they left them) and it is runtime state that dies with
//   the power cycle. Concretely, while the flag stands:
//     - getRtmHeading() withdraws the COMPASS and nothing else. Above rtm_cog_min_speed_kmh the
//       live GPS course is served unchanged at confidence 3, and the kCogHoldMs held-COG bridge is
//       still served at confidence 2 across a short dropout. V2.5-Evo - 2026-08-17: the hold used
//       to be withdrawn here too, on a consistency argument ("the hold is a mode-1 feature"); it is
//       back, because the hold serves a GPS COURSE and this latch is evidence about the COMPASS.
//       Degraded therefore means exactly "mode 1 minus the compass", which is all the evidence
//       supports — see the check below the hold in getRtmHeading();
//     - RTM gate 6 stands aside exactly as it does for a rider who set rtm_compass_required = 0,
//       so an arm can never be refused for lack of a heading — the buggy starts, holds straight,
//       and picks up a course as soon as it is moving. RTM's degraded behaviour is BOUNDED: with no
//       heading the override is pinned at 127 and the align cap holds the throttle at 13/255;
//     - FOLLOW-ME IS DIFFERENT AND DOES NOT DEGRADE. can_be_active carries !heading_disagree_fault
//       and the FM fault-stop classifier carries it too, so FM will not engage while the latch
//       stands and a latch proven mid-engagement ends the run through FM_STOPPING. FM engages
//       autonomously at rider speed plus a margin with continuous steering authority, and the
//       disagreement cannot even be measured during the engagement (the snapshot stops refreshing
//       the moment the trigger is squeezed), so declining is the only honest answer to "one of my
//       two sources is lying and I cannot tell which". See can_be_active in runFmLoop();
//     - mode 2 (compass-only, diagnostic) is the one place there is nothing to fall back to, so the
//       proven-bad compass is withheld and the function returns no heading at all. See there.
//
//   V2.5-Evo - 2026-08-17 - WHERE IT IS CLEARED — EXACTLY THREE ROUTES, and every one of them is
//   EVIDENCE that the problem is gone. The engagement-boundary clears that used to be routes 2, 3
//   and 4 (fmEnterIdle(), the RTM disarm edge, the arm edges) are GONE. They existed only because a
//   standing fault could refuse an RTM arm and would otherwise have been un-clearable on the water;
//   RTM now degrades instead of refusing, so that reason is gone with them — and forgiving a fault
//   at every disarm meant a compass mounted 90 deg out had to be re-proven by coasting on every
//   single run, which is precisely the gap this closes. A mis-mounted compass is wrong every run.
//     1. A SUSTAINED MEASUREMENT SHOWING AGREEMENT — here, in the compare branch below. The
//        evidence is withdrawn, so the verdict goes with it. This still runs while the fault
//        stands, because compare_possible deliberately tests the CONFIGURED mode, not the degraded
//        one; without that, withdrawing the compass would have switched off the very measurement
//        that exonerates it.
//        V2.5-Evo - 2026-08-17 - AND IT NOW TAKES AS MUCH EVIDENCE TO CLEAR AS TO SET. This route
//        used to fire on the FIRST tick where a comparison was possible and the two agreed: four
//        samples over five continuous seconds to convict, one sample to acquit. That asymmetry
//        broke on the case the latch matters most for. A MIRRORED module (an axis sign flipped by
//        the mounting) reports theta - h_true instead of h_true, so the gap between compass and
//        course is 2*h_true - theta, which sweeps the whole circle as the craft turns and passes
//        below kHeadingDisagreeDeg inside two 45-deg-wide windows per revolution. A rider coasting
//        on a heading near one of those windows cleared the latch instantly with the module still
//        mirrored — and mirroring is exactly the fault ?magalign cannot detect, so this route was
//        acquitting the one defect stickiness exists for. Agreement must now be measured
//        CONTINUOUSLY for kHeadingDisagreeMs, with consecutive measured samples no more than
//        kHeadingDisagreeGapMaxMs apart (so at least four of them, the same shape as the set), and
//        that dwell is tracked by heading_agree_since_ms / heading_agree_last_seen_ms below. Any
//        measured disagreement restarts it from zero. Making a clear HARDER can only ever leave a
//        proven fault standing for longer, which is the conservative direction for this guard.
//     2. A SUCCESSFUL ?compasscal (CAL_FULL only) OR ?magalign — the rider physically fixing the
//        mounting and re-measuring it. Wired in Compass.ino via headingDisagreeClearAfterCal();
//        a PARTIAL cal (iron saved, orientation NOT re-measured) and any failed/aborted run do NOT
//        clear it, because those are exactly the runs that leave the old mounting angle in place.
//     3. A REBOOT. The flag is a plain RAM global; it is not persisted anywhere.
//   And separately, the part-finished dwell (not the fault) ages out after
//   kHeadingDisagreeEvidenceMaxMs while no comparison is possible, is restarted from the current
//   sample if the measured disagreements themselves stop being contiguous (kHeadingDisagreeGapMaxMs),
//   and is dropped at both edges of an engagement so a half-built proof never spans two situations.
//   V2.5-Evo - 2026-08-17 - the AGREEMENT dwell added for route 1 obeys the same two bounds, for the
//   same reasons: it ages out after kHeadingDisagreeEvidenceMaxMs while nothing is being measured,
//   and a gap longer than kHeadingDisagreeGapMaxMs between agreeing samples restarts it. It is NOT
//   dropped at engagement edges, and it does not need to be — the gap bound is the tighter test.
//   The measurement goes dormant about a second into any engagement (the snapshot stops refreshing
//   under motor current), so an agreement dwell physically cannot straddle an engagement: the next
//   agreeing sample after the run arrives far more than kHeadingDisagreeGapMaxMs late and opens a
//   fresh dwell of its own.
//
//   V2.5-Evo - 2026-08-17 - WHY THE ENGAGEMENT-BOUNDARY CLEARS WENT, IN FULL, because the argument
//   that put them there is the argument that now takes them out. They were added when a proven
//   fault could REFUSE an engagement: at that point a sticky latch risked becoming an un-clearable
//   arming block on the water, so every engagement boundary forgave it. The evidence window is
//   narrow — a comparison needs a live COG AND a compass snapshot younger than kHeadingCompareSnapMs,
//   and Compass.ino only refreshes that snapshot while thr_received < 25, so the only moments the
//   two inputs coexist are a rider COASTING with the trigger released above rtm_cog_min_speed_kmh
//   and the arm ceremony itself — which meant those boundary clears were throwing away the only
//   evidence the guard ever gets, and the rider had to re-prove a bad compass by coasting on every
//   single run. The reason to keep the verdict is intact: a compass mounted 90 deg out is 90 deg
//   out on the next run too.
//   V2.5-Evo - 2026-08-17 - AND FOLLOW-ME REFUSES ONCE MORE, WHICH DOES NOT BRING THE BOUNDARY
//   CLEARS BACK. The un-clearable-block worry the clears were built for is answered elsewhere and
//   better: RTM is not blocked at all (it runs on GPS course, so the buggy always comes home), and
//   the evidence route is exactly the state an FM rider is in between tows — coasting with the
//   trigger released, which is when the snapshot refreshes and the comparison is live. Forgiving
//   the verdict at a state boundary would hand autonomous following a compass the firmware holds
//   positive evidence against, on the strength of a state change that measured nothing.
//
//   WHY THE FLAG IS NEEDED AT ALL, given a disagreement already returns "no heading": the
//   comparison window is narrow by necessity (see kHeadingCompareSnapMs). The compass snapshot
//   stops refreshing the moment the rider squeezes the trigger, so a real disagreement can be
//   measurable on one tick and unmeasurable on the next — at which point COG alone would look
//   valid again and FM would happily resume steering on the source we just proved suspect. The
//   flag is what stops a proven fault from being washed out by its own evidence going stale.
//
//   RTM CONSUMES IT TOO (this paragraph used to say only FM did). The 2026-08-16 close-range veer
//   changed that: getRtmHeading() itself withdraws the compass while the fault is latched, so every
//   caller inherits it — RTM, FM and anything added later. The original objection was that a sticky
//   fault could become an un-clearable arming block on the water. V2.5-Evo - 2026-08-17: that
//   objection is answered at the root instead of by forgetting the evidence — the fault cannot
//   block an RTM arm any more, because gate 6 stands aside while it stands and RTM simply runs on
//   GPS course. The worst case for RTM is therefore a session on GPS course alone, not an aborted
//   run and not a refusal. V2.5-Evo - 2026-08-17: FOLLOW-ME IS THE DELIBERATE EXCEPTION — it
//   declines to engage while the latch stands (see can_be_active in runFmLoop for why autonomous
//   steering must not be handed a heading the firmware holds positive evidence against), so for FM
//   the worst case is a manual buggy, which is the state it starts every session in. The way out is
//   the same for both: fix the compass (?compasscal / ?magalign), or let a sustained measurement
//   show the two sources agreeing again.
//
// CONCURRENCY: every caller of getRtmHeading() (gate 6, checkFmFaultConditions, the fm_status
// telemetry block, updateRtmSteering) runs in the loop task, so these statics are single-writer.
// Logger.ino's duplicate of this logic runs in loggerTask and READS the fault through the
// headingDisagreeLatched() accessor only — a read that cannot set, clear or age anything — so that
// mirror stays side-effect-free while still describing the same heading the controller chose.
// Compass.ino calls headingDisagreeClearAfterCal() from the ?compasscal / ?magalign commands, which
// run in the loop task as well (System.ino dispatches them there), so the single-writer property
// holds for that clear route too.
// ============================================================
static unsigned long heading_disagree_since_ms = 0;
// V2.5-Evo - 2026-08-17 - volatile. It is written only by the loop task (single-writer, as the
// CONCURRENCY note above says) but it is READ from loggerTask, through headingDisagreeLatched().
// As a file-scope static whose address never escapes this translation unit, the compiler is free to
// keep it in a register across a loop and never re-read the memory the other task can see change.
// That read is log columns only — it decides nothing, caps nothing, gates nothing — so this is a
// correctness tidy on a diagnostic path, not a control fix. Costs one memory access per read.
static volatile bool heading_disagree_fault    = false;

// V2.5-Evo - 2026-08-16 - millis() of the most recent tick on which a disagreement was actually
// MEASURED — as opposed to merely still standing on record. 0 = nothing measured. It is only
// meaningful while heading_disagree_since_ms is non-zero, and it is cleared everywhere that is,
// so the two can never disagree about whether a dwell is running. See kHeadingDisagreeGapMaxMs
// below for what it is for and why the dwell needed a second, finer bound than the span cap.
static unsigned long heading_disagree_last_seen_ms = 0;

// V2.5-Evo - 2026-08-17 - THE MIRROR IMAGE OF THE TWO ABOVE, for clear route 1.
//
// heading_agree_since_ms : millis() when the two sources were first measured AGREEING and have
//   kept agreeing continuously since; 0 = not currently in an agreement dwell.
// heading_agree_last_seen_ms : millis() of the most recent tick on which agreement was actually
//   MEASURED. Only meaningful while heading_agree_since_ms is non-zero, and cleared everywhere
//   that is, so the pair can never disagree about whether a dwell is running — the same invariant
//   the disagreement pair keeps.
//
// WHY THEY EXIST. Convicting the compass needs kHeadingDisagreeMs of continuous measured
// disagreement, which is at least four samples; acquitting it used to need one agreeing tick. That
// let a mirrored module walk free every time the craft's heading wandered into one of the two
// windows per revolution where a mirrored reading happens to land within kHeadingDisagreeDeg of the
// true course. Symmetric evidence, symmetric bookkeeping: same dwell length, same continuity bound,
// same ageing rule. No new constant, no confStruct field, no SPIFFS slot, no web-UI row.
//
// Single-writer from the loop task, exactly like the pair above: only getRtmHeading() writes them,
// plus headingDisagreeClearAfterCal() which resets the whole set together.
static unsigned long heading_agree_since_ms     = 0;
static unsigned long heading_agree_last_seen_ms = 0;

// ------------------------------------------------------------
// headingDisagreeLatched - has a heading disagreement been PROVEN and not yet withdrawn?
// ------------------------------------------------------------
// V2.5-Evo - 2026-08-17. Inputs: none. Returns: the current value of heading_disagree_fault.
// Side effects: NONE — it does not set, clear or age the latch or its dwell.
//
// It exists so checkRtmSafetyGates(), which is defined above this declaration, can ask whether the
// session is degraded — gate 6 stands aside while it is. Forward-declared next to that gate; see
// the note there. Logger.ino reads it through the same accessor for the log mirror.
// ------------------------------------------------------------
static bool headingDisagreeLatched()
{
  return heading_disagree_fault;
}

// ============================================================================================
// Latch persistence - the verdict survives a power cycle
// ============================================================================================
// V2.5-Evo - 2026-08-18 - LATCH-1. heading_disagree_fault lived only in RAM, so a power cycle
// forgot it. A rider whose compass is genuinely mis-mounted saw the warning, switched off,
// switched on, and Follow-Me would engage again on the compass that had just been caught
// disagreeing with GPS course. The safety check was defeated by turning it off and on again,
// which is the first thing anyone does when something misbehaves.
//
// STORED AS A MARKER FILE, NOT A confStruct FIELD. Adding a field changes sizeof(confStruct),
// which bumps SW_VERSION, which wipes every owner's stored config on the next flash - an
// enormous cost for one bit. The file's EXISTENCE is the latch; its contents are only there so
// the boot log can say when and why.
//
// WHY PERSISTING IS NOT HARSH. The latch already clears on FIVE SECONDS OF MEASURED AGREEMENT,
// with no rider action at all. So a false positive - passing a steel bridge, a transient - heals
// by itself the next time the buggy coasts with the throttle released and the two sources agree.
// Persisting costs nothing to the innocent case and closes the power-cycle hole in the guilty
// one. The other two escapes, a FULL ?compasscal and a completed ?magalign, are unchanged.
//
// FAIL-SAFE DIRECTION: a SPIFFS write that fails leaves the fault set in RAM for this session,
// which is the conservative outcome. It is never treated as a clear.
#define HDG_FAULT_FILE "/hdg_fault"

static void headingDisagreePersist(bool latched)
{
  if (latched) {
    File f = SPIFFS.open(HDG_FAULT_FILE, FILE_WRITE);
    if (!f) {
      // Not fatal and deliberately not escalated: the fault still stands for THIS session, which
      // is the safe direction. Only the survival across a reboot is lost.
      Serial.println("HEADING [RX] WARNING: could not persist the disagreement latch to SPIFFS. "
                     "It still applies now, but will not survive a reboot.");
      return;
    }
    f.printf("heading_disagree_fault at %lu ms uptime\n", (unsigned long)millis());
    f.close();
  } else {
    if (SPIFFS.exists(HDG_FAULT_FILE)) SPIFFS.remove(HDG_FAULT_FILE);
  }
}

// Called ONCE from setup(), after SPIFFS is mounted and before RTM/FM can arm.
static void headingDisagreeRestore()
{
  if (!SPIFFS.exists(HDG_FAULT_FILE)) return;

  // ==========================================================================================
  // V2.5-Evo - 2026-08-20 - LATCH-2. ONLY RESTORE A VERDICT THE RIDER CAN STILL CLEAR.
  //
  // Found by Rex auditing LATCH-1 (shipped the same day, 2026-08-18). LATCH-1 made the latch
  // survive a reboot, and in doing so removed clear route 3 — the reboot itself. That is correct
  // in mode 1 with a compass fitted, where routes 1 and 2 both work. It is a TRAP everywhere
  // else, because BOTH remaining routes need a working compass in hybrid mode:
  //
  //   Route 1, five seconds of measured agreement, runs only inside compare_possible, which
  //   begins `(mode == 1) && cog_valid && ...` — so on a COG-only board it can NEVER fire.
  //   Route 2, ?compasscal / ?magalign, needs a compass that answers.
  //
  // So a rider who latched the fault in hybrid mode and then set rtm_use_compass 0, or whose
  // compass has since failed or been unplugged, would boot with Follow-Me blocked and NO route
  // out of it — permanently, on every subsequent boot, with the only escape being a compass they
  // no longer use or no longer have. Unrecoverable in the field is worse than the risk the latch
  // was guarding against, and LATCH-1 created it.
  //
  // The stale marker is DELETED rather than left in place, so the trap cannot re-arm itself on
  // the next boot. If the rider returns to hybrid mode with a working compass, a genuine
  // disagreement re-proves itself within kHeadingDisagreeMs of coasting — the evidence is cheap
  // to regather, which is exactly why it is safe to drop a verdict that can no longer be tested.
  //
  // TRADE-OFF, STATED PLAINLY: a rider could dodge a legitimate latch by setting
  // rtm_use_compass 0. That is not a new hole. Rex confirmed a mode-0 board cannot set the latch
  // in the first place, and cfgValidateCrossField() already force-zeroes rtm_compass_required for
  // that rider class — COG-only is a deliberate, documented, supported configuration, and a rider
  // who selects it has explicitly said the compass is not in use. This removes an unrecoverable
  // state; it does not create a bypass that was not already there by design.
  // ==========================================================================================
  if (usrConf.rtm_use_compass != 1 || !compass_detected) {
    SPIFFS.remove(HDG_FAULT_FILE);
    Serial.printf("HEADING [RX] a stored heading-disagreement verdict was found but DISCARDED: "
                  "%s, so neither route that could clear it is available. Follow-Me is not "
                  "blocked.\n",
                  (usrConf.rtm_use_compass != 1) ? "this board is configured COG-only "
                                                   "(rtm_use_compass 0)"
                                                 : "no compass is detected");
    return;
  }

  heading_disagree_fault = true;
  // heading_degrade_announced is deliberately left false, so the standing degradation notice is
  // printed once on this boot too. A rider who power-cycled needs telling again, not silence.

  Serial.println("HEADING [RX] a heading disagreement was PROVEN before the last reboot and has "
                 "not been withdrawn. Return-to-Me runs on GPS course only and Follow-Me will "
                 "not engage.");
  Serial.println("HEADING [RX] it clears on its own after 5 s of the compass and GPS course "
                 "measured agreeing while coasting, or immediately after a full ?compasscal or "
                 "a completed ?magalign.");
}

// V2.5-Evo - 2026-08-17 - true once the rider has been TOLD, on serial, that this session has been
// degraded to GPS-course-only. Reset by every clear route, so a fault that latches again later
// announces itself again. Notification bookkeeping only: nothing reads it as a control input.
static bool heading_degrade_announced = false;

// ------------------------------------------------------------
// headingDisagreeAnnounceDegraded - tell the rider, once, that the compass has been withdrawn
// ------------------------------------------------------------
// V2.5-Evo - 2026-08-17. Inputs: none. Outputs: none. Side effects: one Serial block, and it sets
// heading_degrade_announced so it cannot repeat.
//
// WHY IT EXISTS. The degradation is otherwise invisible from the bench: the buggy still arms, still
// runs, and simply has no heading until it is moving faster than rtm_cog_min_speed_kmh. A rider who
// does not know why deserves to be told what happened, what the craft will do now, and how to fix
// it — so the message says all three rather than naming a fault code.
//
// RATE LIMIT. It prints on the transition into the degraded state and never again while that state
// stands, which is the strongest rate limit available: the caller sits inside a 10 Hz ladder that
// is itself called several times per tick, so anything weaker would flood the UART. The flag is
// re-armed only when the fault is genuinely cleared (agreement, calibration).
//
// V2.5-Evo - 2026-08-17 - AND IT WAITS FOR THE TRIGGER TO BE RELEASED. WHY: the flag can once again
// end a Follow-Me engagement (can_be_active carries it), and this function is reached from
// checkFmFaultConditions() — i.e. BEFORE fm_throttle_cap = 0 is written on that same tick. Printing
// four lines there would put a possibly-blocking Serial call between a proven fault and the motor
// reaching 0, which is exactly what the F7 rule forbids. So while thr_received >= 25 it returns
// WITHOUT setting its flag, and the notice prints on the first tick after the rider lets go — by
// which time the deadman already has the motor at 0. The message is not lost, only deferred, and
// ?diag reports the same state on demand at any moment.
//
// V2.5-Evo - 2026-08-17 - AND THE RETRY IS NOW WHERE IT HAS TO BE FOR THAT SENTENCE TO BE TRUE.
// "Prints on the first tick after the rider lets go" assumed something would call this again on
// that tick. Nothing did: the only caller was the if (disagree_now) branch in getRtmHeading(), so a
// retry required a fresh MEASURED disagreement — which needs a compass snapshot that only refreshes
// with the trigger released, so a rider who stayed on the throttle for the rest of the session
// never produced one and the notice was LOST, not deferred. getRtmHeading() now calls this
// unconditionally at its top while the fault stands, so the retry happens on the first released-
// trigger tick regardless of what is measurable. The deferral above is unchanged.
//
// It writes NO control variable: not a cap, not a gate, not the override, not usrConf.
// ------------------------------------------------------------
static void headingDisagreeAnnounceDegraded()
{
  if (heading_degrade_announced) return;
  if (thr_received >= 25) return;   // F7: never print while the motor can be live — retry on release
  heading_degrade_announced = true;

  Serial.printf("HEADING [RX] COMPASS WITHDRAWN: the compass disagreed with the GPS course by more than %.0f deg for %lu ms.\n",
                (double)kHeadingDisagreeDeg, (unsigned long)kHeadingDisagreeMs);
  Serial.println("        Return-to-Me now steers on GPS COURSE ONLY for this session. Your stored settings are UNCHANGED.");
  Serial.println("        FOLLOW-ME WILL NOT ENGAGE while this stands: one of the two heading sources is wrong and the board cannot tell which, so it will not steer at you on a guess.");
  Serial.printf("        Expect NO heading, and straight-ahead steering, whenever the buggy is slower than %u km/h and the last good course is more than 3 s old.\n",
                (unsigned)usrConf.rtm_cog_min_speed_kmh);
  Serial.println("        To get the compass back: re-run ?compasscal (a full two-circle run) or ?magalign. A reboot does NOT clear it (the verdict is stored in SPIFFS since 2026-08-18). It also clears on its own once the two sources are measured AGREEING for 5 continuous seconds.");
}

// ------------------------------------------------------------
// headingDisagreeClearAfterCal - the rider has re-measured the compass, so drop the verdict
// ------------------------------------------------------------
// V2.5-Evo - 2026-08-17. Inputs: what - a short name of the command that earned the clear, printed
// so the log says which one it was. Outputs: none. Side effects: clears heading_disagree_fault and
// its dwell bookkeeping, re-arms the degradation notice, prints one line.
//
// WHY THIS IS A CLEAR ROUTE AT ALL, when engagement boundaries are not. The latch is evidence about
// the COMPASS. A disarm is not evidence about anything — the compass is exactly as wrong after it
// as before. A successful ?compasscal or ?magalign IS: the rider has physically re-measured the
// mounting, so the number the verdict was formed against no longer exists. This is the escape hatch
// that keeps a sticky latch honest: fix the compass and you get hybrid mode back on the spot, with
// no reboot and no coasting to re-prove anything.
//
// CALLER'S CONTRACT (enforced in Compass.ino, not here): call this ONLY from a run that actually
// re-measured the mounting — CAL_FULL from runCompassCalibration(), or a completed runMagAlign().
// A PARTIAL cal keeps the OLD mag_orientation, which is usually the very thing that caused the
// disagreement, so it must not clear it. This function cannot check that for itself; it is a
// deliberate, documented split.
//
// It writes NO control variable: no cap, no gate, no override, and nothing in usrConf.
// ------------------------------------------------------------
static void headingDisagreeClearAfterCal(const char *what)
{
  if (!heading_disagree_fault && heading_disagree_since_ms == 0) return;  // nothing to forgive

  heading_disagree_since_ms     = 0;
  heading_disagree_last_seen_ms = 0;
  heading_disagree_fault        = false;
  headingDisagreePersist(false);         // LATCH-1: the stored verdict goes with it
  heading_degrade_announced     = false;
  // V2.5-Evo - 2026-08-17 - the agreement dwell goes with them. It is bookkeeping toward a clear
  // that has just happened by another route, so carrying it forward would describe a measurement
  // of a mounting that no longer exists. Pair cleared together, like every other dwell here.
  heading_agree_since_ms        = 0;
  heading_agree_last_seen_ms    = 0;

  Serial.printf("HEADING [RX] compass re-measured by %s — the heading-disagreement latch is cleared and hybrid heading is available again.\n",
                (what != NULL) ? what : "calibration");
}

// V2.5-Evo - 2026-08-16 - The oldest part-finished disagreement proof that may still be counted.
// It exists because the "no comparison possible" branch of getRtmHeading() FREEZES
// heading_disagree_since_ms instead of clearing it (so a dwell survives a COG dropout mid-proof).
// Frozen without a limit, the timestamp stops measuring a sustained condition and starts measuring
// the wall clock: two one-tick disagreements minutes apart would satisfy "held continuously for
// kHeadingDisagreeMs" and escalate, which is not what the constant is documented to mean.
// 3x the dwell is the smallest multiple that is comfortably clear of the case the freeze was
// written for - a real disagreement measurable either side of one handover, which spans a second
// or two - while refusing to treat evidence from a different minute as part of the same event.
// This is a compile-time constant, like every other k* in this file: no confStruct field, no SPIFFS
// slot, no web-UI row, no SW_VERSION bump.
static const unsigned long kHeadingDisagreeEvidenceMaxMs = 3UL * (unsigned long)kHeadingDisagreeMs;  // ms (15 s at the shipped 5 s dwell)

// V2.5-Evo - 2026-08-16 - The largest gap allowed BETWEEN two measured disagreements for them to
// count as one continuous event.
// WHAT THE SPAN CAP ABOVE STILL LET THROUGH. kHeadingDisagreeEvidenceMaxMs bounds the total SPAN of
// a dwell, measured from heading_disagree_since_ms — the START of it — and it is only ever tested on
// ticks where no comparison is possible. So: one disagreeing tick at t = 0 opens the dwell; the
// comparison is impossible for the next fourteen seconds; one more disagreeing tick arrives at
// t = 14 s. The span test is (14000 - 0) > 15000, which is false, so the evidence survives — and
// then (now - since) >= kHeadingDisagreeMs is true on that same tick, because the dwell clock has
// been frozen the whole time. A fault built from two isolated one-tick samples fourteen seconds
// apart: precisely the defect the span cap was added to kill, merely bounded at 15 s instead of
// unbounded. The consequence is a nuisance FM fault-stop mid-wave, not a danger — but it is still
// the guard reporting something it did not measure.
// WHAT THIS ADDS. Continuity. A dwell only carries forward while the disagreeing samples KEEP
// ARRIVING; a longer silence than this and the next disagreement opens a fresh dwell from itself.
// Span is not evidence of persistence — arrival rate is — so the guard now bounds both.
// WHY 2000 ms. It has to sit comfortably above every gap that is a genuine hiccup in the two inputs
// a comparison needs, and comfortably below the tens of seconds the counterexample depends on. The
// compass snapshot is only comparable for kHeadingCompareSnapMs (1000 ms) after a motor-idle
// capture, and the COG freshness gate in getRtmHeading() is 1500 ms; 2000 ms clears the looser of
// those two, so any gap longer than this means an input was genuinely ABSENT, not merely late —
// which is the ordinary COG dropout the freeze exists for, and one of those spans a second or two,
// not tens of seconds. At the 10 Hz ladder cadence a truly continuous disagreement re-measures every
// ~100 ms, so 2000 ms is ~20 consecutive missed opportunities before continuity is called broken:
// nothing continuous comes near it. And it forces at least four measured samples to span the 5 s
// dwell, so the two-samples-far-apart fault is no longer reachable at any spacing.
// DIRECTION OF THE CHANGE: strictly harder to set the fault, never easier. Restarting a dwell can
// only delay an escalation, and the per-tick verdict (a measured disagreement returns confidence 0
// on that tick regardless) is untouched, so no tick loses protection.
// Compile-time constant like every other k* in this file: no confStruct field, no SPIFFS slot, no
// web-UI row, no SW_VERSION bump.
static const unsigned long kHeadingDisagreeGapMaxMs = 2000UL;  // ms; max spacing between two disagreeing samples


// V2.5-Evo - 2026-08-16 - Last-good COG, held briefly across a dropout.

// RTM governs the craft to rtm_target_speed_kmh (default 4.0 km/h) while COG is abandoned

// below rtm_cog_min_speed_kmh (default 3). That 1 km/h margin is 0.278 m/s, which is inside

// the speed signal's OWN noise - so the heading source flips on sensor noise alone, before any

// wave or gust. Every flip on a rotated compass injects a large heading error.

// Holding the last good COG across a short dip removes the flapping without changing either

// threshold, and without inventing a heading: a course from 2 seconds ago on a craft doing

// 4 km/h is a far better estimate than a compass that is 87-100 deg wrong under motor load.

// V2.5-Evo - 2026-08-16 - These two are deliberately NOT static any more. Logger.ino's duplicate
// heading ladder READS them (it never writes them) so the rtm_source / rtm_confidence columns can
// report a held COG instead of misreporting it as a compass reading or as no source at all. Same
// export pattern as g_heading_error_dx10 above: a plain global here, picked up by a local extern
// in Logger.ino. Single writer - getRtmHeading(), loop task; loggerTask only reads, and the worst
// a preemption between the two reads can cost is ~100 ms of apparent age in a log column, never a
// control decision.
float                cog_last_good_deg = -1.0f;

unsigned long        cog_last_good_ms  = 0;


// V2.5-Evo - 2026-05-06 - D5: Layered heading source for RTM steering.
//
// Returns the best available heading (deg, 0-360 clockwise from North) based on
// usrConf.rtm_use_compass mode and current sensor state. Three modes:
//   0 = GPS COG only — no compass fallback. Safest choice for builds where compass
//       is biased by motor current (this hardware's bench-tested behavior).
//   1 = Hybrid (DEFAULT) — GPS COG primary; compass snapshot when buggy is too slow
//       for COG to be reliable. Compass snapshot is updated only when motor is idle
//       (thr_received < 25), so it represents an unbiased reading.
//   2 = Compass only — DIAGNOSTIC ONLY. Should NOT be used on water on builds with
//       known motor EMI. Available for non-EMI builds with proven clean compass
//       behavior under load.
//
// Confidence levels (output param):
//   3 = HIGH:   GPS COG, fresh and above min_speed threshold
//   2 = MEDIUM: compass snapshot < 1000ms old, or compass-only mode (legacy)
//   1 = LOW:    compass snapshot 1000-8000ms old (degraded — caller should reduce steering authority)
//   0 = NONE:   no valid heading source — caller must hold straight (rtm_steer_override = 127)
//
// Returns true if heading is valid (any non-zero confidence), false if no source.
//
// V2.5-Evo - 2026-07-25 - STAGE 2: two trust guards now sit inside the COG branch. Neither can
// ever make a heading MORE trusted — each can only downgrade a source to "unusable":
//   GUARD 1 (COG liveness by value change): a COG whose VALUE has not moved for kRtmCogFrozenMs
//     while the buggy is moving is not a heading, it is a repeated number. Speed-gated, so a
//     stationary buggy — whose constant course is CORRECT — is never faulted.
//   GUARD 2 (compass vs COG cross-check): if the two independent sources are more than
//     kHeadingDisagreeDeg apart, neither is trusted and NO heading is returned.
//
// V2.5-Evo - 2026-08-17 - A TEMPORARY LADDER THAT IS NOT A SETTING: while guard 2's latch stands,
// the COMPASS is withdrawn from this function — the live GPS course and the kCogHoldMs held course
// are both still served, so the ladder becomes "mode 1 minus the compass" rather than any stored
// mode. usrConf is READ, never written — the rider's rtm_use_compass and rtm_compass_required
// survive untouched, and the withdrawal disappears on the next boot or the moment the compass is
// re-measured or exonerated. Mode 2 is the exception: it has no COG to fall back on, so a standing
// fault there returns no heading at all.
// WHY THE HOLD STAYS (this was the other way round for one pass): the hold serves a last-good GPS
// COURSE and the latch is evidence about the COMPASS, so withdrawing the hold on that evidence
// removed something the guard has nothing against — and it cost a real bridge across the ordinary
// COG dropout. See the check that now sits BELOW the hold.
//
// SAFETY: This function still never modifies sensor state, config, telemetry or any control
//         variable. The only state it writes is the guard-2 bookkeeping declared directly above —
//         heading_disagree_since_ms, heading_disagree_last_seen_ms and heading_disagree_fault
//         (V2.5-Evo - 2026-08-16: last_seen added; the list previously named two of them),
//         V2.5-Evo - 2026-08-17 the heading_agree_since_ms / heading_agree_last_seen_ms pair that
//         measures the agreement dwell for clear route 1, plus the heading_degrade_announced
//         notification flag it sets through headingDisagreeAnnounceDegraded() — and the
//         cog_last_good_deg / cog_last_good_ms pair used by the COG hold. All of it is single-writer from the loop task.
//         Caller (updateRtmSteering) must handle confidence=0 as a hold-straight
//         scenario, not as a steering command.
static bool getRtmHeading(float* out_heading, uint8_t* out_confidence)
{
  uint16_t mode           = usrConf.rtm_use_compass;
  uint16_t cog_min_speed  = usrConf.rtm_cog_min_speed_kmh;
  unsigned long now       = millis();

  // ============================================================
  // V2.5-Evo - 2026-08-17 - THE DEGRADATION NOTICE RETRIES HERE, EVERY TICK, UNTIL IT PRINTS.
  //
  // WHAT WAS WRONG. headingDisagreeAnnounceDegraded() defers while the trigger is held and returns
  // WITHOUT setting its one-shot flag, which is correct (see the F7 note on the function). But its
  // only call site was inside the if (disagree_now) branch far below — so the retry needed another
  // MEASURED disagreement, and a measurement needs a live COG together with a compass snapshot
  // younger than kHeadingCompareSnapMs, which only refreshes while thr_received < 25. Put those two
  // facts together and the deferral was not a deferral: if the dwell completed inside the ~1 s
  // window after a squeeze the notice was suppressed, and if the rider then finished the session
  // under power and never coasted above rtm_cog_min_speed_kmh again, no further comparison was ever
  // possible and THE NOTICE WAS LOST. They rode the whole session with the compass withdrawn and
  // Follow-Me refusing to engage, with no indication anywhere except a manual ?diag.
  //
  // WHAT THIS DOES. Decouples the retry from measurability. While the verdict stands, every call of
  // this function offers to announce it, whether or not anything is currently comparable. Nothing
  // about the notice itself changes: heading_degrade_announced still makes it print exactly once per
  // fault, and the thr_received >= 25 deferral inside the function is untouched, so it still returns
  // early without arming that flag while the motor can be live.
  //
  // WHY THIS PLACEMENT CANNOT PUT A SERIAL CALL BETWEEN A PROVEN FAULT AND A MOTOR-STOPPING WRITE.
  // The function prints only when thr_received < 25, and every motor-relevant write this guard feeds
  // is only consequential while the trigger is HELD — calcPWM() applies rtm_steer_override and the
  // caps under thr_received >= 25, and below that the deadman already has the motor at 0. So on any
  // tick where this can print there is nothing left to defer. Call site by call site:
  //   gate 6 in checkRtmSafetyGates() — unreachable below 25 counts, because gate 1 returns first,
  //     so this line prints NOTHING there and cannot precede that gate's rtm_rx_emergency_stop;
  //   checkFmFaultConditions() — does run on released-trigger ticks and is upstream of
  //     fm_throttle_cap = 0, which is exactly the case the deferral was written for and exactly the
  //     case where cap 0 is redundant, the trigger being released already;
  //   updateRtmSteering() — reached only with the trigger held (RTM past its gates, or FM_ACTIVE
  //     via hard_ok), so again nothing prints;
  //   the fm_status telemetry call in runRtmLoop() — has no motor-stopping write downstream of it.
  // Placed ABOVE the no-fix guard below on purpose: a fault that is standing must be announceable
  // even on a tick with no usable GPS, which is the whole point of decoupling it from measurement.
  //
  // It writes no control variable — not a cap, not a gate, not the override, not usrConf.
  // ============================================================
  if (heading_disagree_fault) headingDisagreeAnnounceDegraded();

  // ---- No-fix engagement guard (2026-07-19 FM triage, Fable audit §5) ----
  // A heading source is only meaningful for RTM/FM steering when the RX has a real
  // GPS position fix: the steering bearing is computed from gps_last_lat/lng, which are
  // 0,0 with no fix. The Fable log showed RTM reporting confidence=2 from a compass
  // snapshot while datetime_unix=0 (no fix) — that must never engage or be reported.
  // Require a fresh RX GPS fix (same age window as Gate 5) before granting ANY confidence.
  // A stationary buggy that HAS a fix still passes (gps_last_ms updates while stopped).
  if (gps_last_ms == 0 || (now - gps_last_ms) > 6000UL) {
    *out_heading = -1.0f;
    *out_confidence = 0;
    return false;
  }

  // ---- Mode 2: Compass only (legacy/diagnostic) ----
  // Use compass directly; valid only if compass returns non-error.
  // SAFETY: This mode SHOULD NOT be used on water — see field-service note in BREmote_V2_Rx.h.
  //
  // V2.5-Evo - 2026-08-17 - A LATCHED DISAGREEMENT LEAVES THIS MODE WITH NOTHING. Elsewhere the
  // fault withdraws the COMPASS and leaves the GPS course (live and held) in place. Mode 2 never
  // consults COG at all, so the only source it has is the one the guard has measured 45+ deg away
  // from the other. Handing that out would be steering a towing buggy at a person on an instrument
  // we have evidence against, and "the operator opted in" is not consent to a sensor that has since
  // been shown to conflict — they opted into a compass they believed was good.
  // SO WE WITHHOLD IT: no heading, confidence 0, and every consumer holds straight. That is
  // conservative in the direction this whole guard runs — it subtracts a source, it never adds one
  // — and it is NOT a refusal to operate: gate 6 stands aside while the latch stands, so RTM still
  // arms and still runs on the rider's throttle, holding straight. FM will not engage — for two
  // independent reasons now: can_be_active carries the fault (V2.5-Evo - 2026-08-17), and FM has
  // always required a heading to exist at all (condition 6).
  // HOW IT CAN ARISE AT ALL: the comparison that sets the fault only runs in mode 1, so this state
  // needs a rider who latched a fault in hybrid mode and then switched to mode 2 in the same power
  // cycle. HOW TO GET OUT: ?compasscal / ?magalign. A reboot does NOT clear it (the verdict is stored in SPIFFS since 2026-08-18). Route 1 (a live measurement
  // showing agreement) cannot fire here, because no comparison is made in mode 2 — which is another
  // reason not to trust the compass in the meantime: in this mode nothing can exonerate it.
  if (mode == 2) {
    if (heading_disagree_fault) {
      *out_heading = -1.0f;
      *out_confidence = 0;
      return false;
    }
    float h = getCompassHeading();
    if (h < 0.0f) {
      *out_heading = -1.0f;
      *out_confidence = 0;
      return false;
    }
    *out_heading = h;
    *out_confidence = 2;  // MEDIUM — known biased under load but user opted in
    return true;
  }

  // ---- GPS COG (preferred for modes 0 and 1) ----
  // Valid if: course was captured (ms > 0), course is in valid range,
  //           course age < 1500ms, GPS speed >= cog_min_speed_kmh,
  //           and (STAGE 2 guard 1) the course VALUE is not frozen while moving.
  bool cog_captured = (gps_last_course_ms > 0) && (gps_last_course_deg >= 0.0f);
  bool cog_fresh_ts = cog_captured && ((now - gps_last_course_ms) < 1500UL);
  bool cog_moving   = (gps_last_speed_kmh >= (float)cog_min_speed);

  // ============================================================
  // GUARD 1 — COG LIVENESS BY VALUE CHANGE, NOT BY TIMESTAMP
  // V2.5-Evo - 2026-07-25 - STAGE 2
  //
  // WHAT THE BUG WAS. The four sub-conditions above ask "is a course present, in range, recent,
  // and are we fast enough for it to mean anything". None of them asks the only question that
  // actually matters: IS IT STILL MOVING. gps_last_course_ms is re-stamped by GPS.ino on every
  // valid course sentence, including a sentence that repeats the previous heading exactly. The
  // owner's bench ?diag caught precisely that: 7.4 timestamp-updates/s against 0.0
  // value-changes/s, one heading held for 533 s. Every tick of those nine minutes this branch
  // returned confidence 3 (HIGH) on a number that had not moved since before the run started,
  // and the steering controller drove on it. That is the Follow-Me veer.
  //
  // WHAT THE FIX DOES. It reuses the Stage 0 tracker rather than duplicating it: GPS.ino already
  // stamps g_diag_cog_change_ms every time the course VALUE moves by more than kDiagCogChangeDeg
  // (0.05 deg). If that stamp is older than kRtmCogFrozenMs, the number is frozen and COG is not
  // a heading source this tick. Nothing in GPS.ino is touched — this is a read.
  //
  // WHY IT IS SPEED-GATED, AND WHY THAT IS NOT OPTIONAL. A stationary buggy legitimately reports
  // the same course forever: that is a correct reading, not a fault, and faulting it would break
  // the low-speed RTM arm the owner relies on (it is exactly why the compass snapshot fallback
  // exists). So the freeze verdict is only formed while gps_last_speed_kmh >= cog_min_speed — the
  // same gate this branch already applies. Below that speed the guard does not exist, and COG is
  // rejected for being too slow exactly as it always was, falling through to the compass.
  //
  // WHY IT ALSO REQUIRES A FRESH TIMESTAMP. The verdict "the value is frozen" is only meaningful
  // while the module is actively pushing course sentences — that is the measured signature
  // (timestamp fresh, value stuck). If the timestamp itself has gone stale, that is the ordinary
  // stale-COG case which already falls through to the compass, and it is left behaving exactly as
  // it does today. This keeps guard 1 aimed at the one failure it was written for.
  //
  // SENTINEL: g_diag_cog_change_ms == 0 means no COG value has EVER been captured this session
  // (?diagz deliberately does not clear it). That cannot coexist with cog_fresh_ts, but if it
  // somehow did we form NO freeze verdict — we never suppress a source on the strength of a
  // contradiction, only on positive evidence.
  // ============================================================
  bool cog_frozen_moving = false;
  if (cog_moving && cog_fresh_ts) {
    unsigned long cog_change_ms = (unsigned long)g_diag_cog_change_ms;
    cog_frozen_moving = (cog_change_ms != 0) &&
                        ((now - cog_change_ms) >= (unsigned long)kRtmCogFrozenMs);
  }

  bool cog_valid = cog_fresh_ts && cog_moving && !cog_frozen_moving;

  // ============================================================
  // GUARD 2 — COMPASS vs COG DISAGREEMENT
  // V2.5-Evo - 2026-07-25 - STAGE 2
  //
  // WHAT IT DOES. When both independent heading estimates are available AND simultaneous, it
  // measures the shortest angular distance between them. Beyond kHeadingDisagreeDeg they cannot
  // both be describing this buggy, so NEITHER is trusted: the function returns no heading
  // (confidence 0) and updateRtmSteering() holds straight. It deliberately does NOT pick a winner
  // — the entire lesson of the veer is that silently preferring the survivor is how you end up
  // steering on the wrong one.
  //
  // WHY ONLY IN MODE 1 (HYBRID). Mode 1 is the only mode in which the ladder can actually PROMOTE
  // the compass to a steering source, so it is the only mode where a bad compass can take the
  // buggy anywhere. In mode 0 the operator has declared the compass untrusted and it is excluded
  // from the ladder entirely — letting that same excluded sensor veto a good COG would invert the
  // meaning of the setting and import faults into the mode chosen to avoid them (and mode 0 needs
  // no help here: with COG gone it already returns nothing). Mode 2 returns above this point and
  // never consults COG at all; it is documented DIAGNOSTIC-ONLY, not for water.
  //
  // WHY THE COMPASS SNAPSHOT AND NOT A LIVE READ. getCompassHeading() is an I2C transaction that
  // takes the shared i2cMutex, and this function is called several times per 10 Hz tick from four
  // call sites — that is 40-60 extra bus transactions a second inserted into the control loop.
  // Worse, a live compass read under motor current is exactly the EMI-biased value this hardware
  // is known for, so it would manufacture disagreements and fault every engagement. The snapshot
  // is captured motor-idle by Compass.ino, which is the only compass reading this firmware trusts.
  //
  // TIMING HONESTY. Because the snapshot only refreshes with the trigger released, the comparison
  // is live around the moment of the squeeze and goes dormant while the rider holds it (see
  // kHeadingCompareSnapMs). Dormant means "not measured", never "measured and passed".
  // ============================================================
  bool compare_possible = (mode == 1) && cog_valid &&
                          (compass_snapshot_heading >= 0.0f) && (compass_snapshot_ms > 0) &&
                          ((now - compass_snapshot_ms) < (unsigned long)kHeadingCompareSnapMs);

  bool disagree_now = false;
  if (compare_possible) {
    // Shortest angular distance, wrap-correct across 0/360 (e.g. 359 vs 1 is 2 deg, not 358).
    // Written out inline rather than calling fmAngleDiff(): that helper is defined ~900 lines
    // below in this same file, and a safety guard should not depend on the Arduino builder's
    // auto-prototype ordering to compile.
    float d = gps_last_course_deg - compass_snapshot_heading;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    disagree_now = (fabsf(d) > kHeadingDisagreeDeg);
  }

  // Persistence bookkeeping. Transient disagreement -> this tick has no heading. Sustained past
  // kHeadingDisagreeMs -> a proven fault.
  // V2.5-Evo - 2026-08-17 - WHAT THE ESCALATION MEANS NOW. It withdraws the COMPASS from the ladder
  // for the rest of the session, so RTM keeps running on GPS course, and it makes Follow-Me decline
  // to engage (can_be_active in runFmLoop carries it, and so does the FM fault-stop classifier).
  // See the block with the declarations. Besides setting the flag it TELLS THE RIDER, once.
  // WHY PRINTING HERE IS STILL F7-SAFE, now that the flag can end an FM engagement. The F7 rule is
  // "motor to 0 first, explain afterwards", because a full USB CDC TX buffer can block inside
  // Serial for as long as the host takes to drain it. This print sits UPSTREAM of the FM stop:
  // checkFmFaultConditions() calls this function, and fm_throttle_cap = 0 is written further down
  // the same tick. So headingDisagreeAnnounceDegraded() refuses to print while the trigger is held
  // (thr_received >= 25) and prints at the next release instead — the notice keeps its one-shot
  // behaviour, and nothing can sit between a proven fault and the motor reaching 0. The rider can
  // also ask the board at any time with ?diag, which reports the latch without printing anything
  // in the control path.
  if (disagree_now) {
    // V2.5-Evo - 2026-08-17 - a measured disagreement ends any agreement dwell outright. The two
    // dwells are mutually exclusive by construction: one disagreeing sample is proof that agreement
    // has NOT held continuously, so the exoneration must start again from the next agreeing sample.
    heading_agree_since_ms     = 0;
    heading_agree_last_seen_ms = 0;

    if (heading_disagree_since_ms == 0) {
      heading_disagree_since_ms = now;
    } else if ((now - heading_disagree_last_seen_ms) > kHeadingDisagreeGapMaxMs) {
      // V2.5-Evo - 2026-08-16 - CONTINUITY CHECK. The last time we actually MEASURED a
      // disagreement is too long ago for this sample to be a continuation of it, so this sample
      // opens a NEW dwell instead of completing the old one. Without this, the frozen dwell clock
      // in the else-branch below let two isolated samples far apart be added together and reported
      // as one sustained condition (see kHeadingDisagreeGapMaxMs for the worked example).
      // Restarting a dwell can only ever DELAY a fault, never create one.
      heading_disagree_since_ms = now;
    } else if ((now - heading_disagree_since_ms) >= (unsigned long)kHeadingDisagreeMs) {
      heading_disagree_fault = true;
      headingDisagreePersist(true);       // LATCH-1: survive a power cycle
      headingDisagreeAnnounceDegraded();   // one line, once per fault, on the transition only
    }
    // Stamped AFTER the continuity test above, which reads the PREVIOUS measurement's time.
    heading_disagree_last_seen_ms = now;
  } else if (compare_possible) {
    // Measured, and the two agree. This is clear route 1, and V2.5-Evo - 2026-08-17 it is the only
    // one that can fire without the rider doing anything — which is why compare_possible above
    // deliberately tests the CONFIGURED mode rather than the withdrawn one. Withdrawing the compass
    // must not switch off the measurement that exonerates it, or the latch really would need a
    // reboot.
    //
    // V2.5-Evo - 2026-08-17 - AN AGREEING SAMPLE IS NO LONGER AN ACQUITTAL ON ITS OWN. Convicting
    // takes kHeadingDisagreeMs of continuous measured disagreement — at least four samples — and
    // this branch used to undo all of it on ONE tick. A mirrored module is the case that breaks on:
    // it reports theta - h_true, so the compass-to-course gap is 2*h_true - theta and dips below
    // kHeadingDisagreeDeg in two 45-deg-wide windows per revolution. A rider coasting on a heading
    // in one of those windows produced agreeing samples with the module still mirrored, and the
    // latch vanished. ?magalign cannot detect mirroring either, so this was the defect the
    // stickiness existed for. Now agreement has to hold for the SAME dwell, with the SAME
    // continuity bound, before the verdict is withdrawn — no new constant, no new tuning.
    //
    // DIRECTION OF THE CHANGE: strictly harder to clear, never easier. A dwell that restarts can
    // only leave a proven fault standing for longer.
    if (heading_agree_since_ms == 0) {
      heading_agree_since_ms = now;
    } else if ((now - heading_agree_last_seen_ms) > kHeadingDisagreeGapMaxMs) {
      // Continuity broken: the previous agreeing sample is too old to be part of this run of
      // agreement, so this sample opens a fresh dwell instead of completing the old one. Same test,
      // same constant and same reasoning as the disagreement side above.
      heading_agree_since_ms = now;
    }
    // Stamped AFTER the continuity test, which reads the PREVIOUS measurement's time.
    heading_agree_last_seen_ms = now;

    // An unfinished DISAGREEMENT proof is dropped immediately, exactly as before: a measured
    // agreement is positive evidence against a dwell that has not yet convicted anything, and
    // dropping it can only delay a fault. Only the FAULT itself now waits for the full dwell.
    heading_disagree_since_ms     = 0;
    heading_disagree_last_seen_ms = 0;

    if ((now - heading_agree_since_ms) >= (unsigned long)kHeadingDisagreeMs) {
      if (heading_disagree_fault) {
        // Transition only — the flag below re-arms the notice, so this cannot repeat while the
        // sources keep agreeing. Safe to print here under the F7 rule: this branch only ever
        // RESTORES capability (the compass comes back, FM becomes eligible again), so there is no
        // motor-stopping write downstream of it that a blocked UART could defer.
        Serial.printf("HEADING [RX] COMPASS RESTORED: compass and GPS course measured agreeing continuously for %lu ms — the disagreement latch is cleared and Follow-Me may engage again.\n",
                      (unsigned long)kHeadingDisagreeMs);
      }
      heading_disagree_fault    = false;
      headingDisagreePersist(false);       // LATCH-1: the stored verdict goes with it
      heading_degrade_announced = false;   // re-arm the notice if it ever latches again
    }
  } else {
    // No comparison possible this tick. Restart the dwell rather than carry a half-finished proof
    // across a gap (same discipline as fm_diverge_since_ms). A fault already PROVEN is kept — it is
    // cleared by a fresh agreement above, by a successful ?compasscal / ?magalign. A reboot does NOT clear it (the verdict is stored in SPIFFS since 2026-08-18).
    // V2.5-Evo - 2026-08-16 - FREEZE, do not clear. This branch runs when the comparison is

    // IMPOSSIBLE (COG invalid), not when the sources agreed. Zeroing here discarded partial

    // evidence at precisely the handover the tester's failure happened on: a disagreement that

    // had been building for 4 of the required 5 seconds was wiped the moment COG dropped out,

    // so the escalation could never fire on the way into the slow zone. Leaving the timestamp

    // in place is safe - the fault only escalates inside if(disagree_now), which still requires

    // a live comparison, so a stale clock cannot fault anything on its own.

    //
    // V2.5-Evo - 2026-08-16 - ...but a frozen proof does not keep FOREVER. WHAT WAS STILL WRONG:
    // with the timestamp held for an unbounded time, two one-tick disagreements minutes apart
    // satisfied "the gap has held continuously for kHeadingDisagreeMs" and escalated to a fault -
    // the dwell was measuring the wall clock between two unrelated moments, not a sustained
    // condition. WHAT THIS DOES: a part-finished proof that has sat unmeasured for longer than
    // kHeadingDisagreeEvidenceMaxMs is discarded, and the next disagreement opens a fresh dwell.
    // It cannot weaken the guard in the case the freeze was written for: a genuinely continuous
    // disagreement escalates at 5 s and never reaches this age, and evidence spanning one COG
    // handover is seconds old, not tens of seconds. A fault already PROVEN is untouched here -
    // only the unfinished proof ages out.
    //
    // V2.5-Evo - 2026-08-16 - ...and this cap alone was not enough, because it bounds the SPAN of a
    // dwell (age measured from its first sample) and not the age of the last MEASUREMENT, and it is
    // only tested here, on ticks where no comparison is possible. Two isolated samples 14 s apart
    // therefore still escalated together. The companion bound is the continuity test in the
    // if(disagree_now) branch above: consecutive disagreements more than kHeadingDisagreeGapMaxMs
    // apart start a fresh dwell rather than completing an old one. The two work together — this one
    // discards evidence that has gone cold, that one refuses to stitch unrelated moments into a
    // single "sustained" condition — and neither can set a fault, only drop unfinished proof.
    if (heading_disagree_since_ms != 0 &&
        (now - heading_disagree_since_ms) > kHeadingDisagreeEvidenceMaxMs) {
      heading_disagree_since_ms     = 0;
      heading_disagree_last_seen_ms = 0;
    }

    // V2.5-Evo - 2026-08-17 - THE AGREEMENT DWELL AGES OUT ON THE SAME TERMS, and for the same
    // reason: a part-finished exoneration that has sat unmeasured longer than
    // kHeadingDisagreeEvidenceMaxMs is measuring the wall clock, not a sustained condition. The
    // continuity test in the agree branch above already refuses to stitch two distant samples
    // together; this is its companion, so a stale agreement clock cannot survive a long dormant
    // stretch and then acquit on the first sample after it. Pair cleared together, as everywhere
    // else here. Discarding it can only make a clear LATER, never earlier.
    if (heading_agree_since_ms != 0 &&
        (now - heading_agree_since_ms) > kHeadingDisagreeEvidenceMaxMs) {
      heading_agree_since_ms     = 0;
      heading_agree_last_seen_ms = 0;
    }
  }

  if (disagree_now) {
    *out_heading = -1.0f;
    *out_confidence = 0;
    return false;
  }

  if (cog_valid) {
    cog_last_good_deg = gps_last_course_deg;

    cog_last_good_ms  = now;

    *out_heading = gps_last_course_deg;
    *out_confidence = 3;  // HIGH
    return true;
  }

  // ---- Mode 0: GPS COG only — no fallback ----
  // If COG is invalid (slow speed, stale, or guard-1 frozen), return no source.
  // updateRtmSteering() will hold straight.
  //
  // ============================================================
  // V2.5-Evo - 2026-08-17 - THE HEADING-DISAGREEMENT TERM THAT WAS ADDED TO THIS RETURN IS GONE
  // AGAIN, AND IS BACK BELOW THE COG HOLD WHERE IT STARTED.
  //
  // WHAT IT WAS DOING HERE. For one pass a standing heading_disagree_fault left the ladder at this
  // line, which withdrew the kCogHoldMs held-COG bridge as well as the compass. The stated reason
  // was consistency: the hold is a mode-1 feature, mode 0 never had it, so a session running as
  // "mode 0" should not have it either.
  //
  // WHY THAT WAS WRONG. The hold serves a LAST-GOOD GPS COURSE. The latch is evidence about the
  // COMPASS — and only about the compass; the cross-check cannot even say which of the two sources
  // is the liar. Withdrawing a GPS-derived value on compass evidence is outside this guard's
  // charter, and it was the one part of that pass that removed something the latch has no evidence
  // against. It cost real behaviour in both loops: a COG dropout longer than 1.5 s failed FM's
  // condition 6 and forced a full stop-and-re-arm the hold would have bridged, and on the RTM side
  // the documented cog_valid flicker in the 3-4 km/h approach band (rtm_target_speed_kmh 4.0
  // against rtm_cog_min_speed_kmh 3, a margin inside the speed signal's own noise) alternated the
  // steering override between centre and bearing at 10 Hz — the exact flapping the hold was written
  // to stop.
  //
  // WHAT IT MEANS NOW. Withdrawn = "mode 1 minus the compass", which is precisely what the evidence
  // supports: live COG at confidence 3, held COG at confidence 2 for up to kCogHoldMs, and no
  // compass. cog_last_good_deg / cog_last_good_ms are maintained in the live-COG branch above this
  // point, so they keep being refreshed while the latch stands — this is a move of one test, not
  // new plumbing.
  //
  // NOT A CONFIG WRITE, unchanged: usrConf.rtm_use_compass is READ here and never written; the
  // rider's stored mode is untouched, nothing is saved to SPIFFS, and ?conf still reports what they
  // set. The withdrawal lives in RAM and dies with the power cycle.
  // ============================================================
  if (mode == 0) {
    *out_heading = -1.0f;
    *out_confidence = 0;
    return false;
  }

  // ============================================================
  // GUARD 1, PART 2 — A FROZEN COG DOES NOT PROMOTE THE COMPASS
  // V2.5-Evo - 2026-07-25 - STAGE 2
  //
  // THE SUBTLE PART, AND THE REASON THE VEER HAPPENED. Guard 1 invalidates COG. Left alone, the
  // ladder below would then do what it has always done and fall through to the compass — which is
  // EXACTLY the handover that steered the buggy the wrong way. Think about what we actually know
  // in this state: the GPS is provably dead (a register that stopped moving is not an opinion),
  // and the compass is unverifiable — the one instrument that could have cross-checked it is the
  // source we just threw away, and on this hardware the compass is biased 100 deg+ by motor
  // current at exactly the moment we would be relying on it (the buggy is moving, so the trigger
  // is held, so the snapshot is not even refreshing).
  //
  // One provably dead source plus one unverifiable source is not a heading. HOLDING STRAIGHT IS
  // SAFER THAN STEERING ON THE SURVIVOR: a buggy that holds its line is predictable and the rider
  // can see it and let go of the trigger, whereas a buggy steering confidently on a bad compass
  // goes somewhere nobody chose. So we return NOTHING.
  //
  // WHAT EACH MODE DOES WITH "NOTHING" — no new code path is invented for either:
  //   RTM: gate 6 stops the run only when rtm_compass_required = 1 AND the heading-disagreement
  //        latch is NOT standing; otherwise updateRtmSteering() holds the override at 127
  //        (straight) and RTM continues under the throttle governor with runPhaseC()'s convergence
  //        check still watching.
  //        V2.5-Evo - 2026-08-17 - CORRECTION, because this used to say flatly that gate 6 stops
  //        the run at the default setting. It does not during a session in which the latch stands:
  //        that gate now stands aside whenever headingDisagreeLatched() is true (see the note at
  //        the gate), so a frozen COG on such a session behaves like the rtm_compass_required = 0
  //        case above — hold straight, keep running — rather than stopping. Stating it the old way
  //        described a stop that does not happen. The bypass is deliberately NOT narrowed here;
  //        narrowing it is a separate change with its own argument to make.
  //   FM : condition 6 in checkFmFaultConditions() fails, which is already classified as a FAULT
  //        — FM_STOPPING, cap 0, ramp back to manual, FM_IDLE, re-arm required. (And while the
  //        latch stands FM does not engage at all — can_be_active carries it — so this path is
  //        reached only with the compass exonerated or never accused.)
  //
  // THIS IS SELF-CLEARING AND STRICTLY NARROW. It only exists while the buggy is MOVING and the
  // GPS is repeating itself; the instant the course value moves again, or the buggy slows below
  // cog_min_speed, the ordinary ladder (including the compass fallback) returns unchanged. The
  // stationary/low-speed RTM arm sequence never reaches this line.
  // ============================================================
  if (cog_frozen_moving) {
    *out_heading = -1.0f;
    *out_confidence = 0;
    return false;
  }

  // ---- Mode 1 (Hybrid): try compass snapshot ----
  // Snapshot is captured by updateCompassSnapshot() in Compass.ino during motor-idle.
  // Age determines confidence:
  //   < 1000ms : MEDIUM (likely still fresh)
  //   1000-8000ms : LOW (degraded; reduce steering authority) — SW45: extended from 3000ms for stationary RTM arm
  //   > 8000ms : NONE (too stale)
  // ============================================================

  // V2.5-Evo - 2026-08-16 - FIELD BUG: RTM veered at close range.

  //

  // Reported by a beta tester: RTM tracked straight toward him, then turned hard at ~5-7 m.

  // The cause is a HANDOVER. RTM decelerates inside rtm_approach_zone_m, speed falls below

  // rtm_cog_min_speed_kmh, cog_valid goes false, and the heading source switches from GPS COG

  // to the compass snapshot. If the compass is mounted rotated - and before SW35 there was no

  // mag_orientation to correct it - the heading STEPS by the mounting angle at that instant and

  // the steering obeys it.

  //

  // Two guards should have caught that and neither did:

  //

  //   1. The per-tick cross-check needs compare_possible, which requires cog_valid. It can only

  //      run while COG is trustworthy - which is exactly NOT the moment the compass takes over.

  //      The check protects you while you do not need it and stands down when you do.

  //

  //   2. The sticky escalation heading_disagree_fault DID survive, but it was consumed only by

  //      runFmLoop(). Follow-Me was protected against a lying compass; Return-to-Me was not -

  //      runRtmLoop() never read it. An asymmetry between two loops sharing one sensor stack.

  //

  //      V2.5-Evo - 2026-08-16 CORRECTION to point 2: "DID survive" was wrong, and it was the

  //      false premise this whole block rested on. The latch survived nothing during RTM -

  //      fmEnterIdle() zeroed it on every 100 ms tick, and runFmLoop() calls fmEnterIdle() while

  //      rtm_rx_active is set. So the check added below was reading a flag that was being wiped

  //      ten times a second and could never be true in an RTM-only run.

  //

  //      V2.5-Evo - 2026-08-17 SECOND CORRECTION: the fix for that was to make both clears

  //      edge-triggered, and those two edge clears are now GONE ENTIRELY - the latch is cleared

  //      only by evidence (a sustained measured agreement, a successful ?compasscal / ?magalign)

  //      A reboot does NOT clear it (the verdict is stored in SPIFFS since 2026-08-18). It therefore survives across engagements, and the withdrawal described below

  //      is exactly what a standing fault does: it takes the COMPASS out of the ladder and nothing

  //      else. The check is a few lines below, between the COG hold and the compass fallback.

  //

  // Fixed at the SOURCE rather than in either loop: if the compass has been caught disagreeing

  // with COG by more than kHeadingDisagreeDeg for kHeadingDisagreeMs, THE COMPASS FALLBACK BELOW

  // is withdrawn - from RTM, from FM, and from any future consumer. Both callers already treat

  // 'no heading' correctly: updateRtmSteering() sets rtm_steer_override = 127 and holds straight,

  // which at close range is the safe outcome and is what the buggy should have done here.

  //

  // V2.5-Evo - 2026-08-17 - SAY WHAT THE CODE DOES (third pass, and it is back to the second one).

  // The paragraph that first sat here explained why a HELD COG is still served while the latch

  // stands: the check sits BELOW the hold, and a held COG is a GPS measurement, not the compass

  // under suspicion. The pass in between moved the check above the hold and declared that

  // reasoning moot. It was not moot - it was the correct reading of what the latch is evidence

  // about, and the move quietly withdrew a GPS-derived value on compass evidence. The check is

  // back below the hold, so the original paragraph stands again and is restated here:

  //

  // A STANDING FAULT WITHDRAWS THE COMPASS AND NOTHING ELSE. Above rtm_cog_min_speed_kmh a live

  // GPS course is served at confidence 3; for up to kCogHoldMs after that it is served held, at

  // confidence 2; after that there is no heading and every consumer holds straight. The cross-check

  // cannot say WHICH of the two sources is lying, so it may only remove the one it is about.

  //

  // THE DIRECTION IS STILL SUBTRACTION, which is the only direction this guard is allowed to move:

  // the compass leaves the ladder and nothing is added. What the previous pass cost, and what

  // moving the check back recovers, is real in both loops - a COG dropout longer than 1.5 s failed

  // FM's condition 6 and forced a stop-and-re-arm, and the cog_valid flicker in the 3-4 km/h

  // approach band alternated the RTM override between centre and bearing at 10 Hz, which is the

  // exact flapping the hold below was written to stop.

  // ============================================================

  // ============================================================
  // V2.5-Evo - 2026-08-16 - HOLD the last good COG across a short dropout.
  //
  // Before falling back to the compass, prefer a COG that was valid moments ago. RTM drives at
  // 4.0 km/h with the COG floor at 3 - a 0.278 m/s margin, inside the speed signal's own noise -
  // so cog_valid flickers on noise alone. Each flicker previously handed steering to a compass
  // that is 87-100 deg wrong under motor load, and the resulting hard turn SLOWED the craft,
  // which kept COG invalid, which kept the bad compass in charge. A closed loop with no exit:
  // that is the 'veers off and never recovers' a tester filmed.
  //
  // Four independent codebases - Betaflight, iNav, Ardumower, ArduPilot/PX4 - blend COG and
  // magnetometer continuously rather than switching between them. A full blend is a bigger
  // change than this craft needs today; holding the last good COG gets most of the benefit for
  // a fraction of the risk, and changes NEITHER threshold.
  //
  // Confidence is deliberately dropped to 2 (MEDIUM): a 2-second-old course is still a real
  // measurement, but it is stale and the caller should not treat it like a live one.
  // ============================================================
  if (cog_last_good_deg >= 0.0f && cog_last_good_ms > 0 &&
      (now - cog_last_good_ms) < (unsigned long)kCogHoldMs) {
    *out_heading = cog_last_good_deg;
    *out_confidence = 2;   // MEDIUM - real course, but stale
    return true;
  }

  // ============================================================
  // V2.5-Evo - 2026-08-17 - GUARD 2's CONSEQUENCE: THE COMPASS FALLBACK IS WITHDRAWN.
  //
  // A compass that has been caught disagreeing with the GPS course by more than kHeadingDisagreeDeg
  // for kHeadingDisagreeMs does not get to steer this craft again until it is exonerated. Every
  // consumer inherits that from here — RTM, FM and anything added later — so no loop has to
  // remember to check the flag for itself.
  //
  // WHY THE CHECK IS HERE AND NOT HIGHER UP THE LADDER. It sits BELOW the live-COG branch and BELOW
  // the kCogHoldMs hold, and above the compass fallback, because those are GPS courses and this is
  // evidence about the COMPASS. For one pass it lived up at the mode-0 return, which also withdrew
  // the hold; that was consistency reasoning ("the hold is a mode-1 feature") applied to a value
  // the latch has nothing against, and it cost a COG-dropout bridge that both loops rely on. The
  // cross-check proves the two sources cannot both be right, not which one is wrong, so it may only
  // subtract the source it names.
  //
  // WHAT THE CALLERS DO WITH THE "NO HEADING" THIS PRODUCES, unchanged: updateRtmSteering() sets
  // rtm_steer_override = 127 and holds straight, which at close range is the safe outcome and is
  // what the buggy should have done in the veer this guard was written for; FM's condition 6 treats
  // it as a fault. And FM will not have engaged at all while the latch stands — can_be_active
  // carries the flag — so for FM this line is a second line of defence, not the first.
  // ============================================================
  if (heading_disagree_fault) {
    *out_heading = -1.0f;
    *out_confidence = 0;
    return false;
  }

  if (compass_snapshot_heading >= 0.0f && compass_snapshot_ms > 0) {
    unsigned long age_ms = now - compass_snapshot_ms;
    if (age_ms < 1000UL) {
      *out_heading = compass_snapshot_heading;
      *out_confidence = 2;  // MEDIUM
      return true;
    } else if (age_ms < 8000UL) {
      *out_heading = compass_snapshot_heading;
      *out_confidence = 1;  // LOW — caller should reduce steering authority
      return true;
    }
  }

  // ---- No valid heading source ----
  *out_heading = -1.0f;
  *out_confidence = 0;
  return false;
}

// ============================================================
// FOLLOW-ME (FM) AUTONOMOUS FOLLOWING - state, tuning constants, geometry
// V2.5-Evo - 2026-07-19 - P3 (DESIGN_FOLLOW_ME.md sections 4-7)
//
// WHAT FM DOES, IN PLAIN ENGLISH
// After the whip, the rider lets go of the rope and surfs the wave. FM makes the
// buggy trail the rider at a set distance and angle, steering itself, so the rider
// can keep their eyes on the wave instead of on the buggy.
//
// THE SAFETY RULES THIS CODE OBEYS (identical to RTM, non-negotiable):
//   1. The buggy ONLY moves while the rider physically holds the throttle trigger
//      (thr_received >= 25). FM never creates motion on its own.
//   2. FM ONLY steers and SUBTRACTS throttle. It can never add throttle. The human
//      trigger stays the one and only throttle source.
//   3. Releasing the trigger stops the buggy immediately (unchanged base architecture).
//   4. Every failure path - GPS, compass, LoRa, bad geometry - drives the motor to 0
//      by writing fm_throttle_cap = 0.
//
// FM writes exactly two things that can reach the motor: fm_throttle_cap (a cap that
// can only reduce throttle) and rtm_steer_override (steering only, and only while the
// trigger is held). Nothing else in this module touches the motor path.
// ============================================================

// ---- FM tuning constants (not user-configurable; the 8 SPIFFS FM params cover tuning) ----

// Rider speed below which a course derived from GPS positions is too noisy to trust.
// Measured: course noise roughly doubles below ~3 mph. Below this we drop to the
// degraded "hold station" geometry (no diagonal) rather than chase a bad course.
static const float    kFmCourseValidSpeedKmh = 5.0f;   // km/h

// How much faster than the rider the buggy is allowed to run while closing the gap.
// Feeds throttle cap 3 (speed governor): target = min(boogie_vmax, rider_speed + this).
static const float    kFmClosingMarginKmh    = 5.0f;   // km/h

// Align-phase throttle cap (~5% of 255). While the heading error is large the buggy
// should pivot toward the target, not drive away from it. Same value RTM's align phase uses.
static const uint8_t  kFmAlignCap            = 13;     // 0-255

// Engage ramp length. On every entry into FM_ACTIVE the throttle cap ramps 0 -> full
// over this time so re-engagement is always a smooth build, never a throttle jump.
static const uint32_t kFmEngageRampMs        = 3500;   // ms

// Minimum time between rider course/speed samples. The rider's position arrives at 2 Hz,
// so we need a baseline of a few hundred ms for a stable course rather than differentiating
// two nearly identical filtered positions and getting noise.
static const float    kFmMotionBaselineS     = 0.4f;   // seconds

// ---- Engagement-semantics constants (V2.5-Evo - 2026-07-20) ----
// These four numbers implement the separation latch and its clears. They are deliberately
// compile-time only: no new confStruct fields, no SW_VERSION bump, no SPIFFS reset.

// How much further than the steady-state follow distance the rider must get before FM is
// allowed to engage for the first time. D_engage = kFmEngageFactor * d_follow.
// WHY THIS EXISTS: before this change the engage distance EQUALLED the follow distance
// (d_follow = min_dist_m + band). The tow rope MEASURES 20 ft = 6.10 m, which is LONGER than
// d_follow at the intended 4+2 = 6 m tuning — so FM could engage while the rider was still
// on the rope, i.e. autonomous steering mid-tow. 1.5x gives 9 m at 4+2 tuning: 9 m clears
// the measured 6.10 m rope by ~1.48x.
// V2.5-Evo - 2026-07-25 - F3-c prose fix: this comment used to quote a "6.7-7.6 m" tow rope.
// That number was an early estimate and it contradicted the MEASURED 6.10 m figure that
// BREmote_V2_Rx.h, ConfigService.ino and the web UI text all use. There is one rope length in
// this project and it is 6.10 m; the estimate is gone so the documentation stops arguing with itself.
// IMPORTANT: this factor is NOT the only thing protecting the engage distance. min_dist_m and
// followme_smoothing_band_m have no lower bound of their own, so a small tuning can make this
// product tiny — which is why kFmEngageDistFloorM is applied to the RESULT of this multiplication
// as well, not just to a manually typed fm_engage_dist_m. See the F3-c clamp in runFmLoop().
static const float    kFmEngageFactor        = 1.5f;   // multiplier on d_follow

// How long the rider must stay beyond D_engage before the separation latch sets.
// The rider position arrives at 2 Hz, so 2000 ms = 4 consecutive independent GPS fixes.
// WHY: the logs contain single-fix GPS spikes implying 41-144 mph. A spike moves the
// apparent distance for one fix; it cannot SUSTAIN it for four. The dwell converts a
// noise-triggerable threshold into one that needs real, persistent separation.
static const uint32_t kFmSepDwellMs          = 2000;   // ms

// How long the throttle may stay released before the separation latch is cleared.
// WHY: this is the session boundary. To rig the next tow the rider necessarily lets go of
// the trigger, so the latch dies with the run and the next tow starts unlatched — FM cannot
// engage again until separation has been re-proven. A rider linking waves keeps the trigger
// held and therefore keeps the latch. Same 25-count threshold FM condition 1 uses.
static const uint32_t kFmThrReleaseClearMs   = 10000;  // ms

// How long the RX keeps a TX-declared FM mode alive without a refresh.
// The TX re-sends 0xF2/mode every 30 s while armed, so 95 s is ~3 missed keepalives.
// WHY: without this the RX stored the declared mode FOREVER. If the TX's disarm burst
// (0xF2/0) was lost in the air, the RX stayed armed for the rest of the session with no
// way to find out. This is the backstop that expires a declaration nobody is refreshing.
static const uint32_t kFmModeAgeMs           = 95000;  // ms

// ---- A3 holds-vs-faults + steer-cancel constants (V2.5-Evo - 2026-07-20) ----
// Compile-time only, like the four above: no confStruct fields, no SW_VERSION bump, no SPIFFS reset.

// Condition 9 (rider speed) RESUME hysteresis. FM HOLDs when the rider drops below
// foiler_low_speed_kmh and only resumes once they are back above foiler_low_speed_kmh +
// kFmSpeedHystKmh. WHY: falling below speed is normal and recurring, so it is a HOLD not a fault;
// without the +2 km/h gap a rider hovering at the threshold would flap HOLD<->ACTIVE every fix.
// Mirrors the distance Schmitt band.
static const float    kFmSpeedHystKmh        = 2.0f;   // km/h

// FAULT stop ramp. On a fault (conditions 2-7) FM hands throttle back to the rider by ramping the
// cap 0 -> 255 over this window, then drops to FM_IDLE (re-arm required). WHY the ramp: the rider
// may still be holding the trigger, so returning full manual throttle instantly would lurch.
static const uint32_t kFmStopRampMs          = 2000;   // ms

// How long the surprise-gated fault-stop notification stays sticky in fm_flags bit 3. WHY:
// telemetry rotates every ~2.4 s, so a one-shot notification could land between rotations and
// never reach the TX. 6 s guarantees the TX sees it and can fire the St + stop buzz exactly once.
static const uint32_t kFmFaultStickyMs       = 6000;   // ms

// Engage grace: FM ignores steer-cancel for this long after engaging. WHY: technique 2 (whip by
// steering the buggy away) means the rider is often still feeding the tail of that steering input
// at the instant FM engages; without the grace that tail would immediately cancel the fresh FM.
static const uint32_t kFmEngageGraceMs       = 2000;   // ms

// Steering persistence filter for steer-cancel. A steering deflection must be sustained beyond
// the deadband for this long before it cancels an ACTIVE FM. WHY: a momentary blip (chop, a bump)
// must not drop FM; only a deliberate, held steer is a manual-control declaration.
static const uint32_t kFmSteerPersistMs      = 500;    // ms

// Steering deadband for steer-cancel, in raw steering counts either side of 127 (centre). A
// deflection smaller than this is treated as centred and never counts toward steer-cancel.
static const uint8_t  kFmSteerCancelDeadband = 40;     // counts from 127

// ---- FM divergence-fault constants (V2.5-Evo - 2026-07-25 - A3) ----
// Compile-time only, like every other kFm* above: no confStruct fields, no SW_VERSION bump, no
// SPIFFS reset.
//
// WHAT WAS MISSING. While FM is ACTIVE the distance condition (condition 8, in runFmLoop) is
// "dist_m >= min_dist" — a LOWER bound only. It answers "is the buggy far enough away to be safe?"
// and nothing else. If the steering is wrong — a mirrored steering_inverted, a compass 180 out, a
// bad course estimate — the buggy drives AWAY from the rider and that condition keeps passing more
// and more comfortably the further it gets. RTM does have a divergence net (runPhaseC's convergence
// check, "distance must be decreasing"), but runPhaseC() is only ever called from runRtmLoop() and
// never from runFmLoop(), so FM had no upper bound at all and would steer away indefinitely for as
// long as the rider held the trigger. These two numbers add that bound.
//
// WHY A CEILING AND NOT runPhaseC's "must be decreasing". RTM's rule is right for RTM: the buggy is
// commanded to close on a stationary rider, so any non-decreasing distance is wrong. FM is not
// closing — it deliberately holds station d_follow behind a MOVING rider, so distance legitimately
// rises and falls every wave and "must be decreasing" would fire constantly. What is never
// legitimate in FM is being far outside the follow geometry. So we keep runPhaseC's bookkeeping
// shape (a single sustained-condition timer, cleared the instant the condition stops holding) and
// change only the test itself.

// Multiple of D_engage beyond which the buggy is considered far enough away for the sustained
// not-closing test below to classify it as diverging. Scales automatically with the effective
// engage distance: the configured fm_engage_dist_m when non-zero, otherwise the auto-computed
// D_engage. V2.5-Evo - 2026-08-25: raised 2x -> 6x so normal catch-up, alignment and GPS scatter
// cannot start the divergence dwell close to the intended follow geometry. Examples: an explicit
// 11 m engage distance now gives a 66 m ceiling; auto D_engage=30 m gives a 180 m ceiling. The
// existing 3 s dwell and requirement to fail to close by more than 2 m are unchanged.
static const float    kFmDivergeFactor       = 6.0f;   // multiplier on effective D_engage

// How long the distance must stay beyond that limit before it counts as divergence. Rider position
// arrives at 2 Hz, so 3000 ms is ~6 consecutive independent fixes — the same spike-proofing argument
// as kFmSepDwellMs. A single bad fix cannot trip it; a genuinely diverging buggy trips it in 3 s.
static const uint32_t kFmDivergeMs           = 3000;   // ms

// V2.5-Evo - 2026-07-25 - F1: how much closer the buggy must have got over the dwell window to be
// judged "following, just far" rather than "running away".
// WHY THIS EXISTS AT ALL. The first cut of this detector was a BARE THRESHOLD: beyond the ceiling for
// kFmDivergeMs = a fault, full stop. That is wrong for two reasons that together aborted ordinary
// engagements. (1) The engage ramp is kFmEngageRampMs = 3500 ms, LONGER than the 3000 ms dwell, so the
// fault could fire before the buggy had even been given full throttle. (2) During align the cap is
// kFmAlignCap = 13/255 (~5%), so the buggy pivots on the spot and the distance GROWS before it starts
// to shrink — while at the engagement instant dist_m is typically 13-21 m against an 18 m ceiling
// (2 x 9 m). The result was FM aborting ~3 s into most real engagements and forcing a mid-session
// re-arm. THE FIX: judge the DERIVATIVE, not the level — exactly what runPhaseC()'s convergence check
// does ("dist_m >= rtm_prev_dist_m" -> not closing -> fail). We snapshot the distance when the dwell
// starts and, at dwell expiry, only fault if the buggy has NOT closed by more than this epsilon.
// WHY 2.0 m. The buggy's closing speed is capped by cap 3, the speed governor, at rider speed +
// kFmClosingMarginKmh = 5 km/h = 1.39 m/s, so over the 3 s dwell a genuinely closing buggy recovers
// up to ~4.2 m — comfortably more than 2 m. 2 m is meanwhile larger than ordinary GPS scatter at these
// distances, so noise alone cannot fake "closing" and cancel a real divergence.
static const float    kFmDivergeCloseEpsM    = 2.0f;   // metres of closure over the dwell

// V2.5-Evo - 2026-07-25 - F3-b: the hard floor for the MANUAL fm_engage_dist_m override,
// kFmEngageDistFloorM, is NOT defined here any more. It used to sit in this block as 5.0f while
// ConfigService.ino carried a SECOND bare 5.0f literal, on the false premise that the Arduino
// concatenation order stopped the two files sharing a constant. It now has exactly one definition,
// in BREmote_V2_Rx.h — raised there to 8.0 m, because 5.0 m was below the tow rope it exists to
// clear (the owner's rope is 20 ft = 6.10 m). Both the config validator and the read-site clamp in
// runFmLoop() below reference that one constant. See BREmote_V2_Rx.h for the full rationale.

// ---- FM state machine (DESIGN_FOLLOW_ME.md section 4) ----
//   FM_IDLE    : FM off (mode 0), RTM owns the buggy, or GPS/FM disabled.
//                No throttle cap (255) and no steering override - fully manual buggy.
//   FM_ARMED   : a mode (1-3) is selected and all monitoring runs, but FM has not engaged
//                yet. The throttle chain is INACTIVE (cap 255) so the rider still has full
//                manual control of the buggy while FM waits for the follow geometry.
//   FM_ACTIVE  : every activation condition holds. Steering override on, throttle cap chain on.
//   FM_HOLD    : FM was ACTIVE and a HOLD condition dropped out - condition 8 (distance / stop
//                radius) or 9 (rider below foiler_low_speed_kmh), or the trigger was released
//                (DEADMAN). These are geometry / throttle pauses, NOT faults: the motor stops
//                (cap 0), the declaration stays ARMED, no alarm sounds, and FM auto-resumes to
//                FM_ACTIVE through the engage ramp once the conditions restore AND the separation
//                latch is set. Falling below speed is a normal, recurring part of riding, so it
//                must never force a re-arm. Kept distinct from FM_ARMED because the two carry
//                different caps (0 vs 255): before FM ever engaged the rider keeps manual throttle,
//                but once FM has held control a paused hold must stop the buggy. (Was FM_DEMOTED.)
//   FM_STOPPING: FM was engaged and a FAULT dropped out - conditions 2-7 (Phase A/B, TX/RX GPS
//                stale, heading invalid, LoRa). Something actually broke, so autonomy ends for
//                this run: the throttle cap ramps 0 -> 255 over kFmStopRampMs (throttle always
//                returns, never a lurch under a held trigger), then FM drops to FM_IDLE and a
//                fresh TX declaration is required to re-arm. A surprise-gated St + stop buzz fires
//                (fm_flags bit 3) only if the trigger was held at the fault instant.
enum FmState : uint8_t { FM_IDLE = 0, FM_ARMED = 1, FM_ACTIVE = 2, FM_HOLD = 3, FM_STOPPING = 4 };
static FmState fm_state = FM_IDLE;

// ---- FM rider tracking state ----
// fm_filt_* is FM's EMA-filtered rider position. It uses the SAME first-order filter
// formula and the SAME preset time constant as RTM's tx_pos_filtered_* (see
// updateRtmSteering) - the filter must keep ignoring the rider's carves so the buggy
// follows the low-passed path instead of mirroring bottom turns (measured p95 turn rate
// 49 deg/s at ~5 m radius). FM keeps its own copy because tracking has to stay warm while
// FM is only ARMED, whereas RTM's filter only runs while RTM is actively steering.
static double        fm_filt_lat         = 0.0;
static double        fm_filt_lng         = 0.0;
static bool          fm_filt_init        = false;
static unsigned long fm_filt_prev_ms     = 0;     // last EMA update (for the filter dt)

// Previous filtered position, used as the baseline for deriving rider course and speed.
static double        fm_prev_filt_lat    = 0.0;
static double        fm_prev_filt_lng    = 0.0;
static unsigned long fm_prev_filt_ms     = 0;

// Rider motion derived from successive FILTERED positions.
// fm_rider_course_deg is -1.0f when the rider is too slow for a trustworthy course.
static float         fm_rider_course_deg = -1.0f;  // 0-360 deg clockwise from North, or -1 = invalid
static float         fm_rider_speed_kmh  = 0.0f;   // km/h

// Side-zone Schmitt state: true = apply the diagonal offset, false = sit directly behind.
static bool          fm_diagonal_engaged = false;

// millis() at the moment FM entered FM_ACTIVE. Drives the engage ramp. 0 = not engaged.
static unsigned long fm_engage_ms        = 0;

// ---- Separation latch state (V2.5-Evo - 2026-07-20) ----
// fm_sep_latched: true once the rider has been proven genuinely separated from the buggy
//   (beyond D_engage for kFmSepDwellMs) during this throttle-hold session. FM may only make
//   its FIRST entry into FM_ACTIVE while this is true. Once latched, the existing distance
//   Schmitt hysteresis governs engage/re-engage as before, so the buggy is free to close back
//   to its normal 6 m station without fighting the interlock. The latch is NEVER cleared by
//   geometry alone — only by the throttle-release clear, a mode change, or entering FM_IDLE.
static bool          fm_sep_latched      = false;

// millis() when the rider first went beyond D_engage; 0 = not currently beyond it.
// Counts the dwell that defeats single-fix GPS spikes.
static unsigned long fm_sep_over_since_ms = 0;

// millis() when thr_received first dropped below 25; 0 = throttle currently held.
// Counts the kFmThrReleaseClearMs window that clears the latch at the end of a run.
static unsigned long fm_thr_low_since_ms  = 0;

// ---- A3 fault-stop + steer-cancel state (V2.5-Evo - 2026-07-20) ----
// millis() when FM entered FM_STOPPING; drives the 0 -> 255 fault ramp. 0 = not stopping.
static unsigned long fm_stop_ms          = 0;

// millis() of the last SURPRISING fault stop (a fault that occurred while the trigger was held).
// Drives the sticky fm_flags bit 3 for kFmFaultStickyMs so the TX cannot miss the stop
// notification across the ~2.4 s telemetry rotation. 0 = no recent surprising fault. Deliberately
// NOT cleared by fmEnterIdle() — the notification must survive the transition into FM_IDLE.
static unsigned long fm_fault_alarm_ms   = 0;

// millis() when the rider's steering first exceeded kFmSteerCancelDeadband while FM was ACTIVE;
// 0 = steering currently centred. Counts the kFmSteerPersistMs persistence filter for steer-cancel.
static unsigned long fm_steer_input_since_ms = 0;

// V2.5-Evo - 2026-07-25 - A3: millis() when dist_m first exceeded kFmDivergeFactor x D_engage while
// FM was ACTIVE; 0 = not currently beyond it. Counts the kFmDivergeMs dwell for the divergence fault.
// Reset discipline is copied from runPhaseC's rtm_prev_dist_m and from fm_sep_over_since_ms: cleared
// the moment the condition stops holding, whenever the distance is untrustworthy (trigger released,
// GPS stale/rejected, link down), whenever FM is not ACTIVE, and on entry to FM_IDLE. A half-finished
// proof is never carried across a data gap or a state change.
static unsigned long fm_diverge_since_ms = 0;

// V2.5-Evo - 2026-07-25 - F1: the buggy-to-rider distance in metres captured at the instant
// fm_diverge_since_ms started, i.e. the baseline the closure test compares against at dwell expiry.
// -1.0f = no dwell running / no baseline. This is the FM twin of runPhaseC's rtm_prev_dist_m: it turns
// the detector from "are you far?" (a level) into "are you failing to close?" (a derivative), which is
// what actually distinguishes a buggy running away from one that is following from further back than
// we would like. Cleared in lockstep with fm_diverge_since_ms everywhere, so a baseline can never
// outlive its own dwell or be compared against a distance from a different engagement.
static float         fm_diverge_start_dist_m = -1.0f;

// The computed trailing target point FM steers toward. Written by computeFmTarget() and
// read by updateRtmSteering() when fm_rx_active is set.
static double        fm_target_lat       = 0.0;
static double        fm_target_lng       = 0.0;

// ---- Compute RTM steering override (Bundle 1: P+D + bearing filter) ----
// V2.5-Evo - 2026-05-08 - Bundle 1: Replaced fixed ±90° clamp with preset-driven P+D controller.
// Added first-order low-pass filter on TX target position for FM path-following smoothness.
// Heading source still comes from getRtmHeading() (GPS COG primary, snapshot fallback).
// LOW-confidence sources reduce steering authority by 50% (unchanged from D5).
// Filter state + D-term reset on invalid heading to satisfy the heading-filter rule.
static void updateRtmSteering()
{
  if (!usrConf.rtm_rx_override_steering) {
    rtm_steer_override = 127;
    // Reset D-term continuity statics here too: with override disabled we produce no
    // steering samples, so on a later off->on toggle the D-term must not differentiate
    // a fresh heading error against a stale pre-toggle sample across the gap (Kd spike).
    prev_heading_src_valid  = false;
    prev_heading_error_deg  = 0.0f;
    prev_steering_update_ms = 0;
    g_heading_error_dx10 = 0x7FFF;
    g_d_error_dx10 = 0x7FFF;
    return;
  }

  float current_heading;
  uint8_t confidence;
  bool valid = getRtmHeading(&current_heading, &confidence);

  if (!valid) {
    // No valid heading — hold straight. Reset filter + D-term state so we don't
    // resume with stale data on next cycle. (project rule)
    rtm_steer_override = 127;
    prev_heading_error_deg = 0.0f;
    prev_heading_src_valid = false;   // no same-source prior sample to differentiate against
    tx_pos_filter_initialized = false;
    g_heading_error_dx10 = 0x7FFF;
    g_d_error_dx10 = 0x7FFF;
    return;
  }

  // Lookup active preset — clamp index defensively
  uint16_t idx = usrConf.rtm_steer_response;
  if (idx > 4) idx = 2;  // fallback to Normal on bad config
  const SteerPreset &p = kSteerPresets[idx];

  // ---- Bearing-target low-pass filter (for FM path-following) ----
  // First-order exponential moving average on TX position.
  // alpha = dt / (tau + dt). dt is loop period, tau is preset's filter time constant.
  unsigned long now = millis();
  float dt_s = (prev_steering_update_ms == 0) ? 0.1f : ((now - prev_steering_update_ms) / 1000.0f);
  if (dt_s <= 0.0f || dt_s > 1.0f) dt_s = 0.1f;  // sanity clamp
  prev_steering_update_ms = now;

  // ---- Steering target selection: RTM aims at the rider, FM aims behind the rider ----
  // V2.5-Evo - 2026-07-19 - P3 FM. Two callers now share this controller:
  //   RTM (rtm_rx_active == true): steer straight at the rider's EMA-filtered position.
  //   FM  (fm_rx_active == true and RTM inactive): steer at the trailing target point that
  //                                runFmLoop() already computed via computeFmTarget(). FM does its
  //                                own EMA filtering in updateFmRiderTracking() (it has to keep
  //                                tracking while only ARMED), so we skip RTM's filter here rather
  //                                than filtering the same rider position twice and adding a second lag.
  // Everything downstream of this block - heading error, +/-180 wrap, preset clamp, P+D, and
  // confidence-scaled authority - is shared by both modes and is untouched.
  double steer_target_lat, steer_target_lng;
  // V2.5-Evo - 2026-08-25 - Give RTM explicit priority during the one loop-window in which an RTM
  // arm packet has set rtm_rx_active but runFmLoop() has not yet cleared fm_rx_active. This makes
  // the RTM arm-edge D reset below line up with the actual target switch instead of differentiating
  // the direct-to-rider target one tick later against a final FM trailing target.
  if (fm_rx_active && !rtm_rx_active) {
    steer_target_lat = fm_target_lat;
    steer_target_lng = fm_target_lng;
  } else {
    if (!tx_pos_filter_initialized) {
      tx_pos_filtered_lat = rx_tx_gps_lat;
      tx_pos_filtered_lng = rx_tx_gps_lng;
      tx_pos_filter_initialized = true;
    } else if (p.target_filter_tau_s > 0.0f) {
      float alpha = dt_s / (p.target_filter_tau_s + dt_s);
      tx_pos_filtered_lat += alpha * (rx_tx_gps_lat - tx_pos_filtered_lat);
      tx_pos_filtered_lng += alpha * (rx_tx_gps_lng - tx_pos_filtered_lng);
    } else {
      tx_pos_filtered_lat = rx_tx_gps_lat;
      tx_pos_filtered_lng = rx_tx_gps_lng;
    }
    steer_target_lat = tx_pos_filtered_lat;
    steer_target_lng = tx_pos_filtered_lng;
  }

  // Bearing from RX GPS to the selected steering target
  double bearing_deg = TinyGPSPlus::courseTo(
      gps_last_lat, gps_last_lng, steer_target_lat, steer_target_lng);

  // Heading error (signed, wrapped to ±180°)
  float heading_error = (float)(bearing_deg - current_heading);
  while (heading_error >  180.0f) heading_error -= 360.0f;
  while (heading_error < -180.0f) heading_error += 360.0f;

  // Saturate (preset clamp angle)
  float clamped = heading_error;
  if (clamped >  p.error_clamp_deg) clamped =  p.error_clamp_deg;
  if (clamped < -p.error_clamp_deg) clamped = -p.error_clamp_deg;

  // P term (normalized to ±127 at full clamp)
  float p_term = (clamped / p.error_clamp_deg) * 127.0f * p.kp;

  // ---- D term: differentiate ONLY across consecutive same-source samples ----
  // Fable audit §5: when the heading source switches (GPS COG <-> compass) or the
  // compass snapshot re-snaps, heading_error steps by tens of degrees in a single
  // 100ms tick. Differentiating across that step injects a false ±300°/s rate into
  // Kd and commands a violent phantom turn. Build a source id that changes on a
  // source switch AND on a snapshot re-snap; skip the D term (d_error=0) whenever
  // the id differs from the previous sample so we never differentiate through a step.
  uint32_t heading_src_id;
  if (confidence == 3) {
    heading_src_id = 1;    // GPS COG — updates smoothly with motion, safe to differentiate
  } else if (usrConf.rtm_use_compass == 2) {
    heading_src_id = 2;    // live compass-only (diagnostic) — continuous reading
  } else {
    // Hybrid compass snapshot: the value is held constant until re-snapped. Fold the
    // snapshot timestamp into the id (high bit set so it can never collide with 1/2)
    // so each re-snap is treated as a new source and the D step across it is skipped.
    heading_src_id = 0x80000000UL | ((uint32_t)compass_snapshot_ms & 0x7FFFFFFFUL);
  }

  float d_error;
  if (prev_heading_src_valid && heading_src_id == prev_heading_src_id) {
    // heading_error lives on a circle. Subtracting its normalized representations directly
    // creates a false +/-360 deg jump at the branch cut (for example +179 -> -179). Differentiate
    // the shortest signed angular delta instead, so that example is +2 deg rather than -358 deg.
    float delta_error = heading_error - prev_heading_error_deg;
    while (delta_error >  180.0f) delta_error -= 360.0f;
    while (delta_error < -180.0f) delta_error += 360.0f;
    d_error = delta_error / dt_s;
  } else {
    d_error = 0.0f;  // source switched or snapshot re-snapped — do not differentiate across the step
  }
  float d_term = p.kd * d_error;
  prev_heading_error_deg = heading_error;
  prev_heading_src_id    = heading_src_id;
  prev_heading_src_valid = true;

  // Confidence: LOW conf reduces total authority by 50% (preserves D5 behavior)
  float authority = (confidence == 1) ? 0.5f : 1.0f;

  float output = 127.0f + authority * (p_term - d_term);
  if (output < 0.0f)   output = 0.0f;
  if (output > 254.0f) output = 254.0f;
  rtm_steer_override = (uint8_t)output;

  // Export for logger (with sentinel-safe conversion)
  g_heading_error_dx10 = (int16_t)(heading_error * 10.0f);
  g_d_error_dx10       = (int16_t)(d_error * 10.0f);
  if (g_heading_error_dx10 == 0x7FFF) g_heading_error_dx10 = 0x7FFE;  // avoid sentinel collision
  if (g_d_error_dx10       == 0x7FFF) g_d_error_dx10       = 0x7FFE;

  #ifdef DEBUG_RX
  Serial.printf("RTM steer[%u]: bear=%.1f head=%.1f err=%.1f d_err=%.1f P=%.1f D=%.1f auth=%.2f ovr=%d\n",
                idx, (float)bearing_deg, current_heading, heading_error, d_error,
                p_term, d_term, authority, (int)rtm_steer_override);
  #endif
}

// ---- Phase C anti-spoofing (runs during active RTM, every 5s) ----
static void runPhaseC()
{
  if (!rtm_rx_active || rtm_rx_emergency_stop) return;

  unsigned long now = millis();
  if (now - rtm_phase_c_ms < 5000UL) return;
  rtm_phase_c_ms = now;

  float dist_m = (float)TinyGPSPlus::distanceBetween(
      gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);

  // Phase C check 1: convergence — distance to TX must be decreasing
  if (rtm_prev_dist_m >= 0.0f && dist_m >= rtm_prev_dist_m)
  {
    Serial.printf("RTM [PhC] FAIL convergence: dist %.0f m (was %.0f m) — not closing\n",
                  dist_m, rtm_prev_dist_m);
    rtm_rx_emergency_stop = true;
    rtm_rx_active = false;
    return;
  }
  rtm_prev_dist_m = dist_m;

  // Phase C check 2: VESC ERPM vs GPS speed (only if vesc_erpm_per_kmh is configured)
  // V2.5-Evo - 2026-05-11 - Freshness guard: vesc.last_packet is read inside the same mutex
  // that protects vesc.erpm. If the VESC data is older than vesc_timeout_s (e.g. VESC dropped
  // during heavy regen braking), skip the check rather than comparing stale ERPM to live GPS
  // speed — a false FAIL here would abort RTM mid-run. Phase C check 1 (convergence) is the
  // primary safety gate and remains active regardless.
  if (usrConf.vesc_erpm_per_kmh > 0.0f)
  {
    extern vesc_struct vesc;
    extern SemaphoreHandle_t vescMutex;
    if (xSemaphoreTake(vescMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
      unsigned long vesc_age_ms = millis() - vesc.last_packet;
      float vesc_speed_kmh = (float)abs(vesc.erpm) / usrConf.vesc_erpm_per_kmh;
      xSemaphoreGive(vescMutex);

      if (vesc_age_ms > (unsigned long)usrConf.vesc_timeout_s * 1000UL)
      {
        // Stale VESC data — skip rather than falsely fail. Log for diagnostics.
        Serial.printf("RTM [PhC] SKIP VESC check: data age %lu ms > timeout %d s\n",
                      vesc_age_ms, (int)usrConf.vesc_timeout_s);
      }
      else
      {
        float speed_diff = fabsf(vesc_speed_kmh - gps_last_speed_kmh);
        if (speed_diff > usrConf.rtm_vesc_speed_diff_kmh)
        {
          Serial.printf("RTM [PhC] FAIL VESC speed: VESC=%.1f km/h GPS=%.1f km/h diff=%.1f\n",
                        vesc_speed_kmh, gps_last_speed_kmh, speed_diff);
          rtm_rx_emergency_stop = true;
          rtm_rx_active = false;
          return;
        }
      }
    }
  }

  // Phase C check 3: TX GPS freshness
  // Finding 6-1: was hardcoded 2000ms — now reads from SPIFFS.
  // NOTE: structurally redundant — Gate 4 already enforces this before
  // runPhaseC() is called. Retained as belt-and-suspenders only.
  if (rx_tx_gps_timestamp == 0 ||
      (millis() - rx_tx_gps_timestamp) > (uint32_t)usrConf.tx_gps_stale_timeout_ms)
  {
    Serial.println("RTM [PhC] FAIL TX GPS freshness");
    rtm_rx_emergency_stop = true;
    rtm_rx_active = false;
    return;
  }

  Serial.printf("RTM [PhC] PASS: dist=%.0f m, converging\n", dist_m);
}

// ---- Main RTM loop — call from RX loop() ----
void runRtmLoop()
{
  // V2.5-Evo - 2026-05-06 - D5: Always update the compass snapshot, regardless
  // of RTM state or rate-limit gate. Snapshot only updates when motor is idle
  // (thr_received < 25, checked inside updateCompassSnapshot()), so this is cheap
  // and safe to call every iteration. The snapshot is consumed by getRtmHeading()
  // as the low-speed fallback heading source in Hybrid mode.
  updateCompassSnapshot();

  // Rate-limit to 10Hz (compass I2C + TinyGPS math takes ~2ms per call)
  static unsigned long last_rtm_ms = 0;
  unsigned long now = millis();
  if (now - last_rtm_ms < 100UL) return;
  last_rtm_ms = now;

  // ============================================================
  // V2.5-Evo - 2026-08-17 - RTM ENGAGEMENT BOUNDARY: DROP AN UNFINISHED PROOF, KEEP THE VERDICT
  //
  // WHAT THIS BLOCK DOES TODAY, IN ONE SENTENCE: on either edge of an RTM engagement it clears the
  // part-built disagreement dwell — heading_disagree_since_ms and heading_disagree_last_seen_ms —
  // so a half-finished proof can never span two different situations, and it does NOT touch
  // heading_disagree_fault at either edge. The verdict is cleared by evidence only: a sustained
  // measurement showing agreement, a successful ?compasscal / ?magalign. A reboot does NOT clear it (the verdict is stored in SPIFFS since 2026-08-18).
  //
  // EVERYTHING FROM HERE TO "WHAT EACH EDGE DOES NOW" IS HISTORY, kept because it is the reasoning
  // that constrains the answer. It describes a disarm-edge clear of the FAULT that no longer
  // exists: it was there to stop a sticky latch becoming an un-clearable RTM ARMING BLOCK, and that
  // block is gone — gate 6 stands aside while the latch stands, so RTM runs on GPS course instead
  // of refusing. With nothing left to unblock ON THE RTM SIDE, forgiving a proven fault at an RTM
  // boundary bought nothing and cost the rider a fresh proof on every single run.
  // V2.5-Evo - 2026-08-17 - AND IT MUST NOT COME BACK NOW THAT FOLLOW-ME REFUSES AGAIN. FM declining
  // to engage is the intended response to symmetric evidence (see can_be_active in runFmLoop), so an
  // RTM disarm — which says nothing whatever about a compass — forgiving that verdict would be the
  // same unearned amnesty in a different place. RTM keeps running throughout, so the rider is never
  // stranded by it, and the evidence-based clears remain reachable while coasting.
  //
  // WHY THIS EXISTED. The sticky heading_disagree_fault used to be cleared only in fmEnterIdle(),
  // which is a Follow-Me function. That was fine while only FM consumed the fault, but
  // getRtmHeading() now consumes it too (it refuses to hand out the compass fallback once the
  // compass has been caught disagreeing with COG), and in RTM-only operation FM never leaves
  // FM_IDLE, so that transition never happens and nothing would ever forgive the fault. Without
  // this block a rider who provoked one genuine disagreement would find the compass fallback
  // silently disabled for the rest of the session, with a power cycle as the only way out.
  //
  // WHAT IT DOES. Exactly what fmEnterIdle() does for FM, on RTM's own terms: the moment an RTM
  // engagement ENDS (rtm_rx_active goes true -> false, whether by TX disarm, Gate 9 handoff,
  // Phase C failure or rtm_rx_enabled being turned off), the latch and its part-finished dwell are
  // dropped so the next arm is judged on fresh sensor evidence.
  //
  // WHY IT IS AN EDGE AND NOT A LEVEL. Clearing it on every tick where RTM is inactive would
  // recreate the exact bug this pairs with — it would zero the latch ten times a second all the
  // way through every FOLLOW-ME engagement, since FM only ever runs while RTM is inactive.
  // Single-writer: this runs in the loop task, as does every other writer of these two.
  //
  // V2.5-Evo - 2026-08-16 - IT IS NOW BOTH EDGES, NOT JUST DISARM. TWO REASONS, AND THE SECOND ONE
  // IS WHY THE DISARM EDGE ALONE WAS NOT ENOUGH.
  //
  //   (a) THE DWELL RUNS WHEN NOTHING IS ARMED. getRtmHeading() is called unconditionally a few
  //       lines below, every 100 ms, purely to fill the heading-confidence bits of the fm_status
  //       telemetry byte — regardless of RTM or FM state. So the comparison, and therefore the
  //       5 s dwell, also accumulates during ordinary manual riding. A rotated or uncalibrated
  //       compass plus a coast above rtm_cog_min_speed_kmh with the trigger released (the snapshot
  //       keeps refreshing while the motor is idle, so the comparison stays live) latches the fault
  //       before the rider has armed anything at all. With only end-of-engagement clears, nothing
  //       forgave that, and it then blocked the NEXT engagement from the inside — FM stuck in
  //       FM_ARMED with the TX showing "ready", because fm_flags bit 2 drops once separation
  //       latches and the explanatory printf in runFmLoop() only fires if FM was already engaged.
  //
  //   (b) THE DISARM EDGE DOES NOT ALWAYS HAPPEN. Gates 2-8 in checkRtmSafetyGates() set
  //       rtm_rx_emergency_stop but deliberately leave rtm_rx_active TRUE (only gate 9 and
  //       runPhaseC() clear it). So if the TX dies mid-tow — flat battery, gate 7 link loss — RTM
  //       stays "active" with the throttle pinned at 0 and this true -> false edge never arrives.
  //       The other clear route cannot help either: the emergency stop keeps the buggy stationary,
  //       so COG never goes valid, so no fresh comparison is ever possible. The fault would sit
  //       there until a power cycle, and the failure it then produced on the next arm was gate 6
  //       printing "No valid heading source", which points at the wrong cause entirely.
  //
  // WHY CLEARING AT ARM CANNOT WEAKEN ANYTHING. This flag only ever SUBTRACTS eligibility — it
  // grants no heading, raises no cap and extends no engagement. And a disagreement that is real is
  // still there when the run starts: it re-proves itself within kHeadingDisagreeMs (5 s) of the
  // first tick, because the same comparison runs on every tick of the run. So the price is at most
  // 5 s of protection at the very beginning of an engagement, and what it buys is the removal of
  // two dead ends that were both silent and both survived until a reboot.
  //
  // NOTE ON REACHING THE ARM EDGE AFTER (b): the TX begins every arm ceremony with a 0xF1/0 burst
  // ("armed but not yet active", TX RTMState.ino setRtmArmed()) before the 0xF1/1 that follows a
  // successful double squeeze. So even a stuck-true rtm_rx_active is driven false and then true
  // again by a re-arm, and BOTH edges of this test fire in the right order.
  //
  // V2.5-Evo - 2026-08-17 - THE TWO EDGES NO LONGER DO THE SAME THING. The paragraphs above argue
  // for clearing at BOTH boundaries, and for the dwell that remains correct — but clearing the
  // FAULT at the ARM edge made the guard incapable of ever acting, because the arm edge is where
  // the only usable evidence has just been gathered.
  //
  // WHY THE ARM EDGE IS THE EVIDENCE, NOT NOISE. A comparison requires a live COG and a compass
  // snapshot younger than kHeadingCompareSnapMs (1000 ms), and Compass.ino only refreshes that
  // snapshot while thr_received < 25 — a snapshot taken under motor current is worthless, which is
  // the entire premise of using a snapshot at all. RTM and FM both require a HELD trigger. So
  // roughly one second into any engagement the comparison goes dormant and the kHeadingDisagreeMs
  // (5 s) dwell can never complete inside a run. The two moments where both inputs genuinely
  // coexist are a rider COASTING with the trigger released above rtm_cog_min_speed_kmh, and the arm
  // ceremony itself, during which the TX pins the throttle byte to zero and the snapshot is
  // therefore refreshing. Wiping a proven fault at that boundary discarded the freshest evidence
  // available anywhere in the whole sequence.
  //
  // WHAT EACH EDGE DOES NOW (V2.5-Evo - 2026-08-17 — BOTH EDGES DO THE SAME THING):
  //   DISARM (true -> false) and ARM (false -> true) both drop the UNFINISHED dwell and nothing
  //   else. A half-built proof must not span two different situations, so it restarts; a PROVEN
  //   fault is untouched at both edges and keeps the compass out of the ladder until it is cleared
  //   by evidence.
  //
  // WHY THE DISARM CLEAR OF THE FAULT WENT. It was forgiveness without evidence. A compass mounted
  // 90 deg out is exactly as wrong after a disarm as before it, so clearing there meant the rider
  // had to re-prove the fault by coasting with the trigger released on every single run — and the
  // proof takes kHeadingDisagreeMs of continuous measurement in a window that only exists while
  // coasting. In practice that made the guard nearly unreachable, which is the gap this closes.
  //
  // WHAT A SURVIVING FAULT ACTUALLY COSTS THE RTM RIDER, TRACED. If COG is live at arm time,
  // nothing: getRtmHeading() returns COG at confidence 3 well above the compass check, so the run
  // proceeds normally. If the buggy is stationary or below rtm_cog_min_speed_kmh — the ordinary RTM
  // arm — there is no heading unless the kCogHoldMs bridge still has one, and RTM ARMS ANYWAY:
  // gate 6 stands aside exactly as it does for a rider who set rtm_compass_required = 0, the
  // steering override holds at 127 (straight) and the align phase caps the throttle at ~5% until a
  // heading exists. The buggy moves off on the rider's own trigger, passes rtm_cog_min_speed_kmh,
  // and the GPS course takes over from there. What it never does is steer a towing buggy at a
  // person on a compass that has been measured 45+ deg wrong.
  // V2.5-Evo - 2026-08-17 - FOLLOW-ME PAYS A DIFFERENT PRICE, DELIBERATELY: it does not engage at
  // all while the latch stands, because autonomous following cannot be run on "one of these two is
  // lying and I cannot tell which". See can_be_active in runFmLoop().
  // ============================================================
  {
    static bool rtm_prev_active = false;
    bool rtm_now_active = rtm_rx_active;
    if (rtm_prev_active != rtm_now_active) {
      // Either edge of an engagement — drop the unfinished proof only. heading_disagree_fault is
      // deliberately NOT touched here: a boundary is not evidence about a compass.
      heading_disagree_since_ms     = 0;
      heading_disagree_last_seen_ms = 0;

      // V2.5-Evo - 2026-08-25 - RTM may preempt an ACTIVE FM run. The FM D history is valid for
      // FM's trailing target but not for RTM's direct-to-rider target, so cold-start the shared
      // derivative on the RTM arm edge. Without this companion to the FM D-state fix below, the
      // first RTM sample could differentiate across two different control targets.
      if (rtm_now_active) {
        prev_heading_error_deg    = 0.0f;
        prev_heading_src_valid    = false;
        prev_steering_update_ms   = 0;
      }
    }
    rtm_prev_active = rtm_now_active;
  }

  // ---- Extended telemetry: rx_heading, fm_heading_err, fm_status ----
  // rx_heading: GPS COG÷2 (0-179 maps to 0-358°); 0xFF = no valid COG
  if (gps_last_course_deg >= 0.0f && gps_last_course_ms > 0 &&
      (now - gps_last_course_ms) < 3000UL) {
    telemetry.rx_heading = (uint8_t)((uint16_t)(gps_last_course_deg) / 2);
  } else {
    telemetry.rx_heading = 0xFF;
  }

  // fm_heading_err: bearing error + 127 bias; 127 = no data
  if (g_heading_error_dx10 == 0x7FFF) {
    telemetry.fm_heading_err = 127;
  } else {
    int16_t e = g_heading_error_dx10 / 10;
    if (e < -126) e = -126;
    if (e >  126) e =  126;
    telemetry.fm_heading_err = (uint8_t)(e + 127);
  }

  // fm_status: [7]=aux2_on [6]=aux1_on [5]=vesc_online [4]=rx_wetness [3:2]=heading_conf [1]=rtm_active [0]=fm_active
  {
    uint8_t st = 0;
    // V2.5-Evo - 2026-07-19 - P3 FM: bit 0 now reports FM actually ENGAGED and steering
    // (fm_rx_active), not merely "a mode is selected". Previously any selected mode 1-3 set
    // this bit, so the TX could show FM as live while FM was only armed and waiting for the
    // follow geometry. fm_rx_active is set by runFmLoop() only in FM_ACTIVE.
    if (fm_rx_active)  st |= (1 << 0);
    if (rtm_rx_active) st |= (1 << 1);
    float h_unused; uint8_t conf;
    getRtmHeading(&h_unused, &conf);
    st |= (conf & 0x03) << 2;
    if (telemetry.error_code == 71) st |= (1 << 4);
    bool vesc_ok = (millis() - last_uart_packet) <
                   ((uint32_t)usrConf.vesc_timeout_s * 1000UL);
    if (vesc_ok)                 st |= (1 << 5);
    if (rx_aux_flags & (1 << 0)) st |= (1 << 6);
    if (rx_aux_flags & (1 << 1)) st |= (1 << 7);
    telemetry.fm_status = st;
  }

  // fm_flags (index 16): coherent Follow-Me engagement sub-state for the TX display (A2/A3/arming).
  // V2.5-Evo - 2026-07-20 - repurposed the former reserved_tx_imu byte. Kept SEPARATE from
  // fm_status (whose 8 bits are already full with aux/vesc/wetness/heading_conf/rtm_active) so no
  // working telemetry is disturbed and a TX still on the old firmware simply ignores this byte.
  // Bit map (the TX renders these in a later pass):
  //   [0] armed           - a live TX declaration is held (FM_ARMED / FM_ACTIVE / FM_HOLD). Scanner.
  //   [1] engaged         - FM is actively following (FM_ACTIVE). Grow-with-far distance bar.
  //   [2] armed-not-ready - armed but not yet engage-eligible on RX facts: no separation latch yet,
  //                         or (V2.5-Evo - 2026-08-17) a standing heading-disagreement latch.
  //                         The TX ORs its own TX-local readiness (own GPS fix/age, pairing, last
  //                         reply age) on top, then renders blink-in-place (not ready) vs sweep (ready).
  //   [3] fault-stop      - a FAULT ended FM while the trigger was held; sticky kFmFaultStickyMs so
  //                         the TX cannot miss it across the ~2.4 s rotation and fires St + stop buzz.
  // The four A3 disarm-ownership facts (armed drops, engaged drops, fault-sticky rises) let the TX
  // detect an RX-side fault and clear its own fm_armed so display and engagement cannot disagree.
  //
  // V2.5-Evo - 2026-08-17 - BIT 2 RISES ON A HEADING DISAGREEMENT AGAIN, BECAUSE FM IS BLOCKED
  // AGAIN. The term was ORed in when the fault blocked FM, taken out when the fault stopped
  // blocking it, and it belongs back now that can_be_active carries !heading_disagree_fault once
  // more. THE RULE THIS BIT FOLLOWS, and the only one: it reports RX-side facts that make an armed
  // FM ineligible to engage. A standing latch is precisely such a fact and it outranks the
  // separation test — separation can latch perfectly well while the fault stands, and if bit 2
  // reported only separation the TX would drop from blink-in-place to sweep and tell the rider
  // "ready" for a Follow-Me that cannot engage at all. That is the display-versus-behaviour
  // contradiction this bit exists to prevent, and it is the one that actually costs the rider
  // something: they wait for an engagement that is never coming, with no explanation on the water.
  // The mirror-image objection (reporting "not ready" for an FM that is about to engage) does not
  // apply here — while the latch stands FM is not about to engage under any conditions.
  // SCOPE UNCHANGED: still only in the armed-but-not-following states, so FM_ACTIVE never sets it
  // and bits 0/1/3 are untouched.
  {
    uint8_t f = 0;
    FmState s = fm_state;
    if (s == FM_ARMED || s == FM_ACTIVE || s == FM_HOLD)    f |= (1 << 0);
    if (s == FM_ACTIVE)                                     f |= (1 << 1);
    if ((s == FM_ARMED || s == FM_HOLD) &&
        (!fm_sep_latched || heading_disagree_fault))        f |= (1 << 2);
    if (fm_fault_alarm_ms != 0 && (now - fm_fault_alarm_ms) < kFmFaultStickyMs) f |= (1 << 3);
    telemetry.fm_flags = f;
  }

  // Finding 6-2: auto-expire Phase B approval when TX GPS goes stale.
  // gpsPhaseBCheck() sets gps_phase_b_ok=true on pass and never clears it —
  // it only runs on meta-packet receipt every ~30s. If TX GPS drops,
  // rx_tx_gps_timestamp stops updating and gps_phase_b_ok stays true
  // indefinitely. Gate 4 catches this during active RTM, but an RTM arm
  // attempt immediately after TX GPS loss could still pass Gate 3.
  // Revoke Phase B if TX GPS is older than 2× the configured stale threshold.
  {
    unsigned long phase_b_stale = (uint32_t)usrConf.tx_gps_stale_timeout_ms * 2UL;
    if (rx_tx_gps_timestamp == 0 ||
        (now - rx_tx_gps_timestamp) > phase_b_stale)
    {
      gps_phase_b_ok = false;
    }
  }

  // ---- Distance computation: telemetry encoding + approach decel cap ----
  // Distance is always encoded when both GPS sources are valid — feeds the TX R5 proximity
  // bar during RTM and FM modes, and enables the TX pre-arm check to correctly block
  // re-arm while within rtm_disengage_distance_m (correct safety behaviour after Gate 9).
  // Approach decel cap is only computed during active RTM; reset to 255 otherwise.
  {
    bool gps_rx_ok = (gps_last_ms > 0) && ((millis() - gps_last_ms) < 6000UL);
    // C1/M2 audit fix: both active and inactive paths now require a fresh
    // rx_tx_gps_timestamp instead of the 0.0 lat/lng sentinel.
    // The 0.0 sentinel accepted any stale coordinate — field logs confirmed
    // GPS timestamps froze for 50+ seconds in urban environments, causing
    // RTM distance to read near-zero while actually 20m+ away.
    // Active RTM: 5s max age (tight — buggy is moving, staleness is dangerous).
    // Inactive/FM: 10s max age (tolerates brief meta-packet gaps without
    //              suppressing the FM bar; still rejects genuinely stale GPS).
    bool gps_tx_ok = (rx_tx_gps_timestamp > 0) &&
                     ((millis() - rx_tx_gps_timestamp) < (rtm_rx_active ? 5000UL : 10000UL));

    if (gps_rx_ok && gps_tx_ok)
    {
      float d = (float)TinyGPSPlus::distanceBetween(
          gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);

      // Always encode real distance when both GPS sources are valid.
      // 0-99: tenths of metre (0.0-9.9 m); 100-254: whole metres offset by 90 (10-164 m)
      if (d < 10.0f)
      {
        telemetry.rtm_distance = (uint8_t)(d * 10.0f);
      }
      else
      {
        uint8_t whole_m = (uint8_t)(d > 164.0f ? 164.0f : d);
        telemetry.rtm_distance = 90u + whole_m;
      }

      // rx_bearing_to_tx: compass bearing from buggy toward rider position÷2 (0-179); 0xFF = N/A
      {
        double btx = TinyGPSPlus::courseTo(
            gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);
        telemetry.rx_bearing_to_tx = (uint8_t)((uint16_t)(btx) / 2);
      }

      if (rtm_rx_active)
      {
        // Approach decel zone: linearly ramp the throttle cap as the buggy closes in.
        // At rtm_approach_zone_m (outer edge): cap = 255 (full user throttle).
        // At rtm_stop_distance_m (Gate 9 edge):  cap = 0  (buggy coasts to stop naturally).
        // Between those two distances: linear interpolation.
        // Gate 9 still fires as the absolute safety floor.
        // rtm_approach_zone_m == 0 disables the feature (hard stop only).
        if (usrConf.rtm_approach_zone_m > 0)
        {
          uint16_t stop_m     = (usrConf.rtm_stop_distance_m > 0) ? usrConf.rtm_stop_distance_m : 10u;
          float    approach_m = (float)usrConf.rtm_approach_zone_m;
          if (approach_m > (float)stop_m && d < approach_m)
          {
            float cap_frac = (d - (float)stop_m) / (approach_m - (float)stop_m);
            if (cap_frac < 0.0f) cap_frac = 0.0f;
            if (cap_frac > 1.0f) cap_frac = 1.0f;
            rtm_approach_cap = (uint8_t)(cap_frac * 255.0f);
          }
          else
          {
            rtm_approach_cap = 255;  // outside zone: no cap
          }
        }
        else
        {
          rtm_approach_cap = 255;  // feature disabled: no cap
        }
      }
      else
      {
        rtm_approach_cap = 255;  // RTM inactive: no approach cap
      }
    }
    else if (!rtm_rx_active)
    {
      // FM/idle: never actively write 0xFF here. The struct field initialises to 0xFF;
      // Fix B above updates it to real distance once gps_rx_ok && gps_tx_ok is satisfied.
      // Actively resetting to 0xFF on any GPS hiccup caused the FM bar to stay dark.
      // rtm_approach_cap must be 255 when RTM is inactive — no throttle capping outside RTM.
      rtm_approach_cap = 255;
    }
    // GPS conditions failed (RTM active or inactive): keep last known distance and cap.
  }

  // ============================================================
  // V2.5-Evo - 2026-08-17 - STALE STEERING OVERRIDE: NEUTRAL WHENEVER NOTHING IS ENGAGED
  //
  // WHAT THE BUG WAS. rtm_steer_override kept its last bearing-derived value indefinitely after an
  // engagement that ended without passing through the throttle-release path. Only two places ever
  // wrote the neutral 127: gate 1 in checkRtmSafetyGates() (trigger released) and
  // updateRtmSteering(). A GATE 9 HANDOFF WITH THE TRIGGER STILL HELD goes through neither —
  // rtm_rx_active is set false there, the inactive path a few lines below returns without touching
  // the override, and fmEnterIdle() deliberately leaves it alone as well (its comment says so: if
  // RTM has just armed, RTM owns it). So a value such as 210 sat there for the rest of the session.
  // The FM mode-age expiry in runFmLoop() has the same shape: it calls fmEnterIdle() straight out
  // of FM_ACTIVE, so the last FM steering value is left parked too.
  //
  // WHY THAT WAS DANGEROUS. calcPWM() applies the override whenever
  // (rtm_rx_active || fm_rx_active) && rtm_rx_override_steering && thr_received >= 25. Re-arming RTM
  // sets rtm_rx_active from the triggeredReceive task, and the rider squeezes the trigger moments
  // later — so the 100 Hz generatePWM task can apply the stale bearing as differential steering
  // BEFORE runRtmLoop() next recomputes it, with no gate 9 and no Phase C involved. Ordinarily that
  // window is one 100 ms tick. It is UNBOUNDED if the loop task happens to be inside a long
  // deliberately non-abortable serial command (?gpssetup, ?wifiupd) when the re-arm lands, because
  // then runRtmLoop() is not running at all and nothing recomputes the override.
  //
  // WHAT THIS DOES. Drives the override back to straight on every tick where NO steering owner
  // exists, so a stale bearing can never be left parked for the next engagement to pick up.
  //
  // WHY THE !fm_rx_active TERM IS NOT OPTIONAL. runRtmLoop() runs BEFORE runFmLoop() in loop()
  // (V2_Integration_Rx.ino), and the 100 Hz generatePWM task preempts freely between the two on
  // this single-core part. An unconditional write here would therefore publish a neutral 127 for one
  // window per cycle all the way through a FOLLOW-ME engagement — RTM is inactive for the whole of
  // one — and FM steering would stutter to centre ten times a second. That is precisely the hazard
  // fm_throttle_cap was given its own separate global to avoid; see the note on that atomic in
  // BREmote_V2_Rx.h. Gating on BOTH flags means this line can only run on a tick where nobody owns
  // the steering, and on such a tick the value is already 127 anyway — every FM exit path except
  // fmEnterIdle() writes 127 itself — so it can never change a value FM or RTM is using.
  //
  // WHY HERE AND NOT LOWER DOWN. This sits above the rtm_rx_enabled early return so it also covers
  // a board with RTM disabled in config, and above the inactive-path return so it covers the
  // ordinary disarmed case. On the ACTIVE path the condition is false, so a live RTM run reaches
  // updateRtmSteering() with its override untouched.
  //
  // SAFETY DIRECTION: writes the neutral value and nothing else. It cannot command a turn, cannot
  // add throttle, and does not touch rtm_rx_active, the emergency stop or any cap.
  // ============================================================
  if (!rtm_rx_active && !fm_rx_active)
  {
    rtm_steer_override = 127;
  }

  if (!usrConf.rtm_rx_enabled)
  {
    rtm_rx_active         = false;
    rtm_rx_emergency_stop = false;
    return;
  }

  if (!rtm_rx_active)
  {
    rtm_rx_emergency_stop    = false;
    rtm_prev_dist_m          = -1.0;
    rtm_phase_c_ms           = 0;
    rtm_approach_cap         = 255;   // belt-and-suspenders: ensure cap is always clear when inactive
    // RTM-only position-filter state is never consumed by FM and may be reset whenever RTM is idle.
    tx_pos_filter_initialized = false;

    // V2.5-Evo - 2026-08-25 - The P+D continuity and exported heading diagnostics are SHARED by
    // RTM and FM. runRtmLoop() executes before runFmLoop(); resetting them merely because RTM is
    // idle erased FM's previous sample on every tick and made its D term permanently zero. Preserve
    // them while FM owns steering. FM already cold-starts these values on every engagement, and an
    // ownerless/HOLD/STOP tick still clears them here so a later run cannot inherit stale history.
    if (!fm_rx_active)
    {
      prev_heading_error_deg    = 0.0f;
      prev_heading_src_valid    = false;
      prev_steering_update_ms   = 0;
      g_heading_error_dx10      = 0x7FFF;
      g_d_error_dx10            = 0x7FFF;
    }
    // telemetry.rtm_distance already set to 0xFF by the block above (inactive path)
    return;
  }

  // RTM active: run all gates
  if (!checkRtmSafetyGates())
  {
    // Gate 1: throttle released — no emergency stop, motor already at 0.
    // Gate 9: stop distance reached — clean disengagement, rtm_rx_active set false, no emergency stop.
    // Gates 2-8: safety failure — rtm_rx_emergency_stop=true, calcPWM() forces throttle to 0.
    return;
  }

  // All gates pass: clear emergency stop, update steering
  rtm_rx_emergency_stop = false;
  updateRtmSteering();

  // Two-phase RTM throttle control (SW32):
  // Phase 1 (Align): heading error > rtm_align_threshold_deg → ~5% throttle cap.
  //   Buggy pivots toward target without driving away. At near-zero throttle, motor
  //   current is minimal so compass bias is reduced — hybrid heading mode gets cleaner
  //   snapshot data during alignment, benefiting builds with BN-880 compass installed.
  // Phase 2 (Run): heading OK → GPS speed governor keeps speed at rtm_target_speed_kmh
  //   regardless of buggy power curve. Behaviour is consistent across different boogies.
  //   rtm_target_speed_kmh == 0 disables the governor (approach decel zone only).
  {
    float abs_err = (g_heading_error_dx10 != 0x7FFF) ?
        fabsf((float)g_heading_error_dx10 / 10.0f) : 180.0f;

    if (abs_err > (float)usrConf.rtm_align_threshold_deg) {
      // Phase 1 — Align: ~5% throttle — differential steers; buggy barely moves forward
      const uint8_t kAlignCap = 13;
      if (rtm_approach_cap > kAlignCap) rtm_approach_cap = kAlignCap;
    } else if (usrConf.rtm_target_speed_kmh > 0.0f) {
      // Phase 2 — Run: proportional GPS speed governor (full cap at target, zero cap at rest)
      float speed_frac = gps_last_speed_kmh / usrConf.rtm_target_speed_kmh;
      if (speed_frac > 1.0f) speed_frac = 1.0f;
      uint8_t speed_cap = (uint8_t)((1.0f - speed_frac) * 255.0f);
      if (rtm_approach_cap > speed_cap) rtm_approach_cap = speed_cap;
    }
  }

  // Phase C (every 5s)
  runPhaseC();
}

// ============================================================
// FOLLOW-ME GEOMETRY HELPERS
// V2.5-Evo - 2026-07-19 - P3 (DESIGN_FOLLOW_ME.md section 6)
// ============================================================

// ------------------------------------------------------------
// projectPoint - move a lat/lng a given distance along a given compass bearing
// ------------------------------------------------------------
// What it does:
//   Standard spherical "destination point given start, bearing and distance" formula.
//   Used to place the lag anchor ahead of the rider and the trailing target behind them.
//
// Inputs:
//   lat, lng     - start position in degrees (WGS84)
//   bearing_deg  - direction to travel, degrees CLOCKWISE FROM NORTH (0=N, 90=E, 180=S, 270=W).
//                  This is the same bearing convention TinyGPSPlus::courseTo(), the compass,
//                  and GPS course-over-ground all use on this board.
//   dist_m       - distance to travel in metres
//
// Outputs:
//   *out_lat, *out_lng - the resulting position in degrees
//
// Side effects: none (pure function).
//
// Precision note: all maths is double, matching the project rule that FM position maths must
// never drop to float - float carries only ~7 significant digits, which is visible error at the
// sub-10 m distances FM steers by.
// ------------------------------------------------------------
static void projectPoint(double lat, double lng, float bearing_deg, float dist_m,
                         double* out_lat, double* out_lng)
{
  const double kEarthRadiusM = 6371000.0;

  double br     = (double)bearing_deg * M_PI / 180.0;   // bearing in radians
  double ang    = (double)dist_m / kEarthRadiusM;       // angular distance in radians
  double lat_r  = lat * M_PI / 180.0;
  double lng_r  = lng * M_PI / 180.0;

  double sin_lat = sin(lat_r);
  double cos_lat = cos(lat_r);
  double sin_ang = sin(ang);
  double cos_ang = cos(ang);

  double new_lat_r = asin(sin_lat * cos_ang + cos_lat * sin_ang * cos(br));
  double new_lng_r = lng_r + atan2(sin(br) * sin_ang * cos_lat,
                                   cos_ang - sin_lat * sin(new_lat_r));

  *out_lat = new_lat_r * 180.0 / M_PI;
  *out_lng = new_lng_r * 180.0 / M_PI;
}

// ------------------------------------------------------------
// fmAngleDiff - smallest absolute angle between two compass bearings
// ------------------------------------------------------------
// Inputs:  a, b - bearings in degrees (any range)
// Returns: the absolute difference wrapped into 0-180 degrees
// Side effects: none.
// ------------------------------------------------------------
static float fmAngleDiff(float a, float b)
{
  float d = a - b;
  while (d >  180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return fabsf(d);
}

// ------------------------------------------------------------
// updateFmRiderTracking - EMA-filter the rider position and derive course + speed
// ------------------------------------------------------------
// What it does (DESIGN_FOLLOW_ME.md section 6 steps 1-2):
//   1. Low-pass filters the raw rider position that arrives from the TX at 2 Hz in the 0xF3
//      meta-packet. Same first-order EMA and same preset time constant RTM uses, so the buggy
//      follows the rider's smoothed path and ignores individual carves.
//   2. Derives the rider's course and speed from two successive FILTERED positions.
//      An EMA does not change steady-state velocity (it only adds lag), so differentiating the
//      filtered track gives a clean speed while rejecting per-sample GPS jitter.
//      Below kFmCourseValidSpeedKmh the course is marked invalid (-1) and FM falls back to the
//      degraded hold-station geometry instead of chasing a meaningless heading.
//
// Called every FM tick (10 Hz) whether FM is ARMED, DEMOTED or ACTIVE, so that rider motion is
// already warm the instant the activation conditions are met.
//
// Inputs:  reads rx_tx_gps_lat/lng/timestamp, usrConf.rtm_steer_response
// Outputs: writes fm_filt_lat/lng, fm_rider_course_deg, fm_rider_speed_kmh and the
//          fm_prev_filt_* baseline.
// Side effects: none outside those module globals. Never touches the motor path.
// ------------------------------------------------------------
static void updateFmRiderTracking()
{
  // No rider position has ever arrived - nothing to track. Leave the last known motion alone.
  if (rx_tx_gps_timestamp == 0) return;

  unsigned long now = millis();

  // Use the active steering preset's filter time constant - the same tau RTM filters with.
  uint16_t idx = usrConf.rtm_steer_response;
  if (idx > 4) idx = 2;                       // defensive fallback to Normal on bad config
  float tau = kSteerPresets[idx].target_filter_tau_s;

  // Filter timestep, sanity-clamped exactly the way updateRtmSteering() clamps it.
  float dt_s = (fm_filt_prev_ms == 0) ? 0.1f : ((now - fm_filt_prev_ms) / 1000.0f);
  if (dt_s <= 0.0f || dt_s > 1.0f) dt_s = 0.1f;
  fm_filt_prev_ms = now;

  // ---- Step 1: EMA filter of the raw rider position ----
  if (!fm_filt_init) {
    fm_filt_lat  = rx_tx_gps_lat;
    fm_filt_lng  = rx_tx_gps_lng;
    fm_filt_init = true;
  } else if (tau > 0.0f) {
    float alpha = dt_s / (tau + dt_s);
    fm_filt_lat += alpha * (rx_tx_gps_lat - fm_filt_lat);
    fm_filt_lng += alpha * (rx_tx_gps_lng - fm_filt_lng);
  } else {
    fm_filt_lat = rx_tx_gps_lat;
    fm_filt_lng = rx_tx_gps_lng;
  }

  // ---- Step 2: derive rider course + speed from successive filtered positions ----
  if (fm_prev_filt_ms == 0) {
    // First sample - just seed the baseline, no motion available yet.
    fm_prev_filt_lat = fm_filt_lat;
    fm_prev_filt_lng = fm_filt_lng;
    fm_prev_filt_ms  = now;
    return;
  }

  float dt2 = (now - fm_prev_filt_ms) / 1000.0f;
  if (dt2 < kFmMotionBaselineS) return;   // baseline too short for a stable course - wait

  float d_m = (float)TinyGPSPlus::distanceBetween(
      fm_prev_filt_lat, fm_prev_filt_lng, fm_filt_lat, fm_filt_lng);

  fm_rider_speed_kmh = (d_m / dt2) * 3.6f;

  if (fm_rider_speed_kmh >= kFmCourseValidSpeedKmh) {
    fm_rider_course_deg = (float)TinyGPSPlus::courseTo(
        fm_prev_filt_lat, fm_prev_filt_lng, fm_filt_lat, fm_filt_lng);
  } else {
    // Too slow for a trustworthy course - degrade to hold-station geometry (no diagonal).
    fm_rider_course_deg = -1.0f;
  }

  fm_prev_filt_lat = fm_filt_lat;
  fm_prev_filt_lng = fm_filt_lng;
  fm_prev_filt_ms  = now;
}

// ------------------------------------------------------------
// computeFmTarget - compute the point behind the rider that the buggy should steer to
// ------------------------------------------------------------
// What it does (DESIGN_FOLLOW_ME.md section 6 steps 3-4 and 6):
//
//   d_follow = min_dist_m + followme_smoothing_band_m   (owner default 4 + 2 = 6 m)
//
//   1. LAG ANCHOR. A first-order filter always trails a moving target by roughly v * tau
//      (13-18 m at 15-20 mph with tau = 2 s - larger than the follow gap itself, and it grows
//      with speed). If we simply sat d_follow behind the filtered position the buggy would fall
//      further behind the faster the rider went. So we first push an anchor point FORWARD along
//      the rider's course by min(v_rider * tau, 2 * d_follow), which cancels the filter lag and
//      makes the geometry speed-independent. The cap at 2 * d_follow stops a bad speed estimate
//      from throwing the anchor far up the track. This is deliberately NOT solved by shrinking
//      tau: the filter has to keep ignoring the rider's carves.
//
//   2. TRAILING POINT. target = anchor + d_follow at bearing (course + 180 + offset).
//
//   3. SIDE-ZONE SCHMITT. The diagonal offset is only applied while the buggy is reasonably
//      lined up behind the rider. We measure how far off the directly-behind axis the buggy
//      currently sits and run a Schmitt trigger on it (engage below zone_angle_enter_deg,
//      release above zone_angle_exit_deg, owner defaults 35 / 45). Outside the zone we fall
//      back to pure-behind. Without this hysteresis an unstable rider course could whip the
//      target point across the wake from one side to the other.
//
//   DEGRADED MODE. If the rider is too slow for a valid course, there is no meaningful "behind".
//   We hold station instead: put the target d_follow from the rider along the current
//   rider->buggy bearing, with no diagonal. The buggy holds its distance without manoeuvring
//   around what may be a rider in the water.
//
// ============================================================
// !!! OFFSET SIGN CONVENTION - VERIFY BEFORE WATER !!!
//
// This whole board works in ONE bearing convention: degrees CLOCKWISE FROM NORTH
// (0 = North, 90 = East, 180 = South, 270 = West). TinyGPSPlus::courseTo(),
// getCompassHeading() and GPS course-over-ground all return that convention, and
// projectPoint() above consumes it. Adding degrees to a bearing therefore rotates
// CLOCKWISE.
//
// Given that, with the rider travelling along course C:
//   directly behind the rider  = bearing C + 180
//   the rider's right-hand side = bearing C + 90   (clockwise from their heading)
//   the rider's left-hand side  = bearing C - 90
//
// So a "behind and to the rider's RIGHT" spot lies between C+180 and C+90, i.e. at
// C + 180 - near_diag_offset_deg. A "behind and to the rider's LEFT" spot lies between
// C+180 and C+270, i.e. at C + 180 + near_diag_offset_deg.
//
// Written as target_bearing = C + 180 + offset, that gives:
//   mode 1 Near-Right -> offset = -near_diag_offset_deg     (NEGATIVE)
//   mode 2 Behind     -> offset = 0
//   mode 3 Near-Left  -> offset = +near_diag_offset_deg     (POSITIVE)
//
// which matches DESIGN_FOLLOW_ME.md section 6 step 4 exactly. The single line that sets
// this sign is the "offset = -/+ usrConf.near_diag_offset_deg" pair below.
//
// ASSUMPTION BEING MADE: "Near-Right" means to the RIDER'S right as the rider faces along
// their direction of travel (not the buggy's right, and not the right of someone watching
// from the beach). Default mode 1 exists because the owner rides left-foot-forward, which
// puts the right side on their open/visible side - consistent with rider-relative.
//
// This must be confirmed against Tools/FollowMe Settings Visualizer.html before any water
// test. If the visualizer shows the mirror image, flip ONLY the two signs below - no other
// part of the geometry depends on this choice.
// ============================================================
//
// Inputs:  reads fm_filt_lat/lng, fm_rider_course_deg, fm_rider_speed_kmh, gps_last_lat/lng,
//          usrConf FM params, fm_mode_runtime
// Outputs: *out_lat / *out_lng - the target point to steer at
// Side effects: updates the fm_diagonal_engaged Schmitt latch.
// ------------------------------------------------------------
static void computeFmTarget(double* out_lat, double* out_lng)
{
  // Follow gap = hard-stop radius plus the smoothing band (owner default 4 + 2 = 6 m).
  float d_follow = usrConf.min_dist_m + usrConf.followme_smoothing_band_m;
  if (d_follow < 0.5f) d_follow = 0.5f;   // guard against a degenerate config

  // ---- Degraded mode: no trustworthy rider course - hold station at distance ----
  if (fm_rider_course_deg < 0.0f) {
    float b_rider_to_buggy = (float)TinyGPSPlus::courseTo(
        fm_filt_lat, fm_filt_lng, gps_last_lat, gps_last_lng);
    projectPoint(fm_filt_lat, fm_filt_lng, b_rider_to_buggy, d_follow, out_lat, out_lng);
    return;
  }

  float course = fm_rider_course_deg;

  // ---- Lag anchor: push forward along the rider's course to cancel the filter lag ----
  uint16_t idx = usrConf.rtm_steer_response;
  if (idx > 4) idx = 2;
  float tau     = kSteerPresets[idx].target_filter_tau_s;
  float v_ms    = fm_rider_speed_kmh / 3.6f;
  float lag_m   = v_ms * tau;
  float max_lag = 2.0f * d_follow;
  if (lag_m > max_lag) lag_m = max_lag;
  if (lag_m < 0.0f)    lag_m = 0.0f;

  double anchor_lat, anchor_lng;
  projectPoint(fm_filt_lat, fm_filt_lng, course, lag_m, &anchor_lat, &anchor_lng);

  // ---- Side-zone Schmitt: is the buggy lined up enough behind the rider to use the diagonal? ----
  float b_rider_to_buggy = (float)TinyGPSPlus::courseTo(
      fm_filt_lat, fm_filt_lng, gps_last_lat, gps_last_lng);
  float off_axis = fmAngleDiff(b_rider_to_buggy, course + 180.0f);

  if (!fm_diagonal_engaged && off_axis < usrConf.zone_angle_enter_deg) {
    fm_diagonal_engaged = true;
  } else if (fm_diagonal_engaged && off_axis > usrConf.zone_angle_exit_deg) {
    fm_diagonal_engaged = false;
  }

  // ---- Trailing point. See the OFFSET SIGN CONVENTION block above before touching these signs. ----
  // V2.5-Evo - 2026-07-20 - R0: the "0xFF falls back to usrConf.followme_mode" line was removed
  // here. 0xFF means the TX has never declared a mode this session; runFmLoop() now sends that
  // straight to FM_IDLE, so this function cannot be reached with m == 0xFF. If it somehow were,
  // neither branch below matches and the offset stays 0 (plain Behind) — the safe geometry.
  uint8_t m = fm_mode_runtime.load(std::memory_order_relaxed);

  float offset = 0.0f;                                          // mode 2 Behind
  if (fm_diagonal_engaged) {
    if (m == 1)      offset = -(float)usrConf.near_diag_offset_deg;   // mode 1 Near-Right
    else if (m == 3) offset = +(float)usrConf.near_diag_offset_deg;   // mode 3 Near-Left
  }

  float target_bearing = course + 180.0f + offset;
  projectPoint(anchor_lat, anchor_lng, target_bearing, d_follow, out_lat, out_lng);
}

// ------------------------------------------------------------
// checkFmFaultConditions - the FM FAULT conditions (2-7)
// ------------------------------------------------------------
// What it does (DESIGN_FOLLOW_ME.md section 5, A3 holds-vs-faults classification):
//   Evaluates the six FAULT conditions only - the ones that mean something actually BROKE, so FM
//   must end for the run and a fresh declaration is required to re-arm. Two conditions are handled
//   by the CALLER, not here, because they are not faults:
//     - Condition 1 (throttle >= 25) is the DEADMAN. A trigger release is never a fault (treating
//       it as one would end FM on every release, worse than the original bug); the caller reads it
//       as thr_held and the motor is already 0 by the base architecture when it is low.
//     - Conditions 8 (distance) and 9 (rider speed) are geometric HOLDs: they pause FM (cap 0) but
//       keep it ARMED and auto-resume. The caller evaluates them as dist_ok / speed_ok.
//   Like RTM, any one of these six failing means FM must not be steering. This function does NOT
//   set rtm_rx_emergency_stop - FM stops the motor through its own fm_throttle_cap so the two
//   systems can never fight over one flag.
//
// Returns: true only if all six fault conditions hold.
// Side effects: none (read-only on all globals).
// ------------------------------------------------------------
static bool checkFmFaultConditions()
{
  unsigned long now = millis();

  // 2. Phase A: the RX's own GPS has not been rejected as implausible/spoofed.
  if (gps_rejected) return false;

  // 3. Phase B: the TX<->RX cross-validation handshake is currently passing.
  if (!gps_phase_b_ok) return false;

  // 4. The rider's (TX) GPS position is fresh.
  if (rx_tx_gps_timestamp == 0 ||
      (now - rx_tx_gps_timestamp) > (uint32_t)usrConf.tx_gps_stale_timeout_ms) return false;

  // 5. The buggy's (RX) GPS position is fresh (same 6 s window as RTM gate 5).
  if (gps_last_ms == 0 || (now - gps_last_ms) > 6000UL) return false;

  // 6. A valid heading source exists. V2.5-Evo - 2026-07-20 - A3: FM ALWAYS requires a heading
  //    source, regardless of rtm_compass_required. That flag was an RTM-arming convenience; a
  //    missing heading source in FM means the buggy would steer blind at the ~5% align cap, which
  //    is a FAULT, not something to silently permit. This is the R4 heading-source-loss fix.
  {
    float h_unused; uint8_t conf_unused;
    if (!getRtmHeading(&h_unused, &conf_unused)) return false;
  }

  // 7. The LoRa link is healthy.
  if (now - last_packet > usrConf.failsafe_time) return false;

  return true;
}

// ------------------------------------------------------------
// fmComputeThrottleCap - the FM throttle cap chain
// ------------------------------------------------------------
// What it does (DESIGN_FOLLOW_ME.md section 7):
//   Runs five independent caps and returns the LOWEST. Every cap can only ever reduce the
//   rider's throttle - none of them can raise it. calcPWM() then applies the result with a
//   plain "if (cap < throttle) throttle = cap", so the human trigger stays the only throttle
//   source and FM can only subtract.
//
//   Cap 1 Hard stop      - dist < min_dist_m. Handled by the caller: that condition demotes FM
//                          out of FM_ACTIVE entirely and forces cap 0, so by the time we get
//                          here the buggy is always outside the stop radius.
//   Cap 2 Approach ramp  - linear 255 -> 0 across the smoothing band, same shape as RTM's
//                          approach decel zone. The buggy coasts down as it closes on the rider.
//   Cap 3 Speed governor - hold the buggy toward min(boogie_vmax, rider_speed + closing margin),
//                          measured against the buggy's own GPS speed. Same proportional form as
//                          RTM's run-phase governor, so behaviour is consistent across boogies.
//   Cap 4 Align phase    - while the heading error is large, clamp to ~5% so the buggy pivots
//                          toward the target instead of driving away from it.
//   Cap 5 Engage ramp    - 0 -> full over kFmEngageRampMs on every entry into FM_ACTIVE, so
//                          engaging and re-engaging is always a smooth build, never a jump.
//
// Inputs:  dist_m - current buggy-to-rider distance in metres; now - millis() for this tick
// Returns: the winning cap, 0-255
// Side effects: none.
// ------------------------------------------------------------
static uint16_t fmComputeThrottleCap(float dist_m, unsigned long now)
{
  uint16_t cap   = 255;                                     // start uncapped, take the lowest
  float min_dist = usrConf.min_dist_m;
  float band     = usrConf.followme_smoothing_band_m;

  // ---- Cap 2: approach ramp across the smoothing band ----
  if (band > 0.01f && dist_m < (min_dist + band)) {
    float frac = (dist_m - min_dist) / band;                // 1.0 at the outer edge, 0.0 at the stop radius
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    uint16_t c = (uint16_t)(frac * 255.0f);
    if (c < cap) cap = c;
  }

  // ---- Cap 3: speed governor ----
  float gov = fm_rider_speed_kmh + kFmClosingMarginKmh;
  if (gov > usrConf.boogie_vmax_in_followme_kmh) gov = usrConf.boogie_vmax_in_followme_kmh;
  if (gov > 0.1f) {
    float speed_frac = gps_last_speed_kmh / gov;            // buggy's own GPS speed vs the target
    if (speed_frac > 1.0f) speed_frac = 1.0f;
    if (speed_frac < 0.0f) speed_frac = 0.0f;
    uint16_t c = (uint16_t)((1.0f - speed_frac) * 255.0f);
    if (c < cap) cap = c;
  }

  // ---- Cap 4: align phase ----
  float abs_err = (g_heading_error_dx10 != 0x7FFF) ?
      fabsf((float)g_heading_error_dx10 / 10.0f) : 180.0f;  // no heading data -> treat as worst case
  if (abs_err > (float)usrConf.rtm_align_threshold_deg && kFmAlignCap < cap) {
    cap = kFmAlignCap;
  }

  // ---- Cap 5: engage ramp ----
  if (fm_engage_ms > 0) {
    unsigned long elapsed = now - fm_engage_ms;
    if (elapsed < kFmEngageRampMs) {
      uint16_t c = (uint16_t)(((float)elapsed / (float)kFmEngageRampMs) * 255.0f);
      if (c < cap) cap = c;
    }
  }

  return cap;
}

// ------------------------------------------------------------
// fmEnterIdle - drop FM fully out of the control path
// ------------------------------------------------------------
// Used when FM is switched off (mode 0), when RTM arms and takes the buggy, or when GPS/RTM
// is disabled in config. Clears the throttle cap back to 255 so the rider's manual throttle
// passes through completely untouched, drops the steering override, and cold-starts the rider
// tracking so the next arm begins from a clean filter rather than a stale position.
//
// Deliberately does NOT touch rtm_steer_override or the shared P+D statics - if we are here
// because RTM just armed, RTM now owns those.
//
// V2.5-Evo - 2026-08-17 - AND IF NOBODY OWNS THEM, runRtmLoop() DOES THE CLEANUP. Leaving the
// override alone here is right (we cannot know whether RTM has just taken it), but on its own it
// used to mean a value from the run that just ended stayed parked for the next one. The neutral
// reset for the "nothing engaged at all" case now lives in runRtmLoop(), gated on both
// rtm_rx_active and fm_rx_active being false - see the block there for why it cannot be gated on
// RTM alone.
// ------------------------------------------------------------
static void fmEnterIdle()
{
  // V2.5-Evo - 2026-08-16 - Was FM already idle when we got here? runFmLoop() calls this function
  // on EVERY 100 ms tick while FM is idle (it is the level-triggered "not eligible" branch), so
  // "we are in fmEnterIdle" does NOT mean "an engagement just ended". Everything else below is
  // idempotent and can safely be re-applied ten times a second; the heading-disagreement latch at
  // the bottom is the one thing that must not be, so it is gated on this edge. See the block there.
  bool fm_was_not_idle = (fm_state != FM_IDLE);

  fm_state            = FM_IDLE;
  fm_rx_active        = false;
  fm_throttle_cap     = 255;          // no cap - fully manual buggy
  fm_diagonal_engaged = false;
  fm_engage_ms        = 0;
  fm_filt_init        = false;
  fm_filt_prev_ms     = 0;
  fm_prev_filt_ms     = 0;
  fm_rider_course_deg = -1.0f;
  fm_rider_speed_kmh  = 0.0f;

  // V2.5-Evo - 2026-07-20 - R1/R2: leaving FM entirely drops the separation proof with it.
  // Whatever put us here (mode 0, RTM preemption, mode-age expiry, GPS/FM disabled) ends the
  // declaration, so the next arm must re-prove separation from scratch before FM may engage.
  fm_sep_latched       = false;
  fm_sep_over_since_ms = 0;
  fm_thr_low_since_ms  = 0;

  // V2.5-Evo - 2026-07-20 - A3: clear the steer-cancel persistence timer and the fault-ramp clock.
  // fm_fault_alarm_ms is deliberately NOT reset here: the surprise-gated stop notification must
  // stay sticky for kFmFaultStickyMs even after FM has dropped into FM_IDLE.
  fm_steer_input_since_ms = 0;
  fm_stop_ms              = 0;

  // V2.5-Evo - 2026-07-25 - A3: leaving FM drops any part-accumulated divergence proof with it, so
  // the next engagement starts its 3 s window from scratch rather than inheriting a stale timer.
  // F1: the closure baseline is cleared in the same breath — a distance measured during the previous
  // engagement must never be the yardstick for the next one.
  fm_diverge_since_ms     = 0;
  fm_diverge_start_dist_m = -1.0f;

  // V2.5-Evo - 2026-08-17 - THE FAULT IS NO LONGER FORGIVEN HERE. This site used to clear
  // heading_disagree_fault on a genuine transition into FM idle, on the reasoning that ending a
  // declaration should let the next arm start from fresh sensor evidence. But an FM declaration
  // ending is not evidence about a compass: the compass is exactly as wrong after it as before,
  // and clearing here made the rider re-prove a mis-mounted compass by coasting before every
  // single run — which, given the proof needs kHeadingDisagreeMs of continuous measurement in a
  // window that only exists while coasting, made the guard nearly unreachable.
  //
  // AND IT STAYS GONE NOW THAT A STANDING FAULT BLOCKS AN FM ENGAGEMENT AGAIN. The original worry
  // was that a sticky latch would become an un-clearable block on the water. It is not one:
  // Return-to-Me is NOT blocked (it runs on GPS course, so the rider can always get the buggy
  // home), and the clear route is exactly the state an FM rider spends time in between tows —
  // coasting above rtm_cog_min_speed_kmh with the trigger released, which is precisely when the
  // compass snapshot refreshes and the comparison is live. Five continuous seconds of measured
  // agreement clears it. Forgiving it at a state boundary instead would hand Follow-Me a compass
  // the firmware has positive evidence against, on the strength of nothing at all.
  //
  // The verdict now survives, and the three routes that can clear it are all evidence: a sustained
  // measurement showing agreement, a successful ?compasscal / ?magalign, and a reboot. See the
  // declarations at the top of this file.
  //
  // The UNFINISHED dwell is still dropped here, on the same FM-idle edge as before, so a half-built
  // proof cannot span an FM declaration and whatever comes after it.
  //
  // HISTORY BELOW, kept because it is what constrains the edge test that remains.
  // V2.5-Evo - 2026-08-16 - EDGE, NOT LEVEL. WHAT THE BUG WAS: this clear ran on every call, and
  // runFmLoop() calls fmEnterIdle() on every 100 ms tick whenever FM is idle — and its FM_IDLE
  // condition includes rtm_rx_active. So THE WHOLE TIME RTM WAS RUNNING, the latch and its dwell
  // timer were being zeroed ten times a second. The 5 s dwell could never accumulate, the fault
  // could never be set, and the check that consumes it in getRtmHeading() (in front of the compass
  // fallback) could never see anything: in RTM-only operation — exactly the field case this
  // backstop was built for, a buggy that veered off at close range and never came back — the
  // advertised sticky guard did nothing at all.
  // WHAT THE FIX DOES: clear only on a genuine TRANSITION into FM idle, i.e. when an FM engagement
  // or declaration actually ends. Within an engagement — RTM's included — the latch now survives
  // across ticks, which is the entire point of it being sticky. RTM's own engagement-boundary clear
  // lives at the top of runRtmLoop() (both edges of rtm_rx_active), so neither loop depends on the
  // other one running to have its fault forgiven. FM's matching ARM-edge clear is a few lines into
  // runFmLoop(), just past the idle gate — see the block there for why the start of an engagement
  // needs a clear as well as the end.
  //
  // V2.5-Evo - 2026-08-17 - the !rtm_rx_active guard that used to protect the FAULT clear here is
  // gone with the clear itself. It existed because runFmLoop()'s idle gate includes rtm_rx_active,
  // so the tick on which RTM ARMS drives FM into idle and the clear would have wiped a proven fault
  // a few microseconds after RTM had decided to carry it into the run (loop() calls runRtmLoop()
  // before runFmLoop()). With no fault clear left at this site, that whole hazard is structural
  // rather than guarded: neither loop can forgive a compass verdict at a state boundary.
  if (fm_was_not_idle) {
    // Unfinished proof only — the verdict is not a boundary's business.
    heading_disagree_since_ms     = 0;
    heading_disagree_last_seen_ms = 0;
  }
}

// ------------------------------------------------------------
// runFmLoop - the Follow-Me state machine. Call from loop().
// ------------------------------------------------------------
// What it does (DESIGN_FOLLOW_ME.md sections 4-7):
//   Runs at 10 Hz, the same cadence as runRtmLoop(). Every tick it re-evaluates all nine
//   activation/hold conditions and moves FM between IDLE / ARMED / ACTIVE / DEMOTED. While
//   ACTIVE it computes the trailing target, hands it to the shared steering controller, and
//   recomputes the throttle cap chain.
//
//   Mutual exclusion with RTM is absolute: if rtm_rx_active is set, FM drops to IDLE and stops
//   writing anything into the control path. RTM arming therefore silently disarms FM, which is
//   the existing documented behaviour.
//
// Inputs:  fm_mode_runtime (0xFF = no TX declaration this session = FM_IDLE), fm_mode_last_rx_ms,
//          all GPS/link globals, the eight FM SPIFFS parameters.
// Outputs: fm_rx_active, fm_throttle_cap, rtm_steer_override (via updateRtmSteering),
//          fm_target_lat/lng.
// Side effects: MOTOR-RELEVANT. fm_throttle_cap can reduce throttle and rtm_steer_override can
//   redirect steering, but only ever through calcPWM()'s existing subtract-only chain and only
//   while the rider is holding the trigger.
// ------------------------------------------------------------
void runFmLoop()
{
  // Rate-limit to 10 Hz (matches runRtmLoop; the geometry maths costs ~1 ms per call).
  static unsigned long last_fm_ms = 0;
  unsigned long now = millis();
  if (now - last_fm_ms < 100UL) return;
  last_fm_ms = now;

  // Keep the rider filter and derived motion warm on every tick, in every state, so the
  // instant the conditions are met we already have a trustworthy course and speed.
  updateFmRiderTracking();

  // ---- Resolve the active mode ----
  // V2.5-Evo - 2026-07-20 - R0: the "0xFF falls back to usrConf.followme_mode" line is GONE.
  // WHAT THE BUG WAS: 0xFF means "the TX has not declared an FM mode this session". Falling
  // back to the SPIFFS value meant a factory RX (defaultConf.followme_mode = 1) booted
  // LATENTLY ARMED — no gesture, no declaration, and nothing on the display to say so. A rider
  // holding the trigger beyond the engage distance would have handed steering to FM without
  // ever asking for it. WHAT THE FIX DOES: 0xFF now means FM_IDLE, always, and 0xFF is greater
  // than 3 so the mode gate below catches it. usrConf.followme_mode keeps exactly one job —
  // it is the value the TX's arm gesture SEEDS from (TX RTMState.ino). It is never again an
  // RX-side auto-arm source. Autonomous steering now always requires a live human declaration.
  uint8_t m = fm_mode_runtime.load(std::memory_order_relaxed);

  // ---- R2(b): expire a declaration nobody is refreshing ----
  // The TX re-sends 0xF2/mode every 30 s while armed. If no refresh has arrived for
  // kFmModeAgeMs (95 s, ~3 missed keepalives), the declaration is stale — most likely the
  // TX disarmed and its 0xF2/0 burst was lost in the air, or the TX is gone. Drop to FM_IDLE
  // and reset the runtime mode to 0xFF so re-arming requires a fresh declaration.
  if (m >= 1 && m <= 3) {
    unsigned long mode_ms = fm_mode_last_rx_ms.load(std::memory_order_relaxed);
    if (mode_ms == 0 || (now - mode_ms) > kFmModeAgeMs) {
      Serial.println("FM [RX] mode declaration expired (no 0xF2 refresh) -> IDLE");
      fm_mode_runtime.store(0xFF, std::memory_order_relaxed);
      fmEnterIdle();
      return;
    }
  }

  // ---- FM_IDLE: FM off / never declared (0xFF), RTM owns the buggy, or GPS/RTM disabled ----
  if (!usrConf.gps_en || !usrConf.rtm_rx_enabled || rtm_rx_active || m < 1 || m > 3) {
    fmEnterIdle();
    return;
  }

  // ============================================================
  // V2.5-Evo - 2026-08-16 - FM ARM BOUNDARY: forgive a heading disagreement proven while nothing
  // was armed. This is FM's half of the arm-edge clear; RTM's half is the rtm_rx_active
  // false -> true edge at the top of runRtmLoop(), which carries the full rationale.
  //
  // WHY IT IS HERE, ONE LINE BELOW THE IDLE GATE. Reaching this point means FM has a live TX
  // declaration, RTM does not own the buggy, and GPS/RTM are enabled — i.e. this tick is part of an
  // arm cycle. fm_state is still whatever the PREVIOUS tick left it as, and every tick that gets
  // past this gate ends by writing FM_ARMED, FM_ACTIVE, FM_HOLD or FM_STOPPING before it returns.
  // So "fm_state is still FM_IDLE here" is true on exactly one tick per arm cycle: the first one.
  // That makes this a genuine EDGE with no extra bookkeeping variable, and it covers both ways out
  // of idle (IDLE -> ARMED, and the rarer IDLE -> ACTIVE when the latch is already proven).
  //
  // AND IT IS DELIBERATELY BELOW THE GATE, NOT ABOVE IT. Above the gate this line would run on
  // every tick RTM owns the buggy — FM is held in FM_IDLE for the whole of an RTM engagement — and
  // that is exactly the wipe-every-tick bug this whole change set exists to undo.
  //
  // The clear runs BEFORE checkFmFaultConditions() below calls getRtmHeading(), so a disagreement
  // that is still genuinely present is re-measured and re-opens its dwell on this very tick; it
  // simply has to prove itself again over kHeadingDisagreeMs, from the start of the run.
  //
  // V2.5-Evo - 2026-08-17 - THIS EDGE NO LONGER CLEARS THE FAULT, ONLY THE UNFINISHED DWELL —
  // matching RTM's arm edge, and for the identical reason. The paragraph above says the fault
  // "simply has to prove itself again over kHeadingDisagreeMs, from the start of the run". IT
  // CANNOT: a comparison needs a compass snapshot younger than kHeadingCompareSnapMs, and
  // Compass.ino only refreshes that snapshot while thr_received < 25. FM engages under a HELD
  // trigger, so about one second into the run the comparison goes dormant and the 5 s dwell can
  // never complete. The evidence exists only BEFORE the run — while the rider coasts with the
  // trigger released above rtm_cog_min_speed_kmh — so clearing the verdict here deleted the one
  // thing the guard had to work with. The unfinished dwell is still dropped, because a half-built
  // proof must not span two different situations.
  //
  // WHAT A SURVIVING FAULT COSTS FOLLOW-ME, AND WHY THAT IS THE POINT (V2.5-Evo - 2026-08-17). It
  // makes can_be_active false, so FM stays in FM_ARMED, fully manual, cap 255, until the compass is
  // exonerated or re-measured. That is the intended behaviour, not a dead end: FM steers a towing
  // buggy at a person, and the guard's finding is "one of my two heading sources is lying and I
  // cannot tell which". Declining is the only answer that does not involve guessing. The real
  // complaint about the old shape was that it was SILENT — fm_flags bit 2 dropped once separation
  // latched, so the TX read "ready" while nothing happened. That is fixed where it belongs: bit 2
  // carries the fault again, the ARMED branch at the bottom of runFmLoop() prints a rate-limited
  // explanation, headingDisagreeAnnounceDegraded() prints once when the fault latches, and ?diag
  // reports the latch on demand. RTM, whose degraded behaviour is bounded, keeps running throughout,
  // so the rider is never stranded — see can_be_active for the full argument.
  // ============================================================
  if (fm_state == FM_IDLE) {
    heading_disagree_since_ms     = 0;
    heading_disagree_last_seen_ms = 0;
  }

  // ---- FM_STOPPING: a FAULT ended FM; ramp throttle back to manual, then go IDLE (A3) ----
  // V2.5-Evo - 2026-07-20 - A3 FAULT semantics. Once a fault has stopped FM this run, autonomy is
  // over until a fresh declaration. We do NOT re-check the conditions here: even if the fault
  // clears mid-ramp, FM stays down and requires re-arm (silent resume after an anomaly is exactly
  // the unrequested autonomy this architecture forbids). We only ramp the throttle cap back up so
  // the rider regains manual control smoothly, then drop to FM_IDLE.
  // MOTOR SAFETY: the cap only ever RISES toward 255 (subtract-only, never adds throttle); the
  // rider's held trigger stays the sole throttle source, and starting the ramp from 0 means no
  // lurch. (RTM preemption / GPS-off / mode-off above still abort straight to IDLE.)
  if (fm_state == FM_STOPPING) {
    fm_rx_active       = false;
    rtm_steer_override = 127;
    unsigned long stop_elapsed = now - fm_stop_ms;
    if (stop_elapsed >= kFmStopRampMs) {
      // Ramp done: require a fresh TX declaration to re-arm (mirror the mode-age expiry path so the
      // TX must re-send 0xF2/mode; the TX also learns of the fault via fm_flags and clears its own
      // fm_armed - see runRtmLoop's fm_flags bit 3).
      fm_mode_runtime.store(0xFF, std::memory_order_relaxed);
      fmEnterIdle();
      return;
    }
    fm_throttle_cap = (uint8_t)(((float)stop_elapsed / (float)kFmStopRampMs) * 255.0f);
    return;
  }

  // ---- R2(a): throttle-release clear — the end-of-run session boundary ----
  // The trigger being released for kFmThrReleaseClearMs (10 s) means this run is over: the
  // rider is swimming, resting, or rigging for the next tow. Clear the separation latch so
  // the next tow starts unproven and FM cannot engage on the rope. The declared MODE survives
  // (the rider still intends to use FM) — only the geometric proof is discarded.
  // MOTOR SAFETY: restoring cap 255 here cannot produce motion. We only reach this branch
  // because the trigger has been released for 10 s, and the deadman already holds the motor
  // at 0 while it is released. When the rider next squeezes, the latch is clear, so FM is not
  // eligible to engage and the buggy is fully manual — the safe steady state.
  if (thr_received < 25) {
    if (fm_thr_low_since_ms == 0) {
      fm_thr_low_since_ms = now;
    } else if ((now - fm_thr_low_since_ms) >= kFmThrReleaseClearMs) {
      if (fm_sep_latched || fm_state == FM_HOLD) {
        Serial.println("FM [RX] throttle released 10s -> separation latch cleared, ARMED-unlatched");
      }
      fm_sep_latched       = false;
      fm_sep_over_since_ms = 0;
      if (fm_state != FM_IDLE) {
        fm_state        = FM_ARMED;
        fm_throttle_cap = 255;   // back to fully manual; trigger is released, so no motion
        fm_rx_active    = false;
      }
    }
  } else {
    fm_thr_low_since_ms = 0;
  }

  // ---- Evaluate the conditions, split by A3 class ----
  // DEADMAN = condition 1 (throttle). FAULT = conditions 2-7. HOLD = conditions 8-9.
  bool  thr_held = (thr_received >= 25);       // condition 1 (DEADMAN — never a fault)
  bool  fault_ok = checkFmFaultConditions();   // conditions 2-7 (FAULT)
  bool  hard_ok  = thr_held && fault_ok;       // both needed for a trustworthy distance / latch
  bool  speed_ok = false;                      // condition 9 (HOLD — rider moving)
  bool  dist_ok  = false;                      // condition 8 (HOLD — follow geometry)
  float dist_m   = 0.0f;
  // V2.5-Evo - 2026-07-25 - A3: sustained divergence while ACTIVE. Classed as a FAULT (same family
  // as conditions 2-7), so it is routed through the SAME FM_STOPPING path below — never its own.
  bool  diverge_fault = false;
  // V2.5-Evo - 2026-07-25 - F7: the numbers the divergence message prints, captured at detection but
  // PRINTED LATER — in the fault branch, after fm_throttle_cap = 0. WHAT THE BUG WAS: the message was
  // printed at the moment of detection, which is upstream of the cap write, so a full UART TX buffer
  // could block inside Serial.printf() and delay the hard stop by however long the host took to drain
  // it. The motor must reach 0 first and the explanation can wait; nothing else reads these.
  float diverge_limit_m = 0.0f;   // the ceiling (kFmDivergeFactor x D_engage) that was exceeded, m
  float diverge_start_m = 0.0f;   // the distance captured when the dwell started, m

  if (hard_ok) {
    // Both GPS sources are guaranteed fresh here by conditions 4 and 5.
    dist_m = (float)TinyGPSPlus::distanceBetween(
        gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);

    float min_dist = usrConf.min_dist_m;
    float band     = usrConf.followme_smoothing_band_m;

    // Condition 9 (HOLD) with RESUME hysteresis. Below foiler_low_speed_kmh the rider may be down
    // or swimming, and the buggy must not manoeuvre around them — but a fall is normal and
    // recurring, so this is a HOLD (stays ARMED), never a fault. When already ACTIVE, stay down to
    // the plain threshold; when trying to (re)engage from HOLD/ARMED, require foiler_low_speed_kmh
    // + kFmSpeedHystKmh so FM cannot flap on and off at the speed line (mirrors the distance Schmitt).
    if (fm_state == FM_ACTIVE)
      speed_ok = (fm_rider_speed_kmh >= usrConf.foiler_low_speed_kmh);
    else
      speed_ok = (fm_rider_speed_kmh >= (usrConf.foiler_low_speed_kmh + kFmSpeedHystKmh));

    // Condition 8: Schmitt hysteresis on distance so FM cannot flap at the band edge.
    //   to ENGAGE  : the rider must be beyond min_dist + band
    //   to STAY ON : hold until the rider is inside min_dist
    if (fm_state == FM_ACTIVE) dist_ok = (dist_m >= min_dist);
    else                       dist_ok = (dist_m >  (min_dist + band));

    // ---- R1: separation latch (the tow interlock) ----
    // Before FM may engage for the first time this run, the rider must be proven genuinely
    // OFF THE ROPE: beyond D_engage (9 m at the 4+2 tuning, clearing the measured 20 ft /
    // 6.10 m rope) continuously for kFmSepDwellMs. The dwell is what makes this spike-proof:
    // a one-fix GPS glitch cannot hold the distance high across 4 consecutive 2 Hz fixes.
    // Once latched it STAYS latched until a §R2 clear, so the buggy may close back to its
    // normal 6 m station and re-engage on the ordinary Schmitt hysteresis without ever having
    // to re-prove separation mid-wave.
    // Same degenerate-config guard computeFmTarget() applies to d_follow: a zeroed min_dist
    // and band would otherwise make D_engage 0 and the latch would set on the first tick,
    // silently disabling the whole interlock.
    float d_follow_e = min_dist + band;
    if (d_follow_e < 0.5f) d_follow_e = 0.5f;

    // V2.5-Evo - 2026-07-25 - A2: honour the fm_engage_dist_m override.
    // WHAT WAS WRONG: the field has existed in confStruct since SW34 and ConfigService validates it
    // (0-50 m), and the web UI shows a row for it — but no code anywhere ever READ it, so turning the
    // knob changed nothing. WHAT THIS DOES: when set above zero, the stored value IS the engage
    // distance, in METRES. It is NOT the rope length — MEASURE the rope and set at least a metre
    // beyond it (the measured 20 ft / 6.10 m rope -> 8 m, which is also the enforced floor).
    // 0 = auto: D_engage = kFmEngageFactor (1.5) x d_follow. The 0.1f compare is the
    // float "is this really zero" guard, not a second threshold — ConfigService already clamps the
    // range to 0-50 m.
    // V2.5-Evo - 2026-07-25 - F3-c amendment: the auto branch is no longer bit-for-bit the pre-A2
    // behaviour — it is now floored at kFmEngageDistFloorM as well (see the F3-c block below). At
    // the 4+2 tuning auto yields 9.0 m, above the floor, so nothing changes at the shipped setting.
    //
    // V2.5-Evo - 2026-07-25 - F5 comment correction. The A2 note here used to claim D_engage feeds
    // "the distance Schmitt hysteresis". IT DOES NOT, and saying so was misleading about what this
    // knob actually moves. The condition-8 Schmitt a dozen lines above works purely off min_dist and
    // min_dist + band; it never looks at D_engage. The two things that DO consume D_engage are: (1)
    // the separation latch immediately below, together with its kFmSepDwellMs dwell, and (2) the A3
    // divergence ceiling further down, which is kFmDivergeFactor x D_engage. Nothing else reads it.
    //
    // V2.5-Evo - 2026-07-25 - F3 defensive floor. WHAT THE BUG WAS: ConfigService accepted any value
    // in 0-50 m with no lower bound above zero, so a stored 3 m was legal — and 3 m is SHORTER than
    // the tow rope. Since this value IS the engage distance, that setting let FM engage while the
    // rider was still on the rope: the exact scenario the separation latch exists to prevent.
    // V2.5-Evo - 2026-07-25 - F3-b: that floor was 5.0 m, which was itself below the rope it exists
    // to clear (the owner's rope is 20 ft = 6.10 m), so 5.0-6.1 m stayed legal and stayed on-rope.
    // It is now kFmEngageDistFloorM = 8.0 m, defined once in BREmote_V2_Rx.h and shared with the
    // validator — no duplicated literal.
    // WHAT THE FIX DOES: cfgValidateCrossField() refuses to STORE anything in (0, kFmEngageDistFloorM),
    // and the clamp below is the belt-and-braces companion for a value already sitting in SPIFFS from
    // before that rule existed — such a config is never re-validated, so without the clamp it would
    // still reach the latch. Behaviour is unchanged: a legacy stored value clamps UP to the floor.
    // 0 (auto) is still accepted by the validator and still takes the auto branch below.
    //
    // V2.5-Evo - 2026-07-25 - F3-c: THE FLOOR NOW GUARDS BOTH BRANCHES, NOT JUST THE TYPED VALUE.
    // WHAT THE BUG WAS: kFmEngageDistFloorM used to be applied INSIDE the manual branch only. The
    // auto branch (fm_engage_dist_m = 0) computed kFmEngageFactor (1.5) x (min_dist_m +
    // followme_smoothing_band_m) and used that product RAW. Those two SPIFFS values have no lower
    // bound of their own — min_dist_m 1 m with a 1 m smoothing band is a perfectly storable tuning,
    // and it yields d_engage = 1.5 x 2 = 3 m. Three metres is BELOW the measured 20 ft / 6.10 m tow
    // rope, so Follow-Me could set the separation latch and engage with the rider still ON the rope:
    // the exact hazard the floor exists to prevent, simply reached down the other branch. The floor
    // was guarding the number the rider TYPES while leaving the number the firmware COMPUTES open.
    // WHAT THE FIX DOES: each branch now only computes its candidate, and ONE clamp is applied to the
    // final d_engage regardless of which branch produced it. The tow rope is a physical fact about
    // this buggy, not a property of the config path, so the safety limit belongs on the result.
    // WHY IT IS SAFE IN BOTH DIRECTIONS: the clamp can only ever RAISE d_engage, never lower it. A
    // larger d_engage means the rider must separate FURTHER before FM may engage — it can only make
    // engagement later and harder, never earlier or easier.
    // NO-OP AT THE OWNER'S TUNING: min_dist_m 4 + followme_smoothing_band_m 2 = 6 m, x 1.5 = 9.0 m,
    // already above the 8.0 m floor. Nothing changes at the shipped setting; the clamp only bites on
    // a small-geometry tuning that would otherwise have produced an on-rope engage distance.
    // KNOCK-ON, CHECKED: the A3 divergence ceiling further down is kFmDivergeFactor x d_engage, so a
    // floored d_engage raises that ceiling in the same proportion — the detector becomes MORE
    // permissive, never less, and cannot be made to fire spuriously by this change.
    float d_engage;
    if (usrConf.fm_engage_dist_m > 0.1f) {
      d_engage = usrConf.fm_engage_dist_m;      // manual: the stored value IS the engage distance, in metres
    } else {
      d_engage = kFmEngageFactor * d_follow_e;  // auto: derived from the follow geometry
    }
    // F3-c: single tow-rope safety floor, applied to the COMPUTED value as well as the typed one.
    if (d_engage < kFmEngageDistFloorM) d_engage = kFmEngageDistFloorM;
    if (dist_m > d_engage) {
      if (fm_sep_over_since_ms == 0) {
        fm_sep_over_since_ms = now;
      } else if (!fm_sep_latched && (now - fm_sep_over_since_ms) >= kFmSepDwellMs) {
        fm_sep_latched = true;
        Serial.printf("FM [RX] separation latch SET: dist=%.1f m > D_engage=%.1f m sustained %lu ms\n",
                      dist_m, d_engage, (unsigned long)kFmSepDwellMs);
      }
    } else {
      fm_sep_over_since_ms = 0;   // fell back inside D_engage - the dwell restarts from scratch
    }

    // ---- A3: DIVERGENCE FAULT — the upper bound FM never had ----
    // V2.5-Evo - 2026-07-25. Condition 8 above is a lower bound only, so a buggy steering the WRONG
    // WAY satisfies it more and more comfortably the further it runs. This adds the missing ceiling:
    // while FM is ACTIVE, being further than kFmDivergeFactor x D_engage from the rider AND FAILING
    // TO CLOSE for kFmDivergeMs is not "following badly", it is "not following", and it is a FAULT.
    //
    // V2.5-Evo - 2026-07-25 - F1: this used to be a BARE THRESHOLD (beyond the ceiling for the dwell
    // = fault) and that aborted legitimate engagements. Two reasons, and they compound. First, the
    // engage ramp is kFmEngageRampMs = 3500 ms but the dwell is only kFmDivergeMs = 3000 ms, so the
    // fault could fire BEFORE the buggy had finished being given throttle. Second, while the heading
    // error is still large, cap 4 pins the throttle at kFmAlignCap = 13/255 (~5%) so the buggy pivots
    // in place and the gap GROWS before it starts to shrink — and at the engagement instant dist_m is
    // typically 13-21 m against an 18 m ceiling (2 x a 9 m D_engage). Net effect: FM aborted ~3 s into
    // most real engagements and forced a re-arm mid-session.
    //
    // THE FIX IS IN TWO PARTS, and both are needed:
    //
    //   1. TEST THE DERIVATIVE, NOT THE LEVEL. This is what runPhaseC() actually does — its
    //      convergence check fails on "dist_m >= rtm_prev_dist_m", i.e. on NOT CLOSING, never on being
    //      far. We do the same in FM's own terms: snapshot the distance when the dwell starts
    //      (fm_diverge_start_dist_m) and, at dwell expiry, fault only if the buggy has failed to close
    //      by more than kFmDivergeCloseEpsM. If it HAS closed by more than that, it is following — just
    //      from further back than we would like — so the timer is cleared and no fault fires. The next
    //      tick starts a fresh window, so a buggy that is beyond the ceiling and genuinely closing is
    //      re-tested every 3 s and keeps passing for exactly as long as it keeps closing.
    //      (Why "must be DECREASING" from runPhaseC is not copied verbatim: RTM closes on a stationary
    //      rider so any non-decrease is wrong, whereas FM holds station behind a MOVING rider and the
    //      distance legitimately rises and falls every wave. The epsilon is what carries that across.)
    //
    //   2. ENGAGE GRACE. The detector is skipped entirely, and its dwell parked, for
    //      kFmEngageRampMs + kFmDivergeMs (3500 + 3000 = 6500 ms) after every entry into FM_ACTIVE.
    //      The buggy must be allowed to finish ramping AND aligning before its geometry is judged;
    //      judging it mid-ramp measures the ramp, not the steering. Parking the dwell (rather than
    //      letting it run) guarantees the first post-grace window is a full, clean kFmDivergeMs.
    //      NOTE: this is deliberately NOT kFmEngageGraceMs (2000 ms) — that constant is the
    //      steer-cancel grace and is a different, shorter window for a different purpose.
    //
    // Bookkeeping is otherwise the same shape runPhaseC() uses: one timer plus one baseline, evaluated
    // every tick, both cleared the instant the condition stops holding, so nothing can accumulate
    // across a gap. The dwell is a plain first-exceed timestamp compared against millis() — no delay(),
    // no blocking, no extra loop.
    //
    // Only evaluated while fm_state == FM_ACTIVE (FM actually has control this tick). ARMED, HOLD and
    // STOPPING are all states in which FM is not steering, so distance says nothing about divergence.
    //
    // SAFETY: this branch only ever sets a flag that REMOVES eligibility. It cannot raise the
    // throttle cap, cannot extend engagement, and does not touch the deadman. Both parts of the fix
    // make the detector STRICTLY LESS likely to fire, never more — a missed divergence still leaves
    // every other fault condition and the deadman in place, and the rider can always let go.
    bool in_engage_grace = (fm_engage_ms != 0) &&
                           ((now - fm_engage_ms) < (kFmEngageRampMs + kFmDivergeMs));

    if (in_engage_grace) {
      // Ramping and/or aligning — not judgeable yet. Park the window so it starts fresh afterwards.
      fm_diverge_since_ms     = 0;
      fm_diverge_start_dist_m = -1.0f;
    }
    else if (fm_state == FM_ACTIVE && dist_m > (kFmDivergeFactor * d_engage)) {
      if (fm_diverge_since_ms == 0) {
        // First tick beyond the ceiling: start the dwell and record what we are closing FROM.
        fm_diverge_since_ms     = now;
        fm_diverge_start_dist_m = dist_m;
      } else if ((now - fm_diverge_since_ms) >= kFmDivergeMs) {
        if (dist_m >= (fm_diverge_start_dist_m - kFmDivergeCloseEpsM)) {
          // Beyond the ceiling for the full dwell and NOT closing — this is divergence.
          // F7: the numbers are stashed and printed later, after fm_throttle_cap = 0.
          diverge_fault   = true;
          diverge_limit_m = kFmDivergeFactor * d_engage;
          diverge_start_m = fm_diverge_start_dist_m;
        } else {
          // It has closed by more than the epsilon: the buggy IS following, just far. No fault —
          // clear the window so the next tick opens a fresh one from the current distance.
          fm_diverge_since_ms     = 0;
          fm_diverge_start_dist_m = -1.0f;
        }
      }
    } else {
      // Back inside the limit, or FM is not ACTIVE — the dwell restarts from scratch.
      fm_diverge_since_ms     = 0;
      fm_diverge_start_dist_m = -1.0f;
    }
  } else {
    // No trustworthy distance this tick (trigger released, GPS stale/rejected, link down).
    // Restart the dwell rather than carrying a half-finished proof across a data gap.
    fm_sep_over_since_ms = 0;
    // A3: same discipline for the divergence dwell — never judge divergence on data we do not trust.
    // F1: the closure baseline goes with it; a baseline must never outlive the dwell that set it.
    fm_diverge_since_ms     = 0;
    fm_diverge_start_dist_m = -1.0f;
  }

  // The separation latch gates eligibility. Without it FM stays ARMED and the buggy stays
  // fully manual, no matter how well the other conditions read.
  // V2.5-Evo - 2026-07-25 - A3: !diverge_fault joins the same AND chain. It can only ever REMOVE
  // eligibility, so the worst case of a false positive is FM handing control back to the rider.
  // V2.5-Evo - 2026-07-25 - STAGE 2 added !heading_disagree_fault to this chain on the same terms.
  //
  // V2.5-Evo - 2026-08-17 - IT WAS REMOVED FOR ONE PASS AND IS RESTORED HERE. RELEASE BLOCKER.
  //
  // WHY THE REMOVAL WAS WRONG. It rested on "the flag is evidence against the COMPASS, so FM can
  // safely run on the GPS course instead". The cross-check does not support that sentence. It
  // proves only that the compass and the GPS course CANNOT BOTH BE RIGHT — guard 2's own header
  // says so: "It deliberately does NOT pick a winner." If the RX's course is the bad source
  // (marina multipath, a wrong dynamic platform model, a lagged COG at low speed), degrading to
  // course-only withdraws the GOOD sensor and hands Follow-Me the bad one. The evidence is
  // symmetric; the response must be too.
  //
  // WHY FOLLOW-ME DECLINES WHERE RTM DEGRADES — the two are not the same risk, and treating them
  // alike is what produced this. RTM's degraded behaviour is BOUNDED: with no heading the steering
  // override is pinned at 127, the align cap holds the throttle at 13/255 on the 180 deg sentinel,
  // Phase C's convergence check keeps watching, and the alternative was a half-armed buggy sitting
  // dead with the TX still displaying RTM as ACTIVE. FOLLOW-ME HAS NO SUCH CEILING: it engages
  // autonomously once the rider is beyond the engage distance at speed, its steering authority is
  // continuous for the whole engagement, and its only backstops are the divergence fault (about a
  // 6.5 s engage grace plus a 3 s dwell before it can even fire) and the deadman.
  //
  // AND THE DISAGREEMENT CANNOT BE RE-MEASURED DURING THE RUN, which is what makes engaging on it
  // unrecoverable rather than merely optimistic. compare_possible needs a compass snapshot younger
  // than kHeadingCompareSnapMs, and Compass.ino only refreshes that snapshot while thr_received <
  // 25. FM runs under a held trigger, so about a second into the engagement the comparison goes
  // dormant and the GPS course is served at confidence 3, unchallenged, for the rest of the run.
  // The firmware would be steering at a person on a source it holds positive evidence against,
  // with no way left to notice it was the wrong one.
  //
  // IT IS ALSO WHAT GUARD 1 ALREADY DOES, one screen up in getRtmHeading(): "One provably dead
  // source plus one unverifiable source is not a heading. HOLDING STRAIGHT IS SAFER THAN STEERING
  // ON THE SURVIVOR." Guard 2 has strictly WEAKER evidence than that — it cannot even name the
  // dead source — so it must not reach a bolder conclusion.
  //
  // THE COST, STATED PLAINLY: FM will not engage until the compass is exonerated (five continuous
  // seconds of measured agreement, which happens while coasting with the trigger released), or
  // re-measured by ?compasscal / ?magalign, or the board is rebooted. The rider is not left
  // guessing — fm_flags bit 2 rises, the ARMED branch below prints a rate-limited explanation, the
  // one-shot degradation notice has already printed, and ?diag answers on demand. And RTM keeps
  // working throughout, so the buggy can always be brought home.
  bool can_be_active = hard_ok && speed_ok && dist_ok && fm_sep_latched &&
                       !diverge_fault && !heading_disagree_fault;

  if (can_be_active) {
    // ---- Steer-cancel while ACTIVE (A3 PART 2 / R-steering) ----
    // If the rider is ALREADY following and applies a sustained steering input, they have taken
    // manual control: exit to ARMED and CLEAR the separation latch, so FM cannot silently resume —
    // it may only re-engage after a fresh >D_engage / kFmSepDwellMs separation is re-proven. This
    // is a rider DECLARATION, not a fault, so no alarm fires and the mode is kept. Two guards stop
    // the tail of the whip-separation steering from killing a just-engaged FM:
    //   1. kFmEngageGraceMs grace after engaging (ignore steer-cancel entirely for the first 2 s);
    //   2. kFmSteerPersistMs persistence past the deadband (a brief blip never cancels).
    // ARMED (not following) has NO steer-cancel path at all — that is what makes the whip-by-
    // steering separation safe: steering while merely armed is just steering. v1 policy = cancel;
    // the advanced steer-adjust-while-following blend (rtm_steer_exit_on_input == 0) is DEFERRED —
    // that param lives on the TX, not the RX, this pass.
    if (fm_state == FM_ACTIVE) {
      int sdev = (int)steering_received - 127;
      if (sdev < 0) sdev = -sdev;
      if (sdev >= (int)kFmSteerCancelDeadband) {
        if (fm_steer_input_since_ms == 0) fm_steer_input_since_ms = now;
      } else {
        fm_steer_input_since_ms = 0;
      }
      bool past_grace = (fm_engage_ms != 0) && ((now - fm_engage_ms) >= kFmEngageGraceMs);
      bool persisted  = (fm_steer_input_since_ms != 0) &&
                        ((now - fm_steer_input_since_ms) >= kFmSteerPersistMs);
      if (past_grace && persisted) {
        Serial.println("FM [RX] steer-cancel -> ARMED-UNLATCHED (separation latch cleared, no alarm)");
        fm_state                = FM_ARMED;
        fm_sep_latched          = false;   // deliberate: no silent resume, separation must re-prove
        fm_sep_over_since_ms    = 0;
        fm_steer_input_since_ms = 0;
        fm_rx_active            = false;
        rtm_steer_override      = 127;
        fm_engage_ms            = 0;
        fm_throttle_cap         = 255;      // manual; trigger may be held, but the latch is now clear
        return;
      }
    }

    // ---- FM_ACTIVE ----
    if (fm_state != FM_ACTIVE) {
      // Entering FM_ACTIVE from ARMED or HOLD.
      fm_engage_ms            = now;    // start the engage ramp - re-engagement is never a jump
      fm_diagonal_engaged     = false;  // re-evaluate which side we are on for this engagement
      fm_steer_input_since_ms = 0;      // ignore any pre-engagement deflection; the grace starts now

      // Reset the shared P+D derivative continuity. Without this the controller would
      // differentiate a fresh heading error against a stale pre-engagement sample across the
      // gap and command a violent phantom turn on the first tick.
      prev_heading_src_valid  = false;
      prev_heading_error_deg  = 0.0f;
      prev_steering_update_ms = 0;

      fm_state = FM_ACTIVE;
      Serial.printf("FM [RX] ENGAGE mode %u: dist=%.1f m rider=%.1f km/h course=%.0f\n",
                    (unsigned)m, dist_m, fm_rider_speed_kmh, fm_rider_course_deg);
    }

    fm_rx_active = true;                                   // gate the steering override on
    computeFmTarget(&fm_target_lat, &fm_target_lng);       // where behind the rider to sit
    updateRtmSteering();                                   // shared P+D controller, unchanged
    fm_throttle_cap = (uint8_t)fmComputeThrottleCap(dist_m, now);
  }
  else {
    // ---- Not eligible to steer — classify the drop (A3 DEADMAN / HOLD / FAULT) ----
    fm_rx_active            = false;
    rtm_steer_override      = 127;   // hand steering straight back to the rider
    fm_engage_ms            = 0;     // any re-engagement ramps from zero again
    fm_steer_input_since_ms = 0;

    bool was_engaged = (fm_state == FM_ACTIVE || fm_state == FM_HOLD);

    // V2.5-Evo - 2026-08-17 - heading_disagree_fault IS a member of this condition again, restored
    // in the same breath as the can_be_active term it was deleted with. IT HAS TO BE, and this is
    // not symmetry for its own sake: restoring only can_be_active would drop an engaged FM into the
    // HOLD branch below instead of this one, because fault_ok can still be true (a live GPS course
    // satisfies condition 6). HOLD means cap 0, stay ARMED, auto-resume when the conditions come
    // back — and a latched fault never comes back on its own, so FM would sit at cap 0 with the
    // rider holding the trigger and the motor dead, with no stop notification and no ramp back to
    // manual. That is the same silent half-armed state this whole change set exists to remove,
    // wearing FM's colours.
    // Entering HERE instead gives it the proven fault semantics: FM_STOPPING, cap 0 now, the
    // kFmStopRampMs ramp back to full manual throttle, FM_IDLE, re-arm required, and the sticky
    // fm_flags bit 3 that drives St + the stop buzz on the TX.
    // V2.5-Evo - 2026-08-17 - PRECISION ON HOW LONG THAT DEAD MOTOR LASTS, because this note used to
    // say the buggy would sit at cap 0 "for the rest of the session" and that overstates it. The
    // R2(a) throttle-release clear a few screens up rescues an FM_HOLD: thr_received < 25 held for
    // kFmThrReleaseClearMs (10 s) CONTINUOUSLY puts FM back in FM_ARMED at cap 255, so a rider who
    // simply lets go and waits does get their manual buggy back. THE REAL HAZARD IS NARROWER AND
    // STILL WORTH FIXING: that clear needs ten CONTINUOUS seconds below 25 counts, and every squeeze
    // resets fm_thr_low_since_ms to zero. A rider FEATHERING the trigger — squeeze, nothing happens,
    // release, squeeze again — never accumulates the ten seconds, so they get a DEAD MOTOR ON EVERY
    // SQUEEZE for as long as they keep trying, with no stop notification, no ramp back to manual and
    // nothing on the TX saying why. Which is exactly what a rider does when the buggy stops
    // responding. The code is unchanged: routing this fault through the branch below is still right.
    if ((!fault_ok || diverge_fault || heading_disagree_fault) && was_engaged) {
      // ---- FAULT (conditions 2-7, plus A3 divergence): something actually broke while FM had control ----
      // End autonomy for the run: enter FM_STOPPING, which ramps the throttle cap 0 -> 255 over the
      // next kFmStopRampMs (handled at the top of runFmLoop), then drops to FM_IDLE — re-arm
      // required. Fire the stop notification (sticky fm_flags bit 3, drives St + stop buzz on the
      // TX) only if the trigger was held at this instant — a fault after release is not surprising,
      // and the bar going dark carries it. (V2.5-Evo - 2026-08-17: "only" now has exactly one
      // documented exemption, the heading-disagree latch; see the note directly above the write.)
      // R4: heading loss is one of these faults now.
      // V2.5-Evo - 2026-07-25 - A3: sustained divergence enters through THIS branch and no other, so
      // it inherits the proven fault semantics unchanged — hard stop to cap 0 now, the same ramp back
      // to manual, the same haptic/St notification, and the same mandatory re-arm. Steering away is
      // exactly as much of a "something broke" event as losing the compass, and a silent auto-resume
      // after it would be the unrequested autonomy this architecture forbids.
      // V2.5-Evo - 2026-07-25 - STAGE 2 routed a sustained COG-vs-compass disagreement through this
      // same branch, and V2.5-Evo - 2026-08-17 it does so again after one pass in which it did not.
      // A disagreement proven while FM has control is a "something broke" event of exactly the same
      // kind as losing the compass or steering away: two heading sources that cannot both be right,
      // and no way to tell which, discovered mid-engagement. It ends autonomy for the run and hands
      // the buggy back to the rider, which is what every other member of this branch does.
      // V2.5-Evo - 2026-08-17 - AND THE HEADING-DISAGREE FAULT IS EXEMPT FROM THE SURPRISE GATE.
      // WHAT WAS WRONG: fm_fault_alarm_ms was set only if (thr_held), but this particular fault can
      // ONLY be proven with the trigger RELEASED — the compass snapshot the comparison needs stops
      // refreshing above 25 counts, so the latch completes while coasting and never while pulling.
      // The alarm was therefore gated on a condition that is never true at the moment it fires: the
      // sticky fm_flags bit 3 never rose, the TX never learned the run had ended on a fault (so it
      // never cleared its own fm_armed), and Follow-Me re-armed silently on the next 0xF2 keepalive
      // into an ARMED state that cannot engage — whose only signal in the field is the not-ready
      // flag. WHY THE EXEMPTION IS NARROW AND CORRECT: the gate exists so a rider who CAUSED a stop
      // is not alarmed about their own action. Nobody causes this one by holding the trigger; it is
      // only ever provable when they are off it, so the gate defeats itself here and nowhere else.
      // Every other fault in this branch keeps the thr_held gating exactly as it was.
      if (thr_held || heading_disagree_fault) fm_fault_alarm_ms = now;
      fm_stop_ms      = now;
      fm_state        = FM_STOPPING;
      fm_throttle_cap = 0;         // subtract-only hard stop; the ramp begins next tick
      // V2.5-Evo - 2026-07-25 - F7: ALL fault logging happens BELOW this line, never above it. The
      // divergence detail used to print at the point of detection, which is upstream of the cap write
      // — so if the USB CDC TX buffer was full (host not draining) Serial.printf() could block and
      // defer the hard stop for as long as the host took. Motor to 0 first, explain afterwards.
      if (diverge_fault) {
        Serial.printf("FM [RX] DIVERGENCE FAULT: dist=%.1f m (was %.1f m at dwell start, closed <%.1f m) > limit %.1f m sustained %lu ms — not closing\n",
                      (double)dist_m, (double)diverge_start_m, (double)kFmDivergeCloseEpsM,
                      (double)diverge_limit_m, (unsigned long)kFmDivergeMs);
      }
      // V2.5-Evo - 2026-08-17 - CAUSE AGAIN, not just context: with the fault back in the condition
      // above, this line names the reason the run ended whenever the latch is what ended it. It
      // still reads correctly in the other case — an FM stopped by condition 6 while the latch
      // happens to stand — because the missing fallback is then the honest explanation for why the
      // GPS course had no backup. Same F7 discipline as the line above: it prints AFTER
      // fm_throttle_cap = 0, so a blocked UART can never defer the hard stop.
      if (heading_disagree_fault) {
        Serial.printf("FM [RX] HEADING DISAGREE FAULT: the compass and the GPS course disagreed by more than %.0f deg for %lu ms, so one of them is wrong and the board cannot tell which. Follow-Me will not steer on a guess — it stays manual until they are measured agreeing again, or you re-run ?compasscal / ?magalign.\n",
                      (double)kHeadingDisagreeDeg, (unsigned long)kHeadingDisagreeMs);
      }
      Serial.printf("FM [RX] FAULT -> STOPPING (ramp %lu ms) -> IDLE, re-arm required (thr_held=%d)\n",
                    (unsigned long)kFmStopRampMs, (int)thr_held);
    } else if (was_engaged) {
      // ---- HOLD (cond 8/9) or DEADMAN (cond 1): a geometry / throttle pause, NOT a fault ----
      // Motor stops (cap 0) but FM stays ARMED (declaration held) and auto-resumes to FM_ACTIVE
      // once the conditions restore AND the latch is set. No alarm. This is what lets a rider
      // fall, slow, or close on the buggy repeatedly without ever having to re-arm.
      fm_state        = FM_HOLD;
      fm_throttle_cap = 0;         // subtract-only hard stop: motor to 0
    } else {
      // ---- FM_ARMED: never engaged this arm cycle — fully manual buggy ----
      // The throttle chain stays INACTIVE (cap 255) so the rider keeps full manual control while
      // FM waits for the follow geometry. ARMED has no steer-cancel path (see the ACTIVE branch).
      fm_state        = FM_ARMED;
      fm_throttle_cap = 255;

      // V2.5-Evo - 2026-08-17 - THE "ARMED, NOT ENGAGING" EXPLANATION IS BACK, because the state it
      // describes is back: can_be_active carries heading_disagree_fault again, so a proven
      // disagreement holds FM here indefinitely with every other condition possibly reading
      // perfectly. Without a word on the wire that is indistinguishable from "the geometry is not
      // right yet", and the rider stands there waiting for an engagement that is never coming.
      //
      // RATE LIMIT. This branch runs at 10 Hz for as long as FM is armed, so the message repeats no
      // more often than kFmHeadingBlockMsgMs. That is deliberately NOT a one-shot like
      // headingDisagreeAnnounceDegraded(): that notice fires once at the instant of the latch and
      // will already have scrolled away by the time the rider gets to the water and arms. This one
      // has to be there when they are actually looking, which is while they are wondering why
      // nothing is happening. ?diag reports the same state without waiting for the repeat.
      //
      // F7 SAFE: FM is fully manual on this branch (fm_throttle_cap = 255, written above), so there
      // is no motor-stopping write downstream that a blocked UART could defer.
      if (heading_disagree_fault) {
        static const unsigned long kFmHeadingBlockMsgMs = 10000UL;  // ms between repeats
        static unsigned long fm_heading_block_msg_ms = 0;
        if (fm_heading_block_msg_ms == 0 ||
            (now - fm_heading_block_msg_ms) >= kFmHeadingBlockMsgMs) {
          fm_heading_block_msg_ms = now;
          Serial.printf("FM [RX] ARMED, NOT ENGAGING: the compass and the GPS course were measured disagreeing by more than %.0f deg for %lu ms, so Follow-Me will not steer on either of them. RTM still works. Clears on %lu ms of measured agreement (coast with the trigger released), or ?compasscal / ?magalign. A reboot does NOT clear it (the verdict is stored in SPIFFS since 2026-08-18).\n",
                        (double)kHeadingDisagreeDeg, (unsigned long)kHeadingDisagreeMs,
                        (unsigned long)kHeadingDisagreeMs);
        }
      }
    }
  }
}
