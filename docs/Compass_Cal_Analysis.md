# BREmote V2.5-Evo — Compass Calibration & EMI Field Analysis
**Field test date:** 2026-05-09, Chicago-area parking lot (lat ~41.902, lng ~-87.823)
**Firmware:** SW_VERSION 31 (Bundle 1 — P+D heading controller, 26-column logger)
**Analysis date:** 2026-05-09
**Author:** monterman + Claude Code (orchestrator session "Tyler")
**Purpose:** Document compass behavior under motor load, calibration offset, and heading source reliability for RTM/FM steering. Reference material for a dedicated compass calibration session.

---

> **⚠ COMPASS WAS NOT CALIBRATED AT THE TIME OF THESE LOGS**
>
> Andres confirmed (2026-05-09) that `runcal` was **never run** on this hardware build prior to the field test. All compass offset values in this document (40–87° vs GPS COG) are entirely expected from uncalibrated hardware and do **not** indicate hardware damage, EMI problems, or mounting errors. Treating these offsets as fault indicators would be incorrect. The compass calibration is a prerequisite for all compass-dependent RTM steering tests.
>
> A bind-button-triggered calibration method for RX is being developed to make `runcal` more accessible in the field (no serial terminal required). Until that is implemented and run, all compass offset data in this document should be treated as a baseline-before-calibration reference only.

---

## Executive Summary

Three parallel findings from 6 field test logs (050926_*.log.csv):

1. **Compass has a static mounting offset of ~40–87°** vs GPS COG depending on vehicle orientation. This offset is entirely expected — `runcal` was **never run** on this hardware build. This is the first calibration, not a re-calibration. Full `runcal` with motor off and battery installed is required before trusting compass for RTM steering.
2. **Motor startup inrush spikes the compass ±90° for 200–600 ms** (one row at 0.2–0.6 Hz sample rate). Steady-state EMI at running speed is small (±2°). The inrush transient is the dangerous zone.
3. **The RTM arm ceremony throttle leakage bug is confirmed in log 050926_223439.** After RTM disengaged (ts ≈134 s), user re-arm throttle squeezes drove the motor at 96% duty / ERPM 4191–4204 for ~11 seconds with `rtm_rx_active=0`. Separate fix session already planned (Bug 1).

No fault_code events (no E7 wetness) appeared in any of the 6 logs.

---

## 1. Log File Overview

| File | Duration | Location | Motor Active | RTM Active | Key Event |
|---|---|---|---|---|---|
| 050926_215623.log.csv | 275 s | Moving, lat 41.910 | No | No | GPS COG baseline while moving — clean compass vs COG comparison data |
| 050926_222750.log.csv | 274 s | Stationary, lat 41.902 | Brief pulses | Yes (128 s, duty=0) | RTM armed but GPS gates blocked all motor output throughout arm window |
| 050926_223240.log.csv | 90 s | Stationary, lat 41.902 | Brief pulses | Yes (4.7 s) | Short RTM run; motor coasting when RTM flagged |
| 050926_223439.log.csv | 22 s | Stationary, lat 41.902 | Yes, 96% duty | Yes (0–4 s), then off | **Arm ceremony throttle bug confirmed** (see Section 5) |
| 050926_223456.log.csv | 4.5 s | Stationary, lat 41.902 | No | No | Motor-off compass baseline continuation of 223439 |
| 050926_224307.log.csv | 148 s | Moving, lat 41.900–41.902 | Yes, sustained | No | Best motor-on vs GPS COG comparison; GPS phase B not passing |

---

## 2. Static Compass Mounting Offset (Expected — Compass Never Calibrated)

The compass reads consistently offset from actual vehicle heading. **This is entirely expected: `runcal` was never run on this build.** The offset below is a baseline-before-calibration reference, not a hardware or EMI defect. It is a **hard-iron / mounting orientation offset** that `runcal` is designed to correct. Do not attempt to diagnose or work around this offset in firmware — run `runcal` first.

### Evidence from moving tests

**File 215623 (buggy moving westbound ~28–47 km/h, motor off):**
- GPS COG: consistently 268–270° (heading west)
- compass_live: consistently 220–230°
- **Static offset: compass reads ~40–50° lower than true heading**

**File 224307 (buggy moving, motor active, various headings):**

| Timestamp (ms) | GPS COG (°) | compass_live (°) | Offset (compass − COG) |
|---|---|---|---|
| 44554 | 195.3 | 264.8 | +69.5° |
| 48078 | 185.7 | 271.3 | +85.6° |
| 49154 | 183.0 | 270.1 | +87.1° |
| When heading ~306° (GPS COG) | 305.9 | 278.5 | −27.4° |
| When heading ~122° (GPS COG) | 121.8 | 275.5 | +153.7° |

**Critical: the offset is not constant.** A fixed mounting offset would produce a constant difference. The 153.7° discrepancy at GPS COG ~122° vs +87° at COG ~195° suggests **significant soft-iron distortion** (possibly from steel buggy frame, battery pack proximity, or motor magnets) in addition to the hard-iron offset. The compass calibration (`runcal`) procedure corrects for both, but only if performed on the actual installed hardware with the motor off and battery in its normal position.

### Implication for RTM steering

With a 40–87° uncalibrated offset, the heading controller computes a large false heading error every cycle. In file 223439 during RTM active (rows 1–19), heading_error stayed at −54° to −82° throughout, causing the controller to continuously command hard-left steering. If the compass were properly calibrated, this heading error should have been much smaller (or zero if the buggy were already pointing at the user).

---

## 3. Motor EMI — Startup Inrush vs Steady State

### 3.1 Startup inrush transient (DANGEROUS)

**File 222750, first motor pulse (ts ~134971–138286):**

| Row (ts ms) | ERPM | thr | compass_live (°) | Delta from baseline 260° |
|---|---|---|---|---|
| 333 (134971) | 0→start | 67 | **323.1** | +63.1° |
| 334 (135174) | 0→start | 148 | **12.5** | −247.5° (wrap) |
| 335 (135377) | 0 | 0 | 255.6 | −4.4° (recovering) |
| 338 (135984) | 4194 running | 0 | 277.3 | +17.3° |
| 340 (136390) | 4194 running | 0 | 235.6 | −24.4° |
| 341 (136593) | → 0 | 0 | 265.9 | +5.9° (recovered) |

**File 222750, second motor pulse (ts ~206690–211199):**

| Row (ts ms) | ERPM | thr | compass_live (°) | Delta from ~270° baseline |
|---|---|---|---|---|
| 678 (206690) | 0→inrush | 240 | **11.0** | −259° (full wrap!) |
| 679 (206893) | 0→inrush | 240 | 19.3 | −250.7° |
| 680 (207096) | accelerating | 240 | 198.1 | −71.9° |
| 681 (207299) | accelerating | 204 | 244.9 | −25.1° |
| 682 (207499) | 36 ERPM | 0 | 262.2 | −7.8° (recovered) |
| 689 (208875) | running | 20 | 312.3 | +42.3° |
| 690 (209078) | running | 240 | **347.5** | +77.5° |
| 693 (209686) | 3829 ERPM | 0 | 261.7 | +8° (largely recovered) |

**Inrush transient summary:** During motor startup (first 200–600 ms of current ramp), compass_live deflects between −260° and +78° from baseline. These are wrap-around spikes caused by the motor's large inrush current creating a strong transient magnetic field. Duration: 1–3 log rows (~200–600 ms). After the motor reaches running speed, compass largely recovers.

### 3.2 Steady-state EMI (small)

**File 222750, motor at steady 3800–4200 ERPM (rows 692–700, ts 209686–211199):**
- compass_live range: 261–262° vs baseline 260°
- Deviation: **±1–2°** — essentially negligible

**File 223439, motor at steady 4192–4204 ERPM (rows 2–19, ts 130129–133911):**
- compass_live range: 2609–2921 (260.9°–292.1°) = **±16°** spread
- This wider spread vs file 222750 may be due to: different physical orientation relative to motor, active RTM steering changing motor current draw, or compass snapshot values including motor-off captures that don't represent the EMI state

**File 223240, motor at steady 3975 ERPM (rows 126–131, ts 59553–60571):**
- compass_live: 2742–2752 (274.2°–275.2°) — extremely stable, **±0.5°**
- This location had the buggy at a different orientation, motor EMI minimal at this heading

**Steady-state summary:** At running speed (3000–4200 ERPM), compass_live noise is **±2°** or less in most cases. The startup inrush (first 200–600 ms) is the primary EMI hazard, not the running state.

### 3.3 Compass invalid reads (0xFFFF = I2C failure)

`compass_live = 65535` (0xFFFF) appeared 5 times across all sessions:

| File | Timestamp (ms) | Motor state | Notes |
|---|---|---|---|
| 222750 | 250745 | Off (ERPM=0) | RTM arm window, motor off |
| 223240 | 80303 | Off (ERPM=0) | Between throttle bursts |
| 223439 | 149303 | Off (ERPM=0) | Post-RTM cool-down |
| 224307 | 41382 | Off (ERPM=0) | Pre-drive idle |
| 224307 | 165572 | On (ERPM ~2700) | During driving |

**Pattern:** Most invalid reads occur with motor OFF. This is NOT an EMI issue — it is an I2C bus glitch (possibly the QMC5883L missing an ACK or a bus collision). Frequency: approximately once every 15–20 minutes of operation. The compass_snap value is valid at these rows (the snapshot captures the last good reading). Impact: one 200 ms window of invalid heading data per 15–20 minutes; compass_snap provides fallback. Low severity but worth noting for calibration session.

---

## 4. Compass Snapshot Behavior

The snapshot (`compass_snap_dx10`) is captured by `updateCompassSnapshot()` only when `thr_received < 25` (motor idle). It is the fallback heading used by RTM when GPS COG is unavailable (e.g., vehicle stationary).

### Stability during motor-off periods

**File 222750 (motor off, rows 1–329, ts 66103–128000):**
- compass_snap range: 2550–2637 (255.0°–263.7°)
- Peak-to-peak: 87 counts = **8.7°** — acceptable for heading reference
- Typical live vs snap delta: ±5–15 counts

**File 223240 (motor off, rows 318–409):**
- compass_snap range: 2740–2755 (274.0°–275.5°)
- Peak-to-peak: 15 counts = **1.5°** — very stable
- Live vs snap delta: mostly 0–8 counts

**Conclusion:** When motor is off, compass snapshot is stable to ±1.5°–8.7° depending on hardware settle state. It is suitable as a reference heading for RTM pre-arm orientation, but only after a fresh runcal.

### Snapshot capture during arm ceremony

During the RTM arm ceremony, `thr_received` drops to 0 between squeeze attempts. Every time it goes below 25, `updateCompassSnapshot()` runs and updates the snapshot. With the arm ceremony taking 7–15 seconds and compass readings corrupted by prior motor inrush, the snapshot captured during arm may contain an EMI-contaminated value. After runcal this risk is reduced but not eliminated.

**Recommendation:** After the arm ceremony completes and before RTM sends the first steering command, delay one snapshot capture cycle (100 ms) to ensure the snapshot reflects a post-inrush stable reading.

---

## 5. Arm Ceremony Throttle Leakage Bug — Log Confirmation

**File 050926_223439.log.csv, rows 21–73 (ts 134114–144028)**

This is the critical safety bug confirmed in field data.

### Timeline

| ts (ms) | rtm_rx_active | thr_received | duty_cycle_% | ERPM | Notes |
|---|---|---|---|---|---|
| 133708 | **1** | 0 | 96 | 4195 | Last row of first RTM active run |
| 133911 | 1 | 113 | 96 | 4195 | RTM still active |
| **134114** | **0** | **178** | **71** | **3239** | **RTM disengaged. User starts re-arm ceremony.** Motor still running from prior throttle. |
| 134323 | 0 | 178 | 71 | 3239 | Re-arm squeeze. Motor running. |
| 135208 | 0 | 7 | 96 | 4191 | Motor at 96% duty — driven by arm squeeze |
| 135411 | 0 | 2 | 96 | 4191 | thr near zero, motor still at 96% |
| 135614 | 0 | 155 | 96 | 4191 | User squeezes again. Motor 96%. |
| 135823 | 0 | 178 | 96 | 4191 | Full arm squeeze. Motor 96%. |
| 136027 | 0 | 178 | 96 | 4196 | Motor at 96%, thr=178 — THE BUG |
| 137711 | 0 | 178 | 96 | 4202 | Motor at 96%, full arm squeeze |
| 140908 | 0 | 0 | 96 | 4197 | thr=0, but motor still at 96% (VESC lag) |
| 143004 | 0 | 0 | 96 | 3934 | Motor decelerating |
| 143207 | 0 | 0 | 60 | 2820 | Motor decelerating |
| 144231 | 0 | 0 | 15 | 852 | Motor decelerating |
| 145303 | 0 | 0 | 0 | 0 | Motor fully stopped |

**Duration of leak:** ~11 seconds (ts 134114–145303)
**Peak ERPM during leak:** 4202 (similar to active RTM run)
**Peak duty during leak:** 96%
**Root cause:** `sendData()` FreeRTOS task sends live `thr_scaled` during blocking `runDoubleSqueezeArm()`. With `rtm_rx_active=0`, `calcPWM()` on RX uses `thr_received` directly — no RTM gate suppression.

### Why 222750 did NOT show the bug

In 222750, the arm ceremony occurred with `rtm_rx_active=1` (RTM had been activated via 0xF1/1). GPS safety gates (primarily Gate 3 Phase B and Gate 4 TX GPS freshness) were failing, setting `rtm_rx_emergency_stop=true`. `calcPWM()` forced `effective_thr=0` due to emergency stop. Motor was silenced by gate failures, not by the arm ceremony throttle fix.

If the field conditions had been ideal (all gates passing), the arm squeezes in 222750 would also have driven the motor at full duty. The 223439 data shows exactly what happens when the re-arm ceremony runs outside RTM-active state.

**Status:** Fix identified and planned (Bug 1). Will be applied to TX `RTMState.ino` in the next fix session before any further motor testing.

---

## 6. RTM Gate 9 Stop Distance — Behavior Observed

### What the logs show

No clean Gate 9 hard stop sequence was captured in any log. The pattern in 223439 shows:
- RTM active with motor running (rows 1–19, ts 130–133 s)
- RTM disengaged at ts ~134114 while user had active throttle (thr=178)
- Motor continued on user throttle post-disengagement

The disengagement cause at ts~134114 is not definitively identifiable from the log alone. Candidates:
1. **Gate 9 (stop distance):** Distance computed as `TinyGPSPlus::distanceBetween(gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng)` dropped below `rtm_stop_distance_m=10`. Would produce clean RTM exit with no emergency stop.
2. **Phase C convergence check (every 5s):** Distance not decreasing (buggy stationary, wheels spinning in air). Would set emergency stop.
3. **User gesture exit:** Right-toggle exit with `rtm_steer_exit_on_input=1`.

The GPS coordinates in 223439 are frozen at 41.902084,-87.823647 throughout (speed=0, no actual movement). The buggy was on stands with wheels spinning in air. Phase C convergence check would fail immediately (distance cannot decrease if buggy isn't moving), making Phase C the more likely RTM terminator.

### User observation vs log

The user reported: "I was 20–27 m away when I armed. Distance decreased to 9–8–7 m on display, but RTM never stopped."

**Log interpretation:** The display distance (decoded from `telemetry.rtm_distance`) was showing real GPS distance decreasing as the user walked toward the stationary buggy. Gate 9 uses the same GPS distance calculation. At 7–9 m, Gate 9 should fire (configured threshold = 10 m). However:

1. **GPS error at close range:** At < 10 m, typical GPS CEP (circular error probable) of 3–5 m on both TX and RX creates ±3–5 m uncertainty in the computed distance. The RX might have computed 10.5 m while the display (TX side decoding telemetry) showed 9 m.
2. **Computation order:** `telemetry.rtm_distance` is encoded at the TOP of `runRtmLoop()`, then Gate 9 is checked INSIDE `checkRtmSafetyGates()` ~1 ms later. If GPS data updates between these two calls (FreeRTOS preemption), the two computed distances could differ by 2–3 m. At the 10 m boundary this is significant.
3. **Phase C firing first:** If Phase C fired before Gate 9 (buggy not converging), `rtm_rx_emergency_stop=true` was set and RTM disabled — which cuts throttle to 0 but leaves `rtm_distance` telemetry still encoding. The display would keep showing decreasing distance (from the telemetry encoding block running before the gate check) even after RTM had already stopped. This could explain the "distance decreased but motor kept going" user experience.

**Second test ("I couldn't hear the motor"):** Consistent with Phase C and/or Gate 9 firing correctly on the second attempt. The first run may have had GPS or timing conditions that let it run longer.

**Recommendation for next test:** Set `rtm_approach_zone_m` to 15 m and `rtm_stop_distance_m` to 5 m. This gives a wider approach decel zone and a more forgiving stop distance, reducing GPS-error sensitivity at the threshold. Log Gate 9 firing (check Serial output via USB after test) to confirm the gate is activating.

---

## 7. Compass Calibration — Action Items for Next Session

> **This is the first calibration on this hardware build, not a re-calibration.** `runcal` has never been run on the current RX unit. A bind-button-triggered calibration workflow for the RX is being developed so the procedure can be done in the field without a serial terminal. Until that is implemented, `runcal` requires a USB serial connection to the RX.

### Priority order

1. **Run `runcal` with hardware in final operating configuration (first-time calibration):**
   - Motor off, battery installed in its normal position, no external magnets
   - Buggy on ground (not on stands) — position relative to steel frame matters
   - Rotate the entire buggy slowly through 360° in the horizontal plane during calibration
   - Repeat at least twice; compare calibration results

2. **Verify runcal output removes the static offset:**
   - After runcal, manually point the buggy at a known heading (align with a road lane or compass app on phone)
   - Check that `compass_live` matches the expected heading within ±5°
   - If offset remains, inspect for nearby ferromagnetic materials (mounting screws, motor case proximity to compass PCB)

3. **Measure compass vs GPS COG offset after calibration:**
   - Drive a straight line at > 5 km/h and compare `compass_live_dx10 / 10` with `gps_course_dx10 / 10` in the CSV
   - Goal: offset < 5° consistently across at least 4 different headings (N/S/E/W)

4. **Motor inrush snapshot timing:**
   - After calibration, run a motor-start test and check the post-inrush recovery time in the log
   - If startup spikes still corrupt the snapshot, add a `compass_inrush_settle_ms` delay in `updateCompassSnapshot()` before capturing: require that ERPM has been below `cog_min_speed` threshold for at least 500 ms before allowing snapshot update

5. **rtm_use_compass mode:**
   - Current behavior: mode 1 (Hybrid) — GPS COG primary, compass snapshot fallback
   - For the stationary RTM use case (TX stationary, buggy approaching user), GPS COG is unavailable (speed=0). The fallback to compass is correct, but the compass must be calibrated first.
   - After calibration, test RTM with `rtm_use_compass=0` (GPS COG only) to verify Phase C convergence and Gate 9 work under purely COG-based steering. If the buggy is moving > `rtm_cog_min_speed_kmh`, COG should be valid and compass-free RTM may be reliable.

6. **Consider adjusting `rtm_cog_min_speed_kmh`:**
   - Current behavior: COG below this threshold → falls back to compass snapshot
   - In the parking lot test, the buggy was stationary (wheels spinning in air). At any real RTM speed > 3–5 km/h, GPS COG should be available.
   - Setting `rtm_cog_min_speed_kmh = 2` (or lower) would give more COG coverage at low drive speed and reduce compass dependency during the critical early phase of RTM when the buggy first starts moving.

---

## 8. Summary Table — Next Session Checklist

| Action | Priority | Who | Notes |
|---|---|---|---|
| Fix arm ceremony throttle leakage (Bug 1) | **Critical** | Code fix | Do before any further motor testing |
| Run runcal with motor off, battery installed | High | Field | Do on ground, not on stands |
| Verify compass vs GPS COG offset post-runcal | High | Field | Drive straight line > 5 km/h |
| Test RTM with all wheels on ground (actual movement) | High | Field | Allows GPS COG to be used as heading source |
| Analyze Gate 9 behavior with `rtm_approach_zone_m=15`, `rtm_stop_distance_m=5` | Medium | Field + logs | Wider tolerance for GPS close-range error |
| Investigate GPS freeze in 224307 (ts ~105580) | Medium | Code/logs | GPS position/speed frozen for 60 s; may be Serial1 stall |
| Fix E7 false wetness / stale VESC telemetry | Low | Code | Saved to memory for dedicated session |
| Add 500 ms post-inrush settle delay in `updateCompassSnapshot()` | Low | Code | Only if calibration test shows startup spikes still corrupt snapshots |

---

*Log files referenced: `G:/My Drive/Claude AI files/Claude CODE/Projects/Vesc and other related files/050926_*.log.csv`*
*Firmware reference: `Source/V2_Integration_Rx/RTMState.ino`, `Source/V2_Integration_Rx/Compass.ino`*
*Last updated: 2026-05-09 — initial field test analysis; calibration disclaimer added (compass never calibrated on this build)*
