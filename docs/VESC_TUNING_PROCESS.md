# VESC Smooth-Start Tuning Process — Iterative Method with Log Analysis

For: Andres, BREmote V2.5-EVO + FlipSky FSESC 75200 V2 + 6384 outrunner + sensorless FOC underwater
App: VESC Tool 6.06 (Android, Google Play)
Date created: 2026-05-03
Companion docs: `VESC_SMOOTH_START_QUICK_REFERENCE.md`, `VESC_FOC_TUNING_NOTES.md`, `THROTTLE_PIPELINE_ANALYSIS.md`

---

## TL;DR — your current symptom and the immediate next move

**Symptom:** With boost = 3 the motor starts well from cold-stop, but if you partially release the trigger and re-engage WHILE THE MOTOR IS STILL SPINNING DOWN, you get "duck-duck-duck" stutter, sometimes catches, sometimes doesn't, sometimes you have to wait until full stop before it'll restart.

**Diagnosis:** Classic spinning-restart problem in sensorless FOC. The motor is rotating at, say, 500–1000 ERPM (still spinning down). Your `Open loop ERPM` threshold is 1500, so VESC sees the motor as "below sensorless" and tries to forcibly restart in openloop mode. But the forced rotation rate clashes with the actual rotor angle → each "duck" is a bad commutation cycle until the rotor either catches up or fully stops.

**Fix (Step 2 in the original guide):** Lower **`Open loop ERPM`** from **1500 → 1000**. With 1000, the observer takes over at 1000 ERPM. When you re-engage at 800–1000 ERPM, the observer locks onto the actual rotor position immediately — no openloop forcing, no clash, smooth resume.

This is your next single change. After bench-testing it, you decide whether to go further (Step 3 lowers `foc_sl_erpm_start` and `foc_sl_erpm`).

---

## 1. THE PROCESS (the loop you'll follow forever)

Tuning a sensorless FOC motor is iterative. You change ONE parameter, test it under a defined condition, observe the result (symptom + log), decide the next change. The doc you're reading is the methodology.

```
┌──────────────────────────────────────────────────────────┐
│  1. SET BASELINE                                          │
│     BREmote: throttle_mode=1, thr_expo=50 (linear)        │
│     RTM: disarmed, system unlocked                        │
│     This eliminates BREmote-side curve interference so    │
│     trigger position correlates 1:1 with motor command.   │
├──────────────────────────────────────────────────────────┤
│  2. SAVE CURRENT VESC CONFIG AS XML (rollback point)     │
│     Motor Settings → menu → Save XML                      │
│     Name it: motor_<date>_<change>.xml                    │
├──────────────────────────────────────────────────────────┤
│  3. CHANGE ONE PARAMETER                                  │
│     Write to VESC, Save to flash                          │
├──────────────────────────────────────────────────────────┤
│  4. RUN A DEFINED TEST                                    │
│     With logging enabled (see §2 below)                   │
│     Each test is one of: cold start, restart-while-       │
│     spinning, full-throttle ramp, sustained low throttle  │
├──────────────────────────────────────────────────────────┤
│  5. SAVE THE LOG (CSV) AND OBSERVE THE SYMPTOM            │
│     Note: trigger position, audible behavior,             │
│     visual smoothness, any faults.                        │
├──────────────────────────────────────────────────────────┤
│  6. INTERPRET (see §4 diagnosis matrix)                   │
│     If better: keep the change. If worse: load rollback.  │
├──────────────────────────────────────────────────────────┤
│  7. NEXT PARAMETER (if needed) — go back to step 2        │
└──────────────────────────────────────────────────────────┘
```

**Iron rule: change ONE parameter at a time.** If you change three things and something improves, you don't know which change did it. If something gets worse, you don't know which change broke it.

---

## 2. HOW TO RECORD VESC LOGS IN THE ANDROID APP (VESC Tool 6.06)

### 2.1 First-time setup (do this once)

1. Open VESC Tool 6.06.
2. Connect to the FSESC over Bluetooth.
3. Go to **Settings** (gear icon) or the side menu → **Developer**.
4. **"Choose File folder"** — pick a folder on your phone storage. Easiest: `Downloads` or create a new folder called `VESC_logs`.
5. Tick **"Enable RT Data Logging"**.
6. Tick **"Log to CSV"** (if separate option).
7. Close settings.

If you get "Could not open file for writing" — create a new folder via Files app first, then re-pick that folder in VESC Tool's Developer settings. Restart the app if needed.

### 2.2 Per-session recording

Each tuning test:

1. Connect to FSESC.
2. Go to **Real-Time Data** (or **Realtime → RT Data**).
3. Tap the **record / log button** (usually a red dot or "Start logging" icon at top).
4. **Run the test** (see §3 below for what tests to run).
5. Tap **stop logging**.
6. The CSV is saved to your chosen folder with a timestamp filename.
7. Optionally rename the file to describe what you tested, e.g.:
   - `2026-05-03_boost3_restart-while-spinning.csv`
   - `2026-05-03_boost3_openloopRPM1000_coldstart.csv`

### 2.3 What ends up in the CSV
The CSV typically contains these columns (one row every ~50ms during recording):
- Timestamp (ms or s since recording started)
- Voltage (input battery)
- Motor RPM (mechanical)
- ERPM (electrical RPM = mechanical × pole pairs)
- Duty Cycle (0–1)
- Battery Current (A)
- Motor Current (A) — Q-axis
- Motor Temperature (°C)
- ESC Temperature (°C)
- Fault Code (0 = no fault)
- Position (electrical angle)
- Other auxiliary fields

The most useful for tuning: **ERPM, Motor Current, Duty Cycle, Fault Code, and time**.

---

## 3. THE THREE TESTS YOU'LL RUN EVERY ITERATION

Run each test with logging on. Each isolates one behavior. Don't skip — the whole process depends on knowing which behavior changed.

### Test A — COLD START
- Motor fully stopped (wait at least 3 seconds after last spin).
- From rest, ramp trigger smoothly from 0% to 30% over ~2 seconds.
- Hold at 30% for 2 seconds.
- Release.
- **What we're checking:** does the motor break free smoothly? At what trigger %? Any audible chirp/stutter at the transition?

### Test B — RESTART-WHILE-SPINNING (your current pain point)
- Bring motor to 50% trigger, run for 2 seconds.
- Release trigger fully (motor coasts down).
- After **0.5 seconds** of coasting, re-engage at 50%.
- Release. Wait for full stop.
- Repeat with **1.0 second** coast time, then **2.0 seconds**, then **5.0 seconds**.
- **What we're checking:** at what coast duration does the duck-duck stutter happen? At what duration does it go away? This tells us exactly which ERPM range the observer is failing in.

### Test C — SLOW LOW-THROTTLE CRUISE
- From rest, slowly bring trigger to 15% over 5 seconds. Hold at 15% for 10 seconds.
- Slowly release.
- **What we're checking:** is sustained low-throttle smooth? Any vibration or growl? Is the motor producing useful thrust at 15%?

---

## 4. DIAGNOSIS MATRIX — SYMPTOM → CAUSE → PARAMETER TO CHANGE

| Symptom | Likely cause | First parameter to try | Notes |
|---|---|---|---|
| Motor needs >25% trigger to break free from cold stop | Boost too low for static load | **Open loop current boost +1 to +2 A** | You're at 3, try 5 if cold-start is still hesitant |
| Duck-duck-duck stutter when re-engaging mid-spin | `Open loop ERPM` threshold too high → openloop forces commutation against still-spinning rotor | **Lower `Open loop ERPM` 1500 → 1000** | This is your next move |
| Motor starts then jerks at "transition" point | Sensorless ERPM threshold mismatch — observer not ready when openloop hands off | **Lower `Sensorless ERPM Start` 2500 → 1500 AND `Sensorless ERPM` 3400 → 2500** | Step 3 — only after Step 2 verified |
| Motor jumps suddenly at 0% trigger | Boost leaking with zero command | **Lower boost** (try 2 if currently 3) | Verify VESC firmware is recent enough to suppress boost at zero command |
| Motor very hot at low throttle | Boost too high, dissipating real I²R losses in stalled windings | **Lower boost** | Hard ceiling: 8 A. If hot at 3 A, something else is wrong (mechanical bind, prop fouled). |
| Motor commutates wrong direction occasionally | Observer giving bad estimates during transition | Lower `Open loop ERPM` slowly, retest | If persistent, motor parameters may have drifted — re-run Motor Detection. |
| Audible whine at idle (above 3 A boost) | High-frequency PWM artifact | Try lowering `foc_f_zv` from 30000 to 25000 (FOC → General) | Cosmetic; not safety-relevant |
| Fault code 6 (Undervoltage) on spin-up | Battery sag from inrush + boost current | Lower `l_in_current_max` if you want gentler inrush, OR lower boost | Check battery health first |
| Fault code 11 (Encoder Spi error) | Not relevant — sensorless mode | Ignore | Only matters for sensored setups |

---

## 5. WHAT TO LOOK FOR IN THE CSV

Open the CSV in a spreadsheet (Excel, Google Sheets, or even a phone CSV viewer). Plot or scroll through these columns over time. Focus on the section where the symptom occurred:

### Pattern: smooth start (good)
- ERPM: ramps cleanly from 0 to several thousand in ~1 second
- Motor current: rises to a peak (~5–15 A) then settles down once spinning
- Duty cycle: rises smoothly with ERPM
- Fault: 0 throughout

### Pattern: duck-duck-duck (bad)
- ERPM: stays low (300–1000) and oscillates, doesn't climb
- Motor current: spikes up and down, alternating high/low
- Duty cycle: also oscillates
- This is the "openloop forcing the wrong angle" signature
- Fault: 0 (it's not actually faulting, just stuttering)

### Pattern: undervoltage trip
- Voltage: dips below `l_battery_cut_end`
- Fault code: jumps to 6
- Motor current then drops to 0

### Pattern: motor commutating backward
- ERPM: goes negative then positive then negative
- Motor current: reverses direction repeatedly
- This is a serious observer failure — re-run motor detection

### What "good" looks like for Test B (restart-while-spinning)
- During coast: ERPM gracefully decreases, Motor Current ≈ 0 (no torque)
- At re-engage: ERPM should JUMP to track the new trigger, NOT drop to 0 first and try openloop
- Motor current spikes briefly then settles
- No oscillation in ERPM

---

## 6. APPLYING THE PROCESS TO YOUR CURRENT STATE

You're at:
- Boost = 3 ✅ (Step 1 done, motor breaks free reasonably)
- Open loop ERPM = 1500 (default — needs to come down)
- Sensorless ERPM Start = 2500, Sensorless ERPM = 3400 (defaults — Step 3 territory)

**Do this in order:**

### Iteration A (next session)
1. Save current XML as rollback: `motor_2026-05-03_boost3_baseline.xml`
2. Run Tests A, B, C with logging on. Save logs as `iter_A_<test>_2026-05-03.csv`.
3. Change ONE parameter: `Open loop ERPM` from 1500 → 1000. Write to VESC, save to flash.
4. Run Tests A, B, C again with logging on. Save as `iter_B_<test>_2026-05-03.csv`.
5. Compare:
   - Test B should show much less duck-duck stutter (the symptom should be largely gone)
   - Test A should still be smooth (didn't break Step 1)
   - Test C should be smooth or smoother
6. **If duck-duck is gone:** keep the change, move to Iteration B if you want even tighter low-speed feel.
7. **If duck-duck got better but not gone:** lower `Open loop ERPM` to 800. Repeat tests.
8. **If anything got worse:** load rollback XML, write to VESC, save. Try a different change.

### Iteration B (only if A's duck-duck is fully fixed and you want sharper low-speed control)
1. Save XML rollback first.
2. Lower `Sensorless ERPM Start` from 2500 → 1500.
3. Lower `Sensorless ERPM` from 3400 → 2500.
4. Run Tests A, B, C. Compare logs.
5. If smoother: keep. If anything stutters during transition: roll back and stop here.

### Iteration C (advanced — only if you've optimized A and B and still want more)
- Try `Open loop lock time` from 0 → 0.05 s. Gives the rotor a moment to align with the openloop start angle. May help cold start at lower boost. May make hot-restart slightly worse (so test B carefully).
- Try `foc_f_zv` (PWM frequency) from 30000 → 25000 in FOC → General. Quieter operation, slightly less precise control.

**Stop iterating when good enough.** Perfect is the enemy of done. If A fixes the duck-duck and the motor feels good, ride it. Don't tune for tuning's sake.

---

## 7. ABOUT PHASE FILTER (intentionally NOT recommending — read this)

You may see suggestions on the VESC forum to enable **Phase Filter** (`foc_phase_filter_enable`) for spinning-restart issues. **DO NOT enable this on the FlipSky FSESC 75200 V2 without first verifying the hardware supports it.**

Phase filter is a SOFTWARE feature that requires HARDWARE — specifically, voltage-divider capacitors of at least 220 nF on the phase voltage sense lines. The FlipSky 75200 V2 may or may not have these (the 75100 and 75200 hardware threads on vesc-project.com discuss this in detail and the answer depends on board revision).

If you enable phase filter on hardware that doesn't support it, behavior includes:
- Negative current readings during detection
- Wrong R/L/flux measurement on motor detection
- Worse low-speed performance, not better

**The fix for your current symptom (lower `Open loop ERPM`) does not depend on phase filter and is safe regardless of hardware.** Save phase filter for last-resort tuning if Step 2 + Step 3 don't fully solve it. If you want to try it, first ASK on the FlipSky forum thread or VESC project forum whether the 75200 V2 has the right cap value. Don't guess.

---

## 8. UPLOADING LOGS FOR ANALYSIS

When you have a log file you want analyzed:

1. **Save the CSV** to your phone's chosen log folder.
2. **Transfer to your computer**, easiest via Google Drive: drop the CSV into your existing project folder structure or a new subfolder, e.g.:
   - `G:\My Drive\Claude AI files\Claude CODE\Projects\Vesc and other related files\logs\`
3. **Name it descriptively:**
   - `iterA_boost3_openRPM1500_testB_restart_2026-05-03.csv`
   - Format: `iter<X>_<changed-params>_test<Y>_<symptom>_<date>.csv`
4. **Tell me in chat:** "I uploaded `iterA_boost3_openRPM1500_testB_restart_2026-05-03.csv` for analysis. The symptom was duck-duck-duck on re-engage at 1.0s coast time."
5. I'll read it, look for the patterns described in §5, and recommend the next single change.

### What I need to know with each upload
- Which iteration (A, B, C…)
- Which test (cold start / restart-while-spinning / slow cruise)
- What you changed since the last log
- What the audible/visual symptom was
- What the trigger sequence was (so I can correlate with the time axis)

The more structured your reports, the faster the diagnosis. If you just upload `vesc_log_2026-05-03_153012.csv` with no notes, I'll have to reconstruct what happened from the data, which is slower and error-prone.

---

## 9. SOURCES

- [Flipsky FSESC75100 & FSESC75200 Phase Filtering — VESC Project](https://vesc-project.com/node/3529)
- [Sometimes startup problems in FOC-Mode — VESC Project](https://vesc-project.com/node/85)
- [VESC 6.0 Data Logging? — VESC Project](https://vesc-project.com/node/77)
- [VESC tool for Android — VESC Project](https://vesc-project.com/node/2772)
- [Data logging — VESC Project](https://vesc-project.com/node/932)
- [Vesc tools log how to — FOIL.zone](https://foil.zone/t/vesc-tools-log-how-to/18244)
- [VESC Tuning tips — Endless Sphere](https://endless-sphere.com/sphere/threads/vesc-tuning-tips.125527/)
- Companion docs in this folder: `VESC_SMOOTH_START_QUICK_REFERENCE.md`, `VESC_FOC_TUNING_NOTES.md`, `THROTTLE_PIPELINE_ANALYSIS.md`

---

End of process doc.
