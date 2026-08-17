# VESC Smooth-Start Quick Reference (Standalone)

For: Andres, BREmote V2.5-EVO + FlipSky FSESC 75200 V2 + 6384 outrunner + sensorless FOC underwater
App: VESC Tool 6.06 (Android, Google Play)
Date created: 2026-05-03
Companion doc (deep dive): `VESC_FOC_TUNING_NOTES.md` in this folder

---

## ONE-LINE SUMMARY

**Set "Open loop current boost" to 3, write to VESC, save config, bench test. That's the whole change for now.**

---

## 1. WHERE TO FIND THE SETTINGS (no Expert mode needed in 6.06)

In VESC Tool 6.06 Android, the openloop parameters are in plain sight — no toggles or advanced mode required. Path:

> **Motor Settings → FOC → Sensorless tab**

The Sensorless tab in your app shows these rows (verified 2026-05-05 by fingerprint testing — values changed in app, XML exported, mapping confirmed):

| UI label (VESC Tool 6.06 Android) | XML parameter name |
|---|---|
| Open loop ERPM | `foc_openloop_rpm` ✓ |
| Open loop ERPM at minimum current | `foc_openloop_rpm_low` |
| Open loop hysteresis | `foc_sl_openloop_hyst` |
| Open loop lock time | `foc_sl_openloop_time_lock` ✓ |
| Open loop ramp time | `foc_sl_openloop_time_ramp` |
| Open loop time | `foc_sl_openloop_time` |
| **Open loop current boost** ⭐ | `foc_sl_openloop_boost_q` ✓ |
| Open loop current max | `foc_sl_openloop_max_q` ✓ |
| Start current decrease | `foc_start_curr_dec` |
| Start current decrease ERPM | `foc_start_curr_dec_rpm` ✓ |
| Saturation compensation mode | `foc_sat_comp_mode` |
| Saturation compensation factor | `foc_sat_comp` |
| Temperature compensation | `foc_temp_comp` |
| Temperature compensation base temp | `foc_temp_comp_base_temp` |

The starred row is the one we're tuning today.

### ⚠️ UI gotcha: "Sensorless ERPM" appears in TWO places, mapping to TWO DIFFERENT XML fields

This is a VESC Tool UI labeling quirk. Watch the section/tab to know which one you're touching:

| Where you see "Sensorless ERPM" | XML parameter | Meaning |
|---|---|---|
| **FOC → General tab → Encoder section** (also visible in standalone Encoder tab) | `foc_sl_erpm` | The threshold above which sensorless mode is **fully active** (end of blend zone). Default 2500-3400. |
| **FOC → Hall Sensors tab → first row** | `foc_sl_erpm_start` | The threshold at which sensorless transition **begins** (start of blend zone). Default 2250-2500. |

**Same label, different fields. Same value displayed in two different rows under the same name. Look at the SECTION HEADER above the row to know which one you're editing.**

Mnemonic: the row in **General/Encoder** = "fully sensorless" = `foc_sl_erpm`. The row in **Hall Sensors** = "where sensorless starts" = `foc_sl_erpm_start`.

(Note: even though you're running sensorless mode, the Hall Sensors tab still has rows that affect sensorless operation — including this one. Don't skip the tab just because you don't have hall wires connected.)

---

## 2. RECOMMENDED VALUES — STEP 1 (DO THIS NOW)

| UI label | Your current value | Set to | Why |
|---|---|---|---|
| **Open loop current boost** | 0 | **3** ⭐ | Adds 3 A of Q-axis (torque) current during openloop startup. Combined with your trigger current, gives ~13–18 A motor current at 10–15% trigger — enough to break a typical underwater prop free of static water load. |
| Open loop ERPM | 1500 | **leave at 1500** | Step 2 — only lower this AFTER Step 1 verified working |
| Open loop ERPM at minimum current | 0 | **leave at 0** | Default, fine |
| Open loop hysteresis | 0.1 | **leave at 0.1** | Default, fine |
| Open loop lock time | 0 | **leave at 0** | Default, fine |
| Open loop ramp time | 0.1 s | **leave at 0.1** | Default, fine |
| Open loop time | 0.05 s | **leave at 0.05** | Default, fine |
| Open loop current max | -1 | **leave at -1** | -1 = no cap; let `Start current` decay handle it |
| Start current | 1.0 | **leave at 1.0** | Default, fine |
| Saturation comp mode / factor | 0 / 0 | **leave** | Don't enable — only useful if motor saturates at high duty |
| Temperature compensation | 0 | **leave at 0** | Don't enable — adds complexity, marginal benefit |

**Only one parameter changes: Open loop current boost from 0 to 3.**

---

## 3. STEP-BY-STEP PROCEDURE

1. Connect the FSESC over Bluetooth in VESC Tool 6.06.
2. Tap **"Read Configuration"** (up-arrow / read icon) to pull the current motor config from the VESC into the app.
3. Navigate: **Motor Settings → FOC → Sensorless** tab.
4. Find **"Open loop current boost"** — currently shows `0`.
5. Tap the field, enter `3`, dismiss keyboard.
6. Tap **"Write Configuration"** (down-arrow / write icon) to push the change to the VESC.
7. Tap **"Save Configuration to Memory"** / **"Store to flash"** (sometimes a separate button or in the menu) so the change survives a power cycle.
8. **Optional but recommended:** save an XML snapshot of the new config to your phone — this is your "rollback point" if anything goes wrong. Menu → "Save XML" → name it something like `motor_2026-05-03_boost3.xml`.

---

## 4. BENCH TEST CHECKLIST (do this BEFORE getting wet)

In order, test these scenarios. Stop and back off the boost if any of them fail.

### Test A — Motor in air, no prop
- Power up the FSESC and BREmote, pair as normal.
- Squeeze the trigger slowly from 0%.
- **Expected:** motor starts spinning smoothly at ~5–10% trigger. No stutter. No audible chirp at very low RPM. Boost current is high enough to spin the unloaded shaft.
- **Pass:** motor spins. Move to Test B.
- **Fail (motor stutters or doesn't spin):** boost is too low for some reason — try `5` instead of `3`.
- **Fail (motor jumps suddenly at trigger zero):** boost is leaking with zero command — try `2` instead of `3`. Verify VESC ignores boost when commanded current is 0 (newer firmware does this; older may not).

### Test B — Motor with prop, in air
- Reattach the prop.
- Squeeze trigger slowly from 0%.
- **Expected:** motor starts at ~10% trigger. No vibration. Smooth ramp.
- **Pass:** move to Test C.
- **Fail (still hesitates):** raise boost to `5`, retest.

### Test C — Motor with prop, in tub of water (or hose-flooded)
- Submerge the propeller in a container deep enough to fully cover the blade.
- Squeeze trigger slowly from 0%.
- **Expected:** motor breaks free at 15–20% trigger. Prop produces visible water flow at 25–30%.
- **Pass:** you're ready for water test.
- **Fail (need 30%+ to break free):** raise boost to `5`. If still failing at `5`, raise to `8`. Hard cap is 10 A. If still failing at 8 A, the issue isn't boost-related — there's something else wrong (motor params drifted, prop fouled, mechanical bind, etc.).

### Test D — Heat check after Test C
- After 30 seconds of running underwater at low throttle (e.g., 20%), power down.
- Touch the motor housing (or read motor temp from VESC realtime data, `m_temp_motor`).
- **Expected:** motor warm but not hot (under 50 °C / 120 °F).
- **Fail (very hot):** boost is too high and the motor is dissipating real heat at low duty — drop boost to 2 or 3 max.

---

## 5. DECISION TREE — WHAT TO DO AFTER STEP 1

```
After Step 1 (boost = 3):
│
├─ Bench tests A/B/C all pass with smooth start at 10–20% trigger?
│   → Done. Ride it. Don't change anything else right now.
│
├─ Bench tests fail (need more boost)?
│   → Raise boost to 5. Retest A/B/C/D.
│   → If still failing, raise to 8. Retest A/B/C/D.
│   → If failing at 8, STOP. Diagnose elsewhere (motor detection, prop, mechanical).
│
├─ Bench tests pass but low-speed control feels "wooden" / not smooth above 1500 ERPM?
│   → That's the cue for Step 2: lower "Open loop ERPM" from 1500 to 1000.
│   → Bench test again. If smoother, keep it.
│   → If motor stutters during transition, raise back toward 1200 in steps of 100.
│
└─ Step 2 also good — want even tighter low-speed control?
    → Step 3: lower "Sensorless ERPM Start" from 2500 to 1500 AND
              "Sensorless ERPM" from 3400 to 2500 (these are below the rows in this table — find them in the same Sensorless tab).
    → Bench test. Easy to back out if it goes wrong.
```

**One change at a time.** If you change three parameters and the motor behaves differently, you cannot tell which change caused what.

---

## 6. ROLLBACK PROCEDURE (if anything goes wrong)

If a change makes the motor worse, back out:

1. In VESC Tool: Motor Settings → menu → **"Load XML"** → pick the rollback file you saved before the change (Step 3.8).
2. Tap **"Write Configuration"**.
3. Tap **"Save Configuration to Memory"**.
4. Power-cycle the FSESC and confirm the old behavior is restored.

If you didn't save a rollback XML and changes went bad: in VESC Tool, find the **"Default"** button (sometimes labeled "Restore Defaults" or "Load Default Config"). This resets motor config to factory blank — then you'll need to re-run Motor Detection + PPM Mapping from scratch. **Save a rollback XML this time.**

---

## 7. WHAT THIS DOES NOT FIX

- **Underwater motor that completely refuses to spin** even with boost = 8: probably a stuck rotor, fouled prop, demagnetized rotor, or wiring issue. Not a tuning problem.
- **Trigger feel during normal riding (above 30% throttle):** boost only acts during openloop (low ERPM). Once you're cruising, boost has zero effect. Trigger feel during normal riding is controlled by `thr_expo` on BREmote and the VESC current control loop.
- **The "I squeeze 100% trigger and the motor instantly comes on hard" feel:** that's a separate user-experience issue. The fix is BREmote auto soft-start (Option A) — see `THROTTLE_PIPELINE_ANALYSIS.md` §2.5. Boost helps the motor START, but doesn't moderate how fast power rises after that.

---

## 8. QUICK PARAMETER REFERENCE — what each Sensorless tab field actually does

For when you're staring at these in the app and wondering what they're for:

- **Open loop ERPM** — speed threshold above which VESC trusts back-EMF observer. Below it: forced rotation (blind). Above: real position feedback. Lower = better low-speed feel, higher = more forgiving startup.
- **Open loop ERPM at minimum current** — lower edge of the openloop ERPM ramp. Usually 0. Don't touch.
- **Open loop hysteresis** — buffer zone around the openloop/closed-loop transition to prevent flicker. 0.1 is fine.
- **Open loop lock time** — how long to hold rotor at zero before starting rotation. 0 = no hold. Some motors that need to "pre-align" want 0.05. Most don't.
- **Open loop ramp time** — how long to ramp from zero to openloop_rpm. 0.1s is standard.
- **Open loop time** — how long to dwell at openloop_rpm before transitioning to closed-loop. 0.05s standard.
- **Open loop current boost** ⭐ — extra Q-axis (torque) current added during openloop. **This is your knob.** 0 = none, 3 = mild, 5 = stronger, 8+ = aggressive.
- **Open loop current max** — cap on total openloop current (boost + commanded). -1 = no cap, let decay handle it.
- **Start current** — decay rate of startup current as ERPM rises. 1.0 is standard.
- **Saturation compensation mode/factor** — for motors that saturate the iron core at high duty. Leave off unless you know you need it.
- **Temperature compensation** — corrects for motor R drifting with temperature. Marginal benefit; off by default.

---

## 9. SOURCES

- `docs/VESC_FOC_TUNING_NOTES.md` — full deep dive on FOC tuning, FOC-vs-BLDC, and parameter analogies
- `docs/THROTTLE_PIPELINE_ANALYSIS.md` — BREmote-side throttle pipeline analysis and recommendations
- [VESC Tool parameters_mcconf.xml — confirms parameter exists in FW 6.06](https://github.com/vedderb/vesc_tool/blob/master/res/config/6.06/parameters_mcconf.xml)
- [VESC Project — Sometimes startup problems in FOC-Mode](https://vesc-project.com/node/85)
- [VESC Project — FOC Startup issues](https://vesc-project.com/node/3671)
- [VESC Project — foc openloop not starting motor on higher erpm](https://www.vesc-project.com/node/589)

---

## 10. VESC2 / WHITE MOTOR — the second-motor tuning (added 2026-07-21)

> **✅ CONFIRMED ON BENCH (2026-07-21) — trust these final values over the narrative below:**
> - VESC2 starts smooth at **Open loop current boost = 4**, **Motor Current Max = 65 A**, **phase filter OFF**. Final file: `docs/vesc_configs/vesc2_MOTORconf_v6.xml`.
> - **The 0.00245 flux in the narrative below was a BAD detection.** A clean re-detection gave **flux 0.004984 — nearly identical to VESC1 (0.004922)**. The white motor is NOT half-flux; it's basically the same motor class as VESC1. So it needed boost **4** (VESC1 uses 3), not the "5-8" the bad-detection math implied.
> - **⚠️ PHASE FILTER GOTCHA (this FSESC HW):** `foc_phase_filter_enable = 1` throws a **"config not compatible with your VESC HW" warning on load** — keep it **OFF (0)**. A FOC detection can flip it back on; re-check after every detection.
> - **VESC identity:** VESC1 = controller_id **1**, VESC2 = **2** (unique CAN IDs — a duplicate ID conflicts the bus). Tell them apart by flux linkage or ID, NOT the CAN "local/2" label.
> - **A "dead" VESC1 turned out to be a LOOSE PWM CABLE, not config** — reseating the RX→VESC signal cables fixed it. Check wiring first (this buggy has a history of the PWM signal wire failing).



VESC1 (black motor) was tuned smooth long ago (§2). VESC2 later got a **different, second motor (the white one)** that would **not start when feathered** — it needed a blunt >10 % shove or it stalled and stayed off.

### Root cause (found by diffing the two exported motor configs)
- The startup tuning was **not** missing by accident — running **FOC Motor Detection on VESC2 had reset `Open loop current boost` back to 0.** Detection **always wipes** the open-loop/sensorless tuning back to defaults.
- The two motors are genuinely different, and **flux linkage is the fingerprint:** VESC1 = `0.004922`, VESC2 (white) = **`0.00245` (half).** Half the flux ⇒ ~half the torque per amp ⇒ it needs **more** startup boost than VESC1, not the same.
- Bonus find: VESC2 still had **FOC-wizard 3S battery defaults** (cutoff 10 V / 8 V) on a **10S pack** → its low-voltage protection could **never fire**.

### The v5 fix — baked into `docs/vesc_configs/vesc2_MOTORconf_v5.xml`
**Startup** (Motor Settings → FOC → Sensorless tab):
| App field | VESC1 | VESC2 v5 | note |
|---|---|---|---|
| **Open loop current boost** | 3 | **5** | doubled for the half-flux motor; bump 5→6→8 if it still stalls (cap 10 A) |
| Open loop lock time | 0.05 | **0.05** | pre-aligns the rotor — helps break-free |
| Open loop ramp time | 0.2 | **0.2** | gentler ramp into open-loop |
| Open loop ERPM | 1000 | **1500 (LEAVE)** | ⚠️ do NOT copy VESC1's 1000 — low-flux = weak back-EMF, needs the higher observer threshold or it gets *rougher* |

**Battery / safety** (General → Voltage / Current / Additional Info):
| App field | was (3S default) | v5 |
|---|---|---|
| Battery Voltage Cutoff Start / End | 10 / 8 | **32 / 29** |
| Min input voltage | 12 | **23** |
| Battery Cells (Series) / Ah | 3 / 6 | **10 / 30** |
| Battery Current Max | 250 | **100** |
| Absolute Maximum Current | 420 | **250** |

Motor detection (R `0.015` / L `7 µH` / λ `0.00245`) left **untouched** — it's the real white-motor fingerprint.
**Still open:** `Motor Current Max` left at **60** (VESC1 is 95) pending the white motor's rating.

### ⭐ TWO GOTCHAS — remember these
1. **Re-running FOC detection wipes `boost_q` AND the battery settings back to defaults.** After ANY detection, re-apply the v5 values — or just re-load `vesc2_MOTORconf_v5.xml`.
2. **Tell the two VESCs/motors apart by flux linkage, not the CAN label** (which flips between "local" and "2" and is confusing): VESC1 = `0.004922`, VESC2 = `0.00245`.

### Reference configs saved (2026-07-21)
- `docs/vesc_configs/vesc1_MOTORconf_v5.xml` — VESC1 known-good (unchanged).
- `docs/vesc_configs/vesc2_MOTORconf_v5.xml` — VESC2 with the smooth-start + 10S-battery fix.

---

## ⭐ VESC1 + VESC2 — FINAL PPM INPUT MAPPING (matched twins, 2026-07-23)

Calibrated to the RX PPM pulse **after** bypassing the broken inter-enclosure Ethernet cable (dead 5V/ground had starved VESC2's opto output → floating 4–13 ms garbage; see session-log 2026-07-22/23). Signal clean now; captured via VESC Tool PPM Input setup. **Both VESCs set to the SAME mapping** so the twin motors respond identically.

| Field | VESC1 | VESC2 |
|---|---|---|
| Pulselength **start** (min) | **1.251 ms** | **1.240 ms** ⭐ final |
| Pulselength **end** (max) | ~2.140 ms | **2.140 ms** |
| Pulselength **center** | 1.700 ms | 1.700 ms |
| **Deadband** | 0 | 0 |

- **VESC2 start fine-tuned to 1.240 (final):** iterated 1.236 → 1.249 → 1.240 (1.236 crept at rest; 1.390 caused false starts). **1.240 gives a fast low-input start without creep.** VESC1 start ≈ 1.251. If creep returns, nudge VESC2 start up a touch.
- VESC1 center was 1.482 (irrelevant in no-reverse mode) → set to 1.700 to match. Deadband 0 both. Start capture matches the measured clean RX pulse (~1.247 / 1.70 / 2.155 ms).

### Twin-match result (free-spin, no load) — MATCHED ✅
| | Start threshold | ~3 % throttle | Full free-spin |
|---|---|---|---|
| VESC1 | 1–2 % | ~10–11k ERPM | ~43,000 ERPM |
| VESC2 | 1–2 % | ~10–11k (up to ~16k on a harder feather) | ~42,000 ERPM |

**~1,000 ERPM apart at full (~2 %) → matched; both feather in at 1–2 %, spin together.** ERPM shown — actual prop RPM = ERPM ÷ pole-pairs.

### VESC2 BAKED CONFIG — full values (backup if the XML is ever lost)
Primary file: `Downloads\vesc backups\v3 new app\vesc2_MOTORconf_v9_currentloop!.xml` (mirror to `docs/vesc_configs/` on next export). To rebuild by hand — **Motor Settings → FOC** (then **do NOT re-run detection**, it overwrites these):

| Param (XML name) | Value | Notes |
|---|---|---|
| `foc_motor_r` | 0.0198 Ω | fresh detect |
| `foc_motor_l` | 1.855e-05 H | fresh detect |
| `foc_motor_flux_linkage` | 0.00498 | fresh detect (VESC2 fingerprint) |
| `foc_observer_gain` | 4.032e+07 | |
| `foc_current_kp` | 0.0371 | auto (2× VESC1 bw; halve to ~0.0186 only if start rough — it isn't) |
| `foc_current_ki` | 39.56 | auto |
| `foc_openloop_rpm` | 1000 | smooth-start |
| `foc_sl_openloop_time_lock` | 0.05 | |
| `foc_sl_openloop_time_ramp` | 0.2 | |
| `foc_sl_openloop_boost_q` | 5 | smooth-start boost |
| `cc_startup_boost_duty` | 0.03 | |
| `foc_phase_filter_enable` | **0 (OFF)** | ⚠️ HW-incompat on this FSESC — keep OFF |
| `m_invert_direction` | 1 | |
| `l_current_max` (Motor Current Max) | 95 A | matched to VESC1 |
| App → `controller_id` | 2 | CAN id (App config, not motor) |

**PPM input** (both VESCs): VESC2 start **1.240** / VESC1 start 1.251 · center 1.700 · end 2.140 · deadband 0 (VESC2 start fine-tuned to 1.240; set just above rest to avoid creep).

> **⚠️ LIVE STARTUP — settled 2026-07-23 (phone app; supersedes v9 XML Sensorless values; NOT yet re-exported — read back / export next PC session):**
> **The real fix was a fresh FOC DETECTION, not tuning.** Heavy startup tuning (ramp 0.4, boost up to 8.5 A, ERPM 1400) still left VESC2 glitchy while VESC1 was flawless on near-defaults → the motor *model* was stale. After a clean re-detection, boost dropped **8.5 → 6.5 → 4.5 A** and it runs better (a correct model needs far less brute boost).
> **Settled VESC2:** boost **4.5 A** · openloop ramp **0.4** (long — "start as slow as possible") · lock **0.1** · hysteresis **0.15** · openloop ERPM **1400** · openloop-ERPM-at-min-current **0%** · PPM start **1.240**. Fresh R/L/λ from re-detection (exact values NOT captured — read back / export next PC session; should be ≈ VESC1: R~0.02 / λ~0.0049 / L~1.8µH).
> **VESC1:** boost **3 A**, openloop ERPM raised to **1400** (the only change — still flawless). The **3 A (VESC1) vs 4.5 A (VESC2)** gap is real coil/magnet/cogging tolerance between the twins — matched *behavior* (both start mostly good), not identical numbers.
> **⚠️ Boost is a WINDOW, not "more is better":** too LOW → won't break away; too HIGH → the excess openloop current **locks/cogs** the rotor instead of spinning it. VESC2's window sits at **4.5 A** (8.5 A was into the locking zone).
> ⚠️ **After ANY detection, verify: phase filter still OFF + direction (`m_invert_direction`) correct** — detection can flip both.
> **Lesson:** when one twin is perfect on defaults and the other needs extreme startup tuning to limp along, the fix is **re-detection / hardware**, not more openloop knobs.

> ⚠️ Re-running PPM Input calibration or FOC detection overwrites this. If a VESC reads garbage again, check the inter-enclosure cable (5V/GND continuity) FIRST — that was the root cause, not the mapping.

---

## 🔴 PPM2 dead / mapping bar flat at zero — read this before touching anything else

**This has now caused two lost sessions (2026-07-22 and 2026-07-27). It is always the 5 V, and it is never the firmware.**

### The mechanism, proven from `Rx_V2-2.sch`

The RX's PPM outputs are **optocouplers** (`IC5` = channel 0, `IC3` = channel 1). An opto's output transistor can only pull the line **down** — it needs a pull-up to produce a high. That pull-up is the 5 V on the corresponding servo connector:

| Net | Goes to | Also feeds |
|---|---|---|
| `ESC0_5V` | `JP1.2` · `IC5.C` | **`D8.VIN` → the board's +5 V rail** |
| `ESC1_5V` | `JP4.2` · `IC3.C` | **nothing else** |

**No 5 V on `ESC1_5V` → `IC3` has no pull-up → the PPM2 line can never go high → VESC 2 sees nothing.** Not a weak signal. **Zero.** The mapping bar sits flat while you sweep the throttle.

### ⭐ Why it is invisible, and why it keeps recurring

`ESC0_5V` also powers the whole RX through `D8`, so **losing channel 0's 5 V stops the board booting — impossible to miss.** `ESC1_5V` powers *only* the opto pull-up, so losing it produces **no symptom anywhere except a dead second motor**. Nothing in the firmware reports it. Nothing on the display shows it. The RX looks perfectly healthy.

### Connector pinout — from the netlist, not the silkscreen

```
JP4.1 = ESC1_SIG → R8        JP4.2 = ESC1_5V → IC3.C        JP4.3 = ESC1_GND
JP1.1 = ESC0_SIG → R7        JP1.2 = ESC0_5V                JP1.3 = GND (board)
```

**Pin 1 is SIGNAL. Pin 3 is GND.** An inverted signal/ground cable has already caused a false diagnosis on this build once — confirm pin 1 by ringing it out to `R8` (channel 1) or `R7` (channel 0).

### Diagnostic order

1. **Continuity, 5 V conductor, VESC 2 → `JP4.2`.** This is the failure, twice running.
2. **~5 V present at `JP4.2`** with the system powered.
3. Probe **VESC 2's own servo pin 2**: reads ~5 V → the cable is broken, repair it. Reads 0 V → VESC 2's BEC is off, feed `JP3.2` (BEC1) instead, which enters through ideal-diode `D7`.
4. Only then look at mapping or firmware.

### Ground

`ESC1_GND` is a separate net from board GND (the opto is nominally isolated), **but VESC 1 and VESC 2 already share ground through the CAN connector and battery negative** — heavy gauge, low impedance. Common ground is correct in this topology and a dedicated ground wire to `JP4.3` is optional. Confirm continuity rather than assuming.

### ❌ Do not bridge PPM1 5 V → PPM2 5 V

That ties VESC 1's BEC directly to VESC 2's with no diode between them — two regulators in parallel. **It is what failed both times.** `JP3` (BEC1) exists precisely so an external 5 V can be injected properly, through `D7`.

**Fixed 2026-07-28** by running VESC 2's own signal and 5 V into `JP4` pins 1 and 2. Both VESCs then showed stable PPM with near-identical mapping.

---

End of quick reference.
