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

- `G:\My Drive\Claude AI files\Claude CODE\Projects\BREmote-V3-monterman-main\docs\VESC_FOC_TUNING_NOTES.md` — full deep dive on FOC tuning, FOC-vs-BLDC, and parameter analogies
- `G:\My Drive\Claude AI files\Claude CODE\Projects\BREmote-V3-monterman-main\docs\THROTTLE_PIPELINE_ANALYSIS.md` — BREmote-side throttle pipeline analysis and recommendations
- [VESC Tool parameters_mcconf.xml — confirms parameter exists in FW 6.06](https://github.com/vedderb/vesc_tool/blob/master/res/config/6.06/parameters_mcconf.xml)
- [VESC Project — Sometimes startup problems in FOC-Mode](https://vesc-project.com/node/85)
- [VESC Project — FOC Startup issues](https://vesc-project.com/node/3671)
- [VESC Project — foc openloop not starting motor on higher erpm](https://www.vesc-project.com/node/589)

---

End of quick reference.
