# VESC FOC Tuning Notes — Smooth Start for E-Foil Underwater Operation

Date: 2026-05-03
Scope: Analysis of Andres's existing VESC motor config (`new FOC 6384 vesc1`), recommendations to fix the "needs ~35–40% throttle to start spinning underwater" symptom, and a brief on FOC vs BLDC for this application. Read-only analysis. No firmware or VESC config files were modified.

---

## EXECUTIVE SUMMARY

1. **Your symptom is a classic FOC sensorless startup-torque problem, not a BREmote bug.** The motor doesn't break free from underwater static load until throttle gets high enough to push real current through. The fix is in VESC, not in the remote.

2. **Three VESC parameters do 80% of the work.** `foc_sl_openloop_boost_q` (currently `0`), `foc_openloop_rpm` (currently `1500`), and `cc_startup_boost_duty` (currently `0.01`). Tune these and the motor will spin at 10–15% trigger reliably.

3. **FOC is the right choice for your use case.** Confirmed via web research and your config: quieter, smoother at low speed, more torque at startup *when properly tuned*. BLDC is louder, has audible cogging, and has worse low-speed control — bad fit for underwater silent operation.

4. **BREmote can help slightly via `thr_expo`** but only as a curve-shaping tool, not a torque-boost. With `thr_expo = 50` (true linear, your planned change) the curve doesn't fight VESC's startup tuning. **Don't use BREmote as a startup booster** — it would either be a Section 9 safety violation (autonomous throttle add) or duplicate work the VESC should do.

5. **Hardware path forward (future, not now):** add hall sensors or an encoder to your motor. Sensored FOC has full torque from zero RPM with known rotor position — completely eliminates the startup hesitation. Sensorless FOC is always a guess at low ERPM. This is the ultimate fix but requires motor mods.

---

## 1. FOC vs BLDC — confirmed for your use case

### Quick verdict for e-foil
**FOC.** Sources from VESC project forum and motor-control documentation back you up on every claim:

- **Quieter** — confirmed. FOC produces sinusoidal phase currents; BLDC produces trapezoidal commutation pulses with audible switching. Underwater the difference is even more pronounced because hull/water transmits high-frequency switching noise.
- **Smoother at low speed** — confirmed. FOC was originally designed for full-torque operation across the entire speed range, including zero. BLDC has ripple torque from its 6-step commutation that becomes audible/feel-able cogging at low RPM.
- **More precise** — confirmed. FOC uses continuous Park/Clarke transforms to control D and Q axis currents independently. BLDC just switches phases on/off based on rotor position estimate.
- **Harder to configure** — confirmed. FOC requires accurate motor parameter measurement (`R`, `L`, `flux linkage`) and observer tuning. BLDC mostly works with default parameters once polarity is right.
- **More power/torque** — partially true. FOC delivers more *usable* torque in low-speed regions. At high RPM the difference shrinks. For e-foil where you spend a lot of time at moderate RPM, FOC has a clear advantage.

### Caveat — FOC sensorless at very low speed is genuinely tricky
FOC sensorless uses back-EMF to estimate rotor position. Below a critical ERPM there's not enough back-EMF to estimate accurately, so VESC runs in **open-loop** mode: it just forces a rotating magnetic field at a programmed rate and hopes the rotor follows. If the load is too high (water on a static prop), the rotor doesn't follow, the openloop current does no useful work, and you feel "the motor doesn't start until I push the trigger."

This is exactly your symptom. It's not a defect — it's how sensorless FOC works. The fix is to either:
- Give openloop more current/torque to break free of the static load, OR
- Reduce the openloop window (lower the threshold at which sensorless takes over).

Both knobs are in VESC config. See §3.

---

## 2. Your current VESC motor config — what I found

Source file: `G:\My Drive\Claude AI files\Claude CODE\Projects\Vesc and other related files\new FOC 6384 vesc1` (XML format, parsed line-by-line).

### Power and current limits — these look reasonable
| Parameter | Your value | Comment |
|---|---|---|
| `motor_type` | 2 (FOC) | ✅ Correct |
| `foc_sensor_mode` | 0 (Sensorless) | ⚠ See §4 about adding sensors |
| `l_current_max` | 95 A | Generous for a 6384, fine |
| `l_current_min` | -60 A | Brake/regen current — fine |
| `l_in_current_max` | 100 A | Battery side, fine |
| `l_abs_current_max` | 250 A | Fault limit, leave alone |
| `l_min_duty` | 0.005 | Very low (0.5%) — fine, don't raise |
| `l_max_duty` | 0.96 | Standard, leave alone |
| `l_min_vin` | 23 V | 6S min — confirm against your battery |
| `l_max_vin` | 72 V | 16S max — confirm against your battery |
| `motor_poles` | 14 (=7 pole pairs) | ✅ Correct for 6384 outrunner |

### Detected motor parameters — auto-measured, look good
| Parameter | Your value | Comment |
|---|---|---|
| `foc_motor_r` | 0.0233 Ω | Reasonable for 6384 |
| `foc_motor_l` | 18.4 µH | Reasonable for 6384 |
| `foc_motor_flux_linkage` | 0.00493 Vs/rad | Reasonable for 6384 |
| `foc_observer_gain` | 41.2 M | Auto-calculated from R/L/λ — leave alone |

### **The startup-tuning parameters — these are your knobs**
| Parameter | Your value | Recommended | Why |
|---|---|---|---|
| **`foc_sl_openloop_boost_q`** | **0** A | **3–8 A** ⭐ | Extra Q-axis (torque-producing) current during open-loop startup. Currently zero, which is why the motor won't move under water load. **This is your #1 lever.** |
| **`foc_openloop_rpm`** | **1500** | **800–1200** | Below this ERPM the motor is in open-loop mode (forced rotation, no feedback). Lowering it means VESC switches to closed-loop sensorless sooner — which has better torque control. |
| `foc_openloop_rpm_low` | 0 | 0 (leave) | Lower threshold for ramp-down. Keep at 0. |
| `foc_sl_openloop_hyst` | 0.1 | 0.1 (leave) | Hysteresis around the openloop transition. Default OK. |
| `foc_sl_openloop_time_lock` | 0 | 0–0.05 s | Time to "lock" rotor at zero before starting rotation. 0.05s can help motors that need to align. Try 0 first. |
| `foc_sl_openloop_time_ramp` | 0.1 s | 0.1 s (leave) | Time to ramp from zero to openloop_rpm. Leave alone. |
| `foc_sl_openloop_time` | 0.05 s | 0.05 s (leave) | Time at openloop_rpm before transitioning. Leave alone. |
| `foc_sl_erpm_start` | 2500 | 1500–2000 | ERPM at which sensorless transition begins. |
| `foc_sl_erpm` | 3400 | 2500–3000 | ERPM at which sensorless is fully active. |
| `foc_start_curr_dec` | 1 | 1 (leave) | Current decay factor at startup. Default OK. |
| `foc_start_curr_dec_rpm` | 2500 | 2500 (leave) | RPM where current decay completes. |
| `cc_startup_boost_duty` | 0.01 | 0.01–0.05 | Min duty cycle at startup. Could raise slightly. Lower priority than openloop_boost_q. |

### Why your motor needs ~35–40% throttle to start
At 35% throttle, VESC commands roughly 33A motor current (35% × 95A max). At openloop, that 33A is enough to overcome water resistance + rotor inertia and the rotor follows the forced field. Below ~33A, the openloop current isn't producing enough torque to spin the loaded prop — you're commanding rotation but the rotor stays put.

With `foc_sl_openloop_boost_q = 5A`, VESC adds 5A on top of whatever the throttle commands during openloop. Your 10–15% throttle commands ~10–14A; with the boost it's ~15–19A — enough to break free in most underwater conditions. Once the prop is spinning and ERPM > `foc_openloop_rpm`, sensorless kicks in and your 10–15% throttle goes to producing actual motor torque, no boost needed.

---

## 3. Recommended VESC config changes — in priority order

### Step 1 — increase openloop Q boost (biggest single win)
In VESC Tool → Motor Settings → FOC → Advanced → Sensorless tab:
- Set `Open-Loop Boost (Q)` from `0` to `3` initially.
- Test on bench (no prop, motor in air): should spin at ~10% trigger, smooth.
- Test in tub of water with prop submerged: should spin at ~15–20% trigger.
- If still hesitant, raise to `5`. If smooth at 5 but you want even quicker break-free, raise to `8`.
- **Hard cap: 10 A.** Above this you risk overcurrent fault on the battery side and excessive heat in the motor when stalled at low RPM.

### Step 2 — lower openloop RPM threshold
In the same tab:
- Set `Sensorless Open-Loop RPM` (`foc_openloop_rpm`) from `1500` to `1000`.
- This makes VESC trust its observer earlier. With your motor's R/L values, the observer should be reliable down to ~800 ERPM.
- If you start getting fault 6 (`UNDERVOLTAGE` from observer disagreement) or rough running at low speed, raise back toward 1500 in steps of 100.

### Step 3 — lower sensorless transition ERPM
- Set `Sensorless ERPM Start` from `2500` to `1500`.
- Set `Sensorless ERPM` (full sensorless) from `3400` to `2500`.
- These define the band where VESC blends openloop and sensorless modes. Lower transition = faster handoff = smoother low-speed.

### Step 4 — verify input config for full BREmote range
This is independent but mandatory after the smooth-start tuning. The app config file you tried to upload (`app vesc1 bremote v1 (1)`) appears empty or unreadable on my end — I couldn't parse it. **Re-export from VESC Tool** as XML and drop it in the same folder; I'll review the PPM input mapping next session.

When you do re-run PPM Mapping, follow the §3.4 procedure from `THROTTLE_PIPELINE_ANALYSIS.md`:
- BREmote `throttle_mode = 1` (no gears)
- BREmote `thr_expo = 50` (true linear — your planned change)
- RTM disarmed, system unlocked
- Trigger to physical max → save max
- Trigger to physical min → save min

### Step 5 — order of operations on first calibration
1. Set `foc_sl_openloop_boost_q = 3` first.
2. Run motor detection (FOC wizard) to confirm motor parameters didn't drift.
3. Run input PPM Mapping with BREmote at full range.
4. Bench test the motor with no prop — confirm smooth start at <15% trigger.
5. In-water test with prop submerged but board out of water (or in a tub) — confirm break-free at ~15–20% trigger.
6. If too eager (motor wants to start before you're ready), lower boost to 2. If still sluggish, raise to 5.
7. Then make your other tweaks (`foc_openloop_rpm`, `foc_sl_erpm*`) one at a time, test between each.

---

## 4. The hardware fix (future, not now)

Sensored FOC eliminates the openloop guesswork entirely. With hall sensors or an absolute encoder, VESC always knows rotor position, even at zero RPM. Result: full torque from zero, no hesitation, no openloop boost needed.

For e-foil, hall sensors are the practical option. Optical encoders don't survive water. Hall sensors are sealed in the motor stator slots and are common on hoverboard-class motors — your 6384 may or may not have them depending on vendor.

If your 6384 has hall sensors:
- Wire them to VESC's sensor connector (5V, GND, H1, H2, H3).
- In VESC: `foc_sensor_mode` = 1 (Hall) instead of 0 (Sensorless).
- Re-run motor detection; VESC measures the hall table.
- Smooth start "just works" — no openloop tuning needed.

If your motor lacks hall sensors, retrofitting is possible but invasive. Park this until your sensorless tuning hits its limits.

---

## 5. How BREmote interacts with this — and why I'm NOT recommending a BREmote-side fix

### What BREmote can legitimately do (within Section 9 safety)
- **`thr_expo` curve** (0–100, default in your code 100, your planned change 50). At 50 = linear. Below 50 = curve flattens at extremes (more output for mid-trigger inputs). Above 50 = curve sharpens at extremes (less output mid-trigger, more at endpoints).
- For your smooth-start problem, a value around 30–40 *might* help by producing slightly more output at low trigger. But it changes the entire curve, not just startup, and it would feel weird everywhere else.
- **My advice:** keep `thr_expo = 50` (linear) as planned. Let the VESC do its job. Don't compensate for VESC tuning by warping the trigger curve.

### What BREmote is NOT going to do
- **No automatic startup boost** that briefly sends >user-commanded throttle to break the motor free. This would violate Section 9 (creator safety: autonomous modes can only subtract, never add). The boost belongs in VESC where it can be implemented as Q-axis current (motor-aware) rather than as PWM duty (motor-blind).
- **No "smart" throttle remapping** that detects motor stall and pushes harder. Same Section 9 issue, plus the RX has no fast feedback path for motor state — VESC sees this in microseconds, BREmote sees it via 10 Hz telemetry at best.

### Where the boundary is
BREmote's job: deliver the user's trigger position to the VESC over LoRa, faithfully and with predictable safety overrides (RTM cap, failsafe). It does NOT substitute for motor controller tuning. Once the VESC is tuned right, BREmote becomes "boring and correct" — exactly what you want for a safety-critical link.

---

## 5b. Deeper explanation of the two key parameters (added 2026-05-03)

### What is `foc_sl_openloop_boost_q` really?

**The short version:** it's a torque-producing current that VESC adds during the brief startup window when it can't yet sense rotor position from back-EMF. Without it, your trigger has to push enough current itself to break the motor free. With it, VESC supplements your trigger with a controlled "shoulder push."

**The technical version:**
- A FOC motor controller controls current in two perpendicular axes: **D-axis** (parallel to rotor magnets — magnetizes/demagnetizes, makes no torque) and **Q-axis** (perpendicular to rotor magnets — produces all the useful torque).
- "Q" is the torque axis. More Q current = more torque, until the motor is in saturation.
- "Openloop" mode is what VESC does when ERPM is too low for the back-EMF observer to lock on (back-EMF voltage scales with speed; below a few hundred ERPM there's not enough signal). In openloop, VESC just rotates the magnetic field at a programmed rate and assumes the rotor is following.
- `foc_sl_openloop_boost_q` is **extra Q current added during openloop only**. Once the motor reaches the closed-loop transition (`foc_openloop_rpm`), the boost falls away.

**Real-world analogies:**

1. **Pushing a stalled car.** Static friction is much higher than rolling friction. You can't lightly nudge a 2-ton car and expect it to roll — you need to put your shoulder into it. Once it's rolling, gentle pushing keeps it going. The boost is your initial shoulder-push force; once the prop is spinning, the user's normal trigger current is enough.

2. **Pulling a cork from a wine bottle.** You apply a lot of force to break the seal, then almost no force to extract the rest. Without enough initial force the cork doesn't come out at all. Without enough boost, the underwater prop won't break free of static water load.

3. **A diesel truck pulling away from a stop.** The driver applies more throttle to overcome inertia, then eases off once moving. The driver sees the full throttle command. With a passenger car, the engine management does this automatically — that's exactly what `boost_q` does at the motor level.

**What different values of `foc_sl_openloop_boost_q` actually do:**

| Value (A) | What happens to your motor | When this is right |
|---|---|---|
| **0 (current)** | No boost. User trigger must overcome static load entirely. Underwater + prop = needs ~30A motor current = ~35–40% throttle to move. **This is your problem today.** | When motor has hall sensors and openloop is unused, or when load is very light (free spinning fan, no friction). |
| **1–2 A** | Minimal boost. May help slightly. Won't break a heavy underwater prop free at low trigger. | E-skate, e-bike, lightly-loaded scenarios. |
| **3 A** ⭐ recommended start | Mild boost. Combined with ~10–15% user throttle = ~13–18A motor current. Should break a typical e-foil prop free underwater. | Your starting point. Test before going higher. |
| **5 A** | Stronger boost. Combined with ~10% trigger = ~15A motor current. Confident startup at very low trigger. | If 3A isn't enough after bench testing. |
| **8 A** | Aggressive. Motor will eagerly break free even at minimum trigger. Risk: at 0% trigger the motor still tries to spin (boost is added on top of zero). VESC has logic to suppress this when commanded current is exactly zero, but worth confirming. | Heavy underwater loads, large props, tow-buggy hauling adult riders through chop. |
| **10 A** | Hard ceiling I'd recommend. | Extreme cases only. |
| **15+ A** | Risky. Motor windings see this current even at zero RPM (no back-EMF to limit it). Without ventilation/water-cooling, windings can overheat in seconds if rotor is jammed. Battery side sees full draw. | Avoid unless you really know what you're doing and have temperature monitoring. |

**Important: the boost DECAYS automatically as ERPM rises.** Look at `foc_start_curr_dec` and `foc_start_curr_dec_rpm` (yours: `1` and `2500`). VESC reduces the boost smoothly as it transitions from openloop to closed-loop sensorless. So the boost is only present during the first ~1 second of motion. It's not a permanent power add.

**What can go wrong:**
- Too little (0–2): motor doesn't start at low trigger, you need higher trigger to break free. Your current symptom.
- Too much (10+): motor runs warm at idle, draws unnecessary battery, can demagnetize the rotor over many startup cycles in extreme cases (rare).
- Just right (3–8): motor spins reliably at 10–15% trigger, no audible struggle, no excess heat. **This is what you're tuning to.**

---

### What is `foc_openloop_rpm` really?

**The short version:** it's the speed threshold above which VESC trusts its rotor-position observer. Below the threshold, VESC operates blind (forced rotation); above it, VESC has actual feedback and can deliver precise torque.

**The technical version:**
- VESC's sensorless observer estimates rotor position by integrating back-EMF voltage (the voltage induced in the unpowered phases by the spinning rotor's magnets).
- At very low speed, back-EMF voltage is tiny (proportional to ERPM × flux linkage). Below some threshold the signal is buried in noise and the observer's estimate is unreliable.
- `foc_openloop_rpm` defines that threshold. Below this ERPM, VESC ignores the observer and forces a rotating magnetic field at a programmed rate. Above this ERPM, VESC trusts the observer and uses actual position feedback.

**Real-world analogies:**

1. **Speedometer in an old car.** Below 5 mph the needle is jumpy and unreliable. Above 5 mph it reads accurately. If you set your "I trust the speedo" threshold at 5 mph, you drive blind below that and accurately above. Set the threshold too high (e.g., 30 mph), you drive blind for a long time and miss subtle changes. Set too low (e.g., 1 mph), you might trust a noisy reading and steer based on garbage data.

2. **GPS navigation in a tunnel.** No satellite signal = no position. Phones use "dead reckoning" (estimated position based on last known speed and direction) until they get the real signal back. The threshold for "dead reckoning vs real GPS" is analogous to `foc_openloop_rpm` — too long in dead reckoning means you've drifted off the actual position.

3. **A dimmer switch with a deadband.** Below the deadband the bulb is fully off. Above it the bulb dims smoothly. The deadband prevents flickering at the very low end. `foc_openloop_rpm` is the upper edge of FOC's "low-RPM deadband" where it switches from forced behavior to controlled behavior.

**What different values of `foc_openloop_rpm` actually do:**

| Value (ERPM) | What happens | When this is right |
|---|---|---|
| **500** (very low) | VESC trusts observer almost immediately. Best low-speed control if observer is accurate. **Risk:** if motor params drift (cold motor, water temperature changes, prop fouling), observer can give wrong position → motor stutters, faults, runs backward. | Motors with very stable parameters and very accurate auto-detection. Bench setups, lab conditions. |
| **800–1000** ⭐ recommended for you | Observer takes over early. Smooth transition from openloop to sensorless. Good low-speed feel without observer stress. | Most well-detected motors with stable conditions. **My recommendation for Step 2 of your tuning.** |
| **1500 (your current default)** | Conservative transition. Stays in openloop longer. Reliable startup, but low-speed control feels a bit "wooden" because openloop is blind. | Motors that struggle to lock on to back-EMF — usually means R/L params need re-detection. |
| **2500** | Very conservative. Stays in openloop until clearly above startup speed. Stable but low-speed control is genuinely poor. | Motors with bad parameter detection, high-noise environments, or cases where the observer keeps faulting. |
| **5000+** | Essentially permanent openloop until you're cruising. VESC never feels the rotor's actual position at low speed. Awful low-speed control, but unkillable startup. | Last-resort tuning when nothing else works. |

**Why lower is generally better (within reason):**
- Closed-loop sensorless gives you ACTUAL torque control. Trigger goes up, observer sees rotor lagging, more current is applied to catch up. Smooth.
- Openloop gives you FORCED rotation at a fixed rate. Trigger goes up, the boost current rises but the rotation rate is whatever VESC programmed. The rotor either follows or doesn't. There's no closed-loop "I want more torque" feedback.

**Why too low is bad:**
- The observer needs back-EMF signal to estimate position. Below the noise floor, it just produces garbage.
- Garbage estimates in closed-loop mode = motor commanded to wrong angle = commutation error = motor stutters, vibrates, or runs backward.
- Symptom: motor starts to spin during openloop, then at the transition point it suddenly jerks/stops.

**The sweet spot for your motor:**
You have a 6384 outrunner with decent R (0.023Ω) and L (18µH) and good auto-detection. Observer should lock cleanly at 1000 ERPM. Going to 800 might give even smoother low-speed feel but increases risk of bad detection days. **Start at 1000, drop to 800 only if 1000 feels rough.**

**Interaction with `foc_sl_erpm_start` and `foc_sl_erpm`:**
- `foc_openloop_rpm` (1500): below this, full openloop.
- `foc_sl_erpm_start` (your 2500): above this, sensorless **starts** taking over (blend zone begins).
- `foc_sl_erpm` (your 3400): above this, sensorless is **fully** in charge (blend complete).

So your actual transition zone is 1500 → 2500 → 3400 ERPM. That's a 2000-ERPM blend window. Lowering all three (1000 / 1500 / 2500 in my recommendation) shifts the entire transition window down ~1000 ERPM, putting smooth control in your usable speed range.

---

## 6. FOC config file — clean copy of recommended changes

If you want to apply changes via XML import (not via the wizard), the diff against your current config:

```xml
<!-- BEFORE -->
<foc_sl_openloop_boost_q>0</foc_sl_openloop_boost_q>
<foc_openloop_rpm>1500</foc_openloop_rpm>
<foc_sl_erpm_start>2500</foc_sl_erpm_start>
<foc_sl_erpm>3400</foc_sl_erpm>

<!-- AFTER (Step 1 only — start here, conservative) -->
<foc_sl_openloop_boost_q>3</foc_sl_openloop_boost_q>
<foc_openloop_rpm>1500</foc_openloop_rpm>
<foc_sl_erpm_start>2500</foc_sl_erpm_start>
<foc_sl_erpm>3400</foc_sl_erpm>

<!-- AFTER (full recommendation if Step 1 helps but isn't enough) -->
<foc_sl_openloop_boost_q>5</foc_sl_openloop_boost_q>
<foc_openloop_rpm>1000</foc_openloop_rpm>
<foc_sl_erpm_start>1500</foc_sl_erpm_start>
<foc_sl_erpm>2500</foc_sl_erpm>
```

**Don't apply both at once.** Step 1 first, test, then Step 2-3 only if needed. One change at a time = easy to back out if anything goes wrong.

---

## 7. The VESC tool repo — what's there for further help

The official source: `https://github.com/vedderb/vesc_tool` (that's the GUI/configurator) and `https://github.com/vedderb/bldc` (the firmware itself). For tuning questions, the `bldc` repo is more useful — `motor/foc_math.c` and the FOC observer code are well-commented and the `CHANGELOG.md` explains why parameters changed across versions.

Most useful pages on `vesc-project.com` for your question:
- "FOC and very low speeds" thread — exactly your problem, multiple solutions discussed
- "FOC Startup issues" thread — focuses on `openloop_boost_q` and `openloop_rpm`
- "Sometimes startup problems in FOC-Mode" — older but the core advice still applies

---

## 8. Decision summary for next session

1. **Keep `thr_expo = 50` on BREmote (true linear).** ✅ You already planned this.
2. **Apply VESC Step 1 only:** `foc_sl_openloop_boost_q = 3`. Bench test. Report back.
3. **Re-export your VESC app config** as XML — the file you uploaded was empty/unreadable.
4. **Don't add a BREmote-side startup boost.** Section 9 forbids it; VESC is the right place.
5. **Long-term:** consider hall sensors on the motor for sensored FOC.

---

## Sources

- [FOC vs. BLDC | VESC Project](https://www.vesc-project.com/node/1869)
- [FOC and very low speeds | VESC Project](https://vesc-project.com/node/505)
- [FOC Startup issues | VESC Project](https://vesc-project.com/node/3671)
- [foc openloop not starting motor on higher erpm | VESC Project](https://www.vesc-project.com/node/589)
- [Sometimes startup problems in FOC-Mode | VESC Project](https://vesc-project.com/node/85)
- [VESC Tuning tips | Endless Sphere DIY EV Forum](https://endless-sphere.com/sphere/threads/vesc-tuning-tips.125527/)
- [FOC wizard confusion | VESC Project](https://vesc-project.com/node/1029)
- [vedderb/vesc_tool on GitHub](https://github.com/vedderb/vesc_tool)
- [vedderb/bldc on GitHub — VESC firmware source](https://github.com/vedderb/bldc)
- [VESC vs ESC pro&cons for efoil — FOIL.zone](https://foil.zone/t/vesc-vs-esc-pro-cons-for-efoil/5445?page=7)
