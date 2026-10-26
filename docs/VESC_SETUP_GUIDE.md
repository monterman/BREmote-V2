# VESC setup for beginners — BLDC first, FOC later

**BREmote V2.5-Evo** · for a builder setting up a VESC (or two) behind the BREmote receiver for the first time

This guide takes a VESC from the box to a working throttle, in the order the VESC Project itself uses:
**motor setup and detection first, then the App / PPM input, then the PPM calibration.**

**Run the motors in BLDC first.** BLDC needs few motor settings and forgives a rough setup, so any
wiring, 5 V, mapping or remote fault shows up on its own. **Only after everything works, try FOC** — it is
quieter and more efficient, and it is the setup to end up with ([§ 4](#4-later-try-foc--and-its-three-traps)).

> ### Wiring: one PPM lead per VESC, and CAN is optional
>
> **Each VESC always gets its own PPM lead from the receiver**, into that VESC's own PPM connector. This
> is fixed: BREmote mixes the steering into its two channels, so each VESC must follow its own.
> **Never drive a VESC's throttle over CAN. CAN is never used for throttle or steering.**
>
> A **CAN cable between the two VESCs is optional.** Choose now; the steps say where the two differ.
>
> - **Without CAN:** the two VESCs are fully independent. Skip the CAN ID (step 5). **Multiple VESCs Over
>   CAN = OFF** (there is nothing to forward to). Battery negative is the shared ground.
> - **With CAN:** each VESC gets its own CAN ID (step 5). **Multiple VESCs Over CAN must still be OFF**,
>   because BREmote mixes the steering into the two channels ([§ 7](#7-twin-vescs)). What CAN adds: the
>   second VESC's telemetry reaches the BREmote log. The receiver reads VESC 2's currents, temperatures,
>   duty, ERPM, voltage and fault code through VESC 1 and logs them at log level 4 and up; without CAN
>   those columns read -999. This is in the nightly firmware; the master firmware reads VESC 1 only.
>   The remote's screen shows VESC 1's values, with or without CAN.

Other guides this one leans on:
- [ZERO_TO_FOILING § 2.5a](ZERO_TO_FOILING.md#25a-power--does-your-esc-put-5-v-on-the-ppm-connector) — 5 V on
  the PPM connector. **Without it the receiver's signal never reaches that VESC.**
- [ZERO_TO_FOILING § 2.5b](ZERO_TO_FOILING.md#25b-calibrate-the-ppm-input-on-both-controllers) — the
  calibration rules this guide follows.
- [VESC FOC tuning notes](VESC_FOC_TUNING_NOTES.md) — for later, when you move to FOC.

---

## 1. PPM and PWM mean the same thing here

VESC Tool calls its servo input **PPM**. What it actually reads is the width of **one servo pulse**:
about 1-2 ms long (1000-2000 µs), about 50 times a second, **one pulse per channel**. Most hobby people
call that signal **PWM**, and it is exactly what the BREmote receiver sends. **In this guide PPM and PWM
mean the same thing.**

One catch: a true multi-channel **PPM stream** (several channels on one wire, from some RC receivers) is a
different signal. The VESC input does not decode it, and BREmote does not send it.

---

## 2. Before you start

- **Drive off the ground** — wheels up, or props out of the water — **and nobody near the props**, from
  step 2 to step 10.
- **VESC Tool and the VESC firmware must match.** If VESC Tool says **Limited Mode**, update first.
- **5 V on each PPM connector:** check it now ([ZERO_TO_FOILING § 2.5a](ZERO_TO_FOILING.md#25a-power--does-your-esc-put-5-v-on-the-ppm-connector)).
- **Two VESCs?** Do every step on **each** VESC. Connect to each one directly, or, with CAN, to VESC 2
  through VESC 1.
- Menu names are given for **desktop** VESC Tool 6.x and the **phone** app where they differ.

---

## 3. The steps

### Motor first

**1. Back up both configs.**
- Phone: Start → **Backup Configs**.
- Desktop: save the motor configuration and the app configuration to files (File menu).

**2. Motor type BLDC, current limits, battery cutoff.**
- Desktop: Wizards → **Setup Other Motors** → Motor Type **BLDC** → set the current limits → soft battery
  cutoff? **Yes** → set the voltage limits → sensor mode **Sensorless** (unless hall wires are plugged in).
- Phone (there is no BLDC wizard; set it by hand): **Motor Cfg** → General → General: **Motor Type = BLDC**.
  BLDC → General: **Sensor Mode = Sensorless**. Then General → **Current** and General → **Voltage**.
  **Write**.
- **Limits:**
  - **Motor Current Max:** from the motor's rating. The firmware default is 60 A; the reference build runs
    95 A on a 63xx outrunner.
  - **Battery Current Max is per VESC, and the pack supplies the sum.** With two VESCs, twice this setting
    must stay within the pack's continuous rating. The firmware default is 99 A; the reference build runs
    35 A each.
  - **Battery Voltage Cutoff Start / End:** set them for your pack. **Never leave the defaults of 10 V /
    8 V** — they never act on a 10S or larger pack. VESC Tool's own wizard uses 3.4 / 3.0 V per cell for
    Li-ion: 34 / 30 V on 10S. The reference build runs 32 / 29 V on 10S, chosen against its measured sag
    under load.

**3. BLDC detection** — motor free to spin, prop out of the water, nothing loading it.
- Unplug the receiver's PPM lead while you detect: no other input may be active during detection.
- Desktop: the **BLDC Settings** page of Setup Other Motors, or Motor Settings → **BLDC** → "Detect BLDC
  Parameters" → start → **Apply** → **Write motor configuration**.
- Phone: Motor Cfg → **"..." → Detect BLDC Parameters...** → Detect → **Apply & Close** → **Write**.
- Keep the detection's default values. It measures the motor; it does **not** change Motor Type.

**4. Direction.**
- Desktop: Motor Settings → General → General: **Invert Motor Direction**.
- Phone: Motor Cfg → General → General, or Start → **Invert Motor Directions**.
- **Re-check the direction after every detection.** The final check is the wheels-up cross-steer test in
  [ZERO_TO_FOILING § 2.6](ZERO_TO_FOILING.md).

**5. CAN ID — with CAN only; skip it without CAN.** Desktop: App Settings → General → **VESC ID**. Phone:
**App Cfg** → General → VESC ID. **Each VESC needs its own ID:** VESC 1 (the one on the receiver's UART) is
ID 1, VESC 2 is ID 2. The nightly firmware asks CAN ID 2 for VESC 2's telemetry.

### Then the App / PPM input

**6. App, timeout, timeout brake.** Desktop: App Settings → General, then **Write app configuration**.
Phone: App Cfg → General, then **Write**.
- **App to Use = PPM and UART** on the VESC whose UART goes to the receiver (BREmote telemetry). **PPM**
  is enough on the other one. **Both** VESCs need a PPM option.
- **Timeout = 1000 ms** (the default; keep it).
- **Timeout Brake Current = 0 A** (the default: the motor coasts when the signal stops).

> ### ⚠️ Do not trust the "Setup Input" wizard on a BREmote twin build
>
> Both Input wizards (desktop and phone) turn **Multiple VESCs Over CAN ON** — the desktop wizard never
> even shows that setting — and they pick **Current No Reverse With Brake**, where a released trigger asks
> for full brake. Both are wrong for BREmote: Multiple VESCs Over CAN makes one VESC send its command to the
> other, which fights the differential steering. Without a CAN cable that setting has nothing to send to
> and does no harm, but set it OFF anyway. **Skip the input wizard and follow steps 7-9, or fix those
> two settings by hand afterwards** (step 8).

### Then the PPM calibration

**7. Map the PPM input (calibration).** Remote **paired** and in **`throttle_mode = 1` (No Gears)**, so a
full squeeze sends the full range ([ZERO_TO_FOILING § 2.5b](ZERO_TO_FOILING.md#25b-calibrate-the-ppm-input-on-both-controllers)).
Drive still off the ground.
- **Twin build: set Control Type = Off on both VESCs before the first squeeze.** Off holds only the VESC it
  is set on: a VESC that is already live drives its motor at full power while you map the other one.
- Write Control Type Off first, then plug the receiver's PPM lead back in (it was unplugged at step 3).
- Desktop: App Settings → PPM → General: **Control Type = Off** → Write. Then PPM → **Mapping** tab, turn on
  **Stream realtime app data** in the toolbar. Trigger released → **Reset**, full squeeze, release →
  **Apply** → **Write app configuration**.
- Phone: App Cfg → PPM → General: Control Type **Off** → Write. Then **"..." → PPM Mapping...**: trigger
  released → **Reset**, full squeeze, release → **Apply and Write**.
- Expect Start ≈ 1.0 ms, End ≈ 2.0 ms, Center ≈ 1.5 ms. Control Type Off while you map is the official
  procedure: the motor cannot move.
- Desktop: **Apply alone is not enough** — press **Write app configuration** too.

**8. PPM settings.** Desktop: App Settings → PPM → General and Mapping. Phone: App Cfg → PPM → General and
Mapping.
- **Control Type = Current No Reverse** (not *Current*, not *Current No Reverse With Brake* — see § 5).
- **Input Deadband = 2-3 %.** The VESC default is **15 %**, which makes the first 15 % of the trigger dead.
- **Safe Start = Regular.**
- **Multiple VESCs Over CAN = OFF**, with CAN or without.
- **Traction Control = OFF.**
- **Median Filter = on.**
- **Positive / Negative Ramping Time = 0.4 / 0.2 s** (the defaults).
- Throttle Curve: leave Throttle Expo at 0.
- **Write.**

**9. Check.** Desktop: app data stream on. Phone: the PPM Mapping screen.
- The **released trigger reads 0 %**, a full squeeze about 100 %, the same on both VESCs.
- No creep with the trigger released.
- After power-up nothing runs until the trigger has been released for about 1 s (Safe Start).
- A gentle squeeze: each motor turns the forward way; if not, step 4 on that VESC.
- **Released trigger above 0 %, or creep: do not ride.** Map again (step 7) and check the deadband (step 8).

**10. Twin build: VESC 2.** Do steps 2-6 on VESC 2 too. Map both VESCs (step 7, each on **its own** PPM
channel, Control Type Off on both) before you set either one live (step 8), then check both (step 9). Map
them the same way: **both motors must start together and top out together.**

**11. Before the first ride, go back to [ZERO_TO_FOILING](ZERO_TO_FOILING.md).** The remote is still in
`throttle_mode = 1`, the full range: pick a gentler riding mode (end of § 2.5b). Then pass the wheels-up
cross-steer check (§ 2.6, the top safety gate) and the dry test (Phase 4). Only then ride in BLDC, and get
the whole craft working (remote, steering, Follow-Me) before you change the motor mode.

---

## 4. Later: try FOC — and its three traps

When everything works in BLDC: desktop Wizards → **Setup Motors FOC**, phone Start → **Setup Motors**.
Usage: **Propeller**.

**Same rules as steps 2-10:** drive off the ground, props clear and nobody near them, and the receiver's PPM
lead unplugged while the wizard detects. The detection spins the motors.

1. **"Load Default Parameters": answer No.** Yes resets the **motor and the app config of every VESC on the
   CAN bus** — IDs, App to Use, the PPM mapping and the PPM settings. If you answered Yes, redo steps 5-9 on
   every VESC.
2. **Check the cell count** on the wizard's Battery page **before detection.** The wizard guesses the pack
   from its voltage, and anything between 39 and 51 V is set to **12 cells**: a **charged 10S pack** (above
   39 V; full is 42 V) or any 11S pack is read as 12S. With Propeller usage the cutoff then becomes 40.8 V
   start / 36.0 V end, and the motors lose power almost at once.
3. **Tune the open-loop values after the wizard, not before.** The wizard rewrites Open loop ERPM and
   Sensorless ERPM, so earlier tuning is lost. Startup tuning: [VESC FOC tuning notes § 3](VESC_FOC_TUNING_NOTES.md#3-recommended-vesc-config-changes--in-priority-order).

Afterwards re-check **Motor and Battery Current Max**, the **cutoff** and the **direction** on each VESC,
repeat **step 9**, and pass the cross-steer check ([ZERO_TO_FOILING § 2.6](ZERO_TO_FOILING.md)) again
before you ride.
The PPM mapping survives if you answered No.

---

## 5. Which control type

| Control Type | Released trigger (1000 µs) means |
|---|---|
| Current | brake, then **reverse** drive |
| **Current No Reverse** ✓ | **zero**: off |
| Current No Reverse With Brake | **full brake** (its zero is the centre pulse). The input wizard leaves this set |

Current No Reverse does not use the centre at all.

## 6. What the PPM settings do

- **Timeout** (default 1000 ms): if no valid pulse arrives for this long, the VESC stops driving and applies
  the Timeout Brake Current (0 A = coast).
- **Safe Start, Regular** (default): the motor may start only after the input has read zero for 50 pulses
  in a row — about 1 s. It re-arms after power-up, after every app-config write, after a timeout and after a
  fault. Keep it on Regular.
- **Input Deadband:** the bottom slice of the range reads zero and the rest is stretched to full scale.
  2-3 % keeps a margin against creep without wasting trigger travel; 0 % leaves no margin.
- **Median Filter** (default on): each pulse is replaced by the median of the last three, so one corrupt
  pulse is ignored.

## 7. Twin VESCs

BREmote mixes the steering into its two channels: one motor gets the throttle minus the turn, the other the
throttle plus the turn. **So each VESC must follow its own channel.**

| Setup | BREmote? |
|---|---|
| **Each VESC on its own PPM channel, Multiple VESCs Over CAN OFF on both**, with or without a CAN cable | **Yes** |
| One PPM into a "master" VESC with Multiple VESCs Over CAN ON, the other VESC commanded over CAN | **No** — the master sends its own command to every VESC it hears on CAN, so the steering is lost and the two commands fight |

- **CAN between the VESCs is optional.** With CAN only: it carries VESC 2's telemetry to the BREmote log
  (nightly firmware) and lets VESC Tool configure VESC 2 through VESC 1. It never carries throttle or
  steering. Without CAN: skip step 5 and connect VESC Tool to each VESC directly.
- VESC Tool's help says "only the master needs an app". That applies to the master / CAN setup, not this one.
- Never split one receiver signal into several VESCs with a Y-cable. BREmote does not: it has two separate
  channels, each powered by its own VESC ([ZERO_TO_FOILING § 2.5a](ZERO_TO_FOILING.md#25a-power--does-your-esc-put-5-v-on-the-ppm-connector)).

## 8. Mistakes beginners make

1. **Calibrating the remote before the motor setup.** The FOC wizard's "Load Default Parameters" resets the
   app config too.
2. **Using the Input wizard and stopping there.** It leaves Multiple VESCs Over CAN on and the control type
   at "With Brake".
3. **Mapping with a live control type.** Map with Control Type Off, then set Current No Reverse.
4. **Desktop: Apply without Write.** Apply only changes VESC Tool's copy.
5. **Detecting with the prop loaded or in the water.** A bad detection gives a wrong motor model; a clean
   re-detection has fixed a "bad" motor before.
6. **Leaving the cutoff at 10 V / 8 V,** or accepting a wrong cell count in the FOC wizard.
7. **Treating Battery Current Max as the pack limit.** It is per VESC: two VESCs draw double.
8. **Leaving the 15 % deadband** (the first part of the trigger does nothing), or setting 0 % (creep risk).
9. **One VESC's input bar flat at zero.** On BREmote that is almost always missing 5 V on that channel's
   connector, not the mapping ([ZERO_TO_FOILING § 2.5a](ZERO_TO_FOILING.md#25a-power--does-your-esc-put-5-v-on-the-ppm-connector)).
10. **Mapping the two VESCs differently.** One motor then starts first.
11. **Not re-checking the direction after a detection.**
