# BREmote V2.5-Evo — Beta Tester Guide

Welcome, and thanks for testing. Read the hardware section first — the settings below are tuned for **my** exact setup. If yours differs, my numbers are a starting point, not gospel.

---

## 0. My hardware (what these settings assume)

| | My setup | If yours differs |
|---|---|---|
| **Motor** | Flipsky **6384, sensorless**, **custom 170 KV** | Off-the-shelf 6384 is ~140 KV. Run your own motor detection. |
| **Poles** | 14 (7 pole pairs) | Set to your motor's poles. |
| **Battery** | **10S** Li-ion | Most people run 12S/14S. My voltage cutoffs are 10S-specific — change them. |
| **ESC** | VESC (FW 6.x), **one per motor** (dual-motor differential steering) | — |
| **Control** | **PPM + UART** from the BREmote RX, one signal wire per VESC | — |

> ⚠️ **Why this matters:** I run an unusual 10S system with a higher-KV custom motor. If you run a different motor, cell count, or KV, **do not blindly copy the current limits and battery cutoffs** — they're sized for the 6384 on 10S. Import my config as a baseline, then run **Motor Detection for your own motor** and set **battery cutoffs for your own cell count**.

---

## 1. Flashing (no Arduino needed)

No Arduino, no compiling. Download the prebuilt `.bin` for each board and flash it.

### ⭐ Flash these — the current builds

| Board | File | Version |
|---|---|---|
| **TX** | [`BREmote-TX-SW27-gps-verified.bin`](../Source/V2_Integration_Tx/TX%20firmware/BREmote-TX-SW27-gps-verified.bin) | **SW27** — current |
| **RX** | [`BREmote-RX-SW34-gps-verified.bin`](../Source/V2_Integration_Rx/RX%20firmware/BREmote-RX-SW34-gps-verified.bin) | **SW34** — current |

Browse all published builds, with notes on what each one is:
**[TX firmware →](../Source/V2_Integration_Tx/TX%20firmware/)** · **[RX firmware →](../Source/V2_Integration_Rx/RX%20firmware/)**

> 🚨 **Download the RAW file.** Click the `.bin` on GitHub, then use the **Download raw file** button
> (⤓, top-right of the file view) — *not* your browser's *Save Page As*, which saves an HTML page
> with a `.bin` name and will not flash. Or just
> `git clone https://github.com/monterman/BREmote-V2.git` and take it from the folder. **Check the
> size:** a real image is a few hundred KB; a few KB means you saved the web page.

Each folder has a `README.md` saying which build to start with and what changed. **The
`-gps-verified` builds above are the ones to use** — older builds in those folders are kept for
rollback and carry known issues.

### How to flash

⭐ **Use Espressif's [Flash Download Tool](https://www.espressif.com/en/support/download/other-tools)**
— the Windows GUI, and the method Ludwig demonstrates. Chip `ESP32-C3`, your `.bin` at address
**`0x10000`**, press START. No Arduino, no compiling.

**[→ Full step-by-step walkthrough](FLASHING_WITH_DOWNLOAD_TOOL.md)** · 🎥 **[Ludwig's video, from 40:00](https://youtu.be/r6JIZEq3aTU?t=2400)**

Prefer a command line? Same result:

```
esptool --chip esp32c3 --port COM<N> write-flash 0x10000 <the-file>.bin
```

> ⚠️ **Do not flash at `0x0`, and do not look for `.merged.bin`.** Merged images are deliberately
> not published: flashing one at `0x0` rewrites the whole chip and **wipes your SPIFFS config** —
> pairing, calibration and every setting. Use the plain `.bin` at `0x10000`.

> **Config note:** flashing a **different SW version** than your board currently runs resets all
> SPIFFS settings to defaults (you will re-calibrate and reconfigure). Flashing the **same SW
> version** keeps your settings. The filenames carry the version — check what your board is running
> with **`?conf`** before you flash, and **back up your config first**
> (see the README's *Backing up your settings*).

### Motor ramping (RX safety, SW33+)
`motor_ramp_s` (RX setting, in **seconds**) limits how fast the motors spin up — smooths the throttle **and** prevents a single motor from taking off at power-on or on a glitch. **Default 0.75 s**, range 0–4 s (`0` = instant/off). It's like the VESC ramp time, but on the remote, so it protects every motor even without a tuned VESC.
> ⚠️ It **also ramps differential steering** — a sharp turn builds over this time. You can always *straighten* instantly (fall is instant); only *starting* a hard turn is ramped. Set higher for smoother/safer, lower for snappier.

---

## 2. VESC Configuration — my values

VESC is powerful but fiddly. Easiest path to replicate mine:

1. **Import my config backups** in VESC Tool (`Load Configuration` → choose the XML). Motor config and App config are separate files.
2. **Run Motor Detection for YOUR motor** — this overwrites the electrical params (R / L / flux) with values matched to your hardware. **Do not keep my detected values.**
3. **Set battery cutoffs for your cell count** (mine are 10S).
4. Copy the **App / PPM** settings below as-is (they match what the BREmote RX outputs).

### 2a. Motor Configuration (6384, 10S) — reference
| Setting | My value | Note |
|---|---|---|
| Motor type | **FOC** | |
| Motor current max | **95 A** | sized for the 6384 |
| Motor current max (brake) | **−60 A** | |
| Battery current max | **100 A** | |
| Battery current regen | **−20 A** | |
| Absolute max current | **250 A** | |
| Battery cutoff start / end | **32 V / 29 V** | **10S** = 3.2 V / 2.9 V per cell — **change for your cell count** |
| Motor temp cutoff start / end | **85 / 100 °C** | |
| Min / Max input voltage | **23 / 72 V** | |
| Motor poles | **14** (7 pole pairs) | |
| Battery cells | **10 (10S)** | |
| Wheel diameter | **0.155 m** | for VESC speed calc; set to yours |
| FOC R / L / flux linkage | 0.02382 Ω / 18.2 µH / 0.004922 | **from MY detection — run your own, don't copy** |

### 2b. App Configuration — PPM (copy these as-is)
| Setting | My value | Note |
|---|---|---|
| App | **PPM and UART** | UART is required for telemetry (battery/watts/temp on the remote) |
| Control type | **2 — Current, with brake** | |
| **Input deadband** | **3 %** (0.03) | **Default ships at 0 — set this so the motor never creeps at idle.** Anywhere 2–5 % is fine. |
| Pulse start / center / end | **1.249 / 1.725 / 2.200 ms** | |
| Median filter | **On** | |
| Safe start | **On** | |
| Ramp up / down | **0.4 / 0.2 s** | |
| Throttle expo | **0** | |

> **The deadband is the one people miss.** With it at 0, the tiniest pulse above idle can make the motor hum/creep. Setting it to ~3 % gives a clean, quiet idle. (The BREmote firmware also outputs a true-zero center, so this is belt-and-suspenders — but set it anyway.)

> **Telemetry needs UART:** run the VESC in **PPM + UART** mode so the RX can read battery %, watts, FET temp, and motor amps and forward them to your remote screen.

---

## 3. Tow-buggy setup — motor direction & props

**Run "props-out".** Set both motors so the propellers push **outward**, not inward. In VESC Tool, after motor detection, flip the direction ("Invert Motor Direction" / reverse) until each side spins the right way — I run **both inverted** from the detected default to land on props-out.

Why props-out:
- Works **with** the differential-steering geometry (outward thrust helps the turn bite).
- More **stability**.
- Preferred over props-in.

**Props (3D printed):** print a **mirrored pair** — one clockwise, one counter-clockwise — so each motor gets the correctly-handed prop for its rotation, installed **props-out** (each pushing outward). Two identical props = one motor running a wrong-handed prop, fighting you.

> Order of operations: motor detection first, *then* set direction for props-out, *then* confirm the BREmote differential steering responds the right way (toggle left = turns left). If steering is reversed after this, flip `steering_inverted` on the RX rather than re-inverting the motors.

---

## 4. Known quirks (don't panic)

- **GPIO9 Hall mod only** — *skip this if you didn't add the second Hall sensor on GPIO9.* Power on the TX with the **magnet removed**. GPIO9 is a boot-strapping pin; if the magnet holds it low at power-on, the TX won't boot (dark screen, no buzz). Remove magnet → power on → it boots normally, then attach the magnet.
- **USB + remote ON** shows the charge screen and pauses boot. Send `?exitchg` over serial, or unplug, to continue. (Charging with the remote **OFF** is silent and faster — that's intended, not a fault.)
- **Calibration:** hold throttle / toggle **fully against the stop** for the entire 3-second window until the display changes. Easing off early shows **"EC"** (calibration error) — it just means the captured swing was too small.

---

## 5. Telemetry check

With the VESC in **PPM + UART** and the RX set to `data_src = 2`:
- Run **`?vescping`** on the RX serial → live battery %, voltage, watts, FET temp, motor amps, plus UART packet age (2 Hz, ~30 s).
- **Good = sane values + low/steady packet age.** If packet age climbs or values read N/A, the VESC UART link is the issue (not the radio).
- Those same values forward to the remote screen (power/watts, buggy battery, temp).

---

*These are the configs that work well for me on my 10S / 6384-170KV dual-motor tow-buggy. They're a strong baseline — adjust the motor-specific and battery-specific numbers for your own rig, and you'll be in good shape.*
