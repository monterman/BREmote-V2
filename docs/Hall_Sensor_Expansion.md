# Hall Sensor Expansion — Magnet Gestures on P_MAG (GPIO 9)

## Background — Hall sensors are already in BREmote V2

BREmote V2 (and V2.5-Evo) already use Hall-effect sensors throughout the handheld TX:

| Sensor | Purpose |
|---|---|
| Hall-effect **throttle** | Trigger finger position → throttle value 0–100% |
| Hall-effect **toggle** | Left/right toggle switches → steering, gear, gesture combos |
| Hall-effect **power switch** | Main on/off switch — no mechanical contacts |

These are the original LudwigBre design choices. Hall sensors are contactless, waterproof, and immune to corrosion — ideal for a device used near water.

---

> **Installing for the first time?** See the [step-by-step install tutorial](Hall_Sensor_Install_Tutorial.md) — includes hardware options, soldering notes, and troubleshooting.

## The Expansion — a fourth sensor for magnet gestures

V2.5-Evo adds an optional fourth Hall sensor on **P_MAG (GPIO 9)**. Touch a small magnet (on your
wrist or glove) to the outside of the remote and you can **arm Follow-Me** or **start
Return-to-Me** without reaching for the toggles, even while you're being towed.

The sensor is **optional**. Everything also works from the toggles.

**Sensor used:** TI **DRV5032** (unipolar Hall-effect switch, 3.3 V, SOT-23). Easier-to-solder
alternatives are listed in the [install tutorial](Hall_Sensor_Install_Tutorial.md#hardware-options).

> ⚠️ **GPIO 9 is an ESP32-C3 boot (strapping) pin. Never power the remote on with the magnet
> against it.** A magnet holding GPIO 9 low at power-up puts the chip into download mode and the
> remote won't boot (dark screen, no buzz). Take the magnet away, power on, then use it. The
> firmware can't prevent this: the chip reads the pin before any firmware runs.

### Why GPIO 9

GPIO 9 on the ESP32-C3 (HT-CT62) is a free input with an internal pull-up. It was unused in the
original BREmote V2 design and sits near the existing Hall sensor circuitry on the PCB.

---

## Wiring

Connect the DRV5032 between the TX board and the enclosure wall (or a small external pocket):

```
DRV5032 VCC  → 3.3V
DRV5032 GND  → GND
DRV5032 OUT  → GPIO 9 (P_MAG)
```

The output is active-low — it pulls low when a magnet is present, floats high when no magnet. The internal pull-up on GPIO 9 handles the idle state; no external resistor needed.

![HT-CT62 Hall Sensor Wiring](HT-CT62%20Hall%20Sensor%20Wiring.png)

*[SVG version](HT-CT62%20Hall%20Sensor%20Wiring.svg)*

---

## Firmware behaviour — `mag_mode` (TX setting)

`mag_mode` picks what the magnet does. Default **0** (off / sensor not fitted).

### Mode 4 — tap and hold *(new in the upcoming release)*

*Available now for testing on the [`fm-stations` branch](https://github.com/monterman/BREmote-V2/tree/fm-stations);
it moves to master after water testing.*

| Magnet | Result |
|---|---|
| **Tap** (under about 0.6 s) | Follow-Me not armed → **arms it** at your stored station. Following → **steps to the next station** (the stations in `mag_fm_set`). Works **with the trigger held**, so you can arm on the rope. After a step, taps for the next second are ignored. |
| 0.6 s – 2.5 s | Nothing (a gap so a tap and a hold can't be confused) |
| **Hold 2.5 s, trigger released** | Two medium pulses: let go to start the **manual Return-to-Me** ("rn", then confirm with the trigger). With the trigger held the hold is ignored. If return-to-me can't start (switched off, no GPS) you get **St**. |

Mode 4 never switches Follow-Me off. Use the toggle gesture for that. See the
[Follow-Me guide](FOLLOW_ME_GUIDE.md) for the full picture.

### Modes 1–3 — hold and release *(current master firmware)*

The action happens when you **take the magnet away**:

| `mag_mode` | Hold | Result |
|---|---|---|
| 1 | about 2 s (one short pulse) | arm / disarm Follow-Me |
| 2 | about 2 s | arm / disarm Return-to-Me |
| 3 | about 2 s → Follow-Me, about 5 s → Return-to-Me | two-tier gesture |

In the upcoming release the Return-to-Me hold in modes 1–3 also needs the trigger released.

### Mode 0 — the older BLE role

With `mag_mode` 0 and `bt_enabled = 1`, the magnet keeps its first job: switching BLE on for the
session (BT dot on the screen).

| Gesture | Hold Duration | Result |
|---|---|---|
| Short hold | 400 ms – 4.9 s | BT dot → slow blink (BLE ready / advertising) |
| Long hold | 5 s+ (from slow state) | BT dot → fast blink (BLE active) |
| Short hold (from slow) | 400 ms – 4.9 s | BT dot → off (BLE off) |

With `mag_mode` 1–4 the magnet is used only for Follow-Me / Return-to-Me, and the BT dot shows the
real BLE state. `bt_enabled = 2` (always on) starts BLE 5 s after boot either way.

---

## Config

Set via the Web Serial Config Tool or serial:

```
?set mag_mode 4
?save
```

| Setting | Values |
|---|---|
| `mag_mode` | 0 off · 1 FM · 2 RTM · 3 FM + RTM · 4 tap = FM / station, hold = return-to-me *(upcoming)* |
| `mag_fm_set` | which stations a tap steps through: bit 0 = F1, bit 1 = F2, bit 2 = F3; 7 = all *(upcoming)* |
| `bt_enabled` | 0 BLE off · 1 magnet/session (mode 0 only) · 2 always on |

---

## No Sensor? Use the Toggles

- Follow-Me: **LEFT tap, then RIGHT hold** (while floating).
- Return-to-Me: **RIGHT tap, then LEFT hold** (trigger released).
- BLE for one session: hold **Throttle + LEFT toggle** while powering on.

---

## Source Reference

```
Source/V2_Integration_Tx/
  Hall.ino               — runMagGesture(): the magnet gestures (mag_mode)
  V2_Integration_Tx.ino  — P_MAG polling and the BT dot (mag_mode 0)
  BREmote_V2_Tx.h        — P_MAG pin, mag_mode / mag_fm_set fields
  BLE.ino                — bt_session_forced flag, bleTelemetryLoop() bt_dot check
```
