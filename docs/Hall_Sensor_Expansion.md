# Hall Sensor Expansion — BLE Activation on P_MAG (GPIO 9)

## Background — Hall sensors are already in BREmote V2

BREmote V2 (and V2.5-Evo) already use Hall-effect sensors throughout the handheld TX:

| Sensor | Purpose |
|---|---|
| Hall-effect **throttle** | Trigger finger position → throttle value 0–100% |
| Hall-effect **toggle** | Left/right toggle switches → steering, gear, gesture combos |
| Hall-effect **power switch** | Main on/off switch — no mechanical contacts |

These are the original LudwigBre design choices. Hall sensors are contactless, waterproof, and immune to corrosion — ideal for a device used near water.

---

## The Expansion — Adding a Fourth Sensor for BLE Control

V2.5-Evo adds an optional fourth Hall sensor on **P_MAG (GPIO 9)** specifically for BLE activation without reaching for the phone. The idea: hold a small magnet against the outside of the enclosure to toggle BLE on/off while riding.

This sensor is **optional** — BLE works fine without it using the SPIFFS config or the boot gesture. It only adds the magnet-based runtime toggle.

**Sensor used:** DRV5032 (unipolar Hall effect, 3.3V, SOT-23)

### Why GPIO 9

GPIO 9 on the ESP32-C3 (HT-CT62) is a free input pin with internal pull-up available. It was unused in the original BREmote V2 design and sits near the existing Hall sensor circuitry on the PCB.

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

## Firmware Behaviour

The sensor is only active when `bt_enabled = 1` (Hall/session mode) in SPIFFS.

| Gesture | Hold Duration | Result |
|---|---|---|
| Short hold | 400 ms – 4.9 s | BT dot → slow blink (BLE ready / advertising) |
| Long hold | 5 s+ (from slow state) | BT dot → fast blink (BLE active) |
| Short hold (from slow) | 400 ms – 4.9 s | BT dot → off (BLE off) |

`bt_enabled = 2` (always-on) bypasses the sensor entirely — BLE starts 5 s after boot regardless.

---

## SPIFFS Config

Set via the Web Serial Config Tool or serial:

```
?set bt_enabled 1
?save
```

| Value | Behaviour |
|---|---|
| `0` | BLE always off |
| `1` | Hall/session mode — magnet gesture activates BLE |
| `2` | Always on — BLE starts 5 s after boot, no magnet needed |

---

## No Sensor? Use the Boot Gesture Instead

If you don't want to add the expansion sensor, hold **Throttle + LEFT toggle** while powering on. BLE activates for that session regardless of `bt_enabled`. The magnet sensor and the boot gesture are independent paths to the same result.

---

## Source Reference

```
Source/V2_Integration_Tx/
  System.ino       — Hall sensor state machine → bt_dot_state (P_MAG polling)
  BREmote_V2_Tx.h  — P_MAG pin definition, bt_dot_state constants
  BLE.ino          — bt_session_forced flag, bleTelemetryLoop() bt_dot check
```
