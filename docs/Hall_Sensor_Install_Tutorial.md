# Hall Sensor Expansion — Install Tutorial

> **Skill level:** Intermediate soldering required. The DRV5032 SOT-23 package is very small — see hardware alternatives below if you prefer an easier solder job.

This tutorial covers fitting the optional P_MAG Hall sensor that lets you activate BLE by holding a small magnet against the outside of the remote enclosure. You do not need this sensor — BLE works without it via SPIFFS config or the boot gesture. This is purely for convenience during a session.

---

## What You Need

| Item | Notes |
|---|---|
| Hall effect sensor (see options below) | Unipolar, active-low, 3.3V compatible |
| 3× thin wires, ~15–20 cm | 28 AWG or similar — needs to reach from the PCB to the enclosure wall |
| Small neodymium magnet | Disc or cube, 6–10 mm — keep one attached to your wetsuit wrist |
| Heat shrink or liquid electrical tape | Waterproof the sensor leads |
| Soldering iron + fine tip | 0.3–0.5 mm tip recommended for SMD options |
| Multimeter | For continuity test before closing the enclosure |

---

## Hardware Options

### What was used here: DRV5032 (SOT-23)

> ⚠️ **The DRV5032 SOT-23 is a very small package** — 3 pads, 1.6 × 2.9 mm footprint, 0.95 mm pitch. If you have never soldered SMD before, use one of the easier options below. The DRV5032 was used because it was already in stock, not because it is the easiest choice.

**DRV5032 SOT-23 specs:**
- Supply: 1.65 V – 5.5 V ✅ (3.3V nominal)
- Output: active-low open-drain
- Current draw: 170 nA (ultra-low — irrelevant here but a nice part)
- Package: SOT-23 (3-pin SMD, very small)

If you source the DRV5032, order a few extras — they are easy to lose and easy to lift a pad on. Use flux, tin the pads first, and work under magnification.

---

### Easier alternatives (same wiring, drop-in compatible)

All options below wire identically: VCC → 3.3V, GND → GND, OUT → GPIO 9.

| Part | Package | Why it's easier | 3.3V? | Notes |
|---|---|---|---|---|
| **US5881LUA** | SIP-3 (TO-92 style) | Through-hole leads, easy hand solder | ✅ 2.5V min | Widely available, cheap, reliable |
| **OH090U** | SIP-3 (TO-92 style) | Through-hole leads, easy hand solder | ✅ 2.7V min | Omnipolar — works with either magnet pole |
| **AH1806** | SOT-23 | Same size as DRV5032 but more available | ✅ 1.65V min | No easier to solder, but easier to source |
| **DRV5032** larger pkg | WSON-6 or SOT-23W (if available) | Slightly larger pads | ✅ | Check your distributor for package variants |

**Recommended for most builders: US5881LUA or OH090U** — through-hole leads make placement and soldering straightforward, no SMD skills required.

> **Magnet polarity note:** The DRV5032 and US5881 are unipolar — they activate on the **south pole** of the magnet facing the sensor. If your magnet doesn't trigger it, flip it over. The OH090U is omnipolar and works with either pole.

---

## Wiring Diagram

Connect the sensor to the HT-CT62 board as shown:

![HT-CT62 Hall Sensor Wiring](HT-CT62%20Hall%20Sensor%20Wiring.png)

*[SVG version for printing](HT-CT62%20Hall%20Sensor%20Wiring.svg)*

```
Sensor VCC  → 3.3V rail on HT-CT62
Sensor GND  → GND
Sensor OUT  → GPIO 9 (P_MAG)
```

The firmware uses the internal pull-up on GPIO 9 — no external resistor needed. Output is active-low: pulls to GND when a magnet is near, floats high otherwise.

---

## Step-by-Step Installation

### Step 1 — Prepare the sensor leads

Cut three lengths of 28 AWG wire, ~15–20 cm each (adjust to fit your enclosure routing). Strip 2 mm at each end.

For **SIP-3 parts (US5881, OH090U):** bend the leads gently at 90° so the sensor body sits flat against the enclosure wall. Pre-tin the leads before inserting.

For **SOT-23 parts (DRV5032, AH1806):** solder the three wires to the pads on a breakout board or directly to the IC. Apply flux first. Tin each pad, then tin each wire, then join — do not apply heat for more than 2–3 seconds per pad.

### Step 2 — Route wires inside the enclosure

Plan the route from the sensor mounting point (inside enclosure wall, near where you will hold the magnet) to the HT-CT62 board. Leave a small service loop so you can open the enclosure later without straining the joints.

Secure the wires with a small amount of hot glue or a cable tie — avoid routing directly over any RF antenna trace on the HT-CT62.

### Step 3 — Solder to the HT-CT62

Identify the following points on the HT-CT62 board:

| Signal | Where to solder |
|---|---|
| 3.3V | 3V3 pad or existing 3.3V component |
| GND | GND pad |
| GPIO 9 | GPIO 9 pad (P_MAG) — check the board silkscreen or pinout diagram |

Solder with a fine tip. GPIO 9 is a small pad — use flux and keep the iron on it no longer than 2 seconds.

### Step 4 — Mount the sensor

Position the sensor flat against the inside of the enclosure wall at a spot you can reach with the magnet from outside while riding. The sensor face must be within ~5 mm of the enclosure wall for reliable activation — most 3D-printed or thin ABS walls work fine.

Apply a small blob of hot glue to hold it in place. Let it cure before closing the enclosure.

Waterproof the three solder joints and exposed leads with heat shrink or liquid electrical tape. This is a water-adjacent device.

### Step 5 — Continuity test

Before closing the enclosure:

1. Power on the TX (on battery — not USB)
2. Hold your test magnet against the sensor face
3. Check with a multimeter that GPIO 9 reads near 0 V with magnet present, ~3.3 V without
4. If inverted, flip the magnet — you have the wrong pole facing the sensor (unless you used an OH090U, which is omnipolar)

### Step 6 — Configure SPIFFS

Connect to the TX via the Web Serial Config Tool or serial at 115200 baud:

```
?set bt_enabled 1
?save
```

Reboot the TX on battery. The BT dot (C7 R1 on the display) should be dark at boot. Hold the magnet against the enclosure — BT dot should start slow blinking within 1 second. Hold 5 s from slow state → fast blink → VESC Tool can now connect.

---

## Magnet Tip

A small disc neodymium magnet (6–8 mm diameter, N35 grade) is ideal. Stick it to the inside of a wetsuit wrist strap or glove cuff — you can activate BLE by touching your wrist to the remote without taking your hand off the throttle.

---

## Troubleshooting

| Symptom | Check |
|---|---|
| BT dot never blinks | `bt_enabled = 1` set and saved? Boot on battery (not USB)? |
| Magnet held but no response | Magnet polarity — try flipping it. Sensor within 5 mm of wall? |
| BT dot blinks constantly without magnet | GPIO 9 floating — check the OUT wire is soldered securely |
| VESC Tool can't find device | BT dot must be fast-blinking, not slow. Hold magnet 5 s past slow blink. |
| No USB boot fix | USB-C power triggers charger mode — always boot on battery for BLE |

---

## Related

- [Hall Sensor Expansion — reference page](Hall_Sensor_Expansion.md)
- [BLE Implementation — protocol details](BLE_Implementation.md)
