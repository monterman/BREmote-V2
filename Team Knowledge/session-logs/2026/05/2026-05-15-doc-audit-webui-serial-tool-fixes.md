# Session Log — 2026-05-15 Documentation Audit + E71 Bug Fix

**Session:** BREmote V2.5-Evo full documentation audit (3a–3f) + E71 water ingress bug fix  
**Preceding session:** [[2026-05-14-sw55-vesc-fix-display-polish-github-push]]  
**Commits this session:** dc10dd6, 009f5c7, 45e9a5e, bee7b21, 2ce4300 (all pushed to master)

---

## What Was Done

### 3a — README Audit (7 findings fixed, commit dc10dd6)

Full read of README.md confirmed 7 gaps from SW51–SW55 development:

- F1: Status "Pre-Alpha" → "Beta — bench-validated, awaiting first water test"
- F2: Display cycle corrected 6→7 modes; AMP mode was entirely missing; added skip-on-unavailable note
- F3: BLE Known Limitations: "planned future release" → hardware confirmed (HT-CT62 integrated BLE); bt_dot_state + Hall sensor documented; `feature/bluetooth` noted
- F4: SW51–SW55 May 2026 changelog entry added
- F5: New RX Serial Diagnostic Commands table — 15 commands including vescping, vescraw, compasscal, magtest, deletelog
- F6: BT Status Dot (C7 R1) section added — bt_dot_state table, Hall sensor hold-duration state machine
- F7: Both monterman credits updated with SW55 VESC fix, INTBAT, BT dot

### 3a/3b combined — BLE dot in display-reference (commit dc10dd6)

User requested BLE dot added to display-reference.svg below the GPS dot. Full SVG rewrite (760×628→760×772px):
- C7 R1 pixel: blue `#3399ff` with glow filter
- C7 zone label: stacked GPS (green) / BLE (blue)
- New BLE STATUS DOT legend: 3 states (Off / Slow 1s blink / Fast 250ms blink)
- PNG regenerated via Edge headless `--screenshot`

### 3b — display-reference.md Audit (4 findings, commit 009f5c7)

- G1: Display modes table fixed 5→7 modes, added 0xFF sentinel logic
- G2: BLE Status Dot (C7 R1) section added with bt_dot_state table
- G3: Unlock Animation (SW53 paintbrush sweep) documented
- G4: Boot Sequence Timing (SW55, ~4.5s total) documented

### 3c — BLE Documentation

Confirmed complete via 3a F3 + F6 and 3b G2. No separate action needed.

### 3g + Step 4 — Hall Sensor Diagram + VESC Fix Doc (commit 45e9a5e)

- Created `docs/HT-CT62 Hall Sensor Wiring.svg` — DRV5032→HT-CT62 wiring, dark theme, bezier wires, connection table. No wire crossings (pin orders matched on both sides).
- PNG rendered at 760×510px (Edge headless).
- `docs/VESC_Telemetry_Fix.md` updated: status changed to resolved; SW51–SW55 resolution section added (USB-C root cause, 3 code defects, SW54 retry revert, status table for all 5 original findings).

### 3d / 3e / 3f — Commands + WebUI + Serial Tool Audit (commit bee7b21)

**3d findings:**
- `compasscal` and `magtest` were already in README (added in 3a F5 — no gap).
- wifi*, testbg, testpercent confirmed intentionally internal — not documented.

**3e — WebUiEmbedded.h:**
- I1: "41 Parameters for RX" comment updated → 57 (count was stale after GPS/RTM/FM expansion)
- I2: `version` defaults TX 25→26, RX 25→31 (cosmetically stale, now match SW_VERSION)

**3f — Web Serial Config Tool HTML:**
- J1: `fm_warn_distance_m` restored "not yet implemented" label + description (was stripped — made it look like a working feature)
- J2: RX command dropdown: added VESC group (vescping, vescraw), magtest to GPS group, deletelog to Logging group
- J4: TX version def 25→26, RX version def 25→31 (match firmware)

---

## Confirmed Firmware State

- TX `SW_VERSION = 26`, compiled 2026-05-14 18:56:48
- RX `SW_VERSION = 31`, compiled 2026-05-14 18:57:57
- Partition: `huge_app` — TX 39% / RX 40% flash
- Both devices flashed and running

---

## E71 Water Ingress Bug Fix (commit 2ce4300)

User raised Finding 5 (E7 de-latch) after the doc audit. Code audit found the de-latch was already implemented (pulse-and-snooze in System.ino, 2026-05-11) but exposed two new bugs introduced at the same time.

### Bug A — Value mismatch: RX sent 7, TX compared against 71

`checkWetness()` set `telemetry.error_code = 7`. TX `Radio.ino` copied it to `remote_error = 7`. But both the full-screen "E 7" blinking display path and the 5-long emergency vibration (Pattern 3) check `if (remote_error == 71)`. With `remote_error = 7`, neither fired. The user would see a plain `E7` in normal font with no blink and no buzz — completely wrong response to a water ingress event.

**Fix:** `telemetry.error_code = 7` → `= 71` in both the set and auto-clear check in `System.ino`.

### Bug B — TX never cleared remote_error when RX cleared it

The old Radio.ino sync was one-directional:
```cpp
if (telemetry.error_code) { remote_error = telemetry.error_code; }
```
When RX's pulse-and-snooze auto-cleared `error_code = 0` (~10s after alarm), the TX `if` condition was false — nothing happened. `remote_error` stayed 71 on TX until power cycle. This was the exact "requires RX reboot to clear" symptom the user originally reported.

**Fix:** Bidirectional sync scoped to E71 only — does not touch TX-local errors (EP, EC, ESV):
```cpp
if (telemetry.error_code == 71) {
    remote_error = 71;
} else if (remote_error == 71) {
    remote_error = 0;  // RX auto-cleared — mirror it
}
```

### Files changed (commit 2ce4300)

| File | Change |
|---|---|
| `Source/V2_Integration_Rx/System.ino` | `error_code` value 7→71 in set + auto-clear check |
| `Source/V2_Integration_Tx/Radio.ino` | One-way sync → bidirectional sync for E71 |
| `Source/V2_Integration_Rx/BREmote_V2_Rx.h` | VescLogData comment: 7→71 |
| `README.md` | `E 7` added to TX error table; new "Water Ingress Detection (E71)" section with pulse-and-snooze explanation, 5-minute snooze, motor-not-cut, safe-to-ride-back |

### Status after fix

Commit pushed. **Requires Arduino IDE recompile and reflash of both TX and RX** before the fix is active on hardware. SW_VERSION unchanged (no confStruct change). Both devices must be reflashed — the error_code value in the LoRa packet changed from 7 to 71.

---

## Remaining Open Items

| Item | Status |
|---|---|
| Finding 5 — E7 de-latch path | **Fixed** — commit 2ce4300; awaiting reflash |
| Finding 2 — Independent GPS/VESC timers | Partially mitigated (SW55 MUX discipline); independent timers not yet implemented |
| Water test | First water test not yet done — RTM bench-validated only; E71 fix must be flashed first |
| Forum post | Draft ready below — user posts manually (update after E71 reflash) |
| BLE GATT layer | `feature/bluetooth` sprint not yet started |

---

## Forum Post Draft — foil.zone

**Thread:** foil.zone/t/bremote-future-development-topic/20820/298  
**User posts manually after review.**

---

**BREmote V2.5-Evo — VESC fixed, RTM water-ready, BLE telemetry coming**

Quick update from the bench. A lot happened this sprint.

---

**VESC Telemetry — Root Cause Found and Fixed**

The VESC silence mystery is solved. The ESP32-C3's native USB peripheral uses GPIO 18 and 19 — the same pins as Serial1, which is the UART that the hardware MUX routes to both the GPS and VESC. Any USB-C serial cable plugged into the board during field operation overrides Serial1 completely, dropping all VESC UART traffic to zero.

Operational fix: unplug the USB-C cable before going out. No firmware change can eliminate this — it's a hardware constraint of the ESP32-C3.

Three additional firmware defects were found and fixed:

1. **GPS MUX never yielding** — `getGPSLoop()` and `configureGPS()` were switching to GPS on the hardware MUX and never switching back. VESC was left waiting indefinitely after every GPS poll. Fixed: both functions now call `setUartMux(0)` before returning. Boot now starts on the VESC channel.

2. **`rcv_err` flag persistence** — A single stray byte in the UART buffer was poisoning the full 200ms receive window. Fixed: flag removed entirely. CRC handles validation.

3. **SW51/SW52 retry loops reverted** — The retry logic added in those builds caused rapid I2C writes to the AW9523 GPIO expander, corrupting the bus and producing zero-packet VESC responses and GPS chars=0. Reverted to single-call MUX switches, which are reliable when the bus isn't stressed.

---

**Display Updates**

- New **7th display mode**: TX internal battery voltage (UBat label). Cycle is now TEMP → THR → SPEED → POWER → AMP → UBat → BAT. Modes with unavailable data are skipped automatically — VESC modes hide themselves when the ESC is offline.
- **SW53 unlock animation**: the padlock sweep is now a 3-frame paintbrush that fills row by row and stays lit. Boot-to-padlock is about 4.5 seconds total.

---

**RTM Status**

Arming, single-squeeze engage, steering gate (heading controller presets 0–4), approach ramp, hard stop, 9-gate safety chain — all bench-validated. GPS anti-spoofing (Phase A/B/C) operational. **First water test is the next step.** Target: calm flat water, no surf.

---

**BLE — Hardware Confirmed, Development Starting**

The HT-CT62 board has integrated WiFi + BLE + LoRa (SX1262) all in one module. No external Bluetooth hardware needed — the BLE radio is physically present on both TX and RX boards.

The Hall sensor (DRV5032 on GPIO 9) is already wired and working as the BLE activation mechanism: short hold → slow blink (BLE ready), long hold → fast blink (BLE active). The dot shows on the TX display at C7 R1 in blue, below the GPS dot.

BLE telemetry to phone is the next firmware sprint — in-house development, no third-party dependency. Goal: real-time telemetry stream to the Motorola Racer. Branch: `feature/bluetooth` when the sprint starts.

Published a wiring guide for anyone who hasn't installed the Hall sensor yet: `docs/HT-CT62 Hall Sensor Wiring.svg` — three wires, no soldering complexity. If you're installing a GPS at the same time this is the natural moment to add the sensor too.

---

**Flash Instructions**

GitHub: monterman/BREmote-V2 — branch master  
**Partition scheme: `huge_app`** (required — default partition gives 96% usage and won't compile)  
TX flash: 39% / RX flash: 40%

Re-pair and re-calibrate after flashing if coming from a build before SW50.
