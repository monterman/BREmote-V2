# BREmote V3 — Tow Buggy / eFoil Remote

> **Fork of [BREmote V2](https://github.com/Luddi96/BREmote) by LudwigBre / Luddi96**

[![Original by LudwigBre](https://img.shields.io/badge/Original%20HW%20%26%20FW-LudwigBre%20%2F%20Luddi96-blue)](https://github.com/Luddi96/BREmote)
[![Web Console by Janrusher](https://img.shields.io/badge/Web%20Console%20%26%20Dynamic%20Throttle-Janrusher-green)](https://github.com/Janrusher)
[![V3 by monterman](https://img.shields.io/badge/V3%20GPS%20%2F%20Logger%20%2F%20Analysis-monterman-orange)](https://github.com/monterman)

ESP32 LoRa wireless remote for efoil and RC tow buggy — 868/915 MHz, 10 Hz control cycle, VESC UART telemetry, GPS speed display, integrated data logger.

---

## Credits

BREmote is a collaborative open-source project built by the efoil and esk8 community:

| Contributor | Contribution |
|---|---|
| **[LudwigBre / Luddi96](https://github.com/Luddi96/BREmote)** | **Original hardware design, original firmware architecture, and project founder. All core features originate here.** |
| **Janrusher** | Dynamic throttle cap mode and Web Console foundation — significant V2 enhancements forked from LudwigBre, further refined in V3 |
| **monterman** | V3 firmware: TX GPS implementation, integrated data logger, deep codebase analysis, critical bug documentation, RTM/FM mode design |

This fork exists because LudwigBre published open hardware and firmware under GPL 3.0. V3 enhancements are released under the same license and dedicated back to the community.

---

## What Is This?

BREmote is a custom wireless remote system for efoils and RC tow buggies. The TX (handheld) sends throttle and steering over LoRa at 10 Hz. The RX (mounted on the vehicle) drives the ESC/VESC and returns telemetry.

```
┌────────────────────────────┐              ┌──────────────────────────────────┐
│      TX — Handheld         │              │       RX — Board Unit            │
│                            │              │                                  │
│  ESP32-C3                  │              │  ESP32-S3  (dual-core)           │
│  SX1262 LoRa               │◄──────────►  │  SX1262 LoRa                     │
│  BN-220 GPS (GPIO 18/19)   │  868/915MHz  │  BN-880 GPS  (Serial1 + I2C mux) │
│  Dot matrix display        │    10 Hz     │  QMC5883L Compass  (I2C)         │
│  Hall-effect throttle      │  6-byte pkt  │  AW9523 I/O Expander (I2C)       │
│  Hall-effect toggle        │              │  VESC UART  or  ESC PWM output   │
│  Vibration motor           │              │  Servo steering output           │
│  WiFi AP — web config      │              │  WiFi AP — web config + log DL   │
│                            │              │  Flash Data Logger               │
└────────────────────────────┘              └──────────────────────────────────┘
```

---

## What's New in V3

| Feature | V2 | V3 |
|---|---|---|
| TX GPS speed display (mph / km/h / knots) | ❌ | ✅ |
| GPS speed source SPIFFS-configurable | ❌ | ✅ |
| Integrated flash data logger | ❌ | ✅ |
| Log download over WiFi web UI | ❌ | ✅ |
| US-format log filenames (MMDDYY) | ❌ | ✅ |
| V3 Config Studio offline HTML tool | ❌ | ✅ |
| Full codebase audit + stability fixes | ❌ | ✅ |
| Return-to-Me mode (`rtn`) | ❌ | 🔜 V3.1 |
| Follow-Me mode override | ❌ | 🔜 V3.x |

---

## Hardware Requirements

| Component | TX (Handheld) | RX (Board Unit) |
|---|---|---|
| MCU | ESP32-C3 | ESP32-S3 (dual-core) |
| Radio | SX1262 LoRa | SX1262 LoRa |
| GPS | BN-220, Serial1 GPIO 18/19 | BN-880, Serial1 via I2C mux |
| Compass | None | QMC5883L (I2C) |
| Display | HT16K33 dot matrix (I2C 0x70) | None |
| ADC | ADS1115 (I2C 0x48) | None |
| I/O Expander | None | AW9523 (I2C) |
| ESC / VESC | None | VESC UART or PWM (RMT GPIO 9) |

---

## Quick Start

1. **Flash firmware** — use the Flash Download Tool (link below) or Arduino IDE
2. **Power on both TX and RX** — TX shows `EP` (not paired) on first boot
3. **Pair** — hold RIGHT toggle on TX at boot; hold BIND on RX at boot simultaneously
4. **Connect to WiFi AP** — SSID shown on the device; default password `12345678`
5. **Open V3 Config Studio** — configure all parameters with plain English labels
6. **Calibrate TX** — hold LEFT toggle at boot, follow the display prompts

---

## 🛠️ V3 Config Studio

The **V3 Config Studio** is a standalone HTML tool that works offline in any browser — no install, no server, no dependencies.

**What it does:**
- Configure all TX and RX parameters with plain English labels and valid range hints
- Compare two configurations side by side
- Export config as **JSON** (for web console import) or **Base64** (for serial paste)
- Download and manage RX data logs over WiFi
- Dirty state highlighting — changed fields are highlighted so you know what's unsaved
- Toast notifications on save / load

**How to use:**
1. Download `BREmote_V3_Config_Studio.html` from this repository
2. Open it in any browser — works fully offline
3. Connect your browser to the TX or RX WiFi AP to push / pull config live

> Config Studio replaces the need to memorize parameter names or edit raw Base64 strings. It covers everything the web console does, plus offline editing and log management.

---

## TX Features

### Standard Features (V2)

- Hall-effect throttle with calibration
- Hall-effect toggle for steering and gear/menu input
- Dot matrix display showing telemetry modes
- LoRa packet transmission at 10 Hz
- Gears mode, no-gears mode, dynamic throttle cap mode
- Configurable throttle expo curve
- System lock / unlock sequence
- Vibration motor feedback
- Internal battery voltage monitoring
- USB charging detection and display
- Pairing with address-based authentication
- WiFi AP for web configuration
- Serial USB configuration interface (`?conf`, `?conf json`, `?tasks`, etc.)

### V3 New: TX GPS Speed Display

The **SP** (Speed) telemetry display mode can now read speed directly from the TX GPS module (BN-220 on Serial1), eliminating dependence on the LoRa telemetry round-trip. Configure `speed_src` in Config Studio or the web UI:

| `speed_src` | Source | Unit | Status |
|---|---|---|---|
| 0 | RX GPS | km/h | V2 original |
| 1 | RX GPS | knots | V2 original |
| 2 | TX GPS | km/h | ✅ V3 new |
| 3 | TX GPS | knots | ✅ V3 new |
| 4 | RX GPS | mph | V2 original |
| 5 | TX GPS | mph | ✅ V3 new |

Display shows `--` when no fix is available or the fix is older than the configured stale timeout. Set `gps_en = 1` and reboot after changing it.

**Telemetry display cycle** (cycle with long right toggle press):

```
TH  →  UB  →  TE  →  SP  →  bA
Throttle  Internal Bat  Temp  Speed  Foil Bat
```

### TX Toggle Button Reference

| Input | Result |
|---|---|
| Boot + hold LEFT toggle | Calibration mode — display shows `CA` |
| Boot + hold RIGHT toggle | Pairing mode — display shows `PA` |
| Boot + THR + LEFT toggle | USB charging mode |
| Boot + THR + RIGHT toggle | Delete SPIFFS config (factory reset) |
| LEFT toggle held 2 s | Safety lock (display shows lock icon) |
| RIGHT toggle held 2 s | Cycle telemetry display mode |
| LEFT toggle held 5 s | 🔜 Return-to-Me `rtn` *(V3.1)* |
| RIGHT toggle held 5 s | 🔜 Follow-Me override `FM` *(V3.x)* |

---

## RX Features

### Standard Features (V2)

- VESC UART telemetry (battery %, FET temperature, speed, power)
- Single motor, differential motor, or servo steering output
- PWM output via RMT (GPIO 9) and AW9523 I/O expander
- Water ingress detection with safety stop
- Configurable failsafe time (motor stop on LoRa link loss)
- Foil battery cell count and voltage monitoring
- BMS detection
- GPS positioning (BN-880)
- QMC5883L compass (hardware present, heading computation coming in V3.x)
- Kalman filter on GPS data
- Follow-me mode framework (positional modes: behind, near right, near left)
- WiFi AP for web configuration and log management

### V3 New: Integrated Data Logger

The RX board logs GPS position, VESC telemetry, voltage, speed, and timestamps to on-board flash storage. Enable with `logger_en = 1` in RX Config Studio.

**Using the logger:**

| Action | LED | Meaning |
|---|---|---|
| Press AUX once | 5× flash | Logging started |
| Press AUX once again | 2× flash | Logging stopped |

**Tips:**
- **Wait for GPS lock** before pressing AUX — entries without a valid fix record zero coordinates
- **WiFi auto-disables** while logging to reduce current draw and RF interference with GPS
- **Brownout warning:** the logger auto-stops if supply voltage drops below threshold; WiFi + logging together draw significant current, ensure adequate power supply
- **File format:** `MMDDYY_HHMMSS.csv` (UTC timestamp in filename)
- **Download:** Connect to RX WiFi AP → open V3 Config Studio or the web UI → **Manage Logs** section

---

## 🛡️ Safety Philosophy

> **The Tow Buggy ONLY moves when the user physically holds the throttle trigger.**

This rule is non-negotiable and is enforced at the firmware level — it cannot be configured away:

- Autonomous assist modes can **only subtract from throttle** — they can never add to it
- Releasing the throttle trigger stops the buggy **immediately**, regardless of any active mode
- No loiter, no station-keeping, no position hold, no autonomous parking
- Return-to-Me and Follow-Me can adjust steering and reduce throttle — they cannot independently spin the motor
- Without active user throttle input, the buggy motor **never moves** under any circumstance

---

## 🔜 Coming in V3.1 — Return-to-Me (`rtn`)

> Full design in [DESIGN_RETURN_TO_ME.md](DESIGN_RETURN_TO_ME.md). Hardware ready on both TX and RX.

For when you are floating on your board and want the buggy to steer itself toward you. **You must actively hold the throttle** — RTM provides automatic steering only.

**Activation:**
1. Hold LEFT toggle at full extent for 5 seconds
2. Dot display shows `rtn` — mode armed (10-second window)
3. Double-squeeze the throttle to engage (configurable)
4. Throttle ramps from 30% cap → 70% cap over 5 seconds (all configurable)
5. Release throttle at any time → buggy stops immediately
6. Hard stop when buggy reaches configured safe distance from TX (default 10 m)

**Requires:** GPS fix on both TX and RX, valid compass heading on RX, healthy LoRa link. Any sensor loss immediately cuts throttle to 0 and disengages RTM.

---

## 🔜 Coming in V3.x — Follow-Me Mode Override

> Hardware ready on both TX and RX. Software implementation in progress.

In-session override of the RX follow-me position mode without modifying RX SPIFFS config. The override is RAM-only — RX returns to the web-configured default on reboot.

**Planned activation:** Hold RIGHT toggle 5 seconds → display cycles through `0ff` / `bEh` / `n-R` / `n-L` → release 2 seconds to confirm → TX sends override to RX.

---

## Status / Error Codes

### TX

| Display | Meaning |
|---|---|
| `XX` | Power saver mode active |
| `EP` | Not paired — hold RIGHT toggle at boot to pair |
| `EC` | Not calibrated — hold LEFT toggle at boot |
| `ESV` | Config version error — SPIFFS config incompatible with this firmware version |
| `ESP3` | SPIFFS error |
| `ESP4` | SPIFFS error |
| `EHFC` | LoRa channel error |
| `EHFI` | LoRa init error |
| `EHFP` | LoRa parameter error |
| `ECH` | Charger error |

### RX

| Indicator | Meaning |
|---|---|
| AUX blink 3× | SPIFFS init error |
| AUX blink 2× | Config version error |
| AUX blink 4× | SPIFFS write error |
| BIND — short periodic blink | Not paired |
| BIND — blinking | Paired, not connected |
| BIND — solid | Connected |
| BIND — blink 2× | TX power error |
| BIND — blink 3× | LoRa setting error |
| BIND — blink 4× | LoRa init error |

---

## Startup Input Combinations

### TX

| Input held at boot | Action |
|---|---|
| LEFT toggle | Calibration mode |
| RIGHT toggle | Pairing mode |
| Throttle + LEFT toggle | USB charging mode |
| Throttle + RIGHT toggle | Delete SPIFFS config (factory reset) |

### RX

| Input held at boot | Action |
|---|---|
| BIND button | Pairing mode |
| BIND + AUX buttons | Delete config (factory reset) |

---

## Connection Examples

![VESC with UART](https://github.com/Luddi96/BREmote-V2/raw/main/img/conn_vesc.PNG)

![ESC with BREmote BEC](https://github.com/Luddi96/BREmote-V2/raw/main/img/conn_esc_bbec.PNG)

![ESC with own BEC](https://github.com/Luddi96/BREmote-V2/raw/main/img/conn_esc_obec.PNG)

![VESC + Servo](https://github.com/Luddi96/BREmote-V2/raw/main/img/conn_vesc_servo.PNG)

![ESC + Servo](https://github.com/Luddi96/BREmote-V2/raw/main/img/conn_esc_servo.PNG)

---

## Known Limitations

### Features In Progress

| Feature | Status |
|---|---|
| Return-to-Me (RTM) | Fully designed, implementation not started — see `DESIGN_RETURN_TO_ME.md` |
| Follow-Me mode override | Fully designed, implementation not started |
| TX GPS coords over LoRa | Protocol designed (opcode 0xF3 reserved), not implemented |
| Compass heading output | QMC5883L hardware works on RX, heading not yet computed in firmware |
| BLE telemetry forwarding | Planned for future release |

### Critical Bugs Identified in V2 Codebase (Fixes in Progress for V3)

| # | Bug | File | Impact |
|---|---|---|---|
| 1 | WDT 1000 ms timeout too close to limit under GPS + VESC + wetness peak load | RX `Init.ino` | Unexpected reboot under load |
| 2 | `vesc_struct` accessed across dual ESP32-S3 cores without mutex | RX `Logger.ino` | Data corruption or crash |
| 3 | `logging_active` flag missing `volatile` — stale CPU cache read across cores | RX `Logger.ino` | Logger state mismatch |
| 4 | `ensureFreeSpace()` silently deletes the currently active log file | RX `Logger.ino` | Active log data loss |
| 5 | `readBCFromSPIFFS()` heap out-of-bounds read if file is < 102 bytes | RX `SPIFFS.ino` | Crash on corrupt or short file |
| 6 | `triggeredReceive` and `generatePWM` tasks have 2048-byte stack (too small) | RX `Init.ino` | Stack overflow under load |
| 7 | `scanI2C()` reinitializes Wire to wrong pins, breaking AW9523 I2C bus | RX `System.ino` | I/O expander failure after scan |

---

## Links

- [Original BREmote V2 repository — Luddi96](https://github.com/Luddi96/BREmote)
- [V3 Config Studio — BREmote_V3_Config_Studio.html](Tools/BREmote_V3_Config_Studio.html) *(offline HTML tool)*
- [RTM Design Document — DESIGN_RETURN_TO_ME.md](DESIGN_RETURN_TO_ME.md)
- [Config Tool — lbre.de](https://lbre.de) *(LudwigBre's original web config tool)*
- [Build Video](https://github.com/Luddi96/BREmote) — see original Luddi96 repository
- [SW Setup / Config Video](https://github.com/Luddi96/BREmote) — see original Luddi96 repository
- [Flash Download Tool](https://github.com/Luddi96/BREmote)
- [Expo Tool](https://github.com/Luddi96/BREmote)
- [LUT Creation Tool](https://github.com/Luddi96/BREmote)
- [Plot Digitizer](https://github.com/Luddi96/BREmote)

---

## Changelog

### V3.0.0 — April 2026 *(monterman)*

- Fork established as BREmote V3
- Full codebase audit completed — 7 critical bugs and 10 important issues documented
- TX GPS reading implemented: Beitian BN-220 on Serial1 GPIO 18/19, UBX binary init (115200 baud, 5 Hz), non-blocking polling — does not stall the 10 Hz LoRa cycle
- GPS speed display in km/h, knots, and mph via `speed_src` SPIFFS parameter (values 2, 3, 5)
- All user parameters SPIFFS-configurable via V3 Config Studio — nothing hardcoded
- V3 Config Studio standalone HTML tool created (offline, no install required)
- `DESIGN_RETURN_TO_ME.md` added — RTM mode fully designed with state machines, safety gates, and SPIFFS parameter table
- LoRa meta-packet protocol designed: opcodes 0xF1–0xFE reserved for V3.x autonomous assist features
- SW_VERSION bumped to 3
- `CLAUDE.md` added for AI-assisted development workflow and standing safety rules

### V2.x — *(Janrusher / LudwigBre releases)*

*Full V2 changelog in the original [Luddi96/BREmote](https://github.com/Luddi96/BREmote) repository.*

Key V2 milestones:
- Dynamic throttle cap mode and Web Console (Janrusher, forked from LudwigBre)
- GPS framework, follow-me skeleton, data logger foundation (Janrusher / LudwigBre)
- Initial release: LoRa link, VESC UART, servo steering, gears, water ingress, expo curve (LudwigBre)

---

## Credits

| Contributor | Role |
|---|---|
| **[LudwigBre / Luddi96](https://github.com/Luddi96/BREmote)** | Original hardware design, original firmware architecture, and project founder. All core features originate here. GPL 3.0 author. |
| **Janrusher** | Dynamic throttle cap mode and Web Console foundation — major V2 enhancements forked from LudwigBre, further refined in V3. |
| **monterman** | BREmote V3 — TX GPS implementation, integrated data logger, deep codebase analysis, critical bug documentation, RTM / Follow-Me mode design. |

Logo uses *watersport* and *Skate* by Adrien Coquet from [thenounproject.com](https://thenounproject.com) (CC BY 3.0).

**License:** GNU General Public License v3.0 — same as the original BREmote. See LICENSE file.
