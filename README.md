# BREmote V2.5-Evo — Tow Buggy / eFoil Remote

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
| Full codebase audit + stability fixes (7 critical) | ❌ | ✅ |
| GPS anti-spoofing: Phase A (RX standalone) | ❌ | ✅ |
| GPS anti-spoofing: Phase B (TX↔RX handshake) | ❌ | ✅ |
| GPS anti-spoofing: Phase C (RTM convergence) | ❌ | ✅ |
| TX→RX GPS coordinate meta-packets (0xF3) | ❌ | ✅ |
| Return-to-Me mode (RTM) | ❌ | ✅ |
| Follow-Me mode override (FM) | ❌ | ✅ |
| RTM/FM info display (distance/speed on TX) | ❌ | ✅ |

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
| 4 | RX GPS | mph | ✅ V3 new |
| 5 | TX GPS | mph | ✅ V3 new |

Display shows `--` when no fix is available or the fix is older than the configured stale timeout. Set `gps_en = 1` and reboot after changing it.

**Telemetry display cycle** (cycle with LEFT toggle hold 2 s):

```
TH       → UB           → TE    → SP    → BA
Throttle → Internal Bat → Temp  → Speed → Foil Bat
```

### TX Toggle Button Reference — V3 P8 Gestures

| Input | Result |
| --- | --- |
| Boot + hold LEFT toggle | Calibration mode |
| Boot + hold RIGHT toggle | Pairing mode |
| Boot + THR + LEFT toggle | USB charging mode |
| Boot + THR + RIGHT toggle | Delete SPIFFS config (factory reset) |
| LEFT tap (quick) | Arm combo window — next RIGHT hold within 3 s triggers FM |
| RIGHT tap (quick) | Arm combo window — next LEFT hold within 3 s triggers RTM |
| LEFT hold 2 s | Cycle telemetry display mode |
| RIGHT hold 2 s | Reserved — no action |
| RIGHT tap → LEFT hold 5 s | Arm **Return-to-Me** (RTM) — display shows `rn` |
| LEFT tap → RIGHT hold 5 s | Cycle **Follow-Me** override mode (F0/F1/F2/F3) |

> **Note:** The lock feature has been removed in V3 P8. The system always boots unlocked. Throttle must be at 0 for long-press actions to fire.

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

## Return-to-Me (RTM) — Full Guide

> RTM is fully implemented in V3 P7/P8. Hardware: GPS on both TX and RX, compass on RX.

For when you are in the water and want the buggy to drive itself toward you. **You must actively hold the throttle** — RTM provides automatic compass-bearing steering only. Releasing the trigger stops the buggy immediately.

### Arming

1. **Combo gesture:** Quick-tap RIGHT toggle, then within 3 seconds hold LEFT toggle for 5 s
2. TX display shows `rn` for 3 s (two 1.5 s static passes) — armed
3. Haptic: 2 fast short pulses confirm arm

### Engaging

With RTM armed, press throttle to engage (10-second arm window):
- **Single-squeeze mode** (`rtm_double_squeeze_en = 0`): hold throttle >30% for 500 ms
- **Double-squeeze mode** (`rtm_double_squeeze_en = 1`, default): squeeze, release, squeeze again

### Active Operation

- Throttle ramps from `rtm_throttle_start_pct` (default 30%) to `rtm_throttle_max_pct` (default 70%) over `rtm_ramp_duration_s` seconds
- TX display shows distance to TX (in metres) or speed, per `rtm_display_mode`
- RX compass auto-steers toward TX GPS position
- Distance-to-TX shown in tenths of metre below 10 m (e.g. `3.5`), whole metres above

### Disengaging

RTM stops automatically when **any** of these conditions occur:

| Safety Gate | Condition |
|---|---|
| Throttle release | User releases trigger → buggy stops (Gate 1, unconfigurable) |
| Hard stop distance | Buggy within `rtm_stop_distance_m` of TX (default 3 m) |
| GPS lost — TX | TX GPS older than `rtm_gps_timeout_ms` (default 2000 ms) |
| GPS lost — RX | RX GPS older than 6 s |
| GPS rejected | Phase A anti-spoofing failure on RX |
| Handshake failed | Phase B TX↔RX position cross-check failed |
| Throttle idle 10 s | No throttle input for 10 consecutive seconds while active |
| LoRa link lost | No packet for failsafe timeout |
| Max runtime | If `rtm_max_runtime_s > 0` (default: 0 = disabled) |
| Convergence fail | Distance to TX not decreasing (Phase C, checked every 5 s) |
| Steering input | Steering override while `rtm_steer_exit_on_input = 1` (default) |

On any gate failure: throttle → 0, TX display shows `Stp` for 2 s, haptic confirms disarm.

### SPIFFS Configuration (TX)

| Parameter | Default | Description |
|---|---|---|
| `rtm_enabled` | 1 | Master on/off switch |
| `rtm_throttle_start_pct` | 30 | Initial throttle cap % when RTM engages |
| `rtm_throttle_max_pct` | 70 | Max throttle cap % after ramp |
| `rtm_ramp_duration_s` | 5 | Ramp time start→max in seconds |
| `rtm_arm_window_s` | 10 | Seconds to engage throttle after arming |
| `rtm_double_squeeze_en` | 1 | 1=double-squeeze, 0=hold 500ms |
| `rtm_disengage_distance_m` | 10 | TX-side disengage distance in metres |
| `rtm_gps_timeout_ms` | 2000 | TX GPS stale timeout in ms |
| `rtm_max_runtime_s` | 0 | Max runtime (0 = disabled) |
| `rtm_display_mode` | 0 | 0=distance, 1=speed, 2=alternating |
| `rtm_steer_exit_on_input` | 1 | 1=steering exits RTM, 0=correction blend |

### SPIFFS Configuration (RX)

| Parameter | Default | Description |
|---|---|---|
| `rtm_rx_enabled` | 1 | RX-side RTM enable |
| `rtm_rx_override_steering` | 1 | Allow RX to auto-steer using compass |
| `rtm_compass_required` | 1 | Require valid compass or stop |
| `rtm_stop_distance_m` | 3 | RX-side hard stop distance |
| `rtm_vesc_speed_diff_kmh` | 20 | Max VESC vs GPS speed diff (Phase C) |
| `vesc_erpm_per_kmh` | 0 | VESC ERPM per km/h for speed check (0=disabled) |

---

## Follow-Me Mode Override (FM) — Full Guide

> FM override is fully implemented in V3 P7/P8. It overrides the RX follow-me positioning mode at runtime without a SPIFFS write.

The override is RAM-only — RX returns to its web-configured `followme_mode` on reboot.

### Activation

1. **Combo gesture:** Quick-tap LEFT toggle, then within 3 seconds hold RIGHT toggle for 5 s
2. TX display shows `F` + mode number (e.g. `F0`, `F1`, `F2`, `F3`)
3. Continue holding RIGHT or re-hold within 2 s to keep cycling modes
4. Release and wait 2 s — TX sends the selected mode to RX via 0xF2 meta-packet

### Modes

| Display | `followme_mode` value | Behaviour |
|---|---|---|
| `F0` | 0 | Disabled — follow-me off |
| `F1` | 1 | Behind — RX follows directly behind TX |
| `F2` | 2 | Near Right — RX follows to the right of TX |
| `F3` | 3 | Near Left — RX follows to the left of TX |

### FM Proximity Warning

If TX-to-RX distance drops below `fm_warn_distance_m` (default 150 m), TX fires a 2×Pattern-2 vibration burst warning (2 short × 2, with 300 ms gap).

### SPIFFS Configuration (TX)

| Parameter | Default | Description |
|---|---|---|
| `fm_override_enabled` | 1 | Master on/off switch |
| `fm_warn_distance_m` | 150 | Proximity warning threshold in metres |

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

### Features Still Pending

| Feature | Status |
|---|---|
| Follow-Me full implementation | FM override operational; full autonomous follow-me behaviour (positional control loop) not yet implemented |
| BLE telemetry forwarding | Planned for future release |
| RTM/FM hardware field test | Static code review passed (10/10 gates). Outdoor GPS + motor bench test still required before field use. |

### V2 Critical Bugs — All Fixed in V3

| # | Bug | Fix in V3 |
|---|---|---|
| 1 | WDT 1000 ms timeout too close under load | Raised to 3000 ms (RX `Init.ino`) |
| 2 | `vesc_struct` race condition across ESP32-S3 cores | `vescMutex` semaphore added (RX `VESC.ino`, `Logger.ino`) |
| 3 | `logging_active` missing `volatile` | Declared `volatile` (RX `Logger.ino`) |
| 4 | `ensureFreeSpace()` deletes active log file | Excludes active log from deletion (RX `Logger.ino`) |
| 5 | `readBCFromSPIFFS()` heap out-of-bounds on short file | `decodedLen < 102` guard added (RX `SPIFFS.ino`) |
| 6 | `triggeredReceive` / `generatePWM` stack 2048 bytes | Raised to 4096 bytes (RX `Init.ino`) |
| 7 | `scanI2C()` reinitializes Wire to wrong pins | `Wire.begin()` removed from `scanI2C()` (RX `System.ino`) |

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

### V3 P8 — April 2026 *(monterman)* — Display, Gesture & UX Overhaul

- **Gesture redesign (breaking):** LEFT hold 2s = cycle display; RIGHT tap→LEFT hold 5s = RTM arm; LEFT tap→RIGHT hold 5s = FM cycle; lock feature removed (always boots unlocked)
- **RTM arm/disarm display:** static `rn` ×2 (3s total), replaces scrolling `rtn`
- **FM mode display:** `F0`/`F1`/`F2`/`F3` instead of named abbreviations
- **RTM/FM active info display:** TX shows distance-to-TX or speed on dot matrix while RTM/FM active; `rtm_display_mode` configures mode (0=distance, 1=speed, 2=alternating 2.5s); distance shown as `X.X` m below 10m with C3 decimal dot, `XX` m above
- **RX→TX distance telemetry:** RX computes and encodes distance into `telemetry.rtm_distance` (TelemetryPacket index 5); TX decodes for display
- **Vibration Pattern 4:** 2×80ms fast short pulses for RTM arm/disarm confirmation
- **RTM steer exit gate:** steering input exits RTM when `rtm_steer_exit_on_input=1` (default)
- **`rtm_max_runtime_s` default 120→0** (0=disabled; safety gates handle all real scenarios)
- **`displayDigits()` clamp bug fixed:** clamp raised 29→33; LET_R/LET_N/LET_S/LET_M now display correctly (was silently rendering as blank)
- **Unlock animation 2× faster:** ANIMATION_DELAY 80ms→40ms
- **ET error handler:** remote_error=20 (LET_T) shows `--` and auto-clears after 3s; no vibration
- **3 new TX SPIFFS fields:** `rtm_display_mode`, `fm_warn_distance_m`, `rtm_steer_exit_on_input`
- **TelemetryPacket grows:** `rtm_distance` added at index 5; `link_quality` moved to index 6 (sizeof 6→7); TX+RX bounds-check auto-adapts
- **confStruct sizeof:** TX 120→126, RX unchanged. First P8 flash resets TX settings to defaults.

### V3.0.0 — April 2026 *(monterman)*

- Fork established as BREmote V2.5-Evo
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
| **monterman** | BREmote V2.5-Evo — TX GPS implementation, integrated data logger, deep codebase analysis, critical bug documentation, RTM / Follow-Me mode design. |

Logo uses *watersport* and *Skate* by Adrien Coquet from [thenounproject.com](https://thenounproject.com) (CC BY 3.0).

**License:** GNU General Public License v3.0 — same as the original BREmote. See LICENSE file.
