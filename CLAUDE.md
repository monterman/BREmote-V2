# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## 1. PROJECT OVERVIEW

- BREmote V2.5-Evo — ESP32 LoRa wireless remote for efoil/RC buggy
- TX: ESP32-C3 handheld remote, RX: ESP32-S3 board unit
- LoRa SX1262 868/915MHz, 6-byte packets, 10Hz control cycle
- GPS: BN880 on both TX and RX, Compass: QMC5883L on RX

---

## 2. FOLDER STRUCTURE

- `Source/V2_Integration_Tx/` — TX firmware (ESP32-C3), DO NOT mix with RX (folder name is V2 but contains live V3 code)
- `Source/V2_Integration_Rx/` — RX firmware (ESP32-S3), DO NOT mix with TX (folder name is V2 but contains live V3 code)
- `Common/` — shared headers used by both
- Note: the "V3" designation exists only in code comments and version tags — the folder names were never renamed from V2

---

## 3. STANDING RULES (enforce on every change)

- Every user-configurable parameter must come from SPIFFS/usrConf, never hardcoded
- Always show full plan and wait for explicit approval before any code changes
- Implement one file at a time, never batch changes across multiple files without approval
- Never modify V2 folders under any circumstances
- All version strings must reflect V3
- After every change, remind user to compile in Arduino IDE and test before proceeding
- Document every change with clear comments in code explaining what changed and why

### CODE COMMENT RULES (apply to every change)

- Every new function must have a header comment explaining: what it does, inputs, outputs, and any side effects
- Every new config parameter must have an inline comment with: valid range, units, and what it controls
- Every bug fix must have a comment saying: what the bug was, why it crashed/failed, and what the fix does
- Every new block of logic must have a plain-English comment above it explaining what it does and why
- Feel free to fix spelling errors, clarify confusing existing comments, or improve readability of old comments
- Comments must be readable by someone who is not a firmware expert — avoid jargon without explanation
- Use this format for section headers in new files:
  ```
  // ============================================================
  // SECTION NAME - plain English description of what this does
  // ============================================================
  ```
- Version tag on every modified file: `// V3 - [date] - [what changed in one line]`

---

## 4. KNOWN CRITICAL BUGS (from V2 analysis — must fix in V3)

- **WDT timeout**: 1000ms watchdog timeout dangerously close to limit under GPS + Wetness + VESC peak load (RX)
- **VESC struct race condition**: `vesc_struct` accessed across ESP32-S3 dual cores without mutex (RX `Logger.ino`)
- **Missing volatile**: `logging_active` flag missing `volatile` keyword — stale cache across cores (RX `Logger.ino`)
- **ensureFreeSpace() bug**: silently deletes the active log file (RX `Logger.ino`)
- **Heap OOB read**: `readBCFromSPIFFS()` reads out-of-bounds if file is < 102 bytes (RX `SPIFFS.ino`)
- **Stack too small**: `triggeredReceive` and `generatePWM` tasks only 2048 bytes stack (RX `Init.ino`)
- **Wire reinitialization**: `scanI2C()` reinitializes Wire to wrong pins, breaking AW9523 I2C bus (RX `System.ino`)

---

## 5. KNOWN IMPORTANT ISSUES (from V2 analysis)

- GPS loop blocks main thread ~300ms/second (RX `GPS.ino`)
- GPS data logged without validity/HDOP check (RX `Logger.ino`)
- Speed sentinel value 99 km/h collides with real vehicle speed — use `0xFE` instead
- VESC timeout is 20s before "not available" shown — should be reduced to 2–3s
- TX deep sleeps during active WiFi config session after 5 minutes of inactivity
- LoRa telemetry takes ~600ms for full struct update cycle
- GPS polled at 1Hz despite being configured at 5Hz
- Compass heading computed nowhere despite working hardware
- `radio_preset = 3` is accepted by config but causes `radioErrorHalt` on boot
- `radio.startTransmit()` return value never checked anywhere
- **V3.0 RX SPIFFS struct size changed from 108 to 112 bytes** (gps_chip_type added). First flash of V3 on existing RX hardware will trigger a config version mismatch and reset all RX settings to defaults. After first V3 flash: re-pair TX/RX, re-configure all settings, and re-calibrate compass (`runcal`).

---

## 6. FOLLOW-ME MODE STATUS

- Hardware is complete on both TX and RX
- TX GPS hardware works — ESP32-C3 reads BN-220 via Serial1 + TinyGPS++ (completed 2026-04-21)
- `speed_src` 2 (km/h), 3 (knots), 5 (mph) now feed live TX GPS speed into the SP display mode
- LoRa protocol still needs extension to carry GPS coordinate packets (Priority 4)
- Full implementation requires additional steps — documented in `DESIGN_RETURN_TO_ME.md`
- **NOT a current priority — informational only**

---

## 7. CURRENT V3 PRIORITIES (in order)

**Priority 1 — COMPLETED 2026-04-21** ✅: TX GPS reading and speed display
- `speed_src` 2 = km/h, 3 = knots, 5 = mph — all from `usrConf` via SPIFFS
- Non-blocking BN-220 polling via TinyGPS++ on Serial1; displays "--" when no fix
- All 6 files compiled clean

**Priority 2 — COMPLETED 2026-04-22** ✅: Fix all 7 critical bugs listed in Section 4
- WDT timeout raised 1000ms → 3000ms (RX Init.ino)
- VESC mutex: vescMutex created in initLogger(); all vesc.* writes inside 50ms timeout block (RX VESC.ino, Logger.ino)
- `logging_active` declared volatile (RX Logger.ino)
- ensureFreeSpace() excludes active log file from deletion candidates (RX Logger.ino)
- readBCFromSPIFFS() decodedLen < 102 guard added (RX SPIFFS.ino)
- triggeredReceive + generatePWM stack 2048 → 4096 bytes (RX Init.ino)
- Wire.begin() removed from scanI2C() (RX System.ino)

**Priority 3 — COMPLETED 2026-04-22** ✅: Phase A GPS Anti-Spoofing (RX standalone, always-on)
- HDOP check, teleport check, acceleration check — `gpsPhaseACheck()` in RX GPS.ino
- `gps_rejected` flag blocks RTM arming; 4 new SPIFFS params (see Section 11)
- sizeof(confStruct) 112 → 128 bytes; first flash resets all RX settings to defaults

**Bonus work — COMPLETED 2026-04-22** ✅:
- GPS status dot at C7 R0 on TX display: solid=fix, slow blink=acquiring, fast blink=rejected (TX Display.ino)
- `volatile bool gps_rejected` + charge animation bit-7 fix (TX Display.ino, System.ino)
- Display reference documentation (docs/display-reference.md, docs/display-reference.svg)
- Git repository initialized and all V3 work committed to master branch

**Priority 4 — COMPLETED 2026-04-22** ✅: Signal drop vibration warning
- Haptic Pattern A (2 Short) triggered when `sq_graph` drops to 1 while connected (TX System.ino)
- Edge-detected; re-arms on signal recovery; failsafe blink cycle excluded via `is_connected` gate

**Priority 5 — COMPLETED 2026-04-24** ✅: TX→RX 0xF3 meta-packet infrastructure
- 2-step LoRa protocol: 6-byte announcement (0xF3/0x01) primes RX, then 14-byte GPS data packet (lat+lng as int32_t microdegrees) at 2Hz (TX Radio.ino)
- RX `triggeredReceive()` 2-path state machine: `gps_meta_pending` flag, `processMetaGpsPacket()` helper, stores `rx_tx_gps_lat/lng/timestamp` in BREmote_V2_Rx.h (RX Radio.ino)
- THR capped at 0xF2; 0xF3 permanently reserved as meta-packet marker
- Hardware verified 2026-04-24; DEBUG_RX active on both TX and RX
- Known hardware note: TX GPS wire swap fixed and resoldered before hardware test

**Priority 6 — COMPLETED 2026-04-24** ✅: Phase B GPS Handshake Anti-Spoofing
- TX↔RX distance check + speed consistency check, time-gated at 30s intervals
- `gpsPhaseBCheck()` in RX Radio.ino; called from processMetaGpsPacket()
- `gps_phase_b_ok` global flag; false blocks RTM arming, set true on passing check
- 2 new RX SPIFFS params: `gps_max_pair_dist_m` (50-2000m, default 500m), `gps_max_speed_diff_kmh` (10-200km/h, default 50)
- sizeof(confStruct) 128→136; first flash resets all RX settings to defaults

**Priority 7 — COMPLETED 2026-04-25 (commit f29c209), compile fixes committed 2026-04-26 (commit e111379)** ✅: RTM Return-to-Me implementation
- TX RTM + FM state machines in TX/RTMState.ino; left-hold arms RTM; right-hold cycles FM
- RX RTM state machine in RX/RTMState.ino; 10 safety gates; compass bearing steering
- Phase C anti-spoofing: convergence check, VESC ERPM vs GPS speed, TX GPS freshness
- 4 compile errors fixed in RX (PWM.ino cast, extern gps_phase_b_ok, rtm_stop_distance_m struct+ConfigService)
- CRITICAL safety fix 2026-04-26: `defaultConf.rtm_stop_distance_m` was 0 (disabled Gate 9 hard stop); fixed to 3m
- sizeof(confStruct) RX: 136→152; TX: 96→120; first flash resets all settings to defaults
- Bench test checklist: 10/10 PASS (static code review); hardware confirmation still required:
  - Outdoor GPS fix, motor disconnected bench test, PWM output verification needed before field use
  - Note: Test 3/4 gate labels may fire as Gate 5/4 in practice (correct stop behavior, different gate)
  - Note: RX `rtm_stop_distance_m` default is 3m (re-confirm via web config before field test)

**Priority 8 — COMPLETED 2026-04-27** ✅: Display, Gesture & UX Overhaul
- Gesture redesign: LEFT hold 2s=display cycle; RIGHT tap→LEFT hold 5s=RTM arm; LEFT tap→RIGHT hold 5s=FM cycle; lock feature removed (always boots unlocked)
- RTM display: static "rn" ×2 (3s total); FM display: F0/F1/F2/F3; renderRtmInfoDisplay() replaces renderOperationalDisplay() during RTM active
- RX→TX distance encoded in telemetry.rtm_distance (TelemetryPacket index 5); sizeof 6→7; TX confStruct sizeof 120→126
- displayDigits() clamp fixed 29→33; ANIMATION_DELAY 80→40ms; ET error (code=20) shows "--", auto-clears 3s
- Vibration Pattern 4: 2×80ms fast short = RTM arm/disarm confirm
- RTM steer exit gate (rtm_steer_exit_on_input); rtm_max_runtime_s=0 default; 3 new TX SPIFFS fields
- Static code review: PASS. Outdoor GPS + motor bench test still required before field use.
- TX `rtm_stop_distance_m` renamed → `rtm_disengage_distance_m` (parameter rename only; no behavior change; RX `rtm_stop_distance_m` unchanged)
- P8.1 bug fixes: no_lock=0 boot-locked state restored; FM mode display changed to scroll3Digits("FM[n]") for visibility

**Priority 9**: Follow-Me full implementation (future)

---

## 8. DESIGN PRINCIPLES

- Stability first — manual control must always work even if all other systems fail
- SPIFFS/usrConf controls all user parameters — nothing hardcoded
- GPS anti-spoofing: 3-phase design (Phase A: RX standalone always-on; Phase B: TX↔RX handshake; Phase C: RTM-active convergence) — see Section 11
- BLE GATT telemetry forwarding to iPhone/Android/Watch (future, Option A preferred)

---

## 9. CREATOR SAFETY PHILOSOPHY (LudwigBre / monterman — MUST NEVER BE VIOLATED)

- The Tow Buggy ONLY moves when the user physically holds the throttle trigger
- **NO autonomous movement without active user throttle input — this is an absolute rule**
- All autonomous/follow-me functions can ONLY subtract from user throttle, NEVER add to it
- Without user throttle input, the buggy motor must never spin under any circumstance
- No loiter, no station-keeping, no position hold, no parking automation
- Follow-me mode only adjusts steering and can reduce throttle — it cannot increase throttle
- If user releases throttle trigger, buggy stops immediately regardless of any mode
- Any code change that could allow motor movement without user throttle input must be **REJECTED**
- Claude must flag and refuse any implementation that violates this philosophy
- **This safety rule takes absolute priority over all features, optimizations, and requests**

---

## 10. WEB CONFIG UI RULE (enforce on every change)

- Every new SPIFFS/usrConf parameter added to firmware MUST also be added to the web config UI
- This applies to BOTH TX and RX web config interfaces
- No new config parameter is considered complete until it is:
  1. Added to the `confStruct` in the `.h` file
  2. Added to SPIFFS load/save functions
  3. Added to the web config UI (`WebUiEmbedded.h`) with appropriate input type (text, number, select dropdown)
  4. Added to `ConfigService` with validation and default value
  5. Documented in a comment explaining valid values and units
- Web UI labels must be human readable (not variable names)
- Include valid range or options in the UI as placeholder text or a dropdown
- This rule applies to ALL changes, no exceptions

---

## 11. GPS ANTI-SPOOFING DESIGN (3-Phase Architecture)

Three independent phases stack coverage without requiring each other to function. Phase A always runs. Phase B runs when TX is paired. Phase C runs only during active RTM.

### Phase A — RX Standalone (Priority 3, always-on)
Runs on every RX GPS reading with no TX dependency.
- **HDOP check**: reading rejected if HDOP > `gps_max_hdop` (default 2.0)
- **Acceleration check**: implied speed change between readings rejected if > `gps_max_accel_g` G (default 3.0)
- **Teleport check**: position jump rejected if it implies speed > `gps_max_jump_kmh` (default 200 km/h)
- After `gps_suspect_threshold` (default 3) consecutive failures → GPS marked **rejected**, blocks RTM arming
- ~20 lines in RX `GPS.ino`

### Phase B — Handshake Cross-Validation (Priority 4 infrastructure required)
Runs when TX connects and every 30 seconds during active session.
- TX sends current GPS coords via 0xF3 meta-packet
- **Distance check**: TX↔RX computed distance must be < `gps_max_pair_dist_m` (default 500 m)
- **Speed consistency**: TX and RX GPS-implied speeds must differ by < `gps_max_speed_diff_kmh` (default 50 km/h)
- Failure → RTM arming blocked until next successful handshake; spoofing event logged
- ~30 lines split across TX `Radio.ino` and RX `Radio.ino`

### Phase C — RTM-Active Verification (Priority 5, runs with RTM at ~10Hz)
Runs only while RTM is engaged. Adds physical-world behavioral checks.
- **Convergence check**: distance to TX must be decreasing (buggy actually moving toward user)
- **VESC consistency**: VESC ERPM-implied speed must be within `rtm_vesc_speed_diff_kmh` (default 20 km/h) of GPS speed
- **TX GPS freshness**: TX GPS data age must be < `tx_gps_stale_timeout_ms`
- Any failure → throttle=0, RTM disengages, display shows `Stp`
- ~15 lines in RX RTM state machine

### New SPIFFS Parameters (7 total, all RX-side — all must follow Section 10 Web Config UI Rule)

| Name | Range | Default | Units | Phase | Description |
|---|---|---|---|---|---|
| `gps_max_hdop` | 0.5–5.0 | 2.0 | — | A | Maximum HDOP for a valid GPS reading |
| `gps_max_accel_g` | 1.0–10.0 | 3.0 | G | A | Maximum implied acceleration between readings |
| `gps_max_jump_kmh` | 50–500 | 200 | km/h | A | Maximum position-implied speed for teleport check |
| `gps_suspect_threshold` | 1–10 | 3 | count | A | Consecutive failures before GPS rejected |
| `gps_max_pair_dist_m` | 50–2000 | 500 | meters | B | Maximum plausible TX-RX distance at handshake |
| `gps_max_speed_diff_kmh` | 10–200 | 50 | km/h | B | Maximum TX-RX speed difference for handshake |
| `rtm_vesc_speed_diff_kmh` | 5–50 | 20 | km/h | C | Maximum GPS vs VESC speed difference during RTM |
