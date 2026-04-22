# GPS Phase A Anti-Spoofing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add Phase A GPS anti-spoofing to the RX firmware — three always-on checks (HDOP, teleport, acceleration) that reject implausible GPS readings and block RTM arming after consecutive failures.

**Architecture:** All logic lives in `GPS.ino`. New config params are added to `confStruct`, `kCfgFields`, and the Web UI. State is kept in file-scope variables in `GPS.ino`. The struct size changes from 112 → 128 bytes, triggering an automatic SPIFFS reset on first flash.

**Tech Stack:** ESP32-S3 / Arduino, TinyGPS++ 1.0.3, FreeRTOS, SPIFFS JSON config engine

---

## Files Modified

| File | Change |
|---|---|
| `Source/V2_Integration_Rx/BREmote_V2_Rx.h` | Add 4 fields to confStruct; update static_assert 112→128; update defaultConf |
| `Source/V2_Integration_Rx/ConfigService.ino` | Add 4 entries to kCfgFields[] |
| `Source/V2_Integration_Rx/WebUiEmbedded.h` | Add 4 fields to JS `fields` array; update param count comment |
| `Source/V2_Integration_Rx/GPS.ino` | Add state vars + gpsPhaseACheck() + modify getGPSLoop(); update file header |

**No other files are modified.** TX firmware is NOT touched.

---

## IMPORTANT: Struct Size Change Notice

Adding 4 new fields (3 floats + 1 uint16_t) changes `sizeof(confStruct)` from **112 → 128 bytes**.

Byte layout after new fields (fields added at tail, after `gps_chip_type`):
```
gps_chip_type   @ offset 108, 2 bytes  (existing)
[2 bytes internal padding — was tail padding, now pushes float to 4-byte boundary]
gps_max_hdop    @ offset 112, 4 bytes  (float)
gps_max_accel_g @ offset 116, 4 bytes  (float)
gps_max_jump_kmh@ offset 120, 4 bytes  (float)
gps_suspect_threshold @ offset 124, 2 bytes  (uint16_t)
[2 bytes tail padding to align struct to 4 bytes]
                                         TOTAL = 128 bytes
```

On first flash of this firmware onto existing hardware:
- SPIFFS detects sizeof mismatch (was 112, now 128) → **resets ALL RX settings to defaults**
- After flashing: re-configure all settings via web UI, re-pair TX/RX, re-run `runcal`

---

## Task 1: Add 4 fields to confStruct — `BREmote_V2_Rx.h`

> CLAUDE.md Rule: one file at a time. Do NOT touch any other file until this task is committed.

**Files:**
- Modify: `Source/V2_Integration_Rx/BREmote_V2_Rx.h`

### Step 1.1 — Add fields to confStruct

Find the end of confStruct (just before the closing `};`). The last field is currently:

```cpp
    uint16_t gps_chip_type;  // 0=BN-220, 1=BN-880+compass (default), 2=M10 no compass, 3=M10+compass; range 0-3
};
```

Replace it with:

```cpp
    uint16_t gps_chip_type;  // 0=BN-220, 1=BN-880+compass (default), 2=M10 no compass, 3=M10+compass; range 0-3

    // ============================================================
    // V3 - 2026-04-22 - PHASE A GPS ANTI-SPOOFING PARAMETERS
    //
    // These four parameters control the always-on Phase A anti-
    // spoofing filter in GPS.ino. A reading is rejected if ANY
    // check fails. After gps_suspect_threshold consecutive
    // rejections, gps_rejected is set and RTM arming is blocked.
    //
    // !!! Adding these fields changes sizeof(confStruct) 112→128. !!!
    // !!! On first V3.1 boot, SPIFFS resets ALL settings to       !!!
    // !!! defaults. After flashing: re-pair TX/RX, re-configure   !!!
    // !!! all settings via web UI, re-run runcal.                 !!!
    // ============================================================
    float    gps_max_hdop;            // Max HDOP for a valid fix; range 0.5-5.0; default 2.0; dimensionless
    float    gps_max_accel_g;         // Max implied acceleration between readings; range 1.0-10.0G; default 3.0G
    float    gps_max_jump_kmh;        // Max position-implied speed (teleport check); range 50-500 km/h; default 200
    uint16_t gps_suspect_threshold;   // Consecutive failures before GPS marked rejected; range 1-10; default 3
};
```

### Step 1.2 — Update static_assert

Find:
```cpp
static_assert(sizeof(confStruct) == 112, "confStruct size mismatch — expected 112 bytes (V3). Update this assert and SPIFFS migration logic if you change the struct.");  // V3 fix (N-1): pinned to exact size; catches both shrinkage and unexpected growth
```

Replace with:
```cpp
static_assert(sizeof(confStruct) == 128, "confStruct size mismatch — expected 128 bytes (V3.1). Update this assert and SPIFFS migration logic if you change the struct.");  // V3 fix (N-1): pinned to exact size; catches both shrinkage and unexpected growth. 112→128 when Phase A anti-spoofing params added 2026-04-22.
```

### Step 1.3 — Update defaultConf

Find the current end of defaultConf:
```cpp
  0, 0,      // mag_offset_x, mag_offset_y (no compass bias correction by default)
  1.0f, 1.0f, // mag_scale_x, mag_scale_y (unity gain — run 'runcal' to calibrate)
  // V3 - 2026-04-22 - GPS chip type: 1 = BN-880 (GPS+compass). RX default.
  1           // gps_chip_type (1 = BN-880 + compass; run 'runcal' after first boot)
};
```

Replace with:
```cpp
  0, 0,      // mag_offset_x, mag_offset_y (no compass bias correction by default)
  1.0f, 1.0f, // mag_scale_x, mag_scale_y (unity gain — run 'runcal' to calibrate)
  // V3 - 2026-04-22 - GPS chip type: 1 = BN-880 (GPS+compass). RX default.
  1,          // gps_chip_type (1 = BN-880 + compass; run 'runcal' after first boot)
  // V3 - 2026-04-22 - Phase A GPS anti-spoofing defaults (see CLAUDE.md Section 11)
  2.0f,       // gps_max_hdop:           max HDOP for valid reading (range 0.5-5.0)
  3.0f,       // gps_max_accel_g:        max implied acceleration (range 1.0-10.0 G)
  200.0f,     // gps_max_jump_kmh:       max teleport-implied speed (range 50-500 km/h)
  3           // gps_suspect_threshold:  consecutive failures before GPS rejected (range 1-10)
};
```

### Step 1.4 — Update file header comment

Find:
```cpp
// V3 - 2026-04-22 - Added gps_chip_type field to confStruct (GPS module selector); sizeof 108→112; updated defaultConf
```

Replace with:
```cpp
// V3 - 2026-04-22 - Added gps_chip_type field to confStruct (GPS module selector); sizeof 108→112; updated defaultConf
// V3 - 2026-04-22 - Added Phase A GPS anti-spoofing params to confStruct; sizeof 112→128; updated defaultConf
```

### Step 1.5 — Verify by compiling

Open Arduino IDE, select ESP32-S3 board, compile. Expected result:
- **0 errors**
- The static_assert on sizeof == 128 must pass silently (any mismatch produces a hard compile error)
- If the size is wrong, the error message will state the actual size — use that to fix the assert value

### Step 1.6 — Commit

```
git add "Source/V2_Integration_Rx/BREmote_V2_Rx.h"
git commit -m "feat(RX): add Phase A anti-spoofing params to confStruct; sizeof 112→128"
```

---

## Task 2: Add config service entries — `ConfigService.ino`

> Wait for Task 1 compile to succeed before starting this task.

**Files:**
- Modify: `Source/V2_Integration_Rx/ConfigService.ino`

### Step 2.1 — Add 4 entries to kCfgFields[]

Find the end of the kCfgFields array. The current last few entries are:

```cpp
  // V3 - 2026-04-22 - GPS chip type: 0=BN-220, 1=BN-880+compass (RX default), 2=M10, 3=M10+compass
  {"gps_chip_type", CFG_U16, offsetof(confStruct, gps_chip_type), true, false, true, 0.0f, 3.0f, 0, false},
  {"logger_en", CFG_U16, offsetof(confStruct, logger_en), true, false, true, 0.0f, 1.0f, 0, false},
```

Insert the 4 new entries immediately after the `gps_chip_type` line and before `logger_en`:

```cpp
  // V3 - 2026-04-22 - GPS chip type: 0=BN-220, 1=BN-880+compass (RX default), 2=M10, 3=M10+compass
  {"gps_chip_type", CFG_U16, offsetof(confStruct, gps_chip_type), true, false, true, 0.0f, 3.0f, 0, false},
  // V3 - 2026-04-22 - Phase A GPS anti-spoofing parameters (see CLAUDE.md Section 11)
  {"gps_max_hdop",           CFG_FLOAT, offsetof(confStruct, gps_max_hdop),           true, false, true,  0.5f,  5.0f, 1, false},
  {"gps_max_accel_g",        CFG_FLOAT, offsetof(confStruct, gps_max_accel_g),        true, false, true,  1.0f, 10.0f, 1, false},
  {"gps_max_jump_kmh",       CFG_FLOAT, offsetof(confStruct, gps_max_jump_kmh),       true, false, true, 50.0f,500.0f, 1, false},
  {"gps_suspect_threshold",  CFG_U16,   offsetof(confStruct, gps_suspect_threshold),  true, false, true,  1.0f, 10.0f, 0, false},
  {"logger_en", CFG_U16, offsetof(confStruct, logger_en), true, false, true, 0.0f, 1.0f, 0, false},
```

### Step 2.2 — Update file header comment

Find:
```cpp
// V3 - 2026-04-22 - Added gps_chip_type field (GPS module selector: 0=BN-220, 1=BN-880+compass, 2=M10, 3=M10+compass)
```

Replace with:
```cpp
// V3 - 2026-04-22 - Added gps_chip_type field (GPS module selector: 0=BN-220, 1=BN-880+compass, 2=M10, 3=M10+compass)
// V3 - 2026-04-22 - Added Phase A GPS anti-spoofing fields: gps_max_hdop, gps_max_accel_g, gps_max_jump_kmh, gps_suspect_threshold
```

### Step 2.3 — Compile

Compile in Arduino IDE. Expected: **0 errors**.

### Step 2.4 — Commit

```
git add "Source/V2_Integration_Rx/ConfigService.ino"
git commit -m "feat(RX): register Phase A anti-spoofing params in config service"
```

---

## Task 3: Add Web UI fields — `WebUiEmbedded.h`

> Wait for Task 2 compile to succeed before starting this task.

**Files:**
- Modify: `Source/V2_Integration_Rx/WebUiEmbedded.h`

### Step 3.1 — Update the parameter count comment

Find (near line 108):
```
// 35 Parameters for RX
```

Replace with:
```
// 40 Parameters for RX
```

### Step 3.2 — Add 4 fields to the JS `fields` array

Find the existing `tx_gps_stale_timeout_ms` line in the fields array:
```javascript
{key:"tx_gps_stale_timeout_ms",label:"TX GPS Stale Timeout",description:"TX GPS data stale timeout",group:"GPS & Follow-Me",type:"int",def:1000,min:0,max:65535,unit:"ms"},
```

Insert the 4 new anti-spoofing fields immediately after it (still in the "GPS & Follow-Me" group):

```javascript
{key:"tx_gps_stale_timeout_ms",label:"TX GPS Stale Timeout",description:"TX GPS data stale timeout",group:"GPS & Follow-Me",type:"int",def:1000,min:0,max:65535,unit:"ms"},
{key:"gps_max_hdop",label:"GPS Max HDOP",description:"Maximum HDOP for a valid GPS fix — lower is stricter. Range 0.5-5.0, default 2.0. Readings above this threshold are rejected.",group:"GPS & Follow-Me",type:"float",def:2.0,min:0.5,max:5.0,step:0.1},
{key:"gps_max_accel_g",label:"GPS Max Acceleration",description:"Maximum implied acceleration between consecutive GPS readings (G-force). Range 1.0-10.0 G, default 3.0 G. Higher-than-max implies a spoofed jump.",group:"GPS & Follow-Me",type:"float",def:3.0,min:1.0,max:10.0,step:0.1,unit:"G"},
{key:"gps_max_jump_kmh",label:"GPS Max Teleport Speed",description:"Maximum speed implied by position change between readings. Range 50-500 km/h, default 200 km/h. Larger implies GPS teleport.",group:"GPS & Follow-Me",type:"float",def:200.0,min:50.0,max:500.0,step:1.0,unit:"km/h"},
{key:"gps_suspect_threshold",label:"GPS Suspect Threshold",description:"Consecutive anti-spoofing failures before GPS is marked rejected. Range 1-10, default 3. While rejected, RTM arming is blocked.",group:"GPS & Follow-Me",type:"int",def:3,min:1,max:10},
```

### Step 3.3 — Compile

Compile in Arduino IDE. Expected: **0 errors**.

### Step 3.4 — Commit

```
git add "Source/V2_Integration_Rx/WebUiEmbedded.h"
git commit -m "feat(RX): add Phase A anti-spoofing params to web config UI"
```

---

## Task 4: Phase A anti-spoofing logic — `GPS.ino`

> Wait for Task 3 compile to succeed before starting this task.

**Files:**
- Modify: `Source/V2_Integration_Rx/GPS.ino`

### Step 4.1 — Update file header comment

Find:
```cpp
// V3 - 2026-04-22 - Added gps_chip_type branch: type 0/1=BN-220/BN-880 (9600→115200, 5Hz), type 2/3=M10 (115200 direct, 10Hz, all constellations)
```

Replace with:
```cpp
// V3 - 2026-04-22 - Added gps_chip_type branch: type 0/1=BN-220/BN-880 (9600→115200, 5Hz), type 2/3=M10 (115200 direct, 10Hz, all constellations)
// V3 - 2026-04-22 - Added Phase A GPS anti-spoofing: HDOP check, teleport check, acceleration check (gpsPhaseACheck)
```

### Step 4.2 — Update the Field Service Note

Find the existing FIELD SERVICE NOTE block (lines 3-16):

```cpp
// ============================================================
// FIELD SERVICE NOTE
//
// V3 - 2026-04-22: sizeof(confStruct) changed from 108 to 112
// bytes (gps_chip_type added). On the first V3 flash, SPIFFS
// will detect the size mismatch and reset ALL settings to
// defaults. After flashing, you must:
//   1) Re-pair TX and RX
//   2) Re-configure all settings via the web UI
//   3) Re-calibrate compass via the 'runcal' serial command
//
// Also verify that gps_chip_type in the web config matches
// the physical GPS module connected to this board.
// ============================================================
```

Replace with:

```cpp
// ============================================================
// FIELD SERVICE NOTE
//
// V3 - 2026-04-22: sizeof(confStruct) changed from 108 to 112
// bytes (gps_chip_type added). On the first V3 flash, SPIFFS
// will detect the size mismatch and reset ALL settings to
// defaults. After flashing, you must:
//   1) Re-pair TX and RX
//   2) Re-configure all settings via the web UI
//   3) Re-calibrate compass via the 'runcal' serial command
//
// V3 - 2026-04-22: sizeof(confStruct) changed from 112 to 128
// bytes (Phase A anti-spoofing params added). On the first
// flash after this change, SPIFFS will again detect the size
// mismatch and reset ALL settings to defaults. After flashing:
//   1) Re-pair TX and RX
//   2) Re-configure all settings via the web UI
//   3) Re-calibrate compass via the 'runcal' serial command
//   4) Verify anti-spoofing defaults in "GPS & Follow-Me"
//      section of the web UI (HDOP 2.0, Accel 3.0G, Jump 200
//      km/h, Threshold 3) — adjust if needed.
//
// Also verify that gps_chip_type in the web config matches
// the physical GPS module connected to this board.
// ============================================================
```

### Step 4.3 — Add Phase A state variables

Add the following block immediately before the `configureGPS()` function (after the Field Service Note and before the `configureGPS` comment block):

```cpp
// ============================================================
// PHASE A GPS ANTI-SPOOFING STATE
// File-scope variables that persist between getGPSLoop() calls.
// These are NOT static so the RTM state machine (GPS.ino future)
// can read gps_rejected without an accessor function.
// ============================================================
uint8_t      gps_suspect_count  = 0;     // consecutive suspicious readings; resets on any clean reading
bool         gps_rejected       = false; // true = GPS marked rejected; blocks RTM arming
double       gps_last_lat       = 0.0;   // last accepted latitude (degrees)
double       gps_last_lng       = 0.0;   // last accepted longitude (degrees)
float        gps_last_speed_kmh = 0.0;   // last accepted speed (km/h)
unsigned long gps_last_ms       = 0;     // millis() timestamp of last accepted reading
```

### Step 4.4 — Add gpsPhaseACheck() function

Add this function immediately before `configureGPS()` (after the state variable block added in Step 4.3):

```cpp
// ============================================================
// gpsPhaseACheck - Phase A GPS anti-spoofing validation
// ============================================================
//
// What it does:
//   Validates one GPS reading against three independent checks:
//
//   1) HDOP check: reject if HDOP > usrConf.gps_max_hdop
//      (poor satellite geometry = untrustworthy position)
//
//   2) Teleport check: reject if the position change since the
//      last accepted reading implies travel faster than
//      usrConf.gps_max_jump_kmh km/h (physically impossible)
//
//   3) Acceleration check: reject if the speed change since the
//      last accepted reading implies acceleration > 
//      usrConf.gps_max_accel_g G (physically impossible for a
//      ground vehicle)
//
//   Checks 2 and 3 are skipped on the very first accepted
//   reading (gps_last_ms == 0) because there is no history to
//   compare against.
//
// Inputs:
//   cur_lat, cur_lng - current GPS position (degrees, from TinyGPS++)
//   cur_speed_kmh    - current GPS speed (km/h, from gps.speed.kmph())
//
// Returns:
//   true  = all checks passed (safe to accept this reading)
//   false = at least one check failed (treat as suspicious)
//
// Side effects:
//   Reads module-level state: gps_last_lat, gps_last_lng,
//   gps_last_ms, gps_last_speed_kmh.
//   Reads usrConf.gps_max_hdop, gps_max_jump_kmh, gps_max_accel_g.
//   Prints diagnostics to Serial when a check fails.
// ============================================================
static bool gpsPhaseACheck(double cur_lat, double cur_lng, float cur_speed_kmh) {
  // ---- Check 1: HDOP ----
  // TinyGPS++ gps.hdop.value() returns HDOP * 100 as an integer.
  // Divide by 100.0 to get the real HDOP float for comparison.
  if (gps.hdop.isValid() && (float)gps.hdop.value() / 100.0f > usrConf.gps_max_hdop) {
    Serial.printf("GPS [PhA] HDOP %.1f exceeds max %.1f — reading rejected\n",
                  (float)gps.hdop.value() / 100.0f, usrConf.gps_max_hdop);
    return false;
  }

  // ---- Checks 2 & 3: require at least one prior accepted reading ----
  if (gps_last_ms > 0) {
    float dt_s = (float)(millis() - gps_last_ms) / 1000.0f;

    // Guard against near-zero dt (duplicate call, millis() wrap) to avoid division by zero.
    if (dt_s > 0.05f) {

      // ---- Check 2: teleport ----
      // TinyGPSPlus::distanceBetween() returns metres between two lat/lng pairs.
      float dist_m = (float)TinyGPSPlus::distanceBetween(
                              gps_last_lat, gps_last_lng, cur_lat, cur_lng);
      float implied_kmh = (dist_m / dt_s) * 3.6f;
      if (implied_kmh > usrConf.gps_max_jump_kmh) {
        Serial.printf("GPS [PhA] Teleport %.0f km/h exceeds max %.0f km/h — reading rejected\n",
                      implied_kmh, usrConf.gps_max_jump_kmh);
        return false;
      }

      // ---- Check 3: acceleration ----
      // Convert speed delta from km/h to m/s, then divide by dt to get m/s², then by 9.81 for G.
      float delta_v_ms = fabsf(cur_speed_kmh - gps_last_speed_kmh) / 3.6f;
      float accel_g    = (delta_v_ms / dt_s) / 9.81f;
      if (accel_g > usrConf.gps_max_accel_g) {
        Serial.printf("GPS [PhA] Accel %.2f G exceeds max %.2f G — reading rejected\n",
                      accel_g, usrConf.gps_max_accel_g);
        return false;
      }
    }
  }

  return true;  // all checks passed
}
```

### Step 4.5 — Replace the getGPSLoop() speed-output block

The current getGPSLoop() ends with this block (after the `while (millis() - startTime < 300)` loop):

```cpp
  if (!newData)
  {
    telemetry.foil_speed = 0xFF;
  }
  else if(!gps.speed.isUpdated())
  {
    telemetry.foil_speed = 0xFF;  // V3 fix (N-4): 99 km/h collides with real vehicle speed; 0xFF is the established "no data" sentinel (CLAUDE.md Section 5)
  }
  else
  {
    telemetry.foil_speed = (uint8_t)gps.speed.kmph();
  }
```

Replace it with:

```cpp
  if (!newData || !gps.location.isValid() || !gps.speed.isUpdated()) {
    // No valid fix or no updated speed — not a spoof event, just no usable data.
    telemetry.foil_speed = 0xFF;  // 0xFF = no data (V3 fix N-4: 99 collides with real speed)
  } else {
    double cur_lat     = gps.location.lat();
    double cur_lng     = gps.location.lng();
    float  cur_speed   = (float)gps.speed.kmph();

    if (gpsPhaseACheck(cur_lat, cur_lng, cur_speed)) {
      // Reading passed all Phase A checks — accept it.
      // Reset suspicion state and update the "last known good" snapshot.
      gps_suspect_count  = 0;
      gps_rejected       = false;
      gps_last_lat       = cur_lat;
      gps_last_lng       = cur_lng;
      gps_last_speed_kmh = cur_speed;
      gps_last_ms        = millis();
      // Cap at 254: 0xFF (255) is the reserved "no data" sentinel.
      telemetry.foil_speed = (cur_speed >= 254.0f) ? 254 : (uint8_t)cur_speed;
    } else {
      // Reading failed at least one Phase A check — track consecutive failures.
      if (gps_suspect_count < 255) gps_suspect_count++;
      if (gps_suspect_count >= usrConf.gps_suspect_threshold && !gps_rejected) {
        gps_rejected = true;
        Serial.printf("GPS [PhA] REJECTED after %u consecutive failures — RTM arming blocked\n",
                      (unsigned)gps_suspect_count);
      }
      // Do not expose spoofed/suspicious data via telemetry.
      telemetry.foil_speed = 0xFF;
    }
  }
```

Also remove the dead commented-out code block that follows (the old `/* if (newData && gps.speed.isUpdated()) { ... } */` block at the end of getGPSLoop()):

```cpp
/*
  // If we received valid GPS data
  if (newData && gps.speed.isUpdated()) 
  {
    telemetry.foil_speed = (uint8_t)gps.speed.kmph();
  } 
  else 
  {
    //telemetry.foil_speed = 0xFF;
  }*/
```

Delete that entire block — it is dead code and the anti-spoofing logic above replaces it.

### Step 4.6 — Compile

Compile in Arduino IDE. Expected: **0 errors**.

Check Serial monitor output after flashing to confirm:
- On a valid GPS fix with good HDOP: no `[PhA]` messages → reading accepted
- On first boot without GPS antenna: `foil_speed = 0xFF` reported (no fix = no data)
- Anti-spoofing trigger can be tested by temporarily lowering `gps_max_hdop` to 0.5 in the web UI — any marginal HDOP should trigger `GPS [PhA] HDOP ... exceeds max` on Serial

### Step 4.7 — Commit

```
git add "Source/V2_Integration_Rx/GPS.ino"
git commit -m "feat(RX): implement Phase A GPS anti-spoofing (HDOP/teleport/accel checks)"
```

---

## Task 5: End-to-end Verification

### Step 5.1 — Compile full RX sketch clean

In Arduino IDE:
- Select board: ESP32-S3
- Select correct COM port
- Click **Verify** (not Upload) — expected: **0 errors, 0 warnings (or only known WiFi/BLE stack warnings)**

### Step 5.2 — Flash and check Serial

Flash firmware to RX board. Open Serial Monitor at 115200 baud. Expected output sequence:

1. SPIFFS detects config size mismatch → prints "Config size mismatch" or similar → resets to defaults
2. GPS initializes (configureGPS() runs)
3. When GPS loop fires (every ~1 second):
   - Without GPS antenna: `foil_speed = 0xFF` (no output from PhA checks — correct, no fix means no check)
   - With GPS antenna and good fix: no `[PhA]` error messages → readings accepted silently

### Step 5.3 — Web UI verification

Connect to the RX WiFi AP (password from config, default `12345678`). Navigate to the config page. Confirm:
- "GPS & Follow-Me" section now shows 4 new anti-spoofing fields:
  - GPS Max HDOP (default 2.0)
  - GPS Max Acceleration (default 3.0)
  - GPS Max Teleport Speed (default 200.0)
  - GPS Suspect Threshold (default 3)
- Changing and saving a value persists across reboot

### Step 5.4 — Re-configure hardware

After flash/test cycle:
1. Re-pair TX and RX (hold bind button or use web UI)
2. Re-enter all custom settings via web UI
3. Re-calibrate compass: type `runcal` in Serial Monitor, follow prompts

---

## Self-Review Checklist

- [x] **HDOP check** covered in gpsPhaseACheck() — Task 4.4
- [x] **Acceleration check** covered in gpsPhaseACheck() — Task 4.4
- [x] **Teleport check** covered in gpsPhaseACheck() — Task 4.4
- [x] **gps_suspect_threshold** consecutive failures → gps_rejected — Task 4.5
- [x] **confStruct** has all 4 new fields — Task 1.1
- [x] **static_assert** updated 112→128 — Task 1.2
- [x] **defaultConf** has all 4 defaults — Task 1.3
- [x] **kCfgFields** has all 4 entries with correct types and ranges — Task 2.1
- [x] **Web UI** has all 4 fields in "GPS & Follow-Me" group — Task 3.2
- [x] **Web UI labels** are human-readable (not variable names) — Task 3.2
- [x] **Web UI ranges** are included in description text — Task 3.2
- [x] **Section 10 rule** satisfied: confStruct → ConfigService → WebUI all updated — Tasks 1-3
- [x] **TX firmware not touched** — correct, Phase A is RX-only
- [x] **V2 folders not touched** — correct
- [x] **File header V3 tag** added to GPS.ino — Task 4.1
- [x] **Field Service Note** updated for 112→128 change — Task 4.2
- [x] **CLAUDE.md Section 9 safety** — not violated (no motor movement path changed)
- [x] **No placeholder code** — all code complete and compilable
