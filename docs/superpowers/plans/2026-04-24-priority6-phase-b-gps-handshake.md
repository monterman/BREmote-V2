# Priority 6 — Phase B GPS Handshake Anti-Spoofing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add Phase B GPS anti-spoofing to the RX firmware: whenever TX sends a 0xF3 GPS meta-packet (already implemented in P5), RX validates that TX and RX are within `gps_max_pair_dist_m` of each other and moving at similar speeds; if not, RTM arming is blocked.

**Architecture:** Phase B runs entirely on the RX side inside `processMetaGpsPacket()` (Radio.ino). No TX code changes — TX already sends GPS coords at 2Hz. The check is time-gated: runs on the first valid meta-packet and every 30 seconds thereafter. Result is stored in the global `gps_phase_b_ok` flag, which the future RTM state machine (Priority 7) will read alongside `gps_rejected` from Phase A.

**Tech Stack:** Arduino / ESP32-S3, C++, TinyGPS++ (distanceBetween already used by Phase A in GPS.ino), FreeRTOS, SPIFFS, RadioLib (SX1262). All .ino files in a sketch are compiled into one translation unit, so globals declared in any .ino file are accessible from all others.

---

## Struct Size Change Warning

Adding 2 floats to `confStruct` changes sizeof from **128 → 136 bytes**. On first flash of this firmware to any RX unit, SPIFFS detects the mismatch and resets ALL RX settings to defaults. After flashing you must:
1. Re-pair TX and RX
2. Re-configure all settings via web UI
3. Re-calibrate compass (`runcal` serial command)
4. Verify Phase B defaults (Pair Dist 500 m, Speed Diff 50 km/h)

---

## File Map

| File | Action | What changes |
|------|--------|-------------|
| `Source/V2_Integration_Rx/BREmote_V2_Rx.h` | Modify | Add 2 fields to confStruct; update defaultConf; update static_assert 128→136; add version comment |
| `Source/V2_Integration_Rx/ConfigService.ino` | Modify | Add 2 kCfgFields entries for the new params |
| `Source/V2_Integration_Rx/WebUiEmbedded.h` | Modify | Add 2 UI entries in "GPS & Follow-Me" group |
| `Source/V2_Integration_Rx/Radio.ino` | Modify | Add `gps_phase_b_ok` global; add `gpsPhaseBCheck()` function; call it from `processMetaGpsPacket()` |

---

## Task 1: Add Phase B fields to confStruct (BREmote_V2_Rx.h)

**Files:**
- Modify: `Source/V2_Integration_Rx/BREmote_V2_Rx.h`

- [ ] **Step 1: Update the version comment at the top of the file**

Replace the existing top version comment block:
```cpp
// V3 - 2026-04-22 - Added gps_chip_type field to confStruct (GPS module selector); sizeof 108→112; updated defaultConf
// V3 - 2026-04-22 - Added Phase A GPS anti-spoofing params to confStruct; sizeof 112→128; updated defaultConf
// V3 - 2026-04-24 - Added rx_tx_gps_lat/lng/timestamp globals for 0xF3 meta-packet reception
```
With:
```cpp
// V3 - 2026-04-22 - Added gps_chip_type field to confStruct (GPS module selector); sizeof 108→112; updated defaultConf
// V3 - 2026-04-22 - Added Phase A GPS anti-spoofing params to confStruct; sizeof 112→128; updated defaultConf
// V3 - 2026-04-24 - Added rx_tx_gps_lat/lng/timestamp globals for 0xF3 meta-packet reception
// V3 - 2026-04-24 - Added Phase B GPS handshake params to confStruct; sizeof 128→136; updated defaultConf
```

- [ ] **Step 2: Add the 2 new fields to confStruct — append after gps_suspect_threshold**

Find this block at the end of confStruct:
```cpp
    float    gps_max_hdop;            // Max HDOP for a valid fix; range 0.5-5.0; default 2.0; dimensionless
    float    gps_max_accel_g;         // Max implied acceleration between readings; range 1.0-10.0G; default 3.0G
    float    gps_max_jump_kmh;        // Max position-implied speed (teleport check); range 50-500 km/h; default 200
    uint16_t gps_suspect_threshold;   // Consecutive failures before GPS marked rejected; range 1-10; default 3
};
```
Replace with:
```cpp
    float    gps_max_hdop;            // Max HDOP for a valid fix; range 0.5-5.0; default 2.0; dimensionless
    float    gps_max_accel_g;         // Max implied acceleration between readings; range 1.0-10.0G; default 3.0G
    float    gps_max_jump_kmh;        // Max position-implied speed (teleport check); range 50-500 km/h; default 200
    uint16_t gps_suspect_threshold;   // Consecutive failures before GPS marked rejected; range 1-10; default 3

    // ============================================================
    // V3 - 2026-04-24 - PHASE B GPS HANDSHAKE ANTI-SPOOFING PARAMETERS
    //
    // These two parameters control Phase B, which runs every time a
    // 0xF3 GPS meta-packet is received from TX (max every 30s).
    //
    // Distance check: TX-RX Haversine distance must be <
    //   gps_max_pair_dist_m or RTM arming is blocked.
    // Speed consistency check: TX implied speed (from consecutive
    //   meta-packet positions) must be within gps_max_speed_diff_kmh
    //   of RX GPS speed or arming is blocked.
    //
    // !!! Adding these fields changes sizeof(confStruct) 128→136. !!!
    // !!! On first flash after this change, SPIFFS resets ALL      !!!
    // !!! settings to defaults. After flashing:                    !!!
    // !!!   1) Re-pair TX and RX                                   !!!
    // !!!   2) Re-configure all settings via web UI                !!!
    // !!!   3) Re-calibrate compass (runcal)                       !!!
    // !!!   4) Verify Phase B defaults (500 m, 50 km/h)            !!!
    // ============================================================
    float gps_max_pair_dist_m;      // Max plausible TX-RX distance at handshake; range 50-2000 m; default 500 m
    float gps_max_speed_diff_kmh;   // Max TX-RX speed difference for handshake; range 10-200 km/h; default 50 km/h
};
```

- [ ] **Step 3: Update the static_assert from 128 to 136**

Find:
```cpp
static_assert(sizeof(confStruct) == 128, "confStruct size mismatch — expected 128 bytes (V3.1). Update this assert and SPIFFS migration logic if you change the struct.");  // V3 fix (N-1): pinned to exact size; catches both shrinkage and unexpected growth. 112→128 when Phase A anti-spoofing params added 2026-04-22.
```
Replace with:
```cpp
static_assert(sizeof(confStruct) == 136, "confStruct size mismatch — expected 136 bytes (V3.2). Update this assert and SPIFFS migration logic if you change the struct.");  // V3 fix (N-1): pinned to exact size; catches both shrinkage and unexpected growth. 112→128 when Phase A added 2026-04-22; 128→136 when Phase B added 2026-04-24.
```

- [ ] **Step 4: Add Phase B defaults to defaultConf**

Find the end of the defaultConf initializer (the last two lines before the closing brace):
```cpp
  2.0f,       // gps_max_hdop:           max HDOP for valid reading (range 0.5-5.0)
  3.0f,       // gps_max_accel_g:        max implied acceleration (range 1.0-10.0 G)
  200.0f,     // gps_max_jump_kmh:       max teleport-implied speed (range 50-500 km/h)
  3           // gps_suspect_threshold:  consecutive failures before GPS rejected (range 1-10)
};
```
Replace with:
```cpp
  2.0f,       // gps_max_hdop:           max HDOP for valid reading (range 0.5-5.0)
  3.0f,       // gps_max_accel_g:        max implied acceleration (range 1.0-10.0 G)
  200.0f,     // gps_max_jump_kmh:       max teleport-implied speed (range 50-500 km/h)
  3,          // gps_suspect_threshold:  consecutive failures before GPS rejected (range 1-10)
  // V3 - 2026-04-24 - Phase B GPS handshake anti-spoofing defaults (see CLAUDE.md Section 11)
  500.0f,     // gps_max_pair_dist_m:    max TX-RX pairing distance (range 50-2000 m)
  50.0f       // gps_max_speed_diff_kmh: max TX-RX speed difference (range 10-200 km/h)
};
```

- [ ] **Step 5: Add a FIELD SERVICE NOTE to GPS.ino documenting the struct size change**

Open `Source/V2_Integration_Rx/GPS.ino`. Find the last existing FIELD SERVICE NOTE block:

```
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

Append immediately AFTER that block (before the Phase A state variables):

```cpp
// ============================================================
// V3 - 2026-04-24: sizeof(confStruct) changed from 128 to 136
// bytes (Phase B GPS handshake params added). On the first
// flash after this change, SPIFFS will again detect the size
// mismatch and reset ALL settings to defaults. After flashing:
//   1) Re-pair TX and RX
//   2) Re-configure all settings via the web UI
//   3) Re-calibrate compass via the 'runcal' serial command
//   4) Verify Phase B defaults in "GPS & Follow-Me" section of
//      the web UI (Pair Dist 500 m, Speed Diff 50 km/h)
// ============================================================
```

- [ ] **Step 6: Verify the file compiles (Arduino IDE check)**

Open Arduino IDE, select the RX sketch, click Verify. Expected: 0 errors. If any error mentions `static_assert` or `confStruct`, the field sizes are wrong — recount bytes.

---

## Task 2: Register Phase B fields in ConfigService.ino

**Files:**
- Modify: `Source/V2_Integration_Rx/ConfigService.ino`

- [ ] **Step 1: Add 2 field specs after gps_suspect_threshold**

Find the Phase A group of fields ending with:
```cpp
  {"gps_suspect_threshold",  CFG_U16,   offsetof(confStruct, gps_suspect_threshold),  true, false, true,  1.0f, 10.0f, 0, false},
```

Append immediately after that line (before `{"logger_en"...}`):
```cpp
  // V3 - 2026-04-24 - Phase B GPS handshake anti-spoofing parameters (see CLAUDE.md Section 11)
  {"gps_max_pair_dist_m",    CFG_FLOAT, offsetof(confStruct, gps_max_pair_dist_m),    true, false, true, 50.0f, 2000.0f, 1, false},
  {"gps_max_speed_diff_kmh", CFG_FLOAT, offsetof(confStruct, gps_max_speed_diff_kmh), true, false, true, 10.0f,  200.0f, 1, false},
```

- [ ] **Step 2: Add version tag at the top of ConfigService.ino**

Find the existing version tags at the very top of the file:
```cpp
// V3 - 2026-04-22 - Added gps_chip_type field (GPS module selector: 0=BN-220, 1=BN-880+compass, 2=M10, 3=M10+compass)
// V3 - 2026-04-22 - Added Phase A GPS anti-spoofing fields: gps_max_hdop, gps_max_accel_g, gps_max_jump_kmh, gps_suspect_threshold
```
Append:
```cpp
// V3 - 2026-04-24 - Added Phase B GPS handshake fields: gps_max_pair_dist_m, gps_max_speed_diff_kmh
```

- [ ] **Step 3: Compile verify**

Click Verify in Arduino IDE. Expected: 0 errors.

---

## Task 3: Add Phase B controls to the Web Config UI (WebUiEmbedded.h)

**Files:**
- Modify: `Source/V2_Integration_Rx/WebUiEmbedded.h`

- [ ] **Step 1: Find the GPS anti-spoofing group and add 2 Phase B entries**

Find this line (the last Phase A entry in the GPS group):
```
{key:"gps_suspect_threshold",label:"GPS Suspect Threshold",description:"Consecutive anti-spoofing failures before GPS is marked rejected. Range 1-10, default 3. While rejected, RTM arming is blocked.",group:"GPS & Follow-Me",type:"int",def:3,min:1,max:10},
```

Append immediately after that line:
```
{key:"gps_max_pair_dist_m",label:"Phase B: Max Pair Distance",description:"Maximum plausible TX-RX distance during GPS handshake check. Range 50-2000 m, default 500 m. If TX and RX are further apart than this, RTM arming is blocked.",group:"GPS & Follow-Me",type:"float",def:500.0,min:50.0,max:2000.0,step:10.0,unit:"m"},
{key:"gps_max_speed_diff_kmh",label:"Phase B: Max Speed Difference",description:"Maximum TX-RX GPS speed difference during handshake check. Range 10-200 km/h, default 50 km/h. If speeds differ more than this, RTM arming is blocked.",group:"GPS & Follow-Me",type:"float",def:50.0,min:10.0,max:200.0,step:1.0,unit:"km/h"},
```

- [ ] **Step 2: Compile verify**

Click Verify in Arduino IDE. Expected: 0 errors.

---

## Task 4: Implement gpsPhaseBCheck() in Radio.ino

**Files:**
- Modify: `Source/V2_Integration_Rx/Radio.ino`

- [ ] **Step 1: Add version tag and global gps_phase_b_ok at the top of Radio.ino**

The current first line of Radio.ino is:
```cpp
// V3 - 2026-04-24 - Added GPS meta-packet reception: gps_meta_pending state, processMetaGpsPacket(), triggeredReceive() 2-path state machine
```
Replace that first line with:
```cpp
// V3 - 2026-04-24 - Added GPS meta-packet reception: gps_meta_pending state, processMetaGpsPacket(), triggeredReceive() 2-path state machine
// V3 - 2026-04-24 - Added Phase B GPS handshake check: gpsPhaseBCheck() called from processMetaGpsPacket()
```

Then, immediately after the version comment lines (before `void radioErrorHalt`), add:
```cpp
// ============================================================
// PHASE B GPS HANDSHAKE STATE
//
// gps_phase_b_ok: set true when Phase B distance and speed checks
// both pass; set false when either fails. Initialized false so
// RTM arming is blocked until the first successful handshake.
// NOT static — the RTM state machine (Priority 7) reads this flag.
//
// The static variables below are internal to gpsPhaseBCheck() and
// are declared here (file scope) to survive across calls.
// ============================================================
bool gps_phase_b_ok = false;  // Phase B handshake result; false = RTM arming blocked

// Last time Phase B check ran (ms). 0 = never run this session.
static unsigned long gps_phase_b_last_check_ms = 0;

// Previous TX GPS position snapshot used to compute TX implied speed.
// Updated each time Phase B check runs. 0 = no prior snapshot.
static double        gps_phase_b_prev_tx_lat = 0.0;
static double        gps_phase_b_prev_tx_lng = 0.0;
static unsigned long gps_phase_b_prev_tx_ms  = 0;
```

- [ ] **Step 2: Add the gpsPhaseBCheck() function — insert before processMetaGpsPacket()**

Find this line (the start of processMetaGpsPacket):
```cpp
// ============================================================
// processMetaGpsPacket - Decode a received 14-byte GPS data packet
```

Insert the entire `gpsPhaseBCheck()` function immediately BEFORE it:

```cpp
// ============================================================
// gpsPhaseBCheck - Phase B GPS handshake anti-spoofing validation
// ============================================================
//
// What it does:
//   Called after every successful 0xF3 GPS meta-packet decode.
//   Validates that TX and RX are physically plausible partners:
//
//   1) Distance check: Haversine distance between the last accepted
//      RX GPS position and the just-received TX GPS position must
//      be < usrConf.gps_max_pair_dist_m. Catches a spoofed TX GPS
//      report placing TX far from RX.
//
//   2) Speed consistency check: TX implied speed, computed from two
//      consecutive Phase B check positions, must differ from RX GPS
//      speed by < usrConf.gps_max_speed_diff_kmh. Catches GPS
//      replay attacks that report implausible TX movement.
//      Skipped on the very first check (no prior TX snapshot yet).
//
//   Time-gated: runs only on the first call after boot (or reset)
//   and every 30 seconds thereafter, regardless of how often
//   meta-packets arrive (they come at 2Hz).
//
//   Skipped entirely if:
//   - GPS disabled (usrConf.gps_en == 0)
//   - RX has no valid GPS reading (gps_last_ms == 0)
//
// Inputs:
//   Reads module globals: rx_tx_gps_lat, rx_tx_gps_lng (just updated by
//   processMetaGpsPacket), gps_last_lat, gps_last_lng, gps_last_speed_kmh,
//   gps_last_ms (from GPS.ino), usrConf.gps_en/gps_max_pair_dist_m/
//   gps_max_speed_diff_kmh.
//
// Side effects:
//   Sets gps_phase_b_ok to true (pass) or false (fail).
//   Updates gps_phase_b_last_check_ms and gps_phase_b_prev_tx_* snapshot.
//   Prints diagnostics to Serial.
// ============================================================
static void gpsPhaseBCheck()
{
  // ---- Prerequisite: GPS must be enabled in config ----
  if (!usrConf.gps_en)
  {
    // GPS disabled — Phase B cannot run; do not change gps_phase_b_ok.
    return;
  }

  // ---- Prerequisite: RX must have at least one valid accepted GPS reading ----
  if (gps_last_ms == 0)
  {
    rxprintln("GPS [PhB] Skipped — RX has no valid GPS reading yet");
    return;
  }

  // ---- Time gate: run on first call, then at most once every 30 seconds ----
  unsigned long now = millis();
  if (gps_phase_b_last_check_ms != 0 &&
      (now - gps_phase_b_last_check_ms) < 30000UL)
  {
    return;  // Not due yet; skip silently
  }
  gps_phase_b_last_check_ms = now;

  // ---- Check 1: TX-RX Haversine distance ----
  // TinyGPSPlus::distanceBetween() returns metres between two WGS84 lat/lng pairs.
  float dist_m = (float)TinyGPSPlus::distanceBetween(
      gps_last_lat, gps_last_lng,
      rx_tx_gps_lat, rx_tx_gps_lng);

  if (dist_m > usrConf.gps_max_pair_dist_m)
  {
    Serial.printf("GPS [PhB] FAIL distance: %.0f m > max %.0f m — RTM arming blocked\n",
                  dist_m, (double)usrConf.gps_max_pair_dist_m);
    gps_phase_b_ok = false;
    // Update snapshot so the next check has a fresh reference point.
    gps_phase_b_prev_tx_lat = rx_tx_gps_lat;
    gps_phase_b_prev_tx_lng = rx_tx_gps_lng;
    gps_phase_b_prev_tx_ms  = now;
    return;
  }

  // ---- Check 2: TX-RX speed consistency ----
  // Compute TX implied speed from the position change since the last Phase B snapshot.
  // Skip on the very first run because there is no prior snapshot to measure from.
  if (gps_phase_b_prev_tx_ms > 0)
  {
    float dt_s = (float)(now - gps_phase_b_prev_tx_ms) / 1000.0f;

    // Guard against near-zero dt (shouldn't happen due to 30s gate, but be safe).
    if (dt_s > 0.1f)
    {
      float tx_delta_m   = (float)TinyGPSPlus::distanceBetween(
          gps_phase_b_prev_tx_lat, gps_phase_b_prev_tx_lng,
          rx_tx_gps_lat, rx_tx_gps_lng);
      float tx_speed_kmh = (tx_delta_m / dt_s) * 3.6f;
      float speed_diff   = fabsf(tx_speed_kmh - gps_last_speed_kmh);

      if (speed_diff > usrConf.gps_max_speed_diff_kmh)
      {
        Serial.printf("GPS [PhB] FAIL speed: TX %.1f km/h, RX %.1f km/h, diff %.1f km/h > max %.1f km/h\n",
                      tx_speed_kmh, gps_last_speed_kmh, speed_diff,
                      (double)usrConf.gps_max_speed_diff_kmh);
        gps_phase_b_ok = false;
        gps_phase_b_prev_tx_lat = rx_tx_gps_lat;
        gps_phase_b_prev_tx_lng = rx_tx_gps_lng;
        gps_phase_b_prev_tx_ms  = now;
        return;
      }
    }
  }

  // ---- All checks passed ----
  Serial.printf("GPS [PhB] PASS: dist %.0f m (max %.0f m)\n",
                dist_m, (double)usrConf.gps_max_pair_dist_m);
  gps_phase_b_ok = true;
  gps_phase_b_prev_tx_lat = rx_tx_gps_lat;
  gps_phase_b_prev_tx_lng = rx_tx_gps_lng;
  gps_phase_b_prev_tx_ms  = now;
}
```

- [ ] **Step 3: Call gpsPhaseBCheck() at the end of processMetaGpsPacket()**

Find the success path inside `processMetaGpsPacket()` — the last three lines before the closing brace:
```cpp
  rx_tx_gps_lat       = (double)lat_ud / 1e6;
  rx_tx_gps_lng       = (double)lng_ud / 1e6;
  rx_tx_gps_timestamp = millis();

  #ifdef DEBUG_RX
  Serial.printf("META GPS received: lat=%.6f lng=%.6f\n",
                rx_tx_gps_lat, rx_tx_gps_lng);
  #endif
}
```
Replace with:
```cpp
  rx_tx_gps_lat       = (double)lat_ud / 1e6;
  rx_tx_gps_lng       = (double)lng_ud / 1e6;
  rx_tx_gps_timestamp = millis();

  #ifdef DEBUG_RX
  Serial.printf("META GPS received: lat=%.6f lng=%.6f\n",
                rx_tx_gps_lat, rx_tx_gps_lng);
  #endif

  // Run Phase B anti-spoofing check against the freshly received TX GPS position.
  // gpsPhaseBCheck() is time-gated (first call + every 30s) and self-throttles.
  gpsPhaseBCheck();
}
```

- [ ] **Step 4: Compile verify**

Click Verify in Arduino IDE. Expected: 0 errors.

---

## Task 5: Final verification and commit

- [ ] **Step 1: Check that all 4 modified files have version tags**

Each file should have a `// V3 - 2026-04-24 - ...` tag. Verify:
- `BREmote_V2_Rx.h`: tag in first line block ✓ (added in Task 1 Step 1)
- `ConfigService.ino`: tag at top ✓ (added in Task 2 Step 2)
- `WebUiEmbedded.h`: no version tag required (it has none already) — OK
- `Radio.ino`: tag in first line block ✓ (added in Task 4 Step 1)

- [ ] **Step 2: Re-verify full compile one more time with all 4 files saved**

Click Verify. Expected: 0 errors, 0 warnings. If any warning about unused variable, investigate — it likely means a variable name typo.

- [ ] **Step 3: Flash to RX hardware and test with DEBUG_RX serial output**

With both TX and RX powered and GPS fix acquired:

Expected serial output sequence (RX serial monitor):
```
GPS [PhB] Skipped — RX has no valid GPS reading yet
```
(on first few meta-packets before RX gets a fix)

Then after RX gets GPS fix and the 30s gate elapses:
```
META GPS received: lat=XX.XXXXXX lng=XX.XXXXXX
GPS [PhB] PASS: dist Xm (max 500m)
```

To trigger a failure for manual testing:
- Temporarily set `gps_max_pair_dist_m` to 1 m via web UI → Save → wait for next check
- Expected: `GPS [PhB] FAIL distance: Xm > max 1m — RTM arming blocked`
- Restore to 500 m → wait 30s → should see PASS again

- [ ] **Step 4: Update CLAUDE.md**

Mark Priority 6 complete in CLAUDE.md Section 7. Change:
```
**Priority 6**: Phase B GPS Handshake Anti-Spoofing
- TX↔RX distance plausibility + speed consistency check via 0xF3 (on connect + every 30s)
- 2 new RX SPIFFS params: `gps_max_pair_dist_m`, `gps_max_speed_diff_kmh` (see Section 11)
- Failure → RTM arming blocked until next successful handshake
```
To:
```
**Priority 6 — COMPLETED 2026-04-24** ✅: Phase B GPS Handshake Anti-Spoofing
- TX↔RX distance check + speed consistency check, time-gated at 30s intervals
- `gpsPhaseBCheck()` in RX Radio.ino; called from processMetaGpsPacket()
- `gps_phase_b_ok` global flag blocks RTM arming on failure
- 2 new RX SPIFFS params: `gps_max_pair_dist_m` (50-2000m, default 500m), `gps_max_speed_diff_kmh` (10-200km/h, default 50)
- sizeof(confStruct) 128→136; first flash resets all RX settings to defaults
```

- [ ] **Step 5: Git commit all changed files**

```bash
git add Source/V2_Integration_Rx/BREmote_V2_Rx.h \
        Source/V2_Integration_Rx/GPS.ino \
        Source/V2_Integration_Rx/ConfigService.ino \
        Source/V2_Integration_Rx/WebUiEmbedded.h \
        Source/V2_Integration_Rx/Radio.ino \
        CLAUDE.md
git commit -m "feat(RX): Priority 6 — Phase B GPS handshake anti-spoofing

gpsPhaseBCheck() runs on first 0xF3 meta-packet + every 30s.
Distance check: TX-RX Haversine > gps_max_pair_dist_m → fail.
Speed check: |TX_speed - RX_speed| > gps_max_speed_diff_kmh → fail.
gps_phase_b_ok flag blocks RTM arming on failure.
sizeof(confStruct) 128→136; first flash resets SPIFFS to defaults.
2 new SPIFFS params added to confStruct, ConfigService, WebUI."
```

---

## Self-Review

### Spec coverage

| Requirement (CLAUDE.md §11 Phase B) | Covered by |
|---|---|
| Runs when TX connects | ✅ Task 4: first meta-packet call (gps_phase_b_last_check_ms == 0) |
| Runs every 30 seconds during active session | ✅ Task 4: 30s time gate in gpsPhaseBCheck() |
| Distance check < gps_max_pair_dist_m | ✅ Task 4: Check 1 using TinyGPSPlus::distanceBetween |
| Speed consistency < gps_max_speed_diff_kmh | ✅ Task 4: Check 2 using position delta over 30s window |
| Failure blocks RTM arming | ✅ Task 4: sets gps_phase_b_ok = false; RTM will check this flag (Priority 7) |
| Spoofing event logged | ✅ Task 4: Serial.printf FAIL messages in both checks |
| 2 new SPIFFS params gps_max_pair_dist_m, gps_max_speed_diff_kmh | ✅ Tasks 1-3 |
| Section 10 Web Config UI Rule | ✅ Task 3: both params added to WebUiEmbedded.h with labels/descriptions/ranges |
| Section 3: version tag on every modified file | ✅ All 4 files get V3 - 2026-04-24 tag |
| Section 3: plain-English comments on every function/block | ✅ gpsPhaseBCheck() has full header comment; every code block has inline comment |

### No-placeholder scan

All code blocks show complete, compilable code. No "TBD" or "similar to Task N" patterns present.

### Type consistency

- `gps_max_pair_dist_m`: declared `float` in confStruct, registered `CFG_FLOAT` in ConfigService, `float` in gpsPhaseBCheck comparison — consistent.
- `gps_max_speed_diff_kmh`: same pattern — consistent.
- `gps_phase_b_ok`: declared `bool` globally, set `true`/`false` in gpsPhaseBCheck — consistent.
- `gps_last_lat/lng/speed_kmh/ms`: declared in GPS.ino (`double`, `double`, `float`, `unsigned long`) — accessed as same types in gpsPhaseBCheck — consistent.
- `rx_tx_gps_lat/lng/timestamp`: declared in BREmote_V2_Rx.h (`double`, `double`, `unsigned long`) — accessed as same types — consistent.
- `TinyGPSPlus::distanceBetween()` signature: `static double distanceBetween(double, double, double, double)` — cast to `float` at call site, which is safe (narrowing of double to float, acceptable for 500m-range comparisons) — consistent with Phase A usage in GPS.ino.
