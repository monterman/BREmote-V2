# BREmote V2.5-Evo — Priority 9 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix 4 safety-critical RTM bugs, add full-screen one-time confirmation messages with a 3×7 compact font, add distance unit selection (metric/imperial/nautical), add an R5 proximity bar, and globally rename "BREmote V3" → "BREmote V2.5-Evo".

**Architecture:** All display changes are TX-only (ESP32-C3). Bug 1A and the "always-compute-distance" fix (which resolves Bugs 1B and 1C) are RX-only (ESP32-S3). Bug 1D and all event wiring are TX-only. `showFullScreenMessage()` is a blocking helper in Display.ino; it is called from RTMState.ino at the exact event moment. All internal math remains in metres; unit conversion is a pure display-layer transform in `renderRtmInfoDisplay()`. The new `dist_unit` field fills the existing 2-byte tail padding in TX confStruct — sizeof stays 128, no SPIFFS reset.

**Tech Stack:** Arduino IDE (ESP32-C3 TX, ESP32-S3 RX), HT16K33 LED matrix I2C, TinyGPS++, FreeRTOS tasks for haptic/bargraph, SPIFFS for config, ConfigServiceEngine + WebConfigEngine.

---

## ⚠️ HARD PREREQUISITE — Must be done before Task 4

`docs/Dot_Matrix_Display_10x7_Render.html` **must exist** in the project before the fontCompact3x7 array can be extracted. The user stated this file will be copied into `docs/` and renamed. Without it, Task 4 (font + showFullScreenMessage) is blocked. All other tasks can proceed in order; Task 5 depends on Task 4.

**User action:**
- [ ] Copy `Dot_Matrix_Display_10x7_WEB.html` to `docs/Dot_Matrix_Display_10x7_Render.html`
- [ ] Confirm the file is present before proceeding to Task 4

---

## Display Buffer Layout Reference (critical for Tasks 4 and 8)

The firmware represents the 10×7 LED matrix as follows:

```
displayBuffer[1..7]  — each element is one ROW
  displayBuffer[1] = R0  (top)
  displayBuffer[2] = R1
  displayBuffer[3] = R2
  displayBuffer[4] = R3
  displayBuffer[5] = R4  (decimal dot at bit 3 = column C3)
  displayBuffer[6] = R5  ← PROXIMITY BAR goes here (currently unused)
  displayBuffer[7] = R6  (battery bar, horizontal)

Bits within each displayBuffer[r]:
  bit 0 = C0  bit 1 = C1  bit 2 = C2  bit 3 = C3 (gap/dot)
  bit 4 = C4  bit 5 = C5  bit 6 = C6  bit 7 = C7 (GPS dot)
  bit 8 = C8 (temp bar)   bit 9 = C9 (signal bar)
```

The existing `num0[]` font is 5 rows (R0-R4) × 3 columns per character.
`fontCompact3x7` will be 7 rows (R0-R6) × 3 columns per character.

---

## File Structure

| File | Changes |
|---|---|
| `Source/V2_Integration_Rx/RTMState.ino` | Task 1: Bug 1A (Gate 9 zero-guard) + Bug 1B/1C (always-compute distance before gates) |
| `Source/V2_Integration_Tx/Display.ino` | Task 4: fontCompact3x7 + showFullScreenMessage() + E71 update; Task 8: dist_unit aware renderRtmInfoDisplay() + R5 proximity bar |
| `Source/V2_Integration_Tx/RTMState.ino` | Task 5: Bug 1B pre-arm check + Bug 1D all-exits Pattern4/StP + Section 2 FM confirms |
| `Source/V2_Integration_Tx/BREmote_V2_Tx.h` | Task 6: dist_unit field + rtm_arm_dist_m RAM global |
| `Source/V2_Integration_Tx/ConfigService.ino` | Task 7: dist_unit in kCfgFields |
| `Source/V2_Integration_Tx/WebUiEmbedded.h` | Task 7: dist_unit dropdown in web UI |
| Multiple files | Task 2: Section 0 rename ("BREmote V3" → "BREmote V2.5-Evo") |
| `docs/display-reference.md` | Task 9: Full Section 5 documentation update |

**Note on Display.ino split (Tasks 4 and 8):** These are two separate editing passes on the same file. Task 4 adds the new functions. Task 8 modifies `renderRtmInfoDisplay()` and adds the proximity bar. Approve and compile after Task 4 before proceeding to Task 8.

---

## Root Cause Analysis (embedded in each task for context)

**Bug 1A:** `usrConf.rtm_stop_distance_m` reads as **0** from SPIFFS on devices that stored the old defaultConf (before the P8 critical fix set it to 3m). Since sizeof didn't change with P8, the SPIFFS struct was not reset, and the old zero persists. `dist_m < 0.0f` is always false → Gate 9 never fires.

**Bug 1B:** RX only populates `telemetry.rtm_distance` during active RTM (when `rtm_rx_active == true`). Before the second squeeze, `telemetry.rtm_distance == 0xFF`. TX cannot perform a pre-arm distance check.

**Bug 1C:** Distance is computed INSIDE the gate success path — if any gate fails (Phase B not yet passed, GPS stale), the distance stays at 0xFF. Even when gates pass, index 5 takes up to 600ms to cycle through the telemetry sequence. Fix: compute distance unconditionally from valid GPS data, before gate checks, in every RX `runRtmLoop()` call.

**Bug 1D:** Three exit paths from RTM_ACTIVE do NOT fire Pattern 4:
  1. Gate 1 (throttle release 10 s) → goes to RTM_IDLE directly, bypasses `rtmDisengage()`.
  2. Gate 1 max-runtime → calls `rtmDisengage()` but that function doesn't set Pattern 4.
  3. Gate 2 (TX GPS stale) → same as above.
  Fix: move `current_vib_pattern = 4` + `showFullScreenMessage("St P", 2000)` into `rtmDisengage()` itself so ALL paths that call it get the haptic + display. Change the Gate 1 throttle-release direct-IDLE path to call `rtmDisengage()` instead.

---

## Task 1 — RX RTMState.ino: Bugs 1A + 1B + 1C

**Files:**
- Modify: `Source/V2_Integration_Rx/RTMState.ino`

### Root changes:

**1A — Gate 9 zero-guard** (add to `checkRtmSafetyGates()`):
```c
// Gate 9: hard stop distance
// Guard: rtm_stop_distance_m==0 means SPIFFS held the pre-fix zero default.
// Use 3m (firmware hard minimum) to keep Gate 9 active regardless of stored config.
uint16_t stop_dist_m = (usrConf.rtm_stop_distance_m > 0) ? usrConf.rtm_stop_distance_m : 3u;
float dist_m = (float)TinyGPSPlus::distanceBetween(
    gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);
if (dist_m < (float)stop_dist_m)
{
  Serial.printf("RTM [RX] STOP: within hard stop (%.0f m < %u m)\n",
                dist_m, stop_dist_m);
  rtm_rx_emergency_stop = true;
  return false;
}
```
Replace the existing Gate 9 block (lines 88–96) with the above.

**1B + 1C — Always-compute distance** (restructure beginning of `runRtmLoop()`):

Move `telemetry.rtm_distance` computation to run unconditionally when valid GPS data is available, before the `!rtm_rx_active` early-return. This ensures:
- TX always has a fresh distance value for Bug 1B pre-arm check.
- Distance is available immediately when RTM activates (Bug 1C).

```c
// ---- Always compute RX→TX distance when GPS data is available ----
// Populates telemetry.rtm_distance for TX at all times (not only during active RTM).
// Bug 1B: TX can perform a pre-arm distance check before second-squeeze confirmation.
// Bug 1C: Distance is available the moment RTM activates, no telemetry cycle delay.
{
  bool gps_rx_ok = (gps_last_ms > 0) && ((millis() - gps_last_ms) < 6000UL);
  bool gps_tx_ok = (rx_tx_gps_timestamp > 0) && ((millis() - rx_tx_gps_timestamp) < 5000UL);

  if (gps_rx_ok && gps_tx_ok)
  {
    float d = (float)TinyGPSPlus::distanceBetween(
        gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);

    if (d < 10.0f)
    {
      // 0-99: tenths of metre (d=0.0..9.9 m)
      telemetry.rtm_distance = (uint8_t)(d * 10.0f);
    }
    else
    {
      // 100-254: whole metres offset by 90 (100=10m, 254=164m cap)
      uint8_t whole_m = (uint8_t)(d > 164.0f ? 164.0f : d);
      telemetry.rtm_distance = 90u + whole_m;
    }
  }
  else if (!rtm_rx_active)
  {
    telemetry.rtm_distance = 0xFF;  // mark N/A only when inactive AND no GPS
  }
  // If rtm_rx_active but GPS data just expired, keep last known value (don't set 0xFF)
}

if (!usrConf.rtm_rx_enabled)
{
  rtm_rx_active         = false;
  rtm_rx_emergency_stop = false;
  return;
}

if (!rtm_rx_active)
{
  rtm_rx_emergency_stop    = false;
  rtm_prev_dist_m          = -1.0;
  rtm_phase_c_ms           = 0;
  // telemetry.rtm_distance already set above
  return;
}
```

Remove the old `telemetry.rtm_distance = 0xFF` assignment from the `!rtm_rx_active` block, and remove the duplicate distance-encode block that was after `checkRtmSafetyGates()`.

- [ ] Read current `Source/V2_Integration_Rx/RTMState.ino` to confirm exact line numbers
- [ ] Apply Gate 9 zero-guard (replace lines 88–96)
- [ ] Add always-compute-distance block at the top of `runRtmLoop()`, before `!rtm_rx_enabled` check
- [ ] Remove old `telemetry.rtm_distance = 0xFF` from `!rtm_rx_active` block
- [ ] Remove old distance-encode block after `checkRtmSafetyGates()` (lines 233–246)
- [ ] Add version tag: `// V2.5-Evo - 2026-04-28 - P9 Bug1A/1B/1C: Gate9 zero-guard; always-compute dist before gates`
- [ ] **Compile in Arduino IDE (RX board). Fix any errors before proceeding.**

---

## Task 2 — Section 0: Project Rename

**Files affected** (global search for "BREmote V3" and standalone "V3" in non-historical contexts):

The following files contain the project name or version string that must change:
- `CLAUDE.md` — title, section headers, Priority list entries
- `docs/display-reference.md` — title line "BREmote V3"
- `Source/V2_Integration_Tx/BREmote_V2_Tx.h` — `<title>` tag in web UI HTML, `SYS_DEVICE_LABEL` if present
- `Source/V2_Integration_Tx/WebUiEmbedded.h` — HTML title string "BREmote V2 TX Config"
- `Source/V2_Integration_Rx/WebUiEmbedded.h` — HTML title string
- `Source/V2_Integration_Tx/System.ino` — `SYS_DEVICE_LABEL` (if "V3" appears)
- `Source/V2_Integration_Rx/System.ino` — same
- Any README.md at repository root
- `Tools/BREmote_V3_Config_Studio AFM_v4.html` — filename contains "V3" (rename file itself)

**Scope clarification:**
- `// V3 - YYYY-MM-DD - ...` comment tags in source files are **historical commit logs** — leave them unchanged. They record what was added in which version and form an audit trail.
- New tags added in this PR use `// V2.5-Evo - 2026-04-28 - P9:`.
- `#define SW_VERSION 3` — leave unchanged. This is the SPIFFS key; changing it resets all stored configs on all devices.
- Web UI titles: "BREmote V2 TX Config" → "BREmote V2.5-Evo TX Config" (same for RX).
- All user-visible strings, doc headings, and README content saying "V3" or "BREmote V3".

- [ ] **Show user the complete list of occurrences found (grep result) and wait for explicit approval before any edits**
- [ ] Edit `CLAUDE.md` — update project name in header and priority entries
- [ ] Edit `docs/display-reference.md` — title line and any "V3" references
- [ ] Edit `Source/V2_Integration_Tx/WebUiEmbedded.h` — HTML `<title>` tag
- [ ] Edit `Source/V2_Integration_Rx/WebUiEmbedded.h` — HTML `<title>` tag
- [ ] Rename `Tools/BREmote_V3_Config_Studio AFM_v4.html` → `Tools/BREmote_V2.5-Evo_Config_Studio.html` (git mv)
- [ ] Edit README.md if present
- [ ] **Compile both TX and RX after rename. Verify no logic changed.**

---

## Task 3 — PREREQUISITE: Copy HTML font file

- [ ] Confirm `docs/Dot_Matrix_Display_10x7_Render.html` exists in the project
- [ ] Read the file and extract the `fontCompact` JavaScript object
- [ ] Record exact column bitmaps for: 'A', 'r', 'M', 'S', 't', 'P', 'F', '0', '1', '2', '3', '7', '1', 'E', ' ' (space = 1 dark column)
- [ ] Confirm layout: each character is exactly 3 display columns × 7 rows; each column value is a 7-bit integer (bit 0 = R0 top, bit 6 = R6 bottom)
- [ ] Document the bitmap table (for use in Task 4)

**If the file is not yet present:** stop here and ask the user to copy it before continuing with Task 4.

---

## Task 4 — TX Display.ino: fontCompact3x7 + showFullScreenMessage() + E71

**Files:**
- Modify: `Source/V2_Integration_Tx/Display.ino`

**Prerequisite:** Task 3 must be complete (bitmaps extracted).

### 4A — fontCompact3x7 C array

Add near the top of Display.ino, after `num0[]` array:

```c
// ============================================================
// COMPACT 3×7 FONT — used by showFullScreenMessage()
// Source: docs/Dot_Matrix_Display_10x7_Render.html fontCompact object.
// Each entry: 3 bytes = 3 display columns (C0-left, C1-mid, C2-right).
// Each byte: 7 bits = 7 display rows (bit 0 = R0 top, bit 6 = R6 bottom).
// Space character: not in this array — rendered as 1 dark column by the caller.
// Character order matches ASCII for lookup: index = character - ' '
// Supported characters: A r M S t P F 0 1 2 3 E 7 (plus space)
// ============================================================
// Format: fontCompact3x7['X' - FONTCOMPACT_BASE] = {col0, col1, col2}
// FONTCOMPACT_BASE = 32 (ASCII space); array covers ' ' through 'z' as needed.

// NOTE TO IMPLEMENTER: Fill the bitmap values below from the extracted
// fontCompact JavaScript object in docs/Dot_Matrix_Display_10x7_Render.html.
// The three values per character are the three column bitmaps (7-bit each).
// Characters not listed here must not be passed to showFullScreenMessage().

struct Fc3x7Entry { uint8_t col[3]; };

// Lookup by character: fc3x7_lookup(c) returns pointer to 3-byte column array,
// or nullptr for unsupported characters.
static const struct Fc3x7Entry fontCompact3x7[] = {
  // Indexed by (char - 48) for digits, separate table for letters.
  // ACTUAL VALUES come from Task 3 extraction. Scaffold only:
  /*  '0' */ { {0x00, 0x00, 0x00} },  // PLACEHOLDER — replace with HTML bitmap
  /*  '1' */ { {0x00, 0x00, 0x00} },
  /*  '2' */ { {0x00, 0x00, 0x00} },
  /*  '3' */ { {0x00, 0x00, 0x00} },
  /*  '7' */ { {0x00, 0x00, 0x00} },
};
// Letters stored separately for clarity
static const struct Fc3x7Entry fontCompact3x7_A = { {0x00, 0x00, 0x00} };  // PLACEHOLDER
static const struct Fc3x7Entry fontCompact3x7_E = { {0x00, 0x00, 0x00} };
static const struct Fc3x7Entry fontCompact3x7_F = { {0x00, 0x00, 0x00} };
static const struct Fc3x7Entry fontCompact3x7_M = { {0x00, 0x00, 0x00} };
static const struct Fc3x7Entry fontCompact3x7_P = { {0x00, 0x00, 0x00} };
static const struct Fc3x7Entry fontCompact3x7_S = { {0x00, 0x00, 0x00} };
static const struct Fc3x7Entry fontCompact3x7_r = { {0x00, 0x00, 0x00} };  // lowercase
static const struct Fc3x7Entry fontCompact3x7_t = { {0x00, 0x00, 0x00} };  // lowercase
```

**IMPORTANT:** The actual bitmap values must be copied verbatim from the `fontCompact` object in the HTML file. The scaffold above uses zeros. Replace each `{0x00, 0x00, 0x00}` with the real 3-byte column values.

### 4B — fc3x7GetChar() lookup helper

```c
// Returns pointer to 3-column bitmap for character c, or nullptr if unsupported.
// Space is handled by the caller (1 dark column, not in this table).
static const struct Fc3x7Entry* fc3x7GetChar(char c)
{
  switch (c) {
    case 'A': return &fontCompact3x7_A;
    case 'E': return &fontCompact3x7_E;
    case 'F': return &fontCompact3x7_F;
    case 'M': return &fontCompact3x7_M;
    case 'P': return &fontCompact3x7_P;
    case 'S': return &fontCompact3x7_S;
    case 'r': return &fontCompact3x7_r;
    case 't': return &fontCompact3x7_t;
    case '0': return &fontCompact3x7[0];
    case '1': return &fontCompact3x7[1];
    case '2': return &fontCompact3x7[2];
    case '3': return &fontCompact3x7[3];
    case '7': return &fontCompact3x7[4];
    default:  return nullptr;
  }
}
```

### 4C — showFullScreenMessage()

```c
// ============================================================
// V2.5-Evo - 2026-04-28 - P9: Full-screen one-time confirmation flash.
// Clears ALL 10 columns and ALL 7 rows (including temp C8, signal C9,
// R5 proximity bar, R6 battery bar). Renders msg using fontCompact3x7.
// Holds for duration_ms (blocking — acceptable for one-time 2-second events).
// On return, normal renderRtmInfoDisplay()/renderOperationalDisplay() rebuilds
// the buffer naturally on the next loop() call.
//
// msg format: any sequence of supported chars and spaces.
//   Space = 1 dark column. Each other char = 3 columns.
//   Caller must ensure total columns <= 10.
// duration_ms: how long to hold the message (typically 2000).
// ============================================================
void showFullScreenMessage(const char* msg, uint16_t duration_ms)
{
  // 1. Clear entire software buffer (all columns including C7-C9, all rows including R5/R6)
  for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000;

  // 2. Clear hardware display immediately (don't wait for updateDisplay)
  clearDisplay();

  // 3. Render msg into displayBuffer using fontCompact3x7
  uint8_t col = 0;  // current physical column (0-9)
  for (int ci = 0; msg[ci] != '\0' && col < 10; ci++)
  {
    if (msg[ci] == ' ')
    {
      col++;  // 1 dark column — buffer is already zero; just advance
    }
    else
    {
      const struct Fc3x7Entry* entry = fc3x7GetChar(msg[ci]);
      if (entry == nullptr) { col++; continue; }  // unknown char → dark column

      for (int fc = 0; fc < 3 && col < 10; fc++, col++)
      {
        uint8_t colBits = entry->col[fc];  // 7-bit column bitmap
        for (int row = 0; row < 7; row++)
        {
          if (colBits & (1u << row))
          {
            // displayBuffer[row+1] is R(row); set bit for physical column `col`
            displayBuffer[row + 1] |= (1u << col);
          }
        }
      }
    }
  }

  // 4. Push rendered message to hardware
  updateDisplay();

  // 5. Hold for duration_ms — other FreeRTOS tasks (vibrationTask, updateBargraphs)
  //    continue to run during this delay.
  delay(duration_ms);

  // 6. Clear buffer on exit so next renderOp() starts with a clean slate
  for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000;
}
```

Add `void showFullScreenMessage(const char* msg, uint16_t duration_ms);` to the forward declarations in `V2_Integration_Tx.ino`.

### 4D — E71 display change (display-only, no haptic/logic changes)

In `renderOperationalDisplay()`, find the error display section and add E71 full-screen handler:

```c
// V2.5-Evo - 2026-04-28 - P9: E71 water ingress — display only change.
// All existing E71 haptic (Pattern 3: 5×500ms) and alert logic are UNCHANGED.
// Only the display changes: render full-screen "E 71" using fontCompact3x7,
// flashing at 250ms on/off, persisting until error clears or power cycle.
// Supported chars: E(3) + space(1) + 7(3) + 1(3) = 10 cols exactly.
if (remote_error == 71)
{
  static unsigned long e71_blink_ms = 0;
  static bool e71_blink_state = false;
  if (millis() - e71_blink_ms >= 250)
  {
    e71_blink_state = !e71_blink_state;
    e71_blink_ms = millis();
    if (e71_blink_state)
    {
      // Render "E 71" full-screen (flash ON)
      for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000;
      // Inline render (cannot call showFullScreenMessage — that blocks for duration)
      const char* e71msg = "E 71";
      uint8_t col = 0;
      for (int ci = 0; e71msg[ci] && col < 10; ci++) {
        if (e71msg[ci] == ' ') { col++; continue; }
        const struct Fc3x7Entry* en = fc3x7GetChar(e71msg[ci]);
        if (!en) { col++; continue; }
        for (int fc = 0; fc < 3 && col < 10; fc++, col++) {
          uint8_t cb = en->col[fc];
          for (int r = 0; r < 7; r++) if (cb & (1u << r)) displayBuffer[r+1] |= (1u << col);
        }
      }
    }
    else
    {
      for (int i = 0; i < 8; i++) displayBuffer[i] = 0x0000;  // flash OFF
    }
    updateDisplay();
  }
  return;  // Do NOT fall through to normal error rendering
}
```

Place this block at the START of the `remote_error != 0` branch in `renderOperationalDisplay()`, before the existing ET handler.

### 4E — Remove scroll3Digits("Stp") from COOLDOWN (prep for Task 5)

The `scroll3Digits(LET_S, LET_T, LET_P, 50)` call in the COOLDOWN case will be replaced in Task 5. No Display.ino change needed here; Task 5 moves the "St P" display call into `rtmDisengage()`.

- [ ] Read current Display.ino to confirm line numbers and insert positions
- [ ] Add `fontCompact3x7` data arrays with real bitmaps from HTML (from Task 3)
- [ ] Add `fc3x7GetChar()` lookup function
- [ ] Add `showFullScreenMessage()` function
- [ ] Add E71 full-screen flash handler in `renderOperationalDisplay()`
- [ ] Add `void showFullScreenMessage(const char* msg, uint16_t duration_ms);` to V2_Integration_Tx.ino forward declarations
- [ ] Add version tag: `// V2.5-Evo - 2026-04-28 - P9: fontCompact3x7 + showFullScreenMessage() + E71 full-screen`
- [ ] **Compile in Arduino IDE (TX board). Fix any errors before proceeding.**

---

## Task 5 — TX RTMState.ino: Bug 1B + Bug 1D + Section 2 event wiring

**Files:**
- Modify: `Source/V2_Integration_Tx/RTMState.ino`

**Prerequisite:** Task 4 must be complete (`showFullScreenMessage` must exist).

### 5A — Bug 1D: Move Pattern 4 + "St P" into `rtmDisengage()`

Replace the existing `rtmDisengage()` function:

```c
// ---- Disengage RTM: return to COOLDOWN, notify RX, confirm with haptic + display ----
// V2.5-Evo - 2026-04-28 - P9 Bug1D: Pattern 4 and "St P" flash moved here so ALL
// exit paths (steer-exit, GPS stale, max runtime, throttle release) fire the confirm.
static void rtmDisengage()
{
  rtm_tx_state    = RTM_COOLDOWN;
  rtm_cooldown_ms = millis();
  rtm_tx_active   = false;
  rtm_thr_cap_tx  = 255;
  queueMetaPacketBurst(0xF1, 0);  // tell RX: RTM inactive

  // Fire Pattern 4 BEFORE blocking display so vibration runs during the 2s flash
  current_vib_pattern = 4;  // 2 fast short = RTM disengage confirm

  // Full-screen "St P" (S=3, t=3, space=1, P=3 = 10 cols exactly)
  showFullScreenMessage("St P", 2000);
}
```

Remove `current_vib_pattern = 4` from `setRtmDisarmed()` (now redundant):
```c
static void setRtmDisarmed()
{
  rtmDisengage();  // Pattern 4 + "St P" now handled inside rtmDisengage()
}
```

### 5B — Bug 1D: Gate 1 throttle-release path calls rtmDisengage()

In `RTM_ACTIVE` case, find the Gate 3 throttle-release path:

```c
// OLD — goes directly to IDLE, no haptic, no display
if (now - rtm_release_ms > 10000UL)
{
  rtm_tx_state   = RTM_IDLE;
  rtm_tx_active  = false;
  rtm_thr_cap_tx = 255;
  queueMetaPacketBurst(0xF1, 0);
  break;
}

// NEW — calls rtmDisengage() which fires Pattern 4 + "St P"
// V2.5-Evo - 2026-04-28 - P9 Bug1D: throttle-release gate now uses rtmDisengage()
// so Pattern 4 and "St P" fire consistently on all RTM exit paths.
if (now - rtm_release_ms > 10000UL)
{
  rtmDisengage();
  break;
}
```

### 5C — COOLDOWN case: remove blocking display (now handled in rtmDisengage())

```c
// OLD
case RTM_COOLDOWN:
  scroll3Digits(LET_S, LET_T, LET_P, 50);
  if (now - rtm_cooldown_ms > 2000UL)
  {
    rtm_tx_state = RTM_IDLE;
  }
  break;

// NEW
case RTM_COOLDOWN:
  // showFullScreenMessage("St P", 2000) was already called at disengage moment.
  // By the time this state runs, 2s has elapsed → go to IDLE immediately.
  if (now - rtm_cooldown_ms > 2000UL)
  {
    rtm_tx_state = RTM_IDLE;
  }
  break;
```

### 5D — Bug 1B: Pre-arm distance check at second-squeeze confirmation

Add this helper above `runRtmLoop()`:

```c
// ---- Decode telemetry.rtm_distance to metres ----
// Returns -1.0f if no valid distance available (telemetry.rtm_distance == 0xFF).
static float decodeRtmDistanceM()
{
  uint8_t d = telemetry.rtm_distance;
  if (d == 0xFF) return -1.0f;
  if (d < 100)   return d / 10.0f;           // tenths of metre (0.0–9.9 m)
  return (float)(d - 90);                    // whole metres (10–164 m)
}
```

In the `RTM_SQUEEZE_WAIT` case, add pre-arm check before transitioning to ACTIVE:

```c
if (thr_scaled > 25)
{
  // V2.5-Evo - 2026-04-28 - P9 Bug1B: Pre-arm distance check.
  // If TX is already within rtm_disengage_distance_m of RX, reject activation.
  // Uses telemetry.rtm_distance — now always populated by RX when GPS is valid.
  // If no distance data (0xFF), skip check gracefully (no GPS = gates will reject on RX).
  float prearm_m = decodeRtmDistanceM();
  if (prearm_m >= 0.0f && prearm_m <= (float)usrConf.rtm_disengage_distance_m)
  {
    Serial.printf("RTM [TX] Pre-arm REJECT: already within %.1f m (limit %u m)\n",
                  prearm_m, usrConf.rtm_disengage_distance_m);
    current_vib_pattern = 4;
    showFullScreenMessage("St P", 2000);
    rtm_tx_state = RTM_IDLE;
    rtm_tx_active = false;
    queueMetaPacketBurst(0xF1, 0);
    break;
  }

  // Second squeeze → ACTIVE
  rtm_tx_state        = RTM_ACTIVE;
  rtm_active_start_ms = now;
  rtm_tx_active       = true;
  rtm_release_ms      = 0;
  queueMetaPacketBurst(0xF1, 1);
  // A rM confirmation flash — fires after RTM is officially active
  showFullScreenMessage("A rM", 2000);
}
```

Apply the same pre-arm check to the single-squeeze mode path in `RTM_ARMED`:

```c
// Single mode: throttle > 30% for 500ms → pre-arm check then ACTIVE
if (now - rtm_hold_start >= 500)
{
  float prearm_m = decodeRtmDistanceM();
  if (prearm_m >= 0.0f && prearm_m <= (float)usrConf.rtm_disengage_distance_m)
  {
    current_vib_pattern = 4;
    showFullScreenMessage("St P", 2000);
    rtm_tx_state   = RTM_IDLE;
    rtm_tx_active  = false;
    rtm_hold_start = 0;
    queueMetaPacketBurst(0xF1, 0);
    break;
  }
  rtm_hold_start      = 0;
  rtm_tx_state        = RTM_ACTIVE;
  rtm_active_start_ms = now;
  rtm_tx_active       = true;
  rtm_release_ms      = 0;
  queueMetaPacketBurst(0xF1, 1);
  showFullScreenMessage("A rM", 2000);
}
```

**Note:** `showFullScreenMessage("A rM", 2000)` is placed AFTER `rtm_tx_active = true` so the loop() display switch sees the correct state when the function returns.

### 5E — Section 2: FM mode full-screen confirmations

In `cycleFmMode()` (arm path), replace `scroll3Digits(LET_F, LET_M, last_fm_mode, 50) × 2` with:

```c
// V2.5-Evo - 2026-04-28 - P9 S2: FM arm confirmation — full-screen flash
char fm_msg[5];
snprintf(fm_msg, sizeof(fm_msg), "FM %u", (unsigned)last_fm_mode);
// Note: 'F'(3) + 'M'(3) + ' '(1) + digit(3) = 10 cols exactly
showFullScreenMessage(fm_msg, 2000);
```

In `fmDisarm()`, replace `scroll3Digits(LET_F, LET_M, DASH, 50)` with:
```c
// FM disarm — re-use "St P" for consistency with RTM disengage
showFullScreenMessage("St P", 2000);
```

In `cycleFmModeArmed()`, replace `showFmMode(last_fm_mode)` with:
```c
char fm_msg[5];
snprintf(fm_msg, sizeof(fm_msg), "FM %u", (unsigned)last_fm_mode);
showFullScreenMessage(fm_msg, 2000);
```

Remove the `showFmMode()` static function (no longer used).

**Note:** `snprintf` with `"FM %u"` produces "FM 0" through "FM 3" — each char is in `fc3x7GetChar()`. The space char is handled as 1 dark column. Digit chars '0'-'3' are in the fontCompact3x7 digit table.

- [ ] Read current RTMState.ino to confirm line numbers
- [ ] Replace `rtmDisengage()` with Pattern 4 + showFullScreenMessage("St P") version
- [ ] Remove `current_vib_pattern = 4` from `setRtmDisarmed()`
- [ ] Change Gate 3 throttle-release path to call `rtmDisengage()` (remove direct IDLE transition)
- [ ] Change COOLDOWN case to remove `scroll3Digits()` (just timer check)
- [ ] Add `decodeRtmDistanceM()` helper
- [ ] Add pre-arm check to double-squeeze SQUEEZE_WAIT transition
- [ ] Add pre-arm check to single-squeeze ARMED transition
- [ ] Add `showFullScreenMessage("A rM", 2000)` after both ACTIVE transitions
- [ ] Replace FM mode display calls with `showFullScreenMessage()` in cycleFmMode(), fmDisarm(), cycleFmModeArmed()
- [ ] Remove `showFmMode()` static function
- [ ] Add version tag: `// V2.5-Evo - 2026-04-28 - P9: Bug1B pre-arm + Bug1D all-exit Pattern4/StP + S2 FM full-screen`
- [ ] **Compile in Arduino IDE (TX board). Fix any errors before proceeding.**

---

## Task 6 — TX BREmote_V2_Tx.h: dist_unit + rtm_arm_dist_m

**Files:**
- Modify: `Source/V2_Integration_Tx/BREmote_V2_Tx.h`

### 6A — Add dist_unit to confStruct (fills tail padding; sizeof stays 128)

Current last field and static_assert:
```c
uint16_t fm_arm_window_s;  // 30s (P8.1)
};
static_assert(sizeof(confStruct) == 128, ...);
```

The struct currently has 126 bytes of data + 2 bytes tail padding = 128. Adding `uint16_t dist_unit` uses those 2 tail padding bytes; sizeof stays exactly 128.

```c
    uint16_t fm_arm_window_s;          // FM auto-disarms after N seconds; 10-60s; default 30

    // ============================================================
    // V2.5-Evo - 2026-04-28 - PRIORITY 9: DISTANCE UNIT SELECTION
    //
    // dist_unit fills the 2-byte tail padding from P8.1; sizeof stays 128.
    // No SPIFFS reset required. Old configs read 0 here (tail padding was zero).
    // 0 (metric) is the correct default, so no migration needed.
    // ============================================================
    uint16_t dist_unit;               // Distance display unit: 0=Metres, 1=Feet; default 0
};
static_assert(sizeof(confStruct) == 128, "confStruct size mismatch — expected 128 bytes (V2.5-Evo P9). Update this assert if you change the struct.");
```

Update `defaultConf` to add `0` as the last initializer:
```c
  30,   // fm_arm_window_s
  0     // dist_unit (0 = Metres; no SPIFFS reset needed — tail padding was already zero)
};
```

### 6B — Add rtm_arm_dist_m RAM global

After the existing RTM globals section, add:

```c
// V2.5-Evo - 2026-04-28 - P9 S4: RTM arm distance captured at engage moment.
// Used by R5 proximity bar to set the 100% reference distance.
// RAM only — never written to SPIFFS. Reset to 0.0f when RTM disengages.
float rtm_arm_dist_m = 0.0f;
```

Update the version tag at the top of the file:
```c
// V2.5-Evo - 2026-04-28 - P9: Added dist_unit (fills tail padding, sizeof stays 128); rtm_arm_dist_m RAM global
```

- [ ] Read current BREmote_V2_Tx.h to confirm exact struct tail and static_assert location
- [ ] Add `dist_unit` field after `fm_arm_window_s` inside confStruct
- [ ] Update static_assert comment to reference V2.5-Evo
- [ ] Add `0` to `defaultConf` as last initializer (for dist_unit)
- [ ] Add `rtm_arm_dist_m` global after RTM globals
- [ ] **Compile in Arduino IDE (TX board). Confirm static_assert does not fire.**

---

## Task 7 — TX ConfigService.ino + WebUiEmbedded.h: dist_unit pipeline

**Files:**
- Modify: `Source/V2_Integration_Tx/ConfigService.ino`
- Modify: `Source/V2_Integration_Tx/WebUiEmbedded.h`

### 7A — ConfigService.ino: add dist_unit to kCfgFields

Add after the `fm_arm_window_s` entry:

```c
  // V2.5-Evo - 2026-04-28 - P9: Distance unit selector. 0=Metres, 1=Feet.
  {"dist_unit", CFG_U16, offsetof(confStruct, dist_unit), true, false, true, 0.0f, 1.0f, 0, false},
```

### 7B — WebUiEmbedded.h: add dist_unit dropdown

Locate the RTM & Follow-Me group section in the HTML. After the `fm_arm_window_s` field, add:

```html
<div class="field">
  <div class="label">Distance Unit</div>
  <div class="desc">Unit used for all distance readouts on the TX display (RTM, FM). Internal math always uses metres.</div>
  <select data-key="dist_unit">
    <option value="0">0 — Metres (1–99 m whole; 100–990 m as X.X km)</option>
    <option value="1">1 — Feet (1–99 ft whole; 100 ft+ as X.X mi)</option>
  </select>
</div>
```

- [ ] Read ConfigService.ino to find insertion point after fm_arm_window_s entry
- [ ] Add `dist_unit` to kCfgFields
- [ ] Read WebUiEmbedded.h to find RTM group insertion point
- [ ] Add dist_unit dropdown HTML
- [ ] **Compile in Arduino IDE (TX board). Verify no syntax errors.**

---

## Task 8 — TX Display.ino: Unit-aware renderRtmInfoDisplay() + R5 proximity bar

**Files:**
- Modify: `Source/V2_Integration_Tx/Display.ino`

**Prerequisite:** Task 6 (dist_unit in confStruct) and Task 4 (showFullScreenMessage exists) must be complete.

### 8A — Unit conversion helper

Add above `renderRtmInfoDisplay()`:

```c
// ============================================================
// V2.5-Evo - 2026-04-28 - P9 S3: Distance unit conversion for TX display.
// All inputs are metres (float). Output: displayDigits() calls + optional
// decimal dot at C3 R4 (displayBuffer[5] bit 3).
// Rules:
//   Metric (0):   1–99 m → whole metres; 100–990 m → X.X km (decimal dot)
//   Imperial (1): 1–99 ft → whole feet; 100 ft+ → X.X miles (decimal dot)
// Below minimum: show "00". 0xFF input → show "--".
// ============================================================
static void displayDistanceInUnits(float dist_m)
{
  const uint16_t unit = usrConf.dist_unit;

  if (unit == 0)
  {
    // Metric
    if (dist_m < 1.0f)
    {
      displayDigits(0, 0);  // show "00" — below 1m minimum
    }
    else if (dist_m < 100.0f)
    {
      // Whole metres 1–99
      uint8_t m = (uint8_t)dist_m;
      if (m > 99) m = 99;
      displayDigits(m / 10, m % 10);
    }
    else
    {
      // Tenths of km: 100m → 1.0, 180m → 1.8, 990m → 9.9 (cap at 9.9)
      uint8_t tenths = (uint8_t)(dist_m / 100.0f);  // 100m → 1, 180m → 1 (floor)
      uint8_t frac   = (uint8_t)((dist_m / 100.0f - tenths) * 10.0f + 0.5f);  // round
      if (tenths > 9) { tenths = 9; frac = 9; }
      displayDigits(tenths, frac);
      displayBuffer[5] |= (1u << 3);  // decimal dot C3 R4 — must be after displayDigits()
    }
  }
  else if (unit == 1)
  {
    // Imperial
    float dist_ft = dist_m * 3.28084f;
    if (dist_ft < 4.0f)
    {
      displayDigits(0, 0);  // below 4ft minimum
    }
    else if (dist_ft < 100.0f)
    {
      uint8_t ft = (uint8_t)dist_ft;
      if (ft > 99) ft = 99;
      displayDigits(ft / 10, ft % 10);
    }
    else
    {
      // Decimal miles: 528ft = 0.1 mi, 5280ft = 1.0 mi
      float miles = dist_ft / 5280.0f;
      uint8_t whole = (uint8_t)miles;
      uint8_t frac  = (uint8_t)((miles - whole) * 10.0f + 0.5f);
      if (whole > 9) { whole = 9; frac = 9; }
      displayDigits(whole, frac);
      displayBuffer[5] |= (1u << 3);
    }
  }
  }
}
```

### 8B — Update renderRtmInfoDisplay() distance block

Replace the existing distance display block (the `if (mode == 0)` branch):

```c
  if (mode == 0)
  {
    // Distance mode — decode telemetry.rtm_distance then convert to selected unit
    uint8_t d = telemetry.rtm_distance;
    if (d == 0xFF)
    {
      displayDigits(DASH, DASH);
    }
    else
    {
      // Decode to metres (same encoding as RX RTMState.ino)
      float actual_m;
      if (d < 100)
        actual_m = d / 10.0f;
      else
        actual_m = (float)(d - 90);

      displayDistanceInUnits(actual_m);
    }
  }
```

### 8C — R5 Proximity Bar

Add after `renderRtmInfoDisplay()`:

```c
// ============================================================
// V2.5-Evo - 2026-04-28 - P9 S4: R5 proximity bar.
// Active during RTM or FM mode. Blinks 1000ms on / 500ms off.
// Suppressed automatically during showFullScreenMessage() (buffer is cleared,
// and this function is not called during blocking messages).
//
// RTM bar: square-root curve from arm distance to 0.
//   At arm moment: capture rtm_arm_dist_m.
//   Pixel count = round(sqrt(current_m / arm_m) * 10), clamped 0-10.
//   Fills left to right C0-C9; shrinks from right as buggy closes in.
//
// FM bar: stub — single center pixel C4R5 when at ideal follow distance.
//   Full implementation deferred to Priority 10 (FM full implementation).
// ============================================================
void updateR5ProximityBar()
{
  static unsigned long r5_blink_ms    = 0;
  static bool          r5_blink_state = false;

  // Blink timer: 1000ms on, 500ms off
  unsigned long now = millis();
  if (r5_blink_state)
  {
    if (now - r5_blink_ms >= 1000UL)
    {
      r5_blink_state = false;
      r5_blink_ms    = now;
    }
  }
  else
  {
    if (now - r5_blink_ms >= 500UL)
    {
      r5_blink_state = true;
      r5_blink_ms    = now;
    }
  }

  // Always clear R5 first (bit 0-9 of displayBuffer[6])
  displayBuffer[6] = 0x0000;

  if (!r5_blink_state) return;  // off phase — leave R5 dark

  if (rtm_tx_active)
  {
    // ---- RTM proximity bar ----
    if (rtm_arm_dist_m <= 0.0f)
    {
      return;  // no valid arm distance captured — skip bar to prevent divide-by-zero
    }

    uint8_t d = telemetry.rtm_distance;
    if (d == 0xFF) return;  // no distance data yet — leave R5 dark

    float current_m;
    if (d < 100) current_m = d / 10.0f;
    else         current_m = (float)(d - 90);

    // Square-root curve: pixel_count = round(sqrt(current/arm) * 10)
    float ratio = current_m / rtm_arm_dist_m;
    if (ratio > 1.0f) ratio = 1.0f;  // clamp (current > arm = just started, show full bar)
    uint8_t pixels = (uint8_t)(sqrtf(ratio) * 10.0f + 0.5f);  // 0-10
    if (pixels > 10) pixels = 10;

    // Fill C0 to C(pixels-1) in R5 (displayBuffer[6])
    for (uint8_t c = 0; c < pixels; c++)
    {
      displayBuffer[6] |= (1u << c);
    }
  }
  else if (fm_armed)  // FM bar (stub — Priority 10)
  {
    // TODO: FM bar — full implementation deferred to Priority 10.
    // Stub: single center pixel C4 R5 lit when FM is armed.
    // Full version: grows from C4 outward as distance error increases,
    // up to full bar C0-C9 at fm_warn_distance_m error.
    displayBuffer[6] |= (1u << 4);  // C4 R5 — center pixel placeholder
  }
}
```

**Capture `rtm_arm_dist_m` in RTMState.ino (add to Task 5 or as amendment):**

In RTMState.ino, at both ACTIVE transition points (double-squeeze and single-squeeze), after `rtm_tx_active = true` but before `showFullScreenMessage("A rM", 2000)`, add:

```c
// Capture current distance for R5 proximity bar reference
rtm_arm_dist_m = decodeRtmDistanceM();
if (rtm_arm_dist_m < 0.0f) rtm_arm_dist_m = 0.0f;  // treat invalid as no-data
```

And in `rtmDisengage()`, reset it:
```c
rtm_arm_dist_m = 0.0f;  // reset so bar clears on next disengage
```

**Call `updateR5ProximityBar()` from `renderRtmInfoDisplay()`:** Add at the end of `renderRtmInfoDisplay()`, before `updateDisplay()`:

```c
  updateR5ProximityBar();
  updateDisplay();
```

Remove the existing `updateDisplay()` call from the end of `renderRtmInfoDisplay()` (it's now called once after the bar update).

**Also declare `fm_armed` as extern in Display.ino** (it's defined in RTMState.ino):
```c
extern bool fm_armed;  // defined in RTMState.ino — needed by updateR5ProximityBar()
```

And declare `rtm_arm_dist_m` as extern in RTMState.ino (it's defined in BREmote_V2_Tx.h):
```c
extern float rtm_arm_dist_m;  // defined in BREmote_V2_Tx.h — captured at RTM engage
```

- [ ] Read Display.ino to confirm current renderRtmInfoDisplay() structure and insertion points
- [ ] Add `displayDistanceInUnits()` helper before `renderRtmInfoDisplay()`
- [ ] Update `renderRtmInfoDisplay()` distance mode block to use `displayDistanceInUnits()`
- [ ] Add `updateR5ProximityBar()` function after `renderRtmInfoDisplay()`
- [ ] Add `updateR5ProximityBar()` call at end of `renderRtmInfoDisplay()` before `updateDisplay()`
- [ ] Add `extern bool fm_armed;` to Display.ino
- [ ] Add `rtm_arm_dist_m` capture + `extern` in RTMState.ino (amendment to Task 5)
- [ ] Add `rtm_arm_dist_m = 0.0f` reset in `rtmDisengage()` (amendment to Task 5)
- [ ] Add version tag: `// V2.5-Evo - 2026-04-28 - P9 S3+S4: dist_unit display + R5 proximity bar`
- [ ] **Compile in Arduino IDE (TX board). Fix any errors before proceeding.**

---

## Task 9 — docs/display-reference.md: Section 5 update

**Files:**
- Modify: `docs/display-reference.md`

Replace the document title and update/add the following sections:

1. **Title:** "BREmote V2.5-Evo — Dot Display Reference"
2. **Full-screen message table** with pixel column layouts for each message
3. **Note:** "StP (no space, scroll) removed — only St P (full-screen via showFullScreenMessage) exists"
4. **R5 bar behavior:** RTM (sqrt curve, rtm_arm_dist_m captured at engage, never to SPIFFS), FM stub (center pixel C4)
5. **dist_unit parameter table:** both modes (Metres / Feet) and conversion rules, minimum display values
6. **rtm_arm_dist_m:** RAM variable captured at RTM engage moment, reset on disengage, never to SPIFFS
7. **Decimal dot C3R4:** reused for all three unit modes (displayBuffer[5] |= (1<<3))
8. **Minimum display distance:** 1m / 4ft → "00"; below these GPS accuracy doesn't justify display
9. **Reference to Dot_Matrix_Display_10x7_Render.html:** noted as canonical font and layout reference
10. **All messages and their exact column counts:** A rM (10), St P (10), FM 0-3 (10), E 71 (10)

- [ ] Read current display-reference.md to identify sections to update vs add
- [ ] Update title and any "V3" occurrences
- [ ] Add full-screen message table
- [ ] Add R5 proximity bar section
- [ ] Add dist_unit section
- [ ] Add runtime-only variables section (rtm_arm_dist_m)
- [ ] Add canonical font reference note

---

## Task 10 — Final commit

```
git add Source/V2_Integration_Rx/RTMState.ino \
        Source/V2_Integration_Tx/Display.ino \
        Source/V2_Integration_Tx/RTMState.ino \
        Source/V2_Integration_Tx/BREmote_V2_Tx.h \
        Source/V2_Integration_Tx/ConfigService.ino \
        Source/V2_Integration_Tx/WebUiEmbedded.h \
        docs/display-reference.md \
        docs/Dot_Matrix_Display_10x7_Render.html \
        CLAUDE.md
```

Commit message:
```
feat(P9): gate fixes, full-screen confirmations, R5 bar, dist units, V2.5-Evo rename

Section 0: Project renamed BREmote V3 → BREmote V2.5-Evo across docs and web UI.
Section 1 bugs:
  1A: Gate 9 zero-guard — SPIFFS-stored rtm_stop_distance_m=0 no longer disables hard stop
  1B: Pre-arm distance check at second-squeeze; RX always computes telemetry.rtm_distance
  1C: Distance computed before safety gates — no telemetry-cycle delay on RTM engage
  1D: Pattern 4 + "St P" in rtmDisengage(); all exit paths now consistent
Section 2: fontCompact3x7 + showFullScreenMessage(); A rM / St P / FM 0-3 / E 71 full-screen
Section 3: dist_unit (metric/feet) — fills tail padding, sizeof TX stays 128
Section 4: R5 proximity bar — sqrt curve for RTM, FM stub
Section 5: display-reference.md updated with all P9 features

Safety: no autonomous throttle additions; all RTM gates unchanged; creator safety philosophy enforced.

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
```

- [ ] Run `git status` — confirm only expected files are staged
- [ ] Confirm both TX and RX compile clean
- [ ] Run `git commit` with above message

---

## Self-Review Against Spec

### Spec coverage check

| Requirement | Task | Status |
|---|---|---|
| S0: Global rename BREmote V3 → V2.5-Evo | Task 2 | Covered |
| S0: Copy HTML to docs/ as canonical reference | Task 3 prereq | Covered (user action) |
| S1 Bug 1A: Gate 9 never fires | Task 1 | Covered — zero-guard on rtm_stop_distance_m |
| S1 Bug 1B: Pre-arm distance check at second-squeeze | Task 5 (TX) + Task 1 (RX always-compute) | Covered |
| S1 Bug 1C: Display delay after RTM engages | Task 1 (RX always-compute before gates) | Covered |
| S1 Bug 1D: Pattern 4 on all RTM exits | Task 5 (rtmDisengage + Gate1 path) | Covered |
| S2: fontCompact3x7 from HTML bitmaps only | Task 4 (scaffold + extraction) | Covered — blocked on HTML prereq |
| S2: showFullScreenMessage() — clears all 10×7 including R5/R6 | Task 4 | Covered |
| S2: A rM on double-squeeze confirm (2s) | Task 5 | Covered |
| S2: St P on ALL RTM gate exits (2s) | Task 5 (via rtmDisengage) | Covered |
| S2: FM 0-3 confirms (2s each) | Task 5 (cycleFmMode, cycleFmModeArmed) | Covered |
| S2: E71 full-screen flash 250ms, no auto-clear | Task 4 | Covered |
| S2: Remove StP (no-space scroll3Digits variant) | Task 5 (COOLDOWN case) | Covered |
| S3: dist_unit SPIFFS param 0/1 | Task 6 (struct), Task 7 (config+UI), Task 8 (display) | Covered |
| S3: All internal math stays metres | Tasks 8 (conversion only in display layer) | Covered |
| S3: Minimum 1m/4ft → "00" | Task 8 (displayDistanceInUnits) | Covered |
| S3: Decimal dot C3R4 for both unit modes | Task 8 | Covered |
| S4: R5 bar active during RTM/FM only | Task 8 (updateR5ProximityBar) | Covered |
| S4: 1000ms on / 500ms off blink, millis-based | Task 8 | Covered |
| S4: RTM bar sqrt curve, rtm_arm_dist_m capture | Task 8 + amendment to Task 5 | Covered |
| S4: FM bar stub, compiles, no crash | Task 8 (stub with TODO comment) | Covered |
| S4: Bar suppressed during showFullScreenMessage | Task 4 (clearAllBuffer) + Task 8 (bar only in renderRtmInfoDisplay) | Covered |
| S5: display-reference.md updated | Task 9 | Covered |

### Known limitations noted in plan
- `rtm_arm_dist_m` may be 0.0f if telemetry.rtm_distance is 0xFF at engage moment (no GPS); bar is suppressed in that case — safe.
- Font bitmaps are placeholders until Task 3 extraction from HTML. Task 4 cannot be finalized without the HTML file.

### Placeholder scan
- fontCompact3x7 bitmaps show `{0x00, 0x00, 0x00}` — intentional placeholder; must be replaced in Task 4.
- No other "TBD" or "TODO" except the FM bar stub (which is explicitly marked per spec: `// TODO: FM bar — full implementation deferred to Priority 10`).

### Type consistency
- `decodeRtmDistanceM()` returns `float` — used consistently in Task 5 (pre-arm check) and Task 8 (R5 bar capture).
- `dist_unit` is `uint16_t` everywhere (confStruct, ConfigService offset, web UI value).
- `rtm_arm_dist_m` declared as `float` in BREmote_V2_Tx.h and `extern float` in RTMState.ino.
- `showFullScreenMessage(const char* msg, uint16_t duration_ms)` — consistent signature in Task 4 definition and Task 5 callsites.
- `fm_armed` — declared `extern bool` in Display.ino, defined as `static bool` in RTMState.ino. **Issue:** static linkage prevents extern access. Fix: remove `static` from `fm_armed` declaration in RTMState.ino. Add this to Task 5 checklist.
