# Priority 4: Signal Drop Vibration Warning — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fire a one-shot Pattern A (2 Short haptic buzz) when LoRa signal quality `sq_graph` drops to bargraph level 1 while the TX is still connected, warning the rider before full failsafe.

**Architecture:** Add edge-detection logic inside `vibrationTask()` in `System.ino`. Three static local variables gate the warning: connection state, previous sq_graph value, and a re-arm flag. The warning only fires while connected (last_packet < 1000 ms) to prevent false triggers from the failsafe blink cycle (which also toggles sq_graph 0↔1). No new SPIFFS parameters, no Web UI changes.

**Tech Stack:** Arduino/ESP32-C3 FreeRTOS; `volatile uint8_t sq_graph` (BREmote_V2_Tx.h:292); `P_MOT` GPIO 0; `vibrationTask()` in System.ino.

---

## Background — How sq_graph Works

`updateBargraphs()` in `Display.ino` runs every 200 ms on core 0:

- **Connected** (`millis() - last_packet < 1000`): `sq_graph` = `map(telemetry.link_quality + local_link_quality, 0, 20, 0, 5)` → values 0–5
- **Failsafe** (no packet in 1 s): toggles `sq_graph` between 0 and 1 as a display-blink artifact — NOT a real signal reading

The warning must only fire when `sq_graph` drops 2→1 (or higher→1) **while connected**. When in failsafe, `sq_graph` oscillates 0↔1 artificially — the connection-lost Pattern B (5 Short) already covers that state.

## Pattern Assignments (existing + new)

| Pattern value | Physical | Trigger |
|---|---|---|
| 1 (A) | 2 Short | Low VESC battery (≤20 %) **and** Weak LoRa signal (sq_graph → 1 while connected) |
| 2 (B) | 5 Short | Radio failsafe — connection lost |
| 3 (C) | 5 Long  | Error 71 — water ingress |

Pattern A is shared between battery warning and signal warning because both are "yellow flag" events. They play identically.

---

## Files

| File | Change |
|---|---|
| `Source/V2_Integration_Tx/System.ino` | **Modify** `vibrationTask()` — add ~15 lines in "MONITOR SYSTEM STATES" block |

No other files change. No confStruct change. No SPIFFS migration. No Web UI update.

---

## Task 1: Add Signal-Drop Edge Detection to vibrationTask()

**Files:**
- Modify: `Source/V2_Integration_Tx/System.ino:654–712` (the `vibrationTask()` function)

### Context — current "MONITOR" block (lines ~659–681)

```cpp
void vibrationTask(void *parameter) {
  uint8_t last_error = 0;
  bool was_connected = true;
  bool bat_warning_sent = false;

  while (1) {
    // --- 1. MONITOR SYSTEM STATES ---

    // Check for Radio Failsafe (Connection Lost)
    bool is_connected = (millis() - last_packet < 1000);
    if (was_connected && !is_connected) {
      current_vib_pattern = 2; // Pattern B: 5 Short (Urgent!)
    }
    was_connected = is_connected;

    // Check for Critical Errors (Like E71 Water Ingress)
    if (remote_error != last_error) {
      if (remote_error == 71) {
        current_vib_pattern = 3; // Pattern C: 5 Long (Emergency!)
      }
      last_error = remote_error;
    }

    // Check for Low VESC Battery (20% or less)
    if (telemetry.foil_bat != 0xFF && telemetry.foil_bat <= 20) {
      if (!bat_warning_sent) {
        current_vib_pattern = 1; // Pattern A: 2 Short (Warning)
        bat_warning_sent = true;
      }
    } else if (telemetry.foil_bat > 20) {
      bat_warning_sent = false; // Reset if battery is changed
    }
```

- [ ] **Step 1: Add three static locals for signal-drop state tracking**

  Insert the three variables at the top of `vibrationTask()`, immediately after `bool bat_warning_sent = false;` (line ~657).

  These must be local (not global) — they are private state of this task.

  ```cpp
  // Signal-drop warning state (Priority 4)
  uint8_t last_sq    = 0;      // sq_graph reading from previous loop iteration
  bool    sq_warned  = false;  // true after Pattern A fired; cleared when signal recovers
  bool    last_con   = true;   // connection state from previous iteration (mirrors was_connected)
  ```

  After the edit, the top of `vibrationTask()` reads:

  ```cpp
  void vibrationTask(void *parameter) {
    uint8_t last_error = 0;
    bool was_connected = true;
    bool bat_warning_sent = false;
    // Signal-drop warning state (Priority 4)
    uint8_t last_sq    = 0;
    bool    sq_warned  = false;
    bool    last_con   = true;
  ```

- [ ] **Step 2: Add signal-drop detection block inside the MONITOR section**

  Insert immediately after the `was_connected = is_connected;` line (and before the error check), so the new code can read the freshly computed `is_connected`:

  ```cpp
    // Check for Weak LoRa Signal (sq_graph drops to 1 while connected)
    // sq_graph == 1 means one bar of signal left — warn before full failsafe (sq_graph == 0).
    // Guard: only while connected; during failsafe, updateBargraphs() toggles sq_graph 0↔1
    // as a display artifact — that state is already covered by Pattern B above.
    // Re-arm when signal recovers above 1 so the warning can fire again on the next drop.
    {
      uint8_t cur_sq = sq_graph; // snapshot volatile once
      if (is_connected) {
        if (!last_con) {
          // Just reconnected — seed last_sq to suppress a spurious edge on reconnect
          last_sq   = cur_sq;
          sq_warned = false;
        } else if (last_sq > 1 && cur_sq == 1 && !sq_warned) {
          // Signal just dropped to 1 bar — fire Pattern A if nothing else is playing
          if (current_vib_pattern == 0) {
            current_vib_pattern = 1; // Pattern A: 2 Short (weak signal warning)
          }
          sq_warned = true;
        } else if (cur_sq > 1) {
          // Signal recovered — allow warning to fire again on next drop
          sq_warned = false;
        }
        last_sq = cur_sq;
      } else {
        // In failsafe — reset so warning re-arms on reconnect
        sq_warned = false;
      }
      last_con = is_connected;
    }
  ```

  **Full resulting MONITOR block** (paste this to replace lines 659–681):

  ```cpp
    // --- 1. MONITOR SYSTEM STATES ---

    // Check for Radio Failsafe (Connection Lost)
    bool is_connected = (millis() - last_packet < 1000);
    if (was_connected && !is_connected) {
      current_vib_pattern = 2; // Pattern B: 5 Short (Urgent!)
    }
    was_connected = is_connected;

    // Check for Weak LoRa Signal (sq_graph drops to 1 while connected)
    // sq_graph == 1 means one bar of signal left — warn before full failsafe (sq_graph == 0).
    // Guard: only while connected; during failsafe, updateBargraphs() toggles sq_graph 0↔1
    // as a display artifact — that state is already covered by Pattern B above.
    // Re-arm when signal recovers above 1 so the warning can fire again on the next drop.
    {
      uint8_t cur_sq = sq_graph; // snapshot volatile once
      if (is_connected) {
        if (!last_con) {
          // Just reconnected — seed last_sq to suppress a spurious edge on reconnect
          last_sq   = cur_sq;
          sq_warned = false;
        } else if (last_sq > 1 && cur_sq == 1 && !sq_warned) {
          // Signal just dropped to 1 bar — fire Pattern A if nothing else is playing
          if (current_vib_pattern == 0) {
            current_vib_pattern = 1; // Pattern A: 2 Short (weak signal warning)
          }
          sq_warned = true;
        } else if (cur_sq > 1) {
          // Signal recovered — allow warning to fire again on next drop
          sq_warned = false;
        }
        last_sq = cur_sq;
      } else {
        // In failsafe — reset so warning re-arms on reconnect
        sq_warned = false;
      }
      last_con = is_connected;
    }

    // Check for Critical Errors (Like E71 Water Ingress)
    if (remote_error != last_error) {
      if (remote_error == 71) {
        current_vib_pattern = 3; // Pattern C: 5 Long (Emergency!)
      }
      last_error = remote_error;
    }

    // Check for Low VESC Battery (20% or less)
    if (telemetry.foil_bat != 0xFF && telemetry.foil_bat <= 20) {
      if (!bat_warning_sent) {
        current_vib_pattern = 1; // Pattern A: 2 Short (Warning)
        bat_warning_sent = true;
      }
    } else if (telemetry.foil_bat > 20) {
      bat_warning_sent = false; // Reset if battery is changed
    }
  ```

- [ ] **Step 3: Add V3 version tag at the top of System.ino**

  The file does not yet have a version tag line. Prepend one before line 1:

  ```cpp
  // V3 - 2026-04-22 - P4: signal-drop haptic warning (Pattern A) when sq_graph drops to 1 while connected
  ```

- [ ] **Step 4: Compile in Arduino IDE**

  Open `Source/V2_Integration_Tx/V2_Integration_Tx.ino` in Arduino IDE.
  Select board: **ESP32C3 Dev Module** (or your specific board variant).
  Click **Verify** (Ctrl+R).

  Expected: **0 errors, 0 warnings** (same as before — the change is pure logic with no new types or includes).

  If compile fails, check:
  - `last_sq`, `sq_warned`, `last_con` are declared before the `while(1)` loop
  - `sq_graph` is not redeclared locally (it's the global `volatile uint8_t` from `BREmote_V2_Tx.h`)

- [ ] **Step 5: Flash and verify manually**

  Flash to TX hardware. To verify the warning fires correctly, simulate weak signal:

  **Test A — Warning fires on drop:**
  1. Power on TX and RX, confirm link established (sq_graph > 1 = 2+ bars on display C9)
  2. Move TX far from RX or attenuate RF until display shows 1 bar (sq_graph == 1)
  3. Expected: **2 short buzzes** from haptic motor within ~50 ms of sq dropping to 1

  **Test B — Warning does not spam:**
  4. Keep TX at weak-signal distance (sq_graph stays 1)
  5. Expected: No repeated buzzing. Warning fires once only.

  **Test C — Warning re-arms after recovery:**
  6. Move TX back close (sq_graph recovers to 2+), then move away again to sq_graph == 1
  7. Expected: **2 short buzzes** again (warning re-armed after recovery)

  **Test D — No false trigger during failsafe:**
  8. Power off RX while TX is on — TX enters failsafe (sq_graph toggles 0↔1)
  9. Expected: Pattern B (5 Short) fires once on connection loss, then **no** Pattern A fires during failsafe blink

  **Test E — Reconnect does not trigger spurious warning:**
  10. Power RX back on — link re-establishes
  11. Expected: No spurious Pattern A buzz on reconnect (last_sq seeding suppresses edge)

- [ ] **Step 6: Commit**

  ```bash
  git add Source/V2_Integration_Tx/System.ino
  git commit -m "feat(TX): P4 signal-drop haptic — Pattern A when sq_graph drops to 1 while connected"
  ```

---

## Self-Review Checklist

- [x] **Spec coverage:** CLAUDE.md P4 — "Haptic Pattern A triggered when sq_graph drops to 1 (LoRa signal loss warning), TX-side only, uses existing vibration motor infrastructure." All three requirements covered.
- [x] **No placeholders:** All code shown in full. No "TBD" or "TODO" in plan body.
- [x] **Type consistency:** `sq_graph` is `volatile uint8_t` throughout. `last_sq` is `uint8_t`. `cur_sq` is `uint8_t`. All consistent.
- [x] **Safety rule (Section 9):** Change is TX display/haptic only. No motor control, no throttle logic, no RX changes. Safety rule not implicated.
- [x] **Section 10 (Web UI Rule):** No new SPIFFS/confStruct parameters added. Rule does not apply.
- [x] **One file only:** Only `System.ino` changes. Matches CLAUDE.md "one file at a time" rule.
- [x] **False-trigger guard:** `last_con` / reconnect seed prevents spurious edge on reconnect. `!sq_warned` prevents repeated firing while sq stays at 1. Failsafe blink excluded via `is_connected` gate.
