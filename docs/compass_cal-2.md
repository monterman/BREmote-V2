# Claude Code Prompt — BIND Button Compass Calibration

**BREmote V2.5-Evo | RX firmware | System.ino only**

---

## Context

This is a single-file change to `Source/V2_Integration_Rx/System.ino`.

The BIND button (`AP_S_BIND`) currently has no runtime function — it only acts at boot
(held at boot = pairing mode, held at boot + AUX = factory reset). After boot, pressing
BIND does nothing. We are repurposing this idle runtime press to trigger compass
calibration without needing a serial connection or computer.

The compass calibration function `runCompassCalibration()` already exists in
`Compass.ino` and handles everything: 45-second collection loop, hard-iron offset
calculation, soft-iron scale calculation, and auto-save to SPIFFS via `cmdSave()`.
We only need to call it from a button press.

---

## Task

**Add runtime BIND button detection for compass calibration to `checkButtons()` in
`System.ino`.**

Do not modify any other file. Do not modify `runCompassCalibration()` in `Compass.ino`.
Do not modify any SPIFFS struct or config service. Plan first, show the full diff, wait
for approval before editing.

---

## Exact behaviour required

1. User presses BIND button at runtime (after boot — not during pairing or factory reset).
2. 50 ms debounce — confirm button is still held.
3. `blinkBind(5)` — 5 fast blinks on BIND LED (50 ms cadence) to confirm cal is starting.
4. Call `runCompassCalibration()` — runs for 45 seconds, saves result to SPIFFS automatically.
   Do NOT touch the BIND LED during the 45 seconds. The `checkConnStatus` task will drive
   it normally during this window. That is intentional.
5. On return from `runCompassCalibration()`:
   - **Success** (function completed normally): `blinkBind(2)` — 2 fast blinks.
   - **Error** (compass not detected, i.e. `compass_detected == false`): `blinkBind(10)` — 10 fast blinks.
6. Wait for user to release BIND button before returning.

`blinkBind()` already exists in `System.ino` and uses 50 ms cadence — use it as-is,
no changes needed.

---

## LED reference (do not conflict with these)

| Button | Event             | LED      | Blinks | Cadence                   |
| ------ | ----------------- | -------- | ------ | ------------------------- |
| AUX    | Logger START      | AUX LED  | 5      | 200 ms (`blinkErr`)       |
| AUX    | Logger STOP       | AUX LED  | 2      | 200 ms (`blinkErr`)       |
| BIND   | Cal START confirm | BIND LED | 5      | 50 ms (`blinkBind`) ← NEW |
| BIND   | Cal SUCCESS       | BIND LED | 2      | 50 ms (`blinkBind`) ← NEW |
| BIND   | Cal ERROR         | BIND LED | 10     | 50 ms (`blinkBind`) ← NEW |

Fast 50 ms cadence on BIND distinguishes compass cal from the slower 200 ms AUX logger
blinks. Same count logic: 5 = started, 2 = done cleanly, 10 = error.

---

## How to detect success vs error

`runCompassCalibration()` in `Compass.ino` returns `void`. The reliable way to detect
failure is to check `compass_detected` — the global `bool` declared in `Compass.ino`
that is set during `initCompass()`. If `compass_detected == false`, the calibration
could not read the sensor. Check this after the call returns.

Declare `extern bool compass_detected;` at the top of the new block if not already
visible in `System.ino` scope.

---

## Where to insert the code

Inside `checkButtons()` in `System.ino`, **after** the existing BIND boot-time block
and **after** the existing AUX logger toggle block. Add the new block at the end of
`checkButtons()`, before the closing `}`.

Use the same static-variable falling-edge pattern already used for the AUX logger toggle:

```cpp
static bool bind_last_state = true;  // true = HIGH (unpressed, pullup active)
bool bind_current = aw.digitalRead(AP_S_BIND);

if (bind_last_state == true && bind_current == false) {
    vTaskDelay(pdMS_TO_TICKS(50));  // debounce
    if (aw.digitalRead(AP_S_BIND) == false) {
        // ... cal sequence here ...
        while (aw.digitalRead(AP_S_BIND) == false) { vTaskDelay(pdMS_TO_TICKS(10)); }
    }
}
bind_last_state = bind_current;
```

---

## Comment requirements (per CLAUDE.md §3)

- Header comment block on the new code section explaining: what it does, why BIND is
  safe to reuse at runtime, and that it does not affect boot-time pairing.
- Inline comment explaining the `compass_detected` extern and why it's used for
  success/error detection instead of a return value.
- Version tag on the file: `// V3 - [today's date] - Added runtime BIND press →
  compass calibration with LED feedback`

---

## Safety checklist before editing

- [ ] Confirm `runCompassCalibration()` is declared/defined in `Compass.ino` (not
  in a header) — it is a free function, no class needed.
- [ ] Confirm `compass_detected` is a global `bool` in `Compass.ino` scope.
- [ ] Confirm `blinkBind(int num)` exists in `System.ino` and uses `AP_L_BIND`.
- [ ] Confirm the existing boot-time BIND block inside `checkButtons()` uses an
  early-return or blocking call (`waitForPairing()`) so it never falls through to
  the new runtime block.
- [ ] No changes to `confStruct`, `ConfigService`, `WebUiEmbedded.h`, or any other
  file.

---

## Files to edit

| File                                  | Change                            |
| ------------------------------------- | --------------------------------- |
| `Source/V2_Integration_Rx/System.ino` | Add ~30 lines to `checkButtons()` |

**No other files.**

---

## After the edit

Remind Andres to:

1. Compile in Arduino IDE (ESP32-S3, Huge APP partition, USB CDC On Boot = Disabled).
2. Flash RX.
3. Field test: power on RX fully booted and paired, press BIND once, confirm 5 fast
   blinks on BIND LED, rotate buggy 360° twice over 45 seconds, confirm 2 blinks
   (success) or 10 blinks (compass not detected).
