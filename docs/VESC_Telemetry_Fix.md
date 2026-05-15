# VESC Telemetry Fix — Analysis & Implementation Plan

**BREmote V2.5-Evo | RX firmware | Filed: 2026-05-09**
**Status: Root cause confirmed and fixed — SW51–SW55 (2026-05-14). See resolution section below.**

---

## SW51–SW55 Resolution — 2026-05-14

### Primary Root Cause (Field Discovery)

**USB-C serial cable on GPIO 18/19 completely silences VESC UART.**

The ESP32-C3 native USB peripheral's D− and D+ lines share GPIO 18 and GPIO 19 with Serial1 (the UART used for GPS and VESC via the hardware MUX). Any USB-C cable plugged into the HT-CT62 during field operation overrides Serial1, dropping all VESC UART traffic to zero.

**Operational fix:** Unplug the USB-C cable before field use. This is a hardware constraint of the ESP32-C3 — no firmware change can eliminate it.

---

### Code Defects Fixed in SW55

Three additional firmware defects were found and fixed as part of the investigation:

**1. GPS MUX never yielding**

`getGPSLoop()` and `configureGPS()` were switching the UART MUX to GPS (channel 1) but never returning it to VESC (channel 0) on exit. VESC was left waiting indefinitely on every GPS poll cycle.

Fix: `setUartMux(0)` added at the end of both `getGPSLoop()` and `configureGPS()`. GPS always has priority on the MUX; GPS always yields the bus back to VESC when done.

**2. `rcv_err` flag persistence bug**

`receiveFromVESC()` set `rcv_err = true` on any bad first byte but never cleared the flag within the 200 ms receive window. One stray byte in the UART buffer poisoned the entire receive attempt for that cycle.

Fix: `rcv_err` flag removed entirely from `receiveFromVESC()`. CRC handles frame validation — the flag was redundant and harmful.

**3. Boot MUX state undefined**

`configureGPS()` (called at boot) left the MUX on GPS channel (channel 1) at exit. The first VESC poll after boot could fail if the MUX was never switched back.

Fix: `configureGPS()` now ends with `setUartMux(0)`. Boot sequence starts with the MUX on the VESC channel.

---

### SW54 Revert (MUX Retry Loops)

SW51 and SW52 added retry loops that re-attempted failed `setUartMux()` I2C writes 3–5 times in rapid succession. Under field conditions, the rapid I2C writes caused AW9523 bus corruption, manifesting as GPS chars=0 and VESC zero-packet responses — the opposite of the intended fix.

SW54 reverted all MUX retry logic. Single `setUartMux()` calls are reliable when the bus is not stressed by rapid retries.

---

### Status of Original Findings After SW51–SW55

| Finding | Description | Status |
|---|---|---|
| 1 | `Serial1.flush()` drains TX not RX | Fixed — removed (2026-05-11) |
| 2 | GPS/VESC share single 1 Hz timer | Partially mitigated — MUX discipline (SW55) reduces contention; independent timers not yet implemented |
| 3 | `foil_speed` / `foil_power` not reset on timeout | Fixed — `foil_power` and `foil_motor_amps` added to timeout reset block (SW49/SW50) |
| 4 | `vesc_timeout_s` default 12 s too long | Default now configurable via SPIFFS; field-appropriate value to be set after water test |
| 5 | E7 wetness de-latch path missing | Still pending — requires log evidence from a session where E7 triggers |

---

## Context & Philosophy

This fix session targets VESC telemetry display flicker visible on the TX dot matrix
during active riding. Before any implementation, the standing safety rule applies:

> Manual throttle control and LoRa packet handling are ALWAYS higher priority than
> telemetry. A failed telemetry read is acceptable. A failed throttle stop or a
> delayed LoRa packet is not. Any fix that introduces scheduling jitter on Core 0
> (generatePWM / triggeredReceive) must be rejected regardless of telemetry benefit.

---

## Exact Execution Model (Read From Source)

```
Core 1 — Arduino loop task
  esp_task_wdt_reset()
  webCfgLoop()
  checkSerial()
  loggerLoop()
  if millis()-loop_timer > 1000:
    wetness check (every 10 iterations = every 10s)
    getGPSLoop()        ← Serial1 via MUX position 1, 0-300ms
    getVescLoop()       ← Serial1 via MUX position 0
      setUartMux(0)
      vTaskDelay(10ms)  ← blocks Core 1
      Serial1.flush()   ← WRONG: drains TX not RX
      sendToVESC()
      receiveFromVESC() ← 200ms blocking timeout

Core 0 — FreeRTOS tasks
  generatePWM     priority 10, every 10ms   ← THROTTLE/STEERING (safety critical)
  triggeredReceive priority 5, IRQ-driven   ← LoRa packets (safety critical)
  checkConnStatus  priority 2, 200ms        ← BIND LED
  loggerTask       priority 1, 1Hz          ← SPIFFS writes
```

**GPS and VESC share Serial1 through a hardware MUX (AW9523 GPIO). There is no
mutex protecting the MUX switch. Both are called sequentially inside the same
1000ms block on Core 1.**

---

## Findings — All Issues Identified

### Finding 1 — Wrong Serial1.flush() Call

**File:** `VESC.ino` → `getVescLoop()`
**Severity:** High — causes corrupted frame reads

`Serial1.flush()` on ESP32 drains the **TX** buffer, not RX. The intent was to
clear stale RX bytes from a previous failed response before sending a new request.
As written it does nothing useful. Stale bytes from a failed prior response remain
in the RX buffer and corrupt the next read's frame start byte detection, causing
`receiveFromVESC()` to return 0 (CRC NOK or "Message Error").

**Fix:** Replace with RX drain:

```cpp
// WRONG — drains TX output buffer, not incoming data
Serial1.flush();

// CORRECT — drains stale incoming bytes before new request
while (Serial1.available()) Serial1.read();
```

**Risk:** Zero. One line. No architectural change.

---

### Finding 2 — VESC Polled at 1Hz, GPS and VESC Share Time Budget

**File:** `V2_Integration_Rx.ino` → `loop()`
**Severity:** High — root cause of display flicker

Everything in the 1-second block runs sequentially. If GPS drains a burst of
sentences (satellite acquisition, 5Hz module output), VESC gets less of its
200ms window. Sequential coupling means a slow GPS drain directly delays VESC.

**Proposed Fix A (REJECTED): New FreeRTOS task for VESC**

- Requires MUX mutex — Serial1 shared, no protection exists
- Core 0 is already full (generatePWM p10, triggeredReceive p5, checkConnStatus p2)
- Any Core 1 task above loop priority delays WDT feed → panic risk
- A misbehaving VESC read (VESC fault, UART garbage) blocking 200ms in a task
  does not yield → WDT fires → full RX reboot → total control loss
- **VERDICT: Do not implement. Violates safety philosophy.**

**Proposed Fix B (ACCEPTED): Separate independent timers in loop()**

```cpp
// Two independent timestamps instead of one shared loop_timer:
static unsigned long vesc_timer = 0;
static unsigned long gps_timer  = 0;

if (millis() - vesc_timer > 500) {   // 2Hz — independent of GPS
    vesc_timer = millis();
    if (usrConf.data_src == 2) getVescLoop();
}
if (millis() - gps_timer > (1000 / max(1, (int)usrConf.gps_update_hz))) {
    gps_timer = millis();
    if (usrConf.gps_en) getGPSLoop();
}
```

- Stays entirely on Core 1. No new tasks. No new mutexes. No WDT changes.
- GPS and VESC no longer compete for the same time slot.
- If both timers fire in the same loop() iteration they still run sequentially
  (MUX switch handles bus isolation) — but this is already the case and is safe.
- The 10ms MUX settle delay still blocks Core 1 at 2Hz = 20ms/second. Acceptable.
- **VERDICT: Implement. Low risk. Highest practical benefit.**

---

### Finding 3 — stale foil_speed and foil_power Never Go N/A

**File:** `VESC.ino` → `getVescLoop()`
**Severity:** Medium — shows wrong values after VESC drops

When `millis() - last_uart_packet > vesc_timeout_s * 1000`, only `foil_bat` and
`foil_temp` are set to `0xFF`. `foil_speed` and `foil_power` are written only on
successful reads and never explicitly invalidated. After VESC drops, the display
continues showing the last-known speed and power indefinitely.

**Fix:** Add missing fields to the timeout block:

```cpp
if (millis() - last_uart_packet > ((unsigned long)usrConf.vesc_timeout_s * 1000UL)) {
    telemetry.foil_bat   = 0xFF;
    telemetry.foil_temp  = 0xFF;
    telemetry.foil_speed = 0xFF;  // ADD — was missing
    telemetry.foil_power = 0xFF;  // ADD — was missing
}
```

**Risk:** Zero. Two additional lines in existing block.

---

### Finding 4 — vesc_timeout_s Default Too High

**File:** `BREmote_V2_Rx.h` → `defaultConf`
**Severity:** Low-medium — stale data shown too long after drop

Current default: 12 seconds. A VESC that is truly connected responds within
200ms every time. 12 seconds means the display shows stale bat/temp for up to
12 seconds after connection drops.

**Critical constraint:** 3 seconds is too aggressive. VESCs under heavy
regenerative braking or in fault recovery can back up their UART for 1–3 seconds.
Setting timeout to 3s means N/A flashes during every legitimate VESC fault event,
alarming the rider during a moment requiring throttle focus.

**Recommended default:** 5–8 seconds. Suggest **6 seconds** as the new default —
fast enough to show stale clearly, slow enough to survive VESC fault transients.

**Fix:** In `defaultConf` initialization:

```cpp
6,   // vesc_timeout_s: was 12; 6s balances responsiveness vs fault tolerance
```

**Risk:** Zero. One value change. User can override via web UI.

---

### Finding 5 — E7 Wetness Latch: De-Latch Path Missing

**Filed from:** Claude Code session + field test observation
**Severity:** High — survives TX power cycle, only clears on RX reboot

**Symptom:** E7 (water ingress) error displays on TX and is latched on RX.
Power cycling TX alone does not clear it. Only RX power cycle clears it.

**Hypothesis — VESC Stale Struct as False Trigger:**
`checkWetness()` reads a hardware sensor via AW9523. However, the wetness
detection path likely reads from a field that could momentarily read garbage
during a VESC UART collision or MUX switch event. If the wetness sensor share
the AW9523 I2C bus with the MUX control lines, a MUX switch during a VESC read
could corrupt an AW9523 read, returning a false wet state. One false wet reading
sets the latch. The latch has no clear path in the runtime code — it would need
an explicit `remote_error &= ~E7_MASK` somewhere that doesn't appear to exist.

**Diagnostic:** Upload logs from a session where E7 appears. Look for:

- VESC dropout events (duty_cycle going to 0, then telemetry going stale)
- Timestamp correlation with E7 onset
- Whether E7 always follows a VESC miss within 1–2 loop iterations

**Fix approach (not yet implemented — needs log confirmation first):**

1. Add explicit de-latch: after N seconds of clean wetness reads, clear E7 flag
2. Guard `checkWetness()` to only run when MUX is NOT in mid-switch state
3. Add wetness column to log output for cross-correlation

**This is filed for a dedicated fix session after log confirmation.**

---

## Implementation Order for Claude Code

| Priority | Finding                            | Files                 | Risk   | Lines                  |
| -------- | ---------------------------------- | --------------------- | ------ | ---------------------- |
| 1        | Serial1.flush() → RX drain         | VESC.ino              | Zero   | 1                      |
| 2        | foil_speed/power → 0xFF on timeout | VESC.ino              | Zero   | 2                      |
| 3        | Separate GPS/VESC timers in loop() | V2_Integration_Rx.ino | Low    | ~10                    |
| 4        | vesc_timeout_s default 12→6        | BREmote_V2_Rx.h       | Zero   | 1                      |
| 5        | E7 de-latch path                   | System.ino            | Medium | TBD after log analysis |

**Findings 1–4 are safe to bundle into one Claude Code session.**
**Finding 5 (E7) needs log evidence first — file separately.**

---

## What This Does NOT Fix

- The telemetry rotation index mismatch (7 fields cycling at 10Hz, VESC updating
  at 2Hz after fix) — the display will still show slightly stale values between
  VESC polls. This is acceptable and not worth the complexity of synchronizing the
  rotation index to VESC update events.
- The fundamental 200ms receive timeout in receiveFromVESC() — this is appropriate
  for the baud rate and should not be reduced without VESC hardware profiling.
- Any telemetry issues caused by a second VESC on CAN — CAN telemetry requires
  a separate implementation path and is out of scope for this fix.

---

## Second VESC / CAN Note

Testing with VESC2 will confirm UART protocol compatibility but will NOT fix the
display flicker — all issues above are firmware-side, not VESC-hardware specific.
When CAN is added for VESC2, the telemetry struct and timeout logic will need to
be extended to handle two VESC sources. File that as a separate feature when ready.

---

## Blocking Priority Reminder

**Bug 1 (motor running during RTM arm window) takes absolute priority over this
entire document.** This VESC fix session does not begin until Bug 1 is confirmed
fixed, compiled, and field-tested. The arm ceremony throttle blackout fix is 4
targeted lines in TX RTMState.ino and is ready to implement on approval.
