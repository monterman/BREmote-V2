# VESC Telemetry Fix — Analysis & Implementation Plan

**BREmote V2.5-Evo | RX firmware | Filed: 2026-05-09**
**Status: RESOLVED. The definitive "telemetry appears dead" root cause was a diagnostic bug — see 2026-07-19 section below. The SW51–SW55 fixes and the USB-C hardware constraint (below that) remain valid.**

---

## 2026-07-19 — Definitive Root Cause: the `?vescraw` Diagnostic Was Lying

The "VESC telemetry is dead / no telemetry" symptom that drove months of investigation
was, in the end, a **diagnostic false negative — not a hardware, cable, MUX, config, or
firmware fault.** Two independent bugs were found and fixed; neither was in the real
telemetry path.

### Bug 1 (the headline) — `?vescraw` hardcoded the wrong CRC

`?vescraw` builds a `COMM_GET_VALUES` probe by hand and hardcoded the CRC16 as `0x4007`.
The correct value is **`0x4084`** — CRC16-CCITT/XMODEM (poly `0x1021`, init 0) computed
over the single-byte payload `{0x04}`.

A VESC **silently discards any packet whose CRC does not match** — it does not NAK, does
not reply at any baud, it just drops the frame. So the malformed probe produced zero bytes
back **every time, against a perfectly healthy VESC**. `?vescraw` dutifully reported
"NO BYTES", which read as "the VESC is dead" — a **false negative that masked good
hardware** and sent the diagnosis chasing the VESC, the RX, the AW9523 MUX, the cabling,
the config, and firmware 6.06, none of which were ever at fault.

**Bench proof (FTDI, direct to VESC):** the corrected frame

```
02 01 04 40 84 03
```

returns a full `GET_VALUES` reply on **both VESC FW 6.05 and FW 6.06** — live 39.5 V /
26.7 °C. Wrong CRC → silence; correct CRC → full telemetry, on the same hardware, same
cable, same session.

**Fix:** `System.ino` `?vescraw` now emits `0x4084` and returns the real reply
(commit `492d672`).

**The production telemetry path was never affected.** The real path
(`getValuesSelective` → `sendToVESC` → `vesc_crc16`, table-driven CRC in `vesc_crc.cpp`)
always computed the CRC correctly. Only the hand-rolled `?vescraw` diagnostic frame carried
the wrong constant, which is exactly why live gauges (BLE, TX display) worked while the
diagnostic insisted the link was dead.

### Bug 2 — in-ride telemetry frozen by a throttle-skip gate

Separately, a regression (the "D1" gate, introduced 2026-06-04) skipped the VESC poll
whenever `thr_received ≥ 25`. On a continuous-throttle vehicle (the tow buggy) that means
`getVescLoop()` never runs once you are on the trigger, so telemetry froze at the boot
`0xFF` default (dashes) for the entire ride. Reverted to the SW55 unconditional 2 Hz poll
(commit `d68ed07`). The MUX-EMI concern the gate was meant to address is already covered by
the read-back-verify inside `setUartMux()`, so the skip was redundant.

### Bottom line

Nothing was wrong with the VESC, the RX, the AW9523 MUX, the cables, the config, or
firmware 6.06. The "no telemetry" verdict came from a diagnostic that was itself broken.
The SW51–SW55 findings below (GPS MUX yield, `rcv_err` persistence, boot MUX state) were
real and are still fixed; the USB-C-on-GPIO-18/19 constraint below is still a genuine
hardware limitation to respect in the field. But the thing that made telemetry *look*
dead was the `?vescraw` CRC.

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
| 2 | GPS/VESC share single 1 Hz timer | Fixed — independent `vesc_loop_timer` (2Hz) and `gps_loop_timer` (rate=`gps_update_hz`) implemented in SW55; GPS and VESC no longer compete for the same 1Hz slot |
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

## Exact Execution Model (Pre-SW55 — Historical Reference)

> **Note:** This block shows the unfixed execution model as it existed before SW55. Findings 1–3 are now fixed. The independent timer structure in Finding 2 replaces the single `loop_timer` shown here.

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

**Filed from:** the build workflow session + field test observation
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

## Implementation Order for the build workflow

| Priority | Finding                            | Files                 | Risk   | Lines                  |
| -------- | ---------------------------------- | --------------------- | ------ | ---------------------- |
| 1        | Serial1.flush() → RX drain         | VESC.ino              | Zero   | 1                      |
| 2        | foil_speed/power → 0xFF on timeout | VESC.ino              | Zero   | 2                      |
| 3        | Separate GPS/VESC timers in loop() | V2_Integration_Rx.ino | Low    | ~10                    |
| 4        | vesc_timeout_s default 12→6        | BREmote_V2_Rx.h       | Zero   | 1                      |
| 5        | E7 de-latch path                   | System.ino            | Medium | TBD after log analysis |

**Findings 1–4 are safe to bundle into one the build workflow session.**
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
