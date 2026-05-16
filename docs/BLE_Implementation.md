# BLE Implementation — BREmote V2.5-Evo TX

**Branch:** `feature/bluetooth`  
**File:** `Source/V2_Integration_Tx/BLE.ino`  
**Status:** Field-confirmed working — VESC Tool ✓ Floaty ✓ (2026-05-16)

---

## Overview

The TX advertises a Nordic UART Service (NUS) over BLE and speaks the VESC binary protocol. Compatible apps (VESC Tool, Floaty) can connect and display live telemetry gauges — FET temp, motor amps, duty, voltage, RPM — sourced from the LoRa telemetry the TX receives from the RX/VESC.

Hardware: ESP32-C3 (HT-CT62). BLE and LoRa use separate radios — no coexistence conflict.

---

## Configuration (`bt_enabled` SPIFFS field)

| Value | Behaviour |
|---|---|
| `0` | BLE always off (old SPIFFS default — tail padding reads as 0) |
| `1` | Hall/session mode — BLE starts only after Hall sensor short-hold activates the BT dot |
| `2` | Always on — BLE starts 5 s after boot regardless of Hall sensor |

Set via web config UI or `?set bt_enabled 2` + `?save`.  
Old SPIFFS (before this field existed) reads 0 — set to 1 or 2 after first flash.

**Boot note:** `bleInitTask` fires 5 s after `initTasks()`. If the TX is plugged into USB during boot, `checkCharger()` blocks until unplugged — `initTasks()` never runs, BLE never starts. **Always boot on battery for BLE use.**

---

## Boot Gesture — Session Override

`Throttle + LEFT toggle hold` during boot sets `bt_session_forced = true`. BLE activates for the session regardless of `bt_enabled`. Useful for one-off testing without changing SPIFFS.

---

## BT Dot (C7 R1 on display)

| `bt_dot_state` | Pattern | Meaning |
|---|---|---|
| `BT_DOT_OFF` | Dark | BLE inactive |
| `BT_DOT_SLOW` | 1 s blink | BLE ready / `bt_enabled=2` always-on |
| `BT_DOT_FAST` | 250 ms blink | BLE active after long Hall hold |

---

## Advertisement Format

**Problem fixed 2026-05-16:** Name (16 B) + NUS UUID (18 B) + flags (3 B) = 37 B, exceeding the 31-byte BLE advertisement limit. The UUID was silently truncated, making the device invisible to apps that filter by NUS service UUID (VESC Tool, Floaty).

**Current layout:**

| Packet | Contents | Size |
|---|---|---|
| Main advertisement | Flags `0x06` + NUS UUID 128-bit | 21 B |
| Scan response | Device name `BRemote-TX-XX` | 16 B |

NUS UUID in main advert ensures scan filters work. Name in scan response is fetched by active scanners and shown in the app device list.

---

## Protocol

### Service + Characteristics

| | UUID |
|---|---|
| NUS Service | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` |
| NUS RX (app→device) | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` |
| NUS TX (device→app) | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` |

### Commands handled (inbound on NUS RX)

| Command | Byte | Response |
|---|---|---|
| `COMM_FW_VERSION` | `0x00` | FW 6.05, HW `BREmote`, zeroed UUID — passes VESC Tool / Floaty handshake |
| `COMM_GET_VALUES` | `0x04` | Full `COMM_GET_VALUES` response with live telemetry (see below) |

All frames use VESC binary framing: `0x02 [len] [payload] [CRC16-CCITT hi] [CRC16-CCITT lo] 0x03`.

### COMM_GET_VALUES fields sent

| Field | Source | Notes |
|---|---|---|
| `temp_fet` | `telemetry.foil_temp` | °C, also mirrored to `temp_motor` |
| `avg_motor_current` | `telemetry.foil_motor_amps` | Amps |
| `duty_cycle` | `telemetry.foil_duty` | Fraction 0.0–1.0 |
| `rpm` | `telemetry.foil_erpm_hi/lo` × 100 | ERPM |
| `v_in` | `telemetry.foil_voltage` × 0.5 | Battery voltage |
| `fault_code` | `telemetry.error_code` | VESC fault indicator |
| `amp_hours`, `watt_hours` | 0 | Not tracked |
| `avg_input_current` | 0 | Not available |

### Auto-detect mode

On each new connection `vescProtoMode = false`. First `COMM_GET_VALUES` request → `vescProtoMode = true` → request-driven mode (app polls, TX responds). If no VESC request arrives within 500 ms → CSV push mode (for generic BT serial terminals).

### CSV push format (non-VESC apps)

```
SPD:%u,BAT:%u,TMP:%u,A:%u,W:%u,SQ:%u\n
```

Fields: speed, battery %, temp, motor amps, watts (×50 encoding), signal quality.

---

## Device Name

`BRemote-TX-XX` where `XX` = last byte of BT MAC address in uppercase hex.  
Example: `BRemote-TX-A4`

---

## App Compatibility

| App | Platform | Status | Notes |
|---|---|---|---|
| VESC Tool | iOS / Android | ✓ Working | Full gauge support — Temp, Motor A, Duty, Voltage, RPM |
| Floaty | iOS / Android | ✗ Blocked | Requires motor config (`COMM_GET_MCCONF` / `COMM_GET_APPCONF`) after FW handshake. Implementing those responses risks a deeper cascade of required commands. Floaty also paywalls most features. Not worth pursuing. |

---

## Known Limitations (Tier 2)

- `amp_hours`, `watt_hours`, `avg_input_current` always report 0 — not present in LoRa telemetry struct
- Single VESC client only — NimBLE configured for 1 connection
- No `COMM_GET_MCCONF` / `COMM_GET_APPCONF` handlers — Floaty requires these after FW handshake; implementing them risks a cascade of additional required responses and is intentionally deferred
- BLE and field operation: unplug USB before riding; USB power triggers charging mode and blocks BLE init

---

## Source Reference

```
Source/V2_Integration_Tx/
  BLE.ino          — full BLE implementation
  BREmote_V2_Tx.h  — bt_enabled field (confStruct), bt_dot_state constants, P_MAG pin
  Init.ino         — bleInitTask (5 s delayed, Core 0, priority 1, 4 KB stack)
  System.ino       — Hall sensor state machine → bt_dot_state; boot gesture → bt_session_forced
```
