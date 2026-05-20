# BLE App Brief — BREmote TX Companion App

**BREmote V2.5-Evo | Filed: 2026-05-16**  
**Status: Research complete — custom Android app brief ready**

---

## App Inventory — Field Research

### VESC Tool (NUS/VESC binary)
- **Platform:** iOS + Android (free)
- **Status:** ✓ Confirmed working with BREmote TX (2026-05-16)
- **Gauges shown:** FET temp, Motor Amps, Duty, Voltage, RPM, Power (computed)
- **Notes:** Free, actively maintained. Requires BLE NUS + VESC binary protocol. Recommended for now.

### Floaty
- **Platform:** iOS + Android
- **Status:** Blocked — requires `COMM_GET_MCCONF` and `COMM_GET_APPCONF` after FW handshake. Implementing these risks a cascade of additional required responses. Also paywalled for full gauge access.
- **Decision:** Not implemented. VESC Tool covers the use case for free.

### Metr
- **Platform:** Android + iOS
- **Status:** Dead end — requires proprietary nRF52 BLE hardware (Metr module). Their shop is permanently closed. Protocol is reverse-engineered but depends on their physical dongle.
- **Decision:** Not viable.

### FreeSK8 Mobile
- **Platform:** Android + iOS (Flutter/Dart, open source)
- **Status:** Best fork candidate for a custom app
- **Source:** https://github.com/FreeSK8/FreeSK8-Mobile
- **Notes:** Implements full NUS + VESC binary. Clean Flutter codebase. Display is esk8-focused (maps, trip logger, speed) but underlying telemetry stack is reusable. Would require UI redesign to efoil-specific fields.

---

## Custom Android App Brief

### Purpose

A lightweight Android BLE companion for BREmote TX. Connects via NUS BLE, speaks VESC binary, displays efoil-relevant live gauges. No esk8 maps, no trip logging in v1.

### Target Platform

Android first (Flutter for future iOS port). Min SDK to be determined by project build environment.

### Connection

- BLE scan filter: NUS Service UUID `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- Device name prefix: `BRemote-TX-`
- Handshake: send `COMM_FW_VERSION (0x00)`, receive FW response, then begin polling `COMM_GET_VALUES (0x04)` at ~4 Hz

### Gauge Fields (in display priority order)

| # | Field | Source (`COMM_GET_VALUES` offset) | Unit | Notes |
|---|---|---|---|---|
| 1 | Battery Voltage | `v_in` | V | Main pack voltage — most-watched |
| 2 | Motor Current | `avg_motor_current` | A | Instantaneous draw |
| 3 | Duty Cycle | `duty_cycle` | % | Throttle position proxy |
| 4 | Power | computed: `v_in × avg_input_current` | W / kW | Show as kW with 1 decimal |
| 5 | FET Temp | `temp_fet` | °C | Primary thermal gauge |
| 6 | Motor Temp | `temp_motor` | °C | Mirrored from FET temp in BREmote |
| 7 | RPM (ERPM) | `rpm` | ERPM | Raw ERPM — do NOT convert to km/h without motor pole count + wheel circumference |
| 8 | Fault Code | `fault_code` | enum | Show as text label (FAULT_CODE_NONE, etc.) |
| 9 | Amp-Hours | `amp_hours` | Ah | Always 0 from BREmote (not tracked) |
| 10 | Watt-Hours | `watt_hours` | Wh | Always 0 from BREmote (not tracked) |

### Anti-Patterns (do not implement in v1)

- **RPM → speed conversion:** Requires motor pole pairs + wheel/prop circumference. These are not in the VESC binary stream. Showing speed without hardware config would be misleading. Defer or make it user-configurable.
- **COMM_GET_MCCONF / COMM_GET_APPCONF:** Do not request motor config — BREmote is not a VESC, it proxies telemetry only. Requesting config would hang waiting for a response that will never come.
- **Multiple simultaneous connections:** NimBLE configured for 1 connection max. App must not attempt multi-device.

### Protocol Reference

All frames: `0x02 [len] [payload] [CRC16-CCITT hi] [CRC16-CCITT lo] 0x03`

CRC16/CCITT: init 0x0000, poly 0x1021, big-endian, computed over payload bytes only.

Full `COMM_GET_VALUES` field offsets: see VESC open-source firmware `datatypes.h` (mc_values struct).

### Reference Implementation

FreeSK8 Mobile — Flutter/Dart, MIT-adjacent license, clean NUS+VESC stack:  
`https://github.com/FreeSK8/FreeSK8-Mobile`

Recommend: fork FreeSK8, strip esk8-specific screens (map, trip log), replace with efoil gauge layout using the field list above.

---

## Decision Log

| Date | Decision | Reason |
|---|---|---|
| 2026-05-16 | VESC Tool as primary app | Free, confirmed working, no extra protocol work |
| 2026-05-16 | Floaty not implemented | Motor config cascade risk + paywalled |
| 2026-05-16 | Metr ruled out | Hardware dependency, shop closed |
| 2026-05-16 | FreeSK8 as fork base | Best open-source VESC BLE codebase in Flutter |
| 2026-05-16 | Custom app v1 Android only | Flutter allows iOS port later with minimal extra work |
