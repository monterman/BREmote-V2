# 2026-05-16 — LoRa Packet Expansion 8→19 + 0xF4 Aux + Display Fix

**Branch:** `feature/bluetooth`
**Commits:** `60e4d41`, `bc3dea4` (both pushed to remote)

---

## What happened

### Commit 60e4d41 — feat(telemetry): expand LoRa packet 8→19 bytes + 0xF4 aux meta-packet

Full LoRa telemetry packet expansion across 7 firmware files (authorized in prior session as "yes full expand").

**New TelemetryPacket layout (both TX and RX — structs must always match):**

| Index | Field | Encoding |
|---|---|---|
| 0 | foil_bat | % SoC |
| 1 | foil_temp | °C |
| 2 | foil_speed | raw |
| 3 | error_code | error enum |
| 4 | foil_power | W÷50 |
| 5 | rtm_distance | raw |
| 6 | foil_motor_amps | whole amps 0–250 |
| 7 | foil_voltage | V×2 (0.5V res); 0xFF=N/A |
| 8 | foil_duty | 0–100%; 0xFF=N/A |
| 9 | foil_erpm_lo | \|ERPM\|÷100 lo byte; 0xFFFF=N/A |
| 10 | foil_erpm_hi | \|ERPM\|÷100 hi byte |
| 11 | foil_wh_lo | Wh×10 lo byte; 0xFFFF=N/A |
| 12 | foil_wh_hi | Wh×10 hi byte |
| 13 | rx_heading | GPS COG÷2 (0-179→0-358°); 0xFF=N/A |
| 14 | fm_heading_err | bearing error+127 bias; 127=no data |
| 15 | fm_status | packed flags byte (see below) |
| 16 | reserved_tx_imu | RESERVED: future TX IMU wipeout |
| 17 | rx_bearing_to_tx | bearing buggy→rider÷2; 0xFF=N/A |
| 18 | link_quality | MUST be last (TX cycling logic) |

**fm_status bit map:**
- [0] fm_active (fm_mode_runtime ≥ 1)
- [1] rtm_active
- [3:2] heading_conf (0–3)
- [4] rx_wetness (error_code==71)
- [5] vesc_online
- [6] aux1_on (rx_aux_flags bit 0)
- [7] aux2_on (rx_aux_flags bit 1)

**VESC changes (RX side):**
- `VESC_MORE_VALUES` now also requests WattHours (bit 3 of vesc_command[3] = overall mask bit 11)
- `VESC_PACK_LEN` 23→27 (floats32_auto wh field is 4 bytes)
- `vescRelayBuffer[30]→[34]`
- Custom `buffer_get_float32_auto()` decoder (exponent bias 126, not IEEE-754's 127)
- `vesc_struct` gains `int32_t wh_raw` (Wh×10, stored after `batVolt`)

**BLE gauges now live (TX side, BLE.ino):**
- Voltage: foil_voltage × 0.5 → v_in
- Duty: foil_duty ÷ 100 → duty_frac
- RPM: (foil_erpm_hi<<8 | foil_erpm_lo) × 100 → rpm

**0xF4 aux meta-packet:**
- TX: `sendAuxCommand(uint8_t flags)` in Radio.ino queues `queueMetaPacketBurst(0xF4, flags)` ×3
- TX: forward declaration added to V2_Integration_Tx.ino
- RX: Radio.ino handler sets `rx_aux_flags = rcvArray[4]`
- RX: `volatile uint8_t rx_aux_flags = 0` declared in BREmote_V2_Rx.h
- flags: bit0=strobe/light, bit1=horn(reserved), bit2=aux3(reserved), bit3=find-me flash

**Files changed:** BREmote_V2_Tx.h, BREmote_V2_Rx.h, VESC.ino, RTMState.ino, BLE.ino, Radio.ino (both), V2_Integration_Tx.ino

---

### Commit bc3dea4 — fix(display): retry HT16K33 probe up to 100ms on startup

**Symptom:** Display sometimes failed to come up on first power cycle; required 2 cycles.

**Root cause:** `startupDisplay()` in `Display.ino` called `beginDisplay()` exactly once. If the HT16K33 wasn't ready (PCB power rail still settling, chip tPOR not yet complete), the single probe failed → entered `while(1) delay(100)` infinite hang → required hard power cycle to recover.

**Fix:** Replaced single probe with retry loop — 20 attempts × 5ms = up to 100ms total.

```cpp
bool found = false;
for (int i = 0; i < 20 && !found; i++) {
  found = beginDisplay();
  if (!found) delay(5);
}
if (!found) { Serial.println(" Failed"); while(1) delay(100); }
```

HT16K33 datasheet: tPOR ≤ 1ms. 100ms gives wide margin for PCB rail settle under any load condition.

---

## Pending (carry forward)

- [ ] Compile TX + RX with `PartitionScheme=huge_app` and flash to hardware
- [ ] Verify VESC Tool BLE gauges show live voltage, duty, RPM (new in this session)
- [ ] Verify display comes up reliably on first power cycle
- [ ] `sendAuxCommand()` wired to UI trigger (strobe/find-me buttons not yet connected to any gesture)

## Cross-links

- [[2026-05-14-sw55-vesc-fix-display-polish-github-push]] — previous session, SW55 VESC fixes + first display polish
- [[2026-05-14-sw51-sw54-vesc-mux-animation-vault-merge]] — MUX EMI resilience work
- [[2026-05-13-sw48-sw50-vesc-uart-emi-diagnosis]] — VESC UART + EMI diagnosis origin
