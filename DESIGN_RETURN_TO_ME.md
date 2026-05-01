# Return-to-Me (RTM) and Follow-Me Override - Design Document
**Project:** BREmote V2.5-Evo  
**Date:** April 21, 2026  
**Status:** Implemented and operational — Beta release as of V2.5-Evo P8/P9. RTM and FM mode selection are live. FM autonomous following (Priority 9) is not yet implemented. Full field test pending.

---

## 1. Purpose and Philosophy

Two new user-activated modes on the TX toggle, both strictly respecting the creator safety philosophy:
- Tow Buggy ONLY moves when user physically holds throttle trigger
- Autonomous systems can only steer and subtract from throttle, never add
- Release throttle = buggy stops immediately, always

### Mode A: Return-to-Me (RTM / "rn")
For when user is sitting on board drifting slowly in water, activate mode to have the buggy come toward their GPS location. User must actively hold throttle to drive it home. Buggy performs automatic steering only, capped and ramped throttle. Hard stop at configured safe distance.

### Mode B: Follow-Me Mode Override (FM)
In-session override of RX's follow-me behavior without modifying RX SPIFFS. Cycles through F0=Off, F1=Right, F2=Behind, F3=Left. Resets to RX web-configured default on reboot.

---

## 2. User Interaction Design

### 2a. RTM Activation Sequence

Step 1: User performs gesture: RIGHT tap, then LEFT hold for `rtm_hold_duration_s` seconds (default 5s)  
Step 2: Dot display shows "rn" blink — mode armed, arm window starts (`rtm_arm_window_s`, configurable)  
Step 3: User squeezes throttle above 10% — motor engages immediately with ramp cap  
*(Note: double-squeeze mode was deprecated in P8. The `rtm_double_squeeze_en` SPIFFS field is retained for compatibility but defaults to 0. "Arm" and "rdy" display codes no longer appear in firmware.)*  
Step 4: Throttle ramps from `rtm_throttle_start_pct` (default 30%) to `rtm_throttle_max_pct` (default 70%) over `rtm_ramp_duration_s` seconds (default 5s)  
Step 5: Release throttle at any time → buggy stops immediately (normal failsafe)  
Step 6: If throttle released >10s during active mode → RTM disengages, user must re-activate from Step 1  
Step 7: If user re-grabs throttle within 10s window → ramp restarts from `rtm_throttle_start_pct` (no sudden jumps)  
Step 8: Hard stop when distance to TX < `rtm_stop_distance_m` (default 10m, range 3-20m) → throttle forced to 0 regardless of user input

### 2b. Follow-Me Mode Override Sequence

Step 1: User holds RIGHT toggle at full extent for `fm_hold_duration_s` seconds (default 5s, configurable 4-10s)  
Step 2: Dot shows full-screen confirmation: "F0", "F1", "F2", or "F3" for the selected mode  
Step 3: Each additional right-hold advances one step through the cycle  
Step 4: User releases toggle for 2s without further right-holds → selection locked  
Step 5: TX sends mode change meta-packet 3x with 100ms gap for reliability  
Step 6: RX applies override to runtime RAM variable only (does not write to SPIFFS)  
Step 7: On RX reboot, mode returns to web-configured SPIFFS default

### 2c. Dot Display States

| State | Display | Duration |
|---|---|---|
| RTM armed, waiting for throttle | `rn` blink | Until throttle or arm window timeout |
| RTM armed + throttle squeezed | `rn` blink + unlock animation | 2s confirm |
| RTM active (motor running) | RTM info display (distance or speed) | While active |
| RTM disengaged or hard stop | `St` (large-font 2s) | 2s then normal telemetry |
| FM mode selected | `F0` / `F1` / `F2` / `F3` full-screen flash | 2s confirmation |
| *(Deprecated)* double-squeeze first squeeze | ~~`Arm`~~ | Removed in P8 |
| *(Deprecated)* double-squeeze ready | ~~`rdy`~~ | Removed in P8 |
| *(Deprecated)* FM cycling abbreviations | ~~`0ff` / `bEh` / `n-R` / `n-L`~~ | Removed in P8 |

---

## 3. Safety Gates

### 3a. Phase A — RX Standalone GPS Validation (always-on, no RTM dependency)

Runs on every RX GPS reading regardless of operating mode.

1. HDOP < `gps_max_hdop` (default 2.0)
2. Implied acceleration between consecutive readings < `gps_max_accel_g` G (default 3.0)
3. Position change since last reading implies speed < `gps_max_jump_kmh` (default 200 km/h)

After `gps_suspect_threshold` (default 3) consecutive failures → RX GPS marked as **rejected**.  
Rejected GPS blocks RTM arming and all RTM distance calculations.

### 3b. Phase B — TX/RX Handshake Cross-Validation (on connect + every 30s)

Requires 0xF3 meta-packet infrastructure (Priority 4). TX sends its GPS coordinates to RX via 0xF3.

4. TX↔RX computed distance is plausible: < `gps_max_pair_dist_m` (default 500 m)
5. TX GPS speed and RX GPS speed are consistent: difference < `gps_max_speed_diff_kmh` (default 50 km/h)

Failure → RTM arming blocked until next successful handshake. Spoofing event logged to SPIFFS.

### 3c. Phase C — RTM-Active Convergence Checks (~10Hz, only during active RTM)

6. Distance to TX is **decreasing** (convergence verification — buggy is actually moving toward user)
7. VESC ERPM-implied speed is within `rtm_vesc_speed_diff_kmh` (default 20 km/h) of GPS speed
8. TX GPS data age < `tx_gps_stale_timeout_ms` (TX GPS still fresh)

Any Phase C failure → throttle immediately forced to 0, RTM disengages, display shows `St` for 2s.

### 3d. RTM Motor Safety Gates (every loop cycle, ALL must pass while RTM active)

When RTM is active, ALL of the following must be true or motor immediately cuts to 0:

1. User physically holding throttle >10% (creator philosophy rule — absolute, non-negotiable)
2. Phase A GPS not rejected on RX
3. Phase B handshake passed within last 30 seconds
4. Valid TX GPS fix (age < `rtm_gps_timeout_ms`, default 2000ms)
5. Valid RX GPS fix (age < `rtm_gps_timeout_ms`)
6. Valid compass heading on RX (calibrated + reading within range)
7. LoRa link healthy (last control packet < failsafe_time)
8. Within `rtm_max_runtime_s` (default 120s, range 30-300s) since activation
9. Distance to TX > `rtm_stop_distance_m` (hard stop distance)
10. Throttle not released for > 10s (or RTM disengages)

Any gate failure → throttle forced to 0, `St` for 2s, RTM disengages.

---

## 4. SPIFFS Parameters Needed

### 4a. TX side (new parameters to add)

| Name | Range | Default | Units | Description |
|---|---|---|---|---|
| `rtm_enabled` | 0-1 | 1 | bool | Master enable for RTM feature |
| `rtm_hold_duration_s` | 4-10 | 5 | seconds | LEFT hold duration to arm |
| `rtm_arm_window_s` | 5-30 | 10 | seconds | Window to engage throttle after arming |
| `rtm_double_squeeze_en` | 0-1 | 1 | bool | Require double-squeeze (1) or 500ms filter (0) |
| `rtm_throttle_start_pct` | 10-50 | 30 | % | Initial throttle cap when engaging |
| `rtm_throttle_max_pct` | 30-90 | 70 | % | Maximum throttle cap after ramp |
| `rtm_ramp_duration_s` | 2-15 | 5 | seconds | Time to ramp from start to max |
| `rtm_stop_distance_m` | 3-20 | 10 | meters | Hard stop distance from TX |
| `rtm_max_runtime_s` | 30-300 | 120 | seconds | Maximum continuous RTM runtime |
| `rtm_gps_timeout_ms` | 500-3000 | 2000 | ms | GPS loss timeout before safety stop |
| `fm_hold_duration_s` | 4-10 | 5 | seconds | RIGHT hold duration for FM mode cycle |
| `fm_override_enabled` | 0-1 | 1 | bool | Allow TX to override RX follow-me mode |

### 4b. RX side (new parameters to add)

| Name | Range | Default | Units | Description |
|---|---|---|---|---|
| `rtm_rx_enabled` | 0-1 | 1 | bool | RX-side master enable (safety kill switch) |
| `rtm_rx_override_steering` | 0-1 | 1 | bool | Allow RTM to override steering |
| `rtm_compass_required` | 0-1 | 1 | bool | Require valid compass for RTM (safety) |

### 4c. Rule reminder
All 15 new parameters must ALSO be added to:
- confStruct in respective .h files (TX and RX)
- SPIFFS load/save functions
- Web config UI (WebUiEmbedded.h) for both TX and RX
- ConfigService with validation ranges and defaults
- Inline code comments explaining valid range, units, default

---

## 5. LoRa Protocol Extensions (Layered Extension Approach)

### 5a. Design principle
Existing 6-byte control packet at 10Hz is UNTOUCHED - zero regression risk.  
Meta-packets share the same 6-byte format but use a different type byte in position [3].

### 5b. Existing packet types (DO NOT CHANGE)

| Byte[3] | Type | Payload | Frequency |
|---|---|---|---|
| throttle value | Control | [throttle][steering] | 10Hz |
| 0xAB | Pairing request | ... | Rare |
| 0xBA | Pairing response | ... | Rare |
| 0xAC | Pairing confirm | ... | Rare |

### 5c. New meta-packet types reserved (for V3 and beyond)

| Byte[3] | Type | Payload [4][5] | Frequency | Purpose |
|---|---|---|---|---|
| 0xF1 | Mode change - RTM state | [rtm_state][reserved] | On event | TX tells RX "RTM is active/inactive" |
| 0xF2 | Follow-Me override | [fm_mode][reserved] | On event | TX overrides RX follow-me mode |
| 0xF3 | TX GPS coords (future) | NEEDS EXPANDED PACKET | 2Hz | For future follow-me implementation |
| 0xF4 | Telemetry extended (future) | [telem_id][telem_val] | As needed | For future BLE forwarding etc |
| 0xF5 | Config override (future) | [param_id][value] | Rare | For future remote config |
| 0xF6-0xFE | Reserved | - | - | Future expansion |

### 5d. Packet collision avoidance

- Opcodes 0xAB, 0xBA, 0xAC reserved for pairing (DO NOT reuse)
- Opcodes 0xF1-0xFE reserved for meta-packets
- Opcodes 0x00-0xAA available for normal throttle values (throttle byte naturally stays in 0-255 range, but valid throttle never exceeds 0xFE so 0xFF unused)
- Opcodes 0xAD-0xEF available for future expansion if needed

### 5e. CRC and address validation

All meta-packets use existing 3-byte address validation and CRC8 at byte[5]. Same security as control packets.

### 5f. Reliability

Meta-packets sent 3 times with 100ms gap between sends. RX considers mode change applied after first valid meta-packet received (de-duplicates by tracking packet sequence).

---

## 6. State Machine Design

### 6a. TX state machine for RTM

States: IDLE → ARMING → ARMED → ACTIVE → COOLDOWN → IDLE

- IDLE: Normal operation, watching for RIGHT tap + LEFT hold gesture
- ARMING: LEFT held after RIGHT tap, counting toward rtm_hold_duration_s
- ARMED: `rn` blink displayed, arm window active, waiting for throttle squeeze
- ACTIVE: Motor engaged, throttle ramping, safety gates checked every loop
- COOLDOWN: `St` for 2s before returning to IDLE
*(SQUEEZE_WAIT state removed — double-squeeze mode deprecated in P8)*

### 6b. RX state machine for RTM

States: NORMAL → RTM_ACTIVE → NORMAL

Driven by meta-packet 0xF1 from TX. All safety gates evaluated locally on RX every loop.

### 6c. FM state machine

TX side: IDLE → CYCLING → CONFIRMING → IDLE  
RX side: Receives 0xF2 meta-packet, updates runtime variable, no SPIFFS write

---

## 7. Failure Modes and Recovery

| Failure | Detection | Recovery |
|---|---|---|
| TX GPS loss | Age > rtm_gps_timeout_ms | Immediate throttle=0, `St` for 2s, RTM disengages |
| RX GPS loss | Age > rtm_gps_timeout_ms | Same |
| Compass invalid | Calibration check fails | Same |
| LoRa link loss | Last packet > failsafe_time | Standard failsafe (existing) |
| VESC stall | ERPM near 0 with throttle > 30% for 20s | Throttle=0, user alerted |
| Max runtime exceeded | runtime > rtm_max_runtime_s | Throttle=0, RTM disengages |
| User releases throttle | Physical | Buggy stops (normal) |
| User releases > 10s | Timer | RTM disengages, user must re-arm |
| GPS spoofing detected | Acceleration > 3g | Throttle=0, RTM disengages |
| Meta-packet not received by RX | No ACK mechanism | User sees no mode change on next telemetry, can re-try |

---

## 8. Test Plan Before Flashing

### 8a. Bench tests (no motor attached)
- Verify TX state machine transitions via serial monitor logs
- Verify meta-packets sent correctly (read RX serial output)
- Verify safety gate logic by forcing fail conditions
- Verify SPIFFS save/load of all new parameters

### 8b. Motor bench tests (motor unloaded, prop removed)
- Verify ramp behavior 30% → 70% over 5s
- Verify throttle=0 on release
- Verify 500ms filter (if single-squeeze mode)
- Verify double-squeeze sequence (if default mode)
- Verify hard stop triggers correctly with simulated GPS coords

### 8c. Controlled water tests (shallow water, tether to buggy)
- Short range RTM test at low speed
- Intentional GPS denial to verify safety stops
- Verify distance-based hard stop

### 8d. Full field test
- Only after all bench and controlled tests pass
- Helmet + PFD, spotter present
- Start with `rtm_throttle_max_pct` at 70-75% (default 70% — this is the recommended field test starting point; 40% is too conservative for meaningful water testing)
- `rtm_throttle_start_pct` controls the ramp start (default 30%); leave at default for first tests
- `rtm_approach_zone_m` controls the decel ramp start distance (default 15m); buggy smoothly reduces throttle to 0 as it enters this zone before the hard stop
- `rtm_stop_distance_m` (RX) is the hard stop distance (default 3m); confirm via RX web config before field use
- `rtm_disengage_distance_m` (TX) is the TX-side disengage threshold — keep ≥ rtm_stop_distance_m (RX)

**Key SPIFFS parameters for RTM field testing:**
| Param | Board | Default | Notes |
|---|---|---|---|
| `rtm_throttle_max_pct` | TX | 70% | Max throttle during RTM run |
| `rtm_throttle_start_pct` | TX | 30% | Ramp start at engagement |
| `rtm_ramp_duration_s` | TX | 5s | Time from start% to max% |
| `rtm_approach_zone_m` | TX | 15m | Distance where decel ramp begins |
| `rtm_disengage_distance_m` | TX | 10m | TX-side disengage trigger |
| `rtm_stop_distance_m` | RX | 3m | Hard stop distance |
| `rtm_max_runtime_s` | TX | 0 (disabled) | 0=no runtime limit |
| `rtm_arm_window_s` | TX | 10s | Window to squeeze after "rn" arm |

**Key SPIFFS parameters for FM field testing (mode selection only — autonomous FM not yet implemented):**
| Param | Board | Default | Notes |
|---|---|---|---|
| `fm_override_enabled` | TX | 1 | Must be 1 to allow F0-F3 cycling via gesture |
| `fm_warn_distance_m` | TX | 150m | TX-RX distance triggers proximity vibration warning |

*FM autonomous following (GPS-guided heading + throttle gating) is Priority 9 and not yet implemented. FM gesture cycles the mode selection on RX; no autonomous steering occurs.*

---

## 9. Implementation Order

### Phase 1: TX GPS reading (Priority 1 - already in progress)
Complete current TX GPS speed implementation (Files 1-6 of GPS work). This provides foundation for knowing TX location.

### Phase 2: LoRa meta-packet infrastructure
- Define meta-packet types in Common/RadioCommon.h
- Update TX Radio.ino to send meta-packets
- Update RX Radio.ino triggeredReceive to dispatch meta-packets
- Test meta-packet reliability (bench test)

### Phase 3: FM Follow-Me Override (easier, good first mode feature)
- Add fm_ parameters to TX SPIFFS + Web UI
- Implement right-hold state machine on TX
- Implement 0xF2 meta-packet on both sides
- Implement runtime override variable on RX
- Test mode switching end-to-end

### Phase 4: RTM Return-to-Me
- Add all rtm_ parameters to TX and RX SPIFFS + Web UI  
- Implement left-hold state machine on TX
- Implement 0xF1 meta-packet on both sides
- Implement safety gates on RX (all 10 gates)
- Implement throttle ramp and distance-based stop
- Implement compass heading integration
- Bench test extensively
- Field test per Test Plan

### Phase 5: Fix critical bugs (from V2 analysis)
Before field testing RTM, all 7 critical bugs from analysis should be fixed to prevent crashes during autonomous operation.

---

## 10. Open Questions

1. What GPIO on RX will be used for potential future relay/lights? (Future expansion, not Phase 4)
2. Should RTM require a minimum user GPS accuracy (HDOP) before arming? (Proposed: HDOP < 2.0)
3. Should RTM include a user-audible/visible confirmation beep/LED when engaging? (TX has no buzzer currently)
4. What happens if user activates RTM but RX has follow-me mode already active? Priority conflict resolution needed.
5. Should there be a "kill switch" - any button press during RTM that immediately disengages?
6. Should RTM auto-disengage if buggy is upside down/flipped? (Would need IMU not present)

---

## Next Steps

1. User reviews this design document
2. Open Questions are resolved through discussion
3. Complete TX GPS implementation (current Priority 1)
4. Fix 7 critical bugs from V2 analysis
5. Begin Phase 2 (LoRa meta-packet infrastructure)
6. Continue through Phases 3-4

**Do NOT begin implementation until this document is reviewed, open questions resolved, and user explicitly approves each phase.**

---

## 11. Dual GPS Anti-Spoofing Architecture

### 11a. Design Overview

TX (BN-220) and RX (BN880) are two independent GPS units that can be cross-validated. Spoofing one unit is plausible. Spoofing both consistently — at close range, while matching VESC telemetry — is implausible. This architecture layers three phases to exploit that redundancy.

Phase A always runs. Phase B activates when TX is paired. Phase C activates only during RTM. Each phase is independently useful; none requires the previous to be implemented first (though B requires 0xF3 infrastructure from Priority 4).

### 11b. Phase A — RX Standalone (Priority 3, ~20 lines in RX GPS.ino)

Always on. No TX dependency. Catches obvious spoofed signals before they reach any RTM logic.

- **HDOP gate**: signal quality check — weak satellite lock rejected immediately
- **Acceleration gate**: physics check — no real vehicle does > 3G without a crash
- **Teleport gate**: continuity check — GPS position cannot jump faster than `gps_max_teleport_kmh` (default 80 km/h, 2× foil/buggy max speed)

These three checks are independent and can be evaluated in any order. Failure counter resets to 0 on any passing reading.

### 11c. Phase B — TX/RX Handshake Cross-Validation (Priority 4 infrastructure, ~30 lines)

On TX connect and every 30 seconds. TX sends its GPS lat/lon via 0xF3 meta-packet. RX receives it and computes:
- Straight-line distance between TX and RX GPS positions
- Difference between TX and RX GPS-reported speeds

If TX and RX GPS agree on both distance and speed within thresholds, both readings are likely real. If they disagree, at least one GPS is wrong (spoofed or faulty). RTM arming is blocked until agreement is restored.

### 11d. Phase C — RTM-Active Verification (Priority 5, ~15 lines in RX RTM state machine)

Only during active RTM. Adds behavioral checks that only make sense when the buggy is supposed to be moving toward the user.

- **Convergence**: if RTM is steering correctly, distance must decrease. Stagnant or increasing distance indicates steering failure, GPS error, or obstacle — stop.
- **VESC cross-check**: the physical drivetrain reports wheel speed. If GPS says 15 km/h but VESC ERPM implies 2 km/h, one sensor is wrong — stop.
- **TX freshness**: stale TX GPS during active RTM means the steering target is invalid — stop.

### 11e. New RX SPIFFS Parameters (7 total — all must follow Section 10 Web Config UI Rule in CLAUDE.md)

| Name | Range | Default | Units | Phase | Description |
|---|---|---|---|---|---|
| `gps_max_hdop` | 0.5–5.0 | 2.0 | — | A | Maximum HDOP for a valid GPS reading |
| `gps_max_accel_g` | 1.0–10.0 | 3.0 | G | A | Maximum implied acceleration between readings |
| `gps_max_teleport_kmh` | 50–500 | 80 | km/h | A | Maximum position-implied speed for teleport check (80 km/h = 2× max craft speed) |
| `gps_suspect_threshold` | 1–10 | 3 | count | A | Consecutive failures before GPS rejected |
| `gps_max_pair_dist_m` | 50–2000 | 500 | meters | B | Maximum plausible TX-RX distance at handshake |
| `gps_max_speed_diff_kmh` | 10–200 | 50 | km/h | B | Maximum TX-RX speed difference for handshake |
| `rtm_vesc_speed_diff_kmh` | 5–50 | 20 | km/h | C | Maximum GPS vs VESC speed difference during RTM |

### 11f. Implementation Notes

- All 7 parameters must be added to Section 4c rule reminder (confStruct + SPIFFS + Web UI + ConfigService)
- Phase A can be implemented standalone (Priority 3) without touching any other feature
- Phase B requires Priority 4 (0xF3 meta-packet) to be complete first — do not implement B before P4
- Phase C is implemented as part of the RTM state machine during Priority 5
- TX-side code changes are limited to 0xF3 sending (Priority 4); Phases A, B, C all live on RX
