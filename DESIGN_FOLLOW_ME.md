# DESIGN_FOLLOW_ME — FM Autonomous Following (Priority 9)

**Project:** BREmote V2.5-Evo
**Date:** July 18, 2026
**Status:** Approved design — not yet implemented. Prerequisite: heading-feedback triage (see §9 Phase 0.5).
**Baseline:** `feature/collision-backoff` @ `d68ed07`.
**Sibling doc:** `DESIGN_RETURN_TO_ME.md` (RTM — implemented, field-tested). FM reuses RTM's validation phases, safety gates, and steering pipeline.

## 1. Purpose and Safety Philosophy

After the whip, the rider releases the rope and rides the wave; the buggy is unmanned on its last heading. FM makes the buggy trail the rider at a configured offset and distance, steering itself, while the rider keeps eyes on the wave.

Non-negotiable (identical to RTM, overrides everything below):
1. The buggy ONLY moves while the rider physically holds the throttle trigger.
2. Autonomous systems ONLY steer and SUBTRACT throttle — never add.
3. Release throttle = buggy stops immediately, always.
4. Every failure path (GPS, compass, LoRa, geometry) drives motor → 0.

FM = autonomous steering + distance-based throttle limiting, under a human throttle hand.

## 2. Canonical mode mapping (single source of truth)

| Mode | Meaning | Geometry |
|---|---|---|
| 0 | Off | — |
| 1 | **Near-Right** (default) | behind-right diagonal at `near_diag_offset_deg` |
| 2 | **Behind** | directly behind rider's course |
| 3 | **Near-Left** | behind-left diagonal |

This TX-side convention is canonical for ALL surfaces (TX display F0–F3, RX struct, both web UIs, README). RX comments/labels currently disagree and must be aligned as part of implementation. Default = 1 (rider is left-foot-forward; the right side is the visible side).

## 3. What already exists (reuse verbatim — do not reimplement)

- **Mode selection plumbing:** TX gesture → 0xF2 meta-packet ×3 → RX `fm_mode_runtime` (RAM). 30 s keepalive; RTM preemption via silent disarm (0xF2/0).
- **Validation:** Phase A (RX GPS: HDOP/teleport/accel → `gps_rejected`), Phase B (TX↔RX cross-validation on 0xF3, 30 s pass / 2 s fail cadence), Phase C pattern (periodic behavioral checks).
- **Steering pipeline** (`RTMState.ino`): EMA target-position filter (τ per preset), P+D controller `kSteerPresets[5]`, heading-source ladder (GPS COG → compass snapshot → live), confidence-scaled authority, wrap-to-±180° error handling.
- **Throttle patterns:** subtract-only cap chain (`PWM.ino:47, 52–55`), approach-zone linear decel ramp, Align-phase ~5 % cap, Run-phase speed governor, engage ramp machinery.
- **Rider position:** 0xF3 GPS meta at 2 Hz (always on with TX fix) — measured sufficient: 2.0–2.4 m rider displacement per update at typical ride speed, 4.5 m at 20 mph, vs 6 m follow gap.
- **Telemetry slots:** `fm_heading_err` (idx 14) and `fm_status` (idx 15) already reserved in `TelemetryPacket`.

## 4. State machine (RX)

```
FM_IDLE → FM_ARMED → FM_ACTIVE → (FM_ARMED | FM_IDLE)
```

- **FM_IDLE:** `fm_mode_runtime` = 0 or unset. New behavior: implement the documented 0xFF → `usrConf.followme_mode` fallback (currently missing).
- **FM_ARMED:** mode 1–3 selected; all monitoring runs; throttle chain inactive until activation conditions met. Display/telemetry reflect armed state (`fm_status`).
- **FM_ACTIVE:** engaged when ALL activation conditions hold (§5). Steering override on; throttle cap chain on (§7).
- **Demotion:** any §5 condition failing → FM_ARMED with throttle cap 0 (motor stops; mode stays selected; re-engages when conditions restore — with the engage ramp, never a jump).
- **Mutual exclusion with RTM:** unchanged (RTM arming silently disarms FM).

## 5. Activation / hold conditions (ALL required while FM_ACTIVE, checked every loop)

1. Throttle held: `thr_received ≥ 25` (absolute rule; motor is already 0 without it).
2. Phase A pass (RX GPS not rejected).
3. Phase B pass (TX↔RX cross-validation current).
4. TX GPS age < `tx_gps_stale_timeout_ms`.
5. RX GPS age < 6000 ms.
6. Valid heading source (per heading ladder; `rtm_compass_required` honored).
7. LoRa healthy: `millis() − last_packet < failsafe_time`.
8. Rider beyond engage distance: `dist > min_dist_m + followme_smoothing_band_m` to activate; deactivate (cap 0) only when `dist < min_dist_m` (Schmitt hysteresis — no flapping at the band edge).
9. Rider moving: rider speed ≥ `foiler_low_speed_kmh`. Below it (rider down, idle, pumping slowly) → cap 0, hold FM_ARMED. Prevents maneuvering around a swimmer.

Failure of 1–7 → cap 0 immediately (same posture as RTM gates). 8–9 are geometric holds, not faults.

## 6. Target-point computation (the new control code)

Per control tick (10 Hz), all on RX:

1. **Filter:** EMA the raw 0xF3 rider position with the active preset's τ (same code path RTM uses).
2. **Rider course + speed:** derived from successive filtered positions. Valid only while rider speed ≥ ~5 km/h (measured: course noise doubles below ~3 mph). Invalid course → degraded mode: offset direction = bearing(rider → buggy) ("hold station at distance"), no diagonal.
3. **Lag anchor:** a first-order EMA trails a moving target by v·τ (13–18 m at 15–20 mph with τ = 2 s — larger than the follow gap and speed-dependent). Correct it: `anchor = filtered_pos + û_course · min(v_rider · τ_active, 2 · d_follow)`. This makes geometry speed-independent without touching the proven smoothing (deliberately NOT solved by shrinking τ: the filter must keep ignoring carves — measured p95 turn rate 49°/s at ~5 m radius; the buggy follows the low-passed path, never mirrors bottom turns).
4. **Trailing point:** `target = anchor − d_follow · R(offset) · û_course`, where `d_follow = min_dist_m + followme_smoothing_band_m` and offset = 0° (mode 2), −`near_diag_offset_deg` (mode 1, rider's right), +`near_diag_offset_deg` (mode 3). Sign convention MUST be fixed against the code's bearing convention and verified in `Tools/FollowMe Settings Visualizer.html` before water.
5. **Steer:** feed `target` to the existing steering pipeline unchanged. Publish `fm_heading_err` and `fm_status` each rotation.
6. **Side-zone hysteresis:** when the angle between rider course and rider→buggy bearing crosses `zone_angle_enter_deg`/`zone_angle_exit_deg`, blend diagonal ↔ pure-behind (Schmitt pair) so an unstable rider course cannot whip the target point across the wake.

## 7. Throttle cap chain (subtract-only; lowest cap wins)

| # | Cap | Source pattern |
|---|---|---|
| 1 | Hard stop: `dist < min_dist_m` → 0 (re-engage per §5.8 hysteresis) | RTM gate-9 distance check |
| 2 | Approach ramp: linear 255→0 across the smoothing band | `RTMState.ino:569–597` verbatim |
| 3 | Speed governor: toward `min(boogie_vmax_in_followme_kmh, rider_speed + closing margin)` | Run-phase governor |
| 4 | Align phase: heading error > threshold → ~5 % cap | Align-phase pattern |
| 5 | Engage ramp: 0→cap over 3–4 s on every FM_ACTIVE entry | RTM ramp machinery |

FM writes caps only. The human trigger remains the sole throttle source; trigger release stops the buggy through the unchanged base architecture.

## 8. Parameters — zero new confStruct fields for v1

All eight existing FM params acquire their intended meaning (no SW_VERSION bump; owner sets values via web UI):

| Param | v1 role | Owner default |
|---|---|---|
| `followme_mode` | SPIFFS default mode (0xFF fallback) | 1 (Near-Right) |
| `min_dist_m` | hard-stop distance | 4 m |
| `followme_smoothing_band_m` | hysteresis + ramp band (follow point = sum = 6 m) | 2 m |
| `near_diag_offset_deg` | diagonal offset (modes 1/3) | 45° |
| `boogie_vmax_in_followme_kmh` | FM speed ceiling | 25 km/h (~15.5 mph) |
| `foiler_low_speed_kmh` | rider-down gate | 8 km/h (~5 mph) |
| `zone_angle_enter_deg` / `zone_angle_exit_deg` | side-zone Schmitt pair | 35° / 45° |

Field-retunable to 4/10/20 m equivalents without reflash. At the next SW_VERSION bump (whenever one happens for other reasons), bake the proven values into `defaultConf` on both sides — a version bump resets stored config to `defaultConf` (verified against `ConfigService`/`SPIFFSEngine`; web-UI-only tuning does not survive bumps).

Deferred to a future bump (one field, shared with RTM): an RX-side autonomous-runtime cap. Neither RTM nor FM has one today (TX `rtm_max_runtime_s` defaults to 0 = disabled).

## 9. Failure modes and recovery

| Failure | Detection | Response |
|---|---|---|
| TX (rider) GPS loss | age > `tx_gps_stale_timeout_ms` | cap 0, hold FM_ARMED, re-engage via ramp on recovery |
| RX GPS loss | age > 6 s | same |
| Heading source invalid | ladder empty | same |
| LoRa loss | > `failsafe_time` | PWM pulses stop (existing failsafe), independent of FM |
| Rider inside stop radius | dist < `min_dist_m` | cap 0 until beyond band |
| Rider down/slow | speed < `foiler_low_speed_kmh` | cap 0, wait |
| Rider course invalid | speed < ~5 km/h | degrade to hold-station (no diagonal) |
| RTM armed | 0xF2/0 silent disarm | FM off (existing) |
| Trigger released | physical | motor stops (base architecture, untouched) |

**Phase 0.5 prerequisite (blocking):** log analysis of all recorded RTM activity shows GPS COG has never engaged as heading source (0 of 1,946 active samples), the compass shows large swings under motor current (evidence contaminated by uncalibrated logs — measure with `?magtest` after the telemetry fix), and steering actuation was intermittent despite valid error. FM ships on this same heading ladder. Before FM implementation: fix the RX GPS feed to the controller, characterize/compensate compass-under-load, close the actuation gap, guard against engagement with no fix, clamp `d_error` across snapshot refreshes. This also resolves the RTM "drives in circles" defect.

## 10. Test plan

1. **Bench, motor off:** mode plumbing end-to-end; simulated coordinates through the target-point math; sign/offset verification against the Settings Visualizer; hysteresis boundaries; every §5 condition force-failed → cap 0.
2. **Bench, wheels up:** cap chain order; engage ramp; align-phase cap; compass-under-load table (from Phase 0.5).
3. **Controlled water, tethered:** mode 2 (Behind) only, walking-pace rider, hard-stop verification, GPS-denial stop.
4. **Field, incremental:** mode 2 first at 6 m; then mode 1 (Near-Right); throttle hand ready to cut throughout; logging enabled (aux button) every run.

## 11. Implementation order

1. Phase 0.5 heading-feedback triage (blocking).
2. RX: FM state machine + activation conditions (§4–5).
3. RX: target-point computation (§6) feeding the existing steering pipeline.
4. RX: throttle cap chain (§7) from existing patterns.
5. Both web UIs + RX comments: canonical mapping + param descriptions (§2, §8); README table.
6. Telemetry: populate idx 14/15; TX display FM-active state.
7. Safety audit (motor-safety verdict first), then §10 gates in order.

No TX control-path changes. No LoRa protocol changes. No new packets, no rate changes (2 Hz 0xF3 measured sufficient).
