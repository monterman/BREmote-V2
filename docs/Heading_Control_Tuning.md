# BREmote V2.5-Evo — Heading Control Tuning Guide

**Document version: 1.1 — 2026-05-08 — Bundle 1 reference (Bundle 5b adds ArduPilot architectural context)**
Applies to firmware SW_VERSION 31 and later.

## TL;DR

The buggy steers itself toward the user (RTM) or the surfer (Follow-Me) using a PID-style heading controller. You don't need to tune individual gains — you pick one of 5 presets via the web UI based on water conditions. **Default: preset 2 — Normal.** Lower numbers (Soft, Very Soft) = gentler / more-damped, for choppy water and aggressive surfer turns. Higher numbers (Sharp, Very Sharp) = more responsive, for flat water and RC-style use.

## 1. What this controller does

The buggy reads two angles each cycle:

- **Current heading** — where the buggy is actually pointing. Taken from GPS course-over-ground (COG) when speed is above the threshold; compass snapshot at very low speeds.
- **Target bearing** — direction from the buggy's GPS position to the target's GPS position. The "target" is the TX (handheld remote) for RTM, and a smoothed version of TX position for Follow-Me.

The difference is the **heading error** (in degrees, signed, wrapped to ±180°). The controller turns this number into a steering output (0-254, with 127 = wheels straight).

Three things shape the output:

1. **Proportional response (Kp)** — immediate "turn toward target" signal proportional to heading error.
2. **Damping (Kd)** — eases off when the buggy is already rotating fast in the right direction, preventing overshoot.
3. **Target-position filter (τ)** — for Follow-Me, smooths the surfer's high-frequency wobbles (bottom turns) so the buggy follows the surfer's average path instead of every twitch.

Together: P drives the chase, D applies the brakes near target, and the filter makes Follow-Me usable when the surfer is doing aggressive turns on a wave.

## 2. The three tuning knobs

### Kp — proportional gain

How aggressively the buggy chases the target heading.

- **Too low** → buggy is sluggish; small heading errors barely move the wheels.
- **Too high** → buggy snaps into corrections. Combined with momentum, this causes "snaking" or "tail-wagging" as the buggy overshoots heading repeatedly.
- **Just right** → buggy turns decisively when far off course, eases into the target heading without overshoot.

Baseline: 1.0 (Normal preset).

### Kd — derivative gain ("damping")

How strongly the buggy applies the brakes when error is shrinking fast.

- **Too low (or 0)** → no damping. Pure-P oscillates as the buggy's mass overshoots target heading.
- **Too high** → over-damped. Buggy feels stuck or sluggish, fights the user's intent, never quite arrives.
- **Just right** → arrives at target heading without overshoot ("critically damped").

Baseline: 0.3 (Normal preset). Bigger waves and aggressive surfer turns favor higher Kd.

### τ — target-position filter (seconds)

The filter time constant on the target's GPS position. Specifically, a first-order low-pass exponential moving average: filtered_pos += (dt / (τ + dt)) * (raw_pos - filtered_pos)  This is the knob that makes Follow-Me work when the surfer is doing bottom turns:

- **τ = 0** → no smoothing. Buggy chases every TX position update. Surfer cuts left → buggy turns left. Buggy snakes.
- **τ = 0.5s** → light smoothing. RTM use (TX stationary); doesn't matter much since the target isn't moving.
- **τ = 2-3s** → moderate path-following. Surfer's 1-2 second bottom turns get smoothed out; buggy follows the average direction.
- **τ = 5s** → heavy smoothing. Big-wave session, aggressive turns. Buggy lags real direction changes by ~5s but doesn't snake.

For RTM where the user is stationary, τ does basically nothing useful (no movement to smooth) but doesn't hurt either. It's a single code path; the only difference is the τ value loaded from the active preset.

## 3. The 5 presets

| # | Name | Conditions | Clamp | Kp | Kd | Filter τ |
|---|---|---|---|---|---|---|
| 0 | **Very Soft** | Heavy surf, big waves, aggressive surfer | ±150° | 0.70 | 0.50 | 5.0s |
| 1 | **Soft** | Choppy water, normal session | ±120° | 0.85 | 0.40 | 3.0s |
| 2 | **Normal** | DEFAULT — mixed conditions | ±90° | 1.00 | 0.30 | 2.0s |
| 3 | **Sharp** | Calm water, calm conditions, RC-style use | ±60° | 1.20 | 0.20 | 1.0s |
| 4 | **Very Sharp** | Glass-flat lake, no waves, parking lot | ±45° | 1.40 | 0.10 | 0.5s |

**Clamp** is the heading error magnitude that maps to full lock. ±90° (Normal) means: 90° of error gives full left or right lock. ±45° (Very Sharp) means full lock at half that error — much more aggressive turn-in for small errors.

### How to pick

- **Default to preset 2 (Normal).** It's the baseline and works for most conditions.
- **Choppy day, real surfer doing wave riding** → preset 0 or 1. Heavy filtering ignores bottom turns; high damping resists wave-induced heading shake.
- **Flat-water lake, towing a beginner around** → preset 3 or 4. Crisp response; filter doesn't matter when the surfer isn't doing much.
- **One beach, conditions build through the day** → start at 2, switch to 1 when whitecaps appear. ~10 seconds in the web UI to change between sessions.

### When NOT to bottom-out the presets

- **Don't pick Very Sharp for surf.** Buggy will snake aggressively as surfer turns.
- **Don't pick Very Soft for flat water.** Buggy will feel mushy and slow to respond.
- **Don't pick Very Sharp on a stationary-user RTM run.** Aggressive turn-in is wasted; the user isn't moving.

## 4. RTM vs Follow-Me — same controller, different effective behavior

One controller, same code path. The difference comes from what's being filtered:

**RTM (Return-to-Me, user stationary):**
- TX position barely changes. Filter τ acts on a near-constant input → near-constant output.
- Filter contributes no real behavior. Controller behaves like a P+D controller chasing a stationary target.
- Preset choice mostly affects how aggressively the buggy turns toward the user.

**Follow-Me (TX moving, surfer riding waves):**
- TX position updates rapidly with surfer's wobbles. Filter has real work to do.
- Buggy chases the *smoothed* position, not the instantaneous one. Higher τ → buggy follows path, not turns.
- Preset choice now also controls how much surfer wobble gets ignored.

## 5. Tuning from log data

Two diagnostic columns in every log file (CSV columns 25 and 26 — added in Bundle 1):

- `heading_error_dx10` — current heading error in 0.1° units. Sentinel value `0x7FFF` (32767) = no valid heading source.
- `d_error_dx10` — rate-of-change of heading error in 0.1°/s units. Sentinel `0x7FFF` = first cycle, no prior sample.

Plot both columns over time. Patterns to look for:

- **Sustained ±20-50° error wobbling at 1-3 Hz** → P-only oscillation. Buggy is snaking. Use a preset with higher Kd, or increase Kd values in the preset table.
- **Error trending steadily toward zero, smooth curve** → controller is working. Ride it.
- **Large `d_error` spikes** → target jumping (GPS noise, or surfer doing aggressive turns). Filter τ should smooth this; if `d_error` spikes are still big, increase τ.
- **Error sitting at a steady 5-15° offset and not closing** → wind / current bias. Currently no I term. Either accept the small offset (buggy still arrives close) or use the Steering Trim setting if the offset is consistent.

After a run, compare the chosen preset's results. If the patterns suggest a different preset would have been better, switch presets in the web UI for the next run. **Don't edit the preset-table values blind** — always test, log, compare.

## 6. Why no I term?

Classical PID adds an integral (I) term that drives steady-state error to zero by accumulating error over time. Useful for compensating consistent disturbances like wind or motor bias. We deliberately omitted it because:

1. **Marine I-term wind-up.** When error oscillates around zero (wave buffeting), the integrator can wind up and cause overshoot when conditions change.
2. **Steady drift handled elsewhere.** The mechanical Steering Trim setting compensates for consistent left/right pull. Wind / current at-sea is rarely consistent enough to integrate against safely.
3. **Path-following matters more than zero-error.** A ~5° steady offset costs almost nothing if the buggy still arrives at the surfer's general path.

Future work: an I term can be added later without restructuring. The preset struct has logical room for `ki` if field data ever shows it would help.

## 7. Glossary

- **PID** — Proportional / Integral / Derivative controller. Industry-standard closed-loop control.
- **Kp / Kd** — proportional and derivative gain constants.
- **τ (tau)** — time constant of a low-pass filter, in seconds. Larger τ = slower response, more smoothing.
- **COG** — GPS Course Over Ground. Direction of motion derived from successive GPS positions.
- **Heading error** — difference between desired bearing and current heading, in degrees, signed and wrapped to ±180°.
- **Critical damping** — the Kd value that brings the buggy to target without overshoot or oscillation. The "sweet spot."
- **Schmitt-trigger hysteresis** — using two thresholds (one for engaging, one for disengaging) to prevent flap-flap when input hovers near a single threshold. Used by `zone_angle_enter_deg` / `zone_angle_exit_deg` for FM.
- **Wind-up** (I-term) — integrator accumulating error during saturation, causing overshoot later.

## 8. References

- **ArduRover** (ArduPilot rover branch) — cascaded PID for ground-vehicle steering. Useful as a *framework* reference: their `STEER_RATE_*` and `STR_TO_SRV_*` parameters are conceptual analogs of our Kp / Kd. The absolute tuning values are NOT directly transferable — ArduRover is tuned for autonomous waypoint-following on much larger, heavier vehicles with different actuator dynamics. **Borrow the structure, not the numbers.**
- **OpenMower / Pixhawk Boat** — PID with I term for marine drift compensation. Their I term works because the disturbances (current, wind) are quasi-steady. We rejected it because tow-buggy use cases see more transient than steady disturbance.
- **Betaflight (drone PID)** — 400 Hz PID with sophisticated filtering. Far more complex than necessary for surface vehicles at 10 Hz LoRa cadence. Useful as a reference for filter design, not for tuning numbers.

### Why we don't run ArduPilot/ArduRover directly

Some custom-built marine projects — for example Mike Holden's [Ardupilot-GPS-Tag](https://github.com/MikeHolden3/Ardupilot-GPS-Tag-Archive), an ESP32 LoRa tag that talks to an onboard Pixhawk running ArduRover BOAT class — get autonomous behavior "for free" by delegating all control to a full ArduPilot stack. We deliberately did NOT take that path:

1. **Safety philosophy is non-negotiable.** ArduPilot's `GUIDED` mode is autonomous by default: the vehicle moves toward the target without any user trigger input. Our hard rule (the Tow Buggy ONLY moves when the user physically holds the throttle trigger) directly conflicts with this. Hacking ArduPilot to require continuous throttle fights the framework's design and adds risk.
2. **Hardware footprint.** ArduPilot needs a Pixhawk-class flight controller (~$100+) plus a separate ESP32 LoRa bridge. Our single ESP32-C3 + VESC stack is leaner, lighter, and cheaper for the use case.
3. **Iteration speed.** Editing a small custom firmware loop and reflashing is faster than configuring an ArduPilot parameter tree, especially for a small purpose-built application.
4. **Scope.** ArduPilot is built for waypoint missions, surveys, RTL, geofences, full mission planning. We need exactly two things: "come back to this person" (RTM) and later "follow this person's path while the user holds the trigger" (FM). A smaller surface is easier to keep safe and predictable.

What we DO borrow from the ArduPilot ecosystem: control-theory structure (PID layout, filter design, anti-windup awareness), log-driven tuning workflow, and validation that "lightweight LoRa GPS feed → downstream controller" is a proven architectural pattern.

---

**Last updated:** 2026-05-08 (Bundle 1 + Bundle 5b)
**Maintainer:** monterman / LudwigBre
**Source firmware:** `Source/V2_Integration_Rx/RTMState.ino` — see `kSteerPresets[]` array and `updateRtmSteering()` function.
