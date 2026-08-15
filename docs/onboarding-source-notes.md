# Onboarding Source Notes (raw material for the "Zero → Foiling" guide)

Owner-dictated 2026-07-24. Raw notes — the structured guide is built from these + a firmware/web audit.

## Button functions (as owner uses them)
- **BIND button:**
  - **Press once (at boot / to pair):** binds the remote to the RX (pairing).
  - **Press again once it's already on / running:** runs the **compass calibration** ("complex calibration") — the 2-circle cal.
  - *(Firmware ref: BIND at boot = pair; BIND short-press while running = compass cal, 5 blinks start / 2 success / 10 no-compass. BIND+AUX at boot = factory wipe.)*
- **AUX button (logging):**
  - Logging is **OFF by default** (`logger_en = 0` at boot) — this is intentional, not a fault.
  - **AUX short-press while running toggles logging ON/OFF per session.** Owner loves this: press to start logging, the lights turn off = it's logging the current session; press again to stop.

## Logging — what it should capture (owner request)
Telemetry to log per session, so range/performance at current TX+RX settings can be measured:
- **amps in / amps out** (battery current + motor current)
- **speed**
- **distance**
- **link quality** (RSSI/SNR) — to measure how far you can go at current settings
Pull logs via the **website (WiFi)** or the **RX** (serial/CSV). *(Audit: confirm the RX logger CSV columns actually include these — amps in/out, speed, distance, link quality — and add any missing.)*

## Screens
- Screen differences / display walkthrough — owner to detail later. Placeholder for the guide's "reading the display" section.

## Design intent for the public release
- Sanitized public firmware ships **fully unbound/unconfigured** (paired off, neutral cal). On first flash the beta tester **sets everything up via the serial console + the web portal** (good labels, working website). Nothing personal ships.

## ⚠️ Difficulties / gotchas the guide MUST cover (owner-flagged 2026-07-24)
These are the two steps people get wrong — call them out prominently.

**1. VESC PPM mapping — do it on BOTH VESCs, matched across the full range.**
- Run VESC Tool's **PPM Input calibration on VESC1 AND VESC2** — capture **min / center / max** so each maps the *full* RX pulse range (owner's example: min ~1.24 ms, center ~1.70 ms, end ~2.14 ms, zero deadband).
- Tune so **both motors start at the same low throttle AND top out together** — well-calibrated from the start of the PPM range to the end, so the two motors match. Set the **min just above the rest pulse** so there's no idle creep (owner: 1.236 crept, 1.240–1.251 clean).
- This is the fix for "one motor starts before the other / uneven steering" — mismatched PPM maps are the root cause.

**2. Steering direction — verify the cross-steer, wheels-up, before trusting FM.**
- Differential steering is **cross-steered**: to turn **RIGHT**, the **LEFT motor (motor 1 / black)** spins up (drives the left side forward → craft yaws right). Left turn → right motor.
- **Check:** stick/toggle right → buggy turns right. If it turns the WRONG way, flip **`steering_inverted`**. (Owner's build needed `steering_inverted = 1`; the public default is `0` — every builder verifies their own.)
- Confirm the **motor assignment** (which physical motor is 1 vs 2 / left vs right) and the **motor spin direction** (prop thrust pushes the correct way — separate from steering_inverted; that's VESC `m_invert_direction`).
- Simple, but **do it wheels-up first** — a wrong steering sign makes FM steer *away* from the rider (closed-loop runaway). This is the single most important pre-water check.
