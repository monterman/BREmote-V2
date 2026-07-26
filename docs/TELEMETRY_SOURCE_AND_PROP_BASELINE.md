# Telemetry Source, RX-vs-VESC Validation, and Prop Baseline

**Established 2026-07-26.** Owner-confirmed wiring fact + a cross-instrument validation of the RX telemetry + the first propeller current baseline.

---

## 0 · 🔒 MOTOR / VESC IDENTITY MAP — owner-set, do not re-derive

> Owner set the CAN IDs deliberately so they match the motor numbers. **Confirmed 2026-07-26.**

| Motor | VESC | **CAN ID** | BLE MAC (last 2) | Prop as of 2026-07-25 |
|---|---|---|---|---|
| **M1** | VESC 1 | **1** | `…d6` | Black, **1.0** pitch — the original, in every prior logged run |
| **M2** | VESC 2 | **2** | `…56` | White, **0.9** pitch — replaced the broken original |

**How to identify a log without asking anyone:**
- **VESC Tool RT log** → the `vesc_id` column *is* the CAN ID, so `vesc_id = 1` means **M1**. (⚠️ The BLE MAC is **not** written into the RT log — only `vesc_id`. Searched all 55 columns of the 2026-07-25 log: no MAC, no UUID, no serial.)
- **RX log** → always M1, by wiring (§1).

---

## 1 · 🔒 WIRING FACT — the RX UART is always on VESC 1 (M1, black motor)

> **Owner-confirmed:** *"the RX is always connected via UART to VESC 1, black motor, M1."*

**Every number in an RX log is motor 1 only.** The RX queries one VESC with `COMM_GET_VALUES_SELECTIVE` over UART. There is **no CAN forwarding in the RX firmware**, and a VESC's `GET_VALUES` reports its own motor — never the CAN slave's. That is why the log columns are singular: one `motor_current_A`, one `ERPM`, one `duty_cycle_%`.

**Consequence:** an RX log can characterise **M1 only**. To compare both motors you need a VESC Tool RT log (see §3).

---

## 2 · ✅ The RX telemetry is validated against VESC Tool

**Two independent instruments recording the SAME RUN simultaneously.** (The VESC Tool filename says `185150` — local Chicago time — but its internal `ms_today` reads 23:51:50, the same UTC clock the RX uses. The windows overlap: RX 23:51:03→23:55:08, VESC 23:51:50→23:56:57.) Same motor, same physical event, measured twice:

| | VESC Tool RT (M1) | RX log | Agreement |
|---|---|---|---|
| Peak motor current | 62.5 A | 59.9 A | 4% |
| Peak ERPM | 37,852 | 37,630 | **0.6%** |
| Max duty | 96% | 96% | exact |
| Min voltage (sag) | 34.7 V | 34.8 V | **0.1 V** |

**Prop signature — amps per 1000 ERPM (the curve *shape* must match, not just the peaks):**

| Duty band | VESC RT | RX log |
|---|---|---|
| 15–25% | 1.81 | 1.77 |
| 25–40% | 1.77 | 1.66 |
| 40–60% | 1.74 | 1.74 |
| 60–80% | 1.42 | 1.34 |

The characteristic **roll-off above 60% duty** (pack sagging, not prop behaviour) appears in both.

### Row-by-row check — and its honest limit
Because the runs are simultaneous, the samples can also be time-aligned directly. Sweeping the clock offset, ERPM correlation **peaks sharply at +2.5 s (r = 0.773)** and decays either side (r = 0.11 at −10 s, 0.32 at +10 s). **That peak is the signature of genuinely correlated data**, with a real ~2.5 s offset between the RX's GPS-UTC stamp and VESC Tool's own clock.

**Why r = 0.77 rather than ~0.95, and why that is NOT instrument disagreement:** the RX timestamp resolution is **1 second** (GPS updates at 1 Hz, so several RX rows share a stamp) while ERPM slews at **>2,100/s at the 90th percentile**. During a throttle ramp, one second of quantisation alone accounts for thousands of ERPM. The row-by-row test is **resolution-limited, not accuracy-limited**.

> **Use the peak/sag/curve-shape agreement as the evidence of accuracy** — those are insensitive to timebase. Do not read the 0.77 as a defect.

> **Conclusion: RX telemetry reports what the VESC reports.** This matters far beyond props — it underwrites the TX display, the CSV logs, and every analysis built on them.
>
> **Practical upshot: props can be characterised from an RX log alone** — aux button, no laptop, no VESC Tool. Reach for the RT log only when you need *both* motors.

---

## 3 · How to compare BOTH motors — the `_setup` columns

A VESC Tool RT log carries `vesc_id`, `num_vescs`, and the **`_setup` columns which aggregate across CAN**:

```
M2 current = current_motor_setup − current_motor
M2 input   = current_in_setup    − current_in
```

Confirmed on the 2026-07-25 log: `vesc_id = 1`, `num_vescs = 2`, `current_motor` peak 62.5 A vs `current_motor_setup` peak 123.6 A.

**Note:** ERPM has no `_setup` equivalent — **RPM is available for the logged VESC only.** Current is derivable for both; RPM is not.

---

## 4 · Propeller baseline — 3D-printed 2-blade set

All props are 2-blade, 3D-printed, differing only in pitch and handedness (left/right mirror). Labels are the **pitch degree difference** between the three.

| Prop | Pitch label | Motor (2026-07-25 test) | Notes |
|---|---|---|---|
| Black | **1.0** | M1 | The original — used in every previously logged run |
| White | **0.9** | M2 | Replacement after the original white prop broke (harbour session) |
| — | **1.1** | not yet tested | |

### Measured: 1.0 vs 0.9 — the difference is negligible

Bucket test, both motors at the same throttle, derived via the `_setup` columns:

| Duty band | M1 (1.0) | M2 (0.9) | M2/M1 | ERPM |
|---|---|---|---|---|
| 5–15% | 2.6 A | 3.6 A | 1.40 ⚠️ | 2,237 |
| 15–25% | 14.0 A | 14.8 A | 1.06 | 7,869 |
| 25–40% | 20.4 A | 21.2 A | 1.04 | 11,764 |
| 40–60% | 33.0 A | 34.0 A | 1.03 | 19,409 |
| 60–80% | 37.6 A | 38.2 A | 1.02 | 26,656 |
| 80–99% | 38.3 A | 36.6 A | 0.96 | 35,526 |

**Overall: M1 19.4 A mean / 62.5 A peak · derived M2 19.8 A mean / 64.5 A peak.**

### ⚠️ RETRACTED: "the props differ by 2% and are interchangeable"
**That conclusion was wrong and is withdrawn.** It claimed a resolution this method does not have.

**M2 was NEVER directly logged.** Both available logs are `vesc_id = 1`. The M2 column above is *derived* by subtracting the local VESC from the CAN-aggregated `_setup` value — an estimate, not a measurement.

Quantifying that estimate's error on the 2026-07-25 log (626 rows under load):

| Check | Result | Reading |
|---|---|---|
| Derived M2 negative | 0.6% of rows | not garbage — the signal is real |
| `setup`/`local` ratio | mean 2.02, p5–p95 **1.76–2.25** | broadly consistent with a second motor |
| **mean \|setup − 2×local\|** | **2.15 A** | **the noise floor of the method** |

**±2.15 A of scatter on a ~19 A mean is roughly ±10%. The claimed 2% difference (≈0.4 A) is five times smaller than the method's own noise.** It was never measurable.

**Why the scatter exists:** `_setup` values are assembled from **CAN status messages arriving on a different cadence** than the local VESC's instantaneous readings. Subtracting two quantities sampled at different instants — during throttle transients, on a rig with a **documented history of a flaky inter-enclosure CAN link** — produces an estimate with a wide error bar, not a measurement.

**What IS supportable:** no *gross* difference between the two motors is evident. Anything below roughly **10–15%** is invisible to this technique, and a 0.1° pitch effect would sit well inside that.

**To actually compare props, log VESC 2 directly** — connect VESC Tool to the `…56` unit and record it, rather than deriving it. Better still, run both props on the *same* motor in back-to-back tests, which removes motor-to-motor and ESC-calibration differences from the comparison entirely.

- ⚠️ **Ignore the 5–15% band.** At 2–3 A you are in cogging/startup territory where small absolute differences look enormous proportionally.
- **Peak combined battery draw: 109.8 A** (`current_in_setup`) — a serious load for a 10S4P pack; relevant to pack-health decisions.

### Method for testing the other props — do NOT repeat the mistake above
The `_setup` subtraction is too coarse (±10%) to compare props. Two better methods, in order:

1. **⭐ Back-to-back on the SAME motor.** Run prop A on M1, log it, swap to prop B on M1, log again. This removes motor-to-motor variation, ESC calibration, and CAN estimation entirely — the only thing that changed is the prop. **Highest confidence, and it needs nothing but the RX log.**
2. **Log both VESCs directly.** Record VESC 1 (`…d6`) and VESC 2 (`…56`) as separate VESC Tool sessions rather than deriving one from the other.

**And remember what a bucket can and cannot show.** A static test measures *load* (amps at a given RPM) — useful for sizing. It cannot measure **efficiency**, because nothing is moving and no useful work is being done. **A prop that is poor on the water can look identical to a good one in a bucket.** Efficiency only comes from on-water data: speed against power (see §6).

---

## 5 · Reading the "amps per 1000 ERPM" signature
This is the prop's fingerprint: roughly how hard it bites the water, independent of throttle position. **Higher = more load per unit RPM.** It held flat at **~1.77 from 15–60% duty** for the 1.0 prop, which is the number to compare future props against. Values above ~60% duty are contaminated by pack sag and should not be used for comparison.
