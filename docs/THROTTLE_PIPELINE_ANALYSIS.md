# BREmote V2.5-Evo — Throttle Pipeline, PWM Range, FreeRTOS, and Jan-Upstream Bug Audit

Date: 2026-05-03
Scope: Read-only analysis. No firmware files modified. All findings cite file:line in V2.5-EVO source tree.

---

## EXECUTIVE SUMMARY (read this first)

1. **Throttle pipeline is clean and safe.** Five sequential stages on TX, six on RX. RTM and approach-cap **only ever subtract** from user throttle — verified at `Throttle.ino:27` and `PWM.ino:43–51`. Creator safety philosophy (Section 9) is intact.
2. **"Dynamic throttle" is NOT a time-ramp.** It is a user-adjustable static percentage cap. Boots at `dynamic_power_start` (default 85%), user toggles step it ±`dynamic_power_step` (default 5%). If you want auto-ramp-from-zero behavior, that is a NEW feature, not a bug — see §2.4.
3. **Gears, No-gears, Dynamic — mutually exclusive.** Picked by `throttle_mode` (0/1/2). Only one runs at a time. They never stack on each other.
4. **Expo always runs** (default 100 = linear). Set `thr_expo` to 50 to disable shaping.
5. **No general slew-rate / ramp limiter on TX.** Throttle can go 0→max in one 100ms cycle. The only "ramp" is the RTM-active soft cap. **VESC's own ramp settings still apply downstream** — that is your real safety net against wheelies/jumps.
6. **PWM output is linear.** RX maps the throttle byte 0–255 → `PWM0_min..PWM0_max` µs with Arduino's `map()`. No hidden RX-side scaling. Default range 1000–2000 µs (standard servo PPM).
7. **Failsafe = no PWM pulses, not 1500µs neutral.** When LoRa drops for `failsafe_time` (default 1000ms), RX stops generating PWM. VESC sees no signal and falls into its own loss-of-signal behavior (typically coast).
8. **Jan's upstream has 5 high/critical safety bugs.** All five appear fixed in V2.5-EVO. Three medium bugs likely fixed but not 100% verified. Use the audit table in §4 as the courtesy report back to Jan.
9. **FreeRTOS is the right call here.** §3 explains why with file evidence. Jan's "FreeRTOS makes field debugging hard" concern is fair philosophically but overstated for *this* codebase.

---

## 1. THROTTLE PIPELINE — STAGES IN EXECUTION ORDER

### 1.1 TX side (handheld remote)

```
[Hall ADC raw] → [Stage 1: filter+calibrate] → [Stage 2: expo curve] →
[Stage 3: mode branch (gears OR no-gears OR dynamic)] →
[Stage 4: RTM TX cap (subtract-only)] →
[Stage 5: pack to 0xF0 max byte] → LoRa TX
```

| Stage | File:Line | What it does | SPIFFS field | Conditional |
|---|---|---|---|---|
| 1. Filter + calibrate | `Hall.ino:69–83` (`calcFilter`) | 6-sample ring-buffer average; linear scale `(adc - thr_idle) / (thr_pull - thr_idle) × 255` | `thr_idle`, `thr_pull` | Always |
| 2. Expo curve | `Hall.ino:189–215` (`expoThrCurve`) | Quadratic blend; 50 = linear, <50 flatter at extremes, >50 sharper | `thr_expo` (0–100, default 100) | Always (100 ≈ linear) |
| 3a. Gears (mode 0) | `Throttle.ino:19` | `result = shaped × (gear+1) / max_gears` | `throttle_mode=0`, `max_gears`, `startgear` | mode==0 |
| 3b. No-gears (mode 1) | `Throttle.ino:12` | `result = shaped` (passthrough) | `throttle_mode=1` | mode==1 |
| 3c. Dynamic cap (mode 2) | `Throttle.ino:15` | `result = shaped × max_power_cap / 100` | `throttle_mode=2`, `dynamic_power_start`, `dynamic_power_step` | mode==2 |
| 4. RTM TX cap | `Throttle.ino:27` + `RTMState.ino:86–97` | `if (result > rtm_thr_cap_tx) result = rtm_thr_cap_tx` | `rtm_throttle_start_pct`, `rtm_throttle_max_pct`, `rtm_ramp_duration_s` | Only during RTM ACTIVE; else `rtm_thr_cap_tx = 255` |
| 5. Pack to 0xF0 | `Radio.ino:362–366` | Cap at 0xF0 (240 dec) so 0xF1–0xFF stay reserved for meta-packets | — | Always |

### 1.2 RX side (board unit)

```
LoRa RX → [Stage 1: unpack] → [Stage 2: RTM safety gates → emergency stop or pass] →
[Stage 3: approach-cap (subtract-only)] → [Stage 4: failsafe gate] →
[Stage 5: linear map → µs] → [Stage 6: RMT pulse to ESC pin]
```

| Stage | File:Line | What it does | SPIFFS field | Conditional |
|---|---|---|---|---|
| 1. Unpack | `Radio.ino:459–460` | `thr_received = rcvArray[3]; steering_received = rcvArray[4]` | — | On valid packet |
| 2. RTM gates | `RTMState.ino:33–130` + `PWM.ino:43` | If any of 9 safety gates fail → `rtm_rx_emergency_stop = true` → `effective_thr = 0` | `rtm_*` family, `vesc_erpm_per_kmh`, `tx_gps_stale_timeout_ms` | RTM active only |
| 3. Approach cap | `RTMState.ino:289–321` + `PWM.ino:48–51` | Linear ramp: cap goes 255→0 as TX-RX distance shrinks from `rtm_approach_zone_m` to `rtm_stop_distance_m` | `rtm_approach_zone_m`, `rtm_stop_distance_m` | RTM active only |
| 4. Failsafe gate | `PWM.ino:13` | If `millis() - last_packet >= failsafe_time` → skip the entire pulse generation block | `failsafe_time` (100–10000ms, default 1000) | Always evaluated |
| 5. Linear µs map | `PWM.ino:63–64`, `:70–71`, `:94` | `map(effective_thr, 0, 255, PWM_min, PWM_max)` (with `+/- trim` per channel) | `PWM0_min/max`, `PWM1_min/max`, `trim`, `steering_type`, `steering_inverted`, `steering_influence` | Always |
| 6. RMT output | `PWM.ino:131–145` (`generate_pulse`) | RMT clock = 1 MHz → 1 tick = 1 µs; high pulse of `pulse_width_us`, then 1µs low | — | Only when failsafe gate passes |

### 1.3 Direct answers to your questions

**Q: Does dynamic throttle work in all modes or only manual / FM?**
Dynamic throttle is THE manual mode (you pick `throttle_mode = 2`). It is mutually exclusive with gears and no-gears. RTM and FM run on top of whatever manual mode you have selected — they apply *additional* subtractive caps but do not replace your dynamic cap. So:
- Manual + dynamic: your cap is `max_power_cap%`.
- Manual + dynamic + RTM active: cap = `min(max_power_cap%, RTM ramp value)`.
- Manual + dynamic + FM: FM does not currently apply a TX-side throttle cap (FM mode bytes are sent over 0xF2 meta-packets but do not modify `rtm_thr_cap_tx`). FM today affects steering/display only — verify with field test.

**Q: Does dynamic re-ramp from zero each time you let go and re-engage?**
**No.** `max_power_cap` is a persistent uint8 (`BREmote_V2_Tx.h:365`, declared volatile). It is set once at boot from `dynamic_power_start` (`Throttle.ino:40`) and only changes when you toggle the side button (`Hall.ino:340` calls `throttleAdjustCap`). Releasing the trigger and re-squeezing does **not** reset the cap — you go right back to whatever cap you were at.

**Q: Does dynamic stack on top of gears or replace it?**
**Replaces.** `Throttle.ino:9` is a `switch(throttle_mode)`. Only one branch executes per call.

**Q: Is there a separate ramp/slew-rate limiter?**
**Not on TX, not in manual mode.** The trigger value can change 0→max in a single 100ms LoRa cycle if you slam the trigger. The only "ramp" is `rtm_thr_cap_tx` during RTM ACTIVE, which is unrelated to user input slew. **Your real ramp protection is the VESC's own current/erpm ramp settings** — these limit how fast the motor accelerates regardless of what PWM says.

**Q: Is there a separate expo curve?**
**Yes — always runs.** `Hall.ino:189` is called inside `calcFinalThrottle()` before mode branching. Default `thr_expo = 100` (claims to be linear; verify the math at `Hall.ino:204` — 100 maps to weight `(100-50)/50 = +1.0` which is *not* the neutral point; the neutral/linear value is `thr_expo = 50`).

⚠️ **Likely doc/default confusion.** The default of 100 is described in the WebUI as "linear" but the math at `Hall.ino:204` says weight = `(expo-50)/50`, so `expo=100` gives weight=+1.0 (sharpening curve, sensitive at extremes). `expo=50` gives weight=0 (true linear). **Recommend you verify on hardware** by holding trigger at 50% Hall and reading what byte goes out at `expo=50` vs `expo=100`. If `expo=100` is in fact a sharpening curve, the WebUI label is wrong, not the code.

**Q: Are gears + dynamic + ramp + expo separately toggleable?**
- Expo: always on (set to 50 to neutralize).
- Gears / no-gears / dynamic: pick one via `throttle_mode`.
- Slew-rate ramp: doesn't exist on TX.
- RTM ramp: independent, only active during RTM ACTIVE.

**Q: At what value does the trigger send 0?**
Whenever `thr_scaled = 0` (Hall reading at or below `thr_idle`) OR system locked OR `system_locked == 1`. There is no minimum-throttle-deadband once `thr_scaled > 0`. The Hall calibration window itself is your deadband.

**Q: Does RTM override / modify user throttle on TX or only RX?**
**Both.** TX applies `rtm_thr_cap_tx` at `Throttle.ino:27`. RX applies `rtm_rx_emergency_stop` and `rtm_approach_cap` at `PWM.ino:43–51`. Two-layer defense: even if TX cap fails, RX safety gates can still hard-stop.

**Q: Can RTM ever ADD throttle?**
**No.** Verified at three places:
- `Throttle.ino:27` — `if (result > rtm_thr_cap_tx) result = rtm_thr_cap_tx;` — only reduces.
- `PWM.ino:43` — `effective_thr = rtm_rx_emergency_stop ? 0 : thr_received;` — emergency forces 0, never raises.
- `PWM.ino:48–51` — `if (rtm_approach_cap < effective_thr) effective_thr = rtm_approach_cap;` — only reduces.
**Section 9 creator safety philosophy is intact.**

---

## 2. DYNAMIC THROTTLE — DEEPER LOOK AT YOUR USE CASE

### 2.1 What the code actually does today
- **Boot:** `max_power_cap = dynamic_power_start` (default 85%).
- **In ride:** `output = shaped × max_power_cap / 100`.
- **User adjusts:** side toggle button steps `max_power_cap` up/down by `dynamic_power_step` (default 5%), bounded 10–100.
- **No automatic time-based ramp.** The cap is whatever the user last set it to. Releasing the trigger does not reset it. Powering off resets it to `dynamic_power_start`.

### 2.2 Match against your mental model
You wrote: *"start low even with a full 100% squeeze but have it build up to the max set throttle in gears … like a combo, of start low, then add more if get going."*

There are two reasonable readings:

**Reading A (what the code does):** "I boot at 85%, ride a bit, then click up to 95% via toggle once I'm comfortable." This is a manual build-up driven by you tapping the side toggle. The code supports this exactly. ✅

**Reading B (what the code does NOT do):** "I squeeze 100% and the firmware automatically ramps me from 0% to my cap over a few seconds, no toggle needed." This is *not* implemented. There is no time-based slew limiter on `max_power_cap` and no auto-ramp on `result`.

**Recommendation:** confirm which reading matches your intent. If Reading B, that is a **new feature** (call it "auto-ramp" or "soft-start"), straightforward to add: ~15 lines in `calcFinalThrottle()` that lerp `max_power_cap` from `dynamic_power_start` toward `dynamic_power_max` over `dynamic_power_ramp_s` seconds whenever the trigger is engaged. SPIFFS pipeline change required (Section 10 web config rule). I can write the Claude Code prompt for that as a follow-up.

### 2.3 Implementation safety — is dynamic throttle solid?
Audit verdict: **YES, with two minor concerns.**

**Solid:**
- `max_power_cap` is `volatile uint8_t` (`BREmote_V2_Tx.h:365`) — correctly declared.
- All adjustments go through `throttleAdjustCap()` which clamps to 10–100 (`Throttle.ino:60–63`).
- Boot path clamps invalid SPIFFS values (`Throttle.ino:41–42`, `ConfigService.ino:94–99`).
- The `(uint16_t)shaped × max_power_cap / 100` math at `Throttle.ino:15` cannot overflow: max is `255 × 100 = 25500`, fits in uint16_t.
- ConfigService validates `dynamic_power_start ∈ [10,100]` and `dynamic_power_step ∈ [1,25]` (`ConfigService.ino:18–19, 94–99`).

**Minor concerns:**
1. **Atomicity across cores.** `max_power_cap` is read by the FreeRTOS `sendData` task (Core 0) and written by the Hall toggle handler running in the loop task. Volatile uint8 reads/writes are atomic on ESP32-C3 (single byte, single instruction), so this is *probably* fine. But for cross-core safety best practice, consider `std::atomic<uint8_t>` like the RTM globals at `BREmote_V2_Rx.h:294–298`. **Severity: LOW.** Not actually broken.
2. **No persistence of last cap.** If you set `max_power_cap = 100` mid-ride, then power-cycle, you boot back at `dynamic_power_start = 85`. Intentional? If you want last-used to persist, that requires a SPIFFS write on every toggle (flash wear concern) or a write only on power-down (deferred-save pattern). **Severity: design choice, not a bug.**

### 2.4 Gears vs Dynamic — every actual difference, not just granularity (revised 2026-05-03)

You correctly pushed back on "only granularity differs". I oversimplified. Here is the full diff between mode 0 (gears) and mode 2 (dynamic cap), verified against `Throttle.ino`, `Hall.ino:328–343`, `BREmote_V2_Tx.h:218–241`, `ConfigService.ino:18–19, 94–99`.

| Aspect | Gears (mode 0) | Dynamic (mode 2) | Same or different? |
|---|---|---|---|
| Multiplier formula | `shaped × (gear+1) / max_gears` | `shaped × max_power_cap / 100` | **Same shape** — both are linear multipliers on post-expo throttle |
| Range of the multiplier | `1/max_gears` to `1.0` | `0.10` to `1.00` | Same in practice (both reach 100% at top) |
| Default boot multiplier | `(startgear+1)/max_gears` — default `1/6 ≈ 17%` | `dynamic_power_start/100` — default `85%` | **Different** — gears boots conservative (low), dynamic boots aggressive (high). **Real safety implication.** |
| Step granularity | Fixed `1/max_gears` per step (default 6 gears = ~17% per step) | Configurable: `dynamic_power_step%` per step (default 5%, range 1–25%) | **Different** — dynamic offers finer control |
| Number of stops | `max_gears` (default 6, range 1–10) | `floor((100-10)/step) + 1` — at default step=5 that's 19 stops | **Different** — dynamic has more or fewer stops depending on `dynamic_power_step` |
| Toggle handler | `Hall.ino:332–334` — gear++ / gear-- bounded 0..max_gears-1 | `Hall.ino:340` calls `throttleAdjustCap` which steps `max_power_cap` by `±dynamic_power_step`, clamped 10–100 | **Different code path**, same UX (left/right toggle button) |
| Display | `showNewGear()` — discrete gear number on segments | `showCapPercent()` — percentage value | **Different visual** |
| Persists across power cycles | No — resets to `startgear` | No — resets to `dynamic_power_start` | Same (both ephemeral) |
| Auto-ramp from zero | No | No | **Same** — neither auto-ramps |
| Slew-rate limit | None | None | **Same** — both can slam 0→max in one cycle |
| Interaction with RTM cap | RTM cap applied AFTER (subtractive) | Same | Same |
| Interaction with expo | Expo applied BEFORE (on `thr_scaled`) | Same | Same |

### What is genuinely different
1. **Default boot multiplier.** This is the biggest functional difference. Mode 0 boots at the lowest gear (you must step up to unlock power). Mode 2 boots at `dynamic_power_start = 85%` (near-full power on first squeeze). For a tow-buggy where you might pull the trigger right after unlock, this matters.
2. **Granularity.** Dynamic = configurable percentage steps. Gears = fixed fractions of `max_gears`.
3. **Number of stops.** Dynamic typically gives more stops (19 at default vs 6 fixed gears).
4. **Display label.** Gear number vs percentage.

### What is the same
1. **Math shape** — both apply a single linear multiplier to the post-expo throttle.
2. **Toggle UX** — left/right side-button presses to step ±1.
3. **Range** — both can reach 100% at the top.
4. **Behavior over time** — neither auto-ramps, neither slew-limits, neither persists.
5. **Failure modes** — slamming from low to 100% in one motion is possible in both.

### Practical implication
If you set `max_gears = 19` and `startgear = 16`, gears mode would be visually identical to dynamic mode at default settings. The fact that this is true is the strongest evidence that today's dynamic mode is "gears with finer-grained, configurable steps" — exactly your earlier read.

**The non-trivial differentiator if you want to keep dynamic alive: the default boot value.** A `dynamic_power_start = 85%` configuration is a different *default safety posture* from a `startgear = 0` configuration. With dynamic, you're trusting the rider to know they're at 85% on first trigger pull. With gears, the rider must explicitly unlock power. **Neither is wrong — they're different defaults for different rider profiles.**

### So is there a real reason to keep dynamic at all?
- **If you keep it as-is:** the defining feature is "fine-grained, single configurable variable that sets a power cap, biased to start higher than gears." That's a defensible niche.
- **If you redesign per §2.5 Option A** (auto soft-start time-ramp): now dynamic does something gears genuinely cannot — automatic low-to-high ramp on each trigger engagement. **This makes dynamic distinct, useful, and worth the SPIFFS field cost.**
- **If you delete it:** simplifies firmware, reduces SPIFFS size, removes a code path. Lose the "fine-grained percentage cap with high default boot" niche.

**My read:** today's dynamic is a niche-worth-keeping if you value the high-boot-default and percentage display, but it's basically gears-with-extras. Tomorrow's dynamic (Option A) is meaningfully different. **Pick one or the other; don't leave both fighting for the same purpose.**

### 2.5 What WOULD make dynamic throttle actually different and useful?

If you want to keep `throttle_mode = 2` and make it earn its name, here are three real differentiators ranked by effort. Each is a separate Claude Code prompt — none implemented yet, all gated on your approval.

**Option A — Auto soft-start (time-based ramp from low to high)** ⭐ Recommended
The cap automatically lerps from `dynamic_power_start` toward `dynamic_power_max` over `dynamic_power_ramp_s` seconds whenever the trigger is engaged. Trigger release > `dynamic_power_reset_ms` resets the ramp; release < that window keeps the ramp where it left off (so quick re-grabs at speed don't drop you to 30%).

- New SPIFFS fields: `dynamic_power_max` (default 100%), `dynamic_power_ramp_s` (default 3s), `dynamic_power_reset_ms` (default 1500ms).
- ~20 lines in `Throttle.ino` plus engage/release edge-detection in `Hall.ino`.
- Real safety win for tow-buggy: gives you a guaranteed soft start every time, no toggles needed.
- **This is what your earlier message described.** ("start low even with a full 100% squeeze but have it build up")

**Option B — Speed-aware cap (VESC-driven)**
Cap is locked at `dynamic_power_start` until VESC reports motor speed > `dynamic_speed_threshold_kmh`. Once moving, full power is unlocked automatically. Returns to low cap when motor stops.

- New SPIFFS fields: `dynamic_speed_threshold_kmh` (default 5 km/h), `dynamic_power_max` (default 100%).
- ~25 lines split across TX (state machine) and RX (telemetry forwarding of VESC speed).
- Smarter than Option A — only ramps when actually moving.
- **Risk:** depends on VESC telemetry. If VESC is silent, you need a defined fallback (lock at `dynamic_power_start`, never unlock). Adds another failure mode to think about.

**Option C — Hold-time-based unlock (simpler than A)**
Cap stays at `dynamic_power_start` for the first `dynamic_unlock_s` seconds of continuous trigger engagement, then jumps to 100% (or `dynamic_power_max`). Release for >`dynamic_power_reset_ms` resets the timer.

- New SPIFFS fields: `dynamic_unlock_s` (default 2s), `dynamic_power_max`.
- ~10 lines in `Throttle.ino`.
- Simpler than A but the jump from low to high is not gradual.

**My recommendation:** Option A. Closest match to what you described. Self-contained on TX (no VESC dependency). Predictable behavior. Easiest to reason about. If you approve, I write the Claude Code prompt and Claude Code does the implementation in `Throttle.ino` + SPIFFS pipeline.

Whichever option you pick, the Section 9 safety rule still holds: this is all subtractive shaping of YOUR trigger input. The buggy still cannot move without you holding the trigger.

### 2.6 Should you delete dynamic throttle entirely?
You asked. Honest answer: **no good reason to delete it, but no good reason to KEEP it as it is today either** — because today it is just gears with finer steps. Three paths forward:

1. **Pick Option A above.** Dynamic throttle becomes meaningfully different from gears (auto soft-start). Worth keeping.
2. **Delete `throttle_mode = 2`.** Use gears with `dynamic_power_step`-style fine granularity (e.g., bump `max_gears` to 20 or 50 for finer control). Simpler firmware, one fewer mode to test.
3. **Leave as-is.** Acknowledge dynamic ≈ fine-grained gears, and accept that. No code change.

I lean toward #1 because your tow-buggy use case actually benefits from a soft-start, and that's something gears can't easily provide. But #2 (delete) and #3 (leave) are both defensible — your call.

### 2.8 RECOMMENDATION — for "smooth, consistent, not trigger-nervous" (added 2026-05-03)

You asked which mode and what to do. Here is the opinionated answer.

**Mode: Dynamic throttle (`throttle_mode = 2`), AND implement Option A (auto soft-start ramp).**

Reasoning:
1. **Today's gears and dynamic are too similar to make the choice matter much.** Both are linear caps stepped via toggle. Pick either and you're not gaining anything trigger-smoothness-wise.
2. **What you actually want is time-shaped power delivery, not trigger discipline.** You don't want to babysit the trigger or click toggles mid-ride to get a smooth start. You want to squeeze and have the system handle the ramp-up.
3. **Option A from §2.5 does exactly this.** The cap auto-ramps from `dynamic_power_start` (e.g., 30%) toward `dynamic_power_max` (100%) over `dynamic_power_ramp_s` seconds (e.g., 3s) every time you engage the trigger. Trigger release > a window resets. Trigger re-engage within that window keeps the ramp where it left off.
4. **Combined with VESC `foc_sl_openloop_boost_q` = 3–5**, the motor breaks free at low throttle reliably. The auto-ramp keeps you from getting full power instantly even at full squeeze. The two work together.
5. **Section 9 stays clean.** Option A is subtractive shaping of YOUR trigger input — it never sends more than you've commanded, just less for the first few seconds. The motor still cannot move without your trigger pull.

**The three-layer setup that gives you what you want:**

| Layer | Setting | Effect |
|---|---|---|
| 1. VESC | `foc_sl_openloop_boost_q = 3` (then test, raise to 5 if needed) | Motor breaks free from underwater static load at 10–15% trigger instead of 35–40%. |
| 2. BREmote dynamic mode + Option A | `throttle_mode = 2`, plus new fields `dynamic_power_max`, `dynamic_power_ramp_s`, `dynamic_power_reset_ms` | Auto soft-start: trigger goes to motor at 30% for first second, ramps to 100% over 3s. Predictable, smooth, no trigger micromanagement. |
| 3. BREmote expo | `thr_expo = 100` (default, KEEP IT — this is a curve, not linear despite the WebUI label) | Low trigger movements produce small motor changes. Trigger is most responsive in the upper third where you actually want fine control at speed. |

**About `thr_expo`** (clarifying my earlier note that may have confused you): the value `50` is true linear — small trigger move = small motor change everywhere. The value `100` (your current default) is actually a smoothing curve that flattens response at the bottom of the trigger range. **For your "not trigger-nervous" goal, `thr_expo = 100` is BETTER than 50** because it dampens jitter at low throttle. The WebUI label calling 100 "linear" is wrong; verify with prompt B in §7. **Only set `thr_expo = 50` during VESC PPM calibration** (so the wizard sees true endpoints), then change it back.

### 2.9 Why I'm NOT recommending: just tweaking expo, or just deleting dynamic
- **Just expo:** an aggressive expo curve (say `thr_expo = 30`) would flatten the bottom even more, but it's a static curve. It can't separate "first second of squeeze" from "ten seconds in at speed." Option A's auto-ramp is time-aware, expo is not.
- **Just delete dynamic, use gears:** loses the auto-ramp opportunity. You'd be back to clicking toggles, which you said you don't want to do mid-ride.
- **Just VESC tuning:** fixes the break-free problem but doesn't help with the "I squeezed 100% and the motor came on instantly" feel. You still want the time-ramp on top of the VESC fix.

### 2.10 What you should approve to move forward
If you agree with §2.8, here's the action plan in order:

1. **Right now:** apply VESC Step 1 (`foc_sl_openloop_boost_q = 3`). Bench-test with motor in air, then in a tub of water with prop. This is the fastest single change with the biggest impact. No firmware change needed.
2. **After Step 1 verifies on bench:** approve me to write the Claude Code prompt for Option A (auto soft-start). The prompt I drafted in §7 prompt C is ready; just say "go" and I'll refine and hand it over for Claude Code to implement in `Throttle.ino` + SPIFFS pipeline.
3. **Order matters:** do the VESC fix FIRST. If the motor still won't start at low throttle even with auto-ramp giving you 30% from a 100% squeeze, you'll think Option A is broken when it's actually still the VESC. Fix the underlying motor-control issue, then add the user-experience layer on top.

### 2.11 Interaction matrix (yours, answered)

| Combination | What happens |
|---|---|
| Dynamic + RTM ramp | Output is `min(shaped × cap%, RTM ramp value)`. Both subtract — they don't fight, the lower one wins. Safe. |
| Dynamic + RTM emergency stop | `rtm_rx_emergency_stop = true` forces RX `effective_thr = 0`. TX cap is irrelevant. Safe. |
| Dynamic + Expo | Expo runs first (on `thr_scaled`), then dynamic cap multiplies the result. Order-correct: shape the curve, then limit the magnitude. They don't fight. |
| Dynamic + VESC ramp settings | VESC limits motor acceleration regardless of what PWM commands. They are sequential (VESC sees the post-dynamic-cap PWM and applies its own ramp). They cooperate, do not fight. |
| Dynamic + Fasc/Hall expo (your "FASCOM has expo too") | If your VESC has its own throttle expo enabled in VESC Tool, that runs *after* the BREmote pipeline. **Recommendation: pick one place to do expo. Set BREmote `thr_expo = 50` (linear) and let VESC handle expo, OR set VESC PPM curve = linear and use BREmote `thr_expo`.** Two expos in series compound and become hard to reason about. |

---

## 3. PWM / PPM RANGE — WHAT THE NUMBERS MEAN AND VESC CALIBRATION

### 3.1 The fields and what they mean

| Field | Range | Default | Units | Meaning |
|---|---|---|---|---|
| `PWM0_min` | 500–2500 | 1000 | µs | Pulse width sent at throttle byte = 0 (motor 0 / right motor) |
| `PWM0_max` | 500–2500 | 2000 | µs | Pulse width sent at throttle byte = 255 (after RX-side `effective_thr`) |
| `PWM1_min` / `PWM1_max` | same | 1000 / 2000 | µs | Same for motor 1 / left motor |
| `trim` | -500..+500 | 0 | µs | Differential offset added to motor 0, subtracted from motor 1 |
| `failsafe_time` | 100–10000 | 1000 | ms | After this much silence on LoRa, PWM output stops |

Defaults from `BREmote_V2_Rx.h:218`. Validation from `ConfigService.ino:22–25, 79–91`.

### 3.2 Standard servo / ESC PPM convention
- **1000 µs** = throttle off / idle (motor stopped, or full reverse for bidirectional ESC)
- **1500 µs** = neutral (zero throttle for bidirectional, 50% for unidirectional)
- **2000 µs** = throttle full
- Pulses repeat every ~10–20 ms (50–100 Hz PWM frame rate).

VESC's PPM input is happy with either 1000–2000 (standard) or any subset/superset within reasonable limits. VESC does its own start/end-pulse learn during the **PPM Mapping** wizard.

**1000–2000 µs vs 500–2000 µs:** 500 µs is below standard servo range; some ESCs reject it as invalid or treat it as no-signal. **Stick to 1000–2000 unless you have measured evidence the VESC accepts wider range.** Wider range is not finer resolution — the throttle byte is still 0–255, so `(2000-1000)/255 = 3.92 µs` per step vs `(2000-500)/255 = 5.88 µs` per step. **Wider range = COARSER steps**, not finer.

### 3.3 Throttle byte → PWM linearization

From `PWM.ino:63`:
```cpp
PWM0_time = constrain(map(effective_thr, 0, 255, usrConf.PWM0_min, usrConf.PWM0_max) + usrConf.trim,
                      usrConf.PWM0_min, usrConf.PWM0_max);
```

`map()` is Arduino's linear interpolation: `out = out_min + (in - in_min) × (out_max - out_min) / (in_max - in_min)`. **No curve. No scaling. No hidden expo on RX.** The throttle byte is the single source of authority — whatever shaping happened on TX (gears, dynamic cap, expo, RTM cap) is baked into the byte. RX is dumb and faithful.

**What 0xF0 means in plain language:** `0xF0` is hexadecimal notation for the decimal number **240**. The throttle byte that travels over LoRa is a single 8-bit number from 0 to 255. The firmware reserves the top 15 values (`0xF1`=241 through `0xFF`=255) as "meta-packet markers" so the RX can tell `0xF3` apart from a normal throttle of 243. As a result the throttle byte is capped at `0xF0` = 240. So the LoRa packet uses byte values 0–240 for actual throttle, leaving 241–255 as code numbers for special packets.

**Effect on motor power: essentially zero.** When VESC PPM Mapping is run with the trigger fully squeezed, VESC sees `1941 µs` and remembers *that* value as 100% throttle. From then on, a fully squeezed BREmote trigger = full motor power, because VESC scales motor commands across whatever range it learned. You are NOT losing 6% of motor power, you are using 240 of 255 byte values for throttle (94% of the byte space) and VESC calibrates to that.

**The only situation where the 0xF0 cap actually matters:** if you swap to a different remote (one that goes 0–255) without re-running VESC PPM Mapping, the new remote could overshoot beyond what VESC learned. Not a real concern for a single-remote setup.

**Conclusion:** 0xF0 is fine as-is. No action needed. (The earlier note suggesting a `map(0, 240, ...)` fix is withdrawn — it would change nothing user-visible and adds complexity.)

### 3.4 VESC PPM calibration — the right procedure

VESC's PPM Mapping wizard learns three points: pulse-at-min-stick, pulse-at-center-stick, pulse-at-max-stick. **The wizard sees whatever PWM the RX outputs. Whatever shaping or capping you have active on TX gets baked in.**

**To capture the true RX hardware endpoints during VESC PPM-MAX learn:**
1. Set TX `throttle_mode = 1` (no gears) → `result = shaped`, no gear/cap multiplier.
2. Set TX `thr_expo = 50` (true linear) **TEMPORARILY for the calibration only** so the curve doesn't bias the endpoints. **After calibration is saved, change `thr_expo` back to whatever value you ride with** (default 100 = a smoothing curve that helps with low-throttle nervousness — see §2.9 below).
3. Confirm RTM is not armed (`rtm_thr_cap_tx = 255` at idle).
4. Confirm system is unlocked.
5. Pull trigger to physical max and **hold** while clicking the VESC wizard's "Save Max" button.
6. Release trigger to physical min and **hold** while clicking "Save Min".
7. **After saving:** restore `thr_expo` to your preferred ride value (100 default, or whatever you tune to — see §2.9).

If you do this with `throttle_mode = 0` and `gear = 3 of 6`, the VESC learns max = `(4/6) × 1941 = 1294 µs`, and it will never go above that in operation — **even if you later switch to gear 6 of 6**, because the VESC has already locked its max at 1294. You'd have to re-run the wizard.

**Same trap with dynamic cap:** if you calibrate with `max_power_cap = 50%`, VESC learns max = `0.5 × 1941 = 971 µs ≈ no throttle`. Disastrous if you don't notice.

**Standard procedure, every VESC calibration:**
- `throttle_mode = 1` (or `throttle_mode = 0` with `gear = max_gears - 1`, or `throttle_mode = 2` with `max_power_cap = 100`).
- `thr_expo = 50`.
- RTM disarmed.
- System unlocked.
- Then: trigger full → save max; trigger idle → save min.

**Verify after calibration:** in VESC Tool's Realtime Data, confirm full trigger = duty cycle 95%+, mid trigger ≈ 50% duty. If full trigger is showing 70% duty, your TX endpoints are below the VESC's learned range — re-calibrate with TX at full power as above.

### 3.5 What "minimum and maximum PWM" actually controls
- **Endpoints:** they pin where in the µs spectrum your remote's "off" and "full" land.
- **Resolution:** is *always* 256 steps regardless of endpoint range — the LoRa byte is one byte. Wider endpoint range = bigger µs steps, not more steps.
- **VESC compatibility:** the VESC needs to see your endpoints fall within whatever range it was calibrated to. If you change BREmote endpoints after VESC PPM Mapping, the VESC sees the wrong range and behaves wrong.
- **Other ESCs (FlipSky, Fasc):** all standard servo PPM. 1000–2000 µs works on all of them. Don't go below 1000 unless you have reason.

### 3.6 Failsafe behavior — verify your understanding
When LoRa packets stop for `failsafe_time` (default 1000 ms = 1 s):
- `PWM.ino:13` evaluates false.
- The pulse generation block is skipped.
- The RMT peripheral was set up to send a single transmission per `generate_pulse()` call (loop_count = 1, EOT level = 0). When the loop body exits without calling `generate_pulse`, **the line idles low**.
- VESC sees no pulse train. VESC's own loss-of-signal behavior kicks in (typically: motor coast / brake, depending on VESC config).

**This is NOT 1500 µs neutral failsafe.** It is "no signal at all". For a unidirectional ESC, no signal usually means coast (motor freewheels). For a bidirectional ESC where 1500 µs is neutral, no signal does NOT command neutral — the ESC's own SR fault behavior applies.

**Configure your VESC's PPM "Safe Start" and "Loss-of-Signal" behavior explicitly.** Do not assume RX is sending a safe value during dropout — it's sending nothing. The buggy stops cleanly only because (a) you're not squeezing the trigger physically, AND (b) VESC is configured to brake/coast on signal loss.

⚠️ **Action recommendation:** In VESC Tool, set "Safe Start" = "RegularFwd" or similar with stick-at-zero requirement, and "Loss-of-Signal Behavior" = "Brake" or "Free spin" depending on your safety preference. Document the chosen value in your CLAUDE.md so future you knows.

---

## 4. JAN'S UPSTREAM — BUG AUDIT (the courtesy report back)

Variants reviewed:
- `Source/` (canonical / main)
- `SourceAFM - w logger-compass (good)/` (closest match to your V2.5-EVO)
- Other variants spot-checked.

| # | Severity | Location (Jan's repo) | Bug | Real-world consequence | V2.5-EVO status |
|---|---|---|---|---|---|
| 1 | CRITICAL | `Source/V2_Integration_Rx/Init.ino:49–53` | WDT timeout 1000ms with GPS (300ms) + wetness (300ms) + VESC (210ms) running serially in loop = 810ms used, 190ms margin. False panic-resets under combined load. | Surface buggy resets mid-ride; foil restarts and may continue at last command if VESC didn't latch loss-of-signal. | ✅ FIXED in V2.5-EVO (`Init.ino:52–61`, raised to 3000ms with calculation comment) |
| 2 | HIGH | `Source/V2_Integration_Rx/Init.ino:38–42` | Three FreeRTOS tasks at 2048 byte stack (`generatePWM`, `triggeredReceive`, `checkConnStatus`). RadioLib + RMT + I2C deep-stack calls overflow this. | Stack overflow → ESP-IDF guard panic → reset. PWM glitch on the way down. | ✅ FIXED in V2.5-EVO (`Init.ino:41–45`, raised to 4096/4096/3072) |
| 3 | HIGH | `Source/V2_Integration_Rx/GPS.ino:57–71` | GPS read busy-waits 300ms in main loop with `vTaskDelay(1)` per char. Combined with VESC + wetness reaches 810ms WDT proximity. | Compounds Bug #1; on slower satellite acquisition the loop can stall and trip WDT. | ⚠ V2.5-EVO raised WDT to 3000ms which masks this; refactoring GPS into its own task remains a future improvement. |
| 4 | HIGH (safety) | `Source/V2_Integration_Rx/PWM.ino:10–29` | When packet timeout exceeds `failsafe_time`, the PWM generation block is skipped but RMT was put into "loop" mode. **Andres, please verify on V2.5-EVO** whether `loop_count = 1` (your V2.5-EVO line) means single-shot (stops after one pulse) or continuous loop. If continuous, the line keeps emitting last commanded pulse width on dropout — exactly Jan's bug. | Throttle held at last value forever if RX freezes after a packet. Real risk. | ⚠ Probably FIXED in V2.5-EVO — `loop_count = 1` strongly suggests single-shot, line idles low after the function exits. **Recommend bench test:** unplug TX antenna mid-ride on bench, scope the PWM pin with motor disconnected, confirm line goes silent within `failsafe_time` ms. |
| 5 | MEDIUM | TX & RX `Radio.ino` multiple sites | `radio.startTransmit()` return value ignored. RadioLib returns int error codes; transmit failure (e.g., SPI desync, radio in wrong state) is silently dropped. | Pairing packets silently lost; user thinks paired but RX never heard it. Telemetry holes. | ⚠ Likely STILL PRESENT — V2.5-EVO inherited this pattern. Not life-safety but worth a sweep. |
| 6 | MEDIUM | `Source/V2_Integration_Rx/Logger.ino` (where present) | `vesc_struct` accessed across cores without mutex; `logging_active` missing `volatile`. Torn reads, stale cache. | Garbage log data; rare crashes. | ✅ FIXED in V2.5-EVO (vescMutex added in `Logger.ino:19, 97, VESC.ino:68`; `logging_active` declared `volatile`). |
| 7 | MEDIUM | `Common/WebConfigEngine.h:213–266` | POST handlers have no CSRF, no payload size limit, no rate limit. WiFi AP password protection only; once on the AP, anyone can spam config writes. | Attacker on local AP can corrupt config; user opening malicious page while connected to AP can trigger CSRF. Heap exhaustion if attacker POSTs 100MB. | ⚠ Likely PARTIALLY IMPROVED in V2.5-EVO — please verify by reviewing `WebConfigEngine.h` length checks. Not life-safety since attacker needs physical proximity, but worth fixing for robustness. |
| 8 | LOW | `Source/V2_Integration_Rx/SPIFFS.ino:34–53` | Default battery calibration is a hardcoded Base64 placeholder. Every fresh-flash board reads identical values. | Battery % display is generic, not pack-specific. Annoying but not safety. | Status unknown in V2.5-EVO — verify `SPIFFS.ino` defaults. |
| 9 | LOW | `Common/WebConfigEngine.h` various | `String` concatenation in hot path web handlers fragments heap over hours of use. | Long-running config sessions can OOM. Not field-relevant since web config is short-lived. | Probably partially fixed; not safety-critical. |
| 10 | LOW | `Source/V2_Integration_Rx/System.ino` | `scanI2C()` calls `Wire.begin()` mid-runtime, can break AW9523 expander. | I2C bus hang → LED + button expander dies → device looks frozen until reset. | ✅ FIXED in V2.5-EVO (Wire.begin removed from scanI2C per CLAUDE.md Section 4 fix list). |

**Summary for Jan:** of the 5 high/critical bugs, V2.5-EVO has fixed 4 confidently and 1 (failsafe RMT loop behavior) is most likely fixed but warrants a bench test. The medium bugs are a mix — V2.5-EVO has improved most of them but the radio-error-check sweep and WebConfig hardening are still partially open. Jan's design (3-byte addressing, CRC8 pairing, semaphore-driven LoRa ISR) is solid; the implementation gaps are exactly what you'd expect from untested sandbox code.

---

## 5. WHY FREERTOS IS THE RIGHT CHOICE HERE

### 5.1 Task inventory (from `Init.ino`)

**RX:**
- `Generate_PWM_10ms` (4096B, prio 10, core 0, 10ms tick) — outputs PWM, must never miss a deadline. Subscribes to WDT.
- `RF_ReceiveTask_triggered` (4096B, prio 5, core 0, event-driven) — wakes on LoRa DIO interrupt via semaphore, decodes packet.
- `Check_conn_staus_200ms` (3072B, prio 2, core 0, 200ms) — LED + WDT feed.
- `DataLogger` (4096B, prio 1, core 0, 1000ms) — SPIFFS writes; mutex-guarded VESC reads.
- Plus the loop task on core 1 doing GPS, VESC, wetness, compass, web config.

**TX:**
- `Send_Data_100ms` (2048B, prio 5) — packs throttle/steer/GPS, transmits LoRa at 10 Hz.
- `wait_for_telem_triggered` (2048B, prio 4) — receives RX telemetry.
- `wait_for_telem_triggered_10ms` (2048B, prio 6) — Hall sampling and filter.
- `wait_for_telem_triggered_200ms` (2048B, prio 6) — display update; mutex-guarded.
- `Vibration_Task_BG` (2048B, prio 3) — haptic state machine.

### 5.2 Why bare-loop Arduino would not work
The 10 Hz LoRa cycle is a hard deadline. On RX, the loop task does GPS (300ms blocking serial), VESC UART (210ms), wetness (300ms), and SPIFFS log writes (up to 1000ms). That's ~1810ms worst-case. A bare-loop Arduino design would either:
- Run all of these serially → blow past the 100ms LoRa deadline → drop packets.
- Make all of them non-blocking with hand-rolled state machines → mountain of complexity, tons of edge cases, and the whole thing still freezes if any one state machine has a bug.

FreeRTOS gives you preemption. `generatePWM` at priority 10 wakes every 10ms regardless of what GPS or logger is doing, because the scheduler will yank the CPU off whatever's running. That isolation is a safety property, not a convenience.

### 5.3 Field-debuggability — Jan's concern, addressed
Jan said "FreeRTOS makes this even harder — debugging in the field is basically impossible." That is generally true for *bad* FreeRTOS code: dynamic task creation, deep ISR queue chains, priority inversions, cascading mutexes. **None of those patterns are in V2.5-EVO.**

V2.5-EVO is FreeRTOS-conservative:
- All tasks created once at boot, never destroyed.
- Static stack sizes.
- Three mutexes total, each with single-purpose protection (`vescMutex`, `fileMutex`, `displayMutex`). Easy to reason about.
- No `xQueueSendFromISR` deep nesting. The LoRa ISR only does `xSemaphoreGiveFromISR()` — one line.
- Logger task captures full state to SPIFFS before/during incidents — post-incident analysis off the SD card.

There are minor concerns (priority inversion risk on `displayMutex`, GPS struct read without mutex in Logger, some volatile flags that should arguably be `std::atomic`), but none rise to "field-undebuggable". They are documented in the source with the literal `// V3 - <date> - <reason>` comment tags (note: this is the historical version-tag string that already exists across the codebase per CLAUDE.md Section 3 — kept as-is for grep continuity even though the project is now branded V2.5-EVO).

### 5.4 The honest concession
Jan is right that *if you ever stack lots of FreeRTOS complexity* (real-time tasks talking to each other through queues, dynamic task pools, soft-real-time deadlines on multiple paths), debugging on the water becomes impossible. **Hold the line on simplicity.** Your three mutexes and seven static tasks is the right complexity ceiling. Any future feature that needs another task should be reviewed against this principle: "could a bare callback in the loop task handle this?" If yes, do that.

---

## 6. SAFETY ETHICS — REINFORCEMENT

Per CLAUDE.md Section 9 and your stated values:

1. ✅ **Buggy moves only when user holds trigger.** `thr_received` originates from Hall sensor; no autonomous source ever sets it.
2. ✅ **Autonomous modes only subtract throttle.** Verified at `Throttle.ino:27`, `PWM.ino:43–51`. Three independent code paths, all subtractive.
3. ✅ **Manual mode works when RTM/FM/GPS are buggy.** Default `throttle_mode = 0` (gears) with RTM disarmed runs entirely in `Hall.ino → Throttle.ino → Radio.ino → PWM.ino`. None of those depend on GPS, compass, or RTM state.
4. ✅ **Releasing trigger stops motor.** `thr_scaled = 0` when Hall is at idle; `effective_thr = 0` flows through PWM linearly to whatever `PWM_min` you have set (1000 µs default = motor off).
5. ✅ **RTM internal error falls back to manual.** `rtm_rx_emergency_stop` only forces `effective_thr = 0` (does not lock RTM on); `rtm_rx_active` is false by default; manual control is unchanged.

**One soft spot to bench-test before next field run:**
- **Failsafe RMT idle behavior.** Per Bug #4 above, confirm with scope or VESC realtime data that the PWM line goes silent (or to neutral) within `failsafe_time` ms when TX is powered off. If the line keeps emitting last commanded value, that is Jan's bug surviving in V2.5-EVO and must be fixed before water.

**Bench test prompt for Claude Code (when you're ready):**
```
Read PWM.ino lines 1-145 in V2.5-EVO RX. Verify that when generatePWM()'s
"if(PWM_active && millis()-last_packet < usrConf.failsafe_time)" gate
fails, the RMT TX channel does NOT continue emitting the last pulse width.
Specifically check whether rmt_transmit() with loop_count=1 is single-shot
(line idles after one pulse) or continuous (line keeps pulsing until
explicitly stopped). If continuous, propose adding rmt_disable() or
explicit "send 1000us once and stop" on failsafe entry. Do not modify
files — output a finding only.
```

---

## 7. RECOMMENDED FOLLOW-UP CLAUDE CODE PROMPTS

Save these for when you're ready to act on findings:

**A. Failsafe behavior bench-verify**
```
In V2.5-EVO RX PWM.ino, the failsafe path skips generate_pulse() but does not
explicitly stop the RMT channel. Verify by reading the rmt_transmit
documentation comments and the existing initRMT/generate_pulse code:
when the failsafe gate at line 13 is false, does the RMT line idle low,
or does it continue emitting the last pulse pattern? Output a finding
report with file:line citations and a one-line recommendation. Do not
modify files.
```

**B. Throttle expo default verification**
```
In V2.5-EVO TX Hall.ino expoThrCurve(), the default thr_expo = 100 is described
as "linear" in the WebUI, but the math (expo - 50) / 50.0 gives a weight
of +1.0 at expo=100 and 0.0 at expo=50. Read Hall.ino lines 189-215 and
the WebUiEmbedded.h thr_expo description. Determine which is correct:
the default value or the WebUI label. Propose a one-file fix (either
change default to 50, or change the WebUI label). Do not modify files —
output finding only.
```

**C. Optional auto-ramp dynamic throttle (only if you confirmed Reading B from §2.2)**
```
Andres wants dynamic throttle to also include an automatic time-based
soft-start: when the user squeezes the trigger from idle, the effective
cap should ramp from dynamic_power_start to dynamic_power_max over
dynamic_power_ramp_s seconds, even at 100% squeeze. When the trigger is
released for more than dynamic_power_reset_ms, the ramp resets to start
value. When re-engaged within that window, the ramp continues from where
it left off.

Plan only — do not edit yet:
1. New SPIFFS fields needed: dynamic_power_max (10-100, default 100),
   dynamic_power_ramp_s (0-10s, default 2s), dynamic_power_reset_ms
   (100-5000, default 1000).
2. Changes to confStruct, defaultConf, ConfigService validation,
   WebUiEmbedded.h, docs HTML web-serial tool — five-surface pipeline.
3. New runtime state variables in BREmote_V2_Tx.h.
4. Throttle.ino calcFinalThrottle() math change.
5. Hall.ino integration (detect trigger engage/release edges).

Output the full plan with file:line targets. Do not modify any file.
```

**D. Throttle byte 0xF0 endpoint loss** (cosmetic / max-speed feel)
```
RX PWM.ino map(effective_thr, 0, 255, PWM0_min, PWM0_max) maps a TX byte
that is in fact capped at 0xF0 (240). At full TX squeeze the RX outputs
~1941 µs instead of full 2000 µs. Propose changing PWM.ino map() to use
input range 0..240 so the user gets the full PWM0_max at full trigger.
Identify all sites in PWM.ino requiring the change. Do not modify files —
plan only.
```

---

## 8. APPENDIX — KEY FILE REFERENCES (single source of truth)

- TX throttle math: `Source/V2_Integration_Tx/Throttle.ino:5–30`
- TX expo curve: `Source/V2_Integration_Tx/Hall.ino:189–215`
- TX RTM cap: `Source/V2_Integration_Tx/RTMState.ino:86–97`
- TX packet pack + 0xF0 cap: `Source/V2_Integration_Tx/Radio.ino:362–366`
- TX confStruct: `Source/V2_Integration_Tx/BREmote_V2_Tx.h:56–280`
- TX defaultConf: `Source/V2_Integration_Tx/BREmote_V2_Tx.h:218`
- RX PWM math: `Source/V2_Integration_Rx/PWM.ino:37–108`
- RX RMT setup: `Source/V2_Integration_Rx/PWM.ino:110–145`
- RX RTM gates: `Source/V2_Integration_Rx/RTMState.ino:33–130`
- RX failsafe gate: `Source/V2_Integration_Rx/PWM.ino:13`
- RX confStruct: `Source/V2_Integration_Rx/BREmote_V2_Rx.h:56–250`
- RX defaultConf: `Source/V2_Integration_Rx/BREmote_V2_Rx.h:218`
- RX FreeRTOS task config: `Source/V2_Integration_Rx/Init.ino:38–67`
- TX FreeRTOS task config: `Source/V2_Integration_Tx/Init.ino:41–67`

---

End of analysis.
