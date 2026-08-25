# Follow-Me (FM) — Rider's Guide

**BREmote V2.5-Evo** · tow buggy / eFoil remote

> **Status:** the Follow-Me control law is implemented and pending on-water field testing.
> This guide documents the intended behavior. It moves into the main README once FM is
> confirmed on the water.

---

## 1. What Follow-Me does

After the whip — once you've released the rope and you're riding the wave — Follow-Me
makes the **buggy trail you** at a set distance and angle, steering itself, so you can
keep your eyes on the wave instead of the remote.

It is **not** an autopilot that drives on its own. The rule never changes:

> **The buggy moves only while you hold the throttle trigger. Follow-Me only *steers* and
> only *reduces* throttle — it can never add throttle, and letting go of the trigger stops
> the buggy instantly.**

Follow-Me is autonomous *steering under your throttle hand*. You stay in charge of go/stop
at all times.

---

## 2. Before you ride

Follow-Me needs, for the buggy to actually engage and follow:

- A **paired** TX and RX (your remote and the buggy talk to each other).
- **GPS fix on both** the remote and the buggy.
- A healthy **radio link** and **telemetry flowing**.
- A **calibrated compass** on the buggy (`?compasscal`).

You can *arm* Follow-Me before all of these are perfect (see §7 — Readiness), but it will
not *engage* until they are.

---

## 3. Arming — telling the buggy "I want you to follow"

Arming is a **declaration of intent.** It does not move anything. There are two ways to arm.

### 3a. The toggle (works while floating)

While floating, before takeoff:

1. **LEFT tap, then RIGHT hold** (~3 s — set by `fm_hold_duration_s`).
2. The remote shows **F1 / F2 / F3 / F4** and buzzes **two quick taps** = armed.
3. Cycle the mode by repeating the gesture before you touch the throttle
   (**F1 → F2 → F3 → F4 → F0-off**).

> The toggle **cannot arm once you're on the throttle** — while you hold the trigger, the
> toggle *is* your steering. That's what the magnet is for.

### 3b. The magnet (works during the tow)

If your remote has the Hall sensor fitted and `mag_mode` is set:

- **Hold the magnet to the sensor for ~2 s, then pull it away.** You feel one short pulse
  at 2 s (get-ready), then **two quick taps** on release = armed.
- It's a **toggle**: armed already → the same gesture **disarms** (one long buzz).
- **It works with the throttle held** — so if you forgot to arm and you're already towing,
  you can arm mid-tow. This is the magnet's whole purpose.

**The arm survives** for up to `fm_arm_window_s` (default **180 s / 3 min**) even if you
don't touch the throttle — so a slow takeoff, a wait, or a short swim won't lose your arm.

---

## 4. The whip — how Follow-Me engages

Arming is not engaging. Follow-Me only **engages** (starts following) when the **geometry
confirms you've actually separated** — not on a timer, not on a button:

1. You're armed (from §3), throttle held.
2. You whip and separate from the buggy.
3. Once you are **beyond the engage distance for 2 continuous seconds** — proven by both
   GPS units — the buggy begins following at your set offset.

You keep the throttle held the whole time. No release step is needed; the buggy engages
*while* your throttle is held (and releasing would just stop it).

### Two ways to separate at the whip

- **Whip yourself to the side** (classic) — you slingshot past the buggy and ride away.
- **Steer the buggy away** — keep the throttle and steer the buggy to its offset side so
  *it* peels off while you carry straight into the wave. Your steering during this moment
  does **not** cancel Follow-Me (you're still armed, not yet following). Once you settle and
  the separation holds for 2 s, it engages.

---

## 5. Where the buggy follows — the offset angle

Four modes set where the buggy stations, **relative to you** (imagine you facing
your direction of travel):

| Mode | Name | Buggy sits |
|---|---|---|
| **F1** | **Near-Right** | behind and to your right |
| **F2** | **Behind** (default) | straight behind you |
| **F3** | **Near-Left** | behind and to your left |
| **F4** | **In Front** | ahead of you as a forward pacer |

F4 never performs an autonomous overtake. Position the buggy ahead manually first. It engages only
after the along-course lead exceeds `fm_engage_dist_m`, stays inside the configured front cone and
remains proven for 2 seconds. If it ceases to be provably ahead, it stops in HOLD, clears the proof
and waits for the trigger to be released for 2 seconds before returning full manual control.

For F1/F3, the exact angle is set by **`near_diag_offset_deg`** — the number of degrees **off
straight-behind**. Near-Right and Near-Left are mirror images of it; F4 does not use this offset:

- **Near-Right = 180° − offset** · **Near-Left = 180° + offset**
- **Bigger offset → more beside you. Smaller offset → more behind you.**

| Offset | Near-Right | Near-Left | Feel |
|---|---|---|---|
| 0° | 180° | 180° | straight behind |
| **45° (current)** | **135°** | **225°** | behind-diagonal |
| 60° | 120° | 240° | tucked behind, slightly out |
| 90° | 90° | 270° | directly beside you |

*(0° = ahead of you, 90° = your right, 180° = straight behind, 270° = your left.)*

Follow distance is a **separate** setting — see §8.

---

## 6. Holds vs Stops — what pauses Follow-Me and what ends it

Not every interruption is the same. Follow-Me tells them apart:

| Situation | What it is | Follow-Me does | Re-arm needed? |
|---|---|---|---|
| **You release the trigger** | deadman | motor stops instantly | no |
| **You fall / slow down / get too close** | a **HOLD** (normal) | pauses (buggy stops), **stays armed**, resumes on its own at your next separation | **no** |
| **GPS, compass, or radio drops out** | a **FAULT** (something broke) | **stops** → shows `St`, throttle returns, must **re-arm** | **yes** |

The idea: **falling is normal, so it stays ready.** A real failure is not, so it steps fully
out and waits for you to deliberately re-arm — autonomy never silently restarts after a
glitch.

The stop alarm (`St` + one long buzz) fires **only when it would surprise you** — i.e. a
fault while you're holding the trigger. A stop after you've already let go just goes quiet.

---

## 7. Reading the remote

| Display | Meaning |
|---|---|
| **No FM bar** | disarmed |
| **Bar sweeping** (scanner) | **armed and ready** — waiting for your whip |
| **Bar blinking in place** | **armed but not ready yet** — still getting GPS / link. It flips to sweeping the moment it's ready. |
| **Steady distance bar** | **following you** — grows/shrinks with distance |
| **`St`** | stopped |

**Haptics (by feel, no need to look):**

| Buzz | Means |
|---|---|
| two quick taps | armed |
| **one long buzz** | stopped / disarmed |
| one short pulse | magnet get-ready (release now for FM) |
| three taps | magnet get-ready (release now for RTM) |

---

## 8. Settings (RX, unless noted)

| Setting | What it does | Note |
|---|---|---|
| `followme_mode` | geometry: 1 = Near-Right, **2 = Behind (default)**, 3 = Near-Left, 4 = In Front | TX seed for the arm gesture |
| `near_diag_offset_deg` | angle off straight-behind (see §5) | **45** = Right 135° / Left 225° |
| `min_dist_m` | hard-stop distance — throttle cut to 0 if the buggy gets this close | |
| `followme_smoothing_band_m` | decel band above the hard stop | follow distance = `min_dist_m` + this |
| `boogie_vmax_in_followme_kmh` | absolute speed ceiling while following | 0 = no absolute ceiling, also for F4; F4 still regulates its front gap relative to rider speed |
| `foiler_low_speed_kmh` | below this rider speed, Follow-Me holds (won't maneuver around a swimmer) | |
| `fm_arm_window_s` *(TX)* | how long an arm survives with no throttle | **180 s** |
| `mag_mode` *(TX)* | magnet gesture role: 0 off, 1 = FM, 2 = RTM, 3 = FM+RTM | needs the Hall sensor |
| `fm_display_mode` *(TX)* | what the digit zone shows while armed | 2 = distance to buggy |

> **Tuning note:** the follow distance is currently set generous (`min_dist_m` + band
> larger than the final target). Tighten it only after the separation interlock is confirmed
> on the water — the generous setting is part of what keeps Follow-Me from engaging while
> you're still on the tow rope.

---

## 9. Disarming

- **Toggle:** repeat the arm gesture.
- **Magnet:** hold ~2 s again (one long buzz = off).
- **Automatic:** arming RTM disarms FM; the arm expires after `fm_arm_window_s` with no
  throttle; a fault ends it.

---

## 10. Not yet — planned for later

- **Steer-adjust while following** — nudge the buggy's position with the toggle *without*
  canceling Follow-Me (today, steering while following cancels it for safety). Rides the
  existing steer-blend setting; enabled after field testing.
- **Continuous angle repositioning** — steer to walk the buggy around your radius to any
  angle, live. Reserved as a v2 feature (`fm_steer_reposition_en`), off by default.

---

## Safety summary (never changes)

1. The buggy moves only while you hold the throttle trigger.
2. Follow-Me only steers and only reduces throttle — never adds.
3. Releasing the trigger stops the buggy at the hardware level, in every mode.
4. Manual control always works, even if GPS / compass / radio fail.
5. Follow-Me and Return-to-Me are mutually exclusive — arming one disarms the other.

*See `BUGGY_FOIL_DOMAIN.md` for the domain model and `DESIGN_FOLLOW_ME.md` for the full
engineering design.*
