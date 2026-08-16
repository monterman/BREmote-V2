# Zero → Foiling

**BREmote V2.5-Evo — from an unflashed board to Follow-Me on the water.**

One linear path. Follow it in order; each phase assumes the one before it passed.

The public firmware ships **fully unbound and unconfigured** — not paired, neutral compass
calibration, logging off. Nothing personal ships in the image. You set it up yourself from the
serial console and the on-device web portal.

Steps are tagged **[DESK]** · **[BENCH]** · **[WEB PORTAL]** · **[WATER]**. Anything marked
*(tweak-later-OK)* is safe to leave alone until after your first sessions.

---

## THE SAFETY MODEL — read first, it never changes

1. **The buggy moves only while you physically hold the throttle trigger.** No throttle input = no motor, ever.
2. **Follow-Me and Return-to-Me can only *steer* and only *subtract* throttle — never add it.** There is no loiter, no station-keeping, no self-driving.
3. **Releasing the trigger stops the buggy instantly, at the hardware level, in every mode.**
4. **Manual control always works**, even if GPS / compass / radio fail.
5. **GPS distance is an ASSIST, not the authority.** At close range GPS can read ~15 m off. Trust your eyes (line-of-sight), not the number. This is why the follow distance ships deliberately generous.
6. **FM and RTM are mutually exclusive** — arming one disarms the other.

> The single most dangerous mistake is a **wrong steering sign**: it makes Follow-Me steer the buggy *toward* you instead of trailing you (closed-loop runaway). The wheels-up steering check in the BENCH phase is the top pre-water gate — do not skip it.

---

## Phase 1 — AT THE DESK (before you touch hardware) — [DESK]

1. **Install VESC Tool** (desktop + the free VESC Tool mobile app for BLE). You'll use it for PPM input calibration and motor detection.
2. **Get the config tools:**
   - The **on-device web portal** (served by each board over its own WiFi AP) — this is the primary, always-current config surface.
   - The standalone **[Web Serial Config Tool](BREmote_V2.5-Evo_Web_Serial_Config_Tool.html)** — a
     single self-contained HTML file that edits every setting over USB serial or over WiFi. Two ways
     to use it:
     1. **Open it from the repo on GitHub** (view raw / download), or
     2. **Keep your own copy** — this is the one to do. `git clone` the repo, or just download that
        one `.html` file somewhere you can find it.

     💡 **Download it before you go anywhere.** It is a single file with no internet dependency, so a
     local copy works at the dock, at the beach, in a car park — anywhere you have a laptop and a USB
     cable but no signal. That is exactly when you need it.
3. **Get a flashing tool — you do not need Arduino.** Prebuilt `.bin` files are published for both
   boards; flashing one takes a minute and is far less error-prone than compiling.
   - ⭐ **[Flash Download Tool](https://www.espressif.com/en/support/download/other-tools)** —
     Espressif's Windows GUI (also called the ESP Download Tool). **This is the recommended way**,
     and the one Ludwig demonstrates. Unzip, pick the `.bin`, press START.
     **[Step-by-step guide →](FLASHING_WITH_DOWNLOAD_TOOL.md)** · 🎥 **[Ludwig's walkthrough from 40:00 →](https://youtu.be/r6JIZEq3aTU?t=2400)**
   - **esptool** — same job from a command line on any OS, if you prefer:
     `esptool --chip esp32c3 --port COM<N> write-flash 0x10000 <file>.bin`

   > **You do not need Arduino, and you should not start there.** Compiling from source depends on
   > your library versions and board settings being exactly right, and getting one partition setting
   > wrong on the RX wipes its config. The published `.bin` is the image that was actually built and
   > tested. Compile only if you are changing the firmware —
   > [RX](FLASHING_RX_ARDUINO.md) · [TX](FLASHING_TX_ARDUINO.md), both advanced.
4. **Read, in this order:** this guide → [`docs/FOLLOW_ME_GUIDE.md`](FOLLOW_ME_GUIDE.md) (rider flow) → [`BUGGY_FOIL_DOMAIN.md`](../BUGGY_FOIL_DOMAIN.md) (safety model) → [`docs/VESC_SMOOTH_START_QUICK_REFERENCE.md`](VESC_SMOOTH_START_QUICK_REFERENCE.md) (PPM and smooth-start).

---

## Phase 2 — AT THE BENCH (hardware, wheels-up, motor-safe) — [BENCH]

### 2.1 Flash both boards — prebuilt `.bin`, no compiling

> ## 🎥 Watch it done first
>
> ### **[Ludwig's flashing walkthrough → starts at 40:00](https://youtu.be/r6JIZEq3aTU?t=2400)**
>
> He flashes a BREmote with the **Flash Download Tool**, step by step. **Nothing about the process
> has changed** — the only difference is that you pick **your** `.bin` from the table below instead
> of his.
>
> Watch that once and this whole section will make sense. Then follow along here for the exact
> files, the address, and the settings.
>
> **[→ Written step-by-step, every setting](FLASHING_WITH_DOWNLOAD_TOOL.md)**

**Download the current build for each board and flash it at offset `0x10000`.**

| Board | File | Version |
|---|---|---|
| **TX** | [`BREmote-TX-SW27-ubx-checksum.bin`](../Source/V2_Integration_Tx/TX%20firmware/BREmote-TX-SW27-ubx-checksum.bin) | **SW27** — current |
| **RX** | [`BREmote-RX-SW34-dyn-model.bin`](../Source/V2_Integration_Rx/RX%20firmware/BREmote-RX-SW34-dyn-model.bin) | **SW34** — current |

Every published build, with notes on each: **[TX firmware →](../Source/V2_Integration_Tx/TX%20firmware/)** ·
**[RX firmware →](../Source/V2_Integration_Rx/RX%20firmware/)**. Older builds are kept for rollback —
**use the two in the table above.**

> ### 🚨 Download the RAW file — this is where people go wrong
>
> **If you save the GitHub page instead of the file, you get an HTML web page with a `.bin` name.**
> It is not firmware. It will not flash, or it will fail in a way that looks like a dead board.
>
> Two ways to get the real bytes:
>
> **1. From the GitHub page** — click the `.bin` file to open it, then use the **Download raw file**
> button (the ⤓ download icon, top-right of the file view). GitHub cannot preview a binary, so the
> page shows *"View raw"* or a download button — **that button is the one you want.** Do not use
> your browser's *File → Save Page As*.
>
> **2. Clone the whole repo** — simplest, and you get every build plus the config tool at once:
> ```
> git clone https://github.com/monterman/BREmote-V2.git
> ```
>
> **Sanity check before flashing:** the RX and TX `.bin` files are a few hundred KB. If the file you
> downloaded is only a few KB, or opens as a web page in a text editor, you saved the HTML — go back
> and use the raw download.

⚠️ **Flash at `0x10000`, not `0x0`**, and ignore any reference to `.merged.bin` — merged images are
deliberately not published; flashing one at `0x0` wipes your SPIFFS config.

**Before you pair, confirm which board is which.** Identify each by its MAC
(`esptool --chip esp32c3 --port COM<N> read-mac`) rather than by COM number, so you cannot
cross-flash TX firmware onto the RX board.

**What you should see on first boot:** the TX shows `EP` (not paired), forces throttle and toggle
calibration (`cal_ok=0`), the compass reads neutral (scales 1.0, offsets 0), and logging is off
(`logger_en=0`). That is the sanitized image behaving correctly — not a fault.

<details>
<summary><b>Only if you compile it yourself — partition schemes</b></summary>

Skip this entirely if you are flashing the prebuilt `.bin` above.

- **TX** uses the **`huge_app`** partition scheme — see [Flashing the TX](FLASHING_TX_ARDUINO.md).
- **RX** has its **own `partitions.csv`** in the sketch folder. **Do not pass any PartitionScheme
  override for the RX** — it clobbers the custom layout, wipes config and halves log space. See
  [Flashing the RX](FLASHING_RX_ARDUINO.md).
- The RX flashes over USB; OTA was intentionally dropped.

</details>

### 2.2 First boot & TX calibration
- Power the TX. It will force **throttle + toggle calibration** (because `cal_ok=0`). Alternatively force it any time: **hold LEFT toggle at boot** → calibration. Follow the on-screen prompts to set throttle rest/pull and toggle left/centre/right endpoints.
- *(Note: LEFT-at-boot = calibration, RIGHT-at-boot = pairing. Some older docs have this backwards — the firmware is RIGHT=pair, LEFT=calibrate.)*

### 2.3 Pair TX ↔ RX (BIND at boot)
- **On the TX: hold RIGHT toggle at boot.** **On the RX: hold the BIND button at boot** — do both simultaneously. They exchange addresses and set `paired=1` on each.
- ⚠️ **WiFi turns off automatically once TX and RX are paired and in range** (LoRa wins over WiFi). To use either board's web portal later, **power the other board off** first, or the link forms and WiFi shuts down.
- Factory wipe if you ever need it: **hold BIND + AUX together at boot** on the RX.

### 2.4 Compass calibration (RX) — BIND short-press, two full 360° circles

The compass lives on the RX, in the buggy. With the RX running, **short-press BIND** — or send
`?compasscal` over serial — to start the **45-second** calibration.

**What you have to do: turn the buggy through TWO COMPLETE 360° CIRCLES.**

Not a partial turn, not a wiggle — **two full rotations**, all the way around, twice. Take your time;
you have 45 seconds, and slow and steady calibrates better than fast.

Any of these works, whichever suits where you are:

- **Floating flat on the water** — rotate it through 360°, twice.
- **Flat on the ground** — spin it in place through 360°, twice.
- **Carried in your hands** — hold it level and turn yourself all the way around, twice.

Keep the buggy **level** throughout, and keep it away from anything metal — cars, rebar in concrete,
tool benches, your own phone.

**LED feedback — one meaning per line:**

| Blinks | Meaning |
|---|---|
| **5 blinks** | **Started** — calibration is running, begin turning now |
| **2 blinks** | **Success** — calibration captured and saved |
| **10 blinks** | **No compass detected** — nothing was calibrated |

If you get **10 blinks**, the RX cannot see a magnetometer. Check that `gps_chip_type` is set to
**1** or **3** — those are the compass-equipped modules. A BN-220 has no compass and will always give
you 10 blinks.

On success the result saves itself to SPIFFS (`mag_offset_x/y`, `mag_scale_x/y`) — there is nothing
else to press.

### 2.5 ESC setup — power the PPM inputs, then calibrate

> ### 🔧 You do not need a VESC
>
> The BREmote RX outputs a **standard PPM (servo-style) signal**, so it drives **any ESC that accepts
> PPM** — VESC, and plenty of non-VESC controllers too. This guide is written around a **dual-VESC**
> buggy because that is the reference build and VESC Tool makes the calibration visible.
>
> - **Running VESCs?** Follow 2.5a and 2.5b as written.
> - **Running a different ESC?** **2.5a still applies** — the PPM signal comes out through an
>   optocoupler that needs 5 V on the connector no matter what is on the other end. Do that part,
>   then set your throttle endpoints and (on a twin-motor build) match the two controllers **using
>   your own ESC's tooling**. Then rejoin the guide at **2.6**.
>
> The requirements are the same whatever you run: both motors must start together and top out
> together, and there must be no idle creep at rest.


Two separate jobs, in this order. **Powering comes first** — a PPM input that is not powered cannot
be calibrated, and the way it fails is deeply misleading.

#### 2.5a Power — does your ESC put 5 V on the PPM connector?

Each PPM signal leaves the RX through an **optocoupler** (`IC5` for channel 0, `IC3` for channel 1).
**An optocoupler can only pull the line *down*.** To produce a high it needs a pull-up, and that
pull-up is powered by the **5 V pin on that channel's connector** — `ESC0_5V` on `JP1.2`,
`ESC1_5V` on `JP4.2`.

**No 5 V on that pin → that PPM line can never go high → nothing reaches that VESC.**

Most VESCs and ESCs already supply 5 V on their PPM/servo connector. **Check yours with a meter
before wiring.** If it does, connect it and you are done. If it does not, you must bring 5 V to that
pin from that controller's own BEC.

> ### ⚠️ The failure that costs people a weekend
>
> The two channels are **not symmetrical**, and this is the whole trap:
>
> - **`ESC0_5V` also powers the RX board itself** (through `D8`). Lose it and the RX will not boot —
>   impossible to miss, you diagnose it in a minute.
> - **`ESC1_5V` powers nothing but the opto pull-up.** Lose it and **everything looks perfectly
>   normal** — the RX boots, the display works, the radio links, telemetry flows, motor 1 runs.
>   The *only* symptom is that **motor 2 never moves**, and VESC 2's PPM input bar sits flat at
>   **zero** rather than reading weak or noisy.
>
> A flat-zero PPM bar on one VESC that survives re-wiring and re-calibration is **almost always this**,
> not a mapping problem. Check for 5 V on that connector's pin 2 before you touch the calibration.

**Connector pinout — read from the netlist, not from memory:**

```
JP1 / JP4:   pin 1 = SIGNAL      pin 2 = 5 V      pin 3 = GND
```

**Do not bridge 5 V from PPM1 across to PPM2.** It looks like a tidy shortcut and it is not: it
parallels two BEC regulators with no diode between them. That arrangement has failed **twice** in
testing. Take 5 V for each channel from **that channel's own controller**.

**Ground:** in a twin-VESC buggy the two controllers already share ground through the CAN link and
through battery negative — heavy gauge and low impedance. A common ground here is correct, and you
do not need a third wire into the connector if ground is already bonded. Confirm continuity rather
than assuming it.

#### 2.5b Calibrate the PPM input on **both** controllers

This is the #1 cause of *"one motor starts before the other"* and uneven steering.

- Run **VESC Tool → PPM Input calibration** on **VESC 1 and VESC 2**. Move the throttle through its
  full travel and capture **min / centre / max** on each. A typical hobby PPM range lands somewhere
  around 1.0–2.0 ms; what matters is not the numbers but that **both controllers see the same range**.
- Tune so **both motors start at the same low throttle and top out together** — mapped start-to-end,
  so the two match across the whole pull, not just at full throttle.
- Set the **minimum just above the resting pulse** so there is no idle creep. Too close to rest and
  the motor will crawl with the trigger released; a small margin fixes it.
- Use **zero deadband** unless you have a reason otherwise.
- Also run **motor / FOC detection** per VESC. See `docs/VESC_SMOOTH_START_QUICK_REFERENCE.md` and
  `docs/VESC_TUNING_PROCESS.md`.

> If one VESC will not calibrate at all — bar flat at zero — **stop and go back to 2.5a.** That is a
> power fault, not a calibration fault, and no amount of re-mapping will fix it.

### 2.6 Steering direction — wheels-up cross-steer check ⚠️ the top safety gate
Differential steering is **cross-steered**:
- To turn **RIGHT**, the **LEFT motor (motor 1 / black)** spins up (drives the left side forward → craft yaws right). Left turn → right motor.
- **Wheels-up / props-clear, verify:** move the stick/toggle **RIGHT → the buggy must try to turn RIGHT.** If it turns the wrong way, flip **`steering_inverted`** (default `0` — every builder verifies their own; plenty of builds need `1`).
- Also confirm **motor assignment** (which physical motor is 1 vs 2 / left vs right) and **prop spin/thrust direction** — that's VESC `m_invert_direction`, a **separate** setting from `steering_inverted`.
- ⚠️ **Why this is the top gate:** a wrong steering sign makes Follow-Me steer *away* from the rider = closed-loop runaway. Never trust FM until this passes wheels-up.

---

## Phase 3 — WEB PORTAL CONFIG (via each board's WiFi AP) — [WEB-PORTAL]

Power **one board at a time** (the other OFF, per 2.3), join its AP (password default `12345678` — change it), open the portal, set values, **Save All**, reboot if prompted.

### Must-be-right-BEFORE-water (RX)
| Setting | Safe starting value | Why |
|---|---|---|
| `steering_inverted` | as verified in 2.6 | Wrong sign = runaway |
| `rtm_stop_distance_m` | **10 m** (do NOT go below 10) | RTM hard-stop radius; below ~10 m is inside GPS error |
| `min_dist_m` | **10 m** (leave generous) | FM hard-stop distance; keeps FM from engaging on the tow rope |
| `followme_smoothing_band_m` | **10 m** | Decel band above the hard stop (follow distance = min_dist_m + band) |
| `boogie_vmax_in_followme_kmh` | 25 km/h or lower for your terrain | FM speed ceiling |
| `foiler_low_speed_kmh` | **8 km/h** | Below this rider speed FM holds (won't chase a swimmer) |
| `followme_mode` | **2 = Behind** (shipped default) | Pick the side you want and confirm it on the display — F1/F2/F3 |
| GPS anti-spoof (Phase A/B) | leave defaults: HDOP 2.0, accel 3.0 G, teleport 80 km/h, suspect 3, pair-dist 500 m, speed-diff 50 km/h | Tuned for this craft; only widen with reason |
| `rtm_compass_required` | 1 | Don't arm RTM without a good compass |
| `rtm_use_compass` | 1 (Hybrid) — never 2 on water | Mode 2 (compass-only) is bench-diagnostic only (motor EMI biases compass 100°+) |

> **Follow-Me mode mapping.** The same on every surface — firmware, both on-device portals, and the
> standalone config tool:
>
> | Value | Mode | Where the buggy sits |
> |---|---|---|
> | **1** | **Near Right** | behind and to your right |
> | **2** | **Behind** | directly behind you — **shipped default** |
> | **3** | **Near Left** | behind and to your left |
>
> `0` disables FM steering entirely (RTM throttle-limit only). Set the side you want, then confirm it
> on the TX display: **F1 / F2 / F3**.
>

### Tweak-later-OK *(safe to adjust after first sessions)* — [WEB-PORTAL, tweak-later-OK]
- **Units:** TX `speed_src` (lead with **mph**: value 5 = TX GPS mph, 4 = RX GPS mph) and `dist_unit` (0 = m/km, 1 = ft/mi). A common convention: **distances in metres, speeds in mph.**
- **Display:** `fm_display_mode` (1=TX speed … 2=distance to buggy), `rtm_display_mode`.
- **Feel:** `rtm_steer_response` preset, `motor_ramp_s` (0.75 s default; smooths throttle + steering), `near_diag_offset_deg` (offset off straight-behind).
- **Timers:** `fm_arm_window_s` (180 s — keep generous so the arm survives float→takeoff→tow→whip), `sleep_timeout_s`.
- **Magnet (only if the Hall sensor is fitted):** TX `mag_mode` (0=off, 1=FM, 2=RTM, 3=FM@2s/RTM@5s).

> `near_diag_offset_deg` ships at **45°**, which puts **Near Right at 135°** and **Near Left at 225°**
> relative to your heading. Raise it to tuck the buggy further behind you; lower it to bring it wider.

---

## Phase 4 — DRY / ARMING TEST (still on land, props clear) — [BENCH]

**Practise the arming gestures until they are muscle memory.** You will be doing them floating in
chop, one-handed, with a foil under you. Learn them dry first.

> ### 📍 Do this outside — and expect a fault if you don't
>
> **Indoors you will almost certainly get a fault, and that is the system working correctly.**
>
> FM and RTM both require a **trustworthy GPS fix on both units**. Through a roof you will not get
> one, so the moment you arm, the gate fails and the buggy **STOPS** — `St` on the display plus a
> long buzz — and you have to re-arm. Nothing is broken. **That is the fault that stops a runaway**,
> and seeing it once, on the bench, is genuinely worth doing: it teaches you what the failure looks
> like before you meet it on the water.
>
> **Then take it outside** where both units can actually see the sky, and do it properly:
>
> 1. Wait for **both** GPS dots to go **solid** — TX and RX. Solid means a fix good enough to steer
>    on (see below); blinking means it is still acquiring.
> 2. Arm FM. **Standing right next to the buggy, it will not engage** — you are inside the minimum
>    distance. It **HOLDS**: still armed, waiting, ready to resume. That is the proximity gate, and
>    it is deliberate.
> 3. Walk away from the buggy and watch the **distance readout on the remote climb** — that is your
>    end-to-end proof that both GPS units and the telemetry link are healthy and talking.
>
> **Hold vs fault — know the difference, they look different on purpose:**
>
> | | What triggers it | What the buggy does |
> |---|---|---|
> | **HOLD** | too close · you slowed down · you fell | Pauses, **stays armed**, resumes on its own |
> | **FAULT** | GPS / compass / radio dropout | **Stops**, shows `St` + long buzz, **you must re-arm** |
>
> **When does the GPS dot go solid?** Not on satellite count — on **fix quality**. The TX requires a
> valid position *and* valid speed, fresher than `tx_gps_stale_timeout_ms`, *and* **HDOP at or below
> `gps_max_hdop` (default 2.0)**. All four, or the dot keeps blinking. Satellite count alone is a
> poor signal: a u-blox module has been observed reporting 254 km/h and 4800 m altitude as a
> "high-confidence" fix on **5–7 satellites with HDOP under 3**, which is exactly why the gate is
> quality-based instead.

Confirm the gestures and display before you're in the water:

- **Arm FM (toggle, works while floating):** **LEFT tap → RIGHT hold ~3 s.** Display shows **F1/F2/F3**, remote buzzes **two quick taps** = armed. Repeat the gesture to cycle **F1→F2→F3→F0-off** before you're on the throttle.
- **Arm FM (magnet, works during the tow — if fitted):** hold magnet ~2 s, feel one pulse, pull away → two taps = armed (toggle: same gesture disarms).
- **Arm RTM:** **RIGHT tap → LEFT hold ~5 s** (needs `gps_en=1` + `rtm_enabled=1`).
- **Reading the FM bar:** no bar = disarmed · **sweeping** = armed & ready · **blinking in place** = armed but still getting GPS/link · **steady distance bar** = following · **`St`** = stopped.
- **Logging (AUX):** logging is **OFF at boot by design** — that is not a fault.

  > **Leave `logger_en` at 0.** With it at 0 the RX boots with logging off, so it never fills SPIFFS
  > sitting on the bench or parked in the car. **Short-press AUX any time the RX is running to start
  > logging — the AUX LED blinks 5× — and short-press again to stop (2 blinks).** You get logging per
  > session, on demand. Setting `logger_en` to 1 only means it starts recording the moment it powers
  > up, whether you are moving or not.
- **Holds vs Stops:** a fall / slow-down / too-close is a **HOLD** — buggy pauses, stays armed, resumes on its own. A GPS/compass/radio dropout is a **FAULT** — it **stops** (`St` + long buzz), throttle returns, and you must **re-arm**.

---

## Phase 5 — ON THE WATER → FOILING — [WATER]

Preconditions to *engage* (you can arm before these are perfect; it won't engage until they're met): paired, **GPS fix on both** units, healthy radio + telemetry, calibrated compass.

1. **Float & arm** (toggle: LEFT tap → RIGHT hold; or magnet mid-tow). Two taps = armed. The arm survives up to `fm_arm_window_s` (180 s) with no throttle.
2. **Whip / separate**, throttle held. FM **engages only when the geometry proves you've separated** — beyond the engage distance for **2 continuous seconds**, confirmed by both GPS units. No button, no timer.
   - Two ways to separate: whip yourself past the buggy, or keep throttle and steer the buggy to its offset side so it peels off while you carry into the wave.
3. **Following:** the buggy trails at your set side/distance, steering itself. You keep the throttle held; the buggy only ever moves on **your** throttle and only *subtracts* from it.
   - Keep your eyes on the wave. Trust line-of-sight — the distance bar/number is an assist and can read ~15 m off up close.
4. **Fall / slow / too close →** it HOLDs (stays armed, resumes). **Let go of the trigger →** instant stop. **Fault →** `St`, re-arm to continue.
5. **Disarm:** repeat the arm gesture (toggle), or hold the magnet ~2 s (long buzz = off); arming RTM also disarms FM.

---

## After the session — pulling logs & telemetry
If you logged (AUX on), pull the CSV two ways:
- **RX serial:** `?list` then `?download /<file>.log` (prints CSV), `?logstat` to diagnose, `?lograte <Hz>` to set rate.
- **WiFi web portal:** open the logs page and download.

The CSV captures what you need to measure range and performance at your current TX/RX settings:

| Column | What it gives you |
|---|---|
| `battery_current_A` · `motor_current_A` | amps in / amps out |
| `speed_kmh` | speed |
| `tx_distance_m` | distance between remote and buggy |
| `rssi_dbm` · `snr_db` | link quality — how far you can go on these settings |
| `latitude` · `longitude` · `datetime_unix` | track and timing |
| `voltage_V` · `ERPM` · `duty_cycle_%` · `temp_mos_C` · `fault_code` | VESC telemetry |

`?printrssi` also shows link quality live if you want it on the bench rather than after the fact.

---

### Where the numbers live
- Field ranges: `Source/V2_Integration_{Tx,Rx}/ConfigService.ino` · defaults: `defaultConf` in `BREmote_V2_{Tx,Rx}.h` · labels: `WebUiEmbedded.h` · standalone tool: `docs/BREmote_V2.5-Evo_Web_Serial_Config_Tool.html`.
- Rider flow: [`FOLLOW_ME_GUIDE.md`](FOLLOW_ME_GUIDE.md) · display: [`display-reference.md`](display-reference.md) · flashing: [`FLASHING_RX_ARDUINO.md`](FLASHING_RX_ARDUINO.md) / [`FLASHING_TX_ARDUINO.md`](FLASHING_TX_ARDUINO.md) · VESC: [`VESC_SMOOTH_START_QUICK_REFERENCE.md`](VESC_SMOOTH_START_QUICK_REFERENCE.md).
