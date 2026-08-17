# Changelog

## 2026-08-16 — RX SW35: the compass knows how it is mounted (⚠️ resets your RX settings once)

**Recommended: reflash the RX. Back it up first.**

### ⚠️ This one resets your RX configuration

`confStruct` grew from 184 to 192 bytes, so the RX rewrites its config to defaults on the first
boot after this flash. **Print a backup with `?conf` before you flash**, restore afterwards with
`?setconf <blob>` then `?applyconf`, then re-pair and re-run `?compasscal`. You want that
calibration run anyway — it is what sets the new mounting orientation.

Moving between two **SW35** builds resets nothing. It is only the 34→35 step (and a 35→34
rollback) that costs you the config and the compass calibration.

### Compass mounting orientation — `?compasscal` measures it for you

Heading is `atan2(y, x)` on the sensor's own axes. Mount the module rotated and every heading was
wrong by that angle — and the old calibration was blind to it, because a rotation leaves the
calibration circle centred and round. Nothing looked wrong. Mount it however it fits; tell the
firmware once:

```
Point the nose of the buggy at NORTH
Run ?compasscal  (or short-press BIND)
Rotate SLOWLY CLOCKWISE, two full circles
Finish with the nose back on NORTH
```

Clockwise is how handedness is detected. Ending on north is how the result is checked. One run
now produces three things: the hard/soft-iron calibration as before, the mounting **handedness**
(a mirrored module is stored as a negative `mag_scale_y`), and the mounting **rotation**, snapped
to 0/90/180/270.

A run too sloppy to trust **stores nothing rather than storing a guess** — the iron calibration is
still saved and the previous orientation is kept, and the report says plainly what was and was not
updated instead of printing "Success!" either way.

**New: `?magalign`** sets the mounting orientation on its own — point the nose at magnetic north,
run it, done — so orientation can be re-checked without redoing the slow two-circle iron sweep. It
refuses to run on an uncalibrated compass, and warns if the reading was more than 25° off the
nearest cardinal.

### Return-to-Me: no longer steers on a compass caught lying

A beta tester's buggy tracked straight toward him, then turned hard at 5–7 m. RTM decelerates
inside the approach zone, speed falls below the COG minimum, and heading hands over from GPS
course to the compass — and a rotated compass makes the heading **step** by the mounting angle at
exactly that moment.

Two guards existed and neither fired: the per-tick cross-check could only run while GPS course was
still trustworthy, and the sticky disagree fault was read only by Follow-Me — Return-to-Me never
read it. The check now sits at the source, in `getRtmHeading()`, so a discredited compass is not
handed out to RTM, to FM, or to anything added later. RTM holds straight instead, which at close
range is the safe outcome.

**A GPS course that was valid moments ago is now held for 3 seconds** before falling back to the
compass. RTM governs to 4.0 km/h and abandons course below 3 km/h — a 1 km/h margin that sits
inside the speed signal's own noise, so the heading source was flipping on noise alone and handing
steering to a compass badly wrong under motor load. A two-second-old course beats an inverted
compass.

### Heading mode: the COG-only trap is repaired for you

`rtm_use_compass 0` (GPS course only) combined with `rtm_compass_required 1` meant RTM could never
arm — that gate does not check for a compass despite its name, it demands a valid heading of any
kind, and there is none while the buggy sits still with the compass off. It failed with
`STOP: No valid heading source` and read like COG-only mode was broken.

Setting `rtm_use_compass 0` now clears `rtm_compass_required` automatically and prints why. It is
enforced on every save path including boot, so an old config backup carrying the trap is repaired
rather than leaving RTM unarmable.

The Follow-Me engage-distance floor now behaves the same way: a too-small value is **clamped up**
to the 8 m tow-rope minimum with a printed explanation, instead of failing validation and taking
every other setting down with it.

### TX: far fewer vibrations

A rider holding the remote while concentrating on a wave does not decode vibration patterns — they
feel *a buzz*. Seventeen trigger sites across seven patterns is not a language, it is noise. Six
triggers are gone: disarming RTM by steering, disarming FM, selecting F0, and cycling FM modes —
things **you** just did, where the display already shows the result. Everything that tells you
something you did not already know is kept.

Informational buzzes now wait for one already playing to finish rather than overwriting it; two
patterns colliding is what made them unreadable.

### Audit fixes before publication (`a8f8512`)

A pre-publication audit found several things this release announced that were not true in the
shipped code. None touched the motor path; four were user-visible and one shipped inside the
published binary. `confStruct` is unchanged at 192 bytes and `SW_VERSION` stays 35, so this is a
drop-in rebuild — no config wipe, no re-pairing, no re-calibration.

- **The RX's own WiFi config page was dead** in the published SW35 binary — the field array was
  never terminated and a block was deleted with it, so the page rendered nothing and no button
  worked. `gps_dyn_model` had also been spliced inside `gps_chip_type`'s options and four fields
  were missing. Repaired and reconciled 1:1 against the firmware's field table. **Eleven default
  values were wrong** — the page claimed the factory defaults were GPS off, telemetry off, EU868
  and Kalman off, when the shipped defaults are the opposite.
- **`gps_dyn_model` was honoured only on the legacy M8 path.** Both CFG-VALSET paths hardcoded
  Sea, so an M9/M10 owner above the 500 m ceiling silently kept Sea. Now single-sourced. The baud
  scan also recognises a checksum-verified UBX frame, so a module left UBX-only by a drone flight
  controller is detected instead of reported dead — detection stays listen-only and transmits
  nothing. NMEA output on UART1 is re-enabled on the VALSET path.
- **Compass:** an under-rotated re-calibration silently destroyed a stored mirror correction, and a
  motionless run saved a noise blob over a good calibration. Both refused now. QMC5883P init writes
  are checked — a failed axis-sign write reports the compass as absent rather than producing
  plausible wrong headings.
- **The heading-disagree backstop could never fire.** The Follow-Me idle path cleared its latch on
  every 100 ms tick, including while RTM was running, so the dwell never accumulated in the one
  mode the guard was written for. Cleared on state transitions only now, and a disagreement must be
  continuous. The COG hold is mirrored into the logger so logs stop misattributing the heading
  source during exactly the transitions those logs are read to diagnose.
- **22 bench commands could freeze the loop task mid-ride**, suspending every autonomous safety
  gate while the last steering override and throttle cap kept being applied. The worst were the
  streamers that run until you type `quit`, and `?download`. They now refuse to start while RTM or
  Follow-Me is engaged, and abort if an engagement begins after they started — the physical BIND
  button included. `?gpssetup` and `?wifiupd` are deliberately **not** abortable: interrupting them
  leaves the GPS module or the stored web UI worse than letting them finish. The watchdog now also
  arms on the first boot after a version bump, previously the one boot that ran without it.
- **A config blob that failed validation was already live** — copied into the running struct before
  being checked, and the result ignored. It now validates into a staging copy first, `?applyconf`
  reports pass or fail, and `?setconf` explains why a backup was rejected instead of failing
  mysteriously two steps later. Shared with the TX, which picks this up at its next rebuild.

---

## 2026-08-16 — RX: a GPS dynamic model you can set

**No config wipe.** This one reuses a reserved slot renamed in place, so `sizeof(confStruct)` stays
184 and `SW_VERSION` stays 34.

The RX hard-coded u-blox dynamic model 5 (Sea). The code comment said the ceiling was deliberate
and that this buggy would never be run above 500 m. The first beta tester to fit an M100-5883 lives
at 550 m — the documented limit was hit within three weeks, by the first outside user.

`gps_dyn_model` is now a setting:

| Value | Model | Use it when |
|---|---|---|
| **0** | default → Sea | **Leave it here.** What every board already does. |
| **4** | Automotive | **You ride above ~500 m altitude.** Sea has a 500 m ceiling and fixes degrade above it. |
| **5** | Sea (explicit) | Same as 0, stated outright. |

```
?set gps_dyn_model 4
?save
```

Reboot; the boot log then reads `dynModel=Automotive`.

**Sea stays the default** because below 500 m it is genuinely better: it constrains the filter to
~25 m/s and pins altitude near the surface, which sharpens course-over-ground — and course is what
Follow-Me actually steers on. Switching everyone to Automotive would cost every sea-level rider
that sharpening to fix a problem they do not have.

**Portable is not offered anywhere.** It permits 310 m/s and is what produced the bogus 254 km/h /
4800 m *high-confidence* fixes this whole line of work exists to prevent. Anything that is not an
explicit `4` resolves to Sea, so a corrupt or out-of-range value fails toward the conservative
model. Every board in the field holds `0` in that slot, which is why `0` means exactly what the
firmware did before the field existed.

---

## 2026-08-15 — RX: one image drives either compass

**No config wipe.** No `confStruct` change, no `SW_VERSION` bump — the I²C address *is* the
identification, so there is nothing to set and nothing to get wrong.

The RX hard-coded one magnetometer: a QMC5883L at `0x0D`, which is what a BN-880 carries. The
**HGLRC M100-5883** carries a **QMC5883P** at `0x2C`, so the RX simply did not see it — no compass,
10 blinks, no Follow-Me heading. It now detects which part is fitted at boot and drives it
correctly:

| I²C address | Part | Found on |
|---|---|---|
| `0x0D` | **QMC5883L** | Beitian BN-880, HGLRC M100 Pro |
| `0x2C` | **QMC5883P** | HGLRC M100-5883 |
| `0x1E` | HMC5883L | very old BN-880 stock — reported, **not** driven |

**Why this needed a real driver and not just a new address.** These are different silicon, not a
revision, and the data block starts at a **different register** — `0x00` on the L, `0x01` on the P.
Point the L's driver at a P and every axis is shifted one byte, with the chip ID as X's low byte.
It does not error and it does not look broken: it returns a smooth, plausible, completely wrong
heading. On a buggy that steers itself toward a rider in the water that is a safety failure, not a
cosmetic one. The read path branches on the detected part for exactly that reason.

Prompted by a pull request from **robertzach**, who got an M100-5883 working on his own fork.
Implemented independently rather than merged, because that change *replaces* the QMC5883L path
rather than adding to it — merging it as-is would have broken every BN-880 build.

> ⚠️ **Re-run `?compasscal` after changing GPS or compass module.** Stored `mag_offset_*` values are
> raw counts and do not survive a part change. The firmware prints this reminder at boot.

---

## 2026-08-15 — TX: GPS config replies are now checksum-verified

**No config wipe.** `SW_VERSION` stays 27; all three SW27 builds are interchangeable.

`ubxPoll()` returned the instant it had collected the payload and **never read the two checksum
bytes at all**. Any byte sequence in the stream that happened to look like a valid UBX header was
believed — and that stream carries NMEA and UBX at 5 Hz, so a chance match on four header bytes is
entirely plausible.

That function feeds the dialect detection which decides whether your module speaks the legacy
u-blox 6/7/8 command set or the M9/M10 `CFG-VALSET` set, and `?gpssetup` writes in whichever
dialect it reports. **A false verdict means the module is sent commands it cannot parse and
configuration fails silently** — no error, just a module that was never configured.

Not hypothetical: the RX carried the identical defect and reported a BN-880 — a u-blox M8, which
has no `CFG-VALGET` at all — as "M9/M10" on three consecutive runs. Fixed on the RX on 2026-08-02;
this is the same fix applied to the TX copy that was missed.

The fix accumulates the Fletcher-8 checksum across class, id, length and payload, then verifies
`CK_A` and `CK_B` before returning. A wrong checksum resyncs and keeps looking.

---

## 2026-07-25 — GPS heading fix (Alpha, pending on-water testing)

**Recommended: reflash both TX and RX.**

### The bug
The receiver has **one serial line shared between the GPS and the VESC**, switched back and forth by a multiplexer. The switching had the priority backwards: it sat on the VESC and only glanced at the GPS for 10 milliseconds, twice a second.

The GPS talks in bursts, so that window almost always landed in the silence between bursts. **The receiver was catching about 2% of what the GPS said** — roughly one course update every 25 seconds.

With no live course, Follow-Me fell back to the compass. The compass is snapshotted while the motor is idle and can't refresh while you're holding the trigger, so it went stale mid-run — **and the buggy steered the wrong way with nothing detecting it.**

The 10 ms delay turned out to be leftover code guarding something that had been deleted long ago. The multiplexer actually switches in under 3 microseconds.

### The fix
**Priority swapped: the line now rests on the GPS, and the VESC is the visitor.** Measured on hardware:

| | Before | After |
|---|---|---|
| GPS sentences per second | 0.1 | **84.8** |
| GPS fix | none | **holds a fix, 23 ms old** |
| VESC telemetry success | 72% | **100%** |
| Worst-case loop time | 239 ms | **29 ms** |

It also uses **fewer** multiplexer switches than before, not more.

### Also fixed
- **Follow-Me now refuses to steer on a dead heading.** A GPS course that stops changing is no longer treated as live just because its timestamp keeps updating, and a frozen course no longer silently hands steering to the compass. If the compass and GPS disagree by more than 45°, neither is trusted and the buggy holds straight.
- **New safety net:** if Follow-Me stops actually following — beyond twice the engage distance and not closing — it stops and hands control back.
- **Signal bar was reading low on a healthy link.** The scale bottomed out ~18 dB too early, which also caused false weak-signal buzzes.
- **Distance display:** the decimal dot now always means a decimal. It used to mean "×100" above 100 m, so 170 m showed as `1.7` and read as 1.7 m. Now `17` is 17 m, `1.7` is 1.7 m, and 100 m or more scrolls **FAR**. Metres only this version.
- **Follow-Me engage distance is now a real setting** — measure your tow rope and set at least a metre beyond it. Minimum 8 m, enforced. A value shorter than your rope would let Follow-Me engage while you're still being towed.
- **Log downloads over WiFi were missing five columns** (including signal strength) and reported **motor RPM 10× too low**. Both export paths now match.
- **Logging levels added** (`log_level`, 0–4). Level 4 records deep diagnostics — GPS throughput, whether the course is actually changing, I²C errors, worst loop time — so a session can be diagnosed from its log.
- **Compile fix:** the RX build reported "97% of program storage" and would have refused to build with ~800 KB still free. Use `PartitionScheme=custom` — see the RX flashing guide. Your partition layout is unchanged; only the size check was wrong.

### Still alpha
**This is fixed but not yet proven on the water.** The measurements above are bench and bucket tests. On-water validation is next. Test at your own risk, keep manual control in reach, and report anything you see.

Your saved settings are **not** wiped by this update.
