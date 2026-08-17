# BREmote RX — prebuilt firmware

Flash these directly with **esptool** — no Arduino IDE, no toolchain, no compiling.

Board: **HT-CT62 (ESP32-C3)** · **custom partition table** (`partitions.csv` in this sketch
folder) · app offset **`0x10000`**

## Which one do I want?

| File | SW | What it is |
|---|---|---|
| `BREmote-RX-SW35-compass-orientation.bin` | **35** | **Start here.** Current. Everything below, **plus compass mounting orientation** — `?compasscal` now measures it for you. ⚠️ **SW35 resets your config once.** See below. |
| `BREmote-RX-SW34-dyn-model.bin` | 34 | The last SW34 build. Everything in `dual-compass` **plus a selectable GPS dynamic model** — set `gps_dyn_model 4` if you ride above ~500 m altitude. See below. |
| `BREmote-RX-SW34-dual-compass.bin` | 34 | **Field-verified by a beta tester** on an HGLRC M100-5883 (2026-08-16). Everything in `gps-verified` **plus automatic compass detection** — drives the **QMC5883L** (BN-880) *or* the **QMC5883P** (HGLRC M100-5883) from one image, chosen at boot by I²C address. See below. |
| `BREmote-RX-SW34-gps-verified.bin` | 34 | The previous build, **kept deliberately** — the field-proven one. QMC5883L only. ACK-verified GPS config with automatic `CFG-VALSET` fallback, so it works with a BN-880 *and* with an M10. Adds `?gpsbaud`, `?gpssetup`. |
| `BREmote-RX-SW34-pre-gpsbaud.bin` | 34 | The build immediately **before** the baud work. Use this to isolate whether a problem is GPS-related. |
| `BREmote-RX-SW32-rtm-working.bin` | 32 | Known-good historical build from 2026-06-05. RTM working; Follow-Me not yet matured. Fallback if something newer misbehaves. |

### What `dual-compass` adds (2026-08-15)

The RX now **detects which magnetometer is fitted at boot** and drives it correctly. One image,
either module:

| I²C address | Part | Found on |
|---|---|---|
| `0x0D` | **QMC5883L** | Beitian BN-880, HGLRC M100 Pro |
| `0x2C` | **QMC5883P** | HGLRC M100-5883 |
| `0x1E` | HMC5883L | very old BN-880 stock — **reported, not supported** |

`?i2c` names whichever it finds, and the boot log says which part it is driving.

**Why this needed a real driver and not just a new address.** The two parts are different silicon,
not a revision. Most importantly their data block starts at a **different register** — `0x00` on the
L, `0x01` on the P. Reading a P with the L's driver returns `CHIPID, XL, XH, YL, YH, ZL`: every axis
shifted one byte. It does not error. It returns a smooth, plausible, completely wrong heading — which
on a Follow-Me buggy is a safety problem, not a cosmetic one. The read path branches on the detected
part for exactly that reason.

They also differ in sensitivity at 8 G (3000 vs 3750 LSB/G) and in axis frame.

> ⚠️ **Re-run `?compasscal` after changing GPS/compass module.** Stored `mag_offset_*` values are raw
> counts and do not survive a part change. Heading is `atan2(y, x)` so a uniform scale error cancels —
> the offsets do not. The firmware prints this reminder at boot.

> ⚠️ **Not yet bench-tested on a QMC5883P by the maintainer.** Written from the QST datasheet and
> cross-checked against iNav and Betaflight, and it compiles clean — but no M100-5883 has been in my
> hands. The QMC5883L path is byte-for-byte unchanged from `gps-verified`, so **BN-880 builds are
> unaffected**. If you fit an M100-5883, please point the buggy north / east / south / west after
> calibrating and confirm the heading tracks, then report back.
>
> **Both builds are `SW_VERSION` 34, so moving between them does NOT touch your settings** — flash
> either way freely, no re-pairing, no re-calibration.

### What `dyn-model` adds (2026-08-16)

**`gps_dyn_model` — the u-blox navigation filter's platform model, now a setting.**

| Value | Model | Use it when |
|---|---|---|
| **0** | default → Sea | **Leave it here.** What every board already does. |
| **4** | Automotive | **You ride above ~500 m altitude.** Sea has a 500 m ceiling and fixes degrade above it. |
| **5** | Sea (explicit) | Same as 0, stated outright. |

```
?set gps_dyn_model 4
?save
```
Reboot. The boot log then reads `dynModel=Automotive` instead of `dynModel=Sea`.

**Why Sea remains the default.** Below 500 m it is the better model: it constrains the filter to
~25 m/s and pins altitude near the surface, which removes a degree of freedom and sharpens
course-over-ground — and COG is what Follow-Me actually steers on. Switching everyone to Automotive
would cost every sea-level rider that sharpening to fix a problem they do not have.

**Why Portable is not offered.** u-blox's dynModel 0 permits 310 m/s. It is what produced the bogus
254 km/h / 4800 m *high-confidence* fixes that started this whole line of work. Anything that is not
an explicit `4` resolves to Sea, so a corrupt or out-of-range value fails toward the conservative
model — never toward Portable.

> **No config wipe.** This reuses a reserved `uint16_t` slot renamed in place, so
> `sizeof(confStruct)` stays 184 and `SW_VERSION` stays 34. Every existing board reads `0` there,
> which means Sea — exactly what it did before. Nothing to re-pair, nothing to re-calibrate.

### What SW35 adds — compass mounting orientation (2026-08-16)

**`?compasscal` now starts and ends pointing north, and measures three things in one run:**

1. Hard/soft-iron calibration — as before
2. **Mounting handedness** — from which way the heading ran while you turned clockwise
3. **Mounting rotation** — from the first sample, taken while pointing north

**The new procedure:**

```
Point the nose of the buggy at NORTH
Run ?compasscal  (or short-press BIND)
Rotate SLOWLY CLOCKWISE, two full circles
Finish with the nose back on NORTH
```

Clockwise matters — the turn direction is how handedness is detected. Ending on north is how the
result is checked.

**Why this exists.** Heading is `atan2(y, x)` on the sensor's own axes, so mounting the module
rotated made every heading wrong by that angle, and the old calibration was mathematically blind to
it — a rotation leaves the calibration circle centred and round, so nothing looked wrong. Mount it
however it fits; tell the firmware once.

**Tolerances are deliberately forgiving** — a rejected calibration costs you a re-run on your feet:

| Check | Limit |
|---|---|
| Total rotation seen | ≥ 400° (want ~720°) |
| Finish vs start | within ±40° |
| Rotation stored | snapped to 0 / 90 / 180 / 270 |

If a run is too sloppy to trust, **the iron calibration is still saved and the previous orientation
is kept** — it never stores a guess. That is the failure ArduPilot warns about, where a calibration
*"appears to succeed while leaving the compass in a very bad state."*

**Mirrored modules** (antenna-down, or a chip whose axes are handed the other way) are stored as a
**negative `mag_scale_y`** — negating that axis *is* the mirror fix, and the field already existed.

> ⚠️ **SW35 resets your configuration once.** `confStruct` grew from 184 to 188 bytes, so the
> firmware rewrites config to defaults on first boot. **Back up first with `?conf`**, restore after
> with `?setconf <blob>` + `?applyconf`. You will need to re-pair and re-run `?compasscal` —
> which you want to do anyway, since that is what sets the new orientation.

### Which heading mode should I run? (`rtm_use_compass`)

RTM needs to know which way the buggy is pointing. There are two real choices:

| Mode | Set | What it does | Run it when |
|---|---|---|---|
| **Hybrid** | `1` *(default)* | GPS course while moving; compass when too slow for course to be reliable | Your compass is calibrated and its mounting orientation is set |
| **GPS COG only** | `0` **+ `rtm_compass_required 0`** | Steers only above ~3 km/h. Below that it holds straight instead of using the compass | You suspect the compass, or you want to prove whether it is the problem |
| Compass only | `2` | **Bench diagnostic. Never on water.** | Never |

```
?set rtm_use_compass 0
?set rtm_compass_required 0
?save
```

> ⚠️ **You must set BOTH.** `rtm_compass_required` does not check for a compass despite its name —
> it requires a valid *heading of any kind* before RTM will arm. With the compass disabled there is
> no heading while the buggy sits still, and RTM is armed from a standstill, so arming fails every
> time with `STOP: No valid heading source`. It reads like COG-only mode is broken; it is the gate.
>
> **The firmware now refuses to save that combination** and tells you which setting to change.

**What COG-only costs you:** steering at low speed and on the final approach. As RTM decelerates it
stops steering and coasts in straight — less precise. **What it buys you:** the compass cannot
contribute an error, because it is never read.

**Worth doing as an A/B.** Run the same spot twice, one mode each, and see which behaves better on
your build. If COG-only is clean and Hybrid is not, the compass is your problem.

### Also in SW35 — RTM heading trust

Two fixes beyond the compass orientation work:

- **RTM will no longer steer on a compass that has been caught disagreeing with GPS.** That check
  existed but was only ever applied to Follow-Me — Return-to-Me never read it. RTM now holds
  straight instead, which at close range is the safe outcome.
- **A GPS course that was valid moments ago is now held for 3 seconds** before falling back to the
  compass. RTM drives at 4.0 km/h while GPS course is abandoned below 3 km/h, and that 1 km/h margin
  is inside the speed signal's own noise — so the heading source was flipping on noise alone, and
  every flip handed steering to a compass that is badly wrong under motor load.

## Flash it

```bash
esptool --chip esp32c3 --port COM<N> write-flash 0x10000 BREmote-RX-SW34-dual-compass.bin
```

**Identify the board by MAC, not by COM number.** The TX and RX are the same chip and COM
numbers move between reboots:

```bash
esptool --chip esp32c3 --port COM<N> read-mac
```

## 🚨 Read this before flashing an RX

**The RX uses its own `partitions.csv` — 2.0 MB app + 1.875 MB SPIFFS.** Never build or flash it
with `PartitionScheme=huge_app`. That overrides the custom table, **halves SPIFFS, moves it, and
wipes your config, compass calibration and all on-board logs.**

Verify the table matches before writing — if the digest matches, your data survives:

```bash
esptool --chip esp32c3 --port COM<N> verify-flash 0x8000 <build>.partitions.bin
```

**Back up first.** `?conf` prints a Base64 blob; `?setconf <blob>` then `?applyconf` restores it.
On the RX the expensive fields are the **compass calibration** (`mag_offset_*`, `mag_scale_*`) —
losing those means re-running `?compasscal` physically — plus pairing.

These are **app-only** images (`0x10000`), so they do not touch the partition table or SPIFFS.
The flash itself never erases your settings, and **all three SW34 builds are interchangeable** —
move between `dual-compass`, `gps-verified` and `pre-gpsbaud` freely.

> ⚠️ **`SW32-rtm-working` is the exception, and on the RX it is expensive.** The firmware
> compares the stored `SW_VERSION` against its own on boot and rewrites the config to defaults
> when they differ. That build is version **32**, the current one is **34**, so rolling back to
> it **wipes the compass calibration** — which you can only restore by physically re-running
> `?compasscal`, walking the buggy through two full circles — **plus pairing and every setting.**
> Coming forward again wipes it a second time.
>
> **Back it up before you roll back**, and only roll back if you actually need to.
>
> *(This paragraph previously claimed `SW_VERSION` was 34 across all published builds and that
> switching between them was safe. It was wrong, and on this board being wrong costs you a
> calibration you have to redo on your feet.)*

## After flashing

The RX has **no display and no LEDs** — serial is its only diagnostic surface. Watch the boot log:

```
GPS: heard the module at 115200 baud
GPS config [legacy CFG (u-blox 6/7/8)]: dynModel=Sea OK | GSV OK | GLL OK | VTG OK
```

Then confirm independently:

```
?gpscfg      →  expect  dynModel : 5 (Sea)
```

⚠️ `?gpsbaud` and `?gpssetup` block the main loop for several seconds — RTM/FM and VESC polling
stop while they run. **Bench only. Never with the buggy powered for a run.**

If anything looks wrong, see [`docs/hardware/gps-troubleshooting.md`](../../../docs/hardware/gps-troubleshooting.md).

> Full-flash `.merged.bin` images are deliberately **not** published here. They are 4 MB each and
> overwrite the whole chip including SPIFFS — which on the RX means your compass calibration.
