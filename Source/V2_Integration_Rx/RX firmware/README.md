# BREmote RX — prebuilt firmware

Flash these directly with **esptool** — no Arduino IDE, no toolchain, no compiling.

Board: **HT-CT62 (ESP32-C3)** · **custom partition table** (`partitions.csv` in this sketch
folder) · app offset **`0x10000`**

## Which one do I want?

| File | SW | What it is |
|---|---|---|
| `BREmote-RX-SW34-dual-compass.bin` | 34 | **Start here.** Current. Everything in `gps-verified` **plus automatic compass detection** — drives the **QMC5883L** (BN-880) *or* the **QMC5883P** (HGLRC M100-5883) from one image, chosen at boot by I²C address. See below. |
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
