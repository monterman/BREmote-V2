# BREmote TX — prebuilt firmware

Flash these directly with **esptool** — no Arduino IDE, no toolchain, no compiling.

Board: **HT-CT62 (ESP32-C3)** · partition scheme `huge_app` · app offset **`0x10000`**

## Which one do I want?

| File | SW | What it is |
|---|---|---|
| `BREmote-TX-SW27-gps-verified.bin` | 27 | **Start here.** Current. Every GPS config write is ACK-verified, auto-detects u-blox M8 vs M9/M10, and never transmits at an unconfirmed baud. Adds `?gpscfg`, `?gpsbaud`, `?gpssetup`. |
| `BREmote-TX-SW26R2-rtm-working.bin` | **26R2** | Known-good historical build from 2026-06-05. RTM working; Follow-Me not yet matured. Fallback if something newer misbehaves. ⚠️ **Different `SW_VERSION` — see below.** |

> The RX folder has a third, intermediate `pre-gpsbaud` build. The TX equivalent was withdrawn
> and is deliberately not published.

## Flash it

```bash
esptool --chip esp32c3 --port COM<N> write-flash 0x10000 BREmote-TX-SW27-gps-verified.bin
```

**Find your port by MAC, not by COM number** — COM numbers move between reboots, and the TX and
RX are the same chip:

```bash
esptool --chip esp32c3 --port COM<N> read-mac
```

## Before you flash

**Back up your config.** `?conf` prints a Base64 blob; `?setconf <blob>` then `?applyconf`
restores it. Throttle calibration and pairing are the expensive things to lose.

**Check the partition table hasn't moved** — if it matches, your settings survive:

```bash
esptool --chip esp32c3 --port COM<N> verify-flash 0x8000 <build>.partitions.bin
```

These are **app-only** images (`0x10000`). They do not touch the partition table or SPIFFS, so
the flash itself never erases your settings.

> ⚠️ **But rolling back to `SW26R2` DOES reset your config, and that is not the flash's doing.**
> The firmware compares the stored `SW_VERSION` against its own on boot; when they differ it
> rewrites the config to defaults. `SW26R2` is version **26**, the current build is **27**, so
> going backwards **wipes throttle calibration, pairing and every setting**.
>
> **Back up first — `?conf` — and expect to restore with `?setconf <blob>` + `?applyconf` and
> re-pair.** Moving *forward* again from 26R2 to 27 wipes it a second time, for the same reason.
>
> *(This paragraph previously claimed `SW_VERSION` was 27 across all builds and that switching
> between them was safe. It was wrong, and it was wrong in the direction that costs you a
> calibration.)*

## After flashing

The TX sits on the **charge screen** while USB is connected — that is normal, and GPS init has
not run yet. Send **`?exitchg`** to boot through. Then:

```
?gpscfg      →  expect  dynModel : 5 (Sea)
```

If it says anything else, see [`docs/hardware/gps-troubleshooting.md`](../../../docs/hardware/gps-troubleshooting.md).

> Full-flash `.merged.bin` images are deliberately **not** published here. They are 4 MB each and
> overwrite the whole chip including your settings — the wrong tool for updating a working remote.
