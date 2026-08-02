# BREmote TX — prebuilt firmware

Flash these directly with **esptool** — no Arduino IDE, no toolchain, no compiling.

Board: **HT-CT62 (ESP32-C3)** · partition scheme `huge_app` · app offset **`0x10000`**

## Which one do I want?

| File | SW | What it is |
|---|---|---|
| `BREmote-TX-SW27-gps-verified.bin` | 27 | **Start here.** Current. Every GPS config write is ACK-verified, auto-detects u-blox M8 vs M9/M10, and never transmits at an unconfirmed baud. Adds `?gpscfg`, `?gpsbaud`, `?gpssetup`. |
| `BREmote-TX-SW27-pre-gpsbaud.bin` | 27 | The build immediately **before** the baud work. Same features otherwise. Use this to isolate whether a problem is GPS-related. |
| `BREmote-TX-SW26R2-rtm-working.bin` | 26R2 | Known-good historical build from 2026-06-05. RTM working; Follow-Me not yet matured. Fallback if something newer misbehaves. |

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
config, calibration and pairing are preserved. `SW_VERSION` is 27 across all three, so moving
between them does not reset settings.

## After flashing

The TX sits on the **charge screen** while USB is connected — that is normal, and GPS init has
not run yet. Send **`?exitchg`** to boot through. Then:

```
?gpscfg      →  expect  dynModel : 5 (Sea)
```

If it says anything else, see [`docs/hardware/gps-troubleshooting.md`](../../../docs/hardware/gps-troubleshooting.md).

> Full-flash `.merged.bin` images are deliberately **not** published here. They are 4 MB each and
> overwrite the whole chip including your settings — the wrong tool for updating a working remote.
