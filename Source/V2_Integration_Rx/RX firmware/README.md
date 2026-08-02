# BREmote RX — prebuilt firmware

Flash these directly with **esptool** — no Arduino IDE, no toolchain, no compiling.

Board: **HT-CT62 (ESP32-C3)** · **custom partition table** (`partitions.csv` in this sketch
folder) · app offset **`0x10000`**

## Which one do I want?

| File | SW | What it is |
|---|---|---|
| `BREmote-RX-SW34-gps-verified.bin` | 34 | **Start here.** Current. ACK-verified GPS config with automatic `CFG-VALSET` fallback, so it works with a BN-880 *and* with an M10 (HGLRC M100 Pro / M100-5883). Adds `?gpsbaud`, `?gpssetup`. |
| `BREmote-RX-SW34-pre-gpsbaud.bin` | 34 | The build immediately **before** the baud work. Use this to isolate whether a problem is GPS-related. |
| `BREmote-RX-SW32-rtm-working.bin` | 32 | Known-good historical build from 2026-06-05. RTM working; Follow-Me not yet matured. Fallback if something newer misbehaves. |

## Flash it

```bash
esptool --chip esp32c3 --port COM<N> write-flash 0x10000 BREmote-RX-SW34-gps-verified.bin
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
`SW_VERSION` is 34 across all three, so moving between them does not reset settings.

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
