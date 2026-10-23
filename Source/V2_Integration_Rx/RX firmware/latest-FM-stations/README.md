# RX (receiver / buggy) — fm-stations NIGHTLY build

**Nightly, bleeding edge, still under on-water testing. Use at your own risk.**
For stable firmware use the `master` branch.

| | |
|---|---|
| Firmware | RX SW36 (V2.5-Evo), `fm-stations` branch |
| Built from commit | `61ec981` |
| Board / FQBN | HT-CT62 (ESP32-C3) — `esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=custom` (uses the sketch's own `partitions.csv`; never `huge_app`) |
| File (update) | `BREmote-RX-SW36-fm-stations-61ec981.bin` (app image, `0x10000`) |
| SHA-256 | `0710f32c8937dc364a39f635da6b458617e1b2d3c3c432419a038d798b7d38af` |
| File (first install) | `BREmote-RX-SW36-fm-stations-61ec981-merged.bin` (full chip, `0x0`) |
| SHA-256 | `39950d1ee5f76a3cf41d8f569878db784d0f2eeb2d0cabd5b6855faaca115565` |

Both files were rebuilt on 2026-10-18 from a clean export of `61ec981`. The code is unchanged.

## Before you flash

1. **Download your buggy's logs first.** This build writes log format 3. It will not turn logs
   recorded by older firmware into CSV.
2. **Update the receiver AND the remote together.** Use the matching TX build in
   `Source/V2_Integration_Tx/TX firmware/latest-FM-stations/`. Mixing this RX with an older or
   stable TX is not supported. The TX build there is now **SW27R4** (`32d67b0`); it changes only how the
   remote finds its own GPS pins, so it matches this RX.

## Flashing

- **Update (the unit already runs V2.5-Evo): app image `.bin` -> `0x10000`.** Your settings, pairing,
  compass calibration and logs stay, because nothing outside the app slot is written.
- **First install (coming from Ludwig's firmware, another fork, or a blank board): `-merged.bin` -> `0x0`.**
  Other firmware uses a different partition layout with a 1.25 MB app slot. This app is about 1.4 MB, so
  the app-only file does not fit and the board stays blank. The merged file writes the bootloader, the
  V2.5-Evo partition table and the app in one go. **It erases the whole chip: settings, pairing, compass
  calibration and logs.** Download your logs first, then set up with the
  [Setup Wizard](https://monterman.github.io/BREmote-V2/setup-wizard.html). Use the app-only `.bin` for
  every update after that.
- Tool steps: [Flashing with the Flash Download Tool](../../../../docs/FLASHING_WITH_DOWNLOAD_TOOL.md).
  For the merged file, set the address to `0x0` instead of `0x10000`.

Example (esptool), update: `esptool --chip esp32c3 --port COMx write-flash 0x10000 BREmote-RX-SW36-fm-stations-61ec981.bin`

Example (esptool), first install: `esptool --chip esp32c3 --port COMx write-flash 0x0 BREmote-RX-SW36-fm-stations-61ec981-merged.bin`

Known issues for this branch are listed at the top of the repository README.
