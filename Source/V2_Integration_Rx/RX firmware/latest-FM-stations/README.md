# RX (receiver / buggy) — fm-stations NIGHTLY build

**Nightly, bleeding edge, still under on-water testing. Use at your own risk.**
For stable firmware use the `master` branch.

| | |
|---|---|
| Firmware | RX SW36 (V2.5-Evo), `fm-stations` branch |
| Built from commit | `87f688f` |
| Board / FQBN | HT-CT62 (ESP32-C3) — `esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=custom` (uses the sketch's own `partitions.csv`; never `huge_app`) |
| File | `BREmote-RX-SW36-fm-stations-87f688f.bin` (app image, `0x10000`) |
| SHA-256 | `f5230ab7888eb6ac9edee7bb893526348f12fd47dfc8d08556ee1fdd49356b57` |

## Before you flash

1. **Download your buggy's logs first.** This build writes log format 3. It will not turn logs
   recorded by older firmware into CSV.
2. **Update the receiver AND the remote together.** Use the matching TX build in
   `Source/V2_Integration_Tx/TX firmware/latest-FM-stations/`. Mixing this RX with an older or
   stable TX is not supported.

## Flashing

- **App image `.bin` -> `0x10000`.** This is the only image published here. Use it to update a unit
  that already runs V2.5-Evo: your settings, pairing, compass calibration and logs stay, because nothing outside the app slot is written.
- **Blank or bricked board?** Follow the normal flashing guide,
  [Flashing with the Flash Download Tool](../../../../docs/FLASHING_WITH_DOWNLOAD_TOOL.md). Full-chip (`.merged.bin`, `0x0`) images are not
  published: a write at `0x0` erases the settings storage.

Example (esptool): `esptool --chip esp32c3 --port COMx write-flash 0x10000 BREmote-RX-SW36-fm-stations-87f688f.bin`

Known issues for this branch are listed at the top of the repository README.
