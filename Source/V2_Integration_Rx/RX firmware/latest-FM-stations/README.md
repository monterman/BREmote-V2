# RX (receiver / buggy) — fm-stations NIGHTLY build

**Nightly, bleeding edge, still under on-water testing. Use at your own risk.**
For stable firmware use the `master` branch.

| | |
|---|---|
| Firmware | RX SW36 (V2.5-Evo), `fm-stations` branch |
| Built from commit | `c69dd68` |
| Board / FQBN | HT-CT62 (ESP32-C3) — `esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=custom` (uses the sketch's own `partitions.csv`; never `huge_app`) |
| Files | `BREmote-RX-SW36-fm-stations-c69dd68.bin` (app image) · `BREmote-RX-SW36-fm-stations-c69dd68.merged.bin` (full-flash image) |

## Before you flash

1. **Download your buggy's logs first.** This build writes log format 3. It will not turn logs
   recorded by older firmware into CSV.
2. **Update the receiver AND the remote together.** Use the matching TX build in
   `Source/V2_Integration_Tx/TX firmware/latest-FM-stations/`. Mixing this RX with an older or
   stable TX is not supported.

## Flash offsets

- **App image `.bin` → `0x10000`.** Use this to update a unit that already runs V2.5-Evo. Your
  settings, pairing and compass calibration stay.
- **Merged image `.merged.bin` → `0x0`.** Only for a blank or bricked board. It overwrites the
  whole chip, **including the settings storage**: pairing, settings, compass calibration and all
  logs are erased. Re-pair, re-enter your settings and re-run `?compasscal` afterwards.

Example (esptool): `esptool --chip esp32c3 --port COMx write-flash 0x10000 BREmote-RX-SW36-fm-stations-c69dd68.bin`

Known issues for this branch are listed at the top of the repository README.
