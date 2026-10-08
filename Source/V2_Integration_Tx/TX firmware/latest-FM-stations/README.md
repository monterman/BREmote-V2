# TX (handheld remote) — fm-stations NIGHTLY build

**Nightly, bleeding edge, still under on-water testing. Use at your own risk.**
For stable firmware use the `master` branch.

| | |
|---|---|
| Firmware | TX SW27 (V2.5-Evo), `fm-stations` branch |
| Built from commit | `c69dd68` |
| Board / FQBN | HT-CT62 (ESP32-C3) — `esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=huge_app` |
| File | `BREmote-TX-SW27-fm-stations-c69dd68.bin` (app image, `0x10000`) |

## Before you flash

1. **Update the remote AND the receiver together.** Use the matching RX build in
   `Source/V2_Integration_Rx/RX firmware/latest-FM-stations/`. Mixing this TX with an older or
   stable RX is not supported.
2. **Download your buggy's logs before flashing the RX** — the matching RX build writes log
   format 3 and will not turn older logs into CSV.

## Flashing

- **App image `.bin` -> `0x10000`.** This is the only image published here. Use it to update a unit
  that already runs V2.5-Evo: your settings, pairing and calibration stay, because nothing outside the app slot is written.
- **Blank or bricked board?** Follow the normal flashing guide,
  [Flashing with the Flash Download Tool](../../../../docs/FLASHING_WITH_DOWNLOAD_TOOL.md). Full-chip (`.merged.bin`, `0x0`) images are not
  published: a write at `0x0` erases the settings storage.

Example (esptool): `esptool --chip esp32c3 --port COMx write-flash 0x10000 BREmote-TX-SW27-fm-stations-c69dd68.bin`

Known issues for this branch are listed at the top of the repository README.
