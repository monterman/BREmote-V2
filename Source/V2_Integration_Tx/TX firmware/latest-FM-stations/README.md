# TX (handheld remote) — fm-stations NIGHTLY build

**Nightly, bleeding edge, still under on-water testing. Use at your own risk.**
For stable firmware use the `master` branch.

| | |
|---|---|
| Firmware | TX SW27 (V2.5-Evo), `fm-stations` branch |
| Built from commit | `61ec981` |
| Board / FQBN | HT-CT62 (ESP32-C3) — `esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=huge_app` |
| File (update) | `BREmote-TX-SW27-fm-stations-61ec981.bin` (app image, `0x10000`) |
| SHA-256 | `234b0844fa528ab6700cbfa0c45d39850ba063508580d87114a2872c7895e305` |
| File (first install) | `BREmote-TX-SW27-fm-stations-61ec981-merged.bin` (full chip, `0x0`) |
| SHA-256 | `69635f22e3133a7fd9c3c9a635a6092716c74226dadec977306602ea076aacb6` |

Both files were rebuilt on 2026-10-18 from a clean export of `61ec981`. The code is unchanged.

## Before you flash

1. **Update the remote AND the receiver together.** Use the matching RX build in
   `Source/V2_Integration_Rx/RX firmware/latest-FM-stations/`. Mixing this TX with an older or
   stable RX is not supported.
2. **Download your buggy's logs before flashing the RX** — the matching RX build writes log
   format 3 and will not turn older logs into CSV.

## Flashing

- **Update (the unit already runs V2.5-Evo): app image `.bin` -> `0x10000`.** Your settings, pairing
  and calibration stay, because nothing outside the app slot is written.
- **First install (coming from Ludwig's firmware, another fork, or a blank board): `-merged.bin` -> `0x0`.**
  Other firmware uses a different partition layout with a 1.25 MB app slot. This app is about 1.5 MB, so
  the app-only file does not fit and the remote stays blank. The merged file writes the bootloader, the
  V2.5-Evo partition table and the app in one go. **It erases the whole chip: settings, pairing and
  calibration.** Then set up with the
  [Setup Wizard](https://monterman.github.io/BREmote-V2/setup-wizard.html). Use the app-only `.bin` for
  every update after that.
- Tool steps: [Flashing with the Flash Download Tool](../../../../docs/FLASHING_WITH_DOWNLOAD_TOOL.md).
  For the merged file, set the address to `0x0` instead of `0x10000`.

Example (esptool), update: `esptool --chip esp32c3 --port COMx write-flash 0x10000 BREmote-TX-SW27-fm-stations-61ec981.bin`

Example (esptool), first install: `esptool --chip esp32c3 --port COMx write-flash 0x0 BREmote-TX-SW27-fm-stations-61ec981-merged.bin`

Known issues for this branch are listed at the top of the repository README.
