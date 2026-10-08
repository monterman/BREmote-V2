# TX (handheld remote) — fm-stations NIGHTLY build

**Nightly, bleeding edge, still under on-water testing. Use at your own risk.**
For stable firmware use the `master` branch.

| | |
|---|---|
| Firmware | TX SW27 (V2.5-Evo), `fm-stations` branch |
| Built from commit | `c69dd68` |
| Board / FQBN | HT-CT62 (ESP32-C3) — `esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=huge_app` |
| Files | `BREmote-TX-SW27-fm-stations-c69dd68.bin` (app image) · `BREmote-TX-SW27-fm-stations-c69dd68.merged.bin` (full-flash image) |

## Before you flash

1. **Update the remote AND the receiver together.** Use the matching RX build in
   `Source/V2_Integration_Rx/RX firmware/latest-FM-stations/`. Mixing this TX with an older or
   stable RX is not supported.
2. **Download your buggy's logs before flashing the RX** — the matching RX build writes log
   format 3 and will not turn older logs into CSV.
3. This remote cannot select the front stations F4 / F5 yet; they are still in development.

## Flash offsets

- **App image `.bin` → `0x10000`.** Use this to update a remote that already runs V2.5-Evo. Your
  settings, pairing and calibration stay.
- **Merged image `.merged.bin` → `0x0`.** Only for a blank or bricked board. It overwrites the
  whole chip, **including the settings storage**: pairing, settings and calibration are erased.
  Re-pair and re-calibrate afterwards.

Example (esptool): `esptool --chip esp32c3 --port COMx write-flash 0x10000 BREmote-TX-SW27-fm-stations-c69dd68.bin`

Known issues for this branch are listed at the top of the repository README.
