# TX (handheld remote) — fm-stations NIGHTLY build

**Nightly, bleeding edge, still under on-water testing. Use at your own risk.**
For stable firmware use the `master` branch.

| | |
|---|---|
| Firmware | **TX SW27R4** (V2.5-Evo), `fm-stations` branch. The boot banner says SW 27; R4 is the file name only. |
| Built from commit | `67a007b` |
| Board / FQBN | HT-CT62 (ESP32-C3) — `esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=huge_app` |
| File (update) | `BREmote-TX-SW27R4-fm-stations-67a007b.bin` (app image, `0x10000`) |
| SHA-256 | `2f9acdf911dd5cae1c8bd234e0a9f0ed5a05c379845f9874b96c66eac1b31501` |
| File (first install) | `BREmote-TX-SW27R4-fm-stations-67a007b-merged.bin` (full chip, `0x0`) |
| SHA-256 | `b900cc09d8a21c8fc1a8237a7b97dee0db4a713303351050750d28c830637d3e` |

Both files were built on 2026-10-23 from a clean export of `67a007b`. It replaces `32d67b0` from earlier the
same day; the only change is the boot-line text for a GPS on G19, which now names OpenFoil (the store that sells
the factory-made remotes). If you already flashed `32d67b0`, there is no need to flash again.

## What is new in SW27R4: the remote finds its GPS on G18 or G19

- **Factory-made remotes (Ludwig / OpenFoil) send GPS data on G19.** Every earlier build only listened on
  G18, so those remotes never got a GPS fix (`?printgps` showed `Chars processed 0`). They are potted, so
  this is fixed in firmware.
- **At power-up the remote checks both pins for about 1.1 s** before it opens the GPS port, driving neither
  pin, and then listens on the one the GPS is sending on. It works at any baud, NMEA or UBX.
- **Self-built remotes wired to G18 see no change.** Data on G18, on neither pin or on both: the remote keeps
  the old pins.
- **Check it on the boot log** (USB serial, 115200): `TX GPS: data on G19, using RX=G19 TX=G18 (factory wiring - OpenFoil)`
  or `TX GPS: data on G18, ... (DIY wiring)`. `?printgps` shows the pins in use and the edge counts.
- **The check only runs while `gps_en = 1`.** If you switched the GPS off, turn it back on first:
  `?set gps_en 1`, `?save`, wait about 15 s, then power-cycle.
- **No config wipe.** `SW_VERSION` stays 27 and the settings layout is unchanged, so the app-only update keeps
  your settings, pairing and calibration.
- **Limit:** the remote cannot repair a GPS module that a drone flight controller left in UBX-only mode (no
  NMEA). It now says so at boot instead of pointing at the wiring. Fix such a module before fitting it: run
  `?gpssetup` on a receiver with current firmware, or use u-center.
- **Not yet run on a factory-made remote.** Audited (motor safety intact: nothing that drives the motors is
  touched; a wrong pin choice can only mean "no remote GPS", which Follow-Me and Return-To-Me already
  handle). Built clean, with no new compiler warnings.

**Previous build, kept for rollback:** `BREmote-TX-SW27-fm-stations-61ec981.bin` (update) and
`BREmote-TX-SW27-fm-stations-61ec981-merged.bin` (first install). Same code apart from the GPS pin check.

## Before you flash

1. **Update the remote AND the receiver together.** Use the matching RX build in
   `Source/V2_Integration_Rx/RX firmware/latest-FM-stations/` (RX SW36, `61ec981`; SW27R4 changes only the
   remote's GPS pins, so that RX build is still the match). Mixing this TX with an older or stable RX is not
   supported.
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

Example (esptool), update: `esptool --chip esp32c3 --port COMx write-flash 0x10000 BREmote-TX-SW27R4-fm-stations-67a007b.bin`

Example (esptool), first install: `esptool --chip esp32c3 --port COMx write-flash 0x0 BREmote-TX-SW27R4-fm-stations-67a007b-merged.bin`

Known issues for this branch are listed at the top of the repository README.
