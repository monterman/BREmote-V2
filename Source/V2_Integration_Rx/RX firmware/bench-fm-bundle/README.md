# RX (receiver / buggy) — fm-bundle BENCH build

**Not a release.** A test build for the owner's buggy, ahead of the next nightly. Not water-tested yet.
For the nightly, use `../latest-FM-stations/`. For stable firmware, use the `master` branch.

| | |
|---|---|
| Firmware | RX SW36 (V2.5-Evo), `fm-bundle` branch |
| Built from commit | `2fca870` |
| Board / FQBN | HT-CT62 (ESP32-C3) — `esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=custom` (uses the sketch's own `partitions.csv`; never `huge_app`) |
| File | `BREmote-RX-SW36-fm-bundle-2fca870.bin` (app image, `0x10000`) |
| SHA-256 | `352d8dfcf57c57cf5958a1b1f94d158ca3dddf5d5b03ecf2c805d4caf2e6c00d` |

## What is new compared with the nightly (61ec981)

- **Steering:** `steering_influence` 51-99 now gives a full-strength pivot below 30 % motor throttle (the inside
  motor stops). From 60 % up, the turn uses the stored value, and between 30 % and 60 % it blends.
  Settings 0-50 and 100 behave as before. No settings wipe. See `CHANGELOG.md`.
- **Logs:** log format 5. Every log file carries the boot counter and reset reason, and level 6 adds a compass
  reading under throttle. Decode with `Tools/logreader/bremote_log.py` from this branch.
- Pair with the nightly remote (TX SW27, `61ec981`). The remote has not changed since.

## Flashing

- **Download your buggy's logs first.**
- **App image `.bin` → `0x10000` only.** Settings, pairing, compass calibration and logs stay, because nothing
  outside the app slot is written. This only works on a unit that already runs V2.5-Evo with the RX
  partition table (`spiffs` at `0x210000`, size `0x1E0000`).

Example (esptool 5.x): `esptool --chip esp32c3 --port COMx write-flash 0x10000 BREmote-RX-SW36-fm-bundle-2fca870.bin`
