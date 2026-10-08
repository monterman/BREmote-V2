# Saved binaries — reflash without compiling

Compiling takes minutes. These folders hold **ready-to-flash binaries** so a bad build can be undone
in about 30 seconds, from any PC with the Arduino ESP32 core installed.

## Flash a saved set

```powershell
# newest saved RX set, board on COM15
.\firmware\FLASH-FROM-BACKUP.ps1 -Board RX -Port COM15

# a specific one
.\firmware\FLASH-FROM-BACKUP.ps1 -Board RX -Port COM15 -Set "SW36-2026-09-21-fm-stations-77a04c9-FLASHED"
```

Run with no `-Set` and it prints every saved set, newest first, then uses the newest.

## What it does and does not touch

- Writes bootloader (`0x0`), partition table (`0x8000`), `boot_app0` (`0xe000`) and the application (`0x10000`).
- **Nothing is written at or above `0x210000`, so SPIFFS survives** — your config, battery curve and logs stay.
- It does **not** change the partition scheme. The RX's own `partitions.csv` layout (2.0 MB app / 1.875 MB SPIFFS)
  is baked into the saved `*.partitions.bin`, which is why a saved set is safe to flash blind.

## Naming rule for new sets

`firmware/<RX|TX>/SW<nn>-<YYYY-MM-DD>-<branch>-<short-sha><-FLASHED if it is what shipped>/`

Keep in each folder: `*.ino.bin`, `*.bootloader.bin`, `*.partitions.bin`, `boot_app0.bin`,
`flash_args` (what arduino-cli used) and, optionally, `*.merged.bin` for the Espressif Flash Download Tool.

## Sets on hand

| Set | Board | What it is |
|---|---|---|
| `RX/SW36-2026-09-25-fm-stations-dd198e8-FLASHED` | RX | **CURRENT — flashed 2026-09-25.** Steering is instant (throttle ramps before the mixer), Q12 ramp honours 0.2–4.0 s, stick-takeover available but OFF (`steer_during_auto 0`), second ramp rate available but OFF (`auto_ramp_s 0`), FM engage ramp 1.5 s. Audited, 7/7 host tests. |
| `RX/SW36-2026-09-21-fm-stations-77a04c9-FLASHED` | RX | **PREVIOUS known-good — the rollback.** Auto-return (`fm_return_mode 1`), stop 3 m, pivot boost, steer-cancels-return, deep log L4/L5. This is the rollback. |
| `TX/SW27-2026-09-19-fm-stations-5fd3f79-FLASHED` | TX | **The known-good build in the remote today.** F-mode wrap fix (F0 removed), `rtm_disengage_distance_m 3`, M10 GPS support. Rebuilt 2026-09-24 from the same source that was flashed. |
| `RX/SW28 … SW33-DEV` (in `firmware/RX`, `firmware/_dev`) | RX | Older releases and dev builds from May–June 2026. |

> **TX note:** the remote uses `PartitionScheme=huge_app`, the RX uses its own `partitions.csv`. Each saved set
> carries its own `*.partitions.bin`, so the script writes the right layout for whichever board you name — but
> never flash an RX set to the TX or the other way round.

## If a flash fails at the beach

1. Re-run the same command — a failed write is not a brick; the ROM bootloader is always there.
2. Still failing: lower the baud in the script (`921600` → `460800`).
3. Port missing: unplug/replug USB, `[System.IO.Ports.SerialPort]::GetPortNames()` to see what appeared.
4. Board boots but misbehaves: flash the previous set from the table above, then `?conf` to confirm your settings.
