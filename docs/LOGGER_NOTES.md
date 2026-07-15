# RX On-Board VESC Logger — Notes

_V2.5-Evo — last updated 2026-07-14_

The RX logs binary VESC + RTM telemetry records to on-board SPIFFS for post-ride analysis
(propeller comparison, max-speed runs, RTM/steering tuning). This note covers the log rate
default, how to change it at runtime, and how long the flash holds a session.

## Default rate: 3 Hz

- **Boot default = 3.0 Hz** (`log_interval_ms = 333`, `Source/V2_Integration_Rx/Logger.ino`).
- **Why 3 Hz:** it is the prop / max-speed testing default (Andres). 3 Hz is plenty for speed and
  trend logging, and it stretches on-board session capacity considerably versus 5 Hz.
- **Not persisted:** the rate is a RAM boot static — there is **no confStruct/SPIFFS field** for it.
  Changing the firmware line above is the only way to move the *default*; any runtime change
  (below) resets to 3 Hz on the next reboot.

## Change the rate at runtime (serial)

Send over the RX USB serial console:

```
?lograte <Hz>
```

Examples: `?lograte 5` (RTM/steering analysis), `?lograte 2`, `?lograte 0.1`.
Handled by `cmdLogRate` → `setLogRate()` (`System.ino`, `Logger.ino`). Valid range: >0 to 1000 Hz.
The change applies immediately and lasts until reboot (then back to the 3 Hz default).

## Per-session guidance

- **3 Hz** — default. Prop swaps, max-speed passes, general session/trend capture.
- **5 Hz** — bump up for RTM / follow-me steering analysis, where finer heading/error resolution
  matters. Use `?lograte 5` at the start of a tuning session.
- **1–2 Hz** — very long endurance logging where per-second detail is not needed.

## Capacity — record size and duration

- **Record size = 52 bytes** (`sizeof(VescLogData)`; `static_assert` in `BREmote_V2_Rx.h`).
- **SPIFFS partition = 1.875 MB** (`0x1E0000`) on the custom no-OTA 4 MB partition table
  (`Source/V2_Integration_Rx/partitions.csv`: 2.0 MB app + 1.875 MB SPIFFS).
- Durations below are continuous single-file logging on a freshly formatted SPIFFS, accounting for
  normal filesystem overhead:

| Rate | Bytes/hour | Approx. continuous logging on 1.875 MB SPIFFS |
|---|---|---|
| 5 Hz | ~936 KB/hr | ~1.9 hr |
| **3 Hz (default)** | ~562 KB/hr | **~3.2 hr** |
| 2 Hz | ~374 KB/hr | ~4.8 hr |
| 1 Hz | ~187 KB/hr | ~9.6 hr |

- **Resilience:** each session is its own file, and `ensureFreeSpace()` rolls off the **oldest** log
  when SPIFFS fills while protecting the active file — so a full session always fits; only old
  sessions age out. Download/clear logs before a long day if you want to keep everything.

## Related

- Partition table: `Source/V2_Integration_Rx/partitions.csv` (custom no-OTA; SPIFFS is wiped and
  reformatted on the first flash after switching partition schemes — settings reset, re-run `runcal`).
- Log record layout / columns: `convertToLogData()` and `VescLogData` (`Logger.ino`, `BREmote_V2_Rx.h`).
