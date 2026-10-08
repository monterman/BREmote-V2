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

## Log level 6 — IMU (V2.5-Evo, 2026-10-08)

`?set log_level 6`, `?save`, then start a log with AUX. Level 6 is the full 126 B level-5 record plus
two 22 B IMU blocks = **170 B/record** (98 CSV columns). Log format stays 3 (a tail append), so every
62 / 104 / 126 B file already on the board still downloads.

- **VESC 2's IMU** (`vesc2_imu_age_ms`, `vesc2_roll_deg`, `vesc2_pitch_deg`, `vesc2_yaw_deg`,
  `vesc2_gyro_x/y/z_dps`, `vesc2_acc_x/y/z_g`, `vesc2_imu_status`). The RX asks VESC 1 to forward
  `COMM_GET_IMU_DATA` to CAN ID 2 inside the existing VESC poll, **2 Hz**, **only while a level-6 log is
  recording** and only while VESC 2's values poll is answering. Nothing in any control path reads it.
- **Status:** 1 OK, 2 STALE (older than 1.5 s; the age is real, values -999), 3 IMPLAUSIBLE (|acc| under
  0.5 g: the IMU is off or not detected in VESC 2's App Settings; values -999), -999 = never answered.
- **RX IMU** (`rx_imu_*`, `rx_roll_deg` …): reserved columns for a future IMU on the RX. None is fitted;
  they always read -999. (A driver on the RX's single I2C bus would share it with the motor-enable
  swap, so it needs its own design and audit; VESC 1's own IMU over the UART is the better path.)
- **Rate:** 3 Hz is right. The IMU updates at 2 Hz, so some rows repeat a sample — `vesc2_imu_age_ms`
  says which. 5 Hz buys nothing for the IMU.
- **What it can tune:** the pitch threshold (read pitch together with `vesc2_acc_x_g`, VESC 2 current /
  ERPM and GPS speed — the AHRS pitch reads high under forward acceleration) and heel (roll with
  `vesc2_gyro_z_dps` and `vesc2_acc_y_g`; a sustained turn biases roll, and the bias shows in acc y).
- **What it cannot tune:** the 80 dps gyro threshold. Gyro spikes last 0.1–0.5 s and are mostly missed
  at 2 Hz — use the script's own peak prints or VESC Tool's realtime IMU view.
- **Capacity:** about 56 min at 3 Hz / 34 min at 5 Hz on 1690 KB free (owner's RX2, no reserve
  subtracted); about 40 / 24 min against the figure `?logstat` reports. Clear old logs first.
- **`?diag`** shows a `VESC 2 IMU` line (poll success, polling / idle / waiting / backed off), the last
  sample with its age and verdict, and an `RX IMU` line.

### Axes and signs (VESC 2 as mounted)

Values are in VESC 2's own frame as configured in VESC Tool (X to the nose), with no sign flips in the
RX, so a threshold read off the log applies 1:1 to the script's `get-imu-rpy` / `get-imu-gyro`. The
signs below are recorded at the bench (B4–B6 of the level-6 test plan) and are not yet measured:

| Column | Physical meaning | Sign when… | Bench result |
|---|---|---|---|
| `vesc2_pitch_deg` | nose up / down | nose lifted ~20° | not yet measured |
| `vesc2_roll_deg` | heel left / right | heeled left ~15° / right ~15° | not yet measured |
| `vesc2_gyro_y_dps` | pitch rate | quick nose flick up | not yet measured |
| `vesc2_gyro_z_dps` | turn rate | spun by hand to the left | not yet measured |
| `vesc2_acc_z_g` | vertical | level and still (expect ≈ ±1 g) | not yet measured |

If pitch reads about 0.35 for a 20° lift the radians-to-degrees step is missing; about 1150 means it
was applied twice (`kImuRpyIsRadians` in `Source/Common/VescImu.h`).

## Related

- Partition table: `Source/V2_Integration_Rx/partitions.csv` (custom no-OTA; SPIFFS is wiped and
  reformatted on the first flash after switching partition schemes — settings reset, re-run `runcal`).
- Log record layout / columns: `convertToLogData()` and `VescLogData` (`Logger.ino`, `BREmote_V2_Rx.h`).
