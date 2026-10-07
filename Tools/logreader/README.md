# BREmote RX Log Reader

A standalone Python decoder for the RX board's on-board deep log. The only "decoder" the
firmware itself ships is the CSV formatter behind the `?download` serial command and the WiFi
`/api/logs/download` endpoint, and both print the Follow-Me gate word (`fm_gate_flags`) as one
raw 32-bit integer and leave several other fields in raw `_dx10` units. This tool decodes the
same on-disk records into real units, names every coded field, unpacks the gate word into one
named boolean per bit, and turns the row-by-row data into a readable timeline of what Follow-Me
actually did during a session.

Python 3.10+, standard library only — no install step, no dependencies.

## Getting a log off the buggy

The RX writes one binary `.log` file per logging session to its on-board SPIFFS (see the
"V2.5-Evo: Data Logger" section of the repo README for starting/stopping a session with the AUX
button). Pull it off with either of the two paths the firmware already has (both read from
`Source/V2_Integration_Rx/Logger.ino`):

- **Serial (USB)** — connect to the RX at its serial console and send:
  ```
  ?list                     # see what's on the board, with sizes
  ?download <filename>      # e.g. ?download /050326_011615.log
  ```
  This prints the file as CSV between `=== BEGIN CSV DATA ===` and `=== END CSV DATA ===`
  markers. Save everything between (and not including) those two lines as a `.csv` file, or feed
  the whole capture to this tool as-is — see "Input" below.

- **WiFi** — with the RX's WiFi AP connected, `GET` the same data directly as a CSV download:
  ```
  http://<rx-ip>/api/logs/download?file=<filename>
  ```
  This is what the RX embedded web page's **Manage Logs** section and the Web Serial Config
  Tool's log manager both call.

Either path gives you **CSV**, already run through the firmware's own formatter. If you instead
copy the **raw binary `.log` file** off SPIFFS (e.g. via a future raw-file endpoint, or by reading
flash directly), this tool reads that too — see "Input" below. The binary file is smaller and
carries the file's own self-describing 8-byte header (magic, format version, log level, record
size), so it is the more future-proof of the two if a raw download path ever ships.

## Running it

```
python bremote_log.py FILE [--csv OUT] [--timeline] [--summary] [--from S] [--to S]
```

- `FILE` — either a raw binary `.log` file or the device's CSV output (with its header line).
  The tool auto-detects which one it is.
- `--csv OUT` — write the **expanded CSV** (every field in real units, plus the decoded name and
  gate-bit columns described below) to `OUT`.
- `--timeline` — print the human-readable event timeline to stdout.
- `--summary` — print session counts (RETURN episodes, RTM activations, latch sets, stops by
  reason, measured log rate, session duration) to stdout.
- `--from S` / `--to S` — only process records between `T+S` and `T+S` seconds (relative to the
  first record in the file). Applies to all three outputs.
- Running with none of `--csv` / `--timeline` / `--summary` defaults to printing the timeline.
  Combine any of the three flags to get more than one output in a single pass.
- Exits non-zero (with a plain-English message on stderr) if the file has no valid BRLG magic, an
  unsupported header format, or a CSV header that does not match any of the six the firmware can
  print. Otherwise it stays quiet — no progress spam.

Examples:

```
python bremote_log.py 050326_011615.log --timeline
python bremote_log.py 050326_011615.log --csv 050326_011615_expanded.csv --summary
python bremote_log.py device_download.csv --timeline --from 120 --to 180
```

## Input

- **Binary `.log`** — the 8-byte `LogFileHeader` (magic `BRLG`, format version, log level, record
  size) picks the record layout. Six tiers exist today, by record size (bytes → columns):

  | Bytes | Tier | Columns | What it adds over the previous tier |
  |---|---|---|---|
  | 59 | `L3` | 31 | base VESC + RTM/heading record |
  | 65 | `L4_DIAG` | 35 | + GPS/loop diagnostics (no Follow-Me data) |
  | 83 | `L4_83` | 48 | + the Follow-Me audit block (gate flags, distance, mode/state/block reason, `fm_return_reason`) — historical, written 2026-09-17 to 2026-09-19 |
  | 85 | `L4_RAW` | 49 | + raw (unfiltered) rider speed — never shipped alone by the firmware, kept so a reader can still name a file this size |
  | 87 | `L4` | 51 | + the two differential-mixer motor commands — **today's everyday level-4 record** |
  | 109 | `L5` | 65 | + the "everything" block: rider lat/lng and fix age, classic-RTM `rtm_phase`, align cap/influence/mixer influence, auto-return override, `fm_flags_sent`, keepalive age |

  Those six are **log format 1** (files written before 2026-10-02). Two later formats exist, and the
  header's `format_ver` byte picks between them - **never the record size alone**, because format 2
  and format 3 both write a 126-byte record with different contents:

  | Format | Bytes | Tier | Columns | What it is |
  |---|---|---|---|---|
  | 2 | 62 / 90 / 112 | `L3_V2` / `L4_V2` / `L5_V2` | 33 / 53 / 67 | the format-1 tiers plus the 3-byte M-2 motor-gate block in the base record (2026-10-02) |
  | 2 | 126 | `L5_VESC2` | 76 | level 5 + the second VESC's telemetry at the tail (2026-10-06, first build) |
  | 3 | 62 | `L3_V2` | 33 | level 3, unchanged |
  | 3 | 104 | `L4_V3` | 62 | level 4 + the second VESC's telemetry (2026-10-06: both VESCs at level 4) |
  | 3 | 126 | `L5_V3` | 76 | level 4 (with VESC 2) + the level-5 block |

  **Firmware that writes format 3 cannot turn format-1 or format-2 files into CSV on the board** - it
  refuses them with a plain-English message. Download logs before flashing. This tool still reads all
  three formats, binary or CSV.

  Any other record size decodes the largest known layout **of the same format** that fits inside it and warns once about
  the ignored trailing bytes — the same range-tiering `logCsvHeaderFor()` uses in the firmware, so
  a reader here and the firmware's own CSV path always agree on which columns a given file gets.
  A future firmware that tail-appends new fields needs one new entry in `RECORD_LAYOUTS_BY_FORMAT`, not a
  rewrite.
- **Device CSV** — every layout above prints a different header line, so the CSV header alone picks
  the layout, whatever the format. For format 1, the tool matches the first line exactly against the six headers the firmware
  can emit (`LOG_CSV_HEADER_L3` / `_L4_DIAG` / `_L4_83` / `_L4_RAW` / `_L4` / `_L5` in
  `BREmote_V2_Rx.h`) to pick the same six layouts. The column order in that header **is** the
  input contract; a test in `test_bremote_log.py` asserts this tool's own field table still
  generates those exact six strings, so the two cannot silently drift apart. Two of those columns
  (`fm_aligning`, `fm_boost`) are not struct fields — they are bits 17/18 of `fm_gate_flags`,
  printed a second time by the firmware so a human never has to mask the flag word; this tool
  derives them the same way (from `fm_gate_flags`) for both binary and CSV input, rather than
  trusting two possibly-inconsistent copies of the same fact.

## What the columns mean

The base column set, scaling, and sentinel values are the firmware's own contract — see the
`VescLogData` / `VescLogDataL4` / `VescLogDataL5` struct field comments and the `FM_LOG_GATE_*`
bit-list comment block in `Source/V2_Integration_Rx/BREmote_V2_Rx.h` (search for `FOLLOW-ME AUDIT
BLOCK` and `LEVEL-5`) rather than a second copy of that documentation here.
`Source/V2_Integration_Rx/RTMState.ino` documents the `FmState`, `FmStopReason`, `FmReturnReason`,
and classic-RTM `rtm_phase` codes, and the `telemetry.fm_flags` bit map, this tool's name tables
are built from.

This tool's **expanded CSV** differs from the firmware's own CSV in one deliberate way: every
field the firmware leaves in raw `_dx10`/sentinel-int form (heading and compass columns, GPS
course, COG age, heading-error terms, rider fix age) is renamed to a plain engineering-unit column
(`_deg`, `_ms`, …) and scaled, instead of staying a raw integer only the firmware's own comments
explain. `fm_gate_flags` is kept as its raw integer *and* unpacked into one boolean column per
named bit (`thr_held`, `fault_ok`, `speed_ok`, `dist_ok`, `sep_latched`, `diverge`, `pivoting`,
`in_grace`, `heading_disagree`, `fade_bypass`, `transit`, `return_candidate`, `proof_ok`,
`needs_dengage`, `yield_to_rtm`, `return_window`, `steer_takeover` (reserved, not yet set by any
code path), `fm_aligning`, `fm_boost`); a bit this tool has not seen documented yet shows up
dynamically as `bit_N`. `fm_flags_sent` (level 5 only) gets the same treatment, prefixed
`fm_flags_` (`fm_flags_armed`, `fm_flags_engaged`, `fm_flags_armed_not_ready`,
`fm_flags_fault_sticky`, `fm_flags_effective_auto_return`; an unassigned bit shows up as
`fm_flags_bit_N`). `rtm_source`, `rtm_confidence`, `fm_mode`, `fm_state`, `fm_block_reason`,
`fm_return_reason`, and `rtm_phase` all keep their raw numeric code plus a `_name` column.

## How to read a RETURN in the timeline

Every time `fm_state` transitions into `RETURN` and back out, the timeline prints one summary
line for that episode: the time it entered, the buggy-to-rider distance at entry, the closest
distance it reached, how long the episode lasted, the state it exited into, the **real exit
reason** (`fm_return_reason`, sticky — the row where `fm_state` leaves `RETURN` carries the code
for why: arrived, cancelled, timed out, faulted, left/yielded, or the rider steered), and — for
files that carry `motor0_cmd`/`motor1_cmd` (87 B tier and up) — the tightest motor0/motor1 split
the differential mixer commanded during the episode's align sub-phase (`fm_aligning` set), which
is the pivot-boost signature: a wide split (e.g. `motor0_min=10, motor1_min=122`) says the buggy
was pivoting hard to face the rider before running the return leg straight. Read the transitions
*around* the episode line for more of the "why": a `gate return_candidate set` line shortly before
entry shows the rider-stopped condition that opened the door; `boost on`/`boost off` bracket the
pivot; a non-zero `FM block reason:` line explains a STOPPING exit instead of a normal arrival.
`exit_reason` prints `not logged in this record version` only for a file whose tier predates the
Follow-Me block entirely (below 83 B) — every 83 B file and up carries the real code.

## Tests

```
python -m unittest test_bremote_log.py -v
```

Builds synthetic records straight from this tool's own record-layout table (no separate hand-copy
of the layout), decodes them, and checks field values, gate-bit names, and the timeline/summary
output for: a scripted ARMED → latch → ACTIVE → RETURN-candidate → RETURN → HOLD session (87 B
tier); a RETURN episode that exits STEERED with a boost/align sub-phase and a motor0/motor1 split
(87 B tier); an `rtm_phase` transition and `fm_flags_sent` bit decode (109 B tier). Also asserts
the field table's generated CSV headers stay byte-identical to the six header strings copied from
`BREmote_V2_Rx.h`, so a firmware column change that is not mirrored here fails a test instead of
silently producing a wrong "expanded" column.
