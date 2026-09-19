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
  unsupported header format, or a CSV header that does not match any of the three the firmware
  can print. Otherwise it stays quiet — no progress spam.

Examples:

```
python bremote_log.py 050326_011615.log --timeline
python bremote_log.py 050326_011615.log --csv 050326_011615_expanded.csv --summary
python bremote_log.py device_download.csv --timeline --from 120 --to 180
```

## Input

- **Binary `.log`** — the 8-byte `LogFileHeader` (magic `BRLG`, format version, log level, record
  size) picks the record layout: 59 bytes is a level-3 record, 65 bytes is the historical
  level-4-diagnostics-only record (no Follow-Me block), 83 bytes is the current level-4 record
  with the Follow-Me audit block. Any other record size decodes the largest known layout that
  fits inside it and warns once about the ignored trailing bytes, so a future firmware that
  tail-appends new fields (the P2 station-fade work is explicitly laid out for this) needs one new
  entry in `RECORD_LAYOUTS`, not a rewrite.
- **Device CSV** — the tool matches the first line exactly against the three headers the firmware
  can emit (`LOG_CSV_HEADER_L3` / `_L4_DIAG` / `_L4` in `BREmote_V2_Rx.h`) to pick the same three
  layouts. The column order in that header **is** the input contract; a test in
  `test_bremote_log.py` asserts this tool's own field table still generates those exact three
  strings, so the two cannot silently drift apart.

## What the columns mean

The base column set, scaling, and sentinel values are the firmware's own contract — see the
`VescLogData` / `VescLogDataL4` struct field comments and the `FM_LOG_GATE_*` bit-list comment
block in `Source/V2_Integration_Rx/BREmote_V2_Rx.h` (search for `FOLLOW-ME AUDIT BLOCK`) rather
than a second copy of that documentation here. `Source/V2_Integration_Rx/RTMState.ino` documents
the `FmState`, `FmStopReason`, and `FmReturnReason` enums this tool's name tables are built from.

This tool's **expanded CSV** differs from the firmware's own CSV in one deliberate way: every
field the firmware leaves in raw `_dx10`/sentinel-int form (heading and compass columns, GPS
course, COG age, heading-error terms) is renamed to a plain engineering-unit column
(`_deg`, `_ms`, …) and scaled, instead of staying a raw integer only the firmware's own comments
explain. `fm_gate_flags` is kept as its raw integer *and* unpacked into one boolean column per
named bit (`thr_held`, `fault_ok`, `speed_ok`, `dist_ok`, `sep_latched`, `diverge`, `pivoting`,
`in_grace`, `heading_disagree`, `fade_bypass`, `transit`, `return_candidate`, `proof_ok`,
`needs_dengage`, `yield_to_rtm`, `return_window`); a bit this tool has not seen documented yet
shows up dynamically as `bit_N`. `rtm_source`, `rtm_confidence`, `fm_mode`, `fm_state`, and
`fm_block_reason` all keep their raw numeric code plus a `_name` column.

## How to read a RETURN in the timeline

Every time `fm_state` transitions into `RETURN` and back out, the timeline prints one summary
line for that episode: the time it entered, the buggy-to-rider distance at entry, the closest
distance it reached, how long the episode lasted, and the state it exited into. Read the
transitions *around* that line for the "why": a `gate return_candidate set` line shortly before
entry shows the rider-stopped condition that opened the door; a `gate sep_latched cleared` or a
non-zero `FM block reason:` line at exit explains a STOPPING exit, while `exit_state=HOLD` with no
block-reason line means it arrived and parked normally. The one thing the timeline **cannot**
currently tell you is the exact `FmReturnReason` the controller used to leave the episode — that
enum is documented in `RTMState.ino` and named in this tool (`FM_RETURN_REASON_NAMES`) for the
day a column carries it, but no field in `VescLogDataL4` logs it yet, so every episode line prints
`exit_reason=not logged in this record version` until that changes.

## Tests

```
python -m unittest test_bremote_log.py -v
```

Builds synthetic records straight from this tool's own record-layout table (no separate hand-copy
of the layout), decodes them, and checks field values, gate-bit names, and the timeline/summary
output for a scripted ARMED → latch → ACTIVE → RETURN-candidate → RETURN → HOLD session. Also
asserts the field table's generated CSV headers stay byte-identical to the three header strings
copied from `BREmote_V2_Rx.h`, so a firmware column change that is not mirrored here fails a test
instead of silently producing a wrong "expanded" column.
