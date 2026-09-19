#!/usr/bin/env python3
"""
bremote_log.py - decoder for the BREmote RX on-board deep log.

Reads either of the two forms a rider can get off the buggy:

  (a) a raw binary ``.log`` file straight off SPIFFS (8-byte self-describing
      header + N fixed-size records), or
  (b) the CSV text the firmware itself emits (serial ``?download`` or the
      WiFi ``GET /api/logs/download?file=...`` endpoint) - header line first.

and turns it into:

  - an EXPANDED CSV with real units on every field, the Follow-Me gate word
    unpacked into one boolean column per named bit, and the numeric state
    codes (rtm_source, rtm_confidence, fm_mode, fm_state, fm_block_reason)
    given readable names alongside the raw value.
  - a human-readable TIMELINE of state changes (FM state transitions, RTM
    active/ended edges, the gate-bit edges that matter for a return-to-rider
    diagnosis, fault stops, and one summary line per RETURN episode).
  - a SUMMARY of counts and rates for the whole session.

Record layouts, field order, scaling and the CSV column contract are all
sourced from ``Source/V2_Integration_Rx/BREmote_V2_Rx.h`` (``VescLogData``,
``VescLogDataL4``, ``LogFileHeader``, the ``FM_LOG_GATE_*`` bit defines and
the ``LOG_CSV_HEADER_*`` / ``logFormatCsvRow()`` macros) and
``Source/V2_Integration_Rx/RTMState.ino`` (``FmState``, ``FmStopReason``,
``FmReturnReason``). All three structs there are ``__attribute__((packed))``,
so byte offsets below follow field declaration order with no alignment gaps
- confirmed against the ``static_assert(sizeof(...) == N, ...)`` lines next
to each struct and cross-checked against the CSV formatter's own field
order, which is the one other place record layout is spelled out.

The record layout table (``RECORD_LAYOUTS``) is keyed by on-disk record
size, so a future record size is one new table entry, not a rewrite.

Python 3.10+, standard library only.
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
from datetime import datetime, timezone
from typing import Any, Callable, Optional


# ============================================================
# FILE HEADER - BREmote_V2_Rx.h: LogFileHeader / LOG_FILE_MAGIC / LOG_FILE_FORMAT_VER
# ============================================================
LOG_FILE_MAGIC = 0x474C5242          # little-endian bytes on disk read "BRLG"
LOG_FILE_FORMAT_VER = 1              # layout of the 8-byte header itself
LOG_FILE_HEADER_FMT = "<IBBH"        # magic u32, format_ver u8, log_level u8, record_size u16
LOG_FILE_HEADER_SIZE = struct.calcsize(LOG_FILE_HEADER_FMT)
assert LOG_FILE_HEADER_SIZE == 8


# ============================================================
# NAME TABLES - sourced from struct field comments and enum definitions
# ============================================================

# BREmote_V2_Rx.h, VescLogData.rtm_source comment:
#   "0=NONE, 1=GPS_COG, 2=COMPASS_SNAPSHOT, 3=COMPASS_LIVE (legacy mode)"
RTM_SOURCE_NAMES = {0: "NONE", 1: "GPS_COG", 2: "COMPASS_SNAPSHOT", 3: "COMPASS_LIVE"}

# BREmote_V2_Rx.h, VescLogData.rtm_confidence comment: "0=NONE, 1=LOW, 2=MEDIUM, 3=HIGH"
RTM_CONFIDENCE_NAMES = {0: "NONE", 1: "LOW", 2: "MEDIUM", 3: "HIGH"}

# BREmote_V2_Rx.h confStruct.followme_mode comment: canonical mapping, matches TX + README.
# fm_mode in the deep log is fm_mode_runtime: 1-3 declared, 0 off, 0xFF never declared this session.
FM_MODE_NAMES = {0: "disabled", 1: "near_right", 2: "behind", 3: "near_left", 0xFF: "never_declared"}

# RTMState.ino: enum FmState
FM_STATE_NAMES = {0: "IDLE", 1: "ARMED", 2: "ACTIVE", 3: "HOLD", 4: "STOPPING", 5: "RETURN"}

# RTMState.ino: enum FmStopReason + fmStopReasonName(). Codes are stable - never renumbered,
# only appended - because they are written into log files.
FM_STOP_REASON_NAMES = {
    0: "NONE",
    1: "PHASE_A (RX GPS rejected)",
    2: "PHASE_B (TX<->RX handshake failing)",
    3: "TX_STALE (rider GPS stale)",
    4: "RX_STALE (buggy GPS stale)",
    5: "HEADING (no valid heading source)",
    6: "LINK (LoRa link lost)",
    7: "DIVERGENCE (not closing on the rider)",
    8: "HEADING_DISAGREE (compass vs GPS course)",
    9: "RETURN_NOT_CLOSING (return leg did not close on the rider)",
}

# RTMState.ino: enum FmReturnReason + fmReturnReasonName(). NOT part of VescLogDataL4 today -
# no column carries it - so this table exists only so the timeline's RETURN-episode summary
# line can name the reason THE DAY the firmware adds a column for it. Until then every episode
# summary prints "not logged in this record version" for the exit reason.
FM_RETURN_REASON_NAMES = {
    0: "none",
    1: "candidate dropped - rider moved",
    2: "candidate dropped - positions untrustworthy",
    3: "candidate dropped - auto-return turned off",
    4: "proof confirmed - RETURN entered",
    5: "proof confirmed but declined - COG-only heading, use RTM",
    6: "RETURN arrived at the stop radius",
    7: "RETURN cancelled - rider moving",
    8: "RETURN timed out (60 s of motion)",
    9: "RETURN fault -> STOPPING",
    10: "RETURN / candidate ended - Follow-Me left or yielded",
    11: "RETURN cancelled - rider steered",
}

# BREmote_V2_Rx.h: FM_LOG_GATE_* bit defines, in bit order. Bit 1 = the condition held on that
# tick. Only these 16 bits are named today; any OTHER bit found set in fm_gate_flags is reported
# dynamically as "bit_N" (see decode_gate_flags()) rather than silently dropped.
FM_LOG_GATE_BITS: list[tuple[int, str]] = [
    (0, "thr_held"),
    (1, "fault_ok"),
    (2, "speed_ok"),
    (3, "dist_ok"),
    (4, "sep_latched"),
    (5, "diverge"),
    (6, "pivoting"),
    (7, "in_grace"),
    (8, "heading_disagree"),
    (9, "fade_bypass"),
    (10, "transit"),
    (11, "return_candidate"),
    (12, "proof_ok"),
    (13, "needs_dengage"),
    (14, "yield_to_rtm"),
    (15, "return_window"),
]
FM_LOG_GATE_BIT_NAMES = [name for _, name in FM_LOG_GATE_BITS]

# The gate-bit edges the timeline calls out by name (task spec: separation latch, needs-D_engage,
# return candidate, return window, yield-to-RTM, thr_held).
TIMELINE_GATE_BITS = ["sep_latched", "needs_dengage", "return_candidate", "return_window", "yield_to_rtm", "thr_held"]


# ============================================================
# FIELD TABLE - one entry per struct field, in DECLARATION ORDER.
#
# 'fmt'          - Python struct format char for this field (binary decode).
# 'csv_col'      - the column name the FIRMWARE'S OWN CSV uses for this field (LOG_CSV_HEADER_*
#                  in BREmote_V2_Rx.h). This is the on-disk/on-wire contract for CSV INPUT.
# 'name'         - the column name THIS TOOL uses in its own expanded output. Differs from
#                  csv_col only for the "_dxN" raw columns the firmware leaves unscaled (that is
#                  the whole point of "expanded": every field gets real units here).
# 'raw_scale'    - divide the raw struct value by this to get the engineering value (binary
#                  path). None = no scaling. A few fields need a *10 instead of a /10 (ERPM,
#                  cog_age_ms_div10); those use raw_scale=0.1 so value/0.1 == value*10.
# 'sentinel_raw' - the raw struct value that means "no data" (None = no sentinel).
# 'csv_prescaled'- True if the FIRMWARE'S CSV already applied raw_scale (i.e. the CSV column
#                  holds the final engineering value, e.g. motor_current_A, ERPM, speed_kmh).
#                  False if the firmware's CSV prints the RAW (unscaled) integer, in which case
#                  this tool applies raw_scale itself after parsing the CSV text as an int -
#                  exactly the columns whose csv_col still carries a "_dxN" suffix.
# 'sentinel_val' - for csv_prescaled columns only: the ALREADY-SCALED sentinel value the
#                  firmware's CSV prints for "no data" (e.g. -1.0 for a 0xFFFF distance).
# 'skip'         - True for padding fields that exist only to keep struct offsets correct
#                  (fm_pad) - consumed by struct.unpack, never emitted as a column.
# ============================================================

def F(name, fmt, csv_col=None, raw_scale=None, sentinel_raw=None,
      csv_prescaled=False, sentinel_val=None, unit="", skip=False):
    return {
        "name": name,
        "fmt": fmt,
        "csv_col": csv_col if csv_col is not None else name,
        "raw_scale": raw_scale,
        "sentinel_raw": sentinel_raw,
        "csv_prescaled": csv_prescaled,
        "sentinel_val": sentinel_val,
        "unit": unit,
        "skip": skip,
    }


# ---- VescLogData (L3, 59 bytes) - BREmote_V2_Rx.h ~:1039-1088 ----
L3_FIELDS = [
    F("timestamp_ms", "I", csv_prescaled=True, unit="ms"),
    F("motor_current_A", "h", raw_scale=100.0, csv_prescaled=True, unit="A"),
    F("battery_current_A", "h", raw_scale=100.0, csv_prescaled=True, unit="A"),
    F("duty_cycle_%", "b", csv_prescaled=True, unit="%"),
    F("voltage_V", "H", raw_scale=10.0, csv_prescaled=True, unit="V"),
    F("ERPM", "h", raw_scale=0.1, csv_prescaled=True, unit="erpm"),          # struct stores ERPM/10
    F("temp_mos_C", "b", csv_prescaled=True, unit="C"),
    F("fault_code", "B", csv_prescaled=True, unit=""),
    F("speed_kmh", "H", raw_scale=10.0, csv_prescaled=True, unit="km/h"),
    F("latitude", "f", csv_prescaled=True, unit="deg"),
    F("longitude", "f", csv_prescaled=True, unit="deg"),
    F("datetime_unix", "I", csv_prescaled=True, unit="unix_s"),
    F("thr_received", "B", csv_prescaled=True, unit="0-255"),
    F("rtm_source", "B", csv_prescaled=True, unit="code"),
    F("rtm_confidence", "B", csv_prescaled=True, unit="code"),
    F("rtm_rx_active", "B", csv_prescaled=True, unit="bool"),
    F("gps_phase_b_ok", "B", csv_prescaled=True, unit="bool"),
    F("rtm_steer_override", "B", csv_prescaled=True, unit="0-255"),
    # -- firmware's CSV leaves these raw (dx10 / sentinel ints); this tool expands them --
    F("rtm_heading_chosen_deg", "h", csv_col="rtm_heading_chosen_dx10",
      raw_scale=10.0, sentinel_raw=-1, unit="deg"),
    F("compass_live_deg", "H", csv_col="compass_live_dx10",
      raw_scale=10.0, sentinel_raw=0xFFFF, unit="deg"),
    F("compass_snap_deg", "H", csv_col="compass_snap_dx10",
      raw_scale=10.0, sentinel_raw=0xFFFF, unit="deg"),
    F("snap_age_s", "H", csv_col="snap_age_s", sentinel_raw=0xFFFF, unit="s"),
    F("gps_course_deg", "H", csv_col="gps_course_dx10",
      raw_scale=10.0, sentinel_raw=0xFFFF, unit="deg"),
    F("cog_age_ms", "H", csv_col="cog_age_ms_div10",
      raw_scale=0.1, sentinel_raw=0xFFFF, unit="ms"),                        # struct stores ms/10
    F("heading_error_deg", "h", csv_col="heading_error_dx10",
      raw_scale=10.0, sentinel_raw=0x7FFF, unit="deg"),
    F("d_error_deg_s", "h", csv_col="d_error_dx10",
      raw_scale=10.0, sentinel_raw=0x7FFF, unit="deg/s"),
    F("remote_error", "B", csv_col="remote_error", csv_prescaled=True, unit="code"),
    F("effective_steer", "B", csv_col="effective_steer", csv_prescaled=True, unit="0-255"),
    F("tx_distance_m", "H", csv_col="tx_distance_m",
      raw_scale=10.0, sentinel_raw=0xFFFF, csv_prescaled=True, sentinel_val=-1.0, unit="m"),
    F("rssi_dbm", "h", csv_col="rssi_dbm",
      sentinel_raw=0x7FFF, csv_prescaled=True, sentinel_val=-999, unit="dBm"),
    F("snr_db", "h", csv_col="snr_db",
      raw_scale=10.0, sentinel_raw=0x7FFF, csv_prescaled=True, sentinel_val=-99.0, unit="dB"),
]

# ---- VescLogDataL4 diagnostics block (bytes 59-64) - the historical 65 B "L4_DIAG" record ----
L4_DIAG_EXTRA_FIELDS = [
    F("gps_sent_per_s", "B", csv_prescaled=True, unit="/s"),
    F("cog_frozen_s", "B", csv_prescaled=True, unit="s"),          # 255 = no COG value ever seen
    F("mux_err_cnt", "H", csv_prescaled=True, unit="count"),
    F("loop_max_ms", "H", csv_prescaled=True, unit="ms"),
]

# ---- Follow-Me audit block (bytes 65-82) - present only in the current 83 B "L4" record ----
L4_FM_EXTRA_FIELDS = [
    F("fm_gate_flags", "I", csv_prescaled=True, unit="bitfield"),   # kept RAW - see decode_gate_flags()
    F("fm_distance_m", "H", csv_col="fm_distance_m",
      raw_scale=10.0, sentinel_raw=0xFFFF, csv_prescaled=True, sentinel_val=-1.0, unit="m"),
    F("fm_d_engage_m", "H", csv_col="fm_d_engage_m",
      raw_scale=10.0, sentinel_raw=0xFFFF, csv_prescaled=True, sentinel_val=-1.0, unit="m"),
    F("fm_rider_speed_kmh", "H", csv_col="fm_rider_speed_kmh",
      raw_scale=10.0, csv_prescaled=True, unit="km/h"),
    F("fm_sep_fix_count", "B", csv_prescaled=True, unit="count"),
    F("fm_mode", "B", csv_prescaled=True, unit="code"),
    F("fm_state", "B", csv_prescaled=True, unit="code"),
    F("fm_block_reason", "B", csv_prescaled=True, unit="code"),
    F("fm_throttle_cap", "B", csv_prescaled=True, unit="0-255"),
    F("fm_station_deg", "h", csv_col="fm_station_deg", raw_scale=10.0, csv_prescaled=True, unit="deg"),
    F("_fm_pad", "B", skip=True),
]


def _layout(level: int, name: str, fields: list[dict]) -> dict:
    struct_fmt = "<" + "".join(f["fmt"] for f in fields)
    record_size = struct.calcsize(struct_fmt)
    return {
        "level": level,
        "name": name,
        "fields": fields,
        "struct_fmt": struct_fmt,
        "record_size": record_size,
        "csv_header": ",".join(f["csv_col"] for f in fields if not f["skip"]),
    }


LAYOUT_L3 = _layout(3, "L3", L3_FIELDS)
LAYOUT_L4_DIAG = _layout(4, "L4_DIAG", L3_FIELDS + L4_DIAG_EXTRA_FIELDS)
LAYOUT_L4 = _layout(4, "L4", L3_FIELDS + L4_DIAG_EXTRA_FIELDS + L4_FM_EXTRA_FIELDS)

# Table-driven: keyed by on-disk record size. A future record (e.g. an 85 or 87 byte layout once
# the P2 station-fade fields go live) is one new _layout() call and one new dict entry here.
RECORD_LAYOUTS: dict[int, dict] = {
    LAYOUT_L3["record_size"]: LAYOUT_L3,
    LAYOUT_L4_DIAG["record_size"]: LAYOUT_L4_DIAG,
    LAYOUT_L4["record_size"]: LAYOUT_L4,
}
assert LAYOUT_L3["record_size"] == 59
assert LAYOUT_L4_DIAG["record_size"] == 65
assert LAYOUT_L4["record_size"] == 83

# The three CSV headers the firmware itself can print (BREmote_V2_Rx.h LOG_CSV_HEADER_L3 /
# _L4_DIAG / _L4). Copied verbatim so CSV-input detection is an exact match against what the
# device actually sends - the CSV column order IS the input contract. A unit test asserts these
# stay byte-identical to the csv_header the field table above generates, so the table can never
# silently drift from the firmware macros it was built from.
LOG_CSV_HEADER_L3 = (
    "timestamp_ms,motor_current_A,battery_current_A,duty_cycle_%,voltage_V,ERPM,temp_mos_C,"
    "fault_code,speed_kmh,latitude,longitude,datetime_unix,thr_received,rtm_source,rtm_confidence,"
    "rtm_rx_active,gps_phase_b_ok,rtm_steer_override,rtm_heading_chosen_dx10,compass_live_dx10,"
    "compass_snap_dx10,snap_age_s,gps_course_dx10,cog_age_ms_div10,heading_error_dx10,d_error_dx10,"
    "remote_error,effective_steer,tx_distance_m,rssi_dbm,snr_db"
)
LOG_CSV_HEADER_L4_DIAG = LOG_CSV_HEADER_L3 + ",gps_sent_per_s,cog_frozen_s,mux_err_cnt,loop_max_ms"
LOG_CSV_HEADER_L4 = LOG_CSV_HEADER_L4_DIAG + (
    ",fm_gate_flags,fm_distance_m,fm_d_engage_m,fm_rider_speed_kmh,fm_sep_fix_count,fm_mode,"
    "fm_state,fm_block_reason,fm_throttle_cap,fm_station_deg"
)

CSV_HEADER_TO_LAYOUT = {
    LOG_CSV_HEADER_L3: LAYOUT_L3,
    LOG_CSV_HEADER_L4_DIAG: LAYOUT_L4_DIAG,
    LOG_CSV_HEADER_L4: LAYOUT_L4,
}


# ============================================================
# DECODE - binary struct values and CSV text, both into the same canonical dict shape
# ============================================================

def raw_to_engineering(field: dict, raw_value) -> Optional[float | int]:
    """Binary-decode path: a struct.unpack() value -> engineering value, or None for sentinel."""
    if field["sentinel_raw"] is not None and raw_value == field["sentinel_raw"]:
        return None
    scale = field["raw_scale"]
    return raw_value if scale is None else raw_value / scale


def csv_text_to_engineering(field: dict, text: str) -> Optional[float | int]:
    """CSV-decode path: one cell of device CSV text -> engineering value, or None for sentinel."""
    text = text.strip()
    try:
        value: float | int = int(text)
    except ValueError:
        value = float(text)

    if field["csv_prescaled"]:
        sentinel = field["sentinel_val"]
        if sentinel is not None and float(value) == float(sentinel):
            return None
        return value

    # Firmware's CSV printed the RAW integer for this column - apply the same sentinel/scale
    # rule as the binary path.
    raw_value = int(value)
    if field["sentinel_raw"] is not None and raw_value == field["sentinel_raw"]:
        return None
    scale = field["raw_scale"]
    return raw_value if scale is None else raw_value / scale


def decode_gate_flags(flags: int) -> dict[str, bool]:
    """fm_gate_flags -> one boolean per named FM_LOG_GATE_* bit, plus bit_N for anything else
    found set that is not one of the 16 bits currently defined (BREmote_V2_Rx.h)."""
    out: dict[str, bool] = {}
    known_bits = {b for b, _ in FM_LOG_GATE_BITS}
    for bit, name in FM_LOG_GATE_BITS:
        out[name] = bool(flags & (1 << bit))
    for bit in range(32):
        if bit in known_bits:
            continue
        if flags & (1 << bit):
            out[f"bit_{bit}"] = True
    return out


def add_decoded_columns(rec: dict) -> dict:
    """Append the human-readable name/bit columns the expanded CSV and timeline both use."""
    if "rtm_source" in rec:
        rec["rtm_source_name"] = RTM_SOURCE_NAMES.get(rec["rtm_source"], f"unknown({rec['rtm_source']})")
    if "rtm_confidence" in rec:
        rec["rtm_confidence_name"] = RTM_CONFIDENCE_NAMES.get(rec["rtm_confidence"], f"unknown({rec['rtm_confidence']})")
    if "fm_mode" in rec:
        rec["fm_mode_name"] = FM_MODE_NAMES.get(rec["fm_mode"], f"unknown({rec['fm_mode']})")
    if "fm_state" in rec:
        rec["fm_state_name"] = FM_STATE_NAMES.get(rec["fm_state"], f"unknown({rec['fm_state']})")
    if "fm_block_reason" in rec:
        rec["fm_block_reason_name"] = FM_STOP_REASON_NAMES.get(rec["fm_block_reason"], f"unknown({rec['fm_block_reason']})")
    if "fm_gate_flags" in rec and rec["fm_gate_flags"] is not None:
        rec.update(decode_gate_flags(int(rec["fm_gate_flags"])))
    return rec


def build_canonical_from_raw(layout: dict, raw_values: tuple) -> dict:
    rec: dict[str, Any] = {"_level": layout["level"], "_record_size": layout["record_size"]}
    for field, raw in zip(layout["fields"], raw_values):
        if field["skip"]:
            continue
        rec[field["name"]] = raw_to_engineering(field, raw)
    return add_decoded_columns(rec)


def build_canonical_from_csv_row(layout: dict, row: list[str]) -> dict:
    rec: dict[str, Any] = {"_level": layout["level"], "_record_size": None}
    for field, text in zip(layout["fields"], row):
        if field["skip"]:
            continue
        rec[field["name"]] = csv_text_to_engineering(field, text)
    return add_decoded_columns(rec)


# ============================================================
# FILE READERS
# ============================================================

class LogFormatError(Exception):
    """Raised for a bad magic/header or an unrecognized CSV header - the two cases the CLI
    must exit non-zero for."""


def _pick_layout_for_record_size(record_size: int) -> tuple[dict, int]:
    """Return (layout, decode_size). Exact match uses the layout as-is. An unrecognized
    record_size decodes the largest known layout that fits inside it (a future firmware adding
    fields tail-appends, per the FM audit block's own design note), with the remainder ignored
    and a warning printed once. Smaller than the smallest known layout cannot be decoded at all."""
    layout = RECORD_LAYOUTS.get(record_size)
    if layout is not None:
        return layout, record_size

    known_sizes = sorted(RECORD_LAYOUTS)
    smaller_or_equal = [s for s in known_sizes if s <= record_size]
    if not smaller_or_equal:
        raise LogFormatError(
            f"record_size={record_size} is smaller than the smallest known record "
            f"({known_sizes[0]} bytes, {RECORD_LAYOUTS[known_sizes[0]]['name']}) - cannot decode."
        )
    chosen_size = smaller_or_equal[-1]
    chosen = RECORD_LAYOUTS[chosen_size]
    print(
        f"warning: unrecognized record_size={record_size}; decoding the known {chosen['name']} "
        f"prefix ({chosen_size} bytes) and ignoring the trailing {record_size - chosen_size} "
        f"byte(s). Add a new RECORD_LAYOUTS entry once this record size is documented.",
        file=sys.stderr,
    )
    return chosen, chosen_size


def iter_binary_records(path: str):
    with open(path, "rb") as f:
        header_bytes = f.read(LOG_FILE_HEADER_SIZE)
        if len(header_bytes) < LOG_FILE_HEADER_SIZE:
            raise LogFormatError(f"{path}: file is smaller than the {LOG_FILE_HEADER_SIZE}-byte log header.")

        magic, format_ver, log_level, record_size = struct.unpack(LOG_FILE_HEADER_FMT, header_bytes)
        if magic != LOG_FILE_MAGIC:
            raise LogFormatError(
                f"{path}: bad magic (0x{magic:08X}, expected 0x{LOG_FILE_MAGIC:08X}) - "
                "this file predates the self-describing log header, or is corrupt."
            )
        if format_ver != LOG_FILE_FORMAT_VER:
            raise LogFormatError(
                f"{path}: header format_ver={format_ver}, this tool understands format_ver={LOG_FILE_FORMAT_VER}."
            )

        layout, decode_size = _pick_layout_for_record_size(record_size)

        while True:
            rec_bytes = f.read(record_size)
            if len(rec_bytes) < record_size:
                break  # truncated tail (power cut mid-write) - stop cleanly, same as the firmware readers
            raw_values = struct.unpack(layout["struct_fmt"], rec_bytes[:decode_size])
            yield build_canonical_from_raw(layout, raw_values)


def iter_csv_records(path: str):
    with open(path, "r", newline="", encoding="utf-8", errors="replace") as f:
        header_line = f.readline().rstrip("\r\n")
        layout = CSV_HEADER_TO_LAYOUT.get(header_line)
        if layout is None:
            raise LogFormatError(
                f"{path}: first line does not match any known BREmote log CSV header "
                "(L3 / L4_DIAG / L4). Not a device log CSV, or from a different firmware era."
            )

        expected_cols = len(layout["fields"]) - sum(1 for f in layout["fields"] if f["skip"])
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            # A user may have pasted the whole serial capture, markers included - skip those,
            # and any short/aborted trailing marker line, without treating either as an error.
            if row[0].startswith("==="):
                continue
            if len(row) != expected_cols:
                print(f"warning: {path}: skipping malformed row (expected {expected_cols} columns, got {len(row)})", file=sys.stderr)
                continue
            yield build_canonical_from_csv_row(layout, row)


def is_binary_log(path: str) -> bool:
    with open(path, "rb") as f:
        head = f.read(4)
    if len(head) < 4:
        return False
    (magic,) = struct.unpack("<I", head)
    return magic == LOG_FILE_MAGIC


def load_records(path: str) -> list[dict]:
    if is_binary_log(path):
        return list(iter_binary_records(path))
    return list(iter_csv_records(path))


# ============================================================
# OUTPUT 1 - EXPANDED CSV
# ============================================================

def csv_field_order(level: int) -> list[str]:
    """Base columns (in struct/CSV order) + the decoded columns this tool adds, for one level."""
    if level == 3:
        base = [f["name"] for f in L3_FIELDS if not f["skip"]]
        extra = ["rtm_source_name", "rtm_confidence_name"]
        return base + extra

    base = [f["name"] for f in LAYOUT_L4_DIAG["fields"] if not f["skip"]]
    extra = ["rtm_source_name", "rtm_confidence_name"]
    return base + extra


def csv_field_order_l4_full() -> list[str]:
    base = [f["name"] for f in LAYOUT_L4["fields"] if not f["skip"]]
    extra = ["rtm_source_name", "rtm_confidence_name", "fm_mode_name", "fm_state_name", "fm_block_reason_name"]
    extra += FM_LOG_GATE_BIT_NAMES
    return base + extra


def format_csv_value(value) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return f"{round(value, 3):g}"
    return str(value)


def write_expanded_csv(records: list[dict], out_path: str) -> None:
    if not records:
        with open(out_path, "w", newline="", encoding="utf-8") as f:
            f.write("")
        return

    has_fm = any(r.get("fm_gate_flags") is not None for r in records)
    level = max(r["_level"] for r in records)
    fieldnames = csv_field_order_l4_full() if (level == 4 and has_fm) else csv_field_order(level)

    # Any UNKNOWN gate bit (bit_N) discovered at runtime gets its own trailing column too, so a
    # firmware that starts using a currently-reserved bit does not lose the information.
    dynamic_bits = sorted(
        {k for r in records for k in r if k.startswith("bit_") and k not in fieldnames},
        key=lambda s: int(s.split("_")[1]),
    )
    fieldnames = fieldnames + dynamic_bits

    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore", restval="")
        writer.writeheader()
        for rec in records:
            writer.writerow({k: format_csv_value(rec.get(k)) for k in fieldnames})


# ============================================================
# OUTPUT 2 - TIMELINE
# ============================================================

def _fmt_or_na(value, unit="") -> str:
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.1f}{unit}"
    return f"{value}{unit}"


def _ts_str(rec: dict, t0_ms: int) -> str:
    t_rel = (rec["timestamp_ms"] - t0_ms) / 1000.0
    out = f"T+{t_rel:7.1f}s"
    dt_unix = rec.get("datetime_unix")
    if dt_unix:
        try:
            clock = datetime.fromtimestamp(dt_unix, tz=timezone.utc).strftime("%H:%M:%S")
            out += f" [{clock}Z]"
        except (OverflowError, OSError, ValueError):
            pass
    return out


def generate_timeline(records: list[dict]) -> list[str]:
    """Row-to-row change list: FM state transitions, RTM active/ended edges, the gate-bit edges
    that matter for a return-to-rider diagnosis, non-zero fault stops, throttle-cap drops into
    the align/approach range, and one summary line per RETURN episode."""
    lines: list[str] = []
    if not records:
        return lines

    t0_ms = records[0]["timestamp_ms"]
    prev: Optional[dict] = None
    ret_episode: Optional[dict] = None  # open RETURN episode being tracked

    def close_return_episode(rec: dict) -> None:
        nonlocal ret_episode
        if ret_episode is None:
            return
        duration_s = (rec["timestamp_ms"] - ret_episode["entry_ts_ms"]) / 1000.0
        lines.append(
            "    RETURN episode: entry T+{entry:.1f}s, entry_dist={ed}, min_dist={md}, "
            "duration={dur:.1f}s, exit_state={ex}, exit_reason=not logged in this record version".format(
                entry=ret_episode["entry_t_rel"],
                ed=_fmt_or_na(ret_episode["entry_dist"], "m"),
                md=_fmt_or_na(ret_episode["min_dist"], "m"),
                dur=duration_s,
                ex=rec.get("fm_state_name", "?"),
            )
        )
        ret_episode = None

    for rec in records:
        if prev is not None:
            # ---- FM state transitions ----
            if "fm_state" in rec and rec["fm_state"] != prev.get("fm_state"):
                lines.append(
                    f"{_ts_str(rec, t0_ms)} FM state: {prev.get('fm_state_name', '?')} -> {rec.get('fm_state_name', '?')}"
                )
                if rec["fm_state"] == 5:  # entering FM_RETURN
                    ret_episode = {
                        "entry_t_rel": (rec["timestamp_ms"] - t0_ms) / 1000.0,
                        "entry_ts_ms": rec["timestamp_ms"],
                        "entry_dist": rec.get("fm_distance_m"),
                        "min_dist": rec.get("fm_distance_m"),
                    }
                elif prev.get("fm_state") == 5:  # leaving FM_RETURN
                    close_return_episode(rec)

            # ---- RTM active/ended edge ----
            if "rtm_rx_active" in rec and rec["rtm_rx_active"] != prev.get("rtm_rx_active"):
                lines.append(f"{_ts_str(rec, t0_ms)} {'RTM active' if rec['rtm_rx_active'] else 'RTM ended'}")

            # ---- gate-bit edges that matter for a return-to-rider diagnosis ----
            for bit_name in TIMELINE_GATE_BITS:
                if bit_name in rec and bool(rec[bit_name]) != bool(prev.get(bit_name)):
                    verb = "set" if rec[bit_name] else "cleared"
                    lines.append(f"{_ts_str(rec, t0_ms)} gate {bit_name} {verb}")

            # ---- fault stop reason ----
            if "fm_block_reason" in rec and rec["fm_block_reason"] != prev.get("fm_block_reason"):
                if rec["fm_block_reason"]:
                    lines.append(f"{_ts_str(rec, t0_ms)} FM block reason: {rec.get('fm_block_reason_name')}")
                elif prev.get("fm_block_reason"):
                    lines.append(f"{_ts_str(rec, t0_ms)} FM block reason cleared (was {prev.get('fm_block_reason_name')})")

            # ---- align/approach throttle cap engagements ----
            if "fm_throttle_cap" in rec and rec["fm_throttle_cap"] != prev.get("fm_throttle_cap"):
                if rec["fm_throttle_cap"] is not None and rec["fm_throttle_cap"] <= 20:
                    lines.append(f"{_ts_str(rec, t0_ms)} FM throttle cap -> {rec['fm_throttle_cap']} (align/approach cap)")

        # track the minimum distance reached while inside an open RETURN episode
        if ret_episode is not None and rec.get("fm_distance_m") is not None:
            if ret_episode["min_dist"] is None or rec["fm_distance_m"] < ret_episode["min_dist"]:
                ret_episode["min_dist"] = rec["fm_distance_m"]

        prev = rec

    if ret_episode is not None:
        # file ended mid-RETURN (e.g. mid-run capture) - close it out against the last record
        close_return_episode(records[-1])

    return lines


# ============================================================
# OUTPUT 3 - SUMMARY
# ============================================================

def generate_summary(records: list[dict]) -> str:
    if not records:
        return "no records"

    t0_ms = records[0]["timestamp_ms"]
    t1_ms = records[-1]["timestamp_ms"]
    duration_s = (t1_ms - t0_ms) / 1000.0
    n = len(records)
    rate_hz = (n - 1) / duration_s if duration_s > 0 else 0.0

    return_episodes = 0
    rtm_activations = 0
    latch_sets = 0
    stops_by_reason: dict[str, int] = {}

    prev: Optional[dict] = None
    for rec in records:
        if prev is not None:
            if rec.get("fm_state") == 5 and prev.get("fm_state") != 5:
                return_episodes += 1
            if rec.get("rtm_rx_active") and not prev.get("rtm_rx_active"):
                rtm_activations += 1
            if rec.get("sep_latched") and not prev.get("sep_latched"):
                latch_sets += 1
            reason = rec.get("fm_block_reason")
            if reason and reason != prev.get("fm_block_reason"):
                name = rec.get("fm_block_reason_name", str(reason))
                stops_by_reason[name] = stops_by_reason.get(name, 0) + 1
        prev = rec

    lines = [
        f"session duration: {duration_s:.1f}s ({n} records)",
        f"log rate (measured): {rate_hz:.2f} Hz",
        f"RETURN episodes: {return_episodes}",
        f"RTM activations: {rtm_activations}",
        f"separation latch sets: {latch_sets}",
        "stops by reason:",
    ]
    if stops_by_reason:
        for name, count in sorted(stops_by_reason.items()):
            lines.append(f"  {name}: {count}")
    else:
        lines.append("  (none)")
    return "\n".join(lines)


# ============================================================
# CLI
# ============================================================

def filter_by_time(records: list[dict], from_s: Optional[float], to_s: Optional[float]) -> list[dict]:
    if not records or (from_s is None and to_s is None):
        return records
    t0_ms = records[0]["timestamp_ms"]
    out = []
    for rec in records:
        t_rel = (rec["timestamp_ms"] - t0_ms) / 1000.0
        if from_s is not None and t_rel < from_s:
            continue
        if to_s is not None and t_rel > to_s:
            continue
        out.append(rec)
    return out


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="bremote_log.py",
        description="Decode a BREmote RX on-board deep log (binary .log or device CSV) into an expanded CSV, a timeline, and/or a summary.",
    )
    parser.add_argument("file", help="raw binary .log file or device CSV output")
    parser.add_argument("--csv", metavar="OUT", help="write the expanded CSV to OUT")
    parser.add_argument("--timeline", action="store_true", help="print the human-readable timeline to stdout")
    parser.add_argument("--summary", action="store_true", help="print the session summary to stdout")
    parser.add_argument("--from", dest="from_s", type=float, default=None, metavar="S", help="only include records at or after T+S seconds")
    parser.add_argument("--to", dest="to_s", type=float, default=None, metavar="S", help="only include records at or before T+S seconds")
    args = parser.parse_args(argv)

    try:
        records = load_records(args.file)
    except LogFormatError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except (OSError, struct.error) as exc:
        print(f"error: {args.file}: {exc}", file=sys.stderr)
        return 1

    records = filter_by_time(records, args.from_s, args.to_s)

    want_csv = args.csv is not None
    want_timeline = args.timeline
    want_summary = args.summary
    if not (want_csv or want_timeline or want_summary):
        want_timeline = True  # default action: timeline to stdout

    if want_csv:
        write_expanded_csv(records, args.csv)
    if want_timeline:
        for line in generate_timeline(records):
            print(line)
    if want_summary:
        print(generate_summary(records))

    return 0


if __name__ == "__main__":
    sys.exit(main())
