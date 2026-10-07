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
    unpacked into one boolean column per named bit PLUS one integer column
    per multi-bit field riding inside it (2026-10-03: the two I2C
    enable-swap failure deltas in bits 23-30), and the numeric state
    codes (rtm_source, rtm_confidence, fm_mode, fm_state, fm_block_reason,
    fm_return_reason, rtm_phase) given readable names alongside the raw
    value.
  - a human-readable TIMELINE of state changes (FM state transitions, RTM
    active/ended edges, RTM phase transitions, the gate-bit edges that
    matter for a return-to-rider diagnosis, boost on/off, fault stops, and
    one summary line per RETURN episode).
  - a SUMMARY of counts and rates for the whole session.

Record layouts, field order, scaling and the CSV column contract are all
sourced from ``Source/V2_Integration_Rx/BREmote_V2_Rx.h`` (``VescLogData``,
``VescLogDataL4``, ``VescLogDataL5``, ``LogFileHeader``, the
``FM_LOG_GATE_*`` bit defines and the ``LOG_CSV_HEADER_*`` /
``logFormatCsvRow()`` macros) and ``Source/V2_Integration_Rx/RTMState.ino``
(``FmState``, ``FmStopReason``, ``FmReturnReason``, the ``rtm_phase`` codes
and the ``telemetry.fm_flags`` bit map). All structs there are
``__attribute__((packed))``, so byte offsets below follow field declaration
order with no alignment gaps - confirmed against the
``static_assert(sizeof(...) == N, ...)`` lines next to each struct and
cross-checked against the CSV formatter's own field order and offset
guards in ``logFormatCsvRow()`` / ``logCsvHeaderFor()``.

Six record tiers exist today, selected by on-disk record size (bytes ->
columns): 59 -> 31 (level 3), 65 -> 35 (level 4 diagnostics only), 83 -> 48
(level 4 + Follow-Me block), 85 -> 49 (+ raw rider speed; never shipped
alone, kept so a reader can still name it), 87 -> 51 (+ the two mixer
outputs - the current everyday level-4 record), 109 -> 65 (level 5,
"everything"). Format 2 (since the M-2 flash, 2026-10-02) adds 3 bytes to the
base record: 62 -> 33 (L3_V2), 90 -> 53 (L4_V2), 112 -> 67 (L5_V2), and since
2026-10-06 126 -> 76 (L5_VESC2: level 5 + the second VESC's telemetry, read
over CAN; -999 / empty = no data or stale). The record layout table (``RECORD_LAYOUTS``) is keyed by
record size, so a future record size is one new table entry, not a
rewrite; an unrecognized size decodes the largest known prefix that fits.

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
LOG_FILE_FORMAT_VER = 2              # current: 2 = the M-2 motor-gate block in the base record
# Both are READABLE. 1 is every file written before the M-2 flash (2026-10-02) and 2 is every
# file after it. The two differ by 3 bytes at the tail of the BASE record, which moves every
# block above byte 59 - record_size alone cannot express that, which is why the firmware bumped
# the version rather than relying on the per-file header. Keeping 1 here is what lets every CSV
# and .log already pulled off a board still parse.
LOG_FILE_FORMAT_VERS_SUPPORTED = (1, 2)
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

# RTMState.ino: enum FmReturnReason + fmReturnReasonName(). Sticky in the deep log as of the
# 2026-09-19 "DEEP LOG (A)" change (VescLogDataL4.fm_return_reason, was the fm_pad byte): 0 = no
# event since boot; during a RETURN leg it reads 4 (ENTERED); the row where fm_state leaves 5
# carries the exit reason (6/7/8/9/10/11); 1-3 = a candidate dropped before RETURN was entered.
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

# RTMState.ino: the classic-RTM rtm_phase codes copied into VescLogDataL5.rtm_phase (level 5
# only). "approach" outranks "run" but is outranked by "align" and "bootstrap-headless"; 6 and 7
# are one-tick/gate branches, not steady states.
RTM_PHASE_NAMES = {
    0: "inactive/disabled",
    1: "align",
    2: "run",
    3: "approach",
    4: "bootstrap_headless",
    5: "gate_stop",
    6: "trigger_released",
    7: "gate9_handoff",
}

# BREmote_V2_Rx.h: FM_LOG_GATE_* bit defines, in bit order. Bit 1 = the condition held on that
# tick. bits 0-15 are the original 2026-09-17 audit block; 16 is reserved for the steer-takeover
# side branch (not yet defined/set by any code path in this tree - documented here so this tool
# names it instead of falling back to "bit_16" the day that branch lands); 17/18 are the
# 2026-09-19 "DEEP LOG (A)" additions, set at PUBLISH time (fmPublishLogSnapshot), not gate
# verdicts. Any bit found set that is NOT in this table is reported dynamically as "bit_N".
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
    (16, "steer_takeover"),  # reserved for the steer-takeover branch; not set by any code path yet
    (17, "fm_aligning"),     # named to match the firmware's own standalone fm_aligning CSV column
    (18, "fm_boost"),        # named to match the firmware's own standalone fm_boost CSV column
    (31, "aim_outward"),     # front stations: PG-4 escape standing - aim is an outward waypoint (2026-10-05;
                             # bit 23 on the fm-stations-front branch, moved to 31 to clear the swap counters)
]
FM_LOG_GATE_BIT_NAMES = [name for _, name in FM_LOG_GATE_BITS]

# BREmote_V2_Rx.h: the MULTI-BIT fields that ride inside the same fm_gate_flags word. These are NOT
# flags - each is a small unsigned integer - so they are decoded by shift+mask into their own
# numeric column instead of a boolean, and their bits are suppressed from the generic "bit_N"
# fallback below so one field does not also appear as four anonymous set bits.
#   (shift, width, name)
# 2026-10-03: bits 23-30 are the two I2C enable-swap failure deltas. The RX drives both motors from
# ONE PPM output by swapping the optocoupler enables over I2C; a swap that loses the bus mutex
# leaves one motor being re-pulsed and starves the other entirely. Each field counts the FAILED
# swaps since the previous log row, per channel, saturating at 15.
# READ THEM THE RIGHT WAY ROUND: swap_fail_ch0_q4 counts failures on ticks where PWM0 was the
# ENABLED channel, so a non-zero ch0 means PWM0 kept its pulses and PWM1 was the STARVED one.
# Both read 0 on a healthy bus. Firmware-side cumulative totals are on ?diag.
# Bit 31 is the front-station aim_outward flag (single bit, decoded in FM_LOG_GATE_BITS above). The word is now full.
# NOT IN THIS TABLE, DELIBERATELY: bits 19-22, the manual pivot assist depth (2026-09-25). This
# tool has always reported them as the four anonymous columns bit_19..bit_22 and that behaviour is
# left exactly as it is here - adding it would change existing output columns, which is outside the
# scope of the 2026-10-03 swap-failure change. The fix, when someone wants it, is one line:
#     (19, 4, "pivot_assist_q4"),
FM_LOG_GATE_FIELDS: list[tuple[int, int, str]] = [
    (23, 4, "swap_fail_ch0_q4"),   # failed ch0 enable swaps this row, 0-15 (2026-10-03)
    (27, 4, "swap_fail_ch1_q4"),   # failed ch1 enable swaps this row, 0-15 (2026-10-03)
]
FM_LOG_GATE_FIELD_NAMES = [name for _, _, name in FM_LOG_GATE_FIELDS]

# RTMState.ino: telemetry.fm_flags bit map (the byte the RX sends the remote every tick), copied
# verbatim into VescLogDataL5.fm_flags_sent. Bits 4-6 are free/reserved (not yet assigned).
FM_FLAGS_SENT_BITS: list[tuple[int, str]] = [
    (0, "armed"),
    (1, "engaged"),
    (2, "armed_not_ready"),
    (3, "fault_sticky"),
    (7, "effective_auto_return"),
]
FM_FLAGS_SENT_BIT_NAMES = [f"fm_flags_{name}" for _, name in FM_FLAGS_SENT_BITS]

# The gate-bit edges the timeline calls out by name (task spec: separation latch, needs-D_engage,
# return candidate, return window, yield-to-RTM, thr_held). fm_boost gets its own dedicated
# on/off wording in the timeline (see generate_timeline()), not this generic "gate X set" form.
TIMELINE_GATE_BITS = ["sep_latched", "needs_dengage", "return_candidate", "return_window", "yield_to_rtm", "thr_held"]

# Layout tier order, lightest to richest - used to pick which tier's column set an expanded CSV
# should use when a file's records are not all the same size (should not happen in practice: the
# level is latched once per FILE by createNewLogFile()), and by the CLI's file-type detection.
TIER_ORDER = ["L3", "L4_DIAG", "L4_83", "L4_RAW", "L4", "L5"]
# 2026-10-06: the format-2 (post-M-2) layouts were never added here, so write_expanded_csv() raised
# ValueError on EVERY file written since the M-2 flash. Appended after the format-1 names - a file
# never mixes the two formats, so their relative order only matters within each group.
TIER_ORDER += ["L3_V2", "L4_V2", "L5_V2"]
TIER_ORDER += ["L5_VESC2"]   # 2026-10-06: level 5 + the VESC 2 block (126 B)
# Tiers that carry the Follow-Me block (decoded gate/state columns) and the level-5 block, by name.
FM_BLOCK_TIERS = ("L4_83", "L4_RAW", "L4", "L5", "L4_V2", "L5_V2", "L5_VESC2")
L5_BLOCK_TIERS = ("L5", "L5_V2", "L5_VESC2")


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
#                  cog_age_ms_div10, rider_fix_age_div10); those use raw_scale=0.1 so
#                  value/0.1 == value*10.
# 'sentinel_raw' - the raw struct value that means "no data" (None = no sentinel).
# 'csv_prescaled'- True if the FIRMWARE'S CSV already applied raw_scale (i.e. the CSV column
#                  holds the final engineering value, e.g. motor_current_A, ERPM, speed_kmh).
#                  False if the firmware's CSV prints the RAW (unscaled) integer, in which case
#                  this tool applies raw_scale itself after parsing the CSV text as an int -
#                  exactly the columns whose csv_col still carries a "_dxN" suffix.
# 'sentinel_val' - for csv_prescaled columns only: the ALREADY-SCALED sentinel value the
#                  firmware's CSV prints for "no data" (e.g. -1.0 for a 0xFFFF distance).
# 'skip'         - True for padding fields that exist only to keep struct offsets correct -
#                  consumed by struct.unpack, never emitted as a column. None of today's fields
#                  need this (the last one that did, fm_pad, became fm_return_reason on
#                  2026-09-19), kept for a future struct that reserves a byte again.
#
# NOTE on "virtual" columns: fm_aligning and fm_boost are real, standalone columns in the
# firmware's own CSV (LOG_CSV_HEADER_L4_83 onward) but are NOT separate struct bytes - they are
# bits 17/18 of fm_gate_flags, printed a second time so a reader never has to mask the flag word.
# This tool does not give them field-table entries: it derives them from fm_gate_flags via
# decode_gate_flags() for BOTH binary and CSV input (one source of truth), and splices their
# names into the generated csv_header at the right position (see _layout()'s virtual_after).
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

# ---- M-2 motor-gate block (bytes 59-61), appended to the BASE record 2026-10-02 ----
# Deliberately in the base: the tiers are cumulative and levels 0-3 all record as level 3, so a
# field here appears at EVERY log level, 0 through 5. The firmware's reasoning was that an incident
# is never re-runnable at a higher log level.
#
# ctrl_pkt_age_ms is millis() - last_control_packet at log time. 0xFFFE is a CAP, not a sentinel
# ("65.5 s or more of control silence is one state"), so it is passed through as a real number
# rather than mapped to -1 - turning a cap into "unknown" would throw away the fact that the gate
# had been shut for a long time, which is the whole point of the column.
#
# motor_gate_open is calcPWM()'s own verdict, so it carries PWM_active too - something the age
# alone cannot show. 1 = pulses reaching the ESCs, 0 = nothing leaving the board.
L3_GATE_EXTRA_FIELDS = [
    F("ctrl_pkt_age_ms", "H", csv_prescaled=True, unit="ms"),
    F("motor_gate_open", "B", csv_prescaled=True, unit="1=open"),
]

# The base record as written from 2026-10-02 onward (62 B).
L3_FIELDS_V2 = L3_FIELDS + L3_GATE_EXTRA_FIELDS

# ---- VescLogDataL4 diagnostics block (bytes 59-64) - the historical 65 B "L4_DIAG" record ----
L4_DIAG_EXTRA_FIELDS = [
    F("gps_sent_per_s", "B", csv_prescaled=True, unit="/s"),
    F("cog_frozen_s", "B", csv_prescaled=True, unit="s"),          # 255 = no COG value ever seen
    F("mux_err_cnt", "H", csv_prescaled=True, unit="count"),
    F("loop_max_ms", "H", csv_prescaled=True, unit="ms"),
]

# ---- Follow-Me audit block (bytes 65-82) - the 83 B "L4_83" tier. fm_return_reason (byte 82) was
#      the fm_pad byte until the 2026-09-19 "DEEP LOG (A)" change; same offset, same size. ----
L4_83_EXTRA_FIELDS = [
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
    F("fm_return_reason", "B", csv_prescaled=True, unit="code"),   # 2026-09-19: was fm_pad
]

# ---- (B) raw rider speed (byte 83-84) - the 85 B "L4_RAW" tier. Never shipped alone by
#      createNewLogFile() (level 4 always writes the full 87 B record) - kept as its own tier so
#      a reader can still name a file that happens to be exactly this size. ----
L4_RAW_EXTRA_FIELDS = [
    F("fm_rider_raw_kmh", "H", csv_col="fm_rider_raw_kmh",
      raw_scale=10.0, sentinel_raw=0xFFFF, csv_prescaled=True, sentinel_val=-1.0, unit="km/h"),
]

# ---- (C) the two mixer outputs (bytes 85-86) - the 87 B "L4" tier: today's everyday deep log. ----
L4_MOTORS_EXTRA_FIELDS = [
    F("motor0_cmd", "B", csv_prescaled=True, unit="0-255"),
    F("motor1_cmd", "B", csv_prescaled=True, unit="0-255"),
]

# ---- Level-5 "everything" block (bytes 87-108) - the 109 B "L5" tier. ----
L5_EXTRA_FIELDS = [
    F("rider_lat", "f", csv_prescaled=True, unit="deg"),
    F("rider_lng", "f", csv_prescaled=True, unit="deg"),
    F("rider_fix_seq", "H", csv_prescaled=True, unit="count"),
    F("rider_fix_age_ms", "H", csv_col="rider_fix_age_ms",
      raw_scale=0.1, sentinel_raw=0xFFFF, csv_prescaled=True, sentinel_val=-1, unit="ms"),
    F("rtm_approach_cap", "B", csv_prescaled=True, unit="0-255"),
    F("rtm_phase", "B", csv_prescaled=True, unit="code"),
    F("align_cap", "B", csv_prescaled=True, unit="0-255"),
    F("align_influence", "B", csv_prescaled=True, unit="%"),
    F("mix_influence", "B", csv_prescaled=True, unit="%"),
    F("fm_return_override", "B", csv_prescaled=True, unit="code"),
    F("fm_flags_sent", "B", csv_prescaled=True, unit="bitfield"),
    F("fm_keepalive_age_s", "B", csv_col="fm_keepalive_age_s",
      raw_scale=10.0, sentinel_raw=0xFF, csv_prescaled=True, sentinel_val=-1.0, unit="s"),
    F("l5_rsvd_takeover_active", "B", csv_prescaled=True, unit="reserved"),
    F("l5_rsvd_takeover_end", "B", csv_prescaled=True, unit="reserved"),
]

# ---- VESC 2 block (bytes 112-125) - the 126 B "L5_VESC2" tier, 2026-10-06. ----
# The second motor controller, read by the RX over CAN through VESC 1 (COMM_FORWARD_CAN, 1 Hz).
# Appended at the TAIL of the largest record, so format_ver stays 2 and the new record_size is
# what says these columns are present - a 112 B file is still the plain L5_V2 tier.
# Same units and clamps as VESC 1's base-record columns. FRESHNESS: the firmware writes the "no
# data" sentinel in EVERY value field when VESC 2 has never answered or its last reply is older
# than 2500 ms, so stale data never decodes as live; vesc2_age_ms is always the real age (sentinel
# only for "never"). In the firmware's CSV every N/A prints -999 (outside every physical range).
VESC2_EXTRA_FIELDS = [
    F("vesc2_age_ms", "H", sentinel_raw=0xFFFF, csv_prescaled=True, sentinel_val=-999, unit="ms"),
    F("vesc2_motor_current_A", "h", raw_scale=100.0, sentinel_raw=0x7FFF,
      csv_prescaled=True, sentinel_val=-999.0, unit="A"),
    F("vesc2_battery_current_A", "h", raw_scale=100.0, sentinel_raw=0x7FFF,
      csv_prescaled=True, sentinel_val=-999.0, unit="A"),
    F("vesc2_duty_cycle_%", "b", sentinel_raw=0x7F, csv_prescaled=True, sentinel_val=-999, unit="%"),
    F("vesc2_voltage_V", "H", raw_scale=10.0, sentinel_raw=0xFFFF,
      csv_prescaled=True, sentinel_val=-999.0, unit="V"),
    F("vesc2_ERPM", "h", raw_scale=0.1, sentinel_raw=0x7FFF,
      csv_prescaled=True, sentinel_val=-999, unit="erpm"),                   # struct stores ERPM/10
    F("vesc2_temp_mos_C", "b", sentinel_raw=0x7F, csv_prescaled=True, sentinel_val=-999, unit="C"),
    F("vesc2_temp_motor_C", "b", sentinel_raw=0x7F, csv_prescaled=True, sentinel_val=-999, unit="C"),
    F("vesc2_fault_code", "B", sentinel_raw=0xFF, csv_prescaled=True, sentinel_val=-999, unit="code"),
]

# fm_aligning / fm_boost are spliced into the CSV header right after fm_return_reason in every
# tier from L4_83 up - see the field-table note above for why they have no field-table entry.
_VIRTUAL_GATE_COLUMNS = {"fm_return_reason": ["fm_aligning", "fm_boost"]}


def _layout(level: int, name: str, fields: list[dict], virtual_after: Optional[dict[str, list[str]]] = None) -> dict:
    struct_fmt = "<" + "".join(f["fmt"] for f in fields)
    record_size = struct.calcsize(struct_fmt)
    real_cols = [f["csv_col"] for f in fields if not f["skip"]]
    cols = []
    for col in real_cols:
        cols.append(col)
        if virtual_after and col in virtual_after:
            cols.extend(virtual_after[col])
    return {
        "level": level,
        "name": name,
        "fields": fields,
        "struct_fmt": struct_fmt,
        "record_size": record_size,
        "csv_header": ",".join(cols),
        "csv_header_cols": cols,
    }


LAYOUT_L3 = _layout(3, "L3", L3_FIELDS)
LAYOUT_L4_DIAG = _layout(4, "L4_DIAG", L3_FIELDS + L4_DIAG_EXTRA_FIELDS)
LAYOUT_L4_83 = _layout(4, "L4_83", L3_FIELDS + L4_DIAG_EXTRA_FIELDS + L4_83_EXTRA_FIELDS,
                       virtual_after=_VIRTUAL_GATE_COLUMNS)
LAYOUT_L4_RAW = _layout(4, "L4_RAW", L3_FIELDS + L4_DIAG_EXTRA_FIELDS + L4_83_EXTRA_FIELDS + L4_RAW_EXTRA_FIELDS,
                        virtual_after=_VIRTUAL_GATE_COLUMNS)
LAYOUT_L4 = _layout(4, "L4",
                    L3_FIELDS + L4_DIAG_EXTRA_FIELDS + L4_83_EXTRA_FIELDS + L4_RAW_EXTRA_FIELDS + L4_MOTORS_EXTRA_FIELDS,
                    virtual_after=_VIRTUAL_GATE_COLUMNS)
LAYOUT_L5 = _layout(5, "L5",
                    L3_FIELDS + L4_DIAG_EXTRA_FIELDS + L4_83_EXTRA_FIELDS + L4_RAW_EXTRA_FIELDS
                    + L4_MOTORS_EXTRA_FIELDS + L5_EXTRA_FIELDS,
                    virtual_after=_VIRTUAL_GATE_COLUMNS)

# ---- the post-M-2 record sizes (2026-10-02). Only these three are written by the current
#      firmware; the six above remain for every file recorded before the flash. ----
LAYOUT_L3_V2 = _layout(3, "L3_V2", L3_FIELDS_V2)
LAYOUT_L4_V2 = _layout(4, "L4_V2",
                       L3_FIELDS_V2 + L4_DIAG_EXTRA_FIELDS + L4_83_EXTRA_FIELDS
                       + L4_RAW_EXTRA_FIELDS + L4_MOTORS_EXTRA_FIELDS,
                       virtual_after=_VIRTUAL_GATE_COLUMNS)
LAYOUT_L5_V2 = _layout(5, "L5_V2",
                       L3_FIELDS_V2 + L4_DIAG_EXTRA_FIELDS + L4_83_EXTRA_FIELDS
                       + L4_RAW_EXTRA_FIELDS + L4_MOTORS_EXTRA_FIELDS + L5_EXTRA_FIELDS,
                       virtual_after=_VIRTUAL_GATE_COLUMNS)
# ---- 2026-10-06: level 5 + the VESC 2 block (126 B), still format_ver 2. ----
LAYOUT_L5_VESC2 = _layout(5, "L5_VESC2",
                          L3_FIELDS_V2 + L4_DIAG_EXTRA_FIELDS + L4_83_EXTRA_FIELDS
                          + L4_RAW_EXTRA_FIELDS + L4_MOTORS_EXTRA_FIELDS + L5_EXTRA_FIELDS
                          + VESC2_EXTRA_FIELDS,
                          virtual_after=_VIRTUAL_GATE_COLUMNS)

LAYOUT_BY_NAME = {lay["name"]: lay for lay in (LAYOUT_L3, LAYOUT_L4_DIAG, LAYOUT_L4_83, LAYOUT_L4_RAW, LAYOUT_L4, LAYOUT_L5,
                                               LAYOUT_L3_V2, LAYOUT_L4_V2, LAYOUT_L5_V2, LAYOUT_L5_VESC2)}

# Table-driven: keyed by on-disk record size. A future record (e.g. the steer-takeover branch
# claiming bit 16 and its own bytes) is one new _layout() call and one new dict entry here.
RECORD_LAYOUTS: dict[int, dict] = {lay["record_size"]: lay for lay in LAYOUT_BY_NAME.values()}

# BREmote_V2_Rx.h LOG_CSV_HEADER_L3 as of the M-2 change, copied verbatim. If the firmware adds a
# base column and this reader is not updated, this assert fires on import rather than letting a
# mis-aligned decode through.
_FW_CSV_HEADER_L3_V2 = (
    "timestamp_ms,motor_current_A,battery_current_A,duty_cycle_%,voltage_V,ERPM,temp_mos_C,"
    "fault_code,speed_kmh,latitude,longitude,datetime_unix,thr_received,rtm_source,rtm_confidence,"
    "rtm_rx_active,gps_phase_b_ok,rtm_steer_override,rtm_heading_chosen_dx10,compass_live_dx10,"
    "compass_snap_dx10,snap_age_s,gps_course_dx10,cog_age_ms_div10,heading_error_dx10,d_error_dx10,"
    "remote_error,effective_steer,tx_distance_m,rssi_dbm,snr_db,ctrl_pkt_age_ms,motor_gate_open"
)
assert LAYOUT_L3_V2["csv_header"] == _FW_CSV_HEADER_L3_V2, (
    "generated level-3 header has drifted from the firmware macro - generated "
    + LAYOUT_L3_V2["csv_header"] + " / firmware " + _FW_CSV_HEADER_L3_V2
)
# 59/65/83/85/87/109 = format_ver 1 (pre-2026-10-02). 62/90/112 = format_ver 2, the M-2 base.
# 126 = format_ver 2 + the VESC 2 block at the tail of level 5 (2026-10-06).
assert RECORD_LAYOUTS.keys() == {59, 65, 83, 85, 87, 109, 62, 90, 112, 126}, sorted(RECORD_LAYOUTS)

# BREmote_V2_Rx.h LOG_CSV_HEADER_L5_VESC2 is LOG_CSV_HEADER_L5 plus this suffix, copied verbatim.
# (A test also reads the macro straight out of the firmware header, so the two cannot drift.)
_FW_CSV_SUFFIX_VESC2 = (
    ",vesc2_age_ms,vesc2_motor_current_A,vesc2_battery_current_A,vesc2_duty_cycle_%,vesc2_voltage_V,"
    "vesc2_ERPM,vesc2_temp_mos_C,vesc2_temp_motor_C,vesc2_fault_code"
)
assert LAYOUT_L5_VESC2["csv_header"] == LAYOUT_L5_V2["csv_header"] + _FW_CSV_SUFFIX_VESC2, (
    "generated VESC 2 header has drifted from the firmware macro"
)

# The six CSV headers the firmware itself can print (BREmote_V2_Rx.h LOG_CSV_HEADER_L3 /
# _L4_DIAG / _L4_83 / _L4_RAW / _L4 / _L5). Copied verbatim so CSV-input detection is an exact
# match against what the device actually sends - the CSV column order IS the input contract. A
# unit test asserts these stay byte-identical to the csv_header the field table above generates,
# so the table can never silently drift from the firmware macros it was built from.
LOG_CSV_HEADER_L3 = (
    "timestamp_ms,motor_current_A,battery_current_A,duty_cycle_%,voltage_V,ERPM,temp_mos_C,"
    "fault_code,speed_kmh,latitude,longitude,datetime_unix,thr_received,rtm_source,rtm_confidence,"
    "rtm_rx_active,gps_phase_b_ok,rtm_steer_override,rtm_heading_chosen_dx10,compass_live_dx10,"
    "compass_snap_dx10,snap_age_s,gps_course_dx10,cog_age_ms_div10,heading_error_dx10,d_error_dx10,"
    "remote_error,effective_steer,tx_distance_m,rssi_dbm,snr_db"
)
LOG_CSV_HEADER_L4_DIAG = LOG_CSV_HEADER_L3 + ",gps_sent_per_s,cog_frozen_s,mux_err_cnt,loop_max_ms"
LOG_CSV_HEADER_L4_83 = LOG_CSV_HEADER_L4_DIAG + (
    ",fm_gate_flags,fm_distance_m,fm_d_engage_m,fm_rider_speed_kmh,fm_sep_fix_count,fm_mode,"
    "fm_state,fm_block_reason,fm_throttle_cap,fm_station_deg,fm_return_reason,fm_aligning,fm_boost"
)
LOG_CSV_HEADER_L4_RAW = LOG_CSV_HEADER_L4_83 + ",fm_rider_raw_kmh"
LOG_CSV_HEADER_L4 = LOG_CSV_HEADER_L4_RAW + ",motor0_cmd,motor1_cmd"
LOG_CSV_HEADER_L5 = LOG_CSV_HEADER_L4 + (
    ",rider_lat,rider_lng,rider_fix_seq,rider_fix_age_ms,rtm_approach_cap,rtm_phase,align_cap,"
    "align_influence,mix_influence,fm_return_override,fm_flags_sent,fm_keepalive_age_s,"
    "l5_rsvd_takeover_active,l5_rsvd_takeover_end"
)

CSV_HEADER_TO_LAYOUT = {
    LOG_CSV_HEADER_L3: LAYOUT_L3,
    LOG_CSV_HEADER_L4_DIAG: LAYOUT_L4_DIAG,
    LOG_CSV_HEADER_L4_83: LAYOUT_L4_83,
    LOG_CSV_HEADER_L4_RAW: LAYOUT_L4_RAW,
    LOG_CSV_HEADER_L4: LAYOUT_L4,
    LOG_CSV_HEADER_L5: LAYOUT_L5,
    # The post-M-2 headers. Taken from the GENERATED layouts rather than hand-typed: the generator
    # is the same field table that produced the byte-identical headers above, and the assert below
    # checks its level-3 output against the firmware macro copied verbatim, so any drift between
    # this table and BREmote_V2_Rx.h is caught at import time instead of in the field.
    LAYOUT_L3_V2["csv_header"]: LAYOUT_L3_V2,
    LAYOUT_L4_V2["csv_header"]: LAYOUT_L4_V2,
    LAYOUT_L5_V2["csv_header"]: LAYOUT_L5_V2,
    LAYOUT_L5_VESC2["csv_header"]: LAYOUT_L5_VESC2,   # 2026-10-06
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


def decode_bitfield(value: int, bit_table: list[tuple[int, str]], prefix: str = "", bit_range: int = 32) -> dict[str, bool]:
    """Generic bitfield decoder: one boolean per named bit in bit_table, plus "{prefix}bit_N" for
    any OTHER bit found set within bit_range that is not in the table."""
    out: dict[str, bool] = {}
    known_bits = {b for b, _ in bit_table}
    for bit, name in bit_table:
        out[f"{prefix}{name}"] = bool(value & (1 << bit))
    for bit in range(bit_range):
        if bit in known_bits:
            continue
        if value & (1 << bit):
            out[f"{prefix}bit_{bit}"] = True
    return out


def decode_gate_flags(flags: int) -> dict[str, Any]:
    """fm_gate_flags -> one boolean per named FM_LOG_GATE_* bit (including fm_aligning/fm_boost,
    the two 2026-09-19 additions, and steer_takeover, reserved), plus the multi-bit numeric fields
    in FM_LOG_GATE_FIELDS, plus bit_N for anything else.

    2026-10-03: bits 23-30 carry the two I2C enable-swap failure deltas, which are 4-bit NUMBERS,
    not flags. They are extracted by shift+mask into their own integer columns and their bits are
    removed from the generic bit_N fallback, so one 4-bit field cannot also surface as four
    anonymous set bits."""
    out: dict[str, Any] = dict(decode_bitfield(flags, FM_LOG_GATE_BITS))
    for shift, width, name in FM_LOG_GATE_FIELDS:
        out[name] = (flags >> shift) & ((1 << width) - 1)
        for bit in range(shift, shift + width):
            out.pop(f"bit_{bit}", None)
    return out


def decode_fm_flags_sent(value: int) -> dict[str, bool]:
    """fm_flags_sent (level 5 only) -> one boolean per named telemetry.fm_flags bit, prefixed
    fm_flags_ to keep them visually and namespace-distinct from the fm_gate_flags bit columns."""
    return decode_bitfield(value, FM_FLAGS_SENT_BITS, prefix="fm_flags_", bit_range=8)


def add_decoded_columns(rec: dict) -> dict:
    """Append the human-readable name/bit columns the expanded CSV and timeline both use."""
    if "rtm_source" in rec and rec["rtm_source"] is not None:
        rec["rtm_source_name"] = RTM_SOURCE_NAMES.get(rec["rtm_source"], f"unknown({rec['rtm_source']})")
    if "rtm_confidence" in rec and rec["rtm_confidence"] is not None:
        rec["rtm_confidence_name"] = RTM_CONFIDENCE_NAMES.get(rec["rtm_confidence"], f"unknown({rec['rtm_confidence']})")
    if "fm_mode" in rec and rec["fm_mode"] is not None:
        rec["fm_mode_name"] = FM_MODE_NAMES.get(rec["fm_mode"], f"unknown({rec['fm_mode']})")
    if "fm_state" in rec and rec["fm_state"] is not None:
        rec["fm_state_name"] = FM_STATE_NAMES.get(rec["fm_state"], f"unknown({rec['fm_state']})")
    if "fm_block_reason" in rec and rec["fm_block_reason"] is not None:
        rec["fm_block_reason_name"] = FM_STOP_REASON_NAMES.get(rec["fm_block_reason"], f"unknown({rec['fm_block_reason']})")
    if "fm_return_reason" in rec and rec["fm_return_reason"] is not None:
        rec["fm_return_reason_name"] = FM_RETURN_REASON_NAMES.get(rec["fm_return_reason"], f"unknown({rec['fm_return_reason']})")
    if "rtm_phase" in rec and rec["rtm_phase"] is not None:
        rec["rtm_phase_name"] = RTM_PHASE_NAMES.get(rec["rtm_phase"], f"unknown({rec['rtm_phase']})")
    if "fm_gate_flags" in rec and rec["fm_gate_flags"] is not None:
        rec.update(decode_gate_flags(int(rec["fm_gate_flags"])))
    if "fm_flags_sent" in rec and rec["fm_flags_sent"] is not None:
        rec.update(decode_fm_flags_sent(int(rec["fm_flags_sent"])))
    return rec


def build_canonical_from_raw(layout: dict, raw_values: tuple) -> dict:
    rec: dict[str, Any] = {"_level": layout["level"], "_record_size": layout["record_size"], "_layout_name": layout["name"]}
    for field, raw in zip(layout["fields"], raw_values):
        if field["skip"]:
            continue
        rec[field["name"]] = raw_to_engineering(field, raw)
    return add_decoded_columns(rec)


def build_canonical_from_csv_row(layout: dict, row: list[str]) -> dict:
    # Named lookup, not positional zip: the device CSV carries two "virtual" columns
    # (fm_aligning, fm_boost) that have no field-table entry (see the note above the field
    # table), so column POSITION in the row does not line up 1:1 with layout["fields"]. Building
    # a name -> text dict from the layout's own known header makes every lookup robust to that.
    cell = dict(zip(layout["csv_header_cols"], row))
    rec: dict[str, Any] = {"_level": layout["level"], "_record_size": None, "_layout_name": layout["name"]}
    for field in layout["fields"]:
        if field["skip"]:
            continue
        text = cell.get(field["csv_col"])
        if text is None:
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
    record_size decodes the largest known layout that fits inside it (matching the firmware's
    own logCsvHeaderFor() range-tiering: anything between two known sizes reads as the smaller
    tier, extra bytes ignored), with a warning printed once. Smaller than the smallest known
    layout cannot be decoded at all."""
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
        if format_ver not in LOG_FILE_FORMAT_VERS_SUPPORTED:
            raise LogFormatError(
                f"{path}: header format_ver={format_ver}, this tool understands "
                f"format_ver={' and '.join(str(v) for v in LOG_FILE_FORMAT_VERS_SUPPORTED)}."
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
                "(L3 / L4_DIAG / L4_83 / L4_RAW / L4 / L5). Not a device log CSV, or from a "
                "different firmware era."
            )

        expected_cols = len(layout["csv_header_cols"])
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

def csv_field_order_for_layout(layout_name: str) -> list[str]:
    """Base columns (struct/CSV order) + the decoded columns this tool adds, for one tier."""
    layout = LAYOUT_BY_NAME[layout_name]
    base = [f["name"] for f in layout["fields"] if not f["skip"]]
    extra = ["rtm_source_name", "rtm_confidence_name"]
    if layout_name in FM_BLOCK_TIERS:
        extra += ["fm_mode_name", "fm_state_name", "fm_block_reason_name", "fm_return_reason_name"]
        extra += FM_LOG_GATE_BIT_NAMES
        # 2026-10-03: the multi-bit numeric fields that ride in the same word (the two enable-swap
        # failure deltas). Appended AFTER the flag columns so every column that existed before this
        # change keeps its position - anything consuming this CSV by index still works.
        extra += FM_LOG_GATE_FIELD_NAMES
    if layout_name in L5_BLOCK_TIERS:
        extra += ["rtm_phase_name"]
        extra += FM_FLAGS_SENT_BIT_NAMES
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

    layout_name = max(records, key=lambda r: TIER_ORDER.index(r["_layout_name"]))["_layout_name"]
    fieldnames = csv_field_order_for_layout(layout_name)

    # Any UNKNOWN bit (bit_N / fm_flags_bit_N) discovered at runtime gets its own trailing column
    # too, so a firmware that starts using a currently-reserved bit does not lose the information.
    dynamic_bits = sorted(
        {k for r in records for k in r if (k.startswith("bit_") or k.startswith("fm_flags_bit_")) and k not in fieldnames}
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
    """Row-to-row change list: FM state transitions, RTM active/ended edges, RTM phase
    transitions (level 5), the gate-bit edges that matter for a return-to-rider diagnosis, boost
    on/off, non-zero fault stops, throttle-cap drops into the align/approach range, and one
    summary line per RETURN episode (including the real exit reason and the tightest motor0/
    motor1 split commanded during the episode's align sub-phase, where those columns exist)."""
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
        exit_reason = rec.get("fm_return_reason_name")
        if exit_reason is None:
            exit_reason = "not logged in this record version"
        lines.append(
            "    RETURN episode: entry T+{entry:.1f}s, entry_dist={ed}, min_dist={md}, "
            "duration={dur:.1f}s, exit_state={ex}, exit_reason={reason}, "
            "motor0_min={m0}, motor1_min={m1}".format(
                entry=ret_episode["entry_t_rel"],
                ed=_fmt_or_na(ret_episode["entry_dist"], "m"),
                md=_fmt_or_na(ret_episode["min_dist"], "m"),
                dur=duration_s,
                ex=rec.get("fm_state_name", "?"),
                reason=exit_reason,
                m0=_fmt_or_na(ret_episode["min_motor0"]),
                m1=_fmt_or_na(ret_episode["min_motor1"]),
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
                        "min_motor0": None,
                        "min_motor1": None,
                    }
                elif prev.get("fm_state") == 5:  # leaving FM_RETURN
                    close_return_episode(rec)

            # ---- RTM active/ended edge ----
            if "rtm_rx_active" in rec and rec["rtm_rx_active"] != prev.get("rtm_rx_active"):
                lines.append(f"{_ts_str(rec, t0_ms)} {'RTM active' if rec['rtm_rx_active'] else 'RTM ended'}")

            # ---- RTM phase transitions (level 5 only) ----
            if "rtm_phase" in rec and rec["rtm_phase"] != prev.get("rtm_phase"):
                lines.append(
                    f"{_ts_str(rec, t0_ms)} RTM phase: {prev.get('rtm_phase_name', '?')} -> {rec.get('rtm_phase_name', '?')}"
                )

            # ---- gate-bit edges that matter for a return-to-rider diagnosis ----
            for bit_name in TIMELINE_GATE_BITS:
                if bit_name in rec and bool(rec[bit_name]) != bool(prev.get(bit_name)):
                    verb = "set" if rec[bit_name] else "cleared"
                    lines.append(f"{_ts_str(rec, t0_ms)} gate {bit_name} {verb}")

            # ---- pivot-boost on/off ----
            if "fm_boost" in rec and bool(rec["fm_boost"]) != bool(prev.get("fm_boost")):
                lines.append(f"{_ts_str(rec, t0_ms)} {'boost on' if rec['fm_boost'] else 'boost off'}")

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

        # track the minimum distance reached, and the tightest motor0/motor1 split commanded
        # during the align sub-phase, while inside an open RETURN episode
        if ret_episode is not None:
            if rec.get("fm_distance_m") is not None:
                if ret_episode["min_dist"] is None or rec["fm_distance_m"] < ret_episode["min_dist"]:
                    ret_episode["min_dist"] = rec["fm_distance_m"]
            if rec.get("fm_aligning"):
                if rec.get("motor0_cmd") is not None:
                    if ret_episode["min_motor0"] is None or rec["motor0_cmd"] < ret_episode["min_motor0"]:
                        ret_episode["min_motor0"] = rec["motor0_cmd"]
                if rec.get("motor1_cmd") is not None:
                    if ret_episode["min_motor1"] is None or rec["motor1_cmd"] < ret_episode["min_motor1"]:
                        ret_episode["min_motor1"] = rec["motor1_cmd"]

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
