#!/usr/bin/env python3
"""
Unit tests for bremote_log.py. Stdlib unittest only - run with:

    python -m unittest Tools/logreader/test_bremote_log.py -v

or, from this directory:

    python test_bremote_log.py -v
"""

from __future__ import annotations

import os
import re
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import bremote_log as bl  # noqa: E402


# ============================================================
# helpers - build synthetic records straight from bl.RECORD_LAYOUTS, so the tests exercise
# EXACTLY the table the decoder reads, not a hand-rolled second copy of the layout.
# ============================================================

def _default_raw(fmt_char: str):
    return 0.0 if fmt_char == "f" else 0


def pack_record(layout: dict, raw_overrides: dict) -> bytes:
    """Build one raw record for `layout`, defaulting every field to 0 (0.0 for floats) and
    overriding by STRUCT field name (i.e. the raw on-wire value, not the engineering value)."""
    values = []
    for field in layout["fields"]:
        name = field["name"]
        values.append(raw_overrides.get(name, _default_raw(field["fmt"])))
    return struct.pack(layout["struct_fmt"], *values)


def write_binary_log(path: str, level: int, record_size: int, records: list[bytes],
                     format_ver: int = bl.LOG_FILE_FORMAT_VER, boot: tuple[int, int] = (7, 1)) -> None:
    """2026-10-06: format_ver is explicit now. The decoder picks a layout by (format_ver, record_size), so a
    test that builds a format-1 or format-2 record must say so - exactly as a real file's header does.
    2026-10-14: a format-4+ file carries the 8-byte boot block (boot_count, reset_reason) after the header,
    exactly as the RX writes it; `boot` sets it."""
    with open(path, "wb") as f:
        f.write(struct.pack(bl.LOG_FILE_HEADER_FMT, bl.LOG_FILE_MAGIC, format_ver, level, record_size))
        if format_ver >= 4:
            f.write(struct.pack(bl.LOG_FILE_BOOT_FMT, *boot))
        for rec in records:
            f.write(rec)


def gate(*names: str) -> int:
    """OR together the FM_LOG_GATE_* bits named, using the tool's own bit table."""
    bit_by_name = {name: bit for bit, name in bl.FM_LOG_GATE_BITS}
    flags = 0
    for name in names:
        flags |= 1 << bit_by_name[name]
    return flags


def fm_flags(*names: str) -> int:
    """OR together the fm_flags_sent bits named, using the tool's own bit table."""
    bit_by_name = {name: bit for bit, name in bl.FM_FLAGS_SENT_BITS}
    flags = 0
    for name in names:
        flags |= 1 << bit_by_name[name]
    return flags


def m_to_dx10(meters: float) -> int:
    return round(meters * 10)


# ============================================================
# 1. field table <-> firmware CSV header contract (drift detector) - all SIX tiers
# ============================================================

class TestFieldTableMatchesFirmwareCsvHeaders(unittest.TestCase):
    def test_l3_header_matches(self):
        self.assertEqual(bl.LAYOUT_L3["csv_header"], bl.LOG_CSV_HEADER_L3)

    def test_l4_diag_header_matches(self):
        self.assertEqual(bl.LAYOUT_L4_DIAG["csv_header"], bl.LOG_CSV_HEADER_L4_DIAG)

    def test_l4_83_header_matches(self):
        self.assertEqual(bl.LAYOUT_L4_83["csv_header"], bl.LOG_CSV_HEADER_L4_83)

    def test_l4_raw_header_matches(self):
        self.assertEqual(bl.LAYOUT_L4_RAW["csv_header"], bl.LOG_CSV_HEADER_L4_RAW)

    def test_l4_header_matches(self):
        self.assertEqual(bl.LAYOUT_L4["csv_header"], bl.LOG_CSV_HEADER_L4)

    def test_l5_header_matches(self):
        self.assertEqual(bl.LAYOUT_L5["csv_header"], bl.LOG_CSV_HEADER_L5)

    def test_record_sizes_match_static_asserts(self):
        # BREmote_V2_Rx.h: sizeof(VescLogData)==59, historical L4_DIAG==65,
        # L4_83==83 (historical, pre-2026-09-19), L4_RAW==85 (never shipped alone),
        # sizeof(VescLogDataL4)==87, sizeof(VescLogDataL5)==109
        self.assertEqual(bl.LAYOUT_L3["record_size"], 59)
        self.assertEqual(bl.LAYOUT_L4_DIAG["record_size"], 65)
        self.assertEqual(bl.LAYOUT_L4_83["record_size"], 83)
        self.assertEqual(bl.LAYOUT_L4_RAW["record_size"], 85)
        self.assertEqual(bl.LAYOUT_L4["record_size"], 87)
        self.assertEqual(bl.LAYOUT_L5["record_size"], 109)

    def test_column_counts_match_report(self):
        # 31/35/48/49/51/65 columns for 59/65/83/85/87/109 bytes
        self.assertEqual(len(bl.LAYOUT_L3["csv_header_cols"]), 31)
        self.assertEqual(len(bl.LAYOUT_L4_DIAG["csv_header_cols"]), 35)
        self.assertEqual(len(bl.LAYOUT_L4_83["csv_header_cols"]), 48)
        self.assertEqual(len(bl.LAYOUT_L4_RAW["csv_header_cols"]), 49)
        self.assertEqual(len(bl.LAYOUT_L4["csv_header_cols"]), 51)
        self.assertEqual(len(bl.LAYOUT_L5["csv_header_cols"]), 65)


# ============================================================
# 2. sentinel / scale decoding, both paths
# ============================================================

class TestSentinelAndScaleDecoding(unittest.TestCase):
    def _field(self, name: str) -> dict:
        by_name = {f["name"]: f for f in bl.LAYOUT_L5["fields"]}
        return by_name[name]

    def test_erpm_scale_binary(self):
        # struct stores ERPM/10; engineering value multiplies back by 10
        field = self._field("ERPM")
        self.assertEqual(bl.raw_to_engineering(field, 150), 1500.0)

    def test_rtm_heading_chosen_sentinel_binary(self):
        field = self._field("rtm_heading_chosen_deg")
        self.assertIsNone(bl.raw_to_engineering(field, -1))
        self.assertEqual(bl.raw_to_engineering(field, 900), 90.0)

    def test_compass_live_sentinel_binary(self):
        field = self._field("compass_live_deg")
        self.assertIsNone(bl.raw_to_engineering(field, 0xFFFF))
        self.assertEqual(bl.raw_to_engineering(field, 3600), 360.0)

    def test_tx_distance_sentinel_csv(self):
        field = self._field("tx_distance_m")
        self.assertIsNone(bl.csv_text_to_engineering(field, "-1.0"))
        self.assertEqual(bl.csv_text_to_engineering(field, "12.3"), 12.3)

    def test_rssi_sentinel_csv(self):
        field = self._field("rssi_dbm")
        self.assertIsNone(bl.csv_text_to_engineering(field, "-999"))
        self.assertEqual(bl.csv_text_to_engineering(field, "-72"), -72)

    def test_raw_dx10_column_csv(self):
        # firmware's own CSV leaves this one RAW (see csv_col="rtm_heading_chosen_dx10")
        field = self._field("rtm_heading_chosen_deg")
        self.assertIsNone(bl.csv_text_to_engineering(field, "-1"))
        self.assertEqual(bl.csv_text_to_engineering(field, "900"), 90.0)

    def test_fm_rider_raw_kmh_sentinel_binary(self):
        field = self._field("fm_rider_raw_kmh")
        self.assertIsNone(bl.raw_to_engineering(field, 0xFFFF))
        self.assertEqual(bl.raw_to_engineering(field, 42), 4.2)

    def test_fm_keepalive_age_s_sentinel_csv(self):
        field = self._field("fm_keepalive_age_s")
        self.assertIsNone(bl.csv_text_to_engineering(field, "-1.0"))
        self.assertEqual(bl.csv_text_to_engineering(field, "2.5"), 2.5)

    def test_rider_fix_age_ms_scale_binary(self):
        # struct stores ms/10; engineering value multiplies back by 10
        field = self._field("rider_fix_age_ms")
        self.assertIsNone(bl.raw_to_engineering(field, 0xFFFF))
        self.assertEqual(bl.raw_to_engineering(field, 250), 2500.0)


# ============================================================
# 3. fm_gate_flags and fm_flags_sent bit decoding
# ============================================================

class TestGateFlagsDecode(unittest.TestCase):
    def test_all_named_bits_present(self):
        bits = bl.decode_gate_flags(0)
        for _, name in bl.FM_LOG_GATE_BITS:
            self.assertIn(name, bits)
            self.assertFalse(bits[name])

    def test_known_bits_set(self):
        flags = gate("thr_held", "sep_latched")
        bits = bl.decode_gate_flags(flags)
        self.assertTrue(bits["thr_held"])
        self.assertTrue(bits["sep_latched"])
        self.assertFalse(bits["diverge"])

    def test_new_2026_09_19_bits(self):
        flags = gate("fm_aligning", "fm_boost")
        bits = bl.decode_gate_flags(flags)
        self.assertTrue(bits["fm_aligning"])
        self.assertTrue(bits["fm_boost"])
        self.assertFalse(bits["steer_takeover"])  # reserved, not set by any code path yet

    def test_unknown_bit_reported_dynamically(self):
        flags = (1 << 20)  # not one of the documented bits
        bits = bl.decode_gate_flags(flags)
        self.assertIn("bit_20", bits)
        self.assertTrue(bits["bit_20"])


class TestFmFlagsSentDecode(unittest.TestCase):
    def test_named_bits(self):
        bits = bl.decode_fm_flags_sent(fm_flags("armed", "engaged"))
        self.assertTrue(bits["fm_flags_armed"])
        self.assertTrue(bits["fm_flags_engaged"])
        self.assertFalse(bits["fm_flags_armed_not_ready"])
        self.assertFalse(bits["fm_flags_fault_sticky"])
        self.assertFalse(bits["fm_flags_effective_auto_return"])

    def test_unknown_bit_reported_dynamically(self):
        bits = bl.decode_fm_flags_sent(1 << 5)  # bits 4-6 are free/reserved
        self.assertIn("fm_flags_bit_5", bits)
        self.assertTrue(bits["fm_flags_bit_5"])


# ============================================================
# 4. binary decode roundtrip (single record), all tiers
# ============================================================

class TestBinaryDecodeRoundtrip(unittest.TestCase):
    def test_l4_83_record_decodes(self):
        raw = {
            "timestamp_ms": 1000,
            "voltage_V": 406,          # -> 40.6 V
            "speed_kmh": 123,          # -> 12.3 km/h
            "fm_state": 2,             # ACTIVE
            "fm_mode": 1,              # near_right
            "fm_gate_flags": gate("thr_held", "fault_ok"),
            "fm_distance_m": m_to_dx10(6.5),
            "fm_return_reason": 4,     # ENTERED
        }
        rec_bytes = pack_record(bl.LAYOUT_L4_83, raw)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "test.log")
            write_binary_log(path, 4, 83, [rec_bytes], format_ver=1)
            records = list(bl.iter_binary_records(path))
        self.assertEqual(len(records), 1)
        rec = records[0]
        self.assertEqual(rec["timestamp_ms"], 1000)
        self.assertAlmostEqual(rec["voltage_V"], 40.6, places=3)
        self.assertAlmostEqual(rec["speed_kmh"], 12.3, places=3)
        self.assertEqual(rec["fm_state_name"], "ACTIVE")
        self.assertEqual(rec["fm_mode_name"], "near_right")
        self.assertEqual(rec["fm_return_reason_name"], "proof confirmed - RETURN entered")
        self.assertTrue(rec["thr_held"])
        self.assertTrue(rec["fault_ok"])
        self.assertFalse(rec["sep_latched"])
        self.assertAlmostEqual(rec["fm_distance_m"], 6.5, places=3)

    def test_l4_87_record_decodes_motor_commands(self):
        raw = {"fm_state": 2, "motor0_cmd": 30, "motor1_cmd": 122, "fm_gate_flags": gate("fm_aligning")}
        rec_bytes = pack_record(bl.LAYOUT_L4, raw)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "test87.log")
            write_binary_log(path, 4, 87, [rec_bytes], format_ver=1)
            records = list(bl.iter_binary_records(path))
        self.assertEqual(len(records), 1)
        rec = records[0]
        self.assertEqual(rec["motor0_cmd"], 30)
        self.assertEqual(rec["motor1_cmd"], 122)
        self.assertTrue(rec["fm_aligning"])

    def test_l5_record_decodes_rider_position_and_phase(self):
        raw = {
            "rider_lat": 41.123456,
            "rider_lng": -87.654321,
            "rider_fix_seq": 42,
            "rtm_phase": 1,  # align
            "fm_flags_sent": fm_flags("armed", "engaged", "effective_auto_return"),
        }
        rec_bytes = pack_record(bl.LAYOUT_L5, raw)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "test109.log")
            write_binary_log(path, 5, 109, [rec_bytes], format_ver=1)
            records = list(bl.iter_binary_records(path))
        self.assertEqual(len(records), 1)
        rec = records[0]
        self.assertAlmostEqual(rec["rider_lat"], 41.123456, places=5)
        self.assertAlmostEqual(rec["rider_lng"], -87.654321, places=5)
        self.assertEqual(rec["rider_fix_seq"], 42)
        self.assertEqual(rec["rtm_phase_name"], "align")
        self.assertTrue(rec["fm_flags_armed"])
        self.assertTrue(rec["fm_flags_engaged"])
        self.assertTrue(rec["fm_flags_effective_auto_return"])
        self.assertFalse(rec["fm_flags_fault_sticky"])

    def test_bad_magic_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "bad.log")
            with open(path, "wb") as f:
                f.write(b"\x00\x00\x00\x00\x01\x03\x3B\x00")  # wrong magic, otherwise well-formed
            with self.assertRaises(bl.LogFormatError):
                list(bl.iter_binary_records(path))

    def test_unrecognized_record_size_decodes_known_prefix(self):
        # 88 bytes: between the 87 B "L4" tier and the 90 B "L4_V2" tier, so the largest known
        # prefix that fits is L4 (87 B). (2026-10-06: was 90, which stopped being "unrecognized"
        # when the M-2 base growth made 90 B the real L4_V2 size.)
        raw = {"fm_state": 3, "fm_mode": 2}
        rec_bytes = pack_record(bl.LAYOUT_L4, raw) + b"\x00"  # 1 trailing byte, size 88
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "future.log")
            write_binary_log(path, 4, 88, [rec_bytes], format_ver=1)
            records = list(bl.iter_binary_records(path))
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]["fm_state_name"], "HOLD")
        self.assertEqual(records[0]["fm_mode_name"], "behind")


# ============================================================
# 5. CSV input decoding
# ============================================================

class TestCsvInputDecoding(unittest.TestCase):
    def test_l3_csv_row(self):
        row = (
            "1000,0.50,1.20,10,40.6,1500,25,0,12.3,41.123456,-87.654321,1700000000,"
            "200,1,3,1,1,127,900,3600,65535,65535,900,65535,32767,32767,0,127,-1.0,-72,-99.0"
        )
        text = bl.LOG_CSV_HEADER_L3 + "\n" + row + "\n"
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "device.csv")
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
            records = list(bl.iter_csv_records(path))
        self.assertEqual(len(records), 1)
        rec = records[0]
        self.assertEqual(rec["timestamp_ms"], 1000)
        self.assertEqual(rec["rtm_source_name"], "GPS_COG")
        self.assertEqual(rec["rtm_confidence_name"], "HIGH")
        self.assertAlmostEqual(rec["rtm_heading_chosen_deg"], 90.0, places=3)
        self.assertAlmostEqual(rec["compass_live_deg"], 360.0, places=3)  # raw 3600, not the 65535 sentinel
        self.assertIsNone(rec["compass_snap_deg"])       # 65535 sentinel
        self.assertIsNone(rec["tx_distance_m"])          # -1.0 sentinel
        self.assertEqual(rec["rssi_dbm"], -72)           # a real value, not the -999 sentinel
        self.assertIsNone(rec["snr_db"])                 # -99.0 sentinel

    def test_l4_83_csv_row_with_virtual_gate_columns(self):
        # exercises the named (non-positional) CSV lookup: fm_aligning/fm_boost have no
        # field-table entry but ARE real columns in the device's own CSV.
        l3_row = (
            "1000,0.50,1.20,10,40.6,1500,25,0,12.3,41.123456,-87.654321,1700000000,"
            "200,1,3,1,1,127,900,3600,65535,65535,900,65535,32767,32767,0,127,-1.0,-72,-99.0"
        )
        diag = "5,10,0,3"
        fm = "131072,65.0,120.0,15.0,2,1,2,0,255,0.0,4,1,0"  # fm_gate_flags=131072=1<<17 (fm_aligning)
        row = ",".join([l3_row, diag, fm])
        text = bl.LOG_CSV_HEADER_L4_83 + "\n" + row + "\n"
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "device83.csv")
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
            records = list(bl.iter_csv_records(path))
        self.assertEqual(len(records), 1)
        rec = records[0]
        self.assertEqual(rec["fm_state_name"], "ACTIVE")
        self.assertEqual(rec["fm_return_reason_name"], "proof confirmed - RETURN entered")
        self.assertTrue(rec["fm_aligning"])   # derived from fm_gate_flags, not the literal cell
        self.assertFalse(rec["fm_boost"])

    def test_bad_header_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "notalog.csv")
            with open(path, "w", encoding="utf-8") as f:
                f.write("some,other,header\n1,2,3\n")
            with self.assertRaises(bl.LogFormatError):
                list(bl.iter_csv_records(path))


# ============================================================
# 6. timeline + summary over a full scripted scenario (87 B tier):
#    ARMED -> latch -> ACTIVE -> RETURN candidate -> RETURN -> HOLD
# ============================================================

class TestTimelineScenario(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        interval_ms = 333
        raws = [
            # 0: ARMED, trigger held
            dict(timestamp_ms=0, fm_state=1, fm_mode=1, rtm_rx_active=0,
                 fm_gate_flags=gate("thr_held"), fm_distance_m=m_to_dx10(15.0),
                 fm_throttle_cap=255, fm_block_reason=0),
            # 1: ARMED, separation latch sets
            dict(timestamp_ms=interval_ms, fm_state=1, fm_mode=1, rtm_rx_active=0,
                 fm_gate_flags=gate("thr_held", "sep_latched"), fm_distance_m=m_to_dx10(14.0),
                 fm_throttle_cap=255, fm_block_reason=0),
            # 2: ACTIVE, align cap engages
            dict(timestamp_ms=2 * interval_ms, fm_state=2, fm_mode=1, rtm_rx_active=0,
                 fm_gate_flags=gate("thr_held", "fault_ok", "speed_ok", "dist_ok", "sep_latched"),
                 fm_distance_m=m_to_dx10(8.0), fm_throttle_cap=15, fm_block_reason=0),
            # 3: ACTIVE, RETURN candidate opens (trigger released), cap released
            dict(timestamp_ms=3 * interval_ms, fm_state=2, fm_mode=1, rtm_rx_active=0,
                 fm_gate_flags=gate("fault_ok", "sep_latched", "return_candidate", "return_window"),
                 fm_distance_m=m_to_dx10(6.0), fm_throttle_cap=255, fm_block_reason=0),
            # 4: enters RETURN
            dict(timestamp_ms=4 * interval_ms, fm_state=5, fm_mode=1, rtm_rx_active=0,
                 fm_gate_flags=gate("proof_ok", "return_window"), fm_distance_m=m_to_dx10(5.5),
                 fm_throttle_cap=255, fm_block_reason=0, fm_return_reason=4),  # ENTERED
            # 5: still RETURN, closes further (this is the min distance of the episode)
            dict(timestamp_ms=5 * interval_ms, fm_state=5, fm_mode=1, rtm_rx_active=0,
                 fm_gate_flags=gate("proof_ok"), fm_distance_m=m_to_dx10(3.0),
                 fm_throttle_cap=255, fm_block_reason=0, fm_return_reason=4),
            # 6: arrives -> HOLD (exit reason ARRIVED=6, sticky, read on this row)
            dict(timestamp_ms=6 * interval_ms, fm_state=3, fm_mode=1, rtm_rx_active=0,
                 fm_gate_flags=0, fm_distance_m=m_to_dx10(2.5),
                 fm_throttle_cap=0, fm_block_reason=0, fm_return_reason=6),
        ]
        rec_bytes = [pack_record(bl.LAYOUT_L4, raw) for raw in raws]
        cls.tmpdir = tempfile.TemporaryDirectory()
        cls.path = os.path.join(cls.tmpdir.name, "scenario.log")
        write_binary_log(cls.path, 4, 87, rec_bytes, format_ver=1)
        cls.records = list(bl.iter_binary_records(cls.path))
        cls.timeline = bl.generate_timeline(cls.records)

    @classmethod
    def tearDownClass(cls):
        cls.tmpdir.cleanup()

    def test_seven_records_decoded(self):
        self.assertEqual(len(self.records), 7)

    def test_state_transitions_in_order(self):
        state_lines = [ln for ln in self.timeline if ln.startswith("T+") and "FM state:" in ln]
        joined = "\n".join(state_lines)
        self.assertIn("ARMED -> ACTIVE", joined)
        self.assertIn("ACTIVE -> RETURN", joined)
        self.assertIn("RETURN -> HOLD", joined)
        # order: ARMED->ACTIVE must appear before ACTIVE->RETURN, which must appear before RETURN->HOLD
        i1 = joined.index("ARMED -> ACTIVE")
        i2 = joined.index("ACTIVE -> RETURN")
        i3 = joined.index("RETURN -> HOLD")
        self.assertLess(i1, i2)
        self.assertLess(i2, i3)

    def test_sep_latched_edge_reported(self):
        self.assertTrue(any("gate sep_latched set" in ln for ln in self.timeline))

    def test_return_candidate_and_window_edges_reported(self):
        self.assertTrue(any("gate return_candidate set" in ln for ln in self.timeline))
        self.assertTrue(any("gate return_window set" in ln for ln in self.timeline))

    def test_throttle_cap_align_line_reported(self):
        self.assertTrue(any("FM throttle cap -> 15 (align/approach cap)" in ln for ln in self.timeline))

    def test_return_episode_summary_line_with_real_exit_reason(self):
        ep_lines = [ln for ln in self.timeline if "RETURN episode:" in ln]
        self.assertEqual(len(ep_lines), 1)
        line = ep_lines[0]
        self.assertIn("entry_dist=5.5m", line)
        self.assertIn("min_dist=3.0m", line)
        self.assertIn("exit_state=HOLD", line)
        # real exit reason now, not the "not logged" placeholder - read from the exit row (6)
        self.assertIn("exit_reason=RETURN arrived at the stop radius", line)
        self.assertNotIn("not logged in this record version", line)
        # duration: entered at T+1.332s (record 4), exited at T+1.998s (record 6) -> 0.666s
        self.assertIn("duration=0.7s", line)
        # no fm_aligning tick in this scenario -> both mins N/A
        self.assertIn("motor0_min=N/A", line)
        self.assertIn("motor1_min=N/A", line)

    def test_summary_counts(self):
        summary = bl.generate_summary(self.records)
        self.assertIn("RETURN episodes: 1", summary)
        self.assertIn("separation latch sets: 1", summary)
        self.assertIn("RTM activations: 0", summary)


class TestClosureCeilingCode11(unittest.TestCase):
    """2026-10-30: fm_block_reason 11 = the fast-closure ceiling bound cap 3 that tick. It is named, it is NOT a stop
    (kept out of "stops by reason"), and the summary counts its rows on their own line."""

    def test_code_11_named_counted_and_not_a_stop(self):
        interval_ms = 100
        reasons = [0, 11, 11, 0, 0, 11, 0, 3, 3, 0]   # two ceiling episodes (3 rows), then one TX_STALE pause
        raws = [dict(timestamp_ms=i * interval_ms, fm_state=2 if r != 3 else 3, fm_mode=1, rtm_rx_active=0,
                     fm_gate_flags=gate("thr_held"), fm_distance_m=m_to_dx10(20.0), fm_throttle_cap=255,
                     fm_block_reason=r)
                for i, r in enumerate(reasons)]
        rec_bytes = [pack_record(bl.LAYOUT_L4, raw) for raw in raws]
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "ceiling.log")
            write_binary_log(path, 4, 87, rec_bytes, format_ver=1)
            records = list(bl.iter_binary_records(path))
        self.assertEqual(records[1]["fm_block_reason_name"], "CLOSURE_CEILING (fast-closure ceiling bound cap 3)")
        summary = bl.generate_summary(records)
        self.assertIn("closure ceiling (code 11, not a stop): 3 of 10 rows (30.0%)", summary)
        self.assertIn("TX_STALE (rider GPS stale): 1", summary)
        self.assertNotIn("CLOSURE_CEILING (fast-closure ceiling bound cap 3): ", summary)


class TestWrongWayNetCodes(unittest.TestCase):
    """2026-10-31: R2-S, the wrong-way net. fm_block_reason 12 / 13 are steering ends (named and tallied as stops, like 7);
    14 is the report-only shadow net on manual riding - named, NOT a stop, counted on its own summary line. The names match
    the firmware's fmStopReasonName() table (RTMState.ino)."""

    def test_codes_12_13_14_named_and_14_not_a_stop(self):
        reasons = [0, 12, 0, 0, 13, 0, 14, 0, 14, 0]
        states = [2, 1, 1, 2, 1, 1, 1, 1, 1, 1]       # an end leaves Follow-Me ARMED on its own tick; 14 is on manual rows
        raws = [dict(timestamp_ms=i * 100, fm_state=st, fm_mode=1, rtm_rx_active=0, fm_gate_flags=gate("thr_held"),
                     fm_distance_m=m_to_dx10(20.0), fm_throttle_cap=255, fm_block_reason=r)
                for i, (r, st) in enumerate(zip(reasons, states))]
        rec_bytes = [pack_record(bl.LAYOUT_L4, raw) for raw in raws]
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "wrongway.log")
            write_binary_log(path, 4, 87, rec_bytes, format_ver=1)
            records = list(bl.iter_binary_records(path))
        self.assertEqual(records[1]["fm_block_reason_name"], "WRONG_WAY (steered one way, turned the other)")
        self.assertEqual(records[4]["fm_block_reason_name"], "POINTING_AWAY (pointed away 4 s, did not come round)")
        self.assertEqual(records[6]["fm_block_reason_name"], "SHADOW_WRONG_WAY (manual, report only)")
        summary = bl.generate_summary(records)
        self.assertIn("WRONG_WAY (steered one way, turned the other): 1", summary)
        self.assertIn("POINTING_AWAY (pointed away 4 s, did not come round): 1", summary)
        self.assertIn("wrong-way net shadow events (code 14, manual riding, not a stop): 2", summary)
        self.assertNotIn("SHADOW_WRONG_WAY (manual, report only): ", summary)

    def test_names_match_the_firmware_table(self):
        with open(_FW_RTM_PATH, "r", encoding="utf-8") as f:
            fw = f.read()
        for code, name in ((12, "WRONG_WAY"), (13, "POINTING_AWAY"), (14, "SHADOW_WRONG_WAY")):
            self.assertRegex(fw, rf"FM_STOP_{name}\s*=\s*{code},")
            self.assertIn(f'return "{bl.FM_STOP_REASON_NAMES[code]}";', fw)


# ============================================================
# 7. RETURN episode that exits STEERED (11), with a boost sub-phase and a motor0/motor1 split
#    (87 B tier) - the coordinator's specific follow-up scenario.
# ============================================================

class TestReturnEpisodeSteeredExitWithBoost(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        interval_ms = 333
        raws = [
            # 0: ACTIVE
            dict(timestamp_ms=0, fm_state=2, fm_mode=1,
                 fm_gate_flags=gate("thr_held", "fault_ok", "sep_latched"),
                 fm_distance_m=m_to_dx10(7.0)),
            # 1: enters RETURN, boost + aligning on, tight pivot (motor0/1 split)
            dict(timestamp_ms=interval_ms, fm_state=5, fm_mode=1,
                 fm_gate_flags=gate("proof_ok", "fm_aligning", "fm_boost"),
                 fm_distance_m=m_to_dx10(6.5), fm_return_reason=4,  # ENTERED
                 motor0_cmd=30, motor1_cmd=122),
            # 2: still aligning, tighter split (new min for both channels)
            dict(timestamp_ms=2 * interval_ms, fm_state=5, fm_mode=1,
                 fm_gate_flags=gate("proof_ok", "fm_aligning", "fm_boost"),
                 fm_distance_m=m_to_dx10(5.0), fm_return_reason=4,
                 motor0_cmd=10, motor1_cmd=140),
            # 3: aligned now, boost drops, running straight (not the align min anymore)
            dict(timestamp_ms=3 * interval_ms, fm_state=5, fm_mode=1,
                 fm_gate_flags=gate("proof_ok"),
                 fm_distance_m=m_to_dx10(4.0), fm_return_reason=4,
                 motor0_cmd=80, motor1_cmd=80),
            # 4: rider steers -> RETURN cancelled (STEERED=11) -> HOLD
            dict(timestamp_ms=4 * interval_ms, fm_state=3, fm_mode=1,
                 fm_gate_flags=0, fm_distance_m=m_to_dx10(3.8), fm_return_reason=11,
                 motor0_cmd=0, motor1_cmd=0),
        ]
        rec_bytes = [pack_record(bl.LAYOUT_L4, raw) for raw in raws]
        cls.tmpdir = tempfile.TemporaryDirectory()
        cls.path = os.path.join(cls.tmpdir.name, "steered.log")
        write_binary_log(cls.path, 4, 87, rec_bytes, format_ver=1)
        cls.records = list(bl.iter_binary_records(cls.path))
        cls.timeline = bl.generate_timeline(cls.records)

    @classmethod
    def tearDownClass(cls):
        cls.tmpdir.cleanup()

    def test_boost_on_off_edges(self):
        self.assertTrue(any(ln.endswith("boost on") for ln in self.timeline))
        self.assertTrue(any(ln.endswith("boost off") for ln in self.timeline))
        on_idx = next(i for i, ln in enumerate(self.timeline) if ln.endswith("boost on"))
        off_idx = next(i for i, ln in enumerate(self.timeline) if ln.endswith("boost off"))
        self.assertLess(on_idx, off_idx)

    def test_return_episode_exits_steered_with_motor_split(self):
        ep_lines = [ln for ln in self.timeline if "RETURN episode:" in ln]
        self.assertEqual(len(ep_lines), 1)
        line = ep_lines[0]
        self.assertIn("exit_state=HOLD", line)
        self.assertIn("exit_reason=RETURN cancelled - rider steered", line)
        # each channel's own minimum across the fm_aligning ticks only (records 1-2; record 3 is
        # not aligning and is excluded even though 80/80 is closer to centre than either motor1
        # value): motor0 bottoms out at record 2 (10), motor1 bottoms out at record 1 (122).
        self.assertIn("motor0_min=10", line)
        self.assertIn("motor1_min=122", line)


# ============================================================
# 8. level-5 timeline: rtm_phase transitions
# ============================================================

class TestRtmPhaseTimeline(unittest.TestCase):
    def test_phase_transition_reported_by_name(self):
        raws = [
            dict(timestamp_ms=0, rtm_phase=1),    # align
            dict(timestamp_ms=333, rtm_phase=2),  # run
        ]
        rec_bytes = [pack_record(bl.LAYOUT_L5, raw) for raw in raws]
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "phase.log")
            write_binary_log(path, 5, 109, rec_bytes, format_ver=1)
            records = list(bl.iter_binary_records(path))
            timeline = bl.generate_timeline(records)
        self.assertTrue(any("RTM phase: align -> run" in ln for ln in timeline))


# ============================================================
# 9. expanded CSV output
# ============================================================

class TestExpandedCsvOutput(unittest.TestCase):
    def test_write_expanded_csv_has_decoded_columns(self):
        raw = dict(timestamp_ms=0, fm_state=2, fm_mode=3, fm_gate_flags=gate("thr_held", "diverge"))
        rec_bytes = pack_record(bl.LAYOUT_L4_83, raw)
        with tempfile.TemporaryDirectory() as tmp:
            log_path = os.path.join(tmp, "one.log")
            write_binary_log(log_path, 4, 83, [rec_bytes], format_ver=1)
            records = list(bl.iter_binary_records(log_path))
            out_path = os.path.join(tmp, "expanded.csv")
            bl.write_expanded_csv(records, out_path)
            with open(out_path, "r", encoding="utf-8") as f:
                header = f.readline().strip()
                row = f.readline().strip()
        self.assertIn("fm_state_name", header)
        self.assertIn("fm_mode_name", header)
        self.assertIn("fm_return_reason_name", header)
        self.assertIn("thr_held", header)
        self.assertIn("diverge", header)
        self.assertIn("fm_gate_flags", header)  # raw value kept alongside the decoded bits
        cols = header.split(",")
        vals = row.split(",")
        as_dict = dict(zip(cols, vals))
        self.assertEqual(as_dict["fm_state_name"], "ACTIVE")
        self.assertEqual(as_dict["fm_mode_name"], "near_left")
        self.assertEqual(as_dict["thr_held"], "1")
        self.assertEqual(as_dict["diverge"], "1")
        self.assertEqual(as_dict["sep_latched"], "0")

    def test_write_expanded_csv_l5_has_phase_and_fm_flags_columns(self):
        raw = dict(rtm_phase=2, fm_flags_sent=fm_flags("armed", "engaged"))
        rec_bytes = pack_record(bl.LAYOUT_L5, raw)
        with tempfile.TemporaryDirectory() as tmp:
            log_path = os.path.join(tmp, "l5.log")
            write_binary_log(log_path, 5, 109, [rec_bytes], format_ver=1)
            records = list(bl.iter_binary_records(log_path))
            out_path = os.path.join(tmp, "expanded_l5.csv")
            bl.write_expanded_csv(records, out_path)
            with open(out_path, "r", encoding="utf-8") as f:
                header = f.readline().strip()
                row = f.readline().strip()
        as_dict = dict(zip(header.split(","), row.split(",")))
        self.assertEqual(as_dict["rtm_phase_name"], "run")
        self.assertEqual(as_dict["fm_flags_armed"], "1")
        self.assertEqual(as_dict["fm_flags_engaged"], "1")
        self.assertEqual(as_dict["fm_flags_fault_sticky"], "0")

    def test_write_expanded_csv_format2_layouts_do_not_crash(self):
        # 2026-10-06: the post-M-2 (format 2) layouts were missing from TIER_ORDER, so --csv raised
        # ValueError on every log written since the M-2 flash. Each one must now produce its
        # decoded columns like its format-1 counterpart.
        for layout, level in ((bl.LAYOUT_L3_V2, 3), (bl.LAYOUT_L4_V2, 4), (bl.LAYOUT_L5_V2, 5)):
            raw = dict(fm_state=2, rtm_phase=2) if level >= 4 else {}
            raw = {k: v for k, v in raw.items() if any(f["name"] == k for f in layout["fields"])}
            rec_bytes = pack_record(layout, raw)
            with tempfile.TemporaryDirectory() as tmp:
                log_path = os.path.join(tmp, "v2.log")
                write_binary_log(log_path, level, layout["record_size"], [rec_bytes], format_ver=2)
                records = list(bl.iter_binary_records(log_path))
                out_path = os.path.join(tmp, "expanded_v2.csv")
                bl.write_expanded_csv(records, out_path)
                with open(out_path, "r", encoding="utf-8") as f:
                    header = f.readline().strip()
                    row = f.readline().strip()
            as_dict = dict(zip(header.split(","), row.split(",")))
            self.assertIn("motor_gate_open", as_dict, layout["name"])
            if level >= 4:
                self.assertEqual(as_dict["fm_state_name"], "ACTIVE", layout["name"])
            if level >= 5:
                self.assertEqual(as_dict["rtm_phase_name"], "run", layout["name"])


# ============================================================
# VESC 2 over CAN (2026-10-06): the 126 B L5_VESC2 tier
# ============================================================

_FW_HEADER_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "..", "Source", "V2_Integration_Rx", "BREmote_V2_Rx.h")
# 2026-10-31: the RX's RTMState.ino (the stop-reason names of the wrong-way net).
_FW_RTM_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "..", "..", "Source", "V2_Integration_Rx", "RTMState.ino")

# One fresh VESC 2 reading, as raw struct values (what fillVesc2Block() writes).
_VESC2_FRESH_RAW = {
    "vesc2_age_ms": 412,
    "vesc2_motor_current_A": 1234,      # 12.34 A
    "vesc2_battery_current_A": -567,    # -5.67 A (regen)
    "vesc2_duty_cycle_%": 45,
    "vesc2_voltage_V": 502,             # 50.2 V
    "vesc2_ERPM": -1200,                # -12000 ERPM
    "vesc2_temp_mos_C": 31,
    "vesc2_temp_motor_C": 28,
    "vesc2_fault_code": 0,
}
# What the firmware writes when VESC 2 is stale (real age, every value N/A).
_VESC2_STALE_RAW = {
    "vesc2_age_ms": 3400,
    "vesc2_motor_current_A": 0x7FFF,
    "vesc2_battery_current_A": 0x7FFF,
    "vesc2_duty_cycle_%": 0x7F,
    "vesc2_voltage_V": 0xFFFF,
    "vesc2_ERPM": 0x7FFF,
    "vesc2_temp_mos_C": 0x7F,
    "vesc2_temp_motor_C": 0x7F,
    "vesc2_fault_code": 0xFF,
}


class TestVesc2Tier(unittest.TestCase):
    def test_record_size_and_column_count(self):
        self.assertEqual(bl.LAYOUT_L5_VESC2["record_size"], 126)
        self.assertEqual(len(bl.LAYOUT_L5_VESC2["csv_header_cols"]), 76)
        self.assertEqual(len(bl.LAYOUT_L5_V2["csv_header_cols"]), 67)
        # the first 112 bytes are byte-identical to the format-2 level-5 record
        self.assertEqual(bl.LAYOUT_L5_VESC2["fields"][:len(bl.LAYOUT_L5_V2["fields"])], bl.LAYOUT_L5_V2["fields"])

    def test_firmware_header_matches(self):
        # Read the macro and the static_asserts straight out of the firmware header, so the reader
        # cannot drift from what the RX actually writes.
        self.assertEqual(bl.LAYOUT_L5_V2["csv_header"] + bl._FW_CSV_SUFFIX_VESC2, bl.LAYOUT_L5_VESC2["csv_header"])
        self.assertEqual(bl.LAYOUT_L5_V2["record_size"] + 14, bl.LAYOUT_L5_VESC2["record_size"])
        # 2026-10-06: LOG FORMAT 3 moved this block into the level-4 record, so the firmware no longer
        # writes this layout; the macro above went with it. This test now pins the FORMAT-2 history the
        # reader must keep decoding: 112 B + this 9-column suffix, which the reader's own constant holds.

    def test_fresh_binary_record_decodes(self):
        rec_bytes = pack_record(bl.LAYOUT_L5_VESC2, dict(_VESC2_FRESH_RAW, rider_fix_seq=7))
        self.assertEqual(len(rec_bytes), 126)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "v2fresh.log")
            write_binary_log(path, 5, 126, [rec_bytes], format_ver=2)
            rec = list(bl.iter_binary_records(path))[0]
        self.assertEqual(rec["_layout_name"], "L5_VESC2")
        self.assertEqual(rec["rider_fix_seq"], 7)                 # level-5 block unaffected
        self.assertEqual(rec["vesc2_age_ms"], 412)
        self.assertAlmostEqual(rec["vesc2_motor_current_A"], 12.34, places=3)
        self.assertAlmostEqual(rec["vesc2_battery_current_A"], -5.67, places=3)
        self.assertEqual(rec["vesc2_duty_cycle_%"], 45)
        self.assertAlmostEqual(rec["vesc2_voltage_V"], 50.2, places=3)
        self.assertAlmostEqual(rec["vesc2_ERPM"], -12000, places=3)
        self.assertEqual(rec["vesc2_temp_mos_C"], 31)
        self.assertEqual(rec["vesc2_temp_motor_C"], 28)
        self.assertEqual(rec["vesc2_fault_code"], 0)              # 0 = no fault is a real value, not N/A

    def test_stale_and_never_binary_records_decode_as_none(self):
        never = dict(_VESC2_STALE_RAW, vesc2_age_ms=0xFFFF)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "v2stale.log")
            write_binary_log(path, 5, 126, [pack_record(bl.LAYOUT_L5_VESC2, _VESC2_STALE_RAW),
                                            pack_record(bl.LAYOUT_L5_VESC2, never)], format_ver=2)
            stale, nev = list(bl.iter_binary_records(path))
        self.assertEqual(stale["vesc2_age_ms"], 3400)             # the real age survives
        self.assertIsNone(nev["vesc2_age_ms"])
        for rec in (stale, nev):
            for f in bl.VESC2_EXTRA_FIELDS[1:]:
                self.assertIsNone(rec[f["name"]], f["name"])

    def test_old_112_byte_level5_file_still_decodes_without_vesc2(self):
        rec_bytes = pack_record(bl.LAYOUT_L5_V2, {"rtm_phase": 1, "rider_fix_seq": 3})
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "old112.log")
            write_binary_log(path, 5, 112, [rec_bytes], format_ver=2)
            rec = list(bl.iter_binary_records(path))[0]
        self.assertEqual(rec["_layout_name"], "L5_V2")
        self.assertEqual(rec["rtm_phase_name"], "align")
        self.assertEqual(rec["rider_fix_seq"], 3)
        self.assertNotIn("vesc2_age_ms", rec)

    def test_device_csv_input_with_sentinels(self):
        base_cells = ["0"] * len(bl.LAYOUT_L5_V2["csv_header_cols"])
        fresh = base_cells + ["412", "12.34", "-5.67", "45", "50.2", "-12000", "31", "28", "0"]
        stale = base_cells + ["3400", "-999.00", "-999.00", "-999", "-999.0", "-999", "-999", "-999", "-999"]
        never = base_cells + ["-999", "-999.00", "-999.00", "-999", "-999.0", "-999", "-999", "-999", "-999"]
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "v2.csv")
            with open(path, "w", encoding="utf-8", newline="") as f:
                f.write(bl.LAYOUT_L5_VESC2["csv_header"] + "\n")
                for row in (fresh, stale, never):
                    f.write(",".join(row) + "\n")
            r_fresh, r_stale, r_never = list(bl.iter_csv_records(path))
        self.assertAlmostEqual(r_fresh["vesc2_motor_current_A"], 12.34, places=3)
        self.assertEqual(r_fresh["vesc2_ERPM"], -12000)
        self.assertEqual(r_fresh["vesc2_fault_code"], 0)
        self.assertEqual(r_stale["vesc2_age_ms"], 3400)
        self.assertIsNone(r_never["vesc2_age_ms"])
        for rec in (r_stale, r_never):
            for f in bl.VESC2_EXTRA_FIELDS[1:]:
                self.assertIsNone(rec[f["name"]], f["name"])

    def test_expanded_csv_carries_vesc2_columns_and_blanks_na(self):
        with tempfile.TemporaryDirectory() as tmp:
            log_path = os.path.join(tmp, "v2.log")
            write_binary_log(log_path, 5, 126, [pack_record(bl.LAYOUT_L5_VESC2, dict(_VESC2_FRESH_RAW, rtm_phase=2)),
                                                pack_record(bl.LAYOUT_L5_VESC2, _VESC2_STALE_RAW)], format_ver=2)
            records = list(bl.iter_binary_records(log_path))
            out_path = os.path.join(tmp, "expanded.csv")
            bl.write_expanded_csv(records, out_path)
            with open(out_path, "r", encoding="utf-8") as f:
                header = f.readline().strip().split(",")
                row1 = dict(zip(header, f.readline().strip().split(",")))
                row2 = dict(zip(header, f.readline().strip().split(",")))
        self.assertEqual(row1["rtm_phase_name"], "run")
        self.assertEqual(row1["vesc2_voltage_V"], "50.2")
        self.assertEqual(row1["vesc2_ERPM"], "-12000")
        self.assertEqual(row2["vesc2_age_ms"], "3400")
        self.assertEqual(row2["vesc2_voltage_V"], "")              # N/A -> empty cell, never a number


# ============================================================
# LOG FORMAT 3 (2026-10-06): the VESC 2 block moved into the level-4 record (owner ruling: both VESCs at
# level 4). Format 3 writes 62 / 104 / 126 B; the 126 B size is shared with format 2's L5_VESC2 but the
# layout differs, so every lookup is by (format_ver, record_size). Formats 1 and 2 must keep decoding.
# ============================================================

def _expand_fw_csv_macro(text: str, name: str) -> str:
    """Expand a LOG_CSV_HEADER_* macro straight out of BREmote_V2_Rx.h: each #define is a run of other
    macro names and string literals, concatenated - the same thing the C preprocessor does."""
    defs = {}
    for m in re.finditer(r'^#define (LOG_CSV_HEADER_\w+) (.+)$', text, re.M):
        defs[m.group(1)] = m.group(2).strip()
    out = []
    for tok in re.findall(r'"[^"]*"|\w+', defs[name]):
        out.append(tok[1:-1] if tok.startswith('"') else _expand_fw_csv_macro(text, tok))
    return "".join(out)


class TestLogFormat3(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(_FW_HEADER_PATH, "r", encoding="utf-8") as f:
            cls.fw = f.read()

    def test_firmware_constants_and_static_asserts(self):
        # 2026-10-14: format 4 added a boot block after the header; its records are format 3's, and the firmware
        # still reads format-3 files (LOG_FILE_FORMAT_MIN_READ).
        # 2026-10-15: the current format is 5 (the level-6 compass diagnostic block, 174 B); MIN_READ stays 3.
        # 2026-10-31: the current format is 6 (the level-6 rider velocity word, 178 B); MIN_READ stays 3.
        # 2026-10-31: the current format is 7 (unit H's level-6 pivot block, 182 B) with FM_PIVOT_LEARN_ENABLED 1; the
        # firmware still writes 6 with the switch at 0, so both defines are in the header. MIN_READ stays 3.
        self.assertEqual(bl.LOG_FILE_FORMAT_VER, 7)
        self.assertIn("#define LOG_FILE_FORMAT_VER  7 ", self.fw)
        self.assertIn("#define LOG_FILE_FORMAT_VER  6 ", self.fw)
        self.assertIn("#define LOG_FILE_FORMAT_MIN_READ 3 ", self.fw)
        self.assertIs(bl.RECORD_LAYOUTS_BY_FORMAT[4], bl.RECORD_LAYOUTS_BY_FORMAT[3])
        self.assertIn("static_assert(sizeof(VescLogDataL4) == 104,", self.fw)
        self.assertIn("static_assert(offsetof(VescLogDataL4, vesc2_age_ms) == 90,", self.fw)
        self.assertIn("static_assert(sizeof(VescLogDataL5) == 126,", self.fw)
        self.assertIn("static_assert(offsetof(VescLogDataL5, rider_lat) == 104,", self.fw)
        self.assertEqual(bl.LAYOUT_L4_V3["record_size"], 104)
        self.assertEqual(bl.LAYOUT_L5_V3["record_size"], 126)
        self.assertEqual(bl.LAYOUT_L3_V2["record_size"], 62)

    def test_firmware_csv_macros_expand_to_the_generated_headers(self):
        # The real macros, expanded here, against this tool's generated headers - all three format-3 tiers.
        self.assertEqual(_expand_fw_csv_macro(self.fw, "LOG_CSV_HEADER_L3"), bl.LAYOUT_L3_V2["csv_header"])
        self.assertEqual(_expand_fw_csv_macro(self.fw, "LOG_CSV_HEADER_L4_MOTORS"), bl.LAYOUT_L4_V2["csv_header"])
        self.assertEqual(_expand_fw_csv_macro(self.fw, "LOG_CSV_HEADER_L4"), bl.LAYOUT_L4_V3["csv_header"])
        self.assertEqual(_expand_fw_csv_macro(self.fw, "LOG_CSV_HEADER_L5"), bl.LAYOUT_L5_V3["csv_header"])
        self.assertEqual(len(bl.LAYOUT_L4_V3["csv_header_cols"]), 62)
        self.assertEqual(len(bl.LAYOUT_L5_V3["csv_header_cols"]), 76)

    def test_level4_record_carries_vesc2(self):
        raw = dict(_VESC2_FRESH_RAW, fm_state=2, motor0_cmd=30, motor_gate_open=1)
        rec_bytes = pack_record(bl.LAYOUT_L4_V3, raw)
        self.assertEqual(len(rec_bytes), 104)
        self.assertEqual(rec_bytes[90:92], struct.pack("<H", 412))     # VESC 2 age at byte 90
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "f3_l4.log")
            write_binary_log(path, 4, 104, [rec_bytes], format_ver=3)
            rec = list(bl.iter_binary_records(path))[0]
        self.assertEqual(rec["_layout_name"], "L4_V3")
        self.assertEqual(rec["fm_state_name"], "ACTIVE")
        self.assertEqual(rec["motor0_cmd"], 30)
        self.assertEqual(rec["motor_gate_open"], 1)
        self.assertEqual(rec["vesc2_age_ms"], 412)
        self.assertAlmostEqual(rec["vesc2_motor_current_A"], 12.34, places=3)
        self.assertAlmostEqual(rec["vesc2_ERPM"], -12000, places=3)
        self.assertNotIn("rider_lat", rec)

    def test_level5_record_carries_vesc2_then_level5_block(self):
        raw = dict(_VESC2_FRESH_RAW, rider_fix_seq=7, rtm_phase=1, rider_lat=41.5)
        rec_bytes = pack_record(bl.LAYOUT_L5_V3, raw)
        self.assertEqual(len(rec_bytes), 126)
        self.assertEqual(rec_bytes[104:108], struct.pack("<f", 41.5))  # level-5 block at byte 104
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "f3_l5.log")
            write_binary_log(path, 5, 126, [rec_bytes], format_ver=3)
            rec = list(bl.iter_binary_records(path))[0]
        self.assertEqual(rec["_layout_name"], "L5_V3")
        self.assertEqual(rec["vesc2_age_ms"], 412)
        self.assertAlmostEqual(rec["vesc2_voltage_V"], 50.2, places=3)
        self.assertEqual(rec["rider_fix_seq"], 7)
        self.assertEqual(rec["rtm_phase_name"], "align")
        self.assertAlmostEqual(rec["rider_lat"], 41.5, places=5)

    def test_same_126_bytes_are_read_by_the_files_own_format(self):
        # The heart of the bump: one 126 B record size, two layouts. The header's format_ver decides.
        f3 = pack_record(bl.LAYOUT_L5_V3, dict(_VESC2_FRESH_RAW, rider_fix_seq=7))
        f2 = pack_record(bl.LAYOUT_L5_VESC2, dict(_VESC2_FRESH_RAW, rider_fix_seq=7))
        self.assertNotEqual(f2, f3)
        with tempfile.TemporaryDirectory() as tmp:
            p3 = os.path.join(tmp, "f3.log")
            p2 = os.path.join(tmp, "f2.log")
            write_binary_log(p3, 5, 126, [f3], format_ver=3)
            write_binary_log(p2, 5, 126, [f2], format_ver=2)
            r3 = list(bl.iter_binary_records(p3))[0]
            r2 = list(bl.iter_binary_records(p2))[0]
        self.assertEqual(r3["_layout_name"], "L5_V3")
        self.assertEqual(r2["_layout_name"], "L5_VESC2")
        for rec in (r2, r3):                                         # both decode the same truth
            self.assertEqual(rec["rider_fix_seq"], 7)
            self.assertEqual(rec["vesc2_age_ms"], 412)
            self.assertAlmostEqual(rec["vesc2_battery_current_A"], -5.67, places=3)

    def test_old_format_logs_of_every_size_still_decode(self):
        # Everything a rider may already have downloaded: format 1 (59/65/83/85/87/109) and format 2
        # (62/90/112/126). Each must decode with its own layout, not a format-3 one.
        cases = [(1, lay) for lay in bl.RECORD_LAYOUTS_BY_FORMAT[1].values()] + \
                [(2, lay) for lay in bl.RECORD_LAYOUTS_BY_FORMAT[2].values()]
        self.assertEqual(len(cases), 10)
        for fv, lay in cases:
            with tempfile.TemporaryDirectory() as tmp:
                path = os.path.join(tmp, "old.log")
                write_binary_log(path, lay["level"], lay["record_size"], [pack_record(lay, {"timestamp_ms": 5})],
                                 format_ver=fv)
                rec = list(bl.iter_binary_records(path))[0]
            self.assertEqual(rec["_layout_name"], lay["name"], (fv, lay["record_size"]))
            self.assertEqual(rec["timestamp_ms"], 5)

    def test_odd_size_falls_back_within_the_files_own_format(self):
        # 110 B: format 3 -> L4_V3 (104), format 2 -> L4_V2 (90), format 1 -> L5 (109). Never across formats.
        for fv, want, lay in ((3, "L4_V3", bl.LAYOUT_L4_V3), (2, "L4_V2", bl.LAYOUT_L4_V2), (1, "L5", bl.LAYOUT_L5)):
            rec_bytes = pack_record(lay, {"fm_state": 3}) + b"\x00" * (110 - lay["record_size"])
            with tempfile.TemporaryDirectory() as tmp:
                path = os.path.join(tmp, "odd.log")
                write_binary_log(path, 4, 110, [rec_bytes], format_ver=fv)
                rec = list(bl.iter_binary_records(path))[0]
            self.assertEqual(rec["_layout_name"], want, fv)
            self.assertEqual(rec["fm_state_name"], "HOLD", fv)

    def test_unknown_format_is_a_clear_error(self):
        # 2026-10-14: 4 is a known format now (the boot block). 2026-10-15: so is 5 (the level-6 compass
        # diagnostic block). 2026-10-31: and 6 (the level-6 rider velocity word), so the first unknown one is 7.
        # 2026-10-31: and 7 (unit H's pivot block), so the first unknown one is 8.
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "f8.log")
            write_binary_log(path, 4, 104, [pack_record(bl.LAYOUT_L4_V3, {})], format_ver=8)
            with self.assertRaises(bl.LogFormatError) as ctx:
                list(bl.iter_binary_records(path))
        self.assertIn("format_ver=8", str(ctx.exception))

    def test_cli_exits_nonzero_with_message_on_unknown_format(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "f9.log")
            write_binary_log(path, 4, 104, [pack_record(bl.LAYOUT_L4_V3, {})], format_ver=9)
            self.assertEqual(bl.main([path, "--summary"]), 1)

    def test_device_csv_format3_and_older_headers_are_all_recognised(self):
        # Format-3 CSVs (the new L4 and L5 headers) and every format-2 CSV header a rider may hold.
        for lay in (bl.LAYOUT_L3_V2, bl.LAYOUT_L4_V2, bl.LAYOUT_L5_V2, bl.LAYOUT_L5_VESC2, bl.LAYOUT_L4_V3, bl.LAYOUT_L5_V3):
            self.assertIs(bl.CSV_HEADER_TO_LAYOUT[lay["csv_header"]], lay, lay["name"])
        # A format-3 level-4 CSV: VESC 2 fresh on row 1, never on row 2.
        cols = bl.LAYOUT_L4_V3["csv_header_cols"]
        base = ["0"] * (len(cols) - 9)
        fresh = base + ["412", "12.34", "-5.67", "45", "50.2", "-12000", "31", "28", "0"]
        never = base + ["-999", "-999.00", "-999.00", "-999", "-999.0", "-999", "-999", "-999", "-999"]
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "f3_l4.csv")
            with open(path, "w", encoding="utf-8", newline="") as f:
                f.write(bl.LAYOUT_L4_V3["csv_header"] + "\n" + ",".join(fresh) + "\n" + ",".join(never) + "\n")
            r_fresh, r_never = list(bl.iter_csv_records(path))
        self.assertEqual(r_fresh["_layout_name"], "L4_V3")
        self.assertAlmostEqual(r_fresh["vesc2_motor_current_A"], 12.34, places=3)
        self.assertEqual(r_fresh["vesc2_fault_code"], 0)
        for f in bl.VESC2_EXTRA_FIELDS:
            self.assertIsNone(r_never[f["name"]], f["name"])

    def test_expanded_csv_for_format3_tiers(self):
        for lay, level in ((bl.LAYOUT_L4_V3, 4), (bl.LAYOUT_L5_V3, 5)):
            raw = dict(_VESC2_FRESH_RAW, fm_state=2)
            if level == 5:
                raw["rtm_phase"] = 2
            with tempfile.TemporaryDirectory() as tmp:
                log_path = os.path.join(tmp, "f3.log")
                write_binary_log(log_path, level, lay["record_size"], [pack_record(lay, raw)], format_ver=3)
                records = list(bl.iter_binary_records(log_path))
                out_path = os.path.join(tmp, "f3.csv")
                bl.write_expanded_csv(records, out_path)
                with open(out_path, "r", encoding="utf-8") as f:
                    header = f.readline().strip().split(",")
                    row = dict(zip(header, f.readline().strip().split(",")))
            self.assertEqual(row["fm_state_name"], "ACTIVE", lay["name"])
            self.assertEqual(row["vesc2_voltage_V"], "50.2", lay["name"])
            if level == 5:
                self.assertEqual(row["rtm_phase_name"], "run")
            else:
                self.assertNotIn("rtm_phase_name", row)


# ============================================================
# 2026-10-08 - LOG LEVEL 6 (IMU): the 126 B format-3 level-5 record + two 22 B IMU blocks = 170 B, still
# format 3 (a tail append). VESC 2's IMU at 126-147, the reserved RX IMU at 148-169 (always N/A today).
# ============================================================

_COMMON_IMU_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "Source", "Common", "VescImu.h")

# One fresh, plausible VESC 2 IMU sample as raw struct values (what imuLogEncode() writes): 12.34 deg roll,
# -5.5 deg pitch, 170 deg yaw, gyro 1.5 / -80.3 / 33.0 dps, acc 0.120 / -0.340 / 0.980 g.
_IMU_FRESH_RAW = {
    "vesc2_imu_age_ms": 250,
    "vesc2_roll_deg": 1234, "vesc2_pitch_deg": -550, "vesc2_yaw_deg": 17000,
    "vesc2_gyro_x_dps": 15, "vesc2_gyro_y_dps": -803, "vesc2_gyro_z_dps": 330,
    "vesc2_acc_x_g": 120, "vesc2_acc_y_g": -340, "vesc2_acc_z_g": 980,
    "vesc2_imu_status": 1,
}
_IMU_VALUE_KEYS = ["vesc2_roll_deg", "vesc2_pitch_deg", "vesc2_yaw_deg", "vesc2_gyro_x_dps", "vesc2_gyro_y_dps",
                   "vesc2_gyro_z_dps", "vesc2_acc_x_g", "vesc2_acc_y_g", "vesc2_acc_z_g"]
# What the firmware always writes in the reserved RX IMU block (imuLogNoSource()).
_RX_IMU_NO_SOURCE_RAW = {"rx_imu_age_ms": 0xFFFF, "rx_roll_deg": 0x7FFF, "rx_pitch_deg": 0x7FFF, "rx_yaw_deg": 0x7FFF,
                         "rx_gyro_x_dps": 0x7FFF, "rx_gyro_y_dps": 0x7FFF, "rx_gyro_z_dps": 0x7FFF,
                         "rx_acc_x_g": 0x7FFF, "rx_acc_y_g": 0x7FFF, "rx_acc_z_g": 0x7FFF, "rx_imu_status": 0xFF}


def _imu_na(age: int, status: int) -> dict:
    raw = {k: 0x7FFF for k in _IMU_VALUE_KEYS}
    raw.update({"vesc2_imu_age_ms": age, "vesc2_imu_status": status})
    return raw


class TestLogLevel6(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(_FW_HEADER_PATH, "r", encoding="utf-8") as f:
            cls.fw = f.read()
        with open(_COMMON_IMU_PATH, "r", encoding="utf-8") as f:
            cls.imu_h = f.read()

    def _decode(self, raw: dict, format_ver: int = 3) -> dict:
        rec_bytes = pack_record(bl.LAYOUT_L6_V3, dict(_RX_IMU_NO_SOURCE_RAW, **raw))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "l6.log")
            write_binary_log(path, 6, 170, [rec_bytes], format_ver=format_ver)
            return list(bl.iter_binary_records(path))[0]

    def test_firmware_static_asserts_and_sizes(self):
        self.assertIn("static_assert(sizeof(VescLogDataL6) == 170,", self.fw)
        self.assertIn("static_assert(offsetof(VescLogDataL6, imu_vesc2) == 126,", self.fw)
        self.assertIn("static_assert(offsetof(VescLogDataL6, imu_rx) == 148,", self.fw)
        self.assertIn("static_assert(sizeof(VescLogDataL5) == 126,", self.fw)
        self.assertIn("static_assert(sizeof(ImuLogBlock) == 22,", self.imu_h)
        # a tail append: no format bump for level 6 (the 2026-10-14 bump to 4 changed the header, not a record)
        self.assertIs(bl.RECORD_LAYOUTS_BY_FORMAT[3][170], bl.LAYOUT_L6_V3)
        self.assertIs(bl.RECORD_LAYOUTS_BY_FORMAT[4][170], bl.LAYOUT_L6_V3)
        self.assertEqual(bl.LAYOUT_L6_V3["record_size"], 170)
        self.assertEqual(bl.LAYOUT_L6_V3["level"], 6)
        # the first 126 bytes are byte-identical to the format-3 level-5 record
        self.assertEqual(bl.LAYOUT_L6_V3["fields"][:len(bl.LAYOUT_L5_V3["fields"])], bl.LAYOUT_L5_V3["fields"])
        self.assertEqual(struct.calcsize("<" + "".join(f["fmt"] for f in bl.IMU_VESC2_FIELDS)), 22)

    def test_firmware_csv_macro_expands_to_the_generated_header(self):
        self.assertEqual(_expand_fw_csv_macro(self.fw, "LOG_CSV_HEADER_L6"), bl.LAYOUT_L6_V3["csv_header"])
        self.assertEqual(len(bl.LAYOUT_L6_V3["csv_header_cols"]), 98)
        self.assertIs(bl.CSV_HEADER_TO_LAYOUT[bl.LAYOUT_L6_V3["csv_header"]], bl.LAYOUT_L6_V3)

    def test_rsvd_bytes_are_not_columns(self):
        cols = bl.LAYOUT_L6_V3["csv_header_cols"]
        self.assertNotIn("vesc2_imu_rsvd", cols)
        self.assertNotIn("rx_imu_rsvd", cols)
        rec = self._decode(_IMU_FRESH_RAW)
        self.assertNotIn("vesc2_imu_rsvd", rec)

    def test_byte_offsets(self):
        rec_bytes = pack_record(bl.LAYOUT_L6_V3, dict(_RX_IMU_NO_SOURCE_RAW, **_IMU_FRESH_RAW))
        self.assertEqual(len(rec_bytes), 170)
        self.assertEqual(rec_bytes[126:128], struct.pack("<H", 250))      # VESC 2 IMU age at 126
        self.assertEqual(rec_bytes[130:132], struct.pack("<h", -550))     # pitch at 130
        self.assertEqual(rec_bytes[146], 1)                               # status at 146
        self.assertEqual(rec_bytes[148:150], struct.pack("<H", 0xFFFF))   # RX IMU age at 148

    def test_fresh_record_decodes(self):
        rec = self._decode(dict(_VESC2_FRESH_RAW, rider_fix_seq=7, **_IMU_FRESH_RAW))
        self.assertEqual(rec["_layout_name"], "L6_V3")
        self.assertEqual(rec["rider_fix_seq"], 7)                         # level-5 block unaffected
        self.assertEqual(rec["vesc2_age_ms"], 412)                        # VESC 2 values block unaffected
        self.assertEqual(rec["vesc2_imu_age_ms"], 250)
        self.assertAlmostEqual(rec["vesc2_roll_deg"], 12.34, places=3)
        self.assertAlmostEqual(rec["vesc2_pitch_deg"], -5.5, places=3)
        self.assertAlmostEqual(rec["vesc2_yaw_deg"], 170.0, places=3)
        self.assertAlmostEqual(rec["vesc2_gyro_y_dps"], -80.3, places=3)
        self.assertAlmostEqual(rec["vesc2_acc_z_g"], 0.98, places=3)
        self.assertEqual(rec["vesc2_imu_status"], 1)
        self.assertEqual(rec["vesc2_imu_status_name"], "OK")
        # the reserved RX IMU always reads N/A
        self.assertIsNone(rec["rx_imu_age_ms"])
        self.assertIsNone(rec["rx_roll_deg"])
        self.assertIsNone(rec["rx_imu_status"])
        self.assertEqual(rec["rx_imu_status_name"], "NO_SOURCE")

    def test_stale_implausible_and_never_decode_as_none(self):
        stale = self._decode(_imu_na(1800, 2))
        implausible = self._decode(_imu_na(120, 3))
        never = self._decode(_imu_na(0xFFFF, 0xFF))
        self.assertEqual(stale["vesc2_imu_age_ms"], 1800)                 # the real age survives
        self.assertEqual(stale["vesc2_imu_status_name"], "STALE")
        self.assertEqual(implausible["vesc2_imu_age_ms"], 120)
        self.assertEqual(implausible["vesc2_imu_status_name"], "IMPLAUSIBLE")
        self.assertIsNone(never["vesc2_imu_age_ms"])
        self.assertIsNone(never["vesc2_imu_status"])
        self.assertEqual(never["vesc2_imu_status_name"], "NO_SOURCE")
        for rec in (stale, implausible, never):
            for k in _IMU_VALUE_KEYS:
                self.assertIsNone(rec[k], k)

    def test_device_csv_input_with_sentinels(self):
        base = ["0"] * len(bl.LAYOUT_L5_V3["csv_header_cols"])
        rx_na = ["-999", "-999.00", "-999.00", "-999.00", "-999.0", "-999.0", "-999.0",
                 "-999.000", "-999.000", "-999.000", "-999"]
        fresh = base + ["250", "12.34", "-5.50", "170.00", "1.5", "-80.3", "33.0", "0.120", "-0.340", "0.980", "1"] + rx_na
        stale = base + ["1800", "-999.00", "-999.00", "-999.00", "-999.0", "-999.0", "-999.0",
                        "-999.000", "-999.000", "-999.000", "2"] + rx_na
        never = base + rx_na + rx_na
        self.assertEqual(len(fresh), 98)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "l6.csv")
            with open(path, "w", encoding="utf-8", newline="") as f:
                f.write(bl.LAYOUT_L6_V3["csv_header"] + "\n")
                for row in (fresh, stale, never):
                    f.write(",".join(row) + "\n")
            r_fresh, r_stale, r_never = list(bl.iter_csv_records(path))
        self.assertEqual(r_fresh["_layout_name"], "L6_V3")
        self.assertAlmostEqual(r_fresh["vesc2_roll_deg"], 12.34, places=3)
        self.assertAlmostEqual(r_fresh["vesc2_acc_y_g"], -0.34, places=3)
        self.assertEqual(r_fresh["vesc2_imu_status_name"], "OK")
        self.assertEqual(r_stale["vesc2_imu_age_ms"], 1800)
        self.assertEqual(r_stale["vesc2_imu_status_name"], "STALE")
        self.assertIsNone(r_never["vesc2_imu_age_ms"])
        self.assertEqual(r_never["vesc2_imu_status_name"], "NO_SOURCE")
        for rec in (r_fresh, r_stale, r_never):
            self.assertIsNone(rec["rx_imu_status"])
        for rec in (r_stale, r_never):
            for k in _IMU_VALUE_KEYS:
                self.assertIsNone(rec[k], k)

    def test_126_byte_format3_file_still_decodes_as_level5(self):
        rec_bytes = pack_record(bl.LAYOUT_L5_V3, dict(_VESC2_FRESH_RAW, rider_fix_seq=9))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "f3_l5.log")
            write_binary_log(path, 5, 126, [rec_bytes], format_ver=3)
            rec = list(bl.iter_binary_records(path))[0]
        self.assertEqual(rec["_layout_name"], "L5_V3")
        self.assertEqual(rec["rider_fix_seq"], 9)
        self.assertNotIn("vesc2_imu_age_ms", rec)

    def test_expanded_csv_and_summary(self):
        with tempfile.TemporaryDirectory() as tmp:
            log_path = os.path.join(tmp, "l6.log")
            recs = [pack_record(bl.LAYOUT_L6_V3, dict(_RX_IMU_NO_SOURCE_RAW, timestamp_ms=1000, **_IMU_FRESH_RAW)),
                    pack_record(bl.LAYOUT_L6_V3, dict(_RX_IMU_NO_SOURCE_RAW, timestamp_ms=1333, **_imu_na(1800, 2)))]
            write_binary_log(log_path, 6, 170, recs, format_ver=3)
            records = list(bl.iter_binary_records(log_path))
            out_path = os.path.join(tmp, "l6.csv")
            bl.write_expanded_csv(records, out_path)
            with open(out_path, "r", encoding="utf-8") as f:
                header = f.readline().strip().split(",")
                row1 = dict(zip(header, f.readline().strip().split(",")))
                row2 = dict(zip(header, f.readline().strip().split(",")))
        self.assertEqual(row1["vesc2_roll_deg"], "12.34")
        self.assertEqual(row1["vesc2_imu_status_name"], "OK")
        self.assertEqual(row1["rx_imu_status_name"], "NO_SOURCE")
        self.assertEqual(row1["rx_roll_deg"], "")                         # N/A -> empty cell
        self.assertEqual(row2["vesc2_pitch_deg"], "")
        self.assertEqual(row2["vesc2_imu_status_name"], "STALE")
        summary = bl.generate_summary(records)
        self.assertIn("VESC 2 IMU: 50.0% of rows OK; max |roll| 12.3 deg; max pitch -5.5 deg", summary)



# ============================================================
# LOG FORMAT 4 (2026-10-14): the boot block - which RX boot recorded a file and why that boot started
# ============================================================

_RESET_REASON_H_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "..", "Source", "Common", "ResetReason.h")


class TestLogFormat4BootBlock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(_FW_HEADER_PATH, "r", encoding="utf-8") as f:
            cls.fw = f.read()
        with open(_RESET_REASON_H_PATH, "r", encoding="utf-8") as f:
            cls.rr_h = f.read()

    def _l3_record(self, ts: int) -> bytes:
        return pack_record(bl.LAYOUT_L3_V2, {"timestamp_ms": ts})

    def test_firmware_boot_block_matches_this_tool(self):
        self.assertIn("static_assert(sizeof(LogFileBootInfo) == 8,", self.fw)
        m = re.search(r"struct __attribute__\(\(packed\)\) LogFileBootInfo \{(.*?)\};", self.fw, re.S)
        self.assertIsNotNone(m)
        fields = re.findall(r"^\s*(uint\d+_t)\s+(\w+)(\[\d+\])?;", m.group(1), re.M)
        self.assertEqual([(t, n) for t, n, _ in fields],
                         [("uint32_t", "boot_count"), ("uint8_t", "reset_reason"), ("uint8_t", "rsvd")])
        self.assertEqual(bl.LOG_FILE_BOOT_FMT, "<IB3x")
        self.assertEqual(bl.LOG_FILE_BOOT_SIZE, 8)

    def test_reset_reason_tables_match_the_firmware_header(self):
        codes = {name: int(v) for name, v in re.findall(r"RR_(\w+)\s*=\s*(\d+)", self.rr_h)}
        self.assertEqual(len(codes), 16)
        name_fn = self.rr_h[self.rr_h.index("resetReasonName(uint8_t code)"):self.rr_h.index("resetReasonClass(uint8_t code)")]
        class_fn = self.rr_h[self.rr_h.index("resetReasonClass(uint8_t code)"):self.rr_h.index("resetReasonIsCrash")]
        fw_names = {codes[k]: v for k, v in re.findall(r'case RR_(\w+):\s*return "([^"]*)";', name_fn)}
        fw_class = {codes[k]: v for k, v in re.findall(r'case RR_(\w+):\s*return "([^"]*)";', class_fn)}
        self.assertEqual(fw_names, bl.RESET_REASON_NAMES)
        self.assertEqual(fw_class, bl.RESET_REASON_CLASS)
        self.assertIn('default:            return "?";', name_fn)
        self.assertIn('default:            return "other / unknown";', class_fn)
        self.assertEqual(bl.describe_reset_reason(200), ("?", "other / unknown"))
        self.assertEqual(bl.describe_reset_reason(6), ("TASK_WDT", "CRASH (task watchdog: a loop stalled)"))

    def test_format4_binary_decodes_like_format3_and_reports_the_boot(self):
        recs = [self._l3_record(1000), self._l3_record(1333)]
        with tempfile.TemporaryDirectory() as tmp:
            p3 = os.path.join(tmp, "f3.log")
            p4 = os.path.join(tmp, "f4.log")
            write_binary_log(p3, 3, 62, recs, format_ver=3)
            write_binary_log(p4, 3, 62, recs, format_ver=4, boot=(1234, 9))
            r3 = list(bl.iter_binary_records(p3))
            r4 = list(bl.iter_binary_records(p4))
            self.assertEqual(r3, r4)
            self.assertEqual([r["timestamp_ms"] for r in r4], [1000, 1333])
            self.assertIsNone(bl.read_boot_info(p3))
            self.assertEqual(bl.read_boot_info(p4), {"boot_count": 1234, "reset_reason": 9,
                                                     "reset_reason_name": "BROWNOUT",
                                                     "reset_reason_class": "supply sagged (brownout)"})
            self.assertIn("RX boot: #1234, reset reason BROWNOUT (9): supply sagged (brownout)",
                          bl.generate_summary(r4, bl.read_boot_info(p4)))
            self.assertNotIn("RX boot:", bl.generate_summary(r3, bl.read_boot_info(p3)))

    def test_format4_file_cut_inside_the_boot_block_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, "cut.log")
            with open(p, "wb") as f:
                f.write(struct.pack(bl.LOG_FILE_HEADER_FMT, bl.LOG_FILE_MAGIC, 4, 3, 62))
                f.write(b"\x01\x00\x00")
            with self.assertRaises(bl.LogFormatError):
                list(bl.iter_binary_records(p))
            self.assertIsNone(bl.read_boot_info(p))

    def test_device_csv_boot_line_is_skipped_as_a_row_and_read_as_the_boot(self):
        # The line exactly as logBootMarker() prints it (its snprintf format is pinned below).
        self.assertIn('"=== RX boot #%lu / reset %s (%u): %s ==="', self.fw)
        name, klass = bl.describe_reset_reason(6)
        boot_line = f"=== RX boot #42 / reset {name} (6): {klass} ==="
        self.assertNotIn(",", boot_line)
        row = (
            "1000,0.50,1.20,10,40.6,1500,25,0,12.3,41.123456,-87.654321,1700000000,"
            "200,1,3,1,1,127,900,3600,65535,65535,900,65535,32767,32767,0,127,-1.0,-72,-99.0"
        )
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, "device4.csv")
            with open(p, "w", encoding="utf-8", newline="") as f:
                f.write(bl.LOG_CSV_HEADER_L3 + "\r\n" + boot_line + "\r\n" + row + "\r\n")
            records = list(bl.iter_csv_records(p))
            self.assertEqual(len(records), 1)
            self.assertEqual(records[0]["timestamp_ms"], 1000)
            self.assertEqual(bl.read_boot_info(p), {"boot_count": 42, "reset_reason": 6,
                                                    "reset_reason_name": "TASK_WDT",
                                                    "reset_reason_class": "CRASH (task watchdog: a loop stalled)"})
            # a format-3 device CSV has no boot line
            p3 = os.path.join(tmp, "device3.csv")
            with open(p3, "w", encoding="utf-8", newline="") as f:
                f.write(bl.LOG_CSV_HEADER_L3 + "\n" + row + "\n")
            self.assertIsNone(bl.read_boot_info(p3))


# ============================================================
# 2026-10-15 - LOG FORMAT 5: the 170 B level-6 record + the compass diagnostic block (two u16) = 174 B.
# compass_diag_dx10 (heading x 10) and compass_diag_norm (calibrated horizontal |B|, counts), 0xFFFF = no reading.
# ============================================================

class TestLogFormat5CompassDiag(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(_FW_HEADER_PATH, "r", encoding="utf-8") as f:
            cls.fw = f.read()

    def _decode(self, raw: dict) -> dict:
        rec_bytes = pack_record(bl.LAYOUT_L6_V5, dict(_RX_IMU_NO_SOURCE_RAW, **raw))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "l6c.log")
            write_binary_log(path, 6, 174, [rec_bytes], format_ver=5)
            return list(bl.iter_binary_records(path))[0]

    def test_firmware_static_asserts_and_sizes(self):
        self.assertIn("static_assert(sizeof(VescLogDataL6Compass) == 174,", self.fw)
        self.assertIn("static_assert(offsetof(VescLogDataL6Compass, compass_diag_dx10) == 170,", self.fw)
        self.assertIn("static_assert(sizeof(VescLogDataL6) == 170,", self.fw)   # the 170 B record is unchanged
        # 2026-10-31: LOG_REC_MAX is the 178 B format-6 record now (TestLogFormat6RiderVel checks it).
        self.assertEqual(bl.LAYOUT_L6_V5["record_size"], 174)
        self.assertEqual(bl.LAYOUT_L6_V5["level"], 6)
        self.assertIs(bl.RECORD_LAYOUTS_BY_FORMAT[5][174], bl.LAYOUT_L6_V5)
        self.assertIs(bl.RECORD_LAYOUTS_BY_FORMAT[5][170], bl.LAYOUT_L6_V3)
        self.assertIs(bl.RECORD_LAYOUTS_BY_FORMAT[5][126], bl.LAYOUT_L5_V3)
        self.assertNotIn(174, bl.RECORD_LAYOUTS_BY_FORMAT[4])
        # the first 170 bytes are byte-identical to the level-6 record
        self.assertEqual(bl.LAYOUT_L6_V5["fields"][:len(bl.LAYOUT_L6_V3["fields"])], bl.LAYOUT_L6_V3["fields"])

    def test_firmware_csv_macro_expands_to_the_generated_header(self):
        self.assertEqual(_expand_fw_csv_macro(self.fw, "LOG_CSV_HEADER_L6_COMPASS"), bl.LAYOUT_L6_V5["csv_header"])
        self.assertEqual(_expand_fw_csv_macro(self.fw, "LOG_CSV_HEADER_L6"), bl.LAYOUT_L6_V3["csv_header"])
        cols = bl.LAYOUT_L6_V5["csv_header_cols"]
        self.assertEqual(len(cols), 100)
        self.assertEqual(cols[-2:], ["compass_diag_dx10", "compass_diag_norm"])
        # every column that existed before keeps its position (a pure tail append)
        self.assertEqual(cols[:98], bl.LAYOUT_L6_V3["csv_header_cols"])
        self.assertIs(bl.CSV_HEADER_TO_LAYOUT[bl.LAYOUT_L6_V5["csv_header"]], bl.LAYOUT_L6_V5)

    def test_byte_offsets(self):
        rec_bytes = pack_record(bl.LAYOUT_L6_V5, dict(_RX_IMU_NO_SOURCE_RAW, compass_diag_deg=1234, compass_diag_norm=4321))
        self.assertEqual(len(rec_bytes), 174)
        self.assertEqual(rec_bytes[170:172], struct.pack("<H", 1234))
        self.assertEqual(rec_bytes[172:174], struct.pack("<H", 4321))

    def test_reading_and_no_reading_decode(self):
        got = self._decode(dict(_IMU_FRESH_RAW, thr_received=180, compass_diag_deg=1234, compass_diag_norm=4321))
        none = self._decode(dict(_IMU_FRESH_RAW, thr_received=10, compass_diag_deg=0xFFFF, compass_diag_norm=0xFFFF))
        self.assertEqual(got["_layout_name"], "L6_V5")
        self.assertAlmostEqual(got["compass_diag_deg"], 123.4, places=3)
        self.assertEqual(got["compass_diag_norm"], 4321)
        self.assertEqual(got["vesc2_imu_age_ms"], 250)                   # the IMU block is unaffected
        self.assertIsNone(none["compass_diag_deg"])
        self.assertIsNone(none["compass_diag_norm"])
        self.assertIn("Compass under throttle (diagnostic): 1 of 1 rows", bl.generate_summary([got, none]))

    def test_device_csv_input_with_sentinels(self):
        base = ["0"] * len(bl.LAYOUT_L6_V3["csv_header_cols"])
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "l6c.csv")
            with open(path, "w", encoding="utf-8", newline="") as f:
                f.write(bl.LAYOUT_L6_V5["csv_header"] + "\n")
                f.write(",".join(base + ["3599", "1500"]) + "\n")
                f.write(",".join(base + ["65535", "65535"]) + "\n")
            r_got, r_none = list(bl.iter_csv_records(path))
        self.assertEqual(r_got["_layout_name"], "L6_V5")
        self.assertAlmostEqual(r_got["compass_diag_deg"], 359.9, places=3)
        self.assertEqual(r_got["compass_diag_norm"], 1500)
        self.assertIsNone(r_none["compass_diag_deg"])
        self.assertIsNone(r_none["compass_diag_norm"])

    def test_format4_level6_file_still_decodes_without_the_block(self):
        rec_bytes = pack_record(bl.LAYOUT_L6_V3, dict(_RX_IMU_NO_SOURCE_RAW, **_IMU_FRESH_RAW))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "f4_l6.log")
            write_binary_log(path, 6, 170, [rec_bytes], format_ver=4)
            rec = list(bl.iter_binary_records(path))[0]
        self.assertEqual(rec["_layout_name"], "L6_V3")
        self.assertNotIn("compass_diag_deg", rec)

    def test_expanded_csv_carries_the_columns(self):
        import csv
        rec = self._decode(dict(_IMU_FRESH_RAW, compass_diag_deg=900, compass_diag_norm=0xFFFF))
        with tempfile.TemporaryDirectory() as tmp:
            out_path = os.path.join(tmp, "x.csv")
            bl.write_expanded_csv([rec], out_path)
            with open(out_path, "r", encoding="utf-8", newline="") as f:
                row = next(csv.DictReader(f))
        self.assertEqual(row["compass_diag_deg"], "90")
        self.assertEqual(row["compass_diag_norm"], "")


# ============================================================
# 2026-10-31 - LOG FORMAT 6: the 174 B level-6 record + the rider's v3 velocity word (u32) = 178 B.
# Bits 24-31 owner tag, 0-9 course (0.36 deg, 1000-1023 none), 10-19 speed (0.02 m/s, 1023 none), 20-23 fix age
# (50 ms, 15 none). Made-up values only.
# ============================================================

def _vel_word(tag: int, crs: int, spd: int, age: int) -> int:
    return ((tag & 0xFF) << 24) | ((age & 0xF) << 20) | ((spd & 0x3FF) << 10) | (crs & 0x3FF)


_VEL_NONE_TAG7 = _vel_word(7, 1023, 1023, 15)


class TestLogFormat6RiderVel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(_FW_HEADER_PATH, "r", encoding="utf-8") as f:
            cls.fw = f.read()
        logger_path = os.path.join(os.path.dirname(_FW_HEADER_PATH), "Logger.ino")
        with open(logger_path, "r", encoding="utf-8") as f:
            cls.logger = f.read()

    def _decode(self, raw: dict) -> dict:
        rec_bytes = pack_record(bl.LAYOUT_L6_V6, dict(_RX_IMU_NO_SOURCE_RAW, **raw))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "l6v.log")
            write_binary_log(path, 6, 178, [rec_bytes], format_ver=6)
            return list(bl.iter_binary_records(path))[0]

    def test_firmware_static_asserts_and_sizes(self):
        self.assertIn("static_assert(sizeof(VescLogDataL6Vel) == 178,", self.fw)
        self.assertIn("static_assert(offsetof(VescLogDataL6Vel, rider_vel_word) == 174,", self.fw)
        self.assertIn("static_assert(sizeof(VescLogDataL6Compass) == 174,", self.fw)   # the 174 B record is unchanged
        self.assertIn("#define LOG_REC_MAX sizeof(VescLogDataL6Vel)", self.fw)
        self.assertIn("#define LOG_FILE_FORMAT_MIN_READ 3 ", self.fw)
        self.assertEqual(bl.LAYOUT_L6_V6["record_size"], 178)
        self.assertEqual(bl.LAYOUT_L6_V6["level"], 6)
        self.assertIs(bl.RECORD_LAYOUTS_BY_FORMAT[6][178], bl.LAYOUT_L6_V6)
        self.assertIs(bl.RECORD_LAYOUTS_BY_FORMAT[6][174], bl.LAYOUT_L6_V5)
        self.assertIs(bl.RECORD_LAYOUTS_BY_FORMAT[6][126], bl.LAYOUT_L5_V3)
        self.assertNotIn(178, bl.RECORD_LAYOUTS_BY_FORMAT[5])
        # the first 174 bytes are byte-identical to the format-5 level-6 record
        self.assertEqual(bl.LAYOUT_L6_V6["fields"][:len(bl.LAYOUT_L6_V5["fields"])], bl.LAYOUT_L6_V5["fields"])

    def test_firmware_csv_macro_expands_to_the_generated_header(self):
        self.assertEqual(_expand_fw_csv_macro(self.fw, "LOG_CSV_HEADER_L6_VEL"), bl.LAYOUT_L6_V6["csv_header"])
        self.assertEqual(_expand_fw_csv_macro(self.fw, "LOG_CSV_HEADER_L6_COMPASS"), bl.LAYOUT_L6_V5["csv_header"])
        cols = bl.LAYOUT_L6_V6["csv_header_cols"]
        self.assertEqual(len(cols), 101)
        self.assertEqual(cols[-1], "rider_vel_word")
        self.assertEqual(cols[:100], bl.LAYOUT_L6_V5["csv_header_cols"])   # a pure tail append
        self.assertIs(bl.CSV_HEADER_TO_LAYOUT[bl.LAYOUT_L6_V6["csv_header"]], bl.LAYOUT_L6_V6)

    def test_the_logger_writes_the_published_word(self):
        # loggerTask's level-6 branch copies the radio task's published word, and builds the 178 B record.
        self.assertIn("logData6.rider_vel_word = rx_tx_gps_vel_word;", self.logger)
        self.assertIn("static VescLogDataL6Vel logData6;", self.logger)

    def test_byte_offsets(self):
        rec_bytes = pack_record(bl.LAYOUT_L6_V6, dict(_RX_IMU_NO_SOURCE_RAW, rider_vel_word=0x12345678))
        self.assertEqual(len(rec_bytes), 178)
        self.assertEqual(rec_bytes[174:178], struct.pack("<I", 0x12345678))

    def test_word_decodes_and_sentinels_are_none(self):
        got = self._decode(dict(_IMU_FRESH_RAW, rider_vel_word=_vel_word(42, 250, 250, 2)))
        self.assertEqual(got["_layout_name"], "L6_V6")
        self.assertEqual(got["rider_vel_tag"], 42)
        self.assertAlmostEqual(got["rider_vel_course_deg"], 90.0, places=6)
        self.assertAlmostEqual(got["rider_vel_speed_kmh"], 18.0, places=6)          # 5 m/s
        self.assertAlmostEqual(got["rider_vel_speed_mph"], 5.0 / 0.44704, places=6)
        self.assertEqual(got["rider_vel_age_ms"], 100)
        self.assertEqual(got["vesc2_imu_age_ms"], 250)                               # the IMU block is unaffected
        none = self._decode(dict(_IMU_FRESH_RAW, rider_vel_word=_VEL_NONE_TAG7))
        self.assertEqual(none["rider_vel_tag"], 7)
        for key in ("rider_vel_course_deg", "rider_vel_speed_kmh", "rider_vel_speed_mph", "rider_vel_age_ms"):
            self.assertIsNone(none[key], key)
        # course codes 1000-1022 are "no course", as on the RX; speed 1022 (20.44 m/s) is the last real one
        odd = bl.decode_rider_vel(_vel_word(1, 1000, 1022, 14))
        self.assertIsNone(odd["rider_vel_course_deg"])
        self.assertAlmostEqual(odd["rider_vel_speed_kmh"], 20.44 * 3.6, places=6)
        self.assertEqual(odd["rider_vel_age_ms"], 700)

    def test_device_csv_input(self):
        base = ["0"] * len(bl.LAYOUT_L6_V5["csv_header_cols"])
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "l6v.csv")
            with open(path, "w", encoding="utf-8", newline="") as f:
                f.write(bl.LAYOUT_L6_V6["csv_header"] + "\n")
                f.write(",".join(base + [str(_vel_word(3, 999, 100, 0))]) + "\n")
                f.write(",".join(base + [str(_VEL_NONE_TAG7)]) + "\n")
            r_got, r_none = list(bl.iter_csv_records(path))
        self.assertEqual(r_got["_layout_name"], "L6_V6")
        self.assertAlmostEqual(r_got["rider_vel_course_deg"], 359.64, places=6)
        self.assertAlmostEqual(r_got["rider_vel_speed_kmh"], 7.2, places=6)
        self.assertEqual(r_got["rider_vel_age_ms"], 0)
        self.assertIsNone(r_none["rider_vel_speed_kmh"])

    def test_format5_level6_file_still_decodes_without_the_word(self):
        rec_bytes = pack_record(bl.LAYOUT_L6_V5, dict(_RX_IMU_NO_SOURCE_RAW, **_IMU_FRESH_RAW))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "f5_l6.log")
            write_binary_log(path, 6, 174, [rec_bytes], format_ver=5)
            rec = list(bl.iter_binary_records(path))[0]
        self.assertEqual(rec["_layout_name"], "L6_V5")
        self.assertNotIn("rider_vel_word", rec)
        self.assertNotIn("rider_vel_speed_kmh", rec)

    def test_expanded_csv_carries_the_decoded_columns(self):
        import csv
        rec = self._decode(dict(_IMU_FRESH_RAW, rider_vel_word=_vel_word(9, 500, 1023, 15)))
        with tempfile.TemporaryDirectory() as tmp:
            out_path = os.path.join(tmp, "x.csv")
            bl.write_expanded_csv([rec], out_path)
            with open(out_path, "r", encoding="utf-8", newline="") as f:
                reader = csv.DictReader(f)
                row = next(reader)
                header = reader.fieldnames
        self.assertEqual(header[-len(bl.RIDER_VEL_DECODED_NAMES):], bl.RIDER_VEL_DECODED_NAMES)   # appended last
        self.assertEqual(row["rider_vel_tag"], "9")
        self.assertEqual(row["rider_vel_course_deg"], "180")
        self.assertEqual(row["rider_vel_speed_kmh"], "")
        self.assertEqual(row["rider_vel_age_ms"], "")


# ============================================================
# 2026-10-31 - LOG FORMAT 7: the 178 B level-6 record + unit H's 4-byte pivot block = 182 B.
# pivot_rate_dx10 (i16, deg/s x 10, 0x7FFF none), pivot_cap (u8, 0 = not acting), pivot_state (u8: bits 0-2 state,
# bits 6-7 calibration 0 / 1 = +1 / 2 = -1). Made-up values only.
# ============================================================

class TestLogFormat7Pivot(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(_FW_HEADER_PATH, "r", encoding="utf-8") as f:
            cls.fw = f.read()
        logger_path = os.path.join(os.path.dirname(_FW_HEADER_PATH), "Logger.ino")
        with open(logger_path, "r", encoding="utf-8") as f:
            cls.logger = f.read()

    def _decode(self, raw: dict) -> dict:
        rec_bytes = pack_record(bl.LAYOUT_L6_V7, dict(_RX_IMU_NO_SOURCE_RAW, **raw))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "l6p.log")
            write_binary_log(path, 6, 182, [rec_bytes], format_ver=7)
            return list(bl.iter_binary_records(path))[0]

    def test_firmware_static_asserts_and_sizes(self):
        self.assertIn("static_assert(sizeof(VescLogDataL6Pv) == 182,", self.fw)
        self.assertIn("static_assert(offsetof(VescLogDataL6Pv, pv) == 178,", self.fw)
        self.assertIn("static_assert(sizeof(LogPivotBlock) == 4,", self.fw)
        self.assertIn("static_assert(sizeof(VescLogDataL6Vel) == 178,", self.fw)   # the 178 B record is unchanged
        self.assertIn("#define LOG_REC_MAX sizeof(VescLogDataL6Pv)", self.fw)
        self.assertIn("#define LOG_REC_MAX sizeof(VescLogDataL6Vel)", self.fw)    # the FM_PIVOT_LEARN_ENABLED 0 build
        self.assertEqual(bl.LAYOUT_L6_V7["record_size"], 182)
        self.assertEqual(bl.LAYOUT_L6_V7["level"], 6)
        self.assertIs(bl.RECORD_LAYOUTS_BY_FORMAT[7][182], bl.LAYOUT_L6_V7)
        self.assertIs(bl.RECORD_LAYOUTS_BY_FORMAT[7][178], bl.LAYOUT_L6_V6)
        self.assertIs(bl.RECORD_LAYOUTS_BY_FORMAT[7][126], bl.LAYOUT_L5_V3)
        self.assertNotIn(182, bl.RECORD_LAYOUTS_BY_FORMAT[6])
        # the first 178 bytes are byte-identical to the format-6 level-6 record
        self.assertEqual(bl.LAYOUT_L6_V7["fields"][:len(bl.LAYOUT_L6_V6["fields"])], bl.LAYOUT_L6_V6["fields"])

    def test_firmware_csv_macro_expands_to_the_generated_header(self):
        self.assertEqual(_expand_fw_csv_macro(self.fw, "LOG_CSV_HEADER_L6_PV"), bl.LAYOUT_L6_V7["csv_header"])
        self.assertEqual(_expand_fw_csv_macro(self.fw, "LOG_CSV_HEADER_L6_VEL"), bl.LAYOUT_L6_V6["csv_header"])
        cols = bl.LAYOUT_L6_V7["csv_header_cols"]
        self.assertEqual(len(cols), 104)
        self.assertEqual(cols[-3:], ["pivot_rate_dx10", "pivot_cap", "pivot_state"])
        self.assertEqual(cols[:101], bl.LAYOUT_L6_V6["csv_header_cols"])   # a pure tail append
        self.assertIs(bl.CSV_HEADER_TO_LAYOUT[bl.LAYOUT_L6_V7["csv_header"]], bl.LAYOUT_L6_V7)

    def test_the_logger_appends_the_snapshot_block(self):
        # loggerTask's level-6 branch appends the pivot block from the row's own snapshot, inside the build switch.
        self.assertIn("pb.pivot_rate_dx10 = snap.pv_rate_dx10;", self.logger)
        self.assertIn("pb.pivot_cap       = snap.pv_cap;", self.logger)
        self.assertIn("pb.pivot_state     = snap.pv_state;", self.logger)
        self.assertIn("#if FM_PIVOT_LEARN_ENABLED", self.logger)

    def test_byte_offsets(self):
        rec_bytes = pack_record(bl.LAYOUT_L6_V7, dict(_RX_IMU_NO_SOURCE_RAW, pivot_rate_dps=-123, pivot_cap=74,
                                                      pivot_state=0x42))
        self.assertEqual(len(rec_bytes), 182)
        self.assertEqual(rec_bytes[178:182], struct.pack("<hBB", -123, 74, 0x42))

    def test_block_decodes_and_the_sentinel_is_none(self):
        got = self._decode(dict(_IMU_FRESH_RAW, pivot_rate_dps=-215, pivot_cap=60, pivot_state=0x40 | 2))
        self.assertEqual(got["_layout_name"], "L6_V7")
        self.assertAlmostEqual(got["pivot_rate_dps"], -21.5, places=6)
        self.assertEqual(got["pivot_cap"], 60)
        self.assertEqual(got["pivot_state_name"], "STEP_UP")
        self.assertEqual(got["pivot_cal_sign"], 1)
        self.assertEqual(got["vesc2_imu_age_ms"], 250)                               # the IMU block is unaffected
        none = self._decode(dict(_IMU_FRESH_RAW, pivot_rate_dps=0x7FFF, pivot_cap=0, pivot_state=0))
        self.assertIsNone(none["pivot_rate_dps"])
        self.assertEqual(none["pivot_state_name"], "NONE")
        self.assertEqual(none["pivot_cal_sign"], 0)
        # every state name, and the -1 calibration
        for code, name in ((1, "PIVOTING"), (3, "STEP_DOWN"), (4, "NOT_TURNING"), (5, "WRONG_WAY")):
            self.assertEqual(bl.decode_pivot_state(0x80 | code), {"pivot_state_name": name, "pivot_cal_sign": -1})

    def test_device_csv_input(self):
        base = ["0"] * len(bl.LAYOUT_L6_V6["csv_header_cols"])
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "l6p.csv")
            with open(path, "w", encoding="utf-8", newline="") as f:
                f.write(bl.LAYOUT_L6_V7["csv_header"] + "\n")
                f.write(",".join(base + ["305", "80", str(0x80 | 3)]) + "\n")
                f.write(",".join(base + ["32767", "0", "0"]) + "\n")
            r_got, r_none = list(bl.iter_csv_records(path))
        self.assertEqual(r_got["_layout_name"], "L6_V7")
        self.assertAlmostEqual(r_got["pivot_rate_dps"], 30.5, places=6)
        self.assertEqual(r_got["pivot_cap"], 80)
        self.assertEqual(r_got["pivot_state_name"], "STEP_DOWN")
        self.assertEqual(r_got["pivot_cal_sign"], -1)
        self.assertIsNone(r_none["pivot_rate_dps"])

    def test_format6_level6_file_still_decodes_without_the_block(self):
        rec_bytes = pack_record(bl.LAYOUT_L6_V6, dict(_RX_IMU_NO_SOURCE_RAW, **_IMU_FRESH_RAW))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "f6_l6.log")
            write_binary_log(path, 6, 178, [rec_bytes], format_ver=6)
            rec = list(bl.iter_binary_records(path))[0]
        self.assertEqual(rec["_layout_name"], "L6_V6")
        self.assertNotIn("pivot_rate_dps", rec)
        self.assertNotIn("pivot_state_name", rec)
        self.assertIn("rider_vel_word", rec)

    def test_expanded_csv_carries_the_decoded_columns(self):
        import csv
        rec = self._decode(dict(_IMU_FRESH_RAW, pivot_rate_dps=100, pivot_cap=50, pivot_state=0x40 | 4))
        with tempfile.TemporaryDirectory() as tmp:
            out_path = os.path.join(tmp, "x.csv")
            bl.write_expanded_csv([rec], out_path)
            with open(out_path, "r", encoding="utf-8", newline="") as f:
                reader = csv.DictReader(f)
                row = next(reader)
                header = reader.fieldnames
        self.assertEqual(header[-len(bl.PIVOT_DECODED_NAMES):], bl.PIVOT_DECODED_NAMES)   # appended last
        self.assertEqual(row["pivot_state_name"], "NOT_TURNING")
        self.assertEqual(row["pivot_cal_sign"], "1")
        self.assertEqual(row["pivot_rate_dps"], "10")


if __name__ == "__main__":
    unittest.main()
