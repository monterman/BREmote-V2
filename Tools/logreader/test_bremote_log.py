#!/usr/bin/env python3
"""
Unit tests for bremote_log.py. Stdlib unittest only - run with:

    python -m unittest Tools/logreader/test_bremote_log.py -v

or, from this directory:

    python test_bremote_log.py -v
"""

from __future__ import annotations

import os
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


def write_binary_log(path: str, level: int, record_size: int, records: list[bytes]) -> None:
    with open(path, "wb") as f:
        f.write(struct.pack(bl.LOG_FILE_HEADER_FMT, bl.LOG_FILE_MAGIC, bl.LOG_FILE_FORMAT_VER, level, record_size))
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
            write_binary_log(path, 4, 83, [rec_bytes])
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
            write_binary_log(path, 4, 87, [rec_bytes])
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
            write_binary_log(path, 5, 109, [rec_bytes])
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
        # 90 bytes: between the 87 B "L4" tier and the 109 B "L5" tier - firmware's own
        # logCsvHeaderFor() would read this range as the L4 (87 B) tier too.
        raw = {"fm_state": 3, "fm_mode": 2}
        rec_bytes = pack_record(bl.LAYOUT_L4, raw) + b"\x00\x00\x00"  # 3 trailing bytes, size 90
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "future.log")
            write_binary_log(path, 4, 90, [rec_bytes])
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
        write_binary_log(cls.path, 4, 87, rec_bytes)
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
        write_binary_log(cls.path, 4, 87, rec_bytes)
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
            write_binary_log(path, 5, 109, rec_bytes)
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
            write_binary_log(log_path, 4, 83, [rec_bytes])
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
            write_binary_log(log_path, 5, 109, [rec_bytes])
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


if __name__ == "__main__":
    unittest.main()
