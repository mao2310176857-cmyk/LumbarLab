"""Protocol, stale-data and host-zero tests; no serial hardware required."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from lumbar.devices import SerialSensor, load_mapping, map_zones, parse_frame


def packet(seq=0, t=0, raw=10, diff=3, base=1):
    result = {"seq": seq, "t": t, "base": base, "diff": [diff] * 256}
    if raw is not None:
        result["raw"] = [raw] * 256
    return json.dumps(result)


class ProtocolTests(unittest.TestCase):
    def test_comments_and_blank(self):
        self.assertIsNone(parse_frame(b"# baseline ready\r\n"))
        self.assertIsNone(parse_frame("  \n"))

    def test_full_array_boundaries(self):
        frame = json.loads(packet(seq=2**32 - 1, t=2**32 - 1, raw=255, diff=255))
        frame.update(peak=255, pr=15, pc=15, nz=256, sum=65280)
        self.assertEqual(parse_frame(json.dumps(frame))["nz"], 256)
        self.assertEqual(parse_frame(packet(base=0))["base"], 0)

    def test_malformed_data_and_wrong_types(self):
        invalid = ["{", "[]", '{"seq":0,"seq":1}', b"\xff", "x" * 9000]
        for key, value in (("seq", True), ("seq", -1), ("seq", 2**32),
                           ("t", 1.2), ("base", 2), ("diff", [0]*255),
                           ("raw", [256]*256), ("raw", [False]*256),
                           ("raw", [float("nan")]*256)):
            frame = json.loads(packet())
            frame[key] = value
            invalid.append(json.dumps(frame))
        for line in invalid:
            with self.subTest(line=str(line)[:80]):
                with self.assertRaises(ValueError):
                    parse_frame(line)


class MappingTests(unittest.TestCase):
    def write_mapping(self, mapping):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "zones.json"
        path.write_text(json.dumps(mapping), encoding="utf-8")
        return path

    def test_empty_and_partial_mapping_never_become_zero_zones(self):
        mapping = {str(zone): [] for zone in range(1, 9)}
        path = self.write_mapping(mapping)
        self.assertIsNone(map_zones([10]*256, load_mapping(path)))
        mapping["1"] = [0]
        self.assertIsNone(map_zones([10]*256, mapping))

    def test_means_and_unassigned_cells(self):
        mapping = {str(zone): [zone - 1] for zone in range(1, 9)}
        mapping["1"].append(8)
        self.assertEqual(map_zones(list(range(256)), mapping), [4., 1., 2., 3., 4., 5., 6., 7.])

    def test_duplicate_bounds_and_missing_zone_rejected(self):
        for value in ([256], [-1], [True], [0, 0]):
            mapping = {str(zone): [] for zone in range(1, 9)}
            mapping["1"] = value
            with self.assertRaises(ValueError):
                load_mapping(self.write_mapping(mapping))
        mapping = {str(zone): [0] for zone in range(1, 9)}
        with self.assertRaises(ValueError):
            load_mapping(self.write_mapping(mapping))
        with self.assertRaises(ValueError):
            load_mapping(self.write_mapping({"1": [0]}))


class SerialStateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        path = Path(self.temp.name) / "zones.json"
        path.write_text(json.dumps({str(n): [n-1] for n in range(1, 9)}), encoding="utf-8")
        self.sensor = SerialSensor("TEST", mapping_path=path)
        self.addCleanup(self.sensor.close)

    def test_frame_handoff_once_and_own_arrival_time(self):
        self.sensor._process_line(packet(seq=10, t=90000), now=2.0)
        frame = self.sensor.read()
        self.assertTrue(frame.valid)
        self.assertEqual(frame.received_at, 2.0)
        self.assertEqual(frame.zones, [3.0]*8)
        self.assertEqual(frame.raw_zones, [10.0]*8)
        self.assertIsNone(self.sensor.read())

    def test_duplicate_and_reordered_frames_do_not_refresh(self):
        self.sensor._process_line(packet(seq=100, t=20000), now=2.0)
        self.sensor.read()
        self.sensor._process_line(packet(seq=100, t=20000), now=3.0)
        self.assertIsNone(self.sensor.read())
        self.assertEqual(self.sensor.sequence_status, "duplicate")
        self.sensor._process_line(packet(seq=99, t=19900), now=4.0)
        self.assertIsNone(self.sensor.read())
        self.assertEqual(self.sensor.sequence_status, "reordered")

    def test_uint32_rollover_is_forward(self):
        self.sensor._process_line(packet(seq=2**32-1, t=2**32-10), now=2.0)
        self.sensor.read()
        self.sensor._process_line(packet(seq=0, t=10), now=2.02)
        self.assertTrue(self.sensor.read().valid)
        self.assertEqual(self.sensor.sequence_status, "ok")

    def test_reset_invalidates_and_requires_reconnect(self):
        self.sensor._process_line(packet(seq=200, t=20000), now=2.0)
        self.sensor.read()
        self.sensor._host_baseline = [10]*256
        self.sensor._process_line(packet(seq=0, t=10), now=3.0)
        self.assertFalse(self.sensor.read().valid)
        self.assertEqual(self.sensor.sequence_status, "reset")
        self.assertIsNone(self.sensor._host_baseline)
        self.sensor._process_line(packet(seq=1, t=100), now=3.1)
        self.assertIsNone(self.sensor.read())

    def test_invalid_payload_replaces_pending_measurement(self):
        self.sensor._process_line(packet(), now=2.0)
        self.sensor._process_line("bad-data", now=3.0)
        frame = self.sensor.read()
        self.assertFalse(frame.valid)
        self.assertIsNone(frame.matrix)
        self.sensor._process_line(packet(seq=1, t=100, base=0), now=3.1)
        self.assertFalse(self.sensor.read().valid)

    def test_zero_uses_stable_fresh_raw_mean_then_next_frame(self):
        for n in range(10):
            self.sensor._process_line(packet(seq=n, t=100*n, raw=10), now=1 + .1*n)
        with patch("lumbar.devices.time.monotonic", return_value=2.0):
            self.sensor.zero()
        self.assertIsNone(self.sensor.read())
        self.sensor._process_line(packet(seq=10, t=1000, raw=12, diff=99), now=2.1)
        frame = self.sensor.read()
        self.assertEqual(frame.matrix, [2]*256)
        self.assertEqual(frame.raw_matrix, [12]*256)
        self.sensor._process_line(packet(seq=11, t=1100, raw=None), now=2.2)
        self.assertFalse(self.sensor.read().valid)

    def test_zero_rejects_missing_stale_or_unstable_raw(self):
        with self.assertRaises(ValueError):
            self.sensor.zero()
        for n in range(10):
            self.sensor._process_line(packet(seq=n, t=100*n, raw=(0 if n == 5 else 50)), now=1 + .1*n)
        with patch("lumbar.devices.time.monotonic", return_value=2.0):
            with self.assertRaisesRegex(ValueError, "波动"):
                self.sensor.zero()
        with patch("lumbar.devices.time.monotonic", return_value=3.0):
            with self.assertRaisesRegex(ValueError, "新鲜"):
                self.sensor.zero()

    def test_reader_recovers_after_oversized_line_without_writes(self):
        sensor = self.sensor
        chunks = [b"x"*4096, b"x"*4096, b"x\n", packet(seq=1, t=100).encode() + b"\n"]

        class ReadOnlyFakePort:
            @property
            def in_waiting(self):
                return len(chunks[0]) if chunks else 0

            def read(self, size):
                if chunks:
                    return chunks.pop(0)
                sensor._stop.set()
                return b""

            def close(self):
                pass

            def write(self, data):
                raise AssertionError("Sensor adapter must not write")

        sensor._serial = ReadOnlyFakePort()
        sensor._reader()
        frame = sensor.read()
        self.assertEqual(frame.seq, 1)
        self.assertTrue(frame.valid)


if __name__ == "__main__":
    unittest.main()
