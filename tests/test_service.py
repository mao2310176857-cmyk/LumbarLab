"""Behavioral integration tests using a deterministic clock and temporary records."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from lumbar.devices import SensorFrame
from lumbar.service import AppService


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.clock = patch("lumbar.service.time.monotonic", return_value=100.0).start()
        self.addCleanup(patch.stopall)
        self.service = AppService(data_dir=Path(self.directory.name))
        self.addCleanup(self.service.close)

    def advance(self, seconds, step=0.1):
        count = max(1, round(seconds / step))
        for _ in range(count):
            self.clock.return_value += seconds / count
            self.service.tick()
        return self.service.state()

    def action(self, operation, **values):
        return self.service.action({"action": operation, **values})

    def calibrate(self):
        self.action("calibrate")
        state = self.advance(3.2)
        self.assertIsNotNone(state["reference"], state["calibration"])
        return state

    def test_initial_state_is_complete_and_has_no_real_actuation(self):
        state = self.service.state()
        expected = {"time", "source", "connected", "zones", "raw_zones", "matrix", "raw_matrix",
                    "frame_seq", "device_time_ms", "frame_age_ms", "signal_valid", "occupied",
                    "contact", "mode", "action", "reason", "position", "reference", "calibration",
                    "metrics", "parameters", "recording", "events", "history", "hardware"}
        self.assertTrue(expected.issubset(state))
        self.assertEqual(state["source"], "simulation")
        self.assertEqual(len(state["zones"]), 8)
        self.assertTrue(state["signal_valid"])
        self.assertFalse(state["hardware"]["actuation_enabled"])
        self.assertIsNone(state["reference"])
        json.dumps(state, allow_nan=False)

    def test_auto_requires_personal_reference(self):
        with self.assertRaises(ValueError):
            self.action("auto_start")

    def test_three_second_calibration_establishes_reference(self):
        self.action("calibrate")
        state = self.advance(1.0)
        self.assertTrue(state["calibration"]["active"])
        self.assertIsNone(state["reference"])
        with self.assertRaises(ValueError):
            self.action("auto_start")
        state = self.advance(2.2)
        self.assertFalse(state["calibration"]["active"])
        self.assertIsNotNone(state["reference"])
        self.assertEqual(len(state["reference"]["zones"]), 8)
        self.assertEqual(len(state["reference"]["noise"]), 8)
        self.assertEqual(self.action("auto_start")["mode"], "auto")

    def test_calibration_rejects_too_few_new_frames(self):
        self.action("calibrate")
        state = self.advance(3.1, step=3.1)
        self.assertIsNone(state["reference"])
        self.assertFalse(state["calibration"]["active"])

    def test_calibration_cancels_on_loss_of_contact(self):
        self.action("calibrate")
        self.advance(1.0)
        self.action("scenario", name="contact_loss")
        state = self.advance(0.2)
        self.assertFalse(state["calibration"]["active"])
        self.assertIsNone(state["reference"])

    def test_manual_action_cancels_active_calibration(self):
        self.action("calibrate")
        state = self.action("manual", direction="advance")
        self.assertFalse(state["calibration"]["active"])
        self.advance(3.3)
        self.assertIsNone(self.service.state()["reference"])

    def test_manual_motion_is_a_small_step_and_stop_is_immediate(self):
        before = self.service.state()["position"]
        self.action("manual", direction="advance")
        state = self.advance(1.0, step=0.02)
        self.assertGreater(state["position"], before)
        self.assertLessEqual(state["position"] - before, 2.25)
        stopped = state["position"]
        self.assertAlmostEqual(self.advance(1.0)["position"], stopped, places=2)
        self.action("manual", direction="retract")
        self.advance(0.1)
        self.action("stop")
        stopped = self.service.state()["position"]
        self.assertAlmostEqual(self.advance(0.5)["position"], stopped, places=2)

    def test_closed_loop_converges_and_holds_through_forward_backward_and_local_load(self):
        self.calibrate()
        self.action("auto_start")
        last_position = self.service.state()["position"]
        for scenario, direction in (("forward", "advance"), ("backward", "retract"), ("left", "retract")):
            with self.subTest(scenario=scenario):
                self.action("scenario", name=scenario)
                decisions = []
                for _ in range(400):
                    self.clock.return_value += 0.1
                    self.service.tick()
                    decisions.append(self.service.controller.snapshot())
                state = self.service.state()
                self.assertIn(direction, {s["action"] for s in decisions})
                self.assertTrue(all(s["mode"] == "auto" and s["contact"] for s in decisions))
                self.assertEqual({s["action"] for s in decisions[-50:]}, {"hold"})
                if scenario == "forward":
                    self.assertGreater(state["position"], last_position)
                else:
                    self.assertLess(state["position"], last_position)
                if scenario != "left":
                    for snapshot in decisions[-50:]:
                        metrics = snapshot["metrics"]
                        self.assertGreaterEqual(metrics["lumbar"], metrics["lower"])
                        self.assertLessEqual(metrics["lumbar"], metrics["upper"])
                last_position = state["position"]

    def test_fault_recovery_requires_explicit_auto_restart(self):
        for scenario in ("away", "contact_loss", "dropout", "invalid"):
            with self.subTest(scenario=scenario):
                self.action("connect", source="simulation")
                self.advance(0.2)
                self.calibrate()
                self.action("auto_start")
                self.action("scenario", name=scenario)
                self.assertEqual(self.advance(1.2)["mode"], "paused")
                self.action("scenario", name="neutral")
                state = self.advance(1.0)
                self.assertEqual(state["mode"], "paused")
                self.assertNotIn(state["action"], ("advance", "retract"))
                self.assertEqual(self.action("auto_start")["mode"], "auto")

    def test_session_roundtrip_preserves_inputs_context_and_feedback(self):
        self.calibrate()
        started = self.action("record_start")
        sid = started["recording"]["id"]
        self.action("auto_start")
        self.action("scenario", name="forward")
        self.advance(1.5)
        self.action("feedback", value="too_much")
        self.action("parameters", values={"speed": 3.0})
        self.advance(0.2)
        self.action("record_stop")
        session = self.service.session(sid)
        self.assertEqual(session["schema_version"], 1)
        self.assertIsNotNone(session["ended_at"])
        self.assertIsNotNone(session["reference"])
        self.assertGreater(len(session["frames"]), 10)
        self.assertTrue(all("raw_zones" in frame for frame in session["frames"]))
        self.assertTrue(all("history" not in frame for frame in session["frames"]))
        self.assertEqual(session["frames"][-1]["parameters"]["speed"], 3.0)
        self.assertTrue(any(e["type"] == "feedback" and e["detail"]["value"] == "too_much"
                            for e in session["events"]))
        self.assertTrue(any(e["type"] == "parameters" for e in session["events"]))
        times = [frame["time"] for frame in session["frames"]]
        self.assertEqual(times, sorted(times))
        saved = json.loads((Path(self.directory.name) / (sid + ".json")).read_text(encoding="utf-8"))
        self.assertEqual(saved, session)
        reopened = AppService(data_dir=Path(self.directory.name))
        try:
            self.assertEqual(reopened.session(sid), session)
            summary = reopened.sessions()["sessions"][0]
            self.assertEqual(summary["id"], sid)
            self.assertEqual(summary["samples"], len(session["frames"]))
            self.assertEqual(summary["feedback_count"], 1)
        finally:
            reopened.close()

    def test_feedback_requires_a_recording_and_valid_value(self):
        with self.assertRaises(ValueError):
            self.action("feedback", value="comfortable")
        self.action("record_start")
        with self.assertRaises(ValueError):
            self.action("feedback", value="unknown")

    def test_device_switch_is_rejected_during_recording(self):
        self.action("record_start")
        with self.assertRaises(ValueError):
            self.action("connect", source="simulation")
        self.assertTrue(self.service.state()["recording"]["active"])

    def test_switch_device_discards_old_personal_reference(self):
        self.calibrate()
        state = self.action("connect", source="simulation")
        self.assertIsNone(state["reference"])
        self.assertNotEqual(state["mode"], "auto")

    def test_invalid_connection_settings_leave_existing_device_available(self):
        original = self.service.device
        with patch.object(original, "close", wraps=original.close) as close:
            with self.assertRaises(ValueError):
                self.action("connect", source="serial", port="COM9", baud=123)
            close.assert_not_called()
        self.assertIs(self.service.device, original)
        self.assertTrue(self.advance(0.1)["signal_valid"])

    def test_failed_serial_connection_keeps_old_source_safely_stopped(self):
        self.calibrate()
        self.action("auto_start")
        original = self.service.device
        with patch("lumbar.service.SerialSensor") as serial_class:
            serial_class.return_value.connect.side_effect = OSError("test port unavailable")
            with self.assertRaises(ValueError):
                self.action("connect", source="serial", port="COM9")
            serial_class.return_value.close.assert_called_once()
        self.assertIs(self.service.device, original)
        state = self.advance(0.2)
        self.assertEqual(state["source"], "simulation")
        self.assertEqual(state["mode"], "paused")
        self.assertEqual(state["action"], "stop")
        self.assertTrue(state["signal_valid"])

    def test_serial_source_allows_suggestions_but_rejects_manual_actuation(self):
        with patch("lumbar.service.SerialSensor") as serial_class:
            sensor = serial_class.return_value
            sensor.port = "COM9"
            sensor.mapping_ready = True
            sensor.message = "test sensor"
            sensor.connected = True
            sensor.read.side_effect = lambda: SensorFrame(
                zones=[45.0] * 8, raw_zones=[50.0] * 8, matrix=None, raw_matrix=None,
                seq=int(self.clock.return_value * 10), device_time_ms=int(self.clock.return_value * 1000),
                received_at=self.clock.return_value, valid=True)
            self.action("connect", source="serial", port="COM9")
            self.advance(0.1)
            self.calibrate()
            state = self.action("auto_start")
            self.assertEqual(state["mode"], "auto")
            self.assertIsNone(state["position"])
            self.assertFalse(state["hardware"]["actuation_enabled"])
            with self.assertRaises(ValueError):
                self.action("manual", direction="advance")
            with self.assertRaises(ValueError):
                self.action("zero")
            sensor.zero.assert_not_called()
            self.action("zero", confirmed_empty=True)
            sensor.zero.assert_called_once()
            self.assertIsNone(self.service.state()["reference"])
            sensor.write.assert_not_called()

    def test_state_is_a_detached_copy(self):
        self.calibrate()
        state = self.service.state()
        state["reference"]["zones"][4] = -999
        state["parameters"]["speed"] = 999
        self.assertGreater(self.service.state()["reference"]["zones"][4], 0)
        self.assertLess(self.service.state()["parameters"]["speed"], 999)

    def test_invalid_session_ids_cannot_access_other_files(self):
        for sid in ("../README", "..\\README", "C:\\Windows", "", None):
            with self.subTest(sid=sid), self.assertRaises(ValueError):
                self.service.session(sid)

    def test_invalid_actions_and_parameters_are_rejected(self):
        for payload in (None, [], {"action": "unknown"}, {"action": "parameters", "values": []},
                        {"action": "parameters", "values": {"speed": 1000}}):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                self.service.action(payload)


if __name__ == "__main__":
    unittest.main()
