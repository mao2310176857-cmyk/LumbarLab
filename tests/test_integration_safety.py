"""Regressions at controller/service boundaries; deterministic fake hardware."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from lumbar.actuators import ActuatorCommand, ActuationDisabledError, DisabledActuator
from lumbar.controller import Controller
from lumbar.devices import SensorFrame
from lumbar.service import AppService


ZONES = [45., 48., 62., 64., 62., 70., 65., 42.]


class StubSerial:
    mapping_ready = True
    port = "TEST"
    message = "test serial"

    def __init__(self):
        self.connected = True
        self.next_frame = None
        self.failure = None

    def read(self):
        if self.failure is not None:
            raise self.failure
        frame, self.next_frame = self.next_frame, None
        return frame

    def close(self):
        self.connected = False


class SafetyIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.app = AppService(data_dir=Path(self.temp.name))
        self.addCleanup(self.app.close)
        self.serial = StubSerial()
        self.app.device.close()
        self.app.device = self.serial
        self.app.source = "serial"
        self.t = self.app.started + 1.0
        self.serial.next_frame = SensorFrame(ZONES[:], ZONES[:], [10]*256, [20]*256,
                                            1, 1000, self.t, True)
        self.app.tick(self.t)
        self.app.controller.set_reference([ZONES[:]]*30)

    def test_known_disconnect_immediately_invalidates_cached_frame(self):
        self.serial.connected = False
        self.app.tick(self.t + .05)
        self.assertFalse(self.app.state()["signal_valid"])
        with patch("lumbar.service.time.monotonic", return_value=self.t + .06):
            with self.assertRaises(ValueError):
                self.app.action({"action": "auto_start"})

    def test_state_ages_when_control_loop_stops_ticking(self):
        with patch("lumbar.service.time.monotonic", return_value=self.t + .1):
            self.assertTrue(self.app.state()["signal_valid"])
        # No tick occurs between reads: the display still must use live time.
        with patch("lumbar.service.time.monotonic", return_value=self.t + 1.0):
            state = self.app.state()
        self.assertFalse(state["signal_valid"])
        self.assertFalse(state["connected"])
        self.assertEqual(state["frame_age_ms"], 1000)

    def test_fatal_loop_failure_latches_until_restart_but_stop_remains_allowed(self):
        self.app.controller.start_auto()
        self.app.fail("control loop failed; restart required")
        self.assertFalse(self.app.state()["signal_valid"])
        self.assertFalse(self.serial.connected)
        self.assertEqual(self.app.state()["action"], "stop")
        for action in ("auto_start", "calibrate", "connect"):
            with self.assertRaisesRegex(ValueError, "restart required"):
                self.app.action({"action": action, "source": "simulation"})
        stopped = self.app.action({"action": "stop"})
        self.assertEqual(stopped["action"], "stop")
        with self.assertRaisesRegex(ValueError, "restart required"):
            self.app.action({"action": "auto_start"})

    def test_read_exception_invalidates_cached_frame(self):
        self.serial.failure = OSError("USB lost")
        self.app.controller.start_auto()
        self.app.tick(self.t + .05)
        state = self.app.state()
        self.assertEqual(state["mode"], "paused")
        self.assertFalse(state["signal_valid"])
        with patch("lumbar.service.time.monotonic", return_value=self.t + .06):
            with self.assertRaises(ValueError):
                self.app.action({"action": "auto_start"})

    def test_read_failure_cancels_calibration_and_cannot_capture_cached_values(self):
        self.app.calibration = {"active": True, "progress": .9, "message": "sampling"}
        self.app.calibration_start = self.t - 2.96
        self.app.calibration_samples = [ZONES[:]]*30
        previous_reference = self.app.controller.reference
        self.serial.failure = OSError("USB lost")
        self.app.tick(self.t + .05)
        self.assertFalse(self.app.calibration["active"])
        self.assertEqual(self.app.calibration["progress"], 0)
        self.assertIs(self.app.controller.reference, previous_reference)

    def test_invalid_serial_frame_is_recorded_as_invalid_without_old_matrix(self):
        with patch("lumbar.service.time.monotonic", return_value=self.t):
            self.app.action({"action": "record_start"})
        self.serial.next_frame = SensorFrame(None, None, None, None, 2, 1010, self.t + .01, False)
        self.app.tick(self.t + .01)
        sample = self.app.record["frames"][-1]
        self.assertFalse(sample["signal_valid"])
        self.assertIsNone(sample["matrix"])
        self.assertIsNone(sample["raw_matrix"])

    def test_reset_invalid_frame_pauses_and_does_not_resume_without_action(self):
        self.app.controller.start_auto()
        self.serial.next_frame = SensorFrame(None, None, None, None, 0, 0, self.t + .01, False)
        self.app.tick(self.t + .01)
        self.assertEqual(self.app.state()["mode"], "paused")
        self.serial.next_frame = SensorFrame(ZONES[:], ZONES[:], [10]*256, [20]*256,
                                            1, 10, self.t + .02, True)
        self.app.tick(self.t + .02)
        self.assertEqual(self.app.state()["mode"], "paused")


class ControllerFreshnessTests(unittest.TestCase):
    def test_persistence_cannot_start_new_motion_without_new_input(self):
        controller = Controller()
        controller.set_reference([ZONES[:]]*30)
        controller.start_auto()
        low = ZONES[:4] + [35., 35., 35.] + ZONES[7:]
        # Drive the low-pass filter below the lower band using real new samples.
        for n in range(6):
            controller.update(low, n*.01, 45, fresh=True)
        self.assertEqual(controller.action, "hold")
        controller.update(low, .7, 45, fresh=False, age=.65)
        self.assertEqual(controller.action, "hold")
        controller.update(low, .71, 45, fresh=True, age=0)
        self.assertEqual(controller.action, "advance")


class ActuatorBoundaryTests(unittest.TestCase):
    def test_default_real_motion_is_rejected(self):
        actuator = DisabledActuator()
        self.assertFalse(actuator.enabled)
        for action in ("advance", "retract"):
            command = ActuatorCommand(action, issued_at=1, speed=5, duration_s=.8)
            with self.assertRaises(ActuationDisabledError):
                actuator.execute(command)

    def test_disabled_stop_does_not_claim_a_hardware_ack(self):
        result = DisabledActuator().execute(ActuatorCommand("stop", issued_at=1))
        self.assertFalse(result.sent)
        self.assertFalse(result.acknowledged)

    def test_unbounded_or_nonfinite_commands_rejected(self):
        for speed, duration in ((5, 0), (0, .8), (5, 3), (float("nan"), .8), (5, float("inf"))):
            with self.assertRaises(ValueError):
                ActuatorCommand("advance", issued_at=1, speed=speed, duration_s=duration)


if __name__ == "__main__":
    unittest.main()
