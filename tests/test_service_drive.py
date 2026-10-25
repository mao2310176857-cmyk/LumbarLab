"""Software-only drive integration with fake serial input and fake actuators.

No serial device, physical motor, or hardware stop is exercised by these tests.
"""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from lumbar.actuators import ActuatorResult, DisabledActuator, SimulationActuator
from lumbar.devices import SensorFrame
from lumbar.service import AppService


REFERENCE = [45., 48., 62., 64., 62., 70., 65., 42.]


class FakeMotor:
    enabled = True
    is_hardware = True  # Exercise the hardware branch; this is a test double.

    def __init__(self, timeline):
        self.timeline = timeline
        self.commands = []
        self.failure = None
        self.acknowledge = True

    def execute(self, command):
        self.commands.append(command)
        self.timeline.append(("motor", command.action))
        if command.action in ("advance", "retract"):
            if self.failure:
                raise self.failure
            return ActuatorResult(True, self.acknowledge, "fake movement response")
        return ActuatorResult(True, True, "fake stop/hold response")

    def close(self):
        self.timeline.append(("motor", "close"))


class FakeSerial:
    mapping_ready = True
    message = "fake sensor, no physical serial I/O"

    def __init__(self, port, clock, timeline):
        self.port = port
        self.clock = clock
        self.timeline = timeline
        self.connected = False
        self.zones = REFERENCE[:]
        self.seq = 0
        self.writes = []

    def connect(self):
        self.connected = True
        self.timeline.append(("sensor", "connect"))

    def read(self):
        self.seq += 1
        now = self.clock.return_value
        return SensorFrame(self.zones[:], self.zones[:], None, None,
                           self.seq, int(now * 1000), now, self.connected)

    def zero(self):
        self.timeline.append(("sensor", "zero"))

    def close(self):
        self.connected = False
        self.timeline.append(("sensor", "close"))

    def write(self, data):
        self.writes.append(data)
        raise AssertionError("The sensor must not be used as an actuator transport")


class ServiceDriveTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        clock_patch = patch("lumbar.service.time.monotonic", return_value=100.)
        self.clock = clock_patch.start()
        self.addCleanup(clock_patch.stop)
        self.timeline = []
        self.motors = []
        self.sensors = []
        sensor_patch = patch("lumbar.service.SerialSensor", side_effect=self.make_sensor)
        sensor_patch.start()
        self.addCleanup(sensor_patch.stop)
        self.app = AppService(data_dir=Path(self.directory.name), actuator_factory=self.make_actuator)
        self.addCleanup(self.app.close)
        self.connect_serial()

    def make_sensor(self, port, baud, mapping_path=None):
        sensor = FakeSerial(port, self.clock, self.timeline)
        self.sensors.append(sensor)
        return sensor

    def make_actuator(self, source, device):
        if source == "simulation":
            return SimulationActuator(device)
        motor = FakeMotor(self.timeline)
        self.motors.append(motor)
        return motor

    @property
    def motor(self):
        return self.motors[-1]

    @property
    def sensor(self):
        return self.sensors[-1]

    def action(self, action, **values):
        return self.app.action({"action": action, **values})

    def tick(self, count=1, step=.1):
        for _ in range(count):
            self.clock.return_value += step
            self.app.tick()
        return self.app.state()

    def connect_serial(self):
        self.action("connect", source="serial", port="FAKE-COM")
        self.tick()

    def start_auto(self):
        # Directly seed a validated stable reference; no manual movement occurs.
        self.app.controller.set_reference([REFERENCE[:] for _ in range(30)])
        self.action("auto_start")

    def wait_for_motion(self, action):
        start = len(self.motor.commands)
        for _ in range(40):
            self.tick()
            if any(command.action == action for command in self.motor.commands[start:]):
                return next(command for command in reversed(self.motor.commands) if command.action == action)
        self.fail(f"automatic {action} command was not dispatched")

    def begin_motion(self):
        self.start_auto()
        self.sensor.zones[4:7] = [30., 30., 30.]
        return self.wait_for_motion("advance")

    def test_serial_pressure_changes_automatically_dispatch_both_directions(self):
        self.start_auto()
        self.sensor.zones[4:7] = [30., 30., 30.]
        advance = self.wait_for_motion("advance")
        self.sensor.zones[4:7] = [110., 110., 110.]
        retract = self.wait_for_motion("retract")
        self.assertGreater(retract.issued_at, advance.issued_at)
        state = self.app.state()
        self.assertEqual(state["mode"], "auto")
        self.assertTrue(state["hardware"]["actuation_enabled"])
        self.assertTrue(state["actuator"]["acknowledged"])
        self.assertIsNone(state["position"])
        self.assertFalse(any(event["type"] == "manual" for event in state["events"]))
        self.assertEqual(self.sensor.writes, [])

    def test_repeated_ticks_do_not_resend_or_extend_the_motion_episode(self):
        command = self.begin_motion()
        count = len(self.motor.commands)
        sequence = self.app.state()["actuator"]["command"]["seq"]
        self.tick(count=5, step=.05)
        self.assertEqual(len(self.motor.commands), count)
        self.assertEqual(self.app.state()["actuator"]["command"]["seq"], sequence)
        self.assertAlmostEqual(self.app.controller.motion_until, command.expires_at)
        self.tick(count=12, step=.05)
        self.assertEqual(self.motor.commands[-1].action, "hold")
        moves = [item for item in self.motor.commands if item.action in ("advance", "retract")]
        self.assertEqual(moves, [command])

    def test_manual_serial_control_uses_the_injected_adapter(self):
        state = self.action("manual", direction="advance")
        self.assertEqual(state["mode"], "manual")
        self.assertEqual(self.motor.commands[-1].action, "advance")
        self.assertEqual(self.sensor.writes, [])

    def test_manual_direction_changes_share_the_original_end_time(self):
        self.action("manual", direction="advance")
        self.tick(step=.05)
        self.action("manual", direction="retract")
        self.tick(step=.05)
        self.action("manual", direction="advance")
        moves = [command for command in self.motor.commands if command.action in ("advance", "retract")]
        self.assertEqual([command.action for command in moves], ["advance", "retract", "advance"])
        for command in moves[1:]:
            self.assertAlmostEqual(command.expires_at, moves[0].expires_at)
        self.assertGreater(moves[1].issued_at, moves[0].issued_at)
        self.assertGreater(moves[2].issued_at, moves[1].issued_at)
        self.assertLess(moves[2].duration_s, moves[1].duration_s)
        self.tick(count=7, step=.05)
        self.assertEqual(self.motor.commands[-1].action, "hold")
        self.assertEqual(len([command for command in self.motor.commands
                              if command.action in ("advance", "retract")]), 3)

    def test_pause_stop_calibration_and_parameter_changes_stop_the_adapter(self):
        for operation, values in (("auto_pause", {}), ("stop", {}), ("calibrate", {}),
                                  ("parameters", {"values": {"speed": 3.}})):
            with self.subTest(operation=operation):
                self.tick()
                self.action("manual", direction="advance")
                self.timeline.clear()
                self.action(operation, **values)
                self.assertEqual(self.timeline[0], ("motor", "stop"))
                self.assertEqual(self.motor.commands[-1].action, "stop")

    def test_zero_stops_before_touching_sensor_state(self):
        self.begin_motion()
        self.timeline.clear()
        state = self.action("zero", confirmed_empty=True)
        self.assertEqual(self.timeline[:2], [("motor", "stop"), ("sensor", "zero")])
        self.assertIsNone(state["reference"])

    def test_reconnect_stops_old_motor_before_closing_old_connections(self):
        self.begin_motion()
        old_motor, old_sensor = self.motor, self.sensor
        self.timeline.clear()
        self.connect_serial()
        self.assertEqual(self.timeline[0], ("motor", "stop"))
        self.assertLess(self.timeline.index(("motor", "stop")), self.timeline.index(("motor", "close")))
        self.assertLess(self.timeline.index(("motor", "stop")), self.timeline.index(("sensor", "close")))
        self.assertIsNot(self.motor, old_motor)
        self.assertFalse(old_sensor.connected)
        self.assertEqual(self.app.controller.mode, "paused")
        self.assertIsNone(self.app.controller.reference)

    def test_failure_and_close_stop_before_releasing_devices(self):
        for operation in ("fail", "close"):
            with self.subTest(operation=operation):
                self.timeline.clear()
                if operation == "fail":
                    self.app.fail("fake loop failure")
                else:
                    self.app.close()
                self.assertEqual(self.timeline[:3], [("motor", "stop"), ("motor", "close"), ("sensor", "close")])

    def test_driver_failure_latches_until_reconnect(self):
        self.motor.failure = OSError("fake transport failure")
        self.start_auto()
        self.sensor.zones[4:7] = [30., 30., 30.]
        self.tick(count=15)
        self.assert_fault_latched_and_reconnect_recovers("fake transport failure")

    def test_missing_acknowledgement_latches_until_reconnect(self):
        self.motor.acknowledge = False
        self.start_auto()
        self.sensor.zones[4:7] = [30., 30., 30.]
        self.tick(count=15)
        self.assert_fault_latched_and_reconnect_recovers("未确认")

    def assert_fault_latched_and_reconnect_recovers(self, message):
        state = self.app.state()
        self.assertEqual(state["mode"], "paused")
        self.assertEqual(self.motor.commands[-1].action, "stop")
        self.assertIn(message, state["actuator"]["fault"])
        self.assertFalse(state["hardware"]["actuation_enabled"])
        for operation in ("auto_start", "manual"):
            with self.assertRaisesRegex(ValueError, "执行器故障"):
                self.action(operation, direction="advance")
        self.motor.failure = None
        self.motor.acknowledge = True
        self.action("stop")
        self.assertIsNotNone(self.app.state()["actuator"]["fault"])
        self.connect_serial()
        self.assertIsNone(self.app.state()["actuator"]["fault"])
        self.start_auto()
        self.assertEqual(self.app.controller.mode, "auto")

    def test_default_serial_adapter_never_writes_motor_commands(self):
        # Restore the default factory, then reconnect to select DisabledActuator.
        self.app.actuator_factory = AppService._default_actuator
        self.connect_serial()
        self.assertIsInstance(self.app.output.actuator, DisabledActuator)
        self.start_auto()
        self.sensor.zones[4:7] = [30., 30., 30.]
        self.tick(count=12)
        state = self.app.state()
        self.assertEqual(state["action"], "advance")
        self.assertFalse(state["hardware"]["actuation_enabled"])
        self.assertFalse(state["actuator"]["sent"])
        self.assertFalse(state["actuator"]["acknowledged"])
        self.assertEqual(self.sensor.writes, [])
        with self.assertRaisesRegex(ValueError, "尚未接入"):
            self.action("manual", direction="advance")


if __name__ == "__main__":
    unittest.main()
