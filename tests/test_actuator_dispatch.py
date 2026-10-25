import unittest
from unittest.mock import Mock

from lumbar.actuators import (
    ActuatorCommand, ActuatorDispatcher, ActuatorResult,
    DisabledActuator, SimulationActuator,
)
from lumbar.simulation import SimulationDevice


class RecordingActuator:
    enabled = True
    is_hardware = True

    def __init__(self):
        self.commands = []
        self.error = None

    def execute(self, command):
        self.commands.append(command)
        if self.error:
            raise self.error
        return ActuatorResult(True, True, "测试适配器确认")


class ActuatorDispatchTests(unittest.TestCase):
    def setUp(self):
        self.actuator = RecordingActuator()
        self.dispatcher = ActuatorDispatcher(self.actuator)

    def test_repeated_movement_keeps_its_original_deadline(self):
        for now in (10, 10.1, 10.4, 10.7):
            self.dispatcher.dispatch(ActuatorCommand("advance", 10, 5, .8), now)
        self.assertEqual(len(self.actuator.commands), 1)
        self.assertEqual(self.actuator.commands[0].expires_at, 10.8)
        self.assertEqual(self.dispatcher.snapshot()["command"]["seq"], 1)
        self.dispatcher.dispatch(ActuatorCommand("advance", 10, 5, .8), 10.8)
        self.assertEqual([command.action for command in self.actuator.commands], ["advance", "stop"])
        state = self.dispatcher.snapshot()
        self.assertEqual(state["requested_action"], "advance")
        self.assertEqual(state["command"]["action"], "stop")
        self.assertIn("过期", state["message"])
        self.dispatcher.dispatch(ActuatorCommand("advance", 10, 5, .8), 11)
        self.assertEqual(len(self.actuator.commands), 2)
        self.assertEqual(self.dispatcher.snapshot()["message"], state["message"])

    def test_expired_movement_is_never_sent(self):
        self.dispatcher.dispatch(ActuatorCommand("retract", 2, 3, .5), 3)
        self.assertEqual([command.action for command in self.actuator.commands], ["stop"])
        self.assertEqual(self.actuator.commands[0].speed, 0)

    def test_stopped_movement_cannot_be_replayed_before_expiry(self):
        move = ActuatorCommand("advance", 10, 5, .8)
        self.dispatcher.dispatch(move, 10)
        self.dispatcher.force_stop(10.1)
        self.dispatcher.dispatch(move, 10.2)
        self.assertEqual([command.action for command in self.actuator.commands], ["advance", "stop"])
        self.assertEqual(self.dispatcher.snapshot()["command"]["action"], "stop")

    def test_replayed_old_movement_stops_a_new_opposite_direction(self):
        forward = ActuatorCommand("advance", 10, 5, .8)
        reverse = ActuatorCommand("retract", 10.1, 5, .6)
        self.dispatcher.dispatch(forward, 10)
        self.dispatcher.dispatch(reverse, 10.1)
        self.dispatcher.dispatch(forward, 10.2)
        self.assertEqual([command.action for command in self.actuator.commands], ["advance", "retract", "stop"])
        state = self.dispatcher.snapshot()
        self.assertEqual(state["requested_action"], "advance")
        self.assertEqual(state["command"]["action"], "stop")
        self.assertIn("已取消或被替代", state["message"])
        self.dispatcher.dispatch(forward, 10.3)
        self.assertEqual(len(self.actuator.commands), 3)
        self.assertEqual(self.dispatcher.snapshot()["message"], state["message"])

    def test_idle_deduplicates_but_forced_stop_is_always_dispatched(self):
        for now in (1, 2):
            self.dispatcher.dispatch(ActuatorCommand("hold", now), now)
        for now in (3, 4):
            self.dispatcher.dispatch(ActuatorCommand("stop", now), now)
        self.dispatcher.force_stop(5)
        self.dispatcher.force_stop(6)
        self.assertEqual([command.action for command in self.actuator.commands], ["hold", "stop", "stop", "stop"])
        self.assertEqual(self.dispatcher.snapshot()["command"]["seq"], 4)

    def test_driver_exception_is_reported_and_survives_successful_stop(self):
        self.actuator.error = OSError("transport failure")
        with self.assertRaisesRegex(OSError, "transport failure"):
            self.dispatcher.dispatch(ActuatorCommand("advance", 1, 5, .8), 1)
        state = self.dispatcher.snapshot()
        self.assertEqual(state["fault"], "transport failure")
        self.assertFalse(state["sent"])
        self.assertFalse(state["acknowledged"])
        self.actuator.error = None
        self.dispatcher.force_stop(1.1)
        state = self.dispatcher.snapshot()
        self.assertEqual(state["fault"], "transport failure")
        self.assertTrue(state["acknowledged"])
        self.assertEqual(state["command"]["action"], "stop")

    def test_disabled_stop_does_not_report_a_hardware_ack(self):
        dispatcher = ActuatorDispatcher(DisabledActuator())
        dispatcher.force_stop(1)
        self.assertFalse(dispatcher.snapshot()["enabled"])
        self.assertFalse(dispatcher.snapshot()["sent"])
        self.assertFalse(dispatcher.snapshot()["acknowledged"])

    def test_snapshot_cannot_mutate_stored_command(self):
        self.dispatcher.force_stop(1)
        self.dispatcher.snapshot()["command"]["action"] = "advance"
        self.assertEqual(self.dispatcher.snapshot()["command"]["action"], "stop")

    def test_simulated_movement_and_stop_use_the_same_interface(self):
        device = SimulationDevice()
        device.set_motion = Mock(wraps=device.set_motion)
        dispatcher = ActuatorDispatcher(SimulationActuator(device))
        dispatcher.dispatch(ActuatorCommand("advance", 1, 5, .8), 1)
        device.set_motion.assert_called_once_with("advance", 5)
        initial = device.position
        device.read(1)
        self.assertGreater(device.position, initial)
        dispatcher.force_stop(1.1)
        device.set_motion.assert_called_with("stop", 0)
        stopped = device.position
        device.read(1.2)
        self.assertEqual(device.position, stopped)
        self.assertFalse(dispatcher.snapshot()["is_hardware"])
        self.assertIn("模拟", dispatcher.snapshot()["message"])
        dispatcher.dispatch(ActuatorCommand("hold", 1.3), 1.3)
        device.set_motion.assert_called_with("hold", 0)


if __name__ == "__main__":
    unittest.main()
