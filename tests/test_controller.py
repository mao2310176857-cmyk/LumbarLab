"""Control behavior and closed-loop simulation tests, independent of hardware."""
import unittest

from lumbar.controller import Controller, valid_zones
from lumbar.simulation import SimulationDevice


REFERENCE = [40, 40, 50, 50, 50, 50, 50, 30]


def pressure(lumbar=50, seat=45):
    return [seat] * 4 + [lumbar] * 3 + [30]


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = Controller()
        self.now = 10.0

    def ready(self):
        self.controller.set_reference([REFERENCE[:] for _ in range(30)], 45)
        self.controller.update(REFERENCE, self.now, 45)
        self.controller.start_auto()

    def step(self, zones, dt=0.1, **kwargs):
        self.now += dt
        return self.controller.update(zones, self.now, kwargs.pop("position", 45), **kwargs)

    def until_action(self, zones, action, limit=50):
        for _ in range(limit):
            state = self.step(zones)
            if state["action"] == action:
                return state
        self.fail(f"Expected {action} for input {zones}, got {state}")

    def test_reference_uses_stable_personal_measurements(self):
        samples = [[value + (0.5 if n % 2 else -0.5) for value in REFERENCE] for n in range(30)]
        self.controller.set_reference(samples, 42)
        self.assertEqual(self.controller.reference["zones"], REFERENCE)
        self.assertEqual(self.controller.reference["noise"], [0.5] * 8)
        self.assertEqual(self.controller.reference["position"], 42)
        self.assertEqual(self.controller.mode, "paused")

    def test_reference_rejects_absence_contact_loss_saturation_and_instability(self):
        cases = {
            "not enough samples": [REFERENCE] * 19,
            "empty seat": [pressure(seat=0)] * 30,
            "missing lumbar zone": [[45] * 4 + [50, 0, 50, 30]] * 30,
            "saturated": [pressure(245)] * 30,
            "unstable": [pressure(30 if n % 2 else 70) for n in range(30)],
        }
        for reason, samples in cases.items():
            with self.subTest(reason=reason), self.assertRaises(ValueError):
                self.controller.set_reference(samples, 45)
        self.assertIsNone(self.controller.reference)

    def test_failed_new_calibration_preserves_previous_reference(self):
        self.ready()
        previous = self.controller.reference.copy()
        with self.assertRaises(ValueError):
            self.controller.set_reference([pressure(0)] * 30, 45)
        self.assertEqual(self.controller.reference, previous)

    def test_auto_without_reference_is_rejected(self):
        with self.assertRaises(ValueError):
            self.controller.start_auto()

    def test_normal_contact_stays_held(self):
        self.ready()
        for _ in range(50):
            state = self.step(REFERENCE)
            self.assertEqual(state["action"], "hold")
        self.assertLess(state["metrics"]["lower"], state["metrics"]["target"])
        self.assertGreater(state["metrics"]["upper"], state["metrics"]["target"])

    def test_short_dip_does_not_move_but_persistent_low_contact_advances(self):
        self.ready()
        self.assertEqual(self.step(pressure(25), dt=0.05)["action"], "hold")
        for _ in range(15):
            self.assertEqual(self.step(REFERENCE)["action"], "hold")
        self.assertEqual(self.until_action(pressure(25), "advance")["mode"], "auto")

    def test_persistent_high_contact_retracts(self):
        self.ready()
        self.until_action(pressure(80), "retract")

    def test_local_high_contact_retracts_even_with_normal_average(self):
        self.ready()
        local = [45] * 4 + [82, 34, 34, 30]
        self.assertAlmostEqual(sum(local[4:7]) / 3, 50)
        self.until_action(local, "retract")

    def test_advance_hysteresis_keeps_moving_inside_outer_band(self):
        self.ready()
        self.until_action(pressure(25), "advance")
        state = self.step(pressure(80), dt=0.02)
        self.assertGreater(state["metrics"]["lumbar"], state["metrics"]["lower"])
        self.assertLess(state["metrics"]["lumbar"], state["metrics"]["target"])
        self.assertEqual(state["action"], "advance")
        state = self.step(pressure(60), dt=0.02)
        self.assertEqual(state["action"], "hold")

    def test_motion_is_bounded_and_followed_by_settling(self):
        self.ready()
        self.until_action(pressure(25), "advance")
        state = self.step(pressure(25), dt=0.81)
        self.assertEqual(state["action"], "hold")
        for _ in range(4):
            self.assertEqual(self.step(pressure(25))["action"], "hold")
        self.until_action(pressure(25), "advance")

    def test_no_contact_absence_invalid_stale_and_saturated_input_latch_pause(self):
        cases = [
            (pressure(0), {}),
            (pressure(seat=0), {}),
            (pressure(), {"valid": False}),
            (pressure(), {"age": 0.81, "fresh": False}),
            (pressure(245), {}),
            ([float("nan")] + REFERENCE[1:], {}),
            ([float("inf")] + REFERENCE[1:], {}),
            ([-1] + REFERENCE[1:], {}),
            ([256] + REFERENCE[1:], {}),
        ]
        for zones, kwargs in cases:
            with self.subTest(zones=zones, kwargs=kwargs):
                self.ready()
                state = self.step(zones, **kwargs)
                self.assertEqual(state["mode"], "paused")
                self.assertEqual(state["action"], "stop")
                for _ in range(10):
                    self.assertEqual(self.step(REFERENCE)["mode"], "paused")
                self.controller.start_auto()
                self.assertEqual(self.step(REFERENCE)["mode"], "auto")

    def test_fault_also_interrupts_manual_movement(self):
        self.controller.manual("advance", self.now)
        self.assertEqual(self.step(pressure(), valid=False)["action"], "stop")
        self.assertEqual(self.step(pressure())["mode"], "paused")

    def test_manual_commands_are_bounded_and_repeated_clicks_do_not_extend(self):
        self.ready()
        self.controller.manual("advance", self.now)
        self.assertEqual(self.step(REFERENCE, dt=0.2)["mode"], "manual")
        self.controller.manual("advance", self.now)
        self.assertEqual(self.step(REFERENCE, dt=0.21)["action"], "hold")
        with self.assertRaises(ValueError):
            self.controller.manual("unbounded", self.now)

    def test_stop_remains_stopped_after_signal_changes(self):
        self.ready()
        self.until_action(pressure(25), "advance")
        self.controller.stop()
        for _ in range(20):
            self.assertEqual(self.step(pressure(25))["action"], "stop")

    def test_position_limit_stops_inward_motion(self):
        for direction, limit in (("advance", 100), ("retract", 0)):
            with self.subTest(direction=direction):
                self.controller.manual(direction, self.now)
                state = self.step(REFERENCE, position=limit)
                self.assertEqual(state["mode"], "paused")
                self.assertEqual(state["action"], "stop")

    def test_parameter_update_is_atomic_and_requires_auto_restart(self):
        self.ready()
        old = self.controller.parameters.copy()
        with self.assertRaises(ValueError):
            self.controller.set_parameters({"speed": 3, "tolerance": 10})
        self.assertEqual(self.controller.parameters, old)
        for bad in (float("nan"), float("inf"), True, 0, "5"):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.controller.set_parameters({"speed": bad})
        self.controller.set_parameters({"speed": 3})
        self.assertEqual(self.controller.mode, "paused")
        self.assertEqual(self.controller.parameters["speed"], 3)

    def test_sensor_shape_rejects_non_numeric_and_boolean_data(self):
        for invalid in ([], [1] * 7, [1] * 9, [True] * 8, ["1"] * 8, None):
            with self.subTest(invalid=invalid):
                self.assertFalse(valid_zones(invalid))


class SimulationTests(unittest.TestCase):
    def test_motion_changes_contact_and_is_clamped_at_travel_limits(self):
        device = SimulationDevice()
        first = device.read(10.0)
        device.set_motion("advance", 5)
        for i in range(1, 51):
            frame = device.read(10 + i * 0.1)
        self.assertGreater(sum(frame.zones[4:7]), sum(first.zones[4:7]))
        device.position = 99.9
        device.read(16.0)
        self.assertEqual(device.position, 100)
        device.position = 0.1
        device.set_motion("retract", 5)
        device.read(16.1)
        self.assertEqual(device.position, 0)

    def test_forward_and_backward_body_motion_change_lumbar_contact(self):
        device = SimulationDevice()
        original = device.read(10)
        device.set_scenario("forward")
        for i in range(1, 41):
            forward = device.read(10 + i * 0.1)
        device.set_scenario("backward")
        for i in range(41, 81):
            backward = device.read(10 + i * 0.1)
        self.assertLess(sum(forward.zones[4:7]), sum(original.zones[4:7]))
        self.assertGreater(sum(backward.zones[4:7]), sum(original.zones[4:7]))

    def test_injected_faults_have_distinct_sensor_semantics(self):
        device = SimulationDevice()
        device.set_scenario("dropout")
        self.assertIsNone(device.read(10))
        device.set_scenario("invalid")
        self.assertFalse(device.read(10.1).valid)
        device.set_scenario("away")
        self.assertEqual(device.read(10.2).zones, [0] * 8)
        device.set_scenario("contact_loss")
        frame = device.read(10.3)
        self.assertGreater(sum(frame.zones[:4]), 0)
        self.assertEqual(frame.zones[4:7], [0] * 3)
        with self.assertRaises(ValueError):
            device.set_scenario("unknown")


if __name__ == "__main__":
    unittest.main()
