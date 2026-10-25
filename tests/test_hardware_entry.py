"""Explicit driver selection and fail-closed template tests; no hardware I/O."""
from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

import run
from integrations.hardware_adapter_template import HardwareActuator, create_actuator
from lumbar.actuators import ActuatorCommand, ActuatorResult, SimulationActuator
from lumbar.service import AppService
from lumbar.simulation import SimulationDevice


class EntryTests(unittest.TestCase):
    def run_main(self, arguments):
        with ExitStack() as stack:
            app_class = stack.enter_context(patch("run.AppService"))
            server_class = stack.enter_context(patch("run.Server"))
            stack.enter_context(patch("run.threading.Thread"))
            browser = stack.enter_context(patch("run.webbrowser.open"))
            stack.enter_context(redirect_stdout(io.StringIO()))
            result = run.main(arguments)
            return result, app_class, server_class, browser

    def test_default_entry_does_not_import_or_enable_a_driver(self):
        with patch("run.load_actuator_factory") as loader:
            result, app, server, browser = self.run_main(["--no-browser", "--port", "8766"])
        self.assertEqual(result, 0)
        loader.assert_not_called()
        app.assert_called_once_with()
        server.assert_called_once_with(("127.0.0.1", 8766), app.return_value)
        app.return_value.close.assert_called_once()
        browser.assert_not_called()

    def test_explicit_factory_is_imported_and_passed_to_service(self):
        module = types.ModuleType("fake_lumbar_driver")
        module.create_actuator = Mock()
        with patch.dict("sys.modules", {module.__name__: module}):
            result, app, _, _ = self.run_main([
                "--no-browser", "--actuator-factory", "fake_lumbar_driver:create_actuator"])
        self.assertEqual(result, 0)
        app.assert_called_once_with(actuator_factory=module.create_actuator)

    def test_invalid_factory_stops_before_app_or_server_startup(self):
        module = types.ModuleType("fake_lumbar_driver")
        module.not_callable = 1
        values = ("bad-format", "../driver:create", "fake_lumbar_driver:missing",
                  "fake_lumbar_driver:not_callable")
        with patch.dict("sys.modules", {module.__name__: module}):
            for value in values:
                with self.subTest(value=value), patch("run.AppService") as app, \
                        patch("run.Server") as server, redirect_stderr(io.StringIO()) as errors:
                    with self.assertRaises(SystemExit) as raised:
                        run.main(["--actuator-factory", value])
                    self.assertEqual(raised.exception.code, 2)
                    self.assertIn("actuator factory", errors.getvalue())
                    app.assert_not_called()
                    server.assert_not_called()

    def test_factory_initialization_failure_does_not_start_server(self):
        with patch("run.AppService", side_effect=RuntimeError("driver initialization failed")), \
                patch("run.Server") as server, redirect_stderr(io.StringIO()) as errors:
            self.assertEqual(run.main(["--no-browser"]), 1)
        self.assertIn("driver initialization failed", errors.getvalue())
        server.assert_not_called()


class TemplateTests(unittest.TestCase):
    def test_template_supports_simulation_but_rejects_real_output(self):
        device = SimulationDevice()
        self.assertIsInstance(create_actuator("simulation", device), SimulationActuator)
        sensor = Mock()
        with self.assertRaisesRegex(NotImplementedError, "尚未实现"):
            create_actuator("serial", sensor)
        sensor.assert_not_called()
        sensor.write.assert_not_called()
        with self.assertRaises(ValueError):
            create_actuator("unknown", sensor)

    def test_unimplemented_template_never_returns_a_fake_ack(self):
        adapter = HardwareActuator(Mock())
        self.assertFalse(adapter.enabled)
        with patch("integrations.hardware_adapter_template.time.monotonic", return_value=10):
            for command in (ActuatorCommand("advance", 10, 5, .8), ActuatorCommand("stop", 10)):
                with self.subTest(action=command.action), self.assertRaises(NotImplementedError):
                    adapter.execute(command)

    def test_template_passes_remaining_duration_and_generates_wire_ids(self):
        adapter = HardwareActuator(Mock())
        adapter._move_and_confirm = Mock(return_value=ActuatorResult(True, True, "fake test response"))
        adapter._stop_and_confirm = Mock(return_value=ActuatorResult(True, True, "fake test stop"))
        command = ActuatorCommand("advance", 10, 5, .8)
        with patch("integrations.hardware_adapter_template.time.monotonic", return_value=10.3):
            adapter.execute(command)
        args = adapter._move_and_confirm.call_args.args
        self.assertEqual(args[:3], (1, "advance", 5))
        self.assertAlmostEqual(args[3], .5)
        with patch("integrations.hardware_adapter_template.time.monotonic", return_value=10.8):
            adapter.execute(command)
        adapter._move_and_confirm.assert_called_once()
        adapter._stop_and_confirm.assert_called_once_with(2)

    def test_connecting_serial_with_unimplemented_template_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            app = AppService(data_dir=Path(directory), actuator_factory=create_actuator)
            try:
                original = app.device
                with patch("lumbar.service.SerialSensor") as sensor_class:
                    sensor = sensor_class.return_value
                    with self.assertRaisesRegex(NotImplementedError, "尚未实现"):
                        app.action({"action": "connect", "source": "serial", "port": "FAKE-COM"})
                    sensor.close.assert_called_once()
                    sensor.write.assert_not_called()
                self.assertIs(app.device, original)
                self.assertEqual(app.source, "simulation")
                self.assertEqual(app.controller.action, "stop")
                self.assertFalse(app.state()["hardware"]["actuation_enabled"])
            finally:
                app.close()


if __name__ == "__main__":
    unittest.main()
