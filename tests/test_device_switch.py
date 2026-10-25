"""Device-switch safety: a dead link must not lock the user out.

A reachable board that does not acknowledge a stop keeps the original
fail-closed behaviour. A link that is already gone cannot acknowledge
anything, so waiting for it would be a deadlock.
"""
import unittest

from lumbar.actuators import ActuatorResult
from lumbar.service import AppService


class DeviceSwitchTests(unittest.TestCase):
    def _app(self):
        app = AppService()
        self.addCleanup(app.close)
        return app

    def test_dead_link_allows_device_switch(self):
        app = self._app()

        class DeadTransport:
            connected = False

        class DeadActuator:
            enabled = True
            is_hardware = True
            _transport = DeadTransport()

            def execute(self, command):
                return ActuatorResult(False, False, "串口未连接，无法发送命令")

            def close(self):
                pass

        app.output.actuator = DeadActuator()
        app.actuator_fault = "执行器未确认接收指令：串口未连接"
        app.action({"action": "connect", "source": "simulation"})
        self.assertEqual(app.source, "simulation")
        self.assertIsNone(app.actuator_fault)

    def test_live_link_without_ack_still_blocks_switch(self):
        app = self._app()

        class LiveTransport:
            connected = True

        class UnacknowledgedActuator:
            enabled = True
            is_hardware = True
            _transport = LiveTransport()

            def execute(self, command):
                return ActuatorResult(True, False, "已发送但未收到板端回执")

            def close(self):
                pass

        app.output.actuator = UnacknowledgedActuator()
        with self.assertRaises(ValueError) as ctx:
            app.action({"action": "connect", "source": "simulation"})
        self.assertIn("停止未确认", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
