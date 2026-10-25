"""Deterministic closed-loop seat model: actuator movement changes contact."""
import math
import time
from .devices import SensorFrame

SCENARIOS = {"neutral", "forward", "backward", "left", "away", "contact_loss", "dropout", "invalid"}


class SimulationDevice:
    def __init__(self):
        self.position = 45.0
        self.scenario = "neutral"
        self.action = "hold"
        self.speed = 5.0
        self.seq = 0
        self.last_time = None
        self.body_offset = 0.0
        self.mapping_ready = True
        self.port = None
        self.message = "模拟设备 · 执行器动作会改变接触读数"

    def set_scenario(self, name):
        if name not in SCENARIOS:
            raise ValueError("未知模拟场景")
        self.scenario = name

    def set_motion(self, action, speed):
        self.action, self.speed = action, speed

    def close(self):
        self.set_motion("stop", self.speed)

    def read(self, now=None):
        now = time.monotonic() if now is None else now
        dt = min(0.1, max(0, now - self.last_time)) if self.last_time is not None else 0.05
        self.last_time = now
        self.position = max(0.0, min(100.0, self.position + {"advance": 1, "retract": -1}.get(self.action, 0) * self.speed * dt))
        target_offset = {"forward": 17, "backward": -16}.get(self.scenario, 0)
        self.body_offset += (target_offset - self.body_offset) * min(1, dt * 3)
        if self.scenario == "dropout":
            return None
        self.seq += 1
        delta = (self.position - 45 - self.body_offset) * 1.4
        zones = [45, 48, 62, 64, 62 + delta, 70 + delta, 65 + delta, 42 - self.body_offset]
        if self.scenario == "forward":
            zones[:4] = [61, 64, 46, 47]
        if self.scenario == "backward":
            zones[:4] = [36, 38, 74, 76]
        if self.scenario == "left":
            zones = [63, 31, 78, 43, 98 + delta, 56 + delta, 47 + delta, 42]
        if self.scenario == "away":
            zones = [0] * 8
        elif self.scenario == "contact_loss":
            zones[4:8] = [0] * 4
        zones = [round(max(0.0, min(255.0, x + (0.5 * math.sin(now * 2.1 + i) if x else 0))), 3) for i, x in enumerate(zones)]
        return SensorFrame(zones=zones, raw_zones=list(zones), matrix=None, raw_matrix=None,
                           seq=self.seq, device_time_ms=int(now * 1000) % 4294967296,
                           received_at=now, valid=self.scenario != "invalid")
