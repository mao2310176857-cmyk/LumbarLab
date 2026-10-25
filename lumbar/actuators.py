"""Actuator extension boundary. No real hardware transport is implemented.

Commands describe intent, not motor bytes, valve pins, or physical distance.
`speed` is simulation travel-percent/second; a real adapter MUST replace that
with calibrated hardware units and independently enforce limits/watchdogs.

advance: one bounded movement toward the user's lumbar region.
retract: one bounded movement away from the user's lumbar region.
hold: request no new movement; does not claim an energized brake or valve state.
stop: cancel pending movement immediately; idempotent safety intent. A real
      implementation must state its measured/acknowledged safe output state.

Real deployment requires a confirmed command protocol, hardware limit switches,
emergency stop, absolute travel/force limits, feedback, an independent board-side
watchdog and a fail-safe loss-of-communication response. An adapter must reject
expired commands; repeated host packets must not extend a motion indefinitely.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Literal, Protocol


Action = Literal["advance", "retract", "hold", "stop"]


@dataclass(frozen=True)
class ActuatorCommand:
    action: Action
    issued_at: float
    speed: float = 0.0
    duration_s: float = 0.0

    def __post_init__(self):
        if self.action not in ("advance", "retract", "hold", "stop"):
            raise ValueError("未知执行器动作")
        for name in ("issued_at", "speed", "duration_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} 必须是有限数字")
        if self.issued_at < 0:
            raise ValueError("issued_at 必须来自非负单调时钟")
        if self.action in ("advance", "retract"):
            if not 0 < self.speed <= 10 or not 0 < self.duration_s <= 2:
                raise ValueError("移动建议需要 0..10 范围内的正速度及不超过 2 秒的正时限")
        elif self.speed != 0 or self.duration_s != 0:
            raise ValueError("hold/stop 不应携带速度或持续时间")

    @property
    def expires_at(self) -> float:
        return self.issued_at + self.duration_s


@dataclass(frozen=True)
class ActuatorResult:
    sent: bool
    acknowledged: bool
    message: str


class Actuator(Protocol):
    enabled: bool
    is_hardware: bool

    def execute(self, command: ActuatorCommand) -> ActuatorResult:
        """Validate, enforce limits and return an honest transport/ack result."""
        ...


class ActuationDisabledError(RuntimeError):
    pass


class DisabledActuator:
    """Default real-output implementation: it owns no port and sends nothing."""

    enabled = False
    is_hardware = True

    def execute(self, command: ActuatorCommand) -> ActuatorResult:
        if not isinstance(command, ActuatorCommand):
            raise ValueError("执行器接口需要 ActuatorCommand")
        if command.action in ("advance", "retract"):
            raise ActuationDisabledError("真实执行器未启用；没有发送任何移动指令")
        # A disabled transport cannot honestly claim a hardware stop ACK.
        return ActuatorResult(False, False, "真实执行器未启用；没有发送指令，也没有硬件停止确认")


class SimulationActuator:
    """Route simulated movement through the same command boundary as adapters.

    Its acknowledgement describes only the simulation model. It is never a
    claim about a physical motor, a hardware stop, or calibrated motor speed.
    """

    enabled = True
    is_hardware = False

    def __init__(self, device):
        self.device = device

    def execute(self, command: ActuatorCommand) -> ActuatorResult:
        if not isinstance(command, ActuatorCommand):
            raise ValueError("执行器接口需要 ActuatorCommand")
        speed = command.speed if command.action in ("advance", "retract") else 0
        self.device.set_motion(command.action, speed)
        return ActuatorResult(True, True, "模拟执行器已接收指令；不代表真实电机动作")


class ActuatorDispatcher:
    """Dispatch bounded intent once and retain honest transport status.

    A movement's issuance time and duration belong to the controller's motion
    episode. Repeated ticks must pass the same values; they do not renew the
    deadline. This host-side boundary is not a hardware watchdog: a real
    adapter and its board must independently enforce the remaining duration.
    """

    def __init__(self, actuator: Actuator):
        self.actuator = actuator
        self._seq = 0
        self._requested_action = None
        self._command = None
        self._last_key = None
        self._seen_moves = {}
        self._result = ActuatorResult(False, False, "尚未派发执行器指令")
        self._fault = None

    @staticmethod
    def _validate_now(now):
        if isinstance(now, bool) or not isinstance(now, (int, float)) or not math.isfinite(now) or now < 0:
            raise ValueError("now 必须来自非负有限单调时钟")

    @staticmethod
    def _key(command):
        if command.action in ("advance", "retract"):
            return (command.action, command.issued_at, command.speed, command.duration_s)
        return (command.action,)

    def _execute(self, command, *, force=False):
        key = self._key(command)
        if not force and key == self._last_key:
            return self._result
        self._seq += 1
        self._command = {"seq": self._seq, **asdict(command)}
        # A failed or ambiguous write must not qualify for de-duplication.
        self._last_key = None
        try:
            result = self.actuator.execute(command)
            if not isinstance(result, ActuatorResult):
                raise TypeError("执行器必须返回 ActuatorResult")
        except Exception as exc:
            self._fault = str(exc) or type(exc).__name__
            self._result = ActuatorResult(False, False, "执行器调用失败，输出状态未确认：" + self._fault)
            raise
        self._result = result
        self._last_key = key
        # A successful stop reports a stop result but does not erase a fault.
        return result

    def dispatch(self, command: ActuatorCommand, now: float) -> ActuatorResult:
        if not isinstance(command, ActuatorCommand):
            raise ValueError("执行器接口需要 ActuatorCommand")
        self._validate_now(now)
        self._requested_action = command.action
        self._seen_moves = {key: expiry for key, expiry in self._seen_moves.items() if expiry > now}
        if command.action in ("advance", "retract"):
            if now >= command.expires_at:
                result = self._execute(ActuatorCommand("stop", issued_at=now))
                # Keep the original requested action separate from the actual
                # stop command, so the UI cannot mistake it for movement.
                prefix = "移动指令已过期；"
                message = result.message if result.message.startswith(prefix) else prefix + result.message
                self._result = ActuatorResult(result.sent, result.acknowledged, message)
                return self._result
            key = self._key(command)
            if key in self._seen_moves:
                if key == self._last_key:
                    return self._result
                # A cancelled/superseded episode is not a new movement. In
                # particular, replaying it while the opposite direction is
                # active must stop that direction, not return its old ACK.
                result = self._execute(ActuatorCommand("stop", issued_at=now))
                prefix = "移动指令已取消或被替代；"
                message = result.message if result.message.startswith(prefix) else prefix + result.message
                self._result = ActuatorResult(result.sent, result.acknowledged, message)
                return self._result
            result = self._execute(command)
            self._seen_moves[key] = command.expires_at
            return result
        return self._execute(command)

    def force_stop(self, now: float) -> ActuatorResult:
        self._validate_now(now)
        self._requested_action = "stop"
        return self._execute(ActuatorCommand("stop", issued_at=now), force=True)

    def snapshot(self) -> dict:
        return {"requested_action": self._requested_action,
                "command": dict(self._command) if self._command else None,
                "sent": self._result.sent, "acknowledged": self._result.acknowledged,
                "message": self._result.message, "fault": self._fault,
                "is_hardware": bool(getattr(self.actuator, "is_hardware", False)),
                "enabled": bool(self.actuator.enabled)}
