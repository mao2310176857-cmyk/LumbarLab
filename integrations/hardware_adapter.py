"""真实电机执行器适配器（LumbarLab 硬件接入 · 板端协议实现）。

配套板端固件：LumbarLab_8ch_板端固件.ino
协议依据：docs/HARDWARE_INTEGRATION.md 第 8.2 节

板端实际情况
------------
    · 同一块 Arduino 上既有 8 路压力传感器（A0-A7），也有双继电器（D8/D9）
    · 因此**传感与执行器共用同一个 COM**，由 lumbar/transport.py 独占打开，
      单读线程按 type 路由，适配器只写命令、只从回执队列取 ACK
    · 继电器是**低电平触发**：板端负责把 action 映射到 D8/D9，适配器不碰引脚

协议
----
    发：{"type":"command","session":"<会话>","command_id":<int>,
         "action":"advance|retract|hold|stop",
         "speed_request":<float>,"remaining_ms":<int>}
    收：{"type":"ack","session":"...","command_id":<int>,
         "accepted":<bool>,"status":"..."}
        {"type":"fault","session":"...","code":"...","motion":"..."}

安全边界（都在板端生效，本文件只负责不越权）
------------------------------------------------
    · 板端有自己的硬时限（BOARD_MAX_MS），收到的 remaining_ms 会被限幅
    · stop/hold 在板端立即终止当前动作
    · 板端有租约看门狗：主机长时间不说话就释放继电器
    · 重复 command_id 不会被当作新命令重启电机
    · 电脑崩溃 / 拔线 → 板端超时自停；本适配器还会在 close() 前先发 stop
"""
from __future__ import annotations

import json
import time
import uuid

from lumbar.actuators import ActuatorCommand, ActuatorResult, SimulationActuator

# 板端硬时限上限，与固件 BOARD_MAX_MS 一致；这里再收一道保险
MAX_MOVE_MS = 2000
# 等回执的最长时间。execute() 运行在服务的 RLock 内，不能长时间阻塞。
ACK_TIMEOUT_S = 1.2
# stop 的回执等待可以短一些，但必须真的等到
STOP_ACK_TIMEOUT_S = 1.2


def create_actuator(source, sensor_device):
    """AppService 使用的工厂。

    模拟分支保持原样；串口分支返回真实适配器。
    """
    if source == "simulation":
        return SimulationActuator(sensor_device)
    if source == "serial":
        return HardwareActuator(sensor_device)
    raise ValueError("未知数据来源")


class HardwareActuator:
    """通过共享串口向板端派发有界动作，并等待匹配的 ACK。

    关于 enabled：只有在**真正完成握手**（拿到板端身份确认且初始输出为停止）
    之后才允许为 True。构造阶段拿不到板端身份就保持 False，
    这样网页不会因为"标志写成 True"就以为电机接通了。
    """

    enabled = False
    is_hardware = True

    def __init__(self, sensor_device):
        self.sensor_device = sensor_device
        # wire command_id 由本适配器自己生成，跨连接不保证唯一，
        # 但在同一 session 内单调递增，板端据此去重。
        self._wire_id = 0
        self._session = "lumbar-" + uuid.uuid4().hex[:8]
        self._transport = getattr(sensor_device, "transport", None)
        self._handshake_ok = False
        self._last_message = "尚未握手"

        if self._transport is None:
            self._last_message = "传感器设备没有共享传输层，无法驱动电机"
            return
        if not getattr(self._transport, "connected", False):
            self._last_message = "串口未连接，无法握手"
            return

        # 握手：先问一次身份，确认板端在运行且当前输出为停止。
        # ★ 板端刚复位时还在自动校准基线（约 1 秒），此期间命令收不到，
        #   所以这里重试几次，不能只试一次就判定失败。
        self._handshake_ok = False
        for attempt in range(6):
            try:
                self._transport.write_line(json.dumps({
                    "type": "command",
                    "session": self._session,
                    "command_id": self._next_id(),
                    "action": "stop",
                }, separators=(",", ":")))
            except Exception as exc:
                self._last_message = f"握手写入失败：{exc}"
                return

            ack = self._transport.wait_for_ack(self._wire_id, STOP_ACK_TIMEOUT_S)
            if ack is None:
                # 板端可能还在启动/校准，等一下再试
                time.sleep(0.5)
                continue
            if not ack.get("accepted", False):
                self._last_message = f"板端拒绝握手：{ack.get('status', '未知')}"
                return

            self._handshake_ok = True
            self.enabled = True
            self._last_message = "已握手，板端回执正常"
            return

        self._last_message = "握手未收到板端回执；电机保持禁用"

    def _next_id(self) -> int:
        self._wire_id += 1
        return self._wire_id

    # ------------------------------------------------------------ 主接口
    def execute(self, command) -> ActuatorResult:
        if not isinstance(command, ActuatorCommand):
            raise ValueError("execute 需要 ActuatorCommand")
        if not self.enabled:
            raise RuntimeError(self._last_message or "执行器未启用")

        if command.action in ("hold", "stop"):
            return self._stop_and_confirm()

        # 电脑时钟 -> 剩余毫秒。不能把 issued_at 当板端 millis() 用。
        remaining_s = command.expires_at - time.monotonic()
        if remaining_s <= 0:
            # 过期命令不得启动运动
            return self._stop_and_confirm()

        remaining_ms = int(remaining_s * 1000)
        if remaining_ms > MAX_MOVE_MS:
            remaining_ms = MAX_MOVE_MS
        if remaining_ms <= 0:
            return self._stop_and_confirm()

        return self._move_and_confirm(command.action, command.speed, remaining_ms)

    # ------------------------------------------------------------ 内部
    def _send_compact(self, verb, command_id, remaining_ms):
        """Send the 3-8 byte compact command and wait for its ACK.

        a123 / r123 / c123. The board replies with the same ACK shape as the
        JSON channel, so the caller can treat both identically.
        """
        try:
            self._transport.write_line(f"{verb}{command_id}")
        except Exception:
            return None
        return self._transport.wait_for_ack(command_id, ACK_TIMEOUT_S)

    def _result_from_ack(self, ack, command_id, action):
        """Turn a board ACK into an ActuatorResult."""
        accepted = bool(ack.get("accepted", False))
        status = ack.get("status", "")
        if accepted and status == "duplicate":
            # The board already ran this command id; the motor state is
            # whatever it left behind. Report it as accepted but note it.
            return ActuatorResult(True, True,
                                  f"板端已接受 {action}（重复指令，状态沿用上一次）")
        if not accepted:
            return ActuatorResult(True, False, f"板端拒绝 {action}：{status}")
        return ActuatorResult(True, True, f"板端已接受 {action}")

    def _move_and_confirm(self, action, speed_intent, remaining_ms) -> ActuatorResult:
        """发送一次有界移动并等待匹配 ACK。

        speed_request 只是主机意图值；板端按继电器方案执行固定时长的动作，
        板端自己的硬时限才是真正生效的时限。

        优先走**短命令通道**（a123 / r123 / c123，3-8 字节）：
        Arduino Uno 的 RX 缓冲只有 64 字节，而完整 JSON 命令约 65-90 字节。
        板端正在发一帧 JSON（1150 字节）时，主机来的字节会挤爆 RX 缓冲，
        板端就永远凑不出一整行 -> 命令完全收不到（实测成功率只有 3/12）。
        短命令只有几个字节，永远塞得下。
        """
        command_id = self._next_id()
        verb = {"advance": "a", "retract": "r"}.get(action)
        if verb is not None:
            ack = self._send_compact(verb, command_id, remaining_ms)
            if ack is not None:
                return self._result_from_ack(ack, command_id, action)
            # 短命令没收到回执：可能板端是旧固件（不认识短命令），
            # 退回完整 JSON 命令再试一次。
        payload = {
            "type": "command",
            "session": self._session,
            "command_id": command_id,
            "action": action,
            "speed_request": float(speed_intent),
            "remaining_ms": int(remaining_ms),
        }
        try:
            self._transport.write_line(json.dumps(payload, separators=(",", ":")))
        except Exception as exc:
            return ActuatorResult(False, False, f"命令发送失败：{exc}")

        ack = self._transport.wait_for_ack(command_id, ACK_TIMEOUT_S)
        if ack is None:
            # 没有匹配回执就不能声称已确认。板端仍会按自己的时限结束动作。
            return ActuatorResult(True, False, "已发送但未收到板端回执；板端会按自身时限停止")
        if not ack.get("accepted", False):
            return ActuatorResult(True, False, f"板端拒绝命令：{ack.get('status', '未知')}")

        return ActuatorResult(True, True,
                              f"板端已接受 {action}（{remaining_ms} ms）；不代表电机已到位")

    def _stop_and_confirm(self) -> ActuatorResult:
        command_id = self._next_id()
        payload = {
            "type": "command",
            "session": self._session,
            "command_id": command_id,
            "action": "stop",
        }
        try:
            self._transport.write_line(json.dumps(payload, separators=(",", ":")))
        except Exception as exc:
            return ActuatorResult(False, False, f"停止指令发送失败：{exc}")

        ack = self._transport.wait_for_ack(command_id, STOP_ACK_TIMEOUT_S)
        if ack is None:
            return ActuatorResult(True, False, "停止指令已发送但未收到回执；板端有独立超时保护")
        if not ack.get("accepted", False):
            return ActuatorResult(True, False, f"板端拒绝停止：{ack.get('status', '未知')}")

        return ActuatorResult(True, True, "板端已确认停止请求")

    def close(self) -> None:
        """退出/重连时先确保停止，再释放连接。

        AppService 会先请求 stop 再调用 close()，这里再兜一次底。
        """
        if self._transport is not None and getattr(self._transport, "connected", False):
            try:
                self._stop_and_confirm()
            except Exception:
                pass
        self.enabled = False
        self._handshake_ok = False
