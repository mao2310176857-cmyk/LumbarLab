"""State coordinator, recording, and device-independent commands."""
import copy
import json
import os
import re
import threading
import time
import uuid
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

from .actuators import ActuatorCommand, ActuatorDispatcher, DisabledActuator, SimulationActuator
from .controller import Controller
from .devices import SerialSensor, load_ignored_zones, load_tuning
from .simulation import SimulationDevice
from .transport import SharedSerialTransport

ROOT = Path(__file__).resolve().parent.parent


def iso_now():
    return datetime.now(timezone.utc).isoformat()


# A frame older than this is considered stale. 0.8 s is the safety contract
# (a real dropout must pause the controller quickly), so the board has to keep
# frames flowing well under this interval.
FRAME_STALE_S = 0.8


class AppService:
    def __init__(self, data_dir=None, mapping_path=None, actuator_factory=None):
        self.data_dir = Path(data_dir) if data_dir else ROOT / "data" / "sessions"
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.mapping_path = Path(mapping_path) if mapping_path else ROOT / "config" / "zones.json"
        self.lock = threading.RLock()
        self.controller = Controller()
        self.device = SimulationDevice()
        self.source = "simulation"
        # Real-serial mode shares one COM between the sensor view and the
        # actuator adapter; the transport owns the port.
        self.transport = None
        # A verified hardware adapter can be injected without changing control logic.
        self.actuator_factory = actuator_factory or self._default_actuator
        self.output = ActuatorDispatcher(self.actuator_factory(self.source, self.device))
        self.actuator_fault = None
        self._move_key = self._move_command = None
        self.started = time.monotonic()
        self.now = self.started
        self.frame = None
        self.history = deque(maxlen=1200)
        self.events = deque(maxlen=30)
        self.calibration = {"active": False, "progress": 0, "message": "尚未记住支撑"}
        self.calibration_samples = []
        self.calibration_start = 0.0
        self.record = None
        self.record_file = None
        self.record_start_time = 0.0
        self.last_history_time = -1.0
        self.last_decision = None
        self.last_record = None
        self.fatal_error = None
        self._event("system", "实验台已就绪 · 当前为模拟设备")
        self.tick()

    @staticmethod
    def _default_actuator(source, device):
        return SimulationActuator(device) if source == "simulation" else DisabledActuator()

    def _output_state(self):
        result = self.output.snapshot()
        result["fault"] = self.actuator_fault or result["fault"]
        return result

    def _dispatch_control(self, force_stop=False):
        """Send each bounded motion once; never renew its deadline on a tick.

        No-adapter mode still computes decisions, but only sends stop intent to
        DisabledActuator. A real adapter must implement its own board-side
        deadline, physical limits, calibrated units and acknowledgement.
        """
        enabled = self.output.actuator.enabled
        if self.actuator_fault:
            self.controller.pause("执行器故障 · 请检查驱动后重新连接")
        action = self.controller.action
        if force_stop or self.actuator_fault or not enabled:
            self._move_key = self._move_command = None
            command = ActuatorCommand("stop", self.now)
        elif action in ("advance", "retract"):
            speed = self.controller.parameters["speed"]
            nominal_duration = 0.8 if self.controller.mode == "auto" else 2.0 / speed
            deadline = self.controller.motion_until if self.controller.mode == "auto" else self.controller.manual_until
            key = (action, self.controller.mode, deadline, speed)
            if deadline <= self.now:
                self._move_key = self._move_command = None
                command = ActuatorCommand("stop", self.now)
            else:
                if key != self._move_key:
                    # A direction change receives a new command, but only the
                    # remaining time from the original bounded motion episode.
                    duration = min(nominal_duration, deadline - self.now)
                    self._move_command = ActuatorCommand(action, deadline - duration, speed, duration)
                    self._move_key = key
                command = self._move_command
        else:
            self._move_key = self._move_command = None
            command = ActuatorCommand(action, self.now)
        try:
            result = self.output.force_stop(self.now) if force_stop else self.output.dispatch(command, self.now)
            if enabled and (not result.sent or not result.acknowledged):
                raise RuntimeError("执行器未确认接收指令：" + result.message)
        except Exception as exc:
            previous_fault = self.actuator_fault
            self.actuator_fault = str(exc) or type(exc).__name__
            self.controller.pause("执行器故障 · 已请求停止，实际输出请检查驱动")
            self._cancel_active_calibration()
            if not force_stop:
                try:
                    stopped = self.output.force_stop(self.now)
                    if enabled and (not stopped.sent or not stopped.acknowledged):
                        self.actuator_fault += "；停止未确认：" + stopped.message
                except Exception as stop_exc:
                    self.actuator_fault += "；停止未确认：" + str(stop_exc)
            if previous_fault != self.actuator_fault:
                self._event("actuator_fault", self.actuator_fault)
            return False
        return True

    def _close_output(self):
        close = getattr(self.output.actuator, "close", None)
        if callable(close):
            close()

    def _output_reachable(self) -> bool:
        """Can the current actuator still talk to the board?

        False when the serial link is already gone. In that case waiting for
        a stop acknowledgement is pointless: the board cannot receive it.
        """
        actuator = getattr(self.output, "actuator", None)
        transport = getattr(actuator, "_transport", None)
        if transport is None:
            # No transport at all (simulation or a disabled actuator): the
            # stop could not have been sent, but there is no hardware to stop.
            return True
        return bool(getattr(transport, "connected", False))

    def _close_transport(self):
        """Release the shared serial port if this service owns one."""
        transport, self.transport = self.transport, None
        if transport is not None:
            try:
                transport.close()
            except Exception:
                pass

    def _event(self, kind, detail):
        event = {"t": round(self.now - self.started, 3), "type": kind, "detail": detail}
        self.events.append(event)
        if self.record is not None:
            self.record["events"].append(event)
            self._journal({"kind": "event", "value": event})

    def _journal(self, value):
        if self.record_file:
            self.record_file.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n")
            self.record_file.flush()

    def tick(self, now=None):
        with self.lock:
            self.now = time.monotonic() if now is None else now
            try:
                frame = self.device.read(self.now) if self.source == "simulation" else self.device.read()
            except Exception as exc:
                frame = None
                self.frame = None
                self.controller.pause("设备读取失败 · 请重新连接")
                self.device.message = str(exc)
            fresh = frame is not None
            if fresh:
                self.frame = frame
            zones = self.frame.zones if self.frame and self.frame.zones is not None else [0.0] * 8
            age = max(0, self.now - self.frame.received_at) if self.frame else 999.0
            valid = bool(self.frame and self.frame.valid and self.frame.zones is not None and getattr(self.device, "connected", True))
            position = self.device.position if self.source == "simulation" else None
            snapshot = self.controller.update(zones, self.now, position, valid=valid, age=age, fresh=fresh)
            if self.calibration["active"]:
                elapsed = self.now - self.calibration_start
                if not valid or age > FRAME_STALE_S or not snapshot["occupied"] or not snapshot["contact"]:
                    self._cancel_calibration("采集已取消 · 需要连续有效的座面与腰部接触")
                else:
                    if fresh:
                        self.calibration_samples.append(list(zones))
                    self.calibration["progress"] = min(1, elapsed / 3.0)
                    if elapsed >= 3:
                        try:
                            self.controller.set_reference(self.calibration_samples, position)
                            self.calibration = {"active": False, "progress": 1, "message": "已记住个人支撑"}
                            self._event("reference", copy.deepcopy(self.controller.reference))
                        except ValueError as exc:
                            self._cancel_calibration(str(exc))
            self._dispatch_control()
            decision = (self.controller.mode, self.controller.action, self.controller.reason)
            if decision != self.last_decision:
                self._event("control", self.controller.reason)
                self.last_decision = decision
            if self.now - self.last_history_time >= 0.095:
                self.last_history_time = self.now
                metrics = self.controller.metrics
                self.history.append({"t": round(self.now - self.started, 3), "z5": zones[4], "z6": zones[5], "z7": zones[6],
                                     "lumbar": metrics["lumbar"], "position": position, "action": self.controller.action,
                                     "lower": metrics["lower"], "upper": metrics["upper"]})
                while self.history and self.history[0]["t"] < self.now - self.started - 60:
                    self.history.popleft()
            # Save every new input frame, including full raw matrices when available.
            if self.record is not None and (fresh or age > FRAME_STALE_S and self.now - self.record.get("_last_sample", 0) >= 0.1):
                sample = self._state(include_history=False)
                self.record["frames"].append(sample)
                self.record["_last_sample"] = self.now
                self._journal({"kind": "frame", "value": sample})
                if self.now - self.record_start_time >= 1800 or len(self.record["frames"]) >= 36000:
                    self._event("record", "记录达到单次 30 分钟或 36000 帧上限，已自动保存")
                    self._stop_recording()

    def _cancel_calibration(self, message):
        self.calibration = {"active": False, "progress": 0, "message": message}
        self.calibration_samples = []
        self._event("calibration", message)

    def _state(self, include_history=True):
        frame = self.frame
        display_now = max(self.now, time.monotonic())
        age = max(0, display_now - frame.received_at) if frame else 999.0
        valid = bool(frame and frame.valid and frame.zones is not None and age <= FRAME_STALE_S and getattr(self.device, "connected", True) and not self.fatal_error)
        recording = {"active": self.record is not None, "id": self.record["id"] if self.record else None,
                     "samples": len(self.record["frames"]) if self.record else 0,
                     "elapsed_s": round(self.now - self.record_start_time, 2) if self.record else 0}
        output = self._output_state()
        result = {"time": round(display_now - self.started, 3), "source": self.source,
                  "connected": age <= FRAME_STALE_S and getattr(self.device, "connected", True),
                  "scenario": self.device.scenario if self.source == "simulation" else None,
                  "zones": list(frame.zones) if frame and frame.zones is not None else [0.0] * 8,
                  "raw_zones": frame.raw_zones if frame else None,
                  # 10-bit display channel (0..1023). The sensors are 10-bit;
                  # "zones" above is the 8-bit scale the algorithm thresholds
                  # are written against. The web page shows this one.
                  "zones10": list(frame.zones10) if frame and frame.zones10 is not None else None,
                  "adc10": frame.adc10 if frame else None,
                  "matrix": frame.matrix if frame else None, "raw_matrix": frame.raw_matrix if frame else None,
                  "frame_seq": frame.seq if frame else None, "device_time_ms": frame.device_time_ms if frame else None,
                  "frame_age_ms": round(age * 1000), "signal_valid": valid,
                  "position": round(self.device.position, 2) if self.source == "simulation" else None,
                  "reference": copy.deepcopy(self.controller.reference), "calibration": dict(self.calibration),
                  "parameters": dict(self.controller.parameters), "recording": recording,
                  "actuator": output,
                  "hardware": {"actuation_enabled": output["enabled"] and output["is_hardware"] and not output["fault"],
                               "mapping_ready": self.device.mapping_ready,
                               "port": self.device.port, "message": self.device.message}}
        result.update(self.controller.snapshot())
        result["occupied"] = valid and result["occupied"]
        result["contact"] = valid and result["contact"]
        # Frame snapshots avoid recursively repeating the event/history streams.
        result["events"] = list(self.events) if include_history else []
        if include_history:
            result["history"] = list(self.history)
        return result

    def state(self):
        with self.lock:
            return copy.deepcopy(self._state())

    def _ensure_signal(self, manual=False):
        """Gate a movement request on a usable signal.

        In manual mode the seat/waist presence checks are relaxed: the user
        is deliberately driving the motor during a bench test, and the
        ergonomic heuristics (is somebody sitting? is the waist in contact?)
        would otherwise block the test.
        """
        state = self._state()
        if not state["signal_valid"]:
            raise ValueError("数据无效、已中断或八区映射未完成，请先检查设备")
        if not manual and (not state["occupied"] or not state["contact"]):
            raise ValueError("请先坐好，并保持腰部接触")
        if max(state["zones"][4:7]) >= self.controller.max_waist_level:
            raise ValueError("腰部信号接近量程上限，请先检查传感器")

    def action(self, payload):
        if not isinstance(payload, dict):
            raise ValueError("请求必须是 JSON 对象")
        with self.lock:
            action = payload.get("action")
            if self.fatal_error and action not in ("stop", "record_stop"):
                raise ValueError(self.fatal_error)
            self.now = time.monotonic()
            if action in ("manual", "auto_start") and self.actuator_fault:
                raise ValueError("执行器故障尚未解除，请检查驱动后重新连接")
            if action == "manual":
                if not self.output.actuator.enabled:
                    raise ValueError("真实执行器尚未接入；串口模式仅采集与展示控制建议")
                self._ensure_signal(manual=True)
                self._cancel_active_calibration()
                self.controller.manual(payload.get("direction"), self.now)
                self._event("manual", payload.get("direction"))
            elif action == "stop":
                self.controller.stop()
                self._cancel_active_calibration()
                self._dispatch_control(force_stop=True)
                self._event("stop", "用户停止")
            elif action == "auto_pause":
                self.controller.pause()
                self._cancel_active_calibration()
                self._dispatch_control(force_stop=True)
                self._event("mode", "自动已暂停")
            elif action == "auto_start":
                if self.calibration["active"]:
                    raise ValueError("正在采集个人参考，请等待完成")
                self._ensure_signal()
                self.controller.start_auto()
                self._event("mode", "自动跟随已开启" if self.output.actuator.enabled else "仅计算控制建议 · 执行器尚未接入")
            elif action == "calibrate":
                self._ensure_signal()
                self.controller.pause("正在采集个人参考 · 请保持坐稳 3 秒")
                if not self._dispatch_control(force_stop=True):
                    raise ValueError("执行器停止未确认，无法采集参考")
                self.calibration_samples = []
                self.calibration_start = self.now
                self.calibration = {"active": True, "progress": 0, "message": "请保持坐稳 3 秒"}
                self._event("calibration", "开始采集个人支撑")
            elif action == "scenario":
                if self.source != "simulation":
                    raise ValueError("模拟场景仅适用于模拟设备")
                self.device.set_scenario(payload.get("name"))
                self._event("scenario", payload.get("name"))
            elif action == "parameters":
                if not isinstance(payload.get("values"), dict):
                    raise ValueError("控制参数必须是对象")
                self._cancel_active_calibration()
                self.controller.set_parameters(payload["values"])
                self._event("parameters", dict(self.controller.parameters))
            elif action == "record_start":
                self._start_recording()
            elif action == "record_stop":
                if self.record is None:
                    raise ValueError("当前没有正在进行的记录")
                self._stop_recording()
            elif action == "feedback":
                names = {"too_much": "有点顶", "comfortable": "支撑合适", "too_little": "有点空"}
                if payload.get("value") not in names:
                    raise ValueError("未知反馈")
                if self.record is None:
                    raise ValueError("请先开始记录，再标记体验反馈")
                self._event("feedback", {"value": payload["value"], "label": names[payload["value"]]})
            elif action == "connect":
                self._connect(payload)
            elif action == "zero":
                if self.source != "serial":
                    raise ValueError("模拟信号已完成空载归零")
                if payload.get("confirmed_empty") is not True:
                    raise ValueError("请先确认传感器上没有人体或物品接触")
                self.controller.pause("正在电脑端空载归零")
                self._cancel_active_calibration()
                if not self._dispatch_control(force_stop=True):
                    raise ValueError("执行器停止未确认，无法归零")
                self.device.zero()
                self.controller.reference = None
                self.controller.filtered = None
                self.frame = None
                self.history.clear()
                self.calibration = {"active": False, "progress": 0, "message": "归零后请重新记住支撑"}
                self._event("zero", "电脑端空载基线已更新 · 个人参考已清除")
            else:
                raise ValueError("未知操作")
            self._dispatch_control()
            return copy.deepcopy(self._state())

    def _cancel_active_calibration(self):
        if self.calibration["active"]:
            self._cancel_calibration("采集已取消")

    def _connect(self, payload):
        source = payload.get("source")
        if source not in ("simulation", "serial"):
            raise ValueError("请选择模拟或串口设备")
        if self.record is not None:
            raise ValueError("请先结束本次记录，再切换设备")
        self.controller.stop()
        self._cancel_active_calibration()
        if not self._dispatch_control(force_stop=True):
            # 旧执行器没能确认停止。两种情况要分开看：
            #   * 通信还活着但板端没回执 -> 拒绝切换（安全，保持原状）
            #   * 通信已经断了（拔线/板子复位）-> 板端收不到任何命令，
            #     这时"等它确认"是死锁，再也连不回来。允许切换并清掉故障锁，
            #     但记一条事件，让用户知道上一次输出状态未确认。
            if self._output_reachable():
                raise ValueError("旧执行器停止未确认，无法切换设备")
            self._event("connection", "旧设备通信已断开，未能确认停止状态；已放行设备切换")
            self.actuator_fault = None
        if source == "serial":
            port = payload.get("port")
            baud = payload.get("baud", 2000000)
            if not isinstance(port, str) or not port or len(port) > 128:
                raise ValueError("请选择有效串口")
            if not isinstance(baud, int) or isinstance(baud, bool) or baud not in (115200, 230400, 460800, 921600, 1000000, 2000000):
                raise ValueError("不支持的波特率")
            # 同一块板既有传感器又有电机：传感与执行器共用一个 COM。
            # SerialSensor 自己创建并拥有 SharedSerialTransport（这样它仍是
            # 串口的唯一所有者），适配器随后从 device.transport 取用同一条通道。
            new_device = SerialSensor(port, baud, mapping_path=self.mapping_path)
            new_transport = getattr(new_device, "transport", None)
            try:
                if self.source == "serial" and self.device.port == port:
                    self.device.close()
                    self.frame = None
                    self.controller.reference = None
                new_device.connect()
            except Exception as exc:
                new_device.close()
                if new_transport is not None:
                    new_transport.close()
                self._event("connection", "串口连接失败")
                raise ValueError(f"串口连接失败：{exc}") from exc
        else:
            new_device = SimulationDevice()
            new_transport = None
        try:
            new_output = ActuatorDispatcher(self.actuator_factory(source, new_device))
        except Exception:
            new_device.close()
            if new_transport is not None:
                new_transport.close()
            raise
        try:
            self._close_output()
            self.device.close()
        except Exception:
            new_device.close()
            if new_transport is not None:
                new_transport.close()
            close = getattr(new_output.actuator, "close", None)
            if callable(close):
                close()
            raise
        self.device, self.source = new_device, source
        self.transport = new_transport
        self.output = new_output
        self.actuator_fault = None
        self._move_key = self._move_command = None
        self.controller = Controller(self.controller.parameters,
                                    load_ignored_zones(
                                        self.mapping_path.parent / "ignored_zones.json"),
                                    load_tuning(
                                        self.mapping_path.parent / "tuning.json"))
        self.controller.pause("设备已切换 · 请重新记住支撑")
        self.frame = None
        self.history.clear()
        self.calibration = {"active": False, "progress": 0, "message": "设备已切换，请重新记住支撑"}
        self._event("connection", "切换至模拟设备" if source == "simulation" else f"串口 {port} · " + ("已接入执行器适配器" if self.output.actuator.enabled else "执行器未接入"))

    def _start_recording(self):
        if self.record is not None:
            raise ValueError("已经在记录中")
        sid = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8]
        self.record = {"schema_version": 1, "id": sid, "started_at": iso_now(), "ended_at": None,
                       "parameters": dict(self.controller.parameters), "reference": copy.deepcopy(self.controller.reference),
                       "source": self.source, "frames": [], "events": []}
        self.record_start_time = self.now
        try:
            self.record_file = (self.data_dir / (sid + ".jsonl")).open("x", encoding="utf-8")
            self._journal({"kind": "header", "value": {k: v for k, v in self.record.items() if k not in ("frames", "events")}})
        except OSError:
            self.record = None
            if self.record_file:
                self.record_file.close()
                self.record_file = None
            raise
        self._event("record", "开始记录")

    def _stop_recording(self):
        self._event("record", "结束记录")
        self.record["ended_at"] = iso_now()
        self.record.pop("_last_sample", None)
        sid = self.record["id"]
        tmp = self.data_dir / (sid + ".json.tmp")
        with tmp.open("w", encoding="utf-8") as stream:
            json.dump(self.record, stream, ensure_ascii=False, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, self.data_dir / (sid + ".json"))
        if self.record_file:
            self.record_file.close()
            self.record_file = None
        # Keep the journal for recovery/audit; it is not shown as a second session.
        self.last_record = sid
        self.record = None

    @staticmethod
    def _validate_id(sid):
        if not isinstance(sid, str) or not re.fullmatch(r"[0-9]{8}T[0-9]{6}Z-[0-9a-f]{8}", sid):
            raise ValueError("无效的记录编号")

    def session(self, sid):
        self._validate_id(sid)
        with self.lock:
            path = self.data_dir / (sid + ".json")
            if not path.is_file():
                raise FileNotFoundError("记录尚未保存或不存在")
            return json.loads(path.read_text(encoding="utf-8"))

    def sessions(self):
        result = []
        with self.lock:
            for path in sorted(self.data_dir.glob("*.json"), reverse=True)[:100]:
                try:
                    session = json.loads(path.read_text(encoding="utf-8"))
                    result.append({"id": session["id"], "started_at": session["started_at"], "ended_at": session["ended_at"],
                                   "samples": len(session["frames"]),
                                   "feedback_count": sum(e["type"] == "feedback" for e in session["events"])})
                except (OSError, ValueError, KeyError, TypeError):
                    continue
        return {"sessions": result}

    def fail(self, reason):
        with self.lock:
            self.fatal_error = reason
            self.controller.pause(reason)
            self.frame = None
            self._dispatch_control(force_stop=True)
            try:
                self._close_output()
            finally:
                self.device.close()
                self._close_transport()

    def close(self):
        with self.lock:
            self.controller.stop()
            self.now = time.monotonic()
            self._dispatch_control(force_stop=True)
            try:
                self._close_output()
            finally:
                self.device.close()
                self._close_transport()
            if self.record:
                self._stop_recording()
