"""Read-only adapter for the supplied 16×16 ViTac JSON sensor firmware.

No actuator protocol exists in the supplied source. This module never writes to
the serial port. pyserial is deliberately imported only when hardware is used.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import importlib
import json
import math
from pathlib import Path
import threading
import time
from typing import Mapping, Sequence

from .transport import SharedSerialTransport


POINTS = 256
UINT32_MAX = (1 << 32) - 1
MAX_LINE_BYTES = 8192
DEFAULT_MAPPING = Path(__file__).resolve().parent.parent / "config" / "zones.json"

# Zones whose sensor is currently not connected. Zone 6 (waist centre, A5)
# reads a constant 0 on the board, so it would drag the lumbar average down
# and make "waist contact" never register.
#
# The list lives in config/ignored_zones.json. If that file is absent or
# empty, no zone is ignored and the behaviour is exactly the original.
# Keep in sync with SENSOR_IGNORE in the board firmware.
DEFAULT_IGNORED = Path(__file__).resolve().parent.parent / "config" / "ignored_zones.json"


DEFAULT_TUNING = Path(__file__).resolve().parent.parent / "config" / "tuning.json"


def load_tuning(path: str | Path = DEFAULT_TUNING) -> dict:
    """Optional threshold overrides for the real hardware.

    Lives next to zones.json so it follows the same per-device config path.
    Absent file -> the original thresholds, exactly as shipped.

    Keys (all optional):
      max_waist_level : int, the 8-bit "waist near full scale" cutoff
                         (default 245). Raise it to stop the false
                         "接近量程上限" alarm.
      min_waist_contact: int, the 8-bit mean of zones 5-7 required before
                         the algorithm treats the waist as in contact
                         (default 3). Lower it to make manual testing easier.
      min_seat_occupied: int, the 8-bit mean of zones 1-4 required before
                         the algorithm treats the seat as occupied
                         (default 5).
    """
    try:
        with Path(path).open(encoding="utf-8-sig") as source:
            data = json.load(source)
    except (OSError, json.JSONDecodeError):
        return {}
    if not isinstance(data, dict):
        return {}
    out = {}
    for key in ("max_waist_level", "min_waist_contact", "min_seat_occupied"):
        value = data.get(key)
        if type(value) is int and 0 <= value <= 255:
            out[key] = value
    return out


def load_ignored_zones(path: str | Path = DEFAULT_IGNORED) -> tuple[int, ...]:
    """Zone numbers whose sensor is not connected (1..8). Empty when unknown.

    The file lives next to zones.json so it follows the same per-device
    configuration path. When it is absent, no zone is ignored and the
    behaviour is exactly the original.
    """
    try:
        with Path(path).open(encoding="utf-8-sig") as source:
            data = json.load(source)
    except (OSError, json.JSONDecodeError):
        return ()
    if isinstance(data, dict):
        data = data.get("ignore", [])
    if not isinstance(data, list):
        return ()
    return tuple(sorted({int(z) for z in data
                         if type(z) is int and 1 <= z <= 8}))


@dataclass
class SensorFrame:
    zones: list[float] | None
    raw_zones: list[float] | None
    matrix: list[int] | None
    raw_matrix: list[int] | None
    seq: int
    device_time_ms: int
    received_at: float
    valid: bool = True
    # 10-bit display channel (0..1023). The sensors are 10-bit; diff/raw are
    # the 8-bit channels the algorithm expects. None when the board does not
    # send "adc10" (e.g. the original 16x16 firmware).
    adc10: list[int] | None = None
    zones10: list[float] | None = None


def _integer(value: object, low: int, high: int, name: str) -> int:
    # bool is an int subclass, but is not a measurement or sequence number.
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} 必须是 {low}..{high} 的整数")
    return value


def _unique_object(pairs: list[tuple[str, object]]) -> dict:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"JSON 存在重复键：{key}")
        result[key] = value
    return result


def parse_frame(line: str | bytes) -> dict | None:
    """Parse a complete line; comments/blank lines return None, errors raise.

    Required fields are seq, t, base, diff; raw and the firmware's descriptive
    statistics are optional. A base=0 payload is structurally valid, but its
    SensorFrame.valid will be false. No sensor values are inferred from stats.
    """
    if isinstance(line, bytes):
        if len(line) > MAX_LINE_BYTES:
            raise ValueError("串口行过长")
        try:
            line = line.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ValueError("串口数据不是 UTF-8；请检查波特率") from exc
    if not isinstance(line, str):
        raise ValueError("串口帧必须是一行文本")
    if len(line.encode("utf-8")) > MAX_LINE_BYTES:
        raise ValueError("串口行过长")
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    try:
        data = json.loads(line, object_pairs_hook=_unique_object)
    except (json.JSONDecodeError, RecursionError) as exc:
        raise ValueError("串口行不是有效 JSON 帧") from exc
    if not isinstance(data, dict):
        raise ValueError("JSON 帧必须是对象")
    for key in ("seq", "t", "base", "diff"):
        if key not in data:
            raise ValueError(f"JSON 帧缺少 {key}")
    _integer(data["seq"], 0, UINT32_MAX, "seq")
    _integer(data["t"], 0, UINT32_MAX, "t")
    _integer(data["base"], 0, 1, "base")
    for key in ("diff", "raw"):
        if key not in data:
            continue
        values = data[key]
        if not isinstance(values, list) or len(values) != POINTS:
            raise ValueError(f"{key} 必须包含 256 个数")
        for value in values:
            _integer(value, 0, 255, key)
    for key, low, high in (("peak", 0, 255), ("pr", -1, 15),
                           ("pc", -1, 15), ("nz", 0, 256),
                           ("sum", 0, 65280)):
        if key in data:
            _integer(data[key], low, high, key)
    # Optional 10-bit display channel: eight raw sensor readings, 0..1023.
    # Unknown keys are ignored by design, so the original 16x16 firmware
    # (which does not send it) stays valid.
    if "adc10" in data:
        values = data["adc10"]
        if not isinstance(values, list) or len(values) != 8:
            raise ValueError("adc10 必须包含 8 个数")
        for value in values:
            _integer(value, 0, 1023, "adc10")
    return data


def _validate_mapping(mapping: object) -> dict[str, list[int]]:
    if not isinstance(mapping, dict) or set(mapping) != {str(n) for n in range(1, 9)}:
        raise ValueError("分区映射必须恰好包含字符串键 1..8")
    result: dict[str, list[int]] = {}
    used: set[int] = set()
    for zone in map(str, range(1, 9)):
        indices = mapping[zone]
        if not isinstance(indices, list):
            raise ValueError(f"分区 {zone} 必须是索引列表")
        result[zone] = []
        for index in indices:
            _integer(index, 0, 255, f"分区 {zone} 索引")
            if index in used:
                raise ValueError(f"矩阵索引 {index} 被重复分配")
            used.add(index)
            result[zone].append(index)
    return result


def load_mapping(path: str | Path = DEFAULT_MAPPING) -> dict[str, list[int]]:
    """Load disjoint zones. Empty/partial mappings are valid but not ready.

    All eight zones must be nonempty to enable aggregation. Unassigned cells are
    permitted: a measured zone need not use every physical pixel of the cloth.
    """
    try:
        with Path(path).open(encoding="utf-8-sig") as source:
            return _validate_mapping(json.load(source, object_pairs_hook=_unique_object))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"无法读取分区映射：{exc}") from exc


def map_zones(values: Sequence[int | float], mapping: Mapping[str, list[int]],
              ignored: Sequence[int] = ()) -> list[float] | None:
    """Mean each physical zone; an incomplete mapping returns None, never zero.

    Zones listed in `ignored` (a sensor that is not connected) are reported
    as 0 and must not be averaged in with live zones.
    """
    checked = _validate_mapping(mapping)
    if len(values) != POINTS or any(
        type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= 255
        for value in values
    ):
        raise ValueError("分区聚合需要 256 个 0..255 的有限数值")
    if any(not indices for indices in checked.values()):
        return None
    result = [sum(values[index] for index in checked[str(zone)]) / len(checked[str(zone)])
              for zone in range(1, 9)]
    for zone in ignored:
        if 1 <= zone <= 8:
            result[zone - 1] = 0.0
    return result


def map_zones10(values: Sequence[int | float],
                mapping: Mapping[str, list[int]],
                ignored: Sequence[int] = ()) -> list[float] | None:
    """Aggregate the 10-bit display channel into the same eight zones.

    The board sends the eight sensors' 10-bit *pressure* readings (0..1023) in
    sensor order: 0 means no load, a larger value means more pressure. The
    board already normalises the reversed sensor (#6) so every channel shares
    this direction. Each zone owns exactly one sensor, so zone k maps to
    sensor k; the mapping file is consulted so a zone that owns several cells
    still averages them consistently with map_zones().
    """
    checked = _validate_mapping(mapping)
    if len(values) != 8 or any(
        type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= 1023
        for value in values
    ):
        raise ValueError("10 位分区聚合需要 8 个 0..1023 的有限数值")
    if any(not indices for indices in checked.values()):
        return None
    # Each sensor k (0-based) sits at cell k*16 + k + 1 in the 256 matrix.
    sensor_cells = {k * 16 + k + 1: k for k in range(8)}
    result = []
    for zone in range(1, 9):
        picked = [sensor_cells[i] for i in checked[str(zone)] if i in sensor_cells]
        if not picked:
            return None
        result.append(sum(values[k] for k in picked) / len(picked))
    for zone in ignored:
        if 1 <= zone <= 8:
            result[zone - 1] = 0.0
    return result


def list_serial_ports() -> list[dict]:
    try:
        ports = importlib.import_module("serial.tools.list_ports")
    except ImportError:
        return []
    return [{"device": port.device, "description": port.description or port.device}
            for port in ports.comports()]


class SerialSensor:
    """One bounded reader thread, latest-frame handoff, and host-only zeroing.

    Real-serial mode uses a SharedSerialTransport that this view owns. The
    transport is the single owner of the COM: it has one reader thread and
    routes messages by the JSON "type" field, so an actuator adapter can share
    the same port without two objects racing to read() it.

    The actuator adapter reaches the write channel through `self.transport`.
    """

    def __init__(self, port: str, baud: int = 2000000,
                 mapping_path: str | Path = DEFAULT_MAPPING, transport=None):
        if not isinstance(port, str) or not port.strip():
            raise ValueError("请选择串口")
        if type(baud) is not int or not 1 <= baud <= 4000000:
            raise ValueError("无效波特率")
        self.port = port
        self.baud = baud
        self.mapping = load_mapping(mapping_path)
        self.mapping_ready = all(self.mapping.values())
        # Ignored-zone list lives beside the mapping file, so a per-device
        # configuration carries both. Absent file -> nothing ignored.
        self.ignored_zones = load_ignored_zones(
            Path(mapping_path).parent / "ignored_zones.json")
        self.tuning = load_tuning(Path(mapping_path).parent / "tuning.json")
        self.connected = False
        self.message = "串口尚未连接"
        self.sequence_status = "waiting"
        self._serial = None
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._pending: SensorFrame | None = None
        self._last_seq: int | None = None
        self._last_device_time: int | None = None
        self._last_raw_at: float | None = None
        self._recent_raw: deque[tuple[float, list[int]]] = deque(maxlen=120)
        self._host_baseline: list[int] | None = None
        self._reset_required = False
        # Shared-transport mode: this view is a consumer of routed frames.
        self.transport = transport
        self._owns_port = transport is None
        if self.transport is None:
            # Same board also drives the motor, so use the shared transport
            # from the start: one COM, one reader thread, routed messages.
            self.transport = SharedSerialTransport(port, baud)
            self._owns_port = True

    def connect(self) -> None:
        if self.connected:
            return
        if self.transport is not None:
            # The transport already owns the port and its reader thread.
            if not self.transport.connected:
                self.transport.connect()
            with self._lock:
                self._pending = None
                self._last_seq = None
                self._last_device_time = None
                self._last_raw_at = None
                self._recent_raw.clear()
                self._host_baseline = None
                self._reset_required = False
                self.sequence_status = "waiting"
                self.connected = True
                self.message = "已连接（共享串口），等待 JSON 帧；连接串口可能令 Nano 重启，请保持布面空载"
            return
        self.close()
        try:
            serial = importlib.import_module("serial")
        except ImportError as exc:
            raise ValueError("真实串口需安装 pyserial：python -m pip install pyserial") from exc
        try:
            serial_port = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.1)
        except Exception as exc:
            raise ValueError(f"无法打开串口 {self.port}：{exc}") from exc
        with self._lock:
            self._serial = serial_port
            self._pending = None
            self._last_seq = None
            self._last_device_time = None
            self._last_raw_at = None
            self._recent_raw.clear()
            self._host_baseline = None
            self._reset_required = False
            self.sequence_status = "waiting"
            self.connected = True
            self.message = "已连接，等待 JSON 帧；连接串口可能令 Nano 重启，请保持布面空载"
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, name="vitac-read-only", daemon=True)
        self._thread.start()

    def close(self) -> None:
        if self.transport is not None:
            # This view owns the transport, so it releases the port with it.
            with self._lock:
                self.connected = False
                self._pending = None
                self._recent_raw.clear()
                self._last_raw_at = None
                self._host_baseline = None
                self.message = "串口已关闭"
            if self._owns_port:
                try:
                    self.transport.close()
                except Exception:
                    pass
            return
        self._stop.set()
        serial_port, self._serial = self._serial, None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        self._thread = None
        with self._lock:
            self.connected = False
            self._pending = None
            self._recent_raw.clear()
            self._last_raw_at = None
            self._host_baseline = None
            self.message = "串口已关闭"

    def read(self) -> SensorFrame | None:
        if self.transport is not None:
            # Shared-transport mode: pull the newest routed frame and run the
            # same validation and freshness logic as the reader thread.
            payload = self.transport.latest_sensor_frame()
            if payload is not None:
                self._process_payload(payload)
            with self._lock:
                frame, self._pending = self._pending, None
                return frame
        with self._lock:
            frame, self._pending = self._pending, None
            return frame

    def zero(self) -> None:
        """Average a recent stable empty-cloth raw window; no board command.

        The caller must obtain explicit empty-cloth confirmation. Require >=5
        frames spanning >=0.5 s in the latest second, fresh within 0.5 s, and
        each pixel's observed range <=20 ADC/4 units. Reconnect discards zero.
        """
        now = time.monotonic()
        with self._lock:
            if self._reset_required:
                raise ValueError("检测到板端重启，请重新连接后再归零")
            if self._last_raw_at is None or now - self._last_raw_at > 0.5:
                raise ValueError("归零需要新鲜 raw 数据；请确认 JSON 保留 raw 数组")
            frames = [(at, raw) for at, raw in self._recent_raw if now - at <= 1.0]
            if len(frames) < 5 or frames[-1][0] - frames[0][0] < 0.5:
                raise ValueError("请保持布面空载稳定至少 1 秒后再归零")
            columns = list(zip(*(raw for _, raw in frames)))
            if any(max(column) - min(column) > 20 for column in columns):
                raise ValueError("空载 raw 波动过大；请保持布面静止后重试")
            self._host_baseline = [round(sum(column) / len(column)) for column in columns]
            self._pending = None  # The next fresh frame must use the new baseline.
            self.message = "电脑端空载基线已更新；未向板端发送命令"

    def _reject(self, message: str, now: float) -> None:
        """Invalidate immediately without copying an old measurement."""
        with self._lock:
            self.message = message
            self._last_raw_at = None
            self._recent_raw.clear()
            self._pending = SensorFrame(None, None, None, None, self._last_seq or 0,
                                        self._last_device_time or 0, now, valid=False)

    def _process_line(self, line: str | bytes, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        try:
            payload = parse_frame(line)
        except ValueError as exc:
            self._reject(str(exc), now)
            return
        if payload is None:
            return
        self._process_payload(payload, now)

    def _process_payload(self, payload: dict, now: float | None = None) -> None:
        """Apply an already-decoded sensor frame.

        Split out from _process_line so the shared-transport path can reuse the
        exact same validation, freshness and baseline rules.
        """
        now = time.monotonic() if now is None else now
        seq, device_time = payload["seq"], payload["t"]
        with self._lock:
            if self._reset_required:
                return
            if self._last_seq is not None:
                seq_delta = (seq - self._last_seq) & UINT32_MAX
                time_delta = (device_time - self._last_device_time) & UINT32_MAX
                if seq_delta == 0:
                    self.sequence_status = "duplicate"
                    self.message = "重复帧已忽略；未刷新数据时间"
                    return
                if seq_delta >= (1 << 31) or time_delta >= (1 << 31):
                    # Without a boot id, an old early packet cannot be proven
                    # different from a reboot. Fail closed on that ambiguity.
                    if seq <= 10 and device_time <= 5000 and device_time < self._last_device_time:
                        self.sequence_status = "reset"
                        self._reset_required = True
                        self._host_baseline = None
                        self._last_raw_at = None
                        self._recent_raw.clear()
                        self._pending = SensorFrame(None, None, None, None, seq,
                                                    device_time, now, valid=False)
                        self.message = "可能检测到板端重启，已清除电脑端基线；请重新连接"
                    else:
                        self.sequence_status = "reordered"
                        self.message = "倒序帧已忽略；未刷新数据时间"
                    return
                self.sequence_status = "gap" if seq_delta > 1 else "ok"
            else:
                self.sequence_status = "first"
            self._last_seq = seq
            self._last_device_time = device_time
            raw = payload.get("raw")
            matrix = payload["diff"]
            valid = payload["base"] == 1
            if raw is not None and valid:
                self._last_raw_at = now
                self._recent_raw.append((now, raw.copy()))
            else:
                self._last_raw_at = None
                self._recent_raw.clear()
            if self._host_baseline is not None:
                if raw is None:
                    matrix = None
                    valid = False
                else:
                    matrix = [max(0, value - base) for value, base in zip(raw, self._host_baseline)]
            zones = (map_zones(matrix, self.mapping, self.ignored_zones)
                     if matrix is not None else None)
            raw_zones = (map_zones(raw, self.mapping, self.ignored_zones)
                         if raw is not None else None)
            # Optional 10-bit display channel; absent on the original firmware.
            adc10 = payload.get("adc10")
            zones10 = (map_zones10(adc10, self.mapping, self.ignored_zones)
                       if adc10 is not None else None)
            self._pending = SensorFrame(zones, raw_zones, matrix, raw, seq,
                                        device_time, now, valid=valid,
                                        adc10=adc10, zones10=zones10)
            if payload["base"] == 0:
                self.message = "板端 base=0，未完成空载校准；数据不可用于控制"
            elif matrix is None:
                self.message = "电脑端归零后缺少 raw，数据不可用于控制"
            elif not self.mapping_ready:
                self.message = "矩阵已接收；8 个分区尚未全部配置，禁止自动控制"
            else:
                self.message = "只读接收正常；真实执行器未启用"

    def _reader(self) -> None:
        buffer = bytearray()
        discarding = False
        try:
            while not self._stop.is_set():
                serial_port = self._serial
                if serial_port is None:
                    return
                data = serial_port.read(min(max(serial_port.in_waiting, 1), 4096))
                for byte in data:
                    if discarding:
                        if byte == 10:
                            discarding = False
                        continue
                    if byte == 10:
                        self._process_line(bytes(buffer))
                        buffer.clear()
                    elif len(buffer) >= MAX_LINE_BYTES:
                        buffer.clear()
                        discarding = True
                        self._reject("串口行过长，已丢弃并等待下一完整行", time.monotonic())
                    else:
                        buffer.append(byte)
        except Exception as exc:
            if not self._stop.is_set():
                self._reject(f"串口读取中断：{exc}", time.monotonic())
                with self._lock:
                    self.connected = False
        finally:
            serial_port = self._serial
            if serial_port is not None:
                try:
                    serial_port.close()
                except Exception:
                    pass
