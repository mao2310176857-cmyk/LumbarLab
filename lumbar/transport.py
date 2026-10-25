"""共享串口传输层：独占打开一个 COM，单读线程，按 type 路由消息。

为什么需要这个文件
------------------
LumbarLab 原设计里 `SerialSensor` 是**只读**的，永远不写串口；执行器适配器
则需要**写**命令并等回执。如果两者都 `serial.Serial(port=...)` 打开同一个
COM，Windows/macOS 上第二个句柄会直接失败；即使能打开，两个线程竞争
`read()` 会各抢走一半数据——传感帧和 ACK 互相错乱。

本模块提供唯一的串口所有者：

    transport = SharedSerialTransport(port, baud)
    transport.connect()
    transport.write_line('{"type":"command",...}')      # 发命令
    transport.latest_sensor_frame()                     # 取最新传感帧
    transport.wait_for_ack(command_id, timeout)         # 等匹配回执
    transport.close()                                   # 释放

消息路由按 JSON 的 `type` 字段：
    "sensor" -> 传感帧队列（供 SerialSensor 消费）
    "ack"    -> 回执队列（供执行器适配器消费）
    "fault"  -> 故障队列
其它没有 type 的行按传感帧处理（兼容原始固件）。

严格性说明：传感帧仍走原 `parse_frame()` 校验，本层只做**路由**，
不放松任何数值检查。
"""
from __future__ import annotations

from collections import deque
import importlib
import json
import threading
import time

MAX_LINE_BYTES = 8192
DEFAULT_BAUD = 115200


class SharedSerialTransport:
    """一个 COM、一个读线程、多个按 type 分流的有界队列。"""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD):
        if not isinstance(port, str) or not port.strip():
            raise ValueError("请选择串口")
        if type(baud) is not int or not 1 <= baud <= 4000000:
            raise ValueError("无效波特率")
        self.port = port
        self.baud = baud
        self.connected = False
        self.message = "串口尚未连接"
        self._serial = None
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._write_lock = threading.Lock()

        self._sensor_frames: deque[dict] = deque(maxlen=16)
        self._acks: deque[dict] = deque(maxlen=64)
        self._faults: deque[dict] = deque(maxlen=32)
        self._rx_bytes = 0
        self._rx_lines = 0
        self._rx_bad_lines = 0
        self._last_rx_at = 0.0

    # ------------------------------------------------------------ 生命周期
    def connect(self) -> None:
        if self.connected:
            return
        self.close()
        try:
            serial = importlib.import_module("serial")
        except ImportError as exc:
            raise ValueError("真实串口需安装 pyserial：python -m pip install pyserial") from exc
        try:
            # write_timeout 必须设置：否则电脑卡住时 execute() 会永久阻塞
            serial_port = serial.Serial(
                port=self.port, baudrate=self.baud, timeout=0.1, write_timeout=0.5
            )
        except Exception as exc:
            raise ValueError(f"无法打开串口 {self.port}：{exc}") from exc
        with self._lock:
            self._serial = serial_port
            self._sensor_frames.clear()
            self._acks.clear()
            self._faults.clear()
            self._rx_bytes = self._rx_lines = self._rx_bad_lines = 0
            self.connected = True
            self.message = "已连接，等待 JSON 帧"
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, name="vitac-shared-rx", daemon=True)
        self._thread.start()

    def close(self) -> None:
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
            self._sensor_frames.clear()
            self._acks.clear()
            self._faults.clear()
            self.message = "串口已关闭"

    # ------------------------------------------------------------ 发送
    def write_line(self, text: str) -> None:
        """写一行（自动补换行）。串口不可用时抛异常，不假装成功。

        ★ Arduino Uno 的 RX 缓冲只有 64 字节，而完整 JSON 命令约 70-90 字节，
        一次 write 会被截断，板端永远凑不出一行 -> 命令完全收不到。
        所以这里**分两段发**，中间留一点间隔让板端的串口中断把 RX 填满、
        主循环有机会取走。这样 90 字节以内的命令都能完整送达。
        """
        if not isinstance(text, str) or not text:
            raise ValueError("要发送的内容必须是非空字符串")
        payload = (text.rstrip("\r\n") + "\n").encode("utf-8")
        with self._write_lock:
            with self._lock:
                serial_port = self._serial
                connected = self.connected
            if serial_port is None or not connected:
                raise RuntimeError("串口未连接，无法发送命令")
            try:
                if len(payload) <= 40:
                    serial_port.write(payload)
                else:
                    # ★ 分多段发，每段不超过 32 字节（远小于 Uno 的 64 字节 RX 缓冲），
                    #   段间 sleep 让板端的串口中断把 RX 填满、主循环取走。
                    #   一次 write 超过 64 字节会被截断，板端永远凑不出一行。
                    chunk = 32
                    for i in range(0, len(payload), chunk):
                        serial_port.write(payload[i:i + chunk])
                        if i + chunk < len(payload):
                            time.sleep(0.03)
            except Exception as exc:
                raise RuntimeError(f"串口写入失败：{exc}") from exc

    # ------------------------------------------------------------ 接收
    def latest_sensor_frame(self) -> dict | None:
        with self._lock:
            return self._sensor_frames[-1] if self._sensor_frames else None

    def wait_for_ack(self, command_id: int, timeout: float) -> dict | None:
        """等待与 command_id 匹配的回执。

        这是**同步 ACK 契约**：拿不到匹配回执就不能声称已确认。
        超时返回 None，调用方必须据此返回 acknowledged=False。
        """
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            with self._lock:
                for frame in self._acks:
                    if frame.get("command_id") == command_id:
                        return frame
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.005)

    def take_fault(self) -> dict | None:
        with self._lock:
            return self._faults.popleft() if self._faults else None

    def stats(self) -> dict:
        with self._lock:
            return {
                "bytes": self._rx_bytes,
                "lines": self._rx_lines,
                "bad_lines": self._rx_bad_lines,
                "sensor_pending": len(self._sensor_frames),
                "acks_pending": len(self._acks),
                "faults_pending": len(self._faults),
                "last_rx_age_ms": int((time.monotonic() - self._last_rx_at) * 1000)
                if self._last_rx_at else None,
            }

    # ------------------------------------------------------------ 内部
    def _route(self, line: str) -> None:
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            return
        try:
            payload = json.loads(stripped)
        except (json.JSONDecodeError, ValueError):
            with self._lock:
                self._rx_bad_lines += 1
            return
        if not isinstance(payload, dict):
            with self._lock:
                self._rx_bad_lines += 1
            return
        kind = payload.get("type")
        with self._lock:
            self._rx_lines += 1
            self._last_rx_at = time.monotonic()
            if kind == "ack":
                self._acks.append(payload)
            elif kind == "fault":
                self._faults.append(payload)
            elif kind in (None, "sensor"):
                self._sensor_frames.append(payload)
            else:
                # 未知 type：不进传感流，避免污染数据校验
                self._rx_bad_lines += 1

    def _reader(self) -> None:
        buffer = bytearray()
        discarding = False
        try:
            while not self._stop.is_set():
                serial_port = self._serial
                if serial_port is None:
                    return
                data = serial_port.read(min(max(serial_port.in_waiting, 1), 4096))
                if not data:
                    continue
                with self._lock:
                    self._rx_bytes += len(data)
                for byte in data:
                    if discarding:
                        if byte == 10:
                            discarding = False
                        continue
                    if byte == 10:
                        try:
                            self._route(bytes(buffer).decode("utf-8", errors="replace"))
                        except Exception:
                            with self._lock:
                                self._rx_bad_lines += 1
                        buffer.clear()
                    elif len(buffer) >= MAX_LINE_BYTES:
                        buffer.clear()
                        discarding = True
                        with self._lock:
                            self._rx_bad_lines += 1
                    else:
                        buffer.append(byte)
        except Exception as exc:
            if not self._stop.is_set():
                with self._lock:
                    self.message = f"串口读取中断：{exc}"
                    self.connected = False
        finally:
            serial_port = self._serial
            if serial_port is not None:
                try:
                    serial_port.close()
                except Exception:
                    pass
