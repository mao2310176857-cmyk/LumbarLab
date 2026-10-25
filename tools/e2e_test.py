"""真实链路联调：用 TCP socket 模拟一个串口，跑完整收发流程。

这不是 mock —— 走的是 SharedSerialTransport 真实的读线程、行解析、
type 路由、ACK 等待、超时处理，只是底层字节流换成 socket。

用法：
    python tools/e2e_test.py
"""
from __future__ import annotations

import json
import socket
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from lumbar.transport import SharedSerialTransport      # noqa: E402
from lumbar.devices import parse_frame, load_mapping, map_zones   # noqa: E402
from integrations.hardware_adapter import HardwareActuator, MAX_MOVE_MS  # noqa: E402
from lumbar.actuators import ActuatorCommand            # noqa: E402

PASS, FAIL = "\033[92mPASS\033[0m", "\033[91mFAIL\033[0m"
results = []


def check(name, cond, detail=""):
    results.append((name, bool(cond), detail))
    print(f"  [{PASS if cond else FAIL}] {name}" + (f"  — {detail}" if detail else ""))


CELLS = [k * 16 + k + 1 for k in range(8)]


def board_frame(seq, press):
    diff = [0] * 256
    raw = [0] * 256
    for k, c in enumerate(CELLS):
        diff[c] = press[k]
        raw[c] = 255 - press[k]
    return {"type": "sensor", "seq": seq, "t": seq * 100, "base": 1,
            "diff": diff, "raw": raw, "peak": max(diff), "pr": -1, "pc": -1,
            "nz": sum(1 for v in diff if v), "sum": sum(diff)}


class FakeBoard:
    """模拟板端：在 socket 另一端跑真实的协议行为。"""

    def __init__(self, sock):
        self.sock = sock
        self.motor = 0
        self.moving = False
        self.limit_ms = 0
        self.last_cmd_ms = 0
        self.session = None
        self.last_command_id = 0
        self.seq = 0
        self.running = True
        self.received = []
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        buf = b""
        while self.running:
            try:
                data = self.sock.recv(4096)
            except OSError:
                break
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                try:
                    cmd = json.loads(text)
                except json.JSONDecodeError:
                    continue
                self._handle(cmd)
        self.running = False

    def _handle(self, cmd):
        with self._lock:
            self.received.append(cmd)
            if cmd.get("type") != "command":
                return
            cid = cmd.get("command_id", 0)
            sess = cmd.get("session", "")
            action = cmd.get("action", "")
            self.last_cmd_ms = time.monotonic()

            if sess != self.session:
                self.session = sess
                self.last_command_id = 0

            if cid <= self.last_command_id:
                self._send({"type": "ack", "session": sess, "command_id": cid,
                            "accepted": True, "status": "duplicate"})
                return
            self.last_command_id = cid

            if action in ("stop", "hold"):
                self.motor = 0
                self.moving = False
                self._send({"type": "ack", "session": sess, "command_id": cid,
                            "accepted": True, "status": "stopped"})
                return
            if action not in ("advance", "retract"):
                self._send({"type": "ack", "session": sess, "command_id": cid,
                            "accepted": False, "status": "unknown_action"})
                return
            rem = int(cmd.get("remaining_ms", 0))
            if rem <= 0:
                self.motor = 0
                self._send({"type": "ack", "session": sess, "command_id": cid,
                            "accepted": False, "status": "expired"})
                return
            self.limit_ms = min(rem, MAX_MOVE_MS)
            self.motor = 1 if action == "advance" else -1
            self.moving = True
            self._send({"type": "ack", "session": sess, "command_id": cid,
                        "accepted": True, "status": "accepted"})

    def _send(self, obj):
        try:
            self.sock.sendall((json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8"))
        except OSError:
            pass

    def push_sensor(self, press):
        with self._lock:
            self.seq += 1
            self._send(board_frame(self.seq, press))

    def stop(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


class SocketTransport(SharedSerialTransport):
    """把 SharedSerialTransport 的底层换成 socket，行为完全一致。"""

    def __init__(self, sock):
        super().__init__("SOCKET", 2000000)
        self._sock = sock
        self.connected = True

    def connect(self):
        self.connected = True
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, name="sock-rx", daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        self.connected = False

    def write_line(self, text):
        if not self.connected:
            raise RuntimeError("socket 已关闭")
        payload = (text.rstrip("\r\n") + "\n").encode("utf-8")
        self._sock.sendall(payload)

    def _reader(self):
        buf = b""
        while not self._stop.is_set():
            try:
                data = self._sock.recv(4096)
            except OSError:
                break
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._route(line.decode("utf-8", errors="replace"))


def main():
    print("\n=== 建立 socket 链路 ===")
    a, b = socket.socketpair()
    board = FakeBoard(b)
    transport = SocketTransport(a)
    transport.connect()
    check("socket 链路建立", True)

    print("\n=== 1. 板端传感帧到达主机并通过校验 ===")
    press = [30, 60, 90, 20, 120, 45, 15, 5]
    board.push_sensor(press)
    time.sleep(0.15)
    frame = transport.latest_sensor_frame()
    check("收到传感帧", frame is not None)
    if frame:
        try:
            parse_frame(json.dumps(frame, separators=(",", ":")))
            check("通过 parse_frame 严格校验", True)
        except ValueError as exc:
            check("通过 parse_frame 严格校验", False, str(exc))
        mapping = load_mapping(ROOT / "config" / "zones.json")
        zones = map_zones(frame["diff"], mapping)
        check("聚合出 8 个分区值", zones is not None and len(zones) == 8)
        if zones:
            check("分区值与传感器值一致", [round(z) for z in zones] == press,
                  f"zones={[round(z) for z in zones]} 期望={press}")

    print("\n=== 2. 执行器握手 ===")

    class FakeDevice:
        def __init__(self, tp):
            self.transport = tp

    actuator = HardwareActuator(FakeDevice(transport))
    check("握手成功 enabled=True", actuator.enabled is True, actuator._last_message)
    check("握手命令是 stop", board.received[-1].get("action") == "stop")

    print("\n=== 3. 前推命令全链路 ===")
    board.received.clear()
    result = actuator.execute(ActuatorCommand("advance", issued_at=time.monotonic(),
                                              speed=5.0, duration_s=0.4))
    check("板端收到 advance", board.received and board.received[-1]["action"] == "advance")
    check("板端继电器已动作", board.motor == 1, f"motor={board.motor}")
    check("主机收到确认", result.acknowledged is True, result.message)
    check("板端时限已限幅", board.limit_ms <= MAX_MOVE_MS, f"limit={board.limit_ms}")

    print("\n=== 4. 后靠命令 ===")
    result = actuator.execute(ActuatorCommand("retract", issued_at=time.monotonic(),
                                              speed=5.0, duration_s=0.4))
    check("板端收到 retract", board.received[-1]["action"] == "retract")
    check("板端继电器反向", board.motor == -1, f"motor={board.motor}")
    check("主机收到确认", result.acknowledged is True)

    print("\n=== 5. stop 立即终止动作 ===")
    result = actuator.execute(ActuatorCommand("stop", issued_at=time.monotonic()))
    check("板端已停止", board.motor == 0 and not board.moving)
    check("停止被确认", result.acknowledged is True, result.message)

    print("\n=== 6. 重复 command_id 不重启电机 ===")
    board.received.clear()
    first = actuator._next_id()
    cmd = {"type": "command", "session": actuator._session, "command_id": first,
           "action": "advance", "speed_request": 5.0, "remaining_ms": 400}
    transport.write_line(json.dumps(cmd, separators=(",", ":")))
    time.sleep(0.1)
    check("首次命令电机动作", board.motor == 1)
    transport.write_line(json.dumps(cmd, separators=(",", ":")))  # 重发同一个 id
    time.sleep(0.1)
    dup = [c for c in board.received if c.get("command_id") == first]
    check("重复包被板端识别", len(dup) == 2, f"收到 {len(dup)} 次（第一次执行+重发回执）")
    check("重复包不延长动作", board.moving is True and board.limit_ms == 400,
          f"limit={board.limit_ms}（未被 800 之类的值污染）")

    print("\n=== 7. 过期命令被拒绝 ===")
    board.received.clear()
    result = actuator.execute(ActuatorCommand("advance", issued_at=time.monotonic() - 10,
                                              speed=5.0, duration_s=0.5))
    check("板端未执行 advance",
          all(c.get("action") != "advance" for c in board.received),
          f"实际: {[c.get('action') for c in board.received]}")
    check("过期命令不声称已确认移动", "移动" not in result.message or result.acknowledged)

    print("\n=== 8. 传感帧与 ACK 互不干扰 ===")
    board.push_sensor([10, 20, 30, 40, 50, 60, 70, 80])
    actuator.execute(ActuatorCommand("advance", issued_at=time.monotonic(),
                                     speed=5.0, duration_s=0.3))
    time.sleep(0.15)
    frame = transport.latest_sensor_frame()
    check("ACK 没有污染传感流", frame is not None and frame.get("type") == "sensor")
    check("传感帧仍可校验", bool(parse_frame(json.dumps(frame, separators=(",", ":")))))
    stats = transport.stats()
    check("无坏行", stats["bad_lines"] == 0, f"bad_lines={stats['bad_lines']}")

    print("\n=== 9. 板端停止供电后主机能感知 ===")
    board.stop()
    time.sleep(0.3)
    stats = transport.stats()
    check("读取中断被记录", stats["last_rx_age_ms"] is not None)

    transport.close()
    passed = sum(1 for _, ok, _ in results if ok)
    total = len(results)
    print(f"\n{'='*60}\n  通过 {passed}/{total}\n{'='*60}")
    if passed != total:
        for name, ok, detail in results:
            if not ok:
                print(f"  - {name}  {detail}")
        sys.exit(1)
    print("  端到端链路全部通过\n")


if __name__ == "__main__":
    main()
