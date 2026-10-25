"""端到端联调自检：不接硬件也能验证协议与状态机是否正确。

用法：
    python tools/selftest.py

覆盖：
    1. 板端 JSON 帧能被 parse_frame 严格校验通过
    2. 8 个传感器落位 + zones.json 映射能正确聚合出八区
    3. zones.json 索引无重复、在范围内、八区非空
    4. 共享传输层能按 type 路由 sensor / ack / fault
    5. 执行器适配器的同步 ACK 契约（收到回执才算确认）
    6. 过期命令不得启动运动
    7. 板端硬时限 <= BOARD_MAX_MS
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from lumbar.devices import parse_frame, load_mapping, map_zones      # noqa: E402
from lumbar.transport import SharedSerialTransport                     # noqa: E402
from lumbar.actuators import ActuatorCommand                          # noqa: E402

PASS, FAIL = "\033[92mPASS\033[0m", "\033[91mFAIL\033[0m"
results = []


def check(name, condition, detail=""):
    results.append((name, bool(condition), detail))
    mark = PASS if condition else FAIL
    print(f"  [{mark}] {name}" + (f"  — {detail}" if detail else ""))
    return bool(condition)


# 板端落位规则：传感器 k(1..8) -> 矩阵索引 (k-1)*16 + k
CELLS = [k * 16 + k + 1 for k in range(8)]


def build_board_frame(seq=1, base=1, values=None):
    """构造一帧与固件输出结构一致的 JSON。"""
    diff = [0] * 256
    raw = [0] * 256
    values = values or [30, 60, 90, 20, 120, 45, 15, 5]
    for k, cell in enumerate(CELLS):
        diff[cell] = values[k]
        raw[cell] = 255 - values[k]
    return {
        "type": "sensor", "seq": seq, "t": seq * 100, "base": base,
        "diff": diff, "raw": raw,
        "peak": max(diff), "pr": 4, "pc": 5, "nz": 8, "sum": sum(diff),
    }


def main():
    print("\n=== 1. 板端帧通过 parse_frame 严格校验 ===")
    frame = build_board_frame()
    line = json.dumps(frame, separators=(",", ":"))
    try:
        parsed = parse_frame(line)
        check("parse_frame 接受板端帧", True, f"{len(line)} 字节")
        check("必需字段齐全", all(k in parsed for k in ("seq", "t", "base", "diff", "raw")))
    except ValueError as exc:
        check("parse_frame 接受板端帧", False, str(exc))
        return

    print("\n=== 2. 坏帧必须被拒绝 ===")
    bad_cases = [
        ("缺 diff", {"seq": 1, "t": 1, "base": 1}),
        ("diff 不是 256", {**frame, "diff": [0] * 255}),
        ("值超出 0..255", {**frame, "diff": [999] * 256}),
        ("base 不是 0/1", {**frame, "base": 2}),
        ("seq 是浮点", {**frame, "seq": 1.5}),
    ]
    for label, obj in bad_cases:
        try:
            parse_frame(json.dumps(obj, separators=(",", ":")))
            check(f"拒绝：{label}", False, "居然通过了")
        except ValueError:
            check(f"拒绝：{label}", True)

    print("\n=== 3. zones.json 映射 ===")
    try:
        mapping = load_mapping(ROOT / "config" / "zones.json")
        check("zones.json 可加载", True, f"{len(mapping)} 个区")
    except ValueError as exc:
        check("zones.json 可加载", False, str(exc))
        return
    check("八区全部非空", all(mapping.values()),
          "空区会让 map_zones 返回 None，自动控制被锁")
    allidx = [i for v in mapping.values() for i in v]
    check("索引无重复", len(allidx) == len(set(allidx)))
    check("索引在 0..255", all(0 <= i < 256 for i in allidx))
    check("区1..区8 键齐全", set(mapping) == {str(n) for n in range(1, 9)})

    zones = map_zones(frame["diff"], mapping)
    check("map_zones 返回 8 个值", zones is not None and len(zones) == 8)
    if zones:
        print(f"        分区值: {[round(z, 1) for z in zones]}")
        check("分区值都在 0..255", all(0 <= z <= 255 for z in zones))
        check("分区非零（有响应）", any(z > 0 for z in zones))

    print("\n=== 4. 共享传输层消息路由 ===")
    transport = SharedSerialTransport("COM0", 115200)
    # 直接测试 _route，不真的开串口
    transport._route(json.dumps(build_board_frame(1), separators=(",", ":")))
    transport._route(json.dumps({"type": "ack", "session": "s", "command_id": 7,
                                 "accepted": True, "status": "accepted"}))
    transport._route(json.dumps({"type": "fault", "session": "s",
                                 "code": "LIMIT", "motion": "stopped"}))
    transport._route("# 诊断行应被忽略")
    transport._route("这不是 JSON")
    routed = transport.latest_sensor_frame()
    check("sensor 帧进入传感队列", routed is not None and routed.get("seq") == 1)
    check("ack 进入回执队列", len(transport._acks) == 1)
    check("fault 进入故障队列", len(transport._faults) == 1)
    check("坏行被计数", transport.stats()["bad_lines"] == 1,
          f"bad_lines={transport.stats()['bad_lines']}")
    ack = transport.wait_for_ack(7, timeout=0.2)
    check("按 command_id 取回执", ack is not None and ack["accepted"] is True)
    check("不匹配的命令号取不到", transport.wait_for_ack(99, timeout=0.1) is None)

    print("\n=== 5. 执行器同步 ACK 契约 ===")
    from integrations.hardware_adapter import HardwareActuator, MAX_MOVE_MS

    class FakeDevice:
        """假传感设备：带一个共享 transport，让适配器能拿到写通道。"""

        def __init__(self, tp):
            self.transport = tp

    # 假 transport：write_line 会自动回一条匹配的 ack，模拟板端行为
    class LoopbackTransport:
        """模拟板端：收到命令立刻回 ack，并记录收到的命令。"""

        connected = True

        def __init__(self):
            self.sent = []
            self.auto_ack = True
            self.ack_ok = True

        def write_line(self, text):
            self.sent.append(json.loads(text))
            if not self.auto_ack:
                return
            cmd = json.loads(text)
            self._pending_ack = {
                "type": "ack", "session": cmd.get("session", ""),
                "command_id": cmd.get("command_id", 0),
                "accepted": self.ack_ok, "status": "accepted" if self.ack_ok else "rejected",
            }

        def wait_for_ack(self, command_id, timeout):
            ack = getattr(self, "_pending_ack", None)
            if ack and ack["command_id"] == command_id:
                self._pending_ack = None
                return ack
            return None

        def close(self):
            pass

    lb = LoopbackTransport()
    actuator = HardwareActuator(FakeDevice(lb))
    check("握手成功后 enabled=True", actuator.enabled is True, actuator._last_message)
    check("握手发的是 stop", lb.sent and lb.sent[0].get("action") == "stop")

    print("\n=== 6. 移动命令的 ACK 与时限 ===")
    lb.sent.clear()
    now = time.monotonic()
    result = actuator.execute(ActuatorCommand("advance", issued_at=now, speed=5.0, duration_s=0.5))
    check("移动命令已发送", len(lb.sent) == 1 and lb.sent[0]["action"] == "advance")
    check("携带 session", bool(lb.sent[0].get("session")))
    check("携带 command_id", isinstance(lb.sent[0].get("command_id"), int))
    check("remaining_ms 换算正确", 400 <= lb.sent[0]["remaining_ms"] <= 500,
          f"remaining_ms={lb.sent[0]['remaining_ms']}")
    check("收到回执才 acknowledged", result.acknowledged is True, result.message)

    print("\n=== 7. 过期命令不得启动运动 ===")
    lb.sent.clear()
    expired = ActuatorCommand("advance", issued_at=time.monotonic() - 10.0,
                              speed=5.0, duration_s=0.5)
    result = actuator.execute(expired)
    check("过期命令不启动运动",
          all(c.get("action") != "advance" for c in lb.sent),
          f"实际发出: {[c.get('action') for c in lb.sent]}")
    check("过期命令转为停止并确认",
          result.acknowledged and lb.sent[0]["action"] == "stop",
          "过期命令应降级为 stop，并如实报告停止已被确认")

    print("\n=== 8. 超时命令被限幅 ===")
    lb.sent.clear()
    actuator.execute(ActuatorCommand("advance", issued_at=time.monotonic(),
                                     speed=5.0, duration_s=1.9))
    check("remaining_ms 不超过板端上限",
          lb.sent[0]["remaining_ms"] <= MAX_MOVE_MS,
          f"remaining_ms={lb.sent[0]['remaining_ms']}, 上限 {MAX_MOVE_MS}")

    print("\n=== 9. 板端拒绝时不得声称已确认 ===")
    lb.auto_ack = False
    result = actuator.execute(ActuatorCommand("retract", issued_at=time.monotonic(),
                                              speed=5.0, duration_s=0.4))
    check("无回执 -> acknowledged=False", result.acknowledged is False, result.message)
    lb.auto_ack = True
    lb.ack_ok = False
    result = actuator.execute(ActuatorCommand("retract", issued_at=time.monotonic(),
                                              speed=5.0, duration_s=0.4))
    check("板端拒绝 -> acknowledged=False", result.acknowledged is False, result.message)
    lb.ack_ok = True

    print("\n=== 10. 板端时限上限与帧率估算 ===")
    check("MAX_MOVE_MS 与固件一致 (2000)", MAX_MOVE_MS == 2000)

    print("\n=== 11. 帧率与带宽估算 ===")
    line_len = len(json.dumps(build_board_frame(), separators=(",", ":")).encode("utf-8"))
    for baud in (115200, 2000000):
        # 10 bits per byte on the wire
        tx_s = line_len * 10 / baud
        fps = 1 / tx_s
        print(f"        {baud:>7} baud: 每帧 {line_len} 字节, 发送耗时 {tx_s*1000:.1f} ms"
              f", 理论上限 {fps:.0f} 帧/秒")
    check("2000000 下帧率充足", 1 / (line_len * 10 / 2000000) >= 20,
          "网页约 200ms 轮询，20 帧/秒足够")
    check("115200 下仍能工作（降采样用）", 1 / (line_len * 10 / 115200) >= 8,
          "10 帧/秒，够网页轮询显示")

    passed = sum(1 for _, ok, _ in results if ok)
    total = len(results)
    print(f"\n{'='*60}\n  通过 {passed}/{total}\n{'='*60}")
    if passed != total:
        print("  失败项：")
        for name, ok, detail in results:
            if not ok:
                print(f"    - {name}  {detail}")
        sys.exit(1)
    print("  全部通过\n")


def _raises(fn):
    try:
        fn()
        return False
    except Exception:
        return True


if __name__ == "__main__":
    main()
