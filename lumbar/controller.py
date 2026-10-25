"""Personal reference and bounded feedback controller; no hardware I/O."""
import math
import statistics
from datetime import datetime, timezone

# A frame older than this is stale. 0.8 s is the safety contract: a real
# dropout must pause the controller quickly. The board therefore has to keep
# frames flowing at well under this interval.
FRAME_STALE_S = 0.8

DEFAULT_PARAMETERS = {"tolerance": 0.18, "persistence_s": 0.6, "speed": 5.0, "settle_s": 0.6}
PARAMETER_BOUNDS = {"tolerance": (0.08, 0.4), "persistence_s": (0.2, 3), "speed": (1, 10), "settle_s": (0.2, 3)}


def valid_zones(zones):
    return isinstance(zones, (list, tuple)) and len(zones) == 8 and all(
        isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) and 0 <= x <= 255 for x in zones)


class Controller:
    def __init__(self, parameters=None, ignored_zones=(), tuning=None):
        # Zones whose sensor is not connected; they are excluded from the
        # contact/occupied averages and the reference requirements.
        self.ignored_zones = tuple(ignored_zones)
        # Optional threshold overrides for the real hardware (config/tuning.json).
        # Absent keys keep the original values, so the shipped behaviour and
        # the test suite are unaffected.
        tuning = tuning or {}
        self.max_waist_level = tuning.get("max_waist_level", 245)
        self.min_waist_contact = tuning.get("min_waist_contact", 3)
        self.min_seat_occupied = tuning.get("min_seat_occupied", 5)
        self.parameters = dict(DEFAULT_PARAMETERS)
        self.reference = None
        self.mode, self.action, self.reason = "manual", "hold", "手动模式 · 先调整到舒服的位置"
        self.filtered = None
        self.candidate = None
        self.candidate_since = 0.0
        self.settle_until = 0.0
        self.motion_until = 0.0
        self.manual_until = 0.0
        self.occupied = False
        self.contact = False
        self.metrics = {"lumbar": 0.0, "target": None, "lower": None, "upper": None}
        if parameters:
            self.set_parameters(parameters)

    def set_parameters(self, values):
        checked = {}
        for key, val in values.items():
            if key not in PARAMETER_BOUNDS:
                raise ValueError("未知控制参数")
            lo, hi = PARAMETER_BOUNDS[key]
            if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val) or not lo <= val <= hi:
                raise ValueError(f"参数 {key} 必须在 {lo} 到 {hi} 之间")
            checked[key] = float(val)
        self.parameters.update(checked)
        self.pause("参数已更新 · 请重新开启自动")

    def set_reference(self, samples, position=None):
        if len(samples) < 20 or not all(valid_zones(x) for x in samples):
            raise ValueError("有效样本不足，请保持坐稳后重试")
        means = [statistics.mean(s[i] for s in samples) for i in range(8)]
        noise = [statistics.pstdev(s[i] for s in samples) for i in range(8)]
        live_waist_ref = [m for i, m in enumerate(means[4:7])
                          if (i + 5) not in self.ignored_zones]
        if (statistics.mean(means[:4]) < self.min_seat_occupied
                or not live_waist_ref or min(live_waist_ref) < self.min_waist_contact):
            raise ValueError("座面和 5、6、7 区需要保持接触，才能记住支撑")
        if max(means) >= self.max_waist_level - 5:
            raise ValueError("读数接近量程上限，请先检查接触和传感器")
        if any(noise[i] > max(2.0, means[i] * 0.08) or
               max(s[i] for s in samples) - min(s[i] for s in samples) > max(8.0, means[i] * 0.3)
               for i in range(7)):
            raise ValueError("采集期间变化过大，请坐稳并重新记住支撑")
        self.reference = {"zones": means, "noise": noise, "position": position,
                          "created_at": datetime.now(timezone.utc).isoformat()}
        self.filtered = means[:]
        self.pause("已记住个人支撑 · 可以开启自动")
        self._metrics()

    def start_auto(self):
        if self.reference is None:
            raise ValueError("请先记住当前支撑")
        self.mode, self.action, self.reason = "auto", "hold", "自动跟随 · 支撑在参考范围内"
        self.candidate = None
        self.settle_until = 0.0

    def pause(self, reason="自动已暂停"):
        self.mode, self.action, self.reason = "paused", "stop", reason
        self.candidate = None
        self.manual_until = self.motion_until = 0.0

    def stop(self):
        self.pause("已停止 · 需要明确操作后才会继续")

    def manual(self, direction, now):
        if direction not in ("advance", "retract"):
            raise ValueError("未知手动方向")
        self.mode, self.action = "manual", direction
        self.reason = "手动前顶一小步" if direction == "advance" else "手动回退一小步"
        if self.manual_until <= now:
            self.manual_until = now + 2.0 / self.parameters["speed"]
        self.candidate = None

    def _metrics(self):
        lumbar = statistics.mean(self.filtered[4:7]) if self.filtered else 0.0
        self.metrics = {"lumbar": lumbar, "target": None, "lower": None, "upper": None}
        if self.reference:
            target = statistics.mean(self.reference["zones"][4:7])
            margin = max(target * self.parameters["tolerance"], statistics.mean(self.reference["noise"][4:7]) * 3, 3)
            self.metrics.update(target=target, lower=target - margin, upper=target + margin)

    def update(self, zones, now, position, valid=True, age=0.0, fresh=True):
        good = valid and valid_zones(zones)
        # A zone whose sensor is not connected would otherwise drag the
        # average down and make contact/occupied never register.
        live_seat = [z for i, z in enumerate(zones[:4])
                     if (i + 1) not in self.ignored_zones]
        live_waist = [z for i, z in enumerate(zones[4:7])
                      if (i + 5) not in self.ignored_zones]
        self.occupied = bool(good and live_seat
                            and statistics.mean(live_seat) >= self.min_seat_occupied)
        self.contact = bool(good and live_waist
                            and statistics.mean(live_waist) >= self.min_waist_contact)
        if good and fresh:
            self.filtered = list(zones) if self.filtered is None else [a * 0.3 + b * 0.7 for a, b in zip(zones, self.filtered)]
        self._metrics()
        guard = None
        if age > FRAME_STALE_S:
            guard = "数据中断 · 恢复后请重新开启"
        elif not good:
            guard = "读数无效 · 请检查数据后重新开启"
        elif self.mode != "manual":
            # The ergonomic heuristics only apply to automatic control.
            # In manual mode the user is deliberately driving the motor on the
            # bench; nobody has to be sitting there, and no signal threshold
            # may cut a movement the user just requested.
            if not self.occupied:
                guard = "检测到离座 · 已暂停"
            elif not self.contact:
                guard = "腰部接触丢失 · 已暂停"
            elif max(zones[4:7]) >= self.max_waist_level:
                guard = "腰部信号接近量程上限 · 已停止"
        if guard:
            if self.mode == "auto" or self.action in ("advance", "retract"):
                self.pause(guard)
            elif self.mode == "manual":
                self.reason = guard
            return self.snapshot()
        if self.action in ("advance", "retract") and position is not None:
            if (self.action == "advance" and position >= 99.9) or (self.action == "retract" and position <= 0.1):
                self.pause("到达模拟行程边界 · 已停止")
                return self.snapshot()
        if self.mode == "manual":
            if now >= self.manual_until:
                self.action, self.reason = "hold", "手动模式 · 先调整到舒服的位置"
            return self.snapshot()
        if self.mode != "auto":
            return self.snapshot()
        target = self.metrics["target"]
        tol = self.parameters["tolerance"]
        ratio = self.metrics["lumbar"] / target
        local = max(self.filtered[i] / self.reference["zones"][i] for i in (4, 5, 6))
        if self.action in ("advance", "retract"):
            reached = (self.action == "advance" and (ratio >= 1 - tol * 0.4 or local > 1 + tol)) or (
                self.action == "retract" and ratio <= 1 + tol * 0.4 and local <= 1 + tol * 0.6)
            if reached or now >= self.motion_until:
                self.action = "hold"
                self.reason = "动作完成 · 等待接触稳定"
                self.settle_until = now + self.parameters["settle_s"]
                self.candidate = None
            return self.snapshot()
        if now < self.settle_until:
            self.action, self.reason = "hold", "保持 · 等待接触稳定"
            return self.snapshot()
        if not fresh:
            return self.snapshot()
        desired = "retract" if self.metrics["lumbar"] > self.metrics["upper"] or local > 1 + tol * 1.5 else (
            "advance" if self.metrics["lumbar"] < self.metrics["lower"] and local <= 1 + tol * 0.6 else "hold")
        if desired == "hold":
            reason = "保持 · 局部接触偏强，请手动调整" if self.metrics["lumbar"] < self.metrics["lower"] else "保持 · 支撑在个人参考范围内"
            self.action, self.reason, self.candidate = "hold", reason, None
        elif desired != self.candidate:
            self.candidate, self.candidate_since = desired, now
            self.reason = "观察持续变化 · 暂不动作"
        elif now - self.candidate_since >= self.parameters["persistence_s"]:
            self.action = desired
            self.reason = "接触偏轻 · 缓慢前顶" if desired == "advance" else "接触偏强 · 缓慢回退"
            self.motion_until = now + 0.8
            self.candidate = None
        return self.snapshot()

    def snapshot(self):
        return {"mode": self.mode, "action": self.action, "reason": self.reason,
                "occupied": self.occupied, "contact": self.contact, "metrics": dict(self.metrics)}
