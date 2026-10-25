"""传感器实时观察工具 —— 用来标定量程。

用法：
    python tools/sensor_watch.py [COM口] [波特率]

它会持续打印 8 个通道的 10 位原始 ADC 值和压力值，
你可以一边看一边用手压传感器，从而知道：
    - 真实满量程是多少（不是猜的 1023，是实测的）
    - 哪个通道接在哪个位置
    - 6 号通道到底有没有信号

按 Ctrl+C 退出。
"""
from __future__ import annotations

import json
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000

#  8 个传感器在 256 点矩阵里的固定位置
CELLS = [k * 16 + k + 1 for k in range(8)]
NAMES = ["#1 右腿", "#2 左腿", "#3 左坐骨", "#4 右坐骨",
         "#5 腰右", "#6 腰中", "#7 腰左", "#8 靠背"]


def main():
    s = serial.Serial(PORT, BAUD, timeout=0.5)
    s.rts = False
    s.dtr = False
    time.sleep(2.0)
    s.reset_input_buffer()

    print(f"监听 {PORT} @ {BAUD}")
    print("每行 = 一次采样：8 个通道的 10 位原始 ADC 值")
    print("用手指压传感器，看哪一列变化、变化多少")
    print("Ctrl+C 退出\n")

    buf = ""
    lo = [9999] * 8
    hi = [0] * 8
    samples = 0

    try:
        while True:
            data = s.read(4096)
            if not data:
                continue
            buf += data.decode("ascii", errors="replace")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line.startswith("{"):
                    continue
                try:
                    frame = json.loads(line)
                except json.JSONDecodeError:
                    continue
                raw = frame.get("raw")
                if not raw:
                    continue
                vals = [raw[c] for c in CELLS]
                # raw 是 8 位刻度（= 10位 >> 2），换算回 10 位便于对照
                vals10 = [v * 4 for v in vals]
                for i, v in enumerate(vals10):
                    lo[i] = min(lo[i], v)
                    hi[i] = max(hi[i], v)
                samples += 1
                if samples % 2 == 0:
                    print("  " + "  ".join(f"{v:4d}" for v in vals10))
    except KeyboardInterrupt:
        pass
    finally:
        s.close()

    print(f"\n=== 采样 {samples} 次，各通道范围（10 位刻度）===")
    for i in range(8):
        span = hi[i] - lo[i]
        print(f"  {NAMES[i]:<10} {lo[i]:>5} ~ {hi[i]:<5}  变化幅度 = {span}")
    if samples:
        best = max(range(8), key=lambda i: hi[i] - lo[i])
        print(f"\n变化最大的是 {NAMES[best]}（幅度 {hi[best]-lo[best]}）")
        if hi[best] - lo[best] < 50:
            print("★ 变化幅度很小：手指轻按不足以产生形变，"
                  "需要真的把传感器压扁/坐上去才会明显变化。")


if __name__ == "__main__":
    main()
