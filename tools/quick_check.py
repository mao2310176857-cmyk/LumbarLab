"""板子插回来后的一键自检。

用法：
    python quick_check.py [COM口] [波特率]

自动做 4 件事：
  1. 扫描所有模拟口（A0..A15），报告读数 + 电压
  2. 继电器电压自检：6 种组合，读回 D8/D9 真实电压
  3. 引脚交叉验证：digitalRead（我们设的）vs analogRead（真实电压）
  4. PORTB 寄存器检查：直接读写寄存器，最权威
"""
import re
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000

s = serial.Serial(PORT, BAUD, timeout=1.0)
s.rts = False
s.dtr = False
time.sleep(3.0)
s.reset_input_buffer()


def read(ms=2000):
    buf = ""
    t0 = time.time()
    while time.time() - t0 < ms / 1000:
        d = s.read(4096)
        if d:
            buf += d.decode("ascii", errors="replace")
    return buf


def run(cmd, wait=2500):
    s.write(cmd)
    time.sleep(0.4)
    return read(wait)


def volts(v):
    return f"{v * 5.0 / 1023:.2f}V"


print(f"=== 自检 {PORT} @ {BAUD} ===\n")

# 1) 模拟口扫描
out = run(b"s\n", 4000)
print("[1] 各模拟口读数")
names = {0: "#1右腿", 1: "#2左腿", 2: "#3左坐骨", 3: "#4右坐骨",
         4: "#5腰右", 5: "#6腰中", 6: "#7腰左", 7: "#8靠背"}
for line in out.split("\n"):
    m = re.match(r"#\s*A(\d+)\s+raw=(\d+)", line.strip())
    if m:
        k, raw = int(m.group(1)), int(m.group(2))
        tag = names.get(k, "=D8继电器A" if k == 8 else "=D9继电器B" if k == 9 else "")
        flag = "  ★ 0V，没接传感器或短路" if raw == 0 else ""
        print(f"   A{k:<3} {tag:<12} = {raw:<5} ({volts(raw)}){flag}")

# 2) 继电器电压自检
out = run(b"^\n", 4000)
print("\n[2] 继电器电压自检（固件内部原子读取）")
for line in out.split("\n"):
    if line.strip().startswith("# ") and "D8=" in line:
        print("  ", line.strip())

# 3) 引脚交叉验证
out = run(b"(\n", 3000)
print("\n[3] 引脚交叉验证")
for line in out.split("\n"):
    if line.strip().startswith("# ") and "digitalRead" in line:
        print("  ", line.strip())

# 4) PORTB 寄存器
out = run(b"&\n", 3000)
print("\n[4] PORTB 寄存器检查（最权威）")
for line in out.split("\n"):
    if line.strip().startswith("# ") and ("PORTB" in line or "PB" in line):
        print("  ", line.strip())

s.close()
print("\n=== 结束 ===")
print("判读：PORTB 说 LOW 但 A8 还是 1023 -> D8 被外部电路拉高，")
print("      继电器 IN 脚只能接 D8/D9，不能接 VCC 或 GND。")
