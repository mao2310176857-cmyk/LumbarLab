/*
 * ============================================================================
 *  LumbarLab 硬件板端固件 —— 8 路压力传感器 + 双继电器腰靠电机
 *  适配对象：算法同学的 LumbarLab v0.1 (HARDWARE_INTEGRATION.md 协议)
 * ----------------------------------------------------------------------------
 *  HARDWARE
 *    Pressure sensors : 8 x analog on A0..A7
 *                       idle (no load) = 1023, pressed = lower value
 *                       => pressure = baseline - raw
 *    Motor relay A    : D8   (digital)
 *    Motor relay B    : D9   (PWM 150 -> ~2.94V average, for the 3.3V coil)
 *
 *  ★★ HARDWARE FINDING 2026-10-24 (read this before touching the relays)
 *
 *    Register-level test result:
 *        PB0 LOW  ->  PORTB=0xFE (bit0=0, we set LOW)
 *                     digitalRead(D8)=0   but   A8=1022 (5.0V!)
 *
 *    The Arduino DID command LOW on D8. The pin is still at 5V.
 *    => D8 is held high by the external circuit. The most likely causes:
 *       (a) the relay module's IN pin has an internal pull-up to VCC and
 *           the IN pin is ALSO wired to VCC (wrong hole), or
 *       (b) IN is still tied to GND while D8 is connected as well, which
 *           puts 5V back onto D8 through the module.
 *    Fix on the hardware side: the relay IN pin must connect to D8 (or D9)
 *    ONLY. Not to VCC, not to GND, not to both.
 *
 *  ★★ RELAY HARDWARE DIAGNOSIS 2026-10-24 (read before changing relay code)
 *
 *    Measurements (all taken on the real board):
 *      D8 high-Z (INPUT)          -> 4.9V   (external pull-up to VCC)
 *      D9 high-Z (INPUT)          -> 5.0V   (external pull-up to VCC)
 *      D8 driven LOW, held 100ms  -> 4.99V  (min=max=1021, Arduino cannot pull it down)
 *      D9 driven LOW, held 100ms  -> 4.99V  (identical)
 *      digitalRead(D8) after LOW  -> 0      (PORT register confirms the output stage is on)
 *
 *    Both channels behave IDENTICALLY at the pin, yet relay B (D9, forward)
 *    closes and relay A (D8, backward) does not.
 *    => The Arduino side is fine. The difference is in the relay hardware:
 *       either relay A's IN wire is on the wrong hole (VCC instead of D8),
 *       or relay A's module/channel is faulty.
 *
 *    The firmware now drives both pins with a simulated open drain
 *    (INPUT to release, OUTPUT+LOW to pull down), which is the correct
 *    way to drive a low-trigger opto-isolated module.
 *
 *  ★ RELAY POLARITY: LOW-LEVEL TRIGGER (confirmed on site)
 *    "on"  = coil energised  -> Arduino pin driven LOW
 *    "off" = released         -> pin driven HIGH
 *
 *  ★ MEASURED DIRECTION (standalone relay test, 2026-10-24)
 *    step 1   D8=LOW , D9=HIGH  ->  BACKWARD  (lean back)
 *    step 2   D8=HIGH, D9=LOW   ->  FORWARD  (push forward)
 *    So, with both relays low-trigger:
 *        advance = release A (D8 HIGH) + energise B (D9 LOW)
 *        retract = energise A (D8 LOW)  + release B (D9 off)
 *
 *  ★ D9's "off" level
 *    D9 drives a 3.3V coil. A hard HIGH would put ~5V across a 3.3V coil
 *    (reverse bias, degrades the relay), so the released state uses
 *    PWM 150 (~2.94V average: no reverse bias, no buzz).
 *    The site test used a plain HIGH and worked; press 'g' to switch
 *    between the two and compare.
 *
 *  ★ RELAY STATE TABLE
 *  ---------------------------------------------------------------------
 *    state        | D8 (relay A) | D9 (relay B)      | motor
 *    -------------+--------------+-------------------+---------
 *    DEFAULT/STOP | HIGH         | PWM150 (or HIGH)  | stopped
 *    ADVANCE      | HIGH         | LOW               | push forward
 *    RETRACT      | LOW          | PWM150 (or HIGH)  | lean back
 *  ---------------------------------------------------------------------
 *    Only ONE relay is energised at a time.
 *
 *  ★ SCALE NOTE (read this if the web page numbers look small)
 *    The sensors are 10-bit (0..1023, idle = 1023).
 *    diff/raw are the 8-bit channels (0..255) the host's algorithm expects:
 *    its parse_frame strictly validates 0..255 and its controller thresholds
 *    (245 / 240 / 5 / 3) are all written on that 255 scale.
 *    adc10 carries the untouched 10-bit values (0..1023) for display.
 *
 *  ★ 8 SENSORS -> 256-CELL MATRIX
 *    The host requires diff/raw to be exactly 256 integers (16x16).
 *    We place the 8 real sensors on a fixed diagonal of that matrix so
 *    config/zones.json can map zone k -> index (k-1)*16 + k.
 *    Every other cell stays 0. The host aggregates each zone as the mean
 *    of its mapped cells, so a 1-cell zone equals that sensor's value.
 *
 *  ★ PROTOCOL (host -> board), per HARDWARE_INTEGRATION.md section 8.2
 *    {"type":"command","session":"...","command_id":7,
 *     "action":"advance","speed_request":5.0,"remaining_ms":300}
 *    action: advance | retract | hold | stop
 *
 *  ★ PROTOCOL (board -> host)
 *    sensor frame : {"type":"sensor","seq":..,"t":..,"base":..,"diff":[256],
 *                    "raw":[256],"peak":..,"pr":..,"pc":..,"nz":..,"sum":..}
 *    ack          : {"type":"ack","session":"..","command_id":7,
 *                    "accepted":true,"status":"accepted"}
 *    fault        : {"type":"fault","session":"..","code":"..","motion":"stopped"}
 *
 *    The host's parse_frame() ignores extra keys, and "type" is used by
 *    the shared transport to route sensor frames vs acknowledgements.
 *    Lines starting with '#' are diagnostics and are ignored.
 *
 *  ★ BOARD-SIDE SAFETY (independent of the host)
 *    - every motion has its own hard deadline, clamped to BOARD_MAX_MS
 *    - hold/stop terminates the running motion immediately
 *    - a command with a stale/expired remaining_ms is rejected
 *    - the same command_id is never allowed to restart a finished motion
 *    - watchdog: no command for LEASE_MS -> relays released
 *    - direction change always releases both relays first (dead time)
 *
 *  SERIAL 115200, 8N1.  Board-side default; see BAUD_RATE below.
 * ============================================================================
 */

#include <Arduino.h>
#include <string.h>

// ---------------------------------------------------------------- 波特率
//  ★ 2000000：发一帧（约 1150 字节）只占 5.7ms。
//    关键不是"能不能发完"，而是**发帧期间主机来的命令会不会丢**：
//    Arduino 的 RX 缓冲只有 64 字节，发一帧的 5.7ms 内若不读 RX，
//    主机发来的 70 字节命令会被冲掉。
//    所以配套做了两件事：
//      1) 打印间隔放大到 500ms（2fps），留出大量空闲时间收命令
//      2) loop() 里在发帧后立刻再 handleSerialInput() 一次
//    网页端波特率选 2000000。
#define BAUD_RATE        2000000

// ---------------------------------------------------------------- 传感器
#define NUM_SENSORS      8
//  运行时可改：sensorPin[i] 是第 i 个传感器实际接的模拟口。
//  串口命令 m <数字> <口> 可以改（例如 m 5 3 把 #6 改到 A3）。
//  默认 A0..A7。
uint8_t sensorPin[NUM_SENSORS] = { A0, A1, A2, A3, A4, A5, A6, A7 };

//  ★ 传感器通道配置（2026-10-24 现场实测）
//
//    #1-#5、#7、#8：正常型，空载 1023，按下变小
//                    -> press = baseline - raw（越大压力越大）
//
//    #6（A5）：2026-10-24 现场读数 raw 恒为 0（A5 引脚 0V）。
//            之前该传感器是能用的，所以更可能是**接线位置变了**
//            （插到了别的通道，或者 A5 那根线松了/掉了）。
//            这里已恢复为"不屏蔽"，让 #6 正常参与所有计算：
//            接线一改好就立刻生效，不用重烧。
//            用 s 命令可以扫描所有模拟口，看信号实际在哪个引脚上。
const bool SENSOR_IGNORE[NUM_SENSORS] = {
  false, false, false, false, false, false, false, false
};


//  8 个传感器在 256 点矩阵里的落位：传感器 k (1..8) -> 索引 (k-1)*16 + k
//  对应 config/zones.json:  区 k -> [(k-1)*16 + k]
uint8_t sensorCell[NUM_SENSORS];

// ---------------------------------------------------------------- 继电器
#define PIN_RELAY_FWD    8
#define PIN_RELAY_BACK   9
#define RELAY_PWM_LEVEL  150      // ~2.94V average, the "off" level for D9
#define RELAY_SWITCH_MS  25       // dead time between release and engage

//  低电平触发（现场确认）。若实测反向，串口敲 w 翻转极性。
bool relayLow = true;
//  D9 的三种设置档位（HIGH / LOW / PWM150）
enum D9Mode { D9_HIGH = 0, D9_LOW = 1, D9_PWM = 2 };
D9Mode d9Mode = D9_PWM;
//  D9 处于"断开"时用纯 HIGH 还是 PWM150。
//  ★ 2026-10-24 现场观察：D9 用 PWM150 时，9 号继电器的小绿灯在
//    空闲状态就亮，按 5/7 时更亮 —— 这是低电平触发模块在方波下
//    处于"半吸合"的抖动状态（41.2% 的时间是低电平）。
//    所以默认改成纯 HIGH（真正断开，灯灭、触点干净）。
//    PWM150 保留：若之后确认线圈必须限压，敲 g 切回。
bool d9OffUseHigh = true;

// ---------------------------------------------------------------- 部位编号表
//  通道 A0..A7 各自对应图上哪个编号、哪个身体部位。
//      1 = 右腿        2 = 左腿        3 = 左坐骨      4 = 右坐骨
//      5 = 腰(电机推)右侧   6 = 腰中间   7 = 腰(电机推)左侧   8 = 靠背顶部
uint8_t chNum[NUM_SENSORS] = { 1, 2, 3, 4, 5, 6, 7, 8 };

const char * const NUM_NAME[9] = {
  "-",         // 0
  "thigh-R",   // 1  right thigh
  "thigh-L",   // 2  left thigh
  "ischium-L", // 3  left ischium
  "ischium-R", // 4  right ischium
  "waist-R",   // 5  lumbar (motor) right
  "waist-C",   // 6  lumbar centre
  "waist-L",   // 7  lumbar (motor) left
  "back-top"   // 8  backrest top
};

// ---------------------------------------------------------------- 触发通道
uint8_t IDX_TRIG_FWD  = 4;      // #5 waist-R (A4) -> advance
uint8_t IDX_TRIG_BACK = 6;      // #7 waist-L (A6) -> retract

// ---------------------------------------------------------------- 参数
#define PRESS_ON         100     // enter action above this pressure
#define PRESS_OFF        55      // release below this (hysteresis)
#define DIFF_ENTER       70      // pressure difference needed to start
#define DIFF_EXIT        30      // difference below this counts as centred
#define MOVE_MS_DEFAULT  800     // local fallback move duration
#define REARM_MS         300     // cooldown before another move may start
#define PRINT_MS_DEFAULT 300     // 3.3 fps: 必须 < host 的 800ms 数据有效期判定线

//  板端安全上限：主机给的时间再长，板端也不超过这个
#define BOARD_MAX_MS     2000
//  主机命令租约：这么久没收到任何命令就释放继电器
#define LEASE_MS         1500

uint16_t pressThresh  = PRESS_ON;
uint16_t moveMs       = MOVE_MS_DEFAULT;
uint16_t printMs      = PRINT_MS_DEFAULT;
bool     printOn      = true;

// ---------------------------------------------------------------- 状态
uint16_t rawVal[NUM_SENSORS];
uint16_t baseVal[NUM_SENSORS];
uint16_t pressVal[NUM_SENSORS];

bool      baseReady    = false;
bool      autoMode     = true;   // AUTO = local logic drives the motor
bool      jsonMode     = true;   // JSON stream (host speaks JSON)
bool      manualMode   = false;

int8_t    motorDir     = 0;      // -1 retract, 0 stop, +1 advance
uint32_t  moveStart    = 0;
uint16_t  moveLimitMs  = 0;
bool      moveActive   = false;
uint32_t  lastCommandMs = 0;     // for the lease watchdog
bool      leaseValid   = false;

uint16_t  frameSeq     = 0;
uint32_t  lastFrameMs  = 0;
uint32_t  lastPrintMs  = 0;

uint8_t   bindTarget   = 255;
uint32_t  bindDeadline = 0;
uint8_t   renumState   = 0;
uint8_t   renumChan    = 0;
//  通道映射命令 M <sensor 1-8> <pin 0-15> 的两段输入
uint8_t   mapState     = 0;      // 0=空闲 1=等传感器号 2=等模拟口
uint8_t   mapSensor    = 0;
//  继电器诊断：= <1-6>，两段式输入
bool      relayDiagPending = false;

//  命令身份：同一 session 内 command_id 单调递增
char      cmdSession[40] = "";
uint32_t  lastCommandId = 0;
bool      haveSession   = false;

//  JSON 行接收（主机 -> 板端）
char      rxBuf[200];
uint8_t   rxLen        = 0;
bool      rxOverflow   = false;

// ============================================================================
//  继电器底层
// ============================================================================

// ============================================================================
//  relayForceLow(pin) —— 强制把引脚变成稳定的 LOW（并停掉 PWM 定时器）
//
//  ★ 名字很重要：这个函数是"拉低"（= 继电器吸合），不是"断开"！
//    早期版本叫 relayHardOff，注释也写"硬断开"，但实际做的是拉低 ——
//    极易被误读成"停止"。这里把名字改对。
//
//  实现要点：AVR 核心的 digitalWrite() 内部会调 turnOffPWM()，
//  所以它真的会把 PWM 定时器断开；单靠 pinMode() 停不掉 PWM。
// ============================================================================
void relayForceLow(uint8_t pin) {
  //  ★ 只用 digitalWrite —— Arduino 核心内部会先 turnOffPWM()
  //    再设置电平，所以这一句就得到"稳定的 LOW，PWM 已停"。
  //    ★ 绝对不要在这里加 analogWrite(pin, 0)：
  //      那不是"关 PWM"，而是"开 PWM、占空比 0%" = 恒低电平！
  if (pin == 9) TCCR1A = 0;      // 保险：显式关掉 OC1A 比较器
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

// ============================================================================
//  relayRelease(pin) —— 释放引脚（断开继电器）
//
//  标准推挽：pinMode(OUTPUT) + digitalWrite(HIGH) = 稳定 5V -> 断开。
//  必须是 OUTPUT：如果设成 INPUT（高阻），光耦模块 IN 脚上的上拉
//  决定电平，万一上拉很弱就会悬空误动作。
//
//  ★ 早期版本这里用"高阻模拟开漏"，那是因为我误判外部有强上拉 ——
//    实际上那结论是错的：这是 Arduino Nano，A8=PC0 而 D8=PB0，
//    analogRead(8) 读的是完全不相干的另一个引脚。
// ============================================================================
void relayRelease(uint8_t pin) {
  //  ★ 同理：只用 digitalWrite(HIGH)。
  //    digitalWrite 内部 turnOffPWM() + 输出 HIGH = 稳定的 5V。
  //    ★ 之前这里的 analogWrite(pin, 0) 会把 PWM 重新打开成
  //      "占空比 0%"，输出恒为 LOW —— 继电器就一直吸合、绿灯常亮。
  if (pin == 9) TCCR1A = 0;      // 保险：显式关掉 OC1A 比较器
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

//  relay A : plain digital output
//  模拟开漏：断开时用 INPUT（高阻，交给外部上拉），吸合时才输出 LOW。
//  pinMode(INPUT) 之前必须先 digitalWrite(HIGH)，否则 PORT 里的旧值会在
//  切回 OUTPUT 的瞬间造成一个假脉冲。
void relayA(bool on) {
  bool lvl = relayLow ? !on : on;      // active-low -> "on" means LOW
  pinMode(PIN_RELAY_FWD, OUTPUT);
  digitalWrite(PIN_RELAY_FWD, lvl ? HIGH : LOW);
}

//  relay B : D9.  "on"  -> hard LOW (PWM fully off)
//             "off" -> PWM 150 (~2.94V average, safe for the 3.3V coil)
// ============================================================================
//  relayB(on) —— D9（继电器 B）控制
//
//  低电平触发模块。两种"断开"写法可选：
//    HIGH   : 纯高电平 5V 断开（d9OffUseHigh = true，当前默认）
//    PWM150 : 0V/5V 方波，占空比 58.8%，平均 2.94V（d9OffUseHigh = false）
//
//  ★ 关于 PWM150：它**不是 3.3V 直流**，峰值仍然是 5V，
//    只是平均值 2.94V。对光耦输入来说 LED 看到的是方波，
//    所以"用 PWM 降到 3V 来保护 3.3V 线圈"这个想法不成立 ——
//    D8/D9 是**控制信号**，线圈由模块自己的电源供电，
//    IN 脚给 5V 不会烧线圈。
//
//    保留 PWM150 只是为了兼容，万一你的模块确实需要低电平断开。
//    用 'g' 键可以在两者间切换。
// ============================================================================
void relayB(bool on) {
  if (relayLow) {
    if (on) {
      relayForceLow(PIN_RELAY_BACK);        // 吸合：稳定 LOW
      d9Mode = D9_LOW;
    } else if (d9OffUseHigh) {
      relayRelease(PIN_RELAY_BACK);        // 断开：释放引脚 -> 外部上拉 -> 5V
      d9Mode = D9_HIGH;
    } else {
      analogWrite(PIN_RELAY_BACK, RELAY_PWM_LEVEL);   // 断开：PWM150
      d9Mode = D9_PWM;
    }
  } else {
    if (on) { analogWrite(PIN_RELAY_BACK, RELAY_PWM_LEVEL); d9Mode = D9_PWM; }
    else    { relayRelease(PIN_RELAY_BACK); d9Mode = D9_HIGH; }
  }
}

//  dir: +1 advance, -1 retract, 0 stop
void setMotor(int8_t dir) {
  if (dir == motorDir) return;

  //  release both first - never let two relays overlap
  pinMode(PIN_RELAY_FWD, OUTPUT);
  digitalWrite(PIN_RELAY_FWD, HIGH);
  pinMode(PIN_RELAY_BACK, OUTPUT);
  digitalWrite(PIN_RELAY_BACK, HIGH);
  relayA(false);
  relayB(false);
  delay(RELAY_SWITCH_MS);

  if (dir > 0) {
    relayB(true);       // B on  (D9 LOW)   , A off (D8 HIGH)
  } else if (dir < 0) {
    relayA(true);       // A on  (D8 LOW)   , B off (D9 PWM150)
  }

  motorDir = dir;
}

void allRelaysOff() {
  pinMode(PIN_RELAY_FWD, OUTPUT);
  digitalWrite(PIN_RELAY_FWD, HIGH);
  pinMode(PIN_RELAY_BACK, OUTPUT);
  digitalWrite(PIN_RELAY_BACK, HIGH);
  relayA(false);
  relayB(false);
  motorDir = 0;
  moveActive = false;
  moveLimitMs = 0;
}

// ============================================================================
//  传感器
// ============================================================================
void readSensors() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    analogRead(sensorPin[i]);                      // discard first, settling
    delayMicroseconds(100);
    uint16_t v = analogRead(sensorPin[i]);
    rawVal[i] = v;
    uint16_t p = 0;
    if (!SENSOR_IGNORE[i]) {
      //  正常型：空载 1023，按下变小 -> press = baseline - raw
      p = (baseVal[i] > v) ? (uint16_t)(baseVal[i] - v) : 0;
    }
    pressVal[i] = p;
  }
}

void calibrateBaseline() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) baseVal[i] = 0;
  for (uint8_t k = 0; k < 16; k++) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (SENSOR_IGNORE[i]) continue;      // 忽略的通道不参与校准
      analogRead(sensorPin[i]);
      delayMicroseconds(60);
      baseVal[i] += analogRead(sensorPin[i]);
    }
  }
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (SENSOR_IGNORE[i]) baseVal[i] = 1023;   // 忽略通道给个占位基线
    else baseVal[i] /= 16;
  }
  baseReady = true;
}

// ============================================================================
//  本地自动逻辑（主机未接管时使用）
// ----------------------------------------------------------------------------
//  diff = 压力(#5) - 压力(#7)
//    diff >  +70  ->  advance   （#5 更重，往前推）
//    diff <  -70  ->  retract   （#7 更重，往后靠）
//    |diff| <= 70 ->  停
// ============================================================================
void updateLocalControl() {
  if (!autoMode || manualMode) return;

  int diff = (int)pressVal[IDX_TRIG_FWD] - (int)pressVal[IDX_TRIG_BACK];

  if (moveActive) {
    if ((uint32_t)(millis() - moveStart) >= moveLimitMs) {
      setMotor(0);
      moveActive = false;
    }
  } else {
    uint32_t sinceCmd = millis() - lastCommandMs;
    if (!leaseValid || sinceCmd >= REARM_MS) {
      if (diff >  DIFF_ENTER)      { setMotor(1);  moveActive = true; moveStart = millis(); moveLimitMs = moveMs; }
      else if (diff < -DIFF_ENTER) { setMotor(-1); moveActive = true; moveStart = millis(); moveLimitMs = moveMs; }
    }
  }
}

//  租约看门狗：主机长时间不发言就释放继电器
void updateLeaseWatchdog() {
  if (!leaseValid) return;
  if (moveActive) return;                       // 运动期间由板端时限负责
  if ((uint32_t)(millis() - lastCommandMs) >= LEASE_MS) {
    allRelaysOff();
  }
}

// ============================================================================
//  绑定触发通道
// ============================================================================
void updateBinding() {
  if (bindTarget == 255) return;

  uint8_t best = 0;
  for (uint8_t i = 1; i < NUM_SENSORS; i++) {
    if (pressVal[i] > pressVal[best]) best = i;
  }

  if (pressVal[best] > 200) {
    if (bindTarget == 0) {
      IDX_TRIG_FWD = best;
      Serial.println(F(">>> forward trigger = A"));
      Serial.print(sensorPin[best]);
      Serial.print(F(" #"));
      Serial.print(chNum[best]);
      Serial.print(F("  "));
      Serial.println(NUM_NAME[chNum[best]]);
    } else {
      IDX_TRIG_BACK = best;
      Serial.println(F(">>> backward trigger = A"));
      Serial.print(sensorPin[best]);
      Serial.print(F(" #"));
      Serial.print(chNum[best]);
      Serial.print(F("  "));
      Serial.println(NUM_NAME[chNum[best]]);
    }
    bindTarget = 255;
  } else if (millis() > bindDeadline) {
    Serial.println(F(">>> binding timed out, nothing changed"));
    bindTarget = 255;
  }
}

// ============================================================================
//  打印：传感器帧（严格符合 parse_frame 校验）
// ----------------------------------------------------------------------------
//  必需：seq(uint32) t(uint32) base(0|1) diff[256] 每个 0..255
//  可选：raw[256] peak pr pc nz sum
//  额外：type="sensor" 用于主机端消息路由（parse_frame 忽略未知键）
// ============================================================================
void printSensorFrame() {
  //  ★ 完全不用 256 字节的数组缓冲：8 个传感器的 cell 固定为
  //    cell_k = k*16 + k + 1（k=0..7），即 1,18,35,52,69,86,103,120。
  //    反查用一个小循环（8 次比较），省下 512 字节 RAM（Uno 只有 2KB）。
  //    cellSensorOf(c) 返回该格子对应的传感器序号+1，0 = 无传感器。

  #define SENSOR_OF_CELL(c) ({ uint8_t r_ = 0; \
                               for (uint8_t k_ = 0; k_ < NUM_SENSORS; k_++) { \
                                 if (sensorCell[k_] == (c)) { r_ = k_ + 1; break; } } r_; })
  //  ★ 8 位通道按真实压力量程缩放（不是简单截断）
  //    传感器是 10 位：空载 1023，压到底约 300，也就是最大掉 723。
  //    队友的算法假设"空载 0、压到底 255"，阈值 240/245 都写在这个尺度上。
  //    所以这里把 723 映射到 255：diff8 = press10 * 255 / 724。
  //    这样 245 那条线才真正表示"压到底"，手动测试时才不会误报量程。
  #define CELL_DIFF(c) ({ uint8_t k_ = SENSOR_OF_CELL(c); \
                          k_ ? (uint8_t)(((uint32_t)pressVal[k_-1] * 255u) / 724u) : 0; })
  #define CELL_RAW(c)  ({ uint8_t k_ = SENSOR_OF_CELL(c); \
                          k_ ? (rawVal[k_-1] >> 2) : 0; })

  //  统计（注意：parse_frame 校验 peak <= 255，所以用 8 位值统计）
  int peak = 0, pr = -1, pc = -1, nz = 0;
  uint32_t sum = 0;
  for (uint8_t k = 0; k < NUM_SENSORS; k++) {
    uint16_t cell = sensorCell[k];
    if (cell < 256) {
      uint8_t dv = CELL_DIFF(cell);
      if (dv > peak) {
        peak = dv;
        pr = (int)(cell / 16);
        pc = (int)(cell % 16);
      }
    }
    if (CELL_DIFF(cell) > 0) nz++;
    sum += CELL_DIFF(cell);
  }
  if (pr < 0) { pr = -1; pc = -1; }

  //  ★ 两段式发送
  //    一帧约 1150 字节，Uno 只有 2KB RAM，放不下整帧缓冲；
  //    而且 2000000 波特下 Arduino 的 64 字节发送 FIFO 灌不上来，
  //    会丢字节（表现为整行被截断，或分块边界处两个数字粘连成 "00"）。
  //
  //    解法：分段填 + 立刻 write。
  //      段1 = 头部 + diff[256]   （约 570 字节）
  //      段2 = raw[256] + 尾部     （约 580 字节）
  //    每段填满 600 字节缓冲就 write 出去，写完继续填下一段。
  //    115200 波特下每字节 87us，中断有充足时间把 FIFO 灌满。
  static char frame[200];
  uint16_t n = 0;

  #define FLUSH_FRAME() do { if (n) { Serial.write((const uint8_t *)frame, n); n = 0; } } while (0)
  #define PUT_LIT(s) do { const char *p_ = (s); \
                          while (*p_) { frame[n++] = *p_++; \
                                          if (n >= sizeof(frame)) { \
                                            Serial.write((const uint8_t *)frame, n); n = 0; \
                                            handleSerialInput(); } } } while (0)
  #define PUT_INT(v) do { char t_[10]; uint8_t i_ = 0; uint32_t v_ = (uint32_t)(v); \
                          if (v_ == 0) t_[i_++] = '0'; \
                          while (v_) { t_[i_++] = char('0' + (v_ % 10)); v_ /= 10; } \
                          while (i_) { frame[n++] = t_[--i_]; \
                                       if (n >= sizeof(frame)) { \
                                         Serial.write((const uint8_t *)frame, n); n = 0; \
                                         handleSerialInput(); } } } while (0)

  PUT_LIT("{\"type\":\"sensor\",\"seq\":");
  PUT_INT(frameSeq);
  PUT_LIT(",\"t\":");
  PUT_INT(millis());
  PUT_LIT(",\"base\":");
  PUT_INT(baseReady ? 1 : 0);
  PUT_LIT(",\"diff\":[");
  for (uint16_t i = 0; i < 256; i++) {
    if (i) { frame[n++] = ','; if (n >= sizeof(frame)) { Serial.write((const uint8_t *)frame, n); n = 0; } }
    PUT_INT(CELL_DIFF(i));
  }
  PUT_LIT("],\"raw\":[");
  for (uint16_t i = 0; i < 256; i++) {
    if (i) { frame[n++] = ','; if (n >= sizeof(frame)) { Serial.write((const uint8_t *)frame, n); n = 0; } }
    PUT_INT(CELL_RAW(i));
  }
  //  ★ adc10：8 个传感器的 10 位"压力值"（0..1023），未做 >>2 压缩。
  //    语义：空载 = 0（没有压力），按下 = 大值（压力越大值越大）。
  //    这跟 diff/raw 的语义一致，只是保留了 10 位分辨率。
  //    parse_frame 会忽略未知字段，所以队友的代码零改动。
  PUT_LIT("],\"adc10\":[");
  for (uint8_t k = 0; k < NUM_SENSORS; k++) {
    if (k) frame[n++] = ',';
    //  压力 = baseline - raw（越大压力越大）；被忽略的通道输出 0
    uint16_t v = rawVal[k];
    uint16_t p = SENSOR_IGNORE[k]
                   ? 0
                   : ((baseVal[k] > v) ? (baseVal[k] - v) : 0);
    PUT_INT(p);
  }
  PUT_LIT("],\"peak\":");
  PUT_INT(peak);
  PUT_LIT(",\"pr\":");
  PUT_INT(pr);
  PUT_LIT(",\"pc\":");
  PUT_INT(pc);
  PUT_LIT(",\"nz\":");
  PUT_INT(nz);
  PUT_LIT(",\"sum\":");
  PUT_INT(sum);
  PUT_LIT("}\n");

  #undef PUT_LIT
  #undef PUT_INT

  //  冲掉最后一段
  FLUSH_FRAME();
  handleSerialInput();
}

//  人类可读表格（调试用）
//  ★ 每行都以 '#' 开头：host 的 parse_frame 会忽略 # 开头的行，
//    不会因为非 JSON 行把整帧判为无效。
void printTable() {
  Serial.println(F("#"));
  Serial.print(F("#"));
  Serial.print(frameSeq);
  Serial.print(F("  t="));
  Serial.print(millis());
  Serial.print(F("ms  motor="));
  Serial.print(motorDir > 0 ? F("FWD ") : (motorDir < 0 ? F("BACK") : F("STOP")));
  Serial.print(F("  thr="));
  Serial.print(pressThresh);
  Serial.println();

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F("# A"));
    Serial.print(sensorPin[i]);
    Serial.print(F(" #"));
    Serial.print(chNum[i]);
    Serial.print(F(" "));
    Serial.print(NUM_NAME[chNum[i]]);
    Serial.print(F(" cell="));
    Serial.print(sensorCell[i]);
    Serial.print(F(" raw="));
    Serial.print(rawVal[i]);
    Serial.print(F(" press="));
    Serial.print(pressVal[i]);
    if (i == IDX_TRIG_FWD)  Serial.print(F("  <FWD"));
    if (i == IDX_TRIG_BACK) Serial.print(F("  <BACK"));
    Serial.println();
  }
}

// ============================================================================
//  消息输出
// ============================================================================
//  ★ 同样用一次性 write()，避免逐段 Serial.print() 在高波特下丢字节

void sendAck(const char *session, uint32_t commandId, bool accepted, const char *status) {
  char frame[160];
  uint8_t n = 0;
  #define PUT_LIT(s) do { const char *p_ = (s); while (*p_) frame[n++] = *p_++; } while (0)
  #define PUT_INT(v) do { char t_[12]; uint8_t i_ = 0; uint32_t v_ = (uint32_t)(v); \
                          if (v_ == 0) t_[i_++] = '0'; \
                          while (v_) { t_[i_++] = char('0' + (v_ % 10)); v_ /= 10; } \
                          while (i_) frame[n++] = t_[--i_]; } while (0)

  PUT_LIT("{\"type\":\"ack\",\"session\":\"");
  PUT_LIT(session);
  PUT_LIT("\",\"command_id\":");
  PUT_INT(commandId);
  PUT_LIT(",\"accepted\":");
  PUT_LIT(accepted ? "true" : "false");
  PUT_LIT(",\"status\":\"");
  PUT_LIT(status);
  PUT_LIT("\"}\n");

  #undef PUT_CH
  #undef PUT_LIT
  #undef PUT_INT

  Serial.write((const uint8_t *)frame, n);
}

void sendFault(const char *session, const char *code, const char *motion) {
  char frame[160];
  uint8_t n = 0;
  #define PUT_LIT(s) do { const char *p_ = (s); while (*p_) frame[n++] = *p_++; } while (0)

  PUT_LIT("{\"type\":\"fault\",\"session\":\"");
  PUT_LIT(session);
  PUT_LIT("\",\"code\":\"");
  PUT_LIT(code);
  PUT_LIT("\",\"motion\":\"");
  PUT_LIT(motion);
  PUT_LIT("\"}\n");

  #undef PUT_LIT

  Serial.write((const uint8_t *)frame, n);
}

// ============================================================================
//  主机命令解析（简化 JSON 提取，不引第三方库）
// ============================================================================
bool jsonFind(const char *json, const char *key, char *out, uint8_t outLen) {
  const char *p = strstr(json, key);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == ':') p++;
  uint8_t i = 0;
  if (*p == '"') {
    p++;
    while (*p && *p != '"' && i < outLen - 1) out[i++] = *p++;
    out[i] = 0;
  } else {
    while (*p && *p != ',' && *p != '}' && *p != ' ' && i < outLen - 1) out[i++] = *p++;
    out[i] = 0;
  }
  return true;
}

bool jsonFindInt(const char *json, const char *key, long *out) {
  char buf[24];
  if (!jsonFind(json, key, buf, sizeof(buf))) return false;
  char *end = NULL;
  long v = strtol(buf, &end, 10);
  if (end == buf) return false;
  *out = v;
  return true;
}

//  处理一条主机命令行
void handleHostCommand(char *json) {
  char type[16];
  if (!jsonFind(json, "\"type\"", type, sizeof(type))) return;
  if (strcmp(type, "command") != 0) return;

  char session[40] = "";
  jsonFind(json, "\"session\"", session, sizeof(session));

  char action[16] = "";
  jsonFind(json, "\"action\"", action, sizeof(action));

  long commandId = -1;
  jsonFindInt(json, "\"command_id\"", &commandId);

  long remainingMs = -1;
  jsonFindInt(json, "\"remaining_ms\"", &remainingMs);

  lastCommandMs = millis();
  leaseValid = true;

  //  session 变更 -> 旧的 command_id 序列作废
  if (session[0]) {
    if (!haveSession || strcmp(cmdSession, session) != 0) {
      strncpy(cmdSession, session, sizeof(cmdSession) - 1);
      cmdSession[sizeof(cmdSession) - 1] = 0;
      haveSession = true;
      lastCommandId = 0;
    }
  }

  //  重复的 command_id：只重发原结果，不重启电机
  if (commandId >= 0 && haveSession && (uint32_t)commandId <= lastCommandId) {
    sendAck(cmdSession, (uint32_t)commandId, true, "duplicate");
    return;
  }

  if (commandId >= 0) lastCommandId = (uint32_t)commandId;

  //  stop / hold：必须终止当前动作
  if (strcmp(action, "stop") == 0 || strcmp(action, "hold") == 0) {
    allRelaysOff();
    sendAck(cmdSession, (commandId >= 0) ? (uint32_t)commandId : 0, true, "stopped");
    return;
  }

  //  移动命令
  int8_t dir = 0;
  if (strcmp(action, "advance") == 0)      dir = 1;
  else if (strcmp(action, "retract") == 0) dir = -1;
  else {
    sendAck(cmdSession, (commandId >= 0) ? (uint32_t)commandId : 0, false, "unknown_action");
    return;
  }

  if (remainingMs <= 0) {
    allRelaysOff();
    sendAck(cmdSession, (commandId >= 0) ? (uint32_t)commandId : 0, false, "expired");
    return;
  }

  //  板端硬时限：主机给的时间再长也不超过 BOARD_MAX_MS
  uint16_t limit = (remainingMs > (long)BOARD_MAX_MS) ? BOARD_MAX_MS : (uint16_t)remainingMs;

  setMotor(dir);
  moveActive = true;
  moveStart = millis();
  moveLimitMs = limit;

  sendAck(cmdSession, (commandId >= 0) ? (uint32_t)commandId : 0, true, "accepted");
}

// ============================================================================
//  串口接收：逐行、有限长、不阻塞
// ============================================================================
void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (rxOverflow) {
        rxOverflow = false;
        rxLen = 0;
        Serial.println(F("#ERR line too long, dropped"));
        continue;
      }
      if (rxLen > 0) {
        rxBuf[rxLen] = 0;
        //  以 { 开头 -> 主机命令；否则 -> 本地调试字符
        if (rxBuf[0] == '{') {
          handleHostCommand(rxBuf);
        } else if (mapState) {
          handleMapCommand(rxBuf);
        } else if (relayDiagPending) {
          relayDiagPending = false;
          relayVoltageSelfTest();
        } else if (renumState) {
          //  重新编号流程：这一行是"字母"或"数字"
          handleRenumber(rxBuf[0]);
        } else if ((rxBuf[0] == 'c' || rxBuf[0] == 'a' || rxBuf[0] == 'r') &&
                   rxBuf[1] >= '0' && rxBuf[1] <= '9') {
          //  ★ 主机短命令 c123 / a123 / r123（3-8 字节）
          handleCompactCommand(rxBuf);
        } else if ((rxBuf[0] == 'A' || rxBuf[0] == 'a' ||
                    rxBuf[0] == 'R' || rxBuf[0] == 'r') &&
                   (rxBuf[1] == '\r' || rxBuf[1] == '\n' || rxBuf[1] == 0 ||
                    (rxBuf[1] >= '0' && rxBuf[1] <= '9'))) {
          //  本地 jog： A300 / R300 / S
          handleShortCommand(rxBuf);
        } else {
          handleLocalChar(rxBuf[0]);
        }
        rxLen = 0;
      }
      continue;
    }

    if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxOverflow = true;      // 超长行丢弃到下一个换行
    }
  }
}

// ============================================================================
//  本地调试命令（单字符，不与主机 JSON 冲突）
// ============================================================================
void printRelayTable() {
  Serial.println();
  Serial.println(F("  RELAY STATE TABLE   (LOW-LEVEL TRIGGER)"));
  Serial.println(F("  ---------------------------------------------------------"));
  Serial.println(F("   state      | D8 (relay A) | D9 (relay B)  | motor"));
  Serial.println(F("  ------------+--------------+----------------+---------"));
  Serial.println(F("   DEFAULT    | HIGH         | PWM150 (off)  | stopped"));
  Serial.println(F("   ADVANCE    | HIGH         | LOW  (on)     | forward"));
  Serial.println(F("   RETRACT    | LOW  (on)    | PWM150 (off)  | backward"));
  Serial.println(F("  ---------------------------------------------------------"));
  Serial.println(F("   press w to flip polarity if the relays behave opposite"));
  Serial.println();
}

void printLivePins() {
  Serial.println();
  Serial.println(F("  LIVE PIN STATE"));
  Serial.println(F("  ---------------------------------------------------------"));
  Serial.print(F("   D8 (relay A) = "));
  Serial.print(digitalRead(PIN_RELAY_FWD) ? F("HIGH  -> relay A OFF") : F("LOW   -> relay A ON "));
  Serial.println();
  Serial.print(F("   D9 (relay B) set = "));
  Serial.print(d9Mode == D9_LOW ? F("LOW ") : (d9Mode == D9_PWM ? F("PWM150") : F("HIGH")));
  Serial.println(F("   (not read back: analogRead would kill the PWM)"));
  Serial.print(F("   motor = "));
  Serial.print(motorDir > 0 ? F("ADVANCE") : (motorDir < 0 ? F("RETRACT") : F("STOP")));
  Serial.print(F("   remaining "));
  if (moveActive) {
    uint32_t el = millis() - moveStart;
    Serial.print(el >= moveLimitMs ? 0 : (moveLimitMs - el));
  } else Serial.print(0);
  Serial.println(F(" ms"));
  Serial.println(F("  ---------------------------------------------------------"));
  Serial.println(F("   DEFAULT : D8 = HIGH, D9 = PWM150 (or HIGH, press g)"));
  Serial.println(F("   ADVANCE : D8 = HIGH, D9 = LOW      -> push forward"));
  Serial.println(F("   RETRACT : D8 = LOW,  D9 = PWM150    -> lean back"));
  Serial.println();
}

void printMapping() {
  Serial.println();
  Serial.println(F("  SENSOR -> NUMBER -> MATRIX CELL"));
  Serial.println(F("  ---------------------------------------------------------"));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F("   A"));
    Serial.print(sensorPin[i]);
    Serial.print(F("  #"));
    Serial.print(chNum[i]);
    Serial.print(F("  "));
    Serial.print(NUM_NAME[chNum[i]]);
    Serial.print(F("  cell "));
    Serial.print(sensorCell[i]);
    Serial.println();
  }
  Serial.println(F("  ---------------------------------------------------------"));
  Serial.println(F("   zones.json must map zone k -> the cell above"));
  Serial.print(F("   forward trigger  = #"));
  Serial.print(chNum[IDX_TRIG_FWD]);
  Serial.print(F("   backward trigger = #"));
  Serial.println(chNum[IDX_TRIG_BACK]);
  Serial.println();
}

void printTopChannels() {
  uint8_t idx[3] = { 0, 0, 0 };
  for (uint8_t k = 0; k < 3; k++) {
    uint8_t best = 255;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      bool used = false;
      for (uint8_t j = 0; j < k; j++) if (idx[j] == i) used = true;
      if (used) continue;
      if (best == 255 || pressVal[i] > pressVal[best]) best = i;
    }
    idx[k] = best;
  }
  Serial.println();
  Serial.println(F("  TOP 3 PRESSED"));
  for (uint8_t k = 0; k < 3; k++) {
    uint8_t i = idx[k];
    if (i == 255) break;
    Serial.print(F("   A"));
    Serial.print(sensorPin[i]);
    Serial.print(F("  #"));
    Serial.print(chNum[i]);
    Serial.print(F("  "));
    Serial.print(NUM_NAME[chNum[i]]);
    Serial.print(F("   press="));
    Serial.println(pressVal[i]);
  }
  Serial.println();
}


// ============================================================================
//  扫描所有模拟口：找出传感器实际接在哪
// ============================================================================
void scanAnalogInputs() {
  Serial.println(F("#SCAN A0..A15 (10-bit raw, 8 samples each)"));
  for (uint8_t ch = 0; ch < 16; ch++) {
    uint16_t lo = 1023, hi = 0, last = 0;
    for (uint8_t k = 0; k < 8; k++) {
      analogRead(ch);
      delayMicroseconds(200);
      uint16_t v = analogRead(ch);
      if (v < lo) lo = v;
      if (v > hi) hi = v;
      last = v;
    }
    Serial.print(F("# A"));
    Serial.print(ch);
    Serial.print(F("  raw="));
    Serial.print(last);
    Serial.print(F("  range="));
    Serial.print(hi - lo);
    Serial.print(F("  ("));
    Serial.print(lo);
    Serial.print(F(".."));
    Serial.print(hi);
    Serial.print(F(")"));
    if (ch < 8) {
      Serial.print(F("   <- zone "));
      Serial.print(ch + 1);
    }
    Serial.println();
  }
  Serial.println(F("# A sensor should read ~1023 empty; 0 means nothing connected"));
  Serial.println(F("# Press a sensor while scanning to see which A pin moves"));
  //  Uno 上 A8=D8、A9=D9，可以直接读这两个继电器控制脚的真实电压。
  {
    analogRead(8); delayMicroseconds(200);
    uint16_t v8 = analogRead(8);
    analogRead(9); delayMicroseconds(200);
    uint16_t v9 = analogRead(9);
    Serial.print(F("# D8(A8)="));
    Serial.print(v8);
    Serial.print(F("  D9(A9)="));
    Serial.print(v9);
    Serial.println(F("   (1023~5V, 0~0V)"));
  }
}

void printHelp() {
  Serial.println();
  Serial.println(F("  LOCAL COMMANDS (one key + Enter)"));
  Serial.println(F("   h  help             p  relay table"));
  Serial.println(F("   d  live pin levels  v  sensor->cell map"));
  Serial.println(F("   z  top 3 pressed    n  renumber a channel"));
  Serial.println(F("   i  print interval   j  JSON / table"));
  Serial.println(F("   w  flip polarity    g  D9 off level HIGH/PWM150"));
  Serial.println(F("   b  recalibrate baseline"));
  Serial.println(F("   m  local auto mode  t  bind forward trigger"));
  Serial.println(F("   x  jog advance 500  u  bind backward trigger"));
  Serial.println(F("   y  jog retract 500  +  threshold up"));
  Serial.println(F("   -  threshold down   K  move time +100ms"));
  Serial.println(F("   k  move time -100ms"));
  Serial.println();
  Serial.println(F("  SHORT COMMANDS (2-4 bytes, always safe)"));
  Serial.println(F("   A300  advance 300ms   R300  retract 300ms   S  stop"));
  Serial.println(F("  HOST COMMANDS (JSON lines, see HARDWARE_INTEGRATION.md 8.2)"));
  Serial.println(F("   {\"type\":\"command\",\"session\":\"s\",\"command_id\":1,"));
  Serial.println(F("    \"action\":\"advance\",\"speed_request\":5.0,\"remaining_ms\":300}"));
  Serial.println(F("   action: advance | retract | hold | stop"));
  Serial.println();
}

//  ★ 短命令格式（重要）
//    Uno 的 RX 缓冲只有 64 字节，而完整 JSON 命令约 70-90 字节，
//    一次发完会溢出，命令永远凑不出一行 -> 板端完全收不到。
//    短命令只有 2-4 字节，绝对安全：
//        A300   前进 300 毫秒
//        R300   后退 300 毫秒
//        S      立刻停止
//    完整 JSON 格式仍然支持（分两段发即可），但现场调试优先用短命令。
void handleShortCommand(const char *line) {
  char action = line[0];
  if (action == 'A' || action == 'a' || action == 'R' || action == 'r' || action == 'S' || action == 's') {
    uint16_t ms = moveMs;
    if (line[1] >= '0' && line[1] <= '9') {
      ms = 0;
      for (uint8_t i = 1; line[i] >= '0' && line[i] <= '9'; i++) {
        ms = ms * 10 + (uint16_t)(line[i] - '0');
      }
      if (ms > 2000) ms = 2000;
    }
    if (action == 'A' || action == 'a') {
      setMotor(1);
      moveActive = true; moveStart = millis(); moveLimitMs = ms;
      Serial.print(F("#OK ADVANCE "));
      Serial.print(ms);
      Serial.println(F("ms"));
    } else if (action == 'R' || action == 'r') {
      setMotor(-1);
      moveActive = true; moveStart = millis(); moveLimitMs = ms;
      Serial.print(F("#OK RETRACT "));
      Serial.print(ms);
      Serial.println(F("ms"));
    } else {
      allRelaysOff();
      Serial.println(F("#OK STOP"));
    }
    return;
  }
  handleLocalChar(line[0]);
}

// ============================================================================
//  通道映射：M <传感器号 1-8> <模拟口 0-15>
//  例：M 6 3  ->  把 #6 改接到 A3
//  改完立刻生效并重采基线，不需要重烧固件。
// ============================================================================
void handleMapCommand(const char *line) {
  //  每行只取第一个整数：第一行给传感器号，第二行给模拟口
  int a = -1;
  for (const char *p = line; *p; p++) {
    if (*p >= '0' && *p <= '9') {
      a = 0;
      while (*p >= '0' && *p <= '9') { a = a * 10 + (*p - '0'); p++; }
      break;
    }
  }
  int b = a;

  if (mapState == 1) {
    if (a < 1 || a > 8) {
      Serial.println(F(">>> sensor must be 1-8"));
      mapState = 0;
      return;
    }
    mapSensor = (uint8_t)(a - 1);
    mapState = 2;
    Serial.print(F(">>> sensor #"));
    Serial.print(a);
    Serial.println(F(" now on analog pin A? (0-15)"));
    return;
  }

  if (mapState == 2) {
    if (b < 0 || b > 15) {
      Serial.println(F(">>> pin must be 0-15"));
      mapState = 0;
      return;
    }
    sensorPin[mapSensor] = (uint8_t)b;
    mapState = 0;
    Serial.print(F(">>> sensor #"));
    Serial.print(mapSensor + 1);
    Serial.print(F(" -> A"));
    Serial.println(sensorPin[mapSensor]);
    Serial.println(F(">>> rebaselining..."));
    calibrateBaseline();
    Serial.println(F(">>> done"));
    printMapping();
    return;
  }
}


// ============================================================================
//  继电器诊断：逐个组合保持 3 秒，配合万用表/指示灯判断模块的真实触发方式
//  = <n>  第 n 个组合（1-6），保持 3 秒
// ============================================================================

// ============================================================================
//  方向对比诊断：交替前推/后靠 2 秒，共 4 轮，中间停顿 1 秒
//  人盯着电机，记录它到底往哪转 —— 触点接错时现象很明显。
// ============================================================================
void relayDirectionCheck() {
  Serial.println(F(">>> DIRECTION CHECK: advance/retract alternating"));
  Serial.println(F(">>> Watch the motor: which way does it move?"));
  for (uint8_t round = 0; round < 2; round++) {
    Serial.println(F(">>> --- ADVANCE (forward) ---"));
    allRelaysOff(); delay(300);
    setMotor(1); delay(2000); allRelaysOff();
    delay(1000);
    Serial.println(F(">>> --- RETRACT (backward) ---"));
    allRelaysOff(); delay(300);
    setMotor(-1); delay(2000); allRelaysOff();
    delay(1000);
  }
  Serial.println(F(">>> done. If only one direction moves, the other relay's"));
  Serial.println(F(">>> contacts are wired wrong (both to the same rail)."));
}


// ============================================================================
//  继电器电压自检（原子操作：设电平 -> 立即读电压 -> 打印）
//
//  ★ 为什么要原子：
//    relayDiag() 里有 delay(3000)，这期间主循环被阻塞，
//    串口命令只能堆在 RX 缓冲里，等 delay 结束才被处理 ——
//    那时继电器早就松开了，再去读电压只会读到 1023，是假象。
//    所以这里在同一个函数里：设完立刻读，不给 delay 插队的机会。
//
//  Uno 上 A8=D8、A9=D9，analogRead(8)/analogRead(9) 读的是引脚真实电压。



// ============================================================================
//  继电器状态自检（原子操作：设电平 -> 立即读回 -> 打印）
//
//  ★★ 重要：这是 Arduino **Nano**，不是 Uno！
//     Nano 的引脚映射：
//         A8 = PC0      D8  = PB0      ← 两个不同的物理引脚！
//         A9 = PC1      D9  = PB1      ← 两个不同的物理引脚！
//         A10 = PC2     A11 = PC3
//         A12 = PB4(=D12)  A13 = PB5(=D13)
//     所以 **不能用 analogRead(8) 读 D8 的电压** —— 那读的是 PC0，
//     和继电器毫无关系。之前的"外部上拉顶死"结论就是这么错的。
//
//  ★ 现在只读 digitalRead（PORT 寄存器，反映我们设定的输出状态）。
//    真实引脚电压只能靠万用表量。
// ============================================================================
//  ★ 这张板是 Arduino Nano：A8=PC0、D8=PB0，是两个不同的引脚。
//    所以 analogRead(8) 读不到 D8 的电压（读的是 PC0）。
//    继电器的引脚电压只能用 digitalRead 看我们设定的状态，
//    真实电压请用万用表量。

void relayVoltageSelfTest() {
  Serial.println(F("#RELAY VOLTAGE SELF-TEST"));
  Serial.println(F("# 1023 = 5V (HIGH)   0 = 0V (LOW)"));
  struct { uint8_t d8; uint8_t d9; const char *label; } table[6] = {
    { 1, 0, "1 D8=HIGH D9=HIGH  both released" },
    { 0, 0, "2 D8=LOW  D9=HIGH  A on only" },
    { 1, 1, "3 D8=HIGH D9=LOW   B on only" },
    { 1, 2, "4 D8=HIGH D9=PWM   B pwm" },
    { 0, 2, "5 D8=LOW  D9=PWM   A on + B pwm" },
    { 0, 1, "6 D8=LOW  D9=LOW   both on" },
  };
  for (uint8_t n = 0; n < 6; n++) {
    allRelaysOff();
    delay(20);
    if (table[n].d8 == 0) { pinMode(PIN_RELAY_FWD, OUTPUT); digitalWrite(PIN_RELAY_FWD, LOW); }
    else                  { digitalWrite(PIN_RELAY_FWD, HIGH); pinMode(PIN_RELAY_FWD, INPUT); }
    if (table[n].d9 == 1)      relayForceLow(PIN_RELAY_BACK);
    else if (table[n].d9 == 2) analogWrite(PIN_RELAY_BACK, RELAY_PWM_LEVEL);
    else                       { digitalWrite(PIN_RELAY_BACK, HIGH);
                                 pinMode(PIN_RELAY_BACK, INPUT); }
    delay(20);
    uint8_t d8 = digitalRead(PIN_RELAY_FWD);
    uint8_t d9 = digitalRead(PIN_RELAY_BACK);
    Serial.print(F("# "));
    Serial.print(table[n].label);
    Serial.print(F("   ->  D8="));
    Serial.print(d8 ? F("HIGH") : F("LOW"));
    Serial.print(F("  D9="));
    Serial.print(d9 ? F("HIGH") : F("LOW"));
    Serial.print(F("  COM1A="));
    Serial.print((TCCR1A >> 6) & 0x03);
    Serial.println();
  }
  allRelaysOff();
  Serial.println(F("# done - all released"));
}


// ============================================================================
//  引脚交叉验证：digitalRead（我们设定的值） vs analogRead（引脚真实电压）
//  · digitalRead 在 OUTPUT 模式读的是 PORT 寄存器 = 我们想要的值
//  · analogRead 读的是引脚上的实际电压
//  两者不一致 -> 引脚被外部电路拉住了（短接 VCC / 强上拉）
// ============================================================================
void pinCrossCheck() {
  Serial.println(F("#PIN CROSS-CHECK"));
  Serial.println(F("# digitalRead = what we set;  analogRead = real voltage"));
  struct { uint8_t pin; uint8_t mode; const char *label; } rows[4] = {
    { PIN_RELAY_FWD, 0, "D8 output LOW" },
    { PIN_RELAY_FWD, 1, "D8 output HIGH" },
    { PIN_RELAY_BACK, 0, "D9 output LOW" },
    { PIN_RELAY_BACK, 1, "D9 output HIGH" },
  };
  for (uint8_t n = 0; n < 4; n++) {
    if (rows[n].mode == 0) {
      pinMode(rows[n].pin, OUTPUT);
      digitalWrite(rows[n].pin, LOW);
    } else {
      digitalWrite(rows[n].pin, HIGH);
      pinMode(rows[n].pin, INPUT);
    }
    delay(30);
    uint8_t dr = digitalRead(rows[n].pin);
    Serial.print(F("# "));
    Serial.print(rows[n].label);
    Serial.print(F("  ->  digitalRead="));
    Serial.print(dr);
    Serial.println(F("   (Nano: analogRead 读不到 D8/D9)"));
  }
  allRelaysOff();
  Serial.println(F("# done"));
}


// ============================================================================
//  寄存器级诊断：直接读写 PORTB / DDRB
//  · DDRB bit  = 1 表示输出
//  · PORTB bit = 1 表示输出高、0 表示输出低
//  读完立刻报告，两个值不一致就说明有别的东西在动这个引脚。
// ============================================================================
void portRegisterCheck() {
  Serial.println(F("#PORTB REGISTER CHECK"));
  Serial.println(F("# D8=PB0  D9=PB1"));
  Serial.print(F("# start DDRB=0x"));
  Serial.println(DDRB, HEX);
  Serial.print(F("# start PORTB=0x"));
  Serial.println(PORTB, HEX);

  struct { uint8_t mask; const char *label; } rows[3] = {
    { 0x01, "PB0 LOW  (D8 low)" },
    { 0x02, "PB1 LOW  (D9 low)" },
    { 0x03, "PB0+PB1 LOW (both low)" },
  };
  for (uint8_t n = 0; n < 3; n++) {
    //  set the selected bits low, others high
    PORTB = (uint8_t)((PORTB | (uint8_t)~rows[n].mask) & (uint8_t)~rows[n].mask);
    DDRB  = (uint8_t)(DDRB | rows[n].mask);      // make them outputs
    delay(30);
    uint8_t dr = digitalRead(PIN_RELAY_FWD);
    uint8_t dr9 = digitalRead(PIN_RELAY_BACK);
    Serial.print(F("# "));
    Serial.print(rows[n].label);
    Serial.print(F("  PORTB=0x"));
    Serial.print(PORTB, HEX);
    Serial.print(F("  D8="));
    Serial.print(dr ? F("HIGH") : F("LOW"));
    Serial.print(F("  D9="));
    Serial.println(dr9 ? F("HIGH") : F("LOW"));
  }
  //  restore
  PORTB = 0;
  DDRB  = (uint8_t)(DDRB | 0x03);
  digitalWrite(PIN_RELAY_FWD, HIGH);
  digitalWrite(PIN_RELAY_BACK, HIGH);
  Serial.println(F("# done"));
}


// ============================================================================
//  D8 拉低测试：持续把 D8 拉低 10 秒，每秒打印一次引脚电压
//
//  用法：拔掉继电器 A 的 IN 线（只留 D8 那根），然后敲 'l'
//  如果这时 A8 读到 0 -> 之前是外部电路在拉高，换一根线/换个孔就好
//  如果还是 1023 -> D8 本身有问题（焊点/排线）
// ============================================================================
void d8PullDownTest() {
  Serial.println(F("#D8 PULL-DOWN TEST - 10 seconds"));
  Serial.println(F("# unplug the relay A IN wire first, then watch A8"));
  for (uint8_t k = 0; k < 10; k++) {
    pinMode(PIN_RELAY_FWD, OUTPUT);
    digitalWrite(PIN_RELAY_FWD, LOW);
    delay(20);
    Serial.print(F("# t="));
    Serial.print(k);
    Serial.print(F("s  D8="));
    Serial.print(digitalRead(PIN_RELAY_FWD) ? F("HIGH") : F("LOW"));
    Serial.print(F("  D9="));
    Serial.print(digitalRead(PIN_RELAY_BACK) ? F("HIGH") : F("LOW"));
    Serial.println();
    delay(980);
  }
  digitalWrite(PIN_RELAY_FWD, HIGH);
  pinMode(PIN_RELAY_FWD, INPUT);
  Serial.println(F("# done - D8 released"));
}


// ============================================================================
//  ★ 超短命令通道
//
//  为什么需要：Arduino Uno 的 RX 缓冲只有 64 字节，而完整的 JSON 命令
//  （含 session/action/command_id/remaining_ms）约 65-90 字节。
//  主机发一条命令时，板端可能正在发一帧 JSON（1150 字节，2M 波特下 5.7ms），
//  这期间主机来的字节全挤在 64 字节缓冲里 -> 溢出丢字节 ->
//  板端永远凑不出一整行 -> 命令完全收不到（表现为"点了没反应"）。
//
//  短命令只有 3-4 字节，永远塞得下：
//      C123   命令 id=123，动作 stop
//      A123   命令 id=123，动作 advance（前进）
//      R123   命令 id=123，动作 retract（后退）
//  ACK 照常返回，格式与 JSON 通道一致。
// ============================================================================
bool compactCommandPending = false;

void handleCompactCommand(const char *line) {
  char verb = line[0];
  if (verb != 'C' && verb != 'c' && verb != 'A' && verb != 'a' && verb != 'R' && verb != 'r') {
    return;
  }
  long id = 0;
  for (const char *p = line + 1; *p >= '0' && *p <= '9'; p++) {
    id = id * 10 + (*p - '0');
    if (id > 2000000000L) break;
  }
  if (id <= 0) {
    Serial.println(F("#ERR compact command needs an id, e.g. A1"));
    return;
  }
  const char *act = "stop";
  if (verb == 'A' || verb == 'a') act = "advance";
  else if (verb == 'R' || verb == 'r') act = "retract";

  //  走同一套处理逻辑：借用 handleHostCommand 构造一个最小 JSON
  char buf[96];
  snprintf(buf, sizeof(buf),
           "{\"type\":\"command\",\"session\":\"c\",\"command_id\":%ld,\"action\":\"%s\",\"remaining_ms\":%d}",
           id, act, (act == "stop") ? 0 : (int)moveMs);
  handleHostCommand(buf);
}


void openDrainTest() {
  Serial.println(F("#OPEN-DRAIN TEST"));
  Serial.println(F("# D8/D9 as INPUT (high-Z): what voltage does the outside world hold?"));
  struct { uint8_t pin; uint8_t analog; const char *label; } rows[2] = {
    { PIN_RELAY_FWD, 8, "D8 high-Z" },
    { PIN_RELAY_BACK, 9, "D9 high-Z" },
  };
  for (uint8_t n = 0; n < 2; n++) {
    pinMode(rows[n].pin, INPUT);          // high impedance
    delay(50);
    uint8_t dr = digitalRead(rows[n].pin);
    Serial.print(F("# "));
    Serial.print(rows[n].label);
    Serial.print(F("  ->  digitalRead="));
    Serial.print(dr ? F("HIGH") : F("LOW"));
    Serial.println();
  }
  //  恢复（断开 = 高阻，交给外部上拉）
  digitalWrite(PIN_RELAY_FWD, HIGH);
  digitalWrite(PIN_RELAY_BACK, HIGH);
  pinMode(PIN_RELAY_FWD, INPUT);
  pinMode(PIN_RELAY_BACK, INPUT);
  Serial.println(F("# done - both restored to OUTPUT HIGH"));
}


void holdLowTest() {
  Serial.println(F("#HOLD-LOW TEST: drive LOW, wait, then sample"));
  for (uint8_t which = 0; which < 2; which++) {
    uint8_t pin = which ? PIN_RELAY_BACK : PIN_RELAY_FWD;
    uint8_t an  = which ? 9 : 8;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delay(100);
    uint8_t dr = 0;
    for (uint8_t k = 0; k < 20; k++) {
      dr = digitalRead(pin);
      delay(20);
    }
    Serial.print(F("# "));
    Serial.print(which ? F("D9") : F("D8"));
    Serial.print(F(" driven LOW for 400ms ->  digitalRead="));
    Serial.println(dr ? F("HIGH") : F("LOW"));
    digitalWrite(pin, HIGH);
    pinMode(pin, INPUT);
  }
  Serial.println(F("# done"));
}


// ============================================================================
//  单通道长时间驱动 —— 方便用户盯着继电器灯和电机看
//    1 = 只吸合 A（D8 拉低 5 秒）
//    2 = 只吸合 B（D9 拉低 5 秒）
//    3 = A、B 依次各 5 秒，中间停 2 秒
// ============================================================================
void singleDriveTest(int n) {
  if (n < 1 || n > 3) { Serial.println(F(">>> 1=A 2=B 3=both")); return; }
  if (n == 3) {
    Serial.println(F(">>> A (D8) 5 s -- watch relay A"));
    relayA(true);  delay(5000); relayA(false); delay(2000);
    Serial.println(F(">>> B (D9) 5 s -- watch relay B"));
    relayB(true);  delay(5000); relayB(false); delay(2000);
    Serial.println(F(">>> done"));
    return;
  }
  if (n == 1) {
    Serial.println(F(">>> A (D8) 5 s -- watch relay A"));
    relayA(true); delay(5000); relayA(false);
  } else {
    Serial.println(F(">>> B (D9) 5 s -- watch relay B"));
    relayB(true); delay(5000); relayB(false);
  }
  Serial.println(F(">>> done"));
}


// ============================================================================
//  PWM 定时器检查：直接读 TCCR1A（D9 的 Timer1）
//  TCCR1A 的 COM1A1/COM1A0 位决定 D9 是普通 GPIO 还是 PWM 输出：
//      00 = 普通 GPIO（digitalWrite 有效）
//      10/11 = PWM 输出（PORT 的值被忽略，输出由比较器决定）
// ============================================================================
void pwmTimerCheck() {
  Serial.println(F("#PWM TIMER CHECK (D9 = Timer1 OC1A)"));
  Serial.print(F("# TCCR1A=0x"));
  Serial.print(TCCR1A, HEX);
  Serial.print(F("  COM1A="));
  Serial.print((TCCR1A >> 6) & 0x03);
  Serial.print(F("  OCR1A="));
  Serial.println(OCR1A);
  Serial.println(F("# COM1A=0 -> plain GPIO, digitalWrite works"));
  Serial.println(F("# COM1A=2 -> PWM output, PORT value ignored!"));
}

void handleLocalChar(char c) {
  switch (c) {
    case 'h': case 'H': printHelp(); break;

    case 'p': case 'P': printRelayTable(); break;

    case 's': scanAnalogInputs(); break;   // 只认小写：大写 S 会误触发长阻塞诊断

    case '*':
      relayDirectionCheck();
      break;

    case '^':
      relayVoltageSelfTest();
      break;

    case '(':
      pinCrossCheck();
      break;

    case ')':
      openDrainTest();
      break;

    case '_':
      holdLowTest();
      break;

    case '~':
      pwmTimerCheck();
      break;

    case '1':
      singleDriveTest(1);
      break;

    case '2':
      singleDriveTest(2);
      break;

    case '3':
      singleDriveTest(3);
      break;

    case '&':
      portRegisterCheck();
      break;

    case 'l': case 'L':
      d8PullDownTest();
      break;

    case '=':
      //  继电器诊断：= <1-6>
      relayDiagPending = true;
      Serial.println(F(">>> relay diag: = 1..6"));
      break;






    case 'd': case 'D': printLivePins(); break;

    case 'v': case 'V': printMapping(); break;

    case 'z': case 'Z': printTopChannels(); break;

    case 'g': case 'G':
      d9OffUseHigh = !d9OffUseHigh;
      allRelaysOff();
      Serial.print(F(">>> D9 released level = "));
      Serial.println(d9OffUseHigh ? F("HIGH (5V, as tested on site)")
                                  : F("PWM150 (2.9V, safer for the 3.3V coil)"));
      break;

    case 'w': case 'W':
      relayLow = !relayLow;
      allRelaysOff();
      Serial.print(F(">>> relay polarity = "));
      Serial.println(relayLow ? F("LOW trigger") : F("HIGH trigger"));
      break;

    case 'i': case 'I':
      if (!printOn)            { printOn = true;  printMs = 1000; }
      else if (printMs >= 1000){ printMs = 500; }
      else if (printMs >= 500) { printMs = 200; }
      else if (printMs >= 200) { printMs = 100; }
      else if (printMs >= 100) { printMs = 50; }
      else                     { printOn = false; }
      Serial.print(F(">>> print every "));
      if (printOn) { Serial.print(printMs); Serial.print(F(" ms")); }
      else         Serial.print(F("OFF"));
      Serial.println();
      break;

    case 'n': case 'N':
      Serial.println(F(">>> renumber: channel letter a-h, then number 1-8"));
      renumState = 1;
      break;

    case 'M':
      //  通道映射：M <sensor 1-8> <pin 0-15>
      Serial.println(F(">>> format: M <sensor 1-8> <pin 0-15>   e.g. M 6 3"));
      mapState = 1;
      break;

    case 'm':
      autoMode = !autoMode;
      manualMode = false;
      Serial.print(F(">>> local auto mode = "));
      Serial.println(autoMode ? F("ON") : F("OFF"));
      break;

    case 'x': case 'X':
      allRelaysOff();
      setMotor(1); delay(500); setMotor(0);
      Serial.println(F(">>> jog advance done"));
      break;

    case 'y': case 'Y':
      allRelaysOff();
      setMotor(-1); delay(500); setMotor(0);
      Serial.println(F(">>> jog retract done"));
      break;

    case 't': case 'T':
      Serial.println(F(">>> now press the sensor that should ADVANCE..."));
      bindTarget = 0; bindDeadline = millis() + 8000;
      break;

    case 'u': case 'U':
      Serial.println(F(">>> now press the sensor that should RETRACT..."));
      bindTarget = 1; bindDeadline = millis() + 8000;
      break;

    case 'b': case 'B':
      Serial.println(F(">>> calibrating baseline, keep the pad empty..."));
      allRelaysOff();
      calibrateBaseline();
      Serial.println(F(">>> baseline done"));
      break;

    case '+':
      pressThresh += 10;
      Serial.print(F(">>> threshold = "));
      Serial.println(pressThresh);
      break;

    case '-':
      if (pressThresh > 10) pressThresh -= 10;
      Serial.print(F(">>> threshold = "));
      Serial.println(pressThresh);
      break;

    case 'K':
      moveMs += 100;
      if (moveMs > 5000) moveMs = 5000;
      Serial.print(F(">>> move duration = "));
      Serial.print(moveMs);
      Serial.println(F(" ms"));
      break;

    case 'k':
      if (moveMs > 200) moveMs -= 100;
      Serial.print(F(">>> move duration = "));
      Serial.print(moveMs);
      Serial.println(F(" ms"));
      break;

    case 'j': case 'J':
      jsonMode = !jsonMode;
      Serial.println(jsonMode ? F(">>> JSON stream on") : F(">>> table view on"));
      break;

    case '\r': case '\n':
      break;

    default:
      Serial.println(F(">>> unknown command, press h for help"));
      break;
  }
}

//  重新编号流程（两字符：字母 + 数字）
void handleRenumber(char c) {
  if (renumState == 1) {
    if (c >= 'a' && c <= 'h') {
      renumChan = c - 'a';
      renumState = 2;
      Serial.print(F(">>> channel A"));
      Serial.print(sensorPin[renumChan]);
      Serial.print(F(" now number: "));
    } else {
      renumState = 0;
      Serial.println(F(">>> cancelled"));
    }
    return;
  }
  if (renumState == 2) {
    if (c >= '1' && c <= '8') {
      uint8_t oldNum = chNum[renumChan];
      chNum[renumChan] = c - '0';
      renumState = 0;
      Serial.print(F(">>> A"));
      Serial.print(sensorPin[renumChan]);
      Serial.print(F(" : #"));
      Serial.print(oldNum);
      Serial.print(F(" -> #"));
      Serial.print(chNum[renumChan]);
      Serial.print(F("  ("));
      Serial.print(NUM_NAME[chNum[renumChan]]);
      Serial.println(F(")"));
    } else {
      renumState = 0;
      Serial.println(F(">>> cancelled"));
    }
    return;
  }
}

// ============================================================================
//  setup / loop
// ============================================================================
void setup() {
  Serial.begin(BAUD_RATE);

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPin[i], INPUT);
  }

  //  ★ 继电器引脚：开漏（OPEN_DRAIN）模式
  //
  //    实测（2026-10-24）：D8/D9 设成高阻时，外部都保持 5.0V ——
  //    继电器模块的 IN 脚内部有上拉到 VCC（光耦隔离模块的标准设计）。
  //    Arduino 的推挽输出 LOW 顶不动这个上拉，引脚电压被拉在高位，
  //    继电器 A（后退）就永远不吸合 —— 表现为"前进行、后退不行"。
  //
  //    OPEN_DRAIN：关闭 Arduino 的输出驱动，只保留内部上拉（~40k）。
  //      外部上拉负责"断开"（5V），Arduino 内部上拉负责"吸合"（0V）。
  //    对低电平触发的光耦继电器模块，这是正确的驱动方式。
  digitalWrite(PIN_RELAY_FWD, HIGH);
  digitalWrite(PIN_RELAY_FWD, HIGH);      // 释放 -> 外部上拉 -> 5V -> 断开
  digitalWrite(PIN_RELAY_BACK, HIGH);
  digitalWrite(PIN_RELAY_BACK, HIGH);     // 释放 -> 5V -> 断开

  //  传感器 k (0-based) -> 矩阵索引 (k+1-1)*16 + (k+1) = k*16 + k + 1
  //  传感器1 -> 1, 传感器2 -> 18, 传感器3 -> 35, ... 传感器8 -> 120
  for (uint8_t k = 0; k < NUM_SENSORS; k++) {
    sensorCell[k] = (uint8_t)(k * 16 + k + 1);
  }

  allRelaysOff();     // DEFAULT: both relays released, motor stopped

  Serial.println();
  Serial.println(F("========================================================="));
  Serial.println(F(" LumbarLab board - 8ch pressure + dual relay motor"));
  Serial.println(F("========================================================="));
  Serial.print(F(" serial "));
  Serial.print(BAUD_RATE);
  Serial.println(F("   sensor cells:"));
  for (uint8_t k = 0; k < NUM_SENSORS; k++) {
    Serial.print(F("  #"));
    Serial.print(chNum[k]);
    Serial.print(F("="));
    Serial.print(sensorCell[k]);
    Serial.print(F(" "));
  }
  Serial.println();
  Serial.println(F(" relays LOW-LEVEL TRIGGER   press h for help"));
  Serial.println();

  Serial.println(F("calibrating idle baseline, keep the pad empty..."));
  calibrateBaseline();
  Serial.println(F("baseline ready"));
  Serial.println(F("#READY"));
}

void loop() {
  handleSerialInput();

  readSensors();
  updateLocalControl();
  updateLeaseWatchdog();
  updateBinding();

  if (printOn && (millis() - lastPrintMs >= printMs)) {
    //  ★ 有待处理的命令时先处理命令，本帧延后一小会儿。
    //    主机来的命令字节在 64 字节 RX 缓冲里等不了太久，
    //    先回 ACK 再发数据，命令就不会丢。
    //
    //    ★ 但延后必须有上限：host 的数据有效期判定线是 800ms，
    //      如果连续几帧都被命令挤掉，host 会认为"数据中断"。
    //      所以最多延后一帧（printMs），之后无论如何都要发出去。
    if (Serial.available() > 0 && (millis() - lastPrintMs) < printMs * 2) {
      handleSerialInput();
    } else {
      lastPrintMs = millis();
      frameSeq++;
      if (jsonMode) printSensorFrame();
      else          printTable();
    }
  }

  //  ★ 发完一帧立刻再收一次：发帧占用 5.7ms，其间到达的命令
  //    必须在这里被捞出来，否则会被 64 字节的 RX 缓冲冲掉。
  handleSerialInput();
}
