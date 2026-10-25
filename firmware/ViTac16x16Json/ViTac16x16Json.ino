/*
 * ============================================================================
 *  3D-ViTac 16×16 压阻阵列读取 —— JSON 输出版（给队友的 Python 端喂数据）
 * ----------------------------------------------------------------------------
 *  硬件：Arduino Nano / Uno（ATmega328P）
 *        CD74HC4067 (16 路模拟多路复用器) ×1
 *        SN74HC595N (8 位移位寄存器) ×2  ← 级联出 16 路
 *        压敏布（16 根行线 + 16 根列线 = 32 根）
 *        20kΩ 电阻 ×1（分压下拉）
 *
 *  Lumbar Lab 修正版：保留原扫描/接线，修复累加、计数、帧号和 ADC 类型。
 *  三个 256 字节全局帧 + 临时 512 字节累加；还需串口缓冲和栈空间。
 *  此工作区未使用 Arduino CLI 编译或在实物上验证 SRAM/时序。
 *
 *  ⚠️ 串口波特率必须是 2000000，否则全是乱码！
 *
 *  ---------------------------------------------------------------------------
 *  【给 Python 队友的约定】
 *  · 串口每收到「一行」= 一帧 = 一个完整 JSON 对象，用 json.loads(line) 解析。
 *  · 所有提示/回执消息都以 "#" 开头，Python 端跳过以 "#" 开头的行即可。
 *  · 帧结构：
 *      {
 *        "seq":  帧序号，从 0 开始递增（int）
 *        "t":    毫秒时间戳 millis()（int，32 位会溢出但够用 49 天）
 *        "base": 1 = 已校准基线，0 = 还没校准（int）
 *        "peak": 本帧最大响应值 0~255（int，= 当前值 - 基线，负数已截成 0）
 *        "pr":   最大响应点的行号 0~15（int，无响应时 = -1）
 *        "pc":   最大响应点的列号 0~15（int，无响应时 = -1）
 *        "nz":   本帧有响应的点数（int，0~256）
 *        "sum":  本帧所有点响应值之和（int，0~65280；相对响应，未经标定不是力）
 *        "diff":[256 个 0~255 的 int]，行优先，索引 = 行*16 + 列
 *                   ← 这是「减掉基线后的相对值」，算法直接用这个
 *        "raw": [256 个 0~255 的 int]，行优先，绝对原始 ADC 值（可选）
 *      }
 *    raw 用命令 "r" 开关（默认带 raw；带宽不够就按 r 关掉，只留 diff）。
 *
 *  ---------------------------------------------------------------------------
 *  串口命令（在串口监视器输入框敲一下回车）：
 *      j  → 切换「网格显示 / 纯数字流 / JSON 流」三种输出
 *      r  → JSON 里是否附带 raw 数组
 *      b  → 重新校准空载基线（必须先让布面完全空载；不能用坐姿压力做空载基线）
 *      t  → MUX 地址 ADC 观察（不能单独判断线路好坏）
 *      h  → 打印帮助
 *
 *  接线说明见配套文档《压敏布接线与操作手册.md》
 * ============================================================================
 */

#include <Arduino.h>

// ---------------------------------------------------------------- 基本参数
#define BAUD_RATE        2000000
#define ROW_COUNT        16
#define COLUMN_COUNT     16
#define TOTAL_POINTS     (ROW_COUNT * COLUMN_COUNT)

// ---------------------------------------------------------------- 引脚定义
//  保留用户提供的原始引脚定义；本项目尚未进行实物接线验证
#define PIN_ADC_INPUT    A0    // ← CD74HC4067 的 SIG
#define PIN_SR_DATA      2     // ← 74HC595 的 SER  (14脚)
#define PIN_SR_CLOCK     3     // ← 74HC595 的 SRCLK(11脚) 和 RCLK(12脚) 都要接这根
#define PIN_MUX_S0       4     // ← CD74HC4067 的 S0  (10脚)
#define PIN_MUX_S1       5     // ← CD74HC4067 的 S1  (11脚)
#define PIN_MUX_S2       6     // ← CD74HC4067 的 S2  (14脚)
#define PIN_MUX_S3       7     // ← CD74HC4067 的 S3  (13脚)
#define PIN_MUX_EN       8     // ← CD74HC4067 的 E#  (15脚) 低电平使能

// ---------------------------------------------------------------- 输出模式
enum OutputMode { MODE_GRID = 0, MODE_STREAM = 1, MODE_JSON = 2 };

// ---------------------------------------------------------------- 缓冲区
uint8_t rawFrame[ROW_COUNT][COLUMN_COUNT];      // 本次读到的原始值
uint8_t baseFrame[ROW_COUNT][COLUMN_COUNT];     // 空载基线
uint8_t diffFrame[ROW_COUNT][COLUMN_COUNT];     // 相减后的相对变化

OutputMode mode      = MODE_JSON;   // 上电默认 JSON，直接对接队友
bool jsonRaw        = true;         // JSON 里是否附带 raw 数组
bool baselineReady  = false;
bool asciiOnly      = false;        // true = 只输出 ASCII（串口显示乱码时按 'a' 切换）

uint32_t frameSeq   = 0;  // 与 millis()/主机协议统一为 uint32

// ============================================================================
//  底层：选行（MUX 地址线）
// ============================================================================
void selectRow(uint8_t row) {
  // 先关再开，避免切换瞬间的毛刺
  digitalWrite(PIN_MUX_EN, HIGH);
  digitalWrite(PIN_MUX_S0, (row >> 0) & 1);
  digitalWrite(PIN_MUX_S1, (row >> 1) & 1);
  digitalWrite(PIN_MUX_S2, (row >> 2) & 1);
  digitalWrite(PIN_MUX_S3, (row >> 3) & 1);
  digitalWrite(PIN_MUX_EN, LOW);    // 低电平使能
}

// ============================================================================
//  底层：移位（换列）
//  这段逻辑必须和官方固件一模一样，否则列的顺序会错。
//  原理：往 595 里灌一个 1，让这个 1 在 16 路输出里循环移动，
//        每次移位就换一列被拉高。
// ============================================================================
void shiftColumn(bool isFirst) {
  if (isFirst) {
    digitalWrite(PIN_SR_DATA, HIGH);
  }
  digitalWrite(PIN_SR_CLOCK, HIGH);
  digitalWrite(PIN_SR_CLOCK, LOW);
  if (isFirst) {
    digitalWrite(PIN_SR_DATA, LOW);
  }
}

// ============================================================================
//  读一整帧 16×16
// ============================================================================
void readFrame(uint8_t out[ROW_COUNT][COLUMN_COUNT]) {
  for (uint8_t r = 0; r < ROW_COUNT; r++) {
    selectRow(r);
    shiftColumn(true);
    shiftColumn(false);
    for (uint8_t c = 0; c < COLUMN_COUNT; c++) {
      out[r][c] = (uint8_t)(analogRead(PIN_ADC_INPUT) >> 2);  // 10位 → 8位
      shiftColumn(false);
    }
  }
}

// ============================================================================
//  校准空载基线（布上没人时采 16 帧取平均）
// ============================================================================
void captureBaseline() {
  uint16_t sum[ROW_COUNT][COLUMN_COUNT] = {{0}}; // 16*255=4080；8位累加会溢出
  const uint8_t SAMPLES = 16;
  baselineReady = false;

  for (uint8_t k = 0; k < SAMPLES; k++) {
    readFrame(rawFrame);
    for (uint8_t r = 0; r < ROW_COUNT; r++) {
      for (uint8_t c = 0; c < COLUMN_COUNT; c++) {
        sum[r][c] += rawFrame[r][c];
      }
    }
  }
  for (uint8_t r = 0; r < ROW_COUNT; r++) {
    for (uint8_t c = 0; c < COLUMN_COUNT; c++) {
      baseFrame[r][c] = (uint8_t)(sum[r][c] / SAMPLES);
    }
  }
  baselineReady = true;
}

bool inJson() { return mode == MODE_JSON; }

// 回执消息：JSON 模式下加 "#" 前缀，Python 端跳过这类行
void ack(const __FlashStringHelper *msg) {
  if (inJson()) Serial.print('#');
  Serial.println(msg);
}

// ============================================================================
//  MUX 地址读取辅助检查；读数仍受布、电路和当前激励影响
//  仅观察 10位 ADC；读数相同不能单独证明接线故障
// ============================================================================
void muxSelfTest() {
  ack(F("=== MUX 自检：16 个地址依次读 A0 ==="));
  ack(F("（读数相同不能单独判故障；请结合地址线、使能和负载检查）"));

  int first = 0; // AVR int 足够容纳 10位 ADC，且差值比较不会无符号下溢
  for (uint8_t r = 0; r < ROW_COUNT; r++) {
    selectRow(r);
    delayMicroseconds(200);
    int v = analogRead(PIN_ADC_INPUT);
    if (r == 0) first = v;
    if (inJson()) Serial.print('#');
    Serial.print(F("  addr "));
    if (r < 10) Serial.print('0');
    Serial.print(r);
    Serial.print(F("  ->  A0 = "));
    if (v < 10)   Serial.print('0');
    if (v < 100)  Serial.print('0');
    Serial.print(v);
    Serial.print(F("   "));
    for (uint8_t k = 0; k < 12; k++) {
      Serial.print((v > first + 3) ? '#' : (v > first - 3 ? '.' : ' '));
    }
    Serial.println();
  }
  ack(F("  # = 比首值高   . = 接近首值   空格 = 比首值低"));
  ack(F("  仅作观察；请配合已知负载检查行列顺序与接线"));
}

// ============================================================================
//  打印网格
// ============================================================================
void printMatrix() {
  Serial.println();
  Serial.print(F("      "));
  for (uint8_t c = 0; c < COLUMN_COUNT; c++) {
    if (c < 10) Serial.print(' ');
    Serial.print(c);
  }
  Serial.println();

  for (uint8_t r = 0; r < ROW_COUNT; r++) {
    Serial.print(F("  r"));
    if (r < 10) Serial.print('0');
    Serial.print(r);
    Serial.print(F("  "));
    for (uint8_t c = 0; c < COLUMN_COUNT; c++) {
      uint8_t v = diffFrame[r][c];
      if (v < 10)   Serial.print(' ');
      if (v < 100)  Serial.print(' ');
      Serial.print(v);
      Serial.print(' ');
    }
    Serial.println();
  }
}

// ============================================================================
//  JSON 输出：一行 = 一个完整对象
//  用 Serial.print 分段拼接，不用 String，省 RAM
// ============================================================================
void printJsonFrame(int maxVal, int maxRow, int maxCol, uint16_t nonZero, uint16_t sum) {
  Serial.print(F("{\"seq\":"));
  Serial.print(frameSeq);
  Serial.print(F(",\"t\":"));
  Serial.print(millis());
  Serial.print(F(",\"base\":"));
  Serial.print(baselineReady ? 1 : 0);
  Serial.print(F(",\"peak\":"));
  Serial.print(maxVal);
  Serial.print(F(",\"pr\":"));
  Serial.print(maxRow);
  Serial.print(F(",\"pc\":"));
  Serial.print(maxCol);
  Serial.print(F(",\"nz\":"));
  Serial.print(nonZero);
  Serial.print(F(",\"sum\":"));
  Serial.print(sum);

  // diff：减基线后的相对值，算法直接用
  Serial.print(F(",\"diff\":["));
  for (uint8_t r = 0; r < ROW_COUNT; r++) {
    for (uint8_t c = 0; c < COLUMN_COUNT; c++) {
      if (r || c) Serial.print(',');
      Serial.print(diffFrame[r][c]);
    }
  }
  Serial.print(']');

  // raw：绝对原始值，可关
  if (jsonRaw) {
    Serial.print(F(",\"raw\":["));
    for (uint8_t r = 0; r < ROW_COUNT; r++) {
      for (uint8_t c = 0; c < COLUMN_COUNT; c++) {
        if (r || c) Serial.print(',');
        Serial.print(rawFrame[r][c]);
      }
    }
    Serial.print(']');
  }

  Serial.println('}');
}

// ============================================================================
//  打印帮助
// ============================================================================
void printHelp() {
  ack(F("命令： j=切换输出模式(网格/数字流/JSON)  r=JSON是否带raw  b=重新校准基线  t=MUX自检  h=帮助"));
}

// ============================================================================
//  处理串口命令
// ============================================================================
void handleCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'b' || c == 'B') {
      ack(F("正在校准基线，请确保布上无人..."));
      captureBaseline();
      ack(F("基线校准完成。"));
    }
    else if (c == 't' || c == 'T') {
      muxSelfTest();
    }
    else if (c == 'j' || c == 'J') {
      mode = (OutputMode)((mode + 1) % 3);
      if (mode == MODE_GRID)      ack(F("-> 网格显示模式"));
      else if (mode == MODE_STREAM) ack(F("-> 纯数字流模式"));
      else                         ack(F("-> JSON 模式（一行一帧，Python 可直接 json.loads）"));
    }
    else if (c == 'r' || c == 'R') {
      jsonRaw = !jsonRaw;
      if (mode == MODE_JSON) {
        ack(jsonRaw ? F("-> JSON 将附带 raw 数组") : F("-> JSON 不带 raw 数组（只留 diff，更省带宽）"));
      } else {
        ack(jsonRaw ? F("-> JSON 将附带 raw 数组") : F("-> JSON 不带 raw 数组"));
      }
    }
    else if (c == 'h' || c == 'H') {
      printHelp();
    }
    else if (c == 'a' || c == 'A') {
      asciiOnly = !asciiOnly;
      ack(asciiOnly ? F("-> ASCII only mode") : F("-> 中文模式"));
    }
    else if (c == '\r' || c == '\n') {
      // 忽略回车换行
    }
    else {
      ack(F("未知命令，输入 h 看帮助"));
    }
  }
}

// ============================================================================
//  初始化
// ============================================================================
void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(PIN_ADC_INPUT,  INPUT);
  pinMode(PIN_SR_DATA,    OUTPUT);
  pinMode(PIN_SR_CLOCK,   OUTPUT);
  pinMode(PIN_MUX_S0,     OUTPUT);
  pinMode(PIN_MUX_S1,     OUTPUT);
  pinMode(PIN_MUX_S2,     OUTPUT);
  pinMode(PIN_MUX_S3,     OUTPUT);
  pinMode(PIN_MUX_EN,     OUTPUT);

  digitalWrite(PIN_MUX_EN, LOW);
  digitalWrite(PIN_SR_DATA, LOW);
  digitalWrite(PIN_SR_CLOCK, LOW);

  ack(F(""));
  ack(F("================================================="));
  ack(F(" 3D-ViTac 16x16  JSON 输出版"));
  ack(F("================================================="));
  ack(F(" 波特率应为 2000000。上电默认 JSON 模式。"));
  ack(F(" 队友 Python：for line in ser: d=json.loads(line)"));
  ack(F(" 调试命令： j=切模式 r=raw开关 b=重校基线 t=自检 h=帮助"));
  ack(F(""));

  // 开机自动校准一次：串口打开可能触发自动复位，开机/连接时必须空载
  ack(F("正在自动校准基线（约 1 秒），请确保布上无人..."));
  captureBaseline();
  ack(F("基线校准完成。"));
  ack(F(""));
}

// ============================================================================
//  主循环
// ============================================================================
void loop() {
  handleCommands();

  readFrame(rawFrame);

  // 减基线，找最大响应点
  int maxVal = 0; // 没有响应时保持坐标 -1，不将(0,0)误报为峰值
  int maxRow = -1, maxCol = -1;
  uint16_t nonZero = 0; // 全阵列响应时可正确计数 256
  uint16_t sum = 0;

  for (uint8_t r = 0; r < ROW_COUNT; r++) {
    for (uint8_t c = 0; c < COLUMN_COUNT; c++) {
      int d = (int)rawFrame[r][c] - (int)baseFrame[r][c];
      if (d < 0)   d = 0;
      if (d > 255) d = 255;
      diffFrame[r][c] = (uint8_t)d;
      if (d > 0) nonZero++;
      sum += d;
      if (d > maxVal) { maxVal = d; maxRow = r; maxCol = c; }
    }
  }
  if (maxVal < 0) { maxVal = 0; maxRow = -1; maxCol = -1; }

  if (mode == MODE_JSON) {
    printJsonFrame(maxVal, maxRow, maxCol, nonZero, sum);
    frameSeq++;
    delay(1);
  }
  else if (mode == MODE_GRID) {
    printMatrix();

    Serial.print(F(" 最大响应: "));
    if (maxVal <= 3) {
      Serial.println(F("无（布上没检测到压力）"));
    } else {
      Serial.print(F("行 ")); Serial.print(maxRow);
      Serial.print(F("  列 ")); Serial.print(maxCol);
      Serial.print(F("   强度 ")); Serial.print(maxVal);
      Serial.print(F("   有响应点 ")); Serial.print(nonZero);
      Serial.println(F(" 个"));
    }
    Serial.println();
    delay(120);          // 网格模式放慢一点，人眼才跟得上
  }
  else {
    // 纯数字流：16 行 × 16 个空格分隔的数，一行一帧
    for (uint8_t r = 0; r < ROW_COUNT; r++) {
      for (uint8_t c = 0; c < COLUMN_COUNT; c++) {
        Serial.print(diffFrame[r][c]);
        Serial.print(' ');
      }
      Serial.println();
    }
  }
}
