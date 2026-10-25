/*
 * ============================================================================
 *  3D-ViTac 16×16 压阻阵列读取 —— 增强调试版
 * ----------------------------------------------------------------------------
 *  硬件：Arduino Nano / Uno（ATmega328P）
 *        CD74HC4067 (16 路模拟多路复用器) ×1
 *        SN74HC595N (8 位移位寄存器) ×2  ← 级联出 16 路
 *        压敏布（16 根行线 + 16 根列线 = 32 根）
 *        20kΩ 电阻 ×1（分压下拉）
 *
 *  ⚠️ 串口波特率必须是 2000000，否则全是乱码！
 *      串口监视器右上角波特率下拉框里选 2000000
 *
 *  串口命令（在串口监视器输入框敲一下回车）：
 *      b  → 重新校准空载基线（换姿态、换人坐之前按这个）
 *      t  → MUX 自检：只看电路通不通，不读布
 *      m  → 切换「网格显示 / 纯数字流」两种输出
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

// ---------------------------------------------------------------- 引脚定义
//  这套定义和官方固件 MatrixArray.ino 完全一致，已验证
#define PIN_ADC_INPUT    A0    // ← CD74HC4067 的 SIG
#define PIN_SR_DATA      2     // ← 74HC595 的 SER  (14脚)
#define PIN_SR_CLOCK     3     // ← 74HC595 的 SRCLK(11脚) 和 RCLK(12脚) 都要接这根
#define PIN_MUX_S0       4     // ← CD74HC4067 的 S0  (10脚)
#define PIN_MUX_S1       5     // ← CD74HC4067 的 S1  (11脚)
#define PIN_MUX_S2       6     // ← CD74HC4067 的 S2  (14脚)
#define PIN_MUX_S3       7     // ← CD74HC4067 的 S3  (13脚)
#define PIN_MUX_EN       8     // ← CD74HC4067 的 E#  (15脚) 低电平使能

// ---------------------------------------------------------------- 缓冲区
uint8_t rawFrame[ROW_COUNT][COLUMN_COUNT];      // 本次读到的原始值
uint8_t baseFrame[ROW_COUNT][COLUMN_COUNT];     // 空载基线
uint8_t diffFrame[ROW_COUNT][COLUMN_COUNT];     // 相减后的相对变化

bool matrixMode   = true;   // true = 打印网格（人看）  false = 纯数字（给程序看）
bool baselineReady = false;
bool asciiOnly    = false;  // true = 只输出 ASCII（串口显示乱码时按 'a' 切换）

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
  uint8_t sum[ROW_COUNT][COLUMN_COUNT] = {{0}};
  const uint8_t SAMPLES = 16;

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

// ============================================================================
//  MUX 自检：只验证电路通不通，完全不依赖布
//  正常表现：16 个数里应该能看到有变化（不是全 0，也不是全一样）
// ============================================================================
void muxSelfTest() {
  Serial.println(F("=== MUX 自检：16 个地址依次读 A0 ==="));
  Serial.println(F("（如果 16 个数完全一样 → 地址线没接对 / MUX 没使能）"));
  Serial.println();

  uint8_t first = analogRead(PIN_ADC_INPUT);
  for (uint8_t r = 0; r < ROW_COUNT; r++) {
    selectRow(r);
    delayMicroseconds(200);
    uint8_t v = analogRead(PIN_ADC_INPUT);
    Serial.print(F("  地址 "));
    if (r < 10) Serial.print('0');
    Serial.print(r);
    Serial.print(F("  →  A0 = "));
    if (v < 10)   Serial.print('0');
    if (v < 100)  Serial.print('0');
    Serial.print(v);
    if (r == 0) first = v;
    Serial.print(F("   "));
    for (uint8_t k = 0; k < 12; k++) {
      Serial.print((v > first + 3) ? '#' : (v > first - 3 ? '.' : ' '));
    }
    Serial.println();
  }
  Serial.println();
  Serial.println(F("  # = 比首值高   . = 接近首值   空格 = 比首值低"));
  Serial.println(F("  正常现象：不同行读到的值不一样（因为布的各行阻值不同）"));
  Serial.println();
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
//  打印帮助
// ============================================================================
void printHelp() {
  Serial.println();
  Serial.println(F("命令： b=重新校准基线  t=MUX自检  m=切换输出模式  h=帮助"));
  Serial.println();
}

// ============================================================================
//  处理串口命令
// ============================================================================
void handleCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'b' || c == 'B') {
      Serial.println(F("正在校准基线，请确保布上无人..."));
      captureBaseline();
      Serial.println(F("基线校准完成。"));
      Serial.println();
    }
    else if (c == 't' || c == 'T') {
      muxSelfTest();
    }
    else if (c == 'm' || c == 'M') {
      matrixMode = !matrixMode;
      Serial.println(matrixMode ? F("→ 网格显示模式") : F("→ 纯数字流模式"));
    }
    else if (c == 'h' || c == 'H') {
      printHelp();
    }
    else if (c == 'a' || c == 'A') {
      asciiOnly = !asciiOnly;
      Serial.println(asciiOnly ? F("-> ASCII only mode") : F("-> 中文模式"));
    }
    else if (c == '\r' || c == '\n') {
      // 忽略回车换行
    }
    else {
      Serial.println(F("未知命令，输入 h 看帮助"));
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

  Serial.println();
  Serial.println(F("================================================="));
  Serial.println(F(" 3D-ViTac 16x16 压阻阵列读取  增强调试版"));
  Serial.println(F("================================================="));
  Serial.println(F(" 波特率应为 2000000。输入 h 看命令，t 做自检。"));
  Serial.println();

  // 开机自动校准一次
  Serial.println(F("正在自动校准基线（约 1 秒），请确保布上无人..."));
  captureBaseline();
  Serial.println(F("基线校准完成。"));
  Serial.println();

  printHelp();
}

// ============================================================================
//  主循环
// ============================================================================
void loop() {
  handleCommands();

  readFrame(rawFrame);

  // 减基线，找最大响应点
  int maxVal = -1;
  int maxRow = -1, maxCol = -1;
  uint8_t nonZero = 0;
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

  if (matrixMode) {
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
