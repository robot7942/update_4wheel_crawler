/*
 * クローラーユニット DCモーター単体テスト
 * ESP32-WROOM-32E / Arduino
 * ドライバ: TB67H450FNG
 *
 * シリアルコマンド (115200bps, 改行コードあり):
 *   s <id>        : テスト対象のモーターを選択 (0:左前 1:右前 2:左後ろ 3:右後ろ)
 *   f <duty>      : 正転 (duty 0-100 [%])
 *   r <duty>      : 逆転
 *   b             : ブレーキ
 *   x             : ストップ (ハイインピーダンス)
 *   e             : エンコーダ値表示
 *   z             : エンコーダ値リセット
 *   a             : 自動テスト (正転→停止→逆転→停止)
 *   h             : ヘルプ
 */

#include <Arduino.h>

// ===== モーター識別 =====
enum MotorId { FL = 0, FR = 1, RL = 2, RR = 3, MOTOR_NUM = 4 };
const char* MOTOR_NAME[MOTOR_NUM] = {"左前(FL)", "右前(FR)", "左後(RL)", "右後(RR)"};

// ===== ピン定義 =====
// モタドラ入力 {A, B}
const int MOTOR_PIN_A[MOTOR_NUM] = {18, 27, 32, 13};
const int MOTOR_PIN_B[MOTOR_NUM] = {19, 26, 33, 14};

// エンコーダ {A, B}
const int ENC_PIN_A[MOTOR_NUM] = {16, 36, 23, 34};
const int ENC_PIN_B[MOTOR_NUM] = {17, 39, 25, 35};

const int PIN_LED = 2;

// ===== PWM設定 =====
const int PWM_FREQ = 20000;   // 20kHz
const int PWM_RES  = 10;      // 10bit (0-1023)
const int PWM_MAX  = (1 << PWM_RES) - 1;

// ===== エンコーダ =====
volatile long encCount[MOTOR_NUM] = {0, 0, 0, 0};
volatile uint8_t encPrev[MOTOR_NUM] = {0, 0, 0, 0};

// 4逓倍デコード用テーブル (prev<<2 | curr)
const int8_t ENC_TABLE[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0
};

template <int N>
void IRAM_ATTR encISR() {
  uint8_t a = digitalRead(ENC_PIN_A[N]);
  uint8_t b = digitalRead(ENC_PIN_B[N]);
  uint8_t curr = (a << 1) | b;
  encCount[N] += ENC_TABLE[(encPrev[N] << 2) | curr];
  encPrev[N] = curr;
}

// ===== モーター制御 =====
int currentMotor = FL;

void motorSetup() {
  for (int i = 0; i < MOTOR_NUM; i++) {
    // ESP32 Arduino core 3.x 系: ledcAttach(pin, freq, resolution)
    ledcAttach(MOTOR_PIN_A[i], PWM_FREQ, PWM_RES);
    ledcAttach(MOTOR_PIN_B[i], PWM_FREQ, PWM_RES);
    ledcWrite(MOTOR_PIN_A[i], 0);
    ledcWrite(MOTOR_PIN_B[i], 0);
  }
}

// duty: 0-100 [%]
void motorForward(int id, int duty) {
  duty = constrain(duty, 0, 100);
  ledcWrite(MOTOR_PIN_A[id], duty * PWM_MAX / 100);
  ledcWrite(MOTOR_PIN_B[id], 0);
}

void motorReverse(int id, int duty) {
  duty = constrain(duty, 0, 100);
  ledcWrite(MOTOR_PIN_A[id], 0);
  ledcWrite(MOTOR_PIN_B[id], duty * PWM_MAX / 100);
}

void motorBrake(int id) {
  ledcWrite(MOTOR_PIN_A[id], PWM_MAX);
  ledcWrite(MOTOR_PIN_B[id], PWM_MAX);
}

void motorStop(int id) {
  ledcWrite(MOTOR_PIN_A[id], 0);
  ledcWrite(MOTOR_PIN_B[id], 0);
}

void allMotorStop() {
  for (int i = 0; i < MOTOR_NUM; i++) motorStop(i);
}

// ===== エンコーダ初期化 =====
void encoderSetup() {
  for (int i = 0; i < MOTOR_NUM; i++) {
    // 34,35,36,39 は入力専用で内蔵プルアップ不可 → 外付けプルアップ前提
    if (ENC_PIN_A[i] >= 34) {
      pinMode(ENC_PIN_A[i], INPUT);
      pinMode(ENC_PIN_B[i], INPUT);
    } else {
      pinMode(ENC_PIN_A[i], INPUT_PULLUP);
      pinMode(ENC_PIN_B[i], INPUT_PULLUP);
    }
    encPrev[i] = (digitalRead(ENC_PIN_A[i]) << 1) | digitalRead(ENC_PIN_B[i]);
  }

  attachInterrupt(ENC_PIN_A[FL], encISR<FL>, CHANGE);
  attachInterrupt(ENC_PIN_B[FL], encISR<FL>, CHANGE);
  attachInterrupt(ENC_PIN_A[FR], encISR<FR>, CHANGE);
  attachInterrupt(ENC_PIN_B[FR], encISR<FR>, CHANGE);
  attachInterrupt(ENC_PIN_A[RL], encISR<RL>, CHANGE);
  attachInterrupt(ENC_PIN_B[RL], encISR<RL>, CHANGE);
  attachInterrupt(ENC_PIN_A[RR], encISR<RR>, CHANGE);
  attachInterrupt(ENC_PIN_B[RR], encISR<RR>, CHANGE);
}

long getEnc(int id) {
  noInterrupts();
  long v = encCount[id];
  interrupts();
  return v;
}

void resetEnc() {
  noInterrupts();
  for (int i = 0; i < MOTOR_NUM; i++) encCount[i] = 0;
  interrupts();
}

void printEnc() {
  Serial.print("Enc [");
  for (int i = 0; i < MOTOR_NUM; i++) {
    Serial.print(MOTOR_NAME[i]);
    Serial.print("=");
    Serial.print(getEnc(i));
    if (i < MOTOR_NUM - 1) Serial.print(", ");
  }
  Serial.println("]");
}

// ===== 自動テスト =====
void runAutoTest(int id) {
  Serial.printf("--- 自動テスト開始: %s ---\n", MOTOR_NAME[id]);
  resetEnc();

  Serial.println("正転 30%");
  motorForward(id, 30);
  delay(1500);
  long fwd = getEnc(id);
  Serial.printf("  正転後カウント: %ld\n", fwd);

  motorStop(id);
  delay(500);

  resetEnc();
  Serial.println("逆転 30%");
  motorReverse(id, 30);
  delay(1500);
  long rev = getEnc(id);
  Serial.printf("  逆転後カウント: %ld\n", rev);

  motorStop(id);

  // 判定
  Serial.println("--- 結果 ---");
  if (fwd == 0 && rev == 0) {
    Serial.println("NG: エンコーダが反応していません (配線/プルアップ/電源を確認)");
  } else if (fwd > 0 && rev < 0) {
    Serial.println("OK: 正転で+、逆転で-");
  } else if (fwd < 0 && rev > 0) {
    Serial.println("注意: 極性が逆です (エンコーダA/B入れ替え or ソフトで符号反転)");
  } else {
    Serial.println("要確認: 方向とカウントの符号が一致しません");
  }
}

// ===== コマンド処理 =====
void printHelp() {
  Serial.println("=== コマンド一覧 ===");
  Serial.println("s <id> : モーター選択 (0:FL 1:FR 2:RL 3:RR)");
  Serial.println("f <duty>: 正転 (0-100)");
  Serial.println("r <duty>: 逆転 (0-100)");
  Serial.println("b : ブレーキ / x : ストップ");
  Serial.println("e : エンコーダ表示 / z : リセット");
  Serial.println("a : 自動テスト / h : ヘルプ");
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);
  int arg = (line.length() > 1) ? line.substring(1).toInt() : 0;

  switch (cmd) {
    case 's':
      if (arg >= 0 && arg < MOTOR_NUM) {
        allMotorStop();
        currentMotor = arg;
        Serial.printf("選択: %s\n", MOTOR_NAME[currentMotor]);
      } else {
        Serial.println("idは0-3");
      }
      break;
    case 'f':
      motorForward(currentMotor, arg);
      Serial.printf("%s 正転 %d%%\n", MOTOR_NAME[currentMotor], arg);
      break;
    case 'r':
      motorReverse(currentMotor, arg);
      Serial.printf("%s 逆転 %d%%\n", MOTOR_NAME[currentMotor], arg);
      break;
    case 'b':
      motorBrake(currentMotor);
      Serial.println("ブレーキ");
      break;
    case 'x':
      motorStop(currentMotor);
      Serial.println("ストップ");
      break;
    case 'e':
      printEnc();
      break;
    case 'z':
      resetEnc();
      Serial.println("エンコーダリセット");
      break;
    case 'a':
      runAutoTest(currentMotor);
      break;
    case 'h':
      printHelp();
      break;
    default:
      Serial.println("不明なコマンド (hでヘルプ)");
      break;
  }
}

// ===== setup / loop =====
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);

  motorSetup();
  encoderSetup();

  Serial.println("\nクローラーモーター単体テスト 起動");
  printHelp();
  Serial.printf("選択中: %s\n", MOTOR_NAME[currentMotor]);
}

void loop() {
  static String buf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      handleCommand(buf);
      buf = "";
    } else {
      buf += c;
    }
  }

  // 生存確認LED (1秒周期)
  digitalWrite(PIN_LED, (millis() / 500) % 2);
}