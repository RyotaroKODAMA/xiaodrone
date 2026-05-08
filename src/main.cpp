#include <Arduino.h>
#include <Wire.h>
#include <math.h> 
#include <Adafruit_BMP280.h>
#include "Adafruit_VL53L0X.h"

// --- 構造体定義 ---
struct DroneState {
  float roll, pitch, yaw;
  float gyroX, gyroY, gyroZ;
  float accX, accY, accZ;
  float altitudeBaro;
  float altitudeToF;
};

struct DroneSetpoint {
  float roll = 0;
  float pitch = 0;
  float yawRate = 0;
  float altitudeTarget = 0; // 目標高度（メートル）
};

struct PIDParameters {
  float Kp, Ki, Kd;
  float error_prev;
  float integral;
};

// --- グローバル変数 ---
DroneState currentState; 
DroneSetpoint targetState;
const uint8_t MPU_ADDR = 0x68;

// ゼロ点オフセット
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
float pitch_offset = 0, roll_offset = 0, yaw_offset = 0;

// Kp=20 に対して、まずは 1/20 くらいの 1.0 あたりから試すのが 12bit では現実的です
PIDParameters pidRoll  = { 20.0, 0.0, 1.0, 0, 0 }; 
PIDParameters pidPitch = { 20.0, 0.0, 1.0, 0, 0 };
PIDParameters pidYaw   = { 20.0, 0.0, 0.0, 0, 0 }; // ヨーは一旦 0 で OK
PIDParameters pidAltitude = { 300.0, 100.0, 30.0, 0, 0 }; // 高度制御用PID (抑制強化版)

// モーターピン
const int PIN_FR = 4, PIN_FL = 8, PIN_RL = 9, PIN_RR = 1;
const int PWM_FREQ = 16000;
const int PWM_RES = 12;

// 【修正】検証用の厳しいリミッター
const int MIN_THROTTLE = 0;    
const int MAX_THROTTLE = 4000;  // 最大を4000に制限

Adafruit_BMP280 bmp;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool tofReady = false;
bool emergencyKill = false;

// フィルタ定数
float lpfAccX = 0, lpfAccY = 0, lpfAccZ = 1.0;
float lpfBeta = 0.05;

// 気圧高度計算用
float seaLevelPressure = 101325.0; // 海面気圧（Pa）
float referencePressure = 101325.0; // キャリブレーション時の気圧
float filteredAltitude = 0.0; // フィルタ済み高度
float altitudeFilterAlpha = 0.03; // 高度のLPF係数（かなり強く平滑）
int throttleLimitRate = 100; // スロットル変化率リミッター（PWM/ループ）
int lastThrottle = 1000; // 前フレームのスロットル値（初期値調整）
float lastValidToFAltitude = -1.0;
unsigned long lastToFReadMs = 0;
const unsigned long TOF_READ_INTERVAL_MS = 50;

// --- プロトタイプ宣言 ---
void calibrateGyro();
void calibrateLevel();
void updateAttitude(float dt);
float calculatePID(float current, float target, PIDParameters &p, float dt);
void updateMotorMixer(int throttle, float p, float r, float y);
void updateAltitude();
float calculateAltitudeFromPressure(float pressure);

// 1. 関数の外（グローバル）に宣言
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

// 2. 読み込み専用の関数
void readRawMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (size_t)14, true);
  AcX = Wire.read()<<8 | Wire.read(); AcY = Wire.read()<<8 | Wire.read(); AcZ = Wire.read()<<8 | Wire.read();
  Wire.read(); Wire.read(); // skip temp
  GyX = Wire.read()<<8 | Wire.read(); GyY = Wire.read()<<8 | Wire.read(); GyZ = Wire.read()<<8 | Wire.read();
}


void stopAllMotors() {
    analogWrite(PIN_FR, 0); analogWrite(PIN_FL, 0);
    analogWrite(PIN_RL, 0); analogWrite(PIN_RR, 0);
}

void testMotor(int pin);

void setup() {
  // 1. ピン初期化 (Serialより先に!)
  pinMode(PIN_FR, OUTPUT); pinMode(PIN_FL, OUTPUT);
  pinMode(PIN_RL, OUTPUT); pinMode(PIN_RR, OUTPUT);
  stopAllMotors();
  analogWriteResolution(PWM_RES);
  analogWriteFrequency(PWM_FREQ);

  // 2. 通信開始
  Serial.begin(921600);
  Wire.begin(5, 6); Wire.setClock(400000);

  // 3. センサー初期化
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission(); // DLPF 10Hz
  
  // BMP280初期化
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found!");
    while (1);
  }
  
  Serial.println("Stabilizing...");
  delay(2000);
  calibrateGyro();
  calibrateLevel();

  // ToF初期化
  if (lox.begin()) {
    lox.startRangeContinuous();
    tofReady = true;
    Serial.println("VL53L0X ready");
  } else {
    tofReady = false;
    Serial.println("VL53L0X not found, fallback to barometer only");
  }
  
  // 気圧キャリブレーション（初期位置を基準高度0とする）
  float initialPressure = 0;
  for(int i = 0; i < 100; i++) {
    initialPressure += bmp.readPressure();
    delay(10);
  }
  referencePressure = initialPressure / 100.0;  // 平均気圧をそのまま基準に
  filteredAltitude = 0.0;  // 初期高度を0に固定
  Serial.printf("Reference Pressure: %.2f Pa\n", referencePressure);

  // シリアルバッファ掃除
  while(Serial.available() > 0) Serial.read();
  Serial.println("=== Commands ===");
  Serial.println("w: Altitude +0.01m, x: Altitude -0.01m, q: Reset Altitude");
  Serial.println("k: KILL - Force stop all motors immediately!");
  Serial.println("u: Release KILL and resume control");
  Serial.println("Note: Hovering throttle is automatically determined by PID integral.");
}

void loop() {
  static unsigned long lastLoopTime = micros();
  unsigned long now = micros();
  float dt = (now - lastLoopTime) / 1000000.0;
  if (dt < 0.004) return; 
  lastLoopTime = now;

  // --- コマンド処理 (高度目標値制御) ---
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'w') targetState.altitudeTarget += 0.01;  // 0.01m上昇
    if (c == 'x') targetState.altitudeTarget -= 0.01;  // 0.01m下降
    if (c == 'q') targetState.altitudeTarget = 0;     // 初期高度に戻す
    if (c == 'k') {
      emergencyKill = true;
      pidAltitude.integral = 0;
      pidAltitude.error_prev = 0;
      lastThrottle = 0;
      stopAllMotors();  // モーター強制停止
      Serial.println("KILL: All motors stopped!");
    }
    if (c == 'u') {
      emergencyKill = false;
      pidAltitude.integral = 0;
      pidAltitude.error_prev = 0;
      lastThrottle = 0;
      Serial.println("KILL released: control resumed");
    }
    targetState.altitudeTarget = constrain(targetState.altitudeTarget, -5.0, 10.0);
    Serial.printf("Alt Target: %.2f m\n", targetState.altitudeTarget);
  }

  if (emergencyKill) {
    stopAllMotors();
    return;
  }
  
  updateAltitude();

  updateAttitude(dt);

  // 高度PIDで直接スロットルを計算（基準値不要、integral項で自動調整）
  int throttle = (int)calculatePID(currentState.altitudeBaro, targetState.altitudeTarget, pidAltitude, dt);
  throttle = constrain(throttle, MIN_THROTTLE, MAX_THROTTLE);
  
  // スロットル変化率リミッター（急激な上下動を防止）
  if (throttle > lastThrottle + throttleLimitRate) {
    throttle = lastThrottle + throttleLimitRate;
  } else if (throttle < lastThrottle - throttleLimitRate) {
    throttle = lastThrottle - throttleLimitRate;
  }
  lastThrottle = throttle;
  
  float outP = calculatePID(currentState.pitch, targetState.pitch, pidPitch, dt);
  float outR = calculatePID(currentState.roll,  targetState.roll,  pidRoll,  dt);

  // --- D. モーター出力の計算 (Mixerの中身をここでシミュレートして表示) ---
  int mFR = throttle + outP - outR;
  int mFL = throttle + outP + outR;
  int mRL = throttle - outP + outR;
  int mRR = throttle - outP - outR;




  updateMotorMixer(throttle, outP, outR, 0);

// --- E. 超詳細ログ出力 (100msおき) ---
  static unsigned long lastLog = 0;
  if (now / 1000 - lastLog > 100) {
    lastLog = now / 1000;

    Serial.println("-----------------------------------------------------------------------");
    // 1段目：姿勢データ
    Serial.printf("ATTITUDE | Pitch:%6.1f deg | Roll:%6.1f deg\n", currentState.pitch, currentState.roll);
    
    // 2段目：PID計算結果
    Serial.printf("PID OUT  | OutP:%7.1f | OutR:%7.1f | (KpP:%.1f, KpR:%.1f)\n", outP, outR, pidPitch.Kp, pidRoll.Kp);
    
    // 3段目：高度情報
    Serial.printf("ALTITUDE | Current:%.2f m | Target:%.2f m | Baro:%.2f m | ToF:%.2f m | Pressure:%.0f Pa\n", 
            currentState.altitudeBaro, targetState.altitudeTarget, 
            filteredAltitude, currentState.altitudeToF, bmp.readPressure());
    
    // 4段目：各モーターへの最終PWM値 (12bit: 0-4095)
    Serial.printf("MOTORS   | FR:%4d | FL:%4d | RL:%4d | RR:%4d | Thr:%d\n", 
                  constrain(mFR, 0, 4095), constrain(mFL, 0, 4095), 
                  constrain(mRL, 0, 4095), constrain(mRR, 0, 4095), 
                  throttle);
  }
}


// void updateAttitude(float dt) {
//   Wire.beginTransmission(MPU_ADDR);
//   Wire.write(0x3B); 
//   Wire.endTransmission(false);
//   Wire.requestFrom(MPU_ADDR, (size_t)14, true);

//   int16_t AcX = Wire.read()<<8|Wire.read(); 
//   int16_t AcY = Wire.read()<<8|Wire.read(); 
//   int16_t AcZ = Wire.read()<<8|Wire.read();
//   Wire.read(); Wire.read(); // skip temp
//   int16_t GyX = Wire.read()<<8|Wire.read(); 
//   int16_t GyY = Wire.read()<<8|Wire.read(); 
//   int16_t GyZ = Wire.read()<<8|Wire.read();

//   // 物理量変換
//   currentState.gyroX = (GyX - gyro_x_offset) / 131.0; 
//   currentState.gyroY = (GyY - gyro_y_offset) / 131.0; 
//   currentState.gyroZ = (GyZ - gyro_z_offset) / 131.0;
  
//   // 加速度角度（生の加速度を使用）
//   float accPitch = atan2((float)AcY, sqrt(pow((float)AcX,2) + pow((float)AcZ,2))) * 180 / PI;
//   float accRoll  = atan2(-(float)AcX, (float)AcZ) * 180 / PI;

//   // 【ドリフト対策】相補フィルタの比率を調整
//   // 加速度の比率(0.05)を少し上げるとドリフトからの復帰が早くなります
//   currentState.pitch = 0.95 * (currentState.pitch + currentState.gyroX * dt) + 0.05 * accPitch;
//   currentState.roll  = 0.95 * (currentState.roll  + currentState.gyroY * dt) + 0.05 * accRoll;
//   currentState.yaw  += currentState.gyroZ * dt;

//   // オフセット適用（これで 0点 が安定する）
//   // currentState.pitch -= pitch_offset;
//   // currentState.roll  -= roll_offset;
//   // Yawは常に起動時が0になるようオフセットを引かないか、別途管理
// }

// 1. グローバル変数で宣言
// float lpfAccX = 0, lpfAccY = 0, lpfAccZ = 1.0;
// float lpfBeta = 0.05; // 0.01（強力）〜0.1（弱め）で調整

void updateAttitude(float dt) {
  readRawMPU();

  // ★重要：ジャイロを dps に変換
  currentState.gyroX = (GyX - gyro_x_offset) / 131.0;
  currentState.gyroY = (GyY - gyro_y_offset) / 131.0;
  currentState.gyroZ = (GyZ - gyro_z_offset) / 131.0;

  static float filteredGyX = 0, filteredGyY = 0;
  float gyroAlpha = 0.3; // 0.1〜0.5で調整。小さいほど強力

  filteredGyX = (1.0 - gyroAlpha) * filteredGyX + gyroAlpha * ((GyX - gyro_x_offset) / 131.0);
  filteredGyY = (1.0 - gyroAlpha) * filteredGyY + gyroAlpha * ((GyY - gyro_y_offset) / 131.0);

  currentState.gyroX = filteredGyX;
  currentState.gyroY = filteredGyY;

  // 加速度LPF
  lpfAccX = (1.0 - lpfBeta) * lpfAccX + lpfBeta * (AcX / 16384.0);
  lpfAccY = (1.0 - lpfBeta) * lpfAccY + lpfBeta * (AcY / 16384.0);
  lpfAccZ = (1.0 - lpfBeta) * lpfAccZ + lpfBeta * (AcZ / 16384.0);

  float accPitch = (atan2(lpfAccY, sqrt(lpfAccX*lpfAccX + lpfAccZ*lpfAccZ)) * 180 / PI) - pitch_offset;
  float accRoll  = (atan2(-lpfAccX, lpfAccZ) * 180 / PI) - roll_offset;

  // 相補フィルタ
  currentState.pitch = 0.98 * (currentState.pitch + currentState.gyroX * dt) + 0.02 * accPitch;
  currentState.roll  = 0.98 * (currentState.roll  + currentState.gyroY * dt) + 0.02 * accRoll;
}



void calibrateLevel() {
  Serial.println("Level Calibrating... PLEASE WAIT 2 SECONDS");
  delay(2000); // フィルタが落ち着くのをしっかり待つ

  float p_sum = 0, r_sum = 0;
  int samples = 500; // サンプルを増やして精度を上げる

  for(int i = 0; i < samples; i++) {
    readRawMPU(); // 加速度・ジャイロを読み込むだけの関数（後述）

    // フィルタを通さない「生の加速度角度」を計算
    float rawAccP = atan2((float)AcY, sqrt(pow((float)AcX,2) + pow((float)AcZ,2))) * 180 / PI;
    float rawAccR = atan2(-(float)AcX, (float)AcZ) * 180 / PI;

    p_sum += rawAccP;
    r_sum += rawAccR;
    delay(2);
  }

  pitch_offset = p_sum / samples;
  roll_offset = r_sum / samples;

  // キャリブレーションが終わったら、今の姿勢を 0 に初期化
  currentState.pitch = 0;
  currentState.roll = 0;
}

void calibrateGyro() {
  long sx=0, sy=0, sz=0;
  for(int i=0; i<500; i++) {
    Wire.beginTransmission(MPU_ADDR); 
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (size_t)6, true);
    sx += (int16_t)(Wire.read()<<8 | Wire.read()); 
    sy += (int16_t)(Wire.read()<<8 | Wire.read()); 
    sz += (int16_t)(Wire.read()<<8 | Wire.read());
    delay(2);
  }
  gyro_x_offset = sx / 500.0; 
  gyro_y_offset = sy / 500.0; 
  gyro_z_offset = sz / 500.0;
}

void updateMotorMixer(int throttle, float p, float r, float y) {
  if (throttle < 50) {
    analogWrite(PIN_FR, 0); analogWrite(PIN_FL, 0);
    analogWrite(PIN_RL, 0); analogWrite(PIN_RR, 0);
    return;
  }
  // int mFR = throttle + p - r - y;
  // int mFL = throttle + p + r + y;
  // int mRL = throttle - p + r - y;
  // int mRR = throttle - p - r + y;
  int mFR = throttle + p - r ;
  int mFL = throttle + p + r ;
  int mRL = throttle - p + r ;
  int mRR = throttle - p - r ;
  analogWrite(PIN_FR, constrain(mFR, MIN_THROTTLE, MAX_THROTTLE));
  analogWrite(PIN_FL, constrain(mFL, MIN_THROTTLE, MAX_THROTTLE));
  analogWrite(PIN_RL, constrain(mRL, MIN_THROTTLE, MAX_THROTTLE));
  analogWrite(PIN_RR, constrain(mRR, MIN_THROTTLE, MAX_THROTTLE));
}

float calculatePID(float current, float target, PIDParameters &p, float dt) {
  float error = target - current;
  float P = p.Kp * error;
  p.integral += error * dt;
  // アンチウインドアップ：積分項を厳しく制限してスロットル暴走を防ぐ
  p.integral = constrain(p.integral, -5, 5);
  float I = p.Ki * p.integral;
  float D = p.Kd * (error - p.error_prev) / dt;
  p.error_prev = error;
  return P + I + D;
}

void testMotor(int pin) {
    stopAllMotors();
    analogWrite(pin,250); // 250くらいで回してみる
}

// 気圧から高度を計算する関数
// 国際標準大気モデルを使用
float calculateAltitudeFromPressure(float pressure) {
  // h = 44330 * (1 - (P/P0)^(1/5.255))
  float ratio = pressure / referencePressure;
  float altitude = 44330.0 * (1.0 - pow(ratio, 1.0 / 5.255));
  return altitude;
}

// 高度を更新する関数
void updateAltitude() {
  float pressure = bmp.readPressure();
  float baroAltitude = calculateAltitudeFromPressure(pressure);
  
  // 低域フィルタ（気圧ノイズを軽減）
  filteredAltitude = (1.0 - altitudeFilterAlpha) * filteredAltitude + altitudeFilterAlpha * baroAltitude;

  float tofAltitude = lastValidToFAltitude;
  if (tofReady && (millis() - lastToFReadMs >= TOF_READ_INTERVAL_MS)) {
    lastToFReadMs = millis();
    if (lox.isRangeComplete()) {
      float distanceMm = (float)lox.readRange();
      if (distanceMm > 0) {
        float pitchRad = currentState.pitch * PI / 180.0;
        float rollRad  = currentState.roll * PI / 180.0;
        tofAltitude = (distanceMm / 1000.0) * cos(pitchRad) * cos(rollRad);
        lastValidToFAltitude = tofAltitude;
      }
    }
  }

  currentState.altitudeToF = tofAltitude;

  // 1m以下ではToFを優先、1m以上は気圧高度を使う
  if (tofAltitude > 0.0 && filteredAltitude < 1.0) {
    currentState.altitudeBaro = tofAltitude;
  } else {
    currentState.altitudeBaro = filteredAltitude;
  }
}