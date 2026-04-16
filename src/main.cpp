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
  int throttle = 150; // 検証用に最初から少し上げる設定
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

// PIDパラメータ (一旦 D は 0 で安定化を優先)
PIDParameters pidRoll  = { 0.6, 0.0, 0.0, 0, 0 }; 
PIDParameters pidPitch = { 0.6, 0.0, 0.0, 0, 0 };
PIDParameters pidYaw   = { 2.0, 0.0, 0.0, 0, 0 };

// モーターピン
const int PIN_FR = 4, PIN_FL = 8, PIN_RL = 9, PIN_RR = 1;
const int PWM_FREQ = 16000;
const int PWM_RES = 12;

// 【修正】検証用の厳しいリミッター
const int MIN_THROTTLE = 0;    
const int MAX_THROTTLE = 125;  // 最大を125に制限

Adafruit_BMP280 bmp;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// --- プロトタイプ宣言 ---
void calibrateGyro();
void calibrateLevel();
void updateAttitude(float dt);
float calculatePID(float current, float target, PIDParameters &p, float dt);
void updateMotorMixer(int throttle, float p, float r, float y);

// 1. 関数の外（グローバル）に宣言
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

// 2. 読み込み専用の関数
void readRawMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // 加速度データの先頭アドレス
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (size_t)14, true);

  // 14バイト分を順番に読み込む
  AcX = Wire.read()<<8 | Wire.read(); 
  AcY = Wire.read()<<8 | Wire.read(); 
  AcZ = Wire.read()<<8 | Wire.read();
  Wire.read(); Wire.read(); // 温度データ(2バイト)を読み飛ばす
  GyX = Wire.read()<<8 | Wire.read(); 
  GyY = Wire.read()<<8 | Wire.read(); 
  GyZ = Wire.read()<<8 | Wire.read();
}

void setup() {
  Serial.begin(921600); // 高速通信
  Wire.begin(5, 6);
  Wire.setClock(400000);

  // MPU6500初期化
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00); // Wake up
  Wire.endTransmission();

  // 【ドリフト・振動対策】内蔵デジタルフィルタ(DLPF)を最強に
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); Wire.write(0x05); // 10Hz LPF
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x18); // 2000dps
  Wire.endTransmission();

  Serial.println("Stabilizing Sensor (Wait 2s)...");
  delay(2000); // ここで DLPF を安定させ

  calibrateGyro();   // 回転のズレを補正

  calibrateLevel();  // 角度のズレを補正（ドリフト対策）

  targetState.throttle = 0; // 最初は停止
  Serial.println("System Ready. Send 's' to start motors.");
}

void loop() {
  // --- A. タイミング管理 (250Hz) ---
  static unsigned long lastLoopTime = micros();
  unsigned long now = micros();
  float dt = (now - lastLoopTime) / 1000000.0;
  if (dt < 0.004) return; 
  lastLoopTime = now;

  // --- B. コマンド処理 ---
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') { targetState.throttle = 150; Serial.println("MOTOR START"); }
    if (c == 'q') { targetState.throttle = 0;   Serial.println("MOTOR STOP"); }
  }

  // --- C. 姿勢更新 & PID計算 ---
  updateAttitude(dt);

  float currentP = currentState.pitch - pitch_offset;
  float currentR = currentState.roll  - roll_offset;

  float outPitch = calculatePID(currentP, targetState.pitch, pidPitch, dt);
  float outRoll  = calculatePID(currentR, targetState.roll,  pidRoll,  dt);
  // ヨーは振動が激しいので一旦 0 に固定
  float outYaw   = 0; 

  // --- D. 出力反映 ---
  updateMotorMixer(targetState.throttle, outPitch, outRoll, outYaw);

  // --- E. 3軸詳細デバッグ出力 ---
  static unsigned long lastLog = 0;
  if (millis() - lastLog > 50) { // 20Hzで出力
    lastLog = millis();

    // 1. 加速度センサーだけの「生」の角度を再計算（デバッグ用）
    float rawAccP = atan2((float)AcY, sqrt(pow((float)AcX,2) + pow((float)AcZ,2))) * 180 / PI - pitch_offset;
    float rawAccR = atan2(-(float)AcX, (float)AcZ) * 180 / PI - roll_offset;

    // 2. モーターの各出力を計算（確認用）
    // updateMotorMixer内の計算と同じものをシミュレート
    int mFR = targetState.throttle - outPitch - outRoll;
    int mFL = targetState.throttle - outPitch + outRoll;

    // 3. 超詳細シリアル表示
    // [姿勢データ] [PID出力] [モーター出力想定]
    Serial.printf("P[Raw:%5.1f Deg:%5.1f Out:%5.0f] | R[Raw:%5.1f Deg:%5.1f Out:%5.0f] | Mot[FR:%4d FL:%4d] | Thr:%d\n", 
                  rawAccP, currentState.pitch, outPitch,
                  rawAccR, currentState.roll, outRoll,
                  mFR, mFL, targetState.throttle);
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

void updateAttitude(float dt) {
  readRawMPU(); // 生データの更新

  // 1. 加速度角度を計算し、その瞬間にオフセットを引く！
  float accPitch = (atan2((float)AcY, sqrt(pow((float)AcX,2) + pow((float)AcZ,2))) * 180 / PI) - pitch_offset;
  float accRoll  = (atan2(-(float)AcX, (float)AcZ) * 180 / PI) - roll_offset;

  // 2. 補正済みの加速度角度を使って相補フィルタ
  // ここではもう -= pitch_offset は絶対にしない
  currentState.pitch = 0.95 * (currentState.pitch + currentState.gyroX * dt) + 0.05 * accPitch;
  currentState.roll  = 0.95 * (currentState.roll  + currentState.gyroY * dt) + 0.05 * accRoll;
  // 0.95 : 0.05 → 0.99 : 0.01 に変更
  // currentState.pitch = 0.99 * (currentState.pitch + currentState.gyroX * dt) + 0.01 * accPitch;
  // currentState.roll  = 0.99 * (currentState.roll  + currentState.gyroY * dt) + 0.01 * accRoll;
  currentState.yaw  += currentState.gyroZ * dt;
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
  int mFR = throttle - p - r - y;
  int mFL = throttle - p + r + y;
  int mRL = throttle + p + r - y;
  int mRR = throttle + p - r + y;

  analogWrite(PIN_FR, constrain(mFR, MIN_THROTTLE, MAX_THROTTLE));
  analogWrite(PIN_FL, constrain(mFL, MIN_THROTTLE, MAX_THROTTLE));
  analogWrite(PIN_RL, constrain(mRL, MIN_THROTTLE, MAX_THROTTLE));
  analogWrite(PIN_RR, constrain(mRR, MIN_THROTTLE, MAX_THROTTLE));
}

float calculatePID(float current, float target, PIDParameters &p, float dt) {
  float error = target - current;
  float P = p.Kp * error;
  p.integral += error * dt;
  p.integral = constrain(p.integral, -50, 50);
  float I = p.Ki * p.integral;
  float D = p.Kd * (error - p.error_prev) / dt;
  p.error_prev = error;
  return P + I + D;
}