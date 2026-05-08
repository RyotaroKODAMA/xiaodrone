#include <Arduino.h>
#include <Wire.h>
#include <math.h> 
#include <Adafruit_BMP280.h>
#include "Adafruit_VL53L0X.h"


// --- parameters to show attitude ---
struct DroneState {
  // 1. naw attitude（角度 deg）
  float roll;
  float pitch;
  float yaw;

  // 2. 現在の回転スピード（角速度 deg/sec）: PIDの「D項」で超重要！
  // now angle speed (deg/sec) USE at D
  float gyroX;
  float gyroY;
  float gyroZ;

  // 3. 現在の加速度（G）
  // now acceralation
  float accX;
  float accY;
  float accZ;

  // 4. 高度情報（mm または cm）
  // now attitude
  float altitudeBaro; // 気圧計からの相対高度
  float altitudeToF;  // ToFからの絶対距離
};

// 実際にデータを入れる「箱」を実体化
DroneState currentState; 

const uint8_t MPU_ADDR = 0x68;
// 角度と時間の変数
// float roll = 0, pitch = 0, yaw = 0;
float ax, ay, az, gx, gy, gz; // 6軸物理量
unsigned long lastTime;
// ジャイロのゼロ点ズレ（オフセット）を保存する変数
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
float baseAltitude = 0; // 起動時の基準高度

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
Adafruit_BMP280 bmp; // I2C

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ==========================================
// モーターピンの定義 (XIAO ESP32-S3)
// ※ピン番号は実際の配線に合わせて変更してください
// ==========================================
const int PIN_FR = 4;  // M1: 右前
const int PIN_FL = 9;  // M2: 左前
const int PIN_RL = 10; // M3: 左後
const int PIN_RR = 1;  // M4: 右後
// PWM設定
const int PWM_FREQ = 16000; // 16kHz
const int PWM_RES = 12;     // 12bit (0-4095)
// 出力の安全制限
const int MIN_THROTTLE = 250;  // 摩擦に打ち勝って回り始める最低値 (要調整)
const int MAX_THROTTLE = 4000; // フルパワー(4095)の少し手前で制限



// --- PIDパラメータ (最初は小さめの値から！) ---
struct PIDParameters {
  float Kp, Ki, Kd;
  float error_prev;
  float integral;
};

// ロール、ピッチ、ヨー、それぞれのPIDコントローラー
PIDParameters pidRoll  = { 1.5, 0.0, 0.05, 0, 0 }; 
PIDParameters pidPitch = { 1.5, 0.0, 0.05, 0, 0 };
PIDParameters pidYaw   = { 2.0, 0.0, 0.0, 0, 0 };

// --- 送信機からの目標値 (Setpoint) ---
struct DroneSetpoint {
  float roll = 0;     // 目標ロール角 (0度 = 水平)
  float pitch = 0;    // 目標ピッチ角
  float yawRate = 0;  // 目標回転速度
  int throttle = 0;   // スロットル (0〜4095)
};
DroneSetpoint targetState;




// --- 初期化・キャリブレーション系 ---
void calibrateGyro();      // ジャイロのゼロ点
void calibrateAltitude();  // 高度のゼロ点

// --- 制御ループ系 ---
void updateAttitude();     // センサー読み取り＆姿勢計算
float calculatePID(float current, float target, PIDParameters &p, float dt); // PIDエンジン

// --- 出力系 ---
// ミキシング計算とモーターへの書き込みをこれ1本にまとめる
void updateMotorMixer(int throttle, float p, float r, float y);




void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Wire.begin(5, 6);
  Wire.setClock(400000);

  // MPU6500を起動
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  lastTime = millis();
  calibrateGyro();
  Serial.println("MPU6500 Start!");

  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }
/* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. ここを NORMAL に変更！ */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. ついでに最速(1ms)にしておく */
  calibrateAltitude(); // 高度のゼロ点補正（相対高度用）


  // VL53L0Xの起動
  Serial.println("Adafruit VL53L0X test.");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power
  Serial.println(F("VL53L0X API Continuous Ranging example\n\n"));
  // start continuous ranging
  lox.startRangeContinuous();


}




// void loop() {
//   static unsigned long lastLoopTime = micros();
//   unsigned long now = micros();
//   float dt = (now - lastLoopTime) / 1000000.0; // 秒単位

//   // 指定した周期（例: 4ms = 250Hz）まで待機
//   if (dt < 0.004) return; 
//   lastLoopTime = now;

//   // 1. 最新の姿勢を取得
//   updateAttitude();

//   // 2. 高度・ToFデータの取得（必要に応じて）
//   // ※ToFは毎秒30回程度なので、isRangeCompleteでチェック
//   if (lox.isRangeComplete()) {
//     currentState.altitudeToF = lox.readRange();
//     // ここで前述の傾き補正を入れても良い
//   }

//   // 3. PID計算 (目標角度と現在の角度の差を埋める)
//   float outPitch = calculatePID(currentState.pitch, targetState.pitch, pidPitch, dt);
//   float outRoll  = calculatePID(currentState.roll,  targetState.roll,  pidRoll,  dt);
//   float outYaw   = calculatePID(currentState.gyroZ, targetState.yawRate, pidYaw,   dt); // ヨーは角速度で制御

//   // 4. モーター出力へ反映 (前述のミキサー関数を呼ぶ)
//   // ※まだ送信機がないので、テスト時は targetState.throttle を安全な値に固定
//   updateMotorMixer(targetState.throttle, outPitch, outRoll, outYaw);

//   // 5. デバッグ表示 (100msおき)
//   static unsigned long lastLog = 0;
//   if (millis() - lastLog > 100) {
//     lastLog = millis();
//     Serial.printf("P:%.1f R:%.1f Thr:%d\n", currentState.pitch, currentState.roll, targetState.throttle);
//   }
// }

void loop() {
  // 1. 姿勢の更新
  updateAttitude();

  // 2. 気圧センサーからの相対高度
  float relAlt = bmp.readAltitude(1013.25) - baseAltitude;

  // 3. ToFセンサーからの距離取得
  float distanceToF = lox.isRangeComplete() ? lox.readRange() : -1;

  // 4. ToFの傾き補正（キャリブレーション）
  float calibratedToF = -1; // 初期値（無効な状態）
  
  if (distanceToF > 0) {
    // 角度(deg)をラジアン(rad)に変換
    float pitchRad = currentState.pitch * PI / 180.0;
    float rollRad  = currentState.roll  * PI / 180.0;

    // 斜めの距離に、cos(pitch) と cos(roll) を掛けて真の垂直高度を出す
    calibratedToF = distanceToF * cos(pitchRad) * cos(rollRad);
  }

  // 2. 表示処理 (毎ループやると遅いので、100msごとに表示)
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 100) {
    lastPrintTime = millis();

    // 6軸表示 (Accel: g, Gyro: deg/s)
    Serial.print("Acc:"); Serial.print(currentState.accX); Serial.print(","); Serial.print(currentState.accY); Serial.print(","); Serial.print(currentState.accZ);
    Serial.print(" | Gyr:"); Serial.print(currentState.gyroX); Serial.print(","); Serial.print(currentState.gyroY); Serial.print(","); Serial.print(currentState.gyroZ
    
    
    );

    // 姿勢表示
    Serial.print(" | P:"); Serial.print(currentState.pitch); Serial.print(" R:"); Serial.print(currentState.roll); Serial.print(" Y:"); Serial.print(currentState.yaw);

    // 相対高度表示
    float relAlt = bmp.readAltitude(1013.25) - baseAltitude;
    Serial.print(" | BaroRel:"); Serial.print(relAlt);

    // ToF表示
    if (lox.isRangeComplete()) {
      Serial.print(" | ToF:"); Serial.print(lox.readRange()); Serial.print("mm");
    }
    Serial.println();
  }
}


// -----------------------------------------
// setup function (setup の中で1回だけ呼ぶ)
// ------------------------------------------

void calibrateGyro() {
  Serial.println("Calibrating Gyro... DO NOT MOVE!");
  long sx=0, sy=0, sz=0;
  for(int i=0; i<500; i++) {
    Wire.beginTransmission(MPU_ADDR); 
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (size_t)6, true);

    // 【修正箇所】一度 int16_t に入れて、マイナス符号を正しく認識させる！
    int16_t rawX = Wire.read()<<8 | Wire.read();
    int16_t rawY = Wire.read()<<8 | Wire.read();
    int16_t rawZ = Wire.read()<<8 | Wire.read();

    sx += rawX; 
    sy += rawY; 
    sz += rawZ;
    delay(2);
  }
  
  gyro_x_offset = sx / 500.0; 
  gyro_y_offset = sy / 500.0; 
  gyro_z_offset = sz / 500.0;
  
  Serial.print("Gyro Offsets - X:"); Serial.print(gyro_x_offset);
  Serial.print(" Y:"); Serial.print(gyro_y_offset);
  Serial.print(" Z:"); Serial.println(gyro_z_offset);
}

void calibrateAltitude() {
  Serial.println("Calibrating Altitude...");
  float sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += bmp.readAltitude(1013.25);
    delay(20);
  }
  baseAltitude = sum / 50.0;
  Serial.print("Base Altitude set to: "); Serial.println(baseAltitude);
}

void setupMotors() {
  // 全ピン共通のPWM設定
  analogWriteResolution(PWM_RES);
  analogWriteFrequency(PWM_FREQ);

  // 初期値として全モーター停止
  analogWrite(PIN_FR, 0);
  analogWrite(PIN_FL, 0);
  analogWrite(PIN_RL, 0);
  analogWrite(PIN_RR, 0);
  
  Serial.println("Motors Setup Complete.");
}

// 2. センサー読み取りと相補フィルタによる姿勢計算
void updateAttitude() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (size_t)14, true);

  // I2Cから生のデータを読み出して、一時的なローカル変数に入れる
  int16_t AcX = Wire.read()<<8|Wire.read(); 
  int16_t AcY = Wire.read()<<8|Wire.read(); 
  int16_t AcZ = Wire.read()<<8|Wire.read();
  Wire.read(); Wire.read(); // 温度データ（2バイト）は今回は使わないので読み飛ばす
  int16_t GyX = Wire.read()<<8|Wire.read(); 
  int16_t GyY = Wire.read()<<8|Wire.read(); 
  int16_t GyZ = Wire.read()<<8|Wire.read();
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;


  // 6軸物理量へ変換し、構造体に格納
  currentState.accX = AcX / 16384.0; 
  currentState.accY = AcY / 16384.0; 
  currentState.accZ = AcZ / 16384.0;
  
  currentState.gyroX = (GyX - gyro_x_offset) / 131.0; 
  currentState.gyroY = (GyY - gyro_y_offset) / 131.0; 
  currentState.gyroZ = (GyZ - gyro_z_offset) / 131.0;

  // 加速度から求めた「大雑把だけどドリフトしない角度」
  float accPitch = atan2(currentState.accY, sqrt(currentState.accX*currentState.accX + currentState.accZ*currentState.accZ)) * 180 / PI;
  float accRoll  = atan2(-currentState.accX, currentState.accZ) * 180 / PI;

  // 相補フィルタで角度を計算し、構造体に格納（ジャイロの機敏さ 96% ＋ 加速度の正確さ 4%）
  currentState.pitch = 0.96 * (currentState.pitch + currentState.gyroX * dt) + 0.04 * accPitch;
  currentState.roll  = 0.96 * (currentState.roll  + currentState.gyroY * dt) + 0.04 * accRoll;
  currentState.yaw   = currentState.yaw + currentState.gyroZ * dt;
}

// ------------------------------------------
// 駆動関数 (loop の中で毎回呼ぶ)
// ------------------------------------------
void writeMotors(int m1, int m2, int m3, int m4) {
  // 【超重要：安全装置】
  // PIDの計算が暴走してマイナスの値や4095以上の値が出た時に、
  // エラーで落ちないように 0〜4095 の間に強制的に収める (constrain)
  m1 = constrain(m1, 0, 4095);
  m2 = constrain(m2, 0, 4095);
  m3 = constrain(m3, 0, 4095);
  m4 = constrain(m4, 0, 4095);

  analogWrite(PIN_FR, m1);
  analogWrite(PIN_FL, m2);
  analogWrite(PIN_RL, m3);
  analogWrite(PIN_RR, m4);
}

void mixAndWriteMotors(int throttle, int pidPitch, int pidRoll, int pidYaw) {
  // スロットルがゼロ（送信機を一番下にしている）時は強制停止
  if (throttle < 50) {
    writeMotors(0, 0, 0, 0);
    return;
  }

  // ミキサー計算
  int m1 = throttle - pidPitch - pidRoll - pidYaw;
  int m2 = throttle - pidPitch + pidRoll + pidYaw;
  int m3 = throttle + pidPitch + pidRoll - pidYaw;
  int m4 = throttle + pidPitch - pidRoll + pidYaw;

  // 空中では絶対に MIN_THROTTLE を下回らないようにする（アイドルアップ）
  m1 = max(m1, MIN_THROTTLE);
  m2 = max(m2, MIN_THROTTLE);
  m3 = max(m3, MIN_THROTTLE);
  m4 = max(m4, MIN_THROTTLE);

  // 最終出力
  writeMotors(m1, m2, m3, m4);
}

/**
 * モーターミキシング関数
 * throttle: 基本の浮力 (0-4095)
 * p: ピッチ修正量 (PID出力)
 * r: ロール修正量 (PID出力)
 * y: ヨー修正量   (PID出力)
 */
void updateMotorMixer(int throttle, float p, float r, float y) {
  if (throttle < 100) { // スロットルが低すぎるときは安全のため停止
    analogWrite(PIN_FR, 0); analogWrite(PIN_FL, 0);
    analogWrite(PIN_RL, 0); analogWrite(PIN_RR, 0);
    return;
  }

  // X型ドローンの標準ミキサー式
  // ※符号 (+/-) は機体の傾きと回転方向に基づいて決定されます
  int mFR = throttle - p - r - y; // M1: 右前 (CCW)
  int mFL = throttle - p + r + y; // M2: 左前 (CW)
  int mRL = throttle + p + r - y; // M3: 左後 (CCW)
  int mRR = throttle + p - r + y; // M4: 右後 (CW)

  // 出力を 0 ～ 4095 の範囲に収めて書き込み
  analogWrite(PIN_FR, constrain(mFR, MIN_THROTTLE, MAX_THROTTLE));
  analogWrite(PIN_FL, constrain(mFL, MIN_THROTTLE, MAX_THROTTLE));
  analogWrite(PIN_RL, constrain(mRL, MIN_THROTTLE, MAX_THROTTLE));
  analogWrite(PIN_RR, constrain(mRR, MIN_THROTTLE, MAX_THROTTLE));
}

float calculatePID(float current, float target, PIDParameters &p, float dt) {
  float error = target - current;
  
  // P項 (比例)
  float P = p.Kp * error;
  
  // I項 (積分) - 誤差の蓄積
  p.integral += error * dt;
  p.integral = constrain(p.integral, -100, 100); // 溜まりすぎ防止(アンチワインドアップ)
  float I = p.Ki * p.integral;
  
  // D項 (微分) - 変化のブレーキ
  float D = p.Kd * (error - p.error_prev) / dt;
  p.error_prev = error;
  
  return P + I + D;
}