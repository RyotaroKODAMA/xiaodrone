#include <Arduino.h>
#include <Wire.h>
#include <math.h> 
#include <Adafruit_BMP280.h>
#include "Adafruit_VL53L0X.h"
#include "sbus.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "calibration_storage.h"

WiFiUDP udp;
// const char* pc_ip = "192.168.179.56"; // PCのIPアドレス
const char* pc_ip = "192.168.179.40"; // PCのIPアドレス
const int udp_port = 22222;



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

struct TargetState {
  float yawRateTarget; // ヨーの回転速度目標
  float pitch;         // ピッチ目標角度
  float roll;          // ロール目標角度
  float throttleRaw;   // SBUSから来た生のスロットル値
  float altitudeTarget; // 高度制御用の目標高さ(m)
  float GainScalingfactor; // ゲインスケーリング係数
};

TargetState targetState = {0, 0, 0, 0, 0, 1.0}; // 全て0で初期化

// --- グローバル変数 ---
DroneState currentState; 
// DroneSetpoint targetState;
const uint8_t MPU_ADDR = 0x68;

// ゼロ点オフセット
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
float pitch_offset = 0, roll_offset = 0, yaw_offset = 0;

// Kp=20 に対して、まずは 1/20 くらいの 1.0 あたりから試すのが 12bit では現実的です
// PIDParameters pidRoll  = { 20.0, 0.0, 1.0, 0, 0 }; 
// PIDParameters pidPitch = { 20.0, 0.0, 1.0, 0, 0 };
// PIDParameters pidYaw   = { 20.0, 0.0, 0.0, 0, 0 }; // ヨーは一旦 0 で OK
// PIDParameters pidAltitude = { 300.0, 100.0, 30.0, 0, 0 }; // 高度制御用PID (抑制強化版)

// scaleもとになるゲイン。これにSbusのゲインを掛けて最終的なKpなどを決定するイメージ
PIDParameters pidRoll  = { 2.0, 0.0, 0.1, 0, 0 }; 
PIDParameters pidPitch = { 2.0, 0.0, 0.1, 0, 0 };
PIDParameters pidYaw   = { 2.0, 0.0, 0.0, 0, 0 }; // ヨーは一旦 0 で OK
PIDParameters pidAltitude = { 300.0, 100.0, 30.0, 0, 0 }; // 高度制御用PID (抑制強化版)







// モーターピン
const int PIN_FR = 1, PIN_FL = 8, PIN_RL = 9, PIN_RR = 4;
const int PWM_FREQ = 16000;
const int PWM_RES = 12;

// 【修正】検証用の厳しいリミッター
const int MIN_THROTTLE = 0;    
const int MAX_THROTTLE = 4000;  // 最大を4000に制限

Adafruit_BMP280 bmp;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool tofReady = false;
bool emergencyKill = false;
unsigned long lastSbusDataMs = 0;
const unsigned long SBUS_TIMEOUT_MS = 200;

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
bool loadCalibrationFromStorage();
void updateAttitude(float dt);
float calculatePID(float current, float target, PIDParameters &p, float scaling, float dt);
void updateMotorMixer(int throttle, float p, float r, float y);
void updateAltitude();
float calculateAltitudeFromPressure(float pressure);
float calculateToFAltitudeWithAttitude(float distanceMm, float pitchDeg, float rollDeg);
void updateSBUS();
void debugLog(const char* format, ...);
// 1. 関数の外（グローバル）に宣言
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float throttle; // スロットル値をグローバルに宣言

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
  targetState.throttleRaw = 0;
  throttle = 0;
}

/* SBUS object, reading SBUS */
const int SBusPin = 3;
bfs::SbusRx sbus_rx(&Serial1, SBusPin, 43, true);
bfs::SbusData data;

// SBUS(368-1680) を 12bit(0-4095) に変換する関数
uint16_t sbusTo12bit(uint16_t val) {
    val = constrain(val, 368, 1680);
    return (uint16_t)map(val, 368, 1680, 0, 4095);
}

// ゲインチューニング用に 0.0 ~ 20.0 などのfloatに変換する関数
float sbusToGain(uint16_t val, float maxGain) {
    val = constrain(val, 144, 1904);
    return ((float)(val - 144) / (1904 - 144)) * maxGain;
}

bool loadCalibrationFromStorage() {
  CalibrationStorage::Data data{};
  if (!CalibrationStorage::load(data)) {
    return false;
  }

  gyro_x_offset = data.gyro_x_offset;
  gyro_y_offset = data.gyro_y_offset;
  gyro_z_offset = data.gyro_z_offset;
  pitch_offset = data.pitch_offset;
  roll_offset = data.roll_offset;

  if (data.referencePressure > 0.0f) {
    referencePressure = data.referencePressure;
  }

  return true;
}



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



  // WiFi接続
  WiFi.begin("SPWH_L12_015d3d", "9026eabeadfa7");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");Serial.println(WiFi.localIP());
  Serial.println("\nUDP Ready");



  // 3. センサー初期化
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission(); // DLPF 10Hz
  
  // 【ここを追加】加速度センサーのレンジを ±8g に変更 (0x1C レジスタに 0x10 を書き込む)
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();


  // BMP280初期化
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found!");
    while (1);
  }

  if (loadCalibrationFromStorage()) {
    Serial.println("Calibration loaded from NVS");
  } else {
    Serial.println("Calibration data not found. Using default offsets.");
  }
  
  Serial.println("Stabilizing...");
  delay(1000);

  // ToF初期化
  if (lox.begin()) {
    lox.startRangeContinuous();
    tofReady = true;
    Serial.println("VL53L0X ready");
  } else {
    tofReady = false;
    Serial.println("VL53L0X not found, fallback to barometer only");
  }
  
  // 保存済みの基準気圧を使って高度を計算する
  filteredAltitude = 0.0;  // 初期高度を0に固定
  debugLog("Reference Pressure: %.2f Pa\n", referencePressure);

  // // シリアルバッファ掃除
  // while(Serial.available() > 0) Serial.read();
  // Serial.println("=== Commands ===");
  // Serial.println("w: Altitude +0.01m, x: Altitude -0.01m, q: Reset Altitude");
  // Serial.println("k: KILL - Force stop all motors immediately!");
  // Serial.println("u: Release KILL and resume control");
  // Serial.println("Note: Hovering throttle is automatically determined by PID integral.");

  // begin Sbus
  sbus_rx.Begin();
  // --- SBUS quick test: 500ms window, print once if data received ---
  {
    bool sbus_ok = false;
    for (int i = 0; i < 50; i++) { // 50 * 10ms = 500ms
      if (sbus_rx.Read()) {
        data = sbus_rx.data();
        Serial.println("SBUS: OK");
        for (int8_t ch = 0; ch < data.NUM_CH; ch++) {
          Serial.print(data.ch[ch]);
          Serial.print('\t');
        }
        Serial.print(data.lost_frame);
        Serial.print('\t');
        Serial.println(data.failsafe);
        sbus_ok = true;
        break;
      }
      delay(10);
    }
    if (!sbus_ok) {
      Serial.println("SBUS: No data (check wiring/baud)");
    }
  }



}


void loop() {
  // 1. 受信（独立関数）
  updateSBUS();

  // if (emergencyKill) {
  //   throttle = 0;
  //   stopAllMotors();
  //   return;
  // }

  // 2. タイミング管理
  static unsigned long lastLoopTime = micros();
  unsigned long now = micros();
  float dt = (now - lastLoopTime) / 1000000.0;
  // keep loop time
  if (dt < 0.004) return; 
  lastLoopTime = now;

  // get drone state from sensors
  updateAltitude();
  updateAttitude(dt);

  // scale PID parameter
  // PIDParameters {
  //   pidRoll.Kp * targetState.GainScalingfactor, pidRoll.Ki * targetState.GainScalingfactor, pidRoll.Kd * targetState.GainScalingfactor, 
  //   0, 0
  // };
  // PIDParameters

  throttle = targetState.throttleRaw; // 3chのスロットル値をそのまま使用

  // // スロットル変化率リミッター（急激な上下動を防止）
  // if (throttle > lastThrottle + throttleLimitRate) {
  //   throttle = lastThrottle + throttleLimitRate;
  // } else if (throttle < lastThrottle - throttleLimitRate) {
  //   throttle = lastThrottle - throttleLimitRate;
  // }
  // lastThrottle = throttle;
  
  // controll for pitch and roll state using PID
  float outP = calculatePID(currentState.pitch, targetState.pitch, pidPitch,targetState.GainScalingfactor, dt);
  float outR = calculatePID(currentState.roll,  targetState.roll,  pidRoll,  targetState.GainScalingfactor, dt);

  float Kpp = pidPitch.Kp * targetState.GainScalingfactor;
  float Kpr = pidRoll.Kp * targetState.GainScalingfactor;


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
    debugLog("ATTITUDE | Pitch:%6.1f deg | Roll:%6.1f deg\n", currentState.pitch, currentState.roll);
    
    // 2段目：PID計算結果
    debugLog("PID OUT  | OutP:%7.1f | OutR:%7.1f | (KpP:%.1f, KpR:%.1f)\n", outP, outR, Kpp, Kpr);
    
    // 3段目：高度情報
    debugLog("ALTITUDE | Current:%.2f m | Target:%.2f m | Baro:%.2f m | ToF:%.2f m | Pressure:%.0f Pa\n", 
            currentState.altitudeBaro, targetState.altitudeTarget, 
            filteredAltitude, currentState.altitudeToF, bmp.readPressure());
    
    // 4段目：各モーターへの最終PWM値 (12bit: 0-4095)
    debugLog("MOTORS   | FR:%4d | FL:%4d | RL:%4d | RR:%4d | Thr:%d\n", 
                  constrain(mFR, 0, 4095), constrain(mFL, 0, 4095), 
                  constrain(mRL, 0, 4095), constrain(mRR, 0, 4095), 
                  throttle);
  }
}

void updateAttitude(float dt) {
  readRawMPU();
  // 100msに1回だけ、生のI2C通信データを覗き見する
  static unsigned long lastRawLog = 0;
  if (millis() - lastRawLog > 100) {
      lastRawLog = millis();
      // GyXやAcXが、いきなり 30000 などの異常値になっていないか確認する
      debugLog("RAW SENSOR | GyX:%6d | GyY:%6d | AcX:%6d | AcY:%6d\n", GyX, GyY, AcX, AcY);
  }

  float temp = GyX; GyX = GyY; GyY = -temp;

  // ★重要：ジャイロを dps に変換
  currentState.gyroX = (GyX - gyro_x_offset) / 131.0;
  currentState.gyroY = (GyY - gyro_y_offset) / 131.0;
  currentState.gyroZ = (GyZ - gyro_z_offset) / 131.0;

  // ジャイロLPF：モーター振動ノイズを減らすため強力なフィルター
  // gyroAlphaが小さいほど過去の値を重視してノイズを減衰
  static float filteredGyX = 0, filteredGyY = 0, filteredGyZ = 0;
  float gyroAlpha = 0.1; // 0.1に強化（デフォルト0.3から変更）

  filteredGyX = (1.0 - gyroAlpha) * filteredGyX + gyroAlpha * ((GyX - gyro_x_offset) / 131.0);
  filteredGyY = (1.0 - gyroAlpha) * filteredGyY + gyroAlpha * ((GyY - gyro_y_offset) / 131.0);
  filteredGyZ = (1.0 - gyroAlpha) * filteredGyZ + gyroAlpha * ((GyZ - gyro_z_offset) / 131.0);

  currentState.gyroX = filteredGyX;
  currentState.gyroY = filteredGyY;
  currentState.gyroZ = filteredGyZ;

  // 加速度LPF
  // lpfAccX = (1.0 - lpfBeta) * lpfAccX + lpfBeta * (AcX / 16384.0);
  // lpfAccY = (1.0 - lpfBeta) * lpfAccY + lpfBeta * (AcY / 16384.0);
  // lpfAccZ = (1.0 - lpfBeta) * lpfAccZ + lpfBeta * (AcZ / 16384.0);
  // 加速度LPF (8Gレンジ用に 4096.0 で割る)
  lpfAccX = (1.0 - lpfBeta) * lpfAccX + lpfBeta * (AcX / 4096.0);
  lpfAccY = (1.0 - lpfBeta) * lpfAccY + lpfBeta * (AcY / 4096.0);
  lpfAccZ = (1.0 - lpfBeta) * lpfAccZ + lpfBeta * (AcZ / 4096.0);


  // 1. まず標準的な計算式に直す（一般的な航空力学の軸）
  float accPitch = (atan2(-lpfAccX, sqrt(lpfAccY*lpfAccY + lpfAccZ*lpfAccZ)) * 180 / PI) - pitch_offset;
  float accRoll  = (atan2(lpfAccY, lpfAccZ) * 180 / PI) - roll_offset;

  // 2. 同士を素直に掛け合わせる
  currentState.pitch = 0.98 * (currentState.pitch + currentState.gyroY * dt) + 0.02 * accPitch;
  currentState.roll  = 0.98 * (currentState.roll  + currentState.gyroX * dt) + 0.02 * accRoll;
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

float calculatePID(float current, float target, PIDParameters &p, float scaling, float dt) {
  float error = target - current;
  float P = p.Kp * error * scaling;
  p.integral += error * dt;
  // アンチウインドアップ：積分項を厳しく制限してスロットル暴走を防ぐ
  p.integral = constrain(p.integral, -5, 5);
  float I = p.Ki * p.integral * scaling;
  float D = p.Kd * (error - p.error_prev) * scaling / dt;
  p.error_prev = error;
  return P + I + D;
}



void updateSBUS() {
  if (sbus_rx.Read()) {
    lastSbusDataMs = millis();
    data = sbus_rx.data();

    // SBUSライブラリでは、信号欠落や無効チャネルを示すために
    // チャネル値に 2047 (11bit の全1) を入れることがあります。
    // もし 2047 が来たらフェイルセーフ扱いにしてモータ停止します。
    bool sbus_invalid = false;
    for (int i = 0; i < 16; ++i) {
      if (data.ch[i] == 2047) { sbus_invalid = true; break; }
    }
    if (sbus_invalid) {
      Serial.println("SBUS: invalid channel value 2047 detected -> entering failsafe");
      emergencyKill = true;
      stopAllMotors();
      return;
    }

    // フェイルセーフ（電波途絶）時の処理
    if (data.failsafe) {
      emergencyKill = true;
      stopAllMotors();
      return;
    }

    // --- モード1割り当て ---
    
    // 0ch: ヨー (Yaw Rate) -> -180〜180 deg/s 程度にマッピング
    int rawYaw = sbusTo12bit(data.ch[0]);
    targetState.yawRateTarget = map(rawYaw, 0, 4095, -180, 180);

    // 1ch: エレベーター (Pitch) -> 前後傾き目標 -30〜30 deg
    int rawPitch = sbusTo12bit(data.ch[1]);
    targetState.pitch = map(rawPitch, 0, 4095, 30, -30); // モード1なら下方向がプラス（前傾）

    // 2ch: スロットル (Altitude/Power) -> 0〜4095 (12bit)
    // ※ 高度制御に使うか直接スロットルにするかは、後のPID計算で利用
    uint16_t rawThrottle = sbusTo12bit(data.ch[2]);
    targetState.throttleRaw = rawThrottle; 

    // 3ch: エルロン (Roll) -> 左右傾き目標 -30〜30 deg
    int rawRoll = sbusTo12bit(data.ch[3]);
    targetState.roll = map(rawRoll, 0, 4095, -30, 30);

    // 6ch: ゲインスケーリング係数
    float maximum_gain = 80.0; // 例: 最大ゲインを2.0に設定
    targetState.GainScalingfactor = sbusToGain(data.ch[6], maximum_gain);
    
    // デッドゾーン処理 (スティック中央の遊び: 12bit中央は2048)
    if (abs(rawRoll - 2048) < 100) targetState.roll = 0;
    if (abs(rawPitch - 2048) < 100) targetState.pitch = 0;
    if (abs(rawYaw - 2048) < 100) targetState.yawRateTarget = 0;
  } else if (millis() - lastSbusDataMs > SBUS_TIMEOUT_MS) {
    emergencyKill = true;
    stopAllMotors();
  }
}


void debugLog(const char* format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  // 1. USBシリアルに出力
  Serial.print(buffer);

  // 2. UDPでPCに送信
  if (WiFi.status() == WL_CONNECTED) {
    udp.beginPacket(pc_ip, udp_port);
    udp.write((uint8_t*)buffer, strlen(buffer));
    udp.endPacket();
  }
}










// 気圧から高度を計算する関数
// 国際標準大気モデルを使用
float calculateAltitudeFromPressure(float pressure) {
  // h = 44330 * (1 - (P/P0)^(1/5.255))
  float ratio = pressure / referencePressure;
  float altitude = 44330.0 * (1.0 - pow(ratio, 1.0 / 5.255));
  return altitude;
}

float calculateToFAltitudeWithAttitude(float distanceMm, float pitchDeg, float rollDeg) {
  if (distanceMm <= 0.0f) {
    return -1.0f;
  }

  float pitchRad = pitchDeg * PI / 180.0f;
  float rollRad = rollDeg * PI / 180.0f;
  float correction = cos(pitchRad) * cos(rollRad);

  // 斜め姿勢での過補正を防ぐため、補正係数を下限付きで扱う
  correction = constrain(correction, 0.2f, 1.0f);

  return (distanceMm / 1000.0f) * correction;
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
        tofAltitude = calculateToFAltitudeWithAttitude(distanceMm, currentState.pitch, currentState.roll);
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


