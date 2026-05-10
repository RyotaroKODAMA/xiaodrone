#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_BMP280.h>

#include "calibration_storage.h"

const uint8_t MPU_ADDR = 0x68;

int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float pitch_offset = 0.0f;
float roll_offset = 0.0f;
float gyro_x_offset = 0.0f;
float gyro_y_offset = 0.0f;
float gyro_z_offset = 0.0f;
float referencePressure = 101325.0f;

Adafruit_BMP280 bmp;

void readRawMPU();

bool detectUpsideDown() {
  long summedZ = 0;
  const int samples = 50;

  for (int i = 0; i < samples; i++) {
    readRawMPU();
    summedZ += AcZ;
    delay(2);
  }

  return summedZ < 0;
}

void getNormalizedAccel(bool upsideDown, float &ax, float &ay, float &az) {
  ax = (float)AcX;
  ay = (float)AcY;
  az = (float)AcZ;

  if (upsideDown) {
    ax = -ax;
    ay = -ay;
    az = -az;
  }
}

void readRawMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (size_t)14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

void calibrateGyro() {
  long sx = 0;
  long sy = 0;
  long sz = 0;

  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (size_t)6, true);
    sx += (int16_t)(Wire.read() << 8 | Wire.read());
    sy += (int16_t)(Wire.read() << 8 | Wire.read());
    sz += (int16_t)(Wire.read() << 8 | Wire.read());
    delay(2);
  }

  gyro_x_offset = sx / 500.0f;
  gyro_y_offset = sy / 500.0f;
  gyro_z_offset = sz / 500.0f;
}

void calibrateLevel() {
  Serial.println("Level Calibrating... PLEASE WAIT 2 SECONDS");
  delay(2000);

  const bool upsideDown = detectUpsideDown();
  if (upsideDown) {
    Serial.println("Upside-down placement detected. Compensating before calculating offsets.");
  } else {
    Serial.println("Normal placement detected.");
  }

  float pitchSum = 0.0f;
  float rollSum = 0.0f;
  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    readRawMPU();
    float accX = 0.0f;
    float accY = 0.0f;
    float accZ = 0.0f;
    getNormalizedAccel(upsideDown, accX, accY, accZ);

    float rawAccP = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180 / PI;
    float rawAccR = atan2(-accX, accZ) * 180 / PI;

    pitchSum += rawAccP;
    rollSum += rawAccR;
    delay(2);
  }

  pitch_offset = pitchSum / samples;
  roll_offset = rollSum / samples;
}

float measureReferencePressure() {
  float totalPressure = 0.0f;
  const int samples = 100;

  for (int i = 0; i < samples; i++) {
    totalPressure += bmp.readPressure();
    delay(10);
  }

  return totalPressure / samples;
}

void setup() {
  Serial.begin(921600);
  Wire.begin(5, 6);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found!");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("Calibration mode started");
  Serial.println("Keep the frame level and still. Upside-down placement is supported.");
  delay(2000);

  calibrateGyro();
  calibrateLevel();
  referencePressure = measureReferencePressure();

  CalibrationStorage::Data data = CalibrationStorage::makeDefault();
  data.gyro_x_offset = gyro_x_offset;
  data.gyro_y_offset = gyro_y_offset;
  data.gyro_z_offset = gyro_z_offset;
  data.pitch_offset = pitch_offset;
  data.roll_offset = roll_offset;
  data.referencePressure = referencePressure;

  if (CalibrationStorage::save(data)) {
    Serial.println("Calibration saved to NVS");
    Serial.printf("gyro: %.3f %.3f %.3f\n", gyro_x_offset, gyro_y_offset, gyro_z_offset);
    Serial.printf("level: pitch %.3f roll %.3f\n", pitch_offset, roll_offset);
    Serial.printf("pressure: %.2f Pa\n", referencePressure);
  } else {
    Serial.println("Failed to save calibration data");
  }
}

void loop() {
  delay(1000);
}