#include <Arduino.h>

constexpr uint8_t MOTOR_PINS[] = {4, 81 , 9, 1};
constexpr uint8_t MOTOR_CHANNELS[] = {0, 1, 2, 3};
constexpr uint8_t MOTOR_COUNT = sizeof(MOTOR_PINS) / sizeof(MOTOR_PINS[0]);
constexpr int PWM_FREQ = 16000;
constexpr int PWM_RESOLUTION = 10;
constexpr int PWM_MAX = (1 << PWM_RESOLUTION) - 1;
constexpr int THROTTLE_STEP = 80;

int motorThrottle[MOTOR_COUNT] = {0, 0, 0, 0};

void setupMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    ledcSetup(MOTOR_CHANNELS[i], PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PINS[i], MOTOR_CHANNELS[i]);
    ledcWrite(MOTOR_CHANNELS[i], 0);
  }
}

void writeMotor(uint8_t motorIndex, int throttle) {
  throttle = constrain(throttle, 0, PWM_MAX);


  motorThrottle[motorIndex] = throttle;

  if (motorIndex < MOTOR_COUNT) {
    ledcWrite(MOTOR_CHANNELS[motorIndex], throttle);
  }
}

void stopAllMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    ledcWrite(MOTOR_CHANNELS[i], 0);
  }
}

void handleSerialCommand(String command) {
  command.trim();
  if (command.length() == 0) {
    return;
  }

  command.toLowerCase();

  if (command == "stop" || command == "all 0" || command == "all stop") {
    stopAllMotors();
    Serial.println("all motors stopped");
    return;
  }

  int motorIndex = -1;
  int amount = THROTTLE_STEP;

  if (sscanf(command.c_str(), "%d %*s %d", &motorIndex, &amount) >= 1) {
    motorIndex -= 1;
    if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) {
      Serial.println("motor index must be 1-4");
      return;
    }

    if (command.indexOf("up") >= 0 || command.indexOf('+') >= 0) {
      writeMotor(static_cast<uint8_t>(motorIndex), motorThrottle[motorIndex] + amount);
      Serial.printf("motor=%d throttle=%d\n", motorIndex + 1, motorThrottle[motorIndex]);
      return;
    }

    if (command.indexOf("down") >= 0 || command.indexOf('-') >= 0) {
      writeMotor(static_cast<uint8_t>(motorIndex), motorThrottle[motorIndex] - amount);
      Serial.printf("motor=%d throttle=%d\n", motorIndex + 1, motorThrottle[motorIndex]);
      return;
    }

    if (sscanf(command.c_str(), "%d %d", &motorIndex, &amount) == 2) {
      motorIndex -= 1;
      if (motorIndex >= 0 && motorIndex < MOTOR_COUNT) {
        writeMotor(static_cast<uint8_t>(motorIndex), amount);
        Serial.printf("motor=%d throttle=%d\n", motorIndex + 1, motorThrottle[motorIndex]);
        return;
      }
    }
  }

  Serial.println("usage: <motor 1-4> up [step], <motor 1-4> down [step], <motor 1-4> <throttle>, or stop");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  setupMotors();
  Serial.println("Motor test start");
  Serial.println("Send: <motor 1-4> up [step], <motor 1-4> down [step], or <motor 1-4> <throttle>");
  Serial.println("Example: 1 up 80, 2 down 40, 3 1200");
  Serial.println("Stop all: stop");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    handleSerialCommand(command);
  }
}