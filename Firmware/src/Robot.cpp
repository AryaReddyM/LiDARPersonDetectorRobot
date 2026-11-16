#include <Arduino.h>
#include "CytronMotorDriver.h"
#include "Bluepad32.h"
#include <ESP32Encoder.h>

#define PWM1 18
#define DIR1 19
#define PWM2 22
#define DIR2 23

#define MAX_WHEEL_SPEED 13.6
#define WATCHDOG_MS 300
#define JOY_PERIOD_MS 20
#define ENCODER_PERIOD_MS 20

#define LEFT_ENCODER_A 32
#define LEFT_ENCODER_B 33
#define RIGHT_ENCODER_A 25
#define RIGHT_ENCODER_B 26

CytronMD motorL(PWM_DIR, PWM1, DIR1);
CytronMD motorR(PWM_DIR, PWM2, DIR2);
GamepadPtr gamepad;

unsigned long lastJoySend = 0;
unsigned long lastVTime = 0;
unsigned long lastEncoderSend = 0;
String rxLine;

ESP32Encoder leftEncoder;
ESP32Encoder rightEncoder;

void OnConnectedGamepad(GamepadPtr gp) {
    Serial.println("# Gamepad connected");
    gamepad = gp;
}

void OnDisconnectedGamepad(GamepadPtr gp) {
    Serial.println("# Gamepad disconnected");
    gamepad = nullptr;
    motorL.setSpeed(0);
    motorR.setSpeed(0);
}

void handleLine(const String& line) {
  float left, right;
  if (sscanf(line.c_str(), "V %f %f", &left, &right) == 2) {
    int speedL = constrain((int)(left  / MAX_WHEEL_SPEED * 255), -255, 255);
    int speedR = constrain((int)(right / MAX_WHEEL_SPEED * 255), -255, 255);

    motorL.setSpeed(speedL);
    motorR.setSpeed(speedR);
    lastVTime = millis();
  }
}

void setup() {
  Serial.begin(115200);
  analogWriteFrequency(20000);

  BP32.setup(&OnConnectedGamepad, &OnDisconnectedGamepad);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  leftEncoder.attachSingleEdge(LEFT_ENCODER_A, LEFT_ENCODER_B);
  rightEncoder.attachSingleEdge(RIGHT_ENCODER_A, RIGHT_ENCODER_B);

  leftEncoder.clearCount();
  rightEncoder.clearCount();
}

void loop() {
  BP32.update();
 
  while (Serial.available() > 0) {
    char c = Serial.read();
 
    if (c == '\n') {
      handleLine(rxLine);
      rxLine = "";
    } else if (c != '\r') {
      rxLine += c;
      if (rxLine.length() > 64) rxLine = "";
    }
  }
 
  if (gamepad && gamepad->isConnected() && millis() - lastJoySend >= JOY_PERIOD_MS) {
    lastJoySend = millis();
 
    float leftY = -(gamepad->axisY()  / 512.0f);
    float rightX = (gamepad->axisRX() / 512.0f);
 
    Serial.print("J ");
    Serial.print(leftY, 3);
    Serial.print(" ");
    Serial.println(rightX, 3);
  }

  if (millis() - lastEncoderSend >= ENCODER_PERIOD_MS) {
    lastEncoderSend = millis();

    Serial.print("E ");
    Serial.print(-leftEncoder.getCount());
    Serial.print(" ");
    Serial.print(rightEncoder.getCount());
    Serial.print(" ");
    Serial.println(millis());
  }
 
  if (millis() - lastVTime > WATCHDOG_MS) {
    motorL.setSpeed(0);
    motorR.setSpeed(0);
  }
}

// L = 0 - 471
// R = 