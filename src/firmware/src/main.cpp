#include <Arduino.h>
#include <Encoder.hpp>
#include <Motor.hpp>
#include <pins_arduino.h>

DEFINE_ENCODER_ISR(leftEncoder, A0, A1)
DEFINE_ENCODER_ISR(rightEncoder, A2, A3)

Motor* leftMotor = nullptr;
Motor* rightMotor = nullptr;

void setup() {
  Serial.begin(115200);
  // setup_leftEncoder();
  // setup_rightEncoder();

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  // leftMotor = new Motor(leftEncoder, 1, 10, 11);
  // rightMotor = new Motor(rightEncoder, 1, 5, 6);
};

void loop() {
  digitalWrite(5, LOW);
  for (int i=0; i<255; i++) {
    analogWrite(6, i);
    delay(10);
  }
  // Serial.print("Angle");
  // Serial.print(leftEncoder->getAngle());
  // Serial.print(",right angle");
  // Serial.print(rightEncoder->getAngle());
  // Serial.print("\n");
};
