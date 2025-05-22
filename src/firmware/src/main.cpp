#include <Arduino.h>
#include <pins_arduino.h>
#include <ArduinoJson.h>
#include "Encoder.hpp"
#include "Motor.hpp"

DEFINE_ENCODER_ISR(leftEncoder, 2, A2)
DEFINE_ENCODER_ISR(rightEncoder, 3, A0)

Motor* leftMotor = nullptr;
Motor* rightMotor = nullptr;

void setup() {
  Serial.begin(9600);
  setup_leftEncoder();
  setup_rightEncoder();

  leftMotor = new Motor(leftEncoder, 1, 10, 11);
  rightMotor = new Motor(rightEncoder, 1, 5, 6);
};

void loop() {
  if (Serial.available()) {
    String json = Serial.readStringUntil('\n');

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);
    JsonDocument resp;
    
    if (!err) {
      String cmd = doc["cmd"];
      resp["resp"] = cmd;
      
      if (cmd == "GET") {
        Encoder* encoder = doc["motor"] == "LEFT" ? leftEncoder: rightEncoder;
        String property = doc["property"];
        
        if (property == "ANGULARVEL") {
          resp["value"] = encoder->getAngularVel();
        } else if (property == "ANGLE") {
          resp["value"] = encoder->getAngle();
        }
      } else if (cmd == "SET") {
        Motor* motor = doc["target"] == "LEFT" ? leftMotor: rightMotor;
        String property = doc["property"];

        if (property == "ANGULARVEL") {
          motor->setTargetAngVel(doc["value"]);
          resp["value"] = "OKAY";
        }
      } else if (cmd == "STATUS") {
        resp["value"] = "OKAY";
      } else {
        resp["value"] = "BAD CMD " + cmd;
      }

    } else {
      resp["resp"] = "UNKNOWN";
      resp["value"] = "BAD";
    }
    serializeJson(resp, Serial);
    Serial.println();
  }
  leftMotor->update();
  rightMotor->update();
};
