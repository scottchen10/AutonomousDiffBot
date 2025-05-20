#include <Arduino.h>
#include <pins_arduino.h>
#include <ArduinoJson.h>
#include "Encoder.hpp"
#include "Motor.hpp"

DEFINE_ENCODER_ISR(leftEncoder, 2, A1)
DEFINE_ENCODER_ISR(rightEncoder, 3, A3)

Motor* leftMotor = nullptr;
Motor* rightMotor = nullptr;

void setup() {
  Serial.begin(115200);
  setup_leftEncoder();
  setup_rightEncoder();

  leftMotor = new Motor(leftEncoder, 1, 10, 11);
  rightMotor = new Motor(rightEncoder, 1, 5, 6);
};



void loop() {
  if (Serial.available()) {
    String json = Serial.readStringUntil('\n');

    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, json);

    if (!err) {
      String cmd = doc["cmd"];
      StaticJsonDocument<128> resp;
      resp["resp"] = cmd;
      
      if (cmd == "GET") {
        Encoder* encoder = doc["motor"] == "LEFT" ? leftEncoder: rightEncoder;
        String property = doc["property"];
        
        if (property == "ANGULARVEL") {
          resp["value"] = encoder->getAngularVel();
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
        resp["value"] = "BAD CMD";
      }


      serializeJson(resp, Serial);
      Serial.println();
    }
  }

  leftMotor->update();
  rightMotor->update();
};
