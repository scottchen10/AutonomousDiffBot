#include <Arduino.h>
#include <pins_arduino.h>
#include <ArduinoJson.h>
#include <string.h>

#include "Encoder.hpp"
#include "Motor.hpp"

const String BAD_CMD = String("resp:bad-cmd");

Encoder* leftEncoder = nullptr;   
Encoder* rightEncoder = nullptr;                                                   
Motor* leftMotor = nullptr;
Motor* rightMotor = nullptr;

void isr_leftEncoder()  {                                                        
  if (leftEncoder) leftEncoder->onInterruptUpdate();                                           
}                      

void isr_rightEncoder()  {                                                        
  if (rightMotor) rightEncoder->onInterruptUpdate();                                           
}               

// cmd:get_motor r angle
// cmd:get_motor r angular_vel
// cmd:set_motor r angular_vel 203.3
// cmd:status

// resp:angle r 102.0
// resp:angular_vel r 2.0
// resp:set_motor okay
// resp:status okay
// resp:bad-cmd

bool getNextToken(char** token, String *dest) {
  (*token) = strtok(NULL, " ");
  if (not token)
    return false;
  (*dest) = String(*token);
  return true;
}

String cmd_get_motor(char * token) {
  String side;
  String property;

  Encoder * encoder = nullptr;
  Motor* motor = nullptr;

  if (!getNextToken(&token, &side) || !getNextToken(&token, &property))
    return BAD_CMD;

  if (side.equals("l")) {
    encoder = leftEncoder;
    motor = leftMotor;
  } else if (side.equals("r")) {
    encoder = rightEncoder;
    motor = rightMotor;
  } else {
    return BAD_CMD;
  }

  String respHeader = String("resp:") + property + String(" ") + side + String(" ");

  if (property.equals("angle")) {
    return respHeader + String(encoder->getAngle(), 3);
  } else if (property.equals("angular_vel")) {
    return respHeader + String(encoder->getAngularVel(), 3);
  } else if (property.equals("angle_angular_vel")) {
    return respHeader
      + String(encoder->getDeltaAngle(), 3) + String(" ")
      + String(encoder->getAngularVel(), 3);
  } else if (property.equals("angular_vel")) {
    return respHeader + String(encoder->getAngularVel(), 3);
  } else if (property.equals("abs_angle")) {
    return respHeader + String(encoder->getDeltaAngle(), 3);
  } else if (property.equals("error")) {
    PIDController* pidcontrol = motor->motorPid;

    return respHeader
      + String(pidcontrol->error, 3) + String(" ") 
      + String(pidcontrol->integralError, 3) + String(" ") 
      + String(pidcontrol->derivativeError, 3);
  }

  return BAD_CMD;
}


String cmd_set_motor(char * token) {
  String side;
  String property;
  String value;

  Motor * motor = nullptr;

  if (!getNextToken(&token, &side) || !getNextToken(&token, &property) || !getNextToken(&token, &value))
    return String("resp:malformed-cmd");

  if (side.equals("l")) {
    motor = leftMotor;
  } else if (side.equals("r")) {
    motor = rightMotor;
  } else {
    return BAD_CMD;
  }

  if (property.equals("angular_vel")) {
    motor->setTargetAngVel(value.toDouble());
    return String("resp:okay");
  } else if (property.equals("tuning")) {
    String ki;
    String kd;

    if (not getNextToken(&token, &ki))
      return BAD_CMD;
    if (not getNextToken(&token, &kd))
      return BAD_CMD;

    motor->motorPid->updateTuning(value.toDouble(), ki.toDouble(), kd.toDouble());
    return String("resp:okay");
  }
  return BAD_CMD;
}

String parseCommand(String command) {
  char copy[50];
  char * token = nullptr;
  strncpy(copy, command.c_str(), sizeof(copy));
  token = strtok(copy, " ");

  if (strcmp(token, "get_motor") == 0) {
    return cmd_get_motor(token);
  } else if (strcmp(token, "set_motor") == 0) {
    return cmd_set_motor(token);
  } else if (strcmp(token, "status") == 0) {
    return String("resp:okay");
  }

  return BAD_CMD;
}

void setup() {
  Serial.begin(115200);

  leftEncoder = new Encoder(2, A2);                              
  rightEncoder = new Encoder(3, A3);     
  leftEncoder->setup();
  rightEncoder->setup();

  leftMotor = new Motor(leftEncoder, 11, 10);
  rightMotor = new Motor(rightEncoder, 6, 5);
  leftMotor->setup();
  rightMotor->setup();

  attachInterrupt(digitalPinToInterrupt(leftEncoder->pinPulseA), isr_leftEncoder, RISING); 
  attachInterrupt(digitalPinToInterrupt(rightEncoder->pinPulseA), isr_rightEncoder, RISING); 
};

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');

    if (command.startsWith("cmd")) {
      command = command.substring(4);
      String result = parseCommand(command);

      if (result == BAD_CMD) {
        result = BAD_CMD + " " + command;
      }
      Serial.println(result);
    } else {
      Serial.println(BAD_CMD + " " + command);
    }   
  }

  leftEncoder->update();
  rightEncoder->update();

  leftMotor->update();
  rightMotor->update();
};
