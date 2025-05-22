#pragma once

#include <Arduino.h>
#include "Encoder.hpp"
#include "PIDControl.hpp"

class Motor {
public:
    void setTargetAngVel(double angVel);
    void setup();
    void update();

    void setMotorPower(int16_t power);
    Motor(Encoder* encoder, double wheelRadius, uint8_t pinInputA, uint8_t pinInputB);

private:
    double wheelRadius;
    double cmdAngVel = 0;
    Encoder* encoder;
    PIDController* motorPid; 

    uint8_t pinInputA;
    uint8_t pinInputB;

};