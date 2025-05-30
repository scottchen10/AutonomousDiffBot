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
    void setMotorPower(int16_t power, bool brake);

    double wheelRadius;
    PIDController* motorPid; 
    uint8_t pinInputA;
    uint8_t pinInputB;

    Motor(Encoder* encoder, double wheelRadius, uint8_t pinInputA, uint8_t pinInputB);
private:
    double cmdAngVel = 0;
    Encoder* encoder;
};