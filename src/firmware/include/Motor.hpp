#pragma once

#include <Arduino.h>
#include <Encoder.hpp>


class Motor {
public:
    void setAngVel(double angVel);
    void setup();
    void update();

    void setMotorSpeed(int16_t power);
    Motor(Encoder* encoder, double wheelRadius, uint8_t pinInputA, uint8_t pinInputB);

private:
    double wheelRadius;
    double cmdAngVel;
    Encoder* encoder;

    uint8_t pinInputA;
    uint8_t pinInputB;

};