#pragma once

#include <Arduino.h>

#define PULSE_PER_REV 11.0
#define MOTOR_RATIO 21.3

class Encoder {
public:
    double getAngle();
    double getDeltaAngle();
    double getAngularVel();
    void update();
    void onInterruptUpdate();
    void setup();
    Encoder(uint8_t pinPulseA, uint8_t pinPulseB);
    uint8_t pinPulseA;
    uint8_t pinPulseB;

    private:
    int pulses = 0;
    double angle = 0;
    double angularVel = 0;
    double lastAngle = 0;
    double lastUpdateTime = 0;

};