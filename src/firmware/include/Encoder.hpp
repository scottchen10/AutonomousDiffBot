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
    volatile int pulses = 0;
    volatile double angle = 0;
    volatile double angularVel = 0;
    volatile double lastAngle = 0;
    volatile double lastUpdateTime = 0;

};