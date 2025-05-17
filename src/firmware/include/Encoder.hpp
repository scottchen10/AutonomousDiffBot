#pragma once

#include <Arduino.h>

#define PULSE_PER_REV 11.0
#define MOTOR_RATIO 21.3

#define DEFINE_ENCODER_ISR(name, pinPulseA, pinPulseB)                         \
    Encoder* name = nullptr;                                                   \
    void isr_##name()  {                                                                         \
        if (name)                                                           \
            name->updatePulses();                                           \
    }                                                                          \
    void setup_##name() {                                                                          \
        name = new Encoder(pinPulseA, pinPulseB);                              \
        name->setup();                                                         \
        attachInterrupt(digitalPinToInterrupt(pinPulseA), isr_##name, RISING); \
    }

class Encoder {
public:
    double getAngle();
    double getAngularVel();
    void updatePulses();
    void setup();
    Encoder(uint8_t pinPulseA, uint8_t pinPulseB);

private:
    int pulses = 0;
    double angle = 0;
    double angularVel = 0;
    unsigned long lastUpdateTime = 0;
    uint8_t pinPulseA;
    uint8_t pinPulseB;

};