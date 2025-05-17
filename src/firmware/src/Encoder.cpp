#include "Encoder.hpp"

Encoder::Encoder(uint8_t pinPulseA, uint8_t pinPulseB): pinPulseA(pinPulseA), pinPulseB(pinPulseB) {};

void Encoder::setup() {
    pinMode(pinPulseA, INPUT);
    pinMode(pinPulseB, INPUT);
}

void Encoder::updatePulses() {
    int pulseA = digitalRead(pinPulseA);
    int pulseB = digitalRead(pinPulseB);

    double lastAngle = angle;
    unsigned long currTime = micros();
    unsigned long deltaTime = currTime - lastUpdateTime;
    

    if (pulseA != pulseB) {
        pulses--;
    } else {
        pulses++;
    }

    angle = pulses * PI/(MOTOR_RATIO * PULSE_PER_REV);
    angularVel = (angle - lastAngle)/(deltaTime * 10^6);

    lastUpdateTime = currTime;
}

double Encoder::getAngle() {
    double normalizedAngle = angle;

    normalizedAngle += PI;
    normalizedAngle -= (int)(normalizedAngle / TWO_PI) * TWO_PI;  
    normalizedAngle -= PI;
    return normalizedAngle;
}

double Encoder::getAngularVel() {
    return angularVel;
}

