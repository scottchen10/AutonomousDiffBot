#include "Encoder.hpp"

Encoder::Encoder(uint8_t pinPulseA, uint8_t pinPulseB): pinPulseA(pinPulseA), pinPulseB(pinPulseB) {
    lastUpdateTime = (double)millis() * 1e-6;
};

void Encoder::setup() {
    pinMode(pinPulseA, INPUT);
    pinMode(pinPulseB, INPUT);
}

void Encoder::update() {
    double currentTime = (double)millis() * 1e-3;

    if (abs(currentTime - lastUpdateTime) < 0.02) {
        return;
    }

    double deltaTime = currentTime - lastUpdateTime;
    angularVel = (angle - lastAngle)/(deltaTime);

    lastUpdateTime = currentTime;
    lastAngle = angle;
}

void Encoder::onInterruptUpdate() {
    int pulseA = digitalRead(pinPulseA);
    int pulseB = digitalRead(pinPulseB);
    
    if (pulseA != pulseB) {
        pulses--;
    } else {
        pulses++;
    }

    angle = pulses * TWO_PI/(MOTOR_RATIO * PULSE_PER_REV);
}
double Encoder::getDeltaAngle() {
    return angle;
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

