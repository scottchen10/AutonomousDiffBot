#include "Motor.hpp"

void Motor::setup() {
    pinMode(pinInputA, OUTPUT);
    pinMode(pinInputB, OUTPUT);
}

Motor::Motor(Encoder* encoder, double wheelRadius, uint8_t pinInputA, uint8_t pinInputB)
    :encoder(encoder), wheelRadius(wheelRadius), pinInputA(pinInputA), pinInputB(pinInputB) 
{
    motorPid = new PIDController(0.15, 0.12, 0.0005); 
}

void Motor::setTargetAngVel(double angVel) {
    cmdAngVel = angVel;
}

void Motor::setMotorPower(int16_t power) {
    bool isFWD = power > 0 ? true: false;

    if (power == 0) {
        digitalWrite(pinInputA, LOW);
        digitalWrite(pinInputB, LOW);
    } else if (isFWD) {
        analogWrite(pinInputA, (uint8_t) abs(power));
        digitalWrite(pinInputB, LOW);
    } else {
        digitalWrite(pinInputA, LOW);
        analogWrite(pinInputB, (uint8_t) abs(power));
    }
}


void Motor::setMotorPower(int16_t power, bool brake) {
    if (brake) {
        digitalWrite(pinInputA, HIGH);
        digitalWrite(pinInputB, HIGH);
    } else {
        this->setMotorPower(power);
    }
}

void Motor::update() {
    double angularVel = encoder->getAngularVel();
    double error = cmdAngVel - angularVel;
    motorPid->updateError(error);
    double responseFactor = motorPid->getResponse();

    bool isAboveSafeThreshold = abs(angularVel) > TWO_PI/8;
    bool isDescelerating = (responseFactor > 0) != (angularVel > 0);

    if (isAboveSafeThreshold && isDescelerating) {
        this->setMotorPower(0);
    } else {
        this->setMotorPower(responseFactor * 255);
    }
}