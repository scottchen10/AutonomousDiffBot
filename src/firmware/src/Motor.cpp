#include "Motor.hpp"

void Motor::setup() {
    pinMode(pinInputA, OUTPUT);
    pinMode(pinInputB, OUTPUT);
}

Motor::Motor(Encoder* encoder, double wheelRadius, uint8_t pinInputA, uint8_t pinInputB)
    :encoder(encoder), wheelRadius(wheelRadius), pinInputA(pinInputA), pinInputB(pinInputB) {
        this->setup();
    }

void Motor::setAngVel(double angVel) {
    cmdAngVel = angVel;
}

void Motor::setMotorSpeed(int16_t power) {
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

void Motor::update() {

}