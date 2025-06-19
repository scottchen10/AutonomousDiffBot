#include "Motor.hpp"

void Motor::setup() {
    pinMode(pinInputA, OUTPUT);
    pinMode(pinInputB, OUTPUT);
}

Motor::Motor(Encoder* encoder, uint8_t pinInputA, uint8_t pinInputB)
    :encoder(encoder), pinInputA(pinInputA), pinInputB(pinInputB) 
{
    motorPid = new PIDController(0.02, 0.05, 0.00005); 
}

void Motor::setTargetAngVel(double angVel) {
    cmdAngVel = angVel;
}

void Motor::setMotorPower(Motor::MotorCommand command, uint8_t power) {
    if (command == CMD_COAST && power == 0) {
        digitalWrite(pinInputA, LOW);
        digitalWrite(pinInputB, LOW);
    } else if (command == CMD_BRAKE && power == 0) {
        digitalWrite(pinInputA, HIGH);
        digitalWrite(pinInputB, HIGH);
    } else if (command == CMD_FWD && power != 0) {
        analogWrite(pinInputA, power);
        digitalWrite(pinInputB, LOW);
    } else if (command == CMD_BWD && power != 0) {
        digitalWrite(pinInputA, LOW);
        analogWrite(pinInputB, power);
    } else { 
        digitalWrite(pinInputA, LOW);
        digitalWrite(pinInputB, LOW);
    }
}

void Motor::update() {
    double angularVel = encoder->getAngularVel();
    double error = cmdAngVel - angularVel;
    motorPid->updateError(error);
    double responseFactor = motorPid->getResponse();

    bool isAboveSafeThreshold = abs(angularVel) > TWO_PI/8;
    bool isDescelerating = (responseFactor > 0) != (angularVel > 0);
    bool isBelowMinimumPower = abs(responseFactor) < TWO_PI/1602.5;

    uint8_t power = (uint8_t) constrain(abs(responseFactor * 255.0), 0.0, 255.0);

    if ((isAboveSafeThreshold && isDescelerating) || isBelowMinimumPower) {
        this->setMotorPower(CMD_COAST, 0);
    } else if (responseFactor > 0){
        this->setMotorPower(CMD_FWD, power);
    } else if (responseFactor < 0) {
        this->setMotorPower(CMD_BWD, power);
    } 
}