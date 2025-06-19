#pragma once

#include <Arduino.h>
#include "Encoder.hpp"
#include "PIDControl.hpp"

class Motor {
public:
    enum MotorCommand {
        CMD_FWD,
        CMD_BWD,
        CMD_BRAKE,
        CMD_COAST
    };

    void setTargetAngVel(double angVel);
    void setup();
    void update();

    void setMotorPower(Motor::MotorCommand command, uint8_t power);

    PIDController* motorPid; 
    uint8_t pinInputA;
    uint8_t pinInputB;

    Motor(Encoder* encoder, uint8_t pinInputA, uint8_t pinInputB);

private:
    double cmdAngVel = 0;
    Encoder* encoder = nullptr;
};