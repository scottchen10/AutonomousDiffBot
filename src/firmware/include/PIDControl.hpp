#pragma once
#include <Arduino.h>

class PIDController {
public:
    volatile double error = 0;
    volatile double derivativeError = 0;
    volatile double integralError = 0;
    volatile double lastUpdateTime = 0;

    void updateError(double error) {
        double currentTime = (double)micros() * 1e6;
        double deltaTime = currentTime - lastUpdateTime;

        integralError += deltaTime * error;
        derivativeError = (error - this->error)/deltaTime;
        this->error = error;

        lastUpdateTime = currentTime;        
    };
    double getResponse() {
        return constrain((kp * error + ki * integralError + kd * derivativeError), -1.0, 1.0);
    };
    PIDController(double kp, double ki, double kd): kp(kp), ki(ki), kd(kd) {
        lastUpdateTime = (double)micros() * 1e-6;
    }

private:
    double kp;
    double ki;
    double kd;
};