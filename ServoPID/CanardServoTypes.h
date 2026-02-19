#pragma once

#include <Servo.h>
#include <stdint.h>

// NOTE: THIS REQUIRES THE SERVOS TO BE IN CONTEXT TO WORK

struct Encoder {
    volatile uint32_t riseTime;
    volatile uint32_t highTime;
    volatile uint32_t period;
    volatile bool newSample;

    uint8_t pin;
    double lastAngle;
};

struct PID {
    double Kp, Ki, Kd;  
    double integral;
    double previousError;
};

struct CanardServo {
    Servo servo;
    PID pid;
    Encoder *encoder;
    double setpoint;
    double neutralPosition;
};
