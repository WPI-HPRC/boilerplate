#pragma once

#include <Arduino.h>
#include <Servo.h>
#include <cstdint>
#include "boilerplate/Logging/Loggable.h"
#include "boilerplate/Utilities/PIDController.h"

struct AxonData {
    float potPos;
};

#define AXON_LOG_DESC(X)                                                            \
    X(0, "AxonPotPos", p.print(read(), 3))                           \

class AxonController : public Loggable{
  public:
    AxonController(uint8_t servo_pin, uint8_t feedback_pin, float kp, float ki, float kd, float outPWMMin, float outPWMMax, float minPotPosReading, float maxPotPosReading)
        : servo_pin(servo_pin), feedback_pin(feedback_pin), kp(kp), ki(ki), kd(kd), outPWMMin(outPWMMin), outPWMMax(outPWMMax), maxPWMRange(outPWMMax - outPWMMin),
        minPotPosReading(minPotPosReading), maxPotPosReading(maxPotPosReading), degreePerReading(maxRotationInDegrees / (maxPotPosReading - minPotPosReading)), servo(), Loggable(NUM_FIELDS(AXON_LOG_DESC)){}

    void init();

    void write(float pos, long currTime);

    int read();

  private:

    MAKE_LOGGABLE(AXON_LOG_DESC)

    uint32_t dataUpdatedAt() override { return lastTimePotRead; }
  

    float kp;
    float ki;
    float kd;
    float outPWMMin;
    float outPWMMax;
    float maxPWMRange;
    float minPotPosReading;
    float maxPotPosReading;
    float degreePerReading;
    float maxRotationInDegrees = 720.;
    uint32_t lastTimePotRead;
    uint8_t servo_pin;
    uint8_t feedback_pin;
    Servo servo;
    PIDController pid = PIDController(kp, ki, kd, outPWMMin, outPWMMax);
};