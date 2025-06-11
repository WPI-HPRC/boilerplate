#include "AxonController.h"

void AxonController::init() {
  servo.attach(servo_pin);
  pinMode(feedback_pin, INPUT);
}

/**
 * @brief write a position to the axon to go to
 */
void AxonController::write(float pos, long currTime) {
  float effort = pid.compute(pos, read(), currTime);
  servo.writeMicroseconds(effort);
}

int AxonController::read() {
  float currDegrees = minPotPosReading + degreePerReading * analogRead(feedback_pin);
  lastTimePotRead = ::millis();
  return currDegrees;
}