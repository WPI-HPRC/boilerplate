#pragma once

// NOTE: THIS REQUIRES THE SERVOS TO BE IN CONTEXT TO WORK

#include "Context.h"
#include "CanardServoTypes.h"

void canardsSetup(Context *ctx);
void canardsLoop(Context *ctx);

void servoSetpoint(CanardServo *servo, double setpoint);
