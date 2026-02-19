#include <Context.h>
#include <Servo.h>
#include "config.h"
#include <stdint.h>
#include "CanardsServo.h"

// NOTE: THIS REQUIRES THE SERVOS TO BE IN CONTEXT TO WORK

static Encoder encoder1;
static Encoder encoder2;
static Encoder encoder3;
static Encoder encoder4;

void updateEncoder(Encoder *enc) {
    if(!enc) return; 

    uint32_t now = micros();
    bool level = digitalRead(enc->pin);

    if (level) {
        enc->period = now - enc->riseTime;
        enc->riseTime = now;
    } else {
        enc->highTime = now - enc->riseTime;
        enc->newSample = true;
    }
}

void encoder1ISR() { updateEncoder(&encoder1); }
void encoder2ISR() { updateEncoder(&encoder2); }
void encoder3ISR() { updateEncoder(&encoder3); }
void encoder4ISR() { updateEncoder(&encoder4); }

void canardsSetup(Context *ctx) {
    ctx->canard1.servo.attach(CANARD_1_PIN);
    ctx->canard1.setpoint = ctx->canard1.neutralPosition;
    pinMode(CANARD_1_FEEDBACK_PIN, INPUT);
    
    encoder1.pin = CANARD_1_FEEDBACK_PIN;
    encoder1.riseTime = micros();
    encoder1.period = 0;
    encoder1.highTime = 0;
    encoder1.newSample = false;
    ctx->canard1.encoder = &encoder1;
    attachInterrupt(digitalPinToInterrupt(CANARD_1_FEEDBACK_PIN), encoder1ISR, CHANGE);

    ctx->canard2.servo.attach(CANARD_2_PIN);
    ctx->canard2.setpoint = ctx->canard2.neutralPosition;
    pinMode(CANARD_2_FEEDBACK_PIN, INPUT);

    encoder2.pin = CANARD_2_FEEDBACK_PIN;
    encoder2.riseTime = micros();
    encoder2.period = 0;
    encoder2.highTime = 0;
    encoder2.newSample = false;
    ctx->canard2.encoder = &encoder2;
    attachInterrupt(digitalPinToInterrupt(CANARD_2_FEEDBACK_PIN), encoder2ISR, CHANGE);

    ctx->canard3.servo.attach(CANARD_3_PIN);
    ctx->canard3.setpoint = ctx->canard3.neutralPosition;
    ctx->canard3.encoder = &encoder3;
    pinMode(CANARD_3_FEEDBACK_PIN, INPUT);

    encoder3.pin = CANARD_3_FEEDBACK_PIN;
    encoder3.riseTime = micros();
    encoder3.period = 0;
    encoder3.highTime = 0;
    encoder3.newSample = false;
    ctx->canard3.encoder = &encoder3;
    attachInterrupt(digitalPinToInterrupt(CANARD_3_FEEDBACK_PIN), encoder3ISR, CHANGE);


    ctx->canard4.servo.attach(CANARD_4_PIN);
    ctx->canard4.setpoint = ctx->canard4.neutralPosition;
    pinMode(CANARD_4_FEEDBACK_PIN, INPUT);

    encoder4.pin = CANARD_4_FEEDBACK_PIN;
    encoder4.riseTime = micros();
    encoder4.period = 0;
    encoder4.highTime = 0;
    encoder4.newSample = false;
    ctx->canard4.encoder = &encoder4;
    attachInterrupt(digitalPinToInterrupt(CANARD_4_FEEDBACK_PIN), encoder4ISR, CHANGE);

}

double pidUpdate(PID *pid, double error, double dt) {
    pid->integral += error * dt;
    double derivative = (error - pid->previousError) / dt;
    pid->previousError = error;

    return (pid->Kp * error) +
           (pid->Ki * pid->integral) +
           (pid->Kd * derivative);
}

double getEncoder(Encoder * enc) {
    uint32_t high, period;
    bool sample;

    noInterrupts();
    high = enc->highTime;
    period = enc->period;
    sample = enc->newSample;
    enc->newSample = false;
    interrupts();

    if(!sample || period == 0) {
        return enc->lastAngle;
    }

    double duty = (double)high / (double)period;
    enc->lastAngle = duty * 180.0; // Assuming 0-180 degree range
    return enc->lastAngle;
}

void servoLoop(CanardServo *canard) { 
    static uint32_t lastTime = 0;
    uint32_t now = millis();
    double dt = (now - lastTime) * 0.001;

    double angle = getEncoder(canard->encoder);
    double error = canard->setpoint - angle;

    double correction = pidUpdate(&canard->pid, error, dt);

    double commandAngle = canard->neutralPosition + correction;

    commandAngle = constrain(commandAngle, SERVO_MIN, SERVO_MAX);
    canard->servo.write(commandAngle);
}

void canardsLoop(Context *ctx) {
    servoLoop(&ctx->canard1);
    servoLoop(&ctx->canard2);
    servoLoop(&ctx->canard3);
    servoLoop(&ctx->canard4);
}

void servo1Setpoint(CanardServo *servo, double setpoint) {
    servo->setpoint = setpoint;
}
