/**
 * @brief Simple PID loop for velocity control.
 */
class PIDController {
public:
    /**
     * @brief Construct a new PIDController.
     * @param kp  Proportional gain.
     * @param ki  Integral gain.
     * @param kd  Derivative gain.
     * @param outMin  Minimum output (e.g. –maxSpeed).
     * @param outMax  Maximum output (e.g. +maxSpeed).
     */
    explicit PIDController(float kp, float ki, float kd, float outMin, float outMax)
      : kp(kp), ki(ki), kd(kd), lastTime(0),
        outMin(outMin), outMax(outMax),
        integral(0), lastError(0)
    {}

    /**
     * @brief Compute the next control action.
     * @param setpoint  Target velocity (units/sec).
     * @param measured  Measured velocity (same units).
     * @return float  Control output, clamped to [outMin, outMax].
     */
    float compute(float setpoint, float measured, long currTime) {
        float error = setpoint - measured;
        float dt = currTime - lastTime;

#ifdef DEBUG
        Serial.print(">pid_error:"); Serial.println(error);
        Serial.print(">pid_dt:"); Serial.println(dt);
#endif

        // integrate with anti-windup
        integral += error * dt;
        float integTerm = ki * integral;
        if (integTerm > outMax) integral = outMax / ki;
        else if (integTerm < outMin) integral = outMin / ki;

#ifdef DEBUG
        Serial.print(">pid_integral:"); Serial.println(integral);
        Serial.print(">pid_integral_term:"); Serial.println(integTerm);
#endif

        // derivative
        float derivative = (error - lastError) / dt;

#ifdef DEBUG
        Serial.print(">pid_derivative:"); Serial.println(derivative);
#endif

        // PID formula
        float output = kp * error
                     + ki * integral
                     + kd * derivative;

#ifdef DEBUG
        Serial.print(">pid_p_term:"); Serial.println(kp * error);
        Serial.print(">pid_i_term:"); Serial.println(ki * integral);
        Serial.print(">pid_d_term:"); Serial.println(kd * derivative);
        Serial.print(">pid_raw_output:"); Serial.println(output);
#endif

        // clamp
        output += outMin; //makes the PID act between the minimum PWM vel write and the max; necceassry fix since we are trying to write position but output our effort as velocity

        if (output > outMax) output = outMax;
        else if (output < outMin) output = outMin;

#ifdef DEBUG
        Serial.print(">pid_final_output:"); Serial.println(output);
        Serial.print(">pid_setpoint:"); Serial.println(setpoint);
        Serial.print(">pid_measured:"); Serial.println(measured);
#endif

        currTime = lastTime;
        lastError = error;
        return output;
    }

    /**
     * @brief Reset integral and derivative state.
     */
    void reset() {
        integral = 0;
        lastError = 0;
    }

private:
    float kp, ki, kd;
    long lastTime;
    float outMin, outMax;
    float integral, lastError;
};
 