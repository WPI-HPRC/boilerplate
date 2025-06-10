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
 
         // integrate with anti-windup
         integral += error * dt;
         float integTerm = ki * integral;
         if (integTerm > outMax) integral = outMax / ki;
         else if (integTerm < outMin) integral = outMin / ki;
 
         // derivative
         float derivative = (error - lastError) / dt;
 
         // PID formula
         float output = kp * error
                      + ki * integral
                      + kd * derivative;
 
         // clamp
         if (output > outMax) output = outMax;
         else if (output < outMin) output = outMin;
 
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
 