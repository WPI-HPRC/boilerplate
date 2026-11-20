// GNC attitude controller and canard linkage kinematics (MATH_DERIVATIONS.md).
//
// All angles are radians, rates rad/s, airspeed m/s.
// Body axes: x = pitch, y = yaw, z = roll. Canard i sits at azimuth 90 deg * i
// from +x towards +y; a positive angle on any canard rolls the rocket about +z.
// Call the LQR at 200 Hz: the gains assume a 5 ms step.

#ifndef GNC_CONTROLLER_H
#define GNC_CONTROLLER_H

namespace gnc {

constexpr float kDegToRad = 3.14159265358979f / 180.0f;

constexpr float kLqrCanardLimit = 3.0f * kDegToRad;    // LQR output failsafe
constexpr float kRollErrorLimit = 10.0f * kDegToRad;   // anti-windup (A9)
constexpr float kServoCanardLimit = 7.5f * kDegToRad;  // linkage failsafe

/**
 * LQR attitude controller: computes the four canard angle commands for one
 * 5 ms control step, c = -K z (MATH_DERIVATIONS Part A).
 *
 * The gain K is looked up from a table by airspeed. The roll error is limited
 * to +/- 10 deg before the gains are applied (anti-windup).
 *
 * Failsafe: if any command would exceed +/- 3 deg, all four are multiplied by
 * the same factor so the largest is exactly 3 deg. If any input is NaN or
 * infinite, all four commands are zero.
 *
 * @param attitude_error  [in] How far the rocket is rotated from its target
 *                        attitude, as a rotation vector Log(R_ref^T R):
 *                        [0] pitch, [1] yaw, [2] roll error [rad].
 * @param body_rate       [in] Angular velocity from the gyro, body axes:
 *                        [0] pitch, [1] yaw, [2] roll rate [rad/s].
 * @param canard_angle    [in] Where the four canards actually are right now
 *                        [rad], canards 0 to 3. Either measured, or the array
 *                        maintained by servoModelStep().
 * @param airspeed        [in] Airspeed [m/s], used only to pick the gains.
 *                        Below 30 or above 130 m/s the end gains are used.
 * @param canard_command  [out] The canard angle to command on each of the four
 *                        canards [rad], each within +/- 3 deg. Pass each one
 *                        to canardToServo() to get the servo angle.
 */
void lqrController(const float attitude_error[3], const float body_rate[3],
                   const float canard_angle[4], float airspeed,
                   float canard_command[4]);

/**
 * Servo model: predicts where the four canards will be one 5 ms step from now,
 * given what was just commanded. Use it only when the vehicle has no canard
 * position feedback.
 *
 * Models each servo as a first-order lag with a 30 ms time constant
 * (placeholder, needs bench data): each step the canard closes 15.35% of the
 * gap between its current angle and the command.
 *
 * Usage: keep one float[4] that starts at zero. Every step, pass it to
 * lqrController() as canard_angle, then call this with the command that came
 * out.
 *
 * @param canard_angle    [in, out] Modelled canard angles [rad]. Read as the
 *                        angles now, overwritten with the angles one step
 *                        later.
 * @param canard_command  [in] The commands lqrController() just produced [rad].
 */
void servoModelStep(float canard_angle[4], const float canard_command[4]);

/**
 * Dimensions and mounting of one servo-to-canard four-bar linkage
 * (MATH_DERIVATIONS Part B). Horn angles are zero when the horn is
 * perpendicular to the line between the two shafts.
 */
struct Linkage {
    float servo_horn;      // a: servo shaft to push rod pin
    float push_rod;        // b: pin to pin
    float canard_horn;     // c: canard shaft to push rod pin
    float shaft_spacing;   // d: servo shaft to canard shaft, same unit as above
    float canard_horn_neutral;  // canard horn angle at zero deflection [rad]
    float sign;  // +1 or -1: horn-positive to controller-positive canard
};

// Whiteboard dimensions in mm. TODO(GNC): confirm the dimensions, the sign of
// each canard, and which neutral pose the hardware has (MATH_DERIVATIONS B10).
// N2: servo horn perpendicular at neutral. N1: canard horn perpendicular.
constexpr Linkage kLinkageN2 = {7.3f, 44.079f, 16.51f, 45.713f,
                                -8.936254f * kDegToRad, 1.0f};
constexpr Linkage kLinkageN1 = {7.3f, 44.079f, 16.51f, 45.713f, 0.0f, 1.0f};

/**
 * Inverse kinematics: the servo angle that produces a given canard deflection
 * (MATH_DERIVATIONS B6).
 *
 * Failsafe: the canard angle is clamped to +/- 7.5 deg before the conversion.
 *
 * @param linkage  [in] Linkage dimensions, e.g. kLinkageN2.
 * @param canard   [in] Wanted canard deflection from neutral [rad].
 * @return Servo horn angle from its neutral position [rad]. The direction and
 *         scaling of the actual servo signal are up to the caller. Returns 0
 *         (neutral) if the input is NaN or infinite, or if the linkage cannot
 *         reach the angle.
 */
float canardToServo(const Linkage& linkage, float canard);

/**
 * Forward kinematics: the canard deflection that results from a given servo
 * angle. The reverse of canardToServo().
 *
 * Failsafe: the returned canard angle is clamped to +/- 7.5 deg.
 *
 * @param linkage  [in] Linkage dimensions, e.g. kLinkageN2.
 * @param servo    [in] Servo horn angle from its neutral position [rad].
 * @return Canard deflection from neutral [rad], within +/- 7.5 deg. Returns 0
 *         if the input is NaN or infinite, or the linkage cannot be assembled
 *         at that servo angle.
 */
float servoToCanard(const Linkage& linkage, float servo);

}  // namespace gnc

#endif  // GNC_CONTROLLER_H
